import * as THREE from 'three';
import {sampleFlowConcentration} from './flowConcentrationField.js';

function makeTexture(sampleCount) {
    const width = Math.max(1, Math.min(1024, sampleCount));
    const height = Math.max(1, Math.ceil(sampleCount / width));
    const data = new Float32Array(width * height * 4);
    const texture = new THREE.DataTexture(data, width, height, THREE.RGBAFormat, THREE.FloatType);
    texture.minFilter = texture.magFilter = THREE.NearestFilter;
    texture.generateMipmaps = false;
    texture.needsUpdate = true;
    return {texture, data};
}

/**
 * Antiderivatives of the piecewise-linear blood concentration field.
 * A territory is a single continuous graph chain, not all branches supplied
 * by the same organ. Separate territories can never borrow each other's iodine.
 * Texture records are [arc length, concentration, cumulative integral, 0].
 * The two histories have identical sample coordinates and layouts.
 */
export function createFlowColumnIntegral(network, field, territories, continuations) {
    const edgeStart = new Float64Array(network.edges.length).fill(NaN);
    const groups = new Map(), ranges = new Map(), ordered = [];
    for (const edge of network.edges) {
        const territory = territories[edge.index];
        if (territory < 0 || edge.transportExcluded || edge.lumenMappingExcluded) continue;
        if (!groups.has(territory)) groups.set(territory, []);
        groups.get(territory).push(edge.index);
    }
    let sampleCount = 0;
    for (const [territory, members] of groups) {
        const memberSet = new Set(members);
        const roots = members.filter(index => {
            const parent = network.edges[index].parentEdgeIndex;
            return !memberSet.has(parent) || continuations[parent] !== index;
        });
        if (roots.length !== 1) throw new RangeError(`Concentration territory ${territory} must be one continuous chain`);
        const start = sampleCount, visited = new Set();
        let length = 0;
        for (let index = roots[0]; memberSet.has(index); index = continuations[index]) {
            if (visited.has(index)) throw new RangeError(`Concentration territory ${territory} contains a cycle`);
            visited.add(index);
            const edge = network.edges[index];
            edgeStart[index] = length;
            ordered.push(index);
            length += edge.length;
            sampleCount += edge.cellCount + 2;
        }
        if (visited.size !== members.length) throw new RangeError(`Concentration territory ${territory} contains disconnected edges`);
        ranges.set(territory, {start, count: sampleCount - start, length});
    }

    const coordinates = new Float64Array(sampleCount);
    const lower = new Uint32Array(sampleCount), upper = new Uint32Array(sampleCount);
    const fraction = new Float64Array(sampleCount), owner = new Int32Array(sampleCount);
    const currentSamples = new Float64Array(sampleCount), previousSamples = new Float64Array(sampleCount);
    const currentPrefix = new Float64Array(sampleCount), previousPrefix = new Float64Array(sampleCount);
    const current = makeTexture(sampleCount), previous = makeTexture(sampleCount);
    function continuationStencil(parentIndex, childIndex) {
        const parent = network.edges[parentIndex], child = network.edges[childIndex];
        if (!parent || !child || continuations[parentIndex] !== childIndex ||
            territories[parentIndex] !== territories[childIndex] ||
            !Number.isFinite(edgeStart[parentIndex]) || !Number.isFinite(edgeStart[childIndex])) return null;
        // A single path needs one trace on both sides of a graph seam. The
        // global junction sample mixes every arm, and its per-arm owner gates
        // can otherwise produce different values at this same coordinate.
        // Interpolate only the two continuation cell centres in physical
        // distance, retaining their measured values and their min/max range.
        const parentHalfCell = parent.length / parent.cellCount * .5;
        const childHalfCell = child.length / child.cellCount * .5;
        return [field.offsets[parentIndex] + parent.cellCount - 1, field.offsets[childIndex],
            parentHalfCell / (parentHalfCell + childHalfCell), -1];
    }
    let sample = 0;
    for (const edgeIndex of ordered) {
        const edge = network.edges[edgeIndex];
        const startTrace = continuationStencil(edge.parentEdgeIndex, edgeIndex);
        const endTrace = continuationStencil(edgeIndex, continuations[edgeIndex]);
        for (let i = -1; i <= edge.cellCount; i++) {
            const t = i < 0 ? 0 : i === edge.cellCount ? 1 : (i + .5) / edge.cellCount;
            coordinates[sample] = edgeStart[edgeIndex] + edge.length * t;
            const trace = i < 0 ? startTrace : i === edge.cellCount ? endTrace : null;
            [lower[sample], upper[sample], fraction[sample], owner[sample]] = trace || field.stencil(edgeIndex, t);
            current.data[sample * 4] = previous.data[sample * 4] = coordinates[sample];
            sample++;
        }
    }

    function updateHistory(values, samples, prefix, data) {
        for (const range of ranges.values()) {
            let accumulated = 0, previousX = 0, previousC = 0;
            const end = range.start + range.count;
            for (let i = range.start; i < end; i++) {
                const c = Math.max(0, sampleFlowConcentration(values, lower[i], upper[i], fraction[i], owner[i]));
                const x = coordinates[i];
                // Duplicate edge endpoints share a continuation trace and
                // contribute neither artificial distance nor extra iodine.
                if (i > range.start) accumulated += .5 * (previousC + c) * (x - previousX);
                samples[i] = c;
                prefix[i] = accumulated;
                data[i * 4 + 1] = c;
                data[i * 4 + 2] = accumulated;
                previousX = x; previousC = c;
            }
        }
    }

    const clamp = (x, range) => Math.max(0, Math.min(range.length, x));
    // Return the right-continuous sample at a duplicate-coordinate seam.
    function lowerSample(range, x) {
        let low = range.start, high = range.start + range.count;
        while (low < high) {
            const middle = (low + high) >>> 1;
            if (coordinates[middle] <= x) low = middle + 1;
            else high = middle;
        }
        return Math.max(range.start, low - 1);
    }
    function valueAt(range, x, samples) {
        const i = lowerSample(range, x), end = range.start + range.count - 1;
        if (i >= end) return samples[end];
        const span = coordinates[i + 1] - coordinates[i];
        return samples[i] + (samples[i + 1] - samples[i]) * (x - coordinates[i]) / span;
    }
    function integralBetween(range, a, b, samples, prefix) {
        if (a === b) return 0;
        if (a > b) return -integralBetween(range, b, a, samples, prefix);
        const first = lowerSample(range, a), last = lowerSample(range, b);
        const ca = valueAt(range, a, samples), cb = valueAt(range, b, samples);
        // Avoid cancellation of two large prefix values for the near-zero
        // axial spans encountered when a ray crosses a vessel side-on.
        if (first === last) return .5 * (ca + cb) * (b - a);
        const left = .5 * (ca + samples[first + 1]) * (coordinates[first + 1] - a);
        const middle = prefix[last] - prefix[first + 1];
        const right = .5 * (samples[last] + cb) * (b - coordinates[last]);
        return left + middle + right;
    }

    return {
        edgeStart, ranges, coordinates, sampleCount,
        currentTexture: current.texture, previousTexture: previous.texture,
        currentData: current.data, previousData: previous.data,
        currentSamples, previousSamples, currentPrefix, previousPrefix,
        update(values, previousValues = values) {
            updateHistory(values, currentSamples, currentPrefix, current.data);
            updateHistory(previousValues, previousSamples, previousPrefix, previous.data);
            current.texture.needsUpdate = previous.texture.needsUpdate = true;
        },
        sample(territory, x, usePrevious = false) {
            const range = ranges.get(territory);
            return range ? valueAt(range, clamp(x, range), usePrevious ? previousSamples : currentSamples) : 0;
        },
        integral(territory, a, b, usePrevious = false) {
            const range = ranges.get(territory);
            return range ? integralBetween(range, clamp(a, range), clamp(b, range),
                usePrevious ? previousSamples : currentSamples, usePrevious ? previousPrefix : currentPrefix) : 0;
        },
        average(territory, a, b, usePrevious = false) {
            const range = ranges.get(territory);
            if (!range) return 0;
            a = clamp(a, range); b = clamp(b, range);
            const samples = usePrevious ? previousSamples : currentSamples;
            if (a === b) return valueAt(range, a, samples);
            return integralBetween(range, a, b, samples, usePrevious ? previousPrefix : currentPrefix) / (b - a);
        },
        memoryBytes: coordinates.byteLength + lower.byteLength + upper.byteLength + fraction.byteLength + owner.byteLength +
            currentSamples.byteLength + previousSamples.byteLength + currentPrefix.byteLength + previousPrefix.byteLength +
            current.data.byteLength + previous.data.byteLength + edgeStart.byteLength,
        dispose() {current.texture.dispose(); previous.texture.dispose();}
    };
}
