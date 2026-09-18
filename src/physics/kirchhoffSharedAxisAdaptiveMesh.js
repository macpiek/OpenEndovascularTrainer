import {kirchhoffMaterialProfile} from './kirchhoffMaterialProfile.js';
import {guidewireMaterialProfile} from './guidewireMaterialProfile.js';

// Experimental spatial approximation budget, separate from solver residuals.
// Contact discovery still tests the entire resulting capsule, not just nodes.
export const DEFAULT_ADAPTIVE_MESH = Object.freeze({
    maxSpacing: 20, shapeTolerance: .15, maxArcLoss: .002,
    maxTurn: Math.PI / 15, contactMargin: 1, tipLength: 30
});
export const ADAPTIVE_SOLVE_OPTIONS = Object.freeze({forceTolerance: 1e-4, lengthTolerance: 1e-3});

export function adaptiveMeshOptions(value) {
    if (!value) return null;
    const options = {...DEFAULT_ADAPTIVE_MESH, ...(value === true ? {} : value)};
    if (!Object.entries(options).every(([key,v]) => Number.isFinite(v) && (key === 'contactMargin' || key === 'maxArcLoss' ? v >= 0 : v > 0)) || options.maxTurn >= Math.PI || options.maxArcLoss >= 1)
        throw new RangeError('Invalid adaptive mesh budget');
    return options;
}

function tipLength(tool, options) {
    const type = tool.type ?? (tool.id === 'wire' ? 'glidewire' : 'berenstein');
    const profile = kirchhoffMaterialProfile(type);
    let length = profile.naturalTipLengthMm + 10;
    if (profile.id === 'glidewire' || profile.id === 'steel-j-035') {
        const wire = guidewireMaterialProfile(profile.id);
        length = Math.max(length, wire.tipCoreLength + wire.tipTransitionLength);
    }
    return Math.max(options.tipLength, length);
}

/** Identify neighbourhoods needing fine mechanics around loaded/near contacts.
 * Old moving-tip endpoints are not pinned: loaded physical sites are remapped
 * separately, without leaving a permanent trail of mechanical nodes. */
export function adaptiveContactKnots(previous, options) {
    const knots = new Set();
    if (!previous) return knots;
    previous.definitions.forEach((row, i) => {
        if (row.kind !== 'wall' || row.subtype === 'bend-limit') return;
        const gap = previous.acceptedWallGaps?.get(row.id);
        if (!(previous.multipliers[i] > 1e-10 || gap <= options.contactMargin)) return;
        for (let n = Math.max(0, row.edge - 1); n <= Math.min(previous.coordinates.length - 1, row.edge + 2); n++)
            knots.add(previous.coordinates[n]);
    });
    return knots;
}

export function retainAdaptiveShapeSamples(coordinates, positions, previous, options) {
    if (!previous) return;
    let edge = 0;
    for (let n = 0; n < previous.coordinates.length; n++) {
        const x = previous.coordinates[n];
        if (x <= coordinates[0] || x >= coordinates.at(-1)) continue;
        while (edge + 1 < coordinates.length - 1 && coordinates[edge + 1] < x) edge++;
        const t = (x - coordinates[edge]) / (coordinates[edge + 1] - coordinates[edge]);
        const p = previous.positions[n];
        const error = Math.hypot(...p.map((v,k) => v - ((1-t)*positions[edge][k]+t*positions[edge+1][k])));
        if (error > options.shapeTolerance && t > 1e-9 && t < 1-1e-9) {
            coordinates.splice(edge+1,0,x); positions.splice(edge+1,0,p.slice()); edge++;
        }
    }
}

/** Coarsen a fine candidate using its accepted physical shape. Every omitted
 * sample is checked against the replacement chord. Arc loss limits artificial
 * shortening; the turn bound protects curvature even at inflection points.
 * No accepted state is mutated. Fine knots can return on the following feed.
 */
export function coarsenSharedAxisMesh(coordinates, positions, {tools, boundaries, previous, spacing, options}) {
    if (options.maxSpacing < spacing) throw new RangeError('Adaptive max spacing must cover the fine spacing');
    const contacts = adaptiveContactKnots(previous, options);
    const contactSpans = previous ? previous.coordinates.slice(0,-1).flatMap((x,i) =>
        contacts.has(x)&&contacts.has(previous.coordinates[i+1]) ? [[x,previous.coordinates[i+1]]] : []) : [];
    const tips = tools.map(tool => ({end: tool.insertion, start: tool.insertion - tipLength(tool, options)}));
    const protectedNode = coordinates.map((x, i) => i < 2 || i === coordinates.length - 1 ||
        boundaries.includes(x) || contacts.has(x) || contactSpans.some(([a,b])=>x>=a&&x<=b) || tips.some(t => x >= t.start && x <= t.end) ||
        boundaries.some(b => Math.abs(b - x) <= spacing));
    const distance = (a, b) => Math.hypot(...a.map((v, k) => v - b[k]));
    const canMerge = (first, last) => {
        if (coordinates[last] - coordinates[first] > options.maxSpacing + 1e-9) return false;
        for (let i = first + 1; i < last; i++) if (protectedNode[i]) return false;
        const a = positions[first], b = positions[last], chord = distance(a, b);
        if (chord < 1e-9) return false;
        if (previous) for (let i = 0; i < previous.coordinates.length; i++) {
            const x = previous.coordinates[i];
            if (x <= coordinates[first] || x >= coordinates[last]) continue;
            const t = (x-coordinates[first])/(coordinates[last]-coordinates[first]);
            if (distance(previous.positions[i],a.map((v,k)=>v+t*(b[k]-v)))>options.shapeTolerance) return false;
        }
        let arc = 0, turn = 0, prior = null;
        for (let i = first; i < last; i++) {
            const delta = positions[i + 1].map((v, k) => v - positions[i][k]), length = Math.hypot(...delta);
            if (length < 1e-9) return false;
            const direction = delta.map(v => v / length);
            if (prior) turn += Math.acos(Math.max(-1, Math.min(1, direction.reduce((sum, v, k) => sum + v * prior[k], 0))));
            prior = direction; arc += length;
            if (i > first) {
                const t = (coordinates[i] - coordinates[first]) / (coordinates[last] - coordinates[first]);
                if (distance(positions[i], a.map((v, k) => v + t * (b[k] - v))) > options.shapeTolerance) return false;
            }
        }
        return turn <= options.maxTurn && arc - chord <= options.maxArcLoss * arc;
    };
    const keep = [0];
    while (keep.at(-1) < coordinates.length - 1) {
        const first = keep.at(-1);
        let last = first + 1;
        // Stop at the first failed budget or protected node. This conservative
        // choice prevents distant endpoints hiding a local fold.
        while (last + 1 < coordinates.length && canMerge(first, last + 1)) last++;
        keep.push(last);
    }
    const xs = keep.map(i => coordinates[i]), ps = keep.map(i => positions[i]);
    coordinates.splice(0, coordinates.length, ...xs);
    positions.splice(0, positions.length, ...ps);
}
