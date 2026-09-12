/**
 * Exact full-DOF basis change for assembleKirchhoffCoupledSystem's columns.
 * All xyz columns are world translations; angular columns stay LOCAL and separate.
 * No strain/contact is approximated, removed or resampled here.
 * Buffers are borrowed, stable after capacity warmup, and overwritten on assembly.
 */

const INNER = 0;
const OUTER = 1;
const COMMON = 2;
const RELATIVE = 3;

function capacity(required, current = 0) {
    let result = Math.max(16, current);
    while (result < required) result *= 2;
    return result;
}

function finite(value, name) {
    if (!Number.isFinite(value)) throw new TypeError(`${name} must be finite`);
    return value;
}

function integer(value, name, minimum = 0) {
    if (!Number.isInteger(value) || value < minimum) throw new RangeError(`${name} must be an integer >= ${minimum}`);
    return value;
}

function growNodes(w, side, required) {
    if (w.nodeCapacity[side] >= required) return;
    const size = capacity(required, w.nodeCapacity[side]);
    w.arc[side] = new Float64Array(size);
    w.partner[side] = new Int32Array(size);
    if (side === 0) w.omittedAxes = new Uint8Array(size);
    w.nodeCapacity[side] = size;
    w.stats.bufferGrowths++;
    w._paired = false;
}

function growColumns(w, required) {
    if (w.columnCapacity >= required) return;
    const size = capacity(required, w.columnCapacity);
    w.columnOffsets = new Int32Array(size + 1);
    w.kind = new Uint8Array(size);
    w.innerDof = new Int32Array(size);
    w.outerDof = new Int32Array(size);
    w.decodeInner = new Float64Array(size);
    w.decodeOuter = new Float64Array(size);
    w.encodeInner = new Float64Array(size);
    w.encodeOuter = new Float64Array(size);
    w.weights = new Float64Array(size);
    w.generalized = new Float64Array(size);
    w.columnCapacity = size;
    w.stats.bufferGrowths++;
}

function growEntries(w, required) {
    if (w.entryCapacity >= required) return;
    const size = capacity(required, w.entryCapacity);
    w.rowIndices = new Int32Array(size);
    w.values = new Float64Array(size);
    w.entryCapacity = size;
    w.stats.bufferGrowths++;
}

/** Optional capacities reserve memory; they never cap or reduce physical DOFs. */
export function createKirchhoffBundleRuntime({ nodeCapacity = 0, columnCapacity = 0, entryCapacity = 0 } = {}) {
    integer(nodeCapacity, 'nodeCapacity');
    integer(columnCapacity, 'columnCapacity');
    integer(entryCapacity, 'entryCapacity');
    const w = {
        nodeCapacity: [0, 0], nodeCount: [0, 0], arc: [null, null], partner: [null, null],
        start: [0, 0], end: [-1, -1], bodies: [null, null],
        columnCapacity: 0, entryCapacity: 0, columnCount: 0, entryCount: 0,
        rowCount: 0, band: 1, pairCount: 0, omittedFixedPairDofs: 0,
        stats: { assemblies: 0, pairingBuilds: 0, coordinateScans: 0, bufferGrowths: 0 },
        _paired: false, _arcs: [null, null], _offsets: [NaN, NaN],
        _revision: undefined, _input: null, _valid: false
    };
    growNodes(w, 0, Math.max(1, nodeCapacity));
    growNodes(w, 1, Math.max(1, nodeCapacity));
    growColumns(w, Math.max(1, columnCapacity));
    growEntries(w, Math.max(1, entryCapacity));
    return w;
}

function weightAt(w, side, dof) {
    const material = w._input.material[side];
    if (!material || dof < w.start[side] * 6 || dof >= (w.end[side] + 1) * 6) return 0;
    const value = material.weight[dof];
    if (!Number.isFinite(value) || value < 0) throw new RangeError('Inverse masses/inertias must be finite and nonnegative');
    return value;
}

function updatePairing(w, system, options) {
    let changed = !w._paired;
    let sameMetadata = w._paired;
    for (let side = 0; side < 2; side++) {
        const body = system.bodies[side];
        const count = integer(body.count, 'body.count');
        const material = system.material[side];
        const start = material ? integer(material.start, 'material.start') : 0;
        const end = material ? integer(material.end, 'material.end') : -1;
        if (material && (end < start || end >= count || material.weight.length < count * 6)) {
            throw new RangeError('Invalid material active range or weight buffer');
        }
        const external = options.axialCoordinates?.[side] ?? null;
        const offset = finite(options.axialOffsets?.[side] ?? 0, 'axial offset');
        if (external && external.length < count) throw new RangeError('Axial coordinate buffer is too small');
        growNodes(w, side, Math.max(1, count));
        if (w.nodeCount[side] !== count || w.start[side] !== start || w.end[side] !== end
            || w.bodies[side] !== body) changed = true;
        if (changed || w._arcs[side] !== external || w._offsets[side] !== offset) sameMetadata = false;
        w.nodeCount[side] = count; w.start[side] = start; w.end[side] = end;
        w.bodies[side] = body; w._arcs[side] = external; w._offsets[side] = offset;
    }
    // The revision is an explicit caller promise covering rest lengths, active
    // ranges and axial alignment. Without it, in-place changes are always scanned.
    if (sameMetadata && options.pairingRevision !== undefined && options.pairingRevision === w._revision) return;
    for (let side = 0; side < 2; side++) {
        const arc = w.arc[side], source = w._arcs[side], offset = w._offsets[side];
        let local = 0;
        for (let node = w.start[side]; node <= w.end[side]; node++) {
            if (node > w.start[side] && !source) {
                const length = finite(w.bodies[side].restLength[node - 1], 'rest length');
                if (!(length > 0)) throw new RangeError('Rest lengths must be positive');
                local += length;
            }
            const value = finite((source ? source[node] : local) + offset, 'axial coordinate');
            if (node > w.start[side] && !(value > arc[node - 1])) {
                throw new RangeError('Active axial coordinates must be strictly increasing');
            }
            if (arc[node] !== value) changed = true;
            arc[node] = value;
        }
    }
    w.stats.coordinateScans++;
    w._revision = options.pairingRevision;
    if (!changed) return;
    w.partner[0].fill(-1, 0, w.nodeCount[0]);
    w.partner[1].fill(-1, 0, w.nodeCount[1]);
    w.pairCount = 0;
    const x = w.arc[0], y = w.arc[1];
    if (w.end[0] >= w.start[0] && w.end[1] >= w.start[1]) {
        const lo = Math.max(x[w.start[0]], y[w.start[1]]);
        const hi = Math.min(x[w.end[0]], y[w.end[1]]);
        let a0 = w.start[0], b0 = w.end[0], a1 = w.start[1], b1 = w.end[1];
        while (a0 <= b0 && x[a0] < lo) a0++;
        while (b0 >= a0 && x[b0] > hi) b0--;
        while (a1 <= b1 && y[a1] < lo) a1++;
        while (b1 >= a1 && y[b1] > hi) b1--;
        const smallSide = b0 - a0 <= b1 - a1 ? 0 : 1;
        const smallStart = smallSide === 0 ? a0 : a1;
        const smallEnd = smallSide === 0 ? b0 : b1;
        const largeEnd = smallSide === 0 ? b1 : b0;
        let large = smallSide === 0 ? a1 : a0;
        const smallArc = w.arc[smallSide], largeArc = w.arc[1 - smallSide];
        for (let small = smallStart; small <= smallEnd; small++) {
            // Nearest feasible partner while reserving one distinct node for
            // every remaining anchor. Monotone, maximum-cardinality matching;
            // a basis choice, not a global minimum-distance correspondence.
            const lastAllowed = largeEnd - (smallEnd - small);
            while (large < lastAllowed && Math.abs(largeArc[large + 1] - smallArc[small])
                < Math.abs(largeArc[large] - smallArc[small])) large++;
            w.partner[smallSide][small] = large;
            w.partner[1 - smallSide][large] = small;
            w.pairCount++;
            large++;
        }
    }
    w._paired = true;
    w.stats.pairingBuilds++;
}

function validateColumns(w, columns) {
    let entries = 0;
    for (let side = 0; side < 2; side++) {
        if (!columns[side] || columns[side].length !== w.nodeCount[side] * 6) {
            throw new RangeError('columns must contain six scalar columns per source node');
        }
        for (let dof = 0; dof < columns[side].length; dof++) {
            const source = columns[side][dof];
            if (!source || source.length % 2) throw new TypeError('A source column must contain row/value pairs');
            let previous = -1;
            for (let k = 0; k < source.length; k += 2) {
                const row = source[k];
                if (!Number.isInteger(row) || row < previous || row < 0 || row >= w.rowCount) {
                    throw new RangeError('Column rows must be sorted and lie within the frozen row system');
                }
                finite(source[k + 1], 'Jacobian entry');
                previous = row;
            }
            entries += source.length / 2;
            weightAt(w, side, dof);
        }
    }
    return entries;
}

function appendColumn(w, kind, innerDof, outerDof, ti, to, ci, co, weight) {
    const column = w.columnCount++, start = w.entryCount;
    w.columnOffsets[column] = start;
    w.kind[column] = kind;
    w.innerDof[column] = innerDof; w.outerDof[column] = outerDof;
    w.decodeInner[column] = ti; w.decodeOuter[column] = to;
    w.encodeInner[column] = ci; w.encodeOuter[column] = co;
    w.weights[column] = weight;
    const a = innerDof < 0 ? null : w._input.columns[0][innerDof];
    const b = outerDof < 0 ? null : w._input.columns[1][outerDof];
    const na = a?.length ?? 0, nb = b?.length ?? 0;
    let ia = 0, ib = 0;
    while (ia < na || ib < nb) {
        const rowA = ia < na ? a[ia] : Infinity, rowB = ib < nb ? b[ib] : Infinity;
        const row = Math.min(rowA, rowB);
        let valueA = 0, valueB = 0;
        while (ia < na && a[ia] === row) { valueA += a[ia + 1]; ia += 2; }
        while (ib < nb && b[ib] === row) { valueB += b[ib + 1]; ib += 2; }
        const value = ti * valueA + to * valueB;
        if (!Number.isFinite(value)) throw new RangeError('Transformed Jacobian overflow');
        if (value !== 0) {
            w.rowIndices[w.entryCount] = row;
            w.values[w.entryCount++] = value;
        }
    }
    w.columnOffsets[column + 1] = w.entryCount;
    if (weight > 0 && w.entryCount > start) {
        w.band = Math.max(w.band, w.rowIndices[w.entryCount - 1] - w.rowIndices[start] + 1);
    }
}

/**
 * Input: {bodies:[inner,outer], material:[directScratch|null,...],
 *         columns:[[row,value,...] per dof, ...], count: rowCount}.
 * Call AFTER the parent permutes/sorts its columns, BEFORE allocating Gram.
 * Does not mutate source arrays or require contact records / material sampling.
 * Optional axialCoordinates use original node indexing, offsets align the rods.
 * pairingRevision enables a topology-only cache; mass/J values are ALWAYS fresh.
 */
export function assembleKirchhoffBundleColumns(w, system, options = {}) {
    w._valid = false;
    if (!system?.bodies || system.bodies.length !== 2 || system.bodies[0] === system.bodies[1]
        || !system.material || system.material.length !== 2 || !system.columns || system.columns.length !== 2) {
        throw new TypeError('Two distinct bodies, material systems and column arrays are required');
    }
    w.rowCount = integer(system.count, 'count');
    w._input = system;
    updatePairing(w, system, options);
    const sourceEntries = validateColumns(w, system.columns);
    growColumns(w, 6 * (w.nodeCount[0] + w.nodeCount[1]));
    // Each input nonzero can occur at most once in c and once in r.
    growEntries(w, sourceEntries * 2);
    w.columnCount = 0; w.entryCount = 0; w.band = 1; w.omittedFixedPairDofs = 0;
    w.omittedAxes.fill(0, 0, w.nodeCount[0]);
    w.columnOffsets[0] = 0;
    for (let node = 0; node < w.nodeCount[0]; node++) {
        const outerNode = w.partner[0][node];
        if (outerNode >= 0) {
            for (let axis = 0; axis < 3; axis++) {
                const di = node * 6 + axis, dO = outerNode * 6 + axis;
                const wi = weightAt(w, 0, di), wo = weightAt(w, 1, dO), sum = wi + wo;
                if (!Number.isFinite(sum)) throw new RangeError('Combined inverse mass overflow');
                if (sum === 0) {
                    w.omittedFixedPairDofs += 2;
                    w.omittedAxes[node] |= 1 << axis;
                    continue;
                }
                const a = wo / sum, b = wi / sum;
                // Avoid wi*wo overflow; compute the smaller times a ratio <= 1.
                const wc = wi <= wo ? wi * a : wo * b;
                appendColumn(w, COMMON, di, dO, 1, 1, a, b, wc);
                appendColumn(w, RELATIVE, di, dO, b, -a, 1, -1, sum);
            }
        }
        for (let axis = outerNode >= 0 ? 3 : 0; axis < 6; axis++) {
            const dof = node * 6 + axis;
            appendColumn(w, INNER, dof, -1, 1, 0, 1, 0, weightAt(w, 0, dof));
        }
    }
    for (let node = 0; node < w.nodeCount[1]; node++) {
        for (let axis = w.partner[1][node] >= 0 ? 3 : 0; axis < 6; axis++) {
            const dof = node * 6 + axis;
            appendColumn(w, OUTER, -1, dof, 0, 1, 0, 1, weightAt(w, 1, dof));
        }
    }
    w.stats.assemblies++;
    w._valid = true;
    return w;
}

function assembled(w) {
    if (!w._valid) throw new Error('A successful current bundle assembly is required');
}

/** Add J' W' J'^T into parent lower-band storage; preserve prefilled alpha. */
export function addKirchhoffBundleGram(w, matrix, band = w.band) {
    assembled(w);
    integer(band, 'band', 1);
    if (band < w.band || matrix.length < w.rowCount * band) throw new RangeError('Gram band/buffer is too small');
    for (let column = 0; column < w.columnCount; column++) {
        const weight = w.weights[column];
        if (weight === 0) continue;
        const start = w.columnOffsets[column], end = w.columnOffsets[column + 1];
        for (let a = start; a < end; a++) {
            const row = w.rowIndices[a], scaled = weight * w.values[a];
            for (let b = start; b <= a; b++) matrix[row * band + row - w.rowIndices[b]] += scaled * w.values[b];
        }
    }
    return matrix;
}

function bufferLength(value, length, name) {
    if (!value || value.length < length) throw new RangeError(`${name} buffer is too small`);
}

function overlaps(a, b) {
    return a === b || (ArrayBuffer.isView(a) && ArrayBuffer.isView(b) && a.buffer === b.buffer
        && a.byteOffset < b.byteOffset + b.byteLength && b.byteOffset < a.byteOffset + a.byteLength);
}

/** Encode increments/velocities. Omitted both-fixed coordinates must be zero. */
export function encodeKirchhoffBundleDofs(w, inner, outer, out = w.generalized) {
    assembled(w);
    bufferLength(inner, w.nodeCount[0] * 6, 'inner');
    bufferLength(outer, w.nodeCount[1] * 6, 'outer');
    bufferLength(out, w.columnCount, 'out');
    if (overlaps(out, inner) || overlaps(out, outer)) throw new TypeError('Encode output must not alias its inputs');
    if (w.omittedFixedPairDofs) {
        for (let node = 0; node < w.nodeCount[0]; node++) {
            const partner = w.partner[0][node];
            if (partner < 0) continue;
            for (let axis = 0; axis < 3; axis++) {
                const di = node * 6 + axis, dO = partner * 6 + axis;
                if (w.omittedAxes[node] & (1 << axis) && (inner[di] !== 0 || outer[dO] !== 0)) {
                    throw new RangeError('Both-fixed omitted coordinates require zero increments');
                }
            }
        }
    }
    for (let k = 0; k < w.columnCount; k++) {
        out[k] = (w.innerDof[k] < 0 ? 0 : w.encodeInner[k] * inner[w.innerDof[k]])
            + (w.outerDof[k] < 0 ? 0 : w.encodeOuter[k] * outer[w.outerDof[k]]);
    }
    return out;
}

/** Decode increments to original DOFs; all angular components remain untouched. */
export function decodeKirchhoffBundleDofs(w, generalized, innerOut, outerOut) {
    assembled(w);
    bufferLength(generalized, w.columnCount, 'generalized');
    bufferLength(innerOut, w.nodeCount[0] * 6, 'innerOut');
    bufferLength(outerOut, w.nodeCount[1] * 6, 'outerOut');
    if (overlaps(innerOut, outerOut) || overlaps(generalized, innerOut) || overlaps(generalized, outerOut)) {
        throw new TypeError('Decode buffers must not alias');
    }
    innerOut.fill(0, 0, w.nodeCount[0] * 6); outerOut.fill(0, 0, w.nodeCount[1] * 6);
    for (let k = 0; k < w.columnCount; k++) {
        const value = generalized[k];
        if (w.innerDof[k] >= 0) innerOut[w.innerDof[k]] += w.decodeInner[k] * value;
        if (w.outerDof[k] >= 0) outerOut[w.outerDof[k]] += w.decodeOuter[k] * value;
    }
}

/** Exact original-coordinate W J^T multiplier increment; no output allocations. */
export function recoverKirchhoffBundleCorrection(w, multiplierIncrement, innerOut, outerOut) {
    assembled(w);
    bufferLength(multiplierIncrement, w.rowCount, 'multiplierIncrement');
    if (overlaps(multiplierIncrement, w.generalized)) throw new TypeError('Multiplier input must not alias generalized scratch');
    for (let k = 0; k < w.columnCount; k++) {
        let impulse = 0;
        for (let j = w.columnOffsets[k]; j < w.columnOffsets[k + 1]; j++) {
            impulse += w.values[j] * multiplierIncrement[w.rowIndices[j]];
        }
        w.generalized[k] = w.weights[k] * impulse;
    }
    decodeKirchhoffBundleDofs(w, w.generalized, innerOut, outerOut);
}

/** Check a proposed reconstruction against ORIGINAL mobility, not its reduced
 * Gram matrix. A projected/welded response can satisfy J*dq + alpha*lambda
 * while violating dq = W*J^T*lambda. Such a candidate requires enrichment.
 * Translation tolerance uses the body's position unit; rotation uses radians.
 * This certificate does not replace the parent's constraint/contact KKT.
 */
export function measureKirchhoffBundleMobility(w, multiplierIncrement, inner, outer,
    { translationTolerance, rotationTolerance } = {}) {
    assembled(w);
    for (const [name, value] of Object.entries({translationTolerance, rotationTolerance}))
        if (!Number.isFinite(value) || value < 0) throw new RangeError(`${name} must be finite and nonnegative`);
    bufferLength(multiplierIncrement, w.rowCount, 'multiplierIncrement');
    const corrections = [inner, outer], violations = [];
    let maximumTranslationError = 0, maximumRotationError = 0;
    for (let side = 0; side < 2; side++) {
        bufferLength(corrections[side], w.nodeCount[side] * 6, 'correction');
        for (let dof = 0; dof < w.nodeCount[side] * 6; dof++) {
            let impulse = 0;
            const entries = w._input.columns[side][dof];
            for (let k = 0; k < entries.length; k += 2)
                impulse += entries[k + 1] * multiplierIncrement[entries[k]];
            const expected = weightAt(w, side, dof) * impulse;
            const difference = corrections[side][dof] - expected;
            const error = Number.isFinite(difference) ? Math.abs(difference) : Infinity;
            const angular = dof % 6 >= 3;
            if (angular) maximumRotationError = Math.max(maximumRotationError, error);
            else maximumTranslationError = Math.max(maximumTranslationError, error);
            if (error > (angular ? rotationTolerance : translationTolerance))
                violations.push({side, node:Math.floor(dof / 6), axis:dof % 6, error});
        }
    }
    return {passed:violations.length === 0, maximumTranslationError, maximumRotationError, violations};
}
