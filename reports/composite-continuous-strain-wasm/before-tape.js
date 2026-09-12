/** Exact local forward-over-reverse differentiation for material strains and
 * scalar elastic densities. Each node keeps its value and first derivatives. Reverse
 * accumulation differentiates adjoints to recover the full configuration
 * Hessian of each requested output, including geometric stiffness.
 *
 * Scratch scales with operations * dimension, not operations * dimension^2.
 * The tape belongs to one sequential frame workspace; checkpoints retain only
 * the current configuration's shared native-frame prefix between samples.
 */
export function createCompositeStrainDifferentialTape(dimension, capacity = 2048, reuse = null) {
    if (!Number.isInteger(dimension) || dimension < 1 || !Number.isInteger(capacity) || capacity < dimension)
        throw new RangeError('A positive tape dimension and sufficient capacity are required');
    const stride = 1 + dimension, count = capacity * stride, numericLength = 2 * count + 5 * capacity;
    const storage = reuse && reuse.numeric.length >= numericLength && reuse.indices.length >= 3 * capacity && reuse.active.length >= capacity
        ? reuse : { numeric: new Float64Array(numericLength), indices: new Int32Array(3 * capacity), active: new Uint8Array(capacity) };
    const data = storage.numeric.subarray(0, count), adjoint = storage.numeric.subarray(count, 2 * count),
        coefficients = storage.numeric.subarray(2 * count, numericLength), parentA = storage.indices.subarray(0, capacity),
        parentB = storage.indices.subarray(capacity, 2 * capacity), variableIndex = storage.indices.subarray(2 * capacity, 3 * capacity), active = storage.active;
    let cursor = 0;
    function alloc(value, a = -1, b = -1, da = 0, db = 0, daa = 0, dab = 0, dbb = 0) {
        if (cursor + stride > count) throw new RangeError('Strain derivative tape capacity exceeded');
        const at = cursor, node = cursor / stride, coefficient = 5 * node; cursor += stride;
        data.fill(0, at, at + stride); data[at] = value;
        parentA[node] = a; parentB[node] = b; variableIndex[node] = -1;
        coefficients[coefficient] = da; coefficients[coefficient + 1] = db;
        coefficients[coefficient + 2] = daa; coefficients[coefficient + 3] = dab; coefficients[coefficient + 4] = dbb;
        return at;
    }
    function unary(a, value, first, second = 0) {
        const r = alloc(value, a, -1, first, 0, second);
        for (let i = 1; i <= dimension; i++) data[r + i] = first * data[a + i];
        return r;
    }
    function binary(a, b, value, da, db, daa = 0, dab = 0, dbb = 0) {
        const r = alloc(value, a, b, da, db, daa, dab, dbb);
        for (let i = 1; i <= dimension; i++) data[r + i] = da * data[a + i] + db * data[b + i];
        return r;
    }
    function hessians(outputs) {
        if (!Array.isArray(outputs) || !outputs.length || outputs.some(at => !Number.isInteger(at) || at < 0 || at >= cursor || at % stride))
            throw new RangeError('Hessian outputs must be live tape nodes');
        const result = new Float64Array(outputs.length * dimension * dimension), nodes = cursor / stride;
        for (let output = 0; output < outputs.length; output++) {
            adjoint.fill(0, 0, cursor); active.fill(0, 0, nodes);
            adjoint[outputs[output]] = 1; active[outputs[output] / stride] = 1;
            for (let node = nodes - 1; node >= 0; node--) {
                if (!active[node]) continue;
                const at = node * stride, a = parentA[node], b = parentB[node];
                if (a < 0) continue;
                const c = node * 5, da = coefficients[c], db = coefficients[c + 1], daa = coefficients[c + 2], dab = coefficients[c + 3], dbb = coefficients[c + 4], bar = adjoint[at];
                active[a / stride] = 1; adjoint[a] += da * bar;
                if (b < 0) {
                    if (daa === 0) for (let j = 1; j <= dimension; j++) adjoint[a + j] += da * adjoint[at + j];
                    else for (let j = 1; j <= dimension; j++) adjoint[a + j] += da * adjoint[at + j] + bar * daa * data[a + j];
                } else {
                    active[b / stride] = 1; adjoint[b] += db * bar;
                    if (daa === 0 && dab === 0 && dbb === 0) {
                        for (let j = 1; j <= dimension; j++) {
                            const direction = adjoint[at + j];
                            adjoint[a + j] += da * direction; adjoint[b + j] += db * direction;
                        }
                    } else if (daa === 0 && dab === 1 && dbb === 0) {
                        for (let j = 1; j <= dimension; j++) {
                            const direction = adjoint[at + j];
                            adjoint[a + j] += da * direction + bar * data[b + j];
                            adjoint[b + j] += db * direction + bar * data[a + j];
                        }
                    } else {
                        for (let j = 1; j <= dimension; j++) {
                            const direction = adjoint[at + j], ga = data[a + j], gb = data[b + j];
                            adjoint[a + j] += da * direction + bar * (daa * ga + dab * gb);
                            adjoint[b + j] += db * direction + bar * (dab * ga + dbb * gb);
                        }
                    }
                }
            }
            for (let node = 0; node < nodes; node++) {
                const index = variableIndex[node]; if (index < 0) continue;
                const out = (output * dimension + index) * dimension, at = node * stride;
                for (let j = 0; j < dimension; j++) result[out + j] += adjoint[at + 1 + j];
            }
        }
        if (!result.every(Number.isFinite)) throw new RangeError('Nonfinite continuous strain Hessian');
        return result;
    }
    return { data, dimension, stride, storage, retainedBytes: storage.numeric.byteLength + storage.indices.byteLength + storage.active.byteLength,
        reset() { cursor = 0; }, used: () => cursor / stride, checkpoint: () => cursor,
        rewind(at) { if (!Number.isInteger(at) || at < 0 || at > cursor || at % stride) throw new RangeError('Invalid strain tape checkpoint'); cursor = at; },
        constant: value => alloc(value),
        variable(value, index) {
            if (!Number.isInteger(index) || index < 0 || index >= dimension) throw new RangeError('Invalid strain tape variable index');
            const at = alloc(value); variableIndex[at / stride] = index; data[at + 1 + index] = 1; return at;
        },
        add: (a, b) => binary(a, b, data[a] + data[b], 1, 1),
        sub: (a, b) => binary(a, b, data[a] - data[b], 1, -1),
        mul: (a, b) => binary(a, b, data[a] * data[b], data[b], data[a], 0, 1),
        scale: (a, scale) => unary(a, data[a] * scale, scale),
        reciprocal(a) { const v = data[a]; if (v === 0) throw new RangeError('Zero strain derivative denominator'); return unary(a, 1 / v, -1 / (v * v), 2 / (v * v * v)); },
        sqrt(a) { const v = Math.sqrt(data[a]); if (!(v > 0)) throw new RangeError('Strain reconstruction requires nonzero tangents'); return unary(a, v, .5 / v, -.25 / (v * v * v)); },
        sin: a => unary(a, Math.sin(data[a]), Math.cos(data[a]), -Math.sin(data[a])),
        cos: a => unary(a, Math.cos(data[a]), -Math.sin(data[a]), -Math.cos(data[a])),
        atan2(y, x) {
            const xv = data[x], yv = data[y], d = xv * xv + yv * yv;
            if (!(d > 0)) throw new RangeError('Unresolved strain reference connection');
            return binary(y, x, Math.atan2(yv, xv), xv / d, -yv / d, -2 * xv * yv / (d * d), (yv * yv - xv * xv) / (d * d), 2 * xv * yv / (d * d));
        },
        hessians, finite() { for (let i = 0; i < cursor; i++) if (!Number.isFinite(data[i])) return false; return true; }
    };
}
