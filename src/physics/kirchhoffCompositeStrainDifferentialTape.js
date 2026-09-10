import {createCompositeStrainKernelStorage} from './kirchhoffCompositeStrainKernel.js';

/** Exact local forward-over-reverse differentiation for material strains and
 * scalar elastic densities. Each node keeps its value and first derivatives. Reverse
 * accumulation differentiates adjoints to recover the full configuration
 * Hessian of each requested output, including geometric stiffness.
 *
 * Scratch scales with operations * dimension, not operations * dimension^2.
 * The tape belongs to one sequential frame workspace; checkpoints retain only
 * the current configuration's shared native-frame prefix between samples.
 * Scalar values are available while recording. finite()/finalize() computes
 * first derivatives in one WASM call before exposing them; hessians() also
 * finalizes before its reverse sweeps. No partial derivatives are current
 * until that finalization. Rewind invalidates the overwritten suffix only.
 */
export function createCompositeStrainDifferentialTape(dimension, capacity = 2048, reuse = null) {
    if (!Number.isInteger(dimension) || dimension < 1 || !Number.isInteger(capacity) || capacity < dimension)
        throw new RangeError('A positive tape dimension and sufficient capacity are required');
    const stride = 1 + dimension, count = capacity * stride, numericLength = 2 * count + 5 * capacity;
    const storage = reuse?.kernel && reuse.numeric.length >= numericLength && reuse.indices.length >= 3 * capacity && reuse.active.length >= capacity && reuse.output.length >= dimension*dimension
        ? reuse : createCompositeStrainKernelStorage(dimension,capacity);
    const data = storage.numeric.subarray(0, count), adjoint = storage.numeric.subarray(count, 2 * count),
        coefficients = storage.numeric.subarray(2 * count, numericLength), parentA = storage.indices.subarray(0, capacity),
        parentB = storage.indices.subarray(capacity, 2 * capacity), variableIndex = storage.indices.subarray(2 * capacity, 3 * capacity), active = storage.active;
    let cursor = 0, prepared = 0, forwardCalls = 0, forwardNodes = 0, reverseCalls = 0;
    function alloc(value, a = -1, b = -1, da = 0, db = 0, daa = 0, dab = 0, dbb = 0) {
        if (cursor + stride > count) throw new RangeError('Strain derivative tape capacity exceeded');
        const at = cursor, node = cursor / stride, coefficient = 5 * node; cursor += stride;
        data[at] = value;
        parentA[node] = a; parentB[node] = b; variableIndex[node] = -1;
        coefficients[coefficient] = da; coefficients[coefficient + 1] = db;
        coefficients[coefficient + 2] = daa; coefficients[coefficient + 3] = dab; coefficients[coefficient + 4] = dbb;
        return at;
    }
    function unary(a, value, first, second = 0) {
        const r = alloc(value, a, -1, first, 0, second);
        return r;
    }
    function binary(a, b, value, da, db, daa = 0, dab = 0, dbb = 0) {
        const r = alloc(value, a, b, da, db, daa, dab, dbb);
        return r;
    }
    function finalize() {
        const end=cursor/stride;if(prepared===end)return true;
        forwardCalls++;forwardNodes+=end-prepared;
        const status=storage.kernel.forward(data.byteOffset,coefficients.byteOffset,parentA.byteOffset,parentB.byteOffset,variableIndex.byteOffset,dimension,prepared,end);
        if(status===0)prepared=end;
        return status===0;
    }
    function hessians(outputs) {
        if (!Array.isArray(outputs) || !outputs.length || outputs.some(at => !Number.isInteger(at) || at < 0 || at >= cursor || at % stride))
            throw new RangeError('Hessian outputs must be live tape nodes');
        if(!finalize())throw new RangeError('Nonfinite continuous strain derivatives');
        const result = new Float64Array(outputs.length * dimension * dimension), nodes = cursor / stride;
        for (let output = 0; output < outputs.length; output++) {
            reverseCalls++;
            const status=storage.kernel.reverse(data.byteOffset,adjoint.byteOffset,coefficients.byteOffset,parentA.byteOffset,parentB.byteOffset,variableIndex.byteOffset,active.byteOffset,
                dimension,nodes,outputs[output],storage.output.byteOffset);
            if(status!==0)throw new RangeError('Nonfinite continuous strain Hessian');
            result.set(storage.output.subarray(0,dimension*dimension),output*dimension*dimension);
        }
        return result;
    }
    return { data, dimension, stride, storage, retainedBytes: storage.memory.buffer.byteLength,
        get diagnostics(){return {forwardCalls,forwardNodes,reverseCalls,preparedNodes:prepared,capacity};},
        reset() { cursor = 0; prepared = 0; }, used: () => cursor / stride, checkpoint: () => cursor,
        rewind(at) { if (!Number.isInteger(at) || at < 0 || at > cursor || at % stride) throw new RangeError('Invalid strain tape checkpoint'); cursor = at; prepared=Math.min(prepared,at/stride); },
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
        hessians, finalize, finite:finalize
    };
}
