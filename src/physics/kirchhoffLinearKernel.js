import { linearKernelBytes } from './kirchhoffLinearKernelBytes.js';
import { bandSimdKernelBytes } from './kirchhoffBandSimdKernelBytes.js';

const module = new WebAssembly.Module(linearKernelBytes);
// SIMD is optional. Unsupported engines retain the original scalar module.
// No relaxed SIMD or fused multiply-add may change rounding in either lane.
const simdModule=WebAssembly.validate(bandSimdKernelBytes)?new WebAssembly.Module(bandSimdKernelBytes):null;
const kernels = new WeakMap();

/** Fixed-capacity numerical workspace. Views are never detached by growth. */
export function createKirchhoffLinearKernel(bytes,{simd=true}={}) {
    const memory = new WebAssembly.Memory({ initial: Math.max(1, Math.ceil(bytes / 65536)) });
    const { exports } = new WebAssembly.Instance(module, { env: { memory } });
    const simdExports=simd&&simdModule?new WebAssembly.Instance(simdModule,{env:{memory}}).exports:null;
    let cursor = 0;
    const kernel = {
        ...exports,
        ...simdExports,
        alloc(Type, count) {
            cursor = Math.ceil(cursor / 8) * 8;
            const result = new Type(memory.buffer, cursor, count);
            cursor += result.byteLength;
            return result;
        }
    };
    kernels.set(memory.buffer, kernel);
    return kernel;
}

export function kirchhoffLinearKernelFor(array) {
    return kernels.get(array.buffer);
}
