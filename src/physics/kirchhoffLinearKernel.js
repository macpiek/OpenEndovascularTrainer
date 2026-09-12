import { linearKernelBytes } from './kirchhoffLinearKernelBytes.js';

const module = new WebAssembly.Module(linearKernelBytes);
const kernels = new WeakMap();

/** Fixed-capacity numerical workspace. Views are never detached by growth. */
export function createKirchhoffLinearKernel(bytes) {
    const memory = new WebAssembly.Memory({ initial: Math.max(1, Math.ceil(bytes / 65536)) });
    const { exports } = new WebAssembly.Instance(module, { env: { memory } });
    let cursor = 0;
    const kernel = {
        ...exports,
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
