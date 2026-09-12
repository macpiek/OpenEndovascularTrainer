import {compositeStrainKernelBytes} from './kirchhoffCompositeStrainKernelBytes.js';
let compiled;

/** One fixed, non-growing WASM memory per sequential tape workspace. Every
 * view refers directly to that memory; no per-sample transfer of the graph,
 * derivatives or adjoints is required. Larger existing storage is reusable.
 */
export function createCompositeStrainKernelStorage(dimension,capacity) {
    const numericLength=capacity*(2*(1+dimension)+5),indicesLength=3*capacity,
        indexOffset=8*numericLength,activeOffset=indexOffset+4*indicesLength,
        resultOffset=8*Math.ceil((activeOffset+capacity)/8),bytes=resultOffset+8*dimension*dimension;
    if(!Number.isSafeInteger(bytes)||bytes>=2**31)throw new RangeError('Strain tape exceeds its 32-bit memory address range');
    const pages=Math.ceil(bytes/65536),memory=new WebAssembly.Memory({initial:pages,maximum:pages});
    compiled??=new WebAssembly.Module(compositeStrainKernelBytes);
    const kernel=new WebAssembly.Instance(compiled,{env:{memory}}).exports;
    return {memory,kernel,numeric:new Float64Array(memory.buffer,0,numericLength),indices:new Int32Array(memory.buffer,indexOffset,indicesLength),
        active:new Uint8Array(memory.buffer,activeOffset,capacity),output:new Float64Array(memory.buffer,resultOffset,dimension*dimension)};
}
