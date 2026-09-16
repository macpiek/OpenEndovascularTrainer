import {sharedAxisMaterialKernelBytes} from './kirchhoffSharedAxisMaterialKernelBytes.js';
let compiled;
const workspaces=new WeakMap();

/** Fixed memory owned by one chain. All hinges are packed into reusable views
 * and submitted together; the global tangent is read directly from WASM.
 * No growing memory, per-hinge calls, global state pool or output copy. */
export function sharedAxisMaterialKernelWorkspace(chain,hinges) {
    const {dofCount,band}=chain.layout,width=2*band-1;
    let w=workspaces.get(chain);
    if(w&&w.hinges===hinges&&w.dofCount===dofCount&&w.width===width)return w;
    const recordLength=91,recordsBytes=hinges*recordLength*8,dofsOffset=recordsBytes,
        tangentOffset=Math.ceil((dofsOffset+hinges*11*4)/8)*8,
        jacobianOffset=tangentOffset+dofCount*width*8,bytes=jacobianOffset+36*8;
    if(!Number.isSafeInteger(bytes)||bytes>=2**31)throw new RangeError('Material kernel exceeds its 32-bit memory range');
    const pages=Math.max(1,Math.ceil(bytes/65536)),memory=new WebAssembly.Memory({initial:pages,maximum:pages});
    compiled??=new WebAssembly.Module(sharedAxisMaterialKernelBytes);
    const kernel=new WebAssembly.Instance(compiled,{env:{memory}}).exports;
    const tangent=new Float64Array(memory.buffer,tangentOffset,dofCount*width);
    const records=Array.from({length:hinges},(_,i)=>({data:new Float64Array(memory.buffer,i*recordLength*8,recordLength),
        dofs:new Int32Array(memory.buffer,dofsOffset+i*11*4,11)}));
    w={hinges,dofCount,width,memory,tangent,records,
        assemble(){kernel.assemble(0,dofsOffset,hinges,tangentOffset,width,band-1,jacobianOffset);}};
    workspaces.set(chain,w);return w;
}
