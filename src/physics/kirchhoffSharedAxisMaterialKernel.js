import {sharedAxisMaterialKernelBytes} from './kirchhoffSharedAxisMaterialKernelBytes.js';
import {sharedAxisInertiaKernelBytes} from './kirchhoffSharedAxisInertiaKernelBytes.js';
let compiled,inertiaCompiled;
const workspaces=new WeakMap();

/** Fixed memory owned by one chain. All hinges are packed into reusable views
 * and submitted together; the global tangent is read directly from WASM.
 * No growing memory, per-hinge calls, global state pool or output copy. */
export function sharedAxisMaterialKernelWorkspace(chain,hinges) {
    const {dofCount,band}=chain.layout,width=2*band-1;
    const recordCount=hinges+(chain.layout.spins?.size??0);
    let w=workspaces.get(chain);
    if(w&&w.hinges===hinges&&w.dofCount===dofCount&&w.width===width&&w.records.length===recordCount)return w;
    // After material assembly the records are dead. Reserve one extra edge
    // per nonempty material so inertia can reuse those same fixed buffers.
    const recordLength=91,recordsBytes=recordCount*recordLength*8,dofsOffset=recordsBytes,
        tangentOffset=Math.ceil((dofsOffset+recordCount*11*4)/8)*8,
        jacobianOffset=tangentOffset+dofCount*width*8,bytes=jacobianOffset+36*8;
    if(!Number.isSafeInteger(bytes)||bytes>=2**31)throw new RangeError('Material kernel exceeds its 32-bit memory range');
    const pages=Math.max(1,Math.ceil(bytes/65536)),memory=new WebAssembly.Memory({initial:pages,maximum:pages});
    compiled??=new WebAssembly.Module(sharedAxisMaterialKernelBytes);
    const kernel=new WebAssembly.Instance(compiled,{env:{memory}}).exports;
    const tangent=new Float64Array(memory.buffer,tangentOffset,dofCount*width);
    const records=Array.from({length:recordCount},(_,i)=>({data:new Float64Array(memory.buffer,i*recordLength*8,recordLength),
        dofs:new Int32Array(memory.buffer,dofsOffset+i*11*4,11)}));
    let inertiaKernel;
    w={hinges,dofCount,width,memory,tangent,records,
        assemble(){kernel.assemble(0,dofsOffset,hinges,tangentOffset,width,band-1,jacobianOffset);},
        assembleInertia(count){
            if(!Number.isInteger(count)||count<0||count>records.length)throw new RangeError('Inertia record capacity exceeded');
            inertiaCompiled??=new WebAssembly.Module(sharedAxisInertiaKernelBytes);
            inertiaKernel??=new WebAssembly.Instance(inertiaCompiled,{env:{memory}}).exports;
            inertiaKernel.assemble(0,dofsOffset,count,tangentOffset,width,band-1);
        }};
    workspaces.set(chain,w);return w;
}

// A different tangent (Gauss-Newton, a caller-owned array or another layout)
// must retain the standalone JS path. Never rebind a live matrix here.
export function sharedAxisMaterialInertiaWorkspace(chain) {
    const w=workspaces.get(chain);
    return w&&chain.tangent===w.tangent&&w.dofCount===chain.layout.dofCount&&w.width===2*chain.layout.band-1&&
        w.records.length===w.hinges+(chain.layout.spins?.size??0)?w:null;
}
