import {sharedAxisFrictionKernelBytes} from './kirchhoffSharedAxisFrictionKernelBytes.js';

// One synchronous scratch block for 7 x 7 contact tangents. It retains no
// state, material or pose. Callers scatter the returned block before another
// evaluation or a generator yield; published force columns remain owned.
let scratch;
export function sharedAxisFrictionTangent(k,law,normal,geometric) {
    if(!scratch) {
        const memory=new WebAssembly.Memory({initial:1,maximum:1});
        const kernel=new WebAssembly.Instance(new WebAssembly.Module(sharedAxisFrictionKernelBytes),{env:{memory}}).exports;
        scratch={kernel,input:new Float64Array(memory.buffer,0,64),H:new Float64Array(memory.buffer,155*8,49)};
    }
    const {input,kernel,H}=scratch;
    input[0]=k.length;input.set(k.tangent,1);input.set(k.rho,4);input.set(normal,7);input.set(law.traction,10);input.set(law.hessian,13);
    for(let i=0;i<7;i++){input.set(k.J[i],22+3*i);input.set(k.omega[i],43+3*i);}
    kernel.evaluate(geometric?1:0);
    return H;
}
