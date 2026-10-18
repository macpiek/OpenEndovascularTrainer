import {sharedAxisBasisKernelBytes} from './kirchhoffSharedAxisBasisKernelBytes.js';
let compiled;

// Private synchronous scratch, owned by the rank workspace. This kernel only
// eliminates spatial coefficients; JS still performs Math.hypot rank tests,
// normalization, dual pivots and the original generalized-force certificate.
export function createSharedAxisBasisKernel(dofCount,rowCapacity) {
    if(!Number.isSafeInteger(dofCount)||dofCount<=0||!Number.isSafeInteger(rowCapacity)||rowCapacity<=0)
        throw new RangeError('Positive basis capacities required');
    const entries=dofCount*rowCapacity,required=entries*12+dofCount*20+rowCapacity*28+128;
    // Large/custom systems retain the reference implementation. Do not grow
    // memory or detach views midway through a cached-prefix elimination.
    if(!Number.isSafeInteger(required)||required>32*1024*1024)return null;
    const pages=Math.ceil(required/65536),memory=new WebAssembly.Memory({initial:pages,maximum:pages});
    let offset=0;
    const view=(Type,count)=>{offset=Math.ceil(offset/8)*8;const out=new Type(memory.buffer,offset,count);offset+=out.byteLength;return out;};
    const vectors=view(Float64Array,entries),supports=view(Int32Array,entries),lengths=view(Int32Array,rowCapacity),pivots=view(Int32Array,rowCapacity);
    const v=view(Float64Array,dofCount),support=view(Int32Array,dofCount),pivotAt=view(Int32Array,dofCount),marks=view(Uint32Array,dofCount);
    const queued=view(Uint32Array,rowCapacity),heap=view(Int32Array,rowCapacity),eliminations=view(Int32Array,rowCapacity),factors=view(Float64Array,rowCapacity),result=view(Int32Array,2);
    compiled??=new WebAssembly.Module(sharedAxisBasisKernelBytes);
    const kernel=new WebAssembly.Instance(compiled,{env:{memory}}).exports;
    const sources=[],revisions=[];let previousStamp=0;
    const args=[vectors,supports,lengths,pivots,v,support,pivotAt,marks,queued,heap,eliminations,factors,result].map(a=>a.byteOffset);
    const vector=index=>new Float64Array(memory.buffer,vectors.byteOffset+index*dofCount*8,dofCount);
    const indices=index=>new Int32Array(memory.buffer,supports.byteOffset+index*dofCount*4,dofCount);
    return {pivotAt,bytes:memory.buffer.byteLength,vector,indices,
        attach(index,row) {
            const target=vector(index);target.set(row.v);row.v=target;
            const targetSupport=indices(index);targetSupport.set(row.support);row.support=targetSupport;
        },
        store(index,row,revision) {
            if(revision!==undefined&&sources[index]===row&&revisions[index]===revision)return;
            if(row.v.buffer!==memory.buffer||row.v.byteOffset!==vectors.byteOffset+index*dofCount*8)vectors.set(row.v,index*dofCount);
            if(row.support.buffer!==memory.buffer||row.support.byteOffset!==supports.byteOffset+index*dofCount*4)supports.set(row.support,index*dofCount);
            lengths[index]=row.supportCount;pivots[index]=row.pivot;
            sources[index]=row;revisions[index]=revision;
        },
        eliminate(row,basisCount,stamp) {
            if(stamp<=previousStamp){marks.fill(0);queued.fill(0);}previousStamp=stamp;
            const shared=row.v.buffer===memory.buffer;
            if(!shared)v.set(row.v);
            const sharedSupport=row.support.buffer===memory.buffer;
            args[4]=shared?row.v.byteOffset:v.byteOffset;args[5]=sharedSupport?row.support.byteOffset:support.byteOffset;
            if(!sharedSupport)support.set(row.support);
            kernel.eliminate(...args,dofCount,basisCount,row.supportCount,stamp);
            if(!shared)row.v.set(v);
            row.supportCount=result[0];if(!sharedSupport)row.support.set(support);
            row.eliminations.length=2*result[1];
            for(let i=0;i<result[1];i++){row.eliminations[2*i]=eliminations[i];row.eliminations[2*i+1]=factors[i];}
        }};
}
