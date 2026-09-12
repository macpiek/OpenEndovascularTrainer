import { createKirchhoffLinearKernel } from './kirchhoffLinearKernel.js';
import { validateCoulombGeneralBandMatrix } from './kirchhoffCoulombBandLU.js';

const factorCaches=new WeakMap();
const topologyCaches=new WeakMap();
const factorArenas=new WeakMap();

// General LU is necessary: section interiors include material multipliers
// and may contain a nonsymmetric contact Newton derivative.
function factorLocal(matrix,n,workspace) {
    if(!workspace||workspace.n!==n) {
        const kernel=createKirchhoffLinearKernel(8*n*n+20*n+64);
        workspace={n,kernel,lu:kernel.alloc(Float64Array,n*n),pivots:kernel.alloc(Int32Array,n),rhs:kernel.alloc(Float64Array,n),starts:kernel.alloc(Int32Array,n),ends:kernel.alloc(Int32Array,n)};
    }
    const {kernel,lu,pivots,rhs,starts,ends}=workspace;
    lu.set(matrix);
    if(kernel.factorDenseLU(lu.byteOffset,pivots.byteOffset,n)!==0)throw new RangeError('Singular section interior');
    if(!lu.every(Number.isFinite))throw new RangeError('Section factor overflow');
    // Inspect the completed factors, so pivot-created fill is retained.
    // Interior zeros stay in their original arithmetic order.
    for(let i=0;i<n;i++) {
        let first=0,last=n-1;
        while(first<i&&lu[i*n+first]===0)first++;
        while(last>i&&lu[i*n+last]===0)last--;
        starts[i]=first;ends[i]=last;
    }
    const solve=input=>{
        rhs.set(input);
        kernel.solveDenseProfileLU(lu.byteOffset,pivots.byteOffset,rhs.byteOffset,n,starts.byteOffset,ends.byteOffset);
        if(!rhs.every(Number.isFinite))throw new RangeError('Section solve overflow');
        return rhs.slice();
    };
    solve.workspace=workspace;
    return solve;
}

/** Exact static condensation of mutually independent section interiors.
 * Each section may contain primal and multiplier coordinates. Only boundary
 * reactions enter the global matrix; recover restores all interior values.
 * Cross-section edges must be made boundary coordinates by the caller.
 */
export function condenseKirchhoffSections({matrix,rhs,sections,workspace}) {
    const began=performance.now();
    let localPackingMs=0,localFactorMs=0,localResponsesMs=0,localReactionsMs=0;
    const n=rhs?.length;
    if(!rhs||!Array.from(rhs).every(Number.isFinite)||!Array.isArray(sections))throw new TypeError('Invalid section input');
    if(workspace!==undefined&&(workspace===null||typeof workspace!=='object'))throw new TypeError('Invalid section workspace');
    let previous=workspace?factorCaches.get(workspace):undefined;
    const nextCache=new Map();
    validateCoulombGeneralBandMatrix(matrix,n);
    const owner=new Int32Array(n).fill(-1);
    const blocks=sections.map((section,b)=>{
        if(!Array.isArray(section))throw new TypeError('Section indices must be arrays');
        return section.map(i=>{
            if(!Number.isInteger(i)||i<0||i>=n||owner[i]!==-1)throw new RangeError('Invalid or repeated section index');
            owner[i]=b;return i;
        });
    }).filter(block=>block.length);
    // One WASM memory per section workspace, not one reserved address space
    // per tiny section. Recovery closures own copies, so repartition may
    // reuse this arena after invalidating factor fingerprints.
    const signature=JSON.stringify(blocks);
    const bytes=blocks.reduce((sum,block)=>sum+8*block.length*block.length+24*block.length+64,0);
    let arena=workspace?factorArenas.get(workspace):null;
    if(!arena||arena.storage.length<bytes) {
        const capacity=Math.max(65536,2**Math.ceil(Math.log2(Math.max(1,bytes))));
        const kernel=createKirchhoffLinearKernel(capacity);
        arena={kernel,storage:kernel.alloc(Uint8Array,capacity),signature:null};
        if(workspace)factorArenas.set(workspace,arena);
    }
    if(arena.signature!==signature) {
        if(previous)previous.clear();previous=undefined;
        arena.signature=signature;arena.slots=new Map();let cursor=0;
        const alloc=(Type,count)=>{cursor=Math.ceil(cursor/8)*8;const view=new Type(arena.storage.buffer,cursor,count);cursor+=view.byteLength;return view;};
        for(const block of blocks){const k=block.length;arena.slots.set(block.join(','),{
            n:k,kernel:arena.kernel,lu:alloc(Float64Array,k*k),pivots:alloc(Int32Array,k),
            rhs:alloc(Float64Array,k),starts:alloc(Int32Array,k),ends:alloc(Int32Array,k)
        });}
    }
    const cachedTopology=workspace?topologyCaches.get(workspace):null;
    let topologyReused=cachedTopology?.rows.length===n;
    // Compare actual nonzero support, not the band envelope: Newton contact
    // rows can change activity without changing their allocated storage.
    if(topologyReused)for(let i=0;i<n;i++) {
        const support=cachedTopology.support[i];let entry=0;
        for(let j=matrix.starts[i];j<=matrix.ends[i];j++) {
            const v=matrix.values[matrix.offsets[i]+j];if(v===0)continue;
            if(owner[i]>=0&&owner[j]>=0&&owner[i]!==owner[j])throw new RangeError('Section interiors are coupled across a boundary');
            if(support[entry++]!==j){topologyReused=false;break;}
        }
        if(entry!==support.length)topologyReused=false;
        if(!topologyReused)break;
    }
    let rows,columns;
    if(topologyReused) {
        ({rows,columns}=cachedTopology);
        for(let i=0;i<n;i++)for(const j of cachedTopology.support[i])rows[i].set(j,matrix.values[matrix.offsets[i]+j]);
    } else {
        rows=Array.from({length:n},()=>new Map());columns=Array.from({length:n},()=>[]);
        for(let i=0;i<n;i++)for(let j=matrix.starts[i];j<=matrix.ends[i];j++) {
            const v=matrix.values[matrix.offsets[i]+j];if(v===0)continue;
            if(owner[i]>=0&&owner[j]>=0&&owner[i]!==owner[j])throw new RangeError('Section interiors are coupled across a boundary');
            rows[i].set(j,v);columns[j].push(i);
        }
        if(workspace)topologyCaches.set(workspace,{rows,columns,support:rows.map(row=>Array.from(row.keys()))});
    }
    const retained=Array.from({length:n},(_,i)=>i).filter(i=>owner[i]<0),map=new Int32Array(n).fill(-1);
    retained.forEach((i,k)=>map[i]=k);
    const count=retained.length,out=retained.map(i=>new Map([...rows[i]].filter(([j])=>map[j]>=0).map(([j,v])=>[map[j],v])));
    const load=Float64Array.from(retained,i=>rhs[i]),recovery=[];
    let localFactorEntries=0,localFactorizations=0,localFactorReuses=0,localResponseSolves=0,localResponseReuses=0,localSchurProducts=0,localSchurReuses=0;
    const add=(i,j,v)=>{
        const next=(out[i].get(j)??0)+v;
        if(!Number.isFinite(next))throw new RangeError('Section Schur overflow');
        if(next===0)out[i].delete(j);else out[i].set(j,next);
    };
    const sectionTopologyMs=performance.now()-began;
    const localIndex=new Int32Array(n).fill(-1);
    for(const block of blocks) {
        let stage=performance.now();
        const k=block.length,A=new Float64Array(k*k),outgoing=new Set(),incoming=new Set();
        block.forEach((i,a)=>localIndex[i]=a);
        block.forEach((i,a)=>{
            for(const [j,v] of rows[i])if(localIndex[j]>=0)A[a*k+localIndex[j]]=v;
            for(const j of rows[i].keys())if(map[j]>=0)outgoing.add(j);
            for(const j of columns[i])if(map[j]>=0)incoming.add(j);
        });
        const key=block.join(','),cached=previous?.get(key);
        const reuse=cached?.matrix.length===A.length&&A.every((v,i)=>v===cached.matrix[i]);
        // Retired recovery closures only own response vectors, not the LU.
        // Invalidate the old factor before reusing its storage, including when
        // this assembly later fails: it must never be mistaken for the old A.
        if(!reuse&&cached)previous.delete(key);
        localPackingMs+=performance.now()-stage;stage=performance.now();
        const solve=reuse?cached.solve:factorLocal(A,k,arena.slots.get(key)),free=solve(block.map(i=>rhs[i]));localFactorEntries+=A.length;
        if(reuse)localFactorReuses++;else localFactorizations++;
        localFactorMs+=performance.now()-stage;stage=performance.now();
        const responseCache=new Map();
        const responses=[...outgoing].map(j=>{
            const input=Float64Array.from(block,i=>rows[i].get(j)??0),old=reuse?cached.responses?.get(j):null;
            const same=old&&input.every((v,i)=>v===old.input[i]);
            const values=same?old.values:solve(input);
            if(same)localResponseReuses++;else localResponseSolves++;
            responseCache.set(j,{input,values});
            // Cache keys use original coordinates, whereas each returned
            // response uses this assembly's current boundary numbering.
            return {originalColumn:j,column:map[j],values};
        });
        localResponsesMs+=performance.now()-stage;stage=performance.now();
        const reactionCache=new Map();
        nextCache.set(key,{matrix:A,solve,responses:responseCache,reactions:reactionCache});
        for(const i of incoming) {
            const coefficients=block.map(j=>rows[i].get(j)??0),r=map[i];
            const nonzero=[];for(let a=0;a<k;a++)if(coefficients[a]!==0)nonzero.push(a);
            const oldReaction=reuse?cached.reactions?.get(i):null;
            const sameReaction=oldReaction&&coefficients.every((v,a)=>v===oldReaction.coefficients[a]);
            const products=new Map();
            reactionCache.set(i,{coefficients,products});
            for(const a of nonzero)load[r]-=coefficients[a]*free[a];
            for(const response of responses) {
                // Reuse only the identical incoming row and identical solved
                // outgoing vector. Original indices survive boundary renumbering.
                const oldProduct=sameReaction?oldReaction.products.get(response.originalColumn):null;
                let value;
                if(oldProduct?.response===response.values) {
                    value=oldProduct.value;localSchurReuses++;
                } else {
                    value=0;for(const a of nonzero)value+=coefficients[a]*response.values[a];
                    localSchurProducts++;
                }
                products.set(response.originalColumn,{response:response.values,value});
                add(r,response.column,-value);
            }
        }
        localReactionsMs+=performance.now()-stage;
        recovery.push({block,free,responses});
        for(const i of block)localIndex[i]=-1;
    }
    if(!load.every(Number.isFinite))throw new RangeError('Section load overflow');
    const starts=new Int32Array(count),ends=new Int32Array(count),offsets=new Int32Array(count);
    let entries=0;
    out.forEach((row,i)=>{
        let first=i,last=i;for(const j of row.keys()){first=Math.min(first,j);last=Math.max(last,j);}
        starts[i]=first;ends[i]=last;offsets[i]=entries-first;entries+=last-first+1;
        if(entries>2147483647)throw new RangeError('Section band exceeds Int32 addressing');
    });
    const values=new Float64Array(entries);out.forEach((row,i)=>{for(const [j,v]of row)values[offsets[i]+j]=v;});
    if(workspace)factorCaches.set(workspace,nextCache);
    return {matrix:{values,starts,ends,offsets},rhs:load,count,retainedIndices:[...retained],
        recover(solution) {
            if(!solution||solution.length!==count||!Array.from(solution).every(Number.isFinite))throw new RangeError('Invalid section boundary solution');
            const full=new Float64Array(n);retained.forEach((i,k)=>full[i]=solution[k]);
            for(const {block,free,responses}of recovery)block.forEach((i,a)=>{
                let value=free[a];for(const r of responses)value-=r.values[a]*solution[r.column];full[i]=value;
            });
            if(!full.every(Number.isFinite))throw new RangeError('Section reconstruction overflow');return full;
        },diagnostics:{originalCount:n,globalCount:count,localCount:n-count,sectionCount:blocks.length,localFactorEntries,
            topologyReused:Boolean(topologyReused),sectionTopologyMs,localPackingMs,localFactorMs,localResponsesMs,localReactionsMs,
            localFactorizations,localFactorReuses,localResponseSolves,localResponseReuses,localSchurProducts,localSchurReuses,bandEntries:entries}};
}
