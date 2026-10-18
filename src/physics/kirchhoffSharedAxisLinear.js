import {fallbackWorkingSetLease} from './kirchhoffSharedAxisRejectedLinearization.js';
import {sharedAxisEffectiveGap} from './kirchhoffSharedAxisCompliance.js';
import {canCondenseSharedAxisContact,assembleCondensedSharedAxisContacts,recoverCondensedSharedAxisReaction,measureUncondensedSharedAxisResidual} from './kirchhoffSharedAxisCondensation.js';
import {materializeSharedAxisContacts,separatedSharedAxisContactDirection,isDeferredSharedAxisContact} from './kirchhoffSharedAxisInactiveContacts.js';
import { assemblePreparedSharedAxisMatrix, applyPreparedSharedAxisFixedMask } from './kirchhoffSharedAxisMatrixAssembly.js';
import { createBorderedContactUpdates } from './kirchhoffBorderedContactUpdates.js';
import { createIncrementalContactLU } from './kirchhoffIncrementalContactLU.js';
import { prepareSharedAxisActiveBasis, sharedAxisBasisLinearization } from './kirchhoffSharedAxisActiveBasis.js';
import { createCoulombBandLU, createCoulombBandLUArena } from './kirchhoffCoulombBandLU.js';
import {createBandRefinement} from './kirchhoffBandRefinement.js';

// Band elimination is synchronous and overwrites its entire factor/RHS. All
// shared-axis workspaces can use one arena; retaining a different WASM memory
// for every active-set layout causes unnecessary browser allocation/GC work.
const sharedLinearScratchArena=createCoulombBandLUArena();
// Compact factorizations are synchronous and their outputs are copied into
// the caller's private workspace before yielding. Reuse bounded structural
// maps across callers and lend one numeric arena to changing layouts. The
// previous per-state layout cache could not recycle an evicted matrix's arrays.
// Retained/incremental factors must keep their own workspaces instead.
const sharedCompactWorkspaces=new Map();
const sharedCompactNumbers={};
let compactNumericAllocations=0;
function numericBuffers(w,arena) {
    const lengths={matrix:w.band.entries,residual:w.count,scales:w.count,solution:w.count,correction:w.count,error:w.count,
        fixed:w.count,increment:w.primal.length,multiplierIncrement:w.dual.length};
    for(const key of Object.keys(lengths)) {
        const size=lengths[key],Type=key==='fixed'?Uint8Array:Float64Array;
        if(!arena){w[key]=new Type(size);continue;}
        let buffer=arena[key];
        if(!buffer||buffer.length<size) {
            buffer=arena[key]=new Type(Math.ceil(size/(key==='matrix'?4096:128))*(key==='matrix'?4096:128));
            compactNumericAllocations++;
        }
        // Layouts keep their structural maps, but borrow numerical storage
        // only for a synchronous solve. Rebind after a larger layout grew it.
        if(w[key]?.buffer!==buffer.buffer||w[key].length!==size)w[key]=buffer.subarray(0,size);
    }
}
const noCondensedRows=Object.freeze([]);
export const getSharedAxisLinearScratchStats=()=>({...sharedLinearScratchArena.diagnostics,
    compactLayouts:sharedCompactWorkspaces.size,compactNumericAllocations,
    compactNumericBytes:Object.values(sharedCompactNumbers).reduce((n,v)=>n+v.byteLength,0)});

const rowSupportKey = (rows,cache) => {
    if(!cache)return rows.map(r=>r.dofs.join('.')+(r.extraForceDofs?.length?':'+r.extraForceDofs.join('.'):'')).join(',');
    return rows.map(r => {
    let key=cache?.get(r);
    if(key===undefined){key=r.dofs.join('.')+(r.extraForceDofs?.length?':'+r.extraForceDofs.join('.'):'');cache?.set(r,key);}
    return key;
}).join(',');
};

// Preserve exactly the previous toPrecision(9) cycle equivalence. Most active
// sets are visited only once: copy their dual, and format numbers only if that
// same set returns. Snapshots belong to this linear call, never the next pose.
export function repeatedSharedAxisDual(visited,key,dual) {
    const previous=visited.get(key);
    if(previous)for(const saved of previous) {
        let same=true;
        for(let i=0;i<dual.length;i++)if(saved[i].toPrecision(9)!==dual[i].toPrecision(9)){same=false;break;}
        if(same)return true;
    }
    const saved=dual.slice();if(previous)previous.push(saved);else visited.set(key,[saved]);return false;
}
function validateExtraForce(row, dofCount) {
    if (!row.extraForceDofs && !row.extraForceJacobian) return;
    if (!row.extraForceDofs || !row.extraForceJacobian || row.extraForceDofs.length !== row.extraForceJacobian.length)
        throw new RangeError('Shared axis extra force support and derivative must have equal lengths');
    row.extraForceDofs.forEach((p, k) => {
        if (!Number.isInteger(p) || p < 0 || p >= dofCount || !Number.isFinite(row.extraForceJacobian[k]))
            throw new RangeError('Invalid shared axis extra force column');
    });
}

// Accurate residual only after the inexpensive certificate failed. Short
// physical spans can produce O(1e9) terms cancelling to a sub-micro residual;
// compensate both summation and product rounding before rejecting/refining.
function compensatedResidual(w,i) {
    const {matrix:A,residual:F,solution:x,band:{starts,ends,offsets}}=w;
    let high=F[i],low=0;
    for(let j=starts[i];j<=ends[i];j++) {
        const a=A[offsets[i]+j],b=x[j];if(a===0||b===0)continue;
        const product=a*b,sa=134217729*a,sb=134217729*b;
        if(!Number.isFinite(sa)||!Number.isFinite(sb)||!Number.isFinite(product))return Infinity;
        const ah=sa-(sa-a),bh=sb-(sb-b),al=a-ah,bl=b-bh;
        const productError=((ah*bh-product)+ah*bl+al*bh)+al*bl;
        const sum=high+product,v=sum-high;
        low+=(high-(sum-v))+(product-v)+productError;high=sum;
    }
    const value=high+low;return Number.isFinite(value)?value:Infinity;
}

// Spatial ordering: a position followed by the independent material spins on
// its outgoing edge. Bending rows touch two edges / three consecutive nodes.
export function createSharedAxisLayout(edgeTools) {
    const positions = new Int32Array(edgeTools.length + 1), spins = new Map();
    for (const ids of edgeTools) for (const id of ids) if (!spins.has(id)) spins.set(id, new Int32Array(edgeTools.length).fill(-1));
    let dofCount = 0;
    for (let node = 0; node < positions.length; node++) {
        positions[node] = dofCount; dofCount += 3;
        for (const id of edgeTools[node] ?? []) spins.get(id)[node] = dofCount++;
    }
    let band = 3;
    for (let node = 0; node + 2 < positions.length; node++) band = Math.max(band, positions[node + 2] + 3 - positions[node]);
    return { positions, spins, dofCount, band };
}

export function createSharedAxisLinear(layout, definitions, {lazy=false,numericArena=null,compactBand=true}={}) {
    // Interleave length/wall reactions locally; never form a dense contact
    // Schur complement. Reuse the tested production general-band LU kernel.
    definitions.forEach(r=>validateExtraForce(r,layout.dofCount));
    if(lazy)return {layout,definitions,matrix:{length:0},increment:new Float64Array(layout.dofCount),
        multiplierIncrement:new Float64Array(definitions.length)};
    const primal = new Int32Array(layout.dofCount), dual = new Int32Array(definitions.length);
    const ending=new Map();definitions.forEach((r,i)=>{let last=r.dofs.at(-1);for(const p of r.extraForceDofs??[])last=Math.max(last,p);if(!ending.has(last))ending.set(last,[]);ending.get(last).push(i);});
    let count = 0;
    for (let i = 0; i < primal.length; i++) {
        primal[i] = count++;
        for(const r of ending.get(i)??[])dual[r]=count++;
    }
    const starts = new Int32Array(count), ends = new Int32Array(count), offsets = new Int32Array(count);
    if(compactBand) {
        // Keep the full physical Hessian stencil, including current zeros.
        // Each constraint adds only its actual local row/column support. A
        // crowded contact site need not widen every unrelated matrix row.
        for(let i=0;i<count;i++)starts[i]=ends[i]=i;
        for(let i=0;i<primal.length;i++) {
            starts[primal[i]]=primal[Math.max(0,i-layout.band+1)];
            ends[primal[i]]=primal[Math.min(primal.length-1,i+layout.band-1)];
        }
        for(let index=0;index<definitions.length;index++) {
            const r=definitions[index],d=dual[index];
            let first=d,last=d;
            for(const p of r.dofs){first=Math.min(first,primal[p]);last=Math.max(last,primal[p]);}
            starts[d]=first;ends[d]=last;
            // The same support also covers a reaction-dependent geometric
            // Hessian. Extra force columns have no constraint transpose.
            for(const p of r.dofs){const row=primal[p];starts[row]=Math.min(starts[row],first);ends[row]=Math.max(ends[row],last);}
            for(const p of r.extraForceDofs??[]){const row=primal[p];starts[row]=Math.min(starts[row],d);ends[row]=Math.max(ends[row],d);}
        }
    } else {
        // Reference storage for equivalence tests and caller diagnostics.
        let width=0;
        for(let i=0;i<primal.length;i++)width=Math.max(width,primal[i]-primal[Math.max(0,i-layout.band+1)]);
        definitions.forEach((d,r)=>{
            for(const p of d.dofs)width=Math.max(width,Math.abs(dual[r]-primal[p]));
            for(const p of d.extraForceDofs??[])width=Math.max(width,Math.abs(dual[r]-primal[p]));
        });
        for(let i=0;i<count;i++){starts[i]=Math.max(0,i-width);ends[i]=Math.min(count-1,i+width);}
    }
    let entries=0,kl=0,ku=0;
    for(let i=0;i<count;i++) {
        offsets[i]=entries-starts[i];entries+=ends[i]-starts[i]+1;
        kl=Math.max(kl,i-starts[i]);ku=Math.max(ku,ends[i]-i);
    }
    const band = { starts, ends, offsets, entries, kl, ku };
    const workspace={primal,dual,count,band,lu:createCoulombBandLU(band,count,{arena:sharedLinearScratchArena})};
    numericBuffers(workspace,numericArena);return workspace;
}

function solveSharedAxisLinearOnce(w, chain, { rows, gradient, fixed, tolerance = 1e-8, activeSet, inactiveRows=[], condensedRows=[], incrementalContext,modifiedNewtonContext,supportKeyCache,matrixAssembly,wasmLinearAssembly=false,supportsVerified=false }) {
    // Optional force support can appear after an originally normal-only
    // workspace was created. Rebuild the reference band when it no longer
    // contains that support; compact workspaces are keyed by both supports.
    if(!w.primal||!supportsVerified&&rows.some((r,index)=>{
        const d=w.dual[index],{starts,ends}=w.band;
        let first=d,last=d;
        for(const p of r.dofs){first=Math.min(first,w.primal[p]);last=Math.max(last,w.primal[p]);}
        if(first<starts[d]||last>ends[d])return true;
        for(const p of r.dofs) {
            const row=w.primal[p];
            if(d<starts[row]||d>ends[row]||r.geometricHessian&&(first<starts[row]||last>ends[row]))return true;
        }
        for(const p of r.extraForceDofs??[])if(d<starts[w.primal[p]]||d>ends[w.primal[p]])return true;
        return false;
    })) {
        const key=rowSupportKey(rows,supportKeyCache);
        if(w.fullReferenceKey!==key){w.fullReference=createSharedAxisLinear(chain.layout,rows);w.fullReferenceKey=key;}
        return solveSharedAxisLinearOnce(w.fullReference,chain,{rows,gradient,fixed,tolerance,activeSet,inactiveRows,condensedRows,incrementalContext,modifiedNewtonContext,supportKeyCache,matrixAssembly,wasmLinearAssembly});
    }
    const { matrix: A, residual: F, band: { starts, ends, offsets }, primal, dual } = w;
    if(matrixAssembly)assemblePreparedSharedAxisMatrix(w,chain,{rows,gradient,fixed,tolerance,activeSet,inactiveRows},matrixAssembly);
    else {
    A.fill(0); F.fill(0); w.fixed.fill(0);
    const add = (i, j, v) => {
        if (!v) return;
        if (j < starts[i] || j > ends[i]) throw new RangeError('Shared axis matrix support overflow');
        A[offsets[i] + j] += v;
    };
    for (let i = 0; i < primal.length; i++) {
        F[primal[i]] = gradient[i]; w.fixed[primal[i]] = fixed[i];
        if (chain.tangent) {
            const half = chain.layout.band - 1, width = 2 * half + 1;
            for (let j = Math.max(0, i-half); j <= Math.min(primal.length-1, i+half); j++)
                add(primal[i], primal[j], chain.tangent[i*width+j-i+half]);
        } else for (let j = Math.max(0, i - chain.layout.band + 1); j <= i; j++) {
            const h = chain.hessian[i * chain.layout.band + i - j];
            add(primal[i], primal[j], h); if (i !== j) add(primal[j], primal[i], h);
        }
    }
    rows.forEach((r, index) => {
        const d = dual[index], sign = r.kind === 'wall' ? -1 : 1;
        const active = r.kind === 'length' || activeSet[index];
        const movable = r.dofs.some((p, k) => !fixed[p] && r.jacobian[k] !== 0);
        r.dofs.forEach((p, k) => add(primal[p], d, sign * r.jacobian[k]));
        // Additive derivative of stationarity with respect to this physical
        // reaction (including its own sign). It is NOT a constraint Jacobian:
        // there is deliberately no transpose in the dual equation below.
        r.extraForceDofs?.forEach((p,k)=>add(primal[p],d,r.extraForceJacobian[k]));
        if (!active || (!movable&&!r.compliance)) {
            if (active && (r.kind === 'length' ? Math.abs(r.gap) > tolerance : r.gap < -tolerance)) throw new RangeError('Incompatible fixed shared axis constraint');
            add(d, d, 1); F[d] = r.multiplier;
        } else {
            F[d] = sharedAxisEffectiveGap(r);
            if(r.compliance)add(d,d,r.compliance);
            r.dofs.forEach((p, k) => add(d, primal[p], r.jacobian[k]));
        }
        if (r.geometricHessian) r.dofs.forEach((p, i) => r.dofs.forEach((q, j) => add(primal[p], primal[q], r.geometricHessian[i * r.dofs.length + j])));
    });
    for(const r of inactiveRows)if(r.geometricHessian)
        r.dofs.forEach((p,i)=>r.dofs.forEach((q,j)=>add(primal[p],primal[q],r.geometricHessian[i*r.dofs.length+j])));
    }
    if(condensedRows.length)assembleCondensedSharedAxisContacts(w,condensedRows);
    const wasmLinear=wasmLinearAssembly&&!incrementalContext&&!modifiedNewtonContext;
    if(matrixAssembly)applyPreparedSharedAxisFixedMask(w);
    for (let i = 0; i < w.count; i++) {
        if(!matrixAssembly) {
            for(let j=starts[i];j<=ends[i];j++)if(w.fixed[i]||w.fixed[j])A[offsets[i]+j]=i===j?1:0;
            if(w.fixed[i])F[i]=0;
        }
        if(!wasmLinear) {
            let maximum = 0;
            for (let j = starts[i]; j <= ends[i]; j++) maximum = Math.max(maximum, Math.abs(A[offsets[i] + j]));
            w.scales[i] = 1 / Math.sqrt(Math.max(maximum, 1e-30));
        }
    }
    // Experimental factors belong to one immutable linearization. Their
    // memory stays exclusive while yielded; the ordinary arena is overwritten
    // by other solvers. Compact mode uses this LU as its border-update base.
    let lu=wasmLinear?(w.wasmLU??=createCoulombBandLU(w.band,w.count,{arena:sharedLinearScratchArena,wasmAssembly:true})):w.lu;
    if(incrementalContext) {
        lu=incrementalContext.workspaces.get(w);
        if(!lu){lu=createIncrementalContactLU(w.band,w.count,{maxRank:incrementalContext.maxRank});incrementalContext.workspaces.set(w,lu);incrementalContext.allDiagnostics.push(lu.diagnostics);}
    }
    if(modifiedNewtonContext)lu=modifiedNewtonContext.getLU(w);
    const retained=!!(incrementalContext||modifiedNewtonContext);
    const factorsBefore=retained?lu.diagnostics.factorizations:0;
    const solved = lu.solve(A, F, w.scales, 0, w.solution,wasmLinear);
    // A rejected factorization does not write a valid solution. Do not report
    // a residual computed from the previous solve's scratch as this direction.
    if (!solved) {
        w.increment.fill(0); w.multiplierIncrement.fill(0);
        return {increment:w.increment,multiplierIncrement:w.multiplierIncrement,
            factorizations:retained?lu.diagnostics.factorizations-factorsBefore:1,residual:Infinity,converged:false,failure:'band-lu-rejected'};
    }
    for (let i = 0; i < w.count; i++) w.solution[i] *= w.scales[i];
    let residual = Infinity, factorizations = 1, refinement;
    // Certify the original unscaled equations, refining the solution when
    // material stiffness / a short edge makes forward error significant.
    try { for (let attempt = 0; attempt < 3; attempt++) {
        residual = wasmLinear?lu.measureOriginalResidual(w.solution,w.error,F):null;
        if(residual===null) {
        residual = 0;
        for (let i = 0; i < w.count; i++) {
            let v = F[i]; for (let j = starts[i]; j <= ends[i]; j++) v += A[offsets[i] + j] * w.solution[j];
            w.error[i] = v; residual = Math.max(residual, Math.abs(v));
        }
        }
        if(residual>tolerance) {
            residual=0;
            for(let i=0;i<w.count;i++) {
                const v=compensatedResidual(w,i);w.error[i]=v;residual=Math.max(residual,Math.abs(v));
            }
        }
        if (residual <= tolerance || !solved || attempt === 2) break;
        if(wasmLinear&&!retained) {
            // Only a failed certificate pays for retained factors. The second
            // correction solves the same matrix with a new residual RHS.
            refinement??=createBandRefinement(w.band,w.count,A,w.scales);
            const before=refinement.diagnostics.factorizations;
            const refined=refinement.solve(w.error,w.correction);
            factorizations+=refinement.diagnostics.factorizations-before;
            if(!refined)break;
        } else {
            factorizations++;
            if (!lu.solve(A, w.error, w.scales, 0, w.correction)) break;
        }
        for (let i = 0; i < w.count; i++) w.solution[i] += w.correction[i] * w.scales[i];
    }} finally {refinement?.dispose();}
    primal.forEach((p, i) => { w.increment[i] = w.solution[p]; });
    dual.forEach((d, i) => { w.multiplierIncrement[i] = w.solution[d]; });
    return { increment: w.increment, multiplierIncrement: w.multiplierIncrement, factorizations:retained?lu.diagnostics.factorizations-factorsBefore:factorizations, residual,
        converged: solved && residual <= tolerance };
}

function solveCompactWorkingSet(w,chain,options,activeSet) {
    const context=options.incrementalContext;
    if(context?.bordered) {
        const updated=context.bordered.solve(activeSet,options.tolerance??1e-8,w,context.maxRank);
        if(updated)return updated;
        // Rebase on the current compact layout after an unstable or large change.
        for(const lu of context.workspaces.values())lu.dispose();
        context.workspaces.clear();context.bordered=null;
    }
    let scratch;
    if(options.reuseStructure!==false) {
        scratch=w.compactScratch;
        if(!scratch||scratch.mask.length!==activeSet.length||scratch.gradient.length!==options.gradient.length)
            scratch=w.compactScratch={mask:new Uint8Array(activeSet.length),initialized:false,indices:[],inactiveIndices:[],rows:[],inactiveRows:[],gradient:new Float64Array(options.gradient.length)};
        let changed=!scratch.initialized;
        for(let i=0;i<activeSet.length;i++)if(scratch.mask[i]!==activeSet[i]){changed=true;break;}
        if(changed) {
            scratch.indices.length=scratch.inactiveIndices.length=0;
            for(let i=0;i<activeSet.length;i++)(activeSet[i]?scratch.indices:scratch.inactiveIndices).push(i);
            scratch.mask.set(activeSet);scratch.initialized=true;
        }
        scratch.rows.length=scratch.indices.length;scratch.inactiveRows.length=scratch.inactiveIndices.length;
        for(let i=0;i<scratch.indices.length;i++)scratch.rows[i]=options.rows[scratch.indices[i]];
        for(let i=0;i<scratch.inactiveIndices.length;i++)scratch.inactiveRows[i]=options.rows[scratch.inactiveIndices[i]];
        scratch.gradient.set(options.gradient);
    }
    const allIndices=scratch?.indices??[],inactiveRows=scratch?.inactiveRows??[];
    if(!scratch)options.rows.forEach((r,i)=>{if(activeSet[i])allIndices.push(i);else inactiveRows.push(r);});
    const condensedIndices=options.condenseCompliantContacts&&!context&&!options.modifiedNewtonContext?
        allIndices.filter(i=>canCondenseSharedAxisContact(options.rows[i],chain.layout)):noCondensedRows;
    const condensedRows=condensedIndices.length?condensedIndices.map(i=>options.rows[i]):noCondensedRows;
    const condensed=condensedIndices.length?new Set(condensedIndices):null;
    const indices=condensedIndices.length?allIndices.filter(i=>!condensed.has(i)):allIndices;
    const activeRows=condensedIndices.length?indices.map(i=>options.rows[i]):scratch?.rows??indices.map(i=>options.rows[i]);
    // Recheck actual supports even with a reused index map: contact/friction
    // derivatives can change their sparsity without changing the active mask.
    const key=`${chain.layout.dofCount}/${chain.layout.band}/`+rowSupportKey(activeRows,options.supportKeyCache);
    const shared=options.reuseConstraintWork&&options.reuseStructure!==false&&!context&&!options.modifiedNewtonContext&&options.reuseLinearBuffers!==false;
    const cache=shared?sharedCompactWorkspaces:(w.activeWorkspaces??=new Map());
    let packed=cache.get(key);
    if(!packed) {
        if(cache.size>=(shared?16:8))cache.delete(cache.keys().next().value);
        packed=createSharedAxisLinear(chain.layout,activeRows,{numericArena:shared?sharedCompactNumbers:null});cache.set(key,packed);
    } else if(shared){cache.delete(key);cache.set(key,packed);}
    if(shared)numericBuffers(packed,sharedCompactNumbers);
    const gradient=scratch?.gradient??Float64Array.from(options.gradient);
    // Inactive equations are delta-lambda = -lambda. Eliminate their force
    // columns exactly, retaining the physical geometric Hessian at entry.
    for(const r of inactiveRows)if(r.multiplier!==0) {
        const sign=r.kind==='wall'?-1:1;
        for(let k=0;k<r.dofs.length;k++)gradient[r.dofs[k]]-=sign*r.multiplier*r.jacobian[k];
        if(r.extraForceDofs)for(let k=0;k<r.extraForceDofs.length;k++)gradient[r.extraForceDofs[k]]-=r.multiplier*r.extraForceJacobian[k];
    }
    const result=solveSharedAxisLinearOnce(packed,chain,{...options,gradient,condensedRows,
        inactiveRows:condensedRows.length?[...inactiveRows,...condensedRows]:inactiveRows,
        // The key above checked every current primal/force support against
        // this layout. Public workspaces still perform the full bounds check.
        rows:activeRows,supportsVerified:true,activeSet:packed.allActive??=new Uint8Array(indices.length).fill(1)});
    if(options.modifiedNewtonContext&&result.converged)options.modifiedNewtonContext.record(packed,indices);
    if(context&&result.converged) {
        const lu=context.workspaces.get(packed);
        context.bordered=createBorderedContactUpdates(packed,lu,indices,options.rows);
        context.borderDiagnostics.push(context.bordered.diagnostics);
    }
    w.increment.set(result.increment);
    options.rows.forEach((r,i)=>{w.multiplierIncrement[i]=-r.multiplier;});
    indices.forEach((index,i)=>{w.multiplierIncrement[index]=result.multiplierIncrement[i];});
    for(const i of condensedIndices)w.multiplierIncrement[i]=recoverCondensedSharedAxisReaction(options.rows[i],w.increment);
    if(condensedIndices.length) {
        if(options.condensationStats) {
            options.condensationStats.attempts++;
            options.condensationStats.eliminatedRows+=condensedIndices.length;
        }
        const originalResidual=result.converged?measureUncondensedSharedAxisResidual(chain,options,activeSet,w.increment,w.multiplierIncrement):Infinity;
        if(originalResidual>(options.tolerance??1e-8)) {
            // Never accept merely because the reduced equations passed. Keep
            // the uncondensed numerical fallback and count its full cost.
            const failedFactorizations=result.factorizations;
            if(options.condensationStats)options.condensationStats.fallbacks++;
            const reference=solveCompactWorkingSet(w,chain,{...options,condenseCompliantContacts:false},activeSet);
            return {...reference,factorizations:failedFactorizations+reference.factorizations,
                condensationFallback:true,condensationResidual:originalResidual};
        }
        result.residual=Math.max(result.residual,originalResidual);
        result.condensedContacts=condensedIndices.length;
    }
    w.peakActiveEntries=Math.max(w.peakActiveEntries??0,packed.matrix.length);
    return {...result,increment:w.increment,multiplierIncrement:w.multiplierIncrement};
}


/** Solve the linearized unilateral problem, not equality rows followed by
 * clamping negative reactions. A released wall changes the global direction;
 * merely clamping its multiplier leaves an unbalanced force in that direction.
 */
// Nearly parallel wall equations can produce enormous opposing reactions.
// Their cancellation may fail the linear residual certificate even after LU
// refinement. A clearly negative wall reaction can still propose which
// inequality to RELEASE in the private working set. Never apply that
// uncertified direction: the new set must be solved and certified from scratch,
// and every released inequality is checked again before returning success.
function recoverNegativeContact(rows,result,dual,activeSet,primalContact,tolerance,trace) {
    if(result.failure!=null||!Number.isFinite(result.residual)||
        !result.increment?.every(Number.isFinite)||!result.multiplierIncrement?.every(Number.isFinite))return false;
    let release=-1,fraction=1;
    for(let i=0;i<rows.length;i++) {
        const target=rows[i].multiplier+result.multiplierIncrement[i];
        if(!Number.isFinite(target))return false;
        if(rows[i].kind!=='wall'||!activeSet[i]||primalContact(rows[i]))continue;
        // This is a pivot heuristic, not a certificate or a replacement for
        // the requested tolerance. Avoid deciding from a tiny uncertain sign.
        if(target < -Math.max(tolerance,32*result.residual)) {
            const next=dual[i]/(dual[i]-target);
            if(next<fraction){fraction=next;release=i;}
        }
    }
    if(release<0)return false;
    for(let i=0;i<rows.length;i++) {
        dual[i]=(1-fraction)*dual[i]+fraction*(rows[i].multiplier+result.multiplierIncrement[i]);
        if(rows[i].kind==='wall')dual[i]=Math.max(0,dual[i]);
    }
    dual[release]=0;activeSet[release]=0;
    trace?.push({kind:'negative-reaction-recovery',release:rows[release].id??release,
        residual:result.residual,predictedReaction:rows[release].multiplier+result.multiplierIncrement[release]});
    return true;
}

function* iterateActiveSet(w, chain, options, batchSize, solutionCache) {
    const { rows, tolerance = 1e-8 } = options;
    const primalContact=r=>options.primalCompliantContacts&&r.kind==='wall'&&r.compliance>0;
    const activeSet=Uint8Array.from(rows,r=>r.kind==='length'||(primalContact(r)?r.gap<0:r.multiplier>tolerance));
    // This dual is a private feasible pivot iterate, not the physical reaction
    // used to assemble the equations. Starting it at zero lets the batch path
    // release all initially negative target reactions at the same first blocker.
    // The single-pivot fallback retains the original physical starting dual.
    let factorizations=0,result,activeSetAttempts=0,batchedRows=0,workingSetReuses=0,negativeReactionRecoveries=0;const visited=options.reuseConstraintWork?new Map():new Set(),dual=Float64Array.from(rows,r=>r.kind==='wall'?(options.zeroDualStart&&batchSize>1?0:Math.max(0,r.multiplier)):r.multiplier);
    const finish=extra=>({...result,...extra,factorizations,activeSetAttempts,batchedRows,workingSetReuses,negativeReactionRecoveries});
    let basisRequired=!options.deferActiveBasis;
    for(let attempt=0;attempt<(options.maxActiveSetAttempts??Math.max(8,rows.length*2));attempt++) {
        yield {kind:'linear-active-set',attempt,batchSize};
        activeSetAttempts++;
        // Most deferred solves never need rank repair. Snapshot/compare the
        // Jacobians only when a failed factorization actually requests it.
        // Rows and the fixed mask are immutable for this linear call; the
        // token is then shared by its later active-set and batch retries.
        if(basisRequired&&options.basisCache===undefined)options.basisCache=options.reuseStructure===false?null:
            options.reuseConstraintWork?sharedAxisBasisLinearization(rows,options.fixed):Symbol('linear-basis');
        const prepared=basisRequired?prepareSharedAxisActiveBasis({rows,fixed:options.fixed,activeSet,dual,trace:options.trace,reuseStructure:options.reuseStructure,basisCache:options.basisCache,basisWorkspaceKey:options.basisWorkspaceKey,lazyBasisCoefficients:options.lazyBasisCoefficients,wasmBasis:options.wasmBasis}):{converged:true};
        if(!prepared.converged)return finish({converged:false,failure:prepared.failure});
        const setKey=activeSet.join('');
        if(options.reuseConstraintWork) {
            if(repeatedSharedAxisDual(visited,setKey,dual))return finish({converged:false,failure:'active-set-cycle'});
        } else {
            const signature=setKey+'/'+Array.from(dual,v=>v.toPrecision(9)).join(',');
            if(visited.has(signature))return finish({converged:false,failure:'active-set-cycle'});
            visited.add(signature);
        }
        const saved=solutionCache?.get(setKey);
        if(saved) {
            // The feasible dual iterate chooses the next pivot, but does not
            // enter this KKT matrix or RHS. Revisit exactly the same working
            // set without refactorization. Restore owned output views because
            // other working sets have overwritten the workspace since then.
            w.increment.set(saved.increment);w.multiplierIncrement.set(saved.multiplierIncrement);
            result={...saved,increment:w.increment,multiplierIncrement:w.multiplierIncrement,factorizations:0};workingSetReuses++;
        } else {
            result=options.compactWorkingSet===false?solveSharedAxisLinearOnce(w,chain,{...options,activeSet}):
                solveCompactWorkingSet(w,chain,options,activeSet);
            // Numerical rejection is deterministic for these immutable
            // equations too. Keep its rejection status and owned diagnostic
            // direction across basis/batch retries; never treat it as a valid
            // Newton step. Retained updates and condensation have their own
            // changing factor state, so only ordinary uncondensed LU qualifies.
            // A public full workspace may override its LU (e.g. a transient
            // failure hook). Memoize factor rejection only for our private
            // compact solver, whose factor and RHS are completely rebuilt.
            const numericalRejection=result.failure==null&&Number.isFinite(result.residual)||
                options.compactWorkingSet!==false&&result.failure==='band-lu-rejected';
            const rejected=options.reuseRejectedWorkingSet!==false&&!result.converged&&numericalRejection&&
                !options.incrementalContext&&!options.modifiedNewtonContext&&
                !options.condenseCompliantContacts;
            if(solutionCache&&(result.converged||rejected)) {
                if(solutionCache.size>=32)solutionCache.delete(solutionCache.keys().next().value);
                solutionCache.set(setKey,{...result,increment:result.increment.slice(),multiplierIncrement:result.multiplierIncrement.slice()});
            }
            factorizations+=result.factorizations;
        }
        if(!result.converged){
            if(!basisRequired){basisRequired=true;visited.clear();attempt--;continue;}
            if(options.recoverNegativeContact===true&&
                recoverNegativeContact(rows,result,dual,activeSet,primalContact,tolerance,options.trace)) {
                negativeReactionRecoveries++;continue;
            }
            return finish();
        }
        let worst=-tolerance,change=-1,alpha=1,activate=null;
        // Move the feasible dual iterate toward the active-set solution only
        // as far as the FIRST multiplier reaching zero. Removing the most
        // negative target instead can cycle at nearly coplanar mesh features.
        for(let i=0;i<rows.length;i++) {
            if(rows[i].kind!=='wall'||!activeSet[i]||primalContact(rows[i]))continue;
            const target=rows[i].multiplier+result.multiplierIncrement[i];
            if(target < -tolerance) {
                const fraction=dual[i]/(dual[i]-target);
                if(fraction<alpha){alpha=fraction;change=i;worst=target;}
            }
        }
        if(change>=0&&options.simultaneousContactRelease&&batchSize>1) {
            // Semismooth working-set update. This changes only the private
            // search path; the final original LCP certificate is unchanged.
            // Cycling/failed directions still retry the single-pivot solver.
            for(let i=0;i<rows.length;i++) {
                const target=rows[i].multiplier+result.multiplierIncrement[i];
                dual[i]=rows[i].kind==='wall'?Math.max(0,target):target;
                if(rows[i].kind==='wall'&&activeSet[i]&&!primalContact(rows[i])&&target < -tolerance) {
                    activeSet[i]=0;batchedRows++;
                }
            }
            continue;
        }
        if(change>=0) {
            // At alpha=0 these contacts are tied first blockers. All have
            // zero feasible reaction; releasing them together preserves the
            // dual iterate. The single-pivot fallback keeps its original order.
            if(options.batchRelease&&batchSize>1&&alpha===0) {
                for(let i=0;i<rows.length;i++)if(rows[i].kind==='wall'&&activeSet[i]&&!primalContact(rows[i])&&dual[i]===0&&rows[i].multiplier+result.multiplierIncrement[i]<-tolerance&&i!==change) {
                    activeSet[i]=0;batchedRows++;
                }
            }
            for(let i=0;i<rows.length;i++) {
                dual[i]+=alpha*(rows[i].multiplier+result.multiplierIncrement[i]-dual[i]);
                if(rows[i].kind==='wall')dual[i]=Math.max(0,dual[i]);
            }
            dual[change]=0;
        } else {
            for(let i=0;i<rows.length;i++) {
                dual[i]=rows[i].multiplier+result.multiplierIncrement[i];
                if(rows[i].kind==='wall')dual[i]=Math.max(0,dual[i]);
            }
            const violated=[];
            for(let i=0;i<rows.length;i++) {
                const r=rows[i];if(r.kind!=='wall'||activeSet[i]||primalContact(r))continue;
                if(separatedSharedAxisContactDirection(r,result.increment))continue;
                const residual=r.gap+r.dofs.reduce((sum,p,k)=>sum+r.jacobian[k]*result.increment[p],0)+(r.compliance??0)*(r.multiplier+result.multiplierIncrement[i]);
                if(batchSize>1&&residual<-tolerance)violated.push({index:i,residual});
                if(residual<worst){worst=residual;change=i;}
            }
            if(violated.length)activate=violated.sort((a,b)=>a.residual-b.residual||a.index-b.index).slice(0,batchSize).map(v=>v.index);
        }
        options.trace?.push({attempt,active:Array.from(activeSet).flatMap((v,i)=>v&&rows[i].kind==='wall'?[rows[i].id??`${rows[i].sample}/${rows[i].edge}`]:[]),change:change<0?null:(rows[change].id??`${rows[change].sample}/${rows[change].edge}`),worst,
            ...(activate?{activate:activate.map(i=>rows[i].id??i)}:{})});
        if(change<0)return finish();
        if(activate){for(const i of activate)activeSet[i]=1;if(activate.length>1)batchedRows+=activate.length;}
        else activeSet[change]=1-activeSet[change];
    }
    return finish({converged:false,failure:'active-set-limit'});
}

function* iterateWithFallback(w,chain,options,batchSize) {
    // A failed earlier strategy may have solved the very same fixed working
    // sets. Only an exact coefficient/RHS match can lend those owned outputs;
    // each strategy still performs its own pivots and complementarity checks.
    const incomingOptions=options;
    const failedCache=options.reuseWorkingSet!==false?options[fallbackWorkingSetLease]:null;
    const solutionCache=options.reuseWorkingSet===false?null:failedCache?.find(chain,options)??new Map();
    // Numerical Jacobian prefixes may survive another solve only after exact
    // support/Jacobian/fixed-mask validation. Matrix and solution caches below
    // remain local to this immutable linearization.
    options={...options,matrixAssembly:options.reuseMatrixAssembly?{rows:options.rows}:null,supportKeyCache:options.reuseConstraintWork?new Map():null,...(options.incrementalContacts?{...(options.incrementalContacts==='full'?{compactWorkingSet:false}:{}),incrementalContext:{maxRank:options.incrementalMaxRank??8,workspaces:new Map(),borderDiagnostics:[],allDiagnostics:[]}}:{}),basisCache:undefined};
    try {
        const first=yield* iterateActiveSet(w,chain,options,batchSize,solutionCache);
        const incrementalStats=()=>options.incrementalContext?{incrementalStats:structuredClone(options.incrementalContext.allDiagnostics),borderedStats:structuredClone(options.incrementalContext.borderDiagnostics)}:{};
        if(first.converged||batchSize===1&&!options.deferActiveBasis)return {...first,...incrementalStats(),batchActivation:batchSize>1,batchFallback:false};
        // Restart from the original physical rows, reactions, gradient and Hessian.
        // Only scratch buffers have changed. The reference result may alias them,
        // so save scalar accounting before the second solve overwrites those views.
        const failedFactorizations=first.factorizations,failedAttempts=first.activeSetAttempts;
        options.trace?.push({kind:'batch-fallback',failure:first.failure});
        const reference=yield* iterateActiveSet(w,chain,{...options,deferActiveBasis:false},1,solutionCache);
        const result={...reference,...incrementalStats(),factorizations:failedFactorizations+reference.factorizations,
            activeSetAttempts:failedAttempts+reference.activeSetAttempts,workingSetReuses:first.workingSetReuses+reference.workingSetReuses,batchActivation:true,batchFallback:true,
            negativeReactionRecoveries:first.negativeReactionRecoveries+reference.negativeReactionRecoveries,
            batchFailure:first.failure,batchFactorizations:failedFactorizations,referenceFactorizations:reference.factorizations,
            batchAttempts:failedAttempts,referenceAttempts:reference.activeSetAttempts};
        failedCache?.remember(chain,incomingOptions,result,solutionCache);
        return result;
    } finally {for(const lu of options.incrementalContext?.workspaces.values()??[])lu.dispose();}
}

/** Yield between active-set/LU attempts while preserving the synchronous API.
 * CPU timing excludes consumer pauses, including pauses before a fallback. */
export function* iterateSharedAxisLinear(w,chain,options) {
    const condensationStats=options.condenseCompliantContacts?{attempts:0,eliminatedRows:0,fallbacks:0}:null;
    if(condensationStats)options={...options,condensationStats};
    if(options.compactWorkingSet===false||options.incrementalContacts||options.modifiedNewtonContext||options.observeLinearSystem)materializeSharedAxisContacts(options.rows);
    options.observeLinearSystem?.({chain,options});
    options.rows.forEach(r=>{
        validateExtraForce(r,chain.layout.dofCount);
        if(r.compliance!==undefined&&(!(r.compliance>=0)||!Number.isFinite(r.compliance)||r.kind!=='wall'&&r.compliance!==0))
            throw new RangeError('Only walls may have nonnegative finite compliance');
    });
    if(options.maxActiveSetAttempts!==undefined&&(!Number.isInteger(options.maxActiveSetAttempts)||options.maxActiveSetAttempts<1))throw new RangeError('Active-set attempt limit must be a positive integer');
    const batchSize=options.batchActivation===false?1:(options.batchActivationSize??8);
    if(!Number.isInteger(batchSize)||batchSize<1||batchSize>256)throw new RangeError('Batch activation size must be between 1 and 256');
    const reductionStarted=performance.now(),reduced=constantRowReduction(w,chain,options);
    let cpuMs=performance.now()-reductionStarted,done=false;
    const iterator=iterateWithFallback(reduced?.workspace??w,chain,reduced?.options??options,batchSize);
    try {
        while(true) {
            const start=performance.now();let next;
            try{next=iterator.next();}finally{cpuMs+=performance.now()-start;}
            if(next.done){
                done=true;
                const mapped=reduced&&next.value.increment&&next.value.multiplierIncrement;
                if(mapped) {
                    w.increment.set(next.value.increment);
                    for(let i=0;i<options.rows.length;i++)w.multiplierIncrement[i]=-options.rows[i].multiplier;
                    for(let i=0;i<reduced.indices.length;i++)w.multiplierIncrement[reduced.indices[i]]=next.value.multiplierIncrement[i];
                    w.peakActiveEntries=Math.max(w.peakActiveEntries??0,reduced.workspace.peakActiveEntries??0);
                }
                return {...next.value,...(mapped?{increment:w.increment,multiplierIncrement:w.multiplierIncrement,
                    constantRowsEliminated:options.rows.length-reduced.indices.length}:{}),cpuMs,...(condensationStats?{condensationStats}:{})};
            }
            yield next.value;
        }
    }finally{if(!done)iterator.return();}
}

function constantRowReduction(w,chain,options) {
    // Discovery still runs on the full physical row set. Its dummy inequality
    // (and a sheath sample outside the sheath) has no force or derivative at
    // this linearization. Omit only equations that are exactly constant and
    // feasible, never a merely small coefficient or a deferred wall witness.
    if(options.eliminateConstantRows===false||!options.reuseConstraintWork||options.compactWorkingSet===false||
        options.incrementalContacts||options.modifiedNewtonContext||options.observeLinearSystem||options.trace)return null;
    const rows=options.rows,indices=[];
    for(let i=0;i<rows.length;i++) {
        const r=rows[i];
        const omit=r.kind==='wall'&&r.multiplier===0&&!r.compliance&&!r.geometricHessian&&!r.extraForceDofs?.length&&
            Number.isFinite(r.gap)&&r.gap>=0&&r.jacobian.every(v=>v===0)&&!isDeferredSharedAxisContact(r);
        if(!omit)indices.push(i);
    }
    if(indices.length===rows.length)return null;
    const selected=indices.map(i=>rows[i]);
    let workspace=w.constantRowWorkspace;
    if(!workspace||workspace.multiplierIncrement.length!==selected.length)
        workspace=w.constantRowWorkspace=createSharedAxisLinear(chain.layout,selected,{lazy:true});
    return {workspace,indices,options:{...options,rows:selected,
        // Removing tautologies must not shorten the reference search budget.
        maxActiveSetAttempts:options.maxActiveSetAttempts??Math.max(8,rows.length*2)}};
}

export function solveSharedAxisLinear(w,chain,options) {
    const iterator=iterateSharedAxisLinear(w,chain,options);let next;
    do{next=iterator.next();}while(!next.done);return next.value;
}
