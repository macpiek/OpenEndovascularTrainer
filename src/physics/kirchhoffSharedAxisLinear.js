import { createBorderedContactUpdates } from './kirchhoffBorderedContactUpdates.js';
import { createIncrementalContactLU } from './kirchhoffIncrementalContactLU.js';
import { prepareSharedAxisActiveBasis } from './kirchhoffSharedAxisActiveBasis.js';
import { createCoulombBandLU, createCoulombBandLUArena } from './kirchhoffCoulombBandLU.js';

// Band elimination is synchronous and overwrites its entire factor/RHS. All
// shared-axis workspaces can use one arena; retaining a different WASM memory
// for every active-set layout causes unnecessary browser allocation/GC work.
const sharedLinearScratchArena=createCoulombBandLUArena();
export const getSharedAxisLinearScratchStats=()=>({...sharedLinearScratchArena.diagnostics});

const rowSupportKey = rows => rows.map(r => r.dofs.join('.') + (r.extraForceDofs?.length ? ':' + r.extraForceDofs.join('.') : '')).join(',');
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

export function createSharedAxisLinear(layout, definitions, {lazy=false}={}) {
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
    let width = 0;
    for (let i = 0; i < primal.length; i++) width = Math.max(width, primal[i] - primal[Math.max(0, i - layout.band + 1)]);
    definitions.forEach((d, r) => {
        d.dofs.forEach(i => { width = Math.max(width, Math.abs(dual[r] - primal[i])); });
        d.extraForceDofs?.forEach(i => { width = Math.max(width, Math.abs(dual[r] - primal[i])); });
    });
    const starts = Int32Array.from({ length: count }, (_, i) => Math.max(0, i - width));
    const ends = Int32Array.from({ length: count }, (_, i) => Math.min(count - 1, i + width));
    let entries = 0;
    const offsets = Int32Array.from(starts, (start, i) => { const offset = entries - start; entries += ends[i] - start + 1; return offset; });
    const band = { starts, ends, offsets, entries, kl: width, ku: width };
    return { primal, dual, count, band, matrix: new Float64Array(entries), residual: new Float64Array(count),
        scales: new Float64Array(count), solution: new Float64Array(count), correction: new Float64Array(count), error: new Float64Array(count), fixed: new Uint8Array(count),
        increment: new Float64Array(primal.length), multiplierIncrement: new Float64Array(dual.length),
        lu: createCoulombBandLU(band, count, {arena:sharedLinearScratchArena}) };
}

function solveSharedAxisLinearOnce(w, chain, { rows, gradient, fixed, tolerance = 1e-8, activeSet, inactiveRows=[], incrementalContext }) {
    // Optional force support can appear after an originally normal-only
    // workspace was created. Rebuild the reference band when it no longer
    // contains that support; compact workspaces are keyed by both supports.
    if(!w.primal||rows.some((r,index)=>r.extraForceDofs?.some(p=>
        w.dual[index]<w.band.starts[w.primal[p]]||w.dual[index]>w.band.ends[w.primal[p]]))) {
        const key=rowSupportKey(rows);
        if(w.fullReferenceKey!==key){w.fullReference=createSharedAxisLinear(chain.layout,rows);w.fullReferenceKey=key;}
        return solveSharedAxisLinearOnce(w.fullReference,chain,{rows,gradient,fixed,tolerance,activeSet,inactiveRows,incrementalContext});
    }
    const { matrix: A, residual: F, band: { starts, ends, offsets }, primal, dual } = w;
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
        if (!active || !movable) {
            if (active && (r.kind === 'length' ? Math.abs(r.gap) > tolerance : r.gap < -tolerance)) throw new RangeError('Incompatible fixed shared axis constraint');
            add(d, d, 1); F[d] = r.multiplier;
        } else {
            F[d] = r.gap;
            r.dofs.forEach((p, k) => add(d, primal[p], r.jacobian[k]));
        }
        if (r.geometricHessian) r.dofs.forEach((p, i) => r.dofs.forEach((q, j) => add(primal[p], primal[q], r.geometricHessian[i * r.dofs.length + j])));
    });
    for(const r of inactiveRows)if(r.geometricHessian)
        r.dofs.forEach((p,i)=>r.dofs.forEach((q,j)=>add(primal[p],primal[q],r.geometricHessian[i*r.dofs.length+j])));
    for (let i = 0; i < w.count; i++) {
        for (let j = starts[i]; j <= ends[i]; j++) if (w.fixed[i] || w.fixed[j]) A[offsets[i] + j] = i === j ? 1 : 0;
        if (w.fixed[i]) F[i] = 0;
        let maximum = 0;
        for (let j = starts[i]; j <= ends[i]; j++) maximum = Math.max(maximum, Math.abs(A[offsets[i] + j]));
        w.scales[i] = 1 / Math.sqrt(Math.max(maximum, 1e-30));
    }
    // Experimental factors belong to one immutable linearization. Their
    // memory stays exclusive while yielded; the ordinary arena is overwritten
    // by other solvers. Compact mode uses this LU as its border-update base.
    let lu=w.lu;
    if(incrementalContext) {
        lu=incrementalContext.workspaces.get(w);
        if(!lu){lu=createIncrementalContactLU(w.band,w.count,{maxRank:incrementalContext.maxRank});incrementalContext.workspaces.set(w,lu);incrementalContext.allDiagnostics.push(lu.diagnostics);}
    }
    const factorsBefore=incrementalContext?lu.diagnostics.factorizations:0;
    const solved = lu.solve(A, F, w.scales, 0, w.solution);
    // A rejected factorization does not write a valid solution. Do not report
    // a residual computed from the previous solve's scratch as this direction.
    if (!solved) {
        w.increment.fill(0); w.multiplierIncrement.fill(0);
        return {increment:w.increment,multiplierIncrement:w.multiplierIncrement,
            factorizations:incrementalContext?lu.diagnostics.factorizations-factorsBefore:1,residual:Infinity,converged:false,failure:'band-lu-rejected'};
    }
    for (let i = 0; i < w.count; i++) w.solution[i] *= w.scales[i];
    let residual = Infinity, factorizations = 1;
    // Certify the original unscaled equations, refining the solution when
    // material stiffness / a short edge makes forward error significant.
    for (let attempt = 0; attempt < 3; attempt++) {
        residual = 0;
        for (let i = 0; i < w.count; i++) {
            let v = F[i]; for (let j = starts[i]; j <= ends[i]; j++) v += A[offsets[i] + j] * w.solution[j];
            w.error[i] = v; residual = Math.max(residual, Math.abs(v));
        }
        if(residual>tolerance) {
            residual=0;
            for(let i=0;i<w.count;i++) {
                const v=compensatedResidual(w,i);w.error[i]=v;residual=Math.max(residual,Math.abs(v));
            }
        }
        if (residual <= tolerance || !solved || attempt === 2) break;
        factorizations++;
        if (!lu.solve(A, w.error, w.scales, 0, w.correction)) break;
        for (let i = 0; i < w.count; i++) w.solution[i] += w.correction[i] * w.scales[i];
    }
    primal.forEach((p, i) => { w.increment[i] = w.solution[p]; });
    dual.forEach((d, i) => { w.multiplierIncrement[i] = w.solution[d]; });
    return { increment: w.increment, multiplierIncrement: w.multiplierIncrement, factorizations:incrementalContext?lu.diagnostics.factorizations-factorsBefore:factorizations, residual,
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
    const indices=scratch?.indices??[],inactiveRows=scratch?.inactiveRows??[];
    if(!scratch)options.rows.forEach((r,i)=>{if(activeSet[i])indices.push(i);else inactiveRows.push(r);});
    const activeRows=scratch?.rows??indices.map(i=>options.rows[i]);
    // Recheck actual supports even with a reused index map: contact/friction
    // derivatives can change their sparsity without changing the active mask.
    const key=`${chain.layout.dofCount}/${chain.layout.band}/`+rowSupportKey(activeRows),cache=w.activeWorkspaces??=new Map();
    let packed=cache.get(key);
    if(!packed) {
        if(cache.size>=8)cache.delete(cache.keys().next().value);
        packed=createSharedAxisLinear(chain.layout,activeRows);cache.set(key,packed);
    }
    const gradient=scratch?.gradient??Float64Array.from(options.gradient);
    // Inactive equations are delta-lambda = -lambda. Eliminate their force
    // columns exactly, retaining the physical geometric Hessian at entry.
    for(const r of inactiveRows) {
        r.dofs.forEach((p,k)=>{gradient[p]-=(r.kind==='wall'?-1:1)*r.multiplier*r.jacobian[k];});
        r.extraForceDofs?.forEach((p,k)=>{gradient[p]-=r.multiplier*r.extraForceJacobian[k];});
    }
    const result=solveSharedAxisLinearOnce(packed,chain,{...options,gradient,inactiveRows,
        rows:activeRows,activeSet:packed.allActive??=new Uint8Array(indices.length).fill(1)});
    if(context&&result.converged) {
        const lu=context.workspaces.get(packed);
        context.bordered=createBorderedContactUpdates(packed,lu,indices,options.rows);
        context.borderDiagnostics.push(context.bordered.diagnostics);
    }
    w.increment.set(result.increment);
    options.rows.forEach((r,i)=>{w.multiplierIncrement[i]=-r.multiplier;});
    indices.forEach((index,i)=>{w.multiplierIncrement[index]=result.multiplierIncrement[i];});
    w.peakActiveEntries=Math.max(w.peakActiveEntries??0,packed.matrix.length);
    return {...result,increment:w.increment,multiplierIncrement:w.multiplierIncrement};
}


/** Solve the linearized unilateral problem, not equality rows followed by
 * clamping negative reactions. A released wall changes the global direction;
 * merely clamping its multiplier leaves an unbalanced force in that direction.
 */
function* iterateActiveSet(w, chain, options, batchSize, solutionCache) {
    const { rows, tolerance = 1e-8 } = options;
    const activeSet=Uint8Array.from(rows,r=>r.kind==='length'||r.multiplier>tolerance);
    let factorizations=0,result,activeSetAttempts=0,batchedRows=0,workingSetReuses=0;const visited=new Set(),dual=Float64Array.from(rows,r=>r.kind==='wall'?Math.max(0,r.multiplier):r.multiplier);
    const finish=extra=>({...result,...extra,factorizations,activeSetAttempts,batchedRows,workingSetReuses});
    for(let attempt=0;attempt<(options.maxActiveSetAttempts??Math.max(8,rows.length*2));attempt++) {
        yield {kind:'linear-active-set',attempt,batchSize};
        activeSetAttempts++;
        const prepared=prepareSharedAxisActiveBasis({rows,fixed:options.fixed,activeSet,dual,trace:options.trace,reuseStructure:options.reuseStructure,basisCache:options.basisCache});
        if(!prepared.converged)return finish({converged:false,failure:prepared.failure});
        const setKey=activeSet.join(''),signature=setKey+'/'+Array.from(dual,v=>v.toPrecision(9)).join(',');
        if(visited.has(signature))return finish({converged:false,failure:'active-set-cycle'});
        visited.add(signature);
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
            if(solutionCache&&result.converged) {
                if(solutionCache.size>=32)solutionCache.delete(solutionCache.keys().next().value);
                solutionCache.set(setKey,{...result,increment:result.increment.slice(),multiplierIncrement:result.multiplierIncrement.slice()});
            }
            factorizations+=result.factorizations;
        }
        if(!result.converged)return finish();
        let worst=-tolerance,change=-1,alpha=1,activate=null;
        // Move the feasible dual iterate toward the active-set solution only
        // as far as the FIRST multiplier reaching zero. Removing the most
        // negative target instead can cycle at nearly coplanar mesh features.
        for(let i=0;i<rows.length;i++) {
            if(rows[i].kind!=='wall'||!activeSet[i])continue;
            const target=rows[i].multiplier+result.multiplierIncrement[i];
            if(target < -tolerance) {
                const fraction=dual[i]/(dual[i]-target);
                if(fraction<alpha){alpha=fraction;change=i;worst=target;}
            }
        }
        if(change>=0) {
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
                const r=rows[i];if(r.kind!=='wall'||activeSet[i])continue;
                const residual=r.gap+r.dofs.reduce((sum,p,k)=>sum+r.jacobian[k]*result.increment[p],0);
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
    // This cache never survives the linear call: geometry, coefficients and
    // physical multipliers are fixed only for this one linearization. The
    // reference activation fallback solves the same equations and may reuse it.
    const solutionCache=options.reuseWorkingSet===false?null:new Map();
    // Only normalized Jacobian prefixes share this token. No cache survives
    // a different linearization (or changed fixed mask/geometry/coefficients).
    options={...options,...(options.incrementalContacts?{...(options.incrementalContacts==='full'?{compactWorkingSet:false}:{}),incrementalContext:{maxRank:options.incrementalMaxRank??8,workspaces:new Map(),borderDiagnostics:[],allDiagnostics:[]}}:{}),basisCache:options.reuseStructure===false?null:Symbol('linear-basis')};
    try {
        const first=yield* iterateActiveSet(w,chain,options,batchSize,solutionCache);
        const incrementalStats=()=>options.incrementalContext?{incrementalStats:structuredClone(options.incrementalContext.allDiagnostics),borderedStats:structuredClone(options.incrementalContext.borderDiagnostics)}:{};
        if(first.converged||batchSize===1)return {...first,...incrementalStats(),batchActivation:batchSize>1,batchFallback:false};
        // Restart from the original physical rows, reactions, gradient and Hessian.
        // Only scratch buffers have changed. The reference result may alias them,
        // so save scalar accounting before the second solve overwrites those views.
        const failedFactorizations=first.factorizations,failedAttempts=first.activeSetAttempts;
        options.trace?.push({kind:'batch-fallback',failure:first.failure});
        const reference=yield* iterateActiveSet(w,chain,options,1,solutionCache);
        return {...reference,...incrementalStats(),factorizations:failedFactorizations+reference.factorizations,
            activeSetAttempts:failedAttempts+reference.activeSetAttempts,workingSetReuses:first.workingSetReuses+reference.workingSetReuses,batchActivation:true,batchFallback:true,
            batchFailure:first.failure,batchFactorizations:failedFactorizations,referenceFactorizations:reference.factorizations,
            batchAttempts:failedAttempts,referenceAttempts:reference.activeSetAttempts};
    } finally {for(const lu of options.incrementalContext?.workspaces.values()??[])lu.dispose();}
}

/** Yield between active-set/LU attempts while preserving the synchronous API.
 * CPU timing excludes consumer pauses, including pauses before a fallback. */
export function* iterateSharedAxisLinear(w,chain,options) {
    options.observeLinearSystem?.({chain,options});
    options.rows.forEach(r=>validateExtraForce(r,chain.layout.dofCount));
    if(options.maxActiveSetAttempts!==undefined&&(!Number.isInteger(options.maxActiveSetAttempts)||options.maxActiveSetAttempts<1))throw new RangeError('Active-set attempt limit must be a positive integer');
    const batchSize=options.batchActivation===false?1:(options.batchActivationSize??8);
    if(!Number.isInteger(batchSize)||batchSize<1||batchSize>16)throw new RangeError('Batch activation size must be between 1 and 16');
    const iterator=iterateWithFallback(w,chain,options,batchSize);let cpuMs=0,done=false;
    try {
        while(true) {
            const start=performance.now();let next;
            try{next=iterator.next();}finally{cpuMs+=performance.now()-start;}
            if(next.done){done=true;return {...next.value,cpuMs};}
            yield next.value;
        }
    }finally{if(!done)iterator.return();}
}

export function solveSharedAxisLinear(w,chain,options) {
    const iterator=iterateSharedAxisLinear(w,chain,options);let next;
    do{next=iterator.next();}while(!next.done);return next.value;
}
