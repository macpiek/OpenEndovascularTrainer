// A failed unrestricted-equivalent Newton search cannot improve when the
// fallback restores exactly the same equations. Keep this snapshot only for
// the current nonlinear iteration, after a completed (not cancelled) solve.
const arrayKeys=['dofs','jacobian','extraForceDofs','extraForceJacobian','geometricHessian'];
const kindCode=kind=>kind==='length'?1:kind==='wall'?2:0;
const optionalLength=value=>value===undefined?-1:value===null?-2:value.length;
const optionalNumberTag=value=>value===undefined?0:value===null?1:2;
function sameArray(a,b) {
    if(a==null||b==null)return a===b;
    if(a.length!==b.length)return false;
    for(let i=0;i<a.length;i++)if(!Object.is(a[i],b[i]))return false;
    return true;
}
export function captureRejectedSharedAxisLinearization(chain,options,direction) {
    // A limited batch may have fallen back to a complete single-pivot search.
    // The later unrestricted batch must still be tried in that case.
    if(direction.converged||direction.failure==='active-set-limit'||direction.batchFailure==='active-set-limit'||
        options.modifiedNewtonContext||options.incrementalContacts)return null;
    let size=options.rows.length*(5+arrayKeys.length);
    for(const row of options.rows) {
        if(!kindCode(row.kind))return null;
        for(const key of arrayKeys)size+=Math.max(0,optionalLength(row[key]));
    }
    // One packed snapshot avoids thousands of temporary row/derivative
    // objects during a difficult step. Only original values are recorded;
    // there is no quantization, hash collision or tolerance comparison.
    const rowValues=new Float64Array(size);let at=0;
    for(const row of options.rows) {
        rowValues[at++]=kindCode(row.kind);rowValues[at++]=row.gap;rowValues[at++]=row.multiplier;
        rowValues[at++]=optionalNumberTag(row.compliance);rowValues[at++]=row.compliance??0;
        for(const key of arrayKeys) {
            const n=optionalLength(row[key]);rowValues[at++]=n;
            if(n>=0){rowValues.set(row[key],at);at+=n;}
        }
    }
    return {tangent:!!chain.tangent,dofCount:chain.layout.dofCount,band:chain.layout.band,
        matrix:(chain.tangent??chain.hessian).slice(),gradient:options.gradient.slice(),fixed:options.fixed.slice(),
        tolerance:options.tolerance,rowCount:options.rows.length,rowValues};
}
export function matchesRejectedSharedAxisLinearization(saved,chain,options) {
    if(!saved||saved.tangent!==!!chain.tangent||saved.dofCount!==chain.layout.dofCount||saved.band!==chain.layout.band||
        !Object.is(saved.tolerance,options.tolerance)||saved.rowCount!==options.rows.length||
        !sameArray(saved.matrix,chain.tangent??chain.hessian)||!sameArray(saved.gradient,options.gradient)||
        !sameArray(saved.fixed,options.fixed))return false;
    const values=saved.rowValues;let at=0;
    for(const row of options.rows) {
        if(values[at++]!==kindCode(row.kind)||!Object.is(values[at++],row.gap)||!Object.is(values[at++],row.multiplier)||
            values[at++]!==optionalNumberTag(row.compliance)||!Object.is(values[at++],row.compliance??0))return false;
        for(const key of arrayKeys) {
            const n=values[at++];if(n!==optionalLength(row[key]))return false;
            for(let i=0;i<n;i++)if(!Object.is(values[at++],row[key][i]))return false;
        }
    }
    return at===values.length;
}

// Private lease shared only by the nested fallbacks of one dynamic timestep.
// It never survives a committed step, a rejected step, or cancellation.
export const fallbackWorkingSetLease = Symbol('shared-axis-fallback-working-set-lease');
export function createSharedAxisFallbackWorkingSetCache() {
    const entries=[];
    // These flags only choose which working set is visited next. They do not
    // enter a fixed working set's KKT equations, solution or residual test.
    const searchKeys=new Set(['batchActivationSize','batchActivation','batchRelease',
        'zeroDualStart','simultaneousContactRelease','maxActiveSetAttempts',
        'recoverNegativeContact','deferActiveBasis']);
    const configuration=options=>{
        if(options.observeLinearSystem||options.trace||options.incrementalContacts||options.modifiedNewtonContext||
            options.compactWorkingSet===false||options.condenseCompliantContacts||options.primalCompliantContacts)return null;
        const values=[];
        for(const key of Object.keys(options).sort()) {
            if(key==='rows'||key==='gradient'||key==='fixed'||searchKeys.has(key))continue;
            const value=options[key];
            if(value!==null&&!['undefined','number','boolean','string'].includes(typeof value))return null;
            values.push(key,value);
        }
        return values;
    };
    return {hits:0,
        find(chain,options) {
            if(!entries.length)return null;
            const config=configuration(options);if(!config)return null;
            for(const entry of entries) {
                if(entry.layout!==chain.layout||!sameArray(config,entry.config)||
                    !sameArray(options.gradient,entry.snapshot.gradient)||
                    !matchesRejectedSharedAxisLinearization(entry.snapshot,chain,options))continue;
                this.hits++;
                return entry.solutions;
            }
            return null;
        },
        remember(chain,options,result,solutions) {
            if(!solutions?.size||entries.some(entry=>entry.solutions===solutions)||result.converged||
                !(Number.isFinite(result.residual)||result.failure==='band-lu-rejected'))return;
            const config=configuration(options);if(!config)return;
            const snapshot=captureRejectedSharedAxisLinearization(chain,options,result);if(!snapshot)return;
            if(entries.length===4)entries.shift();
            // iterateActiveSet already owns copies of each saved direction.
            // Retain that bounded map; never store mutable workspace outputs.
            entries.push({layout:chain.layout,config,snapshot,solutions});
        },
        clear() {entries.length=0;}
    };
}
