import {createCoulombBandLayout,createCoulombBandLU} from './kirchhoffCoulombBandLU.js';
import {condenseKirchhoffSections} from './kirchhoffSectionCondensation.js';

/** Local Newton interiors are eliminated before the global boundary solve.
 * Scaling and numerical shift are identical to the full band solve. Singular
 * local blocks fall back to that solve, preserving all original equations.
 */
export function createCoulombSectionLU(layout,count,sections,workspace={}) {
    const owned=sections.map(section=>Array.from(section));
    let fallback,boundary;
    const values=new Float64Array(layout.entries),rhs=new Float64Array(count);
    const diagnostics={linearSolver:'section-band-lu',sectionSolves:0,sectionFallbacks:0,
        globalRowCount:count,localRowCount:0,maximumLinearBackwardError:0,
        boundaryAllocations:0,boundaryReuses:0,localFactorizations:0,localFactorReuses:0,localResponseSolves:0,localResponseReuses:0,
        linearRefinementSteps:0,linearResidualFailures:0,maximumAcceptedLinearBackwardError:0,
        sectionTopologyMs:0,localPackingMs:0,localFactorMs:0,localResponsesMs:0,localReactionsMs:0,
        sectionAssemblyMs:0,boundaryPreparationMs:0,boundarySolveMs:0,sectionRecoveryMs:0};
    const matrix={values,starts:layout.starts,ends:layout.ends,offsets:layout.offsets};
    const residualRhs=new Float64Array(count);
    function solveCondensed(load) {
        const assemblyStart=performance.now();
        const reduced=condenseKirchhoffSections({matrix,rhs:load,sections:owned,workspace});
        diagnostics.sectionAssemblyMs+=performance.now()-assemblyStart;
        const preparationStart=performance.now();
        diagnostics.globalRowCount=reduced.count;diagnostics.localRowCount=count-reduced.count;
        const smallLayout=createCoulombBandLayout(reduced.matrix,reduced.count,reduced.count,[],'general-band');
        for(const key of ['sectionTopologyMs', 'localPackingMs', 'localFactorMs', 'localResponsesMs', 'localReactionsMs'])diagnostics[key]+=reduced.diagnostics[key];
        diagnostics.localFactorizations+=reduced.diagnostics.localFactorizations;
        diagnostics.localFactorReuses+=reduced.diagnostics.localFactorReuses;
        diagnostics.localResponseSolves+=reduced.diagnostics.localResponseSolves;
        diagnostics.localResponseReuses+=reduced.diagnostics.localResponseReuses;
        const reuse=boundary?.count===reduced.count&&smallLayout.starts.every((v,i)=>v===boundary.layout.starts[i])
            &&smallLayout.ends.every((v,i)=>v===boundary.layout.ends[i]);
        if(reuse)diagnostics.boundaryReuses++;
        else {
            boundary={count:reduced.count,layout:smallLayout,A:new Float64Array(smallLayout.entries),
                x:new Float64Array(reduced.count),F:new Float64Array(reduced.count),scales:new Float64Array(reduced.count).fill(1),
                solver:reduced.count?createCoulombBandLU(smallLayout,reduced.count):null};
            diagnostics.boundaryAllocations++;
        }
        // Repack into the solver's exact envelopes (zero cancellations may
        // narrow them relative to the assembly).
        const {A,x}=boundary;
        for(let i=0;i<reduced.count;i++)for(let j=smallLayout.starts[i];j<=smallLayout.ends[i];j++)
            A[smallLayout.offsets[i]+j]=reduced.matrix.values[reduced.matrix.offsets[i]+j];
        diagnostics.boundaryPreparationMs+=performance.now()-preparationStart;
        if(reduced.count) {
            for(let i=0;i<reduced.count;i++)boundary.F[i]=-reduced.rhs[i];
            const solveStart=performance.now(),ok=boundary.solver.solve(A,boundary.F,boundary.scales,0,x);
            diagnostics.boundarySolveMs+=performance.now()-solveStart;
            if(!ok)return null;
        }
        const recoveryStart=performance.now(),full=reduced.recover(x);
        diagnostics.sectionRecoveryMs+=performance.now()-recoveryStart;
        return full;
    }
    return {diagnostics,solve(J,F,scales,shift,direction) {
        for(let i=0;i<count;i++) {
            rhs[i]=-F[i]*scales[i];
            for(let j=layout.starts[i];j<=layout.ends[i];j++)values[layout.offsets[i]+j]=J[layout.offsets[i]+j]*scales[i]*scales[j];
            values[layout.offsets[i]+i]+=shift;
        }
        try {
            const full=solveCondensed(rhs);
            if(!full)return false;
            for(let refinement=0;refinement<=3;refinement++) {
                let error=0,anorm=0,bnorm=0,xnorm=0;
                for(let i=0;i<count;i++) {
                    let r=-rhs[i],norm=0;
                    for(let j=layout.starts[i];j<=layout.ends[i];j++) {const a=values[layout.offsets[i]+j];r+=a*full[j];norm+=Math.abs(a);}
                    residualRhs[i]=-r;
                    error=Math.max(error,Math.abs(r));anorm=Math.max(anorm,norm);bnorm=Math.max(bnorm,Math.abs(rhs[i]));xnorm=Math.max(xnorm,Math.abs(full[i]));
                }
                const backward=error/Math.max(Number.MIN_VALUE,anorm*xnorm+bnorm);
                diagnostics.maximumLinearBackwardError=Math.max(diagnostics.maximumLinearBackwardError,backward);
                if(Number.isFinite(backward)&&backward<=64*Math.max(1,count)*Number.EPSILON) {
                    direction.set(full);diagnostics.sectionSolves++;
                    diagnostics.maximumAcceptedLinearBackwardError=Math.max(diagnostics.maximumAcceptedLinearBackwardError,backward);
                    return true;
                }
                if(refinement===3||!Number.isFinite(backward))break;
                const correction=solveCondensed(residualRhs);
                if(!correction)break;
                for(let i=0;i<count;i++)full[i]+=correction[i];
                diagnostics.linearRefinementSteps++;
            }
            diagnostics.linearResidualFailures++;return false;
        } catch(error) {
            if(!(error instanceof RangeError)||!/Singular section interior/.test(error.message))throw error;
            diagnostics.sectionFallbacks++;fallback??=createCoulombBandLU(layout,count);
            return fallback.solve(J,F,scales,shift,direction);
        }
    }};
}
