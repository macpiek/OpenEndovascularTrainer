import {solveKirchhoffCoupledSystem} from './kirchhoffCoupledSystem.js';
import {assembleKirchhoffAxialReducedSystem} from './kirchhoffAxialReducedSystem.js';
import {solveCoulombNewton} from './kirchhoffCoulombNewtonSolver.js';

/** Same world-facing contract and application gate as the established solve.
 * Geometry, friction history, trust-region scaling and multiplier application
 * remain owned by the existing coupled step. Only its linear direction changes.
 */
export function solveKirchhoffAxialCoupledSystem(constraint,dt=1/120,options={}) {
    if(options.mobilityAudit)throw new TypeError('Axial solve provides its original mobility certificate directly');
    return solveKirchhoffCoupledSystem(constraint,dt,{...options,jacobianOnly:true,includeAxialLayout:true,
        assemblySolver:solveAxialAssembly});
}
function solveAxialAssembly(system,options) {
    const assemblyStart=performance.now();
    const reduced=assembleKirchhoffAxialReducedSystem(system,{matrixFormat:'general-band',sectionSpan:options.sectionSpan,
        sectionScope:options.sectionScope});
    const assemblyMs=performance.now()-assemblyStart,newtonStart=performance.now();
    const tolerance=options.tolerance??1e-8;
    const linearOptions={...options,matrixFormat:'general-band',localSections:reduced.localSections,
            sectionWorkspace:options.workspace.axialSections??={},
            // The old seed is indexed by dual material/contact rows, not mixed
            // primal and reaction coordinates. Never reinterpret those values.
            initialIncrement:undefined,initialFree:undefined,
            // Small mixed residuals can be amplified during material-force
            // recovery. Continue Newton until the original equations pass.
            acceptCandidate:increment=>reduced.recover(increment).originalKkt.maximumResidual<=tolerance};
    const run=(normalMap,initialIncrement)=>solveCoulombNewton(reduced.matrix,reduced.rhs,reduced.lower,reduced.upper,
        reduced.count,Math.max(reduced.diagnostics.lowerBandwidth,reduced.diagnostics.upperBandwidth)+1,
        reduced.groups,{...linearOptions,normalMap,initialIncrement});
    let solved=run(options.normalMap??'fischer-burmeister');
    if(!solved.diagnostics.converged&&!options.normalMap) {
        const first=solved;
        solved=run('projection',first.increment);
        for(const key of ['iterations','factorizations','backtracks','gradientFallbacks','boundRecoveries',
            'sectionSolves','sectionFallbacks','boundaryAllocations','boundaryReuses','localFactorizations','localFactorReuses','localResponseSolves','localResponseReuses','linearRefinementSteps','linearResidualFailures',
            'sectionTopologyMs','localPackingMs','localFactorMs','localResponsesMs','localReactionsMs','sectionAssemblyMs','boundaryPreparationMs','boundarySolveMs','sectionRecoveryMs'])
            if(key in first.diagnostics)solved.diagnostics[key]=(solved.diagnostics[key]??0)+first.diagnostics[key];
        solved.diagnostics.maximumLinearBackwardError=Math.max(solved.diagnostics.maximumLinearBackwardError??0,
            first.diagnostics.maximumLinearBackwardError??0);
        solved.diagnostics.normalMapFallback=true;
    }
    const newtonMs=performance.now()-newtonStart,recoveryStart=performance.now();
    const recovered=reduced.recover(solved.increment),increment=recovered.increment;
    const allGroups=structuredClone(system.groups).map(group=>({...group,radii:group.normalRow==null?group.radii:
        group.mu.map(mu=>mu*Math.max(0,group.normalLambda+increment[group.normalRow]))}));
    const lower=system.lower.slice(),upper=system.upper.slice();
    for(const group of allGroups)if(group.radii.some(radius=>radius===0))group.rows.forEach((row,axis)=>{
        lower[row]=-group.radii[axis]-group.lambda[axis];upper[row]=group.radii[axis]-group.lambda[axis];
    });
    const originalPassed=recovered.originalKkt.maximumResidual<=tolerance;
    return {increment,residual:recovered.residual,lower,upper,allGroups,
        groups:allGroups.filter(group=>group.radii.every(radius=>radius>0)),
        diagnostics:{...solved.diagnostics,...reduced.diagnostics,axialReduction:true,
            axialAssemblyMs:assemblyMs,axialNewtonMs:newtonMs,axialRecoveryMs:performance.now()-recoveryStart,
            originalRowCount:system.count,originalMobilityError:recovered.maximumMobilityError,
            converged:solved.diagnostics.converged&&originalPassed,
            status:solved.diagnostics.converged&&!originalPassed?'axial-original-residual':solved.diagnostics.status,
            maximumResidual:Math.max(solved.diagnostics.maximumResidual,recovered.originalKkt.maximumResidual)}};
}
