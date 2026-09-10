import assert from 'node:assert/strict';

/** Removing the catheter changes only its material/spin/mass arm. All spatial
 * nodes (including catheter-induced material/tip boundaries) are retained. */
export function prepareSingleVsPair(modules, insertion) {
    const begin=performance.now(),pair=modules.fixture.makeFixture(insertion,'contact-free'),pairSetupMs=performance.now()-begin;
    const wireData={...pair.mesh.data,tools:pair.mesh.data.tools.filter(tool=>tool.id==='wire')};
    const wireLayout=modules.chain.createCompositeChainLayout(pair.state.layout.edgeToolIds.map(()=>['wire']));
    const start=performance.now(),single={...pair,mesh:{...pair.mesh,data:wireData,layout:wireLayout},
        state:modules.time.createCompositeTimeStepState({data:wireData,layout:wireLayout})};
    const singleStateSetupMs=performance.now()-start;
    const cases={single,pair};
    for(const value of Object.values(cases))value.options={...modules.fixture.preparedOptions(value,value.state,'strict-tests'),
        constraintSolver:'mixed',inertiaBackend:'compiled'};
    const a=commonInputs(single),b=commonInputs(pair);assert.deepEqual(a,b,'common wire/position inputs must be identical');
    assert.ok(single.options.inertiaEdges.every(edge=>edge.tools.length===1&&edge.tools[0].id==='wire'));
    return {cases,pairSetupMs,singleStateSetupMs,commonInputs:a};
}
export function commonInputs(fixture) {
    const {state,options}=fixture,layout=state.layout;
    const positionBoundary=options.prescribed.flatMap(boundary=>Array.from(layout.positions,(dof,node)=>
        boundary.dof>=dof&&boundary.dof<dof+3?{node,axis:boundary.dof-dof,value:boundary.value}:null).filter(Boolean));
    const wire=state.data.tools.find(tool=>tool.id==='wire');
    const wireBoundary=options.prescribed.flatMap(boundary=>Array.from(layout.spins.get('wire'),(dof,edge)=>
        dof===boundary.dof?{edge,value:boundary.value}:null).filter(Boolean));
    return {coordinates:Array.from(state.data.coordinates),positions:state.data.positions,reference:state.data.reference,
        angles:Array.from(wire.angles),referenceTwists:Array.from(wire.referenceTwists??[]),positionBoundary,wireBoundary,
        positionalLoads:Array.from(layout.positions,dof=>Array.from(options.loads.slice(dof,dof+3))),
        wireInertia:options.inertiaEdges.map(edge=>edge.tools.find(tool=>tool.id==='wire')),
        dt:options.dt,tolerances:options.tolerances,budget:options.budget};
}

/** Reuse all operator/direction buffers. This is one linearization and mixed
 * direction, not a nonlinear admission. Material integration was prepared by
 * the fixture; every local strain, inertia and original length row is still
 * evaluated on each pass. */
export function createSingleVsPairOperator(modules,fixture,elementBackend) {
    const {state,options}=fixture,layout=state.layout,creationStart=performance.now();
    const chain=modules.chain.createCompositeChainWorkspace(layout,{elementBackend});
    const elasticWorkspaceMs=performance.now()-creationStart,inertiaStart=performance.now();
    const inertia=modules.inertia.createCompositeInertiaCache({layout,coordinates:state.data.coordinates,previousPositions:state.data.positions,
        dt:options.dt,inertiaEdges:options.inertiaEdges});
    const inertiaCompilationMs=performance.now()-inertiaStart,otherStart=performance.now();
    const length=modules.length.createCompositeLengthConstraintWorkspace(layout);
    const definitions=Array.from({length:layout.nodeCount-1},(_,edge)=>({kind:'length',edge,
        dofs:[layout.positions[edge],layout.positions[edge+1]].flatMap(dof=>[dof,dof+1,dof+2])}));
    const mixed=modules.mixed.createCompositeMixedWorkspace(layout,definitions),fixed=new Uint8Array(layout.dofCount);
    options.prescribed.forEach(boundary=>{fixed[boundary.dof]=1;});
    const rows=definitions.map(definition=>({...definition,jacobian:new Float64Array(6),geometricHessian:new Float64Array(36),gap:0,multiplier:0}));
    const setup={elasticWorkspaceMs,inertiaCompilationMs,lengthMixedWorkspaceMs:performance.now()-otherStart};
    const metadata={nodes:layout.nodeCount,positionDofs:3*layout.nodeCount,spinDofs:layout.dofCount-3*layout.nodeCount,
        catheterSpinDofs:Array.from(layout.spins.get('catheter')??[]).filter(v=>v>=0).length,
        unknowns:layout.dofCount,band:layout.band,mixedUnknowns:mixed.count,mixedHalfBandwidth:mixed.packedLayout.kl,
        mixedMatrixEntries:mixed.matrix.length,localToolCounts:layout.hinges.reduce((out,hinge)=>(out[hinge.tools.length]++,out),{1:0,2:0})};
    function run() {
        const start=performance.now();modules.chain.assembleCompositeChain(state.data,chain);const elasticEnd=performance.now();
        inertia.append(state.data.positions,chain);const inertiaEnd=performance.now();
        for(let dof=0;dof<layout.dofCount;dof++)chain.gradient[dof]-=options.loads[dof];
        const physical=modules.length.evaluateCompositeLengthConstraints({positions:state.data.positions,coordinates:state.data.coordinates,
            multipliers:state.lengthMultipliers,tolerance:options.tolerances.length},length);
        for(let dof=0;dof<layout.dofCount;dof++)chain.gradient[dof]-=physical.constraintForces[dof];
        rows.forEach((row,edge)=>{row.jacobian.set(physical.jacobian.subarray(6*edge,6*edge+6));row.gap=physical.residuals[edge];row.multiplier=state.lengthMultipliers[edge];
            assert.equal(row.multiplier,0,'This initial-state probe must not omit a nonzero stress tangent');});
        const assemblyEnd=performance.now();
        const direction=modules.mixed.solveCompositeMixedDirection(mixed,chain,{rows,gradient:chain.gradient,fixed,
            tolerances:{force:options.tolerances.linear,torque:options.tolerances.linear,constraint:options.tolerances.length*.05},maxCorrections:1});
        const end=performance.now();
        return {elasticMs:elasticEnd-start,inertiaMs:inertiaEnd-elasticEnd,lengthRowsMs:assemblyEnd-inertiaEnd,
            operatorMs:assemblyEnd-start,directionMs:end-assemblyEnd,totalMs:end-start,converged:direction.converged,
            backsolves:direction.linearSolves,factorizations:direction.factorizations,proof:direction.proof};
    }
    return {setup,metadata,run};
}
