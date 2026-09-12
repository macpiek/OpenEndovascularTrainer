import assert from 'node:assert/strict';
import test from 'node:test';
import {createCompositeChainLayout} from '../src/physics/kirchhoffCompositeChain.js';
import {captureCompositeReferenceFrames,compileCompositeMaterial} from '../src/physics/kirchhoffCompositeElement.js';
import {createCompositeJointTimeStepState,createCompositeJointTimeStepWorkspace,advanceCompositeJointTimeStep} from '../src/physics/kirchhoffCompositeJointTimeStep.js';
import {createCompositeJointAssembly,createCompositeJointAssemblyWorkspace} from '../src/physics/kirchhoffCompositeJointAssembly.js';
import {createCompositeRelativeDirectionWorkspace} from '../src/physics/kirchhoffCompositeRelativeDirection.js';
import {createCompositeRelativeClusterStructure,assembleCompositeRelativeCluster} from '../src/physics/kirchhoffCompositeRelativeCluster.js';
import {createCompositeJointWallRows} from '../src/physics/kirchhoffCompositeJointWallRows.js';

const close=(a,b,t=1e-9)=>assert.ok(Number.isFinite(a)&&Number.isFinite(b)&&Math.abs(a-b)<=t,`${a} != ${b} (${t})`);
const vector=(a,b,t)=>{assert.equal(a.length,b.length);a.forEach((v,i)=>close(v,b[i],t));};
const accepted=r=>assert.equal(r.accepted,true,JSON.stringify({status:r.status,error:r.error,diagnostics:r.diagnostics}));
function fixture(id,{n=5,dsDx=1.125,y=0,EI=.8,GJ=.4}={}) {
    const layout=createCompositeChainLayout(Array.from({length:n-1},()=>[id])),coordinates=Array.from({length:n},(_,i)=>2*i);
    const positions=coordinates.map(x=>[dsDx*x,y,0]),tools=[{id,dsDx,reference:captureCompositeReferenceFrames(positions),
        referenceTwists:new Float64Array(n-2),material:compileCompositeMaterial({EI1:EI,EI2:1.5*EI,GJ})}];
    // The default relativeToolId must resolve to the sole actual material.
    return createCompositeJointTimeStepState({layout,coordinates,positions,modes:[],relative:new Float64Array(0),tools,
        angles:new Map([[id,new Float64Array(n-1)]]),restLengths:new Map([[id,new Float64Array(n-1).fill(2*dsDx)]]),
        materialCoordinate:'reference-arclength'});
}
function inertia(state,velocity, density=.13) {
    const id=state.tools[0].id,dsDx=state.tools[0].dsDx;
    return {previousPositions:structuredClone(state.toolPositions),inertiaEdges:state.layout.edgeToolIds.map((_,e)=>({tools:[{id,
        massPerMaterialLength:density,materialMap:{sStart:20+state.coordinates[e]*dsDx,dsDx,dsDt:0},
        oldMaterialVelocities:velocity?[Array.from(velocity),Array.from(velocity)]:structuredClone(state.materialVelocities?.[e].tools[0].velocities??[[0,0,0],[0,0,0]])}]}))};
}
function config(state,velocity,extra={}) {
    return {dt:.06,torsionMode:'quasi-static',contacts:'none',inertia:inertia(state,velocity),
        boundaries:{positions:[],spins:[{toolId:state.tools[0].id,edge:0,value:0}]},...extra};
}
function physical(r) {
    accepted(r);const state=r.state,id=state.tools[0].id,p=state.toolPositions.get(id);
    assert.equal(state.tools.length,1);assert.equal(state.layout.spins.size,1);assert.equal(state.toolPositions.size,1);
    assert.equal(state.relativeToolId,id);assert.deepEqual(state.modes,[]);assert.deepEqual(state.relative,new Float64Array(0));
    assert.equal(r.diagnostics.relativeDofs,0);assert.equal(r.diagnostics.originalLengthRows,p.length-1);
    assert.equal(r.perTool.size,1);assert.equal(r.balances.size,1);
    p.forEach(v=>assert.ok(v.every(Number.isFinite)));
    for(let e=0;e<p.length-1;e++)close(Math.hypot(...p[e+1].map((v,k)=>v-p[e][k])),state.restLengths.get(id)[e],1e-8);
    const b=r.balances.get(id);vector(b.momentumRate,b.appliedForce.map((v,k)=>v+b.boundaryForce[k]+(b.contactForce?.[k]??0)),4e-7);
    assert.ok(r.diagnostics.certificate.converged);assert.ok(r.diagnostics.certificate.torque<=1e-8);
}
function samePhysical(a,b) {
    for(const key of ['accepted','status','state','perTool','boundaryForces','spinReactions','balances','contactForces'])assert.deepEqual(a[key],b[key],key);
    for(const key of ['certificate','directions','evaluations','linearSolves','acceptedAlphas','unknowns'])assert.deepEqual(a.diagnostics[key],b.diagnostics[key],key);
}
function plane(state) {
    const id=state.tools[0].id,field={queryCapsuleCoordinates(ax,ay,az,bx,by,bz,radius,out) {
        const t=ay===by?.5:ay<by?0:1;out.signedDistance=(1-t)*ay+t*by;out.signedGap=out.signedDistance-radius;
        out.segmentT=t;out.inward.values.set([0,1,0]);out.closestPoint.values.set([(1-t)*ax+t*bx,0,(1-t)*az+t*bz]);
        out.faceIndex=1;out.branchId=0;out.source='analytic-plane';out.capsuleSampleCount=2;return out;
    }};
    return {mode:'wall-normal',contactMode:'envelope',friction:'none',chartId:`single-${id}-plane`,field,forcePerLength:1,
        contactOwners:{edges:state.layout.edgeToolIds.map((_,edge)=>({edge,wall:{owner:id,radius:.8}}))}};
}

for(const id of ['wire','catheter']) {
    test(`${id}: explicit empty cluster has zero support/energy and a common-only direction`,()=>{
        const state=fixture(id),prepared=inertia(state,[.02,-.03,.04]);
        const args={...state,inertia:{...prepared,dt:.06}},cache=createCompositeJointAssemblyWorkspace({...state,relativeToolId:undefined});
        const a=createCompositeJointAssembly({...args,relativeToolId:undefined}),b=createCompositeJointAssembly({...args,relativeToolId:undefined},{workspace:cache});
        const ao=a.evaluate(state),bo=b.evaluate(state),c=ao.cluster;
        assert.equal(c.relativeRepresentation,'none-single-material');assert.equal(c.relative.dofCount,0);assert.equal(c.relativeDofCount,0);
        assert.equal(c.common.dofs.length,0);assert.deepEqual(c.coupling.rowOffsets,Int32Array.of(0));
        for(const v of [c.common.gradient,c.common.hessian,c.relative.gradient,c.relative.hessian,c.coupling.values,c.coupling.columns,c.coupling.commonDofs])assert.equal(v.length,0);
        assert.equal(c.energy,0);assert.equal(c.elasticEnergy,0);assert.equal(c.inertialEnergy,0);
        assert.deepEqual(c.modes,[]);assert.deepEqual(c.stencils,[]);assert.deepEqual(c.affectedHinges,[]);assert.deepEqual(c.affectedInertiaEdges,[]);
        assert.equal(ao.statistics.materialHinges,state.layout.nodeCount-2);assert.equal(ao.statistics.sharedGeometryHinges,0);
        assert.equal(ao.statistics.materialInertiaEdges,state.layout.nodeCount-1);assert.equal(ao.perTool.size,1);
        close(ao.energy,ao.perTool.get(id).elasticEnergy+ao.perTool.get(id).inertialEnergy,0);
        assert.deepEqual(ao.chain.gradient,bo.chain.gradient);assert.deepEqual(ao.chain.hessian,bo.chain.hessian);
        assert.ok(ao.chain.hessian.every(Number.isFinite));assert.ok(ao.chain.gradient.every(Number.isFinite));
        const direction=createCompositeRelativeDirectionWorkspace(a.layout,c);
        assert.equal(direction.count,state.layout.dofCount);assert.equal(direction.relativeIncrement.length,0);assert.equal(direction.couplingScatter.length,0);
        const symbolic={layout:state.layout,modes:[],data:{positions:state.positions,coordinates:state.coordinates,reference:state.tools[0].reference,tools:state.tools},elementBackend:'wasm-exact'};
        const empty=createCompositeRelativeClusterStructure(symbolic),assembled=assembleCompositeRelativeCluster(symbolic);
        assert.equal(empty.operatorReady,false);assert.equal(assembled.operatorReady,true);assert.equal(assembled.energy,0);
        assert.equal(assembled.common.dofs.length,0);assert.equal(assembled.relative.dofCount,0);
        a.evaluate(state,{order:'gradient'});assert.equal(c.hessianValid,false);
        assert.throws(()=>createCompositeJointAssembly({...args,modes:[{node:1,basis:[[1,0,0],[0,1,0],[0,0,1]]}]}),/complete three-coordinate modes/);
    });

    test(`${id}: free translation preserves original momentum/lengths for two dt, cold/reuse and late retry`,()=>{
        const state=fixture(id),v=[.09,-.03,.025],before=structuredClone(state),options=config(state,v),workspace=createCompositeJointTimeStepWorkspace({...state,relativeToolId:undefined});
        const cold=advanceCompositeJointTimeStep(state,options),first=advanceCompositeJointTimeStep(state,{...options,workspace});physical(first);samePhysical(first,cold);
        assert.deepEqual(state,before);const mass=.13*8*1.125;
        close(first.perTool.get(id).mass,mass,1e-14);vector(first.perTool.get(id).momentum,v.map(v=>v*mass),1e-9);
        state.positions.forEach((p,i)=>vector(first.state.positions[i],p.map((x,k)=>x+options.dt*v[k]),2e-10));
        assert.equal(first.diagnostics.unknowns,state.layout.dofCount+state.positions.length-1);
        const late=advanceCompositeJointTimeStep(state,{...options,workspace,budget:{evaluations:first.diagnostics.evaluations-1}});
        assert.equal(late.accepted,false);assert.equal(late.status,'evaluation-budget');assert.equal(late.state,state);assert.deepEqual(state,before);
        const retry=advanceCompositeJointTimeStep(state,{...options,workspace});samePhysical(retry,first);
        const secondOptions=config(first.state),second=advanceCompositeJointTimeStep(first.state,{...secondOptions,workspace});physical(second);
        samePhysical(second,advanceCompositeJointTimeStep(first.state,secondOptions));
        assert.equal(second.state.step,2);close(second.state.time,2*options.dt,0);
        state.positions.forEach((p,i)=>vector(second.state.positions[i],p.map((x,k)=>x+2*options.dt*v[k]),5e-10));
        second.state.materialVelocities.forEach(e=>{assert.equal(e.tools.length,1);e.tools[0].velocities.forEach(w=>vector(w,v,1e-8));});
        const saved=structuredClone(first.state);second.state.positions[0][0]+=1;assert.deepEqual(first.state,saved);
    });

    test(`${id}: bending, physical position BC, independent spin and length equations use the same solve`,()=>{
        const state=fixture(id),before=structuredClone(state),last=state.positions.length-1;
        const boundaries={positions:[0,1].map(node=>({toolId:id,node,value:state.positions[node].slice()})),spins:[{toolId:id,edge:0,value:.4}]};
        const options=config(state,[0,0,0],{boundaries,loads:{forces:[{toolId:id,node:last,value:[0,.05,.015]}],torques:[{toolId:id,edge:last-1,value:.01}]}});
        const workspace=createCompositeJointTimeStepWorkspace(state),r=advanceCompositeJointTimeStep(state,{...options,workspace});physical(r);
        samePhysical(r,advanceCompositeJointTimeStep(state,options));assert.deepEqual(state,before);
        assert.deepEqual(r.state.positions[0],state.positions[0]);assert.deepEqual(r.state.positions[1],state.positions[1]);
        assert.ok(r.state.positions[last][1]>1e-5);assert.ok(r.perTool.get(id).elasticEnergy>0);
        close(r.state.angles.get(id)[0],.4,0);assert.ok(r.state.angles.get(id).every(Number.isFinite));
        assert.deepEqual(r.diagnostics.suppressedPrescribedLengthRows,[0]);assert.equal(r.state.lengthMultipliers[0],0);
        assert.ok(Math.hypot(...r.balances.get(id).boundaryForce)>0);
        const failed=advanceCompositeJointTimeStep(state,{...options,workspace,budget:{directions:0}});
        assert.equal(failed.accepted,false);assert.equal(failed.state,state);assert.deepEqual(state,before);
        samePhysical(advanceCompositeJointTimeStep(state,{...options,workspace}),r);
    });

    test(`${id}: normal wall envelope retains actual ownership, zero relative columns, hold/release and retry`,()=>{
        const state=fixture(id,{n:3,dsDx:1,y:.82,EI:.001,GJ:.001}),wall=plane(state),before=structuredClone(state),dt=1/120;
        const options=config(state,[0,-10,0],{dt,wall,inertia:inertia(state,[0,-10,0],.01)}),workspace=createCompositeJointTimeStepWorkspace(state);
        const rows=createCompositeJointWallRows({...state,relativeToolId:undefined,wall,tolerances:{force:1e-7,wallGap:1e-8,wallNcp:1e-8,wallWork:1e-9,linearConstraint:5e-11}});
        assert.ok(rows.rows.every(r=>r.owner===id&&r.relativeDofs.length===0&&r.forceColumn.length===6));
        const r=advanceCompositeJointTimeStep(state,{...options,workspace});physical(r);samePhysical(r,advanceCompositeJointTimeStep(state,options));assert.deepEqual(state,before);
        r.state.positions.forEach(p=>close(p[1],.8));
        assert.equal(r.contactForces.size,1);assert.equal(r.diagnostics.certificate.wall.contactMode,'envelope');
        assert.equal(r.diagnostics.certificate.wall.samples.length,6);assert.equal(r.diagnostics.certificate.wall.retainedRows,3);
        assert.equal(r.diagnostics.wallQueries,5*r.diagnostics.evaluations);
        close(r.balances.get(id).contactForce[1],.01*4*(10-.02/dt)/dt,1e-7);
        assert.ok(r.state.wallContactState.normalForces.every(Fn=>Fn>=0));
        assert.ok(r.diagnostics.certificate.wall.samples.every(s=>s.owner===id&&(s.role!=='capsule'||s.Fn===0)));
        const late=advanceCompositeJointTimeStep(state,{...options,workspace,budget:{evaluations:r.diagnostics.evaluations-1}});
        assert.equal(late.accepted,false);assert.equal(late.state,state);assert.deepEqual(state,before);
        const budget=advanceCompositeJointTimeStep(state,{...options,workspace,budget:{contactQueries:1}});
        assert.equal(budget.status,'contact-query-budget');assert.equal(budget.state,state);assert.deepEqual(state,before);
        samePhysical(advanceCompositeJointTimeStep(state,{...options,workspace}),r);
        const holdOptions=config(r.state,[0,0,0],{dt,wall,inertia:inertia(r.state,[0,0,0],.01)});
        const hold=advanceCompositeJointTimeStep(r.state,{...holdOptions,workspace});physical(hold);samePhysical(hold,advanceCompositeJointTimeStep(r.state,holdOptions));
        hold.state.positions.forEach(p=>close(p[1],.8));assert.equal(hold.state.step,2);
        const release=advanceCompositeJointTimeStep(hold.state,config(hold.state,[0,1,0],{dt,wall,inertia:inertia(hold.state,[0,1,0],.01),workspace}));physical(release);
        release.state.positions.forEach(p=>close(p[1],.8+dt));assert.ok(release.state.wallContactState.normalForces.every(Fn=>Fn===0));
    });
}

test('single-material admission never weakens full two-material modes or permits a fictional owner',()=>{
    const single=fixture('catheter');
    assert.throws(()=>createCompositeJointTimeStepState({...single,relativeToolId:'wire'}),/actual materials/);
    assert.throws(()=>createCompositeJointTimeStepState({...single,relative:Float64Array.of(0)}),/relative coordinates/);
    assert.throws(()=>createCompositeJointTimeStepState({...single,tools:[...single.tools,{...single.tools[0],id:'wire'}]}),/Exactly one/);
    const layout=createCompositeChainLayout(Array.from({length:4},()=>['wire','catheter']));
    assert.throws(()=>createCompositeJointTimeStepState({...single,layout,relativeToolId:'wire'}),/EVERY overlap/);
    const empty=createCompositeJointAssembly({...single,inertia:null}).cluster;
    assert.throws(()=>createCompositeRelativeDirectionWorkspace(layout,empty),/empty single-material/);
    const wall=plane(single);wall.contactOwners.edges[0].wall.owner='wire';
    assert.throws(()=>createCompositeJointWallRows({...single,wall}),/occupy its physical edge/);
});
