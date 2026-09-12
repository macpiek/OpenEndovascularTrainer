import assert from 'node:assert/strict';
import test from 'node:test';
import {captureCompositeReferenceFrames,compileCompositeMaterial} from '../src/physics/kirchhoffCompositeElement.js';
import {createCompositeChainLayout,createCompositeChainWorkspace,assembleCompositeChain} from '../src/physics/kirchhoffCompositeChain.js';
import {assembleCompositeRelativePatch,solveCompositeRelativePatch} from '../src/physics/kirchhoffCompositeRelativePatch.js';

const close=(a,b,t=1e-8)=>assert.ok(Math.abs(a-b)<=t*(1+Math.max(Math.abs(a),Math.abs(b))),`${a} != ${b}`);
function fixture(count=9) {
    const positions=Array.from({length:count},(_,i)=>[2*i,0,0]),coordinates=positions.map(p=>p[0]);
    const layout=createCompositeChainLayout(Array.from({length:count-1},()=>['wire','catheter']));
    const data={positions,coordinates,reference:captureCompositeReferenceFrames(positions),tools:[
        {id:'wire',angles:new Float64Array(count-1),material:compileCompositeMaterial({EI1:2,EI2:7,GJ:3})},
        {id:'catheter',angles:new Float64Array(count-1),material:compileCompositeMaterial({EI1:20,GJ:8})}]};
    return {data,layout,node:4,basis:[[0,1,0],[0,0,1]]};
}

test('one relative mode includes all three incident hinges and has the analytic discrete bending stiffness',()=>{
    const f=fixture(),patch=assembleCompositeRelativePatch(f);
    assert.deepEqual(patch.affectedHinges,[3,4,5]);
    assert.equal(patch.dofs.length,19,'five common positions and four affected wire spins');
    close(patch.modeStiffness[0],6*7/2**3);close(patch.modeStiffness[1],0);close(patch.modeStiffness[2],6*2/2**3);
    assert.deepEqual([...patch.modeGradient],[0,0]);
    const r=solveCompositeRelativePatch({patch,clearance:.0405});
    assert.ok(r.converged);assert.equal(r.cell.normalForce,0);assert.equal(r.certified,false);
});

test('relative force is the independent derivative of the complete wire energy, without adding catheter stiffness',()=>{
    const f=fixture(),node=f.node;
    f.data.positions[node][1]=.06;f.data.positions[node+1][2]=f.data.positions[node-1][2]=.03;
    f.data.tools[0].angles[3]=.2;f.data.tools[0].angles[4]=.4;
    const patch=assembleCompositeRelativePatch(f),wireData={...f.data,tools:[f.data.tools[0]]};
    const layout=createCompositeChainLayout(Array.from({length:f.layout.nodeCount-1},()=>['wire']));
    const w=createCompositeChainWorkspace(layout),eps=1e-6;
    for(let axis=0;axis<2;axis++) {
        f.data.positions[node][axis+1]+=eps;const plus=assembleCompositeChain(wireData,w).energy;
        f.data.positions[node][axis+1]-=2*eps;const minus=assembleCompositeChain(wireData,w).energy;
        f.data.positions[node][axis+1]+=eps;close(patch.modeGradient[axis],(plus-minus)/(2*eps));
    }
    f.data.tools[1].material=compileCompositeMaterial({EI1:1e7,GJ:1e8});
    const after=assembleCompositeRelativePatch(f);
    after.modeGradient.forEach((v,i)=>close(v,patch.modeGradient[i],1e-12));
    after.modeStiffness.forEach((v,i)=>close(v,patch.modeStiffness[i],1e-12));
});

test('the relative GN block and common coupling are exact pullbacks of all affected wire hinges',()=>{
    const f=fixture();f.data.positions[4][1]=.1;f.data.positions[5][2]=f.data.positions[3][2]=.07;
    const p=assembleCompositeRelativePatch(f),node=f.layout.positions[f.node],n=p.dofs.length;
    const iy=[...p.dofs].indexOf(node+1),iz=[...p.dofs].indexOf(node+2);
    close(p.modeStiffness[0],p.hessian[iy*n+iy]);close(p.modeStiffness[1],p.hessian[iy*n+iz]);close(p.modeStiffness[2],p.hessian[iz*n+iz]);
    close(p.modeGradient[0],p.gradient[iy]);close(p.modeGradient[1],p.gradient[iz]);
    for(let i=0;i<n;i++){close(p.coupling[2*i],p.hessian[i*n+iy]);close(p.coupling[2*i+1],p.hessian[i*n+iz]);}
});

test('local clearance returns a physical normal force and balanced elastic corrections on the common chain',()=>{
    const f=fixture(),patch=assembleCompositeRelativePatch({...f,relativeGradient:[1,0]});
    const r=solveCompositeRelativePatch({patch,clearance:.0405});
    assert.ok(r.converged);assert.ok(r.cell.active);assert.ok(r.cell.normalForce>0);
    close(r.cell.offset[0],-.0405);close(r.cell.normalForce,1-patch.modeStiffness[0]*.0405);
    const force=[0,0,0];
    for(let i=0;i<patch.dofs.length;i++) for(const start of f.layout.positions) {
        const axis=patch.dofs[i]-start;if(axis>=0&&axis<3)force[axis]+=r.correction.gradient[i];
    }
    force.forEach(v=>close(v,0,1e-12));
    assert.equal(r.correction.eliminatedDofCount,2);
    assert.equal(r.scope,'local-quadratic-correction-only');
});

test('a tangential mode cannot be used as circular lumen clearance',()=>{
    assert.throws(()=>assembleCompositeRelativePatch({...fixture(),basis:[[1,0,0],[0,0,1]]}),/transverse/);
});

test('physical relative inertia includes its common-relative coupling in the condensed force',()=>{
    const f=fixture(),position=f.layout.positions[f.node],massOverDtSquared=100;
    const patch=assembleCompositeRelativePatch({...f,relativeGradient:[1,0],relativeStiffness:[massOverDtSquared,0,massOverDtSquared],
        additionalCoupling:[{dof:position+1,values:[massOverDtSquared,0]},{dof:position+2,values:[0,massOverDtSquared]}]});
    const result=solveCompositeRelativePatch({patch,clearance:.0405}),row=[...patch.dofs].indexOf(position+1);
    assert.ok(result.converged);assert.equal(result.cell.active,false);
    close(patch.coupling[2*row],105.25);close(result.cell.offset[0],-1/105.25);
    close(result.correction.gradient[row],-1);
    // Independent quadratic envelope with the base common inertia already
    // included: E(q,rho)=.5 K(q+rho)^2+(q+rho), rho free inside clearance.
    const energy=q=>{const rho=-q-1/105.25;return .5*105.25*(q+rho)**2+q+rho;};
    const eps=1e-6;close(1+result.correction.gradient[row],(energy(eps)-energy(-eps))/(2*eps),1e-10);
    assert.throws(()=>assembleCompositeRelativePatch({...f,additionalCoupling:[{dof:f.layout.positions[0],values:[1,0]}]}),/local patch support/);
});

test('a longer distant wire does not increase the local mode support or solve size',()=>{
    const short=assembleCompositeRelativePatch(fixture(9)),long=assembleCompositeRelativePatch(fixture(201));
    assert.equal(long.dofs.length,short.dofs.length);assert.deepEqual(long.affectedHinges,short.affectedHinges);
    assert.deepEqual(long.modeStiffness,short.modeStiffness);assert.deepEqual(long.coupling,short.coupling);
});
