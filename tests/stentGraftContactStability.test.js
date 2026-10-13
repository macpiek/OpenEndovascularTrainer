import test from 'node:test';
import assert from 'node:assert/strict';
import * as THREE from 'three';
import {MeshBVH} from 'three-mesh-bvh';
import {createSharedAxisNative} from '../src/physics/kirchhoffSharedAxisNative.js';
import {createStentGraftContacts} from '../src/devices/stentGraftContacts.js';
import {branchRecoveryPotential,createBranchRecoveryWorkspace,graftBranchContactAt,graftBranchRecoveryPlanes} from '../src/devices/stentGraftBranchContact.js';
import {captureReaction} from '../src/devices/stentGraftCapture.js';

const quadratic=(h,v)=>v.reduce((sum,x,i)=>sum+x*h[i].reduce((s,y,j)=>s+y*v[j],0),0);

test('fabric edge fallback removes negative curvature without changing energy or force',()=>{
    const geometry=new THREE.PlaneGeometry(10,10).translate(5,5,0);
    geometry.boundsTree=new MeshBVH(geometry);geometry.computeBoundingBox();
    try {
        const state=createSharedAxisNative({tools:[{id:'wire',insertion:10,radius:.3}],spacing:2,
            samplePosition:s=>[s,-.1,.1]});
        const contact=createStentGraftContacts({geometry,bounds:geometry.boundingBox,revision:1},state);
        const evaluate=mode=>{
            state.chain.gradient.fill(0);state.chain.hessian.fill(0);
            const energy=contact.addPotential(state,true,mode),{layout}=state;
            const v=new Float64Array(layout.dofCount);
            for(const i of layout.positions){v[i+1]=1;v[i+2]=1;}
            let curvature=0;
            for(let i=0;i<v.length;i++)for(let j=Math.max(0,i-layout.band+1);j<=i;j++)
                curvature+=(i===j?1:2)*v[i]*v[j]*state.chain.hessian[i*layout.band+i-j];
            return {energy,gradient:Array.from(state.chain.gradient),curvature};
        };
        const exact=evaluate('newton'),fallback=evaluate('gauss-newton');
        assert.ok(exact.curvature<0,'edge contact exercises an indefinite tangent');
        assert.ok(fallback.curvature>=-1e-8);
        assert.equal(fallback.energy,exact.energy);
        assert.deepEqual(fallback.gradient,exact.gradient);
    } finally {geometry.dispose();}
});

test('portal recovery and compressed crown retain forces with a positive fallback tangent',()=>{
    const contact={point:[0,0,0],planeNormal:[0,-1,0],start:[0,0,0],end:[10,0,0],axisStart:[1,0,0],axisEnd:[1,0,0]};
    for(const x of [.1,.7,1.5,5,8.5,9.3,9.9]) {
        const p=[x,.2,0],exact=branchRecoveryPotential(contact,p,.8),fallback=branchRecoveryPotential(contact,p,.8,'gauss-newton');
        assert.equal(fallback.energy,exact.energy);assert.deepEqual(fallback.gradient,exact.gradient);
        for(const v of [[1,0,0],[0,1,0],[1,1,0],[1,-1,1]])assert.ok(quadratic(fallback.hessian,v)>=-1e-12);
    }
    const links={roots:[[0,0,0]],armLength:2,stiffness:180};
    const exact=captureReaction(links,[1,0,0]),fallback=captureReaction(links,[1,0,0],'gauss-newton');
    assert.ok(exact.hessian[1][1]<0);assert.equal(fallback.hessian[1][1],0);
    assert.equal(fallback.energy,exact.energy);assert.deepEqual(fallback.gradient,exact.gradient);
});

test('reused recovery workspace preserves force and tangent across active, clear and residual evaluations',()=>{
    const contact={point:[0,0,0],planeNormal:[0,-1,0],start:[0,0,0],end:[10,0,0],axisStart:[1,0,0],axisEnd:[1,0,0]};
    const workspace=createBranchRecoveryWorkspace();
    for(const mode of ['newton','gauss-newton'])for(const p of [[.7,.2,0],[5,-2,0],[9.3,.4,0],[0,.2,0],[5,.1,0]]) {
        const expected=branchRecoveryPotential(contact,p,.8,mode);
        const tangent=branchRecoveryPotential(contact,p,.8,mode,workspace);
        assert.equal(tangent,workspace);
        assert.equal(tangent.energy,expected.energy);
        assert.deepEqual(tangent.gradient,expected.gradient);
        assert.deepEqual(tangent.hessian,expected.hessian);
        for(const row of workspace.hessian)row.fill(NaN);
        const residual=branchRecoveryPotential(contact,p,.8,mode,workspace,false);
        assert.equal(residual.energy,expected.energy);
        assert.deepEqual(residual.gradient,expected.gradient);
        assert.ok(residual.hessian.every(row=>row.every(Number.isNaN)),'residual must not form an unused tangent');
    }
});

// Millimetre-scale lumen with only 1 micrometre of axial corrugation. Those
// tiny folds must not turn each triangle into another full-strength wall.
function narrowBranch(rows) {
    const sides=24,positions=[],indices=[],centers=[];
    for(let i=0;i<rows;i++) {
        const x=10*i/(rows-1),r=.6+(rows>2?.001*Math.sin(i*2):0);centers.push([x,0,0]);
        for(let j=0;j<sides;j++){const a=2*Math.PI*j/sides;positions.push(x,r*Math.cos(a),r*Math.sin(a));}
        if(i)for(let j=0;j<sides;j++) {
            const a=(i-1)*sides+j,b=(i-1)*sides+(j+1)%sides,c=i*sides+j,d=i*sides+(j+1)%sides;
            indices.push(a,c,b,b,c,d);
        }
    }
    return {positions,indices,centers,junctionRecovery:true};
}

test('folded-lumen reaction is independent of triangle count and retains opposing wall support',()=>{
    let reference;
    for(const rows of [2,16,32,64]) {
        const p=[5,.1,0],radius=.8,contact=graftBranchContactAt([narrowBranch(rows)],p,radius);
        try {
            const planes=graftBranchRecoveryPlanes(contact,p,radius),gradient=[0,0,0];let energy=0;
            assert.ok(planes.some(p=>p.planeNormal[1]<-.5)&&planes.some(p=>p.planeNormal[1]>.5));
            for(const plane of planes) {
                const r=branchRecoveryPotential(plane,p,radius);
                energy+=plane.weight*r.energy;
                for(let k=0;k<3;k++)gradient[k]+=plane.weight*r.gradient[k];
            }
            assert.ok(energy>0&&energy<.1,'one physical wall cannot acquire dozens of contact springs');
            assert.ok(gradient[1]>0,'off-centre tool is pushed towards the lumen interior');
            reference??={energy,gradient};
            assert.ok(Math.abs(energy/reference.energy-1)<.03);
            assert.ok(Math.hypot(...gradient.map((v,k)=>v-reference.gradient[k]))<.03*Math.hypot(...reference.gradient));
        } finally {contact.geometry.dispose();}
    }
});
