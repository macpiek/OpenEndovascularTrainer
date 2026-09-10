import test from 'node:test';
import assert from 'node:assert/strict';
import {EndovascularPhysicsWorld} from '../src/physics/endovascularPhysicsWorld.js';
import {solveKirchhoffCoupledSystem,applyKirchhoffCoupledCorrection} from '../src/physics/kirchhoffCoupledSystem.js';

// The initial contact is an ordinary penetrated plane. Only after the actual
// joint direction has been applied do we switch the exact-query feature.
class ControlledWall {
    voxelSize=.5;
    planeY=0;
    branchId=0;
    write(p,r,out) {
        const gap=this.planeY-p.y-r,penetration=Math.max(0,-gap);
        Object.assign(out,{signedDistance:this.planeY-p.y,signedGap:gap,penetration,inside:p.y<=this.planeY,
            violation:gap<0,branchId:this.branchId,faceIndex:this.branchId,source:'history-test-plane',timeOfImpact:gap<0?0:1});
        Object.assign(out.point,p);Object.assign(out.closestPoint,{x:p.x,y:this.planeY,z:p.z});
        Object.assign(out.normal,{x:0,y:-1,z:0});Object.assign(out.inward,out.normal);
        Object.assign(out.target,{x:p.x,y:p.y-penetration,z:p.z});return out;
    }
    querySphere(p,r,out){return this.write(p,r,out);}
    queryCapsule(a,b,r,out){const p=a.y>=b.y?a:b;this.write(p,r,out);out.segmentT=p===a?0:1;return out;}
    sweepSphere(a,b,r,out){this.write(b,r,out);const ga=this.planeY-a.y-r,gb=this.planeY-b.y-r;out.timeOfImpact=gb>=0?1:ga<=0?0:ga/(ga-gb);return out;}
}

for(const change of ['gap beyond activation','branch identity']) test(`single common solve retains finite applied wall load through exact refresh: ${change}`,()=>{
    const field=new ControlledWall();
    let chosen,expected,observed,applied=false;
    const world=new EndovascularPhysicsWorld({contactField:field,coupledSystem:{independentComponents:true,
        solve(c,dt,options){
            const result=solveKirchhoffCoupledSystem(c,dt,{...options,activeCondensation:true,simultaneousCoulomb:true});
            assert.ok(result.diagnostics.converged,JSON.stringify(result.diagnostics));
            chosen=options.additionalRows.findIndex((row,i)=>row.kind==='wall' && result.additionalIncrement[i]>0);
            assert.ok(chosen>=0,'the physical solve must generate a real positive normal reaction');
            chosen={index:chosen,node:options.additionalRows[chosen].node,lambda:options.additionalRows[chosen].lambda};
            return result;
        },
        apply(c,result){
            applyKirchhoffCoupledCorrection(c,result);
            expected=chosen.lambda+result.scale*result.additionalIncrement[chosen.index];
            assert.ok(Number.isFinite(expected)&&expected>0);
            applied=true;
            if(change==='gap beyond activation')field.planeY=2;
            else field.branchId=1;
        }
    }});
    const body=world.createRod('single-wall-history',4,5,{radius:.5,sleepFrames:1e6});
    for(let i=0;i<body.count;i++)body.setNodePosition(i,5*i,-.49,0);
    const stop=new Error('captured first exact nonlinear trial');
    world.debugJointTrial=()=>{
        assert.ok(applied);
        observed={lambda:body.wallLambda[chosen.node],active:body.wallActive[chosen.node],
            branch:body.wallBranchId[chosen.node],gap:body.wallGap[chosen.node],solver:world.lastCoupledSolver};
        throw stop;
    };
    // Stop before later nonlinear iterations can obscure ownership at the
    // exact-refresh boundary. No forces are injected, clipped or overwritten.
    assert.throws(()=>world.stepFixed(),error=>error===stop);
    assert.equal(observed.solver,'joint-components');
    if(change==='gap beyond activation')assert.ok(observed.gap>world.contactActivation);
    else assert.equal(observed.branch,1);
    assert.ok(Math.abs(observed.lambda-expected)<=Math.max(1e-12,Math.abs(expected)*2e-7),
        `exact refresh erased applied reaction: expected ${expected}, got ${observed.lambda}`);
    assert.equal(observed.active,1,'a loaded normal row must remain available for the next coupled solve');
});
