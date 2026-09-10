import test from 'node:test';
import assert from 'node:assert/strict';
import {appendKirchhoffSplitPointWalls} from '../src/physics/kirchhoffSplitMotion.js';
import {createCompositeDiscreteWallPoint} from '../src/physics/compositeDiscreteWallPoint.js';

// Exercise the actual previous solver's point-wall assembly, rather than
// reproducing its equations as the reference for the new representation.
const normal=[.6,.8,0],radius=.3;
function field(){return {
    querySphere(p,r,out){
        const q=[p.x,p.y,p.z],sd=q.reduce((sum,v,k)=>sum+v*normal[k],0);
        out.source='analytic-plane';out.signedDistance=sd;out.signedGap=sd-r;
        out.inward.values.set(normal);out.closestPoint.values.set(q.map((v,k)=>v-sd*normal[k]));
        out.point.values.set(q);out.faceIndex=0;out.branchId=0;out.segmentT=1;out.capsuleSampleCount=1;return out;
    },
    queryCapsuleCoordinates(ax,ay,az,bx,by,bz,r,out){
        assert.deepEqual([ax,ay,az],[bx,by,bz]);return this.querySphere({x:ax,y:ay,z:az},r,out);
    }
};}
function body(){return {x:[0,1],y:[.2,-.55],z:[0,0],nodeRadius:[radius,radius],
    activeStart:0,activeEnd:1,collisionStartSegment:0,collisionEndSegment:0,wallCompliance:0};}
for(const epsilon of [1e-6,1e-12])test(`previous endpoint reactions and new fixed points agree across a capsule minimum switch (${epsilon})`,()=>{
    const inner=body(),outer=body();outer.collisionEndSegment=-1;
    const joint={innerBody:inner,outerBody:outer,_splitMotion:{dt:1/120,pointWalls:[new Map(),new Map()]}},
        world={contactField:field(),contactActivation:1},sites=[0,1].map(fraction=>createCompositeDiscreteWallPoint({fraction}));
    let identities;
    for(const sign of [-1,1]){
        inner.y[0]=.2+sign*epsilon;
        const rows=[];appendKirchhoffSplitPointWalls(joint,world,rows);
        assert.equal(rows.length,2);
        if(identities)rows.forEach((row,i)=>assert.equal(row,identities[i]));else identities=rows.slice();
        const positions=[0,1].map(i=>[inner.x[i],inner.y[i],inner.z[i]]);
        rows.forEach((old,i)=>{
            old.lambda=196; // A loaded endpoint must keep its own identity.
            const row=sites[i].refresh({field:world.contactField,positions,radius});
            assert.equal(row.gap,old.strain);
            for(let j=0;j<6;j++){
                const expected=old.gradients.find(g=>g.dof===Math.floor(j/3)*6+j%3)?.value??0;
                assert.equal(row.gapJacobian[j],expected);
                assert.equal(-row.forceColumn[j]||0,expected);
            }
        });
    }
    assert.equal(joint._splitMotion.pointWalls[0].get(0).lambda,196);
    assert.equal(joint._splitMotion.pointWalls[0].get(1).lambda,196);
});
