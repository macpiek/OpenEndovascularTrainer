import test from 'node:test';
import assert from 'node:assert/strict';
import {BufferGeometry,Float32BufferAttribute,Vector3} from 'three';
import {MeshBVH} from 'three-mesh-bvh';
import {createSharedAxisSegmentContact} from '../src/physics/kirchhoffSharedAxisSegmentContact.js';
import {createSharedAxisVesselDiscovery} from '../src/physics/kirchhoffSharedAxisVesselWitnesses.js';

function fixture(){const g=new BufferGeometry();g.setAttribute('position',new Float32BufferAttribute([0,-10,-10,0,10,-10,0,0,10],3));g.boundsTree=new MeshBVH(g);return g;}
test('continuous segment finds a face crossing between separated endpoint samples',()=>{
    const g=fixture(),q=createSharedAxisSegmentContact(g);
    const hit=q.query([-2,0,0],[3,0,0],.5);
    assert.equal(hit.distance,0);assert.equal(hit.crossing,true);assert.equal(hit.t,.4);assert.equal(hit.face,0);
    g.dispose();
});
test('finite triangle distance detects capsule overlap without centerline crossing',()=>{
    const g=fixture(),q=createSharedAxisSegmentContact(g);
    const hit=q.query([.25,-2,0],[.25,2,0],.5);
    assert.equal(hit.crossing,false);assert.ok(Math.abs(hit.distance-.25)<1e-12);
    assert.equal(q.query([2,-2,0],[2,2,0],.5).face,-1);
    assert.equal(q.query([0,20,0],[0,21,0],.5).distance,.5,'The plane beyond the finite triangle is not a wall');
    g.dispose();
});
test('discovery adds a continuous wall witness even when ordinary samples report clearance',()=>{
    const g=fixture(),field={fallbackGeometry:g,voxelSize:10,queryCapsuleSoA(){throw Error('Missed the continuous contact');}};
    const discover=createSharedAxisVesselDiscovery(field,0,{continuousSegmentContacts:true});
    const state={origin:[0,0,0],definitions:[],definitionIds:new Set(),layout:{positions:[0,3]}};
    assert.throws(()=>discover({state,a:[-2,0,0],b:[3,0,0],edge:0,radius:.4,coordinateA:1,coordinateB:6}),/shared-axis-wall-discovery/);
    assert.equal(state.pendingVesselRows.size,1);
    const row=[...state.pendingVesselRows.values()][0];assert.equal(row.witness.t,.4);
    const contact=row.evaluate({state,a:[-2,0,0],b:[-1,0,0],radius:.4});
    assert.ok(contact.gap>0);assert.ok(contact.jacobian[0]<0,'The restored inside pose reacts away from the wall');
    state.definitions.push(row);state.definitionIds.add(row.id);state.pendingVesselRows.clear();
    assert.throws(()=>discover({state,a:[-2,0,0],b:[3,0,0],edge:0,radius:.4,coordinateA:1,coordinateB:6}),/segment crossed/);
    g.dispose();
});

test('captured 835.27 mm anatomical state crosses a wall between all four valid sphere samples',async()=>{
    const {loadCoupledRuntimeAnatomy}=await import('./helpers/coupledRuntimeFixture.js');
    const anatomy=await loadCoupledRuntimeAnatomy();
    try {
        const a=[31.66480179514886,59.825494057311346,-25.344000000339214];
        const b=[32.33286047216568,63.95232112051326,-22.601191951188376];
        const radius=.889/2;
        for(let i=0;i<=3;i++) {
            const t=i/3,hit={};
            anatomy.geometry.boundsTree.closestPointToPoint(new Vector3(...a.map((v,k)=>(1-t)*v+t*b[k])),hit);
            assert.ok(hit.distance>=radius-1e-10,'The old sampled test reports a feasible capsule');
        }
        const hit=createSharedAxisSegmentContact(anatomy.geometry).query(a,b,radius+.01);
        assert.equal(hit.crossing,true);assert.ok(hit.t>1/3&&hit.t<2/3);
        assert.equal(hit.distance,0);
    }finally{anatomy.dispose();}
});
