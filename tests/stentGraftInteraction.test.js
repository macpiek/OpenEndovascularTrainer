import test from 'node:test';
import assert from 'node:assert/strict';
import * as THREE from 'three';
import {MeshBVH} from 'three-mesh-bvh';
import {createStentGraftContacts} from '../src/devices/stentGraftContacts.js';
import {applyStentGraftFlow} from '../src/contrast/stentGraftFlow.js';
import {ContrastFlowNetwork} from '../src/contrast/flowNetwork.js';
import {createSharedAxisNative,feedSharedAxisNative,relaxSharedAxisNative} from '../src/physics/kirchhoffSharedAxisNative.js';
import {defineKirchhoffMaterialProfile} from '../src/physics/kirchhoffMaterialProfile.js';

function tube() {
    const geometry=new THREE.CylinderGeometry(3,3,60,48,1,true).rotateZ(-Math.PI/2).translate(50,0,0);
    geometry.boundsTree=new MeshBVH(geometry);geometry.computeBoundingBox();
    return {geometry,bounds:geometry.boundingBox,revision:1};
}
function discovery(surface,old,a,b) {
    const reference={coordinates:[0,1],positions:old,origin:[0,0,0]};
    const state={...reference,positions:[a,b],definitionIds:new Set(),layout:{positions:[0,3]}};
    createStentGraftContacts(surface,reference)({state,a,b,edge:0,radius:.2,coordinateA:0,coordinateB:1});
    return {state,rows:[...(state.pendingVesselRows?.values()??[])]};
}
test('fabric blocks crossing from both sides, including a whole-step jump; open ends have no cap',()=>{
    const surface=tube();
    try {
        for(const [old,next] of [[2.5,3.5],[3.5,2.5],[0,20]]) {
            assert.throws(()=>discovery(surface,[[45,old,0],[55,old,0]],[45,next,0],[55,next,0]),error=>error.code==='trial-outside-vessel');
        }
        assert.equal(discovery(surface,[[10,0,0],[19,0,0]],[10,0,0],[25,0,0]).rows.length,0);
        assert.equal(discovery(surface,[[79,0,0],[90,0,0]],[75,0,0],[90,0,0]).rows.length,0);
    }finally{surface.geometry.dispose();}
});

test('Newton balances transverse wire load against graft fabric and unloading releases it',()=>{
    const surface=tube(),type=defineKirchhoffMaterialProfile({id:'graft-beam',sampleEI1:()=>1e5,sampleGJ:()=>1e5});
    try {
        let state=createSharedAxisNative({tools:[{id:'wire',insertion:65,type,radius:.2}],spacing:5});
        state.wallSamples.push(createStentGraftContacts(surface,state));state.graftRevision=surface.revision;
        state=feedSharedAxisNative(state,{});
        for(let load=.5;load<=15;load+=.5){
            state.loads[state.layout.positions.at(-1)+1]=load;
            const result=relaxSharedAxisNative(state,{maxIterations:150,forceTolerance:1e-5});
            assert.ok(result.converged,JSON.stringify({load,...result}));
        }
        assert.ok(state.positions.at(-1)[1]<2.9,'fabric stops radial escape');
        assert.ok(state.graftContactStats.contacts>0,'Newton carries a graft reaction');
        state.loads.fill(0);
        const release=relaxSharedAxisNative(state,{maxIterations:150,forceTolerance:1e-5});
        assert.ok(release.converged,JSON.stringify(release));
        assert.ok(Math.abs(state.positions.at(-1)[1])<1e-3);
    }finally{surface.geometry.dispose();}
});

test('sealed graft conserves iodine, excludes a covered side branch, and keeps distal outlets perfused',()=>{
    const nodes=[[0,0,0],[20,0,0],[50,0,0],[80,0,0],[100,10,0],[100,-10,0],[50,25,0]];
    const links=[[0,1],[1,2],[2,3],[3,4],[3,5],[2,6]];
    const network=new ContrastFlowNetwork(links.map(([a,b],i)=>({id:i,nodeStartId:a,nodeEndId:b,
        start:new THREE.Vector3(...nodes[a]),end:new THREE.Vector3(...nodes[b]),radiusStart:10,radiusEnd:10})),{rootPoint:new THREE.Vector3(0,0,0)});
    for(const edge of network.edges)network.depositIodine(edge.index,0,1);
    const mass=network.getIodineMassMg();
    const surface={sealed:false,revision:3,contains:p=>p.x>=20&&p.x<80&&Math.abs(p.y)<5,
        sectionAt:p=>p.x>=20&&p.x<=80&&Math.abs(p.y)<.1?{radius:5}:null};
    assert.equal(applyStentGraftFlow(network,surface),false,'open gate must not exclude the sac');
    surface.sealed=true;assert.equal(applyStentGraftFlow(network,surface),true);
    const branch=network.edges.find(e=>e.end.y===25);
    assert.ok(branch.transportExcluded);assert.equal(branch.meanFlowMm3PerS,0);
    for(const end of [10,-10])assert.ok(network.edges.find(e=>e.end.y===end).meanFlowMm3PerS>0);
    assert.ok(Math.abs(network.getIodineMassMg()+network.stentGraftRemodeling.trappedIodineMassMg-mass)<1e-10);
    const trapped=network.stentGraftRemodeling.trappedIodineMassMg;
    network.depositIodine(0,0,50);
    for(let i=0;i<90;i++)network.update(1/30);
    assert.equal(branch.massMg.reduce((a,b)=>a+b,0),0);
    assert.equal(network.stentGraftRemodeling.trappedIodineMassMg,trapped,'new iodine cannot refill the sac');
    assert.ok(Math.abs(network.getIodineMassMg()+network.outletIodineMassMg+trapped-mass-50)<1e-6);
});

test('release around the captured left wire/catheter does not freeze subsequent withdrawal',async()=>{
    const {readFileSync}=await import('node:fs'),{gunzipSync}=await import('node:zlib');
    const {fixture,place,finish}=await import('./helpers/stentGraftFixture.js');
    const {restoreSharedAxisReplay,captureSharedAxisReplay}=await import('./helpers/sharedAxisReplay.js');
    const {advanceSharedAxis}=await import('../src/physics/kirchhoffSharedAxisAppSystem.js');
    const f=fixture();
    try {
        place(f.system,'right','body');assert.ok(f.system.deploy('right').ok);finish(f.system,'right');
        const input=JSON.parse(gunzipSync(readFileSync(new URL('./fixtures/shared-axis/stent-graft-release-incoming.json.gz',import.meta.url))));
        let state=restoreSharedAxisReplay(input,f.contactField),rotations=input.stepRequest.rotations;
        for(let step=0;step<30;step++) {
            const source={...state,wallSamples:state.wallSamples.filter(s=>!s.graftSurface)};
            const sampler=createStentGraftContacts(f.system.surface,state);
            source.wallSamples.push(sampler);source.graftRevision=f.system.surface.revision;source.graftRecovery=sampler.recovery;
            const tools=input.stepRequest.tools.map(tool=>({...tool,insertion:tool.insertion-(tool.id==='catheter'?step*5/6:0)}));
            const iterator=advanceSharedAxis(source,rotations,input.stepRequest.dt,tools,input.stepRequest.options);let next;
            do{next=iterator.next();}while(!next.done);
            assert.ok(next.value.state,JSON.stringify({step,...next.value.result}));
            state=next.value.state;rotations=next.value.rotations;
            assert.ok(state.positions.every(p=>p.every(Number.isFinite)));
        }
        const replay=JSON.parse(JSON.stringify(captureSharedAxisReplay(state,input.sheath)));
        assert.ok(replay.graftContactSurface.positions.length>0,'archive includes the actual fabric, not just native vessel triangles');
        const restored=restoreSharedAxisReplay(replay,f.contactField);
        assert.deepEqual(restored.positions,state.positions);
        assert.ok(restored.wallSamples.some(s=>s.graftSurface));restored.graftReplayGeometry.dispose();
    }finally{f.dispose();}
});


test('contact CCD owns a pose snapshot rather than aliasing the preceding solver state',()=>{
    const surface=tube();
    try {
        const previous={coordinates:[0,1],positions:[[45,2.5,0],[55,2.5,0]],origin:[0,0,0]};
        const sample=createStentGraftContacts(surface,previous);
        previous.positions[0][1]=3.5;previous.positions[1][1]=3.5;previous.origin[1]=100;
        const state={origin:[0,0,0]};
        assert.doesNotThrow(()=>sample({state,a:[45,2.6,0],b:[55,2.6,0],radius:.2,coordinateA:0,coordinateB:1}));
    }finally{surface.geometry.dispose();}
});
