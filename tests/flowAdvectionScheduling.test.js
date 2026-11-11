import test from 'node:test';
import assert from 'node:assert/strict';
import * as THREE from 'three';
import {ContrastFlowNetwork} from '../src/contrast/flowNetwork.js';

const segment=(id,a,b,start,end)=>({id,nodeStartId:a,nodeEndId:b,
    start:new THREE.Vector3(...start),end:new THREE.Vector3(...end),radiusStart:2,radiusEnd:2});
function fork(background=0) {
    const segments=[segment(0,0,1,[0,0,0],[1,0,0]),
        segment(1,1,2,[1,0,0],[101,10,0]),segment(2,1,3,[1,0,0],[11,-10,0])];
    for(let i=0;i<background;i++)segments.push(segment(i+3,0,i+4,[0,0,0],[-20,i+1,0]));
    if(background)segments.unshift(segment(-1,-1,0,[0,-1000,0],[0,0,0]));
    return new ContrastFlowNetwork(segments,{rootPoint:new THREE.Vector3(0,background?-1000:0,0),cellLengthMm:1000,axialDispersionMm2PerS:0});
}

function assertPreparedFlows(net,waveform) {
    const solver=net._advection;
    solver.prepare(waveform);
    const outflow=new Float64Array(solver.outflow.length);
    let link=0;
    for(const edge of net.edges)for(let face=0;face<=edge.cellCount;face++,link++) {
        const q=net.getFaceSignedFlowMm3PerS(edge.index,face,waveform);
        assert.equal(solver.flows[link],Math.abs(q));
        assert.equal(solver.signs[link],Math.sign(q)||0);
        if(q)outflow[q>0?solver.from[link]:solver.to[link]]+=Math.abs(q);
    }
    assert.deepEqual(solver.outflow,outflow,'Every clear and stained blood inflow contributes to junction dilution');
}

test('compiled transport follows live source faces, clearing, hydraulic redistribution and occlusion',()=>{
    const net=fork();
    for(const waveform of [.7,1,1.6])assertPreparedFlows(net,waveform);
    net.setFlowOverride([0],17);
    net.addFaceFlowDelta(1,0,1,-250);
    net.addFaceFlowDelta(1,1,2,400);
    net._edgeFlowDeltasMm3PerS[2]=-500;
    assertPreparedFlows(net,.9);
    net.clearFlowOverrides();assertPreparedFlows(net,1.1);
    net.setHemodynamics({cardiacOutputMlPerMin:7000});assertPreparedFlows(net,.8);
    const outlet=net.nodes.get(net.edges[1].endNodeId);
    net.setOutletResistance(outlet.id,outlet.outletResistancePaSPerMm3*7);
    assertPreparedFlows(net,1.2);
    net.edges[1].transportExcluded=true;net._computeHydraulicDistribution();
    assertPreparedFlows(net,1);
    net.edges[1].transportExcluded=false;net._computeHydraulicDistribution();
    assertPreparedFlows(net,1);
});

test('sparse sweep includes clear confluence blood without visiting the rest of a clear tree',()=>{
    const net=fork(256),solver=net._advection;
    const donor=net.edges.find(edge=>edge.sourceId===1),receiver=net.edges.find(edge=>edge.sourceId===2),concentration=.3;
    net.setFlowOverride(net.edges.map(edge=>edge.index),1);
    net.setFlowOverride([donor.index],-1);net.setFlowOverride([receiver.index],2);
    net.depositIodine(donor.index,0,donor.volumes[0]*concentration);
    solver.prepare(1);
    const originalOutflow=solver.outflow;let visits=0;
    solver.outflow=new Proxy(originalOutflow,{get(target,key){
        if(typeof key==='string'&&/^\d+$/.test(key))visits++;
        return Reflect.get(target,key,target);
    }});
    const duration=receiver.volumes[0],initial=net.totalIodineMassMg;
    try {solver.update(duration);} finally {solver.outflow=originalOutflow;}
    // Equal clear antegrade and stained retrograde rates meet at the node.
    // After the accepting cell's transit time its concentration is exactly half.
    assert.ok(Math.abs(receiver.nextMassMg[0]/receiver.volumes[0]-concentration/2)<1e-12,
        'The accepting blood must retain the analytic 1:1 dilution by its clear inflow');
    const remaining=solver.touchedEdges.reduce((sum,index)=>sum+net.edges[index].nextMassMg.reduce((a,b)=>a+b,0),0);
    assert.ok(Math.abs(remaining+net.outletIodineMassMg-initial)<1e-10);
    assert.ok(net.edges.filter(edge=>edge.sourceId>=3).every(edge=>edge.massMg[0]===0&&edge.nextMassMg[0]===0));
    assert.ok(visits<10,`Only the connected iodine profile should be visited, got ${visits}`);
    assert.ok(solver.order.length>500,'The background tree spans many sparse-queue words');
});

test('sparse diffusion includes newly reached clear chains and matches the complete region solve',()=>{
    const make=()=>new ContrastFlowNetwork([
        segment(0,0,1,[0,0,0],[10,0,0]),segment(1,1,2,[10,0,0],[20,10,0]),
        segment(2,1,3,[10,0,0],[20,-10,0]),segment(3,2,4,[20,10,0],[30,10,0]),
        segment(4,3,5,[20,-10,0],[30,-10,0])
    ],{rootPoint:new THREE.Vector3(),cellLengthMm:2,axialDispersionMm2PerS:40});
    const sparse=make(),dense=make();
    // The direct mixing API also supports a network without an activity set.
    const referenceNetwork=Object.create(dense);
    referenceNetwork._activeEdgeIndices=undefined;
    dense.dilatedLumenMixing.network=referenceNetwork;
    for(const net of [sparse,dense]) {
        net.depositIodine(0,0,1);
        const marks=new Uint8Array(net.edges.length),touched=[1];
        marks[1]=1;net.edges[1].nextMassMg[0]=.3;
        net.dilatedLumenMixing.update(1/60,touched,marks,1.1);
        net.testTouched=touched;
    }
    assert.deepEqual(sparse.testTouched,dense.testTouched);
    for(let i=0;i<sparse.edges.length;i++)assert.deepEqual(sparse.edges[i].nextMassMg,dense.edges[i].nextMassMg);
    assert.ok(sparse.edges[3].nextMassMg.some(value=>value>0),'Newly reached daughter chain mixes beyond its entry cell');
    assert.ok(sparse.edges[2].nextMassMg.every(value=>value===0),'Separate clear territory stays clear');
});
