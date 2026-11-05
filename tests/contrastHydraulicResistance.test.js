import test from 'node:test';
import assert from 'node:assert/strict';
import fs from 'node:fs';
import * as THREE from 'three';
import {ContrastFlowNetwork} from '../src/contrast/flowNetwork.js';
import {decodeCollisionAsset} from '../src/physics/collision/collisionAssetFormat.js';

const close = (actual, expected, tolerance = 1e-9) =>
    assert.ok(Math.abs(actual - expected) <= tolerance * Math.max(1, Math.abs(expected)), `${actual} != ${expected}`);
function segment(id, a, b, start, end, radius = 2, renalSide = null) {
    return {id, nodeStartId:a, nodeEndId:b, start:new THREE.Vector3(...start),
        end:new THREE.Vector3(...end), radiusStart:radius, radiusEnd:radius, renalSide};
}
function fork(leaves = 1, renal = false) {
    const segments = [segment(0,0,1,[0,20,0],[0,0,0],2),
        segment(1,1,2,[0,0,0],[-20,-20,0],2,renal?'left':null),
        segment(2,1,3,[0,0,0],[20,-20,0],2)];
    if (leaves > 1) for(let i=0;i<leaves;i++)segments.push(
        segment(i+3,2,i+4,[-20,-20,0],[-30-i,-40,i-leaves/2],.6,renal?'left':null));
    return new ContrastFlowNetwork(segments,{rootPoint:new THREE.Vector3(0,20,0)});
}
function checkKirchhoff(net) {
    for(const id of net.nodeOrder) {
        const node=net.nodes.get(id), parent=net.edges[node.parentEdgeIndex];
        const out=node.childEdgeIndices.reduce((s,i)=>s+net.edges[i].meanFlowMm3PerS,0);
        if(parent&&!parent.transportExcluded&&!node.nativeOutlet)close(out,parent.meanFlowMm3PerS);
        if(node.nativeOutlet&&parent&&!parent.transportExcluded)
            close(node.pressurePa,parent.meanFlowMm3PerS*node.outletResistancePaSPerMm3);
        for(const i of node.childEdgeIndices) {
            const edge=net.edges[i];if(edge.transportExcluded)continue;
            close(node.pressurePa-net.nodes.get(edge.endNodeId).pressurePa,
                edge.meanFlowMm3PerS*edge.resistance);
        }
    }
}

test('terminal resistances determine the analytic parallel split and conserve flow and pressure',()=>{
    const net=fork();net.setOutletResistance(2,.4);net.setOutletResistance(3,1.6);
    const left=net.edges.find(e=>e.endNodeId===2),right=net.edges.find(e=>e.endNodeId===3);
    const q=net.hemodynamics.cardiacOutputMlPerMin*1000/60;
    const ra=left.resistance+.4,rb=right.resistance+1.6;
    close(left.meanFlowMm3PerS,q*rb/(ra+rb));
    close(right.meanFlowMm3PerS,q*ra/(ra+rb));checkKirchhoff(net);
    const before=left.meanFlowMm3PerS;
    net.setOutletResistance(2,4);
    assert.ok(left.meanFlowMm3PerS<before/2,'changing an outlet resistance must actually change flow');
    checkKirchhoff(net);
});

test('renal refinement preserves equivalent bed resistance, renal flow and sibling flow',()=>{
    const coarse=fork(1,true),root=n=>n.edges.find(e=>e.renalSide==='left');
    for(const leaves of [2,8,32]){
        const fine=fork(leaves,true);
        close(root(fine).equivalentResistance,root(coarse).equivalentResistance);
        close(root(fine).meanFlowMm3PerS,root(coarse).meanFlowMm3PerS);
        close(fine.edges.find(e=>e.endNodeId===3).meanFlowMm3PerS,
            coarse.edges.find(e=>e.endNodeId===3).meanFlowMm3PerS);
        checkKirchhoff(fine);
        assert.ok(fine.renalResistanceDiagnostics.every(d=>!d.geometryLimited));
    }
});

test('a subsequent renal restriction changes flow without re-normalizing the bed',()=>{
    const net=fork(8,true),root=net.edges.find(e=>e.renalSide==='left');
    const before=root.meanFlowMm3PerS;
    const resistances=[...net.nodes.values()].map(n=>n.outletResistancePaSPerMm3);
    root.resistance+=root.equivalentResistance*4;
    net._computeHydraulicDistribution();
    assert.ok(root.meanFlowMm3PerS<before/2);
    assert.deepEqual([...net.nodes.values()].map(n=>n.outletResistancePaSPerMm3),resistances);
    checkKirchhoff(net);
    root.transportExcluded=true;net._computeHydraulicDistribution();
    assert.equal(root.meanFlowMm3PerS,0);
    close(net.edges.find(e=>e.endNodeId===3).meanFlowMm3PerS,5000*1000/60);
});

test('a severely restrictive renal tree never receives a negative distal resistance',()=>{
    const net=new ContrastFlowNetwork([segment(0,0,1,[0,20,0],[0,0,0],2),
        segment(1,1,2,[0,0,0],[-500,-500,0],.35,'left'),
        segment(2,1,3,[0,0,0],[20,-20,0],2)],{rootPoint:new THREE.Vector3(0,20,0),peripheralResistancePaSPerMm3:.0001});
    assert.equal(net.renalResistanceDiagnostics[0].geometryLimited,true);
    assert.ok([...net.nodes.values()].filter(n=>n.nativeOutlet).every(n=>n.outletResistancePaSPerMm3>=0));
    checkKirchhoff(net);
});

test('resistance of a tapered vessel is independent of axial mesh subdivision',()=>{
    const make=count=>new ContrastFlowNetwork(Array.from({length:count},(_,i)=>({
        ...segment(i,i,i+1,[0,-i*20/count,0],[0,-(i+1)*20/count,0]),
        radiusStart:2-i/count,radiusEnd:2-(i+1)/count
    })),{rootPoint:new THREE.Vector3(0,0,0)});
    const coarse=make(1),fine=make(20);
    close(coarse.rootNode.equivalentResistance,fine.rootNode.equivalentResistance);
});

function nativeSegments(name) {
    const b=fs.readFileSync(new URL(`../res/${name}.collision.bin`,import.meta.url));
    const asset=decodeCollisionAsset(b.buffer.slice(b.byteOffset,b.byteOffset+b.byteLength));
    const d=asset.arrays.centerlineSegments,l=asset.arrays.centerlineEdges;
    const segments=Array.from({length:l.length/2},(_,i)=>({id:i,nodeStartId:l[i*2],nodeEndId:l[i*2+1],
        start:new THREE.Vector3().fromArray(d,i*9),end:new THREE.Vector3().fromArray(d,i*9+3),
        radiusStart:d[i*9+6],radiusEnd:d[i*9+7],safeRadius:d[i*9+8]}));
    for(const r of asset.metadata.renal.segments)Object.assign(segments[r.index],{renalSide:r.side,renalSector:r.sector});
    return segments;
}
for(const name of ['Aorta_plain_renal','Aorta_infrarenal_aneurysm_renal'])
    test(`${name}: hundreds of renal twigs do not change bed flow; pelvic bolus clears`,()=>{
        const segments=nativeSegments(name);
        const fine=new ContrastFlowNetwork(segments);
        const coarse=new ContrastFlowNetwork(segments.filter(s=>!s.renalSide||s.renalSector<0));
        assert.ok(fine.edges.length>coarse.edges.length+1000);
        for(const d of fine.renalResistanceDiagnostics) {
            const other=coarse.renalResistanceDiagnostics.find(v=>v.side===d.side);
            close(d.targetResistancePaSPerMm3,other.targetResistancePaSPerMm3);
            close(fine.edges[d.rootEdgeIndex].meanFlowMm3PerS,coarse.edges[other.rootEdgeIndex].meanFlowMm3PerS);
            assert.equal(d.geometryLimited,false);
        }
        checkKirchhoff(fine);
        const start=fine.edges.find(e=>e.start.distanceTo(new THREE.Vector3(15.3,-361.5,-31.4))<.2);
        assert.ok(start,'locate the reported pelvic branch');
        const path=[];let e=start;while(e){path.push(e);if(e.childEdgeIndices.length!==1)break;e=fine.edges[e.childEdgeIndices[0]];}
        let injected=0,peak=0;
        for(let frame=0;frame<360;frame++) {
            if(frame<30){const m=start.meanFlowMm3PerS*.3/60;fine.depositIodine(start.index,0,m);injected+=m;}
            fine.update(1/60);
            const mass=path.reduce((s,e)=>s+e.massMg.reduce((a,b)=>a+b,0),0);peak=Math.max(peak,mass);
        }
        assert.ok(peak>1,'bolus really entered the branch');
        assert.ok(path.every(e=>e.massMg.every(m=>m===0)),'no isolated contrast plug at 6 seconds');
        close(fine.getIodineMassMg()+fine.outletIodineMassMg,injected);
    });
