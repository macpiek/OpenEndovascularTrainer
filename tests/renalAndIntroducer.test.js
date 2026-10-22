import test from 'node:test';
import assert from 'node:assert/strict';
import fs from 'node:fs';
import * as THREE from 'three';
import {decodeCollisionAsset} from '../src/physics/collision/collisionAssetFormat.js';
import {HybridContrastSystem} from '../src/contrast/hybridContrastSystem.js';
import {ContrastVolumeRenderer} from '../src/contrast/contrastVolumeRenderer.js';
import {extendRenalArteries} from '../src/renalArteries.js';
import {renalSphereLevelBound} from '../src/renalShape.js';
import {RenalPerfusion} from '../src/contrast/renalPerfusion.js';
import {RenalParenchymaRenderer} from '../src/contrast/renalParenchymaRenderer.js';
import {guidewireMaterialProfile,sampleGuidewireRestCenterline} from '../src/physics/guidewireMaterialProfile.js';
import {kirchhoffMaterialProfile} from '../src/physics/kirchhoffMaterialProfile.js';
import {createSharedAxisContacts} from '../src/physics/kirchhoffSharedAxisContacts.js';
import {createSharedAxisNative} from '../src/physics/kirchhoffSharedAxisNative.js';
import {sampleSheathPath,sheathLength} from '../src/physics/sheathPath.js';
import {canSelectIntroducer,prepareIntroducerSelection,prepareIntroducerExchange,prepareIntroducerMotion} from '../src/devices/introducerSheath.js';

test('angled Glidewire has a manufactured 45 degree bend in preview and mechanics',()=>{
    const p=guidewireMaterialProfile('glidewire-angled');
    assert.equal(p.id,'glidewire-angled');
    assert.ok(Math.abs(p.integrateIntrinsicTurn(7.5,15)-Math.PI/4)<1e-12);
    const tip=sampleGuidewireRestCenterline(p.id,40,40);
    assert.ok(Math.abs(tip.turnAngle-Math.PI/4)<1e-6);
    assert.ok(tip.normalDistance>4);
    assert.equal(kirchhoffMaterialProfile(p.id).id,p.id);
    assert.equal(guidewireMaterialProfile('glidewire').naturalTurn,0);
});

const short={start:{x:0,y:0,z:0},end:{x:0,y:70,z:0},radius:1};
const sampleWire=s=>s<=70?{x:0,y:s,z:0}:{x:0,y:70+300*Math.sin((s-70)/300),z:300*(1-Math.cos((s-70)/300))};
test('45 and 90 cm exchanges follow the accepted wire and validate prerequisites',()=>{
    for(const length of [450,900]) {
        const options={sheath:short,length,insertion:length,french:8,transport:{progress:1000},sampleWire,clearanceAt:()=>4};
        const result=prepareIntroducerExchange(options);
        assert.equal(sheathLength(result),length);
        assert.equal(result.french,8);
        assert.ok(result.innerRadius<result.radius);
        const tip=sampleSheathPath(result,length);
        assert.ok(Math.abs(tip.z-sampleWire(length).z)<1e-8);
        assert.deepEqual(short.end,{x:0,y:70,z:0});
        assert.equal(prepareIntroducerExchange({...options,catheterProgress:100}).french,8);
        assert.throws(()=>prepareIntroducerExchange({...options,catheterProgress:100,catheterRadius:2}),/Cewnik nie mieści/);
        assert.equal(sheathLength(prepareIntroducerExchange({...options,transport:{progress:length}})),length,
            'the sheath may reach the support tip without an artificial 3 mm stop');
        assert.throws(()=>prepareIntroducerExchange({...options,clearanceAt:()=>.8}),/nie mieści/);
        const returned=prepareIntroducerExchange({...options,sheath:result,length:70,insertion:70});
        assert.equal(returned.end.z,0);
        const contacts=createSharedAxisContacts({sheath:{...result,proximalExtension:40},localCoordinates:true});
        const a=sampleSheathPath(result,length-4),b=sampleSheathPath(result,length-2);
        const row=contacts.wallSamples[0]({a:[a.x,a.y,a.z],b:[b.x,b.y,b.z],coordinateA:length-4,coordinateB:length-2,radius:.4445});
        assert.ok(row.gap>0.7);
    }
});

test('shared-axis material coverage reaches the full 2000 mm without the old 1000 mm cap',()=>{
    const state=createSharedAxisNative({...createSharedAxisContacts({sheath:{...short,innerRadius:.9,proximalExtension:40}}),
        tools:[{id:'wire',type:'glidewire-angled',length:2000,insertion:2000,shaftStiffness:13.15,tipStiffness:19.4}]});
    assert.equal(state.coordinates.at(-1),2000);
    assert.equal(state.materials[0].spec.length,2000);
});

test('introducer displaces a wall-hugging wire route inward but rejects a narrow lumen',()=>{
    const options={sheath:short,length:450,french:6,insertion:450,transport:{progress:500},
        sampleWire:s=>({x:3.5,y:s,z:0}),
        contactAt:p=>({signedDistance:4-Math.abs(p.x),normal:{x:-Math.sign(p.x),y:0,z:0}}),
        clearanceAt:p=>4-Math.abs(p.x)};
    const result=prepareIntroducerExchange(options);
    assert.ok(result.path.slice(36).every(p=>options.clearanceAt(p)>=1.05));
    assert.throws(()=>prepareIntroducerExchange({...options,
        contactAt:p=>({signedDistance:.8-Math.abs(p.x),normal:{x:-Math.sign(p.x),y:0,z:0}}),
        clearanceAt:p=>.8-Math.abs(p.x)}),/nie mieści|ścian/);
});

test('parenchymal uptake is delayed, selective and conserves iodine during washout',()=>{
    const perfusion=new RenalPerfusion([{side:'left',terminalNodes:[{id:1,sector:0}]},{side:'right',terminalNodes:[{id:2,sector:0}]}]);
    perfusion.receive(1,100);
    assert.equal(perfusion.kidneys[0].tissue[0],0);
    let drained=perfusion.update(.5);
    assert.ok(perfusion.kidneys[0].tissue[0]>0);
    assert.equal(perfusion.kidneys[1].tissue[0],0);
    assert.ok(Math.abs(perfusion.massMg+drained-100)<1e-10);
    for(let i=0;i<300;i++)drained+=perfusion.update(.1);
    assert.ok(perfusion.massMg<.2);
    assert.ok(Math.abs(perfusion.massMg+drained-100)<1e-9);
});

function loadSegments(name) {
    const b=fs.readFileSync(new URL(`../res/${name}.collision.bin`,import.meta.url));
    const asset=decodeCollisionAsset(b.buffer.slice(b.byteOffset,b.byteOffset+b.byteLength));
    const d=asset.arrays.centerlineSegments,e=asset.arrays.centerlineEdges;
    return Array.from({length:e.length/2},(_,i)=>({id:i,start:new THREE.Vector3(...d.slice(i*9,i*9+3)),end:new THREE.Vector3(...d.slice(i*9+3,i*9+6)),
        radiusStart:d[i*9+6],radiusEnd:d[i*9+7],safeRadius:d[i*9+8],nodeStartId:e[2*i],nodeEndId:e[2*i+1]}));
}
for(const name of ['Aorta_plain','Aorta_infrarenal_aneurysm']) test(`${name}: renal paths join smoothly and taper continuously`,()=>{
    const {segments,kidneys}=extendRenalArteries(loadSegments(name));
    const parents=new Map(segments.map(s=>[s.nodeEndId,s]));
    const childCounts=new Map();
    for(const segment of segments)childCounts.set(segment.nodeStartId,(childCounts.get(segment.nodeStartId)??0)+1);
    for(const segment of segments.filter(s=>s.renalSide)) {
        const parent=parents.get(segment.nodeStartId);
        if(segment.renalSector>=0 && !segment.renalHilar) {
            const kidney=kidneys.find(k=>k.side===segment.renalSide);
            // Include the 0.4 mm vessel wall and a 0.1 mm sweep allowance.
            // Stay inside the fully opaque tissue, not merely its faded edge.
            const envelope=Math.max(segment.radiusStart,segment.radiusEnd)+.5;
            for(const point of [segment.start,segment.end])
                assert.ok(renalSphereLevelBound(point,envelope,kidney)<.84,
                    `${kidney.side} vessel ${segment.id} protrudes beyond the renal parenchyma`);
        }
        assert.ok(parent,'every renal branch must connect to the vascular tree');
        const angle=parent.end.clone().sub(parent.start).angleTo(segment.axis);
        assert.ok(angle<15*Math.PI/180,`abrupt renal bend: ${angle*180/Math.PI} degrees`);
        assert.ok(segment.length<=.51,'curves must be sampled densely enough for transport and collision');
        if(parent.renalSide)assert.ok(Math.abs(parent.radiusEnd-segment.radiusStart)<1e-10,'no calibre step at a bifurcation');
    }
    for(const kidney of kidneys) {
        const firstBranches=segments.filter(s=>s.renalSide===kidney.side&&s.renalHilar&&parents.get(s.nodeStartId)?.renalSector===-1);
        assert.equal(firstBranches.length,5);
        for(const branch of firstBranches)
            assert.ok(renalSphereLevelBound(branch.start,0,kidney)>1.1,
                'the first renal division must start outside the parenchymal outline');
        const hilarEnds=segments.filter(s=>s.renalSide===kidney.side&&!s.renalHilar&&parents.get(s.nodeStartId)?.renalHilar);
        for(const branch of hilarEnds)
            assert.ok(renalSphereLevelBound(branch.start,branch.radiusStart+.5,kidney)<.84,
                'hilar branches must enter the kidney before dividing into smaller vessels');
        assert.ok(kidney.terminalNodes.length>=75,
            'two additional cortical generations must supply a dense terminal tree');
        const counts=Array.from({length:5},(_,sector)=>kidney.terminalNodes.filter(n=>n.sector===sector).length);
        assert.ok(new Set(counts).size>=3,'sectors must have different branching patterns');
        assert.deepEqual([...new Set(kidney.terminalNodes.map(n=>n.sector))].sort(),[0,1,2,3,4]);
        const tips=kidney.terminalNodes.map(n=>parents.get(n.id).end.clone().sub(kidney.center));
        assert.ok(tips.some(p=>p.y>42)&&tips.some(p=>p.y< -42),
            'fine vessels must reach both renal poles');
        const corticalTips=kidney.terminalNodes.filter(n=>!parents.get(n.id).renalLateral)
            .map(n=>parents.get(n.id).end.clone().sub(kidney.center));
        const angles=corticalTips.filter(p=>Math.abs(p.z)<9)
            .map(p=>Math.atan2(p.y/55,p.x*kidney.sign/35)).sort((a,b)=>a-b);
        assert.ok(angles[0]< -2&&angles.at(-1)>2,'coverage must extend around the medial sides of the poles');
        assert.ok(angles.slice(1).every((a,i)=>a-angles[i]<.20),
            'the cortical arc must not have large gaps between isolated vascular tufts');
        assert.ok(tips.filter(p=>Math.abs(p.z)>10).length>=30,
            'anterior/posterior territories must fill the projected interior as well as the lateral rim');
        const takeoffs=segments.filter(s=>s.renalSide===kidney.side&&s.renalLateral&&!parents.get(s.nodeStartId)?.renalLateral);
        assert.ok(takeoffs.length>=15,'fine branches must also arise along the sides of main arteries');
        const lateralChildren=new Map();
        for(const s of segments.filter(s=>s.renalLateral)){
            const children=lateralChildren.get(s.nodeStartId)??[];
            children.push(s);lateralChildren.set(s.nodeStartId,children);
        }
        const lateralDepths=[];
        for(const takeoff of takeoffs){
            const continuation=segments.find(s=>s.nodeStartId===takeoff.nodeStartId&&!s.renalLateral);
            assert.ok(continuation,'a lateral takeoff must share a real node with the continuing parent artery');
            let end=takeoff,distance=takeoff.length;
            while(lateralChildren.get(end.nodeEndId)?.length===1){
                end=lateralChildren.get(end.nodeEndId)[0];
                distance+=end.length;
                if(distance>1.6)assert.ok(end.radiusEnd<=.241,
                    'a lateral branch must become fine close to its ostium, not form a thick crossbar');
            }
            const displacement=end.end.clone().sub(takeoff.start);
            assert.ok(displacement.clone().normalize().dot(parents.get(takeoff.nodeStartId).axis)>.5,
                'side branches should fan obliquely along the artery rather than form transverse rungs');
            lateralDepths.push(displacement.z);
        }
        assert.ok(lateralDepths.some(z=>z>1)&&lateralDepths.some(z=>z< -1),
            'side branches must spread through depth, not repeat in a single plane');
        assert.deepEqual([...new Set(takeoffs.map(s=>s.renalSector))].sort(),[0,1,2,3,4],
            'every renal territory needs lateral branches');
        for(const terminal of kidney.terminalNodes){
            let first=parents.get(terminal.id);
            if(!first.renalLateral)continue;
            const end=first.end;
            while(childCounts.get(first.nodeStartId)===1&&parents.get(first.nodeStartId)?.renalLateral)
                first=parents.get(first.nodeStartId);
            const incoming=parents.get(first.nodeStartId);
            assert.ok(end.clone().sub(first.start).dot(incoming.axis)>0,
                'a lateral terminal near the capsule must not fold backward and pinch its lumen');
        }
    }
    assert.notEqual(kidneys[0].terminalNodes.length,kidneys[1].terminalNodes.length,'kidneys must not be mirrored copies');
    const repeated=extendRenalArteries(loadSegments(name));
    assert.deepEqual(repeated.segments,segments,'mesh and runtime generation must agree exactly');
});

for(const name of ['Aorta_plain','Aorta_infrarenal_aneurysm']) test(`${name}: selective renal bolus reaches branched vessels and only its kidney`,()=>{
    const system=new HybridContrastSystem({centerlineSegments:loadSegments(name)});
    assert.equal(system.renalPerfusion.kidneys.length,2);
    const edges=system.flowNetwork.edges.filter(e=>e.renalSide==='left');
    assert.ok(edges.length>60);
    const trunk=edges.find(e=>!system.flowNetwork.edges[e.parentEdgeIndex]?.renalSide);
    assert.ok(trunk);
    const lateralNodes=new Set(system.flowNetwork.sourceSegments
        .filter(s=>s.renalSide==='left'&&s.renalLateral).map(s=>s.nodeEndId));
    let lateralUptake=0;
    const receive=system.renalPerfusion.receive.bind(system.renalPerfusion);
    system.renalPerfusion.receive=(nodeId,mass)=>{
        const received=receive(nodeId,mass);
        if(received&&lateralNodes.has(nodeId))lateralUptake+=mass;
        return received;
    };
    system.flowNetwork.depositIodine(trunk.index,0,200);
    system.totalInjectedIodineMassMg=200;
    for(let i=0;i<60;i++)system.advanceSynchronized(1/30);
    const left=system.renalPerfusion.kidneys.find(k=>k.side==='left'),right=system.renalPerfusion.kidneys.find(k=>k.side==='right');
    assert.ok(left.tissue.reduce((a,b)=>a+b,0)>1,'renal bolus should perfuse parenchyma');
    assert.ok(lateralUptake>0,'lateral branches must carry actual iodine to their perfusion outlets');
    assert.equal(right.tissue.reduce((a,b)=>a+b,0),0);
    assert.ok(Math.abs(system.getMetrics().balanceErrorMg)<1e-5);
});

test('renal tissue clears over 90 percent within three seconds and 99 percent within five',()=>{
    const perfusion=new RenalPerfusion([{side:'left',terminalNodes:[{id:1,sector:0}]}]);
    perfusion.receive(1,100);
    let drained=perfusion.update(.5);
    assert.ok(perfusion.kidneys[0].tissue[0]>40,'retain a visible parenchymal phase');
    for(let i=0;i<25;i++)drained+=perfusion.update(.1);
    assert.ok(perfusion.massMg<10,'avoid persistent nephrogram after the bolus');
    assert.ok(Math.abs(perfusion.massMg+drained-100)<1e-9);
    const singleStep=new RenalPerfusion([{side:'left',terminalNodes:[{id:1,sector:0}]}]);
    singleStep.receive(1,100);singleStep.update(3);
    assert.ok(Math.abs(singleStep.massMg-perfusion.massMg)<1e-9,'washout is independent of frame rate');
    drained+=perfusion.update(2);
    assert.ok(perfusion.massMg<1);
    assert.ok(Math.abs(perfusion.massMg+drained-100)<1e-9);
});

test('nephrogram dilutes and spreads tissue signal without changing iodine mass',()=>{
    const perfusion=new RenalPerfusion([{side:'left',sign:1,center:new THREE.Vector3(),terminalNodes:[]}]);
    const kidney=perfusion.kidneys[0];
    kidney.tissue[2]=120;
    const renderer=new RenalParenchymaRenderer(perfusion);
    try {
        renderer.update(false,.12);
        const density=renderer.group.children[0].material.uniforms.density.value;
        const regional=Array.from(density),total=regional.reduce((a,b)=>a+b,0);
        assert.ok(regional.every(v=>v>0),'a local bolus must produce a diffuse tissue signal');
        assert.ok(regional[2]>regional[0],'retain regional perfusion differences');
        assert.ok(regional[2]<120/(24*350)*44/2,'reduce the former concentrated tissue peak by at least half');
        assert.equal(perfusion.massMg,120,'display dilution must not remove iodine');
        kidney.tissue.fill(24);
        renderer.update(false,.12);
        assert.ok(Math.abs(density.reduce((a,b)=>a+b,0)-total)<1e-7,
            'redistributing the same iodine should preserve total tissue signal');
        kidney.tissue.fill(48);
        renderer.update(false,.12);
        assert.ok(Math.abs(density.reduce((a,b)=>a+b,0)-2*total)<1e-7,
            'concentration before display compression must track the actual iodine dose');
    } finally {renderer.dispose();}
});

test('exchange moves the injection port and changes hydraulic resistance',()=>{
    const system=new HybridContrastSystem({centerlineSegments:[{id:0,start:new THREE.Vector3(),end:new THREE.Vector3(0,1000,0),radiusStart:5,radiusEnd:5,nodeStartId:0,nodeEndId:1}],sheath:short});
    const long=prepareIntroducerExchange({sheath:short,length:900,insertion:900,french:10,transport:{progress:1000},sampleWire});
    system.setAccess(long,null);
    assert.equal(system.deviceHydraulicProfiles.sheath.lengthMm,900);
    assert.equal(system.deviceHydraulicProfiles.sheath.innerDiameterMm,2*long.innerRadius);
    assert.equal(system._sheathPort.position.z,long.end.z);
    assert.ok(Math.abs(system._sheathPort.direction.length()-1)<1e-12);
});

test('renal branches render beyond the native volume and tissue remains visible after arteries clear',()=>{
    const system=new HybridContrastSystem({centerlineSegments:[
        {id:0,start:new THREE.Vector3(0,-100,0),end:new THREE.Vector3(27.49,-194.19,-6.70),radiusStart:3,radiusEnd:.6,nodeStartId:0,nodeEndId:1}
    ]});
    const edge=system.flowNetwork.edges.find(e=>e.renalSide);
    system.flowNetwork.depositIodine(edge.index,0,10);
    const renderer=new ContrastVolumeRenderer(system);
    assert.equal(renderer.flowMesh.visible,true);
    assert.equal(renderer._flowVolume.weights[edge.index],0);
    for(const e of system.flowNetwork.edges){e.massMg.fill(0);e.nextMassMg.fill(0);e.active=false;}
    system.flowNetwork._activeEdgeIndices.clear();system.flowNetwork._updateEdgeConcentrations();
    system.renalPerfusion.receive(system.renalPerfusion.kidneys[0].terminalNodes[0].id,100);
    system.renalPerfusion.update(2);system.simulationTimeSeconds=2;
    renderer.update();
    assert.equal(renderer.flowMesh.visible,false);
    assert.equal(renderer.renalRenderer.group.visible,true);
    assert.equal(renderer.group.visible,true);
    assert.equal(system.hasVisibleContrast(),true);
    renderer.dispose();
});

test('selection requires full withdrawal, preserves deployed tools and starts at zero insertion',()=>{
    const options={sheath:short,length:900,french:6,transport:{progress:100},catheterProgress:90,sampleWire,sampleCatheter:sampleWire};
    assert.equal(canSelectIntroducer(short),false);
    assert.throws(()=>prepareIntroducerSelection(options),/Całkowicie wycofaj/);
    const partial=prepareIntroducerMotion(options,-69.999);
    assert.equal(canSelectIntroducer(partial),false);
    assert.throws(()=>prepareIntroducerSelection({...options,sheath:partial}),/Całkowicie wycofaj/);
    const empty=prepareIntroducerMotion(options,-70);
    assert.equal(canSelectIntroducer(empty),true);
    const exchanged=prepareIntroducerSelection({...options,sheath:empty});
    assert.equal(exchanged.length,900);assert.equal(sheathLength(exchanged),0);
    const resized=prepareIntroducerSelection({...options,sheath:exchanged,french:8});
    assert.equal(resized.french,8);assert.equal(sheathLength(resized),0);
    assert.throws(()=>prepareIntroducerSelection({...options,sheath:empty,catheterRadius:2}),/Cewnik nie mieści/);
    const advanced=prepareIntroducerMotion({...options,sheath:resized},80);
    assert.equal(sheathLength(advanced),80);assert.equal(options.transport.progress,100);assert.equal(options.catheterProgress,90);
    assert.throws(()=>prepareIntroducerSelection({...options,sheath:advanced,length:450}),/Całkowicie wycofaj/);
    assert.throws(()=>prepareIntroducerMotion({...options,sheath:advanced},30),/Brak kontaktu/);
    const withdrawn=prepareIntroducerMotion({...options,sheath:advanced},-100);
    assert.equal(sheathLength(withdrawn),0);assert.equal(withdrawn.length,900);
    const contacts=createSharedAxisContacts({sheath:{...withdrawn,proximalExtension:40},localCoordinates:true});
    const state=createSharedAxisNative({...contacts,tools:[{id:'wire',type:'glidewire',length:2000,insertion:100}]});
    assert.ok(state.positions.flat().every(Number.isFinite));
    assert.ok(Number.isFinite(sampleSheathPath(withdrawn,-40).y));
    const unsupported=prepareIntroducerMotion({sheath:advanced,transport:{progress:0},catheterProgress:0},-5);
    assert.equal(sheathLength(unsupported),75,'withdrawal does not require a deployed support tool');
});
