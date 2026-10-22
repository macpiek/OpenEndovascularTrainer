import test from 'node:test';
import assert from 'node:assert/strict';
import fs from 'node:fs';
import crypto from 'node:crypto';
import * as THREE from 'three';
import {STLLoader} from 'three/examples/jsm/loaders/STLLoader.js';
import {MeshBVH} from 'three-mesh-bvh';
import {decodeCollisionAsset} from '../src/physics/collision/collisionAssetFormat.js';
import {VesselContactField} from '../src/physics/collision/vesselContactField.js';
import {transformAortaGeometry} from '../src/aortaTransform.js';
import {generateVessel} from '../src/vesselGeometry.js';
import {createSharedAxisContacts} from '../src/physics/kirchhoffSharedAxisContacts.js';
import {createSharedAxisNative} from '../src/physics/kirchhoffSharedAxisNative.js';
import {advanceSharedAxis,SHARED_AXIS_APP_PHYSICS_OPTIONS} from '../src/physics/kirchhoffSharedAxisAppSystem.js';
import {HybridContrastSystem} from '../src/contrast/hybridContrastSystem.js';
import {renalSphereLevelBound} from '../src/renalShape.js';
const folder=process.env.RENAL_ASSET_DIR??new URL('../res/',import.meta.url).pathname;
for(const name of ['Aorta_plain','Aorta_infrarenal_aneurysm'])test(`${name}: connected renal lumen admits wire and 5F catheter through trunk and bifurcations`,()=>{
    const bytes=fs.readFileSync(`${folder}/${name}_renal.collision.bin`),stl=fs.readFileSync(`${folder}/${name}_renal.stl`);
    const asset=decodeCollisionAsset(bytes.buffer.slice(bytes.byteOffset,bytes.byteOffset+bytes.byteLength));
    assert.equal(asset.metadata.source.stlSha256,crypto.createHash('sha256').update(stl).digest('hex'));
    const g=new STLLoader().parse(stl.buffer.slice(stl.byteOffset,stl.byteOffset+stl.byteLength));
    transformAortaGeometry(g,generateVessel(140,0).vessel);g.boundsTree=new MeshBVH(g);
    const field=new VesselContactField(asset,{fallbackGeometry:g,bvhValidationDistance:.02,capsuleBvhValidation:-.1});
    const d=asset.arrays.centerlineSegments,e=asset.arrays.centerlineEdges;
    const segments=Array.from({length:e.length/2},(_,i)=>({id:i,start:new THREE.Vector3().fromArray(d,i*9),end:new THREE.Vector3().fromArray(d,i*9+3),radiusStart:d[i*9+6],radiusEnd:d[i*9+7],nodeStartId:e[2*i],nodeEndId:e[2*i+1]}));
    for(const r of asset.metadata.renal.segments)Object.assign(segments[r.index],{renalSide:r.side,renalSector:r.sector,renalHilar:r.hilar,renalLateral:r.lateral});
    const contrast=new HybridContrastSystem({centerlineSegments:segments,contactField:field});
    assert.equal(contrast.renalPerfusion.kidneys.length,2);
    assert.equal(contrast.flowNetwork.sourceSegments.filter(s=>s.renalSide).length,asset.metadata.renal.segments.length,'no duplicate visual-only renal tree');
    for(const side of ['left','right']) {
        const renal=segments.filter(s=>s.renalSide===side),trunk=renal.find(s=>s.renalSector===-1);
        const kidney=contrast.renalPerfusion.kidneys.find(k=>k.side===side);
        const parents=new Map(renal.map(s=>[s.nodeEndId,s]));
        const lateralTakeoffs=renal.filter(s=>s.renalLateral&&!parents.get(s.nodeStartId)?.renalLateral);
        assert.ok(lateralTakeoffs.length>=15,'the collision anatomy must contain lateral takeoffs');
        for(const lateral of lateralTakeoffs)
            assert.ok(renal.some(s=>s.nodeStartId===lateral.nodeStartId&&!s.renalLateral),
                'each lateral branch must share its takeoff with the continuing main artery');
        const firstDivision=renal.find(s=>s.renalHilar);
        assert.ok(firstDivision,'collision asset must include the separate hilar branches');
        assert.ok(renalSphereLevelBound(firstDivision.start,0,kidney)>1.1,
            'the mechanical anatomy must divide before entering the parenchyma');
        const sites=[trunk,...[0,2,4].map(sector=>{
            const junction=renal.find(s=>s.renalSector===sector).start;
            // Start within each daughter, beyond the shared hilar lumen, so
            // this actually exercises selective branch catheterization.
            return renal.find(s=>s.renalSector===sector&&s.start.distanceTo(junction)>8);
        })];
        for(const [index,site] of sites.entries()) {
            const direction=site.end.clone().sub(site.start).normalize();
            const start=site.start.clone().addScaledVector(direction,index===0?-5:-1);
            const end=start.clone().addScaledVector(direction,2);
            const sheath={start,end,innerRadius:1,proximalExtension:40};
            let state=createSharedAxisNative({...createSharedAxisContacts({sheath,contactField:field,localCoordinates:true}),spacing:1,
                tools:[{id:'wire',type:'glidewire',length:2000,insertion:2,radius:.4445},{id:'catheter',type:'straight',insertion:1,radius:5/6}]});
            for(let insertion=2;insertion<=13;insertion+=.5) {
                const tools=state.materials.map(m=>({...m.spec,rotation:0,insertion:m.spec.id==='wire'?insertion:Math.max(1,insertion-1)}));
                const iterator=advanceSharedAxis(state,{wire:0,catheter:0},1/60,tools,SHARED_AXIS_APP_PHYSICS_OPTIONS);
                let step;do{step=iterator.next();}while(!step.done);
                assert.ok(step.value.state,`${side} site ${index}, feed ${insertion}: ${JSON.stringify(step.value.result)}`);
                state=step.value.state;
            }
            const tip=state.positions.at(-1).map((v,k)=>v+state.origin[k]);
            assert.ok(new THREE.Vector3(...tip).sub(site.start).dot(direction)>6,'tip must cross the old terminal or branch junction');
            assert.ok(field.querySphere(tip,.4445).signedGap>-.01,'tip is inside the actual hollow mesh');
        }
        for(const segment of renal) {
            const midpoint=segment.start.clone().lerp(segment.end,.5);
            assert.ok(field.querySphere(midpoint,.1).signedGap>0,
                `${side} segment ${segment.id}: the entire renal centerline must have an open lumen`);
        }
        // A wall is still present: moving a probe away from a distal artery is
        // rejected, rather than making the entire kidney an allowed domain.
        const small=renal.find(s=>s.radiusEnd<.7),mid=small.start.clone().lerp(small.end,.5);
        assert.ok(field.querySphere(mid,.2).signedGap>0);
        assert.ok(field.querySphere(mid.clone().add(new THREE.Vector3(0,0,30)),.2).signedGap<0);
    }
    g.dispose();
});
