import test from 'node:test';
import assert from 'node:assert/strict';
import * as THREE from 'three';
import {loadCoupledRuntimeAnatomy} from './helpers/coupledRuntimeFixture.js';
import {sampleCompliantBalloon} from '../src/devices/compliantBalloon.js';

test('inflated membrane stays inside the actual abdominal aortic lumen',async()=>{
    const anatomy=await loadCoupledRuntimeAnatomy(undefined,'baseline');
    try {
        const data=anatomy.field.centerline,stride=anatomy.field.centerlineStride,points=[];
        for(let i=0;i<data.length;i+=stride)if(data[i+1]>-240&&data[i+1]<-80&&data[i+6]>9&&Math.abs(data[i])<30)
            points.push(new THREE.Vector3(data[i],data[i+1],data[i+2]));
        points.sort((a,b)=>a.y-b.y);const ordered=points.filter((p,i)=>!i||p.y-points[i-1].y>.05);
        assert.ok(ordered.length>10);
        const curve=new THREE.CatmullRomCurve3(ordered),ray=new THREE.Ray(),tree=anatomy.geometry.boundsTree;
        const sample=sampleCompliantBalloon({path:curve,coordinates:[0,curve.getLength()],volumeMl:60,wallDistance:(center,direction)=>{
            ray.origin.copy(center);ray.direction.copy(direction);return tree.raycastFirst(ray,THREE.DoubleSide,0,100)?.distance??NaN;
        }});
        assert.ok(sample&&!sample.invalid);assert.ok(sample.limited);assert.ok(sample.volumeMl>1&&sample.volumeMl<60);
        for(let i=0;i<sample.points.length;i+=3){
            const c=Math.floor(i/3/sample.radialSegments)*3;
            ray.origin.fromArray(sample.centers,c);ray.direction.fromArray(sample.points,i).sub(ray.origin);
            const distance=ray.direction.length();ray.direction.normalize();
            assert.equal(tree.raycastFirst(ray,THREE.DoubleSide,.001,distance-.001),null,'membrane must not cross the anatomical wall');
        }
    }finally{anatomy.dispose();}
});

test('balloon inflation couples to the native rods and wall in the anatomical aorta',async()=>{
    const {createSharedAxisNative,feedSharedAxisNative}=await import('../src/physics/kirchhoffSharedAxisNative.js');
    const {createSharedAxisVesselDiscovery}=await import('../src/physics/kirchhoffSharedAxisVesselWitnesses.js');
    const {stepSharedAxis}=await import('../src/physics/kirchhoffSharedAxisTimeStep.js');
    const {DEFAULT_VESSEL_ELASTICITY}=await import('../src/physics/vesselElasticity.js');
    const {SHARED_AXIS_APP_PHYSICS_OPTIONS}=await import('../src/physics/kirchhoffSharedAxisAppSystem.js');
    const {RELIANT}=await import('../src/devices/balloonProfile.js');
    const anatomy=await loadCoupledRuntimeAnatomy(undefined,'baseline');
    try {
        const data=anatomy.field.centerline,stride=anatomy.field.centerlineStride,points=[];
        for(let i=0;i<data.length;i+=stride)if(data[i+1]>-240&&data[i+1]<-80&&data[i+6]>9&&Math.abs(data[i])<30)
            points.push(new THREE.Vector3(data[i],data[i+1],data[i+2]));
        points.sort((a,b)=>a.y-b.y);const curve=new THREE.CatmullRomCurve3(points.filter((p,i)=>!i||p.y-points[i-1].y>.05));
        const length=curve.getLength();
        let s=createSharedAxisNative({tools:[{id:'wire',type:'glidewire',insertion:length-1,shaftStiffness:13.15,tipStiffness:19.4},
            {id:'catheter',type:RELIANT.type,insertion:length-10,radius:RELIANT.shaftRadius,shaftStiffness:40.65,tipStiffness:100,balloonInflationMl:0,wallStaticFriction:.2,wallKineticFriction:.15}],
            samplePosition:x=>x===5?curve.getPointAt(0).add(curve.getPointAt(5/length).sub(curve.getPointAt(0)).normalize().multiplyScalar(5)).toArray():curve.getPointAt(x/length).toArray(),wallSamples:[createSharedAxisVesselDiscovery(anatomy.field,0)]});
        const options={...SHARED_AXIS_APP_PHYSICS_OPTIONS,vesselElasticity:DEFAULT_VESSEL_ELASTICITY};
        let moved=false,wall=false,compressed=false,previous;
        for(const v of [0,.5,1,2,3,4,6,8,10,12]) {
            const source={...s,materials:s.materials.map(m=>({...m,spec:{...m.spec,...(m.spec.id==='catheter'?{balloonInflationMl:v}:{})}}))};
            s=feedSharedAxisNative(source,{});
            const r=stepSharedAxis(s,1/60,options);
            assert.ok(r.converged,`${v}: ${JSON.stringify(r)}`);
            assert.ok(r.quality.maxPenetration<1e-5);
            wall ||=s.vesselElasticPatches.some(p=>p.displacement>.001);
            compressed ||=r.quality.maxBalloonCompression>.1;
            if(previous)moved ||=s.positions.some((p,i)=>Math.hypot(...p.map((x,k)=>x-previous[i][k]))>.01);
            previous=s.positions.map(p=>p.slice());
        }
        assert.ok(moved&&wall&&compressed);
    }finally{anatomy.dispose();}
});

test('recorded iliac bend accepts inflation and deflation without weakening force certificates',async()=>{
    const {readFileSync}=await import('node:fs'),{gunzipSync}=await import('node:zlib');
    const {restoreSharedAxisReplay}=await import('../src/physics/kirchhoffSharedAxisRestore.js');
    const {advanceSharedAxis}=await import('../src/physics/kirchhoffSharedAxisAppSystem.js');
    const fixture=JSON.parse(gunzipSync(readFileSync(new URL('./fixtures/shared-axis/reliant-inflation-iliac.json.gz',import.meta.url))));
    const anatomy=await loadCoupledRuntimeAnatomy(undefined,'baseline');
    try {
        const request=fixture.stepRequest;
        let state=restoreSharedAxisReplay(fixture,anatomy.field),rotations=request.rotations,maxWall=0;
        const before=state.positions.map(p=>p.slice());
        for(let i=0;i<30;i++) {
            const fill=i<20?12.6+i*.1:Math.max(0,14.5-(i-19)*1.5);
            const tools=request.tools.map(t=>({...t,...(t.id==='catheter'?{balloonInflationMl:fill}:{})}));
            const iterator=advanceSharedAxis(state,rotations,request.dt,tools,request.options),started=performance.now();
            let next;
            do {next=iterator.next();if(!next.done&&performance.now()-started>10000){iterator.return();assert.fail(`Inflation step ${i} exceeded work limit`);}}while(!next.done);
            assert.ok(next.value.state,JSON.stringify(next.value.result));
            const r=next.value.result;
            assert.ok(r.certificateBound<=1e-6&&r.residual.force<=1e-6&&r.residual.torque<=1e-6);
            assert.ok(r.quality.finite&&r.quality.maxPenetration<=1e-5);
            for(const body of r.quality.bodies)assert.ok(body.maxLengthError<=1e-5);
            state=next.value.state;rotations=next.value.rotations;
            maxWall=Math.max(maxWall,r.quality.maxWallDisplacement);
        }
        assert.ok(maxWall>.05);
        assert.equal(state.materials.find(m=>m.spec.id==='catheter').spec.balloonInflationMl,0);
        assert.ok(state.positions.some((p,i)=>Math.hypot(...p.map((x,k)=>x-before[i][k]))>.01));
    }finally{anatomy.dispose();}
});
