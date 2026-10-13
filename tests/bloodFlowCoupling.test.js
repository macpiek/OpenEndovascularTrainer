import test from 'node:test';
import assert from 'node:assert/strict';
import {createSharedAxisNative,relaxSharedAxisNative} from '../src/physics/kirchhoffSharedAxisNative.js';
import {prepareSharedAxisDynamicStep,completeSharedAxisDynamicStep} from '../src/physics/kirchhoffSharedAxisDynamics.js';
import {captureBloodFlow} from '../src/physics/bloodFlowCoupling.js';

const network=(velocity=[0,200,0])=>({time:2,hemodynamics:{bloodDensityKgPerM3:1060,bloodViscosityPaS:.0035},
    sampleVelocityCoordinates(x,y,z,out){return Object.assign(out,{edgeIndex:0,distance:0,radius:5,velocityX:velocity[0],velocityY:velocity[1],velocityZ:velocity[2]});}});
const rod=(tools=[{id:'wire',insertion:30,type:'glidewire'}],spacing=5)=>createSharedAxisNative({tools,spacing});
const totalDrag=s=>s.dynamicStep.bloodFlow.weights.reduce((a,b)=>a+b,0);

test('blood snapshot uses world coordinates, is serializable and independent of subsequent pulses',()=>{
    const s=rod();s.origin=[10,20,30];const n=network(),points=[];
    const sample=n.sampleVelocityCoordinates;
    n.sampleVelocityCoordinates=(x,y,z,out)=>{points.push([x,y,z]);sample(x,y,z,out);if(x>40)out.distance=20;};
    const flow=captureBloodFlow(s,n,10,35),saved=structuredClone(flow);
    assert.deepEqual(points[0],[20,20,30]);assert.deepEqual(flow.velocities.at(-1),[0,0,0]);
    n.time=10;n.hemodynamics.bloodDensityKgPerM3=2000;
    assert.deepEqual(flow,saved);assert.deepEqual(JSON.parse(JSON.stringify(flow)),saved);
});

test('only exposed outer material receives flow, including a partial sheath interval',()=>{
    const catheter={id:'catheter',insertion:30,type:'straight'};
    const solo=rod([catheter]),nested=rod([{id:'wire',insertion:30},catheter]);
    for(const s of [solo,nested])prepareSharedAxisDynamicStep(s,1/120,captureBloodFlow(s,network(),7.5));
    assert.equal(solo.dynamicStep.bloodFlow.exposedLength,22.5);
    assert.ok(Math.abs(totalDrag(solo)-totalDrag(nested))<1e-12,'contained wire does not double external drag');
    const shielded=rod();prepareSharedAxisDynamicStep(shielded,1/120,captureBloodFlow(shielded,network(),40));
    assert.equal(totalDrag(shielded),0);
});

test('uniform flow drag is invariant under mechanical mesh subdivision and stronger across the tool',()=>{
    const values=[2.5,5,10].map(spacing=>{const s=rod(undefined,spacing);prepareSharedAxisDynamicStep(s,1/120,captureBloodFlow(s,network(),0));return totalDrag(s);});
    assert.ok(Math.max(...values)-Math.min(...values)<1e-12);
    const s=rod();prepareSharedAxisDynamicStep(s,1/120,captureBloodFlow(s,network([200,0,0]),0));
    assert.ok(totalDrag(s)<values[0]*.1);
});

test('implicit drag stays bounded at large timesteps and damps a tool moving through still blood',()=>{
    for(const dt of [1/120,1/30,.5]){
        const s=rod();prepareSharedAxisDynamicStep(s,dt,captureBloodFlow(s,network([0,2000,0]),0));
        for(let i=0;i<s.positions.length;i++){
            const speed=(s.dynamicStep.predicted[i][1]-s.positions[i][1])/dt;
            assert.ok(speed>0&&speed<2000);
        }
    }
    const a=rod(),b=rod();for(const s of [a,b])s.velocities=s.positions.map(()=>[0,100,0]);
    prepareSharedAxisDynamicStep(a,1/120);prepareSharedAxisDynamicStep(b,1/120,captureBloodFlow(b,network([0,0,0]),0));
    assert.ok(b.dynamicStep.predicted.at(-1)[1]<a.dynamicStep.predicted.at(-1)[1]);
});

test('flow enters the solved mechanical residual and transfers positive transverse momentum',()=>{
    const s=rod();s.fixed.fill(0);const before=s.positions.map(p=>p.slice()),dt=1/120;
    prepareSharedAxisDynamicStep(s,dt,captureBloodFlow(s,network(),0));
    const r=relaxSharedAxisNative(s);assert.ok(r.converged,JSON.stringify(r));
    const displacement=s.positions.reduce((sum,p,i)=>sum+p[1]-before[i][1],0)/s.positions.length;
    assert.ok(displacement>1e-5&&displacement<200*dt);
    completeSharedAxisDynamicStep(s);assert.equal(s.bloodFlowDiagnostics.maxSpeed,200);
    assert.equal(s.bloodFlowDiagnostics.time,2);
});

test('existing vessel network supplies pulsatile forcing without any contrast injection',async()=>{
    const {Vector3}=await import('three');
    const {ContrastFlowNetwork}=await import('../src/contrast/flowNetwork.js');
    const n=new ContrastFlowNetwork([{id:0,nodeStartId:0,nodeEndId:1,start:new Vector3(),end:new Vector3(100,0,0),radiusStart:5,radiusEnd:5}],{rootPoint:new Vector3()});
    const s=rod(),speeds=[];
    for(let i=0;i<60;i++){n.update(1/120);const flow=captureBloodFlow(s,n,0);speeds.push(flow.velocities[2][0]);}
    assert.ok(Math.min(...speeds)>0);assert.ok(Math.max(...speeds)>Math.min(...speeds)*1.5);
    assert.equal(n.getIodineMassMg(),0,'blood movement does not require contrast');
});

test('projective solver receives the same drag and retains its stability checks',async()=>{
    const {iterateSharedAxisProjective}=await import('../src/physics/kirchhoffSharedAxisProjective.js');
    const s=rod();s.fixed.fill(0);const before=s.positions.map(p=>p.slice());
    const iterator=iterateSharedAxisProjective(s,1/120,{bloodFlow:captureBloodFlow(s,network(),0)});
    let result;do{result=iterator.next();}while(!result.done);
    assert.ok(result.value.converged,JSON.stringify(result.value));
    assert.ok(s.positions.reduce((sum,p,i)=>sum+p[1]-before[i][1],0)>0);
    assert.equal(s.bloodFlowDiagnostics.maxSpeed,200);
});
