import test,{after} from 'node:test';
import assert from 'node:assert/strict';
import {readFileSync} from 'node:fs';
import {gunzipSync} from 'node:zlib';
import {loadCoupledRuntimeAnatomy} from './helpers/coupledRuntimeFixture.js';
import {restoreSharedAxisReplay} from './helpers/sharedAxisReplay.js';
import {advanceSharedAxis} from '../src/physics/kirchhoffSharedAxisAppSystem.js';
import {createContactResult} from '../src/physics/collision/vesselContactField.js';

const fixture=JSON.parse(gunzipSync(readFileSync(new URL('./fixtures/shared-axis/anatomy-wire-569.80-poor-prediction.json.gz',import.meta.url))));
const anatomy=await loadCoupledRuntimeAnatomy();after(()=>anatomy.dispose());
const snapshot=(c,t)=>({t,face:c.faceIndex,distance:c.signedDistance,gap:c.signedGap,inside:c.inside,source:c.source});
test('certified samples preserve exact mesh distances, face selection and endpoint-first order without SDF work',()=>{
    const field=anatomy.field;
    for(const entry of fixture.insideContinuation.filter(e=>e.clearance>2).slice(0,20)) {
        const a=entry.point.slice(),b=a.map((v,k)=>v+(k===0?.01:0)),radius=.4445,sampleCount=3;
        const descriptor={a,b,radius,sampleCount,geometryToken:field.fallbackGeometry.boundsTree};
        const expected=[],actual=[];
        field.queryCapsuleSoA([a[0],b[0]],[a[1],b[1]],[a[2],b[2]],[radius,radius],0,createContactResult(),-1,true,false,-1,false,.01,sampleCount,true,true,(c,t)=>expected.push(snapshot(c,t)));
        const stats=field.getStats();
        assert.equal(field.visitCertifiedInsideCapsule(descriptor,{knownInside:true,lowerBound:entry.clearance-.01},(c,t)=>actual.push(snapshot(c,t))),true);
        assert.deepEqual(actual,expected);
        // SDF hits and sign-cache queries must not increase on a geometric proof.
        for(const key of ['sdfHits','signRefinements','signCacheHits','signCacheMisses'])assert.equal(field.getStats()[key],stats[key]);
        assert.equal(field.getStats().certifiedCapsuleSamples-stats.certifiedCapsuleSamples,sampleCount+1);
        assert.equal(field.visitCertifiedInsideCapsule(descriptor,{knownInside:false,lowerBound:1},()=>assert.fail()),false);
        assert.equal(field.visitCertifiedInsideCapsule({...descriptor,geometryToken:{}},{knownInside:true,lowerBound:1},()=>assert.fail()),false);
    }
});

test('certified-only discovery preserves a captured nonlinear solve and its physical checkpoint',()=>{
    const results=[];
    for(const certifiedDiscoverySamples of [false,true]) {
        const input=restoreSharedAxisReplay({...fixture,certifiedDiscoverySamples},anatomy.field),req=fixture.stepRequest;
        const iterator=advanceSharedAxis(input,req.rotations,req.dt,req.tools,{...req.options,earlyPredictorFallback:true});
        let next;do{next=iterator.next();}while(!next.done);
        const {state,result}=next.value;assert.ok(state&&result.converged,JSON.stringify(result));
        results.push({result,positions:state.positions,velocities:state.velocities,multipliers:Array.from(state.multipliers),
            frames:state.materials.map(m=>['orientationX','orientationY','orientationZ','orientationW'].map(k=>Array.from(m.body[k])))});
    }
    const [reference,actual]=results;
    for(const key of ['positions','velocities','multipliers','frames'])assert.deepEqual(actual[key],reference[key],key);
    for(const key of ['iterations','factorizations','fullAssemblies','residualAssemblies','quality','residual','subdivisions'])
        assert.deepEqual(actual.result[key],reference.result[key],key);
});
