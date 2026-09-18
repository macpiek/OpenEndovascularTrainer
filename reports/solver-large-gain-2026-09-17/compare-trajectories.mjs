import assert from 'node:assert/strict';
import {readFileSync} from 'node:fs';
import {gunzipSync} from 'node:zlib';
const read=p=>JSON.parse(p.endsWith('.gz')?gunzipSync(readFileSync(p)):readFileSync(p));
const [a,b]=process.argv.slice(2).map(read);
assert.equal(a.failed,false);assert.equal(b.failed,false);assert.equal(a.samples.length,b.samples.length);
const sample=(shape,x)=>{
    const {coordinates:xs,positions:ps}=shape;
    let i=0;while(i+1<xs.length-1&&xs[i+1]<x)i++;
    const t=(x-xs[i])/(xs[i+1]-xs[i]);return ps[i].map((v,k)=>(1-t)*v+t*ps[i+1][k]);
};
const phases={};let worst=null;
for(let i=0;i<a.samples.length;i++) {
    const r=a.samples[i],s=b.samples[i];
    assert.deepEqual([r.phase,r.wire,r.catheter],[s.phase,s.wire,s.catheter]);
    assert.ok(s.converged&&s.quality.finite);
    assert.ok(s.certificateBound<=b.modelParameters.options.forceTolerance);
    assert.ok(s.residual.length<=b.modelParameters.options.lengthTolerance);
    const tip=Math.max(...r.toolTips.map((p,j)=>Math.hypot(...p.position.map((v,k)=>v-s.toolTips[j].position[k]))));
    let shape=null;
    if(r.shape&&s.shape)shape=Math.max(...[...new Set([...r.shape.coordinates,...s.shape.coordinates])].map(x=>{
        const p=sample(r.shape,x),q=sample(s.shape,x);return Math.hypot(...p.map((v,k)=>v-q[k]));
    }));
    const p=phases[r.phase]??={steps:0,maxTipDeviationMm:0,maxShapeDeviationMm:0,maxPenetrationMm:0,maxCertificate:0,referenceMs:0,optimizedMs:0};
    p.steps++;p.maxTipDeviationMm=Math.max(p.maxTipDeviationMm,tip);p.maxShapeDeviationMm=Math.max(p.maxShapeDeviationMm,shape??0);
    p.maxPenetrationMm=Math.max(p.maxPenetrationMm,s.quality.maxPenetration);p.maxCertificate=Math.max(p.maxCertificate,s.certificateBound);
    p.referenceMs+=r.totalMs;p.optimizedMs+=s.totalMs;
    if(!worst||(shape??tip)>worst.deviationMm)worst={index:i,phase:r.phase,wire:r.wire,catheter:r.catheter,deviationMm:shape??tip};
}
console.log(JSON.stringify({referenceHash:a.sourceTreeHash,experimentHash:b.sourceTreeHash,steps:a.samples.length,
    note:'Independent trajectories: max shape difference uses the union of both piecewise-linear mechanical grids. Timing is sequential, not paired.',worst,phases},null,2));
