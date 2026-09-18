import {readFileSync} from 'node:fs';
import {gunzipSync} from 'node:zlib';
const source=new URL('../adaptive-optimization-audit-2026-09-17/full/catheter.cpuprofile.gz',import.meta.url);
const p=JSON.parse(gunzipSync(readFileSync(source)));
const nodes=new Map(p.nodes.map(n=>[n.id,n])),parents=new Map();
for(const n of p.nodes)for(const id of n.children??[])parents.set(id,n.id);
const categories={discovery:0,retainedGeometry:0,otherConstraints:0,material:0,friction:0,inertia:0,otherAssembly:0};
let total=0;
for(let i=0;i<p.samples.length;i++) {
    let id=p.samples[i];const stack=[];
    while(id!==undefined) {
        const n=nodes.get(id);stack.push(n.callFrame.functionName+'@'+n.callFrame.url.split('/').at(-1));id=parents.get(id);
    }
    if(!stack.includes('assembleSharedAxisNative@kirchhoffSharedAxisNative.js'))continue;
    const has=prefix=>stack.some(s=>s.startsWith(prefix));
    const category=has('assembleSharedAxisConstraintRows@')?
        (has('sample@kirchhoffSharedAxisVesselWitnesses')?'discovery':has('gapForWitness@')?'retainedGeometry':'otherConstraints'):
        has('assembleSharedAxisMaterialTangent@')||has('assembleSharedAxisGaussNewton@')?'material':
        has('assembleSharedAxisWallFriction@')?'friction':has('assembleSharedAxisInertia@')?'inertia':'otherAssembly';
    categories[category]+=p.timeDeltas[i];total+=p.timeDeltas[i];
}
console.log(JSON.stringify({source:source.pathname,scope:'Exclusive partition of samples with assembleSharedAxisNative in their stack; catheter phase of the earlier adaptive audit',
    sampledAssemblyMs:total/1000,categories:Object.fromEntries(Object.entries(categories).map(([k,v])=>[k,{ms:v/1000,percent:100*v/total}]))},null,2));
