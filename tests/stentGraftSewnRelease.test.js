import {readFileSync} from 'node:fs';
import {gunzipSync} from 'node:zlib';
import {graftBranchContactAt} from '../src/devices/stentGraftBranchContact.js';
import test from 'node:test';
import assert from 'node:assert/strict';
import {Vector3} from 'three';
import {advanceRelease,sewnRingExposure} from '../src/devices/stentGraftDeployment.js';
import {fabricPoint} from '../src/devices/stentGraftScaffold.js';
import {previewFixture} from './helpers/stentGraftPreviewFixture.js';

const ringDevice=()=>({type:'body',phase:'deploying',position:100,implantPosition:100,sheathWithdrawal:0,sheathTravel:40,coverLead:0,tipRelease:0,
 releaseLength:30,gateOpening:0,ipsiOpening:0,parts:[{releaseOffset:0,path:{length:30},scaffoldRings:[{center:6,maxHeight:10},{center:18,maxHeight:10}]}]});
test('a sewn ring stays crimped until its entire band leaves the cover, then opens while cover is stopped',()=>{
 const d=ringDevice(),part=d.parts[0];
 advanceRelease(d,10.9/12,'sheath');assert.equal(part.scaffoldRings[0].opening,0);
 advanceRelease(d,.1/12,'sheath');assert.ok(part.scaffoldRings[0].opening<1e-9);
 advanceRelease(d,.05,null);assert.ok(part.scaffoldRings[0].opening>0&&part.scaffoldRings[0].opening<1);
 for(const s of [1,3,6,9,11])assert.equal(sewnRingExposure(part,s),part.scaffoldRings[0].opening,'one opening degree of freedom for the whole metal band');
 assert.equal(part.scaffoldRings[1].opening,0,'covered neighbour remains crimped');
 advanceRelease(d,.2,null);assert.equal(part.scaffoldRings[0].opening,1);
});
test('ring release uses committed time consistently including a clamped cover endpoint',()=>{
 const a=ringDevice(),b=ringDevice();advanceRelease(a,4,'sheath');
 for(let i=0;i<240;i++)advanceRelease(b,1/60,'sheath');
 assert.deepEqual(a.parts[0].scaffoldRings.map(r=>r.opening),b.parts[0].scaffoldRings.map(r=>r.opening));
});
for(const side of ['left','right'])test(`${side}: progressive leg rings retain sewn material positions and bounded spacing`,()=>{
 const {system,device:d}=previewFixture(side),saved=d.parts.map(p=>p.scaffoldRings.map(r=>r.center));
 let gateProgressive=false,gateEndFree=false;
 try{
 for(let step=0;step<150;step++){
  system.updateAccess(side,.1,null,{deviceId:d.id,release:'sheath'});
  assert.deepEqual(d.parts.map(p=>p.scaffoldRings.map(r=>r.center)),saved);
  const rings=d.parts[2].scaffoldRings;
  gateProgressive||=rings[0].opening===1&&rings.at(-1).opening===0;
  gateEndFree||=rings.at(-1).opening===1;
  for(const p of d.parts.slice(1)){
   const centers=p.scaffoldRings.map(r=>{
    const c=new Vector3();for(let j=0;j<p.sides;j++)c.add(fabricPoint(p,r.center,j/p.sides*Math.PI*2));return c.divideScalar(p.sides);
   });
   for(let i=1;i<centers.length;i++)assert.ok(centers[i].distanceTo(centers[i-1])<=1.1*(p.scaffoldRings[i].center-p.scaffoldRings[i-1].center),'no growing gaps between neighbouring sewn rings');
  }
 }
 assert.ok(gateProgressive&&gateEndFree,'proximal gate rings open progressively before the distal end springs free');
 }finally{system.dispose();}
});

test('rounding deployment to 100 percent cannot complete an attached or still-expanding implant',()=>{
 const {system,device:d}=previewFixture();
 try {
  system.updateAccess('right',100,null,{deviceId:d.id,release:'sheath'});
  d.tipRelease=1-Number.EPSILON;
  system.updateAccess('right',1/60);
  assert.equal(d.phase,'deploying');assert.equal(d.releaseStage,'tip');
  system.updateAccess('right',1/60,null,{deviceId:d.id,release:'tip'});
  assert.equal(d.phase,'deployed');assert.ok(d.parts.every(p=>p.scaffoldRings.every(r=>r.opening===1)));
 }finally{system.dispose();}
});

// Actual partially released left limb. Individual face reorientation used to
// reverse the contact side at 20–45% and 80–90% along this accepted rod segment.
test('folded branch preserves coherent inward contact across concave fabric faces',()=>{
 const {branch,a,b}=JSON.parse(gunzipSync(readFileSync(new URL('./fixtures/shared-axis/stent-graft-folded-branch.json.gz',import.meta.url))));
 for(const t of [0,.1,.25,.4,.6,.85,1]) {
  const q=graftBranchContactAt([branch],a.map((v,k)=>v+(b[k]-v)*t),.8);
  assert.ok(q);assert.equal(q.outside,t<.2,`correct fabric side at ${t}`);
 }
});
