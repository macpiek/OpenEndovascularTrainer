import {readFileSync} from 'node:fs';
import {gunzipSync} from 'node:zlib';
import {graftBranchContactAt} from '../src/devices/stentGraftBranchContact.js';
import test from 'node:test';
import assert from 'node:assert/strict';
import {Vector3} from 'three';
import {advanceRelease,sewnRingExposure,PARTIAL_RING_OPENING} from '../src/devices/stentGraftDeployment.js';
import {fabricPoint} from '../src/devices/stentGraftScaffold.js';
import {previewFixture} from './helpers/stentGraftPreviewFixture.js';

const ringDevice=()=>({type:'body',phase:'deploying',position:100,implantPosition:100,sheathWithdrawal:0,sheathTravel:40,coverLead:0,tipRelease:0,
 releaseLength:30,gateOpening:0,ipsiOpening:0,parts:[{releaseOffset:0,path:{length:30},scaffoldRings:[{center:6,maxHeight:10},{center:18,maxHeight:10}]}]});
test('uncovered struts flare before full ring release, while covered rows stay crimped',()=>{
 const d=ringDevice(),part=d.parts[0];
 advanceRelease(d,6/12,'sheath');
 const partial=sewnRingExposure(part,2);
 assert.ok(partial>0&&partial<=PARTIAL_RING_OPENING);
 assert.ok(sewnRingExposure(part,1)>.75,'free upper end must open substantially at half release');
 assert.ok(sewnRingExposure(part,1)>partial&&partial>sewnRingExposure(part,4),'funnel tapers toward the constrained sleeve lip');
 for(const s of [6,9,11,18])assert.equal(sewnRingExposure(part,s),0,'sleeve still constrains covered material');
 advanceRelease(d,1,null);assert.equal(sewnRingExposure(part,2),partial,'a captured ring cannot fully expand just by waiting');
 advanceRelease(d,4.999/12,'sheath');
 const before=[1,3,6,9,11].map(s=>sewnRingExposure(part,s));
 advanceRelease(d,.001/12,'sheath');
 for(const [i,s] of [1,3,6,9,11].entries())assert.ok(Math.abs(sewnRingExposure(part,s)-before[i])<.001,'no release-boundary pop');
 advanceRelease(d,.05,null);assert.ok(part.scaffoldRings[0].opening>0&&part.scaffoldRings[0].opening<1);
 assert.equal(part.scaffoldRings[1].opening,0,'covered neighbour remains crimped');
 assert.equal(sewnRingExposure(part,12),0,'inter-ring fabric still under the sleeve stays crimped');
 advanceRelease(d,.2,null);
 for(const s of [1,3,6,9,11])assert.equal(sewnRingExposure(part,s),1,'freed ring finishes expanding with a stationary sleeve');
});
test('partial ring flare reverses with the cover and ignores uncommitted time',()=>{
 const d=ringDevice(),part=d.parts[0];advanceRelease(d,8/12,'sheath');
 const before=structuredClone(d);advanceRelease(d,0,'sheath');assert.deepEqual(d,before);
 advanceRelease(d,8/12,'resheath');
 for(const s of [0,1,3,6,11,18,30])assert.equal(sewnRingExposure(part,s),0);
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
  // Removing capture can expose a new equilibrium. The committed cloth
  // approaches it at bounded speed before publishing the final solid.
  if(d.releaseMotionLimited)assert.equal(d.phase,'deploying');
  for(let i=0;i<300&&d.phase!=='deployed';i++)system.updateAccess('right',1/60);
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

test('partially uncovered ring flares around the delivery axis without squeezing the cover',async()=>{
 const {graftOwnedBranches}=await import('../src/devices/stentGraftBranchContact.js');
 const {system,device:d}=previewFixture('right','limb');
 try {
  const p=d.parts[0],ring=p.scaffoldRings[0];
  system.updateAccess('right',(ring.center+ring.maxHeight*.3)/12,null,{deviceId:d.id,release:'sheath'});
  assert.ok(p.exposure[0]>.8&&p.exposure[0]<1);
  const center=new Vector3();for(let j=0;j<p.sides;j++)center.add(fabricPoint(p,0,j/p.sides*Math.PI*2));center.divideScalar(p.sides);
  assert.ok(center.distanceTo(p.folded[0])<1e-5,'the still-supported axis cannot drift toward the released target');
  assert.ok(fabricPoint(p,0,0).distanceTo(center)>p.packedLayout.radius*2,'uncovered top flares while sewn wire length limits its reach');
  assert.ok(Math.abs(ring.lengthError)<1e-4,'partial flaring cannot stretch the metal');
  assert.equal(ring.currentHeight,ring.height,'radial flaring cannot slide the crowns along the cloth');
  const coveredRow=p.path.coordinates.findIndex(s=>s>=d.sheathWithdrawal);
  assert.equal(p.exposure[coveredRow],0,'trailing material is still retained');
  const coveredCenter=p.folded[coveredRow],coveredS=p.path.coordinates[coveredRow];
  for(let j=0;j<p.sides;j++)assert.ok(Math.abs(fabricPoint(p,coveredS,j/p.sides*Math.PI*2).distanceTo(coveredCenter)-p.packedLayout.radius)<1e-5,'covered bottom keeps the packed radius');
  assert.equal(graftOwnedBranches(d).length,0,'the captured ring cannot collide with the larger delivery cover inside it');
  system.updateAccess('right',1,null,{deviceId:d.id,release:'sheath'});
  assert.ok(graftOwnedBranches(d).length>0,'free rings regain normal lumen contacts');
 }finally{system.dispose();}
});
