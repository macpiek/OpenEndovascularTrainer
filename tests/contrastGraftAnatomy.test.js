import test from 'node:test';
import assert from 'node:assert/strict';
import {fixture,place,finish} from './helpers/stentGraftFixture.js';
import {HybridContrastSystem} from '../src/contrast/hybridContrastSystem.js';

test('aneurysm atlas: graft exclusion preserves both original femoral territories and iodine balance',()=>{
 const f=fixture(),s=new HybridContrastSystem({centerlineSegments:f.segments,contactField:f.contactField}),n=s.flowNetwork;
 // Keep the original edge identities: searching again after exclusion can
 // silently select a different artery and hide an accidental femoral closure.
 const outlets=['left','right'].map(side=>n.findNearestLocation(f.system.getPath(side).points[0],{}).edgeIndex);
 try {
  place(f.system,'right','body');f.system.deploy('right');finish(f.system,'right');s.setStentGraftSurface(f.system.surface);
  assert.equal(n.stentGraftRemodeling.surface.sealed,false);
  assert.ok(n.stentGraftRemodeling.sac,'actual open contralateral gate maps to its own lumen');
  const firstCovered=n.edges.find(e=>e.graftCovered);
  n.depositIodine(firstCovered.index,0,25);
  for(let i=0;i<180;i++)n.update(1/30);
  assert.ok(n.stentGraftRemodeling.sac.receivedMassMg>0,'iodine exits the actual gate into the sac');
  assert.ok(n.stentGraftRemodeling.trappedIodineMassMg<2,'patent sac must wash out after a bolus, not retain nearly all iodine for a minute');
  assert.ok(Math.abs(n.totalIodineMassMg+n.outletIodineMassMg+n.stentGraftRemodeling.trappedIodineMassMg-25)<1e-7);
  place(f.system,'left','limb');f.system.deploy('left');finish(f.system,'left');s.setStentGraftSurface(f.system.surface);
  assert.equal(n.stentGraftRemodeling.surface.sealed,true);
  for(const index of outlets){assert.equal(n.edges[index].transportExcluded,false);assert.ok(n.edges[index].meanFlowMm3PerS>0);}
  assert.ok(n.stentGraftRemodeling.coveredEdges>100);
  assert.ok(n.stentGraftRemodeling.excludedEdges>0);
  const inlet=n.edges.find(e=>e.graftCovered);n.depositIodine(inlet.index,0,100);
  for(let i=0;i<60;i++)n.update(1/30);
  for(const index of n.stentGraftRemodeling.excludedEdgeIndices)assert.ok(n.edges[index].massMg.every(v=>v===0));
  assert.ok(Math.abs(n.totalIodineMassMg+n.outletIodineMassMg+n.stentGraftRemodeling.trappedIodineMassMg-125)<1e-7);
 }finally{f.dispose();}
});

test('aneurysm atlas: exposed cloth has contrast flow during trunk and bifurcation release',()=>{
 const f=fixture(),s=new HybridContrastSystem({centerlineSegments:f.segments,contactField:f.contactField}),n=s.flowNetwork;
 try {
  place(f.system,'right','body');f.system.deploy('right');const d=f.system.accesses.right.device;
  for(const fraction of [.45,.75]) {
   const extra=d.sheathTravel*fraction-(d.sheathWithdrawal??0);
   // Use small committed commands, as in the live controls.
   for(let i=0;i<30;i++)f.system.updateAccess('right',Math.max(0,extra)/12/30,null,{deviceId:d.id,release:'sheath'});
   assert.equal(d.phase,'deploying');
   const surface=f.system.getContrastSurface(.2);assert.ok(surface.parts.length);
   s.setStentGraftSurface(surface);assert.ok(n.stentGraftRemodeling.coveredEdges>0);
   assert.ok(n.stentGraftRemodeling.sac,'exposed release frontier maps to a finite-volume outlet');
   const before=n.totalIodineMassMg+n.outletIodineMassMg+n.stentGraftRemodeling.trappedIodineMassMg;
   const inlet=n.edges.find(e=>e.graftCovered);n.depositIodine(inlet.index,0,10);
   for(let i=0;i<30;i++)n.update(1/30);
   assert.ok(Math.abs(n.totalIodineMassMg+n.outletIodineMassMg+n.stentGraftRemodeling.trappedIodineMassMg-before-10)<1e-7);
  }
 }finally{f.dispose();}
});
