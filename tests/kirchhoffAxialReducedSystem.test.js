import test from 'node:test';
import assert from 'node:assert/strict';
import {EndovascularPhysicsWorld} from '../src/physics/endovascularPhysicsWorld.js';
import {applyKirchhoffMaterialProfile} from '../src/physics/applyKirchhoffMaterialProfile.js';
import {assembleKirchhoffCoupledSystem,solveKirchhoffCoupledSystem} from '../src/physics/kirchhoffCoupledSystem.js';
import {assembleKirchhoffAxialReducedSystem} from '../src/physics/kirchhoffAxialReducedSystem.js';
import {solveKirchhoffAxialCoupledSystem} from '../src/physics/kirchhoffAxialCoupledSolver.js';
import {solveCoulombNewton} from '../src/physics/kirchhoffCoulombNewtonSolver.js';

for(const solverMode of ['row-major','general-band','section-band','section-all']) for(const differentPins of [false,true]) for(const shift of [0,.25]) for(const gap of [null,-.001,'friction']) test(`native local axial elimination retains material and contact response, format=${solverMode}, gap=${gap}, shift=${shift}, differentPins=${differentPins}`,()=>{
 const matrixFormat=solverMode==='row-major'?'row-major':'general-band';
 const world=new EndovascularPhysicsWorld();
 const bodies=[0,1].map(side=>{
  const b=world.createRod(`axial-${side}`,5,5,{mass:side?.07:.03});
  applyKirchhoffMaterialProfile(b,'berenstein');b.restRotation1.fill(0);b.restRotation2.fill(0);
  b.setPinned(side && differentPins ? 1 : 0,true);b.setProximalOrientationControl(b.orientationX[0],b.orientationY[0],b.orientationZ[0],b.orientationW[0],0,0);
  b.y[3]+=(side?-.001:.001);return b;
 });
 const c={innerBody:bodies[0],outerBody:bodies[1],startNode:0,outerStartNode:0,innerArcOffset:shift,
 kirchhoffContacts:gap===null?[]:[{_innerSegmentIndex:2,_outerSegmentIndex:2,innerWeights:[.5,.5],outerWeights:[.5,.5],normal:[0,1,0],gap:gap==='friction'?-.001:gap,_normalAlpha:0,manifoldContact:{normalLambda:0}}]};
 const options={includeSystem:true,includeAxialLayout:true,tolerance:1e-9};
 if(gap==='friction') {
  options.resolveNormalLoads=true;options.activeCondensation=true;options.simultaneousCoulomb=true;
  options.additionalRows=[0,2].map(axis=>({strain:-.0001,alpha:.01,lambda:0,
   gradients:[0,1].flatMap(side=>[2,3].map(node=>({side,dof:node*6+axis,value:side?.5:-.5})))}));
  options.groups=[{type:'coulomb-ellipse',rows:[0,1],mu:[.015,.006],normalLambda:0,
   normalContact:c.kirchhoffContacts[0].manifoldContact}];
 }
 const source=assembleKirchhoffCoupledSystem(c,1/120,{...options,jacobianOnly:matrixFormat==='general-band'});
 if(matrixFormat==='general-band') {
  assert.equal(source.matrix,null);
  assert.equal(c._coupledSystemAssembly.matrix,undefined,'no dual Gram allocation');
  assert.equal(c._bundleRuntime,undefined,'no duplicate coordinate transform');
  assert.throws(()=>solveKirchhoffCoupledSystem(c,1/120,{jacobianOnly:true}),/Gram/);
 }
 const reduced=assembleKirchhoffAxialReducedSystem(source,{matrixFormat,sectionSpan:solverMode.startsWith('section-')?4:undefined,sectionScope:solverMode==='section-all'?'all':'catheter'});
 const full=solveKirchhoffCoupledSystem(c,1/120,options);
 assert.ok(full.diagnostics.converged,JSON.stringify(full.diagnostics));
 assert.throws(()=>assembleKirchhoffAxialReducedSystem(full.system),/unsolved native assembly/);
 assert.ok(reduced.diagnostics.eliminatedOffsetDofs>0);
 assert.equal(reduced.diagnostics.eliminatedSolver,'diagonal','hard adaptation leaves offsets locally independent');
 assert.equal(reduced.diagnostics.eliminatedFactorEntries,0);
 assert.ok(reduced.diagnostics.explicitRows>0,'zero-compliance material equations remain explicit');
 const solved=solveCoulombNewton(reduced.matrix,reduced.rhs,reduced.lower,reduced.upper,reduced.count,reduced.count,reduced.groups,
 {matrixFormat,tolerance:1e-9,localSections:reduced.localSections});
 assert.ok(solved.diagnostics.converged,JSON.stringify(solved.diagnostics));
 if(solverMode.startsWith('section-')) {
  assert.ok(solved.diagnostics.sectionSolves>0,JSON.stringify(solved.diagnostics));
  assert.ok(solved.diagnostics.localRowCount>0,JSON.stringify(solved.diagnostics));
 }
 const recovered=reduced.recover(solved.increment);
 const nativeResidual=full.system.rhs.slice(),native=full.system;
 for(let i=0;i<native.count;i++) for(let j=0;j<native.count;j++) {
  const offset=Math.abs(i-j);
  if(offset<native.band) nativeResidual[i]-=native.matrix[Math.max(i,j)*native.band+offset]*recovered.increment[j];
 }
 nativeResidual.forEach((v,i)=>assert.ok(Math.abs(v-recovered.residual[i])<1e-8,`original certificate row ${i}`));
 assert.ok(recovered.originalKkt.maximumResidual<1e-8,JSON.stringify(recovered.originalKkt));
 assert.ok(recovered.maximumMobilityError<1e-8,`${recovered.maximumMobilityError}`);
 for(const [side,reference] of [[0,full.inner.correction],[1,full.outer.correction]])
  reference.forEach((v,i)=>assert.ok(Math.abs(v-recovered.correction[side][i])<1e-7,`${side}/${i}: ${v} vs ${recovered.correction[side][i]}`));
 for(let row=0;row<full.system.count;row++) {
  const source=full.system.rows[full.system.order[row]];
  const expected=source.kind==='material'?[full.inner,full.outer][source.side].lambda[source.local]:
   source.kind==='normal'?full.contactIncrement[source.local]:full.additionalIncrement[source.local];
  assert.ok(Math.abs(recovered.increment[row]-expected)<1e-7*Math.max(1,Math.abs(expected)),`force row ${row}`);
 }
 if(solverMode.startsWith('section-')) {
  const poses=bodies.map(body=>[body.x.slice(),body.y.slice(),body.z.slice()]);
  const integrated=solveKirchhoffAxialCoupledSystem(c,1/120,{...options,sectionSpan:4,sectionScope:solverMode==='section-all'?'all':'catheter'});
  assert.ok(integrated.diagnostics.converged,JSON.stringify(integrated.diagnostics));
  assert.equal(integrated.diagnostics.axialReduction,true);
  assert.ok(Math.abs(integrated.scale-full.scale)<1e-8);
  for(const key of ['inner','outer'])for(const field of ['correction','lambda'])
   full[key][field].forEach((v,i)=>assert.ok(Math.abs(v-integrated[key][field][i])<1e-7*Math.max(1,Math.abs(v)),`${key}/${field}/${i}`));
  for(const key of ['contactIncrement','additionalIncrement'])full[key].forEach((v,i)=>
   assert.ok(Math.abs(v-integrated[key][i])<1e-7*Math.max(1,Math.abs(v)),`${key}/${i}`));
  bodies.forEach((body,i)=>assert.deepEqual([body.x,body.y,body.z],poses[i],'direction solve does not apply positions'));
 }
 bodies[0].y[2]+=.002;
 solveKirchhoffCoupledSystem(c,1/120,options);
 const retained=reduced.recover(solved.increment);
 assert.deepEqual(retained.increment,recovered.increment,'owned recovery survives the next native assembly');
 assert.deepEqual(retained.residual,recovered.residual);
 for(let side=0;side<2;side++) assert.deepEqual(retained.correction[side],recovered.correction[side]);
});
