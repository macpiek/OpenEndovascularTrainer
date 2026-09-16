import {readFileSync} from 'node:fs';
import {loadCoupledRuntimeAnatomy} from '../../tests/helpers/coupledRuntimeFixture.js';
import {restoreSharedAxisReplay} from '../../tests/helpers/sharedAxisReplay.js';
import {sharedAxisOuterMaterialAt} from '../../src/physics/kirchhoffSharedAxisNative.js';
import {assembleSharedAxisConstraintRows,createSharedAxisConstraintRowPool} from '../../src/physics/kirchhoffSharedAxisConstraintRows.js';
const anatomy=await loadCoupledRuntimeAnatomy();
try {
 const fixture=JSON.parse(readFileSync(new URL('../../tests/fixtures/shared-axis/anatomy-berenstein-feed-138.67-live-cycle.json',import.meta.url)));
 for(const tangent of [false,true])for(const changePose of [false,true]) {
  const samples=[[],[]];
  for(let repeat=0;repeat<8;repeat++)for(const mode of repeat%2?[1,0]:[0,1]) {
   const s=restoreSharedAxisReplay(fixture,anatomy.field),pool=createSharedAxisConstraintRowPool(),banks=[pool.acquire()];
   banks.push(pool.acquire([banks[0].rows]));banks.push(pool.acquire(banks.map(b=>b.rows)));
   const start=performance.now();
   for(let i=0;i<500;i++) {
    if(changePose)s.geometryKey=Symbol();
    s.chain.gradient.fill(0);
    assembleSharedAxisConstraintRows(s,{withTangent:tangent,retainWallHessians:true,reuseConstraintWork:true,outerMaterialAt:sharedAxisOuterMaterialAt,storage:mode?banks[i%3]:null});
   }
   if(repeat>1)samples[mode].push((performance.now()-start)/500);
  }
  console.log({tangent,changePose,reference:samples[0].reduce((a,b)=>a+b)/samples[0].length,buffer:samples[1].reduce((a,b)=>a+b)/samples[1].length});
 }
}finally{anatomy.dispose();}
