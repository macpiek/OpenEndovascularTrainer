import assert from 'node:assert/strict';
import test from 'node:test';
import { lineSearchLevel, createLineSearchStats, recordLineSearchTrial } from '../src/physics/kirchhoffLineSearch.js';

test('adaptive ordering retains every reference candidate and the same smallest correction',()=>{
 for(let start=0;start<8;start++) {
  const order=Array.from({length:8},(_,i)=>lineSearchLevel(i,start));
  assert.equal(order[0],start);
  assert.deepEqual([...order].sort((a,b)=>a-b),[0,1,2,3,4,5,6,7]);
  // A changed contact may admit only a formerly skipped large correction.
  for(let onlyAccepted=0;onlyAccepted<8;onlyAccepted++)
   assert.equal(order.find(level=>level===onlyAccepted),onlyAccepted);
 }
});
test('rejection telemetry separates accepted scale, dominant residual and changed worst witness',()=>{
 const stats=createLineSearchStats();
 const base={kind:'wall',node:1,wallBranchId:2};
 recordLineSearchTrial(stats,2,false,{meritTerms:{length:.2,boundary:4}},base,{...base,node:2});
 recordLineSearchTrial(stats,3,true,{meritTerms:{length:.2,boundary:.5}},base,base);
 assert.equal(stats.rejected[2],1);assert.equal(stats.accepted[3],1);
 assert.deepEqual(stats.rejectionTerms,{boundary:1});assert.equal(stats.boundaryWorstChanged,1);
});
