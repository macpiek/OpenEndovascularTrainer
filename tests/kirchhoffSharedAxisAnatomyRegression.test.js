import test, { after } from 'node:test';
import assert from 'node:assert/strict';
import { readFileSync } from 'node:fs';
import { loadCoupledRuntimeAnatomy } from './helpers/coupledRuntimeFixture.js';
import { captureSharedAxisReplay, restoreSharedAxisReplay } from './helpers/sharedAxisReplay.js';
import { auditSharedAxisDirection } from './helpers/sharedAxisContactAudit.js';
import { captureSharedAxisNative, assembleSharedAxisNative } from '../src/physics/kirchhoffSharedAxisNative.js';
import { relaxSharedAxisWithContacts, createSharedAxisVesselDiscovery } from '../src/physics/kirchhoffSharedAxisVesselWitnesses.js';

const fixture=name=>JSON.parse(readFileSync(new URL(`./fixtures/shared-axis/anatomy-wire-145.75-${name}.json`,import.meta.url)));
const anatomy=await loadCoupledRuntimeAnatomy();
after(()=>anatomy.dispose());

test('145.75 mm anatomy checkpoint restores positions, frames, reactions and witness topology exactly',()=>{
    for(const name of ['incoming','terminal']) {
        const saved=fixture(name),s=restoreSharedAxisReplay(saved,anatomy.field);
        assert.deepEqual(captureSharedAxisReplay(s,saved.sheath),saved);
        assert.equal(s.materials[0].spec.insertion,145.75);
        assert.equal(s.materials[1].spec.insertion,0);
    }
});

test('frozen terminal contacts expose an inconsistent equality subset independently of new-face discovery',()=>{
    const saved=fixture('terminal'),s=restoreSharedAxisReplay(saved,anatomy.field);
    // This historical rank certificate used one winner per capsule. Preserve
    // that discovery policy only for its frozen numerical evidence.
    const {start,end}=saved.sheath,length=Math.hypot(end.x-start.x,end.y-start.y,end.z-start.z);
    s.wallSamples[1]=createSharedAxisVesselDiscovery(anatomy.field,length,{allSamples:false});
    const live=['newton','gauss-newton'].map(mode=>auditSharedAxisDirection(s,mode));
    s.wallSamples[1]=()=>({gap:1,jacobian:[0,0,0,0,0,0]});
    const frozen=['newton','gauss-newton'].map(mode=>auditSharedAxisDirection(s,mode));
    assert.deepEqual(frozen,live);
    assert.equal(frozen.find(a=>a.mode==='gauss-newton').converged,true,'The repaired working basis must admit a global direction');
    for(const audit of frozen) {
        assert.equal(audit.activeEquations,50);assert.equal(audit.jacobianRank,48);assert.equal(audit.augmentedRank,49);
        const group=audit.dependentGroups.find(g=>g.ids.some(id=>id.endsWith('/116001')));
        assert.ok(group);assert.ok(group.gaps.every(g=>g>0));
        assert.ok(group.normalResidual.every(v=>Math.abs(v)<1e-12));
        assert.ok(Math.abs(group.gapResidual)>1e-4);
    }
});

test('an interrupted anatomy solve still rolls back its pose, frames and reactions',()=>{
    const s=restoreSharedAxisReplay(fixture('incoming'),anatomy.field),before=captureSharedAxisNative(s);
    const result=relaxSharedAxisWithContacts(s,{maxIterations:0});
    assert.equal(result.converged,false);assert.equal(result.status,'iteration-limit');
    const after=captureSharedAxisNative(s);
    assert.deepEqual(after.positions,before.positions);assert.deepEqual(after.frames,before.frames);
    assert.deepEqual(after.multipliers.slice(0,before.multipliers.length),before.multipliers);
    assert.ok(after.multipliers.slice(before.multipliers.length).every(v=>v===0));
});

test('wire feed from 145.50 to 145.75 mm must converge with unchanged physical tolerances',()=>{
    const s=restoreSharedAxisReplay(fixture('incoming'),anatomy.field);
    const result=relaxSharedAxisWithContacts(s);
    assert.equal(result.converged,true,JSON.stringify(result));
});


test('all-sample discovery supplements the historical checkpoint with previously unretained nearby sites',()=>{
    const s=restoreSharedAxisReplay(fixture('terminal'),anatomy.field),oldIds=new Set(s.definitions.map(r=>r.id));
    assert.throws(()=>assembleSharedAxisNative(s),/shared-axis-wall-discovery/);
    assert.ok(s.pendingVesselRows.size>0);
    for(const row of s.pendingVesselRows.values())assert.equal(oldIds.has(row.id),false);
});
