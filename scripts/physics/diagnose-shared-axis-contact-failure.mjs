import { readFileSync, writeFileSync } from 'node:fs';
import { loadCoupledRuntimeAnatomy } from '../../tests/helpers/coupledRuntimeFixture.js';
import { restoreSharedAxisReplay } from '../../tests/helpers/sharedAxisReplay.js';
import { auditSharedAxisDirection } from '../../tests/helpers/sharedAxisContactAudit.js';
import { relaxSharedAxisWithContacts } from '../../src/physics/kirchhoffSharedAxisVesselWitnesses.js';

const fixture=name=>JSON.parse(readFileSync(new URL(`../../tests/fixtures/shared-axis/anatomy-wire-145.75-${name}.json`,import.meta.url)));
const anatomy=await loadCoupledRuntimeAnatomy();
try {
    const state=restoreSharedAxisReplay(fixture('incoming'),anatomy.field);
    let lastDirection,lastAcceptedTrial;
    const result=relaxSharedAxisWithContacts(state,{observeTrial:event=>{
        if(event.kind==='direction'&&event.direction.converged)lastDirection={
            old:event.base.rows.map(r=>r.multiplier),
            target:event.base.rows.map((r,i)=>r.multiplier+event.direction.multiplierIncrement[i])};
        if(event.kind==='trial'&&event.accept)lastAcceptedTrial={iteration:event.iteration,method:event.method,scale:event.scale,
            rows:event.candidate.rows.flatMap((r,i)=>r.witness?.t===2/3&&r.edge===32?[{
                id:r.id,old:lastDirection.old[i],target:lastDirection.target[i],accepted:r.multiplier,gap:r.gap}]:[])};
    }});
    const terminal=restoreSharedAxisReplay(fixture('terminal'),anatomy.field);
    const liveGeometry=['newton','gauss-newton'].map(mode=>auditSharedAxisDirection(terminal,mode));
    // Disable new-face discovery only: the retained finite surfaces, radii,
    // sheath, material law, loads and all existing reactions stay identical.
    terminal.wallSamples[1]=()=>({gap:1,jacobian:[0,0,0,0,0,0]});
    const frozenGeometry=['newton','gauss-newton'].map(mode=>auditSharedAxisDirection(terminal,mode));
    const report={result,lastAcceptedTrial,liveGeometry,frozenGeometry};
    const output=process.argv[2]??'reports/shared-axis-failure-2026-09-13/diagnosis.json';
    writeFileSync(output,JSON.stringify(report,null,2)+'\n');
    console.log(JSON.stringify(report,null,2));
} finally { anatomy.dispose(); }
