import { mkdirSync, writeFileSync } from 'node:fs';
import { createSharedAxisNative, feedSharedAxisNative, rotateSharedAxisNative, relaxSharedAxisNative,
    captureSharedAxisNative, restoreSharedAxisNative } from '../../src/physics/kirchhoffSharedAxisNative.js';

const output = process.argv[2] ?? 'reports/shared-axis-native-2026-09-12/profile.json';
const wire = {id:'wire',insertion:309,shaftStiffness:39,tipStiffness:30.7};
const catheter = {id:'catheter',insertion:100,shaftStiffness:58.1,tipStiffness:87};
const runs=[];
for(let repeat=0;repeat<4;repeat++)for(const ids of [['wire'],['catheter'],['wire','catheter']]) {
    let s=createSharedAxisNative({tools:ids.map(id=>({... (id==='wire'?wire:catheter)}))});
    const record=(phase,result)=>{
        runs.push({repeat,tools:ids.join('+'),phase,nodes:s.positions.length,...result});
        if(!result.converged)throw new Error(JSON.stringify(runs.at(-1)));
    };
    record('initial',relaxSharedAxisNative(s));
    for(let step=1;step<=12;step++) {
        const start=performance.now();
        const candidate=feedSharedAxisNative(s,Object.fromEntries(ids.map(id=>[id,(id==='wire'?309:100)+step*.25])));
        const transferMs=performance.now()-start;
        s=candidate;record('feed', {...relaxSharedAxisNative(s),transferMs});
    }
    for(let step=1;step<=4;step++){
        const snapshot=captureSharedAxisNative(s);
        rotateSharedAxisNative(s,ids.at(-1),.005);
        const result=relaxSharedAxisNative(s);
        if(!result.converged)restoreSharedAxisNative(s,snapshot);
        record('rotate',result);
    }
}
const statistics=values=>({mean:values.reduce((a,b)=>a+b,0)/values.length,max:Math.max(...values)});
const summaries=[];
for(const tools of [...new Set(runs.map(r=>r.tools))])for(const phase of ['initial','feed','rotate']){
    const rows=runs.filter(r=>r.repeat>0&&r.tools===tools&&r.phase===phase);
    summaries.push({tools,phase,samples:rows.length,ms:statistics(rows.map(r=>r.ms)),
        transferMs:statistics(rows.map(r=>r.transferMs??0)),assemblyMs:statistics(rows.map(r=>r.timings.assemblyMs)),
        linearMs:statistics(rows.map(r=>r.timings.linearMs)),iterations:statistics(rows.map(r=>r.iterations)),
        dofs:statistics(rows.map(r=>r.dofs)),matrixEntries:statistics(rows.map(r=>r.matrixEntries)),
        maximumConstraint:Math.max(...rows.map(r=>r.residual.length))});
}
mkdirSync(output.slice(0,output.lastIndexOf('/')),{recursive:true});
writeFileSync(output,JSON.stringify({scope:'Quasi-static native-law shared-axis laboratory, not application FPS or anatomy replay',warmupRepeats:1,summaries,runs},null,2)+'\n');
console.log(JSON.stringify(summaries,null,2));
