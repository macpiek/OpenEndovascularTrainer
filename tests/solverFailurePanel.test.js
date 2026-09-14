import test from 'node:test';
import assert from 'node:assert/strict';
import {createSolverFailurePanel} from '../src/ui/solverFailurePanel.js';
const report={version:1,tools:[{id:'wire',insertion:675}],stepRequest:{tools:[{id:'wire',insertion:675.7,rotation:0}],rotations:{wire:0}},
    failure:{capturedAt:'2026-09-14T12:00:00.000Z',result:{status:'linear-solve',attempts:[{subdivisions:8,index:1,status:'linear-solve'}]}}};
function panel(storage) {
    let click,download;
    const button={addEventListener(event,fn){assert.equal(event,'click');click=fn;}},output={};
    const api=createSolverFailurePanel({button,output,storage,download:(json,name)=>{download={json,name};}});
    return {api,button,output,click:()=>click(),get download(){return download;}};
}
test('one rejection remains downloadable after reload, without rerunning physics',()=>{
    let stored=null,writes=0;const storage={getItem:()=>stored,setItem:(key,value)=>{stored=value;writes++;}};
    const p=panel(storage);assert.equal(p.button.disabled,true);assert.equal(writes,0);
    p.api.record(report);assert.match(p.output.value,/675 → 675.7 mm/);assert.match(p.output.value,/linear-solve/);
    assert.equal(writes,1);const next=panel(storage);assert.equal(next.button.disabled,false);
    next.click();assert.deepEqual(JSON.parse(next.download.json),report);assert.match(next.download.name,/\.json$/);
    assert.equal(writes,1);
});
test('storage quota and malformed saved data do not prevent an in-memory download',()=>{
    const p=panel({getItem:()=>'{bad',setItem(){throw new Error('quota');}});
    assert.equal(p.button.disabled,true);p.api.record(report);assert.match(p.output.value,/quota/);
    p.click();assert.deepEqual(JSON.parse(p.download.json),report);
});
