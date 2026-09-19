import test from 'node:test';
import assert from 'node:assert/strict';
import {createSolverFailurePanel} from '../src/ui/solverFailurePanel.js';
import {createSolverFailureArchive} from '../src/ui/solverFailureArchive.js';
const report=(id,recovered=false)=>({version:1,positions:[[1,2,3]],tools:[{id:'wire',insertion:675}],
    stepRequest:{tools:[{id:'wire',insertion:675.7,rotation:0}],rotations:{wire:0}},
    failure:{id,capturedAt:'2026-09-14T12:00:00.000Z',recovered,result:{status:recovered?'converged':'linear-solve',
        attempts:[{subdivisions:1,index:0,status:'linear-solve',converged:false},
            ...(recovered?[{subdivisions:2,index:0,status:'converged',converged:true}]:[])]}}});
function memoryArchive() {
    const records=new Map();let last=null;
    return {
        async load(){return {count:records.size,last:structuredClone(last)};},
        async append(r){if(!records.has(r.failure.id)){records.set(r.failure.id,structuredClone(r));last=structuredClone(r);}return this.load();},
        async readAll(){return structuredClone([...records.values()]);},
        async clear(){records.clear();last=null;}
    };
}
function panel(archive=memoryArchive(),storage) {
    const button=()=>({addEventListener(event,fn){assert.equal(event,'click');this.click=fn;}});
    const latest=button(),all=button(),clear=button(),output={},downloads=[];
    const api=createSolverFailurePanel({button:latest,allButton:all,clearButton:clear,output,storage,archive,
        download:(json,name)=>downloads.push({data:JSON.parse(json),name})});
    return {api,latest,all,clear,output,downloads};
}
test('all rejections survive reload with independent same-timestamp IDs, full replays and a batch summary',async()=>{
    const archive=memoryArchive(),p=panel(archive);await p.api.ready;
    assert.equal(p.all.disabled,true);
    await p.api.record(report('a'));await p.api.record(report('b',true));await p.api.record(report('b',true));
    assert.match(p.output.value,/675 → 675.7 mm/);assert.match(p.output.value,/odzyskano zbieżność/);
    const next=panel(archive);await next.api.ready;
    assert.equal(next.all.textContent,'Pobierz wszystkie (2)');next.latest.click();
    assert.deepEqual(next.downloads[0].data,report('b',true));
    await next.all.click();const bundle=next.downloads[1].data;
    assert.equal(bundle.format,'oet-rejected-steps');assert.equal(bundle.count,2);
    assert.deepEqual(bundle.reports,[report('a'),report('b',true)]);
    assert.deepEqual(bundle.summary,{recovered:1,terminal:1,byStatus:{'linear-solve':2}});
});
test('new events arriving while old history is loading retain their own immutable snapshot',async()=>{
    const archive=memoryArchive();await archive.append(report('old'));
    const load=archive.load.bind(archive);let release;
    archive.load=()=>new Promise(resolve=>{release=async()=>resolve(await load());});
    const p=panel(archive),value=report('new');p.api.record(value);value.positions[0][0]=999;
    archive.load=load;await release();await p.api.ready;await p.api.exportAll();
    assert.equal(p.downloads[0].data.count,2);
    assert.equal(p.downloads[0].data.reports.find(r=>r.failure.id==='new').positions[0][0],1);
    p.latest.click();assert.equal(p.downloads[1].data.failure.id,'new');
});
test('quota failures retain unsaved records and export merges them with saved history',async()=>{
    const archive=memoryArchive();await archive.append(report('saved'));
    archive.append=async()=>{throw new Error('quota');};const p=panel(archive);await p.api.ready;
    await p.api.record(report('unsaved'));assert.match(p.output.value,/quota/);
    assert.equal(p.all.textContent,'Pobierz wszystkie (2)');await p.api.exportAll();
    assert.equal(p.downloads[0].data.count,2);assert.match(p.output.value,/W pamięci karty: 1/);
});
test('denied IndexedDB and malformed legacy storage still allow an explicitly marked memory-only export',async()=>{
    const p=panel(createSolverFailureArchive({indexedDB:null}),{getItem:()=>'{bad'});
    await p.api.record(report('ram'));await p.api.exportAll();
    assert.deepEqual(p.downloads[0].data.reports,[report('ram')]);
    assert.match(p.downloads[0].data.archiveReadError,/IndexedDB/);
    assert.match(p.output.value,/Przed zamknięciem/);
});
test('failed archive reads never silently export an incomplete batch when persisted records are known',async()=>{
    const archive=memoryArchive();await archive.append(report('saved'));
    archive.readAll=async()=>{throw new Error('read failed');};
    const p=panel(archive);await p.api.ready;await p.api.exportAll();
    assert.equal(p.downloads.length,0);assert.match(p.output.value,/read failed/);
    assert.equal(p.all.disabled,false);
});
test('legacy single replay migrates once and explicit clear survives reload',async()=>{
    const legacy=report(undefined);let stored=JSON.stringify(legacy);
    const storage={getItem:()=>stored,removeItem:()=>{stored=null;}};
    const archive=memoryArchive(),p=panel(archive,storage);await p.api.ready;
    assert.equal(stored,null);assert.equal(p.all.textContent,'Pobierz wszystkie (1)');
    const next=panel(archive,storage);await next.api.ready;await next.api.clear();
    const empty=panel(archive,storage);await empty.api.ready;
    assert.equal(empty.all.disabled,true);assert.equal(empty.latest.disabled,true);
    assert.equal((await archive.readAll()).length,0);
});
test('new records queued during clear survive, while failed clears preserve history',async()=>{
    const archive=memoryArchive(),p=panel(archive);await p.api.record(report('before'));
    p.api.clear();await p.api.record(report('after'));await p.api.exportAll();
    assert.deepEqual(p.downloads[0].data.reports,[report('after')]);
    p.latest.click();assert.equal(p.downloads[1].data.failure.id,'after');
    archive.clear=async()=>{throw new Error('clear failed');};await p.api.clear();
    assert.match(p.output.value,/clear failed/);await p.api.exportAll();
    assert.equal(p.downloads[2].data.count,1);
});
