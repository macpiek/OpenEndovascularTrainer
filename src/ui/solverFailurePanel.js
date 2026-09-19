import {createSolverFailureArchive} from './solverFailureArchive.js';

const STORAGE_KEY = 'oet.shared-axis.last-failure.v1';

export function describeSolverFailure(report) {
    if (!report) return 'Brak zapisanego odrzucenia.';
    const {result={},capturedAt,captureError,recovered}=report.failure;
    const rejected=(result.attempts??[]).filter(a=>a.converged===false);
    const lines=[`Ostatnie odrzucenie: ${capturedAt}`,`Wynik kroku: ${recovered?'odzyskano zbieżność po odrzuceniu podkroku':'odrzucony'}`,`Przyczyna: ${result.error || rejected[0]?.error || rejected[0]?.status || result.status || 'nieznana'}`];
    for (const tool of report.stepRequest?.tools ?? []) {
        const before=report.tools?.find(t=>t.id===tool.id);
        lines.push(`${tool.id}: ${before?.insertion ?? '?'} → ${tool.insertion} mm; obrót ${report.stepRequest.rotations?.[tool.id] ?? '?'} → ${tool.rotation} rad`);
    }
    lines.push(`Iteracje: ${result.iterations ?? '—'}; faktoryzacje: ${result.factorizations ?? '—'}`);
    if (result.residual) lines.push(`Błędy ograniczeń: ${JSON.stringify(result.residual)}`);
    for (const attempt of result.attempts ?? []) lines.push(`Podział ${attempt.subdivisions}, część ${attempt.index}: ${attempt.status}${attempt.wallNormalFallback ? `; wcześniejsza próba: ${attempt.wallNormalFallback.liveFailure}` : ''}`);
    if (captureError) lines.push(`Niepełny zapis: ${captureError}`);
    return lines.join('\n');
}

/** Full replay files retain the existing v1 format; the batch is an envelope
 * of those files, so each reports[i] can still be fed to existing replay tools. */
export function solverFailureBundle(reports) {
    const byStatus=Object.create(null);let recovered=0;
    for(const report of reports) {
        if(report.failure.recovered)recovered++;
        const result=report.failure.result??{};
        const statuses=new Set((result.attempts??[]).filter(a=>a.converged===false).map(a=>a.error||a.status));
        if(!statuses.size)statuses.add(result.error||result.status||'unknown');
        for(const status of statuses)byStatus[status]=(byStatus[status]??0)+1;
    }
    return {format:'oet-rejected-steps',version:1,exportedAt:new Date().toISOString(),count:reports.length,
        summary:{recovered,terminal:reports.length-recovered,byStatus},reports};
}

/** No eviction: successfully persisted replays live in IndexedDB. Failed
 * writes remain in memory and are included in export, with a visible warning.
 * Serialization and persistence failures must never interrupt physics. */
export function createSolverFailurePanel({button,allButton,clearButton,output,storage,download,archive=createSolverFailureArchive()}) {
    let report=null,storageError='',summary={count:0,last:null},busy=false;
    const pending=new Map(),seen=new Set();
    function render() {
        const count=summary.count+pending.size;
        button.disabled=!report||busy;
        if(allButton){allButton.disabled=!count||busy;allButton.textContent=`Pobierz wszystkie (${count})`;}
        if(clearButton)clearButton.disabled=!count||busy;
        output.value=`Historia odrzuceń: ${count} kroków; trwale zapisane: ${summary.count}.`+
            (pending.size?` W pamięci karty: ${pending.size}.`:'')+'\n'+describeSolverFailure(report)+
            (storageError?`\nProblem zapisu/odczytu archiwum: ${storageError}. Przed zamknięciem karty pobierz historię.`:'');
    }
    let queue=archive.load().then(value=>{
        summary=value;if(!report)report=value.last;if(value.last)seen.add(value.last.failure.id);render();
    }).catch(error=>{storageError=error.message;render();});
    function record(value) {
        if(!value?.failure)return queue;
        const id=value.failure.id??`record-${(globalThis.crypto?.randomUUID?.()??`${Date.now()}-${Math.random().toString(36).slice(2)}`)}`;
        if(seen.has(id))return queue;
        seen.add(id);report=structuredClone(value);report.failure.id=id;
        const saved=report;pending.set(id,saved);render();
        queue=queue.then(async()=>{
            try{summary=await archive.append(saved);pending.delete(id);if(!pending.size)storageError='';}
            catch(error){storageError=error.message;}
            render();
        });
        return queue;
    }
    // Import the previously saved single replay once, without losing it or
    // reintroducing it after an explicit archive clear.
    try {
        const legacy=JSON.parse(storage?.getItem(STORAGE_KEY)??'null');
        if(legacy?.version===1&&legacy.failure){
            legacy.failure.id??=`legacy-${legacy.failure.capturedAt}`;
            record(legacy).then(()=>{if(!pending.has(legacy.failure.id))try{storage?.removeItem?.(STORAGE_KEY);}catch{/* Storage may be denied. */}});
        }
    }catch{/* Malformed legacy storage must not prevent startup. */}
    button.addEventListener('click',()=>{
        if(report)download(JSON.stringify(report),`oet-rejected-step-${report.failure.capturedAt.replace(/[^0-9TZ]/g,'-')}.json`);
    });
    async function exportAll() {
        busy=true;render();
        try {
            await queue;
            let stored,archiveReadError=null;
            try{stored=await archive.readAll();}
            catch(error){if(summary.count)throw error;stored=[];archiveReadError=storageError=error.message;}
            const all=new Map(stored.map(r=>[r.failure.id,r]));for(const [id,r] of pending)all.set(id,r);
            const bundle=solverFailureBundle([...all.values()].sort((a,b)=>a.failure.capturedAt.localeCompare(b.failure.capturedAt)||a.failure.id.localeCompare(b.failure.id)));
            if(archiveReadError)bundle.archiveReadError=archiveReadError;
            if(bundle.count)download(JSON.stringify(bundle),`oet-rejected-steps-${bundle.exportedAt.replace(/[^0-9TZ]/g,'-')}.json`);
        }catch(error){storageError=error.message;}
        finally{busy=false;render();}
    }
    function clear() {
        const clearing=new Set(pending.keys()),clearingSeen=new Set(seen);
        busy=true;render();
        queue=queue.then(async()=>{
            try{await archive.clear();for(const id of clearing)pending.delete(id);for(const id of clearingSeen)seen.delete(id);summary={count:0,last:null};report=[...pending.values()].at(-1)??null;storageError='';storage?.removeItem?.(STORAGE_KEY);}
            catch(error){storageError=error.message;}
            finally{busy=false;render();}
        });
        return queue;
    }
    allButton?.addEventListener('click',exportAll);clearButton?.addEventListener('click',clear);
    render();return {record,exportAll,clear,get ready(){return queue;}};
}
