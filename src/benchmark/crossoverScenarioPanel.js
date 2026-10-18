import {createCrossoverScenarioRecording,CROSSOVER_STAGES,CROSSOVER_RECORDING_VERSION,validateCrossoverRecording} from './crossoverScenarioRecording.js';

function openStore() {
    return new Promise((resolve,reject)=>{
        const request=indexedDB.open('oet-crossover-scenarios',1);
        request.onupgradeneeded=()=>{request.result.createObjectStore('records');request.result.createObjectStore('chunks');};
        request.onsuccess=()=>resolve(request.result);request.onerror=()=>reject(request.error);
    });
}
const transactionDone=tx=>new Promise((resolve,reject)=>{tx.oncomplete=resolve;tx.onerror=()=>reject(tx.error);tx.onabort=()=>reject(tx.error??new Error('Local recording storage aborted'));});
const requestValue=request=>new Promise((resolve,reject)=>{request.onsuccess=()=>resolve(request.result);request.onerror=()=>reject(request.error);});

export async function createCrossoverScenarioPanel({configuration,reset,boundary,dispose}) {
    const panel=document.createElement('section');panel.id='crossoverScenarioRecorder';
    panel.style.cssText='position:fixed;top:74px;left:16px;z-index:10000;background:#10202ff5;color:#fff;padding:12px;border:1px solid #69c8c0;border-radius:8px;max-width:460px;font:14px system-ui';
    panel.innerHTML='<strong>Crossover · cały manewr</strong><p data-status>Gotowy do nagrania od pustych narzędzi.</p><p data-stage>Rozpocznij nagrywanie, następnie wykonaj manewr.</p><button data-record>Nagraj od początku</button> <button data-mark disabled>Zapisz etap 1/5</button> <button data-stop disabled>Przerwij</button><p><button data-replay disabled>Odtwórz całe nagranie</button> <button data-save disabled>Pobierz nagranie / raport</button></p><label>Wczytaj nagranie <input data-file type="file" accept=".json,.gz"></label><small data-storage style="display:block;margin-top:8px">Zapis lokalny co 120 kroków. Nie przeładowuj strony w trakcie manewru.</small>';
    document.body.append(panel);
    const element=name=>panel.querySelector(`[data-${name}]`),status=element('status'),stage=element('stage'),recordButton=element('record'),mark=element('mark'),stop=element('stop'),replay=element('replay'),save=element('save'),file=element('file'),storage=element('storage');
    let db=null,saveQueue=Promise.resolve(),saved=0,lastRecording=null,lastReplayable=false,lastCompleted=null,
        busy=true,disposed=false,downloadUrl=null,feedback=null;
    let recording;
    function update() {
        if(disposed)return;
        const r=recording.record,active=recording.active;
        recordButton.disabled=busy||active;mark.disabled=busy||recording.mode!=='recording';stop.disabled=busy||!active;
        const replayable=()=>lastReplayable&&lastRecording?.complete;
        replay.disabled=busy||active||!replayable();file.disabled=busy||active;save.disabled=busy||active||!r&&!lastRecording;
        replay.textContent=lastRecording?.fullManeuverRecorded?'Odtwórz cały manewr':'Odtwórz zapisany fragment';
        mark.textContent=`Zapisz etap ${Math.min(5,(r?.checkpoints.length??0)+1)}/5`;
        if(!r){if(feedback)status.textContent=feedback;return;}
        const hz=r.summary?.duringMotion.physicsHz??recording.progress.motionHz;
        status.textContent=`${recording.mode==='recording'?'Nagrywanie':recording.mode==='replaying'?'Odtwarzanie':r.complete?'Zakończono':'Przerwano'} · ${r.steps.length} kroków · podczas ruchu: ${hz===null?'—':hz.toFixed(1)} Hz`;
        if(r.error)status.textContent+=` · ${r.error}`;
        else if(recording.mode==='finished'&&r.replayOf)status.textContent+=` · różnica kształtu (informacyjnie): ${r.summary.maximumShapeDifferenceMm.toFixed(3)} mm`;
        if(r.replayOf&&r.validation?.mode==='functional')status.textContent+=' · bez limitu różnicy kształtu';
        stage.textContent=active?`${Math.min(5,recording.progress.stage+1)}. ${CROSSOVER_STAGES[recording.progress.stage]??'Odtwarzanie zapisanych komend'}`:
            r.replayOf&&r.complete?'Odtworzono komendy. Sprawdź przejście do przeciwległej biodrowej, zewnętrznej i udowej oraz stabilność manewru.':
                r.fullManeuverRecorded?'Zapisano wszystkie 5 potwierdzonych etapów.':'Zapis fragmentu — nie jest testem całego crossover.';
        if(!active&&r.complete&&!r.replayOf&&lastCompleted!==r){lastCompleted=lastRecording=r;lastReplayable=true;}
        replay.textContent=lastRecording?.fullManeuverRecorded?'Odtwórz cały manewr':'Odtwórz zapisany fragment';
        replay.disabled=busy||active||!replayable();
        if(!active&&feedback)status.textContent=feedback;
    }
    function persist(r,final) {
        if(r.replayOf||!db)return;
        const id=r.recordedAt,start=saved,steps=r.steps.slice(start),metadata={...r,steps:undefined,stepCount:r.steps.length};saved=r.steps.length;
        saveQueue=saveQueue.then(async()=>{
            const tx=db.transaction(['records','chunks'],'readwrite');
            if(steps.length)tx.objectStore('chunks').put(steps,[id,start]);
            tx.objectStore('records').put(metadata,id);tx.objectStore('records').put(id,'latest');
            await transactionDone(tx);if(!disposed)storage.textContent=`Zapisano lokalnie ${metadata.stepCount} kroków${final?' · nagranie zakończone':''}.`;
        }).catch(error=>{
            storage.textContent=`Błąd zapisu lokalnego: ${error.message}. Pobierz nagranie przed zamknięciem strony.`;
            recording.fail('Nie udało się zachować nagrania lokalnie.');
        });
    }
    recording=createCrossoverScenarioRecording({onChange:update,onChunk:persist});
    // The simulator receives the controller only after this async initializer
    // resolves. Never enable capture while IndexedDB is still being opened.
    update();
    async function start(source=null) {
        busy=true;feedback=null;update();
        try {
            if(source)validateCrossoverRecording(source);
            await reset(source?.configuration??null,()=>{
                if(source)recording.startReplay(configuration(),source);
                else{saved=0;recording.startRecording(configuration());}
            });
        }catch(error){feedback=error.message;}
        finally{busy=false;update();}
    }
    recordButton.onclick=()=>start();replay.onclick=()=>start(lastRecording);
    mark.onclick=()=>boundary(()=>{try{recording.checkpoint();}catch(error){status.textContent=error.message;}});
    stop.textContent='Zakończ zapis';
    stop.onclick=()=>boundary(()=>recording.mode==='recording'?recording.finishRecording():recording.stop());
    save.onclick=async()=>{
        const r=recording.record??lastRecording;if(!r)return;
        const json=new Blob([JSON.stringify(r)],{type:'application/json'});
        const zipped=await new Response(json.stream().pipeThrough(new CompressionStream('gzip'))).blob();
        if(downloadUrl)URL.revokeObjectURL(downloadUrl);downloadUrl=URL.createObjectURL(zipped);
        const link=document.createElement('a');link.href=downloadUrl;link.download=`crossover-${r.replayOf?'replay':'recording'}-${r.recordedAt.replaceAll(':','-')}.json.gz`;link.click();
    };
    file.onchange=async()=>{
        try {
            const selected=file.files[0];if(!selected)return;
            const text=selected.name.endsWith('.gz')?await new Response(selected.stream().pipeThrough(new DecompressionStream('gzip'))).text():await selected.text();
            const candidate=JSON.parse(text);
            validateCrossoverRecording(candidate);
            lastRecording=candidate;lastReplayable=true;feedback=`Wczytano ${candidate.steps.length} kroków.`;update();
        }catch(error){feedback=error.message;update();}
    };
    const visibility=()=>{if(document.visibilityState!=='visible'&&recording.active)recording.fail('Karta została ukryta; pomiar czasu nie byłby wiarygodny.');};
    document.addEventListener('visibilitychange',visibility);
    dispose(()=>{disposed=true;recording.stop();document.removeEventListener('visibilitychange',visibility);panel.remove();if(downloadUrl)URL.revokeObjectURL(downloadUrl);saveQueue.finally(()=>db?.close());});
    try {
        db=await openStore();if(disposed){db.close();return recording;}
        const latest=await requestValue(db.transaction('records').objectStore('records').get('latest'));
        if(latest) {
            const meta=await requestValue(db.transaction('records').objectStore('records').get(latest));
            const chunks=await requestValue(db.transaction('chunks').objectStore('chunks').getAll(IDBKeyRange.bound([latest,0],[latest,Infinity])));
            const steps=chunks.flat();if(steps.length!==meta.stepCount)throw new Error('Niepełne nagranie w pamięci lokalnej.');
            lastRecording={...meta,steps};
            if(meta.complete&&meta.schemaVersion===CROSSOVER_RECORDING_VERSION){validateCrossoverRecording(lastRecording);lastReplayable=true;}
            status.textContent=meta.schemaVersion!==CROSSOVER_RECORDING_VERSION?'Starszy zapis testowy bez pełnych metryk. Rozpocznij nowe nagranie.':
                `Przywrócono ${steps.length} kroków${meta.fullManeuverRecorded?' · cały manewr':meta.complete?' · zapis fragmentu':'; zapis przerwany — nie można go uznać za cały manewr'}.`;
            save.disabled=false;update();
        }
    }catch(error){storage.textContent=`Pamięć lokalna niedostępna: ${error.message}. Po nagraniu pobierz plik.`;}
    finally{busy=false;update();}
    return recording;
}
