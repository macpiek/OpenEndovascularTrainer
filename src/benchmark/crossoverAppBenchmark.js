// Developer-only continuation of the saved manual crossover. Measurement is
// driven by the application's committed clock; no private physics loop runs.
export function crossoverContinuationCommands(step,dt,tools,current) {
    const offset=step<30?0:step<50?-(step-29)*.5:step<70?-(69-step)*.5:0;
    const command=(id,rateIn,rateOut)=>{
        const delta=tools.find(t=>t.id===id).insertion+offset-current[id];
        return Math.max(-1,Math.min(1,delta/((delta>=0?rateIn:rateOut)*dt)));
    };
    return {guidewireAdvance:command('wire',44,44),catheterAdvance:command('catheter',52,32),
        guidewireRotation:0,catheterRotation:0,catheterType:'pigtail'};
}

export function crossoverPhysicalGate(dt,result,{forceTolerance=1e-6}={}) {
    const quality=result?.quality,bodies=quality?.bodies;
    const bounded=(value,limit,inclusive=true)=>Number.isFinite(value)&&value>=0&&(inclusive?value<=limit:value<limit);
    return Number.isFinite(forceTolerance)&&forceTolerance>0&&forceTolerance<=1e-4&&
        dt===1/60&&result?.converged===true&&quality?.finite===true&&
        bounded(result.certificateBound,forceTolerance)&&bounded(result.residual?.length,1e-5)&&
        bounded(quality.maxPenetration,1e-5)&&Array.isArray(bodies)&&bodies.length>0&&
        bodies.every(body=>bounded(body.maxLengthError,1e-5,false));
}

export async function createCrossoverAppBenchmark({mode,ready,restore,read,dispose}) {
    const panel=document.createElement('section');panel.id='crossoverAppBenchmark';
    panel.style.cssText='position:fixed;top:72px;left:16px;z-index:10000;background:#10202fee;color:#fff;padding:12px;border:1px solid #69c8c0;border-radius:8px;max-width:560px;font:14px system-ui';
    panel.innerHTML='<strong>Crossover · fragment końcowy z RTG</strong><p data-status>Ładowanie zapisanego stanu…</p><button data-run disabled>Odtwórz 600 kroków</button> <button data-stop disabled>Zatrzymaj pomiar</button> <a data-download hidden download="crossover-full-app.json">Zapisz raport</a>';
    document.body.append(panel);
    const status=panel.querySelector('[data-status]'),run=panel.querySelector('[data-run]'),stop=panel.querySelector('[data-stop]'),download=panel.querySelector('[data-download]');
    let active=false,report=null,started=0,cpuMs=0,publicationMs=0,url=null,disposed=false;
    const update=()=>{
        if(!report)return;
        const wall=(performance.now()-started)/1000,n=report.steps.length;
        status.textContent=`${active?'Pomiar':report.passed?'PASS':'STOP/FAIL'} · ${mode==='wide'?'rzadszy trzon':'pełna siatka'} · ${n}/600 kroków · fizyka ${(n/Math.max(.001,wall)).toFixed(1)} Hz · obraz ${(report.frames/Math.max(.001,wall)).toFixed(1)} FPS`;
    };
    const finish=error=>{
        if(!active)return;
        active=false;report.wallSeconds=(performance.now()-started)/1000;report.simulationSeconds=report.steps.length/60;
        report.acceptedPhysicsHz=report.steps.length/report.wallSeconds;report.renderFps=report.frames/report.wallSeconds;
        report.error=error??null;report.passed=!error&&report.steps.length===600;
        report.browserBudgetMet=report.passed&&report.acceptedPhysicsHz>=59.5;report.goalAchieved=false;
        report.end=read();update();run.disabled=false;stop.disabled=true;
        if(url)URL.revokeObjectURL(url);url=URL.createObjectURL(new Blob([JSON.stringify(report,null,2)],{type:'application/json'}));
        download.href=url;download.hidden=false;
    };
    dispose(()=>{disposed=true;active=false;panel.remove();if(url)URL.revokeObjectURL(url);});
    const response=await fetch('/reports/performance-scenarios/001-stage-05-replay.json.gz');
    if(!response.ok)throw new Error(`Crossover fixture: ${response.status}`);
    const fixture=await (response.headers.get('Content-Encoding')==='gzip'?response.json():new Response(response.body.pipeThrough(new DecompressionStream('gzip'))).json());
    if(disposed)return null;
    status.textContent='Tylko końcowa pozycja i ruch ±10 mm · nie jest to test całego crossover · krok 1/60 s';run.disabled=false;
    run.onclick=async()=>{
        if(!ready()){status.textContent='Poczekaj na anatomię; włącz tryb Fluoro i prawy dostęp.';return;}
        run.disabled=true;download.hidden=true;
        try {
            await restore(fixture);
            report={recordedAt:new Date().toISOString(),mode,dt:fixture.stepRequest.dt,frames:0,steps:[],frameCosts:[],initial:read(),
                limitations:['Continuation from the saved final crossover, not the complete manual maneuver.','Full application renderer, publication and shared clock. Shape difference is validated by the separate paired model benchmark.']};
            started=performance.now();cpuMs=publicationMs=0;active=true;stop.disabled=false;update();
        }catch(error){status.textContent=`FAIL: ${error.message}`;run.disabled=false;}
    };
    stop.onclick=()=>finish('Stopped by user');
    return {
        commands(dt,current){return active?crossoverContinuationCommands(report.steps.length,dt,fixture.stepRequest.tools,current):null;},
        recordSlice(ms,result){if(!active)return;cpuMs+=ms;if(result.error||result.terminal)finish(result.error?.message??result.status);},
        recordCommit(dt,diagnostics,publishMs){
            if(!active)return;
            const r=diagnostics.last,q=r?.quality;
            if(!crossoverPhysicalGate(dt,r)){
                finish('Committed step failed the crossover physical gate');return;
            }
            publicationMs+=publishMs;
            report.steps.push({step:report.steps.length,wallMs:performance.now()-started,cpuMs,publicationMs,
                providerMs:r.cpuMs,workerWallMs:r.workerWallMs,workerDispatchDelayMs:r.workerDispatchDelayMs,
                workerResponseMs:r.workerResponseMs,workerPublicationDelayMs:r.workerPublicationDelayMs,
                replayCaptureMs:r.replayCaptureMs??0,replayCaptureTimes:r.replayCaptureTimes,nodes:diagnostics.mesh.nodes,iterations:r.iterations,factorizations:r.factorizations,
                certificateBound:r.certificateBound,maxPenetrationMm:q.maxPenetration,
                maxLengthError:Math.max(...q.bodies.map(b=>b.maxLengthError))});
            cpuMs=publicationMs=0;if(report.steps.length===600)finish();
        },
        recordFrame(){if(active){report.frames++;if(report.frames%15===0)update();}},
        recordFrameCosts(costs){if(active)report.frameCosts.push(costs);}
    };
}
