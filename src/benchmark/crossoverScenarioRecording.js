import {crossoverPhysicalGate} from './crossoverAppBenchmark.js';

export const CROSSOVER_STAGES=[
    'Pigtail nad rozwidleniem, prowadnik cofnięty, ogonek skierowany w drugą stronę',
    'Pigtail oparty o rozwidlenie, końcówka w przeciwległej biodrówce',
    'Prowadnik przez pigtaila w przeciwległej biodrówce',
    'Po korektach: prowadnik w biodrowej zewnętrznej i udowej',
    'Oba narzędzia wsunięte na prawie maksymalną długość'
];
const keys=['advance','rotation','catheterAdvance','catheterRotation'];
export const CROSSOVER_RECORDING_VERSION=2;
const validShape=s=>s&&s.coordinates?.length>=2&&s.positions?.length===s.coordinates.length*3&&
    [...s.coordinates,...s.positions].every(Number.isFinite)&&s.coordinates.every((x,i)=>!i||x>s.coordinates[i-1]);
const validTools=t=>t&&['wire','catheter','wireRotation','catheterRotation'].every(key=>Number.isFinite(t[key]))&&t.wire>=0&&t.catheter>=0;
const validQuality=(quality,dt,forceTolerance=1e-6)=>Array.isArray(quality)&&quality.length===2&&quality.every(q=>crossoverPhysicalGate(dt,q,{forceTolerance}));
const captureQuality=d=>({converged:d.last.converged,certificateBound:d.last.certificateBound,residual:{length:d.last.residual.length},
    quality:{finite:d.last.quality.finite,maxPenetration:d.last.quality.maxPenetration,bodies:d.last.quality.bodies.map(b=>({maxLengthError:b.maxLengthError}))}});
const captureWork=d=>({mesh:{...d.mesh},...Object.fromEntries([
    'iterations','factorizations','backtracks','geometryRestarts','frictionIterations','substepAttempts','subdivisions',
    'predictorFallbacks','coupledFrictionFallbacks','wallNormalFallbacks','replayCaptureMs',
    'workerWallMs','workerDispatchDelayMs','workerResponseMs','workerPublicationDelayMs'
].map(key=>[key,d.last?.[key]??0])),timings:{...d.last?.timings},replayCaptureTimes:{...d.last?.replayCaptureTimes}});
const moving=inputs=>Object.values(inputs).some(input=>keys.some(key=>input[key]!==0)||
    input.automatic.guidewireAdvance!==0||input.automatic.catheterAdvance!==0);

export function validateCrossoverInputs(inputs) {
    for(const side of ['right','left']) {
        const input=inputs?.[side];
        if(!input||keys.some(key=>!Number.isFinite(input[key])||Math.abs(input[key])>1)||
            !input.automatic||['guidewireAdvance','catheterAdvance'].some(key=>!Number.isFinite(input.automatic[key])||Math.abs(input.automatic[key])>1)||
            input.wireType!=='glidewire'||input.catheterType!=='pigtail'||input.catheterTool!=='pigtail')
            throw new Error('Nagranie crossover wymaga pigtaila, Glidewire i prawidłowych komend obu dostępów.');
    }
    return inputs;
}

// Both curves are piecewise linear in the same insertion coordinate. The
// maximum distance is attained at a vertex of their combined partition.
export function crossoverShapeDifference(a,b) {
    if(!validShape(a)||!validShape(b)||Math.abs(a.coordinates[0]-b.coordinates[0])>1e-8||Math.abs(a.coordinates.at(-1)-b.coordinates.at(-1))>1e-8)
        throw new Error('Niezgodna dziedzina porównania kształtu narzędzi.');
    const point=(s,x,index)=>{
        while(index.i<s.coordinates.length-2&&s.coordinates[index.i+1]<x)index.i++;
        const i=index.i,t=(x-s.coordinates[i])/(s.coordinates[i+1]-s.coordinates[i]);
        return [0,1,2].map(k=>s.positions[i*3+k]+t*(s.positions[(i+1)*3+k]-s.positions[i*3+k]));
    };
    const ai={i:0},bi={i:0};let max=0;
    for(const x of [...new Set([...a.coordinates,...b.coordinates])].sort((x,y)=>x-y)) {
        const p=point(a,x,ai),q=point(b,x,bi);max=Math.max(max,Math.hypot(...p.map((v,k)=>v-q[k])));
    }
    return max;
}

/** Validate before resetting any live tools. Checkpoints describe operator
 * confirmations; they cannot manufacture missing inputs or stability proof. */
export function validateCrossoverRecording(record,{maxSteps=36000,forceCertificateLimit=1e-6}={}) {
    if(record?.schemaVersion!==CROSSOVER_RECORDING_VERSION||record.kind!=='crossover-input-sequence'||record.dt!==1/60||
        record.complete!==true||!Array.isArray(record.steps)||!record.steps.length||record.steps.length>maxSteps||
        !record.configuration||!Array.isArray(record.checkpoints)||record.checkpoints.length>CROSSOVER_STAGES.length)
        throw new Error('Nagranie jest niekompletne lub ma nieobsługiwany format; wymagany zapis v2 z kontrolą obu solverów.');
    let previous=0;
    for(const [stage,checkpoint] of record.checkpoints.entries()) {
        if(checkpoint.stage!==stage||!Number.isInteger(checkpoint.afterStep)||checkpoint.afterStep<=previous||checkpoint.afterStep>record.steps.length)
            throw new Error('Nieprawidłowa kolejność punktów kontrolnych.');
        previous=checkpoint.afterStep;
    }
    if(record.checkpoints.length===CROSSOVER_STAGES.length&&previous!==record.steps.length)throw new Error('Brak końcowego punktu kontrolnego.');
    let stage=0;
    for(const [index,step] of record.steps.entries()) {
        while(record.checkpoints[stage]?.afterStep<=index)stage++;
        validateCrossoverInputs(step.inputs);
        if(!validShape(step.shape)||!validTools(step.tools)||!validQuality(step.quality,record.dt,forceCertificateLimit)||step.stage!==stage||step.moving!==moving(step.inputs)||
            !Number.isFinite(step.intervalMs)||step.intervalMs<0||!Number.isFinite(step.cpuMs)||step.cpuMs<0)
            throw new Error(`Niekompletny lub nieprawidłowy krok ${index+1}: kształt, sprzęt, stabilność albo czas.`);
    }
    return record;
}

export function summarizeCrossoverRecording(steps,dt=1/60) {
    const group=selected=>{
        const wallMs=selected.reduce((n,s)=>n+s.intervalMs,0),costs=selected.map(s=>s.cpuMs).sort((a,b)=>a-b);
        const q=p=>costs.length?costs[Math.floor((costs.length-1)*p)]:null;
        return {steps:selected.length,wallMs,physicsHz:wallMs>0?selected.length*1000/wallMs:null,
            p50CpuMs:q(.5),p95CpuMs:q(.95),p99CpuMs:q(.99),maxCpuMs:q(1),
            intervalsOverBudget:selected.filter(s=>s.intervalMs>dt*1000+1).length};
    };
    const windowSteps=Math.round(1/dt);let movingRun=[],minimumMovingWindowHz=null;
    for(const step of steps) {
        if(!step.moving){movingRun=[];continue;}
        movingRun.push(step.intervalMs);if(movingRun.length>windowSteps)movingRun.shift();
        if(movingRun.length===windowSteps){const hz=1000*windowSteps/movingRun.reduce((a,b)=>a+b,0);minimumMovingWindowHz=Math.min(minimumMovingWindowHz??Infinity,hz);}
    }
    return {all:group(steps),duringMotion:group(steps.filter(s=>s.moving)),stationary:group(steps.filter(s=>!s.moving)),
        minimumMovingWindowHz,
        stages:CROSSOVER_STAGES.map((label,stage)=>({label,...group(steps.filter(s=>s.stage===stage)),duringMotion:group(steps.filter(s=>s.stage===stage&&s.moving))})),
        maximumShapeDifferenceMm:Math.max(0,...steps.map(s=>s.shapeDifferenceMm??0))};
}

/** Commands are sampled once at the common-step boundary and consumed only
 * after both accesses commit. Cooperative retries never consume another input. */
export function createCrossoverScenarioRecording({now=()=>performance.now(),onChange=()=>{},onChunk=()=>{},maxSteps=36000}={}) {
    let mode='idle',record=null,source=null,pending=null,lastWall=0,cpuMs=0,frames=0,movingSteps=0,movingWallMs=0;
    const fail=message=>finish(false,message);
    function finish(complete,error=null) {
        if(!record||!['recording','replaying'].includes(mode))return;
        record.complete=complete;record.error=error;record.wallMs=now()-record.startedAt;
        record.summary=summarizeCrossoverRecording(record.steps);record.renderFps=frames*1000/Math.max(1,record.wallMs);
        record.allStagesMarked=record.checkpoints.length===CROSSOVER_STAGES.length;
        record.fullManeuverRecorded=complete&&record.allStagesMarked;
        record.inputSequenceVerified=complete&&mode==='replaying';
        // Matching commands and geometry does not independently prove that
        // the operator reached the intended anatomical branches.
        record.fullManeuverVerified=false;
        record.goalAchieved=false; // A recording alone is not an optimization acceptance.
        mode='finished';pending=null;onChunk(record,true);onChange();
    }
    function start(configuration,replay=null,{shapeDifferenceLimitMm=null,forceCertificateLimit=1e-6}={}) {
        if(!Number.isFinite(forceCertificateLimit)||forceCertificateLimit<=0||forceCertificateLimit>1e-4)
            throw new Error('Invalid experimental force certificate limit');
        if(shapeDifferenceLimitMm!==null&&(!Number.isFinite(shapeDifferenceLimitMm)||shapeDifferenceLimitMm<=0))
            throw new Error('Limit różnicy kształtu musi być dodatni albo wyłączony (null).');
        if(['recording','replaying'].includes(mode))throw new Error('Najpierw zakończ bieżące nagranie.');
        if(replay)validateCrossoverRecording(replay,{maxSteps});
        if(replay&&JSON.stringify(replay.configuration)!==JSON.stringify(configuration))throw new Error('Ustawienia anatomii, sprzętu lub przepływu nie pasują do nagrania.');
        source=replay;mode=replay?'replaying':'recording';lastWall=now();cpuMs=frames=movingSteps=movingWallMs=0;pending=null;
        record={schemaVersion:CROSSOVER_RECORDING_VERSION,kind:'crossover-input-sequence',recordedAt:new Date().toISOString(),dt:1/60,
            replayOf:replay?.recordedAt??null,
            validation:{mode:shapeDifferenceLimitMm===null?'functional':'shape-limit',shapeDifferenceLimitMm,forceCertificateLimit,
                shapeComparisonRole:shapeDifferenceLimitMm===null?'diagnostic':'acceptance',
                functionalReviewRequired:true},
            startedAt:lastWall,configuration:structuredClone(configuration),steps:[],checkpoints:replay?structuredClone(replay.checkpoints):[],
            complete:false,fullManeuverVerified:false,goalAchieved:false,
            timingDefinition:'physicsHz counts common accepted steps per wall-clock second; cpuMs measures main-thread slices only. Worker compute and transport timings are recorded separately in providerMs and solverWork.',
            coverage:'Continuous commands from empty tools; each accepted step has a shape reference. Stage placement is confirmed by the operator.'};
        onChange();return record;
    }
    return {
        get mode(){return mode;},get record(){return record;},get active(){return mode==='recording'||mode==='replaying';},
        get progress(){return {motionHz:movingWallMs>0?movingSteps*1000/movingWallMs:null,
            stage:source?.steps[record?.steps.length]?.stage??record?.checkpoints.length??0};},
        startRecording:configuration=>start(configuration),startReplay:(configuration,replay,options)=>start(configuration,replay,options),
        boundary(inputs,configuration) {
            if(!this.active)return inputs;
            if(pending)throw new Error('Repeated scenario preparation before common publication');
            if(JSON.stringify(configuration)!==JSON.stringify(record.configuration)){fail('Zmieniono ustawienia w trakcie scenariusza.');return inputs;}
            const next=mode==='replaying'?source.steps[record.steps.length]?.inputs:inputs;
            try{validateCrossoverInputs(next);}catch(error){fail(error.message);return inputs;}
            pending=structuredClone(next);return structuredClone(pending);
        },
        slice(milliseconds,result) {
            if(!this.active)return;cpuMs+=milliseconds;
            if(result.error||result.terminal)fail(result.error?.message??result.status??'Odrzucono krok fizyki.');
        },
        frame(){if(this.active)frames++;},
        commit({dt,diagnostics,shape,tools}) {
            if(!this.active)return;
            if(!pending||dt!==record.dt||!validQuality(diagnostics?.map(d=>d.last),dt,record.validation.forceCertificateLimit)||!validShape(shape)||!validTools(tools)){
                fail('Krok nie przeszedł kontroli obu solverów, kształtu lub pozycji sprzętu.');return;
            }
            const index=record.steps.length,reference=source?.steps[index];let difference=0;
            if(reference&&(['wire','catheter'].some(key=>Math.abs(reference.tools[key]-tools[key])>1e-8)||
                ['wireRotation','catheterRotation'].some(key=>Math.abs(Math.atan2(Math.sin(reference.tools[key]-tools[key]),Math.cos(reference.tools[key]-tools[key])))>1e-10))) {
                fail('Długość wsunięcia lub obrót narzędzia nie zgadza się z nagraniem.');return;
            }
            try{if(reference)difference=crossoverShapeDifference(reference.shape,shape);}catch(error){fail(error.message);return;}
            const wall=now();
            const isMoving=moving(pending),intervalMs=wall-lastWall;
            if(isMoving){movingSteps++;movingWallMs+=intervalMs;}
            record.steps.push({inputs:pending,shape,tools,quality:diagnostics.map(captureQuality),stage:reference?.stage??record.checkpoints.length,
                moving:isMoving,intervalMs,cpuMs,
                providerMs:diagnostics.map(d=>d.last?.cpuMs??0),solverWork:diagnostics.map(captureWork),shapeDifferenceMm:difference});
            lastWall=wall;pending=null;cpuMs=0;
            const limit=record.validation.shapeDifferenceLimitMm;
            if(limit!==null&&difference>limit){fail(`Różnica kształtu ${difference.toFixed(4)} mm przekracza ${limit.toLocaleString('pl-PL')} mm.`);return;}
            if(record.steps.length%120===0){onChunk(record,false);onChange();}
            if(mode==='replaying'&&record.steps.length===source.steps.length)finish(true);
            else if(record.steps.length>=maxSteps)fail('Osiągnięto limit długości nagrania; zapisz dotychczasowy przebieg.');
        },
        checkpoint() {
            if(mode!=='recording'||!record.steps.length||pending)throw new Error('Punkt kontrolny wymaga granicy zaakceptowanego kroku.');
            const stage=record.checkpoints.length;if(stage>=CROSSOVER_STAGES.length)return;
            if(record.checkpoints.at(-1)?.afterStep===record.steps.length)throw new Error('Kolejny etap wymaga dalszego przebiegu symulacji.');
            record.checkpoints.push({stage,label:CROSSOVER_STAGES[stage],afterStep:record.steps.length,tools:structuredClone(record.steps.at(-1).tools)});
            if(stage===CROSSOVER_STAGES.length-1)finish(true);else{onChunk(record,false);onChange();}
        },
        finishRecording() {
            if(mode==='recording'&&record.steps.length&&!pending)finish(true);
        },
        stop:()=>finish(false,'Nagranie przerwane przed potwierdzeniem wszystkich etapów.'),fail
    };
}
