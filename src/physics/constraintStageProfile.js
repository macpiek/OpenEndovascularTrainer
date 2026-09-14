import { createLineSearchStats } from './kirchhoffLineSearch.js';
const COST_KEYS = ['assemblyMs', 'solveMs', 'applyMs', 'snapshotMs', 'restoreMs', 'measureMs',
    'condensedSetupMs', 'schurMs', 'contactSolveMs', 'reconstructionMs', 'seedMs',
    'solveCalls', 'applyCalls', 'measureCalls', 'fullMeasureCalls', 'snapshots', 'restores'];
const PHASE_KEYS = ['total', 'constraints', 'narrowPhase', 'integrate', 'velocity'];
const KEYS = [...COST_KEYS, ...PHASE_KEYS, 'trials', 'backtracks', 'factorizations',
    'linearMs','frictionMs','providerCpuMs','activeStepCpuMs','wallMs','iterations','substepAttempts','geometryRestarts','frictionIterations'];

// Bounded, allocation-free recording. Percentiles cover the most recent
// capacity steps; means and maxima cover every recorded benchmark step.
export class ConstraintStageProfile {
    constructor(capacity = 8192) {
        this.capacity = capacity;
        this.fields = Object.fromEntries(KEYS.map(key => [key,
            { samples: new Float64Array(capacity), sum: 0, maximum: 0 }]));
        this.reset();
    }
    reset() {
        this.lineSearch = createLineSearchStats();
        this.steps = 0;
        this.failedSteps = 0;
        this.pendingSlices=0;this.sleepingSteps=0;this.solverSteps=0;this.mode='kirchhoff-direct';this.lastResult=null;
        this.timingSource='world-step';this.firstFailure=null;
        for (const field of Object.values(this.fields)) {
            field.sum = 0; field.maximum = 0;field.count=0;
        }
    }
    add(key, value) {
        const field = this.fields[key];
        if(!Number.isFinite(value)){field.samples[this.steps%this.capacity]=NaN;return;}
        field.samples[this.steps % this.capacity] = value;field.count++;
        field.sum += value;
        field.maximum = Math.max(field.maximum, value);
    }
    record(world,{fullStepCpuMs}={}) {
        if(world.wholeStepSystem?.id==='shared-axis') {
            this.mode='shared-axis';
            const result=world.lastStepResult;
            if(!result||result===this.lastResult)return false;
            this.lastResult=result;
            if(!result.accepted) {
                if(result.status==='shared-axis-pending')this.pendingSlices++;
                else {this.failedSteps++;this.firstFailure??={status:result.status,error:result.diagnostics?.last?.error??result.message??null};}
                return false;
            }
            const sleeping=result.status==='sleeping',last=result.diagnostics?.last,t=last?.timings;
            this.sleepingSteps+=Number(sleeping);this.solverSteps+=Number(!sleeping);
            const measured=Number.isFinite(fullStepCpuMs),total=measured?fullStepCpuMs:sleeping?world.timings?.total?.last:last?.cpuMs;
            this.timingSource=measured?'preparation-solver-publication-cpu':'provider-cpu';
            const values={total,activeStepCpuMs:sleeping?NaN:total,providerCpuMs:sleeping?0:last?.cpuMs,wallMs:sleeping?total:last?.wallMs,
                assemblyMs:sleeping?0:t?.assemblyMs,linearMs:sleeping?0:t?.linearMs,solveMs:sleeping?0:t?.linearMs,frictionMs:sleeping?0:t?.frictionMs,
                constraints:sleeping?0:t&&t.assemblyMs+t.linearMs+t.frictionMs};
            for(const key of ['iterations','substepAttempts','geometryRestarts','frictionIterations','backtracks','factorizations'])values[key]=sleeping?0:last?.[key];
            for(const key of KEYS)this.add(key,values[key]);
            this.steps++;return true;
        }
        const search = world.lastJointLineSearch;
        if (search) {
            for (const key of ['accepted', 'rejected']) for (let i = 0; i < 8; i++)
                this.lineSearch[key][i] += search[key][i];
            for (const key of ['boundaryWorstChanged', 'predictedStarts', 'largerFallbacks', 'earlyRejections'])
                this.lineSearch[key] += search[key] ?? 0;
            for (const group of ['rejectionTerms', 'boundaryKinds']) for (const key in search[group])
                this.lineSearch[group][key] = (this.lineSearch[group][key] ?? 0) + search[group][key];
        }
        for (const key of COST_KEYS) this.add(key, world.lastJointCosts?.[key] ?? 0);
        for (const key of PHASE_KEYS) this.add(key, world.timings[key].last);
        this.add('trials', world.lastJointTrialEvaluations ?? 0);
        this.add('backtracks', world.lastJointBacktracks ?? 0);
        this.add('factorizations', world.lastJointFactorizations ?? 0);
        this.failedSteps += Number(Boolean(world.lastJointNonlinearFailure));
        for(const key of ['linearMs','frictionMs','providerCpuMs','activeStepCpuMs','wallMs','iterations','substepAttempts','geometryRestarts','frictionIterations'])this.add(key,NaN);
        this.steps++;return true;
    }
    report() {
        const count = Math.min(this.steps, this.capacity);
        return {mode:this.mode,timingSource:this.timingSource,lineSearch:this.mode==='shared-axis'?null:structuredClone(this.lineSearch), steps: this.steps, failedSteps: this.failedSteps, percentileSteps: count,
            pendingSlices:this.pendingSlices,sleepingSteps:this.sleepingSteps,solverSteps:this.solverSteps,firstFailure:this.firstFailure?{...this.firstFailure}:null,
            fields: Object.fromEntries(KEYS.map(key => {
                const field = this.fields[key];
                const sorted = field.samples.slice(0, count).filter(Number.isFinite).sort();
                return [key, {sampleCount:field.count,percentileSamples:sorted.length,mean:field.count?field.sum/field.count:null,
                    p95:sorted.length?sorted[Math.floor((sorted.length-1)*.95)]:null,
                    maximum:field.count?field.maximum:null,sum:field.count?field.sum:null }];
            })) };
    }
}

/** Authoritative shared geometry is independent of native display resampling.
 * Legacy material-velocity/projection channels are unavailable for this model. */
export function recordSharedAxisQuality(envelope,result) {
    if(!result?.accepted)return false;
    if(envelope.source!=='shared-axis-quality') {
        envelope.source='shared-axis-quality';envelope.missingQualitySteps=0;
        envelope.solverSteps=0;envelope.sleepingSteps=0;
        for(const key of Object.keys(envelope))if(key.startsWith('maxGuidewire')||key.startsWith('guidewireRelease'))envelope[key]=null;
        for(const key of ['maxTransientPenetrationMm','maxTransientPenetrationStep','maxPostStepPenetrationBodyId','maxPostStepPenetrationSegment',
            'maxPostStepPenetrationT','maxPostStepPenetrationX','maxPostStepPenetrationY','maxPostStepPenetrationZ',
            'maxSegmentErrorNodeIndex','maxBendX','maxBendY','maxBendZ'])envelope[key]=null;
        envelope.maxSharedAxisSpeedMmPerSecond=0;envelope.maxCertifiedResidual=0;envelope.maxBendLimitDegrees=null;
    }
    envelope.steps++;
    if(result.status==='sleeping'){envelope.sleepingSteps++;return true;}
    envelope.solverSteps++;
    const last=result.diagnostics?.last,q=last?.quality;
    if(!q||!q.bodies?.length||!Number.isFinite(q.maxPenetration)) {
        envelope.missingQualitySteps++;envelope.finite=false;return false;
    }
    envelope.finite=envelope.finite&&q.finite===true;
    if(q.maxPenetration>envelope.maxPostStepPenetrationMm){envelope.maxPostStepPenetrationMm=q.maxPenetration;envelope.maxPostStepPenetrationStep=envelope.steps;}
    if(Number.isFinite(last.certificateBound))envelope.maxCertifiedResidual=Math.max(envelope.maxCertifiedResidual,last.certificateBound);
    for(const b of q.bodies) {
        envelope.finite=envelope.finite&&b.finite===true&&[b.maxLengthError,b.maxBendAngleDegrees,b.maxSpeed].every(Number.isFinite);
        if(b.maxLengthError*100>envelope.maxSegmentErrorPercent) {
            envelope.maxSegmentErrorPercent=b.maxLengthError*100;envelope.maxSegmentErrorBodyId=b.id;envelope.maxSegmentErrorStep=envelope.steps;
        }
        if(b.maxBendAngleDegrees>envelope.maxBendAngleDegrees) {
            envelope.maxBendAngleDegrees=b.maxBendAngleDegrees;envelope.maxBendBodyId=b.id;envelope.maxBendNodeIndex=b.maxBendNode;envelope.maxBendStep=envelope.steps;
        }
        if(Number.isFinite(b.maxBendLimitDegrees))envelope.maxBendLimitDegrees=Math.min(envelope.maxBendLimitDegrees??Infinity,b.maxBendLimitDegrees);
        envelope.maxSharedAxisSpeedMmPerSecond=Math.max(envelope.maxSharedAxisSpeedMmPerSecond,b.maxSpeed);
    }
    return true;
}

/** Rendering at 60 FPS does not certify that accepted physics follows time.
 * Preserve the old 4/6 ms share of a 120 Hz tick: at 60 Hz this is 8/12 ms. */
export function assessSharedAxisBenchmarkTiming({profile,fixedDt,executedSteps,admittedSeconds,startBacklogSeconds,endBacklogSeconds,peakBacklogSeconds,renderingPass}) {
    const limits={meanStepMs:4*fixedDt*120,p95StepMs:6*fixedDt*120},total=profile.fields.total,active=profile.fields.activeStepCpuMs;
    const samplesPass=profile.steps>0&&profile.solverSteps>0&&profile.steps===executedSteps&&total.sampleCount===profile.steps&&active.sampleCount===profile.solverSteps;
    const physicsBudgetPass=samplesPass&&profile.failedSteps===0&&Number.isFinite(active.mean)&&Number.isFinite(active.p95)&&
        active.mean<=limits.meanStepMs&&active.p95<=limits.p95StepMs;
    const completedSeconds=executedSteps*fixedDt;
    const available=[fixedDt,admittedSeconds,startBacklogSeconds,endBacklogSeconds,peakBacklogSeconds].every(Number.isFinite)&&fixedDt>0&&admittedSeconds>0;
    const accountingErrorSeconds=admittedSeconds+startBacklogSeconds-completedSeconds-endBacklogSeconds;
    const simulationRealtimePass=samplesPass&&available&&Math.abs(accountingErrorSeconds)<=1e-6&&
        endBacklogSeconds<=startBacklogSeconds+fixedDt+1e-6&&peakBacklogSeconds<=startBacklogSeconds+2*fixedDt+1e-6;
    return {limits,samplesPass,physicsBudgetPass,simulationRealtimePass,completedSeconds,
        simulatedToAdmittedTimeRatio:available?completedSeconds/admittedSeconds:null,
        renderingPass:renderingPass===true,realTime60FpsPass:physicsBudgetPass&&simulationRealtimePass&&renderingPass===true};
}
