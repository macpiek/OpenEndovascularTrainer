export const FULL_CYCLE_BENCHMARK_MODE='full-cycle-60hz';

/** Runtime rates: wire moves at 44 mm/s in both directions; catheter at
 * 52 mm/s in and 32 mm/s out. Boundary commands hit the exact endpoint. */
export function fullCyclePhases(dt,target=1000) {
    if(!Number.isFinite(dt)||dt<=0||!Number.isFinite(target)||target<=0||target>1000)throw new RangeError('Invalid full-cycle dt or target');
    let start=0;
    return [['wire-in','wire',44,1],['catheter-in','catheter',52,1],['catheter-out','catheter',32,-1],['wire-out','wire',44,-1]].map(([name,tool,rate,sign])=>{
        const steps=Math.ceil(target/(rate*dt)),phase={name,tool,rate,sign,start,steps,target};start+=steps;return phase;
    });
}
export function fullCycleDurationMs(dt,target=1000){return fullCyclePhases(dt,target).reduce((n,p)=>n+p.steps,0)*dt*1000;}
export function sampleFullCycleBenchmark(elapsedMs,dt,out,catheterType='berenstein',target=1000) {
    const step=Math.floor(elapsedMs/(dt*1000)+1e-6);
    Object.assign(out,{guidewireAdvance:0,catheterAdvance:0,guidewireRotation:0,catheterRotation:0,catheterType});
    const p=fullCyclePhases(dt,target).find(p=>step>=p.start&&step<p.start+p.steps);
    if(p)out[p.tool==='wire'?'guidewireAdvance':'catheterAdvance']=p.sign*Math.max(0,Math.min(1,(target-(step-p.start)*p.rate*dt)/(p.rate*dt)));
    return out;
}

/** Accepted timestamps measure actual physical throughput, independently of
 * render FPS and of the number of cooperative slices per physical timestep. */
export function summarizeFullCycle(steps,{dt=1/60,target=1000,completed=false}={}) {
    const accepted=steps.filter(s=>s.accepted),budgetMs=1000*dt;
    const cpu=accepted.map(s=>s.cpuMs).sort((a,b)=>a-b),q=p=>cpu.length?cpu[Math.floor((cpu.length-1)*p)]:null;
    const windowSteps=Math.round(1/dt);let minHz=null,maxIntervalMs=0;
    for(let i=1;i<accepted.length;i++)maxIntervalMs=Math.max(maxIntervalMs,accepted[i].wallMs-accepted[i-1].wallMs);
    for(let i=windowSteps;i<accepted.length;i++) {
        const elapsed=accepted[i].wallMs-accepted[i-windowSteps].wallMs;
        const hz=elapsed>0?windowSteps*1000/elapsed:0;minHz=minHz===null?hz:Math.min(minHz,hz);
    }
    const last=accepted.at(-1),rejected=steps.filter(s=>!s.accepted).length;
    const completedCycle=completed&&rejected===0&&accepted.some(s=>s.wire>=target-1e-6)&&accepted.some(s=>s.catheter>=target-1e-6)&&!!last&&last.wire<1e-6&&last.catheter<1e-6;
    return {targetMm:target,completedCycle,acceptedSteps:accepted.length,rejectedSteps:rejected,
        budgetMs,p50StepMs:q(.5),p95StepMs:q(.95),p99StepMs:q(.99),maxStepMs:q(1),stepsOverBudget:cpu.filter(v=>v>budgetMs).length,
        minimumOneSecondPhysicsHz:minHz,maxAcceptedIntervalMs:maxIntervalMs};
}
