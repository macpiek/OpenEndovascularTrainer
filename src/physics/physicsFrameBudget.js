/** Run bounded synchronous slices. Only accepted timesteps consume the step
 * limit; pending slices retain the transaction's prepared inputs. Numerical
 * rejection always stops, even when there is spare render time. */
export function runPhysicsFrameBudget({hasDebt,canAttempt,canFit,attempt,maxSteps=2,maxSlices=6,resumePending=false}) {
    let steps=0,slices=0;
    while(steps<maxSteps&&slices<maxSlices&&hasDebt()&&canAttempt()) {
        if(slices>0&&!canFit())break;
        const result=attempt();slices++;
        if(result.accepted)steps++;
        else if(!resumePending||!result.pending)break;
    }
    return {steps,slices};
}

/** Use only spare time before the next render deadline. Unlike the frame
 * entry, even the first idle slice must fit. The caller owns cancellation,
 * visibility and transaction epochs; this synchronous loop owns no timers. */
export function runPhysicsIdleBudget({deadline,now,guardMs,maxSliceMs,hasDebt,canAttempt,canFit,attempt,maxSlices=6}) {
    let steps=0,slices=0;
    while(slices<maxSlices) {
        const room=deadline-now()-guardMs;
        if(!hasDebt()||!canAttempt()||!canFit(room))break;
        if(attempt(Math.min(maxSliceMs,room)))steps++;
        slices++;
    }
    return {steps,slices};
}
