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
