export const WIRE60_BENCHMARK_MODE='wire60-catheter';
export function wire60BenchmarkSteps(dt) {
    return {wire:Math.ceil(600/(44*dt)),catheter:Math.ceil(600/(52*dt))};
}
export function sampleWire60Benchmark(elapsedMs,dt,out,catheterType='berenstein') {
    const counts=wire60BenchmarkSteps(dt),step=Math.floor(elapsedMs/(dt*1000)+1e-6);
    Object.assign(out,{guidewireAdvance:0,catheterAdvance:0,guidewireRotation:0,catheterRotation:0,catheterType});
    if(step<counts.wire)out.guidewireAdvance=Math.max(0,Math.min(1,(600-step*44*dt)/(44*dt)));
    else if(step<counts.wire+counts.catheter)out.catheterAdvance=Math.max(0,Math.min(1,(600-(step-counts.wire)*52*dt)/(52*dt)));
    return out;
}
