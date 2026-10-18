import {sharedAxisOuterIntervals} from './kirchhoffSharedAxisNative.js';

// The rod stiffness/mass use model units, not calibrated SI units. Keep this
// conversion explicit; fluid density, viscosity, geometry and velocity are SI/mm.
export const BLOOD_FORCE_MODEL_SCALE = 1e6;

/** Numeric, immutable-for-the-step samples. No live flow-network reference is
 * retained across cooperative yields, subdivisions or a rejected step. */
export function captureBloodFlow(state, network, sheathLength, requestedTip = state.coordinates.at(-1)) {
    if (!network?.sampleVelocityCoordinates) return null;
    const coordinates = state.coordinates.slice();
    if (requestedTip > coordinates.at(-1)) coordinates.push(requestedTip);
    const velocities = [], sample = {}, origin = state.origin ?? [0,0,0];
    let edge = 0;
    for (const coordinate of coordinates) {
        while (edge+2 < state.coordinates.length && coordinate > state.coordinates[edge+1]) edge++;
        const t=(coordinate-state.coordinates[edge])/(state.coordinates[edge+1]-state.coordinates[edge]);
        const p=state.positions[edge],q=state.positions[edge+1];
        if(coordinate<sheathLength){velocities.push([0,0,0]);continue;}
        network.sampleVelocityCoordinates(...p.map((v,k)=>origin[k]+v+t*(q[k]-v)),sample);
        const velocity=[sample.velocityX,sample.velocityY,sample.velocityZ];
        const inside=sample.edgeIndex>=0 && sample.distance<=sample.radius;
        velocities.push(inside&&velocity.every(Number.isFinite)?velocity:[0,0,0]);
    }
    return {time:network.time??0,coordinates,velocities,sheathLength,
        density:network.hemodynamics?.bloodDensityKgPerM3??1060,
        viscosity:network.hemodynamics?.bloodViscosityPaS??.0035,forceScale:BLOOD_FORCE_MODEL_SCALE};
}

function velocityAt(flow, coordinate) {
    const xs=flow.coordinates;let lo=0,hi=xs.length-1;
    while(lo+1<hi){const mid=(lo+hi)>>>1;if(xs[mid]<=coordinate)lo=mid;else hi=mid;}
    const t=Math.max(0,Math.min(1,(coordinate-xs[lo])/(xs[hi]-xs[lo])));
    return flow.velocities[lo].map((v,k)=>v+t*(flow.velocities[hi][k]-v));
}

/** Freeze a positive drag coefficient at the accepted pose, then integrate its
 * relative-velocity law implicitly. Lumping into kinetic mass/target works in
 * both Kirchhoff and PD; it cannot overshoot the blood velocity on a free DOF.
 * Only the outermost material sees blood: no double load on a contained wire. */
export function applyBloodFlowToPrediction(state, dynamic, flow) {
    if(!flow)return;
    const count=state.positions.length,weights=new Float64Array(count),targets=Array.from({length:count},()=>[0,0,0]);
    let exposedLength=0,maxSpeed=0;
    for(let e=0;e+1<count;e++) {
        const ca=state.coordinates[e],cb=state.coordinates[e+1],span=cb-ca;
        if(cb<=flow.sheathLength)continue;
        const a=state.positions[e],b=state.positions[e+1],direction=b.map((v,k)=>v-a[k]),norm=Math.hypot(...direction);
        if(norm<1e-9)continue;
        for(const interval of sharedAxisOuterIntervals(state,e)) {
            const start=Math.max(interval.start,(flow.sheathLength-ca)/span),end=interval.end;
            if(end<=start||!interval.material)continue;
            const t=(start+end)*.5,length=span*(end-start),diameter=2*interval.material.body.radius;
            const u=velocityAt(flow,ca+t*span);
            const relative=u.map((v,k)=>v-((1-t)*(state.velocities?.[e]?.[k]??0)+t*(state.velocities?.[e+1]?.[k]??0)));
            const speed=Math.hypot(...relative),axial=relative.reduce((sum,v,k)=>sum+v*direction[k]/norm,0);
            // Slender tools offer much less resistance along their axis.
            const orientation=speed>1e-9?.06+.94*Math.max(0,1-(axial/speed)**2):.06;
            // A fixed 5 mm hydrodynamic reference length avoids changing drag
            // when the adaptive mechanical mesh splits or merges an edge.
            const viscous=4*Math.PI*flow.viscosity*(length*.001)/(Math.log(1+10/diameter)+.5);
            const inertial=.5*flow.density*1.2*(diameter*.001)*(length*.001)*(speed*.001);
            const coefficient=(viscous+inertial)*.001*flow.forceScale*orientation;
            for(const [node,fraction] of [[e,1-t],[e+1,t]]){
                const c=coefficient*fraction;weights[node]+=c;
                for(let k=0;k<3;k++)targets[node][k]+=c*u[k];
            }
            exposedLength+=length;maxSpeed=Math.max(maxSpeed,Math.hypot(...u));
        }
    }
    const {dt,masses,predicted,positions}=dynamic;
    for(let n=0;n<count;n++){
        const c=weights[n];if(c===0)continue;
        const originalMass=masses[n],mass=originalMass+dt*c;
        for(let k=0;k<3;k++){
            targets[n][k]/=c;
            predicted[n][k]=(originalMass*predicted[n][k]+dt*c*(positions[n][k]+dt*targets[n][k]))/mass;
        }
        masses[n]=mass;
    }
    dynamic.bloodFlow={weights,targets,exposedLength,maxSpeed,time:flow.time};
}
