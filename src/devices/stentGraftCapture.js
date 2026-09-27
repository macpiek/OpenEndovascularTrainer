import {crownMaterial} from './stentGraftCrownKinematics.js';

// Reduced elastic graft support condensed through rigid crown arms. These are
// simulator stiffness units, not measured manufacturer material parameters.
export const CAPTURE_SUPPORT_STIFFNESS=180;
export function captureConfiguration(device) {
    if(device.type!=='body'||device.tipRelease>=1||!device.parts?.length)return null;
    const material=device.crownMaterial??=crownMaterial(device.diameter,device.dimensionScale??1);
    const latch=device.deliveryPath.sample(device.position+12*(device.dimensionScale??1));
    return {latch,material};
}
export function capturedRoot(point,latch,length) {
    return latch.clone().addScaledVector(point.clone().sub(latch).normalize(),length);
}
export function captureLinks(device) {
    const capture=captureConfiguration(device);
    if(!capture||!device.captureRestRoots)return null;
    const base=device.deliveryPath.sample(device.position);
    return {deviceId:device.id,offset:capture.latch.clone().sub(base).toArray(),
        armLength:capture.material.armLength,stiffness:CAPTURE_SUPPORT_STIFFNESS,
        roots:device.captureRestRoots.map(p=>p.toArray())};
}

// Projecting an elastic root onto the rigid arm's sphere eliminates its three
// displacement unknowns. E = k/2 (|latch - restRoot| - armLength)^2. The arm
// itself never strains: its reaction is transmitted to the delivery shaft.
export function captureReaction(capture,latch) {
    let energy=0;const gradient=[0,0,0],hessian=Array.from({length:3},()=>[0,0,0]),reactions=[];
    for(const root of capture.roots) {
        const delta=latch.map((v,k)=>v-root[k]),length=Math.hypot(...delta),n=delta.map(v=>v/Math.max(1e-9,length));
        const strain=length-capture.armLength,k=capture.stiffness;
        energy+=.5*k*strain*strain;
        for(let a=0;a<3;a++) {
            gradient[a]+=k*strain*n[a];
            for(let b=0;b<3;b++)hessian[a][b]+=k*(n[a]*n[b]+strain/Math.max(1e-9,length)*((a===b?1:0)-n[a]*n[b]));
        }
        reactions.push(n.map(v=>k*strain*v));
    }
    return {energy,gradient,hessian,deliveryForce:gradient.map(v=>-v),graftForces:reactions};
}
export function createCapturePotential(capture) {
    const sample=()=>({gap:1,jacobian:[0,0,0,0,0,0]});
    sample.sharedAxisGeometryOnly=true;sample.graftCapture=true;sample.captureLinks=capture;
    sample.addPotential=(state,withTangent)=>{
        if(!capture)return 0;
        const catheter=state.materials.find(m=>m.spec.id==='catheter');if(!catheter)return 0;
        const s=catheter.spec.insertion;let e=0;
        while(e<state.coordinates.length-2&&state.coordinates[e+1]<s)e++;
        const t=(s-state.coordinates[e])/(state.coordinates[e+1]-state.coordinates[e]);
        const p=state.positions[e].map((v,k)=>v*(1-t)+state.positions[e+1][k]*t+state.origin[k]+capture.offset[k]);
        const reaction=captureReaction(capture,p),{layout,chain}=state;
        const dofs=[layout.positions[e],layout.positions[e+1]].flatMap(i=>[i,i+1,i+2]),H=chain.tangent??chain.hessian,half=layout.band-1,width=2*half+1;
        for(let i=0;i<6;i++) {
            const wi=i<3?1-t:t;chain.gradient[dofs[i]]+=wi*reaction.gradient[i%3];
            if(withTangent)for(let j=0;j<6;j++) {
                const value=wi*(j<3?1-t:t)*reaction.hessian[i%3][j%3],a=dofs[i],b=dofs[j];
                if(chain.tangent)H[a*width+b-a+half]+=value;else if(a>=b)H[a*layout.band+a-b]+=value;
            }
        }
        state.graftCaptureStats={deliveryForce:reaction.deliveryForce,graftForces:reaction.graftForces,energy:reaction.energy};
        return reaction.energy;
    };
    return sample;
}
