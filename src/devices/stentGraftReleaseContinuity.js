import {graftScale} from './stentGraftModels.js';

export const GRAFT_RELEASE_SPEED_MM_S=24;

// A common continuation parameter preserves the linear sewn seam and convex
// upper bounds on fabric chords. This is the committed mechanical geometry:
// collision snapshots and scaffold attachments are built only afterwards.
export function continueGraftRelease(device,dt) {
    if(!(dt>0))return;
    let maximum=0;
    for(const part of device.parts) {
        const a=part.mesh.geometry.attributes.position.array,b=part.releasePreviousPositions;
        if(!b)continue;
        for(let i=0;i<a.length;i+=3)maximum=Math.max(maximum,Math.hypot(a[i]-b[i],a[i+1]-b[i+1],a[i+2]-b[i+2]));
    }
    const fraction=Math.min(1,GRAFT_RELEASE_SPEED_MM_S*graftScale(device)*dt/Math.max(1e-12,maximum));
    device.releaseMotionLimited=fraction<1;
    for(const part of device.parts) {
        const p=part.mesh.geometry.attributes.position,a=p.array,b=part.releasePreviousPositions;
        if(!b)continue;
        if(fraction<1) {
            for(let i=0;i<a.length;i++)a[i]=b[i]+fraction*(a[i]-b[i]);
            // Captured roots stay on the fixed-length metal-arm sphere.
            if(part.sewnCapture)for(let j=0;j<part.sides;j++) {
                const {latch,material}=part.sewnCapture,i=j*3;
                const x=a[i]-latch.x,y=a[i+1]-latch.y,z=a[i+2]-latch.z;
                const scale=material.armLength/Math.max(1e-12,Math.hypot(x,y,z));
                a[i]=latch.x+x*scale;a[i+1]=latch.y+y*scale;a[i+2]=latch.z+z*scale;
            }
            p.needsUpdate=true;
            if(part.contactBasePositions)part.contactBasePositions.set(a);
        }
        // A coarse committed step may already apply the entire equilibrium
        // proposal. Do not restart a second relaxation merely because that
        // completed step moved the mesh (important for paused/benchmark steps).
        part.releaseRelaxing=(fraction<1||Math.exp(-12*dt)>1e-4)&&
            a.some((v,i)=>Math.abs(v-b[i])>.0001);
    }
    // Branch roots are eliminated DOFs, including on the first step where
    // the shared outlet becomes exposed. Never blend them independently.
    if(device.sewnJunctionBindings&&device.parts[0].exposure.at(-1)>0) {
        const trunk=device.parts[0].mesh.geometry.attributes.position.array;
        for(let k=1;k<device.parts.length;k++) {
            const part=device.parts[k],p=part.mesh.geometry.attributes.position;
            for(let j=0;j<part.sides;j++)for(let c=0;c<3;c++) {
                const {indices,weights}=device.sewnJunctionBindings[k-1][j];
                p.array[j*3+c]=weights.reduce((sum,w,n)=>sum+w*trunk[indices[n]+c],0);
            }
            if(fraction<1)p.needsUpdate=true;
            if(part.contactBasePositions)part.contactBasePositions.set(p.array);
        }
    }
}
