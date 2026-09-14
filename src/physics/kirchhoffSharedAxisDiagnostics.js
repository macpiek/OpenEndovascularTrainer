import {sharedAxisOuterMaterialAt} from './kirchhoffSharedAxisNative.js';

/** Diagnostics are measured on the authoritative shared mesh, not its
 * resampled native rendering buffers (which have a different discretization). */
export function measureSharedAxisQuality(s,rows=[]) {
    const bodies=s.materials.map(({spec,last})=>{
        let maxLengthError=0,maxBendAngleDegrees=0,maxBendNode=-1,maxSpeed=0,finite=true,activeWallContacts=0,currentNormalLoad=0;
        for(let i=0;i<=last;i++) {
            finite&&=s.positions[i].every(Number.isFinite);
            maxSpeed=Math.max(maxSpeed,Math.hypot(...(s.velocities?.[i]??[0,0,0])));
            if(i===last)continue;
            const a=s.positions[i+1].map((v,k)=>v-s.positions[i][k]),l=Math.hypot(...a),rest=s.coordinates[i+1]-s.coordinates[i];
            maxLengthError=Math.max(maxLengthError,Math.abs(l-rest)/rest);
            if(i+1<last){const b=s.positions[i+2].map((v,k)=>v-s.positions[i+1][k]),m=Math.hypot(...b),angle=Math.acos(Math.max(-1,Math.min(1,a.reduce((v,x,k)=>v+x*b[k],0)/(l*m))))*180/Math.PI;
                if(angle>maxBendAngleDegrees){maxBendAngleDegrees=angle;maxBendNode=i+1;}}
        }
        for(const r of rows)if(r.kind==='wall'&&r.subtype!=='bend-limit'&&r.multiplier>0) {
            const owner=sharedAxisOuterMaterialAt(s,r.edge,r.witness?.t??r.sampleT??1,r.witness?.owner);
            if(owner.spec.id===spec.id){activeWallContacts++;currentNormalLoad+=r.multiplier;}
        }
        return {id:spec.id,finite,maxLengthError,maxBendAngleDegrees,maxBendNode,maxBendLimitDegrees:Number.isFinite(s.maxBendAngle)?s.maxBendAngle*180/Math.PI:null,maxSpeed,activeWallContacts,currentNormalLoad};
    });
    let maxPenetration=0;
    for(const r of rows)if(r.kind==='wall'&&r.subtype!=='bend-limit')maxPenetration=Math.max(maxPenetration,-r.gap);
    return {mesh:'shared-axis',interToolRows:0,finite:bodies.every(b=>b.finite),maxPenetration,contacts:bodies.reduce((v,b)=>v+b.activeWallContacts,0),bodies};
}
