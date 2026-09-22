import {graftCoverWithdrawal} from './stentGraftDeployment.js';
/** The delivery wire belongs inside its own graft. Released ring sections
 * prescribe a conservative circular lumen; no axial force/friction is added.
 * Other-access wires still see the ordinary two-sided fabric surface. */
export function graftLumenAt(sections,coordinate,point,radius=0) {
    const section=sections?.find(s=>coordinate>=s.start&&coordinate<=s.end);
    if(!section)return null;
    const t=(coordinate-section.start)/(section.end-section.start);
    const center=section.a.map((v,k)=>v*(1-t)+section.b[k]*t);
    const delta=point.map((v,k)=>v-center[k]),axis=section.b.map((v,k)=>v-section.a[k]);
    const length=Math.hypot(...axis);if(length<1e-8)return null;
    for(let k=0;k<3;k++)axis[k]/=length;
    const along=delta.reduce((sum,v,k)=>sum+v*axis[k],0);
    const radial=delta.map((v,k)=>v-along*axis[k]),distance=Math.hypot(...radial);
    const clearance=Math.max(.05,section.radiusA*(1-t)+section.radiusB*t-radius);
    return {axis,distance,clearance,penetration:distance-clearance,normal:radial.map(v=>v/Math.max(1e-12,distance))};
}

export function graftLumenSections(device) {
    const sections=[],exposed=device.phase==='deployed'?Infinity:graftCoverWithdrawal(device)-device.coverLead-2;
    for(const part of device.parts.slice(0,device.type==='body'?2:1)) {
        const rings=part.points.map((_,row)=>{
            const center=[0,0,0];
            for(let j=0;j<part.sides;j++)for(let k=0;k<3;k++)center[k]+=part.target[(row*part.sides+j)*3+k]/part.sides;
            let radius=Infinity;
            for(let j=0;j<part.sides;j++)radius=Math.min(radius,Math.hypot(...center.map((v,k)=>part.target[(row*part.sides+j)*3+k]-v)));
            return {center,radius};
        });
        for(let i=1;i<part.rows;i++) {
            if(part.releaseOffset+part.path.coordinates[i]>exposed)break;
            sections.push({start:device.implantPosition-part.releaseOffset-part.path.coordinates[i],
                end:device.implantPosition-part.releaseOffset-part.path.coordinates[i-1],
                a:rings[i].center,b:rings[i-1].center,radiusA:rings[i].radius,radiusB:rings[i-1].radius});
        }
    }
    return sections;
}
