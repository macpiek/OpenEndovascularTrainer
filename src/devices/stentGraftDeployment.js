import {graftScale} from './stentGraftModels.js';
// Distances are mm, time is committed simulation time. Cover motion is
// reversible and independent of tip capture. Detachment is irreversible.
export const SHEATH_SPEED_MM_S=12;
export const SUPRARENAL_LENGTH_MM=12;
export const NOSECONE_RETRACTION_SPEED_MM_S=12;
export const STENT_RING_PITCH_MM=9;
export const RELEASE_TRANSITION_MM=12;
// A whole neighbouring ring must clear the transition before its target
// surface can participate in rod contact. Rendering and contact share this bound.
export const FULL_OPEN_CLEARANCE_MM=RELEASE_TRANSITION_MM+STENT_RING_PITCH_MM;
const smooth=t=>{t=Math.max(0,Math.min(1,t));return t*t*(3-2*t);};
export function fullyOpenDistance(device) {
    return graftCoverWithdrawal(device)-device.coverLead-FULL_OPEN_CLEARANCE_MM*graftScale(device);
}
export function graftAttached(device) {
    return device.phase==='loaded'||device.phase==='deploying'&&(device.type==='body'?device.tipRelease<1:!device.limbReleased);
}
// After detachment the cover still moves with the delivery shaft, while the
// implant's material origin stays fixed. Exposure must use their relative pose.
export function graftCoverWithdrawal(device) {
    return (device.sheathWithdrawal??0)+((device.implantPosition??device.position)-device.position);
}
export function deliveryNoseState(device) {
    const lead=device.type==='body'?SUPRARENAL_LENGTH_MM*graftScale(device):0;
    const extended=device.position+lead+(device.type==='body'?(device.tipRelease??0)*3*graftScale(device):0);
    const sheathEdge=Math.max(0,device.position+lead-(device.sheathWithdrawal??0));
    const limit=Math.max(0,extended-sheathEdge);
    const retraction=Math.max(0,Math.min(limit,device.noseRetraction??0));
    return {position:extended-retraction,sheathEdge,limit,retraction,remaining:limit-retraction};
}
export function initializeRelease(device) {
    const trunk=device.parts[0];
    for(const [i,part] of device.parts.entries())part.releaseOffset=i===0?0:trunk.path.length;
    device.coverLead=device.type==='body'?SUPRARENAL_LENGTH_MM*graftScale(device):0;
    device.releaseLength=Math.max(...device.parts.map(p=>p.releaseOffset+p.path.length));
    device.sheathTravel=device.coverLead+device.releaseLength+(device.type==='body'?FULL_OPEN_CLEARANCE_MM*graftScale(device):0);
    device.limbReleased=false;device.limbFreeTime=0;device.limbOpening=0;
    device.gateTravel=device.type==='body'
        ?device.coverLead+device.parts[2].releaseOffset+device.parts[2].path.length+FULL_OPEN_CLEARANCE_MM*graftScale(device):0;
    device.gateOpening=0;device.gateFreeTime=0;
    device.ipsiOpening=0;device.ipsiFreeTime=0;
    device.noseRetraction=0;
    device.sheathWithdrawal=0;device.tipRelease=device.type==='body'?0:1;
    device.releaseStage='sheath';
}
export function advanceRelease(device,dt,control) {
    if(!(dt>0))return false;
    const progress=()=>device.sheathWithdrawal+device.tipRelease+(device.noseRetraction??0)+(device.gateOpening??0)+(device.ipsiOpening??0)+(device.limbOpening??0);
    const before=progress();
    const previousEdge=graftCoverWithdrawal(device)-device.coverLead;
    const cover=typeof control==='object'&&control!==null?control.sheath:
        control==='sheath'?1:control==='resheath'?-1:0;
    const tip=control==='tip'||control?.tip===true;
    const coverSpeed=SHEATH_SPEED_MM_S*graftScale(device)*Math.max(-1,Math.min(1,Number(cover)||0));
    device.sheathWithdrawal=Math.max(0,Math.min(device.sheathTravel,
        device.sheathWithdrawal+coverSpeed*dt));
    if(tip&&device.type==='body')device.tipRelease=Math.min(1,device.tipRelease+dt);
    const nose=deliveryNoseState(device);
    device.noseRetraction=Math.min(nose.limit,nose.retraction+
        ((control==='nose'||control?.nose===true)&&device.tipRelease>=1?NOSECONE_RETRACTION_SPEED_MM_S*graftScale(device)*dt:0));
    // Moving the delivery cover after detachment cannot recapture the implant.
    if(device.phase==='deployed')return progress()!==before;
    advanceRingOpenings(device,dt,previousEdge);
    advanceGateOpening(device,dt,previousEdge);
    if(device.type==='body')advanceBranchOpening(device,device.parts[1],'ipsi',dt,previousEdge);
    if(device.type==='limb') {
        const edge=graftCoverWithdrawal(device)-device.coverLead;
        if(!device.limbReleased&&edge>=device.releaseLength-1e-8) {
            device.limbReleased=true;
            device.limbFreeTime=coverSpeed>0?Math.max(0,dt-Math.max(0,device.releaseLength-previousEdge)/coverSpeed):dt;
        } else if(device.limbReleased)device.limbFreeTime+=dt;
        const t=45*device.limbFreeTime;
        device.limbOpening=t>=8?1:1-(1+t)*Math.exp(-t);
    }
    const exposed=Math.max(0,Math.min(device.sheathTravel,graftCoverWithdrawal(device)));
    const completeTravel=device.type==='body'?device.coverLead+device.releaseLength:device.sheathTravel;
    device.releaseStage=exposed>=completeTravel-1e-8
        ?device.tipRelease<1?'tip':'complete':'sheath';
    device.deployment=device.type==='body'
        ?.9*Math.min(1,exposed/completeTravel)+.1*device.tipRelease
        :exposed/device.sheathTravel;
    if(device.releaseStage==='complete')device.deployment=1;
    if(device.type==='body'&&(device.gateOpening<1||device.ipsiOpening<1)) {
        device.deployment=Math.min(device.deployment,1-1e-6);
        if(device.releaseStage==='complete')device.releaseStage='expanding';
    }
    if(device.type==='limb'&&device.limbReleased) {
        device.deployment=.95+.05*device.limbOpening;
        device.releaseStage=device.limbOpening===1?'complete':'expanding';
    }
    if(device.parts?.some(part=>part.scaffoldRings?.some(ring=>(ring.opening??0)<1))) {
        device.deployment=Math.min(device.deployment,1-1e-6);if(device.releaseStage==='complete')device.releaseStage='expanding';
    }
    return progress()!==before;
}

// One committed opening coordinate per sewn ring. The whole ring must leave
// the cover before it expands; stopping the cover does not stop a freed ring.
function advanceRingOpenings(device,dt,previousEdge) {
    const edge=graftCoverWithdrawal(device)-device.coverLead;
    for(const part of device.parts??[])for(const ring of part.scaffoldRings??[]) {
        const end=part.releaseOffset+(ring===part.scaffoldRings.at(-1)?part.path.length:ring.center+ring.maxHeight/2);
        if(edge<end-1e-8&&!device.limbReleased) {ring.freeTime=0;ring.opening=0;continue;}
        const freeDt=previousEdge<end&&edge>previousEdge?Math.max(0,dt-(end-previousEdge)/(SHEATH_SPEED_MM_S*graftScale(device))):dt;
        ring.freeTime=(ring.freeTime??0)+freeDt;
        // Limit cloth speed in simulation time. An unscaled 45/s spring moved
        // the bifurcation several millimetres in a single physics step.
        if(ring.openingRate===undefined) {
            let travel=0;
            if(part.target&&part.folded)for(let row=0;row<part.rows;row++) {
                const s=part.path.coordinates[row];
                if(s<ring.center-ring.maxHeight/2-2||s>ring.center+ring.maxHeight/2+2)continue;
                const p=part.folded[row];
                for(let j=0;j<part.sides;j++) {
                    const k=(row*part.sides+j)*3;
                    travel=Math.max(travel,Math.hypot(part.target[k]-p.x,part.target[k+1]-p.y,part.target[k+2]-p.z)+(part.packedLayout?.radius??0)+Math.abs(part.packedLayout?.lateral??0));
                }
            }
            ring.openingRate=Math.min(45,35*Math.E*graftScale(device)/Math.max(1e-6,travel));
        }
        const t=ring.openingRate*ring.freeTime;
        ring.opening=t>=8?1:1-(1+t)*Math.exp(-t);
    }
}
export function sewnRingExposure(part,s) {
    const rings=part.scaffoldRings;
    if(!rings?.length)return null;
    // Ring bands are fixed material coordinates; interpolate only across the
    // unstented fabric between them, never along the metal's own band.
    for(let i=0;i<rings.length;i++) {
        const ring=rings[i],end=ring.center+ring.maxHeight/2;
        if(s<=end||i===rings.length-1)return ring.opening??0;
        const next=rings[i+1],start=next.center-next.maxHeight/2;
        if(s<start)return (ring.opening??0)*(1-smooth((s-end)/(start-end)))+(next.opening??0)*smooth((s-end)/(start-end));
    }
}
export function rowExposure(device,distance,offset=0) {
    const edge=device.type==='limb'&&device.limbReleased?Math.max(device.releaseLength,graftCoverWithdrawal(device)-device.coverLead):graftCoverWithdrawal(device)-device.coverLead;
    const pitch=STENT_RING_PITCH_MM*graftScale(device);
    const envelope=s=>smooth((edge-s)/(RELEASE_TRANSITION_MM*graftScale(device)));
    // Shared ring opening coordinates, with smooth interpolation between rings.
    // The local envelope keeps every covered section inside the delivery sleeve.
    // This is a reduced geometric release model, not a nitinol constitutive law.
    const ring=Math.floor((distance-offset)/pitch);
    const a=offset+ring*pitch,b=a+pitch;
    const t=smooth((distance-a)/pitch);
    return Math.min(envelope(distance),envelope(a)*(1-t)+envelope(b)*t);
}

// The short branch is packed alongside the ipsilateral limb. Its distal ring
// is retained until the entire gate clears the sleeve, then it springs open in
// committed simulation time, independently of further sleeve movement.
function advanceGateOpening(device,dt,previousEdge) {
    if(device.type!=='body')return;
    advanceBranchOpening(device,device.parts[2],'gate',dt,previousEdge);
}
function advanceBranchOpening(device,part,prefix,dt,previousEdge) {
    if(!part?.path)return;
    const end=part.releaseOffset+part.path.length;
    const edge=graftCoverWithdrawal(device)-device.coverLead;
    if(edge<end-1e-8) {
        device[`${prefix}FreeTime`]=0;device[`${prefix}Opening`]=0;return;
    }
    const freeDt=previousEdge<end&&edge>previousEdge?dt*Math.max(0,edge-end)/(edge-previousEdge):dt;
    device[`${prefix}FreeTime`]=(device[`${prefix}FreeTime`]??0)+freeDt;
    const t=45*device[`${prefix}FreeTime`];
    device[`${prefix}Opening`]=t>=8?1:1-(1+t)*Math.exp(-t);
}
export function partExposure(device,part,s) {
    const sewn=sewnRingExposure(part,s);
    if(sewn!==null)return sewn;
    if(device.type==='limb'&&device.limbReleased)return Math.max(device.limbOpening,rowExposure(device,part.releaseOffset+s,part.releaseOffset));
    if(device.type==='body'&&part===device.parts[1])return Math.max(device.ipsiOpening??0,rowExposure(device,part.releaseOffset+s,part.releaseOffset));
    if(device.type!=='body'||part!==device.parts[2])return rowExposure(device,part.releaseOffset+s,part.releaseOffset);
    const edge=graftCoverWithdrawal(device)-device.coverLead;
    // The sewn root follows the trunk, while the still-captured free end stays
    // crimped. Re-covering the free end recaptures it before detachment.
    const root=rowExposure(device,part.releaseOffset+s,part.releaseOffset)*Math.max(0,1-s/6);
    if(edge<part.releaseOffset+s)return 0;
    return Math.max(root,device.gateOpening);
}
export function graftFaceExposed(device,face) {
    if(face.gate&&face.bindings)return face.bindings.every(binding=>binding.indices.every(index=>binding.part.exposure[Math.floor(index/binding.part.sides)]>0));
    if(face.gate)return device.gateOpening===1;
    if(face.ipsilateral) {
        if(device.ipsiOpening===1)return true;
        // Actual, exposed cloth participates immediately. Folded rows under
        // the delivery cover do not become an artificial wall for the shaft.
        if(face.bindings?.every(binding=>binding.indices.every(index=>binding.part.exposure[Math.floor(index/binding.part.sides)]>0)))return true;
    }
    return face.distance<=fullyOpenDistance(device);
}
