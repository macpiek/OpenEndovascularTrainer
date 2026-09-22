// Distances are mm, time is committed simulation time. Cover motion is
// reversible and independent of tip capture. Detachment is irreversible.
export const SHEATH_SPEED_MM_S=12;
export const SUPRARENAL_LENGTH_MM=12;
export const NOSECONE_RETRACTION_SPEED_MM_S=12;
export function graftAttached(device) {
    return device.phase==='loaded'||device.phase==='deploying'&&(device.type==='body'?device.tipRelease<1:device.deployment<1);
}
// After detachment the cover still moves with the delivery shaft, while the
// implant's material origin stays fixed. Exposure must use their relative pose.
export function graftCoverWithdrawal(device) {
    return (device.sheathWithdrawal??0)+((device.implantPosition??device.position)-device.position);
}
export function deliveryNoseState(device) {
    const lead=device.type==='body'?SUPRARENAL_LENGTH_MM:0;
    const extended=device.position+lead+(device.type==='body'?(device.tipRelease??0)*3:0);
    const sheathEdge=Math.max(0,device.position+lead-(device.sheathWithdrawal??0));
    const limit=Math.max(0,extended-sheathEdge);
    const retraction=Math.max(0,Math.min(limit,device.noseRetraction??0));
    return {position:extended-retraction,sheathEdge,limit,retraction,remaining:limit-retraction};
}
export function initializeRelease(device) {
    const trunk=device.parts[0];
    for(const [i,part] of device.parts.entries())part.releaseOffset=i===0?0:trunk.path.length;
    device.coverLead=device.type==='body'?SUPRARENAL_LENGTH_MM:0;
    device.releaseLength=Math.max(...device.parts.map(p=>p.releaseOffset+p.path.length));
    device.sheathTravel=device.coverLead+device.releaseLength+2;
    device.gateTravel=device.type==='body'
        ?device.coverLead+device.parts[2].releaseOffset+device.parts[2].path.length+2:0;
    device.noseRetraction=0;
    device.sheathWithdrawal=0;device.tipRelease=device.type==='body'?0:1;
    device.releaseStage='sheath';
}
export function advanceRelease(device,dt,control) {
    if(!(dt>0))return false;
    const before=device.sheathWithdrawal+device.tipRelease+(device.noseRetraction??0);
    const cover=typeof control==='object'&&control!==null?control.sheath:
        control==='sheath'?1:control==='resheath'?-1:0;
    const tip=control==='tip'||control?.tip===true;
    device.sheathWithdrawal=Math.max(0,Math.min(device.sheathTravel,
        device.sheathWithdrawal+SHEATH_SPEED_MM_S*dt*Math.max(-1,Math.min(1,Number(cover)||0))));
    if(tip&&device.type==='body')device.tipRelease=Math.min(1,device.tipRelease+dt);
    const nose=deliveryNoseState(device);
    device.noseRetraction=Math.min(nose.limit,nose.retraction+
        ((control==='nose'||control?.nose===true)&&device.tipRelease>=1?NOSECONE_RETRACTION_SPEED_MM_S*dt:0));
    // Moving the delivery cover after detachment cannot recapture the implant.
    if(device.phase==='deployed')return device.sheathWithdrawal+device.tipRelease+device.noseRetraction!==before;
    const exposed=Math.max(0,Math.min(device.sheathTravel,graftCoverWithdrawal(device)));
    device.releaseStage=exposed>=device.sheathTravel
        ?device.tipRelease<1?'tip':'complete':'sheath';
    device.deployment=device.type==='body'
        ?.9*exposed/device.sheathTravel+.1*device.tipRelease
        :exposed/device.sheathTravel;
    if(device.releaseStage==='complete')device.deployment=1;
    return device.sheathWithdrawal+device.tipRelease+device.noseRetraction!==before;
}
export function rowExposure(device,distance) {
    return Math.max(0,Math.min(1,(graftCoverWithdrawal(device)-device.coverLead-distance)/2));
}
