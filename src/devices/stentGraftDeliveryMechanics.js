import {advanceRelease,deliveryNoseState} from './stentGraftDeployment.js';
import {deliveryRadiusMm} from './stentGraftModels.js';
import {defineKirchhoffMaterialProfile} from '../physics/kirchhoffMaterialProfile.js';

export const STENT_GRAFT_DELIVERY_TYPE='stentgraft-delivery';
// Simulator rigidity units, not measured manufacturer material properties.
export const DELIVERY_COVER_EI=8_000_000,DELIVERY_CORE_EI=400_000;
export function deliveryMaterialProfile(exposedLength=0) {
    const rigidity=s=>{
        if(exposedLength<=0)return DELIVERY_COVER_EI;
        const t=Math.max(0,Math.min(1,(s-exposedLength)/3));
        return DELIVERY_CORE_EI+(DELIVERY_COVER_EI-DELIVERY_CORE_EI)*t*t*(3-2*t);
    };
    return defineKirchhoffMaterialProfile({id:STENT_GRAFT_DELIVERY_TYPE,sampleEI1:rigidity,sampleGJ:s=>rigidity(s)/1.4});
}

/** The rod ends at the retractable core base, never at the old implant
 * coordinate once the nose has been pulled back. The flexible cone beyond
 * this base remains a separate rendered component. */
export function deliveryMechanicalExtent(device) {
    const position=Math.max(0,device.position??0),nose=deliveryNoseState({...device,position});
    const insertion=Math.max(0,Math.min(position,nose.position));
    return {insertion,exposedLength:Math.max(0,insertion-nose.sheathEdge)};
}

/** Prepare both feed and release on a private scalar copy. Only the matching
 * committed step publishes the handle position and advances release, so a
 * rejected nose/cover movement cannot leave a different physical pose behind. */
export function prepareDeliveryMotion(device,catheter,dt,advance,wireInserted,release=null) {
    if(!(dt>0))return device.position;
    let command=Math.max(-1,Math.min(1,Number(advance)||0));
    if(!command)command=Math.max(-1,Math.min(1,(device.target-device.position)/(25*dt)));
    if(command>0)command=Math.min(command,Math.max(0,wireInserted-12-device.position)/(25*dt));
    const position=Math.max(0,Math.min(catheter.maxLength??Infinity,device.position+command*25*dt));
    // Ring release state is mutable. A rejected solver trial must not advance
    // the real implant's springs through this shallow device copy.
    const trial={...device,position};
    if(trial.phase==='deploying')trial.parts=device.parts?.map(part=>({...part,
        scaffoldRings:part.scaffoldRings?.map(ring=>({...ring}))}));
    if(trial.phase==='deploying'||trial.phase==='deployed')advanceRelease(trial,dt,release);
    const extent=deliveryMechanicalExtent(trial);
    catheter.advance((extent.insertion-catheter.progress)/(25*dt),dt,wireInserted,25);
    configureDeliveryCatheter(trial,catheter);
    return position;
}

/** Refresh per access before reading solver inputs, including device exchanges. */
export function configureDeliveryCatheter(device,catheter) {
    catheter.setType(STENT_GRAFT_DELIVERY_TYPE);
    catheter.setStiffnessScales({shaftStiffnessScale:1,tipStiffnessScale:1});
    catheter.deliveryRadiusMm=deliveryRadiusMm(device);
    catheter.deliveryExposureMm=Number.isFinite(device.position)?deliveryMechanicalExtent(device).exposedLength:
        Math.max(0,(device.sheathWithdrawal??0)-(device.coverLead??0));
}
