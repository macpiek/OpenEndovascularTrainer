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

/** Prepared rod input only. The public delivery position is published after
 * the complete coupled step commits; rollback restores the catheter proxy. */
export function prepareDeliveryMotion(device,catheter,dt,advance,wireInserted) {
    let command=Math.max(-1,Math.min(1,Number(advance)||0));
    if(!command)command=Math.max(-1,Math.min(1,(device.target-catheter.progress)/Math.max(1e-9,25*dt)));
    if(command>0)command=Math.min(command,Math.max(0,wireInserted-12-catheter.progress)/Math.max(1e-9,25*dt));
    catheter.advance(command,dt,wireInserted,25);
}
