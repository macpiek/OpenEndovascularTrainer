import {introducerDimensions} from './introducerSheath.js';
import {prepareElasticIntroducer} from './introducerMechanics.js';
import {defineKirchhoffMaterialProfile} from '../physics/kirchhoffMaterialProfile.js';

// Generic matching dilator, not a particular manufacturer's calibrated device.
export const DILATOR_TIP_LENGTH_MM=25;
export const DILATOR_LUMEN_RADIUS_MM=.47;
export const DILATOR_TIP_RADIUS_MM=.52;
const EPS=1e-7;

export function dilatorDimensions(sheath) {
    const {innerRadius}=introducerDimensions(sheath.french??6);
    return {radius:innerRadius-.04,innerRadius:DILATOR_LUMEN_RADIUS_MM,
        length:(sheath.length??70)+DILATOR_TIP_LENGTH_MM};
}
export function dilatorRadiusAtDistance(distanceFromTip,shaftRadius) {
    const t=Math.max(0,Math.min(1,distanceFromTip/DILATOR_TIP_LENGTH_MM));
    return DILATOR_TIP_RADIUS_MM+(shaftRadius-DILATOR_TIP_RADIUS_MM)*t*t*(3-2*t);
}
const profiles=new Map();
export function dilatorMaterialProfile(radius) {
    if(!profiles.has(radius)) {
        const reference=.96**4-DILATOR_LUMEN_RADIUS_MM**4;
        const rigidity=s=>4_000_000*(dilatorRadiusAtDistance(s,radius)**4-DILATOR_LUMEN_RADIUS_MM**4)/reference;
        profiles.set(radius,defineKirchhoffMaterialProfile({id:`introducer-dilator-${radius}`,sampleEI1:rigidity,sampleGJ:s=>rigidity(s)/1.4}));
    }
    return profiles.get(radius);
}
export function initializeIntroducerDilator(sheath) {
    sheath.dilatorInsertion??=0;
    sheath.dilatorSeated??=false;
}
export function elasticDilatorTool(sheath,body) {
    const d=dilatorDimensions(sheath);body.radius=d.radius;
    return {id:'dilator',body,type:'introducer-dilator',...d,insertion:sheath.dilatorInsertion??0,
        rotation:0,shaftStiffness:1,tipStiffness:1,
        nodeCoordinates:Array.from({length:body.count},(_,i)=>-40+i/(body.count-1)*((sheath.dilatorInsertion??0)+40))};
}

/** Move the removable core separately. Full seating couples its hub to the
 * sheath; withdrawing it immediately releases that coupling. */
export function prepareDilatorMotion(sheath,delta,{catheterProgress=0,wireProgress=0}={}) {
    if(!Number.isFinite(delta))throw new RangeError('Nieprawidłowy ruch rozszerzacza');
    if(!delta)return false;
    if(delta>0&&catheterProgress>EPS)throw new Error('Wycofaj cewnik przed wsunięciem rozszerzacza.');
    const current=sheath.dilatorInsertion??0,seated=sheath.insertion+DILATOR_TIP_LENGTH_MM;
    const limit=Math.min(seated,Math.max(0,wireProgress-10));
    if(delta>0&&limit<=current+EPS&&!sheath.dilatorSeated)
        throw new Error('Wsuń prowadnik dalej przed rozszerzaczem.');
    const insertion=delta>0?Math.max(current,Math.min(current+delta,limit)):Math.max(0,current+delta);
    if(Math.abs(insertion-current)<EPS)return false;
    sheath.dilatorInsertion=insertion;
    sheath.dilatorSeated=Math.abs(insertion-seated)<EPS&&insertion>0;
    return true;
}

/** Limit independent sheath withdrawal before it would expose more than the
 * tapered tip. A seated pair feeds together, with guidewire support in front. */
export function dilatorLimitedSheathDelta(sheath,delta,wireProgress) {
    if(!(sheath.dilatorInsertion>0))return delta;
    if(sheath.dilatorSeated&&delta>0)
        return Math.max(0,Math.min(delta,wireProgress-10-sheath.dilatorInsertion));
    if(!sheath.dilatorSeated&&delta<0)
        return Math.max(delta,Math.min(0,sheath.dilatorInsertion-DILATOR_TIP_LENGTH_MM-sheath.insertion));
    return delta;
}

export function followSeatedDilator(sheath,previousSheathInsertion) {
    if(sheath.dilatorSeated)sheath.dilatorInsertion+=sheath.insertion-previousSheathInsertion;
}

/** Manual sheath feed from zero brings in the matching core first, then the
 * seated pair. Spend the same feed distance across both stages, without
 * teleporting the 25 mm tip. Exchange over an existing catheter stays coreless. */
export function prepareIntroducerAssembly(sheath,{delta=0,selection=null,catheterProgress=0,
    catheterRadius=5/6,wireProgress=0,hasDilator=true}={}) {
    if(!Number.isFinite(delta))throw new RangeError('Nieprawidłowy ruch koszulki');
    if(selection)return prepareElasticIntroducer(sheath,{selection,catheterProgress,catheterRadius});
    if(!delta)return false;
    const draft={...sheath};
    let remaining=delta,changed=false;
    if(hasDilator&&delta>0&&draft.insertion<=EPS&&catheterProgress<=EPS&&!draft.dilatorSeated) {
        const previous=draft.dilatorInsertion??0;
        changed=prepareDilatorMotion(draft,remaining,{catheterProgress,wireProgress});
        remaining-=draft.dilatorInsertion-previous;
        if(!draft.dilatorSeated)remaining=0;
    }
    const previous=draft.insertion;
    const feed=dilatorLimitedSheathDelta(draft,remaining,wireProgress);
    changed=prepareElasticIntroducer(draft,{delta:feed,catheterProgress,catheterRadius})||changed;
    followSeatedDilator(draft,previous);
    remaining-=draft.insertion-previous;
    // Once the sheath has exited, finish withdrawing its protruding core so
    // the complete set can be exchanged at zero using the same G command.
    if(hasDilator&&remaining<0&&draft.insertion<=EPS&&(draft.dilatorInsertion??0)>0)
        changed=prepareDilatorMotion(draft,remaining,{catheterProgress,wireProgress})||changed;
    if(changed)Object.assign(sheath,draft);
    return changed;
}

export function publishElasticDilator(sheath,body) {
    const view=body?.jointStateView;
    if(!view)return false;
    const path=Array.from(view.coordinates,(s,i)=>({s,x:view.positions[3*i],y:view.positions[3*i+1],z:view.positions[3*i+2]}));
    const old=sheath.dilatorPath;
    if(old?.length===path.length&&path.every((p,i)=>Math.abs(p.s-old[i].s)<1e-8&&
        Math.hypot(p.x-old[i].x,p.y-old[i].y,p.z-old[i].z)<1e-7))return false;
    sheath.dilatorPath=path;return true;
}
