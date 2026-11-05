import {defineKirchhoffMaterialProfile,kirchhoffMaterialProfile} from '../physics/kirchhoffMaterialProfile.js';
import {RELIANT,balloonFreeRadiusAt,validateBalloonInflation} from './balloonProfile.js';

// Force/mm units of the existing rod solver, not clinical pressure calibration.
// Membrane and vessel radial springs act in series, under the same reaction.
export const BALLOON_CONTACT=Object.freeze({membraneCompliance:5e-4,wallCompliance:3e-5,maxWallDisplacement:1.5});
const profiles=new Map();
export function balloonMaterialProfile(volume=0,shaftScale=1,tipScale=1) {
    validateBalloonInflation(volume);
    if(!volume)return kirchhoffMaterialProfile(RELIANT.type);
    const key=`${volume}/${shaftScale}/${tipScale}`;
    if(profiles.has(key))return profiles.get(key);
    const base=kirchhoffMaterialProfile(RELIANT.type);
    // User shaft/tip sliders scale the catheter, not inflation pressure.
    // Compensate their existing constitutive multiplier for membrane tension.
    const rigidity=s=>{
        const t=Math.max(0,Math.min(1,(s-15)/8)),blend=t*t*(3-2*t);
        const scale=tipScale*Math.pow(shaftScale/tipScale,blend);
        return base.sample(s).EI1+20_000*(balloonFreeRadiusAt(s,volume)**2-RELIANT.shaftRadius**2)/scale;
    };
    const profile=defineKirchhoffMaterialProfile({id:RELIANT.type,naturalTipLengthMm:15,
        sampleEI1:rigidity,sampleGJ:s=>rigidity(s)/1.4});
    if(profiles.size>=8)profiles.delete(profiles.keys().next().value);
    profiles.set(key,profile);return profile;
}

/** Volume feedback adjusts the free membrane size/pressure surrogate. This is
 * prepared before equilibrium, and the input checkpoint rolls it back with the
 * rods on rejection. Measured contained volume is from the last accepted pose. */
export function prepareBalloonInflation(catheter,target,dt) {
    if(!Number.isFinite(target)||!Number.isFinite(dt)||dt<0)throw new RangeError('Invalid inflation command');
    if(catheter.type!==RELIANT.type){catheter.balloonDriveMl=0;return;}
    const current=catheter.balloonDriveMl??0,actual=catheter.balloonVolumeMl??0;
    // A confined compliant balloon cannot accept arbitrary fluid. Bound the
    // free-envelope overdrive (a reduced pressure/strain limit), and relieve
    // it gradually if a changing lumen squeezes the balloon. Do not keep
    // pumping toward 60 ml merely because the requested volume cannot fit.
    const limit=actual+Math.max(.35,.35*actual);
    const desired=Math.max(0,Math.min(60,limit,current+Math.max(0,Math.min(60,target))-actual));
    const delta=target<=0?-12*dt:Math.max(-12*dt,Math.min(6*dt,desired-current));
    catheter.balloonDriveMl=Math.max(0,Math.min(60,current+delta));
}
