import {measureCompositeFriction} from './kirchhoffCompositeFriction.js';

/** Decide the physical branch only at a converged COMMON mechanical root.
 * Solving Coulomb with muStatic is a static feasibility test. Nonzero slip
 * there requires re-solving the same prepared dt with muKinetic. A numerical
 * failure or an unconverged trial is never evidence of breakaway.
 *
 * No penalty occurs in this decision. In particular, testing Ft-k*slip
 * against the static cone would impose an artificial minimum sliding speed
 * proportional to (muStatic-muKinetic)/k.
 */
export function assessCompositeStaticKineticFriction({traction,slip,normalForce,muStatic,muKinetic,mode,
    wholeStepConverged,slipTolerance,coneTolerance,workTolerance}) {
    if(!['static','kinetic'].includes(mode))throw new RangeError('An explicit physical friction branch is required');
    for(const a of [muStatic,muKinetic])if(a?.length!==2||!Array.from(a).every(v=>Number.isFinite(v)&&v>=0))throw new RangeError('Two finite nonnegative coefficients per friction branch are required');
    if(muStatic.some((v,i)=>v<muKinetic[i]))throw new RangeError('The static cone must contain the kinetic cone');
    const mu=mode==='static'?muStatic:muKinetic;
    const physical=measureCompositeFriction({traction,slip,normalForce,mu,slipTolerance,coneTolerance,workTolerance});
    const equal=muStatic.every((v,i)=>v===muKinetic[i]);
    if(!wholeStepConverged||!physical.converged)return {status:'unconverged',accepted:false,change:false,mode,physical};
    if(equal)return {status:'equal-coefficients',accepted:true,change:false,mode,nextMode:'static',stopCertificate:null,physical};
    const length=Math.hypot(...slip.filter((_,i)=>muStatic[i]>0)),axes=mu.map(v=>v*normalForce);
    const coneCoordinate=Math.hypot(...traction.map((v,i)=>axes[i]>0?v/axes[i]:v===0?0:Infinity));
    const activeSlip=Math.hypot(...slip.filter((_,i)=>mu[i]>0));
    // Disabled tangent axes are free and cannot be certified as a full stop.
    // When a static axis is enabled but its kinetic coefficient is zero,
    // motion on that axis remains kinetic until its actual slip is zero.
    const exactStop=length===0;
    const interiorStop=normalForce>0&&muStatic.every((v,i)=>v===0||mu[i]>0)&&coneCoordinate<1-coneTolerance&&length<=slipTolerance;
    if(exactStop||interiorStop)return {status:'stopped',accepted:true,change:false,mode,nextMode:'static',
        stopCertificate:exactStop?'exact-zero-slip':'strict-cone-interior-and-physical-residual',physical};
    if(normalForce===0)return {status:'unloaded',accepted:true,change:false,mode,nextMode:length>slipTolerance?'kinetic':null,stopCertificate:null,physical};
    if(mode==='static'&&activeSlip>slipTolerance)return {status:'breakaway',accepted:false,change:true,mode,nextMode:'kinetic',stopCertificate:null,physical};
    if(mode==='kinetic'&&(length>slipTolerance||mu.every(v=>v===0)))return {status:'sliding',accepted:true,change:false,mode,nextMode:'kinetic',stopCertificate:null,physical};
    return {status:'ambiguous',accepted:false,change:false,mode,nextMode:null,stopCertificate:null,physical};
}
