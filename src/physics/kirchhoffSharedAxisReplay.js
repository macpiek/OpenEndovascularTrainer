import {captureSharedAxisNative} from './kirchhoffSharedAxisNative.js';
import {captureSharedAxisWallFriction} from './kirchhoffSharedAxisWallFriction.js';

/** Fixture data only: native profiles are rebuilt, and every stateful pose,
 * frame, reaction and external load is restored as a double precision number. */
export function captureSharedAxisReplay(s,sheath) {
    const pose=captureSharedAxisNative(s);
    return structuredClone({version:1,sheath,...(s.rebaseNearTips?{rebaseNearTips:true}:{}),...(s.wallFrictionHistory||s.wallFrictionStep?{wallFriction:captureSharedAxisWallFriction(s)}:{}),spacing:s.spacing,...(s.acceptedSolves?{acceptedSolves:s.acceptedSolves}:{}),
        ...(s.acceptedWallGaps?{acceptedWallGaps:[...s.acceptedWallGaps]}:{}),...(Number.isFinite(s.maxBendAngle)?{maxBendAngle:s.maxBendAngle}:{}),...(s.origin?.some(v=>v!==0)?{origin:s.origin}:{}),...(s.fractionalTipThreshold>0?{fractionalTipThreshold:s.fractionalTipThreshold}:{}),...(s.minimumEdgeLength?{minimumEdgeLength:s.minimumEdgeLength}:{}),tools:s.materials.map(t=>t.spec),coordinates:s.coordinates.slice(),
        definitions:s.definitions.map(({evaluate,...d})=>d),positions:pose.positions,
        frames:pose.frames.map(f=>f.map(q=>Array.from(q))),multipliers:Array.from(pose.multipliers),
        loads:Array.from(s.loads),fixed:Array.from(s.fixed),
        ...(s.velocities?{velocities:structuredClone(s.velocities),angularVelocities:structuredClone(s.angularVelocities)}:{}),
        ...(s.dynamicStep?{dynamicStep:{...structuredClone(s.dynamicStep),masses:Array.from(s.dynamicStep.masses)}}:{})});
}

