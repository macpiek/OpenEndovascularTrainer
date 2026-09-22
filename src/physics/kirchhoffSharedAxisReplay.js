import {captureSharedAxisNative} from './kirchhoffSharedAxisNative.js';
import {captureSharedAxisWallFriction} from './kirchhoffSharedAxisWallFriction.js';

/** Fixture data only: native profiles are rebuilt, and every stateful pose,
 * frame, reaction and external load is restored as a double precision number. */
export function captureSharedAxisReplay(s,sheath,graftSurfaceOverride=null) {
    const pose=captureSharedAxisNative(s);
    const sampler=s.wallSamples?.find(sample=>sample.graftSurface),graft=graftSurfaceOverride??sampler?.surface;
    const graftContactSurface=graft?{revision:graft.revision,hasBaseRows:!!sampler,recovery:graft.revision===s.graftRevision?(s.graftRecovery??null):null,
        lumenSections:graft.lumenSections??[],positions:Array.from(graft.geometry.attributes.position.array),indices:graft.geometry.index?Array.from(graft.geometry.index.array):null}:null;
    return structuredClone({version:1,sheath,...(s.wallSamples?.some(sample=>sample.continuousSegmentContacts)?{continuousSegmentContacts:s.wallSamples.find(sample=>sample.continuousSegmentContacts).continuousSegmentContacts}:{}),...(s.wallSamples?.some(sample=>sample.certifiedDiscoverySamples)?{certifiedDiscoverySamples:true}:{}),...(s.wallSamples?.some(sample=>sample.retainDiscoveryCertificates)?{discoveryState:s.wallSamples.find(sample=>sample.captureDiscoveryState)?.captureDiscoveryState()}:{}),...(s.wallSamples?.some(sample=>sample.continuousDiscoverySign)?{continuousDiscoverySign:true,insideContinuation:s.wallSamples.find(sample=>sample.insideProofs)?.insideProofs.capture()}:{}),...(s.wallSamples?.some(sample=>sample.retainDiscoveryCertificates)?{retainDiscoveryCertificates:true}:{}),...(s.adaptiveMesh?{adaptiveMesh:s.adaptiveMesh}:{}),...(s.rebaseNearTips?{rebaseNearTips:true}:{}),...(s.wallFrictionHistory||s.wallFrictionStep?{wallFriction:captureSharedAxisWallFriction(s)}:{}),spacing:s.spacing,...(s.acceptedSolves?{acceptedSolves:s.acceptedSolves}:{}),
        ...(s.acceptedWallGaps?{acceptedWallGaps:[...s.acceptedWallGaps]}:{}),...(Number.isFinite(s.maxBendAngle)?{maxBendAngle:s.maxBendAngle}:{}),...(s.origin?.some(v=>v!==0)?{origin:s.origin}:{}),...(s.fractionalTipThreshold>0?{fractionalTipThreshold:s.fractionalTipThreshold}:{}),...(s.minimumEdgeLength?{minimumEdgeLength:s.minimumEdgeLength}:{}),tools:s.materials.map(t=>t.spec),coordinates:s.coordinates.slice(),
        ...(graftContactSurface?{graftContactSurface}:{}),definitions:s.definitions.map(({evaluate,...d})=>d),positions:pose.positions,
        frames:pose.frames.map(f=>f.map(q=>Array.from(q))),multipliers:Array.from(pose.multipliers),
        loads:Array.from(s.loads),fixed:Array.from(s.fixed),
        ...(s.velocities?{velocities:structuredClone(s.velocities),angularVelocities:structuredClone(s.angularVelocities)}:{}),
        ...(s.dynamicStep?{dynamicStep:{...structuredClone(s.dynamicStep),masses:Array.from(s.dynamicStep.masses)}}:{})});
}
