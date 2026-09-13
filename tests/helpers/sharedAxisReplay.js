import assert from 'node:assert/strict';
import {captureSharedAxisWallFriction,restoreSharedAxisWallFriction} from '../../src/physics/kirchhoffSharedAxisWallFriction.js';
import { createSharedAxisContacts } from '../../src/physics/kirchhoffSharedAxisContacts.js';
import { createSharedAxisNative, captureSharedAxisNative, restoreSharedAxisNative, extendSharedAxisNativeRows } from '../../src/physics/kirchhoffSharedAxisNative.js';
import { createSharedAxisVesselWitness } from '../../src/physics/kirchhoffSharedAxisVesselWitnesses.js';

/** Fixture data only: native profiles are rebuilt, and every stateful pose,
 * frame, reaction and external load is restored as a double precision number. */
export function captureSharedAxisReplay(s,sheath) {
    const pose=captureSharedAxisNative(s);
    return {version:1,sheath,...(s.rebaseNearTips?{rebaseNearTips:true}:{}),...(s.wallFrictionHistory||s.wallFrictionStep?{wallFriction:captureSharedAxisWallFriction(s)}:{}),spacing:s.spacing,...(Number.isFinite(s.maxBendAngle)?{maxBendAngle:s.maxBendAngle}:{}),...(s.origin?.some(v=>v!==0)?{origin:s.origin}:{}),...(s.fractionalTipThreshold>0?{fractionalTipThreshold:s.fractionalTipThreshold}:{}),...(s.minimumEdgeLength?{minimumEdgeLength:s.minimumEdgeLength}:{}),tools:s.materials.map(t=>t.spec),coordinates:s.coordinates.slice(),
        definitions:s.definitions.map(({evaluate,...d})=>d),positions:pose.positions,
        frames:pose.frames.map(f=>f.map(q=>Array.from(q))),multipliers:Array.from(pose.multipliers),
        loads:Array.from(s.loads),fixed:Array.from(s.fixed),
        ...(s.velocities?{velocities:structuredClone(s.velocities),angularVelocities:structuredClone(s.angularVelocities)}:{}),
        ...(s.dynamicStep?{dynamicStep:{...structuredClone(s.dynamicStep),masses:Array.from(s.dynamicStep.masses)}}:{})};
}

export function restoreSharedAxisReplay(fixture,field) {
    assert.equal(fixture.version,1);
    const s=createSharedAxisNative({...createSharedAxisContacts({sheath:fixture.sheath,contactField:field,localCoordinates:!!fixture.origin}),
        fractionalTipThreshold:fixture.fractionalTipThreshold??0,rebaseNearTips:fixture.rebaseNearTips??false,spacing:fixture.spacing,tools:fixture.tools,maxBendAngle:fixture.maxBendAngle??Infinity,minimumEdgeLength:fixture.minimumEdgeLength??0});
    if(fixture.origin)s.origin=fixture.origin.slice();
    assert.deepEqual(s.coordinates,fixture.coordinates,'Replay topology changed');
    assert.deepEqual(s.definitions.map(({evaluate,...d})=>d),fixture.definitions.slice(0,s.definitions.length),'Replay base rows changed');
    extendSharedAxisNativeRows(s,fixture.definitions.slice(s.definitions.length).map(d=>createSharedAxisVesselWitness(field,d)));
    assert.equal(s.multipliers.length,fixture.multipliers.length);
    restoreSharedAxisNative(s,fixture);s.loads.set(fixture.loads);s.fixed.set(fixture.fixed);
    if(fixture.velocities){s.velocities=structuredClone(fixture.velocities);s.angularVelocities=structuredClone(fixture.angularVelocities);}
    if(fixture.dynamicStep)s.dynamicStep={...structuredClone(fixture.dynamicStep),masses:Float64Array.from(fixture.dynamicStep.masses)};
    if(fixture.wallFriction)restoreSharedAxisWallFriction(s,fixture.wallFriction);
    return s;
}
