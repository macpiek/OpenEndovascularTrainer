import assert from 'node:assert/strict';
import {BufferGeometry,Float32BufferAttribute} from 'three';
import {MeshBVH} from 'three-mesh-bvh';
import {createStentGraftContacts} from '../../src/devices/stentGraftContacts.js';
import {restoreSharedAxisWallFriction} from '../../src/physics/kirchhoffSharedAxisWallFriction.js';
import { createSharedAxisContacts } from '../../src/physics/kirchhoffSharedAxisContacts.js';
import { createSharedAxisNative, restoreSharedAxisNative, extendSharedAxisNativeRows } from '../../src/physics/kirchhoffSharedAxisNative.js';
import { createSharedAxisVesselWitness } from '../../src/physics/kirchhoffSharedAxisVesselWitnesses.js';

export {captureSharedAxisReplay} from '../../src/physics/kirchhoffSharedAxisReplay.js';

export function restoreSharedAxisReplay(fixture,field) {
    assert.equal(fixture.version,1);
    const contacts=createSharedAxisContacts({sheath:fixture.sheath,contactField:field,localCoordinates:!!fixture.origin,retainDiscoveryCertificates:fixture.retainDiscoveryCertificates===true,continuousDiscoverySign:fixture.continuousDiscoverySign===true,continuousSegmentContacts:fixture.continuousSegmentContacts??false,certifiedDiscoverySamples:fixture.certifiedDiscoverySamples===true});
    let graftSampler,graftGeometry;
    if(fixture.graftContactSurface) {
        const graft=fixture.graftContactSurface;
        graftGeometry=new BufferGeometry();graftGeometry.setAttribute('position',new Float32BufferAttribute(graft.positions,3));
        if(graft.indices)graftGeometry.setIndex(graft.indices);
        graftGeometry.boundsTree=new MeshBVH(graftGeometry);graftGeometry.computeBoundingBox();
        graftSampler=createStentGraftContacts({geometry:graftGeometry,bounds:graftGeometry.boundingBox,revision:graft.revision,lumenSections:graft.lumenSections??[]},
            {coordinates:fixture.coordinates,positions:fixture.positions,origin:fixture.origin??[0,0,0]});
        if(graft.hasBaseRows)contacts.wallSamples.push(graftSampler);
    }
    const s=createSharedAxisNative({...contacts,
        fractionalTipThreshold:fixture.fractionalTipThreshold??0,rebaseNearTips:fixture.rebaseNearTips??false,spacing:fixture.spacing,tools:fixture.tools,maxBendAngle:fixture.maxBendAngle??Infinity,minimumEdgeLength:fixture.minimumEdgeLength??0,
        spatialKnots:fixture.coordinates,adaptiveMesh:fixture.adaptiveMesh});
    if(fixture.discoveryState)s.wallSamples.find(sample=>sample.restoreDiscoveryState)?.restoreDiscoveryState(fixture.discoveryState);
    if(fixture.insideContinuation)s.wallSamples.find(sample=>sample.insideProofs)?.insideProofs.restore(fixture.insideContinuation);
    if(fixture.origin)s.origin=fixture.origin.slice();
    assert.deepEqual(s.coordinates,fixture.coordinates,'Replay topology changed');
    assert.deepEqual(s.definitions.map(({evaluate,...d})=>d),fixture.definitions.slice(0,s.definitions.length),'Replay base rows changed');
    extendSharedAxisNativeRows(s,fixture.definitions.slice(s.definitions.length).map(d=>createSharedAxisVesselWitness(field,d)));
    assert.equal(s.multipliers.length,fixture.multipliers.length);
    restoreSharedAxisNative(s,fixture);s.loads.set(fixture.loads);s.fixed.set(fixture.fixed);
    if(fixture.velocities){s.velocities=structuredClone(fixture.velocities);s.angularVelocities=structuredClone(fixture.angularVelocities);}
    if(fixture.dynamicStep)s.dynamicStep={...structuredClone(fixture.dynamicStep),masses:Float64Array.from(fixture.dynamicStep.masses)};
    if(fixture.wallFriction)restoreSharedAxisWallFriction(s,fixture.wallFriction);
    if(fixture.acceptedWallGaps)s.acceptedWallGaps=new Map(fixture.acceptedWallGaps);
    if(fixture.acceptedSolves!==undefined)s.acceptedSolves=fixture.acceptedSolves;
    if(graftSampler&&!fixture.graftContactSurface.hasBaseRows)s.wallSamples.push(graftSampler);
    if(graftGeometry)s.graftReplayGeometry=graftGeometry;
    if(graftGeometry){
        s.graftRevision=fixture.graftContactSurface.recovery?fixture.graftContactSurface.revision:undefined;
        s.graftRecovery=fixture.graftContactSurface.recovery;
        const current=createStentGraftContacts(graftSampler.surface,s);
        s.wallSamples=s.wallSamples.map(sample=>sample.graftSurface?current:sample);
        s.graftRevision=fixture.graftContactSurface.revision;s.graftRecovery=current.recovery;
    }
    return s;
}
