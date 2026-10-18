import {createCapturePotential} from '../devices/stentGraftCapture.js';
import {BufferGeometry,Float32BufferAttribute} from 'three';
import {MeshBVH} from 'three-mesh-bvh';
import {createStentGraftContacts} from '../devices/stentGraftContacts.js';
import {restoreSharedAxisWallFriction} from './kirchhoffSharedAxisWallFriction.js';
import { createSharedAxisContacts } from './kirchhoffSharedAxisContacts.js';
import { createSharedAxisNative, restoreSharedAxisNative, extendSharedAxisNativeRows, assembleSharedAxisNative } from './kirchhoffSharedAxisNative.js';
import { createSharedAxisVesselWitness } from './kirchhoffSharedAxisVesselWitnesses.js';

// Replay must run in Node regression tests and the actual browser benchmark.
// These guards compare serialized topology (numbers, arrays and plain records)
// before restoring any physical state; no Node-only assertion shim is needed.
function sameReplayValue(a,b) {
    if(Object.is(a,b))return true;
    if(!a||!b||typeof a!=='object'||typeof b!=='object'||Array.isArray(a)!==Array.isArray(b))return false;
    const keys=Object.keys(a);
    return keys.length===Object.keys(b).length&&keys.every(k=>Object.hasOwn(b,k)&&sameReplayValue(a[k],b[k]));
}
const assert={equal(a,b,message='Replay value changed') {if(!Object.is(a,b))throw new RangeError(message);},
    deepEqual(a,b,message='Replay topology changed') {if(!sameReplayValue(a,b))throw new RangeError(message);}};

export {captureSharedAxisReplay} from './kirchhoffSharedAxisReplay.js';

// A restored anatomical pose may discover witnesses absent from the recording.
// Direct assembly tests need the same bounded topology restart as the solver;
// never suppress a different error or move the pose to make a test pass.
export function assembleDiscoveredSharedAxis(s,options) {
    for(let restarts=0;;restarts++) {
        try{return assembleSharedAxisNative(s,options);}
        catch(error) {
            if(error.message!=='shared-axis-wall-discovery'||!s.pendingVesselRows?.size||restarts>=64)throw error;
            extendSharedAxisNativeRows(s,[...s.pendingVesselRows.values()]);s.pendingVesselRows.clear();
        }
    }
}

export function restoreSharedAxisReplay(fixture,field) {
    assert.equal(fixture.version,1);
    const contacts=createSharedAxisContacts({sheath:fixture.sheath,contactField:field,localCoordinates:!!fixture.origin,retainDiscoveryCertificates:fixture.retainDiscoveryCertificates===true,continuousDiscoverySign:fixture.continuousDiscoverySign===true,continuousSegmentContacts:fixture.continuousSegmentContacts??false,certifiedDiscoverySamples:fixture.certifiedDiscoverySamples===true,contactCandidateMargin:fixture.contactCandidateMargin});
    let graftSampler,graftGeometry;
    if(fixture.graftContactSurface) {
        const graft=fixture.graftContactSurface;
        const surface=restoreGraftSurface(graft);graftGeometry=surface.geometry;
        graftSampler=createStentGraftContacts(surface,
            {coordinates:fixture.coordinates,positions:fixture.positions,origin:fixture.origin??[0,0,0]});
        if(graft.hasBaseRows)contacts.wallSamples.push(graftSampler);
    }
    if(fixture.graftCaptureBaseRows)contacts.wallSamples.push(createCapturePotential(fixture.graftCapture));
    const s=createSharedAxisNative({...contacts,
        fractionalTipThreshold:fixture.fractionalTipThreshold??0,rebaseNearTips:fixture.rebaseNearTips??false,spacing:fixture.spacing,tools:fixture.tools,maxBendAngle:fixture.maxBendAngle??Infinity,minimumEdgeLength:fixture.minimumEdgeLength??0,
        spatialKnots:fixture.coordinates,adaptiveMesh:fixture.adaptiveMesh,omitInactiveSampleRows:fixture.omitInactiveSampleRows??false});
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
        if(fixture.graftContactSurface.exteriorPatches)graftSampler.contactPatches=fixture.graftContactSurface.exteriorPatches;
        const current=createStentGraftContacts(graftSampler.surface,s);
        s.wallSamples=s.wallSamples.map(sample=>sample.graftSurface?current:sample);
        s.graftRevision=fixture.graftContactSurface.revision;s.graftRecovery=current.recovery;
    }
    if(fixture.graftCapture&&!fixture.graftCaptureBaseRows)s.wallSamples.push(createCapturePotential(fixture.graftCapture));
    return s;
}

function restoreGraftSurface(graft) {
    const geometry=new BufferGeometry();geometry.setAttribute('position',new Float32BufferAttribute(graft.positions,3));
    if(graft.indices)geometry.setIndex(graft.indices);
    geometry.boundsTree=new MeshBVH(geometry);geometry.computeBoundingBox();
    const surface={geometry,bounds:geometry.boundingBox,revision:graft.revision,complianceApplied:graft.complianceApplied??false,
        contactPatches:graft.contactPatches??[],otherContactPatches:graft.otherContactPatches??[],lumenSections:graft.lumenSections??[],ownedBranches:graft.ownedBranches??[]};
    if(Object.hasOwn(graft,'coveredDeliverySurface')) {
        surface.coveredDeliverySurface=graft.coveredDeliverySurface?restoreGraftSurface(graft.coveredDeliverySurface):null;
        if(surface.coveredDeliverySurface)geometry.addEventListener('dispose',()=>surface.coveredDeliverySurface.geometry.dispose());
    }
    return surface;
}
