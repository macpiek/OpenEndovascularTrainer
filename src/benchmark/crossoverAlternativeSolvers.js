import {createSharedAxisContacts} from '../physics/kirchhoffSharedAxisContacts.js';
import {createSharedAxisNative,feedSharedAxisNative,rotateSharedAxisNative} from '../physics/kirchhoffSharedAxisNative.js';
import {iterateSharedAxisXpbd} from '../physics/sharedAxisXpbdExperimental.js';
import {iterateSharedAxisProjective} from '../physics/kirchhoffSharedAxisProjective.js';
import {captureBloodFlow} from '../physics/bloodFlowCoupling.js';
import {sharedAxisGeometryIdentity} from '../physics/kirchhoffSharedAxisGeometryIdentity.js';
import {validateCrossoverRecording,crossoverShapeDifference} from './crossoverScenarioRecording.js';
import {GUIDEWIRE_RADIUS_MM,PIGTAIL_CATHETER_RADIUS_MM,INTRODUCER_SHEATH_INNER_RADIUS_MM} from '../toolDimensions.js';
import {DEFAULT_TOOL_PROFILES} from '../physics/endovascularPhysicsWorld.js';
import {catheterNodeMass} from '../physics/catheterDiscretization.js';
import {createCrossoverRouteCheck,summarizeCrossoverRoute} from './crossoverFunctionalRoute.js';
import {captureSharedAxisReplay} from '../physics/kirchhoffSharedAxisReplay.js';
import {adaptiveMeshOptions} from '../physics/kirchhoffSharedAxisAdaptiveMesh.js';

const angleDifference=(a,b)=>Math.atan2(Math.sin(a-b),Math.cos(a-b));

/** Screening harness: original input targets from EMPTY tools; never restores
 * reference shapes into the evolving candidate. CPU capacity is not app FPS.
 * Geometry gates are unchanged; XPBD/PD cannot claim Newton force certificates.
 * Yielded rows include failures and are not necessarily accepted steps.
 */
export function* compareCrossoverAlternative({record,field,flow,solver='xpbd',options={},mesh=null,onPose=()=>{}}) {
    validateCrossoverRecording(record);
    if(!record.fullManeuverRecorded)throw new Error('Require the complete recorded maneuver');
    if(!['xpbd','projective'].includes(solver))throw new Error('Unknown alternative solver');
    const adaptiveMesh=adaptiveMeshOptions(mesh);
    if(sharedAxisGeometryIdentity(field.fallbackGeometry)!==record.configuration.geometry)throw new Error('Recording mesh mismatch');
    if(JSON.stringify(flow.hemodynamics)!==JSON.stringify(record.configuration.flow.parameters))throw new Error('Flow settings mismatch');
    const access=record.configuration.accesses.find(a=>a.id==='right');
    const routeCheck=createCrossoverRouteCheck(flow,{accessSide:access.id});
    const sheath={start:access.sheath.start,end:access.sheath.end,innerRadius:INTRODUCER_SHEATH_INNER_RADIUS_MM,proximalExtension:40};
    const tools=[{id:'wire',type:'glidewire',mass:DEFAULT_TOOL_PROFILES.guidewire.mass,radius:GUIDEWIRE_RADIUS_MM,wallStaticFriction:access.friction[0],wallKineticFriction:access.friction[1]},
        {id:'catheter',type:'pigtail',mass:catheterNodeMass(DEFAULT_TOOL_PROFILES.catheter.mass),radius:PIGTAIL_CATHETER_RADIUS_MM,wallStaticFriction:.002,wallKineticFriction:.002}]
        .map((t,i)=>({...t,insertion:0,rotation:0,shaftStiffness:access.stiffness[2*i],tipStiffness:access.stiffness[2*i+1]}));
    const contacts=createSharedAxisContacts({sheath,contactField:field,localCoordinates:true,retainDiscoveryCertificates:true,contactCandidateMargin:.01});
    let state=createSharedAxisNative({...contacts,maxBendAngle:Math.PI/4,tools}),rotations={wire:0,catheter:0};
    const solve=solver==='xpbd'?iterateSharedAxisXpbd:iterateSharedAxisProjective;
    for(let index=0;index<record.steps.length;index++) {
        const reference=record.steps[index],started=performance.now(),attempts=[],projectionFailures=[];
        const bloodFlow=record.configuration.flow.enabled?captureBloodFlow(state,flow,contacts.length,
            Math.max(reference.tools.wire,reference.tools.catheter)):null;
        let accepted=null,last;
        // Same feed/rotation subdivisions as the application. Every retry
        // starts from the accepted pose and private collision discovery.
        for(const subdivisions of [1,2,4,8]) {
            let current={...state,wallSamples:state.wallSamples.map(s=>s.forkDiscovery?.({candidateMargin:.01})??s)},currentRotations=rotations;
            let failed=false;
            for(let part=1;part<=subdivisions;part++) {
                const feeds=Object.fromEntries(current.materials.map(m=>[m.spec.id,state.materials.find(x=>x.spec.id===m.spec.id).spec.insertion+
                    part/subdivisions*(reference.tools[m.spec.id]-state.materials.find(x=>x.spec.id===m.spec.id).spec.insertion)]));
                const nextRotations=Object.fromEntries(tools.map(t=>[t.id,rotations[t.id]+part/subdivisions*angleDifference(reference.tools[t.id+'Rotation'],rotations[t.id])]));
                const refinePrefix=adaptiveMesh?.finePrefixLength;
                const activeMesh=refinePrefix!==undefined&&Math.max(...Object.values(feeds))<=refinePrefix+adaptiveMesh.tipLength+5?null:adaptiveMesh;
                const candidate=feedSharedAxisNative({...current,adaptiveMesh:activeMesh},feeds,{contactCandidateMargin:.01,omitInactiveSampleRows:true});
                for(const t of tools)rotateSharedAxisNative(candidate,t.id,angleDifference(nextRotations[t.id],currentRotations[t.id]));
                const feedById=Object.fromEntries(current.materials.map(m=>[m.spec.id,feeds[m.spec.id]-m.spec.insertion]));
                const iterator=solve(candidate,record.dt/subdivisions,{[solver==='xpbd'?'xpbd':'projective']:options,bloodFlow,feedById,
                    ...(solver==='projective'&&options.profile?{observeProjectionFailure:(state,result)=>{
                        const {converged,failure,residual,factorizations,activeSetAttempts,negativeReactionRecoveries}=result;
                        projectionFailures.push({subdivisions,part,linear:{converged,failure,residual,factorizations,activeSetAttempts,negativeReactionRecoveries},fixture:captureSharedAxisReplay(state,sheath)});
                    }}:{})});
                let n;do{n=iterator.next();}while(!n.done);last=n.value;
                attempts.push({subdivisions,part,status:last.status,error:last.error,iterations:last.iterations,ms:last.ms,timings:last.timings,
                    maxLengthError:last.xpbd?.maxLengthError??last.pd?.maxLengthError,maxShear:last.xpbd?.maxShear??last.pd?.maxShear,
                    penetrationMm:last.quality?.maxPenetration,...(last.chainIterations?{chainIterations:last.chainIterations}:{})});
                if(!last.converged){failed=true;break;}
                current=candidate;currentRotations=nextRotations;
            }
            if(!failed){accepted={state:current,rotations:currentRotations,subdivisions};break;}
        }
        const cpuMs=performance.now()-started;
        const shape=accepted?{coordinates:accepted.state.coordinates,positions:accepted.state.positions.flatMap(p=>p.map((v,k)=>v+accepted.state.origin[k]))}:null;
        const shapeDifferenceMm=shape?crossoverShapeDifference(reference.shape,shape):null;
        const row={step:index+1,stage:reference.stage,moving:reference.moving,tools:reference.tools,cpuMs,
            accepted:!!accepted,shape,shapeDifferenceMm,shapeWithinLimit:shape!==null&&shapeDifferenceMm<=1,
            status:last.status,quality:last.quality,xpbd:last.xpbd,pd:last.pd,attempts,...(projectionFailures.length?{projectionFailures}:{})};
        const checkpoint=record.checkpoints.find(c=>c.afterStep===index+1);
        if(shape&&checkpoint)row.routeCheckpoint=routeCheck.checkpoint(checkpoint.stage,shape,reference.tools);
        if(accepted){state=accepted.state;rotations=accepted.rotations;flow.update(record.dt);onPose(state,reference);}
        yield row;
        if(!accepted)return;
    }
}

export function summarizeAlternative(rows,targetSteps) {
    const accepted=rows.filter(r=>r.accepted),moving=accepted.filter(r=>r.moving),cost=moving.map(r=>r.cpuMs).sort((a,b)=>a-b);
    const sum=moving.reduce((v,r)=>v+r.cpuMs,0),maxShape=Math.max(0,...accepted.map(r=>r.shapeDifferenceMm));
    return {attemptedSteps:rows.length,acceptedSteps:accepted.length,targetSteps,complete:accepted.length===targetSteps,
        movingSteps:moving.length,maximumShapeDifferenceMm:accepted.length?maxShape:null,shapeWithinLimit:accepted.length?accepted.every(r=>r.shapeWithinLimit):null,
        firstShapeLimitStep:accepted.find(r=>!r.shapeWithinLimit)?.step??null,
        movingCpuCapacityHz:sum>0?1000*moving.length/sum:null,movingMedianCpuMs:cost[Math.floor(cost.length/2)]??null,
        movingP95CpuMs:cost[Math.max(0,Math.ceil(cost.length*.95)-1)]??null,
        acceptanceMode:'functional',shapeComparisonRole:'diagnostic',shapeDifferenceLimitMm:null,
        functionalRoute:summarizeCrossoverRoute(accepted.flatMap(r=>r.routeCheckpoint?[r.routeCheckpoint]:[]),{complete:accepted.length===targetSteps}),
        goalAchieved:false,applicationFpsVerified:false,equilibriumCertified:false};
}
