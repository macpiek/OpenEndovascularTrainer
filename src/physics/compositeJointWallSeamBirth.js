import {createCompositeJointWallRows} from './kirchhoffCompositeJointWallRows.js';
import {createCompositeJointWallFrictionRows} from './kirchhoffCompositeJointWallFrictionRows.js';
import {createCompositeDiscreteWallPointBranches} from './compositeDiscreteWallPointBranches.js';

const copy=structuredClone;
// Native material profiles carry one callable materialAt reference. Own every
// numeric/history buffer, while preserving this existing constitutive callback.
function copyState(state){
    if(!Array.isArray(state?.tools))fail('A physical tool state is required');
    const tools=state.tools.map(({materialAt,...tool})=>{
        if(materialAt!==undefined&&typeof materialAt!=='function')fail('Invalid native materialAt callback');
        return tool;
    });
    const result=copy({...state,tools});
    state.tools.forEach((tool,i)=>{if(Object.hasOwn(tool,'materialAt'))result.tools[i].materialAt=tool.materialAt;});
    return result;
}
function reconstructCandidateGeometry(candidate){
    const n=candidate.layout.nodeCount,points=candidate.positions;
    if(!Array.isArray(points)||points.length!==n||points.some(p=>p?.length!==3||!Array.from(p).every(Number.isFinite)))
        fail('Candidate needs finite canonical common positions');
    if(candidate.relative?.length!==3*candidate.modes.length||!Array.from(candidate.relative).every(Number.isFinite))
        fail('Candidate needs finite canonical relative coordinates');
    const geometry=new Map([...candidate.layout.spins.keys()].map(id=>[id,points.map(p=>Array.from(p))]));
    const relativeTool=geometry.get(candidate.relativeToolId);
    for(const mode of candidate.modes)for(let a=0;a<3;a++)for(let k=0;k<3;k++){
        const value=mode.basis[a][k]*candidate.relative[mode.relativeDofs[a]];
        if(!Number.isFinite(value)||!relativeTool?.[mode.node])fail('Invalid candidate physical mode');
        relativeTool[mode.node][k]+=value;
        if(!Number.isFinite(relativeTool[mode.node][k]))fail('Nonfinite reconstructed candidate geometry');
    }
    candidate.toolPositions=geometry;
}
function fail(message){throw Object.assign(new RangeError(message),{code:'unsupported-wall-seam-birth'});}
function equalValues(a,b,label){
    if(a?.length!==b?.length||!Array.from(a??[]).every((v,i)=>Number.isFinite(v)&&Number.isFinite(b[i])&&Math.abs(v-b[i])<=2048*Number.EPSILON*Math.max(1,Math.abs(v),Math.abs(b[i]))))fail(`Seam birth changes ${label}`);
}
const provenance=view=>{const row=view.raw,raw=row.rawContact;return JSON.stringify([row.source,row.t,'fixed-material-point',raw.capsuleSampleCount,raw.faceIndex??-1,Math.sign(raw.signedDistance),...(view.sdfBranch?[view.sdfBranch]:[])]);};

/** Representation-only birth, with no integration or accepted-step authority.
 * The caller must reprepare any state-identity-bound world proofs before using
 * the returned restart state. Accepted histories and trial multipliers are
 * separate; only the reaction at the private candidate is verified here. Existing histories remain untouched on every failure.
 */
export function migrateCompositeJointWallSeamBirth({state,wall,prepared,dt,tolerances,siteIndex,sdfSeam,consumeQuery,candidate=state,normalForces,tractions}) {
    if(consumeQuery!==undefined&&typeof consumeQuery!=='function')fail('Query budget callback must be a function');
    let queries=0;const chargeQuery=()=>{consumeQuery?.();queries++;};
    if(wall?.contactMode!=='material-points'||wall.pressureDiscretization!=='fixed-material-points'||wall.contactUpdate!=='current-query'||
        wall.mode!=='wall-coulomb'||!['coulomb','coulomb-static-kinetic'].includes(wall.friction?.law)||wall.friction.rateMode!=='backward-euler-grid'||
        wall.friction.slipModel!=='implicit-backward-euler-surface-rate'||wall.friction.finiteStepSlipKnown!==false)
        fail('Seam birth supports only fixed material-point current-query Coulomb backward-euler-grid');
    const twoBranch=wall.friction.law==='coulomb-static-kinetic';
    if(twoBranch&&!wall.friction.incomingRateHistory)fail('Static/kinetic seam birth requires physical incomingRateHistory');
    const hasHistory=state?.wallContactState!==undefined;
    if(hasHistory!==(state?.wallFrictionState!==undefined)||hasHistory&&(!state.wallContactState||!state.wallFrictionState))
        fail('Seam birth requires both histories or neither');
    if(!hasHistory&&(normalForces===undefined||tractions===undefined))fail('Cold-start birth requires explicit trial normal forces and tractions');
    if(!Number.isInteger(siteIndex)||siteIndex<0||siteIndex>=wall.pressureSites?.length||wall.pressureSites[siteIndex].sdfSeam)
        fail('Seam birth needs one existing unsplit pressure site');
    equalValues(candidate?.coordinates,state.coordinates,'candidate material coordinates');
    if(candidate?.layout?.dofCount!==state.layout.dofCount||JSON.stringify(candidate.layout.edgeToolIds)!==JSON.stringify(state.layout.edgeToolIds)||
        JSON.stringify(candidate.modes)!==JSON.stringify(state.modes)||candidate.relativeToolId!==state.relativeToolId)
        fail('Seam birth requires unchanged candidate topology');
    if(candidate.tools?.length!==state.tools.length||candidate.tools.some((t,i)=>t.id!==state.tools[i].id||t.materialAt!==state.tools[i].materialAt))
        fail('Seam birth may not replace native material callbacks');
    const current=copyState(candidate);
    reconstructCandidateGeometry(current);
    function trialArray(value,fallback,label){
        const array=value===undefined?fallback:value;
        if(array?.length!==fallback.length||!Array.from(array).every(Number.isFinite))fail(`Invalid trial ${label}`);
        return Float64Array.from(array);
    }
    const oldState=copyState(state),newState=copyState(state),site=wall.pressureSites[siteIndex],newWall={...wall,pressureSites:copy(wall.pressureSites)};
    newWall.pressureSites[siteIndex].sdfSeam={face:copy(sdfSeam?.face),domainBox:copy(sdfSeam?.domainBox)};
    const normalArgs=(s,w,history)=>({...s,wall:{...w,mode:'wall-normal',friction:'none'},history,tolerances,
        preserveSampleReactions:state.wallContactState?.preserveSampleReactions??true});
    function managers(s,w,history,frictionHistory) {
        const candidate=copyState(current),normal=createCompositeJointWallRows(normalArgs(s,w,history));
        const source={...s};if(frictionHistory===null)delete source.wallFrictionState;else source.wallFrictionState=frictionHistory;
        const friction=createCompositeJointWallFrictionRows({state:source,candidate,normal,wall:w,prepared,dt,tolerances,normalRowOffset:0,frictionRowOffset:normal.rows.length});
        return {normal,friction,candidate};
    }
    function refresh(m){
        const common=new Float64Array(state.layout.dofCount),relative=new Float64Array(state.relative.length),toolPositions=m.candidate.toolPositions;
        m.normal.refresh({toolPositions,commonResidual:common,relativeResidual:relative,consumeQuery:chargeQuery});
        m.friction.prepare({toolPositions});
        m.friction.refresh({toolPositions,commonResidual:common,relativeResidual:relative,order:'full'});
        return {common,relative};
    }
    const old=managers(oldState,wall,oldState.wallContactState??null,oldState.wallFrictionState??null);
    const trialNormal=trialArray(normalForces,hasHistory?state.wallContactState.normalForces:old.normal.normalForces,'normal forces'),
        trialTraction=trialArray(tractions,hasHistory?state.wallFrictionState.tractions:old.friction.tractions,'tractions');
    // Newton trial multipliers may be signed. Preserve them exactly; only
    // the ordinary final contact certificate can accept a physical reaction.
    old.normal.normalForces.set(trialNormal);old.friction.tractions.set(trialTraction);
    const before=refresh(old);
    const original=old.normal.surfaceRecords.find(v=>v.owner===site.owner&&v.edge===site.edge&&v.fraction===site.fraction&&!v.sdfBranch);
    if(!original)fail('Original pressure site is missing');
    const p=current.toolPositions.get(site.owner),proof=createCompositeDiscreteWallPointBranches({edge:site.edge,fraction:site.fraction}).refresh({
        field:wall.field,positions:[p[site.edge],p[site.edge+1]],radius:original.radius,consumeQuery:chargeQuery,face:sdfSeam?.face,domainBox:sdfSeam?.domainBox});
    if(!proof.supported)fail(`Unproved seam birth: ${proof.reason}`);
    const branch=proof.branches[proof.selectedBranch];
    equalValues(original.raw.normal,branch.normal,'normal');equalValues(original.raw.closestPoint,branch.wallPoint,'wall lever');
    equalValues(original.raw.forceColumn,branch.forceColumn,'normal force map');
    const target=managers(newState,newWall,null,null),views=target.normal.surfaceRecords;
    const oldByKey=new Map(old.normal.surfaceRecords.map((v,i)=>[v.key,i]));
    const mapping=views.map(v=>v.owner===site.owner&&v.edge===site.edge&&v.fraction===site.fraction&&v.sdfBranch
        ?v.sdfBranch.branchIndex===proof.selectedBranch?original.index:null:oldByKey.get(v.key));
    if(mapping.some(i=>i===undefined))fail('Unrelated pressure identity changed');
    let history=null,frictionHistory=null;
    if(hasHistory){
    history=copy(state.wallContactState);history.signature=target.normal.signature;
    history.normalForces=Float64Array.from(mapping,i=>i===null?0:state.wallContactState.normalForces[old.normal.surfaceRecords[i].base]);
    history.records=views.map((v,i)=>({...(mapping[i]===null?{provenance:null}:copy(state.wallContactState.records[mapping[i]])),key:v.key,role:v.role,seam:null,representative:null,dependent:null}));
    newState.wallContactState=history;
    frictionHistory=copy(state.wallFrictionState);
    frictionHistory.signature=target.friction.signature;frictionHistory.sampleIds=views.map(v=>v.key);
    frictionHistory.tractions=Float64Array.from(mapping.flatMap(i=>i===null?[0,0]:Array.from(state.wallFrictionState.tractions.slice(2*i,2*i+2))));
    if(twoBranch){
        if(!Array.isArray(frictionHistory.modeHistory)||frictionHistory.modeHistory.length!==old.normal.surfaceRecords.length)
            fail('Static/kinetic seam birth requires accepted mode history');
        frictionHistory.modeHistory=mapping.map((index,i)=>index===null?null:{...copy(frictionHistory.modeHistory[index]),sampleId:views[i].key});
    }
    // Pressure descriptors are part of the constructor-generated signature;
    // copying that exact structure avoids independently guessing its packing.
    frictionHistory.pressureSites=JSON.parse(target.friction.signature).pressureSites;
    }
    const mappedNormal=Float64Array.from(mapping,i=>i===null?0:trialNormal[old.normal.surfaceRecords[i].base]),
        mappedTraction=Float64Array.from(mapping.flatMap(i=>i===null?[0,0]:Array.from(trialTraction.slice(2*i,2*i+2))));
    const final=managers(newState,newWall,history,frictionHistory);
    final.normal.normalForces.set(mappedNormal);final.friction.tractions.set(mappedTraction);
    const after=refresh(final);
    if(hasHistory){
    frictionHistory.provenance=final.normal.surfaceRecords.map(provenance);
    newState.wallFrictionState=frictionHistory;
    // Validate the exact returned histories, including their final provenance.
    const validated=managers(newState,newWall,newState.wallContactState,newState.wallFrictionState);
    validated.normal.normalForces.set(mappedNormal);validated.friction.tractions.set(mappedTraction);refresh(validated);
    }
    for(let i=0;i<mapping.length;i++)if(mapping[i]!==null){
        const oldIndex=mapping[i];
        for(let c=0;c<2;c++){
            const a=old.friction.rows[2*oldIndex+c],b=final.friction.rows[2*i+c];
            equalValues(a.commonDofs,b.commonDofs,'common physical support');equalValues(a.relativeDofs,b.relativeDofs,'relative physical support');
            equalValues(a.forceColumn,b.forceColumn,'tangential force and own torque map');
        }
    }
    equalValues(before.common,after.common,'common mechanical reaction');equalValues(before.relative,after.relative,'relative mechanical reaction');
    return {state:newState,wall:newWall,acceptedHistories:{wallContactState:hasHistory?copy(history):undefined,wallFrictionState:hasHistory?copy(frictionHistory):undefined},
        trial:{candidate:copyState(current),normalForces:mappedNormal,tractions:mappedTraction},diagnostics:{scope:'wall-seam-restart-history-and-trial-transfer-not-a-time-step',reactionConfiguration:'candidate',coldStart:!hasHistory,acceptedGeometryCertified:false,siteIndex,selectedBranch:proof.selectedBranch,
        oldSamples:old.normal.surfaceRecords.length,newSamples:views.length,oldToNew:mapping,physicalReactionPreserved:true,
        requiresWorldProofRepreparation:true,timeAdvanced:false,preparationContactQueries:queries}};
}
