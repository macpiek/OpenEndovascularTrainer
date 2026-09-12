import {createCompositeJointWallSurfaceWorkspace,evaluateCompositeJointWallSurface} from './kirchhoffCompositeJointWallSurface.js';
import {createCompositeJointSurfacePullbackFactory,pullbackCompositeJointSurface,evaluateCompositeJointSurfaceLoads} from './kirchhoffCompositeJointSurfacePullback.js';
import {createCompositeJointPhysicalColumnPullback,pullbackCompositeJointPhysicalColumns,evaluateCompositeJointPhysicalColumnLoads} from './kirchhoffCompositeJointPhysicalColumnPullback.js';
import {readCompositeJointSurfacePosePath} from './kirchhoffCompositeJointSurfacePoseHistory.js';
import {compositeJointWallOwner,compositeJointWallSurfaces,expandCompositeJointWallOwners} from './kirchhoffCompositeJointWallOwners.js';
import {createCompositeFrictionEquationWorkspace,evaluateCompositeFrictionEquation,measureCompositeFriction} from './kirchhoffCompositeFriction.js';
import {assessCompositeStaticKineticFriction} from './kirchhoffCompositeStaticKineticFriction.js';
import {sampleCompositeNativeRateHistory} from './kirchhoffCompositeNativeRateHistory.js';

const workspaces=new WeakMap();
const failure=message=>{const e=new RangeError(message);e.code='unsupported-joint-wall-friction';return e;};
const finite=(v,name)=>{if(!Number.isFinite(v))throw failure(`${name} must be finite`);return v;};
const positive=(v,name)=>{if(!(finite(v,name)>0))throw failure(`${name} must be positive`);return v;};
const vector=(v,n,name)=>{if(v?.length!==n||!Array.from(v).every(Number.isFinite))throw failure(`${name} needs ${n} finite entries`);return Array.from(v);};
const same=(a,b)=>a?.length===b.length&&b.every((v,i)=>v===a[i]);
const copy=v=>structuredClone(v);
const mapKey=t=>JSON.stringify([t.id,t.edge,t.edgeId]);
const edgeId=v=>{
    if(typeof v==='string')return `string:${v.length}:${v}`;
    if(typeof v==='number'&&Number.isFinite(v))return `number:${Object.is(v,-0)?0:v}`;
    if(typeof v==='bigint')return `bigint:${v}`;
    throw failure('A semantic material segment ID must be a string, finite number or bigint');
};
/** Lossless manager convention for accepted path edge IDs. Material IDs retain
 * their original primitive type separately in the prepared pose history. */
export function getCompositeJointWallFrictionEdgeId(materialSegmentId) {return edgeId(materialSegmentId);}
const trace=f=>f===0?'right':f===1?'left':undefined;
const roundoffEqual=(a,b,scale)=>Math.abs(a-b)<=64*Number.EPSILON*(Math.abs(a)+Math.abs(b)+scale);


// One heavy surface arena and equation buffer, no physical history or commit authority.
export function createCompositeJointWallFrictionWorkspace() {
    const scratch={surface:createCompositeJointWallSurfaceWorkspace(),equation:createCompositeFrictionEquationWorkspace(),
        generation:0,busy:false,refreshes:0,chartBuilds:0,chartHits:0,createPullback:null};
    const handle=Object.freeze({scope:'sequential-joint-wall-friction-scratch',get diagnostics(){return {
        surfaceWorkspaceBuilds:1,equationWorkspaceBuilds:1,managers:scratch.generation,refreshes:scratch.refreshes,chartBuilds:scratch.chartBuilds,chartHits:scratch.chartHits};}});
    workspaces.set(handle,scratch);return handle;
}

/** Two Coulomb rows per ORIGINAL capsule record, or per explicitly declared
 * nodal endpoint pressure site. The latter has one physical Fn/Ft owner from
 * the initial law; it is not a reduction of capsule pressure. No seam, duplicate
 * representative or normal-pressure remap is a valid tangential reduction.
 * The normal manager owns all queries and current provenance. One stationary
 * current wall material witness is used at both ends of dt, never an old query.
 * Source geometry and candidate spins remain owned by their respective callers.
 */
export function createCompositeJointWallFrictionRows({state,candidate,prepared,normal,wall,dt,tolerances,
    normalRowOffset,frictionRowOffset,workspace=createCompositeJointWallFrictionWorkspace()}) {
    const scratch=workspaces.get(workspace);if(!scratch||scratch.busy)throw failure('A free prepared friction workspace is required');
    const generation=++scratch.generation;
    positive(dt,'dt');if(prepared?.dt!==undefined&&prepared.dt!==dt)throw failure('Prepared friction dt differs from the requested step');
    const policy=wall?.friction;
    const twoBranch=policy?.law==='coulomb-static-kinetic';
    if(wall?.mode!=='wall-coulomb'||!['coulomb','coulomb-static-kinetic'].includes(policy?.law)||policy.materialPath!=='linear-affine-maps'||policy.motion!=='stationary-material'||
        !['analytic-plane','sparse-sdf','sparse-sdf-bvh','original-field'].includes(policy.source)||!['projected-own-tangent','projected-own-reference-director'].includes(policy.tangentBasis))
        throw failure('Explicit wall-coulomb law, affine path, stationary material, source and tangent basis are required');
    const contactMode=wall.contactMode??'capsule',materialPoints=contactMode==='material-points',nodal=contactMode==='nodal-endpoints'||materialPoints,slotCount=nodal?1:3;
    const implicitRate=policy.rateMode==='backward-euler-grid';
    if(policy.rateMode!==undefined&&(!implicitRate||policy.slipModel!=='implicit-backward-euler-surface-rate'||policy.finiteStepSlipKnown!==false))
        throw failure('An explicit consistent native implicit surface-rate law is required');
    if(contactMode!=='capsule'&&!nodal)throw failure('Wall Coulomb requires original capsule or explicit nodal-endpoints pressure records');
    const penalty=positive(policy.forcePerLength,'Friction forcePerLength'),normalSignature=normal?.signature,views=normal?.surfaceRecords;
    if(typeof normalSignature!=='string'||!Array.isArray(views)||!views.length||typeof normal.assertCurrentSurface!=='function'||!Array.isArray(normal.rowForceIndices))
        throw failure('The actual normal surface-record owner and private freshness guard are required');
    const {layout}=state,n=layout.nodeCount,r=3*state.modes.length;
    // Each pair describes tangent axes of ONE Coulomb law, never static vs
    // kinetic coefficients. A per-owner declaration preserves different
    // material surfaces without averaging them into a common coefficient.
    const perOwner=policy.muByOwner!==undefined;
    const commonDeclared=twoBranch?policy.muStatic!==undefined||policy.muKinetic!==undefined:policy.mu!==undefined;
    if(perOwner===commonDeclared||twoBranch&&policy.mu!==undefined||!twoBranch&&(policy.muStatic!==undefined||policy.muKinetic!==undefined))throw failure('Declare either common mu or complete muByOwner coefficients');
    const coefficientVector=value=>{const pair=vector(value,2,'Coulomb coefficients');if(pair.some(v=>v<0))throw failure('Coulomb coefficients must be nonnegative');return pair;};
    const branchCoefficients=value=>{
        if(value.mu!==undefined)throw failure('Two-branch coefficients require muStatic and muKinetic');
        const muStatic=coefficientVector(value.muStatic),muKinetic=coefficientVector(value.muKinetic);
        if(muStatic.some((v,i)=>v<muKinetic[i]))throw failure('The static cone must contain the kinetic cone');
        return {muStatic,muKinetic};
    };
    const ownerIds=[...new Set(views.map(v=>v.owner))].sort();
    const coefficientDescription=()=>{
        if(!Array.isArray(policy.muByOwner)||policy.muByOwner.length!==ownerIds.length)throw failure('muByOwner must cover each exposed physical owner once');
        const entries=new Map();
        for(const entry of policy.muByOwner){
            if(!ownerIds.includes(entry?.owner)||entries.has(entry.owner))throw failure('muByOwner needs distinct exposed physical owners');
            entries.set(entry.owner,twoBranch?branchCoefficients(entry):{mu:coefficientVector(entry.mu)});
        }
        return ownerIds.map(owner=>({owner,...entries.get(owner)}));
    };
    const commonCoefficients=perOwner?null:twoBranch?branchCoefficients(policy):{mu:coefficientVector(policy.mu)},ownerCoefficients=perOwner?coefficientDescription():null,
        coefficientsByOwner=perOwner?new Map(ownerCoefficients.map(e=>[e.owner,e])):null;
    if(candidate.relative?.length!==r)throw failure('Candidate relative coordinates must match the full fixed chart');
    if(!(state.toolPositions instanceof Map)||!(candidate.angles instanceof Map)||prepared?.inertiaEdges?.length!==n-1||!(prepared.previousPositions instanceof Map))
        throw failure('Own incoming geometry, live angles and prepared physical inertia are required');
    const tol=Object.fromEntries(['frictionSlip','frictionCone','frictionWork','frictionEquation','linearConstraint'].map(k=>[k,positive(tolerances?.[k],k)]));
    const retained=normal.rowForceIndices.slice(),sampleIds=views.map(s=>s.key),retainedIndex=new Map(retained.map((s,i)=>[s,i]));
    const ownedEdges=expandCompositeJointWallOwners(wall.contactOwners,layout);
    const ownerDescription=()=>wall.contactOwners?.edges?.map((e,i)=>{
        if(e.edge!==i)throw failure('Wall ownership must follow physical edge order');
        const values=compositeJointWallSurfaces(e).map(w=>[i,w.owner,w.radius,edgeId(w.materialSegmentId)]);
        return e.walls!==undefined?{walls:values}:values[0]??null;
    });
    const owners=JSON.stringify(ownerDescription());
    function pressureDescription() {
        if(materialPoints) {
            if(wall.pressureDiscretization!=='fixed-material-points'||!Array.isArray(wall.pressureSites))throw failure('Fixed material pressure sites must be declared');
            const seen=new Set();
            return wall.pressureSites.flatMap(site=>{
                const f=site.fraction,owner=compositeJointWallOwner(wall.contactOwners,site.edge,site.owner),key=JSON.stringify([site.owner,site.edge,f]);
                if(!owner||!Number.isInteger(site.edge)||!Number.isFinite(f)||f<0||f>1||site.trace!==trace(f)||seen.has(key))throw failure('Distinct owned fixed material points and matching traces required');
                seen.add(key);
                return (site.sdfSeam?[0,1]:[null]).map(branchIndex=>[site.owner,f===0?site.edge:f===1?site.edge+1:null,site.edge,site.trace,f,...(branchIndex===null?[]:[{...copy(site.sdfSeam),branchIndex,fraction:f}])]);
            });
        }
        if(wall.pressureDiscretization!=='nodal-endpoints-one-sided-surface'||!Array.isArray(wall.pressureSites))
            throw failure('Nodal pressure needs its explicit initial discretization and one-sided sites');
        const expected=new Map();
        for(const e of ownedEdges)for(const node of [e.edge,e.edge+1]) {
            const key=JSON.stringify([e.wall.owner,node]),radius=positive(e.wall.radius,'Nodal owner radius');
            if(expected.has(key)&&expected.get(key)!==radius)throw failure('Nodal wall pressure does not support radius discontinuities');
            expected.set(key,radius);
        }
        const seen=new Set(),description=wall.pressureSites.map(site=>{
            const owner=compositeJointWallOwner(wall.contactOwners,site.edge,site.owner);
            if(!Number.isInteger(site.node)||!Number.isInteger(site.edge)||site.edge<0||site.edge>=n-1||!owner||owner.owner!==site.owner||
                !(site.node===site.edge&&site.trace==='right'||site.node===site.edge+1&&site.trace==='left'))
                throw failure('Each nodal site must select an owned physical endpoint and its exact one-sided trace');
            const key=JSON.stringify([site.owner,site.node]);
            if(!expected.has(key)||seen.has(key))throw failure('Each physical owned node needs exactly one nodal Fn/Ft owner');
            seen.add(key);return [site.owner,site.node,site.edge,site.trace];
        });
        if(seen.size!==expected.size)throw failure('Nodal pressure sites must cover every physical owned endpoint');
        return description;
    }
    if(wall.contactOwners?.edges?.length!==n-1)throw failure('Wall ownership must cover actual edges');
    const sites=nodal?copy(pressureDescription()):null,pressure=nodal?JSON.stringify(sites):null;
    const recordIdentity=v=>JSON.stringify([v.index,v.key,v.base,v.owner,v.edge,v.role,v.radius,...(nodal?[v.node,v.trace,...(materialPoints?[v.fraction,v.sdfBranch??null]:[])]:[])]);
    if(new Set(sampleIds).size!==views.length||retained.length!==views.length||new Set(retained).size!==views.length||normal.rows.length!==retained.length||
        views.some((s,i)=>s.index!==i||s.base!==slotCount*i||!retainedIndex.has(s.base)||
            (nodal?(s.owner!==sites[i]?.[0]||(s.node??null)!==sites[i]?.[1]||s.edge!==sites[i]?.[2]||s.trace!==sites[i]?.[3]||
                s.role!==(materialPoints?'material-point':s.trace==='right'?'proximal':'distal')||materialPoints&&(s.fraction!==sites[i]?.[4]||JSON.stringify(s.sdfBranch??null)!==JSON.stringify(sites[i]?.[5]??null))||retained[i]!==s.base):s.role!=='capsule'))||nodal&&sites.length!==views.length)
        throw failure('Original capsule or nodal records must match their declared physical force slots and sites');
    const field=wall.field,query=field?.queryCapsuleCoordinates,plane=policy.source==='analytic-plane'||policy.source==='original-field'&&wall.plane!==undefined?{
        normal:vector(wall.plane?.normal,3,'Stationary plane normal'),offset:finite(wall.plane?.offset,'Stationary plane offset')}:null;
    if(typeof query!=='function')throw failure('Original wall field query provider is required');
    const localFaceIndices=Array.from(wall.localFaceIndices??[]);
    const policySignature=()=>{
        if((policy.muByOwner!==undefined)!==perOwner||perOwner===(twoBranch?policy.muStatic!==undefined||policy.muKinetic!==undefined:policy.mu!==undefined)||twoBranch&&policy.mu!==undefined)
            throw failure('Prepared Coulomb coefficient declaration changed');
        return JSON.stringify({law:policy.law,...(twoBranch?(perOwner?{}:branchCoefficients(policy)):{mu:Array.from(policy.mu??[])}),...(perOwner?{muByOwner:coefficientDescription()}:{}),materialPath:policy.materialPath,motion:policy.motion,
            source:policy.source,tangentBasis:policy.tangentBasis,rateMode:policy.rateMode,slipModel:policy.slipModel,finiteStepSlipKnown:policy.finiteStepSlipKnown,
            plane:policy.source==='analytic-plane'||policy.source==='original-field'?wall.plane??null:null,faces:Array.from(wall.localFaceIndices??[])});
    };
    const frozenPolicy=policySignature();
    for(const [name,value] of Object.entries({normalRowOffset,frictionRowOffset}))if(!Number.isInteger(value)||value<0)throw failure(`${name} must be a nonnegative global row index`);
    if(frictionRowOffset<normalRowOffset+retained.length&&frictionRowOffset+2*retained.length>normalRowOffset)throw failure('Normal and friction global row intervals overlap');
    const history=state.wallFrictionState===undefined?null:copy(state.wallFrictionState);
    const incomingList=twoBranch?copy(policy.incomingSurfaceMotion??[]):[];
    const incomingRateHistory=twoBranch?copy(policy.incomingRateHistory??null):null;
    if(!Array.isArray(incomingList))throw failure('Incoming physical surface motion must be an explicit list');
    const incoming=new Map();
    const motionKey=(owner,edge,node,fraction)=>JSON.stringify([owner,edge,node??null,...(materialPoints?[fraction]:[])]);
    for(const entry of incomingList) {
        const key=motionKey(entry.owner,entry.edge,entry.node,entry.fraction);
        if(incoming.has(key)||!views.some(s=>motionKey(s.owner,s.edge,s.node,s.fraction)===key))
            throw failure('Incoming motion must identify distinct actual physical wall samples');
        if(entry.interpretation==='physical-tangential-surface-velocity') {
            if(views.some(s=>s.sdfBranch&&motionKey(s.owner,s.edge,s.node,s.fraction)===key))throw failure('A branch contact requires physical material incoming velocity');
            incoming.set(key,{velocity:vector(entry.velocity,2,'Incoming tangential surface velocity')});
        }
        else if(entry.interpretation==='physical-material-surface-velocity') {
            const labels=vector(entry.materialLabels,2,'Incoming velocity material labels');
            if(!(labels[1]>labels[0])||entry.centerVelocities?.length!==2)throw failure('Incoming material velocity needs an increasing own material interval and two endpoint velocities');
            incoming.set(key,{labels,centerVelocities:entry.centerVelocities.map(v=>vector(v,3,'Incoming material center velocity')),
                angularVelocity:vector(entry.angularVelocity,3,'Incoming material angular velocity')});
        } else throw failure('Incoming motion requires an explicit physical surface velocity interpretation');
    }
    const oldPositions=new Map([...layout.spins.keys()].map(id=>{
        const p=state.toolPositions.get(id);if(p?.length!==n)throw failure('Missing own incoming physical geometry');
        const owned=p.map(v=>vector(v,3,'Incoming physical position')),previous=prepared.previousPositions.get(id);
        if(previous?.length!==n||owned.some((v,i)=>!same(previous[i],v)))throw failure('Prepared previous geometry must equal the incoming physical state');
        return [id,owned];
    }));
    const byTool=new Map(state.tools.map(t=>[t.id,t])),edges=new Map();
    const oldMaps=new Map();
    if(history) {
        if(!Array.isArray(history.currentMaps))throw failure('Accepted friction history needs its own current material maps');
        for(const m of history.currentMaps){const key=mapKey(m);if(oldMaps.has(key))throw failure('Duplicate accepted material map');oldMaps.set(key,m.currentMap);}
    }
    function ownEdge(id,edge,semanticId) {
        const idString=edgeId(semanticId),key=JSON.stringify([id,edge]);
        if(edges.has(key)){const t=edges.get(key);if(t.edgeId!==idString)throw failure('One actual material edge has inconsistent semantic identities');return t;}
        if(!layout.edgeToolIds[edge]?.includes(id)||!(layout.spins.get(id)?.[edge]>=0))throw failure('A friction surface needs its actual edge and own spin');
        const matches=prepared.inertiaEdges[edge]?.tools?.filter(t=>t.id===id);if(matches?.length!==1)throw failure('Each surface needs exactly one prepared material map');
        const map=matches[0].materialMap,dx=positive(state.coordinates[edge+1]-state.coordinates[edge],'Own coordinate span');
        const sStart=finite(map?.sStart,'Current material start'),dsDx=positive(map?.dsDx,'Current dsDx');
        const rates=typeof map.dsDt==='number'?[finite(map.dsDt,'dsDt'),map.dsDt]:vector(map.dsDt,2,'Material endpoint rates');
        const startChange=finite(dt*rates[0],'Material start increment'),slopeChange=finite(dt*(rates[1]-rates[0])/dx,'Material slope increment');
        const previousMap={sStart:finite(sStart-startChange,'Old material start'),dsDx:positive(dsDx-slopeChange,'Old material slope')};
        const tool=byTool.get(id),frame=tool?.reference?.[edge],oldAngle=state.angles.get(id)?.[edge];
        const result={id,edge,edgeId:idString,materialSegmentId:semanticId,coordinates:[state.coordinates[edge],state.coordinates[edge+1]],rates,currentMap:{sStart,dsDx},previousMap,
            previousPositions:[oldPositions.get(id)[edge],oldPositions.get(id)[edge+1]],
            reference:{tangent:vector(frame?.tangent,3,'Own old tangent'),director:vector(frame?.director,3,'Own old director')},previousAngle:finite(oldAngle,'Own old angle')};
        const prior=oldMaps.get(mapKey(result));
        if(history&&(!prior||!Number.isFinite(prior.sStart)||!Number.isFinite(prior.dsDx)||
            !roundoffEqual(previousMap.sStart,prior.sStart,Math.abs(sStart)+Math.abs(startChange))||
            !roundoffEqual(previousMap.dsDx,prior.dsDx,Math.abs(dsDx)+Math.abs(slopeChange))))throw failure('Derived old material map disagrees with accepted friction history');
        edges.set(key,result);return result;
    }
    // A manager owns only this dt's arenas. The reusable scratch never retains
    // any path/history object or an ever-growing cache of prior preparations.
    const poseList=wall.surfacePosePaths;
    if(poseList!==undefined&&!Array.isArray(poseList))throw failure('surfacePosePaths must be an explicit prepared path list');
    const poseEntries=new Map(),poseContracts=[];
    for(const entry of poseList??[]) {
        const {owner,edge,path}=entry??{},key=JSON.stringify([owner,edge]),data=readCompositeJointSurfacePosePath(path),target=data.edges[data.target];
        const declared=compositeJointWallOwner(wall.contactOwners,edge,owner);
        if(poseEntries.has(key)||!Number.isInteger(edge)||!declared||declared.owner!==owner||!views.some(v=>v.owner===owner&&v.edge===edge)||
            data.toolId!==owner||target.edge!==edge||target.edgeId!==edgeId(declared.materialSegmentId)||target.materialSegmentId!==declared.materialSegmentId||data.dt!==dt)
            throw failure('Prepared wall pose owner/edge/identity/dt must match one declared current surface');
        for(const e of data.edges)if(e.source==='accepted') {
            const declaration=compositeJointWallOwner(wall.contactOwners,e.edge,owner);
            if(e.edgeId!==edgeId(e.materialSegmentId)||declaration?.owner===owner&&declaration.materialSegmentId!==e.materialSegmentId)
                throw failure('Prepared accepted pose uses a changed semantic material edge identity');
            const own=ownEdge(owner,e.edge,e.materialSegmentId),map=data.maps[data.edges.indexOf(e)],dx=own.coordinates[1]-own.coordinates[0];
            if(!same(e.coordinates,own.coordinates)||!same(e.reference.tangent,own.reference.tangent)||!same(e.reference.director,own.reference.director)||e.angle!==own.previousAngle||
                e.nodes.some((j,end)=>data.nodes[j].node!==e.edge+end||!same(data.nodes[j].position,own.previousPositions[end])))
                throw failure('Every accepted pose endpoint/frame/angle must equal its incoming own physical state');
            const oldLabels=[own.previousMap.sStart,own.previousMap.sStart+own.previousMap.dsDx*dx],labels=[own.currentMap.sStart,own.currentMap.sStart+own.currentMap.dsDx*dx];
            if(e.labels.some((v,j)=>!roundoffEqual(v,oldLabels[j],Math.abs(labels[j])+dt*Math.abs(own.rates[j])))||
                map.labels.some((v,j)=>!roundoffEqual(v,labels[j],Math.abs(labels[j])))||
                map.rates.some((v,j)=>!roundoffEqual(dt*v,dt*own.rates[j],Math.abs(labels[j])+Math.abs(oldLabels[j]))))
                throw failure('Prepared accepted/current pose maps must match incoming history and prepared inertia');
        }
        for(const h of data.hinges) {
            const left=data.edges.find(e=>e.edgeId===h.leftEdgeId),right=data.edges.find(e=>e.edgeId===h.rightEdgeId);
            if(left.source==='accepted'&&right.source==='accepted'&&h.referenceTwist!==byTool.get(owner)?.referenceTwists?.[left.edge])
                throw failure('Accepted own hinge winding must equal its incoming physical state');
        }
        const surface=createCompositeJointWallSurfaceWorkspace({surfacePosePath:path});
        const contract={owner,edge,targetEdgeId:target.edgeId,reservoirIdentity:data.reservoirIdentity===null?null:edgeId(data.reservoirIdentity),
            policy:'explicit-prepared-own-pose-same-current-wall-witness',configurationColumns:data.configurationColumns,
            nodes:data.nodes.map(n=>({id:edgeId(n.id),node:n.node??null})),
            edges:data.edges.map(e=>({edgeId:e.edgeId,edge:e.edge??null,materialSegmentId:edgeId(e.materialSegmentId),source:e.source,nodeIds:e.nodeIds.map(edgeId),coordinates:e.coordinates})),
            hinges:data.hinges.map(h=>({leftEdgeId:h.leftEdgeId,rightEdgeId:h.rightEdgeId})),
            // Values/old frames/angles/maps and dt evolve. The geometric
            // dependency support and its linear policy remain physical history.
            nodeBindings:data.nodeBindings.map(b=>({nodeId:edgeId(b.nodeId),terms:b.terms})),
            angleBindings:data.angleBindings.map(b=>({edgeId:b.edgeId,terms:b.terms}))};
        poseContracts.push(contract);poseEntries.set(key,{owner,edge,path,data,surface});
    }
    poseContracts.sort((a,b)=>a.owner.localeCompare(b.owner)||a.edge-b.edge);
    const signature=JSON.stringify({normal:normalSignature,policy:frozenPolicy,owners,surface:'same-current-stationary-wall-material-witness',gauge:'no-normal-pressure-remap',
        ...(nodal?{contactMode,pressureDiscretization:wall.pressureDiscretization,pressureSites:sites}:{}),...(poseEntries.size?{surfacePosePaths:poseContracts}:{})});
    if(history&&(history.signature!==signature||!same(history.sampleIds,sampleIds)||history.tractions?.length!==2*sampleIds.length))
        throw failure('Wall friction chart/coefficient/path/sample history changed without traction transfer');
    const tractions=history?Float64Array.from(vector(history.tractions,2*sampleIds.length,'Accepted tangential history')):new Float64Array(2*sampleIds.length);
    const chart={layout,modes:state.modes,relativeToolId:state.relativeToolId};
    if(scratch.createPullback?.matches(chart))scratch.chartHits++;
    else {scratch.createPullback=createCompositeJointSurfacePullbackFactory(chart);scratch.chartBuilds++;}
    const createPullback=scratch.createPullback;
    const rows=[],records=views.map((view,i)=>{
        const declared=compositeJointWallOwner(wall.contactOwners,view.edge,view.owner);
        if(!declared||declared.owner!==view.owner||declared.radius!==view.radius)throw failure('Normal wall record differs from declared actual owner/radius');
        const tool=ownEdge(view.owner,view.edge,declared.materialSegmentId),index=retainedIndex.get(view.base);
        const pose=poseEntries.get(JSON.stringify([view.owner,view.edge]));
        const mapping=pose?createCompositeJointPhysicalColumnPullback({layout,modes:state.modes,relativeToolId:state.relativeToolId,
            currentTools:pose.surface.currentTools,configurationColumns:pose.surface.configurationColumns}):
            createPullback([tool]);
        const localRows=[0,1].map(component=>{
            const m=mapping.rows[component],size=mapping.dofCount,row={toolId:'wall-friction',index:2*i+component,tractionIndex:2*i+component,sampleIndex:i,component,
                owner:view.owner,edge:view.edge,anchorNode:mapping.anchorNode,commonDofs:m.commonDofs,relativeDofs:m.relativeDofs,unit:'mm',
                multiplierDofs:Int32Array.of(normalRowOffset+index,frictionRowOffset+2*i+1-component),multiplierJacobian:new Float64Array(2).fill(NaN),
                jacobian:new Float64Array(size).fill(NaN),forceColumn:new Float64Array(size).fill(NaN),geometricTangent:new Float64Array(size*size).fill(NaN),
                geometricTangentValid:false,residual:NaN,multiplierDerivative:NaN,tolerance:tol.linearConstraint};rows.push(row);return row;
        });
        const coefficients=commonCoefficients??coefficientsByOwner.get(view.owner);
        return {view,index:i,normalIndex:index,tool,pose,mapping,rows:localRows,identity:recordIdentity(view),coefficients,
            mode:null,decision:null,latest:null,initialMode:null,demotions:0};
    });
    if(history&&oldMaps.size!==edges.size)throw failure('Accepted friction maps contain changed physical support');
    const nodalForces=new Map([...layout.spins.keys()].map(id=>[id,Array.from({length:n},()=>[0,0,0])])),spinTorques=new Map([...layout.spins.keys()].map(id=>[id,new Float64Array(n-1)]));
    const commonScratch=new Float64Array(layout.dofCount),relativeScratch=new Float64Array(r),sampleTraction=new Float64Array(2);
    let preparedReady=false,commitReady=false,last=null,refreshes=0,provenance=null;
    function invalidate() {
        commitReady=false;last=null;commonScratch.fill(0);relativeScratch.fill(0);
        for(const p of nodalForces.values())p.forEach(v=>v.fill(0));for(const v of spinTorques.values())v.fill(0);
        for(const row of rows){row.geometricTangentValid=false;row.residual=row.multiplierDerivative=NaN;row.jacobian.fill(NaN);row.forceColumn.fill(NaN);row.geometricTangent.fill(NaN);row.multiplierJacobian.fill(NaN);}
    }
    function verify() {
        if(scratch.generation!==generation||scratch.busy)throw failure('Stale or busy prepared wall friction manager');
        if(normal.signature!==normalSignature||normal.surfaceRecords!==views||views.length!==records.length||!same(normal.rowForceIndices,retained)||normal.rows.length!==retained.length)
            throw failure('Normal wall row structure or owner changed during a prepared step');
        if(wall.mode!=='wall-coulomb'||wall.friction!==policy||policy.forcePerLength!==penalty||(wall.contactMode??'capsule')!==contactMode||nodal&&JSON.stringify(pressureDescription())!==pressure||
            policySignature()!==frozenPolicy||JSON.stringify(ownerDescription())!==owners||wall.field!==field||field.queryCapsuleCoordinates!==query)
            throw failure('Frozen wall friction source/policy/ownership changed during a prepared step');
        if(wall.surfacePosePaths!==poseList||(poseList?.length??0)!==poseEntries.size||poseList?.some(e=>{
            const owned=poseEntries.get(JSON.stringify([e?.owner,e?.edge]));return !owned||owned.path!==e?.path;
        })||
            poseList&&new Set(poseList.map(e=>JSON.stringify([e.owner,e.edge]))).size!==poseEntries.size)
            throw failure('Prepared wall pose contracts changed during the current step');
        vector(tractions,2*records.length,'Private traction');vector(normal.normalForces,slotCount*records.length,'Private normal forces');
        for(const rec of records) {
            const v=rec.view;
            if(recordIdentity(v)!==rec.identity||v.seam!=null||v.representative!=null||v.dependent!=null||
                (!nodal&&(normal.normalForces[v.base+1]!==0||normal.normalForces[v.base+2]!==0)))
                throw failure('Wall Coulomb does not support seams, representatives, dependent gauges or pressure transfers');
        }
    }
    function configuration(toolPositions) {
        const values=[];
        for(const t of edges.values()) {
            const p=toolPositions?.get(t.id);if(p?.length!==n)throw failure('Missing current physical tool geometry');
            values.push(...vector(p[t.edge],3,'Current surface endpoint'),...vector(p[t.edge+1],3,'Current surface endpoint'),finite(candidate.angles.get(t.id)?.[t.edge],'Current own angle'));
        }
        return values;
    }
    function recordProvenance(rec) {
        const row=rec.view.raw,raw=row?.rawContact;
        const source=policy.source==='original-field'?row?.source:policy.source;
        if(!['analytic-plane','sparse-sdf','sparse-sdf-bvh'].includes(source)||!raw||row.source!==source||raw.source!==source||(!rec.view.sdfBranch&&(row.derivativeSource!==source||row.derivativeUnavailable!==false)))
            throw Object.assign(failure('A supported original current wall differential at the declared source is required'),{details:{declaredSource:source,rowSource:row?.source,rawSource:raw?.source,derivativeSource:row?.derivativeSource,derivativeUnavailable:row?.derivativeUnavailable,derivativeReason:row?.derivativeReason,edge:rec.view.edge,owner:rec.view.owner}});
        if(nodal&&(row.t!==(materialPoints?rec.view.fraction:rec.view.trace==='right'?0:1)||row.role!==rec.view.role||row.edge!==rec.view.edge||row.owner!==rec.view.owner))
            throw failure('Current original endpoint row must match the selected physical site/trace');
        // A degenerate endpoint query's raw fraction does not choose another
        // physical point. Current raw geometry is still verified by Surface.
        return JSON.stringify([row.source,row.t,materialPoints?'fixed-material-point':nodal?'physical-endpoint':raw.segmentT,raw.capsuleSampleCount,raw.faceIndex??-1,Math.sign(raw.signedDistance),...(rec.view.sdfBranch?[rec.view.sdfBranch]:[])]);
    }
    function evaluateSurface(rec,toolPositions,order) {
        const t=rec.tool,v=rec.view,f=v.raw.t,dx=t.coordinates[1]-t.coordinates[0];
        const previousFraction=(t.currentMap.sStart+t.currentMap.dsDx*dx*f-t.previousMap.sStart)/(t.previousMap.dsDx*dx);
        let previousTrace=trace(previousFraction);
        if(rec.pose) {
            // The prepared labels own exact endpoint identities. Subtracting
            // a large label origin and dividing again need not recover 0/1,
            // even for an unchanged endpoint. Do not infer a trace by epsilon.
            const data=rec.pose.data,labels=data.maps[data.target].labels,old=data.edges[data.target].labels;
            const label=f===0?labels[0]:f===1?labels[1]:labels[0]+f*(labels[1]-labels[0]);
            previousTrace=label===old[0]?'right':label===old[1]?'left':undefined;
        }
        const positions=[toolPositions.get(t.id)[t.edge],toolPositions.get(t.id)[t.edge+1]];
        return evaluateCompositeJointWallSurface({current:{row:v.raw,field,positions,seam:v.seam,sdfBranch:v.sdfBranch,plane,localFaceIndices},
            tool:{...t,positions,radius:v.radius,angle:candidate.angles.get(t.id)[t.edge],trace:nodal?v.trace:trace(f),materialMap:{...t.currentMap,dsDtEnds:t.rates.slice()},
                materialPath:{kind:'linear-affine-maps',previousEdgeId:t.edgeId,previousMap:{...t.previousMap},previousTrace}},dt,
            ...(rec.pose?{configuration:Float64Array.from(rec.pose.data.configurationColumns,c=>c.kind==='position'
                ?toolPositions.get(c.toolId)?.[c.node]?.[c.component]:candidate.angles.get(c.toolId)?.[c.edge])}:{}),
            wall:{motion:policy.motion,source:policy.source==='original-field'?v.raw.source:policy.source,tangentBasis:policy.tangentBasis,rateMode:policy.rateMode},order},rec.pose?.surface??scratch.surface);
    }
    function incomingVelocity(rec,surface) {
        if(incomingRateHistory) {
            const t=rec.tool,f=rec.view.raw.t,label=t.currentMap.sStart+t.currentMap.dsDx*(t.coordinates[1]-t.coordinates[0])*f,
                rate=sampleCompositeNativeRateHistory({history:incomingRateHistory,toolId:t.id,label,trace:f===1?'left':'right'}),
                lever=Array.from(surface.point,(v,k)=>v-rate.position[k]),w=rate.angularVelocity,
                rotation=[w[1]*lever[2]-w[2]*lever[1],w[2]*lever[0]-w[0]*lever[2],w[0]*lever[1]-w[1]*lever[0]],velocity=rate.velocity.map((v,k)=>v+rotation[k]);
            return surface.physicalForce.axes.map(a=>a.reduce((sum,v,k)=>sum+v*velocity[k],0));
        }
        const input=incoming.get(motionKey(rec.view.owner,rec.view.edge,rec.view.node,rec.view.fraction));
        if(!input||input.velocity)return input?.velocity;
        // These are physical material rates, not grid transport or a finite
        // spin increment divided by dt. Use the old center of THIS material
        // label and the original stationary wall witness already queried by
        // the normal manager. No second collision query chooses its lever.
        const t=rec.tool,dx=t.coordinates[1]-t.coordinates[0],label=surface.motion.materialLabel,
            u=implicitRate?(label-t.previousMap.sStart)/(t.previousMap.dsDx*dx):null,
            own=implicitRate?{materialLabel:label,previousFraction:u,previousCenter:t.previousPositions[0].map((v,k)=>v+u*(t.previousPositions[1][k]-v))}:surface.motion?.tools?.find(t=>t.id===rec.tool.id);
        if(rec.pose||!own||!roundoffEqual(input.labels[0],t.previousMap.sStart,Math.abs(input.labels[0]))||
            !roundoffEqual(input.labels[1],t.previousMap.sStart+t.previousMap.dsDx*dx,Math.abs(input.labels[1])))
            throw failure('Incoming material surface velocity needs the same incoming own edge and material interval');
        finite(own.materialLabel,'Incoming surface material label');const fraction=own.previousFraction;
        if(!(fraction>=0&&fraction<=1))throw failure('Incoming surface velocity cannot extrapolate to another material edge');
        const center=vector(own.previousCenter,3,'Incoming same-material center'),w=input.angularVelocity,
            lever=Array.from(surface.point,(v,k)=>v-center[k]),rotation=[w[1]*lever[2]-w[2]*lever[1],w[2]*lever[0]-w[0]*lever[2],w[0]*lever[1]-w[1]*lever[0]],
            velocity=input.centerVelocities[0].map((v,k)=>(fraction===0?v:fraction===1?input.centerVelocities[1][k]:v+fraction*(input.centerVelocities[1][k]-v))+rotation[k]);
        if(surface.motion.axes?.length!==2)throw failure('Incoming surface requires the original tangent axes');
        return vector(surface.motion.axes.map(axis=>vector(axis,3,'Incoming contact tangent axis').reduce((sum,a,k)=>sum+a*velocity[k],0)),2,'Projected incoming surface velocity');
    }
    const openWithoutSurface=rec=>implicitRate&&rec.view.raw.derivativeUnavailable===true&&rec.view.raw.gap>0&&normal.normalForces[rec.view.base]===0;
    function initializeMode(rec,explicit) {
        if(!twoBranch||rec.mode!==null)return;
        const c=rec.coefficients,old=history?.modeHistory?.[rec.index],t=rec.tool,
            label=t.currentMap.sStart+t.currentMap.dsDx*(t.coordinates[1]-t.coordinates[0])*rec.view.raw.t,
            sameCertifiedLabel=old?.sampleId===rec.view.key&&old.label===label&&['static','kinetic'].includes(old.nextMode);
        if(c.muStatic.every((v,i)=>v===c.muKinetic[i]))rec.mode='static';
        else if(incomingRateHistory&&sameCertifiedLabel)rec.mode=old.nextMode;
        else if(explicit)rec.mode=explicit.every((v,i)=>c.muStatic[i]===0||v===0)?'static':'kinetic';
        else {
            if(!sameCertifiedLabel)
                throw failure('A new material contact needs explicit incoming surface motion; a spatial slot cannot inherit another label\'s stop');
            rec.mode=old.nextMode;
        }
        rec.initialMode=rec.mode;
    }
    function prepare({toolPositions=state.toolPositions}={}) {
        invalidate();verify();if(preparedReady)throw failure('Wall friction is already prepared');
        normal.assertCurrentSurface({toolPositions});configuration(toolPositions);scratch.busy=true;
        try {
            const initialVelocities=twoBranch?new Map():null;
            const next=records.map(rec=>{if(openWithoutSurface(rec))return null;const p=recordProvenance(rec),surface=evaluateSurface(rec,toolPositions,'value');
                if(twoBranch)initialVelocities.set(rec,incomingVelocity(rec,surface));return p;});
            if(history&&!implicitRate&&(!Array.isArray(history.provenance)||!same(history.provenance,next)))throw failure('Accepted wall friction source/sample/feature history changed without transfer');
            if(twoBranch)for(const rec of records) {
                if(!openWithoutSurface(rec))initializeMode(rec,initialVelocities.get(rec));
            }
            provenance=next;preparedReady=true;return {queries:0,samples:records.length};
        } finally {scratch.busy=false;}
    }
    function refresh({toolPositions,commonResidual,relativeResidual,order}) {
        invalidate();verify();if(!preparedReady)throw failure('Wall friction needs original current normal rows before refresh');
        normal.assertCurrentSurface({toolPositions});scratch.busy=true;
        try {
            vector(commonResidual,layout.dofCount,'Current common residual');vector(relativeResidual,r,'Current relative residual');
            if(order!=='full'&&order!=='gradient')throw failure('Explicit full or gradient friction evaluation is required');
            const values=configuration(toolPositions),proofs=[];let converged=true,merit=0,lineSearchMerit=0;
            for(const rec of records) {
                const s=rec.view,Fn=normal.normalForces[s.base];
                if(openWithoutSurface(rec)) {
                    // The exact open normal equation holds Fn=0, hence its
                    // Coulomb cone is the singleton Ft=0. No witness or spin
                    // estimate is needed to eliminate this unloaded surface.
                    const Ft=Array.from(tractions.slice(2*rec.index,2*rec.index+2)),ok=Ft.every(v=>v===0);converged&&=ok;
                    const eq=Ft.reduce((sum,v)=>sum+(v/penalty/tol.frictionEquation)**2,0);merit+=eq;lineSearchMerit+=eq;
                    rec.rows.forEach((row,c)=>{row.residual=Ft[c]/penalty;row.multiplierDerivative=1/penalty;row.multiplierJacobian.fill(0);row.jacobian.fill(0);row.forceColumn.fill(0);row.geometricTangent.fill(0);row.geometricTangentValid=order==='full';});
                    rec.latest={unloaded:true,traction:Ft,slip:null,normalForce:Fn};rec.decision=null;provenance[rec.index]=null;
                    proofs.push({sampleId:s.key,owner:s.owner,edge:s.edge,Fn,traction:Ft,slip:null,unloaded:true,converged:ok,finiteStepSlipKnown:false,rateMode:policy.rateMode});continue;
                }
                const currentProvenance=recordProvenance(rec);
                if(!implicitRate&&currentProvenance!==provenance[rec.index])throw failure('Original wall source/sample/feature changed during a prepared step');
                if(implicitRate)provenance[rec.index]=currentProvenance;
                const surface=evaluateSurface(rec,toolPositions,order==='full'?'full':'value');
                if(twoBranch&&rec.mode===null)initializeMode(rec,incomingVelocity(rec,surface));
                const mu=twoBranch?rec.coefficients[rec.mode==='static'?'muStatic':'muKinetic']:rec.coefficients.mu;
                const slip=vector(surface.increment,2,'Finite material surface increment');
                if(surface.incrementValid!==true||surface.forceMapValid!==true||order==='full'&&(surface.slipJacobianValid!==true||surface.DforceMapValid!==true))
                    throw failure('Current finite slip/physical B and all requested geometry derivatives are required');
                sampleTraction[0]=tractions[2*rec.index];sampleTraction[1]=tractions[2*rec.index+1];
                const eq=evaluateCompositeFrictionEquation({traction:sampleTraction,slip,normalForce:Fn,mu,penalty},scratch.equation);
                if(!eq.valid||!eq.operatorReady)throw failure('Friction equation evaluation is invalid');
                const equationResidual=Array.from(eq.residual),equation=Math.max(...equationResidual.map(Math.abs));
                const physical=Fn>=0?measureCompositeFriction({traction:sampleTraction,slip,normalForce:Fn,mu,
                    slipTolerance:tol.frictionSlip,coneTolerance:tol.frictionCone,workTolerance:tol.frictionWork}):
                    {converged:false,work:sampleTraction[0]*slip[0]+sampleTraction[1]*slip[1],minimumWork:null,workGap:null,coneViolation:null,slipResidual:null};
                const ok=Fn>=0&&physical.converged&&equation<=tol.frictionEquation;
                rec.latest={traction:Array.from(sampleTraction),slip:slip.slice(),normalForce:Fn};rec.decision=null;
                const equationMerit=equationResidual.reduce((sum,v)=>sum+(v/tol.frictionEquation)**2,0);
                lineSearchMerit+=equationMerit;
                merit+=Fn>=0?equationMerit+(physical.slipResidual/tol.frictionSlip)**2+(physical.coneViolation/tol.frictionCone)**2+(physical.workGap/tol.frictionWork)**2:Infinity;
                converged=converged&&ok;
                proofs.push({sampleId:s.key,key:s.key,owner:s.owner,edge:s.edge,...(nodal?{node:s.node,trace:s.trace,role:s.role,...(materialPoints?{fraction:s.fraction}:{})}:{}),forceIndex:s.base,Fn,traction:Array.from(sampleTraction),slip,mu:mu.slice(),equationResidual,
                    ...(twoBranch?{mode:rec.mode,muStatic:rec.coefficients.muStatic.slice(),muKinetic:rec.coefficients.muKinetic.slice()}:{}),
                    work:physical.work,minimumWork:physical.minimumWork,workGap:physical.workGap,coneViolation:physical.coneViolation,slipResidual:physical.slipResidual,
                    converged:ok,redundant:false,normalAdmissible:Fn>=0,surfaceWitness:surface.identity.wallWitness,wallWork:0,
                    ...(implicitRate?{rateMode:policy.rateMode,slipModel:policy.slipModel,finiteStepSlipKnown:false}: {})});
                // A trial needs its true current B and finite slip. G/DB are
                // formed only for a full Newton assembly, never approximated
                // by B or reused from an earlier geometry.
                const mapped=rec.pose?pullbackCompositeJointPhysicalColumns(order==='full'?surface:{currentTools:surface.currentTools,configurationColumns:surface.configurationColumns,
                    forceMap:surface.forceMap,forceMapValid:true},rec.mapping):
                    pullbackCompositeJointSurface(order==='full'?surface:{tools:surface.tools,forceMap:surface.forceMap,forceMapValid:true},rec.mapping),
                    loads=rec.pose?evaluateCompositeJointPhysicalColumnLoads(sampleTraction,mapped):evaluateCompositeJointSurfaceLoads(sampleTraction,mapped),size=mapped.dofCount;
                rec.rows.forEach((row,c)=>{
                    row.residual=eq.residual[c];row.multiplierDerivative=eq.tractionJacobian[2*c+c];
                    row.multiplierJacobian[0]=eq.normalDerivative[c];row.multiplierJacobian[1]=eq.tractionJacobian[2*c+1-c];
                    for(let j=0;j<size;j++){row.jacobian[j]=order==='full'?eq.slipJacobian[2*c]*mapped.slipJacobian[j]+eq.slipJacobian[2*c+1]*mapped.slipJacobian[size+j]:NaN;row.forceColumn[j]=mapped.rows[c].forceColumn[j];}
                    if(order==='full')for(let j=0;j<size*size;j++)row.geometricTangent[j]=-sampleTraction[c]*mapped.rows[c].forceDerivative[j];
                    row.geometricTangentValid=order==='full';
                });
                mapped.commonDofs.forEach((d,i)=>commonScratch[d]-=loads.common[i]);mapped.relativeDofs.forEach((d,i)=>relativeScratch[d]-=loads.relative[i]);
                for(const t of loads.tools){t.nodes.forEach((node,e)=>t.nodalForces[e].forEach((v,k)=>nodalForces.get(t.id)[node][k]+=v));
                    if(rec.pose)t.edges.forEach((edge,j)=>spinTorques.get(t.id)[edge]+=t.spinTorques[j]);else spinTorques.get(t.id)[t.edge]+=t.scalarTorque;}
            }
            if(!Number.isFinite(lineSearchMerit)||!commonScratch.every(Number.isFinite)||!relativeScratch.every(Number.isFinite)||
                commonScratch.some((v,i)=>!Number.isFinite(v+commonResidual[i]))||relativeScratch.some((v,i)=>!Number.isFinite(v+relativeResidual[i])))throw failure('Nonfinite complete friction residual');
            commonScratch.forEach((v,i)=>commonResidual[i]+=v);relativeScratch.forEach((v,i)=>relativeResidual[i]+=v);
            const certificate={scope:implicitRate?`original-${materialPoints?'material-points':nodal?'nodal-endpoints':'capsule'}-stationary-wall-implicit-rate-Coulomb-rows`:materialPoints?'original-material-points-stationary-wall-finite-Coulomb-rows':nodal?'original-nodal-endpoints-stationary-wall-finite-Coulomb-rows':'original-capsule-stationary-wall-finite-Coulomb-rows',
                ...(implicitRate?{rateMode:policy.rateMode,slipModel:policy.slipModel,finiteStepSlipKnown:false}:{}),converged,merit,lineSearchMerit,samples:proofs,
                sampleCount:records.length,retainedRows:rows.length};
            last={toolPositions,values,tractions:tractions.slice(),normalForces:normal.normalForces.slice()};commitReady=converged;refreshes++;scratch.refreshes++;
            return certificate;
        } catch(error){invalidate();throw error;} finally{scratch.busy=false;}
    }
    function commit() {
        verify();
        if(!commitReady||!last||!same(configuration(last.toolPositions),last.values)||!same(tractions,last.tractions)||!same(normal.normalForces,last.normalForces))
            throw failure('Friction commit requires unchanged certified positions, own angles, Ft and Fn');
        normal.assertCurrentSurface({toolPositions:last.toolPositions});
        if(records.some((rec,i)=>(openWithoutSurface(rec)?null:recordProvenance(rec))!==provenance[i]))throw failure('Certified original wall provenance changed');
        if(twoBranch&&records.some(rec=>!rec.decision?.accepted))throw failure('Two-branch friction needs a fresh whole-step mode certificate');
        return {signature,sampleIds:sampleIds.slice(),tractions:tractions.slice(),law:policy.law,
            ...(implicitRate?{rateMode:policy.rateMode,slipModel:policy.slipModel,finiteStepSlipKnown:false}:{}),
            ...(perOwner?{muByOwner:copy(ownerCoefficients)}:copy(commonCoefficients)),materialPath:'linear-affine-maps',
            ...(twoBranch?{modeHistory:records.map(rec=>({sampleId:rec.view.key,
                label:rec.tool.currentMap.sStart+rec.tool.currentMap.dsDx*(rec.tool.coordinates[1]-rec.tool.coordinates[0])*rec.view.raw.t,
                mode:rec.mode,nextMode:rec.decision.nextMode,stopCertificate:rec.decision.stopCertificate}))}:{}),
            motion:'stationary-material',provenance:provenance.slice(),...(nodal?{contactMode,pressureDiscretization:wall.pressureDiscretization,pressureSites:copy(sites)}:{}),
            currentMaps:Array.from(edges.values(),t=>({id:t.id,edge:t.edge,edgeId:t.edgeId,currentMap:{...t.currentMap}}))};
    }
    function resolveModes({wholeStepConverged}={}) {
        verify();if(!twoBranch)return {accepted:true,changed:false,samples:[]};
        if(!wholeStepConverged||!commitReady||!last||!same(configuration(last.toolPositions),last.values)||!same(tractions,last.tractions)||!same(normal.normalForces,last.normalForces))
            return {accepted:false,changed:false,status:'unconverged',samples:[]};
        normal.assertCurrentSurface({toolPositions:last.toolPositions});
        const decisions=records.map(rec=>{
            if(rec.latest.unloaded)return {accepted:true,change:false,status:'unloaded',nextMode:null,stopCertificate:null};
            let decision=assessCompositeStaticKineticFriction({...rec.latest,...rec.coefficients,mode:rec.mode,wholeStepConverged,
                slipTolerance:tol.frictionSlip,coneTolerance:tol.frictionCone,workTolerance:tol.frictionWork});
            if(implicitRate&&decision.status==='ambiguous'&&rec.mode==='static'&&decision.physical.converged&&
                Math.hypot(...rec.latest.slip.filter((_,i)=>rec.coefficients.muStatic[i]>0))<=tol.frictionSlip)
                decision={...decision,status:'static-within-residual-tolerance',accepted:true,nextMode:'static',stopCertificate:'static-cone-and-declared-slip-residual'};
            return decision;
        });
        const changed=decisions.some(d=>d.change);
        const samples=records.map((rec,i)=>({sampleId:rec.view.key,initialMode:rec.initialMode,...decisions[i]}));
        records.forEach((rec,i)=>{rec.decision=decisions[i];if(rec.decision.change){
            if(rec.mode!=='static'||rec.demotions!==0)throw failure('Each static sample may demote only once within a prepared dt');
            rec.mode='kinetic';rec.demotions++;rec.decision=null;
        }});
        if(changed)invalidate();
        return {accepted:!changed&&decisions.every(d=>d.accepted),changed,status:changed?'breakaway':decisions.every(d=>d.accepted)?'accepted':'ambiguous',samples};
    }
    verify();
    return {signature,rows,tractions,nodalForces,spinTorques,prepare,refresh,commit,resolveModes,workspace,
        get diagnostics(){return {preparationQueries:0,refreshes,retainedRows:rows.length,samples:records.length};}};
}
