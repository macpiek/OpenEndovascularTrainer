import {createCompositeDiscreteWallPointBranches} from './compositeDiscreteWallPointBranches.js';
import {createCompositeDiscreteWallPoint} from './compositeDiscreteWallPoint.js';
import {createCompositeChainLayout} from './kirchhoffCompositeChain.js';
import {createCompositeWallWorkspace,refreshCompositeWallContacts} from './kirchhoffCompositeWallContacts.js';
import {equalCompositeWallGlobalDifferentials,isCompositeWallEndpointCombination} from './kirchhoffCompositeWallDifferentialRows.js';
import {createCompositeWallEnvelopeWorkspace,refreshCompositeWallEnvelope} from './kirchhoffCompositeWallEnvelope.js';
import {createCompositeContactPullbackFactory,pullbackCompositeContact} from './kirchhoffCompositeContactPullback.js';
import {compositeJointWallOwner,expandCompositeJointWallOwners} from './kirchhoffCompositeJointWallOwners.js';
import {createCompositeWallSdfBranchesWorkspace,evaluateCompositeWallSdfBranches,measureCompositeWallSdfBranches,
    findCompositeWallSdfSeamCrossing} from './kirchhoffCompositeWallSdfBranches.js';

const identities=new WeakMap();let nextIdentity=0;
const processScope=globalThis.crypto?.randomUUID?.()??`${Date.now()}-${Math.random()}`;
const identity=value=>{
    if(value===null||value===undefined)return null;
    if(typeof value!=='object'&&typeof value!=='function')return value;
    if(!identities.has(value))identities.set(value,`${processScope}:${++nextIdentity}`);
    return identities.get(value);
};
const failure=reason=>{const e=new RangeError(reason);e.code='unsupported-joint-wall-contact';return e;};
const finite=(v,name)=>{if(!Number.isFinite(v))throw failure(`${name} must be finite`);return v;};
const positive=(v,name)=>{if(!(finite(v,name)>0))throw failure(`${name} must be positive`);return v;};
const nonnegative=(v,name)=>{if(finite(v,name)<0)throw failure(`${name} must be nonnegative`);return v;};
const text=(v,name)=>{if(typeof v!=='string'||!v)throw failure(`${name} must be a nonempty string`);return v;};
const vector=(v,n,name)=>{if(v?.length!==n||!Array.from(v).every(Number.isFinite))throw failure(`${name} needs ${n} finite entries`);return Array.from(v);};
const same=(a,b)=>a.length===b.length&&a.every((v,i)=>v===b[i]);
const copy=v=>structuredClone(v);
const nodalPressureScheme='nodal-endpoints-one-sided-surface';
const storage=value=>value==null?null:{identity:identity(value),buffer:identity(value.buffer),length:value.length??null,
    byteOffset:value.byteOffset??null,byteLength:value.byteLength??null,version:value.version??null};
function sourceSignature(field) {
    const geometry=field.fallbackGeometry,p=geometry?.attributes?.position,index=geometry?.index;
    const scalars=['voxelSize','brickSize','sdfQuantization','bvhValidationDistance','capsuleBvhValidationGap','capsuleBvhValidationDistance',
        'sdfBvhDistancePadding','centerlineStride','centerlinePreparedStride','sourceVersion','version'];
    const buffers=['sdfBrickLookup','sdfDistances','sdfInsideBits','sdfBrickKeys','centerline','centerlinePrepared',
        'centerlineBounds','broadPhaseOffsets','broadPhaseIds'];
    return JSON.stringify({field:identity(field),query:identity(field.queryCapsuleCoordinates),sdfGridCoordinates:identity(field.sdfGridCoordinates),metadata:identity(field.metadata),arrays:identity(field.arrays),
        packed:identity(field.packedLumenField),fallbackCollider:identity(field.fallbackCollider),
        scalars:scalars.map(k=>[k,field[k]??null]),buffers:buffers.map(k=>[k,storage(field[k])]),
        vectors:['sdfOrigin','sdfDimensions','broadPhaseOrigin','broadPhaseDimensions'].map(k=>[k,field[k]==null?null:Array.from(field[k])]),
        geometry:identity(geometry),boundsTree:identity(geometry?.boundsTree),position:p?{identity:identity(p),array:storage(p.array),version:p.version,itemSize:p.itemSize,count:p.count}:null,
        index:index?{identity:identity(index),array:storage(index.array),version:index.version,itemSize:index.itemSize,count:index.count}:null});
}
// World preparation pins the same source contract across numerical retries.
export {sourceSignature as compositeJointWallSourceSignature};
function chartSignature(layout,coordinates,modes,relativeToolId,owners) {
    const serializeWall=wall=>{
        if(wall?.materialSegmentId===undefined)return wall;
        const id=wall.materialSegmentId;
        if(typeof id==='string'||typeof id==='number'&&Number.isFinite(id))return wall;
        if(typeof id==='bigint')return {...wall,materialSegmentId:{type:'joint-wall-bigint-id',value:id.toString()}};
        throw failure('Wall materialSegmentId must be a string, finite number or bigint');
    };
    const serializedOwners={...owners,edges:owners.edges.map(e=>e.walls!==undefined?{...e,walls:e.walls.map(serializeWall)}:{...e,wall:serializeWall(e.wall)})};
    return JSON.stringify({edges:layout.edgeToolIds,positions:Array.from(layout.positions),dofCount:layout.dofCount,band:layout.band,
        spins:Array.from(layout.spins,([id,d])=>[id,Array.from(d)]),coordinates:Array.from(coordinates),relativeToolId,
        modes:modes.map(m=>({node:m.node,basis:m.basis.map(b=>Array.from(b)),relativeDofs:Array.from(m.relativeDofs)})),owners:serializedOwners});
}

/** Local ORIGINAL wall normal rows for the fixed full q/rho chart. Each
 * frozen outer owner has a separate original capsule collector, evaluated on
 * toolPositions.get(owner). G and physical unit-normal B remain distinct.
 * No solve, integration, friction, force clipping or dt commit occurs here.
 *
 * The field and its source buffers are BORROWED IMMUTABLE providers. Identity,
 * storage/version and small policy/geometry metadata are checked on every
 * call; in-place source writes without a version change violate that contract.
 * History tokens are cloneable but process/module-instance scoped: exported
 * history cannot be silently rebound to a reconstructed field in another run.
 *
 * Three fixed force slots per owned edge: capsule mode uses ordinary/lower/
 * upper SDF branches; envelope uses proximal/distal/original-capsule roles.
 * Envelope retains every original gap and removes only exactly proved
 * dependent g/G/B/DB rows. SDF cone charts remain capsule-mode only.
 * Modes/roles are explicit history provenance and cannot be reinterpreted.
 * Caller checkpoints BEFORE trials; restore restores dual AND chart choices,
 * invalidates raw queries/operators and requires a new original refresh.
 */
export function createCompositeJointWallRows({layout,coordinates,modes,relativeToolId=layout?.spins?.size===1?layout.spins.keys().next().value:'wire',wall,history=null,tolerances,preserveSampleReactions=false}) {
    if(wall?.mode!=='wall-normal'||wall.friction!=='none')throw failure('Explicit wall-normal with friction:none is required');
    if(typeof preserveSampleReactions!=='boolean')throw failure('Explicit boolean sample reaction policy is required');
    const contactUpdate=wall.contactUpdate??'frozen-chart';
    if(!['frozen-chart','current-query'].includes(contactUpdate))throw failure('Unknown wall contact update policy');
    const contactMode=wall.contactMode??'capsule';
    if(!['capsule','envelope','nodal-endpoints','material-points'].includes(contactMode))throw failure('Wall contactMode must be capsule, envelope or nodal-endpoints');
    const nodal=contactMode==='nodal-endpoints',material=contactMode==='material-points',pointLaw=nodal||material;
    if(material&&wall.pressureDiscretization!=='fixed-material-points')throw failure('Material pressure needs fixed-material-points discretization');
    if(nodal&&wall.pressureDiscretization!==nodalPressureScheme)throw failure('Nodal pressure needs its explicit physical discretization from the initial state');
    const slotCount=contactMode==='capsule'?3:1;
    const field=wall.field;if(typeof field?.queryCapsuleCoordinates!=='function')throw failure('An actual original wall field is required');
    const chartId=text(wall.chartId,'Wall chart id'),k=positive(wall.forcePerLength,'Wall forcePerLength');
    if(!layout||layout.nodeCount<3||!(layout.spins instanceof Map))throw failure('A real joint layout is required');
    const expected=createCompositeChainLayout(layout.edgeToolIds);
    if(layout.dofCount!==expected.dofCount||layout.band!==expected.band||!same(Array.from(layout.positions),Array.from(expected.positions))||
        JSON.stringify(Array.from(layout.spins,([id,d])=>[id,Array.from(d)]))!==JSON.stringify(Array.from(expected.spins,([id,d])=>[id,Array.from(d)])))throw failure('Layout indices do not match the actual chart');
    const x=vector(coordinates,layout.nodeCount,'Chart coordinates');if(x.some((v,i)=>i&&v<=x[i-1]))throw failure('Chart coordinates must increase');
    const overlap=Array.from({length:layout.nodeCount},(_,i)=>i).filter(i=>new Set([...(layout.edgeToolIds[i-1]??[]),...(layout.edgeToolIds[i]??[])]).size>1);
    if(![1,2].includes(layout.spins.size)||!layout.spins.has(relativeToolId)||!Array.isArray(modes)||modes.length!==overlap.length||
        modes.some((m,i)=>m.node!==overlap[i]||m.basis?.length!==3||m.relativeDofs?.length!==3||m.relativeDofs.some((d,j)=>d!==3*i+j)))throw failure('Full ordered modes must cover every overlap node');
    const owners=copy(wall.contactOwners);
    const ownedEdges=expandCompositeJointWallOwners(owners,layout);
    // Nodal pressure is a declared physical discretization, not a reduction
    // of an existing capsule multiplier. A shared node owns one integrated
    // force and one explicit incident material surface/spin trace.
    const pressureSites=pointLaw?copy(wall.pressureSites):null;
    if(nodal) {
        const required=new Map();
        for(const e of ownedEdges)for(const node of [e.edge,e.edge+1]) {
            const key=JSON.stringify([e.wall.owner,node]),old=required.get(key);
            if(old&&old.radius!==e.wall.radius)throw failure('Nodal pressure at a radius discontinuity needs another physical surface model');
            required.set(key,{owner:e.wall.owner,node,radius:e.wall.radius});
        }
        if(!Array.isArray(pressureSites)||pressureSites.length!==required.size)throw failure('One pressure site must cover every owned material endpoint');
        const seen=new Set();
        for(const s of pressureSites) {
            const key=JSON.stringify([s.owner,s.node]),end=s.node-s.edge;
            if(!Number.isInteger(s.node)||!Number.isInteger(s.edge)||!required.has(key)||seen.has(key)||
                ![0,1].includes(end)||!compositeJointWallOwner(owners,s.edge,s.owner)||s.trace!==(end===0?'right':'left'))
                throw failure('Each unique pressure site needs an explicit incident own edge and matching one-sided trace');
            seen.add(key);
        }
    }
    if(material) {
        const required=new Map(),seen=new Set(),endpoints=new Set();
        for(const e of ownedEdges)for(const node of [e.edge,e.edge+1]) {
            const key=JSON.stringify([e.wall.owner,node]),old=required.get(key);
            if(old!==undefined&&old!==e.wall.radius)throw failure('Material pressure radius discontinuity needs another physical surface model');
            required.set(key,e.wall.radius);
        }
        if(!Array.isArray(pressureSites))throw failure('Material pressure needs declared sites');
        for(const site of pressureSites) {
            const f=site.fraction,owner=compositeJointWallOwner(owners,site.edge,site.owner);
            if(!Number.isInteger(site.edge)||!owner||!Number.isFinite(f)||f<0||f>1||site.trace!==(f===0?'right':f===1?'left':undefined))
                throw failure('Each material site needs an owned edge, fixed fraction and matching trace');
            const key=JSON.stringify([site.owner,site.edge,f]);if(seen.has(key))throw failure('Duplicate material pressure site');seen.add(key);
            if(f===0||f===1) {
                const key=JSON.stringify([site.owner,site.edge+f]);
                if(endpoints.has(key))throw failure('Duplicate material endpoint pressure site');endpoints.add(key);
            }
        }
        if(endpoints.size!==required.size||[...required.keys()].some(key=>!endpoints.has(key)))throw failure('Material sites must cover every owned endpoint');
    }
    const pressureSignature=pointLaw?JSON.stringify({scheme:wall.pressureDiscretization,sites:pressureSites}):null;
    const tol=Object.fromEntries(['force','wallGap','wallNcp','wallWork','linearConstraint'].map(name=>[name,positive(tolerances?.[name],name)]));
    const chart=chartSignature(layout,coordinates,modes,relativeToolId,owners),source=sourceSignature(field);
    const signature=JSON.stringify({chartId,chart,source,mode:wall.mode,friction:wall.friction,contactMode,contactUpdate,preserveSampleReactions,...(pointLaw?{pressure:pressureSignature}:{})});
    const collectors=new Map(),records=[],envelopes=[],createPullback=createCompositeContactPullbackFactory({layout,modes,relativeToolId});
    for(const [owner] of layout.spins) {
        const selected={edges:owners.edges.map(e=>({edge:e.edge,wall:copy(compositeJointWallOwner(owners,e.edge,owner)??null)}))};
        if(selected.edges.some(e=>e.wall!==null))collectors.set(owner,{workspace:(contactMode==='capsule'?createCompositeWallWorkspace:createCompositeWallEnvelopeWorkspace)(layout),owners:selected});
    }
    const pointBranchHelpers=new Map();
    function addRecord(e,roleIndex,role,site=null,branchIndex=null) {
            const owner=e.wall.owner,edge=e.edge;
            const points=[{toolId:owner,node:edge},{toolId:owner,node:edge+1}],base=slotCount*records.length;
            const pulled=createPullback(points),size=pulled.dofCount;
            const rows=Array.from({length:slotCount},(_,branch)=>({toolId:'wall-normal',index:base+branch,anchorNode:pulled.anchorNode,
                commonDofs:pulled.commonDofs,relativeDofs:pulled.relativeDofs,unit:'mm',residual:NaN,multiplierDerivative:0,
                jacobian:new Float64Array(size).fill(NaN),forceColumn:new Float64Array(size).fill(NaN),geometricTangent:new Float64Array(size*size).fill(NaN),
                geometricTangentValid:false,tolerance:tol.linearConstraint,forceIndex:base+branch,owner,edge,role,sdfBranch:branch===0?null:branch-1}));
            const pointOptions=material?{fraction:site.fraction,edge,dofs:[...layout.positions.slice(edge,edge+2)].flatMap(d=>[d,d+1,d+2])}:null;
            let branchHelper=null;
            if(site?.sdfSeam){
                if(!pointBranchHelpers.has(site))pointBranchHelpers.set(site,{helper:createCompositeDiscreteWallPointBranches(pointOptions),site,radius:e.wall.radius,records:[]});
                branchHelper=pointBranchHelpers.get(site);rows[0].sdfBranch=branchIndex;
            }
            const fixedPoint=branchHelper?.helper.point??(material?createCompositeDiscreteWallPoint(pointOptions):null);
            if(fixedPoint)fixedPoint.row.owner=owner;
            const record={index:records.length,key:JSON.stringify([owner,edge,e.wall.radius,role,...(material?[site.fraction,...(branchHelper?[branchIndex]:[])]:[])]),base,owner,edge,role,radius:e.wall.radius,points,pulled,rows,branchHelper,branchIndex,
                ...(site?{node:material?(site.fraction===0||site.fraction===1?edge+site.fraction:undefined):site.node,trace:site.trace,...(material?{fraction:site.fraction}:{})}:{}),fixedPoint,
                raw:fixedPoint?.row??collectors.get(owner).workspace.rows[contactMode==='capsule'?edge:3*edge+roleIndex],
                ordinaryGeometry:{supported:true,gap:NaN,gapJacobian:null,forceColumn:null,normalForceColumn:new Float64Array(6),normalDerivative:null},
                provenance:null,seam:null,representative:null,dependent:null};
            if(branchHelper)branchHelper.records.push(record);
            records.push(record);return record;
    }
    if(material)for(const s of pressureSites)for(const b of s.sdfSeam?[0,1]:[null])addRecord({edge:s.edge,wall:compositeJointWallOwner(owners,s.edge,s.owner)},0,'material-point',s,b);
    else if(nodal)for(const s of pressureSites)addRecord({edge:s.edge,wall:compositeJointWallOwner(owners,s.edge,s.owner)},s.node-s.edge,s.node===s.edge?'proximal':'distal',s);
    else for(const e of ownedEdges) {
        const group=[];
        for(const [roleIndex,role] of (contactMode==='envelope'?['proximal','distal','capsule']:['capsule']).entries())group.push(addRecord(e,roleIndex,role));
        if(contactMode==='envelope')envelopes.push(group);
    }
    // Borrowed original differentials for sequential surface assembly. Stable
    // records preserve detector ownership; validity comes from the private
    // refresh generation, not from the publicly readable geometry buffers.
    const surfaceRecords=Object.freeze(records.map(r=>Object.freeze({
        index:r.index,key:r.key,base:r.base,owner:r.owner,edge:r.edge,role:r.role,radius:r.radius,raw:r.raw,...(r.branchHelper?{sdfBranch:{...copy(r.branchHelper.site.sdfSeam),branchIndex:r.branchIndex,fraction:r.fraction}}:{}),
        ...(pointLaw?{node:r.node,trace:r.trace,...(material?{fraction:r.fraction}:{})}:{}),
        get seam(){return seamRecord(r.seam);},get representative(){return r.representative;},get dependent(){return copy(r.dependent);}
    })));
    const normalForces=new Float64Array(slotCount*records.length),nodalForces=new Map([...layout.spins.keys()].map(id=>[id,Array.from({length:layout.nodeCount},()=>[0,0,0])]));
    if(history!==null) {
        if(history.signature!==signature||history.friction!=='none'||history.contactMode!==contactMode||history.records?.length!==records.length)throw failure('Wall field/chart/owner/history provenance changed');
        normalForces.set(vector(history.normalForces,normalForces.length,'Owned wall history forces').map(v=>nonnegative(v,'Accepted wall force')));
        records.forEach((r,i)=>{
            const old=history.records[i];if(old.key!==r.key||old.role!==r.role)throw failure('Wall history physical support changed');
            r.provenance=copy(old.provenance);r.seam=old.seam===null?null:makeSeam(old.seam);r.representative=old.representative;r.dependent=copy(old.dependent);
            if(r.representative!==null&&(!Number.isInteger(r.representative)||r.representative<0||r.representative>=r.index))throw failure('Invalid duplicate representative');
            if(r.dependent!==null&&(contactMode!=='envelope'||r.role!=='capsule'||r.dependent.kind!=='endpoint-combination'||
                r.dependent.endpoints?.length!==2||r.dependent.endpoints.some((j,n)=>j!==r.index-2+n)||
                r.dependent.weights?.length!==2||r.dependent.weights.some(v=>!Number.isFinite(v)||v<=0||v>=1)||r.dependent.weights[0]+r.dependent.weights[1]!==1))throw failure('Invalid envelope gauge history');
            if((pointLaw||preserveSampleReactions)&&(r.seam||r.representative!==null||r.dependent!==null))throw failure('Preserved sample pressure history cannot contain a seam or pressure transfer');
            if(contactMode!=='capsule'&&r.seam||r.seam&&normalForces[r.base]!==0||
                slotCount===3&&!r.seam&&(normalForces[r.base+1]!==0||normalForces[r.base+2]!==0)||
                (r.representative!==null||r.dependent!==null)&&normalForces[r.base]!==0)throw failure('Wall history duplicates physical reaction ownership');
        });
    }
    let rows=[],rowForceIndices=[],structureVersion=0,rawGeneration=0,lastQuery=null,lastRefresh=null,certificate=null,busy=false;
    const checkpoints=new WeakMap();
    const stats={queries:0,refreshes:0,derivativeRefreshes:0,discoveries:0,restores:0};
    function makeSeam(saved) {
        if(!saved||!Number.isInteger(saved.face?.axis)||saved.face.axis<0||saved.face.axis>2||!Number.isSafeInteger(saved.face.gridIndex)||saved.face.gridIndex<0||
            !Number.isFinite(saved.sampleFraction)||saved.sampleFraction<0||saved.sampleFraction>1||!Number.isSafeInteger(saved.sampleCount)||saved.sampleCount<1)throw failure('Invalid saved SDF chart');
        return {face:{axis:saved.face.axis,gridIndex:saved.face.gridIndex},sampleFraction:saved.sampleFraction,sampleCount:saved.sampleCount,
            workspace:createCompositeWallSdfBranchesWorkspace(2)};
    }
    const seamRecord=s=>s===null?null:{face:{...s.face},sampleFraction:s.sampleFraction,sampleCount:s.sampleCount};
    function metadata(){return records.map(r=>({key:r.key,role:r.role,provenance:copy(r.provenance),seam:seamRecord(r.seam),representative:r.representative,dependent:copy(r.dependent)}));}
    function rebuildRows() {
        const next=[];for(const r of records)if(r.seam)next.push(r.rows[1],r.rows[2]);else if(r.representative===null&&r.dependent===null)next.push(r.rows[0]);
        const indices=next.map(r=>r.forceIndex),changed=!same(indices,rowForceIndices);
        rows=next;rowForceIndices=indices;if(changed)structureVersion++;return changed;
    }
    rebuildRows();
    function invalidate() {certificate=lastRefresh=null;for(const p of nodalForces.values())p.forEach(v=>v.fill(0));for(const r of records)for(const row of r.rows){row.geometricTangentValid=false;row.residual=NaN;row.jacobian.fill(NaN);row.forceColumn.fill(NaN);row.geometricTangent.fill(NaN);}}
    function verifyContract(checkForces=true) {
        if(pointLaw&&JSON.stringify({scheme:wall.pressureDiscretization,sites:wall.pressureSites})!==pressureSignature)throw failure('Frozen nodal pressure law or one-sided surface sites changed');
        if((wall.contactUpdate??'frozen-chart')!==contactUpdate||(wall.contactMode??'capsule')!==contactMode||wall.field!==field||wall.mode!=='wall-normal'||wall.friction!=='none'||wall.chartId!==chartId||wall.forcePerLength!==k||
            chartSignature(layout,coordinates,modes,relativeToolId,wall.contactOwners)!==chart||sourceSignature(field)!==source)throw failure('Frozen wall source/chart/ownership changed');
        if(checkForces&&!normalForces.every(Number.isFinite))throw failure('Nonfinite private wall force');
    }
    function geometryValues(toolPositions) {
        if(!(toolPositions instanceof Map))throw failure('Own physical toolPositions Map is required');
        const values=[];for(const [owner] of collectors){const p=toolPositions.get(owner);if(p?.length!==layout.nodeCount)throw failure('Missing physical owner geometry');for(const v of p)values.push(...vector(v,3,'Owner physical position'));}
        return values;
    }
    function requireCurrentRaw(toolPositions) {
        if(!lastQuery||!same(geometryValues(toolPositions),lastQuery.values)||!same(geometryValues(lastQuery.toolPositions),lastQuery.values))throw failure('A matching original wall query is required; raw geometry is stale');
    }
    function point(r,positions) {const t=material?r.fraction:r.role==='proximal'?0:r.role==='distal'?1:r.raw.rawContact.segmentT,a=positions[r.edge],b=positions[r.edge+1];return a.map((v,i)=>t===0?v:t===1?b[i]:v+(b[i]-v)*t);}
    function provenance(r,positions) {
        const raw=r.raw.rawContact,p=point(r,positions);
        return {role:r.role,endpointFraction:r.raw.t,source:raw.source,sampleFraction:pointLaw?null:raw.segmentT,sampleCount:raw.capsuleSampleCount,faceIndex:raw.faceIndex??-1,
            branchSign:Math.sign(raw.signedDistance),cell:raw.source==='sparse-sdf'&&field.sdfOrigin?p.map((v,i)=>Math.floor((v-field.sdfOrigin[i])/field.voxelSize)):null};
    }
    function admit(r,face,toolPositions) {
        if(r.seam||r.raw.source!=='sparse-sdf'||(normalForces[r.base]===0&&r.raw.gap>0))return false;
        const ws=createCompositeWallSdfBranchesWorkspace(2),p=toolPositions.get(r.owner),chart=evaluateCompositeWallSdfBranches({field,face,positions:[p[r.edge],p[r.edge+1]],radius:r.radius,contact:r.raw.rawContact,dofs:r.raw.dofs},ws);
        if(!chart.supported){r.discoveryFailure={reason:chart.reason,face:{...face}};return false;}
        if(normalForces[r.base+1]!==0||normalForces[r.base+2]!==0)throw failure('Unused SDF force slots must be exactly zero');
        r.seam={face:{axis:face.axis,gridIndex:face.gridIndex},sampleFraction:r.raw.rawContact.segmentT,sampleCount:r.raw.rawContact.capsuleSampleCount,workspace:ws};
        normalForces[r.base+1+chart.selectedRow]=normalForces[r.base];normalForces[r.base]=0;r.representative=null;stats.discoveries++;return true;
    }
    function discover(toolPositions,deltaPhysical=null) {
        let changed=false;
        // Current-query mode retains the original detector's single selected
        // contact normal. It rebuilds its cell derivative at every trial; it
        // does not add the auxiliary two-ray frozen-chart representation.
        if(contactUpdate==='current-query'||contactMode!=='capsule'||!field.sdfOrigin)return false;
        if(deltaPhysical!==null&&!(deltaPhysical instanceof Map))throw failure('Physical trial displacement Map is required');
        for(const r of records)if(!r.seam&&r.raw.source==='sparse-sdf'&&(normalForces[r.base]!==0||r.raw.gap<=0)) {
            const p=toolPositions.get(r.owner),t=r.raw.rawContact.segmentT,delta=deltaPhysical===null?[0,0,0]:(()=>{
                const d=deltaPhysical.get(r.owner);if(d?.length!==layout.nodeCount)throw failure('Physical displacement must cover each wall owner');
                const a=vector(d[r.edge],3,'Physical displacement'),b=vector(d[r.edge+1],3,'Physical displacement');return a.map((v,i)=>(1-t)*v+t*b[i]);})();
            const crossing=findCompositeWallSdfSeamCrossing({field,position:point(r,p),delta});
            if(crossing.hit&&crossing.supported&&(deltaPhysical!==null||crossing.startsOnFace))changed=admit(r,crossing.events[0],toolPositions)||changed;
        }
        return changed;
    }
    function discoverCharts({deltaPhysical=null}={}) {
        if(busy)throw failure('Joint wall adapter is busy');busy=true;
        try {
            verifyContract();if(!lastQuery)throw failure('Discover requires an original query');requireCurrentRaw(lastQuery.toolPositions);
            const changed=discover(lastQuery.toolPositions,deltaPhysical);if(changed){invalidate();rebuildRows();}
            return {rowStructureChanged:changed,structureVersion};
        } catch(error){invalidate();rebuildRows();throw error;} finally{busy=false;}
    }
    function refresh({toolPositions,commonResidual,relativeResidual,order='full',consumeQuery,query=true}) {
        if(busy)throw failure('Joint wall adapter is busy');busy=true;invalidate();
        try {
            verifyContract();if(order!=='full'&&order!=='gradient')throw failure('Wall order must be full or gradient');
            vector(commonResidual,layout.dofCount,'Common residual');vector(relativeResidual,3*modes.length,'Relative residual');
            const values=geometryValues(toolPositions),beforeStructure=rowForceIndices.slice();
            if(query) {
                if(typeof consumeQuery!=='function')throw failure('Every original wall query needs a budget callback');
                lastQuery=null;
                const queryField=Object.create(field);Object.defineProperty(queryField,'queryCapsuleCoordinates',{value:(...args)=>{consumeQuery();stats.queries++;return field.queryCapsuleCoordinates(...args);}});
                for(const [owner,c] of collectors)(contactMode==='capsule'?refreshCompositeWallContacts:refreshCompositeWallEnvelope)({positions:toolPositions.get(owner),contactOwners:c.owners,field:queryField},c.workspace);
                if(material) {
                    for(const r of records)if(!r.branchHelper){const p=toolPositions.get(r.owner);r.fixedPoint.refresh({field:queryField,positions:[p[r.edge],p[r.edge+1]],radius:r.radius});}
                    for(const entry of pointBranchHelpers.values()){
                        const s=entry.site,p=toolPositions.get(s.owner),result=entry.helper.refresh({field:queryField,positions:[p[s.edge],p[s.edge+1]],radius:entry.radius,...s.sdfSeam});
                        if(!result.supported)throw failure(`Unsupported fixed-point branch: ${result.reason}`);
                    }
                }
                lastQuery={values,toolPositions,generation:++rawGeneration};
            } else {requireCurrentRaw(toolPositions);stats.derivativeRefreshes++;}
            stats.refreshes++;discover(toolPositions);
            for(const r of records) {
                const raw=r.raw.rawContact,p=toolPositions.get(r.owner),next=provenance(r,p),loaded=Array.from({length:slotCount},(_,j)=>j).some(j=>normalForces[r.base+j]!==0);
                finite(raw.signedDistance,'Original signed distance');finite(raw.signedGap,'Original gap');
                const currentSource=contactUpdate==='current-query'&&['analytic-plane','sparse-sdf','sparse-sdf-bvh'].includes(next.source)&&
                    (r.branchHelper||r.raw.derivativeUnavailable===false&&r.raw.derivativeSource===next.source)&&r.raw.source===next.source;
                if(r.provenance&&loaded&&!currentSource&&JSON.stringify({...next,cell:null})!==JSON.stringify({...r.provenance,cell:null}))throw failure('Loaded wall source/sample/feature changed without traction transfer');
                // An implicit current-rate contact has no stored finite-slip anchor.
                // Its same normal unknown may follow a fresh, supported cell
                // derivative. The same source provider and supported current derivative are checked above;
                // the current residual and final gap/force certificate remain exact.
                if(r.provenance&&loaded&&!r.seam&&!currentSource&&JSON.stringify(next.cell)!==JSON.stringify(r.provenance.cell))throw Object.assign(failure('Loaded SDF cell changed without discoverCharts'),{details:{discoveryFailure:r.discoveryFailure??null,previousCell:r.provenance.cell,nextCell:next.cell,source:next.source,derivativeUnavailable:r.raw.derivativeUnavailable,derivativeReason:r.raw.derivativeReason,derivativeSource:r.raw.derivativeSource,derivativeDetails:r.raw.derivativeDetails??null}});
                if(r.seam) {
                    if(normalForces[r.base]!==0)throw failure('Ordinary force duplicates its SDF cone');
                    if(raw.source!=='sparse-sdf'||raw.segmentT!==r.seam.sampleFraction||raw.capsuleSampleCount!==r.seam.sampleCount)throw failure('SDF cone source/sample changed');
                    const chart=evaluateCompositeWallSdfBranches({field,face:r.seam.face,positions:[p[r.edge],p[r.edge+1]],radius:r.radius,contact:raw,dofs:r.raw.dofs},r.seam.workspace);
                    if(!chart.supported)throw failure(`Unsupported SDF cone chart: ${chart.reason}`);
                } else if(slotCount===3&&(normalForces[r.base+1]!==0||normalForces[r.base+2]!==0))throw failure('Unowned cone force slots are nonzero');
                r.provenance=next;r.representative=null;r.dependent=null;
            }
            // The ORIGINAL capsule remains a measured inequality. Remove its
            // dual only after the existing exact g/G/B/DB combination proof.
            // Signed private forces transfer linearly, never by clipping. Both
            // endpoint constraints survive a subsequent zero-load capsule
            // sample switch; the loaded provenance guard above is unchanged.
            for(const [a,b,c] of envelopes)if(!preserveSampleReactions&&isCompositeWallEndpointCombination(a.raw,b.raw,c.raw)) {
                const weights=[1-c.raw.t,c.raw.t],Fn=normalForces[c.base];
                normalForces[a.base]=finite(normalForces[a.base]+weights[0]*Fn,'Proximal envelope force');
                normalForces[b.base]=finite(normalForces[b.base]+weights[1]*Fn,'Distal envelope force');normalForces[c.base]=0;
                c.dependent={kind:'endpoint-combination',endpoints:[a.index,b.index],weights};
            }
            // Existing exact g/G/B/DB proof, only for duplicate endpoint loads
            // of the SAME physical owner and radius. Signed trial sums remain
            // private; no sign clipping or approximate dependence is used.
            // Normal-only dependence is not a proof that two Coulomb
            // contacts share a surface lever, spin or friction capacity.
            // Keep each coupled Fn/Ft owner until a full wrench reduction
            // exists; the joint solve retains its exact final force gates.
            const endpoints=new Map();
            for(const r of records)if(!preserveSampleReactions&&!pointLaw&&!r.seam&&r.dependent===null&&!r.raw.derivativeUnavailable&&(r.raw.t===0||r.raw.t===1)) {
                const key=JSON.stringify([r.owner,r.edge+r.raw.t,r.radius]),previous=endpoints.get(key);
                if(previous&&same(Array.from(previous.raw.normal),Array.from(r.raw.normal))&&same(Array.from(previous.raw.closestPoint),Array.from(r.raw.closestPoint))&&equalCompositeWallGlobalDifferentials(previous.raw,r.raw,layout)) {
                    normalForces[previous.base]=finite(normalForces[previous.base]+normalForces[r.base],'Combined duplicate force');normalForces[r.base]=0;r.representative=previous.index;
                } else endpoints.set(key,r);
            }
            rebuildRows();
            const common=new Float64Array(layout.dofCount),relative=new Float64Array(3*modes.length);
            for(const p of nodalForces.values())p.forEach(v=>v.fill(0));
            let minGap=Infinity,minForce=Infinity,penetration=0,negativeForce=0,ncp=0,complementarity=0,merit=0,domainAdmissible=true;
            const proofs=[];
            function append(r,physical,row,Fn,eliminated=false) {
                const g=physical.gap,active=Fn-k*g>0;row.residual=active?g:Fn/k;row.multiplierDerivative=active?0:1/k;row.geometricTangentValid=order==='full';
                if(eliminated){row.jacobian.fill(0);row.forceColumn.fill(0);if(order==='full')row.geometricTangent.fill(0);}
                else {
                    const pulled=pullbackCompositeContact(physical,r.pulled);
                    for(let j=0;j<row.jacobian.length;j++){row.jacobian[j]=active?pulled.gapJacobian[j]:0;row.forceColumn[j]=pulled.forceColumn[j];}
                    if(order==='full')for(let j=0;j<row.geometricTangent.length;j++)row.geometricTangent[j]=-Fn*pulled.normalDerivative[j];
                    row.commonDofs.forEach((d,j)=>common[d]+=Fn*row.forceColumn[j]);row.relativeDofs.forEach((d,j)=>relative[d]+=Fn*row.forceColumn[row.commonDofs.length+j]);
                    r.points.forEach((p,i)=>physical.normalForceColumn.slice(3*i,3*i+3).forEach((v,j)=>nodalForces.get(p.toolId)[p.node][j]+=Fn*v));
                }
                const gapViolation=Math.max(0,-g),forceViolation=Math.max(0,-Fn),residual=Math.abs(row.residual),work=Math.abs(Fn*g);
                minGap=Math.min(minGap,g);minForce=Math.min(minForce,Fn);penetration=Math.max(penetration,gapViolation);negativeForce=Math.max(negativeForce,forceViolation);
                ncp=Math.max(ncp,residual);complementarity=Math.max(complementarity,work);
                merit+=(gapViolation/tol.wallGap)**2+(forceViolation/tol.force)**2+(residual/tol.wallNcp)**2+(work/tol.wallWork)**2;
                proofs.push({key:r.key,owner:r.owner,edge:r.edge,role:r.role,querySampleFraction:r.raw.rawContact.segmentT,querySampleCount:r.raw.rawContact.capsuleSampleCount,endpointFraction:r.raw.t,dependent:copy(r.dependent),forceIndex:row.forceIndex,source:r.raw.source,sdfBranch:row.sdfBranch,Fn,gap:g,ncp:residual,complementarity:work,active,eliminated,representative:r.representative});
            }
            for(const r of records) {
                if(r.branchHelper) {
                    const b=r.branchHelper,raw=b.helper.branches[r.branchIndex],physical=r.ordinaryGeometry;
                    physical.gap=raw.gap;physical.gapJacobian=raw.gapJacobian;physical.forceColumn=raw.forceColumn;physical.normalDerivative=raw.normalDerivative;
                    for(let i=0;i<6;i++)physical.normalForceColumn[i]=-raw.forceColumn[i];
                    const proof=b.helper.measure({forces:b.records.map(x=>normalForces[x.base]),penalty:k,gapTolerance:tol.wallGap,forceTolerance:k*tol.wallNcp,workTolerance:tol.wallWork});
                    domainAdmissible=domainAdmissible&&proof.domainAdmissible&&proof.originalGapMatches;
                    append(r,physical,r.rows[0],normalForces[r.base]);
                } else if(r.seam) {
                    const chart=r.seam.workspace,forces=normalForces.subarray(r.base+1,r.base+3),proof=measureCompositeWallSdfBranches(chart,{forces,penalty:k,gapTolerance:tol.wallGap,forceTolerance:k*tol.wallNcp,workTolerance:tol.wallWork});
                    domainAdmissible=domainAdmissible&&proof.domainAdmissible&&proof.originalGapMatches;
                    for(let j=0;j<2;j++)append(r,{...chart.rows[j],supported:true},r.rows[j+1],forces[j]);
                } else {
                    const physical=r.ordinaryGeometry,raw=r.raw,Fn=normalForces[r.base];
                    physical.gap=raw.gap;physical.gapJacobian=raw.gapJacobian;physical.forceColumn=raw.forceColumn;physical.normalDerivative=raw.normalDerivative;
                    for(let i=0;i<6;i++)physical.normalForceColumn[i]=-raw.forceColumn[i];
                    const supported=!raw.derivativeUnavailable&&raw.derivativeSource===raw.source&&[physical.gapJacobian,physical.forceColumn,physical.normalDerivative].every(v=>v.every(Number.isFinite));
                    const eliminated=!supported&&Fn===0&&raw.gap>0;
                    if(!supported&&!eliminated)throw failure(`Unsupported active/loaded wall derivative: ${raw.source}; ${raw.derivativeReason}`);
                    append(r,physical,r.rows[0],Fn,eliminated);
                }
            }
            // In the nodal law capsule pressure was absent from the INITIAL
            // discretization. Its original inequality, and both endpoint
            // inequalities from every owned edge, are still mandatory. A
            // curved wall can violate an interior gap with open endpoints:
            // report that failure for geometric refinement, never certify it.
            let coverageMerit=0;
            const originalInequalities=[];
            if(pointLaw)for(const [owner,c] of collectors)for(const raw of c.workspace.rows)if(raw.included) {
                const gap=finite(raw.gap,'Original nodal coverage gap'),violation=Math.max(0,-gap);
                minGap=Math.min(minGap,gap);penetration=Math.max(penetration,violation);coverageMerit+=(violation/tol.wallGap)**2;
                originalInequalities.push({owner,edge:raw.edge,role:raw.role,endpointFraction:raw.t,querySampleFraction:raw.rawContact.segmentT,
                    source:raw.source,gap,pressureDof:raw.role==='capsule'?false:'one-declared-node-site'});
            }
            merit+=coverageMerit;
            if(!Number.isFinite(merit)||!common.every(Number.isFinite)||!relative.every(Number.isFinite))throw failure('Nonfinite original wall certificate');
            common.forEach((v,i)=>commonResidual[i]+=v);relative.forEach((v,i)=>relativeResidual[i]+=v);
            certificate={scope:'joint-original-wall-normal-rows-only',contactMode,minGap,minForce,penetration,negativeForce,ncp,complementarity,domainAdmissible,
                samples:proofs,merit,retainedRows:rows.length,converged:normalForces.every(v=>v>=0)&&penetration<=tol.wallGap&&ncp<=tol.wallNcp&&complementarity<=tol.wallWork&&domainAdmissible,
                rowStructureChanged:!same(beforeStructure,rowForceIndices),structureVersion,queryGeneration:lastQuery.generation};
            if(pointLaw)Object.assign(certificate,{pressureDiscretization:material?'fixed-material-points':nodalPressureScheme,originalInequalities,coverageMerit});
            lastRefresh={generation:lastQuery.generation,forces:normalForces.slice(),queried:query};return copy(certificate);
        } catch(error){invalidate();throw error;} finally{busy=false;}
    }
    function checkpoint() {
        if(busy)throw failure('Joint wall adapter is busy');verifyContract();
        const token=Object.freeze({scope:'joint-wall-checkpoint'});checkpoints.set(token,{forces:normalForces.slice(),records:metadata()});return token;
    }
    function restore(token) {
        if(busy)throw failure('Joint wall adapter is busy');verifyContract(false);const saved=checkpoints.get(token);if(!saved)throw failure('Foreign or invalid wall checkpoint');
        normalForces.set(saved.forces);records.forEach((r,i)=>{const old=saved.records[i];r.provenance=copy(old.provenance);r.seam=old.seam===null?null:makeSeam(old.seam);r.representative=old.representative;r.dependent=copy(old.dependent);});
        lastQuery=null;invalidate();stats.restores++;return {rowStructureChanged:rebuildRows(),structureVersion};
    }
    function commit() {
        if(busy)throw failure('Joint wall adapter is busy');verifyContract();
        if(!certificate?.converged||!lastRefresh?.queried||lastRefresh.generation!==lastQuery?.generation||!same(Array.from(normalForces),Array.from(lastRefresh.forces)))throw failure('Commit needs a fresh original query and unchanged certified wall forces');
        requireCurrentRaw(lastQuery.toolPositions);
        return {signature,friction:'none',contactMode,preserveSampleReactions,normalForces:normalForces.slice(),records:metadata(),fieldToken:identity(field),fieldTokenScope:'current-process-and-module-instance'};
    }
    function assertCurrentSurface({toolPositions}) {
        if(busy)throw failure('Joint wall adapter is busy');verifyContract();requireCurrentRaw(toolPositions);
        if(!lastRefresh||lastRefresh.generation!==lastQuery.generation||!same(Array.from(normalForces),Array.from(lastRefresh.forces)))
            throw failure('Surface assembly requires a successful current wall refresh and unchanged normal forces');
        return {generation:lastRefresh.generation,structureVersion};
    }
    return {signature,normalForces,nodalForces,refresh,discoverCharts,checkpoint,restore,commit,assertCurrentSurface,surfaceRecords,
        get rows(){return rows.slice();},get rowForceIndices(){return rowForceIndices.slice();},get diagnostics(){return {...stats,contactMode,structureVersion,retainedRows:rows.length};}};
}
