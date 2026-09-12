import {compositeJointWallSourceSignature} from './kirchhoffCompositeJointWallRows.js';
import {createCompositeDiscreteWallPointBranches} from './compositeDiscreteWallPointBranches.js';

const preparations=new WeakMap();
const fail=(message)=>{const e=new RangeError(message);e.code='joint-world-wall-adapter-required';throw e;};
const positive=(n,name)=>{if(!Number.isFinite(n)||n<=0)fail(`${name} must be finite and positive`);return n;};
const coefficient=(n,name)=>{if(!Number.isFinite(n)||n<0)fail(`${name} must be finite and nonnegative`);return n;};
const signature=value=>JSON.stringify(value);
const vector=(v,name)=>{if(v?.length!==3||Array.from(v).some(x=>!Number.isFinite(x)))fail(`${name} needs three finite components`);return Array.from(v);};
const equal=(a,b)=>a.length===b.length&&a.every((v,i)=>v===b[i]);
const close=(a,b)=>Number.isFinite(a)&&Number.isFinite(b)&&Math.abs(a-b)<=64*Number.EPSILON*(Math.abs(a)+Math.abs(b));

function sourceSnapshot(world,state,bindings) {
    if(!world.contactField||typeof world.contactField.queryCapsuleCoordinates!=='function')fail('The original World capsule query provider is required');
    if(bindings.length!==world.bodies.length||bindings.length!==state.tools.length)fail('Wall source bindings must cover every actual World body');
    const seen=new Set(),ids=new Set();
    return bindings.map(binding=>{
        const {body,toolId}=binding;
        if(!world.bodies.includes(body)||seen.has(body)||ids.has(toolId)||!state.layout.spins.has(toolId))fail('Wall source bindings need distinct actual physical tools');
        seen.add(body);ids.add(toolId);
        const {activeStart:start,activeEnd:end,collisionStartSegment:collisionStart,collisionEndSegment:collisionEnd}=body;
        if(![start,end,collisionStart,collisionEnd,body.count,body.segmentCount].every(Number.isInteger)||start<0||end>=body.count||start>=end||body.segmentCount!==body.count-1)
            fail('Wall source ranges must identify actual physical body segments');
        const nodes=binding.nodes.slice().sort((a,b)=>a.node-b.node);
        if(nodes.length!==end-start+1||nodes.some((r,i)=>r.node!==start+i||!Number.isInteger(r.jointNode)||r.jointNode<0||r.jointNode>=state.layout.nodeCount||i&&r.jointNode<=nodes[i-1].jointNode))
            fail('Wall source needs ordered complete original-node bindings');
        if(body.nodeRadius?.length!==body.count)fail('Wall source radii must cover actual body nodes');
        const ownEdges=state.layout.edgeToolIds.flatMap((ids,e)=>ids.includes(toolId)?[e]:[]);
        if(ownEdges.length!==nodes.at(-1).jointNode-nodes[0].jointNode||ownEdges.some((e,i)=>e!==nodes[0].jointNode+i))
            fail('Every own Joint edge must lie in a covered original physical interval');
        const segments=nodes.slice(0,-1).map((r,i)=>({bodyEdge:r.node,jointStart:r.jointNode,jointEnd:nodes[i+1].jointNode,
            radius:Math.max(positive(body.nodeRadius[r.node],'Source node radius'),positive(body.nodeRadius[r.node+1],'Source node radius')),
            exposed:r.node>=Math.max(start,collisionStart,0)&&r.node<Math.min(end,collisionEnd+1,body.segmentCount)}));
        const exposed=segments.some(s=>s.exposed);
        const muStatic=exposed?coefficient(body.wallStaticFriction,'Source static friction'):null,
            muKinetic=exposed?coefficient(body.wallKineticFriction,'Source kinetic friction'):null;
        if(exposed&&muStatic<muKinetic)fail('The source static friction cone must contain its kinetic cone');
        return {toolId,bodyId:body.id,count:body.count,start,end,collisionStart,collisionEnd,muStatic,muKinetic,segments};
    });
}

function initialContinuousRates({state,bindings,sites,inertia}) {
    if(state.continuousRateHistory)return [];
    if(state.step!==0)fail('A new continuous wall contact needs owned incoming material angular-rate history');
    const rates=new Map();
    for(const owner of new Set(sites.map(s=>s.owner))) {
        const body=bindings.find(b=>b.toolId===owner)?.body,velocity=vector(['velocityX','velocityY','velocityZ'].map(k=>body?.[k]?.[body.activeStart]),'Actual initial material velocity'),
            angularVelocity=vector(['angularVelocityX','angularVelocityY','angularVelocityZ'].map(k=>body?.[k]?.[body.activeStart]),'Actual initial angular velocity');
        for(let node=body.activeStart;node<=body.activeEnd;node++)if(['velocityX','velocityY','velocityZ'].some((k,c)=>body[k][node]!==velocity[c]))fail('Nonuniform initial continuous material velocity needs an explicit label-rate adapter');
        for(let edge=body.activeStart;edge<body.activeEnd;edge++)if(['angularVelocityX','angularVelocityY','angularVelocityZ'].some((k,c)=>body[k][edge]!==angularVelocity[c]))fail('Nonuniform initial continuous angular velocity needs an explicit label-rate adapter');
        const prepared=inertia?.inertiaEdges?.flatMap(e=>e.tools.filter(t=>t.id===owner));
        if(!prepared?.length)fail('Initial continuous wall friction needs actual prepared material velocity');
        for(const t of prepared) {
            const values=t.oldVelocityPieces?.flatMap(p=>p.bernsteinVelocities??p.oldMaterialVelocities)??t.oldMaterialVelocities;
            if(!Array.isArray(values)||!values.length||values.some(v=>!equal(vector(v,'Initial prepared material velocity'),velocity)))fail('Initial prepared material velocity disagrees with actual source motion');
        }
        rates.set(owner,{velocity,angularVelocity});
    }
    return sites.map(s=>({sampleId:s.id,owner:s.owner,...rates.get(s.owner),interpretation:'actual-uniform-initial-material-rate'}));
}

function assertInitialRest(inertia,bindings,owners) {
    for(const owner of owners) {
        const binding=bindings.find(b=>b.toolId===owner),body=binding.body;
        for(const key of ['angularVelocityX','angularVelocityY','angularVelocityZ']) {
            if(body[key]?.length!==body.segmentCount)fail('Initial two-branch wall motion needs actual material angular velocity');
            for(let edge=body.activeStart;edge<body.activeEnd;edge++)if(body[key][edge]!==0)
                fail('Moving initial angular motion needs an explicit material surface velocity adapter');
        }
        for(const key of ['velocityX','velocityY','velocityZ']) {
            if(body[key]?.length!==body.count)fail('Initial two-branch wall motion needs actual material node velocity');
            for(let node=body.activeStart;node<=body.activeEnd;node++)if(body[key][node]!==0)
                fail('Moving initial material needs an explicit tangential surface velocity adapter');
        }
        if(!Array.isArray(inertia?.inertiaEdges)||!inertia.inertiaEdges.length)fail('Initial two-branch wall motion needs prepared old material velocities');
        let found=false;
        for(const e of inertia.inertiaEdges)for(const t of e.tools??[])if(t.id===owner) {
            found=true;
            const pairs=t.oldVelocityPieces?.map(p=>p.bernsteinVelocities??p.oldMaterialVelocities)??[t.oldMaterialVelocities];
            if(!pairs.length||pairs.some(pair=>![2,6].includes(pair?.length)||pair.some(v=>v?.length!==3||Array.from(v).some(x=>x!==0))))
                fail('Moving initial material needs an explicit tangential surface velocity adapter');
        }
        if(!found)fail('Initial two-branch wall motion must cover each exposed tool');
    }
}

function assertPreparedAffineVelocity(prepared,velocities) {
    if(prepared.oldVelocityPieces===undefined) {
        if(prepared.oldMaterialVelocities?.length!==2||prepared.oldMaterialVelocities.some((v,end)=>!equal(vector(v,'Prepared old material velocity'),velocities[end])))
            fail('Prepared inertia disagrees with the initial imported material velocity');
        return;
    }
    const pieces=prepared.oldVelocityPieces;if(!Array.isArray(pieces)||!pieces.length||prepared.oldMaterialVelocities!==undefined)
        fail('Prepared inertia needs one explicit representation of the initial material velocity');
    let end=0;
    for(const piece of pieces) {
        const fractions=piece.fractions,controls=piece.bernsteinVelocities??piece.oldMaterialVelocities;
        if(fractions?.length!==2||fractions[0]!==end||!(fractions[1]>end)||fractions[1]>1||![2,6].includes(controls?.length))
            fail('Prepared inertia must cover the full initial material interval');
        controls.forEach((v,j)=>{const f=fractions[0]+(fractions[1]-fractions[0])*j/(controls.length-1),expected=velocities[0].map((a,k)=>a+f*(velocities[1][k]-a));
            if(vector(v,'Prepared old material velocity').some((a,k)=>a!==expected[k]&&!close(a,expected[k])))
                fail('Prepared inertia disagrees with the initial imported material velocity');});
        end=fractions[1];
    }
    if(end!==1)fail('Prepared inertia must cover the full initial material interval');
}

function initialIncomingMotion({state,inertia,bindings,snapshots,owners,sites}) {
    const fields=new Map(),rest=[];
    for(const owner of owners) {
        const source=snapshots.find(s=>s.toolId===owner),body=bindings.find(b=>b.toolId===owner).body;
        const edges=source.segments.flatMap(s=>Array.from({length:s.jointEnd-s.jointStart},(_,i)=>({edge:s.jointStart+i,bodyEdge:s.bodyEdge})));
        const records=edges.map(({edge})=>state.materialVelocities?.[edge]?.tools?.find(t=>t.id===owner));
        if(records.every(t=>t?.angularVelocityInterpretation===undefined)) {rest.push(owner);continue;}
        for(const key of ['angularVelocityX','angularVelocityY','angularVelocityZ'])
            if(body[key]?.length!==body.segmentCount)fail('Initial material angular velocity must cover actual body edges');
        edges.forEach(({edge,bodyEdge},i)=>{
            const t=records[i],prepared=inertia?.inertiaEdges?.[edge]?.tools?.find(t=>t.id===owner),map=prepared?.materialMap,
                dx=state.coordinates[edge+1]-state.coordinates[edge];
            if(t?.interpretation!=='physical-material-velocity'||t.angularVelocityInterpretation!=='physical-material-angular-velocity')
                fail('Explicit initial physical material velocity and angular velocity must cover every own edge');
            const omega=vector(t.angularVelocity,'Initial material angular velocity'),actual=vector(['angularVelocityX','angularVelocityY','angularVelocityZ'].map(key=>body[key][bodyEdge]),'Actual source angular velocity');
            if(!equal(omega,actual))fail('Initial imported material angular velocity disagrees with the actual source');
            if(t.velocities?.length!==2||!prepared)fail('Initial material surface velocity needs both physical endpoint velocities');
            const velocities=t.velocities.map(v=>vector(v,'Initial material velocity'));
            if(!(t.sEnd>t.sStart)||!close(map?.sStart,t.sStart)||!close(map.sStart+map.dsDx*dx,t.sEnd)||
                !(map.dsDt===0||Array.isArray(map.dsDt)&&map.dsDt.length===2&&map.dsDt.every(v=>v===0)))
                fail('Initial source velocity adapter requires a fixed material chart; feed needs explicit incoming surface history');
            assertPreparedAffineVelocity(prepared,velocities);
            fields.set(signature([owner,edge]),{materialLabels:[t.sStart,t.sEnd],centerVelocities:velocities,angularVelocity:omega,
                interpretation:'physical-material-surface-velocity'});
        });
        // Original source-node rates remain observable throughout a retry;
        // union children do not authorize stale external velocity buffers.
        for(const s of source.segments)for(const [edge,end,node] of [[s.jointStart,0,s.bodyEdge],[s.jointEnd-1,1,s.bodyEdge+1]]) {
            const actual=vector(['velocityX','velocityY','velocityZ'].map(key=>body[key]?.[node]),'Actual source material velocity');
            if(!equal(actual,fields.get(signature([owner,edge])).centerVelocities[end]))fail('Initial imported material velocity disagrees with the actual source');
        }
    }
    assertInitialRest(inertia,bindings,rest);
    return sites.filter(s=>owners.includes(s.owner)).map(s=>({owner:s.owner,edge:s.edge,...(s.node===undefined?{}:{node:s.node}),...(s.fraction===undefined?{}:{fraction:s.fraction,node:s.fraction===0?s.edge:s.fraction===1?s.edge+1:undefined}),
        ...(fields.get(signature([s.owner,s.edge]))??{velocity:[0,0],interpretation:'physical-tangential-surface-velocity'})}));
}

/** Derive every exposed subcapsule from actual World ranges and ORIGINAL
 * endpoint radii. A union subdivision retains its parent's max radius; using
 * interpolated endpoint radii could silently shrink its collision envelope.
 * No sleeping/contact-active flag removes a physical inequality. The field
 * is borrowed, and the actual normal manager still owns all geometry queries.
 */
export function prepareCompositeJointWorldWall({world,state,bindings,forcePerLength=1,frictionForcePerLength=50,
    contactMode='nodal-endpoints',source='original-field',plane,localFaceIndices=[],surfacePosePaths,inertia,rateMode='finite-displacement',sdfSeams=[],seamUpdates='fixed'}={}) {
    positive(forcePerLength,'Wall numerical forcePerLength');positive(frictionForcePerLength,'Friction numerical forcePerLength');
    if(!['capsule','nodal-endpoints','material-points','continuous-samples'].includes(contactMode))fail('World wall requires capsule, nodal-endpoints or continuous-samples pressure');
    if(!['original-field','analytic-plane','sparse-sdf','sparse-sdf-bvh'].includes(source))fail('Unknown original wall geometry source');
    if(!['finite-displacement','backward-euler-grid'].includes(rateMode))fail('Unknown wall friction rate model');
    if(!Array.isArray(sdfSeams)||sdfSeams.length&&contactMode!=='material-points')fail('Declared SDF seams require material-point pressure');
    if(!['fixed','automatic'].includes(seamUpdates)||seamUpdates==='automatic'&&(contactMode!=='material-points'||rateMode!=='backward-euler-grid'))fail('Automatic seams require material-point backward-euler pressure');
    // Accepted two-ray rows retain their declared chart when no native remap
    // discarded the histories. Treat it as a proposal: query and re-prove it
    // below in this new preparation, never trust the old geometric proof.
    const seamRequests=sdfSeams.slice();
    if(seamUpdates==='automatic'&&state.wallFrictionState!==undefined){
        const history=state.wallFrictionState;
        if(history?.contactMode!=='material-points'||history.pressureDiscretization!=='fixed-material-points'||!Array.isArray(history.pressureSites))
            fail('Automatic seam recovery requires fixed material-point history');
        const recovered=new Map(),explicit=new Map();
        for(const request of sdfSeams){
            const key=signature([request?.owner,request?.edge,request?.fraction]);
            if(explicit.has(key))fail('Declared SDF seam needs one distinct existing material site');
            explicit.set(key,request);
        }
        for(const record of history.pressureSites){
            const branch=record?.[5];if(branch===undefined||branch===null)continue;
            const [owner,,edge,,fraction]=record,key=signature([owner,edge,fraction]);
            if(![0,1].includes(branch.branchIndex)||branch.fraction!==fraction||!Number.isInteger(edge)||typeof owner!=='string'||
                !Number.isFinite(fraction)||fraction<0||fraction>1)fail('Invalid accepted SDF branch descriptor');
            const sdfSeam=structuredClone({face:branch.face,domainBox:branch.domainBox}),previous=recovered.get(key);
            if(previous){
                if(signature(previous.sdfSeam)!==signature(sdfSeam)||previous.branches.has(branch.branchIndex))fail('Conflicting accepted SDF branch descriptors');
                previous.branches.add(branch.branchIndex);
            }else recovered.set(key,{owner,edge,fraction,sdfSeam,branches:new Set([branch.branchIndex])});
        }
        for(const [key,record] of recovered){
            if(record.branches.size!==2)fail('Accepted SDF seam requires both branch descriptors');
            const request=explicit.get(key);
            if(request){
                if(signature({face:request.sdfSeam?.face,domainBox:request.sdfSeam?.domainBox})!==signature(record.sdfSeam))
                    fail('Explicit SDF seam conflicts with accepted history');
            }else {const {branches,...request}=record;seamRequests.push(request);}
        }
    }
    const snapshots=sourceSnapshot(world,state,bindings);
    // A tool wholly in the introducer has no original exposed wall interval.
    // Retain an actual-source proof: the next source range change invalidates
    // it before physics or publication. This cannot hide an exposed capsule
    // or reinterpret a nonzero accepted wall reaction (the dt guard rejects it).
    if(!snapshots.some(s=>s.segments.some(segment=>segment.exposed))) {
        if(seamRequests.length)fail('Declared SDF seams require exposed material points');
        const proof=Object.freeze({scope:'actual-world-zero-exposed-capsules',originalCapsules:0,jointCapsules:0,overlappingExposedIntervals:0});
        preparations.set(proof,{world,state,bindings,snapshots:signature(snapshots),field:world.contactField,
            source:compositeJointWallSourceSignature(world.contactField),query:world.contactField.queryCapsuleCoordinates,
            empty:true,numeric:'none',numericSignature:value=>value,initialOwners:[]});
        return {wall:'none',proof};
    }
    const edges=Array.from({length:state.layout.nodeCount-1},(_,edge)=>({edge,walls:[]}));
    let originalCapsules=0;
    for(const s of snapshots)for(const segment of s.segments)if(segment.exposed) {
        originalCapsules++;
        for(let edge=segment.jointStart;edge<segment.jointEnd;edge++)edges[edge].walls.push({owner:s.toolId,radius:segment.radius,
            materialSegmentId:signature(['world-wall',s.toolId,s.bodyId,segment.bodyEdge,edge])});
    }
    const exposed=snapshots.filter(s=>s.segments.some(e=>e.exposed)),twoBranch=exposed.some(s=>s.muStatic!==s.muKinetic);
    if(contactMode==='continuous-samples') {
        if(!(state.inertiaGeometryByTool instanceof Map)||typeof world.contactField.querySphere!=='function')fail('Continuous pressure needs the same C2 material geometry and original point provider');
        const sites=new Map(),intervals=[];
        for(const s of snapshots)for(const segment of s.segments)if(segment.exposed) {
            const {jointStart:a,jointEnd:b,radius}=segment,p=state.toolPositions.get(s.toolId),
                length=Math.hypot(...p[b].map((v,k)=>v-p[a][k])),spacing=Math.max(positive(world.contactField.voxelSize,'Actual wall voxel size')*4,Math.max(.5,radius)),
                count=Math.max(1,Math.ceil(length/spacing)),x0=state.coordinates[a],x1=state.coordinates[b];
            intervals.push({owner:s.toolId,bodyEdge:segment.bodyEdge,jointStart:a,jointEnd:b,radius,sampleCount:count,spacing});
            // Query the complete original grid, including samples that the
            // old detector can skip when endpoint clearances agree. Union
            // cuts are additional actual material points, never substitutes.
            const coordinates=[...Array.from({length:count+1},(_,i)=>i===count?x1:x0+(x1-x0)*(i/count)),...Array.from(state.coordinates.slice(a,b+1))];
            for(const coordinate of coordinates) {
                let edge=a;while(edge<b-1&&coordinate>=state.coordinates[edge+1])edge++;
                const fraction=coordinate===state.coordinates[edge]?0:coordinate===state.coordinates[edge+1]?1:(coordinate-state.coordinates[edge])/(state.coordinates[edge+1]-state.coordinates[edge]),
                    key=signature([s.toolId,coordinate]),old=sites.get(key);
                if(!old||old.radius<radius)sites.set(key,{id:key,owner:s.toolId,edge,fraction,radius,coordinate});
            }
        }
        // Keep previous declared sites while still covered by the SAME
        // original source interval/radius. A changed detector sample count
        // must not discard a loaded accepted physical force.
        for(const record of state.wallContactState?.contactMode==='continuous-samples'?state.wallContactState.records:[]) {
            const p=record.site;if(sites.has(p.id))continue;
            if(intervals.some(i=>i.owner===p.owner&&i.radius===p.radius&&p.coordinate>=state.coordinates[i.jointStart]&&p.coordinate<=state.coordinates[i.jointEnd]))sites.set(p.id,{...p});
            else if(record.force!==0)fail('Loaded continuous wall ownership requires force transfer before range/radius change');
        }
        const frictionEnabled=exposed.some(s=>s.muStatic>0),pressureSites=Array.from(sites.values()).sort((a,b)=>a.owner.localeCompare(b.owner)||a.coordinate-b.coordinate),
            incoming=frictionEnabled?initialContinuousRates({state,bindings,sites:pressureSites,inertia}):[],
            friction=frictionEnabled?{law:'coulomb-static-kinetic',forcePerLength:frictionForcePerLength,rateMode:'backward-euler-grid',
                slipModel:'implicit-backward-euler-surface-rate',finiteStepSlipKnown:false,
                muByOwner:exposed.map(s=>({owner:s.toolId,muStatic:[s.muStatic,s.muStatic],muKinetic:[s.muKinetic,s.muKinetic]})),incomingSurfaceMotion:incoming}:'none',
            wall={mode:frictionEnabled?'wall-coulomb':'wall-normal',contactMode,friction,chartId:'actual-world-C2-original-resolution-pressure',field:world.contactField,forcePerLength,
            contactOwners:{edges},pressureSites,
            pressureDiscretization:'original-resolution-C2-material-samples',samplingIntervals:intervals},
            proof=Object.freeze({scope:'actual-world-original-resolution-C2-samples',originalCapsules,jointCapsules:edges.reduce((n,e)=>n+e.walls.length,0),samples:sites.size,
                continuumClearanceCertified:false}),numeric=({field,...rest})=>signature(rest);
        preparations.set(proof,{world,state,bindings,snapshots:signature(snapshots),field:world.contactField,
            source:compositeJointWallSourceSignature(world.contactField),query:world.contactField.queryCapsuleCoordinates,pointQuery:world.contactField.querySphere,
            numeric:numeric(wall),numericSignature:numeric,initialOwners:[],continuousIncoming:frictionEnabled?{sites:structuredClone(pressureSites),signature:signature(incoming)}:null});
        return {wall,proof};
    }
    const coefficients=exposed.map(s=>twoBranch?{owner:s.toolId,muStatic:[s.muStatic,s.muStatic],muKinetic:[s.muKinetic,s.muKinetic]}:{owner:s.toolId,mu:[s.muKinetic,s.muKinetic]});
    const frictionEnabled=exposed.some(s=>s.muStatic>0);
    const wall={mode:frictionEnabled?'wall-coulomb':'wall-normal',chartId:'actual-world-exposed-union-surfaces',field:world.contactField,
        contactMode,forcePerLength,contactOwners:{edges},...(seamUpdates==='automatic'?{seamUpdates}:{}),
        ...(rateMode==='backward-euler-grid'?{contactUpdate:'current-query'}:{}),
        friction:frictionEnabled?{law:twoBranch?'coulomb-static-kinetic':'coulomb',muByOwner:coefficients,forcePerLength:frictionForcePerLength,
            materialPath:'linear-affine-maps',motion:'stationary-material',source,tangentBasis:'projected-own-tangent',
            ...(rateMode==='backward-euler-grid'?{rateMode,slipModel:'implicit-backward-euler-surface-rate',finiteStepSlipKnown:false}:{}),
            ...(twoBranch&&state.nativeRateHistory?{incomingRateHistory:structuredClone(state.nativeRateHistory)}:{})}:'none',
        ...(plane===undefined?{}:{plane:structuredClone(plane)}),localFaceIndices:Array.from(localFaceIndices),
        ...(surfacePosePaths===undefined?{}:{surfacePosePaths:surfacePosePaths.map(({owner,edge,path})=>({owner,edge,path}))})};
    if(contactMode==='nodal-endpoints'||contactMode==='material-points') {
        wall.pressureDiscretization='nodal-endpoints-one-sided-surface';
        const sites=new Map();
        for(const e of edges)for(const w of e.walls)for(const node of [e.edge,e.edge+1]) {
            const key=signature([w.owner,node]),old=sites.get(key);
            if(old&&old.radius!==w.radius)fail('Radius discontinuity needs capsule pressure or a distinct physical surface trace');
            // Keep one explicit incident own edge, including at an exposure boundary.
            if(!old)sites.set(key,{owner:w.owner,node,edge:e.edge,trace:node===e.edge?'right':'left',radius:w.radius});
        }
        wall.pressureSites=Array.from(sites.values(),({radius,...site})=>site);
        if(contactMode==='material-points') {
            wall.pressureDiscretization='fixed-material-points';
            wall.pressureSites=wall.pressureSites.map(({node,...site})=>({...site,fraction:node-site.edge}));
            // Same spatial sampling scale as the original capsule detector.
            // Fractions are fixed for the prepared step, never its argmin.
            for(const e of edges)for(const w of e.walls) {
                const p=state.toolPositions.get(w.owner),length=Math.hypot(...p[e.edge+1].map((v,k)=>v-p[e.edge][k])),
                    spacing=Math.max(positive(world.contactField.voxelSize,'Original voxel size')*4,.5,w.radius),count=Math.max(1,Math.ceil(length/spacing));
                for(let i=1;i<count;i++)wall.pressureSites.push({owner:w.owner,edge:e.edge,fraction:i/count});
            }
        }
    }
    // Re-prove explicitly requested charts in THIS prepared configuration.
    // Ownership and radius still come solely from actual body coverage.
    // This makes a later same-dt restart eligible for a fresh World proof;
    // it does not itself transfer any accepted reaction or history.
    const declared=new Set();let preparationContactQueries=0;
    for(const request of seamRequests){
        const key=signature([request?.owner,request?.edge,request?.fraction]),site=wall.pressureSites?.find(s=>signature([s.owner,s.edge,s.fraction])===key);
        if(!site||declared.has(key))fail('Declared SDF seam needs one distinct existing material site');
        declared.add(key);
        const radius=edges[site.edge].walls.find(w=>w.owner===site.owner).radius,p=state.toolPositions.get(site.owner),
            sdfSeam=structuredClone({face:request.sdfSeam?.face,domainBox:request.sdfSeam?.domainBox}),helper=createCompositeDiscreteWallPointBranches({fraction:site.fraction,edge:site.edge}),
            result=helper.refresh({field:world.contactField,positions:[p[site.edge],p[site.edge+1]],radius,face:sdfSeam.face,domainBox:sdfSeam.domainBox,
                consumeQuery:()=>{preparationContactQueries++;}});
        if(!result.supported)fail(`Declared SDF seam is not proved in the prepared configuration: ${result.reason}`);
        site.sdfSeam=sdfSeam;
    }
    // Import explicit physical material translation AND angular velocity.
    // Projection waits for the normal manager's original wall witness; no
    // detector query or quasi-static angular-rate estimate is invented here.
    // Undeclared angular sources retain only the exact-rest special case.
    const initialOwners=twoBranch&&!state.nativeRateHistory&&state.step===0&&!state.wallFrictionState?exposed.filter(s=>s.muStatic!==s.muKinetic).map(s=>s.toolId):[],
        initialSites=initialOwners.length?(['nodal-endpoints','material-points'].includes(contactMode)?wall.pressureSites:edges.flatMap(e=>e.walls.map(w=>({owner:w.owner,edge:e.edge})))):null;
    if(initialOwners.length)wall.friction.incomingSurfaceMotion=initialIncomingMotion({state,inertia,bindings,snapshots,owners:initialOwners,sites:initialSites});
    const proof=Object.freeze({scope:'actual-world-exposed-capsule-coverage',originalCapsules,
        jointCapsules:edges.reduce((n,e)=>n+e.walls.length,0),overlappingExposedIntervals:edges.filter(e=>e.walls.length>1).length,
        ...(seamRequests.length?{declaredSdfSeams:seamRequests.length,preparationContactQueries}:{})});
    const numeric=({field,...rest})=>signature(rest);
    preparations.set(proof,{world,state,bindings,snapshots:signature(snapshots),field:world.contactField,
        source:compositeJointWallSourceSignature(world.contactField),query:world.contactField.queryCapsuleCoordinates,numeric:numeric(wall),numericSignature:numeric,
        initialOwners,initialSites:initialOwners.length?structuredClone(initialSites):null,initialMotion:signature(wall.friction.incomingSurfaceMotion)});
    return {wall,proof};
}

/** Verify the private preparation against actual bodies on every attempt and
 * immediately before publication. A supplied field identity or a caller-made
 * coverage report alone cannot authorize omitting an anatomy constraint.
 */
export function assertCompositeJointWorldWall({proof,world,state,bindings,wall,inertia}) {
    const p=preparations.get(proof);
    if(!p||p.world!==world||p.state!==state||p.bindings!==bindings||p.field!==world.contactField||(p.empty?wall!=='none':wall?.field!==p.field)||p.query!==p.field.queryCapsuleCoordinates||
        (p.pointQuery&&p.pointQuery!==p.field.querySphere)||compositeJointWallSourceSignature(world.contactField)!==p.source||signature(sourceSnapshot(world,state,bindings))!==p.snapshots||p.numericSignature(wall)!==p.numeric)
        {
            const changed=!p?'preparation':p.world!==world?'world':p.state!==state?'state':p.bindings!==bindings?'bindings':
                p.field!==world.contactField?'field':(p.empty?wall!=='none':wall?.field!==p.field)?'wall-field':
                p.query!==p.field.queryCapsuleCoordinates?'query':(p.pointQuery&&p.pointQuery!==p.field.querySphere)?'point-query':
                compositeJointWallSourceSignature(world.contactField)!==p.source?'field-source':
                signature(sourceSnapshot(world,state,bindings))!==p.snapshots?'source-ranges':'prepared-options';
            fail(`Prepared World wall source coverage, radius, friction or physical range changed (${changed})`);
        }
    if(p.initialOwners.length&&signature(initialIncomingMotion({state,inertia,bindings,snapshots:sourceSnapshot(world,state,bindings),owners:p.initialOwners,sites:p.initialSites}))!==p.initialMotion)
        fail('Prepared initial material surface velocity changed');
    if(p.continuousIncoming&&signature(initialContinuousRates({state,bindings,sites:p.continuousIncoming.sites,inertia}))!==p.continuousIncoming.signature)
        fail('Prepared initial continuous material rate changed');
    return proof;
}
