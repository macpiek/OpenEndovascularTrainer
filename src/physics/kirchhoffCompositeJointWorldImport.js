import { createCompositeChainLayout } from './kirchhoffCompositeChain.js';
import { createCompositeJointTimeStepState } from './kirchhoffCompositeJointTimeStep.js';
import { compileCompositeMaterial } from './kirchhoffCompositeElement.js';
import { materialFrameDirectors } from './discreteKirchhoffRod.js';

const finite=(x,name)=>{if(!Number.isFinite(x))throw new RangeError(`${name} must be finite`);return x;};
const positive=(x,name)=>{if(!(finite(x,name)>0))throw new RangeError(`${name} must be positive`);return x;};
const vector=(v,n,name)=>{if(v?.length!==n)throw new RangeError(`${name} needs ${n} entries`);return Array.from(v,x=>finite(x,name));};
const dot=(a,b)=>a.reduce((s,v,k)=>s+v*b[k],0),cross=(a,b)=>[a[1]*b[2]-a[2]*b[1],a[2]*b[0]-a[0]*b[2],a[0]*b[1]-a[1]*b[0]];
const freeze=x=>{if(x&&typeof x==='object'){Object.values(x).forEach(freeze);Object.freeze(x);}return x;};
const closeMetric=(a,b)=>Math.abs(a-b)<=64*Number.EPSILON*Math.max(Math.abs(a),Math.abs(b));
// Two rounded operations: rho = source - q, then source' = q + rho.
// A conservative IEEE-754 bound only, with a subnormal underflow allowance.
// Scale before summing so finite large operands do not overflow the bound.
const positionRoundoffBound=(q,rho,source)=>[q,rho,source].reduce((sum,v)=>sum+4*Number.EPSILON*Math.abs(v),8*Number.MIN_VALUE);
const identity=[Object.freeze([1,0,0]),Object.freeze([0,1,0]),Object.freeze([0,0,1])];
function fail(reason,details={}){const e=new RangeError(reason);e.code='joint-world-import-unsupported';e.details={reason,...details};throw e;}
function ownFrame(frame,tangent){const f={tangent:vector(frame?.tangent,3,'Own explicit tangent'),director:vector(frame?.director,3,'Own explicit director')};
    if(Math.abs(dot(f.tangent,f.tangent)-1)>1e-10||Math.abs(dot(f.director,f.director)-1)>1e-10||Math.abs(dot(f.tangent,f.director))>1e-10||Math.hypot(...f.tangent.map((x,k)=>x-tangent[k]))>1e-10)fail('explicit-reference-does-not-belong-to-source-edge');return f;
}
function transport(d,a,b){const den=1+dot(a,b);if(!(den>1e-10))fail('antiparallel-source-hinge-needs-another-chart');const v=cross(a,b),first=cross(v,d),second=cross(v,first);return d.map((x,k)=>x+first[k]+second[k]/den);}
function sourceRecord(record){
    const body=record?.body,toolId=record?.toolId,aliases={guidewire:'wire',wire:'wire',catheter:'catheter'};
    if(!body||!['wire','catheter'].includes(toolId)||aliases[body.id]!==toolId)fail('explicit-world-body-to-physical-tool-mapping-required',{bodyId:body?.id,toolId});
    const {count,activeStart:start,activeEnd:end}=body;
    if(!Number.isInteger(count)||count<2||body.segmentCount!==count-1||!Number.isInteger(start)||!Number.isInteger(end)||start<0||end>=count||end<=start)fail('invalid-own-active-physical-range',{bodyId:body.id,start,end});
    for(const key of ['x','y','z','velocityX','velocityY','velocityZ'])if(body[key]?.length!==count)throw new RangeError(`World ${key} must cover the actual body nodes`);
    for(const key of ['restLength','orientationX','orientationY','orientationZ','orientationW'])if(body[key]?.length!==count-1)throw new RangeError(`World ${key} must cover the actual body edges`);
    const size=end-start+1,x=vector(record.nodeCoordinates,size,'Explicit active own node coordinates');
    if(x.some((v,i)=>i&&!(v>x[i-1])))fail('own-node-coordinates-must-strictly-increase',{toolId});
    if(record.winding!=='explicit-unwrapped')fail('known-own-unwrapped-winding-required',{toolId});
    if(record.velocityInterpretation!=='physical-material-velocity')fail('explicit-source-physical-material-velocity-interpretation-required',{toolId});
    const angularKnown=record.angularVelocityInterpretation==='physical-material-angular-velocity';
    if(record.angularVelocityInterpretation!==undefined&&!angularKnown)fail('explicit-source-physical-material-angular-velocity-interpretation-required',{toolId});
    const angularVelocity=angularKnown?Array.from({length:size-1},(_,i)=>['angularVelocityX','angularVelocityY','angularVelocityZ'].map(key=>{
        if(body[key]?.length!==count-1)throw new RangeError(`World ${key} must cover the actual body edges`);
        return finite(body[key][start+i],'Active physical material angular velocity');
    })):null;
    if(record.reference?.length!==size-1)throw new RangeError('Explicit own reference frames must cover active source edges');
    const angles=vector(record.angles,size-1,'Own unwrapped source angles'),twists=vector(record.referenceTwists,size-2,'Own unwrapped source reference twists'),labels=vector(record.materialLabels,size,'Explicit active own material labels');
    const dsDx=positive(record.material?.dsDx,'Explicit material dsDx'),density=positive(record.material?.massPerMaterialLength,'Explicit mass per reference arclength'),provider=record.material?.materialAt;
    if(typeof provider!=='function')fail('explicit-compiled-material-profile-provider-required',{toolId});
    for(let i=0;i<size;i++)if(!closeMetric(labels[i],labels[0]+dsDx*(x[i]-x[0])))fail('source-labels-disagree-with-reference-arclength-map',{toolId,bodyNode:start+i});
    const positions=Array.from({length:size},(_,i)=>['x','y','z'].map(key=>finite(body[key][start+i],'Active body position'))),
        velocities=Array.from({length:size},(_,i)=>['velocityX','velocityY','velocityZ'].map(key=>finite(body[key][start+i],'Active physical material velocity')));
    const frames=[],rest=[],quaternions=[];let maxOrientationError=0,maxRestStorageError=0;
    for(let i=0;i<size-1;i++){
        const edge=start+i,dx=positive(x[i+1]-x[i],'Own source coordinate length'),expected=positive(dsDx*dx,'Own reference rest length'),actual=positive(body.restLength[edge],'Actual source rest length');
        const stored=body.restLength instanceof Float32Array?Math.fround(expected):expected;
        if(!Number.isFinite(stored)||(body.restLength instanceof Float32Array?actual!==stored:!closeMetric(actual,stored)))fail('source-rest-length-disagrees-with-reference-arclength',{toolId,bodyEdge:edge,sourceRestLength:actual,referenceRestLength:expected,expectedStoredRestLength:stored});
        maxRestStorageError=Math.max(maxRestStorageError,Math.abs(actual-expected));rest.push(actual);
        const chord=positions[i+1].map((v,k)=>v-positions[i][k]),length=positive(Math.hypot(...chord),'Source physical chord'),tangent=chord.map(v=>v/length),frame=ownFrame(record.reference[i],tangent);frames.push(frame);
        const q=Object.fromEntries(['x','y','z','w'].map(key=>[key,finite(body[`orientation${key.toUpperCase()}`][edge],'Actual source material quaternion')]));
        positive(Math.hypot(q.x,q.y,q.z,q.w),'Source quaternion norm');quaternions.push(q);
        // Verify the physical orientation only. A quaternion never supplies the
        // missing 2*pi lift: the explicit source angle/twist records do that.
        const d1=frame.director.map((v,k)=>v*Math.cos(angles[i])+cross(frame.tangent,frame.director)[k]*Math.sin(angles[i])),d2=cross(frame.tangent,d1),actualFrame=materialFrameDirectors(q,{});
        for(const [name,expectedDirector] of [['d1',d1],['d2',d2],['d3',frame.tangent]]){
            const error=Math.hypot(...['x','y','z'].map((axis,k)=>actualFrame[name][axis]-expectedDirector[k]));maxOrientationError=Math.max(maxOrientationError,error);
            if(error>1e-9)fail('body-orientation-disagrees-with-explicit-der-pose',{toolId,bodyEdge:edge,director:name,error});
        }
    }
    for(let i=0;i<twists.length;i++){
        const left=frames[i],right=frames[i+1],d=transport(left.director,left.tangent,right.tangent),phase=Math.atan2(dot(right.tangent,cross(d,right.director)),dot(d,right.director));
        if(Math.abs(Math.sin(twists[i]-phase))>1e-10||Math.abs(Math.cos(twists[i]-phase)-1)>1e-10)fail('source-reference-winding-disagrees-with-own-frames',{toolId,bodyNode:start+i+1});
    }
    return{body,bodyId:body.id,toolId,start,end,x,positions,velocities,angularVelocity,frames,angles,twists,labels,rest,quaternions,dsDx,density,provider,maxOrientationError,maxRestStorageError};
}
function sourceEdgeAt(source,x){
    if(x<source.x[0]||x>source.x.at(-1))fail('own-source-coordinate-extrapolation',{toolId:source.toolId,coordinate:x});
    let lo=0,hi=source.x.length-1;while(lo+1<hi){const mid=(lo+hi)>>>1;if(source.x[mid]<=x)lo=mid;else hi=mid;}return Math.min(lo,source.x.length-2);
}
function sample(source,x,field){
    const original=source.x.indexOf(x);if(original>=0)return{value:source[field][original].slice(),originalBodyNode:source.start+original,bodyEdge:source.start+Math.min(original,source.x.length-2),u:original===source.x.length-1?1:0};
    const e=sourceEdgeAt(source,x),u=(x-source.x[e])/(source.x[e+1]-source.x[e]);
    return{value:source[field][e].map((v,k)=>v+u*(source[field][e+1][k]-v)),originalBodyNode:null,bodyEdge:source.start+e,u};
}
function materialLabelAt(source,x){const node=source.x.indexOf(x);return node>=0?source.labels[node]:source.labels[0]+source.dsDx*(x-source.x[0]);}
function compiledSnapshot(raw){
    const k=vector(raw?.stiffness,9,'Compiled own profile stiffness'),intrinsic=vector(raw?.intrinsic,3,'Compiled own intrinsic strain');
    const m=compileCompositeMaterial({stiffness:[k.slice(0,3),k.slice(3,6),k.slice(6,9)],intrinsic,energyOffset:raw.energyOffset??0});
    return freeze({stiffness:Array.from(m.stiffness),intrinsic:Array.from(m.intrinsic),energyOffset:m.energyOffset});
}

/** Import the EXACT union of explicit own active source node coordinates.
 * Every new own edge is a restriction of ONE original affine source edge;
 * old hinges and their independent winding remain at their original locations.
 * This creates a fixed-chart initial state and a legacy publication VIEW map.
 * It does not advance a solver, mutate World bodies, transfer contact forces,
 * infer angular rates/winding, or coarsen later Joint physics back into bodies.
 */
export function importCompositeJointWorld({tools,time=0,step=0}={}){
    if(!Array.isArray(tools)||![1,2].includes(tools.length))throw new RangeError('One or two explicit actual World tool descriptors are required');
    const sources=tools.map(sourceRecord).sort((a,b)=>['wire','catheter'].indexOf(a.toolId)-['wire','catheter'].indexOf(b.toolId));
    if(new Set(sources.map(s=>s.body)).size!==sources.length||new Set(sources.map(s=>s.toolId)).size!==sources.length)fail('duplicate-physical-world-body-or-tool');
    const coordinates=[...new Set(sources.flatMap(s=>s.x))].sort((a,b)=>a-b),n=coordinates.length;
    if(n<3)fail('joint-import-requires-at-least-three-union-nodes');
    const edgeToolIds=coordinates.slice(0,-1).map((a,i)=>sources.filter(s=>a>=s.x[0]&&coordinates[i+1]<=s.x.at(-1)).map(s=>s.toolId));
    if(edgeToolIds.some(ids=>!ids.length)||edgeToolIds.some((ids,i)=>i&&!ids.some(id=>edgeToolIds[i-1].includes(id))))fail('disconnected-active-material-intervals');
    const layout=createCompositeChainLayout(edgeToolIds),ownPositions=new Map(),ownVelocities=new Map(),mappings=new Map(),angles=new Map(),restLengths=new Map(),materialSamples=new Map();
    for(const source of sources){
        const samples=coordinates.map(x=>x>=source.x[0]&&x<=source.x.at(-1)?sample(source,x,'positions'):null),velocities=coordinates.map(x=>x>=source.x[0]&&x<=source.x.at(-1)?sample(source,x,'velocities').value:null);
        ownPositions.set(source.toolId,samples.map(s=>s?.value??null));ownVelocities.set(source.toolId,velocities);
        const nodeMap=source.x.map((x,j)=>({node:source.start+j,jointNode:coordinates.indexOf(x),coordinate:x,trace:j===source.x.length-1?'left':'right'}));
        const jointEdges=[];
        for(let e=0;e<n-1;e++)if(edgeToolIds[e].includes(source.toolId)){
            const sourceEdge=sourceEdgeAt(source,coordinates[e]),a=source.x[sourceEdge],b=source.x[sourceEdge+1];
            if(coordinates[e+1]>b)fail('union-edge-crossed-an-original-own-hinge',{toolId:source.toolId,jointEdge:e});
            jointEdges.push({jointEdge:e,bodyEdge:source.start+sourceEdge,u0:coordinates[e]===a?0:(coordinates[e]-a)/(b-a),u1:coordinates[e+1]===b?1:(coordinates[e+1]-a)/(b-a),sourceEdge});
        }
        const bodyEdges=source.x.slice(0,-1).map((x,i)=>{
            const children=jointEdges.filter(e=>e.bodyEdge===source.start+i),midpoint=x+(source.x[i+1]-x)/2,
                selected=children.find(e=>coordinates[e.jointEdge]<=midpoint&&midpoint<coordinates[e.jointEdge+1])??children.at(-1);
            return{edge:source.start+i,jointEdge:selected.jointEdge,coordinateInterval:[x,source.x[i+1]],jointEdges:children.map(e=>e.jointEdge),fractions:children.map(e=>[e.u0,e.u1]),midpoint};
        });
        mappings.set(source.toolId,{toolId:source.toolId,bodyId:source.bodyId,activeStart:source.start,activeEnd:source.end,nodes:nodeMap,edges:bodyEdges,jointEdges,
            jointNodes:samples.flatMap((s,j)=>s?[{jointNode:j,bodyEdge:s.bodyEdge,u:s.u,originalBodyNode:s.originalBodyNode}]:[])});
    }
    const positions=coordinates.map((_,j)=>(ownPositions.get('catheter')?.[j]??ownPositions.get('wire')?.[j]).slice()),modes=[],relative=[];
    for(let j=0;j<n;j++)if(ownPositions.get('wire')?.[j]&&ownPositions.get('catheter')?.[j]){
        modes.push({node:j,basis:identity.map(v=>v.slice())});relative.push(...ownPositions.get('wire')[j].map((v,k)=>v-positions[j][k]));
    }
    const stateTools=sources.map(source=>{
        const mapping=mappings.get(source.toolId),reference=new Array(n-1).fill(null),ownAngles=new Float64Array(n-1).fill(NaN),rest=new Float64Array(n-1).fill(NaN),referenceTwists=new Float64Array(n-2).fill(NaN);
        mapping.jointEdges.forEach(({jointEdge:e,sourceEdge:i})=>{reference[e]={tangent:source.frames[i].tangent.slice(),director:source.frames[i].director.slice()};ownAngles[e]=source.angles[i];rest[e]=source.dsDx*(coordinates[e+1]-coordinates[e]);});
        const samples=[];
        for(const h of layout.hinges)if(h.tools.includes(source.toolId)){
            const vertex=h.vertex,x=coordinates[vertex],oldNode=source.x.indexOf(x);
            referenceTwists[vertex-1]=oldNode>0&&oldNode<source.x.length-1?source.twists[oldNode-1]:0;
            const start=(coordinates[vertex-1]+x)/2,end=(x+coordinates[vertex+1])/2,
                materialCoordinate=materialLabelAt(source,x),materialStart=materialLabelAt(source,start),materialEnd=materialLabelAt(source,end),
                sourceEdges=source.x.slice(0,-1).flatMap((a,j)=>a<end&&source.x[j+1]>start?[source.start+j]:[]),
                request=freeze({toolId:source.toolId,bodyId:source.bodyId,vertex,coordinate:x,start,end,materialCoordinate,materialStart,materialEnd,dsDx:source.dsDx,sourceEdges});
            samples.push(freeze({...request,material:compiledSnapshot(source.provider(request))}));
        }
        materialSamples.set(source.toolId,Object.freeze(samples));
        // Exact-support immutable snapshot. Moving material/profile maps need
        // explicit refresh by the next preparation layer, never a live closure.
        const materialAt=({vertex,coordinate,start,end})=>{
            const record=samples.find(s=>s.vertex===vertex&&s.coordinate===coordinate&&s.start===start&&s.end===end);
            if(!record)fail('material-support-changed-requires-explicit-profile-preparation',{toolId:source.toolId,vertex,coordinate,start,end});return record.material;
        };
        angles.set(source.toolId,ownAngles);restLengths.set(source.toolId,rest);
        return{id:source.toolId,dsDx:source.dsDx,massPerMaterialLength:source.density,reference,referenceTwists,materialAt};
    });
    const state=createCompositeJointTimeStepState({layout,coordinates,positions,relative,modes,angles,tools:stateTools,restLengths,materialCoordinate:'reference-arclength',relativeToolId:sources.some(s=>s.toolId==='wire')?'wire':'catheter',time,step});
    let originalNodeCount=0,maxOriginalPositionError=0,maxAffinePositionError=0,maxFloat32PublicationError=0,float32PublicationMismatchNodeCount=0;
    const originalNodeRoundoff=[],nodeModes=new Map(state.modes.map(m=>[m.node,m]));
    for(const source of sources){
        const actual=state.toolPositions.get(source.toolId),wanted=ownPositions.get(source.toolId),mapping=mappings.get(source.toolId);
        mapping.nodes.forEach(({node,jointNode})=>{
            originalNodeCount++;const mode=source.toolId===state.relativeToolId?nodeModes.get(jointNode):null,errors=[],bounds=[],float32PublicationErrors=[];
            source.positions[node-source.start].forEach((v,k)=>{
                const error=Math.abs(actual[jointNode][k]-v),rho=mode?state.relative[mode.relativeDofs[k]]:0,bound=positionRoundoffBound(state.positions[jointNode][k],rho,v);
                errors.push(error);bounds.push(bound);maxOriginalPositionError=Math.max(maxOriginalPositionError,error);
                if(!Number.isFinite(error)||error>bound)fail('original-node-reconstruction-exceeds-arithmetic-roundoff',{toolId:source.toolId,bodyNode:node,jointNode,component:k,error,bound});
                const publicationError=source.body[['x','y','z'][k]] instanceof Float32Array?Math.abs(Math.fround(actual[jointNode][k])-v):null;
                float32PublicationErrors.push(publicationError);if(publicationError!==null)maxFloat32PublicationError=Math.max(maxFloat32PublicationError,publicationError);
            });
            if(float32PublicationErrors.some(error=>error!==null&&error!==0))float32PublicationMismatchNodeCount++;
            originalNodeRoundoff.push({toolId:source.toolId,bodyNode:node,jointNode,errors,bounds,float32PublicationErrors});
        });
        mapping.jointNodes.forEach(({jointNode})=>wanted[jointNode].forEach((v,k)=>{maxAffinePositionError=Math.max(maxAffinePositionError,Math.abs(actual[jointNode][k]-v));}));
    }
    const materialVelocities=layout.edgeToolIds.map((ids,e)=>({edge:e,tools:ids.map(id=>{
        const source=sources.find(s=>s.toolId===id),v=ownVelocities.get(id),sStart=materialLabelAt(source,coordinates[e]),sEnd=materialLabelAt(source,coordinates[e+1]);
        const angularVelocity=source.angularVelocity?.[sourceEdgeAt(source,coordinates[e])].slice()??null;
        return{id,sStart,sEnd,velocities:[v[e].slice(),v[e+1].slice()],interpretation:'physical-material-velocity',angularVelocity,
            ...(angularVelocity?{angularVelocityInterpretation:'physical-material-angular-velocity'}:{}),materialSpin:null,frameSpin:null};
    })}));
    state.materialVelocities=structuredClone(materialVelocities);
    const sourceSnapshots=freeze(sources.map(s=>({toolId:s.toolId,bodyId:s.bodyId,activeStart:s.start,activeEnd:s.end,nodeCoordinates:s.x.slice(),positions:s.positions.map(p=>p.slice()),physicalVelocities:s.velocities.map(v=>v.slice()),
        materialLabels:s.labels.slice(),dsDx:s.dsDx,massPerMaterialLength:s.density,sourceRestLengths:s.rest.slice(),reference:s.frames.map(f=>({tangent:f.tangent.slice(),director:f.director.slice()})),angles:s.angles.slice(),referenceTwists:s.twists.slice(),
        sourceQuaternions:s.quaternions.map(q=>({...q})),winding:'explicit-unwrapped',velocityInterpretation:'physical-material-velocity',angularVelocity:s.angularVelocity?.map(v=>v.slice())??null,
        ...(s.angularVelocity?{angularVelocityInterpretation:'physical-material-angular-velocity'}:{}),materialSpin:null,frameSpin:null})));
    const bindings=sources.map(s=>{const m=mappings.get(s.toolId);return{toolId:s.toolId,body:s.body,nodes:m.nodes.map(({node,jointNode,trace})=>({node,jointNode,trace})),edges:m.edges.map(({edge,jointEdge})=>({edge,jointEdge}))};});
    return{state,bindings,mappings,chart:{coordinates:state.coordinates.slice(),edgeToolIds:layout.edgeToolIds.map(ids=>ids.slice()),relativeToolId:state.relativeToolId},
        history:{sourceSnapshots,materialVelocities:structuredClone(materialVelocities),materialSamples,includesAngularVelocity:sources.every(s=>s.angularVelocity!==null),
            angularVelocityToolIds:sources.filter(s=>s.angularVelocity!==null).map(s=>s.toolId),includesKnownOrientation:true},
        evidence:{originalNodeCount,unionNodeCount:n,originalEdgeCount:sources.reduce((sum,s)=>sum+s.end-s.start,0),ownJointEdgeCount:layout.edgeToolIds.reduce((sum,ids)=>sum+ids.length,0),maxOriginalPositionError,maxOriginalNodeRoundoff:maxOriginalPositionError,originalNodeRoundoff,maxAffinePositionError,
            exactOriginalNodeCoordinateCoverage:true,bitExactOriginalNodeReconstruction:maxOriginalPositionError===0,maxFloat32PublicationError,float32PublicationMismatchNodeCount,
            maxSourceQuaternionDirectorError:Math.max(...sources.map(s=>s.maxOrientationError)),maxSourceRestStorageError:Math.max(...sources.map(s=>s.maxRestStorageError)),
            everyOriginalOwnNodeRetained:true,everyNewOwnEdgeInsideOneSourceEdge:true,knownWindingsPreserved:true,sourceBodiesMutated:false},
        scope:'fixed-union-chart-import-of-actual-own-world-curves',publication:'legacy-node-view-and-reference-midpoint-edge-orientation;retain-authoritative-joint-state',
        profileScope:'owned-compiled-current-label-map-snapshot;refresh-explicitly-when-maps-or-support-change',remapReady:false,fullFeedLifecycleReady:false,solverAdvanced:false,certified:false};
}
