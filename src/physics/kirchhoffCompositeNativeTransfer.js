import {createCompositeChainLayout} from './kirchhoffCompositeChain.js';
import {createCompositeJointTimeStepState} from './kirchhoffCompositeJointTimeStep.js';
import {createCompositeJointMaterialHistory} from './kirchhoffCompositeJointMaterialHistory.js';
import {prepareCompositeAppMaterialProfile} from './kirchhoffCompositeAppInputs.js';

const finite=(v,name)=>{if(!Number.isFinite(v))throw new RangeError(`${name} must be finite`);return v;};
const vector=(v,n,name)=>{if(v?.length!==n)throw new RangeError(`${name} needs ${n} entries`);return Array.from(v,x=>finite(x,name));};
const dot=(a,b)=>a.reduce((s,v,k)=>s+v*b[k],0),cross=(a,b)=>[a[1]*b[2]-a[2]*b[1],a[2]*b[0]-a[0]*b[2],a[0]*b[1]-a[1]*b[0]];
const mix=(a,b,u)=>a.map((v,k)=>(1-u)*v+u*b[k]);
const normalize=v=>{const n=Math.hypot(...v);if(!(n>0))throw new RangeError('Native transfer has a collapsed material edge');return v.map(x=>x/n);};
function transport(d,a,b) {
    const den=1+dot(a,b);if(!(den>1e-10))throw new RangeError('Native frame transfer cannot reverse its tangent');
    const axis=cross(a,b),first=cross(axis,d),second=cross(axis,first),raw=d.map((v,k)=>v+first[k]+second[k]/den),projection=dot(raw,b);
    return normalize(raw.map((v,k)=>v-projection*b[k]));
}
const frame=(f)=>({tangent:vector(f?.tangent,3,'Own reservoir tangent'),director:vector(f?.director,3,'Own reservoir director')});
const close=(a,b)=>Math.abs(a-b)<=64*Number.EPSILON*Math.max(1,Math.abs(a),Math.abs(b));
const labelAt=(tool,x)=>tool.appMaterialProfile.materialOrigin+(x-tool.appMaterialProfile.coordinateOrigin);
const activeEdges=(state,id)=>state.layout.edgeToolIds.flatMap((ids,e)=>ids.includes(id)?[e]:[]);
function ownAffineHistory(entries,extra=[],endLabels=new Map()) {
    const spans=entries.flatMap(entry=>entry.tools.map(record=>structuredClone(record))).concat(extra.map(record=>({id:record.id,sStart:record.sStart,sEnd:record.sEnd,
        velocities:record.velocities.map(v=>v.slice()),interpretation:'physical-material-velocity'}))),byId=new Map();let maximumLabelAdjustment=0;
    const set=(record,key,value)=>{maximumLabelAdjustment=Math.max(maximumLabelAdjustment,Math.abs(record[key]-value));record[key]=value;};
    for(const span of spans){if(!byId.has(span.id))byId.set(span.id,[]);byId.get(span.id).push(span);}
    for(const [id,own] of byId) {
        own.sort((a,b)=>a.sStart-b.sStart);
        for(let i=1;i<own.length;i++)if(close(own[i-1].sEnd,own[i].sStart))set(own[i-1],'sEnd',own[i].sStart);
        const end=endLabels.get(id),last=own.at(-1);if(end!==undefined&&end>last.sEnd&&close(end,last.sEnd))set(last,'sEnd',end);
    }
    return {entries:[...byId.values()].flat().map((record,edge)=>({edge,tools:[record]})),maximumLabelAdjustment};
}

function sourceTool(source,state) {
    const body=source?.body,id=source?.toolId,old=state.tools.find(t=>t.id===id),count=body?.activeEnd-body?.activeStart+1;
    if(!old||!Number.isInteger(count)||count<2||!old.appMaterialProfile)throw new RangeError('Native transfer needs each existing physical application tool');
    const x=vector(source.nodeCoordinates,count,'Current own source coordinates'),s=vector(source.materialLabels,count,'Current own physical labels');
    if(x.some((v,i)=>i&&v<=x[i-1])||s.some((v,i)=>i&&v<=s[i-1]))throw new RangeError('Native source coordinates and material labels must increase');
    if(s.some((v,i)=>!close(v,s[0]+(x[i]-x[0]))))throw new RangeError('Native application chart requires explicit dsDx=1 material labels');
    const material=prepareCompositeAppMaterialProfile(source.profile??old.appMaterialProfile,x[0],s[0]);
    if(!close(s.at(-1),material.appMaterialProfile.tipMaterialCoordinate))throw new RangeError('The current source must retain its actual physical material tip');
    return {id,body,x,s,old,material};
}

// Independent feed charts can describe the same valve/grid point with a few
// ulps of subtraction error (e.g. 55 - 11*5 versus an exact catheter zero).
// Give only those cross-tool points one numerical coordinate. This is not a
// minimum element length: distinct nodes of one source are never merged.
function reconcileSourceCoordinates(sources) {
    const scale=Math.max(1,...sources.flatMap(t=>t.x.map(Math.abs))),resolution=64*Number.EPSILON*scale,
        points=sources.flatMap(source=>source.x.map((x,index)=>({source,index,x}))).sort((a,b)=>a.x-b.x);
    let maximumAdjustment=0,mergedPoints=0;
    for(let begin=0;begin<points.length;) {
        const group=[points[begin]],owners=new Set([points[begin].source]);let end=begin+1;
        while(end<points.length&&points[end].x-points[begin].x<=resolution&&!owners.has(points[end].source)) {
            owners.add(points[end].source);group.push(points[end++]);
        }
        const canonical=group.find(p=>p.x===0)?.x??group[0].x;
        for(const p of group)if(p.x!==canonical) {
            maximumAdjustment=Math.max(maximumAdjustment,Math.abs(p.x-canonical));
            p.source.x[p.index]=canonical;mergedPoints++;
        }
        begin=end;
    }
    for(const source of sources)if(source.material.appMaterialProfile.coordinateOrigin!==source.x[0])
        source.material=prepareCompositeAppMaterialProfile(source.material.appMaterialProfile,source.x[0],source.s[0]);
    return {maximumAdjustment,mergedPoints,resolution};
}

/** Remap first, then run the ordinary Lagrangian Joint dt. New native nodes
 * sample the accepted physical pose and velocity at their OWN material label.
 * The fixed valve therefore acquires label -progress without advecting the
 * velocity twice. Only newly proximal material comes from an explicit inlet
 * reservoir. Float32 body positions are publication views, never the source
 * of already accepted material geometry.
 *
 * Native sampling is deliberately a discretization change: an edge crossing
 * old vertices replaces their polyline by its chord; the edge's midpoint
 * frame/angle is sampled and tangent-transported. It preserves old unwrapped
 * branches but does not claim exact bending-energy or impulse conservation.
 * Accepted velocity pieces remain exact by material label for inertia.
 * BE contact reactions are numerical warm starts and may be cleared. Physical
 * incoming motion is retained independently by its own material labels.
 */
export function transferCompositeNativeState({state,tools,bindings:previousBindings=null,reservoir=null,transferContactHistory=null,contactRateMode=null}={}) {
    if(state?.elasticityGeometry!=='native-discrete-rod'||state.lengthGeometry!=='native-chords'||state.inertiaGeometryByTool!==null)
        throw new RangeError('Application feed transfer requires the native Joint discretization');
    if(!Array.isArray(tools)||tools.length!==state.tools.length||new Set(tools.map(t=>t.toolId)).size!==tools.length)
        throw new RangeError('Native transfer requires every independent application tool once');
    if(reservoir!==null&&typeof reservoir!=='function')throw new TypeError('Native inlet reservoir must be callable');
    const sources=tools.map(t=>sourceTool(t,state)).sort((a,b)=>['wire','catheter'].indexOf(a.id)-['wire','catheter'].indexOf(b.id));
    const coordinateRoundoff=reconcileSourceCoordinates(sources);
    if(previousBindings&&sources.every(source=> {
        const b=previousBindings.find(b=>b.toolId===source.id);
        return b?.body===source.body&&b.nodes.length===source.x.length&&b.nodes.every((r,i)=>r.node===source.body.activeStart+i&&
            state.coordinates[r.jointNode]===source.x[i]&&close(labelAt(source.old,state.coordinates[r.jointNode]),source.s[i]))&&
            ['type','tipMaterialCoordinate','shaftStiffnessScale','tipStiffnessScale'].every(k=>source.material.appMaterialProfile[k]===source.old.appMaterialProfile[k]);
    }))return null;
    const
        coordinates=[...new Set(sources.flatMap(t=>t.x))].sort((a,b)=>a-b),n=coordinates.length,
        edgeToolIds=coordinates.slice(0,-1).map((x,e)=>sources.filter(t=>x>=t.x[0]&&coordinates[e+1]<=t.x.at(-1)).map(t=>t.id));
    if(n<3||edgeToolIds.some(ids=>!ids.length))throw new RangeError('Native material ranges must form a connected union');
    const layout=createCompositeChainLayout(edgeToolIds),ownPositions=new Map(),ownRecords=new Map(),materialAtNode=new Map(),bindings=[],mappings=new Map(),reservoirRecords=new Map();
    let reservoirSamples=0,retainedSamples=0,maximumRemovedVertexDistance=0;
    const historyReservoir=request=>{
        const supplied=reservoir?.(request);if(!supplied)return null;
        const span={...supplied,id:supplied.id,sStart:finite(supplied.sStart,'Reservoir start'),sEnd:finite(supplied.sEnd,'Reservoir end')};
        if(span.id!==request.toolId||!(span.sEnd>span.sStart))throw new RangeError('Reservoir must cover its own physical tool');
        span.positions=span.positions.map(v=>vector(v,3,'Explicit inlet physical position'));span.velocities=span.velocities.map(v=>vector(v,3,'Explicit inlet physical velocity'));
        if(span.positions.length!==2||span.velocities.length!==2||span.interpretation!=='physical-material-velocity')throw new RangeError('Reservoir requires physical position/velocity endpoints');
        span.reference=frame(span.reference);span.angle=finite(span.angle,'Own unwrapped inlet angle');
        if(Math.abs(dot(span.reference.tangent,span.reference.tangent)-1)>1e-10||Math.abs(dot(span.reference.director,span.reference.director)-1)>1e-10||Math.abs(dot(span.reference.tangent,span.reference.director))>1e-10)
            throw new RangeError('Reservoir material frame must be orthonormal');
        const key=JSON.stringify([request.toolId,span.sStart,span.sEnd]);
        if(reservoirRecords.has(key)&&JSON.stringify(reservoirRecords.get(key))!==JSON.stringify(span))throw new RangeError('Prepared reservoir must be deterministic');
        reservoirRecords.set(key,span);return span;
    };
    const acceptedHistory=ownAffineHistory(state.materialVelocities);
    for(const source of sources) {
        const id=source.id,old=source.old,edges=activeEdges(state,id),records=edges.map(e=>({edge:e,sStart:labelAt(old,state.coordinates[e]),sEnd:labelAt(old,state.coordinates[e+1])})),
            cumulative=new Map();let winding=0;
        records.forEach((r,j)=>{if(j)winding+=old.referenceTwists[r.edge-1];cumulative.set(r.edge,winding);});
        const first=records[0],last=records.at(-1),p=state.toolPositions.get(id);
        function sample(s,trace='right') {
            let record=records.find((r,j)=>s>=r.sStart&&s<=r.sEnd&&(s<r.sEnd||trace==='left'||j===records.length-1));
            if(record) {
                const u=(s-record.sStart)/(record.sEnd-record.sStart),e=record.edge;retainedSamples++;
                return {position:s===record.sStart?p[e].slice():s===record.sEnd?p[e+1].slice():mix(p[e],p[e+1],u),
                    reference:old.reference[e],angle:state.angles.get(id)[e],winding:cumulative.get(e),source:'accepted',oldEdge:e,s};
            }
            if(s>=last.sEnd)throw new RangeError('Native transfer cannot invent distal material beyond its own tip');
            const inlet=historyReservoir({toolId:id,s,trace});
            if(!inlet||s<inlet.sStart||s>inlet.sEnd||inlet.sEnd>first.sStart&&!close(inlet.sEnd,first.sStart))
                throw new RangeError(`Missing explicit proximal pose reservoir for ${id}:${s}`);
            const u=(s-inlet.sStart)/(inlet.sEnd-inlet.sStart);reservoirSamples++;
            return {position:mix(inlet.positions[0],inlet.positions[1],u),reference:inlet.reference,angle:inlet.angle,winding:0,source:'reservoir',oldEdge:null,s};
        }
        const own=coordinates.map(x=>x>=source.x[0]&&x<=source.x.at(-1)?sample(source.s[0]+(x-source.x[0])):null),
            ownEdges=coordinates.slice(0,-1).map((x,e)=>edgeToolIds[e].includes(id)?sample(source.s[0]+((x+coordinates[e+1])/2-source.x[0])):null);
        ownPositions.set(id,own.map(p=>p?.position??null));ownRecords.set(id,ownEdges);materialAtNode.set(id,own.map(p=>p?.s??null));
        const nodes=source.x.map((x,i)=>({node:source.body.activeStart+i,jointNode:coordinates.indexOf(x),coordinate:x,materialLabel:source.s[i],trace:i===source.x.length-1?'left':'right'})),
            bodyEdges=source.x.slice(0,-1).map((x,i)=>{
                const children=coordinates.slice(0,-1).flatMap((a,e)=>a>=x&&coordinates[e+1]<=source.x[i+1]?[e]:[]),midpoint=(x+source.x[i+1])/2,
                    jointEdge=children.find(e=>coordinates[e]<=midpoint&&midpoint<coordinates[e+1])??children.at(-1);
                return {edge:source.body.activeStart+i,jointEdge,jointEdges:children,midpoint,coordinateInterval:[x,source.x[i+1]]};
            });
        bindings.push({body:source.body,toolId:id,nodes,edges:bodyEdges});
        mappings.set(id,{toolId:id,bodyId:source.body.id,activeStart:source.body.activeStart,activeEnd:source.body.activeEnd,nodes,edges:bodyEdges});
        // Report the actual geometric approximation at old vertices removed
        // by the new mesh; no false 'exact transfer' flag is returned.
        for(const record of records)for(const [label,point] of [[record.sStart,p[record.edge]],[record.sEnd,p[record.edge+1]]]) {
            const x=source.x[0]+(label-source.s[0]);if(x<source.x[0]||x>source.x.at(-1))continue;
            const e=coordinates.findIndex((a,e)=>a<=x&&x<=coordinates[e+1]&&edgeToolIds[e].includes(id));if(e<0)continue;
            const interpolated=mix(own[e].position,own[e+1].position,(x-coordinates[e])/(coordinates[e+1]-coordinates[e]));
            maximumRemovedVertexDistance=Math.max(maximumRemovedVertexDistance,Math.hypot(...point.map((v,k)=>v-interpolated[k])));
        }
    }
    const positions=coordinates.map((x,j)=>(ownPositions.get('catheter')?.[j]??ownPositions.get('wire')?.[j]).slice()),modes=[],relative=[];
    for(let j=0;j<n;j++)if(ownPositions.get('wire')?.[j]&&ownPositions.get('catheter')?.[j]) {
        modes.push({node:j,basis:[[1,0,0],[0,1,0],[0,0,1]]});relative.push(...ownPositions.get('wire')[j].map((v,k)=>v-positions[j][k]));
    }
    const angles=new Map(),restLengths=new Map(),nextTools=sources.map(source=>{
        const id=source.id,p=ownPositions.get(id),records=ownRecords.get(id),reference=new Array(n-1).fill(null),a=new Float64Array(n-1).fill(NaN),rest=new Float64Array(n-1).fill(NaN),twists=new Float64Array(n-2).fill(NaN);
        records.forEach((r,e)=>{if(!r)return;const tangent=normalize(p[e+1].map((v,k)=>v-p[e][k]));
            reference[e]={tangent,director:transport(r.reference.director,r.reference.tangent,tangent)};a[e]=r.angle;rest[e]=coordinates[e+1]-coordinates[e];});
        for(const h of layout.hinges)if(h.tools.includes(id)) {
            const e=h.vertex,left=reference[e-1],right=reference[e],d=transport(left.director,left.tangent,right.tangent),
                phase=Math.atan2(dot(right.tangent,cross(d,right.director)),dot(d,right.director)),anchor=records[e].winding-records[e-1].winding;
            twists[e-1]=phase+2*Math.PI*Math.round((anchor-phase)/(2*Math.PI));
        }
        angles.set(id,a);restLengths.set(id,rest);
        return {...source.old,...source.material,reference,referenceTwists:twists};
    });
    const next=createCompositeJointTimeStepState({layout,coordinates,positions,modes,relative,angles,restLengths,tools:nextTools,
        materialCoordinate:'reference-arclength',relativeToolId:state.relativeToolId,elasticityGeometry:'native-discrete-rod',lengthGeometry:'native-chords',time:state.time,step:state.step});
    const maps=edgeToolIds.map((ids,e)=>({tools:ids.map(id=>({id,materialMap:{sStart:labelAt(nextTools.find(t=>t.id===id),coordinates[e]),dsDx:1,dsDt:0}}))})),endLabels=new Map();
    maps.forEach((entry,e)=>entry.tools.forEach(t=>endLabels.set(t.id,t.materialMap.sStart+(coordinates[e+1]-coordinates[e]))));
    // Preserve every accepted affine velocity break, even if the new native
    // positional mesh crosses it. The next inertia assembly consumes pieces.
    const transferredHistory=ownAffineHistory(acceptedHistory.entries,[...reservoirRecords.values()],endLabels);
    next.materialVelocities=transferredHistory.entries;
    createCompositeJointMaterialHistory({materialVelocities:next.materialVelocities}).prepare({coordinates,inertiaEdges:maps});
    // Rate history is queried by OWN material label, not the new mesh index.
    // Keep its accepted pieces and endpoint derivatives unchanged. Explicit
    // inlet records add physical rates, without inventing a prior pose path.
    if(state.nativeRateHistory!==undefined) {
        next.nativeRateHistory=structuredClone(state.nativeRateHistory);
        for(const source of sources) {
            const rate=next.nativeRateHistory.tools.find(t=>t.id===source.id),first=rate?.edges[0];
            if(!first)throw new RangeError('Each transferred tool needs its own accepted native rate history');
            if(source.s[0]<first.labels[0]) {
                const inlet=historyReservoir({toolId:source.id,s:source.s[0],trace:'right'});
                if(!inlet||!close(inlet.sEnd,first.labels[0]))throw new RangeError('Explicit inlet rate reservoir must join accepted material history');
                const angularVelocity=vector(inlet.angularVelocity,3,'Explicit inlet physical angular velocity');
                rate.edges.unshift({edge:first.edge-1,labels:[inlet.sStart,inlet.sEnd],positions:inlet.positions.map(p=>p.slice()),
                    centerVelocities:inlet.velocities.map(v=>v.slice()),angularVelocity,
                    source:'explicit-inlet-physical-material-rates',finiteStepPosePathKnown:false});
                rate.edges.forEach((entry,index)=>entry.edge=index);
            }
        }
    }
    // Length/BC multipliers are equilibrium guesses, not material momentum.
    // Restrict the original signed axial force to each new midpoint; map an
    // exactly retained prescribed material point's reaction once.
    const oldRows=state.layout.edgeToolIds.flatMap((ids,edge)=>ids.map(id=>({id,edge}))),newRows=layout.edgeToolIds.flatMap((ids,edge)=>ids.map(id=>({id,edge})));
    next.lengthMultipliers.set(newRows.map(r=>{
        const oldEdge=ownRecords.get(r.id)[r.edge].oldEdge,index=oldRows.findIndex(old=>old.id===r.id&&old.edge===oldEdge);return index<0?0:state.lengthMultipliers[index];
    }));
    const retainedBC=new Map();for(const [key,value] of state.boundaryMultipliers??[]) {
        const [id,node]=JSON.parse(key),oldTool=state.tools.find(t=>t.id===id),label=labelAt(oldTool,state.coordinates[node]),newNode=materialAtNode.get(id).indexOf(label);
        if(newNode>=0)retainedBC.set(JSON.stringify([id,newNode]),Array.from(value));
    }next.boundaryMultipliers=retainedBC;
    const diagnostics={coordinateRoundoff,scope:'native-own-material-remap-before-lagrangian-step',previousNodes:state.layout.nodeCount,nodes:n,
        retainedSamples,reservoirSamples,maximumRemovedVertexDistance,geometryExact:maximumRemovedVertexDistance===0,
        velocityHistory:'preserved-affine-pieces-at-own-material-labels',angularSampling:'native-edge-midpoint-with-unwrapped-branch-and-tangent-transport',
        energyExact:false,timeAdvanced:0,mappings,materialAtNode,previousCoordinates:Array.from(state.coordinates),warmStartReset:false,
        maximumHistoryLabelRoundoffAdjustment:Math.max(acceptedHistory.maximumLabelAdjustment,transferredHistory.maximumLabelAdjustment),
        preservedHistoryFields:['materialPositions','materialFrames','unwrappedAngles','materialVelocities',...(state.nativeRateHistory?['nativeRateHistory']:[])]};
    const fields=['sheathContactState','wallContactState','wallFrictionState','lumenContactState','lumenFrictionState'];
    if(fields.some(key=>state[key]!==undefined)) {
        const frictionFields=['wallFrictionState','lumenFrictionState'],canColdStart=contactRateMode==='backward-euler-grid'&&
            frictionFields.every(key=>state[key]===undefined||state[key].rateMode==='backward-euler-grid');
        if(typeof transferContactHistory!=='function'&&!canColdStart&&frictionFields.some(key=>state[key]!==undefined))
            throw Object.assign(new RangeError('Finite-displacement contact history requires explicit chart transfer'),{code:'native-contact-history-transfer-required',details:diagnostics});
        const histories=transferContactHistory?.({previous:state,state:next,bindings,mappings,materialAtNode,diagnostics})??{};
        if(!histories||typeof histories!=='object')throw new RangeError('Native contact transfer must return explicit owned histories');
        for(const key of fields)if(histories[key]!==undefined)next[key]=structuredClone(histories[key]);
        diagnostics.warmStartReset=fields.some(key=>state[key]!==undefined&&histories[key]===undefined);
    }
    return {state:next,bindings,mappings,diagnostics};
}
