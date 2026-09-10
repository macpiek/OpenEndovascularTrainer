const RATE_MODEL='endpoint-derivative-of-linear-grid-pose-path';
const finite=(v,name)=>{if(!Number.isFinite(v))throw new RangeError(`${name} must be finite`);return v;},
    vector=(v,n,name)=>{if(v?.length!==n)throw new RangeError(`${name} needs ${n} entries`);return Array.from(v,x=>finite(x,name));},
    dot=(a,b)=>a.reduce((sum,v,k)=>sum+v*b[k],0),cross=(a,b)=>[a[1]*b[2]-a[2]*b[1],a[2]*b[0]-a[0]*b[2],a[0]*b[1]-a[1]*b[0]],
    same=(a,b)=>a?.length===b?.length&&a.every((v,j)=>v===b[j]);
function unsupported(message){const e=new RangeError(message);e.code='unsupported-native-rate-history';throw e;}

/** The declared path linearly interpolates OWN grid endpoints and unwrapped
 * angles over the accepted dt, with the old reference transported to the
 * current tangent. Its endpoint derivative is an explicit discrete rate law,
 * not a measured angular velocity inferred from quasi-static angle values.
 * Material translation equals BE grid velocity minus prescribed feed. Native
 * orientation is constant on each open edge. No finite hinge-crossing or
 * same-material surface displacement is reconstructed by this history.
 */
export function captureCompositeNativeRateHistory({previous,current,prepared,dt,rateModel}={}) {
    if(rateModel!==RATE_MODEL)unsupported('Explicit native endpoint pose-path rate model is required');
    if(!(finite(dt,'Accepted native dt')>0))unsupported('Accepted native dt must be positive');
    if(previous?.elasticityGeometry!=='native-discrete-rod'||current?.elasticityGeometry!=='native-discrete-rod'||current.inertiaGeometryByTool!=null)
        unsupported('Native rate history requires the native discrete rod and affine geometry');
    const coordinates=vector(current.coordinates,current.coordinates?.length,'Native coordinates');
    if(coordinates.length<2||coordinates.some((v,j)=>j&&!(v>coordinates[j-1]))||!same(coordinates,previous.coordinates))unsupported('Native rate history needs the same ordered physical coordinate chart');
    if(!(previous.toolPositions instanceof Map)||!(current.toolPositions instanceof Map)||!(previous.angles instanceof Map)||!(current.angles instanceof Map)||
        !Array.isArray(previous.tools)||!Array.isArray(prepared?.inertiaEdges)||prepared.inertiaEdges.length!==coordinates.length-1)
        unsupported('Complete own previous/current native poses and prepared material maps are required');
    const seen=new Set(),tools=previous.tools.map(tool=>{
        const id=tool.id;if(typeof id!=='string'||!id||seen.has(id))unsupported('Distinct own native tool identities are required');seen.add(id);
        const old=previous.toolPositions.get(id),next=current.toolPositions.get(id),oldAngles=previous.angles.get(id),nextAngles=current.angles.get(id);
        if(old?.length!==coordinates.length||next?.length!==coordinates.length||oldAngles?.length!==coordinates.length-1||nextAngles?.length!==coordinates.length-1)
            unsupported('Native rate poses must cover their own coordinate chart');
        const edges=[];
        prepared.inertiaEdges.forEach((entry,edge)=>{
            const matches=entry.tools.filter(t=>t.id===id);if(matches.length>1)unsupported('Duplicate own native material map');if(!matches.length)return;
            if(current.layout&&!current.layout.edgeToolIds[edge]?.includes(id)||previous.layout&&!previous.layout.edgeToolIds[edge]?.includes(id))
                unsupported('Native rate history cannot transfer an active edge to another body');
            const positions=[next[edge],next[edge+1]].map(v=>vector(v,3,'Accepted own position')),previousPositions=[old[edge],old[edge+1]].map(v=>vector(v,3,'Previous own position')),
                angle=finite(nextAngles[edge],'Accepted unwrapped angle'),previousAngle=finite(oldAngles[edge],'Previous unwrapped angle'),
                reference={tangent:vector(tool.reference?.[edge]?.tangent,3,'Previous own tangent'),director:vector(tool.reference?.[edge]?.director,3,'Previous own director')},
                oldChord=previousPositions[1].map((v,k)=>v-previousPositions[0][k]),oldLength=Math.hypot(...oldChord),chord=positions[1].map((v,k)=>v-positions[0][k]),length=Math.hypot(...chord);
            if(!(oldLength>1e-12)||!(length>1e-12)||!Number.isFinite(oldLength+length)||Math.hypot(...reference.tangent.map((v,k)=>v-oldChord[k]/oldLength))>1e-10||
                Math.abs(dot(reference.tangent,reference.tangent)-1)>1e-10||Math.abs(dot(reference.director,reference.director)-1)>1e-10||Math.abs(dot(reference.tangent,reference.director))>1e-10)
                unsupported('The native reference frame must belong to its own nondegenerate previous edge');
            const tangent=chord.map(v=>v/length),positionRates=positions.map((p,j)=>p.map((v,k)=>(v-previousPositions[j][k])/dt)),
                chordRate=positionRates[1].map((v,k)=>v-positionRates[0][k]),stretchRate=dot(tangent,chordRate),tangentRate=chordRate.map((v,k)=>(v-tangent[k]*stretchRate)/length),
                denominator=1+dot(reference.tangent,tangent);
            if(!(denominator>1e-10))unsupported('Antiparallel native endpoint tangents need another declared pose path');
            const angleRate=(angle-previousAngle)/dt,connection=dot(cross(reference.tangent,tangent),tangentRate)/denominator,
                bend=cross(tangent,tangentRate),angularVelocity=bend.map((v,k)=>v+tangent[k]*(angleRate-connection)),
                source=matches[0].materialMap,L=coordinates[edge+1]-coordinates[edge],sStart=finite(source?.sStart,'Own accepted material start'),dsDx=finite(source?.dsDx,'Own accepted material metric'),
                rates=typeof source.dsDt==='number'?[finite(source.dsDt,'Own endpoint label rate'),source.dsDt]:vector(source.dsDt,2,'Own endpoint label rates');
            if(!(dsDx>0))unsupported('Own native material metric must be positive');
            const labels=[sStart,finite(sStart+dsDx*L,'Own accepted material end')],centerVelocities=positionRates.map((v,j)=>v.map((x,k)=>x-rates[j]/dsDx*(chord[k]/L)));
            if(!(labels[1]>labels[0])||[...angularVelocity,...centerVelocities.flat(),...positionRates.flat(),angleRate].some(v=>!Number.isFinite(v)))unsupported('Unresolved native endpoint material rate');
            edges.push({edge,coordinates:[coordinates[edge],coordinates[edge+1]],labels,materialMap:{sStart,dsDx,dsDt:rates},positions,previousPositions,angle,previousAngle,reference,
                positionRates,angleRate,centerVelocities,angularVelocity});
        });
        if(!edges.length)unsupported('Every native body needs an active material rate span');
        // The mechanical kernel also accepts explicitly local edge labels.
        // Recording their own rates must not add a global-label constraint to
        // an otherwise valid step. A global material query checks uniqueness.
        const continuousLabels=edges.every((e,j)=>j===0||e.edge===edges[j-1].edge+1&&e.labels[0]===edges[j-1].labels[1]);
        return {id,edges,continuousLabels};
    });
    return {kind:'owned-native-endpoint-material-rate',rateModel,dt,coordinates,tools,finiteStepSlipKnown:false,includesHingeTransport:false};
}

export function sampleCompositeNativeRateHistory({history,toolId,label,trace='right'}={}) {
    if(history?.kind!=='owned-native-endpoint-material-rate'||history.rateModel!==RATE_MODEL||history.finiteStepSlipKnown!==false||history.includesHingeTransport!==false||
        !(history.dt>0)||!Number.isFinite(history.dt)||!Number.isFinite(label)||!['left','right'].includes(trace))unsupported('Explicit owned native endpoint-path history and material label are required');
    const tool=history.tools.find(t=>t.id===toolId);if(!tool)unsupported('Native rate history belongs to another physical body');
    const spans=tool.edges.filter(e=>label>=e.labels[0]&&label<=e.labels[1]),entry=trace==='left'?spans[0]:spans.at(-1);
    if(!entry)unsupported('Incoming native material rate outside accepted labels needs an explicit reservoir');
    if(spans.length>1&&(spans.length!==2||spans[0].edge+1!==spans[1].edge||spans[0].labels[1]!==label||spans[1].labels[0]!==label))
        unsupported('Incoming native material label is ambiguous across local edge maps');
    const fraction=label===entry.labels[0]?0:label===entry.labels[1]?1:(label-entry.labels[0])/(entry.labels[1]-entry.labels[0]),
        interpolate=p=>p[0].map((v,k)=>fraction===0?v:fraction===1?p[1][k]:v+fraction*(p[1][k]-v));
    return {toolId,edge:entry.edge,label,fraction,trace:fraction===0?'right':fraction===1?'left':null,position:interpolate(entry.positions),velocity:interpolate(entry.centerVelocities),
        angularVelocity:entry.angularVelocity.slice(),interpretation:'accepted-native-endpoint-path-material-rate',rateModel:RATE_MODEL,
        spatialOrientationField:'constant-on-own-open-DER-edge',finiteStepSlipKnown:false,includesHingeTransport:false};
}
