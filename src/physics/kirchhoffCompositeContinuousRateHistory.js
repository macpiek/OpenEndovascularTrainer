import {createCompositeContinuousFrame} from './kirchhoffCompositeContinuousFrame.js';
import {evaluateCompositeContinuousSurfaceMotion} from './kirchhoffCompositeContinuousSurfaceMotion.js';

// Equal coordinate arrays alone do not identify a curve: an interface changes
// its one-sided interpolation and material-director support. Own the active
// structural chart, while allowing an identical freshly compiled geometry.
function ownGeometrySignature(geometry,maps) {
    if(!Array.isArray(geometry?.interfaces)||!Array.isArray(geometry?.edges))throw new RangeError('Rate history belongs to another material geometry');
    const active=new Set(maps.map(m=>m.edge));
    if([...active].some(edge=>!Number.isInteger(edge)||edge<0||edge>=geometry.edges.length||!geometry.edges[edge]))
        throw new RangeError('Rate history belongs to another material geometry');
    return {interfaces:Array.from(geometry.interfaces),edges:Array.from(geometry.edges,(edge,i)=>active.has(i)?
        {nodeIndices:Array.from(edge.nodeIndices),region:Array.from(edge.region)}:null)};
}
const sameArray=(a,b)=>Array.isArray(a)&&Array.isArray(b)&&a.length===b.length&&a.every((v,j)=>v===b[j]);
function sameGeometrySignature(a,b) {
    return !!a&&sameArray(a.interfaces,b.interfaces)&&Array.isArray(a.edges)&&a.edges.length===b.edges.length&&a.edges.every((edge,j)=>
        edge===null?b.edges[j]===null:b.edges[j]!==null&&sameArray(edge.nodeIndices,b.edges[j].nodeIndices)&&sameArray(edge.region,b.edges[j].region));
}

/** Owned reconstruction of the accepted IMPLICIT material rate field. This
 * retains both configurations and the previous frame gauge, so a future
 * contact can evaluate its own incoming label's translation and angular rate.
 * It is not angular inertia or an exact finite material surface displacement.
 */
export function captureCompositeContinuousRateHistory({previous,current,prepared,dt}) {
    if(!(dt>0)||!Number.isFinite(dt)||!(current.inertiaGeometryByTool instanceof Map))throw new RangeError('Continuous rate history needs a positive accepted dt and C2 geometry');
    return {kind:'continuous-backward-euler-material-rate',dt,coordinates:Array.from(current.coordinates),tools:previous.tools.map(t=>{
        const maps=prepared.inertiaEdges.flatMap((e,edge)=>e.tools.filter(v=>v.id===t.id).map(v=>({edge,materialMap:structuredClone(v.materialMap)})));
        return {id:t.id,
        previousPositions:previous.toolPositions.get(t.id).map(v=>v.slice()),positions:current.toolPositions.get(t.id).map(v=>v.slice()),
        previousAngles:Array.from(previous.angles.get(t.id)),angles:Array.from(current.angles.get(t.id)),
        reference:t.reference.map(f=>f?{tangent:f.tangent.slice(),director:f.director.slice()}:null),referenceTwists:Array.from(t.referenceTwists),
        maps,geometrySignature:ownGeometrySignature(current.inertiaGeometryByTool.get(t.id),maps)};})};
}

export function sampleCompositeContinuousRateHistory({history,geometryByTool,toolId,label,trace='right'}) {
    if(history?.kind!=='continuous-backward-euler-material-rate'||!(history.dt>0)||!Number.isFinite(label)||!['left','right'].includes(trace))throw new RangeError('Explicit accepted continuous rate history and material label are required');
    const t=history.tools.find(t=>t.id===toolId),geometry=geometryByTool.get(toolId);
    if(!t||!geometry||geometry.coordinates.length!==history.coordinates.length||geometry.coordinates.some((v,k)=>v!==history.coordinates[k])||
        !sameGeometrySignature(t.geometrySignature,ownGeometrySignature(geometry,t.maps)))throw new RangeError('Rate history belongs to another material geometry');
    const spans=t.maps.filter(m=>{const L=history.coordinates[m.edge+1]-history.coordinates[m.edge],a=m.materialMap.sStart,b=a+m.materialMap.dsDx*L;return label>=a&&label<=b;});
    const entry=trace==='right'?spans.at(-1):spans[0];if(!entry)throw new RangeError('Incoming material angular rate needs an explicit reservoir outside accepted labels');
    const m=entry.materialMap,edge=entry.edge,L=history.coordinates[edge+1]-history.coordinates[edge],end=m.sStart+m.dsDx*L,
        fraction=label===m.sStart?0:label===end?1:(label-m.sStart)/(m.dsDx*L),coordinate=history.coordinates[edge]+fraction*L,
        frame=createCompositeContinuousFrame({geometry,edge,toolId,previousPositions:t.previousPositions,previousAngles:t.previousAngles,reference:t.reference,referenceTwists:t.referenceTwists}),
        result=evaluateCompositeContinuousSurfaceMotion({positions:frame.positionNodeIndices.map(n=>t.positions[n]),angles:frame.angleEdgeIndices.map(e=>t.angles[e]),
            coordinate,materialMap:m,contact:{point:[0,0,0],axes:[[1,0,0],[0,1,0]]},wall:{velocity:[0,0,0]},rateMode:'backward-euler-grid',dt:history.dt,order:'value'},frame);
    return {velocity:Array.from(result.velocity),angularVelocity:Array.from(result.omega),position:Array.from(result.position),label,toolId,
        interpretation:'accepted-continuous-backward-euler-material-rate',finiteStepSlipKnown:false};
}
