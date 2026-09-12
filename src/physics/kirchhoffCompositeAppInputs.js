import { createBishopFrame, materialFrameDirectors } from './discreteKirchhoffRod.js';
import { compileCompositeMaterial } from './kirchhoffCompositeElement.js';
import { kirchhoffMaterialProfile } from './kirchhoffMaterialProfile.js';
import { guidewireMaterialProfile } from './guidewireMaterialProfile.js';
import { importCompositeJointWorld } from './kirchhoffCompositeJointWorldImport.js';
import { createCompositeJointTimeStepState } from './kirchhoffCompositeJointTimeStep.js';
import { createCompositeJointMaterialHistory } from './kirchhoffCompositeJointMaterialHistory.js';

const finite = (v, name) => { if (!Number.isFinite(v)) throw new RangeError(`${name} must be finite`); return v; };
const positive = (v, name) => { if (!(finite(v, name) > 0)) throw new RangeError(`${name} must be positive`); return v; };
const vector = (v, n, name) => { if (v?.length !== n) throw new RangeError(`${name} needs ${n} entries`); return Array.from(v, x => finite(x, name)); };
const dot = (a, b) => a.reduce((s, v, k) => s + v * b[k], 0);
const cross = (a, b) => [a[1]*b[2]-a[2]*b[1], a[2]*b[0]-a[0]*b[2], a[0]*b[1]-a[1]*b[0]];
const read = (v) => ['x','y','z'].map(k => v[k]);
const smooth = x => { const t = Math.max(0, Math.min(1, x)); return t*t*(3-2*t); };
const initialAlignmentEpochs = new WeakMap();
function transport(d, a, b) {
    const den = 1 + dot(a,b); if (!(den > 1e-10)) throw new RangeError('Initial source frame needs a regular tangent chart');
    const v = cross(a,b), first = cross(v,d), second = cross(v,first);
    return d.map((x,k) => x + first[k] + second[k]/den);
}

/** Finalize the initial material-frame pose AFTER Float32 source positions
 * have been prepared, before the one-time Joint import. This explicitly
 * applies a small shortest-tangent swing to the actual d1, preserving roll.
 * It changes current/previous quaternions; it is not an identity import of
 * their old orientations. No pose, length, constitutive profile or linear
 * velocity is changed. Only a declared fresh epoch with zero angular history
 * is supported; accepted solver state must never pass through this helper.
 */
export function alignCompositeAppInitialMaterialFrames({body,epoch,maximumTangentCorrection=1e-3}={}) {
    if(!epoch||typeof epoch!=='object')throw new TypeError('An explicit fresh initialization epoch object is required');
    if(body?.jointStateView!==undefined)throw new RangeError('Accepted Joint frames cannot be reinitialized');
    const limit=positive(maximumTangentCorrection,'Maximum initial tangent correction');
    if(limit>1e-3)throw new RangeError('Initial frame repair only supports small source tangent roundoff');
    const start=body?.activeStart,end=body?.activeEnd;
    if(!Number.isInteger(start)||!Number.isInteger(end)||start<0||end<=start||end>=body.count)
        throw new RangeError('Initial frame repair requires an active source edge range');
    const fields=['orientationX','orientationY','orientationZ','orientationW',
        'previousOrientationX','previousOrientationY','previousOrientationZ','previousOrientationW'];
    for(const name of fields)if(!(body[name] instanceof Float64Array)||body[name].length!==body.segmentCount)
        throw new RangeError('Initial material-frame storage must be complete Float64 arrays');
    for(const name of ['angularVelocityX','angularVelocityY','angularVelocityZ'])
        if(body[name]?.length!==body.segmentCount||Array.from(body[name].subarray(start,end)).some(v=>v!==0))
            throw new RangeError('Initial frame alignment requires explicit zero angular history');
    const positions=Array.from({length:end-start+1},(_,i)=>['x','y','z'].map(k=>finite(body[k]?.[start+i],'Initial source position'))),
        previous=initialAlignmentEpochs.get(body);
    if(previous?.epoch===epoch) {
        const unchanged=previous.start===start&&previous.end===end&&positions.every((p,i)=>p.every((v,k)=>v===previous.positions[i]?.[k]))&&
            previous.quaternions.every((q,i)=>fields.every((name,k)=>body[name][start+i]===q[k%4]));
        if(!unchanged)throw new RangeError('The source changed after its fresh epoch frame alignment');
        return {...previous.evidence,reused:true};
    }
    let maximumCorrection=0,maximumAlignedTangentError=0;
    const quaternions=positions.slice(0,-1).map((p,i)=> {
        const e=start+i,source=Object.fromEntries(['x','y','z','w'].map(k=>[k,finite(body[`orientation${k.toUpperCase()}`][e],'Initial source quaternion')]));
        positive(Math.hypot(source.x,source.y,source.z,source.w),'Initial source quaternion norm');
        const directors=materialFrameDirectors(source,{}),a=read(directors.d3),d=read(directors.d1),
            chord=positions[i+1].map((v,k)=>v-p[k]),length=positive(Math.hypot(...chord),'Initial physical edge length'),
            b=chord.map(v=>v/length),correction=Math.atan2(Math.hypot(...cross(a,b)),dot(a,b));
        if(correction>limit)throw new RangeError('Initial source tangent mismatch exceeds the small frame repair bound');
        maximumCorrection=Math.max(maximumCorrection,correction);
        const carried=transport(d,a,b),q=createBishopFrame({x:b[0],y:b[1],z:b[2]}, {x:carried[0],y:carried[1],z:carried[2]}),
            values=['x','y','z','w'].map(k=>q[k]);
        if(values.reduce((sum,v,k)=>sum+v*source[['x','y','z','w'][k]],0)<0)values.forEach((v,k)=>values[k]=-v);
        const aligned=read(materialFrameDirectors(q,{}).d3),error=Math.hypot(...aligned.map((v,k)=>v-b[k]));
        if(error>1e-10)throw new RangeError('Initial material frame failed exact source chord alignment');
        maximumAlignedTangentError=Math.max(maximumAlignedTangentError,error);return values;
    });
    // Validate every edge first; a failure never repairs only part of a body.
    quaternions.forEach((q,i)=>fields.forEach((name,k)=>body[name][start+i]=q[k%4]));
    const evidence={scope:'fresh-source-material-frame-tangent-alignment',edges:end-start,maximumCorrection,
        maximumAlignedTangentError,physicalOrientationsAdjusted:maximumCorrection>0,restConfigurationChanged:false,reused:false};
    initialAlignmentEpochs.set(body,{epoch,start,end,positions,quaternions,evidence});return {...evidence};
}
function profileMap(input, coordinateOrigin, materialOrigin) {
    const profile = kirchhoffMaterialProfile(input.type);
    if (profile.id !== input.type) throw new RangeError('An explicit supported application material profile is required');
    const spec = Object.freeze({type:profile.id, tipMaterialCoordinate:finite(input.tipMaterialCoordinate,'Physical tip material label'),
        shaftStiffnessScale:positive(input.shaftStiffnessScale ?? 1,'Shaft stiffness scale'),
        tipStiffnessScale:positive(input.tipStiffnessScale ?? 1,'Tip stiffness scale'),
        coordinateOrigin:finite(coordinateOrigin,'Profile chart origin'),materialOrigin:finite(materialOrigin,'Profile label origin')});
    const wire = ['glidewire','steel-j-035'].includes(profile.id), old = wire ? guidewireMaterialProfile(profile.id) : null,
        core = wire ? old.tipCoreLength : profile.naturalTipLengthMm,
        transition = wire ? old.tipTransitionLength : profile.id === 'berenstein' ? 6 : 8;
    // This is the same independent shaft/tip scale field used by
    // applyKirchhoffMaterialProfile, sampled before continuous integration.
    const materialAt = ({coordinate}) => {
        const label = spec.materialOrigin + (finite(coordinate,'Material sample coordinate') - spec.coordinateOrigin),
            distance = spec.tipMaterialCoordinate - label;
        if (distance < -64*Number.EPSILON*Math.max(1,Math.abs(label),Math.abs(spec.tipMaterialCoordinate)))
            throw new RangeError('Material sample is beyond its own physical distal tip');
        const p = profile.sample(Math.max(0,distance)), t = transition > 0 ? smooth((distance-core)/transition) : distance>core?1:0,
            scale = spec.tipStiffnessScale * Math.pow(spec.shaftStiffnessScale/spec.tipStiffnessScale,t);
        return compileCompositeMaterial({EI1:p.EI1*scale, EI2:p.EI2*scale, GJ:p.GJ*scale, intrinsic:[p.kappa01,p.kappa02,p.tau0]});
    };
    // Include every constitutive profile join, as well as the scale joins.
    const distances = wire ? [old.tipCoreLength,old.tipCoreLength+old.tipTransitionLength,15,20] :
        profile.id === 'pigtail' ? [profile.naturalTipLengthMm-4,profile.naturalTipLengthMm] :
        profile.id === 'berenstein' ? [8,10,16,18] : [8,11,17,20,40,43,67,70];
    const materialBreaks = [...new Set([...distances,core,core+transition])].map(d =>
        spec.coordinateOrigin + spec.tipMaterialCoordinate - d - spec.materialOrigin).sort((a,b)=>a-b);
    return {appMaterialProfile:spec,materialAt,materialBreaks};
}

export function prepareCompositeAppMaterialProfile(input,coordinateOrigin,materialOrigin) {
    return profileMap(input,coordinateOrigin,materialOrigin);
}

/** A native source edge with both endpoint positions prescribed has a
 * prescribed affine centerline. Union nodes inserted by the other tool inherit
 * that same boundary, rather than adding unrequested free geometry inside a
 * pinned straight segment. Explicit user constraints retain precedence.
 */
export function completeCompositeAppNativePositionBoundaries({state,bindings,positionBoundaries=[]}) {
    const result=structuredClone(positionBoundaries);if(state.elasticityGeometry!=='native-discrete-rod')return result;
    const key=(id,node)=>JSON.stringify([id,node]),known=new Map(result.map(row=>[key(row.toolId,row.node),row]));
    for(const binding of bindings) {
        const nodes=binding.nodes.slice().sort((a,b)=>a.node-b.node),id=binding.toolId;
        for(let i=1;i<nodes.length;i++) {
            const left=nodes[i-1],right=nodes[i],a=known.get(key(id,left.jointNode)),b=known.get(key(id,right.jointNode));
            if(!a||!b||right.node!==left.node+1)continue;
            const x0=state.coordinates[left.jointNode],x1=state.coordinates[right.jointNode];
            for(let node=left.jointNode+1;node<right.jointNode;node++) {
                if(known.has(key(id,node)))continue;
                const fraction=(state.coordinates[node]-x0)/(x1-x0),row={toolId:id,node,value:a.value.map((v,k)=>v+fraction*(b.value[k]-v))};
                result.push(row);known.set(key(id,node),row);
            }
        }
    }
    return result;
}

/** One-time source descriptor for the actual app bodies. Labels are immutable
 * physical material labels increasing towards the tip; chart coordinates are
 * increasing reference arclength measured from the introducer. The caller
 * supplies the 2*pi lift; quaternions cannot recover lost turns. At a fresh
 * initialization only, the app may declare its known zero-turn epoch.
 */
export function createCompositeAppToolSource({body,toolId,nodeCoordinates,materialLabels,unwrappedAngles,
    referenceWindingTurns,massPerMaterialLength,profile} = {}) {
    const count = body?.activeEnd-body?.activeStart+1;
    if (!Number.isInteger(count) || count<2) throw new RangeError('An active application material edge is required');
    const x = vector(nodeCoordinates,count,'Application chart nodes'), labels = vector(materialLabels,count,'Own physical material labels'),
        angles = vector(unwrappedAngles,count-1,'Known unwrapped material angles'), turns = vector(referenceWindingTurns,count-2,'Known reference winding turns');
    if (turns.some(v=>!Number.isInteger(v))) throw new RangeError('Reference winding turns must be explicit integers');
    const reference = angles.map((angle,local) => {
        const e = body.activeStart+local, q = Object.fromEntries(['x','y','z','w'].map(k=>[k,body[`orientation${k.toUpperCase()}`][e]])),
            frame = materialFrameDirectors(q,{}), tangent = read(frame.d3), d1 = read(frame.d1), transverse = cross(tangent,d1);
        return {tangent,director:d1.map((v,k)=>v*Math.cos(angle)-transverse[k]*Math.sin(angle))};
    });
    const referenceTwists = turns.map((turn,i) => {
        const a = reference[i], b = reference[i+1], d = transport(a.director,a.tangent,b.tangent);
        return Math.atan2(dot(b.tangent,cross(d,b.director)),dot(d,b.director)) + turn*2*Math.PI;
    });
    const material = profileMap(profile,x[0],labels[0]);
    return {body,toolId,nodeCoordinates:x,materialLabels:labels,angles,reference,referenceTwists,
        winding:'explicit-unwrapped',velocityInterpretation:'physical-material-velocity',
        angularVelocityInterpretation:'physical-material-angular-velocity',appMaterialProfile:material.appMaterialProfile,
        material:{dsDx:1,massPerMaterialLength:positive(massPerMaterialLength,'Physical mass per reference length'),materialAt:material.materialAt}};
}

/** Import once and preserve original node poses/velocities/winding. Native
 * geometry keeps the original affine edges, inertia and chord constraints.
 * The default continuous variant selects C2 geometry for inertia, material
 * frames and arclength, changing interpolation between the source nodes.
 * It does not manufacture relaxed lengths from a bent source pose.
 */
export function initializeCompositeAppState({tools,time=0,step=0,geometry='continuous-material-frame'} = {}) {
    if(!['continuous-material-frame','native-discrete-rod'].includes(geometry))throw new RangeError('Unknown application Joint geometry');
    const imported = importCompositeJointWorld({tools,time,step});
    const preparedTools = imported.state.tools.map(tool => {
        const source = tools.find(t=>t.toolId===tool.id), spec = source.appMaterialProfile;
        if (!spec) throw new RangeError('Use explicit application tool source descriptors');
        return {...tool,...profileMap(spec,spec.coordinateOrigin,spec.materialOrigin)};
    });
    const continuous=geometry==='continuous-material-frame';
    const state = createCompositeJointTimeStepState({...imported.state,tools:preparedTools,
        inertiaGeometryByTool:continuous?new Map(preparedTools.map(t=>[t.id,{interfaces:[]}])):null,
        elasticityGeometry:geometry,lengthGeometry:continuous?'continuous-arclength':'native-chords'});
    state.materialVelocities = structuredClone(imported.state.materialVelocities);
    return {...imported,state,scope:continuous?'actual-app-initial-continuous-joint-state':'actual-app-initial-native-joint-state',
        initialInterpolation:continuous?'quintic-C2-through-original-active-source-nodes':'original-affine-source-edges',remapReady:false,fullFeedLifecycleReady:false};
}

/** Prepare one retry-stable step from the authoritative accepted Joint state.
 * feedVelocity is physical material speed in the positive chart direction;
 * dsDt=-feedVelocity. labelShift is the actual change of material labels on
 * this fixed chart, not an additional advection of previous velocities.
 * Independent commands must explicitly cover each tool, including zeros.
 * A changing active range still needs a separate state transfer before this.
 */
export function prepareCompositeAppInputs({state,dt,commands,positionBoundaries=[],loads={forces:[],torques:[]},reservoir=null,
    contacts='none',wall='none',tolerances={},budget={}} = {}) {
    positive(dt,'Application dt');
    const ids = state.tools.map(t=>t.id);
    if (!Array.isArray(commands)||commands.length!==ids.length||new Set(commands.map(c=>c.toolId)).size!==ids.length||commands.some(c=>!ids.includes(c.toolId)))
        throw new RangeError('Explicit independent commands must cover every physical tool');
    const byId = new Map(commands.map(c=>[c.toolId,{...c,labelShift:finite(c.labelShift,'Own label shift'),
        feedVelocity:finite(c.feedVelocity,'Own material feed velocity'),spinIncrement:finite(c.spinIncrement,'Own unwrapped handle increment')}])) ;
    const nextTools = state.tools.map(tool => {
        const spec = tool.appMaterialProfile;
        if (!spec) throw new RangeError('Accepted state is missing its own application material map');
        return {...tool,...profileMap(spec,spec.coordinateOrigin,spec.materialOrigin+byId.get(tool.id).labelShift)};
    });
    const next = createCompositeJointTimeStepState({...state,tools:nextTools});
    next.boundaryMultipliers = structuredClone(state.boundaryMultipliers);
    next.materialVelocities = structuredClone(state.materialVelocities);
    const nextMaterialLabel=new Map(),current = state.layout.edgeToolIds.map((ids,e)=>({tools:ids.map(id=> {
        const own = nextTools.find(t=>t.id===id), spec=own.appMaterialProfile;
        // Adjacent affine cells share one stored endpoint. Re-evaluating the
        // origin formula near label zero loses significant bits and can make
        // the same physical boundary appear discontinuous to the core.
        const sStart=nextMaterialLabel.get(id)??spec.materialOrigin+(state.coordinates[e]-spec.coordinateOrigin);
        nextMaterialLabel.set(id,sStart+(state.coordinates[e+1]-state.coordinates[e]));
        return {id,materialMap:{sStart,
            dsDx:1,dsDt:-byId.get(id).feedVelocity}};
    })}));
    // Native affine maps evaluate a shared label as either start+dx or
    // origin+(x-origin). Canonicalize only their Float64 boundary roundoff
    // in the query copy; physical accepted velocity values remain untouched.
    const velocityHistory=structuredClone(state.materialVelocities);let maximumHistoryLabelRoundoffAdjustment=0;
    if(state.elasticityGeometry==='native-discrete-rod')for(const id of ids) {
        const spans=velocityHistory.flatMap(e=>e.tools).filter(t=>t.id===id).sort((a,b)=>a.sStart-b.sStart);
        const snap=(span,key,value)=>{
            const error=Math.abs(span[key]-value);
            if(error<=64*Number.EPSILON*Math.max(1,Math.abs(span[key]),Math.abs(value))) {
                maximumHistoryLabelRoundoffAdjustment=Math.max(maximumHistoryLabelRoundoffAdjustment,error);span[key]=value;
            }
        };
        for(let i=1;i<spans.length;i++) {
            const left=spans[i-1],right=spans[i],error=Math.abs(left.sEnd-right.sStart);
            if(error<=64*Number.EPSILON*Math.max(1,Math.abs(left.sEnd),Math.abs(right.sStart))) {
                maximumHistoryLabelRoundoffAdjustment=Math.max(maximumHistoryLabelRoundoffAdjustment,error);left.sEnd=right.sStart;
            }
        }
        const ownEdges=current.flatMap((entry,e)=>entry.tools.filter(t=>t.id===id).map(t=>({e,map:t.materialMap}))),
            first=ownEdges[0],last=ownEdges.at(-1);
        // The source tip is often exactly label 0, whereas start+dx gives
        // an adjacent Float64 value. This is the same chart boundary, not
        // newly admitted material requiring a distal reservoir.
        snap(spans[0],'sStart',first.map.sStart);
        snap(spans.at(-1),'sEnd',last.map.sStart+(state.coordinates[last.e+1]-state.coordinates[last.e]));
    }
    const history = createCompositeJointMaterialHistory({materialVelocities:velocityHistory,reservoir}),
        prepared = history.prepare({coordinates:state.coordinates,inertiaEdges:current});
    const inertia = {dt,previousPositions:structuredClone(state.toolPositions),inertiaEdges:prepared.inertiaEdges.map(entry=>({tools:entry.tools.map(t=>({
        id:t.id,materialMap:t.materialMap,massPerMaterialLength:nextTools.find(own=>own.id===t.id).massPerMaterialLength,
        oldVelocityPieces:t.pieces.map(p=>p.bernsteinVelocities ? {fractions:p.fractions,interpretation:p.interpretation,bernsteinVelocities:p.bernsteinVelocities} :
            {fractions:p.fractions,oldMaterialVelocities:p.oldMaterialVelocities})
    }))}))};
    const spins = ids.map(toolId=> {
        const command=byId.get(toolId), edge=command.spinEdge ?? state.layout.edgeToolIds.findIndex(own=>own.includes(toolId));
        if (!state.layout.edgeToolIds[edge]?.includes(toolId)) throw new RangeError('Own handle spin boundary must lie on its material');
        return {toolId,edge,value:state.angles.get(toolId)[edge]+command.spinIncrement};
    });
    // Native application directions use one percent of the declared final
    // force budget. This is numerical Newton forcing; nonlinear force,
    // length and contact gates are unchanged and explicit overrides win.
    return {state:next,options:{dt,torsionMode:'quasi-static',contacts,wall,inertia,
        boundaries:{positions:structuredClone(positionBoundaries),spins},loads:structuredClone(loads),
        tolerances:{...(state.elasticityGeometry==='native-discrete-rod'?{linearForce:.01*(tolerances.force??1e-7)}:{}),...tolerances},budget:{...budget}},
        history:{pieceCount:prepared.pieceCount,requiredCuts:prepared.requiredCuts,includesAngularHistory:false,maximumHistoryLabelRoundoffAdjustment},
        scope:'fixed-chart-owned-app-commands-and-accepted-material-history'};
}
