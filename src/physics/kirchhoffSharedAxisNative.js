import {sharedAxisEffectiveGap,sharedAxisContactElasticEnergy} from './kirchhoffSharedAxisCompliance.js';
import {prunableSharedAxisWitnesses} from './kirchhoffSharedAxisWitnessPruning.js';
import {materializeSharedAxisContacts} from './kirchhoffSharedAxisInactiveContacts.js';
import {createSharedAxisModifiedNewton} from './kirchhoffSharedAxisModifiedNewton.js';
import {assembleSharedAxisConstraintRows,sharedAxisPositionDofMask,createSharedAxisConstraintRowPool,snapshotSharedAxisConstraintMeasure} from './kirchhoffSharedAxisConstraintRows.js';
import {preserveSharedAxisBends} from './kirchhoffSharedAxisRemesh.js';
import {adaptiveMeshOptions,retainAdaptiveShapeSamples,coarsenSharedAxisMesh} from './kirchhoffSharedAxisAdaptiveMesh.js';
import {createSharedAxisCycleGuard} from './kirchhoffSharedAxisCycleGuard.js';
import * as THREE from 'three';
import {assembleSharedAxisWallFriction,refreshSharedAxisWallFriction} from './kirchhoffSharedAxisWallFriction.js';
import {measureSharedAxisQuality} from './kirchhoffSharedAxisDiagnostics.js';
import {evaluateSharedAxisBendLimit} from './kirchhoffSharedAxisBendLimit.js';
import { assembleSharedAxisMaterialTangent } from './kirchhoffSharedAxisMaterialTangent.js';
import { assembleSharedAxisInertia } from './kirchhoffSharedAxisDynamics.js';
import { EndovascularRodBody } from './endovascularPhysicsWorld.js';
import { applyKirchhoffMaterialProfile } from './applyKirchhoffMaterialProfile.js';
import { assembleSharedAxisGaussNewton } from './kirchhoffSharedAxisGaussNewton.js';
import { createSharedAxisLayout, createSharedAxisLinear, iterateSharedAxisLinear, getSharedAxisLinearScratchStats } from './kirchhoffSharedAxisLinear.js';
import {iterateSharedAxisProjection} from './kirchhoffSharedAxisProjection.js';
import {createSharedAxisStagnationGuard} from './kirchhoffSharedAxisStagnationGuard.js';

const XYZ = ['x', 'y', 'z'], Q = ['X', 'Y', 'Z', 'W'];
const vec = p => new THREE.Vector3(...p);
// Keep escaping contact callbacks out of the constructor's closure context.
// Its short-lived interpolation callbacks capture `previous`; an evaluator
// sharing that context can otherwise retain the complete chain of old states.
const bendLimitEvaluator=angle=>Object.assign(input=>evaluateSharedAxisBendLimit(input,angle),{sharedAxisGeometryOnly:true});
const frame = (b, e) => new THREE.Quaternion(...Q.map(k => b['orientation' + k][e]));
const tangent = (p, e) => {
    const v = vec(p[e + 1]).sub(vec(p[e]));
    if (!(v.length() > 1e-9)) throw new RangeError('Shared axis needs nondegenerate segments');
    return v.normalize();
};
function writeFrame(b, e, q) { b.orientationX[e] = q.x; b.orientationY[e] = q.y; b.orientationZ[e] = q.z; b.orientationW[e] = q.w; }
function orient(q, direction, spin = 0) {
    const from = new THREE.Vector3(0, 0, 1).applyQuaternion(q);
    if (from.dot(direction) < -1 + 1e-8) throw new RangeError('Shared axis cannot reverse a segment in one trial');
    q.premultiply(new THREE.Quaternion().setFromUnitVectors(from, direction));
    q.multiply(new THREE.Quaternion().setFromAxisAngle(new THREE.Vector3(0, 0, 1), spin));
    return q.normalize();
}
function interpolate(xs, values, x) {
    if (x <= xs[0]) return values[0].slice();
    if (x >= xs.at(-1)) return values.at(-1).slice();
    let i=0,high=xs.length-1;while(i+1<high){const mid=(i+high)>>>1;if(xs[mid]<x)i=mid;else high=mid;}
    const t = (x - xs[i]) / (xs[i + 1] - xs[i]);
    return values[i].map((v, k) => v * (1 - t) + values[i + 1][k] * t);
}

/** Shared-axis Kirchhoff mechanics. Positions are owned
 * ONCE. Each material owns independent frames/spin and its native rest profile.
 * Native Kirchhoff bend/twist strains and Jacobians are pulled back through
 * the hard director=tangent kinematics. Duplicate adaptation equations become
 * one length constraint per spatial edge. There are no inter-tool rows.
 * Dynamic feed/history transactions are supplied by SharedAxisTimeStep.
 */
export function createSharedAxisNative({ tools, spacing = 5, samplePosition = x => [x, 0, 0],
    wallSample = null, wallSamples = wallSample ? [wallSample] : [], startCoordinate = 0, boundaryCoordinates = [], previous = null, minimumEdgeLength=.5, origin=[0,0,0], maxBendAngle=Infinity, rebaseNearTips=false, fractionalTipThreshold=1e-3, spatialKnots=[], adaptiveMesh=null }) {
    adaptiveMesh=adaptiveMeshOptions(adaptiveMesh);
    if (!Array.isArray(tools) || !tools.length || tools.length > 2 || new Set(tools.map(t => t.id)).size !== tools.length)
        throw new TypeError('One or two uniquely named materials are required');
    if (!(spacing > 0) || !Number.isFinite(spacing)) throw new RangeError('Positive finite spacing required');
    if (!Number.isFinite(startCoordinate) || startCoordinate > 0 || !boundaryCoordinates.every(Number.isFinite)) throw new RangeError('Invalid shared axis domain');
    for (const t of tools) if (!['wire', 'catheter'].includes(t.id) || !(t.insertion >= 0 && t.insertion > startCoordinate) ||
        !Number.isFinite(t.insertion) || t.insertion > (t.length ?? 1000)) throw new RangeError('Invalid material coverage');
    if (!Number.isFinite(minimumEdgeLength) || minimumEdgeLength < 0 || minimumEdgeLength > spacing) throw new RangeError('Invalid minimum edge length');
    if (!(maxBendAngle > 0 && maxBendAngle <= Math.PI) && maxBendAngle !== Infinity) throw new RangeError('Invalid bend limit');
    if(!(fractionalTipThreshold>=0&&Number.isFinite(fractionalTipThreshold)))throw new RangeError('Invalid fractional tip threshold');
    const end = Math.max(...tools.map(t => t.insertion));
    if(!Array.isArray(spatialKnots)||!spatialKnots.every(x=>Number.isFinite(x)&&x>=startCoordinate&&x<=end))throw new RangeError('Invalid spatial knots');
    const physicalKnots=tools.map(t=>t.insertion).filter(x=>x===end||end-x>=fractionalTipThreshold);
    const knots = [startCoordinate, end, ...physicalKnots, ...boundaryCoordinates.filter(x => x > startCoordinate && x < end && !tools.some(t=>Math.abs(t.insertion-x)<minimumEdgeLength))];
    const boundaries=knots.slice();
    // An adaptive replay supplies the exact mechanical grid. Ordinary feed
    // rebuilds fine candidates before applying its spatial error budget.
    for (let x = Math.ceil(startCoordinate / spacing) * spacing; !(adaptiveMesh&&spatialKnots.length)&&x < end; x += spacing)
        if (x > startCoordinate&&!boundaries.some(b=>Math.abs(b-x)<minimumEdgeLength)) knots.push(x);
    // Serialization may supply retained geometry knots. They are not fixed
    // boundaries and feed reconsiders them using the accepted bend geometry.
    knots.push(...spatialKnots);
    const coordinates = [];
    for (const x of knots.sort((a,b) => a-b)) {
        if (!coordinates.length || x - coordinates.at(-1) > 1e-9) coordinates.push(x);
        else coordinates[coordinates.length-1] = x; // merge floating-point copies of one boundary
    }
    if (coordinates.length < 3) throw new RangeError('At least two spatial edges are required');
    const positions = coordinates.map(x => {
        if (previous && x <= previous.coordinates.at(-1)) return interpolate(previous.coordinates, previous.positions, x);
        if (previous) {
            const tip = previous.positions.at(-1), dir = tangent(previous.positions, previous.positions.length - 2);
            return vec(tip).addScaledVector(dir, x - previous.coordinates.at(-1)).toArray();
        }
        const p = samplePosition(x);
        if (p.length !== 3 || !p.every(Number.isFinite)) throw new RangeError('Finite spatial positions required');
        return p.slice();
    });
    preserveSharedAxisBends(coordinates,positions,previous,maxBendAngle);
    if(adaptiveMesh&&!spatialKnots.length)retainAdaptiveShapeSamples(coordinates,positions,previous,adaptiveMesh);
    if(adaptiveMesh&&!spatialKnots.length)coarsenSharedAxisMesh(coordinates,positions,
        {tools,boundaries,previous,spacing,options:adaptiveMesh});
    const edgeTools = coordinates.slice(0,-1).map(x => tools.filter(t => x < t.insertion-1e-12).map(t => t.id));
    const layout = createSharedAxisLayout(edgeTools);
    const materials = tools.map(input => {
        const spec = { ...input }, last = coordinates.findIndex(x => x>=input.insertion-1e-12);
        if (last < 2) throw new RangeError('Each prototype material needs at least two edges');
        const body = new EndovascularRodBody(input.id, last + 1, spacing, { mass: 1, radius: input.radius ?? (input.id === 'catheter' ? .8 : .4445) });
        // Moving tips create non-integer cells; do not quantize their lengths.
        body.restLength = Float64Array.from(body.restLength);
        const coveredCoordinates=coordinates.slice(0,last+1).map(x=>Math.min(x,input.insertion));
        const materialCoordinates = coveredCoordinates.map(x => (input.length ?? 1000) + (x - input.insertion));
        applyKirchhoffMaterialProfile(body, input.type ?? (input.id === 'catheter' ? 'berenstein' : 'glidewire'), {
            materialCoordinates, tipCoordinate: input.length ?? 1000,
            shaftStiffnessScale: input.shaftStiffness ?? 1, tipStiffnessScale: input.tipStiffness ?? 1
        });
        const old = previous?.materials.find(t => t.spec.id === input.id);
        const oldCenters=old?Array.from({length:old.body.segmentCount},(_,i)=>(old.body.materialCoordinate[i]+old.body.materialCoordinate[i+1])/2):null;
        let oldFrameIndex=0;
        for (let e = 0; e < last; e++) {
            body.restLength[e] = coveredCoordinates[e + 1] - coveredCoordinates[e];
            let q;
            if (old && e === 0) q = frame(old.body, 0); // independent prescribed proximal rotation
            else if (old) {
                const s = (materialCoordinates[e] + materialCoordinates[e + 1]) / 2;
                const centers=oldCenters;
                while(oldFrameIndex+1<centers.length&&centers[oldFrameIndex+1]<=s)oldFrameIndex++;
                const i=oldFrameIndex;
                q = frame(old.body, i);
                if (i + 1 < centers.length && s > centers[i]) q.slerp(frame(old.body, i + 1), (s - centers[i]) / (centers[i + 1] - centers[i]));
            } else q = new THREE.Quaternion().setFromUnitVectors(new THREE.Vector3(0, 0, 1), tangent(positions, e));
            writeFrame(body, e, orient(q, tangent(positions, e)));
        }
        if (input.configure) input.configure(body, coveredCoordinates);
        const endFraction=(input.insertion-coordinates[last-1])/(coordinates[last]-coordinates[last-1]);
        return { spec, body, last, endFraction, coordinates:coveredCoordinates };
    });
    const definitions = [];
    for (let e = 0; e < edgeTools.length; e++) {
        const dofs = [layout.positions[e], layout.positions[e + 1]].flatMap(i => [i, i + 1, i + 2]);
        definitions.push({ kind: 'length', edge: e, dofs });
        wallSamples.forEach((sampler, sample) => {
            definitions.push({ kind: 'wall', edge: e, dofs, sample });
            const catheter=materials.find(m=>m.spec.id==='catheter'&&m.last-1===e&&m.endFraction<1);
            if(sampler.sharedAxisSheath&&catheter)definitions.push({kind:'wall',edge:e,dofs,sample,sampleT:catheter.endFraction});
        });
    }
    const evaluateBendLimit=bendLimitEvaluator(maxBendAngle);
    if (Number.isFinite(maxBendAngle)) for(let e=0;e+1<edgeTools.length;e++) definitions.push({kind:'wall',subtype:'bend-limit',edge:e,
        id:`bend/${coordinates[e]}/${coordinates[e+1]}/${coordinates[e+2]}`,
        dofs:[layout.positions[e],layout.positions[e+1],layout.positions[e+2]].flatMap(i=>[i,i+1,i+2]),
        evaluate:evaluateBendLimit});
    const state = { adaptiveMesh, fractionalTipThreshold, rebaseNearTips, geometryKey:Symbol('pose'), maxBendAngle, kind: 'shared-axis-native', spacing, minimumEdgeLength, origin, coordinates, positions, materials, layout, wallSample, wallSamples, startCoordinate, boundaryCoordinates,
        definitions, definitionIds: new Set(definitions.map(row=>row.id)), mixed: createSharedAxisLinear(layout, definitions,{lazy:true}),
        chain: { layout, hessian: new Float64Array(layout.dofCount * layout.band), gradient: new Float64Array(layout.dofCount), hessianValid: true },
        multipliers: new Float64Array(definitions.length), fixed: new Uint8Array(layout.dofCount), loads: new Float64Array(layout.dofCount),
        acceptedSolves: 0, interToolRows: 0 };
    for (const node of [0, 1]) for (let a = 0; a < 3; a++) state.fixed[layout.positions[node] + a] = 1;
    for (const spin of layout.spins.values()) state.fixed[spin[0]] = 1;
    syncNativePositions(state);
    return state;
}

/** Physical material coverage within a spatial cell. Almost coincident tips
 * retain exact material endpoints on one affine spatial segment. */
export function sharedAxisOuterMaterialAt(s,edge,t=1,preferredOwner=null) {
    const catheter=s.materials.find(m=>m.spec.id==='catheter'&&edge<m.last);
    if(catheter) {
        const end=edge===catheter.last-1?catheter.endFraction:1;
        if(t<end||t===end&&preferredOwner!=='wire'||end===1)return catheter;
    }
    return s.materials.find(m=>m.spec.id==='wire'&&edge<m.last)??catheter;
}
export function sharedAxisOuterIntervals(s,edge) {
    const catheter=s.materials.find(m=>m.spec.id==='catheter'&&edge<m.last);
    if(catheter&&edge===catheter.last-1&&catheter.endFraction<1) {
        const wire=s.materials.find(m=>m.spec.id==='wire'&&edge<m.last);
        return [{start:0,end:catheter.endFraction,material:catheter},{start:catheter.endFraction,end:1,material:wire}];
    }
    return [{start:0,end:1,material:sharedAxisOuterMaterialAt(s,edge)}];
}

function syncNativePositions(s) {
    for (const { body, last, endFraction } of s.materials) for (let n = 0; n <= last; n++)
        for (let a = 0; a < 3; a++) body[XYZ[a]][n] = n===last&&endFraction<1?
            (1-endFraction)*s.positions[n-1][a]+endFraction*s.positions[n][a]:s.positions[n][a];
}

/** Material feeds are independent. A candidate preserves the common geometry
 * on the old domain, transports each material's own frame field, and samples
 * that material's rest profile at its new material labels. The accepted state
 * is untouched. New topology must pass relaxation/geometry checks separately. */
export function feedSharedAxisNative(s, insertionById, {pruneInactiveWitnesses=false}={}) {
    // Resolve a tiny interval between physical tips in a local chart near
    // those tips. No endpoint, material coordinate or world-space point is
    // snapped. Subtracting ~300 mm coordinates to recover a .01 mm tangent
    // otherwise amplifies floating-point error through the bending stiffness.
    const tips=s.materials.map(m=>insertionById[m.spec.id]??m.spec.insertion);
    if(s.rebaseNearTips&&tips.length===2&&Math.abs(tips[0]-tips[1])<1) {
        const shift=s.positions.at(-1);
        if(shift.some(v=>v!==0))s={...s,origin:s.origin.map((v,k)=>v+shift[k]),positions:s.positions.map(p=>p.map((v,k)=>v-shift[k]))};
    }
    const candidate = createSharedAxisNative({ tools: s.materials.map(t => ({ ...t.spec,
        insertion: insertionById[t.spec.id] ?? t.spec.insertion })), spacing: s.spacing, wallSamples: s.wallSamples,
        startCoordinate: s.startCoordinate, boundaryCoordinates: s.boundaryCoordinates, previous: s, minimumEdgeLength:s.minimumEdgeLength, origin:s.origin, maxBendAngle:s.maxBendAngle,rebaseNearTips:s.rebaseNearTips,fractionalTipThreshold:s.fractionalTipThreshold,adaptiveMesh:s.adaptiveMesh });
    // Exact surviving rows keep their reaction as an initial guess. Adaptive
    // remeshing also transfers loaded physical wall sites; new sites start
    // unloaded. Acceptance still checks the complete contact/friction law.
    const key = (state, row) => `${row.kind}/${row.sample ?? ''}/${row.sampleT ?? ''}/${row.id ?? ''}/${state.coordinates[row.edge]}/${state.coordinates[row.edge+1]}`;
    const remappedReactions=new Map();
    const dropped=pruneInactiveWitnesses?prunableSharedAxisWitnesses(s):null;
    const retained = s.definitions.flatMap((r,i) => {
        if(!r.evaluate||r.subtype==='bend-limit'||!(s.multipliers[i]!==0||!(s.acceptedWallGaps?.get(r.id)>.75)))return [];
        if(dropped?.has(r))return [];
        const a=s.coordinates[r.edge],b=s.coordinates[r.edge+1];
        const edge=candidate.coordinates.findIndex((x,i)=>x===a&&candidate.coordinates[i+1]===b);
        if(edge<0) {
            // Only loaded sites carry physical friction memory across a mesh
            // change. Rediscover unloaded sites; retaining every old tip site
            // would accumulate submillimetre constraints along the feed path.
            if(!s.adaptiveMesh||!r.witness||!r.evaluate.retarget||!(s.multipliers[i]>1e-10))return [];
            const site=a+(b-a)*r.witness.t;
            const nextEdge=candidate.coordinates.findIndex((x,j)=>x<=site&&candidate.coordinates[j+1]>=site);
            if(nextEdge<0)return [];
            const ca=candidate.coordinates[nextEdge],cb=candidate.coordinates[nextEdge+1],t=(site-ca)/(cb-ca);
            const owner=r.witness.owner;
            const id=`vessel/${ca}/${cb}/${t}/${r.witness.face}${owner?'/'+owner:''}`;
            const row=r.evaluate.retarget({...r,id,edge:nextEdge,witness:{...r.witness,t},
                dofs:[candidate.layout.positions[nextEdge],candidate.layout.positions[nextEdge+1]].flatMap(p=>[p,p+1,p+2])});
            remappedReactions.set(id,s.multipliers[i]);
            return [row];
        }
        return [{...r,edge,dofs:[candidate.layout.positions[edge],candidate.layout.positions[edge+1]].flatMap(i=>[i,i+1,i+2])}];
    });
    extendSharedAxisNativeRows(candidate,retained);
    candidate.mixed.activeWorkspaces=s.mixed.activeWorkspaces;
    const oldRows = new Map(s.definitions.map((row, i) => [key(s, row), s.multipliers[i]]));
    candidate.definitions.forEach((row, i) => { candidate.multipliers[i] = remappedReactions.get(row.id) ?? oldRows.get(key(candidate,row)) ?? 0; });
    candidate.wallFrictionHistory=s.wallFrictionHistory;
    if(s.velocities)candidate.velocities=candidate.coordinates.map(x=>interpolate(s.coordinates,s.velocities,x));
    if(s.angularVelocities)candidate.angularVelocities=Object.fromEntries(candidate.materials.map(({spec,body,last})=>{
        const old=s.materials.find(t=>t.spec.id===spec.id),centers=Array.from({length:old.last},(_,i)=>(old.body.materialCoordinate[i]+old.body.materialCoordinate[i+1])/2);
        return [spec.id,Array.from({length:last},(_,i)=>interpolate(centers,s.angularVelocities[spec.id],(body.materialCoordinate[i]+body.materialCoordinate[i+1])/2))];
    }));
    return candidate;
}

export function extendSharedAxisNativeRows(s, rows) {
    if(!rows.length)return;
    s.definitions.push(...rows);
    for(const row of rows)s.definitionIds.add(row.id);
    const multipliers=new Float64Array(s.definitions.length);multipliers.set(s.multipliers);s.multipliers=multipliers;
    const cache=s.mixed.activeWorkspaces;
    s.mixed=createSharedAxisLinear(s.layout,s.definitions,{lazy:true});s.mixed.activeWorkspaces=cache;
    s.projectionMixed=null;
}

export function rotateSharedAxisNative(s, id, angle) {
    if (!Number.isFinite(angle)) throw new RangeError('Finite rotation required');
    const t = s.materials.find(t => t.spec.id === id);
    if (!t) throw new RangeError('Unknown tool');
    s.geometryKey=Symbol('rotated-pose');
    writeFrame(t.body, 0, orient(frame(t.body, 0), tangent(s.positions, 0), angle));
}

/** Pull back the EXISTING native material rows, including each material's
 * intrinsic curvature. All world position variables belong to the chain;
 * only the spin about each material director remains independent. */
export function assembleSharedAxisNative(s, { tangentMode = s.materialTangent ?? 'newton', withTangent=true, promotion=null, wasmMaterial=false,reuseMaterialScratch=false,reuseConstraintWork=false,rowStorage=null,cullInactiveContacts=false,earlyContactPreflight=false,retainFrictionBase=false } = {}) {
    const { layout, chain } = s, { hessian: H, gradient: g } = chain;
    H.fill(0); g.set(s.loads, 0); for (let i = 0; i < g.length; i++) g[i] = -g[i];
    let energy = 0;
    syncNativePositions(s);
    // This cache is private to one nonlinear solve, whose material parameters
    // are immutable. It never survives a timestep, cancellation or remesh.
    const promote=promotion&&withTangent&&promotion.key===s.geometryKey&&promotion.dynamicStep===s.dynamicStep&&
        promotion.materials===s.materials&&promotion.tangentMode===tangentMode;
    const capture=promotion&&!withTangent&&tangentMode!=='gauss-newton';
    if(capture){promotion.key=null;promotion.hinges??=[];}

    const rowOptions={withTangent:withTangent&&!(reuseConstraintWork&&tangentMode==='gauss-newton'),outerMaterialAt:sharedAxisOuterMaterialAt,reuseConstraintWork,storage:rowStorage,cullInactiveContacts,
        retainWallHessians:!!promotion&&!(reuseConstraintWork&&tangentMode==='gauss-newton')};
    let prepared=null;
    // Only declared geometry-only evaluators may run before material/friction.
    // Use the same row order and the same cache as ordinary assembly. A failed
    // preflight is replayed at its original row after material/friction so that
    // outside-vessel errors and partial discovery retain their precedence.
    if(earlyContactPreflight&&rowStorage&&!cullInactiveContacts&&s.geometryKey!==undefined&&
        (s.cacheMechanicalAssembly||rowOptions.retainWallHessians)&&
        s.wallSamples.some(sample=>sample.sharedAxisDiscovery)&&
        s.definitions.every(d=>d.kind==='length'||(d.evaluate??s.wallSamples[d.sample])?.sharedAxisGeometryOnly)) {
        prepared={contacts:rowStorage.preparedContacts??=[]};
        prepared.contacts.length=s.definitions.length;
        try{assembleSharedAxisConstraintRows(s,{...rowOptions,prepareOnly:prepared});}
        catch(error){prepared.error=error;}
        if(!prepared.error&&s.pendingVesselRows?.size) {
            // A rejected assembly cannot leave an old friction certificate
            // available for publication, even though its forces were skipped.
            const friction=s.wallFrictionStep;
            if(friction){friction.certified=false;friction.certificate=null;if(friction.liveNormalLoad)friction.normalForceColumns=[];}
            if(s.cacheMechanicalAssembly&&withTangent) {
                const cached=s.mechanicalAssemblyCache;
                if(!cached||cached.key!==s.geometryKey||cached.dynamicStep!==s.dynamicStep||cached.tangentMode!==tangentMode)
                    s.mechanicalAssemblyCache={key:s.geometryKey,dynamicStep:s.dynamicStep,tangentMode,deferred:true};
            }
            chain.hessianValid=false;
            throw new Error('shared-axis-wall-discovery');
        }
    }

    // In the primal spring formulation the normal reaction is a function of
    // the current pose, including every trial pose. Evaluate it before friction
    // uses the normal load. Its derivative is eliminated by contact condensation.
    if(s.primalCompliantContacts) {
        if(!prepared) {
            prepared={contacts:[]};
            assembleSharedAxisConstraintRows(s,{...rowOptions,prepareOnly:prepared});
        }
        if(prepared.error)throw prepared.error;
        for(let i=0;i<s.definitions.length;i++)if(s.definitions[i].witness)
            s.multipliers[i]=Math.max(0,-prepared.contacts[i].gap/s.wallCompliance);
    }

    const cache=s.cacheMechanicalAssembly&&s.mechanicalAssemblyCache;
    const mechanicalMatches=cache&&withTangent&&cache.key===s.geometryKey&&cache.dynamicStep===s.dynamicStep&&cache.tangentMode===tangentMode;
    const mechanicalReused=mechanicalMatches&&!cache.deferred;
    if(mechanicalReused) {
        energy=cache.energy;for(let i=0;i<g.length;i++)g[i]=cache.gradient[i]-s.loads[i];
        H.set(cache.hessian);chain.tangent=cache.tangent?.slice()??null;
    } else {
    if (tangentMode !== 'gauss-newton') energy = assembleSharedAxisMaterialTangent(s,withTangent,promote?{reuse:promotion.hinges}:capture?{capture:promotion.hinges}:null,wasmMaterial,reuseMaterialScratch);
    else energy = assembleSharedAxisGaussNewton(s,withTangent);
    energy+=assembleSharedAxisInertia(s,withTangent);
    if(s.cacheMechanicalAssembly&&withTangent) {
        s.mechanicalAssemblyCache={key:s.geometryKey,dynamicStep:s.dynamicStep,tangentMode,energy,
            gradient:Float64Array.from(g,(v,i)=>v+s.loads[i]),hessian:H.slice(),tangent:chain.tangent?.slice()??null};
        // The reference would have cached this pose before its discovery
        // restart. Reproduce its load add/subtract rounding (including -0)
        // when realizing that deferred cache for the first time.
        if(mechanicalMatches&&cache.deferred)for(let i=0;i<g.length;i++)g[i]=s.mechanicalAssemblyCache.gradient[i]-s.loads[i];
    }
    }
    const frictionBase=retainFrictionBase?{key:s.geometryKey,dynamicStep:s.dynamicStep,materials:s.materials,
        loads:s.loads.slice(),gradient:g.slice(),energy}:null;
    energy+=assembleSharedAxisWallFriction(s,withTangent);
    // Gauss-Newton discards geometric constraint Hessians in its direction.
    // Keep the same gaps, Jacobians and current reactions without building
    // those unused matrices (or copying all rows merely to remove them).
    const rows=assembleSharedAxisConstraintRows(s,{...rowOptions,prepared});
    if(withTangent)for(const column of s.wallFrictionStep?.normalForceColumns??[]) {
        rows[column.rowIndex].extraForceDofs=column.dofs;
        rows[column.rowIndex].extraForceJacobian=column.values;
    }
    if(s.pendingVesselRows?.size)throw new Error('shared-axis-wall-discovery');
    if(s.wallCompliance) {
        const elastic=sharedAxisContactElasticEnergy(rows);energy+=elastic;
        if(frictionBase)frictionBase.energy+=elastic;
    }
    let force = 0, torque = 0, constraint = 0;
    const positionDofs = sharedAxisPositionDofMask(layout);
    for (let i = 0; i < g.length; i++) if (!s.fixed[i]) {
        if (positionDofs[i]) force = Math.max(force, Math.abs(g[i])); else torque = Math.max(torque, Math.abs(g[i]));
    }
    rows.forEach((r, i) => { constraint = Math.max(constraint, Math.abs(s.definitions[i].kind === 'length' ? r.gap : Math.max(0, r.multiplier - sharedAxisEffectiveGap(r)) - r.multiplier)); });
    if(capture){promotion.key=s.geometryKey;promotion.dynamicStep=s.dynamicStep;promotion.materials=s.materials;promotion.tangentMode=tangentMode;}
    if(promote&&!mechanicalReused)promotion.hits++;
    chain.energy = energy; chain.hessianValid = withTangent;
    return { rows, energy, force, torque, constraint, ...(frictionBase?{frictionBase}:{}) };
}

/** Refresh only the changed wall law at an accepted, unchanged pose. The
 * material/inertia gradient is owned by this measure; normal forces are added
 * in their original order. Never subtract large assembled forces to obtain a
 * small residual. A subsequent direction still builds its complete tangent. */
export function refreshSharedAxisFrictionMeasure(s,base) {
    const saved=base.frictionBase;
    if(!saved||saved.key!==s.geometryKey||saved.dynamicStep!==s.dynamicStep||saved.materials!==s.materials||
        saved.loads.length!==s.loads.length||saved.loads.some((v,i)=>v!==s.loads[i])||
        base.rows.length!==s.definitions.length||s.pendingVesselRows?.size||
        base.rows.some((r,i)=>r.multiplier!==s.multipliers[i])||
        s.definitions.some(d=>d.kind!=='length'&&!(d.evaluate??s.wallSamples[d.sample])?.sharedAxisGeometryOnly))return null;
    const g=s.chain.gradient;g.set(saved.gradient);
    const energy=saved.energy+assembleSharedAxisWallFriction(s,false);
    for(const r of base.rows) {
        const sign=r.kind==='wall'?-1:1;
        for(let i=0;i<r.dofs.length;i++)g[r.dofs[i]]+=sign*r.jacobian[i]*r.multiplier;
    }
    let force=0,torque=0;
    const positions=sharedAxisPositionDofMask(s.layout);
    for(let i=0;i<g.length;i++)if(!s.fixed[i]) {
        if(positions[i])force=Math.max(force,Math.abs(g[i]));else torque=Math.max(torque,Math.abs(g[i]));
    }
    s.chain.energy=energy;s.chain.hessianValid=false;
    return {...base,energy,force,torque};
}

export function captureSharedAxisNative(s) {
    return { geometryKey:s.geometryKey, positions: s.positions.map(p => p.slice()), frames: s.materials.map(({ body }) => Q.map(k => body['orientation' + k].slice())), multipliers: s.multipliers.slice() };
}
export function restoreSharedAxisNative(s, saved) {
    s.geometryKey=saved.geometryKey??Symbol('restored-pose');
    s.positions.forEach((p, i) => { for (let a = 0; a < 3; a++) p[a] = saved.positions[i][a]; });
    s.materials.forEach(({ body }, i) => Q.forEach((k, j) => body['orientation' + k].set(saved.frames[i][j])));
    s.multipliers.fill(0);s.multipliers.set(saved.multipliers); syncNativePositions(s);
}
export function applySharedAxisNativeIncrement(s, increment, multiplierIncrement, scale = 1) {
    s.geometryKey=Symbol('pose');
    for (let n = 0; n < s.positions.length; n++) for (let a = 0; a < 3; a++)
        s.positions[n][a] += scale * increment[s.layout.positions[n] + a];
    for (const { body, spec, last } of s.materials) for (let e = 0; e < last; e++)
        writeFrame(body, e, orient(frame(body, e), tangent(s.positions, e), scale * increment[s.layout.spins.get(spec.id)[e]]));
    s.multipliers.forEach((v, i) => { s.multipliers[i] = v + scale * multiplierIncrement[i]; });
    syncNativePositions(s);
}

function* correctTrialConstraints(s,base,reuseStructure,projectionMode,reuseConstraintWork,reuseMatrixAssembly) {
    materializeSharedAxisContacts(base.rows);
    if(s.primalCompliantContacts)base={...base,rows:base.rows.filter(r=>!r.compliance)};
    if(s.wallCompliance)base={...base,rows:base.rows.map(r=>({...r,gap:sharedAxisEffectiveGap(r),compliance:0}))};
    // Second-order SQP correction: restore the nonlinear lengths and gaps
    // after a finite tangent step. This is not an accepted physical update;
    // the caller still certifies energy and the full physical residual.
    if(projectionMode) {
        const direction=yield* iterateSharedAxisProjection(s,base,{reuseStructure,mode:projectionMode,reuseConstraintWork,reuseMatrixAssembly});
        if(direction.converged)applySharedAxisNativeIncrement(s,direction.increment,direction.zeroReactions);
        return direction;
    }
    const chain={layout:s.layout,hessian:new Float64Array(s.layout.dofCount*s.layout.band)};
    for(let i=0;i<s.layout.dofCount;i++)chain.hessian[i*s.layout.band]=1;
    const w=s.projectionMixed??=createSharedAxisLinear(s.layout,s.definitions,{lazy:true});
    const direction=yield* iterateSharedAxisLinear(w,chain,{fixed:s.fixed,gradient:new Float64Array(s.layout.dofCount),
        rows:base.rows.map(r=>({...r,multiplier:0,geometricHessian:undefined,extraForceDofs:undefined,extraForceJacobian:undefined})),tolerance:1e-10,reuseStructure,reuseConstraintWork,reuseMatrixAssembly});
    if(direction.converged)applySharedAxisNativeIncrement(s,direction.increment,new Float64Array(s.multipliers.length));
    return direction;
}

// incrementalContacts is an opt-in research path. Full-step timing and strict
// Pigtail parity gates have NOT passed; keep the application default false.
export function* iterateSharedAxisNative(s, { maxIterations = 160, forceTolerance = 1e-6, lengthTolerance = 1e-5, linearToleranceCap = Infinity, observeIteration = null, observeTrial = null, newtonActiveSetLimit = 16, batchActivationSize=8, simultaneousContactRelease=false, condenseCompliantContacts=false, primalCompliantContacts=false, stagnationResidualSearch=false, localContactRestarts = true, reuseWorkingSet = true,reuseStructure=true,earlyLiveFallback=false,lazyTrialTangent=false,observeLinearSystem=null,incrementalContacts=false,promoteTrialAssembly=false,projectionMode=false,stagnationFallback=false,wasmMaterial=false,wasmLinearAssembly=false,reuseMaterialScratch=false,reuseConstraintWork=false,reuseMatrixAssembly=false,reuseRowBuffers=false,lazyBasisCoefficients=false,deferActiveBasis=false,modifiedNewton=false,cullInactiveContacts=false,earlyContactPreflight=false,deferContactPublication=null,coupledFrictionNewton=false,batchRelease=false,reuseDiscoveryTrial=coupledFrictionNewton,zeroDualStart=false,reuseFrictionAssembly=false } = {}) {
    if (!Number.isInteger(maxIterations) || maxIterations < 0 ||
        !(linearToleranceCap>0&&(Number.isFinite(linearToleranceCap)||linearToleranceCap===Infinity)) ||
        ![forceTolerance, lengthTolerance].every(v => Number.isFinite(v) && v > 0) ||
        !(newtonActiveSetLimit === Infinity || (Number.isInteger(newtonActiveSetLimit) && newtonActiveSetLimit > 0))) throw new RangeError('Invalid shared axis convergence options');
    if(primalCompliantContacts&&(!s.primalCompliantContacts||!s.wallCompliance||modifiedNewton||incrementalContacts))throw new RangeError('Primal contact solve requires a compliant timestep');
    const initial = captureSharedAxisNative(s), started = performance.now();
    let lastOutsideContact=null;
    let candidate = null, coupledFrictionRefreshes=0, frictionRefreshMs=0, retainedDiscoveryTrials=0;
    const modified=modifiedNewton?createSharedAxisModifiedNewton():null;
    let cycleGuard=null,detectedCycle=null,stagnationGuard=null,detectedStagnation=null;
    let residualGuard=null,residualSearchActive=false,residualSearchActivation=null;
    const timings = { assemblyMs: 0, linearMs: 0, tangentAssemblyMs:0, residualAssemblyMs:0, projectionMs:0 };
    let fullAssemblies=0,residualAssemblies=0;
    const promotion=promoteTrialAssembly?{hits:0}:null;
    // Friction iterations may run several nonlinear solves at one pose. Keep
    // private contact derivatives for the same lifetime as wallGeometryCache;
    // row banks themselves remain exclusive to this nonlinear invocation.
    const rowPool=reuseRowBuffers?createSharedAxisConstraintRowPool(s.bufferedWallGeometryCache??=new Map()):null;
    let base=null,protectedTrial=null;
    const observed=m=>rowPool?snapshotSharedAxisConstraintMeasure(m):m;
    const assemble = (tangentMode,withTangent=true) => {
        const start = performance.now();
        try{return assembleSharedAxisNative(s,{retainFrictionBase:reuseFrictionAssembly,tangentMode,withTangent,promotion,wasmMaterial,reuseMaterialScratch,reuseConstraintWork,earlyContactPreflight,cullInactiveContacts:cullInactiveContacts&&!modifiedNewton&&!incrementalContacts,rowStorage:rowPool?.acquire([base?.rows,protectedTrial?.rows])});}
        finally{const elapsed=performance.now()-start;timings.assemblyMs+=elapsed;
            timings[withTangent?'tangentAssemblyMs':'residualAssemblyMs']+=elapsed;
            if(withTangent)fullAssemblies++;else residualAssemblies++;}
    };
    let iterations = 0, factorizations = 0, workingSetReuses = 0, backtracks = 0, discoveryIterations = 0, geometryRestarts = 0, error = null, status = 'iteration-limit';
    const discover = error => {
        if(!localContactRestarts || error.message !== 'shared-axis-wall-discovery' || !s.pendingVesselRows?.size || geometryRestarts >= 64)return false;
        extendSharedAxisNativeRows(s,[...s.pendingVesselRows.values()]);s.pendingVesselRows.clear();modified?.invalidate();geometryRestarts++;cycleGuard=null;stagnationGuard=null;residualGuard=null;return true;
    };
    const assembleDiscovered = () => {
        while(true) {try{return assemble();}catch(error){if(!discover(error))throw error;}}
    };
    const merit = m => Math.max(m.force / forceTolerance, m.torque / forceTolerance, m.constraint / lengthTolerance);
    const violation = m => m.rows.reduce((v, r) => v + (r.kind === 'length' ? Math.abs(r.gap) : Math.max(0, -sharedAxisEffectiveGap(r))), 0);
    try {
        base = assembleDiscovered();
        for (; iterations < maxIterations; iterations++) {
            yield {kind:'iteration',iteration:iterations};
            observeIteration?.({state:s,iteration:iterations,base:observed(base)});
            if (merit(base) <= 1) { status = 'converged'; break; }
            // Ordinary successful solves retain exactly their original path.
            // Switch globalization only after two windows show no progress,
            // including frozen-friction solves that the live guard cannot see.
            if(stagnationResidualSearch&&!residualSearchActive) {
                const stalled=(residualGuard??=createSharedAxisStagnationGuard(forceTolerance,lengthTolerance)).observe(base);
                if(stalled) {
                    residualSearchActive=true;residualSearchActivation={iteration:iterations,...stalled};
                    modified?.invalidate();
                    observeTrial?.({kind:'residual-search-activation',iteration:iterations,...stalled});
                }
            }
            if(!residualSearchActive&&earlyLiveFallback&&iterations>=8&&s.wallFrictionStep?.liveNormalLoad===true) {
                detectedCycle=(cycleGuard??=createSharedAxisCycleGuard()).observe(s,base);
                if(detectedCycle){status='live-contact-cycle';break;}
                if(stagnationFallback) {
                    detectedStagnation=(stagnationGuard??=createSharedAxisStagnationGuard(forceTolerance,lengthTolerance)).observe(base);
                    if(detectedStagnation){status='live-contact-stagnation';break;}
                }
            }
            // Keep active-set precision independent of an experimental looser
            // nonlinear stopping budget; retain the roundoff-scaled floor.
            const linearTolerance=Math.max(Math.min(linearToleranceCap,Math.min(forceTolerance,lengthTolerance)*.01),1e-10*Math.max(base.force,base.torque,base.constraint));
            const reuseTangent=modified?.canReuse(base.rows,s.fixed,linearTolerance)??false;
            if(!reuseTangent&&!s.chain.hessianValid)base=assemble();
            const snapshot = captureSharedAxisNative(s);
            try {
            let accepted = false, solvedDirection = false, attemptedTrial = false;
            // A full Newton tangent is much faster near equilibrium but can
            // be indefinite after a large feed. Abandon an unproductive active
            // set search early, then try the positive Gauss-Newton tangent. The
            // original unrestricted Newton search remains the final fallback.
            const methods=[...(reuseTangent?[-1]:[]),0,1,...(newtonActiveSetLimit===Infinity?[]:[2])];
            for (const method of methods) {
                if(accepted)break;
                const isGaussNewton = method === 1, isModified=method === -1;
                if(!isModified)modified?.invalidate();
                if (method !== methods[0] || isGaussNewton) { restoreSharedAxisNative(s, snapshot); base = assemble(isGaussNewton ? 'gauss-newton' : undefined); }
                const linearStart = performance.now();
                const linearOptions = { batchActivationSize, simultaneousContactRelease, primalCompliantContacts, condenseCompliantContacts:condenseCompliantContacts||primalCompliantContacts, wasmLinearAssembly, lazyBasisCoefficients, deferActiveBasis, zeroDualStart, batchRelease:batchRelease||coupledFrictionNewton, rows: isGaussNewton&&!reuseConstraintWork ? base.rows.map(r => ({ ...r, geometricHessian: undefined })) : base.rows, gradient: s.chain.gradient,
                    observeLinearSystem,modifiedNewtonContext:method===0?modified:null,incrementalContacts:modified?false:incrementalContacts==='frozen'?!s.wallFrictionStep?.liveNormalLoad:incrementalContacts,trace:s.linearTrace, fixed: s.fixed, reuseWorkingSet:modified?false:reuseWorkingSet,reuseStructure,reuseConstraintWork,reuseMatrixAssembly, ...(method === 0 && newtonActiveSetLimit !== Infinity ? {maxActiveSetAttempts:newtonActiveSetLimit} : {}), tolerance: Math.max(Math.min(linearToleranceCap,Math.min(forceTolerance, lengthTolerance) * .01), 1e-10 * Math.max(base.force, base.torque, base.constraint)) };
                const direction = isModified?modified.solve(base.rows,s.chain.gradient,linearOptions.tolerance):
                    yield* iterateSharedAxisLinear(s.mixed,s.chain,linearOptions);
                if(method===0&&direction.converged)modified?.seal(base.rows,s.fixed);
                if(isModified)modified.diagnostics.modifiedFallbacks++;
                timings.linearMs += direction.cpuMs ?? performance.now() - linearStart;
                factorizations += direction.factorizations;workingSetReuses += direction.workingSetReuses ?? 0;
                observeTrial?.({kind:'direction',state:s,iteration:iterations,method,base:observed(base),direction});
                yield {kind:'direction',iteration:iterations,method};
                if (!direction.converged) continue;
                solvedDirection = true;
                const penalty = Math.max(1, ...s.multipliers.map((v, i) => 2 * Math.abs(v + direction.multiplierIncrement[i])));
                const baseViolation=violation(base),basePotential = base.energy + penalty * baseViolation;
                const rootSearch=residualSearchActive&&base.constraint<=lengthTolerance;
                let predictedSlope = s.chain.gradient.reduce((sum, v, i) => sum + v * direction.increment[i], 0);
                for (const r of base.rows) predictedSlope -= (r.kind === 'wall' ? -1 : 1) * r.multiplier *
                    r.dofs.reduce((sum, dof, i) => sum + r.jacobian[i] * direction.increment[dof], 0);
                if(s.wallCompliance)for(let i=0;i<base.rows.length;i++) {
                    const r=base.rows[i];if(r.compliance)predictedSlope+=r.compliance*r.multiplier*direction.multiplierIncrement[i];
                }
                predictedSlope -= penalty * baseViolation;
                if (!rootSearch && !isGaussNewton && predictedSlope >= 0) {
                    observeTrial?.({kind:'non-descent',state:s,iteration:iterations,method,predictedSlope});
                    continue;
                }
                const acceptTrial=(candidate,potential,scale)=>merit(candidate)<=1||
                    (rootSearch?
                        merit(candidate)<=(1-1e-4*scale)*merit(base)&&candidate.constraint<=Math.max(lengthTolerance,base.constraint):
                        potential<basePotential||merit(candidate)<merit(base)&&candidate.constraint<=Math.max(lengthTolerance,base.constraint));
                let relativeStep = 0;
                for (let e = 0; e + 1 < s.positions.length; e++) {
                    const p = s.layout.positions[e], q = s.layout.positions[e+1];
                    const delta = Math.hypot(...[0,1,2].map(a => direction.increment[q+a]-direction.increment[p+a]));
                    relativeStep = Math.max(relativeStep, delta / vec(s.positions[e+1]).distanceTo(vec(s.positions[e])));
                }
                for (const indices of s.layout.spins.values()) for (const i of indices) if(i >= 0) relativeStep = Math.max(relativeStep, Math.abs(direction.increment[i]));
                const trustScale = Math.min(1, .25 / Math.max(relativeStep, 1e-30));
                let loadWork = 0;
                for (let i = 0; i < s.loads.length; i++) loadWork += s.loads[i] * direction.increment[i];
                for (let trial = 0; trial < (isGaussNewton ? 14 : 5); trial++) {
                    attemptedTrial = true;
                    restoreSharedAxisNative(s, snapshot);
                    let trialRowsAdded=false;
                    try {
                    applySharedAxisNativeIncrement(s, direction.increment, direction.multiplierIncrement, trustScale * 2 ** -trial);
                    s.definitions.forEach((d, i) => { if (d.kind === 'wall') s.multipliers[i] = Math.max(0, s.multipliers[i]); });
                    // Acceptance needs first derivatives. Promotion retains the local
                    // material preparation and augments it when hessianValid requests
                    // the next full matrix. Plain lazyTrialTangent (without reuse)
                    // remains an opt-in experiment; the eager path is the reference.
                    let candidate;
                    const originalRowCount=s.definitions.length;
                    try {candidate=assemble(undefined,!(lazyTrialTangent||promoteTrialAssembly));}
                    catch(error) {
                        if(!reuseDiscoveryTrial||!discover(error))throw error;
                        // Adding unloaded rows does not change this pose's force.
                        // Keep the trial only if every new gap is feasible and the
                        // ordinary nonlinear acceptance also passes. Otherwise
                        // rebuild the direction on the enlarged contact set.
                        trialRowsAdded=true;candidate=assembleDiscovered();
                        if(candidate.rows.slice(originalRowCount).some(r=>r.gap<0))throw new Error('shared-axis-retry-discovered-trial');
                    }
                    let candidateViolation=violation(candidate),candidatePotential = candidate.energy - trustScale * 2 ** -trial * loadWork + penalty * candidateViolation;
                    let accept = acceptTrial(candidate,candidatePotential,trustScale*2**-trial);
                    if(!accept&&trialRowsAdded)throw new Error('shared-axis-retry-discovered-trial');
                    if(!accept&&trial<2&&candidateViolation>0) {
                        const uncorrected=captureSharedAxisNative(s),uncorrectedMeasure=candidate,uncorrectedPotential=candidatePotential;
                        protectedTrial=uncorrectedMeasure;
                        try {
                        for(let correction=0;correction<2&&!accept;correction++) {
                            const start=performance.now(),projected=yield* correctTrialConstraints(s,candidate,reuseStructure,projectionMode,reuseConstraintWork,reuseMatrixAssembly);
                            const correctionMs=projected.cpuMs??performance.now()-start;timings.linearMs+=correctionMs;timings.projectionMs+=correctionMs;factorizations+=projected.factorizations;workingSetReuses+=projected.workingSetReuses??0;
                            if(!projected.converged)break;
                            candidate=assemble(undefined,!(lazyTrialTangent||promoteTrialAssembly));candidatePotential=candidate.energy-trustScale*2**-trial*loadWork+penalty*violation(candidate);
                            accept=acceptTrial(candidate,candidatePotential,trustScale*2**-trial);
                        }
                        if(!accept){restoreSharedAxisNative(s,uncorrected);candidate=uncorrectedMeasure;candidatePotential=uncorrectedPotential;}
                        } finally {protectedTrial=null;}
                    }
                    observeTrial?.({kind:'trial',state:s,iteration:iterations,method,trial,rootSearch,scale:trustScale*2**-trial,
                        base:observed(base),candidate:observed(candidate),basePotential,candidatePotential,predictedSlope,accept});
                    if (accept) {
                        if(isModified){modified.diagnostics.modifiedAccepted++;modified.diagnostics.modifiedFallbacks--;}
                        modified?.arm((isModified||method===0)&&trial===0&&trustScale===1&&relativeStep<.05&&merit(candidate)<.8*merit(base));
                        accepted = true; base = candidate;if(trialRowsAdded)retainedDiscoveryTrials++;
                        // Update the friction chart only after accepting the pose.
                        // Every direction and its entire line search retain one law.
                        // This avoids fully converging an obsolete stick/slide chart.
                        if(coupledFrictionNewton&&s.wallFrictionStep?.liveNormalLoad) {
                            const started=performance.now();
                            const change=refreshSharedAxisWallFriction(s,{forceTolerance:0});
                            frictionRefreshMs+=performance.now()-started;coupledFrictionRefreshes++;
                            observeTrial?.({kind:'friction-refresh',state:s,iteration:iterations,change});
                            if(change.forceChange>0){
                                modified?.invalidate();
                                let refreshed=null;
                                if(reuseFrictionAssembly&&!cullInactiveContacts) {
                                    const started=performance.now();
                                    try{refreshed=refreshSharedAxisFrictionMeasure(s,base);}
                                    finally{const elapsed=performance.now()-started;timings.assemblyMs+=elapsed;timings.residualAssemblyMs+=elapsed;}
                                    if(refreshed)residualAssemblies++;
                                }
                                base=refreshed??assembleDiscovered();
                            }
                        }
                        break;
                    }
                    backtracks++;
                    } catch(error) {
                        if(error.code!=='trial-outside-vessel')throw error;
                        lastOutsideContact=error.contact??null;
                        observeTrial?.({kind:'outside-vessel',iteration:iterations,method,trial,contact:lastOutsideContact});
                        if(trialRowsAdded)throw new Error('shared-axis-retry-discovered-trial');
                        backtracks++;
                    }
                }
            }
            if (!accepted) {
                status = !solvedDirection ? 'linear-solve' : attemptedTrial ? 'line-search' : 'non-descent';
                break;
            }
            } catch(error) {
                if(error.message!=='shared-axis-retry-discovered-trial'&&!discover(error))throw error;
                // New geometry invalidates this trial direction, not earlier
                // private progress. Restore the current iteration and recompute
                // its merit/direction using every discovered contact. The full
                // timestep still publishes only after the final certificate.
                restoreSharedAxisNative(s,snapshot);base=assembleDiscovered();
                discoveryIterations++;iterations--;
            }
        }
        candidate = base;
    } catch (e) { error = e.message; lastOutsideContact=e.contact??lastOutsideContact; status = 'unsupported-direction'; }
    finally {modified?.dispose();}
    const converged = candidate !== null && merit(candidate) <= 1;
    if (!converged) restoreSharedAxisNative(s, initial); else {
        status = 'converged'; s.acceptedSolves++;
        if(deferContactPublication)deferContactPublication(candidate.rows);
        else {
            materializeSharedAxisContacts(candidate.rows);
            s.acceptedWallGaps=new Map(candidate.rows.filter(r=>r.id).map(r=>[r.id,r.gap]));
        }
    }
    return { converged, status, error, ...(lastOutsideContact?{lastOutsideContact}:{}), coupledFrictionRefreshes, frictionRefreshMs, retainedDiscoveryTrials, iterations:iterations+discoveryIterations, factorizations, workingSetReuses, backtracks, geometryRestarts, ms: performance.now() - started,
        ...modified?.diagnostics,timings,fullAssemblies,residualAssemblies,promotedAssemblies:promotion?.hits??0, quality:converged?measureSharedAxisQuality(s,candidate.rows):null, residual: candidate ? { force: candidate.force, torque: candidate.torque, length: candidate.constraint } : null,
        ...(residualSearchActivation?{residualSearchActivation}:{}),...(detectedCycle?{detectedCycle}:{}),...(detectedStagnation?{detectedStagnation}:{}),dofs: s.layout.dofCount, matrixEntries: s.mixed.peakActiveEntries??0, linearScratch:getSharedAxisLinearScratchStats(), interToolRows: 0 };
}

export function relaxSharedAxisNative(s,options={}) {
    const iterator=iterateSharedAxisNative(s,options);
    let next;do{next=iterator.next();}while(!next.done);
    return next.value;
}
