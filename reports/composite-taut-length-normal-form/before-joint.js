import { createCompositeJointAssembly, createCompositeJointAssemblyWorkspace, invalidateCompositeJointAssemblyWorkspace } from './kirchhoffCompositeJointAssembly.js';
import { createCompositeToolLengthWorkspace, evaluateCompositeToolLengths } from './kirchhoffCompositeToolLengths.js';
import { createCompositeRelativeDirectionWorkspace, solveCompositeRelativeDirection } from './kirchhoffCompositeRelativeDirection.js';
import { createCompositeJointLumenRows, createCompositeJointLumenRowWorkspace } from './kirchhoffCompositeJointLumenRows.js';
import { createCompositeJointLumenFrictionRows, createCompositeJointLumenFrictionWorkspace } from './kirchhoffCompositeJointLumenFrictionRows.js';
import { proposeCompositeFrictionBacktrack } from './kirchhoffCompositeFrictionLineSearch.js';
import { createCompositeJointWallRows } from './kirchhoffCompositeJointWallRows.js';
import { createCompositeJointWallFrictionRows, createCompositeJointWallFrictionWorkspace } from './kirchhoffCompositeJointWallFrictionRows.js';
import { ownCompositeJointInertiaGeometry } from './kirchhoffCompositeJointGeometry.js';
import { createCompositeContinuousMaterialVelocity } from './kirchhoffCompositeContinuousGeometry.js';

const finite = (v, name) => { if (!Number.isFinite(v)) throw new RangeError(`${name} must be finite`); return v; };
const positive = (v, name) => { if (!(finite(v, name) > 0)) throw new RangeError(`${name} must be positive`); return v; };
const vec = (v, n, name) => { if (v?.length !== n || !v.every(Number.isFinite)) throw new RangeError(`${name} requires ${n} finite values`); return Array.from(v); };
const dot = (a, b) => a.reduce((s, v, i) => s + v * b[i], 0);
const cross = (a, b) => [a[1]*b[2]-a[2]*b[1], a[2]*b[0]-a[0]*b[2], a[0]*b[1]-a[1]*b[0]];
const unit = a => { const n = positive(Math.hypot(...a), 'direction length'); return a.map(v => v/n); };
const sameMetric = (a, b) => Math.abs(a-b) <= 64*Number.EPSILON*Math.max(Math.abs(a), Math.abs(b));
const key = (id, node) => JSON.stringify([id, node]);
const nodeIds = (layout, node) => [...new Set([...(layout.edgeToolIds[node-1] ?? []), ...(layout.edgeToolIds[node] ?? [])])];
const ownPoints = points => points.map(p => vec(p, 3, 'physical position'));
const jointWorkspaces=new WeakMap();

/** Bounded symbolic/scratch reuse for sequential fixed-chart calls. This owns
 * no accepted state or physical multiplier history. Current materials,
 * geometry, feed maps, old velocities and reaction seeds are prepared afresh
 * from EACH call; no numeric tangent or factor is reused as current. The LRU
 * retains only 1..4 local row patterns and their matrix/factor STORAGE.
 */
export function createCompositeJointTimeStepWorkspace({layout,coordinates,modes,inertiaGeometryByTool=null,elasticityGeometry='native-discrete-rod',lengthGeometry='native-chords',relativeToolId=layout?.spins?.size===1?layout.spins.keys().next().value:'wire',elementBackend='wasm-exact',rowCacheCapacity=2}) {
    if(!Number.isInteger(rowCacheCapacity)||rowCacheCapacity<1||rowCacheCapacity>4)throw new RangeError('rowCacheCapacity must be 1..4');
    const assembly=createCompositeJointAssemblyWorkspace({layout,coordinates,modes,inertiaGeometryByTool,elasticityGeometry,relativeToolId,elementBackend,physicalInertia:true});
    const scratch={assembly,busy:false,lengths:null,rows:new Map(),rowCacheCapacity,slots:null,fixed:null,cg:null,rg:null,commonLoad:null,relativeLoad:null,
        stats:{calls:0,lengthBuilds:0,directionBuilds:0,directionHits:0}};
    if(!['native-chords','continuous-arclength'].includes(lengthGeometry)||(lengthGeometry==='continuous-arclength'&&inertiaGeometryByTool===null))throw new RangeError('Continuous lengths require the same continuous material geometry');
    const handle=Object.freeze({elementBackend,lengthGeometry,nodeCount:layout.nodeCount,
        get diagnostics(){return {...scratch.stats,assembly:assembly.diagnostics,rowCacheCapacity,retainedRowPatterns:scratch.rows.size,
            lumen:scratch.lumen?.diagnostics??null,lumenFriction:scratch.friction?.diagnostics??null};}});
    jointWorkspaces.set(handle,scratch);return handle;
}
function reusableDirection(scratch,layout,cluster,rows) {
    if(!scratch)return createCompositeRelativeDirectionWorkspace(layout,cluster,rows);
    const signature=JSON.stringify(rows.map(row=>[row.toolId,row.index??null,row.anchorNode,Array.from(row.commonDofs),Array.from(row.relativeDofs),row.unit,Array.from(row.multiplierDofs??[]),row.constraintSupport??null]));
    let value=scratch.rows.get(signature);
    if(value){scratch.stats.directionHits++;scratch.rows.delete(signature);}
    else {value=createCompositeRelativeDirectionWorkspace(layout,cluster,rows);scratch.stats.directionBuilds++;}
    scratch.rows.set(signature,value);
    if(scratch.rows.size>scratch.rowCacheCapacity)scratch.rows.delete(scratch.rows.keys().next().value);
    return value;
}
function transport(d, from, to) {
    const denominator = 1 + dot(from, to);
    if (!(denominator > 1e-10)) throw new RangeError('Time frame transport cannot reverse its tangent');
    const axis = cross(from, to), first = cross(axis, d), second = cross(axis, first);
    return d.map((v, k) => v + first[k] + second[k]/denominator);
}
function geometry(state) {
    const p = new Map([...state.layout.spins.keys()].map(id => [id, ownPoints(state.positions)]));
    const wire = p.get(state.relativeToolId);
    for (const mode of state.modes) for (let a=0; a<3; a++) for (let k=0; k<3; k++)
        wire[mode.node][k] += mode.basis[a][k]*state.relative[mode.relativeDofs[a]];
    return p;
}

/** Owned initial state of the full change of variables on this FIXED chart.
 * Every overlap node, including both material tips, has all three relative
 * coordinates. A single material uses q only, with explicit modes:[] and
 * zero relative coordinates. Each material owns frames, winding, spins and rest lengths.
 * Only immutable material providers may be borrowed. Reference arclength is
 * the sole supported material measure; no remesh/history interpolation occurs.
 */
export function createCompositeJointTimeStepState({ layout, coordinates, positions, relative, modes, angles,inertiaGeometryByTool=null,elasticityGeometry='native-discrete-rod',lengthGeometry='native-chords',
    tools, restLengths, materialCoordinate, relativeToolId=layout?.spins?.size===1?layout.spins.keys().next().value:'wire', lengthMultipliers=null, lumenContactState=undefined, lumenFrictionState=undefined, wallContactState=undefined, wallFrictionState=undefined, time=0, step=0 }) {
    if (materialCoordinate !== 'reference-arclength') throw new RangeError('materialCoordinate must be reference-arclength');
    if(!['native-chords','continuous-arclength'].includes(lengthGeometry)||(lengthGeometry==='continuous-arclength'&&inertiaGeometryByTool===null))throw new RangeError('Continuous lengths require the same continuous material geometry');
    const continuous=ownCompositeJointInertiaGeometry({layout,coordinates,inertiaGeometryByTool,elasticityGeometry}),ownLayout=continuous.layout,n=ownLayout.nodeCount;
    const materialCount=ownLayout.spins.size;
    if (n<3 || ![1,2].includes(materialCount) || !ownLayout.spins.has(relativeToolId)) throw new RangeError('A joint step needs one or two actual materials and at least three common nodes');
    const x = Float64Array.from(vec(coordinates, n, 'chart coordinates'));
    x.forEach((v, i) => { if (i && v<=x[i-1]) throw new RangeError('Chart coordinates must increase'); });
    if (positions?.length!==n || !(angles instanceof Map) || !(restLengths instanceof Map) || !Array.isArray(tools)) throw new RangeError('Explicit independent state maps are required');
    const overlap = Array.from({length:n}, (_,i) => i).filter(i => nodeIds(ownLayout,i).length===2);
    if (!Array.isArray(modes) || modes.length!==overlap.length || (materialCount===2&&!modes.length)) throw new RangeError('Full three-coordinate modes are required at EVERY overlap node; one material requires modes:[]');
    const ownedModes = modes.map((m,i) => {
        if (m.node!==overlap[i] || m.basis?.length!==3) throw new RangeError('Ordered full modes must cover every overlap node including endpoints');
        const basis = m.basis.map(b => vec(b,3,'fixed complete basis'));
        for (let a=0;a<3;a++) for(let b=0;b<=a;b++) if(Math.abs(dot(basis[a],basis[b])-(a===b?1:0))>1e-10) throw new RangeError('Full bases must be orthonormal');
        return {node:m.node,basis,dimension:3,relativeDofs:[3*i,3*i+1,3*i+2]};
    });
    const byId = new Map(tools.map(t => [t.id,t]));
    if (byId.size!==materialCount || tools.length!==materialCount || [...ownLayout.spins.keys()].some(id => !byId.has(id))) throw new RangeError('Exactly one own material record per tool is required');
    const ownAngles = new Map(), ownRest = new Map();
    const ownTools = [...ownLayout.spins].map(([id,spins]) => {
        const t = byId.get(id), a = angles.get(id), r = restLengths.get(id);
        if (a?.length!==n-1 || r?.length!==n-1 || t.reference?.length!==n-1 || t.referenceTwists?.length!==n-2) throw new RangeError('Each material needs own angles, rest metrics, frames and winding');
        // A callable slope does not establish an affine arclength chart. A
        // scalar is an explicit contract, never inferred from equal samples.
        positive(t.dsDx, 'constant material dsDx');
        spins.forEach((d,e) => { if(d>=0){finite(a[e],'active spin');positive(r[e],'active rest length');} });
        for (const h of ownLayout.hinges) if(h.tools.includes(id)) finite(t.referenceTwists[h.vertex-1],'active winding');
        ownAngles.set(id,Float64Array.from(a)); ownRest.set(id,Float64Array.from(r));
        return {...t, reference:t.reference.map((f,e)=>spins[e]<0?null:{tangent:vec(f?.tangent,3,'own tangent'),director:vec(f?.director,3,'own director')}),
            referenceTwists:Float64Array.from(t.referenceTwists),
            ...(t.materialBreaks!==undefined?{materialBreaks:Array.from(t.materialBreaks)}:{}),...(t.elasticQuadrature!==undefined?{elasticQuadrature:{...t.elasticQuadrature}}:{}),
            material:t.material?{...t.material,stiffness:Float64Array.from(t.material.stiffness),intrinsic:Float64Array.from(t.material.intrinsic)}:undefined};
    });
    if (!Number.isInteger(step)||step<0) throw new RangeError('step must be nonnegative integer');
    const count = ownLayout.edgeToolIds.reduce((s,ids)=>s+ids.length,0);
    const state = {layout:ownLayout,coordinates:x,positions:ownPoints(positions),relative:Float64Array.from(vec(relative,3*ownedModes.length,'relative coordinates')),
        modes:ownedModes,angles:ownAngles,tools:ownTools,restLengths:ownRest,materialCoordinate,relativeToolId,inertiaGeometryByTool:continuous.inertiaGeometryByTool,elasticityGeometry,lengthGeometry,
        lengthMultipliers:lengthMultipliers===null?new Float64Array(count):Float64Array.from(vec(lengthMultipliers,count,'length forces')),
        boundaryMultipliers:new Map(),materialVelocities:null,time:finite(time,'time'),step};
    state.toolPositions=geometry(state);
    if(lumenContactState!==undefined)state.lumenContactState=structuredClone(lumenContactState);
    if(lumenFrictionState!==undefined)state.lumenFrictionState=structuredClone(lumenFrictionState);
    if(wallContactState!==undefined)state.wallContactState=structuredClone(wallContactState);
    if(wallFrictionState!==undefined)state.wallFrictionState=structuredClone(wallFrictionState);
    for(const tool of state.tools) {
        const p=state.toolPositions.get(tool.id);
        tool.reference.forEach((frame,e)=>{
            if(frame===null)return;
            const tangent=unit(p[e+1].map((v,k)=>v-p[e][k]));
            if(Math.hypot(...frame.tangent.map((v,k)=>v-tangent[k]))>1e-10 || Math.abs(dot(frame.director,frame.director)-1)>1e-10 || Math.abs(dot(frame.director,tangent))>1e-10)
                throw new RangeError('Incoming reference frames must belong to their own current physical geometry');
        });
    }
    return state;
}

function prepareInertia(state, input, dt) {
    const {layout,coordinates:x} = state, n = layout.nodeCount;
    if (!(input?.previousPositions instanceof Map) || input.inertiaEdges?.length!==n-1) throw new RangeError('Independent previous positions and all prepared inertia edges are required');
    if (input.dt!==undefined && input.dt!==dt) throw new RangeError('Prepared inertia dt differs from the requested dt');
    const current=geometry(state), previousPositions=new Map();
    for(const [id] of layout.spins) {
        const p=input.previousPositions.get(id);if(p?.length!==n)throw new RangeError('Each tool needs its own previous geometry');
        const owned=ownPoints(p);
        for(let i=0;i<n;i++)if(nodeIds(layout,i).includes(id))for(let k=0;k<3;k++)
            if(owned[i][k]!==current.get(id)[i][k] || state.toolPositions?.get(id)?.[i]?.[k]!==current.get(id)[i][k])
                throw new RangeError('Previous physical geometry must equal the incoming state in the SAME chart');
        previousPositions.set(id,owned);
    }
    const byId=new Map(state.tools.map(t=>[t.id,t])), last=new Map();
    const inertiaEdges=input.inertiaEdges.map((edge,e)=>{
        const records=edge?.tools, ids=layout.edgeToolIds[e], dx=x[e+1]-x[e];
        if(!Array.isArray(records)||records.length!==ids.length||new Set(records.map(t=>t.id)).size!==ids.length||records.some(t=>!ids.includes(t.id)))throw new RangeError('Exactly one prepared inertia record per active material edge is required');
        return {tools:records.map(t=>{
            const map=t.materialMap, dsDx=positive(map?.dsDx,'inertia material dsDx'), slope=byId.get(t.id).dsDx;
            if(!sameMetric(dsDx,slope)||!sameMetric(state.restLengths.get(t.id)[e],dx*dsDx))
                throw new RangeError('Elastic dsDx, inertia map and rest length must share one reference-arclength measure');
            const sStart=finite(map.sStart,'material start label');
            if(last.has(t.id)&&!sameMetric(sStart,last.get(t.id)))throw new RangeError('Material labels must be continuous in the fixed affine chart');
            last.set(t.id,sStart+dx*dsDx);
            const dsDt=typeof map.dsDt==='number'?finite(map.dsDt,'material label rate'):vec(map.dsDt,2,'material label rates');
            let history;
            if(t.oldVelocityPieces!==undefined) {
                if(t.oldMaterialVelocities!==undefined||!Array.isArray(t.oldVelocityPieces)||!t.oldVelocityPieces.length)
                    throw new RangeError('Piecewise old velocities require one unambiguous prepared history');
                let covered=0;
                history={oldVelocityPieces:t.oldVelocityPieces.map(piece=>{
                    const fractions=vec(piece.fractions,2,'history piece fractions'),[a,b]=fractions;
                    if(a!==covered||b<=a||b>1)
                        throw new RangeError('History pieces must exactly cover the current material edge');
                    covered=b;
                    if(piece.interpretation==='quintic-bernstein-material-velocity') {
                        if(state.inertiaGeometryByTool===null)throw new RangeError('Polynomial material history requires continuous inertia geometry');
                        if(piece.oldMaterialVelocities!==undefined||piece.bernsteinVelocities?.length!==6)throw new RangeError('Polynomial history requires exactly six velocity controls');
                        return {fractions,interpretation:piece.interpretation,bernsteinVelocities:piece.bernsteinVelocities.map(v=>vec(v,3,'old polynomial material velocity'))};
                    }
                    if(piece.bernsteinVelocities!==undefined||piece.oldMaterialVelocities?.length!==2||![undefined,'physical-material-velocity'].includes(piece.interpretation))
                        throw new RangeError('History pieces require an explicit affine or polynomial material velocity');
                    return {fractions,oldMaterialVelocities:piece.oldMaterialVelocities.map(v=>vec(v,3,'old piece velocity'))};
                })};
                if(covered!==1)throw new RangeError('History pieces must exactly cover the current material edge');
            } else {
                if(t.oldMaterialVelocities?.length!==2)throw new RangeError('Old velocities must be explicit at CURRENT material labels');
                const oldMaterialVelocities=t.oldMaterialVelocities.map(v=>vec(v,3,'old material velocity'));
                history=state.inertiaGeometryByTool===null?{oldMaterialVelocities}:{oldVelocityPieces:[{fractions:[0,1],oldMaterialVelocities}]};
            }
            return {id:t.id,massPerMaterialLength:positive(t.massPerMaterialLength,'physical density per reference arclength'),
                materialMap:{sStart,dsDx,dsDt},...history};
        })};
    });
    return {dt,previousPositions,inertiaEdges};
}

function commitFrames(candidate, physical) {
    for(const tool of candidate.tools) {
        const p=physical.get(tool.id), spin=candidate.layout.spins.get(tool.id), old=tool.reference;
        tool.reference=old.map((frame,e)=>{
            if(spin[e]<0)return null;
            const tangent=unit(p[e+1].map((v,k)=>v-p[e][k])), carried=transport(frame.director,frame.tangent,tangent), projection=dot(carried,tangent);
            return {tangent,director:unit(carried.map((v,k)=>v-projection*tangent[k]))};
        });
        const winding=new Float64Array(candidate.layout.nodeCount-2).fill(NaN);
        for(const h of candidate.layout.hinges)if(h.tools.includes(tool.id)) {
            const i=h.vertex, left=tool.reference[i-1],right=tool.reference[i],d=transport(left.director,left.tangent,right.tangent);
            const raw=Math.atan2(dot(right.tangent,cross(d,right.director)),dot(d,right.director)), anchor=tool.referenceTwists[i-1];
            winding[i-1]=raw+2*Math.PI*Math.round((anchor-raw)/(2*Math.PI));
        }
        tool.referenceTwists=winding;
    }
}
function acceptedVelocities(candidate,inertia,dt) {
    return inertia.inertiaEdges.map((entry,e)=>({edge:e,tools:entry.tools.map(t=>{
        const p=candidate.toolPositions.get(t.id),old=inertia.previousPositions.get(t.id),dx=candidate.coordinates[e+1]-candidate.coordinates[e],m=t.materialMap;
        const continuous=candidate.inertiaGeometryByTool?.get(t.id).edges[e];
        if(continuous)return createCompositeContinuousMaterialVelocity(continuous,{id:t.id,positions:continuous.nodeIndices.map(node=>p[node]),
            previousPositions:continuous.nodeIndices.map(node=>old[node]),dt,materialMap:m});
        const rates=typeof m.dsDt==='number'?[m.dsDt,m.dsDt]:m.dsDt, qx=p[e+1].map((v,k)=>(v-p[e][k])/dx);
        const velocities=[0,1].map(end=>p[e+end].map((v,k)=>(v-old[e+end][k])/dt-rates[end]/m.dsDx*qx[k]));
        velocities.forEach(v=>vec(v,3,'accepted material velocity'));
        return {id:t.id,sStart:m.sStart,sEnd:m.sStart+m.dsDx*dx,velocities,interpretation:'physical-material-velocity',angularVelocity:null,materialSpin:null,frameSpin:null};
    })}));
}

/** One nonlinear prepared physical dt, ONE local band solve for common q,
 * all 3D relative coordinates, independent spins and original length/BC rows.
 * Contacts explicitly select 'none', fixed-pair 'lumen-normal', or
 * 'lumen-coulomb' with its own finite material surface history. Every declared
 * lumen sample retains its original NCP gate; Coulomb rows retain their
 * original cone, maximum-dissipation and work gates in the SAME solve.
 * Angular inertia is absent and torsion MUST be quasi-static.
 * Wall normals optionally enter the same solve on their physical owners,
 * including original two-branch SDF contacts. 'wall-coulomb' adds own finite
 * surface friction for ordinary stationary-wall capsule records. Its current
 * wall point/frame is the same stationary material witness at both times.
 * This fixed topology reference
 * includes declared normal side/fillet/rim features and piecewise old
 * material velocities. Finite friction currently requires same-edge tool
 * histories and fixed supported surface features. Remeshing, changes of
 * loaded contact ownership, continuous
 * collision and FPS are outside this fixed-chart step's certificate.
 *
 * Position boundaries prescribe physical material nodes, never rho alone.
 * Common-axis boundaries use exact Dirichlet q. Wire overlap targets use
 * signed dual rows; their lambda is physical support force ON THE WIRE.
 * A length whose complete physical position support is prescribed has a
 * redundant equation: its dual is omitted with the explicit gauge lambda=0, but its
 * ORIGINAL length is still checked. Other singular Jacobians reject honestly.
 * For continuous arclength this support also includes neighboring shape nodes;
 * prescribing only the edge endpoints does not establish a redundant row.
 * Failed attempts return the same untouched state; retry inputs stay external.
 */
export function advanceCompositeJointTimeStep(state,options={}) {
    const handle=options.workspace;
    if(handle===undefined||handle===null)return advancePreparedJointStep(state,options,null);
    const scratch=jointWorkspaces.get(handle);
    if(!scratch)throw new RangeError('Unknown joint timestep workspace');
    if(scratch.busy)throw new RangeError('Joint timestep workspace is busy');
    if((options.elementBackend??'wasm-exact')!==handle.elementBackend)throw new RangeError('Joint workspace backend changed');
    if((state.lengthGeometry??'native-chords')!==handle.lengthGeometry)throw new RangeError('Joint workspace length geometry changed');
    scratch.busy=true;scratch.stats.calls++;
    try {return advancePreparedJointStep(state,options,scratch);}
    finally {try {invalidateCompositeJointAssemblyWorkspace(scratch.assembly);} finally {scratch.busy=false;}}
}
function advancePreparedJointStep(state,{dt,torsionMode,contacts,wall='none',inertia,boundaries={positions:[],spins:[]},loads={forces:[],torques:[]},
    tolerances={},budget={},elementBackend='wasm-exact',assemblyPolicy='lazy',globalization='adaptive'}={},scratch) {
    const started=performance.now(); positive(dt,'dt');
    const contactMode=contacts==='none'?'none':['lumen-normal','lumen-coulomb'].includes(contacts?.mode)?contacts.mode:null;
    const frictionEnabled=contactMode==='lumen-coulomb';
    if(torsionMode!=='quasi-static'||contactMode===null)throw new RangeError('Explicit quasi-static torsion and contacts:none, lumen-normal or lumen-coulomb are required');
    if(contactMode==='none'&&state.lumenContactState?.normalForces?.some(Fn=>Fn!==0))throw new RangeError('contacts:none cannot discard a nonzero accepted lumen reaction');
    if(!frictionEnabled&&state.lumenFrictionState?.tractions?.some(Ft=>Ft!==0))throw new RangeError('Disabling friction cannot discard a nonzero accepted tangential reaction');
    const wallMode=wall==='none'?'none':['wall-normal','wall-coulomb'].includes(wall?.mode)?wall.mode:null;
    const wallFrictionEnabled=wallMode==='wall-coulomb';
    if(state.inertiaGeometryByTool!==null&&state.inertiaGeometryByTool!==undefined&&(contactMode!=='none'||wallMode!=='none'))
        throw new RangeError('Continuous inertia needs matching curved contact geometry before contact integration');
    if(wallMode===null)throw new RangeError('Wall contacts require none, wall-normal or wall-coulomb');
    if(!wallFrictionEnabled&&state.wallFrictionState?.tractions?.some(Ft=>Ft!==0))throw new RangeError('Disabling wall friction cannot discard a nonzero accepted tangential reaction');
    if(wallMode==='none'&&state.wallContactState?.normalForces?.some(Fn=>Fn!==0))throw new RangeError('wall:none cannot discard a nonzero accepted wall reaction');
    if(!['wasm-exact','wasm','javascript'].includes(elementBackend)||!['lazy','full'].includes(assemblyPolicy))throw new RangeError('Unknown joint backend or assembly policy');
    if(!['adaptive','newton'].includes(globalization))throw new RangeError('Unknown joint globalization policy');
    const tol={force:positive(tolerances.force??1e-7,'force tolerance'),torque:positive(tolerances.torque??1e-8,'torque tolerance'),
        length:positive(tolerances.length??1e-8,'length tolerance'),boundary:positive(tolerances.boundary??1e-9,'boundary tolerance'),
        linearForce:positive(tolerances.linearForce??5e-10,'linear force tolerance'),linearTorque:positive(tolerances.linearTorque??5e-10,'linear torque tolerance'),
        linearConstraint:positive(tolerances.linearConstraint??5e-11,'linear constraint tolerance')};
    if(contactMode!=='none')Object.assign(tol,{lumenGap:positive(tolerances.lumenGap??1e-8,'lumen gap tolerance'),
        lumenNcp:positive(tolerances.lumenNcp??1e-8,'lumen NCP length tolerance'),lumenWork:positive(tolerances.lumenWork??1e-9,'lumen complementarity work tolerance')});
    if(frictionEnabled||wallFrictionEnabled)Object.assign(tol,{frictionSlip:positive(tolerances.frictionSlip??1e-8,'friction slip tolerance'),
        frictionCone:positive(tolerances.frictionCone??1e-9,'friction force-cone tolerance'),frictionWork:positive(tolerances.frictionWork??1e-9,'friction work tolerance'),
        frictionEquation:positive(tolerances.frictionEquation??1e-8,'friction equation length tolerance')});
    if(wallMode!=='none')Object.assign(tol,{wallGap:positive(tolerances.wallGap??1e-8,'wall gap tolerance'),
        wallNcp:positive(tolerances.wallNcp??1e-8,'wall NCP length tolerance'),wallWork:positive(tolerances.wallWork??1e-9,'wall complementarity work tolerance')});
    const limits={directions:budget.directions??64,evaluations:budget.evaluations??256,lineSearchTrials:budget.lineSearchTrials??24,linearSolves:budget.linearSolves??192};
    if(contactMode!=='none'||wallMode!=='none')limits.contactQueries=budget.contactQueries??10000;
    for(const [name,value] of Object.entries(limits))if(!Number.isInteger(value)||value<0)throw new RangeError(`${name} budget must be a nonnegative integer`);
    const candidate=createCompositeJointTimeStepState(state), prepared=prepareInertia(state,inertia,dt),{layout}=candidate,n=layout.nodeCount;
    if(!frictionEnabled)delete candidate.lumenFrictionState;
    if(!wallFrictionEnabled)delete candidate.wallFrictionState;
    const assembly=createCompositeJointAssembly({...candidate,inertia:prepared,elementBackend},{workspace:scratch?.assembly??null}), cluster=assembly.cluster;
    if(contactMode!=='none'&&scratch&&!scratch.lumen)scratch.lumen=createCompositeJointLumenRowWorkspace();
    const lumen=contactMode!=='none'?createCompositeJointLumenRows({layout,coordinates:candidate.coordinates,modes:cluster.modes,relativeToolId:state.relativeToolId,
        contacts:frictionEnabled?{...contacts,mode:'lumen-normal',friction:'none'}:contacts,history:candidate.lumenContactState??null,tolerances:tol,
        preserveSampleReactions:frictionEnabled,workspace:scratch?.lumen??null}):null;
    const vessel=wallMode!=='none'?createCompositeJointWallRows({layout,coordinates:candidate.coordinates,modes:cluster.modes,relativeToolId:state.relativeToolId,
        wall:wallFrictionEnabled?{...wall,mode:'wall-normal',friction:'none'}:wall,history:candidate.wallContactState??null,tolerances:tol}):null;
    const lengths=scratch?.lengths??createCompositeToolLengthWorkspace({layout,modes:cluster.modes,relativeToolId:state.relativeToolId,
        geometryByTool:state.lengthGeometry==='continuous-arclength'?candidate.inertiaGeometryByTool:null});
    if(scratch&&!scratch.lengths){scratch.lengths=lengths;scratch.stats.lengthBuilds++;}
    const modes=scratch?.modes??new Map(cluster.modes.map(m=>[m.node,m])), fixed=scratch?.fixed??new Uint8Array(layout.dofCount),positionBC=new Map(),spinBC=new Set(),bcRows=[];
    fixed.fill(0);
    for(const b of boundaries.positions??[]) {
        if(!Number.isInteger(b.node)||!nodeIds(layout,b.node).includes(b.toolId)||positionBC.has(key(b.toolId,b.node)))throw new RangeError('Position boundaries need distinct active physical material nodes');
        const value=vec(b.value,3,'physical position target'),record={toolId:b.toolId,node:b.node,value};positionBC.set(key(b.toolId,b.node),record);
        const mode=b.toolId===state.relativeToolId?modes.get(b.node):null;
        if(!mode) {candidate.positions[b.node]=value.slice();fixed.fill(1,layout.positions[b.node],layout.positions[b.node]+3);record.direct=true;}
        else for(let axis=0;axis<3;axis++) {
            const J=Float64Array.from([1,...mode.basis.map(v=>v[axis])]);
            const row={anchorNode:b.node,commonDofs:Int32Array.of(layout.positions[b.node]+axis),relativeDofs:Int32Array.from(mode.relativeDofs),unit:'mm',
                jacobian:J,forceColumn:Float64Array.from(J,v=>-v),residual:0,tolerance:tol.linearConstraint,multiplierDerivative:0,
                toolId:b.toolId,node:b.node,axis,target:value[axis],lambda:state.boundaryMultipliers?.get(key(b.toolId,b.node))?.[axis]??0};
            bcRows.push(row);
        }
    }
    for(const b of boundaries.spins??[]) {
        const d=layout.spins.get(b.toolId)?.[b.edge];
        if(!Number.isInteger(b.edge)||!(d>=0)||spinBC.has(d))throw new RangeError('Spin boundaries must identify distinct active material edges');
        candidate.angles.get(b.toolId)[b.edge]=finite(b.value,'spin target');fixed[d]=1;spinBC.add(d);
    }
    for(const spins of layout.spins.values())if(!spins.some(d=>d>=0&&spinBC.has(d)))throw new RangeError('Quasi-static torsion needs an explicit spin boundary for each tool');
    const suppressed=[];
    const activeLengths=lengths.rows.filter(row=>{
        const prescribed=row.nodeIndices.map(node=>positionBC.get(key(row.toolId,node)));
        if(prescribed.some(value=>!value))return true;
        const length=row.lengthGeometry==='continuous-arclength'?row.compiled.evaluate(prescribed.map(v=>v.value),{order:'gradient',lengthTolerance:Math.min(1e-12,.1*tol.length)}).length
            :Math.hypot(...prescribed[1].value.map((v,k)=>v-prescribed[0].value[k])),gap=length-candidate.restLengths.get(row.toolId)[row.edge];
        if(Math.abs(gap)>tol.length)throw new RangeError('Prescribed physical endpoints conflict with their original material rest length');
        candidate.lengthMultipliers[row.index]=0;suppressed.push(row.index);return false;
    });
    const normalRowOffset=activeLengths.length+bcRows.length,frictionRowOffset=normalRowOffset+(lumen?.rows.length??0);
    if(frictionEnabled&&scratch&&!scratch.friction)scratch.friction=createCompositeJointLumenFrictionWorkspace();
    const friction=frictionEnabled?createCompositeJointLumenFrictionRows({state,candidate,prepared,normal:lumen,contacts,dt,tolerances:tol,normalRowOffset,frictionRowOffset,workspace:scratch?.friction}):null;
    const baseRows=[...activeLengths,...bcRows,...(lumen?.rows??[]),...(friction?.rows??[])];
    const wallFrictionRowOffset=baseRows.length+(vessel?.rows.length??0);
    if(wallFrictionEnabled&&scratch&&!scratch.wallFriction)scratch.wallFriction=createCompositeJointWallFrictionWorkspace();
    const wallFriction=wallFrictionEnabled?createCompositeJointWallFrictionRows({state,candidate,prepared,normal:vessel,wall,dt,tolerances:tol,
        normalRowOffset:baseRows.length,frictionRowOffset:wallFrictionRowOffset,workspace:scratch?.wallFriction}):null;
    // Only an explicit, full physical node boundary can make a nodal gap a
    // known constant. A vanishing current gap Jacobian is not that proof.
    const prescribedWallSites=(vessel?.surfaceRecords??[]).flatMap(site=>{
        const boundary=Number.isInteger(site.node)&&positionBC.get(key(site.owner,site.node));
        return boundary?[{site,boundary}]:[];
    });
    let wallFrictionPrepared=false;
    let rows=[...baseRows,...(vessel?.rows??[]),...(wallFriction?.rows??[])],directionWorkspace=reusableDirection(scratch,assembly.layout,cluster,rows);
    const commonLoad=scratch?.commonLoad??new Float64Array(layout.dofCount),relativeLoad=scratch?.relativeLoad??new Float64Array(candidate.relative.length),applied=new Map([...layout.spins.keys()].map(id=>[id,[0,0,0]]));
    commonLoad.fill(0);relativeLoad.fill(0);
    for(const f of loads.forces??[]) {
        if(!Number.isInteger(f.node)||!nodeIds(layout,f.node).includes(f.toolId))throw new RangeError('A force needs an active physical node');
        const v=vec(f.value,3,'physical force');v.forEach((value,k)=>{commonLoad[layout.positions[f.node]+k]+=value;applied.get(f.toolId)[k]+=value;});
        const mode=f.toolId===state.relativeToolId?modes.get(f.node):null;
        if(mode)mode.basis.forEach((b,a)=>relativeLoad[mode.relativeDofs[a]]+=dot(b,v));
    }
    for(const t of loads.torques??[]) {
        const d=layout.spins.get(t.toolId)?.[t.edge];if(!Number.isInteger(t.edge)||!(d>=0))throw new RangeError('A torque needs an active material spin');
        commonLoad[d]+=finite(t.value,'physical torque');
    }
    const slots=scratch?.slots??Array.from({length:layout.dofCount},()=>({array:null,index:0}));
    for(let node=0;node<n;node++)for(let k=0;k<3;k++){const s=slots[layout.positions[node]+k];s.array=candidate.positions[node];s.index=k;}
    for(const [id,spins] of layout.spins)spins.forEach((d,e)=>{if(d>=0){slots[d].array=candidate.angles.get(id);slots[d].index=e;}});
    const cg=scratch?.cg??new Float64Array(layout.dofCount),rg=scratch?.rg??new Float64Array(candidate.relative.length);
    const physicalResidual=scratch?.physicalResidual??new Map([...layout.spins.keys()].map(id=>[id,Array.from({length:n},()=>[0,0,0])]));
    const nodeTools=scratch?.nodeTools??Array.from({length:n},(_,node)=>nodeIds(layout,node));
    const backups=scratch?.backups??{common:new Float64Array(layout.dofCount),relative:new Float64Array(candidate.relative.length),
        lambda:new Float64Array(candidate.lengthMultipliers.length),boundary:new Float64Array(3*n)};
    if(lumen&&backups.normal?.length!==lumen.normalForces.length)backups.normal=new Float64Array(lumen.normalForces.length);
    if(friction&&backups.tractions?.length!==friction.tractions.length)backups.tractions=new Float64Array(friction.tractions.length);
    if(vessel&&backups.wallNormal?.length!==vessel.normalForces.length)backups.wallNormal=new Float64Array(vessel.normalForces.length);
    if(wallFriction&&backups.wallTractions?.length!==wallFriction.tractions.length)backups.wallTractions=new Float64Array(wallFriction.tractions.length);
    if(prescribedWallSites.length) {
        if(backups.beforeWallCommon?.length!==cg.length)backups.beforeWallCommon=new Float64Array(cg.length);
        if(backups.beforeWallRelative?.length!==rg.length)backups.beforeWallRelative=new Float64Array(rg.length);
    }
    if(scratch)Object.assign(scratch,{fixed,commonLoad,relativeLoad,slots,cg,rg,modes,physicalResidual,nodeTools,backups});
    const diagnostics={scope:'fixed-topology-full-relative-nonlinear-dt',contacts:contactMode,torsionMode,materialCoordinate:state.materialCoordinate,
        inertiaGeometry:state.inertiaGeometryByTool===null?'affine':'continuous-quintic',
        elasticityGeometry:state.elasticityGeometry,lengthGeometry:state.lengthGeometry,
        elementBackend,assemblyPolicy,preparationMs:performance.now()-started,iterationMs:0,commitMs:0,totalMs:0,
        directions:0,evaluations:0,fullAssemblies:0,gradientAssemblies:0,factorizations:0,linearSolves:0,lineSearchTrials:0,
        globalization,numericalDirections:0,regularizationActivations:[],tinyNewtonStepsAvoided:0,
        directionMs:0,directionSystems:[],frictionConeTrials:0,frictionConeAccepted:0,
        unknowns:directionWorkspace.count,bandwidth:directionWorkspace.packedLayout.kl,commonDofs:layout.dofCount,relativeDofs:candidate.relative.length,
        originalLengthRows:lengths.rows.length,retainedLengthRows:activeLengths.length,suppressedPrescribedLengthRows:suppressed,acceptedAlphas:[],certificate:null,
        workspaceReused:!!scratch};
    if(lumen||vessel)Object.assign(diagnostics,{contactQueries:0,minimumPrivateNormalForce:Infinity});
    if(lumen)Object.assign(diagnostics,{lumenSamples:lumen.samples.length,lumenRows:lumen.rows.length,lumenGauge:lumen.gauge});
    if(friction)Object.assign(diagnostics,{frictionRows:friction.rows.length,frictionPreparationQueries:0});
    if(vessel)Object.assign(diagnostics,{wall:wallMode,wallQueries:0,wallRowRebuilds:0,wallChartDiscoveries:0,
        wallKnownOpenBacksubstitutions:0,wallKnownOpenRowRefreshes:0});
    if(wallFriction)diagnostics.wallFrictionRows=wallFriction.rows.length;
    const iterationsStart=performance.now(),gradientOrder=elementBackend==='wasm-exact'&&assemblyPolicy==='lazy'?'gradient':'full';
    let output=null;
    function consumeContactQuery() {
        if(diagnostics.contactQueries>=limits.contactQueries){const error=new RangeError('contact-query-budget');error.code='contact-query-budget';throw error;}
        diagnostics.contactQueries++;
    }
    function refreshDirectionRows() {
        if(!vessel)return;
        const next=[...baseRows,...vessel.rows,...(wallFriction?.rows??[])];
        if(next.length===rows.length&&next.every((row,i)=>row===rows[i]))return;
        rows=next;directionWorkspace=reusableDirection(scratch,assembly.layout,cluster,rows);
        diagnostics.wallRowRebuilds++;
        diagnostics.unknowns=directionWorkspace.count;diagnostics.bandwidth=directionWorkspace.packedLayout.kl;
    }
    function evaluate(order=gradientOrder) {
        if(diagnostics.evaluations>=limits.evaluations)throw new RangeError('evaluation-budget');
        diagnostics.evaluations++;diagnostics[order==='full'?'fullAssemblies':'gradientAssemblies']++;
        output=assembly.evaluate(candidate,{order});
        evaluateCompositeToolLengths({toolPositions:output.toolPositions,restLengths:candidate.restLengths,multipliers:candidate.lengthMultipliers,tolerance:tol.linearConstraint,order},lengths);
        cg.forEach((_,i)=>cg[i]=output.chain.gradient[i]+lengths.commonGradient[i]-commonLoad[i]);
        rg.forEach((_,i)=>rg[i]=output.cluster.relative.gradient[i]+lengths.relativeGradient[i]-relativeLoad[i]);
        const contactCertificate=lumen?.refresh({toolPositions:output.toolPositions,commonResidual:cg,relativeResidual:rg,order,consumeQuery:consumeContactQuery});
        if(contactCertificate)diagnostics.minimumPrivateNormalForce=Math.min(diagnostics.minimumPrivateNormalForce,contactCertificate.minForce);
        const frictionCertificate=friction?.refresh({toolPositions:output.toolPositions,commonResidual:cg,relativeResidual:rg,order});
        if(prescribedWallSites.length){backups.beforeWallCommon.set(cg);backups.beforeWallRelative.set(rg);}
        let wallCertificate=vessel?.refresh({toolPositions:output.toolPositions,commonResidual:cg,relativeResidual:rg,order,
            consumeQuery(){consumeContactQuery();diagnostics.wallQueries++;}});
        let knownOpenChanged=false;
        for(const {site,boundary} of prescribedWallSites) {
            // This gap is from the just-completed original query, at the
            // actual prescribed position. A dual physical BC not yet at its
            // target does not qualify. In particular g==0 retains its loaded
            // reaction gauge; no tolerance or force-sign clipping is used.
            const point=output.toolPositions.get(site.owner)[site.node];
            if(!(site.raw.gap>0)||!point.every((v,k)=>v===boundary.value[k]))continue;
            const Ft=wallFriction?.tractions,j=2*site.index;
            if(vessel.normalForces[site.base]===0&&(!Ft||(Ft[j]===0&&Ft[j+1]===0)))continue;
            // Exact unilateral solution for this known-open constant gap:
            // Fn=0 and the Coulomb disk of radius mu*Fn contains only Ft=0.
            vessel.normalForces[site.base]=0;if(Ft){Ft[j]=0;Ft[j+1]=0;}
            knownOpenChanged=true;diagnostics.wallKnownOpenBacksubstitutions++;
        }
        if(knownOpenChanged) {
            // Remove the old reaction once, then rebuild original rows and
            // their force/tangent certificate on the SAME verified geometry.
            // The mandatory final evaluate queries the original wall again;
            // a derivative-only refresh can never serve as the commit query.
            cg.set(backups.beforeWallCommon);rg.set(backups.beforeWallRelative);
            wallCertificate=vessel.refresh({toolPositions:output.toolPositions,commonResidual:cg,relativeResidual:rg,order,query:false});
            diagnostics.wallKnownOpenRowRefreshes++;
        }
        if(wallFriction&&!wallFrictionPrepared){wallFriction.prepare({toolPositions:output.toolPositions});wallFrictionPrepared=true;}
        const wallFrictionCertificate=wallFriction?.refresh({toolPositions:output.toolPositions,commonResidual:cg,relativeResidual:rg,order});
        if(wallCertificate){diagnostics.minimumPrivateNormalForce=Math.min(diagnostics.minimumPrivateNormalForce,wallCertificate.minForce);refreshDirectionRows();}
        for(const row of bcRows) {
            row.residual=output.toolPositions.get(row.toolId)[row.node][row.axis]-row.target;
            row.commonDofs.forEach((d,j)=>cg[d]+=row.forceColumn[j]*row.lambda);
            row.relativeDofs.forEach((d,j)=>rg[d]+=row.forceColumn[row.commonDofs.length+j]*row.lambda);
        }
        const physical=physicalResidual;
        let force=0,torque=0,merit=0,boundary=0;
        for(let node=0;node<n;node++) {
            const ids=nodeTools[node],start=layout.positions[node],mode=modes.get(node);
            if(mode) {
                const wire=physical.get(state.relativeToolId)[node];wire.fill(0);
                mode.basis.forEach((b,a)=>b.forEach((v,k)=>wire[k]+=v*rg[mode.relativeDofs[a]]));
                const other=physical.get(ids.find(id=>id!==state.relativeToolId))[node];for(let k=0;k<3;k++)other[k]=cg[start+k]-wire[k];
            } else for(let k=0;k<3;k++)physical.get(ids[0])[node][k]=cg[start+k];
            for(const id of ids)if(!positionBC.get(key(id,node))?.direct) {
                const f=Math.hypot(...physical.get(id)[node]);force=Math.max(force,f);merit+=(f/tol.force)**2;
            }
        }
        for(const spins of layout.spins.values())for(const d of spins)if(d>=0&&!fixed[d]){torque=Math.max(torque,Math.abs(cg[d]));merit+=(cg[d]/tol.torque)**2;}
        for(const row of lengths.rows)merit+=(row.residual/tol.length)**2;
        for(const b of positionBC.values()) {
            const r=Math.hypot(...output.toolPositions.get(b.toolId)[b.node].map((v,k)=>v-b.value[k]));
            boundary=Math.max(boundary,r);merit+=(r/tol.boundary)**2;
        }
        if(!Number.isFinite(merit)||!cg.every(Number.isFinite)||!rg.every(Number.isFinite))throw new RangeError('Nonfinite original joint certificate');
        let lineSearchMerit=merit;
        if(contactCertificate) {
            merit+=contactCertificate.merit;
            lineSearchMerit+=contactCertificate.lineSearchMerit??contactCertificate.merit;
        }
        if(wallCertificate)for(const sample of wallCertificate.samples)lineSearchMerit+=
            (Math.max(0,-sample.gap)/tol.wallGap)**2+(Math.max(0,-sample.Fn)/tol.force)**2+(sample.ncp/tol.wallNcp)**2;
        if(wallCertificate)lineSearchMerit+=wallCertificate.coverageMerit??0;
        if(wallCertificate)merit+=wallCertificate.merit;
        if(frictionCertificate){merit+=frictionCertificate.merit;lineSearchMerit+=frictionCertificate.lineSearchMerit??frictionCertificate.merit;}
        if(wallFrictionCertificate){merit+=wallFrictionCertificate.merit;lineSearchMerit+=wallFrictionCertificate.lineSearchMerit??wallFrictionCertificate.merit;}
        const certificate={force,torque,length:lengths.maximumResidual,boundary,merit,
            converged:force<=tol.force&&torque<=tol.torque&&lengths.maximumResidual<=tol.length&&boundary<=tol.boundary&&(!contactCertificate||contactCertificate.converged)&&(!wallCertificate||wallCertificate.converged)&&(!frictionCertificate||frictionCertificate.converged)&&(!wallFrictionCertificate||wallFrictionCertificate.converged)};
        if(candidate.lengthGeometry==='continuous-arclength')Object.assign(certificate,{lengthGeometry:'continuous-arclength',pointwiseInextensibility:false,
            parameterMetricDeviationBound:Math.max(...lengths.rows.map(row=>row.parameterMetricDeviationBound))});
        if(contactCertificate)certificate.contact=contactCertificate;
        if(frictionCertificate)certificate.friction=frictionCertificate;
        if(wallCertificate)certificate.wall=wallCertificate;
        if(wallFrictionCertificate)certificate.wallFriction=wallFrictionCertificate;
        // Fn*g is an ORIGINAL acceptance gate, but not an independent
        // equation in this Newton system. At Fn=0 its redundant product can
        // initially grow while both solved equations improve. Globalize the
        // actual force/NCP equations; keep the work gate unchanged at commit.
        if(wallCertificate||contactCertificate)certificate.lineSearchMerit=lineSearchMerit;
        diagnostics.certificate=certificate;
        return {certificate,physical};
    }
    const reject=(status,error=null)=>{
        diagnostics.iterationMs=performance.now()-iterationsStart;diagnostics.totalMs=performance.now()-started;
        return {accepted:false,status,state,diagnostics,error:error?.message??null,contacts:contactMode,certified:false};
    };
    let directionShift=0;
    const stabilize=reason=>{
        if(globalization!=='adaptive'||directionShift>=1)return false;
        directionShift=directionShift===0?.01:Math.min(1,10*directionShift);
        diagnostics.regularizationActivations.push({reason,shift:directionShift,direction:diagnostics.directions});
        return true;
    };
    try {
        lumen?.prepareGauge({toolPositions:state.toolPositions,consumeQuery:consumeContactQuery});
        friction?.prepare({consumeQuery(){consumeContactQuery();diagnostics.frictionPreparationQueries++;}});
        let measured=evaluate();
        while(true) {
        while(!measured.certificate.converged) {
            if(diagnostics.directions>=limits.directions)return reject('direction-budget');
            if(diagnostics.linearSolves>=limits.linearSolves)return reject('linear-solve-budget');
            if(!output.hessianValid)measured=evaluate('full');
            const directionStarted=performance.now();
            const direction=solveCompositeRelativeDirection(directionWorkspace,output.chain,{cluster:output.cluster,commonResidual:cg,relativeResidual:rg,fixed,rows,
                tolerances:{force:tol.linearForce,torque:tol.linearTorque},maxCorrections:Math.min(1,limits.linearSolves-diagnostics.linearSolves-1),numericalShift:directionShift});
            diagnostics.directionMs+=performance.now()-directionStarted;
            diagnostics.directions++;diagnostics.linearSolves+=direction.linearSolves;diagnostics.factorizations+=direction.factorizations;
            diagnostics.directionSystems.push({originalUnknowns:direction.count,solvedUnknowns:direction.solvedCount??direction.count,
                eliminatedZeroDuals:direction.eliminatedZeroDuals??0,bandwidth:direction.solvedBandwidth??direction.bandwidth,
                originalStorage:direction.originalStorage??'full-band',originalEntries:direction.originalMatrixEntries??direction.matrixEntries,
                originalBandEntries:direction.originalBandEntries??direction.matrixEntries,
                materializedOriginalBandEntries:direction.materializedOriginalBandEntries??direction.matrixEntries,
                materializedNumericalBandEntries:direction.materializedNumericalBandEntries??direction.matrixEntries});
            if(directionShift>0)diagnostics.numericalDirections++;
            if(!(directionShift>0?direction.numericalConverged:direction.converged)){
                diagnostics.lastLinearFailure={original:direction.proof,numerical:direction.numericalProof,shift:directionShift};
                if(stabilize('linear-system'))continue;
                return reject('original-linear-equations');
            }
            if(vessel) {
                const deltaPhysical=new Map([...layout.spins.keys()].map(id=>[id,Array.from({length:n},(_,node)=>
                    Array.from({length:3},(_,axis)=>direction.commonIncrement[layout.positions[node]+axis]))]));
                for(const mode of cluster.modes)mode.basis.forEach((b,a)=>b.forEach((v,k)=>
                    deltaPhysical.get(state.relativeToolId)[mode.node][k]+=v*direction.relativeIncrement[mode.relativeDofs[a]]));
                if(vessel.discoverCharts({deltaPhysical}).rowStructureChanged) {
                    // The newly admitted original SDF cone changes the dual
                    // coordinates. Reassemble both materials at this SAME
                    // geometry and solve a fresh direction before any trial.
                    diagnostics.wallChartDiscoveries++;refreshDirectionRows();measured=evaluate('full');continue;
                }
            }
            const {common:base,relative,lambda,boundary:bc}=backups,merit=measured.certificate.lineSearchMerit??measured.certificate.merit;
            slots.forEach((s,i)=>base[i]=s.array[s.index]);relative.set(candidate.relative);lambda.set(candidate.lengthMultipliers);bcRows.forEach((row,i)=>bc[i]=row.lambda);
            const inactive=lumen?.samples.map(s=>!s.active);if(lumen)backups.normal.set(lumen.normalForces);
            if(friction)backups.tractions.set(friction.tractions);
            const wallCheckpoint=vessel?.checkpoint(),wallRows=vessel?.rows,wallIndices=vessel?.rowForceIndices;
            const wallInactive=wallRows?.map(row=>row.multiplierDerivative>0);
            if(vessel)backups.wallNormal.set(vessel.normalForces);
            if(wallFriction)backups.wallTractions.set(wallFriction.tractions);
            let found=false,tinyNewtonStep=false,nextAlpha=1,nextConeTrial=false;
            for(let trial=0;trial<limits.lineSearchTrials;trial++) {
                const alpha=nextAlpha,coneTrial=nextConeTrial;nextAlpha=alpha/2;nextConeTrial=false;diagnostics.lineSearchTrials++;
                // A nearly singular unshifted direction can demand enormous
                // reactions and accept only a tiny step into a worse chart.
                // Rebuild the search direction at the SAME base state before
                // taking that step. This changes only numerical iteration;
                // every accepted physical dt still passes evaluate() below.
                if(globalization==='adaptive'&&directionShift===0&&alpha<2**-12){tinyNewtonStep=true;diagnostics.tinyNewtonStepsAvoided++;break;}
                if(coneTrial)diagnostics.frictionConeTrials++;
                // A rejected trial may have discovered a different SDF chart
                // or duplicate gauge. Restore its metadata as well as Fn.
                if(vessel)vessel.restore(wallCheckpoint);
                slots.forEach((s,i)=>s.array[s.index]=fixed[i]?base[i]:base[i]+alpha*direction.commonIncrement[i]);
                candidate.relative.forEach((_,i)=>candidate.relative[i]=relative[i]+alpha*direction.relativeIncrement[i]);
                activeLengths.forEach((row,i)=>candidate.lengthMultipliers[row.index]=lambda[row.index]+alpha*direction.multiplierIncrement[i]);
                bcRows.forEach((row,i)=>row.lambda=bc[i]+alpha*direction.multiplierIncrement[activeLengths.length+i]);
                if(lumen)lumen.rowSampleIndices.forEach((i,rowIndex)=>{
                    // Exact back-substitution of the prepared inactive linear
                    // equation Fn_target=0, not a force/threshold projection.
                    // Active Newton values remain signed and unprojected.
                    lumen.normalForces[i]=inactive[i]?(1-alpha)*backups.normal[i]:backups.normal[i]+alpha*direction.multiplierIncrement[activeLengths.length+bcRows.length+rowIndex];
                });
                if(friction)lumen.rowSampleIndices.forEach((i,rowIndex)=>{
                    // With Fn<=0 the projection has DPz=0. An inactive
                    // normal equation gives Fn_target=0; at Fn=0 its load
                    // derivative multiplies dFn=0. Both exact linear Ft
                    // targets are therefore zero. Back-substitute that row
                    // without leaving a roundoff traction at a zero cone.
                    const zeroTarget=inactive[i]&&backups.normal[i]<=0;
                    for(let component=0;component<2;component++)friction.tractions[2*i+component]=zeroTarget?(1-alpha)*backups.tractions[2*i+component]:
                        backups.tractions[2*i+component]+alpha*direction.multiplierIncrement[frictionRowOffset+2*rowIndex+component];
                });
                if(vessel)wallIndices.forEach((i,rowIndex)=>{
                    vessel.normalForces[i]=wallInactive[rowIndex]?(1-alpha)*backups.wallNormal[i]:
                        backups.wallNormal[i]+alpha*direction.multiplierIncrement[baseRows.length+rowIndex];
                });
                if(wallFriction)wallIndices.forEach((i,rowIndex)=>{
                    const zeroTarget=wallInactive[rowIndex]&&backups.wallNormal[i]<=0;
                    for(let c=0;c<2;c++)wallFriction.tractions[2*rowIndex+c]=zeroTarget?(1-alpha)*backups.wallTractions[2*rowIndex+c]:
                        backups.wallTractions[2*rowIndex+c]+alpha*direction.multiplierIncrement[wallFrictionRowOffset+2*rowIndex+c];
                });
                let next;
                try {next=evaluate();}
                catch(error) {
                    if(error.message==='evaluation-budget'||error.code==='contact-query-budget')throw error;
                    // Undefined geometry (e.g. an exactly reversed hinge) is
                    // not a candidate certificate. A shorter trial may remain
                    // on the valid original geometric branch.
                    diagnostics.invalidTrials=(diagnostics.invalidTrials??0)+1;
                    diagnostics.lastInvalidTrial=error.message;continue;
                }
                if(next.certificate.converged||(next.certificate.lineSearchMerit??next.certificate.merit)<merit*(1-1e-4*alpha)) {
                    measured=next;found=true;diagnostics.acceptedAlphas.push(alpha);if(coneTrial)diagnostics.frictionConeAccepted++;break;
                }
                if(globalization==='adaptive'&&friction) {
                    // A sticking Newton direction may require traction past
                    // the opposite Coulomb boundary. Try its exact affine
                    // force crossing before repeatedly halving into that
                    // corner. All q/rho/spin/dual coordinates still take the
                    // SAME alpha and pass the original merit and final gates.
                    const hint=proposeCompositeFrictionBacktrack(measured.certificate.friction.samples,next.certificate.friction.samples,alpha);
                    if(hint!==null){nextAlpha=hint;nextConeTrial=true;}
                }
            }
            if(!found){
                if(!stabilize(tinyNewtonStep?'tiny-newton-step':'line-search'))return reject('line-search');
                slots.forEach((s,i)=>s.array[s.index]=base[i]);candidate.relative.set(relative);candidate.lengthMultipliers.set(lambda);
                bcRows.forEach((row,i)=>row.lambda=bc[i]);
                if(lumen)lumen.normalForces.set(backups.normal);if(friction)friction.tractions.set(backups.tractions);
                if(vessel)vessel.restore(wallCheckpoint);if(wallFriction)wallFriction.tractions.set(backups.wallTractions);
                refreshDirectionRows();measured=evaluate('full');
            }
        }
        // A fresh original certificate is mandatory even when the initial
        // prepared candidate or the last trial already appeared converged.
        measured=evaluate();if(!measured.certificate.converged)return reject('fresh-original-certificate');
        if(wallFriction) {
            const mode=wallFriction.resolveModes({wholeStepConverged:true});
            if(mode.samples.length)diagnostics.wallFrictionModes=mode;
            if(mode.changed) {
                // A constitutive branch change keeps the SAME prepared dt,
                // old material history and common mechanics. Do not apply a
                // second force kick or solve either tool independently. All
                // evaluation/query/direction budgets remain cumulative.
                diagnostics.wallFrictionModeChanges=(diagnostics.wallFrictionModeChanges??0)+mode.samples.filter(s=>s.change).length;
                measured=evaluate('full');continue;
            }
            if(!mode.accepted)return reject(`wall-friction-mode-${mode.status}`);
        }
        break;
        }
        diagnostics.iterationMs=performance.now()-iterationsStart;
        const commitStart=performance.now(),physical=new Map([...output.toolPositions].map(([id,p])=>[id,ownPoints(p)]));
        const perTool=new Map([...output.perTool].map(([id,p])=>[id,{...p,momentum:Array.from(p.momentum),oldMomentum:Array.from(p.oldMomentum),momentumIncrement:Array.from(p.momentumIncrement)}]));
        const boundaryForces=new Map([...layout.spins.keys()].map(id=>[id,Array.from({length:n},()=>[0,0,0])]));
        for(const b of positionBC.values())if(b.direct)boundaryForces.get(b.toolId)[b.node]=measured.physical.get(b.toolId)[b.node].slice();
        for(const row of bcRows)boundaryForces.get(row.toolId)[row.node][row.axis]=row.lambda;
        const balances=new Map();
        const contactForces=lumen||vessel?new Map([...layout.spins.keys()].map(id=>[id,Array.from({length:n},(_,node)=>
            Array.from({length:3},(_,k)=>(lumen?.nodalForces.get(id)[node][k]??0)+(friction?.nodalForces.get(id)[node][k]??0)+(vessel?.nodalForces.get(id)[node][k]??0)+(wallFriction?.nodalForces.get(id)[node][k]??0)))])):null;
        for(const [id,p] of perTool) {
            const support=[0,0,0];boundaryForces.get(id).forEach(v=>v.forEach((f,k)=>support[k]+=f));
            const momentumRate=p.momentumIncrement.map(v=>v/dt),load=applied.get(id),residual=momentumRate.map((v,k)=>v-load[k]-support[k]);
            balances.set(id,{momentumRate,appliedForce:load.slice(),boundaryForce:support,residual});
            if(contactForces) {
                const contactForce=[0,0,0];contactForces.get(id).forEach(v=>v.forEach((f,k)=>contactForce[k]+=f));
                balances.get(id).contactForce=contactForce;for(let k=0;k<3;k++)residual[k]-=contactForce[k];
            }
        }
        const spinReactions=new Map([...layout.spins].map(([id,spins])=>[id,Float64Array.from(spins,d=>d>=0&&fixed[d]?cg[d]:0)]));
        commitFrames(candidate,physical);candidate.toolPositions=physical;candidate.materialVelocities=acceptedVelocities(candidate,prepared,dt);
        candidate.boundaryMultipliers=new Map([...positionBC.values()].filter(b=>!b.direct).map(b=>[key(b.toolId,b.node),boundaryForces.get(b.toolId)[b.node].slice()]));
        candidate.time=finite(state.time+dt,'accepted time');candidate.step=state.step+1;
        if(lumen)candidate.lumenContactState=lumen.commit();
        if(friction)candidate.lumenFrictionState=friction.commit();
        if(vessel)candidate.wallContactState=vessel.commit();
        if(wallFriction)candidate.wallFrictionState=wallFriction.commit();
        diagnostics.commitMs=performance.now()-commitStart;diagnostics.totalMs=performance.now()-started;
        return {accepted:true,status:'accepted',state:candidate,diagnostics,perTool,boundaryForces,spinReactions,balances,
            ...(contactForces?{contactForces}:{}),contacts:contactMode,certified:true,
            certificateScope:wallFriction?'original-discrete-force-torque-length-boundaries-and-declared-wall-Coulomb-and-lumen-contacts':friction?'original-discrete-force-torque-length-boundaries-and-declared-lumen-Coulomb-and-normal-wall-contacts':vessel?'original-discrete-force-torque-length-boundaries-and-declared-normal-wall-lumen-contacts':lumen?'original-discrete-force-torque-length-boundaries-and-all-declared-lumen-samples':'original-discrete-force-torque-length-and-position-boundaries',includesAngularInertia:false};
    } catch(error) {return reject(error.message==='evaluation-budget'?'evaluation-budget':error.code??'invalid-nonlinear-candidate',error);}
}
