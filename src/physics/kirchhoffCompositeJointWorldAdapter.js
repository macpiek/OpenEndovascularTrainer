import {createCompositeJointTimeStepState,createCompositeJointTimeStepWorkspace,advanceCompositeJointTimeStep,iterateCompositeJointTimeStep} from './kirchhoffCompositeJointTimeStep.js';
import {readCompositeJointSurfacePosePath} from './kirchhoffCompositeJointSurfacePoseHistory.js';
import {prepareCompositeJointWorldWall,assertCompositeJointWorldWall} from './kirchhoffCompositeJointWorldWall.js';
import {compositeContinuousBezierControls} from './kirchhoffCompositeContinuousGeometry.js';
import {prepareCompositeJointWorldSheath,assertCompositeJointWorldSheath} from './kirchhoffCompositeJointWorldSheath.js';
import {prepareCompositeJointWorldContainment,assertCompositeJointWorldContainment} from './kirchhoffCompositeJointWorldContainment.js';
import {prepareCompositeJointWorldToolContact,assertCompositeJointWorldToolContact} from './kirchhoffCompositeJointWorldToolContact.js';

const fail=(status,message)=>{const e=new RangeError(message);e.code=status;throw e;};
const ownVector=(v,n,name)=>{if(v?.length!==n||!Array.from(v).every(Number.isFinite))fail('joint-world-invalid-state',`${name} requires ${n} finite values`);return Array.from(v);};
const activeIds=(layout,node)=>new Set([...(layout.edgeToolIds[node-1]??[]),...(layout.edgeToolIds[node]??[])]);
const dot=(a,b)=>a.reduce((s,v,i)=>s+v*b[i],0);
const cross=(a,b)=>[a[1]*b[2]-a[2]*b[1],a[2]*b[0]-a[0]*b[2],a[0]*b[1]-a[1]*b[0]];
const keys=['x','y','z','previousX','previousY','previousZ','velocityX','velocityY','velocityZ',
    'orientationX','orientationY','orientationZ','orientationW','previousOrientationX','previousOrientationY','previousOrientationZ','previousOrientationW'];

// Numeric preparation is owned once per dt. The original collision field is
// deliberately borrowed: copying an anatomy here changes source identity and
// duplicates its storage. Functions are immutable material/query providers.
function ownPrepared(value,borrowed=new Set(),seen=new Map()) {
    if(value===null||typeof value!=='object'||borrowed.has(value))return value;
    if(seen.has(value))return seen.get(value);
    if(ArrayBuffer.isView(value)){if(value instanceof DataView)throw new TypeError('DataView is not a prepared numeric array');const out=value.slice();seen.set(value,out);return out;}
    if(value instanceof Map){const out=new Map();seen.set(value,out);for(const [k,v] of value)out.set(k,ownPrepared(v,borrowed,seen));return out;}
    if(value instanceof Set){const out=new Set();seen.set(value,out);for(const v of value)out.add(ownPrepared(v,borrowed,seen));return out;}
    if(Array.isArray(value)){const out=[];seen.set(value,out);for(const v of value)out.push(ownPrepared(v,borrowed,seen));return out;}
    if(Object.getPrototypeOf(value)!==Object.prototype&&Object.getPrototypeOf(value)!==null)throw new TypeError('Prepared data needs plain records or an explicitly borrowed collision field');
    const out={};seen.set(value,out);for(const [k,v] of Object.entries(value))out[k]=ownPrepared(v,borrowed,seen);return out;
}
function ownState(source) {
    const state=createCompositeJointTimeStepState(source);
    state.boundaryMultipliers=ownPrepared(source.boundaryMultipliers??new Map());
    state.materialVelocities=ownPrepared(source.materialVelocities??null);
    return state;
}

function sameValue(a,b) {
    if(Object.is(a,b))return true;
    if(!a||!b||typeof a!=='object'||typeof b!=='object'||a.constructor!==b.constructor)return false;
    if(a instanceof Map)return a.size===b.size&&Array.from(a).every(([k,v])=>b.has(k)&&sameValue(v,b.get(k)));
    const keys=Object.keys(a);return keys.length===Object.keys(b).length&&keys.every(k=>Object.hasOwn(b,k)&&sameValue(a[k],b[k]));
}
function prepareMaterialState(state,source) {
    if(source===undefined)return state;
    const physical=value=>({...value,tools:value.tools.map(tool=>{
        const {material,materialAt,materialBreaks,appMaterialProfile,...rest}=tool;return rest;
    })});
    if(!source||!Array.isArray(source.tools)||!sameValue(physical(state),physical(source)))
        fail('joint-world-invalid-material-update','Prepared profiles may not replace accepted geometry, motion, winding, constraints or history');
    return ownState(source);
}

function quaternion(state,id,edge) {
    const tool=state.tools.find(t=>t.id===id),f=tool.reference[edge],angle=state.angles.get(id)[edge],t=f.tangent,
        transverse=cross(t,f.director),c=Math.cos(angle),s=Math.sin(angle),d=f.director.map((v,k)=>c*v+s*transverse[k]),b=cross(t,d);
    const m00=d[0],m01=b[0],m02=t[0],m10=d[1],m11=b[1],m12=t[1],m20=d[2],m21=b[2],m22=t[2],trace=m00+m11+m22;
    let x,y,z,w;
    if(trace>0){const a=2*Math.sqrt(1+trace);w=a/4;x=(m21-m12)/a;y=(m02-m20)/a;z=(m10-m01)/a;}
    else if(m00>m11&&m00>m22){const a=2*Math.sqrt(1+m00-m11-m22);w=(m21-m12)/a;x=a/4;y=(m01+m10)/a;z=(m02+m20)/a;}
    else if(m11>m22){const a=2*Math.sqrt(1+m11-m00-m22);w=(m02-m20)/a;x=(m01+m10)/a;y=a/4;z=(m12+m21)/a;}
    else {const a=2*Math.sqrt(1+m22-m00-m11);w=(m10-m01)/a;x=(m02+m20)/a;y=(m12+m21)/a;z=a/4;}
    const n=Math.hypot(x,y,z,w);return ownVector([x/n,y/n,z/n,w/n],4,'Own orientation quaternion');
}
function velocity(state,id,node,trace) {
    const edge=trace==='right'?node:node-1,end=trace==='right'?0:1;
    if(!state.layout.edgeToolIds[edge]?.includes(id))fail('joint-world-invalid-binding','Velocity trace must lie on the actual tool');
    const record=state.materialVelocities?.[edge]?.tools.find(t=>t.id===id);
    if(record?.interpretation==='quintic-bernstein-material-velocity') {
        if(record.bernsteinVelocities?.length!==6)fail('joint-world-missing-velocity','Continuous publication requires all six accepted velocity controls');
        return ownVector(record.bernsteinVelocities[end===0?0:5],3,'Accepted continuous endpoint material velocity');
    }
    if(record?.interpretation!=='physical-material-velocity')fail('joint-world-missing-velocity','Publication requires accepted physical material velocity');
    return ownVector(record.velocities[end],3,'Accepted material velocity');
}
function createBindings(world,state,input) {
    if(!Array.isArray(input)||input.length!==state.tools.length||input.length!==world.bodies.length)
        fail('joint-world-incomplete-binding','Every actual World body and Joint material must be bound once');
    const ids=new Set(),bodies=new Set();
    return input.map(b=>{
        const {body,toolId}=b;
        if(!world.bodies.includes(body)||bodies.has(body)||ids.has(toolId)||!state.layout.spins.has(toolId))fail('joint-world-invalid-binding','Bindings need distinct actual bodies and physical tools');
        ids.add(toolId);bodies.add(body);
        const start=body.activeStart,end=body.activeEnd;
        if(!Number.isInteger(start)||!Number.isInteger(end)||end<=start)fail('joint-world-invalid-binding','A bound body needs an active physical edge');
        const nodes=(b.nodes??[]).map(r=>{
            if(!Number.isInteger(r.node)||r.node<start||r.node>end||!Number.isInteger(r.jointNode)||!activeIds(state.layout,r.jointNode).has(toolId)||!['left','right'].includes(r.trace))
                fail('joint-world-invalid-binding','Every body node needs an active Joint node and explicit material velocity trace');
            const edge=r.trace==='right'?r.jointNode:r.jointNode-1;
            if(!state.layout.edgeToolIds[edge]?.includes(toolId))fail('joint-world-invalid-binding','The declared velocity trace leaves its physical tool');
            return {node:r.node,jointNode:r.jointNode,trace:r.trace};
        });
        const edges=(b.edges??[]).map(r=>{
            if(!Number.isInteger(r.edge)||r.edge<start||r.edge>=end||!Number.isInteger(r.jointEdge)||!state.layout.edgeToolIds[r.jointEdge]?.includes(toolId))
                fail('joint-world-invalid-binding','Every body orientation needs an actual own Joint edge');
            return {edge:r.edge,jointEdge:r.jointEdge};
        });
        if(nodes.length!==end-start+1||new Set(nodes.map(r=>r.node)).size!==nodes.length||edges.length!==end-start||new Set(edges.map(r=>r.edge)).size!==edges.length)
            fail('joint-world-incomplete-binding','All active body nodes and edges must be covered exactly once');
        const arrays=Object.fromEntries(keys.map(key=>{
            const array=body[key];if(!(array instanceof Float32Array||array instanceof Float64Array)||array.length<(key.includes('Orientation')||key.startsWith('orientation')?end:end+1))
                fail('joint-world-invalid-binding',`Missing numeric body view ${key}`);
            return [key,array];
        }));
        const writes=[];
        for(const r of nodes)for(let k=0;k<9;k++)writes.push({target:arrays[keys[k]],key:keys[k],index:r.node,kind:'node',record:r,component:k%3,quantity:Math.floor(k/3)});
        for(const r of edges)for(let k=0;k<8;k++)writes.push({target:arrays[keys[9+k]],key:keys[9+k],index:r.edge,kind:'edge',record:r,component:k%4,quantity:Math.floor(k/4)});
        return {body,toolId,start,end,count:body.count,nodes,edges,arrays,writes,values:new Float64Array(writes.length),backup:new Float64Array(writes.length)};
    });
}
function verifyBindings(world,bindings) {
    if(world.bodies.length!==bindings.length)fail('joint-world-remap-required','The set of physical World bodies changed');
    for(const b of bindings){
        if(!world.bodies.includes(b.body)||b.body.activeStart!==b.start||b.body.activeEnd!==b.end||b.body.count!==b.count)
            fail('joint-world-remap-required','Active physical range or storage changed; a state transfer is required');
        for(const key of keys)if(b.body[key]!==b.arrays[key])fail('joint-world-remap-required','A published body storage array was replaced');
        const descriptor=Object.getOwnPropertyDescriptor(b.body,'jointStateView');
        if(descriptor&&!descriptor.configurable)fail('joint-world-publication-invalid','jointStateView must be an owned replaceable view');
        if(!descriptor&&!Object.isExtensible(b.body))fail('joint-world-publication-invalid','Body cannot receive its complete Joint view');
    }
}
// Both actual World sources enter one native normal/Coulomb block. The
// source proofs remain separate, and the combined numeric law is checked
// before every retry; exterior friction does not replace lumen coefficients.
function combineWorldToolContacts(sources) {
    const active=sources.filter(c=>c&&c!=='none');
    if(!active.length)return 'none';
    if(active.length===1)return active[0];
    const pairs=active.flatMap(c=>c.pairs),muByPair={};
    for(const c of active)for(const p of c.pairs) {
        if(Object.hasOwn(muByPair,p.id))throw new RangeError('Actual World tool pair IDs must be distinct');
        muByPair[p.id]=c.friction==='none'?[0,0]:Array.from(c.friction.muByPair?.[p.id]??c.friction.mu);
    }
    if(active.some(c=>c.forcePerLength!==active[0].forcePerLength))throw new RangeError('One native normal block requires one numerical force scale');
    const laws=active.filter(c=>c.friction!=='none').map(c=>c.friction),law=laws[0];
    if(laws.some(f=>f.law!=='coulomb'||f.rateMode!=='backward-euler-grid'||f.slipModel!=='implicit-backward-euler-surface-rate'||f.finiteStepSlipKnown!==false||f.forcePerLength!==law.forcePerLength))
        throw new RangeError('Merged native tools require the same explicit implicit-rate Coulomb law');
    return {mode:law?'lumen-coulomb':'lumen-normal',chartId:'world-native-tool-contacts',forcePerLength:active[0].forcePerLength,pairs,
        ...(active.every(c=>c.historyUpdate==='preserve-and-append')?{historyUpdate:'preserve-and-append'}:{}),
        friction:law?{...law,mu:Array.from(law.mu),muByPair}:'none'};
}
function guardWorldConstraints(world,state,bindings,pending) {
    // No World constraint is silently delegated to an old post-pass. Wall
    // coverage has its source adapter; the remaining constraints still need one.
    if(world.sheaths.some(c=>c.enabled!==false)&&!pending.sheathProof)
        fail('joint-world-sheath-adapter-required','Active sheath constraints need their common-step adapter');
    if(pending.sheathProof)
        assertCompositeJointWorldSheath({proof:pending.sheathProof,world,state,bindings,sheath:pending.options.sheath});
    if(world.containments.some(c=>c.enabled!==false)&&!pending.containmentProof)fail('joint-world-containment-adapter-required','Active World containment needs source-to-Joint contact coverage');
    if(pending.containmentProof)assertCompositeJointWorldContainment({proof:pending.containmentProof,world,state,bindings,contacts:pending.containmentContacts});
    if(pending.toolContactProof)assertCompositeJointWorldToolContact({proof:pending.toolContactProof,world,state,bindings,contacts:pending.toolContacts});
    if((pending.containmentProof||pending.toolContactProof)&&JSON.stringify(pending.options.contacts)!==pending.combinedContactsSignature)fail('joint-world-tool-contact-law-changed','The complete prepared World tool contact block changed');
    if(world.toolContacts.some(c=>c.enabled!==false)&&!pending.toolContactProof)fail('joint-world-external-contact-adapter-required','Active external tool constraints need source-to-Joint coverage');
    // Matching a field object alone proves neither exposed-surface coverage
    // nor the original radii/friction law. Even a formally enabled wall with
    // all owners null can have a vacuously converged contact certificate.
    if(world.contactField||pending.wallProof)assertCompositeJointWorldWall({proof:pending.wallProof,world,state,bindings,wall:pending.options.wall,inertia:pending.options.inertia});
}
function completeView(state,binding,publicationOrigin) {
    const id=binding.toolId,nodes=Int32Array.from(Array.from({length:state.layout.nodeCount},(_,i)=>i).filter(i=>activeIds(state.layout,i).has(id))),
        edges=Int32Array.from(state.layout.edgeToolIds.flatMap((ids,e)=>ids.includes(id)?[e]:[]));
    const geometry=state.inertiaGeometryByTool?.get(id),positions=state.toolPositions.get(id),
        continuousCurve=geometry?Object.freeze({kind:'quintic-bernstein',
            coordinates:Float64Array.from(nodes,n=>state.coordinates[n]),
            controls:Float64Array.from(Array.from(edges,e=>{const g=geometry.edges[e];return compositeContinuousBezierControls(g,g.nodeIndices.map(n=>positions[n])).flat().map((v,i)=>v+(publicationOrigin?.[i%3]??0));}).flat()),
            materialVelocityControls:Float64Array.from(Array.from(edges,e=>state.materialVelocities[e].tools.find(t=>t.id===id).bernsteinVelocities.flat()).flat())}):null;
    return Object.freeze({toolId:id,time:state.time,step:state.step,materialCoordinate:state.materialCoordinate,
        nodes,edges,coordinates:Float64Array.from(nodes,n=>state.coordinates[n]),
        positions:Float64Array.from(Array.from(nodes,n=>state.toolPositions.get(id)[n]).flat(),(v,i)=>v+(publicationOrigin?.[i%3]??0)),
        orientations:Float64Array.from(Array.from(edges,e=>quaternion(state,id,e)).flat()),
        unwrappedAngles:Float64Array.from(edges,e=>state.angles.get(id)[e]),
        edgeMaterialVelocities:Float64Array.from(Array.from(edges,e=>[...velocity(state,id,e,'right'),...velocity(state,id,e+1,'left')]).flat()),
        angularInertia:'quasi-static',materialAngularVelocity:null,continuousCurve,
        scope:'complete-owned-joint-geometry-and-edge-velocity-view',coarseBodyArraysArePhysicsState:false});
}
function stagePublication(world,bindings,previous,next,publicationOrigin) {
    verifyBindings(world,bindings);const staged=[];
    for(const b of bindings){
        const values=new Map(),oldQuaternions=new Map(),newQuaternions=new Map();
        for(const r of b.nodes)values.set(r,[next.toolPositions.get(b.toolId)[r.jointNode],previous.toolPositions.get(b.toolId)[r.jointNode],velocity(next,b.toolId,r.jointNode,r.trace)]);
        for(const r of b.edges){const old=quaternion(previous,b.toolId,r.jointEdge),current=quaternion(next,b.toolId,r.jointEdge);if(dot(old,current)<0)current.forEach((_,i)=>current[i]=-current[i]);oldQuaternions.set(r,old);newQuaternions.set(r,current);}
        b.writes.forEach((w,i)=>{
            const raw=w.kind==='node'?values.get(w.record)[w.quantity][w.component]+(w.quantity<2?(publicationOrigin?.[w.component]??0):0):(w.quantity?oldQuaternions:newQuaternions).get(w.record)[w.component];
            const value=w.target instanceof Float32Array?Math.fround(raw):raw;
            if(!Number.isFinite(value))fail('joint-world-publication-invalid','Accepted state cannot be represented by its body view');
            b.values[i]=value;b.backup[i]=w.target[w.index];
        });
        staged.push({binding:b,view:completeView(next,b,publicationOrigin),oldView:Object.getOwnPropertyDescriptor(b.body,'jointStateView')});
    }
    return staged;
}
function publish(staged) {
    try {
        for(const {binding:b,view} of staged){b.writes.forEach((w,i)=>w.target[w.index]=b.values[i]);Object.defineProperty(b.body,'jointStateView',{value:view,writable:false,configurable:true,enumerable:true});}
    } catch(error){
        for(const {binding:b,oldView} of staged){b.writes.forEach((w,i)=>w.target[w.index]=b.backup[i]);if(oldView)Object.defineProperty(b.body,'jointStateView',oldView);else delete b.body.jointStateView;}
        throw error;
    }
}

/** Actual whole-dt bridge: initialize imports once, prepareStep owns one set
 * of commands/contact data, and EVERY attempt calls the production Joint dt.
 * Only accepted state is published; Float32 body arrays are never re-imported
 * as the next physics state. The full union geometry and unwrapped spins are
 * retained in owned body.jointStateView even when body arrays are coarser.
 *
 * initialize(world) -> {state,bindings,publicationOrigin?}; prepareStep({world,state,dt}) -> Joint
 * options, with read-only state and immutable collision/material providers.
 * worldWall opts into actual body ranges/radii/friction; then prepareStep may
 * provide worldWallSurfacePosePaths, but cannot override the derived wall law.
 * initialize/prepareStep are synchronous and must not mutate World or state.
 * Unsupported active World constraints and topology changes explicitly reject
 * until their real source/transfer adapters exist. This is NOT the app factory
 * or a certificate for omitted anatomy, remeshing, material spin rate or FPS.
 */
export function createCompositeJointWorldAdapter({initialize,prepareStep,transferStep=null,worldWall,worldSheath=false,worldContainment=false,worldToolContact=worldContainment,cooperative=false,workSliceMs=0,id='composite-joint-experimental',elementBackend='wasm-exact'}={}) {
    if(typeof initialize!=='function'||typeof prepareStep!=='function'||typeof id!=='string'||!id)throw new TypeError('Whole-step adapter needs an ID and synchronous initialize/prepareStep functions');
    if(worldWall!==undefined&&(!worldWall||Object.getPrototypeOf(worldWall)!==Object.prototype))throw new TypeError('worldWall must be an explicit source adapter configuration');
    if(typeof cooperative!=='boolean')throw new TypeError('cooperative must be boolean');
    if(!Number.isFinite(workSliceMs)||workSliceMs<0)throw new TypeError('workSliceMs must be finite and nonnegative');
    if(transferStep!==null&&typeof transferStep!=='function')throw new TypeError('transferStep must be a synchronous material transfer callback');
    const wallConfiguration=worldWall===undefined?null:ownPrepared(worldWall);
    let publicationOrigin=null,owner=null,state=null,bindings=null,workspace=null,pending=null,busy=false,initializationError=null,budgetOverride=null,last=null;
    const stats={initializations:0,preparations:0,attempts:0,accepted:0,rejected:0,publications:0};
    function resultFailure(dt,status,error,diagnostics=null){stats.rejected++;return last={accepted:false,dt,status,message:error?.message,errorDetails:error?.details??null,diagnostics};}
    const adapter={id,
        step(world,dt){
            if(busy)throw new RangeError('Joint World adapter is busy');
            if(!Number.isFinite(dt)||!(dt>0))throw new RangeError('Whole-step dt must be positive');
            if(owner&&owner!==world)throw new RangeError('A Joint World adapter belongs to exactly one World');
            if(pending&&pending.dt!==dt)throw new RangeError('A prepared Joint step must retain dt');
            busy=true;stats.attempts++;
            try {
                owner=world;
                if(initializationError)return resultFailure(dt,'joint-world-initialization-error',initializationError);
                if(!state){
                    stats.initializations++;
                    try {const initial=initialize(world);if(initial?.then)throw new TypeError('World initialization must be synchronous');
                        const origin=initial?.publicationOrigin;
                        if(origin!=null&&(origin.length!==3||!Array.from(origin).every(Number.isFinite)))throw new TypeError('publicationOrigin must contain three finite coordinates');
                        publicationOrigin=origin==null?null:Float64Array.from(origin);
                        const next=ownState(initial?.state);const mapping=createBindings(world,next,initial.bindings);
                        workspace=createCompositeJointTimeStepWorkspace({...next,elementBackend});state=next;bindings=mapping;
                    } catch(error){initializationError=error;return resultFailure(dt,error.code??'joint-world-initialization-error',error);}
                }
                if(!pending){
                    pending={dt,options:null,error:null,wallProof:null,sheathProof:null,containmentProof:null,toolContactProof:null,containmentContacts:null,toolContacts:null,combinedContactsSignature:null,state,bindings,workspace,transfer:null};stats.preparations++;
                    try {
                        const transfer=transferStep?.({world,state:ownState(state),dt,
                            bindings:bindings.map(({body,toolId,nodes,edges})=>({body,toolId,nodes:ownPrepared(nodes),edges:ownPrepared(edges)}))});
                        if(transfer?.then)throw new TypeError('Joint material transfer must be synchronous');
                        if(transfer){
                            const remapped=ownState(transfer.state);
                            if(remapped.time!==state.time||remapped.step!==state.step||remapped.tools.length!==state.tools.length||
                                remapped.tools.some(t=>!state.tools.some(previous=>previous.id===t.id)))
                                fail('joint-world-invalid-transfer','A mesh transfer must retain physical time and the same tools');
                            pending.state=remapped;
                            pending.bindings=createBindings(world,remapped,transfer.bindings);
                            pending.workspace=createCompositeJointTimeStepWorkspace({...remapped,elementBackend});
                            pending.transfer=ownPrepared(transfer.diagnostics??{});
                        }
                        verifyBindings(world,pending.bindings);
                        let input=prepareStep({world,state:pending.state,bindings:pending.bindings,dt,transfer:pending.transfer});
                        if(!input||input.then)throw new TypeError('Joint preparation must return synchronous options');
                        const {preparedState,...stepInput}=input;input=stepInput;
                        pending.state=prepareMaterialState(pending.state,preparedState);
                        if(input.dt!==undefined&&input.dt!==dt)throw new RangeError('Prepared Joint dt differs from World dt');
                        if(input.workspace!==undefined)throw new RangeError('The World adapter owns the Joint workspace');
                        if(input.elementBackend!==undefined&&input.elementBackend!==elementBackend)throw new RangeError('Prepared Joint backend changed');
                        if(input.worldWallSurfacePosePaths!==undefined&&!(wallConfiguration&&world.contactField))throw new RangeError('World wall pose paths require the actual World wall source adapter');
                        if(worldSheath) {
                            if(input.sheath!==undefined&&input.sheath!=='none')throw new RangeError('World source adapter owns the sheath law');
                            const prepared=prepareCompositeJointWorldSheath({world,state:pending.state,bindings:pending.bindings});
                            pending.sheathProof=prepared.proof;input={...input,sheath:prepared.sheath};
                        }
                        if(worldContainment||worldToolContact) {
                            if(input.contacts!==undefined&&input.contacts!=='none')throw new RangeError('World source adapter owns the containment law');
                            if(worldContainment) {
                                const prepared=prepareCompositeJointWorldContainment({world,state:pending.state,bindings:pending.bindings,inertia:input.inertia});
                                pending.containmentProof=prepared.proof;pending.containmentContacts=prepared.contacts;
                            }
                            if(worldToolContact) {
                                const prepared=prepareCompositeJointWorldToolContact({world,state:pending.state,bindings:pending.bindings});
                                pending.toolContactProof=prepared.proof;pending.toolContacts=prepared.contacts;
                            }
                            const contacts=combineWorldToolContacts([pending.containmentContacts,pending.toolContacts]);
                            pending.combinedContactsSignature=JSON.stringify(contacts);input={...input,contacts};
                        }
                        if(wallConfiguration&&world.contactField) {
                            if(input.wall!==undefined&&input.wall!=='none')throw new RangeError('World source adapter owns wall geometry and laws; duplicate wall options are ambiguous');
                            const preparedWall=prepareCompositeJointWorldWall({...wallConfiguration,surfacePosePaths:input.worldWallSurfacePosePaths,inertia:input.inertia,world,state:pending.state,bindings:pending.bindings});
                            pending.wallProof=preparedWall.proof;
                            const {worldWallSurfacePosePaths,...rest}=input;input={...rest,wall:preparedWall.wall};
                        }
                        const borrowed=new Set([input.wall?.field].filter(Boolean));
                        // A prepared pose path is recursively frozen and its
                        // identity carries private preparation provenance.
                        // Own the surrounding list/records, but retain these
                        // validated immutable handles across numerical retries.
                        if(input.wall?.surfacePosePaths!==undefined){
                            if(!Array.isArray(input.wall.surfacePosePaths))throw new TypeError('Wall surface pose paths must be an explicit list');
                            for(const record of input.wall.surfacePosePaths){readCompositeJointSurfacePosePath(record?.path);borrowed.add(record.path);}
                        }
                        pending.options=ownPrepared(input,borrowed);
                    } catch(error){pending.error=error;}
                }
                if(pending.error)return resultFailure(dt,pending.error.code??'joint-world-preparation-error',pending.error);
                verifyBindings(world,pending.bindings);guardWorldConstraints(world,pending.state,pending.bindings,pending);
                const options={...pending.options,dt,elementBackend,workspace:pending.workspace,budget:{...pending.options.budget,...budgetOverride}};
                let result;
                if(cooperative) {
                    // Keep the same candidate, inputs and physical dt between
                    // browser frames. A terminal rejection is latched until an
                    // explicit budget retry or reset; do not restart it per frame.
                    if(pending.rejection)return pending.rejection;
                    pending.iterator??=iterateCompositeJointTimeStep(pending.state,options);
                    const sliceStarted=performance.now();
                    let resumed;
                    do {resumed=pending.iterator.next();}
                    while(!resumed.done&&workSliceMs>0&&performance.now()-sliceStarted<workSliceMs);
                    if(!resumed.done)return last={accepted:false,pending:true,dt,status:'computing',diagnostics:{progress:resumed.value}};
                    pending.iterator=null;result=resumed.value;
                } else result=advanceCompositeJointTimeStep(pending.state,options);
                if(!result.accepted){
                    const rejected=resultFailure(dt,result.status,result.error?{message:result.error,details:result.errorDetails}:null,result.diagnostics);
                    if(cooperative)pending.rejection=rejected;
                    return rejected;
                }
                guardWorldConstraints(world,pending.state,pending.bindings,pending);
                const staged=stagePublication(world,pending.bindings,pending.state,result.state,publicationOrigin);
                const wallSource=pending.wallProof;
                publish(staged);state=result.state;bindings=pending.bindings;workspace=pending.workspace;pending=null;stats.accepted++;stats.publications++;
                return last={accepted:true,dt,status:result.status,diagnostics:{...result.diagnostics,worldAdapter:{...stats,fullUnionGeometryPublished:true,physicsStatePrecision:'Float64',legacyBodyReimported:false,
                    wallSource:wallSource?{...wallSource}:null}},
                    balances:result.balances,boundaryForces:result.boundaryForces,spinReactions:result.spinReactions};
            } catch(error){pending?.iterator?.return();if(pending)pending.iterator=null;return resultFailure(dt,error.code??'joint-world-step-error',error);}
            finally {busy=false;}
        },
        reset(world){
            if(busy)throw new RangeError('Cannot reset an active Joint World adapter');
            if(owner&&world!==owner)throw new RangeError('Only the owning World can reset this adapter');
            pending?.iterator?.return();
            for(const b of bindings??[])delete b.body.jointStateView;
            publicationOrigin=null;owner=null;state=bindings=workspace=pending=initializationError=last=null;budgetOverride=null;
            for(const key of Object.keys(stats))stats[key]=0;
        },
        setBudget(value){
            if(busy)throw new RangeError('Cannot change the budget during a Joint attempt');
            if(pending?.iterator)throw new RangeError('Cannot replace a budget while a suspended Joint attempt is computing');
            if(pending)pending.rejection=null;
            if(value===null||value===undefined){budgetOverride=null;return;}
            if(Object.getPrototypeOf(value)!==Object.prototype)throw new TypeError('Budget must be a plain numeric record');
            for(const [key,n] of Object.entries(value))if(!['directions','evaluations','lineSearchTrials','linearSolves','contactQueries'].includes(key)||!Number.isInteger(n)||n<0)throw new RangeError('Only nonnegative integer numerical budgets can change during retry');
            budgetOverride={...value};
        },
        snapshot(){return state?ownState(state):null;},
        get diagnostics(){return {...stats,pending:pending!==null,stateTime:state?.time??null,stateStep:state?.step??null,nodeCount:state?.layout.nodeCount??null,lastStatus:last?.status??null,workspace:workspace?.diagnostics??null};}
    };
    return Object.freeze(adapter);
}
