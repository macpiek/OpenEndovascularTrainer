import {writeFileSync,mkdirSync,readFileSync,readdirSync} from 'node:fs';
import {createHash} from 'node:crypto';
import {execFileSync} from 'node:child_process';
import {fileURLToPath} from 'node:url';
import {Session} from 'node:inspector';
import {promisify} from 'node:util';
import {loadCoupledRuntimeAnatomy} from '../../tests/helpers/coupledRuntimeFixture.js';
import {createSharedAxisContacts} from '../../src/physics/kirchhoffSharedAxisContacts.js';
import {createSharedAxisVesselDiscovery} from '../../src/physics/kirchhoffSharedAxisVesselWitnesses.js';
import {createSharedAxisNative} from '../../src/physics/kirchhoffSharedAxisNative.js';
import {advanceSharedAxis,sampleSharedAxisPosition} from '../../src/physics/kirchhoffSharedAxisAppSystem.js';
import {PROJECTIVE_ROD_DEFAULTS} from '../../src/physics/kirchhoffSharedAxisProjective.js';
import {ADAPTIVE_SOLVE_OPTIONS} from '../../src/physics/kirchhoffSharedAxisAdaptiveMesh.js';

import {INTRODUCER_SHEATH_INNER_RADIUS_MM} from '../../src/toolDimensions.js';
import {DEFAULT_TOOL_PROFILES} from '../../src/physics/endovascularPhysicsWorld.js';
import {catheterNodeMass,CATHETER_PHYSICS_SPACING_MM} from '../../src/physics/catheterDiscretization.js';
import {captureSharedAxisReplay} from '../../tests/helpers/sharedAxisReplay.js';
const root=new URL('../../',import.meta.url),physicsRoot=new URL('src/physics/',root);
const sourceFiles=[...readdirSync(physicsRoot).filter(name=>/^kirchhoffSharedAxis.*\.js$/.test(name)).map(name=>'src/physics/'+name),
    'scripts/physics/profile-shared-axis-dynamic.mjs','tests/helpers/coupledRuntimeFixture.js',
    'src/physics/endovascularPhysicsWorld.js','src/physics/catheterDiscretization.js','src/physics/discreteKirchhoffRod.js',
    'src/physics/kirchhoffWallWitnessGeometry.js','src/physics/kirchhoffWallTriangleKernel.js','src/physics/kirchhoffCoulombBandLU.js','src/physics/kirchhoffLinearKernel.js','src/physics/kirchhoffLinearKernelBytes.js','src/physics/projectiveRodMath.js',
    'src/physics/applyKirchhoffMaterialProfile.js','src/physics/kirchhoffMaterialProfile.js','src/toolDimensions.js'].sort();
const sourceHashes=Object.fromEntries(sourceFiles.map(path=>[path,createHash('sha256').update(readFileSync(new URL(path,root))).digest('hex')]));
let gitHead=null;try{gitHead=execFileSync('git',['rev-parse','HEAD'],{cwd:fileURLToPath(root),encoding:'utf8'}).trim();}catch{/* Hashes still identify an exported checkout. */}
const dt=Number(process.env.SHARED_AXIS_DT??1/60),samples=[],options={forceTolerance:Number(process.env.SHARED_AXIS_FORCE_TOLERANCE??1e-6),lengthTolerance:1e-5,liveWallNormalLoad:process.env.SHARED_AXIS_LIVE_WALL_NORMAL==='1'};
const adaptiveOverrides={};
for(const [key,suffix] of Object.entries({contactMargin:'CONTACT_MARGIN',shapeTolerance:'SHAPE_TOLERANCE',maxArcLoss:'MAX_ARC_LOSS',maxSpacing:'MAX_SPACING'})) {
    const value=process.env['SHARED_AXIS_ADAPTIVE_'+suffix];
    if(value!==undefined) {
        const number=Number(value);
        if(!Number.isFinite(number))throw new RangeError('Invalid adaptive '+key);
        adaptiveOverrides[key]=number;
    }
}
const adaptiveMesh=process.env.SHARED_AXIS_ADAPTIVE_MESH==='1'?(Object.keys(adaptiveOverrides).length?adaptiveOverrides:true):false;
if(adaptiveMesh)Object.assign(options,ADAPTIVE_SOLVE_OPTIONS);
// Explicit experiment budgets override the adaptive defaults too.
if(process.env.SHARED_AXIS_FORCE_TOLERANCE!==undefined)options.forceTolerance=Number(process.env.SHARED_AXIS_FORCE_TOLERANCE);
if(process.env.SHARED_AXIS_LENGTH_TOLERANCE!==undefined)options.lengthTolerance=Number(process.env.SHARED_AXIS_LENGTH_TOLERANCE);
options.projectiveDynamics=process.env.SHARED_AXIS_PROJECTIVE==='1';
if(options.projectiveDynamics)options.projective={...PROJECTIVE_ROD_DEFAULTS,iterations:Number(process.env.SHARED_AXIS_PD_ITERATIONS??16)};
options.pruneInactiveWitnesses=process.env.SHARED_AXIS_PRUNE_WITNESSES==='1';
options.modifiedNewton=process.env.SHARED_AXIS_MODIFIED_NEWTON==='1';
options.coupledFrictionNewton=process.env.SHARED_AXIS_FAST_NEWTON==='1';
options.stagnationFallback=process.env.SHARED_AXIS_STAGNATION!=='0';
options.earlyContactPreflight=process.env.SHARED_AXIS_CONTACT_PREFLIGHT!=='0';
options.reuseTriangleKernel=process.env.SHARED_AXIS_TRIANGLE_KERNEL!=='0';
options.lightweightFriction=process.env.SHARED_AXIS_LIGHTWEIGHT_FRICTION!=='0';
options.reuseMaterialScratch=process.env.SHARED_AXIS_MATERIAL_SCRATCH!=='0';
options.reuseRowBuffers=process.env.SHARED_AXIS_ROW_BUFFERS!=='0';
options.cullInactiveContacts=process.env.SHARED_AXIS_INACTIVE_CONTACTS==='1';
options.reuseMatrixAssembly=process.env.SHARED_AXIS_MATRIX_ASSEMBLY!=='0';
options.reuseConstraintWork=process.env.SHARED_AXIS_CONSTRAINT_WORK!=='0';
options.wasmMaterial=process.env.SHARED_AXIS_WASM_MATERIAL!=='0';
options.projectionMode=process.env.SHARED_AXIS_PROJECTION==='0'?false:(process.env.SHARED_AXIS_PROJECTION??'reduced');
options.promoteTrialAssembly=process.env.SHARED_AXIS_PROMOTE_ASSEMBLY!=='0';
options.lazyTrialTangent=process.env.SHARED_AXIS_LAZY_TRIAL_TANGENT==='1';
options.reuseStructure=process.env.SHARED_AXIS_REUSE_STRUCTURE!=='0';
options.earlyLiveFallback=process.env.SHARED_AXIS_EARLY_FALLBACK!=='0';
const wireTarget=Number(process.env.SHARED_AXIS_WIRE_MM??309),catheterTarget=Number(process.env.SHARED_AXIS_CATHETER_MM??100);
if(![wireTarget,catheterTarget].every(v=>Number.isFinite(v)&&v>=0&&v<=1000))throw new RangeError('Profile insertion targets must be between 0 and 1000 mm');
// Match readTools in simulator.js: catheter body mass is rescaled for the
// current 5 mm material grid. Earlier reports omitted this override and used
// the solver fallback 1.4 instead of the actual UI mass 1.75.
const toolProfiles=[
    {id:'wire',type:'glidewire',mass:DEFAULT_TOOL_PROFILES.guidewire.mass,radius:DEFAULT_TOOL_PROFILES.guidewire.radius,
        wallStaticFriction:.006,wallKineticFriction:.002,shaftStiffness:Number(process.env.SHARED_AXIS_WIRE_SHAFT??9.6),tipStiffness:Number(process.env.SHARED_AXIS_WIRE_TIP??6.8)},
    {id:'catheter',type:process.env.SHARED_AXIS_CATHETER_TYPE??'berenstein',mass:catheterNodeMass(DEFAULT_TOOL_PROFILES.catheter.mass),radius:DEFAULT_TOOL_PROFILES.catheter.radius,
        wallStaticFriction:DEFAULT_TOOL_PROFILES.catheter.wallFriction,wallKineticFriction:DEFAULT_TOOL_PROFILES.catheter.wallFriction,shaftStiffness:Number(process.env.SHARED_AXIS_CATHETER_SHAFT??40.65),tipStiffness:Number(process.env.SHARED_AXIS_CATHETER_TIP??66.8)}
];
const metadata={reportVersion:2,createdAt:new Date().toISOString(),gitHead,sourceHashes,sourceTreeHash:createHash('sha256').update(JSON.stringify(sourceHashes)).digest('hex'),nodeVersion:process.version,
    timingScope:'Synchronous complete prepared physical step, including failed feed subdivisions; rendering excluded',
    historicalComparisonNote:'Reports without reportVersion:2 used implicit catheter mass 1.4; this report uses the UI grid-scaled mass (1.75 at 5 mm). Compare only matching modelParameters and source hashes.',
    modelParameters:{dt,wireTarget,catheterTarget,adaptiveMesh,spacing:CATHETER_PHYSICS_SPACING_MM,maxBendAngle:Math.PI/4,options,tools:toolProfiles,
        sheath:{innerRadius:INTRODUCER_SHEATH_INNER_RADIUS_MM,proximalExtension:40}}};
const anatomy=await loadCoupledRuntimeAnatomy();
const sheath={...anatomy.vessel.sheath,...metadata.modelParameters.sheath};
const contacts=createSharedAxisContacts({sheath,contactField:anatomy.field,localCoordinates:true});
metadata.indexedContacts=process.env.SHARED_AXIS_CONTACT_ID_INDEX!=='0';
metadata.reuseContactBuffers=process.env.SHARED_AXIS_CONTACT_BUFFERS!=='0';
contacts.wallSamples[0].contactOutputOwned=metadata.reuseContactBuffers;
if(!metadata.indexedContacts||!metadata.reuseContactBuffers)contacts.wallSamples[1]=createSharedAxisVesselDiscovery(anatomy.field,contacts.length,
    {indexedContacts:metadata.indexedContacts,reuseContactBuffers:metadata.reuseContactBuffers});
let s=createSharedAxisNative({...contacts,adaptiveMesh,
    spacing:metadata.modelParameters.spacing,maxBendAngle:metadata.modelParameters.maxBendAngle,tools:toolProfiles.map(t=>({...t,insertion:0}))});
let rotations={wire:0,catheter:0};
const output=process.argv[2]??`reports/shared-axis-dynamic-${metadata.createdAt.replace(/[:.]/g,'-')}`;mkdirSync(output,{recursive:true});let failed=false;
const feedOnly=process.env.SHARED_AXIS_FEED_ONLY==='1';
let cpuSession=null,cpuPost=null;
metadata.feedOnly=feedOnly;
const pairedProjection=process.env.SHARED_AXIS_COMPARE_PROJECTION==='1';
metadata.pairedProjection=pairedProjection;
const pairedStagnation=process.env.SHARED_AXIS_COMPARE_STAGNATION==='1';
metadata.pairedStagnation=pairedStagnation;
const pairedMaterial=process.env.SHARED_AXIS_COMPARE_MATERIAL==='1';
metadata.pairedMaterial=pairedMaterial;
const pairedConstraintWork=process.env.SHARED_AXIS_COMPARE_CONSTRAINT_WORK==='1';metadata.pairedConstraintWork=pairedConstraintWork;
const pairedMatrixAssembly=process.env.SHARED_AXIS_COMPARE_MATRIX_ASSEMBLY==='1';metadata.pairedMatrixAssembly=pairedMatrixAssembly;
const pairedRowBuffers=process.env.SHARED_AXIS_COMPARE_ROW_BUFFERS==='1';metadata.pairedRowBuffers=pairedRowBuffers;
const pairedPrunedWitnesses=process.env.SHARED_AXIS_COMPARE_PRUNED_WITNESSES==='1';metadata.pairedPrunedWitnesses=pairedPrunedWitnesses;
const pairedContactPreflight=process.env.SHARED_AXIS_COMPARE_CONTACT_PREFLIGHT==='1';metadata.pairedContactPreflight=pairedContactPreflight;
const pairedTriangleKernel=process.env.SHARED_AXIS_COMPARE_TRIANGLE_KERNEL==='1';metadata.pairedTriangleKernel=pairedTriangleKernel;
const pairedLightweightFriction=process.env.SHARED_AXIS_COMPARE_LIGHTWEIGHT_FRICTION==='1';metadata.pairedLightweightFriction=pairedLightweightFriction;
const pairedMaterialScratch=process.env.SHARED_AXIS_COMPARE_MATERIAL_SCRATCH==='1';metadata.pairedMaterialScratch=pairedMaterialScratch;
const pairedInactiveContacts=process.env.SHARED_AXIS_COMPARE_INACTIVE_CONTACTS==='1';metadata.pairedInactiveContacts=pairedInactiveContacts;
const pairedContactMesh=pairedPrunedWitnesses&&process.env.SHARED_AXIS_COMPARE_CONTACT_MESH==='1';metadata.pairedContactMesh=pairedContactMesh;
const reversePairOrder=process.env.SHARED_AXIS_PAIR_REVERSE_ORDER==='1';
metadata.reversePairOrder=reversePairOrder;
if([pairedProjection,pairedStagnation,pairedMaterial,pairedConstraintWork,pairedMatrixAssembly,pairedRowBuffers,pairedInactiveContacts,pairedMaterialScratch,pairedLightweightFriction,pairedTriangleKernel,pairedContactPreflight,pairedPrunedWitnesses].filter(Boolean).length>1)throw new Error('Compare one optimization at a time');
const captureCatheterMm=Number(process.env.SHARED_AXIS_CAPTURE_CATHETER_MM??NaN);
function step(phase,feeds,spins={}) {
    const start=performance.now();
    const tools=s.materials.map(m=>({...m.spec,insertion:feeds[m.spec.id]??m.spec.insertion,rotation:rotations[m.spec.id]+(spins[m.spec.id]??0)}));
    const capture=phase==='catheter'&&s.materials.find(m=>m.spec.id==='catheter').spec.insertion<captureCatheterMm&&feeds.catheter>=captureCatheterMm;
    if(capture)writeFileSync(`${output}/captured-incoming.json`,JSON.stringify({...captureSharedAxisReplay(s,sheath),stepRequest:{dt,tools,rotations,options}}));
    const trace=[],stateIds=new WeakMap();let stateCount=0;
    const measure=({state,iteration,base})=>{
        if(!stateIds.has(state))stateIds.set(state,++stateCount);
        return {state:stateIds.get(state),iteration,live:state.wallFrictionStep?.liveNormalLoad,force:base.force,torque:base.torque,constraint:base.constraint,energy:base.energy,
            walls:base.rows.filter(r=>r.kind==='wall'&&r.multiplier>0).map(r=>r.id??`${r.sample}/${r.edge}`)};
    };
    const traced=capture?{...options,observeIteration:e=>trace.push({kind:'iteration',...measure(e)}),observeTrial:e=>{
        if(e.kind==='direction')trace.push({kind:e.kind,...measure(e),method:e.method,converged:e.direction.converged,failure:e.direction.failure,factorizations:e.direction.factorizations});
        else if(e.kind==='trial')trace.push({kind:e.kind,iteration:e.iteration,method:e.method,trial:e.trial,scale:e.scale,accept:e.accept,force:e.candidate.force,torque:e.candidate.torque,constraint:e.candidate.constraint});
    }}:options;
    let next,paired=null;
    if(pairedProjection||pairedStagnation||pairedMaterial||pairedConstraintWork||pairedMatrixAssembly||pairedRowBuffers||pairedInactiveContacts||pairedMaterialScratch||pairedLightweightFriction||pairedTriangleKernel||pairedContactPreflight||pairedPrunedWitnesses) {
        // Same incoming physical state, alternating order at every step to
        // reduce slow thermal/load drift. Serialization is outside both timers.
        const outputs=[];
        const referenceFirst=(samples.length%2===0)!==reversePairOrder;
        for(const mode of (referenceFirst?[0,1]:[1,0])) {
            const variant=pairedPrunedWitnesses?{pruneInactiveWitnesses:Boolean(mode)}:pairedContactPreflight?{earlyContactPreflight:Boolean(mode)}:pairedTriangleKernel?{reuseTriangleKernel:Boolean(mode)}:pairedLightweightFriction?{lightweightFriction:Boolean(mode)}:pairedMaterialScratch?{reuseMaterialScratch:Boolean(mode)}:pairedInactiveContacts?{cullInactiveContacts:Boolean(mode)}:pairedRowBuffers?{reuseRowBuffers:Boolean(mode)}:pairedMatrixAssembly?{reuseMatrixAssembly:Boolean(mode)}:pairedConstraintWork?{reuseConstraintWork:Boolean(mode)}:pairedMaterial?{wasmMaterial:Boolean(mode)}:pairedStagnation?{stagnationFallback:Boolean(mode)}:{projectionMode:mode?'reduced':false};
            const begin=performance.now(),iterator=advanceSharedAxis(pairedContactMesh&&mode?{...s,adaptiveMesh:{...(typeof s.adaptiveMesh==='object'?s.adaptiveMesh:{}),contactMargin:0}}:s,rotations,dt,tools,{...traced,...variant});
            let item;do{item=iterator.next();}while(!item.done);
            outputs[mode]={value:item.value,ms:performance.now()-begin};
        }
        const physical=output=>JSON.stringify(output.value.state?captureSharedAxisReplay(output.value.state,sheath):null);
        if(!pairedPrunedWitnesses&&physical(outputs[0])!==physical(outputs[1])) {
            writeFileSync(`${output}/pair-mismatch.json`,JSON.stringify({incoming:{...captureSharedAxisReplay(s,sheath),stepRequest:{dt,tools,rotations,options}},outputs:outputs.map(o=>({result:o.value.result,state:o.value.state?captureSharedAxisReplay(o.value.state,sheath):null}))}));
            throw new Error('Optimization pair changed the complete physical state at '+phase+' '+tools[1].insertion);
        }
        paired={referenceMs:outputs[0].ms,optimizedMs:outputs[1].ms,
            referenceProjectionMs:outputs[0].value.result.timings.projectionMs,optimizedProjectionMs:outputs[1].value.result.timings.projectionMs,
            exactState:pairedPrunedWitnesses?physical(outputs[0])===physical(outputs[1]):true,referenceFirst};
        if(pairedPrunedWitnesses) {
            const a=outputs[0].value.state,b=outputs[1].value.state;
            if(!a||!b)throw new Error('Pruned contact comparison failed to accept both steps');
            const knots=[...new Set([...a.coordinates,...b.coordinates])];
            paired.maxShapeDeviationMm=Math.max(...knots.map(x=>{
                const p=sampleSharedAxisPosition(a,x),q=sampleSharedAxisPosition(b,x);
                return Math.hypot(...p.map((v,k)=>v-q[k]));
            }));
            paired.referenceNodes=a.coordinates.length;paired.optimizedNodes=b.coordinates.length;
            paired.referenceDefinitions=a.definitions.length;paired.optimizedDefinitions=b.definitions.length;
        }
        if(pairedStagnation||pairedMaterial||pairedConstraintWork||pairedMatrixAssembly||pairedRowBuffers||pairedInactiveContacts||pairedMaterialScratch||pairedLightweightFriction||pairedTriangleKernel||pairedContactPreflight||pairedPrunedWitnesses)paired.optimizedResult=outputs[1].value.result;
        next={value:outputs[0].value};
    } else {
        const iterator=advanceSharedAxis(s,rotations,dt,tools,traced);do{next=iterator.next();}while(!next.done);
    }
    const {state:candidate,result}=next.value;
    samples.push({phase,wire:tools[0].insertion,catheter:tools[1].insertion,...result,totalMs:paired?.referenceMs??performance.now()-start,
        ...(candidate?{mechanicalNodes:candidate.coordinates.length,mechanicalDofs:candidate.layout.dofCount,
            toolTips:candidate.materials.map(m=>({id:m.spec.id,position:sampleTip(candidate,m)})),
            ...(process.env.SHARED_AXIS_CAPTURE_SHAPES==='1'?{shape:{coordinates:candidate.coordinates,positions:candidate.positions.map(p=>p.map((v,k)=>v+candidate.origin[k]))}}:{})}:{}),...(paired?{paired}:{})});
    if(capture){writeFileSync(`${output}/captured-trace.json`,JSON.stringify({result,trace}));if(candidate)writeFileSync(`${output}/captured-terminal.json`,JSON.stringify(captureSharedAxisReplay(candidate,sheath)));}
    if(!candidate){failed=true;writeFileSync(`${output}/incoming.json`,JSON.stringify({...captureSharedAxisReplay(s,sheath),stepRequest:{dt,tools,rotations,options},profileMetadata:metadata}));console.log(samples.at(-1));return false;}
    rotations=next.value.rotations;s=candidate;return true;
}
try {
    if(step('initial',{})) {
        for(const [id,target,rate] of [['wire',wireTarget,44],['catheter',catheterTarget,52]]) {
            if(id==='catheter'&&process.env.SHARED_AXIS_CPU_PROFILE==='1') {
                cpuSession=new Session();cpuSession.connect();cpuPost=promisify(cpuSession.post.bind(cpuSession));
                await cpuPost('Profiler.enable');await cpuPost('Profiler.setSamplingInterval',{interval:1000});
                await cpuPost('Profiler.start');
            }
            let progress=s.materials.find(t=>t.spec.id===id).spec.insertion;
            while(progress<target){progress=Math.min(target,progress+rate*dt);if(!step(id,{[id]:progress}))break;if(Math.floor(progress/25)!==Math.floor((progress-rate*dt)/25))console.log(id,progress.toFixed(2),samples.at(-1).totalMs.toFixed(1));}
            if(id==='catheter'&&cpuSession) {
                const {profile}=await cpuPost('Profiler.stop');cpuSession.disconnect();cpuSession=null;
                writeFileSync(`${output}/catheter.cpuprofile`,JSON.stringify(profile));
            }
            if(failed)break;
        }
        if(!failed&&!feedOnly)for(let i=0;i<Math.round(1/dt);i++)if(!step('simultaneous',{wire:wireTarget+44*dt*(i+1),catheter:catheterTarget+52*dt*(i+1)}))break;
        if(!failed&&!feedOnly)for(let i=0;i<Math.round(.5/dt);i++)if(!step('rotation',{}, {wire:.002*dt*120,catheter:-.003*dt*120}))break;
        if(!failed&&!feedOnly)for(let i=0;i<Math.round(1/dt);i++)if(!step('withdrawal',{wire:wireTarget+44-32*dt*(i+1),catheter:catheterTarget+52-32*dt*(i+1)}))break;
        if(!failed&&process.env.SHARED_AXIS_WIRE_WITHDRAW_TO_MM!==undefined) {
            const target=Number(process.env.SHARED_AXIS_WIRE_WITHDRAW_TO_MM);
            if(!Number.isFinite(target)||target<0||target>wireTarget)throw new RangeError('Invalid wire withdrawal target');
            metadata.wireWithdrawToMm=target;
            let progress=s.materials.find(m=>m.spec.id==='wire').spec.insertion;
            while(progress>target){progress=Math.max(target,progress-32*dt);if(!step('wire-withdrawal',{wire:progress}))break;
                if(Math.floor(progress/5)!==Math.floor((progress+32*dt)/5))console.log('wire-withdrawal',progress.toFixed(2),samples.at(-1).totalMs.toFixed(1));}
        }
    }
}finally {
    writeFileSync(`${output}/terminal.json`,JSON.stringify(captureSharedAxisReplay(s,sheath)));
    writeFileSync(`${output}/profile.json`,JSON.stringify({...metadata,dt,failed,samples},null,2));anatomy.dispose();
}
process.exitCode=failed?1:0;

function sampleTip(state,material) {
    const e=material.last-1,t=material.endFraction;
    return state.positions[e].map((v,k)=>state.origin[k]+(1-t)*v+t*state.positions[e+1][k]);
}
