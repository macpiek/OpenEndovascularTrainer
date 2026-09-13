import {writeFileSync,mkdirSync,readFileSync,readdirSync} from 'node:fs';
import {createHash} from 'node:crypto';
import {execFileSync} from 'node:child_process';
import {fileURLToPath} from 'node:url';
import {loadCoupledRuntimeAnatomy} from '../../tests/helpers/coupledRuntimeFixture.js';
import {createSharedAxisContacts} from '../../src/physics/kirchhoffSharedAxisContacts.js';
import {createSharedAxisNative} from '../../src/physics/kirchhoffSharedAxisNative.js';
import {advanceSharedAxis} from '../../src/physics/kirchhoffSharedAxisAppSystem.js';

import {INTRODUCER_SHEATH_INNER_RADIUS_MM} from '../../src/toolDimensions.js';
import {DEFAULT_TOOL_PROFILES} from '../../src/physics/endovascularPhysicsWorld.js';
import {catheterNodeMass,CATHETER_PHYSICS_SPACING_MM} from '../../src/physics/catheterDiscretization.js';
import {captureSharedAxisReplay} from '../../tests/helpers/sharedAxisReplay.js';
const root=new URL('../../',import.meta.url),physicsRoot=new URL('src/physics/',root);
const sourceFiles=[...readdirSync(physicsRoot).filter(name=>/^kirchhoffSharedAxis.*\.js$/.test(name)).map(name=>'src/physics/'+name),
    'scripts/physics/profile-shared-axis-dynamic.mjs','tests/helpers/coupledRuntimeFixture.js',
    'src/physics/endovascularPhysicsWorld.js','src/physics/catheterDiscretization.js','src/physics/discreteKirchhoffRod.js',
    'src/physics/kirchhoffCoulombBandLU.js','src/physics/kirchhoffLinearKernel.js','src/physics/kirchhoffLinearKernelBytes.js',
    'src/physics/applyKirchhoffMaterialProfile.js','src/physics/kirchhoffMaterialProfile.js','src/toolDimensions.js'].sort();
const sourceHashes=Object.fromEntries(sourceFiles.map(path=>[path,createHash('sha256').update(readFileSync(new URL(path,root))).digest('hex')]));
let gitHead=null;try{gitHead=execFileSync('git',['rev-parse','HEAD'],{cwd:fileURLToPath(root),encoding:'utf8'}).trim();}catch{/* Hashes still identify an exported checkout. */}
const dt=Number(process.env.SHARED_AXIS_DT??1/60),samples=[],options={forceTolerance:Number(process.env.SHARED_AXIS_FORCE_TOLERANCE??1e-6),lengthTolerance:1e-5,liveWallNormalLoad:process.env.SHARED_AXIS_LIVE_WALL_NORMAL==='1'};
const wireTarget=Number(process.env.SHARED_AXIS_WIRE_MM??309),catheterTarget=Number(process.env.SHARED_AXIS_CATHETER_MM??100);
if(![wireTarget,catheterTarget].every(v=>Number.isFinite(v)&&v>=0&&v<=900))throw new RangeError('Profile insertion targets must be between 0 and 900 mm');
// Match readTools in simulator.js: catheter body mass is rescaled for the
// current 5 mm material grid. Earlier reports omitted this override and used
// the solver fallback 1.4 instead of the actual UI mass 1.75.
const toolProfiles=[
    {id:'wire',type:'glidewire',mass:DEFAULT_TOOL_PROFILES.guidewire.mass,radius:DEFAULT_TOOL_PROFILES.guidewire.radius,
        wallStaticFriction:.006,wallKineticFriction:.002,shaftStiffness:39,tipStiffness:30.7},
    {id:'catheter',type:process.env.SHARED_AXIS_CATHETER_TYPE??'berenstein',mass:catheterNodeMass(DEFAULT_TOOL_PROFILES.catheter.mass),radius:DEFAULT_TOOL_PROFILES.catheter.radius,
        wallStaticFriction:DEFAULT_TOOL_PROFILES.catheter.wallFriction,wallKineticFriction:DEFAULT_TOOL_PROFILES.catheter.wallFriction,shaftStiffness:58.1,tipStiffness:87}
];
const metadata={reportVersion:2,createdAt:new Date().toISOString(),gitHead,sourceHashes,sourceTreeHash:createHash('sha256').update(JSON.stringify(sourceHashes)).digest('hex'),nodeVersion:process.version,
    timingScope:'Synchronous complete prepared physical step, including failed feed subdivisions; rendering excluded',
    historicalComparisonNote:'Reports without reportVersion:2 used implicit catheter mass 1.4; this report uses the UI grid-scaled mass (1.75 at 5 mm). Compare only matching modelParameters and source hashes.',
    modelParameters:{dt,wireTarget,catheterTarget,spacing:CATHETER_PHYSICS_SPACING_MM,maxBendAngle:Math.PI/4,options,tools:toolProfiles,
        sheath:{innerRadius:INTRODUCER_SHEATH_INNER_RADIUS_MM,proximalExtension:40}}};
const anatomy=await loadCoupledRuntimeAnatomy();
const sheath={...anatomy.vessel.sheath,...metadata.modelParameters.sheath};
let s=createSharedAxisNative({...createSharedAxisContacts({sheath,contactField:anatomy.field,localCoordinates:true}),
    spacing:metadata.modelParameters.spacing,maxBendAngle:metadata.modelParameters.maxBendAngle,tools:toolProfiles.map(t=>({...t,insertion:0}))});
let rotations={wire:0,catheter:0};
const output=process.argv[2]??`reports/shared-axis-dynamic-${metadata.createdAt.replace(/[:.]/g,'-')}`;mkdirSync(output,{recursive:true});let failed=false;
function step(phase,feeds,spins={}) {
    const start=performance.now();
    const tools=s.materials.map(m=>({...m.spec,insertion:feeds[m.spec.id]??m.spec.insertion,rotation:rotations[m.spec.id]+(spins[m.spec.id]??0)}));
    const iterator=advanceSharedAxis(s,rotations,dt,tools,options);let next;do{next=iterator.next();}while(!next.done);
    const {state:candidate,result}=next.value;
    samples.push({phase,wire:tools[0].insertion,catheter:tools[1].insertion,...result,totalMs:performance.now()-start});
    if(!candidate){failed=true;writeFileSync(`${output}/incoming.json`,JSON.stringify({...captureSharedAxisReplay(s,sheath),stepRequest:{dt,tools,rotations,options},profileMetadata:metadata}));console.log(samples.at(-1));return false;}
    rotations=next.value.rotations;s=candidate;return true;
}
try {
    if(step('initial',{})) {
        for(const [id,target,rate] of [['wire',wireTarget,44],['catheter',catheterTarget,52]]) {
            let progress=s.materials.find(t=>t.spec.id===id).spec.insertion;
            while(progress<target){progress=Math.min(target,progress+rate*dt);if(!step(id,{[id]:progress}))break;if(Math.floor(progress/25)!==Math.floor((progress-rate*dt)/25))console.log(id,progress.toFixed(2),samples.at(-1).totalMs.toFixed(1));}
            if(failed)break;
        }
        if(!failed)for(let i=0;i<Math.round(1/dt);i++)if(!step('simultaneous',{wire:wireTarget+44*dt*(i+1),catheter:catheterTarget+52*dt*(i+1)}))break;
        if(!failed)for(let i=0;i<Math.round(.5/dt);i++)if(!step('rotation',{}, {wire:.002*dt*120,catheter:-.003*dt*120}))break;
        if(!failed)for(let i=0;i<Math.round(1/dt);i++)if(!step('withdrawal',{wire:wireTarget+44-32*dt*(i+1),catheter:catheterTarget+52-32*dt*(i+1)}))break;
    }
}finally {
    writeFileSync(`${output}/terminal.json`,JSON.stringify(captureSharedAxisReplay(s,sheath)));
    writeFileSync(`${output}/profile.json`,JSON.stringify({...metadata,dt,failed,samples},null,2));anatomy.dispose();
}
process.exitCode=failed?1:0;
