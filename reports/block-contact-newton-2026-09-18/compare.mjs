import {readFileSync,writeFileSync,mkdirSync,readdirSync} from 'node:fs';import {gunzipSync} from 'node:zlib';import {createHash} from 'node:crypto';import {fileURLToPath} from 'node:url';
const sourceHashes=Object.fromEntries(readdirSync(new URL('../../src/physics/',import.meta.url)).filter(n=>n.startsWith('kirchhoffSharedAxis')).sort().map(n=>[n,createHash('sha256').update(readFileSync(new URL('../../src/physics/'+n,import.meta.url))).digest('hex')]));
import {loadCoupledRuntimeAnatomy} from '../../tests/helpers/coupledRuntimeFixture.js';
import {createSharedAxisContacts} from '../../src/physics/kirchhoffSharedAxisContacts.js';
import {createSharedAxisNative} from '../../src/physics/kirchhoffSharedAxisNative.js';
import {advanceSharedAxis,sampleSharedAxisPosition} from '../../src/physics/kirchhoffSharedAxisAppSystem.js';
import {captureSharedAxisReplay} from '../../tests/helpers/sharedAxisReplay.js';
const repo=fileURLToPath(new URL('../../',import.meta.url)), prior=JSON.parse(gunzipSync(readFileSync(repo+'/reports/current-settings-audit-2026-09-18/current-15mm/profile.json.gz')));
const p=structuredClone(prior.modelParameters);p.adaptiveMesh={...p.adaptiveMesh,shapeTolerance:.15,contactMargin:1,maxArcLoss:.002,maxSpacing:20};
const anatomy=await loadCoupledRuntimeAnatomy(),sheath={...anatomy.vessel.sheath,...p.sheath},contacts=createSharedAxisContacts({sheath,contactField:anatomy.field,localCoordinates:true});
const options={...p.options,coupledFrictionNewton:true,adaptiveNewtonStrategy:false,reuseNewtonFallback:false}, variant=JSON.parse(process.env.VARIANT??'{"adaptiveNewtonStrategy":true,"reuseNewtonFallback":true}'),independent=process.env.INDEPENDENT==='1';
const output=process.argv[2];mkdirSync(output,{recursive:true});
const create=()=>createSharedAxisNative({...contacts,adaptiveMesh:p.adaptiveMesh,spacing:p.spacing,maxBendAngle:p.maxBendAngle,tools:p.tools.map(t=>({...t,insertion:0}))});
const states=[create(),create()],rotations=[{wire:0,catheter:0},{wire:0,catheter:0}],samples=[];
const wireTarget=Number(process.env.WIRE_MM??600),catheterTarget=Number(process.env.CATHETER_MM??600);let failed=false;
function step(phase,feeds={},spins={}) {
 const out=[],referenceFirst=samples.length%2===0;
 for(const mode of referenceFirst?[0,1]:[1,0]) {
  const s=states[independent?mode:0],rotation=rotations[independent?mode:0],tools=s.materials.map(m=>({...m.spec,insertion:feeds[m.spec.id]??m.spec.insertion,rotation:rotation[m.spec.id]+(spins[m.spec.id]??0)}));
  const start=performance.now(),it=advanceSharedAxis(s,rotation,p.dt,tools,{...options,...(mode?variant:{})});let n;do{n=it.next();}while(!n.done);out[mode]={ms:performance.now()-start,...n.value};
 }
 if(out[0].ms>100&&samples.length%2===0){const replay=captureSharedAxisReplay(states[0],sheath);replay.stepRequest={rotations:rotations[0],dt:p.dt,tools:states[0].materials.map(m=>({...m.spec,insertion:feeds[m.spec.id]??m.spec.insertion,rotation:rotations[0][m.spec.id]+(spins[m.spec.id]??0)})),options};writeFileSync(output+'/hot-'+samples.length+'.json',JSON.stringify(replay));}
 const a=out[0].state,b=out[1].state,measure=x=>({ms:x.ms,...x.result,...(x.state?{nodes:x.state.coordinates.length,definitions:x.state.definitions.length}:{})});
 const diff=a&&b?Math.max(...[...new Set([...a.coordinates,...b.coordinates])].map(x=>{const u=sampleSharedAxisPosition(a,x),v=sampleSharedAxisPosition(b,x);return Math.hypot(...u.map((c,k)=>c-v[k]));})):null;
 const row={phase,referenceFirst,feeds,reference:measure(out[0]),optimized:measure(out[1]),maxShapeDeviationMm:diff};samples.push(row);
 if(!a||!b){failed=true;writeFileSync(output+'/failed-incoming.json',JSON.stringify(captureSharedAxisReplay(states[0],sheath)));console.log('FAILED',JSON.stringify(row));return false;}
 states[0]=a;rotations[0]=out[0].rotations;states[1]=b;rotations[1]=out[1].rotations;
 if(samples.length%100===0)console.log(phase,samples.length,feeds,Math.round(out[0].ms),Math.round(out[1].ms),diff);
 return true;
}
try{
 if(step('initial')) {
 for(const [id,target,rate]of [['wire',wireTarget,44],['catheter',catheterTarget,52]]){
 let progress=0;while(progress<target){progress=Math.min(target,progress+rate*p.dt);if(!step(id,{[id]:progress}))break;}if(failed)break;
 }
 if(!failed&&process.env.FEED_ONLY!=='1')for(let i=0;i<60;i++)if(!step('simultaneous',{wire:wireTarget+44*p.dt*(i+1),catheter:catheterTarget+52*p.dt*(i+1)}))break;
 if(!failed&&process.env.FEED_ONLY!=='1')for(let i=0;i<30;i++)if(!step('rotation',{}, {wire:.002*p.dt*120,catheter:-.003*p.dt*120}))break;
 if(!failed&&process.env.FEED_ONLY!=='1')for(let i=0;i<60;i++)if(!step('withdrawal',{wire:wireTarget+44-32*p.dt*(i+1),catheter:catheterTarget+52-32*p.dt*(i+1)}))break;
 }
}finally{
 writeFileSync(output+'/profile.json',JSON.stringify({sourceHashes,createdAt:new Date().toISOString(),parameters:p,options,variant,independent,failed,samples},null,2));
 for(const mode of [0,1])writeFileSync(output+`/terminal-${mode}.json`,JSON.stringify(captureSharedAxisReplay(states[mode],sheath)));
 anatomy.dispose();
}
process.exitCode=failed?1:0;
