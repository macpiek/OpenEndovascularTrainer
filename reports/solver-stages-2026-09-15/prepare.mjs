import {cpSync,mkdirSync,readFileSync,writeFileSync,symlinkSync,readdirSync,realpathSync} from 'node:fs';
import {resolve,join} from 'node:path';
import {fileURLToPath} from 'node:url';
import {createHash} from 'node:crypto';
const root=fileURLToPath(new URL('../../',import.meta.url)),report=fileURLToPath(new URL('./',import.meta.url));
const target=process.argv[2]??'/tmp/oet-solver-stages-20260915';mkdirSync(target,{recursive:false});
for(const name of ['src','scripts'])cpSync(join(root,name),join(target,name),{recursive:true});
for(const name of ['index.html','shared-axis-lab.html','package.json'])cpSync(join(root,name),join(target,name));
for(const name of ['node_modules','res','tests'])symlinkSync(join(root,name),join(target,name),'dir');
cpSync(join(report,'stage-capture.mjs'),join(target,'src/physics/profileCapture.js'));
const manifest=[];
function instrument(file,entries) {
    const path=join(target,'src/physics',file);let source=readFileSync(path,'utf8');
    const before=createHash('sha256').update(source).digest('hex');
    for(const [fn,stage,kind='sync'] of entries) {
        const pattern=new RegExp(`(export )?function(\\*)? ${fn}\\(`);
        const match=source.match(pattern);if(!match)throw new Error('Missing function '+fn);
        source=source.replace(pattern,`function${match[2]??''} __profile_${fn}(`);
        const prefix=match[1]??'',gen=match[2]??'';
        source+=`\n${prefix}function${gen} ${fn}(...args){${gen?'return yield* ': 'return '}${kind==='root'?`profileStep(__profile_${fn}(...args))`:gen?`profileGenerator('${stage}',__profile_${fn}(...args))`:`profileCall('${stage}',()=>__profile_${fn}(...args))`};}\n`;
    }
    source=`import {profileCall,profileGenerator,profileStep} from './profileCapture.js';\n`+source;
    writeFileSync(path,source);manifest.push({file,before,after:createHash('sha256').update(source).digest('hex'),entries});
}
instrument('kirchhoffSharedAxisAppSystem.js',[['advanceSharedAxis','step','root']]);
instrument('kirchhoffSharedAxisNative.js',[
    ['assembleSharedAxisNative','assembly'],['feedSharedAxisNative','feed-remesh'],['captureSharedAxisNative','snapshot'],
    ['restoreSharedAxisNative','restore'],['applySharedAxisNativeIncrement','apply'],['correctTrialConstraints','projection']]);
instrument('kirchhoffSharedAxisMaterialTangent.js',[['assembleSharedAxisMaterialTangent','material']]);
instrument('kirchhoffSharedAxisConstraintRows.js',[['assembleSharedAxisConstraintRows','constraint-rows']]);
instrument('kirchhoffSharedAxisDynamics.js',[['assembleSharedAxisInertia','inertia'],['prepareSharedAxisDynamicStep','prepare-dynamics'],['completeSharedAxisDynamicStep','complete-dynamics']]);
instrument('kirchhoffSharedAxisWallFriction.js',[['assembleSharedAxisWallFriction','friction-assembly'],['refreshSharedAxisWallFriction','friction-certify'],['prepareSharedAxisWallFriction','friction-prepare'],['commitSharedAxisWallFriction','friction-commit']]);
instrument('kirchhoffSharedAxisActiveBasis.js',[['prepareSharedAxisActiveBasis','active-basis']]);
instrument('kirchhoffSharedAxisLinear.js',[['iterateSharedAxisLinear','linear'],['solveSharedAxisLinearOnce','linear-build-certify']]);
instrument('kirchhoffSharedAxisDiagnostics.js',[['measureSharedAxisQuality','quality']]);
instrument('kirchhoffWallWitnessGeometry.js',[['evaluateKirchhoffWallWitnessGeometry','wall-geometry']]);
// Separate raw WASM LU from matrix packing/scaling and its certificate.
{
    const path=join(target,'src/physics/kirchhoffCoulombBandLU.js');let s=readFileSync(path,'utf8');
    s=`import {profileCall} from './profileCapture.js';\n`+s;
    s=s.replace('solve(J, F, scales, shift, direction) {','solve(J, F, scales, shift, direction) { return profileCall(\'LU-pack-certify\',()=>{');
    s=s.replace('return direction.every(Number.isFinite);','return direction.every(Number.isFinite); });');
    s=s.replace('kernel.solveGeneralBandLU(factor.byteOffset, rhs.byteOffset, right.byteOffset, count, kl, ku)',"profileCall('LU-kernel',()=>kernel.solveGeneralBandLU(factor.byteOffset, rhs.byteOffset, right.byteOffset, count, kl, ku))");
    writeFileSync(path,s);
}
{
    const path=join(target,'src/simulator.js');let s=readFileSync(path,'utf8');
    s=s.replace('cpuMs:simulationPendingStepCpuMs,providerMs:last?.cpuMs,timings:last?.timings,','cpuMs:simulationPendingStepCpuMs,providerMs:last?.cpuMs,timings:last?.timings,stageProfile:last?.stageProfile,');
    s=s.replace('wire60Profile:browserBenchmarkScenario.mode===WIRE60_BENCHMARK_MODE?', 'profileTimeOrigin:browserBenchmarkScenario.startedAt,wire60Profile:browserBenchmarkScenario.mode===WIRE60_BENCHMARK_MODE?');
    // A diagnostic-only UI button saves the visible benchmark report locally.
    s+=`\nconst saveProfile=document.createElement('button');saveProfile.textContent='Zapisz profil lokalnie';saveProfile.id='saveStageProfile';
    saveProfile.style.cssText='position:fixed;top:60px;left:15px;z-index:10000';document.body.append(saveProfile);
    saveProfile.onclick=async()=>{const value=document.getElementById('browserBenchmarkReport').value;
        const res=await fetch('/__stage-report/instrumented',{method:'POST',headers:{'Content-Type':'application/json'},body:value});saveProfile.textContent=res.ok?'Profil zapisany':await res.text();};\n`;
    writeFileSync(path,s);
}
writeFileSync(join(target,'import-profile.html'),`<!doctype html><html lang="pl"><meta charset="utf-8"><title>Zapis profilu lokalnego</title>
<label>Raport z pomiaru<textarea id="report"></textarea></label><button id="save">Zapisz raport bazowy</button><p id="status"></p>
<script>document.getElementById('save').onclick=async()=>{const r=await fetch('/__stage-report/baseline',{method:'POST',headers:{'Content-Type':'application/json'},body:document.getElementById('report').value});document.getElementById('status').textContent=r.ok?'Zapisano raport':await r.text();};</script></html>`);
writeFileSync(join(target,'vite.config.js'),`import {defineConfig} from 'vite';import {writeFile} from 'node:fs/promises';
export default defineConfig({server:{host:'127.0.0.1',port:5174,strictPort:true,fs:{allow:[${JSON.stringify(root)},${JSON.stringify(realpathSync(target))}]}},plugins:[{name:'stage-report',configureServer(server){
server.middlewares.use('/__stage-report',async(req,res)=>{if(req.method!=='POST'||!['127.0.0.1','::1','::ffff:127.0.0.1'].includes(req.socket.remoteAddress)||req.headers.origin!=='http://'+req.headers.host||!['/baseline','/instrumented'].includes(req.url)){res.statusCode=403;res.end();return;}
try{const chunks=[];let n=0;for await(const c of req){n+=c.length;if(n>16*1024*1024)throw Error('Too large');chunks.push(c);}const text=Buffer.concat(chunks).toString('utf8'),value=JSON.parse(text);if(value.mode!=='shared-axis'||!Array.isArray(value.wire60Profile?.steps))throw Error('Not a solver benchmark');await writeFile(${JSON.stringify(report)}+req.url.slice(1)+'-browser.json',text);res.end('saved');}catch(e){res.statusCode=400;res.end(e.message);}});}}]});`);
writeFileSync(join(report,'instrumentation-manifest.json'),JSON.stringify({sourceRoot:root,target,createdAt:new Date().toISOString(),manifest},null,2));
console.log(target);
