// Arithmetic microbenchmark only. Whole crossover/browser measurements decide
// performance acceptance; this excludes contacts, remeshing and rendering.
import fs from 'node:fs';
import {performance} from 'node:perf_hooks';
import {createHash} from 'node:crypto';
import {Quaternion,Vector3} from 'three';
import {projectRotation,quaternionColumns} from '../src/physics/projectiveRodMath.js';
import {projectRotationPolar} from '../src/physics/projectiveRotationPolar.js';
import {createProjectRotationScratch} from '../src/physics/projectiveRotationScratch.js';
const projectRotationScratch=createProjectRotationScratch();
const frames=Array.from({length:1000},(_,i)=>quaternionColumns(new Quaternion().setFromAxisAngle(new Vector3(1,2,3).normalize(),i*.017))
    .map((c,j)=>c.map((v,k)=>v*(1+.1*Math.sin(i+j+k))+.01*Math.cos(i*2+j-k))));
const measure=f=>{let checksum=0;const start=performance.now();
    for(let n=0;n<200;n++)for(const c of frames)checksum+=f(c).columns[0][0];
    return {ms:performance.now()-start,checksum};};
measure(projectRotation);measure(projectRotationPolar);measure(projectRotationScratch);
const trials=Array.from({length:3},()=>({jacobi:measure(projectRotation),polar:measure(projectRotationPolar),scratch:measure(projectRotationScratch)}));
const report={scope:'Node arithmetic microbenchmark, not app FPS',runtime:process.version,framesPerTrial:200000,trials,
    sourceHashes:Object.fromEntries(['src/physics/projectiveRodMath.js','src/physics/projectiveRotationPolar.js','src/physics/projectiveRotationScratch.js'].map(p=>[p,createHash('sha256').update(fs.readFileSync(p)).digest('hex')]))};
if(process.argv[2])fs.writeFileSync(process.argv[2],JSON.stringify(report,null,2)+'\n');
console.log(JSON.stringify(report,null,2));
