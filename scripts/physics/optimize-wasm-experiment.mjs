// Research only; deliberately not part of the production build.
// OET_BINARYEN_PATH=/absolute/path/to/binaryen/index.js node this-script.mjs input.js output.js
// Install binaryen@132.0.0 separately with --ignore-scripts. Use an isolated
// checkout and the paired crossover benchmark before considering a rollout.
import fs from 'node:fs/promises';
import assert from 'node:assert/strict';
import {dirname,resolve} from 'node:path';
import {pathToFileURL} from 'node:url';
import {createHash} from 'node:crypto';

const [input,output]=process.argv.slice(2),compilerPath=process.env.OET_BINARYEN_PATH;
assert.ok(input&&output&&compilerPath,'Provide input.js, output.js and OET_BINARYEN_PATH');
const packageInfo=JSON.parse(await fs.readFile(resolve(dirname(compilerPath),'package.json'),'utf8'));
assert.equal(packageInfo.version,'132.0.0','Use the pinned research compiler version');
const {default:binaryen}=await import(pathToFileURL(resolve(compilerPath)));
const exports=await import(pathToFileURL(resolve(input)));
const entries=Object.entries(exports).filter(([,value])=>value instanceof Uint8Array);
assert.equal(entries.length,1,'Expected one exported byte array');
const [name,bytes]=entries[0];
const allowed=new Set(['sharedAxisMaterialKernelBytes','linearKernelBytes','bandSimdKernelBytes']);
assert.ok(allowed.has(name),'Only the measured material/linear/SIMD kernels are supported');

binaryen.setOptimizeLevel(3);binaryen.setShrinkLevel(0);
binaryen.setFastMath(false);binaryen.setLowMemoryUnused(false);
const features=binaryen.Features.MVP |
    (name==='linearKernelBytes'?binaryen.Features.BulkMemory|binaryen.Features.BulkMemoryOpt:0) |
    (name==='bandSimdKernelBytes'?binaryen.Features.SIMD128:0);
const module=binaryen.readBinary(bytes);
let result;
try {
    module.setFeatures(features);
    assert.ok(module.validate(),'Invalid source module');
    module.optimize();
    assert.ok(module.validate(),'Invalid optimized module');
    result=module.emitBinary();
} finally {module.dispose();}
assert.ok(WebAssembly.validate(result),'The local runtime must accept the optimized module');
const header='// Research output: Binaryen 132.0.0, optimize=3, shrink=0, fastMath=false, lowMemoryUnused=false.\n';
await fs.writeFile(output,header+`export const ${name}=new Uint8Array([${Array.from(result)}]);\n`);
const hash=data=>createHash('sha256').update(data).digest('hex');
console.log(JSON.stringify({input,output,name,features,fastMath:binaryen.getFastMath(),
    beforeBytes:bytes.length,afterBytes:result.length,beforeSha256:hash(bytes),afterSha256:hash(result)}));
