// Reproduce the final rejected experiment in a NEW directory, never in the app.
import {cpSync,mkdirSync,existsSync,symlinkSync,readFileSync,writeFileSync} from 'node:fs';
import {resolve,join} from 'node:path';import {fileURLToPath} from 'node:url';
const repo=fileURLToPath(new URL('../../',import.meta.url)),target=process.argv[2];
if(!target)throw Error('Pass an unused absolute output directory');
const out=resolve(target);if(out!==target||existsSync(out)||out.startsWith(repo))throw Error('Output must be a new absolute directory outside this repository');
mkdirSync(out,{recursive:true});
for(const dir of ['src','scripts','tests'])cpSync(join(repo,dir),join(out,dir),{recursive:true});
for(const dir of ['public','res','node_modules'])symlinkSync(join(repo,dir),join(out,dir),'dir');
cpSync(join(repo,'package.json'),join(out,'package.json'));
for(const [saved,destination]of [
 ['kirchhoffSharedAxisNative.js','src/physics/kirchhoffSharedAxisNative.js'],
 ['kirchhoffSharedAxisLinear.js','src/physics/kirchhoffSharedAxisLinear.js'],
 ['profile-shared-axis-dynamic.mjs','scripts/physics/profile-shared-axis-dynamic.mjs'],
 ['sharedAxisWarmStartComparison.js','tests/helpers/sharedAxisWarmStartComparison.js']
])writeFileSync(join(out,destination),readFileSync(new URL('experimental-source/'+saved+'.txt',import.meta.url)));
console.log(out);
