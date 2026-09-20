import fs from 'node:fs';
import os from 'node:os';
import path from 'node:path';
import {spawnSync} from 'node:child_process';

const staging=fs.mkdtempSync(path.join(os.tmpdir(),'oet-aneurysm-'));
const name='Aorta_infrarenal_aneurysm';
const staged=suffix=>path.join(staging,name+suffix);
function run(args) {
    const result=spawnSync(process.execPath,args,{stdio:'inherit'});
    if(result.error)throw result.error;
    if(result.status!==0)throw Error(`Anatomy build failed (${result.status ?? result.signal}); existing assets retained`);
}
try {
    run(['scripts/generate-infrarenal-aneurysm.mjs',staging]);
    run(['--max-old-space-size=8192','scripts/build-collision-asset.mjs',
        staged('.stl'),staged('.collision.bin'),path.join(staging,'collision-report.json'),staged('.json')]);
    // Copy onto the destination filesystem first. Publish only after every
    // expensive generation/validation step has succeeded.
    const suffixes=['.stl','.collision.bin','.json','.mesh-repair.json'];
    for(const suffix of suffixes)fs.copyFileSync(staged(suffix),`res/${name}${suffix}.next`);
    for(const suffix of suffixes)fs.renameSync(`res/${name}${suffix}.next`,`res/${name}${suffix}`);
    fs.mkdirSync('out',{recursive:true});
    fs.copyFileSync(path.join(staging,'collision-report.json'),'out/aneurysm-collision-report.json');
    console.log('Published matching aneurysm geometry and collision assets.');
} finally {
    fs.rmSync(staging,{recursive:true,force:true});
}
