import test from 'node:test';
import assert from 'node:assert/strict';
import fs from 'node:fs';
import os from 'node:os';
import path from 'node:path';
import {fileURLToPath} from 'node:url';
import {spawnSync} from 'node:child_process';

const script=fileURLToPath(new URL('../scripts/rebuild-infrarenal-aneurysm.mjs',import.meta.url));
const name='Aorta_infrarenal_aneurysm';
const suffixes=['.stl','.collision.bin','.json','.mesh-repair.json'];

for(const fail of [false,true])test(`aneurysm rebuild ${fail?'retains published assets on failure':'publishes only the completed pair'}`,()=>{
    const directory=fs.mkdtempSync(path.join(os.tmpdir(),'oet-rebuild-test-'));
    try {
        fs.mkdirSync(path.join(directory,'res'));fs.mkdirSync(path.join(directory,'scripts'));
        for(const suffix of suffixes)fs.writeFileSync(path.join(directory,'res',name+suffix),'old');
        fs.writeFileSync(path.join(directory,'scripts/generate-infrarenal-aneurysm.mjs'),`
            import fs from 'node:fs';import path from 'node:path';
            for(const suffix of ['.stl','.json','.mesh-repair.json'])
                fs.writeFileSync(path.join(process.argv[2],'${name}'+suffix),'new');
        `);
        fs.writeFileSync(path.join(directory,'scripts/build-collision-asset.mjs'),`
            import fs from 'node:fs';import assert from 'node:assert/strict';
            assert.equal(fs.readFileSync('res/${name}.stl','utf8'),'old');
            assert.equal(fs.readFileSync('res/${name}.collision.bin','utf8'),'old');
            assert.equal(fs.readFileSync(process.argv[2],'utf8'),'new');
            if(${fail})process.exit(7);
            fs.writeFileSync(process.argv[3],'new');fs.writeFileSync(process.argv[4],'report');
        `);
        const result=spawnSync(process.execPath,[script],{cwd:directory,encoding:'utf8'});
        assert.equal(result.status===0,!fail,result.stderr);
        for(const suffix of suffixes)
            assert.equal(fs.readFileSync(path.join(directory,'res',name+suffix),'utf8'),fail?'old':'new');
        assert.deepEqual(fs.readdirSync(path.join(directory,'res')).sort(),suffixes.map(s=>name+s).sort());
    } finally {fs.rmSync(directory,{recursive:true,force:true});}
});
