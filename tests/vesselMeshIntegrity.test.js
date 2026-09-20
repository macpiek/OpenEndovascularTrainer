import test from 'node:test';
import assert from 'node:assert/strict';
import fs from 'node:fs';
import crypto from 'node:crypto';
import {decodeCollisionAsset} from '../src/physics/collision/collisionAssetFormat.js';

for (const variant of ['plain','infrarenal_aneurysm']) test(`${variant}: repaired STL has valid triangles and matching collision data`,()=>{
    const base=new URL(`../res/Aorta_${variant}`,import.meta.url);
    const bytes=fs.readFileSync(`${base.pathname}.stl`);
    const report=JSON.parse(fs.readFileSync(`${base.pathname}.mesh-repair.json`));
    assert.equal(crypto.createHash('sha256').update(bytes).digest('hex'),report.outputSha256);
    const count=bytes.readUInt32LE(80);
    assert.equal(count,report.after.triangles);
    assert.equal(bytes.length,84+50*count);
    for(let i=0;i<count;i++) {
        const at=84+50*i,p=Array.from({length:9},(_,j)=>bytes.readFloatLE(at+12+j*4));
        assert.ok(p.every(Number.isFinite),`non-finite vertex at ${i}`);
        const a=[p[3]-p[0],p[4]-p[1],p[5]-p[2]],b=[p[6]-p[0],p[7]-p[1],p[8]-p[2]];
        const cross=[a[1]*b[2]-a[2]*b[1],a[2]*b[0]-a[0]*b[2],a[0]*b[1]-a[1]*b[0]],length=Math.hypot(...cross);
        assert.ok(length>1e-12,`zero-area triangle ${i}`);
        const normal=[0,1,2].map(j=>bytes.readFloatLE(at+j*4));
        assert.ok(Math.abs(Math.hypot(...normal)-1)<1e-6,`invalid normal ${i}`);
        assert.ok(cross.reduce((sum,v,j)=>sum+v*normal[j],0)/length>0.999999,`reversed stored normal ${i}`);
    }
    assert.equal(report.after.zeroArea,0);
    assert.equal(report.after.duplicates,0);
    assert.ok(report.after.boundaryEdges<report.before.boundaryEdges/10);
    assert.ok(report.after.nonManifoldEdges<report.before.nonManifoldEdges);
    assert.ok(report.after.inconsistentWinding<report.before.inconsistentWinding);
    const packed=fs.readFileSync(`${base.pathname}.collision.bin`);
    const asset=decodeCollisionAsset(packed.buffer.slice(packed.byteOffset,packed.byteOffset+packed.byteLength));
    assert.equal(asset.metadata.source.stlSha256,report.outputSha256);
    assert.equal(asset.metadata.source.triangleCount,count);
    assert.equal(asset.metadata.centerline.diagnostics.componentCount,1);
    assert.equal(asset.metadata.centerline.diagnostics.centerlineInvalidSegmentCountFinal,0);
});
