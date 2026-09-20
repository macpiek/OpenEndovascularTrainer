import test from 'node:test';
import assert from 'node:assert/strict';
import fs from 'node:fs';
import crypto from 'node:crypto';
import * as THREE from 'three';
import {STLLoader} from 'three/examples/jsm/loaders/STLLoader.js';
import {MeshBVH} from 'three-mesh-bvh';
import {transformAortaGeometry} from '../src/aortaTransform.js';
import {generateVessel} from '../src/vesselGeometry.js';
import {decodeCollisionAsset} from '../src/physics/collision/collisionAssetFormat.js';
import {VesselContactField,createContactResult} from '../src/physics/collision/vesselContactField.js';
import {createInfrarenalDeformation,axialSection} from '../scripts/anatomy/infrarenalAneurysm.mjs';
import {selectInfrarenalSurface,selectInfrarenalCenterline} from '../scripts/anatomy/infrarenalSelection.mjs';

const read=name=>fs.readFileSync(new URL('../res/'+name,import.meta.url));
const arrayBuffer=b=>b.buffer.slice(b.byteOffset,b.byteOffset+b.byteLength);
const hash=b=>crypto.createHash('sha256').update(b).digest('hex');
const report=JSON.parse(read('Aorta_infrarenal_aneurysm.json'));
const deformation=createInfrarenalDeformation(report);
const baseline=read('Aorta_plain.stl'),generated=read('Aorta_infrarenal_aneurysm.stl');
const geometries=[baseline,generated].map(bytes=>{
    const geometry=new STLLoader().parse(arrayBuffer(bytes));
    transformAortaGeometry(geometry,generateVessel().vessel);return geometry;
});

test('aneurysm is approximately 50 mm and preserves baseline anatomy outside its support',()=>{
    assert.equal(hash(baseline),report.sourceSha256);
    assert.equal(hash(generated),report.outputSha256);
    assert.equal(baseline.readUInt32LE(80),generated.readUInt32LE(80));
    const original=geometries[0].attributes.position.array,next=geometries[1].attributes.position.array;
    const y=report.referenceY;
    const diameter=axialSection(next,y,deformation.centerAt(y),18,original).diameter;
    assert.ok(diameter>49 && diameter<51,`diameter ${diameter}`);
    for(let i=0;i<original.length;i+=3) {
        if(original[i+1]>report.distalY && original[i+1]<report.proximalY)continue;
        assert.equal(next[i],original[i]);assert.equal(next[i+1],original[i+1]);assert.equal(next[i+2],original[i+2]);
    }
    for(const p of [...report.renalOrigins,report.bifurcation,[-73,-383,14],[38.5,-390,10]])
        assert.deepEqual(deformation.move(...p),p);
});

test('sac stays enlarged down to the bifurcation without a distal aortic neck',()=>{
    const original=geometries[0].attributes.position.array,next=geometries[1].attributes.position.array;
    assert.equal(report.sacDistalY,report.bifurcation[1]);
    for(const y of [-230,-240,-248,report.bifurcation[1]+1]) {
        const center=deformation.centerAt(y);
        const baselineDiameter=axialSection(original,y,center).diameter;
        const diameter=axialSection(next,y,center,18,original).diameter;
        assert.ok(diameter>45 && diameter>baselineDiameter*1.5,
            `distal narrowing at ${y}: ${diameter}/${baselineDiameter}`);
        assert.ok(diameter<53,`bifurcation over-expanded at ${y}: ${diameter}`);
    }
    const bif=report.bifurcation;
    assert.deepEqual(deformation.move(...bif),bif,'branch center stays in place');
});

test('neighboring vessels in the same slab keep their original vertices and centerlines',()=>{
    assert.equal(report.selection,'connected-infrarenal-slab');
    const original=geometries[0].attributes.position.array,next=geometries[1].attributes.position.array;
    const selected=selectInfrarenalSurface(original,report,deformation.centerAt);
    let protectedVertices=0,previouslyMoved=0;
    for(let i=0;i<selected.length;i++) {
        if(selected[i])continue;
        for(let k=0;k<3;k++)assert.equal(next[3*i+k],original[3*i+k],`unrelated vertex ${i}`);
        const p=Array.from(original.slice(3*i,3*i+3));
        if(p[1]<=report.distalY || p[1]>=report.proximalY)continue;
        protectedVertices++;
        if(Math.hypot(...deformation.move(...p).map((v,k)=>v-p[k]))>.1)previouslyMoved++;
    }
    assert.ok(protectedVertices>10000);
    assert.ok(previouslyMoved>10000,'must cover nearby vessels previously displaced by the broad field');
    const asset=decodeCollisionAsset(arrayBuffer(read('Aorta_plain.collision.bin')));
    const selectedSegments=selectInfrarenalCenterline(asset,report,deformation.centerAt);
    assert.deepEqual(report.selectedCenterlineSegments,selectedSegments);
    const generatedAsset=decodeCollisionAsset(arrayBuffer(read('Aorta_infrarenal_aneurysm.collision.bin')));
    const key=(data,i)=>Array.from(data.slice(i,i+6)).join(',');
    const unchanged=new Set();
    for(let i=0;i<generatedAsset.arrays.centerlineSegments.length;i+=9)
        unchanged.add(key(generatedAsset.arrays.centerlineSegments,i));
    const included=new Set(selectedSegments);
    for(let i=0;i<asset.arrays.centerlineSegments.length/9;i++) {
        if(included.has(i))continue;
        assert.ok(unchanged.has(key(asset.arrays.centerlineSegments,9*i)),`unrelated centerline segment ${i}`);
    }
});

test('smooth surrounding displacement does not fold the anatomy',()=>{
    const h=.001;
    for(let y=report.distalY+1;y<report.proximalY;y+=3) {
        const center=deformation.centerAt(y);
        for(let radius=1;radius<=report.outerRadius;radius+=2)for(let a=0;a<Math.PI*2;a+=Math.PI/16) {
            const x=center[0]+radius*Math.cos(a),z=center[2]+radius*Math.sin(a);
            const px=deformation.move(x+h,y,z),mx=deformation.move(x-h,y,z);
            const pz=deformation.move(x,y,z+h),mz=deformation.move(x,y,z-h);
            // Y is unchanged: the full 3D determinant equals this transverse one.
            const determinant=((px[0]-mx[0])*(pz[2]-mz[2])-(pz[0]-mz[0])*(px[2]-mx[2]))/(4*h*h);
            assert.ok(determinant>.2,`fold at ${x},${y},${z}: ${determinant}`);
        }
    }
});

test('generated collision field follows the sac and retains a connected lumen',()=>{
    const asset=decodeCollisionAsset(arrayBuffer(read('Aorta_infrarenal_aneurysm.collision.bin')));
    assert.equal(asset.metadata.source.stlSha256,report.outputSha256);
    assert.equal(asset.metadata.centerline.diagnostics.componentCount,1);
    assert.equal(asset.metadata.centerline.diagnostics.centerlineInvalidSegmentCountFinal,0);
    assert.ok(asset.metadata.decodedBytes<=64*1024*1024);
    geometries[1].boundsTree=new MeshBVH(geometries[1]);
    const field=new VesselContactField(asset,{fallbackGeometry:geometries[1]});
    const baselineAsset=decodeCollisionAsset(arrayBuffer(read('Aorta_plain.collision.bin')));
    const junctionDegrees=edges=>{
        const degree=new Map();for(const id of edges)degree.set(id,(degree.get(id)||0)+1);
        return [...degree.values()].filter(d=>d!==2).sort((a,b)=>a-b);
    };
    assert.deepEqual(junctionDegrees(asset.arrays.centerlineEdges),junctionDegrees(baselineAsset.arrays.centerlineEdges),
        'deformation must preserve every branch and terminal');
    const originalField=new VesselContactField(baselineAsset);
    const center=deformation.centerAt(-226),p=new THREE.Vector3(center[0]+18,-226,center[2]+12);
    assert.equal(originalField.querySphere(p,.5,createContactResult()).violation,true,'probe is outside original lumen');
    assert.equal(field.querySphere(p,.5,createContactResult()).violation,false,'expanded lumen must admit the probe');
    const outside=new THREE.Vector3(center[0]+32,-226,center[2]+12);
    assert.equal(field.querySphere(outside,.5,createContactResult()).violation,true,'sac wall must still contain tools');
    const hit={point:new THREE.Vector3()};
    for(let y=report.bifurcation[1];y<report.renalOrigins[0][1];y+=.5) {
        const axis=deformation.centerAt(y),point=new THREE.Vector3(...deformation.move(...axis));
        geometries[1].boundsTree.closestPointToPoint(point,hit);
        assert.ok(hit.distance>2,`obstructed infrarenal lumen at ${y}`);
        assert.equal(field.querySphere(point,.5,createContactResult()).violation,false,`centerline outside at ${y}`);
    }
    // Follow both iliac continuations through the enlarged bifurcation and
    // the distal blend, rather than checking only the upstream aortic axis.
    const data=baselineAsset.arrays.centerlineSegments,sides=new Set();
    const selectedSegments=new Set(report.selectedCenterlineSegments);
    for(let i=0;i<data.length;i+=9) {
        // A nearby mesenteric branch can cross this slab too; it is protected,
        // so it must not be mistaken for a deformed iliac continuation.
        if(!selectedSegments.has(i/9))continue;
        const a=new THREE.Vector3().fromArray(data,i),b=new THREE.Vector3().fromArray(data,i+3);
        const middle=a.clone().lerp(b,.5);
        if(middle.y<report.distalY || middle.y>report.bifurcation[1] ||
            Math.hypot(middle.x-report.bifurcation[0],middle.z-report.bifurcation[2])>40)continue;
        sides.add(middle.x<report.bifurcation[0]?'right':'left');
        const count=Math.max(1,Math.ceil(a.distanceTo(b)/.5));
        for(let j=0;j<=count;j++) {
            const original=a.clone().lerp(b,j/count),point=new THREE.Vector3(...deformation.move(...original.toArray()));
            assert.equal(field.querySphere(point,.5,createContactResult()).violation,false,
                `iliac transition outside at ${original.toArray()}`);
        }
    }
    assert.deepEqual([...sides].sort(),['left','right']);
});
test.after(()=>geometries.forEach(g=>g.dispose()));
