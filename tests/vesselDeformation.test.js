import test from 'node:test';
import assert from 'node:assert/strict';
import * as THREE from 'three';
import {VesselDeformation} from '../src/vesselDeformation.js';

test('spatial shader index retains every patch, handles negative cells and resets on unloading',()=>{
    const deformation=new VesselDeformation();
    const patches=Array.from({length:250},(_,i)=>({center:[i*3-420,(i%11)*12-60,17],width:2,normal:[0,0,1],displacement:.1}));
    assert.equal(deformation.update(patches),true);
    const u=deformation.uniforms,table=u.vesselBuckets.value.image.data,refs=u.vesselReferences.value.image.data;
    for(const [i,p] of patches.entries())for(const offset of [-1.5,0,1.5]) {
        const cell=p.center.map((v,k)=>Math.floor((v+(k===0?offset:0))/8));
        const hash=((cell[0]*17+cell[1]*31+cell[2]*47)%4096+4096)%4096;
        const start=table[hash*4],count=table[hash*4+1];
        assert.ok(Array.from({length:count},(_,j)=>refs[4*(start+j)]).includes(i));
    }
    const revision=deformation.revision;
    assert.equal(deformation.update(structuredClone(patches)),false);assert.equal(deformation.revision,revision);
    deformation.update([]);assert.equal(u.vesselDeformActive.value,false);
    deformation.dispose();
});

test('wall, contrast and depth materials share the same displacement uniforms without changing rest geometry',()=>{
    const deformation=new VesselDeformation();
    const custom=new THREE.ShaderMaterial({vertexShader:'void main(){gl_Position=projectionMatrix*modelViewMatrix*vec4(position,1.);}',fragmentShader:'void main(){gl_FragColor=vec4(1.);}'});
    deformation.attach(custom);
    assert.match(custom.vertexShader,/position\+vesselDisplacement\(position\)/);
    assert.equal(custom.uniforms.vesselPatches,deformation.uniforms.vesselPatches);
    for(const material of [new THREE.MeshBasicMaterial(),new THREE.MeshDepthMaterial()]) {
        deformation.attach(material);
        const shader={uniforms:{},vertexShader:'#include <begin_vertex>'};material.onBeforeCompile(shader);
        assert.match(shader.vertexShader,/transformed\+=vesselDisplacement\(position\)/);
        assert.equal(shader.uniforms.vesselPatches,custom.uniforms.vesselPatches);material.dispose();
    }
    custom.dispose();deformation.dispose();
});

test('reused spatial buckets discard old patches across movement, unloading and reloading',()=>{
    const reused=new VesselDeformation();
    try {
        for(const x of [-100,0,140,0]) {
            const patches=[{center:[x,2,3],width:4,normal:[1,0,0],displacement:.2}];
            const fresh=new VesselDeformation();
            try {
                reused.update(patches);fresh.update(patches);
                for(const name of ['vesselBuckets','vesselReferences','vesselPatches']) {
                    const a=reused.uniforms[name].value.image.data,b=fresh.uniforms[name].value.image.data;
                    assert.deepEqual(a.slice(0,b.length),b);
                }
                for(const at of [-100,0,140])assert.deepEqual(reused.displacementAt(new THREE.Vector3(at,2,3)),fresh.displacementAt(new THREE.Vector3(at,2,3)));
                if(x===140){reused.update([]);assert.deepEqual(reused.displacementAt(new THREE.Vector3(x,2,3)),new THREE.Vector3());}
            }finally{fresh.dispose();}
        }
    }finally{reused.dispose();}
});
