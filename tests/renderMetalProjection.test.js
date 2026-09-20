import test from 'node:test';
import assert from 'node:assert/strict';
import * as THREE from 'three';
import {renderMetalProjection} from '../src/imaging/renderMetalProjection.js';

for(const fail of [false,true])test(`single metal pass preserves both wires, graft visibility and materials${fail?' on failure':''}`,()=>{
    const scene=new THREE.Scene(),camera=new THREE.PerspectiveCamera();scene.background=new THREE.Color('black');
    const wireGroups=[new THREE.Group(),new THREE.Group()],graftGroup=new THREE.Group(),anatomy=new THREE.Group(),hidden=new THREE.Group();
    hidden.visible=false;scene.add(camera,...wireGroups,graftGroup,anatomy,hidden);
    const original=new THREE.MeshBasicMaterial(),wireMaterial=new THREE.MeshBasicMaterial(),graftMaterial=new THREE.MeshBasicMaterial();
    for(const group of wireGroups)group.add(new THREE.Mesh(new THREE.BufferGeometry(),original));
    const fabric=new THREE.Mesh(new THREE.BufferGeometry(),original);fabric.visible=false;graftGroup.add(fabric);
    const metal=new THREE.LineSegments(new THREE.BufferGeometry(),original);graftGroup.add(metal);
    scene.overrideMaterial=original;let calls=0;
    const renderer={render(s,c){
        calls++;assert.equal(s,scene);assert.equal(c,camera);assert.equal(s.overrideMaterial,null);
        assert.ok(wireGroups.every(g=>g.visible&&g.children[0].material===wireMaterial));
        assert.equal(graftGroup.visible,true);assert.equal(metal.material,graftMaterial);assert.equal(fabric.visible,false);
        assert.equal(anatomy.visible,false);assert.equal(hidden.visible,false);
        if(fail)throw Error('GPU error');
    }};
    const draw=()=>renderMetalProjection(renderer,scene,camera,{wireGroups,graftGroup,wireMaterial,graftMaterial});
    if(fail)assert.throws(draw,/GPU error/);else draw();
    assert.equal(calls,1,'multiple renders can clear or discard the preceding wire mask');
    assert.equal(scene.overrideMaterial,original);assert.equal(anatomy.visible,true);assert.equal(hidden.visible,false);
    assert.equal(fabric.visible,false);
    for(const group of [...wireGroups,graftGroup])for(const mesh of group.children)assert.equal(mesh.material,original);
});
