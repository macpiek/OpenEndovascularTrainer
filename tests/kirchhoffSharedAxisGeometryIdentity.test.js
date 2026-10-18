import test from 'node:test';
import assert from 'node:assert/strict';
import {BufferGeometry,BufferAttribute} from 'three';
import {sharedAxisGeometryIdentity} from '../src/physics/kirchhoffSharedAxisGeometryIdentity.js';

// The v1 reference remains independent of the optimized traversal. Replay
// compatibility includes typed-array subviews and unversioned mesh edits.
function reference(geometry) {
    const position=geometry.getAttribute('position');
    let a=2166136261,b=0x9e3779b9;
    const arrays=[position.array,geometry.index?.array];
    for(const array of arrays)if(array)for(const value of new Uint8Array(array.buffer,array.byteOffset,array.byteLength)){
        a=Math.imul(a^value,16777619);b=Math.imul(b^value,2246822519);b=(b<<13)|(b>>>19);
    }
    return `mesh-v1/${position.itemSize}/${arrays.map(x=>x?`${x.constructor.name}:${x.length}`:'none').join('/')}/${(a>>>0).toString(16)}/${(b>>>0).toString(16)}`;
}

test('mesh digest preserves v1 bytes for indexed, nonindexed and offset views',()=>{
    for(const Position of [Float32Array,Float64Array])for(const Index of [null,Uint16Array,Uint32Array]){
        const geometry=new BufferGeometry(),positions=new Position([999,1.25,-2,3,4,5,6,7,8,9,999]);
        geometry.setAttribute('position',new BufferAttribute(positions.subarray(1,10),3));
        if(Index)geometry.setIndex(new BufferAttribute(new Index([99,0,1,2,99]).subarray(1,4),1));
        assert.equal(sharedAxisGeometryIdentity(geometry),reference(geometry));
        const original=sharedAxisGeometryIdentity(geometry);
        positions[0]++;positions[10]++;
        assert.equal(sharedAxisGeometryIdentity(geometry),original,'Bytes outside the view do not belong to the mesh');
        positions[1]+=.25;
        assert.notEqual(sharedAxisGeometryIdentity(geometry),original,'Unversioned vertex edits invalidate certificates');
        assert.equal(sharedAxisGeometryIdentity(geometry),reference(geometry));
        if(Index){const prior=sharedAxisGeometryIdentity(geometry);geometry.index.array[0]=2;
            assert.notEqual(sharedAxisGeometryIdentity(geometry),prior);assert.equal(sharedAxisGeometryIdentity(geometry),reference(geometry));}
        geometry.dispose();
    }
    assert.equal(sharedAxisGeometryIdentity(null),null);
    assert.equal(sharedAxisGeometryIdentity(new BufferGeometry()),null);
});

test('cached identity checks all words, trailing bytes and layout without relying on version',()=>{
    const geometry=new BufferGeometry();
    const positions=new Float32Array(30003);
    geometry.setAttribute('position',new BufferAttribute(positions,3));
    geometry.setIndex(new BufferAttribute(new Uint16Array([0,1,2]),1));
    for(const index of [0,1,positions.length-1]){
        const before=sharedAxisGeometryIdentity(geometry);
        positions[index]+=1;
        assert.notEqual(sharedAxisGeometryIdentity(geometry),before);
        assert.equal(sharedAxisGeometryIdentity(geometry),reference(geometry));
    }
    const before=sharedAxisGeometryIdentity(geometry);
    geometry.index.array[2]=3; // last two bytes, outside the uint32 prefix
    assert.notEqual(sharedAxisGeometryIdentity(geometry),before);
    assert.equal(sharedAxisGeometryIdentity(geometry),reference(geometry));
    geometry.setAttribute('position',new BufferAttribute(positions.slice(),1));
    assert.equal(sharedAxisGeometryIdentity(geometry),reference(geometry));
    geometry.setIndex(null);
    assert.equal(sharedAxisGeometryIdentity(geometry),reference(geometry));
    geometry.dispose();
});
