import test from 'node:test';
import assert from 'node:assert/strict';
import {preserveSharedAxisBends} from '../src/physics/kirchhoffSharedAxisRemesh.js';

function example() {
    const coordinates=[-10,-5,0,5,10,10.8,15,20],angles=[0,0,0,0,35,70,105];
    const positions=[[0,0,0]];
    for(let i=1;i<coordinates.length;i++){
        const length=coordinates[i]-coordinates[i-1],angle=angles[i-1]*Math.PI/180,p=positions.at(-1);
        positions.push([p[0]+length*Math.cos(angle),p[1]+length*Math.sin(angle),0]);
    }
    return {coordinates,positions};
}
function sampled(previous,coordinates) {
    return coordinates.map(x=>{
        const right=previous.coordinates.findIndex(c=>c>=x),left=Math.max(0,right-1);
        if(left===right)return previous.positions[left].slice();
        const t=(x-previous.coordinates[left])/(previous.coordinates[right]-previous.coordinates[left]);
        return previous.positions[left].map((v,k)=>(1-t)*v+t*previous.positions[right][k]);
    });
}
function maxAngle(positions) {
    let result=0;
    for(let i=1;i+1<positions.length;i++){
        const u=positions[i].map((v,k)=>v-positions[i-1][k]),v=positions[i+1].map((v,k)=>v-positions[i][k]);
        result=Math.max(result,Math.acos(Math.max(-1,Math.min(1,u.reduce((s,x,k)=>s+x*v[k],0)/Math.hypot(...u)/Math.hypot(...v)))));
    }
    return result;
}
test('coarsening retains the accepted bends without snapping a moving material tip',()=>{
    const previous=example(),saved=structuredClone(previous),coordinates=[-10,-5,0,5,10.266,15,20],positions=sampled(previous,coordinates);
    assert.ok(maxAngle(positions)>Math.PI/4);
    preserveSharedAxisBends(coordinates,positions,previous,Math.PI/4);
    assert.ok(coordinates.includes(10.266));assert.equal(coordinates.length,8,'One old knot suffices; do not retain the whole tip trail');
    assert.ok(maxAngle(positions)<Math.PI/4+1e-12);
    assert.deepEqual(previous,saved);assert.deepEqual(positions,sampled(previous,coordinates));
});
test('retained knots can disappear after relaxation and do not accumulate permanent boundaries',()=>{
    const previous=example();previous.positions=previous.coordinates.map(x=>[x,0,0]);
    const coordinates=[-10,-5,0,5,10.266,15,20],positions=sampled(previous,coordinates),before=coordinates.slice();
    preserveSharedAxisBends(coordinates,positions,previous,Math.PI/4);
    assert.deepEqual(coordinates,before);
});
test('an unlimited bend policy does not change remeshing',()=>{
    const previous=example(),coordinates=[-10,-5,0,5,10.266,15,20],positions=sampled(previous,coordinates),before=coordinates.slice();
    preserveSharedAxisBends(coordinates,positions,previous,Infinity);assert.deepEqual(coordinates,before);
});
