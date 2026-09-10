import test from 'node:test';
import assert from 'node:assert/strict';
import { selectKirchhoffMechanicalComponents as select } from '../src/physics/kirchhoffMechanicalComponents.js';
import { kirchhoffComponentBodies as bodiesOf } from '../src/physics/kirchhoffComponentBodies.js';

const rod = () => ({activeStart:0,activeEnd:3,segmentCount:3});
const world = (...bodies) => ({bodies,containments:[],toolContacts:[]});

test('disconnected active rods are genuine singletons with persistent identity', () => {
    const a=rod(),b=rod(),w=world(a,b),first=select(w);
    assert.deepEqual(first.map(bodiesOf),[[a],[b]]);
    assert.ok(first.every(c=>c.innerBody===undefined&&c.outerBody===undefined&&c.kirchhoffContacts.length===0));
    first[0].solverState={marker:1}; a.activeStart=1;
    assert.deepEqual(select(w),first);
    w.bodies.reverse();
    assert.equal(select(w)[1],first[0]);
    assert.equal(select(w)[0],first[1]);
});

test('active range must include one actual segment, including clamped bounds', () => {
    const a=rod(),b=rod(),w=world(a,b);
    b.activeStart=b.activeEnd; assert.deepEqual(select(w).map(bodiesOf),[[a]]);
    b.activeEnd=99; b.activeStart=3; assert.equal(select(w).length,1);
    b.activeStart=2; assert.equal(select(w).length,2);
    a.activeStart=-2; a.activeEnd=0; assert.deepEqual(select(w).map(bodiesOf),[[b]]);
});

test('containment object and its ordering/history are returned unchanged', () => {
    const a=rod(),b=rod(),w=world(a,b),c={innerBody:b,outerBody:a,enabled:true,kirchhoffContacts:[{history:1}]};
    w.containments.push(c); w.toolContacts.push({bodyA:a,bodyB:b,enabled:true});
    assert.equal(select(w)[0],c); assert.deepEqual(bodiesOf(select(w)[0]),[b,a]);
    assert.equal(c.kirchhoffContacts[0].history,1); assert.equal(c.bodies,undefined);
});

test('external-only pair merges and splits without losing singleton or pair identity', () => {
    const a=rod(),b=rod(),w=world(a,b),singles=select(w),edge={bodyA:a,bodyB:b,enabled:true};
    w.toolContacts.push(edge); const pair=select(w)[0];
    assert.deepEqual(bodiesOf(pair),[a,b]); assert.deepEqual(pair.kirchhoffContacts,[]);
    edge.enabled=false; assert.deepEqual(select(w),singles);
    edge.enabled=true; assert.equal(select(w)[0],pair);
    const lumen={innerBody:a,outerBody:b,enabled:true}; w.containments.push(lumen);
    assert.equal(select(w)[0],lumen); lumen.enabled=false; assert.equal(select(w)[0],pair);
    b.activeEnd=0; assert.equal(select(w)[0],singles[0]);
    b.activeEnd=3; assert.equal(select(w)[0],pair);
});

test('removed bodies release cached ownership; separate worlds do not share cache', () => {
    const a=rod(),b=rod(),w=world(a,b),workspace={},old=select(w,workspace);
    w.bodies=[a]; select(w,workspace); assert.equal(workspace.components.length,1);
    w.bodies.push(b); assert.notEqual(select(w,workspace)[1],old[1]);
    assert.notEqual(select(world(a))[0],old[0]);
});

test('unsupported connected graphs and ambiguous lumen owners are rejected', () => {
    const a=rod(),b=rod(),c=rod(),w=world(a,b,c);
    w.toolContacts=[{bodyA:a,bodyB:b,enabled:true},{bodyA:b,bodyB:c,enabled:true}];
    assert.throws(()=>select(w),/at most two/);
    w.toolContacts=[]; w.containments=[{innerBody:a,outerBody:b,enabled:true},{innerBody:b,outerBody:a,enabled:true}];
    assert.throws(()=>select(w),/multiple containment/);
    w.containments=[]; w.toolContacts=[{bodyA:a,bodyB:rod(),enabled:true}];
    assert.throws(()=>select(w),/world bodies/);
    w.toolContacts[0].enabled=false; assert.equal(select(w).length,3);
    assert.throws(()=>select(world(a,a)),/distinct/);
});
