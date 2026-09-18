import test from 'node:test';
import assert from 'node:assert/strict';
import {BufferGeometry,Float32BufferAttribute} from 'three';
import {createSharedAxisVesselWitness} from '../src/physics/kirchhoffSharedAxisVesselWitnesses.js';
import {deferSharedAxisContact,materializeSharedAxisContact,separatedSharedAxisContactDirection} from '../src/physics/kirchhoffSharedAxisInactiveContacts.js';
import {createSharedAxisLayout,createSharedAxisLinear,solveSharedAxisLinear} from '../src/physics/kirchhoffSharedAxisLinear.js';
import {createSharedAxisNative,assembleSharedAxisNative,extendSharedAxisNativeRows,captureSharedAxisNative,applySharedAxisNativeIncrement,restoreSharedAxisNative} from '../src/physics/kirchhoffSharedAxisNative.js';
import {createSharedAxisConstraintRowPool} from '../src/physics/kirchhoffSharedAxisConstraintRows.js';

function fixture(t=1) {
    const geometry=new BufferGeometry().setAttribute('position',new Float32BufferAttribute([0,-100,-100,0,100,-100,0,0,100],3));
    const layout=createSharedAxisLayout([['wire']]),p=layout.positions[1];
    const definition=createSharedAxisVesselWitness({fallbackGeometry:geometry},{kind:'wall',edge:0,id:'wall',dofs:[0,1,2,p,p+1,p+2],witness:{face:0,t}});
    const input={a:[2,0,0],b:[2,1,0],state:{origin:[0,0,0]},radius:.5,needHessian:false};
    // First bound request enables a certificate; exact evaluation seeds it.
    assert.equal(definition.evaluate.inactiveClearance(input),-Infinity);
    const contact=definition.evaluate(input);
    const row={...definition,gap:contact.gap,jacobian:contact.jacobian.slice(),multiplier:0};
    return {geometry,layout,p,definition,input,row};
}

test('clearance bounds are conservative across face/edge/vertex changes, origin and radius changes',()=>{
    const {definition,input}=fixture(.3),bound=definition.evaluate.inactiveClearance;
    let seed=73;const random=()=>((seed=(1664525*seed+1013904223)>>>0)/2**32);
    for(let i=0;i<200;i++) {
        const next={...input,a:[random()*20+.1,random()*400-200,random()*400-200],b:[random()*20+.1,random()*400-200,random()*400-200],
            state:{origin:[random(),random(),random()]},radius:random()*2};
        const lower=bound(next),exact=definition.evaluate(next);
        assert.ok(lower<=exact.gap,`${lower} > ${exact.gap}`);
    }
});

test('in-place triangle edits and failed evaluations invalidate certificates without relying on attribute versions',()=>{
    const {geometry,definition,input}=fixture();
    assert.ok(definition.evaluate.inactiveClearance(input)>1);
    geometry.attributes.position.array[0]=1.9;
    assert.equal(definition.evaluate.inactiveClearance(input),-Infinity);
    definition.evaluate(input);
    geometry.attributes.position.array[0]=NaN;
    assert.equal(definition.evaluate.inactiveClearance(input),-Infinity);
    assert.throws(()=>definition.evaluate(input));
    geometry.attributes.position.array[0]=0;
    assert.equal(definition.evaluate.inactiveClearance(input),-Infinity);
    definition.evaluate(input);assert.ok(definition.evaluate.inactiveClearance(input)>1);
});

test('large directions materialize contacts before activation; small directions skip exact evaluation',()=>{
    for(const force of [.1,3]) {
        const results=[];
        for(const deferred of [false,true]) {
            const {layout,p,definition,input,row}=fixture();let calls=0;
            if(deferred)deferSharedAxisContact(row,arg=>{calls++;return definition.evaluate(arg);},input,definition.evaluate.inactiveClearance(input));
            const chain={layout,hessian:new Float64Array(layout.dofCount*layout.band)};
            for(let i=0;i<layout.dofCount;i++)chain.hessian[i*layout.band]=1;
            const fixed=new Uint8Array(layout.dofCount).fill(1);fixed[p]=0;
            const gradient=new Float64Array(layout.dofCount);gradient[p]=force;
            const result=solveSharedAxisLinear(createSharedAxisLinear(layout,[row]),chain,{rows:[row],gradient,fixed,tolerance:1e-10,reuseConstraintWork:true,reuseMatrixAssembly:true});
            assert.ok(result.converged,result.failure);
            if(deferred)assert.equal(calls,force>1?1:0);
            results.push({increment:Array.from(result.increment),dual:Array.from(result.multiplierIncrement),factors:result.factorizations});
        }
        assert.deepEqual(results[1],results[0]);
    }
});

test('deferred rows own their pose, and each changed direction rechecks separation',()=>{
    const {layout,p,definition,input,row}=fixture(.4),exact={gap:row.gap,jacobian:row.jacobian.slice()};
    deferSharedAxisContact(row,definition.evaluate,input,definition.evaluate.inactiveClearance(input));
    input.a[0]=input.b[0]=.001;input.state.origin[0]=10;
    const direction=new Float64Array(layout.dofCount);direction[p]=.1;
    assert.equal(separatedSharedAxisContactDirection(row,direction),true);
    direction[p]=-10;
    assert.equal(separatedSharedAxisContactDirection(row,direction),false);
    assert.equal(row.gap,exact.gap);assert.deepEqual(row.jacobian,exact.jacobian);
    assert.throws(()=>deferSharedAxisContact({...row,multiplier:1},definition.evaluate,input,1));
});

test('row-bank reuse and restored poses never retain a deferred Jacobian for a loaded contact',()=>{
    const {geometry}=fixture();
    const s=createSharedAxisNative({tools:[{id:'wire',insertion:20}],samplePosition:x=>[2,x,0]});
    const edge=2,dofs=[s.layout.positions[edge],s.layout.positions[edge+1]].flatMap(p=>[p,p+1,p+2]);
    const def=createSharedAxisVesselWitness({fallbackGeometry:geometry},{kind:'wall',edge,id:'loaded',dofs,witness:{face:0,t:.5}});
    extendSharedAxisNativeRows(s,[def]);const index=s.definitions.length-1;
    const bank=createSharedAxisConstraintRowPool().acquire(),options={rowStorage:bank,cullInactiveContacts:true};
    assembleSharedAxisNative(s,options);const saved=captureSharedAxisNative(s);
    const dx=new Float64Array(s.layout.dofCount);dx[dofs[0]]=.01;
    applySharedAxisNativeIncrement(s,dx,new Float64Array(s.definitions.length));
    const deferred=assembleSharedAxisNative(s,options).rows[index];assert.ok(deferred.jacobian.every(v=>v===0));
    restoreSharedAxisNative(s,saved);s.multipliers[index]=2;
    const actual=assembleSharedAxisNative(s,options).rows[index],gradient=s.chain.gradient.slice();
    const expected={...assembleSharedAxisNative(s).rows[index],extraForceDofs:undefined,extraForceJacobian:undefined};
    assert.deepEqual(actual,expected);assert.deepEqual(s.chain.gradient,gradient);
    materializeSharedAxisContact(actual);assert.deepEqual(actual,expected);
});

test('protected deferred banks and exact cache entries survive later trials and restore independently',()=>{
    const {geometry}=fixture();
    const s=createSharedAxisNative({tools:[{id:'wire',insertion:20}],samplePosition:x=>[2,x,0]});
    const edge=2,dofs=[s.layout.positions[edge],s.layout.positions[edge+1]].flatMap(p=>[p,p+1,p+2]);
    const def=createSharedAxisVesselWitness({fallbackGeometry:geometry},{kind:'wall',edge,id:'protected',dofs,witness:{face:0,t:.5}});
    extendSharedAxisNativeRows(s,[def]);s.cacheMechanicalAssembly=true;
    const pool=createSharedAxisConstraintRowPool(),initial=assembleSharedAxisNative(s,{rowStorage:pool.acquire(),cullInactiveContacts:true});
    const dx=new Float64Array(s.layout.dofCount);dx[dofs[0]]=.01;
    const change=()=>applySharedAxisNativeIncrement(s,dx,new Float64Array(s.definitions.length));
    const measures=[],references=[],poses=[];
    for(let i=0;i<2;i++) {
        change();poses.push(captureSharedAxisNative(s));
        const bank=pool.acquire([initial.rows,...measures.map(m=>m.rows)]);
        measures.push(assembleSharedAxisNative(s,{rowStorage:bank,cullInactiveContacts:true}));
        const e=def.evaluate({a:s.positions[edge],b:s.positions[edge+1],state:s,radius:s.materials[0].body.radius,needHessian:false});
        references.push({gap:e.gap,jacobian:e.jacobian.slice()});
    }
    for(const i of [0,1,0]) {
        const r=materializeSharedAxisContact(measures[i].rows.at(-1));
        assert.equal(r.gap,references[i].gap);assert.deepEqual(r.jacobian,references[i].jacobian);
    }
    restoreSharedAxisNative(s,poses[0]);
    const restored=assembleSharedAxisNative(s,{rowStorage:pool.acquire(),cullInactiveContacts:true}).rows.at(-1);
    materializeSharedAxisContact(restored);
    assert.equal(restored.gap,references[0].gap);assert.deepEqual(restored.jacobian,references[0].jacobian);
});
