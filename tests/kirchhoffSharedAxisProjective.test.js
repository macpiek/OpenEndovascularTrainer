import test from 'node:test';
import assert from 'node:assert/strict';
import {Quaternion,Vector3} from 'three';
import {factorProjectiveBand,projectRotation,quaternionColumns} from '../src/physics/projectiveRodMath.js';
import {createSharedAxisNative,captureSharedAxisNative,rotateSharedAxisNative} from '../src/physics/kirchhoffSharedAxisNative.js';
import {iterateSharedAxisProjective} from '../src/physics/kirchhoffSharedAxisProjective.js';
import {createSharedAxisContacts} from '../src/physics/kirchhoffSharedAxisContacts.js';
import {advanceSharedAxis} from '../src/physics/kirchhoffSharedAxisAppSystem.js';
const finish=iterator=>{let n;do{n=iterator.next();}while(!n.done);return n.value;};
const create=(extra={})=>createSharedAxisNative({tools:[{id:'catheter',type:'straight',insertion:80}],...extra});
const close=(a,b,tolerance=1e-8)=>assert.ok(Math.abs(a-b)<=tolerance,`${a} vs ${b}`);

test('PD band solve satisfies a coupled SPD system with exact prescribed vectors and owns its factor',()=>{
    const terms=[{ids:[0,1],coefficients:[-1,1],weight:2},{ids:[1,2],coefficients:[-1,1],weight:3},
        {ids:[2],coefficients:[1],weight:4}];
    const factor=factorProjectiveBand(terms,3,new Map([[0,[2,-1,3]]]));
    const rhs=[[0,0,0],[1,2,3],[4,5,6]],x=factor.solve(rhs);
    assert.deepEqual(x[0],[2,-1,3]);
    for(let k=0;k<3;k++){close(5*x[1][k]-3*x[2][k],rhs[1][k]+2*x[0][k]);close(-3*x[1][k]+7*x[2][k],rhs[2][k]);}
    factorProjectiveBand([{ids:[0],coefficients:[1],weight:100}],1,new Map());
    assert.deepEqual(factor.solve(rhs),x);
    const other=factor.solve([[0,0,0],[0,0,0],[0,0,0]]);assert.notDeepEqual(other,x);
});

test('scratch Jacobi keeps loaded contact poses, material frames and dynamic histories bit-identical',()=>{
    const states=[0,1].map(()=>create({tools:[{id:'wire',type:'glidewire',insertion:80},{id:'catheter',type:'pigtail',insertion:65}],
        wallSample:({b})=>({gap:.5-b[1],jacobian:[0,0,0,0,-1,0]})}));
    for(const s of states){s.loads[s.layout.positions.at(-1)+1]=1000;rotateSharedAxisNative(s,'catheter',.3);}
    for(let step=0;step<6;step++) {
        const results=states.map((s,i)=>finish(iterateSharedAxisProjective(s,1/60,{projective:{...strictClosure,reuseGeometry:true,scratchRotations:i===1}})));
        assert.ok(results.every(r=>r.converged),JSON.stringify(results));
        const snapshots=states.map(captureSharedAxisNative);
        for(const key of ['positions','frames','multipliers'])assert.deepEqual(snapshots[0][key],snapshots[1][key]);
        for(const key of ['velocities','angularVelocities'])assert.deepEqual(states[0][key],states[1][key]);
        assert.deepEqual(results[0].quality,results[1].quality);
    }
});

test('local projection recovers proper rotations including pi and distorted directors',()=>{
    for(const angle of [0,.4,2.5,Math.PI]) {
        const q=new Quaternion().setFromAxisAngle(new Vector3(1,2,3).normalize(),angle),columns=quaternionColumns(q);
        const projected=projectRotation(columns.map((c,i)=>c.map(v=>v*(i+1))));
        const recovered=new Quaternion(projected.quaternion.x,projected.quaternion.y,projected.quaternion.z,projected.quaternion.w);
        close(Math.abs(q.dot(recovered)),1);
    }
    const reflection=projectRotation([[1,0,0],[0,1,0],[0,0,-1]]).columns;
    close(new Vector3(...reflection[0]).cross(new Vector3(...reflection[1])).dot(new Vector3(...reflection[2])),1);
});

test('a straight unloaded rod stays at rest with one factorization for several local/global iterations',()=>{
    const s=create(),saved=captureSharedAxisNative(s),r=finish(iterateSharedAxisProjective(s,1/60));
    assert.equal(r.converged,true);assert.equal(r.factorizations,1);assert.ok(r.iterations>=4);
    s.positions.forEach((p,i)=>p.forEach((v,k)=>close(v,saved.positions[i][k])));
    assert.ok(r.pd.maxLengthError<1e-10);assert.ok(r.pd.maxShear<1e-10);
    assert.equal(r.certificateBound,undefined,'PD does not claim a Newton residual certificate');
});

test('intrinsic curved tip and prescribed proximal rotation rotate the entire free solution covariantly',()=>{
    const outputs=[];
    for(const angle of [0,Math.PI/2]) {
        const s=create({tools:[{id:'catheter',type:'berenstein',insertion:70,shaftStiffness:2,tipStiffness:2}]});
        // A freely spun initial material frame rotates the preferred bend plane.
        const axis=new Vector3(1,0,0),rotation=new Quaternion().setFromAxisAngle(axis,angle);
        for(const {body,last} of s.materials)for(let e=0;e<last;e++) {
            const q=new Quaternion(body.orientationX[e],body.orientationY[e],body.orientationZ[e],body.orientationW[e]).premultiply(rotation);
            for(const c of ['X','Y','Z','W'])body['orientation'+c][e]=q[c.toLowerCase()];
        }
        for(let i=0;i<30;i++){const r=finish(iterateSharedAxisProjective(s,1/60));assert.equal(r.converged,true,JSON.stringify(r));}
        outputs.push(s.positions.at(-1));
    }
    assert.ok(Math.hypot(outputs[0][1],outputs[0][2])>.05,'curved profile must produce bending');
    const rotated=new Vector3(...outputs[0]).applyAxisAngle(new Vector3(1,0,0),Math.PI/2).toArray();
    rotated.forEach((v,k)=>close(v,outputs[1][k],1e-5));
});

test('proximal twist propagates to distal material frames without a Newton call',()=>{
    const s=create({tools:[{id:'catheter',type:'straight',insertion:80,shaftStiffness:40.65,tipStiffness:66.8}]});rotateSharedAxisNative(s,'catheter',.2);
    const {body}=s.materials[0],e=body.segmentCount-1,old=new Quaternion(body.orientationX[e],body.orientationY[e],body.orientationZ[e],body.orientationW[e]);
    for(let i=0;i<60;i++)assert.equal(finish(iterateSharedAxisProjective(s,1/60)).converged,true);
    const current=new Quaternion(body.orientationX[e],body.orientationY[e],body.orientationZ[e],body.orientationW[e]);
    assert.ok(old.angleTo(current)>.001,'distal frame should respond to proximal twist');
});

test('PD contact projection limits wall penetration and keeps fixed nodes immobile',()=>{
    const s=create({wallSample:({b})=>({gap:.5-b[1],jacobian:[0,0,0,0,-1,0]})});
    s.loads[s.layout.positions.at(-1)+1]=1000;
    for(let i=2;i<s.positions.length;i++)s.positions[i][1]=.55;
    s.geometryKey=Symbol('initial-contact');
    const before=s.positions.slice(0,2).map(p=>p.slice());let contactSeen=false;
    for(let i=0;i<30;i++) {
        const r=finish(iterateSharedAxisProjective(s,1/60));assert.equal(r.converged,true,JSON.stringify(r));
        contactSeen ||= Math.max(...s.multipliers)>0;
        assert.ok(r.quality.maxPenetration<=.1);assert.deepEqual(s.positions.slice(0,2),before);
    }
    assert.ok(s.positions.at(-1)[1]>.4);assert.ok(contactSeen);
});

test('cancelling or rejecting a PD step restores positions, frames and dynamic history',()=>{
    const s=create({tools:[{id:'catheter',type:'berenstein',insertion:70}]});
    s.velocities=s.positions.map(()=>[0,.2,0]);s.wallFrictionHistory=[{id:'saved'}];
    const before=captureSharedAxisNative(s),velocities=s.velocities,history=s.wallFrictionHistory;
    const iterator=iterateSharedAxisProjective(s,1/60);assert.equal(iterator.next().done,false);iterator.return();
    assert.deepEqual(captureSharedAxisNative(s),before);assert.equal(s.velocities,velocities);assert.equal(s.wallFrictionHistory,history);assert.equal(s.dynamicStep,null);
    const rejected=finish(iterateSharedAxisProjective(s,1/60,{projective:{maxLengthError:1e-14}}));
    assert.equal(rejected.converged,false);assert.deepEqual(captureSharedAxisNative(s),before);
});

test('PD remesh/feed keeps shared wire and catheter state finite and physical endpoints exact',()=>{
    let s=create({adaptiveMesh:true,startCoordinate:-40,tools:[{id:'wire',insertion:0},{id:'catheter',type:'straight',insertion:0}]});
    let rotations={wire:0,catheter:0};
    for(const insertion of [1,6,14,25]) {
        const tools=s.materials.map(m=>({...m.spec,insertion:insertion*(m.spec.id==='wire'?1:.8),rotation:0}));
        const r=finish(advanceSharedAxis(s,rotations,1/60,tools,{projectiveDynamics:true}));
        assert.ok(r.state,JSON.stringify(r.result));s=r.state;rotations=r.rotations;
        assert.equal(r.result.quality.finite,true);assert.equal(r.result.quality.interToolRows,0);
        for(const m of s.materials)close(m.coordinates.at(-1),tools.find(t=>t.id===m.spec.id).insertion);
    }
});


test('a stiff pigtail inside the sheath retains bounded shear at a moving sub-millimetre tip',()=>{
    const contacts=createSharedAxisContacts({sheath:{start:[0,0,0],end:[110,0,0],innerRadius:.9}});
    let s=createSharedAxisNative({...contacts,adaptiveMesh:true,tools:[
        {id:'wire',type:'glidewire',insertion:0,shaftStiffness:9.6,tipStiffness:6.8},
        {id:'catheter',type:'pigtail',insertion:0,shaftStiffness:40.65,tipStiffness:66.8,radius:5/6,mass:1.75}]});
    let rotations={wire:0,catheter:0};
    for(const catheter of [0,.1083333,.8666667,1.733333,5.01]) {
        const tools=s.materials.map(m=>({...m.spec,insertion:m.spec.id==='wire'?20:catheter,rotation:0}));
        const r=finish(advanceSharedAxis(s,rotations,1/60,tools,{projectiveDynamics:true}));
        assert.ok(r.state,JSON.stringify(r.result));assert.ok(r.result.pd.maxShear<=.1);
        s=r.state;rotations=r.rotations;
    }
});


test('a released contact clears both the stored reaction and published contact count',()=>{
    const s=create({wallSample:()=>({gap:100,jacobian:[0,0,0,0,-1,0]})});
    s.definitions.forEach((r,i)=>{if(r.kind==='wall')s.multipliers[i]=10;});
    const r=finish(iterateSharedAxisProjective(s,1/60));assert.equal(r.converged,true);
    assert.equal(r.quality.contacts,0);assert.ok(s.multipliers.every(v=>v===0));
});

const strictClosure={hardClosure:true,maxLengthError:1e-5,maxPenetration:1e-5,maxShear:1e-5};
test('PD hard closure corrects penalty stretch and wall penetration without moving prescribed nodes',()=>{
    const s=create({wallSample:({b})=>({gap:.5-b[1],jacobian:[0,0,0,0,-1,0]})});
    s.loads[s.layout.positions.at(-1)+1]=1000;
    for(let i=2;i<s.positions.length;i++)s.positions[i][1]=.55;
    s.geometryKey=Symbol('initial-contact');
    const fixed=s.positions.slice(0,2).map(p=>p.slice());let corrected=false;
    for(let i=0;i<10;i++) {
        const r=finish(iterateSharedAxisProjective(s,1/60,{projective:strictClosure}));
        assert.equal(r.converged,true,JSON.stringify(r));assert.deepEqual(s.positions.slice(0,2),fixed);
        assert.ok(r.quality.maxPenetration<=1e-5);assert.ok(r.pd.maxLengthErrorMm<=1e-5);
        assert.ok(r.pd.maxLengthError<=1e-5);assert.ok(r.pd.maxShear<=1e-5);
        corrected ||= r.pd.closureCorrectionMm>.01&&r.pd.preClosureShear>1e-5;
        assert.equal(r.pd.localGlobalConverged,false,'post-projection pose is not a converged PD equilibrium');
        assert.equal(r.pd.equilibriumCertified,false);assert.equal(r.certificateBound,undefined);
    }
    assert.ok(corrected);assert.ok(s.positions.at(-1)[1]>.4,'closure must preserve the loaded response');
});

test('hard closure retains preferred tip curvature and exact rest geometry',()=>{
    const s=create({tools:[{id:'catheter',type:'berenstein',insertion:70,shaftStiffness:2,tipStiffness:2}]});
    for(let i=0;i<30;i++)assert.equal(finish(iterateSharedAxisProjective(s,1/60,{projective:strictClosure})).converged,true);
    assert.ok(Math.hypot(s.positions.at(-1)[1],s.positions.at(-1)[2])>.05);
    for(let i=0;i+1<s.positions.length;i++)close(new Vector3(...s.positions[i]).distanceTo(new Vector3(...s.positions[i+1])),s.coordinates[i+1]-s.coordinates[i],1e-5);
});

test('cancelling during hard closure restores pose and accepted dynamic history',()=>{
    const s=create();s.positions.at(-1)[1]=2;s.geometryKey=Symbol('closure-cancellation');
    s.velocities=s.positions.map(()=>[0,.2,0]);s.wallFrictionHistory=[{id:'saved'}];
    const before=captureSharedAxisNative(s),velocities=s.velocities,history=s.wallFrictionHistory;
    const iterator=iterateSharedAxisProjective(s,1/60,{projective:{...strictClosure,iterations:1}});
    assert.equal(iterator.next().value.method,'projective-dynamics');
    const closure=iterator.next();assert.equal(closure.done,false);assert.notEqual(closure.value.method,'projective-dynamics');
    iterator.return();assert.deepEqual(captureSharedAxisNative(s),before);
    assert.equal(s.velocities,velocities);assert.equal(s.wallFrictionHistory,history);assert.equal(s.dynamicStep,null);
});

test('optional PD cost profile measures work without changing the accepted trajectory',()=>{
    const states=[create(),create()];
    for(const s of states)s.loads[s.layout.positions.at(-1)+1]=1000;
    for(let step=0;step<4;step++) {
        const results=states.map((s,i)=>finish(iterateSharedAxisProjective(s,1/60,{projective:{...strictClosure,profile:i===1}})));
        assert.equal(results[0].converged,true);assert.equal(results[1].converged,true);
        const snapshots=states.map(captureSharedAxisNative);
        for(const key of ['positions','frames','multipliers'])assert.deepEqual(snapshots[0][key],snapshots[1][key]);
        assert.deepEqual(states[0].velocities,states[1].velocities);
        assert.deepEqual(results[0].quality,results[1].quality);
        assert.equal(results[0].timings.rotationMs,undefined);
        for(const k of ['setupMs','constraintMs','rotationMs','closureSolveMs'])assert.ok(results[1].timings[k]>=0);
        assert.ok(results[1].timings.constraintCalls>=results[1].iterations*2);
    }
});

test('reusing unchanged PD geometry preserves contact, loaded poses and dynamic history exactly',()=>{
    const states=[false,true].map(()=>create({wallSample:({b})=>({gap:.5-b[1],jacobian:[0,0,0,0,-1,0]})}));
    for(const s of states){s.loads[s.layout.positions.at(-1)+1]=1000;for(let i=2;i<s.positions.length;i++)s.positions[i][1]=.55;s.geometryKey=Symbol('contact');}
    for(let step=0;step<10;step++) {
        const results=states.map((s,i)=>finish(iterateSharedAxisProjective(s,1/60,{projective:{...strictClosure,profile:true,reuseGeometry:i===1}})));
        assert.ok(results.every(r=>r.converged));
        const snapshots=states.map(captureSharedAxisNative);
        for(const key of ['positions','frames','multipliers'])assert.deepEqual(snapshots[0][key],snapshots[1][key]);
        assert.deepEqual(states[0].velocities,states[1].velocities);assert.deepEqual(results[0].quality,results[1].quality);
        assert.ok(results[1].timings.constraintReuse>0);assert.ok(results[1].timings.constraintCalls<results[0].timings.constraintCalls);
    }
});

test('fast PD rotations preserve curved-tip response with strict geometry and fixed inlet',()=>{
    const states=[false,true].map(()=>create({tools:[{id:'catheter',type:'berenstein',insertion:70,shaftStiffness:2,tipStiffness:2}]}));
    const fixed=states[0].positions[0].slice();
    for(let step=0;step<30;step++) {
        const results=states.map((s,i)=>finish(iterateSharedAxisProjective(s,1/60,{projective:{...strictClosure,fastRotations:i===1,reuseGeometry:i===1}})));
        assert.ok(results.every(r=>r.converged));
        states[0].positions.forEach((p,i)=>p.forEach((v,k)=>close(v,states[1].positions[i][k],1e-7)));
        assert.deepEqual(states[1].positions[0],fixed);
    }
    assert.ok(Math.hypot(states[1].positions.at(-1)[1],states[1].positions.at(-1)[2])>.05);
});
