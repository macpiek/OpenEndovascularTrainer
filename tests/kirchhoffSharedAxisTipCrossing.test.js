import test from 'node:test';
import assert from 'node:assert/strict';
import {createSharedAxisNative,feedSharedAxisNative,extendSharedAxisNativeRows,captureSharedAxisNative} from '../src/physics/kirchhoffSharedAxisNative.js';
import {createSharedAxisContacts} from '../src/physics/kirchhoffSharedAxisContacts.js';
import {stepSharedAxis} from '../src/physics/kirchhoffSharedAxisTimeStep.js';

const tools=()=>[{id:'wire',insertion:100,shaftStiffness:39,tipStiffness:30.7,wallStaticFriction:.002,wallKineticFriction:.002},
    {id:'catheter',insertion:99.5,shaftStiffness:58.1,tipStiffness:87,wallStaticFriction:.002,wallKineticFriction:.002}];
const crossings=[99.9,99.99,100,100.01,100.1,100,99.99,99.9999,100,100.0001,99.9999];

function advance(s,insertion,prepare=()=>{}) {
    const before=captureSharedAxisNative(s),c=feedSharedAxisNative(s,{catheter:insertion});prepare(c);
    assert.deepEqual(captureSharedAxisNative(s),before,'Preparing the tip crossing must preserve the accepted state');
    assert.equal(c.materials[0].spec.insertion,100);assert.equal(c.materials[1].spec.insertion,insertion);
    assert.equal(c.materials[0].coordinates.at(-1),100);assert.equal(c.materials[1].coordinates.at(-1),insertion);
    const result=stepSharedAxis(c,1/60,{maxIterations:60,feedById:{catheter:insertion-s.materials[1].spec.insertion}});
    assert.ok(result.converged,`catheter ${insertion}: ${JSON.stringify(result)}`);
    assert.ok(result.certificateBound<=1e-6);assert.equal(result.interToolRows,0);return c;
}

test('two physical tips cross inside the sheath without merging or shifting either material endpoint',()=>{
    const sheath={start:[0,0,0],end:[130,0,0],innerRadius:.9,proximalExtension:40};
    let s=createSharedAxisNative({...createSharedAxisContacts({sheath}),tools:tools(),maxBendAngle:Math.PI/4});
    assert.ok(stepSharedAxis(s,1/60).converged);
    for(const insertion of crossings)s=advance(s,insertion);
});

// A mild curved wall, with a real normal reaction and external wall friction.
// Its capsule endpoint witnesses have exact analytic signed gap/J/H. The
// polygonal initial arc makes the two fixed proximal nodes exactly 5 mm apart.
const sphereRadius=1000,axisRadius=sphereRadius-.8,angle=2*Math.asin(5/(2*axisRadius));
function addSphereContacts(s) {
    const definitions=[];
    for(let e=1;e<s.coordinates.length-1;e++) {
        const id=`sphere/${s.coordinates[e]}/${s.coordinates[e+1]}`;
        if(s.definitions.some(d=>d.id===id))continue;
        definitions.push({kind:'wall',edge:e,id,witness:{face:0,t:1},
            dofs:[s.layout.positions[e],s.layout.positions[e+1]].flatMap(i=>[i,i+1,i+2]),
            evaluate:({b,radius,state})=>{
                const delta=[b[0]+state.origin[0],b[1]+state.origin[1]-sphereRadius,b[2]+state.origin[2]],length=Math.hypot(...delta),normal=delta.map(v=>-v/length),hessian=new Float64Array(36);
                for(let i=0;i<3;i++)for(let j=0;j<3;j++)hessian[(i+3)*6+j+3]=-((i===j?1:0)-normal[i]*normal[j])/length;
                return {gap:sphereRadius-length-radius,jacobian:[0,0,0,...normal],hessian};
            }});
    }
    extendSharedAxisNativeRows(s,definitions);s.loads.fill(0);
    for(let n=2;n<s.positions.length;n++)s.loads[s.layout.positions[n]+1]=-20;
}

test('loaded curved wall permits the catheter tip to pass the wire tip through a 0.01 mm cell',()=>{
    let s=createSharedAxisNative({rebaseNearTips:true,startCoordinate:-40,maxBendAngle:Math.PI/4,tools:tools(),
        samplePosition:x=>[axisRadius*Math.sin(x/5*angle),sphereRadius-axisRadius*Math.cos(x/5*angle),0]});
    addSphereContacts(s);const initial=stepSharedAxis(s,1/60);assert.ok(initial.converged,JSON.stringify(initial));
    assert.ok(initial.friction.contacts>0);
    for(const insertion of crossings)s=advance(s,insertion,addSphereContacts);
});
