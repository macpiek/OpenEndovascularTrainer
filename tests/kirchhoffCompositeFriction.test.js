import assert from 'node:assert/strict';
import test from 'node:test';
import {evaluateCompositeFriction,measureCompositeFriction,createCompositeFrictionEquationWorkspace,evaluateCompositeFrictionEquation} from '../src/physics/kirchhoffCompositeFriction.js';
import {projectKirchhoffSurfaceFriction} from '../src/physics/kirchhoffSurfaceFriction.js';

const close=(a,b,t=1e-8)=>assert.ok(Math.abs(a-b)<=t*(1+Math.max(Math.abs(a),Math.abs(b))),`${a} != ${b}`);
const identity=[1,0,0,1];
const tolerances={slipTolerance:1e-9,coneTolerance:1e-9,workTolerance:1e-9};

test('isotropic sliding gives the analytic maximum-dissipation force and augmented potential',()=>{
    const slip=[.2,.1],radius=.6,penalty=10,length=Math.hypot(...slip);
    const r=evaluateCompositeFriction({slip,traction:[0,0],normalForce:2,mu:[.3,.3],penalty,jacobian:identity,dofCount:2});
    close(r.energy,radius*length-radius**2/(2*penalty));
    slip.forEach((v,i)=>close(r.trialTraction[i],-radius*v/length));
    close(r.gradient[0],radius*slip[0]/length);close(r.gradient[1],radius*slip[1]/length);
    const measured=measureCompositeFriction({...tolerances,slip,traction:r.trialTraction,normalForce:2,mu:[.3,.3]});
    assert.ok(measured.converged);close(measured.work,-radius*length);
});

for(const sliding of [false,true]) test(`anisotropic local operator matches independent finite differences of its energy and gradient (${sliding?'slide':'stick'})`,()=>{
    const J=[1,.2,-.1,.3, .1,-.4,.7,.2],n=4,z=[0,0,0,0];
    const slip0=sliding?[.23,-.17]:[.001,-.002];
    const evaluate=()=>evaluateCompositeFriction({traction:[.03,-.04],slip:[0,1].map(i=>slip0[i]+z.reduce((s,v,j)=>s+J[i*n+j]*v,0)),
        normalForce:2,mu:[.2,.5],penalty:7,jacobian:J,dofCount:n});
    const r=evaluate(),eps=1e-6;
    for(let i=0;i<n;i++) {
        z[i]=eps;const plus=evaluate();z[i]=-eps;const minus=evaluate();z[i]=0;
        close(r.gradient[i],(plus.energy-minus.energy)/(2*eps),1e-8);
        for(let j=0;j<n;j++)close(r.hessian[j*n+i],(plus.gradient[j]-minus.gradient[j])/(2*eps),1e-7);
    }
});

test('independent material slide and spins produce opposite forces and torques with the actual lever arm',()=>{
    const radius=.4445,J=[1,-1,0,0, 0,0,radius,-radius];
    const r=evaluateCompositeFriction({traction:[0,0],slip:[.2,.1],normalForce:3,mu:[.015,.006],penalty:8,jacobian:J,dofCount:4});
    close(r.gradient[0]+r.gradient[1],0);close(r.gradient[2]+r.gradient[3],0);
    close(r.gradient[2],-radius*r.trialTraction[1]);
    const virtual=[.3,-.2,.7,-.1],surface=[virtual[0]-virtual[1],radius*(virtual[2]-virtual[3])];
    close(r.gradient.reduce((s,v,i)=>s+v*virtual[i],0),-r.trialTraction[0]*surface[0]-r.trialTraction[1]*surface[1]);
});

test('zero physical load creates no friction, even with stale trial traction; original KKT rejects that stale force',()=>{
    const args={traction:[.3,-.2],slip:[.1,.3],normalForce:0,mu:[.2,.4],penalty:7,jacobian:identity,dofCount:2};
    const r=evaluateCompositeFriction(args);
    assert.deepEqual(Array.from(r.trialTraction),[0,0]);assert.ok(r.gradient.every(v=>v===0));assert.ok(r.hessian.every(v=>v===0));
    assert.equal(measureCompositeFriction({...args,...tolerances}).converged,false);
    assert.equal(measureCompositeFriction({...args,traction:[0,0],...tolerances}).converged,true);
});

test('original maximum dissipation is independent of numerical penalty and rejects wrong-direction or over-cone traction',()=>{
    const normalForce=3,mu=[.1,.4],slip=[.2,-.3],norm=Math.hypot(mu[0]*slip[0],mu[1]*slip[1]);
    const traction=slip.map((v,i)=>-normalForce*mu[i]**2*v/norm),args={traction,slip,normalForce,mu};
    assert.ok(measureCompositeFriction({...args,...tolerances}).converged);
    for(const penalty of [.1,1,100]) {
        const r=evaluateCompositeFriction({...args,penalty,jacobian:identity,dofCount:2});
        traction.forEach((v,i)=>close(v,r.trialTraction[i],1e-9));
    }
    assert.equal(measureCompositeFriction({...args,traction:traction.map(v=>-v),...tolerances}).converged,false);
    assert.equal(measureCompositeFriction({...args,traction:traction.map(v=>1.1*v),...tolerances}).converged,false);
});

test('zero coefficient disables only its friction direction and missing physical slip data never becomes zero',()=>{
    const args={traction:[0,0],slip:[.1,.2],normalForce:2,mu:[0,.3],penalty:10,jacobian:identity,dofCount:2};
    const r=evaluateCompositeFriction(args);close(r.trialTraction[0],0);close(r.trialTraction[1],-.6);
    close(r.gradient[0],0);close(r.hessian[0],0);
    assert.throws(()=>evaluateCompositeFriction({...args,normalForce:-1}),/normal force/);
    assert.throws(()=>evaluateCompositeFriction({...args,slip:undefined}),/material slip/);
});

const equation=args=>evaluateCompositeFrictionEquation(args,createCompositeFrictionEquationWorkspace());
const same=(a,b,t=1e-10)=>{assert.equal(a.length,b.length);a.forEach((v,i)=>close(v,b[i],t));};
function equationInput(mu=[.015,.006],sliding=true) {
    return {traction:sliding?[.035,-.012]:mu.map(v=>v?2*v*.15:.03),slip:sliding?[.17,-.11]:[0,0],
        normalForce:2,mu,penalty:7};
}
function independentResidual(args) {
    const z=args.traction.map((v,i)=>v-args.penalty*args.slip[i]);
    const p=projectKirchhoffSurfaceFriction(z,Math.max(args.normalForce,0),args.mu).lambda;
    return args.traction.map((v,i)=>(v-p[i])/args.penalty);
}

for(const mu of [[.3,.3],[.2,.5],[.015,.006],[.006,.006],[0,.3],[.3,0],[0,0]])for(const sliding of [false,true])
test(`friction equations full slip/Ft/Fn derivatives match FD for ${mu} ${sliding?'slide':'stick'}`,()=>{
    const args=equationInput(mu,sliding),out=equation(args),h=1e-6;
    assert.ok(out.valid&&out.operatorReady);assert.equal(out.certified,false);assert.equal(out.unit,'mm');
    same(out.residual,independentResidual(args),0);
    for(const [input,matrix] of [['slip','slipJacobian'],['traction','tractionJacobian']])for(let j=0;j<2;j++){
        const p=structuredClone(args),m=structuredClone(args);p[input][j]+=h;m[input][j]-=h;
        const rp=independentResidual(p),rm=independentResidual(m);
        for(let i=0;i<2;i++)close(out[matrix][2*i+j],(rp[i]-rm[i])/(2*h),2e-8);
    }
    const rp=independentResidual({...args,normalForce:args.normalForce+h}),rm=independentResidual({...args,normalForce:args.normalForce-h});
    for(let i=0;i<2;i++)close(out.normalDerivative[i],(rp[i]-rm[i])/(2*h),2e-8);
    for(let i=0;i<4;i++)close(out.tractionJacobian[i],((i===0||i===3?1:0)-out.slipJacobian[i])/args.penalty,1e-15);
    close(out.slipJacobian[1],out.slipJacobian[2],0);
    for(let i=0;i<2;i++)if(mu[i]===0){assert.ok(out.projectedForce[i]===0);assert.ok(out.normalDerivative[i]===0);assert.ok(out.slipJacobian[2*i]===0&&out.slipJacobian[2*i+1]===0);}
});

test('positive-load homogeneity and actual coefficient conditioning persist down to representable tiny loads',()=>{
    for(const mu of [[.015,.006],[.006,.006]])for(const normalForce of [3,1e-6,1e-12,1e-40,1e-100]){
        const args={traction:[0,0],slip:[-.2,.3],normalForce,mu,penalty:7},out=equation(args),delta=normalForce*1e-3;
        const plus=equation({...args,normalForce:normalForce+delta}),minus=equation({...args,normalForce:normalForce-delta});
        for(let i=0;i<2;i++)close(out.normalDerivative[i],(plus.residual[i]-minus.residual[i])/(2*delta),2e-8);
        const D=out.slipJacobian,z=out.trialForce,dn=Array.from(out.normalDerivative,v=>-args.penalty*v);
        same(dn,[(out.projectedForce[0]-D[0]*z[0]-D[1]*z[1])/normalForce,(out.projectedForce[1]-D[2]*z[0]-D[3]*z[1])/normalForce],1e-13);
        assert.ok(Math.hypot(...dn)<=Math.max(...mu)*(1+1e-12));
        const trace=D[0]+D[3],disc=Math.hypot(D[0]-D[3],2*D[1]);
        assert.ok((trace-disc)/2>=-1e-14);assert.ok((trace+disc)/2<=1+1e-14);
        if(normalForce<=1e-12){
            const weighted=Math.hypot(mu[0]*z[0],mu[1]*z[1]);
            same(dn,mu.map((v,i)=>v*v*z[i]/weighted),2e-12);
        }
    }
});

test('Fn=0 uses the exact right load derivative, including disabled axes and directions',()=>{
    for(const [mu,z] of [[[.015,.006],[.4,-.7]],[[.3,.3],[.4,-.7]],[[0,.3],[.4,-.7]],[[.3,0],[.4,-.7]],[[0,.3],[.4,0]],[[0,0],[.4,-.7]]]){
        const args={traction:[0,0],slip:z.map(v=>-v/7),normalForce:0,mu,penalty:7},out=equation(args),h=1e-7;
        assert.ok(out.projectedForce.every(v=>v===0));assert.ok(out.slipJacobian.every(v=>v===0));
        same(out.tractionJacobian,[1/7,0,0,1/7],0);
        const weighted=Math.hypot(mu[0]*z[0],mu[1]*z[1]);
        const expected=mu.map((v,i)=>weighted?-v*v*z[i]/weighted/7:0);
        same(out.normalDerivative,expected,1e-15);
        const right=equation({...args,normalForce:h});
        for(let i=0;i<2;i++)close(out.normalDerivative[i],(right.residual[i]-out.residual[i])/h,2e-8);
        const left=equation({...args,normalForce:-h});assert.ok(left.normalDerivative.every(v=>v===0));
        assert.equal(out.branch,'zero-load-right-derivative');
    }
});

test('signed private release leaves Fn/Ft unchanged and cannot supply a physical cone/work certificate',()=>{
    const args={traction:[.3,-.2],slip:[.1,.3],normalForce:-.01,mu:[.2,.4],penalty:7},before=structuredClone(args),out=equation(args);
    assert.deepEqual(args,before);assert.equal(out.normalForce,-.01);assert.equal(out.projectionLoad,0);
    assert.equal(out.branch,'negative-load-private-extension');assert.equal(out.certified,false);
    same(out.residual,args.traction.map(v=>v/7),0);assert.ok(out.projectedForce.every(v=>v===0));
    assert.ok(out.slipJacobian.every(v=>v===0));assert.ok(out.normalDerivative.every(v=>v===0));same(out.tractionJacobian,[1/7,0,0,1/7],0);
    const h=1e-5,plus=equation({...args,normalForce:args.normalForce+h}),minus=equation({...args,normalForce:args.normalForce-h});
    same(plus.residual,minus.residual,0);
    assert.throws(()=>measureCompositeFriction({...args,...tolerances}),/normal load/);
    const zero={...args,normalForce:0,penalty:1e12},tiny=equation(zero);
    assert.ok(Math.hypot(...tiny.residual)<1e-12);assert.equal(tiny.certified,false);
    assert.equal(measureCompositeFriction({...zero,...tolerances}).converged,false,'small scaled equation cannot hide stale zero-load traction');
    const released={...zero,traction:[0,0]};assert.ok(equation(released).residual.every(v=>v===0));
    assert.equal(measureCompositeFriction({...released,...tolerances}).converged,true);
});

test('zero apex and stick-boundary selections are explicit, bounded and do not smooth the cone',()=>{
    const args={traction:[0,0],slip:[0,0],normalForce:0,mu:[.015,.006],penalty:7},apex=equation(args);
    assert.equal(apex.branch,'zero-load-apex');assert.ok(apex.projectedForce.every(v=>v===0));
    assert.ok(apex.normalDerivative.every(v=>v===0));assert.ok(apex.slipJacobian.every(v=>v===0));
    same(apex.tractionJacobian,[1/7,0,0,1/7],0);
    for(const t of [1e-3,1e-8,1e-40]){
        const ray={...args,normalForce:t,traction:[.01*t,-.02*t],slip:[.03*t,.04*t]},out=equation(ray);
        assert.ok(Math.hypot(...out.residual)<=Math.hypot(...ray.traction)/7+t*Math.max(...args.mu)/7);
        assert.ok(out.slipJacobian.every(v=>Math.abs(v)<=1));assert.ok(Math.hypot(...out.normalDerivative)<=Math.max(...args.mu)/7*(1+1e-12));
    }
    for(const mu of [[.3,.3],[.2,.5],[.3,0]]){
        const out=equation({traction:[2*mu[0],0],slip:[0,0],normalForce:2,mu,penalty:7});
        assert.equal(out.slipJacobian[0],1);assert.ok(out.normalDerivative.every(v=>v===0));
    }
});

test('constitutive roots match original KKT and maximum work for all numerical penalties without accepting invalid traction',()=>{
    for(const mu of [[.3,.3],[.015,.006],[0,.3],[.3,0],[0,0]])for(const slip of [[.2,-.3],[0,0]]){
        const normalForce=2,weighted=Math.hypot(mu[0]*slip[0],mu[1]*slip[1]);
        const traction=weighted?mu.map((v,i)=>-normalForce*v*v*slip[i]/weighted):mu.map(v=>normalForce*v*.1);
        const args={traction,slip,normalForce,mu};assert.ok(measureCompositeFriction({...args,...tolerances}).converged);
        for(const penalty of [.01,.1,1,100,1e4]){
            const out=equation({...args,penalty});same(out.projectedForce,traction,2e-12);assert.ok(Math.hypot(...out.residual)<1e-10);
        }
        if(weighted){const wrong={...args,traction:traction.map(v=>-v)};
            assert.equal(measureCompositeFriction({...wrong,...tolerances}).converged,false);
            assert.ok(Math.hypot(...equation({...wrong,penalty:7}).residual)>1e-5);
        }
    }
});

test('equation workspace owns reusable outputs and any malformed or unrepresentable input revokes validity',()=>{
    const out=createCompositeFrictionEquationWorkspace(),keys=['residual','slipJacobian','tractionJacobian','normalDerivative','projectedForce','trialForce'];
    assert.equal(out.valid,false);const buffers=keys.map(key=>out[key]),args=equationInput(),before=structuredClone(args);
    for(let i=0;i<3;i++){evaluateCompositeFrictionEquation(args,out);assert.ok(out.valid);keys.forEach((key,i)=>assert.equal(out[key],buffers[i]));}
    assert.deepEqual(args,before);const other=equation(args),saved=other.normalDerivative.slice();out.normalDerivative[0]=123;same(other.normalDerivative,saved,0);
    for(const absent of [undefined,null]){
        evaluateCompositeFrictionEquation(args,out);assert.throws(()=>evaluateCompositeFrictionEquation(absent,out));
        assert.equal(out.valid,false);assert.ok(keys.every(key=>out[key].every(Number.isNaN)));
    }
    for(const bad of [{traction:undefined},{traction:[0,NaN]},{slip:[0]},{mu:[.1,-.1]},{mu:[Infinity,.1]},
        {normalForce:NaN},{normalForce:Infinity},{penalty:0},{penalty:NaN},{penalty:Infinity},
        {traction:[1e308,0],slip:[-1e308,0],penalty:2},{normalForce:1e308,mu:[2,2]},
        {normalForce:Number.MIN_VALUE,mu:[.015,.006]},{penalty:Number.MIN_VALUE}]){
        evaluateCompositeFrictionEquation(args,out);
        assert.throws(()=>evaluateCompositeFrictionEquation({...args,...bad},out));
        assert.equal(out.valid,false);assert.equal(out.operatorReady,false);assert.equal(out.certified,false);
        assert.ok(keys.every(key=>out[key].every(Number.isNaN)));assert.ok(Number.isNaN(out.normalForce));
    }
    evaluateCompositeFrictionEquation({...args,normalForce:0},out);assert.ok(out.valid);assert.ok(out.projectedForce.every(v=>v===0));
    assert.throws(()=>evaluateCompositeFrictionEquation(args,{}),/prepared/);
});
