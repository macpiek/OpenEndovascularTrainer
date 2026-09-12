import assert from 'node:assert/strict';
import test from 'node:test';
import {solveCompositeClearanceCell,condenseCompositeClearanceCell} from '../src/physics/kirchhoffCompositeClearance.js';

const close=(a,b,t=1e-9)=>assert.ok(Math.abs(a-b)<=t*(1+Math.max(Math.abs(a),Math.abs(b))),`${a} != ${b}`);

test('an open lumen permits independent transverse relaxation without normal load or friction budget',()=>{
    const r=solveCompositeClearanceCell({stiffness:[100,0,200],gradient:[1,-2],clearance:.0405});
    assert.ok(r.converged);assert.equal(r.active,false);
    close(r.offset[0],-.01);close(r.offset[1],.01);
    assert.ok(r.gap>0);assert.equal(r.normalForce,0);close(r.energyChange,-.015);
});

test('isotropic contact reaches the finite clearance and returns only the excess physical force',()=>{
    const k=100,c=.0405,f=[30,40],norm=50;
    const r=solveCompositeClearanceCell({stiffness:[k,0,k],gradient:f,clearance:c});
    assert.ok(r.converged);assert.ok(r.active);
    close(r.offset[0],-c*30/norm);close(r.offset[1],-c*40/norm);
    close(r.normalForce,norm-k*c);close(r.contactForce[0],(norm-k*c)*30/norm);
    close(r.contactForce[1],(norm-k*c)*40/norm);close(r.gap,0,1e-12);
    close(r.energyChange,-norm*c+.5*k*c*c);
    close(r.response[0],c/norm*(1-.6**2));close(r.response[1],-c/norm*.6*.8);
});

test('anisotropic contact satisfies force balance and minimizes energy on the entire circle',()=>{
    const stiffness=[13,3,41],gradient=[2,-4],clearance=.0405;
    const r=solveCompositeClearanceCell({stiffness,gradient,clearance});
    assert.ok(r.converged);assert.ok(r.active);assert.ok(r.residual<1e-10);
    assert.ok(r.normalForce>=0);assert.ok(r.complementarity<1e-10);
    const energy=(x,y)=>gradient[0]*x+gradient[1]*y+.5*(13*x*x+6*x*y+41*y*y);
    for(let i=0;i<360;i++) {
        const angle=i*Math.PI/180;
        assert.ok(energy(clearance*Math.cos(angle),clearance*Math.sin(angle))>=r.energyChange-1e-12);
    }
});

for(const active of [false,true]) test(`local elimination returns envelope force and tangent, including contact reaction (${active?'contact':'gap'})`,()=>{
    const coupling=[2,-1,.5,3,-.7,.2],n=3,K=[23,4,37],r0=active?[3,-2]:[.03,-.04],c=.0405;
    const evaluate=z=>{
        const gradient=[...r0];
        for(let i=0;i<n;i++){gradient[0]+=coupling[2*i]*z[i];gradient[1]+=coupling[2*i+1]*z[i];}
        const cell=solveCompositeClearanceCell({stiffness:K,gradient,clearance:c});
        assert.ok(cell.converged);assert.equal(cell.active,active);
        return condenseCompositeClearanceCell({cell,coupling,commonDofCount:n});
    };
    const z=[0,0,0],base=evaluate(z),eps=1e-5;
    for(let i=0;i<n;i++) {
        z[i]=eps;const plus=evaluate(z);z[i]=-eps;const minus=evaluate(z);z[i]=0;
        close(base.gradient[i],(plus.energy-minus.energy)/(2*eps),1e-8);
        for(let j=0;j<n;j++) close(base.hessian[j*n+i],(plus.gradient[j]-minus.gradient[j])/(2*eps),1e-8);
    }
    assert.equal(base.eliminatedDofCount,2);
});

test('rotating the transverse frame preserves the gap, energy and physical reaction',()=>{
    const K=[13,3,41],g=[2,-4],c=.0405,angle=.72,C=Math.cos(angle),S=Math.sin(angle);
    const rotate=([x,y])=>[C*x-S*y,S*x+C*y];
    const rotatedK=[C*C*K[0]-2*C*S*K[1]+S*S*K[2],C*S*(K[0]-K[2])+(C*C-S*S)*K[1],
        S*S*K[0]+2*C*S*K[1]+C*C*K[2]];
    const first=solveCompositeClearanceCell({stiffness:K,gradient:g,clearance:c});
    const second=solveCompositeClearanceCell({stiffness:rotatedK,gradient:rotate(g),clearance:c});
    assert.ok(first.converged&&second.converged);close(first.energyChange,second.energyChange);
    close(first.normalForce,second.normalForce);rotate(first.offset).forEach((v,i)=>close(v,second.offset[i]));
    rotate(first.contactForce).forEach((v,i)=>close(v,second.contactForce[i]));
});

test('unresolved contact, welded clearance and unsupported modes cannot silently become accepted reductions',()=>{
    const args={stiffness:[13,3,41],gradient:[2,-4],clearance:.0405};
    const cell=solveCompositeClearanceCell({...args,maxIterations:1});
    assert.equal(cell.converged,false);
    assert.throws(()=>condenseCompositeClearanceCell({cell,coupling:[1,0],commonDofCount:1}),/converged/);
    assert.throws(()=>solveCompositeClearanceCell({...args,clearance:0}),/positive physical clearance/);
    assert.throws(()=>solveCompositeClearanceCell({...args,stiffness:[1,1,1]}),/positive definite/);
    assert.throws(()=>solveCompositeClearanceCell({...args,stiffness:[1,0,-1]}),/positive definite/);
});

test('changing force units cannot restore radial compliance at active contact through underflow',()=>{
    for(const factor of [1,1e150,1e160,1e170,1e200]) {
        const cell=solveCompositeClearanceCell({stiffness:[100*factor,0,100*factor],gradient:[50*factor,0],
            clearance:.0405,forceTolerance:1e-10*factor});
        assert.ok(cell.converged);close(cell.normalForce/factor,45.95);
        close(cell.response[0]*factor,0,1e-15);close(cell.response[2]*factor,.0405/50,1e-15);
        const reduced=condenseCompositeClearanceCell({cell,coupling:[factor,0],commonDofCount:1});
        close(reduced.hessian[0]/factor,0,1e-15);
    }
});
