import assert from 'node:assert/strict';
import test from 'node:test';
import { captureCompositeReferenceFrames, transportCompositeReferenceFrames, compileCompositeMaterial, createCompositeElementWorkspace,
    evaluateCompositeElement } from '../src/physics/kirchhoffCompositeElement.js';

const close = (a,b,t=1e-9) => assert.ok(Math.abs(a-b) <= t*(1+Math.max(Math.abs(a),Math.abs(b))), `${a} != ${b}`);
const isotropic = (ei=2,gj=1) => compileCompositeMaterial({ EI1:ei, GJ:gj });
function fixture() {
    const rest = [[0,0,0],[2,0,0],[4,.2,.1]];
    return { positions:[[.01,.02,0],[2.04,.08,.04],[3.96,.27,.22]], reference:captureCompositeReferenceFrames(rest),
        referenceLength:2, tools:[
            { id:'wire', angles:[.12,.35], dsDx:1.1, material:compileCompositeMaterial({EI:[[2,.2],[.2,3]],GJ:.8,kappa0:[.01,-.03]}) },
            { id:'catheter', angles:[-.3,-.1], dsDx:.9, material:compileCompositeMaterial({EI1:7,EI2:4,GJ:2,kappa0:[-.02,.06],tau0:.01}) }
        ] };
}

test('common centerline sums bending stiffness without welding independent tool spins', () => {
    const angle=.18, h=2, curvature=2*Math.tan(angle/2)/h;
    const positions=[[0,0,0],[h,0,0],[h+h*Math.cos(angle),h*Math.sin(angle),0]];
    const args={positions,reference:captureCompositeReferenceFrames(positions),referenceLength:h,
        tools:[{angles:[0,0],material:isotropic(2,1)},{angles:[0,0],material:isotropic(7,4)}]};
    const w=evaluateCompositeElement(args);
    close(w.energy,.5*h*9*curvature**2);
    close(w.strain[1],curvature); close(w.strain[4],curvature);
    assert.equal(w.dofCount,13, 'nine common positions and four independent spins');
    const flat={...args,positions:[[0,0,0],[2,0,0],[4,0,0]],reference:captureCompositeReferenceFrames([[0,0,0],[2,0,0],[4,0,0]])};
    flat.tools[0].angles=[.1,.5]; flat.tools[1].angles=[-.4,-.7];
    evaluateCompositeElement(flat,w);
    close(w.energy,.5*(1*.4**2+4*.3**2)/h);
    close(w.gradient[9],-.4/h); close(w.gradient[10],.4/h);
    close(w.gradient[11],4*.3/h); close(w.gradient[12],-4*.3/h);
    assert.equal(w.hessian[9*13+11],0, 'independent torsion blocks');
});

test('accepted time transport preserves material strain and winding without replacing history by a spatial Bishop frame', () => {
    const args=fixture();args.referenceTwist=2*Math.PI;
    const original=structuredClone(args.reference),before=evaluateCompositeElement(args);
    const reference=transportCompositeReferenceFrames(args.reference,args.positions);
    assert.deepEqual(args.reference,original,'preparation cannot mutate accepted frames');
    const after=evaluateCompositeElement({...args,reference,referenceTwist:before.referenceTwists[0]});
    close(after.energy,before.energy,1e-12);
    before.strain.forEach((v,i)=>close(v,after.strain[i],1e-12));
    const recaptured=evaluateCompositeElement({...args,reference:captureCompositeReferenceFrames(args.positions)});
    assert.ok(Math.abs(recaptured.energy-after.energy)>1e-7,'spatial recapture is not the accepted time transport');
});

test('anisotropic bending uses Darboux axes and the prescribed natural bend is stress free', () => {
    const h=2,angle=.18,k=2*Math.tan(angle/2)/h;
    const positions=[[0,0,0],[h,0,0],[h+h*Math.cos(angle),h*Math.sin(angle),0]];
    const args={positions,reference:captureCompositeReferenceFrames(positions),referenceLength:h,
        tools:[{angles:[0,0],material:compileCompositeMaterial({EI1:2,EI2:7,GJ:3})}]};
    const r=evaluateCompositeElement(args);
    close(r.energy,.5*h*7*k*k,1e-13);
    close(r.strain[0],0);close(r.strain[1],k);
    args.tools[0].material=compileCompositeMaterial({EI1:2,EI2:7,GJ:3,intrinsic:[0,k,0]});
    assert.ok(evaluateCompositeElement(args).energy<1e-25);
});

test('continuous reference twist preserves compensated frames through the atan2 branch cut', () => {
    for(const gamma of [Math.PI-.01,Math.PI+.01,5*Math.PI+.01,-5*Math.PI-.01]) {
        const args={positions:[[0,0,0],[2,0,0],[4,0,0]],referenceLength:2,referenceTwist:gamma,
            reference:[{tangent:[1,0,0],director:[0,1,0]},
                {tangent:[1,0,0],director:[0,Math.cos(gamma),Math.sin(gamma)]}],
            tools:[{angles:[0,-gamma],material:isotropic(2,5)}]};
        const r=evaluateCompositeElement(args);
        assert.ok(r.energy<1e-25);close(r.referenceTwists[0],gamma,1e-14);
        assert.ok(r.gradient.every(x=>Math.abs(x)<1e-12));
        args.tools[0].referenceTwist=gamma;args.referenceTwist=0;
        assert.ok(evaluateCompositeElement(args).energy<1e-25,'per-tool accepted anchor takes precedence');
    }
});

test('a compiled heterogeneous material energy offset preserves forces and the Gauss-Newton tangent', () => {
    const args=fixture(),r=evaluateCompositeElement(args),energy=r.energy;
    const gradient=r.gradient.slice(),hessian=r.hessian.slice();
    args.tools[0].material={...args.tools[0].material,energyOffset:1.7};
    const shifted=evaluateCompositeElement(args);
    close(shifted.energy,energy+2*1.1*1.7);
    assert.deepEqual(shifted.gradient,gradient);assert.deepEqual(shifted.hessian,hessian);
    assert.equal(compileCompositeMaterial({EI1:2,GJ:1,energyOffset:.3}).energyOffset,.3);
    assert.throws(()=>compileCompositeMaterial({EI1:2,GJ:1,energyOffset:Infinity}),/finite/);
});

test('finite inputs that overflow the constitutive tangent are rejected', () => {
    const positions=[[0,0,0],[1e-50,0,0],[2e-50,0,0]];
    assert.throws(()=>evaluateCompositeElement({positions,reference:captureCompositeReferenceFrames(positions),referenceLength:1,
        tools:[{angles:[0,0],material:compileCompositeMaterial({EI1:1e300,GJ:1e300})}]}),/Nonfinite/);
});

test('both tool energies and gradients add on one position field; individual spin columns stay separate', () => {
    const args=fixture(), both=evaluateCompositeElement(args);
    const singles=args.tools.map(tool=>evaluateCompositeElement({...args,tools:[tool]}));
    close(both.energy,singles[0].energy+singles[1].energy);
    for(let i=0;i<9;i++) close(both.gradient[i],singles[0].gradient[i]+singles[1].gradient[i]);
    for(let s=0;s<2;s++) for(let j=0;j<2;j++) close(both.gradient[9+2*s+j],singles[s].gradient[9+j]);
    for(let axis=0;axis<3;axis++) close(both.gradient[axis]+both.gradient[3+axis]+both.gradient[6+axis],0);
});

test('nonlinear bend/twist gradient, including reference-frame transport, matches independent energy differences', () => {
    const args=fixture(), result=evaluateCompositeElement(args), expected=result.gradient.slice(), n=expected.length;
    const change=(dof,delta)=> { if(dof<9) args.positions[Math.floor(dof/3)][dof%3]+=delta;
        else args.tools[Math.floor((dof-9)/2)].angles[(dof-9)%2]+=delta; };
    for(let i=0;i<n;i++) {
        const eps=1e-6;
        change(i,eps); const plus=evaluateCompositeElement(args).energy;
        change(i,-2*eps); const minus=evaluateCompositeElement(args).energy;
        change(i,eps); close(expected[i],(plus-minus)/(2*eps),2e-8);
    }
    const w=evaluateCompositeElement(args);
    for(let sample=0;sample<12;sample++) {
        const v=Array.from({length:n},(_,i)=>Math.sin((sample+1)*(i+.3)));
        let q=0; for(let i=0;i<n;i++) for(let j=0;j<n;j++) {
            close(w.hessian[i*n+j],w.hessian[j*n+i],1e-13);
            q+=v[i]*w.hessian[i*n+j]*v[j];
        }
        assert.ok(q>=-1e-12, 'constitutive Gauss-Newton tangent stays positive semidefinite');
    }
});

test('a common rigid transform preserves constitutive energy, spins and rotated position forces', () => {
    const args=fixture(), initial=evaluateCompositeElement(args);
    const gradient=initial.gradient.slice(), energy=initial.energy;
    const rotate=v=>[-v[1],v[2],-v[0]];
    args.positions=args.positions.map(p=>rotate(p).map((x,j)=>x+[7,-3,11][j]));
    args.reference=args.reference.map(f=>({tangent:rotate(f.tangent),director:rotate(f.director)}));
    const after=evaluateCompositeElement(args);
    close(after.energy,energy,2e-12);
    for(let i=0;i<3;i++) rotate([...gradient.slice(i*3,i*3+3)]).forEach((v,j)=>close(v,after.gradient[3*i+j],2e-11));
    for(let i=9;i<13;i++) close(gradient[i],after.gradient[i],2e-11);
});

test('independent material-coordinate scales enter strain and integrated energy without averaging compliance', () => {
    const positions=[[0,0,0],[2,0,0],[4,0,0]];
    const args={positions,reference:captureCompositeReferenceFrames(positions),referenceLength:2,
        tools:[{angles:[0,.6],dsDx:3,material:isotropic(2,5)}]};
    const r=evaluateCompositeElement(args);
    close(r.strain[2],.1); close(r.energy,.5*6*5*.1**2);
    close(r.gradient[10],5*.6/6);
});

test('compiled stiffness, reference frames and collapsed geometry are validated; repeated evaluations reuse numeric buffers', () => {
    assert.throws(()=>compileCompositeMaterial({EI1:-1,GJ:1}),/positive definite/);
    const args=fixture(), w=createCompositeElementWorkspace(2);
    const arrays=[w.arena.data,w.gradient,w.hessian,w.jacobian,w.strain];
    const original=structuredClone(args);
    for(let i=0;i<20;i++) evaluateCompositeElement(args,w);
    assert.deepEqual([w.arena.data,w.gradient,w.hessian,w.jacobian,w.strain],arrays);
    [w.arena.data,w.gradient,w.hessian,w.jacobian,w.strain].forEach((array,i)=>assert.equal(array,arrays[i]));
    assert.deepEqual(args,original);
    assert.throws(()=>evaluateCompositeElement({...args,positions:[[0,0,0],[0,0,0],[1,0,0]]}),/collapse/);
    assert.throws(()=>evaluateCompositeElement({...args,referenceLength:0}),/positive/);
    assert.throws(()=>evaluateCompositeElement({...args,tools:[{...args.tools[0],dsDx:0}]}),/positive/);
});
