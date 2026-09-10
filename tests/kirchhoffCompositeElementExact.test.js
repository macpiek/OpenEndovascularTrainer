import assert from 'node:assert/strict';
import test from 'node:test';
import * as oracle from '../src/physics/kirchhoffCompositeElement.js';
import * as exact from '../src/physics/kirchhoffCompositeElementExact.js';
import * as chain from '../src/physics/kirchhoffCompositeChain.js';

const close=(a,b,t=2e-12)=>assert.ok(Number.isFinite(a)&&Number.isFinite(b)&&Math.abs(a-b)<=t*(1+Math.max(Math.abs(a),Math.abs(b))),`${a} != ${b}`);
const outputs=['strain','jacobian','moments','gradient','toolEnergy','referenceTwists'];
const vectorClose=(a,b,t)=>{assert.equal(a.length,b.length);a.forEach((v,i)=>close(v,b[i],t));};
function fixture(seed=1,count=2) {
    const random=()=>{seed=(Math.imul(seed,1664525)+1013904223)>>>0;return seed/4294967296;};
    const rest=[[0,0,0],[2,.2,-.1],[3.8,.6,.3]];
    return {positions:rest.map(p=>p.map(v=>v+(random()-.5)*.2)),reference:oracle.captureCompositeReferenceFrames(rest),
        referenceLength:1.7+random(),referenceTwist:2*Math.PI,
        tools:Array.from({length:count},(_,i)=>({dsDx:.6+random(),angles:[random()-.5,random()-.5],referenceTwist:i?undefined:-4*Math.PI,
            material:oracle.compileCompositeMaterial({stiffness:[[3,.3,.2],[.3,5,-.1],[.2,-.1,2]],
                intrinsic:[.05*random(),-.1*random(),.3*random()],energyOffset:.1+random()})}))};
}
function move(args,dof,amount) {
    if(dof<9)args.positions[Math.floor(dof/3)][dof%3]+=amount;
    else args.tools[Math.floor((dof-9)/2)].angles[(dof-9)%2]+=amount;
}
function compareResponse(a,b) {
    close(a.energy,b.energy);for(const key of outputs)vectorClose(a[key],b[key]);
}

test('exact backend preserves original energy, exact gradient, strain/J, independent materials and winding anchors',()=>{
    for(const count of [1,2]) {
        const a=oracle.createCompositeElementWorkspace(count),b=exact.createCompositeElementWorkspace(count);
        for(let seed=1;seed<=12;seed++) {
            const args=fixture(seed,count),before=structuredClone(args);
            compareResponse(oracle.evaluateCompositeElement(args,a),exact.evaluateCompositeElement(args,b));
            assert.deepEqual(args,before);
        }
    }
});

test('every exact Hessian entry matches independent finite differences of the ORIGINAL JS exact gradient',()=>{
    for(const count of [1,2])for(const seed of [27,421,1039]) {
        const args=fixture(seed,count),w=exact.evaluateCompositeElement(args),n=w.dofCount,hessian=w.hessian.slice(),h=2e-6;
        const referenceA=oracle.createCompositeElementWorkspace(count),referenceB=oracle.createCompositeElementWorkspace(count);
        for(let col=0;col<n;col++) {
            const plus=structuredClone(args),minus=structuredClone(args);move(plus,col,h);move(minus,col,-h);
            const a=oracle.evaluateCompositeElement(plus,referenceA),b=oracle.evaluateCompositeElement(minus,referenceB);
            for(let row=0;row<n;row++)close(hessian[row*n+col],(a.gradient[row]-b.gradient[row])/(2*h),2e-8);
        }
        for(let row=0;row<n;row++)for(let col=0;col<n;col++)assert.equal(hessian[row*n+col],hessian[col*n+row]);
        assert.ok(Math.max(...w.jacobian.slice(2*n,2*n+9).map(Math.abs))>1e-3,'position-dependent reference twist is present');
        assert.ok(w.hessian.some((v,i)=>Math.abs(v-oracle.evaluateCompositeElement(args,referenceA).hessian[i])>.1),'nonzero strain curvature was not dropped');
    }
});

test('exact strain-curvature term vanishes at zero moment and preserves constant energy offsets',()=>{
    const args=fixture(271),reference=oracle.evaluateCompositeElement(args);
    args.tools.forEach((tool,index)=>{tool.material=oracle.compileCompositeMaterial({stiffness:[[3,.3,.2],[.3,5,-.1],[.2,-.1,2]],
        intrinsic:Array.from(reference.strain.slice(3*index,3*index+3)),energyOffset:.2});});
    const full=exact.evaluateCompositeElement(args),gn=oracle.evaluateCompositeElement(args);
    vectorClose(full.hessian,gn.hessian);vectorClose(full.gradient,new Float64Array(full.dofCount));
    const before=full.hessian.slice(),g=full.gradient.slice(),energy=full.energy;
    args.tools[0].material={...args.tools[0].material,energyOffset:args.tools[0].material.energyOffset+.37};
    const after=exact.evaluateCompositeElement(args);
    close(after.energy-energy,args.referenceLength*args.tools[0].dsDx*.37);vectorClose(after.gradient,g);vectorClose(after.hessian,before);
});

test('prestress can make the true Hessian indefinite; the backend applies no PSD clamp or diagonal floor',()=>{
    const p=[[0,0,0],[2,0,0],[4,0,0]],args={positions:p,reference:oracle.captureCompositeReferenceFrames(p),referenceLength:2,
        tools:[{angles:[0,0],material:oracle.compileCompositeMaterial({EI1:2,EI2:3,GJ:1,kappa0:[.5,.2]})}]};
    const full=exact.evaluateCompositeElement(args),gn=oracle.evaluateCompositeElement(args),n=full.dofCount;
    // Witness d=(+x at proximal node,-z at proximal node).
    const quadratic=h=>h[0]+h[2*n+2]-2*h[2];
    close(quadratic(full.hessian),-.25);assert.ok(quadratic(gn.hessian)>=0);
    const plus=structuredClone(args),minus=structuredClone(args),h=2e-5;
    move(plus,0,h);move(plus,2,-h);move(minus,0,-h);move(minus,2,h);
    close((oracle.evaluateCompositeElement(plus).energy-2*full.energy+oracle.evaluateCompositeElement(minus).energy)/(h*h),-.25,2e-6);
});

test('rigid coordinate transforms preserve energy and correctly transform all position/spin Hessian blocks',()=>{
    const args=fixture(51),a=exact.evaluateCompositeElement(args),changed=structuredClone(args),rotate=v=>[-v[1],v[2],-v[0]],shift=[12,-6,20];
    changed.positions=args.positions.map(p=>rotate(p).map((v,i)=>v+shift[i]));
    changed.reference=args.reference.map(frame=>({tangent:rotate(frame.tangent),director:rotate(frame.director)}));
    const b=exact.evaluateCompositeElement(changed),n=a.dofCount;
    close(a.energy,b.energy);
    const mapping=Array.from({length:n},(_,i)=>i<9?{index:3*Math.floor(i/3)+[1,2,0][i%3],sign:[-1,1,-1][i%3]}:{index:i,sign:1});
    for(let i=0;i<n;i++) {
        close(b.gradient[i],mapping[i].sign*a.gradient[mapping[i].index]);
        for(let j=0;j<n;j++)close(b.hessian[i*n+j],mapping[i].sign*mapping[j].sign*a.hessian[mapping[i].index*n+mapping[j].index]);
        for(let axis=0;axis<3;axis++)close(a.hessian[i*n+axis]+a.hessian[i*n+3+axis]+a.hessian[i*n+6+axis],0);
    }
    // Separate material spins have no artificial direct elastic cross term.
    for(const i of [9,10])for(const j of [11,12])assert.equal(a.hessian[i*n+j],0);
});

test('anchored reference twist and its exact Hessian remain continuous across the atan2 branch cut',()=>{
    const p=[[0,0,0],[2,0,0],[4,0,0]],base={positions:p,reference:oracle.captureCompositeReferenceFrames(p),referenceLength:2,referenceTwist:Math.PI,
        tools:[{angles:[.1,.2],dsDx:.7,material:oracle.compileCompositeMaterial({stiffness:[[3,.3,.2],[.3,5,-.1],[.2,-.1,2]],intrinsic:[.1,-.2,.3]})}]};
    const evaluate=epsilon=>{const args=structuredClone(base);args.reference[1].director=[0,-Math.cos(epsilon),Math.sin(epsilon)];
        const response=exact.evaluateCompositeElement(args);compareResponse(response,oracle.evaluateCompositeElement(args));return response;};
    const a=evaluate(-1e-7),b=evaluate(1e-7),n=a.dofCount;
    close(a.referenceTwists[0],Math.PI+1e-7);close(b.referenceTwists[0],Math.PI-1e-7);
    let difference=0;for(let i=0;i<n*n;i++)difference=Math.max(difference,Math.abs(a.hessian[i]-b.hessian[i]));
    const c=evaluate(-1e-5),d=evaluate(1e-5);let coarse=0;for(let i=0;i<n*n;i++)coarse=Math.max(coarse,Math.abs(c.hessian[i]-d.hessian[i]));
    assert.ok(difference<coarse*.011&&difference>coarse*.009,'Hessian has continuous first-order variation, not a branch jump');
    const args=structuredClone(base),h=2e-6;args.reference[1].director=[0,-1,0];const full=exact.evaluateCompositeElement(args);
    for(let col=0;col<n;col++) {
        const plus=structuredClone(args),minus=structuredClone(args);move(plus,col,h);move(minus,col,-h);
        const pa=oracle.evaluateCompositeElement(plus),pb=oracle.evaluateCompositeElement(minus);
        for(let row=0;row<n;row++)close(full.hessian[row*n+col],(pa.gradient[row]-pb.gradient[row])/(2*h),2e-8);
    }
});

test('explicit wasm-exact Chain assembly matches finite differences of the original full mixed-section gradient',()=>{
    const coordinates=[0,2,4,6,8],rest=coordinates.map(x=>[x,.2*Math.sin(x/12),.2*Math.cos(x/17)]);
    const layout=chain.createCompositeChainLayout([['wire'],['wire','catheter'],['wire','catheter'],['catheter']]);
    const data={positions:rest.map((p,i)=>p.map((v,j)=>v+.03*Math.sin(i+j))),coordinates,reference:oracle.captureCompositeReferenceFrames(rest),
        tools:[0,1].map(i=>({id:i?'catheter':'wire',angles:Float64Array.from(coordinates.slice(1),x=>.02*x+.3*i),
            referenceTwists:new Float64Array(coordinates.length-2).fill(i?2*Math.PI:0),dsDx:1.1+.1*i,
            material:oracle.compileCompositeMaterial({EI:[[3,.2],[.2,5]],GJ:2,kappa0:[.01,-.02],energyOffset:.2})}))};
    const full=chain.createCompositeChainWorkspace(layout,{elementBackend:'wasm-exact'}),js=chain.createCompositeChainWorkspace(layout,{elementBackend:'javascript'});
    assert.equal(chain.createCompositeChainWorkspace(layout).elementBackend,'wasm');
    chain.assembleCompositeChain(data,full);chain.assembleCompositeChain(data,js);close(full.energy,js.energy);vectorClose(full.gradient,js.gradient);
    const moveGlobal=(state,dof,delta)=>{for(let node=0;node<coordinates.length;node++)for(let axis=0;axis<3;axis++)
        if(layout.positions[node]+axis===dof){state.positions[node][axis]+=delta;return;}
        for(const tool of state.tools){const edge=layout.spins.get(tool.id).indexOf(dof);if(edge>=0){tool.angles[edge]+=delta;return;}}throw new Error('Unknown DOF');};
    const h=2e-6;
    for(let col=0;col<layout.dofCount;col++) {
        const plus=structuredClone(data),minus=structuredClone(data);moveGlobal(plus,col,h);moveGlobal(minus,col,-h);
        chain.assembleCompositeChain(plus,js);const pg=js.gradient.slice();chain.assembleCompositeChain(minus,js);
        for(let row=0;row<layout.dofCount;row++) {
            const separation=Math.abs(row-col),value=separation<layout.band?full.hessian[Math.max(row,col)*layout.band+separation]:0;
            close(value,(pg[row]-js.gradient[row])/(2*h),2e-8);
        }
    }
});

test('exact outputs reuse stable buffers and retain all geometric/material finite guards',()=>{
    const args=fixture(38),w=exact.createCompositeElementWorkspace(2),arrays=[...outputs,'hessian'].map(k=>w[k]),memory=w.kernel.memory.buffer;
    for(let i=0;i<10;i++)exact.evaluateCompositeElement(args,w);
    [...outputs,'hessian'].forEach((k,i)=>assert.equal(w[k],arrays[i]));assert.equal(w.kernel.memory.buffer,memory);
    assert.notEqual(exact.createCompositeElementWorkspace(2).kernel.memory.buffer,memory);
    assert.throws(()=>exact.evaluateCompositeElement({...args,referenceLength:0}),/positive/);
    assert.throws(()=>exact.evaluateCompositeElement({...args,positions:[[0,0,0],[0,0,0],[1,0,0]]}),/collapse/);
    const broken=fixture(38);broken.tools[0].material.stiffness[0]=Infinity;assert.throws(()=>exact.evaluateCompositeElement(broken),/Nonfinite/);
});
