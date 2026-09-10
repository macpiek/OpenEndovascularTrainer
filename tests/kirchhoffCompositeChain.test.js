import assert from 'node:assert/strict';
import test from 'node:test';
import {captureCompositeReferenceFrames,compileCompositeMaterial} from '../src/physics/kirchhoffCompositeElement.js';
import {createCompositeChainLayout,createCompositeChainWorkspace,assembleCompositeChain,
    solveCompositeChainIncrement} from '../src/physics/kirchhoffCompositeChain.js';

const close=(a,b,t=1e-9)=>assert.ok(Math.abs(a-b)<=t*(1+Math.max(Math.abs(a),Math.abs(b))),`${a} != ${b}`);
function straight(n=10,h=2,catheterEdges=n-1) {
    const positions=Array.from({length:n},(_,i)=>[i*h,0,0]),coordinates=positions.map(p=>p[0]);
    const layout=createCompositeChainLayout(Array.from({length:n-1},(_,i)=>i<catheterEdges?['wire','catheter']:['wire']));
    const w=createCompositeChainWorkspace(layout);
    const data={positions,coordinates,reference:captureCompositeReferenceFrames(positions),tools:[
        {id:'wire',angles:new Float64Array(n-1),material:compileCompositeMaterial({EI1:2,GJ:3})},
        {id:'catheter',angles:new Float64Array(n-1),material:compileCompositeMaterial({EI1:7,GJ:11})}
    ]};
    return {w,data,layout};
}
function clamped(f) {
    const fixed=new Uint8Array(f.layout.dofCount);
    // The analytic test linearizes an inextensible straight centerline.
    // Axial motion has no bending stiffness and is prescribed in this test.
    for(let i=0;i<f.layout.nodeCount;i++) fixed[f.layout.positions[i]]=1;
    for(let node=0;node<2;node++) for(let axis=0;axis<3;axis++) fixed[f.layout.positions[node]+axis]=1;
    for(const offsets of f.layout.spins.values()) if(offsets[0]>=0) fixed[offsets[0]]=1;
    return fixed;
}

test('one spatial chain allocates each position once and only the material spins present on an edge',()=>{
    const {layout,w}=straight(10,2,6);
    assert.equal(layout.dofCount,3*10+9+6);
    assert.equal(layout.spins.get('catheter')[6],-1);
    assert.ok(layout.band<=13);
    assert.equal(w.hessian.length,layout.dofCount*layout.band);
    const owned=[];
    for(const i of layout.positions) owned.push(i,i+1,i+2);
    for(const offsets of layout.spins.values()) owned.push(...offsets.filter(i=>i>=0));
    assert.equal(new Set(owned).size,layout.dofCount);
    assert.deepEqual(owned.sort((a,b)=>a-b),Array.from({length:layout.dofCount},(_,i)=>i));
});

for(const n of [4,10,35]) test(`common-chain bending matches the independent discrete cantilever solution and balances support reactions (${n})`,()=>{
    const h=2,f=straight(n,h),fixed=clamped(f),load=new Float64Array(f.layout.dofCount),force=.003;
    assembleCompositeChain(f.data,f.w);
    const tip=f.layout.positions[n-1]; load[tip+1]=-force;
    const r=solveCompositeChainIncrement(f.w,{fixed,extraGradient:load,tolerance:1e-10});
    assert.ok(r.converged,JSON.stringify({residual:r.maximumResidual}));
    const m=n-2,sumSquares=m*(m+1)*(2*m+1)/6;
    close(r.increment[tip+1],force*h**3*sumSquares/(2+7),3e-10);
    let support=0,moment=0;
    for(let i=0;i<n;i++){support+=r.reactions[f.layout.positions[i]+1];moment+=i*h*r.reactions[f.layout.positions[i]+1];}
    close(support,-force,1e-10);close(moment,-force*(n-1)*h,1e-10);
    assert.equal(r.factorizations,1);
    for(let i=0;i<fixed.length;i++) if(fixed[i]) assert.equal(r.increment[i],0);
});

test('one factorization keeps opposite wire/catheter torques and differing GJ independent',()=>{
    const n=12,h=2,f=straight(n,h),fixed=clamped(f),load=new Float64Array(f.layout.dofCount);
    const wire=f.layout.spins.get('wire'),cat=f.layout.spins.get('catheter');
    const tw=.02,tc=-.07; load[wire[n-2]]=-tw;load[cat[n-2]]=-tc;
    assembleCompositeChain(f.data,f.w);
    const r=solveCompositeChainIncrement(f.w,{fixed,extraGradient:load,tolerance:1e-11});
    assert.ok(r.converged);
    close(r.increment[wire[n-2]],tw*(n-2)*h/3);
    close(r.increment[cat[n-2]],tc*(n-2)*h/11);
    close(r.reactions[wire[0]],-tw);close(r.reactions[cat[0]],-tc);
    for(const offset of f.layout.positions) for(let j=0;j<3;j++) close(r.increment[offset+j],0);
});

test('piecewise wire/overlap sections preserve bending moments instead of averaging their stiffness over the chain',()=>{
    const n=10,h=2,catEdges=5,f=straight(n,h,catEdges),fixed=clamped(f),load=new Float64Array(f.layout.dofCount),force=.003;
    const tip=f.layout.positions[n-1];load[tip+1]=-force;
    assembleCompositeChain(f.data,f.w);
    const r=solveCompositeChainIncrement(f.w,{fixed,extraGradient:load,tolerance:1e-10});
    assert.ok(r.converged);
    let expected=0;
    for(let vertex=1;vertex<n-1;vertex++) expected+=force*h**3*(n-1-vertex)**2/(vertex<catEdges?9:2);
    close(r.increment[tip+1],expected,2e-10);
});

test('assembled curved-chain gradient matches independent changes of the single position field and each spin',()=>{
    const f=straight(5),p=f.data.positions;
    p[2][1]=.1;p[3][1]=.4;p[4][2]=.3;
    f.data.tools[0].angles.set([.1,.2,.4,.6]);f.data.tools[1].angles.set([-.3,-.2,0,.1]);
    assembleCompositeChain(f.data,f.w);const gradient=f.w.gradient.slice();
    const coordinates=[];
    for(let i=0;i<p.length;i++)for(let j=0;j<3;j++)coordinates.push([f.layout.positions[i]+j,d=>p[i][j]+=d]);
    for(const tool of f.data.tools){const offsets=f.layout.spins.get(tool.id);
        for(let i=0;i<offsets.length;i++)if(offsets[i]>=0)coordinates.push([offsets[i],d=>tool.angles[i]+=d]);}
    for(const [dof,change] of coordinates){const eps=1e-6;
        change(eps);const plus=assembleCompositeChain(f.data,f.w).energy;
        change(-2*eps);const minus=assembleCompositeChain(f.data,f.w).energy;
        change(eps);close(gradient[dof],(plus-minus)/(2*eps),3e-8);}
    const diagonal=new Float64Array(f.layout.dofCount).fill(1.3);
    const result=solveCompositeChainIncrement(assembleCompositeChain(f.data,f.w),{diagonal});
    assert.ok(result.converged);
});

test('invalid intervals, detached interfaces and unrestrained zero-energy modes do not silently pass',()=>{
    assert.throws(()=>createCompositeChainLayout([['wire'],['catheter']]),/continuous material/);
    assert.throws(()=>createCompositeChainLayout([['wire'],['catheter'],['wire']]),/connected material/);
    const f=straight();assembleCompositeChain(f.data,f.w);
    assert.throws(()=>solveCompositeChainIncrement(f.w),/zero-energy mode/);
    f.data.coordinates[2]=f.data.coordinates[1];
    assert.throws(()=>assembleCompositeChain(f.data,f.w),/increase/);
});

test('a curved unsupported chain with positive coordinate diagonals and zero RHS retains rigid null modes',()=>{
    const f=straight(5);
    f.data.positions=[[0,0,0],[1,.2,.1],[2,.5,.4],[3,.8,.2],[4,1.5,.9]];
    f.data.reference=captureCompositeReferenceFrames(f.data.positions);
    for(const tool of f.data.tools) tool.angles=[.1,.2,.25,.3];
    assembleCompositeChain(f.data,f.w);
    assert.ok(f.w.gradient.every(Number.isFinite));
    for(let i=0;i<f.layout.dofCount;i++) assert.ok(f.w.hessian[i*f.layout.band]>0);
    assert.throws(()=>solveCompositeChainIncrement(f.w,{extraGradient:Float64Array.from(f.w.gradient,x=>-x)}),/zero-energy mode|singular/);
});

test('fixed supports cannot hide a nonfinite original operator or reaction',()=>{
    const f=straight(3);assembleCompositeChain(f.data,f.w);
    f.w.hessian[0]=Infinity;
    assert.throws(()=>solveCompositeChainIncrement(f.w,{fixed:new Uint8Array(f.layout.dofCount).fill(1)}),/Nonfinite original/);
});

test('chain passes the accepted reference winding to each material',()=>{
    const f=straight(3),gamma=Math.PI+.01;
    f.data.reference[1].director=[0,Math.cos(gamma),Math.sin(gamma)];
    for(const tool of f.data.tools) {tool.angles=[0,-gamma];tool.referenceTwists=[gamma];}
    assembleCompositeChain(f.data,f.w);
    assert.ok(f.w.energy<1e-25);
    for(const values of f.w.evaluatedReferenceTwists.values()) close(values[0],gamma);
});
