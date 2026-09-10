import test from 'node:test';
import assert from 'node:assert/strict';
import {createCompositeContinuousGeometry,evaluateCompositeContinuousGeometry} from '../src/physics/kirchhoffCompositeContinuousGeometry.js';
import {createCompositeContinuousLength as create} from '../src/physics/kirchhoffCompositeContinuousLength.js';
import {createCompositeChainLayout} from '../src/physics/kirchhoffCompositeChain.js';
import {ownCompositeJointInertiaGeometry} from '../src/physics/kirchhoffCompositeJointGeometry.js';
import {createCompositeToolLengthWorkspace,evaluateCompositeToolLengths} from '../src/physics/kirchhoffCompositeToolLengths.js';

const close=(a,b,t=1e-9)=>assert.ok(Number.isFinite(a)&&Number.isFinite(b)&&Math.abs(a-b)<=t,`${a} != ${b}; tolerance ${t}`);
const same=(a,b,t)=>{assert.equal(a.length,b.length);a.forEach((v,j)=>close(v,b[j],t));};
function fixture({bend=.13,edge=1}={}) {
    const coordinates=[0,1,2,3,4],geometry=createCompositeContinuousGeometry({coordinates}),element=geometry.edges[edge],
        positions=element.nodeIndices.map(j=>[coordinates[j],bend*coordinates[j]**2,0]);
    return {geometry,element,positions,operator:create({geometry:element}),bend};
}

test('continuous arclength matches an analytic parabola and bounds tangent speed between all quadrature samples',()=>{
    const f=fixture(),r=f.operator.evaluate(f.positions),a=2*f.bend,F=x=>.5*(x*Math.sqrt(1+a*a*x*x)+Math.asinh(a*x)/a);
    close(r.length,F(2)-F(1),1e-13);
    const left=evaluateCompositeContinuousGeometry(f.element,{positions:f.positions,fraction:0}).position,right=evaluateCompositeContinuousGeometry(f.element,{positions:f.positions,fraction:1}).position;
    assert.ok(r.length>Math.hypot(...right.map((v,k)=>v-left[k]))+1e-4);
    for(let j=0;j<=100;j++) {
        const p=evaluateCompositeContinuousGeometry(f.element,{positions:f.positions,fraction:j/100}),speed=Math.hypot(...p.positionDx);
        assert.ok(speed>=r.speedBounds.lower-1e-14&&speed<=r.speedBounds.upper+1e-14);
    }
    assert.equal(r.speedBounds.wholeInterval,true);assert.equal(r.pointwiseInextensibility,false);
});

test('continuous length gradient and Hessian match independent finite differences, with zero resultant and moment',()=>{
    const f=fixture();f.positions[1][2]=.017;const r=structuredClone(f.operator.evaluate(f.positions)),N=r.gradient.length,h=1e-6;
    for(let j=0;j<N;j++) {
        const plus=structuredClone(f.positions),minus=structuredClone(f.positions);plus[Math.floor(j/3)][j%3]+=h;minus[Math.floor(j/3)][j%3]-=h;
        const a=structuredClone(f.operator.evaluate(plus,{order:'gradient'})),b=f.operator.evaluate(minus,{order:'gradient'});
        close((a.length-b.length)/(2*h),r.gradient[j],4e-9);
        for(let i=0;i<N;i++)close((a.gradient[i]-b.gradient[i])/(2*h),r.hessian[i*N+j],5e-9);
    }
    const force=[0,0,0],moment=[0,0,0];
    f.positions.forEach((p,j)=>{const g=Array.from(r.gradient.slice(3*j,3*j+3));g.forEach((v,k)=>force[k]+=v);
        moment[0]+=p[1]*g[2]-p[2]*g[1];moment[1]+=p[2]*g[0]-p[0]*g[2];moment[2]+=p[0]*g[1]-p[1]*g[0];});
    same(force,[0,0,0],1e-14);same(moment,[0,0,0],1e-14);
    const shifted=f.positions.map(p=>[p[0]+2**30,p[1],p[2]]);assert.deepEqual(f.operator.evaluate(shifted).gradient,r.gradient);
});

test('a hidden interior tangent collapse rejects even when endpoints and sampled quadrature sites have nonzero speed',()=>{
    const f=fixture({edge:0}),positions=f.element.nodeIndices.map(j=>[(j-.237)**2,0,0]);
    assert.ok(Math.hypot(...positions[1].map((v,k)=>v-positions[0][k]))>0);
    assert.throws(()=>f.operator.evaluate(positions),e=>e.code==='continuous-length-refinement-required'&&/regularity/.test(e.message));
    const restored=f.operator.evaluate(f.positions);assert.equal(restored.operatorReady,true);assert.ok(restored.hessian.every(Number.isFinite));
});

test('failed or incomplete quadrature invalidates its borrowed response; gradient mode cannot expose a previous tangent',()=>{
    const f=fixture(),r=f.operator.evaluate(f.positions),old=r.length,bad=structuredClone(f.positions);bad[0][0]=NaN;
    assert.throws(()=>f.operator.evaluate(bad),/Finite positions/);assert.equal(r.operatorReady,false);assert.ok(r.hessian.every(Number.isNaN));
    close(f.operator.evaluate(f.positions,{order:'gradient'}).length,old,1e-13);assert.equal(r.hessianValid,false);assert.ok(r.hessian.every(Number.isNaN));
    assert.throws(()=>create({geometry:f.element,maxEvaluations:1}).evaluate(f.positions),e=>e.code==='continuous-length-refinement-required');
    assert.throws(()=>create({geometry:f.element,maxDepth:0}).evaluate(f.positions,{lengthTolerance:1e-30}),e=>e.code==='continuous-length-refinement-required');
    assert.ok(f.operator.diagnostics.cachedBasisSamples<=512);
});

test('signed continuous constraints pull back all common and relative reactions, including both physical endpoints and neighboring shape nodes',()=>{
    const coordinates=[0,1,2,3,4],base=createCompositeChainLayout(Array.from({length:4},()=>['wire','catheter'])),
        owned=ownCompositeJointInertiaGeometry({layout:base,coordinates,inertiaGeometryByTool:new Map([['wire',{}],['catheter',{}]])}),layout=owned.layout,
        basis=[[.8,.6,0],[-.6,.8,0],[0,0,1]],modes=coordinates.map((_,node)=>({node,basis,relativeDofs:[3*node,3*node+1,3*node+2]})),
        workspace=createCompositeToolLengthWorkspace({layout,modes,geometryByTool:owned.inertiaGeometryByTool}),q=coordinates.map(x=>[x,.04*x*x,0]),
        rho=coordinates.flatMap((_,j)=>[.003*j,.006*j*j,-.002*j]),restLengths=new Map([['wire',[1,1,1,1]],['catheter',[1,1,1,1]]]),
        multipliers=Float64Array.from({length:8},(_,j)=>(j%2?-.7:.4)*(j+1)),N=layout.dofCount+rho.length;
    function evaluate(positions,relative) {
        const wire=positions.map(p=>p.slice());modes.forEach(m=>m.basis.forEach((b,a)=>b.forEach((v,k)=>wire[m.node][k]+=v*relative[m.relativeDofs[a]])));
        evaluateCompositeToolLengths({toolPositions:new Map([['wire',wire],['catheter',positions]]),restLengths,multipliers,tolerance:1e-9},workspace);
        const H=new Float64Array(N*N),g=Float64Array.from([...workspace.commonGradient,...workspace.relativeGradient]);
        for(const row of workspace.rows){const ids=[...row.commonDofs,...Array.from(row.relativeDofs,d=>d+layout.dofCount)];ids.forEach((i,a)=>ids.forEach((j,b)=>H[i*N+j]+=row.geometricTangent[a*ids.length+b]));}
        return {work:workspace.rows.reduce((sum,row)=>sum+row.multiplier*row.residual,0),g,H};
    }
    const r=evaluate(q,rho),h=1e-6;
    assert.ok(workspace.rows.some(row=>row.nodeIndices.length===4));
    for(let j=0;j<rho.length;j++) {
        const plus=rho.slice(),minus=rho.slice();plus[j]+=h;minus[j]-=h;const a=evaluate(q,plus),b=evaluate(q,minus),col=layout.dofCount+j;
        close((a.work-b.work)/(2*h),r.g[col],2e-8);for(let i=0;i<N;i++)close((a.g[i]-b.g[i])/(2*h),r.H[i*N+col],2e-8);
    }
    for(const row of workspace.rows){assert.deepEqual(row.jacobian,row.forceColumn);assert.ok(Number.isFinite(row.parameterMetricDeviationBound));}
});

test('large signed length reactions use the actual force error budget without demanding sub-roundoff derivatives',()=>{
    const coordinates=[0,1,2,3,4],layout=createCompositeChainLayout(Array.from({length:4},()=>['wire'])),
        owned=ownCompositeJointInertiaGeometry({layout,coordinates,inertiaGeometryByTool:new Map([['wire',{}]])}),
        workspace=createCompositeToolLengthWorkspace({layout:owned.layout,modes:[],geometryByTool:owned.inertiaGeometryByTool}),
        positions=coordinates.map(x=>[x,.13*x*x,.007*x*x*x]),multipliers=Float64Array.from([1e5,-8e4,7e4,-9e4]),
        restLengths=new Map([['wire',[1,1,1,1]]]),input={toolPositions:new Map([['wire',positions]]),restLengths,multipliers,tolerance:5e-11,forceTolerance:1e-7};
    const row=workspace.rows[1];
    assert.throws(()=>row.compiled.evaluate(row.nodeIndices.map(n=>positions[n]),{
        gradientTolerance:1e-12/Math.abs(multipliers[1]),hessianTolerance:1e-11/Math.abs(multipliers[1])}),
        e=>e.code==='continuous-length-refinement-required'&&e.details.localTolerance?.length===3);
    evaluateCompositeToolLengths(input,workspace);
    assert.equal(workspace.operatorReady,true);assert.equal(workspace.hessianValid,true);
    assert.ok(workspace.quadrature.estimatedForceError<=workspace.quadrature.forceBudget);
    assert.equal(workspace.quadrature.forceBudget,.02*input.forceTolerance);
    assert.ok(workspace.rows.every(r=>r.quadrature.summation==='compensated'));
    const force=workspace.commonGradient.slice(),tolerance=workspace.rows[1].quadrature.gradientTolerance;
    evaluateCompositeToolLengths({...input,forceTolerance:5e-8},workspace);
    assert.equal(workspace.rows[1].quadrature.gradientTolerance,tolerance/2);
    same(workspace.commonGradient,force,2e-9);
    // The response remains the actual line-integral derivative. An
    // independent Simpson-4000 rule uses analytic C2 basis/tangents, not this
    // adaptive integration or its failure criterion.
    const expected=new Float64Array(layout.dofCount),samples=4000;
    for(const r of workspace.rows) {
        const own=r.nodeIndices.map(n=>positions[n]),a=r.compiled.geometry.coordinates[0],b=r.compiled.geometry.coordinates[1];
        for(let j=0;j<=samples;j++) {
            const response=evaluateCompositeContinuousGeometry(r.compiled.geometry,{positions:own,fraction:j/samples}),
                speed=Math.hypot(...response.positionDx),weight=(b-a)/(3*samples)*(j===0||j===samples?1:j%2?4:2);
            response.basis.first.forEach((v,index)=> {for(let k=0;k<3;k++)expected[layout.positions[r.nodeIndices[index]]+k]+=multipliers[r.index]*weight*v*response.positionDx[k]/speed;});
        }
    }
    same(workspace.commonGradient,expected,3e-8);
});
