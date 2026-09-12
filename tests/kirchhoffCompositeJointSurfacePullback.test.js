import assert from 'node:assert/strict';
import test from 'node:test';
import {pathToFileURL} from 'node:url';
import {createCompositeChainLayout} from '../src/physics/kirchhoffCompositeChain.js';
import {createCompositeJointSurfacePullback as create,createCompositeJointSurfacePullbackFactory as factory,pullbackCompositeJointSurface as pull,evaluateCompositeJointSurfaceLoads as loads} from '../src/physics/kirchhoffCompositeJointSurfacePullback.js';
const motionRoot=process.env.OET_JOINT_SURFACE_MOTION_ROOT;
const motionUrl=p=>motionRoot?pathToFileURL(`${motionRoot}/${p}`):new URL(`../${p}`,import.meta.url);
const {evaluateCompositeJointSurfaceMotion:motion}=await import(motionUrl('src/physics/kirchhoffCompositeJointSurfaceMotion.js'));
const {captureCompositeReferenceFrames}=await import(motionUrl('src/physics/kirchhoffCompositeElement.js'));
const close=(a,b,t=2e-12)=>assert.ok(Number.isFinite(a)&&Number.isFinite(b)&&Math.abs(a-b)<=t,`${a} != ${b} (${t})`);
const vector=(a,b,t)=>{assert.equal(a.length,b.length);a.forEach((v,i)=>close(v,b[i],t));};
const dot=(a,b)=>a.reduce((s,v,i)=>s+v*b[i],0);

test('one owned surface chart builds independent local mappings and detects changed bases or layout before reuse',()=>{
    for(const spec of [{},{single:'catheter'},{mixed:true}]) {
        const f=fixture(spec),original=structuredClone(f),make=factory(f),input=operators(f),a=make(f.tools),b=make(f.tools),cold=create(f);
        assert.equal(make.matches(structuredClone(f)),true);assert.equal(Object.isFrozen(make),true);
        pull(input,a);pull(input,b);pull(input,cold);verifyDense(f,a,input);
        for(const key of ['forceMap','slipJacobian','DforceMap']){assert.deepEqual(a[key],cold[key]);assert.deepEqual(b[key],cold[key]);assert.notEqual(a[key].buffer,b[key].buffer);}
        f.layout.positions[0]+=1;if(f.modes.length)f.modes[0].basis[0][0]+=1;
        assert.equal(make.matches(f),false);
        const retained=make(original.tools);pull(input,retained);assert.deepEqual(retained.DforceMap,cold.DforceMap);
        a.commonDofs[0]+=1;assert.throws(()=>pull(input,a),/support was modified/);
        pull(input,b);assert.deepEqual(b.DforceMap,cold.DforceMap);
    }
});
test('cached surface columns bind new material identities without sharing load readiness or mutable storage',()=>{
    const f=fixture(),make=factory(f),a=make(f.tools),input=operators(f);pull(input,a);loads([.7,-.2],a);
    const renamed={...f,tools:f.tools.map(t=>({...t,edgeId:`new:${t.edgeId}`}))},b=make(renamed.tools),newInput=operators(renamed);
    assert.equal(make.diagnostics.builds,1);assert.equal(make.diagnostics.hits,1);
    assert.throws(()=>loads([.7,-.2],b),/current valid/);
    assert.throws(()=>pull(input,b),/tool\/edge order/);
    pull(newInput,b);verifyDense(renamed,b,newInput);
    const expected=structuredClone(loads([-.3,.9],b));
    a.commonDofs.fill(999);assert.throws(()=>pull(input,a),/support was modified/);
    assert.deepEqual(loads([-.3,.9],b),expected);
    const fresh=make(renamed.tools);pull(newInput,fresh);assert.deepEqual(loads([-.3,.9],fresh),expected);
    for(let i=0;i<100;i++)make(renamed.tools.map(t=>({...t,edgeId:`birth:${i}:${t.id}`})));
    assert.equal(make.diagnostics.builds,1);assert.equal(make.diagnostics.retainedPlans,1);
    // Eviction changes preparation cost only. Reversed orders and adjacent
    // physical edges still receive the independent dense scatter after it.
    for(let edge=0;edge<4;edge++)for(const id of ['wire','catheter'])make([{id,edge,edgeId:`${id}:${edge}`}]);
    for(let edge=0;edge<4;edge++)for(let outer=Math.max(0,edge-1);outer<=Math.min(3,edge+1);outer++)for(const reverse of [false,true]){
        const tools=[{id:'wire',edge,edgeId:'wire'},{id:'catheter',edge:outer,edgeId:'catheter'}];make(reverse?tools.toReversed():tools);
    }
    assert.equal(make.diagnostics.retainedPlans,make.diagnostics.capacity);
    const rebuilt=make(renamed.tools);pull(newInput,rebuilt);verifyDense(renamed,rebuilt,newInput);assert.deepEqual(loads([-.3,.9],rebuilt),expected);
});
function fixture({single=null,mixed=false,wall=false,reverse=false}={}) {
    const edges=single?Array.from({length:4},()=>[single]):mixed?[['wire','catheter'],['wire','catheter'],['wire'],['wire']]:Array.from({length:4},()=>['wire','catheter']);
    const layout=createCompositeChainLayout(edges),overlap=Array.from({length:5},(_,n)=>n).filter(n=>new Set([...(edges[n-1]??[]),...(edges[n]??[])]).size===2);
    const modes=overlap.map((node,i)=>{const a=.27+.13*node,c=Math.cos(a),s=Math.sin(a);return {node,basis:[[c,s,0],[-s,c,0],[0,0,1]],relativeDofs:[3*i,3*i+1,3*i+2]};});
    const tools=single?[{id:single,edge:1,edgeId:`${single}:1`}]:mixed?[{id:'wire',edge:2,edgeId:'wire:2'},{id:'catheter',edge:1,edgeId:'catheter:1'}]:
        [{id:'wire',edge:1,edgeId:'wire:1'},{id:'catheter',edge:0,edgeId:'catheter:0'}];
    return {layout,modes,tools:wall?tools.slice(0,1):reverse?tools.toReversed():tools};
}
function denseMap(f,w) {
    // Independent global scatter, then restrict to the published local support.
    const n=f.layout.dofCount,r=3*f.modes.length,full=Array.from({length:7*f.tools.length},()=>new Array(n+r).fill(0));
    f.tools.forEach((t,i)=>{
        for(let end=0;end<2;end++)for(let axis=0;axis<3;axis++) {
            const node=t.edge+end,row=7*i+3*end+axis;full[row][f.layout.positions[node]+axis]=1;
            if(t.id===(f.relativeToolId??(f.layout.spins.size===1?f.tools[0].id:'wire'))) {
                const m=f.modes.find(m=>m.node===node);if(m)for(let j=0;j<3;j++)full[row][n+m.relativeDofs[j]]=m.basis[j][axis];
            }
        }
        full[7*i+6][f.layout.spins.get(t.id)[t.edge]]=1;
    });
    const columns=[...w.commonDofs,...Array.from(w.relativeDofs,d=>n+d)];return full.map(row=>columns.map(d=>row[d]));
}
function operators(f) {
    const p=7*f.tools.length;
    return {tools:f.tools.map(t=>({...t})),slipJacobianValid:true,forceMapValid:true,DforceMapValid:true,
        slipJacobian:Float64Array.from({length:2*p},(_,i)=>.11*Math.cos(.7*i+.3)),
        forceMap:Float64Array.from({length:2*p},(_,i)=>.17*Math.sin(.3*i+.9)),
        DforceMap:Float64Array.from({length:2*p*p},(_,i)=>.023*Math.sin(.71*i+.2))};
}
function verifyDense(f,w,input) {
    const T=denseMap(f,w),p=T.length,n=w.dofCount;
    for(let i=0;i<n;i++)for(let c=0;c<2;c++) {
        close(w.slipJacobian[c*n+i],T.reduce((v,row,k)=>v+input.slipJacobian[c*p+k]*row[i],0));
        close(w.forceMap[2*i+c],T.reduce((v,row,k)=>v+row[i]*input.forceMap[2*k+c],0));
        close(w.rows[c].jacobian[i],w.slipJacobian[c*n+i],0);close(w.rows[c].forceColumn[i],-w.forceMap[2*i+c],0);
        for(let j=0;j<n;j++) {
            let value=0;for(let a=0;a<p;a++)for(let b=0;b<p;b++)value+=T[a][i]*input.DforceMap[(2*a+c)*p+b]*T[b][j];
            close(w.DforceMap[(2*i+c)*n+j],value);close(w.rows[c].forceDerivative[i*n+j],value);
        }
    }
    assert.ok(w.slipJacobian.some((g,i)=>Math.abs(g-w.forceMap[2*(i%n)+Math.floor(i/n)])>1e-3),'G must remain independent of instantaneous B');
    return T;
}

for(const [name,options] of [['adjacent rotated tools',{}],['reversed physical tool order',{reverse:true}],['mixed exposed/overlap edges',{mixed:true}],['wire against wall within a two-tool chart',{wall:true}],
    ['single wire',{single:'wire'}],['single catheter',{single:'catheter'}]]) {
    test(`${name}: dense G/B/DB scatter and virtual power include actual endpoints and own spins`,()=>{
        const f=fixture(options),w=create(f),input=operators(f);assert.equal(pull(input,w),w);
        const T=verifyDense(f,w,input),rate=Array.from({length:w.dofCount},(_,i)=>.2*Math.cos(i+.4)),physicalRate=T.map(row=>dot(row,rate)),Ft=[-.7,.4];
        const result=loads(Ft,w),expected=physicalRate.map((_,i)=>input.forceMap[2*i]*Ft[0]+input.forceMap[2*i+1]*Ft[1]);
        vector(result.physical,expected);close(dot([...result.common,...result.relative],rate),dot(expected,physicalRate));
        f.tools.forEach((t,i)=>{
            assert.equal(result.tools[i].id,t.id);assert.equal(result.tools[i].edge,t.edge);assert.equal(result.tools[i].edgeId,t.edgeId);
            assert.deepEqual(result.tools[i].nodes,Int32Array.of(t.edge,t.edge+1));
            vector(result.tools[i].nodalForces.flatMap(v=>Array.from(v)),expected.slice(7*i,7*i+6));close(result.tools[i].scalarTorque,expected[7*i+6]);
            assert.ok(Math.abs(result.tools[i].scalarTorque)>1e-3);
            assert.ok(w.commonDofs.includes(f.layout.spins.get(t.id)[t.edge]));
        });
        const spinWork=result.tools.reduce((sum,t,i)=>sum+t.scalarTorque*physicalRate[7*i+6],0);
        assert.ok(Math.abs(spinWork)>1e-4,'Power witness must exercise scalar torques');
        assert.equal(w.rows.length,2);assert.ok(w.rows.every(r=>r.operatorReady&&r.forceDerivativeValid));
        if(options.single){assert.equal(w.relativeDofs.length,0);assert.equal(w.dofCount,7);assert.equal(w.loads.tools.length,1);}
    });
}

test('full physical DforceMap maps cross-tool and both spin configuration derivatives without assuming symmetry',()=>{
    const f=fixture(),w=create(f),input=operators(f),T=denseMap(f,w),p=T.length,n=w.dofCount;
    pull(input,w);const D=w.DforceMap.slice(),direction=Array.from({length:n},(_,i)=>Math.sin(i+.9)),dy=T.map(row=>dot(row,direction)),h=1e-5;
    const mapped=[];
    for(const sign of [-1,1]) {
        const changed={...input,forceMap:Float64Array.from(input.forceMap,(v,i)=>v+sign*h*dy.reduce((s,d,j)=>s+input.DforceMap[i*p+j]*d,0))};
        pull(changed,w);mapped.push(w.forceMap.slice());
    }
    for(let i=0;i<2*n;i++)close((mapped[1][i]-mapped[0][i])/(2*h),direction.reduce((s,d,j)=>s+D[i*n+j]*d,0),2e-11);
    const spins=f.tools.map(t=>Array.from(w.commonDofs).indexOf(f.layout.spins.get(t.id)[t.edge]));
    assert.ok(Math.abs(D[(2*spins[0])*n+spins[1]])>1e-4);assert.ok(Math.abs(D[(2*spins[1]+1)*n+spins[0]])>1e-4);
});

function motionInput(f,w) {
    const dt=.02,T=denseMap(f,w),global=new Float64Array(f.layout.dofCount+3*f.modes.length);
    for(let node=0;node<f.layout.nodeCount;node++)for(let k=0;k<3;k++)global[f.layout.positions[node]+k]=k===0?2*node:k===1?.03*node*node:.01*node;
    for(const [id,spins] of f.layout.spins)spins.forEach((d,e)=>{if(d>=0)global[d]=(id==='wire'?.2:-.3)+.01*e;});
    f.modes.forEach(m=>m.relativeDofs.forEach((d,j)=>global[f.layout.dofCount+d]=[.08,.23,-.07][j]));
    const support=[...w.commonDofs,...Array.from(w.relativeDofs,d=>f.layout.dofCount+d)],z=support.map(d=>global[d]);
    const rate=Array.from({length:w.dofCount},(_,i)=>.07*Math.cos(i+.4)),physical=T.map(row=>dot(row,z)),physicalRates=T.map(row=>dot(row,rate));
    const tools=f.tools.map((t,i)=>{
        const positions=[physical.slice(7*i,7*i+3),physical.slice(7*i+3,7*i+6)],st=i===0?-.2:.1,start=20+10*i;
        return {id:t.id,edgeId:t.edgeId,coordinates:[2*t.edge,2*t.edge+2],coordinate:2*t.edge+1,
            positions,previousPositions:structuredClone(positions),positionRates:[physicalRates.slice(7*i,7*i+3),physicalRates.slice(7*i+3,7*i+6)],
            angle:physical[7*i+6],previousAngle:physical[7*i+6]-.03,angleRate:physicalRates[7*i+6],reference:captureCompositeReferenceFrames(positions)[0],
            materialMap:{sStart:start,dsDx:1.1+.2*i,dsDt:st},materialPath:{kind:'linear-affine-maps',previousEdgeId:t.edgeId,previousMap:{sStart:start-dt*st,dsDx:1.1+.2*i}}};
    });
    return {rate,input:{dt,rateMode:'instantaneous',contact:{point:[2.7,.6,.2],axes:[[1,0,0],[0,0,1]]},tools,
        ...(tools.length===1?{wall:{velocity:[.03,-.02,.04]}}:{})}};
}
for(const single of [null,'wire','catheter'])test(`frozen instantaneous SurfaceMotion -> ${single??'two distinct tools'} loads preserves power, including prescribed feed/wall`,()=>{
    const f=fixture({single}),w=create(f),{rate,input}=motionInput(f,w),r=motion(input),Ft=[.3,-.8];
    // This provider has no finite slip Jacobian. Map its instantaneous B only;
    // never rename the configuration/rate Jacobian as a finite derivative.
    pull({tools:r.tools,forceMap:r.forceMap,forceMapValid:true},w);const result=loads(Ft,w);
    close(dot([...result.common,...result.relative],rate)+dot(Ft,r.prescribedSlipRate),dot(Ft,r.slipRate),3e-12);
    assert.equal(w.operatorReady,false);assert.equal(w.slipJacobianValid,false);assert.equal(w.DforceMapValid,false);
    assert.ok(w.rows.every(row=>!row.operatorReady&&row.forceColumnValid&&!row.jacobianValid&&row.jacobian.every(Number.isNaN)));
    const physicalRates=input.tools.flatMap(t=>t.positionRates.flat().concat(t.angleRate));
    close(dot(result.physical,physicalRates),dot([...result.common,...result.relative],rate));
    result.tools.forEach((t,i)=>close(t.scalarTorque,r.forceMap[2*(7*i+6)]*Ft[0]+r.forceMap[2*(7*i+6)+1]*Ft[1]));
    assert.ok(result.tools.every(t=>Math.abs(t.scalarTorque)>1e-4));
});

test('prepared bases, edge records and input operators are owned; arrays reuse and force-only refresh invalidate stale derivatives/loads',()=>{
    const f=fixture(),w=create(f),input=operators(f),before=structuredClone(input);pull(input,w);
    const B=w.forceMap.slice(),G=w.slipJacobian.slice(),D=w.DforceMap.slice(),Ft=[.2,.6],first=structuredClone(loads(Ft,w));
    f.modes[0].basis[0][0]=999;f.tools[0].edge=99;input.forceMap.fill(999);input.slipJacobian.fill(999);input.DforceMap.fill(999);
    assert.deepEqual(loads(Ft,w),first);assert.deepEqual(w.forceMap,B);assert.deepEqual(w.slipJacobian,G);assert.deepEqual(w.DforceMap,D);
    const arrays=[w.forceMap,w.slipJacobian,w.DforceMap,w.rows[0].forceColumn,w.loads.common];
    pull({tools:before.tools,forceMap:before.forceMap,forceMapValid:true},w);
    assert.equal(w.loads.valid,false);assert.ok(w.loads.common.every(Number.isNaN));
    assert.ok(w.DforceMap.every(Number.isNaN));assert.ok(w.rows.every(r=>r.forceDerivative.every(Number.isNaN)&&!r.forceDerivativeValid));
    arrays.forEach((a,i)=>assert.equal(a,[w.forceMap,w.slipJacobian,w.DforceMap,w.rows[0].forceColumn,w.loads.common][i]));
    assert.deepEqual(loads(Ft,w),first);
});

test('missing physical spin/owner, incomplete modes, forged chart and out-of-stencil pairs reject',()=>{
    const f=fixture();
    assert.throws(()=>create({...f,modes:f.modes.slice(1)}),/every overlap/);
    const reduced=structuredClone(f);reduced.modes[0].basis.pop();assert.throws(()=>create(reduced),/Complete ordered 3D/);
    const spin=structuredClone(f);spin.layout.spins.get('wire')[1]=-1;assert.throws(()=>create(spin),/positions\/spins/);
    const owner=fixture({mixed:true});owner.tools[1].edge=2;assert.throws(()=>create(owner),/actual material edge/);
    assert.throws(()=>create({...f,tools:[f.tools[0],f.tools[0]]}),/distinct actual/);
    assert.throws(()=>create({...f,tools:[{...f.tools[0],edgeId:undefined}]}),/provider edge ID/);
    assert.throws(()=>create({...f,tools:[{...f.tools[0],edge:0},{...f.tools[1],edge:3}]}),/two-edge stencil/);
    const one=fixture({single:'catheter'});assert.throws(()=>create({...one,relativeToolId:'wire'}),/relative material/);
});

test('stale validity, wrong physical identity/dimensions, nonfinite/overflow operators and failed load calls cannot publish stale data',()=>{
    const f=fixture(),w=create(f),good=operators(f);
    const bad=[{...good,forceMapValid:false},{...good,slipJacobianValid:false},{...good,DforceMapValid:false},
        {...good,tools:[...good.tools].reverse()},{...good,tools:good.tools.map(t=>({...t,edgeId:'unmapped'}))},
        {...good,tools:good.tools.map(t=>({...t,edge:99}))},
        {...good,forceMap:Float64Array.of(1,2)},{...good,slipJacobian:new Float64Array(good.slipJacobian.length).fill(NaN)},
        {...good,DforceMap:new Float64Array(good.DforceMap.length).fill(Infinity)},
        {...good,forceMap:new Float64Array(good.forceMap.length).fill(Number.MAX_VALUE)},
        {...good,slipJacobian:undefined},{...good,DforceMap:undefined}];
    for(const input of bad) {
        pull(good,w);loads([.2,.6],w);assert.throws(()=>pull(input,w));
        assert.equal(w.operatorReady,false);assert.equal(w.forceMapValid,false);assert.equal(w.loads.valid,false);
        assert.ok(w.forceMap.every(Number.isNaN));assert.ok(w.rows.every(r=>!r.operatorReady&&r.forceColumn.every(Number.isNaN)));
        assert.throws(()=>loads([.2,.6],w),/current valid/);
    }
    pull(good,w);loads([.2,.6],w);assert.throws(()=>loads([Infinity,0],w),/finite/);assert.equal(w.loads.valid,false);
    assert.ok(w.loads.tools.every(t=>Number.isNaN(t.scalarTorque)&&t.nodalForces.every(v=>v.every(Number.isNaN))));
    assert.equal(loads([.2,.6],w).valid,true);
    w.commonDofs[0]=999;assert.throws(()=>pull(good,w),/support was modified/);assert.throws(()=>loads([.2,.6],w),/support was modified/);
});
