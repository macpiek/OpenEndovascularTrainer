import assert from 'node:assert/strict';
import test from 'node:test';
import {createCompositeChainLayout} from '../src/physics/kirchhoffCompositeChain.js';
import {captureCompositeReferenceFrames} from '../src/physics/kirchhoffCompositeElement.js';
import {evaluateCompositeJointSurfaceForceMap} from '../src/physics/kirchhoffCompositeJointSurfaceMotion.js';
import {createCompositeJointPhysicalColumnPullback as create,pullbackCompositeJointPhysicalColumns as pull,evaluateCompositeJointPhysicalColumnLoads as loads} from '../src/physics/kirchhoffCompositeJointPhysicalColumnPullback.js';
const close=(a,b,t=2e-12)=>assert.ok(Number.isFinite(a)&&Number.isFinite(b)&&Math.abs(a-b)<=t,`${a} != ${b} (${t})`);
const vec=(a,b,t)=>{assert.equal(a.length,b.length);a.forEach((v,i)=>close(v,b[i],t));};
const dot=(a,b)=>a.reduce((s,v,i)=>s+v*b[i],0),add=(a,b)=>a.map((v,i)=>v+b[i]);
const cross=(a,b)=>[a[1]*b[2]-a[2]*b[1],a[2]*b[0]-a[0]*b[2],a[0]*b[1]-a[1]*b[0]];
const key=c=>c.kind==='position'?`${c.toolId}:q:${c.node}:${c.component}`:`${c.toolId}:a:${c.edge}`;
const positions=(id,nodes)=>nodes.flatMap(node=>[0,1,2].map(component=>({kind:'position',toolId:id,node,component})));
const angles=(id,edges)=>edges.map(edge=>({kind:'angle',toolId:id,edge}));
function fixture({single='wire',pair=false,mixed=false,reverse=false,partial=false}={}) {
    const edges=pair?(mixed?[['wire','catheter'],['wire'],['wire'],['wire']]:Array.from({length:4},()=>['wire','catheter'])):Array.from({length:4},()=>[single]);
    const layout=createCompositeChainLayout(edges),nodes=Array.from({length:5},(_,node)=>node).filter(n=>new Set([...(edges[n-1]??[]),...(edges[n]??[])]).size===2);
    const modes=nodes.map((node,i)=>{const a=.27+.13*node,c=Math.cos(a),s=Math.sin(a);return {node,basis:[[c,s,0],[-s,c,0],[0,0,1]],relativeDofs:[3*i,3*i+1,3*i+2]};});
    let currentTools=pair?[{id:'wire',edge:1,edgeId:'wire:1',materialSegmentId:17n},{id:'catheter',edge:0,edgeId:'catheter:0',materialSegmentId:'17'}]:[{id:single,edge:1,edgeId:`${single}:1`,materialSegmentId:17n}];
    let configurationColumns=currentTools.flatMap(t=>t.id==='catheter'&&mixed?positions(t.id,[0,1]).concat(angles(t.id,[0])):
        partial?positions(t.id,[1,2]).concat([{kind:'position',toolId:t.id,node:0,component:0}],angles(t.id,[0,1])):
        positions(t.id,[0,1,2]).concat(angles(t.id,[0,1])));
    if(reverse){currentTools=currentTools.toReversed();configurationColumns=configurationColumns.toReversed();}
    return {layout,modes,currentTools,configurationColumns};
}
function denseMap(f,w) {
    const n=f.layout.dofCount,r=3*f.modes.length,full=f.configurationColumns.map(c=>{
        const row=new Array(n+r).fill(0);
        if(c.kind==='angle')row[f.layout.spins.get(c.toolId)[c.edge]]=1;
        else {row[f.layout.positions[c.node]+c.component]=1;
            if(c.toolId===(f.relativeToolId??(f.layout.spins.size===1?f.currentTools[0].id:'wire'))){const m=f.modes.find(v=>v.node===c.node);if(m)m.relativeDofs.forEach((d,i)=>row[n+d]=m.basis[i][c.component]);}}
        return row;
    });
    const columns=[...w.commonDofs,...Array.from(w.relativeDofs,d=>n+d)];return full.map(row=>columns.map(d=>row[d]));
}
function operators(f) {
    const N=f.configurationColumns.length;
    return {currentTools:structuredClone(f.currentTools),configurationColumns:structuredClone(f.configurationColumns),slipJacobianValid:true,forceMapValid:true,DforceMapValid:true,
        slipJacobian:Float64Array.from({length:2*N},(_,i)=>.11*Math.cos(.7*i+.3)),forceMap:Float64Array.from({length:2*N},(_,i)=>.17*Math.sin(.3*i+.9)),
        DforceMap:Float64Array.from({length:2*N*N},(_,i)=>.023*Math.sin(.71*i+.2))};
}
function verifyDense(f,w,input) {
    const T=denseMap(f,w),N=T.length,M=w.dofCount;
    for(let i=0;i<M;i++)for(let c=0;c<2;c++) {
        close(w.slipJacobian[c*M+i],T.reduce((s,row,k)=>s+input.slipJacobian[c*N+k]*row[i],0));
        close(w.forceMap[2*i+c],T.reduce((s,row,k)=>s+input.forceMap[2*k+c]*row[i],0));
        close(w.rows[c].jacobian[i],w.slipJacobian[c*M+i],0);close(w.rows[c].forceColumn[i],-w.forceMap[2*i+c],0);
        for(let j=0;j<M;j++) {let d=0;for(let a=0;a<N;a++)for(let b=0;b<N;b++)d+=T[a][i]*input.DforceMap[(2*a+c)*N+b]*T[b][j];
            close(w.DforceMap[(2*i+c)*M+j],d);close(w.rows[c].forceDerivative[i*M+j],d);}
    }
    return T;
}
function physicalLoadVector(f,result) {
    return f.configurationColumns.map(c=>{const t=result.tools.find(t=>t.id===c.toolId);return c.kind==='position'?
        t.nodalForces[Array.from(t.nodes).indexOf(c.node)][c.component]:t.spinTorques[Array.from(t.edges).indexOf(c.edge)];});
}
for(const [name,opts] of [['single wire 11',{}],['single catheter 11',{single:'catheter'}],['two tools 22 with opposite current/history edges',{pair:true}],
    ['reversed producer order',{pair:true,reverse:true}],['exposed wire with shorter catheter history',{pair:true,mixed:true}],['partial neighboring position columns',{partial:true}]])
    test(`${name}: dense independent scatter preserves every G/B/DB column and actual physical virtual work`,()=>{
        const f=fixture(opts),w=create(f),input=operators(f);assert.equal(pull(input,w),w);const T=verifyDense(f,w,input),rate=Array.from({length:w.dofCount},(_,i)=>.2*Math.cos(i+.4)),yRate=T.map(row=>dot(row,rate)),Ft=[-.7,.4];
        const result=loads(Ft,w),expected=yRate.map((_,i)=>input.forceMap[2*i]*Ft[0]+input.forceMap[2*i+1]*Ft[1]);
        vec(result.physical,expected);vec(physicalLoadVector(f,result),expected);close(dot([...result.common,...result.relative],rate),dot(expected,yRate));
        assert.ok(w.rows.every(r=>r.operatorReady&&r.jacobianValid&&r.forceColumnValid&&r.forceDerivativeValid));assert.equal(w.anchorNode,0);
        assert.ok(w.slipJacobian.some((g,i)=>Math.abs(g-w.forceMap[2*(i%w.dofCount)+Math.floor(i/w.dofCount)])>.001));
        if(!opts.pair&&!opts.partial){assert.equal(w.physicalDofCount,11);assert.equal(w.dofCount,11);assert.deepEqual(result.tools[0].nodes,Int32Array.of(0,1,2));assert.deepEqual(result.tools[0].edges,Int32Array.of(0,1));}
    });

function currentForce(f) {
    const point=[2.7,.6,.2],tools=f.currentTools.map((t,i)=>{
        const p=[t.edge,t.edge+1].map(node=>[2*node,.3-.2*i,.05*node]);
        return {...t,positions:p,previousPositions:structuredClone(p),coordinates:[2*t.edge,2*t.edge+2],coordinate:2*t.edge+1,
            reference:captureCompositeReferenceFrames(p)[0],angle:.2-.3*i};
    });
    const original=evaluateCompositeJointSurfaceForceMap({tools,forceGeometry:{kind:'explicit-affine-side-query',point,normal:[0,1,0],tangent:[1,0,0]}}),N=f.configurationColumns.length;
    const originalColumns=f.currentTools.flatMap(t=>positions(t.id,[t.edge,t.edge+1]).concat(angles(t.id,[t.edge]))),index=new Map(f.configurationColumns.map((c,i)=>[key(c),i]));
    const result={...operators(f),forceMap:new Float64Array(2*N),DforceMap:new Float64Array(2*N*N)};
    originalColumns.forEach((a,i)=>{const ri=index.get(key(a));for(let c=0;c<2;c++){result.forceMap[2*ri+c]=original.forceMap[2*i+c];
        originalColumns.forEach((b,j)=>result.DforceMap[(2*ri+c)*N+index.get(key(b))]=original.configurationDerivative[(2*i+c)*originalColumns.length+j]);}});
    return {result,original,tools,point,originalKeys:new Set(originalColumns.map(key))};
}
for(const pair of [false,true])test(`actual instantaneous current-edge B embedded in expanded ${pair?22:11} columns preserves wrench and boundary reactions with nonzero upstream G`,()=>{
    const f=fixture({pair}),w=create(f),{result:input,original,tools,point,originalKeys}=currentForce(f),Ft=[.7,-.4];pull(input,w);
    const output=loads(Ft,w);let netForce=[0,0,0],netMoment=[0,0,0];
    for(const [i,t] of output.tools.entries()) {
        const p=tools[i].positions,world=original.axes[0].map((v,k)=>(i===0?1:-1)*(v*Ft[0]+original.axes[1][k]*Ft[1]));
        const force=t.nodalForces.reduce((s,v)=>add(s,Array.from(v)),[0,0,0]);vec(force,world,3e-13);
        const tangent=p[1].map((v,k)=>v-p[0][k]),length=Math.hypot(...tangent);tangent.forEach((_,k)=>tangent[k]/=length);
        const spin=t.spinTorques[Array.from(t.edges).indexOf(tools[i].edge)];let moment=tangent.map(v=>v*spin);
        t.nodes.forEach((node,j)=>{const position=[2*node,.3-.2*i,.05*node];moment=add(moment,cross(position,Array.from(t.nodalForces[j])));});
        vec(moment,cross(point,world),5e-13);netForce=add(netForce,force);netMoment=add(netMoment,moment);
        assert.ok(Math.abs(spin)>1e-4);
    }
    if(pair){vec(netForce,[0,0,0],5e-13);vec(netMoment,[0,0,0],8e-13);}
    for(let i=0;i<f.configurationColumns.length;i++)if(!originalKeys.has(key(f.configurationColumns[i]))){assert.ok(input.forceMap[2*i]===0&&input.forceMap[2*i+1]===0);assert.ok(output.physical[i]===0);assert.ok(Math.abs(input.slipJacobian[i])>1e-5);}
    const upstream=Array.from(w.commonDofs).indexOf(f.layout.positions[0]);assert.ok(Math.abs(w.slipJacobian[upstream])>1e-5);
    // Current body coordinates are retained even when a caller later fixes all
    // of them: their nonzero reactions remain in the physical/local outputs.
    const current=tools[0],fixedNode=current.edge,local=Array.from(w.commonDofs).indexOf(f.layout.positions[fixedNode]);assert.ok(local>=0);assert.ok(Math.abs(output.common[local])>1e-4);
});

test('finite scalar and physical vector FD retain arbitrary cross-history G and nonsymmetric DB in all mapped columns',()=>{
    const f=fixture({pair:true,reverse:true}),w=create(f),base=operators(f),T=denseMap(f,w),N=T.length,M=w.dofCount;
    const z=Array.from({length:M},(_,i)=>.07*Math.cos(i+.4)),y=T.map(row=>dot(row,z)),a=Array.from({length:N},(_,i)=>.03*Math.sin(.7*i+.2)),b=a.map((v,i)=>.5*v+.02*Math.cos(i));
    function at(physical) {
        const ay=dot(a,physical),by=dot(b,physical),increment=[dot(Array.from(base.slipJacobian.slice(0,N)),physical)+ay*ay,dot(Array.from(base.slipJacobian.slice(N)),physical)+Math.sin(by)];
        return {increment,...base,slipJacobian:Float64Array.from(base.slipJacobian,(v,i)=>v+(i<N?2*ay*a[i]:Math.cos(by)*b[i-N])),
            forceMap:Float64Array.from(base.forceMap,(v,i)=>v+physical.reduce((s,x,j)=>s+base.DforceMap[i*N+j]*x,0))};
    }
    pull(at(y),w);const G=w.slipJacobian.slice(),D=w.DforceMap.slice(),Ft=[.4,-.7],h=1e-6;
    for(let col=0;col<M;col++) {
        const samples=[];
        for(const sign of [-1,1]){const changed=y.map((v,i)=>v+sign*h*T[i][col]),input=at(changed);pull(input,w);samples.push({increment:input.increment,B:w.forceMap.slice(),force:Array.from(loads(Ft,w).common).concat(Array.from(w.loads.relative))});}
        for(let c=0;c<2;c++)close((samples[1].increment[c]-samples[0].increment[c])/(2*h),G[c*M+col],2e-10);
        for(let i=0;i<M;i++)for(let c=0;c<2;c++)close((samples[1].B[2*i+c]-samples[0].B[2*i+c])/(2*h),D[(2*i+c)*M+col],2e-10);
        for(let i=0;i<M;i++)close((samples[1].force[i]-samples[0].force[i])/(2*h),D[(2*i)*M+col]*Ft[0]+D[(2*i+1)*M+col]*Ft[1],3e-10);
    }
});

test('value, partial derivatives and reused storage retain a private B snapshot and cannot fabricate a full operator',()=>{
    const f=fixture({pair:true}),w=create(f),input=operators(f),snapshot=structuredClone(input);pull(input,w);const Ft=[.3,.2],expected=structuredClone(loads(Ft,w));
    const arrays=[w.forceMap,w.slipJacobian,w.DforceMap,w.rows[0].forceDerivative,w.loads.physical,w.loads.tools[0].spinTorques];
    input.forceMap.fill(999);w.forceMap.fill(333);f.modes[0].basis[0][0]=123;f.currentTools[0].edge=99;f.configurationColumns[0].node=99;
    assert.deepEqual(loads(Ft,w),expected);assert.deepEqual(loads(w.loads.Ft,w),expected);
    const value={currentTools:snapshot.currentTools,configurationColumns:snapshot.configurationColumns,forceMap:snapshot.forceMap,forceMapValid:true};pull(value,w);
    assert.equal(w.operatorReady,false);assert.equal(w.loads.valid,false);assert.ok(w.slipJacobian.every(Number.isNaN));assert.ok(w.DforceMap.every(Number.isNaN));assert.deepEqual(loads(Ft,w),expected);
    pull({...value,slipJacobian:snapshot.slipJacobian,slipJacobianValid:true},w);assert.equal(w.slipJacobianValid,true);assert.equal(w.operatorReady,false);assert.equal(w.DforceMapValid,false);
    pull({...value,DforceMap:snapshot.DforceMap,DforceMapValid:true},w);assert.equal(w.slipJacobianValid,false);assert.equal(w.operatorReady,false);assert.equal(w.DforceMapValid,true);
    pull(snapshot,w);assert.equal(w.operatorReady,true);arrays.forEach((v,i)=>assert.equal(v,[w.forceMap,w.slipJacobian,w.DforceMap,w.rows[0].forceDerivative,w.loads.physical,w.loads.tools[0].spinTorques][i]));
    const cold=create({...fixture({pair:true})});pull(snapshot,cold);assert.deepEqual(loads(Ft,w),loads(Ft,cold));
});

test('missing current body coordinates, incomplete modes, inactive identities, duplicate columns and over-wide support reject at preparation',()=>{
    const f=fixture({pair:true,mixed:true});
    const mutate=[x=>x.configurationColumns.splice(x.configurationColumns.findIndex(c=>c.kind==='position'&&c.toolId==='wire'&&c.node===2&&c.component===1),1),
        x=>x.configurationColumns.splice(x.configurationColumns.findIndex(c=>c.kind==='angle'&&c.toolId==='wire'&&c.edge===1),1),
        x=>x.configurationColumns.push({...x.configurationColumns[0]}),x=>x.configurationColumns.push({kind:'position',toolId:'wall',node:0,component:0}),
        x=>x.configurationColumns.push({kind:'position',toolId:'catheter',node:2,component:0}),x=>x.configurationColumns.push({kind:'angle',toolId:'catheter',edge:1}),
        x=>x.configurationColumns.push({kind:'angle',toolId:'wire',edge:2}),x=>x.configurationColumns.push({kind:'position',toolId:'wire',node:3,component:0}),
        x=>x.configurationColumns[0].component=3,x=>x.configurationColumns[0].kind='unknown',x=>x.currentTools.push({...x.currentTools[0]}),
        x=>x.currentTools[0].edgeId=undefined,x=>x.modes.pop(),x=>x.modes[0].basis.pop(),x=>x.layout.spins.get('wire')[1]=-1];
    for(const change of mutate){const bad=structuredClone(f);change(bad);assert.throws(()=>create(bad));}
    assert.throws(()=>create({...fixture({single:'catheter'}),relativeToolId:'wire'}),/relative material/);
});

test('source identity, dimension, validity, numeric failures and caller mutations revoke every row and load before retry',()=>{
    const f=fixture({pair:true}),w=create(f),good=operators(f),bad=[{...good,forceMapValid:false},{...good,slipJacobianValid:false},{...good,DforceMapValid:false},
        {...good,currentTools:good.currentTools.toReversed()},{...good,currentTools:good.currentTools.map(t=>({...t,edgeId:'changed'}))},
        {...good,configurationColumns:good.configurationColumns.toReversed()},{...good,configurationColumns:good.configurationColumns.map((c,i)=>i===0?{...c,component:2}:c)},
        {...good,forceMap:Float64Array.of(1,2)},{...good,slipJacobian:new Float64Array(good.slipJacobian.length).fill(NaN)},
        {...good,DforceMap:new Float64Array(good.DforceMap.length).fill(Infinity)},{...good,forceMap:new Float64Array(good.forceMap.length).fill(Number.MAX_VALUE)},
        {...good,slipJacobian:undefined},{...good,DforceMap:undefined}];
    for(const input of bad){pull(good,w);loads([.3,.4],w);assert.throws(()=>pull(input,w));assert.equal(w.forceMapValid,false);assert.equal(w.loads.valid,false);
        assert.ok(w.forceMap.every(Number.isNaN));assert.ok(w.rows.every(r=>r.jacobian.every(Number.isNaN)&&r.forceColumn.every(Number.isNaN)&&r.forceDerivative.every(Number.isNaN)));
        w.forceMapValid=true;assert.throws(()=>loads([.3,.4],w),/current valid/);}
    pull(good,w);assert.throws(()=>loads([Infinity,0],w),/finite/);assert.equal(w.loads.valid,false);assert.ok(w.loads.tools.every(t=>t.spinTorques.every(Number.isNaN)&&t.nodalForces.every(v=>v.every(Number.isNaN))));
    assert.equal(loads([.3,.4],w).valid,true);w.loads.tools[0].edges[0]=99;assert.throws(()=>loads([.3,.4],w),/support was modified/);
    const other=create(f);other.commonDofs[0]=99;assert.throws(()=>pull(good,other),/support was modified/);
});
