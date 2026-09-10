import assert from 'node:assert/strict';
import test from 'node:test';
import {createCompositeChainLayout} from '../src/physics/kirchhoffCompositeChain.js';
import {createCompositeJointSurfacePoseHistory,prepareCompositeJointSurfacePosePath} from '../src/physics/kirchhoffCompositeJointSurfacePoseHistory.js';
import {createCompositeJointReservoirSurfaceWorkspace,evaluateCompositeJointReservoirSurface} from '../src/physics/kirchhoffCompositeJointReservoirSurface.js';
import {createCompositeJointPhysicalColumnPullback,pullbackCompositeJointPhysicalColumns,evaluateCompositeJointPhysicalColumnLoads} from '../src/physics/kirchhoffCompositeJointPhysicalColumnPullback.js';

const close=(a,b,t=2e-11)=>assert.ok(Number.isFinite(a)&&Number.isFinite(b)&&Math.abs(a-b)<=t,`${a} != ${b}`);
const dot=(a,b)=>a.reduce((s,v,i)=>s+v*b[i],0);

function fixture(){
    const columns=[...Array.from({length:3},(_,node)=>[0,1,2].map(component=>({kind:'position',toolId:'wire',node,component}))).flat(),
        ...[0,1].map(edge=>({kind:'angle',toolId:'wire',edge}))];
    const edges=[-1,0,1].map(edge=>({edgeId:`wire:${edge}`,materialSegmentId:edge<0?'external':42,source:edge<0?'reservoir':'accepted',
        ...(edge<0?{}:{edge}),nodeIds:[edge,edge+1],coordinates:[edge,edge+1],labels:[edge,edge+1],reference:{tangent:[1,0,0],director:[0,1,0]},angle:0}));
    const history=createCompositeJointSurfacePoseHistory({toolId:'wire',reservoirIdentity:17,
        nodes:[-1,0,1,2].map(node=>({id:node,position:[node,0,0],...(node<0?{}:{node})})),edges,
        hinges:[{leftEdgeId:'wire:-1',rightEdgeId:'wire:0',referenceTwist:0},{leftEdgeId:'wire:0',rightEdgeId:'wire:1',referenceTwist:0}]});
    const weights=(component,weight)=>[0,1,2].map(k=>k===component?weight:0);
    const path=prepareCompositeJointSurfacePosePath({history,targetEdgeId:'wire:0',dt:.1,reservoirIdentity:17,
        currentMaps:edges.map(e=>({edgeId:e.edgeId,labels:e.labels.map(s=>s-.03),dsDt:-.3})),configurationColumns:columns,
        nodeBindings:[{nodeId:-1,offset:[0,0,0],terms:[0,1,2].flatMap(component=>[
            {column:component,weights:weights(component,1)},
            {column:3+component,weights:weights(component,1)},
            {column:6+component,weights:weights(component,-1)}])},
            ...[0,1,2].map(node=>({nodeId:node,offset:[0,0,0],terms:[0,1,2].map(component=>({column:3*node+component,weights:weights(component,1)}))}))],
        angleBindings:[{edgeId:'wire:-1',offset:0,terms:[{column:10,weight:1}]},
            ...[0,1].map(edge=>({edgeId:`wire:${edge}`,offset:0,terms:[{column:9+edge,weight:1}]}))]});
    const surface=createCompositeJointReservoirSurfaceWorkspace(path),layout=createCompositeChainLayout([['wire','catheter'],['wire','catheter']]);
    const modes=[0,1,2].map(node=>{const a=.3+.1*node,c=Math.cos(a),s=Math.sin(a);return {node,basis:[[c,s,0],[-s,c,0],[0,0,1]],relativeDofs:[3*node,3*node+1,3*node+2]};});
    const mapping=createCompositeJointPhysicalColumnPullback({layout,modes,currentTools:surface.currentTools,configurationColumns:surface.configurationColumns});
    // Independently reconstruct each physical coordinate from q, rho and its
    // OWN spin. This also leaves every physical Dirichlet reaction available.
    const M=mapping.dofCount,T=columns.map(column=>{
        const row=new Array(M).fill(0),common=column.kind==='angle'?layout.spins.get('wire')[column.edge]:layout.positions[column.node]+column.component;
        row[Array.from(mapping.commonDofs).indexOf(common)]=1;
        if(column.kind==='position')modes[column.node].basis.forEach((basis,k)=>{
            row[mapping.commonDofs.length+Array.from(mapping.relativeDofs).indexOf(3*column.node+k)]=basis[column.component];
        });return row;
    });
    const z=Array.from({length:M},(_,i)=>{
        if(i>=mapping.commonDofs.length)return .006*Math.cos(i);
        const d=mapping.commonDofs[i];for(let node=0;node<3;node++)for(let k=0;k<3;k++)if(d===layout.positions[node]+k)return k===0?node:k===1?.01*(node+1):0;
        return d===layout.spins.get('wire')[0]?.09:-.05;
    });
    const input={query:{coordinate:.015},finiteGeometry:{kind:'explicit-affine-side-queries',
        current:{point:[.015,.2,0],normal:[0,1,0],tangent:[1,0,0]},previous:{point:[.015,.2,0],normal:[0,1,0],tangent:[1,0,0]}}};
    function at(values,order='full'){
        const result=evaluateCompositeJointReservoirSurface({...input,configuration:T.map(row=>dot(row,values)),order},surface);
        pullbackCompositeJointPhysicalColumns({currentTools:result.currentTools,configurationColumns:result.configurationColumns,
            forceMap:result.forceMap,forceMapValid:result.forceMapValid,
            ...(order==='full'?{slipJacobian:result.configurationJacobian,slipJacobianValid:result.configurationJacobianValid,
                DforceMap:result.DforceMap,DforceMapValid:result.DforceMapValid}:{})},mapping);
        return {increment:result.increment.slice(),G:mapping.slipJacobian.slice(),B:mapping.forceMap.slice(),DB:mapping.DforceMap.slice()};
    }
    return {surface,mapping,T,z,at,M};
}

test('actual reservoir provider and joint pullback preserve every expanded history column and all mapped derivatives',()=>{
    const f=fixture(),base=f.at(f.z),h=1e-6,Ft=[.3,-.8];
    assert.equal(f.surface.events.length,1);assert.equal(f.surface.configurationDofs,11);
    for(const i of [6,7,8,10]){assert.equal(f.surface.forceMap[2*i],0);assert.equal(f.surface.forceMap[2*i+1],0);}
    assert.ok([6,7,8,10].some(i=>Math.abs(f.surface.configurationJacobian[i])+Math.abs(f.surface.configurationJacobian[11+i])>1e-5));
    const loads=evaluateCompositeJointPhysicalColumnLoads(Ft,f.mapping),rates=f.z.map((_,i)=>.1*Math.cos(i));
    close(dot([...loads.common,...loads.relative],rates),dot(loads.physical,f.T.map(row=>dot(row,rates))));
    assert.ok(Math.abs(loads.tools[0].nodalForces[0][0])>1e-3);
    // These finite differences run through BOTH production modules. Fixed
    // explicit wall queries have no omitted geometry dependency in this case.
    for(let j=0;j<f.M;j++){
        const plus=f.z.slice(),minus=f.z.slice();plus[j]+=h;minus[j]-=h;
        const a=f.at(plus),b=f.at(minus);
        for(let c=0;c<2;c++)close((a.increment[c]-b.increment[c])/(2*h),base.G[c*f.M+j],5e-8);
        for(let i=0;i<2*f.M;i++)close((a.B[i]-b.B[i])/(2*h),base.DB[i*f.M+j],5e-8);
    }
});

test('reservoir full/value bridge omits invalid derivatives and rejects a failed provider before loads can be reused',()=>{
    const f=fixture(),full=f.at(f.z),value=f.at(f.z,'value');
    assert.deepEqual(value.increment,full.increment);assert.deepEqual(value.B,full.B);
    assert.equal(f.mapping.operatorReady,false);assert.ok(f.mapping.slipJacobian.every(Number.isNaN));
    const invalid=f.z.slice();invalid[0]=NaN;assert.throws(()=>f.at(invalid));assert.equal(f.surface.forceMapValid,false);
    assert.throws(()=>pullbackCompositeJointPhysicalColumns({currentTools:f.surface.currentTools,configurationColumns:f.surface.configurationColumns,
        forceMap:f.surface.forceMap,forceMapValid:f.surface.forceMapValid},f.mapping));
    assert.throws(()=>evaluateCompositeJointPhysicalColumnLoads([.3,-.8],f.mapping));
    const retry=f.at(f.z);assert.deepEqual(retry,full);
});
