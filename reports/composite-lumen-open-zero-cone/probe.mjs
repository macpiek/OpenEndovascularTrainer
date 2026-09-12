// Frozen A/B: every dependency is identical except the owned manager.
// Usage: node probe.mjs BASELINE_STAGE CANDIDATE_STAGE [output.json]
import assert from 'node:assert/strict';
import {pathToFileURL} from 'node:url';
import {readFileSync,writeFileSync} from 'node:fs';
import {resolve} from 'node:path';
import {createHash} from 'node:crypto';
import {performance} from 'node:perf_hooks';
const roots=process.argv.slice(2,4);assert.equal(roots.length,2);
const load=(root,path)=>import(pathToFileURL(resolve(root,path)));
const hash=x=>createHash('sha256').update(x).digest('hex');
const sha=(root,path)=>hash(readFileSync(resolve(root,path)));
const encode=x=>x instanceof Map?{$map:[...x].map(([k,v])=>[k,encode(v)])}:ArrayBuffer.isView(x)?{$type:x.constructor.name,values:Array.from(x)}:Array.isArray(x)?x.map(encode):x&&typeof x==='object'?Object.fromEntries(Object.entries(x).map(([k,v])=>[k,encode(v)])):x;
const digest=x=>hash(JSON.stringify(encode(x)));
function maxDifference(a,b){
    if(typeof a==='number'&&typeof b==='number'){if(Object.is(a,b))return 0;assert.ok(Number.isFinite(a)&&Number.isFinite(b));return Math.abs(a-b);}
    if(a instanceof Map){assert.deepEqual([...a.keys()],[...b.keys()]);return Math.max(0,...[...a].map(([k,v])=>maxDifference(v,b.get(k))));}
    if(ArrayBuffer.isView(a)||Array.isArray(a)){assert.equal(a.length,b.length);return Math.max(0,...Array.from(a,(v,i)=>maxDifference(v,b[i])));}
    if(a&&typeof a==='object'){assert.deepEqual(Object.keys(a),Object.keys(b));return Math.max(0,...Object.keys(a).map(k=>maxDifference(a[k],b[k])));}
    assert.equal(a,b);return 0;
}
const summarize=xs=>{const s=xs.slice().sort((a,b)=>a-b);return {mean:xs.reduce((a,b)=>a+b,0)/xs.length,median:s[Math.floor(s.length/2)],p95:s[Math.min(s.length-1,Math.ceil(.95*s.length)-1)],min:s[0],max:s.at(-1)};};
const sides=await Promise.all(roots.map(async root=>({root,
    manager:await load(root,'src/physics/kirchhoffCompositeJointLumenFrictionRows.js'),
    normal:await load(root,'src/physics/kirchhoffCompositeJointLumenRows.js'),
    chain:await load(root,'src/physics/kirchhoffCompositeChain.js'),
    element:await load(root,'src/physics/kirchhoffCompositeElement.js'),
    step:await load(root,'src/physics/kirchhoffCompositeJointTimeStep.js'),
    helper:await load(root,'tests/lumenStepFixture.js')})));
const report={scope:'Frozen same equations and physical inputs; refresh microbenchmark plus real prepared full JointTimeStep; no anatomy/FPS claim',
    source:sides.map(s=>({root:s.root,manager:sha(s.root,'src/physics/kirchhoffCompositeJointLumenFrictionRows.js'),step:sha(s.root,'src/physics/kirchhoffCompositeJointTimeStep.js')})),step:[],refresh:[]};
assert.equal(report.source[0].step,report.source[1].step);
for(const scenario of [{name:'open',offset:.30,forces:[.4,.4,-.4,.4]},{name:'loaded-release',offset:.341,forces:[.4,.4,0,-.4,.4]}]){
    const contexts=sides.map(s=>{const f=s.helper.fixture({offset:scenario.offset});return {s,f,state:f.state,workspace:s.step.createCompositeJointTimeStepWorkspace(f.state)};});
    assert.equal(digest(contexts[0].state),digest(contexts[1].state));
    for(let i=0;i<scenario.forces.length;i++){
        const results=[];
        for(const x of contexts){
            const before=digest(x.state),args=x.s.helper.options(x.f,x.state,{force:scenario.forces[i],workspace:x.workspace});
            const t=performance.now(),r=x.s.step.advanceCompositeJointTimeStep(x.state,args),ms=performance.now()-t;
            assert.equal(digest(x.state),before,'incoming state must remain owned');assert.equal(r.accepted,true,JSON.stringify({status:r.status,error:r.error,diagnostics:r.diagnostics}));
            const cold=x.s.step.advanceCompositeJointTimeStep(x.state,x.s.helper.options(x.f,x.state,{force:scenario.forces[i]}));assert.equal(cold.accepted,true);
            for(const key of ['state','perTool','boundaryForces','contactForces','spinReactions','balances'])assert.deepEqual(r[key],cold[key],`cold/reuse ${key}`);
            if(i===1){const rejected=x.s.step.advanceCompositeJointTimeStep(x.state,{...args,budget:{evaluations:1}});assert.equal(rejected.accepted,false);assert.equal(rejected.state,x.state);assert.equal(digest(x.state),before);
                const retry=x.s.step.advanceCompositeJointTimeStep(x.state,args);assert.equal(retry.accepted,true);assert.deepEqual(retry.state,r.state);}
            results.push({r,ms,inputHash:before,preparedHash:digest({...args,workspace:null})});x.state=r.state;
        }
        const a=results[0],b=results[1],errors=Object.fromEntries(['state','perTool','boundaryForces','contactForces','spinReactions','balances'].map(key=>[key,maxDifference(a.r[key],b.r[key])]));
        assert.ok(Math.max(...Object.values(errors))<1e-11,JSON.stringify(errors));
        report.step.push({scenario:scenario.name,index:i,force:scenario.forces[i],inputHashes:results.map(r=>r.inputHash),preparedHashes:results.map(r=>r.preparedHash),maxAbsoluteErrors:errors,
            results:results.map(({r,ms})=>({ms,accepted:r.accepted,status:r.status,step:r.state.step,time:r.state.time,stateHash:digest(r.state),
                evaluations:r.diagnostics.evaluations,directions:r.diagnostics.directions,contactQueries:r.diagnostics.contactQueries,
                force:r.diagnostics.certificate.force,torque:r.diagnostics.certificate.torque,
                normalForces:Array.from(r.state.lumenContactState.normalForces),tractions:Array.from(r.state.lumenFrictionState.tractions),
                samples:r.diagnostics.certificate.friction.samples.map(s=>({Fn:s.Fn,slip:s.slip,slipRequired:s.slipRequired??true,zeroConeProof:s.zeroConeProof??null}))}))});
    }
}
function many(s,edgeCount=16){
    const dt=.02,coordinates=Array.from({length:edgeCount+1},(_,i)=>2*i),ids=['wire','catheter'],layout=s.chain.createCompositeChainLayout(Array.from({length:edgeCount},()=>ids));
    const toolPositions=new Map([['catheter',coordinates.map(x=>[x,0,0])],['wire',coordinates.map(x=>[x+.125,.25,0])]]);
    const modes=coordinates.map((_,node)=>({node,basis:[[1,0,0],[0,1,0],[0,0,1]],relativeDofs:[3*node,3*node+1,3*node+2]}));
    const state={layout,coordinates,modes,relativeToolId:'wire',toolPositions,relative:new Float64Array(3*coordinates.length),angles:new Map(ids.map(id=>[id,new Float64Array(edgeCount)])),
        tools:[...toolPositions].map(([id,p])=>({id,reference:s.element.captureCompositeReferenceFrames(p),dsDx:1}))};
    const candidate=structuredClone(state),tolerances={force:1e-7,lumenGap:1e-8,lumenNcp:1e-8,lumenWork:1e-8,frictionSlip:1e-8,frictionCone:1e-8,frictionWork:1e-8,frictionEquation:1e-9,linearConstraint:1e-10};
    const contacts={mode:'lumen-coulomb',chartId:'many-open-original-samples',forcePerLength:1,friction:{law:'coulomb',mu:[.3,.6],forcePerLength:5,materialPath:'linear-affine-maps'},
        pairs:Array.from({length:edgeCount},(_,edge)=>({id:`pair:${edge}`,innerToolId:'wire',outerToolId:'catheter',innerEdge:edge,outerEdge:edge,
            innerMaterialSegmentId:`wire:${edge}`,outerMaterialSegmentId:`catheter:${edge}`,lumenRadius:.5,innerRadius:.25,quadrature:[.25,.75],openDistal:false,portalFilletRadius:0}))};
    const normal=s.normal.createCompositeJointLumenRows({...state,contacts:{...contacts,mode:'lumen-normal',friction:'none'},tolerances});normal.prepareGauge({toolPositions,consumeQuery:()=>{throw Error('unexpected gauge query');}});
    const prepared={dt,previousPositions:structuredClone(toolPositions),inertiaEdges:layout.edgeToolIds.map((tools,e)=>({tools:tools.map(id=>({id,materialMap:{sStart:40+2*e,dsDx:1,dsDt:0}}))}))};
    const m=s.manager.createCompositeJointLumenFrictionRows({state,candidate,prepared,normal,contacts,dt,tolerances,normalRowOffset:0,frictionRowOffset:2*edgeCount});let queries=0;
    m.prepare({consumeQuery:()=>queries++});candidate.toolPositions.get('wire').forEach(p=>p[1]=.2);
    const common=new Float64Array(layout.dofCount),relative=new Float64Array(candidate.relative.length);
    return {m,normal,positions:candidate.toolPositions,common,relative,get queries(){return queries;},consumeQuery:()=>queries++,inputHash:digest({state,candidate,prepared,contacts})};
}
for(const order of ['full','gradient']){
    const contexts=sides.map(s=>many(s));assert.equal(contexts[0].inputHash,contexts[1].inputHash);
    const samples=contexts.map(()=>({normal:[],friction:[],total:[]}));
    for(let repeat=-6;repeat<18;repeat++)for(const side of (repeat%2===0?[0,1]:[1,0])){
        const c=contexts[side];c.common.fill(0);c.relative.fill(0);
        const t=performance.now();c.normal.refresh({toolPositions:c.positions,commonResidual:c.common,relativeResidual:c.relative,order,consumeQuery:c.consumeQuery});const mid=performance.now();
        const certificate=c.m.refresh({toolPositions:c.positions,commonResidual:c.common,relativeResidual:c.relative,order}),end=performance.now();
        assert.equal(certificate.converged,true);assert.ok(c.common.every(v=>v===0)&&c.relative.every(v=>v===0));
        if(side===1)assert.ok(certificate.samples.every(s=>s.slipRequired===false&&s.slip===null));
        if(repeat>=0){samples[side].normal.push(mid-t);samples[side].friction.push(end-mid);samples[side].total.push(end-t);}
    }
    report.refresh.push({order,nodes:17,normalSamples:32,frictionRows:64,pairedRepeats:18,warmup:6,inputHash:contexts[0].inputHash,
        results:contexts.map((c,i)=>({diagnostics:c.m.diagnostics,queries:c.queries,normalMs:summarize(samples[i].normal),frictionMs:summarize(samples[i].friction),totalMs:summarize(samples[i].total),raw:samples[i]}))});
}
const output=process.argv[4];if(output)writeFileSync(output,JSON.stringify(report,null,2)+'\n');
console.log(JSON.stringify({...report,refresh:report.refresh.map(x=>({...x,results:x.results.map(({raw,...r})=>r)}))},null,2));
