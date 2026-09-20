import {readFileSync,writeFileSync,mkdirSync} from 'node:fs';
import assert from 'node:assert/strict';
import {loadCoupledRuntimeAnatomy} from '/Users/macpiek/.codex/worktrees/e55c/OpenEndovascularTrainer/tests/helpers/coupledRuntimeFixture.js';
import {restoreSharedAxisReplay} from '/Users/macpiek/.codex/worktrees/e55c/OpenEndovascularTrainer/tests/helpers/sharedAxisReplay.js';
import {advanceSharedAxis} from '/Users/macpiek/.codex/worktrees/e55c/OpenEndovascularTrainer/src/physics/kirchhoffSharedAxisAppSystem.js';
import {createSharedAxisLinear,solveSharedAxisLinear} from '/Users/macpiek/.codex/worktrees/e55c/OpenEndovascularTrainer/src/physics/kirchhoffSharedAxisLinear.js';

const fixture=JSON.parse(readFileSync(process.argv[2],'utf8')),out=process.argv[3];mkdirSync(out,{recursive:true});
const anatomy=await loadCoupledRuntimeAnatomy(),systems=[];
try {
    const state=restoreSharedAxisReplay(fixture,anatomy.field),req=fixture.stepRequest;
    const iterator=advanceSharedAxis(state,req.rotations,req.dt,req.tools,{...req.options,wasmLinearAssembly:true,observeLinearSystem:({chain,options})=>{
        const rows=options.rows.map(r=>Object.fromEntries(['id','kind','dofs','jacobian','gap','multiplier','extraForceDofs','extraForceJacobian','geometricHessian'].filter(k=>r[k]!==undefined).map(k=>[k,structuredClone(r[k])])));
        systems.push({chain:{layout:structuredClone(chain.layout),hessian:chain.hessian?.slice(),tangent:chain.tangent?.slice()},options:{...Object.fromEntries(['wasmLinearAssembly','deferActiveBasis','lazyBasisCoefficients','zeroDualStart','batchRelease','reuseConstraintWork','reuseMatrixAssembly','reuseStructure','reuseWorkingSet'].map(k=>[k,options[k]])),rows,gradient:options.gradient.slice(),fixed:options.fixed.slice(),tolerance:options.tolerance,maxActiveSetAttempts:options.maxActiveSetAttempts}});
    }});
    let next;do{next=iterator.next();}while(!next.done);
    assert.ok(next.value.state,'Captured step must succeed before benchmarking its linear systems');
    const variants={compact:{},retained:{incrementalContacts:true},full:{incrementalContacts:'full'}};
    const samples=[],outputs={};
    for(let repeat=0;repeat<5;repeat++)for(const name of repeat%2?Object.keys(variants).reverse():Object.keys(variants)) {
        let totalMs=0,factorizations=0,updates=0,reuses=0,backsolves=0,updateFailures=0,rankResets=0,maxDifference=0,statusDifferences=0;
        const changedRows={},results=[],borderedStats=[];
        for(let index=0;index<systems.length;index++) {
            const {chain,options}=systems[index],w=createSharedAxisLinear(chain.layout,options.rows,{lazy:true});
            const start=performance.now(),result=solveSharedAxisLinear(w,chain,{...options,...variants[name]});totalMs+=performance.now()-start;
            factorizations+=result.factorizations;
            borderedStats.push(...(result.borderedStats??[]));
            for(const d of result.incrementalStats??[]){updates+=d.updates;reuses+=d.reuses;backsolves+=d.backsolves;updateFailures+=d.updateFailures;rankResets+=d.rankResets;for(const [k,v] of Object.entries(d.changedRows))changedRows[k]=(changedRows[k]??0)+v;}
            const output={converged:result.converged,failure:result.failure??null,residual:result.residual,attempts:result.activeSetAttempts,
                increment:Array.from(result.increment??[]),multipliers:Array.from(result.multiplierIncrement??[])};
            results.push(output);
            if(name!=='compact') {
                const ref=outputs.compact[index];if(ref.converged!==output.converged||ref.failure!==output.failure)statusDifferences++;
                if(ref.converged&&output.converged)for(const key of ['increment','multipliers'])for(let i=0;i<ref[key].length;i++)maxDifference=Math.max(maxDifference,Math.abs(ref[key][i]-output[key][i]));
            }
        }
        outputs[name]=results;samples.push({repeat,name,totalMs,factorizations,updates,reuses,backsolves,updateFailures,rankResets,changedRows,maxDifference,statusDifferences,borderedStats});
        console.log(JSON.stringify(samples.at(-1)));
    }
    writeFileSync(`${out}/comparison.json`,JSON.stringify({scope:'Repeated immutable linearizations from one accepted prepared step; not a browser or full-trajectory benchmark. Repeat 0 is warmup. Includes workspace assembly and solve, excludes material/contact geometry assembly.',sourceFixture:process.argv[2],stepResult:next.value.result,systems:systems.length,sizes:systems.map(s=>({primal:s.chain.layout.dofCount,rows:s.options.rows.length})),samples},null,2));
    writeFileSync(`${out}/directions.json`,JSON.stringify(outputs));
}finally{anatomy.dispose();}
