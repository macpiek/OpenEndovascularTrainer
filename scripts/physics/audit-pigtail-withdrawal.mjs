import {readFileSync,writeFileSync,mkdirSync} from 'node:fs';
import {loadCoupledRuntimeAnatomy} from '../../tests/helpers/coupledRuntimeFixture.js';
import {restoreSharedAxisReplay,captureSharedAxisReplay} from '../../tests/helpers/sharedAxisReplay.js';
import {advanceSharedAxis} from '../../src/physics/kirchhoffSharedAxisAppSystem.js';
const fixture=JSON.parse(readFileSync(process.argv[2],'utf8')),output=process.argv[3];mkdirSync(output,{recursive:true});
const anatomy=await loadCoupledRuntimeAnatomy(),req=fixture.stepRequest;
const json=value=>JSON.stringify(value,(_,v)=>ArrayBuffer.isView(v)?Array.from(v):v);
try {
    const s=restoreSharedAxisReplay(fixture,anatomy.field),failures=[],trace=[],iterations=[];
    const iterator=advanceSharedAxis(s,req.rotations,req.dt,req.tools,{...req.options,
        observeIteration:({state,iteration,base})=>{
            state.linearTrace={captureConflicts:true,push:e=>{if(e.kind==='incompatible-active-constraints')trace.push(e);}};
            iterations.push({iteration,live:state.wallFrictionStep?.liveNormalLoad,force:base.force,torque:base.torque,constraint:base.constraint});
        },observeTrial:e=>{
            if(e.kind==='direction'&&trace.length){
                const index=failures.length;
                failures.push({method:e.method,failure:e.direction.failure,live:e.state.wallFrictionStep?.liveNormalLoad,trace:trace.splice(0)});
                if(index<3)writeFileSync(`${output}/conflict-${index}.json`,json({state:captureSharedAxisReplay(e.state,fixture.sheath),rows:e.base.rows,
                    chain:{layout:e.state.chain.layout,hessian:e.state.chain.hessian,tangent:e.state.chain.tangent,gradient:e.state.chain.gradient},fixed:e.state.fixed}));
            }
        }});
    let next;do{next=iterator.next();}while(!next.done);
    writeFileSync(`${output}/audit.json`,json({result:next.value.result,failures,iterations}));
    console.log(json({result:next.value.result,conflicts:failures.length}));
}finally{anatomy.dispose();}
