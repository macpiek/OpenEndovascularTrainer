import test from 'node:test';
import assert from 'node:assert/strict';
import {prepareSharedAxisActiveBasis as optimized} from '../src/physics/kirchhoffSharedAxisActiveBasis.js';
import {prepareSharedAxisActiveBasis as reference} from './helpers/sharedAxisActiveBasisReference.js';

function run(fn,rows,fixed,active,dual) {
    const activeSet=Uint8Array.from(active),workingDual=Float64Array.from(dual),trace=[];
    const result=fn({rows,fixed,activeSet,dual:workingDual,trace});return {result,activeSet,dual:workingDual,trace};
}
function compare(rows,fixed,active,dual) {
    const saved=structuredClone(rows),a=run(reference,rows,fixed,active,dual),b=run(optimized,rows,fixed,active,dual);
    assert.deepEqual(b,a);assert.deepEqual(rows,saved);return b;
}
const random=seed=>()=>{seed=(Math.imul(seed,1664525)+1013904223)>>>0;return seed/2**32;};
function rowSet(rng,edges) {
    const rows=[];
    for(let e=0;e<edges;e++) {
        const dofs=[e*3,e*3+1,e*3+2,e*3+3,e*3+4,e*3+5];
        rows.push({id:`length${e}`,kind:'length',dofs,jacobian:[-1,0,0,1,0,0],gap:0});
        const t=.1+.8*rng(),normal=[rng(),rng(),rng()];
        const row={id:`wall${e}`,kind:'wall',dofs,jacobian:[...normal.map(v=>(1-t)*v),...normal.map(v=>t*v)],gap:.01*rng()};
        rows.push(row);
        if(e%3===0)rows.push({...row,id:`duplicate${e}`,jacobian:row.jacobian.map(v=>v*2),gap:row.gap*2});
        if(e%5===0)rows.push({...row,id:`tighter${e}`,jacobian:row.jacobian.slice(),gap:row.gap-.001});
    }
    return rows;
}

test('reused sparse workspace exactly matches independent reference over changing masks, active sets and coefficients',()=>{
    const rng=random(761234),fixed=new Uint8Array(63);
    for(let trial=0;trial<80;trial++) {
        const rows=rowSet(rng,trial%2?20:16),active=rows.map(r=>r.kind==='length'||rng()>.15?1:0),dual=rows.map(r=>r.kind==='length'?rng()-.5:2*rng());
        fixed.fill(0);for(let i=0;i<fixed.length;i++)if(rng()<.12)fixed[i]=1;
        compare(rows,fixed,active,dual);
        // Same mask object and row count, entirely different values/supports.
        for(const row of rows){row.jacobian=row.jacobian.map(v=>-v);row.dofs=row.dofs.map(i=>fixed.length-1-i);}
        compare(rows,fixed,active,dual);
    }
});

test('independent calls sharing a fixed-mask workspace return owned duals and preserve failure semantics',()=>{
    const fixed=new Uint8Array(6),rows=[{kind:'wall',id:'a',dofs:[0],jacobian:[1],gap:-1},{kind:'wall',id:'b',dofs:[0],jacobian:[-1],gap:0}];
    const rejected=compare(rows,fixed,[1,1],[1,1]);assert.equal(rejected.result.failure,'incompatible-active-constraints');
    const saved=structuredClone(rejected);
    rows[1].jacobian=[1];rows[1].gap=-2;
    assert.equal(compare(rows,fixed,[1,1],[1,1]).result.converged,true);
    assert.deepEqual(rejected,saved);
    fixed.fill(1);assert.equal(compare(rows,fixed,[1,1],[1,1]).result.pivots,0);
    fixed.fill(0);compare(rows,fixed,[1,1],[1,1]);
});

test('rank norm keeps Math.hypot scaling for extreme local gradients and cancels duplicate DOFs exactly',()=>{
    const fixed=new Uint8Array(8);
    for(const scale of [1e-150,1e-20,1,1e100,1e150]) {
        const rows=[{kind:'wall',id:'a',dofs:[0,1,1,2],jacobian:[scale,2*scale,-2*scale,3*scale],gap:0},
            {kind:'wall',id:'b',dofs:[0,2],jacobian:[2*scale,6*scale],gap:0}];
        compare(rows,fixed,[1,1],[1,2]);
    }
});

test('sparse basis reuse retains ascending elimination when fill-in reaches later pivots',()=>{
    const fixed=new Uint8Array(16);
    const rows=[
        {id:'first',kind:'length',dofs:[0,4],jacobian:[3,1],gap:0},
        {id:'second',kind:'length',dofs:[4,7],jacobian:[4,1],gap:0},
        {id:'third',kind:'length',dofs:[7,10],jacobian:[2,1],gap:0},
        {id:'fourth',kind:'wall',dofs:[10,13],jacobian:[3,1],gap:0},
        {id:'all',kind:'wall',dofs:[0,4,7,10,13],jacobian:[3,5,3,4,1],gap:0},
        {id:'sparse',kind:'wall',dofs:[0,13],jacobian:[1,.2],gap:0}
    ];
    for(let i=0;i<5;i++)compare(rows,fixed,[1,1,1,1,1,1],[-1,2,-3,2,5,1]);
});
