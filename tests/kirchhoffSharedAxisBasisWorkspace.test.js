import test from 'node:test';
import assert from 'node:assert/strict';
import {prepareSharedAxisActiveBasis as optimized} from '../src/physics/kirchhoffSharedAxisActiveBasis.js';
import {prepareSharedAxisActiveBasis as reference} from './helpers/sharedAxisActiveBasisReference.js';

function run(fn,rows,fixed,active,dual,options={}) {
    const activeSet=Uint8Array.from(active),workingDual=Float64Array.from(dual),trace=[];
    const result=fn({rows,fixed,activeSet,dual:workingDual,trace,...options});return {result,activeSet,dual:workingDual,trace};
}
function compare(rows,fixed,active,dual) {
    const saved=structuredClone(rows),a=run(reference,rows,fixed,active,dual),b=run(optimized,rows,fixed,active,dual);
    assert.deepEqual(b,a);
    for(const reuseStructure of [true,false])assert.deepEqual(run(optimized,rows,fixed,active,dual,{lazyBasisCoefficients:true,reuseStructure}),a);
    assert.deepEqual(rows,saved);return b;
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

test('cached row order follows in-place kind and active-mask changes without retaining numerical pivots',()=>{
    const fixed=new Uint8Array(15),rng=random(3719),rows=rowSet(rng,4),active=new Uint8Array(rows.length).fill(1),dual=new Float64Array(rows.length).fill(2);
    for(let trial=0;trial<40;trial++) {
        for(let i=0;i<rows.length;i++) {
            rows[i].kind=(trial+i)%3?'wall':'length';active[i]=(trial+i)%5?1:0;
            rows[i].jacobian=rows[i].jacobian.map(v=>-v);
        }
        fixed[trial%fixed.length]=1-fixed[trial%fixed.length];
        compare(rows,fixed,active,dual);
    }
});

test('unchanged linearizations reuse a numerical prefix but new tokens and active-order changes rebuild it',()=>{
    const fixed=new Uint8Array(18),rows=Array.from({length:12},(_,i)=>({kind:'wall',id:i,dofs:[i],jacobian:[1],gap:0}));
    let reads=0;
    for(const r of rows){const values=r.jacobian;Object.defineProperty(r,'jacobian',{get(){reads++;return values;},enumerable:true});}
    let basisCache=Symbol(),active=new Uint8Array(rows.length).fill(1),dual=new Float64Array(rows.length).fill(1);
    const cached=input=>optimized({...input,basisCache});
    run(cached,rows,fixed,active,dual);reads=0;
    run(cached,rows,fixed,active,dual);assert.equal(reads,0,'unchanged independent prefix should not repeat elimination');
    for(const index of [10,11,0,7,4,0,10]) {
        active[index]=1-active[index];dual[index]+=.3;
        assert.deepEqual(run(cached,rows,fixed,active,dual),run(reference,rows,fixed,active,dual));
    }
    rows[5].jacobian[0]=2;fixed[7]=1;basisCache=Symbol();reads=0;
    const actual=run(cached,rows,fixed,active,dual);assert.ok(reads>0);
    assert.deepEqual(actual,run(reference,rows,fixed,active,dual));
    // Another solve using the same fixed-mask storage must invalidate the
    // first token's prefix, even if the first generator later resumes.
    run(input=>optimized({...input,basisCache:Symbol()}),rows,fixed,active,dual);
    assert.deepEqual(run(cached,rows,fixed,active,dual),run(reference,rows,fixed,active,dual));
});


test('lazy reaction coefficients preserve cached-prefix pivots when active rows change and when switching modes',()=>{
    const rng=random(547812),fixed=new Uint8Array(63),rows=rowSet(rng,20),token=Symbol('fixed-jacobians');
    for(let trial=0;trial<60;trial++) {
        const active=rows.map(r=>r.kind==='length'||rng()>.2?1:0),dual=rows.map(r=>r.kind==='length'?rng()-.5:2*rng());
        const expected=run(reference,rows,fixed,active,dual);
        const actual=run(optimized,rows,fixed,active,dual,{lazyBasisCoefficients:trial%4!==0,basisCache:token});
        assert.deepEqual(actual,expected);
    }
});
