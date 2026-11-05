import test from 'node:test';
import assert from 'node:assert/strict';
import * as THREE from 'three';
import {ContrastFlowNetwork} from '../src/contrast/flowNetwork.js';
import {mixRefluxJunctionBoundaries} from '../src/contrast/refluxJunctionMixing.js';

function boundary(index, mass, volume, q, length=2) {
    return {edge:{index,cellLength:length,areas:[Math.PI*16]},cell:0,mass,volume,q,c:mass/volume};
}
function total(a) {return a.reduce((s,b)=>s+b.mass,0);}
function mixPair(dt, steps=1) {
    const a=[boundary(0,0,90,900),boundary(1,30,100,700)];
    for(let i=0;i<steps;i++)mixRefluxJunctionBoundaries(a,0,dt,.04);
    return a;
}

test('opposed inflows exchange locally, conserve iodine and remain bounded at stiff time steps',()=>{
    for(const dt of [1e-5,1/60,1,1000]) {
        const a=mixPair(dt);
        assert.ok(a[0].mass>0 && a[1].mass<30);
        assert.ok(Math.abs(total(a)-30)<1e-12);
        assert.ok(a.every(b=>b.c>=0&&b.c<=.3));
    }
    const once=mixPair(.5),subdivided=mixPair(.005,100);
    for(let i=0;i<2;i++)assert.ok(Math.abs(once[i].mass-subdivided[i].mass)<1e-11,
        'two boundary volumes have an exact exchange independent of timestep subdivision');
});

test('exchange vanishes continuously with reflux and ignores an ordinary selective outflow fork',()=>{
    const amount=q=>{
        const a=[boundary(0,0,100,1000),boundary(1,30,100,q)];
        mixRefluxJunctionBoundaries(a,0,.02,.04);return a[0].mass;
    };
    assert.equal(amount(0),0);
    assert.ok(amount(1e-9)<1e-10);
    assert.ok(amount(1e-6)>amount(1e-9));
    const single=[boundary(0,0,100,1000)];
    assert.equal(mixRefluxJunctionBoundaries(single,0,.1,.04),false);
    const reversedDaughters=[boundary(1,30,100,500),boundary(2,0,100,500)];
    assert.equal(mixRefluxJunctionBoundaries(reversedDaughters,0,.1,.04),false);
    assert.equal(reversedDaughters[1].mass,0);
});

test('several refluxing daughters mix simultaneously without branch-order bias',()=>{
    const make=()=>[boundary(0,0,100,1000),boundary(1,30,100,700),boundary(2,10,200,200)];
    const a=make(), b=make().reverse();
    mixRefluxJunctionBoundaries(a,0,.2,.04);mixRefluxJunctionBoundaries(b,0,.2,.04);
    assert.ok(Math.abs(total(a)-40)<1e-12);
    for(const value of a) {
        assert.ok(value.c>=0&&value.c<=.3);
        assert.ok(Math.abs(value.mass-b.find(other=>other.edge.index===value.edge.index).mass)<1e-12);
    }
});

function network(spacing=2) {
    const segment=(id,a,b,start,end)=>({id,nodeStartId:a,nodeEndId:b,
        start:new THREE.Vector3(...start),end:new THREE.Vector3(...end),radiusStart:4,radiusEnd:4});
    const net=new ContrastFlowNetwork([
        segment(0,0,1,[0,0,0],[0,30,0]),
        segment(1,1,2,[0,30,0],[-24,62,0]),
        segment(2,1,3,[0,30,0],[24,62,0])
    ],{rootPoint:new THREE.Vector3(),cellLengthMm:spacing,axialDispersionMm2PerS:10});
    net.setFlowOverride([0],1000);net.setFlowOverride([1],-700);net.setFlowOverride([2],1700);
    return net;
}

test('real reflux softens the parent front and hydrates newly reached edge buffers without losing mass',()=>{
    const net=network(),donor=net.edges[1];
    for(let c=0;c<donor.cellCount;c++)net.depositIodine(1,c,donor.volumes[c]*.3);
    const initial=net.totalIodineMassMg;
    for(let step=0;step<30;step++)net.update(1/60);
    const aorta=net.edges[0],c=Array.from(aorta.massMg,(m,i)=>m/aorta.volumes[i]);
    assert.ok(c.at(-1)>0 && c.at(-2)>0,'a short graded boundary layer must enter the parent');
    assert.ok(c.at(-1)>c.at(-2) && c.at(-2)>c.at(-3),'the boundary layer decays into clear upstream blood');
    assert.ok(c[0]<1e-8,'mixing must not repaint the entire aorta');
    assert.ok(net.edges[2].massMg.some(m=>m>0),'the diluted bolus still leaves through the accepting daughter');
    assert.ok(net.edges.every(e=>e.massMg.every(m=>m>=0)));
    assert.ok(Math.abs(net.totalIodineMassMg+net.outletIodineMassMg-initial)<1e-8);
});

test('graft fabric and excluded boundary cells block the confluence exchange',()=>{
    for(const kind of ['graft','excluded']) {
        const net=network(),donor=net.edges[1];
        if(kind==='graft')donor.graftSections=Uint8Array.from({length:donor.cellCount},()=>1);
        else donor.transportExcluded=true;
        donor.nextMassMg[0]=10;
        const marks=new Uint8Array([0,1,0]),touched=[1];
        net._applyJunctionDispersion(.1,touched,1,marks,1);
        assert.equal(net.edges[0].nextMassMg.reduce((a,b)=>a+b,0),0);
        assert.equal(net.edges[2].nextMassMg.reduce((a,b)=>a+b,0),0);
        assert.equal(donor.nextMassMg[0],10);
    }
});

test('transport plus confluence mixing converges with time and spatial refinement',()=>{
    const run=(dt,spacing)=>{
        const net=network(spacing),donor=net.edges[1];
        for(let c=0;c<donor.cellCount;c++)net.depositIodine(1,c,donor.volumes[c]*.3);
        for(let i=0;i<Math.round(.5/dt);i++)net.update(dt);
        return net.edges[0].massMg.reduce((a,b)=>a+b,0);
    };
    const time=[run(1/30,1),run(1/60,1),run(1/120,1)];
    // Resolve the short upstream layer (D/u is below 1 mm here) before
    // measuring spatial convergence; 2 mm cells cannot resolve that scale.
    const space=[run(1/2000,.5),run(1/2000,.25),run(1/2000,.125)];
    assert.ok(Math.abs(time[2]-time[1])<Math.abs(time[1]-time[0]),`temporal convergence: ${time}`);
    assert.ok(Math.abs(space[2]-space[1])<Math.abs(space[1]-space[0]),`spatial convergence: ${space}`);
});
