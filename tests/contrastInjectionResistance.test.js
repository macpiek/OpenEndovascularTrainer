import test from 'node:test';
import assert from 'node:assert/strict';
import * as THREE from 'three';
import {ContrastFlowNetwork} from '../src/contrast/flowNetwork.js';
import {ContinuousInjectionFlowCoupler} from '../src/contrast/continuousInjectionFlowCoupler.js';

function fixture() {
    const segment = (id, startId, endId, a, b) => ({id, nodeStartId:startId, nodeEndId:endId,
        start:new THREE.Vector3(...a), end:new THREE.Vector3(...b), radiusStart:5, radiusEnd:5});
    const net = new ContrastFlowNetwork([
        segment(0,0,1,[0,0,0],[0,60,0]),
        segment(1,1,2,[0,60,0],[-40,120,0]),
        segment(2,1,3,[0,60,0],[40,120,0])
    ], {rootPoint:new THREE.Vector3(), cardiacOutputMlPerMin:600, axialDispersionMm2PerS:0});
    for (const id of [2,3]) net.setOutletResistance(id, 1);
    const edge = net.edges[1];
    const port = {position:edge.start.clone().lerp(edge.end,.5),direction:edge.axis.clone().negate(),valid:true};
    return {net,port,coupler:new ContinuousInjectionFlowCoupler(net)};
}
function apply(f, rate, ports=[f.port]) {
    f.net.clearFlowOverrides(); f.coupler.beginStep();
    return f.coupler.configure({ports,rateMlPerSec:rate});
}
function close(a,b,tolerance=1e-8) { assert.ok(Math.abs(a-b)<tolerance, `${a} != ${b}`); }

test('a volume source follows outlet resistances and preserves cardiac inflow and every junction balance',()=>{
    const f=fixture(), [split]=apply(f,20), {net}=f;
    const loc=split.location, source=net.edges[loc.edgeIndex];
    const proximal=net.getFaceSignedFlowMm3PerS(source.index,loc.cellIndex,1);
    const distal=net.getFaceSignedFlowMm3PerS(source.index,loc.cellIndex+1,1);
    close(distal-proximal,20000);
    close(net.getFaceSignedFlowMm3PerS(0,0,1),10000);
    close(net.getFaceSignedFlowMm3PerS(0,net.edges[0].cellCount,1),
        net.getFaceSignedFlowMm3PerS(1,0,1)+net.getFaceSignedFlowMm3PerS(2,0,1));
    const outletFlow=net.edges.slice(1).reduce((sum,e)=>sum+net.getFaceSignedFlowMm3PerS(e.index,e.cellCount,1),0);
    close(outletFlow,30000);
    assert.ok(proximal<0,'strong injection still generates genuine reflux');
    assert.ok(split.upstreamFraction>.49 && split.upstreamFraction<.51);
    // Outward pressure perturbation is equal at both tissue beds; the source
    // adds the appropriate resolved-vessel pressure drops to each path.
    const sourceT=(loc.cellIndex+.5)/source.cellCount;
    const du=-proximal+source.meanFlowMm3PerS;
    const dd=distal-source.meanFlowMm3PerS;
    close(du*(net.edges[2].equivalentResistance+source.resistance*sourceT),
        dd*(net.nodes.get(source.endNodeId).equivalentResistance+source.resistance*(1-sourceT)),1e-6);
});

test('bulk flow is independent of nozzle direction and responds to changed bed resistance',()=>{
    const f=fixture(), first=apply(f,10)[0];
    f.port.direction.negate();
    close(apply(f,10)[0].upstreamFraction,first.upstreamFraction);
    f.net.setOutletResistance(3,4);
    const after=apply(f,10)[0];
    assert.ok(after.upstreamFraction < first.upstreamFraction*.5,
        'increasing resistance of the opposite bed reduces reflux toward it');
});

test('multiple device ports superpose without adding or losing blood volume',()=>{
    const f=fixture();
    const second={...f.port,position:f.net.edges[2].start.clone().lerp(f.net.edges[2].end,.5)};
    apply(f,12,[f.port,second]);
    const out=f.net.edges.slice(1).reduce((sum,e)=>sum+f.net.getFaceSignedFlowMm3PerS(e.index,e.cellCount,1),0);
    close(out,22000);close(f.net.getFaceSignedFlowMm3PerS(0,0,1),10000);
});

test('selective low-rate injection stays distal while reflux mixes with ongoing clear aortic inflow',()=>{
    const run=(rate)=>{
        const f=fixture();let injected=0;
        for(let step=0;step<360;step++) {
            const [split]=apply(f,rate),loc=split.location;
            // Set the known steady physiological inflows to isolate mixing
            // from cardiac phase while retaining the source's signed deltas.
            for(const e of f.net.edges) f.net.setFlowOverride([e.index],e.meanFlowMm3PerS);
            const mass=rate*300/60;injected+=mass;
            f.net.depositIodine(loc.edgeIndex,loc.cellIndex,mass);
            f.net.update(1/60);
        }
        const e=f.net.edges[2], c=e.massMg[0]/e.volumes[0]/.3;
        close(f.net.totalIodineMassMg+f.net.outletIodineMassMg,injected,1e-7);
        return c;
    };
    close(run(.25),0);
    const diluted=run(20);
    // 5 ml/s reflux at stock + 10 ml/s clear aortic blood -> one-third stock.
    assert.ok(diluted>.3 && diluted<.36, `crossover must dilute, got ${diluted} stock`);
});

test('counterflow keeps shear mixing at zero net transport without changing ordinary vessels',()=>{
    const mix = (counterflow) => {
        const net = new ContrastFlowNetwork([{
            id:0,nodeStartId:0,nodeEndId:1,start:new THREE.Vector3(0,0,0),end:new THREE.Vector3(100,0,0),
            radiusStart:2,radiusEnd:2
        }], {cellLengthMm:2,axialDispersionMm2PerS:10});
        const e=net.edges[0];
        net.setFlowOverride([0],counterflow?1000:0);
        if(counterflow)net.addFaceFlowDelta(0,0,e.cellCount+1,-1000);
        close(net.getFaceSignedFlowMm3PerS(0,20,1),0);
        if(counterflow)close(net.getFaceDispersionFlowMm3PerS(0,20,1),2000);
        net.depositIodine(0,25,1);
        for(let i=0;i<30;i++)net.update(1/60);
        close(net.totalIodineMassMg+net.outletIodineMassMg,1,1e-10);
        assert.ok(e.massMg.every(m=>m>=0));
        return e.massMg.reduce((s,m,i)=>s+m*((i-25)*2)**2,0);
    };
    assert.ok(mix(true)>mix(false)*1.8,'opposed streams must broaden the front even at zero net flow');
    const f=fixture();
    close(f.net.getFaceDispersionFlowMm3PerS(1,0,1),f.net.getFaceSignedFlowMm3PerS(1,0,1));
});
