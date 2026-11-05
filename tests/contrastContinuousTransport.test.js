import test from 'node:test';
import assert from 'node:assert/strict';
import * as THREE from 'three';
import { ContrastFlowNetwork } from '../src/contrast/flowNetwork.js';
import { FlowConcentrationField } from '../src/contrast/flowConcentrationField.js';
import { ContrastVolumeRenderer } from '../src/contrast/contrastVolumeRenderer.js';
import { LocalContrastInjectionSolver } from '../src/contrast/localInjectionSolver.js';

const area = Math.PI * 4;
const segment = (id, a, b, start, end) => ({
    id,
    nodeStartId: a,
    nodeEndId: b,
    start: new THREE.Vector3(...start),
    end: new THREE.Vector3(...end),
    radiusStart: 2,
    radiusEnd: 2
});
function chain(points, options = {}) {
    return new ContrastFlowNetwork(
        points.slice(1).map((x, i) => segment(i, i, i + 1, [points[i], 0, 0], [x, 0, 0])),
        { rootPoint: new THREE.Vector3(points[0], 0, 0), axialDispersionMm2PerS: 0, ...options }
    );
}
function sum(net) {
    return net.getIodineMassMg() + net.outletIodineMassMg;
}
function centroid(net) {
    let mass = 0,
        moment = 0;
    for (const edge of net.edges)
        for (let i = 0; i < edge.cellCount; i++) {
            const m = edge.massMg[i];
            mass += m;
            moment += m * (edge.start.x + (i + 0.5) * edge.cellLength);
        }
    return moment / mass;
}
for (const sign of [1, -1])
    test(`short fragments preserve travel distance at high Courant number (${sign})`, () => {
        const net = chain(Array.from({ length: 101 }, (_, i) => i * 0.01));
        net.setFlowOverride(
            net.edges.map((e) => e.index),
            sign * area * 20
        );
        const start = sign > 0 ? 0 : 90,
            mass = 0.3 * net.edges[start].volumes[0];
        net.depositIodine(start, 0, mass);
        net.update(0.02);
        assert.ok(Math.abs(centroid(net) - ((start + 0.5) * 0.01 + sign * 0.4)) < 1e-6);
        assert.ok(Math.abs(sum(net) - mass) < 1e-10);
        assert.equal(
            net.lastTransportSubstepCount,
            1,
            'tiny cells must not require hundreds of whole-tree steps'
        );
        for (const e of net.edges) assert.ok(e.massMg.every((m) => m >= 0));
    });

test('time profiles keep finite arrival, split by flow, reverse and wash out without junction storage', () => {
    const net = new ContrastFlowNetwork(
        [
            segment(0, 0, 1, [0, 0, 0], [1, 0, 0]),
            segment(1, 1, 2, [1, 0, 0], [2, 1, 0]),
            segment(2, 1, 3, [1, 0, 0], [2, -1, 0])
        ],
        { rootPoint: new THREE.Vector3(0, 0, 0), cellLengthMm: 0.1, axialDispersionMm2PerS: 0 }
    );
    net.setFlowOverride([0], area * 100);
    net.setFlowOverride([1, 2], area * 50);
    const mass = 0.3 * net.edges[0].volumes[0];
    net.depositIodine(0, 0, mass);
    net.update(0.002);
    assert.equal(
        net.edges[1].massMg.reduce((a, b) => a + b, 0),
        0,
        'no instantaneous daughter filling'
    );
    net.update(0.01);
    const daughters = net.edges.slice(1).map((e) => e.massMg.reduce((a, b) => a + b, 0));
    assert.ok(daughters.every((m) => m > 0));
    assert.ok(Math.abs(daughters[0] - daughters[1]) < 1e-12);
    assert.ok(Math.abs(sum(net) - mass) < 1e-10);
    net.setFlowOverride([0], -area * 100);
    net.setFlowOverride([1, 2], -area * 50);
    for (let i = 0; i < 10; i++) net.update(0.01);
    assert.ok(net.getIodineMassMg() < 1e-9);
    assert.ok(Math.abs(sum(net) - mass) < 1e-10);
});

test('closed internal junction retains its donors without losing iodine', () => {
    const net = chain([0, 1, 2]);
    net.setFlowOverride([0], area * 100);
    net.setFlowOverride([1], 0);
    net.depositIodine(0, 0, 1);
    net.update(0.02);
    assert.equal(net.edges[0].massMg[0], 1);
    assert.equal(net.outletIodineMassMg, 0);
});

test('long updates resolve the same arterial pulse as consecutive accuracy steps', () => {
    const a = chain(Array.from({ length: 21 }, (_, i) => i * 2));
    const b = chain(Array.from({ length: 21 }, (_, i) => i * 2));
    a.depositIodine(0, 0, 10);
    b.depositIodine(0, 0, 10);
    a.update(0.1);
    for (let i = 0; i < 3; i++) b.update(1 / 30);
    for (let e = 0; e < a.edges.length; e++)
        for (let i = 0; i < a.edges[e].cellCount; i++)
            assert.ok(Math.abs(a.edges[e].massMg[i] - b.edges[e].massMg[i]) < 1e-10);
    assert.ok(Math.abs(a.outletIodineMassMg - b.outletIodineMassMg) < 1e-10);
});

test('stiff diffusion remains positive and conservative without a tiny-cell timestep', () => {
    const net = chain([0, 0.1], { cellLengthMm: 0.05, axialDispersionMm2PerS: 100 });
    net.setFlowOverride([0], 0);
    net.depositIodine(0, 0, 1);
    net.update(1 / 30);
    assert.ok(Math.abs(net.edges[0].massMg[0] - 0.5) < 1e-9);
    assert.ok(Math.abs(net.edges[0].massMg[1] - 0.5) < 1e-9);
    assert.ok(Math.abs(sum(net) - 1) < 1e-12);
    assert.equal(net.lastTransportSubstepCount, 1);
});

test('connected samples are continuous across unequal one-cell segment boundaries', () => {
    const net = chain([0, 1, 5, 6], { cellLengthMm: 10 });
    const field = new FlowConcentrationField(net, new Uint32Array([0, 1, 2]), 3);
    field.update(new Float32Array([1.5, 4, 6.5])); // c(x)=1+x at cell centres
    for (const [a, b, x] of [
        [0, 1, 1],
        [1, 2, 5]
    ]) {
        assert.ok(Math.abs(field.sample(a, 1) - (1 + x)) < 1e-6);
        assert.ok(Math.abs(field.sample(a, 1) - field.sample(b, 0)) < 1e-7);
    }
    field.update(new Float32Array([1, 0, 0]));
    assert.equal(
        field.sample(0, 1),
        field.sample(1, 0),
        'a clear downstream cell must not cut off the bolus front'
    );
    assert.ok(field.sample(1, 0) > 0);
    assert.equal(field.sample(1, 0.5), 0, 'the interpolation must stop at the clear cell centre');
});

function display(net) {
    const r = new ContrastVolumeRenderer({
        flowNetwork: net,
        localSolver: new LocalContrastInjectionSolver({ flowNetwork: net, capacity: 16 }),
        medium: { iodineMgPerMl: 300 }
    });
    return r;
}

test('junction reconstruction fades in continuously from trace iodine in either flow direction', () => {
    const net = new ContrastFlowNetwork([
        segment(0, 0, 1, [0, 0, 0], [20, 0, 0]),
        segment(1, 1, 2, [20, 0, 0], [40, 15, 0]),
        segment(2, 1, 3, [20, 0, 0], [40, -15, 0])
    ], {rootPoint: new THREE.Vector3(0, 0, 0), cellLengthMm: 2});
    let count = 0;
    const offsets = Uint32Array.from(net.edges, e => {const offset = count; count += e.cellCount; return offset;});
    const field = new FlowConcentrationField(net, offsets, count);
    for (const [edgeIndex, atEnd] of [[0, true], [1, false]]) {
        const edge = net.edges[edgeIndex], cell = offsets[edgeIndex] + (atEnd ? edge.cellCount - 1 : 0);
        const positions = atEnd ? [1, 1 - .25 / edge.cellCount, 1 - .5 / edge.cellCount]
            : [0, .25 / edge.cellCount, .5 / edge.cellCount];
        const samples = field.compileSamples(positions.map(() => edgeIndex), positions);
        const output = new Float32Array(positions.length);
        let previous = -1;
        for (const concentration of [0, 1e-9, 1e-6, .01, .1, .5, 1]) {
            field.values.fill(1); // Strongly opacified junction, newly reached arm.
            field.values[cell] = concentration;
            field.sampleInto(samples, output);
            const signal = field.sample(edgeIndex, positions[0]);
            assert.ok(signal >= previous, 'arrival must remain monotonic');
            assert.ok(signal <= 2 * concentration + 1e-8,
                'a trace of iodine cannot switch on the full junction signal');
            if (concentration === 0) assert.equal(signal, 0, 'an unreached arm stays clear');
            assert.ok(Math.abs(output[2] - concentration) < 1e-7, 'the measured cell centre is unchanged');
            for (let i = 0; i < positions.length; i++)
                assert.ok(Math.abs(output[i] - field.sample(edgeIndex, positions[i])) < 1e-7);
            previous = signal;
        }
    }
});

test('pending source iodine stays in its supplying lumen and is continuous across handoff', () => {
    const net = chain([0, 20, 40]);
    const r = display(net), solver = r.system.localSolver;
    try {
        // A geometric neighbour is closer than the source control volume.
        // Pending source parcels still belong to the cell chosen by transport.
        solver.count = 1;
        solver.positionX[0] = 30;
        solver.sourceEdgeIndex[0] = 0;
        solver.sourceCellIndex[0] = 2;
        solver.iodineMassMg[0] = .03;
        r.update();
        const before = r._flowCellRawConcentration.slice();
        assert.ok(before[r._flowCellOffset[0] + 2] > 0);
        for (let cell = 0; cell < net.edges[1].cellCount; cell++)
            assert.equal(before[r._flowCellOffset[1] + cell], 0,
                'a nearby clear lumen cannot borrow pending source iodine');
        assert.ok(Math.abs(r._flowCellLocalPlumeMassMg.reduce((a, b) => a + b, 0) - .03) < 1e-8);
        net.depositIodine(0, 2, .03);
        solver.count = 0;
        r.update();
        for (let i = 0; i < before.length; i++)
            assert.ok(Math.abs(before[i] - r._flowCellRawConcentration[i]) < 1e-8,
                'handoff must not move the visible bolus or change its concentration');
    } finally { r.dispose(); }
});
test('junction connectors follow the local arm instead of retaining the strongest daughter', () => {
    const net = new ContrastFlowNetwork(
        [
            segment(0, 0, 1, [0, 0, 0], [20, 0, 0]),
            segment(1, 1, 2, [20, 0, 0], [40, 15, 0]),
            segment(2, 1, 3, [20, 0, 0], [40, -15, 0])
        ],
        { rootPoint: new THREE.Vector3(0, 0, 0), cellLengthMm: 2 }
    );
    for (let i = 0; i < 3; i++) net.depositIodine(1, i, net.edges[1].volumes[i] * 0.1);
    const r = display(net);
    try {
        const vertices = [...r._flowTrueJunctionConnectorVertexIndices];
        assert.ok(vertices.length > 0);
        assert.ok(
            vertices.some((i) => r._flowVertexConcentration[i] > 0),
            'perfused arm stays visible'
        );
        for (const i of vertices)
            if (r._flowVertexConcentrationEdgeIndex[i] !== 1)
                assert.equal(
                    r._flowVertexConcentration[i],
                    0,
                    'clear arms must not acquire the maximum neighbour signal'
                );
    } finally {
        r.dispose();
    }
});

test('display does not fill long clear intervals between separate boluses', () => {
    const net = chain(Array.from({ length: 21 }, (_, i) => i * 2));
    for (const i of [1, 2, 17, 18]) net.depositIodine(i, 0, net.edges[i].volumes[0] * 0.1);
    const r = display(net);
    try {
        for (let i = 6; i < 14; i++)
            assert.equal(r._flowCellDisplayConcentration[r._flowCellOffset[i]], 0);
    } finally {
        r.dispose();
    }
});

test('a flowing bolus develops a diluted front and tail across one-cell vessel fragments',()=>{
    const net=chain(Array.from({length:101},(_,i)=>i*2),{axialDispersionMm2PerS:10});
    net.setFlowOverride(net.edges.map(e=>e.index),area*40);
    const initial=.3*net.edges[30].volumes[0];
    net.depositIodine(30,0,initial);
    for(let i=0;i<30;i++)net.update(1/60);
    const concentrations=net.edges.map(e=>e.massMg[0]/e.volumes[0]);
    const peak=Math.max(...concentrations),peakIndex=concentrations.indexOf(peak);
    assert.ok(peak<.1,'the peak must dilute rather than retain the stock concentration');
    assert.ok(concentrations[peakIndex-3]>peak*.1,'a visible trailing gradient remains');
    assert.ok(concentrations[peakIndex+3]>peak*.1,'a visible leading gradient forms');
    assert.ok(Math.abs(centroid(net)-81)<1,'mixing must not change bolus travel speed');
    assert.ok(Math.abs(sum(net)-initial)<1e-9);
    assert.ok(concentrations.every(c=>c>=0));
});

test('axial mixing in a selected daughter does not stain its parent or sibling',()=>{
    const net=new ContrastFlowNetwork([
        segment(0,0,1,[0,0,0],[20,0,0]),
        segment(1,1,2,[20,0,0],[120,20,0]),
        segment(2,1,3,[20,0,0],[120,-20,0])
    ],{rootPoint:new THREE.Vector3(),cellLengthMm:2,axialDispersionMm2PerS:40});
    net.setFlowOverride([0],area*10);net.setFlowOverride([1,2],area*5);
    net.depositIodine(1,0,1);
    for(let i=0;i<60;i++)net.update(1/60);
    assert.ok(net.edges[0].massMg.every(m=>m===0));
    assert.ok(net.edges[2].massMg.every(m=>m===0));
    assert.ok(Math.abs(sum(net)-1)<1e-10);
});

test('normal-vessel mixing does not cross an excluded cell or graft barrier',()=>{
    for(const excluded of [false,true]) {
        const net=chain(Array.from({length:8},(_,i)=>i),{axialDispersionMm2PerS:40});
        net.setFlowOverride(net.edges.map(e=>e.index),0);
        if(excluded)net.edges[3].transportExcluded=true;
        else net.edges[3].graftSections=new Uint8Array([1]);
        net.depositIodine(2,0,1);net.update(.1);
        assert.ok(net.edges.slice(3).every(e=>e.massMg.every(m=>m===0)));
        assert.ok(Math.abs(sum(net)-1)<1e-10);
    }
});

test('junction mixing follows reflux and dilutes it with the incoming blood flow', () => {
    function mix(bloodFlow, blocked = false) {
        const net = new ContrastFlowNetwork([
            segment(0, 0, 1, [0, 0, 0], [20, 0, 0]),
            segment(1, 1, 2, [20, 0, 0], [52, 24, 0]),
            segment(2, 1, 3, [20, 0, 0], [52, -24, 0])
        ], {rootPoint: new THREE.Vector3(), cellLengthMm: 2, axialDispersionMm2PerS: 10,
            axialFlowDispersionFactor: 0});
        net.setFlowOverride([0], bloodFlow);
        net.setFlowOverride([1], -100);
        net.setFlowOverride([2], bloodFlow + 100);
        const donor = net.edges[1];
        donor.nextMassMg[0] = donor.volumes[0] * .3;
        let cells = 0;
        const offsets = Uint32Array.from(net.edges, e => {const offset = cells; cells += e.cellCount; return offset;});
        const field = new FlowConcentrationField(net, offsets, cells), source = new Float32Array(cells);
        source[offsets[1]] = .3; field.update(source);
        assert.ok(Math.abs(field.values[field.start[1]] - .3 * 100 / (bloodFlow + 100)) < 1e-7,
            'the displayed junction must use the same blood dilution as transport');
        if (blocked) donor.graftSections = [1];
        const before = donor.nextMassMg[0], marks = new Uint8Array([0, 1, 0]), touched = [1];
        net._applyJunctionDispersion(.01, touched, 1, marks, 1);
        const total = net.edges.reduce((s, e) => s + e.nextMassMg.reduce((a, b) => a + b, 0), 0);
        assert.ok(Math.abs(total - before) < 1e-12);
        assert.ok(net.edges.every(e => e.nextMassMg.every(m => m >= 0)));
        assert.ok(net.edges[0].nextMassMg.every(m => m === 0), 'clear inflowing aorta is not another outlet');
        return net.edges[2].nextMassMg[0];
    }
    const reflux = mix(0), diluted = mix(300);
    assert.ok(reflux > 0, 'a reversed daughter must exchange iodine at its inlet junction');
    assert.ok(Math.abs(diluted / reflux - .25) < 1e-12, 'three parts blood dilute one part reflux');
    assert.equal(mix(300, true), 0, 'graft fabric still blocks mixing');
});

test('retrograde junction mixing reaches a parent without entering a second inflow branch', () => {
    const net = new ContrastFlowNetwork([
        segment(0, 0, 1, [0, 0, 0], [20, 0, 0]),
        segment(1, 1, 2, [20, 0, 0], [52, 24, 0]),
        segment(2, 1, 3, [20, 0, 0], [52, -24, 0])
    ], {rootPoint: new THREE.Vector3(), cellLengthMm: 2, axialDispersionMm2PerS: 10});
    net.setFlowOverride([0], -200); net.setFlowOverride([1, 2], -100);
    net.edges[1].nextMassMg[0] = 1;
    const touched = [1], marks = new Uint8Array([0, 1, 0]);
    net._applyJunctionDispersion(.01, touched, 1, marks, 1);
    const received = net.edges[0].nextMassMg.at(-1);
    assert.ok(received > 0);
    assert.ok(net.edges[2].nextMassMg.every(m => m === 0));
    assert.ok(Math.abs(received + net.edges[1].nextMassMg[0] - 1) < 1e-12);
    assert.ok(touched.includes(0) && marks[0], 'newly reached parent must be committed after this step');
});
