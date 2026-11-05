import test from 'node:test';
import assert from 'node:assert/strict';
import {Vector3, CylinderGeometry} from 'three';
import {ContrastFlowNetwork} from '../src/contrast/flowNetwork.js';
import {FlowConcentrationField} from '../src/contrast/flowConcentrationField.js';
import {JunctionSurfaceReconstruction} from '../src/contrast/junctionSurfaceReconstruction.js';
import {AnatomicalContrastSurface} from '../src/contrast/anatomicalContrastSurface.js';

const point = (x, y, z = 0) => new Vector3(x, y, z);
const segment = (id, from, to, a, b) => ({id, nodeStartId: from, nodeEndId: to,
    start: point(...a), end: point(...b), radiusStart: 4, radiusEnd: 4});
const close = (a, b, tolerance = 1e-7) => assert.ok(Math.abs(a - b) <= tolerance, `${a} != ${b}`);
function fixture(overlap = false) {
    const network = new ContrastFlowNetwork(overlap ? [
        segment(0, 0, 1, [0, 30], [0, 0]),
        segment(1, 1, 2, [0, 0], [-20, -30]),
        segment(2, 1, 3, [0, 0], [14, 0]),
        segment(3, 3, 4, [14, 0], [40, -15]),
        segment(4, 3, 5, [14, 0], [40, 15])
    ] : [
        segment(0, 0, 1, [0, 30], [0, 0]),
        segment(1, 1, 2, [0, 0], [-30, -40]),
        segment(2, 1, 3, [0, 0], [30, -40])
    ], {rootPoint: point(0, 30), cellLengthMm: 2});
    let count = 0;
    const offsets = Uint32Array.from(network.edges, edge => {
        const offset = count; count += edge.cellCount; return offset;
    });
    const field = new FlowConcentrationField(network, offsets, count);
    const reconstruction = new JunctionSurfaceReconstruction(network, field,
        Int32Array.from(network.edges, edge => edge.index), new Int32Array(network.edges.length).fill(-1));
    const cells = new Float32Array(count);
    const edge = source => network.edges.find(e => e.sourceIndex === source);
    const fill = (source, value) => {
        const e = edge(source); cells.fill(value, offsets[e.index], offsets[e.index] + e.cellCount);
    };
    const flow = (source, value) => network.setFlowOverride([edge(source).index], value);
    const add = (position, source, t) => {
        const e = edge(source);
        return reconstruction.addSample(position, e.index, field.stencil(e.index, t));
    };
    const finish = () => {
        reconstruction.finish();
        const extended = new Float32Array(reconstruction.offset + reconstruction.sampleCount);
        extended.set(field.values); field.values = extended;
    };
    const update = () => { field.update(cells); reconstruction.update(field.values); };
    const read = index => field.values[reconstruction.offset + index];
    return {network, field, reconstruction, cells, offsets, edge, fill, flow, add, finish, update, read};
}

test('junction field preserves uniform concentration and stays within transported concentrations', () => {
    const f = fixture(), samples = [];
    for (let x = -12; x <= 12; x += 2) for (let y = -8; y <= 8; y += 2) {
        const index = f.add(point(x, y, 3), 0, .9);
        if (index >= 0) samples.push(index);
    }
    f.finish();
    f.flow(0, 1000); f.flow(1, -2000); f.flow(2, 3000);
    f.cells.fill(.42); f.update();
    assert.ok(samples.length > 20);
    for (const index of samples) close(f.read(index), .42);
    f.fill(0, .03); f.fill(1, .8); f.fill(2, .12); f.update();
    for (const index of samples) assert.ok(f.read(index) >= 0 && f.read(index) <= .800001);
});

test('a selective outflow injection and a not-yet-arrived reflux cannot activate the common lumen', () => {
    const f = fixture(), parent = f.add(point(0, 2, 3), 0, .9), sibling = f.add(point(3, -3, 2), 2, .1);
    f.finish(); f.flow(0, 1000); f.flow(1, 500); f.flow(2, 500);
    f.fill(1, .9); f.update();
    close(f.read(parent), 0); close(f.read(sibling), 0);
    f.flow(1, -500); f.flow(2, 1500);
    const source = f.edge(1);
    f.cells.fill(0, f.offsets[source.index], f.offsets[source.index] + 4);
    f.update();
    close(f.read(parent), 0); close(f.read(sibling), 0);
});

test('a reflux bolus moving from a remote arm anchor cannot prefill the shared lumen', () => {
    const f = fixture(), samples = [
        f.add(point(0, 2, 2), 0, .95),
        f.add(point(3, -3, 2), 2, .1),
        f.add(point(0, -2, 2), 1, .02)
    ];
    f.finish(); f.flow(0, 1000); f.flow(1, -500); f.flow(2, 1500);
    const source = f.edge(1), offset = f.offsets[source.index];
    for (const frontMm of [16, 12, 8, 6]) {
        f.cells.fill(0);
        for (let i = 0; i < source.cellCount; i++)
            if ((i + .5) * source.cellLength >= frontMm) f.cells[offset + i] = .8;
        f.update();
        for (const sample of samples) close(f.read(sample), 0, 0);
    }
    f.fill(1, .8); f.update();
    assert.ok(samples.some(i => f.read(i) > 0), 'The local front becomes visible when iodine actually reaches the junction');
});

test('the same common-lumen point has one concentration regardless of its triangle territory during a steep front', () => {
    const f = fixture(), pairs = [];
    for (const position of [point(-2, -2, 2), point(0, -3, 2), point(3, -3, 2)]) {
        pairs.push([f.add(position, 0, .95), f.add(position, 1, .05), f.add(position, 2, .05)]);
    }
    assert.ok(pairs.flat().every(i => i >= 0));
    f.finish(); f.flow(0, 3000); f.flow(1, -100); f.flow(2, 3100);
    const source = f.edge(1), offset = f.offsets[source.index];
    for (const frontMm of [10, 4, 0]) {
        f.cells.fill(0);
        for (let i = 0; i < source.cellCount; i++)
            if ((i + .5) * source.cellLength >= frontMm) f.cells[offset + i] = .8;
        f.update();
        for (const samples of pairs) {
            const concentrations = samples.map(i => f.read(i));
            assert.ok(Math.max(...concentrations) - Math.min(...concentrations) < 1e-7,
                `Triangle ownership cannot restore a sharp lip at front ${frontMm} mm: ${concentrations}`);
        }
    }
});

test('crossing zero reflux flow does not abruptly switch the reconstructed concentration', () => {
    const f = fixture(), index = f.add(point(-2, -2, 2), 1, .05);
    f.finish(); f.fill(1, .8); f.flow(0, 1000);
    let previous;
    for (const q of [-1e-5, -1e-8, 0, 1e-8, 1e-5]) {
        f.flow(1, q); f.flow(2, 1000 - q); f.update();
        const value = f.read(index);
        if (previous !== undefined) assert.ok(Math.abs(value - previous) < 1e-5,
            `Flow sign crossing must not expose another spatial field: ${previous} → ${value}`);
        previous = value;
    }
});

test('a daughter projection crossing the hub plane continuously joins the proximal blood column', () => {
    const f = fixture(), pairs = [];
    // For the left daughter direction (-.6, -.8), these nearby wall points
    // straddle its zero axial projection while retaining a nonzero arm weight.
    for (const y of [1, 2, 3, 4]) {
        const x = -4 * y / 3;
        pairs.push([f.add(point(x - 1e-6, y, 0), 0, .9),
            f.add(point(x + 1e-6, y, 0), 0, .9)]);
    }
    assert.ok(pairs.flat().every(i => i >= 0));
    f.finish(); f.fill(0, .02); f.fill(1, .9); f.fill(2, .1);
    f.flow(0, 1000); f.flow(1, -2000); f.flow(2, 3000); f.update();
    for (const [before, after] of pairs) close(f.read(before), f.read(after), 1e-5);
});

test('overlapping connected junction supports form one continuous field', () => {
    const f = fixture(true), samples = [];
    assert.equal(f.reconstruction.junctions.length, 2);
    for (let x = 5; x <= 12; x += .02) samples.push(f.add(point(x, 4, 0), 2, x / 14));
    assert.ok(samples.every(index => index >= 0));
    f.finish();
    [.1, .9, .4, .2, .8].forEach((c, i) => f.fill(i, c));
    f.flow(0, 1000); f.flow(1, -1000); f.flow(2, 2000); f.flow(3, 1000); f.flow(4, 1000);
    f.update();
    for (let i = 1; i < samples.length; i++) assert.ok(Math.abs(f.read(samples[i]) - f.read(samples[i - 1])) < .01,
        `Overlapping junctions cannot select different fields at a winner boundary (sample ${i})`);
});

test('an emptied junction clears its virtual values and leaves samples outside support untouched', () => {
    const f = fixture();
    assert.equal(f.add(point(100, 100, 100), 0, .9), -1);
    const index = f.add(point(0, 0, 2), 0, .95);
    f.finish(); f.cells.fill(.5); f.flow(0, 1000); f.flow(1, 500); f.flow(2, 500); f.update();
    assert.ok(f.read(index) > 0);
    f.cells.fill(0); f.update(); close(f.read(index), 0);
    f.update(); close(f.read(index), 0);
    assert.equal(f.reconstruction.updatedSamples, 0, 'An empty unchanged patch must remain asleep');
});

test('explicit graft or excluded-lumen masks cannot be repainted from opacified junction neighbours', () => {
    const f = fixture(), blocked = f.add(point(-4, -4, 0), 1, .1), clear = f.add(point(0, 2, 2), 0, .9);
    f.finish(); f.cells.fill(.5); f.flow(0, 1000); f.flow(1, 500); f.flow(2, 500); f.update();
    assert.ok(f.read(blocked) > 0 && f.read(clear) > 0);
    const edge = f.edge(1), mask = new Uint8Array(f.cells.length);
    mask.fill(1, f.offsets[edge.index], f.offsets[edge.index] + edge.cellCount);
    f.fill(1, 0); f.field.update(f.cells); f.reconstruction.update(f.field.values, mask);
    close(f.read(blocked), 0, 0);
    assert.ok(f.read(clear) > 0, 'The explicit mask must stay local to the excluded lumen');
    f.fill(1, .5); mask.fill(0); f.field.update(f.cells); f.reconstruction.update(f.field.values, mask);
    assert.ok(f.read(blocked) > 0, 'Removing a temporary graft mask must restore the anatomical field');
});

test('draw-chunk dependencies include a short bolus between the junction and a remote arm anchor', () => {
    const segments = [];
    for (let i = 0; i < 10; i++) segments.push(segment(i, i, i + 1, [0, 30 - i * 3], [0, 27 - i * 3]));
    for (const side of [-1, 1]) for (let i = 0; i < 10; i++) {
        const base = side < 0 ? 10 : 20;
        segments.push(segment(segments.length, i === 0 ? 10 : base + i, base + i + 1,
            [side * i * 2.4, -i * 3.2], [side * (i + 1) * 2.4, -(i + 1) * 3.2]));
    }
    const network = new ContrastFlowNetwork(segments, {rootPoint: point(0, 30), cellLengthMm: 2});
    let count = 0;
    const offsets = Uint32Array.from(network.edges, e => {const i = count; count += e.cellCount; return i;});
    const field = new FlowConcentrationField(network, offsets, count);
    const territories = Int32Array.from(network.edges, e => e.sourceIndex < 10 ? 0 : e.sourceIndex < 20 ? 1 : 2);
    const continuations = Int32Array.from(network.edges, e => e.childEdgeIndices.length === 1 ? e.childEdgeIndices[0] : -1);
    const reconstruction = new JunctionSurfaceReconstruction(network, field, territories, continuations);
    assert.equal(reconstruction.junctions.length, 1);
    const parent = network.edges.find(e => e.sourceIndex === 9);
    const sample = reconstruction.addSample(point(-4, -4, 0), parent.index, field.stencil(parent.index, .9));
    assert.ok(sample >= 0); reconstruction.finish();
    field.values = new Float32Array(reconstruction.offset + reconstruction.sampleCount);
    const cells = new Float32Array(count), middle = network.edges.find(e => e.sourceIndex === 11);
    cells.fill(.8, offsets[middle.index], offsets[middle.index] + middle.cellCount);
    field.update(cells); reconstruction.update(field.values);
    assert.ok(field.values[reconstruction.offset + sample] > 0, 'A local arm sample must see the short bolus');
    const dependencies = new Set(); reconstruction.addChunkDependencies(sample, dependencies);
    assert.ok(dependencies.has(middle.index),
        'An opacified local arm segment must keep its reconstruction draw chunk visible even when both remote endpoints are clear');
});

test('surface virtual samples keep current and previous texture channels aligned through updates and reset', () => {
    const f = fixture(), geometry = new CylinderGeometry(4, 4, 60, 24, 30, true);
    for (let i = 0; i < geometry.index.count; i += 3) {
        const b = geometry.index.getX(i + 1);
        geometry.index.setX(i + 1, geometry.index.getX(i + 2)); geometry.index.setX(i + 2, b);
    }
    const wall = geometry.toNonIndexed();
    const surface = new AnatomicalContrastSurface(f.network, f.offsets, {fallbackGeometry: wall});
    const plume = new Float32Array(f.cells.length);
    const fill = concentration => {
        for (const edge of f.network.edges) for (let i = 0; i < edge.cellCount; i++)
            edge.massMg[i] = edge.volumes[i] * .3 * concentration;
    };
    try {
        assert.ok(surface.junctionField.sampleCount > 0, 'Exercise appended virtual samples, not just original graph cells');
        const offset = surface.junctionField.offset;
        f.flow(0, 1000); f.flow(1, 500); f.flow(2, 500);
        fill(.4); surface.update(plume, .3, true);
        fill(.2); surface.update(plume, .3, true);
        for (let i = offset; i < surface.current.length; i++) {
            close(surface.current[i], .2); close(surface.previous[i], .4);
            close(surface.data[i * 4], .2); close(surface.data[i * 4 + 1], .4);
        }
        fill(.1); surface.update(plume, .3, false);
        for (let i = offset; i < surface.current.length; i++) {
            close(surface.current[i], .1); close(surface.previous[i], .4);
        }
        const stencil = surface.mesh.geometry.attributes.flowStencil;
        for (let i = 0; i < stencil.count; i++) {
            assert.ok(stencil.getX(i) < surface.current.length && stencil.getY(i) < surface.current.length);
        }
        surface.resetHistory();
        assert.ok(surface.current.every(v => v === 0) && surface.previous.every(v => v === 0));
        fill(0); surface.update(plume, .3, true);
        assert.ok(surface.data.every(v => v === 0));
    } finally { surface.dispose(); wall.dispose(); geometry.dispose(); }
});
