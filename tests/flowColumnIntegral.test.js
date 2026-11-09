import test from 'node:test';
import assert from 'node:assert/strict';
import {Vector3} from 'three';
import {ContrastFlowNetwork} from '../src/contrast/flowNetwork.js';
import {FlowConcentrationField} from '../src/contrast/flowConcentrationField.js';
import {createFlowColumnIntegral} from '../src/contrast/flowColumnIntegral.js';

const near = (actual, expected, tolerance = 1e-7) => assert.ok(Math.abs(actual - expected) <= tolerance,
    `${actual} differs from ${expected}`);
function fixture({lengths = [24], cellLength = .75, branch = false} = {}) {
    let y = 0;
    const segments = lengths.map((length, id) => {
        const start = new Vector3(0, y, 0); y += length;
        return {id, nodeStartId: id, nodeEndId: id + 1, start, end: new Vector3(0, y, 0), radiusStart: 3, radiusEnd: 3};
    });
    if (branch) segments.push({id: lengths.length, nodeStartId: 1, nodeEndId: lengths.length + 1,
        start: segments[0].end.clone(), end: segments[0].end.clone().add(new Vector3(10, 0, 0)), radiusStart: 1, radiusEnd: 1});
    const network = new ContrastFlowNetwork(segments, {rootPoint: new Vector3(), cellLengthMm: cellLength});
    let count = 0;
    const offsets = Uint32Array.from(network.edges, edge => {const start = count; count += edge.cellCount; return start;});
    const field = new FlowConcentrationField(network, offsets, count), cells = new Float32Array(count);
    const territories = new Int32Array(network.edges.length), continuations = new Int32Array(network.edges.length).fill(-1);
    const edge = source => network.edges.find(e => e.sourceIndex === source);
    for (let source = 0; source < lengths.length - 1; source++) continuations[edge(source).index] = edge(source + 1).index;
    if (branch) territories[edge(lengths.length).index] = edge(lengths.length).index;
    const columns = createFlowColumnIntegral(network, field, territories, continuations);
    return {network, field, cells, columns, offsets, territories, edge,
        update() {field.update(cells); columns.update(field.values);},
        fill(source, c) {const e = edge(source); cells.fill(c, offsets[e.index], offsets[e.index] + e.cellCount);}};
}

test('column integration preserves the full uniform axial chord and reverses signed intervals', () => {
    const f = fixture();
    try {
        f.cells.fill(.5); f.update();
        near(f.columns.integral(0, 0, 24), 12);
        near(f.columns.integral(0, 24, 0), -12);
        near(f.columns.average(0, 24, 0), .5);
        near(f.columns.average(0, 12, 12), .5);
        near(f.columns.integral(0, -20, 40), 12);
        near(f.columns.average(0, -20, 40), .5);
    } finally {f.columns.dispose();}
});

test('a bolus between clear ends retains its iodine mass in an end-on projection', () => {
    const f = fixture();
    try {
        f.cells.fill(1, 12, 20); f.update();
        near(f.columns.sample(0, 0), 0); near(f.columns.sample(0, 24), 0);
        near(f.columns.integral(0, 0, 24), 6);
        near(f.columns.average(0, 0, 24), .25);
        near(f.columns.sample(0, 9), .5);
        near(f.columns.integral(0, 8.625, 9.375), .375);
        near(f.columns.integral(0, 15, 9), -5.8125);
    } finally {f.columns.dispose();}
});

test('current and previous histories remain independent in CPU queries and texture records', () => {
    const f = fixture();
    try {
        f.cells.fill(.25); f.update(); const previous = f.field.values.slice();
        f.cells.fill(.75); f.field.update(f.cells); f.columns.update(f.field.values, previous);
        near(f.columns.average(0, 0, 24), .75); near(f.columns.average(0, 0, 24, true), .25);
        const last = (f.columns.sampleCount - 1) * 4;
        assert.deepEqual(Array.from(f.columns.currentData.slice(last, last + 4)), [24, .75, 18, 0]);
        assert.deepEqual(Array.from(f.columns.previousData.slice(last, last + 4)), [24, .25, 6, 0]);
    } finally {f.columns.dispose();}
});

test('short side-on axial intervals avoid cancellation from the preceding iodine column', () => {
    const f = fixture({lengths: [1200]});
    try {
        f.cells.fill(.5); f.update();
        for (const width of [1e-4, 1e-7, 1e-10]) {
            const a = 1100.2, b = a + width;
            near(f.columns.average(0, a, b), .5, 1e-12);
            near(f.columns.integral(0, a, b), .5 * (b - a), 1e-15);
        }
    } finally {f.columns.dispose();}
});

test('an empty daughter cannot borrow iodine from a connected filled parent territory', () => {
    const f = fixture({lengths: [12, 12], branch: true});
    try {
        f.fill(0, 1); f.fill(1, 1); f.fill(2, 0); f.update();
        // Force a fully contrasted shared node: the daughter's existing
        // FlowConcentrationField gate must still leave its whole column clear.
        const daughter = f.edge(2);
        f.field.values[f.field.start[daughter.index]] = 1;
        f.columns.update(f.field.values);
        const territory = f.territories[daughter.index];
        near(f.columns.integral(territory, 0, 10), 0);
        near(f.columns.sample(territory, 0), 0);
        assert.ok(f.columns.integral(0, 0, 24) > 20);
        assert.ok(f.columns.ranges.get(0).start !== f.columns.ranges.get(territory).start);
    } finally {f.columns.dispose();}
});

test('a continuation through a fork has one bounded trace even when adjacent cell concentrations differ', () => {
    const f = fixture({lengths: [2.3, 3.7], cellLength: 1, branch: true});
    try {
        f.fill(0, 1); f.fill(1, .05); f.fill(2, .02); f.update();
        const parent = f.edge(0), child = f.edge(1), seam = parent.length;
        const a = seam - parent.cellLength * .5, b = seam + child.cellLength * .5;
        const ca = f.cells[f.offsets[parent.index] + parent.cellCount - 1], cb = f.cells[f.offsets[child.index]];
        const expected = ca + (cb - ca) * (seam - a) / (b - a);
        const transport = f.field.values.slice();
        near(f.columns.sample(0, seam), expected);
        near(f.columns.sample(0, seam - 1e-8), f.columns.sample(0, seam + 1e-8), 3e-8);
        // The seam remains a linear reconstruction of the actual neighbouring
        // centres, not an extra cap or a changed transport concentration.
        for (let i = 0; i <= 20; i++) {
            const t = i / 20, value = f.columns.sample(0, a + (b - a) * t);
            near(value, ca + (cb - ca) * t);
            assert.ok(value >= cb - 1e-12 && value <= ca + 1e-12);
        }
        near(f.columns.integral(0, a, b), .5 * (ca + cb) * (b - a));
        const endpoints = Array.from(f.columns.coordinates.keys()).filter(i => f.columns.coordinates[i] === seam);
        assert.equal(endpoints.length, 2);
        assert.equal(f.columns.currentSamples[endpoints[0]], f.columns.currentSamples[endpoints[1]]);
        assert.equal(f.columns.currentData[endpoints[0] * 4 + 1], f.columns.currentData[endpoints[1] * 4 + 1]);
        assert.deepEqual(f.field.values, transport);
    } finally {f.columns.dispose();}
});

test('a clear side-arm junction trace cannot cut a hole in a uniformly filled continuation', () => {
    const f = fixture({lengths: [12, 12], branch: true});
    try {
        f.fill(0, 1); f.fill(1, 1); f.fill(2, 0); f.update();
        const daughter = f.edge(2), territory = f.territories[daughter.index];
        // A shared transport node can represent another arm's inflow. The
        // main path's presentation must use its own adjacent cell centres.
        f.field.values[f.field.start[daughter.index]] = 0;
        const transport = f.field.values.slice();
        f.columns.update(f.field.values);
        near(f.columns.sample(0, 12), 1);
        near(f.columns.integral(0, 0, 24), 24);
        near(f.columns.integral(territory, 0, 10), 0);
        assert.deepEqual(f.field.values, transport);
    } finally {f.columns.dispose();}
});

test('a fork continuation tapers to a zero centre independently in both concentration histories', () => {
    const f = fixture({lengths: [2.3, 3.7], cellLength: 1, branch: true});
    try {
        f.fill(0, 1); f.fill(1, 0); f.fill(2, 0); f.update();
        const previous = f.field.values.slice();
        f.fill(0, 0); f.fill(1, .8); f.field.update(f.cells);
        f.columns.update(f.field.values, previous);
        const parent = f.edge(0), child = f.edge(1), seam = parent.length;
        const a = seam - parent.cellLength * .5, b = seam + child.cellLength * .5;
        for (const usePrevious of [false, true]) {
            const values = usePrevious ? previous : f.field.values;
            const ca = values[f.offsets[parent.index] + parent.cellCount - 1], cb = values[f.offsets[child.index]];
            for (let i = 0; i <= 20; i++) {
                const t = i / 20;
                near(f.columns.sample(0, a + (b - a) * t, usePrevious), ca + (cb - ca) * t);
            }
            near(f.columns.sample(0, seam - 1e-8, usePrevious), f.columns.sample(0, seam + 1e-8, usePrevious), 3e-8);
            near(f.columns.integral(0, a, b, usePrevious), .5 * (ca + cb) * (b - a));
            const records = usePrevious ? f.columns.previousData : f.columns.currentData;
            const endpoints = Array.from(f.columns.coordinates.keys()).filter(i => f.columns.coordinates[i] === seam);
            assert.equal(records[endpoints[0] * 4 + 1], records[endpoints[1] * 4 + 1]);
            near(f.columns.integral(f.territories[f.edge(2).index], 0, 10, usePrevious), 0);
        }
    } finally {f.columns.dispose();}
});

test('splitting graph edges or refining cells preserves a linear concentration field', () => {
    const fixtures = [fixture(), fixture({lengths: [6, 6, 12]}), fixture({lengths: [6, 6, 12], cellLength: .25}),
        fixture({lengths: [5.7, 6.9, 11.4], cellLength: .7})];
    try {
        for (const f of fixtures) {
            for (const edge of f.network.edges) for (let i = 0; i < edge.cellCount; i++)
                f.cells[f.offsets[edge.index] + i] = .125 + (edge.start.y + edge.cellLength * (i + .5)) / 32;
            f.field.update(f.cells);
            for (let n = 0; n < f.field.nodes.length; n++)
                f.field.values[f.field.cellCount + n] = .125 + f.field.nodes[n].point.y / 32;
            f.columns.update(f.field.values);
            near(f.columns.integral(0, 0, 24), 12);
            near(f.columns.integral(0, 2.2, 18.3), .125 * 16.1 + (18.3 ** 2 - 2.2 ** 2) / 64);
            // Unequal cell spacings produce non-binary Float32 samples.
            near(f.columns.average(0, 6 - 1e-7, 6 + 1e-7), .3125, 2e-8);
        }
    } finally {for (const f of fixtures) f.columns.dispose();}
});

test('duplicate edge seam coordinates add no artificial iodine or distance', () => {
    const f = fixture({lengths: [6, 18]});
    try {
        f.cells.fill(.5); f.update();
        const coordinates = f.columns.coordinates;
        const seams = Array.from(coordinates.keys()).filter(i => i > 0 && coordinates[i] === coordinates[i - 1]);
        assert.equal(seams.length, 1);
        const seam = seams[0];
        assert.equal(coordinates[seam], 6);
        assert.equal(f.columns.currentPrefix[seam], f.columns.currentPrefix[seam - 1]);
        near(f.columns.edgeStart[f.edge(1).index], 6);
        near(f.columns.ranges.get(0).length, 24);
        near(f.columns.integral(0, 0, 24), 12);
    } finally {f.columns.dispose();}
});
