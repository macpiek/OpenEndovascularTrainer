import assert from 'node:assert/strict';
import { performance } from 'node:perf_hooks';
import os from 'node:os';
import * as old from './base/src/physics/kirchhoffCompositeJointSurfaceMotion.js';
import * as current from './src/physics/kirchhoffCompositeJointSurfaceMotion.js';
import { captureCompositeReferenceFrames } from './src/physics/kirchhoffCompositeElement.js';
const dot = (a, b) => a.reduce((s, v, k) => s + v * b[k], 0);
const cross = (a, b) => [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]];
const add = (a, b) => a.map((v, k) => v + b[k]), sub = (a, b) => a.map((v, k) => v - b[k]);
const scale = (a, f) => a.map(v => v * f), unit = a => scale(a, 1 / Math.hypot(...a));
const close = (a, b, eps = 2e-10) => assert.ok(Number.isFinite(a) && Number.isFinite(b) && Math.abs(a - b) <= eps, `${a} != ${b}`);
const same = (a, b, eps) => { assert.equal(a.length, b.length); a.forEach((v, k) => close(v, b[k], eps)); };
function rotate(v, axis, angle) { const n = unit(axis), c = Math.cos(angle), s = Math.sin(angle); return add(add(scale(v, c), scale(cross(n, v), s)), scale(n, dot(n, v) * (1 - c))); }
function tool(id, previous, dt, { st = 0, sx = 1, label = 20, coordinate = 1 } = {}) {
    return { id, edgeId: `${id}:edge`, coordinates: [0, 2], coordinate, positions: structuredClone(previous), previousPositions: structuredClone(previous),
        reference: captureCompositeReferenceFrames(previous)[0], angle: .2, previousAngle: .2, angleRate: 0,
        positionRates: [[0, 0, 0], [0, 0, 0]], materialMap: { sStart: label, dsDx: sx, dsDt: st },
        materialPath: { kind: 'linear-affine-maps', previousEdgeId: `${id}:edge`, previousMap: { sStart: label - dt * st, dsDx: sx } } };
}
function fixture() {
    const dt = .02, w = tool('wire', [[0, 0, 0], [2, .3, .1]], dt, { st: -.5, sx: 1.3, coordinate: .8 }),
        c = tool('catheter', [[0, -.4, .2], [2, -.15, -.1]], dt, { st: .3, sx: .8, label: 100, coordinate: 1.1 });
    w.positions = [[.03, -.01, .02], [2.01, .32, .09]]; c.positions = [[-.02, -.39, .19], [2.04, -.17, -.08]];
    w.positionRates = [[.2, .1, -.05], [-.1, .2, .1]]; c.positionRates = [[-.1, .2, .05], [.3, -.1, .2]];
    w.angle = .5; w.angleRate = 3; c.angle = -.4; c.angleRate = -2;
    return { dt, rateMode: 'instantaneous', contact: { point: [.7, .2, .1], axes: [[1, 0, 0], [0, 0, 1]] }, tools: [w, c] };
}
function change(input, column, amount) {
    const n = 7 * input.tools.length, rates = column >= n, index = column % n, t = input.tools[Math.floor(index / 7)], local = index % 7;
    if (local === 6) t[rates ? 'angleRate' : 'angle'] += amount;
    else t[rates ? 'positionRates' : 'positions'][Math.floor(local / 3)][local % 3] += amount;
}
function straight(dt = .02) {
    const tools = [tool('wire', [[-1, -.25, 0], [1, -.25, 0]], dt), tool('catheter', [[-1, -.5, 0], [1, -.5, 0]], dt, { label: 100 })];
    return { dt, rateMode: 'instantaneous', rotationPath: 'short-contact-frame-own-unwrapped-spins', radii: { inner: .25, outer: .5 },
        contact: { point: [0, 0, 0], axes: [[1, 0, 0], [0, 0, 1]] }, tools };
}

function sideQuery(tools, previous = false) {
    const centers = tools.map(t => {
        const q = previous ? t.previousPositions : t.positions, f = (t.coordinate - t.coordinates[0]) / (t.coordinates[1] - t.coordinates[0]);
        return add(q[0], scale(sub(q[1], q[0]), f));
    });
    const q = previous ? tools[1].previousPositions : tools[1].positions, tangent = unit(sub(q[1], q[0])), delta = sub(centers[0], centers[1]);
    const normal = unit(sub(delta, scale(tangent, dot(delta, tangent))));
    return { point: add(centers[1], scale(normal, .5)), normal, tangent };
}
function generalFixture() {
    const input = fixture(); input.rotationPath = 'short-contact-frame-own-unwrapped-spins';
    input.finiteGeometry = { kind: 'explicit-affine-side-queries', current: sideQuery(input.tools), previous: sideQuery(input.tools, true) }; return input;
}
function forceFixture(count = 2) {
    const input = generalFixture(); input.forceGeometry = { kind: 'explicit-affine-side-query', ...input.finiteGeometry.current };
    input.tools = input.tools.slice(0, count); return input;
}
function variedForceFixture(count, sample) {
    let state = 0x4a31 + sample * 127 + count * 947;
    const random = () => { state = (Math.imul(1664525, state) + 1013904223) >>> 0; return state / 0x100000000; };
    const vec = () => Array.from({ length: 3 }, () => 2 * random() - 1);
    const tools = Array.from({ length: count }, (_, i) => {
        const oldTangent = unit(vec()), oldStart = vec(), previous = [oldStart, add(oldStart, scale(oldTangent, .8 + 2 * random()))];
        const t = tool(`body-${i}`, previous, .01, { coordinate: .05 + 1.9 * random() });
        let currentTangent = unit(vec());
        if (dot(oldTangent, currentTangent) < -.93) currentTangent = scale(currentTangent, -1);
        if (sample === 0) currentTangent = oldTangent;
        if (sample === 1) currentTangent = unit(add(oldTangent, scale(t.reference.director, 1e-7)));
        if (sample === 2) currentTangent = unit(add(scale(oldTangent, -1), scale(t.reference.director, .1)));
        const start = vec(); t.positions = [start, add(start, scale(currentTangent, .7 + 2 * random()))];
        t.reference.director = rotate(t.reference.director, oldTangent, (i + 1) * .7 + 4 * Math.PI * (sample - 4));
        t.angle = 6 * Math.PI * (sample - 5) + .3 + i;
        return t;
    });
    const tangent = vec(), normal = add(vec(), scale(unit(cross(tangent, [1, .2, -.4])), 2));
    return { tools, forceGeometry: { kind: 'explicit-affine-side-query', point: vec(), normal, tangent } };
}


const rows = [], blocks = 21, callsPerBlock = 250, warmupCalls = 1000;
const median = a => a.slice().sort((a, b) => a - b)[Math.floor(a.length / 2)];
let sink = 0;
for (const count of [1, 2]) for (const fixtureKind of ['mild-current-root-fixture', 'varied-nonparallel']) for (const order of ['full', 'value']) {
    const fixtures = (fixtureKind.startsWith('mild') ? [forceFixture(count)] : Array.from({ length: 16 }, (_, i) => variedForceFixture(count, i + 3))).map(x => ({ ...x, order }));
    const workspaces = [old, current].map(m => m.createCompositeJointSurfaceForceMapWorkspace(count));
    const operators = [old, current].map((m, j) => i => m.evaluateCompositeJointSurfaceForceMap(fixtures[i % fixtures.length], workspaces[j]));
    for (const f of fixtures) {
        const a = old.evaluateCompositeJointSurfaceForceMap(f, workspaces[0]), b = current.evaluateCompositeJointSurfaceForceMap(f, workspaces[1]);
        same(a.forceMap, b.forceMap, 3e-11); if (order === 'full') same(a.derivative, b.derivative, 3e-10);
    }
    const run = (which, calls) => {
        const start = performance.now();
        for (let i = 0; i < calls; i++) { const r = operators[which](i); sink += r.forceMap[i % r.forceMap.length] + (order === 'full' ? r.derivative[i % r.derivative.length] : 0); }
        return (performance.now() - start) / calls;
    };
    for (let k = 0; k < 4; k++) run(k % 2, warmupCalls / 2);
    const timings = [[], []];
    for (let block = 0; block < blocks; block++) for (const which of block % 2 ? [1, 0] : [0, 1]) timings[which].push(run(which, callsPerBlock));
    const oldMedianMs = median(timings[0]), newMedianMs = median(timings[1]);
    const storage = workspaces.map(w => Object.fromEntries(Object.entries(w).filter(([, v]) => v?.data instanceof Float64Array).map(([k, a]) => [k, { bytes: a.data.byteLength, usedNodes: a.used(), stride: a.stride }])));
    rows.push({ count, fixtureKind, order, oldMedianMs, newMedianMs, speedup: oldMedianMs / newMedianMs, reductionPercent: 100 * (1 - newMedianMs / oldMedianMs), pairedMedianSpeedup: median(timings[0].map((x, j) => x / timings[1][j])), timingBlocksMsPerCall: { old: timings[0], current: timings[1] }, storage });
}
console.log(JSON.stringify({ node: process.version, platform: process.platform, arch: process.arch, cpu: os.cpus()[0].model, blocks, callsPerBlock, warmupCallsPerVariant: warmupCalls, timingScope: 'Complete evaluateCompositeJointSurfaceForceMap call with reused workspace, input validation, full B/DB construction and owned output allocation; alternate old/new order each block, no explicit GC', rows, sink }, null, 2));
