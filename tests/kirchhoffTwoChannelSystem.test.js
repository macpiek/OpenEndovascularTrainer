import assert from 'node:assert/strict';
import test from 'node:test';
import { EndovascularPhysicsWorld } from '../src/physics/endovascularPhysicsWorld.js';
import { assembleKirchhoffTwoChannelSystem, solveKirchhoffTwoChannelSystem, nextKirchhoffTwoChannelTolerance } from '../src/physics/kirchhoffTwoChannelSystem.js';

const dt = 1 / 120, near = (a, b, message, tolerance = 1e-7) =>
    assert.ok(Number.isFinite(a) && Math.abs(a - b) <= tolerance, `${message}: ${a} vs ${b}`);
function fixture() {
    const world = new EndovascularPhysicsWorld(), gap = .01632478;
    const a = world.createRod('two-channel-wire', 2, 1, { mass: 1, adaptationCompliance: dt * dt / 99 });
    const b = world.createRod('two-channel-catheter', 2, 1, { mass: 1, adaptationCompliance: dt * dt / 99 });
    a.setPinned(0, true); b.inverseMass.fill(0);
    for (const body of [a, b]) for (const axis of [1, 2, 3]) body['inverseInertia' + axis].fill(0);
    a.restLength[0] = 10 + 100 * gap / 99;
    a.setNodePosition(1, 10 - 200 * gap, 0, 0);
    const normal = { kind: 'test-normal', strain: 0, alpha: 0, lambda: 0, lower: 0, upper: Infinity,
        gradients: [{ side: 0, dof: 6, value: 1 }] };
    const constraint = { innerBody: a, outerBody: b, kirchhoffContacts: [] };
    const options = { basis: 'individual', tolerance: 1e-9, includeSystem: true, additionalRows: [normal],
        channels(system) {
            return system.rows.slice(0, system.count).map(row => row.kind === 'material'
                ? { physical: 'pose', bias: { channel: 'bias-motion', strain: 0, alpha: row.alpha, lambda: 0, lower: -Infinity, upper: Infinity } }
                : { physical: 'physical-motion', bias: { channel: 'pose', strain: a.x[1] - 10, alpha: 0, lambda: 0, lower: 0, upper: Infinity } });
        } };
    return { world, a, b, constraint, options };
}

test('native two-channel rod response solves the stiff contact oracle without hardening finite compliance', () => {
    const f = fixture(), before = [f.a.x.slice(), f.a.restLength.slice(), f.a.adaptationCompliance];
    const result = solveKirchhoffTwoChannelSystem(f.constraint, dt, f.options);
    assert.equal(result.diagnostics.converged, true, JSON.stringify(result.diagnostics));
    const stiffness = 1 / (f.a.adaptationCompliance / (dt * dt));
    const physical = stiffness * (f.a.restLength[0] - 10), bias = 10 - f.a.x[1] - physical;
    assert.ok(physical > 0 && bias > 0);
    near(result.physical[0].correction[6], physical, 'physical momentum / elastic reaction');
    near(result.bias[0].correction[6], bias, 'compliant bias displacement');
    near(result.inner.correction[6], 10 - f.a.x[1], 'final nonpenetrating pose increment');
    near(result.additionalIncrement[0], 0, 'unloaded physical normal');
    near(result.biasAdditionalIncrement[0], (1 + stiffness) * bias, 'bias reaction includes the finite elastic metric');
    near(result.inner.lambda[3], physical, 'only physical adaptation multiplier is returned for application');
    near(result.bias[0].lambda[3], -stiffness * bias, 'separate bias adaptation multiplier');
    assert.ok(result.diagnostics.reconstructionResidual <= 1e-9);
    assert.deepEqual([f.a.x, f.a.restLength, f.a.adaptationCompliance], before, 'dry solve does not apply pose, reset rest strain or edit EI');
});

test('compact axial assembly preserves every entry of the two-channel operator',()=>{
    const world=new EndovascularPhysicsWorld(),count=40,bodies=[0,1].map(side=>{
        const body=world.createRod(`axial-${side}`,count,.5,{mass:side?.07:.03});
        for(let i=0;i<count;i++)body.setNodePosition(i,i*.5,side*.03,0);
        body.setPinned(0,true);return body;
    });
    const constraint={innerBody:bodies[0],outerBody:bodies[1],kirchhoffContacts:[]},options={condensation:'none',
        channels:native=>native.rows.slice(0,native.count).map(row=>({physical:'pose',bias:{channel:'bias-motion',strain:0,alpha:row.alpha,lambda:0,lower:-Infinity,upper:Infinity}}))};
    const dense=assembleKirchhoffTwoChannelSystem(constraint,dt,options),compact=assembleKirchhoffTwoChannelSystem(constraint,dt,{...options,matrixStorage:'general-band'});
    assert.equal(compact.matrixFormat,'general-band');assert.ok(compact.matrix.values.length<dense.matrix.length/5);
    for(let i=0;i<dense.count;i++)for(let j=0;j<dense.count;j++)assert.equal(j<compact.matrix.starts[i]||j>compact.matrix.ends[i]?0:compact.matrix.values[compact.matrix.offsets[i]+j],dense.matrix[i*dense.count+j]);
    for(const key of ['rhs','lower','upper','alpha'])assert.deepEqual(compact[key],dense[key]);
});

test('compact axial solve retains separate physical and bias reactions of the established solver',()=>{
    const f=fixture(),dense=solveKirchhoffTwoChannelSystem(f.constraint,dt,{...f.options,condensation:'none'}),
        compact=solveKirchhoffTwoChannelSystem(f.constraint,dt,{...f.options,condensation:'none',matrixStorage:'general-band'});
    assert.equal(compact.diagnostics.converged,true,JSON.stringify(compact.diagnostics));assert.equal(compact.diagnostics.linearSolver,'band-lu');
    for(const key of ['physical','bias'])for(let side=0;side<2;side++)for(const field of ['correction','lambda'])
        compact[key][side][field].forEach((v,i)=>near(v,dense[key][side][field][i],`${key}/${field}/${i}`,1e-8));
    assert.ok(compact.diagnostics.reconstructionResidual<=f.options.tolerance);
});

test('native immovable equations retain both RHS values and bound inexact linear refinement', () => {
    const f = fixture(), original = f.options.channels;
    f.options.additionalRows.push({ kind: 'fixed-pose', strain: 5e-6, alpha: 0, lambda: 0,
        lower: -Infinity, upper: Infinity, gradients: [] });
    f.options.channels = native => {
        const descriptors = original(native);
        descriptors[native.additionalOffset + 1] = { physical: 'pose', bias: { channel: 'bias-motion',
            strain: -7e-6, alpha: 0, lambda: 0, lower: -Infinity, upper: Infinity } };
        return descriptors;
    };
    f.options.tolerance = 2e-4;
    const result = solveKirchhoffTwoChannelSystem(f.constraint, dt, f.options);
    assert.equal(result.diagnostics.converged, true);
    assert.equal(result.diagnostics.structuralResidualFloor, 7e-6);
    const sorted = result.system.native.inverseOrder[result.system.native.additionalOffset + 1];
    assert.equal(result.system.rhs[result.system.physicalRows[sorted]], -5e-6);
    assert.equal(result.system.rhs[result.system.biasRows[sorted]], 7e-6);
    assert.equal(nextKirchhoffTwoChannelTolerance({ ...result.diagnostics, maximumResidual: 1.35e-4 }, 2e-4), 2e-5);
    assert.equal(nextKirchhoffTwoChannelTolerance({ ...result.diagnostics, maximumResidual: 1e-5 }, 2e-5), 7e-6);
    assert.equal(nextKirchhoffTwoChannelTolerance({ ...result.diagnostics, maximumResidual: 7e-6 }, 2e-5), null);
    assert.equal(nextKirchhoffTwoChannelTolerance({ ...result.diagnostics, converged: false }, 2e-4), null);
    assert.equal(nextKirchhoffTwoChannelTolerance(result.diagnostics, 7e-6), null, 'never relax requested inner accuracy');
});

test('pose control rows retain cross-channel response while physical contact rows exclude bias velocity', () => {
    const f = fixture();
    f.options.additionalRows.push({ kind: 'control', strain: 0, alpha: .3, lambda: 0, lower: -Infinity, upper: Infinity,
        gradients: [{ side: 0, dof: 6, value: 1 }] });
    const original = f.options.channels;
    f.options.channels = native => {
        const rows = original(native);
        rows[native.additionalOffset + 1] = { physical: 'pose', bias: { channel: 'bias-motion', strain: 0,
            alpha: .3, lambda: 0, lower: -Infinity, upper: Infinity } };
        return rows;
    };
    const s = assembleKirchhoffTwoChannelSystem(f.constraint, dt, f.options), n = s.native;
    const normal = n.inverseOrder[n.additionalOffset], control = n.inverseOrder[n.additionalOffset + 1];
    const at = (i, j) => s.matrix[i * s.count + j];
    near(at(s.physicalRows[control], s.biasRows[normal]), 1, 'physical pose control reads geometric correction');
    near(at(s.physicalRows[normal], s.biasRows[control]), 0, 'normal physical velocity excludes bias');
    near(at(s.biasRows[normal], s.physicalRows[control]), 1, 'geometric gap sees physical correction');
    near(at(s.biasRows[control], s.physicalRows[normal]), 0, 'bias control motion excludes physical correction');
    near(at(s.physicalRows[control], s.physicalRows[control]), 1.3, 'original control compliance');
});

test('two-channel assembly requires complete explicit row-channel metadata and rejects accidental bias-bank assembly', () => {
    const f = fixture();
    assert.throws(() => assembleKirchhoffTwoChannelSystem(f.constraint, dt, { ...f.options, channels: [] }), /every native row/);
    assert.throws(() => assembleKirchhoffTwoChannelSystem(f.constraint, dt, { ...f.options,
        channels: n => f.options.channels(n).map(({ physical }) => ({ physical })) }), /membership/);
    f.constraint._splitMotion = { phase: 'bias' };
    assert.throws(() => assembleKirchhoffTwoChannelSystem(f.constraint, dt, f.options), /physical material banks/);
});

test('both response channels and increments remain owned after a later native assembly', () => {
    const f = fixture(), result = solveKirchhoffTwoChannelSystem(f.constraint, dt, f.options);
    assert.equal(result.diagnostics.converged, true, JSON.stringify(result.diagnostics));
    const saved = structuredClone({ physical: result.physical, bias: result.bias,
        physicalIncrement: result.physicalIncrement, biasIncrement: result.biasIncrement, total: result.inner.correction });
    f.a.x[1] += .1;
    assembleKirchhoffTwoChannelSystem(f.constraint, dt, f.options);
    assert.deepEqual({ physical: result.physical, bias: result.bias,
        physicalIncrement: result.physicalIncrement, biasIncrement: result.biasIncrement, total: result.inner.correction }, saved);
});

test('integrated condensation matches the dense response without allocating the expanded matrix', () => {
    const f = fixture();
    const dense = solveKirchhoffTwoChannelSystem(f.constraint, dt, { ...f.options, condensation: 'none' });
    const condensed = solveKirchhoffTwoChannelSystem(f.constraint, dt, f.options);
    assert.equal(dense.diagnostics.converged, true);
    assert.equal(condensed.diagnostics.converged, true);
    assert.equal(condensed.diagnostics.condensation, 'condensed');
    assert.equal(condensed.system.matrix, null);
    assert.ok(condensed.system.condensed.count < condensed.system.count);
    for (const key of ['physicalIncrement', 'biasIncrement']) condensed[key].forEach((value, i) => near(value, dense[key][i], key + i));
    for (let side = 0; side < 2; side++) for (const channel of ['physical', 'bias'])
        condensed[channel][side].correction.forEach((value, i) => near(value, dense[channel][side].correction[i], channel + i));
    assert.equal(condensed.diagnostics.materialCondensation.fullExpandedMatrixEntries, 0);
    assert.equal(condensed.diagnostics.materialCondensation.factorizations, 1);
});

test('unsupported paired compliance takes the explicit dense fallback and preserves the supplied alpha', () => {
    const f = fixture(), source = f.options.channels;
    f.options.channels = native => source(native).map((entry, i) => i === 3
        ? { ...entry, bias: { ...entry.bias, alpha: entry.bias.alpha * 2 } } : entry);
    const system = assembleKirchhoffTwoChannelSystem(f.constraint, dt, { ...f.options, condensation: 'auto' });
    assert.equal(system.condensed, null);
    assert.equal(system.condensation.reason, 'unequal-bias-alpha');
    assert.equal(system.matrix.length, system.count ** 2);
    assert.equal(system.descriptors[3].bias.alpha, 2 * system.native.rows[3].alpha);
});

test('full generalized-response certification rejects a corrupted reconstruction after a converged retained solve', () => {
    const f = fixture(), source = f.options.channels;
    let native;
    f.options.channels = n => { native = n; return source(n); };
    f.options.debugCoulombResult = ({ result }) => {
        assert.equal(result.diagnostics.converged, true);
        // Fault injection models a stale/corrupted mobility between the Schur
        // solve and response recovery. Retained convergence cannot certify it.
        native.material[0].weight[6] *= 2;
    };
    const result = solveKirchhoffTwoChannelSystem(f.constraint, dt, f.options);
    assert.equal(result.diagnostics.condensation, 'condensed');
    assert.equal(result.diagnostics.converged, false);
    assert.equal(result.diagnostics.status, 'two-channel-reconstruction-residual');
    assert.ok(result.diagnostics.reconstructionResidual > f.options.tolerance);
});
