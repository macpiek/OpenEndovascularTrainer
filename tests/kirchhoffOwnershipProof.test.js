import assert from 'node:assert/strict';
import test from 'node:test';
import { resolve } from 'node:path';
import { pathToFileURL } from 'node:url';
import { Vector3, Quaternion } from 'three';

// Small helper-level proofs; no World, scene, timestep replay or private
// ledger inspection. Point-force wrenches below are an independent oracle.
const sourceRoot = process.env.OET_OWNERSHIP_PROOF_SOURCE_ROOT
    ? pathToFileURL(resolve(process.env.OET_OWNERSHIP_PROOF_SOURCE_ROOT) + '/')
    : new URL('../', import.meta.url);
const source = name => import(new URL(`src/physics/${name}.js`, sourceRoot));
const [{ buildKirchhoffSurfaceFriction }, ownership] = await Promise.all([
    source('kirchhoffSurfaceFriction'), source('kirchhoffToolContactOwnership')
]);
const axes = ['X', 'Y', 'Z', 'W'];
const vec = values => new Vector3(...values);
function setFrame(body, q) {
    q.toArray().forEach((value, i) => { body['orientation' + axes[i]][0] = value; });
}

function straightRod(y, radius) {
    const body = { count: 2, segmentCount: 1, activeStart: 0, activeEnd: 1,
        radius, nodeRadius: new Float64Array(2).fill(radius),
        inverseMass: new Float64Array(2).fill(1), orientationControlSegment: -1 };
    for (const [key, values] of Object.entries({ x: [0, 0], y: [y, y], z: [0, 1] })) {
        body[key] = Float64Array.from(values);
        body['previous' + key.toUpperCase()] = Float64Array.from(values);
    }
    for (const prefix of ['orientation', 'previousOrientation']) for (const axis of axes)
        body[prefix + axis] = Float64Array.of(Number(axis === 'W'));
    for (const axis of ['X', 'Y', 'Z']) body['toolProjection' + axis] = new Float64Array(2);
    for (let axis = 1; axis <= 3; axis++) body['inverseInertia' + axis] = Float64Array.of(1);
    return body;
}

// Pull an emitted row into physical world forces and moments about a fixed
// origin. This knows no ledger storage, contact normals or release algorithm.
function wrench(rows, bodies, scale = 1) {
    const sides = bodies.map(() => ({ force: new Vector3(), moment: new Vector3() }));
    for (const row of rows) {
        assert.equal(row.lower, row.upper, 'a release carrier has a prescribed final value');
        const delta = scale * (row.lower - row.lambda);
        for (const g of row.gradients) {
            const body = bodies[g.side], node = Math.floor(g.dof / 6), axis = g.dof % 6;
            const v = new Vector3().setComponent(axis % 3, delta * g.value);
            if (axis < 3) {
                sides[g.side].force.add(v);
                sides[g.side].moment.add(new Vector3(body.x[node], body.y[node], body.z[node]).cross(v));
            } else {
                const q = new Quaternion(...axes.map(key => body['orientation' + key][node]));
                sides[g.side].moment.add(v.applyQuaternion(q));
            }
        }
    }
    return sides;
}

function loadedPair() {
    // Circular tools touch at P. An axial tangential load has a transverse
    // moment, which must survive independent spin of either material frame.
    const wire = straightRod(0, 0.4445), catheter = straightRod(1.2778, 0.8333);
    const point = new Vector3(0, 0.4445, 0.5), fn = 0.4, ft = 0.1;
    const tool = { bodyA: wire, bodyB: catheter, lambdas: Float64Array.of(fn), openDistalB: true };
    const contact = { normalLambda: fn, tangentLambda: new Float64Array(2),
        tangentU: [0, 0, 1], tangentV: [1, 0, 0], twistLambda: 0 };
    const joint = { innerBody: wire, outerBody: catheter, innerRadius: 0.485,
        axialFriction: 0.5, circumferentialFriction: 0.5,
        _coupledExternalFriction: { owners: new Map([[tool, new Map([[0, contact]])]]) } };
    const surface = buildKirchhoffSurfaceFriction(joint, {
        kind: 'side', normal: [0, 1, 0], surfaceContactPoint: point.toArray(),
        innerWeights: [0.5, 0.5], outerWeights: [0.5, 0.5],
        _innerSegmentIndex: 0, _outerSegmentIndex: 0, manifoldContact: contact
    }, 1 / 120);
    assert.equal(surface.supported, true);
    assert.ok(vec(surface.axes[0]).distanceTo(new Vector3(0, 0, 1)) < 1e-12);
    // Normal forces act through the two centerlines; the surface builder
    // supplies the complete axial friction Jacobian, including material spin.
    const normal = [0, 1].flatMap(side => [0, 1].map(node => ({
        side, dof: node * 6 + 1, value: (side ? 1 : -1) * 0.5
    })));
    ownership.beginKirchhoffToolReactionStep(tool);
    ownership.recordKirchhoffToolReaction(tool, 0, ownership.captureKirchhoffToolReaction(joint, normal), fn, true);
    ownership.recordKirchhoffToolReaction(tool, 0, ownership.captureKirchhoffToolReaction(joint, surface.rows[0].gradients), ft);
    contact.tangentLambda[0] = ft;
    return { wire, catheter, joint, tool, point, force: new Vector3(0, -fn, ft) };
}

for (const [angle, translate] of [[0, false], [Math.PI / 2, false], [0, true], [Math.PI / 2, true]]) test(`release preserves the applied world wrench after wire spin ${angle} rad, displaced arms ${translate}`, t => {
    const f = loadedPair();
    setFrame(f.wire, new Quaternion().setFromAxisAngle(new Vector3(0, 0, 1), angle));
    if (translate) {
        setFrame(f.catheter, new Quaternion().setFromAxisAngle(new Vector3(1, 1, 0).normalize(), -.7));
        for (const [side, body] of [f.wire, f.catheter].entries()) for (let node = 0; node < 2; node++) {
            body.x[node] += (side ? -.23 : .37) * (node + 1);
            body.y[node] += (side ? .17 : -.29) * (node + 1);
            body.z[node] += (side ? .41 : -.13) * (node + 1);
        }
    }
    const rows = [];
    ownership.appendKirchhoffToolRelease(f.joint, f.tool, 0, rows, 'proof-moved-frame');
    assert.ok(rows.length > 0, 'proof must release a real retained load');
    assert.ok(rows.every(row => row.gradients.every(g => g.dof % 6 < 3 || Math.floor(g.dof / 6) < 1)),
        'Transported terminal-node couples must use a real segment frame');
    // Expected release is the opposite pair of POINT FORCES at P, obtained
    // without reusing or summing the implementation's stored gradient history.
    for (const scale of [1, 0.25]) {
        const actual = wrench(rows, [f.wire, f.catheter], scale);
        const expectedForces = [f.force.clone().multiplyScalar(-scale), f.force.clone().multiplyScalar(scale)];
        const totalMoment = actual[0].moment.clone().add(actual[1].moment);
        t.diagnostic(JSON.stringify({ angle, translate, scale, netMoment: totalMoment.toArray() }));
        for (let side = 0; side < 2; side++) {
            const expectedMoment = f.point.clone().cross(expectedForces[side]);
            assert.ok(actual[side].force.distanceTo(expectedForces[side]) < 1e-11, `side ${side} release force`);
            assert.ok(actual[side].moment.distanceTo(expectedMoment) < 1e-11,
                `side ${side} world release moment ${actual[side].moment.toArray()} != ${expectedMoment.toArray()}`);
        }
        assert.ok(totalMoment.length() < 1e-11, `internal release created net moment ${totalMoment.toArray()}`);
    }
});

function returningLoop(branchHeight, loopZ) {
    const points = [[0, branchHeight, 0], [5, branchHeight, 0], [15, branchHeight, 0],
        [9.99, -1, loopZ], [10.01, 1, loopZ], [15, 1, loopZ]];
    const wire = { count: points.length, segmentCount: points.length - 1, activeStart: 0, activeEnd: points.length - 1,
        nodeRadius: new Float64Array(points.length).fill(0.4445),
        restLength: Float64Array.from(points.slice(1), (p, i) => vec(p).distanceTo(vec(points[i]))) };
    for (const [i, key] of ['x', 'y', 'z'].entries()) wire[key] = Float64Array.from(points, p => p[i]);
    const catheter = { activeStart: 0, activeEnd: 2, segmentCount: 2,
        x: [0, 5, 10], y: [0, 0, 0], z: [0, 0, 0] };
    const joint = { enabled: true, enforceDistalPortal: true, openDistal: true,
        innerBody: wire, outerBody: catheter, innerRadius: 0.485,
        containedLength: 10, innerArcOffset: 0, startNode: 0, endNode: 1, searchWindow: 2 };
    return { joint, tool: { bodyA: wire, bodyB: catheter, openDistalB: true } };
}

test('a certified material branch stays the portal even when a returning loop is closer', () => {
    const { joint } = returningLoop(0.04, 0.02);
    assert.equal(ownership.locateKirchhoffDistalLumenBranch(joint).segment, 1);
    assert.equal(ownership.evaluateKirchhoffOwnedSlidingPortal(joint).segment, 1);
});

test('failed branch certification cannot turn a returning loop into an active lumen portal', t => {
    const { joint, tool } = returningLoop(0.6, 0.1);
    // Independent geometry: the material anchor is (10,.6,0), outside the
    // .485 bore. The unrelated loop crosses the tip plane at (10,0,.1).
    assert.ok(0.6 > joint.innerRadius);
    assert.equal(ownership.locateKirchhoffDistalLumenBranch(joint).segment, -1);
    assert.equal(ownership.isKirchhoffDistalLumenWitness(joint, tool, 3, 1, 0.5, 10, 0, 0.1), false);
    const portal = ownership.evaluateKirchhoffOwnedSlidingPortal(joint);
    t.diagnostic(JSON.stringify({ segment: portal.segment, distance: portal.distance, violation: portal.violation }));
    // Allow either disabling an uncertified portal or recovering the actual
    // material branch; do not prescribe how ownership is implemented.
    assert.ok(portal.segment < 0 || portal.segment === 1,
        `uncertified return segment ${portal.segment} became a portal with ${portal.violation} mm violation`);
});
