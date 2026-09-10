import assert from 'node:assert/strict';
import test from 'node:test';
import { createRequire } from 'node:module';
import { resolve } from 'node:path';
import { fileURLToPath, pathToFileURL } from 'node:url';

const root = resolve(process.env.OET_SPLIT_MOTION_SOURCE_ROOT ?? fileURLToPath(new URL('../', import.meta.url)));
const url = path => pathToFileURL(resolve(root, path));
const { EndovascularPhysicsWorld } = await import(url('src/physics/endovascularPhysicsWorld.js'));
const { solveKirchhoffCoupledSystem: solve, applyKirchhoffCoupledCorrection: apply } = await import(url('src/physics/kirchhoffCoupledSystem.js'));
const { Quaternion, Vector3 } = createRequire(url('package.json'))('three');
const DT = 1 / 120, R = .5, MU = .3, POSITION = 2e-6, VELOCITY = 5e-4, ANGULAR = 2e-5, LAMBDA = 1e-10;
const LENGTH = .5;
const axes = ['X', 'Y', 'Z'], xyz = ['x', 'y', 'z'];
const near = (a, b, tolerance, message) => assert.ok(Number.isFinite(a) && Math.abs(a - b) <= tolerance,
    `${message}: ${a} vs ${b}, tolerance ${tolerance}`);
const qAt = (b, i) => new Quaternion(...['X', 'Y', 'Z', 'W'].map(a => b['orientation' + a][i])).normalize();
const pAt = (b, i) => new Vector3(...xyz.map(a => b[a][i]));

class AffineWall {
    voxelSize = .5;
    write(p, radius, out) {
        const gap = -p.y - radius, penetration = Math.max(0, -gap);
        Object.assign(out, { signedDistance: -p.y, signedGap: gap, penetration, inside: p.y <= 0,
            violation: gap < 0, branchId: 0, faceIndex: 0, source: 'independent-wall-friction', timeOfImpact: gap < 0 ? 0 : 1 });
        Object.assign(out.point, p); Object.assign(out.closestPoint, { x: p.x, y: 0, z: p.z });
        Object.assign(out.normal, { x: 0, y: -1, z: 0 }); Object.assign(out.inward, out.normal);
        Object.assign(out.target, { x: p.x, y: p.y - penetration, z: p.z }); return out;
    }
    querySphere(p, r, out) { return this.write(p, r, out); }
    queryCapsule(a, b, r, out) { const p = a.y >= b.y ? a : b; this.write(p, r, out); out.segmentT = p === a ? 0 : 1; return out; }
    sweepSphere(a, b, r, out) { this.write(b, r, out); const ga = -a.y - r, gb = -b.y - r;
        out.timeOfImpact = gb >= 0 ? 1 : ga <= 0 ? 0 : ga / (ga - gb); return out; }
}

function fixture(y) {
    const records = [], phasePath = []; let currentRows, currentGroups, last;
    const world = new EndovascularPhysicsWorld({ fixedDt: DT, jointMotionMode: 'split-physical-bias', contactField: new AffineWall(),
        coupledSystem: {
            solve(c, dt, options) {
                // Preserve witness identity/position, not production gradients.
                // The affine wall fixes normal -Y and tangent axes +X,+Z.
                currentRows = options.additionalRows.map(r => {
                    const body = r.side ? c.outerBody : c.innerBody;
                    if (!['wall', 'split-point-wall'].includes(r.kind)) return { kind: r.kind };
                    const segment = Math.min(r.node, body.activeEnd - 1);
                    const t = r.kind === 'wall' ? body.wallT[r.node] : Number(r.node === body.activeEnd);
                    const center = pAt(body, segment).lerp(pAt(body, segment + 1), t);
                    return { kind: r.kind, side: r.side, center, point: center.clone().add(new Vector3(0, body.nodeRadius[r.node], 0)) };
                });
                currentGroups = options.groups.filter(g => g.rowIndices?.every(i => options.additionalRows[i]?.kind === 'split-wall-friction'))
                    .map(g => ({ rows: [...g.rowIndices], normal: options.additionalRows.indexOf(g.normalRow) }));
                return solve(c, dt, { ...options, activeCondensation: true, simultaneousCoulomb: true });
            },
            apply(c, result) {
                const force = new Vector3(), moment = new Vector3(), radialMoment = new Vector3();
                const tangentForce = new Vector3(), normalForce = new Vector3();
                const scale = result.scale / DT;
                const add = (point, f) => { force.add(f); moment.add(point.clone().cross(f)); };
                currentRows.forEach((r, i) => {
                    if (r.point) { const f = new Vector3(0, -scale * result.additionalIncrement[i], 0); add(r.point, f); normalForce.add(f); }
                });
                for (const g of currentGroups) {
                    const witness = currentRows[g.normal];
                    assert.ok(witness?.point, 'each actual wall-friction group names its physical normal witness');
                    const f = new Vector3(scale * result.additionalIncrement[g.rows[0]], 0, scale * result.additionalIncrement[g.rows[1]]);
                    add(witness.point, f); tangentForce.add(f);
                    radialMoment.add(witness.point.clone().sub(witness.center).cross(f));
                }
                // Independently recover the complete generalized impulse from
                // the actual full response and mobility. Internal material
                // forces/torques cancel; only the wall supplies external wrench.
                const actualForce = new Vector3(), actualMoment = new Vector3();
                for (const [body, response] of [[c.innerBody, result.inner], [c.outerBody, result.outer]]) {
                    for (let i = body.activeStart; i <= body.activeEnd; i++) {
                        if (body.inverseMass[i] > 0) {
                            const f = new Vector3(...[0, 1, 2].map(a => scale * response.correction[i * 6 + a] / body.inverseMass[i]));
                            actualForce.add(f); actualMoment.add(pAt(body, i).cross(f));
                        }
                        if (i < body.activeEnd && body.inverseInertia1[i] > 0) {
                            const m = new Vector3(...[0, 1, 2].map(a => scale * response.correction[i * 6 + 3 + a] / body['inverseInertia' + (a + 1)][i]));
                            actualMoment.add(m.applyQuaternion(qAt(body, i)));
                        }
                    }
                }
                last = { force, moment, radialMoment, tangentForce, normalForce, actualForce, actualMoment,
                    wireEdge: pAt(c.innerBody, 1).sub(pAt(c.innerBody, 0)).toArray(),
                    frictionGroups: currentGroups.length, rowKinds: [...new Set(currentRows.map(r => r.kind))] };
                records.push(last); apply(c, result);
            }
        } });
    const profile = { radius: R, mass: 1, inverseAngularInertia: 1, adaptationCompliance: 0,
        kirchhoffBendCompliance: 0, kirchhoffTwistCompliance: 0, foldLimitStrength: 0,
        linearDamping: 1, angularDamping: 1, projectionVelocityRetention: 0,
        wallProjectionVelocityRetention: 0, toolProjectionVelocityRetention: 0, wallCompliance: 0,
        wallStaticFriction: MU, wallKineticFriction: MU, sleepFrames: 1e6, sleepVelocity: 0, sleepAngularVelocity: 0 };
    const wire = world.createRod('wall-wire', 2, LENGTH, profile), catheter = world.createRod('supported-catheter', 3, 1, profile);
    // Both x coordinates stay in the same Float32 exponent interval [1,2).
    // Uniform prediction then preserves the exact .5-mm edge. The earlier
    // [-.5,.5] fixture acquired 2.98e-8 mm of artificial material strain when
    // adding vx*dt; its unconverged material torque polluted the wall oracle.
    for (let i = 0; i < wire.count; i++) wire.setNodePosition(i, 1.25 + i * LENGTH, y, 0);
    for (let i = 0; i < catheter.count; i++) catheter.setNodePosition(i, i - 1, -4, 0);
    catheter.inverseMass.fill(0); for (const a of [1, 2, 3]) catheter['inverseInertia' + a].fill(0);
    world.addContainment(wire, catheter, { innerRadius: 100, openDistal: false, openProximal: false,
        axialFriction: 0, torsionalFriction: 0, radialVelocityDamping: 0, coupledBendingRateDamping: 0 });
    wire.copyCurrentToPrevious(); catheter.copyCurrentToPrevious();
    world.debugJointTrial = (c, state) => { last.phase = state.motionPhase; phasePath.push(state.motionPhase); };
    return { world, wire, catheter, records, phasePath };
}

function sample(f, t, name) {
    const d = f.world.getStats().jointMotion;
    const contacts = d?.contacts.filter(c => c.kind === 'wall') ?? [];
    t.diagnostic(JSON.stringify({ name, mode: d?.mode, physicalPasses: d?.physicalPasses, biasPasses: d?.biasPasses,
        historyCommits: d?.historyCommits, certified: d?.certified, limitations: d?.limitations,
        physicalKKTResidualMm: d?.physicalKKTResidualMm, physicalConeViolation: d?.physicalConeViolation,
        sweptWitnesses: d?.sweptWitnesses, phases: f.phasePath,
        wallFrictionGroups: f.records.filter(r => r.phase === 'physical').map(r => r.frictionGroups),
        rawGaps: Array.from(f.wire.y, y => -y - R), velocity: axes.map(a => [...f.wire['velocity' + a]]),
        omega: axes.map(a => [...f.wire['angularVelocity' + a]]), contacts }));
    assert.equal(d?.mode, 'split-physical-bias'); assert.equal(f.world.getStats().coupledSolver, 'joint');
    assert.ok(d.physicalPasses > 0 && d.biasPasses > 0); assert.ok(f.phasePath.includes('physical'));
    assert.ok(f.phasePath.includes('bias') || d.biasInitialStateSettled === true,
        'bias must either solve a correction or certify all fresh initial residuals');
    assert.ok(f.records.some(r => r.phase === 'physical' && r.frictionGroups > 0), 'real SplitWallFriction rows/groups must reach the native solver');
    assert.equal(d.sweptWitnesses, 0, 'public reaction sum is complete only without unsampled swept witnesses');
    assert.equal(d.historyCommits, 1); near(d.physicalDt, DT, 1e-15, 'physical dt');
    assert.equal(d.reactionUnits, 'xpbd-multiplier'); near(d.impulseScale, 1 / DT, 1e-12, 'impulse conversion');
    assert.equal(d.certified, true); assert.equal(f.world.getStats().coupledClosureConverged, true);
    assert.ok(d.physicalKKTResidualMm <= f.world.coupledContainmentTolerance && d.physicalConeViolation <= 1e-9);
    for (const c of contacts) {
        assert.ok(c.id.startsWith('wall:0:') || c.id.startsWith('wall-point:0:'), 'only the wire touches the wall');
        assert.deepEqual(c.mu, [MU, MU]);
        assert.ok(c.normalPhysical >= -LAMBDA && c.normalBias >= -LAMBDA);
        assert.ok(Math.hypot(...c.tangentPhysical) <= MU * c.normalPhysical + LAMBDA, 'physical wall cone');
    }
    for (const y of f.wire.y) assert.ok(-y - R >= -POSITION, 'raw affine wall gap');
    near(pAt(f.wire, 1).distanceTo(pAt(f.wire, 0)), LENGTH, POSITION, 'exact material length');
    return { d, contacts, normal: contacts.reduce((s, c) => s + c.normalPhysical, 0),
        bias: contacts.reduce((s, c) => s + c.normalBias, 0),
        tangent: contacts.reduce((s, c) => s + Math.hypot(...c.tangentPhysical), 0) };
}

function setMotion(f, velocity, omega = [0, 0, 0]) {
    axes.forEach((a, i) => { f.wire['velocity' + a].fill(velocity[i]); f.wire['angularVelocity' + a].fill(omega[i]); });
}
function unchangedMotion(f, velocity) {
    axes.forEach((a, i) => { for (const v of f.wire['velocity' + a]) near(v, velocity[i], VELOCITY, 'preserved physical v' + a);
        for (const w of f.wire['angularVelocity' + a]) near(w, 0, ANGULAR, 'no spurious omega' + a); });
}

test('positive physical wall load and axial slip balance impulse and the surface radius moment', t => {
    const f = fixture(-R); setMotion(f, [4, 1, 0]); f.world.stepFixed(); const s = sample(f, t, 'loaded');
    assert.ok(s.normal > 1e-4 && s.tangent > 1e-5, 'positive physical normal and tangential reactions');
    const px = f.wire.velocityX[0] + f.wire.velocityX[1], py = f.wire.velocityY[0] + f.wire.velocityY[1];
    near(s.normal / DT, 2 - py, .002, 'wall normal impulse equals lost wire momentum');
    near(s.tangent / DT, 8 - px, .002, 'axial wall friction impulse equals lost wire momentum');
    assert.ok(px < 8 - .01 && s.tangent <= MU * s.normal + LAMBDA);
    let radial = 0;
    t.diagnostic(JSON.stringify({ wrenchAudit: f.records.filter(r => r.phase === 'physical').map(r => ({
        edge: r.wireEdge, fullForce: r.actualForce.toArray(), expectedForce: r.force.toArray(),
        fullMoment: r.actualMoment.toArray(), expectedMoment: r.moment.toArray(), radialMoment: r.radialMoment.toArray() })) }));
    for (const r of f.records.filter(r => r.phase === 'physical')) {
        for (const a of xyz) {
            near(r.actualForce[a], r.force[a], 1e-9, 'full physical response / independent wall force ' + a);
            near(r.actualMoment[a], r.moment[a], 1e-9, 'full physical response / independent surface moment ' + a);
        }
        near(r.radialMoment.z, -R * r.tangentForce.x, 1e-10, 'radial lever creates the required axial-friction torque');
        radial += Math.abs(r.radialMoment.z);
    }
    assert.ok(radial > .01, 'fixture must exercise a nonzero radius moment, not just centerline friction');
});

test('positive wall friction coefficient cannot turn bias-only repair into a physical friction budget', t => {
    const f = fixture(-.25); setMotion(f, [4, 0, 0]); f.world.stepFixed(); const s = sample(f, t, 'bias-only');
    assert.ok(s.bias > .01, 'actual positional repair'); near(s.normal, 0, LAMBDA, 'zero physical normal budget');
    near(s.tangent, 0, LAMBDA, 'zero physical friction budget'); unchangedMotion(f, [4, 0, 0]);
});

test('the next physical dt releases wall normal and friction reactions when pressure stops and the wire separates', t => {
    const f = fixture(-R); setMotion(f, [4, 1, 0]); f.world.stepFixed(); const loaded = sample(f, t, 'before-release');
    assert.ok(loaded.normal > 1e-4 && loaded.tangent > 1e-5, 'release starts from a genuine loaded friction contact');
    f.records.length = f.phasePath.length = 0;
    setMotion(f, [3, -2, 0]); f.wire.forceY.fill(0); f.world.stepFixed(); const released = sample(f, t, 'released');
    assert.equal(f.world.stepCount, 2); near(released.normal, 0, LAMBDA, 'released physical normal force');
    near(released.tangent, 0, LAMBDA, 'released physical friction'); unchangedMotion(f, [3, -2, 0]);
    assert.ok(Array.from(f.wire.y, y => -y - R).every(gap => gap > .01), 'actual motion away from the wall');
});
