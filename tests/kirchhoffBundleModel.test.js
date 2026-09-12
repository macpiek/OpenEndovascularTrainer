import assert from 'node:assert/strict';
import test from 'node:test';
import {
    DEFAULT_BUNDLE_RADIAL_CLEARANCE_MM,
    bundlePairMassMatrix, condenseBundleSection, decodeBundlePair, encodeBundlePair,
    evaluateBundleClearance, evaluateBundleSection, evaluateFullBundleSection,
    pullbackBundlePairForces
} from '../src/physics/kirchhoffBundleModel.js';
import { kirchhoffMaterialProfile } from '../src/physics/kirchhoffMaterialProfile.js';

const close = (actual, expected, tolerance = 2e-9) => assert.ok(
    Math.abs(actual - expected) <= tolerance * Math.max(1, Math.abs(expected)),
    `${actual} != ${expected}`
);
const closeVector = (a, b, tol) => a.forEach((v, i) => close(v, b[i], tol));
const dot = (a, b) => a.reduce((s, v, i) => s + v * b[i], 0);
const quadratic = (x, a) => a.reduce((s, row, i) => s + x[i] * dot(row, x), 0);
const rotate = (v, angle) => [
    Math.cos(angle) * v[0] - Math.sin(angle) * v[1],
    Math.sin(angle) * v[0] + Math.cos(angle) * v[1]
];
const material = (ei = 120, gj = 70, kappa0 = [0, 0]) => ({ EI1: ei, EI2: ei, GJ: gj, kappa0 });
const difference = (fn, x, h = 1e-6) => (fn(x + h) - fn(x - h)) / (2 * h);

test('isotropic common bending has summed EI, weighted natural curvature and mismatch energy', () => {
    const a = [0.12, -0.07];
    const b = [-0.03, 0.08];
    const tools = [{ material: material(50, 12, a) }, { material: material(200, 80, b) }];
    const result = condenseBundleSection({ tools });
    const expected = a.map((v, i) => (50 * v + 200 * b[i]) / 250);
    closeVector(result.preferredCurvature, expected);
    assert.deepEqual(result.bendingStiffness, [[250, 0], [0, 250]]);
    const delta = a.map((v, i) => v - b[i]);
    close(result.energyOffset, 0.5 * (50 * 200 / 250) * dot(delta, delta));
    closeVector(result.minimum.bendingMoment, [0, 0]);
    const k = [0.3, -0.2];
    const energy = evaluateBundleSection({ curvature: k, tools }).energy;
    close(energy, result.energyOffset + 125 * dot(k.map((v, i) => v - expected[i]), k.map((v, i) => v - expected[i])));
});

test('condensed mismatch retains physical torque under frictionless relative rotation', () => {
    const ei = 100;
    const k0 = 0.15;
    const angle = 1.1;
    const tools = [0, angle].map(theta => ({ theta, material: material(ei, 30, [k0, 0]) }));
    const result = condenseBundleSection({ tools });
    close(result.energyOffset, ei * k0 * k0 * (1 - Math.cos(angle)) / 2);
    const expectedTorque = ei * k0 * k0 * Math.sin(angle) / 2;
    close(result.minimum.tools[1].dTheta, expectedTorque);
    close(result.minimum.tools[0].dTheta, -expectedTorque);
    close(difference(theta => condenseBundleSection({ tools: [tools[0], { ...tools[1], theta }] }).energyOffset, angle), expectedTorque);
});

test('independent twist rates keep separate GJ and free isotropic straight rotations/translations', () => {
    const tools = [
        { s: 20, theta: 0.8, thetaPrime: 0.2, material: material(100, 20) },
        { s: -40, theta: -3, thetaPrime: -0.3, material: material(300, 60) }
    ];
    const result = evaluateBundleSection({ tools });
    close(result.energy, 0.5 * 20 * 0.2 ** 2 + 0.5 * 60 * 0.3 ** 2);
    closeVector(result.tools.map(v => v.dThetaPrime), [4, -18]);
    assert.deepEqual(result.tools.map(v => v.dTheta), [0, 0]);
    assert.deepEqual(result.tools.map(v => v.dS), [0, 0]);
    close(evaluateBundleSection({ tools: tools.map(v => ({ ...v, s: v.s + 300, theta: v.theta + 7 })) }).energy, result.energy);
});

test('anisotropic EI, including off-diagonal entries, rotates into the common frame', () => {
    const result = condenseBundleSection({ tools: [
        { theta: Math.PI / 2, material: { EI: [[9, 2], [2, 4]], GJ: 3, kappa0: [0.2, -0.1] } },
        { theta: 0, material: { EI: [[1, 0], [0, 2]], GJ: 5 } }
    ] });
    closeVector(result.bendingStiffness[0], [5, -2]);
    closeVector(result.bendingStiffness[1], [-2, 11]);
    const intrinsicRotated = [0.1, 0.2];
    const rhs = [4 * intrinsicRotated[0] - 2 * intrinsicRotated[1], -2 * intrinsicRotated[0] + 9 * intrinsicRotated[1]];
    closeVector(result.bendingStiffness.map(row => dot(row, result.preferredCurvature)), rhs);
});

const coupledMaterial = s => ({
    stiffness: [[15 + s, 2, -1], [2, 10 + s / 2, 1.5], [-1, 1.5, 7 + s / 4]],
    intrinsic: [0.12 + 0.01 * s, -0.08 + 0.02 * s, 0.03 - 0.015 * s]
});
const coupledDerivative = {
    stiffness: [[1, 0, 0], [0, 0.5, 0], [0, 0, 0.25]], intrinsic: [0.01, 0.02, -0.015]
};
const coupledInput = () => ({
    curvature: [0.21, -0.17], frameTwist: 0.06,
    tools: [
        { id: 'wire', s: 2.4, dsDx: 1.3, theta: 0.7, thetaPrime: -0.09, material: coupledMaterial, materialDerivative: coupledDerivative },
        { id: 'catheter', s: 1.8, dsDx: 0.8, theta: -0.4, thetaPrime: 0.04, material: coupledMaterial, materialDerivative: coupledDerivative }
    ]
});

test('common-axis energy equals independent full material energies, including bend–twist coupling', () => {
    const input = coupledInput();
    const result = evaluateBundleSection(input);
    const tools = input.tools.map(tool => {
        const bending = rotate(input.curvature, -tool.theta);
        return { ...tool, strain: [...bending, input.frameTwist + tool.thetaPrime].map(v => v / tool.dsDx) };
    });
    const full = evaluateFullBundleSection({ tools });
    close(result.energy, full.energy);
    let independentEnergy = 0;
    for (const tool of tools) {
        const m = coupledMaterial(tool.s);
        const delta = tool.strain.map((v, i) => v - m.intrinsic[i]);
        independentEnergy += tool.dsDx * 0.5 * quadratic(delta, m.stiffness);
    }
    close(result.energy, independentEnergy);
});

test('all reduced constitutive gradients agree with finite differences of energy', () => {
    const input = coupledInput();
    const result = evaluateBundleSection(input);
    input.curvature.forEach((value, i) => {
        close(result.bendingMoment[i], difference(x => {
            const curvature = [...input.curvature]; curvature[i] = x;
            return evaluateBundleSection({ ...input, curvature }).energy;
        }, value), 2e-7);
    });
    close(result.dFrameTwist, difference(frameTwist => evaluateBundleSection({ ...input, frameTwist }).energy, input.frameTwist), 2e-7);
    for (let i = 0; i < input.tools.length; i++) {
        for (const [key, output] of [['theta', 'dTheta'], ['thetaPrime', 'dThetaPrime'], ['s', 'dS'], ['dsDx', 'dDsDx']]) {
            close(result.tools[i][output], difference(x => {
                const tools = input.tools.map((tool, index) => index === i ? { ...tool, [key]: x } : tool);
                return evaluateBundleSection({ ...input, tools }).energy;
            }, input.tools[i][key]), 2e-7);
        }
    }
});

test('condensed energy reconstruction and envelope derivatives hold with a full coupled tensor', () => {
    const input = coupledInput();
    const condensed = condenseBundleSection(input);
    const delta = input.curvature.map((v, i) => v - condensed.preferredCurvature[i]);
    close(evaluateBundleSection(input).energy, condensed.energyOffset + 0.5 * quadratic(delta, condensed.bendingStiffness));
    closeVector(condensed.minimum.bendingMoment, [0, 0]);
    for (let i = 0; i < input.tools.length; i++) {
        for (const [key, output] of [['theta', 'dTheta'], ['thetaPrime', 'dThetaPrime'], ['s', 'dS'], ['dsDx', 'dDsDx']]) {
            close(condensed.minimum.tools[i][output], difference(x => condenseBundleSection({
                ...input, tools: input.tools.map((tool, index) => index === i ? { ...tool, [key]: x } : tool)
            }).energyOffset, input.tools[i][key]), 2e-7);
        }
    }
});

test('arbitrary rotation and spin of the reference normal basis leave physical energy unchanged', () => {
    const input = coupledInput();
    const angle = 1.2;
    const spin = -0.17;
    const transformed = {
        curvature: rotate(input.curvature, -angle), frameTwist: input.frameTwist + spin,
        tools: input.tools.map(tool => ({ ...tool, theta: tool.theta - angle, thetaPrime: tool.thetaPrime - spin }))
    };
    close(evaluateBundleSection(input).energy, evaluateBundleSection(transformed).energy);
    close(condenseBundleSection(input).energyOffset, condenseBundleSection(transformed).energyOffset);
});

test('full mode has independent strains and correct energy gradients for noncoaxial rods', () => {
    const tools = coupledInput().tools.map((tool, i) => ({ ...tool, strain: [0.2 + i, -0.3 * i, 0.12 - i] }));
    const result = evaluateFullBundleSection({ tools });
    for (let i = 0; i < tools.length; i++) {
        for (let j = 0; j < 3; j++) {
            close(result.tools[i].dStrain[j], difference(value => evaluateFullBundleSection({
                tools: tools.map((tool, index) => index !== i ? tool
                    : { ...tool, strain: tool.strain.map((v, k) => k === j ? value : v) })
            }).energy, tools[i].strain[j]), 2e-7);
        }
        close(result.tools[i].dS, difference(s => evaluateFullBundleSection({
            tools: tools.map((tool, index) => index === i ? { ...tool, s } : tool)
        }).energy, tools[i].s), 2e-7);
    }
});

test('common/relative coordinates retain both axes, independent frames and arbitrary weights', () => {
    const pair = {
        first: { position: [1, 2, 3], frame: [[1, 0, 0], [0, 1, 0], [0, 0, 1]], s: 2, theta: -0.4 },
        second: { position: [1.02, 2.01, 3.3], frame: [[0, 0, 1], [0, 1, 0], [-1, 0, 0]], s: 7, theta: 1.3 }
    };
    for (const weight of [0, 0.21, 0.5, 1]) {
        const encoded = encodeBundlePair({ ...pair, weight });
        const decoded = decodeBundlePair(encoded);
        closeVector(decoded.first.position, pair.first.position);
        closeVector(decoded.second.position, pair.second.position);
        for (const id of ['first', 'second']) {
            assert.deepEqual(decoded[id].frame, pair[id].frame);
            assert.equal(decoded[id].theta, pair[id].theta);
            assert.equal(decoded[id].s, pair[id].s);
        }
        assert.ok(Math.hypot(...encoded.relativePosition) > 0.3, 'axial part is not projected out');
    }
});

test('force pullback and unequal-mass kinetic energy preserve virtual work exactly', () => {
    const weight = 0.31;
    const firstForce = [2, -7, 1];
    const secondForce = [-5, 3, 4];
    const qDot = [0.3, 0.2, -0.1];
    const dDot = [-0.2, 0.5, 0.7];
    const v1 = qDot.map((v, i) => v - weight * dDot[i]);
    const v2 = qDot.map((v, i) => v + (1 - weight) * dDot[i]);
    const transformed = pullbackBundlePairForces({ firstForce, secondForce, weight });
    close(dot(firstForce, v1) + dot(secondForce, v2), dot(transformed.commonForce, qDot) + dot(transformed.relativeForce, dDot));
    const m1 = 2;
    const m2 = 7;
    const mass = bundlePairMassMatrix({ firstMass: m1, secondMass: m2, weight });
    assert.notEqual(mass[0][1], 0);
    const kinetic = qDot.reduce((sum, v, i) => sum + 0.5 * quadratic([v, dDot[i]], mass), 0);
    close(kinetic, 0.5 * m1 * dot(v1, v1) + 0.5 * m2 * dot(v2, v2));
    close(bundlePairMassMatrix({ firstMass: m1, secondMass: m2, weight: m2 / (m1 + m2) })[0][1], 0);
});

test('real 0.0405 mm clearance remains a unilateral gap and never zeros relative motion', () => {
    close(DEFAULT_BUNDLE_RADIAL_CLEARANCE_MM, (0.97 - 0.889) / 2, 1e-15);
    const offset = [0.02, -0.01];
    const result = evaluateBundleClearance({ offset });
    assert.deepEqual(offset, [0.02, -0.01]);
    assert.ok(result.admissible && result.gap > 0);
    close(result.gap, 0.0405 - Math.hypot(...offset));
    assert.equal(evaluateBundleClearance({ offset: [0.0405, 0] }).gap, 0);
    assert.equal(evaluateBundleClearance({ offset: [0.041, 0] }).admissible, false);
    for (let i = 0; i < 2; i++) close(result.gradientSquaredGap[i], difference(value => evaluateBundleClearance({
        offset: offset.map((v, j) => j === i ? value : v)
    }).squaredGap, offset[i]));
    assert.deepEqual(evaluateBundleClearance({ offset: [0, 0] }).gradientSquaredGap, [-0, -0]);
});

test('uncovering a precurved tool restores its own stress-free curvature', () => {
    const wire = { theta: 0.4, material: material(10, 3, [0.2, 0]) };
    const catheter = { theta: 0, material: material(1000, 30) };
    const covered = condenseBundleSection({ tools: [wire, catheter] });
    const exposed = condenseBundleSection({ tools: [wire] });
    close(norm2(covered.preferredCurvature), 10 * 0.2 / 1010);
    closeVector(exposed.preferredCurvature, rotate([0.2, 0], 0.4));
    close(exposed.energyOffset, 0);
    assert.ok(covered.energyOffset > 0);
});
const norm2 = v => Math.hypot(...v);

test('existing sampled profiles are usable without inventing unavailable slide derivatives', () => {
    const profile = kirchhoffMaterialProfile('berenstein');
    const result = evaluateBundleSection({ tools: [{ s: 4, material: profile }], curvature: [0.05, 0] });
    assert.ok(Number.isFinite(result.energy));
    assert.equal(result.tools[0].dS, null);
    const sampled = profile.sample(4);
    close(result.energy, 0.5 * sampled.EI1 * (0.05 - sampled.kappa01) ** 2);
});

test('invalid or singular material data fails visibly instead of clamping stiffness or clearance', () => {
    for (const bad of [
        { material: { EI: [[1, 3], [0, 2]], GJ: 1 } },
        { material: { EI: [[1, 2], [2, 1]], GJ: 1 } },
        { material: material(0) }, { material: material(), dsDx: 0 },
        { material: material(), dsDx: -1 }, { material: material(), theta: Infinity }
    ]) assert.throws(() => evaluateBundleSection({ tools: [bad] }));
    assert.throws(() => evaluateBundleSection({ tools: [] }));
    assert.throws(() => evaluateBundleClearance({ offset: [0, 0], clearance: -1 }));
});
