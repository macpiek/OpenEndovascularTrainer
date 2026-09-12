import assert from 'node:assert/strict';
import test from 'node:test';
import {
    assessBundleReduction, buildAdaptiveBundleMesh, partitionBundleCoverage
} from '../src/physics/kirchhoffBundleDiscretization.js';
import { evaluateBundleSection } from '../src/physics/kirchhoffBundleModel.js';

const close = (a, b, tolerance = 1e-10) => assert.ok(Math.abs(a - b) <= tolerance, `${a} != ${b}`);
const tools = [
    { id: 'wire', interval: [0, 10], materialCoordinate: { offset: 100, scale: 2 }, materialBreakpoints: [110] },
    { id: 'catheter', interval: [0, 6], materialCoordinate: { offset: 10, scale: 1 }, materialBreakpoints: [13] }
];
const meshTolerances = {
    energy: { absolute: 1e-8 }, fields: { position: { absolute: 1e-3 } }
};
const flatSample = x => ({ energyDensity: 1, fields: { position: [x, 0, 0] } });
const simpleTools = [{ id: 'wire', interval: [0, 1] }, { id: 'catheter', interval: [0, 1] }];

test('coverage partitions exact moving tips and material interfaces with independent material maps', () => {
    const result = partitionBundleCoverage({ interval: [0, 10], tools });
    assert.deepEqual(result.map(v => [v.start, v.end]), [[0, 3], [3, 5], [5, 6], [6, 10]]);
    assert.deepEqual(result.at(-1).toolIds, ['wire']);
    assert.deepEqual(result[0].materials, [
        { id: 'wire', sStart: 100, sEnd: 106, sMid: 103, dsDx: 2 },
        { id: 'catheter', sStart: 10, sEnd: 13, sMid: 11.5, dsDx: 1 }
    ]);
    const shifted = partitionBundleCoverage({ interval: [0, 10], tools: [tools[0], {
        ...tools[1], interval: [0, 8], materialCoordinate: { offset: 8 }
    }] });
    assert.deepEqual(shifted.map(v => [v.start, v.end]), [[0, 5], [5, 8], [8, 10]]);
    assert.equal(shifted[0].materials[1].sEnd, 13);
    assert.equal(shifted[0].materials[0].sEnd, 110);
});

test('coverage handles uncovered gaps, clipped domains and explicit contact boundaries', () => {
    const result = partitionBundleCoverage({
        interval: [-1, 9], tools: [{ id: 'wire', interval: [0, 4] }, { id: 'catheter', interval: [6, 20] }], boundaries: [2, 7]
    });
    assert.deepEqual(result.map(v => [v.start, v.end]), [[-1, 0], [0, 2], [2, 4], [4, 6], [6, 7], [7, 9]]);
    assert.deepEqual(result[0].toolIds, []);
    assert.deepEqual(result[3].toolIds, []);
});

test('adaptive element count follows shape error, and tighter tolerances refine the same curve', () => {
    const curved = x => ({ energyDensity: 1, fields: { position: [x, x * x, 0] } });
    const run = tol => buildAdaptiveBundleMesh({
        interval: [0, 1], tools: simpleTools, sample: curved,
        tolerances: { energy: { absolute: 1e-10 }, fields: { position: { absolute: tol } } }
    });
    const loose = run(0.02);
    const tight = run(0.001);
    assert.ok(loose.converged && tight.converged);
    assert.ok(tight.nodes.length > loose.nodes.length);
    for (const element of tight.elements) {
        // Exact maximum secant error of y=x² is h²/4, independently of probes.
        assert.ok((element.end - element.start) ** 2 / 4 <= 0.001);
    }
    const flat = buildAdaptiveBundleMesh({ interval: [0, 1], tools: simpleTools, sample: flatSample, tolerances: meshTolerances });
    assert.equal(flat.elements.length, 1);
    assert.ok(tight.elements.length > flat.elements.length);
    assert.equal(tight.certified, false, 'sampled convergence is not a mathematical bound');
});

test('embedded energy quadrature converges to the analytical integral and budgets error globally', () => {
    const result = buildAdaptiveBundleMesh({
        interval: [0, 1], tools: simpleTools,
        sample: x => ({ energyDensity: x ** 4, fields: { position: [x, 0, 0] } }),
        tolerances: { energy: { absolute: 1e-8 }, fields: { position: { absolute: 1e-5 } } }
    });
    assert.ok(result.converged);
    close(result.energy, 1 / 5, 1e-8);
    assert.ok(result.elements.reduce((sum, v) => sum + v.errors.energy, 0) <= 1e-8);
    for (const element of result.elements) {
        close(element.quadrature.reduce((sum, v) => sum + v.weight, 0), element.end - element.start);
        assert.equal(element.representation, 'full');
        assert.deepEqual(element.fullDofReasons, ['missing-reduction-evidence']);
    }
});

test('one-sided interface samples preserve a constitutive jump without fictitious transition energy', () => {
    const result = buildAdaptiveBundleMesh({
        interval: [0, 1], tools: [{ id: 'wire', interval: [0, 1], materialBreakpoints: [0.5] }],
        sample: (x, segment) => ({ energyDensity: segment.start < 0.5 ? 1 : 4, fields: { position: x } }),
        tolerances: meshTolerances
    });
    assert.deepEqual(result.nodes, [0, 0.5, 1]);
    close(result.energy, 2.5);
    assert.equal(result.elements[0].quadrature.at(-1).materials[0].s, 0.5);
    assert.equal(result.elements[1].quadrature[0].materials[0].s, 0.5);
});

test('moving coverage changes total bending energy by the exact newly covered length', () => {
    const energyForTip = tip => buildAdaptiveBundleMesh({
        interval: [0, 10], tools: [{ id: 'wire', interval: [0, 10] }, { id: 'catheter', interval: [0, tip] }],
        sample: (x, segment) => ({
            energyDensity: evaluateBundleSection({ curvature: [0.1, 0], tools: segment.tools.map(tool => ({
                id: tool.id, s: tool.offset + tool.scale * x,
                material: { EI1: tool.id === 'wire' ? 10 : 1000, GJ: 1 }
            })) }).energy,
            fields: { position: [x, 0, 0] }
        }), tolerances: meshTolerances
    });
    const a = energyForTip(6);
    const b = energyForTip(8);
    close(a.energy, 0.5 * (10 * 10 + 6 * 1000) * 0.1 ** 2);
    close(b.energy - a.energy, 0.5 * 2 * 1000 * 0.1 ** 2);
    assert.ok(a.nodes.includes(6) && b.nodes.includes(8));
});

test('element material and quadrature maps retain unequal dsDx at every refined point', () => {
    const result = buildAdaptiveBundleMesh({
        interval: [0, 10], tools, sample: x => ({ energyDensity: 1, fields: { position: [x * x] } }),
        tolerances: { energy: { absolute: 1e-9 }, fields: { position: { absolute: 0.1 } } }
    });
    for (const element of result.elements) {
        for (const point of element.quadrature) {
            close(point.materials[0].s, 100 + 2 * point.x);
            assert.equal(point.materials[0].dsDx, 2);
            if (point.materials[1]) close(point.materials[1].s, 10 + point.x);
        }
    }
});

test('resource limits report unresolved error and keep full DOFs', () => {
    for (const limits of [{ maxElements: 1 }, { maxDepth: 0 }, { minLength: 0.6 }]) {
        const result = buildAdaptiveBundleMesh({
            interval: [0, 1], tools: simpleTools,
            sample: x => ({ energyDensity: 1, fields: { position: [x * x] } }),
            tolerances: meshTolerances, ...limits
        });
        assert.equal(result.converged, false);
        assert.ok(result.unresolved.length > 0);
        assert.ok(result.unresolved[0].unresolvedReasons.length > 0);
        assert.equal(result.elements[0].representation, 'full');
        assert.deepEqual(result.elements[0].fullDofReasons, ['discretization-error']);
    }
});

test('a supplied curvature bound detects features that alias all initial probes', () => {
    const k = 8 * Math.PI;
    const tol = 0.01;
    const base = {
        interval: [0, 1], tools: simpleTools,
        sample: x => ({ energyDensity: 1, fields: { position: [Math.sin(k * x)] } }),
        tolerances: { energy: { absolute: 1e-9 }, fields: { position: { absolute: tol } } }
    };
    const sampled = buildAdaptiveBundleMesh(base);
    assert.equal(sampled.elements.length, 1);
    assert.equal(sampled.certified, false);
    const bounded = buildAdaptiveBundleMesh({ ...base,
        // Linear interpolation remainder <= h² max|f''|/8 everywhere.
        estimateInterval: ({ start, end }) => ({ normalizedError: (end - start) ** 2 * k * k / (8 * tol), certified: true })
    });
    assert.ok(bounded.converged && bounded.certified);
    assert.ok(bounded.elements.length > 1);
    for (const element of bounded.elements) {
        for (let j = 0; j <= 17; j++) {
            const t = j / 17;
            const x = element.start + t * (element.end - element.start);
            const interpolated = (1 - t) * Math.sin(k * element.start) + t * Math.sin(k * element.end);
            assert.ok(Math.abs(interpolated - Math.sin(k * x)) <= tol);
        }
    }
});

const reductionTolerances = Object.fromEntries(['energy', 'position', 'force', 'moment', 'twist', 'gap']
    .map(key => [key, { absolute: 1e-3 }]));
const state = () => ({
    stateKey: 'coverage-6/materials-4/geometry-21/load-2', probeIds: ['a', 'b'],
    energy: 0.4, positions: [[0, 0, 0], [1, 0, 0]],
    forces: [[0, 0, 0], [0, 0, 0]], moments: [[0, 0, 0], [0, 0, 0]],
    twists: [0, 0], gaps: [0.0405, 0.0405]
});
const evidence = () => {
    const full = state();
    return { stateKey: full.stateKey, full, reduced: state(), tolerances: structuredClone(reductionTolerances),
        certificate: { stateKey: full.stateKey, errorBounds: { energy: 0, position: 0, force: 0, moment: 0, twist: 0, gap: 0 } } };
};

test('reduction requires current full comparisons and explicit bounds, not a mesh size', () => {
    assert.equal(assessBundleReduction(evidence()).admitted, true);
    const missing = evidence(); delete missing.certificate;
    assert.ok(assessBundleReduction(missing).reasons.includes('missing-error-bounds'));
    const stale = evidence(); stale.certificate.stateKey = 'old-geometry';
    assert.ok(assessBundleReduction(stale).reasons.includes('stale-error-bounds'));
    const coverage = evidence(); coverage.reduced.stateKey = 'new-coverage';
    assert.ok(assessBundleReduction(coverage).reasons.includes('state-or-coverage-changed'));
    const bothStale = evidence(); bothStale.stateKey = 'new-state';
    assert.ok(assessBundleReduction(bothStale).reasons.includes('state-or-coverage-changed'));
    const probes = evidence(); probes.reduced.probeIds.reverse();
    assert.ok(assessBundleReduction(probes).reasons.includes('probe-mismatch'));
});

test('equal energy alone cannot justify eliminating transverse clearance or independent twist', () => {
    const trials = [
        ['position', 'positions', [0.01, 0, 0]], ['force', 'forces', [0, 0.01, 0]],
        ['moment', 'moments', [0, 0, 0.01]], ['twist', 'twists', 0.01], ['gap', 'gaps', 0.02]
    ];
    for (const [metric, field, value] of trials) {
        const data = evidence(); data.reduced[field][0] = value;
        const result = assessBundleReduction(data);
        assert.equal(result.admitted, false);
        assert.ok(result.reasons.includes(`${metric}-error`));
    }
    const data = evidence(); data.reduced.energy += 0.01;
    assert.ok(assessBundleReduction(data).reasons.includes('energy-error'));
});

test('eliminated-mode bounds add to observed errors and cannot override a failed comparison', () => {
    const data = evidence();
    data.reduced.positions[0][0] = 0.0007;
    data.certificate.errorBounds.position = 0.0007;
    const result = assessBundleReduction(data);
    assert.ok(result.reasons.includes('position-error'));
    close(result.errors.position, 0.0014);
    const incomplete = evidence(); delete incomplete.certificate.errorBounds.moment;
    assert.ok(assessBundleReduction(incomplete).reasons.includes('missing-moment-bound'));
    const missing = evidence(); delete missing.full.forces;
    assert.ok(assessBundleReduction(missing).reasons.includes('missing-force-evidence'));
});

test('active or uncertain contact forces full DOFs despite small residuals and equal energies', () => {
    const active = evidence(); active.full.gaps[0] = active.reduced.gaps[0] = 0;
    assert.ok(assessBundleReduction(active).reasons.includes('active-or-uncertain-contact'));
    const uncertain = evidence(); uncertain.full.gaps[0] = uncertain.reduced.gaps[0] = 0.0004;
    uncertain.certificate.errorBounds.gap = 0.0005;
    assert.ok(assessBundleReduction(uncertain).reasons.includes('active-or-uncertain-contact'));
    const margin = evidence(); margin.contactMargin = 0.041;
    assert.ok(assessBundleReduction(margin).reasons.includes('active-or-uncertain-contact'));
});

test('mesh exposes explicit activation reasons and only enables reduction with passing evidence', () => {
    const result = buildAdaptiveBundleMesh({
        interval: [0, 1], tools: simpleTools, boundaries: [0.5], sample: flatSample, tolerances: meshTolerances,
        stateKey: state().stateKey,
        reductionEvidence: ({ start }) => {
            const data = evidence();
            if (start >= 0.5) data.reduced.moments[1][2] = 0.5;
            return data;
        }
    });
    assert.equal(result.elements[0].representation, 'common-axis');
    assert.deepEqual(result.elements[0].fullDofReasons, []);
    assert.equal(result.elements[1].representation, 'full');
    assert.ok(result.elements[1].fullDofReasons.includes('moment-error'));
});

test('missing tolerances and invalid material maps fail visibly', () => {
    assert.throws(() => partitionBundleCoverage({ interval: [0, 1], tools: [{ id: 'w', interval: [0, 1], materialCoordinate: { scale: 0 } }] }));
    assert.throws(() => partitionBundleCoverage({ interval: [0, 1], tools: [simpleTools[0], simpleTools[0]] }));
    assert.throws(() => buildAdaptiveBundleMesh({ interval: [0, 1], tools: simpleTools, sample: flatSample }));
    assert.throws(() => buildAdaptiveBundleMesh({ interval: [0, 1], tools: simpleTools, boundaries: [0.5], sample: flatSample, tolerances: meshTolerances, maxElements: 1 }));
    const noTolerance = evidence(); delete noTolerance.tolerances.force;
    assert.throws(() => assessBundleReduction(noTolerance));
});
