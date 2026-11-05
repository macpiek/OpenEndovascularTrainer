import assert from 'node:assert/strict';
import {sampleFlowConcentration} from '../../src/contrast/flowConcentrationField.js';

// Fixed anatomical regions from the reported aortoiliac artefact, in mm.
// Concentration is measured before optical saturation or pixel rasterization.
const inLip = (x, y) => Math.abs(x) < 20 && y >= -302 && y <= -281;
const inProximalAorta = (x, y) => Math.abs(x + 4) < 8 && y >= -287 && y <= -280;

export function measureBifurcationSurface(positions, stencils, values) {
    const count = positions.length / 3, concentration = new Float64Array(count);
    for (let i = 0; i < count; i++) {
        const o = i * 4;
        concentration[i] = sampleFlowConcentration(values, stencils[o], stencils[o + 1], stencils[o + 2], stencils[o + 3]);
        assert.ok(Number.isFinite(concentration[i]) && concentration[i] >= 0, 'Invalid wall concentration');
    }
    let maxLipGradient = 0, lipEdges = 0, proximalArea = 0, proximalIntegral = 0, proximalMax = 0;
    const inside = (i, roi) => roi(positions[i * 3], positions[i * 3 + 1]);
    for (let i = 0; i < count; i += 3) {
        for (const [a, b] of [[i, i + 1], [i + 1, i + 2], [i + 2, i]]) {
            if (!inside(a, inLip) || !inside(b, inLip)) continue;
            const distance = Math.hypot(positions[a * 3] - positions[b * 3],
                positions[a * 3 + 1] - positions[b * 3 + 1], positions[a * 3 + 2] - positions[b * 3 + 2]);
            if (distance < 1e-5) continue;
            lipEdges++;
            maxLipGradient = Math.max(maxLipGradient, Math.abs(concentration[a] - concentration[b]) / distance);
        }
        for (let j = i; j < i + 3; j++) if (inside(j, inProximalAorta)) proximalMax = Math.max(proximalMax, concentration[j]);
        if (![i, i + 1, i + 2].every(j => inside(j, inProximalAorta))) continue;
        const a = positions.slice(i * 3, i * 3 + 3);
        const b = positions.slice(i * 3 + 3, i * 3 + 6).map((v, j) => v - a[j]);
        const c = positions.slice(i * 3 + 6, i * 3 + 9).map((v, j) => v - a[j]);
        const area = .5 * Math.hypot(b[1] * c[2] - b[2] * c[1], b[2] * c[0] - b[0] * c[2], b[0] * c[1] - b[1] * c[0]);
        proximalArea += area;
        proximalIntegral += area * (concentration[i] + concentration[i + 1] + concentration[i + 2]) / 3;
    }
    assert.ok(lipEdges > 100 && proximalArea > 1, 'Regression crop must contain the actual fork and proximal aortic wall');
    return {maxLipGradient, lipEdges, proximalMean: proximalIntegral / proximalArea, proximalMax, proximalArea};
}

export function assertBifurcationMixingRegression(fixture, {requireCausality = true} = {}) {
    const positions = fixture.geometry.data.attributes.position.array;
    const beforeStencil = fixture.geometry.data.attributes.flowStencil.array;
    const frames = fixture.cases.map(frame => ({phase: frame.phase, elapsed: frame.elapsed,
        before: measureBifurcationSurface(positions, beforeStencil, frame.states.before),
        after: measureBifurcationSurface(positions, fixture.afterStencil, frame.states.after)}));
    assert.equal(frames.length, 9, 'Exercise all recorded pulse/front phases, including the late dilute front');
    for (const frame of frames) {
        const label = `phase ${frame.phase}, ${frame.elapsed.toFixed(3)} s`;
        // Accepted GPU images peak at .675/mm; the rejected owner seams reached
        // 2.56/mm. This is an artefact regression limit, not a physiological
        // diffusion constant: a steep moving front is deliberately still allowed.
        assert.ok(frame.after.maxLipGradient < .8, `Mesh-scale concentration seam returned (${label}): ${frame.after.maxLipGradient}/mm`);
        // Area weighting prevents dense triangles from dominating this check.
        // Allow a narrow finite-lumen streak, but not an invented dark proximal
        // column when the recorded parent blood remains dilute.
        assert.ok(frame.after.proximalMean <= 3 * frame.before.proximalMean + .003,
            `Reconstruction creates a broad proximal column (${label}): ${frame.before.proximalMean} → ${frame.after.proximalMean}`);
    }
    for (const [phase, elapsed] of [[.25, 1.3], [.75, 73 / 60]]) {
        const frame = frames.find(f => f.phase === phase && Math.abs(f.elapsed - elapsed) < 1e-6);
        assert.ok(frame && frame.before.maxLipGradient > 1.5, 'The recorded counterexample must still exercise the original seam');
        assert.ok(frame.after.maxLipGradient < .4 * frame.before.maxLipGradient,
            `The actual severe seam is insufficiently reduced at phase ${phase}`);
    }
    const probe = fixture.causalityProbe;
    if (requireCausality) {
        assert.ok(probe && probe.sourceCells > 0 && probe.sourceAnchorMinimum > .9,
            'Regenerate the fixture to exercise a remote iliac bolus');
        assert.equal(probe.junctionConcentration, 0, 'The causality probe must not have reached the junction');
        assert.ok(probe.proximalMaximum <= 1e-6,
            `Remote anchor iodine prefilled clear proximal blood: ${probe.proximalMaximum}`);
    }
    return {frames, causalityProbe: probe};
}
