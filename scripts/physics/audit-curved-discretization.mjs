import fs from 'node:fs';
import { gunzipSync } from 'node:zlib';
import { createHash } from 'node:crypto';
import { evaluateHermiteElement as hermite, boundHermiteGeometry } from './curved-element-geometry.mjs';

// Offline representation audit only. A good position fit does not certify
// coarse mechanics, collision coverage, forces, frame twist or stability.
const [input = 'tests/fixtures/kirchhoff-coupled-full-200.json.gz',
    output = 'reports/kirchhoff-curved-discretization-audit.json'] = process.argv.slice(2);
const stored = fs.readFileSync(input), bytes = input.endsWith('.gz') ? gunzipSync(stored) : stored;
const data = JSON.parse(bytes);
const norm = a => Math.hypot(...a), sub = (a, b) => a.map((x, i) => x - b[i]);
const lerp = (a, b, t) => a.map((x, i) => x * (1 - t) + b[i] * t);
const point = (body, i) => [body.x[i], body.y[i], body.z[i]];
function director(body, i) {
    let x = body.orientationX[i], y = body.orientationY[i], z = body.orientationZ[i], w = body.orientationW[i];
    const length = Math.hypot(x, y, z, w);
    if (!(length > 0)) throw new Error('Invalid material frame');
    x /= length; y /= length; z /= length; w /= length;
    return [2 * (x * z + w * y), 2 * (y * z - w * x), 1 - 2 * (x * x + y * y)];
}
function spherical(a, b, t) {
    const cosine = Math.max(-1, Math.min(1, a.reduce((s, v, i) => s + v * b[i], 0)));
    if (cosine < -1 + 1e-12) return null; // No uniquely determined shortest arc.
    if (cosine > 1 - 1e-12) {
        const d = lerp(a, b, t), length = norm(d);
        return d.map(v => v / length);
    }
    const angle = Math.acos(cosine), sine = Math.sin(angle);
    return a.map((v, i) => (Math.sin((1 - t) * angle) * v + Math.sin(t * angle) * b[i]) / sine);
}
function endpointDirector(body, node) {
    const first = body.activeStart, last = body.activeEnd - 1;
    const a = Math.max(first, Math.min(last - 1, node - 1)), b = a + 1;
    const s = body.materialCoordinate;
    const centerA = (s[a] + s[a + 1]) / 2, centerB = (s[b] + s[b + 1]) / 2;
    return spherical(director(body, a), director(body, b), (s[node] - centerA) / (centerB - centerA));
}
function statistics(values) {
    const a = values.slice().sort((x, y) => x - y);
    const q = p => { const i = p * (a.length - 1), j = Math.floor(i); return a[j] + (a[Math.ceil(i)] - a[j]) * (i - j); };
    return { minimum: a[0], median: q(.5), p95: q(.95), maximum: a.at(-1) };
}
const bodies = data.bodies.map((body, side) => {
    const samples = [], s = body.materialCoordinate;
    for (let i = body.activeStart + 1; i < body.activeEnd; i++) {
        const h0 = s[i] - s[i - 1], h1 = s[i + 1] - s[i], length = h0 + h1, t = h0 / length;
        if (!(h0 > 0 && h1 > 0)) throw new Error('Positive material intervals are required');
        const a = point(body, i - 1), middle = point(body, i), b = point(body, i + 1);
        const da = endpointDirector(body, i - 1), db = endpointDirector(body, i + 1);
        const row = { node: i, fromMaterial: s[i - 1], toMaterial: s[i + 1], lengthMm: length,
            linearMidpointErrorMm: norm(sub(lerp(a, b, t), middle)), tangentDefined: Boolean(da && db) };
        if (da && db) {
            row.curvedMidpointErrorMm = norm(sub(hermite(a, b, da, db, length, t), middle));
            row.curvedPolylineSampleErrorMm = 0; row.maximumSampleStretchError = 0;
            // Quarter/mid samples on EACH old edge, not only a fitted vertex.
            for (const u of [0, t / 4, t / 2, 3 * t / 4, t, t + (1 - t) / 4,
                t + (1 - t) / 2, t + 3 * (1 - t) / 4, 1]) {
                const fine = u <= t ? lerp(a, middle, u / t) : lerp(middle, b, (u - t) / (1 - t));
                row.curvedPolylineSampleErrorMm = Math.max(row.curvedPolylineSampleErrorMm,
                    norm(sub(hermite(a, b, da, db, length, u), fine)));
                row.maximumSampleStretchError = Math.max(row.maximumSampleStretchError,
                    Math.abs(norm(hermite(a, b, da, db, length, u, true)) - 1));
            }
            row.continuousGeometryBounds = boundHermiteGeometry({ a, middle, b, da, db, length, middleFraction: t });
        }
        samples.push(row);
    }
    const defined = samples.filter(row => row.tangentDefined);
    function disjointCandidates(toleranceMm) {
        let end = -Infinity, count = 0;
        for (const row of defined) {
            if (row.node - 1 >= end && row.continuousGeometryBounds.position.upper <= toleranceMm &&
                row.continuousGeometryBounds.stretch.upper <= .002) { count++; end = row.node + 1; }
        }
        return count;
    }
    return { side, activeNodes: body.activeEnd - body.activeStart + 1,
        linearMidpointErrorMm: statistics(samples.map(row => row.linearMidpointErrorMm)),
        curvedMidpointErrorMm: statistics(defined.map(row => row.curvedMidpointErrorMm)),
        curvedPolylineSampleErrorMm: statistics(defined.map(row => row.curvedPolylineSampleErrorMm)),
        curvedPolylineUpperBoundMm: statistics(defined.map(row => row.continuousGeometryBounds.position.upper)),
        geometryOnlyCounts: [.001, .00025].map(toleranceMm => ({ toleranceMm,
            linearMidpoint: samples.filter(r => r.linearMidpointErrorMm <= toleranceMm).length,
            curvedMidpoint: defined.filter(r => r.curvedMidpointErrorMm <= toleranceMm).length,
            curvedPolylineSamples: defined.filter(r => r.curvedPolylineSampleErrorMm <= toleranceMm).length,
            curvedSamplesAndStretch: defined.filter(r => r.curvedPolylineSampleErrorMm <= toleranceMm &&
                r.maximumSampleStretchError <= .002).length,
            curvedContinuousBoundsAndStretch: defined.filter(r => r.continuousGeometryBounds.position.upper <= toleranceMm &&
                r.continuousGeometryBounds.stretch.upper <= .002).length,
            disjointGeometryCandidates: disjointCandidates(toleranceMm) })),
        samples };
});
const report = {
    input, sha256: createHash('sha256').update(bytes).digest('hex'), state: data.state,
    sourceHashes: Object.fromEntries(['audit-curved-discretization.mjs', 'curved-element-geometry.mjs'].map(name =>
        [name, createHash('sha256').update(fs.readFileSync(new URL(name, import.meta.url))).digest('hex')])),
    method: 'Two old material edges replaced by cubic Hermite position field; endpoint unit directors are interpolated/extrapolated from adjacent material-frame centers.',
    tangentInterpretation: 'Estimated nodal director for a prospective curved element, not an existing runtime DOF; twist is not reduced or certified.',
    qualifications: [
        'Preservation of the current discrete polyline and accuracy against an unknown continuum solution are different criteria.',
        'Samples are rejection witnesses, not certified continuous error bounds.',
        'Continuous geometry bounds use polynomial Bernstein subdivision with an arithmetic noise guard, not formal directed-rounding intervals; they do not certify mechanics.',
        'No runtime mesh, physical tolerance, material law, mass or contact sampling is changed.',
        'Contact/support/material interfaces, force/moment/energy error, torsion and eliminated-mode stability still require certification.',
        'A successful necessary geometry screen does not admit coarsening.'
    ],
    bodies, certifiedCoarseningAdmitted: false
};
fs.writeFileSync(output, JSON.stringify(report, null, 2) + '\n');
console.log(JSON.stringify({ output, bodies: bodies.map(({ samples, ...body }) => body) }, null, 2));
