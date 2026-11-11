import test from 'node:test';
import assert from 'node:assert/strict';
import {columnProjectionFragmentShader} from '../src/contrast/anatomicalColumnProjection.js';
import {contrastGpuDiagnosticModes, contrastGpuDiagnosticShader} from './helpers/contrastGpuDiagnosticShader.js';
import {contrastColumnQuadratureReference} from './helpers/contrastColumnQuadratureReference.js';
import {columnNearestReference} from './helpers/columnNearestReference.js';
import {columnSampleHistoryReference, columnPathConstantHistoryReference} from './helpers/columnHistoryReference.js';

const shader = columnProjectionFragmentShader;
const main = source => source.slice(source.lastIndexOf('void main()'));

test('GPU diagnostics leave the production default byte-for-byte unchanged', () => {
    assert.equal(contrastGpuDiagnosticShader(shader), shader);
    for (const mode of Object.keys(contrastGpuDiagnosticModes)) {
        const variant = contrastGpuDiagnosticShader(shader, mode);
        assert.equal(typeof variant, 'string');
        assert.equal(contrastGpuDiagnosticShader(shader, 'full'), shader);
    }
    assert.throws(() => contrastGpuDiagnosticShader(shader, 'typo'), /Unknown/);
});

test('BVH diagnostics retain exact entry, exit, chord and transfer code', () => {
    const original = main(shader);
    const prefix = original.slice(0, original.indexOf('    float mean;'));
    const suffix = original.slice(original.indexOf('    float depth ='));
    for (const mode of ['exitOnly', 'noMix', 'junctionMask']) {
        const variant = main(contrastGpuDiagnosticShader(shader, mode));
        assert.ok(variant.startsWith(prefix));
        assert.ok(variant.endsWith(suffix));
        assert.ok(!variant.includes('junctionColumnAverage('));
        assert.ok(!variant.includes('spatialColumnAverage('));
    }
    assert.ok(main(contrastGpuDiagnosticShader(shader, 'junctionMask')).includes('columnHasJunction('));
    assert.ok(!main(contrastGpuDiagnosticShader(shader, 'exitOnly')).includes('columnHasJunction('));
});

test('raster, point deformation and cache diagnostics isolate only their declared work', () => {
    const raster = main(contrastGpuDiagnosticShader(shader, 'rasterOnly'));
    assert.ok(!raster.includes('bvhIntersectFirstHit('));
    assert.ok(raster.includes('gl_FragColor = vec4(.05)'));
    const deformed = shader.replace('vec3 columnRestPosition(vec3 point) { return point; }',
        'vec3 columnRestPosition(vec3 point) { vec3 rest = point - vesselDisplacement(point); return point - vesselDisplacement(rest); }');
    const noRest = contrastGpuDiagnosticShader(deformed, 'noRestDeformation');
    assert.equal(main(noRest), main(shader));
    assert.ok(!noRest.includes('vesselDisplacement('));
    assert.ok(contrastGpuDiagnosticShader(shader, 'uncached').includes('#define junctionColumnValue junctionColumnValueUncached'));
    assert.throws(() => contrastGpuDiagnosticShader(shader.replace('    float mean;', '    float renamed;'), 'exitOnly'), /mean-selection/);
});

test('stack and candidate A/B modes keep the main ray calculation unchanged', () => {
    const reference = contrastGpuDiagnosticShader(shader, 'referenceStack');
    assert.ok(reference.includes(contrastColumnQuadratureReference.trim()));
    assert.equal(main(reference), main(shader));
    assert.ok(reference.includes('vec3 candidates[4]'));
    const wide = contrastGpuDiagnosticShader(shader, 'wideCandidates');
    assert.ok(wide.includes('vec3 candidates[16]'));
    assert.ok(wide.includes('if (count == 16) return junctionColumnValueUncached('));
    assert.ok(wide.includes('vec4 pendingRight[4]'));
    assert.equal(main(wide), main(shader));
    const both = contrastGpuDiagnosticShader(shader, 'referenceWide');
    assert.ok(both.includes(contrastColumnQuadratureReference.trim()));
    assert.ok(both.includes('vec3 candidates[16]'));
    assert.equal(main(both), main(shader));
    assert.throws(() => contrastGpuDiagnosticShader(shader.replace('vec3 candidates[4]', 'vec3 candidates[8]'), 'wideCandidates'), /marker/);
});

test('overflow mask only adds a flag at the genuine uncapped fallback and final discard', () => {
    const mask = contrastGpuDiagnosticShader(shader, 'cacheOverflow');
    assert.ok(mask.startsWith('bool diagnosticCacheOverflow = false;'));
    assert.ok(mask.includes('if (count == 4) { diagnosticCacheOverflow = true; return junctionColumnValueUncached('));
    assert.equal(main(mask).replace('    if (!diagnosticCacheOverflow) discard;\n', ''), main(shader));
    assert.equal((mask.match(/diagnosticCacheOverflow = true/g) ?? []).length, 1);
    assert.ok(mask.includes('vec3 candidates[4]'));
});

test('reference nearest mode restores only the frozen projection function', () => {
    const reference = contrastGpuDiagnosticShader(shader, 'referenceNearest');
    assert.ok(reference.includes(columnNearestReference.trim()));
    assert.equal(main(reference), main(shader));
    assert.ok(!reference.includes('int cachedIndex = -1;'));
    assert.ok(shader.includes('int cachedIndex = -1;'));
});

test('reference history mode restores finite interpolation and constant-path proof', () => {
    const reference = contrastGpuDiagnosticShader(shader, 'referenceHistory');
    assert.ok(reference.includes(columnSampleHistoryReference.trim()));
    assert.ok(reference.includes(columnPathConstantHistoryReference.trim()));
    assert.equal(main(reference), main(shader));
});
