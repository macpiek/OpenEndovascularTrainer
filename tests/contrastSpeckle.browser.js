import * as THREE from 'three';
import {contrastSpeckleFixture, referenceLumenIntegral} from './helpers/contrastSpeckleFixture.js';

const params = new URLSearchParams(location.search);
const size = Math.max(64, Math.min(512, Number(params.get('size')) || 256));
const diagnostics = params.get('diagnostics') === '1';
const results = document.querySelector('#results'), metrics = document.querySelector('#metrics');
const report = {size, noise: 0, near: .1, far: 2000, cases: [], status: 'running'};
window.contrastSpeckleReport = report;
const renderer = new THREE.WebGLRenderer({antialias: false});
renderer.setPixelRatio(1); renderer.setSize(size, size);
const target = new THREE.WebGLRenderTarget(size, size, {type: THREE.HalfFloatType, depthBuffer: false});
const scene = new THREE.Scene(), raycaster = new THREE.Raycaster();
const raw = new Uint16Array(size * size * 4);
let contextLost = false;
renderer.domElement.addEventListener('webglcontextlost', () => {contextLost = true;});
const publish = () => {metrics.textContent = JSON.stringify(report, null, 2); document.body.dataset.status = report.status;};
function cameraFor(offset, angle, perspective) {
    const span = 86, distance = 340, a = angle * Math.PI / 180;
    const camera = perspective
        ? new THREE.PerspectiveCamera(2 * Math.atan(span / distance) * 180 / Math.PI, 1, report.near, report.far)
        : new THREE.OrthographicCamera(-span, span, span, -span, report.near, report.far);
    camera.position.copy(offset).add(new THREE.Vector3(distance * Math.sin(a), 25, distance * Math.cos(a)));
    camera.lookAt(offset); camera.updateMatrixWorld(true);
    return camera;
}
function expectedFor(fixture, camera) {
    const output = new Float32Array(size * size);
    for (let y = 0; y < size; y++) for (let x = 0; x < size; x++) {
        raycaster.setFromCamera(new THREE.Vector2(2 * (x + .5) / size - 1, 2 * (y + .5) / size - 1), camera);
        output[x + y * size] = referenceLumenIntegral(fixture.reference, raycaster.ray);
    }
    return output;
}
function render(surface, camera, shader = surface.material.fragmentShader) {
    const original = surface.material.fragmentShader;
    surface.mesh.updateMatrixWorld(true);
    surface.material.uniforms.diagnosticCombinedInverse ??= {value: new THREE.Matrix4()};
    surface.material.uniforms.diagnosticCombinedInverse.value.multiplyMatrices(camera.projectionMatrix, camera.matrixWorldInverse)
        .multiply(surface.mesh.matrixWorld).invert();
    if (shader !== original) {surface.material.fragmentShader = shader; surface.material.needsUpdate = true;}
    scene.add(surface.mesh); renderer.setRenderTarget(target); renderer.setClearColor(0, 0); renderer.clear();
    const start = performance.now();
    try {
        renderer.render(scene, camera); renderer.readRenderTargetPixels(target, 0, 0, size, size, raw);
        if (contextLost) throw Error('WebGL context lost');
        return {ms: performance.now() - start,
            rgba: Float32Array.from(raw, value => THREE.DataUtils.fromHalfFloat(value)),
            values: Float32Array.from({length: size * size}, (_, i) => THREE.DataUtils.fromHalfFloat(raw[4 * i]))};
    } finally {
        scene.remove(surface.mesh);
        if (shader !== original) {surface.material.fragmentShader = original; surface.material.needsUpdate = true;}
    }
}
function compare(actual, expected) {
    const errors = [], firstFailures = [];
    let interior = 0, empty = 0, short = 0, outside = 0;
    for (let y = 1; y < size - 1; y++) for (let x = 1; x < size - 1; x++) {
        const i = x + y * size, value = actual[i], reference = expected[i];
        if (reference === 0 && value > .01) outside++;
        // The interior mask excludes subpixel silhouette differences.
        if (reference < 2 || Math.min(expected[i - 1], expected[i + 1], expected[i - size], expected[i + size]) < 1) continue;
        interior++;
        if (value < 1e-5) empty++;
        if (value < reference * .5) {
            short++;
            if (firstFailures.length < 12) firstFailures.push({x, y, actual: value, expected: reference});
        }
        errors.push(Math.abs(value - reference) / reference);
    }
    errors.sort((a, b) => a - b);
    return {interior, empty, short, outside, p95RelativeError: errors[Math.floor(errors.length * .95)] ?? 0, firstFailures};
}
function preview(section, values, caption, rgba = null) {
    const canvas = document.createElement('canvas'); canvas.width = canvas.height = size;
    const bytes = new Uint8ClampedArray(size * size * 4);
    for (let y = 0; y < size; y++) for (let x = 0; x < size; x++) {
        const value = Math.round(245 * Math.exp(-values[x + y * size] * .35)), at = 4 * (x + (size - 1 - y) * size);
        bytes[at] = bytes[at + 1] = bytes[at + 2] = value; bytes[at + 3] = 255;
        if (rgba) for (let c = 0; c < 3; c++) bytes[at + c] = Math.min(255, Math.round(255 * rgba[4 * (x + y * size) + c]));
    }
    canvas.getContext('2d').putImageData(new ImageData(bytes, size, size), 0, 0);
    const figure = document.createElement('figure'), label = document.createElement('figcaption');
    label.textContent = caption; figure.append(canvas, label); section.append(figure);
}
function variant(source, mode) {
    if (mode === 'legacyCombinedInverse') {
        // Keep the former bug reproducible without modifying production:
        // translating homogeneous coordinates before perspective division
        // loses ray-direction precision when near/far differ substantially.
        source = source.replace('uniform mat4 columnInverseProjection;',
            'uniform mat4 columnInverseProjection;\nuniform mat4 diagnosticCombinedInverse;');
        const start = source.indexOf('    vec4 nearH =', source.indexOf('void main() {'));
        const end = source.indexOf('    uvec3 entryIndices =', start);
        if (start < 0 || end < 0) throw Error('Missing production detector-ray block');
        const legacy = `    vec4 nearH = diagnosticCombinedInverse * vec4(ndc, -1., 1.);
    vec4 farH = diagnosticCombinedInverse * vec4(ndc, 1., 1.);
    vec3 nearPoint = nearH.xyz / nearH.w;
    vec3 ray = normalize(farH.xyz / farH.w - nearPoint);
`;
        return source.slice(0, start) + legacy + source.slice(end);
    }
    if (mode === 'exitOnly') {
        const from = source.indexOf('    float mean;', source.indexOf('void main() {'));
        const to = source.indexOf('    float depth =', from);
        if (from < 0 || to < 0) throw Error('Missing mean-selection block');
        return source.slice(0, from) + '    float mean = 1.;\n' + source.slice(to);
    }
    if (mode === 'rejectReason') {
        const start = source.indexOf('void main() {');
        const head = source.slice(0, start);
        let body = source.slice(start), reasons = 0;
        const colors = ['1.,0.,0.', '1.,.5,0.', '1.,0.,1.', '1.,1.,0.', '0.,0.,1.', '0.,1.,1.'];
        body = body.replace(/discard;/g, () => `{gl_FragColor = vec4(${colors[reasons++]},1.); return;}`);
        if (reasons !== colors.length) throw Error(`Expected six rejection sites, found ${reasons}`);
        const success = 'gl_FragColor = vec4((debugMode ? debugColor : vec3(1.)) * depth, depth);';
        if (!body.includes(success)) throw Error('Missing final optical output marker');
        body = body.replace(success, 'gl_FragColor = vec4(0.,1.,0.,1.);');
        return head + body;
    }
    if (mode === 'rasterCoverage') return source.replace('void main() {', 'void main() { gl_FragColor = vec4(1.); return;');
    throw Error(`Unknown diagnostic mode: ${mode}`);
}
function countReasons(rgba, expected) {
    const names = {'000': 'noRaster', '100': 'parallelPlane', '110': 'activeBounds', '101': 'firstRayMiss',
        '001': 'wrongSideOrZeroDistance', '011': 'zeroMean', '010': 'success'};
    const counts = {};
    for (let y = 1; y < size - 1; y++) for (let x = 1; x < size - 1; x++) {
        const i = x + y * size;
        if (expected[i] < 2 || Math.min(expected[i - 1], expected[i + 1], expected[i - size], expected[i + size]) < 1) continue;
        const rgb = [0, 1, 2].map(c => rgba[4 * i + c]);
        const key = rgb.map(v => Math.round(v)).join('');
        // Orange active-bounds and yellow retry-miss differ in their green channel.
        const name = rgb[0] > .9 && rgb[1] > .9 && rgb[2] < .1 ? 'retryRayMiss' : names[key] ?? `other:${rgb.join(',')}`;
        counts[name] = (counts[name] ?? 0) + 1;
    }
    return counts;
}
try {
    for (const translated of [false, true]) {
        results.textContent = `Building ${translated ? 'translated' : 'origin'} lumen…`;
        await new Promise(resolve => requestAnimationFrame(resolve));
        const fixture = contrastSpeckleFixture({translated});
        try {
            for (const [angle, perspective] of [[0, false], [0, true], [31, true], [45, true], [79, true], [150, true]]) {
                const name = `${translated ? 'translated' : 'origin'}, ${angle}°, ${perspective ? 'perspective' : 'orthographic'}`;
                results.textContent = `Testing ${name}…`;
                await new Promise(resolve => requestAnimationFrame(resolve));
                const camera = cameraFor(fixture.offset, angle, perspective);
                const expected = expectedFor(fixture, camera), actual = render(fixture.surface, camera);
                const comparison = compare(actual.values, expected);
                const legacy = render(fixture.surface, camera, variant(fixture.surface.material.fragmentShader, 'legacyCombinedInverse'));
                const item = {name, triangles: fixture.triangles, cachedRaster: Boolean(fixture.surface.material.userData.columnRasterPositionCached),
                    gpuReadbackMs: actual.ms, ...comparison, legacyCombinedInverse: {...compare(legacy.values, expected), gpuReadbackMs: legacy.ms}, variants: {}};
                report.cases.push(item);
                const section = document.createElement('section'); document.querySelector('main').append(section);
                preview(section, expected, `${name}: CPU reference`);
                preview(section, actual.values, `Production: ${comparison.empty} empty / ${comparison.short} short interior chords`);
                preview(section, legacy.values, `Former combined inverse: ${item.legacyCombinedInverse.empty} empty / ${item.legacyCombinedInverse.short} short interior chords`);
                if (diagnostics || comparison.short) {
                    for (const mode of ['exitOnly', 'rasterCoverage', 'rejectReason']) {
                        const result = render(fixture.surface, camera, variant(fixture.surface.material.fragmentShader, mode));
                        item.variants[mode] = mode === 'rejectReason' ? {counts: countReasons(result.rgba, expected)} :
                            {...compare(result.values, expected), gpuReadbackMs: result.ms};
                        preview(section, result.values, `${mode}${mode === 'rejectReason' ? ': green = accepted, purple = first ray miss, blue = wrong side' : `: ${item.variants[mode].empty} empty`}`,
                            mode === 'rejectReason' ? result.rgba : null);
                    }
                }
                publish();
            }
        } finally {fixture.dispose();}
    }
    report.summary = {
        productionEmpty: report.cases.reduce((n, c) => n + c.empty, 0),
        productionShort: report.cases.reduce((n, c) => n + c.short, 0),
        productionWorstP95: Math.max(...report.cases.map(c => c.p95RelativeError)),
        legacyEmpty: report.cases.reduce((n, c) => n + c.legacyCombinedInverse.empty, 0),
        legacyShort: report.cases.reduce((n, c) => n + c.legacyCombinedInverse.short, 0),
        legacyWorstP95: Math.max(...report.cases.map(c => c.legacyCombinedInverse.p95RelativeError))
    };
    report.status = report.cases.every(result => result.interior > 100 && !result.short && result.p95RelativeError < .02) ? 'pass' : 'fail';
    results.textContent = `${report.status.toUpperCase()}: ${report.cases.length} views, production ${report.summary.productionEmpty} empty / ${report.summary.productionShort} short interior chords; former combined inverse ${report.summary.legacyEmpty} empty / ${report.summary.legacyShort} short. No detector noise was used.`;
} catch (error) {
    report.status = 'error'; report.error = error.stack; results.textContent = error.stack;
} finally {renderer.dispose(); target.dispose(); publish();}
