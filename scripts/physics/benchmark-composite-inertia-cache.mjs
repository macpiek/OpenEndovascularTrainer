import fs from 'node:fs';
import assert from 'node:assert/strict';
import { createHash } from 'node:crypto';
import { createCompositeInertiaCache } from '../../src/physics/kirchhoffCompositeInertiaCache.js';
import { createCompositeInertiaWorkspace, assembleCompositeTranslationalInertia, scatterCompositeTranslationalInertia } from '../../src/physics/kirchhoffCompositeKinematics.js';
import { createCompositeChainWorkspace, assembleCompositeChain, solveCompositeChainIncrement } from '../../src/physics/kirchhoffCompositeChain.js';
import { createKirchhoffCompositeMaterialCache } from '../../src/physics/kirchhoffCompositeMaterialCache.js';
import { buildKirchhoffCompositeTopology } from '../../src/physics/kirchhoffCompositeTopology.js';
import { buildKirchhoffCompositeMesh } from '../../src/physics/kirchhoffCompositeMesh.js';
import { kirchhoffMaterialProfile } from '../../src/physics/kirchhoffMaterialProfile.js';

const root = new URL('../../', import.meta.url), output = process.argv[2] ?? '/tmp/oet-composite-inertia-cache.json';
const files = ['src/physics/kirchhoffCompositeInertiaCache.js', 'src/physics/kirchhoffCompositeKinematics.js',
    'src/physics/kirchhoffCompositeChain.js', 'src/physics/kirchhoffCompositeElement.js', 'src/physics/kirchhoffCompositeElementFast.js',
    'src/physics/kirchhoffCompositeElementFastKernelBytes.js', 'src/physics/kirchhoffLinearKernel.js', 'src/physics/kirchhoffLinearKernelBytes.js',
    'src/physics/kirchhoffCompositeMaterialCache.js', 'src/physics/kirchhoffCompositeMesh.js', 'src/physics/kirchhoffCompositeTopology.js',
    'src/physics/kirchhoffMaterialProfile.js', 'src/physics/guidewireMaterialProfile.js', 'src/physics/catheterMaterialProfile.js',
    'scripts/physics/benchmark-composite-inertia-cache.mjs'];
const hash = value => createHash('sha256').update(typeof value === 'string' || Buffer.isBuffer(value) ? value : JSON.stringify(value)).digest('hex');
const hashes = () => Object.fromEntries(files.map(path => [path, hash(fs.readFileSync(new URL(path, root)))]));
const bytes = value => Buffer.from(value.buffer, value.byteOffset, value.byteLength);
const matrixHash = chain => hash(bytes(chain.hessian));
const stats = values => { const sorted = values.toSorted((a, b) => a - b); return { mean: values.reduce((a, b) => a + b, 0) / values.length,
    median: (sorted[Math.floor((sorted.length - 1) / 2)] + sorted[Math.ceil((sorted.length - 1) / 2)]) / 2, max: sorted.at(-1), samples: values }; };
function difference(a, b) {
    let maximumAbsolute = 0, maximumRelative = 0;
    for (let i = 0; i < a.length; i++) { const delta = Math.abs(a[i] - b[i]); maximumAbsolute = Math.max(maximumAbsolute, delta);
        maximumRelative = Math.max(maximumRelative, delta / Math.max(1, Math.abs(a[i]), Math.abs(b[i]))); }
    return { maximumAbsolute, maximumRelative };
}
function fixture(nodes, insertion = 310) {
    const materialCache = createKirchhoffCompositeMaterialCache();
    const wire = materialCache.profileTool({ profile: kirchhoffMaterialProfile('glidewire'), materialInterval: [0, 500], insertion: 318, radius: .4445 });
    const catheter = materialCache.profileTool({ profile: kirchhoffMaterialProfile('berenstein'), materialInterval: [0, 500], insertion, radius: .8, innerRadius: .485 });
    const topology = buildKirchhoffCompositeTopology({ wire, catheter });
    const coordinates = [topology.sections[0].start, ...topology.sections.map(s => s.end)];
    // Retain every exact material/tip boundary; only subdivide the largest
    // current interval until this fixed microfixture has the requested size.
    while (coordinates.length < nodes) {
        let split = 0;
        for (let edge = 1; edge + 1 < coordinates.length; edge++)
            if (coordinates[edge + 1] - coordinates[edge] > coordinates[split + 1] - coordinates[split]) split = edge;
        coordinates.splice(split + 1, 0, (coordinates[split] + coordinates[split + 1]) / 2);
    }
    assert.equal(coordinates.length, nodes);
    const mesh = buildKirchhoffCompositeMesh({ topology, meshCoordinates: coordinates, sampleCenterline: x => [x, .2 * Math.sin(x / 40), .1 * Math.cos(x / 37)],
        spinFields: { wire: .03, catheter: -.02 }, materialIntegrator: materialCache.integrator });
    const previousPositions = structuredClone(mesh.data.positions), dt = 1 / 120;
    const inertiaEdges = mesh.layout.edgeToolIds.map((ids, edge) => ({ tools: ids.map(id => {
        const map = mesh.materialMaps.get(id)[edge];
        return { id, massPerMaterialLength: id === 'wire' ? 1 / 5 : 1.4 / 4,
            materialMap: { sStart: map.sStart, dsDx: map.dsDx, dsDt: id === 'wire' ? [-.3, -.7] : [.4, 1.2] },
            oldMaterialVelocities: [[.1, -.03, .02], [.14, -.02, .01]] };
    }) }));
    const fixed = new Uint8Array(mesh.layout.dofCount);
    for (const offsets of mesh.layout.spins.values()) fixed[offsets.find(v => v >= 0)] = 1;
    for (let node = 0; node < 2; node++) for (let axis = 0; axis < 3; axis++) fixed[mesh.layout.positions[node] + axis] = 1;
    return { mesh, topology, fixed, dt, previousPositions, inertiaEdges };
}
function originalAppender(f) {
    const workspace = [null, createCompositeInertiaWorkspace(1), createCompositeInertiaWorkspace(2)];
    const result = { energy: 0, kineticEnergy: 0 };
    return (positions, chain) => {
        result.energy = result.kineticEnergy = 0;
        for (let edge = 0; edge < f.inertiaEdges.length; edge++) {
            const tools = f.inertiaEdges[edge].tools;
            const local = assembleCompositeTranslationalInertia({ coordinates: [f.mesh.data.coordinates[edge], f.mesh.data.coordinates[edge + 1]],
                positions: [positions[edge], positions[edge + 1]], previousPositions: [f.previousPositions[edge], f.previousPositions[edge + 1]], dt: f.dt, tools }, workspace[tools.length]);
            result.energy += local.energy; result.kineticEnergy += local.kineticEnergy;
            scatterCompositeTranslationalInertia(local, edge, chain);
        }
        return result;
    };
}
const report = { scope: 'Real Glidewire/Berenstein profiles on ONE fixed common chain, wire318/cat310. Inertia-only and elastic+full consistent inertia+one original-matrix linear direction. Not a nonlinear accepted dt, contact/length solve, feed/remesh transaction, anatomy run, or FPS certificate.',
    mass: 'Body model units/mm: wire1/5 and catheter1.4/4; not measured kg. Independent opposite affine material-map rates.',
    dt: 1 / 120, linearTolerance: 5e-10, node: process.version, sourceBefore: hashes(), cases: [] };
for (const nodes of [65, 128]) {
    const setupStart = performance.now(), f = fixture(nodes), setupMs = performance.now() - setupStart;
    const compileStart = performance.now();
    const cache = createCompositeInertiaCache({ layout: f.mesh.layout, coordinates: f.mesh.data.coordinates, previousPositions: f.previousPositions, dt: f.dt, inertiaEdges: f.inertiaEdges });
    const coldCompileMs = performance.now() - compileStart;
    const arms = { original: { workspace: createCompositeChainWorkspace(f.mesh.layout), append: originalAppender(f) },
        compiled: { workspace: createCompositeChainWorkspace(f.mesh.layout), append: cache.append } };
    const preparedHash = hash({ coordinates: f.mesh.data.coordinates, previousPositions: f.previousPositions, inertiaEdges: f.inertiaEdges, dt: f.dt, fixed: Array.from(f.fixed) });
    const rows = [];
    for (let pair = -4; pair < 8; pair++) {
        const positions = f.previousPositions.map((p, i) => p.map((v, axis) => v + 1e-4 * Math.cos(i * .2 + axis + .13 * pair)));
        f.mesh.data.positions = positions;
        for (const mode of ['inertia-only', 'elastic-inertia-direction']) {
            const row = { pair, mode, preparedHash, poseHash: hash(positions) }, responses = {};
            for (const armName of pair % 2 ? ['compiled', 'original'] : ['original', 'compiled']) {
                const arm = arms[armName], chain = arm.workspace;
                const start = performance.now();
                if (mode === 'inertia-only') { chain.energy = 0; chain.gradient.fill(0); chain.hessian.fill(0); }
                else assembleCompositeChain(f.mesh.data, chain);
                const response = arm.append(positions, chain);
                const direction = mode === 'inertia-only' ? null : solveCompositeChainIncrement(chain, { fixed: f.fixed, tolerance: report.linearTolerance });
                row[armName] = { elapsedMs: performance.now() - start, energy: chain.energy, inertialEnergy: response.energy, kineticEnergy: response.kineticEnergy,
                    directionConverged: direction?.converged ?? null, maximumResidual: direction?.maximumResidual ?? null,
                    factorizations: direction?.factorizations ?? 0, backsolves: direction?.linearSolves ?? 0, fullMatrixHash: matrixHash(chain) };
                responses[armName] = { g: chain.gradient.slice(), h: chain.hessian.slice(), increment: direction?.increment.slice() };
            }
            assert.equal(row.original.fullMatrixHash, row.compiled.fullMatrixHash);
            const g = difference(responses.original.g, responses.compiled.g);
            assert.ok(g.maximumRelative <= 1e-11, 'full gradient parity');
            row.parity = { fullMatrixByteIdentical: true, gradient: g, direction: responses.original.increment ? difference(responses.original.increment, responses.compiled.increment) : null,
                inertialEnergy: Math.abs(row.original.inertialEnergy - row.compiled.inertialEnergy), kineticEnergy: Math.abs(row.original.kineticEnergy - row.compiled.kineticEnergy) };
            if (pair >= 0) rows.push(row);
        }
    }
    const summary = Object.fromEntries(['inertia-only', 'elastic-inertia-direction'].map(mode => {
        const selected = rows.filter(row => row.mode === mode), original = stats(selected.map(row => row.original.elapsedMs)), compiled = stats(selected.map(row => row.compiled.elapsedMs));
        return [mode, { originalMs: original, compiledMs: compiled, medianSpeedup: original.median / compiled.median,
            convergedDirections: Object.fromEntries(['original', 'compiled'].map(arm => [arm, selected.filter(row => row[arm].directionConverged === true).length])), pairs: selected.length }];
    }));
    const warmCompileTimes = [];
    for (let repeat = 0; repeat < 4; repeat++) {
        const compileStart = performance.now();
        const recompiled = createCompositeInertiaCache({ layout: f.mesh.layout, coordinates: f.mesh.data.coordinates,
            previousPositions: f.previousPositions, dt: f.dt, inertiaEdges: f.inertiaEdges });
        warmCompileTimes.push(performance.now() - compileStart);
        assert.equal(recompiled.sampleCount, cache.sampleCount);
    }
    const afterPreparedHash = hash({ coordinates: f.mesh.data.coordinates, previousPositions: f.previousPositions, inertiaEdges: f.inertiaEdges, dt: f.dt, fixed: Array.from(f.fixed) });
    assert.equal(preparedHash, afterPreparedHash);
    report.cases.push({ nodes, dofCount: f.mesh.layout.dofCount, band: f.mesh.layout.band, sampleCount: cache.sampleCount,
        storedMatrixEntries: cache.storedMatrixEntries, setupMs, coldCompileMs, warmCompileMs: stats(warmCompileTimes), preparedHash, preparedUnchanged: true, summary, rows });
}
report.sourceAfter = hashes(); report.sourceStable = JSON.stringify(report.sourceBefore) === JSON.stringify(report.sourceAfter);
fs.writeFileSync(output, JSON.stringify(report, null, 2) + '\n');
console.log(JSON.stringify({ output, sourceStable: report.sourceStable, cases: report.cases.map(({ rows, ...rest }) => rest) }, null, 2));
