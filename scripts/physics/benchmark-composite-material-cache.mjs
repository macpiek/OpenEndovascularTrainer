import fs from 'node:fs';
import { createHash } from 'node:crypto';
import { createKirchhoffCompositeMaterialCache } from '../../src/physics/kirchhoffCompositeMaterialCache.js';
import { createKirchhoffCompositeMeshUpdate } from '../../src/physics/kirchhoffCompositeMeshUpdate.js';
import { buildKirchhoffCompositeTopology } from '../../src/physics/kirchhoffCompositeTopology.js';
import { buildKirchhoffCompositeMesh } from '../../src/physics/kirchhoffCompositeMesh.js';
import { kirchhoffMaterialProfile } from '../../src/physics/kirchhoffMaterialProfile.js';
import { createCompositeChainWorkspace, assembleCompositeChain, solveCompositeChainIncrement } from '../../src/physics/kirchhoffCompositeChain.js';

const root = new URL('../../', import.meta.url), output = process.argv[2] ?? '/tmp/oet-composite-material-cache.json';
const paths = ['src/physics/kirchhoffCompositeMaterialCache.js', 'src/physics/kirchhoffCompositeMeshUpdate.js',
    'src/physics/kirchhoffCompositeMesh.js', 'src/physics/kirchhoffCompositeTopology.js', 'src/physics/kirchhoffCompositeChain.js',
    'src/physics/kirchhoffCompositeElementFast.js', 'src/physics/kirchhoffCompositeElementFastKernelBytes.js',
    'src/physics/kirchhoffMaterialProfile.js', 'src/physics/catheterMaterialProfile.js', 'src/physics/guidewireMaterialProfile.js',
    'scripts/physics/benchmark-composite-material-cache.mjs', 'tests/kirchhoffCompositeMaterialCache.test.js'];
const sha = bytes => createHash('sha256').update(bytes).digest('hex');
const hashes = () => Object.fromEntries(paths.map(p => [p, sha(fs.readFileSync(new URL(p, root)))]));
const median = values => { const s = [...values].sort((a, b) => a - b), m = s.length >> 1; return s.length % 2 ? s[m] : (s[m - 1] + s[m]) / 2; };
const pose = x => [x, .2 * Math.sin(x / 40), .1 * Math.cos(x / 37)];
const spinFields = { wire: s => .001 * s, catheter: s => -.002 * s };
const report = { date: new Date().toISOString(), sourceBefore: hashes(), warmups: 3, pairs: 6,
    scope: 'Real immutable Glidewire/Berenstein profile adapters built once. Exact tip/material boundaries. Fixed optional spatial nodes. Paired full uncached Mesh build vs cached material build, topology+mesh+workspace+assembly+direction measured. Same synthetic 3D pose and independent spins per pair. Constitutive direction ONLY: no dt acceptance, remeshing history transfer, dynamics/contact/anatomy/FPS claim.',
    changesFromColdBaseline: 'Profile adapters and optional mesh nodes persist. Both arms use identical topology, positions, frames, spins, nominal Voronoi weights and cached workspace policy. Mandatory moving boundaries still require a candidate remesh, explicitly not an accepted runtime state transfer.',
    cases: [] };

for (const insertion of [9, 160, 310]) {
    const cache = createKirchhoffCompositeMaterialCache();
    const wire = cache.profileTool({ profile: kirchhoffMaterialProfile('glidewire'), materialInterval: [0, 500], insertion: 318, radius: .4445 });
    const catheter = cache.profileTool({ profile: kirchhoffMaterialProfile('berenstein'), materialInterval: [0, 500], insertion: insertion + .017, radius: .8, innerRadius: .485 });
    const initial = buildKirchhoffCompositeTopology({ wire, catheter }), optional = [0, 318];
    for (const section of initial.sections) {
        const count = Math.ceil((section.end - section.start) / 5);
        for (let i = 1; i < count; i++) optional.push(section.start + (section.end - section.start) * i / count);
    }
    optional.sort((a, b) => a - b);
    const workspaces = { oracle: new Map(), cached: new Map() }, rows = [], warmupRows = [];
    function run(kind, catheterInsertion) {
        const start = performance.now();
        const topology = buildKirchhoffCompositeTopology({ wire, catheter: { ...catheter, insertion: catheterInsertion } });
        const partitioned = performance.now(), statistics = { ...cache.statistics };
        const mesh = buildKirchhoffCompositeMesh({ topology, meshCoordinates: optional, sampleCenterline: pose, spinFields,
            materialIntegrator: kind === 'cached' ? cache.integrator : null });
        const compiled = performance.now(), layoutKey = JSON.stringify(mesh.layout.edgeToolIds);
        if (!workspaces[kind].has(layoutKey)) workspaces[kind].set(layoutKey, createCompositeChainWorkspace(mesh.layout));
        const workspace = workspaces[kind].get(layoutKey), ready = performance.now();
        assembleCompositeChain(mesh.data, workspace); const assembled = performance.now();
        const result = solveCompositeChainIncrement(workspace, { diagonal: new Float64Array(mesh.layout.dofCount).fill(3), tolerance: 1e-7 });
        const done = performance.now();
        return { mesh, topology, energy: workspace.energy, gradient: Array.from(workspace.gradient), hessian: Array.from(workspace.hessian),
            increment: Array.from(workspace.increment), summary: { kind, catheterInsertion, nodes: mesh.layout.nodeCount,
                dofCount: mesh.layout.dofCount, factorBand: mesh.layout.band,
                status: result.converged ? 'constitutive-direction-converged' : 'constitutive-direction-failed',
                minimumCoordinateSpacing: Math.min(...Array.from(mesh.data.coordinates).slice(1).map((x, i) => x - mesh.data.coordinates[i])),
                acceptedDt: null, factorCount: result.factorizations, residual: result.maximumResidual,
                topologyMs: partitioned - start, materialMeshMs: compiled - partitioned, workspaceMs: ready - compiled,
                assemblyMs: assembled - ready, solveMs: done - assembled, totalMs: done - start,
                quadratureEvaluations: mesh.quadrature.evaluations,
                cacheStatistics: Object.fromEntries(Object.keys(statistics).map(k => [k, cache.statistics[k] - statistics[k]])),
                poseInputHash: sha(JSON.stringify({ coordinates: Array.from(mesh.data.coordinates), positions: mesh.data.positions,
                    reference: mesh.data.reference, tools: mesh.data.tools.map(t => ({ id: t.id, dsDx: t.dsDx,
                        angles: Array.from(t.angles), referenceTwists: Array.from(t.referenceTwists), maps: mesh.materialMaps.get(t.id) })) })) } };
    }
    const difference = (a, b) => a.reduce((max, value, i) => Math.max(max, Math.abs(value - b[i]) / Math.max(1, Math.abs(value), Math.abs(b[i]))), 0);
    for (let index = 0; index < report.warmups + report.pairs; index++) {
        const delta = index < report.warmups + 3 ? .011 * index : .011 * (2 * (report.warmups + 2) - index);
        const catheterInsertion = catheter.insertion + delta;
        const order = index % 2 ? ['cached', 'oracle'] : ['oracle', 'cached'], pair = {};
        for (const kind of order) pair[kind] = run(kind, catheterInsertion);
        const a = pair.oracle, b = pair.cached;
        if (a.summary.poseInputHash !== b.summary.poseInputHash) throw new Error('Paired pose/input hashes differ');
        const parity = { energy: difference([a.energy], [b.energy]), gradient: difference(a.gradient, b.gradient),
            hessian: difference(a.hessian, b.hessian), increment: difference(a.increment, b.increment) };
        if (Math.max(...Object.values(parity)) > 5e-8) throw new Error(`Full operator parity failed ${JSON.stringify(parity)}`);
        const proposal = createKirchhoffCompositeMeshUpdate(b.mesh, { materialIntegrator: cache.integrator }).plan(
            buildKirchhoffCompositeTopology({ wire, catheter: { ...catheter, insertion: catheterInsertion + .001 } }));
        if (!proposal.requiresStateTransfer) throw new Error('Moving-tip benchmark must not claim accepted history transfer');
        (index >= report.warmups ? rows : warmupRows).push({ order, oracle: a.summary, cached: b.summary, parity,
            nextFeedStatus: proposal.status, missingNextBoundaries: proposal.missingBoundaries });
    }
    const keys = ['topologyMs', 'materialMeshMs', 'workspaceMs', 'assemblyMs', 'solveMs', 'totalMs', 'quadratureEvaluations'];
    const medians = Object.fromEntries(['oracle', 'cached'].map(kind => [kind, Object.fromEntries(keys.map(k => [k, median(rows.map(r => r[kind][k]))]))]));
    report.cases.push({ insertion, medians, pipelineSpeedup: medians.oracle.totalMs / medians.cached.totalMs,
        materialSpeedup: medians.oracle.materialMeshMs / medians.cached.materialMeshMs, cacheStatistics: { ...cache.statistics },
        directionCounts: Object.fromEntries(['oracle', 'cached'].map(kind => [kind, {
            measuredConverged: rows.filter(r => r[kind].status === 'constitutive-direction-converged').length,
            measuredFailed: rows.filter(r => r[kind].status === 'constitutive-direction-failed').length,
            warmupConverged: warmupRows.filter(r => r[kind].status === 'constitutive-direction-converged').length,
            warmupFailed: warmupRows.filter(r => r[kind].status === 'constitutive-direction-failed').length
        }])), rows, warmupRows });
}
report.sourceAfter = hashes(); report.sourceStable = JSON.stringify(report.sourceBefore) === JSON.stringify(report.sourceAfter);
fs.writeFileSync(output, JSON.stringify(report, null, 2) + '\n');
console.log(JSON.stringify({ output, sourceStable: report.sourceStable, cases: report.cases.map(({ rows, warmupRows, ...summary }) => summary) }, null, 2));
