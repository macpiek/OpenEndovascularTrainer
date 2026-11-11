import test from 'node:test';
import assert from 'node:assert/strict';
import { HybridContrastSystem } from '../src/contrast/hybridContrastSystem.js';
import { ContrastVolumeRenderer } from '../src/contrast/contrastVolumeRenderer.js';
import { surfaceFixture } from './helpers/contrastSurfaceFixture.js';

function fixture() {
    const source = surfaceFixture(2);
    source.volume.dispose();
    const system = new HybridContrastSystem({
        centerlineSegments: source.network.sourceSegments,
        contactField: { fallbackGeometry: source.geometry },
        localOptions: { capacity: 8 }
    });
    system.synchronized = true;
    const fast = new ContrastVolumeRenderer(system);
    const reference = new ContrastVolumeRenderer(system);
    // Exercise the complete pre-optimization tube preparation as an oracle.
    reference._needsFlowLumenUpdate = () => true;
    return { system, fast, reference, dispose() {
        fast.dispose(); reference.dispose(); source.geometry.dispose();
    } };
}

function compareNative(a, b, history = true) {
    const left = a._flowVolume, right = b._flowVolume;
    for (const name of ['cellValues', 'excludedCells', 'current', ...(history ? ['previous', 'data'] : [])])
        assert.deepEqual(left[name], right[name], `native ${name}`);
    for (const name of ['columnCurrent', ...(history ? ['columnPrevious'] : [])])
        assert.deepEqual(left.uniforms[name].value.image.data, right.uniforms[name].value.image.data, name);
    assert.equal(left.mesh.visible, right.mesh.visible);
    assert.equal(left.uniforms.displayAlpha.value, right.uniforms.displayAlpha.value);
    assert.equal(left.uniforms.debugMode.value, right.uniforms.debugMode.value);
    assert.deepEqual(left.mesh.geometry.groups, right.mesh.geometry.groups);
}

test('native fluoroscopy projects pending iodine as a local jet without also filling the source cell', () => {
    const f = fixture(), solver = f.system.localSolver, surface = f.fast._flowVolume;
    try {
        const edge = f.system.flowNetwork.edges[0], center = edge.start.clone().lerp(edge.end, .5);
        solver.count = 1;
        solver.sourceEdgeIndex[0] = 0;
        solver.sourceCellIndex[0] = Math.floor(edge.cellCount / 2);
        solver.iodineMassMg[0] = solver.initialIodineMassMg[0] = .3;
        solver.positionX[0] = solver.outletPositionX[0] = center.x;
        solver.positionY[0] = solver.outletPositionY[0] = center.y;
        solver.positionZ[0] = solver.outletPositionZ[0] = center.z;
        solver.jetVelocityX[0] = edge.axis.x * 100;
        solver.jetVelocityY[0] = edge.axis.y * 100;
        solver.jetVelocityZ[0] = edge.axis.z * 100;
        solver.outletRadiusMm[0] = .4;
        f.fast.update();
        const jet = surface.projection.outletJet;
        assert.ok(jet.active && jet.count > 0);
        assert.ok(Math.abs(jet.massMg - .3) < 1e-12);
        assert.ok(surface.cellValues.every(value => value === 0), 'unmixed iodine must not be rendered twice');
        assert.ok(surface.mesh.visible && surface.mesh.geometry.groups.length > 0, 'a clear entry wall must still draw the hidden jet');
        assert.ok(surface.activity.bounds.containsPoint(center), 'activity culling includes the outlet');
        // A parcel that has entered the network is no longer drawn locally.
        f.system.flowNetwork.depositIodine(0, solver.sourceCellIndex[0], .3);
        solver.count = 0;
        f.fast.update();
        assert.equal(jet.count, 0);
        assert.ok(surface.cellValues.some(value => value > 0));
        surface.resetHistory();
        assert.equal(jet.active, false);
        assert.equal(surface.mesh.visible, false);
    } finally { f.dispose(); }
});

test('native image and histories match full tube preparation without any unused tube updates', () => {
    const f = fixture(), { system, fast, reference } = f;
    try {
        const textureVersion = fast._flowSampleTexture.version;
        const concentrationVersion = fast.flowMesh.geometry.attributes.flowConcentration.version;
        for (const method of ['_smoothFlowCellsTopologyAware', '_equalizeRenderedJunctionBoundaryConcentrations',
            '_closeInternalFlowChainGaps', '_updateJunctionOpticalWeights']) {
            fast[method] = () => assert.fail(`unused native tube work: ${method}`);
        }
        fast._flowField.update = () => assert.fail('unused native proxy field update');
        fast._flowField.sampleInto = () => assert.fail('unused native proxy resampling');
        const edge = system.flowNetwork.edges[0], solver = system.localSolver;
        for (let step = 0; step < 12; step++) {
            system.simulationTimeSeconds = system.flowNetwork.time = step / 30;
            edge.massMg.forEach((_, cell) => {
                edge.massMg[cell] = edge.volumes[cell] * .3 * Math.max(0, Math.sin(step * .5 + cell * .7));
            });
            solver.count = step < 8 ? 2 : 0;
            for (let i = 0; i < solver.count; i++) {
                solver.positionY[i] = 2 - i * 4;
                solver.jetVelocityY[i] = -20;
                solver.iodineMassMg[i] = .01 * (step + 1);
                solver.travelDistanceMm[i] = 2;
                solver.handoffDistanceMm[i] = 8;
                solver.sourceEdgeIndex[i] = i ? -1 : 0;
                solver.sourceCellIndex[i] = i ? -1 : 0;
                solver.preferredHandoffEdgeIndex[i] = -1;
            }
            edge.renderExcluded = step === 10;
            const originalMass = edge.massMg.slice();
            for (const debug of [false, true, false]) {
                fast.setDebugMode(debug); reference.setDebugMode(debug);
                fast.update({ reuseUnchanged: true }); reference.update({ reuseUnchanged: true });
                compareNative(fast, reference);
                assert.deepEqual(fast._flowCellLocalPlumeMassMg, reference._flowCellLocalPlumeMassMg);
                assert.deepEqual(fast._plumeArrays, reference._plumeArrays);
                assert.equal(fast.plumeMesh.visible, reference.plumeMesh.visible);
                assert.equal(fast.flowMesh.visible, false);
            }
            assert.deepEqual(edge.massMg, originalMass, 'display preparation cannot alter transported iodine');
        }
        assert.equal(fast._flowSampleTexture.version, textureVersion);
        assert.equal(fast.flowMesh.geometry.attributes.flowConcentration.version, concentrationVersion);
        assert.ok(reference._flowSampleTexture.version > textureVersion);
    } finally { f.dispose(); }
});

test('sampling synchronized contrast on acquisition pulses preserves the exact current iodine field', () => {
    const f = fixture(), { system, fast, reference } = f;
    try {
        reference._needsFlowLumenUpdate = fast._needsFlowLumenUpdate;
        for (let frame = 1; frame <= 60; frame++) {
            system.simulationTimeSeconds = system.flowNetwork.time = frame / 60;
            const edge = system.flowNetwork.edges[0];
            for (let cell = 0; cell < edge.cellCount; cell++)
                edge.massMg[cell] = edge.volumes[cell] * .3 * Math.exp(-(((cell - frame * .12) / 2) ** 2));
            reference.update({ reuseUnchanged: true });
            if (frame % 4 === 0) {
                fast.update({ reuseUnchanged: true });
                compareNative(fast, reference, false);
            }
        }
    } finally { f.dispose(); }
});

test('graft remodeling keeps the existing tube concentration preparation available', () => {
    const f = fixture(), { system, fast } = f;
    try {
        assert.equal(fast._needsFlowLumenUpdate(null), false);
        system.flowNetwork.edges[0].massMg.fill(.01);
        system.flowNetwork.stentGraftRemodeling = {
            surface: { revision: 1, parts: [], sealed: false, openGate: false }, trapped: new Map()
        };
        system.simulationTimeSeconds = 1;
        fast.update({ reuseUnchanged: true });
        assert.equal(fast._flowLumenPrepared, true);
        assert.ok(fast._flowConcentrationSampleValue.some(value => value > 0));
    } finally { f.dispose(); }
});
