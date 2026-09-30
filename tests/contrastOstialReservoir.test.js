import test from 'node:test';
import assert from 'node:assert/strict';
import fs from 'node:fs';
import * as THREE from 'three';
import { decodeCollisionAsset } from '../src/physics/collision/collisionAssetFormat.js';
import { HybridContrastSystem } from '../src/contrast/hybridContrastSystem.js';

const bytes = fs.readFileSync(
    new URL('../res/Aorta_infrarenal_aneurysm.collision.bin', import.meta.url)
);
const asset = decodeCollisionAsset(
    bytes.buffer.slice(bytes.byteOffset, bytes.byteOffset + bytes.byteLength)
);
const data = asset.arrays.centerlineSegments;
const links = asset.arrays.centerlineEdges;
const segments = [];
for (let i = 0; i < data.length; i += 9) {
    const id = i / 9;
    segments.push({
        id,
        nodeStartId: links[id * 2],
        nodeEndId: links[id * 2 + 1],
        start: new THREE.Vector3(...data.slice(i, i + 3)),
        end: new THREE.Vector3(...data.slice(i + 3, i + 6)),
        radiusStart: data[i + 6],
        radiusEnd: data[i + 7],
        safeRadius: data[i + 8]
    });
}
function system() {
    return new HybridContrastSystem({
        centerlineSegments: segments,
        localOptions: { capacity: 16 }
    });
}
function ostia(net) {
    // The two affected side branches in the supplied aneurysm atlas. Select
    // anatomically, without depending on the flow graph's edge numbering.
    return net.aorticBranchPrefixDiagnostics.paths.filter((path) => {
        const root = net.edges[path.rootEdgeIndex];
        return root.start.y < -253 && root.start.y > -254;
    });
}
function pathMass(net, paths) {
    return paths.reduce(
        (total, path) =>
            total +
            path.edgeIndices.reduce(
                (sum, i) => sum + net.edges[i].massMg.reduce((a, b) => a + b, 0),
                0
            ),
        0
    );
}

test('aneurysm ostia use downstream branch calibre, including a repaired parent-radius valley', () => {
    const net = system().flowNetwork;
    const paths = ostia(net);
    assert.equal(paths.length, 2, 'both enlarged technical branch prefixes must be corrected');
    assert.ok(
        paths.some(
            (path) => net.edges[net.edges[path.rootEdgeIndex].parentEdgeIndex].rawRadiusEnd < 10
        ),
        'the test must cover the raw-radius valley which used to bypass correction'
    );
    for (const path of paths) {
        assert.ok(path.branchRadiusMm > 2 && path.branchRadiusMm < 4);
        assert.ok(path.removedVolumeMm3 > 3000, 'do not leave a false multi-millilitre reservoir');
        const root = net.edges[path.rootEdgeIndex];
        assert.ok(root.radiusStart <= path.branchRadiusMm * 1.35 + 1e-6);
        assert.ok(root.meanFlowMm3PerS > 0, 'the branch must remain patent');
    }
    const corrected = new Set(paths.flatMap((path) => path.edgeIndices));
    const trunk = net.edges.filter(
        (e) => e.start.y < -255 && e.start.y > -270 && e.axis.y < -0.95 && e.rawRadiusStart > 8
    );
    assert.ok(trunk.length > 20);
    for (const edge of trunk) {
        assert.ok(!corrected.has(edge.index), 'do not constrict the continuing aortic trunk');
        assert.ok(edge.radiusStart >= edge.rawRadiusStart);
    }
});

test('a finite bolus traverses the aneurysm ostia and washes out without losing iodine', () => {
    const s = system(),
        net = s.flowNetwork,
        paths = ostia(net);
    assert.equal(paths.length, 2);
    const source = net.findNearestLocation(new THREE.Vector3(3, -212, 0), {});
    let peakMass = 0;
    for (let frame = 0; frame < 360; frame++) {
        if (frame < 30) net.depositIodine(source.edgeIndex, source.cellIndex, 20);
        s.update(1 / 60);
        peakMass = Math.max(peakMass, pathMass(net, paths));
        const injected = Math.min(frame + 1, 30) * 20;
        assert.ok(Math.abs(net.totalIodineMassMg + net.outletIodineMassMg - injected) < 1e-7);
    }
    assert.ok(peakMass > 10, 'the bolus must actually enter both branches');
    assert.ok(
        pathMass(net, paths) < 0.6,
        'less than 0.1% may remain after the broadened aneurysm bolus passes (6 s)'
    );
});
