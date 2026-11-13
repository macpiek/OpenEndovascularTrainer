import test from 'node:test';
import assert from 'node:assert/strict';
import {readFileSync} from 'node:fs';
import * as THREE from 'three';
import {decodeCollisionAsset} from '../src/physics/collision/collisionAssetFormat.js';
import {HybridContrastSystem} from '../src/contrast/hybridContrastSystem.js';
import {AnatomicalContrastSurface, buildAnatomicalContrastSurface} from '../src/contrast/anatomicalContrastSurface.js';
import {createFlowColumnIntegral} from '../src/contrast/flowColumnIntegral.js';
import {createFlowColumnGeometry} from '../src/contrast/flowColumnGeometry.js';
import {createFlowJunctionProjection} from '../src/contrast/flowJunctionProjection.js';
import {ContrastFlowNetwork} from '../src/contrast/flowNetwork.js';
import {reattachArchOstia} from '../src/contrast/archOstiumReattachment.js';

function fixture() {
    // This is the default application atlas, including its native renal beds.
    const bytes = readFileSync(new URL('../res/Aorta_plain_renal.collision.bin', import.meta.url));
    const asset = decodeCollisionAsset(bytes.buffer.slice(bytes.byteOffset, bytes.byteOffset + bytes.byteLength));
    const data = asset.arrays.centerlineSegments, nodes = asset.arrays.centerlineEdges, stride = asset.metadata.centerline.stride;
    const segments = Array.from({length: nodes.length / 2}, (_, id) => ({
        id, start: new THREE.Vector3().fromArray(data, id * stride),
        end: new THREE.Vector3().fromArray(data, id * stride + 3),
        radiusStart: data[id * stride + 6], radiusEnd: data[id * stride + 7], safeRadius: data[id * stride + 8],
        nodeStartId: nodes[id * 2], nodeEndId: nodes[id * 2 + 1]
    }));
    for (const renal of asset.metadata.renal?.segments ?? []) Object.assign(segments[renal.index], {
        renalSide: renal.side, renalSector: renal.sector, renalHilar: renal.hilar ?? false,
        renalLateral: renal.lateral ?? false, source: 'renal-mesh'
    });
    const sourcePositions = segments.map(segment => [...segment.start.toArray(), ...segment.end.toArray()]);
    const port = {kind: 'berenstein-end', position: new THREE.Vector3(), direction: new THREE.Vector3(),
        radiusMm: .48, areaMm2: Math.PI * .48 ** 2, weight: 1, valid: true};
    const system = new HybridContrastSystem({centerlineSegments: segments,
        catheter: {type: 'berenstein', getInjectionPorts(out) {out.length = 0; out.push(port); return out;}},
        localOptions: {randomSeed: 1212}});
    const network = system.flowNetwork;
    let count = 0;
    const offsets = Uint32Array.from(network.edges, edge => {const at = count; count += edge.cellCount; return at;});
    const geometry = new THREE.BufferGeometry().setAttribute('position', new THREE.Float32BufferAttribute([], 3));
    const surface = new AnatomicalContrastSurface(network, offsets, {fallbackGeometry: geometry}, {projection: 'surface'});
    const built = buildAnatomicalContrastSurface(network, {fallbackGeometry: geometry}, surface.field, {projection: 'column'});
    const integral = createFlowColumnIntegral(network, surface.field, built.territories, built.continuations);
    const spatial = createFlowColumnGeometry(network, integral, built.territories, built.continuations);
    const projection = createFlowJunctionProjection(network, integral, built.territories, spatial, built.junctionField);
    const plume = new Float32Array(count), stock = system.medium.iodineMgPerMl / 1000;
    return {system, network, port, stock, segments, sourcePositions,
        image() {
            surface.update(plume, stock, false);
            integral.update(surface.current); projection.shared.update(surface.current);
        },
        atEdge(index) {
            const edge = network.edges[index], path = built.territories[index];
            const point = edge.start.clone().lerp(edge.end, .5), s = integral.edgeStart[index] + .5 * edge.length;
            // average() respects whether the point actually has junction support,
            // unlike calling a junction-only helper in a distant straight artery.
            return projection.average(path, s, path, s, point, point);
        },
        dispose() {projection.dispose(); spatial.dispose(); integral.dispose(); built.geometry.dispose(); surface.dispose(); geometry.dispose();}
    };
}

function descendants(network, root) {
    const result = [root];
    for (let i = 0; i < result.length; i++) result.push(...network.edges[result[i]].childEdgeIndices);
    return result;
}

function mass(network, indices) {
    let total = 0;
    for (const index of indices) for (const value of network.edges[index].massMg) total += value;
    return total;
}

test('recorded arch catheter jet supplies only physically reached connected origins and washes out downstream', () => {
    const f = fixture();
    try {
        // Reconstructed world port from the Nov 12 shared-axis export: the
        // solver's exported tip coordinates require adding its rebase origin.
        f.port.position.set(20.797064451630812, 51.21208796661758, -14.132795969281586);
        f.port.direction.set(-.381571375, .178985373, .906844817).normalize();
        assert.equal(f.system.startInjection({source: 'catheter', volumeMl: 10, rateMlPerSec: 12.1}).ok, true);
        const repairs = new Map(f.network.archOstiumReattachmentDiagnostics.repairs.map(repair => [repair.originalRootEdgeIndex, repair]));
        assert.equal(repairs.size, 3);
        // The narrow upstream jet can reach the neighbouring carotid and the
        // tail of the brachiocephalic common volume without reversing bulk
        // blood flow. The earlier technical arch territory remains unreached.
        const upstream = descendants(f.network, 27);
        const downstream = descendants(f.network, repairs.get(64).rootEdgeIndex);
        const leaves = downstream.filter(index => f.network.edges[index].childEdgeIndices.length === 0);
        let peakDownstreamMass = 0, peakDistalConcentration = 0, peakDownstreamImage = 0;
        let firstBrachiocephalicArrival = Infinity, peakBrachiocephalicMass = 0;
        const brachiocephalic = descendants(f.network, repairs.get(45).rootEdgeIndex);
        for (let frame = 1; frame <= 600; frame++) {
            f.system.advanceSynchronized(1 / 60);
            assert.equal(mass(f.network, upstream), 0, 'The jet has not reached the more proximal arch territory');
            const localMass = mass(f.network, brachiocephalic);
            if (localMass > 0) firstBrachiocephalicArrival = Math.min(firstBrachiocephalicArrival, frame);
            peakBrachiocephalicMass = Math.max(peakBrachiocephalicMass, localMass);
            peakDownstreamMass = Math.max(peakDownstreamMass, mass(f.network, downstream));
            for (const index of leaves) {
                const edge = f.network.edges[index], cell = edge.cellCount - 1;
                peakDistalConcentration = Math.max(peakDistalConcentration, edge.massMg[cell] / edge.volumes[cell] / f.stock);
            }
            if ([12, 30, 54, 90, 180, 600].includes(frame)) {
                f.image();
                peakDownstreamImage = Math.max(peakDownstreamImage, f.atEdge(270));
            }
        }
        const metrics = f.system.getMetrics();
        assert.equal(metrics.lastSourceEdgeIndex, 134);
        assert.ok(Math.abs(metrics.lastSourceDistanceMm - 6.67014206448781) < 1e-5);
        assert.ok(Math.abs(metrics.lastDirectionAgainstFlow + .8521101283262006) < 1e-6);
        assert.equal(metrics.continuousFlowSplit.maximumReversedEdgeCount, 0);
        assert.ok(firstBrachiocephalicArrival >= 2, 'The jet must travel about19mm before its tail reaches the first ostium');
        assert.ok(peakBrachiocephalicMass > 0 && peakBrachiocephalicMass < 30,
            'Only the small entrained jet tail reaches the earlier ostium, not a bulk reflux column');
        assert.ok(peakDownstreamMass > 200, 'The downstream subclavian receives a substantial transported bolus');
        assert.ok(peakDistalConcentration > .03, 'Iodine reaches distal native outlets rather than ending at the ostium');
        assert.ok(peakDownstreamImage > .03, 'Transported distal iodine remains visible');
        assert.ok(mass(f.network, downstream) < .001, 'The downstream branch clears after passage of the bolus');
        assert.ok(Math.abs(metrics.balanceErrorMg) < 1e-7, 'Mixing and washout conserve iodine mass');
        assert.ok(f.atEdge(270) < 1e-8, 'The image clears with the transported bolus');
    } finally {f.dispose();}
});

test('an injection proximal to the arch origins really supplies and renders the same distal branches', () => {
    const f = fixture();
    try {
        const parent = f.network.edges[25];
        f.port.position.copy(parent.start).lerp(parent.end, .4); f.port.direction.copy(parent.axis);
        assert.equal(f.system.startInjection({source: 'catheter', volumeMl: 10, rateMlPerSec: 12.1}).ok, true);
        for (let frame = 0; frame < 54; frame++) {
            f.system.advanceSynchronized(1 / 60);
        }
        f.image();
        for (const index of [204, 205, 270]) {
            const edge = f.network.edges[index];
            assert.ok(edge.meanConcentrationMgPerMm3 / f.stock > .01, `Contrast arrives in distal branch ${index}`);
            assert.ok(f.atEdge(index) > .01, `The same distal branch ${index} is visible once it contains iodine`);
        }
        assert.ok(Math.abs(f.system.getMetrics().balanceErrorMg) < 1e-7);
    } finally {f.dispose();}
});

test('selective distal carotid injection cannot cross-fill the other physical arch branches', () => {
    const f = fixture();
    try {
        const selected = f.network.edges[205];
        f.port.position.copy(selected.start).lerp(selected.end, .5); f.port.direction.copy(selected.axis);
        assert.equal(f.system.startInjection({source: 'catheter', volumeMl: 3, rateMlPerSec: 2}).ok, true);
        for (let frame = 0; frame < 90; frame++) f.system.advanceSynchronized(1 / 60);
        const repairs = f.network.archOstiumReattachmentDiagnostics.repairs;
        for (const repair of repairs.filter(repair => repair.originalRootEdgeIndex !== 46))
            assert.equal(mass(f.network, descendants(f.network, repair.rootEdgeIndex)), 0);
        assert.ok(mass(f.network, descendants(f.network, selected.index)) > 10);
        f.image(); assert.ok(f.atEdge(270) < 1e-8); assert.ok(f.atEdge(204) < 1e-8);
        assert.ok(Math.abs(f.system.getMetrics().balanceErrorMg) < 1e-7);
    } finally {f.dispose();}
});

for (const phase of [0, .31]) test(`recorded source44 jet crosses its real ostium instead of stopping at the source cell (phase ${phase})`, () => {
    const f = fixture();
    try {
        f.network.time = phase;
        f.port.position.set(18.39336014451009, 50.51634444100875, 5.364111258641297);
        f.port.direction.set(-.3321058793729568, -.5767459806294072, .7463710596705493);
        assert.equal(f.system.startInjection({source: 'catheter', volumeMl: 10, rateMlPerSec: 12.1}).ok, true);
        let parentPeak = 0, daughterPeak = 0, balancePeak = 0;
        const branch = descendants(f.network, 135);
        for (let frame = 1; frame <= 180; frame++) {
            f.system.advanceSynchronized(1 / 60);
            parentPeak = Math.max(parentPeak, mass(f.network, [41]));
            daughterPeak = Math.max(daughterPeak, mass(f.network, branch));
            balancePeak = Math.max(balancePeak, Math.abs(f.system.getMetrics().balanceErrorMg));
        }
        const metrics = f.system.getMetrics();
        assert.equal(metrics.lastSourceEdgeIndex, 44);
        assert.ok(Math.abs(metrics.lastSourceDistanceMm - 12.846485366582485) < 1e-6);
        assert.ok(parentPeak > 5, 'Physically upstream jet iodine reaches the aortic volume before this fork');
        assert.ok(daughterPeak > 100, 'The reached ostium supplies the connected brachiocephalic artery');
        assert.ok(balancePeak < 1e-7, 'Crossing a fork transfers each parcel mass once');
    } finally {f.dispose();}
});

test('a nearby connected artery without a proven intraluminal connector is never reattached', () => {
    const points = [[0, 0, 0], [0, 15, 0], [0, 30, 0], [0, -200, 0],
        [1, 16, 0], [2, 30, 0], [12, 42, 0], [20, 100, 0]];
    const links = [[0, 1, 12], [1, 2, 10], [2, 3, 10], [1, 4, 5], [4, 5, 5], [5, 6, 4], [6, 7, 3]];
    const segments = links.map(([a, b, radius], id) => ({id, start: new THREE.Vector3(...points[a]),
        end: new THREE.Vector3(...points[b]), nodeStartId: a, nodeEndId: b, radiusStart: radius, radiusEnd: radius}));
    const network = new ContrastFlowNetwork(segments, {rootPoint: new THREE.Vector3(0, 0, 0)});
    const topology = network.edges.map(edge => [edge.startNodeId, edge.endNodeId, edge.parentEdgeIndex, ...edge.start.toArray()]);
    assert.equal(network.archOstiumReattachmentDiagnostics.repairs.length, 0);
    assert.equal(reattachArchOstia(network).repairs.length, 0);
    assert.deepEqual(network.edges.map(edge => [edge.startNodeId, edge.endNodeId, edge.parentEdgeIndex, ...edge.start.toArray()]), topology);
    assert.deepEqual(segments.map(segment => segment.start.toArray()), links.map(([a]) => points[a]),
        'Flow topology changes must never mutate input anatomy/collision segments');
});

test('physical arch attachments preserve a conservative ordered tree without phantom outlets or anatomy edits', () => {
    const f = fixture();
    try {
        const {network} = f, diagnostics = network.archOstiumReattachmentDiagnostics;
        assert.deepEqual(diagnostics.repairs.map(repair => repair.originalRootEdgeIndex), [45, 46, 64]);
        assert.ok(diagnostics.repairs.every(repair => repair.mainDistanceMm > repair.originalMainDistanceMm + 5));
        const order = new Map(network.nodeOrder.map((id, i) => [id, i]));
        for (const edge of network.edges) {
            if (edge.transportExcluded) continue;
            assert.ok(order.get(edge.startNodeId) < order.get(edge.endNodeId), 'Hydraulics must visit the physical parent before its child');
            assert.ok(network.nodes.get(edge.startNodeId).childEdgeIndices.includes(edge.index));
            const childFlow = edge.childEdgeIndices.reduce((sum, index) => sum + network.edges[index].meanFlowMm3PerS, 0);
            if (edge.childEdgeIndices.length) assert.ok(Math.abs(childFlow - edge.meanFlowMm3PerS) < 1e-7,
                'An ostium divides actual blood flow without duplicating an aortic supply');
        }
        for (const index of diagnostics.retiredEdgeIndices) {
            const edge = network.edges[index];
            assert.equal(edge.meanFlowMm3PerS, 0);
            assert.equal(edge.transportExcluded, true);
            assert.notEqual(network.nodes.get(edge.endNodeId).nativeOutlet, true,
                'An emptied technical prefix cannot become a new unresolved tissue bed');
        }
        assert.deepEqual(f.segments.map(segment => [...segment.start.toArray(), ...segment.end.toArray()]), f.sourcePositions,
            'Only cloned flow segments change; anatomical/collision centerlines remain byte-for-byte unchanged');
    } finally {f.dispose();}
});
