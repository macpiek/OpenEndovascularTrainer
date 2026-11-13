import test from 'node:test';
import assert from 'node:assert/strict';
import {readFileSync} from 'node:fs';
import {gunzipSync} from 'node:zlib';
import * as THREE from 'three';
import {decodeCollisionAsset} from '../src/physics/collision/collisionAssetFormat.js';
import {ContrastFlowNetwork} from '../src/contrast/flowNetwork.js';
import {FlowConcentrationField} from '../src/contrast/flowConcentrationField.js';
import {AnatomicalContrastSurface, buildAnatomicalContrastSurface} from '../src/contrast/anatomicalContrastSurface.js';
import {buildAnatomicalColumnProjection} from '../src/contrast/anatomicalColumnProjection.js';
import {createFlowColumnIntegral} from '../src/contrast/flowColumnIntegral.js';
import {createFlowColumnGeometry} from '../src/contrast/flowColumnGeometry.js';
import {createFlowJunctionProjection} from '../src/contrast/flowJunctionProjection.js';
import {opticalParentEdgeIndex} from '../src/contrast/flowOpticalTopology.js';

function archFixture() {
    const bytes = readFileSync(new URL('../res/Aorta_plain.collision.bin', import.meta.url));
    const asset = decodeCollisionAsset(bytes.buffer.slice(bytes.byteOffset, bytes.byteOffset + bytes.byteLength));
    const data = asset.arrays.centerlineSegments, nodes = asset.arrays.centerlineEdges;
    const stride = asset.metadata.centerline.stride;
    const segments = Array.from({length: data.length / stride}, (_, i) => ({
        id: i, nodeStartId: nodes[i * 2], nodeEndId: nodes[i * 2 + 1],
        start: new THREE.Vector3().fromArray(data, i * stride),
        end: new THREE.Vector3().fromArray(data, i * stride + 3),
        radiusStart: data[i * stride + 6], radiusEnd: data[i * stride + 7], safeRadius: data[i * stride + 8]
    }));
    return rendererFixture(new ContrastFlowNetwork(segments));
}

function rendererFixture(network) {
    let count = 0;
    const offsets = Uint32Array.from(network.edges, edge => {const offset = count; count += edge.cellCount; return offset;});
    const field = new FlowConcentrationField(network, offsets, count);
    // Exercise the production topology builder without duplicating its private
    // territory algorithm or loading/mapping the whole anatomical wall mesh.
    const geometry = new THREE.BufferGeometry().setAttribute('position', new THREE.Float32BufferAttribute([], 3));
    const built = buildAnatomicalContrastSurface(network, {fallbackGeometry: geometry}, field, {projection: 'column'});
    const integral = createFlowColumnIntegral(network, field, built.territories, built.continuations);
    const spatial = createFlowColumnGeometry(network, integral, built.territories, built.continuations);
    const projection = createFlowJunctionProjection(network, integral, built.territories, spatial, built.junctionField);
    const cells = new Float32Array(count);
    return {network, field, offsets, count, geometry, built, integral, spatial, projection,
        fill(value) {
            for (const edge of network.edges) for (let cell = 0; cell < edge.cellCount; cell++)
                cells[offsets[edge.index] + cell] = edge.transportExcluded ? 0 : value(edge, cell);
            field.update(cells); integral.update(field.values); projection.shared.update(field.values);
        },
        dispose() {projection.dispose(); spatial.dispose(); integral.dispose(); built.geometry.dispose(); geometry.dispose();}
    };
}

function bridgeFixture() {
    const points=[[0,0,0],[0,30,0],[0,31,0],[-20,55,0],[20,55,0]].map(p=>new THREE.Vector3(...p));
    const segments=[[0,1],[1,2],[2,3],[2,4]].map(([a,b],id)=>({id,nodeStartId:a,nodeEndId:b,
        start:points[a],end:points[b],radiusStart:5,radiusEnd:5}));
    const network=new ContrastFlowNetwork(segments,{rootPoint:points[0]});
    const bridge=network.edges[1];
    bridge.intraluminalAorticConnector=true;bridge.renderExcluded=true;bridge.lumenMappingExcluded=true;
    return rendererFixture(network);
}

test('actual arch optical ancestry follows physical ostia and excludes retired medial prefixes', () => {
    const f = archFixture();
    try {
        const {network, built, projection, spatial, integral} = f;
        const {territories} = built;
        const repairs=network.archOstiumReattachmentDiagnostics.repairs;
        assert.deepEqual(repairs.map(r=>[r.originalRootEdgeIndex,r.rootEdgeIndex,r.parentEdgeIndex]),
            [[45,135,41],[46,157,99],[64,230,170]]);
        for(const repair of repairs){
            const head=network.edges[repair.rootEdgeIndex],parent=network.edges[repair.parentEdgeIndex];
            const path=territories[head.index];
            assert.ok(path>=0&&integral.ranges.has(path),'The true branch keeps its visible transported column');
            assert.equal(opticalParentEdgeIndex(network,territories,head.index),parent.index);
            assert.equal(projection.components[path],projection.components[0]);
            assert.ok(head.start.distanceTo(parent.end)<1e-9,'The visible arm begins at its repaired ostium');
            assert.ok(spatial.candidates(path,0,0,integral.ranges.get(0).length)
                .some(candidate=>candidate.path===territories[parent.index]),'CPU rays include the physical aortic parent');
        }
        for(const edge of network.edges.filter(e=>e.retiredArchPrefix)){
            assert.equal(edge.transportExcluded,true);
            assert.equal(territories[edge.index],-1,'The retired internal strand must not acquire another optical tube');
            assert.equal(integral.ranges.has(edge.index),false);
        }
        // Every true ostium shares its local field, including the narrower
        // carotid outlet whose mixing volume scales with its own caliber.
        for(const parent of [41,99,170])assert.ok(projection.shared.indexByParent.has(parent));
        assert.equal(projection.shared.indexByParent.has(42),false,'Retired connector42 is no longer a blood-filled fork');

        const wall = new THREE.BufferGeometry()
            .setAttribute('position', new THREE.Float32BufferAttribute([0, 0, 0, 1, 0, 0, 0, 1, 0], 3))
            .setAttribute('columnCoordinate', new THREE.Float32BufferAttribute([0, 0, 0, 0, 0, 0], 2));
        const gpu = buildAnatomicalColumnProjection(network, wall, f.field, built);
        try {
            const records = gpu.uniforms.columnRanges.value.image.data;
            for (const child of repairs.map(r=>r.rootEdgeIndex)) {
                const path = territories[child], parent = opticalParentEdgeIndex(network, territories, child);
                assert.equal(records[path * 8 + 4], territories[parent]);
                assert.ok(Math.abs(records[path * 8 + 5] - integral.edgeStart[parent] - network.edges[parent].length) < 1e-5);
                assert.equal(records[path * 8 + 6], projection.components[0]);
            }
        } finally {gpu.dispose(); wall.dispose();}
    } finally {f.dispose();}
});

test('only intraluminal blood connectors bridge excluded optical ancestry', () => {
    const network = {edges: [
        {index: 0, parentEdgeIndex: -1}, {index: 1, parentEdgeIndex: 0, lumenMappingExcluded: true},
        {index: 2, parentEdgeIndex: 1}
    ]};
    const territories = new Int32Array([0, -1, 2]);
    assert.equal(opticalParentEdgeIndex(network, territories, 2), -1, 'An unmatched or excluded path remains separate');
    network.edges[1].intraluminalAorticConnector = true;
    assert.equal(opticalParentEdgeIndex(network, territories, 2), 0);
    network.edges[1].transportExcluded = true;
    assert.equal(opticalParentEdgeIndex(network, territories, 2), -1, 'An occluded connector cannot bridge blood lumens');
});

test('hidden arch connector iodine supplies the fork mixture while fabric and excluded lumens stay masked', () => {
    const f = bridgeFixture();
    const surface = new AnatomicalContrastSurface(f.network, f.offsets, {fallbackGeometry: f.geometry}, {projection: 'surface'});
    try {
        const connector = f.network.edges[1], cell = f.offsets[1], stock = .3;
        const plume = new Float32Array(f.count);
        connector.massMg[0] = connector.volumes[0] * stock;
        surface.update(plume, stock, false);
        assert.equal(surface.current[cell], 1, 'Hiding connector geometry must not erase transported iodine');
        assert.equal(surface.excludedCells[cell], 1, 'The connector remains optically excluded');
        assert.ok(surface.current[surface.field.end[1]] > .99, 'Its outgoing blood supplies the true distal fork mixture');
        f.integral.update(surface.current); f.projection.shared.update(surface.current);
        const index = f.projection.shared.indexByParent.get(1);
        assert.ok(f.projection.shared.sample(index, connector.end).value > .99,
            'Connector-only iodine illuminates the existing common lumen, without a connector tube');

        connector.graftSections = new Uint8Array(connector.cellCount).fill(1);
        surface.update(plume, stock, false, true);
        assert.equal(surface.current[cell], 0, 'A real graft fabric replacement retains its mask');
        connector.graftSections = null;
        connector.transportExcluded = true;
        surface.update(plume, stock, false);
        assert.equal(surface.current[cell], 0, 'An excluded blood compartment is not revived by the connector exception');
        connector.transportExcluded = false;
        connector.intraluminalAorticConnector = false;
        surface.update(plume, stock, false);
        assert.equal(surface.current[cell], 0, 'An ordinary render-excluded edge remains dark');
    } finally {surface.dispose(); f.dispose();}
});

test('actual arch common fields preserve uniform dilution and complete washout through all local arms', () => {
    const f = archFixture();
    try {
        for (const concentration of [.23, 0]) {
            f.fill(() => concentration);
            for (const parent of [25, 41, 84, 99, 155, 170]) {
                const index = f.projection.shared.indexByParent.get(parent), item = f.projection.shared.items[index];
                for (const point of [item.junction.center,
                    ...item.anchors.map(anchor => item.junction.center.clone().lerp(anchor, .4))]) {
                    const local = f.projection.shared.sample(index, point);
                    if (local.blend > 0) assert.ok(Math.abs(local.value - concentration) < 1e-7,
                        `${parent}: local ${local.value} must retain ${concentration}`);
                    const arm = item.arms[0];
                    const value = f.projection.pointValue(point, arm.path, arm.s, arm.path, arm.s);
                    assert.ok(Math.abs(value - concentration) < 1e-7,
                        `${parent}: full projection ${value} must retain ${concentration}, including outside this local field`);
                }
            }
        }
    } finally {f.dispose();}
});

test('overlapping physical arch ostia reconstruct one concentration independently of the wall owner', () => {
    const f = archFixture();
    try {
        for(const parent of [41,99,170]){
            const index=f.projection.shared.indexByParent.get(parent),item=f.projection.shared.items[index];
            const daughter=item.arms[2].path;
            f.fill(edge => f.built.territories[edge.index] === daughter ? .8 : .12);
            for(const offset of [[0,0,0],[-2,-2,1],[2,1,-1]]){
                const point=item.junction.center.clone().add(new THREE.Vector3(...offset));
                const values=item.arms.map(arm=>f.projection.pointValue(point,arm.path,arm.s,arm.path,arm.s));
                assert.ok(Math.max(...values)-Math.min(...values)<1e-6,
                    `Physical ostium ${parent} cannot change with triangle ownership: ${values}`);
                assert.ok(values.every(value=>value>.1&&value<=.800001),'Keep a positive, bounded mixture');
            }
        }
    } finally {f.dispose();}
});

test('the narrow physical carotid ostium has no wall-label seam and does not prefill its clear distal branch', () => {
    const f = archFixture();
    try {
        f.fill(edge => f.built.territories[edge.index] === 26 ? .2 : 0);
        // The carotid is just below the former 35% caliber cutoff. These
        // points lie inside both its proximal lumen and the aortic lumen.
        // Triangle ownership must not decide whether their blood is opaque.
        const points = [
            [22.3269176483, 46.666721344, -8.73833942413],
            [24.3269176483, 51.51254654, -4.73833942413]
        ];
        for (const coordinates of points) {
            const point = new THREE.Vector3(...coordinates);
            const values = [26, 157].map(path => f.projection.pointValue(point, path, 0, path, 0));
            assert.ok(Math.abs(values[0] - values[1]) < 1e-7,
                `One common blood volume cannot change concentration at the carotid wall seam: ${values}`);
        }
        const path = f.built.territories[205];
        const edge = f.network.edges[205], s = f.integral.edgeStart[205] + edge.length * .5;
        const point = edge.start.clone().lerp(edge.end, .5);
        assert.equal(f.projection.average(path, s, path, s, point, point), 0,
            'Continuity at the real ostium must not paint the still-clear downstream carotid');
    } finally {f.dispose();}
});

test('a remote filled arch fork cannot illuminate the still-clear proximal ostium', () => {
    const f = archFixture();
    try {
        const center = f.network.edges[41].end;
        const points = [center, center.clone().add(new THREE.Vector3(-2, -2, 2))];
        for (const source of [157,230,155,162,163]) {
            f.fill(edge => edge.index === source ? 1 : 0);
            for (const point of points) for (const owner of [26,135]) {
                assert.equal(f.projection.pointValue(point, owner, 0, owner, 0), 0,
                    `Iodine at remote edge ${source} must first traverse the intervening local cells`);
            }
        }
    } finally {f.dispose();}
});

test('recorded anatomical-wall rays stay continuous where their labels cross outside the medial-axis tubes', () => {
    const f = archFixture();
    try {
        const replay = JSON.parse(gunzipSync(readFileSync(new URL('./fixtures/arch-physical-seam.json.gz', import.meta.url))));
        const frame = replay.frames.find(frame => frame.kind === 'transport' && frame.port.name === 'source44' && frame.elapsed === .5);
        assert.ok(frame, 'Retain the actual recorded source44 bolus for this regression');
        const nativeFlow = f.network.getFaceSignedFlowMm3PerS.bind(f.network);
        f.network.getFaceSignedFlowMm3PerS = (index, face) =>
            frame.boundaryFlows[index]?.[face === 0 ? 0 : 1] ?? nativeFlow(index, face);
        f.fill((edge, cell) => edge.index < 300 ? frame.mass[edge.index][cell] / edge.volumes[cell] / .3 : 0);
        // Adjacent LAO30 detector pixels75,90 and75,91, intersected against
        // the actual STL. Their smooth chords cross the135→26 surface seam.
        const rays = [
            {a: [8.48425046, 49.76308441, 13.02265647], b: [2.29174146, 49.76308441, 2.29691626],
                paths: [135, 26], s: [15.232161, 6.979785]},
            {a: [8.51798584, 49.60683441, 13.08108787], b: [2.25651804, 49.60683441, 2.23590750],
                paths: [26, 26], s: [3.222338, 6.913085]}
        ];
        const means = [];
        for (const ray of rays) {
            const entry = new THREE.Vector3(...ray.a), exit = new THREE.Vector3(...ray.b);
            means.push(f.projection.average(ray.paths[0], ray.s[0], ray.paths[1], ray.s[1], entry, exit));
            let previous;
            for (let i = 0; i <= 100; i++) {
                const point = entry.clone().lerp(exit, i / 100);
                const values = [26, 135].map(path => f.projection.pointValue(point, path, ray.s[0], path, ray.s[1]));
                assert.ok(Math.abs(values[0] - values[1]) < 1e-7,
                    `Actual blood outside approximate capsules must not inherit its triangle label: ${values}`);
                if (previous !== undefined) assert.ok(Math.abs(values[0] - previous) < .005,
                    'The fallback must blend continuously when crossing the approximate tube boundary');
                previous = values[0];
            }
        }
        assert.ok(Math.abs(means[0] - means[1]) < .005,
            `The actual neighboring rays must not retain the old .236→.111 concentration jump: ${means}`);
    } finally {f.dispose();}
});
