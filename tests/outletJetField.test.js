import test from 'node:test';
import assert from 'node:assert/strict';
import {DataTexture, Vector3} from 'three';
import {createOutletJetField, MAX_OUTLET_JET_KERNELS, outletJetGLSL} from '../src/contrast/outletJetField.js';
import {ContrastFlowNetwork} from '../src/contrast/flowNetwork.js';
import {LocalContrastInjectionSolver, DEFAULT_CONTRAST_MEDIUM} from '../src/contrast/localInjectionSolver.js';
import {ContinuousInjectionFlowCoupler} from '../src/contrast/continuousInjectionFlowCoupler.js';

function fixture(count = 1, paths = 1) {
    const network = {edges: Array.from({length: paths}, (_, index) => ({index, cellCount: 4, radiusStart: 8, radiusEnd: 8}))};
    const solver = {count, resolveSourceHandoffLocation(i, out) {
        return Object.assign(out, {edgeIndex: this.sourceEdgeIndex[i], cellIndex: this.sourceCellIndex[i]});
    }};
    for (const name of ['positionX', 'positionY', 'positionZ', 'jetVelocityX', 'jetVelocityY', 'jetVelocityZ', 'outletDirectionX', 'outletDirectionY', 'outletDirectionZ',
        'outletRadiusMm', 'sourceEdgeIndex', 'sourceCellIndex', 'iodineMassMg', 'ageSeconds', 'travelDistanceMm']) solver[name] = new Float64Array(count);
    for (let i = 0; i < count; i++) {
        solver.iodineMassMg[i] = .075; solver.sourceEdgeIndex[i] = i % paths; solver.sourceCellIndex[i] = 2;
        solver.jetVelocityY[i] = 100; solver.outletDirectionY[i] = 1; solver.outletRadiusMm[i] = .35;
    }
    const field = createOutletJetField(network, Int32Array.from(network.edges, e => e.index));
    return {field, solver, network};
}

test('a local outlet has a filled smooth centre, carries its own mass, and never paints another lumen', () => {
    const {field, solver} = fixture();
    field.update(solver, .3);
    assert.equal(field.count, 1); assert.equal(field.massMg, solver.iodineMassMg[0]);
    const centre = field.sample([0, 0, 0], 0);
    assert.ok(centre > field.sample([.15, 0, 0], 0));
    assert.ok(field.sample([.15, 0, 0], 0) > field.sample([.3, 0, 0], 0));
    assert.equal(field.sample([0, 0, 0], 1), 0);
    assert.equal(field.integrate([0, 0, -10], [0, 0, 1], 20, 1), 0);
    assert.ok(field.integrate([0, 0, -10], [0, 0, 1], 20, 0) > 0);
    assert.ok(field.bounds.containsPoint(new Vector3(0, 0, 0)));
});

test('a jet crossing a common fork has one analytic field across wall labels without entering a remote crossing artery', () => {
    const {solver, network} = fixture(1, 5);
    const coordinates = [
        [[0, 30, 0], [0, 0, 0], 4], [[0, 0, 0], [20, -20, 0], 0],
        [[0, 0, 0], [-20, -20, 0], 0], [[0, 30, 0], [0, 0, 0], 4],
        [[0, 100, 0], [0, 30, 0], -1]
    ];
    network.edges.forEach((edge, i) => Object.assign(edge, {
        start: new Vector3(...coordinates[i][0]), end: new Vector3(...coordinates[i][1]), parentEdgeIndex: coordinates[i][2]
    }));
    const field = createOutletJetField(network, Int32Array.from(network.edges, e => e.index));
    const oldHistory = new Float32Array([.2, .1, 0, 0]);
    const history = new DataTexture(oldHistory, 1, 1);
    field.attachPathMembership(history);
    field.update(solver, .3);
    const centre = field.sample([0, 0, 0], 0), ray = field.integrate([0, 0, -3], [0, 0, 1], 6, 0);
    assert.ok(centre > 0 && ray > 0);
    for (const wall of [1, 2]) {
        assert.equal(field.sample([0, 0, 0], wall), centre, 'A boundary triangle cannot erase the common jet');
        assert.equal(field.integrate([0, 0, -3], [0, 0, 1], 6, wall), ray, 'Keep the exact same Gaussian ray integral');
    }
    assert.equal(field.sample([0, 0, 0], 3), 0,
        'The nearby crossing artery joins upstream outside the kernel support and cannot borrow the jet');
    assert.equal(field.sample([0, 0, 0], 4), 0);
    const offset = field.uniforms.outletJetPathOffset.value, records = history.image.data;
    const first = records[offset * 4], count = records[offset * 4 + 1];
    assert.deepEqual(new Set(records.slice(first * 4, first * 4 + count)), new Set([0, 1, 2]));
    assert.deepEqual(Array.from(records.slice(0, 4)), Array.from(oldHistory), 'Existing scalar history stays intact');
    for (const flag of ['transportExcluded', 'renderExcluded', 'lumenMappingExcluded']) {
        network.edges[1][flag] = true; field.update(solver, .3);
        assert.equal(field.sample([0, 0, 0], 1), 0, 'A newly excluded daughter must not reappear through jet sharing');
        network.edges[1][flag] = false;
    }
    history.dispose();
});

test('an oblique fork outside anisotropic kernel support is not admitted by its bounding sphere', () => {
    const {solver, network} = fixture(1, 2);
    const fork = new Vector3(100, 100, 0);
    Object.assign(network.edges[0], {start: new Vector3(0, 0, 0), end: fork, parentEdgeIndex: -1,
        radiusStart: 138.7, radiusEnd: 138.7});
    Object.assign(network.edges[1], {start: fork, end: new Vector3(120, 120, 0), parentEdgeIndex: 0});
    const field = createOutletJetField(network, new Int32Array([0, 1]));
    field.update(solver, .3);
    // Six-sigma ellipse semiaxes2 and3: its true distance from the fork is
    // ~138.88, although its enclosing sphere comes within138.42. The exact
    // local support test must reject a common volume of radius138.7.
    field.uniforms.outletJetAxis.value[0].set(0, 1, 0, 4);
    field.uniforms.outletJetShape.value[0].set(9, 0, 0, 9);
    field.syncPathMembership();
    assert.equal(field.sample([0, 0, 0], 1), 0);
    assert.ok(field.sample([0, 0, 0], 0) > 0);
});

test('analytic projected iodine agrees with independent quadrature and conserves all-space volume', () => {
    const {field, solver} = fixture();
    solver.travelDistanceMm[0] = 5;
    field.update(solver, .3);
    for (const angle of [0, .3, Math.PI / 2]) {
        const ray = new Vector3(0, Math.sin(angle), Math.cos(angle));
        const entry = ray.clone().multiplyScalar(-5).add(new Vector3(.1, .07, 0));
        let expected = 0;
        const steps = 10000, step = 10 / steps;
        for (let i = 0; i < steps; i++) expected += step * field.sample(entry.clone().addScaledVector(ray, (i + .5) * step), 0);
        const actual = field.integrate(entry, ray, 10, 0);
        assert.ok(Math.abs(actual - expected) < 2e-7, `${angle}: ${actual} vs ${expected}`);
    }
    // Integrate the detector plane; Fubini's theorem gives the total iodine
    // volume independently of the moment/normalization implementation.
    const k = field.kernels[0], sx = 6 / Math.sqrt(k.inverseRadialVariance), sy = 6 / Math.sqrt(k.inverseAxialVariance), n = 180;
    const dx = 2 * sx / n, dy = 2 * sy / n;
    let volume = 0;
    for (let y = 0; y < n; y++) for (let x = 0; x < n; x++)
        volume += dx * dy * field.integrate([-sx + (x + .5) * dx, -sy + (y + .5) * dy, -sx], [0, 0, 1], 2 * sx, 0);
    assert.ok(Math.abs(volume - solver.iodineMassMg[0] / .3) < 2e-7, `${volume}`);
});

test('projection rotates with the jet, stays positive and clips at the anatomical interval', () => {
    const a = fixture(), b = fixture();
    for (const f of [a, b]) f.solver.travelDistanceMm[0] = 2;
    a.field.update(a.solver, .3);
    b.solver.jetVelocityY[0] = 0; b.solver.jetVelocityX[0] = 100;
    b.field.update(b.solver, .3);
    assert.ok(Math.abs(a.field.integrate([.1, -.2, -10], [0, 0, 1], 20, 0) -
        b.field.integrate([-.2, .1, -10], [0, 0, 1], 20, 0)) < 1e-10);
    const whole = a.field.integrate([0, 0, -10], [0, 0, 1], 20, 0);
    const half = a.field.integrate([0, 0, -10], [0, 0, 1], 10, 0);
    assert.ok(half > 0 && Math.abs(2 * half - whole) < 1e-8);
    assert.equal(a.field.integrate([0, 0, -10], [0, 0, 1], 0, 0), 0);
    assert.equal(a.field.integrate([10, 0, -10], [0, 0, 1], 20, 0), 0);
});

test('aggregation retains all iodine within the uniform budget, including disconnected-source overflow', () => {
    const {field, solver} = fixture(300, 6);
    for (let i = 0; i < solver.count; i++) {
        solver.positionY[i] = i % 30; solver.ageSeconds[i] = (i % 14) * .01; solver.travelDistanceMm[i] = i % 30;
    }
    field.update(solver, .3);
    assert.ok(field.count <= MAX_OUTLET_JET_KERNELS);
    assert.ok(Math.abs(field.massMg - solver.iodineMassMg.reduce((a, b) => a + b, 0)) < 1e-10);
    assert.equal(field.fallbackMassByCell.reduce((a, b) => a + b, 0), 0);
    const overflow = fixture(40, 40); overflow.field.update(overflow.solver, .3);
    assert.equal(overflow.field.count, MAX_OUTLET_JET_KERNELS);
    const fallback = overflow.field.fallbackMassByCell.reduce((a, b) => a + b, 0);
    assert.ok(Math.abs(overflow.field.massMg + fallback - overflow.field.totalMassMg) < 1e-10);
    for (let edge = MAX_OUTLET_JET_KERNELS; edge < 40; edge++)
        assert.equal(overflow.field.fallbackMassByCell[edge * 4 + 2], overflow.solver.iodineMassMg[edge]);
    assert.equal(overflow.field.sample([0, 0, 0], 39), 0);
});

test('age-bin transitions are continuous and zero mass clears every GPU/CPU state', () => {
    const {field, solver} = fixture(2);
    solver.positionY[1] = 2;
    const values = [];
    for (const age of [.012 - 1e-7, .012, .012 + 1e-7]) {
        solver.ageSeconds[0] = age; solver.ageSeconds[1] = age + .006;
        field.update(solver, .3); values.push(field.sample([0, .4, 0], 0));
    }
    assert.ok(Math.max(...values) - Math.min(...values) < 1e-4, values.join(','));
    field.update(null, .3);
    assert.equal(field.count, 0); assert.equal(field.active, false); assert.equal(field.massMg, 0);
    assert.equal(field.uniforms.outletJetCount.value, 0); assert.equal(field.bounds.isEmpty(), true);
    assert.equal(field.integrate([0, 0, -10], [0, 0, 1], 20, 0), 0);
    assert.ok(!outletJetGLSL.includes('sampler'), 'The field must not consume another fragment texture unit');
});

test('dynamic lumen exclusions suppress local kernels without discarding pending mass', () => {
    const {field, solver, network} = fixture();
    for (const flag of ['renderExcluded', 'transportExcluded', 'lumenMappingExcluded']) {
        field.update(solver, .3);
        assert.ok(field.active);
        network.edges[0][flag] = true;
        field.update(solver, .3);
        assert.equal(field.count, 0, `${flag}: a pending jet must not resurrect the hidden lumen`);
        assert.ok(field.bounds.isEmpty());
        assert.equal(field.integrate([0, 0, -10], [0, 0, 1], 20, 0), 0);
        assert.equal(field.fallbackMassByCell[2], solver.iodineMassMg[0]);
        assert.equal(field.massMg + field.fallbackMassByCell.reduce((a, b) => a + b, 0), field.totalMassMg);
        network.edges[0][flag] = false;
    }
    field.update(solver, .3);
    assert.ok(field.active, 'restoring the lumen restores its existing local mass');
    assert.ok(field.fallbackMassByCell.every(mass => mass === 0));
});

test('a parcel already in an eligible continuation cannot repaint its excluded source territory', () => {
    const {field, solver, network} = fixture(1, 2);
    solver.resolveSourceHandoffLocation = (i, out) => Object.assign(out, {edgeIndex: 1, cellIndex: 2});
    network.edges[0].renderExcluded = true;
    field.update(solver, .3);
    assert.equal(field.count, 1);
    assert.equal(field.massMg, solver.iodineMassMg[0]);
    assert.equal(field.sample([0, 0, 0], 0), 0);
    assert.ok(field.sample([0, 0, 0], 1) > 0);
    assert.equal(field.kernels[0].pathA, 1);
    assert.equal(field.kernels[0].pathB, 1);
});

test('the resolved jet preserves Q = v A at clinical injection rates without a super-concentrated core', () => {
    for (const radius of [3, 8]) for (const rate of [2, 12.1, 15]) {
        const network = new ContrastFlowNetwork([{id: 0, nodeStartId: 0, nodeEndId: 1,
            start: new Vector3(0, 70, 0), end: new Vector3(0, -70, 0), radiusStart: radius, radiusEnd: radius}], {cellLengthMm: .75});
        const solver = new LocalContrastInjectionSolver({flowNetwork: network, capacity: 4096, parcelVolumeMl: .0025, randomSeed: 1212});
        const field = createOutletJetField(network, new Int32Array([0]));
        const port = {position: new Vector3(0, 35, 0), direction: new Vector3(0, -1, 0), radiusMm: .45, valid: true};
        const dt = 1 / 60;
        for (let step = 0; step < 12; step++) {
            network.setFlowOverride([0], Math.PI * radius * radius * 90);
            solver.emit({ports: [port], volumeMl: rate * dt, rateMlPerSec: rate, emissionWindowSeconds: dt,
                medium: DEFAULT_CONTRAST_MEDIUM, allowPressureDrivenRetrograde: false});
            solver.update(dt); network.update(dt);
            assert.ok(Math.abs(solver.totalEmittedIodineMassMg - solver.getIodineMassMg() - network.getIodineMassMg() - network.outletIodineMassMg) < 1e-8,
                `${radius} mm, ${rate} ml/s: widening/entrainment must conserve iodine`);
        }
        field.update(solver, .3);
        let peak = 0;
        for (let y = 24; y <= 37; y += .2) for (const x of [-.3, 0, .3]) for (const z of [-.3, 0, .3])
            peak = Math.max(peak, field.sample([x, y, z], 0));
        assert.ok(peak > .1 && peak <= 1.05, `${radius} mm, ${rate} ml/s: local peak ${peak} times stock`);
        const resolvedArea = Math.PI * solver.resolvedOutletRadiusMm[0] ** 2;
        assert.ok(Math.abs(resolvedArea * solver.lastResolvedJetSpeedMmPerSec - rate * 1000) / (rate * 1000) < 1e-6,
            'Resolved area compensates the unresolved nozzle velocity exactly');
        // Removing one packet must preserve the surviving packet's width.
        const survivor = solver.count - 1, expectedRadius = solver.resolvedOutletRadiusMm[survivor];
        const expectedTransportSpeed = solver.initialTransportSpeedMmPerSec[survivor];
        solver._removeParticle(0);
        assert.equal(solver.resolvedOutletRadiusMm[0], expectedRadius);
        assert.equal(solver.initialTransportSpeedMmPerSec[0], expectedTransportSpeed);
    }
});

test('the Q/v width starts at the physical nozzle and expands smoothly over one outlet diameter', () => {
    const {field, solver} = fixture();
    solver.resolvedOutletRadiusMm = new Float32Array([1.8]);
    let previous = 0;
    for (const travel of [0, .1, .2, .4, .7, 1]) {
        solver.travelDistanceMm[0] = travel;
        field.update(solver, .3);
        const sigma = 1 / Math.sqrt(field.kernels[0].inverseRadialVariance);
        assert.ok(sigma >= previous);
        if (travel === 0) assert.ok(Math.abs(sigma - solver.outletRadiusMm[0] / Math.SQRT2) < 1e-10);
        previous = sigma;
        assert.equal(field.massMg, solver.iodineMassMg[0], 'Widening changes volume, not iodine mass');
    }
    assert.ok(previous > 1.2);
});

test('a high-rate jet entrains at its countercurrent turning point instead of piling up above stock concentration', () => {
    for (const divergent of [false, true]) {
        const radius = divergent ? 4 : 8;
        const network = new ContrastFlowNetwork([{id: 0, nodeStartId: 0, nodeEndId: 1,
            start: new Vector3(0, 70, 0), end: new Vector3(0, -70, 0), radiusStart: radius, radiusEnd: radius}], {cellLengthMm: .75});
        network.setHemodynamics({cardiacOutputMlPerMin: Math.PI * radius * radius * 90 / 1000 * 60});
        if (divergent) network.hemodynamics.axialDispersionMm2PerS = 0;
        const solver = new LocalContrastInjectionSolver({flowNetwork: network, capacity: 4096, parcelVolumeMl: .0025, randomSeed: 1212});
        const field = createOutletJetField(network, new Int32Array([0])), coupler = new ContinuousInjectionFlowCoupler(network);
        const edge = network.edges[0], rate = 12.1, dt = 1 / 60;
        const port = {position: new Vector3(0, 35, 0), direction: new Vector3(0, 1, 0), radiusMm: .45, valid: true};
        for (let step = 0; step < 12; step++) {
            network.clearFlowOverrides(); coupler.beginStep();
            network.setFlowOverride([0], Math.PI * radius * radius * 90);
            let locations;
            if (divergent) {
                const location = network.findInjectionLocation(port.position, port.direction, {});
                network.setFlowOverride([0], -4000);
                network.addFaceFlowDelta(0, location.cellIndex + 1, edge.cellCount + 1, rate * 1000);
                locations = [location];
            } else locations = coupler.configure({ports: [port], rateMlPerSec: rate}).map(split => split.location);
            solver.emit({ports: [port], volumeMl: rate * dt, rateMlPerSec: rate, emissionWindowSeconds: dt,
                medium: DEFAULT_CONTRAST_MEDIUM, allowPressureDrivenRetrograde: false, portInjectionLocations: locations});
            solver.update(dt); network.update(dt);
            assert.ok(Math.abs(solver.totalEmittedIodineMassMg - solver.getIodineMassMg() - network.getIodineMassMg() - network.outletIodineMassMg) < 1e-8,
                'Countercurrent entrainment must retain every parcel in the local, bulk or outlet mass balance');
        }
        field.update(solver, .3);
        let peak = 0;
        for (let y = 30; y < 60; y += .2) for (const x of [-.3, 0, .3]) for (const z of [-.3, 0, .3])
            peak = Math.max(peak, field.sample([x, y, z], 0));
        if (divergent) {
            assert.equal(peak, 0, 'A pure-injection source cannot also hold a duplicate local core volume');
            assert.equal(solver.count, 0);
            assert.equal(field.active, false);
            const source = solver.lastSourceCellIndex;
            const upstream = edge.massMg.slice(0, source).reduce((sum, mass) => sum + mass, 0);
            const downstream = edge.massMg.slice(source + 1).reduce((sum, mass) => sum + mass, 0);
            assert.ok(upstream > 0 && downstream > 0);
            assert.ok(Math.abs(upstream / (upstream + downstream) - 4 / 12.1) < 1e-8,
                'Direct pure-source iodine follows the exact 4 / 8.1 ml/s hydraulic split');
            for (let cell = 0; cell < edge.cellCount; cell++)
                assert.ok(edge.massMg[cell] <= .3 * edge.volumes[cell] * (1 + 1e-7), 'Pure injected fluid must not exceed stock concentration');
        } else assert.ok(peak > .1 && peak < 1.08, `countercurrent jet peak ${peak}; original pile-up exceeded 2.4 times stock`);
        assert.ok(field.kernels.every(k => k.ageSeconds >= 0 && k.travelDistanceMm >= 0));
        assert.ok(network.getIodineMassMg() > field.massMg, 'Most iodine has mixed into the flowing blood');
        // Stopping the injection retires the remaining core without adding
        // back any iodine released when its upstream momentum disappeared.
        for (let step = 0; step < 12; step++) {solver.update(dt); network.update(dt);}
        assert.equal(solver.count, 0);
        assert.ok(Math.abs(solver.totalEmittedIodineMassMg - network.getIodineMassMg() - network.outletIodineMassMg) < 1e-8);
    }
});
