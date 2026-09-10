import assert from 'node:assert/strict';
import test from 'node:test';
import {
    buildKirchhoffCompositeTopology as build, compositeToolFromTipProfile,
    sampleKirchhoffCompositeSection as sample, evaluateKirchhoffCompositeSection as evaluate,
    condenseKirchhoffCompositeSection as condense
} from '../src/physics/kirchhoffCompositeTopology.js';
import { evaluateFullBundleSection } from '../src/physics/kirchhoffBundleModel.js';
import { kirchhoffMaterialProfile } from '../src/physics/kirchhoffMaterialProfile.js';
import {
    STEEL_J_GUIDEWIRE_TIP_CORE_LENGTH_MM, STEEL_J_GUIDEWIRE_TIP_TRANSITION_LENGTH_MM,
    STEEL_J_GUIDEWIRE_CURVED_TIP_LENGTH_MM, STEEL_J_GUIDEWIRE_CURVATURE_TRANSITION_MM
} from '../src/physics/guidewireMaterialProfile.js';
import { PIGTAIL_NATURAL_ARC_LENGTH_MM, PIGTAIL_CURVATURE_TRANSITION_MM } from '../src/physics/catheterMaterialProfile.js';
import { GUIDEWIRE_RADIUS_MM, PIGTAIL_CATHETER_RADIUS_MM, PIGTAIL_CATHETER_INNER_RADIUS_MM,
    INTRODUCER_SHEATH_INNER_RADIUS_MM } from '../src/toolDimensions.js';

const close = (a, b, tol = 1e-10) => assert.ok(Math.abs(a - b) <= tol * Math.max(1, Math.abs(a), Math.abs(b)), `${a} != ${b}`);
const closeVector = (a, b, tol) => a.forEach((v, i) => close(v, b[i], tol));
const derivative = (fn, x, h = 1e-5) => (fn(x + h) - fn(x - h)) / (2 * h);
const material = (EI = 10, GJ = 4, kappa0 = [0, 0]) => ({ EI1: EI, EI2: EI, GJ, kappa0 });
const tool = (insertion, more = {}) => ({ insertion, materialInterval: [0, 100], material: material(), ...more });
const shape = topology => topology.sections.map(s => [s.start, s.end, s.kind]);
const sectionAt = (topology, x) => topology.sections.find(s => x >= s.start && x < s.end);
const energyIntegral = (topology, options) => topology.sections.reduce((sum, s) => sum + (s.end - s.start) * evaluate(s, options).energy, 0);

test('one chain partitions exact independent tips, material interfaces and sheath ownership', () => {
    const topology = build({
        wire: tool(80, { materialInterval: [100, 200], materialBreakpoints: [180], radius: .4 }),
        catheter: tool(40, { materialInterval: [10, 70], materialBreakpoints: [55], radius: .8, innerRadius: .5 }),
        sheath: { id: 'introducer', interval: [0, 10], innerRadius: .9 }, boundaries: [30]
    });
    assert.deepEqual(shape(topology), [[0, 10, 'overlap'], [10, 25, 'overlap'], [25, 30, 'overlap'],
        [30, 40, 'overlap'], [40, 60, 'wire-only'], [60, 80, 'wire-only']]);
    assert.equal(topology.coordinate, 'common-centerline');
    assert.equal(topology.connected, true); assert.deepEqual(topology.gaps, []);
    assert.deepEqual(topology.twistToolIds, ['wire', 'catheter']);
    assert.deepEqual(topology.slidingToolIds, ['wire', 'catheter']);
    assert.equal(topology.sections[0].contacts.wall, null);
    assert.equal(topology.sections[0].contacts.sheath.owner, 'catheter');
    close(topology.sections[0].contacts.sheath.clearance, .1);
    for (const s of topology.sections.slice(1, 4)) {
        assert.equal(s.contacts.wall.owner, 'catheter'); assert.equal(s.contacts.sheath, null);
        assert.equal(s.contacts.lumen.inner, 'wire'); close(s.contacts.lumen.clearance, .1);
    }
    assert.equal(topology.sections[4].contacts.wall.owner, 'wire');
    assert.equal(topology.sections[4].contacts.lumen, null);
    const portals = topology.boundaries.filter(b => b.portal);
    assert.equal(portals.length, 1);
    assert.deepEqual(portals[0].portal, { kind: 'distal-portal', inner: 'wire', outer: 'catheter',
        x: 40, innerMaterial: 160, outerMaterial: 70, crossing: true, coincidentTips: false, clearance: .5 - .4 });
});

test('unequal dsDx keeps separate affine labels, including exact endpoint labels', () => {
    const topology = build({ wire: tool(10, { materialInterval: [20, 40], dsDx: 2, materialBreakpoints: [30] }),
        catheter: tool(8, { materialInterval: [100, 130], dsDx: 3, materialBreakpoints: [124] }) });
    assert.deepEqual(topology.boundaries.map(b => b.x), [0, 5, 6, 8, 10]);
    for (const s of topology.sections) {
        for (const t of s.tools) {
            const tip = t.id === 'wire' ? 40 : 130;
            close(t.sStart, tip + t.dsDx * (s.start - t.insertion));
            close(t.sEnd, tip + t.dsDx * (s.end - t.insertion));
            close(sample(s, { x: s.end }).find(v => v.id === t.id).s, t.sEnd);
        }
    }
    assert.equal(topology.sections.at(-1).tools[0].sEnd, 40);
    assert.equal(topology.boundaries.find(b => b.portal).portal.innerMaterial, 36);
});

test('feed/retract moves tip ownership through overlap, coincident tips and catheter-only extension', () => {
    const wire = tool(4, { materialInterval: [100, 200] });
    const cases = [2, 4, 6, 4, 2, 0, -1];
    for (const tip of cases) {
        const topology = build({ wire, catheter: tool(tip, { materialInterval: [200, 300] }) });
        assert.deepEqual(shape(topology), tip <= 0 ? [[0, 4, 'wire-only']] : tip < 4
            ? [[0, tip, 'overlap'], [tip, 4, 'wire-only']] : tip === 4
                ? [[0, 4, 'overlap']] : [[0, 4, 'overlap'], [4, tip, 'catheter-only']]);
        const portal = topology.boundaries.find(b => b.portal)?.portal;
        assert.equal(!!portal, tip >= 0 && tip <= 4);
        if (portal) assert.equal(portal.coincidentTips, tip === 4);
        for (const s of topology.sections) for (const t of s.tools)
            assert.equal(t.sEnd - t.sStart, s.end - s.start);
    }
    const before = build({ wire, catheter: tool(2) });
    const saved = shape(before);
    const after = build({ wire: { ...wire, insertion: 5 }, catheter: tool(2) });
    assert.deepEqual(shape(before), saved);
    const a = sample(sectionAt(before, 1), { x: 1 }), b = sample(sectionAt(after, 1), { x: 1 });
    assert.equal(b[0].s, a[0].s - 1); assert.equal(b[1].s, a[1].s);
});

test('empty deployment and clipped/gapped material domains never create bridging material', () => {
    assert.deepEqual(build().sections, []);
    assert.deepEqual(build({ wire: tool(-2), catheter: tool(-1) }).interval, [0, 0]);
    const topology = build({ interval: [-1, 9], wire: tool(3, { materialInterval: [0, 3] }),
        catheter: tool(8, { materialInterval: [0, 3] }) });
    assert.deepEqual(shape(topology), [[0, 3, 'wire-only'], [5, 8, 'catheter-only']]);
    assert.deepEqual(topology.gaps, [{ start: -1, end: 0 }, { start: 3, end: 5 }, { start: 8, end: 9 }]);
    assert.equal(topology.connected, false);
    assert.equal(topology.boundaries.some(b => b.portal), false);
});

test('no boundary epsilon removes short exposed spans or coincident boundary events', () => {
    const tip = 4 + 1e-12;
    const topology = build({ wire: tool(tip), catheter: tool(4, { materialBreakpoints: [100] }), boundaries: [4] });
    assert.deepEqual(shape(topology), [[0, 4, 'overlap'], [4, tip, 'wire-only']]);
    assert.equal(topology.boundaries.filter(b => b.x === 4).length, 1);
    assert.deepEqual(topology.boundaries.find(b => b.x === 4).events.map(v => v.kind), ['external', 'tip', 'material-interface']);
    const x = 1e16;
    const nearPrecision = build({ interval: [x, x + 4], wire: tool(x + 4, { materialInterval: [0, 4] }),
        catheter: tool(x + 2, { materialInterval: [0, 2] }) });
    assert.deepEqual(shape(nearPrecision), [[x, x + 2, 'overlap'], [x + 2, x + 4, 'wire-only']]);
});

test('contacts retain unknown or incompatible clearance as data, without activating or welding it', () => {
    const unknown = build({ wire: tool(5), catheter: tool(3) });
    assert.equal(unknown.sections[0].contacts.lumen.clearance, null);
    const incompatible = build({ wire: tool(5, { radius: .6 }), catheter: tool(3, { innerRadius: .5, radius: .8 }),
        sheath: { interval: [0, 1], innerRadius: .7 } });
    close(incompatible.sections[0].contacts.lumen.clearance, -.1);
    close(incompatible.sections[0].contacts.sheath.clearance, -.1);
    assert.equal(Object.hasOwn(incompatible.sections[0].contacts.lumen, 'active'), false);
    const wireOnly = build({ wire: tool(5), sheath: { interval: [0, 1] } });
    assert.equal(wireOnly.sections[0].contacts.sheath.owner, 'wire');
});

test('sum of energy over moving covered lengths equals the independent rod energy integral', () => {
    const curvature = [.2, -.1];
    const density = (ei, gj, rate) => .5 * ei * (.2 ** 2 + .1 ** 2) + .5 * gj * rate ** 2;
    for (const a of [0, 2, 6, 10]) for (const b of [0, 2, 6, 10]) {
        const topology = build({ wire: tool(a, { material: material(10, 4), rotation: { theta: .4, thetaPrime: .2 } }),
            catheter: tool(b, { material: material(30, 8), rotation: { theta: -.7, thetaPrime: -.3 } }) });
        close(energyIntegral(topology, { curvature }), a * density(10, 4, .2) + b * density(30, 8, -.3));
    }
});

const variableMaterial = s => ({ stiffness: [[15 + s, 2, -1], [2, 10 + s / 2, 1.5], [-1, 1.5, 7 + s / 4]],
    intrinsic: [.12 + .01 * s, -.08 + .02 * s, .03 - .015 * s] });
const materialDerivative = { stiffness: [[1, 0, 0], [0, .5, 0], [0, 0, .25]], intrinsic: [.01, .02, -.015] };
function coupled() {
    return { interval: [0, 2], wire: tool(3, { materialInterval: [0, 10], dsDx: 1.3, material: variableMaterial,
        materialDerivative, rotation: { theta: .7, thetaPrime: -.09 } }),
    catheter: tool(4, { materialInterval: [0, 8], dsDx: .8, material: variableMaterial,
        materialDerivative, rotation: { theta: -.4, thetaPrime: .04 } }) };
}
const fields = { x: 1.2, curvature: [.21, -.17], frameTwist: .06 };

test('anisotropic coupled material energy and independent torques match the full material oracle', () => {
    const s = build(coupled()).sections[0], result = evaluate(s, fields);
    const tools = sample(s, fields).map(t => ({ ...t, strain: [
        Math.cos(t.theta) * fields.curvature[0] + Math.sin(t.theta) * fields.curvature[1],
        -Math.sin(t.theta) * fields.curvature[0] + Math.cos(t.theta) * fields.curvature[1],
        fields.frameTwist + t.thetaPrime
    ].map(v => v / t.dsDx) }));
    const oracle = evaluateFullBundleSection({ tools });
    close(result.energy, oracle.energy);
    result.tools.forEach((t, i) => closeVector(t.materialMoment, oracle.tools[i].materialMoment));
    for (const id of ['wire', 'catheter']) for (const key of ['theta', 'thetaPrime']) {
        const original = coupled()[id].rotation[key];
        const numerical = derivative(v => {
            const input = coupled(); input[id].rotation[key] = v;
            return evaluate(build(input).sections[0], fields).energy;
        }, original);
        close(numerical, result.tools.find(t => t.id === id)[key === 'theta' ? 'dTheta' : 'dThetaPrime'], 2e-7);
    }
    fields.curvature.forEach((value, i) => close(result.bendingMoment[i], derivative(v => {
        const curvature = [...fields.curvature]; curvature[i] = v;
        return evaluate(s, { ...fields, curvature }).energy;
    }, value), 2e-7));
});

test('independent sliding samples its own material; local insertion derivative includes dsDx chain rule', () => {
    const input = coupled(), result = evaluate(build(input).sections[0], fields);
    for (const id of ['wire', 'catheter']) {
        const response = result.tools.find(t => t.id === id);
        const numerical = derivative(insertion => {
            const changed = coupled(); changed[id].insertion = insertion;
            return evaluate(build(changed).sections[0], fields).energy;
        }, input[id].insertion);
        close(numerical, -input[id].dsDx * response.dS, 2e-7);
    }
    // This derivative is local at fixed x/rotation. Moving boundary energy is
    // accounted separately by the covered-length integral test above.
});

test('condensation keeps mismatch energy and opposite frictionless relative-rotation torques', () => {
    const angle = 1.1, EI = 100, k0 = .15;
    const run = theta => condense(build({ wire: tool(5, { material: material(EI, 30, [k0, 0]) }),
        catheter: tool(5, { material: material(EI, 30, [k0, 0]), rotation: theta }) }).sections[0]);
    const result = run(angle);
    close(result.energyOffset, EI * k0 ** 2 * (1 - Math.cos(angle)) / 2);
    const torque = EI * k0 ** 2 * Math.sin(angle) / 2;
    closeVector(result.minimum.tools.map(t => t.dTheta), [-torque, torque]);
    close(derivative(theta => run(theta).energyOffset, angle), torque);
    closeVector(result.bendingStiffness[0], [200, 0]);
    closeVector(result.bendingStiffness[1], [0, 200]);
    const section = build(coupled()).sections[0], condensed = condense(section, fields);
    const delta = fields.curvature.map((v, i) => v - condensed.preferredCurvature[i]);
    const quadratic = delta.reduce((sum, a, i) => sum + a * condensed.bendingStiffness[i].reduce((v, b, j) => v + b * delta[j], 0), 0);
    close(evaluate(section, fields).energy, condensed.energyOffset + .5 * quadratic);
});

test('straight isotropic tools retain independent twists, with no welded GJ or compliance sum', () => {
    const s = build({ wire: tool(4, { material: material(100, 20), rotation: { theta: .8, thetaPrime: .2 } }),
        catheter: tool(4, { material: material(300, 60), rotation: { theta: -3, thetaPrime: -.3 } }) }).sections[0];
    const result = evaluate(s);
    close(result.energy, .5 * 20 * .2 ** 2 + .5 * 60 * .3 ** 2);
    closeVector(result.tools.map(t => t.dThetaPrime), [4, -18]);
    closeVector(result.tools.map(t => t.dTheta), [0, 0]);
    closeVector(result.tools.map(t => t.dS), [0, 0]);
    const changed = evaluate(s, { rotations: { wire: { theta: 9, thetaPrime: .4 } } });
    close(changed.tools[1].dThetaPrime, -18); close(changed.tools[0].dThetaPrime, 8);
});

test('material jumps are sampled on each exact boundary side, without blending or invented derivatives', () => {
    const sampledLabels = [];
    const topology = build({ wire: tool(2, { materialInterval: [0, 2], materialBreakpoints: [1],
        material: (s, { side }) => { sampledLabels.push([s, side]); return material(s < 1 || s === 1 && side === 'left' ? 2 : 8); } }) });
    const left = evaluate(topology.sections[0], { x: 1, curvature: [1, 0] });
    const right = evaluate(topology.sections[1], { x: 1, curvature: [1, 0] });
    close(left.energy, 1); close(right.energy, 4);
    assert.deepEqual(sampledLabels, [[1, 'left'], [1, 'right']]);
    assert.equal(left.tools[0].dS, null); assert.equal(right.tools[0].dS, null);
});

test('topology copies numeric inputs and samples own scratch-backed material values', () => {
    const input = { wire: tool(2, { material: material(10) }), catheter: tool(2, { material: material(30) }) };
    const topology = build(input), original = evaluate(topology.sections[0], { curvature: [1, 0] });
    input.wire.material.EI1 = 300; input.wire.material.kappa0[0] = 5; input.wire.materialInterval[1] = 200;
    close(evaluate(topology.sections[0], { curvature: [1, 0] }).energy, original.energy);
    const scratch = material();
    const shared = build({ wire: tool(3, { material: () => { scratch.EI1 = 2; return scratch; } }),
        catheter: tool(3, { material: () => { scratch.EI1 = 8; return scratch; } }) });
    close(evaluate(shared.sections[0], { curvature: [1, 0] }).energy, 5);
});

test('tip-profile adapter reverses only the lookup side at an exact constitutive jump', () => {
    const seen = [];
    const profile = { sample(distance, out, context) {
        seen.push([distance, context.side]);
        return Object.assign(out, material(distance < 1 || distance === 1 && context.side === 'left' ? 2 : 8));
    } };
    const wire = compositeToolFromTipProfile({ profile, insertion: 2, materialInterval: [10, 12], tipBreakpoints: [1] });
    const topology = build({ wire });
    close(evaluate(topology.sections[0], { x: 1, curvature: [1, 0] }).energy, 4);
    close(evaluate(topology.sections[1], { x: 1, curvature: [1, 0] }).energy, 1);
    assert.deepEqual(seen, [[1, 'right'], [1, 'left']]);
    const shifted = build({ wire: { ...wire, insertion: 3 } });
    assert.equal(shifted.boundaries.find(b => b.events.some(e => e.kind === 'material-interface')).x, 2);
    assert.equal(topology.boundaries.find(b => b.events.some(e => e.kind === 'material-interface')).x, 1);
});

test('material spin fields remain separate and supply explicit rates in the common coordinate', () => {
    const topology = build({ wire: tool(4, { dsDx: 2, rotation: { theta: s => .03 * s, thetaPrime: .06 } }),
        catheter: tool(4, { dsDx: .5, rotation: { theta: s => -.04 * s, thetaPrime: -.02 } }) });
    const [wire, catheter] = sample(topology.sections[0], { x: 1 });
    close(wire.theta, .03 * 94); close(catheter.theta, -.04 * 98.5);
    close(wire.thetaPrime, .06); close(catheter.thetaPrime, -.02);
    const result = evaluate(topology.sections[0], { x: 1 });
    closeVector(result.tools.map(t => t.strain[2]), [.03, -.04]);
    closeVector(result.tools.map(t => t.dThetaPrime), [.12, -.16]);
});

test('real steel J wire and pigtail profiles retain tip lookup, rotation, stiffness and clearance', () => {
    const wireProfile = kirchhoffMaterialProfile('steel-j-035'), catheterProfile = kirchhoffMaterialProfile('pigtail');
    const wire = compositeToolFromTipProfile({ profile: wireProfile, insertion: 130, materialInterval: [0, 1000],
        radius: GUIDEWIRE_RADIUS_MM, rotation: { theta: .3, thetaPrime: .002 }, tipBreakpoints: [
            STEEL_J_GUIDEWIRE_CURVED_TIP_LENGTH_MM - STEEL_J_GUIDEWIRE_CURVATURE_TRANSITION_MM,
            STEEL_J_GUIDEWIRE_CURVED_TIP_LENGTH_MM, STEEL_J_GUIDEWIRE_TIP_CORE_LENGTH_MM,
            STEEL_J_GUIDEWIRE_TIP_CORE_LENGTH_MM + STEEL_J_GUIDEWIRE_TIP_TRANSITION_LENGTH_MM
        ] });
    const catheter = compositeToolFromTipProfile({ profile: catheterProfile, insertion: 120, materialInterval: [20, 820],
        radius: PIGTAIL_CATHETER_RADIUS_MM, innerRadius: PIGTAIL_CATHETER_INNER_RADIUS_MM,
        rotation: { theta: -.8, thetaPrime: -.003 }, tipBreakpoints: [
            PIGTAIL_NATURAL_ARC_LENGTH_MM - PIGTAIL_CURVATURE_TRANSITION_MM, PIGTAIL_NATURAL_ARC_LENGTH_MM
        ] });
    const topology = build({ wire, catheter, sheath: { interval: [0, 10], innerRadius: INTRODUCER_SHEATH_INNER_RADIUS_MM } });
    const x = 112, s = sectionAt(topology, x), inputs = sample(s, { x });
    assert.deepEqual(inputs.map(t => t.s), [982, 812]);
    assert.deepEqual(inputs[0].material(), wireProfile.sample(18));
    assert.deepEqual(inputs[1].material(), catheterProfile.sample(8));
    close(s.contacts.lumen.clearance, .0405);
    const result = evaluate(s, { x, curvature: [.01, -.02], frameTwist: .001 });
    assert.ok(Number.isFinite(result.energy) && result.energy > 0);
    assert.notEqual(result.tools[0].dThetaPrime, result.tools[1].dThetaPrime);
    assert.equal(result.tools[0].dS, null, 'an unavailable profile derivative must not become zero');
    assert.equal(topology.sections.at(-1).kind, 'wire-only');
    assert.equal(topology.boundaries.find(b => b.portal).x, 120);
    const shifted = build({ wire, catheter: { ...catheter, insertion: 125 } });
    const shiftedInputs = sample(sectionAt(shifted, x), { x });
    assert.equal(shiftedInputs[0].s, inputs[0].s);
    assert.equal(shiftedInputs[1].s, inputs[1].s - 5);
    assert.deepEqual(shiftedInputs[1].material(), catheterProfile.sample(13));
});

test('invalid maps, rates, geometry and material are rejected explicitly', () => {
    for (const override of [{ dsDx: 0 }, { dsDx: -1 }, { insertion: NaN }, { materialInterval: [2, 1] },
        { materialBreakpoints: [101] }, { radius: 0 }, { radius: .4, innerRadius: .5 },
        { rotation: { theta: () => 0 } }]) assert.throws(() => build({ wire: tool(4, override) }));
    assert.throws(() => build({ wire: tool(4), interval: [2, 1] }));
    assert.throws(() => build({ wire: tool(4), boundaries: [Infinity] }));
    const s = build({ wire: tool(4) }).sections[0];
    assert.throws(() => sample(s, { x: 5 }), /outside/);
    assert.throws(() => evaluate(build({ wire: tool(4, { material: material(-1) }) }).sections[0]), /positive definite/);
    assert.throws(() => compositeToolFromTipProfile({ profile: kirchhoffMaterialProfile('pigtail'),
        materialInterval: [0, 2], tipBreakpoints: [3] }), /outside/);
});
