import assert from 'node:assert/strict';
import {
    BERENSTEIN_NATURAL_BEND_ANGLE_RAD,
    BERENSTEIN_TIP_SHAPE_LENGTH_MM,
    PIGTAIL_NATURAL_ARC_LENGTH_MM,
    PIGTAIL_NATURAL_TURNS,
    SIM1_TIP_SHAPE_LENGTH_MM,
    SIM1_TOTAL_TURN_RAD
} from '../src/physics/catheterMaterialProfile.js';
import {
    GUIDEWIRE_BODY_BENDING_STIFFNESS,
    GUIDEWIRE_TIP_BENDING_STIFFNESS,
    GUIDEWIRE_TIP_CORE_LENGTH_MM,
    GUIDEWIRE_SOFT_TIP_LENGTH_MM,
    STEEL_J_GUIDEWIRE_BODY_BENDING_STIFFNESS,
    STEEL_J_GUIDEWIRE_CURVED_TIP_LENGTH_MM,
    STEEL_J_GUIDEWIRE_NATURAL_TURN_RAD,
    STEEL_J_GUIDEWIRE_TIP_BENDING_STIFFNESS
} from '../src/physics/guidewireMaterialProfile.js';
import {
    KIRCHHOFF_MATERIAL_PROFILES,
    KIRCHHOFF_PROFILE_EXPECTED_TURNS,
    LEGACY_GUIDEWIRE_RIGIDITY_TO_KIRCHHOFF_EI,
    discretizeKirchhoffProfile,
    integrateKirchhoffMaterial,
    integrateKirchhoffVoronoi,
    kirchhoffMaterialProfile,
    sampleKirchhoffMaterial
} from '../src/physics/kirchhoffMaterialProfile.js';

const PROFILE_CASES = Object.freeze([
    Object.freeze({
        id: 'pigtail',
        length: PIGTAIL_NATURAL_ARC_LENGTH_MM,
        expectedTurn: -PIGTAIL_NATURAL_TURNS * Math.PI * 2
    }),
    Object.freeze({
        id: 'berenstein',
        length: BERENSTEIN_TIP_SHAPE_LENGTH_MM,
        expectedTurn: BERENSTEIN_NATURAL_BEND_ANGLE_RAD
    }),
    Object.freeze({
        id: 'sim1',
        length: SIM1_TIP_SHAPE_LENGTH_MM,
        expectedTurn: -SIM1_TOTAL_TURN_RAD
    }),
    Object.freeze({
        id: 'glidewire',
        length: 80,
        expectedTurn: 0
    }),
    Object.freeze({
        id: 'steel-j-035',
        length: STEEL_J_GUIDEWIRE_CURVED_TIP_LENGTH_MM,
        expectedTurn: STEEL_J_GUIDEWIRE_NATURAL_TURN_RAD
    })
]);

assert.equal(kirchhoffMaterialProfile('bernstein'), KIRCHHOFF_MATERIAL_PROFILES.berenstein);
assert.equal(kirchhoffMaterialProfile('unknown'), KIRCHHOFF_MATERIAL_PROFILES.glidewire);

for (const { id, length, expectedTurn } of PROFILE_CASES) {
    const profile = kirchhoffMaterialProfile(id);
    assert.equal(profile.id, id);
    assert.equal(KIRCHHOFF_PROFILE_EXPECTED_TURNS[id], expectedTurn);

    for (const s of [0, length * 0.25, length * 0.5, length, length + 40]) {
        const sample = sampleKirchhoffMaterial(profile, s);
        assert.ok(Number.isFinite(sample.kappa01));
        assert.equal(sample.kappa02, 0, `${id} must be planar in its material frame`);
        assert.equal(sample.tau0, 0, `${id} must have no manufactured intrinsic twist`);
        assert.ok(sample.EI1 > 0 && Number.isFinite(sample.EI1));
        assert.equal(sample.EI2, sample.EI1, `${id} starts as an isotropic circular rod`);
        assert.ok(sample.GJ > 0 && Number.isFinite(sample.GJ));
    }

    const whole = integrateKirchhoffMaterial(profile, 0, length);
    assert.ok(
        Math.abs(whole.kappa01Integral - expectedTurn) < 1e-10,
        `${id} whole-profile intrinsic turn must be exact`
    );
    assert.equal(whole.kappa02Integral, 0);
    assert.equal(whole.tau0Integral, 0);
    assert.ok(whole.EI1Integral > 0 && whole.EI2Integral > 0 && whole.GJIntegral > 0);

    for (const spacing of [0.75, 1, 2.5, 4, 5.75, 9]) {
        let representedTurn = 0;
        let representedTwist = 0;
        for (let start = 0; start < length; start += spacing) {
            const end = Math.min(length, start + spacing);
            const cell = integrateKirchhoffVoronoi(
                profile,
                (start + end) * 0.5,
                end - start
            );
            representedTurn += cell.kappa01Integral;
            representedTwist += cell.tau0Integral;
        }
        assert.ok(
            Math.abs(representedTurn - expectedTurn) < 1e-9,
            `${id} turn must not depend on ${spacing} mm material cells`
        );
        assert.equal(representedTwist, 0);
    }
}

const pigtailTip = sampleKirchhoffMaterial('pigtail', 0);
const berensteinTip = sampleKirchhoffMaterial('berenstein', 0);
const sim1Tip = sampleKirchhoffMaterial('sim1', 0);
const steelJTip = sampleKirchhoffMaterial('steel-j-035', 0);
assert.ok(pigtailTip.kappa01 < 0, 'Pigtail orientation sign belongs to kappa01');
assert.equal(berensteinTip.kappa01, 0, 'Berenstein retains its straight distal 8 mm tip');
assert.ok(
    Math.abs(sim1Tip.kappa01) < 1e-12,
    'SIM 1 retains its straight distal 8 mm tip'
);
assert.ok(steelJTip.kappa01 > 0, 'Steel J orientation sign belongs to kappa01');

const glidewireTip = sampleKirchhoffMaterial('glidewire', 0);
const glidewireTransitionStart = sampleKirchhoffMaterial(
    'glidewire',
    GUIDEWIRE_TIP_CORE_LENGTH_MM
);
const glidewireBody = sampleKirchhoffMaterial('glidewire', GUIDEWIRE_SOFT_TIP_LENGTH_MM);
const glidewireTransitionMiddle = sampleKirchhoffMaterial(
    'glidewire',
    GUIDEWIRE_TIP_CORE_LENGTH_MM +
        (GUIDEWIRE_SOFT_TIP_LENGTH_MM - GUIDEWIRE_TIP_CORE_LENGTH_MM) * 0.5
);
assert.equal(
    glidewireTip.EI1,
    GUIDEWIRE_TIP_BENDING_STIFFNESS *
        LEGACY_GUIDEWIRE_RIGIDITY_TO_KIRCHHOFF_EI
);
assert.equal(
    glidewireTransitionStart.EI1,
    GUIDEWIRE_TIP_BENDING_STIFFNESS *
        LEGACY_GUIDEWIRE_RIGIDITY_TO_KIRCHHOFF_EI
);
assert.ok(Math.abs(
    glidewireBody.EI1 -
    GUIDEWIRE_BODY_BENDING_STIFFNESS *
        LEGACY_GUIDEWIRE_RIGIDITY_TO_KIRCHHOFF_EI
) < 1e-9);
assert.equal(GUIDEWIRE_SOFT_TIP_LENGTH_MM, 50);
assert.ok(
    glidewireTip.EI1 * 128 <= glidewireBody.EI1,
    'the distal Glidewire core should be at least 128x more flexible than its shaft'
);
assert.ok(
    glidewireTransitionMiddle.EI1 > glidewireTip.EI1 &&
        glidewireTransitionMiddle.EI1 < glidewireBody.EI1,
    'the soft distal core must blend continuously into the shaft'
);

assert.equal(
    steelJTip.EI1,
    STEEL_J_GUIDEWIRE_TIP_BENDING_STIFFNESS *
        LEGACY_GUIDEWIRE_RIGIDITY_TO_KIRCHHOFF_EI
);
const steelJBody = sampleKirchhoffMaterial('steel-j-035', 100);
assert.ok(
    Math.abs(
        steelJBody.EI1 -
        STEEL_J_GUIDEWIRE_BODY_BENDING_STIFFNESS *
            LEGACY_GUIDEWIRE_RIGIDITY_TO_KIRCHHOFF_EI
    ) < 1e-9
);

const reusableSample = {};
assert.equal(sampleKirchhoffMaterial('pigtail', 2, reusableSample), reusableSample);
const reusableIntegral = {};
assert.equal(
    integrateKirchhoffMaterial('pigtail', 4, 0, reusableIntegral),
    reusableIntegral
);
assert.equal(reusableIntegral.startDistanceFromTipMm, 0);
assert.equal(reusableIntegral.endDistanceFromTipMm, 4);

assert.throws(() => sampleKirchhoffMaterial('pigtail', Infinity), /finite/);
assert.throws(
    () => integrateKirchhoffVoronoi('pigtail', 1, -1),
    /Voronoi length/
);

function nonUniformCoordinates(length, origin = 0, direction = 1) {
    const fractions = [0, 0.04, 0.13, 0.27, 0.46, 0.64, 0.79, 0.91, 1];
    return Float64Array.from(
        fractions,
        fraction => origin + direction * length * fraction
    );
}

for (const { id, length, expectedTurn } of PROFILE_CASES) {
    if (id === 'glidewire') continue;
    const materialCoordinates = nonUniformCoordinates(length);
    const discrete = discretizeKirchhoffProfile(
        id,
        materialCoordinates,
        length
    );
    let totalRotation1 = 0;
    let totalRotation2 = 0;
    let totalRotation3 = 0;
    let totalVoronoiLength = 0;
    for (let joint = discrete.jointStart; joint <= discrete.jointEnd; joint++) {
        totalRotation1 += discrete.restRotation1[joint];
        totalRotation2 += discrete.restRotation2[joint];
        totalRotation3 += discrete.restRotation3[joint];
        totalVoronoiLength += discrete.voronoiLength[joint];
        assert.ok(discrete.cellEndS[joint] > discrete.cellStartS[joint]);
        assert.ok(discrete.EI1Mean[joint] > 0 && discrete.EI2Mean[joint] > 0);
        assert.ok(discrete.GJMean[joint] > 0);
        assert.ok(Math.abs(
            discrete.bendCompliance1[joint] -
            discrete.voronoiLength[joint] / discrete.EI1Mean[joint]
        ) < 1e-15);
        assert.ok(Math.abs(
            discrete.bendCompliance2[joint] -
            discrete.voronoiLength[joint] / discrete.EI2Mean[joint]
        ) < 1e-15);
        assert.ok(Math.abs(
            discrete.twistCompliance[joint] -
            discrete.voronoiLength[joint] / discrete.GJMean[joint]
        ) < 1e-15);
    }
    assert.ok(Math.abs(totalRotation1 - expectedTurn) < 1e-10);
    assert.equal(totalRotation2, 0);
    assert.equal(totalRotation3, 0);
    assert.ok(Math.abs(totalVoronoiLength - length) < 1e-12);
    assert.equal(discrete.restRotation1[0], 0);
    assert.equal(discrete.restRotation1[discrete.nodeCount - 1], 0);
    assert.equal(discrete.compliance1, discrete.bendCompliance1);
    assert.equal(discrete.compliance2, discrete.bendCompliance2);
    assert.equal(discrete.compliance3, discrete.twistCompliance);

    // Translation and reversal of the material coordinate axis cannot change
    // constitutive state because only distance from the supplied tip matters.
    const reversedCoordinates = nonUniformCoordinates(length, 83, -1);
    const reversed = discretizeKirchhoffProfile(
        id,
        reversedCoordinates,
        83 - length
    );
    for (let joint = 0; joint < discrete.nodeCount; joint++) {
        assert.ok(Math.abs(
            reversed.restRotation1[joint] - discrete.restRotation1[joint]
        ) < 1e-12);
    }

    // Remeshing resamples from material s rather than copying a live pose.
    const remeshedCoordinates = Float64Array.from(
        [0, 0.09, 0.31, 0.58, 0.76, 0.88, 0.96, 1],
        fraction => fraction * length
    );
    const remeshed = discretizeKirchhoffProfile(
        id,
        remeshedCoordinates,
        length
    );
    const remeshedTurn = Array.from(remeshed.restRotation1)
        .reduce((sum, rotation) => sum + rotation, 0);
    assert.ok(Math.abs(remeshedTurn - expectedTurn) < 1e-10);
}

const reusableDiscretization = discretizeKirchhoffProfile(
    'steel-j-035',
    nonUniformCoordinates(STEEL_J_GUIDEWIRE_CURVED_TIP_LENGTH_MM),
    STEEL_J_GUIDEWIRE_CURVED_TIP_LENGTH_MM
);
const reusedRotationStorage = reusableDiscretization.restRotation1;
assert.equal(
    discretizeKirchhoffProfile(
        'steel-j-035',
        nonUniformCoordinates(STEEL_J_GUIDEWIRE_CURVED_TIP_LENGTH_MM),
        STEEL_J_GUIDEWIRE_CURVED_TIP_LENGTH_MM,
        reusableDiscretization
    ),
    reusableDiscretization
);
assert.equal(reusableDiscretization.restRotation1, reusedRotationStorage);

assert.throws(
    () => discretizeKirchhoffProfile('pigtail', [0, 2], 2),
    /at least three/
);
assert.throws(
    () => discretizeKirchhoffProfile('pigtail', [0, 2, 1, 4], 4),
    /strictly ordered/
);

console.log('Kirchhoff material profile tests passed');
