import {
    BERENSTEIN_NATURAL_BEND_ANGLE_RAD,
    BERENSTEIN_TIP_SHAPE_LENGTH_MM,
    catheterMaterialProfile,
    integrateBerensteinIntrinsicTurn,
    integratePigtailIntrinsicTurn,
    integrateSim1IntrinsicTurn,
    PIGTAIL_NATURAL_ARC_LENGTH_MM,
    PIGTAIL_NATURAL_TURNS,
    SIM1_TIP_SHAPE_LENGTH_MM,
    SIM1_TOTAL_TURN_RAD,
    berensteinIntrinsicCurvature,
    pigtailIntrinsicCurvature,
    sim1IntrinsicCurvature
} from './catheterMaterialProfile.js';
import {
    GUIDEWIRE_TYPE_GLIDEWIRE,
    GUIDEWIRE_TYPE_STEEL_J_035,
    STEEL_J_GUIDEWIRE_CURVED_TIP_LENGTH_MM,
    STEEL_J_GUIDEWIRE_NATURAL_TURN_RAD,
    guidewireMaterialProfile,
    integrateSteelJGuidewireIntrinsicTurn,
    steelJGuidewireIntrinsicCurvature
} from './guidewireMaterialProfile.js';

const TWO_PI = Math.PI * 2;
const MINIMUM_RIGIDITY = 1e-12;
const WIRE_POISSON_RATIO = 0.3;
const CATHETER_POISSON_RATIO = 0.4;
// The legacy guidewire table stores relative bending weights, not EI in the
// millimetre/radian units used by the Kirchhoff energy. For the simulator's
// 4 mm guidewire cells, the old angular XPBD compliance was
//   bendCompliance * 32 * 100 / weight = 0.064 / weight.
// Kirchhoff uses L / EI, so EI = 64 * weight preserves the calibrated shaft
// response (4 / (64 * weight) = 0.0625 / weight) instead of making the wire
// roughly 64 times too soft during migration.
export const LEGACY_GUIDEWIRE_RIGIDITY_TO_KIRCHHOFF_EI = 64;

// Five-point Gauss-Legendre quadrature is used only for the rigidity fields.
// Rest curvature uses each device's exact, boundary-aware integral below.
const GAUSS_SAMPLES = Object.freeze([
    Object.freeze([-0.906179845938664, 0.236926885056189]),
    Object.freeze([-0.538469310105683, 0.478628670499366]),
    Object.freeze([0, 0.568888888888889]),
    Object.freeze([0.538469310105683, 0.478628670499366]),
    Object.freeze([0.906179845938664, 0.236926885056189])
]);

function clamp01(value) {
    return Math.max(0, Math.min(1, value));
}

function smoothstep01(value) {
    const t = clamp01(value);
    return t * t * (3 - 2 * t);
}

function positiveRigidity(value, label, profileId) {
    if (!Number.isFinite(value) || value <= 0) {
        throw new RangeError(`${profileId} ${label} must be finite and positive`);
    }
    return Math.max(MINIMUM_RIGIDITY, value);
}

function validateMaterialCoordinate(value, label = 'material coordinate') {
    if (!Number.isFinite(value)) throw new TypeError(`${label} must be finite`);
    return Math.max(0, value);
}

function normalizedInterval(startDistanceFromTipMm, endDistanceFromTipMm) {
    const first = validateMaterialCoordinate(startDistanceFromTipMm, 'interval start');
    const second = validateMaterialCoordinate(endDistanceFromTipMm, 'interval end');
    return first <= second ? [first, second] : [second, first];
}

function geometricTransition(tipValue, bodyValue, progress) {
    if (tipValue === bodyValue) return tipValue;
    return tipValue * Math.pow(bodyValue / tipValue, smoothstep01(progress));
}

function integrateScalar(sample, start, end) {
    if (end <= start) return 0;
    const midpoint = (start + end) * 0.5;
    const halfWidth = (end - start) * 0.5;
    let result = 0;
    for (const [abscissa, weight] of GAUSS_SAMPLES) {
        result += weight * sample(midpoint + abscissa * halfWidth);
    }
    return result * halfWidth;
}

function integrateScaledRigidity(
    profile,
    component,
    scaleAtDistance,
    start,
    end,
    sampleOut
) {
    return integrateScalar(distanceFromTipMm => {
        profile.sample(distanceFromTipMm, sampleOut);
        const scale = scaleAtDistance(distanceFromTipMm);
        if (!Number.isFinite(scale) || scale <= 0) {
            throw new RangeError(
                `${profile.id} rigidity scale must be finite and positive`
            );
        }
        return sampleOut[component] * scale;
    }, start, end);
}

function zeroIntegral() {
    return 0;
}

/**
 * Defines an immutable Kirchhoff material profile.
 *
 * `s` is the material distance in millimetres measured proximally from the
 * distal device tip. Kappa components and tau are expressed in the material
 * frame (d1, d2, d3=tangent), in 1/mm. Rigidity units are intentionally left
 * solver-independent; the XPBD layer is responsible for converting EI/GJ to
 * compliance for its chosen discrete strain measure.
 */
export function defineKirchhoffMaterialProfile({
    id,
    naturalTipLengthMm = 0,
    sampleKappa01 = () => 0,
    sampleKappa02 = () => 0,
    sampleTau0 = () => 0,
    sampleEI1,
    sampleEI2 = sampleEI1,
    sampleGJ,
    integrateKappa01 = null,
    integrateKappa02 = null,
    integrateTau0 = null
}) {
    if (!id || typeof id !== 'string') throw new TypeError('Material profile id is required');
    if (typeof sampleEI1 !== 'function' || typeof sampleGJ !== 'function') {
        throw new TypeError(`${id} must provide EI and GJ samplers`);
    }

    const profile = {
        id,
        naturalTipLengthMm: Math.max(0, naturalTipLengthMm),
        sample(distanceFromTipMm, out = {}) {
            const s = validateMaterialCoordinate(distanceFromTipMm);
            out.distanceFromTipMm = s;
            out.kappa01 = sampleKappa01(s);
            out.kappa02 = sampleKappa02(s);
            out.tau0 = sampleTau0(s);
            out.EI1 = positiveRigidity(sampleEI1(s), 'EI1', id);
            out.EI2 = positiveRigidity(sampleEI2(s), 'EI2', id);
            out.GJ = positiveRigidity(sampleGJ(s), 'GJ', id);
            return out;
        },
        integrate(startDistanceFromTipMm, endDistanceFromTipMm, out = {}) {
            const [start, end] = normalizedInterval(
                startDistanceFromTipMm,
                endDistanceFromTipMm
            );
            const length = end - start;
            out.startDistanceFromTipMm = start;
            out.endDistanceFromTipMm = end;
            out.lengthMm = length;
            out.kappa01Integral = integrateKappa01
                ? integrateKappa01(start, end)
                : integrateScalar(sampleKappa01, start, end);
            out.kappa02Integral = integrateKappa02
                ? integrateKappa02(start, end)
                : integrateScalar(sampleKappa02, start, end);
            out.tau0Integral = integrateTau0
                ? integrateTau0(start, end)
                : integrateScalar(sampleTau0, start, end);
            out.EI1Integral = integrateScalar(sampleEI1, start, end);
            out.EI2Integral = integrateScalar(sampleEI2, start, end);
            out.GJIntegral = integrateScalar(sampleGJ, start, end);
            out.EI1Mean = length > 0 ? out.EI1Integral / length : sampleEI1(start);
            out.EI2Mean = length > 0 ? out.EI2Integral / length : sampleEI2(start);
            out.GJMean = length > 0 ? out.GJIntegral / length : sampleGJ(start);
            return out;
        },
        integrateVoronoi(centerDistanceFromTipMm, voronoiLengthMm, out = {}) {
            const center = validateMaterialCoordinate(centerDistanceFromTipMm, 'cell center');
            if (!Number.isFinite(voronoiLengthMm) || voronoiLengthMm < 0) {
                throw new RangeError('Voronoi length must be finite and non-negative');
            }
            const halfLength = voronoiLengthMm * 0.5;
            return profile.integrate(
                Math.max(0, center - halfLength),
                center + halfLength,
                out
            );
        }
    };
    return Object.freeze(profile);
}

function exactLegacyIntegral(integrator, sign = 1) {
    return (start, end) => sign * integrator(
        (start + end) * 0.5,
        Math.max(0, end - start)
    );
}

function catheterRigiditySamplers(type) {
    const legacyProfile = catheterMaterialProfile(type);
    // This is a migration bridge, not a claim of calibrated physical units.
    // A single inverse-compliance mapping at least gives straight and curved
    // material one constitutive EI until force-deflection calibration replaces
    // the legacy XPBD tuning value.
    const EI = 1 /
        Math.max(MINIMUM_RIGIDITY, legacyProfile.intrinsicBendCompliance);
    const GJ = EI / (1 + CATHETER_POISSON_RATIO);
    return {
        EI: () => EI,
        GJ: () => GJ
    };
}

function guidewireRigiditySamplers(type) {
    const legacyProfile = guidewireMaterialProfile(type);
    const tipEI = LEGACY_GUIDEWIRE_RIGIDITY_TO_KIRCHHOFF_EI *
        positiveRigidity(legacyProfile.tipBendingStiffness, 'tip EI', type);
    const bodyEI = LEGACY_GUIDEWIRE_RIGIDITY_TO_KIRCHHOFF_EI *
        positiveRigidity(legacyProfile.bodyBendingStiffness, 'body EI', type);
    const coreLength = Math.max(0, legacyProfile.tipCoreLength);
    const transitionLength = Math.max(0, legacyProfile.tipTransitionLength);
    const EI = distanceFromTipMm => geometricTransition(
        tipEI,
        bodyEI,
        transitionLength > 0
            ? (distanceFromTipMm - coreLength) / transitionLength
            : distanceFromTipMm > coreLength ? 1 : 0
    );
    return {
        EI,
        GJ: distanceFromTipMm => EI(distanceFromTipMm) / (1 + WIRE_POISSON_RATIO)
    };
}

const pigtailRigidity = catheterRigiditySamplers('pigtail');
const berensteinRigidity = catheterRigiditySamplers('berenstein');
const sim1Rigidity = catheterRigiditySamplers('sim1');
const glidewireRigidity = guidewireRigiditySamplers(GUIDEWIRE_TYPE_GLIDEWIRE);
const steelJRigidity = guidewireRigiditySamplers(GUIDEWIRE_TYPE_STEEL_J_035);

export const KIRCHHOFF_MATERIAL_PROFILES = Object.freeze({
    pigtail: defineKirchhoffMaterialProfile({
        id: 'pigtail',
        naturalTipLengthMm: PIGTAIL_NATURAL_ARC_LENGTH_MM,
        sampleKappa01: s => -pigtailIntrinsicCurvature(s),
        sampleEI1: pigtailRigidity.EI,
        sampleGJ: pigtailRigidity.GJ,
        integrateKappa01: exactLegacyIntegral(integratePigtailIntrinsicTurn, -1),
        integrateKappa02: zeroIntegral,
        integrateTau0: zeroIntegral
    }),
    berenstein: defineKirchhoffMaterialProfile({
        id: 'berenstein',
        naturalTipLengthMm: BERENSTEIN_TIP_SHAPE_LENGTH_MM,
        sampleKappa01: berensteinIntrinsicCurvature,
        sampleEI1: berensteinRigidity.EI,
        sampleGJ: berensteinRigidity.GJ,
        integrateKappa01: exactLegacyIntegral(integrateBerensteinIntrinsicTurn),
        integrateKappa02: zeroIntegral,
        integrateTau0: zeroIntegral
    }),
    sim1: defineKirchhoffMaterialProfile({
        id: 'sim1',
        naturalTipLengthMm: SIM1_TIP_SHAPE_LENGTH_MM,
        sampleKappa01: s => -sim1IntrinsicCurvature(s),
        sampleEI1: sim1Rigidity.EI,
        sampleGJ: sim1Rigidity.GJ,
        integrateKappa01: exactLegacyIntegral(integrateSim1IntrinsicTurn, -1),
        integrateKappa02: zeroIntegral,
        integrateTau0: zeroIntegral
    }),
    [GUIDEWIRE_TYPE_GLIDEWIRE]: defineKirchhoffMaterialProfile({
        id: GUIDEWIRE_TYPE_GLIDEWIRE,
        naturalTipLengthMm: 0,
        sampleEI1: glidewireRigidity.EI,
        sampleGJ: glidewireRigidity.GJ,
        integrateKappa01: zeroIntegral,
        integrateKappa02: zeroIntegral,
        integrateTau0: zeroIntegral
    }),
    [GUIDEWIRE_TYPE_STEEL_J_035]: defineKirchhoffMaterialProfile({
        id: GUIDEWIRE_TYPE_STEEL_J_035,
        naturalTipLengthMm: STEEL_J_GUIDEWIRE_CURVED_TIP_LENGTH_MM,
        sampleKappa01: steelJGuidewireIntrinsicCurvature,
        sampleEI1: steelJRigidity.EI,
        sampleGJ: steelJRigidity.GJ,
        integrateKappa01: exactLegacyIntegral(integrateSteelJGuidewireIntrinsicTurn),
        integrateKappa02: zeroIntegral,
        integrateTau0: zeroIntegral
    })
});

export const KIRCHHOFF_PROFILE_EXPECTED_TURNS = Object.freeze({
    pigtail: -PIGTAIL_NATURAL_TURNS * TWO_PI,
    berenstein: BERENSTEIN_NATURAL_BEND_ANGLE_RAD,
    sim1: -SIM1_TOTAL_TURN_RAD,
    [GUIDEWIRE_TYPE_GLIDEWIRE]: 0,
    [GUIDEWIRE_TYPE_STEEL_J_035]: STEEL_J_GUIDEWIRE_NATURAL_TURN_RAD
});

export function normalizeKirchhoffMaterialType(type) {
    if (type === 'bernstein') return 'berenstein';
    if (type === 'sim-1' || type === 'simmons-1') return 'sim1';
    if (type === 'pigtail' || type === 'berenstein' || type === 'sim1') return type;
    if (type === GUIDEWIRE_TYPE_STEEL_J_035) return GUIDEWIRE_TYPE_STEEL_J_035;
    return GUIDEWIRE_TYPE_GLIDEWIRE;
}

export function kirchhoffMaterialProfile(type) {
    return KIRCHHOFF_MATERIAL_PROFILES[normalizeKirchhoffMaterialType(type)];
}

function resolveProfile(profileOrType) {
    if (profileOrType?.sample && profileOrType?.integrate) return profileOrType;
    return kirchhoffMaterialProfile(profileOrType);
}

export function sampleKirchhoffMaterial(
    profileOrType,
    distanceFromTipMm,
    out = {}
) {
    return resolveProfile(profileOrType).sample(distanceFromTipMm, out);
}

export function integrateKirchhoffMaterial(
    profileOrType,
    startDistanceFromTipMm,
    endDistanceFromTipMm,
    out = {}
) {
    return resolveProfile(profileOrType).integrate(
        startDistanceFromTipMm,
        endDistanceFromTipMm,
        out
    );
}

export function integrateKirchhoffVoronoi(
    profileOrType,
    centerDistanceFromTipMm,
    voronoiLengthMm,
    out = {}
) {
    return resolveProfile(profileOrType).integrateVoronoi(
        centerDistanceFromTipMm,
        voronoiLengthMm,
        out
    );
}

function prepareDiscretizationArray(out, name, count) {
    let array = out[name];
    if (!array || array.length !== count || typeof array.fill !== 'function') {
        array = new Float64Array(count);
        out[name] = array;
    } else {
        array.fill(0);
    }
    return array;
}

/**
 * Samples a constitutive profile onto the internal joints of a material mesh.
 *
 * Node coordinates are immutable Lagrangian/material coordinates ordered from
 * the proximal end toward `tipCoordinate`. They do not contain or depend on
 * the live spatial pose. Each internal joint owns a boundary-closed Voronoi
 * cell: the first and last cells extend to the material mesh endpoints because
 * endpoint nodes have no bending/twist joint of their own. The cells therefore
 * form a gap-free partition even for non-uniform spacing and after remeshing.
 *
 * Rest rotations are dimensionless integrated strains. The returned XPBD
 * compliances are the solver-independent angular compliances l/EI and l/GJ;
 * the solver must still apply its timestep scaling.
 */
export function discretizeKirchhoffProfile(
    profileOrType,
    materialNodeCoordinates,
    tipCoordinate,
    out = {},
    { rigidityScaleAtDistance = null } = {}
) {
    const profile = resolveProfile(profileOrType);
    if (
        !materialNodeCoordinates ||
        !Number.isInteger(materialNodeCoordinates.length) ||
        materialNodeCoordinates.length < 3
    ) {
        throw new RangeError('Kirchhoff discretization requires at least three material nodes');
    }
    if (!Number.isFinite(tipCoordinate)) {
        throw new TypeError('Material tip coordinate must be finite');
    }

    const count = materialNodeCoordinates.length;
    const materialSFromTip = prepareDiscretizationArray(
        out,
        'materialSFromTip',
        count
    );
    for (let index = 0; index < count; index++) {
        const coordinate = materialNodeCoordinates[index];
        if (!Number.isFinite(coordinate)) {
            throw new TypeError(`Material node coordinate ${index} must be finite`);
        }
        materialSFromTip[index] = Math.abs(tipCoordinate - coordinate);
        if (
            index > 0 &&
            materialSFromTip[index] >= materialSFromTip[index - 1] - 1e-12
        ) {
            throw new RangeError(
                'Material nodes must be strictly ordered from proximal end toward the tip'
            );
        }
    }

    const cellStartS = prepareDiscretizationArray(out, 'cellStartS', count);
    const cellEndS = prepareDiscretizationArray(out, 'cellEndS', count);
    const voronoiLength = prepareDiscretizationArray(out, 'voronoiLength', count);
    const restRotation1 = prepareDiscretizationArray(out, 'restRotation1', count);
    const restRotation2 = prepareDiscretizationArray(out, 'restRotation2', count);
    const restRotation3 = prepareDiscretizationArray(out, 'restRotation3', count);
    const EI1Mean = prepareDiscretizationArray(out, 'EI1Mean', count);
    const EI2Mean = prepareDiscretizationArray(out, 'EI2Mean', count);
    const GJMean = prepareDiscretizationArray(out, 'GJMean', count);
    const bendCompliance1 = prepareDiscretizationArray(out, 'bendCompliance1', count);
    const bendCompliance2 = prepareDiscretizationArray(out, 'bendCompliance2', count);
    const twistCompliance = prepareDiscretizationArray(out, 'twistCompliance', count);
    const integrated = out._integrated ??= {};
    const scaledRigiditySample = out._scaledRigiditySample ??= {};

    for (let joint = 1; joint < count - 1; joint++) {
        const proximalBoundaryCoordinate = joint === 1
            ? materialNodeCoordinates[0]
            : (materialNodeCoordinates[joint - 1] + materialNodeCoordinates[joint]) * 0.5;
        const distalBoundaryCoordinate = joint === count - 2
            ? materialNodeCoordinates[count - 1]
            : (materialNodeCoordinates[joint] + materialNodeCoordinates[joint + 1]) * 0.5;
        const proximalS = Math.abs(tipCoordinate - proximalBoundaryCoordinate);
        const distalS = Math.abs(tipCoordinate - distalBoundaryCoordinate);
        const startS = Math.min(proximalS, distalS);
        const endS = Math.max(proximalS, distalS);
        profile.integrate(startS, endS, integrated);
        if (typeof rigidityScaleAtDistance === 'function') {
            integrated.EI1Integral = integrateScaledRigidity(
                profile,
                'EI1',
                rigidityScaleAtDistance,
                startS,
                endS,
                scaledRigiditySample
            );
            integrated.EI2Integral = integrateScaledRigidity(
                profile,
                'EI2',
                rigidityScaleAtDistance,
                startS,
                endS,
                scaledRigiditySample
            );
            integrated.GJIntegral = integrateScaledRigidity(
                profile,
                'GJ',
                rigidityScaleAtDistance,
                startS,
                endS,
                scaledRigiditySample
            );
            integrated.EI1Mean = integrated.lengthMm > 0
                ? integrated.EI1Integral / integrated.lengthMm
                : profile.sample(startS, scaledRigiditySample).EI1 *
                    rigidityScaleAtDistance(startS);
            integrated.EI2Mean = integrated.lengthMm > 0
                ? integrated.EI2Integral / integrated.lengthMm
                : profile.sample(startS, scaledRigiditySample).EI2 *
                    rigidityScaleAtDistance(startS);
            integrated.GJMean = integrated.lengthMm > 0
                ? integrated.GJIntegral / integrated.lengthMm
                : profile.sample(startS, scaledRigiditySample).GJ *
                    rigidityScaleAtDistance(startS);
        }

        cellStartS[joint] = startS;
        cellEndS[joint] = endS;
        voronoiLength[joint] = integrated.lengthMm;
        restRotation1[joint] = integrated.kappa01Integral;
        restRotation2[joint] = integrated.kappa02Integral;
        restRotation3[joint] = integrated.tau0Integral;
        EI1Mean[joint] = integrated.EI1Mean;
        EI2Mean[joint] = integrated.EI2Mean;
        GJMean[joint] = integrated.GJMean;
        bendCompliance1[joint] = integrated.lengthMm / integrated.EI1Mean;
        bendCompliance2[joint] = integrated.lengthMm / integrated.EI2Mean;
        twistCompliance[joint] = integrated.lengthMm / integrated.GJMean;
    }

    out.profileId = profile.id;
    out.nodeCount = count;
    out.jointStart = 1;
    out.jointEnd = count - 2;
    out.tipCoordinate = tipCoordinate;
    // Generic strain-component aliases make a future vector XPBD constraint
    // possible without obscuring the physically named bend/twist quantities.
    out.compliance1 = bendCompliance1;
    out.compliance2 = bendCompliance2;
    out.compliance3 = twistCompliance;
    return out;
}
