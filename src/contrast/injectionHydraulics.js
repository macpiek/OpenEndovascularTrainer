const ML_PER_SECOND_TO_M3_PER_SECOND = 1e-6;
const MM_TO_M = 1e-3;
const MICROMETER_TO_M = 1e-6;
const PSI_TO_PA = 6894.757293168;
const MIN_REYNOLDS = 1e-9;
const MAXIMUM_RATE_SEARCH_LIMIT_ML_PER_SEC = 1e6;

export const DEFAULT_INJECTOR_HYDRAULICS = Object.freeze({
    label: 'Angiographic power injector',
    maximumPressurePsi: 1200
});

export const DEFAULT_DEVICE_HYDRAULIC_PROFILES = Object.freeze({
    sheath: Object.freeze({
        id: 'sheath-6f-training',
        label: '6F introducer sheath',
        lengthMm: 110,
        innerDiameterMm: 1.8,
        roughnessMicrometer: 1.5,
        outletDischargeCoefficient: 0.82,
        maximumPressurePsi: 300
    }),
    berenstein: Object.freeze({
        id: 'berenstein-5f-training',
        label: '5F Berenstein',
        lengthMm: 1000,
        innerDiameterMm: 0.97,
        roughnessMicrometer: 1.5,
        outletDischargeCoefficient: 0.82,
        maximumPressurePsi: 1050
    }),
    pigtail: Object.freeze({
        id: 'pigtail-5f-training',
        label: '5F pigtail',
        lengthMm: 1000,
        innerDiameterMm: 0.97,
        roughnessMicrometer: 1.5,
        outletDischargeCoefficient: 0.76,
        maximumPressurePsi: 1200
    })
});

function positiveFinite(name, value) {
    if (!(Number.isFinite(value) && value > 0)) {
        throw new RangeError(`${name} must be a positive finite number`);
    }
    return value;
}

function clamp(value, minimum, maximum) {
    return Math.max(minimum, Math.min(maximum, value));
}

function portAreaMm2(port) {
    if (Number.isFinite(port?.areaMm2) && port.areaMm2 > 0) {
        return port.areaMm2;
    }
    const radiusMm = positiveFinite('port radius', port?.radiusMm);
    return Math.PI * radiusMm ** 2;
}

function normalizedPortDirection(port) {
    const x = Number(port?.direction?.x ?? port?.direction?.[0] ?? 0);
    const y = Number(port?.direction?.y ?? port?.direction?.[1] ?? 0);
    const z = Number(port?.direction?.z ?? port?.direction?.[2] ?? 0);
    const length = Math.hypot(x, y, z);
    if (!(length > 0)) return { x: 0, y: 0, z: 0 };
    return { x: x / length, y: y / length, z: z / length };
}

export function normalizeDeviceHydraulicProfile(profile, fallback = null) {
    const source = { ...(fallback || {}), ...(profile || {}) };
    const normalized = {
        id: String(source.id || 'custom-device'),
        label: String(source.label || 'Custom injection device'),
        lengthMm: positiveFinite('device length', source.lengthMm),
        innerDiameterMm: positiveFinite(
            'device inner diameter',
            source.innerDiameterMm
        ),
        roughnessMicrometer: positiveFinite(
            'device roughness',
            source.roughnessMicrometer
        ),
        outletDischargeCoefficient: positiveFinite(
            'outlet discharge coefficient',
            source.outletDischargeCoefficient
        ),
        maximumPressurePsi: positiveFinite(
            'device maximum pressure',
            source.maximumPressurePsi
        )
    };
    if (normalized.outletDischargeCoefficient > 1) {
        throw new RangeError(
            'outlet discharge coefficient must not exceed one'
        );
    }
    if (normalized.innerDiameterMm > 20) {
        throw new RangeError('device inner diameter exceeds supported range');
    }
    if (normalized.maximumPressurePsi > 5000) {
        throw new RangeError('device maximum pressure exceeds supported range');
    }
    return normalized;
}

export function normalizeInjectorHydraulics(settings = {}) {
    return {
        label: String(
            settings.label || DEFAULT_INJECTOR_HYDRAULICS.label
        ),
        maximumPressurePsi: positiveFinite(
            'injector maximum pressure',
            settings.maximumPressurePsi ??
                DEFAULT_INJECTOR_HYDRAULICS.maximumPressurePsi
        )
    };
}

function darcyFrictionFactor(reynoldsNumber, relativeRoughness) {
    const reynolds = Math.max(MIN_REYNOLDS, reynoldsNumber);
    const laminar = 64 / reynolds;
    if (reynolds <= 2300) return laminar;
    const turbulent = 0.25 / Math.log10(
        relativeRoughness / 3.7 + 5.74 / reynolds ** 0.9
    ) ** 2;
    if (reynolds >= 4000) return turbulent;
    const transition = (reynolds - 2300) / (4000 - 2300);
    const smoothTransition = transition ** 2 * (3 - 2 * transition);
    return laminar + (turbulent - laminar) * smoothTransition;
}

function pressureAtRate({
    rateMlPerSec,
    profile,
    medium,
    totalOutletAreaMm2
}) {
    const rateM3PerSec = Math.max(0, rateMlPerSec) *
        ML_PER_SECOND_TO_M3_PER_SECOND;
    const densityKgPerM3 = positiveFinite(
        'contrast density',
        medium.densityKgPerM3
    );
    const viscosityPaS = positiveFinite(
        'contrast viscosity',
        medium.viscosityPaS
    );
    const diameterM = profile.innerDiameterMm * MM_TO_M;
    const lengthM = profile.lengthMm * MM_TO_M;
    const tubeAreaM2 = Math.PI * diameterM ** 2 / 4;
    const tubeVelocityMPerSec = rateM3PerSec / tubeAreaM2;
    const reynoldsNumber = densityKgPerM3 *
        tubeVelocityMPerSec * diameterM / viscosityPaS;
    const relativeRoughness = profile.roughnessMicrometer *
        MICROMETER_TO_M / diameterM;
    const frictionFactor = rateM3PerSec > 0
        ? darcyFrictionFactor(reynoldsNumber, relativeRoughness)
        : 0;
    const dynamicPressurePa = densityKgPerM3 *
        tubeVelocityMPerSec ** 2 / 2;
    const tubePressurePa = frictionFactor *
        lengthM / diameterM * dynamicPressurePa;
    const outletAreaM2 = positiveFinite(
        'total outlet area',
        totalOutletAreaMm2
    ) * 1e-6;
    const outletVelocityMPerSec = rateM3PerSec /
        (profile.outletDischargeCoefficient * outletAreaM2);
    const outletPressurePa = densityKgPerM3 *
        outletVelocityMPerSec ** 2 / 2;
    return {
        pressurePa: tubePressurePa + outletPressurePa,
        tubePressurePa,
        outletPressurePa,
        tubeVelocityMPerSec,
        outletVelocityMPerSec,
        reynoldsNumber,
        frictionFactor,
        flowRegime: reynoldsNumber < 2300
            ? 'laminar'
            : reynoldsNumber < 4000
                ? 'transitional'
                : 'turbulent'
    };
}

function resolvePortDistribution(ports) {
    if (!Array.isArray(ports) || !ports.length) {
        throw new RangeError('at least one injection outlet is required');
    }
    const outletAreasMm2 = ports.map(portAreaMm2);
    let weights = ports.map((port, index) => {
        const explicitWeight = port?.weight;
        return Number.isFinite(explicitWeight) && explicitWeight > 0
            ? explicitWeight
            : outletAreasMm2[index];
    });
    let totalWeight = weights.reduce((sum, weight) => sum + weight, 0);
    if (!(totalWeight > 0)) {
        weights = [...outletAreasMm2];
        totalWeight = weights.reduce((sum, weight) => sum + weight, 0);
    }
    return {
        outletAreasMm2,
        normalizedWeights: weights.map(weight => weight / totalWeight),
        totalOutletAreaMm2: outletAreasMm2.reduce(
            (sum, area) => sum + area,
            0
        )
    };
}

function maximumRateAtPressureLimit({
    profile,
    medium,
    totalOutletAreaMm2,
    pressureLimitPa
}) {
    let lowerRate = 0;
    let upperRate = 1;
    while (
        upperRate < MAXIMUM_RATE_SEARCH_LIMIT_ML_PER_SEC &&
        pressureAtRate({
            rateMlPerSec: upperRate,
            profile,
            medium,
            totalOutletAreaMm2
        }).pressurePa <= pressureLimitPa
    ) {
        lowerRate = upperRate;
        upperRate *= 2;
    }
    upperRate = Math.min(
        upperRate,
        MAXIMUM_RATE_SEARCH_LIMIT_ML_PER_SEC
    );
    for (let iteration = 0; iteration < 64; iteration++) {
        const candidateRate = (lowerRate + upperRate) / 2;
        const candidatePressure = pressureAtRate({
            rateMlPerSec: candidateRate,
            profile,
            medium,
            totalOutletAreaMm2
        }).pressurePa;
        if (candidatePressure <= pressureLimitPa) {
            lowerRate = candidateRate;
        } else {
            upperRate = candidateRate;
        }
    }
    return lowerRate;
}

export function computeInjectionHydraulics({
    requestedRateMlPerSec,
    ports,
    deviceProfile,
    injector = DEFAULT_INJECTOR_HYDRAULICS,
    medium
}) {
    const requestedRate = positiveFinite(
        'requested injection rate',
        requestedRateMlPerSec
    );
    const profile = normalizeDeviceHydraulicProfile(deviceProfile);
    const injectorProfile = normalizeInjectorHydraulics(injector);
    const distribution = resolvePortDistribution(ports);
    const pressureLimitPsi = Math.min(
        injectorProfile.maximumPressurePsi,
        profile.maximumPressurePsi
    );
    const pressureLimitPa = pressureLimitPsi * PSI_TO_PA;
    const requestedState = pressureAtRate({
        rateMlPerSec: requestedRate,
        profile,
        medium,
        totalOutletAreaMm2: distribution.totalOutletAreaMm2
    });
    const maximumAchievableRateMlPerSec = maximumRateAtPressureLimit({
        profile,
        medium,
        totalOutletAreaMm2: distribution.totalOutletAreaMm2,
        pressureLimitPa
    });
    const actualRate = Math.min(
        requestedRate,
        maximumAchievableRateMlPerSec
    );
    const actualState = pressureAtRate({
        rateMlPerSec: actualRate,
        profile,
        medium,
        totalOutletAreaMm2: distribution.totalOutletAreaMm2
    });
    const pressureLimited = actualRate < requestedRate * (1 - 1e-6);
    const limitingComponent = pressureLimited
        ? injectorProfile.maximumPressurePsi <= profile.maximumPressurePsi
            ? 'injector'
            : 'device'
        : 'requested-rate';
    const portFlowRatesMlPerSec = distribution.normalizedWeights.map(
        weight => actualRate * weight
    );
    const portJetVelocitiesMPerSec = portFlowRatesMlPerSec.map(
        (portRate, index) =>
            portRate * ML_PER_SECOND_TO_M3_PER_SECOND /
            (distribution.outletAreasMm2[index] * 1e-6)
    );
    return {
        deviceId: profile.id,
        deviceLabel: profile.label,
        requestedRateMlPerSec: requestedRate,
        actualRateMlPerSec: actualRate,
        maximumAchievableRateMlPerSec,
        achievableRateFraction: actualRate / requestedRate,
        pressureLimited,
        limitingComponent,
        requiredPressurePsi: requestedState.pressurePa / PSI_TO_PA,
        appliedPressurePsi: actualState.pressurePa / PSI_TO_PA,
        pressureLimitPsi,
        injectorPressureLimitPsi: injectorProfile.maximumPressurePsi,
        devicePressureLimitPsi: profile.maximumPressurePsi,
        tubePressurePsi: actualState.tubePressurePa / PSI_TO_PA,
        outletPressurePsi: actualState.outletPressurePa / PSI_TO_PA,
        tubeVelocityMPerSec: actualState.tubeVelocityMPerSec,
        meanOutletVelocityMPerSec: actualState.outletVelocityMPerSec,
        maximumPortJetVelocityMPerSec: Math.max(
            ...portJetVelocitiesMPerSec
        ),
        reynoldsNumber: actualState.reynoldsNumber,
        frictionFactor: actualState.frictionFactor,
        flowRegime: actualState.flowRegime,
        outletCount: ports.length,
        totalOutletAreaMm2: distribution.totalOutletAreaMm2,
        outletAreasMm2: [...distribution.outletAreasMm2],
        portDirections: ports.map(normalizedPortDirection),
        portFlowRatesMlPerSec,
        portJetVelocitiesMPerSec,
        viscosityPaS: medium.viscosityPaS,
        densityKgPerM3: medium.densityKgPerM3,
        deviceProfile: { ...profile }
    };
}

export function mergeDeviceHydraulicProfiles(overrides = {}) {
    const result = {};
    for (const [deviceKey, fallback] of Object.entries(
        DEFAULT_DEVICE_HYDRAULIC_PROFILES
    )) {
        result[deviceKey] = normalizeDeviceHydraulicProfile(
            overrides[deviceKey],
            fallback
        );
    }
    return result;
}

export function pressurePsiToPa(pressurePsi) {
    return pressurePsi * PSI_TO_PA;
}

export function pressurePaToPsi(pressurePa) {
    return pressurePa / PSI_TO_PA;
}
