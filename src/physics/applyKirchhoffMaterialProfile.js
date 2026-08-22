import {
    discretizeKirchhoffProfile,
    kirchhoffMaterialProfile
} from './kirchhoffMaterialProfile.js';

const CONSTITUTIVE_TOLERANCE = 1e-12;

function constitutiveValueChanged(current, next) {
    if (!Number.isFinite(current) || !Number.isFinite(next)) {
        return current !== next;
    }
    return Math.abs(current - next) > CONSTITUTIVE_TOLERANCE * (
        1 + Math.max(Math.abs(current), Math.abs(next))
    );
}

function resolveProfile(profileOrType) {
    if (profileOrType?.sample && profileOrType?.integrate) return profileOrType;
    return kirchhoffMaterialProfile(profileOrType);
}

function activeMaterialCoordinates(
    materialCoordinates,
    bodyCount,
    activeStart,
    activeEnd,
    out = undefined
) {
    if (!materialCoordinates || !Number.isInteger(materialCoordinates.length)) {
        throw new TypeError('Material coordinates must be an array-like value');
    }
    const activeCount = activeEnd - activeStart + 1;
    const coordinates = out?.length === activeCount
        ? out
        : new Float64Array(activeCount);
    if (materialCoordinates.length === activeCount) {
        coordinates.set(materialCoordinates);
        return coordinates;
    }
    if (materialCoordinates.length === bodyCount) {
        for (let local = 0; local < activeCount; local++) {
            coordinates[local] = materialCoordinates[activeStart + local];
        }
        return coordinates;
    }
    throw new RangeError(
        'Material coordinates must cover either the complete body or its active node range'
    );
}

function assertBodyContract(body) {
    const requiredArrays = [
        'materialCoordinate',
        'restRotation1',
        'restRotation2',
        'restRotation3',
        'kirchhoffBendCompliance1',
        'kirchhoffBendCompliance2',
        'kirchhoffTwistCompliance',
        'bendTwistLambda1',
        'bendTwistLambda2',
        'bendTwistLambda3'
    ];
    if (!body || !Number.isInteger(body.count) || body.count < 2) {
        throw new TypeError('A valid EndovascularRodBody is required');
    }
    for (const name of requiredArrays) {
        if (!body[name] || body[name].length !== body.count) {
            throw new TypeError(`Rod body does not expose ${name}`);
        }
    }
    if (
        typeof body.enableKirchhoff !== 'function' ||
        typeof body.setKirchhoffRestRotation !== 'function' ||
        typeof body.setActiveRange !== 'function'
    ) {
        throw new TypeError('Rod body does not expose the Kirchhoff material API');
    }
}

/**
 * Applies a manufactured Kirchhoff profile using material coordinates only.
 *
 * The live x/y/z pose is never sampled to define restRotation or compliance.
 * When a legacy body is migrated, its current geometry is used only to
 * initialize the current material frames, with rest rotation capture disabled.
 */
export function applyKirchhoffMaterialProfile(
    body,
    profileOrType,
    {
        activeStart = body?.activeStart ?? 0,
        activeEnd = body?.activeEnd ?? ((body?.count ?? 1) - 1),
        materialCoordinates = body?.materialCoordinate,
        tipCoordinate = undefined,
        stiffnessScale = 1,
        discretizationOut = undefined
    } = {}
) {
    assertBodyContract(body);
    if (!Number.isFinite(stiffnessScale) || stiffnessScale <= 0) {
        throw new RangeError('Kirchhoff stiffness scale must be finite and positive');
    }
    if (
        !Number.isInteger(activeStart) ||
        !Number.isInteger(activeEnd) ||
        activeStart < 0 ||
        activeEnd >= body.count ||
        activeEnd < activeStart
    ) {
        throw new RangeError('Active range must be a valid inclusive node range');
    }
    const profile = resolveProfile(profileOrType);
    const discretizationScratch = discretizationOut ?? {};
    const coordinates = activeMaterialCoordinates(
        materialCoordinates,
        body.count,
        activeStart,
        activeEnd,
        discretizationScratch._activeMaterialCoordinates
    );
    discretizationScratch._activeMaterialCoordinates = coordinates;
    const resolvedTipCoordinate = tipCoordinate ?? coordinates[coordinates.length - 1];
    if (!Number.isFinite(resolvedTipCoordinate)) {
        throw new TypeError('Material tip coordinate must be finite');
    }
    const wasKirchhoff = body.rodModel === 'kirchhoff';
    body.setActiveRange(activeStart, activeEnd);
    if (!wasKirchhoff) {
        body.enableKirchhoff(true, { captureRest: false });
        // This initializes current directors, angular history and adaptation
        // state. Passing false prevents the deformed live pose from becoming
        // the manufactured bend/twist rest state.
        body.captureKirchhoffRestConfiguration({ captureRestRotation: false });
    }

    let materialCoordinateChanges = 0;
    for (let local = 0; local < coordinates.length; local++) {
        const node = activeStart + local;
        const nextCoordinate = coordinates[local];
        if (!Number.isFinite(nextCoordinate)) {
            throw new TypeError(`Material node coordinate ${node} must be finite`);
        }
        if (body.materialCoordinate[node] !== nextCoordinate) {
            body.materialCoordinate[node] = nextCoordinate;
            materialCoordinateChanges++;
        }
    }

    const cachedCoordinates =
        discretizationScratch._lastMaterialCoordinates?.length ===
            coordinates.length
            ? discretizationScratch._lastMaterialCoordinates
            : null;
    let constitutiveInputUnchanged =
        cachedCoordinates !== null &&
        discretizationScratch._lastBody === body &&
        discretizationScratch._lastProfileId === profile.id &&
        discretizationScratch._lastStiffnessScale === stiffnessScale &&
        discretizationScratch._lastActiveStart === activeStart &&
        discretizationScratch._lastActiveEnd === activeEnd &&
        discretizationScratch._lastTipCoordinate === resolvedTipCoordinate;
    if (constitutiveInputUnchanged) {
        for (let local = 0; local < coordinates.length; local++) {
            if (cachedCoordinates[local] === coordinates[local]) continue;
            constitutiveInputUnchanged = false;
            break;
        }
    }
    if (constitutiveInputUnchanged) {
        const cachedResult = discretizationScratch._applicationResult;
        cachedResult.materialCoordinateChanges = materialCoordinateChanges;
        cachedResult.changedJoints.length = 0;
        return cachedResult;
    }

    if (coordinates.length < 3) {
        const result = discretizationScratch._applicationResult ??= {};
        result.profileId = profile.id;
        result.stiffnessScale = stiffnessScale;
        result.activeStart = activeStart;
        result.activeEnd = activeEnd;
        result.materialCoordinateChanges = materialCoordinateChanges;
        result.changedJoints = discretizationScratch._changedJoints ??= [];
        result.changedJoints.length = 0;
        result.discretization = null;
        return result;
    }

    const discretization = discretizeKirchhoffProfile(
        profile,
        coordinates,
        resolvedTipCoordinate,
        discretizationScratch
    );
    const changedJoints = discretizationScratch._changedJoints ??= [];
    changedJoints.length = 0;
    for (
        let localJoint = discretization.jointStart;
        localJoint <= discretization.jointEnd;
        localJoint++
    ) {
        const joint = activeStart + localJoint;
        // Scaling EI and GJ by the same multiplier preserves the material
        // profile, soft-tip taper and Poisson-ratio relationship. XPBD stores
        // the inverse quantity, so compliance scales by 1 / stiffnessScale.
        const bendCompliance1 =
            discretization.bendCompliance1[localJoint] / stiffnessScale;
        const bendCompliance2 =
            discretization.bendCompliance2[localJoint] / stiffnessScale;
        const twistCompliance =
            discretization.twistCompliance[localJoint] / stiffnessScale;
        const changed =
            constitutiveValueChanged(
                body.restRotation1[joint],
                discretization.restRotation1[localJoint]
            ) ||
            constitutiveValueChanged(
                body.restRotation2[joint],
                discretization.restRotation2[localJoint]
            ) ||
            constitutiveValueChanged(
                body.restRotation3[joint],
                discretization.restRotation3[localJoint]
            ) ||
            constitutiveValueChanged(
                body.kirchhoffBendCompliance1[joint],
                bendCompliance1
            ) ||
            constitutiveValueChanged(
                body.kirchhoffBendCompliance2[joint],
                bendCompliance2
            ) ||
            constitutiveValueChanged(
                body.kirchhoffTwistCompliance[joint],
                twistCompliance
            );
        if (!changed) continue;
        body.setKirchhoffRestRotation(
            joint,
            discretization.restRotation1[localJoint],
            discretization.restRotation2[localJoint],
            discretization.restRotation3[localJoint],
            bendCompliance1,
            bendCompliance2,
            twistCompliance
        );
        changedJoints.push(joint);
    }

    let lastCoordinates = discretizationScratch._lastMaterialCoordinates;
    if (!lastCoordinates || lastCoordinates.length !== coordinates.length) {
        lastCoordinates = new Float64Array(coordinates.length);
        discretizationScratch._lastMaterialCoordinates = lastCoordinates;
    }
    lastCoordinates.set(coordinates);
    discretizationScratch._lastBody = body;
    discretizationScratch._lastProfileId = profile.id;
    discretizationScratch._lastStiffnessScale = stiffnessScale;
    discretizationScratch._lastActiveStart = activeStart;
    discretizationScratch._lastActiveEnd = activeEnd;
    discretizationScratch._lastTipCoordinate = resolvedTipCoordinate;
    const result = discretizationScratch._applicationResult ??= {};
    result.profileId = profile.id;
    result.stiffnessScale = stiffnessScale;
    result.activeStart = activeStart;
    result.activeEnd = activeEnd;
    result.materialCoordinateChanges = materialCoordinateChanges;
    result.changedJoints = changedJoints;
    result.discretization = discretization;
    return result;
}
