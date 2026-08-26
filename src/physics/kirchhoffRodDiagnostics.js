import {
    conjugateQuaternion,
    evaluateAdaptationConstraint,
    evaluateBendTwistConstraint,
    multiplyQuaternions,
    quaternionLog
} from './discreteKirchhoffRod.js';

const EPSILON = 1e-12;

function clamp(value, min, max) {
    return Math.max(min, Math.min(max, value));
}

function magnitude3(x, y, z) {
    return Math.hypot(x, y, z);
}

function valueAt(source, index, body, fallback = undefined) {
    if (typeof source === 'function') {
        const value = source(index, body);
        return Number.isFinite(value) ? value : fallback;
    }
    if (Number.isFinite(source)) return source;
    if (source && Number.isFinite(source[index])) return source[index];
    return fallback;
}

function quaternionAt(body, segment, previous = false) {
    const prefix = previous ? 'previousOrientation' : 'orientation';
    return {
        x: body[`${prefix}X`]?.[segment] ?? 0,
        y: body[`${prefix}Y`]?.[segment] ?? 0,
        z: body[`${prefix}Z`]?.[segment] ?? 0,
        w: body[`${prefix}W`]?.[segment] ?? 1
    };
}

function createAccumulator() {
    return {
        maximum: 0,
        maximumIndex: -1,
        sumSquared: 0,
        count: 0
    };
}

function accumulate(accumulator, value, index) {
    const absolute = Math.abs(value);
    if (absolute > accumulator.maximum) {
        accumulator.maximum = absolute;
        accumulator.maximumIndex = index;
    }
    accumulator.sumSquared += value * value;
    accumulator.count++;
}

function finishAccumulator(accumulator) {
    return {
        max: accumulator.maximum,
        rms: accumulator.count > 0
            ? Math.sqrt(accumulator.sumSquared / accumulator.count)
            : 0,
        maxIndex: accumulator.maximumIndex,
        count: accumulator.count
    };
}

function resolveStiffness({
    explicit,
    bodyExplicit,
    compliance,
    index,
    body,
    voronoiLength,
    hardConstraintStiffness,
    maximumDiagnosticStiffness
}) {
    const configured = valueAt(
        explicit,
        index,
        body,
        valueAt(bodyExplicit, index, body, undefined)
    );
    if (Number.isFinite(configured) && configured >= 0) {
        return Math.min(configured, maximumDiagnosticStiffness);
    }
    const complianceValue = valueAt(compliance, index, body, undefined);
    if (Number.isFinite(complianceValue) && complianceValue > EPSILON) {
        return Math.min(
            voronoiLength / complianceValue,
            maximumDiagnosticStiffness
        );
    }
    // A hard XPBD constraint has infinite physical stiffness. Diagnostics need
    // a finite norm, so hard constraints use a documented reference rigidity.
    return Math.min(hardConstraintStiffness, maximumDiagnosticStiffness);
}

function discreteCurvatureSamples(body, startJoint, endJoint) {
    const samples = [];
    for (let joint = startJoint; joint < endJoint; joint++) {
        let incomingX = body.x[joint] - body.x[joint - 1];
        let incomingY = body.y[joint] - body.y[joint - 1];
        let incomingZ = body.z[joint] - body.z[joint - 1];
        let outgoingX = body.x[joint + 1] - body.x[joint];
        let outgoingY = body.y[joint + 1] - body.y[joint];
        let outgoingZ = body.z[joint + 1] - body.z[joint];
        const incomingLength = magnitude3(incomingX, incomingY, incomingZ);
        const outgoingLength = magnitude3(outgoingX, outgoingY, outgoingZ);
        if (incomingLength < EPSILON || outgoingLength < EPSILON) {
            samples.push({ x: 0, y: 0, z: 0 });
            continue;
        }
        incomingX /= incomingLength;
        incomingY /= incomingLength;
        incomingZ /= incomingLength;
        outgoingX /= outgoingLength;
        outgoingY /= outgoingLength;
        outgoingZ /= outgoingLength;
        const cosine = clamp(
            incomingX * outgoingX +
                incomingY * outgoingY +
                incomingZ * outgoingZ,
            -1,
            1
        );
        const denominator = Math.max(EPSILON, 1 + cosine);
        const scale = 2 / denominator;
        samples.push({
            x: (incomingY * outgoingZ - incomingZ * outgoingY) * scale,
            y: (incomingZ * outgoingX - incomingX * outgoingZ) * scale,
            z: (incomingX * outgoingY - incomingY * outgoingX) * scale
        });
    }
    return samples;
}

/**
 * One-sided DFT of the vector-valued discrete curvature. DC is intentionally
 * retained: a circular arc is a low-frequency curvature field, while the
 * alternating hinge glitch concentrates energy close to Nyquist.
 */
export function curvatureSpectrumDiagnostics(
    samples,
    {
        highModeFraction = 0.25,
        smoothThreshold = 0.12,
        alternatingThreshold = 0.35
    } = {}
) {
    const count = samples.length;
    if (count === 0) {
        return {
            sampleCount: 0,
            nyquistMode: 0,
            highModeStart: 0,
            totalEnergy: 0,
            highFrequencyEnergy: 0,
            highFrequencyShare: 0,
            classification: 'straight'
        };
    }
    const nyquist = Math.floor(count / 2);
    const fraction = clamp(highModeFraction, 0, 1);
    const highModeStart = nyquist > 0
        ? Math.max(1, Math.ceil(nyquist * (1 - fraction)))
        : 0;
    let totalEnergy = 0;
    let highFrequencyEnergy = 0;
    for (let mode = 0; mode <= nyquist; mode++) {
        let realX = 0;
        let realY = 0;
        let realZ = 0;
        let imaginaryX = 0;
        let imaginaryY = 0;
        let imaginaryZ = 0;
        for (let sample = 0; sample < count; sample++) {
            const angle = -2 * Math.PI * mode * sample / count;
            const cosine = Math.cos(angle);
            const sine = Math.sin(angle);
            const curvature = samples[sample];
            realX += curvature.x * cosine;
            realY += curvature.y * cosine;
            realZ += curvature.z * cosine;
            imaginaryX += curvature.x * sine;
            imaginaryY += curvature.y * sine;
            imaginaryZ += curvature.z * sine;
        }
        let energy = (
            realX * realX + realY * realY + realZ * realZ +
            imaginaryX * imaginaryX +
            imaginaryY * imaginaryY + imaginaryZ * imaginaryZ
        ) / (count * count);
        const isNyquist = count % 2 === 0 && mode === nyquist;
        if (mode > 0 && !isNyquist) energy *= 2;
        totalEnergy += energy;
        if (mode >= highModeStart) highFrequencyEnergy += energy;
    }
    const highFrequencyShare = totalEnergy > EPSILON
        ? clamp(highFrequencyEnergy / totalEnergy, 0, 1)
        : 0;
    let classification = 'mixed';
    if (totalEnergy <= EPSILON) classification = 'straight';
    else if (highFrequencyShare <= smoothThreshold) classification = 'smooth-buckling';
    else if (highFrequencyShare >= alternatingThreshold) classification = 'alternating-glitch';
    return {
        sampleCount: count,
        nyquistMode: nyquist,
        highModeStart,
        totalEnergy,
        highFrequencyEnergy,
        highFrequencyShare,
        classification
    };
}

export function kirchhoffClosureState(
    diagnostics,
    {
        adaptationTolerance = 0.002,
        bendTolerance = 0.01,
        twistTolerance = 0.01,
        displacementTolerance = 0.01,
        angularDisplacementTolerance = 0.01
    } = {}
) {
    const ratios = {
        adaptation: diagnostics.adaptation.max /
            Math.max(EPSILON, adaptationTolerance),
        bend1: diagnostics.bend1.max / Math.max(EPSILON, bendTolerance),
        bend2: diagnostics.bend2.max / Math.max(EPSILON, bendTolerance),
        twist: diagnostics.twist.max / Math.max(EPSILON, twistTolerance),
        displacement: diagnostics.motion.maxDeltaX /
            Math.max(EPSILON, displacementTolerance),
        angularDisplacement: diagnostics.motion.maxDeltaTheta /
            Math.max(EPSILON, angularDisplacementTolerance)
    };
    let maximumNormalizedResidual = 0;
    let dominantResidual = 'adaptation';
    for (const [name, ratio] of Object.entries(ratios)) {
        if (ratio <= maximumNormalizedResidual) continue;
        maximumNormalizedResidual = ratio;
        dominantResidual = name;
    }
    return {
        converged: maximumNormalizedResidual <= 1,
        maximumNormalizedResidual,
        dominantResidual,
        ratios
    };
}

/**
 * Read-only diagnostics for a Kirchhoff-enabled EndovascularRodBody. Residuals
 * are reported in millimetres (adaptation/displacement) and radians
 * (bend/twist/angular displacement).
 */
export function computeKirchhoffRodDiagnostics(body, options = {}) {
    if (!body || !body.x || !body.orientationW) {
        throw new TypeError('A Kirchhoff rod body with position and orientation arrays is required');
    }
    const nodeStart = clamp(
        Math.floor(options.nodeStart ?? body.activeStart ?? 0),
        0,
        body.count - 1
    );
    const nodeEnd = clamp(
        Math.floor(options.nodeEnd ?? body.activeEnd ?? body.count - 1),
        nodeStart,
        body.count - 1
    );
    const segmentStart = Math.max(0, nodeStart);
    const segmentEnd = Math.min(body.segmentCount, nodeEnd);
    const jointStart = Math.max(1, nodeStart + 1);
    const jointEnd = Math.min(body.segmentCount, nodeEnd);

    const adaptationAccumulator = createAccumulator();
    const bend1Accumulator = createAccumulator();
    const bend2Accumulator = createAccumulator();
    const twistAccumulator = createAccumulator();
    const deltaXAccumulator = createAccumulator();
    const deltaThetaAccumulator = createAccumulator();

    for (let node = nodeStart; node <= nodeEnd; node++) {
        const delta = magnitude3(
            body.x[node] - (body.previousX?.[node] ?? body.x[node]),
            body.y[node] - (body.previousY?.[node] ?? body.y[node]),
            body.z[node] - (body.previousZ?.[node] ?? body.z[node])
        );
        accumulate(deltaXAccumulator, delta, node);
    }

    for (let segment = segmentStart; segment < segmentEnd; segment++) {
        const orientation = quaternionAt(body, segment);
        const adaptation = evaluateAdaptationConstraint(
            { x: body.x[segment], y: body.y[segment], z: body.z[segment] },
            {
                x: body.x[segment + 1],
                y: body.y[segment + 1],
                z: body.z[segment + 1]
            },
            orientation,
            body.restLength[segment],
            {}
        );
        accumulate(adaptationAccumulator, adaptation.residual, segment);

        const previous = quaternionAt(body, segment, true);
        const worldDelta = multiplyQuaternions(
            orientation,
            conjugateQuaternion(previous, {}),
            {}
        );
        const angularDelta = quaternionLog(worldDelta, {});
        accumulate(
            deltaThetaAccumulator,
            magnitude3(angularDelta.x, angularDelta.y, angularDelta.z),
            segment
        );
    }

    const hardConstraintStiffness = Math.max(
        0,
        options.hardConstraintStiffness ?? 1
    );
    const maximumDiagnosticStiffness = Math.max(
        EPSILON,
        options.maximumDiagnosticStiffness ?? 1e15
    );
    let bend1Energy = 0;
    let bend2Energy = 0;
    let twistEnergy = 0;
    for (let joint = jointStart; joint < jointEnd; joint++) {
        const state = evaluateBendTwistConstraint(
            quaternionAt(body, joint - 1),
            quaternionAt(body, joint),
            {
                x: body.restRotation1?.[joint] ?? 0,
                y: body.restRotation2?.[joint] ?? 0,
                z: body.restRotation3?.[joint] ?? 0
            },
            {}
        );
        accumulate(bend1Accumulator, state.strain.x, joint);
        accumulate(bend2Accumulator, state.strain.y, joint);
        accumulate(twistAccumulator, state.strain.z, joint);
        const voronoiLength = Math.max(EPSILON, 0.5 * (
            body.restLength[joint - 1] + body.restLength[joint]
        ));
        const stiffness1 = resolveStiffness({
            explicit: options.bendStiffness1 ?? options.EI1,
            bodyExplicit: body.kirchhoffBendStiffness1 ?? body.EI1,
            compliance: body.kirchhoffBendCompliance1,
            index: joint,
            body,
            voronoiLength,
            hardConstraintStiffness,
            maximumDiagnosticStiffness
        });
        const stiffness2 = resolveStiffness({
            explicit: options.bendStiffness2 ?? options.EI2,
            bodyExplicit: body.kirchhoffBendStiffness2 ?? body.EI2,
            compliance: body.kirchhoffBendCompliance2,
            index: joint,
            body,
            voronoiLength,
            hardConstraintStiffness,
            maximumDiagnosticStiffness
        });
        const torsionalStiffness = resolveStiffness({
            explicit: options.twistStiffness ?? options.GJ,
            bodyExplicit: body.kirchhoffTwistStiffness ?? body.GJ,
            compliance: body.kirchhoffTwistCompliance,
            index: joint,
            body,
            voronoiLength,
            hardConstraintStiffness,
            maximumDiagnosticStiffness
        });
        bend1Energy += 0.5 * stiffness1 / voronoiLength *
            state.strain.x * state.strain.x;
        bend2Energy += 0.5 * stiffness2 / voronoiLength *
            state.strain.y * state.strain.y;
        twistEnergy += 0.5 * torsionalStiffness / voronoiLength *
            state.strain.z * state.strain.z;
    }

    const adaptation = finishAccumulator(adaptationAccumulator);
    const bend1 = finishAccumulator(bend1Accumulator);
    const bend2 = finishAccumulator(bend2Accumulator);
    const twist = finishAccumulator(twistAccumulator);
    const deltaX = finishAccumulator(deltaXAccumulator);
    const deltaTheta = finishAccumulator(deltaThetaAccumulator);
    const spectrum = curvatureSpectrumDiagnostics(
        discreteCurvatureSamples(body, jointStart, jointEnd),
        options.spectrum
    );
    const diagnostics = {
        range: {
            nodeStart,
            nodeEnd,
            segmentStart,
            segmentEnd,
            jointStart,
            jointEnd
        },
        adaptation,
        bend1,
        bend2,
        twist,
        motion: {
            maxDeltaX: deltaX.max,
            rmsDeltaX: deltaX.rms,
            maxDeltaXNode: deltaX.maxIndex,
            maxDeltaTheta: deltaTheta.max,
            rmsDeltaTheta: deltaTheta.rms,
            maxDeltaThetaSegment: deltaTheta.maxIndex
        },
        energy: {
            bend1: bend1Energy,
            bend2: bend2Energy,
            bend: bend1Energy + bend2Energy,
            twist: twistEnergy,
            total: bend1Energy + bend2Energy + twistEnergy
        },
        spectrum,
        // Flat aliases make the object convenient for frame telemetry without
        // discarding the more descriptive nested representation.
        maxAdaptationResidual: adaptation.max,
        rmsAdaptationResidual: adaptation.rms,
        maxBend1Residual: bend1.max,
        rmsBend1Residual: bend1.rms,
        maxBend2Residual: bend2.max,
        rmsBend2Residual: bend2.rms,
        maxTwistResidual: twist.max,
        rmsTwistResidual: twist.rms,
        maxDeltaX: deltaX.max,
        maxDeltaTheta: deltaTheta.max,
        highFrequencyCurvatureEnergyShare: spectrum.highFrequencyShare
    };
    diagnostics.closure = kirchhoffClosureState(
        diagnostics,
        options.closure
    );
    return diagnostics;
}

