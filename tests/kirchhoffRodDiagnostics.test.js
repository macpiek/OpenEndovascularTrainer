import assert from 'node:assert/strict';
import {
    computeKirchhoffRodDiagnostics,
    curvatureSpectrumDiagnostics,
    kirchhoffClosureState
} from '../src/physics/kirchhoffRodDiagnostics.js';
import {
    conjugateQuaternion,
    createBishopFrame,
    multiplyQuaternions,
    quaternionExp,
    quaternionLog,
    transportBishopFrame
} from '../src/physics/discreteKirchhoffRod.js';

function makeBody(points, { captureRest = false, compliance = 0.02 } = {}) {
    const count = points.length;
    const segmentCount = count - 1;
    const body = {
        count,
        segmentCount,
        activeStart: 0,
        activeEnd: count - 1,
        x: new Float64Array(count),
        y: new Float64Array(count),
        z: new Float64Array(count),
        previousX: new Float64Array(count),
        previousY: new Float64Array(count),
        previousZ: new Float64Array(count),
        restLength: new Float64Array(segmentCount),
        orientationX: new Float64Array(segmentCount),
        orientationY: new Float64Array(segmentCount),
        orientationZ: new Float64Array(segmentCount),
        orientationW: new Float64Array(segmentCount),
        previousOrientationX: new Float64Array(segmentCount),
        previousOrientationY: new Float64Array(segmentCount),
        previousOrientationZ: new Float64Array(segmentCount),
        previousOrientationW: new Float64Array(segmentCount),
        restRotation1: new Float64Array(count),
        restRotation2: new Float64Array(count),
        restRotation3: new Float64Array(count),
        kirchhoffBendCompliance1: new Float64Array(count),
        kirchhoffBendCompliance2: new Float64Array(count),
        kirchhoffTwistCompliance: new Float64Array(count)
    };
    body.kirchhoffBendCompliance1.fill(compliance);
    body.kirchhoffBendCompliance2.fill(compliance * 1.5);
    body.kirchhoffTwistCompliance.fill(compliance * 0.75);
    for (let index = 0; index < count; index++) {
        body.x[index] = points[index].x;
        body.y[index] = points[index].y;
        body.z[index] = points[index].z ?? 0;
        body.previousX[index] = body.x[index];
        body.previousY[index] = body.y[index];
        body.previousZ[index] = body.z[index];
    }
    let previousFrame = null;
    for (let segment = 0; segment < segmentCount; segment++) {
        const edge = {
            x: body.x[segment + 1] - body.x[segment],
            y: body.y[segment + 1] - body.y[segment],
            z: body.z[segment + 1] - body.z[segment]
        };
        body.restLength[segment] = Math.hypot(edge.x, edge.y, edge.z);
        const frame = previousFrame
            ? transportBishopFrame(previousFrame, edge)
            : createBishopFrame(edge);
        body.orientationX[segment] = frame.x;
        body.orientationY[segment] = frame.y;
        body.orientationZ[segment] = frame.z;
        body.orientationW[segment] = frame.w;
        body.previousOrientationX[segment] = frame.x;
        body.previousOrientationY[segment] = frame.y;
        body.previousOrientationZ[segment] = frame.z;
        body.previousOrientationW[segment] = frame.w;
        if (captureRest && previousFrame) {
            const relative = multiplyQuaternions(
                conjugateQuaternion(previousFrame),
                frame
            );
            const rest = quaternionLog(relative);
            body.restRotation1[segment] = rest.x;
            body.restRotation2[segment] = rest.y;
            body.restRotation3[segment] = rest.z;
        }
        previousFrame = frame;
    }
    return body;
}

function circularArcPoints(count, radius = 20, stepAngle = 0.08) {
    const points = [];
    for (let index = 0; index < count; index++) {
        const angle = index * stepAngle;
        points.push({
            x: radius * Math.sin(angle),
            y: radius * (1 - Math.cos(angle)),
            z: 0
        });
    }
    return points;
}

function alternatingPoints(count, spacing = 2, amplitude = 0.7) {
    return Array.from({ length: count }, (_, index) => ({
        x: index * spacing,
        y: index % 2 === 0 ? -amplitude : amplitude,
        z: 0
    }));
}

// When the live centerline and material frames equal the manufactured rest
// state, all structural residuals and elastic energies must be zero.
const exactRestBody = makeBody(circularArcPoints(30), { captureRest: true });
const exactRest = computeKirchhoffRodDiagnostics(exactRestBody);
assert.ok(exactRest.adaptation.max < 1e-11);
assert.ok(exactRest.adaptation.rms < 1e-11);
assert.ok(exactRest.bend1.max < 1e-11);
assert.ok(exactRest.bend2.max < 1e-11);
assert.ok(exactRest.twist.max < 1e-11);
assert.ok(exactRest.energy.total < 1e-20);
assert.equal(exactRest.closure.converged, true);
assert.ok(Number.isFinite(exactRest.energy.total));

// Smooth circular buckling is almost pure DC in the curvature field and must
// not be confused with a node-to-node numerical oscillation.
const smoothBody = makeBody(circularArcPoints(34), { captureRest: false });
const smooth = computeKirchhoffRodDiagnostics(smoothBody, {
    EI1: 4,
    EI2: 6,
    GJ: 3
});
assert.ok(
    smooth.spectrum.highFrequencyShare < 0.03,
    `smooth spectrum share ${smooth.spectrum.highFrequencyShare}`
);
assert.equal(smooth.spectrum.classification, 'smooth-buckling');
assert.ok(smooth.energy.bend > 0);
assert.ok(Number.isFinite(smooth.energy.total));

// Alternating hinge curvature concentrates its energy at Nyquist and is the
// characteristic spectral signature of the observed sinusoidal/zig-zag glitch.
const alternatingBody = makeBody(alternatingPoints(34), { captureRest: false });
const alternating = computeKirchhoffRodDiagnostics(alternatingBody);
assert.ok(
    alternating.spectrum.highFrequencyShare > 0.7,
    `alternating spectrum share ${alternating.spectrum.highFrequencyShare}`
);
assert.equal(alternating.spectrum.classification, 'alternating-glitch');
assert.ok(alternating.energy.bend > 0);
assert.ok(Number.isFinite(alternating.energy.bend));
assert.ok(Number.isFinite(alternating.energy.twist));
assert.ok(Number.isFinite(alternating.energy.total));

// The standalone spectrum API is also usable by telemetry that has already
// sampled curvature outside the body diagnostic.
const directSpectrum = curvatureSpectrumDiagnostics(
    Array.from({ length: 32 }, (_, index) => ({
        x: 0,
        y: 0,
        z: index % 2 === 0 ? 1 : -1
    }))
);
assert.ok(directSpectrum.highFrequencyShare > 0.99);
assert.equal(directSpectrum.classification, 'alternating-glitch');

// Frame-to-frame movement is reported independently of constitutive residuals
// and participates in residual-driven closure.
const movingBody = makeBody(circularArcPoints(20), { captureRest: true });
movingBody.previousX[5] -= 0.25;
const currentFrame = {
    x: movingBody.orientationX[4],
    y: movingBody.orientationY[4],
    z: movingBody.orientationZ[4],
    w: movingBody.orientationW[4]
};
const previousFrame = multiplyQuaternions(
    quaternionExp({ x: 0, y: 0, z: -0.08 }),
    currentFrame
);
movingBody.previousOrientationX[4] = previousFrame.x;
movingBody.previousOrientationY[4] = previousFrame.y;
movingBody.previousOrientationZ[4] = previousFrame.z;
movingBody.previousOrientationW[4] = previousFrame.w;
const moving = computeKirchhoffRodDiagnostics(movingBody);
assert.ok(Math.abs(moving.motion.maxDeltaX - 0.25) < 1e-12);
assert.ok(Math.abs(moving.motion.maxDeltaTheta - 0.08) < 1e-10);
assert.equal(moving.closure.converged, false);
assert.ok(
    ['displacement', 'angularDisplacement'].includes(
        moving.closure.dominantResidual
    )
);

const relaxedMotionClosure = kirchhoffClosureState(moving, {
    displacementTolerance: 1,
    angularDisplacementTolerance: 1,
    adaptationTolerance: 0.002,
    bendTolerance: 0.01,
    twistTolerance: 0.01
});
assert.equal(relaxedMotionClosure.converged, true);

console.log('Kirchhoff rod diagnostics tests passed');

