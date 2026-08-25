import { materialSegmentContactId } from './kirchhoffContactManifold.js';

const EPSILON = 1e-12;
const PORTAL_TOLERANCE = 1e-9;

export const DEFAULT_LUMEN_QUADRATURE = Object.freeze([
    0,
    0.25,
    0.5,
    0.75,
    1
]);

function clamp(value, min, max) {
    return Math.max(min, Math.min(max, value));
}

function finiteNumber(value, label) {
    if (!Number.isFinite(value)) {
        throw new TypeError(`${label} must be a finite number`);
    }
    return value;
}

function nonNegative(value, label) {
    finiteNumber(value, label);
    if (value < 0) throw new RangeError(`${label} must be non-negative`);
    return value;
}

function component(vector, index, key, label) {
    const value = Array.isArray(vector) || ArrayBuffer.isView(vector)
        ? vector[index]
        : vector?.[key];
    return finiteNumber(value, `${label}.${key}`);
}

function vector3(value, label) {
    if (value == null) throw new TypeError(`${label} is required`);
    return [
        component(value, 0, 'x', label),
        component(value, 1, 'y', label),
        component(value, 2, 'z', label)
    ];
}

function addScaled(a, b, t) {
    return [
        a[0] + (b[0] - a[0]) * t,
        a[1] + (b[1] - a[1]) * t,
        a[2] + (b[2] - a[2]) * t
    ];
}

function subtract(a, b) {
    return [a[0] - b[0], a[1] - b[1], a[2] - b[2]];
}

function dot(a, b) {
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

function length(vector) {
    return Math.hypot(vector[0], vector[1], vector[2]);
}

function normalize(vector, label) {
    const magnitude = length(vector);
    if (magnitude <= EPSILON) throw new RangeError(`${label} must have positive length`);
    return [
        vector[0] / magnitude,
        vector[1] / magnitude,
        vector[2] / magnitude
    ];
}

function perpendicularTo(axis) {
    const reference = Math.abs(axis[0]) <= Math.abs(axis[1]) &&
        Math.abs(axis[0]) <= Math.abs(axis[2])
        ? [1, 0, 0]
        : Math.abs(axis[1]) <= Math.abs(axis[2])
            ? [0, 1, 0]
            : [0, 0, 1];
    const axial = dot(reference, axis);
    return normalize([
        reference[0] - axis[0] * axial,
        reference[1] - axis[1] * axial,
        reference[2] - axis[2] * axial
    ], 'generated radial normal');
}

function scaled(vector, scale) {
    return [vector[0] * scale, vector[1] * scale, vector[2] * scale];
}

function pointToSegment(point, start, end) {
    const direction = subtract(end, start);
    const lengthSquared = dot(direction, direction);
    if (lengthSquared <= EPSILON) {
        return {
            rawT: 0,
            t: 0,
            point: [...start],
            offset: subtract(point, start)
        };
    }
    const rawT = dot(subtract(point, start), direction) / lengthSquared;
    const t = clamp(rawT, 0, 1);
    const closestPoint = addScaled(start, end, t);
    return {
        rawT,
        t,
        point: closestPoint,
        offset: subtract(point, closestPoint)
    };
}

/**
 * Exact closest points of two finite line segments. This is useful for broad
 * contact diagnostics; lumen containment itself additionally samples the
 * inner segment because an inside constraint is governed by maximum radial
 * distance rather than minimum centerline distance.
 */
export function closestSegmentSegment(
    firstStartValue,
    firstEndValue,
    secondStartValue,
    secondEndValue
) {
    const firstStart = vector3(firstStartValue, 'firstStart');
    const firstEnd = vector3(firstEndValue, 'firstEnd');
    const secondStart = vector3(secondStartValue, 'secondStart');
    const secondEnd = vector3(secondEndValue, 'secondEnd');
    const firstDirection = subtract(firstEnd, firstStart);
    const secondDirection = subtract(secondEnd, secondStart);
    const offset = subtract(firstStart, secondStart);
    const firstLengthSquared = dot(firstDirection, firstDirection);
    const secondLengthSquared = dot(secondDirection, secondDirection);
    const secondProjection = dot(secondDirection, offset);
    let firstT;
    let secondT;

    if (firstLengthSquared <= EPSILON && secondLengthSquared <= EPSILON) {
        firstT = 0;
        secondT = 0;
    } else if (firstLengthSquared <= EPSILON) {
        firstT = 0;
        secondT = clamp(secondProjection / secondLengthSquared, 0, 1);
    } else {
        const firstProjection = dot(firstDirection, offset);
        if (secondLengthSquared <= EPSILON) {
            secondT = 0;
            firstT = clamp(-firstProjection / firstLengthSquared, 0, 1);
        } else {
            const directionsDot = dot(firstDirection, secondDirection);
            const denominator = firstLengthSquared * secondLengthSquared -
                directionsDot * directionsDot;
            firstT = denominator > EPSILON
                ? clamp(
                    (directionsDot * secondProjection -
                        firstProjection * secondLengthSquared) / denominator,
                    0,
                    1
                )
                : 0;
            secondT = (
                directionsDot * firstT + secondProjection
            ) / secondLengthSquared;
            if (secondT < 0) {
                secondT = 0;
                firstT = clamp(-firstProjection / firstLengthSquared, 0, 1);
            } else if (secondT > 1) {
                secondT = 1;
                firstT = clamp(
                    (directionsDot - firstProjection) / firstLengthSquared,
                    0,
                    1
                );
            }
        }
    }

    const firstPoint = addScaled(firstStart, firstEnd, firstT);
    const secondPoint = addScaled(secondStart, secondEnd, secondT);
    return {
        firstT,
        secondT,
        firstWeights: [1 - firstT, firstT],
        secondWeights: [1 - secondT, secondT],
        firstPoint,
        secondPoint,
        distance: length(subtract(firstPoint, secondPoint))
    };
}

function contactGradients(normal, innerWeights, outerWeights) {
    return {
        inner: [
            scaled(normal, -innerWeights[0]),
            scaled(normal, -innerWeights[1])
        ],
        outer: [
            scaled(normal, outerWeights[0]),
            scaled(normal, outerWeights[1])
        ]
    };
}

function contactRecord({
    kind,
    feature,
    innerMaterialSegmentId,
    outerMaterialSegmentId,
    innerT,
    outerT,
    radialDistance,
    clearance,
    normal,
    tangentU,
    manifold,
    innerSegmentIndex,
    outerSegmentIndex,
    activationDistance,
    effectiveTwistRadius,
    gapOverride = null
}) {
    const gap = Number.isFinite(gapOverride)
        ? gapOverride
        : clearance - radialDistance;
    const innerWeights = [1 - innerT, innerT];
    const outerWeights = [1 - outerT, outerT];
    const id = materialSegmentContactId(
        innerMaterialSegmentId,
        outerMaterialSegmentId,
        feature
    );
    const record = {
        id,
        kind,
        feature,
        gap,
        violation: Math.max(0, -gap),
        active: gap < 0,
        clearance,
        radialDistance,
        normal,
        innerT,
        outerT,
        innerWeights,
        outerWeights,
        gradients: contactGradients(normal, innerWeights, outerWeights),
        manifoldContact: null
    };
    if (manifold && gap <= activationDistance) {
        record.manifoldContact = manifold.upsertContact({
            innerMaterialSegmentId,
            outerMaterialSegmentId,
            feature,
            innerSegmentIndex,
            outerSegmentIndex,
            normal,
            tangentU,
            effectiveTwistRadius
        });
    }
    return record;
}

function validateQuadrature(quadrature) {
    if (!Array.isArray(quadrature) && !ArrayBuffer.isView(quadrature)) {
        throw new TypeError('quadrature must be an array of inner-segment coordinates');
    }
    if (quadrature.length === 0) throw new RangeError('quadrature cannot be empty');
    return Array.from(quadrature, (sample, index) => {
        finiteNumber(sample, `quadrature[${index}]`);
        if (sample < 0 || sample > 1) {
            throw new RangeError(`quadrature[${index}] must lie in [0, 1]`);
        }
        return sample;
    });
}

/**
 * Evaluates one inner Kirchhoff segment against one straight segment of a
 * swept circular catheter lumen. The returned normal points radially outward;
 * for the feasible inequality g = clearance - radius >= 0, inner gradients
 * point inward and outer gradients are equal and opposite.
 *
 * An open distal end has no end-cap or angular/direction constraint. A segment
 * may cross its plane freely when the crossing lies inside the aperture. A
 * crossing outside that aperture creates only a radial rim constraint.
 */
export function evaluateKirchhoffLumenSegmentContact({
    innerStart: innerStartValue,
    innerEnd: innerEndValue,
    outerStart: outerStartValue,
    outerEnd: outerEndValue,
    lumenRadius,
    innerRadius,
    innerMaterialSegmentId,
    outerMaterialSegmentId,
    innerSegmentIndex = -1,
    outerSegmentIndex = -1,
    openDistal = false,
    portalFilletRadius = 0,
    quadrature = DEFAULT_LUMEN_QUADRATURE,
    activationDistance = 0,
    featurePrefix = 'lumen',
    manifold = null
}) {
    const innerStart = vector3(innerStartValue, 'innerStart');
    const innerEnd = vector3(innerEndValue, 'innerEnd');
    const outerStart = vector3(outerStartValue, 'outerStart');
    const outerEnd = vector3(outerEndValue, 'outerEnd');
    const lumen = nonNegative(lumenRadius, 'lumenRadius');
    const inner = nonNegative(innerRadius, 'innerRadius');
    const clearance = Math.max(0, lumen - inner);
    const filletRadius = nonNegative(
        portalFilletRadius,
        'portalFilletRadius'
    );
    nonNegative(activationDistance, 'activationDistance');
    if (typeof featurePrefix !== 'string' || featurePrefix.length === 0) {
        throw new TypeError('featurePrefix must be a non-empty string');
    }

    const outerVector = subtract(outerEnd, outerStart);
    const outerLength = length(outerVector);
    if (outerLength <= EPSILON) {
        throw new RangeError('outer lumen segment must have positive length');
    }
    const axis = scaled(outerVector, 1 / outerLength);
    const sampleCoordinates = validateQuadrature(quadrature);
    const samples = [];
    let worstSample = null;

    for (const innerT of sampleCoordinates) {
        const point = addScaled(innerStart, innerEnd, innerT);
        const closest = pointToSegment(point, outerStart, outerEnd);
        // Adjacent swept segments own points outside this segment's material
        // interval. In particular, an open distal lumen has no spherical cap.
        if (
            closest.rawT < -PORTAL_TOLERANCE ||
            closest.rawT > 1 + PORTAL_TOLERANCE
        ) continue;
        if (openDistal) {
            const distalAxial = dot(subtract(point, outerEnd), axis);
            if (distalAxial > PORTAL_TOLERANCE) continue;
            // The exact aperture/rim solve below owns the distal plane. Letting
            // the final side-wall quadrature point own it as well would count
            // the same normal reaction twice at an invalid crossing.
            if (
                distalAxial >=
                    -Math.max(PORTAL_TOLERANCE, filletRadius)
            ) continue;
        }
        const radialDistance = length(closest.offset);
        const normal = radialDistance > EPSILON
            ? scaled(closest.offset, 1 / radialDistance)
            : perpendicularTo(axis);
        const sample = {
            innerT,
            outerT: closest.t,
            point,
            centerlinePoint: closest.point,
            radialDistance,
            gap: clearance - radialDistance,
            normal,
            innerWeights: [1 - innerT, innerT],
            outerWeights: [1 - closest.t, closest.t]
        };
        samples.push(sample);
        if (!worstSample || sample.gap < worstSample.gap - EPSILON) {
            worstSample = sample;
        }
    }

    const side = worstSample
        ? contactRecord({
            kind: 'side',
            feature: `${featurePrefix}:side`,
            innerMaterialSegmentId,
            outerMaterialSegmentId,
            innerT: worstSample.innerT,
            outerT: worstSample.outerT,
            radialDistance: worstSample.radialDistance,
            clearance,
            normal: worstSample.normal,
            tangentU: axis,
            manifold,
            innerSegmentIndex,
            outerSegmentIndex,
            activationDistance,
            effectiveTwistRadius: worstSample.radialDistance
        })
        : null;

    let fillet = null;
    if (openDistal && filletRadius > EPSILON) {
        const majorRadius = clearance + filletRadius;
        let worstFillet = null;
        for (const innerT of sampleCoordinates) {
            const point = addScaled(innerStart, innerEnd, innerT);
            const offset = subtract(point, outerEnd);
            const axial = dot(offset, axis);
            if (axial < -filletRadius || axial > filletRadius) continue;
            const radial = [
                offset[0] - axis[0] * axial,
                offset[1] - axis[1] * axial,
                offset[2] - axis[2] * axial
            ];
            const radialDistance = length(radial);
            if (radialDistance > majorRadius) continue;
            const radialUnit = radialDistance > EPSILON
                ? scaled(radial, 1 / radialDistance)
                : perpendicularTo(axis);
            const circleAxial = axial + filletRadius;
            const circleRadial = radialDistance - majorRadius;
            const circleDistance = Math.hypot(circleAxial, circleRadial);
            if (circleDistance <= EPSILON) continue;
            const gap = circleDistance - filletRadius;
            if (worstFillet && gap >= worstFillet.gap) continue;
            const gradientAxial = circleAxial / circleDistance;
            const gradientRadial = circleRadial / circleDistance;
            worstFillet = {
                innerT,
                radialDistance,
                gap,
                normal: [
                    -(axis[0] * gradientAxial +
                        radialUnit[0] * gradientRadial),
                    -(axis[1] * gradientAxial +
                        radialUnit[1] * gradientRadial),
                    -(axis[2] * gradientAxial +
                        radialUnit[2] * gradientRadial)
                ]
            };
        }
        fillet = worstFillet
            ? contactRecord({
                kind: 'distal-fillet',
                feature: `${featurePrefix}:distal-fillet`,
                innerMaterialSegmentId,
                outerMaterialSegmentId,
                innerT: worstFillet.innerT,
                outerT: 1,
                radialDistance: worstFillet.radialDistance,
                clearance: filletRadius,
                normal: worstFillet.normal,
                tangentU: axis,
                manifold,
                innerSegmentIndex,
                outerSegmentIndex,
                activationDistance,
                effectiveTwistRadius: worstFillet.radialDistance,
                gapOverride: worstFillet.gap
            })
            : null;
    }

    const innerStartFromPortal = subtract(innerStart, outerEnd);
    const innerEndFromPortal = subtract(innerEnd, outerEnd);
    const startAxial = dot(innerStartFromPortal, axis);
    const endAxial = dot(innerEndFromPortal, axis);
    const axialDelta = endAxial - startAxial;
    const crossesPortal = openDistal &&
        Math.abs(axialDelta) > EPSILON &&
        Math.min(startAxial, endAxial) <= PORTAL_TOLERANCE &&
        Math.max(startAxial, endAxial) >= -PORTAL_TOLERANCE;
    let portal = {
        open: openDistal,
        crosses: false,
        valid: false,
        innerT: null,
        radialDistance: null,
        gap: null,
        contact: null
    };

    if (crossesPortal) {
        const innerT = clamp(-startAxial / axialDelta, 0, 1);
        const crossing = addScaled(innerStart, innerEnd, innerT);
        const portalOffset = subtract(crossing, outerEnd);
        const residualAxial = dot(portalOffset, axis);
        const radial = [
            portalOffset[0] - axis[0] * residualAxial,
            portalOffset[1] - axis[1] * residualAxial,
            portalOffset[2] - axis[2] * residualAxial
        ];
        const radialDistance = length(radial);
        const normal = radialDistance > EPSILON
            ? scaled(radial, 1 / radialDistance)
            : perpendicularTo(axis);
        const portalClearance = clearance + filletRadius;
        const gap = portalClearance - radialDistance;
        const rimContact = gap <= activationDistance
            ? contactRecord({
                kind: 'distal-rim',
                feature: `${featurePrefix}:distal-rim`,
                innerMaterialSegmentId,
                outerMaterialSegmentId,
                innerT,
                outerT: 1,
                radialDistance,
                clearance: portalClearance,
                normal,
                tangentU: axis,
                manifold,
                innerSegmentIndex,
                outerSegmentIndex,
                activationDistance,
                effectiveTwistRadius: radialDistance
            })
            : null;
        portal = {
            open: true,
            crosses: true,
            valid: gap >= 0,
            innerT,
            crossing,
            radialDistance,
            gap,
            contact: rimContact
        };
    }

    const activeContacts = [];
    if (side?.active) activeContacts.push(side);
    if (fillet?.active) activeContacts.push(fillet);
    if (portal.contact?.active) activeContacts.push(portal.contact);
    return {
        clearance,
        axis,
        closest: closestSegmentSegment(
            innerStart,
            innerEnd,
            outerStart,
            outerEnd
        ),
        samples,
        side,
        fillet,
        portal,
        activeContacts
    };
}
