const EPSILON = 1e-8;

function clamp(value, min, max) {
    return Math.max(min, Math.min(max, value));
}

function nodeRadius(body, index) {
    const profiledRadius = body.nodeRadius?.[index];
    return profiledRadius > 0 ? profiledRadius : body.radius ?? 0;
}

function squaredDistanceToSegment(x, y, z, body, segment) {
    const ax = body.x[segment];
    const ay = body.y[segment];
    const az = body.z[segment];
    const dx = body.x[segment + 1] - ax;
    const dy = body.y[segment + 1] - ay;
    const dz = body.z[segment + 1] - az;
    const lengthSquared = dx * dx + dy * dy + dz * dz;
    const t = clamp(
        ((x - ax) * dx + (y - ay) * dy + (z - az) * dz) /
            Math.max(EPSILON, lengthSquared),
        0,
        1
    );
    const rx = x - (ax + dx * t);
    const ry = y - (ay + dy * t);
    const rz = z - (az + dz * t);
    return rx * rx + ry * ry + rz * rz;
}

/**
 * The lumen projection may damp the material span it actually owns, but a
 * broad external-contact search window is only a collision candidate set. It
 * must never extend that numerical damping beyond the distal portal.
 */
export function firstFreeGuidewireNodeAfterContainment({
    activeStart,
    activeEnd,
    containmentEndNode
}) {
    const start = Math.max(0, Math.floor(activeStart));
    const end = Math.max(start, Math.floor(activeEnd));
    return clamp(
        Math.floor(containmentEndNode) + 1,
        start,
        end + 1
    );
}

/**
 * Returns the last guidewire node that can safely be treated as fully inside
 * the catheter. The material coordinate identifies the candidate node, but a
 * moving catheter tip must physically reach that node before containment owns
 * it. Until then the preceding node remains the lumen endpoint and the
 * candidate belongs to the distal portal crossing segment.
 */
export function spatiallyCapturedContainmentEnd({
    innerBody,
    outerBody,
    firstContainedNode,
    materialEndNode,
    outerStartNode = outerBody.activeStart,
    outerInnerRadius = outerBody.innerRadius,
    closestSegment = null,
    captureTolerance = 0.05,
    axialTolerance = 0.25
}) {
    const first = clamp(
        Math.floor(firstContainedNode),
        innerBody.activeStart,
        innerBody.activeEnd
    );
    const materialEnd = clamp(
        Math.floor(materialEndNode),
        first,
        innerBody.activeEnd
    );
    const outerStart = clamp(
        Math.floor(outerStartNode),
        outerBody.activeStart,
        outerBody.activeEnd
    );
    const outerTip = outerBody.activeEnd;
    if (outerTip <= outerStart) return first - 1;

    // Average the final two catheter segments for a stable distal plane. A
    // one-segment tangent flickers as a preformed tip straightens during feed.
    const tangentStart = Math.max(outerStart, outerTip - 2);
    let tangentX = outerBody.x[outerTip] - outerBody.x[tangentStart];
    let tangentY = outerBody.y[outerTip] - outerBody.y[tangentStart];
    let tangentZ = outerBody.z[outerTip] - outerBody.z[tangentStart];
    const tangentLength = Math.hypot(tangentX, tangentY, tangentZ);
    if (tangentLength <= EPSILON) return first - 1;
    tangentX /= tangentLength;
    tangentY /= tangentLength;
    tangentZ /= tangentLength;

    let lastCaptured = first - 1;
    for (let candidate = first; candidate <= materialEnd; candidate++) {
        const x = innerBody.x[candidate];
        const y = innerBody.y[candidate];
        const z = innerBody.z[candidate];
        if (!Number.isFinite(x) || !Number.isFinite(y) || !Number.isFinite(z)) {
            break;
        }
        if (candidate === materialEnd && materialEnd < innerBody.activeEnd) {
            const relativeX = x - outerBody.x[outerTip];
            const relativeY = y - outerBody.y[outerTip];
            const relativeZ = z - outerBody.z[outerTip];
            const axialDistance =
                relativeX * tangentX + relativeY * tangentY + relativeZ * tangentZ;
            if (axialDistance > Math.max(0, axialTolerance)) break;
        }

        const captureRadius = Math.max(
            0,
            outerInnerRadius - nodeRadius(innerBody, candidate) + captureTolerance
        );
        const mappedSegment = closestSegment?.[candidate] ?? -1;
        let closestSquared = Infinity;
        if (mappedSegment >= outerStart && mappedSegment < outerTip) {
            closestSquared = squaredDistanceToSegment(
                x,
                y,
                z,
                outerBody,
                mappedSegment
            );
        } else {
            for (let segment = outerStart; segment < outerTip; segment++) {
                closestSquared = Math.min(
                    closestSquared,
                    squaredDistanceToSegment(x, y, z, outerBody, segment)
                );
            }
        }
        if (closestSquared > captureRadius * captureRadius) break;
        lastCaptured = candidate;
    }
    return lastCaptured;
}

function pushPoint(out, x, y, z) {
    const previous = out[out.length - 1];
    if (
        previous &&
        Math.abs(previous.x - x) +
            Math.abs(previous.y - y) +
            Math.abs(previous.z - z) <= 1e-6
    ) return;
    out.push({ x, y, z });
}

/**
 * Builds a render centerline that follows the catheter polyline throughout
 * the contained span. XPBD can keep sparse guidewire nodes inside the lumen,
 * while a straight 5 mm chord between those nodes still visibly cuts through
 * a tightly curved 5 Fr catheter. Intermediate catheter vertices remove that
 * representation artefact without adding physical masses or solver feedback.
 */
export function buildContainedGuidewireRenderPolyline({
    guidewireNodes,
    outerBody,
    containment,
    out = []
}) {
    out.length = 0;
    out.containedStartIndex = -1;
    out.containedEndIndex = -1;
    if (!guidewireNodes?.length) return out;
    const enabled = containment?.enabled === true;
    const innerStart = enabled
        ? clamp(containment.startNode, 0, guidewireNodes.length - 1)
        : guidewireNodes.length;
    const innerEnd = enabled
        ? clamp(
            containment.renderEndNode ?? containment.endNode,
            innerStart,
            guidewireNodes.length - 1
        )
        : innerStart - 1;
    const materialInnerEnd = enabled
        ? clamp(
            containment.endNode,
            innerStart,
            guidewireNodes.length - 1
        )
        : innerStart - 1;
    for (let index = 0; index < innerStart; index++) {
        const source = guidewireNodes[index];
        pushPoint(out, source.x, source.y, source.z);
    }
    if (!enabled || innerEnd < innerStart) {
        for (let index = innerStart; index < guidewireNodes.length; index++) {
            const source = guidewireNodes[index];
            pushPoint(out, source.x, source.y, source.z);
        }
        return out;
    }

    const startSegment = containment.closestSegment[innerStart];
    const endSegment = containment.closestSegment[innerEnd];
    const mappingsValid =
        startSegment >= outerBody.activeStart &&
        startSegment < outerBody.activeEnd &&
        endSegment >= startSegment &&
        endSegment < outerBody.activeEnd;
    if (!mappingsValid) {
        for (let index = innerStart; index < guidewireNodes.length; index++) {
            const source = guidewireNodes[index];
            pushPoint(out, source.x, source.y, source.z);
        }
        return out;
    }

    const pushMappedPoint = (segment, t) => {
        t = clamp(t, 0, 1);
        pushPoint(
            out,
            outerBody.x[segment] +
                (outerBody.x[segment + 1] - outerBody.x[segment]) * t,
            outerBody.y[segment] +
                (outerBody.y[segment + 1] - outerBody.y[segment]) * t,
            outerBody.z[segment] +
                (outerBody.z[segment + 1] - outerBody.z[segment]) * t
        );
    };

    // Rendering a coarse preformed catheter and its contained wire with one
    // smooth spline can make the wire spline cut outside between two valid
    // physical samples. Add interpolation samples on the catheter's actual
    // piecewise-linear centerline. They are render-only and never feed back
    // into either rod or prescribe a physical path.
    const pushMappedSpan = (segment, startT, endT) => {
        const clampedStart = clamp(startT, 0, 1);
        const clampedEnd = clamp(endT, clampedStart, 1);
        const dx = outerBody.x[segment + 1] - outerBody.x[segment];
        const dy = outerBody.y[segment + 1] - outerBody.y[segment];
        const dz = outerBody.z[segment + 1] - outerBody.z[segment];
        const spanLength = Math.hypot(dx, dy, dz) *
            (clampedEnd - clampedStart);
        const subdivisions = Math.max(1, Math.ceil(spanLength / 0.5));
        for (let step = 0; step <= subdivisions; step++) {
            pushMappedPoint(
                segment,
                clampedStart + (clampedEnd - clampedStart) *
                    step / subdivisions
            );
        }
    };

    out.containedStartIndex = out.length;
    for (let segment = startSegment; segment <= endSegment; segment++) {
        pushMappedSpan(
            segment,
            segment === startSegment
                ? containment.closestT[innerStart]
                : 0,
            segment === endSegment
                ? containment.closestT[innerEnd]
                : 1
        );
    }

    // renderEndNode may deliberately trail the material boundary by one node
    // while the moving catheter captures that node spatially. It is therefore
    // not evidence that the guidewire continues beyond the distal opening.
    // Only the material containment boundary can identify a real distal span.
    const hasExternalSpan = materialInnerEnd < guidewireNodes.length - 1;
    if (hasExternalSpan) {
        for (let segment = endSegment; segment < outerBody.activeEnd; segment++) {
            pushMappedSpan(
                segment,
                segment === endSegment
                    ? containment.closestT[innerEnd]
                    : 0,
                1
            );
        }
        out.containedEndIndex = out.length - 1;

        // A short axial point beyond the opening fixes the Catmull-Rom tangent
        // without constraining the free physical guidewire. The next real wire
        // node can then bend outside the catheter without pulling the rendered
        // curve through its side wall immediately before the opening.
        const outerTip = outerBody.activeEnd;
        const tangentStart = Math.max(outerBody.activeStart, outerTip - 2);
        let tangentX = outerBody.x[outerTip] - outerBody.x[tangentStart];
        let tangentY = outerBody.y[outerTip] - outerBody.y[tangentStart];
        let tangentZ = outerBody.z[outerTip] - outerBody.z[tangentStart];
        const tangentLength = Math.hypot(tangentX, tangentY, tangentZ);
        if (tangentLength > EPSILON) {
            const guideLength = Math.min(0.1, tangentLength * 0.05);
            tangentX /= tangentLength;
            tangentY /= tangentLength;
            tangentZ /= tangentLength;
            pushPoint(
                out,
                outerBody.x[outerTip] + tangentX * guideLength,
                outerBody.y[outerTip] + tangentY * guideLength,
                outerBody.z[outerTip] + tangentZ * guideLength
            );
        }
    } else {
        out.containedEndIndex = out.length - 1;
    }

    for (let index = innerEnd + 1; index < guidewireNodes.length; index++) {
        const source = guidewireNodes[index];
        pushPoint(out, source.x, source.y, source.z);
    }
    return out;
}
