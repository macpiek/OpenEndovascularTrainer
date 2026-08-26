import * as THREE from 'three';
import {
    createContactResult
} from '../physics/collision/vesselContactField.js';

const FLOW_RADIAL_SEGMENTS = 24;
const FLOW_LUMEN_RADIUS_FACTOR = 1.0;
const FLOW_DETECTION_FLOOR = 0.0005;
const FLOW_DETECTION_FULL = 0.02;
const COLUMN_BRANCH_FADE_DELAY_SECONDS = 0.18;
const COLUMN_BRANCH_FADE_END_SECONDS = 0.65;
const FLOW_TOPOLOGY_LOCAL_WEIGHT = 0.55;
const FLOW_TOPOLOGY_NEIGHBOUR_WEIGHT =
    (1 - FLOW_TOPOLOGY_LOCAL_WEIGHT) * 0.5;
const FLOW_INTERNAL_GAP_FILL_FACTOR = 0.72;
const FLOW_TOPOLOGY_RADIUS_BLEND_MM = 12;
const FLOW_JUNCTION_UNION_LENGTH_FACTOR = 1.35;
const FLOW_MIN_JUNCTION_RADIUS_MM = 0.75;
const FLOW_SIDE_BRANCH_FLOW_FRACTION_START = 0.54;
const FLOW_SIDE_BRANCH_FLOW_FRACTION_FULL = 0.61;
const SIDE_OSTIUM_CLIP_FADE_OUTSIDE_MM = 0.35;
const SIDE_OSTIUM_CLIP_FULL_COVERAGE_OUTSIDE_MM = 0.05;

const flowLumenVertexShader = `
    attribute float flowConcentration;
    attribute float flowRadius;
    attribute float flowOpticalWeight;
    varying float vConcentration;
    varying float vRadius;
    varying float vOpticalWeight;
    varying vec3 vViewNormal;
    varying vec3 vViewRay;

    void main() {
        vec4 viewPosition = modelViewMatrix * vec4(position, 1.0);
        vViewNormal = normalize(normalMatrix * normal);
        vViewRay = normalize(-viewPosition.xyz);
        vConcentration = flowConcentration;
        vRadius = flowRadius;
        vOpticalWeight = flowOpticalWeight;
        gl_Position = projectionMatrix * viewPosition;
    }
`;

const flowLumenFragmentShader = `
    uniform bool debugMode;
    uniform bool connectorMode;
    uniform bool maximumBlend;
    uniform vec3 debugColor;
    uniform float signalGain;
    varying float vConcentration;
    varying float vRadius;
    varying float vOpticalWeight;
    varying vec3 vViewNormal;
    varying vec3 vViewRay;

    void main() {
        // Approximate the X-ray chord through a filled lumen from its front
        // surface. A broad plateau prevents the centre from looking like a
        // separate narrow jet while the final smoothstep keeps a soft edge.
        float chordFactor = abs(dot(
            normalize(vViewNormal),
            normalize(vViewRay)
        ));
        float lumenCoverage = connectorMode
            ? mix(
                0.28,
                1.0,
                smoothstep(0.04, 0.45, chordFactor)
            )
            : smoothstep(0.015, 0.18, chordFactor);
        float filledLumenProfile = connectorMode
            ? mix(
                0.78,
                1.0,
                smoothstep(0.1, 0.82, chordFactor)
            )
            : mix(
                0.72,
                1.0,
                smoothstep(0.08, 0.82, chordFactor)
            );
        float opticalDepth =
            max(0.0, vConcentration) *
            max(0.16, vRadius) *
            vOpticalWeight *
            filledLumenProfile *
            lumenCoverage *
            signalGain;
        if (opticalDepth < 0.0001) discard;

        // WebGL ignores blend factors for MIN/MAX equations. Maximum-union
        // surfaces must therefore emit the same premultiplied signal that an
        // ordinary additive surface obtains from source-alpha blending.
        float transferDepth = maximumBlend
            ? opticalDepth * opticalDepth
            : opticalDepth;
        vec3 signal = debugMode
            ? debugColor * transferDepth
            : vec3(transferDepth);
        gl_FragColor = vec4(signal, transferDepth);
    }
`;

const plumeVertexShader = `
    attribute vec3 instanceCenter;
    attribute vec3 instanceVelocity;
    attribute float radialRadius;
    attribute float axialHalfLength;
    attribute float plumeConcentration;
    attribute float opticalScale;
    varying vec2 vPlumeCoordinate;
    varying float vConcentration;
    varying float vOpticalScale;

    void main() {
        vec4 viewCenter = modelViewMatrix * vec4(instanceCenter, 1.0);
        vec3 viewVelocity = mat3(modelViewMatrix) * instanceVelocity;
        float velocityLength = length(viewVelocity);
        float projectedLength = length(viewVelocity.xy);
        vec2 axialDirection = projectedLength > 0.00001
            ? viewVelocity.xy / projectedLength
            : vec2(0.0, 1.0);
        vec2 radialDirection = vec2(-axialDirection.y, axialDirection.x);
        float foreshortening = clamp(
            projectedLength / max(velocityLength, 0.00001),
            0.28,
            1.0
        );
        viewCenter.xy +=
            radialDirection * position.x * radialRadius +
            axialDirection * position.y * axialHalfLength * foreshortening;

        gl_Position = projectionMatrix * viewCenter;
        vPlumeCoordinate = position.xy;
        vConcentration = plumeConcentration;
        vOpticalScale = opticalScale;
    }
`;

const plumeFragmentShader = `
    uniform bool debugMode;
    uniform vec3 debugColor;
    uniform float signalGain;
    varying vec2 vPlumeCoordinate;
    varying float vConcentration;
    varying float vOpticalScale;

    void main() {
        float radiusSquared =
            vPlumeCoordinate.x * vPlumeCoordinate.x * 2.15 +
            vPlumeCoordinate.y * vPlumeCoordinate.y * 1.05;
        float boundary = max(
            abs(vPlumeCoordinate.x),
            abs(vPlumeCoordinate.y)
        );
        float softEdge = 1.0 - smoothstep(0.66, 1.0, boundary);
        float kernel = exp(-radiusSquared * 1.45) * softEdge;
        float opticalDepth =
            max(0.0, vConcentration) *
            vOpticalScale *
            kernel *
            signalGain;
        if (opticalDepth < 0.0001) discard;

        vec3 signal = debugMode
            ? debugColor * opticalDepth
            : vec3(opticalDepth);
        gl_FragColor = vec4(signal, opticalDepth);
    }
`;

function smoothstep(edge0, edge1, value) {
    const t = THREE.MathUtils.clamp(
        (value - edge0) / Math.max(1e-9, edge1 - edge0),
        0,
        1
    );
    return t * t * (3 - 2 * t);
}

function sideOstiumAnatomicalCoverage(
    profile,
    connector,
    contactField
) {
    if (
        !profile.hasGeometricMainContinuation ||
        !contactField?.querySphere
    ) {
        return {
            clippedVertexCount: 0,
            minimumCoverage: 1,
            maximumOutsideDistanceMm: 0,
            suppressConnectorSurface: false
        };
    }

    const point = new THREE.Vector3();
    const contact = createContactResult();
    let clippedVertexCount = 0;
    let minimumCoverage = 1;
    let maximumOutsideDistanceMm = 0;
    for (
        let vertexIndex = 0;
        vertexIndex < connector.vertexCount;
        vertexIndex++
    ) {
        const offset = vertexIndex * 3;
        point.set(
            connector.positions[offset],
            connector.positions[offset + 1],
            connector.positions[offset + 2]
        );
        const signedDistance = contactField.querySphere(
            point,
            0,
            contact
        ).signedDistance;
        if (!Number.isFinite(signedDistance)) continue;
        const outsideDistanceMm = Math.max(0, -signedDistance);
        const coverage = smoothstep(
            SIDE_OSTIUM_CLIP_FULL_COVERAGE_OUTSIDE_MM,
            SIDE_OSTIUM_CLIP_FADE_OUTSIDE_MM,
            outsideDistanceMm
        );
        minimumCoverage = Math.min(minimumCoverage, 1 - coverage);
        maximumOutsideDistanceMm = Math.max(
            maximumOutsideDistanceMm,
            outsideDistanceMm
        );
        if (
            outsideDistanceMm <=
            SIDE_OSTIUM_CLIP_FULL_COVERAGE_OUTSIDE_MM
        ) continue;
        clippedVertexCount++;
    }
    return {
        clippedVertexCount,
        minimumCoverage,
        maximumOutsideDistanceMm,
        // With an exact anatomical contact field, the intersecting tube
        // shoulders already describe the visible ostium and share one MAX
        // optical union. Rendering the coarse synthetic saddle as well can
        // expose marching-tetrahedra facets beyond that lumen silhouette.
        suppressConnectorSurface: true
    };
}

function blendContinuousFlowConcentration(
    localConcentration,
    upstreamConcentration,
    downstreamConcentration
) {
    if (!(localConcentration > FLOW_DETECTION_FLOOR)) {
        return (
            upstreamConcentration > FLOW_DETECTION_FLOOR &&
            downstreamConcentration > FLOW_DETECTION_FLOOR
        )
            ? Math.min(
                upstreamConcentration,
                downstreamConcentration
            ) * FLOW_INTERNAL_GAP_FILL_FACTOR
            : localConcentration;
    }

    const upstreamVisible =
        upstreamConcentration > FLOW_DETECTION_FLOOR;
    const downstreamVisible =
        downstreamConcentration > FLOW_DETECTION_FLOOR;
    if (!upstreamVisible && !downstreamVisible) {
        return 0;
    }
    if (upstreamVisible !== downstreamVisible) {
        const neighbourConcentration = upstreamVisible
            ? upstreamConcentration
            : downstreamConcentration;
        // Preserve the advancing or receding end of a real column. The old
        // support-ratio attenuation repeatedly eroded these end cells at
        // every frame and turned one connected bolus into separated dark
        // islands, especially where a chain crossed a bifurcation.
        return localConcentration * 0.78 +
            neighbourConcentration * 0.22;
    }

    const blended =
        localConcentration * FLOW_TOPOLOGY_LOCAL_WEIGHT +
        upstreamConcentration * FLOW_TOPOLOGY_NEIGHBOUR_WEIGHT +
        downstreamConcentration * FLOW_TOPOLOGY_NEIGHBOUR_WEIGHT;
    const maximumNeighbourConcentration = Math.max(
        upstreamConcentration,
        downstreamConcentration
    );
    return Math.min(
        blended,
        maximumNeighbourConcentration * 1.2
    );
}

function createFlowMaterial({
    connectorMode = false,
    maximumBlend = false
} = {}) {
    const usesMaximumBlend = connectorMode || maximumBlend;
    const material = new THREE.ShaderMaterial({
        uniforms: {
            debugMode: { value: false },
            connectorMode: { value: connectorMode },
            maximumBlend: { value: usesMaximumBlend },
            debugColor: { value: new THREE.Color(0x14b8ff) },
            signalGain: { value: 0.14 }
        },
        vertexShader: flowLumenVertexShader,
        fragmentShader: flowLumenFragmentShader,
        side: THREE.FrontSide,
        transparent: true,
        blending: usesMaximumBlend
            ? THREE.CustomBlending
            : THREE.AdditiveBlending,
        depthTest: false,
        depthWrite: false,
        toneMapped: false
    });
    if (usesMaximumBlend) {
        material.blendEquation = THREE.MaxEquation;
        // MIN/MAX equations ignore these factors in WebGL. The shader emits
        // premultiplied optical signal for this material; explicit One/One
        // documents the actual operation and avoids backend-dependent state.
        material.blendSrc = THREE.OneFactor;
        material.blendDst = THREE.OneFactor;
        material.blendEquationAlpha = THREE.MaxEquation;
        material.blendSrcAlpha = THREE.OneFactor;
        material.blendDstAlpha = THREE.OneFactor;
    }
    return material;
}

function createPlumeMaterial() {
    return new THREE.ShaderMaterial({
        uniforms: {
            debugMode: { value: false },
            debugColor: { value: new THREE.Color(0xff6a24) },
            signalGain: { value: 0.12 }
        },
        vertexShader: plumeVertexShader,
        fragmentShader: plumeFragmentShader,
        transparent: true,
        blending: THREE.AdditiveBlending,
        depthTest: false,
        depthWrite: false,
        toneMapped: false
    });
}

function initialFrameNormal(tangent) {
    const reference = Math.abs(tangent.y) < 0.88
        ? new THREE.Vector3(0, 1, 0)
        : new THREE.Vector3(1, 0, 0);
    return new THREE.Vector3()
        .crossVectors(tangent, reference)
        .normalize();
}

function transportedFrameNormal(previousNormal, tangent) {
    const normal = previousNormal.clone().addScaledVector(
        tangent,
        -previousNormal.dot(tangent)
    );
    if (normal.lengthSq() < 1e-8) return initialFrameNormal(tangent);
    return normal.normalize();
}

function alignedContinuationChild(edges, parent, childIndices) {
    if (childIndices.length === 1) return childIndices[0];
    if (childIndices.length < 2) return -1;
    let bestChildIndex = -1;
    let bestFlow = -1;
    let bestAlignment = -Infinity;
    for (const childIndex of childIndices) {
        const child = edges[childIndex];
        const flow = Math.max(0, child.meanFlowMm3PerS);
        const alignment = THREE.MathUtils.clamp(
            parent.axis.dot(child.axis),
            -1,
            1
        );
        if (
            flow > bestFlow ||
            (flow === bestFlow && alignment > bestAlignment)
        ) {
            bestFlow = flow;
            bestAlignment = alignment;
            bestChildIndex = childIndex;
        }
    }
    const sideBranchDominance = smoothstep(
        FLOW_SIDE_BRANCH_FLOW_FRACTION_START,
        FLOW_SIDE_BRANCH_FLOW_FRACTION_FULL,
        bestFlow /
            Math.max(
                1e-9,
                childIndices.reduce(
                    (sum, childIndex) =>
                        sum + Math.max(
                            0,
                            edges[childIndex].meanFlowMm3PerS
                        ),
                    0
                )
            )
    );
    return sideBranchDominance > 0 ? bestChildIndex : -1;
}

function buildEdgeChains(edges) {
    const isRenderExcluded = edge =>
        !edge || edge.transportExcluded || edge.renderExcluded;
    const chains = [];
    const visited = new Uint8Array(edges.length);
    const chainStarts = [];
    const continuationChild = new Int32Array(edges.length);
    continuationChild.fill(-1);

    for (const edge of edges) {
        if (isRenderExcluded(edge)) continue;
        const transportChildren = edge.childEdgeIndices.filter(
            childIndex => !isRenderExcluded(edges[childIndex])
        );
        // A nearly collinear child is the physical continuation of the parent,
        // not a second coincident tube. Keeping it in one chain prevents a
        // parent-sized overlap ring and leaves only the lateral ostia to join.
        continuationChild[edge.index] = alignedContinuationChild(
            edges,
            edge,
            transportChildren
        );
    }

    for (let edgeIndex = 0; edgeIndex < edges.length; edgeIndex++) {
        const edge = edges[edgeIndex];
        if (isRenderExcluded(edge)) continue;
        const parent = edge.parentEdgeIndex >= 0
            ? edges[edge.parentEdgeIndex]
            : null;
        if (
            !parent || isRenderExcluded(parent) ||
            continuationChild[parent.index] !== edgeIndex
        ) {
            chainStarts.push(edgeIndex);
        }
    }

    const appendChain = startEdgeIndex => {
        if (
            visited[startEdgeIndex] ||
            isRenderExcluded(edges[startEdgeIndex])
        ) return;
        const chain = [];
        let edgeIndex = startEdgeIndex;
        while (edgeIndex >= 0 && !visited[edgeIndex]) {
            visited[edgeIndex] = 1;
            chain.push(edgeIndex);
            edgeIndex = continuationChild[edgeIndex];
        }
        if (chain.length) chains.push(chain);
    };

    for (const startEdgeIndex of chainStarts) appendChain(startEdgeIndex);
    for (let edgeIndex = 0; edgeIndex < edges.length; edgeIndex++) {
        appendChain(edgeIndex);
    }
    return { chains, continuationChild };
}

function bifurcationOpticalAnchor(
    edges,
    parentEdge,
    continuationChild
) {
    if (!parentEdge) return null;
    const children = parentEdge.childEdgeIndices
        .map(childIndex => edges[childIndex])
        .filter(child =>
            child &&
            !child.transportExcluded &&
            !child.renderExcluded
        );
    if (children.length < 2) return null;

    const connectorEnabled =
        parentEdge.radiusEnd >= FLOW_MIN_JUNCTION_RADIUS_MM;
    const continuationEdgeIndex = continuationChild[parentEdge.index];
    if (continuationEdgeIndex >= 0) {
        const continuation = edges[continuationEdgeIndex];
        const sideOstium =
            connectorEnabled &&
            !!continuation;
        const sideChildren = children.filter(
            child => child.index !== continuationEdgeIndex
        );
        const mainRadius = Math.max(0.2, parentEdge.radiusEnd);
        const sideRadii = sideChildren.map(
            child => Math.max(0.2, child.radiusStart)
        );
        const unionRadius = Math.max(mainRadius, ...sideRadii);
        return {
            nodeId: parentEdge.endNodeId,
            parentEdgeIndex: parentEdge.index,
            incidentCount: 1 + sideChildren.length,
            unionRadius,
            opticalFadeLengthMm: connectorEnabled
                ? Math.max(
                    2.5,
                    unionRadius * 3
                )
                : 0,
            opticalWeight: 1,
            sideOstium
        };
    }

    const parentIsRendered =
        !parentEdge.transportExcluded &&
        !parentEdge.renderExcluded;
    const incidentRadii = [
        ...(parentIsRendered
            ? [Math.max(0.2, parentEdge.radiusEnd)]
            : []),
        ...children.map(child => Math.max(0.2, child.radiusStart))
    ];
    const unionRadius = Math.max(...incidentRadii);
    return {
        nodeId: parentEdge.endNodeId,
        parentEdgeIndex: parentEdge.index,
        incidentCount: incidentRadii.length,
        unionRadius,
        opticalFadeLengthMm: connectorEnabled
            ? Math.max(
                0.75,
                unionRadius * 2
            )
            : 0,
        opticalWeight: 1,
        sideOstium: false
    };
}

function junctionUnionProfile(parent, renderedChildren) {
    const incidentRadii = [
        Math.max(
            0.2,
            parent.radiusEnd * FLOW_LUMEN_RADIUS_FACTOR
        ),
        ...renderedChildren.map(child => Math.max(
            0.2,
            child.radiusStart * FLOW_LUMEN_RADIUS_FACTOR
        ))
    ];
    let continuationFlow = -1;
    let continuationAlignment = -Infinity;
    for (const child of renderedChildren) {
        const flow = Math.max(0, child.meanFlowMm3PerS);
        const alignment = THREE.MathUtils.clamp(
            parent.axis.dot(child.axis),
            -1,
            1
        );
        if (
            flow > continuationFlow ||
            (flow === continuationFlow &&
                alignment > continuationAlignment)
        ) {
            continuationFlow = flow;
            continuationAlignment = alignment;
        }
    }
    const totalChildFlow = renderedChildren.reduce(
        (sum, child) => sum + Math.max(0, child.meanFlowMm3PerS),
        0
    );
    const continuationFlowFraction = Math.max(
        0,
        continuationFlow
    ) / Math.max(1e-9, totalChildFlow);
    const sideBranchDominance = smoothstep(
        FLOW_SIDE_BRANCH_FLOW_FRACTION_START,
        FLOW_SIDE_BRANCH_FLOW_FRACTION_FULL,
        continuationFlowFraction
    );
    const minimumIncidentRadius = Math.min(...incidentRadii);
    const maximumIncidentRadius = Math.max(...incidentRadii);
    return {
        incidentRadii,
        minimumIncidentRadius,
        maximumIncidentRadius,
        continuationAlignment,
        continuationFlowFraction,
        sideBranchDominance
    };
}

function junctionSurfaceOverlap(
    profile,
    position,
    currentEdgeIndex
) {
    let maximumOverlap = 0;
    const offset = position.clone().sub(profile.center);
    for (const arm of profile.surfaceArms) {
        if (arm.edgeIndex === currentEdgeIndex) continue;
        const axialDistance = offset.dot(arm.direction);
        const radialOffset = offset.clone().addScaledVector(
            arm.direction,
            -axialDistance
        );
        const radialDistance = radialOffset.length();
        const axialOverlap = smoothstep(
            -arm.radius * 0.06,
            arm.radius * 0.06,
            axialDistance
        );
        const radialOverlap =
            1 - smoothstep(
                arm.radius * 0.92,
                arm.radius,
                radialDistance
            );
        maximumOverlap = Math.max(
            maximumOverlap,
            axialOverlap * radialOverlap
        );
    }
    return maximumOverlap;
}

const JUNCTION_CONNECTOR_GRID_STEPS = 11;
const SIDE_OSTIUM_CONNECTOR_GRID_STEPS = 11;

function cappedCylinderSdf(
    point,
    center,
    arm,
    backwardLengthMm,
    forwardLengthMm
) {
    const offset = point.clone().sub(center);
    const axialDistance = offset.dot(arm.direction);
    const radialDistance = offset.addScaledVector(
        arm.direction,
        -axialDistance
    ).length();
    const effectiveRadius = junctionArmRadiusAtAxialDistance(
        arm,
        axialDistance
    );
    return Math.max(
        radialDistance - effectiveRadius,
        -backwardLengthMm - axialDistance,
        axialDistance - forwardLengthMm
    );
}

function junctionArmRadiusAtAxialDistance(arm, axialDistance) {
    if (!arm.taperFromCenter) return arm.radius;
    const progress = smoothstep(
        0,
        1,
        Math.max(0, axialDistance) /
            Math.max(1e-9, arm.ostiumRampLengthMm)
    );
    return arm.radius * THREE.MathUtils.lerp(
        arm.ostiumStartRadiusFraction,
        1,
        progress
    );
}

function junctionConnectorSdf(profile, point) {
    let distance = Infinity;
    for (const arm of profile.surfaceArms) {
        const armDistance = cappedCylinderSdf(
            point,
            profile.center,
            arm,
            profile.connectorBackwardLengthMm,
            profile.connectorLengthMm
        );
        distance = Math.min(distance, armDistance);
    }
    return distance;
}

function cappedCylinderNormal(profile, point, arm) {
    const epsilon = Math.max(
        0.015,
        profile.minimumSurfaceRadius * 0.025
    );
    const x = new THREE.Vector3(epsilon, 0, 0);
    const y = new THREE.Vector3(0, epsilon, 0);
    const z = new THREE.Vector3(0, 0, epsilon);
    const sample = samplePoint => cappedCylinderSdf(
        samplePoint,
        profile.center,
        arm,
        profile.connectorBackwardLengthMm,
        profile.connectorLengthMm
    );
    return new THREE.Vector3(
        sample(point.clone().add(x)) - sample(point.clone().sub(x)),
        sample(point.clone().add(y)) - sample(point.clone().sub(y)),
        sample(point.clone().add(z)) - sample(point.clone().sub(z))
    ).normalize();
}

function junctionConnectorNormal(profile, point) {
    return cappedCylinderNormal(
        profile,
        point,
        nearestJunctionSurfaceArm(profile, point)
    );
}

function nearestJunctionSurfaceArm(profile, point) {
    let nearest = profile.surfaceArms[0];
    let nearestDistance = Infinity;
    for (const arm of profile.surfaceArms) {
        const distance = cappedCylinderSdf(
            point,
            profile.center,
            arm,
            profile.connectorBackwardLengthMm,
            profile.connectorLengthMm
        );
        if (distance < nearestDistance) {
            nearest = arm;
            nearestDistance = distance;
        }
    }
    return nearest;
}

function junctionConnectorOutwardProgress(profile, point) {
    let outwardProgress = 0;
    const offset = point.clone().sub(profile.center);
    for (const arm of profile.surfaceArms) {
        const axialDistance = offset.dot(arm.direction);
        const radialDistance = offset.clone().addScaledVector(
            arm.direction,
            -axialDistance
        ).length();
        const radialMembership =
            1 - smoothstep(
                arm.radius,
                arm.radius * 1.22,
                radialDistance
            );
        const armProgress = THREE.MathUtils.clamp(
            Math.max(0, axialDistance) /
                Math.max(1e-9, profile.connectorLengthMm),
            0,
            1
        );
        outwardProgress = Math.max(
            outwardProgress,
            armProgress * radialMembership
        );
    }
    return outwardProgress;
}

function interpolateIsoSurfacePoint(
    profile,
    pointA,
    pointB,
    valueA,
    valueB
) {
    let insidePoint = valueA <= 0 ? pointA.clone() : pointB.clone();
    let outsidePoint = valueA <= 0 ? pointB.clone() : pointA.clone();
    for (let iteration = 0; iteration < 10; iteration++) {
        const midpoint = insidePoint.clone().lerp(outsidePoint, 0.5);
        if (junctionConnectorSdf(profile, midpoint) <= 0) {
            insidePoint = midpoint;
        } else {
            outsidePoint = midpoint;
        }
    }
    return insidePoint.lerp(outsidePoint, 0.5);
}

function buildImplicitJunctionConnector(profile) {
    const minimum = profile.center.clone();
    const maximum = profile.center.clone();
    for (const arm of profile.surfaceArms) {
        for (const axialDistance of [
            -profile.connectorBackwardLengthMm,
            profile.connectorLengthMm
        ]) {
            const endpoint = profile.center.clone().addScaledVector(
                arm.direction,
                axialDistance
            );
            minimum.min(endpoint.clone().addScalar(-arm.radius * 1.08));
            maximum.max(endpoint.clone().addScalar(arm.radius * 1.08));
        }
    }
    const gridSteps = profile.hasGeometricMainContinuation
        ? SIDE_OSTIUM_CONNECTOR_GRID_STEPS
        : JUNCTION_CONNECTOR_GRID_STEPS;
    const gridSize = gridSteps + 1;
    const gridPointCount = gridSize ** 3;
    const gridPoints = new Array(gridPointCount);
    const gridValues = new Float32Array(gridPointCount);
    const gridIndex = (x, y, z) =>
        x + gridSize * (y + gridSize * z);
    for (let z = 0; z < gridSize; z++) {
        for (let y = 0; y < gridSize; y++) {
            for (let x = 0; x < gridSize; x++) {
                const point = new THREE.Vector3(
                    THREE.MathUtils.lerp(
                        minimum.x,
                        maximum.x,
                        x / gridSteps
                    ),
                    THREE.MathUtils.lerp(
                        minimum.y,
                        maximum.y,
                        y / gridSteps
                    ),
                    THREE.MathUtils.lerp(
                        minimum.z,
                        maximum.z,
                        z / gridSteps
                    )
                );
                const index = gridIndex(x, y, z);
                gridPoints[index] = point;
                gridValues[index] = junctionConnectorSdf(profile, point);
            }
        }
    }

    const positions = [];
    const normals = [];
    const radii = [];
    const concentrationEdgeIndices = [];
    const concentrationEdgeTs = [];
    const connectorBlends = [];
    let omittedTubeCoveredTriangleCount = 0;
    const tetrahedra = [
        [0, 5, 1, 6],
        [0, 1, 2, 6],
        [0, 2, 3, 6],
        [0, 3, 7, 6],
        [0, 7, 4, 6],
        [0, 4, 5, 6]
    ];
    const appendTriangle = (a, b, c) => {
        const trianglePoints = [a, b, c];
        const faceNormal = b.clone().sub(a).cross(c.clone().sub(a));
        const centroid = a.clone().add(b).add(c).multiplyScalar(1 / 3);
        const armDistances = profile.surfaceArms.map(arm =>
            cappedCylinderSdf(
                centroid,
                profile.center,
                arm,
                profile.connectorBackwardLengthMm,
                profile.connectorLengthMm
            )
        );
        const nearestArmIndex = armDistances.indexOf(
            Math.min(...armDistances)
        );
        const nearestArm = profile.surfaceArms[nearestArmIndex];
        const sortedArmDistances = [...armDistances].sort(
            (left, right) => left - right
        );
        const isSingleArmSleeve =
            sortedArmDistances.length > 1 &&
            sortedArmDistances[1] > Math.max(
                0.008,
                profile.minimumSurfaceRadius * 0.008
            );
        const duplicatesContinuousMainSurface =
            profile.hasGeometricMainContinuation &&
            nearestArm.role !== 'branch';
        const isSideBranchSurface =
            profile.hasGeometricMainContinuation &&
            nearestArm.role === 'branch';
        const connectorOutwardProgress =
            junctionConnectorOutwardProgress(profile, centroid);
        const beyondSurfaceHandoff = connectorOutwardProgress >
            (profile.hasGeometricMainContinuation ? 0.92 : 0.62);
        if (
            (isSingleArmSleeve && !isSideBranchSurface) ||
            beyondSurfaceHandoff ||
            duplicatesContinuousMainSurface
        ) {
            omittedTubeCoveredTriangleCount++;
            return;
        }
        const triangleNormals = trianglePoints.map(point =>
            junctionConnectorNormal(profile, point)
        );
        const averageNormal = triangleNormals[0].clone()
            .add(triangleNormals[1])
            .add(triangleNormals[2]);
        if (faceNormal.dot(averageNormal) < 0) {
            [trianglePoints[1], trianglePoints[2]] = [
                trianglePoints[2],
                trianglePoints[1]
            ];
            [triangleNormals[1], triangleNormals[2]] = [
                triangleNormals[2],
                triangleNormals[1]
            ];
        }
        for (let index = 0; index < 3; index++) {
            const point = trianglePoints[index];
            const normal = triangleNormals[index];
            const arm = nearestJunctionSurfaceArm(profile, point);
            const outwardProgress = junctionConnectorOutwardProgress(
                profile,
                point
            );
            const axialDistanceMm = Math.max(
                0,
                point.clone().sub(profile.center).dot(arm.direction)
            );
            const axialEdgeProgress = THREE.MathUtils.clamp(
                axialDistanceMm / Math.max(1e-9, arm.edgeLength),
                0,
                1
            );
            const concentrationEdgeT = arm.edgeT < 0.5
                ? axialEdgeProgress
                : 1 - axialEdgeProgress;
            const connectorBlend =
                1 - smoothstep(0.62, 0.98, outwardProgress);
            positions.push(point.x, point.y, point.z);
            normals.push(normal.x, normal.y, normal.z);
            radii.push(junctionArmRadiusAtAxialDistance(
                arm,
                axialDistanceMm
            ));
            concentrationEdgeIndices.push(arm.edgeIndex);
            concentrationEdgeTs.push(concentrationEdgeT);
            connectorBlends.push(connectorBlend);
        }
    };
    const polygonizeTetrahedron = (points, values) => {
        const inside = [];
        const outside = [];
        for (let index = 0; index < 4; index++) {
            (values[index] <= 0 ? inside : outside).push(index);
        }
        if (!inside.length || !outside.length) return;
        const crossing = (a, b) => interpolateIsoSurfacePoint(
            profile,
            points[a],
            points[b],
            values[a],
            values[b]
        );
        if (inside.length === 1 || outside.length === 1) {
            const pivot = inside.length === 1 ? inside[0] : outside[0];
            const others = inside.length === 1 ? outside : inside;
            const triangle = others.map(index => crossing(pivot, index));
            appendTriangle(...triangle);
            return;
        }
        const [insideA, insideB] = inside;
        const [outsideA, outsideB] = outside;
        const a = crossing(insideA, outsideA);
        const b = crossing(insideA, outsideB);
        const c = crossing(insideB, outsideA);
        const d = crossing(insideB, outsideB);
        appendTriangle(a, b, c);
        appendTriangle(b, d, c);
    };

    for (let z = 0; z < gridSteps; z++) {
        for (let y = 0; y < gridSteps; y++) {
            for (let x = 0; x < gridSteps; x++) {
                const corners = [
                    gridIndex(x, y, z),
                    gridIndex(x + 1, y, z),
                    gridIndex(x + 1, y + 1, z),
                    gridIndex(x, y + 1, z),
                    gridIndex(x, y, z + 1),
                    gridIndex(x + 1, y, z + 1),
                    gridIndex(x + 1, y + 1, z + 1),
                    gridIndex(x, y + 1, z + 1)
                ];
                for (const tetrahedron of tetrahedra) {
                    polygonizeTetrahedron(
                        tetrahedron.map(index =>
                            gridPoints[corners[index]]
                        ),
                        tetrahedron.map(index =>
                            gridValues[corners[index]]
                        )
                    );
                }
            }
        }
    }
    return {
        positions,
        normals,
        radii,
        concentrationEdgeIndices,
        concentrationEdgeTs,
        connectorBlends,
        omittedTubeCoveredTriangleCount,
        vertexCount: positions.length / 3
    };
}

function junctionConnectorEnvelopeDiagnostics(profile, connector) {
    let maximumOutwardErrorMm = 0;
    let maximumAbsoluteSurfaceErrorMm = 0;
    for (
        let vertexIndex = 0;
        vertexIndex < connector.vertexCount;
        vertexIndex++
    ) {
        const positionOffset = vertexIndex * 3;
        const point = new THREE.Vector3(
            connector.positions[positionOffset],
            connector.positions[positionOffset + 1],
            connector.positions[positionOffset + 2]
        );
        const surfaceErrorMm = junctionConnectorSdf(profile, point);
        maximumOutwardErrorMm = Math.max(
            maximumOutwardErrorMm,
            surfaceErrorMm
        );
        maximumAbsoluteSurfaceErrorMm = Math.max(
            maximumAbsoluteSurfaceErrorMm,
            Math.abs(surfaceErrorMm)
        );
    }
    return {
        maximumOutwardErrorMm,
        maximumAbsoluteSurfaceErrorMm
    };
}

function buildChainRings(edges, chain, continuationChild) {
    const firstEdge = edges[chain[0]];
    const parentEdge = firstEdge.parentEdgeIndex >= 0
        ? edges[firstEdge.parentEdgeIndex]
        : null;
    const firstTopologyAnchor = bifurcationOpticalAnchor(
        edges,
        parentEdge,
        continuationChild
    );
    const firstTangent = firstEdge.axis.clone();
    if (parentEdge) {
        firstTangent.add(parentEdge.axis);
        if (firstTangent.lengthSq() < 1e-8) {
            firstTangent.copy(firstEdge.axis);
        }
        firstTangent.normalize();
    }
    const rings = [{
        center: firstEdge.start,
        radius: firstEdge.radiusStart,
        anatomicalRadius: firstEdge.radiusStart,
        topologyRadiusAnchor: !!firstTopologyAnchor,
        topologyNodeId: firstTopologyAnchor?.nodeId ?? null,
        topologyParentEdgeIndex:
            firstTopologyAnchor?.parentEdgeIndex ?? -1,
        topologyIncidentCount:
            firstTopologyAnchor?.incidentCount ?? 0,
        topologyUnionRadius:
            firstTopologyAnchor?.unionRadius ?? 0,
        topologyOpticalFadeLengthMm:
            firstTopologyAnchor?.opticalFadeLengthMm ?? 0,
        topologyOpticalWeight:
            firstTopologyAnchor?.opticalWeight ?? 1,
        topologySideOstium:
            firstTopologyAnchor?.sideOstium ?? false,
        junctionUnionMaterial: false,
        junctionSideOstium: false,
        junctionOpticalNodeId: null,
        junctionOpticalBlend: 0,
        opticalWeight: 1,
        tangent: firstTangent,
        concentrationEdgeIndex: firstEdge.index,
        concentrationEdgeT: 0,
        distance: 0
    }];
    let distance = 0;

    for (let chainIndex = 0; chainIndex < chain.length; chainIndex++) {
        const edgeIndex = chain[chainIndex];
        const edge = edges[edgeIndex];
        const endTopologyAnchor = bifurcationOpticalAnchor(
            edges,
            edge,
            continuationChild
        );
        const nextEdge = chainIndex + 1 < chain.length
            ? edges[chain[chainIndex + 1]]
            : null;
        const cellLength = edge.length / edge.cellCount;
        for (
            let cellBoundary = 1;
            cellBoundary <= edge.cellCount;
            cellBoundary++
        ) {
            const t = cellBoundary / edge.cellCount;
            distance += cellLength;
            const tangent = edge.axis.clone();
            if (cellBoundary === edge.cellCount && nextEdge) {
                tangent.add(nextEdge.axis);
                if (tangent.lengthSq() < 1e-8) tangent.copy(nextEdge.axis);
                tangent.normalize();
            }
            rings.push({
                center: edge.start.clone().lerp(edge.end, t),
                radius: THREE.MathUtils.lerp(
                    edge.radiusStart,
                    edge.radiusEnd,
                    t
                ),
                anatomicalRadius: THREE.MathUtils.lerp(
                    edge.radiusStart,
                    edge.radiusEnd,
                    t
                ),
                topologyRadiusAnchor:
                    cellBoundary === edge.cellCount &&
                    !!endTopologyAnchor,
                topologyNodeId:
                    cellBoundary === edge.cellCount
                        ? endTopologyAnchor?.nodeId ?? null
                        : null,
                topologyParentEdgeIndex:
                    cellBoundary === edge.cellCount
                        ? endTopologyAnchor?.parentEdgeIndex ?? -1
                        : -1,
                topologyIncidentCount:
                    cellBoundary === edge.cellCount
                        ? endTopologyAnchor?.incidentCount ?? 0
                        : 0,
                topologyUnionRadius:
                    cellBoundary === edge.cellCount
                        ? endTopologyAnchor?.unionRadius ?? 0
                        : 0,
                topologyOpticalFadeLengthMm:
                    cellBoundary === edge.cellCount
                        ? endTopologyAnchor?.opticalFadeLengthMm ?? 0
                        : 0,
                topologyOpticalWeight:
                    cellBoundary === edge.cellCount
                        ? endTopologyAnchor?.opticalWeight ?? 1
                        : 1,
                topologySideOstium:
                    cellBoundary === edge.cellCount
                        ? endTopologyAnchor?.sideOstium ?? false
                        : false,
                junctionUnionMaterial: false,
                junctionSideOstium: false,
                junctionOpticalNodeId: null,
                junctionOpticalBlend: 0,
                opticalWeight: 1,
                tangent,
                concentrationEdgeIndex: edgeIndex,
                concentrationEdgeT: t,
                distance
            });
        }
    }

    // Radius samples near tight bends occasionally collapse to the distance
    // from the centreline to the inner wall instead of the cross-sectional
    // lumen radius. First smooth ordinary noise, then close only short valleys
    // that have a larger calibre on both sides. This preserves genuinely small
    // branches while removing the needle-like artefacts seen in the iliac.
    const rawRadii = Float32Array.from(
        rings,
        ring => ring.radius
    );
    const smoothedRadii = new Float32Array(rings.length);
    const smoothingSigmaMm = 8;
    const smoothingWindowMm = smoothingSigmaMm * 2.5;
    for (let ringIndex = 0; ringIndex < rings.length; ringIndex++) {
        let weightedRadius = 0;
        let totalWeight = 0;
        for (
            let neighbourIndex = ringIndex;
            neighbourIndex >= 0;
            neighbourIndex--
        ) {
            const delta =
                rings[ringIndex].distance - rings[neighbourIndex].distance;
            if (delta > smoothingWindowMm) break;
            const weight = Math.exp(
                -0.5 * (delta / smoothingSigmaMm) ** 2
            );
            weightedRadius += rawRadii[neighbourIndex] * weight;
            totalWeight += weight;
        }
        for (
            let neighbourIndex = ringIndex + 1;
            neighbourIndex < rings.length;
            neighbourIndex++
        ) {
            const delta =
                rings[neighbourIndex].distance - rings[ringIndex].distance;
            if (delta > smoothingWindowMm) break;
            const weight = Math.exp(
                -0.5 * (delta / smoothingSigmaMm) ** 2
            );
            weightedRadius += rawRadii[neighbourIndex] * weight;
            totalWeight += weight;
        }
        smoothedRadii[ringIndex] =
            weightedRadius / Math.max(1e-9, totalWeight);
    }
    for (let ringIndex = 0; ringIndex < rings.length; ringIndex++) {
        rings[ringIndex].radius = smoothedRadii[ringIndex];
    }

    const valleyWindowMm = 28;
    const correctedRadii = new Float32Array(rings.length);
    for (let ringIndex = 0; ringIndex < rings.length; ringIndex++) {
        let leftPeak = rings[ringIndex].radius;
        let rightPeak = rings[ringIndex].radius;
        let hasLeftSupport = false;
        let hasRightSupport = false;
        for (
            let neighbourIndex = ringIndex - 1;
            neighbourIndex >= 0;
            neighbourIndex--
        ) {
            const delta =
                rings[ringIndex].distance - rings[neighbourIndex].distance;
            if (delta > valleyWindowMm) break;
            leftPeak = Math.max(leftPeak, rawRadii[neighbourIndex]);
            hasLeftSupport ||= delta >= 6;
        }
        for (
            let neighbourIndex = ringIndex + 1;
            neighbourIndex < rings.length;
            neighbourIndex++
        ) {
            const delta =
                rings[neighbourIndex].distance - rings[ringIndex].distance;
            if (delta > valleyWindowMm) break;
            rightPeak = Math.max(rightPeak, rawRadii[neighbourIndex]);
            hasRightSupport ||= delta >= 6;
        }
        const supportedRadius = Math.min(leftPeak, rightPeak);
        const currentRadius = rings[ringIndex].radius;
        correctedRadii[ringIndex] =
            hasLeftSupport &&
            hasRightSupport &&
            currentRadius < supportedRadius * 0.78
                ? Math.max(currentRadius, supportedRadius * 0.92)
                : currentRadius;
    }

    const finalRadii = new Float32Array(rings.length);
    const finalSigmaMm = 2.5;
    const finalWindowMm = 7;
    for (let ringIndex = 0; ringIndex < rings.length; ringIndex++) {
        let weightedRadius = 0;
        let totalWeight = 0;
        for (let neighbourIndex = 0;
            neighbourIndex < rings.length;
            neighbourIndex++
        ) {
            const distance = Math.abs(
                rings[neighbourIndex].distance - rings[ringIndex].distance
            );
            if (distance > finalWindowMm) continue;
            const weight = Math.exp(
                -0.5 * (distance / finalSigmaMm) ** 2
            );
            weightedRadius += correctedRadii[neighbourIndex] * weight;
            totalWeight += weight;
        }
        finalRadii[ringIndex] =
            weightedRadius / Math.max(1e-9, totalWeight);
    }
    for (let ringIndex = 0; ringIndex < rings.length; ringIndex++) {
        rings[ringIndex].radius = finalRadii[ringIndex];
    }

    // Independent centreline chains are convenient for tube construction,
    // but a child chain begins exactly where its parent bifurcates. A
    // one-sided Gaussian at that boundary sees only the smaller downstream
    // samples and can therefore invent a stenosis at an otherwise normal
    // ostium. Treat every anatomical bifurcation radius as a Dirichlet-style
    // topology anchor and fade its correction over a short physical distance.
    // This remains generic: each parent and child keeps its own encoded
    // calibre, including a real stenosis if the source anatomy contains one.
    const anchorCorrections = [];
    for (let ringIndex = 0; ringIndex < rings.length; ringIndex++) {
        if (!rings[ringIndex].topologyRadiusAnchor) continue;
        anchorCorrections.push({
            ringIndex,
            correction:
                rings[ringIndex].anatomicalRadius -
                rings[ringIndex].radius
        });
    }
    if (anchorCorrections.length) {
        const topologyCorrectedRadii = new Float32Array(rings.length);
        for (let ringIndex = 0; ringIndex < rings.length; ringIndex++) {
            let weightedCorrection = 0;
            let totalWeight = 0;
            for (const anchor of anchorCorrections) {
                const distance = Math.abs(
                    rings[ringIndex].distance -
                    rings[anchor.ringIndex].distance
                );
                if (distance > FLOW_TOPOLOGY_RADIUS_BLEND_MM) continue;
                const normalizedDistance =
                    distance / FLOW_TOPOLOGY_RADIUS_BLEND_MM;
                const weight =
                    1 - smoothstep(0, 1, normalizedDistance);
                weightedCorrection += anchor.correction * weight;
                totalWeight += weight;
            }
            topologyCorrectedRadii[ringIndex] = Math.max(
                0.2,
                rings[ringIndex].radius +
                    weightedCorrection / Math.max(1, totalWeight)
            );
        }
        for (let ringIndex = 0; ringIndex < rings.length; ringIndex++) {
            rings[ringIndex].radius = rings[ringIndex].topologyRadiusAnchor
                ? rings[ringIndex].anatomicalRadius
                : topologyCorrectedRadii[ringIndex];
        }
    }

    // The full-calibre parent and child rings provide concentration samples,
    // but their local surfaces are handed over to one implicit lumen union.
    // The fade metadata below suppresses the intersecting tube surfaces while
    // that connector takes over. All junction surfaces use maximum optical
    // blending, so overlapping tubes and the fitted connector describe one
    // lumen without multiplying its signal. Side ostia additionally taper
    // only the emerging branch at the parent wall.
    const opticalAnchors = rings.filter(
        ring =>
            ring.topologyRadiusAnchor &&
            ring.topologyOpticalFadeLengthMm > 0
    );
    for (const anchor of opticalAnchors) {
        const unionLengthMm = anchor.topologyOpticalFadeLengthMm;
        for (const ring of rings) {
            const distance = Math.abs(ring.distance - anchor.distance);
            if (distance >= unionLengthMm) continue;
            const junctionOpticalBlend =
                1 - smoothstep(0, 1, distance / unionLengthMm);
            ring.junctionUnionMaterial = true;
            ring.junctionSideOstium ||= anchor.topologySideOstium;
            ring.opticalWeight = 1;
            if (
                junctionOpticalBlend > ring.junctionOpticalBlend
            ) {
                ring.junctionOpticalNodeId = anchor.topologyNodeId;
                ring.junctionOpticalBlend = junctionOpticalBlend;
            }
        }
    }
    for (const anchor of opticalAnchors) {
        // A very short segment can lie inside the fade zones of two adjacent
        // bifurcations. Each exact topology boundary still belongs to its own
        // connector so that the corresponding implicit surface takes over.
        anchor.junctionUnionMaterial = true;
        anchor.junctionSideOstium = anchor.topologySideOstium;
        anchor.junctionOpticalNodeId = anchor.topologyNodeId;
        anchor.junctionOpticalBlend = 1;
        anchor.opticalWeight = anchor.topologyOpticalWeight;
    }
    return rings;
}

function createFlowLumenGeometry(edges, contactField = null) {
    const { chains, continuationChild } = buildEdgeChains(edges);
    const chainRings = chains.map(
        chain => buildChainRings(edges, chain, continuationChild)
    );
    const junctions = edges.filter(edge =>
        !edge.transportExcluded &&
        !edge.renderExcluded &&
        edge.childEdgeIndices.filter(childIndex =>
            !edges[childIndex]?.transportExcluded &&
            !edges[childIndex]?.renderExcluded
        ).length >= 2 &&
        edge.radiusEnd >= FLOW_MIN_JUNCTION_RADIUS_MM
    );
    const junctionDynamicProfiles = junctions.map(parent => {
        const renderedChildren = parent.childEdgeIndices
            .map(childIndex => edges[childIndex])
            .filter(child =>
                child &&
                !child.transportExcluded &&
                !child.renderExcluded
            );
        const continuationEdgeIndex = continuationChild[parent.index];
        const continuation = continuationEdgeIndex >= 0
            ? edges[continuationEdgeIndex]
            : null;
        const hasGeometricMainContinuation =
            !!continuation;
        const arms = continuation
            ? [
                {
                    role: 'main',
                    radius: Math.max(
                        0.2,
                        parent.radiusEnd,
                        continuation.radiusStart
                    ),
                    samples: [
                        { edgeIndex: parent.index, edgeT: 1 },
                        { edgeIndex: continuation.index, edgeT: 0 }
                    ]
                },
                ...renderedChildren
                    .filter(child => child.index !== continuationEdgeIndex)
                    .map(child => ({
                        role: 'side-child',
                        radius: Math.max(0.2, child.radiusStart),
                        samples: [{ edgeIndex: child.index, edgeT: 0 }]
                    }))
            ]
            : [
                {
                    role: 'parent',
                    radius: Math.max(0.2, parent.radiusEnd),
                    samples: [{ edgeIndex: parent.index, edgeT: 1 }]
                },
                ...renderedChildren.map(child => ({
                    role: 'child',
                    radius: Math.max(0.2, child.radiusStart),
                    samples: [{ edgeIndex: child.index, edgeT: 0 }]
                }))
            ];
        const unionRadius = Math.max(...arms.map(arm => arm.radius));
        const minimumSurfaceRadius = Math.min(
            parent.radiusEnd,
            ...renderedChildren.map(child => child.radiusStart)
        );
        const connectorLengthMm = Math.max(
            continuation ? 2.5 : 0.5,
            unionRadius * (
                hasGeometricMainContinuation
                    ? FLOW_JUNCTION_UNION_LENGTH_FACTOR
                    : 0.58
            )
        );
        const surfaceArms = [
            {
                role: 'parent',
                edgeIndex: parent.index,
                edgeT: 1,
                edgeLength: parent.length,
                direction: parent.axis.clone().negate().normalize(),
                radius: Math.max(0.2, parent.radiusEnd),
                taperFromCenter: false,
                ostiumRampLengthMm: 0,
                ostiumStartRadiusFraction: 1
            },
            ...renderedChildren.map(child => {
                const continuationArm =
                    hasGeometricMainContinuation &&
                    child.index === continuationEdgeIndex;
                const alignment = THREE.MathUtils.clamp(
                    Math.abs(parent.axis.dot(child.axis)),
                    0,
                    1
                );
                const departureSine = Math.sqrt(
                    Math.max(0, 1 - alignment * alignment)
                );
                return {
                    role: continuationArm
                        ? 'continuation'
                        : 'branch',
                    edgeIndex: child.index,
                    edgeT: 0,
                    edgeLength: child.length,
                    direction: child.axis.clone().normalize(),
                    radius: Math.max(0.2, child.radiusStart),
                    taperFromCenter:
                        hasGeometricMainContinuation &&
                        !continuationArm,
                    ostiumRampLengthMm: Math.min(
                        connectorLengthMm * 0.78,
                        unionRadius / Math.max(0.35, departureSine)
                    ),
                    ostiumStartRadiusFraction:
                        hasGeometricMainContinuation ? 0.12 : 1
                };
            })
        ];
        return {
            nodeId: parent.endNodeId,
            parentEdgeIndex: parent.index,
            connectorEnabled: true,
            hasGeometricMainContinuation,
            center: parent.end.clone(),
            arms,
            surfaceArms,
            unionRadius,
            minimumSurfaceRadius,
            connectorLengthMm,
            connectorBackwardLengthMm:
                Math.max(0.2, minimumSurfaceRadius * 0.18),
            connectorSmoothingRadiusMm:
                Math.max(0.035, minimumSurfaceRadius * 0.06),
            connectorNormalSmoothingRadiusMm:
                Math.max(0.035, minimumSurfaceRadius * 0.12),
            staticOpticalWeight: hasGeometricMainContinuation
                ? 1
                : THREE.MathUtils.clamp(
                    unionRadius /
                        Math.max(
                            1e-9,
                            Math.sqrt(arms.reduce(
                                (sum, arm) =>
                                    sum + arm.radius * arm.radius,
                                0
                            ))
                        ),
                    0,
                    1
                )
        };
    });
    const junctionSlotByNodeId = new Map(
        junctionDynamicProfiles.map((profile, slot) => [
            profile.nodeId,
            slot
        ])
    );
    const junctionConnectorMeshes = junctionDynamicProfiles.map(
        profile => buildImplicitJunctionConnector(profile)
    );
    const junctionAnatomicalCoverages = junctionConnectorMeshes.map(
        (connector, junctionIndex) => sideOstiumAnatomicalCoverage(
            junctionDynamicProfiles[junctionIndex],
            connector,
            contactField
        )
    );
    const junctionEnvelopeDiagnostics =
        junctionConnectorMeshes.map((connector, junctionIndex) =>
            junctionConnectorEnvelopeDiagnostics(
                junctionDynamicProfiles[junctionIndex],
                connector
            )
        );
    const junctionUnionDiagnostics = junctions.map((parent, junctionIndex) => {
        const renderedChildren = parent.childEdgeIndices
            .map(childIndex => edges[childIndex])
            .filter(child =>
                child &&
                !child.transportExcluded &&
                !child.renderExcluded
            );
        const profile = junctionUnionProfile(parent, renderedChildren);
        const arms = [
            {
                role: 'parent',
                edgeIndex: parent.index,
                direction: parent.axis.clone().negate().normalize(),
                radius: profile.incidentRadii[0]
            },
            ...renderedChildren.map((child, childIndex) => ({
                role: 'child',
                edgeIndex: child.index,
                direction: child.axis.clone().normalize(),
                radius: profile.incidentRadii[childIndex + 1]
            }))
        ];
        return {
            geometryKind:
                junctionDynamicProfiles[junctionIndex]
                    .hasGeometricMainContinuation
                    ? 'implicit-radius-matched-side-ostium-union'
                    : 'implicit-radius-matched-y-union',
            nodeId: parent.endNodeId,
            parentEdgeIndex: parent.index,
            incidentCount: profile.incidentRadii.length,
            minimumIncidentRadius: profile.minimumIncidentRadius,
            maximumIncidentRadius: profile.maximumIncidentRadius,
            continuationAlignment: profile.continuationAlignment,
            continuationFlowFraction:
                profile.continuationFlowFraction,
            sideBranchDominance: profile.sideBranchDominance,
            signalMode: 'implicit-maximum-optical-union',
            surfaceMode:
                junctionAnatomicalCoverages[junctionIndex]
                    .suppressConnectorSurface
                    ? 'contact-field-max-tube-union'
                    : 'exact-union-smoothed-normals',
            unionLengthMm: Math.max(
                profile.sideBranchDominance > 0 ? 2.5 : 0.75,
                profile.maximumIncidentRadius *
                    FLOW_JUNCTION_UNION_LENGTH_FACTOR
            ),
            connectorVertexCount:
                junctionConnectorMeshes[junctionIndex].vertexCount,
            omittedTubeCoveredTriangleCount:
                junctionConnectorMeshes[junctionIndex]
                    .omittedTubeCoveredTriangleCount,
            maximumOutwardErrorMm:
                junctionEnvelopeDiagnostics[junctionIndex]
                    .maximumOutwardErrorMm,
            maximumAbsoluteSurfaceErrorMm:
                junctionEnvelopeDiagnostics[junctionIndex]
                    .maximumAbsoluteSurfaceErrorMm,
            anatomicalClipMode:
                junctionDynamicProfiles[junctionIndex]
                    .hasGeometricMainContinuation &&
                contactField?.querySphere
                    ? 'side-ostium-contact-field'
                    : 'disabled',
            anatomicalClippedVertexCount:
                junctionAnatomicalCoverages[junctionIndex]
                    .clippedVertexCount,
            minimumAnatomicalCoverage:
                junctionAnatomicalCoverages[junctionIndex]
                    .minimumCoverage,
            maximumAnatomicalOutsideDistanceMm:
                junctionAnatomicalCoverages[junctionIndex]
                    .maximumOutsideDistanceMm,
            connectorSurfaceSuppressed:
                junctionAnatomicalCoverages[junctionIndex]
                    .suppressConnectorSurface,
            armRadii: arms.map(arm => arm.radius),
            armDirections: arms.map(arm => arm.direction.toArray())
        };
    });
    const ringCount = chainRings.reduce(
        (sum, rings) => sum + rings.length,
        0
    );
    const axialSegmentCount = chains.reduce(
        (sum, chain) => sum + chain.reduce(
            (chainSum, edgeIndex) =>
                chainSum + edges[edgeIndex].cellCount,
            0
        ),
        0
    );
    const tubeVertexCount = ringCount * FLOW_RADIAL_SEGMENTS;
    const connectorVertexCount = junctionConnectorMeshes.reduce(
        (sum, connector) => sum + connector.vertexCount,
        0
    );
    const vertexCount = tubeVertexCount + connectorVertexCount;
    const positions = new Float32Array(vertexCount * 3);
    const normals = new Float32Array(vertexCount * 3);
    const radii = new Float32Array(vertexCount);
    const opticalWeights = new Float32Array(vertexCount);
    const junctionOpticalBlend = new Float32Array(vertexCount);
    const junctionOpticalSlot = new Int32Array(vertexCount);
    junctionOpticalSlot.fill(-1);
    const junctionConnectorBlend = new Float32Array(vertexCount);
    const junctionConnectorVertex = new Uint8Array(vertexCount);
    const concentrations = new Float32Array(vertexCount);
    const concentrationEdgeIndex = new Uint32Array(vertexCount);
    const concentrationEdgeT = new Float32Array(vertexCount);
    let vertexOffset = 0;
    const ordinaryTubeIndices = [];
    const junctionUnionIndices = [];
    const sideOstiumTubeIndices = [];
    const junctionConnectorIndices = [];
    const sideOstiumConnectorIndices = [];

    for (const rings of chainRings) {
        const chainVertexStart = vertexOffset;
        let frameNormal = null;
        for (let ringIndex = 0; ringIndex < rings.length; ringIndex++) {
            const ring = rings[ringIndex];
            frameNormal = frameNormal
                ? transportedFrameNormal(frameNormal, ring.tangent)
                : initialFrameNormal(ring.tangent);
            const frameBinormal = new THREE.Vector3()
                .crossVectors(ring.tangent, frameNormal)
                .normalize();
            const radius = Math.max(
                0.2,
                ring.radius * FLOW_LUMEN_RADIUS_FACTOR
            );

            for (
                let radialIndex = 0;
                radialIndex < FLOW_RADIAL_SEGMENTS;
                radialIndex++
            ) {
                const angle =
                    radialIndex / FLOW_RADIAL_SEGMENTS * Math.PI * 2;
                const radial = frameNormal.clone()
                    .multiplyScalar(Math.cos(angle))
                    .addScaledVector(frameBinormal, Math.sin(angle));
                const vertexIndex = vertexOffset++;
                const vertexPosition = ring.center.clone()
                    .addScaledVector(radial, radius);
                positions[vertexIndex * 3] = vertexPosition.x;
                positions[vertexIndex * 3 + 1] = vertexPosition.y;
                positions[vertexIndex * 3 + 2] = vertexPosition.z;
                normals[vertexIndex * 3] = radial.x;
                normals[vertexIndex * 3 + 1] = radial.y;
                normals[vertexIndex * 3 + 2] = radial.z;
                radii[vertexIndex] = radius;
                const junctionSlot =
                    ring.junctionOpticalNodeId === null
                        ? -1
                        : junctionSlotByNodeId.get(
                            ring.junctionOpticalNodeId
                        ) ?? -1;
                junctionOpticalSlot[vertexIndex] = junctionSlot;
                junctionOpticalBlend[vertexIndex] =
                    junctionSlot >= 0
                        ? ring.junctionOpticalBlend *
                            junctionSurfaceOverlap(
                                junctionDynamicProfiles[junctionSlot],
                                vertexPosition,
                                ring.concentrationEdgeIndex
                            )
                        : 0;
                junctionConnectorBlend[vertexIndex] =
                    junctionSlot >= 0 &&
                    junctionDynamicProfiles[junctionSlot].connectorEnabled
                        ? ring.junctionOpticalBlend
                        : 0;
                opticalWeights[vertexIndex] = 1;
                concentrationEdgeIndex[vertexIndex] =
                    ring.concentrationEdgeIndex;
                concentrationEdgeT[vertexIndex] =
                    ring.concentrationEdgeT;
            }
        }

        for (let ringIndex = 0; ringIndex < rings.length - 1; ringIndex++) {
            const junctionSegment =
                rings[ringIndex].junctionUnionMaterial ||
                rings[ringIndex + 1].junctionUnionMaterial;
            const sideOstiumSegment =
                rings[ringIndex].junctionSideOstium ||
                rings[ringIndex + 1].junctionSideOstium;
            const targetIndices = !junctionSegment
                ? ordinaryTubeIndices
                : sideOstiumSegment
                    ? sideOstiumTubeIndices
                    : junctionUnionIndices;
            const ringStart =
                chainVertexStart + ringIndex * FLOW_RADIAL_SEGMENTS;
            const nextRingStart = ringStart + FLOW_RADIAL_SEGMENTS;
            for (
                let radialIndex = 0;
                radialIndex < FLOW_RADIAL_SEGMENTS;
                radialIndex++
            ) {
                const radialNext =
                    (radialIndex + 1) % FLOW_RADIAL_SEGMENTS;
                const a = ringStart + radialIndex;
                const b = nextRingStart + radialIndex;
                const c = nextRingStart + radialNext;
                const d = ringStart + radialNext;
                targetIndices.push(a, b, d, b, c, d);
            }
        }
    }

    for (
        let slot = 0;
        slot < junctionConnectorMeshes.length;
        slot++
    ) {
        const connector = junctionConnectorMeshes[slot];
        const connectorVertexStart = vertexOffset;
        for (
            let localVertexIndex = 0;
            localVertexIndex < connector.vertexCount;
            localVertexIndex++
        ) {
            const vertexIndex = vertexOffset++;
            const sourceOffset = localVertexIndex * 3;
            positions[vertexIndex * 3] =
                connector.positions[sourceOffset];
            positions[vertexIndex * 3 + 1] =
                connector.positions[sourceOffset + 1];
            positions[vertexIndex * 3 + 2] =
                connector.positions[sourceOffset + 2];
            normals[vertexIndex * 3] =
                connector.normals[sourceOffset];
            normals[vertexIndex * 3 + 1] =
                connector.normals[sourceOffset + 1];
            normals[vertexIndex * 3 + 2] =
                connector.normals[sourceOffset + 2];
            radii[vertexIndex] = connector.radii[localVertexIndex];
            const connectorBlend =
                connector.connectorBlends[localVertexIndex];
            opticalWeights[vertexIndex] = 1;
            junctionOpticalSlot[vertexIndex] = slot;
            junctionConnectorBlend[vertexIndex] = connectorBlend;
            junctionConnectorVertex[vertexIndex] = 1;
            concentrationEdgeIndex[vertexIndex] =
                connector.concentrationEdgeIndices[localVertexIndex];
            concentrationEdgeT[vertexIndex] =
                connector.concentrationEdgeTs[localVertexIndex];
        }
        for (
            let localVertexIndex = 0;
            localVertexIndex < connector.vertexCount;
            localVertexIndex += 3
        ) {
            if (
                junctionDynamicProfiles[slot]
                    .hasGeometricMainContinuation &&
                junctionAnatomicalCoverages[slot]
                    .suppressConnectorSurface
            ) continue;
            const targetConnectorIndices =
                junctionDynamicProfiles[slot]
                    .hasGeometricMainContinuation
                    ? sideOstiumConnectorIndices
                    : junctionConnectorIndices;
            targetConnectorIndices.push(
                connectorVertexStart + localVertexIndex,
                connectorVertexStart + localVertexIndex + 1,
                connectorVertexStart + localVertexIndex + 2,
                connectorVertexStart + localVertexIndex,
                connectorVertexStart + localVertexIndex + 2,
                connectorVertexStart + localVertexIndex + 1
            );
        }
    }

    if (vertexOffset !== vertexCount) {
        throw new Error(
            `Flow lumen vertex accounting mismatch: ${vertexOffset}/${vertexCount}`
        );
    }

    const tubeIndexCount = ordinaryTubeIndices.length;
    const junctionTubeIndexCount = junctionUnionIndices.length;
    const junctionConnectorIndexCount =
        junctionConnectorIndices.length;
    const sideOstiumTubeIndexCount = sideOstiumTubeIndices.length;
    const sideOstiumConnectorIndexCount =
        sideOstiumConnectorIndices.length;
    const junctionIndexCount =
        junctionTubeIndexCount +
        junctionConnectorIndexCount +
        sideOstiumTubeIndexCount +
        sideOstiumConnectorIndexCount;
    const indices = vertexCount > 65535
        ? new Uint32Array(tubeIndexCount + junctionIndexCount)
        : new Uint16Array(tubeIndexCount + junctionIndexCount);
    indices.set(ordinaryTubeIndices, 0);
    indices.set(junctionUnionIndices, tubeIndexCount);
    indices.set(
        junctionConnectorIndices,
        tubeIndexCount + junctionTubeIndexCount
    );
    indices.set(
        sideOstiumTubeIndices,
        tubeIndexCount +
            junctionTubeIndexCount +
            junctionConnectorIndexCount
    );
    indices.set(
        sideOstiumConnectorIndices,
        tubeIndexCount +
            junctionTubeIndexCount +
            junctionConnectorIndexCount +
            sideOstiumTubeIndexCount
    );

    const geometry = new THREE.BufferGeometry();
    geometry.setAttribute(
        'position',
        new THREE.BufferAttribute(positions, 3)
    );
    geometry.setAttribute(
        'normal',
        new THREE.BufferAttribute(normals, 3)
    );
    geometry.setAttribute(
        'flowRadius',
        new THREE.BufferAttribute(radii, 1)
    );
    geometry.setAttribute(
        'flowOpticalWeight',
        new THREE.BufferAttribute(opticalWeights, 1)
    );
    geometry.setAttribute(
        'flowConcentration',
        new THREE.BufferAttribute(concentrations, 1)
    );
    geometry.setIndex(new THREE.BufferAttribute(indices, 1));
    geometry.addGroup(0, tubeIndexCount, 0);
    geometry.addGroup(
        tubeIndexCount,
        junctionTubeIndexCount,
        1
    );
    geometry.addGroup(
        tubeIndexCount + junctionTubeIndexCount,
        junctionConnectorIndexCount,
        2
    );
    geometry.addGroup(
        tubeIndexCount +
            junctionTubeIndexCount +
            junctionConnectorIndexCount,
        sideOstiumTubeIndexCount,
        3
    );
    geometry.addGroup(
        tubeIndexCount +
            junctionTubeIndexCount +
            junctionConnectorIndexCount +
            sideOstiumTubeIndexCount,
        sideOstiumConnectorIndexCount,
        4
    );
    geometry.computeBoundingSphere();
    return {
        geometry,
        concentrations,
        opticalWeights,
        junctionOpticalBlend,
        junctionOpticalSlot,
        junctionConnectorBlend,
        junctionConnectorVertex,
        junctionDynamicProfiles,
        concentrationEdgeIndex,
        concentrationEdgeT,
        chains,
        continuationChild,
        chainCount: chains.length,
        ringCount,
        tubeVertexCount,
        connectorVertexCount,
        axialSegmentCount,
        junctionCount: junctions.length,
        tubeIndexCount,
        junctionTubeIndexCount,
        junctionConnectorIndexCount,
        sideOstiumTubeIndexCount,
        sideOstiumConnectorIndexCount,
        junctionIndexCount,
        junctionUnionDiagnostics,
        topologyRadiusAnchors: chainRings.flatMap(rings =>
            rings
                .filter(ring => ring.topologyRadiusAnchor)
                .map(ring => ({
                    edgeIndex: ring.concentrationEdgeIndex,
                    edgeT: ring.concentrationEdgeT,
                    anatomicalRadius: ring.anatomicalRadius,
                    renderedRadius: ring.radius,
                    nodeId: ring.topologyNodeId,
                    parentEdgeIndex:
                        ring.topologyParentEdgeIndex,
                    incidentCount: ring.topologyIncidentCount,
                    unionRadius: ring.topologyUnionRadius,
                    opticalFadeLengthMm:
                        ring.topologyOpticalFadeLengthMm,
                    opticalWeight: ring.opticalWeight
                }))
        )
    };
}

function createJunctionFlowMaterial() {
    return createFlowMaterial({ maximumBlend: true });
}

function createJunctionConnectorFlowMaterial() {
    return createFlowMaterial({ maximumBlend: true });
}

function createSideOstiumFlowMaterial() {
    return createFlowMaterial({ maximumBlend: true });
}

function createSideOstiumConnectorFlowMaterial() {
    return createFlowMaterial({ maximumBlend: true });
}

function createPlumeGeometry(capacity) {
    const geometry = new THREE.InstancedBufferGeometry();
    geometry.setAttribute(
        'position',
        new THREE.Float32BufferAttribute([
            -1, -1, 0,
             1, -1, 0,
             1,  1, 0,
            -1,  1, 0
        ], 3)
    );
    geometry.setIndex([0, 1, 2, 0, 2, 3]);

    const arrays = {
        center: new Float32Array(capacity * 3),
        velocity: new Float32Array(capacity * 3),
        radialRadius: new Float32Array(capacity),
        axialHalfLength: new Float32Array(capacity),
        concentration: new Float32Array(capacity),
        opticalScale: new Float32Array(capacity)
    };
    geometry.setAttribute(
        'instanceCenter',
        new THREE.InstancedBufferAttribute(arrays.center, 3)
    );
    geometry.setAttribute(
        'instanceVelocity',
        new THREE.InstancedBufferAttribute(arrays.velocity, 3)
    );
    geometry.setAttribute(
        'radialRadius',
        new THREE.InstancedBufferAttribute(arrays.radialRadius, 1)
    );
    geometry.setAttribute(
        'axialHalfLength',
        new THREE.InstancedBufferAttribute(arrays.axialHalfLength, 1)
    );
    geometry.setAttribute(
        'plumeConcentration',
        new THREE.InstancedBufferAttribute(arrays.concentration, 1)
    );
    geometry.setAttribute(
        'opticalScale',
        new THREE.InstancedBufferAttribute(arrays.opticalScale, 1)
    );
    geometry.instanceCount = 0;
    return { geometry, arrays };
}

export class ContrastVolumeRenderer {
    constructor(system) {
        if (!system?.flowNetwork || !system?.localSolver) {
            throw new TypeError('A hybrid contrast system is required');
        }
        this.system = system;
        this.group = new THREE.Group();
        this.group.name = 'hybrid-contrast-volume';
        this.group.frustumCulled = false;
        this._flowRawConcentration = new Float32Array(
            system.flowNetwork.edges.length
        );
        this._flowSmoothConcentration = new Float32Array(
            system.flowNetwork.edges.length
        );
        this._flowDisplayConcentration = new Float32Array(
            system.flowNetwork.edges.length
        );
        this._flowCellOffset = new Uint32Array(
            system.flowNetwork.edges.length
        );
        let totalFlowCellCount = 0;
        for (
            let edgeIndex = 0;
            edgeIndex < system.flowNetwork.edges.length;
            edgeIndex++
        ) {
            this._flowCellOffset[edgeIndex] = totalFlowCellCount;
            totalFlowCellCount +=
                system.flowNetwork.edges[edgeIndex].cellCount;
        }
        this._flowCellRawConcentration =
            new Float32Array(totalFlowCellCount);
        this._flowCellSmoothConcentration =
            new Float32Array(totalFlowCellCount);
        this._flowCellDisplayConcentration =
            new Float32Array(totalFlowCellCount);
        this._flowCellLocalPlumeMassMg =
            new Float32Array(totalFlowCellCount);
        this._flowCellOnPressureDrivenPath =
            new Uint8Array(totalFlowCellCount);
        this._wasPressureDrivenColumnActive =
            !!this.system.pressureDrivenRetrogradeColumn?.isFlowReversed;
        this._pressureDrivenColumnEndTime = null;
        this.lastDisconnectedBranchFade = 1;
        this._debugMode = false;
        this._plumeLocationScratch = {};
        this._createFlowLumen();
        this._createLocalPlume();
        this.group.add(this.flowMesh, this.plumeMesh);
        this.update();
    }

    _createFlowLumen() {
        const flow = createFlowLumenGeometry(
            this.system.flowNetwork.edges,
            this.system.contactField
        );
        this._flowVertexConcentration = flow.concentrations;
        this._flowVertexOpticalWeight = flow.opticalWeights;
        this._flowVertexJunctionOpticalBlend =
            flow.junctionOpticalBlend;
        this._flowVertexJunctionOpticalSlot =
            flow.junctionOpticalSlot;
        this._flowVertexJunctionConnectorBlend =
            flow.junctionConnectorBlend;
        this._flowVertexIsJunctionConnector =
            flow.junctionConnectorVertex;
        this._flowJunctionDynamicProfiles =
            flow.junctionDynamicProfiles;
        const dynamicOpticalVertexIndices = [];
        const trueJunctionConnectorVertexIndices = [];
        for (
            let vertexIndex = 0;
            vertexIndex < flow.concentrations.length;
            vertexIndex++
        ) {
            const slot = flow.junctionOpticalSlot[vertexIndex];
            if (
                slot >= 0 &&
                flow.junctionConnectorVertex[vertexIndex] &&
                !flow.junctionDynamicProfiles[slot]
                    .hasGeometricMainContinuation
            ) {
                trueJunctionConnectorVertexIndices.push(vertexIndex);
            }
        }
        this._flowTrueJunctionConnectorVertexIndices =
            Uint32Array.from(trueJunctionConnectorVertexIndices);
        this._flowTrueJunctionConnectorConcentration =
            new Float32Array(flow.junctionDynamicProfiles.length);
        this._flowDynamicOpticalVertexIndices = Uint32Array.from(
            dynamicOpticalVertexIndices
        );
        this._flowJunctionDynamicWeight = new Float32Array(
            flow.junctionDynamicProfiles.length
        );
        this.flowJunctionDynamicWeights =
            this._flowJunctionDynamicWeight;
        this._flowVertexConcentrationEdgeIndex =
            flow.concentrationEdgeIndex;
        this._flowVertexConcentrationEdgeT =
            flow.concentrationEdgeT;
        this._flowVertexConcentrationSampleSlot = new Uint32Array(
            flow.concentrations.length
        );
        const concentrationSampleSlotByKey = new Map();
        const concentrationSampleEdgeIndices = [];
        const concentrationSampleEdgeTs = [];
        for (
            let vertexIndex = 0;
            vertexIndex < flow.concentrations.length;
            vertexIndex++
        ) {
            const edgeIndex = flow.concentrationEdgeIndex[vertexIndex];
            const edgeT = flow.concentrationEdgeT[vertexIndex];
            const key = `${edgeIndex}:${edgeT}`;
            let slot = concentrationSampleSlotByKey.get(key);
            if (slot === undefined) {
                slot = concentrationSampleEdgeIndices.length;
                concentrationSampleSlotByKey.set(key, slot);
                concentrationSampleEdgeIndices.push(edgeIndex);
                concentrationSampleEdgeTs.push(edgeT);
            }
            this._flowVertexConcentrationSampleSlot[vertexIndex] = slot;
        }
        this._flowConcentrationSampleEdgeIndex = Uint32Array.from(
            concentrationSampleEdgeIndices
        );
        this._flowConcentrationSampleEdgeT = Float32Array.from(
            concentrationSampleEdgeTs
        );
        this._flowConcentrationSampleValue = new Float32Array(
            concentrationSampleEdgeIndices.length
        );
        this._flowCellChains = flow.chains.map(chain => {
            const cells = [];
            for (const edgeIndex of chain) {
                const edge = this.system.flowNetwork.edges[edgeIndex];
                const cellOffset = this._flowCellOffset[edgeIndex];
                for (
                    let cellIndex = 0;
                    cellIndex < edge.cellCount;
                    cellIndex++
                ) {
                    cells.push(cellOffset + cellIndex);
                }
            }
            return cells;
        });
        this.flowContinuationChild = flow.continuationChild;
        this.flowChainCount = flow.chainCount;
        this.flowRingCount = flow.ringCount;
        this.flowTubeVertexCount = flow.tubeVertexCount;
        this.flowJunctionConnectorVertexCount =
            flow.connectorVertexCount;
        this.flowAxialSegmentCount = flow.axialSegmentCount;
        this.flowJunctionCount = flow.junctionCount;
        this.flowJunctionUnionDiagnostics =
            flow.junctionUnionDiagnostics;
        this.flowTopologyRadiusAnchors = flow.topologyRadiusAnchors;
        this.flowTubeIndexCount = flow.tubeIndexCount;
        this.flowJunctionTubeIndexCount =
            flow.junctionTubeIndexCount;
        this.flowJunctionConnectorIndexCount =
            flow.junctionConnectorIndexCount;
        this.flowSideOstiumTubeIndexCount =
            flow.sideOstiumTubeIndexCount;
        this.flowSideOstiumConnectorIndexCount =
            flow.sideOstiumConnectorIndexCount;
        this.flowJunctionIndexCount = flow.junctionIndexCount;
        this.flowTubeMaterial = createFlowMaterial();
        this.flowJunctionMaterial = createJunctionFlowMaterial();
        this.flowJunctionConnectorMaterial =
            createJunctionConnectorFlowMaterial();
        this.flowSideOstiumMaterial =
            createSideOstiumFlowMaterial();
        this.flowSideOstiumConnectorMaterial =
            createSideOstiumConnectorFlowMaterial();
        this.flowMesh = new THREE.Mesh(
            flow.geometry,
            [
                this.flowTubeMaterial,
                this.flowJunctionMaterial,
                this.flowJunctionConnectorMaterial,
                this.flowSideOstiumMaterial,
                this.flowSideOstiumConnectorMaterial
            ]
        );
        this.flowMesh.name = 'contrast-flow-lumen';
        this.flowMesh.frustumCulled = false;
        this.flowMesh.renderOrder = 6;
        this.flowMesh.visible = false;
    }

    _createLocalPlume() {
        const plume = createPlumeGeometry(
            this.system.localSolver.capacity
        );
        this._plumeArrays = plume.arrays;
        this.plumeMesh = new THREE.Mesh(
            plume.geometry,
            createPlumeMaterial()
        );
        this.plumeMesh.name = 'contrast-local-plume';
        this.plumeMesh.frustumCulled = false;
        this.plumeMesh.renderOrder = 6.2;
        this.plumeMesh.visible = false;
    }

    update() {
        const mediumConcentrationMgPerMm3 =
            this.system.medium.iodineMgPerMl / 1000;
        const edges = this.system.flowNetwork.edges;
        const solver = this.system.localSolver;
        this._flowCellLocalPlumeMassMg.fill(0);

        for (let index = 0; index < solver.count; index++) {
            const volumeMm3 = solver.iodineMassMg[index] /
                Math.max(1e-9, mediumConcentrationMgPerMm3);
            const physicalRadius = Math.cbrt(
                volumeMm3 * 3 / (4 * Math.PI)
            );
            const velocityX = solver.jetVelocityX[index];
            const velocityY = solver.jetVelocityY[index];
            const velocityZ = solver.jetVelocityZ[index];
            const speed = Math.hypot(velocityX, velocityY, velocityZ);
            const location =
                this.system.flowNetwork.findNearestLocationCoordinates(
                    solver.positionX[index],
                    solver.positionY[index],
                    solver.positionZ[index],
                    this._plumeLocationScratch
                );
            if (location.edgeIndex >= 0) {
                const localCellIndex =
                    this._flowCellOffset[location.edgeIndex] +
                    location.cellIndex;
                this._flowCellLocalPlumeMassMg[localCellIndex] +=
                    solver.iodineMassMg[index];
            }
            const vesselRadius = Math.max(
                physicalRadius,
                location.radius || physicalRadius
            );
            const mixingProgress = smoothstep(
                0.04,
                0.88,
                solver.travelDistanceMm[index] /
                    Math.max(1, solver.handoffDistanceMm[index])
            );
            const radialRadius = THREE.MathUtils.clamp(
                THREE.MathUtils.lerp(
                    physicalRadius * 1.08,
                    vesselRadius * 0.76,
                    mixingProgress
                ),
                0.46,
                Math.max(0.8, vesselRadius * 0.82)
            );
            const axialHalfLength = THREE.MathUtils.clamp(
                radialRadius * 1.18 + speed * 0.0026,
                radialRadius * 1.18,
                10
            );
            const projectedKernelArea = Math.max(
                1e-6,
                Math.PI * radialRadius * axialHalfLength
            );

            this._plumeArrays.center[index * 3] =
                solver.positionX[index];
            this._plumeArrays.center[index * 3 + 1] =
                solver.positionY[index];
            this._plumeArrays.center[index * 3 + 2] =
                solver.positionZ[index];
            this._plumeArrays.velocity[index * 3] = velocityX;
            this._plumeArrays.velocity[index * 3 + 1] = velocityY;
            this._plumeArrays.velocity[index * 3 + 2] = velocityZ;
            this._plumeArrays.radialRadius[index] = radialRadius;
            this._plumeArrays.axialHalfLength[index] = axialHalfLength;
            this._plumeArrays.concentration[index] = 1;
            // Volume divided by projected kernel area is the equivalent iodine
            // path length. It prevents overlap from turning discrete parcels
            // into saturated black beads.
            this._plumeArrays.opticalScale[index] =
                volumeMm3 / projectedKernelArea;
        }

        let activeFlowEdges = 0;
        for (let edgeIndex = 0; edgeIndex < edges.length; edgeIndex++) {
            const normalized = THREE.MathUtils.clamp(
                edges[edgeIndex].meanConcentrationMgPerMm3 /
                    Math.max(1e-9, mediumConcentrationMgPerMm3),
                0,
                1.5
            );
            this._flowRawConcentration[edgeIndex] = normalized;
        }
        this._smoothFlowTopologyAware(
            this._flowRawConcentration,
            this._flowSmoothConcentration
        );
        this._smoothFlowTopologyAware(
            this._flowSmoothConcentration,
            this._flowDisplayConcentration
        );
        for (let edgeIndex = 0; edgeIndex < edges.length; edgeIndex++) {
            const normalized = this._flowDisplayConcentration[edgeIndex];
            const detectability = smoothstep(
                FLOW_DETECTION_FLOOR,
                FLOW_DETECTION_FULL,
                normalized
            );
            this._flowDisplayConcentration[edgeIndex] =
                normalized * detectability;
        }

        for (let edgeIndex = 0; edgeIndex < edges.length; edgeIndex++) {
            const edge = edges[edgeIndex];
            const cellOffset = this._flowCellOffset[edgeIndex];
            for (let cellIndex = 0; cellIndex < edge.cellCount; cellIndex++) {
                const normalized = THREE.MathUtils.clamp(
                    (
                        edge.massMg[cellIndex] +
                        this._flowCellLocalPlumeMassMg[
                            cellOffset + cellIndex
                        ]
                    ) /
                        Math.max(1e-9, edge.volumes[cellIndex]) /
                        Math.max(1e-9, mediumConcentrationMgPerMm3),
                    0,
                    1.5
                );
                this._flowCellRawConcentration[cellOffset + cellIndex] =
                    normalized;
            }
        }
        this._smoothFlowCellsTopologyAware(
            this._flowCellRawConcentration,
            this._flowCellSmoothConcentration
        );
        this._smoothFlowCellsTopologyAware(
            this._flowCellSmoothConcentration,
            this._flowCellDisplayConcentration
        );
        this._equalizeRenderedJunctionBoundaryConcentrations(
            this._flowCellDisplayConcentration
        );
        this._closeInternalFlowChainGaps(
            this._flowCellDisplayConcentration
        );
        this._fadeDisconnectedColumnBranchRemnants(
            this._flowCellDisplayConcentration
        );
        this._enforceMonotonicPressureDrivenColumn(
            this._flowCellDisplayConcentration
        );
        for (let edgeIndex = 0; edgeIndex < edges.length; edgeIndex++) {
            const edge = edges[edgeIndex];
            const cellOffset = this._flowCellOffset[edgeIndex];
            let edgeIsActive = false;
            for (let cellIndex = 0; cellIndex < edge.cellCount; cellIndex++) {
                const flatIndex = cellOffset + cellIndex;
                const normalized =
                    this._flowCellDisplayConcentration[flatIndex];
                const detectability = smoothstep(
                    FLOW_DETECTION_FLOOR,
                    FLOW_DETECTION_FULL,
                    normalized
                );
                this._flowCellDisplayConcentration[flatIndex] =
                    normalized * detectability;
                edgeIsActive ||= detectability > 0;
            }
            if (edgeIsActive) activeFlowEdges++;
        }

        for (
            let slot = 0;
            slot < this._flowConcentrationSampleValue.length;
            slot++
        ) {
            this._flowConcentrationSampleValue[slot] =
                this._sampleFlowCellConcentration(
                    this._flowConcentrationSampleEdgeIndex[slot],
                    this._flowConcentrationSampleEdgeT[slot]
                );
        }
        for (
            let vertexIndex = 0;
            vertexIndex < this._flowVertexConcentration.length;
            vertexIndex++
        ) {
            this._flowVertexConcentration[vertexIndex] =
                this._flowConcentrationSampleValue[
                    this._flowVertexConcentrationSampleSlot[vertexIndex]
                ];
        }
        this._updateTrueJunctionConnectorConcentrations();
        this._updateJunctionOpticalWeights();
        this.flowMesh.geometry.attributes.flowConcentration.needsUpdate = true;
        this.flowMesh.geometry.attributes.flowOpticalWeight.needsUpdate =
            this._flowDynamicOpticalVertexIndices.length > 0;
        this.flowMesh.visible = activeFlowEdges > 0;

        this.plumeMesh.geometry.instanceCount = solver.count;
        for (const attributeName of [
            'instanceCenter',
            'instanceVelocity',
            'radialRadius',
            'axialHalfLength',
            'plumeConcentration',
            'opticalScale'
        ]) {
            this.plumeMesh.geometry.attributes[attributeName].needsUpdate =
                solver.count > 0;
        }
        this.plumeMesh.visible = solver.count > 0 && this._debugMode;
        this.group.visible = activeFlowEdges > 0 || solver.count > 0;
        return {
            activeFlowEdges,
            flowChainCount: this.flowChainCount,
            flowRingCount: this.flowRingCount,
            particleCount: solver.count,
            visible: this.group.visible
        };
    }

    _updateJunctionOpticalWeights() {
        if (!this._flowDynamicOpticalVertexIndices.length) return;
        for (
            let slot = 0;
            slot < this._flowJunctionDynamicProfiles.length;
            slot++
        ) {
            const profile = this._flowJunctionDynamicProfiles[slot];
            let strongestOpticalDepth = 0;
            let summedOpticalDepthSquared = 0;
            for (const arm of profile.arms) {
                let concentration = 0;
                for (const sample of arm.samples) {
                    concentration = Math.max(
                        concentration,
                        this._sampleFlowCellConcentration(
                            sample.edgeIndex,
                            sample.edgeT
                        )
                    );
                }
                const opticalDepth = concentration * arm.radius;
                strongestOpticalDepth = Math.max(
                    strongestOpticalDepth,
                    opticalDepth
                );
                summedOpticalDepthSquared +=
                    opticalDepth * opticalDepth;
            }
            this._flowJunctionDynamicWeight[slot] =
                summedOpticalDepthSquared > 1e-12
                    ? THREE.MathUtils.clamp(
                        strongestOpticalDepth /
                            Math.sqrt(summedOpticalDepthSquared),
                        0,
                        1
                    )
                    : profile.staticOpticalWeight;
        }

        for (const vertexIndex of this._flowDynamicOpticalVertexIndices) {
            const slot =
                this._flowVertexJunctionOpticalSlot[vertexIndex];
            const connectorBlend =
                this._flowVertexJunctionConnectorBlend[vertexIndex];
            const overlapBlend =
                this._flowVertexJunctionOpticalBlend[vertexIndex];
            const dynamicOverlapWeight = overlapBlend > 0
                ? THREE.MathUtils.lerp(
                    1,
                    this._flowJunctionDynamicWeight[slot],
                    overlapBlend
                )
                : 1;
            this._flowVertexOpticalWeight[vertexIndex] =
                dynamicOverlapWeight * Math.sqrt(
                    Math.max(
                        0,
                        1 - smoothstep(0.5, 1, connectorBlend)
                    )
                );
        }

    }

    _updateTrueJunctionConnectorConcentrations() {
        for (
            let slot = 0;
            slot < this._flowJunctionDynamicProfiles.length;
            slot++
        ) {
            const profile = this._flowJunctionDynamicProfiles[slot];
            if (profile.hasGeometricMainContinuation) continue;
            let maximumConcentration = 0;
            for (const arm of profile.arms) {
                for (const sample of arm.samples) {
                    maximumConcentration = Math.max(
                        maximumConcentration,
                        this._sampleFlowCellConcentration(
                            sample.edgeIndex,
                            sample.edgeT
                        )
                    );
                }
            }
            this._flowTrueJunctionConnectorConcentration[slot] =
                maximumConcentration;
        }
        for (
            const vertexIndex of
                this._flowTrueJunctionConnectorVertexIndices
        ) {
            const slot =
                this._flowVertexJunctionOpticalSlot[vertexIndex];
            this._flowVertexConcentration[vertexIndex] = Math.max(
                this._flowVertexConcentration[vertexIndex],
                this._flowTrueJunctionConnectorConcentration[slot]
            );
        }
    }

    _smoothFlowTopologyAware(source, target) {
        const network = this.system.flowNetwork;
        for (let edgeIndex = 0; edgeIndex < network.edges.length; edgeIndex++) {
            const edge = network.edges[edgeIndex];
            const localConcentration = source[edgeIndex];

            const upstreamConcentration = edge.parentEdgeIndex >= 0
                ? source[edge.parentEdgeIndex]
                : 0;
            let downstreamConcentration = 0;
            if (edge.childEdgeIndices.length) {
                let downstreamFlow = 0;
                for (const childIndex of edge.childEdgeIndices) {
                    const child = network.edges[childIndex];
                    const flow = Math.max(0, child.meanFlowMm3PerS);
                    downstreamConcentration += source[childIndex] * flow;
                    downstreamFlow += flow;
                }
                if (downstreamFlow > 0) {
                    downstreamConcentration /= downstreamFlow;
                } else {
                    downstreamConcentration = edge.childEdgeIndices.reduce(
                        (sum, childIndex) => sum + source[childIndex],
                        0
                    ) / edge.childEdgeIndices.length;
                }
            }

            // Only cells that already contain detectable iodine are adjusted.
            // Empty side branches therefore remain empty, while an isolated
            // late segment is blended into its clear upstream/downstream
            // neighbours instead of looking like a stationary rectangular
            // stain.
            target[edgeIndex] = blendContinuousFlowConcentration(
                localConcentration,
                upstreamConcentration,
                downstreamConcentration
            );
        }
    }

    _smoothFlowCellsTopologyAware(source, target) {
        const edges = this.system.flowNetwork.edges;
        for (let edgeIndex = 0; edgeIndex < edges.length; edgeIndex++) {
            const edge = edges[edgeIndex];
            const cellOffset = this._flowCellOffset[edgeIndex];
            for (let cellIndex = 0; cellIndex < edge.cellCount; cellIndex++) {
                const flatIndex = cellOffset + cellIndex;
                const localConcentration = source[flatIndex];

                let upstreamConcentration = 0;
                if (cellIndex > 0) {
                    upstreamConcentration = source[flatIndex - 1];
                } else if (edge.parentEdgeIndex >= 0) {
                    const parent = edges[edge.parentEdgeIndex];
                    upstreamConcentration = source[
                        this._flowCellOffset[parent.index] +
                        parent.cellCount - 1
                    ];
                }

                let downstreamConcentration = 0;
                if (cellIndex + 1 < edge.cellCount) {
                    downstreamConcentration = source[flatIndex + 1];
                } else if (edge.childEdgeIndices.length) {
                    let downstreamFlow = 0;
                    for (const childIndex of edge.childEdgeIndices) {
                        const child = edges[childIndex];
                        const flow = Math.max(0, child.meanFlowMm3PerS);
                        downstreamConcentration += source[
                            this._flowCellOffset[childIndex]
                        ] * flow;
                        downstreamFlow += flow;
                    }
                    if (downstreamFlow > 0) {
                        downstreamConcentration /= downstreamFlow;
                    } else {
                        downstreamConcentration =
                            edge.childEdgeIndices.reduce(
                                (sum, childIndex) => sum + source[
                                    this._flowCellOffset[childIndex]
                                ],
                                0
                            ) / edge.childEdgeIndices.length;
                    }
                }

                target[flatIndex] = blendContinuousFlowConcentration(
                    localConcentration,
                    upstreamConcentration,
                    downstreamConcentration
                );
            }
        }
    }

    _equalizeRenderedJunctionBoundaryConcentrations(concentrations) {
        const edges = this.system.flowNetwork.edges;
        for (const profile of this._flowJunctionDynamicProfiles) {
            const boundarySamples = [];
            const seenEdges = new Set();
            for (const arm of profile.surfaceArms) {
                if (seenEdges.has(arm.edgeIndex)) continue;
                seenEdges.add(arm.edgeIndex);
                const edge = edges[arm.edgeIndex];
                if (!edge || edge.cellCount < 1) continue;
                const cellIndex = arm.edgeT < 0.5
                    ? 0
                    : edge.cellCount - 1;
                boundarySamples.push({
                    flatIndex:
                        this._flowCellOffset[arm.edgeIndex] + cellIndex,
                    weight: Math.max(0.04, arm.radius * arm.radius)
                });
            }
            if (boundarySamples.length < 2) continue;
            // Mix only arms which already contain detectable iodine. A small
            // empty side branch must stay empty, but it must not prevent the
            // opacified parent and continuation from sharing one boundary
            // value; otherwise the union becomes a dark circumferential band.
            const visibleBoundarySamples = boundarySamples.filter(
                sample =>
                    concentrations[sample.flatIndex] >
                    FLOW_DETECTION_FLOOR
            );
            if (visibleBoundarySamples.length < 2) continue;
            let weightedConcentration = 0;
            let totalWeight = 0;
            for (const sample of visibleBoundarySamples) {
                weightedConcentration +=
                    concentrations[sample.flatIndex] * sample.weight;
                totalWeight += sample.weight;
            }
            const mixedConcentration = weightedConcentration /
                Math.max(1e-9, totalWeight);
            for (const sample of visibleBoundarySamples) {
                concentrations[sample.flatIndex] = mixedConcentration;
            }
        }
    }

    _closeInternalFlowChainGaps(concentrations) {
        for (const chainCells of this._flowCellChains) {
            let previousVisibleIndex = -1;
            let previousVisibleConcentration = 0;
            for (
                let chainIndex = 0;
                chainIndex < chainCells.length;
                chainIndex++
            ) {
                const flatIndex = chainCells[chainIndex];
                const concentration = concentrations[flatIndex];
                if (!(concentration > FLOW_DETECTION_FLOOR)) continue;

                if (
                    previousVisibleIndex >= 0 &&
                    chainIndex > previousVisibleIndex + 1
                ) {
                    const gapLength =
                        chainIndex - previousVisibleIndex;
                    for (
                        let gapIndex = previousVisibleIndex + 1;
                        gapIndex < chainIndex;
                        gapIndex++
                    ) {
                        const interpolation =
                            (gapIndex - previousVisibleIndex) /
                            gapLength;
                        const supportedConcentration = THREE.MathUtils.lerp(
                            previousVisibleConcentration,
                            concentration,
                            interpolation
                        ) * FLOW_INTERNAL_GAP_FILL_FACTOR;
                        const gapFlatIndex = chainCells[gapIndex];
                        concentrations[gapFlatIndex] = Math.max(
                            concentrations[gapFlatIndex],
                            supportedConcentration
                        );
                    }
                }

                previousVisibleIndex = chainIndex;
                previousVisibleConcentration = concentration;
            }
        }
    }

    _fadeDisconnectedColumnBranchRemnants(concentrations) {
        const column = this.system.pressureDrivenRetrogradeColumn;
        const pressureDrivenColumnActive = !!column?.isFlowReversed;
        const nonColumnInjectionActive =
            !!this.system.injection && !pressureDrivenColumnActive;
        if (pressureDrivenColumnActive) {
            this._pressureDrivenColumnEndTime = null;
        } else if (nonColumnInjectionActive) {
            // A completed pressure-driven path mask must never be reused for
            // a later injection that does not currently reverse flow.
            this._pressureDrivenColumnEndTime = null;
        } else if (
            this._wasPressureDrivenColumnActive &&
            column?.activePath
        ) {
            this._pressureDrivenColumnEndTime =
                this.system.flowNetwork.time;
        }
        this._wasPressureDrivenColumnActive =
            pressureDrivenColumnActive;

        if (
            pressureDrivenColumnActive ||
            this._pressureDrivenColumnEndTime === null ||
            !column?.activePath?.cells?.length
        ) {
            this.lastDisconnectedBranchFade = 1;
            return;
        }

        const elapsedSeconds = Math.max(
            0,
            this.system.flowNetwork.time -
                this._pressureDrivenColumnEndTime
        );
        if (
            elapsedSeconds >
                COLUMN_BRANCH_FADE_END_SECONDS + 0.55
        ) {
            this._pressureDrivenColumnEndTime = null;
            this.lastDisconnectedBranchFade = 1;
            return;
        }
        const fade = 1 - smoothstep(
            COLUMN_BRANCH_FADE_DELAY_SECONDS,
            COLUMN_BRANCH_FADE_END_SECONDS,
            elapsedSeconds
        );
        this.lastDisconnectedBranchFade = fade;
        if (fade >= 0.9999) return;

        const pathMask = this._flowCellOnPressureDrivenPath;
        pathMask.fill(0);
        for (const { edgeIndex, cellIndex } of column.activePath.cells) {
            pathMask[
                this._flowCellOffset[edgeIndex] + cellIndex
            ] = 1;
        }

        for (
            let flatIndex = 0;
            flatIndex < concentrations.length;
            flatIndex++
        ) {
            if (!pathMask[flatIndex]) {
                concentrations[flatIndex] *= fade;
            }
        }
    }

    _enforceMonotonicPressureDrivenColumn(concentrations) {
        const column = this.system.pressureDrivenRetrogradeColumn;
        if (!column?.activePath?.cells?.length) return;
        const pressureDrivenColumnActive = !!column.isFlowReversed;
        const elapsedSinceInjection =
            this._pressureDrivenColumnEndTime === null
            ? 0
            : this.system.flowNetwork.time -
                this._pressureDrivenColumnEndTime;
        if (
            !pressureDrivenColumnActive &&
            (
                this._pressureDrivenColumnEndTime === null ||
                elapsedSinceInjection >
                    COLUMN_BRANCH_FADE_END_SECONDS + 0.55
            )
        ) return;

        let previousConcentration = Infinity;
        for (const { edgeIndex, cellIndex } of column.activePath.cells) {
            const flatIndex =
                this._flowCellOffset[edgeIndex] + cellIndex;
            concentrations[flatIndex] = Math.min(
                concentrations[flatIndex],
                previousConcentration
            );
            previousConcentration = concentrations[flatIndex];
        }
    }

    _sampleFlowCellConcentration(edgeIndex, edgeT) {
        const edge = this.system.flowNetwork.edges[edgeIndex];
        const cellOffset = this._flowCellOffset[edgeIndex];
        const samplePosition = edgeT * edge.cellCount - 0.5;
        if (samplePosition <= 0) {
            return this._flowCellDisplayConcentration[cellOffset];
        }
        if (samplePosition >= edge.cellCount - 1) {
            return this._flowCellDisplayConcentration[
                cellOffset + edge.cellCount - 1
            ];
        }
        const lowerCell = Math.max(
            0,
            Math.min(edge.cellCount - 1, Math.floor(samplePosition))
        );
        const upperCell = Math.max(
            0,
            Math.min(edge.cellCount - 1, lowerCell + 1)
        );
        const interpolation = THREE.MathUtils.clamp(
            samplePosition - lowerCell,
            0,
            1
        );
        return THREE.MathUtils.lerp(
            this._flowCellDisplayConcentration[cellOffset + lowerCell],
            this._flowCellDisplayConcentration[cellOffset + upperCell],
            interpolation
        );
    }

    setDebugMode(enabled) {
        this._debugMode = !!enabled;
        this.flowTubeMaterial.uniforms.debugMode.value = this._debugMode;
        this.flowJunctionMaterial.uniforms.debugMode.value = this._debugMode;
        this.flowJunctionConnectorMaterial.uniforms.debugMode.value =
            this._debugMode;
        this.flowSideOstiumMaterial.uniforms.debugMode.value =
            this._debugMode;
        this.flowSideOstiumConnectorMaterial.uniforms.debugMode.value =
            this._debugMode;
        this.plumeMesh.material.uniforms.debugMode.value = this._debugMode;
        this.plumeMesh.visible =
            this.system.localSolver.count > 0 && this._debugMode;
    }

    dispose() {
        this.flowMesh.geometry.dispose();
        this.flowTubeMaterial.dispose();
        this.flowJunctionMaterial.dispose();
        this.flowJunctionConnectorMaterial.dispose();
        this.flowSideOstiumMaterial.dispose();
        this.flowSideOstiumConnectorMaterial.dispose();
        this.plumeMesh.geometry.dispose();
        this.plumeMesh.material.dispose();
        this.group.clear();
    }
}
