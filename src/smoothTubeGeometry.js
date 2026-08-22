import * as THREE from 'three';

const MIN_TUBULAR_SEGMENTS = 8;
const DEFAULT_ARC_LENGTH_DIVISIONS = 200;
const smoothTubeStates = new WeakMap();

function clamp(value, minimum, maximum) {
    return Math.max(minimum, Math.min(maximum, value));
}

function normalizedPointCount(points, requestedPointCount) {
    if (!Array.isArray(points)) return 0;
    const pointCount = requestedPointCount ?? points.length;
    if (!Number.isFinite(pointCount)) return 0;
    return Math.max(0, Math.min(points.length, Math.floor(pointCount)));
}

function resolveTopology(pointCount, samplesPerSegment, radialSegments, maxTubularSegments) {
    return {
        tubularSegments: Math.min(
            maxTubularSegments,
            Math.max(
                MIN_TUBULAR_SEGMENTS,
                Math.ceil((pointCount - 1) * samplesPerSegment)
            )
        ),
        radialSegments: Math.max(3, Math.floor(radialSegments))
    };
}

function createVectorArray(length) {
    return Array.from({ length }, () => new THREE.Vector3());
}

function createPath(state, points, pointCount) {
    state.pathPoints.length = pointCount;
    for (let index = 0; index < pointCount; index++) {
        state.pathPoints[index] = points[index];
    }

    if (pointCount === 2) {
        if (!state.path?.isLineCurve3) {
            state.path = new THREE.LineCurve3(points[0], points[1]);
        } else {
            state.path.v1 = points[0];
            state.path.v2 = points[1];
        }
    } else if (!state.path?.isCatmullRomCurve3) {
        state.path = new THREE.CatmullRomCurve3(
            state.pathPoints,
            false,
            'centripetal',
            0.5
        );
    } else {
        state.path.points = state.pathPoints;
    }
    return state.path;
}

function refreshArcLengths(state) {
    const path = state.path;
    if (!path?.isCatmullRomCurve3) return;

    const divisions = path.arcLengthDivisions || DEFAULT_ARC_LENGTH_DIVISIONS;
    if (!state.arcLengths || state.arcLengths.length !== divisions + 1) {
        state.arcLengths = new Float64Array(divisions + 1);
    }
    const arcLengths = state.arcLengths;
    let last = state.arcPointA;
    let current = state.arcPointB;
    path.getPoint(0, last);
    arcLengths[0] = 0;
    let sum = 0;
    for (let division = 1; division <= divisions; division++) {
        path.getPoint(division / divisions, current);
        sum += current.distanceTo(last);
        arcLengths[division] = sum;
        const swap = last;
        last = current;
        current = swap;
    }
    state.arcPointA = last;
    state.arcPointB = current;
    path.cacheArcLengths = arcLengths;
    path.needsUpdate = false;
}

function sampleTangent(state, u, target) {
    const path = state.path;
    if (path.isLineCurve3) {
        return target.subVectors(path.v2, path.v1).normalize();
    }

    const t = path.getUtoTmapping(u);
    const delta = 0.0001;
    const t1 = Math.max(0, t - delta);
    const t2 = Math.min(1, t + delta);
    path.getPoint(t1, state.tangentPointA);
    path.getPoint(t2, state.tangentPointB);
    return target
        .copy(state.tangentPointB)
        .sub(state.tangentPointA)
        .normalize();
}

function computeFrenetFrames(state) {
    const { tubularSegments, tangents, frameNormals, binormals } = state;
    const { initialNormal, frameAxis, frameRotation } = state;

    for (let index = 0; index <= tubularSegments; index++) {
        sampleTangent(state, index / tubularSegments, tangents[index]);
    }

    let minimum = Number.MAX_VALUE;
    const tx = Math.abs(tangents[0].x);
    const ty = Math.abs(tangents[0].y);
    const tz = Math.abs(tangents[0].z);
    if (tx <= minimum) {
        minimum = tx;
        initialNormal.set(1, 0, 0);
    }
    if (ty <= minimum) {
        minimum = ty;
        initialNormal.set(0, 1, 0);
    }
    if (tz <= minimum) initialNormal.set(0, 0, 1);

    frameAxis.crossVectors(tangents[0], initialNormal).normalize();
    frameNormals[0].crossVectors(tangents[0], frameAxis);
    binormals[0].crossVectors(tangents[0], frameNormals[0]);

    for (let index = 1; index <= tubularSegments; index++) {
        frameNormals[index].copy(frameNormals[index - 1]);
        binormals[index].copy(binormals[index - 1]);
        frameAxis.crossVectors(tangents[index - 1], tangents[index]);
        if (frameAxis.length() > Number.EPSILON) {
            frameAxis.normalize();
            const theta = Math.acos(clamp(
                tangents[index - 1].dot(tangents[index]),
                -1,
                1
            ));
            frameNormals[index].applyMatrix4(
                frameRotation.makeRotationAxis(frameAxis, theta)
            );
        }
        binormals[index].crossVectors(tangents[index], frameNormals[index]);
    }
}

function fillSurface(state, radius) {
    const { geometry, path, tubularSegments, radialSegments } = state;
    const positions = geometry.getAttribute('position').array;
    const normals = geometry.getAttribute('normal').array;
    const point = state.surfacePoint;
    const normal = state.surfaceNormal;
    let offset = 0;

    for (let ring = 0; ring <= tubularSegments; ring++) {
        path.getPointAt(ring / tubularSegments, point);
        const frameNormal = state.frameNormals[ring];
        const binormal = state.binormals[ring];
        for (let radial = 0; radial <= radialSegments; radial++) {
            const angle = radial / radialSegments * Math.PI * 2;
            const sin = Math.sin(angle);
            const cos = -Math.cos(angle);
            normal.set(
                cos * frameNormal.x + sin * binormal.x,
                cos * frameNormal.y + sin * binormal.y,
                cos * frameNormal.z + sin * binormal.z
            ).normalize();
            normals[offset] = normal.x;
            positions[offset++] = point.x + radius * normal.x;
            normals[offset] = normal.y;
            positions[offset++] = point.y + radius * normal.y;
            normals[offset] = normal.z;
            positions[offset++] = point.z + radius * normal.z;
        }
    }

    geometry.getAttribute('position').needsUpdate = true;
    geometry.getAttribute('normal').needsUpdate = true;
    if (geometry.boundingBox !== null) geometry.computeBoundingBox();
    if (geometry.boundingSphere !== null) geometry.computeBoundingSphere();
}

function createTopology(tubularSegments, radialSegments) {
    const geometry = new THREE.BufferGeometry();
    geometry.type = 'TubeGeometry';
    const ringSize = radialSegments + 1;
    const vertexCount = (tubularSegments + 1) * ringSize;
    const positions = new Float32Array(vertexCount * 3);
    const normals = new Float32Array(vertexCount * 3);
    const uvs = new Float32Array(vertexCount * 2);
    const indexCount = tubularSegments * radialSegments * 6;
    const IndexArray = vertexCount - 1 >= 65535 ? Uint32Array : Uint16Array;
    const indices = new IndexArray(indexCount);

    let uvOffset = 0;
    for (let ring = 0; ring <= tubularSegments; ring++) {
        for (let radial = 0; radial <= radialSegments; radial++) {
            uvs[uvOffset++] = ring / tubularSegments;
            uvs[uvOffset++] = radial / radialSegments;
        }
    }

    let indexOffset = 0;
    for (let ring = 1; ring <= tubularSegments; ring++) {
        for (let radial = 1; radial <= radialSegments; radial++) {
            const a = ringSize * (ring - 1) + radial - 1;
            const b = ringSize * ring + radial - 1;
            const c = ringSize * ring + radial;
            const d = ringSize * (ring - 1) + radial;
            indices[indexOffset++] = a;
            indices[indexOffset++] = b;
            indices[indexOffset++] = d;
            indices[indexOffset++] = b;
            indices[indexOffset++] = c;
            indices[indexOffset++] = d;
        }
    }

    geometry.setIndex(new THREE.BufferAttribute(indices, 1));
    geometry.setAttribute(
        'position',
        new THREE.BufferAttribute(positions, 3).setUsage(THREE.DynamicDrawUsage)
    );
    geometry.setAttribute(
        'normal',
        new THREE.BufferAttribute(normals, 3).setUsage(THREE.DynamicDrawUsage)
    );
    geometry.setAttribute('uv', new THREE.BufferAttribute(uvs, 2));
    return geometry;
}

function createState(geometry, tubularSegments, radialSegments) {
    return {
        geometry,
        path: null,
        pathPoints: [],
        tubularSegments,
        radialSegments,
        tangents: createVectorArray(tubularSegments + 1),
        frameNormals: createVectorArray(tubularSegments + 1),
        binormals: createVectorArray(tubularSegments + 1),
        arcLengths: null,
        arcPointA: new THREE.Vector3(),
        arcPointB: new THREE.Vector3(),
        tangentPointA: new THREE.Vector3(),
        tangentPointB: new THREE.Vector3(),
        initialNormal: new THREE.Vector3(),
        frameAxis: new THREE.Vector3(),
        frameRotation: new THREE.Matrix4(),
        surfacePoint: new THREE.Vector3(),
        surfaceNormal: new THREE.Vector3()
    };
}

/**
 * Updates a smooth open tube in place. A new BufferGeometry is returned only
 * when its vertex/index topology changes; point motion and radius changes
 * reuse the existing CPU arrays and WebGL buffers.
 */
export function updateSmoothTubeGeometry(existingGeometry, points, {
    radius,
    pointCount: requestedPointCount,
    samplesPerSegment = 3,
    radialSegments = 12,
    maxTubularSegments = 900
} = {}) {
    const pointCount = normalizedPointCount(points, requestedPointCount);
    if (pointCount < 2 || !(radius > 0)) {
        return new THREE.BufferGeometry();
    }

    const topology = resolveTopology(
        pointCount,
        samplesPerSegment,
        radialSegments,
        maxTubularSegments
    );
    let geometry = existingGeometry;
    let state = smoothTubeStates.get(geometry);
    if (
        !state ||
        state.tubularSegments !== topology.tubularSegments ||
        state.radialSegments !== topology.radialSegments
    ) {
        geometry = createTopology(
            topology.tubularSegments,
            topology.radialSegments
        );
        state = createState(
            geometry,
            topology.tubularSegments,
            topology.radialSegments
        );
        smoothTubeStates.set(geometry, state);
    }

    createPath(state, points, pointCount);
    refreshArcLengths(state);
    computeFrenetFrames(state);
    fillSurface(state, radius);

    const parameters = geometry.parameters ??= {};
    parameters.path = state.path;
    parameters.tubularSegments = state.tubularSegments;
    parameters.radius = radius;
    parameters.radialSegments = state.radialSegments;
    parameters.closed = false;
    geometry.tangents = state.tangents;
    geometry.normals = state.frameNormals;
    geometry.binormals = state.binormals;
    const metadata = geometry.userData.smoothTube ??= {};
    metadata.sourcePointCount = pointCount;
    metadata.tubularSegments = state.tubularSegments;
    metadata.radialSegments = state.radialSegments;
    return geometry;
}

export function createSmoothTubeGeometry(points, options = {}) {
    return updateSmoothTubeGeometry(null, points, options);
}
