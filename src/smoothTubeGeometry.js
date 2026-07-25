import * as THREE from 'three';

const MIN_TUBULAR_SEGMENTS = 8;

export function createSmoothTubeGeometry(points, {
    radius,
    samplesPerSegment = 3,
    radialSegments = 12,
    maxTubularSegments = 900
} = {}) {
    if (!Array.isArray(points) || points.length < 2 || !(radius > 0)) {
        return new THREE.BufferGeometry();
    }

    const path = points.length === 2
        ? new THREE.LineCurve3(points[0], points[1])
        : new THREE.CatmullRomCurve3(points, false, 'centripetal', 0.5);
    const tubularSegments = Math.min(
        maxTubularSegments,
        Math.max(MIN_TUBULAR_SEGMENTS, Math.ceil((points.length - 1) * samplesPerSegment))
    );
    const geometry = new THREE.TubeGeometry(
        path,
        tubularSegments,
        radius,
        radialSegments,
        false
    );
    geometry.userData.smoothTube = {
        sourcePointCount: points.length,
        tubularSegments,
        radialSegments
    };
    return geometry;
}
