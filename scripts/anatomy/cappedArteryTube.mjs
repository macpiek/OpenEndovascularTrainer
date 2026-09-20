// Offline closed solid used to repair the existing extension junctions.
import * as THREE from 'three';
const RADIAL_SEGMENTS=32,AXIAL_SAMPLE_SPACING_MM=1.5;
export function taperedTubeGeometry(pathDefinition) {
    const controlPoints = pathDefinition.points.map(point => point.position);
    const curve = new THREE.CatmullRomCurve3(
        controlPoints,
        false,
        'centripetal',
        0.5
    );
    const controlDistances = [0];
    for (let index = 1; index < controlPoints.length; index++) {
        controlDistances.push(
            controlDistances[index - 1] +
            controlPoints[index].distanceTo(controlPoints[index - 1])
        );
    }
    const controlLength = controlDistances.at(-1);
    const tubularSegments = Math.max(
        12,
        Math.ceil(curve.getLength() / AXIAL_SAMPLE_SPACING_MM)
    );
    const frames = curve.computeFrenetFrames(tubularSegments, false);
    const ringSize = RADIAL_SEGMENTS;
    const ringCount = tubularSegments + 1;
    const capStartIndex = ringCount * ringSize;
    const capEndIndex = capStartIndex + 1;
    const positions = new Float32Array((capEndIndex + 1) * 3);
    const point = new THREE.Vector3();
    let radiusCursor = 0;

    for (let ring = 0; ring < ringCount; ring++) {
        const u = ring / tubularSegments;
        curve.getPointAt(u, point);
        const distance = u * controlLength;
        while (
            radiusCursor + 1 < controlDistances.length - 1 &&
            controlDistances[radiusCursor + 1] < distance
        ) radiusCursor++;
        const fromDistance = controlDistances[radiusCursor];
        const toDistance = controlDistances[radiusCursor + 1];
        const radiusT = THREE.MathUtils.clamp(
            (distance - fromDistance) /
                Math.max(1e-6, toDistance - fromDistance),
            0,
            1
        );
        const radius = THREE.MathUtils.lerp(
            pathDefinition.points[radiusCursor].radius,
            pathDefinition.points[radiusCursor + 1].radius,
            radiusT
        );
        const normal = frames.normals[ring];
        const binormal = frames.binormals[ring];
        for (let radial = 0; radial < RADIAL_SEGMENTS; radial++) {
            const angle = radial / RADIAL_SEGMENTS * Math.PI * 2;
            const cos = Math.cos(angle);
            const sin = Math.sin(angle);
            const vertexIndex = (ring * ringSize + radial) * 3;
            positions[vertexIndex] = point.x + radius * (
                normal.x * cos + binormal.x * sin
            );
            positions[vertexIndex + 1] = point.y + radius * (
                normal.y * cos + binormal.y * sin
            );
            positions[vertexIndex + 2] = point.z + radius * (
                normal.z * cos + binormal.z * sin
            );
        }
    }
    positions.set(controlPoints[0].toArray(), capStartIndex * 3);
    positions.set(controlPoints.at(-1).toArray(), capEndIndex * 3);

    const indices = [];
    for (let ring = 0; ring < tubularSegments; ring++) {
        for (let radial = 0; radial < RADIAL_SEGMENTS; radial++) {
            const next = (radial + 1) % RADIAL_SEGMENTS;
            const a = ring * ringSize + radial;
            const b = (ring + 1) * ringSize + radial;
            const c = (ring + 1) * ringSize + next;
            const d = ring * ringSize + next;
            indices.push(a, b, d, b, c, d);
        }
    }
    for (let radial = 0; radial < RADIAL_SEGMENTS; radial++) {
        const next = (radial + 1) % RADIAL_SEGMENTS;
        indices.push(capStartIndex, radial, next);
        const lastRing = tubularSegments * ringSize;
        indices.push(capEndIndex, lastRing + next, lastRing + radial);
    }
    // The ring construction above follows the curve frame clockwise. Flip the
    // complete shell so Manifold receives outward-facing triangles; an inward
    // shell represents the complement and would subtract lumen during union.
    for (let index = 0; index < indices.length; index += 3) {
        const swap = indices[index + 1];
        indices[index + 1] = indices[index + 2];
        indices[index + 2] = swap;
    }

    const geometry = new THREE.BufferGeometry();
    geometry.setAttribute('position', new THREE.BufferAttribute(positions, 3));
    geometry.setIndex(indices);
    geometry.computeVertexNormals();
    geometry.userData.extendedArtery = pathDefinition.name;
    return geometry;
}

