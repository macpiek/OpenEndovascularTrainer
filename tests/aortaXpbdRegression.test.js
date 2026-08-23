import assert from 'node:assert/strict';
import fs from 'node:fs';
import * as THREE from 'three';
import { STLLoader } from 'three/examples/jsm/loaders/STLLoader.js';
import { MeshBVH } from 'three-mesh-bvh';
import { transformAortaGeometry } from '../src/aortaTransform.js';
import { decodeCollisionAsset } from '../src/physics/collision/collisionAssetFormat.js';
import { createContactResult, VesselContactField } from '../src/physics/collision/vesselContactField.js';
import {
    DEFAULT_TOOL_PROFILES,
    EndovascularPhysicsWorld
} from '../src/physics/endovascularPhysicsWorld.js';
import { applyKirchhoffMaterialProfile } from '../src/physics/applyKirchhoffMaterialProfile.js';
import {
    buildContainedGuidewireRenderPolyline,
    spatiallyCapturedContainmentEnd
} from '../src/physics/catheterGuidewireCoupling.js';
import { ElasticRod } from '../src/physics/elasticRod.js';
import { GuidewireSolver } from '../src/physics/guidewireSolver.js';
import { applyGuidewireMaterialProfile } from '../src/physics/guidewireMaterialProfile.js';
import { PIGTAIL_NATURAL_ARC_LENGTH_MM } from '../src/physics/catheterMaterialProfile.js';
import { PigtailCatheter } from '../src/pigtailCatheter.js';
import {
    GUIDEWIRE_RENDER_RADIUS_MM,
    PIGTAIL_CATHETER_INNER_RADIUS_MM,
    PIGTAIL_CATHETER_RENDER_RADIUS_MM
} from '../src/toolDimensions.js';
import { generateVessel } from '../src/vesselGeometry.js';

function arrayBufferFromBuffer(buffer) {
    return buffer.buffer.slice(buffer.byteOffset, buffer.byteOffset + buffer.byteLength);
}

function buildTree(asset, rootPoint) {
    const segments = asset.arrays.centerlineSegments;
    const edges = asset.arrays.centerlineEdges;
    const stride = asset.metadata.centerline.stride;
    const nodeCount = asset.metadata.centerline.nodeCount;
    const positions = new Float32Array(nodeCount * 3);
    const radii = new Float32Array(nodeCount);
    const adjacency = Array.from({ length: nodeCount }, () => []);
    const degree = new Uint16Array(nodeCount);

    for (let segmentId = 0; segmentId < edges.length / 2; segmentId++) {
        const nodeA = edges[segmentId * 2];
        const nodeB = edges[segmentId * 2 + 1];
        const offset = segmentId * stride;
        positions.set(segments.subarray(offset, offset + 3), nodeA * 3);
        positions.set(segments.subarray(offset + 3, offset + 6), nodeB * 3);
        radii[nodeA] = Math.max(radii[nodeA], segments[offset + 6]);
        radii[nodeB] = Math.max(radii[nodeB], segments[offset + 7]);
        const length = Math.hypot(
            segments[offset + 3] - segments[offset],
            segments[offset + 4] - segments[offset + 1],
            segments[offset + 5] - segments[offset + 2]
        );
        adjacency[nodeA].push({ node: nodeB, length });
        adjacency[nodeB].push({ node: nodeA, length });
        degree[nodeA]++;
        degree[nodeB]++;
    }

    let root = 0;
    let rootDistance = Infinity;
    for (let node = 0; node < nodeCount; node++) {
        const offset = node * 3;
        const distance = Math.hypot(
            positions[offset] - rootPoint.x,
            positions[offset + 1] - rootPoint.y,
            positions[offset + 2] - rootPoint.z
        );
        if (distance < rootDistance) {
            rootDistance = distance;
            root = node;
        }
    }

    const parents = new Int32Array(nodeCount);
    parents.fill(-2);
    parents[root] = -1;
    const distances = new Float32Array(nodeCount);
    const stack = [root];
    while (stack.length) {
        const node = stack.pop();
        for (const edge of adjacency[node]) {
            if (parents[edge.node] !== -2) continue;
            parents[edge.node] = node;
            distances[edge.node] = distances[node] + edge.length;
            stack.push(edge.node);
        }
    }
    const leaves = [];
    for (let node = 0; node < nodeCount; node++) {
        if (degree[node] === 1 && node !== root) leaves.push(node);
    }
    return { positions, radii, parents, distances, leaves, root, rootDistance };
}

function pathToRoot(tree, leaf) {
    const nodes = [];
    let node = leaf;
    while (node >= 0) {
        nodes.push(node);
        node = tree.parents[node];
    }
    nodes.reverse();
    return nodes.map(nodeId => {
        const offset = nodeId * 3;
        return new THREE.Vector3(
            tree.positions[offset],
            tree.positions[offset + 1],
            tree.positions[offset + 2]
        );
    });
}

function resamplePath(points, spacing) {
    const cumulative = new Float32Array(points.length);
    for (let index = 1; index < points.length; index++) {
        cumulative[index] = cumulative[index - 1] + points[index].distanceTo(points[index - 1]);
    }
    const total = cumulative[cumulative.length - 1];
    const count = Math.floor(total / spacing) + 1;
    const samples = [];
    let edge = 1;
    for (let index = 0; index < count; index++) {
        const distance = Math.min(total, index * spacing);
        while (edge < cumulative.length - 1 && cumulative[edge] < distance) edge++;
        const startDistance = cumulative[edge - 1];
        const length = Math.max(1e-8, cumulative[edge] - startDistance);
        samples.push(points[edge - 1].clone().lerp(points[edge], (distance - startDistance) / length));
    }
    return samples;
}

function orientNormals(points) {
    const normals = [];
    const tangent = new THREE.Vector3();
    const reference = new THREE.Vector3(0, 0, 1);
    for (let index = 0; index < points.length; index++) {
        const previous = points[Math.max(0, index - 1)];
        const next = points[Math.min(points.length - 1, index + 1)];
        tangent.subVectors(next, previous).normalize();
        const normal = new THREE.Vector3().crossVectors(tangent, reference);
        if (normal.lengthSq() < 1e-6) normal.crossVectors(tangent, new THREE.Vector3(0, 1, 0));
        normal.normalize();
        if (normals.length && normal.dot(normals[normals.length - 1]) < 0) normal.multiplyScalar(-1);
        normals.push(normal);
    }
    return normals;
}

function positionAtGap(field, center, direction, radius, targetGap, contact) {
    const probe = new THREE.Vector3();
    let low = 0;
    let high = 0;
    let found = false;
    // In a bifurcation a ray can cross several lumen surfaces, so signed gap
    // is not globally monotonic. Bracket the first wall locally before using
    // bisection; otherwise an exponential probe can jump into another branch.
    for (let sample = 1; sample <= 640; sample++) {
        high = sample * 0.1;
        probe.copy(center).addScaledVector(direction, high);
        field.querySphere(probe, radius, contact);
        if (contact.signedGap <= targetGap) {
            found = true;
            low = high - 0.1;
            break;
        }
    }
    if (!found) return center.clone();
    for (let iteration = 0; iteration < 14; iteration++) {
        const middle = (low + high) * 0.5;
        probe.copy(center).addScaledVector(direction, middle);
        field.querySphere(probe, radius, contact);
        if (contact.signedGap > targetGap) low = middle;
        else high = middle;
    }
    return center.clone().addScaledVector(direction, high);
}

function finalPenetration(field, body) {
    const contact = createContactResult();
    const start = { x: 0, y: 0, z: 0 };
    const end = { x: 0, y: 0, z: 0 };
    let maximum = 0;
    for (let segment = 0; segment < body.segmentCount; segment++) {
        start.x = body.x[segment];
        start.y = body.y[segment];
        start.z = body.z[segment];
        end.x = body.x[segment + 1];
        end.y = body.y[segment + 1];
        end.z = body.z[segment + 1];
        field.queryCapsule(start, end, body.radius, contact);
        maximum = Math.max(maximum, contact.penetration);
    }
    return maximum;
}

function trimToToolClearance(field, points, radius, maximumBaselinePenetration = 0.02) {
    const contact = createContactResult();
    for (let segment = 0; segment < points.length - 1; segment++) {
        field.queryCapsule(points[segment], points[segment + 1], radius, contact);
        if (contact.penetration <= maximumBaselinePenetration) continue;
        // A centerline can continue into a distal vessel that is narrower than this tool.
        return points.slice(0, Math.max(16, segment + 1));
    }
    return points;
}

function exercisePath(name, field, pathPoints, radius) {
    const spacing = 4;
    const points = trimToToolClearance(field, resamplePath(pathPoints, spacing), radius);
    const normals = orientNormals(points);
    const world = new EndovascularPhysicsWorld({ contactField: field });
    const body = world.createRod(name, points.length, spacing, {
        ...DEFAULT_TOOL_PROFILES.guidewire,
        radius,
        stretchCompliance: 0,
        bendCompliance: 5e-4
    });
    for (let index = 0; index < points.length; index++) {
        body.setNodePosition(index, points[index].x, points[index].y, points[index].z);
    }
    body.captureRestConfiguration();
    body.copyCurrentToPrevious();
    const baselinePenetration = finalPenetration(field, body);
    const contact = createContactResult();
    let impactIndex = Math.floor(points.length * 0.6);
    let bestScore = -Infinity;
    for (let index = 6; index < points.length - 6; index++) {
        field.querySphere(points[index], radius, contact);
        const previousDirection = points[index].clone().sub(points[index - 1]).normalize();
        const nextDirection = points[index + 1].clone().sub(points[index]).normalize();
        const bend = Math.acos(Math.max(-1, Math.min(1, previousDirection.dot(nextDirection))));
        const centerBias = -Math.abs(index / points.length - 0.6);
        const score = contact.signedGap - bend * 8 + centerBias;
        if (contact.signedGap > 0.8 && score > bestScore) {
            bestScore = score;
            impactIndex = index;
        }
    }
    for (let local = -5; local <= 5; local++) {
        const index = impactIndex + local;
        const fade = 0.5 + 0.5 * Math.cos(Math.abs(local) / 5 * Math.PI);
        field.querySphere(points[index], radius, contact);
        const targetGap = contact.signedGap + (-0.12 - contact.signedGap) * fade;
        const target = positionAtGap(
            field,
            points[index],
            normals[index],
            radius,
            targetGap,
            contact
        );
        body.x[index] = target.x;
        body.y[index] = target.y;
        body.z[index] = target.z;
    }
    body.setPinned(0, true);

    const initialPenetration = finalPenetration(field, body);
    console.log(`${name} centerline baseline penetration mm`, baselinePenetration.toFixed(4));
    assert.ok(initialPenetration >= 0.08, `${name} should begin with a meaningful wall overlap`);
    assert.ok(initialPenetration <= 0.2, `${name} transient penetration should stay <= 0.20 mm, got ${initialPenetration}`);
    for (let step = 0; step < 120; step++) world.stepFixed();
    const settledPenetration = finalPenetration(field, body);
    const stats = world.getStats().bodies[0];
    console.log(`${name} path length mm`, ((points.length - 1) * spacing).toFixed(1));
    console.log(`${name} initial penetration mm`, initialPenetration.toFixed(4));
    console.log(`${name} settled penetration mm`, settledPenetration.toFixed(4));
    console.log(`${name} max segment error %`, (stats.maxLengthError * 100).toFixed(4));
    assert.ok(settledPenetration <= 0.05, `${name} settled penetration should stay <= 0.05 mm`);
    assert.ok(stats.maxLengthError <= 0.01, `${name} segment length error should stay <= 1%`);
    assert.equal(stats.finite, true, `${name} should not contain NaN`);
    assert.ok(stats.maxBendAngleDegrees < 150, `${name} should avoid acute folds`);
}

function nearestCenterlineDistance(tree, x, y, z) {
    let nearestDistance = 0;
    let nearestSquared = Infinity;
    for (let node = 0; node < tree.positions.length / 3; node++) {
        const offset = node * 3;
        const dx = tree.positions[offset] - x;
        const dy = tree.positions[offset + 1] - y;
        const dz = tree.positions[offset + 2] - z;
        const squared = dx * dx + dy * dy + dz * dz;
        if (squared >= nearestSquared) continue;
        nearestSquared = squared;
        nearestDistance = tree.distances[node];
    }
    return nearestDistance;
}

function nearestCenterlineInfo(tree, x, y, z) {
    let nearestNode = 0;
    let nearestSquared = Infinity;
    for (let node = 0; node < tree.positions.length / 3; node++) {
        const offset = node * 3;
        const dx = tree.positions[offset] - x;
        const dy = tree.positions[offset + 1] - y;
        const dz = tree.positions[offset + 2] - z;
        const squared = dx * dx + dy * dy + dz * dz;
        if (squared >= nearestSquared) continue;
        nearestSquared = squared;
        nearestNode = node;
    }
    return {
        routeDistance: tree.distances[nearestNode],
        radius: tree.radii[nearestNode],
        radialDistance: Math.sqrt(nearestSquared)
    };
}

function activeBodySpeed(body) {
    let maximum = 0;
    for (let index = body.activeStart; index <= body.activeEnd; index++) {
        maximum = Math.max(maximum, Math.hypot(
            body.velocityX[index],
            body.velocityY[index],
            body.velocityZ[index]
        ));
    }
    return maximum;
}

function activePenetration(field, body) {
    const contact = createContactResult();
    const start = { x: 0, y: 0, z: 0 };
    const end = { x: 0, y: 0, z: 0 };
    let maximum = 0;
    for (
        let segment = Math.max(body.activeStart, body.collisionStartSegment);
        segment <= Math.min(body.activeEnd - 1, body.collisionEndSegment);
        segment++
    ) {
        start.x = body.x[segment];
        start.y = body.y[segment];
        start.z = body.z[segment];
        end.x = body.x[segment + 1];
        end.y = body.y[segment + 1];
        end.z = body.z[segment + 1];
        field.queryCapsule(start, end, body.radius, contact);
        maximum = Math.max(maximum, contact.penetration);
    }
    return maximum;
}

function maximumBodyBend(body, startNode = body.activeStart + 1, endNode = body.activeEnd - 1) {
    let maximum = 0;
    const start = Math.max(body.activeStart + 1, startNode);
    const end = Math.min(body.activeEnd - 1, endNode);
    for (let index = start; index <= end; index++) {
        const ax = body.x[index] - body.x[index - 1];
        const ay = body.y[index] - body.y[index - 1];
        const az = body.z[index] - body.z[index - 1];
        const bx = body.x[index + 1] - body.x[index];
        const by = body.y[index + 1] - body.y[index];
        const bz = body.z[index + 1] - body.z[index];
        const denominator = Math.hypot(ax, ay, az) * Math.hypot(bx, by, bz);
        if (denominator < 1e-8) continue;
        maximum = Math.max(maximum, Math.acos(Math.max(-1, Math.min(1,
            (ax * bx + ay * by + az * bz) / denominator
        ))) * 180 / Math.PI);
    }
    return maximum;
}

function maximumBodySegmentError(
    body,
    startSegment = body.activeStart,
    endSegment = body.activeEnd - 1
) {
    let maximumError = 0;
    for (
        let segment = Math.max(body.activeStart, startSegment);
        segment <= Math.min(body.activeEnd - 1, endSegment);
        segment++
    ) {
        maximumError = Math.max(maximumError, Math.abs(
            Math.hypot(
                body.x[segment + 1] - body.x[segment],
                body.y[segment + 1] - body.y[segment],
                body.z[segment + 1] - body.z[segment]
            ) - body.restLength[segment]
        ));
    }
    return maximumError;
}

function portalExitAngleDegrees(inner, segment, outer) {
    segment = Math.max(inner.activeStart, Math.min(inner.activeEnd - 1, segment));
    const tip = outer.activeEnd;
    const catheterX = outer.x[tip] - outer.x[tip - 1];
    const catheterY = outer.y[tip] - outer.y[tip - 1];
    const catheterZ = outer.z[tip] - outer.z[tip - 1];
    const wireX = inner.x[segment + 1] - inner.x[segment];
    const wireY = inner.y[segment + 1] - inner.y[segment];
    const wireZ = inner.z[segment + 1] - inner.z[segment];
    const denominator = Math.max(
        1e-8,
        Math.hypot(catheterX, catheterY, catheterZ) *
            Math.hypot(wireX, wireY, wireZ)
    );
    return Math.acos(Math.max(-1, Math.min(1,
        (catheterX * wireX + catheterY * wireY + catheterZ * wireZ) /
            denominator
    ))) * 180 / Math.PI;
}

function coupledContainmentEscape(inner, outer, startNode, endNode) {
    const allowedRadius = Math.max(0, outer.innerRadius - inner.radius);
    const outerStart = Math.max(outer.activeStart, 0);
    const outerEnd = Math.min(outer.activeEnd - 1, outer.segmentCount - 1);
    const innerStart = Math.max(inner.activeStart, startNode);
    const innerEnd = Math.min(inner.activeEnd, endNode);
    let nodeEscape = 0;
    let segmentEscape = 0;
    let nodeEscapeIndex = -1;
    let segmentEscapeIndex = -1;

    const distanceToOuter = (x, y, z) => {
        let closestSquared = Infinity;
        for (let segment = outerStart; segment <= outerEnd; segment++) {
            const ax = outer.x[segment];
            const ay = outer.y[segment];
            const az = outer.z[segment];
            const dx = outer.x[segment + 1] - ax;
            const dy = outer.y[segment + 1] - ay;
            const dz = outer.z[segment + 1] - az;
            const lengthSquared = dx * dx + dy * dy + dz * dz;
            const t = Math.max(0, Math.min(1,
                ((x - ax) * dx + (y - ay) * dy + (z - az) * dz) /
                    Math.max(1e-8, lengthSquared)
            ));
            const rx = x - (ax + dx * t);
            const ry = y - (ay + dy * t);
            const rz = z - (az + dz * t);
            closestSquared = Math.min(closestSquared, rx * rx + ry * ry + rz * rz);
        }
        return Math.max(0, Math.sqrt(closestSquared) - allowedRadius);
    };

    for (let index = innerStart; index <= innerEnd; index++) {
        const escape = distanceToOuter(
            inner.x[index], inner.y[index], inner.z[index]
        );
        if (escape > nodeEscape) {
            nodeEscape = escape;
            nodeEscapeIndex = index;
        }
    }
    for (let index = innerStart; index < innerEnd; index++) {
        for (const t of [0.2, 0.4, 0.6, 0.8]) {
            const escape = distanceToOuter(
                inner.x[index] * (1 - t) + inner.x[index + 1] * t,
                inner.y[index] * (1 - t) + inner.y[index + 1] * t,
                inner.z[index] * (1 - t) + inner.z[index + 1] * t
            );
            if (escape > segmentEscape) {
                segmentEscape = escape;
                segmentEscapeIndex = index;
            }
        }
    }
    return { nodeEscape, segmentEscape, nodeEscapeIndex, segmentEscapeIndex };
}

function renderedContainmentEscape(points, outer) {
    const startIndex = points.containedStartIndex;
    const endIndex = points.containedEndIndex;
    if (
        points.length < 2 ||
        startIndex < 0 ||
        endIndex < startIndex
    ) return 0;
    const vectors = points.map(point => new THREE.Vector3(
        point.x,
        point.y,
        point.z
    ));
    const curve = vectors.length === 2
        ? new THREE.LineCurve3(vectors[0], vectors[1])
        : new THREE.CatmullRomCurve3(vectors, false, 'centripetal', 0.5);
    const tubularSegments = Math.min(
        900,
        Math.max(8, Math.ceil((vectors.length - 1) * 3))
    );
    const firstSample = Math.max(
        0,
        Math.floor(startIndex / (vectors.length - 1) * tubularSegments)
    );
    const lastSample = Math.min(
        tubularSegments,
        Math.ceil(endIndex / (vectors.length - 1) * tubularSegments)
    );
    const allowedRadius = Math.max(
        0,
        PIGTAIL_CATHETER_INNER_RADIUS_MM - GUIDEWIRE_RENDER_RADIUS_MM
    );
    const sample = new THREE.Vector3();
    let maximumEscape = 0;
    for (let sampleIndex = firstSample; sampleIndex <= lastSample; sampleIndex++) {
        curve.getPoint(sampleIndex / tubularSegments, sample);
        let closestSquared = Infinity;
        for (
            let segment = Math.max(0, outer.activeStart);
            segment < outer.activeEnd;
            segment++
        ) {
            const ax = outer.x[segment];
            const ay = outer.y[segment];
            const az = outer.z[segment];
            const dx = outer.x[segment + 1] - ax;
            const dy = outer.y[segment + 1] - ay;
            const dz = outer.z[segment + 1] - az;
            const lengthSquared = dx * dx + dy * dy + dz * dz;
            const t = Math.max(0, Math.min(1,
                ((sample.x - ax) * dx +
                    (sample.y - ay) * dy +
                    (sample.z - az) * dz) /
                    Math.max(1e-8, lengthSquared)
            ));
            const rx = sample.x - (ax + dx * t);
            const ry = sample.y - (ay + dy * t);
            const rz = sample.z - (az + dz * t);
            closestSquared = Math.min(
                closestSquared,
                rx * rx + ry * ry + rz * rz
            );
        }
        maximumEscape = Math.max(
            maximumEscape,
            Math.max(0, Math.sqrt(closestSquared) - allowedRadius)
        );
    }
    return maximumEscape;
}

function pigtailDistalRadialRecovery(body, catheter) {
    const naturalArcLength = PIGTAIL_NATURAL_ARC_LENGTH_MM;
    const bendStartDistance = Math.max(
        catheter.guidewireInserted,
        catheter.progress - naturalArcLength
    );
    let baseIndex = body.activeEnd - 1;
    for (let index = body.activeStart + 1; index < body.activeEnd; index++) {
        if ((catheter._centerlineDistances[index] ?? -Infinity) < bendStartDistance) continue;
        baseIndex = Math.max(body.activeStart + 1, index - 1);
        break;
    }
    const tangentStart = Math.max(body.activeStart, baseIndex - 2);
    let axisX = body.x[baseIndex] - body.x[tangentStart];
    let axisY = body.y[baseIndex] - body.y[tangentStart];
    let axisZ = body.z[baseIndex] - body.z[tangentStart];
    const axisLength = Math.hypot(axisX, axisY, axisZ);
    assert.ok(axisLength > 1e-6, 'the real-aorta Pigtail base must have a valid tangent');
    axisX /= axisLength;
    axisY /= axisLength;
    axisZ /= axisLength;
    let maximumRadial = 0;
    for (let index = baseIndex + 1; index <= body.activeEnd; index++) {
        const dx = body.x[index] - body.x[baseIndex];
        const dy = body.y[index] - body.y[baseIndex];
        const dz = body.z[index] - body.z[baseIndex];
        const axial = dx * axisX + dy * axisY + dz * axisZ;
        maximumRadial = Math.max(maximumRadial, Math.hypot(
            dx - axisX * axial,
            dy - axisY * axial,
            dz - axisZ * axial
        ));
    }
    return maximumRadial;
}

function pigtailLoopMetrics(
    body,
    naturalArcLength = PIGTAIL_NATURAL_ARC_LENGTH_MM
) {
    let baseIndex = body.activeEnd;
    let materialLength = 0;
    while (baseIndex > body.activeStart && materialLength < naturalArcLength) {
        baseIndex--;
        materialLength += body.restLength[baseIndex];
    }
    let totalTurnDegrees = 0;
    let previousDirection = null;
    let maximumSpan = 0;
    for (let index = baseIndex; index <= body.activeEnd; index++) {
        for (let other = baseIndex; other < index; other++) {
            maximumSpan = Math.max(maximumSpan, Math.hypot(
                body.x[index] - body.x[other],
                body.y[index] - body.y[other],
                body.z[index] - body.z[other]
            ));
        }
        if (index === baseIndex) continue;
        const dx = body.x[index] - body.x[index - 1];
        const dy = body.y[index] - body.y[index - 1];
        const dz = body.z[index] - body.z[index - 1];
        const length = Math.max(1e-8, Math.hypot(dx, dy, dz));
        const direction = [dx / length, dy / length, dz / length];
        if (previousDirection) {
            totalTurnDegrees += Math.acos(Math.max(-1, Math.min(1,
                previousDirection[0] * direction[0] +
                previousDirection[1] * direction[1] +
                previousDirection[2] * direction[2]
            ))) * 180 / Math.PI;
        }
        previousDirection = direction;
    }
    const closureDistance = Math.hypot(
        body.x[body.activeEnd] - body.x[baseIndex],
        body.y[body.activeEnd] - body.y[baseIndex],
        body.z[body.activeEnd] - body.z[baseIndex]
    );
    return {
        baseIndex,
        nodeCount: body.activeEnd - baseIndex + 1,
        totalTurnDegrees,
        maximumSpan,
        closureDistance,
        closureRatio: closureDistance / Math.max(1e-8, maximumSpan)
    };
}

function exerciseCoupledCatheterInAorta(field, vessel, {
    catheterType = 'berenstein',
    guidewireTarget = 236,
    catheterTarget = 276,
    guidewireReleaseTarget = null,
    fixtureName = 'real-aorta-coupled',
    rotationDegrees = null,
    idleStepsOverride = null,
    assertPigtailRecovery = true,
    assertPortalContinuity = false
} = {}) {
    const disableContainment = process.env.OET_DISABLE_AORTA_CONTAINMENT === '1';
    const dt = 1 / 120;
    const guidewireSpacing = 5;
    const guidewireLength = 1000;
    const wire = new ElasticRod(
        guidewireLength / guidewireSpacing + 1,
        guidewireSpacing,
        { constraintIterations: 28 }
    );
    applyGuidewireMaterialProfile(wire, { segmentLength: guidewireSpacing });
    const solver = new GuidewireSolver({
        rod: wire,
        segmentLength: guidewireSpacing,
        guidewireLength,
        sheath: vessel.sheath,
        advanceRate: 44,
        minInsert: 0,
        maxInsert: guidewireLength,
        lumenClearance: DEFAULT_TOOL_PROFILES.guidewire.radius,
        straightening: 0.72,
        routeBlend: 0,
        relaxationIterations: 6,
        lengthIterations: 10,
        meshClearance: DEFAULT_TOOL_PROFILES.guidewire.radius,
        foldGuardAngle: 166,
        foldGuardStrength: 0.62,
        foldGuardPasses: 2,
        foldGuardCenterPull: 1.25,
        stabilityRepairSegmentError: 0.09,
        stabilityRepairBendAngle: 150,
        stabilityRepairTargetBendAngle: 112,
        stabilityRepairPasses: 3,
        stabilityRepairLengthIterations: 10,
        tipBacktrackAngle: 108,
        tipBacktrackStrength: 1,
        segmentProjectionBlend: 0.48,
        maxSegmentProjectionStep: 0.32,
        collisionProjectionRepeats: 1,
        segmentSamples: [0.1, 0.24, 0.38, 0.52, 0.66, 0.8, 0.93],
        finalCollisionPasses: 3,
        finalLengthPasses: 2,
        finalProjectionPasses: 2
    });
    const catheter = new PigtailCatheter({
        wire,
        segmentLength: guidewireSpacing,
        guidewireLength,
        tailProgressRef: () => solver.progress,
        vessel,
        maxLength: guidewireLength
    });
    catheter.setType(catheterType);
    catheter.setExternalCollisionSolver(true);

    const world = new EndovascularPhysicsWorld({
        contactField: field,
        fixedDt: dt,
        maxSubsteps: 2,
        iterations: 6,
        penetrationIterations: 8,
        highPenetration: 0.15,
        contactActivation: 0.2
    });
    const wireBody = world.createRod(
        `${fixtureName}-guidewire`,
        wire.nodes.length,
        guidewireSpacing,
        { ...DEFAULT_TOOL_PROFILES.guidewire, rodModel: 'kirchhoff' }
    );
    if (process.env.OET_TRACE_AORTA_FOLD === '1') {
        let lastWirePhaseBend = 0;
        wireBody.debugConstraintPhase = (phase, body) => {
            let maximumBend = 0;
            let maximumNode = -1;
            for (let node = body.activeStart + 1; node < body.activeEnd; node++) {
                const ax = body.x[node] - body.x[node - 1];
                const ay = body.y[node] - body.y[node - 1];
                const az = body.z[node] - body.z[node - 1];
                const bx = body.x[node + 1] - body.x[node];
                const by = body.y[node + 1] - body.y[node];
                const bz = body.z[node + 1] - body.z[node];
                const denominator = Math.hypot(ax, ay, az) * Math.hypot(bx, by, bz);
                const bend = denominator > 1e-9
                    ? Math.acos(Math.max(-1, Math.min(1, (
                        ax * bx + ay * by + az * bz
                    ) / denominator))) * 180 / Math.PI
                    : 180;
                if (bend > maximumBend) {
                    maximumBend = bend;
                    maximumNode = node;
                }
            }
            if (maximumBend > 60 && lastWirePhaseBend <= 60) {
                const window = [];
                for (
                    let node = Math.max(body.activeStart, maximumNode - 2);
                    node <= Math.min(body.activeEnd, maximumNode + 2);
                    node++
                ) {
                    window.push({
                        node,
                        position: [body.x[node], body.y[node], body.z[node]],
                        previous: [
                            body.previousX[node], body.previousY[node], body.previousZ[node]
                        ],
                        velocity: [
                            body.velocityX[node], body.velocityY[node], body.velocityZ[node]
                        ],
                        restLength: body.restLength[Math.min(node, body.segmentCount - 1)],
                        wallActive: body.wallActive[Math.min(node, body.segmentCount - 1)]
                    });
                }
                console.log('real aorta first catastrophic wire fold', JSON.stringify({
                    phase,
                    progress: solver.progress,
                    maximumBend,
                    maximumNode,
                    previousPhaseBend: lastWirePhaseBend,
                    activeRange: [body.activeStart, body.activeEnd],
                    collisionRange: [body.collisionStartSegment, body.collisionEndSegment],
                    sheathMaterialEndNode: body.sheathMaterialEndNode,
                    window
                }));
            }
            if (
                process.env.OET_TRACE_AORTA_PHASES === '1' &&
                solver.progress >= 148.7 && solver.progress <= 149.4
            ) {
                let maximumLengthError = 0;
                for (let segment = body.activeStart; segment < body.activeEnd; segment++) {
                    const length = Math.hypot(
                        body.x[segment + 1] - body.x[segment],
                        body.y[segment + 1] - body.y[segment],
                        body.z[segment + 1] - body.z[segment]
                    );
                    maximumLengthError = Math.max(
                        maximumLengthError,
                        Math.abs(length - body.restLength[segment])
                    );
                }
                console.log('wire phase', phase, {
                    progress: solver.progress,
                    bend: maximumBend,
                    node: maximumNode,
                    maximumLengthError
                });
            }
            lastWirePhaseBend = maximumBend;
        };
    }
    wireBody.syncFromElasticRod(wire);
    wireBody.captureKirchhoffRestConfiguration({ captureRestRotation: false });
    applyKirchhoffMaterialProfile(wireBody, 'glidewire', {
        activeStart: 0,
        activeEnd: wireBody.count - 1,
        materialCoordinates: Float64Array.from(
            { length: wireBody.count },
            (_, index) => index * guidewireSpacing
        ),
        tipCoordinate: guidewireLength
    });
    const catheterBody = world.createRod(
        `${fixtureName}-catheter`,
        320,
        4,
        { ...DEFAULT_TOOL_PROFILES.catheter, rodModel: 'kirchhoff' }
    );
    if (process.env.OET_TRACE_AORTA_FOLD === '1') {
        let lastPhaseBend = 0;
        catheterBody.debugConstraintPhase = (phase, body) => {
            if (catheter.progress < 60) return;
            let maximumBend = 0;
            let maximumNode = -1;
            for (let node = body.activeStart + 1; node < body.activeEnd; node++) {
                const incomingX = body.x[node] - body.x[node - 1];
                const incomingY = body.y[node] - body.y[node - 1];
                const incomingZ = body.z[node] - body.z[node - 1];
                const outgoingX = body.x[node + 1] - body.x[node];
                const outgoingY = body.y[node + 1] - body.y[node];
                const outgoingZ = body.z[node + 1] - body.z[node];
                const denominator = Math.hypot(incomingX, incomingY, incomingZ) *
                    Math.hypot(outgoingX, outgoingY, outgoingZ);
                const bend = denominator > 1e-9
                    ? Math.acos(Math.max(-1, Math.min(1, (
                        incomingX * outgoingX +
                        incomingY * outgoingY +
                        incomingZ * outgoingZ
                    ) / denominator))) * 180 / Math.PI
                    : 180;
                if (bend > maximumBend) {
                    maximumBend = bend;
                    maximumNode = node;
                }
            }
            if (maximumBend > 60 && lastPhaseBend <= 60) {
                const window = [];
                for (
                    let node = Math.max(body.activeStart, maximumNode - 2);
                    node <= Math.min(body.activeEnd, maximumNode + 2);
                    node++
                ) {
                    window.push({
                        node,
                        material: body.materialCoordinate[node],
                        position: [body.x[node], body.y[node], body.z[node]],
                        previous: [
                            body.previousX[node],
                            body.previousY[node],
                            body.previousZ[node]
                        ],
                        velocity: [
                            body.velocityX[node],
                            body.velocityY[node],
                            body.velocityZ[node]
                        ],
                        restLength: body.restLength[Math.min(node, body.segmentCount - 1)],
                        wallActive: body.wallActive[Math.min(node, body.segmentCount - 1)]
                    });
                }
                console.log('real aorta first catastrophic catheter fold', JSON.stringify({
                    phase,
                    progress: catheter.progress,
                    maximumBend,
                    maximumNode,
                    previousPhaseBend: lastPhaseBend,
                    activeRange: [body.activeStart, body.activeEnd],
                    collisionRange: [body.collisionStartSegment, body.collisionEndSegment],
                    sheathMaterialEndNode: body.sheathMaterialEndNode,
                    window
                }));
            }
            lastPhaseBend = maximumBend;
        };
    }
    const pigtailPhaseMetrics = {};
    if (catheterType === 'pigtail') {
        catheterBody.debugConstraintPhase = (phase, body) => {
            if (catheter._xpbdPigtailRecovery < 0.89) return;
            const metrics = pigtailLoopMetrics(body);
            let signedTurn = 0;
            let negativeTurn = 0;
            for (
                let segment = metrics.baseIndex;
                segment < body.activeEnd;
                segment++
            ) {
                const ax = body.x[segment] - body.x[segment - 1];
                const ay = body.y[segment] - body.y[segment - 1];
                const az = body.z[segment] - body.z[segment - 1];
                const bx = body.x[segment + 1] - body.x[segment];
                const by = body.y[segment + 1] - body.y[segment];
                const bz = body.z[segment + 1] - body.z[segment];
                const turn = Math.atan2(
                    body.restDirectionAxisX[segment] * (ay * bz - az * by) +
                        body.restDirectionAxisY[segment] * (az * bx - ax * bz) +
                        body.restDirectionAxisZ[segment] * (ax * by - ay * bx),
                    ax * bx + ay * by + az * bz
                ) * 180 / Math.PI;
                signedTurn += turn;
                if (turn < 0) negativeTurn += -turn;
            }
            pigtailPhaseMetrics[phase] = {
                turn: Number(metrics.totalTurnDegrees.toFixed(1)),
                signedTurn: Number(signedTurn.toFixed(1)),
                negativeTurn: Number(negativeTurn.toFixed(1)),
                closure: Number(metrics.closureRatio.toFixed(3))
            };
        };
    }
    catheter.syncXpbdBody(catheterBody);
    world.addSheath({
        start: vessel.sheath.start,
        end: vessel.sheath.end,
        innerRadius: DEFAULT_TOOL_PROFILES.sheath.innerRadius,
        proximalExtension: 90,
        bodies: [wireBody, catheterBody]
    });
    const containment = world.addContainment(wireBody, catheterBody, {
        model: 'kirchhoff',
        innerRadius: DEFAULT_TOOL_PROFILES.catheter.innerRadius,
        openProximal: true,
        openDistal: true,
        searchWindow: 2,
        outerStartNode: catheter.physicsLumenStartNode,
        innerResponse: 1,
        outerResponse: 0,
        portalInnerResponse: 1,
        portalOuterResponse: 0,
        portalCompliance: 1e-7,
        portalTransitionLength: 4,
        portalMaxCorrection: 0.15,
        finalProjection: 'inner',
        outerFollowsInnerCenterline: false,
        innerFollowsOuterCenterline: true,
        enforceDistalPortal: true,
        containedLength: 0,
        enabled: false
    });
    const externalContact = world.addToolContact(wireBody, catheterBody, {
        friction: 0.08,
        openDistalB: true,
        enabled: false
    });
    let portalInnerDriven = true;
    const renderPolyline = [];

    const step = (guidewireCommand, catheterCommand) => {
        const guidewireDelta = solver.advance(
            guidewireCommand,
            dt,
            null,
            { routeAssist: false, boundaryDriven: true }
        );
        const inserted = solver.progress;
        const catheterProgressBefore = catheter.progress;
        catheter.advance(catheterCommand, dt, inserted);
        const catheterDelta = catheter.progress - catheterProgressBefore;
        wireBody.syncFromElasticRod(wire);
        wireBody.setActiveRange(
            Math.min(wireBody.count - 2, Math.max(0, solver.firstInsertedNodeIndex() - 1)),
            wireBody.count - 1
        );
        let wireWallCollisionStart = Math.max(
            0,
            solver.firstLumenNodeIndex() - 1
        );
        let wireWallCollisionEnd = wireBody.segmentCount - 1;
        catheter.stepPhysics(dt, { collisions: false });
        const activeCount = catheter.syncXpbdBody(catheterBody);
        const firstContainedNode = Math.max(0, Math.ceil(
            (guidewireLength - inserted) / guidewireSpacing
        ));
        const materialEndNode = Math.min(
            wireBody.count - 1,
            Math.floor(
                (guidewireLength - inserted + catheter.progress) / guidewireSpacing
            )
        );
        containment.outerStartNode = catheter.physicsLumenStartNode;
        const lastContainedNode = materialEndNode;
        containment.enabled = !disableContainment &&
            catheter.progress > 0.5 &&
            activeCount >= 2 &&
            lastContainedNode >= firstContainedNode;
        containment.startNode = firstContainedNode;
        containment.endNode = Math.max(firstContainedNode, lastContainedNode);
        containment.innerArcOffset =
            firstContainedNode * guidewireSpacing - guidewireLength + inserted;
        containment.containedLength = Math.min(catheter.progress, inserted);
        containment.portalRetractionDistance = Math.max(
            0,
            catheter.progress - inserted
        );
        containment.enforceDistalPortal = true;
        if (containment.model !== 'kirchhoff') {
            const relativePortalAdvance = guidewireDelta - catheterDelta;
            if (relativePortalAdvance > 1e-5) portalInnerDriven = true;
            else if (relativePortalAdvance < -1e-5) portalInnerDriven = false;
            containment.portalInnerResponse = portalInnerDriven ? 1 : 0;
            containment.portalOuterResponse = portalInnerDriven ? 0 : 1;
            containment.limitDistalCorrection =
                Math.abs(guidewireDelta) > 1e-5 || Math.abs(catheterDelta) > 1e-5;
            containment.preserveStationaryInnerLength =
                Math.abs(catheterDelta) > 1e-5 && Math.abs(guidewireCommand) <= 1e-5;
            containment.reconcileMovingInnerStructure =
                Math.abs(catheterDelta) > 1e-5 && Math.abs(guidewireCommand) > 1e-5;
            containment.outerResponse = containment.preserveStationaryInnerLength
                ? 0.2
                : containment.reconcileMovingInnerStructure
                    ? 0.04
                    : 0;
        }
        wireBody.maxFrameDisplacement =
            containment.preserveStationaryInnerLength ? 1.5 : Infinity;
        wireBody.frameDisplacementStartNode = Math.max(
            wireBody.activeStart,
            containment.endNode
        );
        if (containment.enabled) {
            const firstWallExposedSegment = Math.max(
                wireWallCollisionStart,
                materialEndNode
            );
            if (firstWallExposedSegment <= wireBody.activeEnd - 1) {
                wireWallCollisionStart = firstWallExposedSegment;
            } else {
                wireWallCollisionStart = wireBody.activeEnd;
                wireWallCollisionEnd = wireBody.activeEnd - 1;
            }
        }
        wireBody.setCollisionRange(wireWallCollisionStart, wireWallCollisionEnd);

        const catheterEndSegment = Math.max(0, activeCount - 2);
        const firstExternalSegment = Math.max(0, Math.min(
            wireBody.segmentCount - 1,
            materialEndNode + 1
        ));
        externalContact.enabled =
            catheter.progress > 4 &&
            activeCount >= 2 &&
            inserted > catheter.progress + 0.5 &&
            firstExternalSegment <= wireBody.activeEnd - 1;
        externalContact.startSegmentA = firstExternalSegment;
        externalContact.endSegmentA = Math.min(
            wireBody.activeEnd - 1,
            firstExternalSegment + 16
        );
        externalContact.startSegmentB = Math.max(0, catheterEndSegment - 8);
        externalContact.endSegmentB = catheterEndSegment;
        const guidewireIsToolCoupled = containment.enabled ||
            externalContact.enabled;
        wireBody.projectionVelocityRetention = guidewireIsToolCoupled
            ? 0.005
            : 1;
        wireBody.distalProjectionVelocityRetention = 1;
        wireBody.distalProjectionVelocityRetentionStartNode =
            guidewireIsToolCoupled
                ? externalContact.enabled
                    ? externalContact.endSegmentA + 2
                    : containment.endNode + 1
                : Infinity;
        world.stepFixed();
        const capturedEndNode = spatiallyCapturedContainmentEnd({
            innerBody: wireBody,
            outerBody: catheterBody,
            firstContainedNode,
            materialEndNode,
            outerStartNode: containment.outerStartNode,
            outerInnerRadius: containment.innerRadius,
            closestSegment: containment.closestSegment
        });
        containment.renderEndNode = Math.min(
            materialEndNode,
            Math.max(
                firstContainedNode,
                materialEndNode - 1,
                capturedEndNode
            )
        );
        wireBody.syncToElasticRod(wire);
        return { firstContainedNode, lastContainedNode, capturedEndNode };
    };

    const feedTo = (target, rate, commandStep) => {
        let safety = 0;
        while (rate === 44 ? solver.progress < target - 1e-7 : catheter.progress < target - 1e-7) {
            const progress = rate === 44 ? solver.progress : catheter.progress;
            commandStep(Math.min(1, (target - progress) / (rate * dt)));
            assert.ok(++safety < 2000, 'coupled real-aorta feed should reach its target');
        }
    };

    feedTo(guidewireTarget, 44, command => step(command, 0));
    const prewireIdleSteps = Number(
        process.env.OET_PREWIRE_IDLE_STEPS ?? 60
    );
    for (let index = 0; index < prewireIdleSteps; index++) step(0, 0);
    console.log('real aorta coupled pre-catheter wire speed mm/s',
        activeBodySpeed(wireBody).toFixed(4));
    if (process.env.OET_TRACE_AORTA_FOLD === '1') {
        const outlet = solver.firstLumenNodeIndex();
        const worldStats = world.getStats();
        let broadPhaseMaximumCandidates = 0;
        let broadPhaseTotalCandidates = 0;
        for (let cell = 0; cell + 1 < field.broadPhaseOffsets.length; cell++) {
            const candidates = field.broadPhaseOffsets[cell + 1] -
                field.broadPhaseOffsets[cell];
            broadPhaseMaximumCandidates = Math.max(
                broadPhaseMaximumCandidates,
                candidates
            );
            broadPhaseTotalCandidates += candidates;
        }
        const activeCandidateCounts = [];
        for (let segment = wireBody.activeStart; segment < wireBody.activeEnd; segment++) {
            if (!wireBody.wallActive[segment]) continue;
            const x = (wireBody.x[segment] + wireBody.x[segment + 1]) * 0.5;
            const y = (wireBody.y[segment] + wireBody.y[segment + 1]) * 0.5;
            const z = (wireBody.z[segment] + wireBody.z[segment + 1]) * 0.5;
            const cellX = Math.floor((x - field.broadPhaseOrigin[0]) /
                field.broadPhaseCellSize);
            const cellY = Math.floor((y - field.broadPhaseOrigin[1]) /
                field.broadPhaseCellSize);
            const cellZ = Math.floor((z - field.broadPhaseOrigin[2]) /
                field.broadPhaseCellSize);
            if (
                cellX < 0 || cellY < 0 || cellZ < 0 ||
                cellX >= field.broadPhaseDimensions[0] ||
                cellY >= field.broadPhaseDimensions[1] ||
                cellZ >= field.broadPhaseDimensions[2]
            ) continue;
            const cell = cellX + field.broadPhaseDimensions[0] * (
                cellY + field.broadPhaseDimensions[1] * cellZ
            );
            activeCandidateCounts.push(
                field.broadPhaseOffsets[cell + 1] - field.broadPhaseOffsets[cell]
            );
        }
        console.log('real aorta pre-catheter outlet wire', JSON.stringify({
            stats: worldStats.bodies[0],
            phases: worldStats.phases,
            lengthPolishPasses: worldStats.lastLengthPolishPasses,
            wallRepairPasses: worldStats.lastWallRepairPasses,
            wallRepairResiduals: worldStats.wallRepairResiduals,
            wallRepairWorstSegments: worldStats.wallRepairWorstSegments,
            wallRepairWorstBodies: worldStats.wallRepairWorstBodies,
            contactFieldStats: field.getStats(),
            centerlineSegments: field.centerline.length / field.centerlineStride,
            broadPhaseMaximumCandidates,
            broadPhaseAverageCandidates: broadPhaseTotalCandidates /
                Math.max(1, field.broadPhaseOffsets.length - 1),
            activeCandidateCounts,
            outlet,
            nodes: Array.from({ length: 8 }, (_, offset) => {
                const node = Math.max(
                    wireBody.activeStart,
                    Math.min(wireBody.activeEnd, outlet - 3 + offset)
                );
                return {
                    node,
                    material: wireBody.materialCoordinate[node],
                    position: [wireBody.x[node], wireBody.y[node], wireBody.z[node]],
                    velocity: [
                        wireBody.velocityX[node],
                        wireBody.velocityY[node],
                        wireBody.velocityZ[node]
                    ]
                };
            })
        }));
    }
    if (process.env.OET_TRACE_PREWIRE_ONLY === '1') {
        catheter.dispose();
        return;
    }
    let previousFeedCatheterTip = null;
    let maximumFeedCatheterTipStep = 0;
    let maximumFeedWireTipStep = 0;
    let previousFeedWireTip = null;
    let previousFeedRenderedWireTip = null;
    let maximumDeployedFeedCatheterTipStep = 0;
    let maximumDeployedFeedWireTipStep = 0;
    let maximumDeployedFeedRenderedWireTipStep = 0;
    let maximumFeedCatheterBend = 0;
    let maximumFeedNodeEscape = 0;
    let maximumFeedRenderedEscape = 0;
    let maximumFeedCatheterTipStepAt = '';
    let maximumFeedWireTipStepAt = '';
    let maximumDeployedFeedCatheterTipStepAt = '';
    let maximumDeployedFeedWireTipStepAt = '';
    let maximumDeployedFeedRenderedWireTipStepAt = '';
    let maximumFeedNodeEscapeAt = '';
    let catheterFeedStep = 0;
    feedTo(catheterTarget, 52, command => {
        const range = step(0, command);
        const catheterTip = [
            catheterBody.x[catheterBody.activeEnd],
            catheterBody.y[catheterBody.activeEnd],
            catheterBody.z[catheterBody.activeEnd]
        ];
        const wireTip = [
            wireBody.x[wireBody.activeEnd],
            wireBody.y[wireBody.activeEnd],
            wireBody.z[wireBody.activeEnd]
        ];
        if (previousFeedCatheterTip) {
            const catheterTipStep = Math.hypot(
                catheterTip[0] - previousFeedCatheterTip[0],
                catheterTip[1] - previousFeedCatheterTip[1],
                catheterTip[2] - previousFeedCatheterTip[2]
            );
            const wireTipStep = Math.hypot(
                wireTip[0] - previousFeedWireTip[0],
                wireTip[1] - previousFeedWireTip[1],
                wireTip[2] - previousFeedWireTip[2]
            );
            if (catheterTipStep > maximumFeedCatheterTipStep) {
                maximumFeedCatheterTipStep = catheterTipStep;
                maximumFeedCatheterTipStepAt =
                    `${catheterFeedStep}/${catheter.progress.toFixed(2)}`;
            }
            if (wireTipStep > maximumFeedWireTipStep) {
                maximumFeedWireTipStep = wireTipStep;
                maximumFeedWireTipStepAt =
                    `${catheterFeedStep}/${catheter.progress.toFixed(2)}`;
            }
            if (catheter.progress >= vessel.sheath.length + 40) {
                if (catheterTipStep > maximumDeployedFeedCatheterTipStep) {
                    maximumDeployedFeedCatheterTipStep = catheterTipStep;
                    maximumDeployedFeedCatheterTipStepAt =
                        `${catheterFeedStep}/${catheter.progress.toFixed(2)}`;
                }
                if (wireTipStep > maximumDeployedFeedWireTipStep) {
                    maximumDeployedFeedWireTipStep = wireTipStep;
                    maximumDeployedFeedWireTipStepAt =
                        `${catheterFeedStep}/${catheter.progress.toFixed(2)}`;
                }
            }
        }
        previousFeedCatheterTip = catheterTip;
        previousFeedWireTip = wireTip;
        maximumFeedCatheterBend = Math.max(
            maximumFeedCatheterBend,
            maximumBodyBend(
                catheterBody,
                catheterBody.collisionStartSegment + 2,
                catheterBody.activeEnd - 5
            )
        );
        const feedEscape = coupledContainmentEscape(
            wireBody,
            catheterBody,
            range.firstContainedNode,
            range.capturedEndNode
        );
        if (feedEscape.nodeEscape > maximumFeedNodeEscape) {
            maximumFeedNodeEscape = feedEscape.nodeEscape;
            maximumFeedNodeEscapeAt =
                `${catheterFeedStep}/${catheter.progress.toFixed(2)}/${feedEscape.nodeEscapeIndex}`;
        }
        const feedRenderPoints = buildContainedGuidewireRenderPolyline({
            guidewireNodes: wire.nodes,
            outerBody: catheterBody,
            containment,
            out: renderPolyline
        });
        const renderedWireTipPoint = feedRenderPoints[feedRenderPoints.length - 1];
        const renderedWireTip = renderedWireTipPoint
            ? [renderedWireTipPoint.x, renderedWireTipPoint.y, renderedWireTipPoint.z]
            : null;
        if (
            previousFeedRenderedWireTip &&
            renderedWireTip &&
            catheter.progress >= vessel.sheath.length + 40
        ) {
            const renderedTipStep = Math.hypot(
                renderedWireTip[0] - previousFeedRenderedWireTip[0],
                renderedWireTip[1] - previousFeedRenderedWireTip[1],
                renderedWireTip[2] - previousFeedRenderedWireTip[2]
            );
            if (renderedTipStep > maximumDeployedFeedRenderedWireTipStep) {
                maximumDeployedFeedRenderedWireTipStep = renderedTipStep;
                maximumDeployedFeedRenderedWireTipStepAt =
                    `${catheterFeedStep}/${catheter.progress.toFixed(2)}`;
            }
        }
        previousFeedRenderedWireTip = renderedWireTip;
        maximumFeedRenderedEscape = Math.max(
            maximumFeedRenderedEscape,
            renderedContainmentEscape(feedRenderPoints, catheterBody)
        );
        catheterFeedStep++;
    });

    if (
        Number.isFinite(guidewireReleaseTarget) &&
        guidewireReleaseTarget < solver.progress - 1e-7
    ) {
        let safety = 0;
        while (solver.progress > guidewireReleaseTarget + 1e-7) {
            const command = -Math.min(
                1,
                (solver.progress - guidewireReleaseTarget) / (44 * dt)
            );
            step(command, 0);
            assert.ok(++safety < 2000,
                'the real-aorta guidewire withdrawal should reach its release target');
        }
    }
    console.log('real aorta coupled post-feed catheter/wire speed mm/s',
        activeBodySpeed(catheterBody).toFixed(4), activeBodySpeed(wireBody).toFixed(4));
    console.log('real aorta coupled feed catheter/wire tip step mm',
        maximumFeedCatheterTipStep.toFixed(4), maximumFeedCatheterTipStepAt,
        maximumFeedWireTipStep.toFixed(4), maximumFeedWireTipStepAt);
    console.log('real aorta coupled deployed feed catheter/physical wire/rendered wire tip step mm',
        maximumDeployedFeedCatheterTipStep.toFixed(4),
        maximumDeployedFeedCatheterTipStepAt,
        maximumDeployedFeedWireTipStep.toFixed(4),
        maximumDeployedFeedWireTipStepAt,
        maximumDeployedFeedRenderedWireTipStep.toFixed(4),
        maximumDeployedFeedRenderedWireTipStepAt);
    console.log('real aorta coupled feed catheter bend/node escape',
        maximumFeedCatheterBend.toFixed(2), maximumFeedNodeEscape.toFixed(4),
        maximumFeedNodeEscapeAt);
    console.log('real aorta coupled feed rendered escape mm',
        maximumFeedRenderedEscape.toFixed(4));
    if (catheterType === 'pigtail') {
        const testRotationDegrees = Number.isFinite(rotationDegrees)
            ? rotationDegrees
            : Number(process.env.PIGTAIL_TEST_ROTATION_DEG ?? 0);
        if (Number.isFinite(testRotationDegrees) && Math.abs(testRotationDegrees) > 1e-6) {
            catheter.rotate(
                Math.sign(testRotationDegrees),
                Math.abs(testRotationDegrees) * Math.PI / 180 / (Math.PI * 0.9)
            );
            catheter.rotate(0, 0);
        }
    }

    let previousCatheterTip = null;
    let previousWireTip = null;
    let maximumLateCatheterTipStep = 0;
    let maximumLateWireTipStep = 0;
    let maximumLateCatheterSpeed = 0;
    let maximumLateWireSpeed = 0;
    let maximumLateCatheterBend = 0;
    let maximumLateNodeEscape = 0;
    let maximumLateSegmentEscape = 0;
    let maximumLateRenderedEscape = 0;
    const idleSteps = Number.isFinite(idleStepsOverride)
        ? idleStepsOverride
        : catheterType === 'pigtail'
            ? Number(process.env.PIGTAIL_TEST_IDLE_STEPS ?? 600)
            : 360;
    const lateIdleStart = Math.max(0, idleSteps - 120);
    for (let idleStep = 0; idleStep < idleSteps; idleStep++) {
        const range = step(0, 0);
        const catheterTip = [
            catheterBody.x[catheterBody.activeEnd],
            catheterBody.y[catheterBody.activeEnd],
            catheterBody.z[catheterBody.activeEnd]
        ];
        const wireTip = [
            wireBody.x[wireBody.activeEnd],
            wireBody.y[wireBody.activeEnd],
            wireBody.z[wireBody.activeEnd]
        ];
        if (idleStep >= lateIdleStart) {
            if (previousCatheterTip) {
                const catheterTipStep = Math.hypot(
                    catheterTip[0] - previousCatheterTip[0],
                    catheterTip[1] - previousCatheterTip[1],
                    catheterTip[2] - previousCatheterTip[2]
                );
                maximumLateCatheterTipStep = Math.max(
                    maximumLateCatheterTipStep,
                    catheterTipStep
                );
                maximumLateWireTipStep = Math.max(maximumLateWireTipStep, Math.hypot(
                    wireTip[0] - previousWireTip[0],
                    wireTip[1] - previousWireTip[1],
                    wireTip[2] - previousWireTip[2]
                ));
            }
            maximumLateCatheterSpeed = Math.max(
                maximumLateCatheterSpeed,
                activeBodySpeed(catheterBody)
            );
            maximumLateWireSpeed = Math.max(maximumLateWireSpeed, activeBodySpeed(wireBody));
            maximumLateCatheterBend = Math.max(
                maximumLateCatheterBend,
                maximumBodyBend(
                    catheterBody,
                    catheterBody.collisionStartSegment + 2,
                    catheterBody.activeEnd - 5
                )
            );
            const escape = coupledContainmentEscape(
                wireBody,
                catheterBody,
                range.firstContainedNode,
                range.capturedEndNode
            );
            maximumLateNodeEscape = Math.max(maximumLateNodeEscape, escape.nodeEscape);
            maximumLateSegmentEscape = Math.max(
                maximumLateSegmentEscape,
                escape.segmentEscape
            );
            const lateRenderPoints = buildContainedGuidewireRenderPolyline({
                guidewireNodes: wire.nodes,
                outerBody: catheterBody,
                containment,
                out: renderPolyline
            });
            maximumLateRenderedEscape = Math.max(
                maximumLateRenderedEscape,
                renderedContainmentEscape(lateRenderPoints, catheterBody)
            );
        }
        previousCatheterTip = catheterTip;
        previousWireTip = wireTip;
        if (idleStep % 60 === 59) {
            const debugEscape = coupledContainmentEscape(
                wireBody,
                catheterBody,
                range.firstContainedNode,
                range.capturedEndNode
            );
            console.log('real aorta coupled idle snapshot', idleStep + 1,
                activeBodySpeed(catheterBody).toFixed(3),
                activeBodySpeed(wireBody).toFixed(3),
                debugEscape.nodeEscape.toFixed(3),
                debugEscape.nodeEscapeIndex,
                debugEscape.segmentEscape.toFixed(3),
                debugEscape.segmentEscapeIndex,
                activePenetration(field, catheterBody).toFixed(3),
                catheterBody.projectionVelocityRetention.toFixed(2),
                catheterBody.postStabilizeBending ? 'bend' : 'no-bend');
        }
    }

    console.log('real aorta coupled final guidewire/catheter mm',
        solver.progress.toFixed(2), catheter.progress.toFixed(2));
    console.log('real aorta coupled late catheter/wire tip step mm',
        maximumLateCatheterTipStep.toFixed(4), maximumLateWireTipStep.toFixed(4));
    console.log('real aorta coupled late catheter/wire speed mm/s',
        maximumLateCatheterSpeed.toFixed(4), maximumLateWireSpeed.toFixed(4));
    console.log('real aorta coupled late catheter bend degrees',
        maximumLateCatheterBend.toFixed(2));
    console.log('real aorta coupled node/segment escape mm',
        maximumLateNodeEscape.toFixed(4), maximumLateSegmentEscape.toFixed(4));
    console.log('real aorta coupled rendered escape mm',
        maximumLateRenderedEscape.toFixed(4));
    const finalPortalSegment = Math.max(
        wireBody.activeStart,
        Math.min(wireBody.activeEnd - 1, containment.endNode)
    );
    const finalPortalAngle = portalExitAngleDegrees(
        wireBody,
        finalPortalSegment,
        catheterBody
    );
    const finalPortalWireBend = maximumBodyBend(
        wireBody,
        Math.max(wireBody.activeStart + 1, finalPortalSegment - 2),
        Math.min(wireBody.activeEnd - 1, finalPortalSegment + 3)
    );
    const finalPortalWireSegmentError = maximumBodySegmentError(
        wireBody,
        Math.max(wireBody.activeStart, finalPortalSegment - 2),
        Math.min(wireBody.activeEnd - 1, finalPortalSegment + 3)
    );
    console.log('real aorta coupled portal angle/bend/segment error',
        finalPortalAngle.toFixed(3),
        finalPortalWireBend.toFixed(3),
        finalPortalWireSegmentError.toFixed(4));
    console.log('real aorta coupled runtime stats', JSON.stringify({
        activeWireNodes: wireBody.activeEnd - wireBody.activeStart + 1,
        activeCatheterNodes: catheterBody.activeEnd - catheterBody.activeStart + 1,
        phases: world.getStats().phases,
        bodies: world.getStats().bodies,
        coupledClosurePasses: world.getStats().coupledClosurePasses,
        lengthPolishPasses: world.getStats().lastLengthPolishPasses,
        wallRepairPasses: world.getStats().lastWallRepairPasses,
        contactField: field.getStats()
    }));

    const pigtailRadialRecovery = catheterType === 'pigtail'
        ? pigtailDistalRadialRecovery(catheterBody, catheter)
        : 0;
    const pigtailLoop = catheterType === 'pigtail'
        ? pigtailLoopMetrics(catheterBody)
        : null;
    if (catheterType === 'pigtail' && assertPigtailRecovery) {
        console.log('real aorta Pigtail 212/140 distal radial recovery mm',
            pigtailRadialRecovery.toFixed(3));
        console.log('real aorta Pigtail 212/140 loop metrics', {
            turnDegrees: pigtailLoop.totalTurnDegrees.toFixed(2),
            spanMm: pigtailLoop.maximumSpan.toFixed(3),
            closureMm: pigtailLoop.closureDistance.toFixed(3),
            closureRatio: pigtailLoop.closureRatio.toFixed(3),
            nodes: pigtailLoop.nodeCount,
            closureConstraint: {
                enabled: catheterBody.shapeClosureEnabled,
                start: catheterBody.shapeClosureStart,
                end: catheterBody.shapeClosureEnd,
                distance: catheterBody.shapeClosureDistance,
                actual: Math.hypot(
                    catheterBody.x[catheterBody.shapeClosureEnd] -
                        catheterBody.x[catheterBody.shapeClosureStart],
                    catheterBody.y[catheterBody.shapeClosureEnd] -
                        catheterBody.y[catheterBody.shapeClosureStart],
                    catheterBody.z[catheterBody.shapeClosureEnd] -
                        catheterBody.z[catheterBody.shapeClosureStart]
                )
            }
        });
        console.log('real aorta Pigtail 212/140 solver phases', pigtailPhaseMetrics);
        if (process.env.OET_DEBUG_PIGTAIL === '1') {
        console.log('real aorta Pigtail 212/140 centerline locations', {
            sheathLength: vessel.sheath.length,
            guideTip: nearestCenterlineInfo(
                tree,
                wireBody.x[wireBody.activeEnd],
                wireBody.y[wireBody.activeEnd],
                wireBody.z[wireBody.activeEnd]
            ),
            base: nearestCenterlineInfo(
                tree,
                catheterBody.x[pigtailLoop.baseIndex],
                catheterBody.y[pigtailLoop.baseIndex],
                catheterBody.z[pigtailLoop.baseIndex]
            ),
            tip: nearestCenterlineInfo(
                tree,
                catheterBody.x[catheterBody.activeEnd],
                catheterBody.y[catheterBody.activeEnd],
                catheterBody.z[catheterBody.activeEnd]
            )
        });
        console.log('real aorta Pigtail free-node material state', catheter.freeNodes
            .filter(node => node._xpbdIndex >= pigtailLoop.baseIndex)
            .map(node => ({
                index: node._xpbdIndex,
                distance: Number(node.distance.toFixed(2)),
                curl: Number(node.curl.toFixed(3)),
                targetOffset: Number(Math.hypot(
                    node.shapeTarget.x - catheterBody.x[node._xpbdIndex],
                    node.shapeTarget.y - catheterBody.y[node._xpbdIndex],
                    node.shapeTarget.z - catheterBody.z[node._xpbdIndex]
                ).toFixed(2)),
                targetRoute: Number(nearestCenterlineInfo(
                    tree,
                    node.shapeTarget.x,
                    node.shapeTarget.y,
                    node.shapeTarget.z
                ).routeDistance.toFixed(1))
            })));
        const turnDetails = [];
        for (
            let segment = pigtailLoop.baseIndex;
            segment < catheterBody.activeEnd;
            segment++
        ) {
            const incoming = segment - 1;
            const ax = catheterBody.x[segment] - catheterBody.x[incoming];
            const ay = catheterBody.y[segment] - catheterBody.y[incoming];
            const az = catheterBody.z[segment] - catheterBody.z[incoming];
            const bx = catheterBody.x[segment + 1] - catheterBody.x[segment];
            const by = catheterBody.y[segment + 1] - catheterBody.y[segment];
            const bz = catheterBody.z[segment + 1] - catheterBody.z[segment];
            const aLength = Math.max(1e-8, Math.hypot(ax, ay, az));
            const bLength = Math.max(1e-8, Math.hypot(bx, by, bz));
            const dot = Math.max(-1, Math.min(1,
                (ax * bx + ay * by + az * bz) / (aLength * bLength)
            ));
            const crossX = ay * bz - az * by;
            const crossY = az * bx - ax * bz;
            const crossZ = ax * by - ay * bx;
            const signed = Math.atan2(
                catheterBody.restDirectionAxisX[segment] * crossX +
                    catheterBody.restDirectionAxisY[segment] * crossY +
                    catheterBody.restDirectionAxisZ[segment] * crossZ,
                dot * aLength * bLength
            ) * 180 / Math.PI;
            const axisX = catheterBody.restDirectionAxisX[segment];
            const axisY = catheterBody.restDirectionAxisY[segment];
            const axisZ = catheterBody.restDirectionAxisZ[segment];
            const bendX = axisY * az / aLength - axisZ * ay / aLength;
            const bendY = axisZ * ax / aLength - axisX * az / aLength;
            const bendZ = axisX * ay / aLength - axisY * ax / aLength;
            const shapeContact = catheterBody.restShapeEnabled[segment]
                ? field.querySphere({
                    x: catheterBody.restShapeX[segment],
                    y: catheterBody.restShapeY[segment],
                    z: catheterBody.restShapeZ[segment]
                }, catheterBody.nodeRadius[segment])
                : null;
            let shapeSigned = null;
            if (
                catheterBody.restShapeEnabled[incoming] &&
                catheterBody.restShapeEnabled[segment] &&
                catheterBody.restShapeEnabled[segment + 1]
            ) {
                const sax = catheterBody.restShapeX[segment] -
                    catheterBody.restShapeX[incoming];
                const say = catheterBody.restShapeY[segment] -
                    catheterBody.restShapeY[incoming];
                const saz = catheterBody.restShapeZ[segment] -
                    catheterBody.restShapeZ[incoming];
                const sbx = catheterBody.restShapeX[segment + 1] -
                    catheterBody.restShapeX[segment];
                const sby = catheterBody.restShapeY[segment + 1] -
                    catheterBody.restShapeY[segment];
                const sbz = catheterBody.restShapeZ[segment + 1] -
                    catheterBody.restShapeZ[segment];
                shapeSigned = Number((Math.atan2(
                    axisX * (say * sbz - saz * sby) +
                        axisY * (saz * sbx - sax * sbz) +
                        axisZ * (sax * sby - say * sbx),
                    sax * sbx + say * sby + saz * sbz
                ) * 180 / Math.PI).toFixed(1));
            }
            turnDetails.push({
                segment,
                actual: Number(signed.toFixed(1)),
                target: Number((
                    catheterBody.restDirectionTurnAngle[segment] * 180 / Math.PI
                ).toFixed(1)),
                shapeSigned,
                compliance: Number(
                    catheterBody.restDirectionCompliance[segment].toExponential(1)
                ),
                wall: catheterBody.wallActive[segment],
                gap: Number.isFinite(catheterBody.wallGap[segment])
                    ? Number(catheterBody.wallGap[segment].toFixed(2))
                    : null,
                shapeOffset: catheterBody.restShapeEnabled[segment]
                    ? Number(Math.hypot(
                        catheterBody.restShapeX[segment] - catheterBody.x[segment],
                        catheterBody.restShapeY[segment] - catheterBody.y[segment],
                        catheterBody.restShapeZ[segment] - catheterBody.z[segment]
                    ).toFixed(2))
                    : null,
                shapePenetration: shapeContact
                    ? Number(shapeContact.penetration.toFixed(2))
                    : null,
                bendInward: Number((
                    bendX * catheterBody.wallNormalX[segment] +
                    bendY * catheterBody.wallNormalY[segment] +
                    bendZ * catheterBody.wallNormalZ[segment]
                ).toFixed(2))
            });
        }
        console.log('real aorta Pigtail 212/140 signed turns', turnDetails);
        let restShapeCapsulePenetration = 0;
        let restShapeSpan = 0;
        let restShapeClosure = 0;
        for (let segment = pigtailLoop.baseIndex; segment < catheterBody.activeEnd; segment++) {
            const contact = field.queryCapsule(
                {
                    x: catheterBody.restShapeX[segment],
                    y: catheterBody.restShapeY[segment],
                    z: catheterBody.restShapeZ[segment]
                },
                {
                    x: catheterBody.restShapeX[segment + 1],
                    y: catheterBody.restShapeY[segment + 1],
                    z: catheterBody.restShapeZ[segment + 1]
                },
                catheterBody.nodeRadius[segment]
            );
            restShapeCapsulePenetration = Math.max(
                restShapeCapsulePenetration,
                contact.penetration
            );
        }
        for (let index = pigtailLoop.baseIndex + 1; index <= catheterBody.activeEnd; index++) {
            restShapeSpan = Math.max(restShapeSpan, Math.hypot(
                catheterBody.restShapeX[index] - catheterBody.restShapeX[pigtailLoop.baseIndex],
                catheterBody.restShapeY[index] - catheterBody.restShapeY[pigtailLoop.baseIndex],
                catheterBody.restShapeZ[index] - catheterBody.restShapeZ[pigtailLoop.baseIndex]
            ));
        }
        restShapeClosure = Math.hypot(
            catheterBody.restShapeX[catheterBody.activeEnd] - catheterBody.restShapeX[pigtailLoop.baseIndex],
            catheterBody.restShapeY[catheterBody.activeEnd] - catheterBody.restShapeY[pigtailLoop.baseIndex],
            catheterBody.restShapeZ[catheterBody.activeEnd] - catheterBody.restShapeZ[pigtailLoop.baseIndex]
        );
        console.log('real aorta Pigtail rest-shape target', {
            fittedIdealPenetration: catheter._xpbdPigtailShapeFitPenetration,
            capsulePenetration: restShapeCapsulePenetration,
            span: restShapeSpan,
            closure: restShapeClosure,
            closureRatio: restShapeClosure / restShapeSpan
        });

        const targetProbe = { x: 0, y: 0, z: 0 };
        let targetX = catheterBody.x[pigtailLoop.baseIndex];
        let targetY = catheterBody.y[pigtailLoop.baseIndex];
        let targetZ = catheterBody.z[pigtailLoop.baseIndex];
        let directionX = catheterBody.x[pigtailLoop.baseIndex] -
            catheterBody.x[pigtailLoop.baseIndex - 1];
        let directionY = catheterBody.y[pigtailLoop.baseIndex] -
            catheterBody.y[pigtailLoop.baseIndex - 1];
        let directionZ = catheterBody.z[pigtailLoop.baseIndex] -
            catheterBody.z[pigtailLoop.baseIndex - 1];
        let directionLength = Math.max(
            1e-8,
            Math.hypot(directionX, directionY, directionZ)
        );
        directionX /= directionLength;
        directionY /= directionLength;
        directionZ /= directionLength;
        let targetMaximumPenetration = 0;
        let targetOutsideNodes = 0;
        for (
            let segment = pigtailLoop.baseIndex;
            segment < catheterBody.activeEnd;
            segment++
        ) {
            const axisX = catheterBody.restDirectionAxisX[segment];
            const axisY = catheterBody.restDirectionAxisY[segment];
            const axisZ = catheterBody.restDirectionAxisZ[segment];
            const angle = catheterBody.restDirectionTurnAngle[segment];
            const cosine = Math.cos(angle);
            const sine = Math.sin(angle);
            const axial = axisX * directionX +
                axisY * directionY + axisZ * directionZ;
            const nextDirectionX = directionX * cosine +
                (axisY * directionZ - axisZ * directionY) * sine +
                axisX * axial * (1 - cosine);
            const nextDirectionY = directionY * cosine +
                (axisZ * directionX - axisX * directionZ) * sine +
                axisY * axial * (1 - cosine);
            const nextDirectionZ = directionZ * cosine +
                (axisX * directionY - axisY * directionX) * sine +
                axisZ * axial * (1 - cosine);
            directionLength = Math.max(
                1e-8,
                Math.hypot(nextDirectionX, nextDirectionY, nextDirectionZ)
            );
            directionX = nextDirectionX / directionLength;
            directionY = nextDirectionY / directionLength;
            directionZ = nextDirectionZ / directionLength;
            const restLength = catheterBody.restLength[segment];
            targetX += directionX * restLength;
            targetY += directionY * restLength;
            targetZ += directionZ * restLength;
            targetProbe.x = targetX;
            targetProbe.y = targetY;
            targetProbe.z = targetZ;
            const contact = field.querySphere(
                targetProbe,
                catheterBody.nodeRadius[segment + 1]
            );
            targetMaximumPenetration = Math.max(
                targetMaximumPenetration,
                contact.penetration
            );
            if (contact.violation) targetOutsideNodes++;
        }
        console.log('real aorta Pigtail intrinsic target wall conflict', {
            maximumPenetration: Number(targetMaximumPenetration.toFixed(3)),
            outsideNodes: targetOutsideNodes,
            targetTipDistance: Number(Math.hypot(
                targetX - catheterBody.x[pigtailLoop.baseIndex],
                targetY - catheterBody.y[pigtailLoop.baseIndex],
                targetZ - catheterBody.z[pigtailLoop.baseIndex]
            ).toFixed(3))
        });

        const baseDirection = [
            catheterBody.x[pigtailLoop.baseIndex] -
                catheterBody.x[pigtailLoop.baseIndex - 1],
            catheterBody.y[pigtailLoop.baseIndex] -
                catheterBody.y[pigtailLoop.baseIndex - 1],
            catheterBody.z[pigtailLoop.baseIndex] -
                catheterBody.z[pigtailLoop.baseIndex - 1]
        ];
        const baseDirectionLength = Math.max(1e-8, Math.hypot(...baseDirection));
        baseDirection[0] /= baseDirectionLength;
        baseDirection[1] /= baseDirectionLength;
        baseDirection[2] /= baseDirectionLength;
        const referenceAxis = [
            catheterBody.restDirectionAxisX[pigtailLoop.baseIndex],
            catheterBody.restDirectionAxisY[pigtailLoop.baseIndex],
            catheterBody.restDirectionAxisZ[pigtailLoop.baseIndex]
        ];
        let bestPlane = null;
        for (let sample = 0; sample < 48; sample++) {
            const rotation = sample * Math.PI * 2 / 48;
            const cosine = Math.cos(rotation);
            const sine = Math.sin(rotation);
            const axisDot = referenceAxis[0] * baseDirection[0] +
                referenceAxis[1] * baseDirection[1] +
                referenceAxis[2] * baseDirection[2];
            const axis = [
                referenceAxis[0] * cosine +
                    (baseDirection[1] * referenceAxis[2] -
                        baseDirection[2] * referenceAxis[1]) * sine +
                    baseDirection[0] * axisDot * (1 - cosine),
                referenceAxis[1] * cosine +
                    (baseDirection[2] * referenceAxis[0] -
                        baseDirection[0] * referenceAxis[2]) * sine +
                    baseDirection[1] * axisDot * (1 - cosine),
                referenceAxis[2] * cosine +
                    (baseDirection[0] * referenceAxis[1] -
                        baseDirection[1] * referenceAxis[0]) * sine +
                    baseDirection[2] * axisDot * (1 - cosine)
            ];
            let candidateX = catheterBody.x[pigtailLoop.baseIndex];
            let candidateY = catheterBody.y[pigtailLoop.baseIndex];
            let candidateZ = catheterBody.z[pigtailLoop.baseIndex];
            let candidateDirection = [...baseDirection];
            let maximumPenetration = 0;
            let squaredPenetration = 0;
            let outsideNodes = 0;
            for (
                let segment = pigtailLoop.baseIndex;
                segment < catheterBody.activeEnd;
                segment++
            ) {
                const turn = catheterBody.restDirectionTurnAngle[segment];
                const turnCosine = Math.cos(turn);
                const turnSine = Math.sin(turn);
                const turnAxial = axis[0] * candidateDirection[0] +
                    axis[1] * candidateDirection[1] +
                    axis[2] * candidateDirection[2];
                const nextDirection = [
                    candidateDirection[0] * turnCosine +
                        (axis[1] * candidateDirection[2] -
                            axis[2] * candidateDirection[1]) * turnSine +
                        axis[0] * turnAxial * (1 - turnCosine),
                    candidateDirection[1] * turnCosine +
                        (axis[2] * candidateDirection[0] -
                            axis[0] * candidateDirection[2]) * turnSine +
                        axis[1] * turnAxial * (1 - turnCosine),
                    candidateDirection[2] * turnCosine +
                        (axis[0] * candidateDirection[1] -
                            axis[1] * candidateDirection[0]) * turnSine +
                        axis[2] * turnAxial * (1 - turnCosine)
                ];
                const nextLength = Math.max(1e-8, Math.hypot(...nextDirection));
                candidateDirection = nextDirection.map(value => value / nextLength);
                const restLength = catheterBody.restLength[segment];
                candidateX += candidateDirection[0] * restLength;
                candidateY += candidateDirection[1] * restLength;
                candidateZ += candidateDirection[2] * restLength;
                targetProbe.x = candidateX;
                targetProbe.y = candidateY;
                targetProbe.z = candidateZ;
                const contact = field.querySphere(
                    targetProbe,
                    catheterBody.nodeRadius[segment + 1]
                );
                maximumPenetration = Math.max(maximumPenetration, contact.penetration);
                squaredPenetration += contact.penetration * contact.penetration;
                if (contact.violation) outsideNodes++;
            }
            const score = maximumPenetration * maximumPenetration * 20 +
                squaredPenetration;
            if (!bestPlane || score < bestPlane.score) {
                bestPlane = {
                    score,
                    rotationDegrees: rotation * 180 / Math.PI,
                    maximumPenetration,
                    squaredPenetration,
                    outsideNodes
                };
            }
        }
        console.log('real aorta Pigtail best rotated intrinsic plane', bestPlane);
        }
    }

    const expectedGuidewireProgress = Number.isFinite(guidewireReleaseTarget)
        ? guidewireReleaseTarget
        : guidewireTarget;
    assert.ok(Math.abs(solver.progress - expectedGuidewireProgress) <= 1e-6,
        `the real-aorta fixture must stop the guidewire at ${expectedGuidewireProgress} mm (${solver.progress} mm)`);
    assert.ok(Math.abs(catheter.progress - catheterTarget) <= 1e-6,
        `the real-aorta fixture must stop the catheter at ${catheterTarget} mm (${catheter.progress} mm)`);
    if (catheterType === 'pigtail' && assertPigtailRecovery) {
        assert.ok(pigtailRadialRecovery >= 10,
            `the released Pigtail must recover a wall-loaded hook in the real aorta (${pigtailRadialRecovery} mm radial recovery)`);
        assert.ok(pigtailLoop.totalTurnDegrees >= 270,
            `the wall-loaded Pigtail must retain its distributed intrinsic curvature (${pigtailLoop.totalTurnDegrees} degrees)`);
        assert.equal(catheterBody.shapeClosureEnabled, false,
            'the Pigtail must curl through distributed elasticity, not a base-to-tip closure tether');
    }
    assert.ok(maximumDeployedFeedCatheterTipStep <= 1.501,
        `the deployed coupled catheter must advance without a tip kick (${maximumDeployedFeedCatheterTipStep} mm)`);
    assert.ok(maximumDeployedFeedRenderedWireTipStep <= (
        catheterType === 'pigtail' ? 2 : 4.25
    ),
        `catheter capture must not move the visible guidewire tip by a full material segment (${maximumDeployedFeedRenderedWireTipStep} mm)`);
    assert.ok(maximumFeedCatheterBend <= (catheterType === 'pigtail' ? 40 : 30),
        `the coupled catheter must not form a wave while advancing (${maximumFeedCatheterBend} degrees)`);
    assert.ok(maximumLateCatheterTipStep <= 0.05,
        `the coupled catheter should settle without residual waving (${maximumLateCatheterTipStep} mm)`);
    assert.ok(maximumLateWireTipStep <= 0.05,
        `the coupled guidewire should settle without residual waving (${maximumLateWireTipStep} mm)`);
    assert.ok(maximumLateCatheterSpeed <= 1,
        `the coupled catheter should be overdamped in the real aorta (${maximumLateCatheterSpeed} mm/s)`);
    assert.ok(maximumLateWireSpeed <= 1,
        `the coupled guidewire should be overdamped in the real aorta (${maximumLateWireSpeed} mm/s)`);
    assert.ok(maximumLateCatheterBend <= (catheterType === 'pigtail' ? 40 : 30),
        `the coupled catheter shaft should not form a wave (${maximumLateCatheterBend} degrees)`);
    assert.ok(maximumFeedNodeEscape <= 0.055,
        `spatially captured guidewire nodes must stay in the lumen during feed (${maximumFeedNodeEscape} mm escape)`);
    assert.ok(maximumFeedRenderedEscape <= 0.05,
        `the rendered guidewire must leave through the catheter opening during feed (${maximumFeedRenderedEscape} mm escape)`);
    assert.ok(maximumLateNodeEscape <= 0.05,
        `contained guidewire nodes must remain inside the catheter (${maximumLateNodeEscape} mm escape)`);
    const sparseChordVisibleTolerance = Math.max(
        0,
        PIGTAIL_CATHETER_RENDER_RADIUS_MM - GUIDEWIRE_RENDER_RADIUS_MM -
            (
                PIGTAIL_CATHETER_INNER_RADIUS_MM -
                DEFAULT_TOOL_PROFILES.guidewire.radius
            )
    );
    assert.ok(maximumLateSegmentEscape <= sparseChordVisibleTolerance,
        `a sparse physics chord must remain beneath the rendered catheter wall (${maximumLateSegmentEscape} mm escape)`);
    assert.ok(maximumLateRenderedEscape <= 0.05,
        `contained rendered guidewire segments must remain inside the catheter (${maximumLateRenderedEscape} mm escape)`);
    if (assertPortalContinuity) {
        const physicalExitConeDegrees = Math.atan2(
            Math.max(0, containment.innerRadius - wireBody.radius) * 2,
            containment.portalTransitionLength
        ) * 180 / Math.PI;
        assert.ok(finalPortalAngle <= physicalExitConeDegrees + 0.5,
            `the guidewire must leave in the catheter-tip direction (${finalPortalAngle} degrees; cone ${physicalExitConeDegrees})`);
        assert.ok(finalPortalWireBend <= 30.5,
            `the continuous guidewire must not kink beside the catheter opening (${finalPortalWireBend} degrees)`);
        assert.ok(finalPortalWireSegmentError <= 0.05,
            `the portal must preserve guidewire material length (${finalPortalWireSegmentError} mm error)`);
    }
    catheter.dispose();
}

function exerciseSoloCatheterInAorta(field, tree, vessel, {
    catheterType = 'berenstein',
    targetProgress = null,
    rotationDegrees = 0,
    fixedDt = 1 / 120,
    physicsSpacing = 4,
    withdrawalDistance = 0
} = {}) {
    const wireLength = 280;
    const wireSpacing = 2;
    const wire = new ElasticRod(wireLength / wireSpacing + 1, wireSpacing);
    const sheath = vessel.sheath;
    const axis = new THREE.Vector3(
        sheath.end.x - sheath.start.x,
        sheath.end.y - sheath.start.y,
        sheath.end.z - sheath.start.z
    ).normalize();
    for (let index = 0; index < wire.nodes.length; index++) {
        const distance = index * wireSpacing - wireLength;
        const node = wire.nodes[index];
        node.x = sheath.start.x + axis.x * distance;
        node.y = sheath.start.y + axis.y * distance;
        node.z = sheath.start.z + axis.z * distance;
        node.vx = 0;
        node.vy = 0;
        node.vz = 0;
    }
    const catheter = new PigtailCatheter({
        wire,
        segmentLength: wireSpacing,
        guidewireLength: wireLength,
        tailProgressRef: () => 0,
        vessel,
        maxLength: wireLength,
        physicsSpacing
    });
    catheter.setType(catheterType);
    catheter.setExternalCollisionSolver(true);
    const world = new EndovascularPhysicsWorld({
        contactField: field,
        fixedDt,
        iterations: 6,
        penetrationIterations: 8
    });
    const body = world.createRod(
        'real-aorta-solo-catheter',
        Math.ceil(240 * 4 / physicsSpacing),
        physicsSpacing,
        {
        ...DEFAULT_TOOL_PROFILES.catheter
        }
    );
    let previousTip = null;
    let maximumDeployedTipStep = 0;
    let maximumDeployedTipStepAt = -1;
    let maximumDeployedTipStepProgress = 0;
    let maximumShaftBend = 0;
    let maximumShaftBendIndex = -1;
    let settledPigtailLoop = null;
    const feedSteps = Number.isFinite(targetProgress)
        ? Math.ceil(targetProgress / (52 * fixedDt))
        : Math.ceil(3.5 / fixedDt);
    for (let step = 0; step < feedSteps; step++) {
        catheter.advance(1, fixedDt, 0);
        catheter.stepPhysics(fixedDt, { collisions: false });
        catheter.syncXpbdBody(body);
        world.stepFixed();
        const tip = [body.x[body.activeEnd], body.y[body.activeEnd], body.z[body.activeEnd]];
        if (previousTip && catheter.progress >= sheath.length + 40) {
            const tipStep = Math.hypot(
                tip[0] - previousTip[0],
                tip[1] - previousTip[1],
                tip[2] - previousTip[2]
            );
            if (tipStep > maximumDeployedTipStep) {
                maximumDeployedTipStep = tipStep;
                maximumDeployedTipStepAt = step;
                maximumDeployedTipStepProgress = catheter.progress;
            }
        }
        previousTip = tip;
        const shaftEnd = Math.max(body.activeStart + 1, body.activeEnd - 20);
        for (let index = body.collisionStartSegment + 1; index < shaftEnd; index++) {
            const ax = body.x[index] - body.x[index - 1];
            const ay = body.y[index] - body.y[index - 1];
            const az = body.z[index] - body.z[index - 1];
            const bx = body.x[index + 1] - body.x[index];
            const by = body.y[index + 1] - body.y[index];
            const bz = body.z[index + 1] - body.z[index];
            const denominator = Math.hypot(ax, ay, az) * Math.hypot(bx, by, bz);
            if (denominator < 1e-8) continue;
            const bend = Math.acos(Math.max(-1, Math.min(1,
                (ax * bx + ay * by + az * bz) / denominator
            ))) * 180 / Math.PI;
            if (bend > maximumShaftBend) {
                maximumShaftBend = bend;
                maximumShaftBendIndex = index;
            }
        }
    }

    if (Math.abs(rotationDegrees) > 1e-6) {
        const targetRotation = rotationDegrees * Math.PI / 180;
        const command = Math.sign(targetRotation);
        for (
            let step = 0;
            step < Math.ceil(2 / fixedDt) &&
                Math.abs(catheter.rotation) < Math.abs(targetRotation);
            step++
        ) {
            catheter.rotate(command, fixedDt);
            catheter.advance(0, fixedDt, 0);
            catheter.stepPhysics(fixedDt, { collisions: false });
            catheter.syncXpbdBody(body);
            world.stepFixed();
        }
    }

    let maximumLateTipStep = 0;
    let maximumLateSpeed = 0;
    const idleSteps = Math.ceil(
        (catheterType === 'pigtail' ? 10 : 1.5) / fixedDt
    );
    const lateIdleWindow = Math.ceil(
        (catheterType === 'pigtail' ? 2 : 0.5) / fixedDt
    );
    const lateIdleStart = Math.max(0, idleSteps - lateIdleWindow);
    for (let step = 0; step < idleSteps; step++) {
        catheter.advance(0, fixedDt, 0);
        catheter.stepPhysics(fixedDt, { collisions: false });
        catheter.syncXpbdBody(body);
        world.stepFixed();
        const tip = [body.x[body.activeEnd], body.y[body.activeEnd], body.z[body.activeEnd]];
        if (step >= lateIdleStart) {
            maximumLateTipStep = Math.max(maximumLateTipStep, Math.hypot(
                tip[0] - previousTip[0],
                tip[1] - previousTip[1],
                tip[2] - previousTip[2]
            ));
            maximumLateSpeed = Math.max(maximumLateSpeed, activeBodySpeed(body));
        }
        previousTip = tip;
    }
    const tipRouteDistance = nearestCenterlineDistance(
        tree,
        body.x[body.activeEnd],
        body.y[body.activeEnd],
        body.z[body.activeEnd]
    );
    const penetration = activePenetration(field, body);
    const label = `real aorta solo ${catheterType} ${rotationDegrees}deg ` +
        `${Math.round(1 / fixedDt)}Hz/${physicsSpacing}mm`;
    console.log(`${label} route distance mm`, tipRouteDistance.toFixed(2));
    console.log(`${label} deployed max tip step mm`,
        maximumDeployedTipStep.toFixed(4));
    console.log(`${label} deployed max tip step detail`,
        maximumDeployedTipStepAt,
        maximumDeployedTipStepProgress.toFixed(2));
    console.log(`${label} max shaft bend degrees/index`,
        maximumShaftBend.toFixed(2), maximumShaftBendIndex,
        'collisionStart', body.collisionStartSegment,
        'activeEnd', body.activeEnd);
    console.log(`${label} late max tip step mm`, maximumLateTipStep.toFixed(4));
    console.log(`${label} late max speed mm/s`, maximumLateSpeed.toFixed(4));
    console.log(`${label} settled penetration mm`, penetration.toFixed(4));
    if (catheterType === 'pigtail') {
        const loop = pigtailLoopMetrics(body);
        settledPigtailLoop = loop;
        const stats = world.getStats().bodies.find(entry => entry.id === body.id);
        console.log(`${label} loop`, {
            turnDegrees: loop.totalTurnDegrees.toFixed(2),
            spanMm: loop.maximumSpan.toFixed(3),
            closureRatio: loop.closureRatio.toFixed(3),
            maxMaterialTurnErrorDegrees:
                stats?.maxMaterialTurnErrorDegrees?.toFixed(2),
            rmsMaterialTurnErrorDegrees:
                stats?.rmsMaterialTurnErrorDegrees?.toFixed(2),
            maxMaterialTurnResidualDegrees:
                stats?.maxMaterialTurnResidualDegrees?.toFixed(2),
            maxMaterialTurnResidualNode:
                stats?.maxMaterialTurnResidualNode,
            maxMaterialActualTargetTurnDegrees: [
                stats?.maxMaterialActualTurnDegrees?.toFixed(2),
                stats?.maxMaterialTargetTurnDegrees?.toFixed(2)
            ],
            rmsMaterialTurnResidualDegrees:
                stats?.rmsMaterialTurnResidualDegrees?.toFixed(2),
            kineticEnergy: stats?.kineticEnergy?.toExponential(3)
        });
        assert.ok(loop.totalTurnDegrees >= 270,
            `the solo Pigtail should recover distributed intrinsic curvature (${loop.totalTurnDegrees} degrees)`);
        assert.ok(loop.totalTurnDegrees <= 430,
            `the solo Pigtail should not over-wind past its natural preform (${loop.totalTurnDegrees} degrees)`);
        assert.ok(loop.maximumSpan >= 12 && loop.maximumSpan <= 22,
            `the solo Pigtail should recover its natural loop diameter (${loop.maximumSpan} mm span)`);
        assert.ok(loop.closureRatio <= 0.37,
            `the solo Pigtail should form a closed tail instead of an open hook (${loop.closureRatio} closure ratio)`);
        assert.ok(Number.isFinite(stats?.kineticEnergy) && stats.kineticEnergy <= 0.01,
            `the settled solo Pigtail should have negligible kinetic energy (${stats?.kineticEnergy})`);
        assert.ok(
            Number.isFinite(stats?.maxMaterialTurnResidualDegrees) &&
                stats.maxMaterialTurnResidualDegrees <= 20,
            `the material curvature equilibrium residual should remain bounded (${stats?.maxMaterialTurnResidualDegrees} degrees)`
        );
        assert.ok(
            Number.isFinite(stats?.rmsMaterialTurnResidualDegrees) &&
                stats.rmsMaterialTurnResidualDegrees <= 8,
            `the distributed material curvature should converge (${stats?.rmsMaterialTurnResidualDegrees} degrees RMS)`
        );
    }
    assert.ok(tipRouteDistance >= 45,
        `the solo catheter should enter the real aorta freely (${tipRouteDistance} mm)`);
    assert.ok(maximumDeployedTipStep <= 1.85 * Math.max(1, fixedDt * 120),
        `the solo catheter should not jump while advancing in the real aorta (${maximumDeployedTipStep} mm)`);
    assert.ok(maximumShaftBend <= 30,
        `the solo catheter shaft should not fold into a wave (${maximumShaftBend} degrees)`);
    assert.ok(maximumLateTipStep <= 0.05,
        `the released solo catheter should stop visibly moving (${maximumLateTipStep} mm)`);
    assert.ok(maximumLateSpeed <= 1,
        `the released solo catheter should damp in the real aorta (${maximumLateSpeed} mm/s)`);
    assert.ok(penetration <= 0.08,
        `the settled solo catheter should stay inside the real aorta (${penetration} mm)`);
    if (catheterType === 'pigtail' && withdrawalDistance > 0) {
        const withdrawalTarget = Math.max(
            0,
            catheter.progress - withdrawalDistance
        );
        let maximumWithdrawalTipStep = 0;
        let maximumWithdrawalBend = 0;
        let maximumWithdrawalPenetration = 0;
        let maximumWithdrawalLoopSpan = settledPigtailLoop.maximumSpan;
        let maximumWithdrawalClosureRatio = settledPigtailLoop.closureRatio;
        let previousWithdrawalTip = previousTip;
        while (catheter.progress > withdrawalTarget + 1e-6) {
            catheter.advance(-1, fixedDt, 0);
            catheter.stepPhysics(fixedDt, { collisions: false });
            catheter.syncXpbdBody(body);
            world.stepFixed();
            const tip = [
                body.x[body.activeEnd],
                body.y[body.activeEnd],
                body.z[body.activeEnd]
            ];
            maximumWithdrawalTipStep = Math.max(
                maximumWithdrawalTipStep,
                Math.hypot(
                    tip[0] - previousWithdrawalTip[0],
                    tip[1] - previousWithdrawalTip[1],
                    tip[2] - previousWithdrawalTip[2]
                )
            );
            previousWithdrawalTip = tip;
            const stats = world.getStats();
            maximumWithdrawalBend = Math.max(
                maximumWithdrawalBend,
                stats.bodies.find(entry => entry.id === body.id)
                    .maxBendAngleDegrees
            );
            maximumWithdrawalPenetration = Math.max(
                maximumWithdrawalPenetration,
                activePenetration(field, body)
            );
            const loop = pigtailLoopMetrics(body);
            maximumWithdrawalLoopSpan = Math.max(
                maximumWithdrawalLoopSpan,
                loop.maximumSpan
            );
            maximumWithdrawalClosureRatio = Math.max(
                maximumWithdrawalClosureRatio,
                loop.closureRatio
            );
        }
        let maximumLateWithdrawalTipStep = 0;
        for (let step = 0; step < Math.ceil(2 / fixedDt); step++) {
            catheter.advance(0, fixedDt, 0);
            catheter.stepPhysics(fixedDt, { collisions: false });
            catheter.syncXpbdBody(body);
            world.stepFixed();
            const tip = [
                body.x[body.activeEnd],
                body.y[body.activeEnd],
                body.z[body.activeEnd]
            ];
            if (step >= Math.ceil(1.5 / fixedDt)) {
                maximumLateWithdrawalTipStep = Math.max(
                    maximumLateWithdrawalTipStep,
                    Math.hypot(
                        tip[0] - previousWithdrawalTip[0],
                        tip[1] - previousWithdrawalTip[1],
                        tip[2] - previousWithdrawalTip[2]
                    )
                );
            }
            previousWithdrawalTip = tip;
        }
        const finalRouteDistance = nearestCenterlineDistance(
            tree,
            body.x[body.activeEnd],
            body.y[body.activeEnd],
            body.z[body.activeEnd]
        );
        console.log(`${label} bifurcation withdrawal`, {
            finalProgressMm: catheter.progress.toFixed(2),
            finalRouteDistanceMm: finalRouteDistance.toFixed(2),
            maxTipStepMm: maximumWithdrawalTipStep.toFixed(4),
            lateTipStepMm: maximumLateWithdrawalTipStep.toFixed(4),
            maxBendDegrees: maximumWithdrawalBend.toFixed(2),
            maxPenetrationMm: maximumWithdrawalPenetration.toFixed(4),
            maxLoopSpanMm: maximumWithdrawalLoopSpan.toFixed(3),
            maxClosureRatio: maximumWithdrawalClosureRatio.toFixed(3)
        });
        assert.ok(catheter.progress <= withdrawalTarget + 0.5,
            `the catheter shaft should physically withdraw by ${withdrawalDistance} mm (${catheter.progress} mm final progress)`);
        assert.ok(Math.abs(finalRouteDistance - tipRouteDistance) <= 15,
            `the Pigtail tip should brace locally while its loop opens instead of replaying the insertion route (${tipRouteDistance} -> ${finalRouteDistance} mm)`);
        assert.ok(maximumWithdrawalTipStep <= 1.5,
            `bifurcation withdrawal should remain continuous (${maximumWithdrawalTipStep} mm tip step)`);
        assert.ok(maximumWithdrawalBend <= 45,
            `the Pigtail should open against the bifurcation without folding (${maximumWithdrawalBend} degrees)`);
        assert.ok(maximumWithdrawalPenetration <= 0.08,
            `bifurcation withdrawal should remain intraluminal (${maximumWithdrawalPenetration} mm)`);
        assert.ok(maximumLateWithdrawalTipStep <= 0.05,
            `the withdrawn Pigtail should settle without oscillation (${maximumLateWithdrawalTipStep} mm)`);
        assert.ok(
            maximumWithdrawalLoopSpan >= settledPigtailLoop.maximumSpan * 1.08 ||
                maximumWithdrawalClosureRatio >= 0.45,
            `the bifurcation should physically open the Pigtail loop (${settledPigtailLoop.maximumSpan} -> ${maximumWithdrawalLoopSpan} mm)`
        );
    }
    catheter.dispose();
}

const sourceBytes = fs.readFileSync('res/Aorta_plain.stl');
const assetBytes = fs.readFileSync('res/Aorta_plain.collision.bin');
const asset = decodeCollisionAsset(arrayBufferFromBuffer(assetBytes));
const geometry = new STLLoader().parse(arrayBufferFromBuffer(sourceBytes));
const { vessel } = generateVessel(140, 0);
transformAortaGeometry(geometry, vessel);
geometry.computeBoundingBox();
geometry.boundsTree = new MeshBVH(geometry);
const field = new VesselContactField(asset, {
    fallbackGeometry: geometry,
    ...(process.env.OET_RUNTIME_CONTACT_CONFIG === '1'
        ? {
            bvhValidationDistance: 0.02,
            capsuleBvhValidation: -0.1
        }
        : {})
});
const tree = buildTree(asset, vessel.sheath.end);
assert.ok(tree.rootDistance <= 1, 'centerline tree should connect to the sheath outlet');

const mainLeaf = [...tree.leaves].sort((a, b) => tree.distances[b] - tree.distances[a])[0];
const branchLeaf = [...tree.leaves]
    .filter(node => tree.distances[node] > 500 && tree.radii[node] >= 0.8 && tree.radii[node] <= 2.5)
    .sort((a, b) => tree.radii[a] - tree.radii[b])[0];
assert.ok(Number.isInteger(branchLeaf), 'a real small-vessel branch should be available for regression');

const onlyCoupledAorta = process.env.OET_ONLY_COUPLED_AORTA === '1';
if (!onlyCoupledAorta) {
    exercisePath('aorta-main', field, pathToRoot(tree, mainLeaf), DEFAULT_TOOL_PROFILES.guidewire.radius);
    exercisePath('aorta-small-branch', field, pathToRoot(tree, branchLeaf), DEFAULT_TOOL_PROFILES.guidewire.radius);
}
exerciseCoupledCatheterInAorta(field, vessel, {
    catheterType: process.env.OET_CATHETER_TYPE || 'berenstein',
    guidewireTarget: Number(process.env.OET_GUIDEWIRE_TARGET ?? 400),
    catheterTarget: Number(process.env.OET_CATHETER_TARGET ?? 240),
    fixtureName: 'real-aorta-kirchhoff-400-240',
    idleStepsOverride: 0,
    assertPigtailRecovery: false
});
if (!onlyCoupledAorta) {
    exerciseSoloCatheterInAorta(field, tree, vessel);
    const soloPigtailRotations = new Set([
        0,
        Number(process.env.SOLO_PIGTAIL_ROTATION_DEG ?? -110)
    ]);
    for (const rotationDegrees of soloPigtailRotations) {
        exerciseSoloCatheterInAorta(field, tree, vessel, {
            catheterType: 'pigtail',
            targetProgress: 266,
            rotationDegrees,
            withdrawalDistance: rotationDegrees === -110 ? 55 : 0
        });
    }
}
