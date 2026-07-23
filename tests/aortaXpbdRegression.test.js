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

const sourceBytes = fs.readFileSync('res/Aorta_plain.stl');
const assetBytes = fs.readFileSync('res/Aorta_plain.collision.bin');
const asset = decodeCollisionAsset(arrayBufferFromBuffer(assetBytes));
const geometry = new STLLoader().parse(arrayBufferFromBuffer(sourceBytes));
const { vessel } = generateVessel(140, 0);
transformAortaGeometry(geometry, vessel);
geometry.computeBoundingBox();
geometry.boundsTree = new MeshBVH(geometry);
const field = new VesselContactField(asset, { fallbackGeometry: geometry });
const tree = buildTree(asset, vessel.sheath.end);
assert.ok(tree.rootDistance <= 1, 'centerline tree should connect to the sheath outlet');

const mainLeaf = [...tree.leaves].sort((a, b) => tree.distances[b] - tree.distances[a])[0];
const branchLeaf = [...tree.leaves]
    .filter(node => tree.distances[node] > 500 && tree.radii[node] >= 0.8 && tree.radii[node] <= 2.5)
    .sort((a, b) => tree.radii[a] - tree.radii[b])[0];
assert.ok(Number.isInteger(branchLeaf), 'a real small-vessel branch should be available for regression');

exercisePath('aorta-main', field, pathToRoot(tree, mainLeaf), DEFAULT_TOOL_PROFILES.guidewire.radius);
exercisePath('aorta-small-branch', field, pathToRoot(tree, branchLeaf), DEFAULT_TOOL_PROFILES.guidewire.radius);
