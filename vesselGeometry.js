import * as THREE from 'three';
import { Brush, Evaluator, ADDITION } from 'https://unpkg.com/three-bvh-csg@0.0.17/build/index.module.js';

function verifyManifold(geometry) {
    const index = geometry.index;
    if (!index) return 1;
    const count = geometry.attributes.position.count;
    const visited = new Array(count).fill(false);
    const adj = Array.from({length: count}, () => []);
    const arr = index.array;
    for (let i = 0; i < arr.length; i += 3) {
        const a = arr[i], b = arr[i + 1], c = arr[i + 2];
        adj[a].push(b, c);
        adj[b].push(a, c);
        adj[c].push(a, b);
    }
    let components = 0;
    const stack = [];
    for (let i = 0; i < count; i++) {
        if (!visited[i]) {
            components++;
            stack.push(i);
            visited[i] = true;
            while (stack.length) {
                const v = stack.pop();
                for (const n of adj[v]) {
                    if (!visited[n]) {
                        visited[n] = true;
                        stack.push(n);
                    }
                }
            }
        }
    }
    if (components > 1) {
        console.warn(`Geometry has ${components} disconnected components`);
    }
    return components;
}

function createTaperedTube(path, tubularSegments, radialSegments, startRadius, endRadius) {
    const geometry = new THREE.TubeGeometry(path, tubularSegments, 1, radialSegments, false);
    const pos = geometry.attributes.position;
    const normals = geometry.attributes.normal;
    const segments = tubularSegments + 1;
    const radials = radialSegments + 1;
    for (let i = 0; i < segments; i++) {
        const t = i / tubularSegments;
        const r = startRadius + (endRadius - startRadius) * t;
        for (let j = 0; j < radials; j++) {
            const idx = i * radials + j;
            pos.setX(idx, pos.getX(idx) + normals.getX(idx) * (r - 1));
            pos.setY(idx, pos.getY(idx) + normals.getY(idx) * (r - 1));
            pos.setZ(idx, pos.getZ(idx) + normals.getZ(idx) * (r - 1));
        }
    }
    pos.needsUpdate = true;
    geometry.computeVertexNormals();
    return geometry;
}

function createBranchingSegment(mainRadius, branchRadius, branchPointY, branchLength, blend, branchAngleOffset) {
    const trunkHeight = Math.abs(branchPointY);
    const trunkGeom = new THREE.CylinderGeometry(mainRadius, mainRadius, trunkHeight, 16, 1, true);
    trunkGeom.translate(0, branchPointY / 2, 0);

    const angleBase = Math.PI / 6;
    const makeCurve = angle => new THREE.QuadraticBezierCurve3(
        new THREE.Vector3(0, branchPointY, 0),
        new THREE.Vector3(Math.sin(angle) * blend, branchPointY - blend, 0),
        new THREE.Vector3(Math.sin(angle) * (blend + branchLength), branchPointY - (blend + branchLength), 0)
    );

    const rightCurve = makeCurve(angleBase + branchAngleOffset);
    const leftCurve = makeCurve(-angleBase - branchAngleOffset);

    const rightGeom = createTaperedTube(rightCurve, 64, 16, mainRadius, branchRadius);
    const leftGeom = createTaperedTube(leftCurve, 64, 16, mainRadius, branchRadius);

    const trunkBrush = new Brush(trunkGeom);
    const rightBrush = new Brush(rightGeom);
    const leftBrush = new Brush(leftGeom);
    trunkBrush.updateMatrixWorld();
    rightBrush.updateMatrixWorld();
    leftBrush.updateMatrixWorld();

    const evaluator = new Evaluator();
    const result1 = evaluator.evaluate(trunkBrush, rightBrush, ADDITION);
    result1.updateMatrixWorld();
    const result = evaluator.evaluate(result1, leftBrush, ADDITION);
    const geometry = result.geometry;
    geometry.computeVertexNormals();
    verifyManifold(geometry);
    return geometry;
}

/**
 * Generates a branched vessel with deterministic parameters.
 * Defaults produce repeatable geometry; modify arguments to change it explicitly.
 * @param {number} branchLength length of each branch in units (default 140)
 * @param {number} branchAngleOffset angle offset in radians for branches (default 0)
 * @param {number|null} sheathLength minimum length of the left-branch sheath; if
 *   omitted or shorter than the required distance it automatically extends to
 *   the branch point
 * @param {number} sheathRadius radius of the left-branch sheath (default 2)
 * The sheath leaves the left branch with a fixed 30° anterior (+Z) angulation.
 * @returns {{vessel: object, geometry: THREE.BufferGeometry}}
*/
export function generateVessel(branchLength = 140, branchAngleOffset = 0, sheathLength = null, sheathRadius = 2) {
    const mainRadius = 20;
    const branchRadius = mainRadius / 2;
    const branchPointY = -300;
    const blend = 40;

    const vessel = {
        radius: mainRadius,
        branchRadius,
        branchPoint: {x: 0, y: branchPointY, z: 0},
        segments: []
    };

    const mainStart = {x: 0, y: 0, z: 0};
    const mainEnd = {x: 0, y: branchPointY + blend, z: 0};
    vessel.main = {start: mainStart, end: mainEnd};
    vessel.segments.push({start: mainStart, end: mainEnd, radius: mainRadius});

    function branch(dir) {
        const angle = Math.PI / 6 * dir + branchAngleOffset * dir;
        const curveEnd = {
            x: Math.sin(angle) * blend,
            y: branchPointY - blend,
            z: 0
        };
        const end = {
            x: Math.sin(angle) * (blend + branchLength),
            y: branchPointY - (blend + branchLength),
            z: 0
        };
        const length = branchLength + blend;
        return {angle, curveEnd, end, length};
    }

    vessel.right = branch(1);
    vessel.left = branch(-1);

    function addCurve(p0, p1, p2) {
        const steps = 24;
        let prev = p0;
        for (let i = 1; i <= steps; i++) {
            const t = i / steps;
            const tt = 1 - t;
            const p = {
                x: tt * tt * p0.x + 2 * tt * t * p1.x + t * t * p2.x,
                y: tt * tt * p0.y + 2 * tt * t * p1.y + t * t * p2.y,
                z: tt * tt * p0.z + 2 * tt * t * p1.z + t * t * p2.z
            };
            const r = mainRadius + (branchRadius - mainRadius) * t;
            vessel.segments.push({start: prev, end: p, radius: r});
            prev = p;
        }
    }

    addCurve(mainEnd, vessel.branchPoint, vessel.right.curveEnd);
    vessel.segments.push({start: vessel.right.curveEnd, end: vessel.right.end, radius: branchRadius});
    addCurve(mainEnd, vessel.branchPoint, vessel.left.curveEnd);
    vessel.segments.push({start: vessel.left.curveEnd, end: vessel.left.end, radius: branchRadius});

    // Introducer sheath extending from an external entry toward the vessel.
    // Ensure it reaches at least to the branch point so the guidewire can
    // transition from outside into the lumen.
    const sheathStart = { x: vessel.left.end.x, y: vessel.left.end.y - 5, z: vessel.left.end.z + 12 };
    const sheathVec = new THREE.Vector3(
        vessel.branchPoint.x - sheathStart.x,
        vessel.branchPoint.y - sheathStart.y,
        vessel.branchPoint.z - sheathStart.z
    );
    const autoLength = sheathVec.length();
    const finalLength = sheathLength == null ? autoLength : Math.max(sheathLength, autoLength);
    const sheathDir = sheathVec.clone().normalize();
    const sheathEnd = {
        x: sheathStart.x + sheathDir.x * finalLength,
        y: sheathStart.y + sheathDir.y * finalLength,
        z: sheathStart.z + sheathDir.z * finalLength
    };
    vessel.sheath = { start: sheathStart, end: sheathEnd, radius: sheathRadius, length: finalLength };

    // Add a segment for the sheath so the guidewire can traverse it
    vessel.segments.push({ start: sheathStart, end: sheathEnd, radius: sheathRadius });

    // Compute segment lengths and volumes
    for (const seg of vessel.segments) {
        const dx = seg.end.x - seg.start.x;
        const dy = seg.end.y - seg.start.y;
        const dz = seg.end.z - seg.start.z;
        const len = Math.sqrt(dx * dx + dy * dy + dz * dz) || 1;
        seg.length = len;
        seg.volume = Math.PI * seg.radius * seg.radius * len;
    }

    // Build nodes mapping unique points to indices
    const nodeMap = new Map();
    const nodes = [];
    function getNode(p) {
        const key = `${p.x.toFixed(5)},${p.y.toFixed(5)},${p.z.toFixed(5)}`;
        if (nodeMap.has(key)) return nodeMap.get(key);
        const idx = nodes.length;
        nodeMap.set(key, idx);
        nodes.push({ position: p, segments: [] });
        return idx;
    }
    vessel.segments.forEach((seg, idx) => {
        seg.startNode = getNode(seg.start);
        seg.endNode = getNode(seg.end);
        nodes[seg.startNode].segments.push(idx);
        nodes[seg.endNode].segments.push(idx);
    });
    vessel.nodes = nodes;

    // Build adjacency list linking each segment to its downstream neighbor(s)
    const segmentGraph = vessel.segments.map(() => []);
    const parents = vessel.segments.map(() => null);
    const epsilon = 1e-6;
    const pointsEqual = (a, b) =>
        Math.abs(a.x - b.x) < epsilon &&
        Math.abs(a.y - b.y) < epsilon &&
        Math.abs(a.z - b.z) < epsilon;
    for (let i = 0; i < vessel.segments.length; i++) {
        for (let j = 0; j < vessel.segments.length; j++) {
            if (i === j) continue;
            if (pointsEqual(vessel.segments[i].end, vessel.segments[j].start)) {
                segmentGraph[i].push(j);
                parents[j] = i;
            }
        }
    }
    vessel.segmentGraph = segmentGraph;
    // Assign parent indices to segments
    for (let i = 0; i < vessel.segments.length; i++) {
        vessel.segments[i].parent = parents[i];
    }

    // Compute flow direction and speed throughout the graph
    // Increased base speed so contrast advects faster through vessels
    const BASE_SPEED = 200; // cm/s default inflow speed
    const flow = {};
    const computeDir = seg => {
        const dx = seg.end.x - seg.start.x;
        const dy = seg.end.y - seg.start.y;
        const dz = seg.end.z - seg.start.z;
        const len = Math.sqrt(dx * dx + dy * dy + dz * dz) || 1;
        return { x: dx / len, y: dy / len, z: dz / len };
    };

    function assignFlow(idx, speed) {
        const seg = vessel.segments[idx];
        const dir = computeDir(seg);
        seg.flowDir = dir;
        seg.flowSpeed = speed;
        const children = segmentGraph[idx];
        flow[idx] = { dir, speed, children };
        if (children.length) {
            let totalRadius = 0;
            for (const c of children) totalRadius += vessel.segments[c].radius;
            for (const c of children) {
                const childSpeed = speed * (vessel.segments[c].radius / totalRadius);
                assignFlow(c, childSpeed);
            }
        }
    }

    for (let i = 0; i < vessel.segments.length; i++) {
        if (parents[i] === null) {
            assignFlow(i, BASE_SPEED);
        }
    }
    vessel.flow = flow;

    const geometry = createBranchingSegment(mainRadius, branchRadius, branchPointY, branchLength, blend, branchAngleOffset);

    // Build geometry for the introducer sheath
    const sheathGeom = new THREE.CylinderGeometry(sheathRadius, sheathRadius, vessel.sheath.length, 16, 1, true);
    const quat = new THREE.Quaternion().setFromUnitVectors(new THREE.Vector3(0, 1, 0), sheathDir);
    sheathGeom.applyQuaternion(quat);
    const mid = new THREE.Vector3(
        sheathStart.x + sheathDir.x * vessel.sheath.length / 2,
        sheathStart.y + sheathDir.y * vessel.sheath.length / 2,
        sheathStart.z + sheathDir.z * vessel.sheath.length / 2
    );
    sheathGeom.translate(mid.x, mid.y, mid.z);

    const evaluator = new Evaluator();
    const vesselBrush = new Brush(geometry);
    vesselBrush.updateMatrixWorld();
    const sheathBrush = new Brush(sheathGeom);
    sheathBrush.updateMatrixWorld();
    const merged = evaluator.evaluate(vesselBrush, sheathBrush, ADDITION);
    merged.updateMatrixWorld();
    const finalGeometry = merged.geometry;
    finalGeometry.computeVertexNormals();
    verifyManifold(finalGeometry);

    return { vessel, geometry: finalGeometry };
}

