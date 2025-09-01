import * as THREE from 'three';
import { mergeGeometries } from 'three/examples/jsm/utils/BufferGeometryUtils.js';
import { MeshBVH } from 'three-mesh-bvh';

export function vesselToGeometry(vessel, radialSegments = 16) {
    const geoms = [];
    const yAxis = new THREE.Vector3(0, 1, 0);
    for (const seg of vessel.segments || []) {
        const start = new THREE.Vector3(seg.start.x, seg.start.y, seg.start.z);
        const end = new THREE.Vector3(seg.end.x, seg.end.y, seg.end.z);
        const dir = new THREE.Vector3().subVectors(end, start);
        const length = dir.length();
        if (!length) continue;
        const geom = new THREE.CylinderGeometry(seg.radius, seg.radius, length, radialSegments, 1, true);
        const quat = new THREE.Quaternion().setFromUnitVectors(yAxis, dir.clone().normalize());
        geom.applyQuaternion(quat);
        const mid = new THREE.Vector3().addVectors(start, end).multiplyScalar(0.5);
        geom.translate(mid.x, mid.y, mid.z);
        geoms.push(geom);
    }
    if (!geoms.length) return new THREE.BufferGeometry();
    const merged = mergeGeometries(geoms, true);
    merged.computeVertexNormals();
    merged.computeBoundsTree?.();
    if (!merged.boundsTree) {
        merged.boundsTree = new MeshBVH(merged);
    }
    return merged;
}

/**
 * Generates a branched vessel with deterministic parameters.
 * Defaults produce repeatable geometry; modify arguments to change it explicitly.
 * @param {number} branchLength length of each branch in units (default 140)
 * @param {number} branchAngleOffset angle offset in radians for branches (default 0)
 * @param {number|null} sheathLength length of the left-branch sheath (default
 *   half the branch length)
 * @param {number} sheathRadius radius of the left-branch sheath (default 2)
 * The sheath leaves the left branch with a fixed 30° anterior (+Z) angulation
 * and enters at the distal branch end.
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

    // Introducer sheath entering the vessel at a fixed 30° angle toward the
    // anterior (+Z) direction. The angle is measured relative to the left
    // branch's axis so the sheath presses against the vessel wall before
    // reaching the lumen at the distal branch end.

    const branchDir = new THREE.Vector3(
        vessel.left.end.x - vessel.branchPoint.x,
        vessel.left.end.y - vessel.branchPoint.y,
        vessel.left.end.z - vessel.branchPoint.z
    ).normalize();
    const axis = new THREE.Vector3().crossVectors(branchDir, new THREE.Vector3(0, 0, 1));
    if (axis.lengthSq() === 0) axis.set(1, 0, 0);
    axis.normalize();
    const outward = branchDir.clone().applyQuaternion(
        new THREE.Quaternion().setFromAxisAngle(axis, THREE.MathUtils.degToRad(30))
    ).normalize();

    // Default sheath length is shorter than the branch itself and may be
    // overridden by the caller.
    const autoLength = vessel.left.length * 0.5;
    const finalLength = sheathLength == null ? autoLength : sheathLength;
    // Translate the entire sheath upward along +Y by 20 units
    const sheathStart = {
        x: vessel.left.end.x + outward.x * finalLength + 10,
        y: vessel.left.end.y + outward.y * finalLength + 20,
        z: vessel.left.end.z + outward.z * finalLength
    };
    const sheathEnd = {
        x: vessel.left.end.x + 10,
        y: vessel.left.end.y + 20,
        z: vessel.left.end.z
    };

    vessel.sheath = { start: sheathStart, end: sheathEnd, radius: sheathRadius, length: finalLength, isSheath: true };

    // Add a segment for the sheath so the guidewire can traverse it
    vessel.segments.push(vessel.sheath);

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

    const geometry = vesselToGeometry(vessel);
    vessel.geometry = geometry;
    return { vessel, geometry };
}

