import * as THREE from 'three';
import { INTRODUCER_SHEATH_RADIUS_MM } from './toolDimensions.js';

/**
 * Generates the lightweight centerline/flow metadata used by the tools.
 * The visible vessel surface and wall collision now come from the STL aorta
 * model, so this object intentionally does not build the old procedural mesh.
 * @param {number} branchLength length of each branch in units (default 140)
 * @param {number} branchAngleOffset angle offset in radians for branches (default 0)
 * @param {number|null} sheathLength length of the introducer sheath (default
 *   half the branch length)
 * @param {number} sheathRadius radius of the sheath in mm (default 6F)
 * @returns {{vessel: object}}
*/
export function generateVessel(
    branchLength = 140,
    branchAngleOffset = 0,
    sheathLength = null,
    sheathRadius = INTRODUCER_SHEATH_RADIUS_MM
) {
    const mainRadius = 20;
    const branchRadius = mainRadius / 2;
    const branchPointY = -300;

    const vessel = {
        radius: mainRadius,
        branchRadius,
        branchPoint: {x: 0, y: branchPointY, z: 0},
        segments: []
    };

    const mainStart = {x: 0, y: 0, z: 0};
    // Keep a minimal centerline graph for flow and tool path metadata.
    const mainEnd = {x: 0, y: branchPointY, z: 0};
    vessel.main = {start: mainStart, end: mainEnd};
    vessel.segments.push({start: mainStart, end: mainEnd, radius: mainRadius});

    // Define straight metadata branches from the bifurcation. The rendered
    // vessel shape is supplied by the STL model.
    function buildBranch(dir) {
        const angle = Math.PI / 6 * dir + branchAngleOffset * dir; // ±30° plus offset
        const end = {
            x: vessel.branchPoint.x + Math.sin(angle) * branchLength,
            y: vessel.branchPoint.y - branchLength,
            z: 0
        };
        const length = branchLength;
        return { angle, end, length };
    }

    vessel.right = buildBranch(1);
    vessel.left = buildBranch(-1);

    // Add branch segments starting exactly at the bifurcation point. The
    // Branch metadata starts at the bifurcation; the visible bifurcation itself
    // is supplied by the imported STL aorta.
    vessel.segments.push({ start: vessel.branchPoint, end: vessel.right.end, radius: branchRadius });
    vessel.segments.push({ start: vessel.branchPoint, end: vessel.left.end, radius: branchRadius });

    // Introducer sheath entering the same iliac branch as before. The tip is
    // placed inside the imported STL lumen rather than at the old cylinder end.
    const sheathTip = { x: -73, y: -383, z: 14 };
    const sheathEntryDir = new THREE.Vector3(0.24, 0.96, -0.21).normalize();

    // Default sheath length is shorter than the branch itself and may be
    // overridden by the caller.
    const autoLength = vessel.left.length * 0.5;
    const finalLength = sheathLength == null ? autoLength : sheathLength;
    const sheathStart = {
        x: sheathTip.x - sheathEntryDir.x * finalLength,
        y: sheathTip.y - sheathEntryDir.y * finalLength,
        z: sheathTip.z - sheathEntryDir.z * finalLength
    };
    const sheathEnd = { ...sheathTip };

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

    // Compute flow direction and speed throughout the graph. Units are model
    // units per second; keep this moderate so contrast visibly washes through
    // the iliac branches instead of disappearing in a single beat.
    const BASE_SPEED = 85;
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

    return { vessel };
}
