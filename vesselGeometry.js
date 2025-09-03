import * as THREE from 'three';
import { MeshBVH } from 'three-mesh-bvh';
import { Brush, Evaluator, ADDITION } from 'three-bvh-csg';

export function vesselToGeometry(vessel, radialSegments = 16) {
    const yAxis = new THREE.Vector3(0, 1, 0);
    const evaluator = new Evaluator();
    let result = null;
    // Track unique endpoints so we can add a small spherical blend at each
    // junction.  Without this the CSG union leaves the bifurcation disconnected
    // from its branches because the open-ended cylinders only meet at their
    // faces and share no volume.
    const nodeMap = new Map();
    const addNode = (p, radius) => {
        const key = `${p.x.toFixed(5)},${p.y.toFixed(5)},${p.z.toFixed(5)}`;
        const r = nodeMap.get(key);
        nodeMap.set(key, r ? Math.max(r, radius) : radius);
    };
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
        const brush = new Brush(geom);
        brush.updateMatrixWorld();
        result = result ? evaluator.evaluate(result, brush, ADDITION) : brush;
        if (seg.isSheath) {
            // Leave the outer sheath entrance open by omitting a blending sphere.
            // Only add a node at the vessel-facing end so it fuses seamlessly with
            // the branch while keeping the external end uncapped.
            addNode(seg.end, seg.radius);
        } else {
            addNode(seg.start, seg.radius);
            addNode(seg.end, seg.radius);
        }
    }
    // Fuse all touching segments by adding a sphere at each node with radius
    // equal to the largest adjoining segment radius. This guarantees the
    // bifurcation and branches share overlapping volume and results in a single
    // connected mesh.
    for (const [key, radius] of nodeMap) {
        const [x, y, z] = key.split(',').map(Number);
        const geom = new THREE.SphereGeometry(radius, radialSegments, radialSegments);
        geom.translate(x, y, z);
        const brush = new Brush(geom);
        brush.updateMatrixWorld();
        result = result ? evaluator.evaluate(result, brush, ADDITION) : brush;
    }
    if (!result) return new THREE.BufferGeometry();
    const merged = result.geometry;
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
    // Extend the primary vessel all the way to the bifurcation point so the
    // branches can attach seamlessly. Previously the main vessel terminated
    // above the bifurcation which left a gap in the generated mesh.
    const mainEnd = {x: 0, y: branchPointY, z: 0};
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

    // Smoothly join each branch to the main vessel without overlapping geometry.
    // Direction pointing downstream from the bifurcation used to define the
    // initial tangent of each branch's joining curve. It's simply a downward
    // vector of length equal to the blend distance.
    const mainDir = { x: 0, y: -blend, z: 0 };

    function addBranchCurve(endPoint) {
        const p0 = vessel.branchPoint;
        const p1 = {
            x: vessel.branchPoint.x + mainDir.x,
            y: vessel.branchPoint.y + mainDir.y,
            z: vessel.branchPoint.z + mainDir.z
        };
        const steps = 24;
        let prev = p0;
        for (let i = 1; i <= steps; i++) {
            const t = i / steps;
            const tt = 1 - t;
            const p = {
                x: tt * tt * p0.x + 2 * tt * t * p1.x + t * t * endPoint.x,
                y: tt * tt * p0.y + 2 * tt * t * p1.y + t * t * endPoint.y,
                z: tt * tt * p0.z + 2 * tt * t * p1.z + t * t * endPoint.z
            };
            // Use the previous step's t value when interpolating the radius so
            // the branch starts with the same radius as the main vessel. The
            // original implementation used the current step's t which resulted
            // in a slightly smaller first segment and left a visible gap where
            // the branches meet the bifurcation.
            const rStep = (i - 1) / (steps - 1);
            const r = mainRadius + (branchRadius - mainRadius) * rStep;
            vessel.segments.push({ start: prev, end: p, radius: r });
            prev = p;
        }
    }

    addBranchCurve(vessel.right.curveEnd);
    vessel.segments.push({ start: vessel.right.curveEnd, end: vessel.right.end, radius: branchRadius });
    addBranchCurve(vessel.left.curveEnd);
    vessel.segments.push({ start: vessel.left.curveEnd, end: vessel.left.end, radius: branchRadius });

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

