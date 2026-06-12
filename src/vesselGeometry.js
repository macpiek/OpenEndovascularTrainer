import * as THREE from 'three';
import { MeshBVH } from 'three-mesh-bvh';
import { Brush, Evaluator, ADDITION } from 'three-bvh-csg';

export function vesselToGeometry(vessel, radialSegments = 24) {
    const yAxis = new THREE.Vector3(0, 1, 0);
    const evaluator = new Evaluator();
    let result = null;
    // Track unique endpoints so we can add a small spherical blend at each
    // junction.  Without this the CSG union leaves the bifurcation disconnected
    // from its branches because the open-ended cylinders only meet at their
    // faces and share no volume.
    const nodeMap = new Map();
    // Track junction connectivity to place additional fillet spheres that
    // smooth the transition at bifurcations (degree >= 3 nodes).
    const junctions = new Map();
    const nodeKey = (p) => `${p.x.toFixed(5)},${p.y.toFixed(5)},${p.z.toFixed(5)}`;
    const addNode = (p, radius) => {
        const key = nodeKey(p);
        const r = nodeMap.get(key);
        nodeMap.set(key, r ? Math.max(r, radius) : radius);
    };
    const recordConn = (pos, dir, radius, isSheath = false) => {
        const key = nodeKey(pos);
        let j = junctions.get(key);
        if (!j) {
            j = { pos: new THREE.Vector3(pos.x, pos.y, pos.z), conns: [], maxR: 0 };
            junctions.set(key, j);
        }
        j.maxR = Math.max(j.maxR, radius);
        j.conns.push({ dir: dir.clone().normalize(), radius, isSheath });
    };
    const addSphere = (pos, radius) => {
        const geom = new THREE.SphereGeometry(radius, radialSegments, radialSegments);
        geom.translate(pos.x, pos.y, pos.z);
        const brush = new Brush(geom);
        brush.updateMatrixWorld();
        result = result ? evaluator.evaluate(result, brush, ADDITION) : brush;
    };
    const lerp = (a, b, t) => a + (b - a) * t;
    for (const seg of vessel.segments || []) {
        const start = new THREE.Vector3(seg.start.x, seg.start.y, seg.start.z);
        const end = new THREE.Vector3(seg.end.x, seg.end.y, seg.end.z);
        const dir = new THREE.Vector3().subVectors(end, start);
        const length = dir.length();
        if (!length) continue;
        // Model each straight segment as an open cylinder aligned to its
        // centerline. Using higher radial resolution improves smoothness.
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
            // Record connectivity for smoothing at the vessel-facing end of the sheath
            const dirToEnd = new THREE.Vector3().subVectors(end, start).normalize();
            recordConn(seg.end, dirToEnd.clone().negate(), seg.radius, true);
        } else {
            addNode(seg.start, seg.radius);
            addNode(seg.end, seg.radius);
            // Record connectivity for potential junction smoothing
            const dirToEnd = new THREE.Vector3().subVectors(end, start).normalize();
            recordConn(seg.start, dirToEnd, seg.radius);
            recordConn(seg.end, dirToEnd.clone().negate(), seg.radius);
        }
    }
    // Fuse all touching segments by adding a sphere at each node with radius
    // equal to the largest adjoining segment radius. This guarantees the
    // bifurcation and branches share overlapping volume and results in a single
    // connected mesh while acting as a simple fillet at junctions for a
    // smoother result.
    for (const [key, radius] of nodeMap) {
        const [x, y, z] = key.split(',').map(Number);
        addSphere(new THREE.Vector3(x, y, z), radius);
    }

    // Add additional smaller fillet spheres along each connected segment near
    // true junctions (degree >= 3). These approximate a rounded Y by gently
    // expanding the union away from the node, reducing creases.
    for (const [key, j] of junctions) {
        if (j.conns.length < 3) continue; // only bifurcations or higher
        const centerR = nodeMap.get(key) ?? j.maxR;
        const vesselConns = j.conns.filter(c => !c.isSheath);
        const branchConns = vesselConns.filter(c => c.radius < centerR);
        if (branchConns.length >= 2) {
            const saddleDir = new THREE.Vector3();
            for (const c of branchConns) saddleDir.add(c.dir);
            if (saddleDir.lengthSq() > 0) {
                saddleDir.normalize();
                addSphere(j.pos.clone().add(saddleDir.multiplyScalar(centerR * 0.45)), centerR * 0.82);
            }
        }

        for (const c of vesselConns) {
            // Skip external sheath entrance; we don't want to bulb it.
            // Blend farther from the branch point and taper toward the child
            // radius so the aorta divides as one rounded volume instead of
            // three cylinder caps meeting at a hard Y seam.
            const isChildBranch = c.radius < centerR;
            const steps = isChildBranch
                ? [
                    [0.35, lerp(centerR, c.radius, 0.22)],
                    [0.75, lerp(centerR, c.radius, 0.55)],
                    [1.15, lerp(centerR, c.radius, 0.85)]
                ]
                : [
                    [0.35, centerR * 0.98],
                    [0.75, centerR * 0.92]
                ];
            for (const [distScale, radius] of steps) {
                const pos = j.pos.clone().add(c.dir.clone().multiplyScalar(centerR * distScale));
                addSphere(pos, radius);
            }
        }
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

    const vessel = {
        radius: mainRadius,
        branchRadius,
        branchPoint: {x: 0, y: branchPointY, z: 0},
        segments: []
    };

    const mainStart = {x: 0, y: 0, z: 0};
    // Extend the primary vessel to the bifurcation point so branches can
    // attach seamlessly. Keeping the model minimal (one main + two branch
    // segments) reduces CSG artifacts and produces a cleaner mesh.
    const mainEnd = {x: 0, y: branchPointY, z: 0};
    vessel.main = {start: mainStart, end: mainEnd};
    vessel.segments.push({start: mainStart, end: mainEnd, radius: mainRadius});

    // Define straight branches from the bifurcation. We rely on a rounded
    // union at the branchPoint (via node spheres in vesselToGeometry) to
    // produce a smooth bifurcation without the complexity of discretized
    // curve segments. This greatly reduces seams and eliminates tiny gaps.
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
    // blending sphere added at this node will smoothly fuse the junction.
    vessel.segments.push({ start: vessel.branchPoint, end: vessel.right.end, radius: branchRadius });
    vessel.segments.push({ start: vessel.branchPoint, end: vessel.left.end, radius: branchRadius });

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

    const geometry = vesselToGeometry(vessel);
    vessel.geometry = geometry;
    return { vessel, geometry };
}
