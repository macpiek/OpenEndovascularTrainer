import * as THREE from 'three';
import { Brush, Evaluator, ADDITION } from 'https://unpkg.com/three-bvh-csg@0.0.17/build/index.module.js';

const canvas = document.getElementById('sim');
const renderer = new THREE.WebGLRenderer({canvas, antialias: true});
renderer.setSize(window.innerWidth, window.innerHeight);

const scene = new THREE.Scene();
scene.background = new THREE.Color(0x000000);

const cameraRadius = 200;
const camera = new THREE.PerspectiveCamera(45, window.innerWidth / window.innerHeight, 0.1, 1000);
camera.position.set(0, 80, cameraRadius);
scene.add(camera);

const light = new THREE.HemisphereLight(0xffffff, 0x444444, 1);
scene.add(light);

let vesselMaterial = new THREE.MeshStandardMaterial({color: 0x3366ff});
let vesselGroup;

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

function generateVessel() {
    const mainRadius = 20;
    const branchRadius = mainRadius / 2;
    const branchPointY = -80;
    const branchLength = 120 + Math.random() * 40;
    const blend = 40;
    const branchAngleOffset = (Math.random() - 0.5) * Math.PI / 12;

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

    if (vesselGroup) {
        scene.remove(vesselGroup);
    }
    vesselGroup = new THREE.Group();
    const geometry = createBranchingSegment(mainRadius, branchRadius, branchPointY, branchLength, blend, branchAngleOffset);
    const vesselMesh = new THREE.Mesh(geometry, vesselMaterial);
    vesselMesh.material.wireframe = true;
    vesselGroup.add(vesselMesh);

    scene.add(vesselGroup);
    return vessel;
}

const vessel = generateVessel();

function clamp(v, min, max) {
    return Math.min(Math.max(v, min), max);
}

function projectOnSegment(n, seg) {
    const vx = seg.end.x - seg.start.x;
    const vy = seg.end.y - seg.start.y;
    const vz = (seg.end.z || 0) - (seg.start.z || 0);
    const wx = n.x - seg.start.x;
    const wy = n.y - seg.start.y;
    const wz = n.z - (seg.start.z || 0);
    const len2 = vx * vx + vy * vy + vz * vz;
    let t = (wx * vx + wy * vy + wz * vz) / len2;
    t = clamp(t, 0, 1);
    const px = seg.start.x + vx * t;
    const py = seg.start.y + vy * t;
    const pz = (seg.start.z || 0) + vz * t;
    const dx = n.x - px;
    const dy = n.y - py;
    const dz = n.z - pz;
    const dist = Math.sqrt(dx * dx + dy * dy + dz * dz);
    return {px, py, pz, dx, dy, dz, dist};
}

const wallFriction = 0.02;

function clampToVessel(n, affectVelocity = true) {
    let nearest = vessel.segments[0];
    let best = projectOnSegment(n, nearest);
    for (let i = 1; i < vessel.segments.length; i++) {
        const seg = vessel.segments[i];
        const p = projectOnSegment(n, seg);
        if (p.dist < best.dist) {
            best = p;
            nearest = seg;
        }
    }
    const radius = nearest.radius - 1;
    if (best.dist > radius) {
        const inv = 1 / best.dist;
        const nx = best.dx * inv;
        const ny = best.dy * inv;
        const nz = best.dz * inv;
        n.x = best.px + nx * radius;
        n.y = best.py + ny * radius;
        n.z = best.pz + nz * radius;
        if (affectVelocity) {
            const vn = n.vx * nx + n.vy * ny + n.vz * nz;
            n.vx = (n.vx - vn * nx) * (1 - wallFriction);
            n.vy = (n.vy - vn * ny) * (1 - wallFriction);
            n.vz = (n.vz - vn * nz) * (1 - wallFriction);
        }
    }
}

const segmentLength = 12;
const nodeCount = 80;

const leftDir = {
    x: (vessel.branchPoint.x - vessel.left.end.x) / vessel.left.length,
    y: (vessel.branchPoint.y - vessel.left.end.y) / vessel.left.length,
    z: (vessel.branchPoint.z - vessel.left.end.z) / vessel.left.length
};

const tailStart = {
    x: vessel.left.end.x - leftDir.x * segmentLength * (nodeCount - 1),
    y: vessel.left.end.y - leftDir.y * segmentLength * (nodeCount - 1),
    z: vessel.left.end.z - leftDir.z * segmentLength * (nodeCount - 1)
};

class Guidewire {
    constructor(segLen, count, start, dir) {
        this.segmentLength = segLen;
        this.tailStart = start;
        this.dir = dir;
        this.nodes = [];
        for (let i = 0; i < count; i++) {
            const x = vessel.left.end.x - dir.x * segLen * i;
            const y = vessel.left.end.y - dir.y * segLen * i;
            const z = vessel.left.end.z - dir.z * segLen * i;
            this.nodes.push({x, y, z, vx: 0, vy: 0, vz: 0, fx: 0, fy: 0, fz: 0, oldx: x, oldy: y, oldz: z});
        }
        this.tailProgress = 0;
        this.maxInsert = segLen * (count - 1);
    }

    advanceTail(advance, dt) {
        this.tailProgress = clamp(this.tailProgress + advance * 40 * dt, 0, this.maxInsert);
        const tail = this.nodes[this.nodes.length - 1];
        tail.x = this.tailStart.x + this.dir.x * this.tailProgress;
        tail.y = this.tailStart.y + this.dir.y * this.tailProgress;
        tail.z = this.tailStart.z + this.dir.z * this.tailProgress;
        tail.vx = tail.vy = tail.vz = 0;
        if (advance > 0) {
            const tip = this.nodes[0];
            tip.fx += this.dir.x * 500;
            tip.fy += this.dir.y * 500;
            tip.fz += this.dir.z * 500;
        }
    }

    accumulateForces() {
        for (const n of this.nodes) {
            n.fx = n.fy = n.fz = 0;
            n.fx -= n.vx * 2;
            n.fy -= n.vy * 2;
            n.fz -= n.vz * 2;
        }
        const len = this.segmentLength;
        for (let i = 1; i < this.nodes.length; i++) {
            const a = this.nodes[i - 1];
            const b = this.nodes[i];
            let dx = b.x - a.x;
            let dy = b.y - a.y;
            let dz = b.z - a.z;
            const dist = Math.sqrt(dx * dx + dy * dy + dz * dz) || 1;
            const diff = dist - len;
            const k = 200;
            const force = k * diff;
            const inv = 1 / dist;
            const fx = force * dx * inv;
            const fy = force * dy * inv;
            const fz = force * dz * inv;
            a.fx += fx;
            a.fy += fy;
            a.fz += fz;
            b.fx -= fx;
            b.fy -= fy;
            b.fz -= fz;
        }
        for (let i = 1; i < this.nodes.length - 1; i++) {
            const prev = this.nodes[i - 1];
            const curr = this.nodes[i];
            const next = this.nodes[i + 1];
            const mx = (prev.x + next.x) * 0.5;
            const my = (prev.y + next.y) * 0.5;
            const mz = (prev.z + next.z) * 0.5;
            const bx = (mx - curr.x) * wireStiffness * 50;
            const by = (my - curr.y) * wireStiffness * 50;
            const bz = (mz - curr.z) * wireStiffness * 50;
            curr.fx += bx;
            curr.fy += by;
            curr.fz += bz;
        }
    }

    integrate(dt) {
        for (let i = 0; i < this.nodes.length - 1; i++) {
            const n = this.nodes[i];
            n.vx += n.fx * dt;
            n.vy += n.fy * dt;
            n.vz += n.fz * dt;
            n.x += n.vx * dt;
            n.y += n.vy * dt;
            n.z += n.vz * dt;
        }
    }

    solveDistances(iterations = 2) {
        const len = this.segmentLength;
        for (let k = 0; k < iterations; k++) {
            for (let i = 1; i < this.nodes.length; i++) {
                const a = this.nodes[i - 1];
                const b = this.nodes[i];
                let dx = b.x - a.x;
                let dy = b.y - a.y;
                let dz = b.z - a.z;
                const dist = Math.sqrt(dx * dx + dy * dy + dz * dz) || 1;
                const diff = (dist - len) / dist;
                const offx = dx * 0.5 * diff;
                const offy = dy * 0.5 * diff;
                const offz = dz * 0.5 * diff;
                a.x += offx;
                a.y += offy;
                a.z += offz;
                if (i !== this.nodes.length - 1) {
                    b.x -= offx;
                    b.y -= offy;
                    b.z -= offz;
                }
            }
        }
    }

    collide() {
        for (let i = 0; i < this.nodes.length - 1; i++) {
            clampToVessel(this.nodes[i]);
        }
    }

    step(dt, advance) {
        for (const n of this.nodes) {
            n.oldx = n.x;
            n.oldy = n.y;
            n.oldz = n.z;
        }
        this.advanceTail(advance, dt);
        this.accumulateForces();
        this.integrate(dt);
        this.solveDistances(4);
        this.collide();
        for (let i = 0; i < this.nodes.length - 1; i++) {
            const n = this.nodes[i];
            n.vx = (n.x - n.oldx) / dt;
            n.vy = (n.y - n.oldy) / dt;
            n.vz = (n.z - n.oldz) / dt;
        }
    }
}

const wire = new Guidewire(segmentLength, nodeCount, tailStart, leftDir);

let advance = 0;
window.addEventListener('keydown', e => {
    if (e.code === 'KeyW' || e.code === 'ArrowUp') {
        advance = 1;
        e.preventDefault();
    }
    if (e.code === 'KeyS' || e.code === 'ArrowDown') {
        advance = -1;
        e.preventDefault();
    }
});
window.addEventListener('keyup', e => {
    if (['KeyW', 'KeyS', 'ArrowUp', 'ArrowDown'].includes(e.code)) {
        advance = 0;
        e.preventDefault();
    }
});

const stiffnessSlider = document.getElementById('stiffness');
const carmYawSlider = document.getElementById('carmYaw');
const carmPitchSlider = document.getElementById('carmPitch');
const carmRollSlider = document.getElementById('carmRoll');
const carmXSlider = document.getElementById('carmX');
const carmYSlider = document.getElementById('carmY');
const carmZSlider = document.getElementById('carmZ');
const wireframeToggle = document.getElementById('wireframe');

let wireStiffness = parseFloat(stiffnessSlider.value);
stiffnessSlider.addEventListener('input', e => {
    wireStiffness = parseFloat(e.target.value);
});

let carmYaw = 0;
let carmPitch = 0;
let carmRoll = 0;
let carmX = 0;
let carmY = 0;
let carmZ = 0;

function updateCamera() {
    const x = Math.sin(carmYaw) * Math.cos(carmPitch) * cameraRadius;
    const y = -vessel.branchPoint.y + Math.sin(carmPitch) * cameraRadius;
    const z = Math.cos(carmYaw) * Math.cos(carmPitch) * cameraRadius;
    camera.position.set(x + carmX, y + carmY, z + carmZ);
    camera.lookAt(carmX, vessel.branchPoint.y + carmY, carmZ);
    camera.rotation.z = carmRoll;
}
updateCamera();

carmYawSlider.addEventListener('input', e => {
    carmYaw = parseFloat(e.target.value) * Math.PI / 180;
    updateCamera();
});
carmPitchSlider.addEventListener('input', e => {
    carmPitch = parseFloat(e.target.value) * Math.PI / 180;
    updateCamera();
});
carmRollSlider.addEventListener('input', e => {
    carmRoll = parseFloat(e.target.value) * Math.PI / 180;
    updateCamera();
});
carmXSlider.addEventListener('input', e => {
    carmX = parseFloat(e.target.value);
    updateCamera();
});
carmYSlider.addEventListener('input', e => {
    carmY = parseFloat(e.target.value);
    updateCamera();
});
carmZSlider.addEventListener('input', e => {
    carmZ = parseFloat(e.target.value);
    updateCamera();
});

wireframeToggle.addEventListener('change', e => {
    vesselMaterial.wireframe = e.target.checked;
});

const wireMaterial = new THREE.LineBasicMaterial({color: 0xffffff});
const wireGeometry = new THREE.BufferGeometry();
const wirePositions = new Float32Array(nodeCount * 3);
wireGeometry.setAttribute('position', new THREE.BufferAttribute(wirePositions, 3));
const wireMesh = new THREE.Line(wireGeometry, wireMaterial);
scene.add(wireMesh);

function updateWireMesh() {
    for (let i = 0; i < wire.nodes.length; i++) {
        const n = wire.nodes[i];
        wirePositions[i * 3] = n.x;
        wirePositions[i * 3 + 1] = n.y;
        wirePositions[i * 3 + 2] = n.z;
    }
    wireGeometry.attributes.position.needsUpdate = true;
}

let lastTime = performance.now();
function animate(time) {
    const dt = (time - lastTime) / 1000;
    lastTime = time;
    wire.step(dt, advance);
    updateWireMesh();
    renderer.render(scene, camera);
    requestAnimationFrame(animate);
}
requestAnimationFrame(animate);

window.addEventListener('resize', () => {
    const w = window.innerWidth;
    const h = window.innerHeight;
    renderer.setSize(w, h);
    camera.aspect = w / h;
    camera.updateProjectionMatrix();
});

