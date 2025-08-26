import * as THREE from 'three';
import { Brush, Evaluator, ADDITION } from 'https://unpkg.com/three-bvh-csg@0.0.17/build/index.module.js';
import { Guidewire, setWireStiffness } from './physics/guidewire.js';

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


const wire = new Guidewire(segmentLength, nodeCount, tailStart, leftDir, vessel);

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
setWireStiffness(wireStiffness);
stiffnessSlider.addEventListener('input', e => {
    wireStiffness = parseFloat(e.target.value);
    setWireStiffness(wireStiffness);
});

let carmYaw = 0;
let carmPitch = 0;
let carmRoll = 0;
let carmX = 0;
let carmY = 0;
let carmZ = 0;

function getPivotPoint() {
    // Orbit around the vessel's branch point so the vasculature stays centered
    return new THREE.Vector3(
        vessel.branchPoint.x + carmX,
        vessel.branchPoint.y + carmY,
        vessel.branchPoint.z + carmZ
    );
}

function updateCamera() {
    const pivot = getPivotPoint();
    const offset = new THREE.Vector3().setFromSpherical(
        new THREE.Spherical(cameraRadius, Math.PI / 2 - carmPitch, carmYaw)
    );
    camera.position.copy(pivot).add(offset);
    camera.lookAt(pivot);
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

