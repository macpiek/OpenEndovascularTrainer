import * as THREE from 'three';
import { Guidewire, setBendingStiffness, setWallFriction, setNormalDamping, setVelocityDamping } from './physics/guidewire.js';
import { generateVessel } from './vesselGeometry.js';

const canvas = document.getElementById('sim');
const renderer = new THREE.WebGLRenderer({canvas, antialias: true});
renderer.setSize(window.innerWidth, window.innerHeight);

const scene = new THREE.Scene();
scene.background = new THREE.Color(0x000000);

const cameraRadius = 350;
const camera = new THREE.PerspectiveCamera(45, window.innerWidth / window.innerHeight, 0.1, 1000);
camera.position.set(0, 80, cameraRadius);
scene.add(camera);

const light = new THREE.HemisphereLight(0xffffff, 0x444444, 1);
scene.add(light);

let vesselMaterial = new THREE.MeshStandardMaterial({color: 0x3366ff});
let vesselGroup;

const { geometry, vessel } = generateVessel(140, 0); // deterministic branch parameters
vesselGroup = new THREE.Group();
const vesselMesh = new THREE.Mesh(geometry, vesselMaterial);
vesselMesh.material.wireframe = true;
vesselGroup.add(vesselMesh);
scene.add(vesselGroup);

const segmentLength = 12;
const nodeCount = 80;
const initialWireLength = segmentLength * (nodeCount - 1);
const initialInsert = segmentLength * 3;

const leftDir = {
    x: (vessel.branchPoint.x - vessel.left.end.x) / vessel.left.length,
    y: (vessel.branchPoint.y - vessel.left.end.y) / vessel.left.length,
    z: (vessel.branchPoint.z - vessel.left.end.z) / vessel.left.length
};

const tailStart = {
    x: vessel.left.end.x - leftDir.x * initialWireLength,
    y: vessel.left.end.y - leftDir.y * initialWireLength,
    z: vessel.left.end.z - leftDir.z * initialWireLength
}; // start outside so the tip begins `initialInsert` inside the vessel


const wire = new Guidewire(segmentLength, nodeCount, tailStart, leftDir, vessel, initialWireLength, undefined, undefined, initialInsert, { left: true });

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

const bendSlider = document.getElementById('stiffness');
const staticFricSlider = document.getElementById('staticFriction');
const kineticFricSlider = document.getElementById('kineticFriction');
const dampingSlider = document.getElementById('normalDamping');
const velDampingSlider = document.getElementById('velocityDamping');
const carmYawSlider = document.getElementById('carmYaw');
const carmPitchSlider = document.getElementById('carmPitch');
const carmRollSlider = document.getElementById('carmRoll');
const carmXSlider = document.getElementById('carmX');
const carmYSlider = document.getElementById('carmY');
const carmZSlider = document.getElementById('carmZ');
const wireframeToggle = document.getElementById('wireframe');

let bendingStiffness = parseFloat(bendSlider.value);
setBendingStiffness(bendingStiffness);
bendSlider.addEventListener('input', e => {
    bendingStiffness = parseFloat(e.target.value);
    setBendingStiffness(bendingStiffness);
});

let staticFriction = parseFloat(staticFricSlider.value);
let kineticFriction = parseFloat(kineticFricSlider.value);
let normalDamping = parseFloat(dampingSlider.value);
let velocityDamping = parseFloat(velDampingSlider.value);
setWallFriction(staticFriction, kineticFriction);
setNormalDamping(normalDamping);
setVelocityDamping(velocityDamping);
staticFricSlider.addEventListener('input', e => {
    staticFriction = parseFloat(e.target.value);
    setWallFriction(staticFriction, kineticFriction);
});
kineticFricSlider.addEventListener('input', e => {
    kineticFriction = parseFloat(e.target.value);
    setWallFriction(staticFriction, kineticFriction);
});
dampingSlider.addEventListener('input', e => {
    normalDamping = parseFloat(e.target.value);
    setNormalDamping(normalDamping);
});
velDampingSlider.addEventListener('input', e => {
    velocityDamping = parseFloat(e.target.value);
    setVelocityDamping(velocityDamping);
});

let carmYaw = 0;
let carmPitch = 0;
let carmRoll = 0;
let carmX = 0;
let carmY = -60;
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
const fixedDt = 1 / 60;
let accumulator = 0;
// Maximum physics steps per frame; adjust for browser performance.
let maxSubSteps = 5;

function animate(time) {
    const dt = (time - lastTime) / 1000;
    lastTime = time;

    // Accumulate time and step the physics at a fixed rate.
    accumulator += Math.min(dt, fixedDt * maxSubSteps);
    while (accumulator >= fixedDt) {
        wire.step(fixedDt, advance);
        accumulator -= fixedDt;
    }

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

