import * as THREE from 'three';
import { Guidewire, setBendingStiffness, setWallFriction, setNormalDamping, setVelocityDamping } from './physics/guidewire.js';
import { generateVessel } from './vesselGeometry.js';
import { setupCArmControls } from './carm.js';
import { ContrastAgent, getContrastGeometry } from './contrastAgent.js';

const canvas = document.getElementById('sim');
const renderer = new THREE.WebGLRenderer({canvas, antialias: true});
renderer.setSize(window.innerWidth, window.innerHeight);

const scene = new THREE.Scene();
scene.background = new THREE.Color(0x000000);

const offscreenTarget = new THREE.WebGLRenderTarget(window.innerWidth, window.innerHeight);
const accumulateTarget1 = new THREE.WebGLRenderTarget(window.innerWidth, window.innerHeight);
const accumulateTarget2 = new THREE.WebGLRenderTarget(window.innerWidth, window.innerHeight);
let previousTarget = accumulateTarget1;
let currentTarget = accumulateTarget2;

const postCamera = new THREE.OrthographicCamera(-1, 1, 1, -1, 0, 1);
const quadGeometry = new THREE.PlaneGeometry(2, 2);
const blendMaterial = new THREE.ShaderMaterial({
    uniforms: {
        currentFrame: { value: null },
        previousFrame: { value: null },
        decay: { value: 0.95 }
    },
    vertexShader: `
        varying vec2 vUv;
        void main() {
            vUv = uv;
            gl_Position = vec4(position.xy, 0.0, 1.0);
        }
    `,
    fragmentShader: `
        uniform sampler2D currentFrame;
        uniform sampler2D previousFrame;
        uniform float decay;
        varying vec2 vUv;
        void main() {
            vec4 prev = texture2D(previousFrame, vUv);
            vec4 curr = texture2D(currentFrame, vUv);
            gl_FragColor = curr + prev * decay;
        }
    `
});
const blendQuad = new THREE.Mesh(quadGeometry, blendMaterial);
const blendScene = new THREE.Scene();
blendScene.add(blendQuad);

const displayMaterial = new THREE.ShaderMaterial({
    uniforms: {
        uTexture: { value: previousTarget.texture },
        gray: { value: new THREE.Color(0xC3C3C3) },
        fluoroscopy: { value: false },
        time: { value: 0 },
        noiseLevel: { value: 0.05 }

    },
    vertexShader: `
        varying vec2 vUv;
        void main() {
            vUv = uv;
            gl_Position = vec4(position.xy, 0.0, 1.0);
        }
    `,
    fragmentShader: `
        uniform sampler2D uTexture;
        uniform vec3 gray;
        uniform bool fluoroscopy;
        uniform float time;
        uniform float noiseLevel;
        varying vec2 vUv;

        float random(vec2 st) {
            return fract(sin(dot(st.xy, vec2(12.9898, 78.233)) + time) * 43758.5453123);
        }
        void main() {
            vec4 tex = texture2D(uTexture, vUv);
            if (fluoroscopy) {
                float intensity = tex.r;
                float noise = random(vUv * 100.0) - 0.5;
                intensity += noise * noiseLevel;
                intensity = clamp(intensity, 0.0, 1.0);
                vec3 color = gray * (1.0 - intensity);
                gl_FragColor = vec4(color, 1.0);
            } else {
                gl_FragColor = tex;
            }
        }
    `
});
const displayQuad = new THREE.Mesh(quadGeometry, displayMaterial);
const displayScene = new THREE.Scene();
displayScene.add(displayQuad);

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

const contrast = new ContrastAgent(vessel.segments, 0);
contrast.start();
let contrastMesh = getContrastGeometry(contrast);
if (contrastMesh) {
    contrastMesh.visible = false;
    scene.add(contrastMesh);
}

const segmentLength = 12;
const nodeCount = 80;
const initialWireLength = segmentLength * (nodeCount - 1);
const initialInsert = segmentLength * 10;

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
document.addEventListener('keydown', e => {
    if (e.code === 'KeyW' || e.code === 'ArrowUp') {
        advance = 1;
        e.preventDefault();
    }
    if (e.code === 'KeyS' || e.code === 'ArrowDown') {
        advance = -1;
        e.preventDefault();
    }
}, true);
document.addEventListener('keyup', e => {
    if (['KeyW', 'KeyS', 'ArrowUp', 'ArrowDown'].includes(e.code)) {
        advance = 0;
        e.preventDefault();
    }
}, true);

const bendSlider = document.getElementById('stiffness');
const staticFricSlider = document.getElementById('staticFriction');
const kineticFricSlider = document.getElementById('kineticFriction');
const dampingSlider = document.getElementById('normalDamping');
const velDampingSlider = document.getElementById('velocityDamping');
const modeToggle = document.getElementById('modeToggle');
const insertedLength = document.getElementById('insertedLength');
const persistenceSlider = document.getElementById('persistence');
const noiseSlider = document.getElementById('noiseLevel');

const sliders = [
    bendSlider,
    staticFricSlider,
    kineticFricSlider,
    dampingSlider,
    velDampingSlider,
    persistenceSlider,
    noiseSlider
];
sliders.forEach(s => s.addEventListener('change', () => s.blur()));

// Display current values next to each slider
document.querySelectorAll('#controls input[type="range"]').forEach(slider => {
    const valueLabel = slider.nextElementSibling;
    if (!valueLabel) return;
    const update = () => { valueLabel.textContent = slider.value; };
    update();
    slider.addEventListener('input', update);
});
setupCArmControls(camera, vessel, cameraRadius);

displayMaterial.uniforms.noiseLevel.value = parseFloat(noiseSlider.value);
noiseSlider.addEventListener('input', e => {
    displayMaterial.uniforms.noiseLevel.value = parseFloat(e.target.value);
});

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
let decay = parseFloat(persistenceSlider.value);
setWallFriction(staticFriction, kineticFriction);
setNormalDamping(normalDamping);
setVelocityDamping(velocityDamping);
blendMaterial.uniforms.decay.value = decay;
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
persistenceSlider.addEventListener('input', e => {
    blendMaterial.uniforms.decay.value = parseFloat(e.target.value);
});

let fluoroscopy = false;
modeToggle.addEventListener('click', () => {
    fluoroscopy = !fluoroscopy;
    vesselGroup.visible = !fluoroscopy;
    displayMaterial.uniforms.fluoroscopy.value = fluoroscopy;
    modeToggle.textContent = fluoroscopy ? 'Wireframe' : 'Fluoroscopy';
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
        const inserted = Math.max(0, wire.tailProgress);
        insertedLength.textContent = (inserted / 10).toFixed(1) + ' cm';
    }

    updateWireMesh();
    contrast.update(dt);
    if (contrastMesh) {
        scene.remove(contrastMesh);
        contrastMesh = null;
    }
    if (contrast.isActive()) {
        contrastMesh = getContrastGeometry(contrast);
        if (contrastMesh) {
            scene.add(contrastMesh);
        }
        vesselGroup.visible = false;
    } else {
        vesselGroup.visible = !fluoroscopy;
    }
    if (fluoroscopy) {
        renderer.setRenderTarget(offscreenTarget);
        renderer.clear();
        renderer.render(scene, camera);

        blendMaterial.uniforms.currentFrame.value = offscreenTarget.texture;
        blendMaterial.uniforms.previousFrame.value = previousTarget.texture;

        renderer.setRenderTarget(currentTarget);
        renderer.render(blendScene, postCamera);
        renderer.setRenderTarget(null);

        displayMaterial.uniforms.uTexture.value = currentTarget.texture;
        displayMaterial.uniforms.time.value = time * 0.001;
        renderer.render(displayScene, postCamera);

        const temp = previousTarget;
        previousTarget = currentTarget;
        currentTarget = temp;
    } else {
        renderer.setRenderTarget(null);
        renderer.render(scene, camera);
    }

    requestAnimationFrame(animate);
}
requestAnimationFrame(animate);

window.addEventListener('resize', () => {
    const w = window.innerWidth;
    const h = window.innerHeight;
    renderer.setSize(w, h);
    camera.aspect = w / h;
    camera.updateProjectionMatrix();
    offscreenTarget.setSize(w, h);
    accumulateTarget1.setSize(w, h);
    accumulateTarget2.setSize(w, h);
});

