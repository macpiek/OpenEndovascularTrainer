import * as THREE from 'three';
import { Guidewire, setBendingStiffness, setWallFriction, setNormalDamping, setVelocityDamping } from './physics/guidewire.js';
import { generateVessel } from './vesselGeometry.js';
import { setupCArmControls } from './carm.js';
import { ContrastAgent, getContrastGeometry } from './contrastAgent.js';
import { PatientMonitor } from './patientMonitor.js';
import { initCArmPreview, cArmPreviewGroup, cArmPreviewGantry } from './carmPreview.js';
import { createBoneModel } from './boneModel.js';

const canvas = document.getElementById('sim');
const renderer = new THREE.WebGLRenderer({canvas, antialias: true});
renderer.setSize(window.innerWidth, window.innerHeight);

const scene = new THREE.Scene();
scene.background = new THREE.Color(0x000000);

const monitor = new PatientMonitor(
    document.getElementById('ecgCanvas'),
    document.getElementById('bpCanvas'),
    document.getElementById('hrValue'),
    document.getElementById('bpValue')
);

initCArmPreview();

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
        contrastTexture: { value: previousTarget.texture },
        gray: { value: new THREE.Color(0xC3C3C3) },
        fluoroscopy: { value: false },
        time: { value: 0 },
        noiseLevel: { value: 0.05 },
        boneOpacity: { value: 1.0 }

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
        uniform sampler2D contrastTexture;
        uniform vec3 gray;
        uniform bool fluoroscopy;
        uniform float time;
        uniform float noiseLevel;
        uniform float boneOpacity;
        varying vec2 vUv;

        float random(vec2 st) {
            return fract(sin(dot(st.xy, vec2(12.9898, 78.233)) + time) * 43758.5453123);
        }
        void main() {
            vec4 tex = texture2D(uTexture, vUv);
            if (fluoroscopy) {
                float intensity = tex.r * boneOpacity;
                float noise = random(vUv * 100.0) - 0.5;
                intensity += noise * noiseLevel;
                intensity = clamp(intensity, 0.0, 1.0);
                float contrast = texture2D(contrastTexture, vUv).r;
                vec3 color = gray * (1.0 - intensity) * (1.0 - contrast);
                gl_FragColor = vec4(clamp(color, 0.0, 1.0), 1.0);
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
const boneGroup = createBoneModel();

const { geometry, vessel } = generateVessel(140, 0); // deterministic branch parameters
vesselGroup = new THREE.Group();
const vesselMesh = new THREE.Mesh(geometry, vesselMaterial);
vesselMesh.material.wireframe = true;
vesselGroup.add(vesselMesh);
scene.add(vesselGroup);

boneGroup.position.set(
    vessel.branchPoint.x,
    vessel.branchPoint.y - 60,
    vessel.branchPoint.z
);
scene.add(boneGroup);

const injSegmentSelect = document.getElementById('injSegment');
// Populate injection segment choices
if (injSegmentSelect) {
    vessel.segments.forEach((_, idx) => {
        const opt = document.createElement('option');
        opt.value = idx;
        opt.textContent = `Segment ${idx}`;
        injSegmentSelect.appendChild(opt);
    });
}

const pivot = new THREE.Vector3(
    vessel.branchPoint.x,
    vessel.branchPoint.y - 60,
    vessel.branchPoint.z
);

const contrast = new ContrastAgent(vessel);
let contrastMesh = null;

// Default to sheath injection and hide the segment selector
const sheathIndex = contrast.sheathIndex;
if (injSegmentSelect) {
    injSegmentSelect.value = sheathIndex;
    injSegmentSelect.parentElement.style.display = 'none';
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
    if (e.code === 'KeyC' && fluoroscopy) {
        injecting = true;
        injectTime = 0;
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
const injectButton = document.getElementById('injectContrast');
const stopInjectButton = document.getElementById('stopInjection');
const injRateSlider = document.getElementById('injRate');
const injDurationSlider = document.getElementById('injDuration');
const injVolumeSlider = document.getElementById('injVolume');

let injecting = false;
let injectTime = 0;
let injectDuration = 1; // seconds
let injectRate = 1; // ml per second
let injectVolume = 0; // total ml
let remainingVolume = 0;
let totalDose = 0;
const insertedLength = document.getElementById('insertedLength');
const doseDisplay = document.getElementById('currentDose');
const persistenceSlider = document.getElementById('persistence');
const noiseSlider = document.getElementById('noiseLevel');
const opacityScaleSlider = document.getElementById('opacityScale');

const sliders = [
    bendSlider,
    staticFricSlider,
    kineticFricSlider,
    dampingSlider,
    velDampingSlider,
    persistenceSlider,
    noiseSlider,
    opacityScaleSlider,
    injVolumeSlider,
    injRateSlider,
    injDurationSlider
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

// Toggle visibility of control sections
document.querySelectorAll('.section-header').forEach(header => {
    header.addEventListener('click', () => {
        const content = header.nextElementSibling;
        header.classList.toggle('collapsed');
        if (content) {
            content.classList.toggle('hidden');
        }
    });
});
setupCArmControls(camera, vessel, cameraRadius, cArmPreviewGroup, cArmPreviewGantry);

displayMaterial.uniforms.noiseLevel.value = parseFloat(noiseSlider.value);
noiseSlider.addEventListener('input', e => {
    displayMaterial.uniforms.noiseLevel.value = parseFloat(e.target.value);
});

let opacityScale = parseFloat(opacityScaleSlider.value);
opacityScaleSlider.addEventListener('input', e => {
    opacityScale = parseFloat(e.target.value);
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

let fluoroscopy = true;
vesselGroup.visible = false;
boneGroup.visible = fluoroscopy;
displayMaterial.uniforms.fluoroscopy.value = true;
modeToggle.textContent = 'Wireframe';
modeToggle.addEventListener('click', () => {
    fluoroscopy = !fluoroscopy;
    vesselGroup.visible = !fluoroscopy;
    boneGroup.visible = fluoroscopy;
    displayMaterial.uniforms.fluoroscopy.value = fluoroscopy;
    modeToggle.textContent = fluoroscopy ? 'Wireframe' : 'Fluoroscopy';
    // Render the guidewire in white so it appears black after the fluoroscopy
    // shader inversion.
    wireMaterial.color.set(0xffffff);
});

injectButton.addEventListener('click', () => {
    if (!injecting) {
        injecting = true;
        injectTime = 0;
        injectRate = parseFloat(injRateSlider.value);
        injectDuration = parseFloat(injDurationSlider.value) / 1000;
        injectVolume = parseFloat(injVolumeSlider.value);
        remainingVolume = injectVolume;
        injectButton.disabled = true;
        stopInjectButton.disabled = false;
    }
});

stopInjectButton.addEventListener('click', () => {
    if (injecting) {
        injecting = false;
        remainingVolume = 0;
        stopInjectButton.disabled = true;
    }
});

// Use a white guidewire so the fluoroscopy shader can invert it to black.
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
    if (injecting) {
        const amt = Math.min(injectRate * dt, remainingVolume);
        contrast.inject(amt);
        totalDose += amt;
        doseDisplay.textContent = totalDose.toFixed(1) + ' ml';
        injectTime += dt;
        remainingVolume -= amt;
        if (injectTime >= injectDuration || remainingVolume <= 0) {
            injecting = false;
            stopInjectButton.disabled = true;
        }
    }
    contrast.update(dt);
    if (contrastMesh) {
        scene.remove(contrastMesh);
        contrastMesh = null;
    }
    const contrastGeoms = getContrastGeometry(contrast);
    if (contrastGeoms.length) {
        contrastMesh = new THREE.Group();
        for (const { geometry, concentration } of contrastGeoms) {
            const material = new THREE.MeshBasicMaterial({
                color: 0xffffff,
                transparent: true,
                opacity: Math.min(concentration * opacityScale, 1)
            });
            contrastMesh.add(new THREE.Mesh(geometry, material));
        }
        scene.add(contrastMesh);
    }
    const contrastActive = contrast.isActive() || injecting;
    vesselGroup.visible = contrastActive ? false : !fluoroscopy;
    boneGroup.visible = fluoroscopy;
    injectButton.disabled = contrastActive;
    stopInjectButton.disabled = !injecting;
    monitor.update(dt);
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
        displayMaterial.uniforms.contrastTexture.value = currentTarget.texture;
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

