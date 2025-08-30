import * as THREE from 'three';
import { ElasticRod, setBendingStiffness, setWallFriction, setSmoothingIterations } from './physics/elasticRod.js';
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

// Separate scene for rendering contrast in fluoroscopy mode
const contrastScene = new THREE.Scene();

const monitor = new PatientMonitor(
    document.getElementById('ecgCanvas'),
    document.getElementById('bpCanvas'),
    document.getElementById('hrValue'),
    document.getElementById('bpValue')
);

initCArmPreview();

const offscreenTarget = new THREE.WebGLRenderTarget(window.innerWidth, window.innerHeight);
const contrastTarget = new THREE.WebGLRenderTarget(window.innerWidth, window.innerHeight);
const accumulateTarget1 = new THREE.WebGLRenderTarget(window.innerWidth, window.innerHeight);
const accumulateTarget2 = new THREE.WebGLRenderTarget(window.innerWidth, window.innerHeight);
const frontDepthTarget = new THREE.WebGLRenderTarget(window.innerWidth, window.innerHeight);
const backDepthTarget = new THREE.WebGLRenderTarget(window.innerWidth, window.innerHeight);
const thicknessTarget = new THREE.WebGLRenderTarget(window.innerWidth, window.innerHeight);
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

const depthMaterialFront = new THREE.MeshDepthMaterial({ side: THREE.FrontSide });
const depthMaterialBack = new THREE.MeshDepthMaterial({
    side: THREE.BackSide,
    depthTest: false
});
const thicknessMaterial = new THREE.ShaderMaterial({
    uniforms: {
        frontDepth: { value: frontDepthTarget.texture },
        backDepth: { value: backDepthTarget.texture }
    },
    vertexShader: `
        varying vec2 vUv;
        void main() {
            vUv = uv;
            gl_Position = vec4(position.xy, 0.0, 1.0);
        }
    `,
    fragmentShader: `
        uniform sampler2D frontDepth;
        uniform sampler2D backDepth;
        varying vec2 vUv;
        void main() {
            float front = texture2D(frontDepth, vUv).r;
            float back = texture2D(backDepth, vUv).r;
            float thick = max(back - front, 0.0);
            gl_FragColor = vec4(vec3(thick), 1.0);
        }
    `
});
const thicknessQuad = new THREE.Mesh(quadGeometry, thicknessMaterial);
const thicknessScene = new THREE.Scene();
thicknessScene.add(thicknessQuad);

const displayMaterial = new THREE.ShaderMaterial({
    uniforms: {
        uTexture: { value: previousTarget.texture },
        contrastTexture: { value: contrastTarget.texture },
        gray: { value: new THREE.Color(0xC3C3C3) },
        fluoroscopy: { value: false },
        time: { value: 0 },
        noiseLevel: { value: 0.05 },
        // Lower default bone opacity so bones appear less prominent
        boneOpacity: { value: 0.5 }

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
                vec4 cSample = texture2D(contrastTexture, vUv);
                float contrast = clamp((cSample.r + cSample.b) * 2.0, 0.0, 1.0);
                vec3 color = gray * (1.0 - intensity);
                gl_FragColor = vec4(mix(color, vec3(0.0), contrast), 1.0);
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
const { group: boneGroup, material: boneMaterial } = createBoneModel();
boneMaterial.uniforms.resolution.value.set(window.innerWidth, window.innerHeight);

const { geometry, vessel } = generateVessel(140, 0); // deterministic branch parameters
vesselGroup = new THREE.Group();
const vesselMesh = new THREE.Mesh(geometry, vesselMaterial);
vesselMesh.material.wireframe = true;
vesselGroup.add(vesselMesh);
scene.add(vesselGroup);

boneGroup.position.set(
    vessel.branchPoint.x,
    vessel.branchPoint.y - 60,
    vessel.branchPoint.z - 50 // push bones back so they render behind vessels
);
boneGroup.renderOrder = -1; // ensure bones draw before vessel geometry
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

// Debug toggle to log contrast information
const debugLabel = document.createElement('label');
debugLabel.style.display = 'block';
const debugCheckbox = document.createElement('input');
debugCheckbox.type = 'checkbox';
debugCheckbox.id = 'debugToggle';
debugLabel.appendChild(debugCheckbox);
debugLabel.appendChild(document.createTextNode(' Debug contrast'));
document.getElementById('controls').appendChild(debugLabel);
debugCheckbox.addEventListener('change', e => {
    contrast.debug = e.target.checked;
});

// Default to injecting into the main vessel and hide the segment selector
const injectSegmentIndex = contrast.sheathIndex;
const parentIndex = injectSegmentIndex > 0 ? injectSegmentIndex - 1 : -1;
if (injSegmentSelect) {
    injSegmentSelect.value = injectSegmentIndex;
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


const wire = new ElasticRod(nodeCount, segmentLength);
let tailProgress = initialInsert;
const maxInsert = tailProgress + initialWireLength;
const minInsert = Math.min(tailProgress - initialWireLength, 0);
for (let i = 0; i < wire.nodes.length; i++) {
    const t = tailProgress + initialWireLength - segmentLength * i;
    wire.nodes[i].x = tailStart.x + leftDir.x * t;
    wire.nodes[i].y = tailStart.y + leftDir.y * t;
    wire.nodes[i].z = tailStart.z + leftDir.z * t;
}

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
const smoothIterSlider = document.getElementById('smoothIterations');
const modeToggle = document.getElementById('modeToggle');
const injectButton = document.getElementById('injectContrast');
const stopInjectButton = document.getElementById('stopInjection');
const injRateSlider = document.getElementById('injRate');
const injDurationSlider = document.getElementById('injDuration');
const injVolumeSlider = document.getElementById('injVolume');

let injecting = false;
let injectTime = 0;
let injectDuration = 2; // seconds
let injectRate = 2; // ml per second
let injectVolume = 10; // total ml
let remainingVolume = 0;
let totalDose = 0;
const insertedLength = document.getElementById('insertedLength');
const doseDisplay = document.getElementById('currentDose');
const persistenceSlider = document.getElementById('persistence');
const noiseSlider = document.getElementById('noiseLevel');
const opacityScaleSlider = document.getElementById('opacityScale');
const gainSlider = document.getElementById('gain');
const perfStats = document.getElementById('perfStats');

const sliders = [
    bendSlider,
    staticFricSlider,
    kineticFricSlider,
    smoothIterSlider,
    persistenceSlider,
    noiseSlider,
    opacityScaleSlider,
    gainSlider,
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

let gain = parseFloat(gainSlider.value);
gainSlider.addEventListener('input', e => {
    gain = parseFloat(e.target.value);
});

// Shader material to render contrast agent with additive brightness and
// concentration-based coloring.
const contrastMaterial = new THREE.ShaderMaterial({
    uniforms: {
        opacityScale: { value: Math.min(opacityScale / 100, 1) },
        gain: { value: gain }
    },
    vertexColors: true,
    transparent: true,
    depthWrite: false,
    blending: THREE.AdditiveBlending,
    vertexShader: `
        varying float vConc;
        void main() {
            vConc = color.r; // concentration encoded in vertex color
            gl_Position = projectionMatrix * modelViewMatrix * vec4(position, 1.0);
        }
    `,
    fragmentShader: `
        uniform float opacityScale;
        uniform float gain;
        varying float vConc;

        void main() {
            float intensity = clamp((1.0 - exp(-gain * vConc * opacityScale)) * 2.0, 0.0, 1.0);
            vec3 color = vec3(vConc, 0.0, 1.0 - vConc);
            gl_FragColor = vec4(color * intensity, intensity);
        }
    `
});

let bendingStiffness = parseFloat(bendSlider.value);
setBendingStiffness(bendingStiffness);
bendSlider.addEventListener('input', e => {
    bendingStiffness = parseFloat(e.target.value);
    setBendingStiffness(bendingStiffness);
});

let staticFriction = parseFloat(staticFricSlider.value);
let kineticFriction = parseFloat(kineticFricSlider.value);
let decay = parseFloat(persistenceSlider.value);
let smoothingIterations = parseInt(smoothIterSlider.value);
setWallFriction(staticFriction, kineticFriction);
setSmoothingIterations(smoothingIterations);
blendMaterial.uniforms.decay.value = decay;
staticFricSlider.addEventListener('input', e => {
    staticFriction = parseFloat(e.target.value);
    setWallFriction(staticFriction, kineticFriction);
});
kineticFricSlider.addEventListener('input', e => {
    kineticFriction = parseFloat(e.target.value);
    setWallFriction(staticFriction, kineticFriction);
});
smoothIterSlider.addEventListener('input', e => {
    smoothingIterations = parseInt(e.target.value);
    setSmoothingIterations(smoothingIterations);
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
const wireMaterial = new THREE.LineBasicMaterial({
    color: 0xffffff,
    depthTest: false
});
const wireGeometry = new THREE.BufferGeometry();
const wirePositions = new Float32Array(nodeCount * 3);
wireGeometry.setAttribute('position', new THREE.BufferAttribute(wirePositions, 3));
const wireMesh = new THREE.Line(wireGeometry, wireMaterial);
wireMesh.renderOrder = 1; // draw on top of additive bone rendering
scene.add(wireMesh);

function advanceTailInput(advance, dt) {
    tailProgress = Math.max(minInsert, Math.min(maxInsert, tailProgress + advance * 40 * dt));
    const tail = wire.nodes[wire.nodes.length - 1];
    tail.x = tailStart.x + leftDir.x * tailProgress;
    tail.y = tailStart.y + leftDir.y * tailProgress;
    tail.z = tailStart.z + leftDir.z * tailProgress;
    tail.vx = tail.vy = tail.vz = 0;
}

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

function withTransparentClear(renderer, fn) {
    renderer.setClearColor(0x000000, 0);
    fn();
    renderer.setClearColor(0x000000, 1);
}

function animate(time) {
    let dt = (time - lastTime) / 1000;
    lastTime = time;
    // When the tab is inactive, requestAnimationFrame pauses. The next
    // frame reports a very large time delta which caused the patient monitor
    // graphs to jump or flatline when returning. Clamp the timestep so we
    // only advance the simulation by a reasonable amount each frame.
    dt = Math.min(dt, 0.1);

    // Accumulate time and step the physics at a fixed rate.
    accumulator += Math.min(dt, fixedDt * maxSubSteps);
    while (accumulator >= fixedDt) {
        advanceTailInput(advance, fixedDt);
        wire.step(fixedDt);
        wire.collide(vessel, fixedDt);
        accumulator -= fixedDt;
        const inserted = Math.max(0, tailProgress);
        insertedLength.textContent = (inserted / 10).toFixed(1) + ' cm';
    }

    updateWireMesh();
    if (injecting) {
        const amt = Math.min(injectRate * dt, remainingVolume);
        contrast.inject(amt, injectSegmentIndex, false);
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
    if (contrast.debug) {
        const mainConc = contrast.concentration[injectSegmentIndex] / (contrast.volumes[injectSegmentIndex] || 1);
        const parentConc = parentIndex >= 0 ? contrast.concentration[parentIndex] / (contrast.volumes[parentIndex] || 1) : 0;
        console.log(`Main conc: ${mainConc.toFixed(4)}, Parent conc: ${parentConc.toFixed(4)}`);
    }
    if (contrastMesh) {
        contrastMesh.traverse(child => {
            if (child.isMesh) {
                if (child.geometry) child.geometry.dispose();
                if (child.material && child.material !== contrastMaterial) {
                    child.material.dispose();
                }
            }
        });
        scene.remove(contrastMesh);
        contrastScene.remove(contrastMesh);
        contrastMesh = null;
    }
    const contrastGeoms = getContrastGeometry(contrast);
    if (contrastGeoms.length) {
        contrastMesh = new THREE.Group();
        contrastMaterial.uniforms.opacityScale.value = Math.min(opacityScale / 100, 1);
        contrastMaterial.uniforms.gain.value = gain;
        for (const geom of contrastGeoms) {
            contrastMesh.add(new THREE.Mesh(geom, contrastMaterial));
        }
        if (!fluoroscopy) {
            scene.add(contrastMesh);
        } else {
            contrastScene.add(contrastMesh);
        }
    }
    const contrastActive = contrast.isActive() || injecting;
    vesselGroup.visible = contrastActive ? false : !fluoroscopy;
    boneGroup.visible = fluoroscopy;
    injectButton.disabled = contrastActive;
    stopInjectButton.disabled = !injecting;
    monitor.update(dt);
    if (fluoroscopy) {
        const hidden = [];
        for (const child of scene.children) {
            if (child !== boneGroup && !child.isCamera) {
                hidden.push({ obj: child, visible: child.visible });
                child.visible = false;
            }
        }
        scene.overrideMaterial = depthMaterialFront;
        renderer.setRenderTarget(frontDepthTarget);
        renderer.clear();
        renderer.render(scene, camera);
        scene.overrideMaterial = depthMaterialBack;
        renderer.setRenderTarget(backDepthTarget);
        renderer.clear();
        renderer.render(scene, camera);
        scene.overrideMaterial = null;
        renderer.setRenderTarget(null);
        for (const h of hidden) h.obj.visible = h.visible;
        thicknessMaterial.uniforms.frontDepth.value = frontDepthTarget.texture;
        thicknessMaterial.uniforms.backDepth.value = backDepthTarget.texture;
        renderer.setRenderTarget(thicknessTarget);
        renderer.render(thicknessScene, postCamera);
        renderer.setRenderTarget(null);
        boneMaterial.uniforms.thicknessMap.value = thicknessTarget.texture;

        renderer.setRenderTarget(contrastTarget);
        withTransparentClear(renderer, () => {
            renderer.clear();
            renderer.render(contrastScene, camera);
        });

        renderer.setRenderTarget(offscreenTarget);
        renderer.clear();
        renderer.render(scene, camera);

        blendMaterial.uniforms.currentFrame.value = offscreenTarget.texture;
        blendMaterial.uniforms.previousFrame.value = previousTarget.texture;

        renderer.setRenderTarget(currentTarget);
        renderer.render(blendScene, postCamera);
        renderer.setRenderTarget(null);

        displayMaterial.uniforms.uTexture.value = currentTarget.texture;
        displayMaterial.uniforms.contrastTexture.value = contrastTarget.texture;
        displayMaterial.uniforms.time.value = time * 0.001;
        renderer.render(displayScene, postCamera);

        const temp = previousTarget;
        previousTarget = currentTarget;
        currentTarget = temp;
    } else {
        renderer.setRenderTarget(null);
        renderer.render(scene, camera);
    }

    if (perfStats) {
        const fps = (1 / dt).toFixed(1);
        let mem = 'N/A';
        if (performance.memory) {
            mem = (performance.memory.usedJSHeapSize / 1048576).toFixed(1) + ' MB';
        }
        perfStats.textContent = `FPS: ${fps} | Mem: ${mem}`;
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
    contrastTarget.setSize(w, h);
    accumulateTarget1.setSize(w, h);
    accumulateTarget2.setSize(w, h);
    frontDepthTarget.setSize(w, h);
    backDepthTarget.setSize(w, h);
    thicknessTarget.setSize(w, h);
    boneMaterial.uniforms.resolution.value.set(w, h);
});

