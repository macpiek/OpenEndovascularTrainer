// Main simulator entry: sets up scenes, physics, rendering passes, and UI.
import * as THREE from 'three';
import { ElasticRod } from './physics/elasticRod.js';
import { generateVessel } from './vesselGeometry.js?v=20260612catheter20';
import { initUI } from './ui/ui.js?v=20260612catheter20';
import { createBoneModel } from './boneModel.js';
import { FlowContrastAgent, updateFlowContrastMesh } from './contrastFlowAgent.js?v=20260612catheter20';
import { PigtailCatheter } from './pigtailCatheter.js?v=20260612catheter20';
import { vertexShader as blendVS, fragmentShader as blendFS } from './shaders/blendShader.js';
import { vertexShader as thicknessVS, fragmentShader as thicknessFS } from './shaders/thicknessShader.js';
import { vertexShader as displayVS, fragmentShader as displayFS } from './shaders/displayShader.js?v=20260612catheter20';

// WebGL renderer attached to the fullscreen canvas
const canvas = document.getElementById('sim');
const renderer = new THREE.WebGLRenderer({canvas, antialias: true});
renderer.setSize(window.innerWidth, window.innerHeight);

// Primary 3D scene (wire, vessels, bones)
const scene = new THREE.Scene();
scene.background = new THREE.Color(0x000000);

// Separate scene for rendering contrast meshes in fluoroscopy mode
const contrastScene = new THREE.Scene();

// Offscreen render targets used by various post-processing passes
const offscreenTarget = new THREE.WebGLRenderTarget(window.innerWidth, window.innerHeight);
const contrastTarget = new THREE.WebGLRenderTarget(window.innerWidth, window.innerHeight);
const accumulateTarget1 = new THREE.WebGLRenderTarget(window.innerWidth, window.innerHeight);
const accumulateTarget2 = new THREE.WebGLRenderTarget(window.innerWidth, window.innerHeight);
const frontDepthTarget = new THREE.WebGLRenderTarget(window.innerWidth, window.innerHeight);
const backDepthTarget = new THREE.WebGLRenderTarget(window.innerWidth, window.innerHeight);
const thicknessTarget = new THREE.WebGLRenderTarget(window.innerWidth, window.innerHeight);
let previousTarget = accumulateTarget1;
let currentTarget = accumulateTarget2;

// Fullscreen post-processing setup
const postCamera = new THREE.OrthographicCamera(-1, 1, 1, -1, 0, 1);
const quadGeometry = new THREE.PlaneGeometry(2, 2);
const blendMaterial = new THREE.ShaderMaterial({
    uniforms: {
        currentFrame: { value: null },
        previousFrame: { value: null },
        decay: { value: 0.95 }
    },
    vertexShader: blendVS,
    fragmentShader: blendFS
});
const blendQuad = new THREE.Mesh(quadGeometry, blendMaterial);
const blendScene = new THREE.Scene();
blendScene.add(blendQuad);

// Depth-only materials used to compute front/back depth for thickness
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
    vertexShader: thicknessVS,
    fragmentShader: thicknessFS
});
const thicknessQuad = new THREE.Mesh(quadGeometry, thicknessMaterial);
const thicknessScene = new THREE.Scene();
thicknessScene.add(thicknessQuad);

const displayMaterial = new THREE.ShaderMaterial({
    uniforms: {
        uTexture: { value: previousTarget.texture },
        contrastTexture: { value: contrastTarget.texture },
        gray: { value: new THREE.Color(0xEBEBEB) },
        fluoroscopy: { value: false },
        time: { value: 0 },
        noiseLevel: { value: 0.05 },
        // Lower default bone opacity so bones appear less prominent
        boneOpacity: { value: 0.5 },
        resolution: { value: new THREE.Vector2(window.innerWidth, window.innerHeight) },
        edgeStrength: { value: 1.0 }

    },
    vertexShader: displayVS,
    fragmentShader: displayFS
});
const displayQuad = new THREE.Mesh(quadGeometry, displayMaterial);
const displayScene = new THREE.Scene();
displayScene.add(displayQuad);

// C-arm configuration: camera acts as X-ray source; detector is simulated in shaders
const cameraRadius = 350;
const camera = new THREE.PerspectiveCamera(45, window.innerWidth / window.innerHeight, 0.1, 1000);
camera.position.set(0, 80, cameraRadius);
scene.add(camera);

// Vessel mesh (procedurally generated)
// Use unlit material so wireframe is visible without lights
let vesselMaterial = new THREE.MeshBasicMaterial({ color: 0x3366ff, wireframe: true });
let vesselGroup;
const { group: skeletonModel, material: boneMaterial } = createBoneModel();

// Generate a deterministic vessel model (branch parameter = 0)
const { geometry, vessel } = generateVessel(140, 0); // deterministic branch parameters
vesselGroup = new THREE.Group();
const vesselMesh = new THREE.Mesh(geometry, vesselMaterial);
vesselGroup.add(vesselMesh);
scene.add(vesselGroup);

skeletonModel.position.set(
    vessel.branchPoint.x,
    vessel.branchPoint.y - 60,
    vessel.branchPoint.z - 50 // push bones back so they render behind vessels
);
skeletonModel.renderOrder = -1; // ensure bones draw before vessel geometry
scene.add(skeletonModel);

// Contrast agent simulation: a centreline flow model with fast core stream,
// slower wall layer, axial dispersion, reflux from the sheath, and downstream
// washout through the vessel graph.
const contrastAgent = new FlowContrastAgent(vessel, 3.5);
const voxelGroup = new THREE.Group();
scene.add(voxelGroup);
let contrastMesh = null;
let contrastMeshCount = 0;
let contrastRenderAccumulator = 0;

// Guidewire physical model (discrete elastic rod)
const segmentLength = 5;
const nodeCount = 100;
const guidewireLength = segmentLength * (nodeCount - 1);

// Direction along the sheath from its outer start toward the vessel
const sheathDirVec = {
    x: vessel.sheath.end.x - vessel.sheath.start.x,
    y: vessel.sheath.end.y - vessel.sheath.start.y,
    z: vessel.sheath.end.z - vessel.sheath.start.z
};
const sheathPath = Math.hypot(sheathDirVec.x, sheathDirVec.y, sheathDirVec.z) || 1;
const wireDir = {
    x: sheathDirVec.x / sheathPath,
    y: sheathDirVec.y / sheathPath,
    z: sheathDirVec.z / sheathPath
};

// Start the tip just inside the sheath entrance
const tipStart = {
    x: vessel.sheath.start.x,
    y: vessel.sheath.start.y,
    z: vessel.sheath.start.z
};

// Position the tail so the wire extends far outside the sheath
const tailStart = {
    x: tipStart.x - wireDir.x * guidewireLength,
    y: tipStart.y - wireDir.y * guidewireLength,
    z: tipStart.z - wireDir.z * guidewireLength
};

// Initialize wire nodes along the sheath axis, tail outside the body
const wire = new ElasticRod(nodeCount, segmentLength);
let tailProgress = 0;
const maxInsert = guidewireLength;
// Prevent withdrawing the wire past the sheath entrance so the tip
// always remains within the sheath.
const minInsert = 0;
for (let i = 0; i < wire.nodes.length; i++) {

    const t = segmentLength * i;

    wire.nodes[i].x = tailStart.x + wireDir.x * t;
    wire.nodes[i].y = tailStart.y + wireDir.y * t;
    wire.nodes[i].z = tailStart.z + wireDir.z * t;
}

// Keep the tail fixed outside the sheath
wire.nodes[0].pinned = true;

// Injection state (managed by UI callbacks)
let injecting = false;
let injectTime = 0;
let injectDuration = 2; // seconds
let injectRate = 2; // ml per second
let injectVolume = 10; // total ml
let remainingVolume = 0;
let totalDose = 0;

// Renderable guidewire: white so the fluoroscopy shader can invert it to black
const wireMaterial = new THREE.LineBasicMaterial({
    color: 0xffffff,
    depthTest: false
});
// Initialize UI after wireMaterial is created so mode toggle can affect it
let fluoroscopy = true;
const ui = initUI({
    camera,
    cameraRadius,
    vessel,
    voxelGroup,
    displayMaterial,
    blendMaterial,
    wireMaterial,
    onStartInjection: ({ rate, duration, volume }) => {
        if (!injecting) {
            injecting = true;
            injectTime = 0;
            injectRate = rate;
            injectDuration = duration;
            injectVolume = volume;
            remainingVolume = injectVolume;
        }
    },
    onStopInjection: () => {
        if (injecting) {
            injecting = false;
            remainingVolume = 0;
        }
    },
    onModeChange: (f) => {
        fluoroscopy = f;
        vesselGroup.visible = !fluoroscopy;
        skeletonModel.visible = fluoroscopy;
        displayMaterial.uniforms.fluoroscopy.value = fluoroscopy;
    },
});
const { monitor } = ui;
const wireGeometry = new THREE.BufferGeometry();
const wirePositions = new Float32Array(nodeCount * 3);
wireGeometry.setAttribute('position', new THREE.BufferAttribute(wirePositions, 3));
const wireMesh = new THREE.Line(wireGeometry, wireMaterial);
wireMesh.renderOrder = 1; // draw on top of additive bone rendering
scene.add(wireMesh);

const pigtailCatheter = new PigtailCatheter({
    wire,
    segmentLength,
    guidewireLength,
    tailProgressRef: () => tailProgress,
    vessel
});
scene.add(pigtailCatheter.mesh);

function advanceTailInput(advance, dt) {
    const nextProgress = Math.max(minInsert, Math.min(maxInsert, tailProgress + advance * 40 * dt));
    const delta = nextProgress - tailProgress;
    tailProgress = nextProgress;
    if (delta !== 0) {
        const dx = wireDir.x * delta;
        const dy = wireDir.y * delta;
        const dz = wireDir.z * delta;
        for (const n of wire.nodes) {
            n.x += dx;
            n.y += dy;
            n.z += dz;
        }
    }
    const tail = wire.nodes[0];
    tail.x = tailStart.x + wireDir.x * tailProgress;
    tail.y = tailStart.y + wireDir.y * tailProgress;
    tail.z = tailStart.z + wireDir.z * tailProgress;
    tail.vx = tail.vy = tail.vz = 0;
}

function updateWireMesh() {
    // Copy simulated node positions into the GPU buffer used by the line
    for (let i = 0; i < wire.nodes.length; i++) {
        const n = wire.nodes[i];
        wirePositions[i * 3] = n.x;
        wirePositions[i * 3 + 1] = n.y;
        wirePositions[i * 3 + 2] = n.z;
    }
    wireGeometry.attributes.position.needsUpdate = true;
    wireGeometry.computeBoundingSphere();
}

const fixedDt = 1 / 60;
let lastRenderTime = performance.now();

function stepSimulation() {
    // Advance input, integrate rod physics, collisions, and update medical monitors
    const advance = ui.getAdvance();
    advanceTailInput(advance, fixedDt);
    wire.step(fixedDt);
    wire.collide(vessel, fixedDt);
    wire.solveConstraints(fixedDt);
    if (advance < 0) {
        wire.releaseFromVesselWall(vessel.segments, 0.08, 2);
        wire.solveConstraints(fixedDt);
        wire.straightenByTension(0.18, 4);
        wire.solveConstraints(fixedDt);
    }
    wire.collide(vessel, fixedDt);
    const inserted = Math.max(0, tailProgress);
    pigtailCatheter.advance(ui.getCatheterAdvance(), fixedDt, inserted);
    pigtailCatheter.rotate(ui.getCatheterRotation(), fixedDt);
    pigtailCatheter.stepPhysics(fixedDt);
    pigtailCatheter.constrainGuidewire(fixedDt);
    wire.solveConstraints(fixedDt);
    pigtailCatheter.constrainGuidewire(fixedDt);
    ui.updateInsertedLength(inserted / 10);
    ui.updateCatheterLength(pigtailCatheter.progress / 10);

    if (injecting) {
        const amt = Math.min(injectRate * fixedDt, remainingVolume);
        contrastAgent.injectThroughSheath(amt, injectRate);
        totalDose += amt;
        ui.updateDose(totalDose);
        injectTime += fixedDt;
        remainingVolume -= amt;
        if (injectTime >= injectDuration || remainingVolume <= 0) {
            injecting = false;
            ui.setStopInjectionDisabled(true);
        }
    }
    contrastAgent.update(fixedDt);
    monitor.update(fixedDt);
}

// Keep simulation ticking even when the tab is hidden (decoupled from rendering)
setInterval(stepSimulation, fixedDt * 1000);

function withTransparentClear(renderer, fn) {
    // Temporarily render with transparent clears (for contrast overlay)
    renderer.setClearColor(0x000000, 0);
    fn();
    renderer.setClearColor(0x000000, 1);
}

function animate(time) {
    // Render loop: updates geometry, handles fluoroscopy accumulation, and UI
    const dt = (time - lastRenderTime) / 1000;
    lastRenderTime = time;

    updateWireMesh();
    pigtailCatheter.updateMesh();
    const contrastShouldRender = injecting || contrastAgent.hasVisibleContrast() || contrastMeshCount > 0;
    if (contrastShouldRender) {
        contrastRenderAccumulator += dt;
        const contrastRenderInterval = injecting ? 1 / 30 : 1 / 24;
        if (!contrastMesh || contrastRenderAccumulator >= contrastRenderInterval) {
            contrastRenderAccumulator = 0;
            const contrastRender = updateFlowContrastMesh(contrastAgent, 0.01, !fluoroscopy, contrastMesh, 6000);
            contrastMeshCount = contrastRender.count;
            if (contrastRender.mesh && contrastRender.mesh !== contrastMesh) {
                if (contrastMesh) voxelGroup.remove(contrastMesh);
                contrastMesh = contrastRender.mesh;
                voxelGroup.add(contrastMesh);
            }
        }
    } else if (contrastMesh) {
        contrastMesh.visible = false;
    }

    if (fluoroscopy && voxelGroup.parent !== contrastScene) {
        scene.remove(voxelGroup);
        contrastScene.add(voxelGroup);
    } else if (!fluoroscopy && voxelGroup.parent !== scene) {
        contrastScene.remove(voxelGroup);
        scene.add(voxelGroup);
    }
    const contrastActive = contrastMeshCount > 0 || injecting || contrastAgent.hasVisibleContrast();

    vesselGroup.visible = !fluoroscopy;
    skeletonModel.visible = fluoroscopy;
    ui.setInjectButtonDisabled(contrastActive);
    ui.setStopInjectionDisabled(!injecting);
    if (fluoroscopy) {
        // Fluoroscopy path:
        // 1) render front/back depth for thickness
        // 2) render contrast to its target with transparent clear
        // 3) render scene to offscreen, accumulate with decay
        // 4) display accumulated + contrast via display shader
        const hidden = [];
        for (const child of scene.children) {
            if (child !== skeletonModel && !child.isCamera) {
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

        // Ping-pong accumulation targets for next frame's persistence
        const temp = previousTarget;
        previousTarget = currentTarget;
        currentTarget = temp;
    } else {
        renderer.setRenderTarget(null);
        renderer.render(scene, camera);
    }

    ui.updatePerfStats(dt);

    requestAnimationFrame(animate);
}
requestAnimationFrame(animate);

window.addEventListener('resize', () => {
    // Keep all targets and shader uniforms in sync with the canvas size
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
    displayMaterial.uniforms.resolution.value.set(w, h);
});
