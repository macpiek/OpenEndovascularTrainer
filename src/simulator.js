import * as THREE from 'three';
import { ElasticRod } from './physics/elasticRod.js';
import { generateVessel } from './vesselGeometry.js';
import { initUI } from './ui/ui.js';
import { createBoneModel } from './boneModel.js';
import { VoxelContrastAgent, getVoxelMeshes } from './voxelContrastAgent.js';

const canvas = document.getElementById('sim');
const renderer = new THREE.WebGLRenderer({canvas, antialias: true});
renderer.setSize(window.innerWidth, window.innerHeight);

const scene = new THREE.Scene();
scene.background = new THREE.Color(0x000000);

// Separate scene for rendering contrast in fluoroscopy mode
const contrastScene = new THREE.Scene();

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
        gray: { value: new THREE.Color(0xEBEBEB) },
        fluoroscopy: { value: false },
        time: { value: 0 },
        noiseLevel: { value: 0.05 },
        // Lower default bone opacity so bones appear less prominent
        boneOpacity: { value: 0.5 },
        resolution: { value: new THREE.Vector2(window.innerWidth, window.innerHeight) },
        edgeStrength: { value: 1.0 }

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
        uniform vec2 resolution;
        uniform float edgeStrength;
        varying vec2 vUv;

        float random(vec2 st) {
            return fract(sin(dot(st.xy, vec2(12.9898, 78.233)) + time) * 43758.5453123);
        }

        float edgeFactor(vec2 uv) {
            vec2 texel = 1.0 / resolution;
            float tl = texture2D(uTexture, uv + texel * vec2(-1.0, -1.0)).a;
            float t  = texture2D(uTexture, uv + texel * vec2(0.0, -1.0)).a;
            float tr = texture2D(uTexture, uv + texel * vec2(1.0, -1.0)).a;
            float l  = texture2D(uTexture, uv + texel * vec2(-1.0, 0.0)).a;
            float r  = texture2D(uTexture, uv + texel * vec2(1.0, 0.0)).a;
            float bl = texture2D(uTexture, uv + texel * vec2(-1.0, 1.0)).a;
            float b  = texture2D(uTexture, uv + texel * vec2(0.0, 1.0)).a;
            float br = texture2D(uTexture, uv + texel * vec2(1.0, 1.0)).a;
            float gx = -tl - 2.0*l - bl + tr + 2.0*r + br;
            float gy = -tl - 2.0*t - tr + bl + 2.0*b + br;
            return length(vec2(gx, gy));
        }
        void main() {
            vec4 tex = texture2D(uTexture, vUv);
            float edge = edgeFactor(vUv) * edgeStrength;
            if (fluoroscopy) {
                float intensity = tex.r * boneOpacity;
                float noise = random(vUv * 100.0) - 0.5;
                intensity += noise * noiseLevel;
                intensity = clamp(intensity, 0.0, 1.0);
                vec4 cSample = texture2D(contrastTexture, vUv);
                float contrast = clamp((cSample.r + cSample.b) * 2.0, 0.0, 1.0);
                vec3 color = gray * (1.0 - intensity);
                float alpha = clamp(1.0 + edge, 0.0, 1.0);
                gl_FragColor = vec4(mix(color, vec3(0.0), contrast), alpha);
            } else {
                float alpha = clamp(tex.a + edge, 0.0, 1.0);
                gl_FragColor = vec4(tex.rgb, alpha);
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

let vesselMaterial = new THREE.MeshStandardMaterial({color: 0x3366ff});
let vesselGroup;
const { group: skeletonModel, material: boneMaterial } = createBoneModel();

const { geometry, vessel } = generateVessel(140, 0); // deterministic branch parameters
vesselGroup = new THREE.Group();
const vesselMesh = new THREE.Mesh(geometry, vesselMaterial);
vesselMesh.material.wireframe = true;
vesselGroup.add(vesselMesh);
scene.add(vesselGroup);

skeletonModel.position.set(
    vessel.branchPoint.x,
    vessel.branchPoint.y - 60,
    vessel.branchPoint.z - 50 // push bones back so they render behind vessels
);
skeletonModel.renderOrder = -1; // ensure bones draw before vessel geometry
scene.add(skeletonModel);

// Removed injection segment selector UI; default injection remains main vessel
const voxelAgent = new VoxelContrastAgent(vessel, 2, 0.05);
const voxelGroup = new THREE.Group();
scene.add(voxelGroup);

// Default to injecting into the main vessel
const injectSegmentIndex = 0;

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

let injecting = false;
let injectTime = 0;
let injectDuration = 2; // seconds
let injectRate = 2; // ml per second
let injectVolume = 10; // total ml
let remainingVolume = 0;
let totalDose = 0;

// Use a white guidewire so the fluoroscopy shader can invert it to black.
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

function advanceTailInput(advance, dt) {
    tailProgress = Math.max(minInsert, Math.min(maxInsert, tailProgress + advance * 40 * dt));
    const tail = wire.nodes[0];
    tail.x = tailStart.x + wireDir.x * tailProgress;
    tail.y = tailStart.y + wireDir.y * tailProgress;
    tail.z = tailStart.z + wireDir.z * tailProgress;
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
    wireGeometry.computeBoundingSphere();
}

const fixedDt = 1 / 60;
let lastRenderTime = performance.now();

function stepSimulation() {
    advanceTailInput(ui.getAdvance(), fixedDt);
    wire.step(fixedDt);
    wire.collide(vessel, fixedDt);
    const inserted = Math.max(0, tailProgress);
    ui.updateInsertedLength(inserted / 10);

    if (injecting) {
        const amt = Math.min(injectRate * fixedDt, remainingVolume);
        voxelAgent.inject(amt, injectSegmentIndex, false);
        totalDose += amt;
        ui.updateDose(totalDose);
        injectTime += fixedDt;
        remainingVolume -= amt;
        if (injectTime >= injectDuration || remainingVolume <= 0) {
            injecting = false;
            ui.setStopInjectionDisabled(true);
        }
    }
    voxelAgent.update(fixedDt);
    monitor.update(fixedDt);
}

// Run simulation logic independent of rendering to keep it active when the page is hidden.
setInterval(stepSimulation, fixedDt * 1000);

function withTransparentClear(renderer, fn) {
    renderer.setClearColor(0x000000, 0);
    fn();
    renderer.setClearColor(0x000000, 1);
}

function animate(time) {
    const dt = (time - lastRenderTime) / 1000;
    lastRenderTime = time;

    updateWireMesh();
    const voxMeshes = getVoxelMeshes(voxelAgent, 1e-4, !fluoroscopy);

    if (voxelGroup.visible) {
        voxelGroup.clear();
        for (const m of voxMeshes) voxelGroup.add(m);
    }
    if (fluoroscopy && voxelGroup.parent !== contrastScene) {
        scene.remove(voxelGroup);
        contrastScene.add(voxelGroup);
    } else if (!fluoroscopy && voxelGroup.parent !== scene) {
        contrastScene.remove(voxelGroup);
        scene.add(voxelGroup);
    }
    const contrastActive = voxMeshes.length > 0 || injecting;

    vesselGroup.visible = !fluoroscopy;
    skeletonModel.visible = fluoroscopy;
    ui.setInjectButtonDisabled(contrastActive);
    ui.setStopInjectionDisabled(!injecting);
    if (fluoroscopy) {
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
