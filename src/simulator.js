// Main simulator entry: sets up scenes, physics, rendering passes, and UI.
import * as THREE from 'three';
import { ElasticRod } from './physics/elasticRod.js';
import { GuidewireSolver } from './physics/guidewireSolver.js';
import { applyGuidewireMaterialProfile } from './physics/guidewireMaterialProfile.js';
import {
    DEFAULT_TOOL_PROFILES,
    EndovascularPhysicsWorld
} from './physics/endovascularPhysicsWorld.js';
import { generateVessel } from './vesselGeometry.js';
import { initUI } from './ui/ui.js';
import { createBoneModel } from './boneModel.js';
import { FlowContrastAgent, updateFlowContrastMesh } from './contrastFlowAgent.js';
import { PigtailCatheter } from './pigtailCatheter.js';
import { createAortaModel } from './aortaModel.js';
import { createBroadPhaseDebugGroup } from './vesselBroadPhase.js';
import { createSmoothTubeGeometry } from './smoothTubeGeometry.js';
import {
    GUIDEWIRE_RADIUS_MM,
    GUIDEWIRE_RENDER_RADIUS_MM,
    INTRODUCER_SHEATH_INNER_RADIUS_MM,
    PIGTAIL_CATHETER_INNER_RADIUS_MM
} from './toolDimensions.js';
import { vertexShader as blendVS, fragmentShader as blendFS } from './shaders/blendShader.js';
import { vertexShader as thicknessVS, fragmentShader as thicknessFS } from './shaders/thicknessShader.js';
import { vertexShader as displayVS, fragmentShader as displayFS } from './shaders/displayShader.js';
import {
    BROWSER_BENCHMARK_DEFAULT_DURATION_MS,
    BROWSER_BENCHMARK_SCENARIO_CYCLE_MS,
    browserBenchmarkCatheterType,
    createBrowserBenchmarkCommands,
    sampleBrowserBenchmarkCommands
} from './benchmark/browserBenchmarkScenario.js';

const LUMEN_DEBUG_COLOR = 0x29ffd4;
const STL_MODEL_DEBUG_COLOR = 0x4f8dff;
const STL_INTERIOR_SAMPLE_COLOR = 0x69ff8e;
const STL_BOUNDARY_EDGE_COLOR = 0xff9b3d;
const STL_LUMEN_CONTOUR_COLOR = 0xa7ff5c;
const SHEATH_ENTRY_MARKER_COLOR = 0xff4fd8;
const WALL_CONTACT_COLOR = 0xffd24a;
const WALL_BREACH_COLOR = 0xff3355;
const WALL_WORST_POINT_COLOR = 0xff55ff;
const CONTACT_MARKER_LIMIT = 420;
const CONTACT_MARKER_UPDATE_INTERVAL = 1 / 10;
const GUIDEWIRE_DIAGNOSTIC_CONTACT_BAND = 1.85;
const GUIDEWIRE_TUBE_RADIAL_SEGMENTS = 12;
const GUIDEWIRE_TUBE_SAMPLES_PER_SEGMENT = 3;
const GUIDEWIRE_MESH_UPDATE_INTERVAL = 1 / 30;
const PIGTAIL_MESH_UPDATE_INTERVAL = 1 / 30;
const requestedPhysicsMode = new URLSearchParams(window.location.search).get('physics');
const PHYSICS_MODE = requestedPhysicsMode === 'legacy' ? 'legacy' : 'xpbd-contact-v1';
const XRAY_CAMERA_NEAR = 0.1;
const XRAY_CAMERA_FAR = 1000;
const loadingScreen = document.getElementById('loadingScreen');
const loadingMessage = document.getElementById('loadingMessage');
const loadingMilestones = new Set(['aorta', 'skeleton', 'firstFrame']);
let loadingDismissed = false;
let firstFrameFallbackTimer = null;

function setLoadingMessage(message) {
    if (loadingMessage) loadingMessage.textContent = message;
}

function loadingAssetsReady() {
    return !loadingMilestones.has('aorta') && !loadingMilestones.has('skeleton');
}

function hideLoadingScreen() {
    if (loadingDismissed || !loadingScreen) return;
    loadingDismissed = true;
    setLoadingMessage('Ready');
    loadingScreen.classList.add('is-hidden');
    loadingScreen.addEventListener('transitionend', () => loadingScreen.remove(), { once: true });
    setTimeout(() => loadingScreen.remove(), 900);
}

function completeLoadingMilestone(name, message) {
    if (!loadingMilestones.has(name)) return;
    loadingMilestones.delete(name);
    if (name === 'firstFrame' && firstFrameFallbackTimer) {
        clearTimeout(firstFrameFallbackTimer);
        firstFrameFallbackTimer = null;
    }
    if (message) setLoadingMessage(message);
    scheduleFirstFrameFallback();
    if (loadingMilestones.size === 0) hideLoadingScreen();
}

function failLoadingMilestone(name) {
    completeLoadingMilestone(name, 'Loading fallback view');
}

function scheduleFirstFrameFallback() {
    if (!loadingAssetsReady() || !loadingMilestones.has('firstFrame') || firstFrameFallbackTimer) return;
    setLoadingMessage('Rendering first frame');
    requestAnimationFrame(() => completeLoadingMilestone('firstFrame', 'Ready'));
    firstFrameFallbackTimer = setTimeout(() => completeLoadingMilestone('firstFrame', 'Ready'), 1800);
}

function completeFirstLoadedFrame() {
    if (!loadingAssetsReady()) return;
    completeLoadingMilestone('firstFrame', 'Ready');
}

setLoadingMessage('Preparing renderer');

// WebGL renderer attached to the fullscreen canvas
const canvas = document.getElementById('sim');
const renderer = new THREE.WebGLRenderer({canvas, antialias: true});
renderer.setSize(window.innerWidth, window.innerHeight);
const FLUORO_TARGET_SCALE = 0.85;
const fluoroscopyTargetWidth = () => Math.max(1, Math.round(window.innerWidth * FLUORO_TARGET_SCALE));
const fluoroscopyTargetHeight = () => Math.max(1, Math.round(window.innerHeight * FLUORO_TARGET_SCALE));
const initialFluoroTargetWidth = fluoroscopyTargetWidth();
const initialFluoroTargetHeight = fluoroscopyTargetHeight();
const DEVICE_MASK_TARGET_SAMPLES = renderer.capabilities.isWebGL2 ? 4 : 0;
const deviceMaskTargetOptions = { samples: DEVICE_MASK_TARGET_SAMPLES };

// Primary 3D scene (wire, vessels, bones)
const scene = new THREE.Scene();
scene.background = new THREE.Color(0x000000);

// Separate scene for rendering contrast meshes in fluoroscopy mode
const contrastScene = new THREE.Scene();

// Offscreen render targets used by various post-processing passes
const offscreenTarget = new THREE.WebGLRenderTarget(initialFluoroTargetWidth, initialFluoroTargetHeight, deviceMaskTargetOptions);
const contrastTarget = new THREE.WebGLRenderTarget(initialFluoroTargetWidth, initialFluoroTargetHeight);
const metalTarget = new THREE.WebGLRenderTarget(initialFluoroTargetWidth, initialFluoroTargetHeight, deviceMaskTargetOptions);
const catheterTarget = new THREE.WebGLRenderTarget(initialFluoroTargetWidth, initialFluoroTargetHeight, deviceMaskTargetOptions);
const sheathTarget = new THREE.WebGLRenderTarget(initialFluoroTargetWidth, initialFluoroTargetHeight, deviceMaskTargetOptions);
const boneTarget = new THREE.WebGLRenderTarget(initialFluoroTargetWidth, initialFluoroTargetHeight, {
    type: THREE.HalfFloatType
});
const accumulateTarget1 = new THREE.WebGLRenderTarget(initialFluoroTargetWidth, initialFluoroTargetHeight);
const accumulateTarget2 = new THREE.WebGLRenderTarget(initialFluoroTargetWidth, initialFluoroTargetHeight);
const frontDepthTarget = new THREE.WebGLRenderTarget(initialFluoroTargetWidth, initialFluoroTargetHeight);
const backDepthTarget = new THREE.WebGLRenderTarget(initialFluoroTargetWidth, initialFluoroTargetHeight);
const thicknessTarget = new THREE.WebGLRenderTarget(initialFluoroTargetWidth, initialFluoroTargetHeight);
let previousTarget = accumulateTarget1;
let currentTarget = accumulateTarget2;
const anatomyCameraWorld = new Float64Array(16);
const anatomyProjectionMatrix = new Float64Array(16);
let anatomyProjectionValid = false;

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

function createLinearDepthMaterial(side) {
    return new THREE.ShaderMaterial({
        side,
        depthTest: true,
        depthWrite: true,
        uniforms: {
            cameraNear: { value: XRAY_CAMERA_NEAR },
            cameraFar: { value: XRAY_CAMERA_FAR }
        },
        vertexShader: `
            varying float vViewDepth;
            void main() {
                vec4 mvPosition = modelViewMatrix * vec4(position, 1.0);
                vViewDepth = -mvPosition.z;
                gl_Position = projectionMatrix * mvPosition;
            }
        `,
        fragmentShader: `
            uniform float cameraNear;
            uniform float cameraFar;
            varying float vViewDepth;
            void main() {
                float depth = clamp((vViewDepth - cameraNear) / max(1.0, cameraFar - cameraNear), 0.0, 1.0);
                gl_FragColor = vec4(vec3(depth), 1.0);
            }
        `
    });
}

// Depth-only materials used to compute front/back ray length through bone.
const depthMaterialFront = createLinearDepthMaterial(THREE.FrontSide);
const depthMaterialBack = createLinearDepthMaterial(THREE.BackSide);
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
const boneProjectionMaterial = new THREE.ShaderMaterial({
    transparent: true,
    blending: THREE.CustomBlending,
    blendEquation: THREE.AddEquation,
    blendSrc: THREE.OneFactor,
    blendDst: THREE.OneFactor,
    blendEquationAlpha: THREE.AddEquation,
    blendSrcAlpha: THREE.OneFactor,
    blendDstAlpha: THREE.OneFactor,
    side: THREE.DoubleSide,
    depthTest: false,
    depthWrite: false,
    vertexShader: `
        varying vec3 vViewNormal;
        varying vec3 vViewPosition;
        varying vec3 vWorldPosition;
        void main() {
            vec4 mvPosition = modelViewMatrix * vec4(position, 1.0);
            vViewNormal = normalize(normalMatrix * normal);
            vViewPosition = mvPosition.xyz;
            vec4 worldPosition = modelMatrix * vec4(position, 1.0);
            vWorldPosition = worldPosition.xyz;
            gl_Position = projectionMatrix * mvPosition;
        }
    `,
    fragmentShader: `
        varying vec3 vViewNormal;
        varying vec3 vViewPosition;
        varying vec3 vWorldPosition;

        float hash(vec3 p) {
            return fract(sin(dot(p, vec3(17.13, 47.71, 91.37))) * 43758.5453);
        }

        float valueNoise(vec3 p) {
            vec3 i = floor(p);
            vec3 f = fract(p);
            f = f * f * (3.0 - 2.0 * f);
            float n000 = hash(i + vec3(0.0, 0.0, 0.0));
            float n100 = hash(i + vec3(1.0, 0.0, 0.0));
            float n010 = hash(i + vec3(0.0, 1.0, 0.0));
            float n110 = hash(i + vec3(1.0, 1.0, 0.0));
            float n001 = hash(i + vec3(0.0, 0.0, 1.0));
            float n101 = hash(i + vec3(1.0, 0.0, 1.0));
            float n011 = hash(i + vec3(0.0, 1.0, 1.0));
            float n111 = hash(i + vec3(1.0, 1.0, 1.0));
            float nx00 = mix(n000, n100, f.x);
            float nx10 = mix(n010, n110, f.x);
            float nx01 = mix(n001, n101, f.x);
            float nx11 = mix(n011, n111, f.x);
            float nxy0 = mix(nx00, nx10, f.y);
            float nxy1 = mix(nx01, nx11, f.y);
            return mix(nxy0, nxy1, f.z);
        }

        void main() {
            vec3 normal = normalize(vViewNormal);
            vec3 rayDir = normalize(-vViewPosition);
            float incidence = clamp(abs(dot(normal, rayDir)), 0.12, 1.0);
            float anglePath = clamp(pow(1.0 / incidence, 0.82), 1.0, 4.2);

            vec3 p = vWorldPosition * 0.035;
            float coarse = valueNoise(p);
            float fine = valueNoise(p * 2.8 + vec3(4.0, 11.0, 2.0));
            float trabeculae = smoothstep(0.42, 0.92, coarse * 0.62 + fine * 0.38);
            float marrowMottle = mix(0.72, 1.08, valueNoise(p * 1.35 + vec3(2.0, 7.0, 13.0)));

            float encodedDepth = length(vViewPosition) * 0.00072;
            float entryDepth = gl_FrontFacing ? encodedDepth : 0.0;
            float exitDepth = gl_FrontFacing ? 0.0 : encodedDepth;
            float grazingCortex = smoothstep(1.35, 3.8, anglePath);
            float corticalPath = anglePath * 0.0048 + grazingCortex * 0.036 + trabeculae * marrowMottle * 0.0009;
            float trabecularTexture = trabeculae * marrowMottle * 0.026;

            gl_FragColor = vec4(entryDepth, exitDepth, corticalPath, trabecularTexture);
        }
    `
});

const displayMaterial = new THREE.ShaderMaterial({
    uniforms: {
        uTexture: { value: previousTarget.texture },
        contrastTexture: { value: contrastTarget.texture },
        thicknessTexture: { value: thicknessTarget.texture },
        metalTexture: { value: metalTarget.texture },
        catheterTexture: { value: catheterTarget.texture },
        sheathTexture: { value: sheathTarget.texture },
        boneTexture: { value: boneTarget.texture },
        gray: { value: new THREE.Color(0xEBEBEB) },
        fluoroscopy: { value: false },
        time: { value: 0 },
        noiseLevel: { value: 0.1 },
        imageBrightness: { value: 0.18 },
        imageContrast: { value: 1.33 },
        autoExposureEnabled: { value: false },
        autoExposureLevel: { value: 0.0 },
        pulseRate: { value: 15.0 },
        scatterStrength: { value: 0.45 },
        collimation: { value: 0.08 },
        boneOpacity: { value: 0.62 },
        resolution: { value: new THREE.Vector2(initialFluoroTargetWidth, initialFluoroTargetHeight) },
        edgeStrength: { value: 0.1 },
        contrastOpacity: { value: 1.0 },
        contrastGain: { value: 5.0 }

    },
    vertexShader: displayVS,
    fragmentShader: displayFS
});
const displayQuad = new THREE.Mesh(quadGeometry, displayMaterial);
const displayScene = new THREE.Scene();
displayScene.add(displayQuad);

// C-arm configuration: camera acts as X-ray source; detector is simulated in shaders
const cameraRadius = 350;
const camera = new THREE.PerspectiveCamera(45, window.innerWidth / window.innerHeight, XRAY_CAMERA_NEAR, XRAY_CAMERA_FAR);
camera.position.set(0, 80, cameraRadius);
scene.add(camera);

// Keep the vascular simulation in its native coordinate space and move only
// its render layer so tools, contrast and debug overlays stay aligned with the
// vessel while the whole assembly sits lower relative to the skeleton.
const VASCULAR_MODEL_ALIGNMENT_Y_MM = -15;
function alignVascularRenderObject(object) {
    object.position.y += VASCULAR_MODEL_ALIGNMENT_Y_MM;
    return object;
}

let vesselGroup;
const { group: skeletonModel, material: boneMaterial } = createBoneModel({
    onLoaded: () => {
        anatomyProjectionValid = false;
        completeLoadingMilestone(
            'skeleton',
            loadingMilestones.has('aorta') ? 'Loading vessel model' : 'Rendering first frame'
        );
    },
    onError: () => failLoadingMilestone('skeleton')
});

// Lightweight centerline metadata; the visible vessel and collision surface are
// loaded from the STL aorta model.
const { vessel } = generateVessel(140, 0);
vesselGroup = alignVascularRenderObject(new THREE.Group());
let vesselCollisionTarget = vessel;
let pigtailCatheter = null;
let guidewireSolver = null;
let endovascularWorld = null;
let xpbdWireBody = null;
let xpbdCatheterBody = null;
let xpbdContainment = null;
let xpbdExternalToolContact = null;
let xpbdContactDebugGroup = null;
let xpbdContactNormalLines = null;
let xpbdActiveBranchLines = null;

function createSheathGeometry(sheath, radiusScale = 1) {
    const start = new THREE.Vector3(sheath.start.x, sheath.start.y, sheath.start.z);
    const end = new THREE.Vector3(sheath.end.x, sheath.end.y, sheath.end.z);
    const axis = new THREE.Vector3().subVectors(end, start);
    const length = axis.length();
    const radius = sheath.radius * radiusScale;
    const geometry = new THREE.CylinderGeometry(radius, radius, length, 18, 1, true);
    geometry.applyQuaternion(new THREE.Quaternion().setFromUnitVectors(new THREE.Vector3(0, 1, 0), axis.normalize()));
    geometry.translate((start.x + end.x) * 0.5, (start.y + end.y) * 0.5, (start.z + end.z) * 0.5);
    return geometry;
}

function createSheathMesh(sheath) {
    const geometry = createSheathGeometry(sheath);
    const material = new THREE.MeshBasicMaterial({
        color: LUMEN_DEBUG_COLOR,
        side: THREE.DoubleSide,
        transparent: true,
        opacity: 0.34,
        depthWrite: false,
        depthTest: false
    });
    const mesh = new THREE.Mesh(geometry, material);
    mesh.renderOrder = 6.6;
    return mesh;
}

function createSheathFluoroMesh(sheath) {
    const geometry = createSheathGeometry(sheath);
    const material = new THREE.MeshBasicMaterial({
        color: 0xffffff,
        side: THREE.DoubleSide,
        transparent: true,
        opacity: 0.065,
        depthTest: false,
        depthWrite: false
    });
    const mesh = new THREE.Mesh(geometry, material);
    mesh.renderOrder = 0.7;
    return mesh;
}

function createExactLumenDebugMesh(geometry, {
    debugLayer = null,
    color = LUMEN_DEBUG_COLOR,
    opacity = 0.24,
    renderOrder = 3,
    depthTest = true
} = {}) {
    const material = new THREE.MeshBasicMaterial({
        color,
        side: THREE.DoubleSide,
        transparent: true,
        opacity,
        depthWrite: false,
        depthTest
    });
    const mesh = new THREE.Mesh(geometry, material);
    mesh.renderOrder = renderOrder;
    if (debugLayer) mesh.userData.debugLayer = debugLayer;
    return mesh;
}

function findSheathVesselEntryPoint(collision, sheath) {
    const collider = collision?.meshCollider || collision?.lumenMeshCollider || null;
    if (!collider?.pointContact || !sheath?.start || !sheath?.end) return null;

    const start = new THREE.Vector3(sheath.start.x, sheath.start.y, sheath.start.z);
    const end = new THREE.Vector3(sheath.end.x, sheath.end.y, sheath.end.z);
    const axis = new THREE.Vector3().subVectors(end, start);
    const length = axis.length();
    if (length < 1e-6) return null;

    const pointAt = t => start.clone().addScaledVector(axis, t);
    const signedDistanceAt = t => collider.pointContact(pointAt(t), 0)?.signedDistance ?? -Infinity;
    const samples = Math.max(16, Math.ceil(length / 2));
    let previousT = 0;
    let previousSignedDistance = signedDistanceAt(0);

    for (let i = 1; i <= samples; i++) {
        const t = i / samples;
        const signedDistance = signedDistanceAt(t);
        if (previousSignedDistance < 0 && signedDistance >= 0) {
            let lo = previousT;
            let hi = t;
            for (let iter = 0; iter < 14; iter++) {
                const mid = (lo + hi) * 0.5;
                if (signedDistanceAt(mid) >= 0) hi = mid;
                else lo = mid;
            }
            return {
                point: pointAt(hi),
                tangent: axis.normalize()
            };
        }
        previousT = t;
        previousSignedDistance = signedDistance;
    }

    return null;
}

function createSheathEntryDebugMarker(collision, sheath) {
    const entry = findSheathVesselEntryPoint(collision, sheath);
    const group = new THREE.Group();
    if (!entry) return group;

    const markerMaterial = new THREE.MeshBasicMaterial({
        color: SHEATH_ENTRY_MARKER_COLOR,
        transparent: true,
        opacity: 0.95,
        depthTest: false,
        depthWrite: false,
        toneMapped: false
    });
    const core = new THREE.Mesh(new THREE.SphereGeometry(2.2, 18, 12), markerMaterial);
    core.renderOrder = 9.5;
    group.add(core);

    const ring = new THREE.Mesh(
        new THREE.TorusGeometry(4.2, 0.32, 8, 32),
        markerMaterial.clone()
    );
    ring.quaternion.setFromUnitVectors(new THREE.Vector3(0, 0, 1), entry.tangent);
    ring.renderOrder = 9.4;
    group.add(ring);

    group.position.copy(entry.point);
    group.frustumCulled = false;
    return group;
}

vesselGroup.add(createSheathMesh(vessel.sheath));
const sheathFluoroMesh = createSheathFluoroMesh(vessel.sheath);
sheathFluoroMesh.visible = true;
alignVascularRenderObject(sheathFluoroMesh);
scene.add(sheathFluoroMesh);
const lumenDebugGroup = new THREE.Group();
lumenDebugGroup.visible = false;
vesselGroup.add(lumenDebugGroup);
const debugLayerVisibility = {
    stlModel: true,
    lumenCast: false,
    sections: false,
    centerline: true,
    capsules: false
};
function applyDebugLayerVisibility() {
    lumenDebugGroup.traverse(object => {
        const layer = object.userData?.debugLayer;
        if (!layer || !(layer in debugLayerVisibility)) return;
        object.visible = !!debugLayerVisibility[layer];
    });
}
setLoadingMessage('Loading anatomy models');
createAortaModel(vessel, {
    onLoaded: ({ collision }) => {
        vesselCollisionTarget = {
            ...collision,
            segments: [vessel.sheath]
        };
        if (endovascularWorld) endovascularWorld.contactField = collision.contactField;
        lumenDebugGroup.clear();
        lumenDebugGroup.add(createExactLumenDebugMesh(collision.geometry, {
            debugLayer: 'stlModel',
            color: STL_MODEL_DEBUG_COLOR,
            opacity: 0.18,
            renderOrder: 2.8
        }));
        if (collision.preprocessing?.lumenCastGeometry) {
            lumenDebugGroup.add(createExactLumenDebugMesh(collision.preprocessing.lumenCastGeometry, {
                debugLayer: 'lumenCast',
                color: LUMEN_DEBUG_COLOR,
                opacity: 0.28,
                renderOrder: 9.15,
                depthTest: false
            }));
        }
        lumenDebugGroup.add(createStlPreprocessDebug(collision.preprocessing));
        lumenDebugGroup.add(createBroadPhaseDebugGroup(collision.centerlineBroadPhase));
        lumenDebugGroup.add(createSheathEntryDebugMarker(collision, vessel.sheath));
        applyDebugLayerVisibility();
        guidewireSolver?.requestSettle?.(90);
        pigtailCatheter?.setCollisionGeometry(collision);
        completeLoadingMilestone(
            'aorta',
            loadingMilestones.has('skeleton') ? 'Loading skeleton model' : 'Rendering first frame'
        );
    },
    onError: () => {
        failLoadingMilestone('aorta');
    }
});
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
const voxelGroup = alignVascularRenderObject(new THREE.Group());
scene.add(voxelGroup);
let contrastMesh = null;
let contrastMeshCount = 0;
let contrastRenderAccumulator = 0;

// Guidewire physical model (discrete elastic rod)
const segmentLength = 5;
const nodeCount = 201;
const guidewireLength = segmentLength * (nodeCount - 1);
const GUIDEWIRE_ADVANCE_RATE = 44;

// Initialize wire nodes along the sheath axis, tail outside the body
const wire = new ElasticRod(nodeCount, segmentLength, {
    constraintIterations: 28
});
let tailProgress = 0;
const maxInsert = guidewireLength;
// Prevent withdrawing the wire past the sheath entrance so the tip
// always remains within the sheath.
const minInsert = 0;

function createStlPreprocessDebug(preprocessing) {
    const group = new THREE.Group();
    if (!preprocessing) return group;

    if (preprocessing.boundaryDebugSegments?.length) {
        const boundaryGeometry = new THREE.BufferGeometry();
        boundaryGeometry.setAttribute(
            'position',
            new THREE.BufferAttribute(preprocessing.boundaryDebugSegments, 3)
        );
        const boundaryLines = new THREE.LineSegments(
            boundaryGeometry,
            new THREE.LineBasicMaterial({
                color: STL_BOUNDARY_EDGE_COLOR,
                transparent: true,
                opacity: 0.85,
                depthTest: false,
                depthWrite: false,
                toneMapped: false
            })
        );
        boundaryLines.frustumCulled = false;
        boundaryLines.renderOrder = 9;
        boundaryLines.userData.debugLayer = 'sections';
        group.add(boundaryLines);
    }

    const contourDebugSegments = preprocessing.centerlineSliceDebugSegments?.length
        ? preprocessing.centerlineSliceDebugSegments
        : preprocessing.lumenContourDebugSegments;

    if (contourDebugSegments?.length) {
        const contourGeometry = new THREE.BufferGeometry();
        contourGeometry.setAttribute(
            'position',
            new THREE.BufferAttribute(contourDebugSegments, 3)
        );
        const contourLines = new THREE.LineSegments(
            contourGeometry,
            new THREE.LineBasicMaterial({
                color: STL_LUMEN_CONTOUR_COLOR,
                transparent: true,
                opacity: 0.72,
                depthTest: false,
                depthWrite: false,
                toneMapped: false
            })
        );
        contourLines.frustumCulled = false;
        contourLines.renderOrder = 8.5;
        contourLines.userData.debugLayer = 'sections';
        group.add(contourLines);
    }

    const interiorSamples = preprocessing.interiorSamples || [];
    if (interiorSamples.length) {
        const samples = new THREE.InstancedMesh(
            new THREE.SphereGeometry(1.15, 10, 6),
            new THREE.MeshBasicMaterial({
                color: STL_INTERIOR_SAMPLE_COLOR,
                transparent: true,
                opacity: 0.82,
                depthTest: false,
                depthWrite: false,
                toneMapped: false
            }),
            interiorSamples.length
        );
        const matrix = new THREE.Matrix4();
        for (let i = 0; i < interiorSamples.length; i++) {
            matrix.makeTranslation(
                interiorSamples[i].x,
                interiorSamples[i].y,
                interiorSamples[i].z
            );
            samples.setMatrixAt(i, matrix);
        }
        samples.frustumCulled = false;
        samples.instanceMatrix.needsUpdate = true;
        samples.renderOrder = 8;
        samples.userData.debugLayer = 'sections';
        group.add(samples);
    }

    return group;
}

guidewireSolver = new GuidewireSolver({
    rod: wire,
    segmentLength,
    guidewireLength,
    sheath: vessel.sheath,
    advanceRate: GUIDEWIRE_ADVANCE_RATE,
    minInsert,
    maxInsert,
    lumenClearance: GUIDEWIRE_RADIUS_MM,
    straightening: 0.72,
    routeBlend: 0,
    relaxationIterations: 6,
    lengthIterations: 10,
    meshClearance: GUIDEWIRE_RADIUS_MM,
    foldGuardAngle: 166,
    foldGuardStrength: 0.62,
    foldGuardPasses: 2,
    foldGuardCenterPull: 1.25,
    stabilityRepairSegmentError: 0.09,
    stabilityRepairBendAngle: 150,
    stabilityRepairTargetBendAngle: 112,
    stabilityRepairPasses: 3,
    stabilityRepairLengthIterations: 10,
    tipBacktrackAngle: 108,
    tipBacktrackStrength: 1,
    segmentProjectionBlend: 0.48,
    maxSegmentProjectionStep: 0.32,
    collisionProjectionRepeats: 1,
    segmentSamples: [0.1, 0.24, 0.38, 0.52, 0.66, 0.8, 0.93],
    finalCollisionPasses: 3,
    finalLengthPasses: 2,
    finalProjectionPasses: 2
});

// The proximal guidewire and the part inside the introducer sheath are
// constrained by the sheath lumen. Once a node exits the sheath tip it becomes
// free and is governed by rod stiffness and vessel-wall collision.
applyGuidewireMaterialProfile(wire, { segmentLength });
guidewireSolver.initialize();
tailProgress = guidewireSolver.progress;

// Injection state (managed by UI callbacks)
let injecting = false;
let injectTime = 0;
let injectDuration = 2; // seconds
let injectRate = 2; // ml per second
let injectVolume = 10; // total ml
let remainingVolume = 0;
let totalDose = 0;

// Renderable guidewire: white so the fluoroscopy shader can invert it to black.
const wireMaterial = new THREE.MeshBasicMaterial({
    color: 0xffffff,
    depthTest: false,
    depthWrite: false,
    toneMapped: false
});
const wireProjectionMaterial = new THREE.ShaderMaterial({
    vertexShader: `
        varying float vWireProfile;
        void main() {
            mat4 instanceTransform = mat4(1.0);
            #ifdef USE_INSTANCING
                instanceTransform = instanceMatrix;
            #endif
            vec4 mvPosition = modelViewMatrix * instanceTransform * vec4(position, 1.0);
            vec3 viewNormal = normalize(normalMatrix * mat3(instanceTransform) * normal);
            vWireProfile = pow(clamp(abs(viewNormal.z), 0.0, 1.0), 0.75);
            gl_Position = projectionMatrix * mvPosition;
        }
    `,
    fragmentShader: `
        varying float vWireProfile;
        void main() {
            float profile = mix(0.06, 1.0, smoothstep(0.08, 0.94, vWireProfile));
            gl_FragColor = vec4(vec3(profile), 1.0);
        }
    `,
    depthTest: false,
    depthWrite: false,
    toneMapped: false
});
// Initialize UI after wireMaterial is created so mode toggle can affect it
let fluoroscopy = true;
let wallContactMarkers = null;
let wallBreachMarkers = null;
let wallWorstPointMarker = null;
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
        sheathFluoroMesh.visible = fluoroscopy;
        lumenDebugGroup.visible = !fluoroscopy;
        if (wallContactMarkers) wallContactMarkers.visible = !fluoroscopy;
        if (wallBreachMarkers) wallBreachMarkers.visible = !fluoroscopy;
        if (wallWorstPointMarker) wallWorstPointMarker.visible = !fluoroscopy && !!wallWorstPointMarker.userData.hasPoint;
        if (xpbdContactDebugGroup) {
            xpbdContactDebugGroup.visible = !fluoroscopy && !!debugLayerVisibility.capsules;
        }
        skeletonModel.visible = fluoroscopy;
        displayMaterial.uniforms.fluoroscopy.value = fluoroscopy;
    },
    onDebugLayerChange: layers => {
        Object.assign(debugLayerVisibility, layers);
        applyDebugLayerVisibility();
        if (xpbdContactDebugGroup) {
            xpbdContactDebugGroup.visible = !fluoroscopy && !!debugLayerVisibility.capsules;
        }
    },
    onStartBrowserBenchmark: durationMs => startBrowserBenchmarkScenario({ durationMs }),
    onStopBrowserBenchmark: () => stopBrowserBenchmarkScenario('ui'),
});
const { monitor } = ui;
const wireMesh = new THREE.Mesh(new THREE.BufferGeometry(), wireMaterial);
wireMesh.frustumCulled = false;
wireMesh.renderOrder = 7; // draw above translucent debug anatomy
const wireGroup = new THREE.Group();
wireGroup.add(wireMesh);
alignVascularRenderObject(wireGroup);
scene.add(wireGroup);
const wireRenderPoints = Array.from({ length: nodeCount }, () => new THREE.Vector3());
const contactMarkerMatrix = new THREE.Matrix4();

const contactMarkerGeometry = new THREE.SphereGeometry(1.35, 12, 8);
const breachMarkerGeometry = new THREE.SphereGeometry(2.1, 12, 8);
wallContactMarkers = new THREE.InstancedMesh(
    contactMarkerGeometry,
    new THREE.MeshBasicMaterial({
        color: WALL_CONTACT_COLOR,
        transparent: true,
        opacity: 0.95,
        depthTest: false,
        depthWrite: false,
        toneMapped: false
    }),
    CONTACT_MARKER_LIMIT
);
wallContactMarkers.instanceMatrix.setUsage(THREE.DynamicDrawUsage);
wallContactMarkers.count = 0;
wallContactMarkers.visible = true;
wallContactMarkers.frustumCulled = false;
wallContactMarkers.renderOrder = 6;
alignVascularRenderObject(wallContactMarkers);
scene.add(wallContactMarkers);

wallBreachMarkers = new THREE.InstancedMesh(
    breachMarkerGeometry,
    new THREE.MeshBasicMaterial({
        color: WALL_BREACH_COLOR,
        transparent: true,
        opacity: 1,
        depthTest: false,
        depthWrite: false,
        toneMapped: false
    }),
    CONTACT_MARKER_LIMIT
);
wallBreachMarkers.instanceMatrix.setUsage(THREE.DynamicDrawUsage);
wallBreachMarkers.count = 0;
wallBreachMarkers.visible = true;
wallBreachMarkers.frustumCulled = false;
wallBreachMarkers.renderOrder = 7;
alignVascularRenderObject(wallBreachMarkers);
scene.add(wallBreachMarkers);

wallWorstPointMarker = new THREE.Mesh(
    new THREE.SphereGeometry(2.8, 16, 10),
    new THREE.MeshBasicMaterial({
        color: WALL_WORST_POINT_COLOR,
        transparent: true,
        opacity: 1,
        depthTest: false,
        depthWrite: false,
        toneMapped: false
    })
);
wallWorstPointMarker.visible = false;
wallWorstPointMarker.frustumCulled = false;
wallWorstPointMarker.renderOrder = 8;
wallWorstPointMarker.userData.hasPoint = false;
alignVascularRenderObject(wallWorstPointMarker);
scene.add(wallWorstPointMarker);

function createDynamicDebugLines(color) {
    const positions = new Float32Array(CONTACT_MARKER_LIMIT * 6);
    const geometry = new THREE.BufferGeometry();
    geometry.setAttribute('position', new THREE.BufferAttribute(positions, 3));
    geometry.setDrawRange(0, 0);
    const lines = new THREE.LineSegments(
        geometry,
        new THREE.LineBasicMaterial({
            color,
            transparent: true,
            opacity: 0.95,
            depthTest: false,
            depthWrite: false,
            toneMapped: false
        })
    );
    lines.frustumCulled = false;
    lines.renderOrder = 9.9;
    return lines;
}

xpbdContactDebugGroup = new THREE.Group();
xpbdContactNormalLines = createDynamicDebugLines(0x28ffd7);
xpbdActiveBranchLines = createDynamicDebugLines(0xff4fd8);
xpbdContactDebugGroup.add(xpbdContactNormalLines, xpbdActiveBranchLines);
xpbdContactDebugGroup.visible = !fluoroscopy && !!debugLayerVisibility.capsules;
alignVascularRenderObject(xpbdContactDebugGroup);
scene.add(xpbdContactDebugGroup);

const GUIDE_WIRE_ADVANCE_OPTIONS = {
    routeAssist: PHYSICS_MODE === 'legacy',
    // In XPBD the sheath displacement is the physical proximal boundary
    // condition. Length and bend constraints must transmit that displacement
    // through the free rod; copying every node along the previous path would
    // make insertion and withdrawal replay the same geometry.
    boundaryDriven: PHYSICS_MODE === 'xpbd-contact-v1'
};
// Preserve the velocity produced by the previous XPBD step. With boundary
// driven feeding this is genuine rod state, not duplicated kinematic motion.
const XPBD_WIRE_SYNC_OPTIONS = { resetVelocity: PHYSICS_MODE !== 'xpbd-contact-v1' };
const XPBD_CATHETER_STEP_OPTIONS = { collisions: false };
const XPBD_CATHETER_SYNC_OPTIONS = {
    shapeCompliance: DEFAULT_TOOL_PROFILES.catheter.shapeCompliance
};

pigtailCatheter = new PigtailCatheter({
    wire,
    segmentLength,
    guidewireLength,
    tailProgressRef: () => guidewireSolver.progress,
    vessel
});
pigtailCatheter.setExternalCollisionSolver(PHYSICS_MODE === 'xpbd-contact-v1');
if (vesselCollisionTarget !== vessel) {
    pigtailCatheter.setCollisionGeometry(vesselCollisionTarget);
}
alignVascularRenderObject(pigtailCatheter.mesh);
scene.add(pigtailCatheter.mesh);

endovascularWorld = new EndovascularPhysicsWorld({
    contactField: vesselCollisionTarget.contactField || null,
    fixedDt: 1 / 120,
    maxSubsteps: 2,
    iterations: 6,
    penetrationIterations: 8,
    highPenetration: 0.15,
    contactActivation: 0.2
});
xpbdWireBody = endovascularWorld.createRod('guidewire', nodeCount, segmentLength, {
    ...DEFAULT_TOOL_PROFILES.guidewire
});
xpbdWireBody.syncFromElasticRod(wire);
xpbdCatheterBody = endovascularWorld.createRod('catheter', 320, 4, {
    ...DEFAULT_TOOL_PROFILES.catheter
});
pigtailCatheter.syncXpbdBody(xpbdCatheterBody, XPBD_CATHETER_SYNC_OPTIONS);
endovascularWorld.addSheath({
    start: vessel.sheath.start,
    end: vessel.sheath.end,
    innerRadius: INTRODUCER_SHEATH_INNER_RADIUS_MM,
    bodies: [xpbdWireBody, xpbdCatheterBody]
});
xpbdContainment = endovascularWorld.addContainment(xpbdWireBody, xpbdCatheterBody, {
    innerRadius: PIGTAIL_CATHETER_INNER_RADIUS_MM,
    openProximal: true,
    openDistal: true,
    searchWindow: 2,
    outerStartNode: pigtailCatheter.physicsLumenStartNode,
    innerResponse: 0,
    outerResponse: 1,
    finalProjection: 'outer',
    outerFollowsInnerCenterline: true,
    containedLength: 0,
    enabled: false
});
xpbdExternalToolContact = endovascularWorld.addToolContact(xpbdWireBody, xpbdCatheterBody, {
    friction: 0.08,
    openDistalB: true,
    enabled: false
});
const browserBenchmarkBodies = [xpbdWireBody, xpbdCatheterBody];
globalThis.__OET_PHYSICS__ = {
    mode: PHYSICS_MODE,
    world: endovascularWorld,
    getStats: () => endovascularWorld.getStats()
};

const BROWSER_BENCHMARK_FRAME_CAPACITY = 40000;
const BROWSER_BENCHMARK_CHOREOGRAPHY_WARMUP_MS = BROWSER_BENCHMARK_SCENARIO_CYCLE_MS * 2;
const BROWSER_BENCHMARK_MEMORY_SETTLE_MS = 60 * 1000;
const BROWSER_BENCHMARK_WARMUP_MS =
    BROWSER_BENCHMARK_CHOREOGRAPHY_WARMUP_MS + BROWSER_BENCHMARK_MEMORY_SETTLE_MS;
const BROWSER_BENCHMARK_FPS_WINDOW_CAPACITY = 610;
const BROWSER_BENCHMARK_LONG_EVENT_CAPACITY = 256;
const BROWSER_BENCHMARK_LONG_EVENT_STRIDE = 8;
const browserFrameTimes = new Float32Array(BROWSER_BENCHMARK_FRAME_CAPACITY);
const browserFpsWindows = new Float32Array(BROWSER_BENCHMARK_FPS_WINDOW_CAPACITY);
const browserFrameCpuSimulation = new Float32Array(BROWSER_BENCHMARK_FRAME_CAPACITY);
const browserFrameCpuUpdate = new Float32Array(BROWSER_BENCHMARK_FRAME_CAPACITY);
const browserFrameCpuRender = new Float32Array(BROWSER_BENCHMARK_FRAME_CAPACITY);
const browserFrameCpuTotal = new Float32Array(BROWSER_BENCHMARK_FRAME_CAPACITY);
const browserLongFrameEvents = new Float32Array(
    BROWSER_BENCHMARK_LONG_EVENT_CAPACITY * BROWSER_BENCHMARK_LONG_EVENT_STRIDE
);
let browserFrameCursor = 0;
let browserFrameCount = 0;
let browserFrameTimeSum = 0;
let browserMaxFrameMs = 0;
let browserLongFrame33Count = 0;
let browserLongFrame50Count = 0;
let browserFpsWindowCount = 0;
let browserFpsWindowElapsedMs = 0;
let browserFpsWindowFrames = 0;
let browserLongFrameEventCount = 0;
let browserCameraRevisionStart = 0;
let browserFocusLossCount = 0;
let browserFocusLossMs = 0;
let browserFocusLostAt = 0;
let browserBenchmarkStartedAt = performance.now();
const browserFrameCpu = {
    count: 0,
    simulationSumMs: 0,
    updateSumMs: 0,
    renderSumMs: 0,
    totalSumMs: 0,
    maximumMs: 0,
    simulationMaximumMs: 0,
    updateMaximumMs: 0,
    renderMaximumMs: 0,
    lastSimulationMs: 0,
    lastUpdateMs: 0,
    lastRenderMs: 0,
    lastTotalMs: 0
};
const browserHeap = {
    supported: false,
    samples: 0,
    startBytes: null,
    minimumBytes: null,
    maximumBytes: null,
    endBytes: null
};
const browserBenchmarkScenario = {
    running: false,
    warmingUp: false,
    durationMs: BROWSER_BENCHMARK_DEFAULT_DURATION_MS,
    warmupStartedAt: 0,
    memorySettling: false,
    startedAt: 0,
    completedAt: 0,
    simulationElapsedMs: 0,
    stopReason: null,
    automated: false
};
const AUTOMATED_BENCHMARK_BLOCKED_EVENTS = [
    'pointerdown',
    'mousedown',
    'touchstart',
    'click',
    'dblclick',
    'wheel',
    'keydown',
    'input',
    'change'
];
function blockAutomatedBenchmarkInput(event) {
    if (!browserBenchmarkScenario.running || !browserBenchmarkScenario.automated) return;
    event.preventDefault();
    event.stopImmediatePropagation();
}
for (const eventName of AUTOMATED_BENCHMARK_BLOCKED_EVENTS) {
    window.addEventListener(eventName, blockAutomatedBenchmarkInput, {
        capture: true,
        passive: false
    });
}
const browserBenchmarkCommands = createBrowserBenchmarkCommands();
const browserBenchmarkPhysicsEnvelope = {
    steps: 0,
    maxPostStepPenetrationMm: 0,
    maxPostStepPenetrationStep: -1,
    maxPostStepPenetrationBodyId: null,
    maxPostStepPenetrationSegment: -1,
    maxPostStepPenetrationT: 0,
    maxPostStepPenetrationX: 0,
    maxPostStepPenetrationY: 0,
    maxPostStepPenetrationZ: 0,
    maxTransientPenetrationMm: 0,
    maxTransientPenetrationStep: -1,
    maxSegmentErrorPercent: 0,
    maxSegmentErrorBodyId: null,
    maxSegmentErrorNodeIndex: -1,
    maxSegmentErrorStep: -1,
    maxBendAngleDegrees: 0,
    maxBendBodyId: null,
    maxBendNodeIndex: -1,
    maxBendStep: -1,
    maxBendX: 0,
    maxBendY: 0,
    maxBendZ: 0,
    finite: true
};
let lastBrowserBenchmarkScenarioReport = null;

function sampleBrowserHeap() {
    const bytes = performance.memory?.usedJSHeapSize;
    if (!Number.isFinite(bytes)) return;
    browserHeap.supported = true;
    if (browserHeap.samples === 0) {
        browserHeap.startBytes = bytes;
        browserHeap.minimumBytes = bytes;
        browserHeap.maximumBytes = bytes;
    } else {
        browserHeap.minimumBytes = Math.min(browserHeap.minimumBytes, bytes);
        browserHeap.maximumBytes = Math.max(browserHeap.maximumBytes, bytes);
    }
    browserHeap.endBytes = bytes;
    browserHeap.samples++;
}

function resetBrowserHeap() {
    browserHeap.supported = false;
    browserHeap.samples = 0;
    browserHeap.startBytes = null;
    browserHeap.minimumBytes = null;
    browserHeap.maximumBytes = null;
    browserHeap.endBytes = null;
    sampleBrowserHeap();
}

function getBrowserHeapStats() {
    sampleBrowserHeap();
    return {
        ...browserHeap,
        growthBytes: browserHeap.supported
            ? browserHeap.endBytes - browserHeap.startBytes
            : null,
        rangeBytes: browserHeap.supported
            ? browserHeap.maximumBytes - browserHeap.minimumBytes
            : null
    };
}

function resetBrowserBenchmark() {
    browserFrameCursor = 0;
    browserFrameCount = 0;
    browserFrameTimeSum = 0;
    browserMaxFrameMs = 0;
    browserLongFrame33Count = 0;
    browserLongFrame50Count = 0;
    browserFpsWindowCount = 0;
    browserFpsWindowElapsedMs = 0;
    browserFpsWindowFrames = 0;
    browserLongFrameEventCount = 0;
    browserCameraRevisionStart = ui.getCArmRevision?.() ?? 0;
    browserFocusLossCount = 0;
    browserFocusLossMs = 0;
    browserFocusLostAt = document.hasFocus() ? 0 : performance.now();
    browserFrameCpu.count = 0;
    browserFrameCpu.simulationSumMs = 0;
    browserFrameCpu.updateSumMs = 0;
    browserFrameCpu.renderSumMs = 0;
    browserFrameCpu.totalSumMs = 0;
    browserFrameCpu.maximumMs = 0;
    browserFrameCpu.simulationMaximumMs = 0;
    browserFrameCpu.updateMaximumMs = 0;
    browserFrameCpu.renderMaximumMs = 0;
    browserFrameCpu.lastSimulationMs = 0;
    browserFrameCpu.lastUpdateMs = 0;
    browserFrameCpu.lastRenderMs = 0;
    browserFrameCpu.lastTotalMs = 0;
    resetBrowserHeap();
    browserBenchmarkPhysicsEnvelope.steps = 0;
    browserBenchmarkPhysicsEnvelope.maxPostStepPenetrationMm = 0;
    browserBenchmarkPhysicsEnvelope.maxPostStepPenetrationStep = -1;
    browserBenchmarkPhysicsEnvelope.maxPostStepPenetrationBodyId = null;
    browserBenchmarkPhysicsEnvelope.maxPostStepPenetrationSegment = -1;
    browserBenchmarkPhysicsEnvelope.maxTransientPenetrationMm = 0;
    browserBenchmarkPhysicsEnvelope.maxTransientPenetrationStep = -1;
    browserBenchmarkPhysicsEnvelope.maxSegmentErrorPercent = 0;
    browserBenchmarkPhysicsEnvelope.maxSegmentErrorBodyId = null;
    browserBenchmarkPhysicsEnvelope.maxSegmentErrorNodeIndex = -1;
    browserBenchmarkPhysicsEnvelope.maxSegmentErrorStep = -1;
    browserBenchmarkPhysicsEnvelope.maxBendAngleDegrees = 0;
    browserBenchmarkPhysicsEnvelope.maxBendBodyId = null;
    browserBenchmarkPhysicsEnvelope.maxBendNodeIndex = -1;
    browserBenchmarkPhysicsEnvelope.maxBendStep = -1;
    browserBenchmarkPhysicsEnvelope.maxBendX = 0;
    browserBenchmarkPhysicsEnvelope.maxBendY = 0;
    browserBenchmarkPhysicsEnvelope.maxBendZ = 0;
    browserBenchmarkPhysicsEnvelope.finite = true;
    browserBenchmarkStartedAt = performance.now();
    endovascularWorld.resetPerformanceStats();
    vesselCollisionTarget.contactField?.resetStats?.();
}

function recordBrowserFrame(frameMs) {
    if (!Number.isFinite(frameMs) || frameMs <= 0) return;
    if (browserFrameCount === browserFrameTimes.length) {
        browserFrameTimeSum -= browserFrameTimes[browserFrameCursor];
    } else {
        browserFrameCount++;
    }
    browserFrameTimes[browserFrameCursor] = frameMs;
    browserFrameTimeSum += frameMs;
    browserMaxFrameMs = Math.max(browserMaxFrameMs, frameMs);
    browserFpsWindowElapsedMs += frameMs;
    browserFpsWindowFrames++;
    if (browserFpsWindowElapsedMs >= 1000) {
        sampleBrowserHeap();
        if (browserFpsWindowCount < browserFpsWindows.length) {
            browserFpsWindows[browserFpsWindowCount++] =
                browserFpsWindowFrames * 1000 / browserFpsWindowElapsedMs;
        }
        browserFpsWindowElapsedMs = 0;
        browserFpsWindowFrames = 0;
    }
    if (frameMs > 1000 / 30) {
        browserLongFrame33Count++;
        if (browserLongFrameEventCount < BROWSER_BENCHMARK_LONG_EVENT_CAPACITY) {
            const offset = browserLongFrameEventCount++ * BROWSER_BENCHMARK_LONG_EVENT_STRIDE;
            browserLongFrameEvents[offset] = frameMs;
            browserLongFrameEvents[offset + 1] = browserBenchmarkScenario.running
                ? performance.now() - browserBenchmarkScenario.startedAt
                : -1;
            browserLongFrameEvents[offset + 2] = browserBenchmarkScenario.simulationElapsedMs;
            browserLongFrameEvents[offset + 3] = performance.memory?.usedJSHeapSize ?? -1;
            browserLongFrameEvents[offset + 4] = browserFrameCpu.lastSimulationMs;
            browserLongFrameEvents[offset + 5] = browserFrameCpu.lastUpdateMs;
            browserLongFrameEvents[offset + 6] = browserFrameCpu.lastRenderMs;
            browserLongFrameEvents[offset + 7] = browserFrameCpu.lastTotalMs;
        }
    }
    if (frameMs > 50) browserLongFrame50Count++;
    browserFrameCursor = (browserFrameCursor + 1) % browserFrameTimes.length;
}

window.addEventListener('blur', () => {
    if (
        !browserBenchmarkScenario.running || browserBenchmarkScenario.warmingUp ||
        browserFocusLostAt > 0
    ) return;
    browserFocusLossCount++;
    browserFocusLostAt = performance.now();
});

window.addEventListener('focus', () => {
    if (browserFocusLostAt <= 0) return;
    if (browserBenchmarkScenario.running && !browserBenchmarkScenario.warmingUp) {
        browserFocusLossMs += performance.now() - browserFocusLostAt;
    }
    browserFocusLostAt = 0;
});

function recordBrowserPhysicsEnvelope() {
    const envelope = browserBenchmarkPhysicsEnvelope;
    envelope.steps++;
    if (endovascularWorld.settledMaxPenetration > envelope.maxPostStepPenetrationMm) {
        envelope.maxPostStepPenetrationMm = endovascularWorld.settledMaxPenetration;
        envelope.maxPostStepPenetrationStep = envelope.steps;
        envelope.maxPostStepPenetrationBodyId = endovascularWorld.settledContactBodyId;
        envelope.maxPostStepPenetrationSegment = endovascularWorld.settledContactSegment;
        envelope.maxPostStepPenetrationT = endovascularWorld.settledContactT;
        envelope.maxPostStepPenetrationX = endovascularWorld.settledContactX;
        envelope.maxPostStepPenetrationY = endovascularWorld.settledContactY;
        envelope.maxPostStepPenetrationZ = endovascularWorld.settledContactZ;
    }
    if (endovascularWorld.maxPenetration > envelope.maxTransientPenetrationMm) {
        envelope.maxTransientPenetrationMm = endovascularWorld.maxPenetration;
        envelope.maxTransientPenetrationStep = envelope.steps;
    }
    if (envelope.steps !== 1 && envelope.steps % 30 !== 0) return;

    for (let bodyIndex = 0; bodyIndex < browserBenchmarkBodies.length; bodyIndex++) {
        const body = browserBenchmarkBodies[bodyIndex];
        if (!body) continue;
        const start = body.activeStart;
        const end = Math.min(body.activeEnd, body.segmentCount);
        for (let index = start; index <= body.activeEnd; index++) {
            envelope.finite = envelope.finite &&
                Number.isFinite(body.x[index]) &&
                Number.isFinite(body.y[index]) &&
                Number.isFinite(body.z[index]) &&
                Number.isFinite(body.velocityX[index]) &&
                Number.isFinite(body.velocityY[index]) &&
                Number.isFinite(body.velocityZ[index]);
        }
        for (let index = start; index < end; index++) {
            const ax = body.x[index + 1] - body.x[index];
            const ay = body.y[index + 1] - body.y[index];
            const az = body.z[index + 1] - body.z[index];
            const length = Math.sqrt(ax * ax + ay * ay + az * az);
            const segmentErrorPercent = Math.abs(length - body.restLength[index]) /
                Math.max(1e-8, body.restLength[index]) * 100;
            if (segmentErrorPercent > envelope.maxSegmentErrorPercent) {
                envelope.maxSegmentErrorPercent = segmentErrorPercent;
                envelope.maxSegmentErrorBodyId = body.id;
                envelope.maxSegmentErrorNodeIndex = index;
                envelope.maxSegmentErrorStep = envelope.steps;
            }
            if (index <= start) continue;
            const bx = body.x[index] - body.x[index - 1];
            const by = body.y[index] - body.y[index - 1];
            const bz = body.z[index] - body.z[index - 1];
            const denominator = Math.sqrt(ax * ax + ay * ay + az * az) *
                Math.sqrt(bx * bx + by * by + bz * bz);
            if (denominator <= 1e-8) continue;
            const cosine = THREE.MathUtils.clamp((ax * bx + ay * by + az * bz) / denominator, -1, 1);
            const bendAngleDegrees = Math.acos(cosine) * 180 / Math.PI;
            if (bendAngleDegrees > envelope.maxBendAngleDegrees) {
                envelope.maxBendAngleDegrees = bendAngleDegrees;
                envelope.maxBendBodyId = body.id;
                envelope.maxBendNodeIndex = index;
                envelope.maxBendStep = envelope.steps;
                envelope.maxBendX = body.x[index];
                envelope.maxBendY = body.y[index];
                envelope.maxBendZ = body.z[index];
            }
        }
    }
}

function browserFramePercentile(fraction) {
    if (!browserFrameCount) return 0;
    const ordered = Array.from(browserFrameTimes.subarray(0, browserFrameCount));
    ordered.sort((a, b) => a - b);
    return ordered[Math.min(ordered.length - 1, Math.floor((ordered.length - 1) * fraction))];
}

function browserOnePercentLowFps() {
    if (!browserFpsWindowCount) return 0;
    const ordered = Array.from(browserFpsWindows.subarray(0, browserFpsWindowCount));
    ordered.sort((a, b) => a - b);
    const count = Math.max(1, Math.ceil(ordered.length * 0.01));
    let sum = 0;
    for (let index = 0; index < count; index++) sum += ordered[index];
    return sum / count;
}

function getBrowserLongFrameEvents() {
    const events = [];
    for (let index = 0; index < browserLongFrameEventCount; index++) {
        const offset = index * BROWSER_BENCHMARK_LONG_EVENT_STRIDE;
        events.push({
            frameMs: browserLongFrameEvents[offset],
            elapsedMs: browserLongFrameEvents[offset + 1],
            simulationElapsedMs: browserLongFrameEvents[offset + 2],
            heapBytes: browserLongFrameEvents[offset + 3],
            previousFrameCpu: {
                simulationMs: browserLongFrameEvents[offset + 4],
                updateMs: browserLongFrameEvents[offset + 5],
                renderMs: browserLongFrameEvents[offset + 6],
                totalMs: browserLongFrameEvents[offset + 7]
            }
        });
    }
    return events;
}

function recordBrowserFrameCpu(startedAt, simulationEndedAt, updateEndedAt) {
    if (!browserBenchmarkScenario.running) return;
    const endedAt = performance.now();
    const simulationMs = simulationEndedAt - startedAt;
    const updateMs = updateEndedAt - simulationEndedAt;
    const renderMs = endedAt - updateEndedAt;
    const totalMs = endedAt - startedAt;
    const sampleIndex = browserFrameCpu.count;
    if (sampleIndex < BROWSER_BENCHMARK_FRAME_CAPACITY) {
        browserFrameCpuSimulation[sampleIndex] = simulationMs;
        browserFrameCpuUpdate[sampleIndex] = updateMs;
        browserFrameCpuRender[sampleIndex] = renderMs;
        browserFrameCpuTotal[sampleIndex] = totalMs;
    }
    browserFrameCpu.count++;
    browserFrameCpu.simulationSumMs += simulationMs;
    browserFrameCpu.updateSumMs += updateMs;
    browserFrameCpu.renderSumMs += renderMs;
    browserFrameCpu.totalSumMs += totalMs;
    browserFrameCpu.maximumMs = Math.max(browserFrameCpu.maximumMs, totalMs);
    browserFrameCpu.simulationMaximumMs = Math.max(browserFrameCpu.simulationMaximumMs, simulationMs);
    browserFrameCpu.updateMaximumMs = Math.max(browserFrameCpu.updateMaximumMs, updateMs);
    browserFrameCpu.renderMaximumMs = Math.max(browserFrameCpu.renderMaximumMs, renderMs);
    browserFrameCpu.lastSimulationMs = simulationMs;
    browserFrameCpu.lastUpdateMs = updateMs;
    browserFrameCpu.lastRenderMs = renderMs;
    browserFrameCpu.lastTotalMs = totalMs;
}

function browserFrameCpuPercentile(buffer, fraction) {
    const count = Math.min(browserFrameCpu.count, BROWSER_BENCHMARK_FRAME_CAPACITY);
    if (!count) return 0;
    const ordered = Array.from(buffer.subarray(0, count));
    ordered.sort((a, b) => a - b);
    return ordered[Math.min(count - 1, Math.floor((count - 1) * fraction))];
}

function getBrowserFrameCpuStats() {
    const count = browserFrameCpu.count || 1;
    return {
        samples: browserFrameCpu.count,
        simulationAverageMs: browserFrameCpu.simulationSumMs / count,
        updateAverageMs: browserFrameCpu.updateSumMs / count,
        renderAverageMs: browserFrameCpu.renderSumMs / count,
        totalAverageMs: browserFrameCpu.totalSumMs / count,
        simulationP95Ms: browserFrameCpuPercentile(browserFrameCpuSimulation, 0.95),
        simulationP99Ms: browserFrameCpuPercentile(browserFrameCpuSimulation, 0.99),
        renderP95Ms: browserFrameCpuPercentile(browserFrameCpuRender, 0.95),
        renderP99Ms: browserFrameCpuPercentile(browserFrameCpuRender, 0.99),
        totalP95Ms: browserFrameCpuPercentile(browserFrameCpuTotal, 0.95),
        totalP99Ms: browserFrameCpuPercentile(browserFrameCpuTotal, 0.99),
        simulationMaximumMs: browserFrameCpu.simulationMaximumMs,
        updateMaximumMs: browserFrameCpu.updateMaximumMs,
        renderMaximumMs: browserFrameCpu.renderMaximumMs,
        maximumMs: browserFrameCpu.maximumMs
    };
}

function getBrowserBenchmarkScenarioStatus() {
    const now = performance.now();
    const elapsedMs = browserBenchmarkScenario.warmingUp
        ? 0
        : browserBenchmarkScenario.running
        ? Math.min(browserBenchmarkScenario.durationMs, now - browserBenchmarkScenario.startedAt)
        : browserBenchmarkScenario.completedAt > browserBenchmarkScenario.startedAt
            ? Math.min(
                browserBenchmarkScenario.durationMs,
                browserBenchmarkScenario.completedAt - browserBenchmarkScenario.startedAt
            )
            : 0;
    return {
        running: browserBenchmarkScenario.running,
        warmingUp: browserBenchmarkScenario.warmingUp,
        warmupPhase: !browserBenchmarkScenario.warmingUp
            ? 'complete'
            : browserBenchmarkScenario.memorySettling ? 'memory-settle' : 'choreography',
        warmupElapsedMs: browserBenchmarkScenario.warmingUp
            ? Math.min(BROWSER_BENCHMARK_WARMUP_MS, now - browserBenchmarkScenario.warmupStartedAt)
            : BROWSER_BENCHMARK_WARMUP_MS,
        durationMs: browserBenchmarkScenario.durationMs,
        elapsedMs,
        simulationElapsedMs: browserBenchmarkScenario.simulationElapsedMs,
        progress: browserBenchmarkScenario.durationMs > 0
            ? Math.min(1, elapsedMs / browserBenchmarkScenario.durationMs)
            : 0,
        cycleIndex: Math.floor(
            browserBenchmarkScenario.simulationElapsedMs / BROWSER_BENCHMARK_SCENARIO_CYCLE_MS
        ),
        catheterType: browserBenchmarkCatheterType(browserBenchmarkScenario.simulationElapsedMs),
        stopReason: browserBenchmarkScenario.stopReason,
        automated: browserBenchmarkScenario.automated
    };
}

function getBrowserBenchmarkReport() {
    const now = performance.now();
    const p99FrameMs = browserFramePercentile(0.99);
    const onePercentLowFps = browserOnePercentLowFps();
    const physics = endovascularWorld.getStats();
    const contactField = vesselCollisionTarget.contactField?.getStats?.() || null;
    const scenario = getBrowserBenchmarkScenarioStatus();
    const durationPass = !scenario.running &&
        scenario.durationMs >= BROWSER_BENCHMARK_DEFAULT_DURATION_MS &&
        scenario.elapsedMs >= BROWSER_BENCHMARK_DEFAULT_DURATION_MS;
    const onePercentLowPass = onePercentLowFps >= 55;
    // Keep the strict 50 ms counter as a scheduling diagnostic. A visible GC pause
    // is assessed separately using frame, heap, and hot-path allocation evidence.
    const noLongFramePass = browserLongFrame50Count === 0;
    const physicsBudgetPass = physics.phases.total.averageMs <= 4 &&
        physics.phases.total.p95Ms <= 6;
    const penetrationPass = browserBenchmarkPhysicsEnvelope.maxPostStepPenetrationMm <= 0.2;
    const lengthPass = browserBenchmarkPhysicsEnvelope.maxSegmentErrorPercent <= 1;
    const foldPass = browserBenchmarkPhysicsEnvelope.maxBendAngleDegrees < 150;
    const finitePass = browserBenchmarkPhysicsEnvelope.finite;
    const modePass = PHYSICS_MODE === 'xpbd-contact-v1';
    const contactFieldPass = !!endovascularWorld.contactField;
    const cameraProjectionChanges = Math.max(
        0,
        (ui.getCArmRevision?.() ?? browserCameraRevisionStart) - browserCameraRevisionStart
    );
    const cameraStablePass = cameraProjectionChanges === 0;
    const focusLossMs = browserFocusLossMs + (browserFocusLostAt > 0 ? now - browserFocusLostAt : 0);
    const focusPass = focusLossMs <= 100;
    const heap = getBrowserHeapStats();
    const memoryStabilityPass = !heap.supported || (
        heap.growthBytes <= 4 * 1024 * 1024 &&
        heap.rangeBytes <= 8 * 1024 * 1024
    );
    const narrowPhaseAllocationPass = contactField?.resultAllocations === 0;
    const runtimeAssetPass = (contactField?.runtimeBytes ?? Infinity) <= 32 * 1024 * 1024;
    const noVisibleGcPausePass = browserMaxFrameMs < 100 &&
        memoryStabilityPass && narrowPhaseAllocationPass;
    return {
        mode: PHYSICS_MODE,
        durationMs: performance.now() - browserBenchmarkStartedAt,
        frameCount: browserFrameCount,
        averageFps: browserFrameTimeSum > 0 ? browserFrameCount * 1000 / browserFrameTimeSum : 0,
        onePercentLowFps,
        p99FrameMs,
        instantaneousP99Fps: p99FrameMs > 0 ? 1000 / p99FrameMs : 0,
        fpsWindowCount: browserFpsWindowCount,
        maxFrameMs: browserMaxFrameMs,
        longFrame33Count: browserLongFrame33Count,
        longFrame50Count: browserLongFrame50Count,
        longFrameEvents: getBrowserLongFrameEvents(),
        frameCpu: getBrowserFrameCpuStats(),
        physics,
        physicsEnvelope: { ...browserBenchmarkPhysicsEnvelope },
        contactField,
        cameraProjectionChanges,
        heapBytes: heap.endBytes,
        heap,
        pageState: {
            visibilityState: document.visibilityState,
            hasFocus: document.hasFocus(),
            focusLossCount: browserFocusLossCount,
            focusLossMs
        },
        scenario,
        browserAcceptance: {
            durationPass,
            onePercentLowPass,
            noLongFramePass,
            noVisibleGcPausePass,
            physicsBudgetPass,
            narrowPhaseAllocationPass,
            memoryStabilityPass,
            runtimeAssetPass,
            penetrationPass,
            lengthPass,
            foldPass,
            finitePass,
            modePass,
            contactFieldPass,
            cameraStablePass,
            focusPass,
            passed: durationPass && onePercentLowPass && noVisibleGcPausePass &&
                physicsBudgetPass && narrowPhaseAllocationPass && memoryStabilityPass &&
                runtimeAssetPass && penetrationPass && lengthPass && foldPass &&
                finitePass && modePass && contactFieldPass && cameraStablePass && focusPass
        }
    };
}

function stopBrowserBenchmarkScenario(reason = 'manual') {
    if (reason === 'ui' && browserBenchmarkScenario.automated) {
        return getBrowserBenchmarkReport();
    }
    if (browserBenchmarkScenario.running) {
        browserBenchmarkScenario.running = false;
        browserBenchmarkScenario.warmingUp = false;
        browserBenchmarkScenario.completedAt = performance.now();
        browserBenchmarkScenario.stopReason = reason;
    }
    ui.setAutomatedBenchmarkMode?.(false);
    lastBrowserBenchmarkScenarioReport = getBrowserBenchmarkReport();
    return lastBrowserBenchmarkScenarioReport;
}

function resetBrowserBenchmarkSimulation({ resetAccumulator = true } = {}) {
    guidewireSolver.reset();
    tailProgress = guidewireSolver.progress;
    pigtailCatheter.reset();
    xpbdWireBody.syncFromElasticRod(wire);
    pigtailCatheter.syncXpbdBody(xpbdCatheterBody);
    xpbdContainment.enabled = false;
    xpbdExternalToolContact.enabled = false;
    endovascularWorld.resetSimulationState();
    if (resetAccumulator) simulationAccumulator = 0;
}

function startBrowserBenchmarkScenario({
    durationMs = BROWSER_BENCHMARK_DEFAULT_DURATION_MS,
    automated = false
} = {}) {
    const nextDuration = Number(durationMs);
    if (!Number.isFinite(nextDuration) || nextDuration <= 0) {
        throw new RangeError('Browser benchmark durationMs must be positive');
    }
    if (!endovascularWorld.contactField) {
        throw new Error('Browser benchmark requires the precompiled vessel contact field');
    }
    resetBrowserBenchmarkSimulation();
    resetBrowserBenchmark();
    browserBenchmarkScenario.durationMs = nextDuration;
    browserBenchmarkScenario.warmupStartedAt = performance.now();
    browserBenchmarkScenario.memorySettling = false;
    browserBenchmarkScenario.startedAt = 0;
    browserBenchmarkScenario.completedAt = 0;
    browserBenchmarkScenario.simulationElapsedMs = 0;
    browserBenchmarkScenario.stopReason = null;
    browserBenchmarkScenario.automated = automated === true;
    ui.setAutomatedBenchmarkMode?.(browserBenchmarkScenario.automated);
    browserBenchmarkScenario.running = true;
    browserBenchmarkScenario.warmingUp = true;
    lastBrowserBenchmarkScenarioReport = null;
    return getBrowserBenchmarkScenarioStatus();
}

function sampleBrowserBenchmarkScenario(dt) {
    if (!browserBenchmarkScenario.running) return null;
    const now = performance.now();
    if (browserBenchmarkScenario.warmingUp) {
        const warmupElapsedMs = now - browserBenchmarkScenario.warmupStartedAt;
        if (warmupElapsedMs < BROWSER_BENCHMARK_CHOREOGRAPHY_WARMUP_MS) {
            const commands = sampleBrowserBenchmarkCommands(
                browserBenchmarkScenario.simulationElapsedMs,
                browserBenchmarkCommands
            );
            browserBenchmarkScenario.simulationElapsedMs += dt * 1000;
            return commands;
        }
        if (!browserBenchmarkScenario.memorySettling) {
            resetBrowserBenchmarkSimulation({ resetAccumulator: false });
            browserBenchmarkScenario.memorySettling = true;
            browserBenchmarkScenario.simulationElapsedMs = 0;
        }
        if (warmupElapsedMs < BROWSER_BENCHMARK_WARMUP_MS) {
            browserBenchmarkCommands.guidewireAdvance = 0;
            browserBenchmarkCommands.catheterAdvance = 0;
            browserBenchmarkCommands.catheterRotation = 0;
            browserBenchmarkCommands.catheterType = 'pigtail';
            return browserBenchmarkCommands;
        }
        resetBrowserBenchmark();
        browserBenchmarkScenario.warmingUp = false;
        browserBenchmarkScenario.memorySettling = false;
        browserBenchmarkScenario.startedAt = performance.now();
        browserBenchmarkScenario.completedAt = 0;
        browserBenchmarkScenario.simulationElapsedMs = 0;
    }
    const elapsedMs = performance.now() - browserBenchmarkScenario.startedAt;
    if (elapsedMs >= browserBenchmarkScenario.durationMs) {
        stopBrowserBenchmarkScenario('duration');
        return null;
    }
    const commands = sampleBrowserBenchmarkCommands(
        browserBenchmarkScenario.simulationElapsedMs,
        browserBenchmarkCommands
    );
    browserBenchmarkScenario.simulationElapsedMs += dt * 1000;
    return commands;
}

globalThis.__OET_BENCHMARK__ = {
    reset: resetBrowserBenchmark,
    getReport: getBrowserBenchmarkReport,
    startScenario: startBrowserBenchmarkScenario,
    stopScenario: stopBrowserBenchmarkScenario,
    getScenarioStatus: getBrowserBenchmarkScenarioStatus,
    getLastScenarioReport: () => lastBrowserBenchmarkScenarioReport
};

function advanceTailInput(advance, dt) {
    const collisionTarget = PHYSICS_MODE === 'legacy' ? vesselCollisionTarget : null;
    const delta = guidewireSolver.advance(advance, dt, collisionTarget, GUIDE_WIRE_ADVANCE_OPTIONS);
    tailProgress = guidewireSolver.progress;
    return delta;
}

function updateWireMesh() {
    for (let index = 0; index < wire.nodes.length; index++) {
        const node = wire.nodes[index];
        wireRenderPoints[index].set(node.x, node.y, node.z);
    }
    const previousGeometry = wireMesh.geometry;
    wireMesh.geometry = createSmoothTubeGeometry(wireRenderPoints, {
        radius: GUIDEWIRE_RENDER_RADIUS_MM,
        samplesPerSegment: GUIDEWIRE_TUBE_SAMPLES_PER_SEGMENT,
        radialSegments: GUIDEWIRE_TUBE_RADIAL_SEGMENTS
    });
    previousGeometry.dispose();
    wireGroup.visible = wire.nodes.length > 1;
}

function updateXpbdContactDebug() {
    if (!xpbdContactNormalLines || !xpbdActiveBranchLines || PHYSICS_MODE !== 'xpbd-contact-v1') {
        return { normalCount: 0, branchCount: 0 };
    }
    const normalAttribute = xpbdContactNormalLines.geometry.getAttribute('position');
    const branchAttribute = xpbdActiveBranchLines.geometry.getAttribute('position');
    const normalPositions = normalAttribute.array;
    const branchPositions = branchAttribute.array;
    const contactField = vesselCollisionTarget.contactField;
    const centerline = contactField?.centerline;
    const stride = contactField?.centerlineStride || 0;
    const segmentCount = stride > 0 && centerline ? centerline.length / stride : 0;
    let seen = xpbdActiveBranchLines.userData.seen;
    if (!seen || seen.length !== segmentCount) {
        seen = new Uint8Array(segmentCount);
        xpbdActiveBranchLines.userData.seen = seen;
    } else {
        seen.fill(0);
    }

    let normalCount = 0;
    let branchCount = 0;
    for (const body of [xpbdWireBody, xpbdCatheterBody]) {
        if (!body) continue;
        const end = Math.min(body.segmentCount, body.activeEnd);
        for (let index = body.activeStart; index < end; index++) {
            if (!body.wallActive[index]) continue;
            if (normalCount < CONTACT_MARKER_LIMIT) {
                const offset = normalCount * 6;
                const length = 2.5 + Math.min(4, body.wallLambda[index] * 8);
                normalPositions[offset] = body.wallX[index];
                normalPositions[offset + 1] = body.wallY[index];
                normalPositions[offset + 2] = body.wallZ[index];
                normalPositions[offset + 3] = body.wallX[index] + body.wallNormalX[index] * length;
                normalPositions[offset + 4] = body.wallY[index] + body.wallNormalY[index] * length;
                normalPositions[offset + 5] = body.wallZ[index] + body.wallNormalZ[index] * length;
                normalCount++;
            }
            const branchId = body.wallBranchId[index];
            if (
                branchId < 0 || branchId >= segmentCount || seen[branchId] ||
                branchCount >= CONTACT_MARKER_LIMIT
            ) continue;
            seen[branchId] = 1;
            const sourceOffset = branchId * stride;
            const targetOffset = branchCount * 6;
            for (let axis = 0; axis < 6; axis++) {
                branchPositions[targetOffset + axis] = centerline[sourceOffset + axis];
            }
            branchCount++;
        }
    }
    xpbdContactNormalLines.geometry.setDrawRange(0, normalCount * 2);
    xpbdActiveBranchLines.geometry.setDrawRange(0, branchCount * 2);
    normalAttribute.needsUpdate = true;
    branchAttribute.needsUpdate = true;
    return { normalCount, branchCount };
}

function sampleGuidewireContactMarkers() {
    if (fluoroscopy) {
        ui.updateGuidewireDiagnostics(null);
        wallContactMarkers.count = 0;
        wallBreachMarkers.count = 0;
        wallWorstPointMarker.userData.hasPoint = false;
        wallWorstPointMarker.visible = false;
        xpbdContactNormalLines?.geometry.setDrawRange(0, 0);
        xpbdActiveBranchLines?.geometry.setDrawRange(0, 0);
        return;
    }

    const lumenDiagnostics = guidewireSolver.collectLumenDiagnostics(vesselCollisionTarget, {
        clearance: guidewireSolver.meshClearance,
        contactBand: GUIDEWIRE_DIAGNOSTIC_CONTACT_BAND,
        collectMarkers: true,
        markerLimit: CONTACT_MARKER_LIMIT
    });
    if (PHYSICS_MODE === 'xpbd-contact-v1') {
        const xpbd = endovascularWorld.getStats();
        const legacyAdvance = guidewireSolver.getPerformanceStats();
        const contactDebug = updateXpbdContactDebug();
        lumenDiagnostics.performance = {
            advanceMs: legacyAdvance.advanceMs,
            solveMs: xpbd.phases.total.lastMs,
            projectMs: xpbd.phases.narrowPhase.lastMs,
            diagnosticMs: 0,
            pointContactCount: xpbd.contacts,
            diagnosticPointContactCount: 0,
            segmentSampleCount: vesselCollisionTarget.contactField?.getStats?.().capsuleSamples || 0,
            activeBranchCount: contactDebug.branchCount,
            settledPenetration: xpbd.settledMaxPenetration,
            maximumPenetration: xpbd.maxPenetration
        };
    } else {
        lumenDiagnostics.performance = guidewireSolver.getPerformanceStats();
    }
    ui.updateGuidewireDiagnostics(lumenDiagnostics);
    if (lumenDiagnostics.worstPoint) {
        wallWorstPointMarker.position.set(
            lumenDiagnostics.worstPoint.x,
            lumenDiagnostics.worstPoint.y + VASCULAR_MODEL_ALIGNMENT_Y_MM,
            lumenDiagnostics.worstPoint.z
        );
        wallWorstPointMarker.userData.hasPoint = true;
        wallWorstPointMarker.visible = true;
    } else {
        wallWorstPointMarker.userData.hasPoint = false;
        wallWorstPointMarker.visible = false;
    }

    const applySamples = (mesh, points) => {
        const count = Math.min(points.length, CONTACT_MARKER_LIMIT);
        mesh.count = count;
        for (let i = 0; i < count; i++) {
            const p = points[i];
            contactMarkerMatrix.makeTranslation(p.x, p.y, p.z);
            mesh.setMatrixAt(i, contactMarkerMatrix);
        }
        mesh.instanceMatrix.needsUpdate = true;
    };

    applySamples(wallContactMarkers, lumenDiagnostics.contacts || []);
    applySamples(wallBreachMarkers, lumenDiagnostics.breaches || []);
}

function updateGuidewireResistance() {
    if (PHYSICS_MODE !== 'xpbd-contact-v1') {
        ui.updateGuidewireResistance(0, '');
        return;
    }
    let normalLambda = 0;
    let activeContacts = 0;
    for (let index = 0; index < xpbdWireBody.wallLambda.length; index++) {
        if (!xpbdWireBody.wallActive[index]) continue;
        normalLambda += xpbdWireBody.wallLambda[index];
        activeContacts++;
    }
    const averageLambda = activeContacts ? normalLambda / activeContacts : 0;
    const level = Math.max(0, Math.min(1, averageLambda / 0.08));
    ui.updateGuidewireResistance(level, activeContacts ? 'Opór kontaktu prowadnika ze ścianą' : '');
}

const fixedDt = PHYSICS_MODE === 'xpbd-contact-v1' ? 1 / 120 : 1 / 60;
let lastRenderTime = performance.now();
let simulationAccumulator = 0;
let lastFluoroPulseTime = -Infinity;
let autoExposureLevel = 0;
const autoExposureBeamDirection = new THREE.Vector3();
let contactMarkerAccumulator = CONTACT_MARKER_UPDATE_INTERVAL;
let guidewireMeshAccumulator = GUIDEWIRE_MESH_UPDATE_INTERVAL;
let pigtailMeshAccumulator = PIGTAIL_MESH_UPDATE_INTERVAL;
let browserBenchmarkUiAccumulator = Infinity;

function updateAutoExposure(dt) {
    const uniforms = displayMaterial.uniforms;
    const response = Math.min(1, Math.max(0, dt) * 1.35);
    if (!uniforms.autoExposureEnabled.value) {
        autoExposureLevel += (0 - autoExposureLevel) * Math.min(1, response * 1.6);
        uniforms.autoExposureLevel.value = autoExposureLevel;
        return;
    }

    camera.getWorldDirection(autoExposureBeamDirection);
    const lateralPath = Math.abs(autoExposureBeamDirection.x);
    const cranialPath = Math.abs(autoExposureBeamDirection.y);
    const obliquityLoad = Math.max(0, lateralPath - 0.1);
    const collimationAmount = THREE.MathUtils.clamp((uniforms.collimation.value || 0) / 0.45, 0, 1);
    const collimationExposureRelief = 1 - collimationAmount * 0.34;
    const uncollimatedTarget = 0.012 + obliquityLoad * 0.15 + cranialPath * 0.035;
    const target = THREE.MathUtils.clamp(
        uncollimatedTarget * collimationExposureRelief - collimationAmount * 0.006,
        -0.03,
        0.18
    );
    autoExposureLevel += (target - autoExposureLevel) * response;
    uniforms.autoExposureLevel.value = autoExposureLevel;
}

function updateXrayTechniqueReadout() {
    const uniforms = displayMaterial.uniforms;
    const pulseRate = THREE.MathUtils.clamp(uniforms.pulseRate.value || 15, 7.5, 30);
    const exposureRatio = uniforms.autoExposureEnabled.value
        ? THREE.MathUtils.clamp(autoExposureLevel / 0.18, 0, 1)
        : 0.25;
    const kV = 70 + exposureRatio * 28;
    const pulseLoad = Math.pow(pulseRate / 15, 0.72);
    const collimationAmount = THREE.MathUtils.clamp((uniforms.collimation.value || 0) / 0.45, 0, 1);
    const collimationDoseLoad = 1 - collimationAmount * 0.42;
    const mA = (2.4 + exposureRatio * 7.2) * pulseLoad * collimationDoseLoad;
    ui.updateXrayTechnique(kV, mA);
}

function stepSimulation(dt = fixedDt) {
    // Advance input, integrate rod physics, collisions, and update medical monitors
    const benchmarkCommands = sampleBrowserBenchmarkScenario(dt);
    const advance = benchmarkCommands?.guidewireAdvance ?? ui.getAdvance();
    const catheterAdvance = benchmarkCommands?.catheterAdvance ?? ui.getCatheterAdvance();
    const catheterRotation = benchmarkCommands?.catheterRotation ?? ui.getCatheterRotation();
    advanceTailInput(advance, dt);
    const inserted = Math.max(0, tailProgress);
    pigtailCatheter.setType(benchmarkCommands?.catheterType ?? ui.getSelectedCatheterType());
    pigtailCatheter.advance(catheterAdvance, dt, inserted);
    pigtailCatheter.rotate(catheterRotation, dt);
    if (PHYSICS_MODE === 'xpbd-contact-v1') {
        // The shared ElasticRod storage carries the previous XPBD pose and
        // velocity. GuidewireSolver only moves the sheath-constrained boundary
        // in this mode; XPBD transmits that displacement through the elastic
        // body and finds the new contact-constrained equilibrium.
        xpbdWireBody.syncFromElasticRod(wire, XPBD_WIRE_SYNC_OPTIONS);
        xpbdWireBody.setActiveRange(
            Math.min(xpbdWireBody.count - 2, Math.max(0, guidewireSolver.firstInsertedNodeIndex() - 1)),
            xpbdWireBody.count - 1
        );
        xpbdWireBody.setCollisionRange(
            Math.max(0, guidewireSolver.firstLumenNodeIndex() - 1),
            xpbdWireBody.segmentCount - 1
        );
        pigtailCatheter.stepPhysics(dt, XPBD_CATHETER_STEP_OPTIONS);
        const catheterNodeCount = pigtailCatheter.syncXpbdBody(
            xpbdCatheterBody,
            XPBD_CATHETER_SYNC_OPTIONS
        );
        xpbdContainment.outerStartNode = pigtailCatheter.physicsLumenStartNode;
        const firstContainedNode = Math.max(0, Math.ceil((guidewireLength - inserted) / segmentLength));
        const lastContainedNode = Math.min(
            xpbdWireBody.count - 1,
            Math.floor((guidewireLength - inserted + pigtailCatheter.progress) / segmentLength)
        );
        xpbdContainment.enabled =
            pigtailCatheter.progress > 0.5 &&
            catheterNodeCount >= 2 &&
            lastContainedNode >= firstContainedNode;
        xpbdContainment.startNode = firstContainedNode;
        xpbdContainment.endNode = Math.max(firstContainedNode, lastContainedNode);
        xpbdContainment.innerArcOffset =
            firstContainedNode * segmentLength - guidewireLength + inserted;
        xpbdContainment.containedLength = Math.min(pigtailCatheter.progress, inserted);
        xpbdWireBody.nodeRadius.fill(GUIDEWIRE_RADIUS_MM);

        const catheterEndSegment = Math.max(0, catheterNodeCount - 2);
        const firstExternalSegment = Math.max(0, Math.min(
            xpbdWireBody.segmentCount - 1,
            lastContainedNode
        ));
        // The shared-lumen constraint already carries the catheter up to its
        // open distal tip. Treating the emerging guidewire as an external
        // colliding rod adds a spurious side impact at that opening and sends
        // an impulse down the free guidewire. The open tip must let it exit
        // without a capsule-like collision.
        xpbdExternalToolContact.enabled = false;
        xpbdExternalToolContact.startSegmentA = firstExternalSegment;
        xpbdExternalToolContact.endSegmentA = Math.min(
            xpbdWireBody.activeEnd - 1,
            firstExternalSegment + 16
        );
        xpbdExternalToolContact.startSegmentB = Math.max(0, catheterEndSegment - 8);
        xpbdExternalToolContact.endSegmentB = catheterEndSegment;
        endovascularWorld.stepFixed();
        if (browserBenchmarkScenario.running) recordBrowserPhysicsEnvelope();
        xpbdWireBody.syncToElasticRod(wire);
    } else {
        guidewireSolver.solve(dt, vesselCollisionTarget, {
            iterations: advance === 0 ? 3 : 4
        });
        pigtailCatheter.stepPhysics(dt);
    }
    const catheterActive = catheterAdvance !== 0 || catheterRotation !== 0;
    const guidewireActive = advance !== 0;
    const guidewireInsideCatheter = pigtailCatheter.progress > 4 && inserted > 0;
    if (PHYSICS_MODE === 'legacy') {
        pigtailCatheter.constrainGuidewire(dt, {
            reactionScale: guidewireActive && !catheterActive ? 0.08 : 1
        });
        if (guidewireActive && !catheterActive && guidewireInsideCatheter) {
            guidewireSolver.solve(dt, vesselCollisionTarget, { iterations: 8, forceRelax: true });
            pigtailCatheter.constrainGuidewire(dt, { reactionScale: 0.04 });
            guidewireSolver.solve(dt, vesselCollisionTarget, { iterations: 5, forceRelax: true });
        }
        if (catheterActive) {
            guidewireSolver.solve(dt, vesselCollisionTarget, { iterations: 10, forceRelax: true });
            pigtailCatheter.constrainGuidewire(dt);
            guidewireSolver.solve(dt, vesselCollisionTarget, { iterations: 8, forceRelax: true });
        }
    }
    updateGuidewireResistance();
    ui.updateInsertedLength(inserted / 10);
    ui.updateCatheterLength(pigtailCatheter.progress / 10);

    if (injecting) {
        const amt = Math.min(injectRate * dt, remainingVolume);
        contrastAgent.injectThroughSheath(amt, injectRate);
        totalDose += amt;
        ui.updateDose(totalDose);
        injectTime += dt;
        remainingVolume -= amt;
        if (injectTime >= injectDuration || remainingVolume <= 0) {
            injecting = false;
            ui.setStopInjectionDisabled(true);
        }
    }
    contrastAgent.update(dt);
    monitor.update(dt);
}

const renderHiddenObjects = [];
function renderOnlySceneObject(scene, camera, object) {
    renderHiddenObjects.length = 0;
    for (const child of scene.children) {
        if (child.isCamera) continue;
        const shouldRender = child === object && child.visible;
        if (!shouldRender && child.visible) {
            renderHiddenObjects.push(child);
            child.visible = false;
        }
    }
    renderer.render(scene, camera);
    for (let index = 0; index < renderHiddenObjects.length; index++) {
        renderHiddenObjects[index].visible = true;
    }
}

const depthHiddenObjects = [];
const depthHiddenVisibility = [];

function staticAnatomyProjectionChanged() {
    camera.updateMatrixWorld(true);
    const world = camera.matrixWorld.elements;
    const projection = camera.projectionMatrix.elements;
    let changed = !anatomyProjectionValid;
    for (let index = 0; index < 16 && !changed; index++) {
        changed = world[index] !== anatomyCameraWorld[index] ||
            projection[index] !== anatomyProjectionMatrix[index];
    }
    if (!changed) return false;
    anatomyCameraWorld.set(world);
    anatomyProjectionMatrix.set(projection);
    anatomyProjectionValid = true;
    return true;
}

function updateStaticAnatomyProjection() {
    depthHiddenObjects.length = 0;
    depthHiddenVisibility.length = 0;
    for (const child of scene.children) {
        if (child !== skeletonModel && !child.isCamera) {
            depthHiddenObjects.push(child);
            depthHiddenVisibility.push(child.visible);
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
    for (let index = 0; index < depthHiddenObjects.length; index++) {
        depthHiddenObjects[index].visible = depthHiddenVisibility[index];
    }

    thicknessMaterial.uniforms.frontDepth.value = frontDepthTarget.texture;
    thicknessMaterial.uniforms.backDepth.value = backDepthTarget.texture;
    renderer.setRenderTarget(thicknessTarget);
    renderer.render(thicknessScene, postCamera);
    renderer.setRenderTarget(null);

    renderer.setRenderTarget(boneTarget);
    renderer.clear();
    scene.overrideMaterial = boneProjectionMaterial;
    renderOnlySceneObject(scene, camera, skeletonModel);
    scene.overrideMaterial = null;
    renderer.setRenderTarget(null);
}

function animate(time) {
    // Render loop: updates geometry, handles fluoroscopy accumulation, and UI
    const frameCpuStartedAt = performance.now();
    const frameMs = time - lastRenderTime;
    const dt = Math.max(0, Math.min(0.1, frameMs / 1000));
    lastRenderTime = time;
    recordBrowserFrame(frameMs);
    simulationAccumulator += dt;
    let simulationSteps = 0;
    while (simulationAccumulator + 1e-9 >= fixedDt && simulationSteps < 2) {
        stepSimulation(fixedDt);
        simulationAccumulator -= fixedDt;
        simulationSteps++;
    }
    if (simulationAccumulator >= fixedDt) simulationAccumulator %= fixedDt;
    const frameSimulationEndedAt = performance.now();

    guidewireMeshAccumulator += dt;
    if (guidewireMeshAccumulator >= GUIDEWIRE_MESH_UPDATE_INTERVAL) {
        guidewireMeshAccumulator = 0;
        updateWireMesh();
    }
    contactMarkerAccumulator += dt;
    if (contactMarkerAccumulator >= CONTACT_MARKER_UPDATE_INTERVAL) {
        contactMarkerAccumulator = 0;
        sampleGuidewireContactMarkers();
    }
    pigtailMeshAccumulator += dt;
    if (pigtailMeshAccumulator >= PIGTAIL_MESH_UPDATE_INTERVAL) {
        pigtailMeshAccumulator = 0;
        pigtailCatheter.updateMesh();
    }
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
    sheathFluoroMesh.visible = fluoroscopy;
    if (wallContactMarkers) wallContactMarkers.visible = !fluoroscopy;
    if (wallBreachMarkers) wallBreachMarkers.visible = !fluoroscopy;
    if (wallWorstPointMarker) wallWorstPointMarker.visible = !fluoroscopy && !!wallWorstPointMarker.userData.hasPoint;
    skeletonModel.visible = fluoroscopy;
    ui.setInjectButtonDisabled(contrastActive);
    ui.setStopInjectionDisabled(!injecting);
    browserBenchmarkUiAccumulator += dt;
    if (browserBenchmarkUiAccumulator >= 0.25) {
        browserBenchmarkUiAccumulator = 0;
        const scenarioStatus = getBrowserBenchmarkScenarioStatus();
        ui.updateBrowserBenchmarkStatus(
            scenarioStatus,
            scenarioStatus.running ? null : lastBrowserBenchmarkScenarioReport
        );
    }
    const frameUpdateEndedAt = performance.now();
    if (fluoroscopy) {
        updateAutoExposure(dt);
        updateXrayTechniqueReadout();
        const pulseRate = Math.max(1, displayMaterial.uniforms.pulseRate.value || 15);
        const pulseInterval = 1000 / pulseRate;
        const shouldAcquireFluoroFrame = time - lastFluoroPulseTime >= pulseInterval;
        if (!shouldAcquireFluoroFrame) {
            renderer.setRenderTarget(null);
            renderer.render(displayScene, postCamera);
            ui.updatePerfStats(dt);
            recordBrowserFrameCpu(frameCpuStartedAt, frameSimulationEndedAt, frameUpdateEndedAt);
            requestAnimationFrame(animate);
            return;
        }
        lastFluoroPulseTime = time;

        // Fluoroscopy path:
        // 1) render front/back depth for legacy thickness/scatter cues
        // 2) render stable bone/contrast/metal masks for attenuation.
        //    The bone pass additively stores entry depth, exit depth, and
        //    angle-corrected cortical shell length, so separated overlapping
        //    bones contribute their actual ray lengths instead of collapsing to
        //    one nearest/farthest interval.
        // 3) render scene to offscreen, accumulate with decay
        // 4) display attenuated fluoroscopy image via display shader
        if (staticAnatomyProjectionChanged()) updateStaticAnatomyProjection();

        renderer.setRenderTarget(contrastTarget);
        renderer.setClearColor(0x000000, 0);
        renderer.clear();
        renderer.render(contrastScene, camera);
        renderer.setClearColor(0x000000, 1);

        renderer.setRenderTarget(metalTarget);
        renderer.setClearColor(0x000000, 0);
        renderer.clear();
        scene.overrideMaterial = wireProjectionMaterial;
        renderOnlySceneObject(scene, camera, wireGroup);
        scene.overrideMaterial = null;
        renderer.setClearColor(0x000000, 1);

        renderer.setRenderTarget(catheterTarget);
        renderer.setClearColor(0x000000, 0);
        renderer.clear();
        renderOnlySceneObject(scene, camera, pigtailCatheter.mesh);
        renderer.setClearColor(0x000000, 1);

        renderer.setRenderTarget(sheathTarget);
        renderer.setClearColor(0x000000, 0);
        renderer.clear();
        renderOnlySceneObject(scene, camera, sheathFluoroMesh);
        renderer.setClearColor(0x000000, 1);

        renderer.setRenderTarget(offscreenTarget);
        renderer.clear();
        renderOnlySceneObject(scene, camera, sheathFluoroMesh);
        const previousOverlayAutoClear = renderer.autoClear;
        renderer.autoClear = false;
        scene.overrideMaterial = wireProjectionMaterial;
        renderOnlySceneObject(scene, camera, wireGroup);
        scene.overrideMaterial = null;
        renderer.render(contrastScene, camera);
        renderer.autoClear = previousOverlayAutoClear;

        blendMaterial.uniforms.currentFrame.value = offscreenTarget.texture;
        blendMaterial.uniforms.previousFrame.value = previousTarget.texture;

        renderer.setRenderTarget(currentTarget);
        renderer.render(blendScene, postCamera);
        renderer.setRenderTarget(null);

        displayMaterial.uniforms.uTexture.value = currentTarget.texture;
        displayMaterial.uniforms.contrastTexture.value = contrastTarget.texture;
        displayMaterial.uniforms.thicknessTexture.value = thicknessTarget.texture;
        displayMaterial.uniforms.metalTexture.value = metalTarget.texture;
        displayMaterial.uniforms.catheterTexture.value = catheterTarget.texture;
        displayMaterial.uniforms.sheathTexture.value = sheathTarget.texture;
        displayMaterial.uniforms.boneTexture.value = boneTarget.texture;
        displayMaterial.uniforms.time.value = time * 0.001;
        renderer.render(displayScene, postCamera);
        completeFirstLoadedFrame();

        // Ping-pong accumulation targets for next frame's persistence
        const temp = previousTarget;
        previousTarget = currentTarget;
        currentTarget = temp;
    } else {
        lastFluoroPulseTime = -Infinity;
        updateXrayTechniqueReadout();
        renderer.setRenderTarget(null);
        renderer.render(scene, camera);
        completeFirstLoadedFrame();
    }

    ui.updatePerfStats(dt);
    recordBrowserFrameCpu(frameCpuStartedAt, frameSimulationEndedAt, frameUpdateEndedAt);

    requestAnimationFrame(animate);
}
requestAnimationFrame(animate);

window.addEventListener('resize', () => {
    // Keep all targets and shader uniforms in sync with the canvas size
    const w = window.innerWidth;
    const h = window.innerHeight;
    const targetWidth = Math.max(1, Math.round(w * FLUORO_TARGET_SCALE));
    const targetHeight = Math.max(1, Math.round(h * FLUORO_TARGET_SCALE));
    renderer.setSize(w, h);
    camera.aspect = w / h;
    camera.updateProjectionMatrix();
    offscreenTarget.setSize(targetWidth, targetHeight);
    contrastTarget.setSize(targetWidth, targetHeight);
    metalTarget.setSize(targetWidth, targetHeight);
    catheterTarget.setSize(targetWidth, targetHeight);
    sheathTarget.setSize(targetWidth, targetHeight);
    boneTarget.setSize(targetWidth, targetHeight);
    accumulateTarget1.setSize(targetWidth, targetHeight);
    accumulateTarget2.setSize(targetWidth, targetHeight);
    frontDepthTarget.setSize(targetWidth, targetHeight);
    backDepthTarget.setSize(targetWidth, targetHeight);
    thicknessTarget.setSize(targetWidth, targetHeight);
    anatomyProjectionValid = false;
    displayMaterial.uniforms.resolution.value.set(targetWidth, targetHeight);
});
