// Main simulator entry: sets up scenes, physics, rendering passes, and UI.
import * as THREE from 'three';
import { CSS2DRenderer } from 'three/examples/jsm/renderers/CSS2DRenderer.js';
import { ElasticRod } from './physics/elasticRod.js';
import { GuidewireSolver } from './physics/guidewireSolver.js';
import {
    applyGuidewireMaterialProfile,
    GUIDEWIRE_TYPE_GLIDEWIRE,
    normalizeGuidewireType
} from './physics/guidewireMaterialProfile.js';
import { applyKirchhoffMaterialProfile } from './physics/applyKirchhoffMaterialProfile.js';
import { applyProximalTwistBoundary } from './physics/kirchhoffOrientationBoundary.js';
import { GuidewireResistanceEstimator } from './physics/guidewireResistance.js';
import {
    clampGuidewireRelaxationRate,
    guidewireRelaxationPasses
} from './physics/guidewireRelaxationRate.js';
import {
    DEFAULT_TOOL_PROFILES,
    EndovascularPhysicsWorld
} from './physics/endovascularPhysicsWorld.js';
import {
    buildContainedGuidewireRenderPolyline,
    firstFreeGuidewireNodeAfterContainment,
    spatiallyCapturedContainmentEnd
} from './physics/catheterGuidewireCoupling.js';
import { generateVessel } from './vesselGeometry.js';
import { initUI } from './ui/ui.js';
import { createBoneModel } from './boneModel.js';
import { ContrastVolumeRenderer } from './contrast/contrastVolumeRenderer.js';
import { HybridContrastSystem } from './contrast/hybridContrastSystem.js';
import {
    CATHETER_PROXIMAL_LOADING_SUPPORT_LENGTH_MM,
    PigtailCatheter
} from './pigtailCatheter.js';
import { createAortaModel } from './aortaModel.js';
import { createAortoiliacDebugLabelGroup } from './anatomyDebugLabels.js';
import { createBroadPhaseDebugGroup } from './vesselBroadPhase.js';
import { updateSmoothTubeGeometry } from './smoothTubeGeometry.js';
import {
    DsaRoadmapState,
    scoreProjectedContrastRgba
} from './imaging/dsaRoadmapState.js';
import {
    dsaArchiveDimensions,
    dsaScoreDimensions
} from './imaging/dsaCaptureSizing.js';
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
    BROWSER_BENCHMARK_MODE_COUPLED,
    BROWSER_BENCHMARK_MODE_GUIDEWIRE,
    BROWSER_BENCHMARK_SCENARIO_CYCLE_MS,
    GUIDEWIRE_BROWSER_BENCHMARK_CYCLE_MS,
    browserBenchmarkCatheterType,
    createBrowserBenchmarkCommands,
    sampleBrowserBenchmarkCommands,
    sampleGuidewireBrowserBenchmarkCommands
} from './benchmark/browserBenchmarkScenario.js';
import {
    ARCH_BOLUS_CATHETER_TARGET_MM,
    ARCH_BOLUS_GUIDEWIRE_TARGET_MM,
    createCatheterAortaSetupState,
    ILIAC_BUG_CATHETER_TARGET_MM,
    ILIAC_BUG_GUIDEWIRE_TARGET_MM,
    RETROGRADE_GAP_CATHETER_TARGET_MM,
    RETROGRADE_GAP_GUIDEWIRE_TARGET_MM,
    sampleCatheterAortaSetup,
    startCatheterAortaSetup,
    stopCatheterAortaSetup
} from './benchmark/catheterAortaSetup.js';

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
const LEGACY_GUIDEWIRE_TOOL_COUPLED_PROJECTION_RETENTION = 0.005;
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
const anatomyLabelRenderer = new CSS2DRenderer();
anatomyLabelRenderer.setSize(window.innerWidth, window.innerHeight);
anatomyLabelRenderer.domElement.className = 'anatomy-label-layer';
anatomyLabelRenderer.domElement.setAttribute('aria-hidden', 'true');
anatomyLabelRenderer.domElement.style.display = 'none';
document.body.appendChild(anatomyLabelRenderer.domElement);
const FLUORO_TARGET_SCALE = 0.85;
const fluoroscopyTargetWidth = () => Math.max(1, Math.round(window.innerWidth * FLUORO_TARGET_SCALE));
const fluoroscopyTargetHeight = () => Math.max(1, Math.round(window.innerHeight * FLUORO_TARGET_SCALE));
const initialFluoroTargetWidth = fluoroscopyTargetWidth();
const initialFluoroTargetHeight = fluoroscopyTargetHeight();
const DSA_SCORE_MAX_DIMENSION = 256;
const initialDsaArchiveDimensions = dsaArchiveDimensions(
    initialFluoroTargetWidth,
    initialFluoroTargetHeight
);
const initialDsaScoreDimensions = dsaScoreDimensions(
    initialFluoroTargetWidth,
    initialFluoroTargetHeight,
    DSA_SCORE_MAX_DIMENSION
);
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
const catheterMarkerTarget = new THREE.WebGLRenderTarget(initialFluoroTargetWidth, initialFluoroTargetHeight, deviceMaskTargetOptions);
const sheathTarget = new THREE.WebGLRenderTarget(initialFluoroTargetWidth, initialFluoroTargetHeight, deviceMaskTargetOptions);
const boneTarget = new THREE.WebGLRenderTarget(initialFluoroTargetWidth, initialFluoroTargetHeight, {
    type: THREE.HalfFloatType
});
const accumulateTarget1 = new THREE.WebGLRenderTarget(initialFluoroTargetWidth, initialFluoroTargetHeight);
const accumulateTarget2 = new THREE.WebGLRenderTarget(initialFluoroTargetWidth, initialFluoroTargetHeight);
const frontDepthTarget = new THREE.WebGLRenderTarget(initialFluoroTargetWidth, initialFluoroTargetHeight);
const backDepthTarget = new THREE.WebGLRenderTarget(initialFluoroTargetWidth, initialFluoroTargetHeight);
const thicknessTarget = new THREE.WebGLRenderTarget(initialFluoroTargetWidth, initialFluoroTargetHeight);
const dsaMaskTarget = new THREE.WebGLRenderTarget(initialFluoroTargetWidth, initialFluoroTargetHeight);
const roadmapTarget = new THREE.WebGLRenderTarget(initialFluoroTargetWidth, initialFluoroTargetHeight);
const dsaFrameCaptureTarget = new THREE.WebGLRenderTarget(
    initialDsaArchiveDimensions.width,
    initialDsaArchiveDimensions.height
);
const dsaContrastScoreTarget = new THREE.WebGLRenderTarget(
    initialDsaScoreDimensions.width,
    initialDsaScoreDimensions.height
);
let dsaFrameReadback = new Uint8Array(
    initialDsaArchiveDimensions.width *
    initialDsaArchiveDimensions.height * 4
);
let dsaContrastScoreReadback = new Uint8Array(
    initialDsaScoreDimensions.width *
    initialDsaScoreDimensions.height * 4
);
const dsaSequenceFrameTextures = new Map();
const dsaSequencePreviewUrls = new Map();
let previousTarget = accumulateTarget1;
let currentTarget = accumulateTarget2;
const dsaRoadmapState = new DsaRoadmapState();
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
        catheterMarkerTexture: { value: catheterMarkerTarget.texture },
        sheathTexture: { value: sheathTarget.texture },
        boneTexture: { value: boneTarget.texture },
        dsaMaskTexture: { value: dsaMaskTarget.texture },
        roadmapTexture: { value: roadmapTarget.texture },
        cineTexture: { value: roadmapTarget.texture },
        gray: { value: new THREE.Color(0xEBEBEB) },
        fluoroscopy: { value: false },
        dsaEnabled: { value: false },
        dsaMaskValid: { value: false },
        roadmapEnabled: { value: false },
        roadmapValid: { value: false },
        cineEnabled: { value: false },
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
        contrastGain: { value: 5.0 },
        dsaGain: { value: 2.2 },
        roadmapOpacity: { value: 0.72 },
        roadmapBackgroundVisibility: { value: 1.0 }

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
let xpbdPortalInnerDriven = true;
let catheterShaftStiffnessScale = 25;
let catheterTipStiffnessScale = 5;
let catheterRelaxationRate = 30;
let guidewireShaftStiffnessScale = 10;
let guidewireTipStiffnessScale = 4.55;
let guidewireRelaxationRate = 30;
const MIN_CATHETER_STIFFNESS_SCALE = 0.25;
const MAX_CATHETER_SHAFT_STIFFNESS_SCALE = 25;
const MAX_CATHETER_TIP_STIFFNESS_SCALE = 10;
const MIN_GUIDEWIRE_STIFFNESS_SCALE = 0.25;
const MAX_GUIDEWIRE_SHAFT_STIFFNESS_SCALE = 25;
const MAX_GUIDEWIRE_TIP_STIFFNESS_SCALE = 10;
let guidewireStaticWallFriction = DEFAULT_TOOL_PROFILES.guidewire.wallFriction;
let guidewireKineticWallFriction = 0.002;
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
        contrastSystem = new HybridContrastSystem({
            centerlineSegments: collision.centerlineBroadPhase.segments,
            contactField: collision.contactField,
            sheath: vessel.sheath,
            catheter: pigtailCatheter,
            hemodynamics: contrastHemodynamics
        });
        if (contrastHydraulicParameters) {
            contrastSystem.setInjectionHydraulicParameters(
                contrastHydraulicParameters
            );
        }
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
        lumenDebugGroup.add(createBroadPhaseDebugGroup(collision.centerlineBroadPhase, {
            flowEdges: contrastSystem.flowNetwork.edges
        }));
        lumenDebugGroup.add(
            createAortoiliacDebugLabelGroup(contrastSystem.flowNetwork)
        );
        lumenDebugGroup.add(createSheathEntryDebugMarker(collision, vessel.sheath));
        applyDebugLayerVisibility();
        guidewireSolver?.requestSettle?.(90);
        pigtailCatheter?.setCollisionGeometry(collision);
        contrastVolumeRenderer = new ContrastVolumeRenderer(contrastSystem);
        contrastVolumeRenderer.setDebugMode(!fluoroscopy);
        voxelGroup.add(contrastVolumeRenderer.group);
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

const voxelGroup = alignVascularRenderObject(new THREE.Group());
scene.add(voxelGroup);
let contrastSystem = null;
let contrastVolumeRenderer = null;
let contrastRenderAccumulator = 0;
const contrastHemodynamics = {
    cardiacOutputMlPerMin: 5000,
    heartRateBpm: 72
};
let contrastHydraulicParameters = null;

// Guidewire physical model (discrete elastic rod)
const segmentLength = 5;
const nodeCount = 201;
const guidewireLength = segmentLength * (nodeCount - 1);
const GUIDEWIRE_ADVANCE_RATE = 44;
const GUIDEWIRE_ROTATION_SPEED = Math.PI * 0.9;
let activeGuidewireType = GUIDEWIRE_TYPE_GLIDEWIRE;
let guidewireRotation = 0;
const guidewireShaftAxis = new THREE.Vector3(
    vessel.sheath.end.x - vessel.sheath.start.x,
    vessel.sheath.end.y - vessel.sheath.start.y,
    vessel.sheath.end.z - vessel.sheath.start.z
).normalize();
const guidewireIntrinsicReferenceAxis = new THREE.Vector3(0, 0, 1)
    .addScaledVector(guidewireShaftAxis, -guidewireShaftAxis.z);
if (guidewireIntrinsicReferenceAxis.lengthSq() < 1e-8) {
    guidewireIntrinsicReferenceAxis.set(1, 0, 0)
        .addScaledVector(guidewireShaftAxis, -guidewireShaftAxis.x);
}
guidewireIntrinsicReferenceAxis.normalize();
const guidewireMaterialCoordinates = Float64Array.from(
    { length: nodeCount },
    (_, index) => index * segmentLength
);

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
applyActiveGuidewireElasticProfile();
guidewireSolver.initialize();
tailProgress = guidewireSolver.progress;

let displayedContrastDoseMl = 0;
const guidewireResistanceEstimator = new GuidewireResistanceEstimator();
const guidewireResistanceResult = {};
const guidewireResistanceOptions = {
    dt: 0,
    command: 0,
    atMaximumInsertion: false
};
let lastGuidewireAdvanceCommand = 0;

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
    onStartInjection: ({ source, rate, volume }) => {
        if (!contrastSystem) {
            ui.setInjectionSourceStatus(false, 'Flow model is still loading');
            return;
        }
        const result = contrastSystem.startInjection({
            source,
            rateMlPerSec: rate,
            volumeMl: volume
        });
        if (!result.ok) ui.setInjectionSourceStatus(false, result.reason);
    },
    onStopInjection: () => {
        contrastSystem?.stopInjection();
    },
    onModeChange: (f) => {
        fluoroscopy = f;
        anatomyLabelRenderer.domElement.style.display =
            fluoroscopy ? 'none' : 'block';
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
        contrastVolumeRenderer?.setDebugMode(!fluoroscopy);
    },
    onDebugLayerChange: layers => {
        Object.assign(debugLayerVisibility, layers);
        applyDebugLayerVisibility();
        if (xpbdContactDebugGroup) {
            xpbdContactDebugGroup.visible = !fluoroscopy && !!debugLayerVisibility.capsules;
        }
    },
    onCatheterStiffnessChange: ({
        shaftStiffnessScale,
        tipStiffnessScale
    }) => {
        catheterShaftStiffnessScale = THREE.MathUtils.clamp(
            Number.isFinite(shaftStiffnessScale) ? shaftStiffnessScale : 1,
            MIN_CATHETER_STIFFNESS_SCALE,
            MAX_CATHETER_SHAFT_STIFFNESS_SCALE
        );
        catheterTipStiffnessScale = THREE.MathUtils.clamp(
            Number.isFinite(tipStiffnessScale) ? tipStiffnessScale : 1,
            MIN_CATHETER_STIFFNESS_SCALE,
            MAX_CATHETER_TIP_STIFFNESS_SCALE
        );
        pigtailCatheter?.setStiffnessScales({
            shaftStiffnessScale: catheterShaftStiffnessScale,
            tipStiffnessScale: catheterTipStiffnessScale
        });
    },
    onCatheterRelaxationChange: value => {
        catheterRelaxationRate = clampGuidewireRelaxationRate(value);
        xpbdCatheterBody?.wake();
    },
    onGuidewireStiffnessChange: ({
        shaftStiffnessScale,
        tipStiffnessScale
    }) => {
        guidewireShaftStiffnessScale = THREE.MathUtils.clamp(
            Number.isFinite(shaftStiffnessScale)
                ? shaftStiffnessScale
                : 1,
            MIN_GUIDEWIRE_STIFFNESS_SCALE,
            MAX_GUIDEWIRE_SHAFT_STIFFNESS_SCALE
        );
        guidewireTipStiffnessScale = THREE.MathUtils.clamp(
            Number.isFinite(tipStiffnessScale) ? tipStiffnessScale : 1,
            MIN_GUIDEWIRE_STIFFNESS_SCALE,
            MAX_GUIDEWIRE_TIP_STIFFNESS_SCALE
        );
        applyActiveGuidewireElasticProfile();
        if (xpbdWireBody) applyActiveGuidewireKirchhoffProfile();
    },
    onGuidewireRelaxationChange: value => {
        guidewireRelaxationRate = clampGuidewireRelaxationRate(value);
        xpbdWireBody?.wake();
    },
    onGuidewireFrictionChange: ({ staticFriction, kineticFriction }) => {
        guidewireStaticWallFriction = Math.max(0, staticFriction);
        guidewireKineticWallFriction = Math.max(0, kineticFriction);
        applyActiveGuidewireWallFriction();
    },
    onContrastHemodynamicsChange: parameters => {
        Object.assign(contrastHemodynamics, parameters);
        contrastSystem?.setHemodynamics(parameters);
    },
    onContrastInjectionParametersChange: parameters => {
        contrastHydraulicParameters = parameters;
        contrastSystem?.setInjectionHydraulicParameters(parameters);
    },
    onPrepareCatheterAorta: () => prepareCatheterAortaScenario(),
    onReproduceIliacContrastBug: () => prepareCatheterAortaScenario({
        reproduceIliacBug: true
    }),
    onReproduceRetrogradeGap: () => prepareCatheterAortaScenario({
        reproduceRetrogradeGap: true
    }),
    onReproduceArchBolus: () => prepareCatheterAortaScenario({
        reproduceArchBolus: true
    }),
    onStartBrowserBenchmark: options => startBrowserBenchmarkScenario(
        Number.isFinite(options)
            ? { durationMs: options }
            : options
    ),
    onStopBrowserBenchmark: () => stopBrowserBenchmarkScenario('ui'),
    onRequestDsaMask: () => {
        dsaRoadmapState.stopCine();
        const result = dsaRoadmapState.requestMaskCapture({
            contrastVisible: contrastSystem?.hasVisibleContrast?.() === true
        });
        syncDsaRoadmapState();
        return result;
    },
    onToggleDsa: () => {
        dsaRoadmapState.stopCine();
        const result = dsaRoadmapState.toggleDsa();
        syncDsaRoadmapState();
        return result;
    },
    onCaptureRoadmap: () => {
        dsaRoadmapState.stopCine();
        const result = dsaRoadmapState.requestRoadmapCapture({
            contrastVisible: contrastSystem?.hasVisibleContrast?.() === true
        });
        syncDsaRoadmapState();
        return result;
    },
    onToggleRoadmap: () => {
        dsaRoadmapState.stopCine();
        const result = dsaRoadmapState.toggleRoadmap();
        syncDsaRoadmapState();
        return result;
    },
    onClearRoadmap: () => {
        dsaRoadmapState.clearRoadmap();
        syncDsaRoadmapState();
    },
    onStartDsaRecording: () => startDsaSequenceRecording(),
    onStopDsaRecording: () => finishDsaSequenceRecording(),
    onSelectDsaSequence: sequenceId => {
        dsaRoadmapState.stopCine();
        const result = dsaRoadmapState.selectSequence(sequenceId);
        syncDsaRoadmapState();
        return result;
    },
    onSelectDsaFrame: (sequenceId, frameIndex) => {
        dsaRoadmapState.stopCine();
        const result = dsaRoadmapState.selectRoadmapFrame(
            sequenceId,
            frameIndex
        );
        syncDsaRoadmapState();
        return result;
    },
    onUseBestDsaFrame: sequenceId => {
        dsaRoadmapState.stopCine();
        const result = dsaRoadmapState.useBestFrame(sequenceId);
        syncDsaRoadmapState();
        return result;
    },
    onToggleDsaCine: sequenceId => {
        const result = dsaRoadmapState.toggleCine(sequenceId, {
            nowMs: performance.now()
        });
        syncDsaRoadmapState();
        return result;
    },
    onStopDsaCine: () => {
        const result = dsaRoadmapState.stopCine();
        syncDsaRoadmapState();
        return result;
    },
});
const { monitor } = ui;
globalThis.__OET_MONITOR__ = {
    timingDiagnostics: () => monitor.timingDiagnostics()
};

function dsaFrameTextureKey(sequenceId, frameIndex) {
    return `${sequenceId}:${frameIndex}`;
}

function pruneDsaFrameTextures(snapshot) {
    const retainedKeys = new Set(
        snapshot.sequences.flatMap(sequence =>
            sequence.frames.map(frame => frame.storageKey)
        )
    );
    for (const [storageKey, texture] of dsaSequenceFrameTextures) {
        if (retainedKeys.has(storageKey)) continue;
        texture.dispose();
        dsaSequenceFrameTextures.delete(storageKey);
    }
    const retainedSequenceIds = new Set(
        snapshot.sequences.map(sequence => sequence.id)
    );
    for (const sequenceId of dsaSequencePreviewUrls.keys()) {
        if (!retainedSequenceIds.has(sequenceId)) {
            dsaSequencePreviewUrls.delete(sequenceId);
        }
    }
}

function selectedDsaFrameTexture(snapshot) {
    if (
        snapshot.selectedSequenceId === null ||
        snapshot.selectedFrameIndex === null
    ) return null;
    return dsaSequenceFrameTextures.get(
        dsaFrameTextureKey(
            snapshot.selectedSequenceId,
            snapshot.selectedFrameIndex
        )
    ) || null;
}

function cineDsaFrameTexture(snapshot) {
    if (
        snapshot.cineSequenceId === null ||
        snapshot.cineFrameIndex === null
    ) return null;
    return dsaSequenceFrameTextures.get(
        dsaFrameTextureKey(
            snapshot.cineSequenceId,
            snapshot.cineFrameIndex
        )
    ) || null;
}

function createDsaPreviewDataUrl(texture, maxWidth = 176, maxHeight = 118) {
    const source = texture?.image;
    if (!source?.data || !source.width || !source.height) return '';
    const scale = Math.min(
        1,
        maxWidth / source.width,
        maxHeight / source.height
    );
    const width = Math.max(1, Math.round(source.width * scale));
    const height = Math.max(1, Math.round(source.height * scale));
    const previewCanvas = document.createElement('canvas');
    previewCanvas.width = width;
    previewCanvas.height = height;
    const context = previewCanvas.getContext('2d', { alpha: false });
    if (!context) return '';
    const image = context.createImageData(width, height);
    for (let y = 0; y < height; y++) {
        const sourceY = source.height - 1 - Math.min(
            source.height - 1,
            Math.floor((y + 0.5) * source.height / height)
        );
        for (let x = 0; x < width; x++) {
            const sourceX = Math.min(
                source.width - 1,
                Math.floor((x + 0.5) * source.width / width)
            );
            const luma = source.data[sourceX + sourceY * source.width];
            const offset = (x + y * width) * 4;
            image.data[offset] = luma;
            image.data[offset + 1] = luma;
            image.data[offset + 2] = luma;
            image.data[offset + 3] = 255;
        }
    }
    context.putImageData(image, 0, 0);
    return previewCanvas.toDataURL('image/jpeg', 0.82);
}

function syncDsaRoadmapState() {
    const snapshot = dsaRoadmapState.getSnapshot();
    pruneDsaFrameTextures(snapshot);
    const cineTexture = cineDsaFrameTexture(snapshot);
    displayMaterial.uniforms.roadmapTexture.value =
        selectedDsaFrameTexture(snapshot) || roadmapTarget.texture;
    displayMaterial.uniforms.cineTexture.value =
        cineTexture || roadmapTarget.texture;
    displayMaterial.uniforms.dsaEnabled.value = snapshot.dsaEnabled;
    displayMaterial.uniforms.dsaMaskValid.value = snapshot.maskValid;
    displayMaterial.uniforms.roadmapEnabled.value = snapshot.roadmapEnabled;
    displayMaterial.uniforms.roadmapValid.value = snapshot.roadmapValid;
    displayMaterial.uniforms.cineEnabled.value = cineTexture !== null;
    ui.updateDsaRoadmapState({
        ...snapshot,
        sequences: snapshot.sequences.map(sequence => ({
            ...sequence,
            previewUrl: dsaSequencePreviewUrls.get(sequence.id)?.url || '',
            previewKey:
                dsaSequencePreviewUrls.get(sequence.id)?.storageKey || ''
        }))
    });
    return snapshot;
}

function startDsaSequenceRecording() {
    const result = dsaRoadmapState.startSequenceRecording({
        revision: ui.getCArmRevision(),
        contrastVisible: contrastSystem?.hasVisibleContrast?.() === true,
        startedAtMs: performance.now()
    });
    syncDsaRoadmapState();
    return result;
}

function finishDsaSequenceRecording() {
    const result = dsaRoadmapState.finishSequenceRecording({
        endedAtMs: performance.now()
    });
    syncDsaRoadmapState();
    return result;
}

syncDsaRoadmapState();
const wireMesh = new THREE.Mesh(new THREE.BufferGeometry(), wireMaterial);
wireMesh.frustumCulled = false;
wireMesh.renderOrder = 7; // draw above translucent debug anatomy
const wireGroup = new THREE.Group();
wireGroup.add(wireMesh);
alignVascularRenderObject(wireGroup);
scene.add(wireGroup);
const wireRenderPoints = Array.from({ length: nodeCount }, () => new THREE.Vector3());
const wireRenderPolyline = [];
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
pigtailCatheter.setStiffnessScales({
    shaftStiffnessScale: catheterShaftStiffnessScale,
    tipStiffnessScale: catheterTipStiffnessScale
});
pigtailCatheter.setExternalCollisionSolver(PHYSICS_MODE === 'xpbd-contact-v1');
if (vesselCollisionTarget !== vessel) {
    pigtailCatheter.setCollisionGeometry(vesselCollisionTarget);
}
alignVascularRenderObject(pigtailCatheter.mesh);
scene.add(pigtailCatheter.mesh);

const guidewireBoundaryFrame = {};
const guidewireBoundaryOptions = {
    twist: 0,
    segment: 0,
    preferredD1: guidewireIntrinsicReferenceAxis,
    compliance: 0,
    out: guidewireBoundaryFrame
};

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
    ...DEFAULT_TOOL_PROFILES.guidewire,
    rodModel: 'kirchhoff'
});
applyActiveGuidewireWallFriction();
xpbdWireBody.syncFromElasticRod(wire);
// createRod precedes the first live wire sync, so align the current material
// frames once without deriving a manufactured rest shape from that pose.
xpbdWireBody.captureKirchhoffRestConfiguration({ captureRestRotation: false });
applyActiveGuidewireKirchhoffProfile();
xpbdCatheterBody = endovascularWorld.createRod('catheter', 320, 4, {
    ...DEFAULT_TOOL_PROFILES.catheter,
    rodModel: 'kirchhoff'
});
pigtailCatheter.syncXpbdBody(xpbdCatheterBody, XPBD_CATHETER_SYNC_OPTIONS);
endovascularWorld.addSheath({
    start: vessel.sheath.start,
    end: vessel.sheath.end,
    innerRadius: INTRODUCER_SHEATH_INNER_RADIUS_MM,
    // The visible proximal catheter is inside the straight loading
    // hub/haemostatic valve. The same open radial lumen lets it slide axially
    // while balancing the preformed tip's constitutive bending moment before
    // that material exits the distal introducer opening.
    proximalExtension: CATHETER_PROXIMAL_LOADING_SUPPORT_LENGTH_MM,
    bodies: [xpbdWireBody, xpbdCatheterBody]
});
xpbdContainment = endovascularWorld.addContainment(xpbdWireBody, xpbdCatheterBody, {
    model: 'kirchhoff',
    innerRadius: PIGTAIL_CATHETER_INNER_RADIUS_MM,
    openProximal: true,
    openDistal: true,
    searchWindow: 2,
    outerStartNode: pigtailCatheter.physicsLumenStartNode,
    // The catheter shaft is the dominant member of the coupled pair. The
    // guidewire is projected into its lumen. The wire cannot laterally pull
    // the much stiffer catheter shaft or turn a proximal push into a distal
    // oscillation.
    innerResponse: 1,
    outerResponse: 0,
    // The distal opening is a unilateral guidewire constraint. A crossing
    // wire is centered into the lumen without turning that projection into a
    // lateral impulse on the catheter tip.
    portalInnerResponse: 1,
    portalOuterResponse: 0,
    portalCompliance: 1e-7,
    portalTransitionLength: 4,
    portalMaxCorrection: 0.15,
    finalProjection: 'inner',
    outerFollowsInnerCenterline: false,
    innerFollowsOuterCenterline: true,
    enforceDistalPortal: true,
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
    getStats: () => endovascularWorld.getStats(),
    getGuidewireType: () => activeGuidewireType,
    getGuidewireStiffnessScale: () => guidewireShaftStiffnessScale,
    getGuidewireStiffnessScales: () => ({
        shaft: guidewireShaftStiffnessScale,
        tip: guidewireTipStiffnessScale
    }),
    getCatheterStiffnessScales: () => ({
        shaft: catheterShaftStiffnessScale,
        tip: catheterTipStiffnessScale
    }),
    getCatheterRelaxationRate: () => catheterRelaxationRate,
    getGuidewireRelaxationRate: () => guidewireRelaxationRate,
    getGuidewireMotionDiagnostics: () => {
        const body = endovascularWorld.getStats().bodies.find(
            candidate => candidate.id === 'guidewire'
        );
        const transport = guidewireSolver.getPerformanceStats();
        return {
            boundaryTransportDeltaMm: transport.transportDeltaMm,
            boundaryTransportSpeedMmPerSecond:
                transport.transportSpeedMmPerSecond,
            maximumMaterialSpeedMmPerSecond: body?.maxSpeed ?? 0,
            materialVelocityLimitMmPerSecond: xpbdWireBody.maxSpeed,
            maximumRawDisplacementSpeedMmPerSecond:
                body?.maximumRawSpeed ?? 0,
            maximumWallProjectionSpeedMmPerSecond:
                body?.maximumWallProjectionSpeed ?? 0,
            maximumRejectedWallProjectionSpeedMmPerSecond:
                body?.maximumRejectedWallProjectionSpeed ?? 0,
            maximumReconstructedSpeedMmPerSecond:
                body?.maximumReconstructedSpeed ?? 0,
            wallProjectionVelocityRetention:
                body?.wallProjectionVelocityRetention ?? 1,
            stiffnessScale: guidewireShaftStiffnessScale,
            shaftStiffnessScale: guidewireShaftStiffnessScale,
            tipStiffnessScale: guidewireTipStiffnessScale,
            relaxationRate: guidewireRelaxationRate,
            relaxationPasses: body?.lastRelaxationPasses ?? 0
        };
    }
};

function updateGuidewireType(type) {
    const nextType = normalizeGuidewireType(type);
    if (nextType === activeGuidewireType) return;
    activeGuidewireType = nextType;
    guidewireRotation = 0;
    applyActiveGuidewireElasticProfile();
    xpbdWireBody.syncFromElasticRod(wire);
    applyActiveGuidewireKirchhoffProfile();
}

function applyActiveGuidewireElasticProfile() {
    applyGuidewireMaterialProfile(wire, {
        segmentLength,
        type: activeGuidewireType,
        shaftStiffnessScale: guidewireShaftStiffnessScale,
        tipStiffnessScale: guidewireTipStiffnessScale
    });
}

function applyActiveGuidewireKirchhoffProfile() {
    applyKirchhoffMaterialProfile(xpbdWireBody, activeGuidewireType, {
        activeStart: 0,
        activeEnd: xpbdWireBody.count - 1,
        materialCoordinates: guidewireMaterialCoordinates,
        tipCoordinate: guidewireMaterialCoordinates[guidewireMaterialCoordinates.length - 1],
        shaftStiffnessScale: guidewireShaftStiffnessScale,
        tipStiffnessScale: guidewireTipStiffnessScale
    });
    applyGuidewireProximalOrientation();
}

function applyActiveGuidewireWallFriction() {
    if (!xpbdWireBody) return;
    xpbdWireBody.wallStaticFriction = guidewireStaticWallFriction;
    xpbdWireBody.wallKineticFriction = guidewireKineticWallFriction;
    // Retain the legacy scalar as a kinetic-friction alias for diagnostics and
    // callers that have not yet migrated to the split Coulomb coefficients.
    xpbdWireBody.wallFriction = guidewireKineticWallFriction;
}

function applyGuidewireProximalOrientation() {
    guidewireBoundaryOptions.twist = guidewireRotation;
    guidewireBoundaryOptions.segment = xpbdWireBody.activeStart;
    applyProximalTwistBoundary(xpbdWireBody, guidewireBoundaryOptions);
}

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
let browserBenchmarkExecutedStepsStart = 0;
let browserBenchmarkIdleExecutedStepsStart = 0;
let browserBenchmarkAcceptedTimeStart = 0;
let browserBenchmarkAccumulatorStart = 0;
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
    automated: false,
    mode: BROWSER_BENCHMARK_MODE_COUPLED
};
const catheterAortaSetup = createCatheterAortaSetupState();
const catheterAortaSetupCommands = createBrowserBenchmarkCommands();
let catheterAortaSetupStatusBucket = -1;
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
let browserBenchmarkPreviousGuidewireCommand = 0;
let browserBenchmarkPreviousGuidewireStepSpeed = 0;
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
    maxGuidewireMaterialSpeedMmPerSecond: 0,
    maxGuidewireMaterialSpeedStep: -1,
    maxGuidewireMaterialSpeedNode: -1,
    maxGuidewireMaterialSpeedX: 0,
    maxGuidewireMaterialSpeedY: 0,
    maxGuidewireMaterialSpeedZ: 0,
    maxGuidewireMaterialSpeedNormalMmPerSecond: 0,
    maxGuidewireMaterialSpeedTangentMmPerSecond: 0,
    maxGuidewireMaterialSpeedAtWall: false,
    maxGuidewireRawDisplacementSpeedMmPerSecond: 0,
    maxGuidewireRawDisplacementSpeedStep: -1,
    maxGuidewireWallProjectionSpeedMmPerSecond: 0,
    maxGuidewireWallProjectionSpeedStep: -1,
    maxGuidewireWallProjectionSpeedNode: -1,
    maxGuidewireRejectedWallProjectionSpeedMmPerSecond: 0,
    maxGuidewireRejectedWallProjectionSpeedStep: -1,
    maxGuidewireReconstructedSpeedMmPerSecond: 0,
    maxGuidewireReconstructedSpeedStep: -1,
    maxGuidewireProjectionLeakSpeedMmPerSecond: 0,
    maxGuidewireSpeedWhileAdvancingMmPerSecond: 0,
    maxGuidewireSpeedWhileRetractingMmPerSecond: 0,
    maxGuidewireSpeedWhileIdleMmPerSecond: 0,
    guidewireReleaseEventCount: 0,
    maxGuidewireSpeedBeforeReleaseMmPerSecond: 0,
    maxGuidewireSpeedAfterReleaseMmPerSecond: 0,
    maxGuidewireReleaseSpeedIncreaseMmPerSecond: 0,
    maxGuidewireReleaseSpeedRatio: 0,
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
    simulationPeakBacklog = simulationAccumulator;
    simulationPeakBacklogScenarioMs = 0;
    simulationPeakBacklogElapsedMs = 0;
    simulationPeakBacklogGuidewireMm = guidewireSolver.progress;
    browserBenchmarkExecutedStepsStart = simulationExecutedSteps;
    browserBenchmarkIdleExecutedStepsStart = simulationIdleExecutedSteps;
    browserBenchmarkAcceptedTimeStart = simulationAcceptedTime;
    browserBenchmarkAccumulatorStart = simulationAccumulator;
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
    browserBenchmarkPhysicsEnvelope.maxGuidewireMaterialSpeedMmPerSecond = 0;
    browserBenchmarkPhysicsEnvelope.maxGuidewireMaterialSpeedStep = -1;
    browserBenchmarkPhysicsEnvelope.maxGuidewireMaterialSpeedNode = -1;
    browserBenchmarkPhysicsEnvelope.maxGuidewireMaterialSpeedX = 0;
    browserBenchmarkPhysicsEnvelope.maxGuidewireMaterialSpeedY = 0;
    browserBenchmarkPhysicsEnvelope.maxGuidewireMaterialSpeedZ = 0;
    browserBenchmarkPhysicsEnvelope.maxGuidewireMaterialSpeedNormalMmPerSecond = 0;
    browserBenchmarkPhysicsEnvelope.maxGuidewireMaterialSpeedTangentMmPerSecond = 0;
    browserBenchmarkPhysicsEnvelope.maxGuidewireMaterialSpeedAtWall = false;
    browserBenchmarkPhysicsEnvelope.maxGuidewireRawDisplacementSpeedMmPerSecond = 0;
    browserBenchmarkPhysicsEnvelope.maxGuidewireRawDisplacementSpeedStep = -1;
    browserBenchmarkPhysicsEnvelope.maxGuidewireWallProjectionSpeedMmPerSecond = 0;
    browserBenchmarkPhysicsEnvelope.maxGuidewireWallProjectionSpeedStep = -1;
    browserBenchmarkPhysicsEnvelope.maxGuidewireWallProjectionSpeedNode = -1;
    browserBenchmarkPhysicsEnvelope.maxGuidewireRejectedWallProjectionSpeedMmPerSecond = 0;
    browserBenchmarkPhysicsEnvelope.maxGuidewireRejectedWallProjectionSpeedStep = -1;
    browserBenchmarkPhysicsEnvelope.maxGuidewireReconstructedSpeedMmPerSecond = 0;
    browserBenchmarkPhysicsEnvelope.maxGuidewireReconstructedSpeedStep = -1;
    browserBenchmarkPhysicsEnvelope.maxGuidewireProjectionLeakSpeedMmPerSecond = 0;
    browserBenchmarkPhysicsEnvelope.maxGuidewireSpeedWhileAdvancingMmPerSecond = 0;
    browserBenchmarkPhysicsEnvelope.maxGuidewireSpeedWhileRetractingMmPerSecond = 0;
    browserBenchmarkPhysicsEnvelope.maxGuidewireSpeedWhileIdleMmPerSecond = 0;
    browserBenchmarkPhysicsEnvelope.guidewireReleaseEventCount = 0;
    browserBenchmarkPhysicsEnvelope.maxGuidewireSpeedBeforeReleaseMmPerSecond = 0;
    browserBenchmarkPhysicsEnvelope.maxGuidewireSpeedAfterReleaseMmPerSecond = 0;
    browserBenchmarkPhysicsEnvelope.maxGuidewireReleaseSpeedIncreaseMmPerSecond = 0;
    browserBenchmarkPhysicsEnvelope.maxGuidewireReleaseSpeedRatio = 0;
    browserBenchmarkPreviousGuidewireCommand = 0;
    browserBenchmarkPreviousGuidewireStepSpeed = 0;
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
    const guidewireBody = xpbdWireBody;
    const guidewireStepSpeed =
        guidewireBody.lastMaximumReconstructedSpeed;
    if (lastGuidewireAdvanceCommand > 1e-6) {
        envelope.maxGuidewireSpeedWhileAdvancingMmPerSecond = Math.max(
            envelope.maxGuidewireSpeedWhileAdvancingMmPerSecond,
            guidewireStepSpeed
        );
    } else if (lastGuidewireAdvanceCommand < -1e-6) {
        envelope.maxGuidewireSpeedWhileRetractingMmPerSecond = Math.max(
            envelope.maxGuidewireSpeedWhileRetractingMmPerSecond,
            guidewireStepSpeed
        );
    } else {
        envelope.maxGuidewireSpeedWhileIdleMmPerSecond = Math.max(
            envelope.maxGuidewireSpeedWhileIdleMmPerSecond,
            guidewireStepSpeed
        );
    }
    if (
        Math.abs(browserBenchmarkPreviousGuidewireCommand) > 1e-6 &&
        Math.abs(lastGuidewireAdvanceCommand) <= 1e-6
    ) {
        envelope.guidewireReleaseEventCount++;
        envelope.maxGuidewireSpeedBeforeReleaseMmPerSecond = Math.max(
            envelope.maxGuidewireSpeedBeforeReleaseMmPerSecond,
            browserBenchmarkPreviousGuidewireStepSpeed
        );
        envelope.maxGuidewireSpeedAfterReleaseMmPerSecond = Math.max(
            envelope.maxGuidewireSpeedAfterReleaseMmPerSecond,
            guidewireStepSpeed
        );
        envelope.maxGuidewireReleaseSpeedIncreaseMmPerSecond = Math.max(
            envelope.maxGuidewireReleaseSpeedIncreaseMmPerSecond,
            guidewireStepSpeed - browserBenchmarkPreviousGuidewireStepSpeed
        );
        envelope.maxGuidewireReleaseSpeedRatio = Math.max(
            envelope.maxGuidewireReleaseSpeedRatio,
            guidewireStepSpeed /
                Math.max(1e-6, browserBenchmarkPreviousGuidewireStepSpeed)
        );
    }
    browserBenchmarkPreviousGuidewireCommand =
        lastGuidewireAdvanceCommand;
    browserBenchmarkPreviousGuidewireStepSpeed = guidewireStepSpeed;
    if (
        guidewireBody.lastMaximumRawSpeed >
        envelope.maxGuidewireRawDisplacementSpeedMmPerSecond
    ) {
        envelope.maxGuidewireRawDisplacementSpeedMmPerSecond =
            guidewireBody.lastMaximumRawSpeed;
        envelope.maxGuidewireRawDisplacementSpeedStep = envelope.steps;
    }
    if (
        guidewireBody.lastMaximumWallProjectionSpeed >
        envelope.maxGuidewireWallProjectionSpeedMmPerSecond
    ) {
        envelope.maxGuidewireWallProjectionSpeedMmPerSecond =
            guidewireBody.lastMaximumWallProjectionSpeed;
        envelope.maxGuidewireWallProjectionSpeedStep = envelope.steps;
        envelope.maxGuidewireWallProjectionSpeedNode =
            guidewireBody.lastMaximumWallProjectionNode;
    }
    if (
        guidewireBody.lastMaximumReconstructedSpeed >
        envelope.maxGuidewireReconstructedSpeedMmPerSecond
    ) {
        envelope.maxGuidewireReconstructedSpeedMmPerSecond =
            guidewireBody.lastMaximumReconstructedSpeed;
        envelope.maxGuidewireReconstructedSpeedStep = envelope.steps;
    }
    if (
        guidewireBody.lastMaximumRejectedWallProjectionSpeed >
        envelope.maxGuidewireRejectedWallProjectionSpeedMmPerSecond
    ) {
        envelope.maxGuidewireRejectedWallProjectionSpeedMmPerSecond =
            guidewireBody.lastMaximumRejectedWallProjectionSpeed;
        envelope.maxGuidewireRejectedWallProjectionSpeedStep =
            envelope.steps;
    }
    envelope.maxGuidewireProjectionLeakSpeedMmPerSecond = Math.max(
        envelope.maxGuidewireProjectionLeakSpeedMmPerSecond,
        guidewireBody.lastMaximumWallProjectionSpeed *
            guidewireBody.wallProjectionVelocityRetention
    );
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

    const bodyCount = browserBenchmarkScenario.mode ===
        BROWSER_BENCHMARK_MODE_GUIDEWIRE
        ? 1
        : browserBenchmarkBodies.length;
    for (let bodyIndex = 0; bodyIndex < bodyCount; bodyIndex++) {
        const body = browserBenchmarkBodies[bodyIndex];
        if (!body) continue;
        const start = body.activeStart;
        const end = Math.min(body.activeEnd, body.segmentCount);
        let maximumMaterialSpeed = 0;
        let maximumMaterialSpeedNode = -1;
        for (let index = start; index <= body.activeEnd; index++) {
            const materialSpeed = Math.hypot(
                body.velocityX[index],
                body.velocityY[index],
                body.velocityZ[index]
            );
            if (materialSpeed > maximumMaterialSpeed) {
                maximumMaterialSpeed = materialSpeed;
                maximumMaterialSpeedNode = index;
            }
            envelope.finite = envelope.finite &&
                Number.isFinite(body.x[index]) &&
                Number.isFinite(body.y[index]) &&
                Number.isFinite(body.z[index]) &&
                Number.isFinite(body.velocityX[index]) &&
                Number.isFinite(body.velocityY[index]) &&
                Number.isFinite(body.velocityZ[index]);
        }
        if (body.id === 'guidewire') {
            if (
                maximumMaterialSpeed >
                envelope.maxGuidewireMaterialSpeedMmPerSecond
            ) {
                envelope.maxGuidewireMaterialSpeedMmPerSecond =
                    maximumMaterialSpeed;
                envelope.maxGuidewireMaterialSpeedStep = envelope.steps;
                envelope.maxGuidewireMaterialSpeedNode =
                    maximumMaterialSpeedNode;
                envelope.maxGuidewireMaterialSpeedX =
                    body.x[maximumMaterialSpeedNode];
                envelope.maxGuidewireMaterialSpeedY =
                    body.y[maximumMaterialSpeedNode];
                envelope.maxGuidewireMaterialSpeedZ =
                    body.z[maximumMaterialSpeedNode];
                let normalX = 0;
                let normalY = 0;
                let normalZ = 0;
                let wallContacts = 0;
                for (const segment of [
                    maximumMaterialSpeedNode - 1,
                    maximumMaterialSpeedNode
                ]) {
                    if (
                        segment < 0 ||
                        segment >= body.segmentCount ||
                        !body.wallActive[segment]
                    ) continue;
                    normalX += body.wallNormalX[segment];
                    normalY += body.wallNormalY[segment];
                    normalZ += body.wallNormalZ[segment];
                    wallContacts++;
                }
                const normalLength = Math.hypot(
                    normalX,
                    normalY,
                    normalZ
                );
                const velocityX = body.velocityX[maximumMaterialSpeedNode];
                const velocityY = body.velocityY[maximumMaterialSpeedNode];
                const velocityZ = body.velocityZ[maximumMaterialSpeedNode];
                const normalSpeed = normalLength > 1e-8
                    ? (
                        velocityX * normalX +
                        velocityY * normalY +
                        velocityZ * normalZ
                    ) / normalLength
                    : 0;
                envelope.maxGuidewireMaterialSpeedNormalMmPerSecond =
                    normalSpeed;
                envelope.maxGuidewireMaterialSpeedTangentMmPerSecond =
                    Math.sqrt(Math.max(
                        0,
                        maximumMaterialSpeed * maximumMaterialSpeed -
                        normalSpeed * normalSpeed
                    ));
                envelope.maxGuidewireMaterialSpeedAtWall = wallContacts > 0;
            }
            if (lastGuidewireAdvanceCommand > 1e-6) {
                envelope.maxGuidewireSpeedWhileAdvancingMmPerSecond = Math.max(
                    envelope.maxGuidewireSpeedWhileAdvancingMmPerSecond,
                    maximumMaterialSpeed
                );
            } else if (lastGuidewireAdvanceCommand < -1e-6) {
                envelope.maxGuidewireSpeedWhileRetractingMmPerSecond = Math.max(
                    envelope.maxGuidewireSpeedWhileRetractingMmPerSecond,
                    maximumMaterialSpeed
                );
            } else {
                envelope.maxGuidewireSpeedWhileIdleMmPerSecond = Math.max(
                    envelope.maxGuidewireSpeedWhileIdleMmPerSecond,
                    maximumMaterialSpeed
                );
            }
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
        cycleIndex: Math.floor(browserBenchmarkScenario.simulationElapsedMs / (
            browserBenchmarkScenario.mode === BROWSER_BENCHMARK_MODE_GUIDEWIRE
                ? GUIDEWIRE_BROWSER_BENCHMARK_CYCLE_MS
                : BROWSER_BENCHMARK_SCENARIO_CYCLE_MS
        )),
        catheterType: browserBenchmarkCatheterType(browserBenchmarkScenario.simulationElapsedMs),
        stopReason: browserBenchmarkScenario.stopReason,
        automated: browserBenchmarkScenario.automated,
        mode: browserBenchmarkScenario.mode
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
    const guidewireTransportSeparationPass =
        scenario.mode !== BROWSER_BENCHMARK_MODE_GUIDEWIRE ||
        browserBenchmarkPhysicsEnvelope
            .maxGuidewireSpeedWhileAdvancingMmPerSecond > 45;
    const guidewireReleaseContinuityPass =
        scenario.mode !== BROWSER_BENCHMARK_MODE_GUIDEWIRE || (
            browserBenchmarkPhysicsEnvelope.guidewireReleaseEventCount >= 2 &&
            browserBenchmarkPhysicsEnvelope
                .maxGuidewireReleaseSpeedIncreaseMmPerSecond <= 5 &&
            browserBenchmarkPhysicsEnvelope.maxGuidewireReleaseSpeedRatio <= 1.25
        );
    const guidewireImpulsePass =
        scenario.mode !== BROWSER_BENCHMARK_MODE_GUIDEWIRE || (
            browserBenchmarkPhysicsEnvelope
                .maxGuidewireProjectionLeakSpeedMmPerSecond <= 1e-6 &&
            browserBenchmarkPhysicsEnvelope
                .maxGuidewireReconstructedSpeedMmPerSecond <= 120
        );
    const benchmarkExecutedSteps = Math.max(
        0,
        simulationExecutedSteps - browserBenchmarkExecutedStepsStart
    );
    const benchmarkIdleExecutedSteps = Math.max(
        0,
        simulationIdleExecutedSteps - browserBenchmarkIdleExecutedStepsStart
    );
    const benchmarkAcceptedSeconds = Math.max(
        0,
        simulationAcceptedTime - browserBenchmarkAcceptedTimeStart
    );
    const accountedSeconds = benchmarkExecutedSteps * fixedDt +
        simulationAccumulator - browserBenchmarkAccumulatorStart;
    const accountingErrorSeconds =
        benchmarkAcceptedSeconds - accountedSeconds;
    const noDroppedStepsPass = Math.abs(accountingErrorSeconds) <= 1e-6;
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
        physicsScheduler: {
            fixedDt,
            maxStepsPerFrame: MAX_PHYSICS_STEPS_PER_FRAME,
            maxIdleSteps: MAX_IDLE_PHYSICS_STEPS,
            acceptedSeconds: benchmarkAcceptedSeconds,
            accountedSeconds,
            accountingErrorSeconds,
            executedSteps: benchmarkExecutedSteps,
            renderFrameExecutedSteps:
                benchmarkExecutedSteps - benchmarkIdleExecutedSteps,
            idleExecutedSteps: benchmarkIdleExecutedSteps,
            startBacklogSeconds: browserBenchmarkAccumulatorStart,
            backlogSeconds: simulationAccumulator,
            backlogSteps: Math.floor(
                (simulationAccumulator + 1e-9) / fixedDt
            ),
            peakBacklogSeconds: simulationPeakBacklog,
            peakBacklogIncreaseSeconds: Math.max(
                0,
                simulationPeakBacklog - browserBenchmarkAccumulatorStart
            ),
            peakBacklogScenarioMs: simulationPeakBacklogScenarioMs,
            peakBacklogElapsedMs: simulationPeakBacklogElapsedMs,
            peakBacklogGuidewireMm: simulationPeakBacklogGuidewireMm,
            droppedSteps: noDroppedStepsPass
                ? 0
                : Math.max(0, accountingErrorSeconds / fixedDt)
        },
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
            noDroppedStepsPass,
            guidewireTransportSeparationPass,
            guidewireReleaseContinuityPass,
            guidewireImpulsePass,
            passed: durationPass && onePercentLowPass && noVisibleGcPausePass &&
                physicsBudgetPass && narrowPhaseAllocationPass && memoryStabilityPass &&
                runtimeAssetPass && penetrationPass && lengthPass && foldPass &&
                finitePass && modePass && contactFieldPass && cameraStablePass &&
                focusPass && noDroppedStepsPass &&
                guidewireTransportSeparationPass &&
                guidewireReleaseContinuityPass && guidewireImpulsePass
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
    guidewireResistanceEstimator.reset();
    lastGuidewireAdvanceCommand = 0;
    pigtailCatheter.reset();
    xpbdWireBody.syncFromElasticRod(wire);
    guidewireRotation = 0;
    applyActiveGuidewireKirchhoffProfile();
    pigtailCatheter.syncXpbdBody(xpbdCatheterBody);
    xpbdContainment.enabled = false;
    xpbdContainment.enforceDistalPortal = true;
    xpbdPortalInnerDriven = true;
    xpbdExternalToolContact.enabled = false;
    endovascularWorld.resetSimulationState();
    if (resetAccumulator) {
        simulationAccumulator = 0;
        simulationPeakBacklog = 0;
    }
}

function getCatheterAortaSetupStatus() {
    return {
        running: catheterAortaSetup.running,
        phase: catheterAortaSetup.phase,
        guidewireProgressCm: guidewireSolver.progress / 10,
        guidewireTargetCm: catheterAortaSetup.guidewireTargetMm / 10,
        finalGuidewireTargetCm:
            (catheterAortaSetup.finalGuidewireTargetMm ??
                catheterAortaSetup.guidewireTargetMm) / 10,
        catheterProgressCm: pigtailCatheter.progress / 10,
        catheterTargetCm: catheterAortaSetup.catheterTargetMm / 10
    };
}

function prepareCatheterAortaScenario({
    reproduceIliacBug = false,
    reproduceRetrogradeGap = false,
    reproduceArchBolus = false
} = {}) {
    stopBrowserBenchmarkScenario('catheter-aorta-setup');
    resetBrowserBenchmarkSimulation();
    startCatheterAortaSetup(
        catheterAortaSetup,
        reproduceArchBolus
            ? {
                guidewireTargetMm:
                    ARCH_BOLUS_GUIDEWIRE_TARGET_MM,
                catheterTargetMm:
                    ARCH_BOLUS_CATHETER_TARGET_MM
            }
            : reproduceIliacBug
            ? {
                catheterTargetMm: ILIAC_BUG_CATHETER_TARGET_MM,
                finalGuidewireTargetMm:
                    ILIAC_BUG_GUIDEWIRE_TARGET_MM
            }
            : reproduceRetrogradeGap
            ? {
                catheterTargetMm:
                    RETROGRADE_GAP_CATHETER_TARGET_MM,
                finalGuidewireTargetMm:
                    RETROGRADE_GAP_GUIDEWIRE_TARGET_MM
            }
            : undefined
    );
    catheterAortaSetupStatusBucket = -1;
    const status = getCatheterAortaSetupStatus();
    ui.updateCatheterAortaSetupStatus?.(status);
    return status;
}

function sampleCatheterAortaScenario() {
    if (!catheterAortaSetup.running) return null;
    const previousPhase = catheterAortaSetup.phase;
    const commands = sampleCatheterAortaSetup(
        catheterAortaSetup,
        {
            guidewireProgressMm: guidewireSolver.progress,
            catheterProgressMm: pigtailCatheter.progress
        },
        catheterAortaSetupCommands
    );
    const activeProgress = catheterAortaSetup.phase === 'catheter'
        ? pigtailCatheter.progress
        : guidewireSolver.progress;
    const statusBucket = Math.floor(activeProgress / 10);
    if (
        catheterAortaSetup.phase !== previousPhase ||
        statusBucket !== catheterAortaSetupStatusBucket ||
        !catheterAortaSetup.running
    ) {
        catheterAortaSetupStatusBucket = statusBucket;
        ui.updateCatheterAortaSetupStatus?.(getCatheterAortaSetupStatus());
    }
    return commands;
}

function startBrowserBenchmarkScenario({
    durationMs = BROWSER_BENCHMARK_DEFAULT_DURATION_MS,
    automated = false,
    skipWarmup = false,
    mode = BROWSER_BENCHMARK_MODE_COUPLED
} = {}) {
    const nextDuration = Number(durationMs);
    if (!Number.isFinite(nextDuration) || nextDuration <= 0) {
        throw new RangeError('Browser benchmark durationMs must be positive');
    }
    if (!endovascularWorld.contactField) {
        throw new Error('Browser benchmark requires the precompiled vessel contact field');
    }
    if (
        mode !== BROWSER_BENCHMARK_MODE_COUPLED &&
        mode !== BROWSER_BENCHMARK_MODE_GUIDEWIRE
    ) {
        throw new RangeError(`Unknown browser benchmark mode: ${mode}`);
    }
    stopCatheterAortaSetup(catheterAortaSetup);
    ui.updateCatheterAortaSetupStatus?.(getCatheterAortaSetupStatus());
    resetBrowserBenchmarkSimulation();
    resetBrowserBenchmark();
    browserBenchmarkScenario.durationMs = nextDuration;
    const startedAt = performance.now();
    browserBenchmarkScenario.warmupStartedAt = startedAt;
    browserBenchmarkScenario.memorySettling = false;
    browserBenchmarkScenario.startedAt = skipWarmup ? startedAt : 0;
    browserBenchmarkScenario.completedAt = 0;
    browserBenchmarkScenario.simulationElapsedMs = 0;
    browserBenchmarkScenario.stopReason = null;
    browserBenchmarkScenario.automated = automated === true;
    browserBenchmarkScenario.mode = mode;
    ui.setAutomatedBenchmarkMode?.(browserBenchmarkScenario.automated);
    browserBenchmarkScenario.running = true;
    browserBenchmarkScenario.warmingUp = !skipWarmup;
    lastBrowserBenchmarkScenarioReport = null;
    return getBrowserBenchmarkScenarioStatus();
}

function sampleBrowserBenchmarkScenario(dt) {
    if (!browserBenchmarkScenario.running) return null;
    const now = performance.now();
    if (browserBenchmarkScenario.warmingUp) {
        const warmupElapsedMs = now - browserBenchmarkScenario.warmupStartedAt;
        if (warmupElapsedMs < BROWSER_BENCHMARK_CHOREOGRAPHY_WARMUP_MS) {
            const commands = sampleActiveBrowserBenchmarkCommands(
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
    const commands = sampleActiveBrowserBenchmarkCommands(
        browserBenchmarkScenario.simulationElapsedMs,
        browserBenchmarkCommands
    );
    browserBenchmarkScenario.simulationElapsedMs += dt * 1000;
    return commands;
}

function sampleActiveBrowserBenchmarkCommands(elapsedMs, out) {
    return browserBenchmarkScenario.mode === BROWSER_BENCHMARK_MODE_GUIDEWIRE
        ? sampleGuidewireBrowserBenchmarkCommands(elapsedMs, out)
        : sampleBrowserBenchmarkCommands(elapsedMs, out);
}

globalThis.__OET_BENCHMARK__ = {
    reset: resetBrowserBenchmark,
    getReport: getBrowserBenchmarkReport,
    startScenario: startBrowserBenchmarkScenario,
    stopScenario: stopBrowserBenchmarkScenario,
    getScenarioStatus: getBrowserBenchmarkScenarioStatus,
    getLastScenarioReport: () => lastBrowserBenchmarkScenarioReport,
    getContrastDebugSnapshot: () => {
        const metrics = contrastSystem?.getMetrics?.() || null;
        const ports = pigtailCatheter.getInjectionPorts([]).map(port => ({
            kind: port.kind,
            position: port.position.toArray(),
            direction: port.direction.toArray(),
            valid: port.valid !== false
        }));
        const network = contrastSystem?.flowNetwork;
        const bifurcation = network?.edges.find(edge =>
            edge.end.y < -275 &&
            edge.end.y > -305 &&
            edge.radiusEnd > 6 &&
            edge.childEdgeIndices.length === 2 &&
            edge.childEdgeIndices.every(
                childIndex => network.edges[childIndex].radiusStart > 6
            )
        );
        const stockConcentrationMgPerMm3 =
            contrastSystem?.medium?.iodineMgPerMl / 1000 || 0.3;
        const iliacs = bifurcation?.childEdgeIndices.map(edgeIndex => {
            const edge = network.edges[edgeIndex];
            return {
                edgeIndex,
                meanFlowMlPerSec: edge.meanFlowMm3PerS / 1000,
                entryStockFraction:
                    edge.massMg[0] /
                    Math.max(1e-9, edge.volumes[0]) /
                    stockConcentrationMgPerMm3,
                totalIodineMassMg: edge.massMg.reduce(
                    (sum, mass) => sum + mass,
                    0
                )
            };
        }) || [];
        return {
            guidewireProgressMm: guidewireSolver.progress,
            catheterProgressMm: pigtailCatheter.progress,
            catheterType: pigtailCatheter.type,
            ports,
            metrics,
            aortoiliacParentEdgeIndex: bifurcation?.index ?? -1,
            iliacs
        };
    }
};

function advanceTailInput(advance, dt) {
    const collisionTarget = PHYSICS_MODE === 'legacy' ? vesselCollisionTarget : null;
    const delta = guidewireSolver.advance(advance, dt, collisionTarget, GUIDE_WIRE_ADVANCE_OPTIONS);
    tailProgress = guidewireSolver.progress;
    lastGuidewireAdvanceCommand = advance;
    return delta;
}

function updateWireMesh() {
    const sourcePoints = PHYSICS_MODE === 'xpbd-contact-v1'
        ? buildContainedGuidewireRenderPolyline({
            guidewireNodes: wire.nodes,
            outerBody: xpbdCatheterBody,
            containment: xpbdContainment,
            out: wireRenderPolyline
        })
        : wire.nodes;
    while (wireRenderPoints.length < sourcePoints.length) {
        wireRenderPoints.push(new THREE.Vector3());
    }
    for (let index = 0; index < sourcePoints.length; index++) {
        const point = sourcePoints[index];
        wireRenderPoints[index].set(point.x, point.y, point.z);
    }
    const previousGeometry = wireMesh.geometry;
    const nextGeometry = updateSmoothTubeGeometry(
        previousGeometry,
        wireRenderPoints,
        {
            radius: GUIDEWIRE_RENDER_RADIUS_MM,
            pointCount: sourcePoints.length,
            samplesPerSegment: GUIDEWIRE_TUBE_SAMPLES_PER_SEGMENT,
            radialSegments: GUIDEWIRE_TUBE_RADIAL_SEGMENTS
        }
    );
    if (nextGeometry !== previousGeometry) {
        wireMesh.geometry = nextGeometry;
        previousGeometry.dispose();
    }
    wireGroup.visible = sourcePoints.length > 1;
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
            constraintPrimaryMs: xpbd.phases.constraintPrimary.lastMs,
            constraintBodyClosureMs:
                xpbd.phases.constraintBodyClosure.lastMs,
            constraintCoupledClosureMs:
                xpbd.phases.constraintCoupledClosure.lastMs,
            constraintMovingClosureMs:
                xpbd.phases.constraintMovingClosure.lastMs,
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
    guidewireResistanceOptions.dt = fixedDt;
    guidewireResistanceOptions.command = lastGuidewireAdvanceCommand;
    guidewireResistanceOptions.atMaximumInsertion =
        tailProgress >= maxInsert - 1e-6;
    const resistance = guidewireResistanceEstimator.update(
        xpbdWireBody,
        guidewireResistanceOptions,
        guidewireResistanceResult
    );
    ui.updateGuidewireResistance(resistance.level, resistance.reason);
}

const fixedDt = PHYSICS_MODE === 'xpbd-contact-v1' ? 1 / 120 : 1 / 60;
const MAX_PHYSICS_STEPS_PER_FRAME = 2;
const MAX_IDLE_PHYSICS_STEPS = 6;
const TARGET_RENDER_FRAME_MS = 1000 / 60;
const PHYSICS_IDLE_GUARD_MS = 0.75;
const PHYSICS_RENDER_RESERVE_MS = 3.5;
// The renderer starts only after the asynchronous anatomy/contact assets are
// ready. Time spent loading those assets is not elapsed simulation time: no
// controls are available and no physical state has started advancing yet.
// Establish the wall-clock origin on the first animation frame so startup
// cannot manufacture minutes of physics backlog that then has to be replayed.
let lastRenderTime = null;
let simulationAccumulator = 0;
let simulationPeakBacklog = 0;
let simulationPeakBacklogScenarioMs = 0;
let simulationPeakBacklogElapsedMs = 0;
let simulationPeakBacklogGuidewireMm = 0;
let simulationExecutedSteps = 0;
let simulationIdleExecutedSteps = 0;
let simulationAcceptedTime = 0;
let simulationStepEstimateMs = 1;
let simulationCatchupPending = false;
let lastFluoroPulseTime = -Infinity;
let autoExposureLevel = 0;
const autoExposureBeamDirection = new THREE.Vector3();
let contactMarkerAccumulator = CONTACT_MARKER_UPDATE_INTERVAL;
let guidewireMeshAccumulator = GUIDEWIRE_MESH_UPDATE_INTERVAL;
let pigtailMeshAccumulator = PIGTAIL_MESH_UPDATE_INTERVAL;
let browserBenchmarkUiAccumulator = Infinity;
let injectionUiAccumulator = Infinity;
let contrastDiagnosticsAccumulator = Infinity;

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
    const automatedCommands =
        sampleCatheterAortaScenario() || sampleBrowserBenchmarkScenario(dt);
    updateGuidewireType(ui.getSelectedGuidewireType());
    const advance = automatedCommands?.guidewireAdvance ?? ui.getAdvance();
    const guidewireRotationCommand = ui.getGuidewireRotation();
    if (guidewireRotationCommand !== 0) {
        guidewireRotation += guidewireRotationCommand *
            GUIDEWIRE_ROTATION_SPEED * dt;
        guidewireRotation = Math.atan2(
            Math.sin(guidewireRotation),
            Math.cos(guidewireRotation)
        );
    }
    const catheterAdvance = automatedCommands?.catheterAdvance ?? ui.getCatheterAdvance();
    const catheterRotation = automatedCommands?.catheterRotation ?? ui.getCatheterRotation();
    const guidewireProgressDelta = advanceTailInput(advance, dt);
    const inserted = Math.max(0, tailProgress);
    pigtailCatheter.setType(automatedCommands?.catheterType ?? ui.getSelectedCatheterType());
    const catheterProgressBefore = pigtailCatheter.progress;
    pigtailCatheter.advance(catheterAdvance, dt, inserted);
    const catheterProgressDelta = pigtailCatheter.progress - catheterProgressBefore;
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
        // Handle rotation is a torsional material-frame boundary. It does not
        // rotate positions or the manufactured kappa_0 field in world space.
        applyGuidewireProximalOrientation();
        let wireWallCollisionStart = Math.max(
            0,
            guidewireSolver.firstLumenNodeIndex() - 1
        );
        xpbdWireBody.setSheathMaterialEndNode(wireWallCollisionStart);
        let wireWallCollisionEnd = xpbdWireBody.segmentCount - 1;
        pigtailCatheter.stepPhysics(dt, XPBD_CATHETER_STEP_OPTIONS);
        const catheterNodeCount = pigtailCatheter.syncXpbdBody(
            xpbdCatheterBody,
            XPBD_CATHETER_SYNC_OPTIONS
        );
        xpbdContainment.outerStartNode = pigtailCatheter.physicsLumenStartNode;
        const firstContainedNode = Math.max(0, Math.ceil((guidewireLength - inserted) / segmentLength));
        const materialEndNode = Math.min(
            xpbdWireBody.count - 1,
            Math.floor((guidewireLength - inserted + pigtailCatheter.progress) / segmentLength)
        );
        // Material overlap identifies the segment crossing the moving distal
        // opening. The solver keeps that crossing under its portal constraint;
        // spatial capture below separately decides when it may be rendered as
        // fully inside the catheter.
        const lastContainedNode = materialEndNode;
        xpbdContainment.enabled =
            pigtailCatheter.progress > 0.5 &&
            catheterNodeCount >= 2 &&
            lastContainedNode >= firstContainedNode;
        xpbdContainment.startNode = firstContainedNode;
        xpbdContainment.endNode = Math.max(firstContainedNode, lastContainedNode);
        xpbdContainment.innerArcOffset =
            firstContainedNode * segmentLength - guidewireLength + inserted;
        xpbdContainment.containedLength = Math.min(pigtailCatheter.progress, inserted);
        // The XPBD catheter body starts at the sheath outlet, while progress
        // is measured from the handle. Give the portal the actual material
        // tip-to-tip distance so it turns off after the catheter overtakes the
        // guidewire instead of constraining an internal wire segment as if it
        // still crossed the distal opening.
        xpbdContainment.portalRetractionDistance = Math.max(
            0,
            pigtailCatheter.progress - inserted
        );
        xpbdContainment.enforceDistalPortal = true;
        if (xpbdContainment.model !== 'kirchhoff') {
            const relativePortalAdvance = guidewireProgressDelta - catheterProgressDelta;
            if (relativePortalAdvance > 1e-5) xpbdPortalInnerDriven = true;
            else if (relativePortalAdvance < -1e-5) xpbdPortalInnerDriven = false;
            // Legacy containment selected a command-dependent one-way owner.
            // Kirchhoff contact instead uses its unilateral gradients and may
            // not change mechanics when the same pose is reached by a
            // different combination of handle commands.
            xpbdContainment.portalInnerResponse = xpbdPortalInnerDriven ? 1 : 0;
            xpbdContainment.portalOuterResponse = xpbdPortalInnerDriven ? 0 : 1;
            xpbdContainment.limitDistalCorrection =
                Math.abs(guidewireProgressDelta) > 1e-5 ||
                Math.abs(catheterProgressDelta) > 1e-5;
            xpbdContainment.preserveStationaryInnerLength =
                Math.abs(catheterProgressDelta) > 1e-5 &&
                Math.abs(advance) <= 1e-5;
            xpbdContainment.reconcileMovingInnerStructure =
                Math.abs(catheterProgressDelta) > 1e-5 &&
                Math.abs(advance) > 1e-5;
            xpbdContainment.outerResponse = xpbdContainment.preserveStationaryInnerLength
                ? 0.2
                : xpbdContainment.reconcileMovingInnerStructure
                    ? 0.04
                    : 0;
        }
        xpbdWireBody.nodeRadius.fill(GUIDEWIRE_RADIUS_MM);
        xpbdWireBody.maxFrameDisplacement =
            xpbdContainment.model !== 'kirchhoff' &&
            xpbdContainment.preserveStationaryInnerLength
                ? 1.5
                : Infinity;
        xpbdWireBody.frameDisplacementStartNode = Math.max(
            xpbdWireBody.activeStart,
            xpbdContainment.endNode
        );

        // The catheter shields the contained guidewire from the vessel wall.
        // Applying vessel contact to both concentric tools gives the wall two
        // independent ways to move the same coupled span and makes containment
        // fight a non-physical force through the catheter. Only the crossing
        // segment and guidewire material distal to the catheter remain exposed.
        if (xpbdContainment.enabled) {
            // Material overlap still owns shielding/contact classification.
            // Spatial capture only decides which nodes are solved as fully
            // contained; treating the transition nodes as externally exposed
            // makes them collide with both catheter and vessel at once.
            // The catheter may only move the collision boundary distally. A
            // short catheter initially ends proximal to the sheath outlet;
            // replacing the sheath boundary with that material node would
            // expose guidewire that is physically still inside the introducer.
            const firstWallExposedSegment = Math.max(
                wireWallCollisionStart,
                materialEndNode
            );
            if (firstWallExposedSegment <= xpbdWireBody.activeEnd - 1) {
                wireWallCollisionStart = firstWallExposedSegment;
            } else {
                wireWallCollisionStart = xpbdWireBody.activeEnd;
                wireWallCollisionEnd = xpbdWireBody.activeEnd - 1;
            }
        }
        // Commit the final mask once. Toggling through the unshielded range and
        // back in one frame woke a stationary guidewire every step.
        xpbdWireBody.setCollisionRange(
            wireWallCollisionStart,
            wireWallCollisionEnd
        );

        const catheterEndSegment = Math.max(0, catheterNodeCount - 2);
        const firstExternalSegment = Math.max(0, Math.min(
            xpbdWireBody.segmentCount - 1,
            materialEndNode + 1
        ));
        // The portal constraint owns the crossing segment. External capsule
        // contact starts one segment farther distally, where the wire is fully
        // outside the catheter, so the opening remains free of a false cap.
        xpbdExternalToolContact.enabled =
            pigtailCatheter.progress > 4 &&
            catheterNodeCount >= 2 &&
            inserted > pigtailCatheter.progress + 0.5 &&
            firstExternalSegment <= xpbdWireBody.activeEnd - 1;
        xpbdExternalToolContact.startSegmentA = firstExternalSegment;
        xpbdExternalToolContact.endSegmentA = Math.min(
            xpbdWireBody.activeEnd - 1,
            firstExternalSegment + 16
        );
        xpbdExternalToolContact.startSegmentB = Math.max(0, catheterEndSegment - 8);
        xpbdExternalToolContact.endSegmentB = catheterEndSegment;

        // Vessel-wall contact already removes forbidden normal motion and
        // applies local Coulomb friction in the world solver. It must not
        // globally freeze tangential sliding or Kirchhoff straightening.
        // Keep the old suppression only in the material span currently being
        // projected by tool-tool constraints; the unsupported distal shaft
        // retains its elastic recovery velocity.
        const guidewireIsToolCoupled = xpbdContainment.enabled ||
            xpbdExternalToolContact.enabled;
        // Relaxation is a constitutive convergence rate, not a release-only
        // effect. Apply the selected value during feed, withdrawal, rotation,
        // catheter coupling and rest so the wire never changes solver mode
        // when the operator releases a control.
        xpbdWireBody.relaxationPasses =
            guidewireRelaxationPasses(guidewireRelaxationRate);
        // The catheter uses the same constitutive convergence control as the
        // guidewire, but keeps an independent rate. Apply it in every solver
        // state so feeding, withdrawal and rest share one physical model.
        xpbdCatheterBody.relaxationPasses =
            guidewireRelaxationPasses(catheterRelaxationRate);
        xpbdWireBody.projectionVelocityRetention = guidewireIsToolCoupled
            ? LEGACY_GUIDEWIRE_TOOL_COUPLED_PROJECTION_RETENTION
            : 1;
        xpbdWireBody.distalProjectionVelocityRetention = 1;
        if (guidewireIsToolCoupled && xpbdContainment.model === 'kirchhoff') {
            // The broad external-contact candidate window extends several
            // centimetres beyond the catheter tip. It must not numerically
            // damp that entire free shaft: only the lumen-contained span is
            // owned by the coupling projection. Real side/rim contact already
            // contributes its own Coulomb friction.
            const firstFreeNode = firstFreeGuidewireNodeAfterContainment({
                activeStart: xpbdWireBody.activeStart,
                activeEnd: xpbdWireBody.activeEnd,
                containmentEndNode: xpbdContainment.endNode
            });
            xpbdWireBody.distalProjectionVelocityRetentionStartNode =
                Math.max(xpbdWireBody.activeStart, firstFreeNode);
        } else {
            xpbdWireBody.distalProjectionVelocityRetentionStartNode = Infinity;
        }
        endovascularWorld.stepFixed();
        const spatialRenderEnd = spatiallyCapturedContainmentEnd({
            innerBody: xpbdWireBody,
            outerBody: xpbdCatheterBody,
            firstContainedNode,
            materialEndNode,
            outerStartNode: xpbdContainment.outerStartNode,
            outerInnerRadius: xpbdContainment.innerRadius,
            closestSegment: xpbdContainment.closestSegment
        });
        xpbdContainment.renderEndNode = Math.min(
            materialEndNode,
            Math.max(
                firstContainedNode,
                materialEndNode - 1,
                spatialRenderEnd
            )
        );
        if (browserBenchmarkScenario.running) recordBrowserPhysicsEnvelope();
        xpbdWireBody.syncToElasticRod(wire);
    } else {
        guidewireSolver.solve(dt, vesselCollisionTarget, {
            iterations: advance === 0 ? 3 : 4
        });
        pigtailCatheter.stepPhysics(dt);
    }
    const catheterActive = catheterAdvance !== 0 || catheterRotation !== 0;
    const guidewireActive = advance !== 0 || guidewireRotationCommand !== 0;
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
    ui.updateInsertedLength(inserted / 10, guidewireRotation);
    ui.updateCatheterLength(
        pigtailCatheter.progress / 10,
        pigtailCatheter.rotation
    );

    if (contrastSystem) {
        contrastSystem.update(dt);
        if (
            Math.abs(contrastSystem.totalDeliveredVolumeMl - displayedContrastDoseMl) >= 0.01
        ) {
            displayedContrastDoseMl = contrastSystem.totalDeliveredVolumeMl;
            ui.updateDose(displayedContrastDoseMl);
        }
    }
    monitor.advance(dt);
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

function renderDisplayCapture(target, {
    dsaEnabled = false,
    dsaMaskValid = false
} = {}) {
    const uniforms = displayMaterial.uniforms;
    const previousDsaEnabled = uniforms.dsaEnabled.value;
    const previousDsaMaskValid = uniforms.dsaMaskValid.value;
    const previousRoadmapEnabled = uniforms.roadmapEnabled.value;
    const previousRoadmapValid = uniforms.roadmapValid.value;
    const previousCineEnabled = uniforms.cineEnabled.value;
    const previousDsaMaskTexture = uniforms.dsaMaskTexture.value;
    const previousRoadmapTexture = uniforms.roadmapTexture.value;
    const previousResolutionX = uniforms.resolution.value.x;
    const previousResolutionY = uniforms.resolution.value.y;

    uniforms.dsaEnabled.value = dsaEnabled;
    uniforms.dsaMaskValid.value = dsaMaskValid;
    uniforms.roadmapEnabled.value = false;
    uniforms.roadmapValid.value = false;
    uniforms.cineEnabled.value = false;
    uniforms.resolution.value.set(target.width, target.height);

    // Never bind a render target as a sampler while writing into it.
    if (target === dsaMaskTarget) {
        uniforms.dsaMaskTexture.value = roadmapTarget.texture;
    }
    if (target === roadmapTarget) {
        uniforms.roadmapTexture.value = dsaMaskTarget.texture;
    }

    renderer.setRenderTarget(target);
    renderer.clear();
    renderer.render(displayScene, postCamera);

    uniforms.dsaEnabled.value = previousDsaEnabled;
    uniforms.dsaMaskValid.value = previousDsaMaskValid;
    uniforms.roadmapEnabled.value = previousRoadmapEnabled;
    uniforms.roadmapValid.value = previousRoadmapValid;
    uniforms.cineEnabled.value = previousCineEnabled;
    uniforms.dsaMaskTexture.value = previousDsaMaskTexture;
    uniforms.roadmapTexture.value = previousRoadmapTexture;
    uniforms.resolution.value.set(previousResolutionX, previousResolutionY);
}

function createArchivedDsaFrame() {
    const width = dsaFrameCaptureTarget.width;
    const height = dsaFrameCaptureTarget.height;
    const requiredLength = width * height * 4;
    if (dsaFrameReadback.length !== requiredLength) {
        dsaFrameReadback = new Uint8Array(requiredLength);
    }
    renderer.readRenderTargetPixels(
        dsaFrameCaptureTarget,
        0,
        0,
        width,
        height,
        dsaFrameReadback
    );
    const redChannel = new Uint8Array(width * height);
    for (let index = 0; index < redChannel.length; index++) {
        redChannel[index] = dsaFrameReadback[index * 4];
    }
    const textureFormat = renderer.capabilities.isWebGL2
        ? THREE.RedFormat
        : THREE.LuminanceFormat;
    const texture = new THREE.DataTexture(
        redChannel,
        width,
        height,
        textureFormat,
        THREE.UnsignedByteType
    );
    texture.minFilter = THREE.LinearFilter;
    texture.magFilter = THREE.LinearFilter;
    texture.wrapS = THREE.ClampToEdgeWrapping;
    texture.wrapT = THREE.ClampToEdgeWrapping;
    texture.generateMipmaps = false;
    texture.flipY = false;
    texture.colorSpace = THREE.NoColorSpace;
    texture.needsUpdate = true;
    return texture;
}

function projectedContrastScore() {
    renderer.setRenderTarget(dsaContrastScoreTarget);
    renderer.setClearColor(0x000000, 0);
    renderer.clear();
    renderer.render(contrastScene, camera);
    renderer.setClearColor(0x000000, 1);
    const width = dsaContrastScoreTarget.width;
    const height = dsaContrastScoreTarget.height;
    const requiredLength = width * height * 4;
    if (dsaContrastScoreReadback.length !== requiredLength) {
        dsaContrastScoreReadback = new Uint8Array(requiredLength);
    }
    renderer.readRenderTargetPixels(
        dsaContrastScoreTarget,
        0,
        0,
        width,
        height,
        dsaContrastScoreReadback
    );
    return scoreProjectedContrastRgba(
        dsaContrastScoreReadback,
        width,
        height,
        {
            collimation:
                displayMaterial.uniforms.collimation.value
        }
    );
}

function captureDsaSequenceFrame() {
    const contrastScore = projectedContrastScore();
    renderDisplayCapture(dsaFrameCaptureTarget, {
        dsaEnabled: true,
        dsaMaskValid: true
    });
    const archivedTexture = createArchivedDsaFrame();
    const appended = dsaRoadmapState.appendRecordingFrame({
        contrastScore,
        capturedAtMs: performance.now()
    });
    if (!appended.ok) {
        archivedTexture.dispose();
        return appended;
    }
    archivedTexture.name = `dsa-sequence-${appended.storageKey}`;
    dsaSequenceFrameTextures.set(appended.storageKey, archivedTexture);
    if (appended.isBest) {
        dsaSequencePreviewUrls.set(appended.sequenceId, {
            storageKey: appended.storageKey,
            url: createDsaPreviewDataUrl(archivedTexture)
        });
    }
    if (appended.shouldStop) {
        dsaRoadmapState.finishSequenceRecording({
            endedAtMs: performance.now()
        });
    }
    return appended;
}

function processDsaRoadmapCapture() {
    const revision = ui.getCArmRevision();
    const nowMs = performance.now();
    let snapshot = dsaRoadmapState.getSnapshot();
    const contrastVisible = contrastSystem?.hasVisibleContrast?.() === true;
    let maskCapturedThisPulse = false;

    if (
        snapshot.maskCapturePending &&
        dsaRoadmapState.isMaskCaptureReady({ nowMs })
    ) {
        renderDisplayCapture(dsaMaskTarget, {
            dsaEnabled: false,
            dsaMaskValid: false
        });
        dsaRoadmapState.markMaskCaptured(revision, { nowMs });
        maskCapturedThisPulse = true;
        snapshot = syncDsaRoadmapState();
    }

    if (snapshot.roadmapCapturePending) {
        if (!contrastVisible) {
            dsaRoadmapState.failRoadmapCapture(
                'Contrast is no longer visible · inject before capturing roadmap'
            );
        } else {
            renderDisplayCapture(roadmapTarget, {
                dsaEnabled: true,
                dsaMaskValid: true
            });
            dsaRoadmapState.markRoadmapCaptured(revision);
        }
        snapshot = syncDsaRoadmapState();
    }

    if (
        snapshot.recording &&
        snapshot.maskValid &&
        !maskCapturedThisPulse
    ) {
        captureDsaSequenceFrame();
        syncDsaRoadmapState();
    }
}

function executeAccumulatedPhysicsStep(idle = false) {
    const startedAt = performance.now();
    stepSimulation(fixedDt);
    simulationAccumulator -= fixedDt;
    simulationExecutedSteps++;
    if (idle) simulationIdleExecutedSteps++;
    const duration = performance.now() - startedAt;
    // A conservative decaying estimate keeps catch-up work inside the time
    // actually offered by the browser. It affects scheduling only; every
    // physical step still uses the same fixed dt and solver sequence.
    simulationStepEstimateMs = Math.max(
        duration,
        simulationStepEstimateMs * 0.8
    );
}

function runIdlePhysicsCatchup(deadline = null) {
    simulationCatchupPending = false;
    if (
        document.visibilityState !== 'visible' ||
        simulationAccumulator + 1e-9 < fixedDt
    ) return;
    const fallbackDeadline = lastRenderTime === null
        ? performance.now()
        : lastRenderTime + TARGET_RENDER_FRAME_MS;
    let steps = 0;
    while (
        simulationAccumulator + 1e-9 >= fixedDt &&
        steps < MAX_IDLE_PHYSICS_STEPS
    ) {
        const remainingMs = deadline
            ? deadline.timeRemaining()
            : fallbackDeadline - performance.now();
        if (
            remainingMs <=
                simulationStepEstimateMs + PHYSICS_IDLE_GUARD_MS
        ) break;
        executeAccumulatedPhysicsStep(true);
        steps++;
    }
}

function scheduleIdlePhysicsCatchup() {
    if (
        simulationCatchupPending ||
        simulationAccumulator + 1e-9 < fixedDt ||
        document.visibilityState !== 'visible'
    ) return;
    simulationCatchupPending = true;
    // requestIdleCallback is intentionally not used here. Chromium often
    // withholds it while a continuously animated WebGL page is visible, so a
    // single missed vsync could leave four 120 Hz steps pending while the next
    // rAF was allowed to execute only two. A zero-delay task is guaranteed to
    // run after the current render task; runIdlePhysicsCatchup still observes
    // the same 60 Hz deadline and refuses work that does not fit.
    window.setTimeout(() => runIdlePhysicsCatchup(null), 0);
}

function animate(time) {
    // Render loop: updates geometry, handles fluoroscopy accumulation, and UI
    const frameCpuStartedAt = performance.now();
    const frameMs = lastRenderTime === null ? 0 : time - lastRenderTime;
    const dt = Math.max(0, frameMs / 1000);
    lastRenderTime = time;
    recordBrowserFrame(frameMs);
    simulationAcceptedTime += dt;
    simulationAccumulator += dt;
    if (simulationAccumulator > simulationPeakBacklog) {
        simulationPeakBacklog = simulationAccumulator;
        if (
            browserBenchmarkScenario.running &&
            !browserBenchmarkScenario.warmingUp
        ) {
            simulationPeakBacklogScenarioMs =
                browserBenchmarkScenario.simulationElapsedMs;
            simulationPeakBacklogElapsedMs =
                performance.now() - browserBenchmarkScenario.startedAt;
            simulationPeakBacklogGuidewireMm = guidewireSolver.progress;
        }
    }
    let simulationSteps = 0;
    while (
        simulationAccumulator + 1e-9 >= fixedDt &&
        simulationSteps < MAX_PHYSICS_STEPS_PER_FRAME
    ) {
        if (
            simulationSteps > 0 &&
            performance.now() - frameCpuStartedAt +
                simulationStepEstimateMs + PHYSICS_RENDER_RESERVE_MS >=
                TARGET_RENDER_FRAME_MS
        ) break;
        executeAccumulatedPhysicsStep(false);
        simulationSteps++;
    }
    scheduleIdlePhysicsCatchup();
    const frameSimulationEndedAt = performance.now();

    // Physiology advances in the fixed simulation step above, preserving its
    // numerical behavior. Only the screen sweep follows presented wall-clock
    // frames, so physics catch-up cannot make the trace pause and jump.
    monitor.render(dt);

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
    const contrastShouldRender = !!contrastSystem && (
        contrastSystem.isInjecting || contrastSystem.hasVisibleContrast()
    );
    if (contrastShouldRender) {
        contrastRenderAccumulator += dt;
        const contrastRenderInterval = contrastSystem.isInjecting ? 1 / 30 : 1 / 24;
        if (contrastRenderAccumulator >= contrastRenderInterval) {
            contrastRenderAccumulator = 0;
            contrastVolumeRenderer?.setDebugMode(!fluoroscopy);
            contrastVolumeRenderer?.update();
        }
    } else if (contrastVolumeRenderer) {
        contrastVolumeRenderer.group.visible = false;
    }

    if (fluoroscopy && voxelGroup.parent !== contrastScene) {
        scene.remove(voxelGroup);
        contrastScene.add(voxelGroup);
    } else if (!fluoroscopy && voxelGroup.parent !== scene) {
        contrastScene.remove(voxelGroup);
        scene.add(voxelGroup);
    }
    vesselGroup.visible = !fluoroscopy;
    sheathFluoroMesh.visible = fluoroscopy;
    if (wallContactMarkers) wallContactMarkers.visible = !fluoroscopy;
    if (wallBreachMarkers) wallBreachMarkers.visible = !fluoroscopy;
    if (wallWorstPointMarker) wallWorstPointMarker.visible = !fluoroscopy && !!wallWorstPointMarker.userData.hasPoint;
    skeletonModel.visible = fluoroscopy;
    injectionUiAccumulator += dt;
    if (injectionUiAccumulator >= 0.1) {
        injectionUiAccumulator = 0;
        const injectionSourceStatus = contrastSystem
            ? contrastSystem.getSourceStatus(ui.getInjectionSource())
            : { valid: false, label: '', reason: 'Loading flow model' };
        ui.setInjectButtonDisabled(!injectionSourceStatus.valid || !!contrastSystem?.isInjecting);
        ui.setStopInjectionDisabled(!contrastSystem?.isInjecting);
        ui.setInjectionSourceStatus(
            injectionSourceStatus.valid,
            contrastSystem?.isInjecting
                ? `Injecting via ${injectionSourceStatus.label}`
                : injectionSourceStatus.reason || `${injectionSourceStatus.label} ready`
        );
        const injectionRequest = ui.getInjectionRequest();
        const injectionPreview = contrastSystem
            ? contrastSystem.isInjecting
                ? {
                    valid: true,
                    source: contrastSystem.injection.source,
                    ...contrastSystem.injection.hydraulics
                }
                : contrastSystem.getInjectionPreview(
                    injectionRequest,
                    injectionSourceStatus
                )
            : {
                valid: false,
                reason: 'Flow model is still loading'
            };
        ui.updateInjectionHydraulics(injectionPreview);
    }
    contrastDiagnosticsAccumulator += dt;
    if (contrastDiagnosticsAccumulator >= 0.25) {
        contrastDiagnosticsAccumulator = 0;
        ui.updateContrastDiagnostics(contrastSystem?.getMetrics?.() || null);
    }
    if (
        dsaRoadmapState.invalidateForGeometryRevision(
            ui.getCArmRevision()
        )
    ) {
        syncDsaRoadmapState();
    }
    if (dsaRoadmapState.cinePlaying) {
        const cineAdvance = dsaRoadmapState.advanceCine({ nowMs: time });
        if (cineAdvance.changed) syncDsaRoadmapState();
    }
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

        renderer.setRenderTarget(catheterMarkerTarget);
        renderer.setClearColor(0x000000, 0);
        renderer.clear();
        const catheterShaftVisible = pigtailCatheter.shaftMesh.visible;
        pigtailCatheter.shaftMesh.visible = false;
        renderOnlySceneObject(scene, camera, pigtailCatheter.mesh);
        pigtailCatheter.shaftMesh.visible = catheterShaftVisible;
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
        displayMaterial.uniforms.catheterMarkerTexture.value = catheterMarkerTarget.texture;
        displayMaterial.uniforms.sheathTexture.value = sheathTarget.texture;
        displayMaterial.uniforms.boneTexture.value = boneTarget.texture;
        displayMaterial.uniforms.time.value = time * 0.001;
        processDsaRoadmapCapture();
        renderer.setRenderTarget(null);
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
        anatomyLabelRenderer.render(scene, camera);
        completeFirstLoadedFrame();
    }

    ui.updatePerfStats(dt);
    recordBrowserFrameCpu(frameCpuStartedAt, frameSimulationEndedAt, frameUpdateEndedAt);

    requestAnimationFrame(animate);
}
requestAnimationFrame(animate);

document.addEventListener('visibilitychange', () => {
    if (document.visibilityState === 'visible') {
        // Time spent while requestAnimationFrame is suspended is a deliberate
        // application pause, not elapsed simulation time. Preserve any real
        // pre-existing backlog and restart only the wall-clock reference.
        lastRenderTime = performance.now();
    }
});

window.addEventListener('resize', () => {
    // Keep all targets and shader uniforms in sync with the canvas size
    const w = window.innerWidth;
    const h = window.innerHeight;
    const targetWidth = Math.max(1, Math.round(w * FLUORO_TARGET_SCALE));
    const targetHeight = Math.max(1, Math.round(h * FLUORO_TARGET_SCALE));
    renderer.setSize(w, h);
    anatomyLabelRenderer.setSize(w, h);
    camera.aspect = w / h;
    camera.updateProjectionMatrix();
    offscreenTarget.setSize(targetWidth, targetHeight);
    contrastTarget.setSize(targetWidth, targetHeight);
    metalTarget.setSize(targetWidth, targetHeight);
    catheterTarget.setSize(targetWidth, targetHeight);
    catheterMarkerTarget.setSize(targetWidth, targetHeight);
    sheathTarget.setSize(targetWidth, targetHeight);
    boneTarget.setSize(targetWidth, targetHeight);
    accumulateTarget1.setSize(targetWidth, targetHeight);
    accumulateTarget2.setSize(targetWidth, targetHeight);
    frontDepthTarget.setSize(targetWidth, targetHeight);
    backDepthTarget.setSize(targetWidth, targetHeight);
    thicknessTarget.setSize(targetWidth, targetHeight);
    dsaMaskTarget.setSize(targetWidth, targetHeight);
    roadmapTarget.setSize(targetWidth, targetHeight);
    const archiveDimensions = dsaArchiveDimensions(
        targetWidth,
        targetHeight
    );
    dsaFrameCaptureTarget.setSize(
        archiveDimensions.width,
        archiveDimensions.height
    );
    dsaFrameReadback = new Uint8Array(
        archiveDimensions.width * archiveDimensions.height * 4
    );
    const scoreDimensions = dsaScoreDimensions(
        targetWidth,
        targetHeight,
        DSA_SCORE_MAX_DIMENSION
    );
    const scoreWidth = scoreDimensions.width;
    const scoreHeight = scoreDimensions.height;
    dsaContrastScoreTarget.setSize(scoreWidth, scoreHeight);
    dsaContrastScoreReadback = new Uint8Array(
        scoreWidth * scoreHeight * 4
    );
    dsaRoadmapState.invalidate(
        'Detector resized · saved roadmap retained · hold R for a new DSA view'
    );
    syncDsaRoadmapState();
    anatomyProjectionValid = false;
    displayMaterial.uniforms.resolution.value.set(targetWidth, targetHeight);
});
