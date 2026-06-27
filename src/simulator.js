// Main simulator entry: sets up scenes, physics, rendering passes, and UI.
import * as THREE from 'three';
import { ElasticRod } from './physics/elasticRod.js?v=20260615rigidguidewire1';
import { GuidewireSolver } from './physics/guidewireSolver.js?v=20260618withdrawrelax4';
import { generateVessel } from './vesselGeometry.js?v=20260614guidewirestable1';
import { initUI } from './ui/ui.js?v=20260620rollpreview1';
import { createBoneModel } from './boneModel.js?v=20260618loading1';
import { FlowContrastAgent, updateFlowContrastMesh } from './contrastFlowAgent.js?v=20260614guidewirestable1';
import { PigtailCatheter } from './pigtailCatheter.js?v=20260614guidewirestable1';
import { createAortaModel } from './aortaModel.js?v=20260616aortacollider2';
import { GUIDEWIRE_RADIUS_MM, GUIDEWIRE_RENDER_RADIUS_MM } from './toolDimensions.js';
import { vertexShader as blendVS, fragmentShader as blendFS } from './shaders/blendShader.js';
import { vertexShader as thicknessVS, fragmentShader as thicknessFS } from './shaders/thicknessShader.js';
import { vertexShader as displayVS, fragmentShader as displayFS } from './shaders/displayShader.js?v=20260623imagingdefaults1';

const LUMEN_DEBUG_COLOR = 0x29ffd4;
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
const GUIDEWIRE_SEGMENT_RADIAL_SEGMENTS = 32;
const GUIDEWIRE_SEGMENT_OVERLAP_MM = GUIDEWIRE_RENDER_RADIUS_MM * 1.35;
const PIGTAIL_MESH_UPDATE_INTERVAL = 1 / 30;
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
const DEVICE_MASK_TARGET_SAMPLES = renderer.capabilities.isWebGL2 ? 4 : 0;
const deviceMaskTargetOptions = { samples: DEVICE_MASK_TARGET_SAMPLES };

// Primary 3D scene (wire, vessels, bones)
const scene = new THREE.Scene();
scene.background = new THREE.Color(0x000000);

// Separate scene for rendering contrast meshes in fluoroscopy mode
const contrastScene = new THREE.Scene();

// Offscreen render targets used by various post-processing passes
const offscreenTarget = new THREE.WebGLRenderTarget(window.innerWidth, window.innerHeight, deviceMaskTargetOptions);
const contrastTarget = new THREE.WebGLRenderTarget(window.innerWidth, window.innerHeight);
const metalTarget = new THREE.WebGLRenderTarget(window.innerWidth, window.innerHeight, deviceMaskTargetOptions);
const catheterTarget = new THREE.WebGLRenderTarget(window.innerWidth, window.innerHeight);
const sheathTarget = new THREE.WebGLRenderTarget(window.innerWidth, window.innerHeight);
const boneTarget = new THREE.WebGLRenderTarget(window.innerWidth, window.innerHeight, {
    type: THREE.HalfFloatType
});
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
        resolution: { value: new THREE.Vector2(window.innerWidth, window.innerHeight) },
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

let vesselGroup;
const { group: skeletonModel, material: boneMaterial } = createBoneModel({
    onLoaded: () => completeLoadingMilestone(
        'skeleton',
        loadingMilestones.has('aorta') ? 'Loading vessel model' : 'Rendering first frame'
    ),
    onError: () => failLoadingMilestone('skeleton')
});

// Lightweight centerline metadata; the visible vessel and collision surface are
// loaded from the STL aorta model.
const { vessel } = generateVessel(140, 0);
vesselGroup = new THREE.Group();
let vesselCollisionTarget = vessel;
let pigtailCatheter = null;
let guidewireSolver = null;

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

function createExactLumenDebugMesh(geometry) {
    const material = new THREE.MeshBasicMaterial({
        color: LUMEN_DEBUG_COLOR,
        side: THREE.DoubleSide,
        transparent: true,
        opacity: 0.24,
        depthWrite: false,
        depthTest: true
    });
    const mesh = new THREE.Mesh(geometry, material);
    mesh.renderOrder = 3;
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
scene.add(sheathFluoroMesh);
const lumenDebugGroup = new THREE.Group();
lumenDebugGroup.visible = false;
vesselGroup.add(lumenDebugGroup);
setLoadingMessage('Loading anatomy models');
createAortaModel(vessel, {
    onLoaded: ({ collision }) => {
        vesselCollisionTarget = {
            ...collision,
            segments: [vessel.sheath]
        };
        lumenDebugGroup.clear();
        lumenDebugGroup.add(createExactLumenDebugMesh(collision.geometry));
        lumenDebugGroup.add(createStlPreprocessDebug(collision.preprocessing));
        lumenDebugGroup.add(createSheathEntryDebugMarker(collision, vessel.sheath));
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
const voxelGroup = new THREE.Group();
scene.add(voxelGroup);
let contrastMesh = null;
let contrastMeshCount = 0;
let contrastRenderAccumulator = 0;

// Guidewire physical model (discrete elastic rod)
const segmentLength = 5;
const nodeCount = 201;
const guidewireLength = segmentLength * (nodeCount - 1);
const GUIDEWIRE_ADVANCE_RATE = 44;
const GUIDEWIRE_BODY_BENDING_STIFFNESS = 32;
const GUIDEWIRE_TIP_BENDING_STIFFNESS = 8;
const GUIDEWIRE_TIP_FLEX_LENGTH = 105;
const GUIDEWIRE_TIP_SOFT_LENGTH = 24;

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
        group.add(boundaryLines);
    }

    if (preprocessing.lumenContourDebugSegments?.length) {
        const contourGeometry = new THREE.BufferGeometry();
        contourGeometry.setAttribute(
            'position',
            new THREE.BufferAttribute(preprocessing.lumenContourDebugSegments, 3)
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

function smoothRange(start, end, value) {
    const t = Math.max(0, Math.min(1, (value - start) / Math.max(1e-6, end - start)));
    return t * t * (3 - 2 * t);
}

function applyGuidewireStiffnessProfile() {
    const lastIndex = wire.nodes.length - 1;
    for (let i = 0; i < wire.nodes.length; i++) {
        const distanceFromTip = (lastIndex - i) * segmentLength;
        const bodyBlend = smoothRange(GUIDEWIRE_TIP_SOFT_LENGTH, GUIDEWIRE_TIP_FLEX_LENGTH, distanceFromTip);
        wire.nodes[i].bendingStiffness =
            GUIDEWIRE_TIP_BENDING_STIFFNESS * (1 - bodyBlend) +
            GUIDEWIRE_BODY_BENDING_STIFFNESS * bodyBlend;
    }
}

// The proximal guidewire and the part inside the introducer sheath are
// constrained by the sheath lumen. Once a node exits the sheath tip it becomes
// free and is governed by rod stiffness and vessel-wall collision.
applyGuidewireStiffnessProfile();
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
        skeletonModel.visible = fluoroscopy;
        displayMaterial.uniforms.fluoroscopy.value = fluoroscopy;
    },
});
const { monitor } = ui;
const wireSegmentGeometry = new THREE.CylinderGeometry(
    GUIDEWIRE_RENDER_RADIUS_MM,
    GUIDEWIRE_RENDER_RADIUS_MM,
    1,
    GUIDEWIRE_SEGMENT_RADIAL_SEGMENTS,
    1,
    false
);
const wireMesh = new THREE.InstancedMesh(wireSegmentGeometry, wireMaterial, nodeCount - 1);
wireMesh.instanceMatrix.setUsage(THREE.DynamicDrawUsage);
wireMesh.frustumCulled = false;
wireMesh.renderOrder = 7; // draw above translucent debug anatomy
wireMesh.count = 0;
const wireGroup = new THREE.Group();
wireGroup.add(wireMesh);
scene.add(wireGroup);
const wireSegmentMatrix = new THREE.Matrix4();
const wireSegmentQuaternion = new THREE.Quaternion();
const wireSegmentAxis = new THREE.Vector3();
const wireSegmentMidpoint = new THREE.Vector3();
const wireSegmentScale = new THREE.Vector3(1, 1, 1);
const wireSegmentUp = new THREE.Vector3(0, 1, 0);

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
scene.add(wallWorstPointMarker);

pigtailCatheter = new PigtailCatheter({
    wire,
    segmentLength,
    guidewireLength,
    tailProgressRef: () => guidewireSolver.progress,
    vessel
});
if (vesselCollisionTarget !== vessel) {
    pigtailCatheter.setCollisionGeometry(vesselCollisionTarget);
}
scene.add(pigtailCatheter.mesh);

function advanceTailInput(advance, dt) {
    const delta = guidewireSolver.advance(advance, dt, vesselCollisionTarget);
    tailProgress = guidewireSolver.progress;
    return delta;
}

function updateWireMesh() {
    let segmentIndex = 0;
    // Keep the full physical guidewire visible, including the external tail
    // before it enters the introducer sheath.
    for (let i = 0; i < wire.nodes.length - 1; i++) {
        const a = wire.nodes[i];
        const b = wire.nodes[i + 1];
        wireSegmentAxis.set(b.x - a.x, b.y - a.y, b.z - a.z);
        const length = wireSegmentAxis.length();
        if (length < 1e-6) continue;

        wireSegmentAxis.multiplyScalar(1 / length);
        wireSegmentMidpoint.set(
            (a.x + b.x) * 0.5,
            (a.y + b.y) * 0.5,
            (a.z + b.z) * 0.5
        );
        wireSegmentQuaternion.setFromUnitVectors(wireSegmentUp, wireSegmentAxis);
        wireSegmentScale.set(1, length + GUIDEWIRE_SEGMENT_OVERLAP_MM, 1);
        wireSegmentMatrix.compose(wireSegmentMidpoint, wireSegmentQuaternion, wireSegmentScale);
        wireMesh.setMatrixAt(segmentIndex, wireSegmentMatrix);
        segmentIndex++;
    }

    wireMesh.count = segmentIndex;
    wireMesh.instanceMatrix.needsUpdate = true;
    wireGroup.visible = segmentIndex > 0;
}

function sampleGuidewireContactMarkers() {
    const markerMatrix = new THREE.Matrix4();
    if (fluoroscopy) {
        ui.updateGuidewireDiagnostics(null);
        wallContactMarkers.count = 0;
        wallBreachMarkers.count = 0;
        wallWorstPointMarker.userData.hasPoint = false;
        wallWorstPointMarker.visible = false;
        return;
    }

    const lumenDiagnostics = guidewireSolver.collectLumenDiagnostics(vesselCollisionTarget, {
        clearance: guidewireSolver.meshClearance,
        contactBand: GUIDEWIRE_DIAGNOSTIC_CONTACT_BAND,
        collectMarkers: true,
        markerLimit: CONTACT_MARKER_LIMIT
    });
    lumenDiagnostics.performance = guidewireSolver.getPerformanceStats();
    ui.updateGuidewireDiagnostics(lumenDiagnostics);
    if (lumenDiagnostics.worstPoint) {
        wallWorstPointMarker.position.set(
            lumenDiagnostics.worstPoint.x,
            lumenDiagnostics.worstPoint.y,
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
            markerMatrix.makeTranslation(p.x, p.y, p.z);
            mesh.setMatrixAt(i, markerMatrix);
        }
        mesh.instanceMatrix.needsUpdate = true;
    };

    applySamples(wallContactMarkers, lumenDiagnostics.contacts || []);
    applySamples(wallBreachMarkers, lumenDiagnostics.breaches || []);
}

function updateGuidewireResistance() {
    ui.updateGuidewireResistance(0, '');
}

const fixedDt = 1 / 60;
let lastRenderTime = performance.now();
let lastFluoroPulseTime = -Infinity;
let autoExposureLevel = 0;
const autoExposureBeamDirection = new THREE.Vector3();
let contactMarkerAccumulator = CONTACT_MARKER_UPDATE_INTERVAL;
let pigtailMeshAccumulator = PIGTAIL_MESH_UPDATE_INTERVAL;

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

function stepSimulation() {
    // Advance input, integrate rod physics, collisions, and update medical monitors
    const advance = ui.getAdvance();
    advanceTailInput(advance, fixedDt);
    guidewireSolver.solve(fixedDt, vesselCollisionTarget, {
        iterations: advance === 0 ? 3 : 4
    });
    const inserted = Math.max(0, tailProgress);
    pigtailCatheter.setType(ui.getSelectedCatheterType());
    pigtailCatheter.advance(ui.getCatheterAdvance(), fixedDt, inserted);
    pigtailCatheter.rotate(ui.getCatheterRotation(), fixedDt);
    pigtailCatheter.stepPhysics(fixedDt);
    const catheterActive = ui.getCatheterAdvance() !== 0 || ui.getCatheterRotation() !== 0;
    const guidewireActive = advance !== 0;
    const guidewireInsideCatheter = pigtailCatheter.progress > 4 && inserted > 0;
    pigtailCatheter.constrainGuidewire(fixedDt, {
        reactionScale: guidewireActive && !catheterActive ? 0.08 : 1
    });
    if (guidewireActive && !catheterActive && guidewireInsideCatheter) {
        guidewireSolver.solve(fixedDt, vesselCollisionTarget, { iterations: 8, forceRelax: true });
        pigtailCatheter.constrainGuidewire(fixedDt, { reactionScale: 0.04 });
        guidewireSolver.solve(fixedDt, vesselCollisionTarget, { iterations: 5, forceRelax: true });
    }
    if (catheterActive) {
        guidewireSolver.solve(fixedDt, vesselCollisionTarget, { iterations: 10, forceRelax: true });
        pigtailCatheter.constrainGuidewire(fixedDt);
        guidewireSolver.solve(fixedDt, vesselCollisionTarget, { iterations: 8, forceRelax: true });
    }
    updateGuidewireResistance();
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

function renderOnlySceneObjects(scene, camera, objects) {
    const keep = new Set(objects.filter(Boolean));
    const hidden = [];
    for (const child of scene.children) {
        if (child.isCamera) continue;
        const shouldRender = keep.has(child) && child.visible;
        if (!shouldRender && child.visible) {
            hidden.push(child);
            child.visible = false;
        }
    }
    renderer.render(scene, camera);
    for (const obj of hidden) obj.visible = true;
}

function animate(time) {
    // Render loop: updates geometry, handles fluoroscopy accumulation, and UI
    const dt = (time - lastRenderTime) / 1000;
    lastRenderTime = time;

    updateWireMesh();
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

        renderer.setRenderTarget(boneTarget);
        renderer.clear();
        scene.overrideMaterial = boneProjectionMaterial;
        renderOnlySceneObjects(scene, camera, [skeletonModel]);
        scene.overrideMaterial = null;
        renderer.setRenderTarget(null);

        renderer.setRenderTarget(contrastTarget);
        withTransparentClear(renderer, () => {
            renderer.clear();
            renderer.render(contrastScene, camera);
        });

        renderer.setRenderTarget(metalTarget);
        withTransparentClear(renderer, () => {
            renderer.clear();
            scene.overrideMaterial = wireProjectionMaterial;
            renderOnlySceneObjects(scene, camera, [wireGroup]);
            scene.overrideMaterial = null;
        });

        renderer.setRenderTarget(catheterTarget);
        withTransparentClear(renderer, () => {
            renderer.clear();
            renderOnlySceneObjects(scene, camera, [pigtailCatheter.mesh]);
        });

        renderer.setRenderTarget(sheathTarget);
        withTransparentClear(renderer, () => {
            renderer.clear();
            renderOnlySceneObjects(scene, camera, [sheathFluoroMesh]);
        });

        renderer.setRenderTarget(offscreenTarget);
        renderer.clear();
        renderOnlySceneObjects(scene, camera, [sheathFluoroMesh]);
        const previousOverlayAutoClear = renderer.autoClear;
        renderer.autoClear = false;
        scene.overrideMaterial = wireProjectionMaterial;
        renderOnlySceneObjects(scene, camera, [wireGroup]);
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
    metalTarget.setSize(w, h);
    catheterTarget.setSize(w, h);
    sheathTarget.setSize(w, h);
    boneTarget.setSize(w, h);
    accumulateTarget1.setSize(w, h);
    accumulateTarget2.setSize(w, h);
    frontDepthTarget.setSize(w, h);
    backDepthTarget.setSize(w, h);
    thicknessTarget.setSize(w, h);
    displayMaterial.uniforms.resolution.value.set(w, h);
});
