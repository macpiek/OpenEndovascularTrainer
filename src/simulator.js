// Main simulator entry: sets up scenes, physics, rendering passes, and UI.
import * as THREE from 'three';
import { ElasticRod } from './physics/elasticRod.js?v=20260614physrevert1';
import { generateVessel } from './vesselGeometry.js?v=20260614guidewirestable1';
import { initUI } from './ui/ui.js?v=20260615fluoro8';
import { createBoneModel } from './boneModel.js';
import { FlowContrastAgent, updateFlowContrastMesh } from './contrastFlowAgent.js?v=20260614guidewirestable1';
import { PigtailCatheter } from './pigtailCatheter.js?v=20260614guidewirestable1';
import { createAortaModel } from './aortaModel.js?v=20260614guidewirestable1';
import { vertexShader as blendVS, fragmentShader as blendFS } from './shaders/blendShader.js';
import { vertexShader as thicknessVS, fragmentShader as thicknessFS } from './shaders/thicknessShader.js';
import { vertexShader as displayVS, fragmentShader as displayFS } from './shaders/displayShader.js?v=20260615fluoro9';

const LUMEN_DEBUG_COLOR = 0x29ffd4;
const WALL_CONTACT_COLOR = 0xffd24a;
const WALL_BREACH_COLOR = 0xff3355;
const CONTACT_MARKER_LIMIT = 420;
const CONTACT_MARKER_UPDATE_INTERVAL = 1 / 18;
const PIGTAIL_MESH_UPDATE_INTERVAL = 1 / 30;
const XRAY_CAMERA_NEAR = 0.1;
const XRAY_CAMERA_FAR = 1000;

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
const metalTarget = new THREE.WebGLRenderTarget(window.innerWidth, window.innerHeight);
const sheathTarget = new THREE.WebGLRenderTarget(window.innerWidth, window.innerHeight);
const boneTarget = new THREE.WebGLRenderTarget(window.innerWidth, window.innerHeight);
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
    blending: THREE.AdditiveBlending,
    side: THREE.DoubleSide,
    depthTest: true,
    depthWrite: false,
    vertexShader: `
        varying vec3 vViewNormal;
        varying vec3 vWorldPosition;
        void main() {
            vViewNormal = normalize(normalMatrix * normal);
            vec4 worldPosition = modelMatrix * vec4(position, 1.0);
            vWorldPosition = worldPosition.xyz;
            gl_Position = projectionMatrix * viewMatrix * worldPosition;
        }
    `,
    fragmentShader: `
        varying vec3 vViewNormal;
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
            float facing = abs(normalize(vViewNormal).z);
            float cortex = pow(1.0 - facing, 2.35);
            float broadDensity = pow(facing, 0.55) * 0.035;

            vec3 p = vWorldPosition * 0.035;
            float coarse = valueNoise(p);
            float fine = valueNoise(p * 2.8 + vec3(4.0, 11.0, 2.0));
            float trabeculae = smoothstep(0.38, 0.86, coarse * 0.68 + fine * 0.32);
            float striation = 0.5 + 0.5 * sin(vWorldPosition.y * 0.18 + valueNoise(p * 0.7) * 5.0);

            float corticalSignal = cortex * 0.24 + broadDensity * 0.08;
            float trabecularSignal = broadDensity * 0.72 + trabeculae * striation * 0.06;
            float totalSignal = corticalSignal + trabecularSignal * 0.62;

            gl_FragColor = vec4(corticalSignal, trabecularSignal, totalSignal, 1.0);
        }
    `
});

const displayMaterial = new THREE.ShaderMaterial({
    uniforms: {
        uTexture: { value: previousTarget.texture },
        contrastTexture: { value: contrastTarget.texture },
        thicknessTexture: { value: thicknessTarget.texture },
        metalTexture: { value: metalTarget.texture },
        sheathTexture: { value: sheathTarget.texture },
        boneTexture: { value: boneTarget.texture },
        gray: { value: new THREE.Color(0xEBEBEB) },
        fluoroscopy: { value: false },
        time: { value: 0 },
        noiseLevel: { value: 0.05 },
        boneOpacity: { value: 0.5 },
        resolution: { value: new THREE.Vector2(window.innerWidth, window.innerHeight) },
        edgeStrength: { value: 1.0 },
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
const { group: skeletonModel, material: boneMaterial } = createBoneModel();

// Lightweight centerline metadata; the visible vessel and collision surface are
// loaded from the STL aorta model.
const { vessel } = generateVessel(140, 0);
vesselGroup = new THREE.Group();
let vesselCollisionTarget = vessel;
let pigtailCatheter = null;
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
        opacity: 0.24,
        depthWrite: false,
        depthTest: true
    });
    const mesh = new THREE.Mesh(geometry, material);
    mesh.renderOrder = 3;
    return mesh;
}

function createSheathFluoroMesh(sheath) {
    const geometry = createSheathGeometry(sheath, 1.28);
    const material = new THREE.MeshBasicMaterial({
        color: 0xffffff,
        side: THREE.DoubleSide,
        transparent: true,
        opacity: 0.10,
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
vesselGroup.add(createSheathMesh(vessel.sheath));
const sheathFluoroMesh = createSheathFluoroMesh(vessel.sheath);
sheathFluoroMesh.visible = true;
scene.add(sheathFluoroMesh);
const lumenDebugGroup = new THREE.Group();
lumenDebugGroup.visible = false;
vesselGroup.add(lumenDebugGroup);
createAortaModel(vessel, {
    onLoaded: ({ collision }) => {
        vesselCollisionTarget = {
            ...collision,
            segments: [vessel.sheath]
        };
        lumenDebugGroup.clear();
        lumenDebugGroup.add(createExactLumenDebugMesh(collision.geometry));
        pigtailCatheter?.setCollisionGeometry(collision);
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
const GUIDEWIRE_FEED_VELOCITY_BLEND = 0.015;
const GUIDEWIRE_FEED_ACTIVE_LENGTH = guidewireLength;
const GUIDEWIRE_FEED_DISTAL_WEIGHT = 0.34;
const GUIDEWIRE_FEED_WALL_BAND = 3.2;
const GUIDEWIRE_RESISTANCE_CLUTCH_START = 0.42;
const GUIDEWIRE_RESISTANCE_CLUTCH_FULL = 0.92;
const GUIDEWIRE_MAX_CLUTCH_BLOCK = 0.88;
const GUIDEWIRE_BEND_CLUTCH_START = 92;
const GUIDEWIRE_BEND_CLUTCH_FULL = 145;
const GUIDEWIRE_BUCKLE_START_ANGLE = 94;
const GUIDEWIRE_BUCKLE_LIMIT_STRENGTH = 0.042;
const GUIDEWIRE_RESISTANCE_TIP_RATIO = 0.48;
const GUIDEWIRE_BODY_BENDING_STIFFNESS = 32;
const GUIDEWIRE_TIP_BENDING_STIFFNESS = 8;
const GUIDEWIRE_TIP_FLEX_LENGTH = 105;
const GUIDEWIRE_TIP_SOFT_LENGTH = 24;
const SHEATH_EXIT_SUPPORT_LENGTH = segmentLength * 2.4;
const SHEATH_EXIT_SUPPORT_BLEND = 0.12;
const SHEATH_EXIT_VELOCITY_DAMPING = 0.28;
const GUIDEWIRE_LUMEN_GLIDE_STRENGTH = 0.86;
const GUIDEWIRE_LUMEN_GLIDE_DAMPING = 0.86;
const GUIDEWIRE_LUMEN_GLIDE_START = 0;
const GUIDEWIRE_LUMEN_GLIDE_FADE = 20;
const GUIDEWIRE_LUMEN_CENTERING = 0.035;

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
const wire = new ElasticRod(nodeCount, segmentLength, {
    constraintIterations: 28
});
let tailProgress = 0;
const wireSheathPinnedState = new Array(nodeCount).fill(false);
const wireReleasedFromSheath = new Array(nodeCount).fill(false);
let guidewireResistanceLevel = 0;
let guidewireResistanceReason = '';
let guidewireFeedClutchLevel = 0;
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

function guidewireInsertedCoordinate(index) {
    return segmentLength * index - guidewireLength + tailProgress;
}

function sheathAxisPoint(inserted) {
    return {
        x: vessel.sheath.start.x + wireDir.x * inserted,
        y: vessel.sheath.start.y + wireDir.y * inserted,
        z: vessel.sheath.start.z + wireDir.z * inserted
    };
}

const guidewireLumenPath = [
    { point: new THREE.Vector3(vessel.sheath.end.x, vessel.sheath.end.y, vessel.sheath.end.z), radius: 2.8 },
    { point: new THREE.Vector3(-71, -374, 12), radius: 3.4 },
    { point: new THREE.Vector3(-69, -365, 10), radius: 4.6 },
    { point: new THREE.Vector3(-61, -338, 7), radius: 7.2 },
    { point: new THREE.Vector3(-41, -315, 4), radius: 9.8 },
    { point: new THREE.Vector3(-10, -290, 1), radius: 13.8 },
    { point: new THREE.Vector3(0, -230, 0), radius: 17.5 },
    { point: new THREE.Vector3(0, -160, 0), radius: 18.0 },
    { point: new THREE.Vector3(-9, -125, -6), radius: 17.0 },
    { point: new THREE.Vector3(28, -100, -33), radius: 16.0 },
    { point: new THREE.Vector3(38, -60, -49), radius: 13.0 },
    { point: new THREE.Vector3(36, -25, -56), radius: 13.0 },
    { point: new THREE.Vector3(19, 0, -22), radius: 16.0 },
    { point: new THREE.Vector3(-8, 35, -18), radius: 18.0 },
    { point: new THREE.Vector3(3, 95, -18), radius: 18.0 },
    { point: new THREE.Vector3(10, 145, -18), radius: 18.0 },
    { point: new THREE.Vector3(20, 230, -18), radius: 18.0 },
    { point: new THREE.Vector3(32, 330, -18), radius: 18.0 }
];
const guidewireLumenSegments = [];
let guidewireLumenLength = 0;
for (let i = 0; i < guidewireLumenPath.length - 1; i++) {
    const start = guidewireLumenPath[i].point;
    const end = guidewireLumenPath[i + 1].point;
    const length = start.distanceTo(end);
    guidewireLumenSegments.push({
        start,
        end,
        startRadius: guidewireLumenPath[i].radius,
        endRadius: guidewireLumenPath[i + 1].radius,
        length,
        offset: guidewireLumenLength
    });
    guidewireLumenLength += length;
}
const guidewireLumenCurve = new THREE.CatmullRomCurve3(
    guidewireLumenPath.map(entry => entry.point),
    false,
    'centripetal',
    0.35
);
guidewireLumenCurve.arcLengthDivisions = Math.max(200, guidewireLumenPath.length * 48);
const guidewireLumenCurveLength = guidewireLumenCurve.getLength();
const GUIDEWIRE_LUMEN_SAMPLE_SPACING = 2.5;
const guidewireLumenSamples = [];
const guidewireLumenSampleCount = Math.max(2, Math.ceil(guidewireLumenCurveLength / GUIDEWIRE_LUMEN_SAMPLE_SPACING) + 1);
for (let i = 0; i < guidewireLumenSampleCount; i++) {
    const u = i / (guidewireLumenSampleCount - 1);
    const point = guidewireLumenCurve.getPointAt(u);
    const tangent = guidewireLumenCurve.getTangentAt(u).normalize();
    guidewireLumenSamples.push({
        distance: u * guidewireLumenCurveLength,
        x: point.x,
        y: point.y,
        z: point.z,
        tx: tangent.x,
        ty: tangent.y,
        tz: tangent.z,
        radius: sampleGuidewireLumenRadius(u * guidewireLumenLength)
    });
}

function sampleGuidewireLumenRadius(distance) {
    const d = Math.max(0, distance);
    for (const seg of guidewireLumenSegments) {
        if (d <= seg.offset + seg.length) {
            const t = Math.max(0, Math.min(1, (d - seg.offset) / Math.max(1e-6, seg.length)));
            return seg.startRadius * (1 - t) + seg.endRadius * t;
        }
    }

    const last = guidewireLumenSegments[guidewireLumenSegments.length - 1];
    return last.endRadius;
}

function sampleGuidewireLumen(distance) {
    const d = Math.max(0, distance);
    if (d <= guidewireLumenCurveLength) {
        const samplePosition = d / Math.max(1e-6, guidewireLumenCurveLength) * (guidewireLumenSamples.length - 1);
        const lowerIndex = Math.max(0, Math.min(guidewireLumenSamples.length - 2, Math.floor(samplePosition)));
        const upperIndex = lowerIndex + 1;
        const t = samplePosition - lowerIndex;
        const a = guidewireLumenSamples[lowerIndex];
        const b = guidewireLumenSamples[upperIndex];
        let tx = a.tx * (1 - t) + b.tx * t;
        let ty = a.ty * (1 - t) + b.ty * t;
        let tz = a.tz * (1 - t) + b.tz * t;
        const tangentLength = Math.hypot(tx, ty, tz) || 1;
        tx /= tangentLength;
        ty /= tangentLength;
        tz /= tangentLength;
        return {
            point: {
                x: a.x * (1 - t) + b.x * t,
                y: a.y * (1 - t) + b.y * t,
                z: a.z * (1 - t) + b.z * t
            },
            tangent: { x: tx, y: ty, z: tz },
            radius: a.radius * (1 - t) + b.radius * t
        };
    }

    const last = guidewireLumenSegments[guidewireLumenSegments.length - 1];
    const endSample = guidewireLumenSamples[guidewireLumenSamples.length - 1];
    const overrun = d - guidewireLumenCurveLength;
    return {
        point: {
            x: endSample.x + endSample.tx * overrun,
            y: endSample.y + endSample.ty * overrun,
            z: endSample.z + endSample.tz * overrun
        },
        tangent: { x: endSample.tx, y: endSample.ty, z: endSample.tz },
        radius: last.endRadius
    };
}

function constrainWireToSheath(feedSpeed = 0) {
    for (let i = 0; i < wire.nodes.length; i++) {
        const inserted = guidewireInsertedCoordinate(i);
        const inSheath = inserted <= sheathPath;
        const n = wire.nodes[i];
        const wasInSheath = wireSheathPinnedState[i];
        const released = wasInSheath && !inSheath;
        n.pinned = inSheath;
        wireReleasedFromSheath[i] = released;
        if (inSheath || released) {
            const target = sheathAxisPoint(inserted);
            n.x = target.x;
            n.y = target.y;
            n.z = target.z;
            const speed = inSheath ? 0 : Math.max(0, feedSpeed);
            n.vx = wireDir.x * speed;
            n.vy = wireDir.y * speed;
            n.vz = wireDir.z * speed;
        }
        wireSheathPinnedState[i] = inSheath;
    }
}

function guidewireTangentAt(index) {
    const prev = wire.nodes[Math.max(0, index - 1)];
    const next = wire.nodes[Math.min(wire.nodes.length - 1, index + 1)];
    let tx = next.x - prev.x;
    let ty = next.y - prev.y;
    let tz = next.z - prev.z;
    let length = Math.hypot(tx, ty, tz);
    if (length < 1e-6 && index > 0) {
        const n = wire.nodes[index];
        const p = wire.nodes[index - 1];
        tx = n.x - p.x;
        ty = n.y - p.y;
        tz = n.z - p.z;
        length = Math.hypot(tx, ty, tz);
    }
    if (length < 1e-6) {
        return { x: wireDir.x, y: wireDir.y, z: wireDir.z };
    }
    return { x: tx / length, y: ty / length, z: tz / length };
}

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

function guidewireMeshCollider() {
    return vesselCollisionTarget?.meshCollider || vesselCollisionTarget?.lumenMeshCollider || null;
}

function normalizeDirection(direction, fallback = wireDir) {
    const length = Math.hypot(direction.x, direction.y, direction.z);
    if (length < 1e-6) {
        return { x: fallback.x, y: fallback.y, z: fallback.z };
    }
    return {
        x: direction.x / length,
        y: direction.y / length,
        z: direction.z / length
    };
}

function projectGuidewireFeedDirection(point, direction) {
    const collider = guidewireMeshCollider();
    if (!collider?.pointContact) return normalizeDirection(direction);

    const contact = collider.pointContact(point, 0);
    if (!contact || !Number.isFinite(contact.distance)) return normalizeDirection(direction);

    const projected = {
        x: direction.x,
        y: direction.y,
        z: direction.z
    };

    const normal = contact.normal;
    const nearWall = contact.violation || contact.distance <= GUIDEWIRE_FEED_WALL_BAND;
    if (nearWall && normal) {
        const dot = projected.x * normal.x + projected.y * normal.y + projected.z * normal.z;
        if (dot > 0) {
            projected.x -= normal.x * dot;
            projected.y -= normal.y * dot;
            projected.z -= normal.z * dot;
        }
    }

    if (contact.violation && contact.target) {
        const correction = {
            x: contact.target.x - point.x,
            y: contact.target.y - point.y,
            z: contact.target.z - point.z
        };
        const inward = normalizeDirection(correction, projected);
        projected.x = projected.x * 0.72 + inward.x * 0.28;
        projected.y = projected.y * 0.72 + inward.y * 0.28;
        projected.z = projected.z * 0.72 + inward.z * 0.28;
    }

    return normalizeDirection(projected, direction);
}

function samplePreviousGuidewirePosition(previousPositions, sourceIndex) {
    const lastIndex = previousPositions.length - 1;
    if (sourceIndex <= 0) {
        return { ...previousPositions[0] };
    }
    if (sourceIndex < lastIndex) {
        const lower = Math.floor(sourceIndex);
        const upper = Math.min(lastIndex, lower + 1);
        const t = sourceIndex - lower;
        const p0 = previousPositions[lower];
        const p1 = previousPositions[upper];
        return {
            x: p0.x * (1 - t) + p1.x * t,
            y: p0.y * (1 - t) + p1.y * t,
            z: p0.z * (1 - t) + p1.z * t
        };
    }

    const tip = previousPositions[lastIndex];
    const prev = previousPositions[Math.max(0, lastIndex - 1)];
    const direction = projectGuidewireFeedDirection(tip, {
        x: tip.x - prev.x,
        y: tip.y - prev.y,
        z: tip.z - prev.z
    });
    const distance = (sourceIndex - lastIndex) * segmentLength;
    return {
        x: tip.x + direction.x * distance,
        y: tip.y + direction.y * distance,
        z: tip.z + direction.z * distance
    };
}

function guidewireAdvanceClutch(advance) {
    if (advance <= 0) {
        guidewireFeedClutchLevel = 0;
        return 1;
    }

    const bendStats = guidewireBendStats();
    const resistanceBlock = smoothRange(
        GUIDEWIRE_RESISTANCE_CLUTCH_START,
        GUIDEWIRE_RESISTANCE_CLUTCH_FULL,
        guidewireResistanceLevel
    );
    const bendBlock = smoothRange(
        GUIDEWIRE_BEND_CLUTCH_START,
        GUIDEWIRE_BEND_CLUTCH_FULL,
        bendStats.maxAngle
    );
    guidewireFeedClutchLevel = Math.max(resistanceBlock, bendBlock);
    return 1 - guidewireFeedClutchLevel * GUIDEWIRE_MAX_CLUTCH_BLOCK;
}

function feedGuidewireMaterial(delta, dt, previousPositions) {
    if (Math.abs(delta) < 1e-6) return;
    if (delta > 0 && previousPositions?.length === wire.nodes.length) {
        const sourceShift = delta / segmentLength;
        const invDt = 1 / Math.max(dt, 1e-6);

        for (let i = 0; i < wire.nodes.length; i++) {
            const n = wire.nodes[i];
            if (n.pinned) continue;

            const inserted = guidewireInsertedCoordinate(i);
            if (inserted <= sheathPath) continue;

            const target = samplePreviousGuidewirePosition(previousPositions, i + sourceShift);
            const exitDistance = inserted - sheathPath;
            const sheathBlend = Math.max(0, Math.min(1, 1 - exitDistance / SHEATH_EXIT_SUPPORT_LENGTH));
            if (sheathBlend > 0) {
                const sheathTarget = sheathAxisPoint(inserted);
                const axial = {
                    x: sheathTarget.x * sheathBlend + target.x * (1 - sheathBlend),
                    y: sheathTarget.y * sheathBlend + target.y * (1 - sheathBlend),
                    z: sheathTarget.z * sheathBlend + target.z * (1 - sheathBlend)
                };
                target.x = axial.x;
                target.y = axial.y;
                target.z = axial.z;
            }

            const old = previousPositions[i];
            const moveX = target.x - n.x;
            const moveY = target.y - n.y;
            const moveZ = target.z - n.z;
            n.x = target.x;
            n.y = target.y;
            n.z = target.z;
            n.vx = n.vx * 0.18 + (target.x - old.x) * invDt * 0.06 + moveX * invDt * 0.04;
            n.vy = n.vy * 0.18 + (target.y - old.y) * invDt * 0.06 + moveY * invDt * 0.04;
            n.vz = n.vz * 0.18 + (target.z - old.z) * invDt * 0.06 + moveZ * invDt * 0.04;
        }
        return;
    }

    const speed = delta / Math.max(dt, 1e-6);
    const tangents = wire.nodes.map((_, i) => guidewireTangentAt(i));

    for (let i = 0; i < wire.nodes.length; i++) {
        const n = wire.nodes[i];
        if (n.pinned || wireReleasedFromSheath[i]) continue;

        const inserted = guidewireInsertedCoordinate(i);
        if (inserted <= sheathPath) continue;

        const exitDistance = inserted - sheathPath;
        if (exitDistance > GUIDEWIRE_FEED_ACTIVE_LENGTH) continue;
        const feedTaper = 1 - exitDistance / GUIDEWIRE_FEED_ACTIVE_LENGTH;
        const proximalWeight = feedTaper * feedTaper * (3 - 2 * feedTaper);
        const feedWeight = GUIDEWIRE_FEED_DISTAL_WEIGHT + (1 - GUIDEWIRE_FEED_DISTAL_WEIGHT) * proximalWeight;
        const sheathBlend = Math.max(0, Math.min(1, 1 - exitDistance / SHEATH_EXIT_SUPPORT_LENGTH));
        const tangent = tangents[i];
        let tx = tangent.x * (1 - sheathBlend) + wireDir.x * sheathBlend;
        let ty = tangent.y * (1 - sheathBlend) + wireDir.y * sheathBlend;
        let tz = tangent.z * (1 - sheathBlend) + wireDir.z * sheathBlend;
        const length = Math.hypot(tx, ty, tz) || 1;
        tx /= length;
        ty /= length;
        tz /= length;
        const projected = projectGuidewireFeedDirection(n, { x: tx, y: ty, z: tz });
        tx = projected.x;
        ty = projected.y;
        tz = projected.z;

        n.x += tx * delta * feedWeight;
        n.y += ty * delta * feedWeight;
        n.z += tz * delta * feedWeight;
        n.vx += tx * speed * GUIDEWIRE_FEED_VELOCITY_BLEND * feedWeight;
        n.vy += ty * speed * GUIDEWIRE_FEED_VELOCITY_BLEND * feedWeight;
        n.vz += tz * speed * GUIDEWIRE_FEED_VELOCITY_BLEND * feedWeight;
    }
}

function guidewireBendAngleAt(index) {
    const prev = wire.nodes[index - 1];
    const curr = wire.nodes[index];
    const next = wire.nodes[index + 1];
    if (!prev || !curr || !next) return 0;

    const ax = curr.x - prev.x;
    const ay = curr.y - prev.y;
    const az = curr.z - prev.z;
    const bx = next.x - curr.x;
    const by = next.y - curr.y;
    const bz = next.z - curr.z;
    const aLen = Math.hypot(ax, ay, az);
    const bLen = Math.hypot(bx, by, bz);
    if (aLen < 1e-6 || bLen < 1e-6) return 0;

    const dot = (ax * bx + ay * by + az * bz) / (aLen * bLen);
    return Math.acos(Math.max(-1, Math.min(1, dot))) * 180 / Math.PI;
}

function guidewireBendStats() {
    let maxAngle = 0;
    let maxInserted = 0;
    for (let i = 1; i < wire.nodes.length - 1; i++) {
        if (wire.nodes[i].pinned) continue;
        const inserted = guidewireInsertedCoordinate(i);
        if (inserted <= sheathPath + segmentLength) continue;
        const angle = guidewireBendAngleAt(i);
        if (angle > maxAngle) {
            maxAngle = angle;
            maxInserted = inserted;
        }
    }
    return { maxAngle, maxInserted };
}

function limitGuidewireBuckling(strength = GUIDEWIRE_BUCKLE_LIMIT_STRENGTH, iterations = 1) {
    if (strength <= 0) return;
    for (let iter = 0; iter < iterations; iter++) {
        const corrections = new Array(wire.nodes.length);
        for (let i = 1; i < wire.nodes.length - 1; i++) {
            const n = wire.nodes[i];
            if (n.pinned) continue;
            const inserted = guidewireInsertedCoordinate(i);
            if (inserted <= sheathPath + segmentLength) continue;

            const angle = guidewireBendAngleAt(i);
            if (angle <= GUIDEWIRE_BUCKLE_START_ANGLE) continue;

            const prev = wire.nodes[i - 1];
            const next = wire.nodes[i + 1];
            const severity = Math.max(0, Math.min(1, (angle - GUIDEWIRE_BUCKLE_START_ANGLE) / (170 - GUIDEWIRE_BUCKLE_START_ANGLE)));
            corrections[i] = {
                x: ((prev.x + next.x) * 0.5 - n.x) * strength * severity,
                y: ((prev.y + next.y) * 0.5 - n.y) * strength * severity,
                z: ((prev.z + next.z) * 0.5 - n.z) * strength * severity
            };
        }

        for (let i = 1; i < wire.nodes.length - 1; i++) {
            const n = wire.nodes[i];
            const c = corrections[i];
            if (!c || n.pinned) continue;
            n.x += c.x;
            n.y += c.y;
            n.z += c.z;
            n.vx *= 0.86;
            n.vy *= 0.86;
            n.vz *= 0.86;
        }
    }
}

function supportWireAtSheathExit(strength = 1) {
    if (strength <= 0) return;
    for (let i = 0; i < wire.nodes.length; i++) {
        const inserted = guidewireInsertedCoordinate(i);
        const exitDistance = inserted - sheathPath;
        if (exitDistance <= 0 || exitDistance > SHEATH_EXIT_SUPPORT_LENGTH) continue;

        const n = wire.nodes[i];
        if (n.pinned) continue;

        const target = sheathAxisPoint(inserted);
        const dx = n.x - target.x;
        const dy = n.y - target.y;
        const dz = n.z - target.z;
        const axialOffset = dx * wireDir.x + dy * wireDir.y + dz * wireDir.z;
        const lateralX = dx - wireDir.x * axialOffset;
        const lateralY = dy - wireDir.y * axialOffset;
        const lateralZ = dz - wireDir.z * axialOffset;
        const t = 1 - exitDistance / SHEATH_EXIT_SUPPORT_LENGTH;
        const taper = t * t * (3 - 2 * t);
        const amount = SHEATH_EXIT_SUPPORT_BLEND * taper * strength;

        n.x -= lateralX * amount;
        n.y -= lateralY * amount;
        n.z -= lateralZ * amount;

        const axialVelocity = n.vx * wireDir.x + n.vy * wireDir.y + n.vz * wireDir.z;
        const velocityDamping = SHEATH_EXIT_VELOCITY_DAMPING * taper * strength;
        n.vx -= (n.vx - wireDir.x * axialVelocity) * velocityDamping;
        n.vy -= (n.vy - wireDir.y * axialVelocity) * velocityDamping;
        n.vz -= (n.vz - wireDir.z * axialVelocity) * velocityDamping;
    }
}

function applyGuidewireLumenGlide(strength = GUIDEWIRE_LUMEN_GLIDE_STRENGTH) {
    if (strength <= 0) return;
    for (let i = 0; i < wire.nodes.length; i++) {
        const n = wire.nodes[i];
        if (n.pinned) continue;

        const inserted = guidewireInsertedCoordinate(i);
        const exitDistance = inserted - sheathPath;
        if (exitDistance <= GUIDEWIRE_LUMEN_GLIDE_START) continue;

        const { point, tangent, radius } = sampleGuidewireLumen(exitDistance);
        const fade = smoothRange(
            GUIDEWIRE_LUMEN_GLIDE_START,
            GUIDEWIRE_LUMEN_GLIDE_START + GUIDEWIRE_LUMEN_GLIDE_FADE,
            exitDistance
        );
        const distalTaper = 1 - Math.max(0, Math.min(1, exitDistance / guidewireLength));
        const amount = strength * fade * (0.38 + distalTaper * 0.62);
        if (amount <= 0) continue;

        const dx = n.x - point.x;
        const dy = n.y - point.y;
        const dz = n.z - point.z;
        const axialOffset = dx * tangent.x + dy * tangent.y + dz * tangent.z;
        const lateralX = dx - tangent.x * axialOffset;
        const lateralY = dy - tangent.y * axialOffset;
        const lateralZ = dz - tangent.z * axialOffset;
        const lateralLength = Math.hypot(lateralX, lateralY, lateralZ);
        const corridorRadius = Math.max(1.2, radius - 0.8);

        if (lateralLength > corridorRadius) {
            const scale = corridorRadius / Math.max(1e-6, lateralLength);
            const targetX = point.x + tangent.x * axialOffset + lateralX * scale;
            const targetY = point.y + tangent.y * axialOffset + lateralY * scale;
            const targetZ = point.z + tangent.z * axialOffset + lateralZ * scale;
            n.x += (targetX - n.x) * amount;
            n.y += (targetY - n.y) * amount;
            n.z += (targetZ - n.z) * amount;
        }

        const axialLimit = segmentLength * 0.6;
        if (Math.abs(axialOffset) > axialLimit) {
            const targetAxial = Math.sign(axialOffset) * axialLimit;
            const axialCorrection = (targetAxial - axialOffset) * amount * 0.62;
            n.x += tangent.x * axialCorrection;
            n.y += tangent.y * axialCorrection;
            n.z += tangent.z * axialCorrection;
        }

        const centering = GUIDEWIRE_LUMEN_CENTERING * amount;
        n.x += (point.x - n.x) * centering;
        n.y += (point.y - n.y) * centering;
        n.z += (point.z - n.z) * centering;

        const axialVelocity = n.vx * tangent.x + n.vy * tangent.y + n.vz * tangent.z;
        const damping = GUIDEWIRE_LUMEN_GLIDE_DAMPING * amount;
        n.vx -= (n.vx - tangent.x * axialVelocity) * damping;
        n.vy -= (n.vy - tangent.y * axialVelocity) * damping;
        n.vz -= (n.vz - tangent.z * axialVelocity) * damping;
    }
}

// The proximal guidewire and the part inside the introducer sheath are
// constrained by the sheath lumen. Once a node exits the sheath tip it becomes
// free and is governed by rod stiffness and vessel-wall collision.
applyGuidewireStiffnessProfile();
constrainWireToSheath();
supportWireAtSheathExit();
applyGuidewireLumenGlide(0.5);

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
let wallContactMarkers = null;
let wallBreachMarkers = null;
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
        if (wallContactMarkers) wallContactMarkers.visible = true;
        if (wallBreachMarkers) wallBreachMarkers.visible = true;
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

const contactMarkerGeometry = new THREE.SphereGeometry(2.1, 12, 8);
wallContactMarkers = new THREE.InstancedMesh(
    contactMarkerGeometry,
    new THREE.MeshBasicMaterial({
        color: WALL_CONTACT_COLOR,
        transparent: true,
        opacity: 0.95,
        depthTest: false
    }),
    CONTACT_MARKER_LIMIT
);
wallContactMarkers.instanceMatrix.setUsage(THREE.DynamicDrawUsage);
wallContactMarkers.count = 0;
wallContactMarkers.visible = true;
wallContactMarkers.renderOrder = 6;
scene.add(wallContactMarkers);

wallBreachMarkers = new THREE.InstancedMesh(
    contactMarkerGeometry,
    new THREE.MeshBasicMaterial({
        color: WALL_BREACH_COLOR,
        transparent: true,
        opacity: 1,
        depthTest: false
    }),
    CONTACT_MARKER_LIMIT
);
wallBreachMarkers.instanceMatrix.setUsage(THREE.DynamicDrawUsage);
wallBreachMarkers.count = 0;
wallBreachMarkers.visible = true;
wallBreachMarkers.renderOrder = 7;
scene.add(wallBreachMarkers);

pigtailCatheter = new PigtailCatheter({
    wire,
    segmentLength,
    guidewireLength,
    tailProgressRef: () => tailProgress,
    vessel
});
if (vesselCollisionTarget !== vessel) {
    pigtailCatheter.setCollisionGeometry(vesselCollisionTarget);
}
scene.add(pigtailCatheter.mesh);

function advanceTailInput(advance, dt) {
    const clutch = guidewireAdvanceClutch(advance);
    const nextProgress = Math.max(minInsert, Math.min(maxInsert, tailProgress + advance * GUIDEWIRE_ADVANCE_RATE * clutch * dt));
    const delta = nextProgress - tailProgress;
    const feedSpeed = delta / Math.max(dt, 1e-6);
    const previousPositions = wire.nodes.map(n => ({ x: n.x, y: n.y, z: n.z }));
    tailProgress = nextProgress;
    constrainWireToSheath(feedSpeed);
    feedGuidewireMaterial(delta, dt, previousPositions);
    supportWireAtSheathExit(0.7);
    if (advance > 0) applyGuidewireLumenGlide(0.72);
    return delta;
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

function sampleGuidewireContactMarkers() {
    const collider = vesselCollisionTarget.meshCollider || vesselCollisionTarget.lumenMeshCollider;
    const markerMatrix = new THREE.Matrix4();
    const contactSamples = [];
    const breachSamples = [];
    const samples = [0.15, 0.35, 0.55, 0.75, 0.9];
    const contactBand = 1.75;
    if (!collider?.pointContact) {
        wallContactMarkers.count = 0;
        wallBreachMarkers.count = 0;
        return;
    }

    for (let i = 0; i < wire.nodes.length - 1; i++) {
        const n0 = wire.nodes[i];
        const n1 = wire.nodes[i + 1];
        for (const t of samples) {
            const inserted = segmentLength * (i + t) - guidewireLength + tailProgress;
            if (inserted <= sheathPath) continue;

            const p = {
                x: n0.x * (1 - t) + n1.x * t,
                y: n0.y * (1 - t) + n1.y * t,
                z: n0.z * (1 - t) + n1.z * t
            };
            const contact = collider.pointContact(p, 0);
            if (!Number.isFinite(contact.signedDistance)) continue;

            if (!contact.inside || contact.signedDistance > 0.1) {
                breachSamples.push(p);
            } else if (contact.distance < contactBand) {
                contactSamples.push(p);
            }
        }
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

    applySamples(wallContactMarkers, contactSamples);
    applySamples(wallBreachMarkers, breachSamples);
}

function guidewireTipPosition(target = new THREE.Vector3()) {
    const tip = wire.nodes[wire.nodes.length - 1];
    return target.set(tip.x, tip.y, tip.z);
}

function updateGuidewireResistance(advance, commandedDelta, tipBefore) {
    let instantLevel = 0;
    let reason = '';
    if (advance > 0 && commandedDelta > 1e-5) {
        const tipAfter = guidewireTipPosition();
        const tipTravel = tipAfter.distanceTo(tipBefore);
        const tipRatio = tipTravel / commandedDelta;
        const bendStats = guidewireBendStats();
        const stallLevel = Math.max(0, Math.min(1, (GUIDEWIRE_RESISTANCE_TIP_RATIO - tipRatio) / GUIDEWIRE_RESISTANCE_TIP_RATIO));
        const buckleLevel = Math.max(0, Math.min(1, (bendStats.maxAngle - GUIDEWIRE_BUCKLE_START_ANGLE) / (170 - GUIDEWIRE_BUCKLE_START_ANGLE)));

        if (stallLevel >= buckleLevel && stallLevel > 0) {
            instantLevel = stallLevel;
            reason = 'Opór na prowadniku - końcówka nie postępuje proporcjonalnie do podawania.';
        } else if (buckleLevel > 0) {
            instantLevel = buckleLevel;
            reason = 'Opór na prowadniku - prowadnik zaczyna się zawijać.';
        }
    }
    if (advance > 0 && guidewireFeedClutchLevel > instantLevel) {
        instantLevel = guidewireFeedClutchLevel;
        reason = guidewireFeedClutchLevel > 0.55
            ? 'Opór na prowadniku - zmniejszono podawanie, żeby nie zawijać prowadnika.'
            : 'Narastający opór na prowadniku.';
    }

    const rise = instantLevel > guidewireResistanceLevel ? 0.28 : 0.08;
    guidewireResistanceLevel += (instantLevel - guidewireResistanceLevel) * rise;
    if (instantLevel > 0.2) {
        guidewireResistanceReason = reason;
    } else if (guidewireResistanceLevel < 0.2) {
        guidewireResistanceReason = '';
    }

    ui.updateGuidewireResistance(guidewireResistanceLevel, guidewireResistanceReason);
}

const fixedDt = 1 / 60;
let lastRenderTime = performance.now();
let contactMarkerAccumulator = CONTACT_MARKER_UPDATE_INTERVAL;
let pigtailMeshAccumulator = PIGTAIL_MESH_UPDATE_INTERVAL;

function stepSimulation() {
    // Advance input, integrate rod physics, collisions, and update medical monitors
    const advance = ui.getAdvance();
    const tipBefore = guidewireTipPosition();
    const commandedDelta = advanceTailInput(advance, fixedDt);
    wire.step(fixedDt);
    wire.collide(vesselCollisionTarget, fixedDt);
    wire.solveConstraints(fixedDt);
    supportWireAtSheathExit();
    if (advance > 0) applyGuidewireLumenGlide();
    if (advance > 0) {
        limitGuidewireBuckling(GUIDEWIRE_BUCKLE_LIMIT_STRENGTH, 2);
        wire.solveConstraints(fixedDt);
        wire.collide(vesselCollisionTarget, fixedDt);
        applyGuidewireLumenGlide();
    }
    if (advance < 0) {
        if (vesselCollisionTarget === vessel) {
            wire.releaseFromVesselWall(vessel.segments, 0.08, 2);
            wire.solveConstraints(fixedDt);
            supportWireAtSheathExit(0.8);
        }
        wire.straightenByTension(0.18, 4);
        wire.solveConstraints(fixedDt);
        supportWireAtSheathExit(0.8);
    }
    wire.collide(vesselCollisionTarget, fixedDt);
    supportWireAtSheathExit(0.7);
    if (advance > 0) {
        limitGuidewireBuckling(0.055, 2);
        applyGuidewireLumenGlide(0.68);
    }
    const inserted = Math.max(0, tailProgress);
    pigtailCatheter.advance(ui.getCatheterAdvance(), fixedDt, inserted);
    pigtailCatheter.rotate(ui.getCatheterRotation(), fixedDt);
    pigtailCatheter.stepPhysics(fixedDt);
    pigtailCatheter.constrainGuidewire(fixedDt);
    wire.solveConstraints(fixedDt);
    supportWireAtSheathExit();
    pigtailCatheter.constrainGuidewire(fixedDt);
    wire.collide(vesselCollisionTarget, fixedDt);
    wire.solveConstraints(fixedDt);
    supportWireAtSheathExit(0.8);
    if (advance > 0) applyGuidewireLumenGlide(0.52);
    updateGuidewireResistance(advance, Math.max(0, commandedDelta), tipBefore);
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
    skeletonModel.visible = fluoroscopy;
    ui.setInjectButtonDisabled(contrastActive);
    ui.setStopInjectionDisabled(!injecting);
    if (fluoroscopy) {
        // Fluoroscopy path:
        // 1) render front/back depth for thickness
        // 2) render stable bone/contrast/metal masks for attenuation
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
            renderOnlySceneObjects(scene, camera, [
                wireMesh,
                pigtailCatheter.mesh
            ]);
        });

        renderer.setRenderTarget(sheathTarget);
        withTransparentClear(renderer, () => {
            renderer.clear();
            renderOnlySceneObjects(scene, camera, [sheathFluoroMesh]);
        });

        renderer.setRenderTarget(offscreenTarget);
        renderer.clear();
        renderOnlySceneObjects(scene, camera, [
            skeletonModel,
            sheathFluoroMesh,
            wireMesh,
            pigtailCatheter.mesh
        ]);

        blendMaterial.uniforms.currentFrame.value = offscreenTarget.texture;
        blendMaterial.uniforms.previousFrame.value = previousTarget.texture;

        renderer.setRenderTarget(currentTarget);
        renderer.render(blendScene, postCamera);
        renderer.setRenderTarget(null);

        displayMaterial.uniforms.uTexture.value = currentTarget.texture;
        displayMaterial.uniforms.contrastTexture.value = contrastTarget.texture;
        displayMaterial.uniforms.thicknessTexture.value = thicknessTarget.texture;
        displayMaterial.uniforms.metalTexture.value = metalTarget.texture;
        displayMaterial.uniforms.sheathTexture.value = sheathTarget.texture;
        displayMaterial.uniforms.boneTexture.value = boneTarget.texture;
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
    metalTarget.setSize(w, h);
    sheathTarget.setSize(w, h);
    boneTarget.setSize(w, h);
    accumulateTarget1.setSize(w, h);
    accumulateTarget2.setSize(w, h);
    frontDepthTarget.setSize(w, h);
    backDepthTarget.setSize(w, h);
    thicknessTarget.setSize(w, h);
    displayMaterial.uniforms.resolution.value.set(w, h);
});
