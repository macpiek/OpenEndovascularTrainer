import * as THREE from 'three';
import { createCArmModel } from './carmModel.js?v=20260620rollpreview1';
import { createOperatingTable } from './operatingTable.js?v=20260614carmaxis3';

let previewScene;
let previewCamera;
let previewRenderer;
let cArmGroup;
let cArmGantry;
let cArmLift;
let cArmTable;
let cArmDetectorAssembly;
const PREVIEW_CAMERA_TARGET = new THREE.Vector3(0, 24, -30);

export function initCArmPreview() {
    const container = document.getElementById('carm-preview');
    if (!container) return null;
    container.replaceChildren();

    previewScene = new THREE.Scene();
    previewScene.background = new THREE.Color(0x020303);

    // Simple lighting so models are visible in the preview.
    const ambient = new THREE.AmbientLight(0xdde7e4, 0.72);
    previewScene.add(ambient);
    const dirLight = new THREE.DirectionalLight(0xffffff, 0.85);
    dirLight.position.set(120, 180, 160);
    previewScene.add(dirLight);
    const fillLight = new THREE.DirectionalLight(0x9ab9b3, 0.24);
    fillLight.position.set(-160, 40, -130);
    previewScene.add(fillLight);

    const width = container.clientWidth;
    const height = container.clientHeight;

    previewCamera = new THREE.PerspectiveCamera(39, width / height, 0.1, 1000);
    previewCamera.position.set(268, 146, 289);
    previewCamera.lookAt(PREVIEW_CAMERA_TARGET);
    previewScene.add(previewCamera);

    previewRenderer = new THREE.WebGLRenderer({ antialias: true, alpha: true });
    previewRenderer.setSize(width, height);
    previewRenderer.setPixelRatio(Math.min(window.devicePixelRatio || 1, 2));
    container.appendChild(previewRenderer.domElement);

    const floor = new THREE.GridHelper(300, 12, 0x485a57, 0x1d2524);
    floor.position.y = -94;
    previewScene.add(floor);

    cArmTable = createOperatingTable();
    previewScene.add(cArmTable);

    cArmGroup = new THREE.Group();
    const { group: cArm, gantryGroup, liftGroup, detectorAssembly } = createCArmModel();
    cArmGantry = gantryGroup;
    cArmLift = liftGroup;
    cArmDetectorAssembly = detectorAssembly;
    cArmGroup.add(cArm);
    previewScene.add(cArmGroup);

    // Render once so the preview displays immediately.
    renderCArmPreview();
    return {
        group: cArmGroup,
        gantry: cArmGantry,
        detectorAssembly: cArmDetectorAssembly,
        lift: cArmLift,
        table: cArmTable
    };
}

export function renderCArmPreview() {
    if (!previewRenderer || !previewScene || !previewCamera) return;
    previewRenderer.render(previewScene, previewCamera);
}

export {
    cArmGroup as cArmPreviewGroup,
    cArmGantry as cArmPreviewGantry,
    cArmDetectorAssembly as cArmPreviewDetectorAssembly,
    cArmTable as cArmPreviewTable
};
