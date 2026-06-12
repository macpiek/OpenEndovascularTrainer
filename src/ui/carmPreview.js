import * as THREE from 'three';
import { createCArmModel } from './carmModel.js?v=20260611carmmodel1';
import { createOperatingTable } from './operatingTable.js?v=20260611carmmodel1';

let previewScene;
let previewCamera;
let previewRenderer;
let cArmGroup;
let cArmGantry;
let cArmTable;

export function initCArmPreview() {
    const container = document.getElementById('carm-preview');
    if (!container) return;

    previewScene = new THREE.Scene();
    previewScene.background = new THREE.Color(0x071725);

    // Simple lighting so models are visible in the preview.
    const ambient = new THREE.AmbientLight(0xd7efff, 0.8);
    previewScene.add(ambient);
    const dirLight = new THREE.DirectionalLight(0xffffff, 0.85);
    dirLight.position.set(120, 180, 160);
    previewScene.add(dirLight);
    const fillLight = new THREE.DirectionalLight(0x87b9ff, 0.35);
    fillLight.position.set(-160, 40, -130);
    previewScene.add(fillLight);

    const width = container.clientWidth;
    const height = container.clientHeight;

    previewCamera = new THREE.PerspectiveCamera(39, width / height, 0.1, 1000);
    previewCamera.position.set(210, 112, 245);
    previewCamera.lookAt(-8, 4, -14);
    previewScene.add(previewCamera);

    previewRenderer = new THREE.WebGLRenderer({ antialias: true, alpha: true });
    previewRenderer.setSize(width, height);
    previewRenderer.setPixelRatio(Math.min(window.devicePixelRatio || 1, 2));
    container.appendChild(previewRenderer.domElement);

    const floor = new THREE.GridHelper(300, 12, 0x2f566f, 0x183247);
    floor.position.y = -94;
    previewScene.add(floor);

    cArmTable = createOperatingTable();
    previewScene.add(cArmTable);

    cArmGroup = new THREE.Group();
    const { group: cArm, gantryGroup } = createCArmModel();
    cArmGantry = gantryGroup;
    cArmGroup.add(cArm);
    previewScene.add(cArmGroup);

    // Render once so the preview displays immediately.
    renderCArmPreview();
}

export function renderCArmPreview() {
    if (!previewRenderer || !previewScene || !previewCamera) return;
    previewRenderer.render(previewScene, previewCamera);
}

export {
    previewScene as cArmPreviewScene,
    previewCamera as cArmPreviewCamera,
    cArmGroup as cArmPreviewGroup,
    cArmGantry as cArmPreviewGantry,
    cArmTable as cArmPreviewTable
};
