import * as THREE from 'three';
import { createCArmModel } from './carmModel.js';
import { createOperatingTable } from './operatingTable.js';

let previewScene;
let previewCamera;
let previewRenderer;
let cArmGroup;

export function initCArmPreview() {
    const container = document.getElementById('carm-preview');
    if (!container) return;

    previewScene = new THREE.Scene();

    // Simple lighting so models are visible in the preview.
    const ambient = new THREE.AmbientLight(0xffffff, 0.8);
    previewScene.add(ambient);
    const dirLight = new THREE.DirectionalLight(0xffffff, 0.5);
    dirLight.position.set(1, 1, 1);
    previewScene.add(dirLight);

    const width = container.clientWidth;
    const height = container.clientHeight;

    previewCamera = new THREE.PerspectiveCamera(45, width / height, 0.1, 1000);
    previewCamera.position.set(200, 150, 300);
    previewCamera.lookAt(0, 0, 0);
    previewScene.add(previewCamera);

    previewRenderer = new THREE.WebGLRenderer({ antialias: true });
    previewRenderer.setSize(width, height);
    container.appendChild(previewRenderer.domElement);

    const table = createOperatingTable();
    // Lower the table so the patient lies below the C-arm's isocenter
    // making the gantry clearly visible in the preview.
    table.position.y = -120;
    previewScene.add(table);

    cArmGroup = new THREE.Group();
    const cArm = createCArmModel();
    cArm.position.y = -70; // align gantry center with the group's origin
    cArmGroup.add(cArm);
    previewScene.add(cArmGroup);

    // Render once so the preview displays immediately.
    renderCArmPreview();
}

export function renderCArmPreview() {
    if (!previewRenderer || !previewScene || !previewCamera) return;
    previewRenderer.render(previewScene, previewCamera);
}

export { previewScene as cArmPreviewScene, previewCamera as cArmPreviewCamera, cArmGroup as cArmPreviewGroup };
