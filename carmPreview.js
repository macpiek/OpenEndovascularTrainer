import * as THREE from 'three';

let previewScene;
let previewCamera;
let previewRenderer;

export function initCArmPreview() {
    const container = document.getElementById('carm-preview');
    if (!container) return;

    previewScene = new THREE.Scene();
    const width = container.clientWidth;
    const height = container.clientHeight;

    previewCamera = new THREE.PerspectiveCamera(45, width / height, 0.1, 1000);
    previewCamera.position.set(0, 0, 5);
    previewScene.add(previewCamera);

    previewRenderer = new THREE.WebGLRenderer({ antialias: true });
    previewRenderer.setSize(width, height);
    container.appendChild(previewRenderer.domElement);
}

export function renderCArmPreview() {
    if (!previewRenderer || !previewScene || !previewCamera) return;
    previewRenderer.render(previewScene, previewCamera);
}

export { previewScene as cArmPreviewScene, previewCamera as cArmPreviewCamera };
