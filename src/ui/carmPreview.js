import * as THREE from 'three';
import { createCArmModel } from './carmModel.js';
import { createOperatingTable } from './operatingTable.js';
import { createCArmPreviewFramer } from './carmPreviewFraming.js';
import { disposeThreeResources } from '../disposeThreeResources.js';
import { startupDiagnostics } from '../startupDiagnostics.js';
import {createManagedWebGLRenderer} from '../graphicsLifetime.js';

let previewScene;
let previewCamera;
let previewRenderer;
let previewResizeObserver;
let previewGuard;
let fitPreviewCamera;
let cArmGroup;
let cArmGantry;
let cArmLift;
let cArmTable;
let cArmDetectorAssembly;
const PREVIEW_CAMERA_TARGET = new THREE.Vector3(0, 24, -30);

export function initCArmPreview({runtime} = {}) {
    const container = document.getElementById('carm-preview');
    if (!container) return null;
    disposeCArmPreview();
    container.replaceChildren();
    // The title has its own strip, so fitting the scene never puts equipment
    // behind the "C-ARM POSITION" heading.
    const viewport = document.createElement('div');
    viewport.className = 'carm-preview-viewport';
    container.appendChild(viewport);

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

    const width = Math.max(1, viewport.clientWidth);
    const height = Math.max(1, viewport.clientHeight);

    previewCamera = new THREE.PerspectiveCamera(39, width / height, 0.1, 1000);
    previewCamera.position.set(268, 146, 289);
    previewCamera.lookAt(PREVIEW_CAMERA_TARGET);
    previewScene.add(previewCamera);

    startupDiagnostics.mark('Creating C-arm preview graphics context');
    const canvas = document.createElement('canvas');
    const showUnavailable = () => {
        disposeCArmPreview();
        const message = document.createElement('p');
        message.textContent = 'Podgląd ramienia C niedostępny — błąd grafiki.';
        container.replaceChildren(message);
    };
    try {
        const managed = createManagedWebGLRenderer(
            () => new THREE.WebGLRenderer({canvas, antialias: true, alpha: true}),
            {canvas, label: 'c-arm-preview', runtime, onFailure: showUnavailable}
        );
        previewRenderer = managed.renderer;
        previewGuard = managed.guard;
    } catch (error) {
        // The small secondary view must not retain a partially created context
        // or prevent a healthy main fluoroscopy renderer from starting.
        showUnavailable();
        console.warn('C-arm preview unavailable', error);
        return null;
    }
    runtime?.onDispose(disposeCArmPreview);
    previewRenderer.setSize(width, height, false);
    previewRenderer.setPixelRatio(Math.min(window.devicePixelRatio || 1, 2));
    viewport.appendChild(previewRenderer.domElement);
    previewResizeObserver = new ResizeObserver(() => {
        if (!previewRenderer || !previewGuard.check()) return;
        const width = Math.max(1, viewport.clientWidth);
        const height = Math.max(1, viewport.clientHeight);
        previewCamera.aspect = width / height;
        previewRenderer.setSize(width, height, false);
        renderCArmPreview();
    });
    previewResizeObserver.observe(viewport);

    startupDiagnostics.mark('Building C-arm preview geometry');
    cArmTable = createOperatingTable();
    previewScene.add(cArmTable);

    cArmGroup = new THREE.Group();
    const { group: cArm, gantryGroup, liftGroup, detectorAssembly } = createCArmModel();
    cArmGantry = gantryGroup;
    cArmLift = liftGroup;
    cArmDetectorAssembly = detectorAssembly;
    cArmGroup.add(cArm);
    previewScene.add(cArmGroup);
    fitPreviewCamera = createCArmPreviewFramer(previewCamera, [cArmGroup, cArmTable]);

    // Render once so the preview displays immediately.
    startupDiagnostics.mark('Rendering first C-arm preview');
    renderCArmPreview();
    startupDiagnostics.mark('C-arm preview ready');
    return {
        group: cArmGroup,
        gantry: cArmGantry,
        detectorAssembly: cArmDetectorAssembly,
        lift: cArmLift,
        table: cArmTable
    };
}

export function disposeCArmPreview() {
    previewResizeObserver?.disconnect();
    previewResizeObserver = null;
    fitPreviewCamera = null;
    previewGuard?.dispose();
    previewGuard = null;
    disposeThreeResources({ roots: [previewScene] });
    if (previewRenderer) {
        previewRenderer.dispose();
        previewRenderer.forceContextLoss();
        previewRenderer.domElement.remove();
    }
    previewRenderer = previewScene = previewCamera = null;
    cArmGroup = cArmGantry = cArmLift = cArmTable = cArmDetectorAssembly = null;
}

export function renderCArmPreview() {
    if (!previewRenderer || !previewScene || !previewCamera || !previewGuard.check()) return;
    fitPreviewCamera?.();
    previewRenderer.render(previewScene, previewCamera);
}

export {
    cArmGroup as cArmPreviewGroup,
    cArmGantry as cArmPreviewGantry,
    cArmDetectorAssembly as cArmPreviewDetectorAssembly,
    cArmTable as cArmPreviewTable
};
