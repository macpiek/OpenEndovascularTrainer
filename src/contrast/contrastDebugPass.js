import * as THREE from 'three';

// MAX joins overlapping lumen surfaces; ADD integrates independent volumes.
// Both must operate on a black signal buffer. Applying MAX to the coloured
// anatomy/background while ADD includes it produces bands at material borders.
export class ContrastDebugPass {
    constructor(root) {
        this.root = root;
        this.signalScene = new THREE.Scene();
        this.signalScene.matrixAutoUpdate = false;
        this.target = new THREE.WebGLRenderTarget(1, 1, {
            depthBuffer: false,
            // Keep weak subcell contributions until the final composite.
            // Rounding each additive sample to 8 bits can erase dilute flow.
            type: THREE.HalfFloatType
        });
        this.camera = new THREE.OrthographicCamera(-1, 1, 1, -1, 0, 1);
        this.composite = new THREE.Scene();
        this.quad = new THREE.Mesh(
            new THREE.PlaneGeometry(2, 2),
            new THREE.ShaderMaterial({
                uniforms: { signal: { value: this.target.texture } },
                vertexShader: `varying vec2 uvSignal; void main() {
                uvSignal = uv; gl_Position = vec4(position.xy, 0., 1.);
            }`,
                fragmentShader: `uniform sampler2D signal; varying vec2 uvSignal;
                void main() { gl_FragColor = texture2D(signal, uvSignal); }`,
                transparent: true,
                premultipliedAlpha: true,
                blending: THREE.AdditiveBlending,
                depthTest: false,
                depthWrite: false,
                toneMapped: false
            })
        );
        this.quad.frustumCulled = false;
        this.composite.add(this.quad);
        this.size = new THREE.Vector2();
        this.clearColor = new THREE.Color();
        this.viewport = new THREE.Vector4();
        this.scissor = new THREE.Vector4();
    }

    render(renderer, scene, camera) {
        const root = this.root,
            parent = root.parent;
        if (!root.visible || !parent) {
            renderer.render(scene, camera);
            return;
        }
        const output = renderer.getRenderTarget();
        const autoClear = renderer.autoClear,
            clearAlpha = renderer.getClearAlpha();
        const scissorTest = renderer.getScissorTest();
        renderer.getClearColor(this.clearColor);
        renderer.getViewport(this.viewport);
        renderer.getScissor(this.scissor);
        if (output) this.size.set(output.width, output.height);
        else renderer.getDrawingBufferSize(this.size);
        this.target.setSize(this.size.x, this.size.y);
        this.target.samples = Math.min(4, renderer.capabilities.maxSamples || 0);
        const childIndex = parent.children.indexOf(root);
        let moved = false;
        try {
            root.visible = false;
            renderer.render(scene, camera);
            root.visible = true;
            parent.updateWorldMatrix(true, false);
            this.signalScene.matrix.copy(parent.matrixWorld);
            this.signalScene.matrixWorldNeedsUpdate = true;
            this.signalScene.add(root);
            moved = true;
            renderer.setRenderTarget(this.target);
            renderer.setScissorTest(false);
            renderer.setClearColor(0x000000, 0);
            renderer.clear(true, false, false);
            renderer.autoClear = false;
            renderer.render(this.signalScene, camera);
            renderer.setRenderTarget(output);
            renderer.setViewport(this.viewport);
            renderer.setScissor(this.scissor);
            renderer.setScissorTest(scissorTest);
            renderer.render(this.composite, this.camera);
        } finally {
            if (moved) {
                parent.add(root);
                parent.children.splice(parent.children.indexOf(root), 1);
                parent.children.splice(childIndex, 0, root);
            }
            root.visible = true;
            renderer.autoClear = autoClear;
            renderer.setRenderTarget(output);
            renderer.setViewport(this.viewport);
            renderer.setScissor(this.scissor);
            renderer.setScissorTest(scissorTest);
            renderer.setClearColor(this.clearColor, clearAlpha);
        }
    }

    dispose() {
        this.target.dispose();
        this.quad.geometry.dispose();
        this.quad.material.dispose();
    }
}
