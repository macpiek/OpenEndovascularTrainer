import { defineConfig } from 'vite';

export default defineConfig({
    optimizeDeps: {
        include: [
            'three',
            'three-mesh-bvh',
            'three/examples/jsm/loaders/OBJLoader.js',
            'three/examples/jsm/loaders/STLLoader.js'
        ]
    }
});
