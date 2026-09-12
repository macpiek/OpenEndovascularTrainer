import { OBJLoader } from 'three/examples/jsm/loaders/OBJLoader.js';
import { packSkeletonGeometry, skeletonTransferBuffers } from '../skeletonGeometry.js';

self.onmessage = async ({ data: { url } }) => {
    try {
        const response = await fetch(url);
        if (!response.ok) throw new Error(`Skeleton HTTP ${response.status}`);
        const object = new OBJLoader().parse(await response.text());
        const meshes = packSkeletonGeometry(object);
        self.postMessage({ meshes }, skeletonTransferBuffers(meshes));
    } catch (error) {
        self.postMessage({ error: error.message || String(error) });
    } finally {
        // The OBJ text, split lines and expandable parser arrays belong only
        // to this short-lived worker. The main page receives the final buffers.
        self.close();
    }
};
