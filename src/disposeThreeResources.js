import { releaseOwnedBuffers } from './releaseOwnedBuffers.js';

// Final destruction only: after disposal release CPU buffers as well as GPU
// objects, even if a diagnostic reference temporarily retains an old mesh.
export function disposeThreeResources({ roots = [], materials = [], textures = [], targets = [], buffers = [] }) {
    const geometries = new Set();
    const ownedMaterials = new Set(materials);
    const ownedTextures = new Set(textures);
    const ownedBuffers = [...buffers];
    for (const root of roots) root?.traverse(object => {
        if (object.geometry) geometries.add(object.geometry);
        for (const material of Array.isArray(object.material) ? object.material : [object.material]) {
            if (material) ownedMaterials.add(material);
        }
    });
    function collectTexture(value) {
        if (value?.isTexture) ownedTextures.add(value);
        else if (Array.isArray(value)) value.forEach(collectTexture);
    }
    for (const material of ownedMaterials) {
        Object.values(material).forEach(collectTexture);
        for (const uniform of Object.values(material.uniforms ?? {})) collectTexture(uniform.value);
    }
    for (const target of new Set(targets)) {
        target.dispose();
        collectTexture(target.texture);
        collectTexture(target.depthTexture);
    }
    for (const geometry of geometries) {
        geometry.dispose();
        for (const [name, attribute] of Object.entries(geometry.attributes)) {
            ownedBuffers.push(attribute.array ?? attribute.data?.array);
            geometry.deleteAttribute(name);
        }
        ownedBuffers.push(geometry.index?.array);
        for (const attributes of Object.values(geometry.morphAttributes)) {
            for (const attribute of attributes) ownedBuffers.push(attribute.array);
        }
        geometry.setIndex(null);
        geometry.morphAttributes = {};
        geometry.boundsTree = null;
    }
    for (const material of ownedMaterials) {
        material.dispose();
        for (const name of Object.keys(material)) {
            if (material[name]?.isTexture) material[name] = null;
        }
        if (material.uniforms) material.uniforms = {};
    }
    for (const texture of ownedTextures) {
        if (!texture) continue;
        texture.dispose();
        if (texture.source) {
            ownedBuffers.push(texture.source.data?.data);
            texture.source.data = null;
        }
    }
    for (const root of roots) root?.clear();
    releaseOwnedBuffers(ownedBuffers);
}
