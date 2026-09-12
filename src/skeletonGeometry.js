import { BufferAttribute, BufferGeometry, Group, Mesh } from 'three';

// OBJLoader returns flat mesh children. Keep typed arrays as typed arrays:
// Object3D.toJSON() would turn millions of vertices into copied JS numbers.
export function packSkeletonGeometry(object) {
    return object.children.map(child => {
        if (!child.isMesh || child.children.length) {
            throw new Error('The skeleton asset must contain flat meshes');
        }
        const geometry = child.geometry;
        const attributes = {};
        for (const [name, attribute] of Object.entries(geometry.attributes)) {
            attributes[name] = {
                array: attribute.array,
                itemSize: attribute.itemSize,
                normalized: attribute.normalized
            };
        }
        child.updateMatrix();
        return {
            name: child.name, attributes,
            index: geometry.index?.array ?? null,
            groups: geometry.groups,
            matrix: child.matrix.toArray()
        };
    });
}

export function skeletonTransferBuffers(meshes) {
    const buffers = new Set();
    for (const mesh of meshes) {
        for (const attribute of Object.values(mesh.attributes)) buffers.add(attribute.array.buffer);
        if (mesh.index) buffers.add(mesh.index.buffer);
    }
    return [...buffers];
}

export function unpackSkeletonGeometry(meshes, material) {
    const object = new Group();
    for (const mesh of meshes) {
        const geometry = new BufferGeometry();
        for (const [name, attribute] of Object.entries(mesh.attributes)) {
            geometry.setAttribute(name, new BufferAttribute(
                attribute.array, attribute.itemSize, attribute.normalized
            ));
        }
        if (mesh.index) geometry.setIndex(new BufferAttribute(mesh.index, 1));
        geometry.groups = mesh.groups;
        const child = new Mesh(geometry, material);
        child.name = mesh.name;
        child.matrix.fromArray(mesh.matrix);
        child.matrix.decompose(child.position, child.quaternion, child.scale);
        object.add(child);
    }
    return object;
}
