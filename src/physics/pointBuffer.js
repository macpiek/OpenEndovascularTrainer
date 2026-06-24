export function ensurePointBuffer(buffer, count) {
    if (!buffer || buffer.length !== count) {
        return Array.from({ length: count }, () => ({ x: 0, y: 0, z: 0, active: false }));
    }
    return buffer;
}

export function snapshotNodePositions(nodes, buffer) {
    const positions = ensurePointBuffer(buffer, nodes.length);
    const storage = nodes.nodeStorage;
    if (storage) {
        const { x, y, z } = storage;
        for (let i = 0; i < nodes.length; i++) {
            const position = positions[i];
            position.x = x[i];
            position.y = y[i];
            position.z = z[i];
            position.active = true;
        }
        return positions;
    }
    for (let i = 0; i < nodes.length; i++) {
        const node = nodes[i];
        const position = positions[i];
        position.x = node.x;
        position.y = node.y;
        position.z = node.z;
        position.active = true;
    }
    return positions;
}

export function clearPointBuffer(buffer) {
    for (let i = 0; i < buffer.length; i++) {
        const point = buffer[i];
        point.x = 0;
        point.y = 0;
        point.z = 0;
        point.active = false;
    }
}

export function addPointCorrection(buffer, index, x, y, z) {
    const correction = buffer[index];
    correction.x += x;
    correction.y += y;
    correction.z += z;
    correction.active = true;
}
