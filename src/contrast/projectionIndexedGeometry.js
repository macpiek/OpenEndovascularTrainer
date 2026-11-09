import {BufferAttribute, BufferGeometry} from 'three';

const canonicalBits = bits => bits === 0x80000000 ? 0 : bits;

function positionHash(x, y, z) {
    let hash = Math.imul(x, 0x9e3779b1) ^ Math.imul(y, 0x85ebca77) ^ Math.imul(z, 0xc2b2ae3d);
    hash = Math.imul(hash ^ (hash >>> 16), 0x7feb352d);
    hash = Math.imul(hash ^ (hash >>> 15), 0x846ca68b);
    return (hash ^ (hash >>> 16)) >>> 0;
}

/**
 * Make the position-only, indexed copy used by the anatomical projection BVH.
 * Welding is exact in Float32 coordinates, with +0 and -0 treated equally.
 * Neither the source buffers nor its triangle order, groups or attributes are
 * modified. Distinct coordinates are never rounded onto a spatial grid.
 */
export function createProjectionIndexedGeometry(source) {
    const position = source.getAttribute('position');
    const array = position?.isInterleavedBufferAttribute ? position.data.array : position?.array;
    if (!position || position.itemSize !== 3 || !(array instanceof Float32Array))
        throw new TypeError('Projection indexing requires a three-component Float32 position attribute.');
    const stride = position.isInterleavedBufferAttribute ? position.data.stride : 3;
    const offset = position.isInterleavedBufferAttribute ? position.offset : 0;
    const bits = new Uint32Array(array.buffer, array.byteOffset, array.length);
    const vertexCount = position.count;
    // This array becomes the result's index for the usual non-indexed STL.
    // Each hash entry stores its first source vertex; its unique ID is read
    // from this same array, avoiding a second whole-mesh ownership buffer.
    const vertexIds = new Uint32Array(vertexCount);
    let capacity = 16;
    const estimate = Math.min(1 << 20, Math.ceil(vertexCount / 4));
    while (capacity < estimate) capacity *= 2;
    let table = new Uint32Array(capacity), mask = capacity - 1, uniqueCount = 0;

    const grow = () => {
        const previous = table;
        capacity *= 2;
        table = new Uint32Array(capacity);
        mask = capacity - 1;
        for (let entry = 0; entry < previous.length; entry++) {
            const stored = previous[entry];
            if (!stored) continue;
            const at = (stored - 1) * stride + offset;
            let slot = positionHash(canonicalBits(bits[at]), canonicalBits(bits[at + 1]), canonicalBits(bits[at + 2])) & mask;
            while (table[slot]) slot = (slot + 1) & mask;
            table[slot] = stored;
        }
    };

    for (let vertex = 0; vertex < vertexCount; vertex++) {
        const at = vertex * stride + offset;
        const x = canonicalBits(bits[at]), y = canonicalBits(bits[at + 1]), z = canonicalBits(bits[at + 2]);
        let slot = positionHash(x, y, z) & mask;
        while (table[slot]) {
            const previous = (table[slot] - 1) * stride + offset;
            if (canonicalBits(bits[previous]) === x && canonicalBits(bits[previous + 1]) === y &&
                canonicalBits(bits[previous + 2]) === z) break;
            slot = (slot + 1) & mask;
        }
        if (table[slot]) {
            vertexIds[vertex] = vertexIds[table[slot] - 1];
            continue;
        }
        if ((uniqueCount + 1) * 10 > capacity * 7) {
            grow();
            slot = positionHash(x, y, z) & mask;
            while (table[slot]) slot = (slot + 1) & mask;
        }
        table[slot] = vertex + 1;
        vertexIds[vertex] = uniqueCount++;
    }

    const positions = new Float32Array(uniqueCount * 3);
    const outputBits = new Uint32Array(positions.buffer);
    // Copy bits rather than converting through Number, preserving even the
    // exact stored Float32 payload. Repeated writes need no extra unique list.
    for (let vertex = 0; vertex < vertexCount; vertex++) {
        const from = vertex * stride + offset, to = vertexIds[vertex] * 3;
        outputBits[to] = canonicalBits(bits[from]);
        outputBits[to + 1] = canonicalBits(bits[from + 1]);
        outputBits[to + 2] = canonicalBits(bits[from + 2]);
    }
    let indices = vertexIds;
    if (source.index) {
        indices = new Uint32Array(source.index.count);
        for (let corner = 0; corner < indices.length; corner++) {
            const vertex = source.index.getX(corner);
            if (vertex < 0 || vertex >= vertexCount || !Number.isInteger(vertex))
                throw new RangeError('Projection geometry contains an invalid vertex index.');
            indices[corner] = vertexIds[vertex];
        }
    }
    const geometry = new BufferGeometry();
    geometry.setAttribute('position', new BufferAttribute(positions, 3));
    geometry.setIndex(new BufferAttribute(indices, 1));
    for (const group of source.groups) geometry.addGroup(group.start, group.count, group.materialIndex);
    geometry.setDrawRange(source.drawRange.start, source.drawRange.count);
    geometry.boundingBox = source.boundingBox?.clone() ?? null;
    geometry.boundingSphere = source.boundingSphere?.clone() ?? null;
    return geometry;
}
