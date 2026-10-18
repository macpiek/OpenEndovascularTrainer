// The collision asset's signed samples come from the anatomical lumen, not
// from flow-graph radii. Keep its native resolution and share border samples.
export function buildAnatomicalVolumeAtlas(field) {
    const size = field.brickSize,
        dims = field.sdfDimensions;
    const lookup = field.sdfBrickLookup,
        missing = field.sdfMissingBrick;
    const source = field.sdfDistances,
        inside = field.sdfInsideBits;
    const count = size ** 3,
        tile = size + 2;
    const keyOf = (x, y, z) => x + dims[0] * (y + dims[1] * z);
    const interior = new Set();
    function signed(x, y, z) {
        const bx = Math.floor(x / size),
            by = Math.floor(y / size),
            bz = Math.floor(z / size);
        if (
            bx < 0 ||
            by < 0 ||
            bz < 0 ||
            bx >= dims[0] ||
            by >= dims[1] ||
            bz >= dims[2]
        )
            return -5;
        const brick = lookup[keyOf(bx, by, bz)];
        if (brick === missing) return interior.has(keyOf(bx, by, bz)) ? 5 : -5;
        const index =
            brick * count +
            (x - bx * size) +
            size * (y - by * size + size * (z - bz * size));
        return (
            source[index] *
            field.sdfQuantization *
            (inside[index >> 3] & (1 << (index & 7)) ? 1 : -1)
        );
    }
    const occupiedKeys = [];
    for (let i = 0; i < field.sdfBrickKeys.length; i++) {
        for (let j = (i * count) / 8; j < ((i + 1) * count) / 8; j++) {
            if (!inside[j]) continue;
            occupiedKeys.push(field.sdfBrickKeys[i]);
            break;
        }
    }
    const step = size * field.voxelSize;
    // The SDF is a narrow band around walls. Missing bricks are unknown,
    // not empty: recover enclosed blood volumes using anatomical contours.
    // Only visit missing neighbours of known interior, avoiding a dense scan.
    const checked = new Set();
    const queue = occupiedKeys.slice();
    if (field.packedLumenField) {
        const neighbours = [
            [-1, 0, 0],
            [1, 0, 0],
            [0, -1, 0],
            [0, 1, 0],
            [0, 0, -1],
            [0, 0, 1]
        ];
        for (let cursor = 0; cursor < queue.length; cursor++) {
            const key = queue[cursor];
            const bx = key % dims[0],
                by = Math.floor(key / dims[0]) % dims[1],
                bz = Math.floor(key / (dims[0] * dims[1]));
            for (const [dx, dy, dz] of neighbours) {
                const x = bx + dx,
                    y = by + dy,
                    z = bz + dz;
                if (
                    x < 0 ||
                    y < 0 ||
                    z < 0 ||
                    x >= dims[0] ||
                    y >= dims[1] ||
                    z >= dims[2]
                )
                    continue;
                const neighbour = keyOf(x, y, z);
                if (lookup[neighbour] !== missing || checked.has(neighbour))
                    continue;
                checked.add(neighbour);
                if (
                    !field.packedLumenField.isInsideCoordinates(
                        field.sdfOrigin[0] + (x + 0.5) * step,
                        field.sdfOrigin[1] + (y + 0.5) * step,
                        field.sdfOrigin[2] + (z + 0.5) * step
                    )
                )
                    continue;
                interior.add(neighbour);
                queue.push(neighbour);
            }
        }
    }
    const candidates = new Set();
    for (const key of queue) {
        const x = key % dims[0],
            y = Math.floor(key / dims[0]) % dims[1],
            z = Math.floor(key / (dims[0] * dims[1]));
        // An inside sample on a brick's lower face also bounds the cell in
        // the preceding brick. Retain those neighbours to avoid seams.
        for (let dz = -1; dz <= 0; dz++)
            for (let dy = -1; dy <= 0; dy++)
                for (let dx = -1; dx <= 0; dx++)
                    if (x + dx >= 0 && y + dy >= 0 && z + dz >= 0)
                        candidates.add(keyOf(x + dx, y + dy, z + dz));
    }
    const keys = Uint32Array.from(candidates);
    const side = Math.ceil(Math.cbrt(keys.length)),
        layers = Math.ceil(keys.length / (side * side));
    const atlasDims = [side * tile, side * tile, layers * tile];
    const quantization = Math.max(field.sdfQuantization * 2, 0.04);
    const data = new Uint8Array(atlasDims[0] * atlasDims[1] * atlasDims[2]);
    const brickOrigins = new Float32Array(keys.length * 3),
        atlasOrigins = brickOrigins.slice();
    const vertexDims = dims.map((d) => d + 1),
        vertexCount = vertexDims.reduce((a, b) => a * b, 1);
    const vertexMarks = new Uint8Array(vertexCount),
        brickVertices = new Uint32Array(keys.length * 8);
    for (let i = 0; i < keys.length; i++) {
        const key = keys[i],
            bx = key % dims[0],
            by = Math.floor(key / dims[0]) % dims[1],
            bz = Math.floor(key / (dims[0] * dims[1]));
        const ax = (i % side) * tile,
            ay = (Math.floor(i / side) % side) * tile,
            az = Math.floor(i / (side * side)) * tile;
        brickOrigins.set(
            [
                field.sdfOrigin[0] + bx * step,
                field.sdfOrigin[1] + by * step,
                field.sdfOrigin[2] + bz * step
            ],
            i * 3
        );
        atlasOrigins.set([ax, ay, az], i * 3);
        for (let z = 0; z < tile; z++)
            for (let y = 0; y < tile; y++)
                for (let x = 0; x < tile; x++) {
                    const d = signed(
                        bx * size + x - 1,
                        by * size + y - 1,
                        bz * size + z - 1
                    );
                    data[
                        ax +
                            x +
                            atlasDims[0] * (ay + y + atlasDims[1] * (az + z))
                    ] = Math.max(
                        1,
                        Math.min(255, 128 + Math.round(d / quantization))
                    );
                }
        for (let z = 0; z < 2; z++)
            for (let y = 0; y < 2; y++)
                for (let x = 0; x < 2; x++) {
                    const v =
                        bx +
                        x +
                        vertexDims[0] * (by + y + vertexDims[1] * (bz + z));
                    brickVertices[i * 8 + x + 2 * y + 4 * z] = v;
                    vertexMarks[v] = 1;
                }
    }
    const vertices = [],
        compact = new Uint32Array(vertexCount);
    for (let i = 0; i < vertexMarks.length; i++)
        if (vertexMarks[i]) {
            compact[i] = vertices.length;
            vertices.push(i);
        }
    for (let i = 0; i < brickVertices.length; i++)
        brickVertices[i] = compact[brickVertices[i]];
    return {
        data,
        atlasDims,
        tileSize: tile,
        concentrationDims: [side * 2, side * 2, layers * 2],
        brickOrigins,
        atlasOrigins,
        brickVertices,
        vertexDims,
        vertices: Uint32Array.from(vertices),
        step,
        voxelSize: field.voxelSize,
        origin: field.sdfOrigin,
        quantization,
        brickCount: keys.length
    };
}
