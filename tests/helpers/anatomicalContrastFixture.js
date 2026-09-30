import * as THREE from 'three';
import { ContrastFlowNetwork } from '../../src/contrast/flowNetwork.js';

// An asymmetric rectangular lumen, deliberately much wider than the 1 mm
// graph radius. Its projected thickness is constant, giving an analytic
// optical-depth reference independent of rendering implementation.
export function anatomicalContrastFixture({ sparse = false, wallThickness = 0 } = {}) {
    const halfDepth = sparse ? 12 : 3;
    const distance = (x, y, z) =>
        Math.min(x + 6, 10 - x, 14 - Math.abs(y), halfDepth - Math.abs(z));
    const dims = [8, 10, 8],
        size = 8,
        origin = [-16, -20, -16],
        voxel = 0.5;
    const keys = [],
        distances = [],
        bits = [];
    const lookup = new Int32Array(dims[0] * dims[1] * dims[2]).fill(-1);
    for (let bz = 0; bz < dims[2]; bz++)
        for (let by = 0; by < dims[1]; by++)
            for (let bx = 0; bx < dims[0]; bx++) {
                // Omit a deep interior column, as a narrow-band collision asset does.
                if (
                    sparse &&
                    bx === 4 &&
                    by >= 3 &&
                    by <= 6 &&
                    bz >= 2 &&
                    bz <= 5
                )
                    continue;
                const key = bx + dims[0] * (by + dims[1] * bz),
                    brick = keys.length;
                keys.push(key);
                lookup[key] = brick;
                for (let z = 0; z < size; z++)
                    for (let y = 0; y < size; y++)
                        for (let x = 0; x < size; x++) {
                            const d = distance(
                                origin[0] + (bx * size + x) * voxel,
                                origin[1] + (by * size + y) * voxel,
                                origin[2] + (bz * size + z) * voxel
                            );
                            const i = distances.length;
                            // Collision distances include the outer wall too,
                            // although only the inner contour contains blood.
                            const unsignedDistance = wallThickness > 0 && d < 0
                                ? Math.min(Math.abs(d), Math.abs(d + wallThickness))
                                : Math.abs(d);
                            distances.push(
                                Math.min(255, Math.round(unsignedDistance / 0.02))
                            );
                            bits[i >> 3] =
                                (bits[i >> 3] || 0) |
                                (d >= 0 ? 1 << (i & 7) : 0);
                        }
            }
    const field = {
        brickSize: size,
        sdfDimensions: dims,
        sdfOrigin: origin,
        voxelSize: voxel,
        sdfQuantization: 0.02,
        sdfMissingBrick: -1,
        sdfBrickLookup: lookup,
        sdfBrickKeys: Uint32Array.from(keys),
        sdfDistances: Uint8Array.from(distances),
        sdfInsideBits: Uint8Array.from(bits),
        packedLumenField: {
            isInsideCoordinates: (x, y, z) => distance(x, y, z) >= 0
        }
    };
    const network = new ContrastFlowNetwork([
        {
            id: 0,
            start: new THREE.Vector3(0, 16, 0),
            end: new THREE.Vector3(0, -16, 0),
            radiusStart: 1,
            radiusEnd: 1,
            safeRadius: 1,
            nodeStartId: 0,
            nodeEndId: 1
        }
    ]);
    let count = 0;
    const offsets = Uint32Array.from(network.edges, (e) => {
        const offset = count;
        count += e.cellCount;
        return offset;
    });
    const plume = new Float32Array(count);
    function fill(concentration = 0.03) {
        for (const e of network.edges)
            for (let i = 0; i < e.cellCount; i++)
                e.massMg[i] = e.volumes[i] * concentration;
    }
    fill();
    return { field, network, offsets, plume, fill };
}
