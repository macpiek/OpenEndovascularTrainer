import * as THREE from 'three';

// Utility to build a simple voxel grid that fills cylindrical vessel segments.
// Each segment is discretised into a 3D array of cubic voxels.  Voxels outside
// the circular cross section are stored as null so we can reuse indexing across
// the entire segment.
export function voxelizeVessel(vessel, voxelSize = 5) {
    const segments = [];
    for (let s = 0; s < vessel.segments.length; s++) {
        const seg = vessel.segments[s];
        const start = new THREE.Vector3(seg.start.x, seg.start.y, seg.start.z);
        const end = new THREE.Vector3(seg.end.x, seg.end.y, seg.end.z);
        const dir = new THREE.Vector3().subVectors(end, start);
        const length = dir.length();
        if (!length) continue;
        dir.normalize();

        // Build an orthonormal basis for the cross section
        const up = Math.abs(dir.y) < 0.99 ? new THREE.Vector3(0, 1, 0) : new THREE.Vector3(1, 0, 0);
        const u = new THREE.Vector3().crossVectors(dir, up).normalize();
        const v = new THREE.Vector3().crossVectors(dir, u).normalize();

        const steps = Math.ceil(length / voxelSize);
        const radialSteps = Math.ceil(seg.radius / voxelSize);
        const width = radialSteps * 2 + 1;
        const voxels = Array.from({ length: steps }, () =>
            Array.from({ length: width }, () => new Array(width).fill(null)));

        for (let i = 0; i < steps; i++) {
            const center = start.clone().addScaledVector(dir, (i + 0.5) * voxelSize);
            for (let y = -radialSteps; y <= radialSteps; y++) {
                for (let x = -radialSteps; x <= radialSteps; x++) {
                    const px = x * voxelSize;
                    const py = y * voxelSize;
                    if (px * px + py * py <= seg.radius * seg.radius) {
                        const pos = center.clone()
                            .addScaledVector(u, px)
                            .addScaledVector(v, py);
                        voxels[i][y + radialSteps][x + radialSteps] = {
                            position: pos,
                            concentration: 0
                        };
                    }
                }
            }
        }
        segments.push({
            voxels,
            width,
            steps,
            flowSpeed: seg.flowSpeed || 0,
            voxelSize
        });
    }
    return { segments, voxelSize };
}

// Contrast transport on the voxelised vessel. Advection is handled along the
// axial direction of each segment while diffusion is applied in all three axes
// to spread concentration across the volume.
export class VoxelContrastAgent {
    constructor(vessel, voxelSize = 5, diffusion = 0.1) {
        this.grid = voxelizeVessel(vessel, voxelSize);
        this.diffusion = diffusion;
    }

    inject(volumeMl, segmentIndex = 0, atEnd = false) {
        const seg = this.grid.segments[segmentIndex];
        if (!seg) return;
        const slice = atEnd ? seg.voxels[seg.voxels.length - 1] : seg.voxels[0];
        // convert ml to mm^3 and distribute evenly
        const totalVoxels = slice.flat().filter(v => v).length;
        const volume = (volumeMl * 1000) / Math.max(1, totalVoxels);
        for (const row of slice) {
            for (const vox of row) {
                if (vox) vox.concentration += volume;
            }
        }
    }

    update(dt) {
        for (const seg of this.grid.segments) {
            const voxels = seg.voxels;
            const width = seg.width;
            const steps = seg.steps;
            const voxelSize = seg.voxelSize;
            const dist = seg.flowSpeed * dt;
            const frac = Math.min(1, Math.abs(dist) / voxelSize);
            const dir = Math.sign(dist) || 1;

            // advection along the axis
            for (let i = dir > 0 ? steps - 1 : 0;
                 dir > 0 ? i >= 0 : i < steps;
                 i -= dir) {
                const slice = voxels[i];
                for (let y = 0; y < width; y++) {
                    for (let x = 0; x < width; x++) {
                        const vox = slice[y][x];
                        if (!vox) continue;
                        const moved = vox.concentration * frac;
                        vox.concentration -= moved;
                        const nextIdx = i + dir;
                        if (nextIdx >= 0 && nextIdx < steps) {
                            const dest = voxels[nextIdx][y][x];
                            if (dest) dest.concentration += moved;
                        }
                    }
                }
            }

            // diffusion across all axes
            if (this.diffusion > 0) {
                const nextConc = voxels.map(slice =>
                    slice.map(row => row.map(v => (v ? v.concentration : 0))));
                for (let i = 0; i < steps; i++) {
                    for (let y = 0; y < width; y++) {
                        for (let x = 0; x < width; x++) {
                            const vox = voxels[i][y][x];
                            if (!vox) continue;
                            let sum = vox.concentration;
                            let count = 1;
                            const add = (nx, ny, ni) => {
                                const s = voxels[ni];
                                if (!s) return;
                                const n = s[ny] && s[ny][nx];
                                if (n) {
                                    sum += n.concentration;
                                    count++;
                                }
                            };
                            add(x + 1, y, i);
                            add(x - 1, y, i);
                            add(x, y + 1, i);
                            add(x, y - 1, i);
                            add(x, y, i + 1);
                            add(x, y, i - 1);
                            nextConc[i][y][x] = vox.concentration + this.diffusion * (sum / count - vox.concentration);
                        }
                    }
                }
                // write back
                for (let i = 0; i < steps; i++) {
                    for (let y = 0; y < width; y++) {
                        for (let x = 0; x < width; x++) {
                            const vox = voxels[i][y][x];
                            if (vox) vox.concentration = nextConc[i][y][x];
                        }
                    }
                }
            }
        }
    }

    // Convenience to gather concentrations per segment for logging/visualisation
    getSegmentConcentrations(segmentIndex) {
        const seg = this.grid.segments[segmentIndex];
        if (!seg) return [];
        return seg.voxels.map(slice => {
            let total = 0;
            let count = 0;
            for (const row of slice) {
                for (const vox of row) {
                    if (vox) {
                        total += vox.concentration;
                        count++;
                    }
                }
            }
            const voxelVol = seg.voxelSize ** 3;
            return count ? total / (count * voxelVol) : 0;
        });
    }
}

// Generate a Three.js mesh for every voxel above a minimum concentration.
// Pass `wireframe = true` to render the voxels as wireframe boxes instead of
// solid cubes.
export function getVoxelMeshes(agent, minConc = 1e-4, wireframe = false) {
    if (!agent || !agent.grid) return [];
    const size = agent.grid.voxelSize;
    const geometry = new THREE.BoxGeometry(size, size, size);
    const meshes = [];
    for (const seg of agent.grid.segments) {
        const width = seg.width;
        const steps = seg.steps;
        for (let i = 0; i < steps; i++) {
            for (let y = 0; y < width; y++) {
                for (let x = 0; x < width; x++) {
                    const vox = seg.voxels[i][y][x];
                    if (!vox || vox.concentration <= minConc) continue;
                    const material = new THREE.MeshBasicMaterial({ wireframe });
                    const color = new THREE.Color(
                        vox.concentration,
                        0,
                        1 - vox.concentration
                    );
                    material.color.copy(color);
                    const mesh = new THREE.Mesh(geometry, material);
                    mesh.position.copy(vox.position);
                    meshes.push(mesh);
                }
            }
        }
    }
    return meshes;
}

