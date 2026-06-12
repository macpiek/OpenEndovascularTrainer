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
            voxelSize,
            segmentIndex: s,
            sourceSegment: seg,
            start,
            end,
            dir,
            radius: seg.radius,
            isSheath: !!seg.isSheath
        });
    }
    return { segments, voxelSize };
}

// Contrast transport on the voxelised vessel. Advection is handled along the
// axial direction of each segment while diffusion is applied in all three axes
// to spread concentration across the volume.
export class VoxelContrastAgent {
    constructor(vessel, voxelSize = 5, diffusion = 0.1) {
        this.vessel = vessel;
        this.grid = voxelizeVessel(vessel, voxelSize);
        this.diffusion = diffusion;
        this.washoutRate = 0.42;
        this.sheathSegmentIndex = vessel.segments.findIndex(seg => seg.isSheath);
        this.segmentGraph = vessel.segmentGraph || vessel.segments.map(() => []);
    }

    inject(volumeMl, segmentIndex = 0, atEnd = false) {
        const seg = this.grid.segments[segmentIndex];
        if (!seg) return;
        this.#injectIntoSlice(seg, atEnd ? seg.voxels.length - 1 : 0, volumeMl);
    }

    injectThroughSheath(volumeMl) {
        if (volumeMl <= 0) return;
        if (this.sheathSegmentIndex >= 0) {
            this.inject(volumeMl * 0.08, this.sheathSegmentIndex, true);
        }
        const tip = this.vessel.sheath?.end;
        if (tip) {
            const target = this.#findNearestSegmentSlice(tip, { excludeSheath: true });
            if (target) {
                this.#injectRetrogradeJet(target, volumeMl * 0.92);
            }
        } else {
            this.inject(volumeMl, 0, false);
        }
    }

    injectAtPoint(point, volumeMl, { excludeSheath = false, spreadSlices = 1 } = {}) {
        const target = this.#findNearestSegmentSlice(point, { excludeSheath });
        if (!target) return;
        for (let offset = -spreadSlices; offset <= spreadSlices; offset++) {
            const idx = target.sliceIndex + offset;
            if (idx >= 0 && idx < target.segment.steps) {
                const weight = offset === 0 ? 0.5 : 0.25 / spreadSlices;
                this.#injectIntoSlice(target.segment, idx, volumeMl * weight);
            }
        }
    }

    #injectIntoSlice(seg, sliceIndex, volumeMl) {
        const slice = seg.voxels[sliceIndex];
        if (!slice) return;
        // convert ml to mm^3 and distribute evenly
        const totalVoxels = slice.flat().filter(v => v).length;
        const volume = (volumeMl * 1000) / Math.max(1, totalVoxels);
        for (const row of slice) {
            for (const vox of row) {
                if (vox) vox.concentration += volume;
            }
        }
    }

    #injectWeightedSlice(seg, sliceIndex, volumeMl, weight = 1) {
        if (volumeMl <= 0 || weight <= 0) return;
        this.#injectIntoSlice(seg, sliceIndex, volumeMl * weight);
    }

    #injectRetrogradeJet(target, volumeMl) {
        const targetSeg = target.segment;
        const targetIdx = target.segmentIndex;
        const parentIdx = targetSeg.sourceSegment?.parent;
        const entrySlice = target.sliceIndex;

        // Local sheath jet at the puncture site.
        this.#injectWeightedSlice(targetSeg, entrySlice, volumeMl, 0.18);

        // Retrograde column from the sheath tip toward the bifurcation.
        const start = Math.max(0, entrySlice);
        const count = Math.max(1, start + 1);
        const branchVolume = volumeMl * 0.48;
        let branchWeightSum = 0;
        const branchWeights = [];
        const stride = Math.max(1, Math.floor(count / 18));
        for (let i = start; i >= 0; i -= stride) {
            const t = count <= 1 ? 0 : 1 - i / start;
            // Keep the bolus dense near the sheath but visibly connected all
            // the way back to the bifurcation/aorta.
            const weight = 0.45 + 0.55 * Math.exp(-t * 1.7);
            branchWeights.push([i, weight]);
            branchWeightSum += weight;
        }
        for (const [slice, weight] of branchWeights) {
            this.#injectWeightedSlice(targetSeg, slice, branchVolume, weight / branchWeightSum);
        }

        // Reflux into the parent aorta around the bifurcation. This is what
        // makes a sheath injection opacify the aorta before blood washes it
        // antegrade into both iliac branches.
        const parentSeg = Number.isInteger(parentIdx) ? this.grid.segments[parentIdx] : null;
        if (parentSeg) {
            const aortaVolume = volumeMl * 0.34;
            const distal = parentSeg.steps - 1;
            const aortaSlices = Math.min(28, parentSeg.steps);
            let aortaWeightSum = 0;
            const aortaWeights = [];
            for (let k = 0; k < aortaSlices; k++) {
                const slice = distal - k;
                const weight = Math.exp(-k / 10);
                aortaWeights.push([slice, weight]);
                aortaWeightSum += weight;
            }
            for (const [slice, weight] of aortaWeights) {
                this.#injectWeightedSlice(parentSeg, slice, aortaVolume, weight / aortaWeightSum);
            }
        } else {
            // Fallback for test vessels without parent metadata.
            this.injectAtPoint(targetSeg.start, volumeMl * 0.34, { excludeSheath: true, spreadSlices: 2 });
        }
    }

    update(dt) {
        const outgoing = [];
        for (let segIndex = 0; segIndex < this.grid.segments.length; segIndex++) {
            const seg = this.grid.segments[segIndex];
            const voxels = seg.voxels;
            const width = seg.width;
            const steps = seg.steps;
            const voxelSize = seg.voxelSize;
            const dist = seg.flowSpeed * dt;
            const frac = Math.min(1, Math.abs(dist) / voxelSize);
            const dir = Math.sign(dist) || 1;
            const nextConc = voxels.map(slice =>
                slice.map(row => row.map(v => (v ? v.concentration : 0))));

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
                        nextConc[i][y][x] -= moved;
                        const nextIdx = i + dir;
                        if (nextIdx >= 0 && nextIdx < steps) {
                            const dest = voxels[nextIdx][y][x];
                            if (dest) {
                                nextConc[nextIdx][y][x] += moved;
                            } else {
                                nextConc[i][y][x] += moved * 0.35;
                            }
                        } else if (moved > 0) {
                            outgoing.push({ segIndex, amount: moved, atEnd: dir > 0 });
                        }
                    }
                }
            }

            const washout = Math.exp(-this.washoutRate * dt);
            for (let i = 0; i < steps; i++) {
                for (let y = 0; y < width; y++) {
                    for (let x = 0; x < width; x++) {
                        const vox = voxels[i][y][x];
                        if (vox) vox.concentration = Math.max(0, nextConc[i][y][x] * washout);
                    }
                }
            }
        }

        for (const out of outgoing) {
            this.#transferOutflow(out.segIndex, out.amount, out.atEnd);
        }

        for (const seg of this.grid.segments) {
            const voxels = seg.voxels;
            const width = seg.width;
            const steps = seg.steps;

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

    #transferOutflow(segIndex, amount, atEnd) {
        const children = atEnd ? (this.segmentGraph[segIndex] || []) : [];
        if (children.length) {
            const totalArea = children.reduce((sum, childIdx) => {
                const child = this.grid.segments[childIdx];
                return sum + (child ? child.radius * child.radius : 0);
            }, 0) || children.length;
            for (const childIdx of children) {
                const child = this.grid.segments[childIdx];
                if (!child) continue;
                const share = amount * (child.radius * child.radius || 1) / totalArea;
                this.#injectIntoSlice(child, 0, share / 1000);
            }
            return;
        }

        const seg = this.grid.segments[segIndex];
        if (seg?.isSheath && this.vessel.sheath?.end) {
            this.injectAtPoint(this.vessel.sheath.end, amount / 1000, { excludeSheath: true, spreadSlices: 1 });
        }
    }

    #findNearestSegmentSlice(point, { excludeSheath = false } = {}) {
        const p = new THREE.Vector3(point.x, point.y, point.z);
        let best = null;
        for (const segment of this.grid.segments) {
            if (excludeSheath && segment.isSheath) continue;
            const rel = new THREE.Vector3().subVectors(p, segment.start);
            const axial = THREE.MathUtils.clamp(rel.dot(segment.dir), 0, segment.steps * segment.voxelSize);
            const center = segment.start.clone().addScaledVector(segment.dir, axial);
            const radialDist = center.distanceTo(p);
            const score = Math.max(0, radialDist - segment.radius);
            const sliceIndex = THREE.MathUtils.clamp(Math.floor(axial / segment.voxelSize), 0, segment.steps - 1);
            if (!best || score < best.score) {
                best = { segment, segmentIndex: segment.segmentIndex, sliceIndex, score };
            }
        }
        return best;
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
    const { mesh, count } = updateVoxelInstancedMesh(agent, minConc, wireframe);
    return count > 0 && mesh ? [mesh] : [];
}

export function updateVoxelInstancedMesh(agent, minConc = 1e-4, wireframe = false, mesh = null, maxInstances = 18000, sampleStride = 1) {
    if (!agent || !agent.grid) return { mesh, count: 0 };
    const size = agent.grid.voxelSize;
    const capacity = Math.max(maxInstances, 128);
    const stride = Math.max(1, Math.floor(sampleStride));

    const ensureMesh = () => {
        if (mesh && mesh.userData.capacity >= capacity && mesh.userData.voxelSize === size) return mesh;
        mesh?.geometry?.dispose?.();
        mesh?.material?.dispose?.();
        const geometry = new THREE.BoxGeometry(size, size, size);
        const material = new THREE.MeshBasicMaterial({
            wireframe,
            vertexColors: true,
            transparent: true,
            opacity: wireframe ? 0.9 : 0.95,
            blending: THREE.AdditiveBlending,
            depthTest: false,
            depthWrite: false
        });
        mesh = new THREE.InstancedMesh(geometry, material, capacity);
        mesh.userData.capacity = capacity;
        mesh.userData.voxelSize = size;
        mesh.frustumCulled = false;
        mesh.instanceMatrix.setUsage(THREE.DynamicDrawUsage);
        return mesh;
    };

    const matrix = new THREE.Matrix4();
    const color = new THREE.Color();
    let activeCount = 0;
    let instance = 0;
    outer:
    for (const seg of agent.grid.segments) {
        const width = seg.width;
        const steps = seg.steps;
        for (let i = 0; i < steps; i++) {
            for (let y = 0; y < width; y++) {
                for (let x = 0; x < width; x++) {
                    const vox = seg.voxels[i][y][x];
                    if (!vox || vox.concentration <= minConc) continue;
                    activeCount++;
                    if ((activeCount - 1) % stride !== 0) {
                        continue;
                    }
                    if (instance >= maxInstances) break outer;
                    const target = ensureMesh();
                    const density = Math.min(1, 0.35 + (1 - Math.exp(-vox.concentration * 0.55)) * 1.4);
                    matrix.makeTranslation(vox.position.x, vox.position.y, vox.position.z);
                    target.setMatrixAt(instance, matrix);
                    color.setRGB(density, density, density);
                    target.setColorAt(instance, color);
                    instance++;
                }
            }
        }
    }

    if (activeCount === 0) {
        if (mesh) {
            mesh.count = 0;
            mesh.visible = false;
        }
        return { mesh, count: 0 };
    }

    mesh.visible = true;
    mesh.material.wireframe = wireframe;
    mesh.material.opacity = wireframe ? 0.9 : 0.95;
    mesh.material.transparent = true;
    mesh.material.blending = THREE.AdditiveBlending;
    mesh.material.depthTest = false;
    mesh.material.depthWrite = false;
    mesh.count = instance;
    mesh.instanceMatrix.needsUpdate = true;
    if (mesh.instanceColor) mesh.instanceColor.needsUpdate = true;
    return { mesh, count: instance };
}
