import * as THREE from 'three';

const Y_AXIS = new THREE.Vector3(0, 1, 0);

function toVector(point) {
    return new THREE.Vector3(point.x, point.y, point.z);
}

function clampIndex(value, length) {
    return THREE.MathUtils.clamp(Math.floor(value), 0, Math.max(0, length - 1));
}

function smoothstep(edge0, edge1, x) {
    const t = THREE.MathUtils.clamp((x - edge0) / Math.max(1e-6, edge1 - edge0), 0, 1);
    return t * t * (3 - 2 * t);
}

function contrastSignal(seg, index) {
    if (index < 0 || index >= seg.cells) return 0;
    return seg.core[index] * 0.58 + seg.wall[index] * 0.72;
}

function contrastWallSignal(seg, index) {
    if (index < 0 || index >= seg.cells) return 0;
    return seg.wall[index];
}

function interpolatedSignal(seg, cellFloat) {
    const left = Math.floor(cellFloat);
    const t = cellFloat - left;
    return THREE.MathUtils.lerp(contrastSignal(seg, left), contrastSignal(seg, left + 1), t);
}

function interpolatedWallSignal(seg, cellFloat) {
    const left = Math.floor(cellFloat);
    const t = cellFloat - left;
    return THREE.MathUtils.lerp(contrastWallSignal(seg, left), contrastWallSignal(seg, left + 1), t);
}

function buildFlowSegments(vessel, cellLength) {
    return vessel.segments.map((sourceSegment, segmentIndex) => {
        const start = toVector(sourceSegment.start);
        const end = toVector(sourceSegment.end);
        const axis = new THREE.Vector3().subVectors(end, start);
        const length = Math.max(1, axis.length());
        const dir = axis.clone().normalize();
        const cells = Math.max(2, Math.ceil(length / cellLength));
        const actualCellLength = length / cells;
        return {
            sourceSegment,
            segmentIndex,
            start,
            end,
            dir,
            length,
            cells,
            cellLength: actualCellLength,
            radius: sourceSegment.radius,
            area: Math.PI * sourceSegment.radius * sourceSegment.radius,
            flowSpeed: sourceSegment.flowSpeed || 0,
            isSheath: !!sourceSegment.isSheath,
            core: new Float32Array(cells),
            wall: new Float32Array(cells),
            nextCore: new Float32Array(cells),
            nextWall: new Float32Array(cells),
            orientation: new THREE.Quaternion().setFromUnitVectors(Y_AXIS, dir)
        };
    });
}

// Lightweight contrast transport model. It follows the vessel centreline
// graph, but each axial cell has a fast central stream and a slower wall layer.
// That gives a recognisable angiographic bolus front, trailing edge, reflux,
// and washout without a costly 3D voxel fluid simulation.
export class FlowContrastAgent {
    constructor(vessel, cellLength = 3.5) {
        this.vessel = vessel;
        this.segments = buildFlowSegments(vessel, cellLength);
        this.segmentGraph = vessel.segmentGraph || vessel.segments.map(() => []);
        this.outgoing = Array.from(
            { length: this.segments.length * 2 },
            () => ({ segmentIndex: -1, amount: 0, wallShare: 0, sourceArea: 0 })
        );
        this.outgoingCount = 0;
        this.sheathSegmentIndex = vessel.segments.findIndex(seg => seg.isSheath);
        this.time = 0;
        this.totalSignal = 0;
        this.lastInjectionTime = -Infinity;
        this.coreSpeedScale = 1.82;
        this.wallSpeedScale = 1.24;
        this.wallExchange = 4.6;
        this.axialDispersion = 0.46;
        this.clearance = 0.95;
        this.tailClearance = 3.1;
    }

    injectThroughSheath(volumeMl, rateMlPerSec = 0) {
        if (volumeMl <= 0) return;
        this.lastInjectionTime = this.time;

        if (this.sheathSegmentIndex >= 0) {
            const sheath = this.segments[this.sheathSegmentIndex];
            this.#addToCell(sheath, sheath.cells - 1, volumeMl * 0.06, 0.48);
        }

        const tip = this.vessel.sheath?.end;
        const target = tip ? this.#findNearestSegmentCell(tip, { excludeSheath: true }) : null;
        if (!target) {
            const main = this.segments.find(seg => !seg.isSheath);
            if (main) this.#addToCell(main, 0, volumeMl, 0.85);
            return;
        }
        this.#injectSheathJet(target, volumeMl * 0.92, rateMlPerSec);
    }

    update(dt) {
        this.time += dt;
        if (this.totalSignal <= 0) return;
        this.totalSignal = 0;
        const pulse = 1 + 0.18 * Math.sin(this.time * Math.PI * 2.15);
        this.outgoingCount = 0;

        for (let segmentIndex = 0; segmentIndex < this.segments.length; segmentIndex++) {
            const seg = this.segments[segmentIndex];
            seg.nextCore.set(seg.core);
            seg.nextWall.set(seg.wall);
            const coreFrac = Math.min(0.96, Math.max(0, seg.flowSpeed * this.coreSpeedScale * pulse * dt / seg.cellLength));
            const wallFrac = Math.min(0.78, Math.max(0, seg.flowSpeed * this.wallSpeedScale * dt / seg.cellLength));
            this.#advectCompartment(seg, seg.core, seg.nextCore, coreFrac, 1);
            this.#advectCompartment(seg, seg.wall, seg.nextWall, wallFrac, 0.35);

            seg.core.set(seg.nextCore);
            seg.wall.set(seg.nextWall);
            this.#exchangeAndDisperse(seg, dt);
        }

        for (let index = 0; index < this.outgoingCount; index++) {
            this.#transferOutflow(this.outgoing[index]);
        }

        const postBolus = this.time - this.lastInjectionTime > 0.38;
        for (let segmentIndex = 0; segmentIndex < this.segments.length; segmentIndex++) {
            const seg = this.segments[segmentIndex];
            for (let i = 0; i < seg.cells; i++) {
                const localSignal = seg.core[i] + seg.wall[i] * 0.8;
                const tailFactor = postBolus ? 1 - smoothstep(0.012, 0.13, localSignal) : 0;
                const coreDecay = Math.exp(-(this.clearance + tailFactor * this.tailClearance) * dt);
                const wallDecay = Math.exp(-(this.clearance * 1.2 + tailFactor * this.tailClearance * 1.35) * dt);
                seg.core[i] *= coreDecay;
                seg.wall[i] *= wallDecay;
                if (seg.core[i] < 4e-4) seg.core[i] = 0;
                if (seg.wall[i] < 4e-4) seg.wall[i] = 0;
                this.totalSignal += seg.core[i] + seg.wall[i] * 0.8;
            }
        }
    }

    hasVisibleContrast(threshold = 0.02) {
        return this.totalSignal > threshold;
    }

    #injectSheathJet(target, volumeMl, rateMlPerSec) {
        const seg = target.segment;
        const entry = target.cellIndex;
        const jetStrength = THREE.MathUtils.clamp(rateMlPerSec / 45, 0.35, 1.35);

        // Dense central jet from the sheath tip.
        this.#addToCell(seg, entry, volumeMl * 0.25, 0.72);

        // Retrograde streak toward the aortic bifurcation. High-rate injection
        // reaches farther upstream; low-rate injection stays more local.
        const proximalCells = entry + 1;
        const retroCells = Math.max(5, Math.min(proximalCells, Math.round(18 * jetStrength)));
        let retroWeight = 0;
        const weights = [];
        for (let k = 0; k < retroCells; k++) {
            const idx = entry - k;
            if (idx < 0) break;
            const w = Math.exp(-k / (5.5 + jetStrength * 4));
            weights.push([idx, w]);
            retroWeight += w;
        }
        for (const [idx, weight] of weights) {
            this.#addToCell(seg, idx, volumeMl * 0.4 * weight / retroWeight, 0.64);
        }

        // If this branch has a parent aorta segment, opacify the distal aorta
        // around the bifurcation so blood can wash it antegrade into both legs.
        const parentIdx = seg.sourceSegment?.parent;
        const parent = Number.isInteger(parentIdx) ? this.segments[parentIdx] : null;
        if (parent) {
            const aortaCells = Math.min(parent.cells, Math.round(24 * jetStrength));
            let parentWeight = 0;
            const parentWeights = [];
            for (let k = 0; k < aortaCells; k++) {
                const idx = parent.cells - 1 - k;
                const w = Math.exp(-k / 8);
                parentWeights.push([idx, w]);
                parentWeight += w;
            }
            for (const [idx, weight] of parentWeights) {
                this.#addToCell(parent, idx, volumeMl * 0.35 * weight / parentWeight, 0.58);
            }
        }
    }

    #addToCell(seg, cellIndex, volumeMl, coreFraction) {
        if (!seg || volumeMl <= 0) return;
        const idx = clampIndex(cellIndex, seg.cells);
        const cellVolume = Math.max(1, seg.area * seg.cellLength);
        const concentration = volumeMl * 1000 / cellVolume;
        seg.core[idx] += concentration * coreFraction;
        seg.wall[idx] += concentration * (1 - coreFraction);
        this.totalSignal += concentration;
    }

    #advectCompartment(seg, values, next, frac, wallShare) {
        if (frac <= 0) return;
        for (let i = seg.cells - 1; i >= 0; i--) {
            const amount = values[i] * frac;
            next[i] -= amount;
            if (i + 1 < seg.cells) {
                next[i + 1] += amount;
            } else if (amount > 0) {
                const out = this.outgoing[this.outgoingCount++];
                out.segmentIndex = seg.segmentIndex;
                out.amount = amount;
                out.wallShare = wallShare;
                out.sourceArea = seg.area;
            }
        }
    }

    #exchangeAndDisperse(seg, dt) {
        const exchange = THREE.MathUtils.clamp(this.wallExchange * dt, 0, 0.22);
        const disperse = THREE.MathUtils.clamp(this.axialDispersion * dt, 0, 0.08);
        for (let i = 0; i < seg.cells; i++) {
            const delta = (seg.core[i] - seg.wall[i]) * exchange;
            seg.core[i] -= delta;
            seg.wall[i] += delta;
        }
        if (disperse <= 0 || seg.cells < 3) return;
        seg.nextCore.set(seg.core);
        seg.nextWall.set(seg.wall);
        for (let i = 1; i < seg.cells - 1; i++) {
            seg.nextCore[i] += (seg.core[i - 1] + seg.core[i + 1] - seg.core[i] * 2) * disperse;
            seg.nextWall[i] += (seg.wall[i - 1] + seg.wall[i + 1] - seg.wall[i] * 2) * disperse * 1.35;
        }
        seg.core.set(seg.nextCore);
        seg.wall.set(seg.nextWall);
    }

    #transferOutflow(out) {
        const children = this.segmentGraph[out.segmentIndex] || [];
        if (!children.length) return;
        let totalArea = 0;
        for (let index = 0; index < children.length; index++) {
            totalArea += this.segments[children[index]]?.area || 0;
        }
        if (totalArea <= 0) totalArea = children.length;

        for (let index = 0; index < children.length; index++) {
            const childIdx = children[index];
            const child = this.segments[childIdx];
            if (!child) continue;
            const share = (child.area || 1) / totalArea;
            const amount = out.amount * share * out.sourceArea * child.cellLength / 1000;
            this.#addToCell(child, 0, amount, 0.58 + (1 - out.wallShare) * 0.18);
        }
    }

    #findNearestSegmentCell(point, { excludeSheath = false } = {}) {
        const p = toVector(point);
        let best = null;
        for (const segment of this.segments) {
            if (excludeSheath && segment.isSheath) continue;
            const rel = new THREE.Vector3().subVectors(p, segment.start);
            const axial = THREE.MathUtils.clamp(rel.dot(segment.dir), 0, segment.length);
            const center = segment.start.clone().addScaledVector(segment.dir, axial);
            const radialDist = center.distanceTo(p);
            const score = Math.max(0, radialDist - segment.radius);
            const cellIndex = clampIndex(axial / segment.cellLength, segment.cells);
            if (!best || score < best.score) {
                best = { segment, segmentIndex: segment.segmentIndex, cellIndex, score };
            }
        }
        return best;
    }
}

export function updateFlowContrastMesh(agent, minConc = 0.015, wireframe = false, mesh = null) {
    if (!agent?.segments) return { mesh, count: 0 };
    const sourceGeometry = agent.vessel?.geometry;
    if (!sourceGeometry?.attributes?.position) return { mesh, count: 0 };

    if (!mesh || !mesh.isMesh || mesh.userData.sourceGeometry !== sourceGeometry) {
        disposeContrastMesh(mesh);
        const geometry = sourceGeometry.clone();
        const vertexCount = geometry.attributes.position.count;
        geometry.setAttribute('color', new THREE.BufferAttribute(new Float32Array(vertexCount * 3), 3));
        const material = new THREE.MeshBasicMaterial({
            vertexColors: true,
            transparent: true,
            opacity: wireframe ? 0.78 : 0.96,
            blending: THREE.AdditiveBlending,
            depthTest: false,
            depthWrite: false,
            side: THREE.DoubleSide,
            wireframe
        });
        mesh = new THREE.Mesh(geometry, material);
        mesh.frustumCulled = false;
        mesh.userData.sourceGeometry = sourceGeometry;
        mesh.userData.influences = buildSurfaceInfluences(agent, geometry);
    }

    const colors = mesh.geometry.attributes.color;
    const influences = mesh.userData.influences || [];
    let activeVertices = 0;
    for (let i = 0; i < colors.count; i++) {
        const vertexInfluence = influences[i];
        let signal = 0;
        let wall = 0;
        if (vertexInfluence?.length) {
            for (const inf of vertexInfluence) {
                const seg = agent.segments[inf.segmentIndex];
                signal += interpolatedSignal(seg, inf.cellFloat) * inf.weight;
                wall += interpolatedWallSignal(seg, inf.cellFloat) * inf.weight;
            }
        }
        const density = smoothstep(minConc * 0.18, minConc * 4.4, signal);
        const wallFill = smoothstep(minConc * 0.28, minConc * 4.0, wall);
        const lumenSignal = Math.max(density, wallFill * 0.82);
        const brightness = lumenSignal > 0.018 ? Math.min(1, Math.pow(lumenSignal, 0.78) * 1.18) : 0;
        if (brightness > 0.02) activeVertices++;
        colors.setXYZ(i, brightness, brightness, brightness);
    }
    colors.needsUpdate = true;

    mesh.visible = activeVertices > 0;
    mesh.material.wireframe = wireframe;
    mesh.material.opacity = wireframe ? 0.78 : 0.96;
    return { mesh, count: activeVertices };
}

function buildSurfaceInfluences(agent, geometry) {
    const position = geometry.attributes.position;
    const point = new THREE.Vector3();
    const influences = new Array(position.count);
    for (let i = 0; i < position.count; i++) {
        point.fromBufferAttribute(position, i);
        const candidates = [];
        for (const seg of agent.segments) {
            if (seg.isSheath) continue;
            const rel = new THREE.Vector3().subVectors(point, seg.start);
            const axial = rel.dot(seg.dir);
            const axialClamped = THREE.MathUtils.clamp(axial, 0, seg.length);
            const center = seg.start.clone().addScaledVector(seg.dir, axialClamped);
            const radialDist = point.distanceTo(center);
            const axialOvershoot = Math.max(0, -axial, axial - seg.length);
            const surfaceDistance = Math.abs(radialDist - seg.radius) + axialOvershoot * 0.45;
            const sigma = Math.max(2, seg.radius * 0.48);
            const weight = Math.exp(-(surfaceDistance * surfaceDistance) / (2 * sigma * sigma));
            if (weight < 0.02) continue;
            const cellFloat = THREE.MathUtils.clamp(axialClamped / seg.cellLength - 0.5, 0, seg.cells - 1);
            candidates.push({ segmentIndex: seg.segmentIndex, cellFloat, weight });
        }
        candidates.sort((a, b) => b.weight - a.weight);
        const picked = candidates.slice(0, 3);
        const total = picked.reduce((sum, inf) => sum + inf.weight, 0);
        influences[i] = total > 0
            ? picked.map(inf => ({ ...inf, weight: inf.weight / total }))
            : [];
    }
    return influences;
}

function disposeContrastMesh(mesh) {
    if (!mesh) return;
    if (mesh.isGroup) {
        for (const child of mesh.children) {
            child.geometry?.dispose?.();
            child.material?.dispose?.();
        }
    } else {
        mesh.geometry?.dispose?.();
        mesh.material?.dispose?.();
    }
}
