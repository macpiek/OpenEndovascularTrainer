import * as THREE from 'three';
import { mergeBufferGeometries } from 'three/examples/jsm/utils/BufferGeometryUtils.js';

// Pool of geometries indexed by "segmentIndex:sampleIndex" keys so that
// TubeGeometry instances and their associated color buffers can be reused
// across frames instead of being recreated every time.
const geometryPool = new Map();

// Simulates advection and dilution of a contrast agent through a vessel graph.
export class ContrastAgent {
    constructor(vessel, washout = 0.5, backflow = 0.2, debug = false, samplesPerSegment = 10) {
        this.vessel = vessel;
        this.segments = vessel.segments;
        this.nodes = vessel.nodes || [];
        this.washout = washout;
        this.backflow = backflow;
        this.debug = debug;
        this.samplesPerSegment = samplesPerSegment;

        this.lengths = this.segments.map(s => s.length || 1);
        this.volumes = this.segments.map(s => s.volume || 1);
        this.concentration = this.segments.map(() => new Array(this.samplesPerSegment).fill(0));
        // Preallocate next concentration buffers so we can reuse them each frame
        this.next = this.segments.map(() => new Array(this.samplesPerSegment).fill(0));
        this.pendingNodeMass = new Array(this.nodes.length).fill(0);

        const eps = 1e-6;
        const equal = (a, b) =>
            Math.abs(a.x - b.x) < eps &&
            Math.abs(a.y - b.y) < eps &&
            Math.abs(a.z - b.z) < eps;
        // Default to injecting into the main vessel segment rather than a branch
        const mainNode = vessel.main ? vessel.main.start : this.segments[0].start;
        this.sheathIndex = this.segments.findIndex(
            s => mainNode && equal(s.start, mainNode)
        );
        if (this.sheathIndex === -1) this.sheathIndex = 0;
    }

    // Adds contrast volume (in milliliters) into a segment (default: main vessel
    // segment). Internally converts the volume to cubic millimeters so that
    // concentrations are stored in mm^3. By default, the contrast is injected at
    // the segment's start node. Pass `atEnd = true` to inject at the end node
    // instead.
    inject(volume, segmentIndex = this.sheathIndex, atEnd = false, spread = 1, injectionSpeed = null) {
        if (segmentIndex >= 0 && segmentIndex < this.segments.length) {
            const seg = this.segments[segmentIndex];

            const origSpeed = seg._origFlowSpeed !== undefined ? seg._origFlowSpeed : seg.flowSpeed;
            const arr = this.concentration[segmentIndex];
            const len = arr.length;
            const flowDir = Math.sign(origSpeed || 0) || 1; // start->end if >=0

            // Temporarily override flow speed to simulate a contrast jet
            if (injectionSpeed !== null) {
                if (seg._origFlowSpeed === undefined) {
                    seg._origFlowSpeed = seg.flowSpeed;
                }
                seg.flowSpeed = injectionSpeed;
                seg._restoreFlowSpeed = true;
            }

            // Convert from milliliters to cubic millimeters for internal storage
            const volumeMm3 = volume * 1000;
            const perSample = volumeMm3 / Math.max(1, spread);
            let idx = atEnd ? len - 1 : 0; // injection node
            const step = -flowDir; // distribute opposite to flow

            for (let k = 0; k < spread; k++) {
                if (idx < 0 || idx >= len) break;
                arr[idx] += perSample;
                idx += step;
            }

            if (this.debug) {
                console.log(
                    `Injected ${volumeMm3.toFixed(4)} mm^3 into segment ${segmentIndex} over ${Math.min(spread, len)} samples,`
                    + ` flow ${flowDir >= 0 ? 'start->end' : 'end->start'}`
                );
            }
        }
    }

    update(dt) {
        const next = this.next;
        for (let i = 0; i < next.length; i++) {
            next[i].fill(0);
        }
        const nodeMass = this.pendingNodeMass.slice();
        this.pendingNodeMass.fill(0);
        for (let i = 0; i < this.segments.length; i++) {
            const seg = this.segments[i];
            const speed = seg.flowSpeed || 0;
            const arr = this.concentration[i];
            const nextArr = next[i];
            const stepLen = this.lengths[i] / this.samplesPerSegment;
            const dist = speed * dt;
            const frac = Math.min(1, Math.abs(dist) / stepLen);
            const dir = Math.sign(dist) || 1;
            for (let j = 0; j < arr.length; j++) {
                const moved = arr[j] * frac;
                nextArr[j] += arr[j] - moved;
                if (dir >= 0) {
                    if (j < arr.length - 1) {
                        nextArr[j + 1] += moved;
                    } else {
                        const back = moved * this.backflow;
                        const fwd = moved - back;
                        if (seg.startNode != null) nodeMass[seg.startNode] += back;
                        if (seg.endNode != null) nodeMass[seg.endNode] += fwd;
                    }
                } else {
                    if (j > 0) {
                        nextArr[j - 1] += moved;
                    } else {
                        const back = moved * this.backflow;
                        const fwd = moved - back;
                        if (seg.endNode != null) nodeMass[seg.endNode] += back;
                        if (seg.startNode != null) nodeMass[seg.startNode] += fwd;
                    }
                }
            }
        }
        // Redistribute mixed contrast from nodes to connected segments.
        // Contrast should enter based on flow direction rather than always
        // appearing at the proximal start of a segment.
        for (let n = 0; n < this.nodes.length; n++) {
            const pool = nodeMass[n];
            if (pool <= 0) continue;
            const segs = this.nodes[n].segments || [];
            if (!segs.length) continue;
            let total = 0;
            for (const s of segs) total += Math.abs(this.segments[s].flowSpeed || 0);
            for (const s of segs) {
                const segObj = this.segments[s];
                const flow = segObj.flowSpeed || 0;
                const flowSign = Math.sign(flow);
                const w = total > 0 ? Math.abs(flow) / total : 1 / segs.length;
                const arr = next[s];
                const idxStart = flowSign < 0 ? arr.length - 1 : 0;
                const idxEnd = flowSign < 0 ? 0 : arr.length - 1;
                const idx = segObj.startNode === n ? idxStart : idxEnd;
                arr[idx] += pool * w;
                if (this.debug) {
                    console.log(
                        `Node ${n} -> segment ${s} index ${idx} (flow ${
                            flowSign >= 0 ? 'start->end' : 'end->start'
                        }) mass ${(pool * w).toFixed(4)}`
                    );
                }
            }
        }
        const decay = Math.exp(-this.washout * dt);
        for (let i = 0; i < next.length; i++) {
            for (let j = 0; j < next[i].length; j++) {
                next[i][j] *= decay;
            }
        }

        // Swap concentration buffers to avoid reallocating arrays
        const current = this.concentration;
        this.concentration = next;
        this.next = current;

        // Restore any temporary flow speed overrides after injection
        for (let i = 0; i < this.segments.length; i++) {
            const seg = this.segments[i];
            if (seg._restoreFlowSpeed) {
                seg.flowSpeed = seg._origFlowSpeed;
                delete seg._origFlowSpeed;
                delete seg._restoreFlowSpeed;
                if (this.debug) {
                    console.log(`Restored flowSpeed of segment ${i} to ${seg.flowSpeed}`);
                }
            }
        }
        if (this.debug) {
            const masses = this.concentration
                .map((arr, i) => `Seg ${i}: ${arr.reduce((a, b) => a + b, 0).toFixed(4)}`)
                .join(', ');
            console.log(`Contrast masses: ${masses}`);
        }
    }

    isActive() {
        return this.concentration.some((arr, i) => {
            const vol = (this.volumes[i] || 1) / this.samplesPerSegment;
            return arr.some(amt => amt / vol > 1e-4);
        });
    }

    getSegmentSamples(segmentIndex) {
        if (segmentIndex < 0 || segmentIndex >= this.segments.length) return [];
        const vol = (this.volumes[segmentIndex] || 1) / this.samplesPerSegment;
        return this.concentration[segmentIndex].map(amt => amt / vol);
    }
}

// Generate geometry for each sub-section of every segment containing contrast.
// Each geometry includes vertex colors representing concentration. The function
// can optionally merge all geometries into a single BufferGeometry.

export function getContrastGeometry(agent, merge = false) {
    // If the agent is inactive, dispose any pooled geometries and return.
    if (!agent || !agent.isActive()) {
        for (const geom of geometryPool.values()) geom.dispose();
        geometryPool.clear();
        return merge ? null : [];
    }

    const geoms = [];
    const used = new Set();

    for (let i = 0; i < agent.segments.length; i++) {
        const arr = agent.concentration[i];
        const vol = (agent.volumes[i] || 1) / arr.length;
        const seg = agent.segments[i];
        const start = new THREE.Vector3(seg.start.x, seg.start.y, seg.start.z);
        const end = new THREE.Vector3(seg.end.x, seg.end.y, seg.end.z);
        const dir = new THREE.Vector3().subVectors(end, start);
        for (let j = 0; j < arr.length; j++) {
            const conc = arr[j] / vol;
            if (conc <= 1e-4) continue;
            const key = `${i}:${j}`;

            let geom = geometryPool.get(key);
            if (!geom) {
                const t0 = j / arr.length;
                const t1 = (j + 1) / arr.length;
                const p0 = start.clone().addScaledVector(dir, t0);
                const p1 = start.clone().addScaledVector(dir, t1);
                const path = new THREE.LineCurve3(p0, p1);
                geom = new THREE.TubeGeometry(path, 1, seg.radius * 0.9, 8, false);

                const colors = new Float32Array(geom.attributes.position.count * 3);
                geom.setAttribute('color', new THREE.Float32BufferAttribute(colors, 3));
                geometryPool.set(key, geom);
            }

            // Update vertex colors based on concentration
            const color = new THREE.Color(conc, 0, 1 - conc);
            const colors = geom.attributes.color.array;
            for (let k = 0; k < colors.length; k += 3) {
                colors[k] = color.r;
                colors[k + 1] = color.g;
                colors[k + 2] = color.b;
            }
            geom.attributes.color.needsUpdate = true;
            geom.userData.concentration = conc;
            geoms.push(geom);
            used.add(key);
        }
    }

    // Dispose of any geometries that were not used this frame.
    for (const [key, geom] of geometryPool.entries()) {
        if (!used.has(key)) {
            geom.dispose();
            geometryPool.delete(key);
        }
    }

    if (merge) {
        return geoms.length ? mergeBufferGeometries(geoms, false) : null;
    }
    return geoms;
}
