import * as THREE from 'three';

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
        this.pendingNodeMass = new Array(this.nodes.length).fill(0);

        const eps = 1e-6;
        const equal = (a, b) =>
            Math.abs(a.x - b.x) < eps &&
            Math.abs(a.y - b.y) < eps &&
            Math.abs(a.z - b.z) < eps;
        this.sheathIndex = this.segments.findIndex(
            s => equal(s.end, vessel.left.end)
        );
        if (this.sheathIndex === -1) this.sheathIndex = 0;
    }

    // Adds contrast volume into a segment (default: sheath-connected segment).
    // By default, the contrast is injected at the segment's end node. Pass
    // `atEnd = false` to inject at the start node instead.
    inject(volume, segmentIndex = this.sheathIndex, atEnd = true) {
        if (segmentIndex >= 0 && segmentIndex < this.segments.length) {
            const seg = this.segments[segmentIndex];
            const node = atEnd ? seg.endNode : seg.startNode;
            if (node != null) {
                this.pendingNodeMass[node] += volume;
            }
        }
    }

    update(dt) {
        const next = this.segments.map(() => new Array(this.samplesPerSegment).fill(0));
        const nodeMass = this.pendingNodeMass.slice();
        this.pendingNodeMass.fill(0);
        for (let i = 0; i < this.segments.length; i++) {
            const seg = this.segments[i];
            const speed = seg.flowSpeed || 0;
            const arr = this.concentration[i];
            const nextArr = next[i];
            const stepLen = this.lengths[i] / this.samplesPerSegment;
            const frac = Math.min(1, (speed * dt) / stepLen);
            for (let j = 0; j < arr.length; j++) {
                const moved = arr[j] * frac;
                nextArr[j] += arr[j] - moved;
                if (j < arr.length - 1) {
                    nextArr[j + 1] += moved;
                } else {
                    const back = moved * this.backflow;
                    const fwd = moved - back;
                    if (seg.startNode != null) nodeMass[seg.startNode] += back;
                    if (seg.endNode != null) nodeMass[seg.endNode] += fwd;
                }
            }
        }
        // Redistribute mixed contrast from nodes to connected segments
        for (let n = 0; n < this.nodes.length; n++) {
            const pool = nodeMass[n];
            if (pool <= 0) continue;
            const segs = this.nodes[n].segments || [];
            if (!segs.length) continue;
            let total = 0;
            for (const s of segs) total += this.segments[s].flowSpeed || 0;
            for (const s of segs) {
                const w = total > 0 ? (this.segments[s].flowSpeed || 0) / total : 1 / segs.length;
                next[s][0] += pool * w;
            }
        }
        const decay = Math.exp(-this.washout * dt);
        for (let i = 0; i < next.length; i++) {
            for (let j = 0; j < next[i].length; j++) {
                next[i][j] *= decay;
            }
        }
        this.concentration = next;
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

// Generate TubeGeometry for segments with contrast. The caller is
// responsible for assigning materials based on the returned
// concentration value.
export function getContrastGeometry(agent) {
    if (!agent || !agent.isActive()) return [];
    const geoms = [];
    for (let i = 0; i < agent.segments.length; i++) {
        const amt = agent.concentration[i].reduce((a, b) => a + b, 0);
        const conc = amt / (agent.volumes[i] || 1);
        if (conc <= 1e-4) continue;
        const seg = agent.segments[i];
        const start = new THREE.Vector3(seg.start.x, seg.start.y, seg.start.z);
        const end = new THREE.Vector3(seg.end.x, seg.end.y, seg.end.z);
        const path = new THREE.LineCurve3(start, end);
        const geom = new THREE.TubeGeometry(path, 4, seg.radius * 0.9, 8, false);
        geoms.push({ geometry: geom, concentration: conc });
    }
    return geoms;
}
