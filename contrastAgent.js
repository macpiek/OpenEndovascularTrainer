import * as THREE from 'three';

// Simulates advection of a contrast agent through a vessel graph.
export class ContrastAgent {
    constructor(vessel, washout = 0.5) {
        this.vessel = vessel;
        this.segments = vessel.segments;
        this.graph = vessel.segmentGraph;
        this.washout = washout;

        this.lengths = this.segments.map(s => {
            const dx = s.end.x - s.start.x;
            const dy = s.end.y - s.start.y;
            const dz = s.end.z - s.start.z;
            return Math.sqrt(dx * dx + dy * dy + dz * dz) || 1;
        });
        this.concentration = new Array(this.segments.length).fill(0);

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

    // Adds concentration into the segment connected to the sheath.
    inject(rate) {
        if (this.sheathIndex >= 0) {
            this.concentration[this.sheathIndex] += rate;
        }
    }

    update(dt) {
        const next = new Array(this.segments.length).fill(0);
        for (let i = 0; i < this.segments.length; i++) {
            const seg = this.segments[i];
            const speed = seg.flowSpeed || 0;
            const frac = Math.min(1, (speed * dt) / this.lengths[i]);
            const moved = this.concentration[i] * frac;
            const remain = this.concentration[i] - moved;

            const children = this.graph[i] || [];
            if (children.length) {
                let total = 0;
                for (const c of children) total += this.segments[c].flowSpeed || 0;
                for (const c of children) {
                    const w = total > 0 ? (this.segments[c].flowSpeed || 0) / total : 1 / children.length;
                    next[c] += moved * w;
                }
            }
            next[i] += remain;
        }
        const decay = Math.exp(-this.washout * dt);
        for (let i = 0; i < next.length; i++) {
            next[i] *= decay;
        }
        this.concentration = next;
    }

    isActive() {
        return this.concentration.some(c => c > 1e-3);
    }
}

// Generate TubeGeometry for segments with contrast.  The caller is
// responsible for assigning materials based on the returned
// concentration value.
export function getContrastGeometry(agent) {
    if (!agent || !agent.isActive()) return [];
    const geoms = [];
    for (let i = 0; i < agent.segments.length; i++) {
        const c = agent.concentration[i];
        if (c <= 1e-3) continue;
        const seg = agent.segments[i];
        const start = new THREE.Vector3(seg.start.x, seg.start.y, seg.start.z);
        const end = new THREE.Vector3(seg.end.x, seg.end.y, seg.end.z);
        const path = new THREE.LineCurve3(start, end);
        const geom = new THREE.TubeGeometry(path, 4, seg.radius * 0.9, 8, false);
        geoms.push({ geometry: geom, concentration: c });
    }
    return geoms;
}
