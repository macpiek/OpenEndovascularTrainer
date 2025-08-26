import * as THREE from 'three';

export class ContrastAgent {
    constructor(segments, startIndex, speed = 50) {
        this.segments = segments;
        this.startIndex = startIndex;
        this.speed = speed;
        this.segmentLengths = [];
        this.cumulativeLengths = [];
        let cumulative = 0;
        for (let i = startIndex; i < segments.length; i++) {
            const s = segments[i];
            const len = Math.sqrt(
                (s.end.x - s.start.x) * (s.end.x - s.start.x) +
                (s.end.y - s.start.y) * (s.end.y - s.start.y) +
                (s.end.z - s.start.z) * (s.end.z - s.start.z)
            );
            this.segmentLengths.push(len);
            cumulative += len;
            this.cumulativeLengths.push(cumulative);
        }
        this.totalLength = cumulative;
        this.filledLength = 0;
        this.active = false;
    }

    start() {
        this.filledLength = 0;
        this.active = true;
    }

    update(dt) {
        if (!this.active) return;
        this.filledLength += this.speed * dt;
        if (this.filledLength >= this.totalLength) {
            this.filledLength = this.totalLength;
            this.active = false;
        }
    }

    isActive() {
        return this.active;
    }
}

export function getContrastGeometry(agent) {
    if (!agent || agent.filledLength <= 0) return null;
    const points = [];
    let remaining = agent.filledLength;
    const baseIndex = agent.startIndex;
    const firstSeg = agent.segments[baseIndex];
    points.push(new THREE.Vector3(firstSeg.start.x, firstSeg.start.y, firstSeg.start.z));
    for (let i = 0; i < agent.segmentLengths.length && remaining > 0; i++) {
        const seg = agent.segments[baseIndex + i];
        const length = agent.segmentLengths[i];
        if (remaining >= length) {
            points.push(new THREE.Vector3(seg.end.x, seg.end.y, seg.end.z));
            remaining -= length;
        } else {
            const t = remaining / length;
            const endX = seg.start.x + (seg.end.x - seg.start.x) * t;
            const endY = seg.start.y + (seg.end.y - seg.start.y) * t;
            const endZ = seg.start.z + (seg.end.z - seg.start.z) * t;
            points.push(new THREE.Vector3(endX, endY, endZ));
            remaining = 0;
        }
    }
    const geometry = new THREE.BufferGeometry().setFromPoints(points);
    const material = new THREE.LineBasicMaterial({ color: 0xffffff });
    return new THREE.Line(geometry, material);
}
