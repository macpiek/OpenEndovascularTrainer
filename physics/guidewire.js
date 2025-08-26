const wallFriction = 0.02;

function clamp(v, min, max) {
    return Math.min(Math.max(v, min), max);
}

function projectOnSegment(n, seg) {
    const vx = seg.end.x - seg.start.x;
    const vy = seg.end.y - seg.start.y;
    const vz = (seg.end.z || 0) - (seg.start.z || 0);
    const wx = n.x - seg.start.x;
    const wy = n.y - seg.start.y;
    const wz = n.z - (seg.start.z || 0);
    const len2 = vx * vx + vy * vy + vz * vz;
    let t = (wx * vx + wy * vy + wz * vz) / len2;
    t = clamp(t, 0, 1);
    const px = seg.start.x + vx * t;
    const py = seg.start.y + vy * t;
    const pz = (seg.start.z || 0) + vz * t;
    const dx = n.x - px;
    const dy = n.y - py;
    const dz = n.z - pz;
    const dist = Math.sqrt(dx * dx + dy * dy + dz * dz);
    return {px, py, pz, dx, dy, dz, dist};
}

function clampToVessel(n, vessel, affectVelocity = true) {
    let nearest = vessel.segments[0];
    let best = projectOnSegment(n, nearest);
    for (let i = 1; i < vessel.segments.length; i++) {
        const seg = vessel.segments[i];
        const p = projectOnSegment(n, seg);
        if (p.dist < best.dist) {
            best = p;
            nearest = seg;
        }
    }
    const radius = nearest.radius - 1;
    if (best.dist > radius) {
        const inv = 1 / best.dist;
        const nx = best.dx * inv;
        const ny = best.dy * inv;
        const nz = best.dz * inv;
        n.x = best.px + nx * radius;
        n.y = best.py + ny * radius;
        n.z = best.pz + nz * radius;
        if (affectVelocity) {
            const vn = n.vx * nx + n.vy * ny + n.vz * nz;
            n.vx = (n.vx - vn * nx) * (1 - wallFriction);
            n.vy = (n.vy - vn * ny) * (1 - wallFriction);
            n.vz = (n.vz - vn * nz) * (1 - wallFriction);
        }
    }
}

let wireStiffness = 1;
export function setWireStiffness(value) {
    wireStiffness = value;
}

export class Guidewire {
    constructor(segLen, count, start, dir, vessel) {
        this.segmentLength = segLen;
        this.tailStart = start;
        this.dir = dir;
        this.vessel = vessel;
        this.nodes = [];
        for (let i = 0; i < count; i++) {
            const x = vessel.left.end.x - dir.x * segLen * i;
            const y = vessel.left.end.y - dir.y * segLen * i;
            const z = vessel.left.end.z - dir.z * segLen * i;
            this.nodes.push({x, y, z, vx: 0, vy: 0, vz: 0, fx: 0, fy: 0, fz: 0, oldx: x, oldy: y, oldz: z});
        }
        this.tailProgress = 0;
        this.maxInsert = segLen * (count - 1);
    }

    advanceTail(advance, dt) {
        this.tailProgress = clamp(this.tailProgress + advance * 40 * dt, 0, this.maxInsert);
        const tail = this.nodes[this.nodes.length - 1];
        tail.x = this.tailStart.x + this.dir.x * this.tailProgress;
        tail.y = this.tailStart.y + this.dir.y * this.tailProgress;
        tail.z = this.tailStart.z + this.dir.z * this.tailProgress;
        tail.vx = tail.vy = tail.vz = 0;
        if (advance > 0) {
            const tip = this.nodes[0];
            tip.fx += this.dir.x * 500;
            tip.fy += this.dir.y * 500;
            tip.fz += this.dir.z * 500;
        }
    }

    accumulateForces() {
        for (const n of this.nodes) {
            n.fx = n.fy = n.fz = 0;
            n.fx -= n.vx * 2;
            n.fy -= n.vy * 2;
            n.fz -= n.vz * 2;
        }
        const len = this.segmentLength;
        for (let i = 1; i < this.nodes.length; i++) {
            const a = this.nodes[i - 1];
            const b = this.nodes[i];
            let dx = b.x - a.x;
            let dy = b.y - a.y;
            let dz = b.z - a.z;
            const dist = Math.sqrt(dx * dx + dy * dy + dz * dz) || 1;
            const diff = dist - len;
            const k = 200;
            const force = k * diff;
            const inv = 1 / dist;
            const fx = force * dx * inv;
            const fy = force * dy * inv;
            const fz = force * dz * inv;
            a.fx += fx;
            a.fy += fy;
            a.fz += fz;
            b.fx -= fx;
            b.fy -= fy;
            b.fz -= fz;
        }
        for (let i = 1; i < this.nodes.length - 1; i++) {
            const prev = this.nodes[i - 1];
            const curr = this.nodes[i];
            const next = this.nodes[i + 1];
            const mx = (prev.x + next.x) * 0.5;
            const my = (prev.y + next.y) * 0.5;
            const mz = (prev.z + next.z) * 0.5;
            const bx = (mx - curr.x) * wireStiffness * 50;
            const by = (my - curr.y) * wireStiffness * 50;
            const bz = (mz - curr.z) * wireStiffness * 50;
            curr.fx += bx;
            curr.fy += by;
            curr.fz += bz;
        }
    }

    integrate(dt) {
        for (let i = 0; i < this.nodes.length - 1; i++) {
            const n = this.nodes[i];
            n.vx += n.fx * dt;
            n.vy += n.fy * dt;
            n.vz += n.fz * dt;
            n.x += n.vx * dt;
            n.y += n.vy * dt;
            n.z += n.vz * dt;
        }
    }

    solveDistances(iterations = 2) {
        const len = this.segmentLength;
        for (let k = 0; k < iterations; k++) {
            for (let i = 1; i < this.nodes.length; i++) {
                const a = this.nodes[i - 1];
                const b = this.nodes[i];
                let dx = b.x - a.x;
                let dy = b.y - a.y;
                let dz = b.z - a.z;
                const dist = Math.sqrt(dx * dx + dy * dy + dz * dz) || 1;
                const diff = (dist - len) / dist;
                const offx = dx * 0.5 * diff;
                const offy = dy * 0.5 * diff;
                const offz = dz * 0.5 * diff;
                a.x += offx;
                a.y += offy;
                a.z += offz;
                if (i !== this.nodes.length - 1) {
                    b.x -= offx;
                    b.y -= offy;
                    b.z -= offz;
                }
            }
        }
    }

    collide() {
        for (let i = 0; i < this.nodes.length - 1; i++) {
            clampToVessel(this.nodes[i], this.vessel);
        }
    }

    step(dt, advance) {
        for (const n of this.nodes) {
            n.oldx = n.x;
            n.oldy = n.y;
            n.oldz = n.z;
        }
        this.advanceTail(advance, dt);
        this.accumulateForces();
        this.integrate(dt);
        this.solveDistances(4);
        this.collide();
        for (let i = 0; i < this.nodes.length - 1; i++) {
            const n = this.nodes[i];
            n.vx = (n.x - n.oldx) / dt;
            n.vy = (n.y - n.oldy) / dt;
            n.vz = (n.z - n.oldz) / dt;
        }
    }
}

