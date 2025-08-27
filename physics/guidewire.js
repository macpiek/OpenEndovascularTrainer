// Wall interaction parameters with defaults
let wallStaticFriction = 0.2;
let wallKineticFriction = 0.1;
let wallNormalDamping = 0.5;

// Force applied to the tip when advancing the tail
let advanceForce = 100;

// Global velocity damping applied each integrate step
let velocityDamping = 0.98;

// Allow configuration from the outside
export function setWallFriction(staticCoeff, kineticCoeff) {
    wallStaticFriction = staticCoeff;
    wallKineticFriction = kineticCoeff;
}

export function setNormalDamping(value) {
    wallNormalDamping = value;
}

export function setAdvanceForce(value) {
    advanceForce = value;
}

export function setVelocityDamping(value) {
    velocityDamping = value;
}

function clamp(v, min, max) {
    return Math.min(Math.max(v, min), max);
}

function isBeyond(start, end, p) {
    const dx = end.x - start.x;
    const dy = end.y - start.y;
    const dz = (end.z || 0) - (start.z || 0);
    const vx = p.x - start.x;
    const vy = p.y - start.y;
    const vz = p.z - (start.z || 0);
    const len2 = dx * dx + dy * dy + dz * dz;
    return (vx * dx + vy * dy + vz * dz) > len2;
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

function clampToVessel(
    n,
    vessel,
    affectVelocity = true,
    staticFriction = wallStaticFriction,
    kineticFriction = wallKineticFriction,
    normalDamping = wallNormalDamping,
    openEnds = {}
) {
    if (openEnds.left && isBeyond(vessel.branchPoint, vessel.left.end, n)) return;
    if (openEnds.right && isBeyond(vessel.branchPoint, vessel.right.end, n)) return;

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
            let tx = n.vx - vn * nx;
            let ty = n.vy - vn * ny;
            let tz = n.vz - vn * nz;
            const tMag = Math.sqrt(tx * tx + ty * ty + tz * tz);
            if (tMag < staticFriction) {
                tx = ty = tz = 0;
            } else {
                tx *= (1 - kineticFriction);
                ty *= (1 - kineticFriction);
                tz *= (1 - kineticFriction);
            }
            const dampedVn = vn * (1 - normalDamping);
            n.vx = tx + dampedVn * nx;
            n.vy = ty + dampedVn * ny;
            n.vz = tz + dampedVn * nz;
        }
    }
}

function clampToSheath(n, sheath) {
    const sx = sheath.start.x;
    const sy = sheath.start.y;
    const sz = sheath.start.z;
    const ex = sheath.end.x;
    const ey = sheath.end.y;
    const ez = sheath.end.z;
    const dx = ex - sx;
    const dy = ey - sy;
    const dz = (ez || 0) - (sz || 0);
    const len2 = dx * dx + dy * dy + dz * dz;
    const px = n.x - sx;
    const py = n.y - sy;
    const pz = n.z - (sz || 0);
    let t = (px * dx + py * dy + pz * dz) / len2;
    const cx = sx + dx * t;
    const cy = sy + dy * t;
    const cz = (sz || 0) + dz * t;
    const offx = n.x - cx;
    const offy = n.y - cy;
    const offz = n.z - cz;
    const dist = Math.sqrt(offx * offx + offy * offy + offz * offz);
    if (dist > sheath.radius) {
        const inv = sheath.radius / dist;
        n.x = cx + offx * inv;
        n.y = cy + offy * inv;
        n.z = cz + offz * inv;
        const vn = n.vx * offx + n.vy * offy + n.vz * offz;
        const rInv = 1 / dist;
        const rx = offx * rInv;
        const ry = offy * rInv;
        const rz = offz * rInv;
        n.vx -= vn * rx;
        n.vy -= vn * ry;
        n.vz -= vn * rz;
    }
}

// Bending stiffness for angular PBD constraint
let bendingStiffness = 0.5;
export function setBendingStiffness(value) {
    bendingStiffness = value;
}

// Parameters controlling the PBD length solver
let pbdIterations = 4;
let lengthTolerance = 0.01;
export function setPbdIterations(value) {
    pbdIterations = value;
}
export function setLengthTolerance(value) {
    lengthTolerance = value;
}

export class Guidewire {
    constructor(
        segLen,
        count,
        start,
        dir,
        vessel,
        initialLength = segLen * (count - 1),
        iterations = pbdIterations,
        tolerance = lengthTolerance,
        initialInsert = 0,
        openEnds = {}
    ) {
        this.segmentLength = segLen;
        this.tailStart = start;
        this.dir = dir;
        this.vessel = vessel;
        this.iterations = iterations;
        this.lengthTolerance = tolerance;
        this.nodes = [];
        this.tailProgress = initialInsert;
        this.openEnds = openEnds;
        for (let i = 0; i < count; i++) {
            const t = this.tailProgress + initialLength - segLen * i;
            const x = start.x + dir.x * t;
            const y = start.y + dir.y * t;
            const z = start.z + dir.z * t;
            this.nodes.push({x, y, z, vx: 0, vy: 0, vz: 0, fx: 0, fy: 0, fz: 0, oldx: x, oldy: y, oldz: z});
        }
        this.maxInsert = this.tailProgress + initialLength;
        this.minInsert = Math.min(this.tailProgress - initialLength, 0);
        this.solvePbd();
    }

    advanceTail(advance, dt) {
        this.tailProgress = clamp(
            this.tailProgress + advance * 40 * dt,
            this.minInsert,
            this.maxInsert
        );
        const tail = this.nodes[this.nodes.length - 1];
        tail.x = this.tailStart.x + this.dir.x * this.tailProgress;
        tail.y = this.tailStart.y + this.dir.y * this.tailProgress;
        tail.z = this.tailStart.z + this.dir.z * this.tailProgress;
        tail.vx = tail.vy = tail.vz = 0;
        if (advance > 0) {
            const tip = this.nodes[0];
            tip.fx += this.dir.x * advanceForce;
            tip.fy += this.dir.y * advanceForce;
            tip.fz += this.dir.z * advanceForce;
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
        // no internal bending forces; bending handled via PBD constraint
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
            n.vx *= velocityDamping;
            n.vy *= velocityDamping;
            n.vz *= velocityDamping;
        }
    }

    // Position Based Dynamics solver enforcing length and bending constraints
    solvePbd() {
        const len = this.segmentLength;
        for (let k = 0; k < this.iterations; k++) {
            let maxError = 0;
            // enforce distance constraints
            for (let i = 1; i < this.nodes.length; i++) {
                const a = this.nodes[i - 1];
                const b = this.nodes[i];
                let dx = b.x - a.x;
                let dy = b.y - a.y;
                let dz = b.z - a.z;
                const dist = Math.sqrt(dx * dx + dy * dy + dz * dz) || 1;
                const diff = (dist - len) / dist;
                maxError = Math.max(maxError, Math.abs(dist - len));
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
            // bending constraints using angular difference between segments
            if (bendingStiffness > 0) {
                for (let i = 1; i < this.nodes.length - 1; i++) {
                    const p0 = this.nodes[i - 1];
                    const p1 = this.nodes[i];
                    const p2 = this.nodes[i + 1];

                    // compute normalized directions of adjacent segments
                    let d0x = p1.x - p0.x;
                    let d0y = p1.y - p0.y;
                    let d0z = p1.z - p0.z;
                    let d1x = p2.x - p1.x;
                    let d1y = p2.y - p1.y;
                    let d1z = p2.z - p1.z;
                    const l0 = Math.sqrt(d0x * d0x + d0y * d0y + d0z * d0z) || 1;
                    const l1 = Math.sqrt(d1x * d1x + d1y * d1y + d1z * d1z) || 1;
                    d0x /= l0; d0y /= l0; d0z /= l0;
                    d1x /= l1; d1y /= l1; d1z /= l1;

                    // angle between segments
                    const dot = clamp(d0x * d1x + d0y * d1y + d0z * d1z, -1, 1);
                    const angle = Math.acos(dot);
                    if (angle === 0) continue;

                    // axis of rotation and its magnitude
                    let ax = d0y * d1z - d0z * d1y;
                    let ay = d0z * d1x - d0x * d1z;
                    let az = d0x * d1y - d0y * d1x;
                    const amag = Math.sqrt(ax * ax + ay * ay + az * az) || 1;
                    ax /= amag; ay /= amag; az /= amag;

                    // gradients for the angle constraint
                    let g0x = (d0y * az - d0z * ay) / l0;
                    let g0y = (d0z * ax - d0x * az) / l0;
                    let g0z = (d0x * ay - d0y * ax) / l0;
                    let g2x = (d1y * az - d1z * ay) / l1;
                    let g2y = (d1z * ax - d1x * az) / l1;
                    let g2z = (d1x * ay - d1y * ax) / l1;
                    let g1x = -g0x - g2x;
                    let g1y = -g0y - g2y;
                    let g1z = -g0z - g2z;

                    const scale = bendingStiffness * angle;
                    p0.x += g0x * scale;
                    p0.y += g0y * scale;
                    p0.z += g0z * scale;
                    p1.x += g1x * scale;
                    p1.y += g1y * scale;
                    p1.z += g1z * scale;
                    if (i + 1 < this.nodes.length - 1) {
                        p2.x += g2x * scale;
                        p2.y += g2y * scale;
                        p2.z += g2z * scale;
                    }
                }
            }
            if (maxError <= this.lengthTolerance) break;
        }
    }

    // Laplacian smoothing to limit sharp kinks
    smooth() {
        for (let i = 1; i < this.nodes.length - 1; i++) {
            const prev = this.nodes[i - 1];
            const curr = this.nodes[i];
            const next = this.nodes[i + 1];
            const avgx = (prev.x + next.x) * 0.5;
            const avgy = (prev.y + next.y) * 0.5;
            const avgz = (prev.z + next.z) * 0.5;
            curr.x += (avgx - curr.x) * 0.5;
            curr.y += (avgy - curr.y) * 0.5;
            curr.z += (avgz - curr.z) * 0.5;
        }
    }

    collide() {
        for (let i = 0; i < this.nodes.length - 1; i++) {
            const n = this.nodes[i];
            if (this.openEnds.left && isBeyond(this.vessel.branchPoint, this.vessel.left.end, n)) {
                clampToSheath(n, this.vessel.sheath);
            } else {
                clampToVessel(n, this.vessel, true, undefined, undefined, undefined, this.openEnds);
            }
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
        this.solvePbd();
        this.smooth();
        this.collide();
        for (let i = 0; i < this.nodes.length - 1; i++) {
            const n = this.nodes[i];
            n.vx = (n.x - n.oldx) / dt;
            n.vy = (n.y - n.oldy) / dt;
            n.vz = (n.z - n.oldz) / dt;
        }
    }
}

