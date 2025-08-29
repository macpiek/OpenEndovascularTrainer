/**
 * ElasticRod models a slender elastic rod using a simple discrete formulation.
 *
 * Discretization: The rod is represented by N nodes connected by segments of
 * fixed rest length L. Each node stores position (x,y,z), velocity (vx,vy,vz),
 * accumulated force (fx,fy,fz) and material parameters. Here we lump mass and
 * bending stiffness (EI) at the nodes for simplicity.
 *
 * Bending energy for a rod is approximated as:
 *     E_b = 0.5 * EI * kappa^2 * L
 * where kappa is the curvature magnitude. Curvature is estimated from the
 * change in tangent across a segment:
 *     kappa ≈ |t_{i+1} - t_i| / L
 * with t_i the unit tangent of segment i. The bending moment is then
 *     M = EI * kappa
 * and we apply forces proportional to this curvature difference. This
 * discretization assumes small deflections and uniform material properties.
 * Shear and torsion effects are ignored.
 */

// Default configuration values. These can be overridden from outside the module
// using the exported setter functions below.
let defaultBendingStiffness = 1;
let defaultSmoothingIterations = 0;

// Coefficients for static and kinetic friction against vessel walls.
// Values are relative to the normal component of velocity.
let wallStaticFriction = 0.1;
let wallKineticFriction = 0.05;

export function setBendingStiffness(value) {
    defaultBendingStiffness = value;
}

export function setSmoothingIterations(value) {
    defaultSmoothingIterations = value;
}

export function setWallFriction(staticCoeff, kineticCoeff) {
    wallStaticFriction = staticCoeff;
    wallKineticFriction = kineticCoeff;
}

// Project point n onto vessel segment seg.
// Returns closest point (px,py,pz), offset vector (dx,dy,dz) from projection
// to the node and the distance between them.
function projectOnSegment(n, seg) {
    const vx = seg.end.x - seg.start.x;
    const vy = seg.end.y - seg.start.y;
    const vz = (seg.end.z || 0) - (seg.start.z || 0);
    const wx = n.x - seg.start.x;
    const wy = n.y - seg.start.y;
    const wz = n.z - (seg.start.z || 0);
    const len2 = vx * vx + vy * vy + vz * vz;
    let t = (wx * vx + wy * vy + wz * vz) / len2;
    t = Math.max(0, Math.min(1, t));
    const px = seg.start.x + vx * t;
    const py = seg.start.y + vy * t;
    const pz = (seg.start.z || 0) + vz * t;
    const dx = n.x - px;
    const dy = n.y - py;
    const dz = n.z - pz;
    const dist = Math.sqrt(dx * dx + dy * dy + dz * dz);
    return { px, py, pz, dx, dy, dz, dist };
}

export class ElasticRod {
    constructor(count, segmentLength, {
        mass = 1,
        bendingStiffness = defaultBendingStiffness,
        smoothingIterations = defaultSmoothingIterations,
    } = {}) {
        this.segmentLength = segmentLength;
        this.nodes = [];
        this.smoothingIterations = smoothingIterations;
        for (let i = 0; i < count; i++) {
            const x = i * segmentLength;
            const y = 0, z = 0;
            this.nodes.push({
                x, y, z,
                vx: 0, vy: 0, vz: 0,
                fx: 0, fy: 0, fz: 0,
                mass,
                bendingStiffness,
                kx: 0, ky: 0, kz: 0,
            });
        }
    }

    resetForces() {
        for (const n of this.nodes) {
            n.fx = n.fy = n.fz = 0;
        }
    }

    // Compute discrete curvature vector for each interior node using
    // a second derivative approximation along the rod.
    updateCurvature() {
        const L2 = this.segmentLength * this.segmentLength;
        // reset curvature
        for (const n of this.nodes) {
            n.kx = n.ky = n.kz = 0;
        }
        for (let i = 1; i < this.nodes.length - 1; i++) {
            const p0 = this.nodes[i - 1];
            const p1 = this.nodes[i];
            const p2 = this.nodes[i + 1];
            p1.kx = (p0.x - 2 * p1.x + p2.x) / L2;
            p1.ky = (p0.y - 2 * p1.y + p2.y) / L2;
            p1.kz = (p0.z - 2 * p1.z + p2.z) / L2;
        }
    }

    // Accumulate bending forces that attempt to straighten the rod
    // proportional to the curvature vector.
    accumulateBendingForces() {
        for (const n of this.nodes) {
            const EI = n.bendingStiffness;
            n.fx -= EI * n.kx;
            n.fy -= EI * n.ky;
            n.fz -= EI * n.kz;
        }
    }

    // Integrate positions and velocities using semi-implicit Euler
    integrate(dt) {
        for (const n of this.nodes) {
            const ax = n.fx / n.mass;
            const ay = n.fy / n.mass;
            const az = n.fz / n.mass;
            n.vx += ax * dt;
            n.vy += ay * dt;
            n.vz += az * dt;
            n.x += n.vx * dt;
            n.y += n.vy * dt;
            n.z += n.vz * dt;
        }
    }

    // Solve positional constraints and apply velocity damping
    solveConstraints(dt) {
        const L = this.segmentLength;

        // enforce segment lengths
        for (let i = 0; i < this.nodes.length - 1; i++) {
            const n0 = this.nodes[i];
            const n1 = this.nodes[i + 1];
            let dx = n1.x - n0.x;
            let dy = n1.y - n0.y;
            let dz = n1.z - n0.z;
            let dist = Math.hypot(dx, dy, dz);
            if (!dist) continue;
            const diff = (dist - L) / dist * 0.5;
            dx *= diff; dy *= diff; dz *= diff;
            n0.x += dx; n0.y += dy; n0.z += dz;
            n1.x -= dx; n1.y -= dy; n1.z -= dz;

            // update velocities from positional corrections
            const invDt = 1 / dt;
            n0.vx += dx * invDt; n0.vy += dy * invDt; n0.vz += dz * invDt;
            n1.vx -= dx * invDt; n1.vy -= dy * invDt; n1.vz -= dz * invDt;
        }

        // simple bending constraint: pull interior nodes toward midpoint of neighbours
        for (let i = 1; i < this.nodes.length - 1; i++) {
            const p0 = this.nodes[i - 1];
            const p1 = this.nodes[i];
            const p2 = this.nodes[i + 1];
            const cx = (p0.x + p2.x) * 0.5;
            const cy = (p0.y + p2.y) * 0.5;
            const cz = (p0.z + p2.z) * 0.5;
            const dx = p1.x - cx;
            const dy = p1.y - cy;
            const dz = p1.z - cz;
            const k = Math.min(1, p1.bendingStiffness * dt);
            const corrX = dx * k;
            const corrY = dy * k;
            const corrZ = dz * k;
            p1.x -= corrX; p1.y -= corrY; p1.z -= corrZ;
            p1.vx -= corrX / dt; p1.vy -= corrY / dt; p1.vz -= corrZ / dt;
        }

        // velocity damping
        for (const n of this.nodes) {
            n.vx *= 0.98;
            n.vy *= 0.98;
            n.vz *= 0.98;
        }

        // optional Laplacian smoothing after constraints
        if (this.smoothingIterations > 0) {
            this.laplacianSmooth(dt);
        }
    }

    // Simple Laplacian smoothing applied to interior nodes.
    laplacianSmooth(dt) {
        const count = this.nodes.length;
        if (count < 3) return;
        for (let iter = 0; iter < this.smoothingIterations; iter++) {
            const newPos = new Array(count);
            for (let i = 1; i < count - 1; i++) {
                const p0 = this.nodes[i - 1];
                const p2 = this.nodes[i + 1];
                newPos[i] = {
                    x: (p0.x + p2.x) * 0.5,
                    y: (p0.y + p2.y) * 0.5,
                    z: (p0.z + p2.z) * 0.5,
                };
            }
            for (let i = 1; i < count - 1; i++) {
                const n = this.nodes[i];
                const np = newPos[i];
                const dx = np.x - n.x;
                const dy = np.y - n.y;
                const dz = np.z - n.z;
                n.x = np.x; n.y = np.y; n.z = np.z;
                n.vx += dx / dt; n.vy += dy / dt; n.vz += dz / dt;
            }
        }
    }

    // Constrain nodes to stay inside the vessel geometry.
    collide(vessel, dt = 1) {
        if (!vessel || !vessel.segments || !vessel.segments.length) return;
        for (const n of this.nodes) {
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
            const radius = nearest.radius;
            if (best.dist > radius) {
                const inv = 1 / (best.dist || 1);
                const nx = best.dx * inv;
                const ny = best.dy * inv;
                const nz = best.dz * inv;
                n.x = best.px + nx * radius;
                n.y = best.py + ny * radius;
                n.z = best.pz + nz * radius;
                const vn = n.vx * nx + n.vy * ny + n.vz * nz;
                let tx = n.vx - vn * nx;
                let ty = n.vy - vn * ny;
                let tz = n.vz - vn * nz;
                const tMag = Math.sqrt(tx * tx + ty * ty + tz * tz);
                const normalMag = Math.abs(vn);
                if (tMag < wallStaticFriction * normalMag) {
                    tx = 0; ty = 0; tz = 0;
                } else {
                    const frictionMag = wallKineticFriction * normalMag;
                    const scale = Math.max(0, tMag - frictionMag) / (tMag || 1);
                    tx *= scale;
                    ty *= scale;
                    tz *= scale;
                }
                n.vx = tx;
                n.vy = ty;
                n.vz = tz;
            }
        }
        if (this.smoothingIterations > 0) {
            this.laplacianSmooth(dt);
        }
    }

    step(dt) {
        this.resetForces();
        this.updateCurvature();
        this.accumulateBendingForces();
        this.integrate(dt);
        this.solveConstraints(dt);
    }
}
