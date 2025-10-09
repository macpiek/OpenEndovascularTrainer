import * as THREE from 'three';

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
 * and we apply forces proportional to this curvature difference. A nonlinear
 * factor (1 + kappa^2) further amplifies the response in highly bent regions,
 * producing stronger self-straightening when curvature grows large. This
 * discretization assumes small deflections and uniform material properties.
 * Shear and torsion effects are ignored.
*/

// Default configuration values. These can be overridden from outside the module
// using the exported setter functions below.
// higher default stiffness gives stronger self-straightening
let defaultBendingStiffness = 20;
let defaultSmoothingIterations = 0;
let defaultInternalDamping = 2.5;
let defaultVelocityDamping = 4;

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

export function setInternalDamping(value) {
    defaultInternalDamping = value;
}

export function setVelocityDamping(value) {
    defaultVelocityDamping = value;
}

export function setWallFriction(staticCoeff, kineticCoeff) {
    wallStaticFriction = staticCoeff;
    wallKineticFriction = kineticCoeff;
}

// Project point n onto vessel segment seg.
// Returns closest point (px,py,pz), offset vector (dx,dy,dz) from projection
// to the node and the distance between them. Also returns the unclamped
// segment parameter t before restricting it to [0,1] so callers can detect
// if the node lies beyond an endpoint.
function projectOnSegment(n, seg) {
    const vx = seg.end.x - seg.start.x;
    const vy = seg.end.y - seg.start.y;
    const vz = (seg.end.z || 0) - (seg.start.z || 0);
    const wx = n.x - seg.start.x;
    const wy = n.y - seg.start.y;
    const wz = n.z - (seg.start.z || 0);
    const len2 = vx * vx + vy * vy + vz * vz;
    const tRaw = (wx * vx + wy * vy + wz * vz) / len2;
    const t = Math.max(0, Math.min(1, tRaw));
    const px = seg.start.x + vx * t;
    const py = seg.start.y + vy * t;
    const pz = (seg.start.z || 0) + vz * t;
    const dx = n.x - px;
    const dy = n.y - py;
    const dz = n.z - pz;
    const dist = Math.sqrt(dx * dx + dy * dy + dz * dz);
    return { px, py, pz, dx, dy, dz, dist, t: tRaw };
}

export class ElasticRod {
    constructor(count, segmentLength, {
        mass = 1,
        bendingStiffness = defaultBendingStiffness,
        smoothingIterations = defaultSmoothingIterations,
        logger = null,
        internalDamping = defaultInternalDamping,
        velocityDamping = defaultVelocityDamping,
    } = {}) {
        this.segmentLength = segmentLength;
        this.nodes = [];
        this.smoothingIterations = smoothingIterations;
        this.logger = logger;
        this.iteration = 0;
        this.internalDamping = internalDamping;
        this.velocityDamping = velocityDamping;
        this.energy = { kinetic: 0, potential: 0, total: 0 };
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
                pinned: false,
            });
        }
    }

    computeLength() {
        let total = 0;
        for (let i = 0; i < this.nodes.length - 1; i++) {
            const n0 = this.nodes[i];
            const n1 = this.nodes[i + 1];
            total += Math.hypot(n1.x - n0.x, n1.y - n0.y, n1.z - n0.z);
        }
        return total;
    }

    averageCurvature() {
        let sum = 0;
        for (const n of this.nodes) {
            sum += Math.hypot(n.kx, n.ky, n.kz);
        }
        return sum / this.nodes.length;
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

    // Accumulate bending forces that attempt to straighten the rod.
    // Forces are spread across a five-node stencil so curvature changes
    // propagate smoothly rather than concentrating at a single joint.
    accumulateBendingForces() {
        const count = this.nodes.length;
        if (count < 3) return;
        for (let i = 1; i < count - 1; i++) {
            const prev = this.nodes[i - 1];
            const curr = this.nodes[i];
            const next = this.nodes[i + 1];
            const prev2 = this.nodes[i - 2];
            const next2 = this.nodes[i + 2];
            const kx = curr.kx;
            const ky = curr.ky;
            const kz = curr.kz;

            const EI = curr.bendingStiffness;
            // amplify straightening force when curvature is large
            const k2 = kx * kx + ky * ky + kz * kz;
            const scale = 1 + k2; // nonlinear scaling for strong bends
            const fx = EI * kx * scale;
            const fy = EI * ky * scale;
            const fz = EI * kz * scale;

            // weights for immediate and next-nearest neighbours
            const w1 = 0.4;
            const w2 = 0.1;
            if (prev2) {
                prev2.fx += w2 * fx; prev2.fy += w2 * fy; prev2.fz += w2 * fz;
            }
            if (next2) {
                next2.fx += w2 * fx; next2.fy += w2 * fy; next2.fz += w2 * fz;
            }
            prev.fx += w1 * fx; prev.fy += w1 * fy; prev.fz += w1 * fz;
            next.fx += w1 * fx; next.fy += w1 * fy; next.fz += w1 * fz;

            const sumWeights = w1 * 2 + (prev2 ? w2 : 0) + (next2 ? w2 : 0);
            curr.fx -= sumWeights * fx;
            curr.fy -= sumWeights * fy;
            curr.fz -= sumWeights * fz;
        }
    }

    applyInternalDamping() {
        if (!this.internalDamping) return;
        const damping = this.internalDamping;
        for (const n of this.nodes) {
            if (n.pinned) continue;
            const mass = n.mass;
            n.fx -= damping * mass * n.vx;
            n.fy -= damping * mass * n.vy;
            n.fz -= damping * mass * n.vz;
        }
    }

    // Integrate positions and velocities using semi-implicit Euler
    integrate(dt) {
        for (const n of this.nodes) {
            if (n.pinned) {
                n.vx = n.vy = n.vz = 0;
                continue;
            }
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
            const diff = (dist - L) / dist;
            const invDt = 1 / dt;
            if (n0.pinned && n1.pinned) continue;
            if (n0.pinned) {
                dx *= diff; dy *= diff; dz *= diff;
                n1.x -= dx; n1.y -= dy; n1.z -= dz;
                n1.vx -= dx * invDt; n1.vy -= dy * invDt; n1.vz -= dz * invDt;
            } else if (n1.pinned) {
                dx *= diff; dy *= diff; dz *= diff;
                n0.x += dx; n0.y += dy; n0.z += dz;
                n0.vx += dx * invDt; n0.vy += dy * invDt; n0.vz += dz * invDt;
            } else {
                dx *= diff * 0.5; dy *= diff * 0.5; dz *= diff * 0.5;
                n0.x += dx; n0.y += dy; n0.z += dz;
                n1.x -= dx; n1.y -= dy; n1.z -= dz;
                n0.vx += dx * invDt; n0.vy += dy * invDt; n0.vz += dz * invDt;
                n1.vx -= dx * invDt; n1.vy -= dy * invDt; n1.vz -= dz * invDt;
            }
        }

        // simple bending constraint: pull interior nodes toward midpoint of neighbours
        for (let i = 1; i < this.nodes.length - 1; i++) {
            const p0 = this.nodes[i - 1];
            const p1 = this.nodes[i];
            const p2 = this.nodes[i + 1];
            if (p1.pinned) continue;
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
        const damping = Math.exp(-Math.max(0, this.velocityDamping) * dt);
        for (const n of this.nodes) {
            if (n.pinned) continue;
            n.vx *= damping;
            n.vy *= damping;
            n.vz *= damping;
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
                const n = this.nodes[i];
                if (n.pinned) {
                    newPos[i] = { x: n.x, y: n.y, z: n.z };
                    continue;
                }
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
                if (n.pinned) continue;
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
        if (!vessel) return;
        const geom = vessel.geometry;
        const sheath = vessel.segments && vessel.segments.find(s => s.isSheath);
        if (!geom || !geom.boundsTree) return;
        const p = new THREE.Vector3();
        const target = new THREE.Vector3();
        const normal = new THREE.Vector3();
        for (const n of this.nodes) {
            if (n.pinned) continue;
            if (sheath) {
                const shProj = projectOnSegment(n, sheath);
                const radial = Math.sqrt(shProj.dx * shProj.dx + shProj.dy * shProj.dy + shProj.dz * shProj.dz);
                if (shProj.t < 0 && radial <= sheath.radius) continue;
            }
            p.set(n.x, n.y, n.z);
            geom.boundsTree.closestPointToPoint(p, { point: target, normal });
            const dx = p.x - target.x;
            const dy = p.y - target.y;
            const dz = p.z - target.z;
            const dist = Math.sqrt(dx * dx + dy * dy + dz * dz);
            if (normal.lengthSq() === 0) {
                const inv = 1 / (dist || 1);
                normal.set(dx * inv, dy * inv, dz * inv);
            }
            const dot = dx * normal.x + dy * normal.y + dz * normal.z;
            const penetration = dot > 0 ? dist : -dist;
            const nx = normal.x;
            const ny = normal.y;
            const nz = normal.z;
            if (penetration > 0) {
                n.x = target.x;
                n.y = target.y;
                n.z = target.z;
                const vn = n.vx * nx + n.vy * ny + n.vz * nz;
                let tx = n.vx - vn * nx;
                let ty = n.vy - vn * ny;
                let tz = n.vz - vn * nz;
                const tMag = Math.sqrt(tx * tx + ty * ty + tz * tz);
                const normalForce = Math.max(0, n.fx * nx + n.fy * ny + n.fz * nz) + Math.abs(vn) * n.mass / dt;
                const staticLimit = wallStaticFriction * normalForce * dt / n.mass;
                const kineticLoss = wallKineticFriction * normalForce * dt / n.mass;
                if (tMag <= staticLimit) {
                    tx = 0; ty = 0; tz = 0;
                } else {
                    const scale = Math.max(0, tMag - kineticLoss) / (tMag || 1);
                    tx *= scale; ty *= scale; tz *= scale;
                }
                n.vx = tx; n.vy = ty; n.vz = tz;
            } else {
                const vn = n.vx * nx + n.vy * ny + n.vz * nz;
                let tx = n.vx - vn * nx;
                let ty = n.vy - vn * ny;
                let tz = n.vz - vn * nz;
                const tMag = Math.sqrt(tx * tx + ty * ty + tz * tz);
                const normalForce = Math.max(0, n.fx * nx + n.fy * ny + n.fz * nz);
                if (normalForce > 0 && tMag > 0) {
                    const staticLimit = wallStaticFriction * normalForce * dt / n.mass;
                    const kineticLoss = wallKineticFriction * normalForce * dt / n.mass;
                    if (tMag <= staticLimit) {
                        tx = 0; ty = 0; tz = 0;
                    } else {
                        const scale = Math.max(0, tMag - kineticLoss) / (tMag || 1);
                        tx *= scale; ty *= scale; tz *= scale;
                    }
                    n.vx = tx; n.vy = ty; n.vz = tz;
                }
            }
        }
        if (this.smoothingIterations > 0) {
            this.laplacianSmooth(dt);
        }
    }

    computeKineticEnergy() {
        let sum = 0;
        for (const n of this.nodes) {
            if (n.pinned) continue;
            const v2 = n.vx * n.vx + n.vy * n.vy + n.vz * n.vz;
            sum += 0.5 * n.mass * v2;
        }
        return sum;
    }

    computePotentialEnergy() {
        let sum = 0;
        const L = this.segmentLength;
        if (this.nodes.length < 3) return sum;
        for (let i = 1; i < this.nodes.length - 1; i++) {
            const n = this.nodes[i];
            const kx = n.kx;
            const ky = n.ky;
            const kz = n.kz;
            const k2 = kx * kx + ky * ky + kz * kz;
            sum += 0.5 * n.bendingStiffness * k2 * L;
        }
        return sum;
    }

    computeEnergies() {
        const kinetic = this.computeKineticEnergy();
        const potential = this.computePotentialEnergy();
        return { kinetic, potential, total: kinetic + potential };
    }

    step(dt) {
        this.resetForces();
        this.updateCurvature();
        this.accumulateBendingForces();
        this.applyInternalDamping();
        this.integrate(dt);
        this.solveConstraints(dt);
        this.updateCurvature();
        this.energy = this.computeEnergies();
        this.iteration++;
        if (this.logger) {
            this.logger({
                iteration: this.iteration,
                curvature: this.averageCurvature(),
                length: this.computeLength(),
                kineticEnergy: this.energy.kinetic,
                potentialEnergy: this.energy.potential,
                totalEnergy: this.energy.total,
            });
        }
    }
}
