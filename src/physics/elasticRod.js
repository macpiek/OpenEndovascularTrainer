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
let defaultConstraintIterations = 8;

// Coefficients for static and kinetic friction against vessel walls.
// Values are relative to the normal component of velocity.
let wallStaticFriction = 0.03;
let wallKineticFriction = 0.01;

export function setBendingStiffness(value) {
    defaultBendingStiffness = value;
}

export function setSmoothingIterations(value) {
    defaultSmoothingIterations = value;
}

export function setConstraintIterations(value) {
    defaultConstraintIterations = value;
}

export function setWallFriction(staticCoeff, kineticCoeff) {
    wallStaticFriction = staticCoeff;
    wallKineticFriction = kineticCoeff;
}

const SHEATH_ENTRY_TOLERANCE = 1e-4;
const MAX_WALL_CORRECTION_SEGMENTS = 1.25;

export class ElasticRod {
    constructor(count, segmentLength, {
        mass = 1,
        bendingStiffness = defaultBendingStiffness,
        smoothingIterations = defaultSmoothingIterations,
        constraintIterations = defaultConstraintIterations,
        logger = null,
    } = {}) {
        this.segmentLength = segmentLength;
        this.nodes = [];
        this.smoothingIterations = smoothingIterations;
        this.constraintIterations = constraintIterations;
        this.logger = logger;
        this.iteration = 0;
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
        const prev = this.nodes.map(n => ({ x: n.x, y: n.y, z: n.z }));

        // enforce segment lengths
        for (let iter = 0; iter < this.constraintIterations; iter++) {
            for (let i = 0; i < this.nodes.length - 1; i++) {
                const n0 = this.nodes[i];
                const n1 = this.nodes[i + 1];
                let dx = n1.x - n0.x;
                let dy = n1.y - n0.y;
                let dz = n1.z - n0.z;
                let dist = Math.hypot(dx, dy, dz);
                if (!dist) continue;
                const diff = (dist - L) / dist;
                if (n0.pinned && n1.pinned) continue;
                if (n0.pinned) {
                    dx *= diff; dy *= diff; dz *= diff;
                    n1.x -= dx; n1.y -= dy; n1.z -= dz;
                } else if (n1.pinned) {
                    dx *= diff; dy *= diff; dz *= diff;
                    n0.x += dx; n0.y += dy; n0.z += dz;
                } else {
                    dx *= diff * 0.5; dy *= diff * 0.5; dz *= diff * 0.5;
                    n0.x += dx; n0.y += dy; n0.z += dz;
                    n1.x -= dx; n1.y -= dy; n1.z -= dz;
                }
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
        }

        // optional Laplacian smoothing after constraints
        if (this.smoothingIterations > 0) {
            this.laplacianSmooth();
        }

        const invDt = 1 / dt;
        for (let i = 0; i < this.nodes.length; i++) {
            const n = this.nodes[i];
            if (n.pinned) {
                n.vx = n.vy = n.vz = 0;
                continue;
            }
            n.vx = (n.x - prev[i].x) * invDt * 0.92;
            n.vy = (n.y - prev[i].y) * invDt * 0.92;
            n.vz = (n.z - prev[i].z) * invDt * 0.92;
        }
    }

    // Simple Laplacian smoothing applied to interior nodes.
    laplacianSmooth() {
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
                n.x = np.x; n.y = np.y; n.z = np.z;
            }
        }
    }

    straightenByTension(strength = 0.2, iterations = 1) {
        const count = this.nodes.length;
        if (count < 3 || strength <= 0 || iterations <= 0) return;
        const alpha = Math.max(0, Math.min(1, strength));
        for (let iter = 0; iter < iterations; iter++) {
            const corrections = new Array(count);
            for (let i = 1; i < count - 1; i++) {
                const n = this.nodes[i];
                if (n.pinned) continue;
                const prev = this.nodes[i - 1];
                const next = this.nodes[i + 1];
                corrections[i] = {
                    x: ((prev.x + next.x) * 0.5 - n.x) * alpha,
                    y: ((prev.y + next.y) * 0.5 - n.y) * alpha,
                    z: ((prev.z + next.z) * 0.5 - n.z) * alpha,
                };
            }
            for (let i = 1; i < count - 1; i++) {
                const n = this.nodes[i];
                const c = corrections[i];
                if (!c || n.pinned) continue;
                n.x += c.x;
                n.y += c.y;
                n.z += c.z;
                n.vx *= 0.65;
                n.vy *= 0.65;
                n.vz *= 0.65;
            }
        }
    }

    releaseFromVesselWall(segments, strength = 0.1, iterations = 1) {
        if (!segments || strength <= 0 || iterations <= 0) return;
        const alpha = Math.max(0, Math.min(1, strength));
        for (let iter = 0; iter < iterations; iter++) {
            for (const n of this.nodes) {
                if (n.pinned) continue;
                let best = null;
                for (const seg of segments) {
                    if (seg.isSheath) continue;
                    const ax = seg.end.x - seg.start.x;
                    const ay = seg.end.y - seg.start.y;
                    const az = seg.end.z - seg.start.z;
                    const lenSq = ax * ax + ay * ay + az * az;
                    if (!lenSq) continue;

                    const px = n.x - seg.start.x;
                    const py = n.y - seg.start.y;
                    const pz = n.z - seg.start.z;
                    const rawT = (px * ax + py * ay + pz * az) / lenSq;
                    const t = Math.max(0, Math.min(1, rawT));
                    const cx = seg.start.x + ax * t;
                    const cy = seg.start.y + ay * t;
                    const cz = seg.start.z + az * t;
                    const rx = n.x - cx;
                    const ry = n.y - cy;
                    const rz = n.z - cz;
                    const radialDist = Math.hypot(rx, ry, rz);
                    const normalized = radialDist / (seg.radius || 1);
                    const distanceScore = Math.abs(normalized - 0.75) + Math.max(0, Math.abs(rawT - 0.5) - 0.5);
                    if (normalized <= 1.25 && (!best || distanceScore < best.distanceScore)) {
                        best = { cx, cy, cz, radialDist, normalized, distanceScore };
                    }
                }
                if (!best || best.radialDist <= this.segmentLength * 0.15) continue;
                const wallBias = Math.max(0, Math.min(1, (best.normalized - 0.25) / 0.75));
                const amount = alpha * wallBias;
                n.x += (best.cx - n.x) * amount;
                n.y += (best.cy - n.y) * amount;
                n.z += (best.cz - n.z) * amount;
                n.vx *= 0.75;
                n.vy *= 0.75;
                n.vz *= 0.75;
            }
        }
    }

    applyWallResponse(n, nx, ny, nz, dt, includeImpact) {
        const vn = n.vx * nx + n.vy * ny + n.vz * nz;
        let tx = n.vx - vn * nx;
        let ty = n.vy - vn * ny;
        let tz = n.vz - vn * nz;
        const tMag = Math.sqrt(tx * tx + ty * ty + tz * tz);
        const impactForce = includeImpact ? Math.abs(vn) * n.mass / dt : 0;
        const normalForce = Math.max(0, n.fx * nx + n.fy * ny + n.fz * nz) + impactForce;

        if (normalForce > 0 && tMag > 0) {
            const staticLimit = wallStaticFriction * normalForce * dt / n.mass;
            const kineticLoss = wallKineticFriction * normalForce * dt / n.mass;
            if (tMag <= staticLimit) {
                tx = 0; ty = 0; tz = 0;
            } else {
                const scale = Math.max(0, tMag - kineticLoss) / (tMag || 1);
                tx *= scale; ty *= scale; tz *= scale;
            }
        }

        n.vx = tx; n.vy = ty; n.vz = tz;
    }

    isPastOpenSheathEntrance(n, segments) {
        for (const seg of segments) {
            if (!seg.isSheath) continue;
            const ax = seg.end.x - seg.start.x;
            const ay = seg.end.y - seg.start.y;
            const az = seg.end.z - seg.start.z;
            const lenSq = ax * ax + ay * ay + az * az;
            if (!lenSq) continue;

            const px = n.x - seg.start.x;
            const py = n.y - seg.start.y;
            const pz = n.z - seg.start.z;
            const t = (px * ax + py * ay + pz * az) / lenSq;
            if (t >= -SHEATH_ENTRY_TOLERANCE) continue;

            const radialX = px - ax * t;
            const radialY = py - ay * t;
            const radialZ = pz - az * t;
            const radialDist = Math.hypot(radialX, radialY, radialZ);
            if (radialDist <= seg.radius + this.segmentLength) return true;
        }
        return false;
    }

    isInsideSegmentVolume(n, segments) {
        for (const seg of segments) {
            const ax = seg.end.x - seg.start.x;
            const ay = seg.end.y - seg.start.y;
            const az = seg.end.z - seg.start.z;
            const lenSq = ax * ax + ay * ay + az * az;
            if (!lenSq) continue;

            const px = n.x - seg.start.x;
            const py = n.y - seg.start.y;
            const pz = n.z - seg.start.z;
            const t = (px * ax + py * ay + pz * az) / lenSq;
            if (t >= 0 && t <= 1) {
                const radialX = px - ax * t;
                const radialY = py - ay * t;
                const radialZ = pz - az * t;
                if (Math.hypot(radialX, radialY, radialZ) <= seg.radius) return true;
            }

            if (!seg.isSheath) {
                const startDist = Math.hypot(px, py, pz);
                if (startDist <= seg.radius) return true;
            }
            const endDist = Math.hypot(n.x - seg.end.x, n.y - seg.end.y, n.z - seg.end.z);
            if (endDist <= seg.radius) return true;
        }
        return false;
    }

    collideWithSegments(n, segments, dt) {
        let best = null;
        for (const seg of segments) {
            const ax = seg.end.x - seg.start.x;
            const ay = seg.end.y - seg.start.y;
            const az = seg.end.z - seg.start.z;
            const lenSq = ax * ax + ay * ay + az * az;
            if (!lenSq) continue;

            const px = n.x - seg.start.x;
            const py = n.y - seg.start.y;
            const pz = n.z - seg.start.z;
            const t = (px * ax + py * ay + pz * az) / lenSq;
            if (t < 0 || t > 1) continue;

            const cx = seg.start.x + ax * t;
            const cy = seg.start.y + ay * t;
            const cz = seg.start.z + az * t;
            const rx = n.x - cx;
            const ry = n.y - cy;
            const rz = n.z - cz;
            const radialDist = Math.hypot(rx, ry, rz);
            const penetration = radialDist - seg.radius;
            if (penetration <= 0) continue;
            if (penetration > this.segmentLength * MAX_WALL_CORRECTION_SEGMENTS) continue;
            if (!best || penetration < best.penetration) {
                best = { cx, cy, cz, rx, ry, rz, radialDist, radius: seg.radius, penetration };
            }
        }

        if (!best) return false;

        const invRadial = 1 / (best.radialDist || 1);
        const nx = best.rx * invRadial;
        const ny = best.ry * invRadial;
        const nz = best.rz * invRadial;
        n.x = best.cx + nx * best.radius;
        n.y = best.cy + ny * best.radius;
        n.z = best.cz + nz * best.radius;
        this.applyWallResponse(n, nx, ny, nz, dt, true);
        return true;
    }

    collideWithMesh(n, geom, dt) {
        const p = new THREE.Vector3(n.x, n.y, n.z);
        const closest = new THREE.Vector3();
        const normal = new THREE.Vector3();
        geom.boundsTree.closestPointToPoint(p, { point: closest, normal });
        const dx = p.x - closest.x;
        const dy = p.y - closest.y;
        const dz = p.z - closest.z;
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
            n.x = closest.x;
            n.y = closest.y;
            n.z = closest.z;
            this.applyWallResponse(n, nx, ny, nz, dt, true);
        } else {
            const normalForce = Math.max(0, n.fx * nx + n.fy * ny + n.fz * nz);
            const tangentialSpeedSq =
                n.vx * n.vx + n.vy * n.vy + n.vz * n.vz -
                (n.vx * nx + n.vy * ny + n.vz * nz) ** 2;
            if (normalForce > 0 && tangentialSpeedSq > 0) {
                this.applyWallResponse(n, nx, ny, nz, dt, false);
            }
        }
    }

    // Constrain nodes to stay inside the vessel while leaving the outer sheath
    // entrance open so the unsimulated wire length can remain outside the body.
    // Accepts either a THREE.BufferGeometry, a THREE.Mesh, or any object with
    // a `geometry` property containing a BufferGeometry with a boundsTree.
    collide(target, dt = 1) {
        if (!target) return;
        const segments = target.segments || null;
        const geom = target.isBufferGeometry ? target : (target.geometry || target);
        if ((!geom || !geom.boundsTree) && !segments) return;
        for (const n of this.nodes) {
            if (n.pinned) continue;
            if (segments) {
                if (this.isInsideSegmentVolume(n, segments)) continue;
                if (this.isPastOpenSheathEntrance(n, segments)) continue;
                this.collideWithSegments(n, segments, dt);
                continue;
            }
            if (geom && geom.boundsTree) this.collideWithMesh(n, geom, dt);
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
        this.iteration++;
        if (this.logger) {
            this.logger({
                iteration: this.iteration,
                curvature: this.averageCurvature(),
                length: this.computeLength(),
            });
        }
    }
}
