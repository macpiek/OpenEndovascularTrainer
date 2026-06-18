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
let defaultBendingStiffness = 32;
let defaultSmoothingIterations = 0;
let defaultConstraintIterations = 8;

// Coefficients for static and kinetic friction against vessel walls.
// Values are relative to the normal component of velocity.
let wallStaticFriction = 0.006;
let wallKineticFriction = 0.002;

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
const MESH_SEGMENT_CORRECTION_BLEND = 0.92;
const VOLUME_SEGMENT_CORRECTION_BLEND = 0.68;
const WALL_CORRECTION_VELOCITY_DAMPING = 0.94;
const DEFAULT_MESH_COLLISION_PASSES = 2;
const VOLUME_SEGMENT_SAMPLES = [0.25, 0.5, 0.75];
const MESH_COLLIDER_SEGMENT_SAMPLES = [0.25, 0.5, 0.75];

export class ElasticRod {
    constructor(count, segmentLength, {
        mass = 1,
        bendingStiffness = defaultBendingStiffness,
        smoothingIterations = defaultSmoothingIterations,
        constraintIterations = defaultConstraintIterations,
        bendingConstraintIterations = 0,
        bendAngleLimit = 50,
        bendProjectionStrength = 0.35,
        curvatureFlow = 0,
        logger = null,
    } = {}) {
        this.segmentLength = segmentLength;
        this.nodes = [];
        this.smoothingIterations = smoothingIterations;
        this.constraintIterations = constraintIterations;
        this.bendingConstraintIterations = bendingConstraintIterations;
        this.bendAngleLimit = bendAngleLimit;
        this.bendProjectionStrength = bendProjectionStrength;
        this.curvatureFlow = curvatureFlow;
        this.logger = logger;
        this.iteration = 0;
        this.collisionPrevPositions = null;
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

    storeCollisionPreviousPositions() {
        if (!this.collisionPrevPositions || this.collisionPrevPositions.length !== this.nodes.length) {
            this.collisionPrevPositions = this.nodes.map(n => new THREE.Vector3(n.x, n.y, n.z));
            return;
        }

        for (let i = 0; i < this.nodes.length; i++) {
            const n = this.nodes[i];
            this.collisionPrevPositions[i].set(n.x, n.y, n.z);
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

    bendAngleAt(index) {
        const prev = this.nodes[index - 1];
        const curr = this.nodes[index];
        const next = this.nodes[index + 1];
        if (!prev || !curr || !next) return 0;

        const ax = curr.x - prev.x;
        const ay = curr.y - prev.y;
        const az = curr.z - prev.z;
        const bx = next.x - curr.x;
        const by = next.y - curr.y;
        const bz = next.z - curr.z;
        const aLen = Math.hypot(ax, ay, az);
        const bLen = Math.hypot(bx, by, bz);
        if (aLen < 1e-8 || bLen < 1e-8) return 0;

        const dot = (ax * bx + ay * by + az * bz) / (aLen * bLen);
        return Math.acos(Math.max(-1, Math.min(1, dot))) * 180 / Math.PI;
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

    bendAngleAt(index) {
        if (index <= 0 || index >= this.nodes.length - 1) return 0;
        const prev = this.nodes[index - 1];
        const curr = this.nodes[index];
        const next = this.nodes[index + 1];
        const a = new THREE.Vector3(prev.x - curr.x, prev.y - curr.y, prev.z - curr.z).normalize();
        const b = new THREE.Vector3(next.x - curr.x, next.y - curr.y, next.z - curr.z).normalize();
        const internalAngle = Math.acos(THREE.MathUtils.clamp(a.dot(b), -1, 1));
        return 180 - THREE.MathUtils.radToDeg(internalAngle);
    }

    // Solve positional constraints and apply velocity damping
    solveConstraints(dt, options = {}) {
        const L = this.segmentLength;
        const prev = this.nodes.map(n => ({ x: n.x, y: n.y, z: n.z }));
        const applyBending = options.applyBending ?? true;
        const velocityDamping = options.velocityDamping ?? 0.92;

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

        if (applyBending) {
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
                const k = Math.min(1, p1.bendingStiffness * dt + this.curvatureFlow);
                const corrX = dx * k;
                const corrY = dy * k;
                const corrZ = dz * k;
                p1.x -= corrX; p1.y -= corrY; p1.z -= corrZ;
            }

            const bendIterations = options.bendingConstraintIterations ?? this.bendingConstraintIterations;
            if (bendIterations > 0) {
                this.projectBendingConstraints(bendIterations);
                for (let iter = 0; iter < Math.max(2, Math.ceil(this.constraintIterations * 0.5)); iter++) {
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
            }
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
            n.vx = (n.x - prev[i].x) * invDt * velocityDamping;
            n.vy = (n.y - prev[i].y) * invDt * velocityDamping;
            n.vz = (n.z - prev[i].z) * invDt * velocityDamping;
        }
    }

    projectBendingConstraints(iterations = this.bendingConstraintIterations) {
        if (iterations <= 0 || this.nodes.length < 3) return;
        const limit = Math.max(0, this.bendAngleLimit);
        const baseStrength = Math.max(0, Math.min(1, this.bendProjectionStrength));

        for (let iter = 0; iter < iterations; iter++) {
            const corrections = new Array(this.nodes.length);
            for (let i = 1; i < this.nodes.length - 1; i++) {
                const n = this.nodes[i];
                if (n.pinned) continue;
                const angle = this.bendAngleAt(i);
                if (angle <= limit) continue;

                const prev = this.nodes[i - 1];
                const next = this.nodes[i + 1];
                const severity = Math.max(0, Math.min(1, (angle - limit) / Math.max(1, 180 - limit)));
                const strength = baseStrength * (0.35 + 0.65 * severity);
                corrections[i] = {
                    x: ((prev.x + next.x) * 0.5 - n.x) * strength,
                    y: ((prev.y + next.y) * 0.5 - n.y) * strength,
                    z: ((prev.z + next.z) * 0.5 - n.z) * strength
                };
            }

            for (let i = 1; i < this.nodes.length - 1; i++) {
                const n = this.nodes[i];
                const c = corrections[i];
                if (!c || n.pinned) continue;
                n.x += c.x;
                n.y += c.y;
                n.z += c.z;
            }
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
        return this.segmentVolumeContact(n, segments).inside;
    }

    segmentVolumeContact(n, segments) {
        let closestOutside = null;
        for (const seg of segments || []) {
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
            const outside = radialDist - seg.radius;

            if (outside <= 0) {
                return { inside: true, segment: seg, outside, cx, cy, cz, rx, ry, rz, radialDist, rawT };
            }

            if (!closestOutside || outside < closestOutside.outside) {
                closestOutside = { inside: false, segment: seg, outside, cx, cy, cz, rx, ry, rz, radialDist, rawT };
            }
        }
        return closestOutside || { inside: false, outside: Infinity };
    }

    collideWithSegments(n, segments, dt, options = {}) {
        const contact = this.segmentVolumeContact(n, segments);
        if (contact.inside || !Number.isFinite(contact.outside)) return false;
        if (options.localOnly) {
            const band = options.contactBand ?? this.segmentLength * MAX_WALL_CORRECTION_SEGMENTS;
            if (contact.outside > band) return false;
            if (contact.rawT < -SHEATH_ENTRY_TOLERANCE || contact.rawT > 1 + SHEATH_ENTRY_TOLERANCE) {
                return false;
            }
        }

        const invRadial = 1 / (contact.radialDist || 1);
        const nx = contact.rx * invRadial;
        const ny = contact.ry * invRadial;
        const nz = contact.rz * invRadial;
        n.x = contact.cx + nx * contact.segment.radius;
        n.y = contact.cy + ny * contact.segment.radius;
        n.z = contact.cz + nz * contact.segment.radius;
        this.applyWallResponse(n, nx, ny, nz, dt, true);
        return true;
    }

    collideRodSegmentsWithSegments(segments, dt, options = {}) {
        if (!segments?.length) return;
        const samples = options.segmentSamples || VOLUME_SEGMENT_SAMPLES;
        const band = options.contactBand ?? this.segmentLength * MAX_WALL_CORRECTION_SEGMENTS;
        for (let i = 0; i < this.nodes.length - 1; i++) {
            const n0 = this.nodes[i];
            const n1 = this.nodes[i + 1];
            if (n0.pinned && n1.pinned) continue;

            for (const t of samples) {
                const w0 = 1 - t;
                const w1 = t;
                const sample = {
                    x: n0.x * w0 + n1.x * w1,
                    y: n0.y * w0 + n1.y * w1,
                    z: n0.z * w0 + n1.z * w1
                };
                if (this.isPastOpenSheathEntrance(sample, segments)) continue;
                const contact = this.segmentVolumeContact(sample, segments);
                if (contact.inside || !Number.isFinite(contact.outside)) continue;
                if (options.localOnly) {
                    if (contact.outside > band) continue;
                    if (contact.rawT < -SHEATH_ENTRY_TOLERANCE || contact.rawT > 1 + SHEATH_ENTRY_TOLERANCE) {
                        continue;
                    }
                }

                const invRadial = 1 / (contact.radialDist || 1);
                const targetX = contact.cx + contact.rx * invRadial * contact.segment.radius;
                const targetY = contact.cy + contact.ry * invRadial * contact.segment.radius;
                const targetZ = contact.cz + contact.rz * invRadial * contact.segment.radius;
                const corrX = (targetX - sample.x) * VOLUME_SEGMENT_CORRECTION_BLEND;
                const corrY = (targetY - sample.y) * VOLUME_SEGMENT_CORRECTION_BLEND;
                const corrZ = (targetZ - sample.z) * VOLUME_SEGMENT_CORRECTION_BLEND;

                const freeW0 = n0.pinned ? 0 : w0;
                const freeW1 = n1.pinned ? 0 : w1;
                const denom = freeW0 * freeW0 + freeW1 * freeW1;
                if (denom <= 1e-8) continue;

                if (!n0.pinned) {
                    const s = freeW0 / denom;
                    n0.x += corrX * s;
                    n0.y += corrY * s;
                    n0.z += corrZ * s;
                    n0.vx *= WALL_CORRECTION_VELOCITY_DAMPING;
                    n0.vy *= WALL_CORRECTION_VELOCITY_DAMPING;
                    n0.vz *= WALL_CORRECTION_VELOCITY_DAMPING;
                }
                if (!n1.pinned) {
                    const s = freeW1 / denom;
                    n1.x += corrX * s;
                    n1.y += corrY * s;
                    n1.z += corrZ * s;
                    n1.vx *= WALL_CORRECTION_VELOCITY_DAMPING;
                    n1.vy *= WALL_CORRECTION_VELOCITY_DAMPING;
                    n1.vz *= WALL_CORRECTION_VELOCITY_DAMPING;
                }
            }
        }
    }

    collideWithMeshCollider(n, collider, dt, clearance = 0, previous = null) {
        if (previous && collider?.crossingContact) {
            const crossing = collider.crossingContact(previous, n, clearance);
            if (crossing) {
                n.x = crossing.target.x;
                n.y = crossing.target.y;
                n.z = crossing.target.z;
                const normal = crossing.normal || new THREE.Vector3(1, 0, 0);
                this.applyWallResponse(n, normal.x, normal.y, normal.z, dt, true);
                return true;
            }
        }

        const contact = collider?.pointContact?.(n, clearance);
        if (!contact?.violation) return false;
        n.x = contact.target.x;
        n.y = contact.target.y;
        n.z = contact.target.z;
        const normal = contact.normal || new THREE.Vector3(1, 0, 0);
        this.applyWallResponse(n, normal.x, normal.y, normal.z, dt, true);
        return true;
    }

    isPastOpenMeshOutlet(point, options = {}) {
        return Number.isFinite(options.openOutletY) && point.y > options.openOutletY;
    }

    meshContactAtPoint(p, geom, options = {}) {
        const clearance = Math.max(0, options.clearance || 0);
        const closest = new THREE.Vector3();
        const hit = geom.boundsTree.closestPointToPoint(p, { point: closest });
        const dist = hit?.distance ?? p.distanceTo(closest);
        const interior = typeof options.interiorDirection === 'function'
            ? options.interiorDirection(p, closest).clone()
            : new THREE.Vector3().subVectors(p, closest);
        if (interior.lengthSq() < 1e-8) interior.set(1, 0, 0);
        interior.normalize();
        const insideDepth = new THREE.Vector3().subVectors(p, closest).dot(interior);
        return { closest, interior, insideDepth, dist, clearance };
    }

    collideWithMesh(n, geom, dt, options = {}) {
        const p = new THREE.Vector3(n.x, n.y, n.z);
        const contact = this.meshContactAtPoint(p, geom, options);
        const { closest, interior, insideDepth, dist, clearance } = contact;
        const contactBand = Math.max(clearance + this.segmentLength * MAX_WALL_CORRECTION_SEGMENTS, this.segmentLength * 1.5);

        const nx = -interior.x;
        const ny = -interior.y;
        const nz = -interior.z;

        if (insideDepth < clearance) {
            n.x = closest.x + interior.x * clearance;
            n.y = closest.y + interior.y * clearance;
            n.z = closest.z + interior.z * clearance;
            this.applyWallResponse(n, nx, ny, nz, dt, true);
        } else {
            if (dist > contactBand) return;
            const normalForce = Math.max(0, n.fx * nx + n.fy * ny + n.fz * nz);
            const tangentialSpeedSq =
                n.vx * n.vx + n.vy * n.vy + n.vz * n.vz -
                (n.vx * nx + n.vy * ny + n.vz * nz) ** 2;
            if (normalForce > 0 && tangentialSpeedSq > 0) {
                this.applyWallResponse(n, nx, ny, nz, dt, false);
            }
        }
    }

    collideRodSegmentsWithMesh(geom, dt, options = {}, segments = null) {
        const samples = options.segmentSamples || MESH_COLLIDER_SEGMENT_SAMPLES;
        const segmentClearance = Math.max(
            0,
            options.segmentClearance ?? Math.min(options.clearance || 0, this.segmentLength * 0.06)
        );
        for (let i = 0; i < this.nodes.length - 1; i++) {
            const n0 = this.nodes[i];
            const n1 = this.nodes[i + 1];
            if (n0.pinned && n1.pinned) continue;

            for (const t of samples) {
                const w0 = 1 - t;
                const w1 = t;
                const p = new THREE.Vector3(
                    n0.x * w0 + n1.x * w1,
                    n0.y * w0 + n1.y * w1,
                    n0.z * w0 + n1.z * w1
                );
                const sample = { x: p.x, y: p.y, z: p.z };
                if (this.isPastOpenMeshOutlet(sample, options)) continue;
                if (segments) {
                    if (this.isInsideSegmentVolume(sample, segments)) continue;
                    if (this.isPastOpenSheathEntrance(sample, segments)) continue;
                }

                const contact = this.meshContactAtPoint(p, geom, options);
                if (contact.insideDepth >= segmentClearance) continue;

                const targetX = contact.closest.x + contact.interior.x * segmentClearance;
                const targetY = contact.closest.y + contact.interior.y * segmentClearance;
                const targetZ = contact.closest.z + contact.interior.z * segmentClearance;
                const corrX = (targetX - p.x) * MESH_SEGMENT_CORRECTION_BLEND;
                const corrY = (targetY - p.y) * MESH_SEGMENT_CORRECTION_BLEND;
                const corrZ = (targetZ - p.z) * MESH_SEGMENT_CORRECTION_BLEND;

                const freeW0 = n0.pinned ? 0 : w0;
                const freeW1 = n1.pinned ? 0 : w1;
                const denom = freeW0 * freeW0 + freeW1 * freeW1;
                if (denom <= 1e-8) continue;

                if (!n0.pinned) {
                    const s = freeW0 / denom;
                    n0.x += corrX * s;
                    n0.y += corrY * s;
                    n0.z += corrZ * s;
                    n0.vx *= WALL_CORRECTION_VELOCITY_DAMPING;
                    n0.vy *= WALL_CORRECTION_VELOCITY_DAMPING;
                    n0.vz *= WALL_CORRECTION_VELOCITY_DAMPING;
                }
                if (!n1.pinned) {
                    const s = freeW1 / denom;
                    n1.x += corrX * s;
                    n1.y += corrY * s;
                    n1.z += corrZ * s;
                    n1.vx *= WALL_CORRECTION_VELOCITY_DAMPING;
                    n1.vy *= WALL_CORRECTION_VELOCITY_DAMPING;
                    n1.vz *= WALL_CORRECTION_VELOCITY_DAMPING;
                }
            }
        }
    }

    collideRodSegmentsWithMeshCollider(collider, dt, options = {}, segments = null, prevPositions = null) {
        const samples = options.segmentSamples || MESH_COLLIDER_SEGMENT_SAMPLES;
        const segmentClearance = Math.max(
            0,
            options.segmentClearance ?? Math.min(options.clearance || 0, this.segmentLength * 0.08)
        );
        const maxCorrection = this.segmentLength * 4.5;
        const samplePoint = new THREE.Vector3();
        const correction = new THREE.Vector3();
        const startPoint = new THREE.Vector3();
        const endPoint = new THREE.Vector3();
        const applySegmentCorrection = (n0, n1, t, sample, target) => {
            correction.subVectors(target, sample);
            if (correction.length() > maxCorrection) correction.setLength(maxCorrection);
            correction.multiplyScalar(MESH_SEGMENT_CORRECTION_BLEND);

            const w0 = 1 - t;
            const w1 = t;
            const freeW0 = n0.pinned ? 0 : w0;
            const freeW1 = n1.pinned ? 0 : w1;
            const denom = freeW0 * freeW0 + freeW1 * freeW1;
            if (denom <= 1e-8) return;

            if (!n0.pinned) {
                const s = freeW0 / denom;
                n0.x += correction.x * s;
                n0.y += correction.y * s;
                n0.z += correction.z * s;
                n0.vx *= WALL_CORRECTION_VELOCITY_DAMPING;
                n0.vy *= WALL_CORRECTION_VELOCITY_DAMPING;
                n0.vz *= WALL_CORRECTION_VELOCITY_DAMPING;
            }
            if (!n1.pinned) {
                const s = freeW1 / denom;
                n1.x += correction.x * s;
                n1.y += correction.y * s;
                n1.z += correction.z * s;
                n1.vx *= WALL_CORRECTION_VELOCITY_DAMPING;
                n1.vy *= WALL_CORRECTION_VELOCITY_DAMPING;
                n1.vz *= WALL_CORRECTION_VELOCITY_DAMPING;
            }
        };

        for (let i = 0; i < this.nodes.length - 1; i++) {
            const n0 = this.nodes[i];
            const n1 = this.nodes[i + 1];
            if (n0.pinned && n1.pinned) continue;

            if (collider.crossingContact) {
                samplePoint.set(
                    (n0.x + n1.x) * 0.5,
                    (n0.y + n1.y) * 0.5,
                    (n0.z + n1.z) * 0.5
                );
                const sample = { x: samplePoint.x, y: samplePoint.y, z: samplePoint.z };
                if (this.isPastOpenMeshOutlet(sample, options)) continue;
                const skipSegment = segments && (
                    this.isInsideSegmentVolume(sample, segments) ||
                    this.isPastOpenSheathEntrance(sample, segments)
                );
                if (!skipSegment) {
                    startPoint.set(n0.x, n0.y, n0.z);
                    endPoint.set(n1.x, n1.y, n1.z);
                    const crossing = collider.crossingContact(startPoint, endPoint, segmentClearance);
                    if (crossing && crossing.t > 0.03 && crossing.t < 0.97) {
                        applySegmentCorrection(n0, n1, crossing.t, crossing.point, crossing.target);
                    }
                }
            }

            for (const t of samples) {
                const w0 = 1 - t;
                const w1 = t;
                samplePoint.set(
                    n0.x * w0 + n1.x * w1,
                    n0.y * w0 + n1.y * w1,
                    n0.z * w0 + n1.z * w1
                );
                const sample = { x: samplePoint.x, y: samplePoint.y, z: samplePoint.z };
                if (this.isPastOpenMeshOutlet(sample, options)) continue;
                if (segments) {
                    if (this.isInsideSegmentVolume(sample, segments)) continue;
                    if (this.isPastOpenSheathEntrance(sample, segments)) continue;
                }

                const contact = collider.pointContact(samplePoint, segmentClearance);
                if (!contact?.violation) continue;
                applySegmentCorrection(n0, n1, t, samplePoint, contact.target);
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
        const meshCollider = target.meshCollider || target.lumenMeshCollider || null;
        const geom = target.collisionGeometry || (target.isBufferGeometry ? target : (target.geometry || target));
        const prevPositions = this.collisionPrevPositions;
        const meshOptions = {
            clearance: target.guidewireClearance ?? target.collisionClearance ?? target.clearance ?? 0,
            segmentClearance: target.guidewireSegmentClearance ?? target.segmentClearance,
            segmentSamples: target.guidewireSegmentSamples ?? target.segmentSamples,
            openOutletY: target.openOutletY,
            interiorDirection: target.interiorDirection || target.collisionInteriorDirection
        };
        const hasSurfaceCollider = !!meshCollider || !!geom?.boundsTree;
        const segmentOptions = { localOnly: hasSurfaceCollider };
        const collisionPasses = Math.max(
            1,
            target.guidewireCollisionPasses ?? target.collisionPasses ?? (hasSurfaceCollider ? DEFAULT_MESH_COLLISION_PASSES : 4)
        );
        if (!hasSurfaceCollider && !segments) return;
        const collideNodes = () => {
            for (let i = 0; i < this.nodes.length; i++) {
                const n = this.nodes[i];
                if (n.pinned) continue;
                if (segments) {
                    if (this.isInsideSegmentVolume(n, segments)) continue;
                    if (this.isPastOpenSheathEntrance(n, segments)) continue;
                    if (this.collideWithSegments(n, segments, dt, segmentOptions) || !hasSurfaceCollider) continue;
                }
                if (this.isPastOpenMeshOutlet(n, meshOptions)) continue;
                if (meshCollider) {
                    const previous = prevPositions?.[i] || null;
                    this.collideWithMeshCollider(n, meshCollider, dt, meshOptions.clearance, previous);
                } else if (geom && geom.boundsTree) {
                    this.collideWithMesh(n, geom, dt, meshOptions);
                }
            }
        };
        if (meshCollider) {
            for (let pass = 0; pass < collisionPasses; pass++) {
                collideNodes();
                if (segments) this.collideRodSegmentsWithSegments(segments, dt, segmentOptions);
                this.collideRodSegmentsWithMeshCollider(meshCollider, dt, meshOptions, segments, prevPositions);
            }
            collideNodes();
            if (segments) this.collideRodSegmentsWithSegments(segments, dt, segmentOptions);
        } else if (geom && geom.boundsTree) {
            for (let pass = 0; pass < collisionPasses; pass++) {
                collideNodes();
                if (segments) this.collideRodSegmentsWithSegments(segments, dt, segmentOptions);
                this.collideRodSegmentsWithMesh(geom, dt, meshOptions, segments);
            }
            collideNodes();
            if (segments) this.collideRodSegmentsWithSegments(segments, dt, segmentOptions);
        } else {
            for (let pass = 0; pass < collisionPasses; pass++) {
                collideNodes();
                if (segments) this.collideRodSegmentsWithSegments(segments, dt);
            }
        }
        if (this.smoothingIterations > 0) {
            this.laplacianSmooth(dt);
        }
        this.storeCollisionPreviousPositions();
    }

    step(dt) {
        this.storeCollisionPreviousPositions();
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
