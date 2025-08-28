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

export class ElasticRod {
    constructor(count, segmentLength, {
        mass = 1,
        bendingStiffness = 1,
    } = {}) {
        this.segmentLength = segmentLength;
        this.nodes = [];
        for (let i = 0; i < count; i++) {
            const x = i * segmentLength;
            const y = 0, z = 0;
            this.nodes.push({
                x, y, z,
                vx: 0, vy: 0, vz: 0,
                fx: 0, fy: 0, fz: 0,
                mass,
                bendingStiffness,
            });
        }
    }

    resetForces() {
        for (const n of this.nodes) {
            n.fx = n.fy = n.fz = 0;
        }
    }

    // Accumulate bending forces based on discrete curvature
    accumulateBendingForces() {
        const L = this.segmentLength;
        for (let i = 1; i < this.nodes.length - 1; i++) {
            const p0 = this.nodes[i - 1];
            const p1 = this.nodes[i];
            const p2 = this.nodes[i + 1];

            // unit tangent vectors for adjacent segments
            let t0x = p1.x - p0.x;
            let t0y = p1.y - p0.y;
            let t0z = p1.z - p0.z;
            let len0 = Math.hypot(t0x, t0y, t0z) || 1;
            t0x /= len0; t0y /= len0; t0z /= len0;

            let t1x = p2.x - p1.x;
            let t1y = p2.y - p1.y;
            let t1z = p2.z - p1.z;
            let len1 = Math.hypot(t1x, t1y, t1z) || 1;
            t1x /= len1; t1y /= len1; t1z /= len1;

            // curvature vector kappa ≈ (t1 - t0) / L
            const kx = (t1x - t0x) / L;
            const ky = (t1y - t0y) / L;
            const kz = (t1z - t0z) / L;

            const EI = p1.bendingStiffness;
            const fx = EI * kx;
            const fy = EI * ky;
            const fz = EI * kz;

            // distribute forces to maintain equilibrium
            p0.fx += fx; p0.fy += fy; p0.fz += fz;
            p1.fx -= 2 * fx; p1.fy -= 2 * fy; p1.fz -= 2 * fz;
            p2.fx += fx; p2.fy += fy; p2.fz += fz;
        }
    }

    // Integrate positions and velocities using explicit Euler
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

    // Enforce constant segment length constraint
    enforceConstraints() {
        const L = this.segmentLength;
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
        }
    }

    step(dt) {
        this.resetForces();
        this.accumulateBendingForces();
        this.integrate(dt);
        this.enforceConstraints();
    }
}
