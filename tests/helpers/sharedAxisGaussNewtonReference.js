import * as THREE from 'three';
import {assembleKirchhoffDirect} from '../../src/physics/kirchhoffDirectSolver.js';
const vec=p=>new THREE.Vector3(...p);
const frame=(b,e)=>new THREE.Quaternion(...['X','Y','Z','W'].map(k=>b['orientation'+k][e]));
export function referenceSharedAxisGaussNewton(s) {
const {layout,chain}=s,{hessian:H,gradient:g}=chain; H.fill(0);for(let i=0;i<g.length;i++)g[i]=-s.loads[i];let energy=0;
    const addOuter = (row, k) => {
        for (let i = 0; i < row.length; i++) for (let j = 0; j <= i; j++) {
            const [a, u] = row[i], [b, v] = row[j], hi = Math.max(a, b), lo = Math.min(a, b);
            if (hi - lo >= layout.band) throw new RangeError('Native pullback exceeds shared band');
            H[hi * layout.band + hi - lo] += k * u * v;
        }
    };
    for (const { body, spec, last } of s.materials) {
        const native = assembleKirchhoffDirect(body, 1), rows = Array.from({ length: native.rowCount }, () => new Map());
        const spin = layout.spins.get(spec.id);
        for (let e = 0; e < last; e++) {
            const q = frame(body, e), d1 = new THREE.Vector3(1, 0, 0).applyQuaternion(q), d2 = new THREE.Vector3(0, 1, 0).applyQuaternion(q);
            const length = vec(s.positions[e + 1]).distanceTo(vec(s.positions[e]));
            const pulls = [[], [], [[spin[e], 1]]];
            for (let axis = 0; axis < 3; axis++) {
                const u = -d2.getComponent(axis) / length, v = d1.getComponent(axis) / length;
                pulls[0].push([layout.positions[e] + axis, -u], [layout.positions[e + 1] + axis, u]);
                pulls[1].push([layout.positions[e] + axis, -v], [layout.positions[e + 1] + axis, v]);
            }
            for (let a = 0; a < 3; a++) {
                const dof = e * 6 + 3 + a;
                for (let k = 0; k < native.degree[dof]; k++) {
                    const at = dof * native.degreeCapacity + k, r = native.rows[at];
                    if (r % 6 >= 3) continue; // hard adaptation is represented kinematically
                    for (const [i, w] of pulls[a]) rows[r].set(i, (rows[r].get(i) ?? 0) + w * native.gradients[at]);
                }
            }
        }
        for (let r = 6; r < native.rowCount; r++) {
            if (r % 6 >= 3) continue;
            if (!(native.alpha[r] > 0)) throw new RangeError('Prototype needs finite positive material bend/twist compliance');
            const row = [...rows[r]].filter(([, v]) => v !== 0).sort((a, b) => a[0] - b[0]);
            const k = 1 / native.alpha[r], strain = native.strain[r];
            energy += .5 * k * strain * strain;
            for (const [i, v] of row) g[i] += v * k * strain;
            addOuter(row, k);
        }
    }
return energy;
}
