// Frozen pre-optimization implementation for numerical regression and profiling.
import * as THREE from 'three';
import {sharedAxisOuterMaterialAt} from '../../src/physics/kirchhoffSharedAxisNative.js';
const vec=p=>new THREE.Vector3(...p);
export function assembleSharedAxisConstraintRowsReference(s,{withTangent=true}={}) {
const g=s.chain.gradient;
    const rows = s.definitions.map((def, index) => {
        const e = def.edge, a = s.positions[e], b = s.positions[e + 1], length = vec(b).distanceTo(vec(a));
        let gap, J, gapHessian = null, penalty = 1;
        if (def.kind === 'length') {
            gap = length - (s.coordinates[e + 1] - s.coordinates[e]);
            const t = vec(b).sub(vec(a)).multiplyScalar(1 / length).toArray(); J = [...t.map(v => -v), ...t];
        } else {
            const owner = sharedAxisOuterMaterialAt(s,e,def.witness?.t??1,def.witness?.owner);
            const needHessian=withTangent&&s.multipliers[index]!==0;
            const cache=s.cacheMechanicalAssembly?(s.wallGeometryCache??=new Map()):null,cached=cache?.get(def);
            let contact;
            if(cached?.key===s.geometryKey&&(!needHessian||cached.withHessian)) {
                contact=cached.contact;s.wallGeometryCacheHits=(s.wallGeometryCacheHits??0)+1;
            } else {
                contact=(def.evaluate ?? s.wallSamples[def.sample])({state:s,a,b,edge:e,radius:owner.body.radius,owner:owner.spec.id,needHessian,sampleT:def.sampleT,
                    coordinateA:s.coordinates[e],coordinateB:s.coordinates[e+1]});
                if(cache&&contact){contact={...contact,jacobian:contact.jacobian?.slice(),hessian:contact.hessian?.slice()};cache.set(def,{key:s.geometryKey,withHessian:needHessian,contact});}
            }
            if (!contact || !Number.isFinite(contact.gap) || contact.jacobian?.length !== def.dofs.length || !contact.jacobian.every(Number.isFinite))
                throw new RangeError('Wall sample must supply its signed gap and exact edge Jacobian');
            gap = contact.gap; J = contact.jacobian; gapHessian = contact.hessian ?? null;
        }
        const multiplier = s.multipliers[index], sign = def.kind === 'wall' ? -1 : 1;
        for (let i = 0; i < def.dofs.length; i++) g[def.dofs[i]] += sign * J[i] * multiplier;
        let geometricHessian = gapHessian && multiplier!==0 && withTangent ? Float64Array.from(gapHessian, v => -multiplier * v) : undefined;
        if (withTangent && def.kind === 'length' && multiplier !== 0) {
            geometricHessian = new Float64Array(36);
            for (let i = 0; i < 6; i++) for (let j = 0; j < 6; j++)
                geometricHessian[i * 6 + j] = multiplier / length * (i < 3 ? -1 : 1) * (j < 3 ? -1 : 1) *
                    ((i % 3 === j % 3 ? 1 : 0) - J[3 + i % 3] * J[3 + j % 3]);
        }
        return { ...def, gap, jacobian: J, multiplier, penalty, geometricHessian };
    });
return rows;
}
