import { assembleKirchhoffCoupledSystem } from './kirchhoffCoupledSystem.js';
import { solveCoulombNewton } from './kirchhoffCoulombNewtonSolver.js';
import { measureCoupledLoadKKT } from './kirchhoffCoupledLoadSolver.js';
import { condenseKirchhoffTwoChannelSystem } from './kirchhoffTwoChannelCondensation.js';

function gram(system, i, j) {
    const hi = Math.max(i, j), lo = Math.min(i, j);
    if (hi - lo >= system.band) return 0;
    return system.matrix[hi * system.band + hi - lo] -
        (i === j ? system.rows[system.order[i]].alpha : 0);
}

// A bilateral equation with exactly zero mobility Gram AND zero compliance
// cannot reduce its RHS at this frozen state. Preserve that residual in the
// full certificate, and avoid requesting an impossible inner accuracy.
function structuralResidualFloor(system) {
    const n = system.native, mobile = new Uint8Array(n.count);
    // Inspect original sparse J and W: subtracting a large alpha from the
    // assembled diagonal could round a small, nonzero mobility to zero.
    for (let side = 0; side < 2; side++) {
        if (!n.material[side]) continue;
        for (let dof = 0; dof < n.columns[side].length; dof++) {
            if (n.material[side].weight[dof] === 0) continue;
            const entries = n.columns[side][dof];
            for (let k = 0; k < entries.length; k += 2)
                if (entries[k + 1] !== 0) mobile[entries[k]] = 1;
        }
    }
    let floor = 0;
    for (let sorted = 0; sorted < n.count; sorted++) {
        if (mobile[sorted]) continue;
        for (const row of [system.physicalRows[sorted], system.biasRows[sorted]])
            if (row >= 0 && system.alpha[row] === 0 && system.lower[row] === -Infinity && system.upper[row] === Infinity)
                floor = Math.max(floor, Math.abs(system.rhs[row]));
    }
    return floor;
}

/** A failed nonlinear trial can require a more accurate inexact direction.
 * Tighten only; the native zero-row residual stays in every certificate.
 * Returning null means another solve cannot improve the certified residual
 * at this accuracy. The caller still uses all original physical final gates.
 */
export function nextKirchhoffTwoChannelTolerance(diagnostics, tolerance) {
    const floor = diagnostics.structuralResidualFloor;
    if (!diagnostics.converged || !Number.isFinite(tolerance) || tolerance <= 0 ||
        !Number.isFinite(floor) || floor < 0 || !Number.isFinite(diagnostics.maximumResidual)) return null;
    const next = Math.max(tolerance * .1, floor);
    return next > 0 && next < tolerance && diagnostics.maximumResidual > next ? next : null;
}

/** Frozen two-channel direction on the existing full two-rod J and mobility.
 * `channels` is an array (or factory receiving the native assembly) indexed
 * by ORIGINAL, unsorted native row index. Every entry explicitly supplies:
 * {physical:'pose'|'physical-motion', bias:null|{
 *   channel:'pose'|'bias-motion', strain, alpha, lambda, lower, upper}}.
 * A bias row uses the same J as its physical counterpart. Missing rows are
 * errors: controls must not accidentally lose their dependence on final pose.
 *
 * Material and positional controls read dqP+dqB; physical normal/friction
 * read dqP. Bias material reads dqB; geometric normal reads dqP+dqB.
 * This generally NONSYMMETRIC block keeps physical and bias multipliers
 * separate. Friction cones reference physical normal load exclusively.
 *
 * Assembly defaults to the dense reference. With condensation:'auto', an
 * exact paired material elimination is attempted before allocating a matrix.
 * Successful condensation returns matrix:null and the retained operator in
 * `condensed`; the full O(N) row metadata remains for independent J*dq checks.
 * Unsupported or unsafe elimination explicitly falls back to the dense block.
 * matrixStorage:'general-band' stores the uncondensed block directly in axial
 * row intervals instead, preserving every coefficient without a dense copy.
 */
export function assembleKirchhoffTwoChannelSystem(constraint, dt, options = {}) {
    if (constraint._splitMotion?.phase === 'bias') throw new Error('Two-channel assembly needs physical material banks');
    const condensationMode = options.condensation ?? 'none';
    if (!['none', 'auto'].includes(condensationMode)) throw new RangeError('Unknown two-channel condensation mode');
    const matrixStorage=options.matrixStorage??'row-major';
    if(!['row-major','general-band'].includes(matrixStorage))throw new RangeError('Unknown two-channel matrix storage');
    const native = assembleKirchhoffCoupledSystem(constraint, dt, options);
    const specification = typeof options.channels === 'function' ? options.channels(native) : options.channels;
    if (!Array.isArray(specification) || specification.length !== native.count)
        throw new RangeError('An explicit channel is required for every native row');
    const descriptors = specification.map(item => {
        if (!['pose', 'physical-motion'].includes(item?.physical)) throw new RangeError('Unknown physical residual channel');
        if (!Object.hasOwn(item, 'bias')) throw new RangeError('Bias membership must be explicit');
        const b = item.bias;
        if (b !== null && (!['pose', 'bias-motion'].includes(b?.channel) ||
            ![b.strain, b.alpha, b.lambda].every(Number.isFinite) || b.alpha < 0 ||
            typeof b.lower !== 'number' || typeof b.upper !== 'number' || Number.isNaN(b.lower) ||
            Number.isNaN(b.upper) || b.lower > b.upper)) throw new RangeError('Invalid bias row');
        return { physical: item.physical, bias: b === null ? null : { ...b } };
    });
    const physicalRows = new Int32Array(native.count), biasRows = new Int32Array(native.count).fill(-1), rows = [];
    native.order.forEach((original, sorted) => {
        physicalRows[sorted] = rows.length;
        rows.push({ sorted, original, phase: 'physical', channel: descriptors[original].physical });
        const bias = descriptors[original].bias;
        if (bias) {
            biasRows[sorted] = rows.length;
            rows.push({ sorted, original, phase: 'bias', channel: bias.channel });
        }
    });
    const condensation = condensationMode === 'auto'
        ? condenseKirchhoffTwoChannelSystem(native, descriptors, options) : { status: 'disabled' };
    const condensed = condensation.status === 'condensed' ? condensation : null;
    const count = rows.length;
    let matrix=null;
    if(!condensed&&matrixStorage==='general-band'){
        const starts=new Int32Array(count),ends=new Int32Array(count),offsets=new Int32Array(count);let entries=0;
        for(let i=0;i<count;i++){
            const sorted=rows[i].sorted,lo=Math.max(0,sorted-native.band+1),hi=Math.min(native.count-1,sorted+native.band-1);
            starts[i]=physicalRows[lo];ends[i]=biasRows[hi]>=0?biasRows[hi]:physicalRows[hi];
            offsets[i]=entries-starts[i];entries+=ends[i]-starts[i]+1;
        }
        matrix={values:new Float64Array(entries),starts,ends,offsets};
    } else if(!condensed)matrix=new Float64Array(count*count);
    const rhs = new Float64Array(count), lower = new Float64Array(count), upper = new Float64Array(count);
    const alpha = new Float64Array(count);
    for (let i = 0; i < count; i++) {
        const row = rows[i], original = native.rows[row.original], bias = descriptors[row.original].bias;
        if (row.phase === 'physical') {
            rhs[i] = native.rhs[row.sorted]; lower[i] = native.lower[row.sorted]; upper[i] = native.upper[row.sorted];
            alpha[i] = original.alpha;
        } else {
            rhs[i] = -bias.strain - bias.alpha * bias.lambda;
            lower[i] = bias.lower - bias.lambda; upper[i] = bias.upper - bias.lambda;
            alpha[i] = bias.alpha;
        }
        if (matrix) for (let j = matrix.starts?.[i]??0; j <= (matrix.ends?.[i]??count-1); j++) {
            const column = rows[j];
            if (row.phase === column.phase || row.channel === 'pose') {
                if(matrixStorage==='general-band')matrix.values[matrix.offsets[i]+j]=gram(native,row.sorted,column.sorted);
                else matrix[i * count + j] = gram(native, row.sorted, column.sorted);
            }
        }
        if (matrix) {
            if(matrixStorage==='general-band')matrix.values[matrix.offsets[i]+i]+=alpha[i];
            else matrix[i * count + i] += alpha[i];
        }
    }
    const groups = native.groups.map(group => ({ ...group,
        rows: group.rows.map(i => physicalRows[i]),
        normalRow: group.normalRow == null ? undefined : physicalRows[group.normalRow],
        lambda: [...group.lambda], radii: [...group.radii], ...(group.mu ? { mu: [...group.mu] } : {}) }));
    return { native, descriptors, rows, physicalRows, biasRows, matrix, rhs, lower, upper, alpha, groups, count,
        condensed, condensation, matrixFormat:matrixStorage };
}

/** Solve without applying either channel. Returned geometry corrections are
 * dqP+dqB; material lambdas on inner/outer are PHYSICAL increments only.
 * `physical` and `bias` own their independent generalized responses and row
 * increments. The caller must publish dqP/dt once, apply total pose once,
 * and accumulate physical/bias normal banks separately with the SAME scale.
 * Solving defaults to exact material condensation with an explicit dense
 * fallback. Set condensation:'none' to run the full reference directly;
 * combine with matrixStorage:'general-band' for the compact axial route.
 */
export function solveKirchhoffTwoChannelSystem(constraint, dt, options = {}) {
    const system = assembleKirchhoffTwoChannelSystem(constraint, dt,
        { ...options, condensation: options.condensation ?? 'auto' }), native = system.native;
    const linear = system.condensed ?? system;
    let initialIncrement = options.initialIncrement;
    if (initialIncrement && system.condensed) {
        if (initialIncrement.length !== system.count) throw new RangeError('Initial increment must use the full two-channel row order');
        initialIncrement = Float64Array.from(linear.retainedRows, row => options.initialIncrement[
            (row.phase === 'physical' ? system.physicalRows : system.biasRows)[row.sorted]]);
    }
    const solved = solveCoulombNewton(linear.matrix, linear.rhs, linear.lower, linear.upper,
        linear.count, linear.count, linear.groups, { ...options, initialIncrement, matrixFormat: system.condensed?'row-major':system.matrixFormat });
    const increment = system.condensed ? system.condensed.recover(solved.increment).fullIncrement : solved.increment;
    const physical = native.bodies.map(body => ({ correction: new Float64Array(body.count * 6),
        lambda: new Float64Array(body.segmentCount * 6) }));
    const bias = native.bodies.map(body => ({ correction: new Float64Array(body.count * 6),
        lambda: new Float64Array(body.segmentCount * 6) }));
    const physicalIncrement = new Float64Array(native.count), biasIncrement = new Float64Array(native.count);
    native.order.forEach((original, sorted) => {
        physicalIncrement[original] = increment[system.physicalRows[sorted]];
        biasIncrement[original] = system.biasRows[sorted] < 0 ? 0 : increment[system.biasRows[sorted]];
    });
    for (let side = 0; side < 2; side++) {
        const material = native.material[side];
        if (!material) continue;
        for (let dof = 0; dof < native.columns[side].length; dof++) {
            const entries = native.columns[side][dof];
            let p = 0, b = 0;
            for (let k = 0; k < entries.length; k += 2) {
                const original = native.order[entries[k]], gradient = entries[k + 1];
                p += gradient * physicalIncrement[original]; b += gradient * biasIncrement[original];
            }
            physical[side].correction[dof] = material.weight[dof] * p;
            bias[side].correction[dof] = material.weight[dof] * b;
        }
        for (let row = 0; row < material.rowCount; row++) {
            physical[side].lambda[row] = physicalIncrement[native.materialOffsets[side] + row];
            bias[side].lambda[row] = biasIncrement[native.materialOffsets[side] + row];
        }
    }
    // Independent J*dq reconstruction certifies the actual two responses,
    // including off-diagonal terms. It never substitutes the solver's A*x.
    const reconstructed = Float64Array.from(system.rhs, (value, i) => value - system.alpha[i] * increment[i]);
    for (let side = 0; side < 2; side++) for (let dof = 0; dof < native.columns[side].length; dof++) {
        const entries = native.columns[side][dof], p = physical[side].correction[dof], b = bias[side].correction[dof];
        for (let k = 0; k < entries.length; k += 2) {
            const sorted = entries[k], g = entries[k + 1], pr = system.physicalRows[sorted], br = system.biasRows[sorted];
            reconstructed[pr] -= g * (p + (system.rows[pr].channel === 'pose' ? b : 0));
            if (br >= 0) reconstructed[br] -= g * (b + (system.rows[br].channel === 'pose' ? p : 0));
        }
    }
    // Recompute every physical cone at the recovered physical normal load.
    // Retained indices and geometric normal reactions never enter this check.
    const fullLower = system.lower.slice(), fullUpper = system.upper.slice();
    const fullGroups = system.groups.map(group => {
        const load = group.normalRow == null ? 1 : Math.max(0, group.normalLambda + increment[group.normalRow]);
        const radii = (group.normalRow == null ? group.radii : group.mu).map(mu => mu * load);
        if (radii.some(radius => radius === 0)) group.rows.forEach((row, axis) => {
            fullLower[row] = -radii[axis] - group.lambda[axis]; fullUpper[row] = radii[axis] - group.lambda[axis];
        });
        return { ...group, radii };
    });
    const kkt = measureCoupledLoadKKT(reconstructed, increment, fullLower, fullUpper, fullGroups);
    const diagnostics = { ...solved.diagnostics, reconstructionResidual: kkt.maximumResidual,
        maximumResidual: Math.max(solved.diagnostics.maximumResidual, kkt.maximumResidual),
        structuralResidualFloor: structuralResidualFloor(system), requestedTolerance: options.tolerance ?? 1e-8,
        physicalRows: native.count, biasRows: system.count - native.count, channels: 2,
        fullRowCount: system.count, condensation: system.condensation.status,
        factorizations: solved.diagnostics.factorizations + (system.condensation.diagnostics?.factorizations ?? system.condensation.factorizations ?? 0),
        ...(system.condensed ? { materialCondensation: { ...system.condensed.diagnostics } }
            : system.condensation.status === 'fallback' ? { condensationFallback: system.condensation.reason } : {}) };
    if (!(kkt.maximumResidual <= (options.tolerance ?? 1e-8))) {
        diagnostics.converged = false;
        if (diagnostics.status === 'converged') diagnostics.status = 'two-channel-reconstruction-residual';
    }
    let scale = 1;
    const total = native.bodies.map((body, side) => {
        const m = native.material[side], p = physical[side].correction, b = bias[side].correction;
        const correction = Float64Array.from(p, (value, i) => value + b[i]);
        if (m) for (let node = m.start; node <= m.end; node++) for (const c of [p, b, correction]) {
            const i = node * 6, spatial = Math.hypot(c[i], c[i + 1], c[i + 2]);
            const angular = node === m.end ? 0 : Math.hypot(c[i + 3], c[i + 4], c[i + 5]);
            const maxPosition = options.maximumPosition ?? body.segmentLength * .25, maxAngle = options.maximumAngle ?? .25;
            scale = Math.min(scale, maxPosition / Math.max(maxPosition, spatial), maxAngle / Math.max(maxAngle, angular));
        }
        return { correction, lambda: physical[side].lambda, start: m?.start, end: m?.end, redundantAxes: m?.redundantAxes.slice() };
    });
    const result = { inner: total[0], outer: total[1], physical, bias, physicalIncrement, biasIncrement,
        contactIncrement: physicalIncrement.slice(native.contactOffset, native.additionalOffset),
        additionalIncrement: physicalIncrement.slice(native.additionalOffset),
        biasContactIncrement: biasIncrement.slice(native.contactOffset, native.additionalOffset),
        biasAdditionalIncrement: biasIncrement.slice(native.additionalOffset), scale, diagnostics };
    if (options.includeSystem) result.system = system;
    return result;
}
