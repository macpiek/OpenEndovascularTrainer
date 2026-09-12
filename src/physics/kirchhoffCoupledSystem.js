import { kirchhoffComponentBodies } from './kirchhoffComponentBodies.js';
import { assembleKirchhoffDirect, applyKirchhoffDirectCorrection } from './kirchhoffDirectSolver.js';
import { solveCoupledBandQP } from './kirchhoffCoupledLinearSolver.js';
import { solveCoupledFrictionQP, measureCoupledFrictionKKT } from './kirchhoffCoupledFrictionSolver.js';
import { solveCoupledLoadQP, measureCoupledLoadKKT } from './kirchhoffCoupledLoadSolver.js';
import { solveActiveCondensedCoupledQP } from './kirchhoffActiveCondensedSolver.js';
import { buildKirchhoffAxialSystemLayout } from './kirchhoffAxialSystemLayout.js';
import { createKirchhoffBundleRuntime, assembleKirchhoffBundleColumns, addKirchhoffBundleGram, recoverKirchhoffBundleCorrection, measureKirchhoffBundleMobility } from './kirchhoffBundleRuntime.js';

function stencil(record, side, visit) {
    const prefix = side === 0 ? '_inner' : '_outer';
    const nodes = record[prefix + 'NodeIndices'], weights = record[prefix + 'NodeWeights'];
    const fallback = side === 0 ? record.innerWeights : record.outerWeights;
    const count = nodes ? record[prefix + 'NodeCount'] : 2;
    for (let i = 0; i < count; i++) visit(nodes ? nodes[i] : record[prefix + 'SegmentIndex'] + i, weights ? weights[i] : fallback[i]);
}

function arcCoordinates(body) {
    const arc = new Float64Array(body.count);
    for (let i = body.activeStart; i < body.activeEnd; i++) arc[i + 1] = arc[i] + body.restLength[i];
    return arc;
}

/** Assemble the FULL two-rod reference model. No physical dof is identified,
 * condensed or discarded. The default common-relative basis exactly transforms
 * paired translations AND their inverse-mass metric; local angular dofs remain
 * separate. options.basis:"individual" retains the independent reference path.
 * record.normalGradients:[{side,dof,value}] overrides the default normal
 * stencil, including optional local angular moment derivatives. Repeated dofs
 * are summed before Gram assembly. Geometry must already be collected and frozen by the caller. Assembly arrays
 * are borrowed until the next assembly on this constraint; solve result
 * corrections and multiplier arrays are owned snapshots.
 * options.jacobianOnly skips the dual Gram and its invertible bundle basis;
 * the original rows/columns are sufficient for the reduced axial assembly.
 *
 * additionalRows: [{strain, alpha, lambda, lower=-Infinity, upper=Infinity,
 *   gradients:[{side:0|1, dof:node*6+axis, value}], coordinate?, activeHint?}]. alpha is
 * compliance/dt² and bounds are on the TOTAL multiplier. xyz translations
 * are world coordinates; angular xyz are LOCAL right-sided frame increments.
 * options.groups supplies disjoint fixed-load U/V disks/ellipses:
 * {type:'coulomb-disk'|'coulomb-ellipse', rows:[u,v], radius | radii:[ru,rv]}.
 * Indices are RELATIVE to additionalRows. Surface-helper aliases
 * {kind,rowIndices,mu:[muU,muV],normalLambda} are also accepted. Radii stay
 * fixed during this solve; no radius derivative acts on normal forces. Caller
 * updates the load between joint relinearizations (non-associated friction).
 * Zero load/axis reduces exactly to fixed/interval force bounds.
 */
export function assembleKirchhoffCoupledSystem(constraint, dt = 1 / 120, options = {}) {
    if (!Number.isFinite(dt) || dt <= 0) throw new RangeError('A positive finite timestep is required');
    const bodies = kirchhoffComponentBodies(constraint);
    if (bodies.length === 1 && constraint.kirchhoffContacts?.length) throw new TypeError('Lumen contact requires two bodies');
    const material = bodies.map(body => assembleKirchhoffDirect(body, dt));
    const arcs = options.axialCoordinates ?? bodies.map(arcCoordinates);
    const axialLayout = options.includeAxialLayout
        ? buildKirchhoffAxialSystemLayout(constraint, material, arcs, options) : null;
    const records = constraint.kirchhoffContacts ?? [];
    let offset = 0;
    if (!options.axialCoordinates && records.length) {
        const offsets = records.map(record => {
            const s = [0, 0];
            for (let side = 0; side < bodies.length; side++) stencil(record, side, (node, weight) => { s[side] += arcs[side][node] * weight; });
            return s[0] - s[1];
        }).sort((a, b) => a - b);
        offset = offsets[Math.floor(offsets.length / 2)];
    }
    const workspace = constraint._coupledSystemAssembly ??= {};
    if (!workspace.columns || workspace.bodies.length !== bodies.length || workspace.bodies.some((body, side) => body !== bodies[side]) ||
        workspace.columns.some((columns, side) => columns.length !== bodies[side].count * 6)) {
        workspace.columns = bodies.map(body => Array.from({ length: body.count * 6 }, () => []));
        workspace.bodies = bodies;
    }
    const columns = workspace.columns, rows = workspace.rows ??= [];
    for (const side of columns) for (const entries of side) entries.length = 0;
    let rowCount = 0;
    const addRow = (kind, side, local, rhs, alpha, lambda, lower, upper, coordinate) => {
        const row = rows[rowCount] ??= {};
        row.kind = kind; row.side = side; row.local = local; row.rhs = rhs;
        row.alpha = alpha; row.lambda = lambda; row.lower = lower; row.upper = upper;
        row.coordinate = coordinate; row.index = rowCount; row.activeHint = false;
        return rowCount++;
    };
    const materialOffsets = [];
    const preserveBiasStrain = constraint._splitMotion?.phase === 'bias' &&
        constraint._splitMotion.biasMaterialMode === 'preserve-strain';
    for (let side = 0; side < bodies.length; side++) {
        const s = material[side];
        materialOffsets.push(rowCount);
        if (!s) continue;
        for (let i = 0; i < s.rowCount; i++) {
            const segment = s.start + Math.floor(i / 6);
            // Optional stabilization channel: preserve the physical phase's
            // strain while solving its geometric increment. Original material
            // law, Jacobians, metric, compliance and rest data stay unchanged.
            const strainOffset = options.materialStrainOffsets?.[side]?.[i] ?? 0;
            if (!Number.isFinite(strainOffset)) throw new RangeError('Invalid material strain offset');
            if (preserveBiasStrain && options.materialStrainOffsets?.[side]?.[i] == null)
                throw new RangeError('Preserving bias strain requires the physical phase reference');
            // Geometric bias may preserve C exactly without a second elastic
            // relaxation. Only this coupled bias row loses compliance; the
            // native material assembly and physical multipliers are untouched.
            addRow('material', side, i, preserveBiasStrain ? -s.strain[i] + strainOffset : s.rhs[i] + strainOffset,
                preserveBiasStrain ? 0 : s.alpha[i], s.lambda[i], -Infinity, Infinity,
                arcs[side][segment] + (side ? offset : 0) + (i % 6) * 1e-5);
        }
        for (let dof = s.start * 6; dof < (s.end + 1) * 6; dof++) {
            for (let k = 0; k < s.degree[dof]; k++) {
                const entry = dof * s.degreeCapacity + k;
                columns[side][dof].push(materialOffsets[side] + s.rows[entry], s.gradients[entry]);
            }
        }
    }
    const contactOffset = rowCount;
    records.forEach((record, i) => {
        const alpha = record._normalAlpha ?? 0, lambda = record.manifoldContact.normalLambda;
        let coordinate = 0;
        stencil(record, 0, (node, weight) => { coordinate += arcs[0][node] * weight; });
        const row = addRow('normal', -1, i, -record.gap - alpha * lambda, alpha, lambda, -lambda, Infinity, coordinate);
        rows[row].activeHint = record.manifoldContact._jointActive ?? false;
        if (record.normalGradients) {
            for (const { side, dof, value } of record.normalGradients) {
                if ((side !== 0 && side !== 1) || !Number.isInteger(dof) || !Number.isFinite(value)) throw new TypeError('Invalid custom normal gradient');
                const s = material[side];
                if (s && dof >= s.start * 6 && dof < s.end * 6 + 3 && value && s.weight[dof]) columns[side][dof].push(row, value);
            }
        } else for (let side = 0; side < bodies.length; side++) {
            const s = material[side];
            stencil(record, side, (node, weight) => {
                if (!s || node < s.start || node > s.end) return;
                for (let axis = 0; axis < 3; axis++) {
                    const value = (side === 0 ? -1 : 1) * weight * record.normal[axis];
                    if (value && s.weight[node * 6 + axis]) columns[side][node * 6 + axis].push(row, value);
                }
            });
        }
    });
    const additionalOffset = rowCount;
    for (const source of options.additionalRows ?? []) {
        if (source.group != null) throw new Error('Grouped friction rows require a cone solver');
        const alpha = source.alpha ?? 0, lambda = source.lambda ?? 0;
        let coordinate = source.coordinate;
        if (coordinate == null && source.gradients.length) {
            const g = source.gradients[0]; coordinate = arcs[g.side][Math.floor(g.dof / 6)] + (g.side ? offset : 0);
        }
        const row = addRow('additional', -1, rowCount - additionalOffset, -source.strain - alpha * lambda, alpha, lambda,
            (source.lower ?? -Infinity) - lambda, (source.upper ?? Infinity) - lambda, coordinate ?? 0);
        rows[row].activeHint = source.activeHint ?? false;
        for (const { side, dof, value } of source.gradients) {
            const s = material[side];
            if (s && dof >= s.start * 6 && dof < s.end * 6 + 3 && value && s.weight[dof]) columns[side][dof].push(row, value);
        }
    }
    const definitions = [], groupedRows = new Set();
    const normalRows = options.resolveNormalLoads ? new Map(records.map((record, i) => [record.manifoldContact, contactOffset + i])) : null;
    for (const [sourceIndex, source] of (options.groups ?? []).entries()) {
        const type = source.type ?? source.kind;
        if (type !== 'coulomb-disk' && type !== 'coulomb-ellipse') throw new TypeError('Unknown coupled friction group');
        const members = source.rows ?? source.rowIndices;
        if (!members || members.length !== 2 || members[0] === members[1] || members.some(i => !Number.isInteger(i) || i < 0 || i >= rowCount - additionalOffset)) throw new RangeError('Friction rows must name two distinct additional-row indices');
        const radii = source.radii ?? (source.radius != null ? [source.radius, source.radius]
            : source.mu?.map(mu => mu * source.normalLambda));
        if (!radii || radii.length !== 2 || radii.some(r => !Number.isFinite(r) || r < 0)) throw new RangeError('Friction groups require fixed finite nonnegative radii');
        if (type === 'coulomb-disk' && radii[0] !== radii[1]) throw new RangeError('A Coulomb disk has equal radii');
        // Runtime batches store row indices in Uint32Array. Mapping force
        // multipliers through that array would silently truncate them to
        // integers and turn a total-force cone into an increment-only cone.
        const originalRows = Array.from(members, i => i + additionalOffset);
        for (const original of originalRows) {
            if (groupedRows.has(original)) throw new Error('Friction groups must be disjoint');
            groupedRows.add(original);
            if (rows[original].lower !== -Infinity || rows[original].upper !== Infinity) throw new Error('Grouped rows cannot also have independent box bounds');
        }
        const lambda = originalRows.map(i => rows[i].lambda);
        let normalOriginal = normalRows?.get(source.normalContact);
        if (options.resolveNormalLoads && source.normalRow) {
            const index = options.additionalRows.indexOf(source.normalRow);
            if (index < 0) throw new RangeError('A friction normal row must belong to the current additional rows');
            normalOriginal = additionalOffset + index;
        }
        if (options.resolveNormalLoads && normalOriginal == null && source.normalContact?.owner) {
            const contact = source.normalContact;
            const index = options.additionalRows.findIndex(row => row.owner === contact.owner && row.node === contact.index && row.kind === 'tool');
            if (index >= 0) normalOriginal = additionalOffset + index;
        }
        if (normalOriginal != null && source.mu) {
            definitions.push({ originalRows, radii: [...radii], lambda, sourceIndex, normalOriginal,
                normalLambda: source.normalLambda, mu: Array.from(source.mu) });
        } else if (radii[0] === 0 || radii[1] === 0) {
            // Exact degenerate ellipse: a point or one force interval.
            originalRows.forEach((row, axis) => { rows[row].lower = -radii[axis] - lambda[axis]; rows[row].upper = radii[axis] - lambda[axis]; });
        } else definitions.push({ originalRows, radii: [...radii], lambda, sourceIndex });
    }
    rows.length = rowCount;
    const order = workspace.order ??= [];
    order.length = rowCount;
    for (let i = 0; i < rowCount; i++) order[i] = i;
    order.sort((a, b) => rows[a].coordinate - rows[b].coordinate || a - b);
    if (!workspace.inverseOrder || workspace.inverseOrder.length < rowCount) workspace.inverseOrder = new Int32Array(rowCount);
    const inverseOrder = workspace.inverseOrder;
    order.forEach((original, i) => { inverseOrder[original] = i; });
    // Merge repeated contributions to a coordinate BEFORE taking J W J^T.
    // Endpoint/cubic stencils and extension blocks can repeat a node.
    let band = 1;
    for (const bodyColumns of columns) for (const entries of bodyColumns) {
        // The columns are short; in-place insertion sort/merge avoids a Map
        // and arrays of entry pairs for every generalized coordinate.
        for (let k = 0; k < entries.length; k += 2) entries[k] = inverseOrder[entries[k]];
        for (let k = 2; k < entries.length; k += 2) {
            const row = entries[k], value = entries[k + 1];
            let j = k - 2;
            while (j >= 0 && entries[j] > row) { entries[j + 2] = entries[j]; entries[j + 3] = entries[j + 1]; j -= 2; }
            entries[j + 2] = row; entries[j + 3] = value;
        }
        let n = 0;
        for (let k = 0; k < entries.length; k += 2) {
            if (n && entries[n - 2] === entries[k]) entries[n - 1] += entries[k + 1];
            else { entries[n++] = entries[k]; entries[n++] = entries[k + 1]; }
        }
        entries.length = n;
        if (entries.length) band = Math.max(band, entries[entries.length - 2] - entries[0] + 1);
    }
    const count = rowCount, individualBand = band, basis = options.basis ?? (bodies.length === 1 ? 'individual' : 'common-relative');
    if (basis !== 'individual' && basis !== 'common-relative') throw new RangeError('Unknown coupled coordinate basis');
    let bundle = null;
    if (basis === 'common-relative' && bodies.length === 2 && !options.jacobianOnly) {
        bundle = constraint._bundleRuntime ??= createKirchhoffBundleRuntime();
        assembleKirchhoffBundleColumns(bundle, { bodies, material, columns, count },
            { axialCoordinates: arcs, axialOffsets: [0, offset], pairingRevision: options.pairingRevision });
        band = bundle.band;
    }
    if (!options.jacobianOnly && (!workspace.matrix || workspace.matrix.length < count * band)) workspace.matrix = new Float64Array(count * band);
    if (!workspace.rhs || workspace.rhs.length < count) {
        for (const key of ['rhs', 'lower', 'upper']) workspace[key] = new Float64Array(count);
        workspace.initialFree = new Uint8Array(count);
    }
    const matrix = options.jacobianOnly ? null : workspace.matrix.subarray(0, count * band);
    matrix?.fill(0);
    const rhs = workspace.rhs.subarray(0, count), lower = workspace.lower.subarray(0, count), upper = workspace.upper.subarray(0, count);
    order.forEach((original, i) => {
        const row = rows[original];
        if (!Number.isFinite(row.rhs) || !Number.isFinite(row.alpha) || row.alpha < 0 || row.lower > row.upper) throw new RangeError('Invalid coupled row');
        if (matrix) matrix[i * band] = row.alpha;
        rhs[i] = row.rhs; lower[i] = row.lower; upper[i] = row.upper;
        workspace.initialFree[i] = Number(row.activeHint);
    });
    if (bundle) addKirchhoffBundleGram(bundle, matrix, band);
    else if (matrix) for (let side = 0; side < bodies.length; side++) for (let dof = 0; dof < columns[side].length; dof++) {
        const entries = columns[side][dof], weight = material[side]?.weight[dof] ?? 0;
        for (let a = 0; a < entries.length; a += 2) for (let b = 0; b <= a; b += 2) {
            matrix[entries[a] * band + entries[a] - entries[b]] += weight * entries[a + 1] * entries[b + 1];
        }
    }
    return { bodies, material, columns, rows, order, inverseOrder, materialOffsets, contactOffset, additionalOffset, bundle, axialLayout, basis, individualBand,
        matrix, rhs, lower, upper, count, band, initialFree: workspace.initialFree,
        groups: definitions.map(group => ({ ...group, rows: group.originalRows.map(i => inverseOrder[i]),
            normalRow: group.normalOriginal == null ? undefined : inverseOrder[group.normalOriginal] })) };
}

/** One simultaneous material + unilateral normal Newton/XPBD direction.
 * Default apply:false; assembly only enforces already prescribed boundary
 * frames (same as solo direct). Unscaled corrections and delta multipliers are
 * returned with ONE proposed trust-region scale. Parent must relinearize
 * contact geometry and check nonlinear residuals after applying the direction.
 * Successful solves update only boolean manifoldContact._jointActive and
 * additionalRows[].activeHint (disable via updateActiveHints:false). These
 * hints retain no force, survive multiplier resets, and are permutation-aware.
 */
export function solveKirchhoffCoupledSystem(constraint, dt = 1 / 120, options = {}) {
    if (options.jacobianOnly && typeof options.assemblySolver !== 'function') throw new TypeError('The dual solver requires its Gram matrix');
    const system = assembleKirchhoffCoupledSystem(constraint, dt, options);
    const originalGroups = options.includeSystem ? system.groups : null;
    const linearOptions = { ...options, workspace: constraint._coupledSystemQP ??= {}, initialFree: system.initialFree };
    const solveLoads = system.groups.some(group => group.normalRow != null);
    const solved = options.assemblySolver ? options.assemblySolver(system, linearOptions)
        : options.activeCondensation ? solveActiveCondensedCoupledQP(system.matrix, system.rhs, system.lower, system.upper, system.count, system.band, system.groups,
        { ...linearOptions, frictionWorkspace: constraint._coupledSystemFriction ??= {}, loadWorkspace: constraint._coupledSystemLoad ??= {} })
        : solveLoads ? solveCoupledLoadQP(system.matrix, system.rhs, system.lower, system.upper, system.count, system.band, system.groups,
        { ...linearOptions, frictionWorkspace: constraint._coupledSystemFriction ??= {}, loadWorkspace: constraint._coupledSystemLoad ??= {} })
        : system.groups.length ? solveCoupledFrictionQP(system.matrix, system.rhs, system.lower, system.upper, system.count, system.band, system.groups,
        { ...linearOptions, frictionWorkspace: constraint._coupledSystemFriction ??= {} })
        : solveCoupledBandQP(system.matrix, system.rhs, system.lower, system.upper, system.count, system.band, linearOptions);
    if (solveLoads) { system.groups = solved.groups; system.lower = solved.lower; system.upper = solved.upper; }
    const results = system.bodies.map((body, side) => ({ correction: new Float64Array(body.count * 6),
        lambda: new Float64Array(body.segmentCount * 6), start: system.material[side]?.start, end: system.material[side]?.end,
        redundantAxes: system.material[side]?.redundantAxes.slice() }));
    if (system.bundle) recoverKirchhoffBundleCorrection(system.bundle, solved.increment, results[0].correction, results[1].correction);
    for (let side = 0; side < system.bodies.length; side++) {
        const s = system.material[side];
        if (!s) continue;
        if (!system.bundle) for (let dof = s.start * 6; dof < (s.end + 1) * 6; dof++) {
            const entries = system.columns[side][dof];
            let impulse = 0;
            for (let k = 0; k < entries.length; k += 2) impulse += entries[k + 1] * solved.increment[entries[k]];
            results[side].correction[dof] = s.weight[dof] * impulse;
        }
        for (let row = 0; row < s.rowCount; row++) results[side].lambda[row] = solved.increment[system.inverseOrder[system.materialOffsets[side] + row]];
    }
    // Recheck the equations through the actual reconstructed generalized
    // correction, independently of band-matrix multiplication. This catches
    // cancellation in a nearly dependent dual force representation before
    // any correction can be applied to the physical bodies.
    const physicalResidual = new Float64Array(system.count);
    system.order.forEach((original, i) => { physicalResidual[i] = system.rhs[i] - system.rows[original].alpha * solved.increment[i]; });
    for (let side = 0; side < system.bodies.length; side++) for (let dof = 0; dof < system.columns[side].length; dof++) {
        const entries = system.columns[side][dof], correction = results[side].correction[dof];
        for (let k = 0; k < entries.length; k += 2) physicalResidual[entries[k]] -= entries[k + 1] * correction;
    }
    const reconstruction = solveLoads ? measureCoupledLoadKKT(physicalResidual, solved.increment,
        system.lower, system.upper, solved.allGroups) : measureCoupledFrictionKKT(physicalResidual, solved.increment,
        system.lower, system.upper, system.groups);
    const reconstructionResidual = reconstruction.maximumResidual;
    if (system.groups.length) solved.diagnostics.frictionResidual = reconstruction.groupResidual;
    solved.diagnostics.reconstructionResidual = reconstructionResidual;
    if (!(reconstructionResidual <= (options.tolerance ?? 1e-8))) {
        solved.diagnostics.converged = false;
        if (solved.diagnostics.status === 'converged') solved.diagnostics.status = 'reconstruction-residual';
    }
    solved.diagnostics.maximumResidual = Math.max(solved.diagnostics.maximumResidual, reconstructionResidual);
    if (!solved.diagnostics.converged) {
        const grouped = new Set((solved.allGroups ?? system.groups).flatMap(group => group.rows));
        let worst = null;
        system.order.forEach((original, i) => {
            if (grouped.has(i)) return;
            const r = physicalResidual[i], x = solved.increment[i], lo = system.lower[i], hi = system.upper[i];
            const violation = lo === hi ? 0 : x <= lo ? Math.max(0, r) : x >= hi ? Math.max(0, -r) : Math.abs(r);
            if (!(violation > (worst?.violation ?? 0))) return;
            const row = system.rows[original], source = row.kind === 'additional' ? options.additionalRows?.[row.local] : null;
            worst = { row: i, kind: row.kind, sourceKind: source?.kind ?? null, side: source?.side ?? row.side,
                node: source?.node ?? null, component: source?.component ?? null, local: row.local,
                violation, residual: r, rhs: row.rhs, alpha: row.alpha, increment: x, lower: lo, upper: hi };
        });
        solved.diagnostics.worstScalarConstraint = worst;
    }
    const contactIncrement = new Float64Array(system.additionalOffset - system.contactOffset);
    for (let i = 0; i < contactIncrement.length; i++) contactIncrement[i] = solved.increment[system.inverseOrder[system.contactOffset + i]];
    const contactActive = new Uint8Array(contactIncrement.length);
    for (let i = 0; i < contactActive.length; i++) contactActive[i] = Number(contactIncrement[i] + constraint.kirchhoffContacts[i].manifoldContact.normalLambda > 1e-14);
    const additionalIncrement = new Float64Array(system.count - system.additionalOffset);
    for (let i = 0; i < additionalIncrement.length; i++) additionalIncrement[i] = solved.increment[system.inverseOrder[system.additionalOffset + i]];
    const additionalActive = new Uint8Array(additionalIncrement.length);
    for (let i = 0; i < additionalActive.length; i++) {
        const row = system.rows[system.additionalOffset + i], delta = additionalIncrement[i];
        additionalActive[i] = Number(delta > row.lower + 1e-14 && delta < row.upper - 1e-14);
    }
    // Identity-aware working-set hints are not retained forces. They survive
    // world multiplier resets and sorting/topology changes. Every omitted row
    // is still tested against the complete original KKT equations.
    if (solved.diagnostics.converged && options.updateActiveHints !== false) {
        constraint.kirchhoffContacts?.forEach((r, i) => { r.manifoldContact._jointActive = Boolean(contactActive[i]); });
        options.additionalRows?.forEach((r, i) => { r.activeHint = Boolean(additionalActive[i]); });
    }
    let scale = 1, materialResidual = 0, contactResidual = 0;
    for (let side = 0; side < system.bodies.length; side++) {
        const body = system.bodies[side], result = results[side];
        for (let node = result.start; node <= result.end; node++) {
            const d = node * 6, c = result.correction;
            const angular = node === result.end ? 0 : Math.hypot(c[d + 3], c[d + 4], c[d + 5]);
            const spatial = Math.hypot(c[d], c[d + 1], c[d + 2]);
            const maxPosition = options.maximumPosition ?? body.segmentLength * 0.25;
            const maxAngle = options.maximumAngle ?? 0.25;
            scale = Math.min(scale, maxAngle / Math.max(maxAngle, angular), maxPosition / Math.max(maxPosition, spatial));
        }
    }
    system.order.forEach((original, i) => {
        if (system.rows[original].kind === 'material') materialResidual = Math.max(materialResidual, Math.abs(physicalResidual[i]));
        else if (system.rows[original].kind === 'normal') contactResidual = Math.max(contactResidual,
            solved.increment[i] > system.lower[i] ? Math.abs(physicalResidual[i]) : Math.max(0, physicalResidual[i]));
    });
    const result = { bodies: system.bodies.slice(), responses: results, inner: results[0], outer: results[1], contactIncrement, additionalIncrement, contactActive, additionalActive, scale,
        diagnostics: { ...solved.diagnostics, basis: system.basis, individualBand: system.individualBand,
            pairedNodes: system.bundle?.pairCount ?? 0, bundleEntries: system.bundle?.entryCount ?? 0, materialResidual, contactResidual, activeContacts: contactActive.reduce((sum, active) => sum + active, 0),
            dofCount: system.material.reduce((n, s) => n + (s ? (s.end - s.start) * 6 + 3 : 0), 0) } };
    if (options.includeSystem) {
        // A frozen-system audit also needs zero-load groups, which the final
        // positive-radius working set may omit. Preserve their input law.
        system.originalGroups = originalGroups;
        result.system = system;
    }
    if (options.mobilityAudit) {
        if (!system.bundle) throw new RangeError('Mobility audit requires the common-relative basis');
        result.diagnostics.mobility = measureKirchhoffBundleMobility(system.bundle, solved.increment,
            results[0].correction, results[1].correction, options.mobilityAudit);
        if (!result.diagnostics.mobility.passed) {
            result.diagnostics.converged = false;
            result.diagnostics.status = 'original-mobility-residual';
        }
    }
    if (options.apply && result.diagnostics.converged) applyKirchhoffCoupledCorrection(constraint, result);
    return result;
}

/** Apply positions, frames and MATERIAL multipliers with one common scale.
 * Does NOT modify normalLambda or additional-row multipliers. The caller must
 * pass scale*contactIncrement to its manifold.accumulateKnownNormalLambda,
 * including its friction projection, exactly once. No toolProjection is added:
 * this is the entire material/contact direction, not just a contact correction.
 */
export function applyKirchhoffCoupledCorrection(constraint, result, scale = result.scale) {
    if (!Number.isFinite(scale) || scale < 0 || scale > 1) throw new RangeError('Coupled scale must be between zero and one');
    if (!result.diagnostics.converged) throw new Error('Cannot apply a coupled direction that failed the original KKT residual test');
    const bodies = kirchhoffComponentBodies(constraint);
    if(result.bodies && (result.bodies.length !== bodies.length || result.bodies.some((body,i)=>body!==bodies[i])))
        throw new Error('Coupled result bodies changed before application');
    const responses = result.responses ?? [result.inner,result.outer];
    for (let side=0;side<bodies.length;side++) {
        const body=bodies[side],response=responses[side];
        if (response.start == null) continue;
        // Reject stale topology rather than interpreting row multipliers on a
        // different range; borrowed direct scratch is not the result contract.
        if (body.activeStart !== response.start || body.activeEnd !== response.end) throw new Error('Coupled result topology changed before application');
        body.kirchhoffScratch.direct.start = response.start; body.kirchhoffScratch.direct.end = response.end;
        applyKirchhoffDirectCorrection(body, response.correction, response.lambda, scale);
        // A prescribed two-endpoint edge has one algebraically redundant
        // adaptation row. Delay its force reset until application so dry
        // assembly never changes a material multiplier.
        if (scale > 0) for (let segment = response.start; segment < response.end; segment++) {
            const axis = response.redundantAxes[segment];
            if (axis >= 0) [body.adaptationLambdaX, body.adaptationLambdaY, body.adaptationLambdaZ][axis][segment] = 0;
        }
        body.kirchhoffScratch.direct.factorAge = Infinity;
    }
}
