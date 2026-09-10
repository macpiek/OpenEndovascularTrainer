import { createCompositeChainLayout, createCompositeChainWorkspace, assembleCompositeChain } from './kirchhoffCompositeChain.js';
import { transportCompositeReferenceFrames } from './kirchhoffCompositeElement.js';
import { evaluateCompositeKinematics } from './kirchhoffCompositeKinematics.js';

const finite = (x, name) => { if (!Number.isFinite(x)) throw new TypeError(`${name} must be finite`); return x; };
const positive = (x, name) => { if (!(finite(x, name) > 0)) throw new RangeError(`${name} must be positive`); return x; };
const vector = (v, n, name) => {
    if (!v || v.length !== n) throw new TypeError(`${name} needs ${n} entries`);
    return Array.from(v, x => finite(x, name));
};
const ends = (v, name) => typeof v === 'number' ? [finite(v, name), v] : vector(v, 2, name);
const mix = (a, b, t) => t === 0 ? a : t === 1 ? b : a + t * (b - a);
const mixVector = (a, b, t) => a.map((v, i) => mix(v, b[i], t));
const dot = (a, b) => a.reduce((sum, v, i) => sum + v * b[i], 0);
const cross = (a, b) => [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]];
const distance = (a, b) => Math.hypot(...a.map((v, i) => v - b[i]));
const GAUSS = [.5 - 1 / (2 * Math.sqrt(3)), .5 + 1 / (2 * Math.sqrt(3))];
const CRITERIA = ['position', 'rotation', 'materialLabel', 'velocity', 'angularVelocity', 'rotationGradient', 'mass', 'momentum', 'kineticEnergy', 'energy', 'force', 'torque', 'work'];

function increasing(values, name) {
    const x = Array.from(values ?? [], v => finite(v, name));
    if (x.length < 2 || x.some((v, i) => i && v <= x[i - 1])) throw new RangeError(`${name} must strictly increase`);
    return x;
}
function angular(input) {
    if (input === undefined || input === null) return null;
    const frame = input.frameSpin;
    return { thetaDt: ends(input.thetaDt, 'thetaDt'), thetaDx: ends(input.thetaDx, 'thetaDx'),
        frameSpin: frame && !Array.isArray(frame) && !ArrayBuffer.isView(frame) && typeof frame === 'object'
            ? { dt: ends(frame.dt, 'frameSpin.dt'), dx: ends(frame.dx, 'frameSpin.dx') }
            : ends(frame, 'material-path frameSpin') };
}
function restrictAngular(input, a, b) {
    if (input === null) return null;
    const restrict = v => [mix(v[0], v[1], a), mix(v[0], v[1], b)];
    return { thetaDt: restrict(input.thetaDt), thetaDx: restrict(input.thetaDx),
        frameSpin: Array.isArray(input.frameSpin) ? restrict(input.frameSpin)
            : { dt: restrict(input.frameSpin.dt), dx: restrict(input.frameSpin.dx) } };
}
function kinematicInput(tool, fraction) {
    const result = { id: tool.id, dsDx: tool.materialMap.dsDx, dsDt: mix(...tool.materialMap.dsDt, fraction) };
    if (tool.angularKinematics !== null) {
        const a = tool.angularKinematics;
        result.thetaDt = mix(...a.thetaDt, fraction); result.thetaDx = mix(...a.thetaDx, fraction);
        result.frameSpin = Array.isArray(a.frameSpin) ? mix(...a.frameSpin, fraction)
            : { dt: mix(...a.frameSpin.dt, fraction), dx: mix(...a.frameSpin.dx, fraction) };
    }
    return result;
}
function axialRates(tool, fraction) {
    // q_t and q_x do not enter the axial angular equation. Only this scalar
    // is read; the dummy translational result is never published as velocity.
    return evaluateCompositeKinematics({ positionDt: [0, 0, 0], positionDx: [0, 0, 0], tools: [kinematicInput(tool, fraction)] }).tools[0];
}
function compareAngularFields(a, fractionA, b, fractionB, errors) {
    const x = kinematicInput(a, fractionA), y = kinematicInput(b, fractionB);
    errors.angularVelocity = Math.max(errors.angularVelocity, Math.abs(x.thetaDt - y.thetaDt));
    errors.rotationGradient = Math.max(errors.rotationGradient, Math.abs(x.thetaDx - y.thetaDx));
    if (typeof x.frameSpin !== typeof y.frameSpin) throw new TypeError('The accepted frame-connection representation must be preserved');
    if (typeof x.frameSpin === 'number') errors.angularVelocity = Math.max(errors.angularVelocity, Math.abs(x.frameSpin - y.frameSpin));
    else {
        errors.angularVelocity = Math.max(errors.angularVelocity, Math.abs(x.frameSpin.dt - y.frameSpin.dt));
        errors.rotationGradient = Math.max(errors.rotationGradient, Math.abs(x.frameSpin.dx - y.frameSpin.dx));
    }
    const p = axialRates(a, fractionA), q = axialRates(b, fractionB);
    errors.angularVelocity = Math.max(errors.angularVelocity, Math.abs(p.spin - q.spin), Math.abs(p.frameSpin - q.frameSpin));
}
function normalizedFields(edges, layout) {
    if (!edges || edges.length !== layout.nodeCount - 1) throw new RangeError('One prepared material field per old edge is required');
    return edges.map((edge, i) => {
        const ids = layout.edgeToolIds[i];
        if (!edge.tools || edge.tools.length !== ids.length || new Set(edge.tools.map(t => t.id)).size !== ids.length ||
            edge.tools.some(t => !ids.includes(t.id))) throw new RangeError('Material fields must match exact edge ownership');
        return { tools: edge.tools.map(t => {
            const m = t.materialMap;
            if (!t.oldMaterialVelocities || t.oldMaterialVelocities.length !== 2) throw new TypeError('OLD MATERIAL velocity endpoints are required');
            const omega = t.oldAngularVelocities;
            if (omega !== undefined && omega !== null && omega.length !== 2) throw new TypeError('Angular velocity needs two vector endpoints');
            return { id: t.id, massPerMaterialLength: positive(t.massPerMaterialLength, 'massPerMaterialLength'),
                materialMap: { sStart: finite(m?.sStart, 'sStart'), dsDx: positive(m?.dsDx, 'dsDx'), dsDt: ends(m?.dsDt, 'dsDt') },
                oldMaterialVelocities: t.oldMaterialVelocities.map(v => vector(v, 3, 'old material velocity')),
                angularKinematics: angular(t.angularKinematics),
                oldAngularVelocities: omega == null ? null : omega.map(v => vector(v, 3, 'old angular velocity')) };
        }) };
    });
}
function restriction(edges, oldX, newX, parents) {
    return parents.map((parent, child) => {
        const length = oldX[parent + 1] - oldX[parent], a = (newX[child] - oldX[parent]) / length,
            b = (newX[child + 1] - oldX[parent]) / length;
        return { tools: edges[parent].tools.map(t => ({ ...t,
            materialMap: { sStart: t.materialMap.sStart + t.materialMap.dsDx * (newX[child] - oldX[parent]),
                dsDx: t.materialMap.dsDx, dsDt: [mix(...t.materialMap.dsDt, a), mix(...t.materialMap.dsDt, b)] },
            oldMaterialVelocities: [a, b].map(f => mixVector(...t.oldMaterialVelocities, f)),
            angularKinematics: restrictAngular(t.angularKinematics, a, b),
            oldAngularVelocities: t.oldAngularVelocities === null ? null : [a, b].map(f => mixVector(...t.oldAngularVelocities, f)) })) };
    });
}
function totals(edges, coordinates) {
    const result = new Map();
    edges.forEach((e, i) => e.tools.forEach(t => {
        if (!result.has(t.id)) result.set(t.id, { mass: 0, momentum: [0, 0, 0], kineticEnergy: 0 });
        const total = result.get(t.id), mass = t.massPerMaterialLength * t.materialMap.dsDx * (coordinates[i + 1] - coordinates[i]);
        total.mass += mass;
        for (const f of GAUSS) {
            const v = mixVector(...t.oldMaterialVelocities, f);
            v.forEach((value, axis) => { total.momentum[axis] += mass * value / 2; });
            total.kineticEnergy += mass * dot(v, v) / 4;
        }
    }));
    return result;
}
function cachedTool(source, id, layout, coordinates, angles, referenceTwists, fields) {
    if (!source || typeof source.materialAt !== 'function' && !source.material)
        throw new TypeError(`Explicit target material for ${id} is required; old cached dual cells cannot be reused implicitly`);
    const cells = new Map(), scales = new Map();
    for (const { vertex: i, tools } of layout.hinges) if (tools.includes(id)) {
        const coordinate = coordinates[i], start = (coordinates[i - 1] + coordinate) / 2, end = (coordinate + coordinates[i + 1]) / 2;
        const raw = source.materialAt ? source.materialAt({ vertex: i, coordinate, start, end }) : source.material;
        const value = { ...raw, stiffness: Float64Array.from(vector(raw?.stiffness, 9, 'compiled target stiffness')),
            intrinsic: Float64Array.from(vector(raw?.intrinsic, 3, 'compiled target intrinsic')),
            energyOffset: finite(raw?.energyOffset ?? 0, 'target energy offset') };
        const dsDx = positive(typeof source.dsDx === 'function' ? source.dsDx(coordinate) : source.dsDx, 'explicit target dsDx');
        const a = fields[i - 1].tools.find(t => t.id === id).materialMap.dsDx, b = fields[i].tools.find(t => t.id === id).materialMap.dsDx;
        if (a !== b || dsDx !== a) throw new RangeError('Target material dsDx must match unchanged material map over each hinge');
        cells.set(i, { coordinate, start, end, value }); scales.set(coordinate, dsDx);
    }
    return { id, angles, referenceTwists, dsDx: x => scales.get(x), materialAt({ vertex, coordinate, start, end }) {
        const c = cells.get(vertex);
        if (!c || coordinate !== c.coordinate || start !== c.start || end !== c.end) throw new RangeError('Transferred material cache requires its frozen mesh coordinates');
        return c.value;
    } };
}
function pullback(fine, map, oldCount) {
    const result = new Float64Array(oldCount);
    map.forEach((entries, row) => entries.forEach(([col, weight]) => { result[col] += weight * fine[row]; }));
    return result;
}
function gradientErrors(a, b, layout) {
    let force = 0, torque = 0;
    for (const start of layout.positions) force = Math.max(force, distance(a.subarray(start, start + 3), b.subarray(start, start + 3)));
    for (const rows of layout.spins.values()) for (const dof of rows) if (dof >= 0) torque = Math.max(torque, Math.abs(a[dof] - b[dof]));
    return { force, torque };
}
function lengths(data, layout, multipliers) {
    const gradient = new Float64Array(layout.dofCount); let work = 0;
    for (let edge = 0; edge < multipliers.length; edge++) {
        const d = data.positions[edge + 1].map((v, i) => v - data.positions[edge][i]), length = positive(Math.hypot(...d), 'spatial edge length');
        const lambda = finite(multipliers[edge], 'signed axial tension');
        work += lambda * (length - (data.coordinates[edge + 1] - data.coordinates[edge]));
        for (let axis = 0; axis < 3; axis++) {
            gradient[layout.positions[edge] + axis] -= lambda * d[axis] / length;
            gradient[layout.positions[edge + 1] + axis] += lambda * d[axis] / length;
        }
    }
    return { work, gradient };
}
function orientation(data, layout, referenceTwists) {
    const frames = transportCompositeReferenceFrames(data.reference, data.positions), phases = new Map();
    const byId = new Map(data.tools.map(t => [t.id, t]));
    return layout.edgeToolIds.map((ids, edge) => ({ edge, tools: ids.map(id => {
        const theta = byId.get(id).angles[edge], frame = frames[edge], perpendicular = cross(frame.tangent, frame.director);
        const previous = phases.get(id), connection = previous === undefined ? 0 : referenceTwists.get(id)[edge - 1];
        const cumulative = (previous ?? 0) + connection; phases.set(id, cumulative);
        return { id, theta, cumulativeReferenceTwist: cumulative, unwrappedPhase: theta + cumulative,
            tangent: [...frame.tangent], director1: frame.director.map((v, i) => Math.cos(theta) * v + Math.sin(theta) * perpendicular[i]),
            director2: frame.director.map((v, i) => Math.cos(theta) * perpendicular[i] - Math.sin(theta) * v) };
    }) }));
}
function pointHistories(records, source, target, fields, parents, map, errors) {
    if (!Array.isArray(records)) throw new TypeError('An explicit contactHistory list is required (empty if none)');
    const before = new Float64Array(source.layout.dofCount), after = new Float64Array(target.layout.dofCount);
    const ids = new Set(), output = records.map(record => {
        if (typeof record.id !== 'string' || !record.id || ids.has(record.id)) throw new TypeError('Persistent point histories need unique nonempty ids');
        ids.add(record.id);
        const edge = record.edge, fraction = finite(record.fraction, 'contact fraction');
        if (!Number.isInteger(edge) || edge < 0 || edge >= parents.at(-1) + 1 || fraction < 0 || fraction > 1)
            throw new RangeError('Contact foot must lie on an old edge');
        const tool = fields[edge].tools.find(t => t.id === record.owner);
        if (!tool) throw new RangeError('Contact owner must occupy its old edge');
        const x0 = source.data.coordinates[edge], x1 = source.data.coordinates[edge + 1], x = mix(x0, x1, fraction);
        const label = tool.materialMap.sStart + tool.materialMap.dsDx * (x - x0);
        errors.materialLabel = Math.max(errors.materialLabel, Math.abs(finite(record.s, 'persistent material label') - label));
        const children = parents.map((p, i) => p === edge ? i : -1).filter(i => i >= 0);
        // A split-point foot belongs to the child to its right, except the
        // old edge's distal endpoint, which remains on its last child.
        const child = children.find(i => x >= target.data.coordinates[i] && x < target.data.coordinates[i + 1]) ?? children.at(-1);
        const f = (x - target.data.coordinates[child]) / (target.data.coordinates[child + 1] - target.data.coordinates[child]);
        const p = mixVector(source.data.positions[edge], source.data.positions[edge + 1], fraction);
        const q = mixVector(target.data.positions[child], target.data.positions[child + 1], f);
        errors.position = Math.max(errors.position, distance(p, q));
        const force = vector(record.worldForce, 3, 'contact worldForce'), couple = vector(record.worldCouple, 3, 'contact worldCouple');
        // worldCouple is the complete couple about the centerline foot,
        // including any lever arm to a surface contact point.
        const ma = cross(p, force).map((v, i) => v + couple[i]), mb = cross(q, force).map((v, i) => v + couple[i]);
        errors.torque = Math.max(errors.torque, distance(ma, mb));
        for (let axis = 0; axis < 3; axis++) for (let end = 0; end < 2; end++) {
            before[source.layout.positions[edge + end] + axis] += (end ? fraction : 1 - fraction) * force[axis];
            after[target.layout.positions[child + end] + axis] += (end ? f : 1 - f) * force[axis];
        }
        return { ...structuredClone(record), edge: child, fraction: f, s: record.s, worldForce: force, worldCouple: couple };
    });
    const pulled = pullback(after, map, source.layout.dofCount);
    errors.force = Math.max(errors.force, gradientErrors(before, pulled, source.layout).force);
    return { records: output, sourceForce: before, targetForce: after, pulledForce: pulled,
        coupleInterpretation: 'world couple about centerline foot; unchanged as a point wrench',
        spinJacobianReconstruction: 'caller-owned; no transverse couple is silently projected onto a scalar spin' };
}

/** Restrict a transferred material field at a one-sided edge fraction.
 * The explicit piecewise-constant DER directors and unwrapped phase retain
 * jumps at original hinges. They are not a reconstructed smooth twist field.
 * Returned kinematicsInput is directly usable as one evaluateCompositeKinematics
 * tool input when the caller supplies its actual q_t and q_x. Unknown angular
 * derivatives/3-D angular velocity remain null; theta_t is never inferred from
 * a same-time mesh change. Known axial spin includes u*theta_x AND the frame
 * connection. No bending angular velocity is inferred from that scalar.
 */
export function sampleCompositeTransferredMaterial(transfer, { edge, id, fraction }) {
    if (!transfer.accepted) throw new RangeError('Only an accepted transfer can be sampled');
    if (!Number.isInteger(edge) || edge < 0 || edge >= transfer.inertiaEdges.length ||
        !(finite(fraction, 'fraction') >= 0 && fraction <= 1)) throw new RangeError('Sample must lie on one transferred edge');
    const t = transfer.inertiaEdges[edge].tools.find(t => t.id === id), o = transfer.orientation[edge].tools.find(t => t.id === id);
    if (!t || !o) throw new RangeError('Sampled tool must occupy the selected edge');
    const data = transfer.state.data, length = data.coordinates[edge + 1] - data.coordinates[edge];
    const input = kinematicInput(t, fraction), evaluated = axialRates(t, fraction);
    return { id, s: t.materialMap.sStart + t.materialMap.dsDx * length * fraction,
        position: mixVector(data.positions[edge], data.positions[edge + 1], fraction), orientation: structuredClone(o),
        oldMaterialVelocity: mixVector(...t.oldMaterialVelocities, fraction),
        oldAngularVelocity: t.oldAngularVelocities === null ? null : mixVector(...t.oldAngularVelocities, fraction),
        kinematicsInput: input, materialSpin: evaluated.spin, materialFrameSpin: evaluated.frameSpin };
}

/** Same time, unchanged affine material maps, REFINEMENT ONLY. Every old
 * coordinate and every explicit boundary must occur exactly in coordinates.
 * All old position nodes remain, with affine child positions; each old edge
 * spin is copied to ALL its children. Position/spin prolongation is returned
 * as sparse rows for state/virtual-work comparison, NOT automatically as
 * future physical boundary conditions. spinDofMap identifies each old spin's
 * children and its proximal/distal child; callers choose the intended physical
 * support. A prescribed old edge field covers all children, while a proximal
 * handle boundary convention selects only proximalDof.
 *
 * targetTools explicitly supplies freshly integrated target dual-cell material
 * providers (or an explicitly constant material); old mesh caches are never
 * automatically recycled. inertiaEdges is Kinematics' prepared old material
 * velocity field with constant rho, affine s, linear s_t/v_old per old edge.
 * Optional angularKinematics contains linear endpoint thetaDt/thetaDx and
 * frameSpin:{dt,dx}, or a material-path frameSpin endpoint pair. Optional
 * oldAngularVelocities carries a full linear 3-D field independently.
 *
 * contactHistory contains persistent point records: id,edge,fraction,owner,s,
 * worldForce,worldCouple,payload. Each record maps ONCE, with an owned payload;
 * normal force is never treated as a density to split among children.
 *
 * Every criterion needs an explicit nonnegative ABSOLUTE tolerance. Independent
 * coarse/fine elastic assemblies compare E and P^T g, plus fine detail-mode
 * forces/torques. This is a sampled discretization-change estimate, NOT exact
 * DER energy preservation or a bound on an arbitrary smooth curve. No dt,
 * feed, contact recertification, coarsening or new equilibrium is performed.
 * Failure returns the original state identity, with zero committed history.
 */
export function refineCompositeState(state, { coordinates, boundaries, targetTools, inertiaEdges, contactHistory, tolerances,
    elementBackend = 'wasm' }) {
    const tolerance = Object.fromEntries(CRITERIA.map(key => {
        const value = finite(tolerances?.[key], `${key} tolerance`);
        if (value < 0) throw new RangeError(`${key} tolerance must be nonnegative`);
        return [key, value];
    }));
    const errors = Object.fromEntries(CRITERIA.map(key => [key, 0]));
    const diagnostics = { tolerance, errors, historyCommits: 0, timeAdvanced: 0, energyExact: false,
        geometryInterpretation: 'restriction of the old piecewise-linear field, not an arbitrary smooth curve',
        orientationInterpretation: 'piecewise-constant edge material directors with preserved original hinge winding' };
    const reject = (status, error = null) => ({ accepted: false, status, state, diagnostics: { ...diagnostics, error: error?.message ?? null } });
    try {
        const allowed = new Set(['data', 'layout', 'lengthMultipliers', 'time', 'step', 'materialVelocities', 'torsionMode', 'contactHistory']);
        if (Object.keys(state).some(key => !allowed.has(key))) throw new TypeError('Unrecognized state history needs an explicit transfer adapter');
        finite(state.time, 'unchanged state time');
        if (!Number.isSafeInteger(state.step) || state.step < 0) throw new TypeError('State step must be a nonnegative safe integer');
        if (state.contactHistory !== undefined && contactHistory !== state.contactHistory)
            throw new TypeError('Pass the existing state.contactHistory; do not silently replace accepted point history');
        const oldX = increasing(state.data.coordinates, 'old coordinates'), newX = increasing(coordinates, 'refined coordinates');
        if (oldX[0] !== newX[0] || oldX.at(-1) !== newX.at(-1) || oldX.some(x => !newX.includes(x)))
            throw new RangeError('Refinement must retain every old node exactly and preserve both endpoints');
        if (!Array.isArray(boundaries) || boundaries.some(x => !Number.isFinite(x) || !oldX.includes(x) || !newX.includes(x)))
            throw new RangeError('Every explicit exact boundary must already exist and remain in the mesh');
        const sourceLayout = createCompositeChainLayout(state.layout.edgeToolIds);
        if (sourceLayout.nodeCount !== oldX.length) throw new RangeError('Old coordinates must match the layout');
        const source = { ...state, layout: sourceLayout }, fields = normalizedFields(inertiaEdges, sourceLayout);
        let cursor = 0;
        const parents = newX.slice(0, -1).map((x, i) => {
            while (x >= oldX[cursor + 1]) cursor++;
            if (newX[i + 1] > oldX[cursor + 1]) throw new RangeError('A refined edge cannot cross an original node');
            return cursor;
        });
        const layout = createCompositeChainLayout(parents.map(p => sourceLayout.edgeToolIds[p]));
        const newFields = restriction(fields, oldX, newX, parents), oldNodes = new Map(oldX.map((x, i) => [x, i]));
        const newNodes = new Map(newX.map((x, i) => [x, i])), prolongation = new Array(layout.dofCount);
        const positions = newX.map((x, node) => {
            const old = oldNodes.get(x), parent = parents[Math.min(node, parents.length - 1)];
            const f = old === undefined ? (x - oldX[parent]) / (oldX[parent + 1] - oldX[parent]) : 0;
            for (let axis = 0; axis < 3; axis++) prolongation[layout.positions[node] + axis] = old === undefined
                ? [[sourceLayout.positions[parent] + axis, 1 - f], [sourceLayout.positions[parent + 1] + axis, f]]
                : [[sourceLayout.positions[old] + axis, 1]];
            return old === undefined ? mixVector(vector(state.data.positions[parent], 3, 'old position'), vector(state.data.positions[parent + 1], 3, 'old position'), f)
                : vector(state.data.positions[old], 3, 'old position');
        });
        const sourceW = assembleCompositeChain(state.data, createCompositeChainWorkspace(sourceLayout, { elementBackend }));
        const sourceOrientation = orientation(state.data, sourceLayout, sourceW.evaluatedReferenceTwists);
        const sourceById = new Map(state.data.tools.map(t => [t.id, t]));
        for (const { vertex: i, tools: ids } of sourceLayout.hinges) for (const id of ids) {
            const tool = sourceById.get(id), scale = typeof tool.dsDx === 'function' ? tool.dsDx(oldX[i]) : tool.dsDx;
            const a = fields[i - 1].tools.find(t => t.id === id).materialMap.dsDx, b = fields[i].tools.find(t => t.id === id).materialMap.dsDx;
            if (scale !== a || a !== b) throw new RangeError('Old elastic dsDx and prepared material maps must agree at each hinge');
        }
        if (!Array.isArray(targetTools) || targetTools.length !== sourceById.size || new Set(targetTools.map(t => t.id)).size !== sourceById.size ||
            targetTools.some(t => !sourceById.has(t.id))) throw new RangeError('Explicit target tools must match original material ownership');
        const tools = [...layout.spins.keys()].map(id => {
            const old = sourceById.get(id), angles = new Float64Array(parents.length).fill(NaN), anchors = new Float64Array(newX.length - 2).fill(NaN);
            for (const [edge, parent] of parents.entries()) if (layout.edgeToolIds[edge].includes(id)) {
                angles[edge] = old.angles[parent]; prolongation[layout.spins.get(id)[edge]] = [[sourceLayout.spins.get(id)[parent], 1]];
            }
            for (const { vertex: i, tools: ids } of layout.hinges) if (ids.includes(id)) {
                const oldNode = oldNodes.get(newX[i]);
                anchors[i - 1] = oldNode === undefined ? 0 : finite(old.referenceTwists?.[oldNode - 1] ?? 0, 'old winding anchor');
            }
            return cachedTool(targetTools.find(t => t.id === id), id, layout, newX, angles, anchors, newFields);
        });
        const data = { positions, coordinates: Float64Array.from(newX), reference: parents.map(parent => structuredClone(state.data.reference[parent])), tools };
        const oldLambda = vector(state.lengthMultipliers, oldX.length - 1, 'length multipliers'), lengthMultipliers = Float64Array.from(parents, p => oldLambda[p]);
        const target = { data, layout, lengthMultipliers, time: state.time, step: state.step, torsionMode: state.torsionMode };
        const targetW = assembleCompositeChain(data, createCompositeChainWorkspace(layout, { elementBackend }));
        const targetOrientation = orientation(data, layout, targetW.evaluatedReferenceTwists), pulled = pullback(targetW.gradient, prolongation, sourceLayout.dofCount);
        const elasticError = gradientErrors(sourceW.gradient, pulled, sourceLayout);
        errors.energy = Math.abs(targetW.energy - sourceW.energy); errors.force = elasticError.force; errors.torque = elasticError.torque;
        let detailForce = 0, detailTorque = 0;
        newX.forEach((x, i) => { if (!oldNodes.has(x)) detailForce = Math.max(detailForce, Math.hypot(...targetW.gradient.subarray(layout.positions[i], layout.positions[i] + 3))); });
        for (const [id, offsets] of sourceLayout.spins) offsets.forEach((dof, parent) => {
            if (dof < 0) return;
            const children = parents.map((p, i) => p === parent ? i : -1).filter(i => i >= 0), sum = children.reduce((s, child) => s + targetW.gradient[layout.spins.get(id)[child]], 0);
            for (const child of children) detailTorque = Math.max(detailTorque, Math.abs(targetW.gradient[layout.spins.get(id)[child]] -
                (newX[child + 1] - newX[child]) / (oldX[parent + 1] - oldX[parent]) * sum));
        });
        errors.force = Math.max(errors.force, detailForce); errors.torque = Math.max(errors.torque, detailTorque);
        const oldTotals = totals(fields, oldX), newTotals = totals(newFields, newX);
        for (const [id, a] of oldTotals) {
            const b = newTotals.get(id);
            errors.mass = Math.max(errors.mass, Math.abs(a.mass - b.mass)); errors.momentum = Math.max(errors.momentum, distance(a.momentum, b.momentum));
            errors.kineticEnergy = Math.max(errors.kineticEnergy, Math.abs(a.kineticEnergy - b.kineticEnergy));
        }
        for (const [child, parent] of parents.entries()) for (const t of newFields[child].tools) {
            const old = fields[parent].tools.find(a => a.id === t.id), a = sourceOrientation[parent].tools.find(o => o.id === t.id), b = targetOrientation[child].tools.find(o => o.id === t.id);
            // Unit-vector chord differences converted to angular discrepancies.
            errors.rotation = Math.max(errors.rotation, 2 * Math.asin(Math.min(1, distance(a.director1, b.director1) / 2)),
                2 * Math.asin(Math.min(1, distance(a.director2, b.director2) / 2)), Math.abs(a.unwrappedPhase - b.unwrappedPhase));
            for (const f of [0, .5, 1]) {
                const x = mix(newX[child], newX[child + 1], f), oldF = (x - oldX[parent]) / (oldX[parent + 1] - oldX[parent]);
                errors.position = Math.max(errors.position, distance(mixVector(positions[child], positions[child + 1], f), mixVector(state.data.positions[parent], state.data.positions[parent + 1], oldF)));
                errors.materialLabel = Math.max(errors.materialLabel, Math.abs(t.materialMap.sStart + t.materialMap.dsDx * (x - newX[child]) -
                    (old.materialMap.sStart + old.materialMap.dsDx * (x - oldX[parent]))));
                errors.velocity = Math.max(errors.velocity, distance(mixVector(...t.oldMaterialVelocities, f), mixVector(...old.oldMaterialVelocities, oldF)));
                if (t.oldAngularVelocities !== null) errors.angularVelocity = Math.max(errors.angularVelocity,
                    distance(mixVector(...t.oldAngularVelocities, f), mixVector(...old.oldAngularVelocities, oldF)));
                if (t.angularKinematics !== null) {
                    compareAngularFields(t, f, old, oldF, errors);
                }
            }
        }
        // Labels and accepted velocity history must be the declared old field,
        // with independent one-sided values allowed at an original hinge.
        fields.forEach((e, edge) => e.tools.forEach(t => {
            if (edge && fields[edge - 1].tools.some(p => p.id === t.id)) {
                const previous = fields[edge - 1].tools.find(p => p.id === t.id);
                errors.materialLabel = Math.max(errors.materialLabel, Math.abs(t.materialMap.sStart - previous.materialMap.sStart -
                    previous.materialMap.dsDx * (oldX[edge] - oldX[edge - 1])));
            }
            if (state.materialVelocities !== null && state.materialVelocities !== undefined) {
                const old = state.materialVelocities[edge]?.tools.find(p => p.id === t.id);
                if (!old) throw new TypeError('Accepted material velocity history is missing an edge/tool');
                errors.materialLabel = Math.max(errors.materialLabel, Math.abs(finite(old.sStart, 'old velocity sStart') - t.materialMap.sStart),
                    Math.abs(finite(old.sEnd, 'old velocity sEnd') - t.materialMap.sStart - t.materialMap.dsDx * (oldX[edge + 1] - oldX[edge])));
                for (const end of [0, 1]) errors.velocity = Math.max(errors.velocity, distance(vector(old.velocities[end], 3, 'accepted material velocity'), t.oldMaterialVelocities[end]));
                if (old.angularVelocity != null) {
                    if (!t.oldAngularVelocities || old.angularVelocity.length !== 2) throw new TypeError('Known accepted 3-D angular velocity must be explicitly supplied');
                    for (const end of [0, 1]) errors.angularVelocity = Math.max(errors.angularVelocity,
                        distance(vector(old.angularVelocity[end], 3, 'accepted angular velocity'), t.oldAngularVelocities[end]));
                }
                if (old.materialSpin != null || old.frameSpin != null || old.angularKinematics != null) {
                    if (t.angularKinematics === null || old.angularKinematics == null)
                        throw new TypeError('Known accepted axial history needs its derivative fields; scalar endpoints do not define arbitrary material-path spin');
                    const acceptedAngular = { ...t, angularKinematics: angular(old.angularKinematics) };
                    for (const f of [0, .5, 1]) {
                        compareAngularFields(acceptedAngular, f, t, f, errors);
                    }
                    for (const [key, evaluatedKey] of [['materialSpin', 'spin'], ['frameSpin', 'frameSpin']]) if (old[key] != null) {
                        const values = ends(old[key], `accepted ${key}`);
                        for (const end of [0, 1]) errors.angularVelocity = Math.max(errors.angularVelocity, Math.abs(values[end] - axialRates(t, end)[evaluatedKey]));
                    }
                }
            }
        }));
        const lengthBefore = lengths(state.data, sourceLayout, oldLambda), lengthAfter = lengths(data, layout, lengthMultipliers);
        const lengthPulled = pullback(lengthAfter.gradient, prolongation, sourceLayout.dofCount), lengthError = gradientErrors(lengthBefore.gradient, lengthPulled, sourceLayout);
        errors.work = Math.abs(lengthBefore.work - lengthAfter.work); errors.force = Math.max(errors.force, lengthError.force);
        const contact = pointHistories(contactHistory, source, target, fields, parents, prolongation, errors);
        Object.assign(diagnostics, { elastic: { sourceEnergy: sourceW.energy, targetEnergy: targetW.energy, sourceGradient: sourceW.gradient.slice(),
            targetGradient: targetW.gradient.slice(), pulledGradient: pulled, ...elasticError, detailForce, detailTorque },
            massIntegrals: { source: oldTotals, target: newTotals, rule: 'Gauss-2, exact for declared affine velocity and constant rho*dsDx' },
            length: { sourceWork: lengthBefore.work, targetWork: lengthAfter.work, sourceGradient: lengthBefore.gradient,
                targetGradient: lengthAfter.gradient, pulledGradient: lengthPulled, force: lengthError.force }, contact,
            sourceNodes: oldX.length, targetNodes: newX.length, addedNodes: newX.length - oldX.length });
        if (Object.values(errors).some(v => !Number.isFinite(v))) throw new RangeError('Nonfinite transfer comparison');
        diagnostics.failedCriteria = CRITERIA.filter(key => errors[key] > tolerance[key]);
        if (diagnostics.failedCriteria.length) return reject('transfer-tolerance-rejected');
        target.contactHistory = contact.records;
        target.materialVelocities = newFields.map((e, edge) => ({ edge, tools: e.tools.map(t => ({ id: t.id,
            sStart: t.materialMap.sStart, sEnd: t.materialMap.sStart + t.materialMap.dsDx * (newX[edge + 1] - newX[edge]),
            velocities: t.oldMaterialVelocities.map(v => [...v]), interpretation: 'physical-material-velocity',
            angularVelocity: t.oldAngularVelocities === null ? null : structuredClone(t.oldAngularVelocities),
            angularKinematics: structuredClone(t.angularKinematics),
            materialSpin: t.angularKinematics === null ? null : [0, 1].map(f => axialRates(t, f).spin),
            frameSpin: t.angularKinematics === null ? null : [0, 1].map(f => axialRates(t, f).frameSpin) })) }));
        diagnostics.historyCommits = 1;
        return { accepted: true, status: 'accepted-same-time-refinement', state: target, diagnostics,
            inertiaEdges: newFields, orientation: targetOrientation, parentEdges: Int32Array.from(parents), prolongation,
            oldNodeToNewNode: Int32Array.from(oldX, x => newNodes.get(x)),
            spinDofMap: [...sourceLayout.spins].flatMap(([id, rows]) => Array.from(rows, (dof, parent) => {
                if (dof < 0) return null;
                const childDofs = parents.map((p, i) => p === parent ? layout.spins.get(id)[i] : -1).filter(i => i >= 0);
                return { id, oldDof: dof, childDofs, proximalDof: childDofs[0], distalDof: childDofs.at(-1) };
            }).filter(Boolean)),
            angularHistory: 'Explicit angular derivative fields remain in inertiaEdges; no angular rate is inferred from remeshing' };
    } catch (error) { return reject('invalid-transfer', error); }
}
