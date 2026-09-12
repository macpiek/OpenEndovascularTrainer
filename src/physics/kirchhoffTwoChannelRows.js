import { evaluateBendTwistConstraint, rotateVectorByQuaternion } from './discreteKirchhoffRod.js';
import { measureKirchhoffTwoChannelMaterial } from './kirchhoffTwoChannelMotion.js';
import { KIRCHHOFF_BIAS_RELEASE, kirchhoffBiasReleaseIssue, retireKirchhoffBiasNormal,
    transportKirchhoffBiasNormal, refreshKirchhoffBiasReactionAnchors,
    appendKirchhoffBiasReleaseRows, measureKirchhoffBiasReleases } from './kirchhoffTwoChannelRelease.js';

const XYZ = ['x', 'y', 'z'], Q = [...XYZ, 'w'];
const NORMALS = new Set(['normal', 'wall', 'tool', 'sheath', 'split-sweep', 'split-point-wall']);
const FRICTION = new Set(['external-friction', 'split-wall-friction']);
const bodiesOf = joint => [joint.innerBody, joint.outerBody];
const valueOf = entry => entry.bank[entry.slot];
const same = (a, b) => a.length === b.length && a.every((v, i) => Object.is(v, b[i]));
const finite = (v, label) => { if (!Number.isFinite(v)) throw new RangeError(label + ' must be finite'); return v; };
const pairMaps = () => [new Map(), new Map()];

function stateOf(joint) {
    const s = joint._splitMotion, m = s?.twoChannel, state = m?.rows;
    if (!state || !['physical', 'complete'].includes(s.phase) || s.dt !== m.dt || s.step !== m.step || s.biasBank !== state.bank)
        throw new Error('Two-channel rows require their original physical split step and bias bank');
    for (const [side, body] of bodiesOf(joint).entries()) {
        const l = m.layout[side];
        if (body !== state.bodies[side] || body.count !== l.count || body.segmentCount !== l.segmentCount ||
            Math.max(0, body.activeStart) !== l.start || Math.min(body.segmentCount, body.activeEnd) !== l.end || Boolean(body.sleeping) !== l.sleeping)
            throw new Error('Two-channel row body/material layout changed');
    }
    return state;
}

/** Once after beginKirchhoffTwoChannelMotion. These are real independent beta
 * banks, owned by the existing trial snapshot. No physical array, velocity,
 * target or history is reset and no legacy physical/bias phase swap occurs. */
export function beginKirchhoffTwoChannelRows(joint, world) {
    const s = joint._splitMotion, m = s?.twoChannel, bodies = bodiesOf(joint);
    if (!m || s.phase !== 'physical' || s.dt !== m.dt || s.step !== m.step || world.fixedDt !== s.dt)
        throw new Error('Begin two-channel motion before rows');
    if (m.rows || s.biasBank || s.bank) throw new Error('Two-channel row banks must not restart within a timestep');
    const boundary = { controls: bodies.map(b => new Float64Array(b.count * 3)), sheaths: new Map() };
    for (const sheath of world.sheaths ?? []) boundary.sheaths.set(sheath, bodies.map(b => ({
        lambda: new Float64Array(b.count), normal: new Float64Array(b.count * 3), active: new Uint8Array(b.count) })));
    const tools = [...(world.toolContacts ?? [])];
    const bank = { bodies: bodies.map(b => ({ wallLambda: new Float64Array(b.segmentCount), orientationControlLambda: new Float64Array(3) })),
        joints: { _coupledBoundaries: boundary }, contacts: [], tools: tools.map(tool => ({ tool, lambdas: new Float64Array(tool.lambdas.length) })),
        pointWalls: pairMaps(), sweeps: pairMaps() };
    // References to actual physical owners, used by existing sheath auditing
    // and normal stats. Their numeric arrays remain installed on the owners.
    s.bank = { bodies: bodies.map(b => ({ wallLambda: b.wallLambda, orientationControlLambda: b.orientationControlLambda })),
        joints: { _coupledBoundaries: joint._coupledBoundaries }, contacts: [],
        tools: tools.map(tool => ({ tool, lambdas: tool.lambdas, reactions: tool._jointReactions })), sweeps: s.sweeps, pointWalls: s.pointWalls };
    s.biasBank = bank;
    m.rows = { bodies, bank, owners: new Map(), entries: [], releases: [], releaseVersion: 0, historyVersion: 0, version: 0, pending: null };
    return m.rows;
}

function quaternion(body, segment, target = false) {
    const q = Object.fromEntries(Q.map(a => [a, finite(target ? body['orientationControl' + a.toUpperCase()] :
        body['orientation' + a.toUpperCase()][segment], 'Orientation frame')]));
    const n = Math.hypot(...Object.values(q));
    if (!(n > 0) || !Number.isFinite(n)) throw new RangeError('Nonzero finite orientation frame required');
    for (const a of Q) q[a] /= n;
    return q;
}
function orientationDifference(joint, side, segment) {
    const body = bodiesOf(joint)[side], target = quaternion(body, segment, true), zero = { x: 0, y: 0, z: 0 };
    const g = evaluateBendTwistConstraint(target, quaternion(body, segment), zero, {}).strain;
    const p = evaluateBendTwistConstraint(target, quaternion(joint._splitMotion.twoChannel.physicalPose[side], segment), zero, {}).strain;
    // Both logs are in the SAME native target frame. Log(qP^-1 qG) is a
    // different nonlinear quantity and cannot replace this difference.
    return XYZ.map(a => finite(g[a] - p[a], 'Orientation bias strain'));
}
function gradientsOf(record) {
    if (record.normalGradients) return record.normalGradients;
    const gradients = [];
    for (let side = 0; side < 2; side++) {
        const prefix = side ? '_outer' : '_inner', nodes = record[prefix + 'NodeIndices'];
        const weights = record[prefix + 'NodeWeights'] ?? record[side ? 'outerWeights' : 'innerWeights'];
        const count = nodes ? record[prefix + 'NodeCount'] : 2;
        for (let i = 0; i < count; i++) for (let axis = 0; axis < 3; axis++) gradients.push({ side,
            dof: (nodes ? nodes[i] : record[prefix + 'SegmentIndex'] + i) * 6 + axis,
            value: (side ? 1 : -1) * weights[i] * record.normal[axis] });
    }
    return gradients;
}
function freezeGradients(joint, gradients) {
    if (!Array.isArray(gradients)) throw new TypeError('A runtime row requires explicit gradients');
    const sums = new Map(), bodies = bodiesOf(joint);
    for (const g of gradients) {
        if ((g.side !== 0 && g.side !== 1) || !Number.isInteger(g.dof) || g.dof < 0 || g.dof >= bodies[g.side].count * 6)
            throw new RangeError('Invalid runtime row gradient');
        const key = g.side + ':' + g.dof;
        sums.set(key, (sums.get(key) ?? 0) + finite(g.value, 'Row gradient'));
    }
    return [...sums].map(([key, value]) => ({ side: Number(key.split(':')[0]), dof: Number(key.split(':')[1]), value }))
        .sort((a, b) => a.side - b.side || a.dof - b.dof);
}
function worldGradients(joint, gradients) {
    const angular = new Map(), result = [];
    for (const g of gradients) {
        const axis = g.dof % 6, node = Math.floor(g.dof / 6);
        if (axis < 3) { result.push({ ...g }); continue; }
        const key = g.side + ':' + node;
        if (!angular.has(key)) angular.set(key, { side: g.side, node, local: { x: 0, y: 0, z: 0 } });
        angular.get(key).local[XYZ[axis - 3]] += g.value;
    }
    for (const { side, node, local } of angular.values()) {
        const world = rotateVectorByQuaternion(quaternion(bodiesOf(joint)[side], node), local, {});
        XYZ.forEach((a, axis) => result.push({ side, dof: node * 6 + 3 + axis, value: world[a] }));
    }
    return result.sort((a, b) => a.side - b.side || a.dof - b.dof);
}
function rowSpec(joint, row, kind = row.kind) {
    const bodies = bodiesOf(joint), normal = kind === 'normal', side = row.side, node = row.node;
    const body = bodies[side], gradients = freezeGradients(joint, normal ? gradientsOf(row) : row.gradients);
    let owner = normal ? row.manifoldContact : kind === 'sheath' ? row.sheathWitness?.geometry?.sheath : kind === 'tool' ? row.owner : body;
    if (!owner || (typeof owner !== 'object' && typeof owner !== 'function')) throw new TypeError('Missing stable runtime row owner');
    if (!normal && kind !== 'tool' && !body) throw new RangeError('Runtime boundary requires a body side');
    const component = row.component ?? -1, segment = row.segment ?? node;
    const index = kind === 'orientation-control' ? segment : node;
    if (!normal && (!Number.isInteger(index) || index < 0)) throw new RangeError('Invalid stable row index');
    if (kind === 'control' || kind === 'orientation-control') {
        if (![0, 1, 2].includes(component)) throw new RangeError('Invalid control component');
    }
    // There is one orientation beta vector per body; moving its controlled
    // segment changes its identity, never creates a second alias to the bank.
    const key = normal ? 'normal' : kind === 'orientation-control' ? `${kind}:${side}:${component}` : `${kind}:${side ?? -1}:${index}:${component}`;
    const alpha = finite(normal ? row._normalAlpha ?? 0 : row.alpha ?? 0, 'Row compliance');
    if (alpha < 0) throw new RangeError('Negative row compliance');
    const identity = [kind, alpha, row.feature, row.featureId], semanticIdentity = identity.slice(), guards = [];
    const guard = (object, key, at = null) => {
        const value = at === null ? object[key] : object[key]?.[at];
        guards.push({ object, key, at, value }); return value;
    };
    if (normal) {
        identity.push(...['id', 'innerMaterialSegmentId', 'outerMaterialSegmentId', 'feature', '_manifold'].map(k => guard(owner, k)), row.kind);
        semanticIdentity.push(...identity.slice(4));
        for (const prefix of ['_inner', '_outer']) {
            const nodeIds = row[prefix + 'NodeIndices'], count = nodeIds ? row[prefix + 'NodeCount'] : 2;
            const weights = row[prefix + 'NodeWeights'] ?? row[prefix === '_inner' ? 'innerWeights' : 'outerWeights'];
            identity.push(prefix, count);
            for (let i = 0; i < count; i++) identity.push(nodeIds ? nodeIds[i] : row[prefix + 'SegmentIndex'] + i, weights?.[i]);
        }
    }
    // Material labels, rather than a borrowed collector row, own reactions.
    const nodes = new Map();
    for (const g of gradients) nodes.set(g.side + ':' + Math.floor(g.dof / 6), [g.side, Math.floor(g.dof / 6)]);
    for (const [s, n] of [...nodes.values()].sort((a, b) => a[0] - b[0] || a[1] - b[1]))
        identity.push(s, n, guard(bodies[s], 'materialCoordinate', n), guard(bodies[s], 'nodeRadius', n));
    const historyGuards = guards.filter(g => g.key === 'materialCoordinate' || g.key === 'nodeRadius').map(g => ({ ...g }));
    const remember = (object, key, at = null) => historyGuards.push({ object, key, at, value: at === null ? object[key] : object[key]?.[at] });
    for (const g of gradients) {
        const b = bodies[g.side], n = Math.floor(g.dof / 6), axis = g.dof % 6;
        remember(b, axis < 3 ? 'inverseMass' : 'inverseInertia' + (axis - 2), n);
        if (axis >= 3) { remember(b, 'orientationControlCompliance'); remember(b, 'orientationControlSegment'); }
    }
    let strain;
    if (kind === 'control') {
        if (node >= body.count) throw new RangeError('Control node outside body');
        identity.push(guard(body, 'controlEnabled', node), body.activeStart, body.activeEnd,
            ...XYZ.map(a => guard(body, 'control' + a.toUpperCase(), node)), guard(body, 'controlCompliance', node));
        strain = body[XYZ[component]][node] - joint._splitMotion.twoChannel.physicalPose[side][XYZ[component]][node];
    } else if (kind === 'orientation-control') {
        if (segment >= body.segmentCount) throw new RangeError('Orientation segment outside body');
        identity.push(segment, guard(body, 'orientationControlSegment'), guard(body, 'orientationControlCompliance'), body.activeStart, body.activeEnd,
            ...Q.map(a => guard(body, 'orientationControl' + a.toUpperCase())));
        strain = orientationDifference(joint, side, segment)[component];
    } else {
        strain = normal ? row._splitActualGap : row._splitActualStrain;
        if (kind === 'sheath') {
            const g = row.sheathWitness.geometry;
            for (const k of ['origin', 'axis', 'included', 'start', 'end']) { identity.push(...(g[k] ?? [])); semanticIdentity.push(...(g[k] ?? [])); }
            identity.push(g.minimumAxial, g.maximumAxial, g.innerRadius); semanticIdentity.push(g.minimumAxial, g.maximumAxial, g.innerRadius);
            for (const k of ['startX', 'startY', 'startZ', 'axisX', 'axisY', 'axisZ', 'proximalExtension', 'length', 'innerRadius']) {
                guard(owner, k); remember(owner, k);
            }
        } else if (kind === 'split-sweep') { identity.push(...(row.point ?? []), ...(row.n ?? [])); semanticIdentity.push(...(row.point ?? []), ...(row.n ?? [])); }
        else if (kind === 'wall') identity.push(body.wallT[node]);
        else if (kind === 'tool') {
            identity.push(row.bodyA, row.bodyB, row.segmentA, row.segmentB, row.tA, row.tB);
            semanticIdentity.push(row.bodyA, row.bodyB, row.segmentA, row.segmentB);
        }
    }
    return { owner, key, kind, side, node, segment, component, identity, semanticIdentity, guards, historyGuards, gradients, worldGradients: worldGradients(joint, gradients), alpha, strain: finite(strain, 'Raw bias strain'),
        physicalStrain: normal ? row.gap : row.strain, physicalLambda: normal ? owner.normalLambda : row.lambda ?? 0,
        unilateral: NORMALS.has(kind) };
}
function newEntry(state, spec) {
    const { bank } = state, { kind, side, node, owner, component } = spec;
    let storage, slot;
    if (kind === 'normal') {
        storage = { contact: owner, normalLambda: 0 }; slot = 'normalLambda'; bank.contacts.push(storage);
    } else if (kind === 'wall') { storage = bank.bodies[side].wallLambda; slot = node; }
    else if (kind === 'control') { storage = bank.joints._coupledBoundaries.controls[side]; slot = node * 3 + component; }
    else if (kind === 'orientation-control') { storage = bank.bodies[side].orientationControlLambda; slot = component; }
    else if (kind === 'sheath') {
        const sheath = bank.joints._coupledBoundaries.sheaths.get(owner);
        if (!sheath) throw new Error('Sheath was not present when bias banks began');
        storage = sheath[side].lambda; slot = node;
    } else if (kind === 'tool') {
        const tool = bank.tools.find(t => t.tool === owner);
        if (!tool || owner.lambdas.length !== tool.lambdas.length) throw new Error('Tool membership/layout changed');
        storage = tool.lambdas; slot = node;
    } else {
        const map = (kind === 'split-sweep' ? bank.sweeps : bank.pointWalls)[side];
        storage = map.get(node);
        if (!storage) { storage = { lambda: 0 }; map.set(node, storage); }
        slot = 'lambda';
    }
    if (storage[slot] === undefined) throw new RangeError('Bias bank row index outside owner');
    const entry = { owner, key: spec.key, kind, side, node, segment: spec.segment, component, bank: storage, slot,
        identity: spec.identity.slice(), semanticIdentity: spec.unilateral ? spec.semanticIdentity.slice() : null, difference: null, reactionAnchor: null,
        alpha: spec.alpha, historyGuards: spec.historyGuards.map(g => ({ ...g })), frozenGradients: spec.gradients.map(g => ({ ...g })),
        frozenWorldGradients: spec.worldGradients.map(g => ({ ...g })) };
    let owned = state.owners.get(owner);
    if (!owned) { owned = new Map(); state.owners.set(owner, owned); }
    owned.set(spec.key, entry); state.entries.push(entry);
    return entry;
}
function issue(reason, entry) {
    return { reason, kind: entry.kind, side: entry.side, node: entry.node, segment: entry.segment,
        component: entry.component, key: entry.key, beta: entry.bank ? valueOf(entry) : 0 };
}
function frictionIndices(rows, groups) {
    const indices = new Set();
    for (const group of groups) for (const i of group.rows ?? group.rowIndices ?? []) {
        if (!Number.isInteger(i) || i < 0 || i >= rows.length || indices.has(i)) throw new RangeError('Invalid or repeated friction row index');
        if (NORMALS.has(rows[i].kind) || ['control', 'orientation-control', 'fold'].includes(rows[i].kind))
            throw new Error('A geometric/control row cannot become friction by group membership');
        indices.add(i);
    }
    return indices;
}
function sameWrench(a, b) {
    const clean = gradients => gradients.filter(g => g.value !== 0);
    a = clean(a); b = clean(b);
    return a.length === b.length && a.every((g, i) => g.side === b[i].side && g.dof === b[i].dof && g.value === b[i].value);
}
function scan(joint, additionalRows, groups, create, geometryOnly = false) {
    const state = stateOf(joint), targets = [], descriptors = [], issues = [], seen = new Map();
    const grouped = geometryOnly ? new Set() : frictionIndices(additionalRows, groups);
    const visit = (row, kind, source, index) => {
        const spec = rowSpec(joint, row, kind);
        let entry = state.owners.get(spec.owner)?.get(spec.key);
        if (!entry && create) entry = newEntry(state, spec);
        if (entry && seen.has(entry)) { issues.push(issue('duplicate-stable-row', entry)); return; }
        if (entry) seen.set(entry, spec);
        targets.push({ ...spec, source, index, entry });
    };
    (joint.kirchhoffContacts ?? []).forEach((row, index) => visit(row, 'normal', 'contact', index));
    additionalRows.forEach((row, index) => {
        if (grouped.has(index) || FRICTION.has(row.kind)) descriptors[index] = { physical: 'physical-motion', bias: null };
        else if (row.kind === 'fold') descriptors[index] = { physical: 'pose', bias: null };
        else if (row.kind === 'tool-release') descriptors[index] = { physical: 'physical-motion', bias: null };
        else if (NORMALS.has(row.kind) || ['control', 'orientation-control'].includes(row.kind)) visit(row, row.kind, 'additional', index);
        else if (!geometryOnly) throw new Error('Unsupported two-channel runtime row: ' + row.kind);
    });
    for (const entry of state.entries) {
        const beta = finite(valueOf(entry), 'Bias multiplier');
        if (beta === 0 && !(entry.difference && valueOf(entry.difference) !== 0)) continue;
        const spec = seen.get(entry), normal = NORMALS.has(entry.kind);
        if (normal && beta < 0) throw new RangeError('Negative normal bias bank');
        const historyFailure = normal ? kirchhoffBiasReleaseIssue(entry) ?? (entry.difference ? kirchhoffBiasReleaseIssue(entry.difference) : null) : null;
        if (historyFailure) { issues.push(issue(historyFailure, entry)); continue; }
        const reason = !spec ? 'missing-loaded-bias-row' : !same(entry.identity, spec.identity) ? 'loaded-bias-row-identity-changed'
            : normal && !sameWrench(entry.frozenWorldGradients, spec.worldGradients) ? 'loaded-bias-wrench-changed' : null;
        if (!reason) continue;
        if (!normal) { issues.push(issue(reason, entry)); continue; }
        const failure = kirchhoffBiasReleaseIssue(entry) ?? (spec && entry.alpha !== spec.alpha ? 'bias-normal-compliance-changed' : null);
        if (failure) issues.push(issue(failure, entry));
        else if (spec && same(entry.semanticIdentity, spec.semanticIdentity)) transportKirchhoffBiasNormal(joint, entry, spec);
        else retireKirchhoffBiasNormal(joint, entry, reason);
    }
    for (const t of targets) {
        t.beta = t.entry ? valueOf(t.entry) : 0;
        if (t.source === 'additional') descriptors[t.index] = { physical: t.unilateral ? 'physical-motion' : 'pose',
            bias: { channel: t.unilateral ? 'pose' : 'bias-motion', strain: t.strain, alpha: t.alpha,
                lambda: t.beta, lower: t.unilateral ? 0 : -Infinity, upper: Infinity } };
    }
    for (const release of state.releases) if (valueOf(release) !== 0) {
        const failure = kirchhoffBiasReleaseIssue(release);
        if (failure) issues.push(issue(failure, release));
    }
    return { targets, descriptors, issues };
}

/** Snapshot targets before native assembly. Rows are pooled, so only copied
 * equations, semantic owner identities and independent beta storage survive.
 * Missing/semantically changed loaded normals retain the whole old reaction.
 * Continuous same-contact J changes preserve beta through an explicit world
 * wrench difference carrier. Unsupported material/history changes still block. */
export function prepareKirchhoffTwoChannelRows(joint, additionalRows = [], groups = []) {
    if (joint._splitMotion?.phase !== 'physical') throw new Error('Preparing rows requires the physical phase');
    if ([...joint._splitMotion.twoChannel.applications.values()].some(r => r.physicalApplied && !r.rowsApplied))
        throw new Error('Commit the preceding applied row batch before preparing another solve');
    const state = stateOf(joint);
    // A caller may reuse its combined array. Only our own trailing rows are
    // replaced; all existing runtime row/group offsets remain unchanged.
    while (additionalRows.at(-1)?.kind === KIRCHHOFF_BIAS_RELEASE) additionalRows.pop();
    const scanned = scan(joint, additionalRows, groups, true);
    if (!scanned.issues.length) for (const { release, row, index, beta } of appendKirchhoffBiasReleaseRows(joint, additionalRows)) {
        scanned.targets.push({ source: 'additional', index, entry: release, beta, kind: release.kind,
            owner: release.owner, side: release.side, node: release.node, component: release.component, segment: release.segment,
            release: true, alpha: 0, physicalStrain: 0, physicalLambda: 0, guards: release.historyGuards,
            gradients: row.gradients.map(g => ({ ...g })) });
        scanned.descriptors[index] = { physical: 'physical-motion', bias: { channel: 'bias-motion',
            strain: 0, alpha: 0, lambda: beta, lower: 0, upper: 0 } };
    }
    const pending = { ...scanned, version: ++state.version, historyVersion: state.historyVersion, additionalCount: additionalRows.length,
        contactCount: (joint.kirchhoffContacts ?? []).length, applicationIds: [...joint._splitMotion.twoChannel.applications.keys()],
        assembled: false, committed: false };
    state.pending = pending;
    return { ready: !pending.issues.length, issues: pending.issues.map(i => ({ ...i })), version: pending.version,
        channels(native) {
            if (stateOf(joint).pending !== pending || pending.committed) throw new Error('Stale two-channel row preparation');
            if (pending.issues.length) throw new Error('Unsupported bias history: ' + pending.issues.map(i => i.reason).join(', '));
            if (native.count - native.additionalOffset !== pending.additionalCount ||
                native.additionalOffset - native.contactOffset !== pending.contactCount) throw new Error('Runtime row counts changed after preparation');
            const material = measureKirchhoffTwoChannelMaterial(joint);
            if (!material.finite) throw new Error('Nonfinite two-channel material measurement');
            pending.originalCount = native.count;
            pending.contactOffset = native.contactOffset; pending.additionalOffset = native.additionalOffset;
            const contactTargets = pending.targets.filter(t => t.source === 'contact');
            for (const t of pending.targets) {
                const row = native.rows[(t.source === 'contact' ? native.contactOffset : native.additionalOffset) + t.index];
                if (row.alpha !== t.alpha || row.lambda !== t.physicalLambda || row.rhs !== -t.physicalStrain - t.alpha * t.physicalLambda)
                    throw new Error('Runtime equation changed after preparation');
                for (const g of t.guards) if (!Object.is(g.at === null ? g.object[g.key] : g.object[g.key]?.[g.at], g.value))
                    throw new Error('Runtime owner changed after preparation');
            }
            const descriptors = native.rows.slice(0, native.count).map((row, original) => {
                if (row.kind === 'material') return { physical: 'pose', bias: { channel: 'bias-motion',
                    strain: material.biasStrain[row.side][row.local], alpha: row.alpha,
                    lambda: material.biasLambda[row.side][row.local], lower: -Infinity, upper: Infinity } };
                if (row.kind === 'normal') {
                    const t = contactTargets[row.local];
                    if (!t) throw new Error('Missing native contact target');
                    return { physical: 'physical-motion', bias: { channel: 'pose', strain: t.strain,
                        alpha: row.alpha, lambda: t.beta, lower: 0, upper: Infinity } };
                }
                const d = pending.descriptors[original - native.additionalOffset];
                if (row.kind !== 'additional' || !d) throw new Error('Unclassified native row');
                return { physical: d.physical, bias: d.bias && { ...d.bias } };
            });
            pending.assembled = true;
            return descriptors;
        } };
}

/** Immediately after Motion's physical application AND material beta commit.
 * Exactly one new Motion receipt must exist since preparation, at precisely
 * this common scale. Prevalidation precedes every beta/history write. */
export function commitKirchhoffTwoChannelRows(joint, result) {
    if (joint._splitMotion?.phase !== 'physical') throw new Error('Committing rows requires the physical phase');
    const state = stateOf(joint), p = state.pending, m = joint._splitMotion.twoChannel;
    if (!p?.assembled || p.committed || p.issues.length) throw new Error('No uncommitted supported two-channel rows');
    if (p.historyVersion !== state.historyVersion) throw new Error('Bias force representation changed after preparation');
    if (!result?.diagnostics?.converged || !Number.isFinite(result.scale) || result.scale < 0 || result.scale > 1)
        throw new Error('Converged result with a common scale required');
    const receipts = [...m.applications].filter(([id]) => !p.applicationIds.includes(id));
    if (receipts.length !== 1 || !receipts[0][1].physicalApplied || !receipts[0][1].biasApplied ||
        receipts[0][1].scale !== result.scale || receipts[0][1].rowsApplied)
        throw new Error('Rows require exactly one physical/material application at the same scale');
    for (const [name, count] of [['biasContactIncrement', p.contactCount], ['biasAdditionalIncrement', p.additionalCount]]) {
        if (!result[name] || result[name].length !== count) throw new RangeError('Missing or stale ' + name);
        for (const v of result[name]) finite(v, name);
    }
    // Rows without a bias channel must carry exactly zero bias response.
    p.descriptors.forEach((d, i) => { if (d?.bias === null && result.biasAdditionalIncrement[i] !== 0) throw new Error('Bias force on physical-only row'); });
    const updates = p.targets.map(t => {
        if (valueOf(t.entry) !== t.beta) throw new Error('Bias bank changed after row preparation');
        for (const g of [...t.guards, ...(t.historyGuards ?? [])]) if (!Object.is(g.at === null ? g.object[g.key] : g.object[g.key]?.[g.at], g.value))
            throw new Error('Runtime owner changed before bias commit');
        const delta = result.scale * result[t.source === 'contact' ? 'biasContactIncrement' : 'biasAdditionalIncrement'][t.index];
        if (t.release && (result.additionalIncrement?.[t.index] !== 0 || result.biasAdditionalIncrement[t.index] !== -t.beta))
            throw new Error('Release requires the full fixed physical-zero and negative-beta response');
        const next = finite(t.beta + delta, 'Next bias multiplier');
        // Do not clamp even a tiny negative normal: that would discard part
        // of the response already applied by the common mechanical solve.
        if ((t.unilateral || t.release) && next < 0) throw new RangeError('Negative solved normal bias multiplier');
        let journal;
        if (t.kind === 'sheath') {
            journal = joint._splitMotion.sheathHistory.get(t.owner)?.reactionHistory?.bias?.[t.side];
            if (!journal || !Number.isFinite(journal[t.node])) throw new Error('Missing sheath bias reaction history');
        }
        const journalScale = t.release && t.entry.mode === 'difference'
            ? Math.hypot(...t.entry.frozenWorldGradients.map(g => g.value)) : 1;
        return { t, delta, next, journal, journalScale };
    });
    for (const { t, delta, next, journal, journalScale } of updates) {
        t.entry.bank[t.entry.slot] = next;
        if (!t.release) {
            t.entry.identity = t.identity.slice(); t.entry.frozenGradients = t.gradients.map(g => ({ ...g }));
            t.entry.frozenWorldGradients = t.worldGradients.map(g => ({ ...g })); t.entry.segment = t.segment;
            t.entry.historyGuards = t.historyGuards.map(g => ({ ...g })); t.entry.alpha = t.alpha;
            t.entry.semanticIdentity = t.unilateral ? t.semanticIdentity.slice() : null;
        }
        // A difference uses a unit carrier multiplying an owned WORLD wrench.
        // Its carrier is not a normal multiplier and must not enter this
        // force-history journal as a spurious unit reaction.
        if (journal) journal[t.node] = Math.max(journal[t.node], ...[t.beta, delta, next].map(v => Math.abs(v) * journalScale));
    }
    refreshKirchhoffBiasReactionAnchors(joint);
    receipts[0][1].rowsApplied = true; p.committed = true;
    return state.bank;
}

/** Caller freshly recollects geometry and stages _splitActual* first. This
 * measurement may re-express owned force history into canonical/retired banks
 * when the witness changes; the total world wrench, pose and velocity remain
 * unchanged. Fresh orientation rows may be supplied as a batch or an array.
 * Old linearized RHS is never presented as accepted-pose geometry. */
export function measureKirchhoffTwoChannelRows(joint, freshBoundaryRows = [], orientationBatch = null) {
    const orientationRows = Array.isArray(orientationBatch) ? orientationBatch : orientationBatch?.rows ?? [];
    if (orientationBatch === null) for (const [side, body] of bodiesOf(joint).entries()) {
        const segment = body.orientationControlSegment;
        if (!(body.orientationControlCompliance > 0 && segment >= Math.max(0, body.activeStart) && segment < Math.min(body.segmentCount, body.activeEnd))) continue;
        if (freshBoundaryRows.some(r => r.kind === 'orientation-control' && r.side === side)) continue;
        for (let component = 0; component < 3; component++) orientationRows.push({ kind: 'orientation-control', side, segment, component,
            alpha: body.orientationControlCompliance / joint._splitMotion.dt ** 2,
            gradients: XYZ.map((_, axis) => ({ side, dof: segment * 6 + 3 + axis, value: 0 })) });
    }
    const rows = [...freshBoundaryRows, ...orientationRows];
    const { targets, issues } = scan(joint, rows, [], false, true), release = measureKirchhoffBiasReleases(joint);
    issues.push(...release.issues);
    let normalResidualMm = 0, controlResidualMm = 0, orientationResidualRad = 0;
    const controls = new Map(), orientations = new Map();
    for (const t of targets) {
        const r = t.strain + t.alpha * t.beta;
        if (t.unilateral) normalResidualMm = Math.max(normalResidualMm, t.beta > 0 ? Math.abs(r) : Math.max(0, -r));
        else {
            const map = t.kind === 'control' ? controls : orientations, key = t.side + ':' + (t.kind === 'control' ? t.node : t.segment);
            if (!map.has(key)) map.set(key, new Float64Array(3));
            map.get(key)[t.component] = r;
        }
    }
    for (const r of controls.values()) controlResidualMm = Math.max(controlResidualMm, Math.hypot(...r));
    for (const r of orientations.values()) orientationResidualRad = Math.max(orientationResidualRad, Math.hypot(...r));
    const missingLoadedRows = issues.filter(i => i.reason === 'missing-loaded-bias-row');
    return { finite: release.finite && [normalResidualMm, controlResidualMm, orientationResidualRad].every(Number.isFinite), supported: issues.length === 0,
        normalResidualMm, controlResidualMm, orientationResidualRad, missingLoadedRows, issues,
        releasePositionMm: release.releasePositionMm, releaseAngleRad: release.releaseAngleRad,
        releaseCorrection: release.releaseCorrection, pendingReleases: release.pendingReleases };
}
