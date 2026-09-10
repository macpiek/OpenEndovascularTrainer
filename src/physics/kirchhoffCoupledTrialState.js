import { kirchhoffComponentBodies } from './kirchhoffComponentBodies.js';
// Factorization workspaces are reassembled from restored mechanics. They contain
// large borrowed matrices, not retained physical multipliers/contact identity.
const linearWorkspaces = new Set(['_bundleRuntime', '_coupledSystemAssembly', '_coupledSystemQP', '_coupledSystemFriction', '_coupledSystemLoad', '_jointConeRepair', '_jointTrialState']);
const worldGeometryFields = new Set(['contactCount', 'maxPenetration', 'settledMaxPenetration', 'settledContactBodyId', 'settledContactSegment']);
const numericBodyState = value => ArrayBuffer.isView(value) ||
    value === null || ['number', 'boolean', 'string', 'bigint', 'undefined'].includes(typeof value);
const bodyFilter = (_key, value) => numericBodyState(value);
const constraintFilter = key => !linearWorkspaces.has(key);
const worldFilter = key => worldGeometryFields.has(key);
const isObject = value => value !== null && typeof value === 'object';

function sameKeys(a, b) {
    if (a.length !== b.length) return false;
    for (let i = 0; i < a.length; i++) if (a[i] !== b[i]) return false;
    return true;
}

/** Capture a joint nonlinear TRIAL, after assembly and before applying it.
 * Preserves existing object/array/contact identities and exact typed-array
 * bytes. Owned roots include contact records/pools/manifold/maps/curve caches,
 * boundary/fold/friction state, and optional tool-contact/sheath state.
 * Bodies are barriers to graph traversal: only their direct numeric arrays
 * and scalar state are saved, never renderer/BVH/contactField/rod scratch.
 * Solver factorization/QP buffers are excluded and retained on restore; the
 * next solve reassembles their numeric contents. Bundle pairing is invalidated.
 * This does not snapshot the whole integrator step or permit topology edits.
 * World timings and work counters intentionally retain rejected-trial work.
 * reusePropertyLayout is for owned runtime state whose property attributes
 * cannot change during a solve (values, keys and array lengths may change).
 * frozenFrictionBatches excludes immutable solve rows and disposable friction
 * scratch. Callers must NOT rebuild these batches during a trial. Geometry
 * measurement must use separate outputs; commit may be retried after restore.
 */
export function captureKirchhoffCoupledTrialState(constraint, {
    world = null, toolContacts = world?.toolContacts ?? [], sheaths = world?.sheaths ?? [], external = [],
    reusePropertyLayout = false, frozenFrictionBatches = false
} = {}, out = {}) {
    const bodies = kirchhoffComponentBodies(constraint);
    const barriers = new Set([out, ...bodies, ...(world?.bodies ?? []), world, world?.contactField,
        ...bodies.map(body => body.contactField), ...external].filter(Boolean));
    // Never copy a previous rollback snapshot into a new whole-candidate
    // snapshot. Its records recursively own copies of earlier runtime graphs.
    if (constraint._jointTrialState) barriers.add(constraint._jointTrialState);
    // Measurement banks are rebuilt from restored mechanics. They own no
    // contact history: that remains reachable through _wallWitnessRows.
    if (constraint._wallWitnessFrictionMeasure) barriers.add(constraint._wallWitnessFrictionMeasure);
    for (const batch of [constraint._wallWitnessFrictionSolve, constraint._wallWitnessFrictionMeasure])
        for (const key of ['measurementBatch', 'measurement'])
            if (batch?.[key]) barriers.add(batch[key]);
    const batches = out.frozenBatches ??= [];
    batches.length = 0;
    if (frozenFrictionBatches) {
        for (const key of ['_jointOptions', '_jointFrictionResidual', '_jointExternalFrictionResidual',
            '_jointSplitWallFrictionResidual', '_jointStateMeasurement', '_jointMaterialResidual',
            '_jointFrictionMerit', '_jointExternalFrictionMerit', '_jointSplitWallFrictionMerit'])
            if (constraint[key]) barriers.add(constraint[key]);
        for (const key of ['_jointFrictionBatch', '_jointExternalFrictionBatch', '_jointSplitWallFrictionBatch', '_wallWitnessFrictionSolve']) {
            const batch = constraint[key];
            if (!batch) continue;
            barriers.add(batch);
            for (const row of batch.rows) barriers.add(row);
            batches.push({ batch, version: batch.version, committed: batch.committed, commitScale: batch.commitScale });
        }
    }
    const records = out.records ?? [];
    const cache = out._recordCache ??= new WeakMap();
    const epoch = out._captureEpoch = (out._captureEpoch ?? 0) + 1;
    const roots = [...bodies, constraint, ...toolContacts];
    const filters = [...bodies.map(() => bodyFilter), constraintFilter];
    for (let i = bodies.length + 1; i < roots.length; i++) filters.push(null);
    for (const sheath of sheaths) if (sheath.lambdas) { roots.push(sheath.lambdas); filters.push(null); }
    if (world) { roots.push(world); filters.push(worldFilter); }
    let sameGraph = reusePropertyLayout && out._reusePropertyLayout &&
        sameKeys(roots, out._roots) && sameKeys(filters, out._filters) &&
        barriers.size === out._barriers.size;
    if (sameGraph) for (const object of barriers) if (!out._barriers.has(object)) { sameGraph = false; break; }
    let bytes = 0;
    let graphChanged = !sameGraph;
    function refresh(record) {
        const { object, filter } = record;
        if (record.captureEpoch === epoch) return;
        record.captureEpoch = epoch;
        if (ArrayBuffer.isView(object)) {
            if (!record.view || record.view.buffer !== object.buffer || record.view.byteOffset !== object.byteOffset ||
                record.view.byteLength !== object.byteLength) {
                record.view = new Uint8Array(object.buffer, object.byteOffset, object.byteLength);
                record.copy = new Uint8Array(object.byteLength);
            }
            record.copy.set(record.view);
        } else if (object instanceof Map) {
            let index = 0;
            for (const [key, value] of object) {
                const pair = record.entries[index++] ??= [];
                if (pair[1] !== value && (isObject(pair[1]) || isObject(value))) graphChanged = true;
                pair[0] = key; pair[1] = value;
            }
            if (record.entries.length !== index) graphChanged = true;
            record.entries.length = index;
        } else if (object instanceof Set) {
            let index = 0;
            for (const value of object) {
                const previous = record.entries[index];
                if (previous !== value && (isObject(previous) || isObject(value))) graphChanged = true;
                record.entries[index++] = value;
            }
            if (record.entries.length !== index) graphChanged = true;
            record.entries.length = index;
        } else {
            record.reusePropertyLayout = reusePropertyLayout;
            // A sealed object cannot add/remove keys. The runtime layout
            // contract also fixes descriptor attributes; values/references
            // are still read on every capture. Arrays keep their length path.
            const keys = reusePropertyLayout && record.fixedKeys ? record.keys : Object.getOwnPropertyNames(object);
            const sameLayout = reusePropertyLayout && !filter && sameKeys(keys, record.keys);
            if (sameLayout) {
                // Runtime contact pools usually keep their shape. The owned
                // layout contract preserves property attributes, so only the
                // data values need refreshing. Accessors are never invoked.
                // In particular, avoid constructing a Set and enumerating the
                // descriptor table again for every small gradient object.
                for (let i = 0; i < record.dataKeys.length; i++) {
                    const saved = record.dataDescriptors[i], value = object[record.dataKeys[i]];
                    if (saved.value !== value && (isObject(saved.value) || isObject(value))) graphChanged = true;
                    saved.value = value;
                }
            } else {
                // A changed key/filter layout can change reachability even if
                // it has the same number of properties. Recompile the graph.
                const previousKeys = record.keys.slice();
                let count = 0;
                // Preserve descriptor semantics, including accessors. Reuse the
                // storage and byte buffers; only property metadata is refreshed.
                for (const key of keys) {
                    const saved = record.descriptors[key];
                    if (reusePropertyLayout && saved && 'value' in saved) {
                        const value = object[key];
                        if (filter && !filter(key, value)) continue;
                        if (saved.value !== value && (isObject(saved.value) || isObject(value))) graphChanged = true;
                        saved.value = value;
                    } else {
                        const descriptor = Object.getOwnPropertyDescriptor(object, key);
                        if (filter && !filter(key, descriptor.value)) continue;
                        if (saved && 'value' in saved && 'value' in descriptor) {
                            if (saved.value !== descriptor.value && (isObject(saved.value) || isObject(descriptor.value))) graphChanged = true;
                            saved.value = descriptor.value; saved.writable = descriptor.writable;
                            saved.enumerable = descriptor.enumerable; saved.configurable = descriptor.configurable;
                        } else record.descriptors[key] = descriptor;
                    }
                    record.keys[count++] = key;
                }
                record.keys.length = count;
                if (!sameKeys(previousKeys, record.keys)) graphChanged = true;
                const present = new Set(record.keys);
                for (const key of Object.keys(record.descriptors)) if (!present.has(key)) delete record.descriptors[key];
                record.dataKeys.length = record.dataDescriptors.length = 0;
                for (const key of record.keys) {
                    const descriptor = record.descriptors[key];
                    if ('value' in descriptor) {
                        record.dataKeys.push(key); record.dataDescriptors.push(descriptor);
                    }
                }
                record.fixedKeys = !filter && !Array.isArray(object) && Object.isSealed(object);
            }
        }
    }
    function visit(object, filter = null, force = false) {
        if (!isObject(object) || (!force && barriers.has(object))) return;
        let record = cache.get(object);
        if (!record) {
            record = { object, filter, captureEpoch: 0, visitEpoch: 0 };
            if (ArrayBuffer.isView(object)) record.kind = 'bytes';
            else if (object instanceof Map || object instanceof Set) {
                record.kind = object instanceof Map ? 'map' : 'set'; record.entries = [];
            } else {
                record.kind = 'object'; record.descriptors = Object.create(null);
                record.keys = []; record.dataKeys = []; record.dataDescriptors = [];
            }
            cache.set(object, record);
        }
        if (record.visitEpoch === epoch) return;
        record.visitEpoch = epoch;
        if (record.filter !== filter) { record.filter = filter; record.captureEpoch = 0; }
        refresh(record);
        records.push(record);
        if (record.kind === 'bytes') bytes += record.copy.byteLength;
        if (record.kind === 'map') for (const [, value] of record.entries) visit(value);
        else if (record.kind === 'set') for (const value of record.entries) visit(value);
        else if (record.kind === 'object') for (const d of record.dataDescriptors) visit(d.value);
    }
    // The retained records are also a compiled traversal plan. Read every
    // value and every own string key on every capture. If references and root
    // barriers are unchanged, no per-object WeakMap/Set traversal is needed.
    // When they change, rebuild membership from the just-captured values;
    // newly reachable objects are captured and unreachable ones are removed.
    if (sameGraph) for (const record of records) refresh(record);
    if (graphChanged) {
        records.length = 0;
        for (let i = 0; i < roots.length; i++) visit(roots[i], filters[i], i < bodies.length || roots[i] === world);
    } else for (const record of records) if (record.kind === 'bytes') bytes += record.copy.byteLength;
    out._roots = roots; out._filters = filters; out._barriers = barriers;
    out._reusePropertyLayout = reusePropertyLayout;
    out.constraint = constraint; out.bodies = bodies;
    out.topology = bodies.map(body => ({ count: body.count, segmentCount: body.segmentCount, activeStart: body.activeStart, activeEnd: body.activeEnd }));
    out.records = records; out.bytes = bytes; out.objectCount = records.length;
    out.manifold = constraint.manifold;
    out.contacts = new Set(constraint.manifold?.contacts() ?? []);
    out.restoreCount = 0; out.needsReassembly = false;
    return out;
}

/** Restore a rejected candidate in place, including Map ownership/rekeying.
 * Caller's old solver result owns its corrections and may be reused at a new
 * scale. Rebuild geometry/batches/rows before a NEW solve; never reuse borrowed
 * linear assembly or cached trial residuals as a convergence certificate.
 */
export function restoreKirchhoffCoupledTrialState(snapshot) {
    const { constraint, bodies } = snapshot;
    const currentBodies = kirchhoffComponentBodies(constraint);
    if(currentBodies.length!==bodies.length)throw new Error('Cannot roll back a nonlinear trial across topology changes');
    for (let side = 0; side < bodies.length; side++) {
        const body = bodies[side], topology = snapshot.topology[side];
        if (body !== currentBodies[side] || body.count !== topology.count ||
            body.segmentCount !== topology.segmentCount || body.activeStart !== topology.activeStart || body.activeEnd !== topology.activeEnd)
            throw new Error('Cannot roll back a nonlinear trial across topology changes');
    }
    for (const { batch, version } of snapshot.frozenBatches) if (batch.version !== version)
        throw new Error('A frozen friction solve batch was rebuilt during a trial');
    // A new trial contact must not retain a valid-looking owner after removal
    // from the restored Map. Previously retained/rekeyed objects are restored.
    for (const contact of snapshot.manifold?.contacts() ?? []) if (!snapshot.contacts.has(contact)) contact._manifold = null;
    for (const record of snapshot.records) {
        const { object } = record;
        if (record.kind === 'bytes') record.view.set(record.copy);
        else if (record.kind === 'map') { object.clear(); for (const [key, value] of record.entries) object.set(key, value); }
        else if (record.kind === 'set') { object.clear(); for (const value of record.entries) object.add(value); }
        else {
            if (!record.fixedKeys) for (const key of Object.getOwnPropertyNames(object)) {
                if ((!record.filter || record.filter(key, object[key])) && !(key in record.descriptors)) delete object[key];
            }
            if (record.reusePropertyLayout) {
                for (const key of record.keys) {
                    const descriptor = record.descriptors[key];
                    if ('value' in descriptor && descriptor.writable) object[key] = descriptor.value;
                    else Object.defineProperty(object, key, descriptor);
                }
            } else Object.defineProperties(object, record.descriptors);
        }
    }
    for (const { batch, committed, commitScale } of snapshot.frozenBatches) {
        batch.committed = committed; batch.commitScale = commitScale;
    }
    // Keep allocated matrices/WASM memory. Coupled assembly rewrites J, Gram,
    // RHS and bounds; the QP starts from zero clamped to new bounds, and resets
    // its local factor-valid flag. Working-set hints are not retained forces.
    // Pairing alone has a revision cache and must be explicitly invalidated.
    if (constraint._bundleRuntime) constraint._bundleRuntime._paired = false;
    delete constraint._jointConeRepair;
    for (const body of bodies) if (body.kirchhoffScratch?.direct) body.kirchhoffScratch.direct.factorAge = Infinity;
    snapshot.restoreCount++; snapshot.needsReassembly = true;
    return snapshot;
}
