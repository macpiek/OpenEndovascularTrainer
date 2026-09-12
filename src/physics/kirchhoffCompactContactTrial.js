// Position-history trials rebuild normal Jacobians before measuring a candidate.
// Their scratch owns no forces/history. Body/contact identity and every retained
// multiplier still belong to the ordinary rollback graph. Small contact vectors
// are copied byte-exactly into one reusable bank instead of thousands of records.
const vectorKeys = [
    'normal', 'innerWeights', 'outerWeights',
    '_smoothInnerNodeIndices', '_smoothInnerNodeWeights', '_smoothOuterNodeIndices', '_smoothOuterNodeWeights',
    'innerSurfaceMomentIncrement', 'outerSurfaceMomentIncrement', 'surfaceTangentialIncrement',
    'innerSurfaceMomentImpulse', 'outerSurfaceMomentImpulse', 'surfaceTangentialImpulse', 'surfaceFrictionPoint'
];

function sameKeys(a, b) { return a.length === b.length && a.every((key, i) => key === b[i]); }

export function captureCompactContactTrial(constraint, barriers, out = {}) {
    const copies = out.copies ??= [];
    const cache = out.cache ??= new WeakMap();
    const seen = out.seen ??= new Set();
    seen.clear();
    out.bytes = 0;
    const metadata = out.metadata ??= [];
    const metadataCache = out.metadataCache ??= new WeakMap();
    const roots = out.roots ??= [];
    metadata.length = roots.length = 0;
    let count = 0;
    const save = object => {
        if (!ArrayBuffer.isView(object) || seen.has(object)) return;
        seen.add(object);
        let item = cache.get(object);
        if (!item || item.view.buffer !== object.buffer || item.view.byteOffset !== object.byteOffset ||
            item.view.byteLength !== object.byteLength) {
            item = { view: new Uint8Array(object.buffer, object.byteOffset, object.byteLength) };
            cache.set(object, item);
        }
        item.offset = out.bytes;
        out.bytes += item.view.byteLength;
        copies[count++] = item;
        barriers.add(object);
    };
    const derived = out.derived ??= new Set();
    derived.clear();
    const omit = object => { if (object) { barriers.add(object); derived.add(object); } };
    // Clipped portal samples are rebuilt from endpoint positions on every
    // containment collection. Contact identities/forces live in the manifold,
    // never in this geometry pool. Gradients also alias runtime contact rows.
    omit(constraint._jointPortalSideSamples);
    for (const sampleSet of constraint._jointPortalSideSamples ?? [])
        for (const sample of sampleSet?.pool ?? []) if (sample) omit(sample.gradients);
    for (const segment of constraint._kirchhoffRuntimeRecordPool ?? []) for (const record of segment ?? []) {
        if (!record) continue;
        // A pooled runtime record has writable data properties. Retain values
        // directly, avoiding a recursive descriptor snapshot of every record.
        // Unknown child objects remain roots of the conservative graph copier.
        const keys = Object.getOwnPropertyNames(record);
        let meta = metadataCache.get(record);
        if (!meta || !sameKeys(keys, meta.keys)) {
            if (!keys.every(key => {
                const d = Object.getOwnPropertyDescriptor(record, key);
                return d && 'value' in d && d.writable && d.configurable && d.enumerable;
            })) continue; // Custom descriptor semantics use the complete path.
            meta = { object: record, keys, values: new Array(keys.length) };
            metadataCache.set(record, meta);
        }
        for (let i = 0; i < keys.length; i++) {
            const value = record[keys[i]];
            meta.values[i] = value;
            if (value && typeof value === 'object') roots.push(value);
        }
        metadata.push(meta);
        barriers.add(record);
        for (const key of vectorKeys) save(record[key]);
        // This entire workspace belongs to the partitioned projection. The
        // built-in position-history joint apply/measure never uses it.
        omit(record.contactScratch);
        const scratch = record._normalRowScratch;
        if (!scratch) continue;
        // Normal operator workspaces contain only derived geometry. Register
        // the two public aliases too; the next measure/build overwrites them.
        omit(scratch); omit(scratch.diagnostics); omit(scratch.gradients);
    }
    copies.length = count;
    if (!out.buffer || out.buffer.length < out.bytes) out.buffer = new Uint8Array(Math.max(out.bytes, (out.buffer?.length ?? 0) * 2));
    for (const item of copies) out.buffer.set(item.view, item.offset);
    return out;
}

export function restoreCompactContactTrial(snapshot) {
    if (!snapshot) return;
    for (const meta of snapshot.metadata) {
        const keys = Object.getOwnPropertyNames(meta.object);
        if (!sameKeys(keys, meta.keys)) for (const key of keys) if (!meta.keys.includes(key)) delete meta.object[key];
        for (let i = 0; i < meta.keys.length; i++) meta.object[meta.keys[i]] = meta.values[i];
    }
    for (const item of snapshot.copies) item.view.set(snapshot.buffer.subarray(item.offset, item.offset + item.view.byteLength));
}
