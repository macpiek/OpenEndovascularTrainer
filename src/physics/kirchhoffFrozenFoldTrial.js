// The apply/measure phase never rebuilds fold equations. apply changes lambda
// and pending; measure overwrites frames/geometry/measurement from the current
// pose. Branch axes, limits, gradients and row indices belong to the frozen
// build. Keep force/hint state, not another copy of that entire operator graph.
export function captureFrozenFoldTrial(constraint, barriers, out = {}) {
    const state = constraint._coupledFoldRows;
    out.state = state ?? null;
    if (!state) { out.bytes = 0; return out; }
    out.version = state.buildVersion;
    out.step = state.step;
    out.pending = state.pending;
    out.rows = state.rows;
    out.count = state.rows.length;
    if (!out.lambda || out.lambda.length < out.count) {
        out.lambda = new Float64Array(out.count);
        out.hints = new Array(out.count);
        out.rowOwners = new Array(out.count);
    }
    for (let i = 0; i < out.count; i++) {
        const row = state.rows[i];
        out.rowOwners[i] = row;
        out.lambda[i] = row.lambda;
        out.hints[i] = row.activeHint;
    }
    out.rowOwners.length = out.hints.length = out.count;
    out.storage = state.storage;
    const storage = out.copies ??= [];
    storage.length = state.storage.length;
    out.bytes = out.count * 8;
    for (let i = 0; i < state.storage.length; i++) {
        const owner = state.storage[i], lambda = owner.lambda;
        const item = storage[i] ??= {};
        item.owner = owner; item.lambda = lambda;
        if (!item.view || item.view.buffer !== lambda.buffer || item.view.byteOffset !== lambda.byteOffset ||
            item.view.byteLength !== lambda.byteLength) {
            item.view = new Uint8Array(lambda.buffer, lambda.byteOffset, lambda.byteLength);
            item.copy = new Uint8Array(lambda.byteLength);
        }
        item.copy.set(item.view);
        out.bytes += item.copy.byteLength;
        // Pools/rows can also be reached via the combined boundary collector.
        // Register those aliases, not just the fold-state root.
        barriers.add(owner); barriers.add(owner.pool);
        for (const row of owner.pool) if (row) {
            barriers.add(row); barriers.add(row.gradients);
            for (const gradient of row.gradients) barriers.add(gradient);
        }
    }
    barriers.add(state); barriers.add(state.rows); barriers.add(state.storage);
    return out;
}

// Validate before any rollback writes, just like the frozen friction batches.
export function validateFrozenFoldTrial(constraint, snapshot) {
    if (!snapshot?.state) return;
    const state = snapshot.state;
    if (constraint._coupledFoldRows !== state || state.buildVersion !== snapshot.version ||
        state.step !== snapshot.step || state.rows !== snapshot.rows || state.rows.length !== snapshot.count ||
        state.storage !== snapshot.storage || state.storage.length !== snapshot.copies.length)
        throw new Error('A frozen fold build changed during a trial');
    for (let i = 0; i < snapshot.count; i++) if (state.rows[i] !== snapshot.rowOwners[i])
        throw new Error('A frozen fold row changed during a trial');
    for (let i = 0; i < snapshot.copies.length; i++) {
        const item = snapshot.copies[i];
        if (state.storage[i] !== item.owner || item.owner.lambda !== item.lambda)
            throw new Error('Frozen fold multiplier storage changed during a trial');
    }
}

export function restoreFrozenFoldTrial(snapshot) {
    if (!snapshot?.state) return;
    for (const item of snapshot.copies) item.view.set(item.copy);
    for (let i = 0; i < snapshot.count; i++) {
        snapshot.rows[i].lambda = snapshot.lambda[i];
        snapshot.rows[i].activeHint = snapshot.hints[i];
    }
    snapshot.state.pending = snapshot.pending;
}
