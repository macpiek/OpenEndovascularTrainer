/** Sparse exact local elimination for [[H,-J^T],[J,alpha]]. Coupled
 * eliminated coordinates are retained as a group, never diagonalized.
 */
export function condenseKirchhoffAxialSparse({ hRows, b, gradients, explicitRows, eliminatedIndices, axialCoordinates }) {
    const n = hRows?.length;
    const fail = message => { throw new RangeError(message); };
    if (!Array.isArray(hRows) || !b || b.length !== n || !Array.from(b).every(Number.isFinite)
        || !axialCoordinates || axialCoordinates.length !== n || !Array.from(axialCoordinates).every(Number.isFinite)
        || !Array.isArray(gradients) || !Array.isArray(explicitRows) || !Array.isArray(eliminatedIndices))
        fail('Invalid sparse axial dimensions or nonfinite inputs');
    const index = i => Number.isInteger(i) && i >= 0 && i < n;
    // Assembly is synchronous and never mutates H. Recovery owns only the
    // small supports it needs, so duplicating every source Map is unnecessary.
    const H = hRows;
    for (const row of H) {
        if (!(row instanceof Map)) fail('hRows must contain Maps');
        for (const [i, v] of row) {
            if (!index(i) || !Number.isFinite(v)) fail('Invalid sparse H entry');
        }
    }
    for (let i = 0; i < n; i++) for (const [j, v] of H[i]) {
        if ((H[j].get(i) ?? 0) !== v) fail('Sparse H must be exactly symmetric');
    }
    if (eliminatedIndices.some(i => !index(i)) || new Set(eliminatedIndices).size !== eliminatedIndices.length)
        fail('Eliminated indices must be unique and valid');
    const requested = new Set(eliminatedIndices);
    let coupled = false;
    for (const i of requested) for (const [j, v] of H[i]) if (i !== j && requested.has(j) && v !== 0) coupled = true;
    const eliminated = coupled ? [] : [...eliminatedIndices];
    const eliminatedSet = new Set(eliminated);
    for (const i of eliminated) if (!(H[i].get(i) > 0)) fail('Eliminated diagonal must be positive');
    const G = new Map();
    function gradient(gradientIndex) {
        if(G.has(gradientIndex)) return G.get(gradientIndex);
        const row=gradients[gradientIndex];
        if (!Array.isArray(row)) fail('Gradient must be a sparse pair array');
        const result = new Map();
        for (const pair of row) {
            if (!pair || pair.length !== 2 || !index(pair[0]) || !Number.isFinite(pair[1])) fail('Invalid gradient entry');
            const value = (result.get(pair[0]) ?? 0) + pair[1];
            if (!Number.isFinite(value)) fail('Gradient accumulation overflow');
            if (value === 0) result.delete(pair[0]); else result.set(pair[0], value);
        }
        G.set(gradientIndex,result);
        return result;
    }
    const records = explicitRows.map(row => {
        if (!row || !Number.isInteger(row.gradientIndex) || row.gradientIndex < 0 || row.gradientIndex >= gradients.length
            || !Number.isFinite(row.alpha) || row.alpha < 0 || !Number.isFinite(row.rhs)
            || typeof row.lower !== 'number' || typeof row.upper !== 'number'
            || Number.isNaN(row.lower) || Number.isNaN(row.upper) || row.lower > row.upper
            || row.lower === Infinity || row.upper === -Infinity) fail('Invalid explicit row');
        return { ...row, gradient: gradient(row.gradientIndex) };
    });
    const retained = Array.from({ length: n }, (_, i) => i).filter(i => !eliminatedSet.has(i));
    // Sort once by actual axial coordinates; ties retain deterministic primal
    // then explicit ordering. Empty gradients have no geometric support.
    const ordering = retained.map(i => ({ primal: i, coordinate: axialCoordinates[i] }));
    records.forEach((row, i) => {
        const support = [...row.gradient.keys()];
        let coordinate = 0;
        for (const j of support) coordinate += axialCoordinates[j] / support.length;
        if (!Number.isFinite(coordinate)) fail('Explicit axial coordinate overflow');
        ordering.push({ explicit: i, coordinate });
    });
    ordering.sort((a, c) => a.coordinate - c.coordinate || (a.explicit === undefined ? -1 : a.explicit) - (c.explicit === undefined ? -1 : c.explicit));
    const count = ordering.length, primalMap = new Int32Array(n).fill(-1), explicitMap = new Int32Array(records.length);
    ordering.forEach((row, i) => { if (row.primal !== undefined) primalMap[row.primal] = i; else explicitMap[row.explicit] = i; });
    const rows = Array.from({ length: count }, () => new Map());
    const rhs = new Float64Array(count), lower = new Float64Array(count).fill(-Infinity), upper = new Float64Array(count).fill(Infinity);
    const add = (i, j, value) => {
        if (value === 0) return;
        const next = (rows[i].get(j) ?? 0) + value;
        if (!Number.isFinite(next)) fail('Sparse matrix accumulation overflow');
        if (next === 0) rows[i].delete(j); else rows[i].set(j, next);
    };
    for (const i of retained) {
        rhs[primalMap[i]] = b[i];
        for (const [j, v] of H[i]) if (primalMap[j] >= 0) add(primalMap[i], primalMap[j], v);
    }
    // Store each eliminated column's explicit support for sparse outer updates.
    const explicitSupport = new Map(eliminated.map(i => [i, []]));
    records.forEach((row, c) => {
        const m = explicitMap[c]; rhs[m] = row.rhs; lower[m] = row.lower; upper[m] = row.upper;
        add(m, m, row.alpha);
        for (const [j, v] of row.gradient) {
            if (primalMap[j] >= 0) { add(m, primalMap[j], v); add(primalMap[j], m, -v); }
            else explicitSupport.get(j).push([m, v]);
        }
    });
    const recovery = [];
    let rankUpdateEntries = 0;
    for (const e of eliminated) {
        const diagonal = H[e].get(e), load = b[e];
        const h = [...H[e]].filter(([j,v]) => j !== e && v !== 0).map(([j, v]) => [primalMap[j], v]);
        const j = explicitSupport.get(e);
        recovery.push({ original: e, diagonal, load, h, j });
        for (const [i, v] of h) {
            rhs[i] -= (v / diagonal) * load;
            for (const [k, u] of h) { add(i, k, -(v / diagonal) * u); rankUpdateEntries++; }
            for (const [k, u] of j) { const a = (v / diagonal) * u; add(i, k, a); add(k, i, -a); rankUpdateEntries += 2; }
        }
        for (const [i, v] of j) {
            rhs[i] -= (v / diagonal) * load;
            for (const [k, u] of j) { add(i, k, (v / diagonal) * u); rankUpdateEntries++; }
        }
    }
    if (!rhs.every(Number.isFinite)) fail('Condensed rhs overflow');
    const starts = new Int32Array(count), ends = new Int32Array(count), offsets = new Int32Array(count);
    let entries = 0, nonzeros = 0, lowerBandwidth = 0, upperBandwidth = 0;
    for (let i = 0; i < count; i++) {
        let first = i, last = i;
        for (const j of rows[i].keys()) { first = Math.min(first, j); last = Math.max(last, j); nonzeros++; }
        starts[i] = first; ends[i] = last; offsets[i] = entries - first;
        entries += last - first + 1;
        if (entries > 2147483647) fail('General band storage exceeds Int32 addressing');
        lowerBandwidth = Math.max(lowerBandwidth, i - first); upperBandwidth = Math.max(upperBandwidth, last - i);
    }
    const values = new Float64Array(entries);
    rows.forEach((row, i) => { for (const [j, v] of row) values[offsets[i] + j] = v; });
    function recover(solution) {
        if (!solution || solution.length !== count || !Array.from(solution).every(Number.isFinite)) fail('Invalid mixed solution');
        const q = new Float64Array(n);
        for (const i of retained) q[i] = solution[primalMap[i]];
        for (const row of recovery) {
            let value = row.load;
            for (const [i, v] of row.h) value -= v * solution[i];
            for (const [i, v] of row.j) value += v * solution[i];
            q[row.original] = value / row.diagonal;
        }
        if (!q.every(Number.isFinite)) fail('Sparse axial recovery overflow');
        return q;
    }
    // Original retained order is ascending primal index. Its positions in the
    // interleaved mixed solution are explicitly provided, never assumed contiguous.
    return { matrix: { values, starts, ends, offsets }, rhs, lower, upper, count,
        coordinates: ordering.map(row=>row.coordinate), primalSolutionIndices: Array.from(primalMap),
        retainedIndices: [...retained], retainedSolutionIndices: retained.map(i => primalMap[i]),
        explicitIndices: Array.from(explicitMap), recover,
        diagnostics: { originalDofs: n, retainedDofs: retained.length, eliminatedOffsetDofs: eliminated.length,
            eliminatedSolver: coupled ? 'retained-coupled-block' : 'diagonal', eliminatedFactorEntries: 0,
            coupledEliminationFallback: coupled, explicitRows: records.length, nonzeros, bandEntries: entries,
            lowerBandwidth, upperBandwidth, rankUpdateEntries } };
}
