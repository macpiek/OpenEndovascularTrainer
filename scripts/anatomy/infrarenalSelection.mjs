// Select the connected infrarenal surface within the deformation slab. Spatial
// proximity alone also selects mesenteric arteries passing in front of the sac.
export function selectInfrarenalSurface(positions, config, centerAt) {
    const ids = new Map(), parent = [], occurrences = [];
    const root = id => {
        while (parent[id] !== id) { parent[id] = parent[parent[id]]; id = parent[id]; }
        return id;
    };
    const join = (a, b) => { parent[root(a)] = root(b); };
    let seed = -1, score = Infinity;
    const center = centerAt(config.referenceY);
    for (let i = 0; i < positions.length; i += 9) {
        const triangle = [];
        for (let k = 0; k < 9; k += 3) {
            const index = i + k, x = positions[index], y = positions[index + 1], z = positions[index + 2];
            if (y <= config.distalY || y >= config.proximalY) continue;
            const key = `${x},${y},${z}`;
            let id = ids.get(key);
            if (id === undefined) { id = parent.length; ids.set(key, id); parent.push(id); }
            triangle.push(id); occurrences.push([index / 3, id]);
            const distance = (y - config.referenceY) ** 2 + (x - center[0]) ** 2 + (z - center[2]) ** 2;
            if (distance < score) { score = distance; seed = id; }
        }
        for (let k = 1; k < triangle.length; k++) join(triangle[0], triangle[k]);
    }
    if (seed < 0) throw Error('Missing infrarenal surface');
    const selected = new Uint8Array(positions.length / 3), selectedRoot = root(seed);
    for (const [index, id] of occurrences) if (root(id) === selectedRoot) selected[index] = 1;
    return selected;
}

// Mirror the surface selection on the baseline centerline graph. Include edges
// crossing the slab boundaries but never traverse a node outside the slab.
export function selectInfrarenalCenterline(asset, config, centerAt) {
    const data = asset.arrays.centerlineSegments, edges = asset.arrays.centerlineEdges;
    const adjacency = new Map();
    let seed = -1, best = Infinity;
    const center = centerAt(config.referenceY);
    for (let i = 0; i < edges.length / 2; i++) {
        const ya = data[9 * i + 1], yb = data[9 * i + 4];
        if (Math.max(ya, yb) <= config.distalY || Math.min(ya, yb) >= config.proximalY) continue;
        for (let k = 0; k < 2; k++) {
            const offset = 9 * i + 3 * k, y = data[offset + 1];
            if (y <= config.distalY || y >= config.proximalY) continue;
            const node = edges[2 * i + k];
            if (!adjacency.has(node)) adjacency.set(node, []);
            adjacency.get(node).push(i);
            const distance = (data[offset] - center[0]) ** 2 + (y - config.referenceY) ** 2 + (data[offset + 2] - center[2]) ** 2;
            if (distance < best) { best = distance; seed = i; }
        }
    }
    if (seed < 0) throw Error('Missing infrarenal centerline');
    const selected = new Set([seed]), queue = [seed];
    for (let cursor = 0; cursor < queue.length; cursor++) {
        const i = queue[cursor];
        for (let k = 0; k < 2; k++) for (const next of adjacency.get(edges[2 * i + k]) || []) {
            if (!selected.has(next)) { selected.add(next); queue.push(next); }
        }
    }
    return [...selected].sort((a, b) => a - b);
}
