// Conservative bounds of the exact atlas segments used by spatial mixing.
// Non-own paths are already restricted to their tube interiors by the mixing
// kernel, so a path which misses the whole detector chord cannot contribute.
export function createFlowColumnRayCandidates(spatial, {leafSize = 8} = {}) {
    const items = [];
    for (const [path, range] of spatial.ranges) for (let i = 0; i < range.count - 1; i++) {
        const at = (range.start + i) * 4, a = spatial.data, min = [], max = [];
        for (let axis = 0; axis < 3; axis++) {
            min[axis] = Math.min(a[at + axis] - a[at + 3], a[at + 4 + axis] - a[at + 7]);
            max[axis] = Math.max(a[at + axis] + a[at + 3], a[at + 4 + axis] + a[at + 7]);
        }
        items.push({path, min, max});
    }
    const nodes = [], leafPaths = [];
    function build(first, last) {
        const index = nodes.length, min = [Infinity, Infinity, Infinity], max = [-Infinity, -Infinity, -Infinity];
        for (let i = first; i < last; i++) for (let axis = 0; axis < 3; axis++) {
            min[axis] = Math.min(min[axis], items[i].min[axis]); max[axis] = Math.max(max[axis], items[i].max[axis]);
        }
        const node = {min, max, escape: -1, first: -1, count: 0}; nodes.push(node);
        if (last - first <= leafSize) {
            node.first = leafPaths.length;
            const unique = new Set(); for (let i = first; i < last; i++) unique.add(items[i].path);
            leafPaths.push(...unique); node.count = unique.size;
        } else {
            let axis = 0; for (let k = 1; k < 3; k++) if (max[k] - min[k] > max[axis] - min[axis]) axis = k;
            const ordered = items.slice(first, last).sort((a, b) => a.min[axis] + a.max[axis] - b.min[axis] - b.max[axis]);
            for (let i = 0; i < ordered.length; i++) items[first + i] = ordered[i];
            const middle = (first + last) >>> 1; build(first, middle); build(middle, last);
        }
        node.escape = nodes.length;
    }
    if (items.length) build(0, items.length);
    const data = new Float32Array(nodes.length * 8 + Math.ceil(leafPaths.length / 4) * 4);
    for (let i = 0; i < nodes.length; i++) {
        const node = nodes[i], at = i * 8;
        for (let axis = 0; axis < 3; axis++) {
            // Outward allowance exceeds Float32 rounding; thin lumens may
            // acquire an extra candidate but can never lose their own path.
            const margin = Math.max(1e-5, Math.abs(node.min[axis]) * 1e-6, Math.abs(node.max[axis]) * 1e-6);
            data[at + axis] = node.min[axis] - margin; data[at + 4 + axis] = node.max[axis] + margin;
            node.min[axis] = data[at + axis]; node.max[axis] = data[at + 4 + axis];
        }
        data[at + 3] = node.escape;
        data[at + 7] = node.first < 0 ? -1 : node.first * (leafSize + 1) + node.count;
    }
    if (leafPaths.length * (leafSize + 1) >= 2 ** 24)
        throw new RangeError('Ray candidate metadata exceeds exact Float32 indexing');
    data.set(leafPaths, nodes.length * 8);
    function query(entry, exit, {limit = Infinity, padding = 0, accept = null} = {}) {
        const paths = new Set(); let visited = 0;
        for (let index = 0; index < nodes.length;) {
            const node = nodes[index]; let lo = 0, hi = 1;
            for (let axis = 0; axis < 3; axis++) {
                const start = entry.getComponent(axis), direction = exit.getComponent(axis) - start;
                const min = node.min[axis] - padding, max = node.max[axis] + padding;
                if (Math.abs(direction) < 1e-12) {if (start < min || start > max) {lo = 1; hi = 0; break;}}
                else {const a = (min - start) / direction, b = (max - start) / direction; lo = Math.max(lo, Math.min(a, b)); hi = Math.min(hi, Math.max(a, b));}
            }
            visited++;
            if (lo > hi) {index = node.escape; continue;}
            if (node.first >= 0) for (let i = 0; i < node.count; i++) {
                const path = leafPaths[node.first + i]; if (accept && !accept(path)) continue;
                paths.add(path); if (paths.size > limit) return {paths, overflow: true, visited};
            }
            index++;
        }
        return {paths, overflow: false, visited};
    }
    return {nodes, data, leafSize, segmentCount: items.length, nodeCount: nodes.length,
        pathScalarOffset: nodes.length * 8, memoryBytes: data.byteLength, query};
}
