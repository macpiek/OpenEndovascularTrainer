import {DoubleSide, Ray} from 'three';

// Static nearest-axis search for mapping STL triangles to flow samples. A
// small BVH avoids scanning a whole renal territory for every fine triangle.
// It only assigns concentration owners; it never alters the anatomical mesh.
export function createLumenSurfaceMapping(network, {wallTree = null, territories = null, radiusWeighted = false} = {}) {
    const edges = network.edges.filter(e => !e.transportExcluded && !e.lumenMappingExcluded);
    const ray = new Ray();
    const maxRadius = edges.reduce((r,e)=>Math.max(r,e.radiusStart,e.radiusEnd),0);
    function build(items) {
        const min = [Infinity, Infinity, Infinity], max = [-Infinity, -Infinity, -Infinity];
        let reach = 0, radius = 0;
        for (const e of items) radius = Math.max(radius, e.radiusStart, e.radiusEnd);
        reach = 2 * radius + 1;
        for (const e of items) for (let k = 0; k < 3; k++) {
            const a = e.start.getComponent(k), b = e.end.getComponent(k);
            min[k] = Math.min(min[k], a, b); max[k] = Math.max(max[k], a, b);
        }
        const node = {min, max, reach, radius, items: null, left: null, right: null};
        if (items.length <= 8) node.items = items;
        else {
            let axis = 0;
            for (let k = 1; k < 3; k++) if (max[k] - min[k] > max[axis] - min[axis]) axis = k;
            items.sort((a, b) => a.start.getComponent(axis) + a.end.getComponent(axis) -
                b.start.getComponent(axis) - b.end.getComponent(axis));
            const middle = items.length >> 1;
            node.left = build(items.slice(0, middle)); node.right = build(items.slice(middle));
        }
        return node;
    }
    const root = build(edges), stack = [];
    function boxDistance(node, x, y, z) {
        const dx = Math.max(node.min[0] - x, 0, x - node.max[0]);
        const dy = Math.max(node.min[1] - y, 0, y - node.max[1]);
        const dz = Math.max(node.min[2] - z, 0, z - node.max[2]);
        return dx * dx + dy * dy + dz * dz;
    }
    return (x, y, z, out, normal = null, maxDistance = Infinity, territory = -1) => {
        // A side-branch axis starts inside its parent lumen. Pure distance
        // lets this narrow, still-empty prefix steal the broad aortic wall
        // and cut a hole in a bolus. Select the physical lumen by normalized
        // radial distance. Once that territory is known, use ordinary axial
        // projection so calibre changes cannot move its concentration front.
        const radiusAware = radiusWeighted && territory < 0;
        const limit = maxDistance * maxDistance;
        let best = limit, bestScore = Infinity, selected = null, bestT = 0;
        const scoreBound = node => boxDistance(node, x, y, z) /
            (radiusAware ? Math.max(1e-8, node.radius * node.radius) : 1);
        stack.length = 0; stack.push(root);
        while (stack.length) {
            const node = stack.pop();
            if (boxDistance(node, x, y, z) > limit || scoreBound(node) > bestScore) continue;
            if (normal && boxDistance(node,x,y,z) > node.reach * node.reach) continue;
            if (node.items) {
                for (const e of node.items) {
                    if (territory >= 0 && territories?.[e.index] !== territory) continue;
                    const t = Math.max(0, Math.min(1, ((x - e.start.x) * e.axis.x +
                        (y - e.start.y) * e.axis.y + (z - e.start.z) * e.axis.z) / e.length));
                    const dx = x - e.start.x - e.axis.x * e.length * t;
                    const dy = y - e.start.y - e.axis.y * e.length * t;
                    const dz = z - e.start.z - e.axis.z * e.length * t;
                    const d = dx * dx + dy * dy + dz * dz;
                    const radius = e.radiusStart + (e.radiusEnd - e.radiusStart) * t;
                    const score = d / (radiusAware ? Math.max(1e-8, radius * radius) : 1);
                    if (d <= limit && (score < bestScore || (score === bestScore && e.index < selected?.index))) {
                        // An axis represents a local cross-section, not every
                        // unobstructed point along the vessel's entire length.
                        const reach = 2 * Math.max(e.radiusStart,e.radiusEnd) + 1;
                        if (normal && d > reach * reach) continue;
                        if (normal && normal.x * -dx + normal.y * -dy + normal.z * -dz <= 1e-5) continue;
                        if (wallTree && normal) {
                            const distance = Math.sqrt(d);
                            ray.origin.set(x,y,z); ray.direction.set(-dx,-dy,-dz).divideScalar(distance);
                            // Only skip the originating face's numerical thickness.
                            // The complete open segment must stay in this lumen;
                            // a closer axis across a wall cannot lend its iodine.
                            if (wallTree.raycastFirst(ray,DoubleSide,1e-4,distance-1e-4)) continue;
                        }
                        best = d; bestScore = score; selected = e; bestT = t;
                    }
                }
            } else {
                const left = scoreBound(node.left), right = scoreBound(node.right);
                if (left < right) {if (right <= bestScore) stack.push(node.right); if (left <= bestScore) stack.push(node.left);}
                else {if (left <= bestScore) stack.push(node.left); if (right <= bestScore) stack.push(node.right);}
            }
        }
        out.edgeIndex = selected?.index ?? -1;
        // Bounds a second, visibility-constrained search when the nearest axis
        // lies across a wall. An unrepresented lumen remains unassigned rather
        // than borrowing contrast from a distant unrelated artery.
        out.searchRadius = Math.sqrt(best) + 2 * maxRadius;
        if (selected) {
            out.centerX = selected.start.x + selected.axis.x * selected.length * bestT;
            out.centerY = selected.start.y + selected.axis.y * selected.length * bestT;
            out.centerZ = selected.start.z + selected.axis.z * selected.length * bestT;
            out.radius = selected.radiusStart + (selected.radiusEnd - selected.radiusStart) * bestT;
            out.distance = Math.sqrt(best); out.t = bestT;
        }
        return out;
    };
}
