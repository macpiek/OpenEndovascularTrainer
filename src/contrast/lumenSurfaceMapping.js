import {Ray} from 'three';
import {createLumenOcclusionTest} from './lumenOcclusion.js';

// Static nearest-axis search for mapping STL triangles to flow samples. A
// small BVH avoids scanning a whole renal territory for every fine triangle.
// It only assigns concentration owners; it never alters the anatomical mesh.
export function createLumenSurfaceMapping(network, {wallTree = null, territories = null, radiusWeighted = false} = {}) {
    const edges = network.edges.filter(e => !e.transportExcluded && !e.lumenMappingExcluded);
    const ray = new Ray();
    const occluded = createLumenOcclusionTest(wallTree);
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
    // A wall sample already assigned to a continuous lumen must search only
    // that territory. Keep identical distance/visibility tests and tie breaks,
    // but avoid visiting thousands of unrelated fine renal branches.
    const territoryItems = new Map(), territoryRoots = new Map();
    if (territories) for (const edge of edges) {
        const territory = territories[edge.index];
        if (territory < 0) continue;
        let items = territoryItems.get(territory);
        if (!items) territoryItems.set(territory, items = []);
        items.push(edge);
    }
    function rootForTerritory(territory) {
        if (!(territory >= 0) || !territories) return root;
        if (!territoryRoots.has(territory)) {
            const items = territoryItems.get(territory);
            territoryRoots.set(territory, items?.length ? build(items) : null);
            territoryItems.delete(territory);
        }
        return territoryRoots.get(territory);
    }
    function boxDistance(node, x, y, z) {
        const dx = Math.max(node.min[0] - x, 0, x - node.max[0]);
        const dy = Math.max(node.min[1] - y, 0, y - node.max[1]);
        const dz = Math.max(node.min[2] - z, 0, z - node.max[2]);
        return dx * dx + dy * dy + dz * dz;
    }
    let unrestrictedSeed = null;
    return (x, y, z, out, normal = null, maxDistance = Infinity, territory = -1) => {
        // A side-branch axis starts inside its parent lumen. Pure distance
        // lets this narrow, still-empty prefix steal the broad aortic wall
        // and cut a hole in a bolus. Select the physical lumen by normalized
        // radial distance. Once that territory is known, use ordinary axial
        // projection so calibre changes cannot move its concentration front.
        const radiusAware = radiusWeighted && territory < 0;
        const limit = maxDistance * maxDistance;
        let best = limit, bestScore = Infinity, selected = null, bestT = 0;
        // Neighbouring STL triangles usually share their closest axis. Use
        // the previous exact winner only as an initial upper bound, then run
        // the complete search with the same distance and index tie breaks.
        // Visibility-constrained searches retain their original evaluation.
        if (!normal && territory < 0 && unrestrictedSeed) {
            const e = unrestrictedSeed;
            const t = Math.max(0, Math.min(1, ((x - e.start.x) * e.axis.x +
                (y - e.start.y) * e.axis.y + (z - e.start.z) * e.axis.z) / e.length));
            const dx = x - e.start.x - e.axis.x * e.length * t;
            const dy = y - e.start.y - e.axis.y * e.length * t;
            const dz = z - e.start.z - e.axis.z * e.length * t;
            const d = dx * dx + dy * dy + dz * dz;
            const radius = e.radiusStart + (e.radiusEnd - e.radiusStart) * t;
            if (d <= limit) {
                best = d; bestScore = d / (radiusAware ? Math.max(1e-8, radius * radius) : 1);
                selected = e; bestT = t;
            }
        }
        // The projected point of every axis is inside its node's bounds.
        // A whole node behind the wall plane cannot pass the existing inward
        // normal test. Keep a scale-aware roundoff margin at that plane.
        const normalRoundoff = normal ? 32 * Number.EPSILON * (1 +
            Math.abs(normal.x) * (Math.abs(x) + Math.max(Math.abs(root.min[0]), Math.abs(root.max[0]))) +
            Math.abs(normal.y) * (Math.abs(y) + Math.max(Math.abs(root.min[1]), Math.abs(root.max[1]))) +
            Math.abs(normal.z) * (Math.abs(z) + Math.max(Math.abs(root.min[2]), Math.abs(root.max[2])))) : 0;
        const scoreBound = node => boxDistance(node, x, y, z) /
            (radiusAware ? Math.max(1e-8, node.radius * node.radius) : 1);
        stack.length = 0;
        const searchRoot = rootForTerritory(territory);
        if (searchRoot) stack.push(searchRoot);
        while (stack.length) {
            const node = stack.pop();
            const boundDistance = boxDistance(node, x, y, z);
            if (boundDistance > limit || boundDistance /
                (radiusAware ? Math.max(1e-8, node.radius * node.radius) : 1) > bestScore) continue;
            if (normal) {
                if (boundDistance > node.reach * node.reach) continue;
                const facingBound = normal.x * ((normal.x >= 0 ? node.max[0] : node.min[0]) - x) +
                    normal.y * ((normal.y >= 0 ? node.max[1] : node.min[1]) - y) +
                    normal.z * ((normal.z >= 0 ? node.max[2] : node.min[2]) - z);
                if (facingBound <= 1e-5 - normalRoundoff) continue;
            }
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
                            if (occluded(ray,1e-4,distance-1e-4)) continue;
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
        if (!normal && territory < 0) unrestrictedSeed = selected;
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
