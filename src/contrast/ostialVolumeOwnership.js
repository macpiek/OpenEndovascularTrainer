import * as THREE from 'three';

// A small daughter starts on the parent's centreline, inside lumen already
// represented by the continuing trunk. Only this topologically shared prefix
// has an owner; unrelated vessels crossing in projection must still add.
export function buildOstialVolumeOwners(network) {
    const owners = new Map();
    const edges = network.edges;
    for (const parent of edges) {
        if (parent.childEdgeIndices?.length < 2 || !parent.childEdgeIndices) continue;
        const children = parent.childEdgeIndices.map((i) => edges[i]).filter(Boolean);
        const continuation = children.reduce(
            (best, e) => (!best || e.radiusEnd > best.radiusEnd ? e : best),
            null
        );
        if (!continuation || continuation.radiusEnd < parent.radiusEnd * 0.75) continue;
        const branches = children.filter(
            (e) =>
                e !== continuation &&
                Math.max(e.radiusStart, e.radiusEnd) < continuation.radiusEnd * 0.65
        );
        if (!branches.length) continue;
        const reach = parent.radiusEnd * 4;
        const trunk = [parent];
        let length = parent.length,
            e = edges[parent.parentEdgeIndex];
        while (e && length < reach) {
            trunk.push(e);
            length += e.length;
            e = edges[e.parentEdgeIndex];
        }
        length = 0;
        e = continuation;
        while (e && length < reach) {
            trunk.push(e);
            length += e.length;
            e = e.childEdgeIndices?.length
                ? e.childEdgeIndices
                      .map((i) => edges[i])
                      .reduce((a, b) => (a.radiusEnd > b.radiusEnd ? a : b))
                : null;
        }
        for (const branch of branches) {
            length = 0;
            e = branch;
            while (e && length < reach) {
                owners.set(e.index, trunk);
                length += e.length;
                e = e.childEdgeIndices?.length === 1 ? edges[e.childEdgeIndices[0]] : null;
            }
        }
    }
    return owners;
}

export function sampleOstialVolumeOwner(candidates, point, supportRadius) {
    if (!candidates) return null;
    let best = null,
        bestDistance = Infinity;
    const offset = new THREE.Vector3();
    for (const edge of candidates) {
        if (edge.renderExcluded || edge.transportExcluded) continue;
        offset.subVectors(point, edge.start);
        const length = edge.start.distanceTo(edge.end);
        const t = THREE.MathUtils.clamp(offset.dot(edge.axis) / Math.max(1e-9, length), 0, 1);
        const center = edge.start.clone().lerp(edge.end, t);
        const distance = point.distanceTo(center);
        if (distance < bestDistance) {
            bestDistance = distance;
            best = {
                edge,
                t,
                center,
                radius: THREE.MathUtils.lerp(edge.radiusStart, edge.radiusEnd, t)
            };
        }
    }
    if (!best || bestDistance >= best.radius + supportRadius) return null;
    // Bound the local cylinder by the actual connected trunk. This matters
    // when looking along its axis: a separate vessel behind its end is not
    // part of the common lumen.
    best.lower = Infinity;
    best.upper = -Infinity;
    for (const edge of candidates) {
        for (const p of [edge.start, edge.end]) {
            const along = offset.subVectors(p, best.center).dot(best.edge.axis);
            best.lower = Math.min(best.lower, along);
            best.upper = Math.max(best.upper, along);
        }
    }
    return best;
}
