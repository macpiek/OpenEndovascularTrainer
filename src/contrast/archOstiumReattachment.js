// The atlas medial tree may route several real arch branches through a shared
// strand inside the aorta. That strand is not their anatomical ostium: it can
// put a downstream subclavian on the upstream side of an aortic injection.
// Reattach only proven exiting branches of an already identified intraluminal
// connector. Edge/source IDs and the anatomical/collision mesh remain intact.
export function reattachArchOstia(network) {
    const {edges, nodes} = network;
    const diagnostics = {repairs: [], retiredEdgeCount: 0};
    const bridges = edges.filter(edge => edge.intraluminalAorticConnector && !edge.transportExcluded);
    if (!bridges.length) return diagnostics;

    // Identify the systemic aortic route independently of current flow (this
    // repair runs before resistance initialization). It leads to the most
    // inferior native territory, whereas the supra-aortic branches do not.
    const minimumY = new Map();
    for (let i = network.nodeOrder.length - 1; i >= 0; i--) {
        const node = nodes.get(network.nodeOrder[i]);
        let y = node.point.y;
        for (const index of node.childEdgeIndices) if (!edges[index].transportExcluded)
            y = Math.min(y, minimumY.get(edges[index].endNodeId) ?? y);
        minimumY.set(node.id, y);
    }
    const main = [], mainEdges = new Set();
    let node = network.rootNode, distance = 0;
    while (node) {
        const choices = node.childEdgeIndices.map(index => edges[index]).filter(edge => !edge.transportExcluded);
        if (!choices.length) break;
        choices.sort((a, b) => minimumY.get(a.endNodeId) - minimumY.get(b.endNodeId));
        const edge = choices[0];
        main.push({edge, distance}); mainEdges.add(edge.index); distance += edge.length;
        if (edge.end.y < network.rootNode.point.y - 150) break;
        node = nodes.get(edge.endNodeId);
    }
    if (main.length < 2) return diagnostics;
    const nearest = point => {
        let best = null;
        for (const item of main) {
            const edge = item.edge;
            const t = Math.max(0, Math.min(1, point.clone().sub(edge.start).dot(edge.axis) / edge.length));
            const center = edge.start.clone().lerp(edge.end, t), d = point.distanceTo(center);
            if (!best || d < best.distance) best = {edge, t, distance: d,
                radius: edge.radiusStart + (edge.radiusEnd - edge.radiusStart) * t,
                station: item.distance + t * edge.length};
        }
        return best;
    };
    const plans = [];
    for (const bridge of bridges) for (const originalRootEdgeIndex of bridge.childEdgeIndices) {
        const root = edges[originalRootEdgeIndex];
        if (root.transportExcluded || root.lumenMappingExcluded || mainEdges.has(root.index)) continue;
        const chain = [];
        let edge = root, length = 0;
        while (edge && !edge.transportExcluded && !mainEdges.has(edge.index)) {
            chain.push({edge, distance: length}); length += edge.length;
            if (edge.childEdgeIndices.length !== 1) break;
            edge = edges[edge.childEdgeIndices[0]];
        }
        // Ignore short contained terminal strands. A real exiting branch has
        // substantial independent artery beyond its neck, not just a cap.
        let exit = -1;
        for (let i = 0; i + 2 < chain.length; i++) {
            if (chain[i].distance < 8) continue;
            const sample = nearest(chain[i].edge.end);
            if (sample.distance <= sample.radius) continue;
            if (!chain.slice(i + 1, i + 3).every(item => {
                const next = nearest(item.edge.end); return next.distance > next.radius;
            })) continue;
            if (length - chain[i].distance < Math.max(20, 3 * chain[i].edge.radiusEnd)) break;
            exit = i; break;
        }
        if (exit < 1) continue;
        const head = chain[exit].edge, neck = head.end;
        const location = nearest(neck);
        let ancestor = edges[root.parentEdgeIndex];
        while (ancestor && !mainEdges.has(ancestor.index)) ancestor = edges[ancestor.parentEdgeIndex];
        if (!ancestor) continue;
        const original = nearest(ancestor.end);
        if (location.radius < 6 || location.station - original.station < 5) continue;
        const parent = location.t < .5 ? edges[location.edge.parentEdgeIndex] : location.edge;
        if (!parent || !mainEdges.has(parent.index) || parent.index === head.index) continue;
        plans.push({root, head, parent, retired: chain.slice(0, exit).map(item => item.edge.index),
            mainDistanceMm: location.station, originalMainDistanceMm: original.station});
    }
    const retired = new Set();
    const detach = edge => {
        const children = nodes.get(edge.startNodeId).childEdgeIndices;
        const at = children.indexOf(edge.index); if (at >= 0) children.splice(at, 1);
    };
    const retire = edge => {
        retired.add(edge.index);
        edge.transportExcluded = edge.renderExcluded = edge.lumenMappingExcluded = true;
        edge.retiredArchPrefix = true;
    };
    for (const plan of plans) {
        const {root, head, parent} = plan;
        detach(root); detach(head);
        for (const index of plan.retired) retire(edges[index]);
        const oldStartNodeId = root.startNodeId;
        head.startNodeId = parent.endNodeId;
        head.start.copy(parent.end); head.axis.subVectors(head.end, head.start);
        head.length = head.axis.length(); head.axis.divideScalar(head.length);
        head.cellCount = Math.max(1, Math.ceil(head.length / network.cellLengthMm));
        head.cellLength = head.length / head.cellCount;
        head.volumes = new Float64Array(head.cellCount); head.areas = new Float64Array(head.cellCount);
        head.massMg = new Float64Array(head.cellCount); head.nextMassMg = new Float64Array(head.cellCount);
        head.faceFlowDeltaMm3PerS = new Float64Array(head.cellCount + 1);
        head.physicalArchOstium = true; head.originalArchRootEdgeIndex = root.index;
        network._rebuildEdgeTransportGeometry(head);
        nodes.get(parent.endNodeId).childEdgeIndices.push(head.index);
        // Retiring both daughters must not turn their former shared connector
        // into a new tissue outlet. Prune only newly empty technical ancestors.
        let oldNode = nodes.get(oldStartNodeId);
        while (oldNode && oldNode.childEdgeIndices.length === 0) {
            const previous = edges[oldNode.parentEdgeIndex];
            if (!previous || mainEdges.has(previous.index)) break;
            detach(previous); retire(previous); oldNode = nodes.get(previous.startNodeId);
        }
        diagnostics.repairs.push({originalRootEdgeIndex: root.index, rootEdgeIndex: head.index,
            parentEdgeIndex: parent.index, retiredEdgeIndices: plan.retired,
            mainDistanceMm: plan.mainDistanceMm, originalMainDistanceMm: plan.originalMainDistanceMm,
            originalStart: root.start.toArray(), ostium: head.start.toArray(), neck: head.end.toArray()});
    }
    if (plans.length) {
        for (const node of nodes.values()) node.links.length = 0;
        for (const edge of edges) {
            edge.parentEdgeIndex = nodes.get(edge.startNodeId).parentEdgeIndex;
            edge.childEdgeIndices = nodes.get(edge.endNodeId).childEdgeIndices;
            if (!edge.transportExcluded) {
                nodes.get(edge.startNodeId).links.push({sourceIndex: edge.sourceIndex, otherId: edge.endNodeId});
                nodes.get(edge.endNodeId).links.push({sourceIndex: edge.sourceIndex, otherId: edge.startNodeId});
            }
        }
        const queue = [network.rootNode.id], visited = new Set(queue);
        for (let i = 0; i < queue.length; i++) for (const index of nodes.get(queue[i]).childEdgeIndices) {
            const next = edges[index].endNodeId;
            if (visited.has(next)) throw new Error('Arch ostium repair must retain an acyclic transport tree');
            visited.add(next); queue.push(next);
        }
        network.nodeOrder = queue;
    }
    diagnostics.retiredEdgeCount = retired.size;
    diagnostics.retiredEdgeIndices = [...retired];
    return diagnostics;
}
