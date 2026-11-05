// Terminal calibre estimates unresolved tissue demand in the native atlas.
// A kidney contributes one territory at its attachment, whatever the number
// of resolved leaves. Inside that territory the distal beds share its budget.
export function initializeOutletResistances(network) {
    const {edges, nodes, rootNode, nodeOrder} = network;
    const reference = network.peripheralResistancePaSPerMm3;
    const renalRoot = edge => edge.renalSide && edges[edge.parentEdgeIndex]?.renalSide !== edge.renalSide;
    const weight = edge => renalRoot(edge)
        ? edge.radiusStart ** 2 : nodes.get(edge.endNodeId).tissueTerritoryWeight;
    for (let i = nodeOrder.length - 1; i >= 0; i--) {
        const node = nodes.get(nodeOrder[i]), parent = edges[node.parentEdgeIndex];
        const children = node.childEdgeIndices.map(i => edges[i]).filter(e => !e.transportExcluded);
        node.nativeOutlet = children.length === 0 && !!parent && !parent.transportExcluded;
        // The native atlas ends at artificial caps. A small medial-axis
        // radius at a cap must not imply a negligible downstream tissue bed.
        let terminalWeight = node.terminalRadius ** 2;
        if (node.nativeOutlet && !parent.renalSide) {
            let edge = parent, volume = 0, length = 0;
            while (edge && !edge.transportExcluded && !edge.renalSide) {
                volume += edge.totalVolume;
                length += edge.length;
                const previous = edges[edge.parentEdgeIndex];
                if (!previous || previous.childEdgeIndices.length !== 1) break;
                edge = previous;
            }
            // Average supply calibre along the final unbranched artery,
            // not the artificial narrowing at its mesh cap. Volume/length
            // is invariant when this same artery is split into more samples.
            terminalWeight = Math.max(network.minimumOutletTerritoryRadiusMm ** 2,
                volume / (Math.PI * Math.max(1e-9, length)));
        }
        node.tissueTerritoryWeight = node.nativeOutlet ? terminalWeight
            : children.reduce((sum, edge) => sum + weight(edge), 0);
    }
    rootNode.tissueTerritoryFraction = 1;
    for (const id of nodeOrder) {
        const node = nodes.get(id);
        const children = node.childEdgeIndices.map(i => edges[i]).filter(e => !e.transportExcluded);
        if (node.nativeOutlet) {
            node.outletResistancePaSPerMm3 = reference / node.tissueTerritoryFraction;
            continue;
        }
        const total = children.reduce((sum, edge) => sum + weight(edge), 0);
        for (const edge of children) nodes.get(edge.endNodeId).tissueTerritoryFraction =
            node.tissueTerritoryFraction * weight(edge) / total;
    }

    // A kidney is a fixed tissue bed. More resolved intrarenal branches replace
    // some of the unresolved bed resistance; they do not add parallel organs.
    // Fit nonnegative distal resistances so artery + bed has the same target
    // resistance at the renal attachment, independent of leaf count. Do this
    // once: later stenoses/occlusions must still change flow, not be cancelled.
    network.renalResistanceDiagnostics = [];
    for (const root of edges) {
        if (!root.renalSide || root.transportExcluded || edges[root.parentEdgeIndex]?.renalSide === root.renalSide) continue;
        const subtree = [root];
        for (let i = 0; i < subtree.length; i++) {
            for (const index of subtree[i].childEdgeIndices) {
                const edge = edges[index];
                if (!edge.transportExcluded) subtree.push(edge);
            }
        }
        const target = reference / nodes.get(root.endNodeId).tissueTerritoryFraction;
        const equivalent = new Map();
        const evaluate = scale => {
            for (let i = subtree.length - 1; i >= 0; i--) {
                const edge = subtree[i], node = nodes.get(edge.endNodeId);
                let downstream;
                if (node.nativeOutlet) downstream = node.outletResistancePaSPerMm3 * scale;
                else {
                    let conductance = 0;
                    for (const index of edge.childEdgeIndices)
                        if (!edges[index].transportExcluded) conductance += 1 / equivalent.get(index);
                    downstream = conductance > 0 ? 1 / conductance : Infinity;
                }
                equivalent.set(edge.index, edge.resistance + downstream);
            }
            return equivalent.get(root.index);
        };
        const minimum = evaluate(0);
        let lower = 0, upper = 1;
        // Arterial geometry alone may exceed the bed target. Never introduce
        // negative resistance to hide a restrictive vessel.
        if (minimum < target) {
            for (let i = 0; i < 48; i++) {
                const middle = (lower + upper) / 2;
                if (evaluate(middle) < target) lower = middle;
                else upper = middle;
            }
        } else upper = 0;
        const scale = (lower + upper) / 2;
        const actual = evaluate(scale);
        for (const edge of subtree) {
            const node = nodes.get(edge.endNodeId);
            if (node.nativeOutlet) node.outletResistancePaSPerMm3 *= scale;
        }
        network.renalResistanceDiagnostics.push({side: root.renalSide, rootEdgeIndex: root.index,
            targetResistancePaSPerMm3: target, equivalentResistancePaSPerMm3: actual,
            resolvedResistancePaSPerMm3: minimum, distalResistanceScale: scale,
            geometryLimited: minimum >= target});
    }
}
