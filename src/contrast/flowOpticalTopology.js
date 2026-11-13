// Some atlas connectors carry blood between nearby arch ostia while having
// no independent wall. They are transparent to optical ancestry, unlike an
// excluded/occluded lumen or a graft replacement.
export const isIntraluminalOpticalBridge = edge => Boolean(
    edge?.intraluminalAorticConnector && !edge.transportExcluded
);

export function opticalMappedEdgeIndex(network, territories, edgeIndex) {
    let edge = network.edges[edgeIndex];
    while (edge && territories[edge.index] < 0 && isIntraluminalOpticalBridge(edge))
        edge = network.edges[edge.parentEdgeIndex];
    return edge && territories[edge.index] >= 0 ? edge.index : -1;
}

export function opticalParentEdgeIndex(network, territories, edgeIndex) {
    return opticalMappedEdgeIndex(network, territories,
        network.edges[edgeIndex]?.parentEdgeIndex ?? -1);
}
