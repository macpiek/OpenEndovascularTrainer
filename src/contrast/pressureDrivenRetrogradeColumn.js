const MM3_PER_ML = 1000;
// The transition is driven by the resolved retrograde volumetric component,
// regardless of whether it comes from a sheath or a catheter. The small
// margin avoids a discontinuous numerical toggle around exact flow balance.
const PRESSURE_DOMINANCE_MARGIN = 0.85;
const VISIBLE_STOCK_FRACTION = 0.08;

function directionDotFlow(direction, location) {
    return direction.x * location.tangentX +
        direction.y * location.tangentY +
        direction.z * location.tangentZ;
}

function portArea(port) {
    if (port?.areaMm2 > 0) return port.areaMm2;
    const radius = Math.max(0.01, port?.radiusMm || 0.01);
    return Math.PI * radius ** 2;
}

export class PressureDrivenRetrogradeColumn {
    constructor(flowNetwork) {
        this.flowNetwork = flowNetwork;
        this.activePath = null;
        this.totalInjectedIodineMassMg = 0;
        this.maximumReversedEdgeCount = 0;
        this.maximumContiguousFilledFraction = 0;
        this.maximumAorticIodineMassMg = 0;
        this.totalAorticHandoffIodineMassMg = 0;
        this.lastPressureRatio = 0;
        this.activationCount = 0;
        this.isFlowReversed = false;
        this.currentRetrogradeFlowMlPerSec = 0;
        this.maximumRetrogradeFlowMlPerSec = 0;
        this.totalSideBranchPerfusionMassMg = 0;
        this.maximumPerfusedSideBranchCount = 0;
    }

    beginStep() {
        this.isFlowReversed = false;
        this.currentRetrogradeFlowMlPerSec = 0;
    }

    emit({
        ports,
        volumeMl,
        rateMlPerSec,
        emissionWindowSeconds,
        medium
    }) {
        if (
            !Array.isArray(ports) ||
            !ports.length ||
            !(volumeMl > 0) ||
            !(rateMlPerSec > 0)
        ) return null;

        const totalWeight = ports.reduce(
            (sum, port) => sum + Math.max(0, port.weight ?? portArea(port)),
            0
        ) || ports.length;
        let dominantPort = null;
        let dominantLocation = null;
        let dominantRetrogradeRate = 0;
        let retrogradeRateMlPerSec = 0;
        for (const port of ports) {
            if (!port?.position || port.valid === false) continue;
            const location = this.flowNetwork.findInjectionLocation(
                port.position,
                port.direction
            );
            if (location.edgeIndex < 0) continue;
            const weight = Math.max(
                0,
                port.weight ?? portArea(port)
            ) / totalWeight;
            const alignment = Math.max(
                0,
                -directionDotFlow(port.direction, location)
            );
            const alignedRate = rateMlPerSec * weight * alignment;
            retrogradeRateMlPerSec += alignedRate;
            if (alignedRate > dominantRetrogradeRate) {
                dominantRetrogradeRate = alignedRate;
                dominantPort = port;
                dominantLocation = location;
            }
        }
        if (!dominantPort || !(retrogradeRateMlPerSec > 0)) return null;

        const port = dominantPort;
        const location = dominantLocation;

        const target = this.flowNetwork.findUpstreamMixingJunction(location);
        if (!target?.pathEdgeIndices?.length) return null;

        const maximumOpposingFlowMm3PerS = target.pathEdgeIndices.reduce(
            (maximum, edgeIndex) => Math.max(
                maximum,
                this.flowNetwork.edges[edgeIndex].meanFlowMm3PerS
            ),
            0
        );
        const injectionFlowMm3PerS =
            retrogradeRateMlPerSec * MM3_PER_ML;
        const pressureRatio = injectionFlowMm3PerS /
            Math.max(1, maximumOpposingFlowMm3PerS);
        this.lastPressureRatio = pressureRatio;
        if (pressureRatio < PRESSURE_DOMINANCE_MARGIN) return null;
        this.isFlowReversed = true;
        this.currentRetrogradeFlowMlPerSec = retrogradeRateMlPerSec;
        this.maximumRetrogradeFlowMlPerSec = Math.max(
            this.maximumRetrogradeFlowMlPerSec,
            retrogradeRateMlPerSec
        );

        const pathChanged =
            !this.activePath ||
            this.activePath.sourceEdgeIndex !== location.edgeIndex ||
            this.activePath.sourceCellIndex !== location.cellIndex;
        if (pathChanged) {
            this.activePath = this._createPath(location, target);
            this.activationCount++;
        }

        // Once injection pressure dominates the ipsilateral arterial inflow,
        // the resolved lumen behaves as a driven plug. Physiological advection
        // is suspended on this path for the current step while a conservative
        // volume remap moves the complete cross-section upstream.
        this.flowNetwork.setFlowOverride(
            [
                ...this.activePath.edgeIndices,
                ...this.activePath.aorticEdgeIndices
            ],
            0
        );

        const stockConcentrationMgPerMm3 =
            medium.iodineMgPerMl / MM3_PER_ML;
        const sideBranchPerfusion =
            this.flowNetwork.configureSideBranchInlets({
                pathEdgeIndices:
                    this.activePath.aorticEdgeIndices,
                excludedEdgeIndices: [
                    ...this.activePath.edgeIndices,
                    ...this.activePath.aorticEdgeIndices
                ],
                availableFlowMm3PerS:
                    volumeMl /
                    Math.max(
                        1e-9,
                        emissionWindowSeconds
                    ) *
                    MM3_PER_ML,
                stockConcentrationMgPerMm3
            });
        const sideBranchVolumeMl =
            sideBranchPerfusion.inletFlowMm3PerS /
            MM3_PER_ML *
            emissionWindowSeconds;
        const axialColumnVolumeMl = Math.max(
            0,
            volumeMl - sideBranchVolumeMl
        );
        const iodineMassMg = volumeMl * medium.iodineMgPerMl;
        const deposited = axialColumnVolumeMl > 1e-9
            ? this.flowNetwork.pushRetrogradeColumn({
                pathCells: this.activePath.cells,
                aorticPathStartIndex:
                    this.activePath.aorticPathStartIndex,
                volumeMm3:
                    axialColumnVolumeMl * MM3_PER_ML,
                stockConcentrationMgPerMm3
            })
            : {
                injectedMassMg: 0,
                aorticMassMg: 0,
                overflowMassMg: 0
            };
        if (!deposited) return null;
        const sideBranchPerfusionMassMg =
            sideBranchVolumeMl * medium.iodineMgPerMl;

        this.totalInjectedIodineMassMg += iodineMassMg;
        this.totalAorticHandoffIodineMassMg +=
            deposited.aorticMassMg +
            sideBranchPerfusionMassMg;
        this.totalSideBranchPerfusionMassMg +=
            sideBranchPerfusionMassMg;
        this.maximumPerfusedSideBranchCount = Math.max(
            this.maximumPerfusedSideBranchCount,
            sideBranchPerfusion.perfusedBranchCount
        );
        this.maximumReversedEdgeCount = Math.max(
            this.maximumReversedEdgeCount,
            this.activePath.edgeIndices.length
        );
        return {
            emittedVolumeMl: volumeMl,
            emittedIodineMassMg: iodineMassMg,
            particleCount: 0,
            mode: 'pressure-driven-retrograde-column'
        };
    }

    _createPath(location, target) {
        const cells = [];
        target.pathEdgeIndices.forEach((edgeIndex, pathIndex) => {
            const edge = this.flowNetwork.edges[edgeIndex];
            const firstCell = pathIndex === 0
                ? location.cellIndex
                : edge.cellCount - 1;
            for (let cellIndex = firstCell; cellIndex >= 0; cellIndex--) {
                cells.push({ edgeIndex, cellIndex });
            }
        });
        const aorticPathStartIndex = cells.length;
        const aorticEdgeIndices = [];
        let aorticEdge = this.flowNetwork.edges[target.edgeIndex];
        let firstAorticCellIndex = target.cellIndex;
        const visitedAorticEdges = new Set();
        while (
            aorticEdge &&
            !visitedAorticEdges.has(aorticEdge.index)
        ) {
            visitedAorticEdges.add(aorticEdge.index);
            aorticEdgeIndices.push(aorticEdge.index);
            for (
                let cellIndex = firstAorticCellIndex;
                cellIndex >= 0;
                cellIndex--
            ) {
                cells.push({
                    edgeIndex: aorticEdge.index,
                    cellIndex
                });
            }
            if (aorticEdge.parentEdgeIndex < 0) break;
            aorticEdge =
                this.flowNetwork.edges[aorticEdge.parentEdgeIndex];
            firstAorticCellIndex = aorticEdge.cellCount - 1;
        }
        return {
            sourceEdgeIndex: location.edgeIndex,
            sourceCellIndex: location.cellIndex,
            edgeIndices: [...target.pathEdgeIndices],
            aorticEdgeIndices,
            cells,
            sourcePathCellCount: aorticPathStartIndex,
            aorticPathStartIndex,
            aorticEdgeIndex: target.edgeIndex,
            aorticCellIndex: target.cellIndex
        };
    }

    updateDiagnostics(stockConcentrationMgPerMl) {
        if (!this.activePath) return;
        const visibleThresholdMgPerMm3 =
            stockConcentrationMgPerMl / MM3_PER_ML * VISIBLE_STOCK_FRACTION;
        let contiguousFilledCells = 0;
        const sourcePathCells = this.activePath.cells.slice(
            0,
            this.activePath.sourcePathCellCount
        );
        for (const { edgeIndex, cellIndex } of sourcePathCells) {
            const edge = this.flowNetwork.edges[edgeIndex];
            const concentration = edge.massMg[cellIndex] /
                Math.max(1e-9, edge.volumes[cellIndex]);
            if (concentration < visibleThresholdMgPerMm3) break;
            contiguousFilledCells++;
        }
        this.maximumContiguousFilledFraction = Math.max(
            this.maximumContiguousFilledFraction,
            contiguousFilledCells / Math.max(1, sourcePathCells.length)
        );

        const aorticEdge = this.flowNetwork.edges[this.activePath.aorticEdgeIndex];
        this.maximumAorticIodineMassMg = Math.max(
            this.maximumAorticIodineMassMg,
            aorticEdge.massMg.reduce((sum, mass) => sum + mass, 0)
        );
    }

    getDiagnostics() {
        return {
            active: this.isFlowReversed,
            activationCount: this.activationCount,
            totalInjectedIodineMassMg: this.totalInjectedIodineMassMg,
            maximumReversedEdgeCount: this.maximumReversedEdgeCount,
            maximumContiguousFilledFraction:
                this.maximumContiguousFilledFraction,
            maximumAorticIodineMassMg: this.maximumAorticIodineMassMg,
            totalAorticHandoffIodineMassMg:
                this.totalAorticHandoffIodineMassMg,
            totalSideBranchPerfusionMassMg:
                this.totalSideBranchPerfusionMassMg,
            maximumPerfusedSideBranchCount:
                this.maximumPerfusedSideBranchCount,
            currentRetrogradeFlowMlPerSec:
                this.currentRetrogradeFlowMlPerSec,
            maximumRetrogradeFlowMlPerSec:
                this.maximumRetrogradeFlowMlPerSec,
            lastPressureRatio: this.lastPressureRatio
        };
    }
}
