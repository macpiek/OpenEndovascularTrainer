import fs from 'node:fs';
import path from 'node:path';
import zlib from 'node:zlib';
import * as THREE from 'three';
import { ContrastFlowNetwork } from '../src/contrast/flowNetwork.js';
import { ContrastVolumeRenderer } from '../src/contrast/contrastVolumeRenderer.js';
import { LocalContrastInjectionSolver } from '../src/contrast/localInjectionSolver.js';
import { decodeCollisionAsset } from '../src/physics/collision/collisionAssetFormat.js';
import { VesselContactField } from '../src/physics/collision/vesselContactField.js';

const IMAGE_SIZE = 480;
const FIELD_OF_VIEW_MM = 90;
const SIGNAL_GAIN = 0.14;

function loadCenterlineSegments() {
    const sourceBuffer = fs.readFileSync(
        new URL('../res/Aorta_plain.collision.bin', import.meta.url)
    );
    const arrayBuffer = sourceBuffer.buffer.slice(
        sourceBuffer.byteOffset,
        sourceBuffer.byteOffset + sourceBuffer.byteLength
    );
    const asset = decodeCollisionAsset(arrayBuffer);
    const data = asset.arrays.centerlineSegments;
    const nodeEdges = asset.arrays.centerlineEdges;
    const stride = asset.metadata.centerline.stride;
    const segments = [];
    for (let index = 0; index < data.length / stride; index++) {
        const offset = index * stride;
        segments.push({
            id: index,
            start: new THREE.Vector3(
                data[offset],
                data[offset + 1],
                data[offset + 2]
            ),
            end: new THREE.Vector3(
                data[offset + 3],
                data[offset + 4],
                data[offset + 5]
            ),
            radiusStart: data[offset + 6],
            radiusEnd: data[offset + 7],
            safeRadius: data[offset + 8],
            nodeStartId: nodeEdges[index * 2],
            nodeEndId: nodeEdges[index * 2 + 1]
        });
    }
    return { asset, segments };
}

function findAortoiliacParent(network) {
    return network.edges.find(edge =>
        edge.end.y < -275 &&
        edge.end.y > -305 &&
        edge.radiusEnd > 6 &&
        edge.childEdgeIndices.length === 2 &&
        edge.childEdgeIndices.every(
            childIndex => network.edges[childIndex].radiusStart > 6
        )
    );
}

function upstreamPath(network, firstEdge, maximumDistanceMm) {
    const edges = [];
    let edge = firstEdge;
    let distance = 0;
    while (edge && distance < maximumDistanceMm) {
        edges.push(edge);
        distance += edge.length;
        edge = edge.parentEdgeIndex >= 0
            ? network.edges[edge.parentEdgeIndex]
            : null;
    }
    return edges;
}

function downstreamPath(network, firstEdge, maximumDistanceMm) {
    const edges = [];
    let edge = firstEdge;
    let distance = 0;
    const visited = new Set();
    while (
        edge &&
        !visited.has(edge.index) &&
        distance < maximumDistanceMm
    ) {
        visited.add(edge.index);
        edges.push(edge);
        distance += edge.length;
        edge = edge.childEdgeIndices.reduce((widest, childIndex) => {
            const child = network.edges[childIndex];
            if (!widest) return child;
            return child.meanFlowMm3PerS > widest.meanFlowMm3PerS
                ? child
                : widest;
        }, null);
    }
    return edges;
}

function smoothstep(edge0, edge1, value) {
    const t = Math.max(
        0,
        Math.min(1, (value - edge0) / Math.max(1e-9, edge1 - edge0))
    );
    return t * t * (3 - 2 * t);
}

function toProjectionView(point, center, angleDegrees) {
    const angleRadians = THREE.MathUtils.degToRad(angleDegrees);
    const cosine = Math.cos(angleRadians);
    const sine = Math.sin(angleRadians);
    const x = point.x - center.x;
    const z = point.z - center.z;
    return new THREE.Vector3(
        center.x + x * cosine + z * sine,
        point.y,
        center.z - x * sine + z * cosine
    );
}

function project(point, center, angleDegrees = 0) {
    const viewPoint = toProjectionView(point, center, angleDegrees);
    const scale = IMAGE_SIZE / FIELD_OF_VIEW_MM;
    return {
        x: (viewPoint.x - center.x) * scale + IMAGE_SIZE * 0.5,
        y: (center.y - viewPoint.y) * scale + IMAGE_SIZE * 0.5
    };
}

function rasterize(
    renderer,
    center,
    surfaceMode = 'all',
    projectionAngleDegrees = 0
) {
    const geometry = renderer.flowMesh.geometry;
    const positions = geometry.attributes.position.array;
    const normals = geometry.attributes.normal.array;
    const radii = geometry.attributes.flowRadius.array;
    const weights = geometry.attributes.flowOpticalWeight.array;
    const concentrations = geometry.attributes.flowConcentration.array;
    const indices = geometry.index.array;
    const signal = new Float32Array(IMAGE_SIZE * IMAGE_SIZE);

    for (let offset = 0; offset < indices.length; offset += 3) {
        const ia = indices[offset];
        const ib = indices[offset + 1];
        const ic = indices[offset + 2];
        if (
            concentrations[ia] < 1e-5 &&
            concentrations[ib] < 1e-5 &&
            concentrations[ic] < 1e-5
        ) continue;
        const connectorTriangle =
            renderer._flowVertexIsJunctionConnector[ia] &&
            renderer._flowVertexIsJunctionConnector[ib] &&
            renderer._flowVertexIsJunctionConnector[ic];
        const trueConnectorStart =
            renderer.flowTubeIndexCount +
            renderer.flowJunctionTubeIndexCount;
        const sideOstiumTubeStart =
            trueConnectorStart +
            renderer.flowJunctionConnectorIndexCount;
        const sideOstiumConnectorStart =
            sideOstiumTubeStart +
            renderer.flowSideOstiumTubeIndexCount;
        const trueConnectorTriangle =
            offset >= trueConnectorStart &&
            offset < sideOstiumTubeStart;
        const sideOstiumTubeTriangle =
            offset >= sideOstiumTubeStart &&
            offset < sideOstiumConnectorStart;
        const sideOstiumConnectorTriangle =
            offset >= sideOstiumConnectorStart;
        const maximumUnionTriangle =
            offset >= renderer.flowTubeIndexCount;
        if (
            (surfaceMode === 'connector' && !connectorTriangle) ||
            (surfaceMode === 'tubes' && connectorTriangle)
        ) continue;
        const a = new THREE.Vector3(
            positions[ia * 3],
            positions[ia * 3 + 1],
            positions[ia * 3 + 2]
        );
        const b = new THREE.Vector3(
            positions[ib * 3],
            positions[ib * 3 + 1],
            positions[ib * 3 + 2]
        );
        const c = new THREE.Vector3(
            positions[ic * 3],
            positions[ic * 3 + 1],
            positions[ic * 3 + 2]
        );
        const viewA = toProjectionView(
            a,
            center,
            projectionAngleDegrees
        );
        const viewB = toProjectionView(
            b,
            center,
            projectionAngleDegrees
        );
        const viewC = toProjectionView(
            c,
            center,
            projectionAngleDegrees
        );
        const faceNormalZ = viewB.clone().sub(viewA)
            .cross(viewC.clone().sub(viewA)).z;
        if (faceNormalZ <= 0) continue;

        const pa = project(a, center, projectionAngleDegrees);
        const pb = project(b, center, projectionAngleDegrees);
        const pc = project(c, center, projectionAngleDegrees);
        const minimumX = Math.max(
            0,
            Math.floor(Math.min(pa.x, pb.x, pc.x))
        );
        const maximumX = Math.min(
            IMAGE_SIZE - 1,
            Math.ceil(Math.max(pa.x, pb.x, pc.x))
        );
        const minimumY = Math.max(
            0,
            Math.floor(Math.min(pa.y, pb.y, pc.y))
        );
        const maximumY = Math.min(
            IMAGE_SIZE - 1,
            Math.ceil(Math.max(pa.y, pb.y, pc.y))
        );
        if (minimumX > maximumX || minimumY > maximumY) continue;
        const denominator =
            (pb.y - pc.y) * (pa.x - pc.x) +
            (pc.x - pb.x) * (pa.y - pc.y);
        if (Math.abs(denominator) < 1e-8) continue;

        for (let y = minimumY; y <= maximumY; y++) {
            for (let x = minimumX; x <= maximumX; x++) {
                const px = x + 0.5;
                const py = y + 0.5;
                const wa = (
                    (pb.y - pc.y) * (px - pc.x) +
                    (pc.x - pb.x) * (py - pc.y)
                ) / denominator;
                const wb = (
                    (pc.y - pa.y) * (px - pc.x) +
                    (pa.x - pc.x) * (py - pc.y)
                ) / denominator;
                const wc = 1 - wa - wb;
                if (wa < 0 || wb < 0 || wc < 0) continue;

                const normalX =
                    normals[ia * 3] * wa +
                    normals[ib * 3] * wb +
                    normals[ic * 3] * wc;
                const normalY =
                    normals[ia * 3 + 1] * wa +
                    normals[ib * 3 + 1] * wb +
                    normals[ic * 3 + 1] * wc;
                const normalZ =
                    normals[ia * 3 + 2] * wa +
                    normals[ib * 3 + 2] * wb +
                    normals[ic * 3 + 2] * wc;
                const normalLength = Math.hypot(
                    normalX,
                    normalY,
                    normalZ
                );
                const projectionAngleRadians =
                    THREE.MathUtils.degToRad(projectionAngleDegrees);
                const viewNormalZ =
                    -normalX * Math.sin(projectionAngleRadians) +
                    normalZ * Math.cos(projectionAngleRadians);
                const chordFactor = Math.abs(
                    viewNormalZ / Math.max(1e-9, normalLength)
                );
                const lumenCoverage = smoothstep(
                    0.015,
                    0.18,
                    chordFactor
                );
                const filledLumenProfile = 0.72 +
                    0.28 * smoothstep(0.08, 0.82, chordFactor);
                const concentration =
                    concentrations[ia] * wa +
                    concentrations[ib] * wb +
                    concentrations[ic] * wc;
                const radius =
                    radii[ia] * wa + radii[ib] * wb + radii[ic] * wc;
                const weight =
                    weights[ia] * wa + weights[ib] * wb + weights[ic] * wc;
                const opticalDepth = Math.max(0, concentration) *
                    Math.max(0.16, radius) *
                    weight *
                    filledLumenProfile *
                    lumenCoverage *
                    SIGNAL_GAIN;
                const pixelIndex = y * IMAGE_SIZE + x;
                const transferredSignal = opticalDepth * opticalDepth;
                signal[pixelIndex] = maximumUnionTriangle
                    ? Math.max(signal[pixelIndex], transferredSignal)
                    : signal[pixelIndex] + transferredSignal;
            }
        }
    }
    return signal;
}

const CRC_TABLE = new Uint32Array(256).map((_, index) => {
    let value = index;
    for (let bit = 0; bit < 8; bit++) {
        value = value & 1
            ? 0xedb88320 ^ (value >>> 1)
            : value >>> 1;
    }
    return value >>> 0;
});

function crc32(buffer) {
    let value = 0xffffffff;
    for (const byte of buffer) {
        value = CRC_TABLE[(value ^ byte) & 0xff] ^ (value >>> 8);
    }
    return (value ^ 0xffffffff) >>> 0;
}

function pngChunk(type, data) {
    const typeBuffer = Buffer.from(type);
    const length = Buffer.alloc(4);
    length.writeUInt32BE(data.length);
    const checksum = Buffer.alloc(4);
    checksum.writeUInt32BE(crc32(Buffer.concat([typeBuffer, data])));
    return Buffer.concat([length, typeBuffer, data, checksum]);
}

function writeSignalPng(outputPath, signal) {
    const scanlines = Buffer.alloc((IMAGE_SIZE + 1) * IMAGE_SIZE);
    for (let y = 0; y < IMAGE_SIZE; y++) {
        const row = y * (IMAGE_SIZE + 1);
        scanlines[row] = 0;
        for (let x = 0; x < IMAGE_SIZE; x++) {
            const opticalSignal = signal[y * IMAGE_SIZE + x];
            scanlines[row + x + 1] = Math.round(
                255 * Math.exp(-opticalSignal * 1.35)
            );
        }
    }
    const header = Buffer.alloc(13);
    header.writeUInt32BE(IMAGE_SIZE, 0);
    header.writeUInt32BE(IMAGE_SIZE, 4);
    header[8] = 8;
    header[9] = 0;
    const png = Buffer.concat([
        Buffer.from([137, 80, 78, 71, 13, 10, 26, 10]),
        pngChunk('IHDR', header),
        pngChunk('IDAT', zlib.deflateSync(scanlines)),
        pngChunk('IEND', Buffer.alloc(0))
    ]);
    fs.mkdirSync(path.dirname(outputPath), { recursive: true });
    fs.writeFileSync(outputPath, png);
}

function maximumSignalNear(signal, projectedPoint, radiusPixels = 3) {
    let maximum = 0;
    for (
        let y = Math.max(0, Math.round(projectedPoint.y) - radiusPixels);
        y <= Math.min(
            IMAGE_SIZE - 1,
            Math.round(projectedPoint.y) + radiusPixels
        );
        y++
    ) {
        for (
            let x = Math.max(0, Math.round(projectedPoint.x) - radiusPixels);
            x <= Math.min(
                IMAGE_SIZE - 1,
                Math.round(projectedPoint.x) + radiusPixels
            );
            x++
        ) {
            maximum = Math.max(maximum, signal[y * IMAGE_SIZE + x]);
        }
    }
    return maximum;
}

const { asset, segments } = loadCenterlineSegments();
const network = new ContrastFlowNetwork(segments);
const localSolver = new LocalContrastInjectionSolver({
    flowNetwork: network,
    capacity: 16
});
const renderer = new ContrastVolumeRenderer({
    flowNetwork: network,
    localSolver,
    contactField: new VesselContactField(asset),
    medium: { iodineMgPerMl: 300 }
});
const parent = findAortoiliacParent(network);
if (!parent) throw new Error('Aortoiliac bifurcation was not found');
const children = parent.childEdgeIndices.map(index => network.edges[index]);
const parentPath = upstreamPath(network, parent, 55);
const childPaths = children.map(child => downstreamPath(network, child, 55));
const center = parent.end;
const phases = [
    { name: '01-parent-front', concentrations: [1, 0, 0] },
    { name: '02-early-split', concentrations: [1, 0.35, 0.2] },
    { name: '03-full-y', concentrations: [1, 1, 1] },
    { name: '04-washout', concentrations: [0.2, 0.72, 0.64] }
];
const projections = [
    { name: 'AP', angleDegrees: 0 },
    { name: 'LAO 30', angleDegrees: 30 },
    { name: 'RAO 30', angleDegrees: -30 }
];
const report = {
    schemaVersion: 1,
    projection:
        'AP/LAO/RAO software reproduction of the production lumen shader',
    parentEdgeIndex: parent.index,
    childEdgeIndices: children.map(child => child.index),
    phases: []
};
const aortoiliacJunctionDiagnostic =
    renderer.flowJunctionUnionDiagnostics.find(
        diagnostic => diagnostic.parentEdgeIndex === parent.index
    );
const clippedSideOstia = renderer.flowJunctionUnionDiagnostics.filter(
    diagnostic =>
        diagnostic.geometryKind ===
            'implicit-radius-matched-side-ostium-union' &&
        diagnostic.connectorSurfaceSuppressed
);
const worstClippedSideOstium = clippedSideOstia.reduce(
    (worst, diagnostic) =>
        !worst ||
        diagnostic.maximumAnatomicalOutsideDistanceMm >
            worst.maximumAnatomicalOutsideDistanceMm
            ? diagnostic
            : worst,
    null
);

for (const projection of projections) {
for (const phase of phases) {
    renderer._flowCellDisplayConcentration.fill(0);
    const pathGroups = [parentPath, ...childPaths];
    for (let groupIndex = 0; groupIndex < pathGroups.length; groupIndex++) {
        for (const edge of pathGroups[groupIndex]) {
            const offset = renderer._flowCellOffset[edge.index];
            renderer._flowCellDisplayConcentration.fill(
                phase.concentrations[groupIndex],
                offset,
                offset + edge.cellCount
            );
        }
    }
    for (
        let vertexIndex = 0;
        vertexIndex < renderer._flowVertexConcentration.length;
        vertexIndex++
    ) {
        renderer._flowVertexConcentration[vertexIndex] =
            renderer._sampleFlowCellConcentration(
                renderer._flowVertexConcentrationEdgeIndex[vertexIndex],
                renderer._flowVertexConcentrationEdgeT[vertexIndex]
            );
    }
    renderer._updateTrueJunctionConnectorConcentrations();
    renderer._updateJunctionOpticalWeights();
    const signal = rasterize(
        renderer,
        center,
        'all',
        projection.angleDegrees
    );
    const projectionSlug = projection.name
        .toLowerCase()
        .replaceAll(' ', '-');
    const outputPath = path.resolve(
        `reports/contrast-bifurcation-regression-${projectionSlug}-${phase.name}.png`
    );
    writeSignalPng(outputPath, signal);
    if (projection.name === 'AP' && phase.name === '03-full-y') {
        writeSignalPng(
            path.resolve('reports/contrast-bifurcation-connector-only.png'),
            rasterize(renderer, center, 'connector', 0)
        );
        writeSignalPng(
            path.resolve('reports/contrast-bifurcation-tubes-only.png'),
            rasterize(renderer, center, 'tubes', 0)
        );
    }

    const nodeSignal = maximumSignalNear(
        signal,
        project(center, center, projection.angleDegrees),
        4
    );
    const parentReferenceSignal = maximumSignalNear(
        signal,
        project(
            center.clone().addScaledVector(parent.axis, -12),
            center,
            projection.angleDegrees
        ),
        4
    );
    const childReferenceSignals = children.map(child =>
        maximumSignalNear(
            signal,
            project(
                center.clone().addScaledVector(child.axis, 12),
                center,
                projection.angleDegrees
            ),
            4
        )
    );
    report.phases.push({
        projection: projection.name,
        name: phase.name,
        concentrations: phase.concentrations,
        outputPath,
        nodeSignal,
        parentReferenceSignal,
        childReferenceSignals,
        nodeToStrongestReferenceRatio:
            nodeSignal /
            Math.max(
                1e-9,
                parentReferenceSignal,
                ...childReferenceSignals
            )
    });
}
}

if (worstClippedSideOstium) {
    renderer._flowCellDisplayConcentration.fill(1);
    for (
        let vertexIndex = 0;
        vertexIndex < renderer._flowVertexConcentration.length;
        vertexIndex++
    ) {
        renderer._flowVertexConcentration[vertexIndex] =
            renderer._sampleFlowCellConcentration(
                renderer._flowVertexConcentrationEdgeIndex[vertexIndex],
                renderer._flowVertexConcentrationEdgeT[vertexIndex]
            );
    }
    renderer._updateTrueJunctionConnectorConcentrations();
    renderer._updateJunctionOpticalWeights();
    const sideOstiumParent =
        network.edges[worstClippedSideOstium.parentEdgeIndex];
    const sideOstiumCenter = sideOstiumParent.end;
    const sideOstiumImages = [];
    const sideOstiumNodeRatios = [];
    for (const projection of projections) {
        const signal = rasterize(
            renderer,
            sideOstiumCenter,
            'all',
            projection.angleDegrees
        );
        const projectionSlug = projection.name
            .toLowerCase()
            .replaceAll(' ', '-');
        const outputPath = path.resolve(
            `reports/contrast-side-ostium-regression-${projectionSlug}.png`
        );
        writeSignalPng(outputPath, signal);
        if (projection.name === 'AP') {
            writeSignalPng(
                path.resolve(
                    'reports/contrast-side-ostium-connector-only.png'
                ),
                rasterize(renderer, sideOstiumCenter, 'connector', 0)
            );
            writeSignalPng(
                path.resolve(
                    'reports/contrast-side-ostium-tubes-only.png'
                ),
                rasterize(renderer, sideOstiumCenter, 'tubes', 0)
            );
        }
        sideOstiumImages.push(outputPath);
        const nodeSignal = maximumSignalNear(
            signal,
            project(
                sideOstiumCenter,
                sideOstiumCenter,
                projection.angleDegrees
            ),
            4
        );
        const referenceSignals = [
            maximumSignalNear(
                signal,
                project(
                    sideOstiumCenter.clone().addScaledVector(
                        sideOstiumParent.axis,
                        -12
                    ),
                    sideOstiumCenter,
                    projection.angleDegrees
                ),
                4
            ),
            ...sideOstiumParent.childEdgeIndices.map(childIndex => {
                const child = network.edges[childIndex];
                return maximumSignalNear(
                    signal,
                    project(
                        sideOstiumCenter.clone().addScaledVector(
                            child.axis,
                            12
                        ),
                        sideOstiumCenter,
                        projection.angleDegrees
                    ),
                    4
                );
            })
        ];
        sideOstiumNodeRatios.push(
            nodeSignal / Math.max(1e-9, ...referenceSignals)
        );
    }
    report.sideOstiumClip = {
        nodeId: worstClippedSideOstium.nodeId,
        parentEdgeIndex: worstClippedSideOstium.parentEdgeIndex,
        clippedVertexCount:
            worstClippedSideOstium.anatomicalClippedVertexCount,
        minimumCoverage:
            worstClippedSideOstium.minimumAnatomicalCoverage,
        maximumOutsideDistanceMm:
            worstClippedSideOstium.maximumAnatomicalOutsideDistanceMm,
        nodeToReferenceRatios: sideOstiumNodeRatios,
        outputPaths: sideOstiumImages
    };
}

for (const phase of report.phases) {
    const fullyFilledNodeSignal = report.phases.find(
        candidate =>
            candidate.projection === phase.projection &&
            candidate.name === '03-full-y'
    )?.nodeSignal || 0;
    const strongestConcentration = Math.max(...phase.concentrations);
    phase.nodeToExpectedStrongestSignalRatio =
        phase.nodeSignal /
        Math.max(
            1e-9,
            fullyFilledNodeSignal * strongestConcentration ** 2
        );
}
report.acceptance = {
    minimumNormalizedNodeSignal: Math.min(
        ...report.phases.map(
            phase => phase.nodeToExpectedStrongestSignalRatio
        )
    ),
    maximumNormalizedNodeSignal: Math.max(
        ...report.phases.map(
            phase => phase.nodeToExpectedStrongestSignalRatio
        )
    )
};
report.acceptance.passed =
    report.acceptance.minimumNormalizedNodeSignal >= 0.8 &&
    report.acceptance.maximumNormalizedNodeSignal <= 1.2 &&
    aortoiliacJunctionDiagnostic?.minimumAnatomicalCoverage === 1 &&
    clippedSideOstia.length > 0 &&
    report.sideOstiumClip?.minimumCoverage < 0.999 &&
    Math.max(...report.sideOstiumClip.nodeToReferenceRatios) <= 1.2;
report.acceptance.aortoiliacCoverage =
    aortoiliacJunctionDiagnostic?.minimumAnatomicalCoverage ?? null;
report.acceptance.clippedSideOstiumCount = clippedSideOstia.length;
report.acceptance.maximumSideOstiumNodeToReferenceRatio =
    report.sideOstiumClip
        ? Math.max(...report.sideOstiumClip.nodeToReferenceRatios)
        : null;

const reportPath = path.resolve(
    'reports/contrast-bifurcation-regression.json'
);
fs.writeFileSync(reportPath, `${JSON.stringify(report, null, 2)}\n`);
console.log(JSON.stringify({
    reportPath,
    acceptance: report.acceptance,
    phases: report.phases
}, null, 2));
renderer.dispose();
if (!report.acceptance.passed) {
    throw new Error(
        `Bifurcation visual regression failed: ${JSON.stringify(report.acceptance)}`
    );
}
