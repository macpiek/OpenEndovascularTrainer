import fs from 'node:fs';
import * as THREE from 'three';
import { STLLoader } from 'three/examples/jsm/loaders/STLLoader.js';
import { preprocessAortaGeometry } from '../src/aortaPreprocess.js';
import { buildStlSliceCenterline } from '../src/stlCenterline.js';
import { generateVessel } from '../src/vesselGeometry.js';

function loadTransformedAorta() {
    const buffer = fs.readFileSync('res/Aorta_plain.stl');
    const arrayBuffer = buffer.buffer.slice(buffer.byteOffset, buffer.byteOffset + buffer.byteLength);
    const geometry = new STLLoader().parse(arrayBuffer);
    geometry.computeBoundingBox();
    const sourceBox = geometry.boundingBox;
    const sourceSize = sourceBox.getSize(new THREE.Vector3());
    const sourceCenter = sourceBox.getCenter(new THREE.Vector3());
    const { vessel } = generateVessel(140, 0);
    const ys = [];
    for (const segment of vessel.segments || []) {
        if (!segment.isSheath) ys.push(segment.start.y, segment.end.y);
    }
    const top = Math.max(...ys, 0) + 15;
    const bottom = Math.min(...ys, -420) - 15;
    const targetCenter = new THREE.Vector3(
        vessel.branchPoint?.x || 0,
        (top + bottom) * 0.5 + 40,
        vessel.branchPoint?.z || 0
    );
    const targetLength = Math.max(300, top - bottom);
    const scale = targetLength * 1.3 / Math.max(1e-6, sourceSize.z);
    geometry.translate(-sourceCenter.x, -sourceCenter.y, -sourceCenter.z);
    geometry.rotateX(-Math.PI / 2);
    geometry.scale(scale, scale, scale);
    geometry.translate(targetCenter.x, targetCenter.y, targetCenter.z);
    return geometry;
}

function nodeKey(segment, endpoint) {
    const id = endpoint === 'start' ? segment.nodeStartId : segment.nodeEndId;
    if (id !== undefined && id !== null) return String(id);
    const point = endpoint === 'start' ? segment.start : segment.end;
    return `${point.x.toFixed(4)},${point.y.toFixed(4)},${point.z.toFixed(4)}`;
}

function buildNodes(segments) {
    const nodes = new Map();
    const add = (segment, endpoint) => {
        const key = nodeKey(segment, endpoint);
        let node = nodes.get(key);
        if (!node) {
            node = {
                key,
                point: (endpoint === 'start' ? segment.start : segment.end).clone(),
                neighbours: []
            };
            nodes.set(key, node);
        }
        const otherPoint = endpoint === 'start' ? segment.end : segment.start;
        node.neighbours.push(otherPoint.clone());
    };
    for (const segment of segments) {
        add(segment, 'start');
        add(segment, 'end');
    }
    return nodes;
}

function nodeDeflection(node) {
    if (node.neighbours.length !== 2) return 0;
    const first = node.neighbours[0].clone().sub(node.point).normalize();
    const second = node.neighbours[1].clone().sub(node.point).normalize();
    return 180 - THREE.MathUtils.radToDeg(Math.acos(THREE.MathUtils.clamp(first.dot(second), -1, 1)));
}

function svgProjection(geometry, centerline, nodes) {
    const panelWidth = 620;
    const panelHeight = 720;
    const margin = 34;
    const projections = [
        { name: 'Front X/Y', axes: ['x', 'y'] },
        { name: 'Side Z/Y', axes: ['z', 'y'] },
        { name: 'Axial X/Z', axes: ['x', 'z'] }
    ];
    const box = geometry.boundingBox;
    const mapPoint = (point, projection, panelIndex) => {
        const [horizontal, vertical] = projection.axes;
        const minHorizontal = box.min[horizontal];
        const maxHorizontal = box.max[horizontal];
        const minVertical = box.min[vertical];
        const maxVertical = box.max[vertical];
        const scale = Math.min(
            (panelWidth - margin * 2) / Math.max(1e-6, maxHorizontal - minHorizontal),
            (panelHeight - margin * 2) / Math.max(1e-6, maxVertical - minVertical)
        );
        const contentWidth = (maxHorizontal - minHorizontal) * scale;
        const contentHeight = (maxVertical - minVertical) * scale;
        return {
            x: panelIndex * panelWidth + (panelWidth - contentWidth) * 0.5 +
                (point[horizontal] - minHorizontal) * scale,
            y: (panelHeight - contentHeight) * 0.5 +
                (maxVertical - point[vertical]) * scale
        };
    };

    const surface = geometry.attributes.position;
    const parts = [
        `<svg xmlns="http://www.w3.org/2000/svg" width="${panelWidth * projections.length}" height="${panelHeight}" viewBox="0 0 ${panelWidth * projections.length} ${panelHeight}">`,
        '<rect width="100%" height="100%" fill="#081019"/>'
    ];
    for (let panelIndex = 0; panelIndex < projections.length; panelIndex++) {
        const projection = projections[panelIndex];
        parts.push(`<text x="${panelIndex * panelWidth + 20}" y="28" fill="#e8eef5" font-family="Arial" font-size="17">${projection.name}</text>`);
        parts.push(`<rect x="${panelIndex * panelWidth + 8}" y="8" width="${panelWidth - 16}" height="${panelHeight - 16}" fill="none" stroke="#26384a"/>`);
        for (let index = 0; index < surface.count; index += 90) {
            const a = new THREE.Vector3().fromBufferAttribute(surface, index);
            const b = new THREE.Vector3().fromBufferAttribute(surface, Math.min(index + 1, surface.count - 1));
            const pa = mapPoint(a, projection, panelIndex);
            const pb = mapPoint(b, projection, panelIndex);
            parts.push(`<line x1="${pa.x.toFixed(1)}" y1="${pa.y.toFixed(1)}" x2="${pb.x.toFixed(1)}" y2="${pb.y.toFixed(1)}" stroke="#61778a" stroke-opacity="0.16" stroke-width="0.55"/>`);
        }
        for (const segment of centerline.segments) {
            const a = mapPoint(segment.start, projection, panelIndex);
            const b = mapPoint(segment.end, projection, panelIndex);
            parts.push(`<line x1="${a.x.toFixed(2)}" y1="${a.y.toFixed(2)}" x2="${b.x.toFixed(2)}" y2="${b.y.toFixed(2)}" stroke="#3fe0c5" stroke-opacity="0.86" stroke-width="1.15" stroke-linecap="round"/>`);
        }
        for (const node of nodes.values()) {
            const degree = node.neighbours.length;
            const deflection = nodeDeflection(node);
            if (degree === 2 && deflection < 82) continue;
            const point = mapPoint(node.point, projection, panelIndex);
            const color = deflection >= 82 ? '#ff4fa3' : degree > 2 ? '#ffd166' : '#ff6b6b';
            const radius = deflection >= 82 ? 3 : degree > 2 ? 2.2 : 1.8;
            parts.push(`<circle cx="${point.x.toFixed(2)}" cy="${point.y.toFixed(2)}" r="${radius}" fill="${color}"/>`);
        }
    }
    parts.push('</svg>');
    return parts.join('\n');
}

function rasterProjection(geometry, centerline, nodes) {
    const panelWidth = 620;
    const width = panelWidth * 3;
    const height = 720;
    const margin = 34;
    const projections = [['x', 'y'], ['z', 'y'], ['x', 'z']];
    const pixels = Buffer.alloc(width * height * 3);
    for (let offset = 0; offset < pixels.length; offset += 3) {
        pixels[offset] = 8;
        pixels[offset + 1] = 16;
        pixels[offset + 2] = 25;
    }
    const box = geometry.boundingBox;
    const mapPoint = (point, axes, panelIndex) => {
        const [horizontal, vertical] = axes;
        const scale = Math.min(
            (panelWidth - margin * 2) / Math.max(1e-6, box.max[horizontal] - box.min[horizontal]),
            (height - margin * 2) / Math.max(1e-6, box.max[vertical] - box.min[vertical])
        );
        const contentWidth = (box.max[horizontal] - box.min[horizontal]) * scale;
        const contentHeight = (box.max[vertical] - box.min[vertical]) * scale;
        return {
            x: Math.round(panelIndex * panelWidth + (panelWidth - contentWidth) * 0.5 +
                (point[horizontal] - box.min[horizontal]) * scale),
            y: Math.round((height - contentHeight) * 0.5 +
                (box.max[vertical] - point[vertical]) * scale)
        };
    };
    const blendPixel = (x, y, color, alpha = 1) => {
        if (x < 0 || y < 0 || x >= width || y >= height) return;
        const offset = (y * width + x) * 3;
        for (let channel = 0; channel < 3; channel++) {
            pixels[offset + channel] = Math.round(
                pixels[offset + channel] * (1 - alpha) + color[channel] * alpha
            );
        }
    };
    const drawLine = (a, b, color, alpha = 1, thickness = 1) => {
        const steps = Math.max(1, Math.ceil(Math.max(Math.abs(b.x - a.x), Math.abs(b.y - a.y))));
        for (let step = 0; step <= steps; step++) {
            const x = Math.round(a.x + (b.x - a.x) * step / steps);
            const y = Math.round(a.y + (b.y - a.y) * step / steps);
            for (let dy = -thickness; dy <= thickness; dy++) {
                for (let dx = -thickness; dx <= thickness; dx++) {
                    if (dx * dx + dy * dy > thickness * thickness) continue;
                    blendPixel(x + dx, y + dy, color, alpha);
                }
            }
        }
    };
    const drawDot = (point, color, radius) => {
        for (let dy = -radius; dy <= radius; dy++) {
            for (let dx = -radius; dx <= radius; dx++) {
                if (dx * dx + dy * dy <= radius * radius) blendPixel(point.x + dx, point.y + dy, color, 1);
            }
        }
    };
    const surface = geometry.attributes.position;
    for (let panelIndex = 0; panelIndex < projections.length; panelIndex++) {
        const axes = projections[panelIndex];
        for (let index = 0; index < surface.count; index += 90) {
            const a = new THREE.Vector3().fromBufferAttribute(surface, index);
            const b = new THREE.Vector3().fromBufferAttribute(surface, Math.min(index + 1, surface.count - 1));
            drawLine(mapPoint(a, axes, panelIndex), mapPoint(b, axes, panelIndex), [97, 119, 138], 0.18, 0);
        }
        for (const segment of centerline.segments) {
            drawLine(
                mapPoint(segment.start, axes, panelIndex),
                mapPoint(segment.end, axes, panelIndex),
                [63, 224, 197],
                0.88,
                1
            );
        }
        for (const node of nodes.values()) {
            const degree = node.neighbours.length;
            const deflection = nodeDeflection(node);
            if (degree === 2 && deflection < 82) continue;
            const color = deflection >= 82
                ? [255, 79, 163]
                : degree > 2
                    ? [255, 209, 102]
                    : [255, 107, 107];
            drawDot(mapPoint(node.point, axes, panelIndex), color, deflection >= 82 ? 4 : 3);
        }
    }
    return Buffer.concat([Buffer.from(`P6\n${width} ${height}\n255\n`), pixels]);
}

const geometry = loadTransformedAorta();
const preprocessing = preprocessAortaGeometry(geometry);
const centerline = buildStlSliceCenterline(geometry, { lumenField: preprocessing.lumenField });
const nodes = buildNodes(centerline.segments);
const outputDir = 'out/centerline-diagnostics';
fs.mkdirSync(outputDir, { recursive: true });
fs.writeFileSync(`${outputDir}/centerline-projections.svg`, svgProjection(geometry, centerline, nodes));
fs.writeFileSync(`${outputDir}/centerline-projections.ppm`, rasterProjection(geometry, centerline, nodes));
fs.writeFileSync(`${outputDir}/centerline-metrics.json`, JSON.stringify(centerline.diagnostics, null, 2));
fs.writeFileSync(`${outputDir}/centerline-segments.json`, JSON.stringify(
    centerline.segments.map(segment => ({
        start: segment.start.toArray(),
        end: segment.end.toArray(),
        radiusStart: segment.radiusStart,
        radiusEnd: segment.radiusEnd,
        source: segment.source
    })),
    null,
    2
));
console.log(`${outputDir}/centerline-projections.svg`);
console.log(JSON.stringify({
    segments: centerline.segments.length,
    components: centerline.diagnostics.componentCount,
    topology: centerline.diagnostics.centerlineTopologyAfterCleanup,
    coverage: centerline.diagnostics.centerlineCoverage,
    averageOffset: centerline.diagnostics.centerlineCenteringAverageOffset,
    maxOffset: centerline.diagnostics.centerlineCenteringMaxOffset,
    invalidSegments: centerline.diagnostics.centerlineInvalidSegmentCountFinal,
    totalMs: centerline.diagnostics.timings.totalMs
}, null, 2));
