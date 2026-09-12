import * as THREE from 'three';
import {
    BRAIN_MIDLINE_X_MM,
    HEAD_ARTERY_CIRCLE_Y_MM
} from './headArteries.js';

const FULL_ANATOMY_MINIMUM_Y_MM = -1200;
const FULL_ANATOMY_MAXIMUM_Y_MM = 350;
const CAROTID_ATTACHMENT_MINIMUM_Y_MM = 115;
const CAROTID_ATTACHMENT_MAXIMUM_Y_MM = 132;
const CAROTID_ATTACHMENT_MAXIMUM_DISTANCE_MM = 3;
const CAROTID_ATTACHMENT_MINIMUM_RADIUS_MM = 1.5;
const CAROTID_ATTACHMENT_MAXIMUM_ABS_X_MM = 35;
const CAROTID_ATTACHMENT_MINIMUM_Z_MM = -12;
const ACOM_CUT_TARGET_Z_MM = 30;
const ACOM_CUT_MAXIMUM_DISTANCE_MM = 6;
const ACOM_CUT_MAXIMUM_RADIUS_MM = 1.2;

export const AORTIC_INLET_TARGET = Object.freeze({ x: 0, y: 0, z: 0 });

function endpointRadius(segment, atStart) {
    return atStart ? segment.radiusStart : segment.radiusEnd;
}

function buildNodes(segments) {
    const nodes = new Map();
    const add = (nodeId, point, radius, segmentIndex, otherId) => {
        let node = nodes.get(nodeId);
        if (!node) {
            node = {
                id: nodeId,
                point: point.clone(),
                radius: radius || 0,
                links: []
            };
            nodes.set(nodeId, node);
        }
        node.radius = Math.max(node.radius, radius || 0);
        node.links.push({ segmentIndex, otherId });
    };

    segments.forEach((segment, segmentIndex) => {
        add(
            segment.nodeStartId,
            segment.start,
            endpointRadius(segment, true),
            segmentIndex,
            segment.nodeEndId
        );
        add(
            segment.nodeEndId,
            segment.end,
            endpointRadius(segment, false),
            segmentIndex,
            segment.nodeStartId
        );
    });
    return nodes;
}

function anatomyBounds(segments) {
    let minimumY = Infinity;
    let maximumY = -Infinity;
    for (const segment of segments) {
        minimumY = Math.min(minimumY, segment.start.y, segment.end.y);
        maximumY = Math.max(maximumY, segment.start.y, segment.end.y);
    }
    return { minimumY, maximumY };
}

function attachmentPair(nodes) {
    const leaves = [...nodes.values()].filter(node =>
        node.links.length === 1 &&
        node.radius >= CAROTID_ATTACHMENT_MINIMUM_RADIUS_MM &&
        node.point.y >= CAROTID_ATTACHMENT_MINIMUM_Y_MM &&
        node.point.y <= CAROTID_ATTACHMENT_MAXIMUM_Y_MM &&
        Math.abs(node.point.x) <= CAROTID_ATTACHMENT_MAXIMUM_ABS_X_MM &&
        node.point.z >= CAROTID_ATTACHMENT_MINIMUM_Z_MM
    );

    let best = null;
    for (let firstIndex = 0; firstIndex < leaves.length; firstIndex++) {
        for (let secondIndex = firstIndex + 1;
            secondIndex < leaves.length;
            secondIndex++
        ) {
            const first = leaves[firstIndex];
            const second = leaves[secondIndex];
            const distance = first.point.distanceTo(second.point);
            if (
                distance > CAROTID_ATTACHMENT_MAXIMUM_DISTANCE_MM ||
                (best && distance >= best.distance)
            ) continue;
            best = { first, second, distance };
        }
    }
    return best;
}

function pathBetween(nodes, startId, endId) {
    const queue = [startId];
    const previous = new Map([[startId, null]]);
    let queueIndex = 0;
    while (queueIndex < queue.length && !previous.has(endId)) {
        const nodeId = queue[queueIndex++];
        for (const link of nodes.get(nodeId)?.links || []) {
            if (previous.has(link.otherId)) continue;
            previous.set(link.otherId, {
                nodeId,
                segmentIndex: link.segmentIndex
            });
            queue.push(link.otherId);
        }
    }
    if (!previous.has(endId)) return [];

    const segmentIndices = [];
    let nodeId = endId;
    while (nodeId !== startId) {
        const entry = previous.get(nodeId);
        if (!entry) return [];
        segmentIndices.push(entry.segmentIndex);
        nodeId = entry.nodeId;
    }
    segmentIndices.reverse();
    return segmentIndices;
}

function anteriorCommunicatingCut(segments, pathSegmentIndices) {
    const target = new THREE.Vector3(
        BRAIN_MIDLINE_X_MM,
        HEAD_ARTERY_CIRCLE_Y_MM,
        ACOM_CUT_TARGET_Z_MM
    );
    return pathSegmentIndices
        .map(segmentIndex => {
            const segment = segments[segmentIndex];
            const midpoint = segment.start.clone()
                .add(segment.end)
                .multiplyScalar(0.5);
            return {
                segment,
                segmentIndex,
                distance: midpoint.distanceTo(target),
                maximumRadius: Math.max(
                    segment.radiusStart || 0,
                    segment.radiusEnd || 0
                )
            };
        })
        .filter(candidate =>
            candidate.distance <= ACOM_CUT_MAXIMUM_DISTANCE_MM &&
            candidate.maximumRadius <= ACOM_CUT_MAXIMUM_RADIUS_MM
        )
        .sort((a, b) =>
            a.distance - b.distance ||
            a.maximumRadius - b.maximumRadius ||
            a.segmentIndex - b.segmentIndex
        )[0] || null;
}

/**
 * Repairs the acyclic transport centerline after the complete Circle of Willis
 * is reduced to a tree. The medial-axis extraction can leave the left carotid
 * extension about 2 mm away from its original stump and use the ACom as the
 * alternate connection. That makes the entire left ICA flow retrogradely.
 * Move the unavoidable tree cut into the narrow ACom and reconnect the carotid
 * stump, preserving every segment count and a single connected component.
 */
export function repairArterialFlowTopology(sourceSegments) {
    if (!Array.isArray(sourceSegments) || !sourceSegments.length) {
        return {
            segments: sourceSegments,
            rootPoint: null,
            diagnostics: { repaired: false, reason: 'empty-centerline' }
        };
    }

    const bounds = anatomyBounds(sourceSegments);
    const fullAnatomy =
        bounds.minimumY <= FULL_ANATOMY_MINIMUM_Y_MM &&
        bounds.maximumY >= FULL_ANATOMY_MAXIMUM_Y_MM;
    const rootPoint = fullAnatomy
        ? new THREE.Vector3(
            AORTIC_INLET_TARGET.x,
            AORTIC_INLET_TARGET.y,
            AORTIC_INLET_TARGET.z
        )
        : null;
    if (!fullAnatomy) {
        return {
            segments: sourceSegments,
            rootPoint,
            diagnostics: { repaired: false, reason: 'not-full-anatomy' }
        };
    }

    const nodes = buildNodes(sourceSegments);
    const pair = attachmentPair(nodes);
    if (!pair) {
        return {
            segments: sourceSegments,
            rootPoint,
            diagnostics: {
                repaired: false,
                reason: 'no-carotid-attachment-gap'
            }
        };
    }

    const pathSegmentIndices = pathBetween(
        nodes,
        pair.first.id,
        pair.second.id
    );
    const cut = anteriorCommunicatingCut(
        sourceSegments,
        pathSegmentIndices
    );
    if (!cut) {
        return {
            segments: sourceSegments,
            rootPoint,
            diagnostics: {
                repaired: false,
                reason: 'no-safe-acom-cut',
                attachmentGapMm: pair.distance
            }
        };
    }

    const segments = sourceSegments.slice();
    const bridgeAxis = pair.second.point.clone().sub(pair.first.point);
    const bridgeLength = bridgeAxis.length();
    if (bridgeLength > 1e-8) bridgeAxis.multiplyScalar(1 / bridgeLength);
    segments[cut.segmentIndex] = {
        ...cut.segment,
        id: cut.segment.id ?? cut.segmentIndex,
        start: pair.first.point.clone(),
        end: pair.second.point.clone(),
        axis: bridgeAxis,
        length: bridgeLength,
        radiusStart: pair.first.radius,
        radiusEnd: pair.second.radius,
        safeRadius: Math.min(pair.first.radius, pair.second.radius),
        nodeStartId: pair.first.id,
        nodeEndId: pair.second.id,
        source: 'physiological-left-carotid-attachment-bridge'
    };

    return {
        segments,
        rootPoint,
        diagnostics: {
            repaired: true,
            attachmentGapMm: pair.distance,
            attachmentBridgeSourceIndex: cut.segmentIndex,
            attachmentNodeIds: [pair.first.id, pair.second.id],
            acomCutStart: cut.segment.start.toArray(),
            acomCutEnd: cut.segment.end.toArray(),
            aorticInletTarget: rootPoint.toArray()
        }
    };
}
