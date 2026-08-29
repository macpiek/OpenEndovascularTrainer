import * as THREE from 'three';
import { CSS2DObject } from 'three/examples/jsm/renderers/CSS2DRenderer.js';

const AORTOILIAC_TARGET_Y_MM = -290;
const AORTOILIAC_SEARCH_HALF_RANGE_MM = 20;
const MIN_AORTOILIAC_RADIUS_MM = 6;
const AORTA_LABEL_DISTANCE_MM = 48;
const ILIAC_LABEL_DISTANCE_MM = 45;
const INTERNAL_ILIAC_LABEL_DISTANCE_MM = 30;
const BRACHIOCEPHALIC_LABEL_DISTANCE_MM = 35;
const LEFT_COMMON_CAROTID_LABEL_DISTANCE_MM = 65;
const LEFT_SUBCLAVIAN_LABEL_DISTANCE_MM = 60;
const MIN_AORTIC_ARCH_BRANCH_RADIUS_MM = 2.5;
const AORTIC_CALIBER_CONNECTOR_RATIO = 0.78;
const SUPRA_AORTIC_SIDE_PROBE_DISTANCE_MM = 20;

function findAortoiliacParent(flowNetwork) {
    const candidates = flowNetwork?.edges?.filter(edge =>
        Math.abs(edge.end.y - AORTOILIAC_TARGET_Y_MM) <=
            AORTOILIAC_SEARCH_HALF_RANGE_MM &&
        edge.radiusEnd > MIN_AORTOILIAC_RADIUS_MM &&
        edge.childEdgeIndices.length === 2 &&
        edge.childEdgeIndices.every(childIndex =>
            flowNetwork.edges[childIndex]?.radiusStart >
                MIN_AORTOILIAC_RADIUS_MM
        )
    ) || [];

    candidates.sort((a, b) =>
        Math.abs(a.end.y - AORTOILIAC_TARGET_Y_MM) -
        Math.abs(b.end.y - AORTOILIAC_TARGET_Y_MM)
    );
    return candidates[0] || null;
}

function sampleUpstream(flowNetwork, startEdge, distanceMm) {
    let edge = startEdge;
    let remaining = Math.max(0, distanceMm);

    while (edge) {
        if (remaining <= edge.length) {
            return edge.end.clone().addScaledVector(edge.axis, -remaining);
        }
        remaining -= edge.length;
        if (edge.parentEdgeIndex < 0) return edge.start.clone();
        edge = flowNetwork.edges[edge.parentEdgeIndex];
    }

    return startEdge.start.clone();
}

function mainContinuation(flowNetwork, edge) {
    if (!edge.childEdgeIndices.length) return null;
    return edge.childEdgeIndices
        .map(index => flowNetwork.edges[index])
        .filter(Boolean)
        .sort((a, b) =>
            (b.radiusStart + b.radiusEnd) -
            (a.radiusStart + a.radiusEnd)
        )[0] || null;
}

function firstDownstreamSplit(flowNetwork, startEdge) {
    let edge = startEdge;
    const visited = new Set();
    while (edge && !visited.has(edge.index)) {
        visited.add(edge.index);
        if (edge.childEdgeIndices.length >= 2) return edge;
        if (edge.childEdgeIndices.length !== 1) return null;
        edge = flowNetwork.edges[edge.childEdgeIndices[0]];
    }
    return null;
}

function internalIliacLocation(flowNetwork, commonIliacRoot) {
    const split = firstDownstreamSplit(flowNetwork, commonIliacRoot);
    if (!split) return null;
    const children = split.childEdgeIndices
        .map(index => flowNetwork.edges[index])
        .filter(Boolean)
        .sort((a, b) => a.meanFlowMm3PerS - b.meanFlowMm3PerS);
    if (children.length < 2) return null;

    // At the common-iliac bifurcation the external iliac is the dominant
    // continuation toward the lower limb. The lower-flow pelvic branch is
    // the internal iliac. This uses the resolved flow tree rather than an
    // asset-specific edge number or a screen-space coordinate.
    const internalRoot = children[0];
    return {
        rootEdgeIndex: internalRoot.index,
        ...sampleDownstreamLocation(
            flowNetwork,
            internalRoot,
            INTERNAL_ILIAC_LABEL_DISTANCE_MM
        )
    };
}

function sampleDownstreamLocation(flowNetwork, startEdge, distanceMm) {
    let edge = startEdge;
    let remaining = Math.max(0, distanceMm);

    while (edge) {
        if (remaining <= edge.length) {
            return {
                edge,
                point: edge.start.clone().addScaledVector(edge.axis, remaining)
            };
        }
        remaining -= edge.length;
        const next = mainContinuation(flowNetwork, edge);
        if (!next) return { edge, point: edge.end.clone() };
        edge = next;
    }

    return { edge: startEdge, point: startEdge.end.clone() };
}

function sampleDownstream(flowNetwork, startEdge, distanceMm) {
    return sampleDownstreamLocation(
        flowNetwork,
        startEdge,
        distanceMm
    ).point;
}

function distanceBeforeFirstSplit(flowNetwork, startEdge, preferredDistanceMm) {
    let edge = startEdge;
    let distanceToSplit = 0;
    while (edge) {
        distanceToSplit += edge.length;
        if (edge.childEdgeIndices.length !== 1) break;
        edge = flowNetwork.edges[edge.childEdgeIndices[0]];
    }
    if (!edge || edge.childEdgeIndices.length < 2) {
        return preferredDistanceMm;
    }

    const splitClearance = Math.min(4, distanceToSplit * 0.25);
    return Math.min(
        preferredDistanceMm,
        Math.max(0, distanceToSplit - splitClearance)
    );
}

function isAorticCaliberConnector(flowNetwork, edge) {
    if (edge.childEdgeIndices.length !== 2 || edge.parentEdgeIndex < 0) {
        return false;
    }
    const parent = flowNetwork.edges[edge.parentEdgeIndex];
    if (!parent) return false;
    const parentCaliber = Math.max(
        parent.rawRadiusStart ?? parent.radiusStart,
        parent.rawRadiusEnd ?? parent.radiusEnd
    );
    const edgeCaliber = Math.max(
        edge.rawRadiusStart ?? edge.radiusStart,
        edge.rawRadiusEnd ?? edge.radiusEnd
    );
    return edgeCaliber >= parentCaliber * AORTIC_CALIBER_CONNECTOR_RATIO;
}

function aorticPathFromRoot(flowNetwork, distalEdge) {
    const path = [];
    let edge = distalEdge;
    while (edge) {
        path.push(edge);
        edge = edge.parentEdgeIndex >= 0
            ? flowNetwork.edges[edge.parentEdgeIndex]
            : null;
    }
    path.reverse();
    return path;
}

function diagnosticSupraAorticRoots(flowNetwork) {
    const prefixPaths = flowNetwork.getTopologyDiagnostics?.()
        ?.aorticBranchPrefixes?.paths
        ?.filter(path =>
            path.branchRadiusMm >= MIN_AORTIC_ARCH_BRANCH_RADIUS_MM
        );
    if (prefixPaths?.length !== 3) return null;

    const prefixRoots = prefixPaths
        .map(path => flowNetwork.edges[path.rootEdgeIndex])
        .filter(Boolean);
    if (prefixRoots.length !== 3) return null;

    const siblingsByParent = new Map();
    for (const root of prefixRoots) {
        const siblings = siblingsByParent.get(root.parentEdgeIndex) || [];
        siblings.push(root);
        siblingsByParent.set(root.parentEdgeIndex, siblings);
    }
    const sharedRoots = [...siblingsByParent.values()].find(
        roots => roots.length === 2
    );
    if (!sharedRoots) return null;

    const separateRoot = prefixRoots.find(
        root => !sharedRoots.includes(root)
    );
    if (!separateRoot) return null;

    const sharedPaths = sharedRoots
        .map(edge => ({
            edge,
            probe: sampleDownstream(
                flowNetwork,
                edge,
                SUPRA_AORTIC_SIDE_PROBE_DISTANCE_MM
            )
        }))
        .sort((a, b) => a.probe.x - b.probe.x);
    return {
        brachiocephalic: sharedPaths[0].edge,
        leftCommonCarotid: sharedPaths[1].edge,
        leftSubclavian: separateRoot
    };
}

/**
 * Finds the three supra-aortic vessels without relying on asset-specific edge
 * numbers. The extracted centerline contains a short, aorta-caliber connector
 * shared by the brachiocephalic and left common carotid paths. Treating that
 * connector as a named vessel puts every marker on the arch. We instead split
 * its two outgoing paths by patient-side X, then use the next side branch of
 * the aortic path for the left subclavian artery.
 */
export function findAorticArchDebugAnchors(flowNetwork) {
    const aortoiliacParent = findAortoiliacParent(flowNetwork);
    if (!aortoiliacParent) return null;

    const aorticPath = aorticPathFromRoot(
        flowNetwork,
        aortoiliacParent
    );
    const aorticPathEdgeIndices = new Set(
        aorticPath.map(edge => edge.index)
    );
    const branchRoots = [];
    for (const edge of aorticPath) {
        for (const childIndex of edge.childEdgeIndices) {
            if (aorticPathEdgeIndices.has(childIndex)) continue;
            const child = flowNetwork.edges[childIndex];
            if (!child) continue;
            if (
                Math.max(child.radiusStart, child.radiusEnd) <
                MIN_AORTIC_ARCH_BRANCH_RADIUS_MM
            ) continue;
            branchRoots.push(child);
            if (branchRoots.length === 3) break;
        }
        if (branchRoots.length === 3) break;
    }
    if (branchRoots.length < 2) return null;

    let {
        brachiocephalic,
        leftCommonCarotid,
        leftSubclavian
    } = diagnosticSupraAorticRoots(flowNetwork) || {};
    const sharedConnector = branchRoots[0];
    if (
        !brachiocephalic &&
        isAorticCaliberConnector(flowNetwork, sharedConnector)
    ) {
        const sharedPaths = sharedConnector.childEdgeIndices
            .map(index => flowNetwork.edges[index])
            .filter(Boolean)
            .map(edge => ({
                edge,
                probe: sampleDownstream(
                    flowNetwork,
                    edge,
                    SUPRA_AORTIC_SIDE_PROBE_DISTANCE_MM
                )
            }))
            .sort((a, b) => a.probe.x - b.probe.x);
        if (sharedPaths.length !== 2) return null;
        brachiocephalic = sharedPaths[0].edge;
        leftCommonCarotid = sharedPaths[1].edge;
        leftSubclavian = branchRoots[1];
    } else if (!brachiocephalic) {
        if (branchRoots.length !== 3) return null;
        [brachiocephalic, leftCommonCarotid, leftSubclavian] = branchRoots;
    }

    const brachiocephalicLocation = sampleDownstreamLocation(
        flowNetwork,
        brachiocephalic,
        distanceBeforeFirstSplit(
            flowNetwork,
            brachiocephalic,
            BRACHIOCEPHALIC_LABEL_DISTANCE_MM
        )
    );
    const leftCommonCarotidLocation = sampleDownstreamLocation(
        flowNetwork,
        leftCommonCarotid,
        LEFT_COMMON_CAROTID_LABEL_DISTANCE_MM
    );
    const leftSubclavianLocation = sampleDownstreamLocation(
        flowNetwork,
        leftSubclavian,
        LEFT_SUBCLAVIAN_LABEL_DISTANCE_MM
    );
    return {
        brachiocephalicTrunk: brachiocephalicLocation.point,
        brachiocephalicTrunkEdgeIndex: brachiocephalicLocation.edge.index,
        brachiocephalicTrunkRootEdgeIndex: brachiocephalic.index,
        leftCommonCarotid: leftCommonCarotidLocation.point,
        leftCommonCarotidEdgeIndex: leftCommonCarotidLocation.edge.index,
        leftCommonCarotidRootEdgeIndex: leftCommonCarotid.index,
        leftSubclavian: leftSubclavianLocation.point,
        leftSubclavianEdgeIndex: leftSubclavianLocation.edge.index,
        leftSubclavianRootEdgeIndex: leftSubclavian.index
    };
}

/**
 * Locates the main aortoiliac anatomy from the actual directed flow tree.
 * In the AP radiological view, patient-right is displayed on screen-left.
 * The imported anatomy therefore maps patient-right to -X and patient-left
 * to +X, independently of the legacy procedural metadata in vesselGeometry.
 */
export function findAortoiliacDebugAnchors(flowNetwork) {
    const parent = findAortoiliacParent(flowNetwork);
    if (!parent) return null;

    const iliacAnchors = parent.childEdgeIndices.map(edgeIndex => {
        const edge = flowNetwork.edges[edgeIndex];
        return {
            edgeIndex,
            point: sampleDownstream(
                flowNetwork,
                edge,
                ILIAC_LABEL_DISTANCE_MM
            )
        };
    });
    if (iliacAnchors.length !== 2) return null;

    const branchX = parent.end.x;
    iliacAnchors.sort(
        (a, b) => (a.point.x - branchX) - (b.point.x - branchX)
    );
    const rightInternalIliac = internalIliacLocation(
        flowNetwork,
        flowNetwork.edges[iliacAnchors[0].edgeIndex]
    );

    return {
        aortoiliacParentEdgeIndex: parent.index,
        aorta: sampleUpstream(
            flowNetwork,
            parent,
            AORTA_LABEL_DISTANCE_MM
        ),
        leftIliac: iliacAnchors[1].point,
        leftIliacEdgeIndex: iliacAnchors[1].edgeIndex,
        rightIliac: iliacAnchors[0].point,
        rightIliacEdgeIndex: iliacAnchors[0].edgeIndex,
        rightInternalIliac: rightInternalIliac?.point || null,
        rightInternalIliacEdgeIndex:
            rightInternalIliac?.edge?.index ?? -1,
        rightInternalIliacRootEdgeIndex:
            rightInternalIliac?.rootEdgeIndex ?? -1
    };
}

function createLabelElement({
    text,
    className,
    offsetX,
    offsetY
}) {
    const root = document.createElement('div');
    root.className = `anatomy-vessel-label-anchor ${className}`;
    root.style.setProperty('--anatomy-label-x', String(offsetX));
    root.style.setProperty('--anatomy-label-y', String(offsetY));

    const leaderLength = Math.hypot(offsetX, offsetY);
    const leaderAngle = Math.atan2(offsetY, offsetX) * 180 / Math.PI;
    root.style.setProperty('--anatomy-leader-length', `${leaderLength}px`);
    root.style.setProperty('--anatomy-leader-angle', `${leaderAngle}deg`);

    const leader = document.createElement('span');
    leader.className = 'anatomy-vessel-label-leader';
    leader.setAttribute('aria-hidden', 'true');

    const marker = document.createElement('span');
    marker.className = 'anatomy-vessel-label-marker';
    marker.setAttribute('aria-hidden', 'true');

    const label = document.createElement('span');
    label.className = 'anatomy-vessel-label';
    label.textContent = text;

    root.append(leader, marker, label);
    return root;
}

function addLabel(group, point, options) {
    const object = new CSS2DObject(createLabelElement(options));
    object.position.copy(point);
    object.name = options.className;
    group.add(object);
}

export function createAortoiliacDebugLabelGroup(flowNetwork) {
    const group = new THREE.Group();
    group.name = 'vascular-anatomy-labels';
    const anchors = findAortoiliacDebugAnchors(flowNetwork);
    const archAnchors = findAorticArchDebugAnchors(flowNetwork);
    if (!anchors && !archAnchors) {
        group.userData.anatomyLabelsAvailable = false;
        return group;
    }

    if (anchors) {
        addLabel(group, anchors.aorta, {
            text: 'AORTA',
            className: 'anatomy-vessel-label-anchor--aorta',
            offsetX: 82,
            offsetY: -24
        });
        addLabel(group, anchors.leftIliac, {
            text: 'LEWA TĘTNICA BIODROWA',
            className: 'anatomy-vessel-label-anchor--left',
            offsetX: 132,
            offsetY: -12
        });
        addLabel(group, anchors.rightIliac, {
            text: 'PRAWA TĘTNICA BIODROWA',
            className: 'anatomy-vessel-label-anchor--right',
            offsetX: -132,
            offsetY: -12
        });
        if (anchors.rightInternalIliac) {
            addLabel(group, anchors.rightInternalIliac, {
                text: 'PRAWA TĘTNICA BIODROWA WEWNĘTRZNA',
                className: 'anatomy-vessel-label-anchor--right-internal',
                offsetX: -186,
                offsetY: 56
            });
        }
    }

    if (archAnchors) {
        addLabel(group, archAnchors.brachiocephalicTrunk, {
            text: 'PIEŃ RAMIENNO-GŁOWOWY',
            className: 'anatomy-vessel-label-anchor--brachiocephalic',
            offsetX: -174,
            offsetY: 54
        });
        addLabel(group, archAnchors.leftCommonCarotid, {
            text: 'LEWA TĘTNICA SZYJNA WSPÓLNA',
            className: 'anatomy-vessel-label-anchor--left-carotid',
            offsetX: -52,
            offsetY: -76
        });
        addLabel(group, archAnchors.leftSubclavian, {
            text: 'LEWA TĘTNICA PODOBOJCZYKOWA',
            className: 'anatomy-vessel-label-anchor--left-subclavian',
            offsetX: 132,
            offsetY: 48
        });
    }

    group.userData.anatomyLabelsAvailable = true;
    group.userData.aortoiliacAnchors = anchors;
    group.userData.aorticArchAnchors = archAnchors;
    return group;
}
