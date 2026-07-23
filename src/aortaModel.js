import * as THREE from 'three';
import { STLLoader } from 'three/examples/jsm/loaders/STLLoader.js';
import { MeshBVH } from 'three-mesh-bvh';
import {
    AORTA_COLLISION_URL,
    AORTA_MODEL_URL,
    transformAortaGeometry
} from './aortaTransform.js';
import { decodeCollisionAsset } from './physics/collision/collisionAssetFormat.js';
import { createContactResult, VesselContactField } from './physics/collision/vesselContactField.js?v=20260721contactband7';
import { GUIDEWIRE_RADIUS_MM } from './toolDimensions.js';

function bufferHash(buffer) {
    return globalThis.crypto.subtle.digest('SHA-256', buffer).then(digest => {
        return [...new Uint8Array(digest)]
            .map(value => value.toString(16).padStart(2, '0'))
            .join('');
    });
}

async function fetchArrayBuffer(url) {
    const response = await fetch(url);
    if (!response.ok) throw new Error(`Failed to load ${url}: ${response.status} ${response.statusText}`);
    return response.arrayBuffer();
}

function validateCollisionAsset(asset, sourceHash, transform) {
    if (asset.metadata.source?.stlSha256 !== sourceHash) {
        throw new Error('Aorta collision asset does not match Aorta_plain.stl; run npm run collision:build');
    }
    const expected = asset.metadata.transform;
    if (
        expected?.version !== transform.version ||
        Math.abs((expected?.scale ?? Infinity) - transform.scale) > 1e-7 ||
        Math.abs((expected?.targetLength ?? Infinity) - transform.targetLength) > 1e-6
    ) {
        throw new Error('Aorta collision asset transform is stale; run npm run collision:build');
    }
}

function createAssetCenterlineBroadPhase(contactField) {
    const data = contactField.arrays.centerlineSegments;
    const edges = contactField.arrays.centerlineEdges;
    const stride = contactField.metadata.centerline.stride;
    const segments = [];
    for (let index = 0; index < data.length / stride; index++) {
        const offset = index * stride;
        const start = new THREE.Vector3(data[offset], data[offset + 1], data[offset + 2]);
        const end = new THREE.Vector3(data[offset + 3], data[offset + 4], data[offset + 5]);
        const axis = end.clone().sub(start);
        const length = axis.length();
        if (length > 1e-8) axis.multiplyScalar(1 / length);
        else axis.set(0, 1, 0);
        segments.push({
            id: index,
            start,
            end,
            axis,
            length,
            radiusStart: data[offset + 6],
            radiusEnd: data[offset + 7],
            safeRadius: data[offset + 8],
            nodeStartId: edges[index * 2],
            nodeEndId: edges[index * 2 + 1],
            source: 'medial-slice-teasar',
            aabb: null
        });
    }
    return {
        type: 'centerline-capsule-broadphase',
        source: 'medial-slice-teasar',
        diagnostics: contactField.metadata.centerline.diagnostics,
        inflation: contactField.metadata.sdf.band,
        cellSize: contactField.metadata.broadPhase.cellSize,
        segments,
        contactField
    };
}

function createPackedLumenDebugSegments(contactField, maxEdges = 12000) {
    const arrays = contactField.arrays;
    if (!arrays.lumenSliceYs?.length) return new Float32Array();
    const pointQuantization = arrays.lumenPoints instanceof Int16Array
        ? contactField.metadata.lumen?.pointQuantization || 0.02
        : 1;
    const axisBases = arrays.lumenAxisBases || new Float32Array([
        1, 0, 0,
        0, 1, 0,
        0, 0, 1
    ]);
    const axisSliceOffsets = arrays.lumenAxisSliceOffsets || new Uint32Array([
        0,
        arrays.lumenSliceYs.length
    ]);
    const totalEdges = arrays.lumenPoints.length / 2;
    const step = Math.max(1, Math.ceil(totalEdges / maxEdges));
    const positions = [];
    let edgeIndex = 0;
    for (let axisIndex = 0; axisIndex < axisSliceOffsets.length - 1; axisIndex++) {
        const basisOffset = axisIndex * 9;
        for (
            let sliceIndex = axisSliceOffsets[axisIndex];
            sliceIndex < axisSliceOffsets[axisIndex + 1];
            sliceIndex++
        ) {
            const localY = arrays.lumenSliceYs[sliceIndex];
            const contourStart = arrays.lumenSliceContourOffsets[sliceIndex];
            const contourEnd = arrays.lumenSliceContourOffsets[sliceIndex + 1];
            for (let contourIndex = contourStart; contourIndex < contourEnd; contourIndex++) {
                const pointStart = arrays.lumenContourPointOffsets[contourIndex];
                const pointEnd = arrays.lumenContourPointOffsets[contourIndex + 1];
                for (let pointIndex = pointStart; pointIndex < pointEnd; pointIndex++, edgeIndex++) {
                    if (edgeIndex % step !== 0) continue;
                    const nextIndex = pointIndex + 1 < pointEnd ? pointIndex + 1 : pointStart;
                    const ax = arrays.lumenPoints[pointIndex * 2] * pointQuantization;
                    const az = arrays.lumenPoints[pointIndex * 2 + 1] * pointQuantization;
                    const bx = arrays.lumenPoints[nextIndex * 2] * pointQuantization;
                    const bz = arrays.lumenPoints[nextIndex * 2 + 1] * pointQuantization;
                    positions.push(
                        axisBases[basisOffset] * ax + axisBases[basisOffset + 3] * localY + axisBases[basisOffset + 6] * az,
                        axisBases[basisOffset + 1] * ax + axisBases[basisOffset + 4] * localY + axisBases[basisOffset + 7] * az,
                        axisBases[basisOffset + 2] * ax + axisBases[basisOffset + 5] * localY + axisBases[basisOffset + 8] * az,
                        axisBases[basisOffset] * bx + axisBases[basisOffset + 3] * localY + axisBases[basisOffset + 6] * bz,
                        axisBases[basisOffset + 1] * bx + axisBases[basisOffset + 4] * localY + axisBases[basisOffset + 7] * bz,
                        axisBases[basisOffset + 2] * bx + axisBases[basisOffset + 5] * localY + axisBases[basisOffset + 8] * bz
                    );
                }
            }
        }
    }
    return new Float32Array(positions);
}

function createAssetPreprocessing(contactField, geometry, transform) {
    const metadata = contactField.metadata;
    return {
        geometry,
        interiorSamples: [],
        lumenSlices: [],
        lumenField: contactField.packedLumenField,
        boundaryDebugSegments: new Float32Array(),
        lumenContourDebugSegments: createPackedLumenDebugSegments(contactField),
        centerlineSliceDebugSegments: null,
        centerlineExtraction: metadata.centerline.diagnostics,
        lumenCastGeometry: null,
        lumenCast: null,
        collisionAsset: metadata,
        diagnostics: {
            boundingBox: geometry.boundingBox.clone(),
            boundaryEdgeCount: 0,
            degenerateTriangleCount: 0,
            edgeCount: 0,
            interiorSampleCount: 0,
            lumenSliceCount: metadata.lumen.sliceCount,
            nonManifoldEdgeCount: 0,
            size: geometry.boundingBox.getSize(new THREE.Vector3()),
            transform,
            triangleCount: metadata.source.triangleCount,
            vertexCount: geometry.attributes.position.count,
            source: 'precompiled-collision-asset'
        }
    };
}

export function createAortaModel(vessel, { onLoaded, onError } = {}) {
    const group = new THREE.Group();
    group.visible = false;

    const material = new THREE.MeshBasicMaterial({
        color: 0x4f8dff,
        transparent: true,
        opacity: 0.34,
        depthWrite: false,
        side: THREE.DoubleSide
    });

    Promise.all([
        fetchArrayBuffer(AORTA_MODEL_URL),
        fetchArrayBuffer(AORTA_COLLISION_URL)
    ]).then(async ([sourceBuffer, collisionBuffer]) => {
            const [sourceHash] = await Promise.all([bufferHash(sourceBuffer)]);
            const geometry = new STLLoader().parse(sourceBuffer);
            const transform = transformAortaGeometry(geometry, vessel);
            const asset = decodeCollisionAsset(collisionBuffer);
            validateCollisionAsset(asset, sourceHash, transform);
            geometry.computeVertexNormals();
            geometry.computeBoundingSphere();
            geometry.boundsTree = new MeshBVH(geometry);
            const contactField = new VesselContactField(asset, {
                fallbackGeometry: geometry,
                bvhValidationDistance: 0.02,
                capsuleBvhValidation: -0.1
            });
            const centerlineBroadPhase = createAssetCenterlineBroadPhase(contactField);
            const preprocessing = createAssetPreprocessing(contactField, geometry, transform);
            const mesh = new THREE.Mesh(geometry, material);
            mesh.renderOrder = 0;
            group.add(mesh);
            group.visible = true;
            const collision = {
                geometry,
                contactField,
                meshCollider: createVesselContactColliderAdapter(contactField),
                centerlineBroadPhase,
                clearance: 0.6,
                guidewireClearance: GUIDEWIRE_RADIUS_MM,
                guidewireSegmentClearance: 0.12,
                guidewireCollisionPasses: 3,
                guidewireSegmentSamples: [0.2, 0.4, 0.6, 0.8],
                openOutletY: geometry.boundingBox.max.y - 1,
                preprocessing
            };

            if (typeof onLoaded === 'function') {
                onLoaded({ group, mesh, geometry, collision, preprocessing, scale: transform.scale });
            }
        }).catch(error => {
            console.warn('Failed to load aorta STL model', error);
            if (typeof onError === 'function') onError(error);
        });

    return { group, material };
}

export function createVesselContactColliderAdapter(contactField) {
    const pointScratch = createContactResult();
    const sweepScratch = createContactResult();
    const setPoint = (target, x, y, z) => {
        if (typeof target?.set === 'function') target.set(x, y, z);
        else {
            target.x = x;
            target.y = y;
            target.z = z;
        }
        return target;
    };
    const copyPointContact = (contact, out) => {
        const result = out || {
            target: new THREE.Vector3(),
            closestPoint: new THREE.Vector3(),
            inward: new THREE.Vector3(),
            normal: new THREE.Vector3()
        };
        result.inside = contact.inside;
        result.violation = contact.violation;
        result.distance = Math.max(0, contact.signedDistance);
        result.signedDistance = contact.signedDistance;
        result.signedGap = contact.signedGap;
        result.penetration = contact.penetration;
        result.branchId = contact.branchId;
        result.source = contact.source;
        result.target = result.target || {};
        result.closestPoint = result.closestPoint || {};
        result.inward = result.inward || {};
        result.normal = result.normal || {};
        setPoint(result.target, contact.target.x, contact.target.y, contact.target.z);
        setPoint(result.closestPoint, contact.closestPoint.x, contact.closestPoint.y, contact.closestPoint.z);
        setPoint(result.inward, contact.inward.x, contact.inward.y, contact.inward.z);
        setPoint(result.normal, -contact.inward.x, -contact.inward.y, -contact.inward.z);
        return result;
    };
    const pointContact = (input, clearance = 0, out = null) => {
        return copyPointContact(contactField.querySphere(input, clearance, pointScratch), out);
    };
    const crossingContact = (from, to, clearance = 0) => {
        const contact = contactField.sweepSphere(from, to, clearance, sweepScratch);
        if (!contact.violation && contact.timeOfImpact >= 1) return null;
        return {
            penetration: contact.penetration,
            point: new THREE.Vector3(contact.point.x, contact.point.y, contact.point.z),
            target: new THREE.Vector3(contact.target.x, contact.target.y, contact.target.z),
            normal: new THREE.Vector3(-contact.inward.x, -contact.inward.y, -contact.inward.z),
            inward: new THREE.Vector3(contact.inward.x, contact.inward.y, contact.inward.z),
            t: contact.timeOfImpact,
            branchId: contact.branchId
        };
    };
    return {
        geometry: contactField.fallbackGeometry,
        lumenField: contactField.packedLumenField,
        broadPhase: contactField,
        contactField,
        containsPoint: input => !pointContact(input, 0, pointScratch).violation,
        pointContact,
        crossingContact,
        clearCache: () => {}
    };
}

export function createMeshLumenCollider(geometry, { lumenField = null, broadPhase = null } = {}) {
    const setPoint = (target, x, y, z) => {
        if (typeof target?.set === 'function') target.set(x, y, z);
        else {
            target.x = x;
            target.y = y;
            target.z = z;
        }
        return target;
    };
    const finiteComponent = (value, fallback) => Number.isFinite(value) ? value : fallback;
    const setQueryTargetAtClearance = (target, input, query, clearance) => {
        const signedDistance = query?.signedDistance;
        const correction = Number.isFinite(signedDistance)
            ? Math.max(0, clearance - signedDistance)
            : Math.max(0, clearance);
        const inward = query?.inward;
        const inwardX = finiteComponent(inward?.x, 1);
        const inwardY = finiteComponent(inward?.y, 0);
        const inwardZ = finiteComponent(inward?.z, 0);
        return setPoint(
            target,
            input.x + inwardX * correction,
            input.y + inwardY * correction,
            input.z + inwardZ * correction
        );
    };

    function pointContact(input, clearance = 0, out = null) {
        const query = lumenField?.query?.(input, out?.query);
        if (!query) {
            if (out) {
                out.inside = true;
                out.violation = false;
                out.distance = Infinity;
                out.signedDistance = Infinity;
                out.target = setPoint(out.target || (out.target = {}), input.x, input.y, input.z);
                out.normal = setPoint(out.normal || (out.normal = {}), 1, 0, 0);
                return out;
            }
            return {
                inside: true,
                violation: false,
                distance: Infinity,
                signedDistance: Infinity,
                target: new THREE.Vector3(input.x, input.y, input.z),
                normal: new THREE.Vector3(1, 0, 0)
            };
        }
        const signedDistance = Number.isFinite(query.signedDistance) ? query.signedDistance : -Infinity;
        const violation = signedDistance < clearance;
        if (out) {
            out.inside = query.inside;
            out.violation = violation;
            out.distance = Math.max(0, signedDistance);
            out.signedDistance = signedDistance;
            out.target = out.target || {};
            if (violation) {
                setQueryTargetAtClearance(out.target, input, query, clearance);
            } else {
                setPoint(out.target, input.x, input.y, input.z);
            }
            out.closestPoint = setPoint(out.closestPoint || (out.closestPoint = {}), query.closestPoint.x, query.closestPoint.y, query.closestPoint.z);
            out.inward = setPoint(out.inward || (out.inward = {}), query.inward.x, query.inward.y, query.inward.z);
            out.normal = setPoint(out.normal || (out.normal = {}), query.normal.x, query.normal.y, query.normal.z);
            return out;
        }
        return {
            inside: query.inside,
            violation,
            distance: Math.max(0, signedDistance),
            signedDistance,
            target: violation
                ? (query.targetAtClearance?.(clearance) || setQueryTargetAtClearance(new THREE.Vector3(), input, query, clearance))
                : new THREE.Vector3(input.x, input.y, input.z),
            closestPoint: query.closestPoint.clone(),
            inward: query.inward.clone(),
            normal: query.normal.clone()
        };
    }

    function crossingContact(fromInput, toInput, clearance = 0) {
        const from = new THREE.Vector3(fromInput.x, fromInput.y, fromInput.z);
        const to = new THREE.Vector3(toInput.x, toInput.y, toInput.z);
        const delta = to.clone().sub(from);
        const length = delta.length();
        if (length < 1e-6) return null;

        let worst = null;
        const sampleCount = Math.max(5, Math.ceil(length / 3));
        for (let i = 1; i < sampleCount; i++) {
            const t = i / sampleCount;
            const point = from.clone().addScaledVector(delta, t);
            const contact = pointContact(point, clearance);
            if (!contact.violation) continue;
            const penetration = clearance - contact.signedDistance;
            if (!worst || penetration > worst.penetration) {
                worst = {
                    penetration,
                    point,
                    target: contact.target,
                    normal: contact.normal,
                    inward: contact.inward,
                    t
                };
            }
        }
        return worst;
    }

    return {
        geometry,
        lumenField,
        broadPhase,
        containsPoint: input => !pointContact(input, 0).violation,
        pointContact,
        crossingContact,
        clearCache: () => {}
    };
}
