import * as THREE from 'three';
import { STLLoader } from 'three/examples/jsm/loaders/STLLoader.js';
import { MeshBVH } from 'three-mesh-bvh';

const AORTA_MODEL_URL = 'res/Aorta_plain.stl';
const AORTA_MODEL_SCALE = 1.3;
const AORTA_MODEL_Y_OFFSET = 40;

function targetFromVessel(vessel) {
    const ys = [];
    for (const seg of vessel?.segments || []) {
        if (seg.isSheath) continue;
        ys.push(seg.start.y, seg.end.y);
    }

    const top = Math.max(...ys, 0) + 15;
    const bottom = Math.min(...ys, -420) - 15;
    return {
        center: new THREE.Vector3(
            vessel?.branchPoint?.x || 0,
            (top + bottom) * 0.5 + AORTA_MODEL_Y_OFFSET,
            vessel?.branchPoint?.z || 0
        ),
        length: Math.max(300, top - bottom)
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

    const loader = new STLLoader();
    loader.load(
        AORTA_MODEL_URL,
        geometry => {
            geometry.computeBoundingBox();
            const sourceBox = geometry.boundingBox;
            const sourceSize = sourceBox.getSize(new THREE.Vector3());
            const sourceCenter = sourceBox.getCenter(new THREE.Vector3());
            const target = targetFromVessel(vessel);
            const scale = target.length * AORTA_MODEL_SCALE / Math.max(1e-6, sourceSize.z);

            geometry.translate(-sourceCenter.x, -sourceCenter.y, -sourceCenter.z);
            geometry.rotateX(-Math.PI / 2);
            geometry.scale(scale, scale, scale);
            geometry.translate(target.center.x, target.center.y, target.center.z);
            geometry.computeVertexNormals();
            geometry.computeBoundingBox();
            geometry.boundsTree = new MeshBVH(geometry);

            const mesh = new THREE.Mesh(geometry, material);
            mesh.renderOrder = 0;
            group.add(mesh);
            group.visible = true;
            const collision = {
                geometry,
                meshCollider: createMeshLumenCollider(geometry),
                clearance: 0.6,
                guidewireClearance: 0.35,
                guidewireSegmentClearance: 0.12,
                guidewireCollisionPasses: 3,
                guidewireSegmentSamples: [0.2, 0.4, 0.6, 0.8],
                openOutletY: geometry.boundingBox.max.y - 1
            };

            if (typeof onLoaded === 'function') {
                onLoaded({ group, mesh, geometry, collision, scale });
            }
        },
        undefined,
        error => {
            console.warn('Failed to load aorta STL model', error);
            if (typeof onError === 'function') onError(error);
        }
    );

    return { group, material };
}

export function createMeshLumenCollider(geometry) {
    const closest = new THREE.Vector3();
    const point = new THREE.Vector3();
    const ray = new THREE.Ray();
    const fallbackInward = new THREE.Vector3(0, 0, 1);
    const rayFar = (geometry.boundingBox?.getSize(new THREE.Vector3()).length() || 1000) * 2;
    const normalA = new THREE.Vector3();
    const normalB = new THREE.Vector3();
    const normalC = new THREE.Vector3();
    const faceNormal = new THREE.Vector3();
    const pointDelta = new THREE.Vector3();

    function triangleNormal(faceIndex) {
        const position = geometry.attributes.position;
        const base = faceIndex * 3;
        normalA.fromBufferAttribute(position, base);
        normalB.fromBufferAttribute(position, base + 1);
        normalC.fromBufferAttribute(position, base + 2);
        faceNormal.subVectors(normalB, normalA).cross(normalC.sub(normalA));
        if (faceNormal.lengthSq() < 1e-8) faceNormal.copy(fallbackInward);
        return faceNormal.normalize();
    }

    function inwardNormalForFace(faceIndex) {
        if (!Number.isFinite(faceIndex)) return fallbackInward.clone();
        return triangleNormal(faceIndex).clone().negate();
    }

    // The STL is an open vessel surface, so ray-parity containment is unstable
    // around branch ostia and large lumens. Treat the STL face normal as the
    // wall/outside direction and the opposite side as the lumen side.
    function pointContact(input, clearance = 0) {
        point.set(input.x, input.y, input.z);
        const hit = geometry.boundsTree.closestPointToPoint(point, { point: closest });
        const distance = hit?.distance ?? point.distanceTo(closest);
        const inward = inwardNormalForFace(hit?.faceIndex).clone();
        const depth = pointDelta.subVectors(point, closest).dot(inward);
        const target = closest.clone().addScaledVector(inward, Math.max(0, clearance));
        const wallNormal = inward.clone().negate();

        if (depth >= clearance) {
            return {
                inside: true,
                violation: false,
                distance,
                signedDistance: -depth,
                target: point.clone(),
                normal: wallNormal
            };
        }

        return {
            inside: depth >= 0,
            violation: true,
            distance,
            signedDistance: -depth,
            target,
            normal: wallNormal
        };
    }

    function crossingContact(fromInput, toInput, clearance = 0) {
        const from = new THREE.Vector3(fromInput.x, fromInput.y, fromInput.z);
        const to = new THREE.Vector3(toInput.x, toInput.y, toInput.z);
        const delta = to.sub(from);
        const length = delta.length();
        if (length < 1e-6) return null;

        const direction = delta.multiplyScalar(1 / length);
        ray.origin.copy(from);
        ray.direction.copy(direction);
        const far = length + Math.max(0, clearance) + 1e-3;
        const hit = typeof geometry.boundsTree.raycastFirst === 'function'
            ? geometry.boundsTree.raycastFirst(ray, THREE.DoubleSide, 1e-4, far)
            : geometry.boundsTree
                .raycast(ray, THREE.DoubleSide, 1e-4, far)
                .filter(candidate => candidate.distance > 1e-4)
                .sort((a, b) => a.distance - b.distance)[0];
        if (!hit || hit.distance > length + 1e-3) return null;

        const inward = inwardNormalForFace(hit.faceIndex);
        const target = hit.point.clone().addScaledVector(inward, Math.max(0, clearance));
        return {
            hit,
            point: hit.point.clone(),
            target,
            normal: inward.clone().negate(),
            inward,
            t: hit.distance / length
        };
    }

    return {
        geometry,
        containsPoint: input => !pointContact(input, 0).violation,
        pointContact,
        crossingContact,
        clearCache: () => {}
    };
}
