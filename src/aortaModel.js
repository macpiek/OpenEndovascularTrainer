import * as THREE from 'three';
import { STLLoader } from 'three/examples/jsm/loaders/STLLoader.js';
import { MeshBVH } from 'three-mesh-bvh';

const AORTA_MODEL_URL = 'res/Aorta_plain.stl';
const AORTA_MODEL_SCALE = 1.3;
const AORTA_MODEL_Y_OFFSET = 40;
const LUMEN_HINT_SAMPLE_SPACING = 3;
const LUMEN_HINT_Y_WINDOW = 38;

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
            const lumenHintSamples = buildAortaLumenHintSamples(vessel);
            const collision = {
                geometry,
                meshCollider: createMeshLumenCollider(geometry, { lumenHintSamples }),
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

export function buildAortaLumenHintSamples(vessel, spacing = LUMEN_HINT_SAMPLE_SPACING) {
    const path = [
        { point: new THREE.Vector3(vessel.sheath.end.x, vessel.sheath.end.y, vessel.sheath.end.z) },
        { point: new THREE.Vector3(-71, -374, 12) },
        { point: new THREE.Vector3(-68, -365, 10) },
        { point: new THREE.Vector3(-60, -355, 2) },
        { point: new THREE.Vector3(-28, -338, -8) },
        { point: new THREE.Vector3(-10, -315, 2) },
        { point: new THREE.Vector3(-14, -290, 8) },
        { point: new THREE.Vector3(0, -230, 0) },
        { point: new THREE.Vector3(0, -160, 0) },
        { point: new THREE.Vector3(-9, -125, -6) },
        { point: new THREE.Vector3(28, -100, -33) },
        { point: new THREE.Vector3(38, -60, -49) },
        { point: new THREE.Vector3(36, -25, -56) },
        { point: new THREE.Vector3(19, 0, -22) },
        { point: new THREE.Vector3(-8, 35, -18) },
        { point: new THREE.Vector3(3, 95, -18) },
        { point: new THREE.Vector3(10, 145, -18) },
        { point: new THREE.Vector3(20, 230, -18) },
        { point: new THREE.Vector3(32, 330, -18) }
    ];
    const curve = new THREE.CatmullRomCurve3(
        path.map(entry => entry.point),
        false,
        'centripetal',
        0.35
    );
    curve.arcLengthDivisions = Math.max(200, path.length * 48);
    const curveLength = curve.getLength();
    const sampleCount = Math.max(2, Math.ceil(curveLength / Math.max(0.5, spacing)) + 1);
    const samples = [];
    for (let i = 0; i < sampleCount; i++) {
        samples.push(curve.getPointAt(i / (sampleCount - 1)));
    }
    return samples;
}

export function createMeshLumenCollider(geometry, { lumenHintSamples = [] } = {}) {
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
    const hintedInward = new THREE.Vector3();
    const orderedHints = lumenHintSamples
        .map(sample => sample.clone ? sample.clone() : new THREE.Vector3(sample.x, sample.y, sample.z))
        .sort((a, b) => a.y - b.y);

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

    function inwardDirectionForSurface(surfacePoint, faceIndex) {
        let best = null;
        let bestDistanceSq = Infinity;
        const yMin = surfacePoint.y - LUMEN_HINT_Y_WINDOW;
        const yMax = surfacePoint.y + LUMEN_HINT_Y_WINDOW;
        let candidates = 0;
        for (const sample of orderedHints) {
            if (sample.y < yMin) continue;
            if (sample.y > yMax) break;
            const distanceSq = surfacePoint.distanceToSquared(sample);
            candidates++;
            if (distanceSq < bestDistanceSq) {
                best = sample;
                bestDistanceSq = distanceSq;
            }
        }

        if (candidates === 0) {
            for (const sample of orderedHints) {
                const distanceSq = surfacePoint.distanceToSquared(sample);
                if (distanceSq < bestDistanceSq) {
                    best = sample;
                    bestDistanceSq = distanceSq;
                }
            }
        }

        if (best) {
            hintedInward.subVectors(best, surfacePoint);
            if (hintedInward.lengthSq() > 1e-8) {
                return hintedInward.normalize().clone();
            }
        }

        return inwardNormalForFace(faceIndex);
    }

    // The STL is an open vessel surface, so ray-parity containment is unstable
    // around branch ostia and raw triangle normals are not a reliable inside
    // signal. Orient each local wall normal toward a sampled lumen centerline.
    function pointContact(input, clearance = 0) {
        point.set(input.x, input.y, input.z);
        const hit = geometry.boundsTree.closestPointToPoint(point, { point: closest });
        const distance = hit?.distance ?? point.distanceTo(closest);
        const inward = inwardDirectionForSurface(closest, hit?.faceIndex);
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

        const inward = inwardDirectionForSurface(hit.point, hit.faceIndex);
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
