import * as THREE from 'three';
import { STLLoader } from 'three/examples/jsm/loaders/STLLoader.js';
import { preprocessAortaGeometry } from './aortaPreprocess.js?v=20260616stlpreprocess6';

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
            const preprocessing = preprocessAortaGeometry(geometry, {
                transform: {
                    rotationX: -Math.PI / 2,
                    scale,
                    sourceCenter,
                    sourceSize,
                    targetCenter: target.center,
                    targetLength: target.length
                }
            });

            const mesh = new THREE.Mesh(geometry, material);
            mesh.renderOrder = 0;
            group.add(mesh);
            group.visible = true;
            const collision = {
                geometry,
                meshCollider: createMeshLumenCollider(geometry, {
                    lumenField: preprocessing.lumenField
                }),
                clearance: 0.6,
                guidewireClearance: 0.35,
                guidewireSegmentClearance: 0.12,
                guidewireCollisionPasses: 3,
                guidewireSegmentSamples: [0.2, 0.4, 0.6, 0.8],
                openOutletY: geometry.boundingBox.max.y - 1,
                preprocessing
            };

            if (typeof onLoaded === 'function') {
                onLoaded({ group, mesh, geometry, collision, preprocessing, scale });
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

export function createMeshLumenCollider(geometry, { lumenField = null } = {}) {
    function pointContact(input, clearance = 0) {
        const query = lumenField?.query?.(input);
        if (!query) return {
            inside: true,
            violation: false,
            distance: Infinity,
            signedDistance: Infinity,
            target: new THREE.Vector3(input.x, input.y, input.z),
            normal: new THREE.Vector3(1, 0, 0)
        };
        const violation = query.signedDistance < clearance;
        return {
            inside: query.inside,
            violation,
            distance: Math.max(0, query.signedDistance),
            signedDistance: query.signedDistance,
            target: violation
                ? query.targetAtClearance(clearance)
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
        containsPoint: input => !pointContact(input, 0).violation,
        pointContact,
        crossingContact,
        clearCache: () => {}
    };
}
