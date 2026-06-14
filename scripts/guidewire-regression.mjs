import fs from 'node:fs/promises';
import * as THREE from 'three';
import { STLLoader } from 'three/examples/jsm/loaders/STLLoader.js';
import { MeshBVH } from 'three-mesh-bvh';
import { generateVessel } from '../src/vesselGeometry.js';
import { ElasticRod } from '../src/physics/elasticRod.js';
import { createMeshLumenCollider } from '../src/aortaModel.js';

const AORTA_MODEL_SCALE = 1.3;
const AORTA_MODEL_Y_OFFSET = 40;
const fixedDt = 1 / 60;
const segmentLength = 5;
const nodeCount = 201;
const guidewireLength = segmentLength * (nodeCount - 1);
const GUIDEWIRE_ADVANCE_RATE = 44;
const GUIDEWIRE_FEED_VELOCITY_BLEND = 0.015;
const GUIDEWIRE_FEED_ACTIVE_LENGTH = guidewireLength;
const GUIDEWIRE_FEED_DISTAL_WEIGHT = 0.34;
const GUIDEWIRE_FEED_WALL_BAND = 3.2;
const GUIDEWIRE_BODY_BENDING_STIFFNESS = 32;
const GUIDEWIRE_TIP_BENDING_STIFFNESS = 8;
const GUIDEWIRE_TIP_FLEX_LENGTH = 105;
const GUIDEWIRE_TIP_SOFT_LENGTH = 24;
const SHEATH_EXIT_SUPPORT_LENGTH = segmentLength * 2.4;
const SHEATH_EXIT_SUPPORT_BLEND = 0.12;
const SHEATH_EXIT_VELOCITY_DAMPING = 0.28;
const GUIDEWIRE_LUMEN_GLIDE_STRENGTH = 0.86;
const GUIDEWIRE_LUMEN_GLIDE_DAMPING = 0.86;
const GUIDEWIRE_LUMEN_GLIDE_START = 0;
const GUIDEWIRE_LUMEN_GLIDE_FADE = 20;
const GUIDEWIRE_LUMEN_CENTERING = 0.035;
const GUIDEWIRE_BUCKLE_START_ANGLE = 94;
const GUIDEWIRE_BUCKLE_LIMIT_STRENGTH = 0.042;

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

async function loadCollision(vessel) {
    const bytes = await fs.readFile(new URL('../res/Aorta_plain.stl', import.meta.url));
    const arrayBuffer = bytes.buffer.slice(bytes.byteOffset, bytes.byteOffset + bytes.byteLength);
    const geometry = new STLLoader().parse(arrayBuffer);
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

    return {
        geometry,
        meshCollider: createMeshLumenCollider(geometry),
        segments: vessel.segments,
        clearance: 0.6,
        guidewireClearance: 0.35,
        guidewireSegmentClearance: 0.12,
        guidewireCollisionPasses: 3,
        guidewireSegmentSamples: [0.2, 0.4, 0.6, 0.8],
        openOutletY: geometry.boundingBox.max.y - 1
    };
}

function smoothRange(start, end, value) {
    const t = Math.max(0, Math.min(1, (value - start) / Math.max(1e-6, end - start)));
    return t * t * (3 - 2 * t);
}

function normalizeDirection(direction, fallback) {
    const length = Math.hypot(direction.x, direction.y, direction.z);
    if (length < 1e-6) return { ...fallback };
    return {
        x: direction.x / length,
        y: direction.y / length,
        z: direction.z / length
    };
}

function bendAngleAt(nodes, index) {
    const prev = nodes[index - 1];
    const curr = nodes[index];
    const next = nodes[index + 1];
    if (!prev || !curr || !next) return 0;

    const ax = curr.x - prev.x;
    const ay = curr.y - prev.y;
    const az = curr.z - prev.z;
    const bx = next.x - curr.x;
    const by = next.y - curr.y;
    const bz = next.z - curr.z;
    const aLen = Math.hypot(ax, ay, az);
    const bLen = Math.hypot(bx, by, bz);
    if (aLen < 1e-6 || bLen < 1e-6) return 0;

    const dot = (ax * bx + ay * by + az * bz) / (aLen * bLen);
    return Math.acos(Math.max(-1, Math.min(1, dot))) * 180 / Math.PI;
}

const { vessel } = generateVessel();
const collision = await loadCollision(vessel);
const sheathDirVec = {
    x: vessel.sheath.end.x - vessel.sheath.start.x,
    y: vessel.sheath.end.y - vessel.sheath.start.y,
    z: vessel.sheath.end.z - vessel.sheath.start.z
};
const sheathPath = Math.hypot(sheathDirVec.x, sheathDirVec.y, sheathDirVec.z) || 1;
const wireDir = {
    x: sheathDirVec.x / sheathPath,
    y: sheathDirVec.y / sheathPath,
    z: sheathDirVec.z / sheathPath
};
const tipStart = vessel.sheath.start;
const tailStart = {
    x: tipStart.x - wireDir.x * guidewireLength,
    y: tipStart.y - wireDir.y * guidewireLength,
    z: tipStart.z - wireDir.z * guidewireLength
};
const wire = new ElasticRod(nodeCount, segmentLength, { constraintIterations: 28 });
let tailProgress = 0;
const wireSheathPinnedState = new Array(nodeCount).fill(false);
const wireReleasedFromSheath = new Array(nodeCount).fill(false);

for (let i = 0; i < wire.nodes.length; i++) {
    const t = segmentLength * i;
    wire.nodes[i].x = tailStart.x + wireDir.x * t;
    wire.nodes[i].y = tailStart.y + wireDir.y * t;
    wire.nodes[i].z = tailStart.z + wireDir.z * t;
}

function guidewireInsertedCoordinate(index) {
    return segmentLength * index - guidewireLength + tailProgress;
}

function sheathAxisPoint(inserted) {
    return {
        x: vessel.sheath.start.x + wireDir.x * inserted,
        y: vessel.sheath.start.y + wireDir.y * inserted,
        z: vessel.sheath.start.z + wireDir.z * inserted
    };
}

const guidewireLumenPath = [
    { point: new THREE.Vector3(vessel.sheath.end.x, vessel.sheath.end.y, vessel.sheath.end.z), radius: 2.8 },
    { point: new THREE.Vector3(-71, -374, 12), radius: 3.4 },
    { point: new THREE.Vector3(-69, -365, 10), radius: 4.6 },
    { point: new THREE.Vector3(-61, -338, 7), radius: 7.2 },
    { point: new THREE.Vector3(-41, -315, 4), radius: 9.8 },
    { point: new THREE.Vector3(-10, -290, 1), radius: 13.8 },
    { point: new THREE.Vector3(0, -230, 0), radius: 17.5 },
    { point: new THREE.Vector3(0, -160, 0), radius: 18.0 },
    { point: new THREE.Vector3(-9, -125, -6), radius: 17.0 },
    { point: new THREE.Vector3(28, -100, -33), radius: 16.0 },
    { point: new THREE.Vector3(38, -60, -49), radius: 13.0 },
    { point: new THREE.Vector3(36, -25, -56), radius: 13.0 },
    { point: new THREE.Vector3(19, 0, -22), radius: 16.0 },
    { point: new THREE.Vector3(-8, 35, -18), radius: 18.0 },
    { point: new THREE.Vector3(3, 95, -18), radius: 18.0 },
    { point: new THREE.Vector3(10, 145, -18), radius: 18.0 },
    { point: new THREE.Vector3(20, 230, -18), radius: 18.0 },
    { point: new THREE.Vector3(32, 330, -18), radius: 18.0 }
];
const guidewireLumenSegments = [];
let guidewireLumenLength = 0;
for (let i = 0; i < guidewireLumenPath.length - 1; i++) {
    const start = guidewireLumenPath[i].point;
    const end = guidewireLumenPath[i + 1].point;
    const length = start.distanceTo(end);
    guidewireLumenSegments.push({
        start,
        end,
        startRadius: guidewireLumenPath[i].radius,
        endRadius: guidewireLumenPath[i + 1].radius,
        length,
        offset: guidewireLumenLength
    });
    guidewireLumenLength += length;
}
const guidewireLumenCurve = new THREE.CatmullRomCurve3(
    guidewireLumenPath.map(entry => entry.point),
    false,
    'centripetal',
    0.35
);
guidewireLumenCurve.arcLengthDivisions = Math.max(200, guidewireLumenPath.length * 48);
const guidewireLumenCurveLength = guidewireLumenCurve.getLength();
const GUIDEWIRE_LUMEN_SAMPLE_SPACING = 2.5;
const guidewireLumenSamples = [];
const guidewireLumenSampleCount = Math.max(2, Math.ceil(guidewireLumenCurveLength / GUIDEWIRE_LUMEN_SAMPLE_SPACING) + 1);
for (let i = 0; i < guidewireLumenSampleCount; i++) {
    const u = i / (guidewireLumenSampleCount - 1);
    const point = guidewireLumenCurve.getPointAt(u);
    const tangent = guidewireLumenCurve.getTangentAt(u).normalize();
    guidewireLumenSamples.push({
        distance: u * guidewireLumenCurveLength,
        x: point.x,
        y: point.y,
        z: point.z,
        tx: tangent.x,
        ty: tangent.y,
        tz: tangent.z,
        radius: sampleGuidewireLumenRadius(u * guidewireLumenLength)
    });
}

function sampleGuidewireLumenRadius(distance) {
    const d = Math.max(0, distance);
    for (const seg of guidewireLumenSegments) {
        if (d <= seg.offset + seg.length) {
            const t = Math.max(0, Math.min(1, (d - seg.offset) / Math.max(1e-6, seg.length)));
            return seg.startRadius * (1 - t) + seg.endRadius * t;
        }
    }

    const last = guidewireLumenSegments[guidewireLumenSegments.length - 1];
    return last.endRadius;
}

function sampleGuidewireLumen(distance) {
    const d = Math.max(0, distance);
    if (d <= guidewireLumenCurveLength) {
        const samplePosition = d / Math.max(1e-6, guidewireLumenCurveLength) * (guidewireLumenSamples.length - 1);
        const lowerIndex = Math.max(0, Math.min(guidewireLumenSamples.length - 2, Math.floor(samplePosition)));
        const upperIndex = lowerIndex + 1;
        const t = samplePosition - lowerIndex;
        const a = guidewireLumenSamples[lowerIndex];
        const b = guidewireLumenSamples[upperIndex];
        let tx = a.tx * (1 - t) + b.tx * t;
        let ty = a.ty * (1 - t) + b.ty * t;
        let tz = a.tz * (1 - t) + b.tz * t;
        const tangentLength = Math.hypot(tx, ty, tz) || 1;
        tx /= tangentLength;
        ty /= tangentLength;
        tz /= tangentLength;
        return {
            point: {
                x: a.x * (1 - t) + b.x * t,
                y: a.y * (1 - t) + b.y * t,
                z: a.z * (1 - t) + b.z * t
            },
            tangent: { x: tx, y: ty, z: tz },
            radius: a.radius * (1 - t) + b.radius * t
        };
    }

    const last = guidewireLumenSegments[guidewireLumenSegments.length - 1];
    const endSample = guidewireLumenSamples[guidewireLumenSamples.length - 1];
    const overrun = d - guidewireLumenCurveLength;
    return {
        point: {
            x: endSample.x + endSample.tx * overrun,
            y: endSample.y + endSample.ty * overrun,
            z: endSample.z + endSample.tz * overrun
        },
        tangent: { x: endSample.tx, y: endSample.ty, z: endSample.tz },
        radius: last.endRadius
    };
}

function constrainWireToSheath(feedSpeed = 0) {
    for (let i = 0; i < wire.nodes.length; i++) {
        const inserted = guidewireInsertedCoordinate(i);
        const inSheath = inserted <= sheathPath;
        const n = wire.nodes[i];
        const wasInSheath = wireSheathPinnedState[i];
        const released = wasInSheath && !inSheath;
        n.pinned = inSheath;
        wireReleasedFromSheath[i] = released;
        if (inSheath || released) {
            const target = sheathAxisPoint(inserted);
            n.x = target.x;
            n.y = target.y;
            n.z = target.z;
            const speed = inSheath ? 0 : Math.max(0, feedSpeed);
            n.vx = wireDir.x * speed;
            n.vy = wireDir.y * speed;
            n.vz = wireDir.z * speed;
        }
        wireSheathPinnedState[i] = inSheath;
    }
}

function guidewireTangentAt(index) {
    const prev = wire.nodes[Math.max(0, index - 1)];
    const next = wire.nodes[Math.min(wire.nodes.length - 1, index + 1)];
    let tx = next.x - prev.x;
    let ty = next.y - prev.y;
    let tz = next.z - prev.z;
    let length = Math.hypot(tx, ty, tz);
    if (length < 1e-6 && index > 0) {
        const n = wire.nodes[index];
        const p = wire.nodes[index - 1];
        tx = n.x - p.x;
        ty = n.y - p.y;
        tz = n.z - p.z;
        length = Math.hypot(tx, ty, tz);
    }
    if (length < 1e-6) return { ...wireDir };
    return { x: tx / length, y: ty / length, z: tz / length };
}

function applyGuidewireStiffnessProfile() {
    const lastIndex = wire.nodes.length - 1;
    for (let i = 0; i < wire.nodes.length; i++) {
        const distanceFromTip = (lastIndex - i) * segmentLength;
        const bodyBlend = smoothRange(GUIDEWIRE_TIP_SOFT_LENGTH, GUIDEWIRE_TIP_FLEX_LENGTH, distanceFromTip);
        wire.nodes[i].bendingStiffness =
            GUIDEWIRE_TIP_BENDING_STIFFNESS * (1 - bodyBlend) +
            GUIDEWIRE_BODY_BENDING_STIFFNESS * bodyBlend;
    }
}

function projectGuidewireFeedDirection(point, direction) {
    const contact = collision.meshCollider.pointContact(point, 0);
    if (!contact || !Number.isFinite(contact.distance)) return normalizeDirection(direction, wireDir);

    const projected = { x: direction.x, y: direction.y, z: direction.z };
    const normal = contact.normal;
    const nearWall = contact.violation || contact.distance <= GUIDEWIRE_FEED_WALL_BAND;
    if (nearWall && normal) {
        const dot = projected.x * normal.x + projected.y * normal.y + projected.z * normal.z;
        if (dot > 0) {
            projected.x -= normal.x * dot;
            projected.y -= normal.y * dot;
            projected.z -= normal.z * dot;
        }
    }

    if (contact.violation && contact.target) {
        const correction = {
            x: contact.target.x - point.x,
            y: contact.target.y - point.y,
            z: contact.target.z - point.z
        };
        const inward = normalizeDirection(correction, projected);
        projected.x = projected.x * 0.72 + inward.x * 0.28;
        projected.y = projected.y * 0.72 + inward.y * 0.28;
        projected.z = projected.z * 0.72 + inward.z * 0.28;
    }

    return normalizeDirection(projected, direction);
}

function samplePreviousGuidewirePosition(previousPositions, sourceIndex) {
    const lastIndex = previousPositions.length - 1;
    if (sourceIndex <= 0) return { ...previousPositions[0] };
    if (sourceIndex < lastIndex) {
        const lower = Math.floor(sourceIndex);
        const upper = Math.min(lastIndex, lower + 1);
        const t = sourceIndex - lower;
        const p0 = previousPositions[lower];
        const p1 = previousPositions[upper];
        return {
            x: p0.x * (1 - t) + p1.x * t,
            y: p0.y * (1 - t) + p1.y * t,
            z: p0.z * (1 - t) + p1.z * t
        };
    }

    const tip = previousPositions[lastIndex];
    const prev = previousPositions[Math.max(0, lastIndex - 1)];
    const direction = projectGuidewireFeedDirection(tip, {
        x: tip.x - prev.x,
        y: tip.y - prev.y,
        z: tip.z - prev.z
    });
    const distance = (sourceIndex - lastIndex) * segmentLength;
    return {
        x: tip.x + direction.x * distance,
        y: tip.y + direction.y * distance,
        z: tip.z + direction.z * distance
    };
}

function feedGuidewireMaterial(delta, dt, previousPositions) {
    if (Math.abs(delta) < 1e-6) return;
    if (delta > 0 && previousPositions?.length === wire.nodes.length) {
        const sourceShift = delta / segmentLength;
        const invDt = 1 / Math.max(dt, 1e-6);
        for (let i = 0; i < wire.nodes.length; i++) {
            const n = wire.nodes[i];
            if (n.pinned) continue;

            const inserted = guidewireInsertedCoordinate(i);
            if (inserted <= sheathPath) continue;

            const target = samplePreviousGuidewirePosition(previousPositions, i + sourceShift);
            const exitDistance = inserted - sheathPath;
            const sheathBlend = Math.max(0, Math.min(1, 1 - exitDistance / SHEATH_EXIT_SUPPORT_LENGTH));
            if (sheathBlend > 0) {
                const sheathTarget = sheathAxisPoint(inserted);
                const axial = {
                    x: sheathTarget.x * sheathBlend + target.x * (1 - sheathBlend),
                    y: sheathTarget.y * sheathBlend + target.y * (1 - sheathBlend),
                    z: sheathTarget.z * sheathBlend + target.z * (1 - sheathBlend)
                };
                target.x = axial.x;
                target.y = axial.y;
                target.z = axial.z;
            }

            const old = previousPositions[i];
            const moveX = target.x - n.x;
            const moveY = target.y - n.y;
            const moveZ = target.z - n.z;
            n.x = target.x;
            n.y = target.y;
            n.z = target.z;
            n.vx = n.vx * 0.18 + (target.x - old.x) * invDt * 0.06 + moveX * invDt * 0.04;
            n.vy = n.vy * 0.18 + (target.y - old.y) * invDt * 0.06 + moveY * invDt * 0.04;
            n.vz = n.vz * 0.18 + (target.z - old.z) * invDt * 0.06 + moveZ * invDt * 0.04;
        }
        return;
    }

    const speed = delta / Math.max(dt, 1e-6);
    const tangents = wire.nodes.map((_, i) => guidewireTangentAt(i));
    for (let i = 0; i < wire.nodes.length; i++) {
        const n = wire.nodes[i];
        if (n.pinned || wireReleasedFromSheath[i]) continue;

        const inserted = guidewireInsertedCoordinate(i);
        if (inserted <= sheathPath) continue;

        const exitDistance = inserted - sheathPath;
        if (exitDistance > GUIDEWIRE_FEED_ACTIVE_LENGTH) continue;
        const feedTaper = 1 - exitDistance / GUIDEWIRE_FEED_ACTIVE_LENGTH;
        const proximalWeight = feedTaper * feedTaper * (3 - 2 * feedTaper);
        const feedWeight = GUIDEWIRE_FEED_DISTAL_WEIGHT + (1 - GUIDEWIRE_FEED_DISTAL_WEIGHT) * proximalWeight;
        const sheathBlend = Math.max(0, Math.min(1, 1 - exitDistance / SHEATH_EXIT_SUPPORT_LENGTH));
        const tangent = tangents[i];
        let tx = tangent.x * (1 - sheathBlend) + wireDir.x * sheathBlend;
        let ty = tangent.y * (1 - sheathBlend) + wireDir.y * sheathBlend;
        let tz = tangent.z * (1 - sheathBlend) + wireDir.z * sheathBlend;
        const length = Math.hypot(tx, ty, tz) || 1;
        tx /= length;
        ty /= length;
        tz /= length;
        const projected = projectGuidewireFeedDirection(n, { x: tx, y: ty, z: tz });
        n.x += projected.x * delta * feedWeight;
        n.y += projected.y * delta * feedWeight;
        n.z += projected.z * delta * feedWeight;
        n.vx += projected.x * speed * GUIDEWIRE_FEED_VELOCITY_BLEND * feedWeight;
        n.vy += projected.y * speed * GUIDEWIRE_FEED_VELOCITY_BLEND * feedWeight;
        n.vz += projected.z * speed * GUIDEWIRE_FEED_VELOCITY_BLEND * feedWeight;
    }
}

function supportWireAtSheathExit(strength = 1) {
    if (strength <= 0) return;
    for (let i = 0; i < wire.nodes.length; i++) {
        const inserted = guidewireInsertedCoordinate(i);
        const exitDistance = inserted - sheathPath;
        if (exitDistance <= 0 || exitDistance > SHEATH_EXIT_SUPPORT_LENGTH) continue;

        const n = wire.nodes[i];
        if (n.pinned) continue;

        const target = sheathAxisPoint(inserted);
        const dx = n.x - target.x;
        const dy = n.y - target.y;
        const dz = n.z - target.z;
        const axialOffset = dx * wireDir.x + dy * wireDir.y + dz * wireDir.z;
        const lateralX = dx - wireDir.x * axialOffset;
        const lateralY = dy - wireDir.y * axialOffset;
        const lateralZ = dz - wireDir.z * axialOffset;
        const t = 1 - exitDistance / SHEATH_EXIT_SUPPORT_LENGTH;
        const taper = t * t * (3 - 2 * t);
        const amount = SHEATH_EXIT_SUPPORT_BLEND * taper * strength;

        n.x -= lateralX * amount;
        n.y -= lateralY * amount;
        n.z -= lateralZ * amount;

        const axialVelocity = n.vx * wireDir.x + n.vy * wireDir.y + n.vz * wireDir.z;
        const velocityDamping = SHEATH_EXIT_VELOCITY_DAMPING * taper * strength;
        n.vx -= (n.vx - wireDir.x * axialVelocity) * velocityDamping;
        n.vy -= (n.vy - wireDir.y * axialVelocity) * velocityDamping;
        n.vz -= (n.vz - wireDir.z * axialVelocity) * velocityDamping;
    }
}

function applyGuidewireLumenGlide(strength = GUIDEWIRE_LUMEN_GLIDE_STRENGTH) {
    if (strength <= 0) return;
    for (let i = 0; i < wire.nodes.length; i++) {
        const n = wire.nodes[i];
        if (n.pinned) continue;

        const inserted = guidewireInsertedCoordinate(i);
        const exitDistance = inserted - sheathPath;
        if (exitDistance <= GUIDEWIRE_LUMEN_GLIDE_START) continue;

        const { point, tangent, radius } = sampleGuidewireLumen(exitDistance);
        const fade = smoothRange(
            GUIDEWIRE_LUMEN_GLIDE_START,
            GUIDEWIRE_LUMEN_GLIDE_START + GUIDEWIRE_LUMEN_GLIDE_FADE,
            exitDistance
        );
        const distalTaper = 1 - Math.max(0, Math.min(1, exitDistance / guidewireLength));
        const amount = strength * fade * (0.38 + distalTaper * 0.62);
        if (amount <= 0) continue;

        const dx = n.x - point.x;
        const dy = n.y - point.y;
        const dz = n.z - point.z;
        const axialOffset = dx * tangent.x + dy * tangent.y + dz * tangent.z;
        const lateralX = dx - tangent.x * axialOffset;
        const lateralY = dy - tangent.y * axialOffset;
        const lateralZ = dz - tangent.z * axialOffset;
        const lateralLength = Math.hypot(lateralX, lateralY, lateralZ);
        const corridorRadius = Math.max(1.2, radius - 0.8);

        if (lateralLength > corridorRadius) {
            const scale = corridorRadius / Math.max(1e-6, lateralLength);
            const targetX = point.x + tangent.x * axialOffset + lateralX * scale;
            const targetY = point.y + tangent.y * axialOffset + lateralY * scale;
            const targetZ = point.z + tangent.z * axialOffset + lateralZ * scale;
            n.x += (targetX - n.x) * amount;
            n.y += (targetY - n.y) * amount;
            n.z += (targetZ - n.z) * amount;
        }

        const axialLimit = segmentLength * 0.6;
        if (Math.abs(axialOffset) > axialLimit) {
            const targetAxial = Math.sign(axialOffset) * axialLimit;
            const axialCorrection = (targetAxial - axialOffset) * amount * 0.62;
            n.x += tangent.x * axialCorrection;
            n.y += tangent.y * axialCorrection;
            n.z += tangent.z * axialCorrection;
        }

        const centering = GUIDEWIRE_LUMEN_CENTERING * amount;
        n.x += (point.x - n.x) * centering;
        n.y += (point.y - n.y) * centering;
        n.z += (point.z - n.z) * centering;

        const axialVelocity = n.vx * tangent.x + n.vy * tangent.y + n.vz * tangent.z;
        const damping = GUIDEWIRE_LUMEN_GLIDE_DAMPING * amount;
        n.vx -= (n.vx - tangent.x * axialVelocity) * damping;
        n.vy -= (n.vy - tangent.y * axialVelocity) * damping;
        n.vz -= (n.vz - tangent.z * axialVelocity) * damping;
    }
}

function limitGuidewireBuckling(strength = GUIDEWIRE_BUCKLE_LIMIT_STRENGTH, iterations = 1) {
    if (strength <= 0) return;
    for (let iter = 0; iter < iterations; iter++) {
        const corrections = new Array(wire.nodes.length);
        for (let i = 1; i < wire.nodes.length - 1; i++) {
            const n = wire.nodes[i];
            if (n.pinned) continue;
            const inserted = guidewireInsertedCoordinate(i);
            if (inserted <= sheathPath + segmentLength) continue;

            const angle = bendAngleAt(wire.nodes, i);
            if (angle <= GUIDEWIRE_BUCKLE_START_ANGLE) continue;

            const prev = wire.nodes[i - 1];
            const next = wire.nodes[i + 1];
            const severity = Math.max(0, Math.min(1, (angle - GUIDEWIRE_BUCKLE_START_ANGLE) / (170 - GUIDEWIRE_BUCKLE_START_ANGLE)));
            corrections[i] = {
                x: ((prev.x + next.x) * 0.5 - n.x) * strength * severity,
                y: ((prev.y + next.y) * 0.5 - n.y) * strength * severity,
                z: ((prev.z + next.z) * 0.5 - n.z) * strength * severity
            };
        }

        for (let i = 1; i < wire.nodes.length - 1; i++) {
            const n = wire.nodes[i];
            const c = corrections[i];
            if (!c || n.pinned) continue;
            n.x += c.x;
            n.y += c.y;
            n.z += c.z;
            n.vx *= 0.86;
            n.vy *= 0.86;
            n.vz *= 0.86;
        }
    }
}

function advanceTailInput(advance, dt) {
    const nextProgress = Math.max(0, Math.min(guidewireLength, tailProgress + advance * GUIDEWIRE_ADVANCE_RATE * dt));
    const delta = nextProgress - tailProgress;
    const feedSpeed = delta / Math.max(dt, 1e-6);
    const previousPositions = wire.nodes.map(n => ({ x: n.x, y: n.y, z: n.z }));
    tailProgress = nextProgress;
    constrainWireToSheath(feedSpeed);
    feedGuidewireMaterial(delta, dt, previousPositions);
    supportWireAtSheathExit(0.7);
    if (advance > 0) applyGuidewireLumenGlide(0.72);
    return delta;
}

function step(advance = 1) {
    advanceTailInput(advance, fixedDt);
    wire.step(fixedDt);
    wire.collide(collision, fixedDt);
    wire.solveConstraints(fixedDt);
    supportWireAtSheathExit();
    if (advance > 0) applyGuidewireLumenGlide();
    if (advance > 0) {
        limitGuidewireBuckling(GUIDEWIRE_BUCKLE_LIMIT_STRENGTH, 2);
        wire.solveConstraints(fixedDt);
        wire.collide(collision, fixedDt);
        applyGuidewireLumenGlide();
    }
    wire.collide(collision, fixedDt);
    supportWireAtSheathExit(0.7);
    if (advance > 0) {
        limitGuidewireBuckling(0.055, 2);
        applyGuidewireLumenGlide(0.68);
    }
    wire.solveConstraints(fixedDt);
    supportWireAtSheathExit();
    wire.collide(collision, fixedDt);
    wire.solveConstraints(fixedDt);
    supportWireAtSheathExit(0.8);
    if (advance > 0) applyGuidewireLumenGlide(0.52);
}

function stats() {
    let maxAngle = 0;
    let maxEarlyAngle = 0;
    let maxAngleAt = 0;
    let maxEarlyAngleAt = 0;
    let maxLateral = 0;
    let maxEarlyLateral = 0;
    let outsideCount = 0;
    let visibleNodes = 0;
    for (let i = 1; i < wire.nodes.length - 1; i++) {
        const inserted = guidewireInsertedCoordinate(i);
        if (inserted <= sheathPath) continue;
        const exitDistance = inserted - sheathPath;
        visibleNodes++;
        const angle = bendAngleAt(wire.nodes, i);
        if (angle > maxAngle) {
            maxAngle = angle;
            maxAngleAt = exitDistance;
        }
        if (exitDistance < 90 && angle > maxEarlyAngle) {
            maxEarlyAngle = angle;
            maxEarlyAngleAt = exitDistance;
        }
        const { point, tangent, radius } = sampleGuidewireLumen(exitDistance);
        const n = wire.nodes[i];
        const dx = n.x - point.x;
        const dy = n.y - point.y;
        const dz = n.z - point.z;
        const axialOffset = dx * tangent.x + dy * tangent.y + dz * tangent.z;
        const lateral = Math.hypot(dx - tangent.x * axialOffset, dy - tangent.y * axialOffset, dz - tangent.z * axialOffset);
        const excess = lateral - radius;
        maxLateral = Math.max(maxLateral, excess);
        if (exitDistance < 90) maxEarlyLateral = Math.max(maxEarlyLateral, excess);
        if (excess > 2) outsideCount++;
    }
    const tip = wire.nodes[wire.nodes.length - 1];
    return {
        insertedCm: +(tailProgress / 10).toFixed(1),
        tip: {
            x: +tip.x.toFixed(1),
            y: +tip.y.toFixed(1),
            z: +tip.z.toFixed(1)
        },
        maxAngle: +maxAngle.toFixed(1),
        maxAngleAt: +maxAngleAt.toFixed(1),
        maxEarlyAngle: +maxEarlyAngle.toFixed(1),
        maxEarlyAngleAt: +maxEarlyAngleAt.toFixed(1),
        maxLateral: +maxLateral.toFixed(1),
        maxEarlyLateral: +maxEarlyLateral.toFixed(1),
        outsideCount,
        visibleNodes
    };
}

applyGuidewireStiffnessProfile();
constrainWireToSheath();
supportWireAtSheathExit();
applyGuidewireLumenGlide(0.5);

const checkpoints = new Set([10, 15, 20, 30, 40, 60, 80]);
let nextCheckpoint = 10;
const reports = [];
for (let frame = 0; frame < 1200; frame++) {
    step(1);
    const currentCm = tailProgress / 10;
    if (currentCm >= nextCheckpoint) {
        reports.push(stats());
        const candidates = [...checkpoints].filter(value => value > nextCheckpoint);
        nextCheckpoint = candidates.length ? Math.min(...candidates) : Infinity;
    }
    if (tailProgress >= 820) break;
}

console.log(JSON.stringify(reports, null, 2));
