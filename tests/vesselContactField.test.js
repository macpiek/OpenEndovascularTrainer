import assert from 'node:assert/strict';
import crypto from 'node:crypto';
import fs from 'node:fs';
import * as THREE from 'three';
import { STLLoader } from 'three/examples/jsm/loaders/STLLoader.js';
import { preprocessAortaGeometry } from '../src/aortaPreprocess.js';
import { transformAortaGeometry } from '../src/aortaTransform.js';
import { buildStlLumenCast } from '../src/stlCenterline.js';
import {
    decodeCollisionAsset,
    encodeCollisionAsset
} from '../src/physics/collision/collisionAssetFormat.js';
import {
    createBatchContactOutput,
    createContactResult,
    VesselContactField
} from '../src/physics/collision/vesselContactField.js';
import { generateVessel } from '../src/vesselGeometry.js';

const MAX_ASSET_BYTES = 32 * 1024 * 1024;

function arrayBufferFromBuffer(buffer) {
    return buffer.buffer.slice(buffer.byteOffset, buffer.byteOffset + buffer.byteLength);
}

function createConstantAsset(signedDistance = 2) {
    const brickSize = 2;
    return {
        metadata: {
            version: 1,
            decodedBytes: 128,
            centerline: { stride: 9, segmentCount: 1, nodeCount: 2 },
            broadPhase: { cellSize: 16, origin: [-8, -8, -8], dimensions: [1, 1, 1] },
            sdf: {
                voxelSize: 1,
                brickSize,
                band: 4,
                quantization: 0.02,
                distanceQuantization: 0.02,
                origin: [-1, -1, -1],
                dimensions: [1, 1, 1],
                brickCount: 1
            },
            sections: []
        },
        arrays: {
            centerlineSegments: new Float32Array([0, -4, 0, 0, 4, 0, 4, 4, 0]),
            centerlineEdges: new Uint32Array([0, 1]),
            broadPhaseOffsets: new Uint32Array([0, 1]),
            broadPhaseIds: new Uint32Array([0]),
            sdfBrickKeys: new Uint32Array([0]),
            sdfDistances: new Uint8Array(brickSize ** 3).fill(Math.round(Math.abs(signedDistance) / 0.02)),
            sdfInsideBits: new Uint8Array(1).fill(0xff)
        }
    };
}

const formatSource = createConstantAsset();
const encoded = encodeCollisionAsset(formatSource.metadata, formatSource.arrays);
const decoded = decodeCollisionAsset(encoded);
assert.equal(decoded.metadata.format, 'OETCOLL1');
assert.deepEqual([...decoded.arrays.centerlineEdges], [0, 1]);
assert.deepEqual([...decoded.arrays.sdfDistances], [...formatSource.arrays.sdfDistances]);

const constantField = new VesselContactField(formatSource);
const constantContact = constantField.querySphere({ x: -0.5, y: -0.5, z: -0.5 }, 0.5, createContactResult());
assert.equal(constantContact.source, 'sparse-sdf');
assert.ok(Math.abs(constantContact.signedDistance - 2) < 1e-6);
assert.ok(Math.abs(constantContact.signedGap - 1.5) < 1e-6);
assert.equal(constantContact.violation, false);

const batchPositions = new Float32Array([-0.5, -0.5, -0.5, -0.25, -0.25, -0.25]);
const batchRadii = new Float32Array([0.5, 2.5]);
const batchOutput = createBatchContactOutput(2);
constantField.queryBatch(batchPositions, batchRadii, 2, batchOutput);
assert.equal(batchOutput.count, 2);
assert.equal(batchOutput.violations[0], 0);
assert.equal(batchOutput.violations[1], 1);

const capsuleX = new Float32Array([-0.5, -0.25]);
const capsuleY = new Float32Array([-0.5, -0.25]);
const capsuleZ = new Float32Array([-0.5, -0.25]);
const capsuleRadii = new Float32Array([0.5, 0.5]);
const ordinaryCapsule = constantField.queryCapsuleSoA(
    capsuleX,
    capsuleY,
    capsuleZ,
    capsuleRadii,
    0,
    createContactResult()
);
const certifiedInsideField = new VesselContactField(formatSource);
const certifiedInsideCapsule = certifiedInsideField.queryCapsuleSoA(
    capsuleX,
    capsuleY,
    capsuleZ,
    capsuleRadii,
    0,
    createContactResult(),
    ordinaryCapsule.branchId,
    true
);
assert.deepEqual(
    Array.from(certifiedInsideCapsule.values),
    Array.from(ordinaryCapsule.values),
    'an inside-clearance certificate must preserve the complete capsule result'
);
assert.deepEqual(
    Array.from(certifiedInsideCapsule.inward.values),
    Array.from(ordinaryCapsule.inward.values),
    'the certified-inside fast path must preserve the wall normal'
);

const sourceBytes = fs.readFileSync('res/Aorta_plain.stl');
const sourceHash = crypto.createHash('sha256').update(sourceBytes).digest('hex');
const assetBytes = fs.readFileSync('res/Aorta_plain.collision.bin');
const asset = decodeCollisionAsset(arrayBufferFromBuffer(assetBytes));
assert.equal(asset.metadata.source.stlSha256, sourceHash, 'collision asset should match the current STL');
assert.ok(asset.metadata.decodedBytes <= MAX_ASSET_BYTES, 'decoded collision asset should stay under 32 MB');

const field = new VesselContactField(asset);
assert.ok(field.runtimeBytes <= MAX_ASSET_BYTES, 'asset plus runtime brick lookup should stay under 32 MB');

const centerlineContact = createContactResult();
const centerlineSegments = asset.arrays.centerlineSegments;
const centerlineStride = asset.metadata.centerline.stride;
let centerlineSampleCount = 0;
let minimumCenterlineDistance = Infinity;
for (let segment = 0; segment < asset.metadata.centerline.segmentCount; segment++) {
    const offset = segment * centerlineStride;
    const ax = centerlineSegments[offset];
    const ay = centerlineSegments[offset + 1];
    const az = centerlineSegments[offset + 2];
    const bx = centerlineSegments[offset + 3];
    const by = centerlineSegments[offset + 4];
    const bz = centerlineSegments[offset + 5];
    const sampleCount = Math.max(1, Math.ceil(Math.hypot(bx - ax, by - ay, bz - az) / 0.4));
    for (let sample = 0; sample <= sampleCount; sample++) {
        const t = sample / sampleCount;
        field.querySphere({
            x: ax + (bx - ax) * t,
            y: ay + (by - ay) * t,
            z: az + (bz - az) * t
        }, 0, centerlineContact);
        minimumCenterlineDistance = Math.min(minimumCenterlineDistance, centerlineContact.signedDistance);
        centerlineSampleCount++;
        assert.ok(
            centerlineContact.signedDistance >= 0,
            `centerline segment ${segment} sample ${sample} should stay inside the lumen`
        );
    }
}
console.log('collision centerline lumen samples', centerlineSampleCount);
console.log('collision centerline minimum wall distance mm', minimumCenterlineDistance.toFixed(6));

const geometry = new STLLoader().parse(arrayBufferFromBuffer(sourceBytes));
const { vessel } = generateVessel(140, 0);
transformAortaGeometry(geometry, vessel);
const preprocessing = preprocessAortaGeometry(geometry);
const referenceLumenCast = buildStlLumenCast(geometry, {
    lumenField: preprocessing.lumenField,
    fieldOnly: true
});
field.setFallbackGeometry(geometry);
const positions = geometry.attributes.position;
const point = new THREE.Vector3();
const closest = { point: new THREE.Vector3(), distance: Infinity, faceIndex: -1 };
const queryScratch = {
    inward: { x: 0, y: 0, z: 0 },
    normal: { x: 0, y: 0, z: 0 },
    closestPoint: { x: 0, y: 0, z: 0 }
};
const contact = createContactResult();
let seed = 0x6d2b79f5;
const random = () => {
    seed = (Math.imul(seed, 1664525) + 1013904223) >>> 0;
    return seed / 0x100000000;
};
const packedQueryScratch = {
    inward: { x: 0, y: 0, z: 0 },
    normal: { x: 0, y: 0, z: 0 },
    closestPoint: { x: 0, y: 0, z: 0 }
};
let maxPackedLumenError = 0;
let packedSignMismatchCount = 0;

for (let sampleIndex = 0; sampleIndex < 8000; sampleIndex++) {
    const vertexIndex = Math.floor(random() * positions.count);
    point.set(
        positions.getX(vertexIndex) + (random() * 2 - 1) * 6,
        positions.getY(vertexIndex) + (random() * 2 - 1) * 6,
        positions.getZ(vertexIndex) + (random() * 2 - 1) * 6
    );
    const expected = referenceLumenCast.field.query(point, queryScratch);
    const actual = field.packedLumenField.query(point, packedQueryScratch);
    maxPackedLumenError = Math.max(
        maxPackedLumenError,
        Math.abs(expected.signedDistance - actual.signedDistance)
    );
    if (
        Math.abs(expected.signedDistance) > 0.03 &&
        Math.sign(expected.signedDistance) !== Math.sign(actual.signedDistance)
    ) packedSignMismatchCount++;
}

console.log('packed lumen max distance error mm', maxPackedLumenError.toExponential(3));
assert.ok(maxPackedLumenError <= 0.03, 'packed lumen distance should match the source field within 0.03 mm');
assert.equal(packedSignMismatchCount, 0, 'packed lumen field should preserve every unambiguous inside/outside sign');

let sdfSamples = 0;
let maxDistanceError = 0;
let signMismatchCount = 0;
let worstSample = null;
const signMismatchSamples = [];

for (let sampleIndex = 0; sampleIndex < 12000; sampleIndex++) {
    const vertexIndex = Math.floor(random() * positions.count);
    point.set(
        positions.getX(vertexIndex) + (random() * 2 - 1) * 3.5,
        positions.getY(vertexIndex) + (random() * 2 - 1) * 3.5,
        positions.getZ(vertexIndex) + (random() * 2 - 1) * 3.5
    );
    closest.distance = Infinity;
    const hit = geometry.boundsTree.closestPointToPoint(point, closest);
    const unsignedDistance = hit?.distance ?? point.distanceTo(closest.point);
    if (unsignedDistance > 3.5) continue;
    const lumenState = referenceLumenCast.field.query(point, queryScratch);
    const expected = lumenState.inside ? unsignedDistance : -unsignedDistance;
    field.querySphere(point, 0, contact);
    if (!contact.source.startsWith('sparse-sdf')) continue;
    sdfSamples++;
    const distanceError = Math.abs(Math.abs(contact.signedDistance) - unsignedDistance);
    if (distanceError > maxDistanceError) {
        maxDistanceError = distanceError;
        worstSample = {
            point: point.toArray(),
            closest: closest.point.toArray(),
            expected,
            actual: contact.signedDistance,
            branchId: contact.branchId
        };
    }
    if (Math.abs(lumenState.signedDistance) > 0.1 && Math.sign(contact.signedDistance) !== Math.sign(expected)) {
        signMismatchCount++;
        if (signMismatchSamples.length < 8) {
            signMismatchSamples.push({
                point: point.toArray(),
                expected,
                actual: contact.signedDistance,
                branchId: contact.branchId
            });
        }
    }
}

console.log('collision asset encoded MB', (assetBytes.byteLength / 1048576).toFixed(2));
console.log('collision field runtime MB', (field.runtimeBytes / 1048576).toFixed(2));
console.log('collision SDF validation samples', sdfSamples);
console.log('collision SDF max error mm', maxDistanceError.toFixed(4));
console.log('collision SDF sign mismatches', signMismatchCount);
console.log('collision BVH refinements', field.getStats().bvhRefinements);
if (worstSample) console.log('collision SDF worst sample', JSON.stringify(worstSample));
if (signMismatchSamples.length) console.log('collision SDF mismatch samples', JSON.stringify(signMismatchSamples));

assert.ok(sdfSamples >= 5000, 'validation should cover thousands of points in the SDF wall band');
assert.ok(maxDistanceError <= 0.2, 'interpolated SDF error should stay within 0.20 mm of BVH');
assert.equal(signMismatchCount, 0, 'SDF should preserve the lumen sign outside the 0.20 mm ambiguity band');
assert.ok(field.getStats().bvhRefinements > 0, 'BVH should validate the narrowest contact band');

field.resetStats();
field.querySphere(point, 0, contact);
assert.equal(field.getStats().resultAllocations, 0, 'reused contact output should not allocate a result');
field.querySphere(point, 0);
assert.equal(field.getStats().resultAllocations, 1, 'missing output should be visible in allocation diagnostics');
