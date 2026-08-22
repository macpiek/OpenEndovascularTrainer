import assert from 'node:assert/strict';
import {
    createContactResult,
    VesselContactField
} from '../src/physics/collision/vesselContactField.js';

function createConstantAsset(signedDistance = 2) {
    const brickSize = 2;
    return {
        metadata: {
            version: 1,
            decodedBytes: 128,
            centerline: { stride: 9, segmentCount: 1, nodeCount: 2 },
            broadPhase: {
                cellSize: 16,
                origin: [-8, -8, -8],
                dimensions: [1, 1, 1]
            },
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
            centerlineSegments: new Float32Array([
                0, -4, 0,
                0, 4, 0,
                4, 4, 0
            ]),
            centerlineEdges: new Uint32Array([0, 1]),
            broadPhaseOffsets: new Uint32Array([0, 1]),
            broadPhaseIds: new Uint32Array([0]),
            sdfBrickKeys: new Uint32Array([0]),
            sdfDistances: new Uint8Array(brickSize ** 3).fill(
                Math.round(Math.abs(signedDistance) / 0.02)
            ),
            sdfInsideBits: new Uint8Array(1).fill(0xff)
        }
    };
}

function snapshot(contact) {
    return {
        values: Array.from(contact.values),
        inside: contact.inside,
        violation: contact.violation,
        conservative: contact.conservative,
        source: contact.source,
        faceIndex: contact.faceIndex,
        insideClearance: contact.insideClearance,
        capsuleSampleCount: contact.capsuleSampleCount,
        point: Array.from(contact.point.values),
        target: Array.from(contact.target.values),
        closestPoint: Array.from(contact.closestPoint.values),
        normal: Array.from(contact.normal.values),
        inward: Array.from(contact.inward.values)
    };
}

const x = new Float32Array([-0.5, -0.25]);
const y = new Float32Array([-0.5, -0.25]);
const z = new Float32Array([-0.5, -0.25]);
const radii = new Float32Array([0.5, 0.5]);
const dx = x[1] - x[0];
const dy = y[1] - y[0];
const dz = z[1] - z[0];
const length = Math.sqrt(dx * dx + dy * dy + dz * dz);
const radius = Math.max(radii[0], radii[1]);
const sampleCount = Math.max(
    1,
    Math.ceil(length / Math.max(4, Math.max(0.5, radius)))
);

const ordinaryField = new VesselContactField(createConstantAsset());
const ordinary = ordinaryField.queryCapsuleSoA(
    x,
    y,
    z,
    radii,
    0,
    createContactResult()
);

const precomputedField = new VesselContactField(createConstantAsset());
const precomputed = precomputedField.queryCapsuleSoA(
    x,
    y,
    z,
    radii,
    0,
    createContactResult(),
    -1,
    false,
    false,
    -1,
    false,
    length,
    sampleCount
);

assert.deepEqual(
    snapshot(precomputed),
    snapshot(ordinary),
    'reusing identical sampling parameters must preserve the complete contact'
);

const coordinateField = new VesselContactField(createConstantAsset());
const coordinate = coordinateField.queryCapsuleCoordinates(
    x[0],
    y[0],
    z[0],
    x[1],
    y[1],
    z[1],
    radius,
    createContactResult()
);

assert.deepEqual(
    snapshot(coordinate),
    snapshot(ordinary),
    'direct coordinate forwarding must preserve the complete contact'
);

console.log('vessel contact sampling reuse tests passed');
