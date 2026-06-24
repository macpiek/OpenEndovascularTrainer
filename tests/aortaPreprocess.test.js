import assert from 'node:assert/strict';
import fs from 'node:fs';
import * as THREE from 'three';
import { STLLoader } from 'three/examples/jsm/loaders/STLLoader.js';
import { createMeshLumenCollider } from '../src/aortaModel.js';
import { createLumenField, preprocessAortaGeometry } from '../src/aortaPreprocess.js';
import { generateVessel } from '../src/vesselGeometry.js';

function assertFinitePoint(point, label) {
    assert.ok(Number.isFinite(point.x), `${label}.x should be finite`);
    assert.ok(Number.isFinite(point.y), `${label}.y should be finite`);
    assert.ok(Number.isFinite(point.z), `${label}.z should be finite`);
}

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
    for (const seg of vessel.segments || []) {
        if (!seg.isSheath) ys.push(seg.start.y, seg.end.y);
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

const geometry = loadTransformedAorta();
const preprocessing = preprocessAortaGeometry(geometry);
const field = preprocessing.lumenField;

console.log('aorta lumen slices', preprocessing.lumenSlices.length);
console.log('aorta lumen samples', preprocessing.interiorSamples.length);
console.log('aorta contour debug segments', preprocessing.lumenContourDebugSegments.length / 6);

assert.ok(preprocessing.lumenSlices.length > 50, 'STL preprocessing should extract lumen slices');
assert.ok(preprocessing.interiorSamples.length > 100, 'STL preprocessing should extract lumen interior samples');

for (const sample of preprocessing.interiorSamples) {
    const state = field.query(sample);
    assert.ok(
        state.signedDistance > 0,
        `interior sample should be inside lumen, got ${state.signedDistance}`
    );
}

const insideMain = field.query(new THREE.Vector3(0, -300, 0));
const outsideMain = field.query(new THREE.Vector3(0, -300, 80));
const insideIliac = field.query(new THREE.Vector3(-74, -390, 16));
const outsideIliac = field.query(new THREE.Vector3(-74, -390, 70));

assert.ok(insideMain.inside, 'known central lumen point should be inside');
assert.ok(!outsideMain.inside, 'known external main-vessel point should be outside');
assert.ok(insideIliac.inside, 'known iliac lumen point should be inside');
assert.ok(!outsideIliac.inside, 'known external iliac point should be outside');

const collider = createMeshLumenCollider(geometry, { lumenField: field });
assert.equal(collider.pointContact(new THREE.Vector3(0, -300, 0), 0.45).violation, false);
assert.equal(collider.pointContact(new THREE.Vector3(0, -300, 80), 0.45).violation, true);

const emptyField = createLumenField([]);
const emptyCollider = createMeshLumenCollider(new THREE.BufferGeometry(), { lumenField: emptyField });
const malformedSample = new THREE.Vector3(12, -34, 56);
const emptyContact = emptyCollider.pointContact(malformedSample, 0.45);
assert.equal(emptyContact.violation, true, 'empty lumen should still report a boundary violation');
assert.equal(emptyContact.signedDistance, -Infinity, 'empty lumen should preserve signed-distance failure state');
assertFinitePoint(emptyContact.target, 'empty non-scratch contact target');

const scratchContact = { query: {} };
emptyCollider.pointContact(malformedSample, 0.45, scratchContact);
assert.equal(scratchContact.violation, true, 'empty scratch lumen should still report a boundary violation');
assert.equal(scratchContact.signedDistance, -Infinity, 'empty scratch lumen should preserve signed-distance failure state');
assertFinitePoint(scratchContact.target, 'empty scratch contact target');
