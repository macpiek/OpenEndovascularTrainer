import test from 'node:test';
import assert from 'node:assert/strict';
import * as THREE from 'three';
import {axialProjectionFixture} from './helpers/contrastAxialProjectionFixture.js';

// Exercise the precision of the uploaded matrices, not just their double-
// precision algebra. A combined inverse projection/view matrix loses camera
// translation bits when homogeneous coordinates almost cancel at the far plane.
const f32 = Math.fround;
function multiplyFloat32(matrix, vector) {
    const m = matrix.elements.map(f32);
    return [0, 1, 2, 3].map(row => f32(f32(f32(
        f32(m[row] * vector[0]) + f32(m[row + 4] * vector[1])) +
        f32(m[row + 8] * vector[2])) + f32(m[row + 12] * vector[3])));
}
const divide = v => v.slice(0, 3).map(value => f32(value / v[3]));
function normalizeFloat32(v) {
    const length = f32(Math.sqrt(f32(f32(f32(v[0] * v[0]) + f32(v[1] * v[1])) + f32(v[2] * v[2]))));
    return v.map(value => f32(value / length));
}
function uploadedRay(uniforms, ndc) {
    const inverse = uniforms.columnInverseProjection.value;
    const viewToLocal = uniforms.columnViewToLocal.value;
    const near = divide(multiplyFloat32(inverse, [...ndc, -1, 1]));
    const far = divide(multiplyFloat32(inverse, [...ndc, 1, 1]));
    const localOrigin = multiplyFloat32(viewToLocal, [...near, 1]).slice(0, 3);
    const viewDirection = far.map((value, index) => f32(value - near[index]));
    const localDirection = normalizeFloat32(multiplyFloat32(viewToLocal, [...viewDirection, 0]).slice(0, 3));
    return new THREE.Ray(new THREE.Vector3(...localOrigin), new THREE.Vector3(...localDirection).normalize());
}
function combinedRay(camera, mesh, ndc) {
    const inverse = new THREE.Matrix4().multiplyMatrices(camera.projectionMatrix, camera.matrixWorldInverse)
        .multiply(mesh.matrixWorld).invert();
    const near = divide(multiplyFloat32(inverse, [...ndc, -1, 1]));
    const far = divide(multiplyFloat32(inverse, [...ndc, 1, 1]));
    return new THREE.Ray(new THREE.Vector3(...near),
        new THREE.Vector3(...normalizeFloat32(far.map((value, index) => f32(value - near[index])))).normalize());
}
function cameraFor(offset, angle, orthographic = false) {
    const span = 86, distance = 340, radians = angle * Math.PI / 180;
    const camera = orthographic
        ? new THREE.OrthographicCamera(-span, span, span, -span, .1, 900)
        : new THREE.PerspectiveCamera(2 * Math.atan(span / distance) * 180 / Math.PI, 1, .1, 900);
    camera.position.copy(offset).add(new THREE.Vector3(distance * Math.sin(radians), 25, distance * Math.cos(radians)));
    camera.lookAt(offset); camera.updateMatrixWorld(true);
    return camera;
}
const viewport = new THREE.Vector4(17, 23, 1555, 1024);
const renderer = {getCurrentViewport: output => output.copy(viewport)};
function updateCamera(surface, camera) {
    surface.mesh.updateMatrixWorld(true);
    surface.mesh.onBeforeRender(renderer, null, camera, surface.mesh.geometry, surface.material, null);
}

test('uploaded column camera keeps submicron ray accuracy under translated and oblique perspective views', () => {
    const fixture = axialProjectionFixture({projection: 'column'}), caster = new THREE.Raycaster();
    let worstNew = 0, worstLegacy = 0;
    try {
        for (const offset of [new THREE.Vector3(), new THREE.Vector3(142, 654, -63)]) {
            for (const angle of [0, 31, 79]) {
                const camera = cameraFor(offset, angle);
                updateCamera(fixture.surface, camera);
                for (const x of [-.9, -.4, 0, .4, .9]) for (const y of [-.9, -.4, 0, .4, .9]) {
                    const ndc = [x, y];
                    caster.setFromCamera(new THREE.Vector2(...ndc), camera);
                    const referencePoint = caster.ray.at(340, new THREE.Vector3());
                    const actual = uploadedRay(fixture.surface.uniforms, ndc);
                    const error = actual.distanceToPoint(referencePoint);
                    worstNew = Math.max(worstNew, error);
                    worstLegacy = Math.max(worstLegacy, combinedRay(camera, fixture.surface.mesh, ndc).distanceToPoint(referencePoint));
                    assert.ok(error < .0003, `Detector ray error ${error} mm at ${offset.toArray()}, ${angle}°, ${ndc}`);
                    assert.ok(actual.direction.distanceTo(caster.ray.direction) < 1e-6,
                        'The corrected ray must retain the physical perspective direction');
                }
            }
        }
        assert.ok(worstLegacy > .02, 'Fixture must expose the original far-plane cancellation, not just easy origin views');
        assert.ok(worstNew < worstLegacy / 100, 'Separate camera matrices must materially remove the precision loss');
    } finally {fixture.dispose();}
});

test('column camera refreshes both matrices and viewport for transformed anatomy and orthographic views', () => {
    const fixture = axialProjectionFixture({projection: 'column'}), caster = new THREE.Raycaster();
    const mesh = fixture.surface.mesh;
    try {
        for (const orthographic of [false, true]) for (const angle of [0, 31, 79]) {
            mesh.position.set(142, 654, -63); mesh.rotation.set(.13, -.21, .09); mesh.scale.set(1.3, .7, 1.1);
            const camera = cameraFor(mesh.position, angle, orthographic);
            updateCamera(fixture.surface, camera);
            const localFromWorld = mesh.matrixWorld.clone().invert();
            assert.deepEqual(fixture.surface.uniforms.columnViewport.value.toArray(), viewport.toArray());
            assert.deepEqual(fixture.surface.uniforms.columnInverseProjection.value.toArray(), camera.projectionMatrixInverse.toArray(),
                'Perspective divide must use only the camera projection inverse');
            const roundTrip = new THREE.Matrix4().multiplyMatrices(camera.matrixWorldInverse, mesh.matrixWorld)
                .multiply(fixture.surface.uniforms.columnViewToLocal.value);
            roundTrip.elements.forEach((value, i) => assert.ok(Math.abs(value - (i % 5 === 0 ? 1 : 0)) < 1e-10));
            for (const ndc of [[0, 0], [-.7, .4], [.8, -.6]]) {
                caster.setFromCamera(new THREE.Vector2(...ndc), camera);
                const physicalRay = caster.ray.clone().applyMatrix4(localFromWorld);
                const physicalPoint = caster.ray.at(340, new THREE.Vector3()).applyMatrix4(localFromWorld);
                const actual = uploadedRay(fixture.surface.uniforms, ndc);
                assert.ok(actual.distanceToPoint(physicalPoint) < .0003,
                    `Transformed ${orthographic ? 'orthographic' : 'perspective'} ray must hit the same anatomical position`);
                assert.ok(actual.direction.distanceTo(physicalRay.direction) < 1e-6);
            }
        }
    } finally {fixture.dispose();}
});
