import assert from 'node:assert/strict';
import test from 'node:test';
import {
    applyProximalTwistBoundary,
    createProximalMaterialFrame
} from '../src/physics/kirchhoffOrientationBoundary.js';
import { materialFrameDirectors } from '../src/physics/discreteKirchhoffRod.js';

test('proximal material frame aligns d3 to the edge and applies twist about it', () => {
    const base = createProximalMaterialFrame(
        { x: 0, y: 0, z: 2 },
        0,
        { x: 1, y: 0, z: 0 }
    );
    const twisted = createProximalMaterialFrame(
        { x: 0, y: 0, z: 2 },
        Math.PI / 2,
        { x: 1, y: 0, z: 0 }
    );
    const baseDirectors = materialFrameDirectors(base, {});
    const twistedDirectors = materialFrameDirectors(twisted, {});

    assert.deepEqual(baseDirectors.d3, { x: 0, y: 0, z: 1 });
    assert.ok(Math.abs(twistedDirectors.d3.z - 1) < 1e-12);
    assert.ok(Math.abs(twistedDirectors.d1.x) < 1e-12);
    assert.ok(Math.abs(twistedDirectors.d1.y - 1) < 1e-12);
});

test('orientation boundary leaves positions, previous positions and velocities untouched', () => {
    const body = {
        rodModel: 'kirchhoff',
        activeStart: 0,
        segmentCount: 2,
        x: new Float64Array([0, 0, 0]),
        y: new Float64Array([0, 0, 1]),
        z: new Float64Array([0, 1, 2]),
        previousX: new Float64Array([3, 4, 5]),
        previousY: new Float64Array([6, 7, 8]),
        previousZ: new Float64Array([9, 10, 11]),
        velocityX: new Float64Array([12, 13, 14]),
        velocityY: new Float64Array([15, 16, 17]),
        velocityZ: new Float64Array([18, 19, 20]),
        setProximalOrientationControl(...args) {
            this.controlArgs = args;
        }
    };
    const snapshots = [
        body.x, body.y, body.z,
        body.previousX, body.previousY, body.previousZ,
        body.velocityX, body.velocityY, body.velocityZ
    ].map(array => Array.from(array));

    applyProximalTwistBoundary(body, {
        twist: 0.35,
        preferredD1: { x: 1, y: 0, z: 0 },
        compliance: 1e-7
    });

    [
        body.x, body.y, body.z,
        body.previousX, body.previousY, body.previousZ,
        body.velocityX, body.velocityY, body.velocityZ
    ].forEach((array, index) => assert.deepEqual(Array.from(array), snapshots[index]));
    assert.equal(body.controlArgs[4], 1e-7);
    assert.equal(body.controlArgs[5], 0);
});
