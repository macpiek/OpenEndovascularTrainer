import test from 'node:test';
import assert from 'node:assert/strict';
import { readFileSync } from 'node:fs';
import { gunzipSync } from 'node:zlib';
import { Vector3, Ray, DoubleSide } from 'three';
import { loadCoupledRuntimeAnatomy } from './helpers/coupledRuntimeFixture.js';
import { restoreSharedAxisReplay } from './helpers/sharedAxisReplay.js';
import { advanceSharedAxis } from '../src/physics/kirchhoffSharedAxisAppSystem.js';

// Browser capture immediately before entering the missing interior SDF band.
// The former centreline fallback eventually classified a valid accepted
// lumen position as exterior and blocked even a zero-feed retry.
test('aneurysm sparse interior remains navigable with exact wall clearance', async () => {
    const fixture = JSON.parse(gunzipSync(readFileSync(new URL('./fixtures/shared-axis/aneurysm-wire-272.80-sparse-interior.json.gz', import.meta.url))));
    const anatomy = await loadCoupledRuntimeAnatomy(undefined, fixture.anatomy);
    try {
        const point = new Vector3(11.272326525048044, -217.19431519129168, 4.856151701539986);
        assert.equal(anatomy.field.packedLumenField.isInsideCoordinates(point.x, point.y, point.z), true);
        const contact = anatomy.field.querySphere(point, 0.4445);
        const closest = anatomy.geometry.boundsTree.closestPointToPoint(point, {});
        assert.equal(contact.source, 'sparse-sdf-bvh');
        assert.ok(contact.inside && contact.signedDistance > 8, 'missing SDF data must not mean outside the lumen');
        assert.ok(Math.abs(contact.signedDistance - closest.distance) < 1e-9, 'clearance must be the exact 3D distance');
        const outside = new Vector3(17.161502350897848, -205.5902698495843, 12.862636720356896);
        assert.ok(anatomy.field.querySphere(outside, 0.4445).signedDistance < 0, 'a genuinely exterior point must still be rejected');
        let state = restoreSharedAxisReplay(fixture, anatomy.field);
        const request = fixture.stepRequest;
        const ray = new Ray();
        const origin = new Vector3(...state.origin);
        const a = new Vector3();
        const b = new Vector3();
        for (let frame = 0; frame < 100; frame++) {
            const tools = request.tools.map(tool => ({
                ...tool,
                insertion: tool.insertion + (tool.id === 'wire' ? frame * 44 / 60 : 0)
            }));
            const iterator = advanceSharedAxis(state, request.rotations, request.dt, tools, request.options);
            let next;
            do {
                next = iterator.next();
            } while (!next.done);
            assert.ok(next.value.state, `frame ${frame}: ${JSON.stringify(next.value.result)}`);
            assert.ok(next.value.result.converged);
            state = next.value.state;
            for (let i = 0; i + 1 < state.positions.length; i++) {
                if (state.coordinates[i] < fixture.sheath.length) continue;
                a.fromArray(state.positions[i]).add(origin);
                b.fromArray(state.positions[i + 1]).add(origin);
                ray.origin.copy(a);
                ray.direction.subVectors(b, a).normalize();
                assert.equal(
                    anatomy.geometry.boundsTree.raycastFirst(ray, DoubleSide, 0, a.distanceTo(b)),
                    null,
                    `frame ${frame}, segment ${i} crossed a wall`
                );
            }
        }
        assert.ok(state.coordinates.at(-1) > 346, 'the wire must pass the former blocking position');
    } finally {
        anatomy.dispose();
    }
});
