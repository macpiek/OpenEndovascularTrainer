import test from 'node:test';
import assert from 'node:assert/strict';
import {readFileSync} from 'node:fs';
import {gunzipSync} from 'node:zlib';
import {loadCoupledRuntimeAnatomy} from './helpers/coupledRuntimeFixture.js';
import {restoreSharedAxisReplay} from './helpers/sharedAxisReplay.js';
import {advanceSharedAxis} from '../src/physics/kirchhoffSharedAxisAppSystem.js';

test('captured 330 mm wire can withdraw and advance through changing tip grids', async t => {
    const replay = JSON.parse(gunzipSync(readFileSync(new URL('./fixtures/shared-axis/wire-330-motion-block.json.gz', import.meta.url))));
    const anatomy = await loadCoupledRuntimeAnatomy(undefined, replay.anatomy);
    const request = replay.stepRequest, initialInsertion = replay.tools.find(tool => tool.id === 'wire').insertion;
    const increment = initialInsertion - request.tools.find(tool => tool.id === 'wire').insertion;
    try {
        for (const direction of [-1, 1]) {
            let state = restoreSharedAxisReplay(replay, anatomy.field), rotations = request.rotations;
            const original = state, saved = original.wallSamples.find(s => s.captureDiscoveryState).captureDiscoveryState();
            let insertion = initialInsertion, maxPenetration = 0, maxIterations = 0;
            // Move away from the rejected pose, reverse, then settle. Every
            // step changes the distal material interval used by discovery.
            const commands = [...Array(12).fill(direction), ...Array(12).fill(-direction), ...Array(3).fill(0)];
            for (const [step, command] of commands.entries()) {
                insertion += command * increment;
                const tools = request.tools.map(tool => ({...tool, insertion: tool.id === 'wire' ? insertion : tool.insertion}));
                const iterator = advanceSharedAxis(state, rotations, request.dt, tools, request.options);
                let next; do { next = iterator.next(); } while (!next.done);
                const {result} = next.value;
                assert.ok(next.value.state && result.converged, JSON.stringify({direction, step, insertion, result}));
                assert.ok(result.quality.finite && result.quality.maxPenetration < 1e-4, 'wall constraints remain enforced');
                assert.ok(result.quality.bodies.every(body => body.maxLengthError < 1e-4), 'remeshing retains rod lengths');
                maxPenetration = Math.max(maxPenetration, result.quality.maxPenetration);
                maxIterations = Math.max(maxIterations, result.iterations);
                state = next.value.state; rotations = next.value.rotations;
                assert.ok(Math.abs(state.materials.find(material => material.spec.id === 'wire').spec.insertion - insertion) < 1e-9);
            }
            assert.deepEqual(original.wallSamples.find(s => s.captureDiscoveryState).captureDiscoveryState(), saved,
                'candidate discovery never mutates the accepted source history');
            t.diagnostic(JSON.stringify({direction, steps: commands.length, maxPenetration, maxIterations}));
        }
    } finally { anatomy.dispose(); }
});
