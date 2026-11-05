// Generate once with npm run preview:contrast:mixing. The generator runs these
// same assertions against current renderer code before writing its fixture.
import fs from 'node:fs';
import test from 'node:test';
import assert from 'node:assert/strict';
import {assertBifurcationMixingRegression} from './helpers/actualBifurcationMixingRegression.js';

const file = new URL('./actualBifurcationMixing.fixture.json', import.meta.url);
const fixture = fs.existsSync(file) ? JSON.parse(fs.readFileSync(file, 'utf8')) : null;
const options = {skip: fixture ? false : 'Run npm run preview:contrast:mixing to generate the actual-anatomy recording'};

test('actual aortoiliac wall stays continuous and causally dilute throughout nine recorded pulse/front phases', options, () => {
    assertBifurcationMixingRegression(fixture);
});

test('the actual-anatomy check rejects the old separate-arm seam rather than merely accepting any fixture', options, () => {
    const broken = {...fixture, afterStencil: fixture.geometry.data.attributes.flowStencil.array,
        cases: fixture.cases.map(c => ({...c, states: {...c.states, after: c.states.before}}))};
    assert.throws(() => assertBifurcationMixingRegression(broken), /concentration seam returned/);
});

test('the actual-anatomy check rejects a fabricated proximal column and remote-anchor prefill', options, () => {
    const cases = fixture.cases.map(c => ({...c, states: {...c.states}}));
    const peak = cases.find(c => c.phase === .25 && Math.abs(c.elapsed - 1.3) < 1e-6);
    // A uniform false column is perfectly smooth, so the seam check alone
    // cannot reject it; the independent dilution budget must do so.
    peak.states.after = peak.states.after.slice().fill(.4);
    assert.throws(() => assertBifurcationMixingRegression({...fixture, cases}),
        /broad proximal column/);
    assert.throws(() => assertBifurcationMixingRegression({...fixture,
        causalityProbe: {sourceCells: 2, sourceAnchorMinimum: 1, junctionConcentration: 0, proximalMaximum: .2}}),
        /Remote anchor iodine prefilled/);
});
