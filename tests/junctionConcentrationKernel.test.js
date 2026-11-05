import test from 'node:test';
import assert from 'node:assert/strict';
import {closestTriangleWeights, junctionStarWeights, junctionBlendWeight} from '../src/contrast/junctionConcentrationKernel.js';

const p = (x, y, z = 0) => ({x, y, z});
const anchors = [p(0, 10), p(-8, -10), p(12, -10)];
const close = (a, b, tolerance = 1e-12) => assert.ok(Math.abs(a - b) <= tolerance, `${a} != ${b}`);
const value = (weights, values) => weights.reduce((sum, w, i) => sum + w * values[i], 0);

test('junction interpolation reproduces affine concentration and is invariant through the anchor plane', () => {
    const concentration = a => .4 + .01 * a.x + .015 * a.y;
    const values = anchors.map(concentration);
    for (const weights of [[.2, .3, .5], [.8, .1, .1], [.01, .45, .54]]) {
        const point = p(value(weights, anchors.map(a => a.x)), value(weights, anchors.map(a => a.y)));
        for (const z of [-8, 0, 8]) {
            const result = closestTriangleWeights({...point, z}, anchors);
            result.forEach((w, i) => close(w, weights[i]));
            close(value(result, values), concentration(point));
        }
    }
});

test('anchors are cardinal, while the whole spatial field stays convex and independent of arm order', () => {
    anchors.forEach((point, i) => closestTriangleWeights(point, anchors).forEach((w, j) => close(w, i === j ? 1 : 0)));
    const values = [.05, .8, .2];
    for (let x = -24; x <= 24; x += 2) for (let y = -24; y <= 24; y += 2) {
        const point = p(x, y, 4), weights = closestTriangleWeights(point, anchors);
        close(weights.reduce((sum, w) => sum + w, 0), 1);
        assert.ok(weights.every(w => Number.isFinite(w) && w >= -1e-14 && w <= 1 + 1e-14));
        close(value(weights, [1, 1, 1]), 1);
        const reverse = closestTriangleWeights(point, [...anchors].reverse());
        close(value(weights, values), value(reverse, [...values].reverse()));
    }
});

test('crossing closest-arm and triangle-edge boundaries cannot produce a concentration jump', () => {
    const values = [0, 1, .15], epsilon = 1e-5;
    for (let x = -20; x <= 20; x += .5) for (let y = -20; y <= 20; y += .5) {
        const left = value(closestTriangleWeights(p(x - epsilon, y), anchors), values);
        const right = value(closestTriangleWeights(p(x + epsilon, y), anchors), values);
        assert.ok(Math.abs(left - right) < 1e-5);
    }
});

test('thin and collapsed junctions stay finite without negative interpolation weights', () => {
    for (const tiny of [0, 1e-14, 1e-8]) {
        const thin = [p(0, 0), p(5, tiny), p(10, 0)];
        const weights = closestTriangleWeights(p(4, 2, 3), thin);
        close(value(weights, [0, .5, 1]), .4);
        assert.ok(weights.every(w => Number.isFinite(w) && w >= 0));
    }
    const collapsed = closestTriangleWeights(p(5, 0), [p(0, 0), p(0, 0), p(0, 0)]);
    close(value(collapsed, [.2, .2, .2]), .2);
});

test('junction support is compact with zero slope where it joins the ordinary field', () => {
    const center = p(0, 0), weight = x => junctionBlendWeight(p(x, 0), center, 10, 20);
    close(weight(0), 1); close(weight(10), 1); close(weight(20), 0); close(weight(100), 0);
    close(weight(15), .5);
    assert.ok((weight(10) - weight(10 + 1e-4)) / 1e-4 < 1e-5);
    assert.ok((weight(20 - 1e-4) - weight(20)) / 1e-4 < 1e-5);
    close(junctionBlendWeight(p(0, 0), center, 20, 10), 0);
});

test('the four-anchor star is cardinal at actual junction blood and at all three arms', () => {
    const center = p(1, 0, 7), hub = p(1, 0);
    const centerWeights = junctionStarWeights(hub, center, anchors);
    centerWeights.forEach((w, i) => close(w, i === 0 ? 1 : 0));
    anchors.forEach((point, i) => junctionStarWeights(point, center, anchors)
        .forEach((w, j) => close(w, j === i + 1 ? 1 : 0)));
    for (let x = -24; x <= 24; x += 2) for (let y = -24; y <= 24; y += 2) {
        const weights = junctionStarWeights(p(x, y, 3), center, anchors);
        close(value(weights, [1, 1, 1, 1]), 1);
        assert.ok(weights.every(w => Number.isFinite(w) && w >= -1e-12 && w <= 1 + 1e-12));
    }
});

test('different hub and arm concentrations remain continuous across all star seams', () => {
    const center = p(1, 0), concentrations = [.32, .03, .9, .12], epsilon = 1e-5;
    for (let x = -20; x <= 20; x += .25) for (let y = -20; y <= 20; y += .25) {
        const left = value(junctionStarWeights(p(x - epsilon, y), center, anchors), concentrations);
        const right = value(junctionStarWeights(p(x + epsilon, y), center, anchors), concentrations);
        assert.ok(Math.abs(left - right) < 1e-5, `Star seam at ${x}, ${y}: ${left} → ${right}`);
        const reversed = junctionStarWeights(p(x, y), center, [...anchors].reverse());
        const ordinary = junctionStarWeights(p(x, y), center, anchors);
        close(value(ordinary, concentrations), value(reversed, [concentrations[0], ...concentrations.slice(1).reverse()]));
    }
});

test('the star preserves a physical affine field throughout its plane and handles degenerate geometry', () => {
    const center = p(1, 0), concentration = a => .4 + .01 * a.x + .015 * a.y;
    const values = [concentration(center), ...anchors.map(concentration)];
    for (const weights of [[.2, .3, .5], [.8, .1, .1], [.01, .45, .54]]) {
        const point = p(value(weights, anchors.map(a => a.x)), value(weights, anchors.map(a => a.y)));
        close(value(junctionStarWeights(point, center, anchors), values), concentration(point));
    }
    const collapsed = junctionStarWeights(p(5, 0), p(0, 0), [p(0, 0), p(0, 0), p(0, 0)]);
    close(value(collapsed, [.2, .2, .2, .2]), .2);
    assert.ok(collapsed.every(w => Number.isFinite(w) && w >= 0));
});
