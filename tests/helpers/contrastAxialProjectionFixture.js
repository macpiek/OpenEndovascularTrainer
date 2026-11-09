import * as THREE from 'three';
import {mergeGeometries} from 'three/examples/jsm/utils/BufferGeometryUtils.js';
import {ContrastFlowNetwork} from '../../src/contrast/flowNetwork.js';
import {AnatomicalContrastSurface} from '../../src/contrast/anatomicalContrastSurface.js';

// An open wall deliberately has no end-cap triangles. Blood still occupies
// its interior: an end-on X-ray must integrate that blood, not see a ring.
export function axialProjectionFixture({radius = 3, length = 24, projection = 'volume'} = {}) {
    const walls = [radius, radius + .2].map((r, index) => {
        const geometry = new THREE.CylinderGeometry(r, r, length, 96, 48, true).toNonIndexed();
        if (index === 0) {
            const p = geometry.attributes.position.array;
            for (let i = 0; i < p.length; i += 9) for (let k = 0; k < 3; k++)
                [p[i + 3 + k], p[i + 6 + k]] = [p[i + 6 + k], p[i + 3 + k]];
            geometry.computeVertexNormals();
        }
        return geometry;
    });
    const geometry = mergeGeometries(walls);
    walls.forEach(g => g.dispose());
    const network = new ContrastFlowNetwork([{id: 0, nodeStartId: 0, nodeEndId: 1,
        start: new THREE.Vector3(0, length / 2, 0), end: new THREE.Vector3(0, -length / 2, 0),
        radiusStart: radius, radiusEnd: radius}], {cellLengthMm: .75});
    const edge = network.edges[0], plume = new Float32Array(edge.cellCount);
    const surface = new AnatomicalContrastSurface(network, new Uint32Array([0]), {fallbackGeometry: geometry}, {projection});
    const concentrations = new Float64Array(edge.cellCount);
    return {radius, length, geometry, network, surface, concentrations,
        fill(profile) {
            for (let i = 0; i < edge.cellCount; i++) {
                const y = length / 2 - (i + .5) * length / edge.cellCount;
                concentrations[i] = typeof profile === 'function' ? profile(y) : profile;
                edge.massMg[i] = edge.volumes[i] * .3 * concentrations[i];
            }
            surface.update(plume, .3, false);
        },
        dispose() {surface.dispose(); geometry.dispose();}
    };
}

// Independent analytic ray/cell intersection. It uses only the known finite
// cylinder and transported cell concentrations, never renderer geometry,
// normals, optical-radius attributes, caps, or shader-derived expressions.
export function finiteCylinderIodineIntegral(origin, direction, radius, length, concentrations, reconstruction = 'constant') {
    const a = direction.x ** 2 + direction.z ** 2;
    const b = 2 * (origin.x * direction.x + origin.z * direction.z);
    const c = origin.x ** 2 + origin.z ** 2 - radius ** 2;
    let radialLow = -Infinity, radialHigh = Infinity;
    if (a < 1e-14) {
        if (c >= 0) return 0;
    } else {
        const determinant = b * b - 4 * a * c;
        if (determinant <= 0) return 0;
        const root = Math.sqrt(determinant);
        radialLow = (-b - root) / (2 * a); radialHigh = (-b + root) / (2 * a);
    }
    const cells = concentrations.length;
    if (reconstruction === 'linear') {
        // Exact integral of the independently specified piecewise-linear
        // concentration profile. Cell-centre interpolation spreads a front
        // over one cell; a step-function oracle would misclassify that mixing
        // as a projection error, especially in a lateral perspective view.
        const knots = [{y: -length / 2, value: concentrations[cells - 1]}];
        for (let i = cells - 1; i >= 0; i--) knots.push({y: length / 2 - (i + .5) * length / cells, value: concentrations[i]});
        knots.push({y: length / 2, value: concentrations[0]});
        let integral = 0;
        for (let i = 1; i < knots.length; i++) {
            const low = knots[i - 1], high = knots[i];
            let entry = Math.max(0, radialLow), exit = radialHigh;
            if (Math.abs(direction.y) < 1e-14) {
                if (origin.y < low.y || origin.y >= high.y) continue;
            } else {
                const a = (low.y - origin.y) / direction.y, b = (high.y - origin.y) / direction.y;
                entry = Math.max(entry, Math.min(a, b)); exit = Math.min(exit, Math.max(a, b));
            }
            if (exit <= entry) continue;
            const midpointY = origin.y + direction.y * (entry + exit) / 2;
            const t = (midpointY - low.y) / (high.y - low.y);
            integral += (exit - entry) * (low.value + t * (high.value - low.value));
        }
        return integral;
    }
    let integral = 0;
    for (let i = 0; i < cells; i++) {
        if (!(concentrations[i] > 0)) continue;
        const high = length / 2 - i * length / cells;
        const low = length / 2 - (i + 1) * length / cells;
        let entry = radialLow, exit = radialHigh;
        if (Math.abs(direction.y) < 1e-14) {
            if (origin.y < low || origin.y >= high) continue;
        } else {
            const t0 = (low - origin.y) / direction.y, t1 = (high - origin.y) / direction.y;
            entry = Math.max(entry, Math.min(t0, t1)); exit = Math.min(exit, Math.max(t0, t1));
        }
        integral += Math.max(0, exit - Math.max(0, entry)) * concentrations[i];
    }
    return integral;
}
