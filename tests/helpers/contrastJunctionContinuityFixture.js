import * as THREE from 'three';
import {FlowConcentrationField} from '../../src/contrast/flowConcentrationField.js';
import {createFlowColumnIntegral} from '../../src/contrast/flowColumnIntegral.js';
import {createFlowColumnGeometry} from '../../src/contrast/flowColumnGeometry.js';
import {createFlowJunctionProjection} from '../../src/contrast/flowJunctionProjection.js';

export function smoothConcentrationRamp(x) {
    x = Math.max(0, Math.min(1, x));
    return x * x * (3 - 2 * x);
}

/** Small T/Y graph whose one-dimensional concentrations are continuous at the node. */
export function junctionContinuityFixture(oblique = false, subdivideParent = false) {
    const point = (x, y, z = 0) => new THREE.Vector3(x, y, z);
    const vertices = [point(0, 30), point(0, 0), point(0, -30), oblique ? point(20, -20) : point(30, 0)];
    if (subdivideParent) vertices.push(point(0, 15));
    const definitions = subdivideParent ? [[0, 4, 6], [4, 1, 6], [1, 2, 6], [1, 3, 3]] : [[0, 1, 6], [1, 2, 6], [1, 3, 3]];
    const nodes = new Map(vertices.map((p, id) => [id, {id, point: p, parentEdgeIndex: -1, childEdgeIndices: []}]));
    const edges = definitions.map(([startNodeId, endNodeId, radius], index) => {
        const start = vertices[startNodeId], end = vertices[endNodeId], length = start.distanceTo(end);
        const cellCount = Math.ceil(length / .1), cellLength = length / cellCount;
        nodes.get(startNodeId).childEdgeIndices.push(index); nodes.get(endNodeId).parentEdgeIndex = index;
        return {index, startNodeId, endNodeId, start, end, axis: end.clone().sub(start).normalize(), length,
            radiusStart: radius, radiusEnd: radius, cellCount, cellLength,
            areas: new Float64Array(cellCount).fill(Math.PI * radius * radius),
            volumes: new Float64Array(cellCount).fill(Math.PI * radius * radius * cellLength),
            parentEdgeIndex: -1, childEdgeIndices: []};
    });
    for (const edge of edges) {
        edge.parentEdgeIndex = nodes.get(edge.startNodeId).parentEdgeIndex;
        edge.childEdgeIndices = nodes.get(edge.endNodeId).childEdgeIndices;
    }
    const daughter = edges.length - 1;
    const network = {nodes, edges}, territories = new Int32Array(edges.length), continuations = new Int32Array(edges.length).fill(-1);
    territories[daughter] = daughter;
    for (let i = 0; i < daughter - 1; i++) continuations[i] = i + 1;
    let count = 0;
    const offsets = Uint32Array.from(edges, edge => {const at = count; count += edge.cellCount; return at;});
    const field = new FlowConcentrationField(network, offsets, count);
    const integral = createFlowColumnIntegral(network, field, territories, continuations);
    const geometry = createFlowColumnGeometry(network, integral, territories, continuations);
    const projection = createFlowJunctionProjection(network, integral, territories, geometry);
    const values = new Float32Array(count);
    function fill(profile) {
        for (const edge of edges) for (let cell = 0; cell < edge.cellCount; cell++) values[offsets[edge.index] + cell] =
            profile(edge.index, (cell + .5) * edge.cellLength);
        field.update(values); integral.update(field.values);
    }
    fill((edge, s) => edge === daughter ? 1 - .7 * smoothConcentrationRamp(s / 8) : 1);
    return {network, field, integral, geometry, projection, territories, continuations, daughter, fill,
        dispose() {projection.dispose(); geometry.dispose(); integral.dispose();}};
}

/** Frozen winner-takes-all spatial reconstruction for the before illustration. */
export function frozenJunctionPointValue(f, point, a, sa, b = a, sb = sa) {
    let best = Infinity, owner = a, coordinate = sa;
    const selected = {};
    const consider = (path, seed) => {
        f.geometry.nearest(path, point, seed, 6, selected);
        const score = point.distanceTo(selected.point) - selected.radius;
        if (score < best || (score === best && path < owner)) {
            best = score; owner = path; coordinate = selected.s;
        }
    };
    consider(a, sa); if (b !== a) consider(b, sb);
    for (const junction of f.projection.junctions) {
        if (junction.root !== f.projection.components[a] && junction.root !== f.projection.components[b]) continue;
        if ([0, 1, 2].some(axis => point.getComponent(axis) < junction.min[axis] || point.getComponent(axis) > junction.max[axis])) continue;
        for (const [path, seed] of junction.seeds) consider(path, seed);
    }
    return f.integral.sample(owner, coordinate);
}

/** Keep the historical hard plane and discover the current outer activation plane separately. */
export function anteriorSupportBoundaries(real, projection) {
    const daughter = real.network.edges.find(edge => edge.sourceIndex === 1566);
    if (!daughter) throw Error('The real fixture must contain the original anterior aortic ostium');
    const node = real.network.nodes.get(daughter.startNodeId), parent = real.network.edges[node.parentEdgeIndex];
    const arms = [parent, ...node.childEdgeIndices.map(index => real.network.edges[index])];
    const radius = Math.max(...arms.map(edge => Math.max(edge.radiusStart, edge.radiusEnd)));
    const path = real.built.territories[daughter.index];
    const junction = projection.junctions.find(item => item.seeds.has(path) &&
        Math.abs(.5 * (item.min[1] + item.max[1]) - node.point.y) < 1e-5);
    if (!junction) throw Error('No local support for the original anterior aortic ostium');
    return [
        {name: 'historical inner plane', y: node.point.y - (2 * radius + .25), junction, outer: false},
        {name: 'current outer activation plane', y: junction.min[1], junction, outer: true}
    ];
}
