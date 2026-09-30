import * as THREE from 'three';

export function ostialVolumeFixture({ detached = false, axial = false } = {}) {
    const edges = [];
    function add(start, end, radius, parentEdgeIndex = -1) {
        start = new THREE.Vector3(...start);
        end = new THREE.Vector3(...end);
        const length = start.distanceTo(end),
            cellCount = Math.ceil(length / 1.5);
        const volume = (Math.PI * radius * radius * length) / cellCount;
        const edge = {
            index: edges.length,
            start,
            end,
            length,
            axis: end.clone().sub(start).normalize(),
            parentEdgeIndex,
            childEdgeIndices: [],
            radiusStart: radius,
            radiusEnd: radius,
            cellCount,
            cellLength: length / cellCount,
            massMg: new Float64Array(cellCount).fill(volume * 0.006),
            volumes: new Float64Array(cellCount).fill(volume)
        };
        if (parentEdgeIndex >= 0) edges[parentEdgeIndex].childEdgeIndices.push(edge.index);
        edges.push(edge);
    }
    add(axial ? [0, 0, -10] : [0, -30, 0], [0, 0, 0], 10);
    add([0, 0, 0], axial ? [0, 0, 10] : [0, 30, 0], 10, 0);
    if (axial) {
        add([0, 0, 0], [0, 0, 30], 3, 0);
        add([0, 0, 30], [18, 0, 30], 3, 2);
    } else {
        add([0, 0, detached ? 30 : 0], [18, 0, detached ? 30 : 0], 3, detached ? -1 : 0);
    }
    let count = 0;
    const offsets = Uint32Array.from(edges, (e) => {
        const offset = count;
        count += e.cellCount;
        return offset;
    });
    return { network: { edges }, offsets, plume: new Float32Array(count) };
}

export function bifurcationVolumeFixture(reverse = false, crossing = false) {
    const edges = [
        [[0, 30, 0], [0, 0, 0], -1],
        [[0, 0, 0], [-18, -30, 0], 0],
        [[0, 0, 0], [18, -30, 0], 0]
    ].map(([a, b, parentEdgeIndex], index) => {
        const start = new THREE.Vector3(...a),
            end = new THREE.Vector3(...b),
            length = start.distanceTo(end),
            cellCount = Math.ceil(length / 1.5);
        const cellLength = length / cellCount,
            volume = Math.PI * 36 * cellLength;
        return {
            index,
            start,
            end,
            length,
            cellCount,
            cellLength,
            axis: end.clone().sub(start).normalize(),
            parentEdgeIndex,
            childEdgeIndices: index === 0 ? (reverse ? [2, 1] : [1, 2]) : [],
            radiusStart: 6,
            radiusEnd: 6,
            volumes: new Float64Array(cellCount).fill(volume),
            massMg: new Float64Array(cellCount).fill(volume * 0.006)
        };
    });
    if (crossing) {
        const edge = {
            ...edges[1],
            index: 3,
            parentEdgeIndex: -1,
            childEdgeIndices: [],
            start: new THREE.Vector3(-15, 0, 8),
            end: new THREE.Vector3(15, 0, 8),
            axis: new THREE.Vector3(1, 0, 0),
            radiusStart: 1.5,
            radiusEnd: 1.5,
            length: 30,
            cellLength: 1.5,
            cellCount: 20
        };
        edge.volumes = new Float64Array(20).fill(Math.PI * 1.5 ** 2 * 1.5);
        edge.massMg = Float64Array.from(edge.volumes, (v) => v * 0.006);
        edges.push(edge);
    }
    let count = 0;
    const offsets = Uint32Array.from(edges, (e) => {
        const i = count;
        count += e.cellCount;
        return i;
    });
    return { network: { edges }, offsets, plume: new Float32Array(count) };
}
