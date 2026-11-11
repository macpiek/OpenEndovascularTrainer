import * as THREE from 'three';
import {mergeGeometries} from 'three/examples/jsm/utils/BufferGeometryUtils.js';
import {MeshBVH} from 'three-mesh-bvh';
import {ContrastFlowNetwork} from '../../src/contrast/flowNetwork.js';
import {AnatomicalContrastSurface} from '../../src/contrast/anatomicalContrastSurface.js';
import {createProjectionIndexedGeometry} from '../../src/contrast/projectionIndexedGeometry.js';
import {VesselDeformation} from '../../src/vesselDeformation.js';

// Deliberately small, anatomy-free fixture. Uniform iodine makes its optical
// integral depend only on the lumen chord, independently of flow sampling.
export function contrastSpeckleFixture({translated = true, tubularSegments = 256, radialSegments = 64,
    cachedRaster = true} = {}) {
    const offset = new THREE.Vector3(...(translated ? [142, 654, -63] : [0, 0, 0]));
    const curve = new THREE.CubicBezierCurve3(...[
        [-44, -68, 4], [-58, 4, -8], [35, -8, 18], [22, 69, 2]
    ].map(p => new THREE.Vector3(...p).add(offset)));
    const radius = 6;
    const walls = [radius, radius + .25].map((r, wall) => {
        const geometry = new THREE.TubeGeometry(curve, tubularSegments, r, radialSegments, false).toNonIndexed();
        if (wall === 0) {
            const p = geometry.attributes.position.array;
            for (let i = 0; i < p.length; i += 9) for (let k = 0; k < 3; k++)
                [p[i + 3 + k], p[i + 6 + k]] = [p[i + 6 + k], p[i + 3 + k]];
            geometry.computeVertexNormals();
        }
        return geometry;
    });
    const geometry = mergeGeometries(walls);
    walls.forEach(g => g.dispose());
    const edgeCount = 48;
    const network = new ContrastFlowNetwork(Array.from({length: edgeCount}, (_, i) => ({
        id: i, nodeStartId: i, nodeEndId: i + 1,
        start: curve.getPoint(i / edgeCount), end: curve.getPoint((i + 1) / edgeCount),
        radiusStart: radius, radiusEnd: radius
    })), {cellLengthMm: .75});
    let cellCount = 0;
    const offsets = Uint32Array.from(network.edges, edge => {
        const result = cellCount;
        cellCount += edge.cellCount;
        for (let i = 0; i < edge.cellCount; i++) edge.massMg[i] = edge.volumes[i] * .3;
        return result;
    });
    const surface = new AnatomicalContrastSurface(network, offsets, {fallbackGeometry: geometry}, {projection: 'column'});
    const deformation = cachedRaster ? new VesselDeformation() : null;
    if (deformation) deformation.attach(surface.material);
    surface.uniforms.signalGain.value = 1;
    surface.update(new Float32Array(cellCount), .3, false);
    // CPU ray/triangle intersections use double-precision arithmetic and
    // independently traverse both surfaces of the same closed lumen.
    const referenceGeometry = createProjectionIndexedGeometry(surface.samplingGeometry);
    const reference = new MeshBVH(referenceGeometry, {maxLeafTris: 8});
    return {surface, offset, reference, radius, triangles: referenceGeometry.index.count / 3,
        dispose() {referenceGeometry.dispose(); surface.dispose(); deformation?.dispose(); geometry.dispose();}};
}

export function referenceLumenIntegral(tree, ray) {
    const hits = tree.raycast(ray, THREE.DoubleSide).sort((a, b) => a.distance - b.distance);
    let integral = 0, entry = null, previous = -Infinity;
    for (const hit of hits) {
        // Adjacent triangles can both report a shared edge or vertex.
        if (hit.distance - previous < 1e-6) continue;
        previous = hit.distance;
        if (entry === null) entry = hit.distance;
        else {integral += .5 * (hit.distance - entry); entry = null;}
    }
    return integral;
}
