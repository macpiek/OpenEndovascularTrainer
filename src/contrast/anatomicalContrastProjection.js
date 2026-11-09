import * as THREE from 'three';
import {MeshBVH, MeshBVHUniformStruct, BVHShaderGLSL} from 'three-mesh-bvh';
import {closestTriangleWeights} from './junctionConcentrationKernel.js';
import {vesselDeformationGLSL} from '../vesselDeformation.js';
import {createProjectionIndexedGeometry} from './projectionIndexedGeometry.js';

// The upstream traversal accepts an in/out distance but initializes its
// pruning distance to infinity. Respect a supplied finite segment length for
// interior-to-axis visibility queries; ordinary exit rays pass zero.
export const boundedRayFunctions = BVHShaderGLSL.bvh_ray_functions.replace(
    'float triangleDistance = INFINITY;', 'float triangleDistance = dist > 0. ? dist : INFINITY;')
    .replace('vec3 a = texelFetch1D( positionAttr, indices.x ).rgb;',
        'vec3 a = projectionTrianglePosition(texelFetch1D(positionAttr, indices.x).rgb);')
    .replace('vec3 b = texelFetch1D( positionAttr, indices.y ).rgb;',
        'vec3 b = projectionTrianglePosition(texelFetch1D(positionAttr, indices.y).rgb);')
    .replace('vec3 c = texelFetch1D( positionAttr, indices.z ).rgb;',
        'vec3 c = projectionTrianglePosition(texelFetch1D(positionAttr, indices.z).rgb);')
    .replace('return intersectsBounds( rayOrigin, rayDirection, boundsMin, boundsMax, dist );',
        'float padding = projectionBoundsPadding(boundsMin, boundsMax); return intersectsBounds(rayOrigin, rayDirection, boundsMin - vec3(padding), boundsMax + vec3(padding), dist);');

export const deformationHooks = {
    vertex: 'vec3 projectionVertexPosition(vec3 point) { return point; }',
    triangle: 'vec3 projectionTrianglePosition(vec3 point) { return point; }',
    bounds: 'float projectionBoundsPadding(vec3 low, vec3 high) { return 0.; }'
};

// Raster vertices use the existing device-contact shader. Ray vertices are
// cached once per contact revision, instead of evaluating contact textures
// inside every nested fragment/BVH traversal (which can trip GPU watchdogs).
// Conservative live node padding keeps the static BVH topology valid.
export function attachAnatomicalProjectionDeformation(deformation, material) {
    Object.assign(material.uniforms, deformation.uniforms);
    if (!material.userData.anatomicalProjectionDeformationAttached) {
        material.vertexShader = vesselDeformationGLSL + material.vertexShader.replace(deformationHooks.vertex,
            'vec3 projectionVertexPosition(vec3 point) { return point + vesselDisplacement(point); }');
        material.fragmentShader = `
            uniform bool vesselDeformActive;
            uniform vec3 vesselDeformMin;
            uniform vec3 vesselDeformMax;
            uniform float vesselDeformMaxDisplacement;
        ` +
            material.fragmentShader
                .replace(deformationHooks.bounds, `float projectionBoundsPadding(vec3 low, vec3 high) {
                    if (!vesselDeformActive || any(greaterThan(low, vesselDeformMax)) ||
                        any(lessThan(high, vesselDeformMin))) return 0.;
                    return vesselDeformMaxDisplacement + .0001;
                }`);
        material.userData.anatomicalProjectionDeformationAttached = true;
    }
    material.needsUpdate = true;
}

export function createProjectionDeformationCache(tree, geometry, bvh) {
    const source = geometry.getAttribute('position'), indices = geometry.index.array;
    const texture = bvh.position, data = texture.image.data;
    const stride = data.length / (texture.image.width * texture.image.height);
    const visited = new Uint32Array(source.count), point = new THREE.Vector3(), displacement = new THREE.Vector3();
    const sphere = new THREE.Sphere(new THREE.Vector3(), 0), candidates = [];
    let stamp = 0, affected = [], deformation = null, unsubscribe = null;
    const diagnostics = {revision: -1, candidateVertices: 0, updatedVertices: 0, uploads: 0};
    const restore = vertex => {
        const at = vertex * stride;
        data[at] = source.getX(vertex); data[at + 1] = source.getY(vertex); data[at + 2] = source.getZ(vertex);
    };
    const update = () => {
        if (!deformation || diagnostics.revision === deformation.revision) return;
        diagnostics.revision = deformation.revision;
        const hadChanges = affected.length > 0;
        for (const vertex of affected) restore(vertex);
        affected.length = 0; candidates.length = 0;
        stamp = (stamp + 1) >>> 0;
        if (!stamp) { visited.fill(0); stamp = 1; }
        // Query each finite patch support independently. A long tool may
        // touch distant regions; its union box must not scan the whole atlas.
        for (const patch of deformation.patches) {
            sphere.center.fromArray(patch.center); sphere.radius = patch.width;
            tree.shapecast({
                intersectsBounds: box => box.intersectsSphere(sphere),
                intersectsRange: (offset, count) => {
                    for (let i = offset * 3, end = (offset + count) * 3; i < end; i++) {
                        const triangle = tree.indirect ? tree.resolveTriangleIndex(Math.floor(i / 3)) : Math.floor(i / 3);
                        const vertex = indices[triangle * 3 + i % 3];
                        if (visited[vertex] === stamp) continue;
                        point.fromBufferAttribute(source, vertex);
                        if (point.distanceToSquared(sphere.center) >= sphere.radius * sphere.radius) continue;
                        visited[vertex] = stamp; candidates.push(vertex);
                    }
                    return false;
                }
            });
        }
        for (const vertex of candidates) {
            point.fromBufferAttribute(source, vertex); deformation.displacementAt(point, displacement);
            if (displacement.lengthSq() === 0) continue;
            const at = vertex * stride;
            data[at] = point.x + displacement.x; data[at + 1] = point.y + displacement.y; data[at + 2] = point.z + displacement.z;
            affected.push(vertex);
        }
        diagnostics.candidateVertices = candidates.length; diagnostics.updatedVertices = affected.length;
        if (hadChanges || affected.length) { texture.needsUpdate = true; diagnostics.uploads++; }
    };
    return {
        diagnostics,
        attach(value) {
            if (deformation === value) return;
            unsubscribe?.(); deformation = value; diagnostics.revision = -1;
            unsubscribe = deformation.addUpdateListener(update); update();
        },
        dispose() { unsubscribe?.(); affected.length = candidates.length = 0; }
    };
}

function dataTexture(values) {
    const width = Math.max(1, Math.min(1024, Math.ceil(Math.sqrt(values.length / 4))));
    const height = Math.max(1, Math.ceil(values.length / (4 * width)));
    const data = new Float32Array(width * height * 4);
    data.set(values);
    const texture = new THREE.DataTexture(data, width, height, THREE.RGBAFormat, THREE.FloatType);
    texture.minFilter = texture.magFilter = THREE.NearestFilter;
    texture.generateMipmaps = false;
    texture.needsUpdate = true;
    return texture;
}

// Concentration is defined on the transport graph, whereas the integration
// interval is defined exclusively by the anatomical triangles. A second,
// small BVH searches graph axes inside that interval without voxelizing away
// fine branches. Bounds include a maximum radius for normalized-distance
// pruning, matching the anatomical wall's ownership rule.
function buildEdgeLookup(network, field, territories, opticalAxes, continuations, geometry) {
    const eligible = network.edges.filter(edge => !edge.transportExcluded && !edge.lumenMappingExcluded);
    // A failed visibility test must never turn into a whole-body graph scan.
    // Bound ownership by the actual wall assigned to each axis, including
    // aneurysmal calibre; graph radii alone would clip dilated anatomy.
    const reach = Float64Array.from(network.edges, edge => 2 * Math.max(edge.radiusStart, edge.radiusEnd) + 1);
    const positions = geometry?.getAttribute('position'), owners = geometry?.getAttribute('projectionOwner');
    if (owners) for (let i = 0; i < owners.count; i++) {
        const edge = network.edges[Math.round(owners.getX(i))];
        if (!edge) continue;
        const x = positions.getX(i) - edge.start.x, y = positions.getY(i) - edge.start.y, z = positions.getZ(i) - edge.start.z;
        const axial = Math.max(0, Math.min(edge.length, x * edge.axis.x + y * edge.axis.y + z * edge.axis.z));
        const distance = Math.hypot(x - axial * edge.axis.x, y - axial * edge.axis.y, z - axial * edge.axis.z);
        reach[edge.index] = Math.max(reach[edge.index], distance + 1);
    }
    // A point between adjacent cross-sections can be supplied by either end
    // of that graph interval. Keep the neighbouring wall's measured reach.
    const ownReach = reach.slice();
    for (const edge of eligible) {
        if (edge.parentEdgeIndex >= 0) reach[edge.index] = Math.max(reach[edge.index], ownReach[edge.parentEdgeIndex]);
        for (const child of edge.childEdgeIndices) reach[edge.index] = Math.max(reach[edge.index], ownReach[child]);
    }
    const edges = new Float32Array(Math.max(1, network.edges.length) * 24);
    const coordinates = new Float32Array(Math.max(1, network.edges.length) * 8);
    let minimumCellLength = Infinity;
    for (const edge of eligible) {
        const i = edge.index * 24;
        edges.set([edge.start.x, edge.start.y, edge.start.z, edge.length,
            edge.axis.x, edge.axis.y, edge.axis.z, edge.radiusStart,
            edge.radiusEnd, field.offsets[edge.index], edge.cellCount, field.start[edge.index],
            field.end[edge.index], field.nodes[field.start[edge.index] - field.cellCount]?.childEdgeIndices.length > 1 ? 1 : 0,
            edge.childEdgeIndices.length > 1 ? 1 : 0, edge.index,
            edge.end.x, edge.end.y, edge.end.z, edge.cellLength,
            territories?.[edge.index] ?? -1, reach[edge.index], 0, 0], i);
        // A short medial-axis connector at an ostium owns a geometric region,
        // but must not clamp that whole region to its tiny axial interval.
        // Use the same continuous lumen coordinate as the wall reconstruction.
        const parent = edge.parentEdgeIndex >= 0 && territories &&
            territories[edge.parentEdgeIndex] === territories[edge.index] ? edge.parentEdgeIndex : -1;
        coordinates.set([opticalAxes?.[edge.index * 3] ?? edge.axis.x,
            opticalAxes?.[edge.index * 3 + 1] ?? edge.axis.y,
            opticalAxes?.[edge.index * 3 + 2] ?? edge.axis.z,
            continuations?.[edge.index] ?? -1, parent, 0, 0, 0], edge.index * 8);
        minimumCellLength = Math.min(minimumCellLength, edge.cellLength);
    }
    const nodes = [], ordered = [];
    function build(items) {
        const id = nodes.length;
        const node = {min: [Infinity, Infinity, Infinity], max: [-Infinity, -Infinity, -Infinity],
            radius: 0, reach: 0, left: -1, right: -1, offset: -1, count: 0};
        nodes.push(node);
        for (const edge of items) {
            node.radius = Math.max(node.radius, edge.radiusStart, edge.radiusEnd);
            node.reach = Math.max(node.reach, reach[edge.index]);
            for (let k = 0; k < 3; k++) {
                node.min[k] = Math.min(node.min[k], edge.start.getComponent(k), edge.end.getComponent(k));
                node.max[k] = Math.max(node.max[k], edge.start.getComponent(k), edge.end.getComponent(k));
            }
        }
        if (items.length <= 4) {
            node.offset = ordered.length;
            node.count = items.length;
            ordered.push(...items.map(edge => edge.index));
        } else {
            let axis = 0;
            for (let k = 1; k < 3; k++) if (node.max[k] - node.min[k] > node.max[axis] - node.min[axis]) axis = k;
            items.sort((a, b) => a.start.getComponent(axis) + a.end.getComponent(axis) -
                b.start.getComponent(axis) - b.end.getComponent(axis));
            const half = items.length >> 1;
            node.left = build(items.slice(0, half));
            node.right = build(items.slice(half));
        }
        return id;
    }
    if (eligible.length) build(eligible);
    else nodes.push({min: [0, 0, 0], max: [0, 0, 0], radius: 0, reach: 0, offset: 0, count: 0, left: -1, right: -1});
    const bounds = new Float32Array(nodes.length * 12);
    for (let i = 0; i < nodes.length; i++) {
        const node = nodes[i];
        // Float32 conversion must not shrink a pruning bound. An explicit
        // padding also covers endpoint roundoff at translated anatomy scales.
        const padding = 1e-4 + 2e-7 * Math.max(...node.min.map(Math.abs), ...node.max.map(Math.abs));
        bounds.set([...node.min.map(x => x - padding), node.radius + padding,
            ...node.max.map(x => x + padding), node.left,
            node.right, node.offset, node.count, node.reach + padding], i * 12);
    }
    const order = new Float32Array(Math.max(1, ordered.length) * 4);
    ordered.forEach((edge, i) => { order[i * 4] = edge; });
    return {edgeTexture: dataTexture(edges), coordinateTexture: dataTexture(coordinates),
        nodeTexture: dataTexture(bounds), orderTexture: dataTexture(order),
        sampleSpacing: Number.isFinite(minimumCellLength) ? Math.max(1e-4, minimumCellLength * .5) : .25};
}

function buildJunctionLookup(junctionField) {
    const junctions = junctionField?.junctions ?? [];
    const data = new Float32Array(Math.max(1, junctions.length) * 52), paths = [];
    for (const junction of junctions) {
        const offset = junction.index * 52;
        data.set([junction.center.x, junction.center.y, junction.center.z, junction.radius,
            junction.axis.x, junction.axis.y, junction.axis.z, junction.nodeSample], offset);
        const territories = [...junction.territories];
        data.set([territories[0] ?? -2, territories[1] ?? -2, territories[2] ?? -2, 0], offset + 8);
        const points = junction.anchors.map(anchor => anchor.point);
        const hubWeights = closestTriangleWeights(junction.center, points);
        const hub = new THREE.Vector3();
        for (let arm = 0; arm < 3; arm++) {
            const anchor = junction.anchors[arm];
            data.set([anchor.point.x, anchor.point.y, anchor.point.z, paths.length / 4,
                anchor.direction.x, anchor.direction.y, anchor.direction.z, anchor.path.length,
                anchor.travelled, anchor.backwards ? 1 : 0, 0, 0], offset + 12 + arm * 12);
            for (const part of anchor.path) paths.push(part.edge, part.start, part.length, 0);
            hub.addScaledVector(anchor.point, hubWeights[arm]);
        }
        data.set([hub.x, hub.y, hub.z, 0], offset + 48);
    }
    // Junction support is finite. Point queries should visit the local fork,
    // not read every fork in a whole-body atlas for every quadrature sample.
    const nodes = [];
    function split(items) {
        const index = nodes.length, low = [Infinity, Infinity, Infinity], high = [-Infinity, -Infinity, -Infinity];
        const node = {low, high, left: -1, right: -1}; nodes.push(node);
        for (const junction of items) for (let k = 0; k < 3; k++) {
            low[k] = Math.min(low[k], junction.center.getComponent(k) - 4 * junction.radius - .001);
            high[k] = Math.max(high[k], junction.center.getComponent(k) + 4 * junction.radius + .001);
        }
        if (items.length === 1) node.left = -items[0].index - 1;
        else {
            let axis = 0;
            for (let k = 1; k < 3; k++) if (high[k] - low[k] > high[axis] - low[axis]) axis = k;
            items.sort((a, b) => a.center.getComponent(axis) - b.center.getComponent(axis));
            const half = items.length >> 1;
            node.left = split(items.slice(0, half)); node.right = split(items.slice(half));
        }
        return index;
    }
    if (junctions.length) split(junctions.slice());
    const bounds = new Float32Array(Math.max(1, nodes.length) * 8);
    nodes.forEach((node, i) => bounds.set([...node.low, node.left, ...node.high, node.right], i * 8));
    return {texture: dataTexture(data), paths: dataTexture(paths), bounds: dataTexture(bounds), count: junctions.length};
}

export const anatomicalProjectionVertexShader = /* glsl */`
attribute float projectionOwner;
varying vec3 lumenPosition;
varying vec3 lumenRay;
varying float lumenOwner;
${deformationHooks.vertex}
void main() {
    vec3 projected = projectionVertexPosition(position);
    vec4 view = modelViewMatrix * vec4(projected, 1.);
    lumenPosition = projected;
    lumenOwner = projectionOwner;
    lumenRay = inverse(mat3(modelViewMatrix)) * (isOrthographic ? vec3(0., 0., -1.) : view.xyz);
    gl_Position = projectionMatrix * view;
}`;

export const anatomicalProjectionFragmentShader = /* glsl */`
precision highp usampler2D;
precision highp isampler2D;
#define BVH_STACK_DEPTH 64
#define TRI_INTERSECT_EPSILON 1e-7
${BVHShaderGLSL.common_functions}
${BVHShaderGLSL.bvh_struct_definitions}
${deformationHooks.triangle}
${deformationHooks.bounds}
${boundedRayFunctions}
uniform BVH lumenBvh;
uniform sampler2D projectionEdges;
uniform sampler2D projectionCoordinates;
uniform int projectionEdgeCount;
uniform sampler2D projectionNodes;
uniform sampler2D projectionOrder;
uniform sampler2D projectionJunctions;
uniform sampler2D projectionJunctionPaths;
uniform sampler2D projectionJunctionBounds;
uniform int projectionJunctionCount;
uniform sampler2D flowValues;
uniform vec2 flowSize;
uniform float displayAlpha;
uniform float signalGain;
uniform float projectionSampleSpacing;
uniform vec3 projectionActiveMin;
uniform vec3 projectionActiveMax;
uniform bool projectionCullActive;
uniform bool debugMode;
uniform vec3 debugColor;
varying vec3 lumenPosition;
varying vec3 lumenRay;
varying float lumenOwner;

float concentrationAt(float index) {
    vec2 uv = (vec2(mod(index, flowSize.x), floor(index / flowSize.x)) + .5) / flowSize;
    vec2 value = texture2D(flowValues, uv).rg;
    return max(0., mix(value.y, value.x, displayAlpha));
}

float edgeConcentration(int edge, float t, float sharedNode) {
    vec4 samples = texelFetch1D(projectionEdges, uint(edge * 6 + 2));
    vec4 ends = texelFetch1D(projectionEdges, uint(edge * 6 + 3));
    float offset = samples.y, count = samples.z;
    float x = clamp(t, 0., 1.) * count - .5;
    float lower, upper, fraction, owner = -1.;
    if (x < 0.) {
        lower = samples.w; upper = offset; fraction = (x + .5) * 2.;
        if (ends.y > .5) owner = offset;
    } else if (x > count - 1.) {
        lower = offset + count - 1.; upper = ends.x; fraction = (x - count + 1.) * 2.;
        if (ends.z > .5) owner = lower;
    } else {
        lower = offset + floor(x); upper = offset + min(count - 1., floor(x) + 1.); fraction = fract(x);
    }
    float a = concentrationAt(lower), b = concentrationAt(upper);
    if (owner >= 0. && lower != sharedNode && upper != sharedNode) {
        float limit = 2. * concentrationAt(owner);
        a = min(a, limit); b = min(b, limit);
    }
    return mix(a, b, fraction);
}

vec3 triangleWeights(vec3 point, vec3 a, vec3 b, vec3 c) {
    vec3 ab = b - a, ac = c - a, ap = point - a;
    float ab2 = dot(ab, ab), ac2 = dot(ac, ac), crossDot = dot(ab, ac);
    if (ab2 * ac2 - crossDot * crossDot <= 1e-12 * max(ab2 * ac2, 1e-30)) {
        float bc2 = dot(c - b, c - b);
        if (ab2 >= ac2 && ab2 >= bc2) {
            float t = ab2 > 0. ? clamp(dot(ap, ab) / ab2, 0., 1.) : 0.;
            return vec3(1. - t, t, 0.);
        }
        if (ac2 >= bc2) {
            float t = ac2 > 0. ? clamp(dot(ap, ac) / ac2, 0., 1.) : 0.;
            return vec3(1. - t, 0., t);
        }
        float t = bc2 > 0. ? clamp(dot(point - b, c - b) / bc2, 0., 1.) : 0.;
        return vec3(0., 1. - t, t);
    }
    float d1 = dot(ab, ap), d2 = dot(ac, ap);
    if (d1 <= 0. && d2 <= 0.) return vec3(1., 0., 0.);
    vec3 bp = point - b;
    float d3 = dot(ab, bp), d4 = dot(ac, bp);
    if (d3 >= 0. && d4 <= d3) return vec3(0., 1., 0.);
    float vc = d1 * d4 - d3 * d2;
    if (vc <= 0. && d1 >= 0. && d3 <= 0.) {
        float v = d1 / (d1 - d3); return vec3(1. - v, v, 0.);
    }
    vec3 cp = point - c;
    float d5 = dot(ab, cp), d6 = dot(ac, cp);
    if (d6 >= 0. && d5 <= d6) return vec3(0., 0., 1.);
    float vb = d5 * d2 - d1 * d6;
    if (vb <= 0. && d2 >= 0. && d6 <= 0.) {
        float w = d2 / (d2 - d6); return vec3(1. - w, 0., w);
    }
    float va = d3 * d6 - d5 * d4;
    if (va <= 0. && d4 - d3 >= 0. && d5 - d6 >= 0.) {
        float w = (d4 - d3) / ((d4 - d3) + (d5 - d6)); return vec3(0., 1. - w, w);
    }
    float denominator = va + vb + vc, v = vb / denominator, w = vc / denominator;
    return vec3(1. - v - w, v, w);
}

vec4 starWeights(vec3 point, int junction) {
    uint base = uint(junction * 13);
    vec3 anchors[3];
    anchors[0] = texelFetch1D(projectionJunctions, base + 3u).xyz;
    anchors[1] = texelFetch1D(projectionJunctions, base + 6u).xyz;
    anchors[2] = texelFetch1D(projectionJunctions, base + 9u).xyz;
    vec3 ab = anchors[1] - anchors[0], ac = anchors[2] - anchors[0];
    vec3 normal = cross(ab, ac);
    float n2 = dot(normal, normal);
    if (n2 <= 1e-12 * max(dot(ab, ab) * dot(ac, ac), 1e-30))
        return vec4(0., triangleWeights(point, anchors[0], anchors[1], anchors[2]));
    vec3 projected = point - normal * (dot(point - anchors[0], normal) / n2);
    vec3 hub = texelFetch1D(projectionJunctions, base + 12u).xyz;
    float closest = INFINITY;
    vec4 result = vec4(0.);
    for (int i = 0; i < 3; i++) {
        int j = (i + 1) % 3;
        vec3 weights = triangleWeights(projected, hub, anchors[i], anchors[j]);
        vec3 d = projected - (weights.x * hub + weights.y * anchors[i] + weights.z * anchors[j]);
        float distance2 = dot(d, d);
        if (distance2 < closest) {
            closest = distance2; result = vec4(0.);
            result[0] = weights.x; result[i + 1] = weights.y; result[j + 1] = weights.z;
        }
    }
    return result;
}

float anchorConcentration(int junction, int arm, float distance, float nodeSample) {
    uint index = uint(junction * 13 + 3 + arm * 3);
    vec4 anchor = texelFetch1D(projectionJunctions, index);
    vec4 direction = texelFetch1D(projectionJunctions, index + 1u);
    vec4 extent = texelFetch1D(projectionJunctions, index + 2u);
    distance = clamp(distance, 0., extent.x);
    int count = int(direction.w);
    vec4 part = texelFetch1D(projectionJunctionPaths, uint(int(anchor.w) + count - 1));
    for (int i = 0; i < count; i++) {
        vec4 candidate = texelFetch1D(projectionJunctionPaths, uint(int(anchor.w) + i));
        if (distance <= candidate.y + candidate.z + 1e-6) { part = candidate; break; }
    }
    float length = texelFetch1D(projectionEdges, uint(int(part.x) * 6)).w;
    float fraction = clamp((distance - part.y) / max(length, 1e-8), 0., 1.);
    return edgeConcentration(int(part.x), extent.y > .5 ? 1. - fraction : fraction, nodeSample);
}

void continuousProjectionCoordinate(vec3 point, inout int owner, out float coordinate) {
    vec4 start = texelFetch1D(projectionEdges, uint(owner * 6));
    vec3 end = texelFetch1D(projectionEdges, uint(owner * 6 + 4)).xyz;
    vec4 frame = texelFetch1D(projectionCoordinates, uint(owner * 2));
    float axialDistance = start.w * .5 + dot(point - (start.xyz + end) * .5, frame.xyz);
    // Links stay within a single continuous lumen, never the other daughter
    // of a true bifurcation. Each step consumes a positive edge length.
    for (int step = 0; step < projectionEdgeCount; step++) {
        int parent = int(texelFetch1D(projectionCoordinates, uint(owner * 2 + 1)).x);
        if (axialDistance < 0. && parent >= 0) {
            owner = parent;
            start = texelFetch1D(projectionEdges, uint(owner * 6));
            axialDistance += start.w;
        } else if (axialDistance > start.w && frame.w >= 0.) {
            axialDistance -= start.w;
            owner = int(frame.w);
            start = texelFetch1D(projectionEdges, uint(owner * 6));
        } else break;
        frame = texelFetch1D(projectionCoordinates, uint(owner * 2));
    }
    coordinate = clamp(axialDistance / max(start.w, 1e-8), 0., 1.);
}

float junctionConcentration(vec3 point, int owner, float coordinate) {
    continuousProjectionCoordinate(point, owner, coordinate);
    vec4 ownerSamples = texelFetch1D(projectionEdges, uint(owner * 6 + 2));
    float ownerCell = ownerSamples.y + min(ownerSamples.z - 1., floor(clamp(coordinate, 0., 1.) * ownerSamples.z));
    vec2 ownerUv = (vec2(mod(ownerCell, flowSize.x), floor(ownerCell / flowSize.x)) + .5) / flowSize;
    // Deliberately replaced lumen is distinct from temporarily clear blood.
    // A junction can mix clear blood, but it cannot recolour graft fabric.
    if (texture2D(flowValues, ownerUv).b > .5) return 0.;
    float own = edgeConcentration(owner, coordinate, -1.);
    float territory = texelFetch1D(projectionEdges, uint(owner * 6 + 5)).x;
    float weighted = 0., sum = 0., remaining = 1.;
    int junctionStack[32], junctionPointer = 0;
    junctionStack[0] = 0;
    while (projectionJunctionCount > 0 && junctionPointer >= 0 && junctionPointer < 32) {
        uint node = uint(junctionStack[junctionPointer--] * 2);
        vec4 low = texelFetch1D(projectionJunctionBounds, node);
        vec4 high = texelFetch1D(projectionJunctionBounds, node + 1u);
        if (any(lessThan(point, low.xyz)) || any(greaterThan(point, high.xyz))) continue;
        if (low.w >= 0.) {
            junctionStack[++junctionPointer] = int(low.w);
            junctionStack[++junctionPointer] = int(high.w);
            continue;
        }
        int j = -int(low.w) - 1;
        uint index = uint(j * 13);
        vec4 center = texelFetch1D(projectionJunctions, index);
        vec3 delta = point - center.xyz;
        if (dot(delta, delta) > 16. * center.w * center.w) continue;
        vec3 territories = texelFetch1D(projectionJunctions, index + 2u).xyz;
        if (all(notEqual(territories, vec3(territory)))) continue;
        vec4 axis = texelFetch1D(projectionJunctions, index + 1u);
        float axial = dot(delta, axis.xyz);
        float transverse2 = max(0., dot(delta, delta) - axial * axial);
        float scaledAxial = axial / (center.w * 1.2), transverseRadius = center.w * 2.;
        float metric = sqrt(scaledAxial * scaledAxial + transverse2 / (transverseRadius * transverseRadius));
        float blend = 1. - smoothstep(1., 2., metric);
        if (blend <= 0.) continue;
        vec4 local;
        local[0] = axial > 0. ? anchorConcentration(j, 0, axial, axis.w) : concentrationAt(axis.w);
        for (int arm = 0; arm < 3; arm++) {
            vec3 direction = texelFetch1D(projectionJunctions, index + uint(4 + arm * 3)).xyz;
            float projected = dot(delta, direction);
            bool parent = arm > 0 && projected < 0. && axial > 0.;
            local[arm + 1] = anchorConcentration(j, parent ? 0 : arm, parent ? -projected : projected, axis.w);
        }
        weighted += dot(local, starWeights(point, j)) * blend;
        sum += blend; remaining *= 1. - blend;
    }
    return sum > 0. ? own * remaining + weighted / sum * (1. - remaining) : own;
}

float nodeScore(int node, vec3 point) {
    vec4 low = texelFetch1D(projectionNodes, uint(node * 3));
    vec4 high = texelFetch1D(projectionNodes, uint(node * 3 + 1));
    float reach = texelFetch1D(projectionNodes, uint(node * 3 + 2)).w;
    reach += projectionBoundsPadding(low.xyz - vec3(reach), high.xyz + vec3(reach));
    vec3 d = max(max(low.xyz - point, point - high.xyz), vec3(0.));
    if (dot(d, d) > reach * reach) return INFINITY;
    return dot(d, d) / max(1e-8, low.w * low.w);
}

// The visibility check is essential for adjacent arteries: the closest graph
// axis may be across a real wall. Such an axis cannot supply this blood's
// concentration, regardless of its calibre or whether it is opacified.
bool axisVisible(vec3 point, vec3 center) {
    vec3 delta = center - point;
    float axisDistance = length(delta);
    if (axisDistance < .0003) return true;
    uvec4 face = uvec4(0u);
    vec3 normal = vec3(0.), bary = vec3(0.);
    // Match the CPU wall mapper's open segment: a roundoff hit on the
    // originating triangle is not a wall separating two blood volumes.
    float epsilon = max(.0001, max(max(abs(point.x), abs(point.y)), abs(point.z)) * 2e-7);
    if (axisDistance <= 2. * epsilon) return true;
    vec3 direction = delta / axisDistance;
    float side = 0., distance = axisDistance - 2. * epsilon;
    bool hit = bvhIntersectFirstHit(lumenBvh, point + direction * epsilon, direction, face, normal, bary, side, distance);
    return !hit || distance >= axisDistance - 2. * epsilon;
}

void findProjectionOwner(vec3 point, inout int previousOwner, out float coordinate) {
    int stack[40];
    int pointer = 0;
    stack[0] = 0;
    float best = INFINITY;
    coordinate = 0.;
    int owner = -1;
    // Adjacent quadrature points usually share a vessel. Seed the exact
    // search from the previous owner to prune distant axes immediately,
    // while still checking the wall and allowing a new owner at a fork.
    if (previousOwner >= 0) {
        vec4 start = texelFetch1D(projectionEdges, uint(previousOwner * 6));
        vec4 axis = texelFetch1D(projectionEdges, uint(previousOwner * 6 + 1));
        float radiusEnd = texelFetch1D(projectionEdges, uint(previousOwner * 6 + 2)).x;
        float t = clamp(dot(point - start.xyz, axis.xyz) / max(start.w, 1e-8), 0., 1.);
        vec3 center = start.xyz + axis.xyz * (start.w * t);
        if (axisVisible(point, center)) {
            vec3 d = point - center;
            float radius = mix(axis.w, radiusEnd, t);
            best = dot(d, d) / max(1e-8, radius * radius);
            owner = previousOwner; coordinate = t;
        }
    }
    while (pointer >= 0 && pointer < 40) {
        int node = stack[pointer--];
        float bound = nodeScore(node, point);
        if (bound >= INFINITY || bound > best) continue;
        vec4 high = texelFetch1D(projectionNodes, uint(node * 3 + 1));
        vec4 children = texelFetch1D(projectionNodes, uint(node * 3 + 2));
        if (children.y >= 0.) {
            for (int j = 0; j < 4; j++) {
                if (j >= int(children.z)) break;
                int edge = int(texelFetch1D(projectionOrder, uint(int(children.y) + j)).x);
                if (edge == previousOwner) continue;
                vec4 start = texelFetch1D(projectionEdges, uint(edge * 6));
                vec4 axis = texelFetch1D(projectionEdges, uint(edge * 6 + 1));
                float radiusEnd = texelFetch1D(projectionEdges, uint(edge * 6 + 2)).x;
                float t = clamp(dot(point - start.xyz, axis.xyz) / max(start.w, 1e-8), 0., 1.);
                vec3 center = start.xyz + axis.xyz * (start.w * t);
                vec3 d = point - center;
                float radius = mix(axis.w, radiusEnd, t);
                float reach = texelFetch1D(projectionEdges, uint(edge * 6 + 5)).y;
                reach += projectionBoundsPadding(start.xyz - vec3(reach), start.xyz + vec3(reach));
                if (dot(d, d) > reach * reach) continue;
                float score = dot(d, d) / max(1e-8, radius * radius);
                if ((score < best || (score == best && edge < owner)) && axisVisible(point, center)) {
                    best = score; owner = edge; coordinate = t;
                }
            }
        } else {
            int left = int(high.w), right = int(children.x);
            float a = nodeScore(left, point), b = nodeScore(right, point);
            // Nearer nodes first makes both ownership and wall ray searches
            // bounded by the local lumen rather than the full arterial tree.
            if (a < b) {
                if (b < INFINITY && b <= best) stack[++pointer] = right;
                if (a < INFINITY && a <= best) stack[++pointer] = left;
            } else {
                if (a < INFINITY && a <= best) stack[++pointer] = left;
                if (b < INFINITY && b <= best) stack[++pointer] = right;
            }
        }
    }
    previousOwner = owner;
}

float iodineAt(vec3 point, inout int previousOwner) {
    float coordinate;
    findProjectionOwner(point, previousOwner, coordinate);
    return previousOwner < 0 ? 0. : junctionConcentration(point, previousOwner, coordinate);
}

float localProjectionStep(vec3 point, vec3 ray, int owner, bool clipAtEndpoint) {
    if (owner < 0) return max(.25, projectionSampleSpacing);
    // Integrate in the same continuous axial coordinate as concentration.
    // A millimetre-wide region can belong geometrically to a 0.02 mm medial
    // connector at an ostium. Using that raw connector's cell length here
    // forces thousands of tiny samples even though its blood field has
    // already continued into the neighbouring, ordinary-length interval.
    float coordinate;
    continuousProjectionCoordinate(point, owner, coordinate);
    vec4 start = texelFetch1D(projectionEdges, uint(owner * 6));
    vec4 axis = texelFetch1D(projectionEdges, uint(owner * 6 + 1));
    axis.xyz = texelFetch1D(projectionCoordinates, uint(owner * 2)).xyz;
    float radiusEnd = texelFetch1D(projectionEdges, uint(owner * 6 + 2)).x;
    float cellLength = texelFetch1D(projectionEdges, uint(owner * 6 + 4)).w;
    float axial = coordinate * start.w, speed = dot(ray, axis.xyz);
    float radius = mix(axis.w, radiusEnd, clamp(axial / max(start.w, 1e-8), 0., 1.));
    // Across a vessel the concentration is almost constant; along it we must
    // resolve transported cells. This avoids making a distant tiny atlas cell
    // force submillimetre sampling on every aortic detector ray.
    float along = .5 * cellLength / max(abs(speed), .001);
    float across = .5 * radius / max(length(cross(ray, axis.xyz)), .001);
    float stepLength = max(.0001, min(along, across));
    if (clipAtEndpoint && abs(speed) > 1e-6) {
        float endDistance = ((speed > 0. ? start.w : 0.) - axial) / speed;
        // Stop at the next graph edge before choosing another cell spacing.
        // The next probe lies just beyond the endpoint, so a short connector
        // cannot be skipped merely because the preceding cell was longer.
        if (endDistance > .0001) stepLength = min(stepLength, endDistance);
    }
    return stepLength;
}

void main() {
    vec3 ray = normalize(lumenRay);
    float activeDistance;
    float activePadding = projectionBoundsPadding(projectionActiveMin, projectionActiveMax);
    if (projectionCullActive && !intersectsBounds(lumenPosition, ray,
        projectionActiveMin - vec3(activePadding), projectionActiveMax + vec3(activePadding), activeDistance)) discard;
    // Only inward-facing entry triangles are drawn. Every disconnected or
    // re-entrant interval gets its own entry fragment and additive integral.
    float epsilon = max(.0001, max(max(abs(lumenPosition.x), abs(lumenPosition.y)), abs(lumenPosition.z)) * 2e-7);
    vec3 origin = lumenPosition + ray * epsilon;
    uvec4 face = uvec4(0u);
    vec3 normal = vec3(0.), bary = vec3(0.);
    float side = 0., distance = 0.;
    if (!bvhIntersectFirstHit(lumenBvh, origin, ray, face, normal, bary, side, distance)) discard;
    if (side < 0. || distance <= 0.) discard;
    float chord = distance + epsilon;
    // Adaptive midpoint quadrature resolves local transported cells, including
    // iodine hidden between clear entry and exit cross-sections. The exact
    // mesh interval continues to determine coverage at every sampling rate.
    float travelled = 0., integral = 0.;
    int previousOwner = int(lumenOwner + .5);
    if (previousOwner < 0 || previousOwner >= projectionEdgeCount) previousOwner = -1;
    for (int sampleIndex = 0; sampleIndex < 1024; sampleIndex++) {
        float remaining = chord - travelled;
        if (remaining <= .00001) break;
        // Reuse the preceding midpoint (or validated entry-face owner) to
        // propose a step; the new midpoint performs the exact spatial search
        // and shrinks it if it encounters a finer interval. Searching again
        // on the boundary both duplicates GPU work and amplifies roundoff.
        float stepLength = min(remaining, localProjectionStep(lumenPosition + ray * travelled, ray, previousOwner, true));
        if (sampleIndex == 1023 && stepLength < remaining) {
            // Bound work for pathological custom meshes while still covering
            // the complete physical interval, instead of dropping its tail.
            // Ordinary arterial rays finish before this coarse fallback.
            for (int tail = 0; tail < 16; tail++) {
                vec3 point = lumenPosition + ray * (travelled + remaining * (float(tail) + .5) / 16.);
                integral += iodineAt(point, previousOwner) * (remaining / 16.);
            }
            break;
        }
        int midpointOwner = previousOwner;
        vec3 midpoint = lumenPosition + ray * (travelled + stepLength * .5);
        float iodine = iodineAt(midpoint, midpointOwner);
        if (midpointOwner >= 0 && midpointOwner != previousOwner) {
            float finerStep = localProjectionStep(midpoint, ray, midpointOwner, false);
            if (finerStep < stepLength) {
                stepLength = finerStep;
                midpoint = lumenPosition + ray * (travelled + stepLength * .5);
                iodine = iodineAt(midpoint, midpointOwner);
            }
        }
        integral += iodine * stepLength;
        travelled += stepLength;
        previousOwner = midpointOwner;
    }
    // The previous two-wall renderer contributed one quarter of the physical
    // chord on each wall. Keep its side-on optical calibration while replacing
    // the divergent wall approximation with a finite interior line integral.
    float depth = .5 * integral * signalGain;
    if (depth <= 0.) discard;
    gl_FragColor = vec4((debugMode ? debugColor : vec3(1.)) * depth, depth);
}`;

/** Build an exact anatomical interval projector from a CLOSED, inward-wound lumen mesh. */
export function buildAnatomicalContrastProjection(network, geometry, concentrationField,
    {junctionField = null, territories = null, opticalAxes = null, continuations = null} = {}) {
    // Groups and triangle order belong to the public surface. The private
    // wrapper permits BVH index sorting without changing either of them.
    const bvhGeometry = createProjectionIndexedGeometry(geometry);
    bvhGeometry.clearGroups();
    const tree = new MeshBVH(bvhGeometry, {maxLeafTris: 8});
    const bvh = new MeshBVHUniformStruct();
    bvh.updateFrom(tree);
    const deformationCache = createProjectionDeformationCache(tree, bvhGeometry, bvh);
    const lookup = buildEdgeLookup(network, concentrationField, territories, opticalAxes, continuations, geometry);
    const junctions = buildJunctionLookup(junctionField);
    const textures = [bvh.index, bvh.position, bvh.bvhBounds, bvh.bvhContents,
        lookup.edgeTexture, lookup.coordinateTexture, lookup.nodeTexture, lookup.orderTexture, junctions.texture, junctions.paths, junctions.bounds];
    return {
        vertexShader: anatomicalProjectionVertexShader,
        fragmentShader: anatomicalProjectionFragmentShader,
        attachVesselDeformation(deformation, material) {
            attachAnatomicalProjectionDeformation(deformation, material);
            deformationCache.attach(deformation);
        },
        deformationDiagnostics: deformationCache.diagnostics,
        uniforms: {
            lumenBvh: {value: bvh},
            projectionEdges: {value: lookup.edgeTexture},
            projectionCoordinates: {value: lookup.coordinateTexture},
            projectionEdgeCount: {value: network.edges.length},
            projectionNodes: {value: lookup.nodeTexture},
            projectionOrder: {value: lookup.orderTexture},
            projectionJunctions: {value: junctions.texture},
            projectionJunctionPaths: {value: junctions.paths},
            projectionJunctionBounds: {value: junctions.bounds},
            projectionJunctionCount: {value: junctions.count},
            projectionSampleSpacing: {value: lookup.sampleSpacing}
        },
        memoryBytes: textures.reduce((sum, texture) => sum + texture.image.data.byteLength, 0),
        dispose() {
            deformationCache.dispose();
            bvh.dispose();
            lookup.edgeTexture.dispose(); lookup.coordinateTexture.dispose(); lookup.nodeTexture.dispose(); lookup.orderTexture.dispose();
            junctions.texture.dispose(); junctions.paths.dispose(); junctions.bounds.dispose();
            bvhGeometry.dispose();
        }
    };
}
