import * as THREE from 'three';
import {MeshBVH, MeshBVHUniformStruct, BVHShaderGLSL} from 'three-mesh-bvh';
import {createProjectionIndexedGeometry} from './projectionIndexedGeometry.js';
import {createFlowColumnIntegral} from './flowColumnIntegral.js';
import {createFlowColumnGeometry} from './flowColumnGeometry.js';
import {createFlowJunctionProjection} from './flowJunctionProjection.js';
import {vesselDeformationGLSL} from '../vesselDeformation.js';
import {boundedRayFunctions, deformationHooks, attachAnatomicalProjectionDeformation,
    createProjectionDeformationCache} from './anatomicalContrastProjection.js';

function texture(values) {
    const width = Math.min(2048, Math.max(1, Math.ceil(Math.sqrt(values.length / 4))));
    const height = Math.max(1, Math.ceil(values.length / 4 / width));
    const data = new Float32Array(width * height * 4); data.set(values);
    const result = new THREE.DataTexture(data, width, height, THREE.RGBAFormat, THREE.FloatType);
    result.needsUpdate = true; return result;
}

export const columnProjectionVertexShader = /* glsl */`
attribute vec2 columnCoordinate;
attribute float columnFace;
flat varying float columnFaceId;
varying vec3 lumenPosition;
varying vec3 lumenRay;
varying vec2 column;
${deformationHooks.vertex}
void main() {
    vec3 projected = projectionVertexPosition(position);
    vec4 view = modelViewMatrix * vec4(projected, 1.);
    lumenPosition = projected;
    column = columnCoordinate;
    columnFaceId = columnFace;
    lumenRay = inverse(mat3(modelViewMatrix)) * (isOrthographic ? vec3(0.,0.,-1.) : view.xyz);
    gl_Position = projectionMatrix * view;
}`;

// One exact exit query per entry fragment. Ordinary single-vessel chords use
// the preintegrated concentration primitive. At a fork only, a bounded local
// quadrature resolves the physical parent/daughter volumes, without any
// interior geometry-BVH search.
export const columnProjectionFragmentShader = /* glsl */`
precision highp usampler2D;
precision highp isampler2D;
#define BVH_STACK_DEPTH 64
#define TRI_INTERSECT_EPSILON 1e-7
${BVHShaderGLSL.common_functions}
${BVHShaderGLSL.bvh_struct_definitions}
${deformationHooks.triangle}
${deformationHooks.bounds}
flat varying float columnFaceId;
${boundedRayFunctions.replace('uvec3 indices = uTexelFetch1D( indexAttr, i ).xyz;',
    'if (i == uint(columnFaceId + .5)) continue; uvec3 indices = uTexelFetch1D(indexAttr, i).xyz;')}
uniform BVH lumenBvh;
uniform sampler2D columnFaces;
uniform sampler2D columnRanges;
uniform sampler2D columnGeometry;
uniform sampler2D columnGeometryRanges;
uniform sampler2D columnJunctionBounds;
uniform sampler2D columnJunctionRecords;
uniform int columnJunctionNodeCount;
uniform sampler2D columnCurrent;
uniform sampler2D columnPrevious;
uniform mat4 columnInverseProjection;
uniform vec4 columnViewport;
uniform float displayAlpha;
uniform float signalGain;
uniform vec3 projectionActiveMin;
uniform vec3 projectionActiveMax;
uniform bool projectionCullActive;
uniform bool debugMode;
uniform vec3 debugColor;
varying vec3 lumenPosition;
varying vec3 lumenRay;
varying vec2 column;

vec4 columnSample(int path, float s) {
    vec4 range = texelFetch1D(columnRanges, uint(path * 2));
    int first = int(range.x), last = first + int(range.y) - 1;
    s = clamp(s, 0., range.z);
    int lo = first, hi = last;
    for (int i = 0; i < 24; i++) {
        if (hi - lo <= 1) break;
        int mid = (lo + hi) / 2;
        float x = texelFetch1D(columnCurrent, uint(mid)).x;
        if (x <= s) lo = mid; else hi = mid;
    }
    vec4 a = mix(texelFetch1D(columnPrevious, uint(lo)), texelFetch1D(columnCurrent, uint(lo)), displayAlpha);
    vec4 b = mix(texelFetch1D(columnPrevious, uint(hi)), texelFetch1D(columnCurrent, uint(hi)), displayAlpha);
    float dx = max(0., s - a.x), span = b.x - a.x;
    float c = span > 1e-8 ? mix(a.y, b.y, clamp(dx / span, 0., 1.)) : b.y;
    return vec4(max(0., c), a.z + dx * (a.y + c) * .5, s, float(lo));
}

float columnAverage(int a, float sa, int b, float sb) {
    float mass = 0., distance = 0.;
    int originalA = a, originalB = b;
    float originalSa = sa, originalSb = sb;
    // Continuous territories form a tree. At a real fork integrate each
    // incident arm to its shared node; unrelated projected arteries are
    // never paired because the geometry query stops at the first lumen exit.
    for (int i = 0; i < 64; i++) {
        if (a == b) {
            vec4 p = columnSample(a, sa), q = columnSample(b, sb);
            float ds = abs(p.z - q.z);
            if (distance + ds < .001) return columnSample(a, .5 * (p.z + q.z)).x;
            float part = p.w == q.w ? ds * .5 * (p.x + q.x) : abs(p.y - q.y);
            return max(0., (mass + part) / (distance + ds));
        }
        vec4 ra = texelFetch1D(columnRanges, uint(a * 2));
        vec4 rb = texelFetch1D(columnRanges, uint(b * 2));
        bool stepA = ra.w >= rb.w;
        int path = stepA ? a : b;
        float s = stepA ? sa : sb;
        vec4 parent = texelFetch1D(columnRanges, uint(path * 2 + 1));
        if (parent.x < 0.) break;
        vec4 p = columnSample(path, s), q = columnSample(path, 0.);
        mass += max(0., p.y - q.y); distance += p.z;
        if (stepA) { a = int(parent.x); sa = parent.y; }
        else { b = int(parent.x); sb = parent.y; }
    }
    // Disconnected graph labels may share a custom anatomical junction.
    // Do not borrow any remote root's bolus across such an unmapped seam.
    return .5 * (columnSample(originalA, originalSa).x + columnSample(originalB, originalSb).x);
}

// This hook is replaced only when device-contact deformation is attached.
vec3 columnRestPosition(vec3 point) { return point; }

vec4 columnGeometryAt(vec4 range, float s, out vec4 derivative) {
    float x = clamp(s / max(range.w, 1e-8), 0., range.y - 1.);
    int index = min(int(range.y) - 2, int(floor(x)));
    vec4 a = texelFetch1D(columnGeometry, uint(int(range.x) + index));
    vec4 b = texelFetch1D(columnGeometry, uint(int(range.x) + index + 1));
    derivative = (b - a) / max(range.w, 1e-8);
    return mix(a, b, x - float(index));
}

float columnNearestRefined(int path, vec3 point, float seed, int iterations, out float score) {
    vec4 range = texelFetch1D(columnGeometryRanges, uint(path));
    float s = clamp(seed, 0., range.z), bestS = s, bestDistance = 1e30, bestRadius = 0.;
    vec4 derivative;
    for (int i = 0; i <= 6; i++) {
        vec4 local = columnGeometryAt(range, s, derivative);
        float next = s + dot(point - local.xyz, derivative.xyz) / max(1e-8, dot(derivative.xyz, derivative.xyz));
        float segment = min(range.y - 2., floor(s / max(range.w, 1e-8)));
        float candidate = clamp(next, segment * range.w, (segment + 1.) * range.w);
        vec4 nearest = local + derivative * (candidate - s);
        float distance = dot(point - nearest.xyz, point - nearest.xyz);
        if (distance < bestDistance || (distance == bestDistance && candidate < bestS)) {
            bestDistance = distance; bestS = candidate; bestRadius = nearest.w;
        }
        float bounded = clamp(next, 0., range.z);
        if (i >= iterations || bounded == s) break;
        s = bounded;
    }
    score = sqrt(max(0., bestDistance)) - bestRadius;
    return bestS;
}

float columnNearestCoordinate(int path, vec3 point, float seed, out float score) {
    return columnNearestRefined(path, point, seed, 3, score);
}

float spatialColumnAverage(int a, float sa, int b, float sb, vec3 entry, vec3 ray, float chord) {
    int paths[16]; float coordinates[16]; int count = 0;
    paths[count] = a; coordinates[count++] = sa;
    if (b != a) { paths[count] = b; coordinates[count++] = sb; }
    // Include the common parent even if both visible walls belong to clear
    // daughters. Their intervening aortic blood remains on the detector ray.
    for (int step = 0; step < 64; step++) {
        if (a == b) break;
        vec4 ra = texelFetch1D(columnRanges, uint(a * 2));
        vec4 rb = texelFetch1D(columnRanges, uint(b * 2));
        bool stepA = ra.w >= rb.w;
        int path = stepA ? a : b;
        vec4 parent = texelFetch1D(columnRanges, uint(path * 2 + 1));
        if (parent.x < 0.) break;
        int next = int(parent.x); bool present = false;
        for (int candidate = 0; candidate < 16; candidate++) {
            if (candidate >= count) break;
            if (paths[candidate] == next) { coordinates[candidate] = .5 * (coordinates[candidate] + parent.y); present = true; break; }
        }
        if (!present) {
            // Keep the most local arms and reserve the final slot for the
            // highest ancestor if a custom model has unusually deep forks.
            int slot = min(count, 15);
            paths[slot] = next; coordinates[slot] = parent.y;
            count = min(16, count + 1);
        }
        if (stepA) { a = next; sa = parent.y; } else { b = next; sb = parent.y; }
    }
    vec3 midpoint = columnRestPosition(entry + ray * (chord * .5));
    for (int candidate = 0; candidate < 16; candidate++) {
        if (candidate >= count) break;
        float score;
        coordinates[candidate] = columnNearestCoordinate(paths[candidate], midpoint, coordinates[candidate], score);
        coordinates[candidate] = columnNearestCoordinate(paths[candidate], midpoint, coordinates[candidate], score);
    }
    float sum = 0.;
    for (int sampleIndex = 0; sampleIndex < 24; sampleIndex++) {
        vec3 point = columnRestPosition(entry + ray * (chord * (float(sampleIndex) + .5) / 24.));
        float best = 1e30, s = 0.; int owner = paths[0];
        for (int candidate = 0; candidate < 16; candidate++) {
            if (candidate >= count) break;
            float score;
            float coordinate = columnNearestCoordinate(paths[candidate], point, coordinates[candidate], score);
            if (score < best || (score == best && paths[candidate] < owner)) {
                best = score; owner = paths[candidate]; s = coordinate;
            }
        }
        sum += columnSample(owner, s).x;
    }
    return sum / 24.;
}

bool columnSegmentBox(vec3 entry, vec3 ray, float chord, vec3 low, vec3 high) {
    float first = 0., last = chord;
    for (int axis = 0; axis < 3; axis++) {
        if (abs(ray[axis]) < 1e-10) {
            if (entry[axis] < low[axis] || entry[axis] > high[axis]) return false;
        } else {
            float a = (low[axis] - entry[axis]) / ray[axis];
            float b = (high[axis] - entry[axis]) / ray[axis];
            first = max(first, min(a, b)); last = min(last, max(a, b));
            if (last < first) return false;
        }
    }
    return true;
}

bool columnJunctionConnected(vec4 record, float componentA, float componentB) {
    return record.y == componentA || record.y == componentB;
}

bool columnHasJunction(vec3 entry, vec3 ray, float chord, int a, int b) {
    float componentA = texelFetch1D(columnRanges, uint(a * 2 + 1)).z;
    float componentB = texelFetch1D(columnRanges, uint(b * 2 + 1)).z;
    int node = 0;
    // Preorder escape links only move forwards: each node is visited at most
    // once, with no stack, fixed candidate capacity or nested lumen query.
    while (node < columnJunctionNodeCount) {
        vec4 low = texelFetch1D(columnJunctionBounds, uint(node * 2));
        vec4 high = texelFetch1D(columnJunctionBounds, uint(node * 2 + 1));
        float padding = projectionBoundsPadding(low.xyz, high.xyz);
        if (!columnSegmentBox(entry, ray, chord, low.xyz - vec3(padding), high.xyz + vec3(padding))) {
            node = int(low.w); continue;
        }
        if (high.w >= 0.) {
            vec4 record = texelFetch1D(columnJunctionRecords, uint(high.w));
            if (columnJunctionConnected(record, componentA, componentB)) return true;
        }
        node++;
    }
    return false;
}

float columnJunctionTaper(vec3 point, vec3 low, vec3 high, float radius) {
    vec3 inside = min(point - low, high - point);
    float u = clamp(min(min(inside.x, inside.y), inside.z) / max(radius, 1e-8), 0., 1.);
    return u * u * (3. - 2. * u);
}

vec4 columnPathSupport(int path, vec3 point, float fallback) {
    vec4 descriptor = texelFetch1D(columnJunctionRecords, uint(path));
    float alpha = 0., firstActive = 1e30, firstDistance = 1e30, secondDistance = 1e30;
    float seed = fallback, secondSeed = fallback;
    for (int i = 0; i < int(descriptor.y); i++) {
        vec4 reference = texelFetch1D(columnJunctionRecords, uint(int(descriptor.x) + i));
        vec4 low = texelFetch1D(columnJunctionBounds, uint(int(reference.x) * 2));
        vec4 high = texelFetch1D(columnJunctionBounds, uint(int(reference.x) * 2 + 1));
        float weight = columnJunctionTaper(point, low.xyz, high.xyz, reference.w);
        if (weight > 0.) {alpha = max(alpha, weight); firstActive = min(firstActive, reference.y);}
        vec3 d = point - .5 * (low.xyz + high.xyz); float distance = dot(d, d);
        if (distance < firstDistance) {
            secondDistance = firstDistance; secondSeed = seed;
            firstDistance = distance; seed = reference.z;
        } else if (distance < secondDistance) {secondDistance = distance; secondSeed = reference.z;}
    }
    if (descriptor.y < 2.) secondSeed = seed;
    return vec4(alpha, firstActive, seed, secondSeed);
}

float columnCanonicalProjection(int path, vec3 point, vec4 support, out float coordinate, out float localRadius) {
    float score; coordinate = columnNearestRefined(path, point, support.z, 6, score);
    vec4 range = texelFetch1D(columnGeometryRanges, uint(path)), derivative;
    vec4 first = columnGeometryAt(range, coordinate, derivative); localRadius = first.w;
    if (support.w != support.z) {
        float otherScore;
        float other = columnNearestRefined(path, point, support.w, 6, otherScore);
        vec4 next = columnGeometryAt(range, other, derivative);
        float distance = dot(point - first.xyz, point - first.xyz), nextDistance = dot(point - next.xyz, point - next.xyz);
        if (nextDistance < distance || (nextDistance == distance && other < coordinate)) {coordinate = other; score = otherScore; localRadius = next.w;}
    }
    return score;
}

void columnMixCandidate(int path, float seed, bool own, int reference, vec3 point, float width, int pass,
    inout float best, inout int owner, inout float weighted, inout float total) {
    float lowerBound = -texelFetch1D(columnRanges, uint(path * 2 + 1)).w;
    if (lowerBound > best + (pass == 0 ? 0. : width)) return;
    vec4 support = columnPathSupport(path, point, seed);
    if (!own && support.y != float(reference)) return;
    float alpha = own ? 1. : support.x;
    if (alpha <= 0.) return;
    lowerBound -= width * log(alpha);
    if (lowerBound > best + (pass == 0 ? 0. : width)) return;
    float coordinate, localRadius;
    float score = columnCanonicalProjection(path, point, support, coordinate, localRadius);
    if (!own) {
        // A nearby connected artery across a wall must not lend iodine merely
        // because it lies in a broad-phase junction box.
        float fade = min(.5 * width, .2 * max(localRadius, 1e-4));
        float u = clamp(-score / fade, 0., 1.);
        alpha *= u * u * (3. - 2. * u);
    }
    if (alpha <= 0.) return;
    float effective = score - width * log(alpha);
    if (pass == 0) {
        if (effective < best || (effective == best && path < owner)) {best = effective; owner = path;}
    } else {
        float u = clamp(1. - (effective - best) / width, 0., 1.);
        float weight = u * u * (3. - 2. * u);
        if (weight > 0.) {weighted += weight * columnSample(path, coordinate).x; total += weight;}
    }
}

vec3 junctionColumnValue(vec3 point, int a, float sa, int b, float sb, float componentA, float componentB) {
    float beta = 0., localRadius = 0.; int node = 0;
    while (node < columnJunctionNodeCount) {
        vec4 low = texelFetch1D(columnJunctionBounds, uint(node * 2));
        vec4 high = texelFetch1D(columnJunctionBounds, uint(node * 2 + 1));
        if (any(lessThan(point, low.xyz)) || any(greaterThan(point, high.xyz))) {node = int(low.w); continue;}
        if (high.w >= 0.) {
            vec4 record = texelFetch1D(columnJunctionRecords, uint(high.w));
            if (columnJunctionConnected(record, componentA, componentB)) {
                float weight = columnJunctionTaper(point, low.xyz, high.xyz, record.z);
                beta = max(beta, weight); localRadius = max(localRadius, weight * record.z);
            }
        }
        node++;
    }
    // Only beta*C enters the optical correction; outside the support the
    // original analytic column is used exactly, including narrow boluses.
    if (beta <= 0.) return vec3(0., 0., float(a));
    float width = clamp(.2 * localRadius, .08, 1.2);
    float best = 1e30, weighted = 0., total = 0.; int owner = a;
    for (int pass = 0; pass < 2; pass++) {
        columnMixCandidate(a, sa, true, -1, point, width, pass, best, owner, weighted, total);
        if (b != a) columnMixCandidate(b, sb, true, -1, point, width, pass, best, owner, weighted, total);
        node = 0;
        while (node < columnJunctionNodeCount) {
            vec4 low = texelFetch1D(columnJunctionBounds, uint(node * 2));
            vec4 high = texelFetch1D(columnJunctionBounds, uint(node * 2 + 1));
            if (any(lessThan(point, low.xyz)) || any(greaterThan(point, high.xyz))) {node = int(low.w); continue;}
            if (high.w >= 0.) {
                int first = int(high.w); vec4 record = texelFetch1D(columnJunctionRecords, uint(first));
                if (columnJunctionConnected(record, componentA, componentB) && columnJunctionTaper(point, low.xyz, high.xyz, record.z) > 0.) {
                    for (int incident = 0; incident < int(record.x); incident++) {
                        int reference = first + 1 + incident;
                        vec4 candidate = texelFetch1D(columnJunctionRecords, uint(reference));
                        int path = int(candidate.x);
                        if (path == a || path == b) continue;
                        columnMixCandidate(path, candidate.y, false, reference, point, width, pass, best, owner, weighted, total);
                    }
                }
            }
            node++;
        }
    }
    return vec3(total > 0. ? weighted / total : 0., beta, float(owner));
}

float junctionColumnAverage(int a, float sa, int b, float sb, vec3 entry, vec3 ray, float chord) {
    float componentA = texelFetch1D(columnRanges, uint(a * 2 + 1)).z;
    float componentB = texelFetch1D(columnRanges, uint(b * 2 + 1)).z;
    // Keep the original wall coordinates for the analytic baseline. Local
    // spatial projection has a separate, canonical, per-territory seed.
    float mixedBaseline = -1., sum = 0.;
    vec3 previous = junctionColumnValue(columnRestPosition(entry), a, sa, b, sb, componentA, componentB);
    vec4 pendingSamples[5]; vec3 pendingStates[5]; vec2 pendingSupports[5];
    for (int sampleIndex = 1; sampleIndex <= 24; sampleIndex++) {
        float lo = float(sampleIndex - 1) / 24., hi = float(sampleIndex) / 24.;
        vec3 current = junctionColumnValue(columnRestPosition(entry + ray * (chord * hi)), a, sa, b, sb, componentA, componentB);
        pendingSamples[0] = vec4(lo, hi, previous.x, current.x);
        pendingStates[0] = vec3(previous.z, current.z, 0.); pendingSupports[0] = vec2(previous.y, current.y);
        int top = 0;
        while (top >= 0) {
            vec4 interval = pendingSamples[top]; vec3 state = pendingStates[top]; vec2 support = pendingSupports[top]; top--;
            if (state.z >= 4. || state.x == state.y || interval.z == interval.w) {
                float beta = .5 * (support.x + support.y), base = 0.;
                if (beta < 1.) {
                    if (a == b) base = columnAverage(a, mix(sa, sb, interval.x), a, mix(sa, sb, interval.y));
                    else {
                        if (mixedBaseline < 0.) mixedBaseline = spatialColumnAverage(a, sa, b, sb, entry, ray, chord);
                        base = mixedBaseline;
                    }
                }
                // This is a convex combination of an exact prefix integral
                // and a positive spatial integral. It cannot subtract iodine
                // below zero when a narrow front falls between sample points.
                sum += (interval.y - interval.x) * ((1. - beta) * base + .5 * (support.x * interval.z + support.y * interval.w));
                continue;
            }
            float middle = .5 * (interval.x + interval.y);
            vec3 value = junctionColumnValue(columnRestPosition(entry + ray * (chord * middle)), a, sa, b, sb, componentA, componentB);
            top++; pendingSamples[top] = vec4(middle, interval.y, value.x, interval.w);
            pendingStates[top] = vec3(value.z, state.y, state.z + 1.); pendingSupports[top] = vec2(value.y, support.y);
            top++; pendingSamples[top] = vec4(interval.x, middle, interval.z, value.x);
            pendingStates[top] = vec3(state.x, value.z, state.z + 1.); pendingSupports[top] = vec2(support.x, value.y);
        }
        previous = current;
    }
    return sum;
}

void main() {
    // Raster interpolants follow quantized triangle coverage, not exactly the
    // detector pixel centre. A 0.0001 mm transverse error can select the wrong
    // exit on a nearly tangent concave wall. Reconstruct the detector ray and
    // intersect it with this entry triangle's plane before traversing the BVH.
    vec2 ndc = 2. * (gl_FragCoord.xy - columnViewport.xy) / columnViewport.zw - 1.;
    vec4 nearH = columnInverseProjection * vec4(ndc, -1., 1.);
    vec4 farH = columnInverseProjection * vec4(ndc, 1., 1.);
    vec3 nearPoint = nearH.xyz / nearH.w;
    vec3 ray = normalize(farH.xyz / farH.w - nearPoint);
    uvec3 entryIndices = uTexelFetch1D(lumenBvh.index, uint(columnFaceId + .5)).xyz;
    vec3 a = texelFetch1D(lumenBvh.position, entryIndices.x).xyz;
    vec3 b = texelFetch1D(lumenBvh.position, entryIndices.y).xyz;
    vec3 c = texelFetch1D(lumenBvh.position, entryIndices.z).xyz;
    vec3 plane = normalize(cross(b - a, c - a));
    float denominator = dot(plane, ray);
    if (abs(denominator) < 1e-8) discard;
    vec3 entryPosition = nearPoint + ray * (dot(plane, a - nearPoint) / denominator);
    float activeDistance;
    float padding = projectionBoundsPadding(projectionActiveMin, projectionActiveMax);
    if (projectionCullActive && !intersectsBounds(entryPosition, ray,
        projectionActiveMin - vec3(padding), projectionActiveMax + vec3(padding), activeDistance)) discard;
    float epsilon = max(.0001, max(max(abs(lumenPosition.x),abs(lumenPosition.y)),abs(lumenPosition.z)) * 2e-7);
    uvec4 face = uvec4(0u); vec3 normal = vec3(0.), bary = vec3(0.);
    float side = 0., distance = 0.;
    if (!bvhIntersectFirstHit(lumenBvh, entryPosition + ray * epsilon, ray, face, normal, bary, side, distance)) discard;
    vec2 entryColumn = column;
    // Fixed-point raster coverage can select the triangle immediately beside
    // the true entry at a shared edge. Resolve that tiny entry discrepancy;
    // do not follow a distant, unrelated lumen if raster coverage was outside.
    for (int retry = 0; retry < 2; retry++) {
        if (side >= 0. || distance > max(.002, epsilon * 4.)) break;
        entryPosition += ray * (distance + epsilon);
        vec4 actualEntry = texelFetch1D(columnFaces, face.w);
        entryColumn = vec2(actualEntry.w, dot(actualEntry.xyz, bary));
        distance = 0.;
        if (!bvhIntersectFirstHit(lumenBvh, entryPosition + ray * epsilon, ray, face, normal, bary, side, distance)) discard;
    }
    if (side < 0. || distance <= 0.) discard;
    vec4 exitColumn = texelFetch1D(columnFaces, face.w);
    int entryPath = int(entryColumn.x + .5), exitPath = int(exitColumn.w + .5);
    float exitS = dot(exitColumn.xyz, bary);
    float chord = distance + epsilon;
    float mean;
    if (columnHasJunction(entryPosition, ray, chord, entryPath, exitPath))
        mean = junctionColumnAverage(entryPath, entryColumn.y, exitPath, exitS, entryPosition, ray, chord);
    else mean = entryPath == exitPath
        ? columnAverage(entryPath, entryColumn.y, exitPath, exitS)
        : spatialColumnAverage(entryPath, entryColumn.y, exitPath, exitS, entryPosition, ray, chord);
    float depth = .5 * (distance + epsilon) * mean * signalGain;
    if (depth <= 0.) discard;
    gl_FragColor = vec4((debugMode ? debugColor : vec3(1.)) * depth, depth);
}`;

export function buildAnatomicalColumnProjection(network, geometry, field, {territories, continuations}) {
    const integral = createFlowColumnIntegral(network, field, territories, continuations);
    const spatial = createFlowColumnGeometry(network, integral, territories, continuations);
    const junctions = createFlowJunctionProjection(network, integral, territories, spatial);
    const indexed = createProjectionIndexedGeometry(geometry); indexed.clearGroups();
    // Indirect ordering preserves the triangle-to-flow metadata. The texture
    // is explicitly reordered to the BVH's GPU face indices below.
    const tree = new MeshBVH(indexed, {maxLeafTris: 8, indirect: true});
    const bvh = new MeshBVHUniformStruct(); bvh.updateFrom(tree);
    const cache = createProjectionDeformationCache(tree, indexed, bvh);
    const coordinate = geometry.getAttribute('columnCoordinate');
    const triangles = indexed.index.count / 3, faces = new Float32Array(triangles * 4);
    const entryFaces = new Float32Array(geometry.getAttribute('position').count);
    for (let i = 0; i < triangles; i++) {
        const source = tree.resolveTriangleIndex(i) * 3;
        entryFaces.fill(i, source, source + 3);
        const first = geometry.index ? geometry.index.getX(source) : source;
        for (let k = 0; k < 3; k++) {
            const vertex = geometry.index ? geometry.index.getX(source + k) : source + k;
            faces[i * 4 + k] = coordinate.getY(vertex);
        }
        faces[i * 4 + 3] = coordinate.getX(first);
    }
    geometry.setAttribute('columnFace', new THREE.BufferAttribute(entryFaces, 1));
    const ranges = new Float32Array(network.edges.length * 8), depths = new Int32Array(network.edges.length).fill(-1);
    const depth = path => {
        if (depths[path] >= 0) return depths[path];
        const parent = network.edges[path].parentEdgeIndex;
        return depths[path] = parent < 0 || territories[parent] < 0 ? 0 : depth(territories[parent]) + 1;
    };
    for (const [path, range] of integral.ranges) {
        const parent = network.edges[path].parentEdgeIndex;
        ranges.set([range.start, range.count, range.length, depth(path), parent < 0 ? -1 : territories[parent],
            parent < 0 ? 0 : integral.edgeStart[parent] + network.edges[parent].length, junctions.components[path], junctions.maxRadii[path]], path * 8);
    }
    const spatialRanges = new Float32Array(network.edges.length * 4);
    for (const [path, range] of spatial.ranges)
        spatialRanges.set([range.start, range.count, range.length, range.step], path * 4);
    const faceTexture = texture(faces), rangeTexture = texture(ranges), geometryRangeTexture = texture(spatialRanges);
    const textures = [bvh.index,bvh.position,bvh.bvhBounds,bvh.bvhContents,faceTexture,rangeTexture,
        integral.currentTexture, integral.previousTexture, spatial.texture, geometryRangeTexture,
        junctions.boundsTexture, junctions.recordsTexture];
    const inverseProjection = new THREE.Matrix4(), viewport = new THREE.Vector4();
    return {
        install(mesh) {
            const before = mesh.onBeforeRender;
            mesh.onBeforeRender = function(renderer, scene, camera, renderGeometry, material, group) {
                before.call(this, renderer, scene, camera, renderGeometry, material, group);
                inverseProjection.multiplyMatrices(camera.projectionMatrix, camera.matrixWorldInverse)
                    .multiply(mesh.matrixWorld).invert();
                renderer.getCurrentViewport(viewport);
            };
        },
        vertexShader: columnProjectionVertexShader, fragmentShader: columnProjectionFragmentShader,
        uniforms: {lumenBvh: {value: bvh}, columnFaces: {value: faceTexture}, columnRanges: {value: rangeTexture},
            columnCurrent: {value: integral.currentTexture}, columnPrevious: {value: integral.previousTexture},
            columnGeometry: {value: spatial.texture}, columnGeometryRanges: {value: geometryRangeTexture},
            columnJunctionBounds: {value: junctions.boundsTexture}, columnJunctionRecords: {value: junctions.recordsTexture},
            columnJunctionNodeCount: {value: junctions.nodes.length},
            columnInverseProjection: {value: inverseProjection}, columnViewport: {value: viewport}},
        update(current, previous) { integral.update(current, previous); },
        attachVesselDeformation(deformation, material) {
            attachAnatomicalProjectionDeformation(deformation, material); cache.attach(deformation);
            if (!material.userData.columnSpatialDeformationAttached) {
                // The shared attachment already declares these bounds. Keep
                // contact sampling outside the geometry traversal and invert
                // the small deformation once per spatial quadrature point.
                material.fragmentShader = vesselDeformationGLSL + material.fragmentShader
                    .replace(/uniform (?:vec3 vesselDeformMin|vec3 vesselDeformMax|bool vesselDeformActive);/g, '')
                    .replace(
                    'vec3 columnRestPosition(vec3 point) { return point; }',
                    'vec3 columnRestPosition(vec3 point) { vec3 rest = point - vesselDisplacement(point); return point - vesselDisplacement(rest); }');
                material.userData.columnSpatialDeformationAttached = true;
                material.needsUpdate = true;
            }
        },
        deformationDiagnostics: cache.diagnostics,
        memoryBytes: textures.reduce((n,t) => n + t.image.data.byteLength, 0),
        dispose() { cache.dispose(); bvh.dispose(); indexed.dispose(); faceTexture.dispose(); rangeTexture.dispose(); geometryRangeTexture.dispose(); junctions.dispose(); spatial.dispose(); integral.dispose(); }
    };
}
