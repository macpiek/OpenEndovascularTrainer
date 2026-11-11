import * as THREE from 'three';
import {MeshBVH, MeshBVHUniformStruct, BVHShaderGLSL} from 'three-mesh-bvh';
import {createProjectionIndexedGeometry} from './projectionIndexedGeometry.js';
import {createFlowColumnIntegral} from './flowColumnIntegral.js';
import {createFlowColumnLookup} from './flowColumnLookup.js';
import {createFlowColumnGeometry} from './flowColumnGeometry.js';
import {createFlowJunctionProjection} from './flowJunctionProjection.js';
import {columnJunctionMixingGLSL} from './columnJunctionMixing.js';
import {attachColumnRasterPosition} from './columnRasterPosition.js';
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
uniform int columnLookupOffset;
uniform sampler2D columnGeometry;
uniform sampler2D columnGeometryRanges;
uniform sampler2D columnJunctionBounds;
uniform sampler2D columnJunctionRecords;
uniform int columnJunctionNodeCount;
uniform int columnRayNodeOffset;
uniform int columnRayNodeCount;
uniform int columnRayLeafStride;
uniform bool columnRayCacheEnabled;
uniform sampler2D columnCurrent;
uniform sampler2D columnPrevious;
uniform mat4 columnInverseProjection;
uniform mat4 columnViewToLocal;
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
bool columnRayCacheValid = false;
int columnRayJunctions[16];
int columnRayJunctionCount = 0;
int columnRayPaths[16];
int columnRayPathCount = 0;

vec4 columnHistory(uint index) {
    // Synchronized frames normally use one exact history endpoint. Avoid
    // reading the unused texture while retaining the original interpolation
    // for intermediate (or extrapolated) display phases.
    if (displayAlpha == 1.) return texelFetch1D(columnCurrent, index);
    if (displayAlpha == 0.) return texelFetch1D(columnPrevious, index);
    return mix(texelFetch1D(columnPrevious, index), texelFetch1D(columnCurrent, index), displayAlpha);
}

vec4 columnSample(int path, float s) {
    vec4 range = texelFetch1D(columnRanges, uint(path * 2));
    int first = int(range.x), last = first + int(range.y) - 1;
    s = clamp(s, 0., range.z);
    vec4 lookup = texelFetch1D(columnRanges, uint(columnLookupOffset + path));
    int bucket = clamp(int(s / max(lookup.z, 1e-8)), 0, int(lookup.y) - 1);
    vec4 bracket = texelFetch1D(columnRanges, uint(columnLookupOffset + int(lookup.x) + bucket));
    int lo = int(bracket.x), hi = int(bracket.y);
    for (int i = 0; i < 24; i++) {
        if (hi - lo <= 1) break;
        int mid = (lo + hi) / 2;
        float x = texelFetch1D(columnCurrent, uint(mid)).x;
        if (x <= s) lo = mid; else hi = mid;
    }
    vec4 a = columnHistory(uint(lo));
    vec4 b = columnHistory(uint(hi));
    float dx = max(0., s - a.x), span = b.x - a.x;
    float c = span > 1e-8 ? mix(a.y, b.y, clamp(dx / span, 0., 1.)) : b.y;
    return vec4(max(0., c), a.z + dx * (a.y + c) * .5, s, float(lo));
}

${columnJunctionMixingGLSL}

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
float columnRayDeformationPadding() { return 0.; }

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
    vec4 derivative, cachedA, cachedB;
    int cachedIndex = -1;
    for (int i = 0; i <= 6; i++) {
        // Repeated Newton iterates often remain on the same atlas segment.
        // Reuse its immutable texels and derivative, but recompute the exact
        // interpolation and every Newton step (including corner oscillation).
        float x = clamp(s / max(range.w, 1e-8), 0., range.y - 1.);
        int index = min(int(range.y) - 2, int(floor(x)));
        if (index != cachedIndex) {
            cachedA = texelFetch1D(columnGeometry, uint(int(range.x) + index));
            cachedB = texelFetch1D(columnGeometry, uint(int(range.x) + index + 1));
            derivative = (cachedB - cachedA) / max(range.w, 1e-8);
            cachedIndex = index;
        }
        vec4 local = mix(cachedA, cachedB, x - float(index));
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
    columnRayCacheValid = false; columnRayJunctionCount = 0; columnRayPathCount = 0;
    float componentA = texelFetch1D(columnRanges, uint(a * 2 + 1)).z;
    float componentB = texelFetch1D(columnRanges, uint(b * 2 + 1)).z;
    float padding = columnRayDeformationPadding();
    int node = 0;
    // Preorder escape links only move forwards: each node is visited at most
    // once, with no stack, fixed candidate capacity or nested lumen query.
    while (node < columnJunctionNodeCount) {
        vec4 low = texelFetch1D(columnJunctionBounds, uint(node * 2));
        vec4 high = texelFetch1D(columnJunctionBounds, uint(node * 2 + 1));
        if (!columnSegmentBox(entry, ray, chord, low.xyz - vec3(padding), high.xyz + vec3(padding))) {
            node = int(low.w); continue;
        }
        if (high.w >= 0.) {
            vec4 record = texelFetch1D(columnJunctionRecords, uint(high.w));
            if (columnJunctionConnected(record, componentA, componentB)) {
                if (!columnRayCacheEnabled || columnRayJunctionCount == 16) return true;
                columnRayJunctions[columnRayJunctionCount++] = node;
            }
        }
        node++;
    }
    if (columnRayJunctionCount == 0) return false;
    node = 0;
    while (node < columnRayNodeCount) {
        vec4 low = texelFetch1D(columnJunctionRecords, uint(columnRayNodeOffset + node * 2));
        vec4 high = texelFetch1D(columnJunctionRecords, uint(columnRayNodeOffset + node * 2 + 1));
        if (!columnSegmentBox(entry, ray, chord, low.xyz - vec3(padding), high.xyz + vec3(padding))) {node = int(low.w); continue;}
        if (high.w >= 0.) {
            int packed = int(high.w), first = packed / columnRayLeafStride, count = packed - first * columnRayLeafStride;
            for (int i = 0; i < count; i++) {
                int scalar = first + i;
                vec4 paths = texelFetch1D(columnJunctionRecords, uint(columnRayNodeOffset + columnRayNodeCount * 2 + scalar / 4));
                int path = int(paths[scalar - (scalar / 4) * 4]);
                if (path == a || path == b) continue;
                float component = texelFetch1D(columnRanges, uint(path * 2 + 1)).z;
                if (component != componentA && component != componentB) continue;
                bool present = false;
                for (int k = 0; k < 16; k++) {if (k >= columnRayPathCount) break; if (columnRayPaths[k] == path) {present = true; break;}}
                if (present) continue;
                if (columnRayPathCount == 16) return true;
                columnRayPaths[columnRayPathCount++] = path;
            }
        }
        node++;
    }
    columnRayCacheValid = true;
    return true;
}

bool columnRayContainsPath(int path) {
    if (!columnRayCacheValid) return true;
    for (int i = 0; i < 16; i++) {if (i >= columnRayPathCount) break; if (columnRayPaths[i] == path) return true;}
    return false;
}

bool columnPathConstant(int path, out float value) {
    if (displayAlpha < 0. || displayAlpha > 1.) return false;
    int first = int(texelFetch1D(columnRanges, uint(path * 2)).x);
    if (displayAlpha == 1.) {
        float minimum = texelFetch1D(columnCurrent, uint(first)).w;
        if (minimum != texelFetch1D(columnCurrent, uint(first + 1)).w) return false;
        value = minimum; return true;
    }
    if (displayAlpha == 0.) {
        float minimum = texelFetch1D(columnPrevious, uint(first)).w;
        if (minimum != texelFetch1D(columnPrevious, uint(first + 1)).w) return false;
        value = minimum; return true;
    }
    float currentMin = texelFetch1D(columnCurrent, uint(first)).w;
    float currentMax = texelFetch1D(columnCurrent, uint(first + 1)).w;
    float previousMin = texelFetch1D(columnPrevious, uint(first)).w;
    float previousMax = texelFetch1D(columnPrevious, uint(first + 1)).w;
    if ((displayAlpha > 0. && currentMin != currentMax) || (displayAlpha < 1. && previousMin != previousMax)) return false;
    value = mix(previousMin, currentMin, displayAlpha);
    return true;
}

bool columnUniformRay(int a, int b, out float value) {
    if (!columnRayCacheValid || !columnPathConstant(a, value)) return false;
    float other;
    if (!columnPathConstant(b, other) || other != value) return false;
    for (int i = 0; i < 16; i++) {
        if (i >= columnRayPathCount) break;
        if (!columnPathConstant(columnRayPaths[i], other) || other != value) return false;
    }
    // A common-lumen field also samples the other arms and the actual mixed
    // node. They need not physically intersect this ray, but must agree before
    // its quadrature can be replaced by an exact constant value.
    for (int i = 0; i < 16; i++) {
        if (i >= columnRayJunctionCount) break;
        vec4 high = texelFetch1D(columnJunctionBounds, uint(columnRayJunctions[i] * 2 + 1));
        int shared = int(texelFetch1D(columnJunctionRecords, uint(high.w)).w) - 1;
        if (shared < 0 || !sharedJunctionHasPath(shared, a, b)) continue;
        if (sharedNodeConcentration(shared) != value) return false;
        for (int arm = 0; arm < 3; arm++) {
            int path = int(sharedRecord( uint(shared * 12 + 3 + arm * 3)).w);
            if (!columnPathConstant(path, other) || other != value) return false;
        }
    }
    // Mixed-wall support skirts retain the original graph-route baseline.
    // Ancestors may enter that baseline even if their physical tube misses
    // this chord, so they must also satisfy the constant-field proof.
    for (int i = 0; i < 64; i++) {
        if (a == b) return true;
        vec4 ra = texelFetch1D(columnRanges, uint(a * 2)), rb = texelFetch1D(columnRanges, uint(b * 2));
        bool stepA = ra.w >= rb.w;
        vec4 parent = texelFetch1D(columnRanges, uint((stepA ? a : b) * 2 + 1));
        if (parent.x < 0.) return false;
        int next = int(parent.x);
        if (!columnPathConstant(next, other) || other != value) return false;
        if (stepA) a = next; else b = next;
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

bool columnEvaluateMixCandidate(int path, float seed, bool own, int reference, vec3 point, float width,
    float margin, float best, out vec3 candidate) {
    float lowerBound = -texelFetch1D(columnRanges, uint(path * 2 + 1)).w;
    if (lowerBound > best + margin) return false;
    vec4 support = columnPathSupport(path, point, seed);
    if (!own && support.y != float(reference)) return false;
    float alpha = own ? 1. : support.x;
    if (alpha <= 0.) return false;
    if (!own) lowerBound -= width * log(alpha);
    if (lowerBound > best + margin) return false;
    float coordinate, localRadius;
    float score = columnCanonicalProjection(path, point, support, coordinate, localRadius);
    if (!own) {
        // A nearby connected artery across a wall must not lend iodine merely
        // because it lies in a broad-phase junction box.
        float fade = min(.5 * width, .2 * max(localRadius, 1e-4));
        float u = clamp(-score / fade, 0., 1.);
        alpha *= u * u * (3. - 2. * u);
    }
    if (alpha <= 0.) return false;
    float effective = own ? score : score - width * log(alpha);
    candidate = vec3(effective, coordinate, float(path));
    return true;
}

void columnMixCandidate(int path, float seed, bool own, int reference, vec3 point, float width, int pass,
    inout float best, inout int owner, inout float weighted, inout float total) {
    vec3 candidate;
    if (!columnEvaluateMixCandidate(path, seed, own, reference, point, width, pass == 0 ? 0. : width, best, candidate)) return;
    float effective = candidate.x, coordinate = candidate.y;
    if (pass == 0) {
        if (effective < best || (effective == best && path < owner)) {best = effective; owner = path;}
    } else {
        float u = clamp(1. - (effective - best) / width, 0., 1.);
        float weight = u * u * (3. - 2. * u);
        if (weight > 0.) {weighted += weight * columnSample(path, coordinate).x; total += weight;}
    }
}

vec3 junctionColumnValueUncached(vec3 point, int a, float sa, int b, float sb, float componentA, float componentB) {
    float beta = 0., localRadius = 0., sharedWeighted = 0., sharedTotal = 0., remaining = 1.; int node = 0;
    while (node < columnJunctionNodeCount) {
        vec4 low = texelFetch1D(columnJunctionBounds, uint(node * 2));
        vec4 high = texelFetch1D(columnJunctionBounds, uint(node * 2 + 1));
        if (any(lessThan(point, low.xyz)) || any(greaterThan(point, high.xyz))) {node = int(low.w); continue;}
        if (high.w >= 0.) {
            vec4 record = texelFetch1D(columnJunctionRecords, uint(high.w));
            if (columnJunctionConnected(record, componentA, componentB)) {
                float weight = columnJunctionTaper(point, low.xyz, high.xyz, record.z);
                beta = max(beta, weight); localRadius = max(localRadius, weight * record.z);
                int shared = int(record.w) - 1;
                if (weight > 0. && shared >= 0 && sharedJunctionHasPath(shared, a, b)) {
                    vec2 mixedValue = sharedJunctionConcentration(shared, point);
                    float amount = weight * mixedValue.y;
                    sharedWeighted += amount * mixedValue.x; sharedTotal += amount; remaining *= 1. - amount;
                }
            }
        }
        node++;
    }
    // Only beta*C enters the optical correction; outside the support the
    // original analytic column is used exactly, including narrow boluses.
    if (beta <= 0.) return vec3(0., 0., float(a));
    if (remaining == 0.) return vec3(sharedWeighted / sharedTotal, beta, -1.);
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
    float baseline = total > 0. ? weighted / total : 0.;
    float value = sharedTotal > 0. ? mix(sharedWeighted / sharedTotal, baseline, remaining) : baseline;
    return vec3(value, beta, float(owner));
}

vec3 junctionColumnValue(vec3 point, int a, float sa, int b, float sb, float componentA, float componentB) {
    float beta = 0., localRadius = 0., sharedWeighted = 0., sharedTotal = 0., remaining = 1.; int node = 0, cursor = 0;
    while (columnRayCacheValid ? cursor < columnRayJunctionCount : node < columnJunctionNodeCount) {
        if (columnRayCacheValid) node = columnRayJunctions[cursor++];
        vec4 low = texelFetch1D(columnJunctionBounds, uint(node * 2));
        vec4 high = texelFetch1D(columnJunctionBounds, uint(node * 2 + 1));
        if (any(lessThan(point, low.xyz)) || any(greaterThan(point, high.xyz))) {node = int(low.w); continue;}
        if (high.w >= 0.) {
            vec4 record = texelFetch1D(columnJunctionRecords, uint(high.w));
            if (columnJunctionConnected(record, componentA, componentB)) {
                float weight = columnJunctionTaper(point, low.xyz, high.xyz, record.z);
                beta = max(beta, weight); localRadius = max(localRadius, weight * record.z);
                int shared = int(record.w) - 1;
                if (weight > 0. && shared >= 0 && sharedJunctionHasPath(shared, a, b)) {
                    vec2 mixedValue = sharedJunctionConcentration(shared, point);
                    float amount = weight * mixedValue.y;
                    sharedWeighted += amount * mixedValue.x; sharedTotal += amount; remaining *= 1. - amount;
                }
            }
        }
        node++;
    }
    if (beta <= 0.) return vec3(0., 0., float(a));
    if (remaining == 0.) return vec3(sharedWeighted / sharedTotal, beta, -1.);
    float width = clamp(.2 * localRadius, .08, 1.2), best = 1e30;
    vec3 candidates[4], candidate; int count = 0, owner = a;
    // Geometry and its canonical arc coordinate do not depend on iodine or
    // on the eventual minimum score. Retain them once for the weighting pass.
    // The width margin preserves candidates which cannot win but still mix.
    if (columnEvaluateMixCandidate(a, sa, true, -1, point, width, width, best, candidate)) {
        candidates[count++] = candidate; best = candidate.x; owner = a;
    }
    if (b != a && columnEvaluateMixCandidate(b, sb, true, -1, point, width, width, best, candidate)) {
        candidates[count++] = candidate;
        if (candidate.x < best || (candidate.x == best && b < owner)) {best = candidate.x; owner = b;}
    }
    node = 0; cursor = 0;
    while (columnRayCacheValid ? cursor < columnRayJunctionCount : node < columnJunctionNodeCount) {
        if (columnRayCacheValid) node = columnRayJunctions[cursor++];
        vec4 low = texelFetch1D(columnJunctionBounds, uint(node * 2));
        vec4 high = texelFetch1D(columnJunctionBounds, uint(node * 2 + 1));
        if (any(lessThan(point, low.xyz)) || any(greaterThan(point, high.xyz))) {node = int(low.w); continue;}
        if (high.w >= 0.) {
            int first = int(high.w); vec4 record = texelFetch1D(columnJunctionRecords, uint(first));
            if (columnJunctionConnected(record, componentA, componentB) && columnJunctionTaper(point, low.xyz, high.xyz, record.z) > 0.) {
                for (int incident = 0; incident < int(record.x); incident++) {
                    int reference = first + 1 + incident;
                    vec4 arm = texelFetch1D(columnJunctionRecords, uint(reference));
                    int path = int(arm.x);
                    if (path == a || path == b) continue;
                    if (!columnRayContainsPath(path)) continue;
                    if (!columnEvaluateMixCandidate(path, arm.y, false, reference, point, width, width, best, candidate)) continue;
                    // Capacity bounds register/storage cost, never anatomy.
                    // Custom high-degree or overlapping forks use the exact
                    // original uncapped traversal rather than lose any arm.
                    if (count == 4) return junctionColumnValueUncached(point, a, sa, b, sb, componentA, componentB);
                    candidates[count++] = candidate;
                    if (candidate.x < best || (candidate.x == best && path < owner)) {best = candidate.x; owner = path;}
                }
            }
        }
        node++;
    }
    float weighted = 0., total = 0.;
    for (int i = 0; i < 4; i++) {
        if (i >= count) break;
        candidate = candidates[i];
        float u = clamp(1. - (candidate.x - best) / width, 0., 1.);
        float weight = u * u * (3. - 2. * u);
        if (weight > 0.) {weighted += weight * columnSample(int(candidate.z), candidate.y).x; total += weight;}
    }
    float baseline = total > 0. ? weighted / total : 0.;
    float value = sharedTotal > 0. ? mix(sharedWeighted / sharedTotal, baseline, remaining) : baseline;
    return vec3(value, beta, float(owner));
}

float junctionColumnAverage(int a, float sa, int b, float sb, vec3 entry, vec3 ray, float chord) {
    float componentA = texelFetch1D(columnRanges, uint(a * 2 + 1)).z;
    float componentB = texelFetch1D(columnRanges, uint(b * 2 + 1)).z;
    float mixedBaseline = -1., sum = 0.;
    vec3 previous = junctionColumnValue(columnRestPosition(entry), a, sa, b, sb, componentA, componentB);
    // Depth-first integration only needs the postponed RIGHT endpoints.
    // The completed left endpoint is shared with the next interval. Keep
    // four vec4s instead of three five-element arrays; the 3-bit depths fit
    // in one uint. Sample positions, visit order and arithmetic are unchanged.
    vec4 pendingRight[4];
    for (int sampleIndex = 1; sampleIndex <= 24; sampleIndex++) {
        float lo = float(sampleIndex - 1) / 24., hi = float(sampleIndex) / 24.;
        vec3 current = junctionColumnValue(columnRestPosition(entry + ray * (chord * hi)), a, sa, b, sb, componentA, componentB);
        vec3 left = previous, right = current;
        int depth = 0, top = 0; uint pendingDepths = 0u;
        while (true) {
            if (depth >= 4 || left.z == right.z || left.x == right.x) {
                float beta = .5 * (left.y + right.y), base = 0.;
                if (beta < 1.) {
                    if (a == b) base = columnAverage(a, mix(sa, sb, lo), a, mix(sa, sb, hi));
                    else {
                        if (mixedBaseline < 0.) mixedBaseline = spatialColumnAverage(a, sa, b, sb, entry, ray, chord);
                        base = mixedBaseline;
                    }
                }
                sum += (hi - lo) * ((1. - beta) * base + .5 * (left.y * left.x + right.y * right.x));
                if (top == 0) break;
                left = right; lo = hi;
                vec4 pending = pendingRight[--top]; right = pending.xyz; hi = pending.w;
                depth = int(pendingDepths & 7u); pendingDepths >>= 3;
                continue;
            }
            float middle = .5 * (lo + hi);
            vec3 value = junctionColumnValue(columnRestPosition(entry + ray * (chord * middle)), a, sa, b, sb, componentA, componentB);
            pendingRight[top++] = vec4(right, hi);
            pendingDepths = (pendingDepths << 3) | uint(depth + 1);
            hi = middle; right = value; depth++;
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
    // Keep the projective divide in camera space. Combining camera translation
    // with inverse projection cancels large float32 coefficients at the far
    // plane, shifting rays across tiny entry triangles and leaving pinholes.
    vec3 nearView = nearH.xyz / nearH.w;
    vec3 nearPoint = (columnViewToLocal * vec4(nearView, 1.)).xyz;
    vec3 ray = normalize(mat3(columnViewToLocal) * (farH.xyz / farH.w - nearView));
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
    if (columnHasJunction(entryPosition, ray, chord, entryPath, exitPath)) {
        if (!columnUniformRay(entryPath, exitPath, mean))
            mean = junctionColumnAverage(entryPath, entryColumn.y, exitPath, exitS, entryPosition, ray, chord);
    }
    else mean = entryPath == exitPath
        ? columnAverage(entryPath, entryColumn.y, exitPath, exitS)
        : spatialColumnAverage(entryPath, entryColumn.y, exitPath, exitS, entryPosition, ray, chord);
    float depth = .5 * (distance + epsilon) * mean * signalGain;
    if (depth <= 0.) discard;
    gl_FragColor = vec4((debugMode ? debugColor : vec3(1.)) * depth, depth);
}`;

export function buildAnatomicalColumnProjection(network, geometry, field, {territories, continuations, junctionField = null}) {
    const integral = createFlowColumnIntegral(network, field, territories, continuations);
    const spatial = createFlowColumnGeometry(network, integral, territories, continuations);
    const junctions = createFlowJunctionProjection(network, integral, territories, spatial, junctionField);
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
    const lookup = createFlowColumnLookup(integral, network.edges.length), lookupOffset = ranges.length / 4;
    const rangeRecords = new Float32Array(ranges.length + lookup.data.length);
    rangeRecords.set(ranges); rangeRecords.set(lookup.data, ranges.length);
    const faceTexture = texture(faces), rangeTexture = texture(rangeRecords), geometryRangeTexture = texture(spatialRanges);
    const textures = [bvh.index,bvh.position,bvh.bvhBounds,bvh.bvhContents,faceTexture,rangeTexture,
        integral.currentTexture, integral.previousTexture, spatial.texture, geometryRangeTexture,
        junctions.boundsTexture, junctions.recordsTexture, junctions.shared.history];
    const inverseProjection = new THREE.Matrix4(), viewToLocal = new THREE.Matrix4(), viewport = new THREE.Vector4();
    return {
        install(mesh) {
            const before = mesh.onBeforeRender;
            mesh.onBeforeRender = function(renderer, scene, camera, renderGeometry, material, group) {
                before.call(this, renderer, scene, camera, renderGeometry, material, group);
                inverseProjection.copy(camera.projectionMatrixInverse);
                viewToLocal.multiplyMatrices(camera.matrixWorldInverse, mesh.matrixWorld).invert();
                renderer.getCurrentViewport(viewport);
            };
        },
        vertexShader: columnProjectionVertexShader, fragmentShader: columnProjectionFragmentShader,
        uniforms: {lumenBvh: {value: bvh}, columnFaces: {value: faceTexture}, columnRanges: {value: rangeTexture},
            columnLookupOffset: {value: lookupOffset},
            columnCurrent: {value: integral.currentTexture}, columnPrevious: {value: integral.previousTexture},
            columnGeometry: {value: spatial.texture}, columnGeometryRanges: {value: geometryRangeTexture},
            columnJunctionBounds: {value: junctions.boundsTexture}, columnJunctionRecords: {value: junctions.recordsTexture},
            columnJunctionNodeCount: {value: junctions.nodes.length},
            columnSharedOffset: {value: junctions.sharedOffset}, columnSharedHistory: {value: junctions.shared.history},
            columnRayNodeOffset: {value: junctions.rayNodeOffset}, columnRayNodeCount: {value: junctions.rayCandidates.nodeCount},
            columnRayLeafStride: {value: junctions.rayCandidates.leafSize + 1}, columnRayCacheEnabled: {value: true},
            columnInverseProjection: {value: inverseProjection}, columnViewToLocal: {value: viewToLocal},
            columnViewport: {value: viewport}},
        update(current, previous) { integral.update(current, previous); junctions.shared.update(current, previous); },
        attachVesselDeformation(deformation, material) {
            attachAnatomicalProjectionDeformation(deformation, material); cache.attach(deformation);
            attachColumnRasterPosition(material, geometry);
            if (!material.userData.columnSpatialDeformationAttached) {
                // The shared attachment already declares these bounds. Keep
                // contact sampling outside the geometry traversal and invert
                // the small deformation once per spatial quadrature point.
                material.fragmentShader = vesselDeformationGLSL + material.fragmentShader
                    .replace(/uniform (?:vec3 vesselDeformMin|vec3 vesselDeformMax|bool vesselDeformActive);/g, '')
                    .replace(
                    'vec3 columnRestPosition(vec3 point) { return point; }',
                    'vec3 columnRestPosition(vec3 point) { vec3 rest = point - vesselDisplacement(point); return point - vesselDisplacement(rest); }')
                    .replace('float columnRayDeformationPadding() { return 0.; }',
                        'float columnRayDeformationPadding() { return vesselDeformActive ? vesselDeformMaxDisplacement + .0001 : 0.; }');
                material.userData.columnSpatialDeformationAttached = true;
                material.needsUpdate = true;
            }
        },
        deformationDiagnostics: cache.diagnostics,
        memoryBytes: textures.reduce((n,t) => n + t.image.data.byteLength, 0),
        dispose() { cache.dispose(); bvh.dispose(); indexed.dispose(); faceTexture.dispose(); rangeTexture.dispose(); geometryRangeTexture.dispose(); junctions.dispose(); spatial.dispose(); integral.dispose(); }
    };
}
