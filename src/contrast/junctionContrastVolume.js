import * as THREE from 'three';

const STEPS = 6;
const vertexShader = `
varying vec3 rayOffset;
varying vec3 rayDirection;
varying vec2 coordinate;
uniform float radius;
void main() {
    coordinate = position.xy;
    vec2 offset = coordinate * radius;
    rayOffset = vec3(dot(modelViewMatrix[0].xy, offset), dot(modelViewMatrix[1].xy, offset), dot(modelViewMatrix[2].xy, offset));
    rayDirection = vec3(modelViewMatrix[0].z, modelViewMatrix[1].z, modelViewMatrix[2].z);
    vec4 center = modelViewMatrix * vec4(0., 0., 0., 1.);
    center.xy += offset;
    gl_Position = projectionMatrix * center;
}`;
const fragmentShader = `
varying vec3 rayOffset;
varying vec3 rayDirection;
varying vec2 coordinate;
uniform vec4 starts[18];
uniform vec4 ends[18];
uniform vec2 concentration[18];
uniform vec2 previousConcentration[18];
uniform float radius;
uniform float displayAlpha;
uniform float signalGain;
uniform bool debugMode;
uniform vec3 debugColor;
void main() {
    float r2 = dot(coordinate, coordinate);
    if(r2 >= 1.) discard;
    float halfChord = radius * sqrt(1. - r2);
    float ds = 2. * halfChord / 32.;
    float depth = 0.;
    for(int step = 0; step < 32; step++) {
        vec3 p = rayOffset + rayDirection * (-halfChord + (float(step) + .5) * ds);
        float field = 0.;
        for(int i = 0; i < 18; i++) {
            vec3 axis = ends[i].xyz - starts[i].xyz;
            float t = clamp(dot(p - starts[i].xyz, axis) / max(.000001, dot(axis, axis)), 0., 1.);
            float r = mix(starts[i].w, ends[i].w, t);
            float radial2 = dot(p - mix(starts[i].xyz, ends[i].xyz, t), p - mix(starts[i].xyz, ends[i].xyz, t)) / (r*r);
            if(radial2 < 1.) {
                vec2 c = mix(previousConcentration[i], concentration[i], displayAlpha);
                // Match the filled-kernel radial density, but count shared
                // blood only once, independent of limb ordering or direction.
                float density = 1.5 * sqrt(1. - radial2);
                field = max(field, mix(c.x, c.y, t) * density);
            }
        }
        float blend = 1. - smoothstep(.55 * radius, radius, length(p));
        depth += .5 * field * blend * ds;
    }
    depth *= signalGain;
    gl_FragColor = vec4((debugMode ? debugColor : vec3(1.)) * depth, depth);
}`;

function walk(edges, first, upstream, reach) {
    const path = [];
    let edge = first,
        length = 0;
    while (edge && length < reach) {
        path.push({ edge, offset: length, upstream });
        length += edge.length ?? edge.start.distanceTo(edge.end);
        if (upstream) edge = edges[edge.parentEdgeIndex];
        else
            edge = edge.childEdgeIndices?.length
                ? edge.childEdgeIndices
                      .map((i) => edges[i])
                      .reduce((a, b) => (a.radiusEnd > b.radiusEnd ? a : b))
                : null;
    }
    return { path, length };
}
function sample(path, distance) {
    const entry =
        path.find(
            ({ edge, offset }) =>
                offset + (edge.length ?? edge.start.distanceTo(edge.end)) >= distance
        ) ?? path.at(-1);
    const { edge, offset, upstream } = entry;
    const length = edge.length ?? edge.start.distanceTo(edge.end);
    let t = THREE.MathUtils.clamp((distance - offset) / Math.max(1e-9, length), 0, 1);
    if (upstream) t = 1 - t;
    return {
        edge,
        t,
        point: edge.start.clone().lerp(edge.end, t),
        radius: THREE.MathUtils.lerp(edge.radiusStart, edge.radiusEnd, t)
    };
}
function fieldSample(path, distance, spacing) {
    const result = sample(path, distance),
        stencil = [];
    const lo = Math.max(0, distance - spacing * 0.5),
        hi = distance + spacing * 0.5;
    let total = 0;
    for (const { edge, offset, upstream } of path) {
        for (let cell = 0; cell < edge.cellCount; cell++) {
            const alongCell = upstream ? edge.cellCount - 1 - cell : cell;
            const a = offset + alongCell * edge.cellLength,
                b = a + edge.cellLength;
            const weight = Math.max(0, Math.min(hi, b) - Math.max(lo, a));
            if (weight > 0) {
                stencil.push({ edge, cell, weight });
                total += weight;
            }
        }
    }
    for (const entry of stencil) entry.weight /= total;
    result.stencil = stencil;
    return result;
}
export function buildJunctionRegions(network) {
    const edges = network.edges,
        candidates = [];
    for (const parent of edges) {
        if (
            parent.renderExcluded ||
            parent.transportExcluded ||
            parent.radiusEnd < 3.5 ||
            parent.childEdgeIndices?.length !== 2
        )
            continue;
        const children = parent.childEdgeIndices.map((i) => edges[i]);
        // Large balanced forks need all three limbs, not an arbitrarily chosen
        // continuing trunk. Small side branches retain the ostial correction.
        const radius = parent.radiusEnd,
            reach = radius * 2.6;
        const limbs = [
            walk(edges, parent, true, reach),
            ...children.map((e) => walk(edges, e, false, reach))
        ];
        if (limbs.some((l) => l.length < radius)) continue;
        const daughterRadii = limbs
            .slice(1)
            .map((l) => sample(l.path, Math.min(radius, l.length)).radius);
        if (
            Math.min(...daughterRadii) < radius * 0.45 ||
            Math.min(...daughterRadii) < Math.max(...daughterRadii) * 0.65
        )
            continue;
        const center = parent.end.clone(),
            region = {
                center,
                radius: radius * 2,
                limbs: [],
                edgeIndices: new Set(
                    limbs.flatMap((limb) => limb.path.map(({ edge }) => edge.index))
                )
            };
        for (const limb of limbs) {
            const extent = Math.min(reach, limb.length);
            region.limbs.push(
                Array.from({ length: STEPS + 1 }, (_, i) =>
                    fieldSample(limb.path, (extent * i) / STEPS, extent / STEPS)
                )
            );
        }
        candidates.push(region);
    }
    const regions = [];
    for (const candidate of candidates.sort((a, b) => b.radius - a.radius)) {
        if (
            !regions.some(
                (r) => r.center.distanceTo(candidate.center) < r.radius + candidate.radius
            )
        )
            regions.push(candidate);
    }
    return regions;
}

export class JunctionContrastVolume {
    constructor(regions, offsets, uniforms) {
        this.regions = regions;
        this.offsets = offsets;
        this.group = new THREE.Group();
        this.group.name = 'shared-junction-contrast';
        for (const region of regions) {
            const starts = [],
                ends = [],
                concentration = [],
                previousConcentration = [],
                samples = [];
            for (const limb of region.limbs)
                for (let i = 0; i < STEPS; i++) {
                    for (const [list, s] of [
                        [starts, limb[i]],
                        [ends, limb[i + 1]]
                    ]) {
                        const p = s.point.clone().sub(region.center);
                        list.push(new THREE.Vector4(p.x, p.y, p.z, s.radius));
                    }
                    samples.push([limb[i], limb[i + 1]]);
                    concentration.push(new THREE.Vector2());
                    previousConcentration.push(new THREE.Vector2());
                }
            const material = new THREE.ShaderMaterial({
                vertexShader,
                fragmentShader,
                uniforms: {
                    ...uniforms,
                    radius: { value: region.radius },
                    starts: { value: starts },
                    ends: { value: ends },
                    concentration: { value: concentration },
                    previousConcentration: { value: previousConcentration }
                },
                transparent: true,
                premultipliedAlpha: true,
                blending: THREE.AdditiveBlending,
                depthTest: false,
                depthWrite: false,
                toneMapped: false
            });
            const mesh = new THREE.Mesh(new THREE.PlaneGeometry(2, 2), material);
            mesh.position.copy(region.center);
            mesh.frustumCulled = false;
            mesh.renderOrder = 6;
            this.group.add(mesh);
            Object.assign(region, { mesh, samples, concentration, previousConcentration });
        }
    }
    update(plume, stock, advanceHistory, graftReplacesLumen) {
        for (const region of this.regions) {
            // Keep the existing graft/sac rendering responsible for a junction
            // whose native lumen has been replaced by deployed fabric.
            region.enabled = !region.samples.some((pair) =>
                pair.some(
                    ({ edge, t }) =>
                        edge.renderExcluded ||
                        edge.transportExcluded ||
                        (graftReplacesLumen &&
                            edge.graftSections?.[
                                Math.min(edge.cellCount - 1, Math.floor(t * edge.cellCount))
                            ])
                )
            );
            region.mesh.visible = region.enabled;
            region.samples.forEach((pair, i) => {
                if (advanceHistory) region.previousConcentration[i].copy(region.concentration[i]);
                pair.forEach(({ stencil }, j) => {
                    let c = 0;
                    for (const { edge, cell, weight } of stencil)
                        c +=
                            (weight *
                                (edge.massMg[cell] + plume[this.offsets[edge.index] + cell])) /
                            edge.volumes[cell] /
                            stock;
                    region.concentration[i].setComponent(j, c);
                });
            });
            region.mesh.visible =
                region.enabled &&
                region.concentration.some(
                    (c, i) =>
                        c.x > 0 ||
                        c.y > 0 ||
                        region.previousConcentration[i].x > 0 ||
                        region.previousConcentration[i].y > 0
                );
        }
    }
    resetHistory() {
        for (const region of this.regions)
            for (const list of [region.concentration, region.previousConcentration])
                for (const c of list) c.set(0, 0);
    }
    dispose() {
        for (const region of this.regions) {
            region.mesh.geometry.dispose();
            region.mesh.material.dispose();
        }
    }
}
