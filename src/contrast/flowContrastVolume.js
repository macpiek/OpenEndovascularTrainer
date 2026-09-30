import * as THREE from 'three';
import { buildJunctionRegions, JunctionContrastVolume } from './junctionContrastVolume.js';
import { buildOstialVolumeOwners, sampleOstialVolumeOwner } from './ostialVolumeOwnership.js';

// Analytically projected filled ellipsoids. Each kernel carries a cell's
// iodine mass; integrating along the view ray gives a filled cloud rather
// than colouring the front surface of a bent tube. No surface-normal shading.
const vertexShader = `
attribute vec3 center;
attribute vec3 axis;
attribute vec2 sigma;
attribute float opticalMass;
attribute float previousOpticalMass;
attribute vec4 lumenOwner;
attribute vec4 junctionRegion;
attribute vec3 lumenOwnerAxis;
attribute vec2 lumenOwnerBounds;
attribute float ownerOpticalMass;
attribute float previousOwnerOpticalMass;
uniform float displayAlpha;
varying vec2 coordinate;
varying vec4 junction;
varying float density;
varying float sharedFraction;
varying vec3 viewAxis;
varying vec2 kernelSize;
varying vec4 owner;
varying vec3 ownerAxis;
varying vec2 ownerBounds;
varying vec2 rayOffset;
void main() {
    float mass = mix(previousOpticalMass, opticalMass, displayAlpha);
    if (mass <= 0.) {
        coordinate = vec2(0.);
        density = 0.;
        gl_Position = vec4(2., 2., 2., 1.);
        return;
    }
    // In shared blood keep the stronger local concentration, not the sum.
    // A daughter-only bolus remains visible even before the trunk opacifies.
    sharedFraction = clamp(mix(previousOwnerOpticalMass, ownerOpticalMass, displayAlpha) / mass, 0., 1.);
    vec4 p = modelViewMatrix * vec4(center, 1.0);
    vec3 direction = normalize(mat3(modelViewMatrix) * axis);
    float projected = length(direction.xy);
    vec2 along = projected > 0.00001 ? direction.xy / projected : vec2(0., 1.);
    float radial = sigma.x;
    float axial = sqrt(sigma.y*sigma.y*projected*projected + radial*radial*direction.z*direction.z);
    coordinate = position.xy * 3.;
    viewAxis = direction;
    kernelSize = sigma * 3.;
    junction = vec4((modelViewMatrix * vec4(junctionRegion.xyz, 1.)).xyz - p.xyz, junctionRegion.w);
    owner = vec4((modelViewMatrix * vec4(lumenOwner.xyz, 1.)).xyz - p.xyz, lumenOwner.w);
    ownerBounds = lumenOwnerBounds;
    ownerAxis = normalize(mat3(modelViewMatrix) * lumenOwnerAxis);
    rayOffset = along * coordinate.y * axial + vec2(along.y, -along.x) * coordinate.x * radial;
    p.xy += along * coordinate.y * axial + vec2(along.y, -along.x) * coordinate.x * radial;
    // Normalized chord integral, including the soft boundary taper.
    density = mass / (6.28318530718 * radial * axial * 2.8069689275);
    gl_Position = projectionMatrix * p;
}`;
const fragmentShader = `
uniform bool debugMode;
uniform vec3 debugColor;
uniform float signalGain;
varying vec2 coordinate;
varying vec4 junction;
varying float density;
varying float sharedFraction;
varying vec3 viewAxis;
varying vec2 kernelSize;
varying vec4 owner;
varying vec3 ownerAxis;
varying vec2 ownerBounds;
varying vec2 rayOffset;
void main() {
    float r2 = dot(coordinate, coordinate);
    if(r2 >= 9.) discard;
    float kernel = sqrt(max(0., 1. - r2 / 9.)) * (1. - smoothstep(6.25, 9., r2));
    float coverage = 1.;
    if (owner.w > 0.) {
        // Ellipsoid chord along this view ray (including a tilted long axis).
        float invR2 = 1. / (kernelSize.x * kernelSize.x);
        float delta = 1. / (kernelSize.y * kernelSize.y) - invR2;
        float qzz = invR2 + delta * viewAxis.z * viewAxis.z;
        float midpoint = -delta * dot(rayOffset, viewAxis.xy) * viewAxis.z / qzz;
        float halfChord = sqrt(max(0., (1. - r2 / 9.) / qzz));
        // Intersect with the local continuing lumen in 3D, not its screen
        // silhouette. Separated vessels at different depths remain additive.
        vec2 xy = rayOffset - owner.xy;
        float along = dot(xy, ownerAxis.xy);
        float a = 1. - ownerAxis.z * ownerAxis.z;
        float b = -ownerAxis.z * along;
        float c = dot(xy, xy) - along * along - owner.w * owner.w;
        float disc = b*b - a*c;
        float lo = 1., hi = -1.;
        if (a > 0.00001 && disc > 0.) {
            lo = owner.z + (-b - sqrt(disc)) / a;
            hi = owner.z + (-b + sqrt(disc)) / a;
        } else if (a <= 0.00001 && c < 0.) {
            lo = -1.e20; hi = 1.e20;
        }
        if (abs(ownerAxis.z) > 0.00001) {
            float end0 = owner.z + (ownerBounds.x - along) / ownerAxis.z;
            float end1 = owner.z + (ownerBounds.y - along) / ownerAxis.z;
            lo = max(lo, min(end0, end1));
            hi = min(hi, max(end0, end1));
        } else if (along < ownerBounds.x || along > ownerBounds.y) {
            hi = lo;
        }
        float overlap = max(0., min(midpoint + halfChord, hi) - max(midpoint - halfChord, lo));
        coverage = 1. - sharedFraction * clamp(overlap / max(0.00001, 2. * halfChord), 0., 1.);
    }
    if (junction.w > 0.) {
        float invR2 = 1. / (kernelSize.x * kernelSize.x);
        float delta = 1. / (kernelSize.y * kernelSize.y) - invR2;
        float qzz = invR2 + delta * viewAxis.z * viewAxis.z;
        float midpoint = -delta * dot(rayOffset, viewAxis.xy) * viewAxis.z / qzz;
        float halfChord = sqrt(max(0., (1. - r2 / 9.) / qzz));
        float shared = 0.;
        for(int i=0;i<8;i++) {
            float z = midpoint + halfChord * (2. * (float(i)+.5)/8. - 1.);
            float distance = length(vec3(rayOffset,z)-junction.xyz);
            shared += 1. - smoothstep(.55 * junction.w, junction.w, distance);
        }
        coverage *= 1. - shared / 8.;
    }
    float depth = density * kernel * coverage * signalGain;
    gl_FragColor = vec4((debugMode ? debugColor : vec3(1.)) * depth, depth);
}`;

export class FlowContrastVolume {
    constructor(network, offsets) {
        this.network = network;
        this.offsets = offsets;
        this.weights = new Float32Array(network.edges.length);
        this.cells = [];
        // Use one volume representation through the aorta, its forks and
        // narrow daughters. A calibre-based switch back to tube surfaces
        // changes the radial intensity profile at the bifurcation.
        for (const edge of network.edges) {
            if (edge.renderExcluded || edge.transportExcluded) continue;
            this.weights[edge.index] = 1;
            for (let cell = 0; cell < edge.cellCount; cell++) {
                const center = (cell + 0.5) / edge.cellCount;
                const radius = THREE.MathUtils.lerp(edge.radiusStart, edge.radiusEnd, center);
                const axial = Math.max(edge.cellLength * 0.6, radius * 0.3);
                // Integrate each finite cell using up to four subcell
                // samples. One compact kernel per long cell creates a
                // periodic intensity ripple even at uniform concentration.
                // This changes only quadrature, never the transported mass.
                const count = Math.max(1, Math.ceil((2.4 * edge.cellLength) / axial - 1e-9));
                for (let sample = 0; sample < count; sample++)
                    this.cells.push({
                        edge,
                        cell,
                        t: (cell + (sample + 0.5) / count) / edge.cellCount,
                        radius,
                        axial,
                        fraction: 1 / count
                    });
            }
        }
        const owners = buildOstialVolumeOwners(network);
        const junctions = buildJunctionRegions(network);
        this.cellJunctions = [];
        const count = this.cells.length;
        const geometry = new THREE.InstancedBufferGeometry();
        geometry.setAttribute(
            'position',
            new THREE.Float32BufferAttribute([-1, -1, 0, 1, -1, 0, 1, 1, 0, -1, 1, 0], 3)
        );
        geometry.setIndex([0, 1, 2, 0, 2, 3]);
        const centers = new Float32Array(count * 3),
            axes = new Float32Array(count * 3),
            sigmas = new Float32Array(count * 2);
        const lumenOwners = new Float32Array(count * 4);
        const lumenAxes = new Float32Array(count * 3);
        const lumenBounds = new Float32Array(count * 2);
        const junctionData = new Float32Array(count * 4);
        this.owners = new Array(count);
        this.current = new Float32Array(count);
        this.previous = new Float32Array(count);
        this.ownerCurrent = new Float32Array(count);
        this.ownerPrevious = new Float32Array(count);
        this.cells.forEach(({ edge, t, radius, axial }, i) => {
            edge.start
                .clone()
                .lerp(edge.end, t)
                .toArray(centers, i * 3);
            const point = edge.start.clone().lerp(edge.end, t);
            const junction = junctions.find(
                (r) =>
                    r.edgeIndices.has(edge.index) &&
                    r.center.distanceTo(point) < r.radius + Math.max(radius, axial * 3)
            );
            this.cellJunctions[i] = junction;
            if (junction) {
                junction.center.toArray(junctionData, i * 4);
                junctionData[i * 4 + 3] = junction.radius;
            }
            const owner = sampleOstialVolumeOwner(
                owners.get(edge.index),
                point,
                Math.max(radius, axial * 3)
            );
            this.owners[i] = owner;
            if (owner) {
                owner.center.toArray(lumenOwners, i * 4);
                lumenOwners[i * 4 + 3] = owner.radius;
                owner.edge.axis.toArray(lumenAxes, i * 3);
                lumenBounds[i * 2] = owner.lower;
                lumenBounds[i * 2 + 1] = owner.upper;
            } else lumenAxes[i * 3 + 1] = 1;
            edge.axis.toArray(axes, i * 3);
            sigmas[i * 2] = radius / 3;
            sigmas[i * 2 + 1] = axial;
        });
        for (const [name, array, size] of [
            ['center', centers, 3],
            ['junctionRegion', junctionData, 4],
            ['lumenOwner', lumenOwners, 4],
            ['lumenOwnerAxis', lumenAxes, 3],
            ['lumenOwnerBounds', lumenBounds, 2],
            ['axis', axes, 3],
            ['sigma', sigmas, 2],
            ['opticalMass', this.current, 1],
            ['previousOpticalMass', this.previous, 1],
            ['ownerOpticalMass', this.ownerCurrent, 1],
            ['previousOwnerOpticalMass', this.ownerPrevious, 1]
        ])
            geometry.setAttribute(name, new THREE.InstancedBufferAttribute(array, size));
        geometry.instanceCount = count;
        const material = new THREE.ShaderMaterial({
            vertexShader,
            fragmentShader,
            uniforms: {
                displayAlpha: { value: 1 },
                signalGain: { value: 0.14 },
                debugMode: { value: false },
                debugColor: { value: new THREE.Color(0x14b8ff) }
            },
            transparent: true,
            premultipliedAlpha: true,
            blending: THREE.AdditiveBlending,
            depthTest: false,
            depthWrite: false,
            toneMapped: false
        });
        this.mesh = new THREE.Mesh(geometry, material);
        this.mesh.name = 'flow-contrast-volume';
        this.junctions = new JunctionContrastVolume(junctions, offsets, material.uniforms);
        this.mesh.add(this.junctions.group);
        this.mesh.frustumCulled = false;
        this.mesh.renderOrder = 6;
        this.mesh.visible = false;
    }
    update(plumeMass, stockConcentration, advanceHistory, graftReplacesLumen = false) {
        if (advanceHistory) {
            this.previous.set(this.current);
            this.ownerPrevious.set(this.ownerCurrent);
        }
        this.junctions.update(plumeMass, stockConcentration, advanceHistory, graftReplacesLumen);
        let visible = false;
        this.cells.forEach(({ edge, cell, fraction }, i) => {
            const junction = this.cellJunctions[i];
            if (junction)
                this.mesh.geometry.attributes.junctionRegion.setW(
                    i,
                    junction.enabled ? junction.radius : 0
                );
            const owner = this.owners[i];
            if (owner) {
                const ownerCell = Math.min(
                    owner.edge.cellCount - 1,
                    Math.floor(owner.t * owner.edge.cellCount)
                );
                const ownerHidden =
                    owner.edge.renderExcluded ||
                    owner.edge.transportExcluded ||
                    (graftReplacesLumen && owner.edge.graftSections?.[ownerCell]);
                this.mesh.geometry.attributes.lumenOwner.setW(i, ownerHidden ? 0 : owner.radius);
                const ownerConcentration =
                    (owner.edge.massMg[ownerCell] +
                        plumeMass[this.offsets[owner.edge.index] + ownerCell]) /
                    owner.edge.volumes[ownerCell];
                this.ownerCurrent[i] = ownerHidden
                    ? 0
                    : (0.5 * fraction * ownerConcentration * edge.volumes[cell]) /
                      stockConcentration;
            }
            const hidden =
                edge.transportExcluded ||
                edge.renderExcluded ||
                (graftReplacesLumen && edge.graftSections?.[cell]);
            // The legacy tube signal represents half the integrated chord;
            // keep the same calibration for the filled lumen image.
            this.current[i] = hidden
                ? 0
                : (0.5 *
                      fraction *
                      this.weights[edge.index] *
                      (edge.massMg[cell] + plumeMass[this.offsets[edge.index] + cell])) /
                  stockConcentration;
            visible ||= this.current[i] > 0 || this.previous[i] > 0;
        });
        this.mesh.geometry.attributes.junctionRegion.needsUpdate = true;
        this.mesh.geometry.attributes.ownerOpticalMass.needsUpdate = true;
        this.mesh.geometry.attributes.previousOwnerOpticalMass.needsUpdate = true;
        this.mesh.geometry.attributes.lumenOwner.needsUpdate = true;
        this.mesh.geometry.attributes.opticalMass.needsUpdate = true;
        this.mesh.geometry.attributes.previousOpticalMass.needsUpdate = true;
        this.mesh.visible = visible;
    }
    resetHistory() {
        this.current.fill(0);
        this.previous.fill(0);
        this.ownerCurrent.fill(0);
        this.ownerPrevious.fill(0);
        this.junctions.resetHistory();
    }
    dispose() {
        this.junctions.dispose();
        this.mesh.geometry.dispose();
        this.mesh.material.dispose();
    }
}
