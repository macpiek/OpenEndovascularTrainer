import * as THREE from 'three';
import {MeshBVH} from 'three-mesh-bvh';
import {createLumenSurfaceMapping} from './lumenSurfaceMapping.js';
import {FlowConcentrationField} from './flowConcentrationField.js';
import {JunctionSurfaceReconstruction} from './junctionSurfaceReconstruction.js';
import {closeLumenBoundaries} from './lumenBoundaryCaps.js';
import {buildAnatomicalContrastProjection} from './anatomicalContrastProjection.js';
import {buildAnatomicalColumnProjection} from './anatomicalColumnProjection.js';
import {createProjectionActivity} from './projectionActivity.js';

function continuousTerritories(network) {
    const territories = new Int32Array(network.edges.length).fill(-1);
    const continuations = new Int32Array(network.edges.length).fill(-1);
    const eligible = e => e && !e.transportExcluded && !e.lumenMappingExcluded;
    for (const edge of network.edges) {
        if (!eligible(edge)) continue;
        const children = (edge.childEdgeIndices ?? []).map(i => network.edges[i]).filter(eligible)
            .sort((a, b) => b.radiusStart - a.radiusStart);
        // The aortic wall continues past a small side ostium. Splitting its
        // sampling territory there clamps an entire cross-section to a very
        // short atlas connector, producing a dark ring at the bolus front.
        // Balanced bifurcations still retain their separate daughter lumens.
        if (children.length === 1 || (children.length > 1 &&
            children[0].radiusStart >= 2 * children[1].radiusStart &&
            children[0].radiusStart >= .65 * edge.radiusEnd))
            continuations[edge.index] = children[0].index;
    }
    for (const root of network.edges) {
        if (!eligible(root) || territories[root.index] >= 0) continue;
        let first = root;
        while (eligible(network.edges[first.parentEdgeIndex]) &&
            continuations[first.parentEdgeIndex] === first.index)
            first = network.edges[first.parentEdgeIndex];
        for (let edge = first; eligible(edge) && territories[edge.index] < 0;
            edge = network.edges[continuations[edge.index]])
            territories[edge.index] = first.index;
    }
    return {territories, continuations};
}

function projectionAxes(network, territories, continuations) {
    const axes = new Float32Array(network.edges.length * 3);
    const before = new THREE.Vector3(), after = new THREE.Vector3();
    const sample = (edge, distance, point) => {
        while (distance < 0 && edge.parentEdgeIndex >= 0 &&
            territories[edge.parentEdgeIndex] === territories[edge.index]) {
            edge = network.edges[edge.parentEdgeIndex];
            distance += edge.length;
        }
        while (distance > edge.length && continuations[edge.index] >= 0) {
            distance -= edge.length;
            edge = network.edges[continuations[edge.index]];
        }
        return point.copy(edge.start).addScaledVector(edge.axis, Math.max(0, Math.min(edge.length, distance)));
    };
    for (const edge of network.edges) {
        if (territories[edge.index] < 0) continue;
        // The optical chord needs the direction of the lumen, not a tiny
        // radial medial-axis step at an ostium. Average over one local radius
        // on either side, staying on the same continuous vessel wall.
        const span = Math.max(.1, (edge.radiusStart + edge.radiusEnd) * .5);
        sample(edge, edge.length * .5 - span, before);
        sample(edge, edge.length * .5 + span, after);
        after.sub(before).normalize().toArray(axes, edge.index * 3);
    }
    return axes;
}

// Render the blood-facing triangles of the anatomical wall, in their original
// coordinates. The transport graph supplies concentration, never the outline.
// This also retains vessels smaller than the collision SDF's voxel spacing.
export function buildAnatomicalContrastSurface(network, field, concentrationField, {junctionReconstruction = true, projection = 'volume'} = {}) {
    const source = field.fallbackGeometry;
    const position = source.getAttribute('position');
    const index = source.index;
    const count = index ? index.count : position.count;
    const accepted = [], owners = [], radii = [];
    const a = new THREE.Vector3(), b = new THREE.Vector3(), c = new THREE.Vector3();
    const ab = new THREE.Vector3(), normal = new THREE.Vector3(), center = new THREE.Vector3();
    const location = {};
    // Production anatomy already owns a BVH. Small synthetic/custom meshes
    // get a private indexed wrapper without changing the source's triangle order.
    let wallGeometry;
    const wallTree = source.boundsTree ?? new MeshBVH((wallGeometry = new THREE.BufferGeometry()
        .setAttribute('position',position).setIndex(index?.clone() ?? null)),{indirect:true});
    const {territories, continuations} = continuousTerritories(network);
    const opticalAxes = projectionAxes(network, territories, continuations);
    const edgeArcStart = new Float64Array(network.edges.length);
    for (const root of network.edges) {
        if (territories[root.index] !== root.index) continue;
        let distance = 0;
        for (let edge = root; edge; edge = network.edges[continuations[edge.index]]) {
            edgeArcStart[edge.index] = distance; distance += edge.length;
        }
    }
    const connections = new Map();
    const connectionKey = (a, b) => a < b ? `${a},${b}` : `${b},${a}`;
    for (const node of network.nodes?.values?.() ?? []) {
        const adjacent = [...new Set([node.parentEdgeIndex, ...node.childEdgeIndices]
            .filter(i => i >= 0).map(i => territories[i]).filter(i => i >= 0))];
        for (let a = 0; a < adjacent.length; a++) for (let b = a + 1; b < adjacent.length; b++) {
            const key = connectionKey(adjacent[a], adjacent[b]);
            if (!connections.has(key)) connections.set(key, []);
            connections.get(key).push(node.point);
        }
    }
    const mapToFlow = createLumenSurfaceMapping(network, {wallTree, territories, radiusWeighted: true});
    const ray = new THREE.Ray();
    const read = (target, i) => target.fromBufferAttribute(position, index ? index.getX(i) : i);
    for (let i = 0; i < count; i += 3) {
        read(a, i); read(b, i + 1); read(c, i + 2);
        normal.crossVectors(ab.subVectors(b, a), center.subVectors(c, a)).normalize();
        center.copy(a).add(b).add(c).multiplyScalar(1 / 3);
        mapToFlow(center.x, center.y, center.z, location);
        if (location.edgeIndex < 0) continue;
        ray.origin.copy(center);
        ray.direction.set(location.centerX-center.x,location.centerY-center.y,location.centerZ-center.z).normalize();
        const inward=normal.dot(ray.direction)*location.distance;
        const hit=wallTree.raycastFirst(ray,THREE.DoubleSide,1e-4,location.distance-1e-4);
        if (hit || inward<=1e-5) {
            // Test orientation against each candidate, not just the closest
            // axis: a small neighbour can be closer than this wall's own axis.
            // A concave inner wall can face away from the nearest axis and
            // cross one other fold on that ray. This is not proof of an outer
            // wall: only the constrained search can establish its ownership.
            const searchRadius = location.searchRadius;
            mapToFlow(center.x, center.y, center.z, location, normal, searchRadius);
            if (location.edgeIndex < 0) continue;
        }
        accepted.push(i); owners.push(location.edgeIndex);
        // Measured distance to the lumen axis is only an optical calibration.
        // It cannot move vertices or dilate the projected silhouette.
        radii.push(Math.max(0.01, location.distance));
    }
    const positions = new Float32Array(accepted.length * 9);
    const normals = new Int16Array(positions.length);
    const stencils = new Float32Array(accepted.length * 12);
    const optical = new Float32Array(accepted.length * 3);
    const edges = new Uint32Array(accepted.length);
    const projectionOwners = new Float32Array(accepted.length * 3);
    const columnCoordinates = new Float32Array(accepted.length * 6);
    const axes = new Int16Array(positions.length);
    // Contiguous chunks bound draw calls and let empty territories sleep.
    const chunks = [];
    const CHUNK_TRIANGLES = 4096;
    // STL duplicates the same wall vertex in neighbouring triangles. Assign
    // its flow sample once per continuous lumen, not once per triangle owner:
    // clamping to those different short edges exposes a sawtooth bolus front.
    // Real forks stay separate, so an empty side branch cannot borrow iodine.
    const sharedSamples = new Map();
    const sharedVertices = new Uint32Array(accepted.length * 3);
    const canonicalSamples = new Int32Array(accepted.length * 3).fill(-1);
    const representative = sample => {
        while (sample.canonical) sample = sample.canonical;
        return sample;
    };
    const joinSamples = (a, b) => {
        a = representative(a); b = representative(b);
        if (a === b) return;
        const ea = network.edges[a.edgeIndex], eb = network.edges[b.edgeIndex];
        const ra = Math.max(ea.radiusStart, ea.radiusEnd), rb = Math.max(eb.radiusStart, eb.radiusEnd);
        // A shared wall point has one physical concentration. Use its local
        // projection in the dominant lumen, never the remote graph-node value.
        // The choice depends on geometry, not concentration or triangle order.
        const keepA = ra > rb || (ra === rb && a.edgeIndex < b.edgeIndex);
        const kept = keepA ? a : b, replaced = keepA ? b : a;
        replaced.canonical = kept; kept.joined = true;
    };
    for (let triangle = 0; triangle < accepted.length; triangle++) {
        const i = accepted[triangle], edge = network.edges[owners[triangle]];
        edges[triangle] = edge.index;
        projectionOwners.fill(edge.index, triangle * 3, triangle * 3 + 3);
        read(a, i); read(b, i + 1); read(c, i + 2);
        normal.crossVectors(ab.subVectors(b, a), center.subVectors(c, a)).normalize();
        if (triangle % CHUNK_TRIANGLES === 0) chunks.push({start: triangle * 3, count: 0, edges: new Set()});
        const chunk = chunks.at(-1); chunk.count += 3; chunk.edges.add(edge.index);
        for (let j = 0; j < 3; j++) {
            const p = [a, b, c][j], vertex = triangle * 3 + j;
            p.toArray(positions, vertex * 3);
            const territory = territories[edge.index];
            // Procedural tubes duplicate the seam with sin(2π) roundoff.
            // Weld only numerical zero; physical coordinates stay untouched.
            const key = `${Math.abs(p.x) < 1e-7 ? 0 : p.x},${Math.abs(p.y) < 1e-7 ? 0 : p.y},${Math.abs(p.z) < 1e-7 ? 0 : p.z}`;
            const head = sharedSamples.get(key);
            let sample = head;
            while (sample && sample.territory !== territory) sample = sample.next;
            if (!sample) {
                mapToFlow(p.x, p.y, p.z, location, normal, Infinity, territory);
                let owner = location.edgeIndex >= 0 ? network.edges[location.edgeIndex] : edge;
                // Project onto the local lumen cross-section, then walk its
                // continuous axial coordinate. Clamping every wall point to
                // a tiny radial connector flattens several millimetres of a
                // smooth concentration gradient into a single dark slice.
                let distance = owner.length * .5;
                for (let k = 0; k < 3; k++) distance +=
                    (p.getComponent(k) - (owner.start.getComponent(k) + owner.end.getComponent(k)) * .5) *
                    opticalAxes[owner.index * 3 + k];
                while (distance < 0 && owner.parentEdgeIndex >= 0 &&
                    territories[owner.parentEdgeIndex] === territories[owner.index]) {
                    owner = network.edges[owner.parentEdgeIndex];
                    distance += owner.length;
                }
                while (distance > owner.length && continuations[owner.index] >= 0) {
                    distance -= owner.length;
                    owner = network.edges[continuations[owner.index]];
                }
                const t = Math.max(0, Math.min(1, distance / owner.length));
                stencils.set(concentrationField.stencil(owner.index, t), vertex * 4);
                columnCoordinates[vertex * 2] = territory;
                columnCoordinates[vertex * 2 + 1] = edgeArcStart[owner.index] + t * owner.length;
                optical[vertex] = location.edgeIndex >= 0 ? Math.max(.01, location.distance) : radii[triangle];
                for (let k = 0; k < 3; k++) axes[vertex * 3 + k] = Math.round(opticalAxes[owner.index * 3 + k] * 32767);
                sample = {vertex, edgeIndex: owner.index, territory, next: head};
                // The graph can branch inside a continuing aortic territory;
                // checking only territory endpoints misses these side ostia.
                // Reconcile exactly shared, topologically connected wall points
                // while keeping separate vessels and distal empty arms isolated.
                for (let other = head; other; other = other.next) {
                    const joints = connections.get(connectionKey(territory, other.territory));
                    if (!joints) continue;
                    const reach = 2 * Math.max(edge.radiusStart, edge.radiusEnd,
                        network.edges[other.edgeIndex].radiusStart, network.edges[other.edgeIndex].radiusEnd) + 1;
                    if (!joints.some(point => point && p.distanceToSquared(point) <= reach * reach)) continue;
                    joinSamples(sample, other);
                }
                sharedSamples.set(key, sample);
            } else {
                for (let k = 0; k < 4; k++) stencils[vertex * 4 + k] = stencils[sample.vertex * 4 + k];
                for (let k = 0; k < 3; k++) axes[vertex * 3 + k] = axes[sample.vertex * 3 + k];
                optical[vertex] = optical[sample.vertex];
                columnCoordinates[vertex * 2] = columnCoordinates[sample.vertex * 2];
                columnCoordinates[vertex * 2 + 1] = columnCoordinates[sample.vertex * 2 + 1];
            }
            sharedVertices[vertex] = sample.vertex;
            chunk.edges.add(sample.edgeIndex);
            for (let k = 0; k < 3; k++) {
                normals[vertex * 3 + k] = Math.round(normal.getComponent(k) * 32767);
            }
        }
    }
    const canonicalEdges = new Map();
    for (const head of sharedSamples.values()) for (let sample = head; sample; sample = sample.next) {
        const canonical = representative(sample);
        if (!canonical.joined) continue;
        canonicalSamples[sample.vertex] = canonical.vertex;
        canonicalEdges.set(canonical.vertex, canonical.edgeIndex);
        // Preserve this local sample's continuous perfusion limit too. A
        // special ungated seam would expose the full node concentration next
        // to limited neighbours and recreate a dark, mesh-shaped ridge.
    }
    for (let vertex = 0; vertex < sharedVertices.length; vertex++) {
        const canonical = canonicalSamples[sharedVertices[vertex]];
        if (canonical < 0) continue;
        for (let k = 0; k < 4; k++) stencils[vertex * 4 + k] = stencils[canonical * 4 + k];
        for (let k = 0; k < 3; k++) axes[vertex * 3 + k] = axes[canonical * 3 + k];
        optical[vertex] = optical[canonical];
        sharedVertices[vertex] = canonical;
        chunks[Math.floor(vertex / (CHUNK_TRIANGLES * 3))].edges.add(canonicalEdges.get(canonical));
    }
    const junctionField = new JunctionSurfaceReconstruction(network, concentrationField,
        territories, continuations, junctionReconstruction);
    // Reuse the temporary canonical map instead of another whole-mesh buffer.
    canonicalSamples.fill(-1);
    for (const head of sharedSamples.values()) for (let sample = head; sample; sample = sample.next) {
        if (representative(sample) !== sample) continue;
        a.fromArray(positions, sample.vertex * 3);
        canonicalSamples[sample.vertex] = junctionField.addSample(a, sample.edgeIndex,
            stencils.subarray(sample.vertex * 4, sample.vertex * 4 + 4));
    }
    junctionField.finish();
    for (let vertex = 0; vertex < sharedVertices.length; vertex++) {
        const sample = canonicalSamples[sharedVertices[vertex]];
        if (sample < 0) continue;
        const index = junctionField.offset + sample;
        stencils.set([index, index, 0, -1], vertex * 4);
        junctionField.addChunkDependencies(sample, chunks[Math.floor(vertex / (CHUNK_TRIANGLES * 3))].edges);
    }
    const geometry = new THREE.BufferGeometry();
    for (const [name, array, size] of [
        ['position', positions, 3], ['normal', normals, 3], ['flowStencil', stencils, 4],
        ['opticalRadius', optical, 1], ['flowAxis', axes, 3], ['projectionOwner', projectionOwners, 1],
        ['columnCoordinate', columnCoordinates, 2]
    ]) geometry.setAttribute(name, new THREE.BufferAttribute(array, size, array instanceof Int16Array));
    let caps = {triangles: 0, loops: 0, boundaryEdges: 0, skippedLoops: 0};
    if (projection !== 'surface') {
        // Geometric identity is independent of concentration territories.
        // Reuse the existing weld map and scratch buffers for closure edges.
        for (const head of sharedSamples.values()) for (let sample = head; sample; sample = sample.next)
            canonicalSamples[sample.vertex] = head.vertex;
        for (let vertex = 0; vertex < sharedVertices.length; vertex++)
            sharedVertices[vertex] = canonicalSamples[sharedVertices[vertex]];
        caps = closeLumenBoundaries(geometry, sharedVertices);
    }
    geometry.computeBoundingSphere();
    for (const chunk of chunks) chunk.edges = Array.from(chunk.edges);
    wallGeometry?.dispose();
    return {geometry, chunks, triangleEdges: edges, sourceTriangleCount: count / 3, junctionField,
        territories, opticalAxes, continuations, caps};
}

const vertexShader = `
attribute vec4 flowStencil;
attribute float opticalRadius;
attribute vec3 flowAxis;
uniform sampler2D flowValues;
uniform vec2 flowSize;
uniform float displayAlpha;
varying float iodine;
varying float radius;
varying vec3 viewNormal;
varying vec3 viewRay;
varying vec3 axis;
float concentrationAt(float i) {
    vec2 uv=(vec2(mod(i,flowSize.x),floor(i/flowSize.x))+.5)/flowSize;
    vec2 c=texture2D(flowValues,uv).rg;
    return mix(c.y,c.x,displayAlpha);
}
void main() {
    float lower=concentrationAt(flowStencil.x), upper=concentrationAt(flowStencil.y);
    if(flowStencil.w>=0.) {
        // A vanishingly small arrival cannot expose a fully opacified node
        // over the whole empty arm. Limit half-cell endpoint reconstruction
        // continuously by the arm's own concentration, as on the CPU field.
        float limit=2.*max(0.,concentrationAt(flowStencil.w));
        lower=min(lower,limit); upper=min(upper,limit);
    }
    iodine=mix(lower,upper,flowStencil.z);
    vec4 view=modelViewMatrix*vec4(position,1.);
    viewNormal=normalMatrix*normal;
    viewRay=isOrthographic ? vec3(0.,0.,1.) : -view.xyz;
    axis=mat3(modelViewMatrix)*flowAxis;
    radius=opticalRadius;
    gl_Position=projectionMatrix*view;
}`;
const fragmentShader = `
uniform float signalGain;
uniform bool debugMode;
uniform vec3 debugColor;
varying float iodine;
varying float radius;
varying vec3 viewNormal;
varying vec3 viewRay;
varying vec3 axis;
void main() {
    if(iodine<=0.) discard;
    vec3 ray=normalize(viewRay);
    float transverse=max(.08,1.-pow(dot(normalize(axis),ray),2.));
    // Half-chord calibration matches the volume renderer. Geometry alone
    // determines coverage; no radius-based billboard, blur or edge expansion.
    float chord=radius*abs(dot(normalize(viewNormal),ray))/transverse;
    float depth=.5*max(0.,iodine)*chord*signalGain;
    gl_FragColor=vec4((debugMode?debugColor:vec3(1.))*depth,depth);
}`;

export class AnatomicalContrastSurface {
    constructor(network, offsets, contactField, options) {
        this.network = network;
        this.offsets = offsets;
        this.weights = new Float32Array(network.edges.length).fill(1);
        const count = network.edges.reduce((n, e) => n + e.cellCount, 0);
        this.field = new FlowConcentrationField(network, offsets, count);
        this.cellValues = new Float32Array(count);
        this.excludedCells = new Uint8Array(count);
        const built = buildAnatomicalContrastSurface(network, contactField, this.field, options);
        this.junctionField = built.junctionField;
        if (this.junctionField.sampleCount) {
            this.field.values = new Float32Array(this.field.values.length + this.junctionField.sampleCount);
        }
        this.current = this.field.values;
        this.previous = new Float32Array(this.current.length);
        const width = Math.min(1024, this.current.length), height = Math.ceil(this.current.length / width);
        this.data = new Float32Array(width * height * 4);
        this.texture = new THREE.DataTexture(this.data, width, height, THREE.RGBAFormat, THREE.FloatType);
        this.texture.needsUpdate = true;
        const projectionBuilder = options?.projection === 'column' ? buildAnatomicalColumnProjection : buildAnatomicalContrastProjection;
        this.projection = options?.projection === 'surface' ? null :
            projectionBuilder(network, built.geometry, this.field,
                {junctionField: built.junctionField, territories: built.territories,
                    opticalAxes: built.opticalAxes, continuations: built.continuations});
        this.activity = this.projection ? createProjectionActivity(network, built.geometry, this.field,
            built.triangleEdges, built.junctionField) : null;
        this.diagnostics = {sourceTriangleCount: built.sourceTriangleCount, lumenTriangleCount: built.triangleEdges.length,
            junctionCount: this.junctionField.junctions.length, junctionSamples: this.junctionField.sampleCount,
            junctionMemoryBytes: this.junctionField.memoryBytes, boundaryCaps: built.caps,
            projectionMemoryBytes: this.projection?.memoryBytes ?? 0,
            projectionActivity: this.activity?.diagnostics ?? null};
        this.material = new THREE.ShaderMaterial({
            vertexShader: this.projection?.vertexShader ?? vertexShader,
            fragmentShader: this.projection?.fragmentShader ?? fragmentShader,
            uniforms: {...this.projection?.uniforms, ...this.activity?.uniforms, flowValues: {value: this.texture}, flowSize: {value: new THREE.Vector2(width, height)},
                displayAlpha: {value: 1}, signalGain: {value: .14}, debugMode: {value: false},
                debugColor: {value: new THREE.Color(0x14b8ff)}},
            // The inward-wound entry wall bounds the actual iodine integral.
            // The legacy scalar-field preview averages both blood-facing walls.
            side: this.projection ? THREE.BackSide : THREE.DoubleSide, forceSinglePass: true, transparent: true, premultipliedAlpha: true,
            blending: THREE.AdditiveBlending, depthTest: false, depthWrite: false, toneMapped: false});
        this.material.userData.attachVesselDeformation = this.projection?.attachVesselDeformation;
        // Keep reconstruction metadata on the CPU. Three uploads every
        // enumerable geometry attribute, even if the current shader does not
        // use it; the volume projector needs only anatomical positions and
        // the triangle's initial owner for its bounded interior search.
        this.samplingGeometry = built.geometry;
        const rasterGeometry = this.projection ? new THREE.BufferGeometry()
            .setAttribute('position', built.geometry.getAttribute('position')) : built.geometry;
        if (this.projection) {
            const owner = built.geometry.getAttribute('projectionOwner');
            // The column shader uses territory/arc coordinates and its BVH
            // face ID. Keep edge ownership on the CPU without uploading an
            // unused four-byte attribute for every anatomical wall vertex.
            if (owner && options?.projection !== 'column') rasterGeometry.setAttribute('projectionOwner', owner);
            if (options?.projection === 'column') {
                rasterGeometry.setAttribute('columnCoordinate', built.geometry.getAttribute('columnCoordinate'));
                rasterGeometry.setAttribute('columnFace', built.geometry.getAttribute('columnFace'));
            }
            if (built.geometry.index) rasterGeometry.setIndex(built.geometry.index);
            rasterGeometry.boundingSphere = built.geometry.boundingSphere;
            rasterGeometry.boundingBox = built.geometry.boundingBox;
        }
        this.mesh = new THREE.Mesh(rasterGeometry, [this.material]);
        this.activity?.install(this.mesh);
        this.projection?.install?.(this.mesh);
        // Public material uniform contract used by ContrastVolumeRenderer.
        this.uniforms = this.material.uniforms;
        this.mesh.name = 'anatomical-contrast-inner-surface';
        this.mesh.renderOrder = 6;
        this.chunks = built.chunks;
        this.edgeVisible = new Uint8Array(network.edges.length);
        this.mesh.visible = false;
    }
    update(plume, stock, advanceHistory, graftReplacesLumen = false, localSolver = null) {
        const outletJet = this.projection?.outletJet;
        // The graft proxy owns blood inside covered fabric. Keep its existing
        // source sampling until the jet can be clipped to that separate lumen.
        const spatialJet = outletJet && localSolver && !graftReplacesLumen;
        outletJet?.update(spatialJet ? localSolver : null, stock);
        if (advanceHistory) this.previous.set(this.current);
        for (const e of this.network.edges) {
            let visible = false;
            for (let cell = 0; cell < e.cellCount; cell++) {
                const i = this.offsets[e.index] + cell;
                this.excludedCells[i] = Boolean(e.transportExcluded || e.renderExcluded ||
                    (graftReplacesLumen && e.graftSections?.[cell]));
                const pending = spatialJet ? (outletJet.fallbackMassByCell?.[i] ?? 0) : plume[i];
                this.cellValues[i] = this.excludedCells[i] ? 0 :
                    (e.massMg[cell] + pending) / e.volumes[cell] / stock;
                visible ||= this.cellValues[i] > 0 || this.previous[i] > 0;
            }
            this.edgeVisible[e.index] = visible;
        }
        this.field.update(this.cellValues);
        this.junctionField.update(this.current, this.excludedCells);
        this.projection?.update?.(this.current, this.previous);
        for (let i = 0; i < this.current.length; i++) {
            this.data[i * 4] = this.current[i]; this.data[i * 4 + 1] = this.previous[i];
            this.data[i * 4 + 2] = this.excludedCells[i] ?? 0;
        }
        this.texture.needsUpdate = true;
        this.mesh.geometry.clearGroups();
        if (this.projection) {
            // Keep clear entry walls whose rays can reach active blood, while
            // avoiding unrelated anatomical regions in this camera's view.
            const active = this.activity.update(this.current, this.previous, outletJet?.bounds);
            if (active) {
                this.activity.setGroups();
                // Activity support is computed from the CPU metadata, but
                // the renderer must receive the same anatomical draw ranges.
                this.mesh.geometry.groups = this.samplingGeometry.groups;
            }
            this.mesh.visible = active;
            return;
        }
        for (const chunk of this.chunks) {
            // Endpoint reconstruction can colour the half-cell before a front.
            const visible = chunk.edges.some(i => {
                return this.edgeVisible[i] || this.current[this.field.start[i]] > 0 ||
                    this.current[this.field.end[i]] > 0 || this.previous[this.field.start[i]] > 0 ||
                    this.previous[this.field.end[i]] > 0;
            });
            if (visible) this.mesh.geometry.addGroup(chunk.start, chunk.count, 0);
        }
        this.mesh.visible = this.mesh.geometry.groups.length > 0;
    }
    resetHistory() {
        this.projection?.outletJet?.update(null, 1);
        this.cellValues.fill(0); this.excludedCells.fill(0); this.current.fill(0); this.previous.fill(0); this.data.fill(0);
        this.texture.needsUpdate = true; this.mesh.visible = false;
        this.projection?.update?.(this.current, this.previous);
    }
    dispose() {
        this.projection?.dispose(); this.texture.dispose(); this.mesh.geometry.dispose();
        if (this.samplingGeometry !== this.mesh.geometry) this.samplingGeometry.dispose();
        this.material.dispose();
    }
}
