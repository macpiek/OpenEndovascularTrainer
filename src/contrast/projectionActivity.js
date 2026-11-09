import {Box3, Matrix4, Vector3} from 'three';

// Derive optical support from the actual anatomical triangles and their
// transported sample dependencies. A graph radius is not a safe bound for
// aneurysms or other lumens whose wall lies far from the nominal axis.
export function createProjectionActivity(network, geometry, field, triangleEdges, junctionField) {
    const positions = geometry.getAttribute('position'), stencil = geometry.getAttribute('flowStencil');
    const full = new Box3().setFromBufferAttribute(positions);
    const edges = Array.from(network.edges, () => new Box3());
    const junctions = (junctionField?.junctions ?? []).map(() => new Box3());
    const cellEdges = new Uint32Array(field.cellCount);
    for (const edge of network.edges) cellEdges.fill(edge.index, field.offsets[edge.index], field.offsets[edge.index] + edge.cellCount);
    const triangle = new Box3(), point = new Vector3();
    const addIndex = index => {
        if (index < 0) return;
        if (index < field.cellCount) edges[cellEdges[index]].union(triangle);
        else if (index < field.cellCount + field.nodes.length) {
            const node = field.nodes[index - field.cellCount];
            if (node.parentEdgeIndex >= 0) edges[node.parentEdgeIndex].union(triangle);
            for (const child of node.childEdgeIndices) edges[child].union(triangle);
        }
    };
    for (let i = 0; i < positions.count; i += 3) {
        triangle.makeEmpty();
        for (let k = 0; k < 3; k++) triangle.expandByPoint(point.fromBufferAttribute(positions, i + k));
        const owner = triangleEdges?.[i / 3];
        if (owner !== undefined && edges[owner]) edges[owner].union(triangle);
        for (let k = 0; stencil && k < 3; k++) {
            const lower = stencil.getX(i + k), upper = stencil.getY(i + k);
            if (junctionField && lower >= junctionField.offset && lower < junctionField.offset + junctionField.sampleCount) {
                const sample = lower - junctionField.offset;
                addIndex(junctionField.stencils[sample * 4]); addIndex(junctionField.stencils[sample * 4 + 1]);
                for (let j = junctionField.starts[sample]; j < junctionField.starts[sample + 1]; j++)
                    junctions[junctionField.junctionIndices[j]].union(triangle);
            } else { addIndex(lower); addIndex(upper); }
        }
    }
    // Include the complete interior between the wall and the axis. Adjacent
    // connected regions cover endpoint reconstruction and shared fork blood.
    const ownBounds = edges.map(box => box.clone());
    for (const edge of network.edges) {
        if (!edges[edge.index].isEmpty()) edges[edge.index].expandByPoint(edge.start).expandByPoint(edge.end);
        if (edge.parentEdgeIndex >= 0) edges[edge.index].union(ownBounds[edge.parentEdgeIndex]);
        for (const child of edge.childEdgeIndices) edges[edge.index].union(ownBounds[child]);
    }
    for (const junction of junctionField?.junctions ?? []) {
        const box = junctions[junction.index];
        box.expandByPoint(junction.center);
        for (const anchor of junction.anchors) box.expandByPoint(anchor.point);
    }
    const bounds = full.clone(), uniforms = {
        projectionActiveMin: {value: bounds.min}, projectionActiveMax: {value: bounds.max},
        projectionCullActive: {value: false}
    };
    const chunks = [], chunkVertices = 4096 * 3;
    for (let start = 0; start < positions.count; start += chunkVertices) {
        const count = Math.min(chunkVertices, positions.count - start), box = new Box3();
        for (let i = start; i < start + count; i++) box.expandByPoint(point.fromBufferAttribute(positions, i));
        chunks.push({start, count, box});
    }
    const matrix = new Matrix4(), activeRectangle = new Float64Array(4), rectangle = new Float64Array(4);
    const project = (box, padding, output) => {
        output[0] = output[1] = Infinity; output[2] = output[3] = -Infinity;
        const m = matrix.elements;
        for (let corner = 0; corner < 8; corner++) {
            const x = corner & 1 ? box.max.x + padding : box.min.x - padding;
            const y = corner & 2 ? box.max.y + padding : box.min.y - padding;
            const z = corner & 4 ? box.max.z + padding : box.min.z - padding;
            const w = m[3] * x + m[7] * y + m[11] * z + m[15];
            if (w <= 1e-6) { output.set([-Infinity, -Infinity, Infinity, Infinity]); return; }
            const px = (m[0] * x + m[4] * y + m[8] * z + m[12]) / w;
            const py = (m[1] * x + m[5] * y + m[9] * z + m[13]) / w;
            output[0] = Math.min(output[0], px); output[1] = Math.min(output[1], py);
            output[2] = Math.max(output[2], px); output[3] = Math.max(output[3], py);
        }
    };
    const diagnostics = {activeEdges: 0, fallbackEdges: 0, drawnChunks: 0, culledChunks: 0};
    return {
        uniforms, bounds, chunks, diagnostics,
        update(current, previous) {
            bounds.makeEmpty(); diagnostics.activeEdges = 0; diagnostics.fallbackEdges = 0;
            for (const edge of network.edges) {
                if (edge.transportExcluded || edge.lumenMappingExcluded || edge.renderExcluded) continue;
                const offset = field.offsets[edge.index];
                let active = current[field.start[edge.index]] > 0 || previous[field.start[edge.index]] > 0 ||
                    current[field.end[edge.index]] > 0 || previous[field.end[edge.index]] > 0;
                for (let cell = 0; !active && cell < edge.cellCount; cell++)
                    active = current[offset + cell] > 0 || previous[offset + cell] > 0;
                if (!active) continue;
                diagnostics.activeEdges++;
                if (edges[edge.index].isEmpty()) { bounds.union(full); diagnostics.fallbackEdges++; }
                else bounds.union(edges[edge.index]);
            }
            for (const junction of junctionField?.junctions ?? [])
                if (junction.dependencies.some(index => current[index] > 0 || previous[index] > 0)) bounds.union(junctions[junction.index]);
            const active = !bounds.isEmpty();
            if (active) bounds.expandByScalar(.001);
            uniforms.projectionCullActive.value = active;
            return active;
        },
        install(mesh) {
            const before = mesh.onBeforeRender, after = mesh.onAfterRender;
            mesh.onBeforeRender = function(renderer, scene, camera, renderGeometry, material, group) {
                before.call(this, renderer, scene, camera, renderGeometry, material, group);
                if (!group || !uniforms.projectionCullActive.value) return;
                const index = Math.floor(group.start / chunkVertices), chunk = chunks[index];
                if (!chunk) return;
                const padding = material.uniforms.vesselDeformMaxDisplacement?.value ?? 0;
                matrix.multiplyMatrices(camera.projectionMatrix, camera.matrixWorldInverse).multiply(mesh.matrixWorld);
                project(bounds, padding, activeRectangle); project(chunk.box, padding, rectangle);
                const visible = rectangle[2] >= activeRectangle[0] && rectangle[0] <= activeRectangle[2] &&
                    rectangle[3] >= activeRectangle[1] && rectangle[1] <= activeRectangle[3];
                // Render-list groups are already captured when this callback
                // runs. drawRange is evaluated afterwards by renderBufferDirect,
                // so a disjoint chunk can skip its GPU draw in this same view.
                renderGeometry.setDrawRange(visible ? group.start : 0, visible ? group.count : 0);
                if (visible) diagnostics.drawnChunks++; else diagnostics.culledChunks++;
            };
            mesh.onAfterRender = function(renderer, scene, camera, renderGeometry, material, group) {
                renderGeometry.setDrawRange(0, Infinity);
                after.call(this, renderer, scene, camera, renderGeometry, material, group);
            };
        },
        setGroups() {
            geometry.clearGroups(); diagnostics.drawnChunks = diagnostics.culledChunks = 0;
            for (const chunk of chunks) geometry.addGroup(chunk.start, chunk.count, 0);
        }
    };
}
