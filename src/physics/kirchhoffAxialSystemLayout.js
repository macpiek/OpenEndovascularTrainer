import { kirchhoffComponentBodies } from './kirchhoffComponentBodies.js';
import { buildKirchhoffAxialLayout } from './kirchhoffAxialLayout.js';

/** Bind a shared-axis description to the ORIGINAL active Kirchhoff nodes.
 * This adapter does not move/remesh nodes, copy angular state, or infer
 * material alignment from a spatial nearest-contact match.
 */
export function buildKirchhoffAxialSystemLayout(constraint, material, arcs, options = {}) {
    const bodies = kirchhoffComponentBodies(constraint);
    if (bodies.length === 1) return singleBodyLayout(bodies[0], material[0], arcs[0], options);
    let offsets;
    if (options.axialCoordinates) offsets = options.axialOffsets ?? [0, 0];
    else {
        const {startNode, outerStartNode, innerArcOffset} = constraint;
        if (!Number.isInteger(startNode) || !Number.isInteger(outerStartNode) || !Number.isFinite(innerArcOffset)
            || startNode < material[0]?.start || startNode > material[0]?.end
            || outerStartNode < material[1]?.start || outerStartNode > material[1]?.end)
            throw new RangeError('Shared-axis layout requires explicit axial coordinates or a valid material containment window');
        // At inner startNode the material coordinate relative to the
        // catheter lumen start is innerArcOffset (the existing feed map).
        offsets = [0, arcs[0][startNode] - innerArcOffset - arcs[1][outerStartNode]];
    }
    const sources = bodies.map((body, side) => {
        const range = material[side];
        if (!range) throw new RangeError('Shared-axis layout requires two active material ranges');
        const count = range.end - range.start + 1;
        const positions = new Float64Array(count * 3), axialCoordinates = new Float64Array(count);
        const nodeIndices = new Int32Array(count), materialIds = [];
        for (let i = 0; i < count; i++) {
            const node = range.start + i;
            positions.set([body.x[node],body.y[node],body.z[node]],3*i);
            axialCoordinates[i] = arcs[side][node]; nodeIndices[i] = node;
            materialIds.push(`${body.id}:${body.materialCoordinate[node]}`);
        }
        return {positions,axialCoordinates,nodeIndices,materialIds};
    });
    return buildKirchhoffAxialLayout({wire:sources[0],catheter:sources[1],axialOffsets:offsets});
}


/** A single rod already owns its physical axis. There are no relative tool
 * coordinates to create or eliminate; retain only its real material nodes.
 */
function singleBodyLayout(body, range, arc, options) {
    if (!range || !Number.isInteger(range.start) || !Number.isInteger(range.end) ||
        range.start < 0 || range.end <= range.start || range.end >= body.count)
        throw new RangeError('Single-axis layout requires an active material range');
    const offsets = options.axialOffsets ?? [0];
    if (offsets.length !== 1 || !Number.isFinite(offsets[0]))
        throw new TypeError('Single-axis layout requires one finite axial offset');
    const count = range.end - range.start + 1;
    const nodes = new Int32Array(count), coordinates = new Float64Array(count);
    const positions = new Float64Array(3 * count), materialIds = [];
    for (let slot = 0; slot < count; slot++) {
        const node = range.start + slot;
        nodes[slot] = node;
        coordinates[slot] = arc?.[node] + offsets[0];
        if (!Number.isFinite(coordinates[slot]) || slot && !(coordinates[slot] > coordinates[slot - 1]))
            throw new RangeError('Single-axis coordinates must be finite and strictly increasing');
        for (let axis = 0; axis < 3; axis++) {
            const value = body[['x', 'y', 'z'][axis]][node];
            if (!Number.isFinite(value)) throw new TypeError('Single-axis positions must be finite');
            positions[3 * slot + axis] = value;
        }
        const material = body.materialCoordinate?.[node];
        if (!Number.isFinite(material)) throw new TypeError('Single-axis material coordinates must be finite');
        materialIds.push(`${body.id}:${material}`);
    }
    if (new Set(materialIds).size !== count) throw new TypeError('Single-axis material identities must be unique');
    return {kind: 'single', nodes, coordinates, positions, materialIds};
}
