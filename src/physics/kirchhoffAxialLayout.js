/** Full translational coordinate map, not a welded or reduced rod model.
 * The wire supplies the reference axis. Every catheter node retains its own
 * three coordinates: an offset within overlap, an absolute value outside it.
 * Angular variables and material state remain owned by the original rods.
 */
function vector(values, length, name) {
    if (!values || values.length !== length || !Array.from(values).every(Number.isFinite))
        throw new TypeError(`${name} must contain ${length} finite entries`);
    return Float64Array.from(values);
}

function tool(input, owner, offset) {
    if (!Number.isFinite(offset)) throw new TypeError('Axial offsets must be finite');
    const count = input?.axialCoordinates?.length;
    if (!Number.isInteger(count) || count < 0) throw new TypeError(`${owner} axialCoordinates are required`);
    const axial = vector(input.axialCoordinates, count, `${owner} axialCoordinates`);
    const positions = vector(input.positions, 3 * count, `${owner} positions`);
    for (let i = 0; i < count; i++) {
        if (i && !(axial[i] > axial[i - 1])) throw new RangeError('Axial coordinates must be strictly increasing');
    }
    const shifted = Float64Array.from(axial, value => value + offset);
    for (let i = 0; i < count; i++) {
        if (!Number.isFinite(shifted[i]) || i && !(shifted[i] > shifted[i - 1]))
            throw new RangeError('Shifted axial coordinates must be finite and strictly increasing');
    }
    const nodes = input.nodeIndices ? Array.from(input.nodeIndices) : Array.from({ length: count }, (_, i) => i);
    if (nodes.length !== count || nodes.some(node => !Number.isSafeInteger(node) || node < 0) || new Set(nodes).size !== count)
        throw new RangeError('Original node indices must be unique nonnegative integers');
    const ids = input.materialIds ? Array.from(input.materialIds) : nodes.map(node => `${owner}:${node}`);
    if (ids.length !== count || ids.some(id => typeof id !== 'string' || !id) || new Set(ids).size !== count)
        throw new TypeError('Material identities must be unique nonempty strings per tool');
    return { count, axial, shifted, positions, nodes, ids };
}

/** Inputs are packed xyz arrays and strictly increasing per-tool axial labels.
 * Offsets align those labels without modifying material identities. Rebuild
 * after feed/topology changes; a layout itself is a snapshot, never a cache.
 */
export function buildKirchhoffAxialLayout({ wire, catheter, axialOffsets = [0, 0] }) {
    if (!axialOffsets || axialOffsets.length !== 2) throw new TypeError('Two axial offsets are required');
    const w = tool(wire, 'wire', axialOffsets[0]), c = tool(catheter, 'catheter', axialOffsets[1]);
    const start = Math.max(w.shifted[0] ?? Infinity, c.shifted[0] ?? Infinity);
    const end = Math.min(w.shifted[w.count - 1] ?? -Infinity, c.shifted[c.count - 1] ?? -Infinity);
    const overlap = start <= end ? [start, end] : null;
    const sites = [];
    let edge = 0;
    for (let i = 0; i < c.count; i++) {
        const x = c.shifted[i], site = { owner: 'catheter', slot: i, nodeIndex: c.nodes[i], materialId: c.ids[i],
            materialCoordinate: c.axial[i], axialCoordinate: x, wireSlots: [], wireNodes: [], weights: [] };
        if (overlap && x >= overlap[0] && x <= overlap[1]) {
            while (edge + 1 < w.count && w.shifted[edge + 1] < x) edge++;
            if (w.shifted[edge] === x || w.count === 1) {
                site.wireSlots = [edge]; site.weights = [1];
            } else if (w.shifted[edge + 1] === x) {
                site.wireSlots = [edge + 1]; site.weights = [1];
            } else {
                const fraction = (x - w.shifted[edge]) / (w.shifted[edge + 1] - w.shifted[edge]);
                site.wireSlots = [edge, edge + 1]; site.weights = [1 - fraction, fraction];
            }
            site.wireNodes = site.wireSlots.map(slot => w.nodes[slot]);
        }
        sites.push(site);
    }
    const layout = { wireCount: w.count, catheterCount: c.count, overlap, sites,
        wireNodes: w.nodes, wireMaterialIds: w.ids, wireAxialCoordinates: w.axial,
        wireAlignedCoordinates: w.shifted, axialOffsets: Array.from(axialOffsets) };
    layout.referenceCoordinates = encodeKirchhoffAxialLayout(layout, w.positions, c.positions);
    return layout;
}

/** Encode positions OR increments, using the same linear map. No rest offset
 * is added here: encoding positions captures it, increments encode changes.
 */
export function encodeKirchhoffAxialLayout(layout, wire, catheter) {
    const axis = vector(wire, 3 * layout.wireCount, 'wire xyz');
    const coordinates = vector(catheter, 3 * layout.catheterCount, 'catheter xyz');
    for (const site of layout.sites) for (let d = 0; d < 3; d++) {
        let center = 0;
        for (let k = 0; k < site.wireSlots.length; k++) center += site.weights[k] * axis[3 * site.wireSlots[k] + d];
        coordinates[3 * site.slot + d] -= center;
    }
    return { axis, catheterCoordinates: coordinates };
}

/** Owned packed xyz output; never modifies input buffers or material data. */
export function decodeKirchhoffAxialLayout(layout, { axis, catheterCoordinates }) {
    const wire = vector(axis, 3 * layout.wireCount, 'axis xyz');
    const catheter = vector(catheterCoordinates, 3 * layout.catheterCount, 'catheter coordinates');
    for (const site of layout.sites) for (let d = 0; d < 3; d++) {
        let center = 0;
        for (let k = 0; k < site.wireSlots.length; k++) center += site.weights[k] * wire[3 * site.wireSlots[k] + d];
        catheter[3 * site.slot + d] += center;
    }
    return { wire, catheter };
}

/** Covector pullback P^T f for original positions q=P*[axis,offset].
 * Forces on an interpolated catheter point act on its reference wire nodes
 * AND its independent offset. Outside overlap P is the identity.
 */
export function pullbackKirchhoffAxialForces(layout, wireForce, catheterForce) {
    const axis = vector(wireForce, 3 * layout.wireCount, 'wire force');
    const catheterCoordinates = vector(catheterForce, 3 * layout.catheterCount, 'catheter force');
    for (const site of layout.sites) for (let k = 0; k < site.wireSlots.length; k++) {
        for (let d = 0; d < 3; d++)
            axis[3 * site.wireSlots[k] + d] += site.weights[k] * catheterCoordinates[3 * site.slot + d];
    }
    return { axis, catheterCoordinates };
}

/** Exact action P^-1 W P^-T, with original diagonal inverse masses per xyz.
 * The transformed metric is not diagonal: different offsets can share wire
 * nodes. Zero inverse mass denotes a pinned ORIGINAL coordinate.
 */
export function applyKirchhoffAxialMobility(layout, generalizedForces, wireInverseMass, catheterInverseMass) {
    const wireForce = vector(generalizedForces.axis, 3 * layout.wireCount, 'generalized axis force');
    const catheterForce = vector(generalizedForces.catheterCoordinates, 3 * layout.catheterCount, 'generalized catheter force');
    const wi = vector(wireInverseMass, wireForce.length, 'wire inverse mass');
    const wc = vector(catheterInverseMass, catheterForce.length, 'catheter inverse mass');
    if (wi.some(v => v < 0) || wc.some(v => v < 0)) throw new RangeError('Inverse masses must be nonnegative');
    // P^-T: undo the force pullback before applying the ORIGINAL metric.
    for (const site of layout.sites) for (let k = 0; k < site.wireSlots.length; k++) {
        for (let d = 0; d < 3; d++)
            wireForce[3 * site.wireSlots[k] + d] -= site.weights[k] * catheterForce[3 * site.slot + d];
    }
    for (let i = 0; i < wireForce.length; i++) wireForce[i] *= wi[i];
    for (let i = 0; i < catheterForce.length; i++) catheterForce[i] *= wc[i];
    return encodeKirchhoffAxialLayout(layout, wireForce, catheterForce);
}
