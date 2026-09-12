import { buildKirchhoffCompositeMesh } from './kirchhoffCompositeMesh.js';

/** Incremental material refresh on an accepted, fixed spatial mesh. This owns
 * no material advection or frame/spin transfer. Fields and layout stay exactly
 * the same objects. A tip/material/sheath boundary missing from the accepted
 * coordinates, or changed tool ownership, returns an explicit candidate and
 * leaves ALL accepted state untouched. The caller must perform a separately
 * validated remesh/history transfer before using such a candidate at runtime.
 *
 * The original builder is the structural oracle and performs an atomic dry
 * build before committing material/maps/contact metadata. Its optional cached
 * integrator removes repeated quadrature; it never weakens error tolerances.
 * Even a successful material-only refresh does NOT approve common-axis
 * clearance, update velocity history, transport spins or certify contacts.
 */
export function createKirchhoffCompositeMeshUpdate(mesh, { materialIntegrator = null, quadrature = {} } = {}) {
    if (!mesh?.data?.coordinates || !mesh?.layout || !mesh?.materialMaps)
        throw new TypeError('An accepted composite mesh is required');
    const coordinates = mesh.data.coordinates;
    function plan(topology) {
        if (!topology?.sections?.length || !topology.coversDomain || !topology.connected)
            throw new RangeError('A material refresh requires a continuously covered topology');
        const missingBoundaries = topology.boundaries.filter(b => !coordinates.includes(b.x)).map(b => b.x);
        const domainChanged = topology.interval[0] !== coordinates[0] || topology.interval[1] !== coordinates.at(-1);
        let cursor = 0, ownershipChanged = domainChanged;
        if (!missingBoundaries.length && !domainChanged) for (let edge = 0; edge + 1 < coordinates.length; edge++) {
            while (cursor < topology.sections.length && coordinates[edge] >= topology.sections[cursor].end) cursor++;
            const section = topology.sections[cursor], ids = mesh.layout.edgeToolIds[edge];
            if (!section || coordinates[edge + 1] > section.end || section.tools.length !== ids.length ||
                section.tools.some((tool, i) => tool.id !== ids[i])) { ownershipChanged = true; break; }
        }
        const requiresStateTransfer = missingBoundaries.length > 0 || ownershipChanged;
        return { status: requiresStateTransfer ? 'candidate-needs-state-transfer' : 'material-refresh-ready',
            requiresStateTransfer, missingBoundaries, domainChanged, ownershipChanged,
            proposedCoordinates: requiresStateTransfer ? Float64Array.from([...new Set([
                ...Array.from(coordinates).filter(x => x >= topology.interval[0] && x <= topology.interval[1]),
                ...topology.boundaries.map(b => b.x)])].sort((a, b) => a - b)) : coordinates,
            certified: false };
    }
    function refresh(topology) {
        const proposal = plan(topology);
        if (proposal.requiresStateTransfer) return { ...proposal, updated: false, mesh };
        const indexByCoordinate = new Map(Array.from(coordinates, (x, i) => [x, i]));
        const spinFields = {}, referenceTwistFields = {};
        for (const tool of mesh.data.tools) {
            spinFields[tool.id] = (_, context) => tool.angles[context.edge];
            referenceTwistFields[tool.id] = (_, context) => tool.referenceTwists[context.vertex - 1];
        }
        // Any quadrature/validation exception occurs before touching accepted
        // data; no partial material or map update leaks into a retry.
        const draft = buildKirchhoffCompositeMesh({ topology, meshCoordinates: coordinates, boundaryPolicy: 'reject',
            sampleCenterline: x => mesh.data.positions[indexByCoordinate.get(x)], spinFields,
            referenceFrames: mesh.data.reference, referenceTwistFields, materialIntegrator, quadrature });
        if (draft.materialCells.length !== mesh.materialCells.length || draft.boundaryCells.length !== mesh.boundaryCells.length)
            throw new RangeError('Material refresh unexpectedly changed mesh support ownership');
        for (let i = 0; i < draft.materialCells.length; i++) {
            const a = draft.materialCells[i], b = mesh.materialCells[i];
            if (a.id !== b.id || a.vertex !== b.vertex || a.nominalStart !== b.nominalStart || a.nominalEnd !== b.nominalEnd)
                throw new RangeError('Material refresh unexpectedly changed nominal Voronoi supports');
        }
        for (let i = 0; i < draft.boundaryCells.length; i++) {
            const a = draft.boundaryCells[i], b = mesh.boundaryCells[i];
            if (a.id !== b.id || a.side !== b.side || a.start !== b.start || a.end !== b.end)
                throw new RangeError('Material refresh unexpectedly changed boundary half-cells');
        }
        for (let i = 0; i < draft.materialCells.length; i++) Object.assign(mesh.materialCells[i], draft.materialCells[i]);
        for (let i = 0; i < draft.boundaryCells.length; i++) Object.assign(mesh.boundaryCells[i], draft.boundaryCells[i]);
        for (const [id, maps] of draft.materialMaps) {
            const old = mesh.materialMaps.get(id);
            for (let i = 0; i < maps.length; i++) if (maps[i]) Object.assign(old[i], maps[i]);
            mesh.data.tools.find(tool => tool.id === id).dsDx = draft.data.tools.find(tool => tool.id === id).dsDx;
        }
        // Earlier section approvals refer to earlier topology objects. A
        // material update may never carry them across to different contacts.
        mesh.edgeSections = draft.edgeSections; mesh.contactOwners = draft.contactOwners;
        mesh.admission = draft.admission; mesh.quadrature = draft.quadrature;
        return { ...proposal, updated: true, mesh };
    }
    return { mesh, plan, refresh };
}
