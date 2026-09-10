/** Read-only, owned diagnostic snapshot at a fixed-step boundary. Browser-safe:
 * no filesystem, Node, Three.js, world refresh or solver dependencies. Pass
 * capture-time source hashes in provenance when execution-source identity is
 * required; this helper cannot infer those hashes from live class instances.
 * Use stringifyCoupledContactAudit to preserve nonfinite values in JSON. */
export function captureCoupledContactAudit(world, constraint = world.containments.find(c => c.enabled), provenance = {}) {
    if (!constraint) return { schemaVersion: 1, step: world.stepCount, constraint: null, provenance: copy(provenance) };
    const bodies = [constraint.innerBody, constraint.outerBody];
    const array = (v, end) => v == null ? null : Array.from(v).slice(0, end);
    const stencil = (r, side) => {
        const prefix = side ? '_outer' : '_inner', segment = r[prefix + 'SegmentIndex'];
        const nodes = r[prefix + 'NodeIndices'];
        const count = nodes ? r[prefix + 'NodeCount'] : 2;
        return { segment, nodes: nodes ? array(nodes, count) : [segment, segment + 1],
            weights: array(nodes ? r[prefix + 'NodeWeights'] : r[side ? 'outerWeights' : 'innerWeights'], count) };
    };
    const record = r => {
        const contact = r.manifoldContact, fn = contact?.normalLambda ?? 0, alpha = r._normalAlpha ?? 0;
        const stencils = [stencil(r, 0), stencil(r, 1)];
        const centers = stencils.map((s, side) => ['x', 'y', 'z'].map(axis => s.nodes.reduce((sum, node, i) => sum + s.weights[i] * bodies[side][axis][node], 0)));
        let offset = 0;
        for (let i = 0; i < 3; i++) offset += (centers[0][i] - centers[1][i]) * r.normal[i];
        const adjustedGap = r.gap - (offset - r._normalReference);
        return { id: r.id, kind: r.kind, feature: r.feature, gap: r.gap, adjustedGap,
            normalAlpha: alpha, normalLambda: fn, alphaLambda: alpha * fn, equilibriumResidual: adjustedGap + alpha * fn,
            containedSpanFraction: r._containedSpanFraction, normalReference: r._normalReference,
            violation: r.violation, normal: array(r.normal), innerT: r.innerT, outerT: r.outerT,
            radius: r.radialDistance, clearance: r.clearance, stencils, centers,
            manifold: contact ? { id: contact.id, normalLambda: fn, tangentLambda: array(contact.tangentLambda),
                normal: array(contact.normal), tangentU: array(contact.tangentU), tangentV: array(contact.tangentV),
                innerMaterialSegmentId: contact.innerMaterialSegmentId, outerMaterialSegmentId: contact.outerMaterialSegmentId,
                innerSegmentIndex: contact.innerSegmentIndex, outerSegmentIndex: contact.outerSegmentIndex } : null };
    };
    const current = new Set(constraint.kirchhoffContacts.map(r => r.manifoldContact));
    const retainedUnlisted = [...constraint.manifold.contacts()].filter(c => !current.has(c) &&
        (c.normalLambda !== 0 || c.tangentLambda[0] !== 0 || c.tangentLambda[1] !== 0)).map(c => ({
            id: c.id, normalLambda: c.normalLambda, tangentLambda: array(c.tangentLambda),
            innerSegmentIndex: c.innerSegmentIndex, outerSegmentIndex: c.outerSegmentIndex, lastSeenStep: c.lastSeenStep }));
    const scalars = {};
    for (const key of ['enabled', 'innerRadius', 'compliance', 'portalFilletRadius', 'openDistal', 'openProximal',
        'startNode', 'endNode', 'outerStartNode', 'innerArcOffset', 'containedLength', 'searchWindow',
        'kirchhoffContactActivation', 'kirchhoffMaxViolation', 'kirchhoffSolverResidual', 'kirchhoffContactMotion',
        'kirchhoffMeasuredSideViolation', 'kirchhoffMeasuredSpatialPortalViolation', 'kirchhoffMeasuredPortalViolation',
        'kirchhoffMeasuredWorstSide']) scalars[key] = copy(constraint[key]);
    return { schemaVersion: 1, step: world.stepCount, solver: world.lastCoupledSolver,
        converged: world.lastCoupledClosureConverged, provenance: copy(provenance),
        constraint: scalars, records: constraint.kirchhoffContacts.map(record), retainedUnlisted,
        outerSegmentByInner: array(constraint.kirchhoffOuterSegmentByInner, bodies[0].activeEnd + 1),
        outerArcAtNode: array(constraint._kirchhoffOuterArcAtNode, bodies[1].activeEnd + 1),
        bodies: bodies.map(b => ({ id: b.id, count: b.count, segmentCount: b.segmentCount,
            activeStart: b.activeStart, activeEnd: b.activeEnd, segmentLength: b.segmentLength,
            x: array(b.x, b.activeEnd + 1), y: array(b.y, b.activeEnd + 1), z: array(b.z, b.activeEnd + 1),
            nodeRadius: array(b.nodeRadius, b.activeEnd + 1), restLength: array(b.restLength, b.activeEnd),
            materialCoordinate: array(b.materialCoordinate, b.activeEnd + 1) })) };
}

function copy(value) {
    if (ArrayBuffer.isView(value) || Array.isArray(value)) return Array.from(value, copy);
    if (value && typeof value === 'object') return Object.fromEntries(Object.entries(value).map(([key, item]) => [key, copy(item)]));
    return value;
}

/** String tags Infinity/-Infinity/NaN avoid JSON's ambiguous conversion to null. */
export function stringifyCoupledContactAudit(audit, space = 2) {
    return JSON.stringify(audit, (key, value) => typeof value === 'number' && !Number.isFinite(value) ? String(value) : value, space);
}
