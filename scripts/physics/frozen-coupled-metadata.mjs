// Read-only diagnostic identities. Object ids are run-local and do not imply
// material continuity: exported material labels/features/generations must also
// match before a numerical predictor treats two rows as the same variable.
export function createFrozenCoupledMetadata() {
    const ids = new WeakMap(); let nextId = 1;
    const objectId = object => {
        if (!object || typeof object !== 'object') return null;
        if (!ids.has(object)) ids.set(object, nextId++);
        return ids.get(object);
    };
    const vector = value => value ? Array.from(value) : null;
    function contact(contact, record) {
        return {
            objectId: objectId(contact), recordObjectId: objectId(record),
            id: contact?.id, recordId: record?.id, kind: record?.kind, feature: contact?.feature,
            innerMaterialSegmentId: contact?.innerMaterialSegmentId,
            outerMaterialSegmentId: contact?.outerMaterialSegmentId,
            innerSegmentIndex: contact?.innerSegmentIndex ?? record?._innerSegmentIndex,
            outerSegmentIndex: contact?.outerSegmentIndex ?? record?._outerSegmentIndex,
            innerT: record?.innerT, outerT: record?.outerT,
            generation: contact?.generation, lastSeenStep: contact?.lastSeenStep,
            normalLambda: contact?.normalLambda, tangentLambda: vector(contact?.tangentLambda),
            normal: vector(contact?.normal), tangentU: vector(contact?.tangentU), tangentV: vector(contact?.tangentV)
        };
    }
    function sourceRow(row) {
        return {
            ownerId: objectId(row.owner), kind: row.kind, side: row.side,
            node: row.node, segment: row.segment, joint: row.joint, component: row.component,
            bodyAId: objectId(row.bodyA), bodyBId: objectId(row.bodyB),
            segmentA: row.segmentA, segmentB: row.segmentB, tA: row.tA, tB: row.tB,
            reason: row.reason
        };
    }
    function friction(constraint) {
        const entries = [];
        for (const [scope, batch] of [['lumen', constraint._jointFrictionBatch],
            ['external', constraint._jointExternalFrictionBatch]]) {
            for (const entry of batch?.entries ?? []) entries.push({
                scope, ownerId: objectId(entry.owner ?? constraint.manifold),
                contact: contact(entry.contact, entry.record), generation: entry.generation,
                additionalRows: [batch.rowOffset + entry.rowStart, batch.rowOffset + entry.rowStart + 1],
                axes: entry.surface.axes.map(vector), forceSide: 0,
                lambda: entry.surface.rows.map(row => row.lambda),
                normalLambda: entry.surface.group.normalLambda,
                segmentA: entry.segmentA, segmentB: entry.segmentB, tA: entry.tA, tB: entry.tB,
                bodyAId: objectId(entry.bodyA), bodyBId: objectId(entry.bodyB)
            });
        }
        return entries;
    }
    function trialState(constraint) {
        return {
            // Fresh geometry may have remapped the manifold basis or applied
            // a certified cone repair after the common direction. These are
            // actual provisional multipliers, not lambda + guessed delta.
            contacts: constraint.kirchhoffContacts.map(record => contact(record.manifoldContact, record)),
            external: [...(constraint._coupledExternalFriction?.owners ?? [])].flatMap(([owner, contacts]) =>
                [...contacts].map(([index, value]) => ({ ownerId: objectId(owner), index, contact: contact(value) }))),
            material: [constraint.innerBody, constraint.outerBody].map(body => ({
                bodyId: objectId(body),
                adaptationLambdaX: vector(body.adaptationLambdaX), adaptationLambdaY: vector(body.adaptationLambdaY),
                adaptationLambdaZ: vector(body.adaptationLambdaZ), bendTwistLambda1: vector(body.bendTwistLambda1),
                bendTwistLambda2: vector(body.bendTwistLambda2), bendTwistLambda3: vector(body.bendTwistLambda3)
            }))
        };
    }
    return { objectId, contact, sourceRow, friction, trialState };
}
