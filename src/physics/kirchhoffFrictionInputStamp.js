// Exact read-set guard for the position-history lumen-friction evaluator.
// No hash/tolerance comparison: a changed pose, material, feature, stencil or
// input invalidates reuse. Forces and their manifold basis are read fresh.
// Post-pass motion references are excluded:
// they affect the separate contact-motion stopping criterion, not friction.
const bodyScalars = ['count', 'segmentCount', 'radius', 'activeStart', 'activeEnd',
    'orientationControlCompliance', 'orientationControlSegment'];
const bodyArrays = ['x', 'y', 'z', 'previousX', 'previousY', 'previousZ',
    'orientationX', 'orientationY', 'orientationZ', 'orientationW',
    'previousOrientationX', 'previousOrientationY', 'previousOrientationZ', 'previousOrientationW',
    'nodeRadius', 'inverseMass', 'inverseInertia1', 'inverseInertia2', 'inverseInertia3'];
const recordScalars = ['id', 'kind', 'gap', 'innerT', 'outerT', 'surfaceGeometryVerified',
    '_innerSegmentIndex', '_outerSegmentIndex', '_innerNodeCount', '_outerNodeCount'];
const recordArrays = ['innerWeights', 'outerWeights', '_innerNodeIndices', '_outerNodeIndices',
    '_innerNodeWeights', '_outerNodeWeights'];
const contactScalars = ['innerSegmentIndex', 'outerSegmentIndex'];


export function updateKirchhoffFrictionInputStamp(constraint, dt, inverseMobility, stamp = {}, compare = true) {
    const values = stamp.values ??= [];
    let cursor = 0, bank = 0, same = compare && stamp.valid === true;
    const banks = stamp.banks ??= [];
    const put = value => { if (same && !Object.is(values[cursor], value)) same = false; values[cursor++] = value; };
    const array = value => {
        put(value); put(value?.length);
        if (value) for (let i = 0; i < value.length; i++) put(value[i]);
    };
    // Large rod arrays stay numeric: copying them uses a bulk typed-array
    // operation. Only a reuse request scans for equality, with exact signed-zero
    // semantics. Candidate evaluations only capture the next possible input.
    const bodyArray = value => {
        put(value); put(value?.length);
        const index = bank++;
        if (!value) { banks[index] = undefined; return; }
        let saved = banks[index];
        if (!saved || saved.length !== value.length) {
            saved = banks[index] = new Float64Array(value.length);
            same = false;
        }
        if (same) for (let i = 0; i < value.length; i++) {
            if (!Object.is(saved[i], value[i])) { same = false; break; }
        }
        saved.set(value);
    };
    const vector = value => {
        put(value);
        if (value) { put(value[0] ?? value.x); put(value[1] ?? value.y); put(value[2] ?? value.z); }
    };
    put(constraint); put(dt); put(inverseMobility);
    for (const key of ['innerRadius', 'axialFriction', 'circumferentialFriction', 'torsionalFriction']) put(constraint[key]);
    for (const body of [constraint.innerBody, constraint.outerBody]) {
        put(body);
        for (const key of bodyScalars) put(body?.[key]);
        for (const key of bodyArrays) bodyArray(body?.[key]);
    }
    put(constraint.kirchhoffContacts?.length ?? 0);
    for (const record of constraint.kirchhoffContacts ?? []) {
        put(record); put(record.manifoldContact);
        for (const key of recordScalars) put(record[key]);
        for (const key of recordArrays) array(record[key]);
        for (const key of ['normal', 'surfaceContactPoint', 'surfaceAxialTangent']) vector(record[key]);
        const contact = record.manifoldContact;
        if (contact) {
            for (const key of contactScalars) put(contact[key]);
        }
    }
    if (cursor !== values.length) same = false;
    values.length = cursor;
    // Velocity/split channels have a different read set; never reuse them.
    stamp.valid = !constraint.surfaceMotion && !constraint._splitMotion;
    return same && stamp.valid;
}
