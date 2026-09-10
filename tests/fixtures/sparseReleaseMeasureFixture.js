/** Deterministic measurement-only fixture: sparse world wrenches over long
 * rods, anisotropic inertia, rotated frames and nonzero activeStart. No solver,
 * World step or anatomy is involved in this local cost/parity measurement. */
export function sparseReleaseMeasureFixture({ counts = [200, 200], releases = 400,
    activeStart = [7, 11], paired = false, roundingSensitive = false } = {}) {
    const bodies = counts.map((count, side) => {
        const b = { id: 'measure-' + side, count, segmentCount: count - 1, activeStart: activeStart[side], activeEnd: count - 3,
            sleeping: false, orientationControlCompliance: side ? .125 : 0, orientationControlSegment: activeStart[side] + 2 };
        for (const key of ['inverseMass', 'inverseInertia1', 'inverseInertia2', 'inverseInertia3', 'orientationX', 'orientationY', 'orientationZ', 'orientationW'])
            b[key] = new Float64Array(count);
        for (let node = 0; node < count; node++) {
            b.inverseMass[node] = node % 19 === 0 ? 0 : .5 + (node % 7) * .125;
            for (let axis = 1; axis <= 3; axis++) b['inverseInertia' + axis][node] = .125 * axis + (node % 5) * .0625;
            const axis = [.7 + side, Math.sin(node * .37), Math.cos(node * .23)], norm = Math.hypot(...axis), angle = .13 * node + .21 * side;
            ['X', 'Y', 'Z'].forEach((a, i) => { b['orientation' + a][node] = axis[i] * Math.sin(angle * .5) / norm; });
            b.orientationW[node] = Math.cos(angle * .5);
        }
        return b;
    });
    const retired = Array.from({ length: releases }, (_, index) => {
        const seed = paired ? Math.floor(index / 2) : index, sign = paired && index % 2 ? -1 : 1;
        const gradients = [], guards = [];
        for (let side = 0; side < 2; side++) {
            const b = bodies[side], start = b.activeStart + (seed * 17 + side * 7) % (b.activeEnd - b.activeStart - 1);
            for (let offset = 0; offset < 2; offset++) {
                const node = start + offset;
                for (let axis = 0; axis < 6; axis++) gradients.push({ side, dof: node * 6 + axis,
                    value: sign * Math.sin((seed + 1) * .31 + side + offset + axis * .27) * (axis < 3 ? .2 : .03) });
                // Real duplicate DOFs; the native gradient helper coalesces
                // angular duplicates but preserves translational ones.
                gradients.push({ side, dof: node * 6, value: sign * .125 }, { side, dof: node * 6, value: -sign * .0625 },
                    { side, dof: node * 6 + 4, value: sign * .015625 });
                guards.push({ object: b, key: 'inverseMass', at: node, value: b.inverseMass[node] });
                if (roundingSensitive && seed % 3 === 0) gradients.push({ side, dof: node * 6 + 1, value: sign * 2 ** 54 },
                    { side, dof: node * 6 + 1, value: sign }, { side, dof: node * 6 + 1, value: -sign * 2 ** 54 });
            }
            // Existing mobility masks also see sparse entries outside the
            // active interval and at the final translational node.
            gradients.push({ side, dof: (b.activeStart - 1) * 6 + 2, value: sign * .25 },
                { side, dof: b.activeEnd * 6 + 1, value: sign * .125 });
        }
        return { id: index + 1, mode: seed % 2 ? 'difference' : 'full', kind: 'normal', key: 'measure-' + seed,
            owner: bodies[seed % 2], side: seed % 2, node: seed % counts[seed % 2], reason: 'measure-fixture',
            bank: { lambda: .125 + seed % 7 * .03125 }, slot: 'lambda', historyGuards: guards, frozenWorldGradients: gradients };
    });
    return { innerBody: bodies[0], outerBody: bodies[1], _splitMotion: { twoChannel: { rows: { releases: retired } } } };
}

/** Measurement must not replace or mutate any owned state. Value snapshots
 * alone would miss replacement by equal arrays; retain these references too. */
export function releaseMeasureOwnedReferences(joint) {
    const state = joint._splitMotion.twoChannel.rows;
    const refs = [joint, joint.innerBody, joint.outerBody, joint._splitMotion, joint._splitMotion.twoChannel, state, state.releases];
    for (const body of [joint.innerBody, joint.outerBody]) refs.push(...Object.values(body).filter(v => ArrayBuffer.isView(v)));
    for (const release of state.releases) refs.push(release, release.bank, release.historyGuards, ...(release.historyGuards ?? []),
        release.frozenWorldGradients, ...release.frozenWorldGradients);
    return refs;
}

export function measureFloat64Allocations(measure, joint) {
    const Original = globalThis.Float64Array, lengths = [];
    globalThis.Float64Array = new Proxy(Original, { construct(target, args) {
        const array = Reflect.construct(target, args); lengths.push(array.length); return array;
    } });
    try { measure(joint); }
    finally { globalThis.Float64Array = Original; }
    return { arrays: lengths.length, doubles: lengths.reduce((a, n) => a + n, 0), bytes: lengths.reduce((a, n) => a + n * 8, 0), lengths };
}

/** Canonical logical graph with exact IEEE754 numbers/backing-buffer bytes.
 * v8.serialize alone also encodes packed-vs-holey Array internals, which can
 * change after JIT warmup without changing a single output value/property.
 * Array holes, property order, types and shared object/buffer references are
 * still explicit here; no numeric JSON rounding or NaN/-0 conversion occurs. */
export function releaseMeasureBytes(value) {
    const seen = new Map();
    function encode(v) {
        if (v === null) return ['null'];
        const type = typeof v;
        if (type === 'number') { const bytes = Buffer.allocUnsafe(8); bytes.writeDoubleLE(v); return ['number', bytes.toString('hex')]; }
        if (['string', 'boolean', 'undefined'].includes(type)) return type === 'undefined' ? [type] : [type, v];
        if (type !== 'object') throw new TypeError('Unsupported parity value: ' + type);
        if (seen.has(v)) return ['reference', seen.get(v)];
        const id = seen.size; seen.set(v, id);
        if (v instanceof ArrayBuffer) return ['ArrayBuffer', id, Buffer.from(v).toString('base64')];
        if (ArrayBuffer.isView(v)) return [v.constructor.name, id, v.byteOffset, v.byteLength, encode(v.buffer)];
        if (v instanceof Map) return ['Map', id, [...v].map(([k, item]) => [encode(k), encode(item)])];
        if (v instanceof Set) return ['Set', id, [...v].map(encode)];
        return [Array.isArray(v) ? 'Array' : 'Object', id, Array.isArray(v) ? v.length : null,
            Object.keys(v).map(key => [key, encode(v[key])])];
    }
    return Buffer.from(JSON.stringify(encode(value)));
}
