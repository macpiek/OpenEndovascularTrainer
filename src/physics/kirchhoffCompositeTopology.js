import { condenseBundleSection, evaluateBundleSection } from './kirchhoffBundleModel.js';

/**
 * One common centerline, with independently advected material labels and
 * material rotations. No rod meshes, compliance conversion, contact solve or
 * identification of the two twist fields occurs here.
 *
 * x increases distally; insertion is the distal tip's x coordinate. Each
 * tool's s increases distally on materialInterval=[sProximal,sTip]:
 *     s(x) = sTip + dsDx * (x - insertion), dsDx > 0.
 * Energy/curvature/thetaPrime are per dx, as in kirchhoffBundleModel.
 */
const roles = ['wire', 'catheter'];
const finite = (value, name) => {
    if (!Number.isFinite(value)) throw new TypeError(`${name} must be finite`);
    return value;
};
function interval(value, name, allowEmpty = false) {
    if (!value || value.length !== 2) throw new TypeError(`${name} needs two endpoints`);
    const [a, b] = Array.from(value, v => finite(v, name));
    if (allowEmpty ? b < a : b <= a) throw new RangeError(`${name} must be ordered${allowEmpty ? '' : ' with positive length'}`);
    return [a, b];
}
function positive(value, name) {
    if (!(finite(value, name) > 0)) throw new RangeError(`${name} must be positive`);
    return value;
}
function optionalRadius(value, name) {
    return value == null ? null : positive(value, name);
}
function sampler(value, name) {
    if (typeof value !== 'function' && (!value || typeof value !== 'object'))
        throw new TypeError(`${name} requires a constitutive object or sampler`);
    return value;
}
function rotation(value = {}) {
    if (typeof value === 'number') value = { theta: value };
    if (!value || typeof value !== 'object') throw new TypeError('rotation requires theta and thetaPrime');
    const theta = value.theta ?? 0;
    if (typeof theta === 'function' && value.thetaPrime === undefined)
        throw new TypeError('A variable theta requires its independent thetaPrime rate per x');
    const thetaPrime = value.thetaPrime ?? 0;
    for (const [name, field] of Object.entries({ theta, thetaPrime }))
        if (typeof field !== 'function') finite(field, name);
    return { theta, thetaPrime };
}
function copyValue(value) {
    if (typeof value === 'function' || value == null || typeof value !== 'object') return value;
    if (Array.isArray(value) || ArrayBuffer.isView(value)) return Array.from(value, copyValue);
    return Object.fromEntries(Object.entries(value).map(([key, entry]) => [key, copyValue(entry)]));
}
function normalizeTool(input, id) {
    const materialInterval = interval(input.materialInterval, `${id}.materialInterval`);
    const insertion = finite(input.insertion, `${id}.insertion`);
    const dsDx = positive(input.dsDx ?? 1, `${id}.dsDx`);
    const start = insertion - (materialInterval[1] - materialInterval[0]) / dsDx;
    if (!Number.isFinite(start) || !(start < insertion)) throw new RangeError(`${id} coverage is not representable`);
    const radius = optionalRadius(input.radius, `${id}.radius`);
    const innerRadius = optionalRadius(input.innerRadius, `${id}.innerRadius`);
    if (radius !== null && innerRadius !== null && innerRadius > radius)
        throw new RangeError(`${id} innerRadius exceeds radius`);
    const materialBreakpoints = Array.from(input.materialBreakpoints ?? [], s => finite(s, 'material breakpoint'));
    if (materialBreakpoints.some(s => s < materialInterval[0] || s > materialInterval[1]))
        throw new RangeError(`${id} material breakpoint lies outside the tool`);
    return { id, materialInterval, insertion, dsDx, start, end: insertion,
        material: copyValue(sampler(input.material, `${id}.material`)),
        materialDerivative: input.materialDerivative == null ? null : copyValue(sampler(input.materialDerivative, 'materialDerivative')),
        materialBreakpoints, rotation: rotation(input.rotation), radius, innerRadius };
}
function materialAt(tool, x) {
    if (x === tool.end) return tool.materialInterval[1];
    if (x === tool.start) return tool.materialInterval[0];
    return tool.materialInterval[1] + tool.dsDx * (x - tool.insertion);
}
function variable(value) {
    return typeof value === 'function' || typeof value?.sample === 'function';
}
function resolve(value, s, context) {
    // A function may use context.side to select one-sided data at a material
    // jump. Objects retain sample(s,out); an optional third argument carries
    // the same context for profiles with discontinuities.
    return copyValue(typeof value === 'function' ? value(s, context)
        : typeof value?.sample === 'function' ? value.sample(s, {}, context) : value);
}
function clearance(outer, inner) {
    return outer === null || inner === null ? null : outer - inner;
}

/**
 * Exact positive-length sections. No epsilon snaps, midpoint classification,
 * averaging through moving tips, or synthetic material across uncovered gaps.
 * Defaults to the deployed domain [0,max(0,wireTip,catheterTip)].
 *
 * Contact entries identify surfaces and tool ownership only: they neither
 * activate a constraint nor assert a zero lumen offset. Negative clearance is
 * retained as incompatible geometry, never clamped into a fit. The distal
 * catheter portal is owned once by its boundary, including coincident tips.
 * Static inputs are copied; sampler functions must remain pure for a topology.
 */
export function buildKirchhoffCompositeTopology({ wire = null, catheter = null,
    interval: domain = null, sheath = null, boundaries = [] } = {}) {
    const inputs = { wire, catheter };
    const tools = roles.filter(id => inputs[id] !== null).map(id => normalizeTool(inputs[id], id));
    const [start, end] = interval(domain ?? [0, Math.max(0, ...tools.map(tool => tool.end))], 'interval', true);
    let normalizedSheath = null;
    if (sheath !== null) {
        const sheathInterval = interval(sheath.interval, 'sheath.interval');
        const id = sheath.id ?? 'sheath';
        if (typeof id !== 'string' || !id) throw new TypeError('sheath.id must be a nonempty string');
        normalizedSheath = { id, interval: sheathInterval, innerRadius: optionalRadius(sheath.innerRadius, 'sheath.innerRadius') };
    }
    const points = new Map();
    const add = (x, event) => {
        finite(x, 'boundary');
        if (x < start || x > end) return;
        if (!points.has(x)) points.set(x, { x, events: [], portal: null });
        points.get(x).events.push(event);
    };
    add(start, { kind: 'domain-start' }); add(end, { kind: 'domain-end' });
    for (const x of boundaries) add(x, { kind: 'external' });
    for (const tool of tools) {
        add(tool.start, { kind: 'material-start', tool: tool.id, s: tool.materialInterval[0] });
        add(tool.end, { kind: 'tip', tool: tool.id, s: tool.materialInterval[1] });
        for (const s of new Set(tool.materialBreakpoints)) {
            const x = s === tool.materialInterval[0] ? tool.start : s === tool.materialInterval[1]
                ? tool.end : tool.insertion + (s - tool.materialInterval[1]) / tool.dsDx;
            add(x, { kind: 'material-interface', tool: tool.id, s });
        }
    }
    if (normalizedSheath) {
        add(normalizedSheath.interval[0], { kind: 'sheath-start', sheath: normalizedSheath.id });
        add(normalizedSheath.interval[1], { kind: 'sheath-end', sheath: normalizedSheath.id });
    }
    const ordered = [...points.values()].sort((a, b) => a.x - b.x);
    const sections = [], gaps = [];
    for (let i = 0; i + 1 < ordered.length; i++) {
        const a = ordered[i].x, b = ordered[i + 1].x;
        const active = tools.filter(tool => a >= tool.start && b <= tool.end);
        if (!active.length) { gaps.push({ start: a, end: b }); continue; }
        const outer = active.find(tool => tool.id === 'catheter') ?? active[0];
        const inSheath = normalizedSheath && a >= normalizedSheath.interval[0] && b <= normalizedSheath.interval[1];
        const inner = active.find(tool => tool.id === 'wire');
        const lumen = active.length === 2 ? { kind: 'lumen', inner: 'wire', outer: 'catheter',
            clearance: clearance(outer.innerRadius, inner.radius) } : null;
        sections.push({ start: a, end: b, kind: active.length === 2 ? 'overlap' : `${outer.id}-only`,
            startBoundary: i, endBoundary: i + 1,
            tools: active.map(tool => ({ ...tool, sStart: materialAt(tool, a), sEnd: materialAt(tool, b) })),
            contacts: {
                wall: inSheath ? null : { kind: 'wall', owner: outer.id, radius: outer.radius },
                sheath: inSheath ? { kind: 'sheath', owner: outer.id, sheath: normalizedSheath.id,
                    radius: outer.radius, clearance: clearance(normalizedSheath.innerRadius, outer.radius) } : null,
                lumen
            } });
    }
    const inner = tools.find(tool => tool.id === 'wire'), outer = tools.find(tool => tool.id === 'catheter');
    if (inner && outer && outer.end > inner.start && outer.end <= inner.end && points.has(outer.end)) {
        points.get(outer.end).portal = { kind: 'distal-portal', inner: 'wire', outer: 'catheter',
            x: outer.end, innerMaterial: materialAt(inner, outer.end), outerMaterial: outer.materialInterval[1],
            crossing: outer.end < inner.end, coincidentTips: outer.end === inner.end,
            clearance: clearance(outer.innerRadius, inner.radius) };
    }
    return { interval: [start, end], tools, boundaries: ordered, sections, gaps,
        connected: sections.every((section, i) => i === 0 || section.start === sections[i - 1].end),
        coversDomain: gaps.length === 0, coordinate: 'common-centerline',
        twistToolIds: tools.map(tool => tool.id), slidingToolIds: tools.map(tool => tool.id) };
}

/**
 * Snapshot BundleModel-ready material inputs at one point of a section.
 * thetaPrime is always per x. Overrides update individual rotations only.
 * At interfaces, sample the section's interior side without moving x or s.
 * Unknown material derivatives remain unknown (null dS in BundleModel).
 */
export function sampleKirchhoffCompositeSection(section, { x = section.start + (section.end - section.start) / 2,
    rotations = {} } = {}) {
    finite(x, 'x');
    if (x < section.start || x > section.end) throw new RangeError('x lies outside the section');
    const side = x === section.end ? 'left' : x === section.start ? 'right' : null;
    return section.tools.map(tool => {
        const s = materialAt(tool, x), context = { x, side, tool: tool.id, sStart: tool.sStart, sEnd: tool.sEnd };
        const state = rotation(rotations[tool.id] ?? tool.rotation);
        const materialSample = resolve(tool.material, s, context);
        const materialDerivative = tool.materialDerivative === null ? null : resolve(tool.materialDerivative, s, context);
        return { id: tool.id, s, dsDx: tool.dsDx,
            theta: finite(typeof state.theta === 'function' ? state.theta(s, context) : state.theta, 'theta'),
            thetaPrime: finite(typeof state.thetaPrime === 'function' ? state.thetaPrime(s, context) : state.thetaPrime, 'thetaPrime'),
            // Preserve the oracle's distinction between a homogeneous constant
            // material and a variable profile with an unknown derivative.
            material: variable(tool.material) ? () => materialSample : materialSample,
            materialDerivative };
    });
}

/** Energy density and separate material torques; never sums compliance/GJ DOFs. */
export function evaluateKirchhoffCompositeSection(section, { curvature = [0, 0], frameTwist = 0, ...sample } = {}) {
    return evaluateBundleSection({ curvature, frameTwist, tools: sampleKirchhoffCompositeSection(section, sample) });
}

/** Eliminate only common bending; retain mismatch energy and both twist torques. */
export function condenseKirchhoffCompositeSection(section, { frameTwist = 0, ...sample } = {}) {
    return condenseBundleSection({ frameTwist, tools: sampleKirchhoffCompositeSection(section, sample) });
}

/**
 * Bridge existing profiles sampled by distance PROXIMALLY from the tip to the
 * distally increasing material label. This reverses only the lookup label,
 * not the physical tangent/directors or intrinsic curvature components.
 * tipBreakpoints are profile interfaces in distance-from-tip units. Include
 * every constitutive jump/shape interface; sampling does not discover them.
 * A discontinuous profile can use sample(distance,out,context). context.side
 * is reversed into distance-from-tip coordinates at the exact interface.
 */
export function compositeToolFromTipProfile({ profile, tipBreakpoints = [], ...tool }) {
    if (!profile || typeof profile.sample !== 'function') throw new TypeError('profile.sample is required');
    const [s0, sTip] = interval(tool.materialInterval, 'materialInterval');
    const mapped = tipBreakpoints.map(distance => {
        finite(distance, 'tip breakpoint');
        if (distance < 0 || distance > sTip - s0) throw new RangeError('tip breakpoint lies outside the tool');
        return sTip - distance;
    });
    return { ...tool, material: (s, context) => profile.sample(sTip - s, {}, {
        ...context, side: context?.side === 'left' ? 'right' : context?.side === 'right' ? 'left' : null
    }),
        materialBreakpoints: [...(tool.materialBreakpoints ?? []), ...mapped] };
}
