/** Select real, active mechanical components without creating surrogate rods.
 * Enabled interaction owners define graph edges, even before their narrow-phase
 * contact batch is populated. Otherwise a newly contacting pair would be solved
 * independently during precisely the step when reciprocal reactions are needed.
 */
const worldWorkspaces = new WeakMap();

function active(body) {
    return Number.isInteger(body.activeStart) && Number.isInteger(body.activeEnd) &&
        Number.isInteger(body.segmentCount) &&
        Math.min(body.segmentCount, body.activeEnd) > Math.max(0, body.activeStart);
}

/** Returns component owners usable with kirchhoffComponentBodies(). Existing
 * containment objects remain authoritative owners of their lumen history.
 * Generated owners contain only real bodies and an empty lumen batch. Workspace
 * caches retain identity across split/merge cycles while those bodies remain in
 * the world; changing an active range does not change component identity.
 *
 * This adapter supports the existing one/two-body kernel. A larger connected
 * graph or multiple lumen owners is rejected, never silently split or dropped.
 */
export function selectKirchhoffMechanicalComponents(world, workspace) {
    if (!world || !Array.isArray(world.bodies) || !Array.isArray(world.containments ?? []) ||
        !Array.isArray(world.toolContacts ?? [])) throw new TypeError('A world with body and interaction arrays is required');
    if (!workspace) {
        workspace = worldWorkspaces.get(world);
        if (!workspace) worldWorkspaces.set(world, workspace = {});
    }
    const all = world.bodies;
    if (all.some(body => !body || typeof body !== 'object') || new Set(all).size !== all.length)
        throw new TypeError('World bodies must be distinct objects');
    const members = all.filter(active), present = new Set(all), indices = new Map(members.map((body, i) => [body, i]));
    const parent = members.map((_, i) => i);
    const root = i => { while (parent[i] !== i) i = parent[i]; return i; };
    const edges = [];
    const connect = (owner, a, b, lumen) => {
        if (!owner.enabled) return;
        if (!present.has(a) || !present.has(b) || a === b)
            throw new TypeError('Enabled interaction must join two distinct world bodies');
        if (!indices.has(a) || !indices.has(b)) return;
        const i = root(indices.get(a)), j = root(indices.get(b));
        parent[j] = i;
        edges.push({owner, a, lumen});
    };
    for (const owner of world.containments ?? []) connect(owner, owner.innerBody, owner.outerBody, true);
    for (const owner of world.toolContacts ?? []) connect(owner, owner.bodyA, owner.bodyB, false);
    const groups = new Map();
    members.forEach((body, i) => {
        const key = root(i);
        if (!groups.has(key)) groups.set(key, {bodies: [], lumens: []});
        groups.get(key).bodies.push(body);
    });
    for (const edge of edges) if (edge.lumen) groups.get(root(indices.get(edge.a))).lumens.push(edge.owner);
    // Validate the entire graph before changing the persistent component cache.
    for (const group of groups.values()) {
        if (group.bodies.length > 2) throw new RangeError('Mechanical kernel supports at most two connected bodies');
        if (group.lumens.length > 1) throw new RangeError('Mechanical component has multiple containment owners');
    }
    const cache = (workspace.components ?? []).filter(component => component.bodies.every(body => present.has(body)));
    const result = [];
    for (const group of groups.values()) {
        if (group.lumens.length) {
            result.push(group.lumens[0]);
            continue;
        }
        let component = cache.find(candidate => candidate.bodies.length === group.bodies.length &&
            group.bodies.every(body => candidate.bodies.includes(body)));
        if (!component) {
            component = {bodies: group.bodies.slice(), kirchhoffContacts: []};
            cache.push(component);
        }
        result.push(component);
    }
    // Re-entering a component after merge/split preserves its contact owner,
    // but its old equilibrium certificate no longer describes current loads.
    for (const component of result) if (!workspace.active?.has(component)) component._jointClosureConverged = false;
    workspace.active = new Set(result);
    workspace.components = cache;
    return result;
}
