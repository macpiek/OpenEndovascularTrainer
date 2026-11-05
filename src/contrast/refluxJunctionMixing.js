// Local unresolved shear where antegrade parent blood meets a refluxing
// daughter. A pure outflow fork and confluence of two reversed daughters do
// not activate this counter-stream closure. No extra reservoir is introduced.
export function mixRefluxJunctionBoundaries(incoming, parentEdgeIndex, dt, shear) {
    if (!(dt > 0 && shear > 0) || incoming.length < 2) return false;
    const parent = incoming.find(a => a.edge.index === parentEdgeIndex);
    if (!parent || !(parent.q > 0)) return false;
    const children = incoming.filter(a => a !== parent && a.q > 0);
    if (!children.length) return false;
    for (const child of children) {
        const area = Math.min(parent.edge.areas[parent.cell], child.edge.areas[child.cell]);
        const radius = Math.sqrt(area / Math.PI);
        const distance = Math.max(1e-9, (parent.edge.cellLength + child.edge.cellLength) * .5);
        // Both streams must be present. Unlike a boolean reflux switch, this
        // conductance vanishes continuously as either interacting flow stops.
        const interactingFlow = 2 * parent.q * child.q / (parent.q + child.q);
        child.refluxConductance = shear * interactingFlow * radius / distance;
    }
    if (children.length === 1) {
        const child = children[0];
        const effectiveVolume = parent.volume * child.volume / (parent.volume + child.volume);
        const transfer = (child.c - parent.c) * effectiveVolume *
            -Math.expm1(-dt * child.refluxConductance / effectiveVolume);
        parent.mass += transfer;
        child.mass -= transfer;
        parent.c = parent.mass / parent.volume;
        child.c = child.mass / child.volume;
    } else {
        // Implicit star elimination couples all refluxing daughters to the
        // parent simultaneously, avoiding child-order bias at a trifurcation.
        let numerator = parent.mass, denominator = parent.volume;
        for (const child of children) {
            const k = dt * child.refluxConductance;
            numerator += k * child.mass / (child.volume + k);
            denominator += k * child.volume / (child.volume + k);
        }
        parent.c = numerator / denominator;
        parent.mass = parent.c * parent.volume;
        for (const child of children) {
            const k = dt * child.refluxConductance;
            child.c = (child.mass + k * parent.c) / (child.volume + k);
            child.mass = child.c * child.volume;
        }
    }
    return true;
}
