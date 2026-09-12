import { evaluateKirchhoffSlidingPortal } from './kirchhoffSlidingPortal.js';
import { rotateVectorByQuaternion, inverseRotateVectorByQuaternion } from './discreteKirchhoffRod.js';

/** Trace the connected bore branch from the fractional material end. Overlap
 * is an anchor, not an equality of arclengths. Walk adjacent wire segments to
 * the first outward mouth crossing without leaving the bore. Never jump to a
 * spatial nearest returning loop or add an artificial index allowance.
 * Failure to certify this branch keeps external contact. This is not a full
 * annular detector for an oblique finite-radius wire. */
export function locateKirchhoffDistalLumenBranch(joint, out = {}) {
    out.segment = -1;
    if (!joint?.enabled || !joint.enforceDistalPortal || !joint.openDistal ||
        !Number.isFinite(joint.containedLength) || !(joint.innerRadius > 0)) return out;
    const inner = joint.innerBody, outer = joint.outerBody, tip = outer.activeEnd;
    if (tip <= outer.activeStart || inner.activeEnd <= inner.activeStart) return out;
    const dx = outer.x[tip] - outer.x[tip - 1], dy = outer.y[tip] - outer.y[tip - 1], dz = outer.z[tip] - outer.z[tip - 1];
    const length = Math.hypot(dx, dy, dz);
    if (!(length > 0)) return out;
    const nx = dx / length, ny = dy / length, nz = dz / length, radius2 = joint.innerRadius ** 2;
    function point(segment, t) {
        const x = inner.x[segment] * (1 - t) + inner.x[segment + 1] * t - outer.x[tip];
        const y = inner.y[segment] * (1 - t) + inner.y[segment + 1] * t - outer.y[tip];
        const z = inner.z[segment] * (1 - t) + inner.z[segment + 1] * t - outer.z[tip];
        const axial = x * nx + y * ny + z * nz;
        return { axial, radial2: (x - axial * nx) ** 2 + (y - axial * ny) ** 2 + (z - axial * nz) ** 2 };
    }
    let remaining = joint.containedLength - Math.max(0, joint.innerArcOffset ?? 0);
    let segment = Math.max(inner.activeStart, joint.startNode), t = 0;
    if (!(remaining >= 0)) return out;
    for (; segment < inner.activeEnd; segment++) {
        const h = inner.restLength[segment];
        if (!(h > 0)) return out;
        if (remaining <= h) { t = remaining / h; break; }
        remaining -= h;
    }
    if (segment >= inner.activeEnd) return out;
    let current = point(segment, t);
    if (!(current.radial2 < radius2)) return out;
    const backwards = current.axial > 0;
    for (; segment >= inner.activeStart && segment < inner.activeEnd; segment += backwards ? -1 : 1) {
        const a = point(segment, 0), b = point(segment, 1), axialDelta = b.axial - a.axial;
        if (axialDelta > 0) {
            const crossing = -a.axial / axialDelta;
            if (crossing >= 0 && crossing <= 1 && (backwards ? crossing <= t : crossing >= t)) {
                if (point(segment, crossing).radial2 < radius2) { out.segment = segment; out.t = crossing; }
                return out;
            }
        }
        current = backwards ? a : b;
        if (!(current.radial2 < radius2)) return out;
        t = backwards ? 1 : 0;
    }
    return out;
}

/** Keep the existing sliding-aperture geometry on the certified material
 * branch. Otherwise its nearest search could still attach a returning loop
 * to the lumen while the external detector correctly keeps that loop out. */
export function evaluateKirchhoffOwnedSlidingPortal(joint, out = {}) {
    evaluateKirchhoffSlidingPortal(joint, out);
    if (out.segment < 0) return out; // Preserve the withdrawn-wire-tip guard.
    const branch = locateKirchhoffDistalLumenBranch(joint);
    if (branch.segment < 0) {
        // No certified lumen branch means external ownership. A nearest loop
        // cannot supply either an aperture row or an external-contact exemption.
        out.segment = -1; out.distance = Infinity; out.violation = 0;
        return out;
    }
    if (branch.segment === out.segment) return out;
    const inner = joint.innerBody, outer = joint.outerBody, s = branch.segment, tip = outer.activeEnd;
    const dx = inner.x[s + 1] - inner.x[s], dy = inner.y[s + 1] - inner.y[s], dz = inner.z[s + 1] - inner.z[s];
    const t = Math.max(0, Math.min(1, ((outer.x[tip] - inner.x[s]) * dx +
        (outer.y[tip] - inner.y[s]) * dy + (outer.z[tip] - inner.z[s]) * dz) / (dx * dx + dy * dy + dz * dz)));
    out.segment = s; out.t = t;
    out.x = inner.x[s] + t * dx - outer.x[tip];
    out.y = inner.y[s] + t * dy - outer.y[tip];
    out.z = inner.z[s] + t * dz - outer.z[tip];
    out.distance = Math.hypot(out.x, out.y, out.z);
    out.clearance = Math.max(0, joint.innerRadius - Math.max(inner.nodeRadius[s], inner.nodeRadius[s + 1]));
    out.violation = Math.max(0, out.distance - out.clearance);
    return out;
}

/** Geometry only. Applied force is retired by an explicit common solve row;
 * neither this predicate nor geometry refresh clears a multiplier. */
export function isKirchhoffDistalLumenWitness(joint, tool, segmentA, segmentB, tA, x, y, z,
    branch = null) {
    if (!joint?.enabled || !joint.enforceDistalPortal || !joint.openDistal || !tool.openDistalB ||
        tool.bodyA !== joint.innerBody || tool.bodyB !== joint.outerBody) return false;
    branch ??= locateKirchhoffDistalLumenBranch(joint);
    const outer = joint.outerBody, tip = outer.activeEnd;
    if (branch.segment < 0 || segmentB !== tip - 1) return false;
    // Beyond the material side/rim collector the sliding row must represent
    // THIS certified branch before it can receive ownership.
    if (branch.segment > joint.endNode && joint._slidingPortalState?.segment !== branch.segment) return false;
    if (segmentA !== branch.segment && !(tA === 0 && segmentA === branch.segment + 1) &&
        !(tA === 1 && segmentA + 1 === branch.segment)) return false;
    const dx = outer.x[tip] - outer.x[tip - 1], dy = outer.y[tip] - outer.y[tip - 1], dz = outer.z[tip] - outer.z[tip - 1];
    const length = Math.hypot(dx, dy, dz);
    if (!(length > 0)) return false;
    const px = x - outer.x[tip], py = y - outer.y[tip], pz = z - outer.z[tip];
    const axial = (px * dx + py * dy + pz * dz) / length;
    return (px - axial * dx / length) ** 2 + (py - axial * dy / length) ** 2 +
        (pz - axial * dz / length) ** 2 < joint.innerRadius ** 2;
}

export function beginKirchhoffToolReactionStep(tool) {
    for (const reaction of tool._jointReactions?.values() ?? []) {
        reaction.wrenches.length = reaction.normalWrenches.length = 0;
        reaction.retiring = false;
    }
}

const wrenchKeys = ['fx', 'fy', 'fz', 'mx', 'my', 'mz'];
function wrenchFor(storage, side, node) {
    let value = storage.find(w => w.side === side && w.node === node);
    if (!value) storage.push(value = { side, node, fx: 0, fy: 0, fz: 0, mx: 0, my: 0, mz: 0 });
    return value;
}
function frame(body, node) {
    return { x: body.orientationX[node], y: body.orientationY[node],
        z: body.orientationZ[node], w: body.orientationW[node] };
}

/** Freeze unit world force and moment about the fixed world origin at row
 * assembly, BEFORE the joint update changes positions and local-right frames.
 * A later multiplier commit must use this snapshot, not its post-apply pose. */
export function captureKirchhoffToolReaction(joint, gradients, out = []) {
    // Rows keep their bounded node stencil across relinearizations. Reuse the
    // records, including zero-weight feet, so frozen trial snapshots retain
    // stable identities without allocating another object per contact foot.
    for (const value of out) for (const key of wrenchKeys) value[key] = 0;
    for (const g of gradients) {
        if (!g.value) continue;
        const node = Math.floor(g.dof / 6), axis = g.dof % 6;
        const body = g.side ? joint.outerBody : joint.innerBody;
        if (axis >= 3 && node >= Math.min(body.segmentCount, body.activeEnd))
            throw new RangeError('A terminal translation node has no material rotation');
        const value = wrenchFor(out, g.side, node);
        value[wrenchKeys[axis]] += g.value;
    }
    for (const value of out) {
        const body = value.side ? joint.outerBody : joint.innerBody, node = value.node;
        const spin = node < Math.min(body.segmentCount, body.activeEnd)
            ? rotateVectorByQuaternion(frame(body, node), { x: value.mx, y: value.my, z: value.mz })
            : { x: 0, y: 0, z: 0 };
        value.mx = spin.x + body.y[node] * value.fz - body.z[node] * value.fy;
        value.my = spin.y + body.z[node] * value.fx - body.x[node] * value.fz;
        value.mz = spin.z + body.x[node] * value.fy - body.y[node] * value.fx;
    }
    return out;
}

/** Journal the actual, commonly scaled increments as spatial wrenches.
 * Contributions from different positions/frames have a common origin/basis.
 * This owned mechanical state is included in nonlinear rollback. */
export function recordKirchhoffToolReaction(tool, index, wrenches, delta, normal = false) {
    if (!delta) return;
    // An unfrozen gradient array is not a spatial snapshot. Fail before any
    // owned state changes instead of silently interpreting the old API as NaN.
    if (!wrenches || !Number.isFinite(delta) || wrenches.some(w => wrenchKeys.some(k => !Number.isFinite(w[k]))))
        throw new TypeError('Tool reaction commit requires frozen finite world wrenches');
    const reactions = tool._jointReactions ??= new Map();
    let reaction = reactions.get(index);
    if (!reaction) reactions.set(index, reaction = { owner: tool, node: index,
        wrenches: [], normalWrenches: [], retiring: false });
    for (const source of wrenches) {
        const value = wrenchFor(reaction.wrenches, source.side, source.node);
        const normalValue = normal ? wrenchFor(reaction.normalWrenches, source.side, source.node) : null;
        for (const key of wrenchKeys) {
            value[key] += delta * source[key];
            if (normalValue) normalValue[key] += delta * source[key];
        }
    }
}

function addGradient(storage, side, dof, value) {
    if (!value) return;
    let target = storage.find(g => g.side === side && g.dof === dof);
    if (!target) storage.push(target = { side, dof, value: 0 });
    target.value += value;
}

/** Express the retained spatial action in CURRENT solver coordinates. The
 * final active node has translation only: its transported couple belongs to
 * the adjacent last material frame, never to a nonexistent terminal rotation.
 * This preserves each rod's world force/moment and unchanged-pose J^T action;
 * it is not an inverse of interleaved finite SO(3) position updates. */
function currentGradients(joint, wrenches, out) {
    out.length = 0;
    for (const value of wrenches) {
        const body = value.side ? joint.outerBody : joint.innerBody, node = value.node;
        const segment = Math.min(node, body.segmentCount - 1, body.activeEnd - 1);
        const spin = inverseRotateVectorByQuaternion(frame(body, segment), {
            x: value.mx - (body.y[node] * value.fz - body.z[node] * value.fy),
            y: value.my - (body.z[node] * value.fx - body.x[node] * value.fz),
            z: value.mz - (body.x[node] * value.fy - body.y[node] * value.fx)
        });
        for (const [axis, amount] of [value.fx, value.fy, value.fz].entries())
            addGradient(out, value.side, node * 6 + axis, amount);
        for (const [axis, amount] of [spin.x, spin.y, spin.z].entries())
            addGradient(out, value.side, segment * 6 + 3 + axis, amount);
    }
    return out;
}

function hasForce(joint, tool, index) {
    return tool.lambdas[index] !== 0 ||
        joint._coupledExternalFriction?.owners.get(tool)?.get(index)?.tangentLambda.some(v => v !== 0);
}

export function hasKirchhoffToolReaction(joint, tool, index) {
    return hasForce(joint, tool, index) || tool._jointReactions?.get(index)?.wrenches.some(w => wrenchKeys.some(k => w[k] !== 0));
}

/** Carrier 1 -> 0 eliminates the retained generalized reaction in the SAME
 * full solve. This is not a collision constraint or a force-only clip. Normal
 * and tangent components share its apply scale. No force can be retired
 * without the Jacobians that actually applied it. */
export function appendKirchhoffToolRelease(joint, tool, index, rows, reason) {
    if (!hasKirchhoffToolReaction(joint, tool, index)) return;
    const reaction = tool._jointReactions?.get(index);
    if (!reaction) throw new Error('Cannot retire a loaded tool contact without its applied reaction');
    reaction.retiring = true;
    const row = reaction.row ??= { kind: 'tool-release', owner: tool, node: index,
        lower: 0, upper: 0, alpha: 0, strain: 0, lambda: 1, gradients: [] };
    row.reason = reason; row.lambda = 1;
    currentGradients(joint, reaction.wrenches, row.gradients);
    currentGradients(joint, reaction.normalWrenches, row.normalGradients ??= []);
    rows.push(row);
}

export function commitKirchhoffToolRelease(joint, row, delta) {
    const fraction = row.lambda + delta;
    if (!Number.isFinite(fraction) || fraction < 0 || fraction > 1)
        throw new RangeError('Invalid common-scale tool reaction release');
    const tool = row.owner, reaction = tool._jointReactions.get(row.node);
    for (const values of [reaction.wrenches, reaction.normalWrenches])
        for (const value of values) for (const key of wrenchKeys) value[key] *= fraction;
    tool.lambdas[row.node] *= fraction;
    const contact = joint._coupledExternalFriction?.owners.get(tool)?.get(row.node);
    if (contact) for (let axis = 0; axis < 2; axis++) contact.tangentLambda[axis] *= fraction;
    // Match normal toolProjection bookkeeping during loading; tangential
    // loading did not write this velocity-reconstruction channel.
    for (const g of row.normalGradients) {
        const body = g.side ? joint.outerBody : joint.innerBody, node = Math.floor(g.dof / 6), axis = g.dof % 6;
        if (axis > 2) continue;
        const projection = axis === 0 ? body.toolProjectionX : axis === 1 ? body.toolProjectionY : body.toolProjectionZ;
        projection[node] += body.inverseMass[node] * g.value * delta;
    }
    if (fraction === 0) reaction.retiring = false;
}

export function measureKirchhoffToolReleaseRows(joint, rows, out = {}) {
    out.pending = 0; out.positionMm = out.angleRad = 0;
    for (const row of rows) if (row.kind === 'tool-release') {
        out.pending++;
        for (const g of row.gradients) {
            const body = g.side ? joint.outerBody : joint.innerBody, node = Math.floor(g.dof / 6), axis = g.dof % 6;
            const prescribed = axis >= 3 && body.orientationControlCompliance === 0 && body.orientationControlSegment === node;
            const weight = prescribed ? 0 : axis < 3 ? body.inverseMass[node] : body['inverseInertia' + (axis - 2)][node];
            const value = Math.abs(weight * g.value);
            if (axis < 3) out.positionMm = Math.max(out.positionMm, value);
            else out.angleRad = Math.max(out.angleRad, value);
        }
    }
    return out;
}
