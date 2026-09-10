/** Material kinematics on ONE common centerline. x is a frozen mesh coordinate,
 * s_i(x,t) labels each tool independently, and q_x is NOT a unit tangent.
 * Unknown time/spatial derivatives are never replaced by zero.
 */
const finite = (value, name) => {
    if (!Number.isFinite(value)) throw new TypeError(`${name} must be finite`);
    return value;
};
function positive(value, name) {
    if (!(finite(value, name) > 0)) throw new RangeError(`${name} must be positive`);
    return value;
}
function vector(value, size, name) {
    if (!value || value.length !== size) throw new TypeError(`${name} needs ${size} components`);
    return Array.from(value, v => finite(v, name));
}
function endpoints(value, name) {
    return typeof value === 'number' ? [finite(value, name), value] : vector(value, 2, name);
}
function checkTools(tools) {
    if (!Array.isArray(tools) || tools.length < 1 || tools.length > 2)
        throw new RangeError('One or two tool inputs are required');
    const ids = new Set();
    for (const tool of tools) {
        if (typeof tool.id !== 'string' || !tool.id || ids.has(tool.id)) throw new TypeError('Tool ids must be distinct nonempty strings');
        ids.add(tool.id);
    }
}

/**
 * u_i=-s_t/s_x; v_i=q_t+u_i*q_x.
 * Tool derivatives are pointwise in the SAME x,t coordinate system.
 * Optional twist requires all of thetaDt,thetaDx,frameSpin. A scalar frameSpin
 * is the frame's axial spin ALONG THAT MATERIAL PATH. Alternatively supply
 * frameSpin:{dt,dx}; its material-path value is dt+u_i*dx. It is not inferred
 * from q_x, and this function does not assert that bending angular velocity is
 * zero. With no twist inputs, spin=null explicitly denotes unavailable data.
 */
export function evaluateCompositeKinematics({ positionDt, positionDx, tools }) {
    checkTools(tools);
    const qt = vector(positionDt, 3, 'positionDt'), qx = vector(positionDx, 3, 'positionDx');
    return { tools: tools.map(tool => {
        const dsDx = positive(tool.dsDx, 'dsDx'), dsDt = finite(tool.dsDt, 'dsDt'), u = -dsDt / dsDx;
        const hasTwist = ['thetaDt', 'thetaDx', 'frameSpin'].some(key => tool[key] !== undefined);
        let spin = null, frameSpin = null;
        if (hasTwist) {
            const thetaDt = finite(tool.thetaDt, 'thetaDt'), thetaDx = finite(tool.thetaDx, 'thetaDx');
            frameSpin = typeof tool.frameSpin === 'object' && tool.frameSpin !== null
                ? finite(tool.frameSpin.dt, 'frameSpin.dt') + u * finite(tool.frameSpin.dx, 'frameSpin.dx')
                : finite(tool.frameSpin, 'frameSpin');
            spin = finite(thetaDt + u * thetaDx + frameSpin, 'material spin');
        }
        return { id: tool.id, dsDx, dsDt, u: finite(u, 'material coordinate velocity'),
            velocity: qt.map((v, axis) => finite(v + u * qx[axis], 'material velocity')),
            spin, frameSpin, spinKnown: hasTwist };
    }) };
}

const GAUSS = [.5 - 1 / (2 * Math.sqrt(3)), .5 + 1 / (2 * Math.sqrt(3))];
export function createCompositeInertiaWorkspace(toolCount = 2) {
    if (!Number.isInteger(toolCount) || toolCount < 1 || toolCount > 2) throw new RangeError('One or two tools are required');
    return { toolCount, dofCount: 6, energy: 0, kineticEnergy: 0, oldKineticEnergy: 0, mass: 0,
        gradient: new Float64Array(6), kineticGradient: new Float64Array(6), hessian: new Float64Array(36),
        momentumIncrement: new Float64Array(3),
        tools: Array.from({ length: toolCount }, () => ({ id: null, energy: 0, kineticEnergy: 0,
            oldKineticEnergy: 0, mass: 0, momentum: new Float64Array(3), oldMomentum: new Float64Array(3),
            samples: GAUSS.map(fraction => ({ fraction, s: 0, dsDt: 0, u: 0, massWeight: 0,
                coefficients: new Float64Array(2), velocity: new Float64Array(3),
                oldMaterialVelocity: new Float64Array(3), velocityIncrement: new Float64Array(3) })) })),
        quadrature: { rule: 'gauss-2', exactForDeclaredFields: true },
        energyKind: 'material-velocity-increment', includesAngularInertia: false };
}

/**
 * Exact local translational backward-Euler inertia for a frozen affine edge:
 *   q_t = (q_current(x)-q_old(x))/dt, at the SAME mesh coordinate x;
 *   v_i = q_t + u_i*q_current,x, u_i = -s_i,t / s_i,x;
 *   energy = 1/2 integral rho_i |v_i-v_old_material(s_i)|^2 ds_i.
 *
 * kineticEnergy separately reports the actual 1/2 integral rho_i |v_i|^2 ds_i.
 * gradient differentiates energy; kineticGradient differentiates kineticEnergy.
 * hessian is the SAME exact Hessian for both, not a diagonal/lumped or GN
 * approximation. DOFs: [q0.x,q0.y,q0.z,q1.x,q1.y,q1.z]. Outputs are borrowed
 * until this workspace is reused. All input arrays/maps remain untouched.
 *
 * coordinates:[x0,x1] and positions/previousPositions contain the two edge
 * endpoints. previousPositions must be resampled at current x after remeshing.
 * Each tool supplies constant massPerMaterialLength and materialMap:
 *   {sStart,dsDx>0,dsDt:scalar OR [dsDtAtStart,dsDtAtEnd]}.
 * s(x)=sStart+dsDx*(x-x0); s_t is constant or explicitly linear over the edge.
 * oldMaterialVelocities MUST be [v_old(sStart),v_old(sEnd)] for the material
 * CURRENTLY covering this edge, linearly interpolated in s. These are physical
 * material velocities, not old q_t or samples at the old spatial coordinates.
 * No missing mass, map rate or old velocity defaults to zero.
 *
 * Gauss-2 is exact for these declared fields: q, q_old and v_old are linear,
 * u is at most linear, rho and s_x are constant. Split an edge at other density
 * or map changes, or supply a separately validated higher-order integration.
 * The integration weight contains dsDx. The current q_x remains in v and J:
 *   J0=N0/dt-u/L, J1=N1/dt+u/L.
 * Angular/frame inertia and time integration of the material maps are external.
 */
export function assembleCompositeTranslationalInertia({ coordinates, positions, previousPositions, dt, tools },
    workspace = createCompositeInertiaWorkspace(tools.length)) {
    checkTools(tools);
    if (workspace.toolCount !== tools.length) throw new RangeError('Workspace tool count must match');
    const x = vector(coordinates, 2, 'coordinates'), length = positive(x[1] - x[0], 'edge coordinate length');
    positive(dt, 'dt');
    if (!positions || positions.length !== 2 || !previousPositions || previousPositions.length !== 2)
        throw new TypeError('Two current and previous endpoint positions are required');
    const p = positions.map(v => vector(v, 3, 'position')), old = previousPositions.map(v => vector(v, 3, 'previous position'));
    const inputs = tools.map(tool => {
        const map = tool.materialMap;
        if (!map) throw new TypeError(`${tool.id} materialMap is required`);
        const dsDx = positive(map.dsDx, 'dsDx'), sStart = finite(map.sStart, 'sStart');
        const dsDt = endpoints(map.dsDt, 'dsDt');
        positive(tool.massPerMaterialLength, 'massPerMaterialLength');
        if (!tool.oldMaterialVelocities || tool.oldMaterialVelocities.length !== 2)
            throw new TypeError(`${tool.id} needs two OLD MATERIAL velocity samples`);
        return { id: tool.id, dsDx, sStart, dsDt, mass: tool.massPerMaterialLength,
            oldVelocity: tool.oldMaterialVelocities.map(v => vector(v, 3, 'old material velocity')) };
    });
    const { gradient, kineticGradient, hessian, momentumIncrement } = workspace;
    gradient.fill(0); kineticGradient.fill(0); hessian.fill(0); momentumIncrement.fill(0);
    workspace.energy = workspace.kineticEnergy = workspace.oldKineticEnergy = workspace.mass = 0;
    for (let index = 0; index < inputs.length; index++) {
        const input = inputs[index], output = workspace.tools[index];
        output.id = input.id; output.energy = output.kineticEnergy = output.oldKineticEnergy = output.mass = 0;
        output.momentum.fill(0); output.oldMomentum.fill(0);
        for (const sample of output.samples) {
            const n1 = sample.fraction, n0 = 1 - n1;
            sample.s = input.sStart + input.dsDx * length * n1;
            sample.dsDt = n0 * input.dsDt[0] + n1 * input.dsDt[1];
            sample.u = -sample.dsDt / input.dsDx;
            sample.massWeight = input.mass * input.dsDx * length / 2;
            const weight = sample.massWeight, a = sample.coefficients;
            a[0] = n0 / dt - sample.u / length; a[1] = n1 / dt + sample.u / length;
            output.mass += weight;
            for (let axis = 0; axis < 3; axis++) {
                // Difference form avoids subtracting two large absolute world
                // positions after interpolation and preserves rigid translation.
                const qt = (n0 * (p[0][axis] - old[0][axis]) + n1 * (p[1][axis] - old[1][axis])) / dt;
                const velocity = qt + sample.u * (p[1][axis] - p[0][axis]) / length;
                const oldVelocity = n0 * input.oldVelocity[0][axis] + n1 * input.oldVelocity[1][axis];
                const increment = velocity - oldVelocity;
                sample.velocity[axis] = velocity; sample.oldMaterialVelocity[axis] = oldVelocity;
                sample.velocityIncrement[axis] = increment;
                output.energy += .5 * weight * increment * increment;
                output.kineticEnergy += .5 * weight * velocity * velocity;
                output.oldKineticEnergy += .5 * weight * oldVelocity * oldVelocity;
                output.momentum[axis] += weight * velocity; output.oldMomentum[axis] += weight * oldVelocity;
                momentumIncrement[axis] += weight * increment;
                for (let node = 0; node < 2; node++) {
                    const row = 3 * node + axis;
                    gradient[row] += weight * a[node] * increment;
                    kineticGradient[row] += weight * a[node] * velocity;
                    for (let other = 0; other < 2; other++) hessian[6 * row + 3 * other + axis] += weight * a[node] * a[other];
                }
            }
        }
        workspace.energy += output.energy; workspace.kineticEnergy += output.kineticEnergy;
        workspace.oldKineticEnergy += output.oldKineticEnergy; workspace.mass += output.mass;
    }
    if (![workspace.energy, workspace.kineticEnergy, workspace.oldKineticEnergy, workspace.mass].every(Number.isFinite) ||
        !gradient.every(Number.isFinite) || !kineticGradient.every(Number.isFinite) || !hessian.every(Number.isFinite) ||
        !momentumIncrement.every(Number.isFinite) || workspace.tools.some(tool => tool.samples.some(sample =>
            ![sample.s, sample.dsDt, sample.u, sample.massWeight, ...sample.coefficients].every(Number.isFinite))))
        throw new RangeError('Nonfinite composite inertia response');
    return workspace;
}

/** Add one edge's already-summed material inertia to Chain AFTER its elastic
 * assembly and BEFORE its solve. Only six shared position DOFs receive entries;
 * every independent spin row remains untouched. Scatter before reusing the
 * local workspace. The complete consistent Hessian goes into the lower band;
 * do not replace it by solveCompositeChainIncrement's optional diagonal.
 */
export function scatterCompositeTranslationalInertia(local, edge, chain) {
    const { layout, gradient, hessian } = chain;
    if (!layout || !Number.isInteger(edge) || edge < 0 || edge + 1 >= layout.nodeCount)
        throw new RangeError('A valid Chain edge is required');
    if (local.gradient.length !== 6 || local.hessian.length !== 36 || gradient.length !== layout.dofCount ||
        hessian.length !== layout.dofCount * layout.band) throw new RangeError('Inertia/Chain buffers must match their layouts');
    const ids = layout.edgeToolIds[edge];
    if (local.tools.length !== ids.length || local.tools.some(tool => !ids.includes(tool.id)))
        throw new RangeError('Local inertia tools must match the material on the Chain edge');
    const dofs = [layout.positions[edge], layout.positions[edge + 1]].flatMap(start => [start, start + 1, start + 2]);
    if (!Number.isFinite(local.energy) || !local.gradient.every(Number.isFinite) || !local.hessian.every(Number.isFinite) ||
        !Number.isFinite(chain.energy + local.energy)) throw new RangeError('Finite inertia/Chain energy and operators are required');
    for (let i = 0; i < 6; i++) {
        if (!Number.isInteger(dofs[i]) || dofs[i] < 0 || dofs[i] >= layout.dofCount) throw new RangeError('Invalid Chain position DOF');
        if (!Number.isFinite(gradient[dofs[i]] + local.gradient[i])) throw new RangeError('Nonfinite scattered gradient');
        for (let j = 0; j <= i; j++) {
            const value = local.hessian[6 * i + j];
            if (value === 0) continue;
            if (Math.abs(dofs[i] - dofs[j]) >= layout.band) throw new RangeError('Chain band cannot contain the consistent edge inertia');
            const hi = Math.max(dofs[i], dofs[j]), lo = Math.min(dofs[i], dofs[j]);
            if (!Number.isFinite(hessian[hi * layout.band + hi - lo] + value)) throw new RangeError('Nonfinite scattered Hessian');
        }
    }
    chain.energy += local.energy;
    for (let i = 0; i < 6; i++) {
        gradient[dofs[i]] += local.gradient[i];
        for (let j = 0; j <= i; j++) {
            const value = local.hessian[6 * i + j];
            if (value === 0) continue;
            const hi = Math.max(dofs[i], dofs[j]), lo = Math.min(dofs[i], dofs[j]);
            hessian[hi * layout.band + hi - lo] += value;
        }
    }
    return chain;
}
