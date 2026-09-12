import { createCompositeMaterialInertiaEdge } from './kirchhoffCompositeMaterialInertia.js';
import { createCompositeInertiaWorkspace } from './kirchhoffCompositeKinematics.js';

function finite(value, name) {
    if (!Number.isFinite(value)) throw new RangeError(`${name} must be finite`);
    return value;
}
function positive(value, name) {
    if (!(finite(value, name) > 0)) throw new RangeError(`${name} must be positive`);
    return value;
}
function vector(value, size, name) {
    if (value?.length !== size) throw new RangeError(`${name} needs ${size} components`);
    return Array.from(value, v => finite(v, name));
}
function interpolate(a, b, t) {
    if (t === 0) return a;
    if (t === 1) return b;
    return (1 - t) * a + t * b;
}

/** Exact piecewise integration on the SAME two physical endpoints / six DOFs.
 * tool.oldVelocityPieces supplies ordered fractions:[a,b] and two own old
 * material velocities on each piece. Coverage must be exactly [0,1], without
 * gaps/overlap. Density and dsDx remain constant; dsDt is scalar or affine.
 *
 * Each subinterval compiles the existing exact material inertia operator once.
 * Virtual endpoint positions are a fixed affine map T of the original edge.
 * E and momentum add; g=T^T g_local and H=T^T H_local T. No new geometric node,
 * material path, angular inertia, nonlinear unknown, or accepted history is
 * introduced. Output buffers are borrowed until the next evaluate call.
 */
export function createCompositePiecewiseMaterialInertiaEdge({ coordinates, previousPositions, dt, tool }) {
    const x = vector(coordinates, 2, 'Coordinates'), length = positive(x[1] - x[0], 'Coordinate length');
    positive(dt, 'dt');
    if (previousPositions?.length !== 2) throw new RangeError('Two previous physical endpoints are required');
    const previous = previousPositions.map(p => vector(p, 3, 'Previous position'));
    if (typeof tool?.id !== 'string' || !tool.id) throw new RangeError('An explicit physical tool id is required');
    const id = tool.id, density = positive(tool.massPerMaterialLength, 'Physical density');
    const sStart = finite(tool.materialMap?.sStart, 'Material start label'), dsDx = positive(tool.materialMap?.dsDx, 'Material dsDx');
    const dsDt = typeof tool.materialMap.dsDt === 'number'
        ? [finite(tool.materialMap.dsDt, 'Material dsDt'), tool.materialMap.dsDt]
        : vector(tool.materialMap.dsDt, 2, 'Material dsDt');
    if (tool.oldMaterialVelocities !== undefined) throw new RangeError('Piecewise history must not also specify an ambiguous whole-edge oldMaterialVelocities pair');
    if (!Array.isArray(tool.oldVelocityPieces) || !tool.oldVelocityPieces.length) throw new RangeError('Explicit old material velocity pieces are required');
    const output = createCompositeInertiaWorkspace(1), local = output.tools[0], constantH = new Float64Array(36), pieces = [];
    let covered = 0, mass = 0, oldKineticEnergy = 0;
    const oldMomentum = new Float64Array(3);
    for (let index = 0; index < tool.oldVelocityPieces.length; index++) {
        const record = tool.oldVelocityPieces[index], [a, b] = vector(record.fractions, 2, 'Piece fractions');
        if (a !== covered || !(b > a) || b > 1) throw new RangeError('Ordered pieces must exactly cover [0,1] without gap or overlap');
        if (record.oldMaterialVelocities?.length !== 2) throw new RangeError('Each piece needs two own old material velocity endpoints');
        const velocities = record.oldMaterialVelocities.map(v => vector(v, 3, 'Old material velocity'));
        const T = [[1 - a, a], [1 - b, b]], points = [a, b].map(t => previous[0].map((v, k) => interpolate(v, previous[1][k], t)));
        const subcoordinates = [interpolate(x[0], x[1], a), interpolate(x[0], x[1], b)];
        if (!(subcoordinates[1] > subcoordinates[0])) throw new RangeError('Piece coordinate interval collapsed through floating-point loss');
        const compiled = createCompositeMaterialInertiaEdge({ coordinates: subcoordinates, previousPositions: points, dt,
            tool: { id, massPerMaterialLength: density,
                materialMap: { sStart: sStart + dsDx * length * a, dsDx, dsDt: [interpolate(dsDt[0], dsDt[1], a), interpolate(dsDt[0], dsDt[1], b)] },
                oldMaterialVelocities: velocities } });
        const initial = compiled.evaluate(points);
        mass += initial.mass; oldKineticEnergy += initial.oldKineticEnergy;
        for (let axis = 0; axis < 3; axis++) oldMomentum[axis] += initial.tools[0].oldMomentum[axis];
        // Full 6x6 pullback, including convective off-diagonal endpoint terms.
        for (let i = 0; i < 6; i++) for (let j = 0; j < 6; j++)
            for (let u = 0; u < 2; u++) for (let v = 0; v < 2; v++)
                constantH[6 * i + j] += T[u][Math.floor(i / 3)] * initial.hessian[6 * (3 * u + i % 3) + 3 * v + j % 3] * T[v][Math.floor(j / 3)];
        pieces.push({ a, b, T, compiled, points }); covered = b;
    }
    if (covered !== 1) throw new RangeError('Ordered pieces must exactly cover [0,1] without gap or overlap');
    if (![mass, oldKineticEnergy].every(Number.isFinite) || !oldMomentum.every(Number.isFinite) || !constantH.every(Number.isFinite))
        throw new RangeError('Nonfinite prepared piecewise inertia');
    local.samples = pieces.flatMap((piece, pieceIndex) => [0, 1].map(() => ({ pieceIndex, localFraction: NaN,
        fraction: NaN, s: NaN, dsDt: NaN, u: NaN, massWeight: NaN, coefficients: new Float64Array(2),
        velocity: new Float64Array(3), oldMaterialVelocity: new Float64Array(3), velocityIncrement: new Float64Array(3) })));
    output.quadrature = { rule: 'piecewise-gauss-2', exactForDeclaredFields: true, pieceCount: pieces.length };
    output.scope = 'prepared-one-material-piecewise-inertia'; output.certified = false; output.hessianValid = false;
    function evaluate(positions, { order = 'full' } = {}) {
        output.hessianValid = false; output.evaluationOrder = order;
        if (order !== 'full' && order !== 'gradient') throw new RangeError('Piecewise inertia order must be full or gradient');
        if (positions?.length !== 2) throw new RangeError('Two current physical endpoints are required');
        for (const position of positions) {
            if (position?.length !== 3) throw new RangeError('Each current endpoint needs three coordinates');
            for (const value of position) finite(value, 'Current position');
        }
        output.energy = output.kineticEnergy = local.energy = local.kineticEnergy = 0;
        output.mass = local.mass = mass; output.oldKineticEnergy = local.oldKineticEnergy = oldKineticEnergy;
        output.gradient.fill(0); output.kineticGradient.fill(0); output.momentumIncrement.fill(0);
        local.id = id; local.momentum.fill(0); local.oldMomentum.set(oldMomentum);
        for (let index = 0; index < pieces.length; index++) {
            const piece = pieces[index], { a, b, T, points } = piece;
            for (let axis = 0; axis < 3; axis++) {
                points[0][axis] = interpolate(positions[0][axis], positions[1][axis], a);
                points[1][axis] = interpolate(positions[0][axis], positions[1][axis], b);
            }
            // H was pulled back once during preparation. Local gradient mode
            // leaves its stale H unused; no per-trial suboperator construction.
            const response = piece.compiled.evaluate(points, { order: 'gradient' }), toolResponse = response.tools[0];
            output.energy += response.energy; output.kineticEnergy += response.kineticEnergy;
            for (let axis = 0; axis < 3; axis++) {
                local.momentum[axis] += toolResponse.momentum[axis]; output.momentumIncrement[axis] += response.momentumIncrement[axis];
                for (let end = 0; end < 2; end++) {
                    output.gradient[3 * end + axis] += T[0][end] * response.gradient[axis] + T[1][end] * response.gradient[3 + axis];
                    output.kineticGradient[3 * end + axis] += T[0][end] * response.kineticGradient[axis] + T[1][end] * response.kineticGradient[3 + axis];
                }
            }
            for (let j = 0; j < 2; j++) {
                const sample = toolResponse.samples[j], target = local.samples[2 * index + j];
                target.localFraction = sample.fraction; target.fraction = interpolate(a, b, sample.fraction);
                for (const field of ['s', 'dsDt', 'u', 'massWeight']) target[field] = sample[field];
                for (let end = 0; end < 2; end++) target.coefficients[end] = T[0][end] * sample.coefficients[0] + T[1][end] * sample.coefficients[1];
                target.velocity.set(sample.velocity); target.oldMaterialVelocity.set(sample.oldMaterialVelocity); target.velocityIncrement.set(sample.velocityIncrement);
            }
        }
        local.energy = output.energy; local.kineticEnergy = output.kineticEnergy;
        if (!Number.isFinite(output.energy) || !Number.isFinite(output.kineticEnergy) || !output.gradient.every(Number.isFinite) ||
            !output.kineticGradient.every(Number.isFinite) || !output.momentumIncrement.every(Number.isFinite) || !local.momentum.every(Number.isFinite) ||
            local.samples.some(s => ![s.fraction, s.s, s.dsDt, s.u, s.massWeight, ...s.coefficients].every(Number.isFinite)))
            throw new RangeError('Nonfinite piecewise material inertia response');
        if (order === 'full') { output.hessian.set(constantH); output.hessianValid = true; }
        return output;
    }
    return Object.freeze({ id, dt, mass, coordinateLength: length, pieceCount: pieces.length, dofCount: 6, evaluate,
        scope: 'prepared-one-material-piecewise-inertia', includesAngularInertia: false });
}
