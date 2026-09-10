import { createCompositeWallGeometryWorkspace, differentiateCompositeWallContact } from './kirchhoffCompositeWallGeometry.js';
import { createCompositeWallBvhGeometryWorkspace, differentiateCompositeWallBvhContact } from './kirchhoffCompositeWallBvhGeometry.js';

/** Reusable derivative storage for already-selected wall rows. B distributes
 * the physical inward UNIT normal. G=dg/dq generally differs from B.
 * forceColumn=-B is SIGNED; normalDerivative=DB contains no normal force.
 */
export function initializeCompositeWallDifferentialRow(row, layout) {
    const first = layout.positions[row.edge], second = layout.positions[row.edge + 1];
    row.dofs = Int32Array.from([first, first + 1, first + 2, second, second + 1, second + 2]);
    row.gapJacobian = new Float64Array(6).fill(NaN);
    row.forceColumn = new Float64Array(6).fill(NaN);
    row.normalDerivative = new Float64Array(36).fill(NaN);
    row.derivativeUnavailable = true; row.derivativeReason = 'not-evaluated';
    row.derivativeSource = null; row.inactiveForSolve = false; row.deltaNormalForce = null;
    row.rawContact = { signedDistance: NaN, signedGap: NaN, source: null, segmentT: NaN, capsuleSampleCount: 0,
        queryPoint:new Float64Array(3).fill(NaN), inward: { values: new Float64Array(3) }, closestPoint: { values: new Float64Array(3) }, faceIndex: -1 };
    return row;
}
export function createCompositeWallDifferentialWorkspace() {
    return { point: createCompositeWallGeometryWorkspace(1), capsule: createCompositeWallGeometryWorkspace(2),
        bvhPoint: createCompositeWallBvhGeometryWorkspace(1), bvhCapsule: createCompositeWallBvhGeometryWorkspace(2) };
}

/** Copy raw query values BEFORE its scratch buffer is reused. Endpoint rows
 * retain the raw degenerate-capsule segmentT in rawContact; row.t is the
 * explicit proximal/distal scatter fraction, a different piece of data.
 */
export function captureCompositeWallDifferentialContact(row, contact, radius) {
    if (!row.rawContact) throw new TypeError('Initialize wall-row buffers and global DOFs before capturing contact');
    const raw = row.rawContact;
    raw.signedDistance = contact.signedDistance; raw.signedGap = contact.signedGap;
    raw.source = contact.source; raw.segmentT = contact.segmentT; raw.capsuleSampleCount = contact.capsuleSampleCount;
    raw.queryPoint.set(contact.point?.values??[NaN,NaN,NaN]);
    raw.inward.values.set(contact.inward.values);
    raw.closestPoint.values.set(contact.closestPoint.values); raw.faceIndex = contact.faceIndex;
    row.signedDistance = raw.signedDistance; row.signedGap = raw.signedGap; row.querySegmentT = raw.segmentT;
    row.radius = radius; row.gapJacobian.fill(NaN); row.forceColumn.fill(NaN); row.normalDerivative.fill(NaN);
    row.derivativeUnavailable = true; row.derivativeReason = 'not-evaluated';
    row.derivativeSource = null; row.inactiveForSolve = false; row.deltaNormalForce = null;
    return row;
}

/** No query, perturbation, provider setting change or normal fallback.
 * Unsupported derivatives remain NaN and must be checked against physical Fn
 * before using/omitting the row. A point envelope row uses the POINT contract,
 * then scatters 3 -> 6 DOFs; it never borrows a degenerate capsule derivative.
 */
export function differentiateCompositeWallRow({ field, positions, row }, scratch) {
    if (!row.rawContact) throw new TypeError('Capture the original contact before differentiating its wall row');
    row.gapJacobian.fill(NaN); row.forceColumn.fill(NaN); row.normalDerivative.fill(NaN);
    row.derivativeUnavailable = true; row.derivativeReason = 'not-evaluated'; row.derivativeSource = row.source;
    row.inactiveForSolve = false; row.deltaNormalForce = null;
    if (!row.included) { row.derivativeReason = 'not-included'; return row; }
    if (row.source === 'analytic-plane') {
        row.normalDerivative.fill(0);
        for (let end = 0; end < 2; end++) for (let axis = 0; axis < 3; axis++) {
            const B = (end ? row.t : 1 - row.t) * row.normal[axis];
            row.gapJacobian[3 * end + axis] = B; row.forceColumn[3 * end + axis] = -B;
        }
    } else if (row.source === 'sparse-sdf' || row.source === 'sparse-sdf-bvh') {
        const point = row.role === 'proximal' || row.role === 'distal', end = row.role === 'distal' ? 1 : 0;
        const bvh = row.source === 'sparse-sdf-bvh';
        const differentiate = bvh ? differentiateCompositeWallBvhContact : differentiateCompositeWallContact;
        const geometryScratch = bvh ? (point ? scratch.bvhPoint : scratch.bvhCapsule) : (point ? scratch.point : scratch.capsule);
        let geometry;
        try {
            geometry = differentiate({ field, contact: row.rawContact,
                positions: point ? [positions[row.edge + end]] : [positions[row.edge], positions[row.edge + 1]], radius: row.radius },
                geometryScratch);
        } catch (error) {
            row.derivativeReason = `invalid-geometry-contract:${error.message}`; return row;
        }
        if (!geometry.supported) { row.derivativeReason = geometry.reason; row.derivativeDetails={point:Array.from(geometry.point??[]),queryPoint:Array.from(row.rawContact.queryPoint??[]),cellValue:geometry.cellData?.value,signedDistance:row.rawContact.signedDistance,signedGap:row.rawContact.signedGap,radius:row.radius}; return row; }
        if (point) {
            row.gapJacobian.fill(0); row.forceColumn.fill(0); row.normalDerivative.fill(0);
            for (let i = 0; i < 3; i++) {
                row.gapJacobian[3 * end + i] = geometry.gapGradient[i];
                row.forceColumn[3 * end + i] = -geometry.normalForceColumn[i];
                for (let j = 0; j < 3; j++) row.normalDerivative[6 * (3 * end + i) + 3 * end + j] = geometry.normalForceJacobian[3 * i + j];
            }
        } else {
            row.gapJacobian.set(geometry.gapGradient);
            for (let i = 0; i < 6; i++) row.forceColumn[i] = -geometry.normalForceColumn[i];
            row.normalDerivative.set(geometry.normalForceJacobian);
        }
    } else { row.derivativeReason = `unsupported-source:${row.source}`; return row; }
    if (!row.gapJacobian.every(Number.isFinite) || !row.forceColumn.every(Number.isFinite) || !row.normalDerivative.every(Number.isFinite)) {
        row.gapJacobian.fill(NaN); row.forceColumn.fill(NaN); row.normalDerivative.fill(NaN);
        row.derivativeReason = 'nonfinite-row-derivative'; return row;
    }
    row.derivativeUnavailable = false; row.derivativeReason = null;
    return row;
}

/** This guard is mandatory before mixed assembly or canonicalization.
 * An open unsupported row with physical Fn EXACTLY zero has NCP deltaFn=0
 * and may be analytically eliminated, while its original gap remains measured.
 * Unsupported active/loaded rows reject; no G=n substitution is made.
 */
export function requireCompositeWallDifferentialRow(row, normalForce) {
    if (!Number.isFinite(normalForce) || normalForce < 0) throw new RangeError('Finite nonnegative physical wall force is required');
    row.inactiveForSolve = false; row.deltaNormalForce = null;
    if (!row.included) {
        if (normalForce !== 0) throw new RangeError('Removed wall ownership requires an explicit traction transfer');
        return false;
    }
    if (!Number.isFinite(row.gap)) throw new RangeError('Original wall gap must be finite');
    if (!row.derivativeUnavailable && row.derivativeSource === row.source && row.gapJacobian?.length === 6 &&
        row.forceColumn?.length === 6 && row.normalDerivative?.length === 36 &&
        row.gapJacobian.every(Number.isFinite) && row.forceColumn.every(Number.isFinite) && row.normalDerivative.every(Number.isFinite)) return true;
    row.derivativeUnavailable = true;
    row.gapJacobian?.fill(NaN); row.forceColumn?.fill(NaN); row.normalDerivative?.fill(NaN);
    if (normalForce === 0 && row.gap > 0) {
        row.inactiveForSolve = true; row.deltaNormalForce = 0; return false;
    }
    throw new RangeError(`Unsupported active/loaded wall derivative (${row.source}; ${row.derivativeReason ?? 'stale derivative source'})`);
}
export function requireCompositeWallDifferentialRows(workspace, normalForces) {
    if (normalForces?.length !== workspace.rows.length) throw new RangeError('Physical forces must match wall rows');
    for (const row of workspace.rows) requireCompositeWallDifferentialRow(row, normalForces[row.index]);
    return workspace;
}

function dofs(layout, row) {
    if (row.dofs?.length !== 6 || row.dofs[0] !== layout.positions[row.edge] || row.dofs[3] !== layout.positions[row.edge + 1])
        throw new RangeError('Persistent wall DOFs must match the frozen layout');
    return row.dofs;
}
/** Exact comparison AFTER global position scatter. Zero entries on a row's
 * unused endpoint do not create fictitious extra support. Unknown derivatives
 * never establish a dependence, even when all associated forces are zero.
 */
export function equalCompositeWallGlobalDifferentials(a, b, layout) {
    if (a.derivativeUnavailable || b.derivativeUnavailable || a.source !== b.source || a.gap !== b.gap) return false;
    const ad = dofs(layout, a), bd = dofs(layout, b), union = [...new Set([...ad, ...bd])];
    for (const i of union) {
        const ai = ad.indexOf(i), bi = bd.indexOf(i);
        for (const key of ['gapJacobian', 'forceColumn']) if ((ai < 0 ? 0 : a[key][ai]) !== (bi < 0 ? 0 : b[key][bi])) return false;
        for (const j of union) {
            const aj = ad.indexOf(j), bj = bd.indexOf(j);
            if ((ai < 0 || aj < 0 ? 0 : a.normalDerivative[6 * ai + aj]) !==
                (bi < 0 || bj < 0 ? 0 : b.normalDerivative[6 * bi + bj])) return false;
        }
    }
    return true;
}

/** The original g/G/B/DB of an interior capsule must equal the weighted
 * endpoint combination in ALL entries. DB includes cross-endpoint blocks;
 * matching normal and gap alone cannot remove a nonlinear capsule row.
 */
export function isCompositeWallEndpointCombination(a, b, c) {
    if ([a, b, c].some(row => !row.included || row.derivativeUnavailable) || a.edge !== c.edge || b.edge !== c.edge ||
        a.source !== c.source || b.source !== c.source || a.owner !== c.owner || b.owner !== c.owner ||
        a.radius !== c.radius || b.radius !== c.radius || a.t !== 0 || b.t !== 1 || !(c.t > 0 && c.t < 1)) return false;
    const w0 = 1 - c.t, w1 = c.t;
    if (c.gap !== w0 * a.gap + w1 * b.gap || c.normal.some((v, i) => v !== a.normal[i] || v !== b.normal[i])) return false;
    for (const key of ['gapJacobian', 'forceColumn', 'normalDerivative'])
        for (let i = 0; i < c[key].length; i++) if (c[key][i] !== w0 * a[key][i] + w1 * b[key][i]) return false;
    return true;
}
