import { createCompositeChainLayout, createCompositeChainWorkspace, assembleCompositeChain,
    solveCompositeChainIncrement } from './kirchhoffCompositeChain.js';
import { createCompositeInertiaWorkspace, assembleCompositeTranslationalInertia,
    scatterCompositeTranslationalInertia, evaluateCompositeKinematics } from './kirchhoffCompositeKinematics.js';
import { createCompositeLengthConstraintWorkspace, assembleCompositeLengthConstraints,
    measureCompositeLengthConstraints } from './kirchhoffCompositeLengthConstraints.js';
import { createCompositeWallWorkspace, refreshCompositeWallContacts,
    assembleCompositeWallAugmented, measureCompositeWallConstraints,
    canonicalizeCompositeWallReactions } from './kirchhoffCompositeWallContacts.js';
import { createCompositeMixedWorkspace, solveCompositeMixedDirection } from './kirchhoffCompositeMixedDirection.js';
import { createCompositeWallEnvelopeWorkspace, refreshCompositeWallEnvelope,
    canonicalizeCompositeWallEnvelope } from './kirchhoffCompositeWallEnvelope.js';
import { createCompositeInertiaCache } from './kirchhoffCompositeInertiaCache.js';
import { createCompositeWallSdfBranchesWorkspace, evaluateCompositeWallSdfBranches,
    measureCompositeWallSdfBranches, findCompositeWallSdfSeamCrossing } from './kirchhoffCompositeWallSdfBranches.js';

const finite = (v, name) => { if (!Number.isFinite(v)) throw new TypeError(`${name} must be finite`); return v; };
const positive = (v, name) => { if (!(finite(v, name) > 0)) throw new RangeError(`${name} must be positive`); return v; };
function vector(v, n, name) {
    if (!v || v.length !== n) throw new TypeError(`${name} needs ${n} entries`);
    return Array.from(v, x => finite(x, name));
}
const dot = (a, b) => a.reduce((sum, v, i) => sum + v * b[i], 0);
const cross = (a, b) => [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]];
const timeStepWorkspaces = new WeakMap();

/** Reusable numeric scratch for one fixed topology and constitutive backend.
 * No accepted geometry, multipliers, material history or factorization is
 * cached here. Each call still assembles and solves the original equations.
 * A bounded LRU retains only the layouts/buffers of recent wall row patterns;
 * changing a contact's value or active branch never reuses an old factor.
 * The opaque handle may be shared between sequential solves, never nested.
 */
export function createCompositeTimeStepWorkspace(layout, { elementBackend = 'wasm', mixedCacheCapacity = 2 } = {}) {
    if (!Number.isInteger(mixedCacheCapacity) || mixedCacheCapacity < 1 || mixedCacheCapacity > 4)
        throw new RangeError('mixedCacheCapacity must be 1..4');
    const ownLayout = createCompositeChainLayout(layout.edgeToolIds);
    const stats = { chainBuilds: 1, wallBuilds: 0, mixedBuilds: 0, mixedHits: 0, calls: 0 };
    const scratch = { layout: ownLayout, elementBackend, busy: false, stats, mixedCacheCapacity,
        chain: createCompositeChainWorkspace(ownLayout, { elementBackend }),
        lengthWorkspace: createCompositeLengthConstraintWorkspace(ownLayout),
        lengthOriginal: createCompositeLengthConstraintWorkspace(ownLayout),
        inertiaWorkspaces: [null, createCompositeInertiaWorkspace(1), createCompositeInertiaWorkspace(2)],
        walls: new Map(), mixed: new Map(),
        lengthDefinitions: Array.from({ length: ownLayout.nodeCount - 1 }, (_, edge) => ({ kind: 'length', edge,
            dofs: [ownLayout.positions[edge], ownLayout.positions[edge + 1]].flatMap(start => [start, start + 1, start + 2]) })) };
    const handle = Object.freeze({ elementBackend, nodeCount: ownLayout.nodeCount, dofCount: ownLayout.dofCount,
        get diagnostics() { return { ...stats, retainedMixedSystems: scratch.mixed.size, mixedCacheCapacity }; } });
    timeStepWorkspaces.set(handle, scratch);
    return handle;
}

function mixedWorkspace(scratch, wallMode, representatives, wallWorkspace) {
    const key = wallMode + ':' + (representatives ?? []).join(',');
    let value = scratch.mixed.get(key);
    if (value) { scratch.stats.mixedHits++; scratch.mixed.delete(key); }
    else {
        const definitions = [...scratch.lengthDefinitions, ...(representatives ?? []).map(index => {
            const wallIndex = index >= 0 ? index : Math.floor((-index - 1) / 2);
            return { ...scratch.lengthDefinitions[wallWorkspace.rows[wallIndex].edge], kind: 'wall', wallIndex,
                sdfBranch: index >= 0 ? null : (-index - 1) % 2 };
        })];
        value = { mixed: createCompositeMixedWorkspace(scratch.layout, definitions),
            rows: definitions.map(definition => ({ ...definition, jacobian: new Float64Array(6),
                forceColumn: definition.kind === 'wall' ? new Float64Array(6) : undefined,
                geometricHessian: new Float64Array(36) })) };
        scratch.stats.mixedBuilds++;
    }
    scratch.mixed.set(key, value);
    if (scratch.mixed.size > scratch.mixedCacheCapacity) scratch.mixed.delete(scratch.mixed.keys().next().value);
    return value;
}
function unit(v) { const length = positive(Math.hypot(...v), 'direction length'); return v.map(x => x / length); }
function transport(director, from, to) {
    const denominator = 1 + dot(from, to);
    if (!(denominator > 1e-10)) throw new RangeError('Accepted frame transport cannot reverse its tangent');
    const axis = cross(from, to), first = cross(axis, director), second = cross(axis, first);
    return director.map((v, i) => v + first[i] + second[i] / denominator);
}
function copyData(data) {
    return { positions: data.positions.map(p => vector(p, 3, 'position')),
        coordinates: Float64Array.from(data.coordinates),
        reference: data.reference.map(frame => ({ tangent: vector(frame.tangent, 3, 'reference tangent'),
            director: vector(frame.director, 3, 'reference director') })),
        tools: data.tools.map(tool => ({ ...tool, angles: Float64Array.from(tool.angles),
            referenceTwists: tool.referenceTwists ? Float64Array.from(tool.referenceTwists) : undefined })) };
}

/** Initial zero length multipliers are a solver seed, never a certificate.
 * Mutable positions, spins, frames and history arrays are owned copies.
 * Material providers/compiled materials are borrowed read-only inputs.
 */
export function createCompositeTimeStepState({ data, layout, lengthMultipliers = null, wallContactState = null, time = 0, step = 0 }) {
    const ownLayout = createCompositeChainLayout(layout.edgeToolIds), ownData = copyData(data);
    if (ownData.positions.length !== ownLayout.nodeCount || ownData.coordinates.length !== ownLayout.nodeCount ||
        ownData.reference.length !== ownLayout.nodeCount - 1) throw new RangeError('State geometry must match its layout');
    if (!Number.isInteger(step) || step < 0) throw new RangeError('step must be a nonnegative integer');
    finite(time, 'time');
    const lambda = lengthMultipliers === null ? new Float64Array(ownLayout.nodeCount - 1)
        : Float64Array.from(vector(lengthMultipliers, ownLayout.nodeCount - 1, 'lengthMultipliers'));
    return { data: ownData, layout: ownLayout, lengthMultipliers: lambda,
        wallContactState: wallContactState === null ? null : structuredClone(wallContactState),
        time, step, materialVelocities: null };
}

function variables(data, layout) {
    const slots = new Array(layout.dofCount), byId = new Map(data.tools.map(tool => [tool.id, tool]));
    for (let node = 0; node < layout.nodeCount; node++) for (let axis = 0; axis < 3; axis++)
        slots[layout.positions[node] + axis] = { values: data.positions[node], index: axis, kind: 'position' };
    for (const [id, offsets] of layout.spins) {
        const tool = byId.get(id);
        if (!tool || tool.angles.length !== layout.nodeCount - 1) throw new RangeError('Every active spin needs matching tool data');
        for (let edge = 0; edge < offsets.length; edge++) if (offsets[edge] >= 0)
            slots[offsets[edge]] = { values: tool.angles, index: edge, kind: 'spin', id };
    }
    for (const slot of slots) finite(slot.values[slot.index], 'state variable');
    return { slots, get: () => Float64Array.from(slots, slot => slot.values[slot.index]),
        set(values) { slots.forEach((slot, i) => { slot.values[slot.index] = values[i]; }); } };
}
function residualNorms(values, layout, fixed) {
    let force = 0, torque = 0;
    for (const start of layout.positions) force = Math.max(force, Math.hypot(
        fixed[start] ? 0 : values[start], fixed[start + 1] ? 0 : values[start + 1], fixed[start + 2] ? 0 : values[start + 2]));
    for (const offsets of layout.spins.values()) for (const dof of offsets) if (dof >= 0 && !fixed[dof]) torque = Math.max(torque, Math.abs(values[dof]));
    return { force, torque };
}
function frozenMaterials(data, layout) {
    for (const tool of data.tools) {
        const source = tool.materialAt, material = tool.material, scaleSource = tool.dsDx ?? 1;
        const samples = new Map(), scales = new Map();
        for (const { vertex: i, tools } of layout.hinges) if (tools.includes(tool.id)) {
            const args = { vertex: i, coordinate: data.coordinates[i], start: (data.coordinates[i - 1] + data.coordinates[i]) / 2,
                end: (data.coordinates[i] + data.coordinates[i + 1]) / 2 };
            const value = source ? source(args) : material;
            samples.set(i, { ...value, stiffness: Float64Array.from(value.stiffness), intrinsic: Float64Array.from(value.intrinsic) });
            scales.set(args.coordinate, positive(typeof scaleSource === 'function' ? scaleSource(args.coordinate) : scaleSource, 'material dsDx'));
        }
        tool.materialAt = ({ vertex }) => samples.get(vertex);
        tool.dsDx = typeof scaleSource === 'function' ? coordinate => scales.get(coordinate) : positive(scaleSource, 'material dsDx');
    }
}

function acceptedFrames(data, layout) {
    const reference = data.reference.map((frame, edge) => {
        const tangent = unit(data.positions[edge + 1].map((v, axis) => v - data.positions[edge][axis]));
        const carried = transport(frame.director, frame.tangent, tangent);
        const projection = dot(carried, tangent);
        return { tangent, director: unit(carried.map((v, axis) => v - projection * tangent[axis])) };
    });
    const windings = new Map();
    for (const tool of data.tools) {
        const values = new Float64Array(layout.hinges.length).fill(NaN);
        for (const { vertex: i, tools } of layout.hinges) if (tools.includes(tool.id)) {
            const left = reference[i - 1], right = reference[i];
            const d = transport(left.director, left.tangent, right.tangent);
            const raw = Math.atan2(dot(right.tangent, cross(d, right.director)), dot(d, right.director));
            const anchor = tool.referenceTwists?.[i - 1] ?? 0;
            values[i - 1] = raw + 2 * Math.PI * Math.round((anchor - raw) / (2 * Math.PI));
        }
        windings.set(tool.id, values);
    }
    return { reference, windings };
}
function acceptedVelocities(data, oldPositions, inertiaEdges, dt) {
    return inertiaEdges.map((edgeInput, edge) => {
        const length = data.coordinates[edge + 1] - data.coordinates[edge];
        const qx = data.positions[edge + 1].map((v, axis) => (v - data.positions[edge][axis]) / length);
        return { edge, tools: edgeInput.tools.map(tool => {
            const map = tool.materialMap, rates = typeof map.dsDt === 'number' ? [map.dsDt, map.dsDt] : map.dsDt;
            const velocities = [0, 1].map(end => kinematicsAt(end));
            function kinematicsAt(end) {
                const qt = data.positions[edge + end].map((v, axis) => (v - oldPositions[edge + end][axis]) / dt);
                return evaluateCompositeKinematics({ positionDt: qt, positionDx: qx,
                    tools: [{ id: tool.id, dsDx: map.dsDx, dsDt: rates[end] }] }).tools[0].velocity;
            }
            return { id: tool.id, sStart: map.sStart, sEnd: map.sStart + map.dsDx * length,
                velocities, interpretation: 'physical-material-velocity', angularVelocity: null,
                materialSpin: null, frameSpin: null };
        }) };
    });
}

/**
 * One fixed-topology nonlinear dt: elastic Chain + exact translational inertia
 * + bilateral lengths, with EXPLICIT quasi-static torsion. Optional wall uses
 * the real vessel capsule field and ONE outer material per edge. This bounded
 * contact mode requires explicit friction:'frictionless'; it is a discrete
 * unilateral solve, without continuous/swept certification or lumen modes.
 * No remeshing, angular inertia, material-history interpolation or app clock.
 *
 * state is NEVER mutated. Success returns a new owned mutable state; failure
 * returns the original state object and commits no frames, velocity or lambda.
 * inertiaEdges[e].tools is Kinematics' REQUIRED prepared mass/map/OLD MATERIAL
 * velocity data. Map rates and old velocities freeze for the whole solve.
 * Callers must sample those old velocities at the current material labels.
 *
 * prescribed:[{dof,value}] supplies absolute root/handle targets. Every tool
 * requires at least one explicit spin boundary. loads is a generalized nodal
 * force/torque vector. Material samples and accepted frames freeze per dt.
 *
 * Each AL inner solve uses banded elastic-GN + exact inertia + length-GN.
 * Backtracking uses the exact augmented objective at fixed lambda and penalty.
 * The outer update lambda+=mu*g becomes only a CANDIDATE physical multiplier.
 * Acceptance independently reassembles base forces and original g/J^T lambda;
 * neither small increments nor augmented energy can certify a timestep.
 */
export function advanceCompositeTimeStep(state, options) {
    const elementBackend = options.elementBackend ?? 'wasm';
    const handle = options.workspace ?? createCompositeTimeStepWorkspace(state.layout, { elementBackend });
    const scratch = timeStepWorkspaces.get(handle);
    if (!scratch || scratch.elementBackend !== elementBackend)
        throw new RangeError('A matching composite timestep workspace/backend is required');
    const edges = state.layout.edgeToolIds, cached = scratch.layout.edgeToolIds;
    if (edges.length !== cached.length || edges.some((ids, e) => ids.length !== cached[e].length || ids.some((id, i) => id !== cached[e][i])))
        throw new RangeError('Changed topology requires a new composite timestep workspace');
    if (scratch.busy) throw new RangeError('A composite timestep workspace cannot be used reentrantly');
    scratch.busy = true; scratch.stats.calls++;
    try { return advancePreparedTimeStep(state, options, scratch); }
    finally { scratch.busy = false; }
}

function advancePreparedTimeStep(state, { dt, torsionMode, inertiaEdges, prescribed = [], loads = null,
    tolerances, initialPenalty, maximumPenalty = 1e12, penaltyGrowth = 4, initialGuess = null,
    budget = {}, elementBackend = 'wasm', wall = null, constraintSolver = 'augmented', inertiaBackend = 'compiled', workspace = null,
    assemblyPolicy = 'auto' }, scratch) {
    positive(dt, 'dt');
    if (!['mixed', 'augmented'].includes(constraintSolver)) throw new TypeError('constraintSolver must be mixed or augmented');
    if (!['compiled', 'oracle'].includes(inertiaBackend)) throw new TypeError('inertiaBackend must be compiled or oracle');
    if (!['auto', 'full', 'lazy'].includes(assemblyPolicy)) throw new TypeError('assemblyPolicy must be auto, full or lazy');
    // Final acceptance needs the original forces and gaps, but no Newton
    // tangent. The oracle remains available for an identical full comparison.
    const certificateOrder = assemblyPolicy !== 'full' && elementBackend === 'wasm-exact' && inertiaBackend === 'compiled'
        ? 'gradient' : 'full';
    // Lazy trial tangents remain explicit: fewer full Hessians do not alone
    // prove lower whole-step cost when extra gradient passes are required.
    const trialOrder = assemblyPolicy === 'lazy' && constraintSolver === 'mixed' ? certificateOrder : 'full';
    if (torsionMode !== 'quasi-static') throw new TypeError('This bounded model requires explicit torsionMode: quasi-static');
    const tolerance = { force: positive(tolerances?.force, 'force tolerance'), torque: positive(tolerances?.torque, 'torque tolerance'),
        length: positive(tolerances?.length, 'length tolerance') };
    const linearTolerance = positive(tolerances?.linear ?? Math.min(tolerance.force, tolerance.torque) * .05, 'linear tolerance');
    let penalty = positive(initialPenalty, 'initialPenalty');
    if (!(positive(maximumPenalty, 'maximumPenalty') >= penalty) || !(positive(penaltyGrowth, 'penaltyGrowth') > 1))
        throw new RangeError('Penalty growth and maximum must preserve a positive initial penalty');
    const limits = { directions: budget.directions ?? 100, outerIterations: budget.outerIterations ?? 16,
        lineSearchTrials: budget.lineSearchTrials ?? 24, evaluations: budget.evaluations ?? 1000,
        linearSolves: budget.linearSolves ?? 3 * (budget.directions ?? 100) };
    for (const [key, value] of Object.entries(limits)) if (!Number.isInteger(value) || value < 0)
        throw new RangeError(`budget.${key} must be a nonnegative integer`);
    const layout = scratch.layout, data = copyData(state.data), vars = variables(data, layout);
    const origin = vars.get(), fixed = new Uint8Array(layout.dofCount), targets = origin.slice();
    for (const boundary of prescribed) {
        const dof = boundary.dof;
        if (!Number.isInteger(dof) || dof < 0 || dof >= layout.dofCount || fixed[dof]) throw new RangeError('Prescribed DOFs must be distinct and in range');
        fixed[dof] = 1; targets[dof] = finite(boundary.value, 'prescribed target');
    }
    for (const [id, offsets] of layout.spins) if (!Array.from(offsets).some(dof => dof >= 0 && fixed[dof]))
        throw new TypeError(`An explicit quasi-static spin boundary is required for ${id}`);
    const load = loads === null ? new Float64Array(layout.dofCount) : Float64Array.from(vector(loads, layout.dofCount, 'loads'));
    if (!inertiaEdges || inertiaEdges.length !== layout.nodeCount - 1) throw new RangeError('One prepared inertia input per edge is required');
    const frozenInertia = structuredClone(inertiaEdges), oldPositions = state.data.positions.map(p => [...p]);
    const lambda = Float64Array.from(vector(state.lengthMultipliers, layout.nodeCount - 1, 'physical length multipliers'));
    const { chain, lengthWorkspace, lengthOriginal, inertiaWorkspaces } = scratch;
    let compiledInertia = null;
    if (wall === null && state.wallContactState !== null && state.wallContactState !== undefined)
        throw new TypeError('An accepted wall history requires an explicit wall model on the next step');
    let wallWorkspace = null, wallOwners = null, wallLambda = null, wallPenalty = null, wallTolerance = null;
    // Transaction-owned cone forces. The collector retains every original
    // query; only its obsolete single-normal equation is replaced locally.
    const sdfSeams = new Map();
    let ordinaryWallView = null, excludedWallRows = null;
    const wallMode = wall?.contactMode ?? (constraintSolver === 'mixed' ? 'envelope' : 'capsule');
    if (!['capsule', 'envelope'].includes(wallMode)) throw new TypeError('wall.contactMode must be capsule or envelope');
    if (wall !== null) {
        if (wall.friction !== 'frictionless') throw new TypeError('This contact stage requires explicit wall.friction: frictionless');
        wallOwners = structuredClone(wall.contactOwners);
        wallWorkspace = scratch.walls.get(wallMode);
        if (!wallWorkspace) {
            wallWorkspace = wallMode === 'envelope' ? createCompositeWallEnvelopeWorkspace(layout) : createCompositeWallWorkspace(layout);
            scratch.walls.set(wallMode, wallWorkspace); scratch.stats.wallBuilds++;
        }
        wallPenalty = positive(wall.initialPenalty, 'wall.initialPenalty');
        if (wallPenalty > maximumPenalty) throw new RangeError('Wall penalty exceeds the explicit maximum');
        wallTolerance = { gapTolerance: positive(wall.tolerances?.gap, 'wall gap tolerance'),
            forceTolerance: positive(wall.tolerances?.force, 'wall force tolerance'),
            workTolerance: positive(wall.tolerances?.work, 'wall work tolerance') };
        if (wallOwners?.edges?.length !== layout.nodeCount - 1) throw new RangeError('Wall ownership must match this fixed topology');
        const owners = wallWorkspace.rows.map(row => wallOwners.edges[row.edge].wall?.owner ?? null);
        const previous = state.wallContactState;
        if (previous && ((previous.contactMode ?? 'capsule') !== wallMode || previous.owners.length !== owners.length || previous.owners.some((id, i) => id !== owners[i])))
            throw new RangeError('Changed wall ownership requires an explicit contact history transfer');
        wallLambda = previous ? Float64Array.from(vector(previous.multipliers, owners.length, 'wall multipliers'))
            : new Float64Array(owners.length);
        ordinaryWallView = { ...wallWorkspace, rows: [...wallWorkspace.rows] };
        excludedWallRows = wallWorkspace.rows.map(row => ({ index: row.index, edge: row.edge, included: false }));
        for (const saved of previous?.sdfSeams ?? []) {
            if (constraintSolver !== 'mixed' || wallMode !== 'capsule') throw new TypeError('SDF cone history requires the mixed capsule contact model');
            const index = saved.wallIndex, row = wallWorkspace.rows[index];
            if (!row || sdfSeams.has(index) || saved.edge !== row.edge || saved.owner !== owners[index] ||
                saved.radius !== wallOwners.edges[row.edge].wall?.radius || wallLambda[index] !== 0)
                throw new RangeError('Changed SDF cone ownership requires an explicit contact history transfer');
            const forces = Float64Array.from(vector(saved.forces, 2, 'SDF cone forces'));
            if (forces.some(v => v < 0)) throw new RangeError('SDF cone forces must be nonnegative');
            sdfSeams.set(index, { ...structuredClone(saved), forces, workspace: createCompositeWallSdfBranchesWorkspace(2) });
        }
    }
    const stats = { limits, evaluations: 0, directions: 0, lineSearchTrials: 0, acceptedTrials: 0,
        rejectedTrials: 0, outerIterations: 0, dualUpdates: 0, penaltyUpdates: 0, historyCommits: 0,
        maximumLinearResidual: 0, linearSolves: 0, refinementSteps: 0, factorizations: 0,
        penalty, initialPenalty, elementBackend, constraintSolver, inertiaBackend, wallQueries: 0, wallDualUpdates: 0,
        reusedWorkspace: workspace !== null,
        fullAssemblies: 0, gradientAssemblies: 0, tangentRebuilds: 0, certificateAssembly: certificateOrder, trialAssembly: trialOrder,
        contactScope: wall === null ? 'contact-free' : 'discrete-frictionless-vessel-contact' };
    stats.sdfChartActivations = 0; stats.sdfChartRebuilds = 0;
    let last = null;
    const passes = certificate => certificate && certificate.force <= tolerance.force && certificate.torque <= tolerance.torque &&
        certificate.length <= tolerance.length && (certificate.wall === null || certificate.wall.converged);
    const failed = (status, error = null) => ({ accepted: false, dt, status, state,
        diagnostics: { torsionMode, tolerance, ...stats, penalty, wallPenalty, certificate: last?.certificate ?? null,
            originalResidual: last?.originalResidual ?? null,
            objective: last?.objective ?? null, error: error?.message ?? null } });
    function refreshSdfChart(index, seam) {
        const row = wallWorkspace.rows[index];
        if (!row.included || row.owner !== seam.owner || row.radius !== seam.radius || row.rawContact.segmentT !== seam.sampleFraction ||
            row.rawContact.capsuleSampleCount !== seam.sampleCount)
            throw new RangeError('Changed SDF cone sample or ownership requires an explicit traction transfer');
        const chart = evaluateCompositeWallSdfBranches({ field: wall.field, face: seam.face,
            positions: [data.positions[row.edge], data.positions[row.edge + 1]], radius: row.radius,
            contact: row.rawContact, dofs: row.dofs }, seam.workspace);
        if (!chart.supported) throw new RangeError(`Unsupported SDF cone chart: ${chart.reason}`);
        return chart;
    }
    function admitSdfChart(index, face) {
        if (constraintSolver !== 'mixed' || wallMode !== 'capsule' || sdfSeams.has(index)) return false;
        const row = wallWorkspace.rows[index];
        if (!row.included || row.source !== 'sparse-sdf' || (wallLambda[index] === 0 && row.gap > 0)) return false;
        const workspace = createCompositeWallSdfBranchesWorkspace(2);
        const chart = evaluateCompositeWallSdfBranches({ field: wall.field, face,
            positions: [data.positions[row.edge], data.positions[row.edge + 1]], radius: row.radius,
            contact: row.rawContact, dofs: row.dofs }, workspace);
        if (!chart.supported) return false;
        const forces = new Float64Array(2); forces[chart.selectedRow] = wallLambda[index];
        sdfSeams.set(index, { wallIndex: index, edge: row.edge, owner: row.owner, radius: row.radius,
            sampleFraction: row.rawContact.segmentT, sampleCount: row.rawContact.capsuleSampleCount,
            face: { axis: face.axis, gridIndex: face.gridIndex }, workspace, forces });
        wallLambda[index] = 0; stats.sdfChartActivations++;
        return true;
    }
    function discoverSdfCharts(delta = null) {
        if (constraintSolver !== 'mixed' || wallMode !== 'capsule' || !wall?.field?.sdfOrigin) return false;
        let added = false;
        for (const row of wallWorkspace.rows) if (row.included && row.source === 'sparse-sdf' && !sdfSeams.has(row.index) &&
            (wallLambda[row.index] !== 0 || row.gap <= 0)) {
            const a = data.positions[row.edge], b = data.positions[row.edge + 1], t = row.rawContact.segmentT;
            const point = a.map((v, i) => t === 0 ? v : t === 1 ? b[i] : v + (b[i] - v) * t);
            const displacement = delta === null ? [0, 0, 0] : [0, 1, 2].map(i =>
                (1 - t) * delta[row.dofs[i]] + t * delta[row.dofs[3 + i]]);
            const crossing = findCompositeWallSdfSeamCrossing({ field: wall.field, position: point, delta: displacement });
            if (crossing.hit && crossing.supported && (delta !== null || crossing.startsOnFace))
                added = admitSdfChart(row.index, crossing.events[0]) || added;
        }
        return added;
    }
    function measureWalls(original, augmented, refresh = true) {
        if (refresh) {
            (wallMode === 'envelope' ? refreshCompositeWallEnvelope : refreshCompositeWallContacts)(
                { positions: data.positions, contactOwners: wallOwners, field: wall.field }, wallWorkspace);
            stats.wallQueries += wallWorkspace.queries;
            discoverSdfCharts();
        }
        for (const [index, seam] of sdfSeams) refreshSdfChart(index, seam);
        for (const row of wallWorkspace.rows) ordinaryWallView.rows[row.index] = sdfSeams.has(row.index) ? excludedWallRows[row.index] : row;
        let wallRepresentatives = null;
        if (constraintSolver === 'mixed') {
            wallRepresentatives = (wallMode === 'envelope' ? canonicalizeCompositeWallEnvelope :
                canonicalizeCompositeWallReactions)(ordinaryWallView, wallLambda, wallOwners);
            wallRepresentatives = wallRepresentatives.filter(index => wallLambda[index] !== 0 || wallWorkspace.rows[index].gap <= 0);
        }
        const measured = measureCompositeWallConstraints(ordinaryWallView, { lambdas: wallLambda, penalty: wallPenalty, ...wallTolerance });
        original.forEach((_, i) => { original[i] += measured.physicalGradient[i]; });
        const { physicalGradient: _physicalGradient, ...wallProof } = measured;
        if (sdfSeams.size) {
            wallProof.scope = 'discrete-wall-KKT-with-sdf-cones'; wallProof.sdfSeams = [];
            wallProof.domainAdmissible = true;
            for (const [index, seam] of sdfSeams) {
                const chart = seam.workspace, proof = measureCompositeWallSdfBranches(chart, { forces: seam.forces,
                    penalty: wallPenalty, ...wallTolerance });
                for (let i = 0; i < 6; i++) original[chart.rows[0].dofs[i]] += proof.physicalGradient[i];
                for (const key of ['maximumPenetration', 'maximumProjectedResidual', 'maximumComplementarity'])
                    wallProof[key] = Math.max(wallProof[key], proof[key]);
                wallProof.converged = wallProof.converged && proof.converged;
                wallProof.domainAdmissible = wallProof.domainAdmissible && proof.domainAdmissible;
                const { physicalGradient: _gradient, nodalForces: _nodal, ...record } = proof;
                wallProof.sdfSeams.push({ wallIndex: index, face: { ...seam.face }, ...record });
                for (const row of chart.rows) if (seam.forces[row.index] !== 0 || row.gap <= 0)
                    wallRepresentatives.push(-1 - 2 * index - row.index);
            }
        }
        const wallTrial = Float64Array.from(wallWorkspace.rows, row => row.included ? Math.max(0, wallLambda[row.index] - wallPenalty * row.gap) : 0);
        if (augmented) assembleCompositeWallAugmented(wallWorkspace, chain, { lambdas: wallLambda, penalty: wallPenalty });
        return { wallProof, wallTrial, wallRepresentatives };
    }
    function assembleMaterials(order) {
        if (stats.evaluations >= limits.evaluations) { const error = new Error('Full assembly evaluation budget exhausted'); error.budget = true; throw error; }
        stats.evaluations++;
        stats[order === 'full' ? 'fullAssemblies' : 'gradientAssemblies']++;
        assembleCompositeChain(data, chain, { order });
        const elasticEnergy = chain.energy;
        let inertialEnergy = 0, kineticEnergy = 0;
        if (compiledInertia !== null) {
            const inertia = compiledInertia.append(data.positions, chain, { order });
            inertialEnergy = inertia.energy; kineticEnergy = inertia.kineticEnergy;
        } else for (let edge = 0; edge < frozenInertia.length; edge++) {
            const tools = frozenInertia[edge].tools;
            const inertia = assembleCompositeTranslationalInertia({ coordinates: [data.coordinates[edge], data.coordinates[edge + 1]],
                positions: [data.positions[edge], data.positions[edge + 1]], previousPositions: [oldPositions[edge], oldPositions[edge + 1]], dt, tools },
                inertiaWorkspaces[tools.length]);
            inertialEnergy += inertia.energy; kineticEnergy += inertia.kineticEnergy;
            scatterCompositeTranslationalInertia(inertia, edge, chain);
        }
        return { elasticEnergy, inertialEnergy, kineticEnergy };
    }
    function evaluate(augmented, order = 'full') {
        if (augmented && order !== 'full') throw new TypeError('Augmented directions require a full tangent');
        const { elasticEnergy, inertialEnergy, kineticEnergy } = assembleMaterials(order);
        const values = vars.get();
        for (let dof = 0; dof < layout.dofCount; dof++) {
            chain.energy -= load[dof] * (values[dof] - origin[dof]); chain.gradient[dof] -= load[dof];
        }
        const physical = measureCompositeLengthConstraints({ positions: data.positions, coordinates: data.coordinates,
            multipliers: lambda, tolerance: tolerance.length }, lengthOriginal);
        const original = Float64Array.from(chain.gradient, (v, i) => v - physical.constraintForces[i]);
        const unwalledResidual = original.slice();
        let wallProof = null, wallTrial = null, wallRepresentatives = null;
        if (wallWorkspace !== null) {
            ({ wallProof, wallTrial, wallRepresentatives } = measureWalls(original, augmented));
        }
        const certificate = { ...residualNorms(original, layout, fixed), length: physical.maximumLengthResidual, wall: wallProof };
        const residuals = physical.residuals.slice(), reactions = Float64Array.from(original, (v, i) => fixed[i] ? v : 0);
        if (augmented) assembleCompositeLengthConstraints({ positions: data.positions, coordinates: data.coordinates,
            multipliers: lambda, penalty, tolerance: tolerance.length }, lengthWorkspace, chain);
        return { objective: finite(chain.energy, 'objective'), certificate, residuals, reactions, originalResidual: original, unwalledResidual, wallTrial, wallRepresentatives,
            augmented: residualNorms(chain.gradient, layout, fixed), elasticEnergy, inertialEnergy, kineticEnergy };
    }
    function commit() {
        // Fresh original equations; no AL trial reaction or stale certificate.
        const proof = evaluate(false, certificateOrder); last = proof;
        if (!passes(proof.certificate)) return failed('original-residual-rejected');
        const frameState = acceptedFrames(data, layout), output = copyData(data);
        // Restore the immutable material providers; only trial sampling was frozen.
        output.tools.forEach((tool, i) => {
            tool.materialAt = state.data.tools[i].materialAt;
            tool.material = state.data.tools[i].material;
            tool.dsDx = state.data.tools[i].dsDx;
            tool.referenceTwists = frameState.windings.get(tool.id);
        });
        output.reference = frameState.reference;
        const velocities = acceptedVelocities(output, oldPositions, frozenInertia, dt);
        stats.historyCommits = 1;
        return { accepted: true, dt, status: 'accepted',
            state: { data: output, layout: createCompositeChainLayout(layout.edgeToolIds), lengthMultipliers: lambda.slice(), time: state.time + dt,
                step: state.step + 1, materialVelocities: velocities, torsionMode,
                wallContactState: wallLambda === null ? null : { multipliers: wallLambda.slice(), contactMode: wallMode,
                    owners: wallWorkspace.rows.map(row => wallOwners.edges[row.edge].wall?.owner ?? null),
                    sdfSeams: Array.from(sdfSeams.values(), seam => ({ wallIndex: seam.wallIndex, edge: seam.edge,
                        owner: seam.owner, radius: seam.radius, sampleFraction: seam.sampleFraction,
                        sampleCount: seam.sampleCount, face: { ...seam.face }, forces: seam.forces.slice() })),
                    records: wallWorkspace.rows.filter(row => row.included).flatMap(row => {
                        const { edge, t, owner, normal } = row;
                        const map = frozenInertia[edge].tools.find(tool => tool.id === owner).materialMap;
                        const base = { edge, fraction: t, owner,
                            s: map.sStart + map.dsDx * (data.coordinates[edge + 1] - data.coordinates[edge]) * t,
                            point: data.positions[edge].map((v, axis) => (1 - t) * v + t * data.positions[edge + 1][axis]),
                            surfacePoint: Array.from(row.closestPoint), normal: Array.from(normal),
                            worldForce: Array.from(normal, v => v * wallLambda[row.index]), worldCouple: [0, 0, 0],
                            history: { normalForce: wallLambda[row.index], friction: 'frictionless' } };
                        const seam = sdfSeams.get(row.index);
                        if (!seam) return [base];
                        return seam.workspace.rows.map(branch => ({ ...base, point: [...seam.workspace.point],
                            source: branch.source, sdfCell: [...branch.cell], sdfFace: { ...seam.face },
                            normal: [...branch.normal], surfacePoint: Array.from(seam.workspace.point, (v, i) => v - branch.signedDistance * branch.normal[i]),
                            worldForce: Array.from(branch.normal, v => v * seam.forces[branch.index]), worldCouple: [0, 0, 0],
                            history: { normalForce: seam.forces[branch.index], friction: 'frictionless' } }));
                    }) } },
            diagnostics: { torsionMode, tolerance, ...stats, penalty, wallPenalty, certificate: proof.certificate,
                originalResidual: proof.originalResidual,
                reactions: proof.reactions, elasticEnergy: proof.elasticEnergy, inertialEnergy: proof.inertialEnergy,
                kineticEnergy: proof.kineticEnergy, objective: proof.objective } };
    }
    function mixedSolve() {
        const positionDofs = new Set(Array.from(layout.positions).flatMap(start => [start, start + 1, start + 2]));
        const merit = evaluation => {
            let value = 0;
            evaluation.originalResidual.forEach((r, i) => {
                if (!fixed[i]) value += (r / (positionDofs.has(i) ? tolerance.force : tolerance.torque)) ** 2;
            });
            for (const r of evaluation.residuals) value += (r / tolerance.length) ** 2;
            if (evaluation.certificate.wall !== null) {
                const proof = evaluation.certificate.wall;
                value += (proof.maximumProjectedResidual / wallTolerance.forceTolerance) ** 2 +
                    (proof.maximumPenetration / wallTolerance.gapTolerance) ** 2 +
                    (proof.maximumComplementarity / wallTolerance.workTolerance) ** 2;
            }
            return finite(value, 'original mixed residual merit');
        };
        let current = evaluate(false, trialOrder); last = current;
        stats.initialCertificate = { ...current.certificate };
        for (let iteration = 0; iteration < limits.outerIterations; iteration++) {
            stats.outerIterations++;
            if (passes(current.certificate)) return commit();
            if (stats.directions >= limits.directions) return failed('direction-budget-exhausted');
            if (stats.linearSolves >= limits.linearSolves) return failed('linear-solve-budget-exhausted');
            if (chain.hessianValid === false) {
                // Rebuild the exact constitutive/inertial tangent at this
                // already measured trial. Frozen material inputs and geometry
                // are unchanged. The mixed solve uses current.originalResidual;
                // contact gaps and physical forces need no extra geometry query.
                assembleMaterials('full'); stats.tangentRebuilds++;
            }
            const { mixed, rows } = mixedWorkspace(scratch, wallMode, current.wallRepresentatives, wallWorkspace);
            const length = lengthOriginal.evaluation;
            rows.forEach(row => {
                const edge = row.edge;
                if (row.kind === 'length') {
                    row.gap = length.residuals[edge]; row.multiplier = lambda[edge];
                    row.jacobian.set(length.jacobian.subarray(6 * edge, 6 * edge + 6));
                    for (let i = 0; i < 6; i++) for (let j = 0; j < 6; j++) {
                        const a = i % 3, b = j % 3, sign = (i < 3) === (j < 3) ? 1 : -1;
                        row.geometricHessian[6 * i + j] = sign * lambda[edge] / length.lengths[edge] *
                            ((a === b ? 1 : 0) - length.directions[3 * edge + a] * length.directions[3 * edge + b]);
                    }
                } else {
                    const seam = row.sdfBranch === null ? null : sdfSeams.get(row.wallIndex);
                    const contact = seam ? seam.workspace.rows[row.sdfBranch] : wallWorkspace.rows[row.wallIndex];
                    row.allowSignedWallIterate = seam !== null;
                    row.gap = contact.gap; row.multiplier = seam ? seam.forces[row.sdfBranch] : wallLambda[row.wallIndex]; row.penalty = wallPenalty;
                    // Geometry differentiates the actual provider gap. Its
                    // normalized normal distributes physical Fn and has its
                    // own derivative; sparse-SDF generally makes this block
                    // nonsymmetric. The collector rejects unsupported rows.
                    row.jacobian.set(contact.gapJacobian);
                    row.forceColumn.set(contact.forceColumn);
                    for (let i = 0; i < 36; i++) row.geometricHessian[i] = -row.multiplier * contact.normalDerivative[i];
                }
            });
            stats.directions++;
            const direction = solveCompositeMixedDirection(mixed, chain, { rows, gradient: current.originalResidual, fixed,
                tolerances: { force: linearTolerance, torque: linearTolerance,
                    constraint: Math.min(tolerance.length * .05, wallTolerance === null ? Infinity :
                        Math.min(wallTolerance.gapTolerance, wallTolerance.forceTolerance / wallPenalty) * .05) },
                maxCorrections: Math.min(1, limits.linearSolves - stats.linearSolves - 1) });
            stats.linearSolves += direction.linearSolves; stats.factorizations += direction.factorizations;
            stats.refinementSteps += direction.linearSolves - 1; stats.lastLinearProof = direction.proof;
            stats.mixedCount = direction.count; stats.mixedBandwidth = direction.bandwidth;
            if (!direction.converged) return failed('mixed-linear-residual-rejected');
            const delta = direction.increment, dualDelta = direction.multiplierIncrement;
            if (discoverSdfCharts(delta)) {
                // Rebuild just the contact parameterization at the SAME state
                // and already captured original query. No repeated provider
                // query, material assembly, geometric snap or time substep.
                const original = current.unwalledResidual.slice(), measured = measureWalls(original, false, false);
                current = { ...current, originalResidual: original, wallTrial: measured.wallTrial,
                    wallRepresentatives: measured.wallRepresentatives,
                    reactions: Float64Array.from(original, (v, i) => fixed[i] ? v : 0),
                    certificate: { ...residualNorms(original, layout, fixed), length: current.certificate.length, wall: measured.wallProof } };
                last = current; stats.sdfChartRebuilds++; continue;
            }
            const before = vars.get(), beforeLambda = lambda.slice(), beforeWall = wallLambda?.slice(), initialMerit = merit(current);
            const beforeSeams = new Map(Array.from(sdfSeams, ([index, seam]) => [index, { seam, forces: seam.forces.slice() }]));
            let acceptedTrial = false;
            for (let trial = 0; trial < limits.lineSearchTrials; trial++) {
                stats.lineSearchTrials++;
                const scale = 2 ** (-trial);
                vars.set(Float64Array.from(before, (v, i) => fixed[i] ? targets[i] : v + scale * delta[i]));
                // A rejected trial may redistribute exact dependent wall
                // reactions. Every retry starts from the same physical dual.
                if (wallLambda !== null) wallLambda.set(beforeWall);
                sdfSeams.clear();
                for (const [index, saved] of beforeSeams) { saved.seam.forces.set(saved.forces); sdfSeams.set(index, saved.seam); }
                rows.forEach((row, i) => {
                    if (row.kind === 'length') lambda[row.edge] = beforeLambda[row.edge] + scale * dualDelta[i];
                    else {
                        // On the inactive NCP branch the exact direction is
                        // -lambda. Evaluate that affine update directly so a
                        // full release is exactly zero, without a force floor.
                        const seam = row.sdfBranch === null ? null : sdfSeams.get(row.wallIndex);
                        const previous = seam ? beforeSeams.get(row.wallIndex).forces[row.sdfBranch] : beforeWall[row.wallIndex];
                        const inactive = previous - row.penalty * row.gap <= 0;
                        // A private SDF NCP unknown may cross zero before the
                        // next branch evaluation releases that face. Clamping
                        // only its dual destroys the Newton equation and can
                        // pin a releasing contact to g=0. The original cone
                        // certificate forbids every negative accepted force.
                        const next = inactive ? (scale === 1 ? 0 : (1 - scale) * previous) : seam ? previous + scale * dualDelta[i]
                            : Math.max(0, previous + scale * dualDelta[i]);
                        if (seam) seam.forces[row.sdfBranch] = next;
                        else wallLambda[row.wallIndex] = next;
                    }
                });
                let candidate;
                try { candidate = evaluate(false, trialOrder); }
                catch (error) { if (error.budget) throw error; stats.rejectedTrials++; continue; }
                if (passes(candidate.certificate) || merit(candidate) <= initialMerit * (1 - 1e-4 * scale)) {
                    current = candidate; last = current; acceptedTrial = true; stats.acceptedTrials++; stats.dualUpdates++;
                    if (wallLambda !== null) stats.wallDualUpdates++;
                    break;
                }
                stats.rejectedTrials++;
            }
            if (!acceptedTrial) return failed('line-search-budget-exhausted');
            if (passes(current.certificate)) return commit();
        }
        return failed('outer-budget-exhausted');
    }
    try {
        frozenMaterials(data, layout);
        if (inertiaBackend === 'compiled') compiledInertia = createCompositeInertiaCache({ layout,
            coordinates: data.coordinates, previousPositions: oldPositions, dt, inertiaEdges: frozenInertia });
        if (initialGuess !== null) vars.set(vector(initialGuess, layout.dofCount, 'initialGuess'));
        const initial = vars.get(); initial.forEach((_, i) => { if (fixed[i]) initial[i] = targets[i]; }); vars.set(initial);
        if (constraintSolver === 'mixed') return mixedSolve();
        let previousLength = Infinity, previousWallPenetration = Infinity;
        for (let outer = 0; outer < limits.outerIterations; outer++) {
            stats.outerIterations++;
            let current = evaluate(true); last = current;
            stats.initialCertificate ??= { ...current.certificate };
            if (passes(current.certificate)) return commit();
            while (current.augmented.force > tolerance.force * .1 || current.augmented.torque > tolerance.torque * .1) {
                if (stats.directions >= limits.directions) return failed('direction-budget-exhausted');
                if (stats.linearSolves >= limits.linearSolves) return failed('linear-solve-budget-exhausted');
                stats.directions++;
                const direction = solveCompositeChainIncrement(chain, { fixed, tolerance: linearTolerance,
                    maxRefinementSteps: Math.min(2, limits.linearSolves - stats.linearSolves - 1) });
                stats.linearSolves += direction.linearSolves;
                stats.refinementSteps += direction.refinementSteps;
                stats.factorizations += direction.factorizations;
                stats.maximumLinearResidual = Math.max(stats.maximumLinearResidual, direction.maximumResidual);
                if (!direction.converged) return failed('linear-residual-rejected');
                const delta = direction.increment.slice(), slope = dot(chain.gradient, delta);
                if (!(slope < 0) || !Number.isFinite(slope)) return failed('no-descent-direction');
                const before = vars.get(); let acceptedTrial = false;
                for (let trial = 0; trial < limits.lineSearchTrials; trial++) {
                    stats.lineSearchTrials++;
                    const scale = 2 ** (-trial), next = Float64Array.from(before, (v, i) => fixed[i] ? targets[i] : v + scale * delta[i]);
                    vars.set(next);
                    let candidate;
                    try { candidate = evaluate(true); }
                    catch (error) { if (error.budget) throw error; stats.rejectedTrials++; continue; }
                    if (candidate.objective <= current.objective + 1e-4 * scale * slope) {
                        current = candidate; last = current; stats.acceptedTrials++; acceptedTrial = true; break;
                    }
                    stats.rejectedTrials++;
                }
                if (!acceptedTrial) { vars.set(before); return failed('line-search-budget-exhausted'); }
                if (passes(current.certificate)) return commit();
            }
            // This remains transaction-local until the original equations pass.
            lambda.forEach((_, edge) => { lambda[edge] += penalty * current.residuals[edge]; }); stats.dualUpdates++;
            if (wallLambda !== null) { wallLambda.set(current.wallTrial); stats.wallDualUpdates++; }
            const original = evaluate(false); last = original;
            if (passes(original.certificate)) return commit();
            if (original.certificate.length > previousLength * .25) {
                if (penalty * penaltyGrowth > maximumPenalty) return failed('penalty-budget-exhausted');
                penalty *= penaltyGrowth; stats.penaltyUpdates++;
            }
            previousLength = original.certificate.length;
            if (wallLambda !== null) {
                const penetration = original.certificate.wall.maximumPenetration;
                if (penetration > wallTolerance.gapTolerance && penetration > previousWallPenetration * .25) {
                    if (wallPenalty * penaltyGrowth > maximumPenalty) return failed('wall-penalty-budget-exhausted');
                    wallPenalty *= penaltyGrowth; stats.penaltyUpdates++;
                }
                previousWallPenetration = penetration;
            }
        }
        return failed('outer-budget-exhausted');
    } catch (error) { return failed(error.budget ? 'evaluation-budget-exhausted' : 'numerical-rejection', error); }
}
