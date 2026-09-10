import { sampleKirchhoffCompositeSection } from './kirchhoffCompositeTopology.js';
import { captureCompositeReferenceFrames, compileCompositeMaterial } from './kirchhoffCompositeElement.js';
import { createCompositeChainLayout } from './kirchhoffCompositeChain.js';

const finite = (value, name) => {
    if (!Number.isFinite(value)) throw new TypeError(`${name} must be finite`);
    return value;
};
const vector = (value, size, name) => {
    if (!value || value.length !== size) throw new TypeError(`${name} needs ${size} values`);
    return Array.from(value, v => finite(v, name));
};
const midpoint = (a, b) => a + (b - a) / 2;
const dot = (a, b) => a.reduce((sum, v, i) => sum + v * b[i], 0);
const matvec = (k, v) => [0, 1, 2].map(i => k[3 * i] * v[0] + k[3 * i + 1] * v[1] + k[3 * i + 2] * v[2]);

function quadratureOptions(input = {}) {
    const result = { absoluteTolerance: input.absoluteTolerance ?? 1e-9,
        relativeTolerance: input.relativeTolerance ?? 1e-8, maxDepth: input.maxDepth ?? 16 };
    for (const key of ['absoluteTolerance', 'relativeTolerance'])
        if (!(finite(result[key], key) >= 0)) throw new RangeError(`${key} must be nonnegative`);
    if (!(result.absoluteTolerance + result.relativeTolerance > 0)) throw new RangeError('A positive quadrature tolerance is required');
    if (!Number.isInteger(result.maxDepth) || result.maxDepth < 0 || result.maxDepth > 30)
        throw new RangeError('maxDepth must be an integer from 0 to 30');
    return result;
}

// Adaptive Simpson comparison, with positive Boole weights on accepted leaves.
// The error is a sampled estimate, not a bound on arbitrary unseen functions.
function integrate(sample, start, end, options, statistics, supportLength, depth = 0) {
    const middle = midpoint(start, end), q1 = midpoint(start, middle), q3 = midpoint(middle, end);
    if (!(start < q1 && q1 < middle && middle < q3 && q3 < end))
        throw new RangeError('Material quadrature interval needs representable interior samples');
    const samples = [start, q1, middle, q3, end].map(sample), h = end - start;
    const fine = samples[0].map((_, i) => h * (samples[0][i] + 4 * samples[1][i] + 2 * samples[2][i] + 4 * samples[3][i] + samples[4][i]) / 12);
    const coarse = samples[0].map((_, i) => h * (samples[0][i] + 4 * samples[2][i] + samples[4][i]) / 6);
    const errors = fine.map((v, i) => Math.abs(v - coarse[i]) / 15);
    const converged = errors.every((error, i) => error <= options.absoluteTolerance * h / supportLength +
        options.relativeTolerance * Math.max(Math.abs(fine[i]), Math.abs(coarse[i])));
    if (!converged) {
        if (depth >= options.maxDepth) throw new RangeError(`Material quadrature did not converge on [${start},${end}]`);
        const a = integrate(sample, start, middle, options, statistics, supportLength, depth + 1);
        const b = integrate(sample, middle, end, options, statistics, supportLength, depth + 1);
        return a.map((v, i) => v + b[i]);
    }
    statistics.leaves++; statistics.maximumDepth = Math.max(statistics.maximumDepth, depth);
    statistics.maximumEstimatedComponentError = Math.max(statistics.maximumEstimatedComponentError, ...errors);
    return samples[0].map((_, i) => h * (7 * samples[0][i] + 32 * samples[1][i] + 12 * samples[2][i] +
        32 * samples[3][i] + 7 * samples[4][i]) / 90);
}

function solvePositive3(k, rhs) {
    const l = new Float64Array(9), y = new Float64Array(3), x = new Float64Array(3);
    for (let i = 0; i < 3; i++) for (let j = 0; j <= i; j++) {
        let value = k[3 * i + j];
        for (let h = 0; h < j; h++) value -= l[3 * i + h] * l[3 * j + h];
        if (i === j && !(value > 0)) throw new RangeError('Integrated material must remain positive definite');
        l[3 * i + j] = i === j ? Math.sqrt(value) : value / l[3 * j + j];
    }
    for (let i = 0; i < 3; i++) {
        let value = rhs[i]; for (let j = 0; j < i; j++) value -= l[3 * i + j] * y[j];
        y[i] = value / l[3 * i + i];
    }
    for (let i = 2; i >= 0; i--) {
        let value = y[i]; for (let j = i + 1; j < 3; j++) value -= l[3 * j + i] * x[j];
        x[i] = value / l[3 * i + i];
    }
    return x;
}

function integrateMaterial(topology, id, start, end, nominalLength, dsDx, options) {
    const pieces = topology.sections.map((section, sectionIndex) => ({ section, sectionIndex,
        start: Math.max(start, section.start), end: Math.min(end, section.end) }))
        .filter(piece => piece.start < piece.end && piece.section.tools.some(tool => tool.id === id));
    if (!pieces.length || pieces[0].start !== start || pieces.at(-1).end !== end ||
        pieces.some((piece, i) => i && piece.start !== pieces[i - 1].end))
        throw new RangeError(`Material support for ${id} is not continuously covered`);
    const statistics = { evaluations: 0, leaves: 0, maximumDepth: 0, maximumEstimatedComponentError: 0,
        converged: true, certified: false };
    const integral = new Float64Array(12);
    let intrinsicAnchor = null;
    for (const piece of pieces) {
        const source = piece.section.tools.find(tool => tool.id === id);
        if (source.dsDx !== dsDx) throw new RangeError('A tool needs one dsDx over a hinge support');
        const section = { ...piece.section, tools: [source] }, cache = new Map();
        piece.at = x => {
            if (!cache.has(x)) {
                const sample = sampleKirchhoffCompositeSection(section, { x })[0];
                const raw = typeof sample.material === 'function' ? sample.material(sample.s) : sample.material;
                const compiled = compileCompositeMaterial(raw), energyOffset = raw.energyOffset ?? 0;
                if (!(finite(energyOffset, 'material energyOffset') >= 0)) throw new RangeError('material energyOffset must be nonnegative');
                cache.set(x, { stiffness: compiled.stiffness, intrinsic: compiled.intrinsic, energyOffset });
                statistics.evaluations++;
            }
            return cache.get(x);
        };
        intrinsicAnchor ??= Array.from(piece.at(piece.start).intrinsic);
        const values = integrate(x => {
            const m = piece.at(x);
            const relative = m.intrinsic.map((v, i) => v - intrinsicAnchor[i]);
            return [...m.stiffness, ...matvec(m.stiffness, relative)].map(v => v * dsDx);
        }, piece.start, piece.end, options, statistics, end - start);
        integral.forEach((_, i) => { integral[i] += values[i]; });
    }
    // Solve for a small shift from an actual material sample, so a large
    // common intrinsic value does not contaminate the weighted mean itself.
    const intrinsic = solvePositive3(integral, integral.subarray(9)).map((v, i) => v + intrinsicAnchor[i]);
    const materialLength = nominalLength * dsDx;
    // A second positive integral preserves mismatch without subtracting two
    // large, almost equal energies. Each material/profile jump stays split.
    let mismatchIntegral = 0;
    for (const piece of pieces) mismatchIntegral += integrate(x => {
        const m = piece.at(x), error = m.intrinsic.map((v, i) => v - intrinsic[i]);
        return [dsDx * (.5 * dot(error, matvec(m.stiffness, error)) + m.energyOffset)];
    }, piece.start, piece.end, options, statistics, end - start)[0];
    const energyOffset = mismatchIntegral / materialLength;
    const stiffness = [0, 1, 2].map(i => Array.from(integral.subarray(3 * i, 3 * i + 3), v => v / materialLength));
    const material = compileCompositeMaterial({ stiffness, intrinsic, energyOffset });
    if (material.energyOffset !== energyOffset)
        throw new Error('Composite Element must preserve compiled material.energyOffset per material length');
    return { material, id, start, end, nominalLength, materialLength,
        energyAtZeroStrain: materialLength * (.5 * dot(intrinsic, matvec(material.stiffness, intrinsic)) + energyOffset),
        integratedMaterialLength: dsDx * (end - start), quadrature: statistics,
        pieces: pieces.map(piece => ({ start: piece.start, end: piece.end, sectionIndex: piece.sectionIndex })) };
}

/**
 * Compile one constitutive chain from topology and explicit spatial/spin fields.
 * Required topology boundaries are inserted exactly (default), or rejected via
 * boundaryPolicy:'reject'. No interpolation of angles through missing material.
 * Each present tool receives >=2 edges: a midpoint is inserted if its initial
 * covered interval has just one edge, so first insertion retains a hinge.
 *
 * spinFields[id] is a finite constant or function(s,context) returning radians
 * relative to context.reference. Reference frames are captured once for this
 * new mesh, or supplied through referenceFrames. Remeshing/history transfer is
 * the caller's responsibility; frames are never recaptured during assembly.
 * referenceTwistFields[id] optionally supplies the accepted continuous winding
 * at each hinge, as a constant or function(s,context); a new mesh defaults to 0.
 *
 * Material properties are frozen quadrature results for fixed coordinates.
 * Hinge supports use Chain's nominal midpoint-to-midpoint Voronoi cells.
 * Unassembled boundary half-cells are returned separately with their material
 * energy polynomial. They never extend a hinge's weight or alter the discrete
 * cantilever convention. This is constant-strain quadrature, not stiffness blending.
 *
 * approvedSections must contain current topology.sections object references.
 * Without explicit approval, overlap edges are candidate common-axis elements;
 * this adapter never certifies clearance, contacts, or quadrature error bounds.
 */
export function buildKirchhoffCompositeMesh({ topology, meshCoordinates, sampleCenterline, spinFields,
    boundaryPolicy = 'insert', initialDirector = [0, 1, 0], referenceFrames = null,
    referenceTwistFields = {}, approvedSections = [], quadrature = {}, materialIntegrator = null }) {
    if (materialIntegrator !== null && typeof materialIntegrator !== 'function')
        throw new TypeError('materialIntegrator must be a function');
    const compileMaterialCell = materialIntegrator === null ? integrateMaterial
        : (...args) => materialIntegrator(...args, integrateMaterial);
    if (!topology?.sections?.length || !topology.coversDomain || !topology.connected)
        throw new RangeError('A composite mesh requires a nonempty, continuously covered topology domain');
    if (!meshCoordinates || meshCoordinates.length < 2) throw new RangeError('At least two explicit mesh coordinates are required');
    if (typeof sampleCenterline !== 'function') throw new TypeError('sampleCenterline(x) is required');
    if (!['insert', 'reject'].includes(boundaryPolicy)) throw new RangeError('boundaryPolicy must be insert or reject');
    const requested = Array.from(meshCoordinates, x => finite(x, 'mesh coordinate'));
    if (requested.some((x, i) => i && x <= requested[i - 1])) throw new RangeError('meshCoordinates must strictly increase');
    const [start, end] = topology.interval;
    if (requested.some(x => x < start || x > end)) throw new RangeError('meshCoordinates lie outside the topology domain');
    const points = new Set(requested), insertedCoordinates = [];
    for (const boundary of topology.boundaries) if (!points.has(boundary.x)) {
        if (boundaryPolicy === 'reject') throw new RangeError(`Mesh misses topology boundary ${boundary.x}`);
        points.add(boundary.x); insertedCoordinates.push(boundary.x);
    }
    const strainCoordinates = [];
    for (const input of topology.tools) {
        const covered = topology.sections.filter(section => section.tools.some(tool => tool.id === input.id));
        if (!covered.length) continue;
        const a = covered[0].start, b = covered.at(-1).end;
        if ([...points].filter(x => x >= a && x <= b).length === 2) {
            const middle = midpoint(a, b);
            if (!(middle > a && middle < b)) throw new RangeError(`${input.id} strain midpoint is not representable`);
            points.add(middle); insertedCoordinates.push(middle); strainCoordinates.push(middle);
        }
    }
    const coordinates = Float64Array.from([...points].sort((a, b) => a - b));
    const positions = Array.from(coordinates, x => vector(sampleCenterline(x), 3, 'centerline position'));
    let cursor = 0;
    const edgeSections = Array.from({ length: coordinates.length - 1 }, (_, edge) => {
        while (coordinates[edge] >= topology.sections[cursor].end) cursor++;
        const section = topology.sections[cursor];
        if (coordinates[edge] < section.start || coordinates[edge + 1] > section.end)
            throw new RangeError('An edge crosses a topology boundary');
        return cursor;
    });
    const edgeToolIds = edgeSections.map(index => topology.sections[index].tools.map(tool => tool.id));
    const layout = createCompositeChainLayout(edgeToolIds);
    const reference = referenceFrames === null ? captureCompositeReferenceFrames(positions, initialDirector)
        : Array.from(referenceFrames, frame => ({ tangent: vector(frame.tangent, 3, 'reference tangent'),
            director: vector(frame.director, 3, 'reference director') }));
    if (reference.length !== edgeToolIds.length) throw new RangeError('referenceFrames must match final mesh edges');
    const options = quadratureOptions(quadrature), materialCells = [], boundaryCells = [], materialMaps = new Map();
    const tools = [...layout.spins.keys()].map(id => {
        const input = topology.tools.find(tool => tool.id === id);
        const covered = edgeToolIds.map((ids, i) => ids.includes(id) ? i : -1).filter(i => i >= 0);
        if (covered.length < 2) throw new RangeError(`${id} needs at least two mesh edges to retain its strain energy`);
        const field = spinFields?.[id];
        if (typeof field !== 'function' && !Number.isFinite(field)) throw new TypeError(`An explicit finite spin field is required for ${id}`);
        const first = covered[0], last = covered.at(-1), angles = new Float64Array(edgeToolIds.length).fill(NaN);
        const maps = new Array(edgeToolIds.length).fill(null);
        for (const edge of covered) {
            const x = midpoint(coordinates[edge], coordinates[edge + 1]);
            const section = topology.sections[edgeSections[edge]], source = section.tools.find(tool => tool.id === id);
            const label = x => x === source.end ? source.materialInterval[1] : x === source.start ? source.materialInterval[0]
                : source.materialInterval[1] + source.dsDx * (x - source.insertion);
            const context = { id, x, edge, start: coordinates[edge], end: coordinates[edge + 1],
                materialStart: label(coordinates[edge]), materialEnd: label(coordinates[edge + 1]),
                dsDx: source.dsDx, reference: reference[edge] };
            maps[edge] = { sStart: context.materialStart, sEnd: context.materialEnd, s: label(x), dsDx: source.dsDx };
            angles[edge] = finite(typeof field === 'function' ? field(label(x), context) : field, `${id} spin`);
        }
        materialMaps.set(id, maps);
        const cells = new Map(), referenceTwists = new Float64Array(layout.hinges.length).fill(NaN);
        const winding = referenceTwistFields[id] ?? 0;
        if (typeof winding !== 'function' && !Number.isFinite(winding)) throw new TypeError(`${id} reference winding must be finite`);
        for (const hinge of layout.hinges.filter(hinge => hinge.tools.includes(id))) {
            // Match Chain's arithmetic as well as its Voronoi convention.
            const i = hinge.vertex, nominalStart = (coordinates[i - 1] + coordinates[i]) / 2,
                nominalEnd = (coordinates[i] + coordinates[i + 1]) / 2;
            const cell = compileMaterialCell(topology, id, nominalStart, nominalEnd,
                (coordinates[i + 1] - coordinates[i - 1]) / 2, input.dsDx, options);
            const label = input.materialInterval[1] + input.dsDx * (coordinates[i] - input.insertion);
            referenceTwists[i - 1] = finite(typeof winding === 'function' ? winding(label, {
                id, vertex: i, coordinate: coordinates[i], reference: [reference[i - 1], reference[i]]
            }) : winding, `${id} reference winding`);
            Object.assign(cell, { vertex: i, coordinate: coordinates[i], nominalStart, nominalEnd });
            cells.set(i, cell); materialCells.push(cell);
        }
        for (const [side, edge, a, b] of [
            ['proximal', first, coordinates[first], (coordinates[first] + coordinates[first + 1]) / 2],
            ['distal', last, (coordinates[last] + coordinates[last + 1]) / 2, coordinates[last + 1]]
        ]) boundaryCells.push({ ...compileMaterialCell(topology, id, a, b, b - a, input.dsDx, options),
            side, assembled: false, strain: null,
            // Geometry refers to the one live position field rather than a
            // duplicated boundary pose that could go stale during a solve.
            geometry: { edge, positionNodes: [edge, edge + 1],
                startFraction: (a - coordinates[edge]) / (coordinates[edge + 1] - coordinates[edge]),
                endFraction: (b - coordinates[edge]) / (coordinates[edge + 1] - coordinates[edge]) },
            materialCoordinates: { sStart: input.materialInterval[1] + input.dsDx * (a - input.insertion),
                sEnd: input.materialInterval[1] + input.dsDx * (b - input.insertion), dsDx: input.dsDx } });
        return { id, angles, referenceTwists, dsDx: input.dsDx, materialAt({ vertex, coordinate, start, end }) {
            const cell = cells.get(vertex);
            if (!cell || coordinate !== cell.coordinate || start !== cell.nominalStart || end !== cell.nominalEnd)
                throw new RangeError('Material mesh coordinates changed; rebuild the composite mesh');
            return cell.material;
        } };
    });
    if (!Array.isArray(approvedSections) || approvedSections.some(section => !topology.sections.includes(section)))
        throw new TypeError('approvedSections must identify sections of the current topology');
    const approval = new Set(approvedSections), candidateSections = topology.sections
        .map((section, i) => section.kind === 'overlap' && !approval.has(section) ? i : -1).filter(i => i >= 0);
    const nodeByCoordinate = new Map(Array.from(coordinates, (x, i) => [x, i]));
    return { data: { positions, coordinates, reference, tools }, layout, materialCells, boundaryCells, materialMaps,
        boundaryRule: { hinges: 'nominal-voronoi', halfCells: 'reported-unassembled' },
        insertedCoordinates: insertedCoordinates.sort((a, b) => a - b), edgeSections,
        strainCoordinates,
        contactOwners: {
            edges: edgeSections.map((sectionIndex, edge) => ({ edge, sectionIndex, start: coordinates[edge],
                end: coordinates[edge + 1], ...structuredClone(topology.sections[sectionIndex].contacts) })),
            nodes: topology.boundaries.map(boundary => ({ node: nodeByCoordinate.get(boundary.x), ...structuredClone(boundary) }))
        }, admission: { status: candidateSections.length ? 'candidate' : 'caller-approved',
            candidate: candidateSections.length > 0, candidateSections, certified: false, clearanceCertified: false },
        quadrature: { converged: true, certified: false,
            evaluations: [...materialCells, ...boundaryCells].reduce((sum, cell) => sum + cell.quadrature.evaluations, 0) } };
}
