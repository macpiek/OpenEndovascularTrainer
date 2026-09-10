const finite = (value, name) => {
    if (!Number.isFinite(value)) throw new RangeError(`${name} must be finite`);
    return value;
};
const positive = (value, name) => {
    if (!(finite(value, name) > 0)) throw new RangeError(`${name} must be positive`);
    return value;
};
const identifier = value => {
    if (typeof value !== 'string' || !value.length) throw new TypeError('A physical tool id is required');
    return value;
};
function vector(value) {
    if (value?.length !== 3) throw new RangeError('Physical material velocity needs three components');
    return Object.freeze(Array.from(value, v => finite(v, 'Physical material velocity')));
}
function ownSpan(record, source, edge = null) {
    const id = identifier(record?.id), sStart = finite(record.sStart, 'History start label'), sEnd = finite(record.sEnd, 'History end label');
    if (!(sEnd > sStart) || !Number.isFinite(sEnd - sStart)) throw new RangeError('History labels must delimit a finite positive affine span');
    if (record.interpretation === 'quintic-bernstein-material-velocity') {
        if (record.bernsteinVelocities?.length !== 6 || record.velocities !== undefined) throw new RangeError('Polynomial material history requires exactly six Bernstein velocity controls');
        return Object.freeze({id,sStart,sEnd,bernsteinVelocities:Object.freeze(record.bernsteinVelocities.map(vector)),source,edge});
    }
    if (record.interpretation !== 'physical-material-velocity' || record.velocities?.length !== 2)
        throw new RangeError('Explicit physical material velocity at both affine span endpoints is required');
    return Object.freeze({ id, sStart, sEnd, velocities: Object.freeze(record.velocities.map(vector)), source, edge });
}
function velocityAt(span, s) {
    if (!(s >= span.sStart && s <= span.sEnd)) throw new RangeError('Material history extrapolation is forbidden');
    if (span.bernsteinVelocities) {
        if(s===span.sStart)return span.bernsteinVelocities[0].slice();
        if(s===span.sEnd)return span.bernsteinVelocities[5].slice();
        const u=(s-span.sStart)/(span.sEnd-span.sStart),levels=span.bernsteinVelocities.map(v=>v.slice());
        for(let degree=5;degree>0;degree--)for(let i=0;i<degree;i++)for(let k=0;k<3;k++)levels[i][k]=(1-u)*levels[i][k]+u*levels[i+1][k];
        return Array.from(vector(levels[0]));
    }
    // Endpoints retain their exact one-sided values.
    if (s === span.sStart) return span.velocities[0].slice();
    if (s === span.sEnd) return span.velocities[1].slice();
    const fraction = (s - span.sStart) / (span.sEnd - span.sStart);
    return span.velocities[0].map((v, k) => finite((1 - fraction) * v + fraction * span.velocities[1][k], 'Interpolated material velocity'));
}
function missing(toolId, s, trace) {
    const error = new RangeError(`No own accepted material history or explicit reservoir at ${toolId}:${s} (${trace})`);
    error.code = 'missing-material-history';
    return error;
}
const GAUSS = [[(1 - 1 / Math.sqrt(3)) / 2,.5], [(1 + 1 / Math.sqrt(3)) / 2,.5]];
const GAUSS6 = [[.033765242898423975,.08566224618958517],[.16939530676686776,.1803807865240693],[.3806904069584015,.23395696728634552],
    [.6193095930415985,.23395696728634552],[.8306046932331322,.1803807865240693],[.966234757101576,.08566224618958517]];
function restrictControls(span,start,end) {
    const split=(controls,u)=>{
        const level=controls.map(v=>v.slice()),left=[level[0].slice()],right=[level[5].slice()];
        for(let degree=5;degree>0;degree--){
            for(let i=0;i<degree;i++)for(let k=0;k<3;k++)level[i][k]=(1-u)*level[i][k]+u*level[i+1][k];
            left.push(level[0].slice());right.unshift(level[degree-1].slice());
        }return {left,right};
    };
    const a=(start-span.sStart)/(span.sEnd-span.sStart),b=(end-span.sStart)/(span.sEnd-span.sStart);
    let controls=span.bernsteinVelocities.map(v=>v.slice());
    if(b!==1)controls=split(controls,b).left;
    if(a!==0)controls=split(controls,a/b).right;
    return controls.map(v=>Array.from(vector(v)));
}

/** Own accepted TRANSLATIONAL history by physical material label, never mesh
 * index, common-axis position, or an already advected query. Each source edge
 * is an explicitly affine or quintic Bernstein velocity field. All boundaries are preserved,
 * including continuous values with a derivative jump. There is no angular or
 * material-spin reconstruction and no geometry/history mutation.
 *
 * A missing label needs reservoir({toolId,s,trace}), returning an explicit
 * affine record {id,sStart,sEnd,velocities,interpretation}, or null if unknown.
 * The provider must be deterministic for a prepared/retried input. Its returned
 * arrays are copied; no provider values are cached across calls or failures.
 */
export function createCompositeJointMaterialHistory({ materialVelocities, reservoir = null } = {}) {
    if (materialVelocities !== null && !Array.isArray(materialVelocities)) throw new TypeError('Explicit accepted materialVelocities array or null is required');
    if (reservoir !== null && typeof reservoir !== 'function') throw new TypeError('An explicit reservoir provider must be callable');
    const byTool = new Map(), edges = new Set();
    for (const entry of materialVelocities ?? []) {
        if (!Number.isInteger(entry?.edge) || entry.edge < 0 || edges.has(entry.edge) || !Array.isArray(entry.tools) || !entry.tools.length)
            throw new RangeError('Accepted history needs distinct nonnegative edge records and their active materials');
        edges.add(entry.edge);
        const ids = new Set();
        for (const record of entry.tools) {
            const span = ownSpan(record, 'accepted', entry.edge);
            if (ids.has(span.id)) throw new RangeError('Duplicate material history on one accepted edge');
            ids.add(span.id);
            if (!byTool.has(span.id)) byTool.set(span.id, []);
            byTool.get(span.id).push(span);
        }
    }
    for (const spans of byTool.values()) {
        spans.sort((a, b) => a.sStart - b.sStart);
        for (let i = 1; i < spans.length; i++)
            if (spans[i].sStart < spans[i - 1].sEnd) throw new RangeError('Overlapping own material histories are ambiguous');
        Object.freeze(spans);
    }
    function resolve(toolId, s, trace) {
        const spans = byTool.get(toolId) ?? [];
        // Binary search by label; edge numbering and exposure are irrelevant.
        let lo = 0, hi = spans.length;
        while (lo < hi) {
            const mid = (lo + hi) >>> 1;
            if (spans[mid].sStart <= s) lo = mid + 1; else hi = mid;
        }
        for (const i of [lo - 1, lo - 2]) {
            const span = spans[i];
            if (span && (trace === 'left' ? s > span.sStart && s <= span.sEnd : s >= span.sStart && s < span.sEnd)) return span;
        }
        if (!reservoir) return null;
        const supplied = reservoir(Object.freeze({ toolId, s, trace }));
        if (supplied === null || supplied === undefined) return null;
        const span = ownSpan(supplied, 'reservoir');
        if (span.id !== toolId || !(trace === 'left' ? s > span.sStart && s <= span.sEnd : s >= span.sStart && s < span.sEnd))
            throw new RangeError('Reservoir must cover the requested physical material and one-sided label without extrapolation');
        return span;
    }
    function sample(toolId, s, { trace } = {}) {
        identifier(toolId); finite(s, 'Query material label');
        if (trace !== undefined && trace !== 'left' && trace !== 'right') throw new RangeError('History trace must be explicitly left or right');
        if (trace !== undefined) {
            const span = resolve(toolId, s, trace);
            if (!span) throw missing(toolId, s, trace);
            return velocityAt(span, s);
        }
        const left = resolve(toolId, s, 'left'), right = resolve(toolId, s, 'right');
        if (!left && !right) throw missing(toolId, s, 'unspecified');
        const value = velocityAt(left ?? right, s);
        if (left && right && !value.every((v, k) => v === velocityAt(right, s)[k])) {
            const error = new RangeError('Discontinuous material history requires an explicit left or right trace');
            error.code = 'ambiguous-material-history-trace'; throw error;
        }
        return value;
    }
    /** Split QUADRATURE domains only. Geometry and topology remain untouched.
     * Each piece contains exact old endpoint velocities and Gauss2 samples,
     * using fractions of the ORIGINAL current edge and physical label weights.
     * Across an old boundary, no single oldMaterialVelocities pair is returned:
     * current inertia must consume pieces or reject until supported. Quintic
     * pieces retain six controls and Gauss-6; they never expose a misleading
     * oldMaterialVelocities endpoint pair for the affine-only operator.
     * dsDt is retained for the caller, but NEVER applied to s a second time.
     */
    function prepare({ coordinates, inertiaEdges, maxPieces = 10000 }) {
        if (!coordinates || coordinates.length < 2 || !Array.isArray(inertiaEdges) || inertiaEdges.length !== coordinates.length - 1)
            throw new RangeError('Current coordinates and matching material-map edges are required');
        const x = Array.from(coordinates, v => finite(v, 'Current coordinate'));
        if (x.some((v, i) => i && !(v > x[i - 1]))) throw new RangeError('Current coordinates must be strictly increasing');
        if (!Number.isInteger(maxPieces) || maxPieces < 1) throw new RangeError('A positive explicit quadrature-piece budget is required');
        const requiredCuts = [], result = []; let count = 0;
        for (let edge = 0; edge < inertiaEdges.length; edge++) {
            const dx = positive(x[edge + 1] - x[edge], 'Current edge length'), ids = new Set(), records = inertiaEdges[edge]?.tools;
            if (!Array.isArray(records) || !records.length) throw new RangeError('Each current edge needs its explicit active materials');
            const tools = records.map(record => {
                const id = identifier(record.id), map = record.materialMap;
                if (ids.has(id)) throw new RangeError('Duplicate material map on a current edge'); ids.add(id);
                const sStart = finite(map?.sStart, 'Current start label'), dsDx = positive(map?.dsDx, 'Current dsDx');
                const dsDt = typeof map.dsDt === 'number' ? finite(map.dsDt, 'Current dsDt') : (() => {
                    if (map.dsDt?.length !== 2) throw new RangeError('Current dsDt must be a scalar or two endpoint rates');
                    return Array.from(map.dsDt, v => finite(v, 'Current dsDt'));
                })();
                const sEnd = finite(sStart + dsDx * dx, 'Current end label');
                if (!(sEnd > sStart)) throw new RangeError('Current material span cannot collapse through floating-point label loss');
                const pieces = [], cuts = []; let cursor = sStart;
                while (cursor < sEnd) {
                    if (++count > maxPieces) { const error = new RangeError('Material history quadrature-piece budget exhausted'); error.code = 'history-piece-budget'; throw error; }
                    const span = resolve(id, cursor, 'right');
                    if (!span) throw missing(id, cursor, 'right');
                    let end = Math.min(sEnd, span.sEnd);
                    // An external reservoir may extend into known history.
                    // Its ownership ends at the first accepted start label.
                    if (span.source === 'reservoir') for (const known of byTool.get(id) ?? [])
                        if (known.sStart > cursor) { end = Math.min(end, known.sStart); break; }
                    if (!(end > cursor)) throw new RangeError('Material history piece made no forward progress');
                    const a = (cursor - sStart) / (sEnd - sStart), b = (end - sStart) / (sEnd - sStart);
                    const polynomial=span.bernsteinVelocities!==undefined,
                        field=polynomial?{bernsteinVelocities:restrictControls(span,cursor,end),interpretation:'quintic-bernstein-material-velocity'}
                            :{oldMaterialVelocities:[velocityAt(span,cursor),velocityAt(span,end)]};
                    const samples = (polynomial?GAUSS6:GAUSS).map(([t,weight]) => {
                        const fraction = (1 - t) * a + t * b, s = (1 - t) * cursor + t * end;
                        return { fraction, s, coordinate: x[edge] + dx * fraction, coordinateWeight: dx * (b - a) * weight,
                            materialWeight: (end - cursor) * weight, oldMaterialVelocity: velocityAt(span, s) };
                    });
                    pieces.push({ sStart: cursor, sEnd: end, fractions: [a, b], coordinates: [x[edge] + dx * a, x[edge] + dx * b],
                        ...field, samples, source: span.source, sourceEdge: span.edge,
                        sourceInterval: [span.sStart, span.sEnd], startTrace: 'right', endTrace: 'left' });
                    cursor = end;
                    if (cursor < sEnd) {
                        const cut = { edge, toolId: id, s: cursor, x: x[edge] + dx * ((cursor - sStart) / (sEnd - sStart)), reason: polynomial?'own-polynomial-history-boundary':'own-affine-history-boundary' };
                        cuts.push(cut); requiredCuts.push({ ...cut });
                    }
                }
                return { id, materialMap: { sStart, dsDx, dsDt }, sEnd, pieces, requiredCuts: cuts,
                    ...(pieces.length === 1 && pieces[0].oldMaterialVelocities ? { oldMaterialVelocities: pieces[0].oldMaterialVelocities.map(v => v.slice()) } : {}),
                    ...(pieces.some(p=>p.bernsteinVelocities)?{requiresContinuousInertia:true}:{}),
                    compatibleWithAffineEdgeOperator: pieces.length === 1 && !!pieces[0].oldMaterialVelocities, angularVelocity: null, materialSpin: null, frameSpin: null };
            });
            result.push({ edge, tools });
        }
        return { inertiaEdges: result, requiredCuts, requiresSubdivision: requiredCuts.length !== 0, pieceCount: count,
            scope: 'own-accepted-translational-material-history-quadrature', includesAngularHistory: false };
    }
    return Object.freeze({ sample, prepare, toolIds: Object.freeze([...byTool.keys()]),
        scope: 'own-accepted-translational-material-history', includesAngularHistory: false });
}
