const edges = new WeakMap();
const finite = (x, name) => { if (!Number.isFinite(x)) throw new RangeError(`${name} must be finite`); return x; };
const vector = (v, n, name) => {
    if (v?.length !== n) throw new RangeError(`${name} needs ${n} entries`);
    return Array.from(v, x => finite(x, name));
};
const freeze = x => { if (x && typeof x === 'object') { Object.values(x).forEach(freeze); Object.freeze(x); } return x; };
const cross = (a, b) => [a[1]*b[2]-a[2]*b[1], a[2]*b[0]-a[0]*b[2], a[0]*b[1]-a[1]*b[0]];

function derivativeStencil(x, node, start, end) {
    const nodes = end-start === 1 ? [start, end]
        : node === start ? [start, start+1, start+2]
        : node === end ? [end-2, end-1, end] : [node-1, node, node+1];
    if (nodes.length === 2) {
        const h = x[end]-x[start];
        return { nodes, first: [-1/h, 1/h], second: [0, 0] };
    }
    // Translate and scale before forming Lagrange derivatives. In particular,
    // a large material-coordinate origin must not enter a squared denominator.
    const h = x[nodes[2]]-x[nodes[0]], at = (x[node]-x[nodes[0]])/h,
        u = nodes.map(j => (x[j]-x[nodes[0]])/h);
    return { nodes,
        first: nodes.map((_, i) => { const others = [0,1,2].filter(j => j !== i);
            return (2*at-u[others[0]]-u[others[1]])/((u[i]-u[others[0]])*(u[i]-u[others[1]]))/h; }),
        second: nodes.map((_, i) => 2/[0,1,2].filter(j => j !== i).reduce((v,j) => v*(u[i]-u[j]), 1)/h/h) };
}

function bernstein(values, fraction) {
    const row = values.slice();
    for (let degree = row.length-1; degree > 0; degree--)
        for (let i = 0; i < degree; i++) row[i] = (1-fraction)*row[i]+fraction*row[i+1];
    return row[0];
}

/** A common C2 position field in each explicitly smooth physical interval.
 * Shared nodal first/second derivatives are fixed linear functions of existing
 * position nodes. Every physical interface retains its own one-sided stencils.
 * This adds no mechanical DOFs and does not silently smooth a declared interface.
 * It defines geometry and material TRANSLATION; material directors/spins and
 * curved narrow-phase contact must use their own consistent reconstruction.
 */
export function createCompositeContinuousGeometry({ coordinates, interfaces = [] } = {}) {
    if (!coordinates || coordinates.length < 2) throw new RangeError('At least two physical coordinates are required');
    const x = vector(coordinates, coordinates.length, 'Physical coordinates');
    if (x.some((v,i) => i && (!(v>x[i-1]) || !Number.isFinite(v-x[i-1])))) throw new RangeError('Physical coordinates must increase finitely');
    if (!Array.isArray(interfaces) || new Set(interfaces).size !== interfaces.length || interfaces.some(i => !Number.isInteger(i) || i<1 || i>=x.length-1))
        throw new RangeError('Physical interfaces must be distinct interior node indices');
    const cuts = [0, ...interfaces.slice().sort((a,b)=>a-b), x.length-1], result = [];
    for (let region = 0; region < cuts.length-1; region++) {
        const start = cuts[region], end = cuts[region+1], stencils = new Map();
        for (let node = start; node <= end; node++) stencils.set(node, derivativeStencil(x,node,start,end));
        for (let edge = start; edge < end; edge++) {
            const left = stencils.get(edge), right = stencils.get(edge+1),
                nodes = [...new Set([...left.nodes,...right.nodes])].sort((a,b)=>a-b), n = nodes.length, h = x[edge+1]-x[edge],
                controls = Array.from({length:6},()=>new Array(n).fill(0));
            const add = (row, node, value) => controls[row][nodes.indexOf(node)] += value;
            for (const row of [0,1,2]) add(row,edge,1);
            for (const row of [3,4,5]) add(row,edge+1,1);
            left.nodes.forEach((node,j) => {
                add(1,node,h*left.first[j]/5);
                add(2,node,2*h*left.first[j]/5+h*h*left.second[j]/20);
            });
            right.nodes.forEach((node,j) => {
                add(4,node,-h*right.first[j]/5);
                add(3,node,-2*h*right.first[j]/5+h*h*right.second[j]/20);
            });
            if (controls.flat().some(v=>!Number.isFinite(v))) throw new RangeError('Nonfinite continuous geometry stencil');
            const handle = freeze({ scope:'shared-nodal-C2-quintic-geometry', edge, coordinates:[x[edge],x[edge+1]], nodeIndices:nodes,
                region:[start,end], degree:5, independentDofs:3*n, contactCertified:false });
            edges.set(handle,{controls,n,h,anchor:nodes.indexOf(edge)}); result.push(handle);
        }
    }
    return freeze({ coordinates:x, interfaces:cuts.slice(1,-1), edges:result, scope:'piecewise-C2-existing-position-node-geometry', contactCertified:false });
}

/** Shape coefficients are copied, so mutating a sample cannot change a later
 * surface/inertia evaluation. Exact endpoint identities are handled by the
 * same Bernstein polynomial; no neighboring spatial slot supplies a value.
 */
export function sampleCompositeContinuousBasis(edge, fraction) {
    const plan = edges.get(edge); if (!plan) throw new TypeError('Use a compiled continuous geometry edge');
    finite(fraction,'Edge fraction'); if (fraction<0 || fraction>1) throw new RangeError('Continuous geometry extrapolation is forbidden');
    const {controls,n,h,anchor} = plan, weights = [], first = [], second = [], third = [];
    for (let j = 0; j < n; j++) {
        const b = controls.map(row=>row[j]), d = b.slice(1).map((v,i)=>5*(v-b[i])/h), dd = d.slice(1).map((v,i)=>4*(v-d[i])/h), ddd=dd.slice(1).map((v,i)=>3*(v-dd[i])/h);
        weights.push(bernstein(b,fraction)); first.push(bernstein(d,fraction)); second.push(bernstein(dd,fraction)); third.push(bernstein(ddd,fraction));
    }
    // Enforce the exact partition identities in the stored floating-point
    // representation, using the same anchor as the difference-form evaluator.
    for (const [values,total] of [[weights,1],[first,0],[second,0],[third,0]])
        values[anchor] = total-values.reduce((sum,v,j)=>j===anchor?sum:sum+v,0);
    if ([...weights,...first,...second,...third].some(v=>!Number.isFinite(v))) throw new RangeError('Nonfinite continuous basis');
    return { weights, first, second, third, anchor, fraction };
}

export function evaluateCompositeContinuousGeometry(edge, { positions, fraction, previousPositions, dt, materialMap } = {}) {
    const plan = edges.get(edge); if (!plan) throw new TypeError('Use a compiled continuous geometry edge');
    if (positions?.length !== plan.n) throw new RangeError('Positions must follow the compiled local node support');
    const p = positions.map(v=>vector(v,3,'Own position')), b = sampleCompositeContinuousBasis(edge,fraction);
    const interpolate = (values, coefficients, total) => [0,1,2].map(k=>{
        const base = values[b.anchor][k];
        return coefficients.reduce((sum,w,j)=>j===b.anchor?sum:sum+w*(values[j][k]-base), total*base);
    });
    const position = interpolate(p,b.weights,1), positionDx = interpolate(p,b.first,0), positionDxx = interpolate(p,b.second,0), length = Math.hypot(...positionDx);
    if (!(length>0) || !Number.isFinite(length)) throw new RangeError('Continuous geometry needs a nondegenerate physical tangent');
    const tangent = positionDx.map(v=>v/length), projection = tangent.reduce((sum,v,k)=>sum+v*positionDxx[k],0),
        tangentDx = positionDxx.map((v,k)=>(v-tangent[k]*projection)/length);
    const output = { position, positionDx, positionDxx, tangent, tangentDx, basis:b, contactCertified:false };
    if (previousPositions !== undefined || dt !== undefined || materialMap !== undefined) {
        if (previousPositions?.length !== plan.n || !(finite(dt,'dt')>0)) throw new RangeError('Material motion requires matching old positions and positive dt');
        const old = previousPositions.map(v=>vector(v,3,'Own previous position')), map = materialMap,
            dsDx = finite(map?.dsDx,'Material metric'), sStart = finite(map?.sStart,'Material start');
        if (!(dsDx>0)) throw new RangeError('Material metric must be positive');
        const rates = typeof map.dsDt === 'number' ? [finite(map.dsDt,'Material rate'),map.dsDt] : vector(map.dsDt,2,'Material rates'),
            feed = -((1-fraction)*rates[0]+fraction*rates[1])/dsDx,
            change = p.map((v,j)=>v.map((x,k)=>(x-old[j][k])/dt)), positionDt = interpolate(change,b.weights,1), positionDxDt = interpolate(change,b.first,0),
            materialVelocity = positionDt.map((v,k)=>v+feed*positionDx[k]), tangentRateNumerator = positionDxDt.map((v,k)=>v+feed*positionDxx[k]),
            bendingOmega = cross(positionDx,tangentRateNumerator).map(v=>v/(length*length));
        Object.assign(output,{ positionDt, positionDxDt, materialVelocity, bendingOmega, materialLabel:finite(sStart+dsDx*plan.h*fraction,'Current material label'), feed,
            angularVelocity:null, materialSpin:null, interpretation:'physical-material-velocity' });
    }
    for (const value of Object.values(output)) if (Array.isArray(value) && value.some(v=>!Number.isFinite(v))) throw new RangeError('Nonfinite continuous material geometry');
    return output;
}

/** Bernstein controls describe the SAME geometry for a later conservative
 * curved broad/narrow phase. They are not a certificate for the old capsule.
 */
export function compositeContinuousBezierControls(edge, positions) {
    const plan = edges.get(edge); if (!plan) throw new TypeError('Use a compiled continuous geometry edge');
    if (positions?.length !== plan.n) throw new RangeError('Positions must follow the compiled local node support');
    const p = positions.map(v=>vector(v,3,'Own position')), {anchor} = plan;
    return plan.controls.map(row=>[0,1,2].map(k=>finite(row.reduce((sum,w,j)=>j===anchor?sum:sum+w*(p[j][k]-p[anchor][k]),p[anchor][k]),'Bezier control')));
}

/** Capture a polynomial physical material velocity field after accepting the
 * supplied geometry. Retain all six coefficients for future label transport;
 * endpoint values alone would erase its interior momentum and kinetic energy.
 * This constructor neither accepts a solver step nor infers angular history.
 */
export function createCompositeContinuousMaterialVelocity(edge,{id,positions,previousPositions,dt,materialMap}={}) {
    if(typeof id!=='string'||!id)throw new RangeError('An own physical tool id is required');
    const plan=edges.get(edge);if(!plan)throw new TypeError('Use a compiled continuous geometry edge');
    if(positions?.length!==plan.n||previousPositions?.length!==plan.n||!(finite(dt,'dt')>0))throw new RangeError('Matching own position supports and positive dt are required');
    const p=positions.map(v=>vector(v,3,'Own position')),old=previousPositions.map(v=>vector(v,3,'Own previous position')),
        change=p.map((v,j)=>v.map((x,k)=>(x-old[j][k])/dt)),qdot=compositeContinuousBezierControls(edge,change),
        origin=p[0],controls=compositeContinuousBezierControls(edge,p.map(v=>v.map((x,k)=>x-origin[k]))),
        dsDx=finite(materialMap?.dsDx,'Material metric'),sStart=finite(materialMap?.sStart,'Material start');
    if(!(dsDx>0))throw new RangeError('Material metric must be positive');
    const rates=typeof materialMap.dsDt==='number'?[finite(materialMap.dsDt,'Material rate'),materialMap.dsDt]:vector(materialMap.dsDt,2,'Material rates'),
        u=rates.map(v=>-v/dsDx),derivative=controls.slice(1).map((v,j)=>v.map((x,k)=>5*(x-controls[j][k])/plan.h)),
        bernsteinVelocities=qdot.map((v,j)=>v.map((x,k)=>finite(x+(j<5?(5-j)/5*u[0]*derivative[j][k]:0)+(j>0?j/5*u[1]*derivative[j-1][k]:0),'Physical polynomial material velocity'))),
        sEnd=finite(sStart+dsDx*plan.h,'Material end');
    if(!(sEnd>sStart))throw new RangeError('Material labels must delimit a nonempty physical interval');
    return freeze({id,sStart,sEnd,bernsteinVelocities,interpretation:'quintic-bernstein-material-velocity',angularVelocity:null,materialSpin:null,frameSpin:null});
}
