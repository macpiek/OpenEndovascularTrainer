/** A common centerline hinge with INDEPENDENT material spins per tool.
 * There are nine local position coordinates and two edge spins per tool.
 * Globally this is 3N + T(N-1), rather than T*(6N-3) position/frame DOFs.
 * This element applies only where a common-axis model is admitted. It does
 * not discard clearance, certify that reduction, or solve contact by itself.
 *
 * Reduced centerline/spin kinematics and parallel transport: Bergou et al.,
 * Discrete Elastic Rods, https://www.cs.columbia.edu/cg/pdfs/143-rods.pdf.
 * Curvature components here are DARBOUX components in the material frame,
 * matching kirchhoffMaterialProfile, rather than curvature-vector components.
 * Time-reference frames stay frozen during a nonlinear step. Their spatial
 * transport angle is differentiated, including the force induced by twist.
 */

const dot = (a, b) => a[0]*b[0] + a[1]*b[1] + a[2]*b[2];
const cross = (a, b) => [a[1]*b[2]-a[2]*b[1], a[2]*b[0]-a[0]*b[2], a[0]*b[1]-a[1]*b[0]];
function unit(v) {
    const length = Math.hypot(...v);
    if (!(length > 0) || !Number.isFinite(length)) throw new RangeError('A finite nonzero direction is required');
    return v.map(x => x / length);
}
function finiteVector(v, size, name) {
    if (!v || v.length !== size || !Array.from(v).every(Number.isFinite)) throw new TypeError(`${name} must contain ${size} finite values`);
    return Array.from(v);
}
function transport(v, a, b) {
    const cosine = dot(a, b), axis = cross(a, b);
    if (!(1 + cosine > 1e-10)) throw new RangeError('Antiparallel tangents require mesh refinement or a different reference frame');
    const first = cross(axis, v), second = cross(axis, first);
    return v.map((x, i) => x + first[i] + second[i] / (1 + cosine));
}

/** Called once when creating/rebasing the chain, not during differentiation. */
export function captureCompositeReferenceFrames(positions, initialDirector = [0, 1, 0]) {
    if (!positions || positions.length < 2) throw new RangeError('At least two centerline points are required');
    const points = positions.map(p => finiteVector(p, 3, 'position'));
    const result = [];
    for (let i = 0; i + 1 < points.length; i++) {
        const tangent = unit(points[i + 1].map((x, j) => x - points[i][j]));
        const old = result.at(-1);
        const trial = old ? transport(old.director, old.tangent, tangent) : finiteVector(initialDirector, 3, 'initialDirector');
        const axial = dot(trial, tangent);
        const director = unit(trial.map((x, j) => x - axial * tangent[j]));
        result.push({ tangent, director });
    }
    return result;
}

/** Advance each reference frame by TIME parallel transport after an accepted
 * step. This preserves material directors with unchanged edge angles. Do not
 * replace it by a fresh spatial Bishop capture, which erases frame holonomy.
 * Returns new frames; rejected trial evaluations leave accepted history alone.
 */
export function transportCompositeReferenceFrames(reference,positions) {
    if(!reference||reference.length!==positions.length-1) throw new RangeError('Reference frames must match the edges');
    return reference.map((frame,i)=>{
        const a=finiteVector(positions[i],3,'position'),b=finiteVector(positions[i+1],3,'position');
        const from=finiteVector(frame.tangent,3,'reference tangent'),d1=finiteVector(frame.director,3,'reference director');
        if(Math.abs(dot(from,from)-1)>1e-10||Math.abs(dot(d1,d1)-1)>1e-10||Math.abs(dot(from,d1))>1e-10)
            throw new RangeError('Reference frame must be orthonormal');
        const tangent=unit(b.map((v,j)=>v-a[j]));
        const trial=transport(d1,from,tangent),axial=dot(trial,tangent);
        return {tangent,director:unit(trial.map((v,j)=>v-axial*tangent[j]))};
    });
}

/** Compile constitutive samples outside the assembly loop. Units are EI/GJ,
 * inverse material length, and their derivatives, not XPBD compliance. */
export function compileCompositeMaterial(sample) {
    const energyOffset = sample.energyOffset ?? 0;
    if (!Number.isFinite(energyOffset)) throw new TypeError('energyOffset must be finite');
    const stiffness = sample.stiffness ?? [
        [sample.EI?.[0]?.[0] ?? sample.EI1, sample.EI?.[0]?.[1] ?? 0, 0],
        [sample.EI?.[1]?.[0] ?? 0, sample.EI?.[1]?.[1] ?? sample.EI2 ?? sample.EI1, 0],
        [0, 0, sample.GJ]
    ];
    const k = stiffness.flatMap(row => finiteVector(row, 3, 'stiffness row'));
    if (k.length !== 9) throw new TypeError('stiffness must be 3 by 3');
    const l = new Float64Array(9);
    for (let i = 0; i < 3; i++) for (let j = 0; j <= i; j++) {
        if (k[i*3+j] !== k[j*3+i]) throw new RangeError('stiffness must be symmetric');
        let value = k[i*3+j];
        for (let h = 0; h < j; h++) value -= l[i*3+h] * l[j*3+h];
        if (i === j && !(value > 0)) throw new RangeError('stiffness must be positive definite');
        l[i*3+j] = i === j ? Math.sqrt(value) : value / l[j*3+j];
    }
    return Object.freeze({ energyOffset, stiffness: Float64Array.from(k), intrinsic: Float64Array.from(finiteVector(
        sample.intrinsic ?? [...(sample.kappa0 ?? [sample.kappa01 ?? 0, sample.kappa02 ?? 0]), sample.tau0 ?? 0],
        3, 'intrinsic')) });
}

/** Forward derivatives live in a reused packed arena. Each operation returns
 * an offset, avoiding a derivative-array allocation for every scalar op. */
function jetArena(dimension) {
    const stride = dimension + 1, data = new Float64Array(1024 * stride);
    let cursor = 0;
    const alloc = () => {
        const at = cursor; cursor += stride;
        if (cursor > data.length) throw new RangeError('Composite element derivative arena exceeded');
        data.fill(0, at, at + stride); return at;
    };
    const constant = value => { const a = alloc(); data[a] = value; return a; };
    const unary = (a, value, derivative) => {
        const r = alloc(); data[r] = value;
        for (let i = 1; i < stride; i++) data[r+i] = derivative * data[a+i];
        return r;
    };
    const binary = (a, b, value, da, db) => {
        const r = alloc(); data[r] = value;
        for (let i = 1; i < stride; i++) data[r+i] = da * data[a+i] + db * data[b+i];
        return r;
    };
    return { data, dimension, reset() { cursor = 0; }, used() { return cursor; }, constant,
        variable(value, index) { const a = constant(value); data[a+1+index] = 1; return a; },
        add: (a,b) => binary(a,b,data[a]+data[b],1,1),
        sub: (a,b) => binary(a,b,data[a]-data[b],1,-1),
        mul: (a,b) => binary(a,b,data[a]*data[b],data[b],data[a]),
        div: (a,b) => binary(a,b,data[a]/data[b],1/data[b],-data[a]/(data[b]*data[b])),
        scale: (a,s) => unary(a,data[a]*s,s),
        sqrt: a => unary(a,Math.sqrt(data[a]),.5/Math.sqrt(data[a])),
        sin: a => unary(a,Math.sin(data[a]),Math.cos(data[a])),
        cos: a => unary(a,Math.cos(data[a]),-Math.sin(data[a])),
        atan2(y,x) { const d = data[x]*data[x] + data[y]*data[y]; return binary(y,x,Math.atan2(data[y],data[x]),data[x]/d,-data[y]/d); }
    };
}

export function createCompositeElementWorkspace(toolCount = 2) {
    if (!Number.isInteger(toolCount) || toolCount < 1 || toolCount > 2) throw new RangeError('An element has one or two tools');
    const dofCount = 9 + 2 * toolCount;
    return { toolCount, dofCount, arena: jetArena(dofCount),
        gradient: new Float64Array(dofCount), hessian: new Float64Array(dofCount*dofCount),
        strain: new Float64Array(3*toolCount), jacobian: new Float64Array(3*toolCount*dofCount),
        moments: new Float64Array(3*toolCount), toolEnergy: new Float64Array(toolCount),
        referenceTwists: new Float64Array(toolCount), energy: 0 };
}

/** Evaluate a local common-centerline element. All outputs are borrowed until
 * the next call using workspace. hessian is the PSD Gauss-Newton tangent;
 * gradient is the EXACT first derivative of the stated nonlinear energy.
 * referenceLength is the centerline material Voronoi length; dsDx converts
 * it and all three strain rates into each independent tool's material units.
 * No parameter or history is changed by evaluation.
 * referenceTwist (or tool.referenceTwist) anchors the continuous reference
 * transport angle from the accepted state. Zero initializes a Bishop frame;
 * callers must retain winding history when frames are transported/rebased.
 */
export function evaluateCompositeElement({ positions, reference, referenceLength, tools, referenceTwist: twistAnchor = 0 },
    workspace = createCompositeElementWorkspace(tools.length)) {
    if (!Number.isFinite(referenceLength) || referenceLength <= 0) throw new RangeError('referenceLength must be positive');
    if (positions.length !== 3 || reference.length !== 2 || tools.length !== workspace.toolCount)
        throw new RangeError('A hinge needs three points, two reference frames, and the workspace tool count');
    const a = workspace.arena, n = workspace.dofCount;
    a.reset(); workspace.gradient.fill(0); workspace.hessian.fill(0); workspace.energy = 0;
    const add = (x,y) => x.map((v,i) => a.add(v,y[i]));
    const sub = (x,y) => x.map((v,i) => a.sub(v,y[i]));
    const times = (x,s) => x.map(v => a.mul(v,s));
    const dotJ = (x,y) => a.add(a.add(a.mul(x[0],y[0]),a.mul(x[1],y[1])),a.mul(x[2],y[2]));
    const crossJ = (x,y) => [a.sub(a.mul(x[1],y[2]),a.mul(x[2],y[1])),
        a.sub(a.mul(x[2],y[0]),a.mul(x[0],y[2])),a.sub(a.mul(x[0],y[1]),a.mul(x[1],y[0]))];
    const unitJ = x => {
        const length = a.sqrt(dotJ(x,x));
        if (!(a.data[length] > 0)) throw new RangeError('A centerline edge cannot collapse');
        return x.map(v => a.div(v,length));
    };
    const one = a.constant(1);
    const transportJ = (v,from,to) => {
        const axis = crossJ(from,to), denominator = a.add(one,dotJ(from,to));
        if (!(a.data[denominator] > 1e-10)) throw new RangeError('Antiparallel tangents require refinement/rebasing');
        const first = crossJ(axis,v), second = crossJ(axis,first).map(x => a.div(x,denominator));
        return add(add(v,first),second);
    };
    const p = positions.map((point,i) => finiteVector(point,3,'position').map((v,j) => a.variable(v,3*i+j)));
    const t = [unitJ(sub(p[1],p[0])),unitJ(sub(p[2],p[1]))];
    const director = reference.map((frame,i) => {
        const from = finiteVector(frame.tangent,3,'reference tangent'), d1 = finiteVector(frame.director,3,'reference director');
        if (Math.abs(dot(from,from)-1)>1e-10 || Math.abs(dot(d1,d1)-1)>1e-10 || Math.abs(dot(from,d1))>1e-10)
            throw new RangeError('Reference frame must be orthonormal');
        return transportJ(d1.map(a.constant),from.map(a.constant),t[i]);
    });
    const perpendicular = t.map((v,i) => crossJ(v,director[i]));
    const denominator = a.add(one,dotJ(t[0],t[1]));
    if (!(a.data[denominator] > 1e-10)) throw new RangeError('A reversed hinge requires refinement');
    const kb = crossJ(t[0],t[1]).map(x => a.div(a.scale(x,2),denominator));
    const transported = transportJ(director[0],t[0],t[1]);
    const rawReferenceTwist = a.atan2(dotJ(t[1],crossJ(transported,director[1])),dotJ(transported,director[1]));
    for (let toolIndex = 0; toolIndex < tools.length; toolIndex++) {
        const tool = tools[toolIndex], dsDx = tool.dsDx ?? 1, material = tool.material;
        const anchor = tool.referenceTwist ?? twistAnchor;
        if (!Number.isFinite(anchor)) throw new TypeError('referenceTwist anchor must be finite');
        const winding = 2 * Math.PI * Math.round((anchor - a.data[rawReferenceTwist]) / (2 * Math.PI));
        const referenceTwist = a.add(rawReferenceTwist, a.constant(winding));
        workspace.referenceTwists[toolIndex] = a.data[referenceTwist];
        if (!Number.isFinite(dsDx) || dsDx <= 0 || material?.stiffness?.length !== 9 || material?.intrinsic?.length !== 3)
            throw new TypeError('A positive dsDx and compiled material are required');
        const theta = finiteVector(tool.angles,2,'angles').map((v,i) => a.variable(v,9+2*toolIndex+i));
        const c = theta.map(a.cos), s = theta.map(a.sin);
        const m1 = director.map((v,i) => add(times(v,c[i]),times(perpendicular[i],s[i])));
        const m2 = director.map((v,i) => sub(times(perpendicular[i],c[i]),times(v,s[i])));
        const inverseLength = 1 / (referenceLength * dsDx);
        const rates = [a.scale(dotJ(kb,add(m1[0],m1[1])),.5*inverseLength),
            a.scale(dotJ(kb,add(m2[0],m2[1])),.5*inverseLength),
            a.scale(a.add(a.sub(theta[1],theta[0]),referenceTwist),inverseLength)];
        const error = rates.map((r,i) => a.data[r] - material.intrinsic[i]), k = material.stiffness;
        const moment = error.map((_,i) => k[i*3]*error[0] + k[i*3+1]*error[1] + k[i*3+2]*error[2]);
        const materialLength = referenceLength * dsDx;
        const energy = materialLength * (.5 * dot(error,moment) + (material.energyOffset ?? 0));
        workspace.energy += energy; workspace.toolEnergy[toolIndex] = energy;
        for (let row = 0; row < 3; row++) {
            workspace.strain[3*toolIndex+row] = a.data[rates[row]];
            workspace.moments[3*toolIndex+row] = moment[row];
            for (let i = 0; i < n; i++) {
                const ji = a.data[rates[row]+1+i];
                workspace.jacobian[(3*toolIndex+row)*n+i] = ji;
                workspace.gradient[i] += materialLength * ji * moment[row];
                for (let j = 0; j <= i; j++) {
                    let weightedJ = 0;
                    for (let component = 0; component < 3; component++) weightedJ += k[3*row+component] * a.data[rates[component]+1+j];
                    workspace.hessian[i*n+j] += materialLength * ji * weightedJ;
                }
            }
        }
    }
    for (let i = 0; i < n; i++) for (let j = 0; j < i; j++) workspace.hessian[j*n+i] = workspace.hessian[i*n+j];
    if (!Number.isFinite(workspace.energy) || [workspace.gradient, workspace.hessian,
        workspace.strain, workspace.jacobian, workspace.moments, workspace.toolEnergy,
        workspace.referenceTwists].some(values => !values.every(Number.isFinite))) throw new RangeError('Nonfinite composite response');
    return workspace;
}
