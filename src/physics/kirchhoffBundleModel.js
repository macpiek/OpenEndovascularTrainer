/**
 * Local Kirchhoff constitutive laws. No time step, contact penalty or solver.
 * x is an oriented bundle coordinate; s_i(x) is each tool's material label.
 * Curvature/frameTwist/thetaPrime are rates per x, dsDx > 0 converts to s_i.
 * Energy is per dx. Rotations act on Darboux-vector bending components.
 */
export const DEFAULT_BUNDLE_RADIAL_CLEARANCE_MM = 0.0405;

function finite(value, name) {
    if (!Number.isFinite(value)) throw new TypeError(`${name} must be finite`);
    return value;
}

function vector(value, size, name) {
    if (!value || value.length !== size) throw new TypeError(`${name} needs ${size} components`);
    return Array.from(value, (v, i) => finite(v, `${name}[${i}]`));
}

function matrix(value, size, name, positive = true) {
    if (!value || value.length !== size) throw new TypeError(`${name} needs ${size} rows`);
    const result = Array.from(value, (row, i) => vector(row, size, `${name}[${i}]`));
    for (let i = 0; i < size; i++) {
        for (let j = 0; j < i; j++) {
            const scale = Math.max(Math.abs(result[i][j]), Math.abs(result[j][i]), 1);
            if (Math.abs(result[i][j] - result[j][i]) > 1e-12 * scale) {
                throw new RangeError(`${name} must be symmetric`);
            }
            result[i][j] = result[j][i] = (result[i][j] + result[j][i]) / 2;
        }
    }
    if (positive) {
        const l = Array.from({ length: size }, () => Array(size).fill(0));
        for (let i = 0; i < size; i++) {
            for (let j = 0; j <= i; j++) {
                let entry = result[i][j];
                for (let k = 0; k < j; k++) entry -= l[i][k] * l[j][k];
                if (i === j) {
                    if (!(entry > 0)) throw new RangeError(`${name} must be positive definite`);
                    l[i][j] = Math.sqrt(entry);
                } else l[i][j] = entry / l[j][j];
            }
        }
    }
    return result;
}

const dot = (a, b) => a.reduce((sum, v, i) => sum + v * b[i], 0);
const multiply = (a, x) => a.map(row => dot(row, x));

function constitutive(sample, derivative = false) {
    if (!sample || typeof sample !== 'object') throw new TypeError('A material sample is required');
    let stiffness;
    if (sample.stiffness) stiffness = matrix(sample.stiffness, 3, 'stiffness', !derivative);
    else {
        const ei = sample.EI
            ? matrix(sample.EI, 2, 'EI', !derivative)
            : [[finite(sample.EI1 ?? (derivative ? 0 : undefined), 'EI1'), 0],
                [0, finite(sample.EI2 ?? sample.EI1 ?? (derivative ? 0 : undefined), 'EI2')]];
        stiffness = matrix([
            [ei[0][0], ei[0][1], 0], [ei[1][0], ei[1][1], 0],
            [0, 0, finite(sample.GJ ?? (derivative ? 0 : undefined), 'GJ')]
        ], 3, 'stiffness', !derivative);
    }
    const intrinsic = sample.intrinsic
        ? vector(sample.intrinsic, 3, 'intrinsic')
        : [...vector(sample.kappa0 ?? [sample.kappa01 ?? 0, sample.kappa02 ?? 0], 2, 'kappa0'),
            finite(sample.tau0 ?? 0, 'tau0')];
    return { stiffness, intrinsic };
}

function resolveTool(tool) {
    const s = finite(tool.s ?? 0, 's');
    const dsDx = finite(tool.dsDx ?? 1, 'dsDx');
    if (!(dsDx > 0)) throw new RangeError('dsDx must be positive; reverse frames explicitly for reversed coordinates');
    const sampler = tool.material;
    const variable = typeof sampler === 'function' || typeof sampler?.sample === 'function';
    const sample = typeof sampler === 'function' ? sampler(s)
        : typeof sampler?.sample === 'function' ? sampler.sample(s) : sampler;
    const material = constitutive(sample);
    const suppliedDerivative = typeof tool.materialDerivative === 'function'
        ? tool.materialDerivative(s) : tool.materialDerivative;
    // Unknown derivatives are deliberately null, never silently zero.
    const derivative = suppliedDerivative ? constitutive(suppliedDerivative, true)
        : variable ? null : constitutive({}, true);
    return { id: tool.id, s, dsDx, ...material, derivative };
}

function materialResponse(tool, strain) {
    const error = strain.map((v, i) => v - tool.intrinsic[i]);
    const moment = multiply(tool.stiffness, error);
    const density = 0.5 * dot(error, moment);
    const dS = tool.derivative
        ? tool.dsDx * (0.5 * dot(error, multiply(tool.derivative.stiffness, error))
            - dot(moment, tool.derivative.intrinsic)) : null;
    return { energy: tool.dsDx * density, density, error, moment, dS };
}

function reducedTool(tool) {
    const theta = finite(tool.theta ?? 0, 'theta');
    return {
        ...resolveTool(tool), theta, thetaPrime: finite(tool.thetaPrime ?? 0, 'thetaPrime'),
        cos: Math.cos(theta), sin: Math.sin(theta)
    };
}

function evaluateResolved(curvature, frameTwist, tools) {
    const bendingMoment = [0, 0];
    let energy = 0;
    let dFrameTwist = 0;
    const responses = tools.map(tool => {
        const { cos: c, sin: s, dsDx: lambda } = tool;
        const localRate = [c * curvature[0] + s * curvature[1],
            -s * curvature[0] + c * curvature[1], frameTwist + tool.thetaPrime];
        const strain = localRate.map(v => v / lambda);
        const response = materialResponse(tool, strain);
        const m = response.moment;
        const commonMoment = [c * m[0] - s * m[1], s * m[0] + c * m[1], m[2]];
        energy += response.energy;
        bendingMoment[0] += commonMoment[0];
        bendingMoment[1] += commonMoment[1];
        dFrameTwist += m[2];
        return {
            id: tool.id, s: tool.s, energy: response.energy, strain,
            materialMoment: m, commonMoment,
            dTheta: m[0] * localRate[1] - m[1] * localRate[0],
            dThetaPrime: m[2], dS: response.dS,
            dDsDx: response.density - dot(m, strain)
        };
    });
    return { energy, bendingMoment, dFrameTwist, tools: responses };
}

function checkTools(tools) {
    if (!Array.isArray(tools) || tools.length === 0) throw new TypeError('tools must be nonempty');
}

/** Common-axis approximation; theta and thetaPrime remain independent per tool. */
export function evaluateBundleSection({ curvature = [0, 0], frameTwist = 0, tools }) {
    checkTools(tools);
    return evaluateResolved(vector(curvature, 2, 'curvature'), finite(frameTwist, 'frameTwist'),
        tools.map(reducedTool));
}

/**
 * Eliminate only the two common bending strains, at fixed independent twists.
 * E(k) = energyOffset + 1/2 (k-preferredCurvature)^T bendingStiffness (k-preferredCurvature).
 * energyOffset includes twist and relative-rotation-dependent mismatch energy.
 * minimum.tools contains envelope derivatives of this offset, not a welded GJ.
 */
export function condenseBundleSection({ frameTwist = 0, tools }) {
    checkTools(tools);
    finite(frameTwist, 'frameTwist');
    const resolved = tools.map(reducedTool);
    const h = [[0, 0], [0, 0]];
    for (const { stiffness: k, cos: c, sin: s, dsDx } of resolved) {
        const rows = [[c, -s, 0], [s, c, 0]];
        for (let i = 0; i < 2; i++) {
            for (let j = 0; j < 2; j++) h[i][j] += dot(rows[i], multiply(k, rows[j])) / dsDx;
        }
    }
    const linear = evaluateResolved([0, 0], frameTwist, resolved).bendingMoment;
    // Cholesky avoids determinant cancellation for strongly anisotropic EI.
    const l00 = Math.sqrt(h[0][0]);
    const l10 = h[1][0] / l00;
    const pivot = h[1][1] - l10 * l10;
    if (!(pivot > 0)) throw new RangeError('Bundle bending tensor is numerically singular');
    const l11 = Math.sqrt(pivot);
    const y0 = -linear[0] / l00;
    const y1 = (-linear[1] - l10 * y0) / l11;
    const k1 = y1 / l11;
    const preferredCurvature = [(y0 - l10 * k1) / l00, k1];
    // Sum positive individual energies, avoiding subtraction of two large energies.
    const minimum = evaluateResolved(preferredCurvature, frameTwist, resolved);
    return { bendingStiffness: h, preferredCurvature, energyOffset: minimum.energy, minimum };
}

/**
 * Exact sum for separate rods. strain is each rod's actual material-frame
 * Darboux vector per ds_i, computed from its OWN directors by the discretizer.
 * No assumption of common tangents, curvatures, material labels or twist.
 */
export function evaluateFullBundleSection({ tools }) {
    checkTools(tools);
    let energy = 0;
    const responses = tools.map(input => {
        const tool = resolveTool(input);
        const strain = vector(input.strain, 3, 'strain');
        const response = materialResponse(tool, strain);
        energy += response.energy;
        return {
            id: tool.id, s: tool.s, energy: response.energy, materialMoment: response.moment,
            dStrain: response.moment.map(v => tool.dsDx * v),
            dDsDx: response.density, dS: response.dS
        };
    });
    return { energy, tools: responses };
}

function pairWeight(weight) {
    finite(weight, 'weight');
    if (weight < 0 || weight > 1) throw new RangeError('weight must lie in [0, 1]');
    return weight;
}

/** Exact linear change of variables; all directors and material data stay separate. */
export function encodeBundlePair({ first, second, weight = 0.5 }) {
    pairWeight(weight);
    const a = vector(first.position, 3, 'first.position');
    const b = vector(second.position, 3, 'second.position');
    const { position: ignoredA, ...firstData } = first;
    const { position: ignoredB, ...secondData } = second;
    return {
        commonPosition: a.map((v, i) => (1 - weight) * v + weight * b[i]),
        relativePosition: b.map((v, i) => v - a[i]), weight,
        first: firstData, second: secondData
    };
}

export function decodeBundlePair({ commonPosition, relativePosition, first, second, weight = 0.5 }) {
    pairWeight(weight);
    const q = vector(commonPosition, 3, 'commonPosition');
    const d = vector(relativePosition, 3, 'relativePosition');
    return {
        first: { ...first, position: q.map((v, i) => v - weight * d[i]) },
        second: { ...second, position: q.map((v, i) => v + (1 - weight) * d[i]) }
    };
}

/** Transpose Jacobian; preserves virtual work of arbitrary forces on both rods. */
export function pullbackBundlePairForces({ firstForce, secondForce, weight = 0.5 }) {
    pairWeight(weight);
    const a = vector(firstForce, 3, 'firstForce');
    const b = vector(secondForce, 3, 'secondForce');
    return {
        commonForce: a.map((v, i) => v + b[i]),
        relativeForce: a.map((v, i) => -weight * v + (1 - weight) * b[i])
    };
}

/** Per Cartesian component: T = 1/2 [qDot,dDot]^T M [qDot,dDot]. */
export function bundlePairMassMatrix({ firstMass, secondMass, weight = 0.5 }) {
    pairWeight(weight);
    finite(firstMass, 'firstMass');
    finite(secondMass, 'secondMass');
    if (!(firstMass > 0) || !(secondMass > 0)) throw new RangeError('Both masses must be positive');
    const cross = -weight * firstMass + (1 - weight) * secondMass;
    return [[firstMass + secondMass, cross],
        [cross, weight * weight * firstMass + (1 - weight) ** 2 * secondMass]];
}

/**
 * Circular-lumen cross-section inequality g >= 0; never modifies the offset.
 * offset is measured in the catheter normal plane at the actual curve foot.
 * Finding that foot and checking the swept surface/end cap belong to contact geometry.
 * squaredGap has a smooth derivative even at the center; gap is in mm.
 */
export function evaluateBundleClearance({ offset, clearance = DEFAULT_BUNDLE_RADIAL_CLEARANCE_MM }) {
    const d = vector(offset, 2, 'offset');
    finite(clearance, 'clearance');
    if (clearance < 0) throw new RangeError('clearance must be nonnegative');
    const radius = Math.hypot(...d);
    return {
        clearance, gap: clearance - radius, squaredGap: clearance * clearance - dot(d, d),
        gradientSquaredGap: d.map(v => -2 * v), admissible: radius <= clearance
    };
}
