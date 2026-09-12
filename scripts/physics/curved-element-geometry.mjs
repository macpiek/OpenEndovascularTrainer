// Offline geometry screens only: no contact, force, twist or mesh certificate.
const sub = (a, b) => a.map((v, i) => v - b[i]);
const scale = (a, s) => a.map(v => v * s);
const lerp = (a, b, t) => a.map((v, i) => v * (1 - t) + b[i] * t);
const dot = (a, b) => a.reduce((sum, v, i) => sum + v * b[i], 0);

export function evaluateHermiteElement(a, b, da, db, length, t, derivative = false) {
    const t2 = t * t, t3 = t2 * t;
    const h = derivative ? [6 * t2 - 6 * t, 3 * t2 - 4 * t + 1, -6 * t2 + 6 * t, 3 * t2 - 2 * t]
        : [2 * t3 - 3 * t2 + 1, t3 - 2 * t2 + t, -2 * t3 + 3 * t2, t3 - t2];
    return a.map((v, axis) => (h[0] * v + h[1] * length * da[axis]
        + h[2] * b[axis] + h[3] * length * db[axis]) / (derivative ? length : 1));
}

function split(coefficients, t) {
    const work = coefficients.map(v => v.slice());
    const left = [work[0]], right = [work.at(-1)];
    for (let n = work.length - 1; n > 0; n--) {
        for (let i = 0; i < n; i++) work[i] = lerp(work[i], work[i + 1], t);
        left.push(work[0]); right.push(work[n - 1]);
    }
    return [left, right.reverse()];
}

function choose(n, k) {
    let result = 1;
    for (let i = 1; i <= k; i++) result *= (n + 1 - i) / i;
    return result;
}

// Bernstein coefficients of |p|^2, where p is a degree-n vector polynomial.
function squaredNorm(coefficients) {
    const n = coefficients.length - 1, out = Array(2 * n + 1).fill(0);
    for (let i = 0; i <= n; i++) for (let j = 0; j <= n; j++) {
        out[i + j] += choose(n, i) * choose(n, j) / choose(2 * n, i + j)
            * dot(coefficients[i], coefficients[j]);
    }
    return out;
}

function boundMaximum(coefficients, stretch, accuracy) {
    const f = v => stretch ? Math.abs(Math.sqrt(Math.max(0, v)) - 1) : Math.sqrt(Math.max(0, v));
    const pending = [{ c: coefficients.map(v => [v]), depth: 0 }];
    let lower = 0, upper = 0, subdivisions = 0, depthLimited = false;
    while (pending.length) {
        const { c, depth } = pending.pop();
        const [left, right] = split(c, .5);
        lower = Math.max(lower, f(c[0][0]), f(c.at(-1)[0]), f(left.at(-1)[0]));
        const values = c.map(v => v[0]);
        const candidate = Math.max(f(Math.min(...values)), f(Math.max(...values)));
        if (candidate <= lower + accuracy || depth >= 24) {
            upper = Math.max(upper, candidate);
            if (depth >= 24 && candidate > lower + accuracy) depthLimited = true;
        } else {
            pending.push({ c: left, depth: depth + 1 }, { c: right, depth: depth + 1 });
            subdivisions++;
        }
    }
    // The convex-hull bound is continuous in exact arithmetic. A deliberately
    // conservative noise guard is NOT a directed-rounding interval proof.
    const roundoffGuard = Math.sqrt(128 * Number.EPSILON * Math.max(1, ...coefficients.map(Math.abs)) * 25);
    return { lower: Math.max(0, lower - roundoffGuard), upper: Math.max(lower, upper) + roundoffGuard,
        roundoffGuard, subdivisions, depthLimited };
}

export function boundHermiteGeometry({ a, middle, b, da, db, length, middleFraction, accuracy = 1e-8 }) {
    if (![length, middleFraction, accuracy].every(Number.isFinite)
        || !(length > 0 && middleFraction > 0 && middleFraction < 1 && accuracy > 0)) {
        throw new RangeError('Positive length/accuracy and an interior material fraction are required');
    }
    for (const v of [a, middle, b, da, db]) {
        if (v.length !== 3 || !v.every(Number.isFinite)) throw new TypeError('Finite 3D vectors required');
    }
    const delta = sub(b, a), mid = sub(middle, a);
    const controls = [[0, 0, 0], scale(da, length / 3), sub(delta, scale(db, length / 3)), delta];
    const pieces = split(controls, middleFraction);
    const endpoints = [[[0, 0, 0], mid], [mid, delta]];
    const pieceBounds = pieces.map((c, edge) => {
        const difference = c.map((v, i) => sub(v, lerp(...endpoints[edge], i / 3)));
        return boundMaximum(squaredNorm(difference), false, accuracy);
    });
    const derivative = controls.slice(1).map((v, i) => scale(sub(v, controls[i]), 3 / length));
    return {
        position: { lower: Math.max(...pieceBounds.map(b => b.lower)), upper: Math.max(...pieceBounds.map(b => b.upper)),
            subdivisions: pieceBounds.reduce((n, b) => n + b.subdivisions, 0),
            depthLimited: pieceBounds.some(b => b.depthLimited), pieceBounds },
        stretch: boundMaximum(squaredNorm(derivative), true, accuracy)
    };
}
