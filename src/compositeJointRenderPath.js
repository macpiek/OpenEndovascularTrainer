const paths = new WeakMap();

/** Render the published physical curve directly. The parameter follows the
 * common coordinate; it is not an independently fitted display spline.
 * Publication owns these coefficients and never feeds them back into physics.
 */
export function getCompositeJointRenderPath(view) {
    if (!view || !view.continuousCurve && !view.positions) return null;
    if (paths.has(view)) return paths.get(view);
    const curve = view.continuousCurve ?? {kind:'polyline',coordinates:view.coordinates,controls:view.positions};
    const {kind, coordinates, controls} = curve, continuous = kind === 'quintic-bernstein';
    if (!continuous && kind !== 'polyline' || !coordinates || !controls || coordinates.length < 2 ||
        controls.length !== (continuous ? (coordinates.length - 1) * 18 : coordinates.length * 3) ||
        !coordinates.every((v, i) => Number.isFinite(v) && (!i || v > coordinates[i - 1])) ||
        !controls.every(Number.isFinite)) throw new RangeError('Invalid published continuous curve');
    const scratch = new Float64Array(18), first = coordinates[0], span = coordinates.at(-1) - first;
    function sample(u, target, tangent) {
        if (!Number.isFinite(u) || u < 0 || u > 1) throw new RangeError('Render sampling leaves the physical curve');
        const x = first + u * span;
        let lo = 0, hi = coordinates.length - 1;
        while (lo + 1 < hi) {const mid = (lo + hi) >>> 1; if (coordinates[mid] <= x) lo = mid; else hi = mid;}
        const t = u === 1 ? 1 : (x - coordinates[lo]) / (coordinates[lo + 1] - coordinates[lo]), offset = (continuous ? 18 : 3) * lo;
        const physicalDegree = continuous ? 5 : 1, degree = tangent ? physicalDegree - 1 : physicalDegree;
        for (let j = 0; j <= degree; j++) for (let k = 0; k < 3; k++) {
            scratch[3 * j + k] = tangent ? physicalDegree * (controls[offset + 3 * (j + 1) + k] - controls[offset + 3 * j + k]) : controls[offset + 3 * j + k];
        }
        for (let level = degree; level > 0; level--) for (let j = 0; j < level; j++) for (let k = 0; k < 3; k++) {
            const at = 3 * j + k; scratch[at] += t * (scratch[at + 3] - scratch[at]);
        }
        target.set(scratch[0], scratch[1], scratch[2]);
        if (tangent) {
            const length = Math.hypot(scratch[0], scratch[1], scratch[2]);
            if (!(length > 0)) throw new RangeError('Published curve has a degenerate tangent');
            target.multiplyScalar(1 / length);
        }
        return target;
    }
    const path = Object.freeze({pointCount: coordinates.length, getPointAt: (u, out) => sample(u, out, false), getTangentAt: (u, out) => sample(u, out, true)});
    paths.set(view, path);
    return path;
}
