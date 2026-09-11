/** Positive numerical scales for a Gram system. A Schur complement may lose
 * a null diagonal to cancellation. Only the preconditioner uses the reference
 * scale 1; the original matrix, force bounds and KKT equations are untouched.
 */
export function fillKirchhoffGramMobilities(matrix, count, band, tolerances, out) {
    let reference = 0;
    for (let i = 0; i < count; i++) {
        const diagonal = matrix[i * band];
        if (!Number.isFinite(diagonal)) throw new RangeError('Non-finite Gram diagonal');
        reference = Math.max(reference, Math.abs(diagonal));
    }
    const fallbackTolerance = 32 * Number.EPSILON * Math.max(1, count) * reference;
    for (let i = 0; i < count; i++) {
        const diagonal = matrix[i * band];
        const bound = tolerances?.[i] ?? (diagonal < 0 ? fallbackTolerance : 0);
        if (!Number.isFinite(bound) || bound < 0) throw new RangeError('Invalid Gram roundoff bound');
        if (diagonal < -bound) throw new RangeError('Negative Gram diagonal exceeds roundoff bound');
        out[i] = diagonal > bound ? diagonal : 1;
    }
    return out;
}
