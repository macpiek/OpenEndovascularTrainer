// Replay the captured 61-row contact system without anatomy or rendering.
import fs from 'node:fs';
import { solveSeededCoulombNewton } from '../../src/physics/kirchhoffCoulombNewtonSolver.js';

const input = JSON.parse(fs.readFileSync(new URL('../../reports/friction-roundoff-reproducer-2026-09-10.json', import.meta.url)),
    (_key, value) => value === 'Infinity' ? Infinity : value === '-Infinity' ? -Infinity : value);
const { matrix, rhs, lower, upper, count, band, groups } = input;
const negativeDiagonals = Array.from({ length: count }, (_, row) => ({ row, diagonal: matrix[row * band] }))
    .filter(({ diagonal }) => diagonal < 0);
try {
    const result = solveSeededCoulombNewton(Float64Array.from(matrix), Float64Array.from(rhs),
        Float64Array.from(lower), Float64Array.from(upper), count, band, groups);
    console.log(JSON.stringify({ reproduced: false, negativeDiagonals, status: result.diagnostics.status }));
} catch (error) {
    if (!(error instanceof RangeError) || error.message !== 'Invalid friction data') throw error;
    console.log(JSON.stringify({ reproduced: true, negativeDiagonals, message: error.message, cause: error.cause },
        (_key, value) => typeof value === 'number' && !Number.isFinite(value) ? String(value) : value, 2));
}
