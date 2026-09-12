import fs from 'node:fs';
import { createHash } from 'node:crypto';
import { fileURLToPath } from 'node:url';
import { resolve } from 'node:path';
import { gunzipSync } from 'node:zlib';
import { measureCoupledLoadKKT } from '../../src/physics/kirchhoffCoupledLoadSolver.js';

/** Verify the independent-coordinate export against the full original dual
 * operator and its final-load KKT. Does not run or time a physics step. */
export function auditFrozenCoupledSystem(data) {
    const { count, band, matrix, increment, columns, weights, rows } = data;
    if (!Number.isInteger(count) || count < 1 || !Number.isInteger(band) || band < 1 ||
        matrix.length !== count * band || increment.length !== count || rows.length !== count ||
        data.rhs.length !== count || data.lower.length !== count || data.upper.length !== count)
        throw new RangeError('Invalid frozen system dimensions');
    const gram = new Float64Array(count * band);
    for (let row = 0; row < count; row++) gram[row * band] = rows[row].alpha;
    for (let side = 0; side < columns.length; side++) {
        if (weights[side].length !== columns[side].length) throw new RangeError('J/W dimension mismatch');
        for (let dof = 0; dof < columns[side].length; dof++) {
            const entries = columns[side][dof], weight = weights[side][dof];
            if (entries.length % 2) throw new RangeError('Incomplete sparse column');
            for (let i = 0; i < entries.length; i += 2) {
                const row = entries[i];
                if (!Number.isInteger(row) || row < 0 || row >= count || i && row <= entries[i - 2])
                    throw new RangeError('Sparse column must contain sorted unique row indices');
                for (let j = 0; j <= i; j += 2) {
                    const distance = row - entries[j];
                    if (distance >= band) throw new RangeError('Exported band omits a J/W coupling');
                    gram[row * band + distance] += weight * entries[i + 1] * entries[j + 1];
                }
            }
        }
    }
    let maximumGramError = 0, maximumRelativeGramError = 0, nonzeroLowerEntries = 0;
    for (let i = 0; i < matrix.length; i++) {
        const error = Math.abs(matrix[i] - gram[i]);
        maximumGramError = Math.max(maximumGramError, error);
        maximumRelativeGramError = Math.max(maximumRelativeGramError,
            error / Math.max(1, Math.abs(matrix[i]), Math.abs(gram[i])));
        nonzeroLowerEntries += Number(matrix[i] !== 0);
    }
    const residual = Float64Array.from(data.rhs);
    for (let row = 0; row < count; row++) for (let col = Math.max(0, row - band + 1); col <= row; col++) {
        const a = matrix[row * band + row - col];
        residual[row] -= a * increment[col];
        if (col !== row) residual[col] -= a * increment[row];
    }
    const groups = data.groups.map(group => ({ ...group,
        radii: group.normalRow == null ? group.radii : group.mu.map(mu =>
            mu * Math.max(0, group.normalLambda + increment[group.normalRow]))
    }));
    const kkt = measureCoupledLoadKKT(residual, increment, data.lower, data.upper, groups);
    const allFinite = [matrix, increment, residual, gram].every(array => array.every(Number.isFinite));
    const boundsValid = increment.every((value, row) => value >= data.lower[row] && value <= data.upper[row]);
    const tolerance = data.options.tolerance;
    if (!(Number.isFinite(tolerance) && tolerance > 0)) throw new RangeError('Missing original solve tolerance');
    return {
        scope: 'Frozen original equations only; no runtime or FPS measurement',
        count, band, nonzeroLowerEntries, allocatedEntries: matrix.length,
        denseLowerEntries: count * (count + 1) / 2,
        maximumGramError, maximumRelativeGramError, allFinite, boundsValid,
        maximumResidual: kkt.maximumResidual, frictionResidual: kkt.groupResidual,
        coneViolation: kkt.coneViolation, tolerance,
        passed: allFinite && boundsValid && maximumRelativeGramError <= 1e-11 &&
            kkt.maximumResidual <= tolerance && kkt.coneViolation <= 1e-9
    };
}

if (process.argv[1] && resolve(process.argv[1]) === fileURLToPath(import.meta.url)) {
    const [input, output] = process.argv.slice(2);
    if (!input) throw new Error('Use: node scripts/physics/audit-frozen-coupled-system.mjs INPUT.json [OUTPUT.json]');
    const stored = fs.readFileSync(input);
    const bytes = input.endsWith('.gz') ? gunzipSync(stored) : stored;
    const data = JSON.parse(bytes, (_key, value) => value === 'Infinity' ? Infinity : value === '-Infinity' ? -Infinity : value);
    const report = { input: resolve(input), sha256: createHash('sha256').update(bytes).digest('hex'),
        ...auditFrozenCoupledSystem(data) };
    if (output) fs.writeFileSync(output, JSON.stringify(report, null, 2) + '\n');
    console.log(JSON.stringify(report, null, 2));
    if (!report.passed) process.exitCode = 1;
}
