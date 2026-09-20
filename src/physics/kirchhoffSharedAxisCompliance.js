/** Optional elastic normal contact: gap + C * reaction >= 0, reaction >= 0.
 * C is a physical compliance, not a numerical regularizer. Geometric gaps
 * remain unchanged in contact rows, diagnostics, discovery and replay.
 */
export const sharedAxisEffectiveGap = row => row.compliance
    ? row.gap + row.compliance * row.multiplier : row.gap;

export function sharedAxisContactElasticEnergy(rows) {
    let energy=0;
    for(const row of rows)if(row.compliance)
        energy+=.5*row.compliance*row.multiplier*row.multiplier;
    return energy;
}
