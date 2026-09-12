import { createLineSearchStats } from './kirchhoffLineSearch.js';
const COST_KEYS = ['assemblyMs', 'solveMs', 'applyMs', 'snapshotMs', 'restoreMs', 'measureMs',
    'condensedSetupMs', 'schurMs', 'contactSolveMs', 'reconstructionMs', 'seedMs',
    'solveCalls', 'applyCalls', 'measureCalls', 'fullMeasureCalls', 'snapshots', 'restores'];
const PHASE_KEYS = ['total', 'constraints', 'narrowPhase', 'integrate', 'velocity'];
const KEYS = [...COST_KEYS, ...PHASE_KEYS, 'trials', 'backtracks', 'factorizations'];

// Bounded, allocation-free recording. Percentiles cover the most recent
// capacity steps; means and maxima cover every recorded benchmark step.
export class ConstraintStageProfile {
    constructor(capacity = 8192) {
        this.capacity = capacity;
        this.fields = Object.fromEntries(KEYS.map(key => [key,
            { samples: new Float64Array(capacity), sum: 0, maximum: 0 }]));
        this.reset();
    }
    reset() {
        this.lineSearch = createLineSearchStats();
        this.steps = 0;
        this.failedSteps = 0;
        for (const field of Object.values(this.fields)) {
            field.sum = 0; field.maximum = 0;
        }
    }
    add(key, value) {
        const field = this.fields[key];
        field.samples[this.steps % this.capacity] = value;
        field.sum += value;
        field.maximum = Math.max(field.maximum, value);
    }
    record(world) {
        const search = world.lastJointLineSearch;
        if (search) {
            for (const key of ['accepted', 'rejected']) for (let i = 0; i < 8; i++)
                this.lineSearch[key][i] += search[key][i];
            for (const key of ['boundaryWorstChanged', 'predictedStarts', 'largerFallbacks', 'earlyRejections'])
                this.lineSearch[key] += search[key] ?? 0;
            for (const group of ['rejectionTerms', 'boundaryKinds']) for (const key in search[group])
                this.lineSearch[group][key] = (this.lineSearch[group][key] ?? 0) + search[group][key];
        }
        for (const key of COST_KEYS) this.add(key, world.lastJointCosts?.[key] ?? 0);
        for (const key of PHASE_KEYS) this.add(key, world.timings[key].last);
        this.add('trials', world.lastJointTrialEvaluations ?? 0);
        this.add('backtracks', world.lastJointBacktracks ?? 0);
        this.add('factorizations', world.lastJointFactorizations ?? 0);
        this.failedSteps += Number(Boolean(world.lastJointNonlinearFailure));
        this.steps++;
    }
    report() {
        const count = Math.min(this.steps, this.capacity);
        return { lineSearch: structuredClone(this.lineSearch), steps: this.steps, failedSteps: this.failedSteps, percentileSteps: count,
            fields: Object.fromEntries(KEYS.map(key => {
                const field = this.fields[key];
                const sorted = field.samples.slice(0, count).sort();
                return [key, { mean: this.steps ? field.sum / this.steps : 0,
                    p95: count ? sorted[Math.floor((count - 1) * .95)] : 0,
                    maximum: field.maximum, sum: field.sum }];
            })) };
    }
}
