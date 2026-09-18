import {readFileSync} from 'node:fs';
import {gunzipSync} from 'node:zlib';

// Inclusive stack percentages overlap. File self percentages do not.
const read = path => {
    try { return JSON.parse(readFileSync(path)); }
    catch (error) {
        if (error.code !== 'ENOENT') throw error;
        return JSON.parse(gunzipSync(readFileSync(path + '.gz')));
    }
};
const mean = values => values.reduce((a, b) => a + b, 0) / values.length;
const percentile = (values, fraction) => [...values].sort((a, b) => a - b)[Math.ceil(values.length * fraction) - 1];
const reports = [];
for (const directory of process.argv.slice(2)) {
    const profile = read(directory + '/profile.json');
    const {samples, ...metadata} = profile;
    const phases = {};
    for (const phase of new Set(samples.map(s => s.phase))) {
        const rows = samples.filter(s => s.phase === phase);
        const values = key => rows.map(s => s[key] ?? 0);
        phases[phase] = {
            steps: rows.length,
            failures: rows.filter(s => !s.converged).length,
            meanMs: mean(values('totalMs')),
            p95Ms: percentile(values('totalMs'), .95),
            maxMs: Math.max(...values('totalMs')),
            timings: Object.fromEntries(Object.keys(rows[0].timings).map(k => [k, mean(rows.map(s => s.timings[k] ?? 0))])),
            averages: Object.fromEntries(['iterations', 'factorizations', 'geometryRestarts', 'backtracks',
                'fullAssemblies', 'residualAssemblies', 'frictionIterations', 'substepAttempts',
                'mechanicalNodes', 'mechanicalDofs'].map(k => [k, mean(values(k))])),
            maxNodes: Math.max(...values('mechanicalNodes')),
        };
    }
    const cpu = read(directory + '/catheter.cpuprofile');
    const nodes = new Map(cpu.nodes.map(n => [n.id, n])), parents = new Map();
    for (const n of cpu.nodes) for (const id of n.children ?? []) parents.set(id, n.id);
    const self = new Map(), inclusive = new Map();
    let totalUs = 0;
    for (let i = 0; i < cpu.samples.length; i++) {
        let id = cpu.samples[i];
        const time = cpu.timeDeltas[i];
        totalUs += time;
        const file = nodes.get(id).callFrame.url.split('/').at(-1) || nodes.get(id).callFrame.functionName;
        self.set(file, (self.get(file) ?? 0) + time);
        const seen = new Set();
        while (id !== undefined) {
            const frame = nodes.get(id).callFrame;
            const key = `${frame.functionName || '(anonymous)'} @ ${frame.url.split('/').at(-1)}:${frame.lineNumber + 1}`;
            if (!seen.has(key)) { inclusive.set(key, (inclusive.get(key) ?? 0) + time); seen.add(key); }
            id = parents.get(id);
        }
    }
    const summarize = map => [...map].sort((a, b) => b[1] - a[1]).map(([name, us]) => ({name, ms: us / 1000, percent: 100 * us / totalUs}));
    const terminal = read(directory + '/terminal.json');
    reports.push({metadata, phases, terminal: {
        nodes: terminal.coordinates.length,
        definitions: terminal.definitions.length,
        rowKinds: terminal.definitions.reduce((counts, row) => {
            const key = row.witness ? 'vessel-witness' : row.subtype ?? row.kind;
            counts[key] = (counts[key] ?? 0) + 1;
            return counts;
        }, {}),
        vesselWitnessesWithPositiveReaction: terminal.definitions.filter((row, i) => row.witness && terminal.multipliers[i] > 0).length,
        vesselWitnessesWithZeroReaction: terminal.definitions.filter((row, i) => row.witness && terminal.multipliers[i] === 0).length,
    }, cpu: {phase: 'catheter', sampledMs: totalUs / 1000, selfByFile: summarize(self), inclusiveByFunction: summarize(inclusive)}});
}
console.log(JSON.stringify(reports, null, 2));
