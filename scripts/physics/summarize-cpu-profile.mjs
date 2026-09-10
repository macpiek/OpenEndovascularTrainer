import fs from 'node:fs';

const profile = JSON.parse(fs.readFileSync(process.argv[2], 'utf8'));
if (profile.samples.length !== profile.timeDeltas.length) throw new Error('Mismatched CPU sample weights');
const nodes = new Map(profile.nodes.map(node => [node.id, node])), parents = new Map();
for (const node of profile.nodes) for (const child of node.children ?? []) parents.set(child, node.id);
const functions = new Map(), files = new Map();
let totalUs = 0;
function entry(id) {
    const frame = nodes.get(id).callFrame;
    const key = JSON.stringify([frame.functionName, frame.url, frame.lineNumber]);
    if (!functions.has(key)) functions.set(key, { name: frame.functionName || '(anonymous)',
        url: frame.url, line: frame.lineNumber + 1, selfUs: 0, inclusiveUs: 0 });
    return functions.get(key);
}
for (let i = 0; i < profile.samples.length; i++) {
    let id = profile.samples[i];
    const delta = profile.timeDeltas[i];
    if (!Number.isFinite(delta) || delta < 0 || !nodes.has(id)) throw new Error('Invalid CPU sample');
    totalUs += delta;
    const leaf = entry(id);
    leaf.selfUs += delta;
    const file = leaf.url || leaf.name;
    files.set(file, (files.get(file) ?? 0) + delta);
    const seen = new Set();
    while (id !== undefined) {
        const item = entry(id);
        // Recursive occurrences of the same function share one inclusive sample.
        if (!seen.has(item)) { item.inclusiveUs += delta; seen.add(item); }
        id = parents.get(id);
    }
}
const format = item => ({ name: item.name, url: item.url, line: item.line,
    selfMs: item.selfUs / 1000, selfPercent: item.selfUs / totalUs * 100,
    inclusiveMs: item.inclusiveUs / 1000, inclusivePercent: item.inclusiveUs / totalUs * 100 });
const report = { samples: profile.samples.length, sampledMs: totalUs / 1000,
    note: 'Loop only: physics plus benchmark bookkeeping; excludes anatomy loading. Inclusive times overlap and must not be added.',
    self: [...functions.values()].sort((a, b) => b.selfUs - a.selfUs).slice(0, 40).map(format),
    inclusive: [...functions.values()].sort((a, b) => b.inclusiveUs - a.inclusiveUs).slice(0, 60).map(format),
    files: [...files].sort((a, b) => b[1] - a[1]).map(([url, us]) => ({ url, selfMs: us / 1000, percent: us / totalUs * 100 })) };
if (process.argv[3]) fs.writeFileSync(process.argv[3], JSON.stringify(report, null, 2) + '\n');
console.log(JSON.stringify(report.self.slice(0, 15), null, 2));
