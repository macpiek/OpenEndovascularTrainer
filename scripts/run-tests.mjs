import { readFileSync } from 'node:fs';
import { spawnSync } from 'node:child_process';
import { fileURLToPath } from 'node:url';

const root = fileURLToPath(new URL('../', import.meta.url));
const suites = JSON.parse(readFileSync(new URL('./test-suites.json', import.meta.url), 'utf8'));
const suiteName = process.argv[2] ?? 'default';

if (!Object.hasOwn(suites, suiteName)) {
    console.error(`Unknown test suite: ${suiteName}. Available: ${Object.keys(suites).join(', ')}`);
    process.exit(1);
}

// Keep each suite's original process boundaries, flags and execution order.
// In particular, standalone assertions and node:test files run as before.
for (const [command, ...args] of suites[suiteName]) {
    console.log(`\n> ${command} ${args.join(' ')}`);
    const npmPath = command === 'npm' && process.env.npm_execpath;
    const executable = command === 'node' || npmPath ? process.execPath : command;
    const commandArgs = npmPath ? [npmPath, ...args] : args;
    const result = spawnSync(executable, commandArgs, { cwd: root, stdio: 'inherit' });
    if (result.error) {
        console.error(result.error.message);
        process.exit(1);
    }
    if (result.signal) {
        console.error(`Test process terminated by ${result.signal}`);
        process.exit(1);
    }
    if (result.status !== 0) process.exit(result.status ?? 1);
}
