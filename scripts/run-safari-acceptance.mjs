import fs from 'node:fs';
import path from 'node:path';
import { execFile } from 'node:child_process';
import { promisify } from 'node:util';

const execFileAsync = promisify(execFile);

function argument(name, fallback) {
    const index = process.argv.indexOf(`--${name}`);
    return index >= 0 && index + 1 < process.argv.length ? process.argv[index + 1] : fallback;
}

function delay(ms) {
    return new Promise(resolve => setTimeout(resolve, ms));
}

async function activateApplication(name) {
    if (!name) return;
    if (!/^[A-Za-z0-9 ._-]+$/.test(name)) throw new Error(`Invalid application name: ${name}`);
    await execFileAsync('/usr/bin/osascript', ['-e', `tell application "${name}" to activate`]);
}

const endpoint = argument('endpoint', 'http://127.0.0.1:4444').replace(/\/$/, '');
const durationMs = Number(argument('duration-ms', '600000'));
const outputPath = path.resolve(argument('output', 'reports/browser-acceptance-safari.json'));
const screenshotPath = path.resolve(argument('screenshot', 'reports/browser-acceptance-safari.png'));
const targetUrl = argument('target-url', 'http://127.0.0.1:5173/?acceptance=final-safari');
const activateApp = argument('activate-app', 'Safari');
const focusIntervalMs = Number(argument('focus-interval-ms', '0'));

if (!Number.isFinite(durationMs) || durationMs <= 0) {
    throw new RangeError('--duration-ms must be a positive number');
}
if (!Number.isFinite(focusIntervalMs) || focusIntervalMs < 0) {
    throw new RangeError('--focus-interval-ms must be zero or a positive number');
}

async function webdriver(method, commandPath, body = null) {
    const response = await fetch(`${endpoint}${commandPath}`, {
        method,
        headers: body === null ? undefined : { 'Content-Type': 'application/json' },
        body: body === null ? undefined : JSON.stringify(body)
    });
    const payload = await response.json();
    if (!response.ok || payload.value?.error) {
        throw new Error(payload.value?.message || `WebDriver ${method} ${commandPath} failed (${response.status})`);
    }
    return payload.value;
}

let sessionId = null;
let maintainFocus = false;
let focusLoop = Promise.resolve();

try {
    const session = await webdriver('POST', '/session', {
        capabilities: {
            alwaysMatch: { browserName: 'safari' }
        }
    });
    sessionId = session.sessionId;
    const sessionPath = `/session/${sessionId}`;
    await webdriver('POST', `${sessionPath}/window/rect`, { width: 1440, height: 900 });
    await webdriver('POST', `${sessionPath}/url`, { url: targetUrl });

    const execute = script => webdriver('POST', `${sessionPath}/execute/sync`, {
        script: `return (${script});`,
        args: []
    });
    const startedLoadingAt = Date.now();
    while (Date.now() - startedLoadingAt < 120000) {
        const ready = await execute(
            `Boolean(window.__OET_BENCHMARK__?.startScenario &&
                !document.querySelector('#loadingScreen:not(.is-hidden)'))`
        );
        if (ready) break;
        await delay(250);
    }
    const ready = await execute(
        `Boolean(window.__OET_BENCHMARK__?.startScenario &&
            !document.querySelector('#loadingScreen:not(.is-hidden)'))`
    );
    if (!ready) throw new Error('Timed out waiting for the simulator');

    const browser = await execute(`(() => {
        const canvas = document.querySelector('canvas');
        const gl = canvas?.getContext('webgl2') || canvas?.getContext('webgl');
        const extension = gl?.getExtension('WEBGL_debug_renderer_info');
        return {
            userAgent: navigator.userAgent,
            platform: navigator.platform,
            viewport: { width: innerWidth, height: innerHeight, devicePixelRatio },
            renderer: extension ? gl.getParameter(extension.UNMASKED_RENDERER_WEBGL) : null
        };
    })()`);

    if (activateApp) await activateApplication(activateApp);
    await execute(`window.__OET_BENCHMARK__.startScenario({ durationMs: ${durationMs}, automated: true })`);
    if (activateApp && focusIntervalMs > 0) {
        maintainFocus = true;
        focusLoop = (async () => {
            while (maintainFocus) {
                await delay(focusIntervalMs);
                if (!maintainFocus) break;
                await activateApplication(activateApp);
            }
        })();
    }

    const totalWaitMs = durationMs + 5 * 60 * 1000;
    const startedAt = Date.now();
    let status = await execute('window.__OET_BENCHMARK__.getScenarioStatus()');
    let nextWarmupLogAt = 0;
    while (status.running && status.warmingUp && Date.now() - startedAt < totalWaitMs) {
        if (Date.now() >= nextWarmupLogAt) {
            console.log('Safari acceptance: warmup');
            nextWarmupLogAt = Date.now() + 30000;
        }
        await delay(100);
        status = await execute('window.__OET_BENCHMARK__.getScenarioStatus()');
    }
    while (Date.now() - startedAt < totalWaitMs + 120000) {
        status = await execute('window.__OET_BENCHMARK__.getScenarioStatus()');
        console.log(`Safari acceptance: ${Math.round(status.elapsedMs / 1000)}s`);
        if (!status.running) break;
        await delay(30000);
    }

    const report = await execute(
        'window.__OET_BENCHMARK__.getLastScenarioReport() || window.__OET_BENCHMARK__.getReport()'
    );
    if (report?.scenario?.running || report?.scenario?.elapsedMs < durationMs) {
        throw new Error(
            `Safari benchmark did not finish (${report?.scenario?.elapsedMs || 0}/${durationMs} ms, ` +
            `reason: ${report?.scenario?.stopReason || 'unknown'})`
        );
    }

    const screenshot = await webdriver('GET', `${sessionPath}/screenshot`);
    fs.mkdirSync(path.dirname(outputPath), { recursive: true });
    fs.writeFileSync(screenshotPath, Buffer.from(screenshot, 'base64'));
    fs.writeFileSync(outputPath, `${JSON.stringify({
        schemaVersion: 1,
        generatedAt: new Date().toISOString(),
        browser,
        report
    }, null, 2)}\n`);
    console.log(JSON.stringify({
        outputPath,
        screenshotPath,
        averageFps: report.averageFps,
        onePercentLowFps: report.onePercentLowFps,
        physicsAverageMs: report.physics.phases.total.averageMs,
        physicsP95Ms: report.physics.phases.total.p95Ms,
        maxPostStepPenetrationMm: report.physicsEnvelope.maxPostStepPenetrationMm,
        passed: report.browserAcceptance.passed
    }, null, 2));
} finally {
    maintainFocus = false;
    await focusLoop.catch(() => {});
    if (sessionId) await webdriver('DELETE', `/session/${sessionId}`).catch(() => {});
}
