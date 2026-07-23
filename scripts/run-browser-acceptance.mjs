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

class CdpClient {
    constructor(url) {
        this.url = url;
        this.socket = null;
        this.nextId = 1;
        this.pending = new Map();
        this.disconnectError = null;
    }

    async connect() {
        this.socket = new WebSocket(this.url);
        this.socket.addEventListener('message', event => {
            const message = JSON.parse(event.data);
            if (!message.id) return;
            const pending = this.pending.get(message.id);
            if (!pending) return;
            this.pending.delete(message.id);
            if (message.error) pending.reject(new Error(message.error.message));
            else pending.resolve(message.result);
        });
        const rejectPending = () => {
            this.disconnectError ??= new Error('CDP socket closed while the browser acceptance test was running');
            for (const pending of this.pending.values()) pending.reject(this.disconnectError);
            this.pending.clear();
        };
        this.socket.addEventListener('close', rejectPending);
        this.socket.addEventListener('error', rejectPending);
        await new Promise((resolve, reject) => {
            this.socket.addEventListener('open', resolve, { once: true });
            this.socket.addEventListener('error', reject, { once: true });
        });
    }

    send(method, params = {}) {
        if (this.disconnectError) return Promise.reject(this.disconnectError);
        if (this.socket?.readyState !== WebSocket.OPEN) {
            return Promise.reject(new Error(`Cannot send ${method}: CDP socket is not open`));
        }
        const id = this.nextId++;
        return new Promise((resolve, reject) => {
            this.pending.set(id, { resolve, reject });
            try {
                this.socket.send(JSON.stringify({ id, method, params }));
            } catch (error) {
                this.pending.delete(id);
                reject(error);
            }
        });
    }

    async evaluate(expression) {
        const result = await this.send('Runtime.evaluate', {
            expression,
            awaitPromise: true,
            returnByValue: true
        });
        if (result.exceptionDetails) {
            throw new Error(result.exceptionDetails.exception?.description || 'Browser evaluation failed');
        }
        return result.result.value;
    }

    close() {
        this.socket?.close();
    }
}

async function waitFor(client, expression, timeoutMs, label) {
    const startedAt = Date.now();
    while (Date.now() - startedAt < timeoutMs) {
        if (await client.evaluate(expression)) return;
        await delay(250);
    }
    throw new Error(`Timed out waiting for ${label}`);
}

const endpoint = argument('endpoint', 'http://127.0.0.1:9223');
const durationMs = Number(argument('duration-ms', '600000'));
const outputPath = path.resolve(argument('output', 'reports/browser-acceptance-chrome.json'));
const screenshotPath = path.resolve(argument('screenshot', 'reports/browser-acceptance-chrome.png'));
const heapProfileArgument = argument('heap-profile', '');
const heapProfilePath = heapProfileArgument ? path.resolve(heapProfileArgument) : null;
const activateApp = argument('activate-app', '');
const focusIntervalMs = Number(argument('focus-interval-ms', '0'));
const emulateFocus = argument('emulate-focus', 'false') === 'true';
const targetUrl = argument('target-url', 'http://127.0.0.1:5173/');

if (!Number.isFinite(durationMs) || durationMs <= 0) {
    throw new RangeError('--duration-ms must be a positive number');
}
if (!Number.isFinite(focusIntervalMs) || focusIntervalMs < 0) {
    throw new RangeError('--focus-interval-ms must be zero or a positive number');
}

const targets = await fetch(`${endpoint}/json/list`).then(response => {
    if (!response.ok) throw new Error(`CDP target discovery failed with ${response.status}`);
    return response.json();
});
const target = targets.find(candidate => candidate.type === 'page' && candidate.url.startsWith(targetUrl));
if (!target?.webSocketDebuggerUrl) throw new Error(`No page target found for ${targetUrl}`);

const client = new CdpClient(target.webSocketDebuggerUrl);
await client.connect();
let maintainFocus = false;
let focusLoop = Promise.resolve();
let heapSamplingStarted = false;

try {
    await client.send('Runtime.enable');
    await client.send('Page.enable');
    await client.send('Page.bringToFront');
    if (emulateFocus) await client.send('Emulation.setFocusEmulationEnabled', { enabled: true });
    await waitFor(
        client,
        `Boolean(window.__OET_BENCHMARK__?.startScenario && !document.querySelector('#loadingScreen:not(.is-hidden)'))`,
        120000,
        'the simulator'
    );

    const browser = await client.evaluate(`(() => {
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

    if (heapProfilePath) {
        await client.send('HeapProfiler.enable');
    }
    if (activateApp) await activateApplication(activateApp);
    await client.evaluate(
        `window.__OET_BENCHMARK__.startScenario({ durationMs: ${durationMs}, automated: true })`
    );
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
    let status = await client.evaluate(`window.__OET_BENCHMARK__.getScenarioStatus()`);
    let nextWarmupLogAt = 0;
    while (status.running && status.warmingUp && Date.now() - startedAt < totalWaitMs) {
        if (Date.now() >= nextWarmupLogAt) {
            console.log('browser acceptance: warmup');
            nextWarmupLogAt = Date.now() + 30000;
        }
        await delay(100);
        status = await client.evaluate(`window.__OET_BENCHMARK__.getScenarioStatus()`);
    }
    if (heapProfilePath && status.running) {
        await client.send('HeapProfiler.startSampling', {
            samplingInterval: 32768,
            includeObjectsCollectedByMajorGC: true,
            includeObjectsCollectedByMinorGC: true
        });
        heapSamplingStarted = true;
    }
    // Keep CDP completely idle while frame pacing is measured. Even a small
    // Runtime.evaluate pause can contaminate the worst one-second FPS windows.
    await delay(durationMs + 1000);
    status = await client.evaluate(`window.__OET_BENCHMARK__.getScenarioStatus()`);
    while (status.running && Date.now() - startedAt < totalWaitMs + 120000) {
        await delay(1000);
        status = await client.evaluate(`window.__OET_BENCHMARK__.getScenarioStatus()`);
    }
    console.log(`browser acceptance: ${Math.round(status.elapsedMs / 1000)}s`);

    const report = await client.evaluate(
        `window.__OET_BENCHMARK__.getLastScenarioReport() || window.__OET_BENCHMARK__.getReport()`
    );
    if (report?.scenario?.running || report?.scenario?.elapsedMs < durationMs) {
        throw new Error(
            `Browser benchmark did not finish (${report?.scenario?.elapsedMs || 0}/${durationMs} ms, ` +
            `reason: ${report?.scenario?.stopReason || 'unknown'})`
        );
    }

    if (heapSamplingStarted) {
        const { profile } = await client.send('HeapProfiler.stopSampling');
        heapSamplingStarted = false;
        fs.mkdirSync(path.dirname(heapProfilePath), { recursive: true });
        fs.writeFileSync(heapProfilePath, `${JSON.stringify(profile)}\n`);
    }

    const screenshot = await client.send('Page.captureScreenshot', {
        format: 'png',
        captureBeyondViewport: false
    });
    fs.mkdirSync(path.dirname(outputPath), { recursive: true });
    fs.writeFileSync(screenshotPath, Buffer.from(screenshot.data, 'base64'));
    fs.writeFileSync(outputPath, `${JSON.stringify({
        schemaVersion: 1,
        generatedAt: new Date().toISOString(),
        browser,
        report
    }, null, 2)}\n`);
    console.log(JSON.stringify({
        outputPath,
        screenshotPath,
        heapProfilePath,
        averageFps: report.averageFps,
        onePercentLowFps: report.onePercentLowFps,
        physicsAverageMs: report.physics.phases.total.averageMs,
        physicsP95Ms: report.physics.phases.total.p95Ms,
        maxPostStepPenetrationMm: report.physicsEnvelope.maxPostStepPenetrationMm,
        passed: report.browserAcceptance.passed
    }, null, 2));
} finally {
    maintainFocus = false;
    if (heapSamplingStarted) await client.send('HeapProfiler.stopSampling').catch(() => {});
    client.close();
    await focusLoop.catch(() => {});
}
