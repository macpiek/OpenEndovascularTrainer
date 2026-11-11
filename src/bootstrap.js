import {startupDiagnostics} from './startupDiagnostics.js';

const loadingMessage = document.getElementById('loadingMessage');

function setLoadingMessage(message) {
    if (loadingMessage) loadingMessage.textContent = message;
}

function describeBootError(error) {
    if (error instanceof Error && error.message) return error.message;
    return String(error || 'Unknown startup error');
}

await startupDiagnostics.before('Loading simulator code');

const slowBootTimer = setTimeout(() => {
    const stage = loadingMessage?.textContent || 'Starting simulator';
    setLoadingMessage(`${stage} — taking longer than expected`);
}, 15000);

try {
    await import('./simulator.js');
    clearTimeout(slowBootTimer);
} catch (error) {
    clearTimeout(slowBootTimer);
    const detail = describeBootError(error);
    setLoadingMessage(`Simulator failed to start: ${detail}`);
    console.error('Simulator bootstrap failed', error);
}
