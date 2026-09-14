const STORAGE_KEY = 'oet.shared-axis.last-failure.v1';

export function describeSolverFailure(report) {
    if (!report) return 'Brak zapisanego odrzucenia.';
    const {result={},capturedAt,captureError}=report.failure;
    const lines=[`Ostatnie odrzucenie: ${capturedAt}`,`Przyczyna: ${result.error || result.status || 'nieznana'}`];
    for (const tool of report.stepRequest?.tools ?? []) {
        const before=report.tools?.find(t=>t.id===tool.id);
        lines.push(`${tool.id}: ${before?.insertion ?? '?'} → ${tool.insertion} mm; obrót ${report.stepRequest.rotations?.[tool.id] ?? '?'} → ${tool.rotation} rad`);
    }
    lines.push(`Iteracje: ${result.iterations ?? '—'}; faktoryzacje: ${result.factorizations ?? '—'}`);
    if (result.residual) lines.push(`Błędy ograniczeń: ${JSON.stringify(result.residual)}`);
    for (const attempt of result.attempts ?? []) lines.push(`Podział ${attempt.subdivisions}, część ${attempt.index}: ${attempt.status}${attempt.wallNormalFallback ? `; wcześniejsza próba: ${attempt.wallNormalFallback.liveFailure}` : ''}`);
    if (captureError) lines.push(`Niepełny zapis: ${captureError}`);
    return lines.join('\n');
}

/** One local record, replaced only on rejection. Storage denial/quota cannot
 * affect physics; the in-memory report remains downloadable in that session. */
export function createSolverFailurePanel({button,output,storage,download}) {
    let report=null,storageError='';
    function render() {
        button.disabled=!report;
        output.value=describeSolverFailure(report)+(storageError?`\nZapis tylko w pamięci karty: ${storageError}`:'');
    }
    try {
        const saved=storage?.getItem(STORAGE_KEY);
        if(saved) {const value=JSON.parse(saved);if(value.version===1&&value.failure)report=value;}
    } catch { /* An unavailable/obsolete local record does not prevent startup. */ }
    button.addEventListener('click',()=>{
        if(report)download(JSON.stringify(report),`oet-rejected-step-${report.failure.capturedAt.replace(/[^0-9TZ]/g,'-')}.json`);
    });
    render();
    return {record(value) {
        if(!value)return;
        report=value;storageError='';
        try {storage?.setItem(STORAGE_KEY,JSON.stringify(report));}
        catch(error) {storageError=error.message;}
        render();
    }};
}
