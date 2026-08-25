import assert from 'node:assert/strict';
import { PatientMonitor } from '../src/ui/patientMonitor.js';

class FakeGradient {
    addColorStop() {}
}

class FakeContext {
    constructor() {
        this.paths = [];
        this.currentPath = [];
    }

    createLinearGradient() { return new FakeGradient(); }
    setTransform() {}
    clearRect() {}
    fillRect() {}
    save() {}
    restore() {}
    setLineDash() {}
    beginPath() { this.currentPath = []; }
    moveTo(x, y) { this.currentPath.push({ x, y }); }
    lineTo(x, y) { this.currentPath.push({ x, y }); }
    stroke() { this.paths.push(this.currentPath.slice()); }
}

class FakeCanvas {
    constructor(width = 244, height = 48) {
        this.clientWidth = width;
        this.clientHeight = height;
        this.width = width;
        this.height = height;
        this.context = new FakeContext();
    }

    getContext() { return this.context; }
}

function element() {
    return { textContent: '' };
}

function createMonitor() {
    const ecgCanvas = new FakeCanvas();
    const bpCanvas = new FakeCanvas();
    const monitor = new PatientMonitor(
        ecgCanvas,
        bpCanvas,
        element(),
        element(),
        {
            spo2Elem: element(),
            mapElem: element(),
            rrElem: element(),
            rhythmElem: element(),
            clockElem: element()
        }
    );
    return { monitor, ecgCanvas, bpCanvas };
}

function longestPath(context) {
    return context.paths.reduce(
        (longest, path) => path.length > longest.length ? path : longest,
        []
    );
}

globalThis.window = { devicePixelRatio: 2 };

{
    const regular = createMonitor().monitor;
    const delayed = createMonitor().monitor;

    for (let step = 0; step < 120; step++) {
        regular.advance(1 / 120);
        delayed.advance(1 / 120);
        regular.render(1 / 120);
    }
    for (const dt of [0.117, 0.033, 0.4, 0.2, 0.25]) delayed.render(dt);

    const regularTiming = regular.timingDiagnostics();
    const delayedTiming = delayed.timingDiagnostics();
    assert.ok(
        Math.abs(regularTiming.ecgContinuousCursor - 250) < 1e-9,
        `regular ECG clock drifted to ${regularTiming.ecgContinuousCursor}`
    );
    assert.ok(
        Math.abs(delayedTiming.ecgContinuousCursor - 250) < 1e-9,
        `delayed ECG clock drifted to ${delayedTiming.ecgContinuousCursor}`
    );
    assert.ok(
        Math.abs(regularTiming.bpContinuousCursor - 50) < 1e-9,
        `regular BP clock drifted to ${regularTiming.bpContinuousCursor}`
    );
    assert.ok(
        Math.abs(delayedTiming.bpContinuousCursor - 50) < 1e-9,
        `delayed BP clock drifted to ${delayedTiming.bpContinuousCursor}`
    );
    assert.ok(
        Math.abs(regularTiming.ecgPresentationCursor - 250) < 1e-9,
        `regular ECG presentation clock drifted to ${regularTiming.ecgPresentationCursor}`
    );
    assert.ok(
        Math.abs(delayedTiming.ecgPresentationCursor - 250) < 1e-9,
        `delayed ECG presentation clock drifted to ${delayedTiming.ecgPresentationCursor}`
    );
}

{
    const slow = createMonitor().monitor;
    const fast = createMonitor().monitor;
    slow.setHeartRate(60);
    fast.setHeartRate(100);
    slow.advance(0.5);
    fast.advance(0.5);
    slow.render(0.5);
    fast.render(0.5);
    assert.equal(
        slow.timingDiagnostics().ecgPresentationCursor,
        fast.timingDiagnostics().ecgPresentationCursor,
        'heart rate changed the ECG sweep clock'
    );
}

{
    const { monitor } = createMonitor();
    monitor.advance(0.25);
    const before = {
        currentHR: monitor.currentHR,
        systolic: monitor.systolic,
        diastolic: monitor.diastolic,
        meanPressure: monitor.meanPressure,
        spo2: monitor.spo2,
        respiratoryRate: monitor.respiratoryRate,
        timing: monitor.timingDiagnostics()
    };
    for (let frame = 0; frame < 20; frame++) monitor.render(1 / 60);
    const after = monitor.timingDiagnostics();
    assert.equal(after.ecgContinuousCursor, before.timing.ecgContinuousCursor);
    assert.equal(after.bpContinuousCursor, before.timing.bpContinuousCursor);
    assert.ok(Math.abs(after.ecgPresentationCursor - 20 / 60 * 250) < 1e-9);
    assert.equal(after.renderCount, before.timing.renderCount + 20);
    assert.equal(monitor.currentHR, before.currentHR);
    assert.equal(monitor.systolic, before.systolic);
    assert.equal(monitor.diastolic, before.diastolic);
    assert.equal(monitor.meanPressure, before.meanPressure);
    assert.equal(monitor.spo2, before.spo2);
    assert.equal(monitor.respiratoryRate, before.respiratoryRate);
}

{
    const { monitor, ecgCanvas } = createMonitor();
    monitor.render(0.001);
    const firstTrace = longestPath(ecgCanvas.context);
    const firstLastX = firstTrace.at(-1).x;
    const firstPhase = monitor.timingDiagnostics().ecgPresentationFractionalPhase;

    ecgCanvas.context.paths.length = 0;
    monitor.render(0.002);
    const secondTrace = longestPath(ecgCanvas.context);
    const secondLastX = secondTrace.at(-1).x;
    const secondPhase = monitor.timingDiagnostics().ecgPresentationFractionalPhase;

    assert.ok(Math.abs(firstPhase - 0.25) < 1e-9);
    assert.ok(Math.abs(secondPhase - 0.75) < 1e-9);
    assert.ok(
        secondLastX < firstLastX,
        'trace did not move between two ECG sample boundaries'
    );
    const expectedShift = 0.5 / (monitor.ecgBufferLength - 1) * ecgCanvas.clientWidth;
    assert.ok(
        Math.abs((firstLastX - secondLastX) - expectedShift) < 1e-9,
        'fractional cursor did not produce the expected continuous screen shift'
    );
}

// A heavily loaded tool solver may execute only one physics step per 30 Hz
// presented frame. The 60 Hz monitor must follow presentation time instead of
// that unrelated backlog, keeping produced samples adjacent to the sweep
// cursor even after many buffer windows have elapsed.
{
    const { monitor, ecgCanvas, bpCanvas } = createMonitor();
    const frameDt = 1 / 30;
    const frameCount = 30 * 30;
    for (let frame = 0; frame < frameCount; frame++) {
        ecgCanvas.context.paths.length = 0;
        bpCanvas.context.paths.length = 0;
        monitor.updatePresentation(frameDt);
    }
    const timing = monitor.timingDiagnostics();
    assert.ok(
        Math.abs(
            timing.ecgContinuousCursor - timing.ecgPresentationCursor
        ) <= monitor.ecgSampleRate / 60 + 1e-9,
        'ECG presentation cursor outran wall-clock sample production'
    );
    assert.ok(
        Math.abs(
            timing.bpContinuousCursor - timing.bpPresentationCursor
        ) <= monitor.bpSampleRate / 60 + 1e-9,
        'BP presentation cursor outran wall-clock sample production'
    );
    assert.ok(longestPath(ecgCanvas.context).length > 100,
        'ECG trace disappeared after sustained physics backlog');
    assert.ok(longestPath(bpCanvas.context).length > 100,
        'BP trace disappeared after sustained physics backlog');
    assert.ok(timing.presentationAdvanceAccumulator < 1 / 60,
        'monitor fixed-step accumulator did not remain bounded');
}

console.log('patient monitor timing tests passed');
