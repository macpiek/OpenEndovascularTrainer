const MONITOR_DPR_LIMIT = 2;
const MONITOR_READOUT_INTERVAL = 1 / 15;
const MONITOR_ADVANCE_STEP = 1 / 60;
const ECG_BASELINE = 0.58;
const ECG_TRACE_GAIN = 0.29;
const clamp = (value, min, max) => Math.min(max, Math.max(min, value));
const lerp = (from, to, amount) => from + (to - from) * amount;

export class PatientMonitor {
    constructor(ecgCanvas, bpCanvas, hrElem, bpElem, elements = {}) {
        this.ecgCanvas = ecgCanvas;
        this.bpCanvas = bpCanvas;
        this.hrElem = hrElem;
        this.bpElem = bpElem;
        this.spo2Elem = elements.spo2Elem || null;
        this.mapElem = elements.mapElem || null;
        this.rrElem = elements.rrElem || null;
        this.rhythmElem = elements.rhythmElem || null;
        this.clockElem = elements.clockElem || null;

        this.ecgCtx = ecgCanvas.getContext('2d');
        this.bpCtx = bpCanvas.getContext('2d');
        this.ecgCanvasState = this.#createCanvasState('#020303', '#000000');
        this.bpCanvasState = this.#createCanvasState('#030202', '#000000');
        this.baselineDash = [6, 8];

        this.ecgSampleRate = 250;
        this.bpSampleRate = 50;

        this.ecgBufferLength = this.ecgSampleRate * 10;
        this.bpBufferLength = this.bpSampleRate * 10;

        this.ecgData = new Float32Array(this.ecgBufferLength);
        this.bpData = new Float32Array(this.bpBufferLength);
        this.bpData.fill(100);
        this.ecgCursor = 0;
        this.bpCursor = 0;
        this.ecgTotalSamples = 0;
        this.bpTotalSamples = 0;
        this.readoutAccumulator = MONITOR_READOUT_INTERVAL;
        this.renderCount = 0;
        this.presentationTime = 0;
        this.ecgPresentationCursor = 0;
        this.bpPresentationCursor = 0;
        this.presentationAdvanceAccumulator = 0;
        this.lastReadouts = Object.create(null);
        this.lastClockSecond = -1;
        this.clockLabel = '00:00';

        this.time = 0;
        this.cycleTime = 0;
        this.variabilitySeed = Math.random() * Math.PI * 2;
        this.baseHeartRate = 75; // bpm
        this.heartRate = this.baseHeartRate;
        this.beatInterval = 60 / this.heartRate;

        this.ecgAccumulator = 0;
        this.bpAccumulator = 0;

        this.currentHR = this.heartRate;
        this.baselineSystolic = 120;
        this.baselineDiastolic = 80;
        this.waveSystolic = this.baselineSystolic;
        this.waveDiastolic = this.baselineDiastolic;
        this.systolic = 120;
        this.diastolic = 80;
        this.meanPressure = 93;
        this.spo2 = 98;
        this.spo2Target = this.spo2;
        this.respiratoryRate = 14;
        this.respiratoryRateTarget = this.respiratoryRate;
        this.bpMax = 0;
        this.bpMin = Infinity;

        this.ecgTemplate = this.#createEcgTemplate();
        this.ecgTemplateIndex = 0;
        this.ecgSamplesSinceBeat = 0;
        this.ecgSamplesToNextBeat = this.#ecgSamplesForCurrentBeat();
        this.bpTemplate = this.#createBpTemplate();
    }

    setHeartRate(hr) {
        this.baseHeartRate = hr;
        this.heartRate = hr;
        this.beatInterval = 60 / this.heartRate;
        this.currentHR = hr;
    }

    advance(dt) {
        if (!Number.isFinite(dt) || dt <= 0) return;
        this.ecgAccumulator += dt;
        this.bpAccumulator += dt;
        this.time += dt;
        this.cycleTime += dt;

        const ecgStep = 1 / this.ecgSampleRate;
        while (this.ecgAccumulator >= ecgStep) {
            this.ecgAccumulator -= ecgStep;
            const ecg = this.#nextEcgSample();
            this.ecgData[this.ecgCursor] = ecg;
            this.ecgCursor = (this.ecgCursor + 1) % this.ecgBufferLength;
            this.ecgTotalSamples += 1;
        }

        const bpStep = 1 / this.bpSampleRate;
        while (this.bpAccumulator >= bpStep) {
            this.bpAccumulator -= bpStep;
            const phase = (this.cycleTime / this.beatInterval) % 1;
            const index = Math.floor(phase * this.bpTemplate.length);
            const pressure = this.#scaledBpPressure(this.bpTemplate[index]);
            this.bpData[this.bpCursor] = pressure;
            this.bpCursor = (this.bpCursor + 1) % this.bpBufferLength;
            this.bpTotalSamples += 1;
            if (pressure > this.bpMax) this.bpMax = pressure;
            if (pressure < this.bpMin) this.bpMin = pressure;
        }

        if (this.cycleTime >= this.beatInterval) {
            this.currentHR = 60 / this.beatInterval;
            this.systolic = this.bpMax;
            this.diastolic = this.bpMin;
            this.meanPressure = this.diastolic + (this.systolic - this.diastolic) / 3;
            this.cycleTime -= this.beatInterval;
            this.bpMax = 0;
            this.bpMin = Infinity;
            this.#advanceBeatVitals();
        }

        this.#updateContinuousVitals(dt);

        this.readoutAccumulator += dt;
    }

    render(dt = 0) {
        if (Number.isFinite(dt) && dt > 0) {
            this.presentationTime += dt;
            this.ecgPresentationCursor += dt * this.ecgSampleRate;
            this.bpPresentationCursor += dt * this.bpSampleRate;
        }
        if (this.readoutAccumulator >= MONITOR_READOUT_INTERVAL) {
            this.readoutAccumulator %= MONITOR_READOUT_INTERVAL;

            this.#setReadout('hr', this.hrElem, Math.round(this.currentHR));
            this.#setBloodPressureReadout();
            this.#setReadout('spo2', this.spo2Elem, Math.round(this.spo2));
            this.#setReadout('map', this.mapElem, Math.round(this.meanPressure));
            this.#setReadout('rr', this.rrElem, Math.round(this.respiratoryRate));
            this.#setReadout('rhythm', this.rhythmElem, this.#rhythmLabel());
            this.#setReadout('clock', this.clockElem, this.#clockLabel());
        }

        this.#drawEcg();
        this.#drawBp();
        this.renderCount += 1;
    }

    // Keep physiology and the screen sweep on one presentation-time clock.
    // The fixed inner step preserves deterministic waveform generation, while
    // decoupling it from the expensive tool solver prevents a physics backlog
    // from letting the screen cursor run beyond every available sample.
    updatePresentation(dt, fixedStep = MONITOR_ADVANCE_STEP) {
        if (!Number.isFinite(dt) || dt < 0) return;
        const step = Number.isFinite(fixedStep) && fixedStep > 0
            ? fixedStep
            : MONITOR_ADVANCE_STEP;
        this.presentationAdvanceAccumulator += dt;
        while (this.presentationAdvanceAccumulator + 1e-12 >= step) {
            this.advance(step);
            this.presentationAdvanceAccumulator -= step;
        }
        if (this.presentationAdvanceAccumulator < 0) {
            this.presentationAdvanceAccumulator = 0;
        }
        this.render(dt);
    }

    // Preserve the old public entry point for embedders. The simulator uses
    // advance() and render() separately so physics catch-up cannot produce
    // several invisible monitor draws between two presented frames.
    update(dt) {
        this.advance(dt);
        this.render(dt);
    }

    timingDiagnostics() {
        return {
            timeSeconds: this.time,
            ecgSampleRate: this.ecgSampleRate,
            ecgCursor: this.ecgCursor,
            ecgFractionalSamplePhase: this.ecgAccumulator * this.ecgSampleRate,
            ecgContinuousCursor:
                this.ecgTotalSamples + this.ecgAccumulator * this.ecgSampleRate,
            ecgTotalSamples: this.ecgTotalSamples,
            ecgPresentationCursor: this.ecgPresentationCursor,
            ecgPresentationFractionalPhase:
                this.ecgPresentationCursor - Math.floor(this.ecgPresentationCursor),
            bpSampleRate: this.bpSampleRate,
            bpCursor: this.bpCursor,
            bpFractionalSamplePhase: this.bpAccumulator * this.bpSampleRate,
            bpContinuousCursor:
                this.bpTotalSamples + this.bpAccumulator * this.bpSampleRate,
            bpTotalSamples: this.bpTotalSamples,
            bpPresentationCursor: this.bpPresentationCursor,
            bpPresentationFractionalPhase:
                this.bpPresentationCursor - Math.floor(this.bpPresentationCursor),
            presentationTimeSeconds: this.presentationTime,
            presentationAdvanceAccumulator:
                this.presentationAdvanceAccumulator,
            sweepWindowSeconds: this.ecgBufferLength / this.ecgSampleRate,
            renderCount: this.renderCount
        };
    }

    #setReadout(key, element, value) {
        if (!element || this.lastReadouts[key] === value) return;
        element.textContent = value;
        this.lastReadouts[key] = value;
    }

    #setBloodPressureReadout() {
        if (!this.bpElem) return;
        const systolic = Math.round(this.systolic);
        const diastolic = Math.round(this.diastolic);
        const key = systolic * 256 + diastolic;
        if (this.lastReadouts.bp === key) return;
        this.bpElem.textContent = `${systolic}/${diastolic}`;
        this.lastReadouts.bp = key;
    }

    #advanceBeatVitals() {
        const respiratoryWave = Math.sin(this.time * 0.34 + this.variabilitySeed);
        const baroreflexWave = Math.sin(this.time * 0.11 + this.variabilitySeed * 0.7);
        const beatNoise = (Math.random() - 0.5) * 1.8;
        const pressureNoise = (Math.random() - 0.5) * 2.2;

        this.heartRate = clamp(
            this.baseHeartRate + respiratoryWave * 2.2 + baroreflexWave * 1.4 + beatNoise,
            58,
            96
        );
        this.beatInterval = 60 / this.heartRate;
        this.ecgSamplesToNextBeat = this.#ecgSamplesForCurrentBeat();
        this.currentHR = lerp(this.currentHR, this.heartRate, 0.75);

        this.waveSystolic = clamp(
            this.baselineSystolic + respiratoryWave * 3.2 + baroreflexWave * 2 + pressureNoise,
            106,
            134
        );
        this.waveDiastolic = clamp(
            this.baselineDiastolic + respiratoryWave * 1.6 + baroreflexWave * 1.2 + pressureNoise * 0.45,
            68,
            88
        );
    }

    #nextEcgSample() {
        const sample = this.ecgTemplateIndex < this.ecgTemplate.length
            ? this.ecgTemplate[this.ecgTemplateIndex]
            : 0;

        this.ecgTemplateIndex += 1;
        this.ecgSamplesSinceBeat += 1;

        if (this.ecgSamplesSinceBeat >= this.ecgSamplesToNextBeat) {
            this.ecgTemplateIndex = 0;
            this.ecgSamplesSinceBeat = 0;
            this.ecgSamplesToNextBeat = this.#ecgSamplesForCurrentBeat();
        }

        return sample;
    }

    #ecgSamplesForCurrentBeat() {
        return Math.max(this.ecgTemplate?.length || 1, Math.round(this.beatInterval * this.ecgSampleRate));
    }

    #updateContinuousVitals(dt) {
        const respiratoryWave = Math.sin(this.time * 0.31 + this.variabilitySeed);
        const slowWave = Math.sin(this.time * 0.07 + this.variabilitySeed * 1.9);
        const targetSaturation = 98 + respiratoryWave * 0.9 + slowWave * 0.65;
        const targetRespirations = 14 + respiratoryWave * 0.9 + slowWave * 0.5;

        this.spo2Target = clamp(targetSaturation, 96, 100);
        this.respiratoryRateTarget = clamp(targetRespirations, 11, 18);
        this.spo2 = lerp(this.spo2, this.spo2Target, clamp(dt * 1.4, 0, 1));
        this.respiratoryRate = lerp(this.respiratoryRate, this.respiratoryRateTarget, clamp(dt * 0.8, 0, 1));
    }

    #ecgWaveformAtTime(secondsAfterBeat) {
        const gaussian = (centerSeconds, widthSeconds, amplitude) => {
            const normalizedTime = (secondsAfterBeat - centerSeconds) / widthSeconds;
            return amplitude * Math.exp(-0.5 * normalizedTime ** 2);
        };
        return (
            gaussian(0.095, 0.022, 0.08) +
            gaussian(0.178, 0.009, -0.12) +
            gaussian(0.198, 0.007, 0.82) +
            gaussian(0.222, 0.012, -0.18) +
            gaussian(0.42, 0.062, 0.17)
        );
    }

    #bpWaveform(phase) {
        const sys = 120;
        const dia = 80;
        const upstroke = 1 / (1 + Math.exp(-(phase - 0.11) / 0.018));
        const decay = Math.exp(-Math.max(phase - 0.16, 0) / 0.36);
        const notch = -5.5 * Math.exp(-0.5 * ((phase - 0.33) / 0.018) ** 2);
        const rebound = 3.2 * Math.exp(-0.5 * ((phase - 0.37) / 0.026) ** 2);
        return dia + (sys - dia) * upstroke * decay + notch + rebound;
    }

    #scaledBpPressure(templatePressure) {
        const normalized = clamp((templatePressure - 80) / 40, 0, 1.25);
        return this.waveDiastolic + normalized * (this.waveSystolic - this.waveDiastolic);
    }

    #createEcgTemplate() {
        const templateDurationSeconds = 0.62;
        const sampleCount = Math.round(templateDurationSeconds * this.ecgSampleRate);
        const arr = [];
        for (let i = 0; i < sampleCount; i++) {
            arr.push(this.#ecgWaveformAtTime(i / this.ecgSampleRate));
        }
        return arr;
    }

    #createBpTemplate() {
        const arr = [];
        for (let i = 0; i < this.bpSampleRate; i++) {
            const phase = i / this.bpSampleRate;
            arr.push(this.#bpWaveform(phase));
        }
        return arr;
    }

    #drawEcg() {
        const ctx = this.ecgCtx;
        const state = this.#prepareCanvas(this.ecgCanvas, ctx, this.ecgCanvasState);
        const w = state.w;
        const h = state.h;
        const len = this.ecgData.length;
        this.#clearTracePanel(ctx, w, h, state.backgroundGradient);
        const baseline = h * ECG_BASELINE;
        const gain = h * ECG_TRACE_GAIN;
        this.#drawBaseline(ctx, w, h, baseline, 'rgba(82, 118, 102, 0.32)');
        ctx.beginPath();
        let pathStarted = false;
        const oldestSampleNumber = this.ecgTotalSamples - len;
        const presentationWindowStart = this.ecgPresentationCursor - len;
        for (let logicalIndex = 0; logicalIndex < len; logicalIndex++) {
            let dataIndex = this.ecgCursor + logicalIndex;
            if (dataIndex >= len) dataIndex -= len;
            const sampleNumber = oldestSampleNumber + logicalIndex;
            const x = (sampleNumber - presentationWindowStart) / (len - 1) * w;
            if (x < 0 || x > w) continue;
            const y = baseline - this.ecgData[dataIndex] * gain;
            if (!pathStarted) {
                ctx.moveTo(x, y);
                pathStarted = true;
            }
            else ctx.lineTo(x, y);
        }
        this.#strokeTrace(ctx, '#39e75f', 1.35);
        this.#drawNowMarker(ctx, w, h, state.markerGradient, '#39e75f');
    }

    #drawBp() {
        const ctx = this.bpCtx;
        const state = this.#prepareCanvas(this.bpCanvas, ctx, this.bpCanvasState);
        const w = state.w;
        const h = state.h;
        const len = this.bpData.length;
        this.#clearTracePanel(ctx, w, h, state.backgroundGradient);
        this.#drawBaseline(ctx, w, h, h - 45 / 85 * h, 'rgba(120, 88, 88, 0.3)');
        ctx.beginPath();
        let pathStarted = false;
        const oldestSampleNumber = this.bpTotalSamples - len;
        const presentationWindowStart = this.bpPresentationCursor - len;
        for (let logicalIndex = 0; logicalIndex < len; logicalIndex++) {
            let dataIndex = this.bpCursor + logicalIndex;
            if (dataIndex >= len) dataIndex -= len;
            const pressure = this.bpData[dataIndex];
            const sampleNumber = oldestSampleNumber + logicalIndex;
            const x = (sampleNumber - presentationWindowStart) / (len - 1) * w;
            if (x < 0 || x > w) continue;
            const y = h - (pressure - 55) / 85 * h;
            if (!pathStarted) {
                ctx.moveTo(x, y);
                pathStarted = true;
            }
            else ctx.lineTo(x, y);
        }
        this.#strokeTrace(ctx, '#f04d4d', 1.35);
        this.#drawNowMarker(ctx, w, h, state.markerGradient, '#f04d4d');
    }

    #createCanvasState(topColor, bottomColor) {
        return {
            w: 0,
            h: 0,
            dpr: 0,
            topColor,
            bottomColor,
            backgroundGradient: null,
            markerGradient: null
        };
    }

    #prepareCanvas(canvas, ctx, state) {
        const w = Math.max(1, canvas.clientWidth || canvas.width);
        const h = Math.max(1, canvas.clientHeight || canvas.height);
        const dpr = Math.min(window.devicePixelRatio || 1, MONITOR_DPR_LIMIT);
        const targetWidth = Math.round(w * dpr);
        const targetHeight = Math.round(h * dpr);
        if (canvas.width !== targetWidth || canvas.height !== targetHeight) {
            canvas.width = targetWidth;
            canvas.height = targetHeight;
        }
        ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
        if (state.w !== w || state.h !== h || state.dpr !== dpr || !state.backgroundGradient) {
            const backgroundGradient = ctx.createLinearGradient(0, 0, 0, h);
            backgroundGradient.addColorStop(0, state.topColor);
            backgroundGradient.addColorStop(1, state.bottomColor);
            const markerX = w - 10.5;
            const markerGradient = ctx.createLinearGradient(markerX - 20, 0, markerX + 4, 0);
            markerGradient.addColorStop(0, 'rgba(255,255,255,0)');
            markerGradient.addColorStop(1, 'rgba(210,220,218,0.12)');
            state.w = w;
            state.h = h;
            state.dpr = dpr;
            state.backgroundGradient = backgroundGradient;
            state.markerGradient = markerGradient;
        }
        return state;
    }

    #clearTracePanel(ctx, w, h, backgroundGradient) {
        ctx.clearRect(0, 0, w, h);
        ctx.fillStyle = backgroundGradient;
        ctx.fillRect(0, 0, w, h);
        this.#drawGrid(ctx, w, h);
    }

    #drawGrid(ctx, w, h) {
        ctx.save();
        ctx.strokeStyle = 'rgba(88, 112, 106, 0.16)';
        ctx.lineWidth = 1;
        for (let x = 0.5; x < w; x += 32) {
            ctx.beginPath();
            ctx.moveTo(x, 0);
            ctx.lineTo(x, h);
            ctx.stroke();
        }
        for (let y = 0.5; y < h; y += 24) {
            ctx.beginPath();
            ctx.moveTo(0, y);
            ctx.lineTo(w, y);
            ctx.stroke();
        }
        ctx.strokeStyle = 'rgba(88, 112, 106, 0.26)';
        for (let x = 0.5; x < w; x += 160) {
            ctx.beginPath();
            ctx.moveTo(x, 0);
            ctx.lineTo(x, h);
            ctx.stroke();
        }
        ctx.restore();
    }

    #drawBaseline(ctx, w, h, y, color) {
        ctx.save();
        ctx.strokeStyle = color;
        ctx.setLineDash(this.baselineDash);
        ctx.lineWidth = 1;
        ctx.beginPath();
        ctx.moveTo(0, y);
        ctx.lineTo(w, y);
        ctx.stroke();
        ctx.restore();
    }

    #strokeTrace(ctx, color, width) {
        ctx.save();
        ctx.lineJoin = 'round';
        ctx.lineCap = 'round';
        ctx.strokeStyle = color;
        ctx.lineWidth = width;
        ctx.stroke();
        ctx.restore();
    }

    #drawNowMarker(ctx, w, h, markerGradient, color) {
        const x = w - 10.5;
        ctx.save();
        ctx.fillStyle = markerGradient;
        ctx.fillRect(Math.max(0, x - 20), 0, 24, h);
        ctx.strokeStyle = color;
        ctx.globalAlpha = 0.85;
        ctx.lineWidth = 1;
        ctx.beginPath();
        ctx.moveTo(x, 8);
        ctx.lineTo(x, h - 8);
        ctx.stroke();
        ctx.restore();
    }

    #rhythmLabel() {
        if (this.currentHR >= 105) return 'TACHY';
        if (this.currentHR <= 50) return 'BRADY';
        if (this.meanPressure < 65) return 'LOW MAP';
        return 'SINUS';
    }

    #clockLabel() {
        const totalSeconds = Math.floor(this.time);
        if (totalSeconds === this.lastClockSecond) return this.clockLabel;
        const minutes = Math.floor(totalSeconds / 60).toString().padStart(2, '0');
        const seconds = (totalSeconds % 60).toString().padStart(2, '0');
        this.lastClockSecond = totalSeconds;
        this.clockLabel = `${minutes}:${seconds}`;
        return this.clockLabel;
    }
}
