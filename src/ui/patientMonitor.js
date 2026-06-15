const MONITOR_DPR_LIMIT = 2;
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

        this.ecgSampleRate = 250;
        this.bpSampleRate = 50;

        this.ecgBufferLength = this.ecgSampleRate * 10;
        this.bpBufferLength = this.bpSampleRate * 10;

        this.ecgData = new Array(this.ecgBufferLength).fill(0);
        this.bpData = new Array(this.bpBufferLength).fill(100);

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

    update(dt) {
        this.ecgAccumulator += dt;
        this.bpAccumulator += dt;
        this.time += dt;
        this.cycleTime += dt;

        const ecgStep = 1 / this.ecgSampleRate;
        while (this.ecgAccumulator >= ecgStep) {
            this.ecgAccumulator -= ecgStep;
            const ecg = this.#nextEcgSample();
            this.ecgData.push(ecg);
            while (this.ecgData.length > this.ecgBufferLength) this.ecgData.shift();
        }

        const bpStep = 1 / this.bpSampleRate;
        while (this.bpAccumulator >= bpStep) {
            this.bpAccumulator -= bpStep;
            const phase = (this.cycleTime / this.beatInterval) % 1;
            const index = Math.floor(phase * this.bpTemplate.length);
            const pressure = this.#scaledBpPressure(this.bpTemplate[index]);
            this.bpData.push(pressure);
            while (this.bpData.length > this.bpBufferLength) this.bpData.shift();
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

        this.hrElem.textContent = this.currentHR.toFixed(0);
        this.bpElem.textContent = `${Math.round(this.systolic)}/${Math.round(this.diastolic)}`;
        if (this.spo2Elem) this.spo2Elem.textContent = Math.round(this.spo2).toString();
        if (this.mapElem) this.mapElem.textContent = Math.round(this.meanPressure).toString();
        if (this.rrElem) this.rrElem.textContent = Math.round(this.respiratoryRate).toString();
        if (this.rhythmElem) this.rhythmElem.textContent = this.#rhythmLabel();
        if (this.clockElem) this.clockElem.textContent = this.#clockLabel();

        this.#drawEcg();
        this.#drawBp();
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
        const { w, h } = this.#prepareCanvas(this.ecgCanvas, ctx);
        const len = this.ecgData.length;
        this.#clearTracePanel(ctx, w, h, '#020303', '#000000');
        const baseline = h * ECG_BASELINE;
        const gain = h * ECG_TRACE_GAIN;
        this.#drawBaseline(ctx, w, h, baseline, 'rgba(82, 118, 102, 0.32)');
        ctx.beginPath();
        ctx.moveTo(0, baseline - this.ecgData[0] * gain);
        for (let i = 1; i < len; i++) {
            const x = (i / (len - 1)) * w;
            ctx.lineTo(x, baseline - this.ecgData[i] * gain);
        }
        this.#strokeTrace(ctx, '#39e75f', 1.35);
        this.#drawNowMarker(ctx, w, h, '#39e75f');
    }

    #drawBp() {
        const ctx = this.bpCtx;
        const { w, h } = this.#prepareCanvas(this.bpCanvas, ctx);
        const len = this.bpData.length;
        const mapY = p => h - (p - 55) / 85 * h;
        this.#clearTracePanel(ctx, w, h, '#030202', '#000000');
        this.#drawBaseline(ctx, w, h, mapY(100), 'rgba(120, 88, 88, 0.3)');
        ctx.beginPath();
        ctx.moveTo(0, mapY(this.bpData[0]));
        for (let i = 1; i < len; i++) {
            const x = (i / (len - 1)) * w;
            ctx.lineTo(x, mapY(this.bpData[i]));
        }
        this.#strokeTrace(ctx, '#f04d4d', 1.35);
        this.#drawNowMarker(ctx, w, h, '#f04d4d');
    }

    #prepareCanvas(canvas, ctx) {
        const rect = canvas.getBoundingClientRect();
        const w = Math.max(1, rect.width || canvas.width);
        const h = Math.max(1, rect.height || canvas.height);
        const dpr = Math.min(window.devicePixelRatio || 1, MONITOR_DPR_LIMIT);
        const targetWidth = Math.round(w * dpr);
        const targetHeight = Math.round(h * dpr);
        if (canvas.width !== targetWidth || canvas.height !== targetHeight) {
            canvas.width = targetWidth;
            canvas.height = targetHeight;
        }
        ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
        return { w, h };
    }

    #clearTracePanel(ctx, w, h, topColor, bottomColor) {
        ctx.clearRect(0, 0, w, h);
        const gradient = ctx.createLinearGradient(0, 0, 0, h);
        gradient.addColorStop(0, topColor);
        gradient.addColorStop(1, bottomColor);
        ctx.fillStyle = gradient;
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
        ctx.setLineDash([6, 8]);
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

    #drawNowMarker(ctx, w, h, color) {
        const x = w - 10.5;
        const gradient = ctx.createLinearGradient(x - 20, 0, x + 4, 0);
        gradient.addColorStop(0, 'rgba(255,255,255,0)');
        gradient.addColorStop(1, 'rgba(210,220,218,0.12)');
        ctx.save();
        ctx.fillStyle = gradient;
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
        const minutes = Math.floor(totalSeconds / 60).toString().padStart(2, '0');
        const seconds = (totalSeconds % 60).toString().padStart(2, '0');
        return `${minutes}:${seconds}`;
    }
}
