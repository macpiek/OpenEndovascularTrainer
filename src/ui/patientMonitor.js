export class PatientMonitor {
    constructor(ecgCanvas, bpCanvas, hrElem, bpElem) {
        this.ecgCanvas = ecgCanvas;
        this.bpCanvas = bpCanvas;
        this.hrElem = hrElem;
        this.bpElem = bpElem;

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
        this.heartRate = 75; // bpm
        this.beatInterval = 60 / this.heartRate;

        this.ecgAccumulator = 0;
        this.bpAccumulator = 0;

        this.currentHR = this.heartRate;
        this.systolic = 120;
        this.diastolic = 80;
        this.bpMax = 0;
        this.bpMin = Infinity;

        // Precomputed waveforms for one cardiac cycle
        this.ecgTemplate = this.#createEcgTemplate();
        this.bpTemplate = this.#createBpTemplate();
    }

    setHeartRate(hr) {
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
            const phase = (this.cycleTime / this.beatInterval) % 1;
            const index = Math.floor(phase * this.ecgTemplate.length);
            const ecg = this.ecgTemplate[index];
            this.ecgData.push(ecg);
            while (this.ecgData.length > this.ecgBufferLength) this.ecgData.shift();
        }

        const bpStep = 1 / this.bpSampleRate;
        while (this.bpAccumulator >= bpStep) {
            this.bpAccumulator -= bpStep;
            const phase = (this.cycleTime / this.beatInterval) % 1;
            const index = Math.floor(phase * this.bpTemplate.length);
            const pressure = this.bpTemplate[index];
            this.bpData.push(pressure);
            while (this.bpData.length > this.bpBufferLength) this.bpData.shift();
            if (pressure > this.bpMax) this.bpMax = pressure;
            if (pressure < this.bpMin) this.bpMin = pressure;
        }

        if (this.cycleTime >= this.beatInterval) {
            this.currentHR = 60 / this.beatInterval;
            this.systolic = this.bpMax;
            this.diastolic = this.bpMin;
            this.cycleTime -= this.beatInterval;
            this.bpMax = 0;
            this.bpMin = Infinity;
        }

        this.hrElem.textContent = this.currentHR.toFixed(0);
        this.bpElem.textContent = `${Math.round(this.systolic)}/${Math.round(this.diastolic)}`;

        this.#drawEcg();
        this.#drawBp();
    }

    #ecgWaveform(phase) {
        const gaussian = (center, width, amplitude) => amplitude * Math.exp(-0.5 * ((phase - center) / width) ** 2);
        return (
            gaussian(0.12, 0.025, 0.12) +
            gaussian(0.215, 0.008, -0.18) +
            gaussian(0.235, 0.006, 1.05) +
            gaussian(0.258, 0.012, -0.28) +
            gaussian(0.5, 0.055, 0.26)
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

    #createEcgTemplate() {
        const arr = [];
        for (let i = 0; i < this.ecgSampleRate; i++) {
            const phase = i / this.ecgSampleRate;
            arr.push(this.#ecgWaveform(phase));
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
        const w = this.ecgCanvas.width;
        const h = this.ecgCanvas.height;
        const len = this.ecgData.length;
        this.#clearTracePanel(ctx, w, h);
        ctx.beginPath();
        ctx.moveTo(0, h * 0.58 - this.ecgData[0] * h * 0.42);
        for (let i = 1; i < len; i++) {
            const x = (i / (len - 1)) * w;
            ctx.lineTo(x, h * 0.58 - this.ecgData[i] * h * 0.42);
        }
        this.#strokeTrace(ctx, '#2dff68', 1.35);
    }

    #drawBp() {
        const ctx = this.bpCtx;
        const w = this.bpCanvas.width;
        const h = this.bpCanvas.height;
        const len = this.bpData.length;
        const mapY = p => h - (p - 55) / 85 * h;
        this.#clearTracePanel(ctx, w, h);
        ctx.beginPath();
        ctx.moveTo(0, mapY(this.bpData[0]));
        for (let i = 1; i < len; i++) {
            const x = (i / (len - 1)) * w;
            ctx.lineTo(x, mapY(this.bpData[i]));
        }
        this.#strokeTrace(ctx, '#ff3a3a', 1.35);
    }

    #clearTracePanel(ctx, w, h) {
        ctx.clearRect(0, 0, w, h);
        const gradient = ctx.createLinearGradient(0, 0, 0, h);
        gradient.addColorStop(0, '#050707');
        gradient.addColorStop(1, '#010202');
        ctx.fillStyle = gradient;
        ctx.fillRect(0, 0, w, h);
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
}
