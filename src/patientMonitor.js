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
        let y = 0;
        if (phase < 0.1) {
            y = 0.1 * Math.sin(Math.PI * phase / 0.1); // P wave
        } else if (phase < 0.2) {
            y = 0; // PR segment
        } else if (phase < 0.22) {
            y = -0.15 * (phase - 0.2) / 0.02; // Q wave
        } else if (phase < 0.23) {
            y = 1 - 25 * Math.abs(phase - 0.225); // R wave
        } else if (phase < 0.25) {
            y = -0.15 * (0.25 - phase) / 0.02; // S wave
        } else if (phase < 0.45) {
            y = 0; // ST segment
        } else if (phase < 0.6) {
            y = 0.2 * Math.sin(Math.PI * (phase - 0.45) / 0.15); // T wave
        }
        return y;
    }

    #bpWaveform(phase) {
        const sys = 120;
        const dia = 80;
        let p = dia;
        if (phase < 0.15) {
            // rapid systolic upstroke
            p = dia + (sys - dia) * Math.sin((phase / 0.15) * Math.PI / 2);
        } else if (phase < 0.3) {
            // decline from systole
            p = sys - 20 * ((phase - 0.15) / 0.15);
        } else if (phase < 0.35) {
            // dicrotic notch
            p = 100 - 10 * Math.sin(Math.PI * (phase - 0.3) / 0.05);
        } else {
            // diastolic runoff
            p = 100 - 20 * ((phase - 0.35) / 0.65);
        }
        return p;
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
        ctx.clearRect(0, 0, w, h);
        ctx.beginPath();
        ctx.moveTo(0, h / 2 - this.ecgData[0] * h / 2);
        for (let i = 1; i < len; i++) {
            const x = (i / (len - 1)) * w;
            ctx.lineTo(x, h / 2 - this.ecgData[i] * h / 2);
        }
        ctx.strokeStyle = 'lime';
        ctx.stroke();
    }

    #drawBp() {
        const ctx = this.bpCtx;
        const w = this.bpCanvas.width;
        const h = this.bpCanvas.height;
        const len = this.bpData.length;
        const mapY = p => h - (p - 60) / 80 * h;
        ctx.clearRect(0, 0, w, h);
        ctx.beginPath();
        ctx.moveTo(0, mapY(this.bpData[0]));
        for (let i = 1; i < len; i++) {
            const x = (i / (len - 1)) * w;
            ctx.lineTo(x, mapY(this.bpData[i]));
        }
        ctx.save();
        ctx.strokeStyle = '#ff0000';
        ctx.stroke();
        ctx.restore();
    }
}

