export class PatientMonitor {
    constructor(ecgCanvas, bpCanvas, hrElem, bpElem) {
        this.ecgCanvas = ecgCanvas;
        this.bpCanvas = bpCanvas;
        this.hrElem = hrElem;
        this.bpElem = bpElem;

        this.ecgCtx = ecgCanvas.getContext('2d');
        this.bpCtx = bpCanvas.getContext('2d');

        this.ecgData = new Array(ecgCanvas.width).fill(0);
        this.bpData = new Array(bpCanvas.width).fill(100);

        this.time = 0;
        this.cycleTime = 0;
        this.heartRate = 75; // bpm
        this.beatInterval = 60 / this.heartRate;

        this.ecgSampleRate = 250;
        this.bpSampleRate = 50;
        this.ecgAccumulator = 0;
        this.bpAccumulator = 0;

        this.currentHR = this.heartRate;
        this.systolic = 120;
        this.diastolic = 80;
        this.bpMax = 0;
        this.bpMin = Infinity;
    }

    update(dt) {
        this.ecgAccumulator += dt;
        this.bpAccumulator += dt;
        this.time += dt;
        this.cycleTime += dt;

        const ecgStep = 1 / this.ecgSampleRate;
        while (this.ecgAccumulator >= ecgStep) {
            this.ecgAccumulator -= ecgStep;
            const phase = this.cycleTime / this.beatInterval;
            const ecg = this.#generateEcgSample(phase);
            this.ecgData.shift();
            this.ecgData.push(ecg);
        }

        const bpStep = 1 / this.bpSampleRate;
        while (this.bpAccumulator >= bpStep) {
            this.bpAccumulator -= bpStep;
            const phase = this.cycleTime / this.beatInterval;
            const pressure = this.#generateBpSample(phase);
            this.bpData.shift();
            this.bpData.push(pressure);
            if (pressure > this.bpMax) this.bpMax = pressure;
            if (pressure < this.bpMin) this.bpMin = pressure;
        }

        if (this.cycleTime >= this.beatInterval) {
            this.currentHR = 60 / this.beatInterval;
            this.systolic = this.bpMax;
            this.diastolic = this.bpMin;
            this.cycleTime -= this.beatInterval;
            this.heartRate = 70 + Math.random() * 10;
            this.beatInterval = 60 / this.heartRate;
            this.bpMax = 0;
            this.bpMin = Infinity;
        }

        this.hrElem.textContent = this.currentHR.toFixed(0);
        this.bpElem.textContent = `${Math.round(this.systolic)}/${Math.round(this.diastolic)}`;

        this.#drawEcg();
        this.#drawBp();
    }

    #generateEcgSample(phase) {
        let y = 0;
        if (phase < 0.1) {
            y = 0.1 * Math.sin(Math.PI * phase / 0.1);
        } else if (phase < 0.2) {
            y = 0;
        } else if (phase < 0.22) {
            y = -0.15 * (phase - 0.2) / 0.02;
        } else if (phase < 0.23) {
            y = 1 - 25 * Math.abs(phase - 0.225);
        } else if (phase < 0.25) {
            y = -0.15 * (0.25 - phase) / 0.02;
        } else if (phase < 0.45) {
            y = 0;
        } else if (phase < 0.6) {
            y = 0.2 * Math.sin(Math.PI * (phase - 0.45) / 0.15);
        }
        return y + (Math.random() - 0.5) * 0.05;
    }

    #generateBpSample(phase) {
        return 100 + 20 * Math.sin(2 * Math.PI * phase) + (Math.random() - 0.5) * 2;
    }

    #drawEcg() {
        const ctx = this.ecgCtx;
        const w = this.ecgCanvas.width;
        const h = this.ecgCanvas.height;
        ctx.clearRect(0, 0, w, h);
        ctx.beginPath();
        ctx.moveTo(0, h / 2 - this.ecgData[0] * h / 2);
        for (let i = 1; i < w; i++) {
            ctx.lineTo(i, h / 2 - this.ecgData[i] * h / 2);
        }
        ctx.strokeStyle = 'lime';
        ctx.stroke();
    }

    #drawBp() {
        const ctx = this.bpCtx;
        const w = this.bpCanvas.width;
        const h = this.bpCanvas.height;
        const mapY = p => h - (p - 60) / 80 * h;
        ctx.clearRect(0, 0, w, h);
        ctx.beginPath();
        ctx.moveTo(0, mapY(this.bpData[0]));
        for (let i = 1; i < w; i++) {
            ctx.lineTo(i, mapY(this.bpData[i]));
        }
        ctx.strokeStyle = 'yellow';
        ctx.stroke();
    }
}

