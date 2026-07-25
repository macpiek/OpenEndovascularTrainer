export class AutomaticWithdrawalController {
    constructor({ emptyThresholdCm = 0.05 } = {}) {
        this.emptyThresholdCm = emptyThresholdCm;
        this.insertedCm = 0;
        this.active = false;
    }

    updateLength(insertedCm) {
        this.insertedCm = Math.max(0, Number.isFinite(insertedCm) ? insertedCm : 0);
        if (this.insertedCm <= this.emptyThresholdCm) this.active = false;
        return this;
    }

    toggle() {
        if (this.active) {
            this.active = false;
        } else if (this.insertedCm > this.emptyThresholdCm) {
            this.active = true;
        }
        return this.active;
    }

    cancel() {
        this.active = false;
        return this;
    }

    get command() {
        return this.active ? -1 : 0;
    }

    get disabled() {
        return !this.active && this.insertedCm <= this.emptyThresholdCm;
    }
}
