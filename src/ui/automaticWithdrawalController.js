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

// Latched withdrawals belong to a sheath, independently of keyboard focus.
export class AccessAutomaticWithdrawalController {
    constructor(options={}) {
        this.accesses=new Map(['right','left'].map(id=>[id,{
            guidewire:new AutomaticWithdrawalController(options),
            catheter:new AutomaticWithdrawalController(options)
        }]));
    }
    forAccess(id) {
        const access=this.accesses.get(id);
        if(!access)throw new Error(`Unknown withdrawal access: ${id}`);
        return access;
    }
    commands(id) {
        const access=this.forAccess(id);
        return {guidewireAdvance:access.guidewire.command,catheterAdvance:access.catheter.command};
    }
    updateLengths(id,guidewireCm,catheterCm) {
        const access=this.forAccess(id);
        access.guidewire.updateLength(guidewireCm);access.catheter.updateLength(catheterCm);
    }
}
