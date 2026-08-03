const EPSILON = 1e-8;

function clamp01(value) {
    return Math.max(0, Math.min(1, value));
}

export function sampleGuidewireResistance(body) {
    let activeContacts = 0;
    let normalReaction = 0;
    let axialReaction = 0;

    for (let index = 0; index < body.wallLambda.length; index++) {
        if (!body.wallActive[index]) continue;
        const lambda = Math.max(0, body.wallLambda[index]);
        if (lambda <= EPSILON) continue;

        const dx = body.x[index + 1] - body.x[index];
        const dy = body.y[index + 1] - body.y[index];
        const dz = body.z[index + 1] - body.z[index];
        const length = Math.hypot(dx, dy, dz);
        if (length <= EPSILON) continue;

        const normalProjection = Math.abs(
            (dx * body.wallNormalX[index] +
                dy * body.wallNormalY[index] +
                dz * body.wallNormalZ[index]) / length
        );
        normalReaction += lambda;
        axialReaction += lambda * (normalProjection + body.wallFriction);
        activeContacts++;
    }

    // Lambdas are solver reactions, not calibrated forces. Combining their
    // axial projection with total normal load makes the indicator respond to
    // wedging and accumulated friction instead of contact count alone.
    const level = clamp01(1 - Math.exp(-(axialReaction * 12 + normalReaction * 1.8)));
    return { level, activeContacts, normalReaction, axialReaction };
}

export class GuidewireResistanceEstimator {
    constructor({ attackSeconds = 0.08, releaseSeconds = 0.32 } = {}) {
        this.attackSeconds = attackSeconds;
        this.releaseSeconds = releaseSeconds;
        this.level = 0;
    }

    reset() {
        this.level = 0;
    }

    update(body, {
        dt,
        command = 0,
        atMaximumInsertion = false
    } = {}) {
        const sample = sampleGuidewireResistance(body);
        const requestedAdvanceAtLimit = command > 0 && atMaximumInsertion;
        const target = requestedAdvanceAtLimit ? 1 : sample.level;
        const timeConstant = target > this.level ? this.attackSeconds : this.releaseSeconds;
        const blend = 1 - Math.exp(-Math.max(0, dt) / Math.max(EPSILON, timeConstant));
        this.level += (target - this.level) * blend;

        let reason = 'Swobodne wsuwanie prowadnika';
        if (requestedAdvanceAtLimit) {
            reason = 'Osiągnięto maksymalną długość prowadnika';
        } else if (this.level >= 0.72) {
            reason = 'Wysoki opór — cofnij lub zmień kierunek';
        } else if (sample.activeContacts > 0 && this.level >= 0.22) {
            reason = 'Kontakt prowadnika ze ścianą naczynia';
        }

        return {
            ...sample,
            level: clamp01(this.level),
            reason
        };
    }
}
