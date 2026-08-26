const EPSILON = 1e-8;

function clamp01(value) {
    return Math.max(0, Math.min(1, value));
}

export function sampleGuidewireResistance(body, out = null) {
    let activeContacts = 0;
    let normalReaction = 0;
    let axialReaction = 0;

    const firstCollisionSegment = Math.max(
        body.activeStart ?? 0,
        body.collisionStartSegment ?? 0
    );
    const lastCollisionSegment = Math.min(
        (body.activeEnd ?? body.wallLambda.length) - 1,
        body.collisionEndSegment ?? body.wallLambda.length - 1,
        body.wallLambda.length - 1
    );
    for (
        let index = firstCollisionSegment;
        index <= lastCollisionSegment;
        index++
    ) {
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
        const slidingFriction = body.wallKineticFriction ?? body.wallFriction;
        axialReaction += lambda * (normalProjection + slidingFriction);
        activeContacts++;
    }

    // Lambdas are solver reactions, not calibrated forces. The handle should
    // report the axial component needed to advance the wire, not total lateral
    // support from a long wire resting against a curved vessel wall.
    const level = clamp01(1 - Math.exp(-axialReaction * 12));
    const result = out || {};
    result.level = level;
    result.activeContacts = activeContacts;
    result.normalReaction = normalReaction;
    result.axialReaction = axialReaction;
    return result;
}

export class GuidewireResistanceEstimator {
    constructor({ attackSeconds = 0.08, releaseSeconds = 0.32 } = {}) {
        this.attackSeconds = attackSeconds;
        this.releaseSeconds = releaseSeconds;
        this.level = 0;
        this._sample = {};
    }

    reset() {
        this.level = 0;
    }

    update(body, {
        dt,
        command = 0,
        atMaximumInsertion = false
    } = {}, out = null) {
        const sample = sampleGuidewireResistance(body, this._sample);
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

        const result = out || {};
        result.level = clamp01(this.level);
        result.activeContacts = sample.activeContacts;
        result.normalReaction = sample.normalReaction;
        result.axialReaction = sample.axialReaction;
        result.reason = reason;
        return result;
    }
}
