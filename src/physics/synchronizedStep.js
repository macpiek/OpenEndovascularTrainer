/** A publication barrier for independent, cooperatively scheduled solvers.
 * Successful members wait at the barrier; retries never advance them twice.
 * All member inputs are sampled at begin(), before any solver is attempted.
 * Only publish() may expose the new state or advance dependent physics.
 */
export function createSynchronizedStep({members, begin = () => {}, publish}) {
    let stepDt = null, lastMemberResult = null;
    let results = null, cursor = 0, fault = null, epoch = 0;
    return {
        get pending() { return results !== null; },
        get fault() { return fault; },
        get epoch() { return epoch; },
        canAttempt() {
            return !fault && members.some((member, i) => !results?.[i] && member.canAttempt());
        },
        canContinueFrame(resumePending = false) {
            return this.pending && this.canAttempt() &&
                (lastMemberResult?.accepted || resumePending && lastMemberResult?.pending === true);
        },
        beginFrame() { for (const member of members) member.beginFrame(); },
        reset() { lastMemberResult = null; stepDt = null; results = null; fault = null; cursor = 0; epoch++; },
        attempt(dt, budget) {
            if (!this.canAttempt()) return {accepted:false, pending:false};
            try {
                if (!Number.isFinite(dt) || dt <= 0 || stepDt !== null && dt !== stepDt)
                    throw new RangeError('A synchronized step must retain its positive dt');
                if (!results) {
                    stepDt = dt;
                    begin(dt);
                    results = new Array(members.length).fill(null);
                }
                for (let n = 0; n < members.length; n++) {
                    const i = cursor;
                    cursor = (cursor + 1) % members.length;
                    if (results[i] || !members[i].canAttempt()) continue;
                    const result = members[i].attempt(dt, budget);
                    lastMemberResult = result;
                    if (result.accepted) results[i] = result;
                    if (results.every(Boolean)) {
                        // Publication is irreversible. A failing consumer must
                        // stop the clock, never replay a partially published dt.
                        publish(dt, results);
                        results = null;
                        stepDt = null;
                        return {accepted:true, pending:false};
                    }
                    return {accepted:false, pending:this.canAttempt(), result};
                }
                return {accepted:false, pending:false};
            } catch (error) {
                fault = error;
                throw error;
            }
        }
    };
}
