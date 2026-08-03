function captureResult(ok, reason = '', details = {}) {
    return { ok, reason, ...details };
}

const DSA_BACKGROUND_RED = 0.94 * (0xeb / 0xff) * 0.992 * 255;
export const DEFAULT_DSA_PREPARATION_DELAY_MS = 650;

/**
 * Scores the red/luma channel of a rendered DSA frame. The subtraction view
 * has a bright, nearly uniform field and dark iodine-filled vessels. Summing
 * thresholded darkness rewards both vessel coverage and iodine density while
 * ignoring detector noise and the collimated area outside the X-ray beam.
 */
export function scoreDsaFrameRedChannel(
    redChannel,
    width,
    height,
    { collimation = 0.08, sampleStep = 2 } = {}
) {
    if (
        !redChannel ||
        width <= 0 ||
        height <= 0 ||
        redChannel.length < width * height
    ) return 0;

    const step = Math.max(1, Math.round(sampleStep));
    const aspect = width / Math.max(1, height);
    const halfSize = Math.max(
        0.08,
        1 - Math.min(1, Math.max(0, collimation)) * 1.35
    );
    let accumulatedSignal = 0;
    let occupiedSamples = 0;
    let sampleCount = 0;

    for (let y = Math.floor(step / 2); y < height; y += step) {
        const centeredY = (y + 0.5) / height * 2 - 1;
        for (let x = Math.floor(step / 2); x < width; x += step) {
            const centeredX = (x + 0.5) / width * 2 - 1;
            const squareX = aspect >= 1
                ? centeredX * aspect
                : centeredX;
            const squareY = aspect >= 1
                ? centeredY
                : centeredY / Math.max(0.001, aspect);
            if (
                Math.abs(squareX) > halfSize * 0.985 ||
                Math.abs(squareY) > halfSize * 0.985
            ) continue;

            sampleCount++;
            const red = redChannel[x + y * width];
            const darkness = Math.max(
                0,
                (DSA_BACKGROUND_RED - red) / DSA_BACKGROUND_RED
            );
            if (darkness <= 0.03) continue;
            const vesselSignal = (darkness - 0.03) / 0.97;
            accumulatedSignal += Math.pow(vesselSignal, 1.35);
            occupiedSamples++;
        }
    }

    if (!sampleCount) return 0;
    const densityScore = accumulatedSignal / sampleCount;
    const coverageScore = occupiedSamples / sampleCount;
    return densityScore * 0.82 + coverageScore * 0.18;
}

/**
 * Scores the projected iodine buffer before detector noise is added. This is
 * the authoritative automatic-selection metric: it measures the amount and
 * density of contrast visible in the current projection without mistaking
 * quantum mottle, fixed-pattern noise, bones, or devices for filled vessels.
 */
export function scoreProjectedContrastRgba(
    rgba,
    width,
    height,
    { collimation = 0.08, sampleStep = 1 } = {}
) {
    if (
        !rgba ||
        width <= 0 ||
        height <= 0 ||
        rgba.length < width * height * 4
    ) return 0;
    const step = Math.max(1, Math.round(sampleStep));
    const aspect = width / Math.max(1, height);
    const halfSize = Math.max(
        0.08,
        1 - Math.min(1, Math.max(0, collimation)) * 1.35
    );
    let projectedSignal = 0;
    let occupiedSamples = 0;
    let sampleCount = 0;
    for (let y = Math.floor(step / 2); y < height; y += step) {
        const centeredY = (y + 0.5) / height * 2 - 1;
        for (let x = Math.floor(step / 2); x < width; x += step) {
            const centeredX = (x + 0.5) / width * 2 - 1;
            const squareX = aspect >= 1
                ? centeredX * aspect
                : centeredX;
            const squareY = aspect >= 1
                ? centeredY
                : centeredY / Math.max(0.001, aspect);
            if (
                Math.abs(squareX) > halfSize * 0.985 ||
                Math.abs(squareY) > halfSize * 0.985
            ) continue;
            sampleCount++;
            const offset = (x + y * width) * 4;
            const opticalSignal = Math.max(
                rgba[offset],
                rgba[offset + 1],
                rgba[offset + 2]
            ) / 255;
            if (opticalSignal <= 0.002) continue;
            const detectorSignal =
                1 - Math.exp(-opticalSignal * 5 * 1.45);
            projectedSignal += Math.pow(detectorSignal, 0.85);
            if (detectorSignal >= 0.035) occupiedSamples++;
        }
    }
    if (!sampleCount) return 0;
    return projectedSignal / sampleCount * 0.9 +
        occupiedSamples / sampleCount * 0.1;
}

function cloneFrame(frame) {
    return { ...frame };
}

function cloneSequence(sequence) {
    return {
        ...sequence,
        frames: sequence.frames.map(cloneFrame)
    };
}

/**
 * Owns the acquisition state for DSA and fluoroscopic roadmapping. GPU frame
 * textures remain in the simulator; this class stores stable sequence/frame
 * identifiers and keeps the workflow deterministic and independently testable.
 */
export class DsaRoadmapState {
    constructor({
        maxSequences = 4,
        maxFramesPerSequence = 120,
        preparationDelayMs = DEFAULT_DSA_PREPARATION_DELAY_MS
    } = {}) {
        this.maxSequences = Math.max(2, Math.round(maxSequences));
        this.maxFramesPerSequence = Math.max(
            1,
            Math.round(maxFramesPerSequence)
        );
        this.maskValid = false;
        this.maskCapturePending = false;
        this.maskCaptureNotBeforeMs = null;
        this.preparationDelayMs = Math.max(0, Number(preparationDelayMs) || 0);
        this.dsaEnabled = false;
        this.roadmapValid = false;
        this.roadmapCapturePending = false;
        this.roadmapEnabled = false;
        this.acquisitionRevision = null;
        this.recording = false;
        this.recordingSequenceId = null;
        this.sequences = [];
        this.selectedSequenceId = null;
        this.selectedFrameIndex = null;
        this.cinePlaying = false;
        this.cineSequenceId = null;
        this.cineFrameIndex = null;
        this.cinePlaybackStartedAtMs = null;
        this.cineElapsedMs = 0;
        this.nextSequenceId = 1;
        this._roadmapWasEnabledBeforeRecording = false;
        this.status = 'Hold R to record a DSA sequence';
    }

    requestMaskCapture({
        contrastVisible = false,
        notBeforeMs = null
    } = {}) {
        this.maskCapturePending = true;
        this.maskCaptureNotBeforeMs = Number.isFinite(notBeforeMs)
            ? notBeforeMs
            : null;
        this.roadmapCapturePending = false;
        this.dsaEnabled = false;
        if (this.maskCaptureNotBeforeMs !== null) {
            this.status = 'C-arm preparing · new DSA mask will be acquired before recording';
        } else {
            this.status = contrastVisible
                ? 'Acquiring a new mask now · residual contrast is allowed'
                : 'Acquiring mask on the next X-ray pulse';
        }
        return captureResult(true);
    }

    isMaskCaptureReady({ nowMs = Date.now() } = {}) {
        return this.maskCapturePending && (
            this.maskCaptureNotBeforeMs === null ||
            nowMs >= this.maskCaptureNotBeforeMs
        );
    }

    failMaskCapture(reason) {
        this.maskCapturePending = false;
        this.maskCaptureNotBeforeMs = null;
        this.status = reason || 'Mask acquisition failed';
    }

    markMaskCaptured(revision = 0, { nowMs = Date.now() } = {}) {
        if (!this.isMaskCaptureReady({ nowMs })) return false;
        this.maskCapturePending = false;
        this.maskCaptureNotBeforeMs = null;
        this.maskValid = true;
        this.dsaEnabled = true;
        this.roadmapEnabled = false;
        this.acquisitionRevision = revision;
        const sequence = this._recordingSequence();
        if (sequence) sequence.maskCapturedAtMs = nowMs;
        this.status = this.recording
            ? 'New mask acquired · recording DSA · release R to save'
            : 'Mask acquired · DSA active';
        return true;
    }

    toggleDsa() {
        if (!this.maskValid) {
            this.status = 'Acquire a DSA mask first';
            return captureResult(false, this.status);
        }
        this.dsaEnabled = !this.dsaEnabled;
        if (this.dsaEnabled) this.roadmapEnabled = false;
        this.status = this.dsaEnabled
            ? 'DSA active'
            : this.roadmapEnabled
                ? 'Live fluoroscopy · roadmap active'
                : 'DSA paused · live fluoroscopy';
        return captureResult(true);
    }

    requestRoadmapCapture({ contrastVisible = false } = {}) {
        if (!this.maskValid) {
            this.status = 'Acquire a DSA mask first';
            return captureResult(false, this.status);
        }
        if (!contrastVisible) {
            this.status = 'Inject contrast before capturing the roadmap';
            return captureResult(false, this.status);
        }
        this.roadmapCapturePending = true;
        this.status = 'Capturing current DSA frame as roadmap';
        return captureResult(true);
    }

    failRoadmapCapture(reason) {
        this.roadmapCapturePending = false;
        this.status = reason || 'Roadmap capture failed';
    }

    markRoadmapCaptured(revision = 0) {
        if (
            !this.maskValid ||
            this.acquisitionRevision !== revision
        ) {
            this.failRoadmapCapture(
                'C-arm position changed · acquire a mask for this view'
            );
            return false;
        }
        this.roadmapCapturePending = false;
        this.roadmapValid = true;
        this.roadmapEnabled = true;
        this.dsaEnabled = false;
        this.selectedSequenceId = null;
        this.selectedFrameIndex = null;
        this.status = 'Current DSA frame selected as roadmap';
        return true;
    }

    startSequenceRecording({
        revision = 0,
        contrastVisible = false,
        startedAtMs = Date.now()
    } = {}) {
        if (this.recording) {
            return captureResult(
                true,
                this.status,
                { sequenceId: this.recordingSequenceId }
            );
        }

        this._resetCinePlayback();

        this._roadmapWasEnabledBeforeRecording = this.roadmapEnabled;
        this.maskValid = false;
        this.acquisitionRevision = null;
        this.roadmapEnabled = false;
        this.requestMaskCapture({
            contrastVisible,
            notBeforeMs: startedAtMs + this.preparationDelayMs
        });

        const sequence = {
            id: this.nextSequenceId++,
            revision,
            startedAtMs,
            maskCapturedAtMs: null,
            endedAtMs: null,
            complete: false,
            bestFrameIndex: null,
            bestScore: 0,
            selectedFrameIndex: null,
            frames: []
        };
        this.sequences.push(sequence);
        this.recording = true;
        this.recordingSequenceId = sequence.id;
        const evictedSequenceIds = this._pruneSequences();
        this.status = `C-arm preparing for DSA · new mask in ${(
            this.preparationDelayMs / 1000
        ).toFixed(1)} s · keep holding R`;
        return captureResult(true, '', {
            sequenceId: sequence.id,
            evictedSequenceIds,
            preparationDelayMs: this.preparationDelayMs,
            maskCaptureNotBeforeMs: this.maskCaptureNotBeforeMs
        });
    }

    appendRecordingFrame({
        contrastScore = 0,
        capturedAtMs = Date.now()
    } = {}) {
        const sequence = this._recordingSequence();
        if (!this.recording || !sequence || !this.maskValid) {
            return captureResult(false, 'DSA recording is not ready');
        }
        const frameIndex = sequence.frames.length;
        const score = Number.isFinite(contrastScore)
            ? Math.max(0, contrastScore)
            : 0;
        const frame = {
            index: frameIndex,
            storageKey: `${sequence.id}:${frameIndex}`,
            capturedAtMs,
            contrastScore: score
        };
        sequence.frames.push(frame);
        if (
            sequence.bestFrameIndex === null ||
            score > sequence.bestScore
        ) {
            sequence.bestFrameIndex = frameIndex;
            sequence.bestScore = score;
        }
        this.status = `Recording DSA · ${sequence.frames.length} frame${
            sequence.frames.length === 1 ? '' : 's'
        } · release R to save`;
        return captureResult(true, '', {
            sequenceId: sequence.id,
            frameIndex,
            storageKey: frame.storageKey,
            isBest: sequence.bestFrameIndex === frameIndex,
            shouldStop:
                sequence.frames.length >= this.maxFramesPerSequence
        });
    }

    finishSequenceRecording({ endedAtMs = Date.now() } = {}) {
        const sequence = this._recordingSequence();
        if (!this.recording || !sequence) {
            return captureResult(false, 'No DSA sequence is being recorded');
        }
        this.recording = false;
        this.recordingSequenceId = null;

        if (!sequence.frames.length) {
            const cancelledDuringPreparation = this.maskCapturePending;
            this.maskCapturePending = false;
            this.maskCaptureNotBeforeMs = null;
            this.sequences = this.sequences.filter(
                candidate => candidate.id !== sequence.id
            );
            this.dsaEnabled = this.maskValid;
            this.roadmapEnabled =
                this.roadmapValid && this._roadmapWasEnabledBeforeRecording;
            this.status = cancelledDuringPreparation
                ? 'DSA preparation cancelled · hold R until the new mask is acquired'
                : 'No DSA frame recorded · hold R through an X-ray pulse';
            return captureResult(false, this.status, {
                discardedSequenceId: sequence.id
            });
        }

        sequence.complete = true;
        sequence.endedAtMs = endedAtMs;
        const selected = this.selectRoadmapFrame(
            sequence.id,
            sequence.bestFrameIndex,
            { automatic: true }
        );
        return captureResult(selected.ok, selected.reason, {
            sequenceId: sequence.id,
            frameIndex: sequence.bestFrameIndex,
            storageKey:
                sequence.frames[sequence.bestFrameIndex]?.storageKey ?? null,
            automatic: true
        });
    }

    selectSequence(sequenceId) {
        const sequence = this._sequence(sequenceId);
        if (!sequence?.complete || !sequence.frames.length) {
            this.status = 'The selected DSA sequence has no saved frames';
            return captureResult(false, this.status);
        }
        const frameIndex = sequence.selectedFrameIndex ??
            sequence.bestFrameIndex ?? 0;
        return this.selectRoadmapFrame(sequence.id, frameIndex);
    }

    selectRoadmapFrame(
        sequenceId,
        frameIndex,
        { automatic = false } = {}
    ) {
        const sequence = this._sequence(sequenceId);
        const index = Math.round(frameIndex);
        const frame = sequence?.frames[index];
        if (!sequence?.complete || !frame) {
            this.status = 'The selected DSA frame is unavailable';
            return captureResult(false, this.status);
        }
        sequence.selectedFrameIndex = index;
        this.selectedSequenceId = sequence.id;
        this.selectedFrameIndex = index;
        this.roadmapValid = true;
        this.roadmapEnabled = true;
        this.dsaEnabled = false;
        this.status = automatic
            ? `DSA ${sequence.id} saved · best-filled frame ${index + 1}/${sequence.frames.length} selected automatically`
            : `DSA ${sequence.id} · frame ${index + 1}/${sequence.frames.length} selected as roadmap`;
        return captureResult(true, '', {
            sequenceId: sequence.id,
            frameIndex: index,
            storageKey: frame.storageKey,
            automatic
        });
    }

    useBestFrame(sequenceId = this.selectedSequenceId) {
        const sequence = this._sequence(sequenceId) ||
            [...this.sequences].reverse().find(candidate => candidate.complete);
        if (!sequence || sequence.bestFrameIndex === null) {
            this.status = 'No completed DSA sequence is available';
            return captureResult(false, this.status);
        }
        return this.selectRoadmapFrame(
            sequence.id,
            sequence.bestFrameIndex,
            { automatic: true }
        );
    }

    playCine(
        sequenceId = this.cineSequenceId ?? this.selectedSequenceId,
        { nowMs = Date.now(), restart = false } = {}
    ) {
        const sequence = this._sequence(sequenceId) ||
            [...this.sequences].reverse().find(candidate => candidate.complete);
        if (!sequence?.complete || !sequence.frames.length) {
            this.status = 'No completed DSA sequence is available for cine';
            return captureResult(false, this.status);
        }
        const sameSequence = this.cineSequenceId === sequence.id;
        if (!sameSequence || restart || this.cineFrameIndex === null) {
            this.cineElapsedMs = 0;
            this.cineFrameIndex = 0;
        }
        this.cineSequenceId = sequence.id;
        this.cinePlaying = true;
        this.cinePlaybackStartedAtMs = nowMs - this.cineElapsedMs;
        this.status = `Playing DSA ${sequence.id} as cine`;
        return captureResult(true, '', {
            sequenceId: sequence.id,
            frameIndex: this.cineFrameIndex
        });
    }

    pauseCine({ nowMs = Date.now() } = {}) {
        if (this.cineSequenceId === null) {
            return captureResult(false, 'No DSA cine is open');
        }
        this.advanceCine({ nowMs });
        this.cinePlaying = false;
        this.cinePlaybackStartedAtMs = null;
        this.status = `DSA ${this.cineSequenceId} cine paused`;
        return captureResult(true, '', {
            sequenceId: this.cineSequenceId,
            frameIndex: this.cineFrameIndex
        });
    }

    toggleCine(sequenceId = this.cineSequenceId, { nowMs = Date.now() } = {}) {
        const numericSequenceId = sequenceId === null || sequenceId === undefined
            ? null
            : Number(sequenceId);
        if (
            this.cinePlaying &&
            (numericSequenceId === null || numericSequenceId === this.cineSequenceId)
        ) return this.pauseCine({ nowMs });
        return this.playCine(
            numericSequenceId ?? this.cineSequenceId ?? this.selectedSequenceId,
            {
                nowMs,
                restart:
                    numericSequenceId !== null &&
                    numericSequenceId !== this.cineSequenceId
            }
        );
    }

    stopCine() {
        if (this.cineSequenceId === null) {
            return captureResult(false, 'No DSA cine is open');
        }
        const sequenceId = this.cineSequenceId;
        this._resetCinePlayback();
        this.status = this.roadmapEnabled
            ? 'Live fluoroscopy · roadmap active'
            : this.dsaEnabled
                ? 'DSA active'
                : 'Live fluoroscopy';
        return captureResult(true, '', { sequenceId });
    }

    advanceCine({ nowMs = Date.now() } = {}) {
        const sequence = this._sequence(this.cineSequenceId);
        if (!this.cinePlaying || !sequence?.complete || !sequence.frames.length) {
            return captureResult(false, '', { changed: false });
        }
        const timing = this._cineTiming(sequence);
        const elapsed = timing.durationMs > 0
            ? Math.max(0, nowMs - this.cinePlaybackStartedAtMs) % timing.durationMs
            : 0;
        this.cineElapsedMs = elapsed;
        let frameIndex = 0;
        for (let index = 1; index < timing.frameOffsetsMs.length; index++) {
            if (timing.frameOffsetsMs[index] > elapsed) break;
            frameIndex = index;
        }
        const changed = frameIndex !== this.cineFrameIndex;
        this.cineFrameIndex = frameIndex;
        return captureResult(true, '', {
            changed,
            sequenceId: sequence.id,
            frameIndex,
            durationMs: timing.durationMs
        });
    }

    toggleRoadmap() {
        if (!this.roadmapValid) {
            this.status = 'Record DSA or capture a roadmap first';
            return captureResult(false, this.status);
        }
        this.roadmapEnabled = !this.roadmapEnabled;
        if (this.roadmapEnabled) this.dsaEnabled = false;
        this.status = this.roadmapEnabled
            ? 'Live fluoroscopy · roadmap active'
            : this.dsaEnabled
                ? 'DSA active'
                : 'Roadmap hidden · live fluoroscopy';
        return captureResult(true);
    }

    clearRoadmap() {
        this.roadmapCapturePending = false;
        this.roadmapValid = false;
        this.roadmapEnabled = false;
        this.selectedSequenceId = null;
        this.selectedFrameIndex = null;
        this.status = this.dsaEnabled
            ? 'DSA active · roadmap cleared'
            : this.sequences.some(sequence => sequence.complete)
                ? 'Roadmap cleared · saved DSA sequences retained'
                : this.maskValid
                    ? 'Roadmap cleared · DSA mask retained'
                    : 'Hold R to record a DSA sequence';
    }

    invalidateForGeometryRevision(revision = 0) {
        if (
            this.acquisitionRevision === null ||
            this.acquisitionRevision === revision
        ) return false;
        if (this.recording) this.finishSequenceRecording();
        this.maskValid = false;
        this.maskCapturePending = false;
        this.maskCaptureNotBeforeMs = null;
        this.dsaEnabled = false;
        this.roadmapCapturePending = false;
        this.acquisitionRevision = null;
        this.status = this.roadmapValid
            ? 'C-arm moved · roadmap retained · hold R for a new DSA view'
            : 'C-arm moved · hold R to record DSA for this view';
        return true;
    }

    invalidate(
        reason = 'Detector geometry changed · acquire a new DSA mask',
        { preserveRoadmap = true } = {}
    ) {
        if (this.recording) this.finishSequenceRecording();
        this.maskValid = false;
        this.maskCapturePending = false;
        this.maskCaptureNotBeforeMs = null;
        this.dsaEnabled = false;
        this.roadmapCapturePending = false;
        this.acquisitionRevision = null;
        if (!preserveRoadmap) {
            this.roadmapValid = false;
            this.roadmapEnabled = false;
            this.selectedSequenceId = null;
            this.selectedFrameIndex = null;
        }
        this.status = reason;
    }

    getSnapshot() {
        return {
            maskValid: this.maskValid,
            maskCapturePending: this.maskCapturePending,
            maskCaptureNotBeforeMs: this.maskCaptureNotBeforeMs,
            preparationDelayMs: this.preparationDelayMs,
            dsaEnabled: this.dsaEnabled,
            roadmapValid: this.roadmapValid,
            roadmapCapturePending: this.roadmapCapturePending,
            roadmapEnabled: this.roadmapEnabled,
            acquisitionRevision: this.acquisitionRevision,
            recording: this.recording,
            recordingSequenceId: this.recordingSequenceId,
            selectedSequenceId: this.selectedSequenceId,
            selectedFrameIndex: this.selectedFrameIndex,
            cinePlaying: this.cinePlaying,
            cineSequenceId: this.cineSequenceId,
            cineFrameIndex: this.cineFrameIndex,
            maxSequences: this.maxSequences,
            maxFramesPerSequence: this.maxFramesPerSequence,
            sequences: this.sequences.map(cloneSequence),
            status: this.status
        };
    }

    _sequence(sequenceId) {
        const numericId = Number(sequenceId);
        return this.sequences.find(sequence => sequence.id === numericId) || null;
    }

    _recordingSequence() {
        return this._sequence(this.recordingSequenceId);
    }

    _cineTiming(sequence) {
        const firstCapturedAtMs = sequence.frames[0]?.capturedAtMs ?? 0;
        const frameOffsetsMs = sequence.frames.map(frame =>
            Math.max(0, (frame.capturedAtMs ?? firstCapturedAtMs) - firstCapturedAtMs)
        );
        const positiveIntervals = [];
        for (let index = 1; index < frameOffsetsMs.length; index++) {
            const interval = frameOffsetsMs[index] - frameOffsetsMs[index - 1];
            if (interval > 0) positiveIntervals.push(interval);
        }
        positiveIntervals.sort((a, b) => a - b);
        const cadenceMs = positiveIntervals.length
            ? positiveIntervals[Math.floor(positiveIntervals.length / 2)]
            : 1000 / 15;
        const durationMs = Math.max(
            cadenceMs,
            (frameOffsetsMs.at(-1) || 0) + cadenceMs
        );
        return { frameOffsetsMs, durationMs };
    }

    _resetCinePlayback() {
        this.cinePlaying = false;
        this.cineSequenceId = null;
        this.cineFrameIndex = null;
        this.cinePlaybackStartedAtMs = null;
        this.cineElapsedMs = 0;
    }

    _pruneSequences() {
        const evicted = [];
        while (this.sequences.length > this.maxSequences) {
            let index = this.sequences.findIndex(sequence =>
                sequence.id !== this.recordingSequenceId &&
                sequence.id !== this.selectedSequenceId
            );
            if (index < 0) {
                index = this.sequences.findIndex(sequence =>
                    sequence.id !== this.recordingSequenceId
                );
            }
            if (index < 0) break;
            const [removed] = this.sequences.splice(index, 1);
            evicted.push(removed.id);
            if (removed.id === this.cineSequenceId) {
                this._resetCinePlayback();
            }
            if (removed.id === this.selectedSequenceId) {
                this.selectedSequenceId = null;
                this.selectedFrameIndex = null;
                this.roadmapValid = false;
                this.roadmapEnabled = false;
            }
        }
        return evicted;
    }
}
