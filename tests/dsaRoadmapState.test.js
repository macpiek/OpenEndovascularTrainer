import assert from 'node:assert/strict';
import {
    DsaRoadmapState,
    scoreDsaFrameRedChannel,
    scoreProjectedContrastRgba
} from '../src/imaging/dsaRoadmapState.js';
import {
    dsaArchiveDimensions,
    dsaScoreDimensions
} from '../src/imaging/dsaCaptureSizing.js';
import { composeDsaFrameRangeRedChannels } from '../src/imaging/dsaFrameComposite.js';
import { DSA_NEUTRAL_RED } from '../src/imaging/dsaDisplayParameters.js';
import {
    MAX_COLLIMATION,
    collimatedFieldReduction,
    createCollimatorFieldPolygon,
    createCollimatorGeometry,
    isDetectorPointInsideCollimator,
    normalizeCollimatorSettings,
    sideCollimatorBoundaryXAtY
} from '../src/imaging/collimatorGeometry.js';

const state = new DsaRoadmapState({
    maxSequences: 3,
    maxFramesPerSequence: 6
});
let snapshot = state.getSnapshot();
assert.equal(snapshot.maskValid, false);
assert.equal(snapshot.recording, false);
assert.deepEqual(snapshot.sequences, []);
assert.equal(snapshot.status, 'Hold R to record a DSA sequence');

// A new acquisition is allowed immediately even if iodine is still visible.
const contaminatedMask = state.requestMaskCapture({ contrastVisible: true });
assert.equal(contaminatedMask.ok, true);
assert.equal(state.getSnapshot().maskCapturePending, true);
assert.match(state.getSnapshot().status, /residual contrast is allowed/i);
state.markMaskCaptured(7);
assert.equal(state.getSnapshot().maskValid, true);
assert.equal(state.getSnapshot().dsaEnabled, true);
assert.equal(state.getSnapshot().acquisitionRevision, 7);

const cancelledPreparation = new DsaRoadmapState({ preparationDelayMs: 650 });
cancelledPreparation.startSequenceRecording({ revision: 2, startedAtMs: 0 });
const cancelledResult = cancelledPreparation.finishSequenceRecording({
    endedAtMs: 200
});
assert.equal(cancelledResult.ok, false);
assert.equal(cancelledPreparation.getSnapshot().maskCapturePending, false);
assert.equal(cancelledPreparation.getSnapshot().sequences.length, 0);
assert.match(cancelledPreparation.getSnapshot().status, /preparation cancelled/i);

// Every DSA recording first pauses for C-arm preparation and requires a fresh
// mask even when the previous mask is valid for the same projection.
const firstRecording = state.startSequenceRecording({
    revision: 7,
    contrastVisible: true,
    startedAtMs: 100
});
assert.equal(firstRecording.ok, true);
assert.equal(state.getSnapshot().recording, true);
assert.equal(state.getSnapshot().maskCapturePending, true);
assert.equal(firstRecording.preparationDelayMs, 650);
assert.equal(firstRecording.maskCaptureNotBeforeMs, 750);
assert.equal(state.isMaskCaptureReady({ nowMs: 749 }), false);
assert.equal(state.markMaskCaptured(7, { nowMs: 749 }), false);
assert.equal(state.markMaskCaptured(7, { nowMs: 750 }), true);
state.appendRecordingFrame({ contrastScore: 0.12, capturedAtMs: 760 });
state.appendRecordingFrame({ contrastScore: 0.44, capturedAtMs: 770 });
state.appendRecordingFrame({ contrastScore: 0.27, capturedAtMs: 780 });
const firstFinished = state.finishSequenceRecording({ endedAtMs: 790 });
assert.equal(firstFinished.ok, true);
assert.equal(firstFinished.frameIndex, 1);
snapshot = state.getSnapshot();
assert.equal(snapshot.recording, false);
assert.equal(snapshot.sequences.length, 1);
assert.equal(snapshot.sequences[0].complete, true);
assert.equal(snapshot.sequences[0].bestFrameIndex, 1);
assert.equal(snapshot.selectedSequenceId, firstRecording.sequenceId);
assert.equal(snapshot.selectedFrameIndex, 1);
assert.equal(snapshot.roadmapValid, true);
assert.equal(snapshot.roadmapEnabled, true);
assert.equal(snapshot.dsaEnabled, false);

// Every archived frame remains manually selectable as a roadmap.
assert.equal(
    state.selectRoadmapFrame(firstRecording.sequenceId, 0).ok,
    true
);
assert.equal(state.getSnapshot().selectedFrameIndex, 0);
assert.equal(state.useBestFrame(firstRecording.sequenceId).ok, true);
assert.equal(state.getSnapshot().selectedFrameIndex, 1);

// A roadmap can also combine a continuous frame range. The state normalizes
// reversed endpoints and exposes every source texture key to the renderer.
const rangeRoadmap = state.selectRoadmapRange(
    firstRecording.sequenceId,
    2,
    0
);
assert.equal(rangeRoadmap.ok, true);
assert.equal(rangeRoadmap.startFrameIndex, 0);
assert.equal(rangeRoadmap.endFrameIndex, 2);
assert.equal(rangeRoadmap.frameCount, 3);
assert.deepEqual(rangeRoadmap.storageKeys, ['1:0', '1:1', '1:2']);
snapshot = state.getSnapshot();
assert.equal(snapshot.roadmapCompositeActive, true);
assert.equal(snapshot.selectedFrameIndex, null);
assert.equal(snapshot.selectedRangeStartIndex, 0);
assert.equal(snapshot.selectedRangeEndIndex, 2);

// Archived cine uses the captured frame timing, can be paused/resumed, and
// remains independent from the frame currently selected for roadmapping.
assert.equal(
    state.playCine(firstRecording.sequenceId, { nowMs: 1000 }).ok,
    true
);
assert.equal(state.getSnapshot().cineFrameIndex, 0);
let cineAdvance = state.advanceCine({ nowMs: 1015 });
assert.equal(cineAdvance.changed, true);
assert.equal(state.getSnapshot().cineFrameIndex, 1);
assert.equal(state.pauseCine({ nowMs: 1018 }).ok, true);
assert.equal(state.getSnapshot().cinePlaying, false);
assert.equal(state.seekCineFrame(0, { nowMs: 1500 }).ok, true);
assert.equal(state.getSnapshot().cineFrameIndex, 0);
assert.equal(state.setCinePlaybackRate(2, { nowMs: 1500 }).ok, true);
assert.equal(state.getSnapshot().cinePlaybackRate, 2);
assert.equal(state.toggleCine(firstRecording.sequenceId, { nowMs: 2000 }).ok, true);
assert.equal(state.getSnapshot().cinePlaying, true);
state.advanceCine({ nowMs: 2005 });
assert.equal(state.getSnapshot().cineFrameIndex, 1);
assert.equal(state.seekCineFrame(2, { nowMs: 2006 }).ok, true);
assert.equal(state.getSnapshot().cineFrameIndex, 2);
assert.equal(state.setCinePlaybackRate(0.5, { nowMs: 2006 }).ok, true);
assert.equal(state.getSnapshot().cinePlaybackRate, 0.5);
assert.equal(state.stopCine().ok, true);
assert.equal(state.getSnapshot().cineSequenceId, null);

// The range compositor keeps the darkest vessel signal from every selected
// frame, producing the temporal union used by the roadmap texture.
assert.deepEqual(
    [...composeDsaFrameRangeRedChannels([
        new Uint8Array([220, 40, 220, 220]),
        new Uint8Array([220, 220, 35, 220]),
        new Uint8Array([220, 220, 220, 28])
    ])],
    [220, 40, 35, 28]
);
assert.equal(
    composeDsaFrameRangeRedChannels([
        new Uint8Array([220, 40]),
        new Uint8Array([220])
    ]),
    null
);

// Sequence frames retain the full detector buffer. Only the lightweight
// contrast-scoring pass is allowed to downsample.
assert.deepEqual(dsaArchiveDimensions(949.2, 774.8), {
    width: 949,
    height: 775
});
const scoreDimensions = dsaScoreDimensions(949, 775, 256);
assert.equal(Math.max(scoreDimensions.width, scoreDimensions.height), 256);

// Recording another angiography always refreshes the mask after the same
// preparation pause. Residual contrast remains allowed.
const secondRecording = state.startSequenceRecording({
    revision: 7,
    contrastVisible: true,
    startedAtMs: 200
});
assert.equal(secondRecording.ok, true);
assert.equal(state.getSnapshot().maskCapturePending, true);
assert.equal(state.markMaskCaptured(7, { nowMs: 850 }), true);
state.appendRecordingFrame({ contrastScore: 0.18, capturedAtMs: 860 });
state.appendRecordingFrame({ contrastScore: 0.31, capturedAtMs: 870 });
state.finishSequenceRecording({ endedAtMs: 880 });
assert.equal(state.getSnapshot().sequences.length, 2);
assert.equal(state.selectSequence(firstRecording.sequenceId).ok, true);
assert.equal(state.getSnapshot().selectedSequenceId, firstRecording.sequenceId);

// Moving the C-arm invalidates only the subtraction mask. The selected roadmap
// and all saved angiographies deliberately remain available and visible.
assert.equal(state.invalidateForGeometryRevision(7), false);
assert.equal(state.invalidateForGeometryRevision(8), true);
snapshot = state.getSnapshot();
assert.equal(snapshot.maskValid, false);
assert.equal(snapshot.dsaEnabled, false);
assert.equal(snapshot.roadmapValid, true);
assert.equal(snapshot.roadmapEnabled, true);
assert.equal(snapshot.selectedSequenceId, firstRecording.sequenceId);
assert.equal(snapshot.sequences.length, 2);
assert.match(snapshot.status, /roadmap retained/i);

// Holding R in the new view captures a new mask without waiting for washout,
// then records the following pulses.
const thirdRecording = state.startSequenceRecording({
    revision: 8,
    contrastVisible: true,
    startedAtMs: 300
});
assert.equal(thirdRecording.ok, true);
assert.equal(state.getSnapshot().maskCapturePending, true);
assert.equal(state.markMaskCaptured(8, { nowMs: 950 }), true);
state.appendRecordingFrame({ contrastScore: 0.52, capturedAtMs: 960 });
state.finishSequenceRecording({ endedAtMs: 970 });
assert.equal(state.getSnapshot().sequences.length, 3);
assert.equal(state.getSnapshot().selectedSequenceId, thirdRecording.sequenceId);

// The bounded archive evicts an unselected old sequence without discarding the
// active roadmap or leaking an unbounded number of frames.
const fourthRecording = state.startSequenceRecording({
    revision: 8,
    startedAtMs: 400
});
assert.equal(fourthRecording.evictedSequenceIds.length, 1);
assert.equal(state.markMaskCaptured(8, { nowMs: 1050 }), true);
state.appendRecordingFrame({ contrastScore: 0.11, capturedAtMs: 1060 });
state.finishSequenceRecording({ endedAtMs: 1070 });
assert.equal(state.getSnapshot().sequences.length, 3);
assert.ok(
    state.getSnapshot().sequences.every(sequence =>
        sequence.frames.length <= state.getSnapshot().maxFramesPerSequence
    )
);

state.clearRoadmap();
assert.equal(state.getSnapshot().roadmapValid, false);
assert.equal(state.getSnapshot().sequences.length, 3);

// Image scoring rewards a large, dense opacified region over a smaller or
// blank one and ignores the dark field outside collimation.
const width = 80;
const height = 64;
const blank = new Uint8Array(width * height).fill(
    Math.round(DSA_NEUTRAL_RED)
);
const smallVessel = blank.slice();
const filledVessels = blank.slice();
const brightMotionResidual = blank.slice();
for (let y = 20; y < 44; y++) {
    for (let x = 34; x < 39; x++) {
        smallVessel[x + y * width] = 42;
    }
    for (let x = 24; x < 54; x++) {
        filledVessels[x + y * width] = 42;
    }
}
brightMotionResidual.fill(235, 30 * width, 34 * width);
const blankScore = scoreDsaFrameRedChannel(blank, width, height);
const smallScore = scoreDsaFrameRedChannel(smallVessel, width, height);
const filledScore = scoreDsaFrameRedChannel(filledVessels, width, height);
const brightResidualScore = scoreDsaFrameRedChannel(
    brightMotionResidual,
    width,
    height
);
assert.ok(blankScore < 1e-6, `blank DSA score was ${blankScore}`);
assert.ok(
    brightResidualScore < 1e-6,
    `bright motion residual scored as contrast: ${brightResidualScore}`
);
assert.ok(smallScore > blankScore);
assert.ok(filledScore > smallScore * 3);

const blankProjection = new Uint8Array(width * height * 4);
const sparseProjection = blankProjection.slice();
const denseProjection = blankProjection.slice();
for (let y = 20; y < 44; y++) {
    for (let x = 34; x < 39; x++) {
        const offset = (x + y * width) * 4;
        sparseProjection[offset] = 150;
        sparseProjection[offset + 1] = 150;
        sparseProjection[offset + 2] = 150;
    }
    for (let x = 24; x < 54; x++) {
        const offset = (x + y * width) * 4;
        denseProjection[offset] = 150;
        denseProjection[offset + 1] = 150;
        denseProjection[offset + 2] = 150;
    }
}
assert.equal(
    scoreProjectedContrastRgba(blankProjection, width, height),
    0
);
assert.ok(
    scoreProjectedContrastRgba(denseProjection, width, height) >
        scoreProjectedContrastRgba(sparseProjection, width, height) * 3
);

const legacyCollimator = createCollimatorGeometry({ collimation: 0.32 });
assert.equal(legacyCollimator.left, 0.32);
assert.equal(legacyCollimator.right, 0.32);
assert.equal(legacyCollimator.top, 0.32);
assert.equal(legacyCollimator.bottom, 0.32);
assert.equal(legacyCollimator.sideRotationDegrees, 0);
assert.deepEqual(
    normalizeCollimatorSettings({
        collimationLeft: -1,
        collimationRight: 5,
        collimationTop: 0.2,
        collimationBottom: 0.4,
        sideCollimatorRotationDegrees: 140
    }),
    {
        left: 0,
        right: MAX_COLLIMATION,
        top: 0.2,
        bottom: 0.4,
        sideRotationDegrees: 30
    }
);
assert.ok(
    collimatedFieldReduction({
        collimationLeft: 0.5,
        collimationRight: 0.1,
        collimationTop: 0.1,
        collimationBottom: 0.1
    }) > collimatedFieldReduction({
        collimationLeft: 0.1,
        collimationRight: 0.1,
        collimationTop: 0.1,
        collimationBottom: 0.1
    })
);

const independentlyMovedLeft = createCollimatorGeometry({
    collimationLeft: 0.5,
    collimationRight: 0,
    collimationTop: 0,
    collimationBottom: 0
});
assert.ok(independentlyMovedLeft.leftBoundary > -0.4);
assert.equal(independentlyMovedLeft.rightBoundary, 1);
assert.equal(
    isDetectorPointInsideCollimator(-0.8, 0, 1, independentlyMovedLeft),
    false
);
assert.equal(
    isDetectorPointInsideCollimator(0.8, 0, 1, independentlyMovedLeft),
    true
);
const independentlyMovedTop = createCollimatorGeometry({
    collimationLeft: 0,
    collimationRight: 0,
    collimationTop: 0.5,
    collimationBottom: 0
});
assert.ok(independentlyMovedTop.topBoundary < 0.4);
assert.equal(independentlyMovedTop.bottomBoundary, -1);
assert.equal(
    isDetectorPointInsideCollimator(0, 0.8, 1, independentlyMovedTop),
    false
);
assert.equal(
    isDetectorPointInsideCollimator(0, -0.8, 1, independentlyMovedTop),
    true
);

// Detector magnification must not resize the monitor field or move the
// displayed shutter mask. Only the anatomy behind that fixed viewport zooms.
const fixedViewportCollimation = {
    collimationLeft: 0.35,
    collimationRight: 0,
    collimationTop: 0,
    collimationBottom: 0
};
const fixedViewportGeometry = createCollimatorGeometry(fixedViewportCollimation);
const ignoredZoomGeometry = createCollimatorGeometry({
    ...fixedViewportCollimation,
    detectorZoomFactor: 2.1
});
assert.deepEqual(ignoredZoomGeometry, fixedViewportGeometry);
assert.equal(
    isDetectorPointInsideCollimator(-0.8, 0, 1, ignoredZoomGeometry),
    false
);
assert.equal(
    collimatedFieldReduction({
        ...fixedViewportCollimation,
        detectorZoomFactor: 2.1
    }),
    collimatedFieldReduction(fixedViewportCollimation)
);

// A narrow field between the side shutters keeps a vertical vessel, while
// independently narrowing the top/bottom shutters removes most of it.
const squareSize = 80;
const verticalProjection = new Uint8Array(squareSize * squareSize * 4);
for (let y = 4; y < squareSize - 4; y++) {
    for (let x = 37; x < 43; x++) {
        const offset = (x + y * squareSize) * 4;
        verticalProjection[offset] = 180;
        verticalProjection[offset + 1] = 180;
        verticalProjection[offset + 2] = 180;
    }
}
const horizontalFieldScore = scoreProjectedContrastRgba(
    verticalProjection,
    squareSize,
    squareSize,
    {
        collimationLeft: 0.6,
        collimationRight: 0.6,
        collimationTop: 0,
        collimationBottom: 0,
        sideCollimatorRotationDegrees: 0
    }
);
const verticalFieldScore = scoreProjectedContrastRgba(
    verticalProjection,
    squareSize,
    squareSize,
    {
        collimationLeft: 0,
        collimationRight: 0,
        collimationTop: 0.6,
        collimationBottom: 0.6,
        sideCollimatorRotationDegrees: 0
    }
);
assert.ok(horizontalFieldScore > verticalFieldScore * 3);

// The side-shutter pair rotates rigidly around the detector center. The
// perpendicular distance of each boundary from that center is retained, which
// distinguishes rotation from a simple x += tan(angle) * y shear.
const straightSideGeometry = createCollimatorGeometry({
    collimationLeft: 0.6,
    collimationRight: 0.6,
    collimationTop: 0,
    collimationBottom: 0,
    sideCollimatorRotationDegrees: 0
});
const rotatedSideGeometry = createCollimatorGeometry({
    collimationLeft: 0.6,
    collimationRight: 0.6,
    collimationTop: 0,
    collimationBottom: 0,
    sideCollimatorRotationDegrees: 30
});
const sampleY = 0.7;
const rotatedCenterX = sideCollimatorBoundaryXAtY(
    rotatedSideGeometry.centerX,
    sampleY,
    rotatedSideGeometry
);
assert.ok(
    isDetectorPointInsideCollimator(
        rotatedCenterX,
        sampleY,
        1,
        rotatedSideGeometry
    )
);
assert.equal(
    isDetectorPointInsideCollimator(
        rotatedCenterX,
        sampleY,
        1,
        straightSideGeometry
    ),
    false
);
const rotatedRightBoundaryAtCenter = sideCollimatorBoundaryXAtY(
    rotatedSideGeometry.rightBoundary,
    0,
    rotatedSideGeometry
);
assert.ok(rotatedRightBoundaryAtCenter > rotatedSideGeometry.rightBoundary);
assert.equal(
    isDetectorPointInsideCollimator(
        rotatedRightBoundaryAtCenter - 0.005,
        0,
        1,
        rotatedSideGeometry
    ),
    true,
    'rotated boundary must retain its distance from the detector center'
);
assert.equal(
    isDetectorPointInsideCollimator(
        rotatedRightBoundaryAtCenter - 0.005,
        0,
        1,
        straightSideGeometry
    ),
    false,
    'rotation must not behave like a simple shear'
);
assert.ok(
    createCollimatorFieldPolygon({
        collimationLeft: 0.6,
        collimationRight: 0.6,
        collimationTop: 0,
        collimationBottom: 0,
        sideCollimatorRotationDegrees: 30
    }).every(point => Number.isFinite(point.x) && Number.isFinite(point.y)),
    'preview polygon must remain finite at maximum rotation'
);
const openRotatedSideGeometry = createCollimatorGeometry({
    collimationLeft: 0,
    collimationRight: 0,
    collimationTop: 0,
    collimationBottom: 0,
    sideCollimatorRotationDegrees: 30
});
assert.equal(
    isDetectorPointInsideCollimator(
        0.85,
        -0.4,
        1,
        openRotatedSideGeometry
    ),
    true,
    'rotated shutters may expose the image only inside the detector aperture'
);
assert.equal(
    isDetectorPointInsideCollimator(
        0.95,
        -0.4,
        1,
        openRotatedSideGeometry
    ),
    false,
    'rotated shutters must never reveal image outside the detector aperture'
);
const verticallyCollimatedGeometry = createCollimatorGeometry({
    collimationLeft: 0,
    collimationRight: 0,
    collimationTop: 0.5,
    collimationBottom: 0.5,
    sideCollimatorRotationDegrees: 30
});
assert.equal(
    isDetectorPointInsideCollimator(
        sideCollimatorBoundaryXAtY(
            verticallyCollimatedGeometry.centerX,
            sampleY,
            verticallyCollimatedGeometry
        ),
        sampleY,
        1,
        verticallyCollimatedGeometry
    ),
    false,
    'rotating side shutters must not rotate the top/bottom field limit'
);

console.log('DSA sequence and roadmap state tests passed');
