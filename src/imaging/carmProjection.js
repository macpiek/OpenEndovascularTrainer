import { DETECTOR_APERTURE_HALF_EXTENT } from './collimatorGeometry.js';
import { DEFAULT_DETECTOR_SIDE_MM } from './detectorZoomModes.js';

export const CARM_SOURCE_TO_ISOCENTER_MM = 350;

// World coordinates and source/detector distances are in millimetres. The
// displayed detector occupies only the central aperture of the raw image;
// include that margin in the camera cone so the visible field matches the
// selected detector size at its plane, not at the isocentre or texture edge.
export function applyCArmProjection(
    camera,
    sourceToIsocenterMm,
    isocenterToDetectorMm,
    zoomFactor = 1,
    detectorSideMm = DEFAULT_DETECTOR_SIDE_MM
) {
    for (const value of [sourceToIsocenterMm, isocenterToDetectorMm, zoomFactor, detectorSideMm]) {
        if (!Number.isFinite(value) || value <= 0) {
            throw new RangeError('C-arm distances, detector size and zoom must be positive.');
        }
    }
    const sourceToDetectorMm = sourceToIsocenterMm + isocenterToDetectorMm;
    camera.fov = 2 * Math.atan(
        detectorSideMm /
        (2 * sourceToDetectorMm * DETECTOR_APERTURE_HALF_EXTENT)
    ) * 180 / Math.PI;
    camera.zoom = zoomFactor;
    // Anatomy behind the detector cannot contribute to its image. Without this
    // limit, moving the whole arm away could expose a field larger than it.
    camera.far = sourceToDetectorMm;
    camera.updateProjectionMatrix();

    return {
        sourceToDetectorMm,
        detectorFieldMm: detectorSideMm / zoomFactor,
        isocenterFieldMm: detectorSideMm / zoomFactor *
            sourceToIsocenterMm / sourceToDetectorMm
    };
}
