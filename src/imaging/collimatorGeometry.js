export const DEFAULT_COLLIMATION = 0.08;
export const MAX_COLLIMATION = 0.68;
export const COLLIMATION_SHUTTER_SCALE = 1.35;
export const DETECTOR_APERTURE_HALF_EXTENT = 0.88;
export const MIN_SIDE_COLLIMATOR_ROTATION_DEG = -30;
export const MAX_SIDE_COLLIMATOR_ROTATION_DEG = 30;

function finiteOr(value, fallback) {
    const numericValue = Number(value);
    return Number.isFinite(numericValue) ? numericValue : fallback;
}

function clamp(value, minimum, maximum) {
    return Math.min(maximum, Math.max(minimum, value));
}

export function normalizeCollimatorSettings(settings = {}) {
    const legacyCollimation = clamp(
        finiteOr(settings.collimation, DEFAULT_COLLIMATION),
        0,
        MAX_COLLIMATION
    );
    const horizontalCollimation = clamp(
        finiteOr(settings.collimationHorizontal, legacyCollimation),
        0,
        MAX_COLLIMATION
    );
    const verticalCollimation = clamp(
        finiteOr(settings.collimationVertical, legacyCollimation),
        0,
        MAX_COLLIMATION
    );
    return {
        left: clamp(
            finiteOr(settings.collimationLeft, horizontalCollimation),
            0,
            MAX_COLLIMATION
        ),
        right: clamp(
            finiteOr(settings.collimationRight, horizontalCollimation),
            0,
            MAX_COLLIMATION
        ),
        top: clamp(
            finiteOr(settings.collimationTop, verticalCollimation),
            0,
            MAX_COLLIMATION
        ),
        bottom: clamp(
            finiteOr(settings.collimationBottom, verticalCollimation),
            0,
            MAX_COLLIMATION
        ),
        sideRotationDegrees: clamp(
            finiteOr(
                settings.sideCollimatorRotationDegrees,
                finiteOr(settings.sideCollimatorTiltDegrees, 0)
            ),
            MIN_SIDE_COLLIMATOR_ROTATION_DEG,
            MAX_SIDE_COLLIMATOR_ROTATION_DEG
        )
    };
}

export function createCollimatorGeometry(settings = {}) {
    const normalized = normalizeCollimatorSettings(settings);
    const sideRotationRadians = normalized.sideRotationDegrees * Math.PI / 180;
    const detectorApertureHalfExtent = clamp(
        finiteOr(
            settings.detectorApertureHalfExtent,
            DETECTOR_APERTURE_HALF_EXTENT
        ),
        0.5,
        1
    );
    const leftBoundary = -1 + normalized.left * COLLIMATION_SHUTTER_SCALE;
    const rightBoundary = 1 - normalized.right * COLLIMATION_SHUTTER_SCALE;
    const topBoundary = 1 - normalized.top * COLLIMATION_SHUTTER_SCALE;
    const bottomBoundary = -1 + normalized.bottom * COLLIMATION_SHUTTER_SCALE;
    return {
        ...normalized,
        leftBoundary,
        rightBoundary,
        topBoundary,
        bottomBoundary,
        centerX: (leftBoundary + rightBoundary) * 0.5,
        centerY: (bottomBoundary + topBoundary) * 0.5,
        halfWidth: (rightBoundary - leftBoundary) * 0.5,
        halfHeight: (topBoundary - bottomBoundary) * 0.5,
        detectorApertureHalfExtent,
        sideRotationCos: Math.cos(sideRotationRadians),
        sideRotationSin: Math.sin(sideRotationRadians)
    };
}

export function sideCollimatorBoundaryXAtY(boundary, detectorY, geometry) {
    return (
        finiteOr(boundary, 0) -
        geometry.sideRotationSin * finiteOr(detectorY, 0)
    ) / geometry.sideRotationCos;
}

function clipPolygonToHalfPlane(points, signedDistance) {
    const clipped = [];
    for (let index = 0; index < points.length; index++) {
        const start = points[index];
        const end = points[(index + 1) % points.length];
        const startDistance = signedDistance(start);
        const endDistance = signedDistance(end);
        const startInside = startDistance >= 0;
        const endInside = endDistance >= 0;

        if (startInside) clipped.push(start);
        if (startInside === endInside) continue;

        const amount = startDistance / (startDistance - endDistance);
        clipped.push({
            x: start.x + (end.x - start.x) * amount,
            y: start.y + (end.y - start.y) * amount
        });
    }
    return clipped;
}

export function createCollimatorFieldPolygon(settings = {}) {
    const geometry = createCollimatorGeometry(settings);
    const aperture = geometry.detectorApertureHalfExtent;
    const localX = point =>
        geometry.sideRotationCos * point.x +
        geometry.sideRotationSin * point.y;
    let polygon = [
        {
            x: -aperture,
            y: -aperture
        },
        {
            x: aperture,
            y: -aperture
        },
        {
            x: aperture,
            y: aperture
        },
        {
            x: -aperture,
            y: aperture
        }
    ];
    polygon = clipPolygonToHalfPlane(
        polygon,
        point => localX(point) - geometry.leftBoundary
    );
    polygon = clipPolygonToHalfPlane(
        polygon,
        point => geometry.rightBoundary - localX(point)
    );
    polygon = clipPolygonToHalfPlane(
        polygon,
        point => point.y - geometry.bottomBoundary
    );
    polygon = clipPolygonToHalfPlane(
        polygon,
        point => geometry.topBoundary - point.y
    );
    return polygon;
}

export function collimatedFieldAreaFraction(settings = {}) {
    const geometry = createCollimatorGeometry(settings);
    const polygon = createCollimatorFieldPolygon(settings);
    let doubledArea = 0;
    for (let index = 0; index < polygon.length; index++) {
        const point = polygon[index];
        const next = polygon[(index + 1) % polygon.length];
        doubledArea += point.x * next.y - next.x * point.y;
    }
    const aperture = geometry.detectorApertureHalfExtent;
    const doubledDetectorArea = 8 *
        aperture * aperture;
    return clamp(Math.abs(doubledArea) / doubledDetectorArea, 0, 1);
}

export function collimatedFieldReduction(settings = {}) {
    return 1 - collimatedFieldAreaFraction(settings);
}

export function isDetectorPointInsideCollimator(
    centeredX,
    centeredY,
    aspect,
    geometry,
    edgeInset = 1
) {
    const safeAspect = Math.max(0.001, finiteOr(aspect, 1));
    const squareX = (safeAspect >= 1
        ? centeredX * safeAspect
        : centeredX);
    const squareY = (safeAspect >= 1
        ? centeredY
        : centeredY / safeAspect);
    // The left/right shutter pair rotates around the detector center. Their
    // boundaries stay parallel and retain their perpendicular distance from
    // the center; the top and bottom shutters remain horizontal.
    const localX =
        geometry.sideRotationCos * squareX +
        geometry.sideRotationSin * squareY;
    const localY = squareY;
    const inset = clamp(finiteOr(edgeInset, 1), 0, 1);
    const apertureInset = geometry.detectorApertureHalfExtent * inset;
    const insideDetectorAperture = Math.abs(squareX) <= apertureInset &&
        Math.abs(squareY) <= apertureInset;
    return insideDetectorAperture &&
        Math.abs(localX - geometry.centerX) <= geometry.halfWidth * inset &&
        Math.abs(localY - geometry.centerY) <= geometry.halfHeight * inset;
}
