import {
  guidewireMaterialProfile,
  normalizeGuidewireType,
  sampleGuidewireRestCenterline
} from '../physics/guidewireMaterialProfile.js';

const SVG_WIDTH = 72;
const SVG_HEIGHT = 64;
const SVG_MARGIN = 5;
const SAMPLE_SPACING_MM = 0.5;
const PREVIEW_SHAFT_LENGTH_MM = 14;
const STRAIGHT_PREVIEW_LENGTH_MM = 30;

export function buildGuidewireTipPreviewPath(type) {
  const normalizedType = normalizeGuidewireType(type);
  const profile = guidewireMaterialProfile(normalizedType);
  const exposedLength = profile.naturalArcLength > 0
    ? profile.naturalArcLength + PREVIEW_SHAFT_LENGTH_MM
    : STRAIGHT_PREVIEW_LENGTH_MM;
  const sampleCount = Math.max(
    2,
    Math.ceil(exposedLength / SAMPLE_SPACING_MM) + 1
  );
  const points = [];
  const sample = {};

  for (let index = 0; index < sampleCount; index++) {
    const distance = exposedLength * index / (sampleCount - 1);
    sampleGuidewireRestCenterline(
      normalizedType,
      exposedLength,
      distance,
      1,
      sample
    );
    points.push({
      x: sample.normalDistance * profile.frameNormalSign,
      y: -sample.tangentDistance
    });
  }

  const minX = Math.min(...points.map(point => point.x));
  const maxX = Math.max(...points.map(point => point.x));
  const minY = Math.min(...points.map(point => point.y));
  const maxY = Math.max(...points.map(point => point.y));
  const shapeWidth = Math.max(1e-6, maxX - minX);
  const shapeHeight = Math.max(1e-6, maxY - minY);
  const scale = Math.min(
    (SVG_WIDTH - SVG_MARGIN * 2) / shapeWidth,
    (SVG_HEIGHT - SVG_MARGIN * 2) / shapeHeight
  );
  const offsetX = (SVG_WIDTH - shapeWidth * scale) * 0.5;
  const offsetY = (SVG_HEIGHT - shapeHeight * scale) * 0.5;

  return points.map((point, index) => {
    const x = offsetX + (point.x - minX) * scale;
    const y = offsetY + (point.y - minY) * scale;
    return `${index === 0 ? 'M' : 'L'}${x.toFixed(2)} ${y.toFixed(2)}`;
  }).join(' ');
}

export function renderGuidewireTipPreviews(root = document) {
  for (const type of ['glidewire', 'steel-j-035']) {
    const path = root.querySelector(`[data-guidewire-tip="${type}"]`);
    if (path) path.setAttribute('d', buildGuidewireTipPreviewPath(type));
  }
}
