export const vertexShader = `
// Fullscreen quad vertex shader for the final display pass.
// Simply forwards UVs to the fragment shader.
varying vec2 vUv;
void main() {
    vUv = uv;
    gl_Position = vec4(position.xy, 0.0, 1.0);
}
`;

export const fragmentShader = `
// Final display shader.
// - In fluoroscopy mode: composes anatomy thickness, iodine contrast, and
//   metallic devices as radiographic attenuation, then applies detector-style
//   windowing, edge enhancement, field falloff, and dose-dependent noise.
// - In wireframe mode: just visualizes the scene with edge-enhanced alpha.
uniform sampler2D uTexture;
uniform sampler2D contrastTexture;
uniform sampler2D thicknessTexture;
uniform sampler2D metalTexture;
uniform sampler2D sheathTexture;
uniform sampler2D boneTexture;
uniform vec3 gray;
uniform bool fluoroscopy;
uniform float time;
uniform float noiseLevel;
uniform float imageBrightness;
uniform float imageContrast;
uniform bool autoExposureEnabled;
uniform float autoExposureLevel;
uniform float pulseRate;
uniform float scatterStrength;
uniform float collimation;
uniform float boneOpacity;
uniform vec2 resolution;
uniform float edgeStrength;
uniform float contrastOpacity;
uniform float contrastGain;
varying vec2 vUv;

float saturate(float value) {
    return clamp(value, 0.0, 1.0);
}

float max3(vec3 value) {
    return max(max(value.r, value.g), value.b);
}

// Hash-based noise. Animated samples mimic quantum mottle; stable samples are
// reused for detector fixed-pattern noise.
float random(vec2 st) {
    return fract(sin(dot(st.xy, vec2(12.9898, 78.233))) * 43758.5453123);
}

float animatedNoise(vec2 st, float phase) {
    return random(st + vec2(phase * 17.37, phase * 5.91));
}

float sampleSignal(sampler2D source, vec2 uv) {
    return max3(texture2D(source, uv).rgb);
}

float contrastAt(vec2 uv) {
    float signal = sampleSignal(contrastTexture, uv);
    return saturate(1.0 - exp(-signal * max(0.0, contrastGain) * 1.45));
}

float metalAt(vec2 uv) {
    float signal = sampleSignal(metalTexture, uv);
    return smoothstep(0.025, 0.58, signal);
}

float sheathAt(vec2 uv) {
    float signal = sampleSignal(sheathTexture, uv);
    return smoothstep(0.03, 0.32, signal);
}

vec3 boneProjectionSampleAt(vec2 uv) {
    return texture2D(boneTexture, uv).rgb;
}

float boneProjectionAt(vec2 uv) {
    vec2 texel = 1.0 / resolution;
    float center = boneProjectionSampleAt(uv).b * 0.42;
    float axial = (
        boneProjectionSampleAt(uv + texel * vec2(1.5, 0.0)).b +
        boneProjectionSampleAt(uv + texel * vec2(-1.5, 0.0)).b +
        boneProjectionSampleAt(uv + texel * vec2(0.0, 1.5)).b +
        boneProjectionSampleAt(uv + texel * vec2(0.0, -1.5)).b
    ) * 0.10;
    float diagonal = (
        boneProjectionSampleAt(uv + texel * vec2(1.2, 1.2)).b +
        boneProjectionSampleAt(uv + texel * vec2(-1.2, 1.2)).b +
        boneProjectionSampleAt(uv + texel * vec2(1.2, -1.2)).b +
        boneProjectionSampleAt(uv + texel * vec2(-1.2, -1.2)).b
    ) * 0.045;
    return center + axial + diagonal;
}

float corticalProjectionAt(vec2 uv) {
    vec2 texel = 1.0 / resolution;
    float center = boneProjectionSampleAt(uv).r * 0.34;
    float nearAxial = (
        boneProjectionSampleAt(uv + texel * vec2(1.45, 0.0)).r +
        boneProjectionSampleAt(uv + texel * vec2(-1.45, 0.0)).r +
        boneProjectionSampleAt(uv + texel * vec2(0.0, 1.45)).r +
        boneProjectionSampleAt(uv + texel * vec2(0.0, -1.45)).r
    ) * 0.075;
    float diagonal = (
        boneProjectionSampleAt(uv + texel * vec2(1.75, 1.75)).r +
        boneProjectionSampleAt(uv + texel * vec2(-1.75, 1.75)).r +
        boneProjectionSampleAt(uv + texel * vec2(1.75, -1.75)).r +
        boneProjectionSampleAt(uv + texel * vec2(-1.75, -1.75)).r
    ) * 0.04;
    float wideAxial = (
        boneProjectionSampleAt(uv + texel * vec2(3.25, 0.0)).r +
        boneProjectionSampleAt(uv + texel * vec2(-3.25, 0.0)).r +
        boneProjectionSampleAt(uv + texel * vec2(0.0, 3.25)).r +
        boneProjectionSampleAt(uv + texel * vec2(0.0, -3.25)).r
    ) * 0.03;
    float wideDiagonal = (
        boneProjectionSampleAt(uv + texel * vec2(2.75, 2.75)).r +
        boneProjectionSampleAt(uv + texel * vec2(-2.75, 2.75)).r +
        boneProjectionSampleAt(uv + texel * vec2(2.75, -2.75)).r +
        boneProjectionSampleAt(uv + texel * vec2(-2.75, -2.75)).r
    ) * 0.018;
    return center + nearAxial + diagonal + wideAxial + wideDiagonal;
}

float thicknessPathAt(vec2 uv) {
    float thickness = texture2D(thicknessTexture, uv).r;
    return saturate(thickness * 6.2);
}

float bonePathAt(vec2 uv) {
    float thicknessPath = pow(thicknessPathAt(uv), 0.68);
    float projectedBone = boneProjectionAt(uv);
    float projectionPath = pow(saturate(projectedBone * 2.5), 0.9) * 0.46;
    return saturate(thicknessPath * (1.12 + projectionPath * 0.52) + projectionPath * 0.12);
}

float corticalBoneAt(vec2 uv) {
    vec2 texel = 1.0 / resolution;
    float c = thicknessPathAt(uv);
    float l = thicknessPathAt(uv + texel * vec2(-1.0, 0.0));
    float r = thicknessPathAt(uv + texel * vec2(1.0, 0.0));
    float t = thicknessPathAt(uv + texel * vec2(0.0, -1.0));
    float b = thicknessPathAt(uv + texel * vec2(0.0, 1.0));
    float thicknessEdge = smoothstep(0.014, 0.19, length(vec2(r - l, b - t)));
    float entryExitCortex = smoothstep(0.026, 0.31, c) * (0.15 + thicknessEdge * 0.48);
    float projectedCortex = pow(saturate(corticalProjectionAt(uv) * 2.65), 0.78);
    return saturate(entryExitCortex + projectedCortex * (0.27 + c * 0.38));
}

float attenuationAt(vec2 uv) {
    float boneVisibility = saturate(boneOpacity);
    float boneSignal = bonePathAt(uv) * 1.08 + corticalBoneAt(uv) * 0.88;
    float bone = boneSignal * 2.15 * boneVisibility;
    float iodine = contrastAt(uv) * saturate(contrastOpacity) * 3.25;
    float metal = metalAt(uv) * 5.25;
    float sheath = sheathAt(uv) * 0.42;

    // The accumulated visible frame creates detector persistence across the
    // full fluoroscopy image while current attenuation still leads the frame.
    float temporalTrace = smoothstep(0.025, 0.72, sampleSignal(uTexture, uv)) * 0.46;
    return max(0.0, bone + iodine + metal + sheath + temporalTrace);
}

float vignetteField(vec2 uv) {
    vec2 centered = uv * 2.0 - 1.0;
    centered.x *= resolution.x / max(1.0, resolution.y);
    float radius = length(centered);
    return 1.0 - 0.22 * smoothstep(0.28, 1.35, radius);
}

float patientBodyField(vec2 uv) {
    vec2 centered = uv * 2.0 - 1.0;
    centered.x *= resolution.x / max(1.0, resolution.y);
    float pelvis = 1.0 - smoothstep(0.42, 1.18, length(centered * vec2(0.72, 1.05)));
    float trunk = 1.0 - smoothstep(0.35, 1.08, length((centered - vec2(0.0, -0.18)) * vec2(0.62, 1.35)));
    return saturate(max(pelvis, trunk * 0.72));
}

float scatterFieldAt(vec2 uv, float attenuation) {
    float tissuePath = patientBodyField(uv);
    float projectedPath = saturate(thicknessPathAt(uv) * 0.55 + bonePathAt(uv) * 0.26 + tissuePath * 0.22);
    return saturate(scatterStrength * (projectedPath * 0.72 + attenuation * 0.08));
}

float collimatorMask(vec2 uv) {
    vec2 centered = uv * 2.0 - 1.0;
    float aspect = resolution.x / max(1.0, resolution.y);
    vec2 squareCoord = centered;
    if (aspect >= 1.0) {
        squareCoord.x *= aspect;
    } else {
        squareCoord.y /= max(0.001, aspect);
    }
    float crop = saturate(collimation);
    float halfSize = 1.0 - crop * 1.35;
    float softness = 0.022 + crop * 0.055;
    float maskX = 1.0 - smoothstep(halfSize, halfSize + softness, abs(squareCoord.x));
    float maskY = 1.0 - smoothstep(halfSize, halfSize + softness, abs(squareCoord.y));
    return saturate(maskX * maskY);
}

// Simple Sobel-like edge factor based on alpha channel of uTexture.
// Used to enhance edges in the displayed result.
float edgeFactor(vec2 uv) {
    vec2 texel = 1.0 / resolution;
    float tl = texture2D(uTexture, uv + texel * vec2(-1.0, -1.0)).a;
    float t  = texture2D(uTexture, uv + texel * vec2(0.0, -1.0)).a;
    float tr = texture2D(uTexture, uv + texel * vec2(1.0, -1.0)).a;
    float l  = texture2D(uTexture, uv + texel * vec2(-1.0, 0.0)).a;
    float r  = texture2D(uTexture, uv + texel * vec2(1.0, 0.0)).a;
    float bl = texture2D(uTexture, uv + texel * vec2(-1.0, 1.0)).a;
    float b  = texture2D(uTexture, uv + texel * vec2(0.0, 1.0)).a;
    float br = texture2D(uTexture, uv + texel * vec2(1.0, 1.0)).a;
    float gx = -tl - 2.0*l - bl + tr + 2.0*r + br;
    float gy = -tl - 2.0*t - tr + bl + 2.0*b + br;
    return length(vec2(gx, gy));
}
void main() {
    vec4 tex = texture2D(uTexture, vUv);
    if (fluoroscopy) {
        vec2 texel = 1.0 / resolution;
        float centerAttenuation = attenuationAt(vUv);
        float localScatter = scatterFieldAt(vUv, centerAttenuation);
        float exposureLift = autoExposureEnabled ? autoExposureLevel : 0.0;
        float neighborAttenuation = (
            attenuationAt(vUv + texel * vec2(1.0, 0.0)) +
            attenuationAt(vUv + texel * vec2(-1.0, 0.0)) +
            attenuationAt(vUv + texel * vec2(0.0, 1.0)) +
            attenuationAt(vUv + texel * vec2(0.0, -1.0))
        ) * 0.25;

        // C-arm images are usually edge-enhanced after acquisition. Sharpen
        // attenuation before transmission so radiopaque borders get the expected
        // dark/bright overshoot instead of an alpha-only outline.
        float scatterSoftenedEdge = mix(0.34, 0.16, localScatter);
        float sharpenedAttenuation = max(
            0.0,
            centerAttenuation + (centerAttenuation - neighborAttenuation) * edgeStrength * scatterSoftenedEdge
        );

        float transmission = exp(-sharpenedAttenuation);
        float scatterFog = saturate(centerAttenuation * 0.045 + localScatter * 0.42);
        transmission = mix(transmission, 0.60 + exposureLift * 0.18, scatterFog);

        // Detector window/level with a soft shoulder. This keeps the air field
        // from becoming pure white and gives dense contrast a real black floor.
        float luma = pow(saturate(transmission), 0.72);
        luma = smoothstep(0.035, 0.985, luma);
        luma = mix(0.08, 0.88, luma);

        float field = vignetteField(vUv);
        float fixedPattern = (random(floor(vUv * resolution / 7.0)) - 0.5) * 0.012;
        float columnPattern = (random(vec2(floor(vUv.x * resolution.x / 3.0), 19.0)) - 0.5) * 0.004;
        float gridPattern =
            sin(vUv.x * resolution.x * 0.86) * 0.0008 +
            sin(vUv.y * resolution.y * 0.42) * 0.0006;
        float doseNoiseScale = sqrt(30.0 / clamp(pulseRate, 7.5, 30.0));
        float pulseIndex = floor(time * max(1.0, pulseRate));
        float pulseJitter = (random(vec2(pulseIndex, 37.0)) - 0.5) * 0.003 * doseNoiseScale;
        float stableMottle = random(floor(vUv * resolution / 2.0)) - 0.5;
        float animatedMottle = animatedNoise(vUv * resolution, pulseIndex * 0.73) - 0.5;
        float mottle = mix(stableMottle, animatedMottle, 0.32)
            * noiseLevel
            * doseNoiseScale
            * (0.10 + 0.24 * sqrt(max(luma, 0.0)));

        luma = saturate(luma * field + fixedPattern + columnPattern + gridPattern + mottle + pulseJitter);
        luma = mix(luma, 0.56 + (luma - 0.56) * 0.54, localScatter * 0.46);
        luma = saturate(luma + exposureLift);
        luma = saturate((luma - 0.5) * max(0.0, imageContrast) + 0.5 + imageBrightness);
        luma = mix(0.018, luma, collimatorMask(vUv));

        // Phosphor/detector response is slightly warm-neutral, not mathematically
        // flat grayscale. Keep it subtle so it still reads as fluoroscopy.
        vec3 detectorTint = vec3(0.965, 0.982, 1.0);
        gl_FragColor = vec4(gray * detectorTint * luma, 1.0);
    } else {
        // Debug mode: keep original color, use edge to boost alpha.
        float edge = edgeFactor(vUv) * edgeStrength;
        float alpha = clamp(tex.a + edge, 0.0, 1.0);
        gl_FragColor = vec4(tex.rgb, alpha);
    }
}
`;
