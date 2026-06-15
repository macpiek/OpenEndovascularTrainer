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
uniform vec3 gray;
uniform bool fluoroscopy;
uniform float time;
uniform float noiseLevel;
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

float bonePathAt(vec2 uv) {
    float thickness = texture2D(thicknessTexture, uv).r;
    return pow(saturate(thickness * 58.0), 0.82);
}

float attenuationAt(vec2 uv) {
    float boneMu = mix(0.18, 2.15, saturate(boneOpacity));
    float bone = bonePathAt(uv) * boneMu;
    float iodine = contrastAt(uv) * saturate(contrastOpacity) * 3.25;
    float metal = metalAt(uv) * 5.25;

    // A small contribution from the accumulated visible frame preserves mild
    // detector lag without letting the old postprocess dominate the image.
    float temporalTrace = smoothstep(0.04, 0.85, sampleSignal(uTexture, uv)) * 0.22;
    return max(0.0, bone + iodine + metal + temporalTrace);
}

float vignetteField(vec2 uv) {
    vec2 centered = uv * 2.0 - 1.0;
    centered.x *= resolution.x / max(1.0, resolution.y);
    float radius = length(centered);
    return 1.0 - 0.22 * smoothstep(0.28, 1.35, radius);
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
        float neighborAttenuation = (
            attenuationAt(vUv + texel * vec2(1.0, 0.0)) +
            attenuationAt(vUv + texel * vec2(-1.0, 0.0)) +
            attenuationAt(vUv + texel * vec2(0.0, 1.0)) +
            attenuationAt(vUv + texel * vec2(0.0, -1.0))
        ) * 0.25;

        // C-arm images are usually edge-enhanced after acquisition. Sharpen
        // attenuation before transmission so radiopaque borders get the expected
        // dark/bright overshoot instead of an alpha-only outline.
        float sharpenedAttenuation = max(
            0.0,
            centerAttenuation + (centerAttenuation - neighborAttenuation) * edgeStrength * 0.34
        );

        float transmission = exp(-sharpenedAttenuation);
        float scatterFog = saturate(centerAttenuation * 0.06);
        transmission = mix(transmission, 0.62, scatterFog);

        // Detector window/level with a soft shoulder. This keeps the air field
        // from becoming pure white and gives dense contrast a real black floor.
        float luma = pow(saturate(transmission), 0.72);
        luma = smoothstep(0.035, 0.985, luma);
        luma = mix(0.08, 0.88, luma);

        float field = vignetteField(vUv);
        float fixedPattern = (random(floor(vUv * resolution / 7.0)) - 0.5) * 0.022;
        float columnPattern = (random(vec2(floor(vUv.x * resolution.x / 3.0), 19.0)) - 0.5) * 0.012;
        float gridPattern =
            sin(vUv.x * resolution.x * 0.86) * 0.0035 +
            sin(vUv.y * resolution.y * 0.42) * 0.0025;
        float mottle = (animatedNoise(vUv * resolution, time * 23.0) - 0.5)
            * noiseLevel
            * (0.15 + 0.32 * sqrt(max(luma, 0.0)));

        luma = saturate(luma * field + fixedPattern + columnPattern + gridPattern + mottle);

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
