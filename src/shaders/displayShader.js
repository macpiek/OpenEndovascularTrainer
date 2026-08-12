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
uniform sampler2D catheterTexture;
uniform sampler2D sheathTexture;
uniform sampler2D boneTexture;
uniform sampler2D dsaMaskTexture;
uniform sampler2D roadmapTexture;
uniform sampler2D cineTexture;
uniform vec3 gray;
uniform bool fluoroscopy;
uniform bool dsaEnabled;
uniform bool dsaMaskValid;
uniform bool roadmapEnabled;
uniform bool roadmapValid;
uniform bool cineEnabled;
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
uniform float dsaGain;
uniform float roadmapOpacity;
uniform float roadmapBackgroundVisibility;
uniform float roadmapEdgeEnhancement;
uniform float roadmapEdgeDarkness;
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
    float center = sampleSignal(metalTexture, uv);
    return pow(saturate(center), 1.18);
}

float catheterAt(vec2 uv) {
    float signal = sampleSignal(catheterTexture, uv);
    return smoothstep(0.025, 0.48, signal);
}

float sheathAt(vec2 uv) {
    float signal = sampleSignal(sheathTexture, uv);
    return smoothstep(0.03, 0.32, signal);
}

vec4 boneProjectionSampleAt(vec2 uv) {
    return texture2D(boneTexture, uv);
}

float rawBoneThicknessFromSample(vec4 projectionSample) {
    return max(projectionSample.g - projectionSample.r, 0.0) * 10.2;
}

vec4 boneTransportAt(vec2 uv) {
    vec2 texel = 1.0 / resolution;
    vec4 center = boneProjectionSampleAt(uv);
    vec4 axial = (
        boneProjectionSampleAt(uv + texel * vec2(1.4, 0.0)) +
        boneProjectionSampleAt(uv + texel * vec2(-1.4, 0.0)) +
        boneProjectionSampleAt(uv + texel * vec2(0.0, 1.4)) +
        boneProjectionSampleAt(uv + texel * vec2(0.0, -1.4))
    ) * 0.04;
    vec4 diagonal = (
        boneProjectionSampleAt(uv + texel * vec2(1.7, 1.7)) +
        boneProjectionSampleAt(uv + texel * vec2(-1.7, 1.7)) +
        boneProjectionSampleAt(uv + texel * vec2(1.7, -1.7)) +
        boneProjectionSampleAt(uv + texel * vec2(-1.7, -1.7))
    ) * 0.015;
    return center * 0.78 + axial + diagonal;
}

float thicknessPathAt(vec2 uv) {
    float thickness = texture2D(thicknessTexture, uv).r;
    return saturate(thickness * 6.2);
}

vec4 boneLayerPathsAt(vec2 uv) {
    vec4 transport = boneTransportAt(uv);
    float rawThickness = max(transport.g - transport.r, 0.0) * 10.2;
    float totalPath = 1.0 - exp(-rawThickness * 0.5);
    float corticalPath = min(transport.b * 11.2, totalPath * 0.92);
    float corticalShare = saturate(corticalPath / max(totalPath, 0.001));
    float cancellousPath = max(totalPath - corticalPath * 0.8, 0.0) * mix(0.3, 0.54, corticalShare);
    float cancellousTexture = mix(0.72, 0.98, saturate(transport.a * 2.05));
    return vec4(totalPath, corticalPath, cancellousPath, cancellousTexture);
}

float bonePathAt(vec2 uv) {
    return saturate(boneLayerPathsAt(uv).x);
}

float corticalEdgeAt(vec2 uv) {
    vec2 texel = 1.0 / resolution;
    float thicknessL = thicknessPathAt(uv + texel * vec2(-1.0, 0.0));
    float thicknessR = thicknessPathAt(uv + texel * vec2(1.0, 0.0));
    float thicknessT = thicknessPathAt(uv + texel * vec2(0.0, -1.0));
    float thicknessB = thicknessPathAt(uv + texel * vec2(0.0, 1.0));
    float thicknessEdge = length(vec2(thicknessR - thicknessL, thicknessB - thicknessT));

    vec4 sampleL = boneProjectionSampleAt(uv + texel * vec2(-1.0, 0.0));
    vec4 sampleR = boneProjectionSampleAt(uv + texel * vec2(1.0, 0.0));
    vec4 sampleT = boneProjectionSampleAt(uv + texel * vec2(0.0, -1.0));
    vec4 sampleB = boneProjectionSampleAt(uv + texel * vec2(0.0, 1.0));
    float pathEdge = length(vec2(
        rawBoneThicknessFromSample(sampleR) - rawBoneThicknessFromSample(sampleL),
        rawBoneThicknessFromSample(sampleB) - rawBoneThicknessFromSample(sampleT)
    ));
    float cortexEdge = length(vec2(sampleR.b - sampleL.b, sampleB.b - sampleT.b));

    float depthEdge = smoothstep(0.02, 0.2, thicknessEdge);
    float transportEdge = smoothstep(0.006, 0.095, pathEdge);
    float corticalShellEdge = smoothstep(0.0025, 0.045, cortexEdge);
    return saturate(max(depthEdge * 0.34, max(transportEdge * 0.38, corticalShellEdge * 0.42)));
}

float attenuationAt(vec2 uv, vec4 bonePaths, float corticalEdge) {
    float boneVisibility = pow(saturate(boneOpacity), 0.55);
    float corticalAbsorption = pow(saturate(bonePaths.y * 1.6), 0.96) * 0.84;
    float edgeAbsorption = corticalEdge * 0.16;
    float cancellousAbsorption = pow(saturate(bonePaths.z), 0.82) * bonePaths.w * 0.34;
    float layeredAbsorption = pow(saturate(bonePaths.x), 0.72) * 0.66;
    float softBoneAbsorption = smoothstep(0.01, 0.72, bonePaths.x) * 0.48;
    float rawBoneSignal = corticalAbsorption + edgeAbsorption + cancellousAbsorption + layeredAbsorption + softBoneAbsorption;
    float boneSignal = 1.0 - exp(-rawBoneSignal * 1.08);
    float bone = boneSignal * 1.58 * boneVisibility;
    float iodine = contrastAt(uv) * saturate(contrastOpacity) * 3.25;
    float metal = metalAt(uv) * 5.25;
    float catheter = catheterAt(uv) * 0.28;
    float sheath = sheathAt(uv) * 0.42;

    // The accumulated visible frame creates detector persistence across the
    // full fluoroscopy image while current attenuation still leads the frame.
    float temporalTrace = smoothstep(0.025, 0.72, sampleSignal(uTexture, uv)) * 0.46;
    return max(0.0, bone + iodine + metal + catheter + sheath + temporalTrace);
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
    float lowerBody = 1.0 - smoothstep(0.42, 1.18, length(centered * vec2(0.72, 1.05)));
    float trunk = 1.0 - smoothstep(0.35, 1.08, length((centered - vec2(0.0, -0.18)) * vec2(0.62, 1.35)));
    return saturate(max(lowerBody, trunk * 0.72));
}

float scatterFieldAt(vec2 uv, float attenuation, float bonePath) {
    float tissuePath = patientBodyField(uv);
    float projectedPath = saturate(thicknessPathAt(uv) * 0.55 + bonePath * 0.26 + tissuePath * 0.22);
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

float roadmapVesselAt(vec2 uv) {
    float storedRoadmapLuma = texture2D(roadmapTexture, uv).r /
        max(0.001, gray.r * 0.992);
    float roadmapDifference = max(0.0, 0.94 - storedRoadmapLuma);
    return smoothstep(0.032, 0.34, roadmapDifference);
}

float roadmapEdgeAt(vec2 uv) {
    vec2 texel = 1.0 / resolution;
    float tl = roadmapVesselAt(uv + texel * vec2(-1.0, -1.0));
    float t  = roadmapVesselAt(uv + texel * vec2(0.0, -1.0));
    float tr = roadmapVesselAt(uv + texel * vec2(1.0, -1.0));
    float l  = roadmapVesselAt(uv + texel * vec2(-1.0, 0.0));
    float r  = roadmapVesselAt(uv + texel * vec2(1.0, 0.0));
    float bl = roadmapVesselAt(uv + texel * vec2(-1.0, 1.0));
    float b  = roadmapVesselAt(uv + texel * vec2(0.0, 1.0));
    float br = roadmapVesselAt(uv + texel * vec2(1.0, 1.0));
    float gx = -tl - 2.0 * l - bl + tr + 2.0 * r + br;
    float gy = -tl - 2.0 * t - tr + bl + 2.0 * b + br;
    return smoothstep(0.08, 1.4, length(vec2(gx, gy)));
}

void main() {
    vec4 tex = texture2D(uTexture, vUv);
    if (fluoroscopy) {
        vec4 centerBonePaths = boneLayerPathsAt(vUv);
        float centerCorticalEdge = corticalEdgeAt(vUv);
        float centerAttenuation = attenuationAt(vUv, centerBonePaths, centerCorticalEdge);
        float localScatter = scatterFieldAt(vUv, centerAttenuation, centerBonePaths.x);
        float exposureLift = autoExposureEnabled ? autoExposureLevel : 0.0;

        // C-arm images are usually edge-enhanced after acquisition. Sharpen
        // attenuation before transmission. Screen-space derivatives preserve
        // local radiopaque borders without four full neighboring attenuation
        // evaluations per detector pixel.
        float scatterSoftenedEdge = mix(0.34, 0.16, localScatter);
        float sharpenedAttenuation = max(
            0.0,
            centerAttenuation + fwidth(centerAttenuation) * edgeStrength * scatterSoftenedEdge
        );

        float transmission = exp(-sharpenedAttenuation);
        float scatterFog = saturate(centerAttenuation * 0.025 + localScatter * 0.24);
        transmission = mix(transmission, 0.55 + exposureLift * 0.12, scatterFog);

        // Detector window/level with a soft shoulder. This keeps the air field
        // from becoming pure white and gives dense contrast a real black floor.
        float luma = pow(saturate(transmission), 0.88);
        luma = smoothstep(0.025, 0.975, luma);
        luma = mix(0.045, 0.72, luma);

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
            * (0.08 + 0.18 * sqrt(max(luma, 0.0)));

        luma = saturate(luma * field + fixedPattern + columnPattern + gridPattern + mottle + pulseJitter);
        luma = mix(luma, 0.50 + (luma - 0.50) * 0.68, localScatter * 0.22);
        luma = saturate(luma + exposureLift);
        luma = saturate((luma - 0.5) * max(0.0, imageContrast) + 0.5 + imageBrightness);
        float beamMask = collimatorMask(vUv);
        luma = mix(0.018, luma, beamMask);

        if (dsaEnabled && dsaMaskValid) {
            // The mask stores the fully processed pre-contrast detector frame.
            // Iodine reduces current detector brightness, so mask-current is
            // the positive logarithmic-subtraction cue used by the DSA view.
            float storedMaskLuma = texture2D(dsaMaskTexture, vUv).r /
                max(0.001, gray.r * 0.992);
            float subtraction = max(storedMaskLuma - luma - 0.004, 0.0);
            float vesselSignal = saturate(subtraction * max(0.0, dsaGain) * 4.0);
            float dsaLuma = mix(0.94, 0.025, vesselSignal);
            luma = mix(0.018, dsaLuma, beamMask);
        } else if (roadmapEnabled && roadmapValid) {
            // A roadmap target stores a DSA frame (light background, dark
            // opacified vessels). Convert its darkness to a fixed vessel mask
            // and use that mask to brighten the corresponding vessels in the
            // current live fluoroscopy.
            // Reject subtraction noise before turning the saved DSA frame into
            // a persistent vessel stencil. True iodine-filled vessels are much
            // darker than the 2-3% detector mottle around the DSA background.
            float roadmapVessel = roadmapVesselAt(vUv) * beamMask;
            float roadmapEdge = roadmapEdgeAt(vUv) * beamMask;
            // Background visibility is independent from the vessel overlay.
            // At 0% retain the bright subtraction field inside the beam; at
            // 100% retain the full live fluoroscopic anatomy.
            float neutralRoadmapBackground = mix(0.018, 0.94, beamMask);
            float visibleRoadmapBackground = mix(
                neutralRoadmapBackground,
                luma,
                saturate(roadmapBackgroundVisibility)
            );
            float roadmapLayer = mix(
                visibleRoadmapBackground,
                1.0,
                roadmapVessel
            );
            // A Sobel contour sharpens the saved vessel mask. Its color can be
            // pulled slightly below the live background to create a subtle,
            // adjustable dark border around the bright roadmap vessel.
            float roadmapEdgeTarget = visibleRoadmapBackground * (
                1.0 - saturate(roadmapEdgeDarkness) * 0.7
            );
            roadmapLayer = mix(
                roadmapLayer,
                roadmapEdgeTarget,
                roadmapEdge * saturate(roadmapEdgeEnhancement)
            );
            luma = mix(
                visibleRoadmapBackground,
                roadmapLayer,
                saturate(roadmapOpacity)
            );
        }

        if (cineEnabled) {
            // Archived DSA frames use the same native detector resolution as
            // the live display. Reconstruct their stored grayscale directly,
            // without another subtraction, denoise, or resize pass.
            luma = texture2D(cineTexture, vUv).r /
                max(0.001, gray.r * 0.992);
        }

        // Phosphor/detector response is slightly warm-neutral, not mathematically
        // flat grayscale. Keep it subtle so it still reads as fluoroscopy.
        vec3 detectorTint = vec3(0.992, 0.992, 0.988);
        gl_FragColor = vec4(gray * detectorTint * luma, 1.0);
    } else {
        // Debug mode: keep original color, use edge to boost alpha.
        float edge = edgeFactor(vUv) * edgeStrength;
        float alpha = clamp(tex.a + edge, 0.0, 1.0);
        gl_FragColor = vec4(tex.rgb, alpha);
    }
}
`;
