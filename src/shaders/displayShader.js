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
// - In fluoroscopy mode: interprets the accumulated buffer as intensity,
//   adds procedural noise, mixes contrast agent texture, and inverts to
//   a grayscale X-ray look with adjustable edge emphasis.
// - In wireframe mode: just visualizes the scene with edge-enhanced alpha.
uniform sampler2D uTexture;
uniform sampler2D contrastTexture;
uniform vec3 gray;
uniform bool fluoroscopy;
uniform float time;
uniform float noiseLevel;
uniform float boneOpacity;
uniform vec2 resolution;
uniform float edgeStrength;
varying vec2 vUv;

// Hash-based noise; time-varying for animated grain.
float random(vec2 st) {
    return fract(sin(dot(st.xy, vec2(12.9898, 78.233)) + time) * 43758.5453123);
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
    float edge = edgeFactor(vUv) * edgeStrength;
    if (fluoroscopy) {
        // Convert accumulated intensity to inverted grayscale appearance.
        float intensity = tex.r * boneOpacity;
        float noise = random(vUv * 100.0) - 0.5;
        intensity += noise * noiseLevel;
        intensity = clamp(intensity, 0.0, 1.0);
        // Contrast agent (additive brightening) sampled separately and
        // mixed into the image.
        vec4 cSample = texture2D(contrastTexture, vUv);
        float contrast = clamp((cSample.r + cSample.b) * 2.0, 0.0, 1.0);
        vec3 color = gray * (1.0 - intensity);
        float alpha = clamp(1.0 + edge, 0.0, 1.0);
        gl_FragColor = vec4(mix(color, vec3(0.0), contrast), alpha);
    } else {
        // Wireframe mode: keep original color, use edge to boost alpha.
        float alpha = clamp(tex.a + edge, 0.0, 1.0);
        gl_FragColor = vec4(tex.rgb, alpha);
    }
}
`;

