export const vertexShader = `
// Fullscreen quad vertex shader for thickness pass.
// Renders a screen-aligned quad; vUv is used to sample depth textures.
varying vec2 vUv;
void main() {
    vUv = uv;
    gl_Position = vec4(position.xy, 0.0, 1.0);
}
`;

export const fragmentShader = `
// Thickness estimation shader.
// Uses front and back depth renders of opaque geometry to estimate
// per-pixel thickness along the view ray: thickness = back - front.
// This can be used to approximate X-ray attenuation or soft shading.
uniform sampler2D frontDepth;
uniform sampler2D backDepth;
varying vec2 vUv;
void main() {
    float front = texture2D(frontDepth, vUv).r;
    float back = texture2D(backDepth, vUv).r;
    float thick = max(back - front, 0.0);

    gl_FragColor = vec4(vec3(thick), 1.0);
}
`;

