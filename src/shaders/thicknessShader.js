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
// Uses front and back depth renders to estimate per-pixel path length through
// the rendered anatomy. MeshDepthMaterial depth direction can vary with the
// packing path, so the signed front/back delta is treated as a magnitude here.
uniform sampler2D frontDepth;
uniform sampler2D backDepth;
varying vec2 vUv;
void main() {
    float front = texture2D(frontDepth, vUv).r;
    float back = texture2D(backDepth, vUv).r;
    float thick = abs(back - front);

    gl_FragColor = vec4(vec3(thick), 1.0);
}
`;
