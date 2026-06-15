export const vertexShader = `
// Fullscreen quad vertex shader.
// Passes through the quad UVs and positions to draw a screen-aligned quad.
varying vec2 vUv;
void main() {
    vUv = uv;
    gl_Position = vec4(position.xy, 0.0, 1.0);
}
`;

export const fragmentShader = `
// Persistence blend (accumulation) shader.
// Combines the newly rendered frame (currentFrame) with the previous
// accumulated frame (previousFrame) using exponential decay. This
// produces a fluoroscopy-like persistence trail over time.
uniform sampler2D currentFrame;
uniform sampler2D previousFrame;
uniform float decay;
varying vec2 vUv;
void main() {
    vec4 prev = texture2D(previousFrame, vUv);
    vec4 curr = texture2D(currentFrame, vUv);
    gl_FragColor = curr + prev * decay;
}
`;

