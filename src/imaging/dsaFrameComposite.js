/**
 * Builds a minimum-intensity projection from grayscale DSA frames. DSA stores
 * opacified vessels as dark pixels, so the minimum at each detector pixel
 * preserves the strongest vessel signal seen anywhere in the selected range.
 */
export function composeDsaFrameRangeRedChannels(redChannels) {
    if (!Array.isArray(redChannels) || !redChannels.length) return null;
    const first = redChannels[0];
    if (!first?.length) return null;
    const composite = new Uint8Array(first);
    for (let frameIndex = 1; frameIndex < redChannels.length; frameIndex++) {
        const channel = redChannels[frameIndex];
        if (!channel || channel.length !== composite.length) return null;
        for (let pixelIndex = 0; pixelIndex < composite.length; pixelIndex++) {
            composite[pixelIndex] = Math.min(
                composite[pixelIndex],
                channel[pixelIndex]
            );
        }
    }
    return composite;
}
