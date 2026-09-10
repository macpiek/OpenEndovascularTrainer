// Only pass buffers whose entire owner has finished using them. In modern
// browsers transfer(0) discards their backing storage and detaches all views;
// retaining an old JS wrapper can no longer retain the large byte allocation.
// Older engines still use normal garbage collection after references are cleared.
export function releaseOwnedBuffers(values) {
    const buffers = new Set();
    for (const value of values) {
        const buffer = ArrayBuffer.isView(value) ? value.buffer : value;
        if (buffer instanceof ArrayBuffer) buffers.add(buffer);
    }
    for (const buffer of buffers) {
        if (buffer.byteLength && typeof buffer.transfer === 'function') buffer.transfer(0);
    }
}
