// Static accelerator for the existing right-continuous GPU binary search.
// Entries refer to the original sample records: no field resampling or
// concentration interpolation is introduced here.
export function createFlowColumnLookup(integral, pathCount) {
    let bucketCount = 0;
    for (const range of integral.ranges.values()) bucketCount += range.count;
    const data = new Float32Array((pathCount + bucketCount) * 4);
    const samples = integral.currentData;
    let next = pathCount;
    function predecessor(range, coordinate) {
        let low = range.start, high = range.start + range.count;
        while (low < high) {
            const middle = (low + high) >>> 1;
            if (samples[middle * 4] <= coordinate) low = middle + 1;
            else high = middle;
        }
        return low - 1;
    }
    for (const [path, range] of integral.ranges) {
        const count = range.count, last = range.start + count - 1;
        const length = Math.fround(range.length), step = Math.fround(length / count);
        // Tiny/degenerate paths retain the full original search. Normal
        // anatomy has positive millimetre-scale length and at least 3 samples.
        data.set([next, count, step, 0], path * 4);
        for (let bucket = 0; bucket < count; bucket++) {
            // Include neighbouring bins so Float32 division at a bucket
            // boundary cannot exclude the correct duplicate-coordinate seam.
            // The final binary search still chooses exactly the old indices.
            const left = (bucket - 1) * step, right = (bucket + 2) * step;
            const lo = step > 1e-8 ? Math.max(range.start, Math.min(last - 1, predecessor(range, left))) : range.start;
            const hi = step > 1e-8 ? Math.min(last, Math.max(lo + 1, predecessor(range, right) + 1)) : last;
            data.set([lo, hi, 0, 0], next * 4);
            next++;
        }
    }
    return {data, pathCount, bucketCount, memoryBytes: data.byteLength};
}
