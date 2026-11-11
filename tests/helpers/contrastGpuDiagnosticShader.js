// Deliberately nonphysical shader variants for the performance fixture only.
// Never import this module from src/. Every variant starts from an immutable
// production source, so switching modes cannot accumulate replacements.
import {contrastColumnQuadratureReference} from './contrastColumnQuadratureReference.js';
import {columnNearestReference} from './columnNearestReference.js';
import {columnSampleHistoryReference, columnPathConstantHistoryReference} from './columnHistoryReference.js';

export const contrastGpuDiagnosticModes = Object.freeze({
    full: 'Pełna projekcja produkcyjna',
    referenceNearest: 'Referencyjna projekcja na oś bez pamięci segmentu',
    referenceHistory: 'Referencyjne odczyty obu historii stężenia',
    referenceStack: 'Referencyjny stos całkowania',
    wideCandidates: 'Bufor16 kandydatów',
    referenceWide: 'Referencyjny stos i bufor16 kandydatów',
    cacheOverflow: 'Maska przekroczenia bufora4 kandydatów',
    rasterOriginal: 'Pełna projekcja z dawną deformacją wierzchołków',
    noRayCache: 'Pełna projekcja bez pamięci podręcznej promienia',
    rasterOnly: 'Sam raster (bez zapytania BVH)',
    exitOnly: 'Raster + dokładne wyjście BVH',
    noMix: 'BVH + całka osiowa (bez mieszania przestrzennego)',
    junctionMask: 'BVH + maska obszarów mieszania',
    noRestDeformation: 'Pełna projekcja bez odwrotnej deformacji próbek',
    uncached: 'Pełna projekcja bez pamięci podręcznej punktów'
});

function functionBody(source, signature) {
    const start = source.indexOf(signature);
    if (start < 0 || source.indexOf(signature, start + signature.length) >= 0)
        throw Error(`Expected exactly one shader function: ${signature}`);
    const open = source.indexOf('{', start);
    let depth = 1, end = open + 1;
    for (; end < source.length && depth; end++) {
        if (source[end] === '{') depth++;
        else if (source[end] === '}') depth--;
    }
    if (depth) throw Error(`Unbalanced shader function: ${signature}`);
    return {start, open, close: end - 1};
}

function replaceRequired(source, from, to) {
    if (!source.includes(from) || source.indexOf(from) !== source.lastIndexOf(from))
        throw Error(`Expected exactly one diagnostic shader marker: ${from}`);
    return source.replace(from, to);
}

export function contrastGpuDiagnosticShader(source, mode = 'full') {
    if (!Object.hasOwn(contrastGpuDiagnosticModes, mode)) throw Error(`Unknown GPU diagnostic mode: ${mode}`);
    if (['full', 'rasterOriginal', 'noRayCache'].includes(mode)) return source;
    if (mode === 'referenceNearest') {
        const {start, close} = functionBody(source, 'float columnNearestRefined(');
        return source.slice(0, start) + columnNearestReference.trim() + source.slice(close + 1);
    }
    if (mode === 'referenceHistory') {
        for (const [signature, reference] of [['vec4 columnSample(', columnSampleHistoryReference],
            ['bool columnPathConstant(', columnPathConstantHistoryReference]]) {
            const {start, close} = functionBody(source, signature);
            source = source.slice(0, start) + reference.trim() + source.slice(close + 1);
        }
        return source;
    }
    if (mode === 'referenceStack' || mode === 'referenceWide') {
        const {start, close} = functionBody(source, 'float junctionColumnAverage(');
        source = source.slice(0, start) + contrastColumnQuadratureReference.trim() + source.slice(close + 1);
        if (mode === 'referenceStack') return source;
    }
    if (mode === 'wideCandidates' || mode === 'referenceWide') {
        const {open, close} = functionBody(source, 'vec3 junctionColumnValue(');
        let body = source.slice(open + 1, close);
        body = replaceRequired(body, 'vec3 candidates[4]', 'vec3 candidates[16]');
        body = replaceRequired(body, 'if (count == 4)', 'if (count == 16)');
        body = replaceRequired(body, 'for (int i = 0; i < 4; i++)', 'for (int i = 0; i < 16; i++)');
        return source.slice(0, open + 1) + body + source.slice(close);
    }
    if (mode === 'cacheOverflow') {
        source = replaceRequired(source,
            'if (count == 4) return junctionColumnValueUncached(point, a, sa, b, sb, componentA, componentB);',
            'if (count == 4) { diagnosticCacheOverflow = true; return junctionColumnValueUncached(point, a, sa, b, sb, componentA, componentB); }');
        source = 'bool diagnosticCacheOverflow = false;\n' + source;
        const {open, close} = functionBody(source, 'void main()');
        let body = source.slice(open + 1, close);
        body = replaceRequired(body, '    float depth =',
            '    if (!diagnosticCacheOverflow) discard;\n    float depth =');
        // Retain the physical iodine value of each affected ray, including
        // its exact geometry and unchanged fallback calculation. This counts
        // iodine-visible pixels which required the uncapped path at least once.
        return source.slice(0, open + 1) + body + source.slice(close);
    }
    if (mode === 'uncached') {
        const signature = 'float junctionColumnAverage(';
        functionBody(source, signature);
        return source.replace(signature, '#define junctionColumnValue junctionColumnValueUncached\n' + signature);
    }
    const signature = mode === 'noRestDeformation' ? 'vec3 columnRestPosition(' : 'void main()';
    const {open, close} = functionBody(source, signature);
    if (mode === 'rasterOnly') return source.slice(0, open + 1) +
        '\n    gl_FragColor = vec4(.05); // DIAGNOSTIC: raster coverage, not iodine.\n' + source.slice(close);
    if (mode === 'noRestDeformation') return source.slice(0, open + 1) +
        '\n    return point; // DIAGNOSTIC: inverse displacement disabled.\n' + source.slice(close);
    // Preserve the detector ray, active-volume cull, exact BVH exit, retries,
    // chord length and radiographic transfer. Only the scalar evaluation is
    // replaced. Throw on source changes instead of silently timing full work.
    const body = source.slice(open + 1, close);
    const from = body.indexOf('    float mean;'), to = body.indexOf('    float depth =');
    if (from < 0 || to <= from) throw Error('Missing production column mean-selection block');
    const replacement = mode === 'noMix'
        ? '    float mean = columnAverage(entryPath, entryColumn.y, exitPath, exitS);\n'
        : mode === 'junctionMask'
            ? '    if (!columnHasJunction(entryPosition, ray, chord, entryPath, exitPath)) discard;\n    float mean = .3;\n'
            : '    float mean = .3; // DIAGNOSTIC: uniform iodine, exact lumen chord.\n';
    return source.slice(0, open + 1) + body.slice(0, from) + replacement + body.slice(to) + source.slice(close);
}
