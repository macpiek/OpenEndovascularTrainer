export const DEBUG_SOLVERS = Object.freeze([
    {id:'shared-axis', label:'Kirchhoff — siatka 5 mm (referencyjny)'},
    {id:'shared-axis-adaptive', label:'Kirchhoff — siatka adaptacyjna (eksperymentalny)'},
    {id:'shared-axis-realtime', label:'Kirchhoff — eksperyment 60 Hz (w rozwoju)'},
    {id:'shared-axis-projective', label:'Projective Dynamics — pręt podatny, siatka adaptacyjna (eksperymentalny)'},
    {id:'joint-active-coulomb', label:'Starszy sprzężony Kirchhoff / Coulomb'},
    {id:'reference', label:'Starszy bezpośredni Kirchhoff'}
]);

export function solverDebugUrl(href, solver, modifiedNewton, pruneWitnesses, fastNewton, predictiveNewton) {
    if (!DEBUG_SOLVERS.some(s => s.id === solver)) throw new RangeError('Unknown debug solver');
    const url = new URL(href);
    url.searchParams.set('coupledSolver', solver);
    url.searchParams.set('solverDebug', '1');
    if(modifiedNewton!==undefined){if(modifiedNewton)url.searchParams.set('modifiedNewton','1');else url.searchParams.delete('modifiedNewton');}
    if(pruneWitnesses!==undefined){if(pruneWitnesses)url.searchParams.set('pruneWitnesses','1');else url.searchParams.delete('pruneWitnesses');}
    if(fastNewton!==undefined){if(fastNewton)url.searchParams.set('fastNewton','1');else url.searchParams.set('fastNewton','0');}
    if(predictiveNewton!==undefined)url.searchParams.set('predictiveNewton',predictiveNewton?'1':'0');
    url.searchParams.delete('experimentalSplitMotion');
    url.searchParams.delete('coupledLinearSolver');
    return url.href;
}

export function initSolverDebugControls({select, button, current, navigate, href, modifiedNewtonToggle, pruneWitnessesToggle, fastNewtonToggle, predictiveNewtonToggle, tolerance, toleranceOutput, onToleranceChange, contactMargin, contactMarginOutput, onContactMarginChange, maxArcLoss, maxArcLossOutput, onMaxArcLossChange, maxSpacing, maxSpacingOutput, onMaxSpacingChange}) {
    if (!select || !button) return;
    const choices = DEBUG_SOLVERS.some(s => s.id === current) ? DEBUG_SOLVERS :
        [...DEBUG_SOLVERS, {id:current, label:`Bieżący: ${current}`}];
    for (const choice of choices) {
        const option = select.ownerDocument.createElement('option');
        option.value = choice.id; option.textContent = choice.label; select.append(option);
    }
    select.value = current;
    const originalModified=new URL(href).searchParams.get('modifiedNewton')==='1';
    if(modifiedNewtonToggle)modifiedNewtonToggle.checked=originalModified;
    const originalPruned=new URL(href).searchParams.get('pruneWitnesses')==='1';
    if(pruneWitnessesToggle)pruneWitnessesToggle.checked=originalPruned;
    const originalFast=new URL(href).searchParams.get('fastNewton')!=='0';
    if(fastNewtonToggle)fastNewtonToggle.checked=originalFast;
    const originalPredictive=new URL(href).searchParams.get('predictiveNewton')!=='0';
    if(predictiveNewtonToggle)predictiveNewtonToggle.checked=originalPredictive;
    const refreshChoice=()=>{
        if(modifiedNewtonToggle)modifiedNewtonToggle.disabled=!['shared-axis','shared-axis-adaptive','shared-axis-realtime'].includes(select.value);
        if(pruneWitnessesToggle)pruneWitnessesToggle.disabled=!['shared-axis','shared-axis-adaptive','shared-axis-realtime'].includes(select.value);
        if(fastNewtonToggle)fastNewtonToggle.disabled=!['shared-axis','shared-axis-adaptive','shared-axis-realtime'].includes(select.value);
        if(predictiveNewtonToggle)predictiveNewtonToggle.disabled=!['shared-axis','shared-axis-adaptive','shared-axis-realtime'].includes(select.value)||!!fastNewtonToggle&&!fastNewtonToggle.checked;
        button.disabled=(!predictiveNewtonToggle||predictiveNewtonToggle.checked===originalPredictive)&&(!fastNewtonToggle||fastNewtonToggle.checked===originalFast)&&select.value===current&&(!modifiedNewtonToggle||modifiedNewtonToggle.checked===originalModified)&&(!pruneWitnessesToggle||pruneWitnessesToggle.checked===originalPruned);
    };
    refreshChoice();
    modifiedNewtonToggle?.addEventListener('change',refreshChoice);
    pruneWitnessesToggle?.addEventListener('change',refreshChoice);
    fastNewtonToggle?.addEventListener('change',refreshChoice);
    predictiveNewtonToggle?.addEventListener('change',refreshChoice);
    for (const [slider, output, onChange, decimals, suffix] of [
        [tolerance, toleranceOutput, onToleranceChange, 2, ' mm'],
        [contactMargin, contactMarginOutput, onContactMarginChange, 2, ' mm'],
        [maxArcLoss, maxArcLossOutput, onMaxArcLossChange, 2, ' %'],
        [maxSpacing, maxSpacingOutput, onMaxSpacingChange, 0, ' mm']
    ]) {
        if (!slider) continue;
        slider.disabled = !['shared-axis-adaptive','shared-axis-projective','shared-axis-realtime'].includes(current);
        const updateLabel = () => {
            if (output) output.textContent = `${Number(slider.value).toFixed(decimals).replace('.', ',')}${suffix}`;
        };
        updateLabel();
        slider.addEventListener('input', () => {
            if (slider.disabled) return;
            updateLabel();
            onChange?.(Number(slider.value));
        });
    }
    select.addEventListener('change',refreshChoice);
    button.addEventListener('click', () => navigate(solverDebugUrl(href,select.value,modifiedNewtonToggle?modifiedNewtonToggle.checked&&!modifiedNewtonToggle.disabled:undefined,pruneWitnessesToggle?pruneWitnessesToggle.checked&&!pruneWitnessesToggle.disabled:undefined,fastNewtonToggle?fastNewtonToggle.checked&&!fastNewtonToggle.disabled:undefined,predictiveNewtonToggle?predictiveNewtonToggle.checked:undefined)));
}
