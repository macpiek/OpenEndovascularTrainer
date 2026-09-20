import {ANATOMY_VARIANTS,resolveAnatomyVariant,anatomyVariantUrl} from '../anatomyVariant.js';

export function initAnatomyControls({select,button,description,status,href,navigate}) {
    const current=resolveAnatomyVariant(new URL(href).search);
    select.value=current.id;
    status.textContent=`Aktualna anatomia: ${current.label}`;
    const update=()=>{
        const variant=ANATOMY_VARIANTS[select.value] ?? current;
        description.textContent=variant.description;
        button.disabled=variant.id===current.id;
        button.textContent=variant.id==='baseline'
            ? 'Przywróć model bazowy i zresetuj scenę'
            : 'Wygeneruj tętniaka i zresetuj scenę';
    };
    select.addEventListener('change',update);
    button.addEventListener('click',()=>{
        if(select.value===current.id)return;
        button.disabled=true;
        status.textContent='Wczytywanie nowej anatomii…';
        navigate(anatomyVariantUrl(href,select.value));
    });
    update();
}
