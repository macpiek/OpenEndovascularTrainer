import {AORTA_MODEL_URL,AORTA_COLLISION_URL} from './aortaTransform.js';

export const ANATOMY_VARIANTS = Object.freeze({
    baseline: Object.freeze({id:'baseline',label:'Model bazowy',
        description:'Oryginalna aorta bez dodatkowego tętniaka.',
        modelUrl:AORTA_MODEL_URL,collisionUrl:AORTA_COLLISION_URL}),
    'infrarenal-aneurysm': Object.freeze({id:'infrarenal-aneurysm',label:'Tętniak aorty podnerkowej',
        focusY:-280,
        description:'Poszerzenie światła do około 50 mm, od odcinka podnerkowego aż do rozwidlenia aorty, z płynnym przejściem do tętnic biodrowych. Wariant bez skrzepliny.',
        modelUrl:new URL('../res/Aorta_infrarenal_aneurysm.stl',import.meta.url).href,
        collisionUrl:new URL('../res/Aorta_infrarenal_aneurysm.collision.bin',import.meta.url).href})
});

export function resolveAnatomyVariant(search='') {
    const id=new URLSearchParams(search).get('anatomy');
    return Object.hasOwn(ANATOMY_VARIANTS,id) ? ANATOMY_VARIANTS[id] : ANATOMY_VARIANTS.baseline;
}
export function anatomyVariantUrl(href,id) {
    if(!Object.hasOwn(ANATOMY_VARIANTS,id))throw Error('Unknown anatomy variant');
    const url=new URL(href);
    if(id==='baseline')url.searchParams.delete('anatomy');
    else url.searchParams.set('anatomy',id);
    url.searchParams.set('panel','anatomy');
    return url.href;
}
