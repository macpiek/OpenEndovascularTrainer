import {MAIN_BODY_MODELS,bodyDimensions,proximalDiameters,distalDiameters} from '../devices/stentGraftModels.js';
import {catalogueGeometry} from './stentGraftModelGeometry.js';

const drawings=new Map(),number=n=>Number(n.toFixed(3));
// Every card uses the same millimetre scale on BOTH axes, across bodies and limbs.
function drawing(device) {
    const key=[device.type,device.modelId,device.diameter,device.distalDiameter,device.length].join(':');
    if(drawings.has(key))return drawings.get(key);
    const geometry=catalogueGeometry(device),point=p=>`${number(p.x)} ${number(p.y)}`;
    const path=points=>points.map((p,i)=>`${i?'L':'M'}${point(p)}`).join('');
    const segments=list=>list.map(([a,b])=>`M${point(a)}L${point(b)}`).join('');
    const d=device.type==='body'?bodyDimensions(device):null;
    const distalX=d?-d.gateDiameter/2:0;
    const measure=(x,y,width,label)=>`<path d="M${x-width/2} ${y}h${width}m${-width} -1.5v3m${width} -3v3"/><text stroke="none" x="${x}" y="${y-3}">${label}</text>`;
    const svg=`<svg viewBox="-55 -30 110 245" aria-hidden="true" data-proximal-mm="${device.diameter}" data-distal-mm="${device.distalDiameter}" data-length-mm="${device.length}"><g fill="#87a9b9" fill-opacity=".16" stroke="#96b1c3" stroke-width=".2">${geometry.silhouettes.map(p=>`<path d="${path(p)}Z"/>`).join('')}</g><path d="${segments(geometry.wires)}" fill="none" stroke="#d7e2ec" stroke-opacity=".55" stroke-width=".16" stroke-linecap="round"/><path d="${segments(geometry.markers)}" fill="none" stroke="#49ebda" stroke-width=".65" stroke-linecap="round"/><g fill="#cde0ed" stroke="#a6c4d8" stroke-width=".25" font-size="5" text-anchor="middle"><g stroke="none"><text x="0" y="-24">Ø ${device.diameter} mm</text></g><path d="M${-device.diameter/2} -20h${device.diameter}m${-device.diameter} -1.5v3m${device.diameter} -3v3"/>${measure(distalX,device.length+11,device.distalDiameter,`Ø ${device.distalDiameter}`)}${d?measure(d.ipsiRootDiameter/2,d.gateLength+11,d.gateDiameter,`Ø ${d.gateDiameter}`):''}<path d="M40 0v${device.length}M38 0h4M38 ${device.length}h4"/><text stroke="none" x="46" y="${device.length/2}" transform="rotate(90 46 ${device.length/2})">${device.length} mm</text></g></svg>`;
    drawings.set(key,svg);return svg;
}
export function cardDimensions(model,diameter=23,distalDiameter=14) {
    if(!proximalDiameters(model.id).includes(diameter))diameter=23;
    const allowed=distalDiameters(model.id,diameter);
    if(!allowed.includes(distalDiameter))distalDiameter=allowed.includes(16)?16:allowed[0];
    return {type:'body',modelId:model.id,length:model.length,diameter,distalDiameter};
}
export function modelDrawing(model,diameter=23,distalDiameter=14) {return drawing(cardDimensions(model,diameter,distalDiameter));}
export function mainBodyCardContent(model,diameter=23,distalDiameter=14) {
    const d=cardDimensions(model,diameter,distalDiameter);
    return `${drawing(d)}<strong>${model.name}</strong><span>Ø ${d.diameter} / ${d.distalDiameter} mm · ${model.length} mm</span>`;
}
export function mainBodyCards() {
    return MAIN_BODY_MODELS.map(m=>`<button type="button" id="graftModel-${m.id}" class="stent-graft-model" aria-pressed="false" aria-label="${m.name}, korpus ${m.length} mm">${mainBodyCardContent(m)}</button>`).join('');
}
export function limbDrawing(model) {return drawing({...model,modelId:model.id,type:'limb'});}
export function limbCards(models) {
    return models.map(m=>`<button type="button" id="graftLimb-${m.id}" class="stent-graft-model" aria-pressed="false" aria-label="Endurant II / IIs, nóżka 16 na ${m.distalDiameter} mm, długość ${m.length} mm, ${m.deliveryFr} Fr">${limbDrawing(m)}<strong>16 → ${m.distalDiameter} mm</strong><span>${m.length} mm · ${m.deliveryFr} Fr</span><small>${m.id}</small></button>`).join('');
}
