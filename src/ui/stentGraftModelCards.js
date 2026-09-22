import {MAIN_BODY_MODELS} from '../devices/stentGraftModels.js';
// Drawings use one shared scale, so main-body length is directly comparable.
export function modelDrawing(model) {
    const scale=.95,top=26,crotch=top+model.trunkLength*scale,end=top+model.length*scale,gate=top+model.gateLength*scale;
    const ring=(x,y,width)=>`M ${x} ${y} ${x+width/4} ${y+6} ${x+width/2} ${y} ${x+3*width/4} ${y+6} ${x+width} ${y}`;
    let wires='';
    for(let y=top+2;y<crotch-7;y+=9)wires+=`<path d="${ring(33,y,34)}"/>`;
    for(const [x,limit]of [[33,end],[51,gate]])for(let y=crotch+2;y<limit-7;y+=9)wires+=`<path d="${ring(x,y,16)}"/>`;
    return `<svg viewBox="0 0 115 210" aria-hidden="true"><g fill="#87a9b9" fill-opacity=".35" stroke="#96b1c3" stroke-width=".8"><path d="M33 ${top}H67V${gate}H51V${crotch}H49V${end}H33Z"/></g><g fill="none" stroke="#d7e2ec" stroke-width="1" stroke-linejoin="round">${wires}<path d="M33 ${top}L36 12L41 ${top}L46 12L51 ${top}L56 12L61 ${top}L65 12L67 ${top}"/></g><path d="M51 ${gate}H67" stroke="#49ebda" stroke-width="3"/><g stroke="#a6c4d8" stroke-width=".7"><path d="M85 ${top}V${end}M81 ${top}H89M81 ${end}H89"/></g><text x="91" y="${(top+end)/2}" fill="#cde0ed" font-size="9" transform="rotate(90 91 ${(top+end)/2})">${model.length} mm</text></svg>`;
}
export function mainBodyCards() {
    return MAIN_BODY_MODELS.map(m=>`<button type="button" id="graftModel-${m.id}" class="stent-graft-model" aria-pressed="false" aria-label="${m.name}, korpus ${m.length} mm">${modelDrawing(m)}<strong>${m.name}</strong><span>${m.length} mm</span></button>`).join('');
}
