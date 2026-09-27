import {MAIN_BODY_MODELS,nominalLimbRadius} from '../devices/stentGraftModels.js';
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

// Schematic, common-scale drawings: proximal diameter is 16 mm; the distal
// end can narrow, remain straight, or flare. These are not manufacturer CAD.
export function limbDrawing(model) {
    const top=20,scale=.72,end=top+model.length*scale,x=48;
    const radius=y=>nominalLimbRadius(model,(y-top)/scale)*1.5;
    const outline=[];
    for(let i=0;i<=model.length;i++)outline.push([radius(top+i*scale),top+i*scale]);
    const silhouette=outline.map(([r,y],i)=>`${i?'L':'M'}${x-r} ${y}`).join(' ')+
        outline.slice().reverse().map(([r,y])=>`L${x+r} ${y}`).join(' ')+'Z';
    let wires='';
    for(let y=top+3;y<end-6;y+=9) {
        const r=radius(y);
        wires+=`<path d="M${x-r} ${y} q${r/4} 9 ${r/2} 0 t${r/2} 0 t${r/2} 0 t${r/2} 0"/>`;
    }
    return `<svg viewBox="0 0 116 190" aria-hidden="true"><path d="${silhouette}" fill="#87a9b9" fill-opacity=".3" stroke="#96b1c3" stroke-width=".8"/><g fill="none" stroke="#d7e2ec" stroke-width=".8">${wires}</g><path d="M${x-radius(top)} ${top}H${x+radius(top)}M${x-radius(end)} ${end}H${x+radius(end)}" stroke="#49ebda" stroke-width="2"/><g fill="#cde0ed" font-size="9" text-anchor="middle"><text x="${x}" y="12">Ø ${model.diameter} mm</text><text x="${x}" y="${end+13}">Ø ${model.distalDiameter} mm</text><text x="97" y="${(top+end)/2}" transform="rotate(90 97 ${(top+end)/2})">${model.length} mm</text></g></svg>`;
}
export function limbCards(models) {
    return models.map(m=>`<button type="button" id="graftLimb-${m.id}" class="stent-graft-model" aria-pressed="false" aria-label="Endurant II / IIs, nóżka 16 na ${m.distalDiameter} mm, długość ${m.length} mm, ${m.deliveryFr} Fr">${limbDrawing(m)}<strong>16 → ${m.distalDiameter} mm</strong><span>${m.length} mm · ${m.deliveryFr} Fr</span><small>${m.id}</small></button>`).join('');
}
