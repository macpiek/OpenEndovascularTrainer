// Endurant II/IIs IFU, FDA P100021/S063, Figure 1 (printed p. 4),
// sections 11.2.1 and 11.2.14. These are implant landmarks, not tube seams.
// Button size/angular placement is a rendering approximation; the IFU drawing
// is not to scale. Lengths below are material coordinates, so markers stay sewn.
export function graftMarkerLayout(device,index,length) {
    const scale=device.dimensionScale??1,span=.5*scale,inset=.75*scale;
    const button=(s,angle,role)=>({start:Math.max(0,s),end:Math.min(length,s+span),angle,role});
    const proximal=angle=>button(inset,angle,'proximal');
    const distal=angle=>button(length-inset-span,angle,'distal');
    if(device.type==='limb')return [proximal(0),proximal(Math.PI),distal(0),distal(Math.PI),
        button(inset+25*scale,Math.PI/2,'overlap')];
    if(index===0)return [proximal(0),proximal(Math.PI),proximal(3*Math.PI/2),
        button(length-span,Math.PI/2,'flow-divider')];
    if(index===1)return device.modelId==='iis-103'?[distal(0),distal(Math.PI)]:[distal(0)];
    // The short gate has its own asymmetric radiopaque gate marker.
    return [];
}
