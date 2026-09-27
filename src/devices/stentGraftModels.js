// Nominal covered lengths and diameters: Medtronic Endurant II/IIs sizing sheet.
// Gate lengths/diameters and crotch width: Medtronic Aortic Product Catalogue, pp. 15–18.
// Crown and ring details are a procedural approximation of the diagram,
// not manufacturer CAD. Length excludes the bare suprarenal crown.
export const MAIN_BODY_MODELS = Object.freeze([
    {id:'iis-103',name:'Endurant IIs',length:103,trunkLength:50,gateLength:84},
    {id:'ii-124',name:'Endurant II',length:124,trunkLength:40,gateLength:74},
    {id:'ii-145',name:'Endurant II',length:145,trunkLength:50,gateLength:84},
    {id:'ii-166',name:'Endurant II',length:166,trunkLength:50,gateLength:84},
]);
export const DEFAULT_MAIN_BODY='iis-103';
export function mainBodyModel(id=DEFAULT_MAIN_BODY) {return MAIN_BODY_MODELS.find(m=>m.id===id)??MAIN_BODY_MODELS[0];}
export function proximalDiameters(id) {return id==='ii-124'?[23,25,28,32]:[23,25,28,32,36];}
export function distalDiameters(id,diameter) {
    return id==='iis-103'?[14]:diameter<28?[13,16]:diameter===28?[13,16,20]:[16,20];
}
export function bodyDimensions(device) {
    const model=mainBodyModel(device.modelId);
    const gateDiameter=model.id!=='iis-103'&&device.diameter===23?12:14;
    const crotchDiameter=model.id==='iis-103'?28:device.diameter===23?25:device.distalDiameter===13?27:30;
    const ipsiRootDiameter=crotchDiameter-gateDiameter;
    const distalStraight=ipsiRootDiameter!==device.distalDiameter?({124:30,145:40,166:60}[model.length]??0):0;
    return {...model,gateDiameter,crotchDiameter,ipsiRootDiameter,distalStraight,ipsiLength:model.length-model.trunkLength,contraLength:model.gateLength-model.trunkLength};
}

// Delivery OD from the 2025 Endurant II/IIs sizing sheet (Fr / 6 = radius in mm):
// https://www.medtronic.com/content/dam/medtronic-wide/public/western-europe/products/cardiac-vascular/cardiovascular/aortic-stent-grafts/endurant-ii-sizing-sheet-print-en-gb.pdf
export function deliveryRadiusMm(device) {
    const catalog=device.type==='limb'&&LIMB_MODELS.find(m=>m.id===device.modelId);
    if(catalog)return catalog.deliveryFr/6*graftScale(device);
    const french=device.type==='body'?(device.diameter>=32?20:18):
        (device.diameter>16||device.length>124?16:14);
    return french/6*graftScale(device);
}

// Shared by the crimped preview and released scaffold: wire length belongs to
// the catalogue geometry, not the anatomy-deformed surface or delivery pose.
function unscaledPartRadius(device,index,s) {
    if(device.type!=='body') {
        return nominalLimbRadius(device,s);
    }
    const d=bodyDimensions(device),blend=(a,b,t)=>{
        t=Math.max(0,Math.min(1,t));return (a+(b-a)*t*t*t*(10-15*t+6*t*t))/2;
    };
    if(index===0) {
        const a=blend(device.diameter,d.crotchDiameter,s/d.trunkLength);
        const b=blend(device.diameter,device.modelId==='iis-103'?14:d.crotchDiameter,s/d.trunkLength);
        // Equivalent perimeter radius of the flattened bifurcation section.
        const h=((a-b)/(a+b))**2;
        return (a+b)/2*(1+3*h/(10+Math.sqrt(4-3*h)));
    }
    if(index===1)return blend(d.ipsiRootDiameter,device.distalDiameter,(s-(d.ipsiLength-d.distalStraight-10))/10);
    return index===2?d.gateDiameter/2:device.diameter/2;
}

// Endurant II and IIs share the ETLW iliac limb catalogue (European EE system).
// Source: Medtronic UK ordering information, “Limbs”, checked 2026-09-23.
export const LIMB_CATALOGUE_SOURCE='https://www.medtronic.com/en-gb/healthcare-professionals/products/cardiovascular/aortic/aortic-stent-grafts/endurant-ii-stent-graft-system.html';
export const LIMB_LENGTHS=Object.freeze([82,93,124,156,199]);
export const LIMB_DISTAL_DIAMETERS=Object.freeze([10,13,16,20,24,28]);
export const LIMB_MODELS=Object.freeze(LIMB_DISTAL_DIAMETERS.flatMap(distalDiameter=>LIMB_LENGTHS.map(length=>Object.freeze({
    id:`ETLW16${distalDiameter}C${length}EE`,name:'Endurant II / IIs',diameter:16,distalDiameter,length,
    deliveryFr:distalDiameter>16||length>124?16:14,
}))));
export const DEFAULT_LIMB='ETLW1616C82EE';
export function limbModel(id=DEFAULT_LIMB){return LIMB_MODELS.find(m=>m.id===id)??LIMB_MODELS.find(m=>m.id===DEFAULT_LIMB);}

// Medtronic Aortic Product Catalogue, printed pp. 19–22:
// A = transition length; B = straight distal segment after the transition.
// https://www.medtronic.com/content/dam/medtronic-wide/public/western-europe/products/cardiac-vascular/cardiovascular/aortic-stent-grafts/aortic-product-catalogue.pdf
// Smooth shoulders within A are a procedural approximation, not manufacturer CAD.
export function limbProfileDimensions(device) {
    const distal=device.distalDiameter??device.diameter;
    if(distal===device.diameter)return {transitionStart:device.length,transitionLength:0,distalStraight:0};
    const transitionLength=[10,24,28].includes(distal)?20:10;
    const distalStraight=distal<16
        ?({82:30,93:40,124:40,146:62,156:72,199:115}[device.length]??40)
        :(device.length===82?30:40);
    return {transitionStart:Math.max(0,device.length-distalStraight-transitionLength),transitionLength,distalStraight};
}
export function nominalLimbRadius(device,s) {
    const distal=device.distalDiameter??device.diameter;
    const {transitionStart,transitionLength}=limbProfileDimensions(device);
    if(!transitionLength)return device.diameter/2;
    const t=Math.max(0,Math.min(1,(s-transitionStart)/transitionLength));
    const smooth=t*t*t*(10-15*t+6*t*t);
    return (device.diameter+(distal-device.diameter)*smooth)/2;
}

// Catalog values stay in millimetres; only world-space geometry is converted.
export const graftScale=device=>device.dimensionScale??1;
export function worldBodyDimensions(device) {
    const dimensions=bodyDimensions(device),scale=graftScale(device);
    return Object.fromEntries(Object.entries(dimensions).map(([key,value])=>[key,typeof value==='number'?value*scale:value]));
}
export function nominalPartRadius(device,index,s) {
    const scale=graftScale(device);
    return unscaledPartRadius(device,index,s/scale)*scale;
}
