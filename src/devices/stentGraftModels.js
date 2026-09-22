// Nominal covered lengths and diameters: Medtronic Endurant II/IIs sizing sheet.
// Crotch, crown and ring details are a procedural approximation of the diagram,
// not manufacturer CAD. Length excludes the bare suprarenal crown.
export const MAIN_BODY_MODELS = Object.freeze([
    {id:'iis-103',name:'Endurant IIs',length:103,trunkLength:50,gateLength:80},
    {id:'ii-124',name:'Endurant II',length:124,trunkLength:40,gateLength:70},
    {id:'ii-145',name:'Endurant II',length:145,trunkLength:50,gateLength:80},
    {id:'ii-166',name:'Endurant II',length:166,trunkLength:50,gateLength:80},
]);
export const DEFAULT_MAIN_BODY='iis-103';
export function mainBodyModel(id=DEFAULT_MAIN_BODY) {return MAIN_BODY_MODELS.find(m=>m.id===id)??MAIN_BODY_MODELS[0];}
export function proximalDiameters(id) {return id==='ii-124'?[23,25,28,32]:[23,25,28,32,36];}
export function distalDiameters(id,diameter) {
    return id==='iis-103'?[14]:diameter<28?[13,16]:diameter===28?[13,16,20]:[16,20];
}
export function bodyDimensions(device) {
    const model=mainBodyModel(device.modelId);
    return {...model,ipsiLength:model.length-model.trunkLength,contraLength:model.gateLength-model.trunkLength};
}
