import {readFileSync} from 'node:fs';
import {EndovascularPhysicsWorld,DEFAULT_TOOL_PROFILES} from '../../src/physics/endovascularPhysicsWorld.js';
import {PIGTAIL_CATHETER_INNER_RADIUS_MM} from '../../src/toolDimensions.js';
import {RodState} from '../../src/physics/rodState.js';
import {GuidewireTransport} from '../../src/physics/guidewireTransport.js';
import {PigtailCatheter} from '../../src/pigtailCatheter.js';
import {createCompositeJointAppSystem} from '../../src/physics/kirchhoffCompositeAppSystem.js';
import {createCompositeAppToolSource,alignCompositeAppInitialMaterialFrames} from '../../src/physics/kirchhoffCompositeAppInputs.js';
import {createCompositeAppInletReservoir} from '../../src/physics/kirchhoffCompositeAppReservoir.js';
export function createCapturedCompositeAppFixture({wireRate=16,worldWall={rateMode:'backward-euler-grid'},coordinateOrigin}={}) {
const capture=JSON.parse(readFileSync(new URL('../fixtures/compositeNativeAppInitial.json',import.meta.url),'utf8')),captures=capture.tools,context=capture.context,dt=1/120;
const world=new EndovascularPhysicsWorld({fixedDt:dt}),s=context.sheaths[0],point=values=>Object.fromEntries(['x','y','z'].map((k,i)=>[k,values[i]])),
 sheath={start:point(s.start),end:point(s.start.map((v,i)=>v+s.length*s.axis[i]))},bodies=new Map();
for(const capture of captures){const raw=capture.body,body=world.createRod(capture.toolId==='wire'?'guidewire':'catheter',raw.count,raw.segmentLength);for(const [key,value]of Object.entries(raw))body[key]=value;for(const [key,record] of Object.entries(capture.arrays))if('fill'in record)body[key].fill(record.fill===null?NaN:record.fill);else body[key].set(record.values.map(v=>v===null?NaN:v),record.offset);bodies.set(capture.toolId,body);}
const rawWire=captures.find(t=>t.toolId==='wire').body,rod=new RodState(rawWire.count,rawWire.segmentLength),length=(rawWire.count-1)*rawWire.segmentLength,
 transport=new GuidewireTransport({rod,segmentLength:rawWire.segmentLength,guidewireLength:length,sheath,advanceRate:wireRate});transport.reset();
const catheter=new PigtailCatheter({wire:rod,segmentLength:5,guidewireLength:length,tailProgressRef:()=>transport.progress,vessel:{sheath},retainMaterialTip:true});
catheter.setType('berenstein');catheter.setStiffnessScales({shaftStiffnessScale:25,tipStiffnessScale:5});
const sourceSheath=world.addSheath({...sheath,innerRadius:s.innerRadius,proximalExtension:s.proximalExtension,bodies:world.bodies});
Object.assign(sourceSheath,{axisX:s.axis[0],axisY:s.axis[1],axisZ:s.axis[2],length:s.length});
const containment=world.addContainment(bodies.get('wire'),bodies.get('catheter'),{innerRadius:PIGTAIL_CATHETER_INNER_RADIUS_MM,friction:DEFAULT_TOOL_PROFILES.catheter.lumenFriction,axialFriction:DEFAULT_TOOL_PROFILES.catheter.lumenAxialFriction,torsionalFriction:DEFAULT_TOOL_PROFILES.catheter.lumenTorsionalFriction,lumenMaxCorrection:.4,openProximal:true,openDistal:true,searchWindow:2,outerStartNode:15,innerResponse:1,outerResponse:1,enforceDistalPortal:true,containedLength:0,enabled:false});
const external=world.addToolContact(bodies.get('wire'),bodies.get('catheter'),{friction:.08,openDistalB:true,enabled:false});
const counters={layout:0,controls:0,initial:0};
function layout(){return captures.map(capture=>{const toolId=capture.toolId,body=bodies.get(toolId),wire=toolId==='wire',progress=wire?transport.progress:catheter.progress,x=Array.from({length:body.activeEnd-body.activeStart+1},(_,i)=>wire?transport.insertedCoordinate(body.activeStart+i):body.materialCoordinate[body.activeStart+i]);return {body,toolId,nodeCoordinates:x,materialLabels:x.map(x=>x-progress),profile:capture.appMaterialProfile};});}
const system=createCompositeJointAppSystem({geometry:'native-discrete-rod',worldWall,coordinateOrigin,readToolSources(){counters.initial++;return layout().map(source=>{alignCompositeAppInitialMaterialFrames({body:source.body,epoch:system});return createCompositeAppToolSource({...source,unwrappedAngles:Array(source.nodeCoordinates.length-1).fill(0),referenceWindingTurns:Array(source.nodeCoordinates.length-2).fill(0),massPerMaterialLength:captures.find(c=>c.toolId===source.toolId).material.massPerMaterialLength});});},
readNativeLayout({state}){counters.layout++;const tools=layout();return {tools,reservoir:createCompositeAppInletReservoir({state,tools})};},
readControls({state,bindings}){counters.controls++;return {commands:state.tools.map(t=>({toolId:t.id,labelShift:0,feedVelocity:0,spinIncrement:0})),positionBoundaries:bindings.flatMap(b=>b.nodes.filter(r=>b.body.pinned[r.node]).map(r=>({toolId:b.toolId,node:r.jointNode,value:b.toolId==='wire'?['x','y','z'].map(k=>rod.nodes[r.node][k]):['x','y','z'].map(k=>b.body[k][r.node])})))};}});
function prepare(wireCommand,catCommand){
    transport.advance(wireCommand,dt);catheter.advance(catCommand,dt,transport.progress);
    const wire=bodies.get('wire');wire.syncFromRodState(rod);
    wire.setActiveRange(transport.firstProximalSupportNodeIndex(),wire.count-1);
    const count=catheter.syncXpbdBody(bodies.get('catheter')),
        first=Math.max(0,Math.ceil((length-transport.progress)/5)),
        last=Math.min(wire.count-1,Math.floor((length-transport.progress+catheter.progress)/5));
    world.updateContainmentWindow(containment,{enabled:catheter.progress>.5&&count>=2&&last>=first,
        outerStartNode:catheter.physicsLumenStartNode,startNode:first,endNode:Math.max(first,last),
        innerArcOffset:first*5-length+transport.progress,containedLength:Math.min(catheter.progress,transport.progress),enforceDistalPortal:true});
    let wallStart=Math.max(0,transport.firstLumenNodeIndex()-1),wallEnd=wire.segmentCount-1;
    wire.setSheathMaterialEndNode(wallStart);
    if(containment.enabled) {
        wallStart=Math.max(wallStart,last);
        if(wallStart>wire.activeEnd-1){wallStart=wire.activeEnd;wallEnd=wire.activeEnd-1;}
    }
    wire.setCollisionRange(wallStart,wallEnd);
    const catheterEnd=Math.max(0,count-2),firstExternal=Math.max(0,Math.min(wire.segmentCount-1,last+1));
    Object.assign(external,{enabled:catheter.progress>4&&count>=2&&transport.progress>catheter.progress+.5&&firstExternal<=wire.activeEnd-1,
        startSegmentA:firstExternal,endSegmentA:Math.min(wire.activeEnd-1,firstExternal+16),
        startSegmentB:Math.max(0,catheterEnd-8),endSegmentB:catheterEnd});
}

return {system,world,bodies,rod,transport,catheter,containment,external,counters,dt,prepare};
}
