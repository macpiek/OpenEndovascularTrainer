import {readFileSync} from 'node:fs';
import {createCompositeChainLayout} from '../../src/physics/kirchhoffCompositeChain.js';
import {createCompositeJointTimeStepState} from '../../src/physics/kirchhoffCompositeJointTimeStep.js';
import {prepareCompositeAppMaterialProfile,prepareCompositeAppInputs,completeCompositeAppNativePositionBoundaries} from '../../src/physics/kirchhoffCompositeAppInputs.js';
export function createCompositeCoordinatePrecisionCase({translated=false}={}) {
const source=JSON.parse(readFileSync(new URL('../../reports/composite-joint-ui-cases/fractional-catheter-input.json',import.meta.url),'utf8')),
 context=JSON.parse(readFileSync(new URL('../fixtures/compositeNativeAppInitial.json',import.meta.url),'utf8')).context,s=context.sheaths[0],raw=source.state,
 origin=translated?s.start:[0,0,0],
 shift=p=>p.map((v,k)=>v-origin[k]);
const nativeRateHistory=structuredClone(raw.nativeRateHistory);for(const tool of nativeRateHistory.tools)for(const e of tool.edges){e.positions=e.positions.map(shift);if(e.previousPositions)e.previousPositions=e.previousPositions.map(shift);}
const state=createCompositeJointTimeStepState({...raw,positions:raw.positions.map(shift),nativeRateHistory,layout:createCompositeChainLayout(raw.layout.edgeToolIds),angles:new Map(raw.angles),restLengths:new Map(raw.restLengths),tools:raw.tools.map(t=>({...t,...prepareCompositeAppMaterialProfile(t.appMaterialProfile,t.appMaterialProfile.coordinateOrigin,t.appMaterialProfile.materialOrigin)}))});
state.materialVelocities=raw.materialVelocities;state.boundaryMultipliers=new Map(raw.boundaryMultipliers);
const positionBoundaries=completeCompositeAppNativePositionBoundaries({state,bindings:source.bindings,positionBoundaries:source.bindings.flatMap(b=>b.nodes.filter(n=>b.pinned[n.node]).map(n=>({toolId:b.toolId,node:n.jointNode,value:shift(b.toolId==='wire'?s.start.map((v,k)=>v+s.axis[k]*state.coordinates[n.jointNode]):context.positionBoundaries.find(b=>b.toolId==='catheter').value)})))});
const sheath={mode:'world-sheath-normal',forcePerLength:1,sites:source.bindings.flatMap(b=>b.nodes.map(n=>({id:JSON.stringify(['world-sheath',0,b.toolId,b.toolId==='wire'?'guidewire':'catheter',n.node,n.jointNode]),toolId:b.toolId,node:n.jointNode,start:shift(s.start),axis:s.axis,length:s.length,proximalExtension:s.proximalExtension,clearance:s.innerRadius-(b.toolId==='wire'?Math.fround(.4445):Math.fround(2.5/3)),sourceRadius:b.toolId==='wire'?Math.fround(.4445):Math.fround(2.5/3),sourceInnerRadius:s.innerRadius})))};
const prepared=prepareCompositeAppInputs({state,dt:1/120,commands:state.tools.map(t=>({toolId:t.id,labelShift:0,feedVelocity:0,spinIncrement:0})),positionBoundaries});
return {state:prepared.state,options:{...prepared.options,sheath},origin:origin.slice(),sourcePositions:raw.positions.map(p=>p.slice())};
}
