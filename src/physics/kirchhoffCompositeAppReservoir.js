import {sampleCompositeNativeRateHistory} from './kirchhoffCompositeNativeRateHistory.js';

/** Explicit proximal supply for the native remap. Extend the accepted first
 * material segment backwards, retaining its frame, unwrapped angle and inlet
 * velocity. This is a boundary model for newly admitted material, not a read
 * of the legacy display buffers or a reconstruction of unobserved motion. */
export function createCompositeAppInletReservoir({state,tools}) {
    const records=new Map();
    for(const source of tools) {
        const id=source.toolId,tool=state.tools.find(t=>t.id===id),
            edge=state.layout.edgeToolIds.findIndex(ids=>ids.includes(id)),
            spec=tool.appMaterialProfile,sEnd=spec.materialOrigin+(state.coordinates[edge]-spec.coordinateOrigin),
            sStart=Math.min(...source.materialLabels);
        if(sStart>=sEnd)continue;
        const points=state.toolPositions.get(id),a=points[edge],b=points[edge+1],
            span=state.coordinates[edge+1]-state.coordinates[edge],
            velocity=state.materialVelocities.flatMap(e=>e.tools).find(t=>t.id===id&&t.sStart<=sEnd&&sEnd<t.sEnd);
        if(!(span>0)||!velocity||velocity.interpretation!=='physical-material-velocity')
            throw new RangeError('The inlet needs the accepted native material segment and velocity');
        const fraction=(sEnd-velocity.sStart)/(velocity.sEnd-velocity.sStart),
            inletVelocity=velocity.velocities[0].map((v,k)=>v+fraction*(velocity.velocities[1][k]-v)),
            proximal=a.map((v,k)=>v+(sStart-sEnd)*(b[k]-v)/span);
        const angularVelocity=state.nativeRateHistory
            ? sampleCompositeNativeRateHistory({history:state.nativeRateHistory,toolId:id,label:sEnd,trace:'right'}).angularVelocity
            : state.step===0&&velocity.angularVelocityInterpretation==='physical-material-angular-velocity'
                ? velocity.angularVelocity?.slice() : null;
        if(angularVelocity?.length!==3||!angularVelocity.every(Number.isFinite))
            throw new RangeError('New inlet material needs explicit accepted angular-rate history');
        records.set(id,{id,sStart,sEnd,positions:[proximal,a.slice()],
            velocities:[inletVelocity.slice(),inletVelocity.slice()],reference:structuredClone(tool.reference[edge]),
            angle:state.angles.get(id)[edge],angularVelocity,interpretation:'physical-material-velocity'});
    }
    return ({toolId,s})=>{
        const record=records.get(toolId);
        return record&&s>=record.sStart&&s<=record.sEnd?record:null;
    };
}
