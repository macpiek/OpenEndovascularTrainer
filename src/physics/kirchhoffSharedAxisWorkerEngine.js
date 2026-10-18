import {createSharedAxisAppSystem} from './kirchhoffSharedAxisAppSystem.js';
import {restoreSharedAxisFieldMessage,updateSharedAxisWorkerTools,captureSharedAxisWorkerBodies} from './kirchhoffSharedAxisWorkerData.js';

// Transport-independent host: Node worker tests and the browser worker execute
// the same provider as the ordinary simulator, including its recovery policy.
export function createSharedAxisWorkerEngine() {
    let system,world,tools=[],sheath,flow=null,failures=[],bodies=new Map();
    const read=()=>({diagnostics:system.diagnostics,flowState:system.captureFlowSamplingState(),
        bodies:captureSharedAxisWorkerBodies(tools),contactCount:world.contactCount,
        maxPenetration:world.maxPenetration,settledMaxPenetration:world.settledMaxPenetration,failures:failures.splice(0)});
    return {
        dispatch(operation,data) {
            if(operation==='initialize') {
                system?.reset();world?.contactField.fallbackGeometry.dispose();bodies=new Map();
                world={contactField:restoreSharedAxisFieldMessage(data.field)};
                system=createSharedAxisAppSystem({...data.options,workSliceMs:Infinity,
                    readTools:()=>tools,readSheath:()=>sheath,readFrozenBloodFlow:()=>flow,
                    onRejectedStep:event=>failures.push(event)});
                return {ready:true};
            }
            if(!system)throw new Error('Physics worker is not initialized');
            if(operation==='reset'){system.reset();flow=null;failures=[];return {reset:true};}
            if(operation==='retry')return {retried:system.retryFromLastAccepted()};
            if(operation==='setting')return {changed:system[data.method](data.value)};
            if(operation==='restore'||operation==='step') {
                tools=updateSharedAxisWorkerTools(data.tools,bodies);sheath=data.sheath;flow=data.flow??null;
                if(operation==='restore') {system.restoreReplay(data.fixture,world);return read();}
                let result=system.step(world,data.dt);
                while(result.pending)result=system.step(world,data.dt);
                return {...read(),result};
            }
            throw new Error('Unknown physics worker operation');
        },
        dispose(){system?.reset();world?.contactField.fallbackGeometry.dispose();system=null;world=null;}
    };
}
