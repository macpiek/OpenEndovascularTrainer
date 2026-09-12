// Mechanical replay of captured application sources. Timings are Node timings,
// not browser FPS or a real-time physics acceptance measurement.
import {writeFileSync} from 'node:fs';
import {createCompositeAnatomyField} from '../tests/helpers/compositeAnatomyField.js';
import {createCapturedCompositeAppFixture} from '../tests/helpers/capturedCompositeAppFixture.js';
const args=process.argv.slice(2),argument=(name,fallback)=>{const i=args.indexOf(name);return i<0?fallback:args[i+1];};
const steps=Number(argument('--steps',400)),output=argument('--output',null),contactMode=argument('--contact-mode','capsule');
if(!['capsule','nodal-endpoints','material-points'].includes(contactMode))throw new RangeError('--contact-mode must be capsule, nodal-endpoints or material-points');
const contactQueries=Number(argument('--contact-queries',10000));
const seamUpdates=argument('--seam-updates','fixed');
if(!['fixed','automatic'].includes(seamUpdates))throw new RangeError('--seam-updates requires fixed or automatic');
if(!Number.isInteger(contactQueries)||contactQueries<1)throw new RangeError('--contact-queries requires a positive integer');
if(!Number.isInteger(steps)||steps<1)throw new RangeError('--steps requires a positive integer');
const anatomy=createCompositeAnatomyField(),f=createCapturedCompositeAppFixture({wireRate:44,coordinateOrigin:'sheath-start',
    worldWall:{source:'original-field',contactMode,rateMode:'backward-euler-grid',seamUpdates}});
f.world.contactField=anatomy.field;
// Diagnostic override only; no dt, tolerance or application default changes.
f.system.setBudget({contactQueries});
const started=performance.now(),contactTransitions=[];let last=null,attempted=0;
try {
    for(;attempted<steps;attempted++) {
        f.prepare(1,0);last=f.system.step(f.world,f.dt);
        if(last.diagnostics?.seamRestarts?.length||last.diagnostics?.exactSeamRestorations?.length)contactTransitions.push({step:attempted,accepted:last.accepted,
            status:last.status,directions:last.diagnostics.directions,evaluations:last.diagnostics.evaluations,contactQueries:last.diagnostics.contactQueries,
            seamRestarts:last.diagnostics.seamRestarts??[],exactSeamRestorations:last.diagnostics.exactSeamRestorations??[],certificate:last.diagnostics.certificate});
        if(attempted%30===0||!last.accepted)console.log(JSON.stringify({step:attempted,guidewireMm:f.transport.progress,
            accepted:last.accepted,status:last.status,force:last.diagnostics?.certificate?.force,elapsedMs:performance.now()-started}));
        if(!last.accepted){attempted++;break;}
        f.bodies.get('wire').syncToRodState(f.rod);
    }
    const report={scope:'captured-app-source-mechanical-replay-with-active-anatomy',contactMode,seamUpdates,contactQueries,attempted,requestedSteps:steps,
        guidewireMm:f.transport.progress,catheterMm:f.catheter.progress,contactTransitions,result:last,state:f.system.snapshot()};
    if(output)writeFileSync(output,JSON.stringify(report,(key,value)=>value instanceof Map?[...value]:ArrayBuffer.isView(value)?Array.from(value):value));
    console.log(JSON.stringify({acceptedSteps:report.state.step,attempted,status:last.status,guidewireMm:f.transport.progress}));
    if(!last.accepted)process.exitCode=1;
} finally {f.catheter.dispose();anatomy.dispose();}
