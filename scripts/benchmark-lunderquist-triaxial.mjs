// Node solver timing excludes rendering; never equate it to browser FPS.
import fs from 'node:fs';
import {loadCoupledRuntimeAnatomy} from '../tests/helpers/coupledRuntimeFixture.js';
import {readLunderquistTriaxialReplay,runLunderquistTriaxialReplay,compareTriaxialPoses} from '../tests/helpers/lunderquistTriaxialReplay.js';
import {SHARED_AXIS_APP_PHYSICS_OPTIONS} from '../src/physics/kirchhoffSharedAxisAppSystem.js';
const fixture=readLunderquistTriaxialReplay(),anatomy=await loadCoupledRuntimeAnatomy(undefined,'baseline');
const patches=JSON.parse(process.argv[3]??'[{"newtonActiveSetLimit":16},{},{"newtonActiveSetLimit":16},{}]');
const report={scope:'captured triaxial step plus short continuation with frozen blood flow; Node wall timing, not browser FPS',
    runs:[],tolerances:{force:1e-6,length:1e-5},physicalDt:fixture.stepRequest.dt,
    productionNewtonActiveSetLimit:SHARED_AXIS_APP_PHYSICS_OPTIONS.newtonActiveSetLimit};
let reference;
try {
    for(const patch of patches) {
        const {poses,...run}=runLunderquistTriaxialReplay(fixture,anatomy.field,{patch,count:Number(process.argv[4]??30),
            onStep:step=>console.log(JSON.stringify({patch,...step,quality:undefined}))});
        reference??=poses;
        report.runs.push({patch,...run,comparisonToFirstRun:compareTriaxialPoses(reference,poses)});
        if(!run.summary.allAccepted)process.exitCode=1;
        fs.writeFileSync(process.argv[2]??'/tmp/lunderquist-continuation.json',JSON.stringify(report,null,2));
    }
} finally {anatomy.dispose();}
