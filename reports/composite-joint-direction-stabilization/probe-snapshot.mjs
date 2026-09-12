import {fixture,addPreparedWall} from './tests/fixture.mjs';
import {advanceCompositeJointTimeStep} from './src/physics/kirchhoffCompositeJointTimeStep.js';
const f=fixture();addPreparedWall(f,{height:.49});const r=advanceCompositeJointTimeStep(f.initial,f.options(f.initial));
console.log(JSON.stringify({primalShift:process.env.OET_PRIMAL_SHIFT??0,debugLinear:globalThis.__OET_LINEAR,scope:'original initial penetration fixture; unchanged physical tolerances, only iterative refinement cap varied',corrections:process.env.OET_LINEAR_CORRECTIONS??1,accepted:r.accepted,status:r.status,directions:r.diagnostics.directions,evaluations:r.diagnostics.evaluations,linear:r.diagnostics.lastLinear,certificate:r.diagnostics.certificate},null,2));
