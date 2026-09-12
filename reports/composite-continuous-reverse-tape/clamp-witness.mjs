import fs from 'node:fs';
import {createCompositeContinuousGeometry} from '../../src/physics/kirchhoffCompositeContinuousGeometry.js';
import {createCompositeContinuousLength} from '../../src/physics/kirchhoffCompositeContinuousLength.js';
const geometry=createCompositeContinuousGeometry({coordinates:[0,1,2,3,4,5]}).edges[0],operator=createCompositeContinuousLength({geometry}),
    positions=geometry.nodeIndices.map(x=>[x,0,0]),first=structuredClone(operator.evaluate(positions)),N=first.gradient.length,
    free=geometry.nodeIndices.flatMap((node,j)=>node>1?[3*j,3*j+1,3*j+2]:[]),freeGradient=free.map(j=>first.gradient[j]),freeHessian=free.map(i=>free.map(j=>first.hessian[i*N+j]));
positions[geometry.nodeIndices.indexOf(2)][1]=1e-4;
const perturbed=operator.evaluate(positions),result={scope:'straight first continuous element, endpoints 0 and 1 prescribed one rest-length apart',nodes:geometry.nodeIndices,
    baseLength:first.length,freeGradient,freeHessian,neighborTransversePerturbation:1e-4,perturbedLength:perturbed.length,excessLength:perturbed.length-1};
fs.writeFileSync('/tmp/oet-continuous-clamp-nonregular-witness.json',JSON.stringify(result,null,2)+'\n');console.log(JSON.stringify(result));
