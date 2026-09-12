import test from 'node:test';
import assert from 'node:assert/strict';
import fs from 'node:fs';
import {gunzipSync} from 'node:zlib';
import {solveActiveCondensedCoupledQP as solve} from '../src/physics/kirchhoffActiveCondensedSolver.js';
import {measureCoupledLoadKKT} from '../src/physics/kirchhoffCoupledLoadSolver.js';

for (const name of ['friction-boundary-cycle','normal-load-cycle','active-condensed-1999','coupled-full-200']) {
    test(`batched equality responses preserve full original equations: ${name}`, () => {
        const p = JSON.parse(gunzipSync(fs.readFileSync(new URL(`./fixtures/kirchhoff-${name}.json.gz`,import.meta.url))),
            (_,v)=>v==='Infinity'?Infinity:v==='-Infinity'?-Infinity:v);
        const args = ['matrix','rhs','lower','upper'].map(k=>Float64Array.from(p[k]));
        const before = args.map(a=>a.slice());
        const options = {tolerance:.0002,numericalShift:1e-8,...p.options,
            initialFree:p.initialFree??p.options?.initialFree,simultaneousCoulomb:true};
        const reference = solve(...args,p.count,p.band,p.groups,{...options,batchEqualityResponses:false});
        const batched = solve(...args,p.count,p.band,p.groups,{...options,batchEqualityResponses:true});
        assert.ok(reference.diagnostics.converged);
        assert.ok(batched.diagnostics.converged,JSON.stringify(batched.diagnostics));
        assert.ok(batched.diagnostics.responseBatches>0);
        assert.ok(batched.diagnostics.responseKernelCalls<reference.diagnostics.responseKernelCalls);
        assert.deepEqual(batched.increment,reference.increment);
        assert.deepEqual(batched.residual,reference.residual);
        assert.deepEqual(batched.free,reference.free);
        assert.deepEqual(batched.lower,reference.lower);
        assert.deepEqual(batched.upper,reference.upper);
        args.forEach((a,i)=>assert.deepEqual(a,before[i]));
        const residual=Float64Array.from(p.rhs), x=batched.increment;
        for(let i=0;i<p.count;i++)for(let j=Math.max(0,i-p.band+1);j<=i;j++){
            const a=p.matrix[i*p.band+i-j];residual[i]-=a*x[j];if(i!==j)residual[j]-=a*x[i];
        }
        const groups=p.groups.map(g=>({...g,radii:g.normalRow==null?g.radii:g.mu.map(mu=>mu*Math.max(0,g.normalLambda+x[g.normalRow]))}));
        const kkt=measureCoupledLoadKKT(residual,x,p.lower,p.upper,groups);
        assert.ok(kkt.maximumResidual<=options.tolerance,JSON.stringify(kkt));
        assert.ok(kkt.coneViolation<=1e-9);
    });
}
