import assert from 'node:assert/strict';
import test from 'node:test';
import {makeFixture,preparedOptions,advance,conventions} from '../scripts/physics/helpers/compositeTimeStepBenchmark.js';

function options(f,state=f.state){return {...preparedOptions(f,state,'strict-tests'),constraintSolver:'mixed'};}
const accept=r=>assert.ok(r.accepted,JSON.stringify({status:r.status,...r.diagnostics}));
const close=(a,b,t=1e-7)=>assert.ok(Math.abs(a-b)<=t*Math.max(1,Math.abs(a),Math.abs(b)),`${a} != ${b}`);
for(const scenario of ['contact-free','analytic-plane'])for(const insertion of [9,160,310]){
    test(`real profiles on one chain execute two strict mixed physical steps (${scenario}, catheter ${insertion}mm)`,()=>{
        const f=makeFixture(insertion,scenario);let state=f.state;
        for(let step=0;step<2;step++){
            const before=state.data.positions.map(p=>[...p]),beforeLambda=state.lengthMultipliers.slice();
            const opts=options(f,state),result=advance(state,opts);accept(result);
            assert.deepEqual(state.data.positions,before);assert.deepEqual(state.lengthMultipliers,beforeLambda);
            assert.notEqual(result.state,state);assert.equal(result.state.step,step+1);close(result.state.time,(step+1)*conventions.dt);
            const d=result.diagnostics;assert.equal(d.historyCommits,1);assert.equal(d.constraintSolver,'mixed');
            assert.ok(d.directions<=opts.budget.directions);assert.ok(d.linearSolves<=d.limits.linearSolves);
            assert.ok(d.certificate.force<=opts.tolerances.force);assert.ok(d.certificate.torque<=opts.tolerances.torque);
            // Independent original geometry, rather than the stored certificate.
            const p=result.state.data.positions,x=result.state.data.coordinates;
            for(let e=0;e<p.length-1;e++)close(Math.hypot(...p[e+1].map((v,a)=>v-p[e][a])),x[e+1]-x[e],2e-12);
            if(scenario==='analytic-plane'){
                let force=0,rate=0;
                result.state.wallContactState.records.forEach(record=>{force+=record.worldForce[1];assert.ok(record.history.normalForce>=0);});
                result.state.materialVelocities.forEach((edge,e)=>edge.tools.forEach(tool=>{
                    const old=opts.inertiaEdges[e].tools.find(t=>t.id===tool.id);
                    rate+=old.massPerMaterialLength*old.materialMap.dsDx*(x[e+1]-x[e])/conventions.dt*
                        .5*(tool.velocities[0][1]+tool.velocities[1][1]-old.oldMaterialVelocities[0][1]-old.oldMaterialVelocities[1][1]);
                }));
                close(force+opts.loads.reduce((sum,v,i)=>sum+(Array.from(state.layout.positions).some(s=>s+1===i)?v:0),0),rate,3e-7);
                for(const owner of f.mesh.contactOwners.edges)if(owner.wall){
                    assert.ok(Math.min(p[owner.edge][1],p[owner.edge+1][1])-owner.wall.radius>=-opts.wall.tolerances.gap);
                }
                assert.ok(d.certificate.wall.converged);
            }
            state=result.state;
        }
    });
}

test('constant constitutive energy cannot trap the mixed solve at a scalar objective rounding floor',()=>{
    const f=makeFixture(160,'contact-free'),plain=advance(f.state,options(f));accept(plain);
    for(const tool of f.state.data.tools){const source=tool.materialAt;tool.materialAt=args=>{const sample=source(args);return {...sample,energyOffset:sample.energyOffset+1e20};};}
    const shifted=advance(f.state,options(f));accept(shifted);
    assert.deepEqual(shifted.state.data.positions,plain.state.data.positions);
    assert.deepEqual(shifted.state.lengthMultipliers,plain.state.lengthMultipliers);
    assert.deepEqual(shifted.diagnostics.certificate,plain.diagnostics.certificate);
    assert.equal(shifted.diagnostics.evaluations,plain.diagnostics.evaluations);
    assert.ok(shifted.diagnostics.elasticEnergy>1e22);
});

test('a zero backsolve budget rejects the prepared step without consuming history or time',()=>{
    const f=makeFixture(160,'contact-free'),opts=options(f),before=f.state.data.positions.map(p=>[...p]);
    const r=advance(f.state,{...opts,budget:{...opts.budget,linearSolves:0}});
    assert.equal(r.accepted,false);assert.equal(r.status,'linear-solve-budget-exhausted');assert.equal(r.state,f.state);
    assert.equal(r.diagnostics.linearSolves,0);assert.equal(r.diagnostics.historyCommits,0);
    assert.deepEqual(f.state.data.positions,before);assert.equal(f.state.time,0);
});
