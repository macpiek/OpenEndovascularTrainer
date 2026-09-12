import assert from 'node:assert/strict';
import {readFileSync,writeFileSync,existsSync} from 'node:fs';
import {resolve} from 'node:path';
import {pathToFileURL} from 'node:url';
import {createHash} from 'node:crypto';

const roots=process.argv.slice(2,4);assert.equal(roots.length,2,'Pass frozen baseline and candidate roots');
const load=(root,path)=>import(pathToFileURL(resolve(root,path)));
const sides=await Promise.all(roots.map(async root=>({root,
    step:await load(root,'src/physics/kirchhoffCompositeJointTimeStep.js'),
    fixture:await load(root,'tests/fixtures/compositeRelativeSearch.js'),mesh:await load(root,'tests/fixtures/compositeMechanicalMesh.js')})));
const summarize=xs=>{const s=xs.slice().sort((a,b)=>a-b);return {mean:xs.reduce((a,b)=>a+b,0)/xs.length,median:s[Math.floor(xs.length/2)],p95:s[Math.ceil(xs.length*.95)-1]};};
const cases=[{name:'open-lumen-65',geometry:{n:65,lumen:true},input:{force:[.01,0,0]}},
    {name:'loaded-lumen-17',geometry:{n:17,lumen:true,offset:.04075,wireSlope:-.0005},input:{force:[.1,.4,0],forceNode:0}},
    {name:'loaded-lumen-65',geometry:{n:65,lumen:true,offset:.0406,wireSlope:-.0002},input:{force:[.1,.4,0],forceNode:0}},
    {name:'two-axis-sliding-17',geometry:{n:17,lumen:true,offset:.04075,wireSlope:-.0005},input:{force:[.1,.4,.02],forceNode:0}},
    {name:'graded-mechanics-15-with-128-contact-sites',meshNodes:[0,1,2,3,4,5,6,7,8,12,16,24,32,48,64],input:{force:[.1,.4,0],forceNode:0}}];
const files=['src/physics/kirchhoffCompositeJointTimeStep.js','src/physics/kirchhoffCompositeRelativeDirection.js','src/physics/kirchhoffCompositeZeroDuals.js','src/physics/kirchhoffCompositeSparseDirection.js','src/physics/kirchhoffCompositeFrictionLineSearch.js',
    'src/physics/kirchhoffCompositeJointLumenRows.js','src/physics/kirchhoffCompositeJointLumenFrictionRows.js','tests/fixtures/compositeRelativeSearch.js','tests/fixtures/compositeMechanicalMesh.js'];
const report={scope:'Paired whole-step CPU benchmark of direct sparse original assembly/certification and compressed factor construction, including a graded mechanical grid with all physical contact samples; original equations and acceptance; synthetic Node shafts, not anatomy or browser FPS',
    warmupPairs:10,measuredPairs:24,sources:roots.map(root=>({root,sha256:Object.fromEntries(files.map(name=>[name,existsSync(resolve(root,name))?createHash('sha256').update(readFileSync(resolve(root,name))).digest('hex'):null]))})),cases:[]};
for(const spec of cases) {
    const contexts=sides.map(s=>{const f=spec.meshNodes?s.mesh.mechanicalMeshFixture(spec.meshNodes):s.fixture.relativeSearchFixture(spec.geometry);return {s,f,workspace:s.step.createCompositeJointTimeStepWorkspace(f.state)};});
    const records=[[],[]];let maxShapeError=0,maxSpinError=0,maxReactionError=0;
    for(let repeat=-report.warmupPairs;repeat<report.measuredPairs;repeat++) {
        const results=[];
        for(const side of repeat%2===0?[0,1]:[1,0]) {
            const c=contexts[side],r=c.s.step.advanceCompositeJointTimeStep(c.f.state,c.s.fixture.relativeSearchOptions(c.f,c.f.state,{...spec.input,workspace:c.workspace}));
            assert.equal(r.accepted,true,JSON.stringify({scenario:spec.name,side,status:r.status,error:r.error,diagnostics:r.diagnostics}));results[side]=r;
            const d=r.diagnostics,p=d.certificate;assert.ok(p.converged&&p.contact.converged&&p.friction.converged);
            assert.ok(p.force<=1e-7&&p.torque<=1e-8&&p.length<=1e-8&&p.boundary<=1e-9);
            if(repeat>=0)records[side].push({totalMs:d.totalMs,directionMs:d.directionMs,preparationMs:d.preparationMs,iterationMs:d.iterationMs,
                directions:d.directions,evaluations:d.evaluations,contactQueries:d.contactQueries,linearSolves:d.linearSolves,
                originalEntries:Math.max(...d.directionSystems.map(s=>s.originalEntries)),originalBandEntries:Math.max(...d.directionSystems.map(s=>s.originalBandEntries)),
                fullOriginalMaterialized:Math.max(...d.directionSystems.map(s=>s.materializedOriginalBandEntries)),fullNumericalMaterialized:Math.max(...d.directionSystems.map(s=>s.materializedNumericalBandEntries)),
                originalUnknowns:d.unknowns,solvedUnknowns:Math.max(...d.directionSystems.map(s=>s.solvedUnknowns)),
                eliminatedZeroDuals:Math.min(...d.directionSystems.map(s=>s.eliminatedZeroDuals)),
                frictionConeTrials:d.frictionConeTrials??0,frictionConeAccepted:d.frictionConeAccepted??0,
                force:p.force,torque:p.torque,length:p.length,minimumGap:p.contact.minGap,minimumForce:p.contact.minForce,
                frictionSlip:Math.max(...p.friction.samples.map(s=>s.slipResidual)),frictionWork:Math.max(...p.friction.samples.map(s=>s.workGap)),
                frictionCone:Math.max(...p.friction.samples.map(s=>s.coneViolation))});
        }
        const [a,b]=results;assert.equal(a.state.step,b.state.step);assert.equal(a.state.time,b.state.time);
        for(const [id,p] of a.state.toolPositions)p.forEach((v,i)=>v.forEach((x,k)=>maxShapeError=Math.max(maxShapeError,Math.abs(x-b.state.toolPositions.get(id)[i][k]))));
        for(const [id,angles] of a.state.angles)angles.forEach((x,i)=>maxSpinError=Math.max(maxSpinError,Math.abs(x-b.state.angles.get(id)[i])));
        for(const [key,field] of [['lumenContactState','normalForces'],['lumenFrictionState','tractions']])a.state[key][field].forEach((x,i)=>maxReactionError=Math.max(maxReactionError,Math.abs(x-b.state[key][field][i])));
        assert.ok(maxShapeError<=1e-9&&maxSpinError<=1e-9&&maxReactionError<=1e-7);
        for(const key of ['state','perTool','boundaryForces','contactForces','spinReactions','balances'])assert.deepEqual(a[key],b[key],key);
        for(const key of ['certificate','directions','evaluations','contactQueries','linearSolves','acceptedAlphas'])assert.deepEqual(a.diagnostics[key],b.diagnostics[key],key);
        assert.ok(b.diagnostics.directionSystems.every(s=>s.originalStorage==='csr'&&s.materializedOriginalBandEntries===0&&s.materializedNumericalBandEntries===0));
    }
    const entry={...spec,maxShapeError,maxSpinError,maxReactionError,results:records.map((r,side)=>({root:roots[side],
        statistics:Object.fromEntries(Object.keys(r[0]).map(key=>[key,summarize(r.map(x=>x[key]))])),records:r}))};report.cases.push(entry);
    console.log(JSON.stringify({...entry,results:entry.results.map(({records,...r})=>r)}));
    if(process.argv[4])writeFileSync(process.argv[4],JSON.stringify(report,null,2)+'\n');
}
