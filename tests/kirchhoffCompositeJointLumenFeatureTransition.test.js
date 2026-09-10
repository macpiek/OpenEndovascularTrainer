import assert from 'node:assert/strict';
import test from 'node:test';
import {createCompositeChainLayout} from '../src/physics/kirchhoffCompositeChain.js';
import {captureCompositeReferenceFrames} from '../src/physics/kirchhoffCompositeElement.js';
import {createCompositeJointLumenRows} from '../src/physics/kirchhoffCompositeJointLumenRows.js';
import {createCompositeJointLumenFrictionRows} from '../src/physics/kirchhoffCompositeJointLumenFrictionRows.js';

const tol={force:1e-7,lumenGap:1e-8,lumenNcp:1e-8,lumenWork:1e-8,frictionSlip:1e-8,frictionCone:1e-8,frictionWork:1e-8,frictionEquation:1e-9,linearConstraint:1e-10},
    close=(a,b,t=3e-6,where='')=>assert.ok(Number.isFinite(a)&&Number.isFinite(b)&&Math.abs(a-b)<=t*Math.max(1,Math.abs(a),Math.abs(b)),`${where}: ${a} != ${b}`),
    keys=row=>[...Array.from(row.commonDofs,d=>`c${d}`),...Array.from(row.relativeDofs,d=>`r${d}`)];

function fixture(initialFeature) {
    const shift=initialFeature==='side'?0:.18,coordinates=[0,2,4],layout=createCompositeChainLayout([['wire','catheter'],['wire','catheter']]),
        positions=new Map([['catheter',[[0,0,0],[2,0,0],[4,0,0]]],['wire',[[1.1+shift,.35,.015],[3.5+shift,.37,.02],[4.5+shift,.41,.03]]]]),
        modes=coordinates.map((_,node)=>({node,basis:[[.8,.6,0],[-.6,.8,0],[0,0,1]],relativeDofs:[3*node,3*node+1,3*node+2]})),
        state={layout,coordinates,modes,relativeToolId:'wire',toolPositions:positions,relative:new Float64Array(9),
            angles:new Map([['wire',new Float64Array([.1,.31])],['catheter',new Float64Array([-.2,-.13])]]),
            tools:[...positions].map(([id,p])=>({id,reference:captureCompositeReferenceFrames(p),dsDx:1}))},candidate=structuredClone(state),dt=.02,
        contacts={mode:'lumen-coulomb',chartId:'native-feature-transition',forcePerLength:1,
            friction:{law:'coulomb',mu:[.07,.09],forcePerLength:5,materialPath:'linear-affine-maps',rateMode:'backward-euler-grid',slipModel:'implicit-backward-euler-surface-rate',finiteStepSlipKnown:false},
            pairs:[{id:'native-distal-sample',innerToolId:'wire',outerToolId:'catheter',innerEdge:1,outerEdge:1,
                innerMaterialSegmentId:'wire-last',outerMaterialSegmentId:'catheter-last',lumenRadius:.5,innerRadius:.16,quadrature:[.25],
                openDistal:true,portalFilletRadius:.15,feature:initialFeature,featurePolicy:'native-side-fillet'}]},
        normal=createCompositeJointLumenRows({...state,contacts:{...contacts,mode:'lumen-normal',friction:'none'},tolerances:tol,preserveSampleReactions:true}),
        prepared={dt,previousPositions:structuredClone(positions),inertiaEdges:layout.edgeToolIds.map((ids,edge)=>({tools:ids.map(id=>({id,
            materialMap:{sStart:20+2*edge+(id==='wire'?0:10),dsDx:1,dsDt:id==='wire'?.11:-.07}}))}))};
    normal.normalForces[0]=1.7;normal.prepareGauge({toolPositions:positions,consumeQuery:()=>{}});
    const friction=createCompositeJointLumenFrictionRows({state,candidate,contacts,normal,prepared,dt,tolerances:tol,normalRowOffset:0,frictionRowOffset:1});
    friction.prepare({consumeQuery:()=>{}});friction.tractions.set([.18,-.16]);
    candidate.angles.get('wire')[1]+=.17;candidate.angles.get('catheter')[1]-=.11;
    return {state,candidate,normal,friction,initialFeature,contacts,basePositions:structuredClone(positions)};
}
function refresh(f) {
    const common=new Float64Array(f.state.layout.dofCount),relative=new Float64Array(f.candidate.relative.length),args={toolPositions:f.candidate.toolPositions,commonResidual:common,relativeResidual:relative,order:'full'};
    f.normal.refresh({...args,consumeQuery:()=>{}});const proof=f.friction.refresh(args);
    return {common,relative,proof,equations:[...f.normal.rows,...f.friction.rows].map(row=>row.residual)};
}
function perturb(f,support,column,amount) {
    if(column<support.commonDofs.length) {
        const dof=support.commonDofs[column],layout=f.state.layout,node=Array.from(layout.positions).findIndex(p=>dof>=p&&dof<p+3);
        if(node>=0)for(const p of f.candidate.toolPositions.values())p[node][dof-layout.positions[node]]+=amount;
        else for(const [id,spins] of layout.spins) {const edge=Array.from(spins).indexOf(dof);if(edge>=0)f.candidate.angles.get(id)[edge]+=amount;}
    } else {
        const dof=support.relativeDofs[column-support.commonDofs.length],mode=f.state.modes.find(m=>m.relativeDofs.includes(dof)),axis=mode.relativeDofs.indexOf(dof);
        mode.basis[axis].forEach((v,k)=>f.candidate.toolPositions.get('wire')[mode.node][k]+=amount*v);
    }
}
const force=(r,support)=>[...Array.from(support.commonDofs,d=>r.common[d]),...Array.from(support.relativeDofs,d=>r.relative[d])];
function snapshot(f) {
    return [...f.normal.rows,...f.friction.rows].map(row=>({keys:keys(row),jacobian:Array.from(row.jacobian),forceColumn:Array.from(row.forceColumn),
        geometricTangent:Array.from(row.geometricTangent),multiplierDerivative:row.multiplierDerivative,multiplierDofs:Array.from(row.multiplierDofs??[]),
        multiplierJacobian:Array.from(row.multiplierJacobian??[])}));
}
function derivatives(f,expectedFeature) {
    const base=refresh(f),support=f.friction.rows[0],supportKeys=keys(support),size=supportKeys.length,rows=snapshot(f),h=1e-6,
        positions=structuredClone(f.candidate.toolPositions),angles=structuredClone(f.candidate.angles),H=new Float64Array(size*size);
    assert.equal(size,14);assert.equal(f.normal.samples[0].currentFeature,expectedFeature);assert.equal(f.normal.samples[0].geometry.rawContact.kind,expectedFeature);
    for(const row of rows)for(let i=0;i<row.keys.length;i++)for(let j=0;j<row.keys.length;j++)H[supportKeys.indexOf(row.keys[i])*size+supportKeys.indexOf(row.keys[j])]+=row.geometricTangent[i*row.keys.length+j];
    for(let j=0;j<size;j++) {
        const result=[];
        for(const sign of [-1,1]) {
            f.candidate.toolPositions=structuredClone(positions);f.candidate.angles=structuredClone(angles);perturb(f,support,j,sign*h);
            const r=refresh(f);assert.equal(f.normal.samples[0].currentFeature,expectedFeature);result.push({equations:r.equations,force:force(r,support)});
        }
        for(let row=0;row<3;row++) {const local=rows[row].keys.indexOf(supportKeys[j]),expected=local<0?0:rows[row].jacobian[local];
            close((result[1].equations[row]-result[0].equations[row])/(2*h),expected,3e-6,`${expectedFeature} G row${row} col${j}`);}
        for(let i=0;i<size;i++)close((result[1].force[i]-result[0].force[i])/(2*h),H[i*size+j],4e-6,`${expectedFeature} DB ${i},${j}`);
    }
    f.candidate.toolPositions=positions;f.candidate.angles=angles;refresh(f);
    for(let source=0;source<3;source++) {
        const storage=source===0?f.normal.normalForces:f.friction.tractions,index=source===0?0:source-1,old=storage[index],results=[];
        for(const sign of [-1,1]) {storage[index]=old+sign*h;const r=refresh(f);results.push({equations:r.equations,force:force(r,support)});}storage[index]=old;
        for(let i=0;i<size;i++){const local=rows[source].keys.indexOf(supportKeys[i]);close((results[1].force[i]-results[0].force[i])/(2*h),local<0?0:rows[source].forceColumn[local],3e-8,`${expectedFeature} B ${source},${i}`);}
        for(let i=0;i<3;i++) {const row=rows[i],cross=row.multiplierDofs.indexOf(source),expected=i===source?row.multiplierDerivative:cross<0?0:row.multiplierJacobian[cross];
            close((results[1].equations[i]-results[0].equations[i])/(2*h),expected,3e-8,`${expectedFeature} dual ${i},${source}`);}
    }
    const restored=refresh(f);assert.deepEqual(restored,base);return snapshot(f);
}

for(const initialFeature of ['side','distal-fillet'])test(`loaded ${initialFeature} round trip changes the original native feature in one manager without changing row, sample or traction ownership`,()=>{
    const f=fixture(initialFeature),normalRows=f.normal.rows,frictionRows=f.friction.rows,normalRow=normalRows[0],tangentRows=frictionRows.slice(),
        Fn=f.normal.normalForces,Ft=f.friction.tractions,sample=f.normal.samples[0],sampleId=sample.sampleId,
        rowArrays=[normalRow,...tangentRows].map(r=>[r.jacobian,r.forceColumn,r.geometricTangent]),opposite=initialFeature==='side'?'distal-fillet':'side';
    let first;
    for(const [iteration,feature] of [initialFeature,opposite,initialFeature].entries()) {
        const shift=(feature===initialFeature?0:initialFeature==='side'?.18:-.18);
        f.candidate.toolPositions=structuredClone(f.basePositions);for(const p of f.candidate.toolPositions.get('wire'))p[0]+=shift;
        const result=derivatives(f,feature);if(iteration===0)first=result;else if(iteration===2)assert.deepEqual(result,first);
        assert.equal(f.normal.rows,normalRows);assert.equal(f.friction.rows,frictionRows);assert.equal(normalRows[0],normalRow);
        assert.equal(f.normal.normalForces,Fn);assert.equal(f.friction.tractions,Ft);assert.deepEqual(Array.from(Fn),[1.7]);assert.deepEqual(Array.from(Ft),[.18,-.16]);
        assert.equal(f.normal.samples[0],sample);assert.equal(sample.sampleId,sampleId);assert.equal(sample.feature,initialFeature);
        [normalRow,...f.friction.rows].forEach((row,i)=>{assert.equal(row,i===0?normalRow:tangentRows[i-1]);
            for(const [j,key] of ['jacobian','forceColumn','geometricTangent'].entries())assert.equal(row[key],rowArrays[i][j]);});
    }
});
