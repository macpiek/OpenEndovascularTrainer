import assert from 'node:assert/strict';
import test from 'node:test';
import {captureCompositeReferenceFrames,compileCompositeMaterial} from '../src/physics/kirchhoffCompositeElement.js';
import {createCompositeChainLayout,createCompositeChainWorkspace,assembleCompositeChain} from '../src/physics/kirchhoffCompositeChain.js';
import {assembleCompositeTranslationalInertia,scatterCompositeTranslationalInertia} from '../src/physics/kirchhoffCompositeKinematics.js';
import {createCompositeToolLengthWorkspace,evaluateCompositeToolLengths} from '../src/physics/kirchhoffCompositeToolLengths.js';
import {assembleCompositeRelativeCluster,createCompositeRelativeClusterStructure} from '../src/physics/kirchhoffCompositeRelativeCluster.js';
import {createCompositeRelativeDirectionWorkspace as create,solveCompositeRelativeDirection as solve} from '../src/physics/kirchhoffCompositeRelativeDirection.js';

const tolerance={force:1e-9,torque:1e-10};
const close=(a,b,t=2e-9)=>assert.ok(Number.isFinite(a)&&Number.isFinite(b)&&Math.abs(a-b)<=t*(1+Math.max(Math.abs(a),Math.abs(b))),`${a} != ${b}`);
const vectorClose=(a,b,t)=>{assert.equal(a.length,b.length);a.forEach((v,i)=>close(v,b[i],t));};
const packed=(i,j,band)=>Math.max(i,j)*band+Math.abs(i-j);
const cross=(a,b)=>[a[1]*b[2]-a[2]*b[1],a[2]*b[0]-a[0]*b[2],a[0]*b[1]-a[1]*b[0]];
const unit=v=>{const l=Math.hypot(...v);return v.map(x=>x/l);};

function fixture({count=11,nodes=[4,5,6],dimensions=null,edgeToolIds=null,elementBackend='wasm',curved=true}={}) {
    const rest=Array.from({length:count},(_,i)=>[2*i,curved?.17*Math.sin(i*.3):0,curved?.12*Math.cos(i*.27):0]);
    const positions=rest.map((p,i)=>p.map((v,j)=>v+(curved?.03*Math.sin(i*.2+j):0))),coordinates=Array.from({length:count},(_,i)=>2*i);
    const layout=createCompositeChainLayout(edgeToolIds??Array.from({length:count-1},()=>['wire','catheter'])),chain=createCompositeChainWorkspace(layout,{elementBackend});
    const data={positions,coordinates,reference:captureCompositeReferenceFrames(rest),tools:[
        {id:'wire',angles:Float64Array.from({length:count-1},(_,i)=>curved?.13*Math.sin(i*.7):0),dsDx:1.3,
            material:compileCompositeMaterial({stiffness:[[3,.3,.2],[.3,5,-.1],[.2,-.1,2]],intrinsic:[.03,-.07,.13],energyOffset:.2})},
        {id:'catheter',angles:new Float64Array(count-1),dsDx:.8,material:compileCompositeMaterial({EI1:20,EI2:17,GJ:8})}]};
    const modes=nodes.map((node,index)=>{
        if(dimensions?.[index]===3)return{node,basis:[[Math.cos(.37),Math.sin(.37),0],[-Math.sin(.37),Math.cos(.37),0],[0,0,1]]};
        const t=unit(positions[node+1].map((v,i)=>v-positions[node-1][i])),b=unit(cross([0,0,1],t));
        return{node,basis:[b,cross(t,b)]};});
    const inertia={dt:.13,previousPositions:positions.map((p,i)=>p.map((v,j)=>v-.003*Math.cos(i+j))),
        inertiaEdges:layout.edgeToolIds.map((ids,e)=>({tools:ids.map(id=>({id,massPerMaterialLength:id==='wire'?2.4:4.1,
            materialMap:{sStart:100+e,dsDx:id==='wire'?1.3:.8,dsDt:id==='wire'?[-.7,-2.1]:[.4,1.2]},
            oldMaterialVelocities:id==='wire'?[[.3,-.2,.1],[.8,.4,-.3]]:[[-.7,.1,.2],[-.4,-.1,.6]]}))}))};
    assembleCompositeChain(data,chain);
    for(let edge=0;edge<count-1;edge++) {
        const local=assembleCompositeTranslationalInertia({coordinates:coordinates.slice(edge,edge+2),positions:positions.slice(edge,edge+2),
            previousPositions:inertia.previousPositions.slice(edge,edge+2),dt:inertia.dt,tools:inertia.inertiaEdges[edge].tools});
        const dofs=[edge,edge+1].flatMap(node=>[layout.positions[node],layout.positions[node]+1,layout.positions[node]+2]);
        dofs.forEach((d,i)=>{chain.gradient[d]+=local.gradient[i];for(let j=0;j<=i;j++)chain.hessian[packed(d,dofs[j],layout.band)]+=local.hessian[i*6+j];});
    }
    const cluster=assembleCompositeRelativeCluster({data,layout,modes,elementBackend,inertia}),fixed=new Uint8Array(layout.dofCount);
    fixed.fill(1,0,layout.positions[1]);
    const commonResidual=Float64Array.from(chain.gradient,(v,i)=>v+.05*Math.sin(.7*i)),relativeResidual=Float64Array.from(cluster.relative.gradient,(v,i)=>v+.2*Math.cos(.6*i));
    return {data,layout,chain,cluster,fixed,commonResidual,relativeResidual,modes,inertia};
}

function localRows(f) {
    const p=f.layout.positions,spin=f.layout.spins.get('wire');
    const definitions=[
        {anchorNode:4,commonDofs:Int32Array.from([p[4]+1,p[5]+2,spin[4]]),relativeDofs:Int32Array.from([0,1,2]),unit:'mm'},
        {anchorNode:6,commonDofs:Int32Array.from([p[6]+2]),relativeDofs:Int32Array.from([4,5]),unit:'N s'}];
    const tangent=new Float64Array(36);tangent[0]=-.9;tangent[3]=.17;tangent[3*6]=-.06;tangent[4*6+5]=.21;
    const rows=[{residual:.02,jacobian:[2,-.1,.2,1.5,.3,-.8],forceColumn:[-1,.3,-.4,-1,.6,.2],geometricTangent:tangent,tolerance:1e-11},
        {residual:-.007,jacobian:[.3,-.2,.5],forceColumn:[-.7,.4,.2],multiplierDerivative:-.4,tolerance:3e-12}];
    return {definitions,rows};
}

// TEST-ONLY dense assembly in natural [all common, all rho, all dual] order.
// No production permutation, band scatters, row equilibration, factor or
// residual calculator is used by this independent partial-pivot oracle.
function denseOriginal(f,definitions=[],rows=[]) {
    const n=f.layout.dofCount,r=f.cluster.relative.dofCount,N=n+r+rows.length;
    const A=Array.from({length:N},()=>new Float64Array(N)),F=Float64Array.from([...f.commonResidual,...f.relativeResidual,...rows.map(row=>row.residual)]);
    for(let i=0;i<n;i++)for(let j=0;j<n;j++)if(Math.abs(i-j)<f.layout.band)A[i][j]=f.chain.hessian[packed(i,j,f.layout.band)];
    for(let i=0;i<r;i++)for(let j=0;j<r;j++)if(Math.abs(i-j)<f.cluster.relative.band)A[n+i][n+j]=f.cluster.relative.hessian[packed(i,j,f.cluster.relative.band)];
    const c=f.cluster.coupling;
    c.commonDofs.forEach((d,row)=>{for(let k=c.rowOffsets[row];k<c.rowOffsets[row+1];k++){A[d][n+c.columns[k]]=c.values[k];A[n+c.columns[k]][d]=c.values[k];}});
    rows.forEach((row,index)=>{
        const def=definitions[index],support=[...def.commonDofs,...Array.from(def.relativeDofs,d=>n+d)],dual=n+r+index;
        A[dual][dual]=row.multiplierDerivative??0;
        def.multiplierDofs?.forEach((other,j)=>A[dual][n+r+other]=row.multiplierJacobian[j]);
        support.forEach((d,i)=>{A[dual][d]=row.jacobian[i];A[d][dual]=row.forceColumn[i];
            if(row.geometricTangent)support.forEach((other,j)=>{A[d][other]+=row.geometricTangent[i*support.length+j];});});
    });
    return {A,F,n,r,N};
}

function denseSolve(original,fixed) {
    const A=original.A.map(row=>row.slice()),b=Float64Array.from(original.F,v=>-v),N=b.length;
    for(let i=0;i<fixed.length;i++)if(fixed[i]){for(let j=0;j<N;j++)A[i][j]=A[j][i]=i===j?1:0;b[i]=0;}
    for(let k=0;k<N;k++) {
        let pivot=k;for(let i=k+1;i<N;i++)if(Math.abs(A[i][k])>Math.abs(A[pivot][k]))pivot=i;
        assert.ok(Math.abs(A[pivot][k])>1e-13,`Dense oracle pivot ${k} is singular`);
        [A[k],A[pivot]]=[A[pivot],A[k]];[b[k],b[pivot]]=[b[pivot],b[k]];
        for(let i=k+1;i<N;i++){const factor=A[i][k]/A[k][k];for(let j=k+1;j<N;j++)A[i][j]-=factor*A[k][j];b[i]-=factor*b[k];}
    }
    const x=new Float64Array(N);
    for(let i=N-1;i>=0;i--){let value=b[i];for(let j=i+1;j<N;j++)value-=A[i][j]*x[j];x[i]=value/A[i][i];}
    return x;
}
const run=(f,w,rows=[])=>solve(w,f.chain,{cluster:f.cluster,commonResidual:f.commonResidual,relativeResidual:f.relativeResidual,fixed:f.fixed,rows,tolerances:tolerance});
const allIncrement=result=>Float64Array.from([...result.commonIncrement,...result.relativeIncrement,...result.multiplierIncrement]);

test('many contact samples assemble and certify the original sparse equations without materializing either full band',()=>{
    const records=[];
    for(const count of [8,32,128]) {
        const f=fixture(),p=f.layout.positions[4],definitions=Array.from({length:count},()=>({anchorNode:4,
            commonDofs:Int32Array.of(p,p+1),relativeDofs:Int32Array.of(0),unit:'mm'}));
        const rows=definitions.map(()=>({residual:0,jacobian:[0,0,0],forceColumn:[-.2,.3,.4],multiplierDerivative:.5,tolerance:1e-10}));
        const w=create(f.layout,f.cluster,definitions),op=w.sparseOperator;
        assert.equal(op.nativeFactorEntries,0);assert.equal(op.materializedOriginalEntries,0);assert.equal(op.materializedNumericalEntries,0);
        const r=run(f,w,rows);assert.equal(r.converged,true);assert.equal(r.eliminatedZeroDuals,count);
        assert.equal(r.originalStorage,'csr');assert.equal(r.materializedOriginalBandEntries,0);assert.equal(r.materializedNumericalBandEntries,0);
        assert.equal(op.nativeFactorEntries,0);assert.equal(r.solvedCount,f.layout.dofCount+f.cluster.relative.dofCount);
        vectorClose(allIncrement(r),denseSolve(denseOriginal(f,definitions,rows),f.fixed));
        records.push({original:r.originalMatrixEntries,fullBand:r.originalBandEntries,factor:r.factorEntries});
    }
    assert.ok(records[2].original/records[1].original<4);
    assert.equal(records[0].factor,records[2].factor,'inactive pressure count does not enlarge the actual factor');
    assert.ok(records[2].original<records[2].fullBand/5);
});

test('sparse structural zero slots and diagnostic views refresh from current coefficients, including tiny and nonsymmetric terms',()=>{
    const f=fixture(),p=f.layout.positions[4],definitions=[{anchorNode:4,commonDofs:Int32Array.of(p,p+1),relativeDofs:Int32Array.of(0),unit:'mm'}];
    const rows=[{residual:0,jacobian:[0,0,0],forceColumn:[-.2,.3,.4],multiplierDerivative:.5,geometricTangent:new Float64Array(9),tolerance:1e-10}];
    const w=create(f.layout,f.cluster,definitions),first=allIncrement(run(f,w,rows)),view=w.originalMatrix,op=w.sparseOperator;
    const slots=op.rowPlans[0];rows[0].geometricTangent[1]=.123;rows[0].geometricTangent[3]=-.052;
    rows[0].forceColumn[2]=1e-300;
    const r=run(f,w,rows);assert.equal(r.converged,true);assert.notDeepEqual(allIncrement(r),first);
    assert.equal(op.original[slots.forceSlots[2]],1e-300);assert.equal(op.nativeFactorEntries,0);assert.equal(op.materializedNumericalEntries,0);
    const original=denseOriginal(f,definitions,rows);vectorClose(allIncrement(r),denseSolve(original,f.fixed));
    const next=w.originalMatrix;assert.equal(view,next);
    const permutation=[...w.common,...w.relative,...w.dual];
    original.A.forEach((row,i)=>row.forEach((v,j)=>{
        const a=permutation[i],b=permutation[j],packed=w.packedLayout;
        close(v,b<packed.starts[a]||b>packed.ends[a]?0:next[packed.offsets[a]+b],1e-14);
    }));
    const sparseIncrement=allIncrement(r),full=solve(w,f.chain,{...f,rows,tolerances:tolerance,eliminateZeroDuals:false});
    assert.equal(full.converged,true);vectorClose(allIncrement(full),sparseIncrement);
    assert.ok(op.nativeFactorEntries>0&&op.materializedNumericalEntries>0);
});

test('exact zero-dual substitution retains physical columns and geometric tangents in the independent full equations',()=>{
    const f=fixture(),p=f.layout.positions[4];
    const definitions=[0,1,2].map(i=>({anchorNode:4,commonDofs:Int32Array.of(p,p+1),relativeDofs:Int32Array.of(0),unit:'mm',
        multiplierDofs:i===0?new Int32Array():Int32Array.of(i-1)}));
    const rows=definitions.map((d,i)=>({residual:0,jacobian:[0,0,0],forceColumn:[-.5-i,.2,.4],multiplierDerivative:.5+i,
        multiplierJacobian:i===0?[]:[-.7],geometricTangent:Float64Array.from([.2,.03,0,-.04,.1,0,0,0,.3]),tolerance:1e-10}));
    const original=denseOriginal(f,definitions,rows),w=create(f.layout,f.cluster,definitions),r=run(f,w,rows),delta=allIncrement(r);
    assert.equal(r.converged,true);assert.equal(r.eliminatedZeroDuals,3);assert.equal(r.solvedCount,r.count-3);
    assert.ok(r.multiplierIncrement.every(v=>v===0));vectorClose(delta,denseSolve(original,f.fixed));
    const permutation=[...w.common,...w.relative,...w.dual];
    original.A.forEach((row,i)=>close(r.originalLinearResidual[permutation[i]],original.F[i]+row.reduce((sum,v,j)=>sum+v*delta[j],0),1e-10));
    const uncompressed=solve(create(f.layout,f.cluster,definitions),f.chain,{...f,rows,tolerances:tolerance,eliminateZeroDuals:false});
    assert.equal(uncompressed.eliminatedZeroDuals,0);vectorClose(delta,allIncrement(uncompressed));
});

test('reactivated rows, tiny nonzero geometry coefficients and unresolved dual cycles stay in the full equations',()=>{
    const f=fixture(),p=f.layout.positions[4],definitions=[0,1].map(i=>({anchorNode:4,commonDofs:Int32Array.of(p),relativeDofs:Int32Array.of(0),unit:'mm',multiplierDofs:Int32Array.of(1-i)}));
    const rows=definitions.map(()=>({residual:0,jacobian:[0,0],forceColumn:[-.2,.3],multiplierDerivative:1,multiplierJacobian:[0],tolerance:1e-10})),w=create(f.layout,f.cluster,definitions);
    assert.equal(run(f,w,rows).eliminatedZeroDuals,2);
    rows[0].residual=1e-20;assert.equal(run(f,w,rows).eliminatedZeroDuals,1);
    rows[0].residual=0;rows[0].jacobian[0]=1e-20;assert.equal(run(f,w,rows).eliminatedZeroDuals,1);
    rows[0].jacobian[0]=0;rows[0].multiplierJacobian[0]=.2;rows[1].multiplierJacobian[0]=-.3;
    const r=run(f,w,rows);assert.equal(r.converged,true);assert.equal(r.eliminatedZeroDuals,0);
    vectorClose(allIncrement(r),denseSolve(denseOriginal(f,definitions,rows),f.fixed));
    rows[0].multiplierJacobian[0]=rows[1].multiplierJacobian[0]=0;
    const shifted=solve(w,f.chain,{...f,rows,tolerances:tolerance,numericalShift:.01});assert.equal(shifted.eliminatedZeroDuals,0);
});

test('zero-dual pattern storage stays bounded and never caches numeric stiffness or residuals across solves',()=>{
    const f=fixture(),p=f.layout.positions[4],definitions=Array.from({length:7},()=>({anchorNode:4,commonDofs:Int32Array.of(p),relativeDofs:Int32Array.of(0),unit:'mm'}));
    const rows=definitions.map(()=>({residual:0,jacobian:[0,0],forceColumn:[-.2,.3],multiplierDerivative:1,tolerance:1e-10})),w=create(f.layout,f.cluster,definitions);
    for(let i=0;i<9;i++) {
        rows.forEach((row,k)=>{row.residual=k===i%7?.01:0;row.forceColumn[0]=-.1*(i+1);});
        const r=run(f,w,rows);assert.equal(r.converged,true);assert.equal(r.eliminatedZeroDuals,6);assert.ok(w.zeroDualSystems.size<=4);
        vectorClose(allIncrement(r),denseSolve(denseOriginal(f,definitions,rows),f.fixed));
    }
});

test('numerical regularization solves its explicit scaled system while preserving the distinct original equation proof',()=>{
    const f=fixture(),{definitions,rows}=localRows(f),original=denseOriginal(f,definitions,rows),w=create(f.layout,f.cluster,definitions),shift=.01;
    const r=solve(w,f.chain,{cluster:f.cluster,commonResidual:f.commonResidual,relativeResidual:f.relativeResidual,fixed:f.fixed,rows,tolerances:tolerance,numericalShift:shift});
    assert.equal(r.numericalConverged,true,JSON.stringify(r.numericalProof));assert.equal(r.converged,false);assert.equal(r.numericalShift,shift);
    assert.equal(r.certified,false);assert.equal(r.nonlinearStepAccepted,false);
    const permutation=[...w.common,...w.relative,...w.dual],modified={...original,A:original.A.map(row=>row.slice())};
    permutation.forEach((d,i)=>{if(!f.fixed[i])modified.A[i][i]+=shift/(w.scales[d]*w.scales[d]);});
    const expected=denseSolve(modified,f.fixed);vectorClose(allIncrement(r),expected,2e-9);
    const delta=allIncrement(r);
    original.A.forEach((row,i)=>{
        const residual=original.F[i]+row.reduce((sum,v,j)=>sum+v*delta[j],0);
        close(r.originalLinearResidual[permutation[i]],residual,1e-10);
        for(let j=0;j<row.length;j++)if(permutation[j]>=w.packedLayout.starts[permutation[i]]&&permutation[j]<=w.packedLayout.ends[permutation[i]])
            close(w.originalMatrix[w.packedLayout.offsets[permutation[i]]+permutation[j]],row[j],1e-14);
    });
    assert.ok(r.proof.force>tolerance.force||r.proof.constraints.some(c=>!c.converged));
    const cold=run(f,create(f.layout,f.cluster,definitions),rows),restored=run(f,w,rows);
    assert.equal(restored.converged,true);assert.equal(restored.numericalShift,0);assert.deepEqual(allIncrement(restored),allIncrement(cold));
});

test('regularization cannot hide an incompatible held physical row or move fixed coordinates',()=>{
    const f=fixture(),definitions=[{anchorNode:0,commonDofs:Int32Array.of(0,1),relativeDofs:new Int32Array(),unit:'mm'}],
        rows=[{residual:2e-8,jacobian:[1,0],forceColumn:[-1,0],tolerance:1e-9}],w=create(f.layout,f.cluster,definitions);
    const r=solve(w,f.chain,{cluster:f.cluster,commonResidual:f.commonResidual,relativeResidual:f.relativeResidual,fixed:f.fixed,rows,tolerances:tolerance,numericalShift:.01});
    assert.equal(r.heldMultiplierRows[0],1);assert.equal(r.multiplierIncrement[0],0);assert.equal(r.converged,false);assert.equal(r.numericalConverged,false);
    assert.equal(r.proof.constraints[0].residual,2e-8);assert.equal(r.numericalProof.constraints[0].residual,2e-8);
    f.fixed.forEach((fixed,i)=>{if(fixed)assert.equal(r.commonIncrement[i],0);});
});

test('selective dual regularization preserves excluded original constraint equations',()=>{
    const f=fixture(),{definitions,rows}=localRows(f),original=denseOriginal(f,definitions,rows),shift=.01;
    for(const numericalDualRows of [[0],[1],[]]) {
        const w=create(f.layout,f.cluster,definitions),r=solve(w,f.chain,{...f,rows,tolerances:tolerance,numericalShift:shift,numericalDualRows});
        assert.equal(r.numericalConverged,true,JSON.stringify(r.numericalProof));
        const delta=allIncrement(r),permutation=[...w.common,...w.relative,...w.dual],primalCount=w.common.length+w.relative.length,
            modified={...original,A:original.A.map(row=>row.slice())};
        permutation.forEach((d,i)=>{
            if(!f.fixed[i]&&(i<primalCount||numericalDualRows.includes(i-primalCount)))modified.A[i][i]+=shift/(w.scales[d]*w.scales[d]);
        });
        vectorClose(delta,denseSolve(modified,f.fixed),2e-9);
        rows.forEach((_,index)=>{
            const i=primalCount+index,residual=original.F[i]+original.A[i].reduce((sum,v,j)=>sum+v*delta[j],0);
            close(r.proof.constraints[index].residual,residual,1e-10);
            if(numericalDualRows.includes(index)) {
                assert.ok(Math.abs(residual)>rows[index].tolerance,'selected row has a numerical shift visible in its original proof');
                close(residual,-shift*delta[i]/(w.scales[w.dual[index]]**2),1e-10);
            } else {
                assert.equal(r.proof.constraints[index].converged,true);
                assert.ok(Math.abs(residual)<=rows[index].tolerance,'unselected physical constraint stays exact');
            }
        });
        f.fixed.forEach((fixed,i)=>{if(fixed)assert.equal(r.commonIncrement[i],0);});
    }
});

test('null dual mask preserves default regularization and mask changes do not persist in reused workspace',()=>{
    const f=fixture(),{definitions,rows}=localRows(f),w=create(f.layout,f.cluster,definitions),options={...f,rows,tolerances:tolerance,numericalShift:.01};
    const baseline=allIncrement(solve(w,f.chain,options));
    assert.deepEqual(allIncrement(solve(w,f.chain,{...options,numericalDualRows:null})),baseline);
    const selected=allIncrement(solve(w,f.chain,{...options,numericalDualRows:[0]}));assert.notDeepEqual(selected,baseline);
    assert.deepEqual(allIncrement(solve(w,f.chain,options)),baseline);
    assert.deepEqual(allIncrement(solve(w,f.chain,{...options,numericalDualRows:[0,1]})),baseline);
});

test('invalid numerical dual masks reject before assembly and leave later original solves unchanged',()=>{
    const f=fixture(),{definitions,rows}=localRows(f),w=create(f.layout,f.cluster,definitions),baseline=allIncrement(run(f,w,rows));
    for(const numericalDualRows of [false,{},new Int32Array([0]),[0,0],[-1],[2],[.5],[NaN],[Infinity]]) {
        for(const numericalShift of [0,.01])assert.throws(()=>solve(w,f.chain,{...f,rows,tolerances:tolerance,numericalShift,numericalDualRows}),/numericalDualRows/);
    }
    assert.deepEqual(allIncrement(run(f,w,rows)),baseline);
});

test('invalid numerical shifts reject and cannot contaminate a later original solve',()=>{
    const f=fixture(),w=create(f.layout,f.cluster),expected=allIncrement(run(f,w));
    for(const numericalShift of [-1,NaN,Infinity])assert.throws(()=>solve(w,f.chain,{cluster:f.cluster,commonResidual:f.commonResidual,relativeResidual:f.relativeResidual,
        fixed:f.fixed,tolerances:tolerance,numericalShift}),/numericalShift/);
    assert.deepEqual(allIncrement(run(f,w)),expected);
});

function duplicateConstraintFixture() {
    const f=fixture(),p=f.layout.positions[4],spin=f.layout.spins.get('wire')[4],support=()=>({anchorNode:4,
        commonDofs:Int32Array.of(p+1,p,spin),relativeDofs:new Int32Array(),unit:'mm'}),
        definitions=[support(),support(),{...support(),multiplierDofs:Int32Array.of(0,1)}],
        rows=[{residual:.02,jacobian:[1,0,0],forceColumn:[-1,.2,.1],tolerance:1e-11,lambda:2},
            {residual:.02,jacobian:[1,0,0],forceColumn:[-.8,-.3,.4],tolerance:1e-11,lambda:3},
            {residual:-.03,jacobian:[.1,.3,.2],forceColumn:[.2,-.1,.3],multiplierDerivative:1,
                multiplierJacobian:[.2,-.4],tolerance:1e-11}];
    return {f,definitions,rows};
}

test('duplicate equations hold only one multiplier increment while distinct force and friction columns remain exact',()=>{
    const {f,definitions,rows}=duplicateConstraintFixture(),w=create(f.layout,f.cluster,definitions),original=denseOriginal(f,definitions,rows),
        before=structuredClone(rows),r=solve(w,f.chain,{...f,rows,tolerances:tolerance,holdDuplicateConstraints:true});
    assert.equal(r.converged,true,JSON.stringify(r.proof));assert.equal(r.duplicateConstraints.length,1);
    const {index:heldIndex,representative}=r.duplicateConstraints[0];assert.deepEqual([heldIndex,representative].sort(),[0,1]);
    assert.equal(r.heldMultiplierRows[heldIndex],1);assert.equal(r.multiplierIncrement[heldIndex],0);
    assert.equal(rows[heldIndex].lambda+r.multiplierIncrement[heldIndex],before[heldIndex].lambda);
    assert.deepEqual(rows,before);assert.equal(r.numericalShift,0);
    const modified={...original,A:original.A.map(row=>row.slice()),F:original.F.slice()},held=original.n+original.r+heldIndex;
    modified.A[held].fill(0);modified.A[held][held]=1;modified.F[held]=0;
    const delta=allIncrement(r);vectorClose(delta,denseSolve(modified,f.fixed),2e-9);
    original.A.forEach((row,i)=>{
        if(f.fixed[i])return;
        close(original.F[i]+row.reduce((sum,v,j)=>sum+v*delta[j],0),0,1e-9);
    });
    assert.ok(r.proof.constraints.every(c=>c.converged));
});

test('duplicate-equation handling is opt-in and exact, never merging tiny residual or Jacobian differences',()=>{
    for(const variant of ['default','false','rhs','jacobian','unit']) {
        const {f,definitions,rows}=duplicateConstraintFixture();
        if(variant==='rhs')rows[1].residual+=1e-16;
        if(variant==='jacobian')rows[1].jacobian[1]=1e-16;
        if(variant==='unit')definitions[1].unit='N s';
        const w=create(f.layout,f.cluster,definitions),options={...f,rows,tolerances:tolerance};
        if(variant!=='default')options.holdDuplicateConstraints=variant!=='false';
        const r=solve(w,f.chain,options);assert.deepEqual(r.duplicateConstraints,[],variant);assert.equal(r.heldMultiplierRows[1],0,variant);
    }
    const {f,definitions,rows}=duplicateConstraintFixture(),w=create(f.layout,f.cluster,definitions);
    assert.throws(()=>solve(w,f.chain,{...f,rows,tolerances:tolerance,holdDuplicateConstraints:1}),/holdDuplicateConstraints/);
});

test('one sliding Coulomb block retains normal-force and cross-tangent derivatives in the original joint solve',()=>{
    const f=fixture(),p=f.layout.positions[4],sw=f.layout.spins.get('wire')[4],sc=f.layout.spins.get('catheter')[4];
    const definitions=[
        {anchorNode:4,commonDofs:Int32Array.of(p+1),relativeDofs:Int32Array.of(0),unit:'mm'},
        {anchorNode:4,commonDofs:Int32Array.of(p,sw,sc),relativeDofs:Int32Array.of(1),multiplierDofs:Int32Array.of(0,2),unit:'mm'},
        {anchorNode:4,commonDofs:Int32Array.of(p,sw,sc),relativeDofs:Int32Array.of(1),multiplierDofs:Int32Array.of(0,1),unit:'mm'}];
    const Fn=2,mu=.3,k=50,Ft=[.1,-.05],slip=[.02,.01],z=Ft.map((v,i)=>v-k*slip[i]),length=Math.hypot(...z),u=z.map(v=>v/length),radius=mu*Fn;
    assert.ok(length>radius);
    const D=[[0,0],[0,0]].map((row,i)=>row.map((_,j)=>radius/length*((i===j?1:0)-u[i]*u[j])));
    // Independent affine slip derivatives and physical rate-power columns.
    // They deliberately differ; this is one current Newton linearization,
    // not an assertion that finite slip and instantaneous power are equal.
    const G=[[1,.15,-.11,.6],[.2,.44,-.55,-.2]],B=[[1,.12,-.10,.58],[.18,.43,-.54,-.21]];
    const rows=[{residual:-.02,jacobian:[-1,-.4],forceColumn:[1,.4],tolerance:1e-11},...[0,1].map(i=>({
        residual:(Ft[i]-radius*u[i])/k,jacobian:G[0].map((_,j)=>D[i][0]*G[0][j]+D[i][1]*G[1][j]),forceColumn:B[i].map(v=>-v),
        multiplierDerivative:(1-D[i][i])/k,multiplierJacobian:[-mu*u[i]/k,-D[i][1-i]/k],tolerance:1e-11}))];
    // Current physical reactions belong to the original mechanical residual.
    rows.forEach((row,i)=>{
        const force=i===0?Fn:Ft[i-1],def=definitions[i];
        def.commonDofs.forEach((d,j)=>f.commonResidual[d]+=force*row.forceColumn[j]);
        def.relativeDofs.forEach((d,j)=>f.relativeResidual[d]+=force*row.forceColumn[def.commonDofs.length+j]);
    });
    const original=denseOriginal(f,definitions,rows),expected=denseSolve(original,f.fixed),w=create(f.layout,f.cluster,definitions),r=run(f,w,rows);
    assert.ok(r.converged,JSON.stringify(r.proof));vectorClose(allIncrement(r),expected,5e-9);assert.ok(r.heldMultiplierRows.every(v=>v===0));
    assert.equal(r.factorizations,1);assert.equal(r.nonlinearStepAccepted,false);
    for(let i=0;i<3;i++)for(let j=0;j<3;j++)close(w.originalMatrix[w.packedLayout.offsets[w.dual[i]]+w.dual[j]],original.A[original.n+original.r+i][original.n+original.r+j],1e-14);
    const wrong=structuredClone(rows);wrong[1].multiplierJacobian.fill(0);wrong[2].multiplierJacobian.fill(0);
    const omitted=allIncrement(run(f,w,wrong));
    const mismatch=Math.max(...original.A.slice(original.n+original.r).map((row,i)=>Math.abs(original.F[original.n+original.r+i]+row.reduce((sum,v,j)=>sum+v*omitted[j],0))));
    assert.ok(mismatch>1e-4,'freezing Fn or dropping cross-tangent derivatives must fail the original contact equations');
});

test('a local dual block with zero diagonal still solves its coupled equations at fully prescribed geometry',()=>{
    const f=fixture(),definitions=[0,1].map(i=>({anchorNode:0,commonDofs:Int32Array.of(0,1),relativeDofs:new Int32Array(),multiplierDofs:Int32Array.of(1-i),unit:'mm'}));
    const rows=[{residual:.4,jacobian:[1,0],forceColumn:[-1,.1],multiplierJacobian:[2],tolerance:1e-11},
        {residual:-.3,jacobian:[0,1],forceColumn:[.2,-1],multiplierJacobian:[-3],tolerance:1e-11}];
    const original=denseOriginal(f,definitions,rows),w=create(f.layout,f.cluster,definitions),r=run(f,w,rows);
    assert.ok(r.converged,JSON.stringify(r.proof));assert.deepEqual(Array.from(r.heldMultiplierRows),[0,0]);
    vectorClose(allIncrement(r),denseSolve(original,f.fixed));vectorClose(r.multiplierIncrement,[-.1,-.2],1e-12);
    // Changed values reuse storage and factor the current dual block again.
    rows[0].multiplierJacobian[0]=4;const next=run(f,w,rows);assert.ok(next.converged);vectorClose(next.multiplierIncrement,[-.1,-.1],1e-12);
    // An explicitly inactive zero-increment gauge propagates only through
    // held dependencies; the original incompatible residual stays visible.
    rows[1].multiplierJacobian[0]=0;rows[1].residual=0;
    const held=run(f,w,rows);assert.equal(held.converged,false);assert.deepEqual(Array.from(held.heldMultiplierRows),[1,1]);
    assert.equal(held.proof.constraints[0].residual,.4);vectorClose(held.multiplierIncrement,[0,0],0);
});

test('dual block support is owned, local and explicit; stale or missing derivatives cannot reuse it',()=>{
    const f=fixture(),definition={anchorNode:4,commonDofs:Int32Array.of(f.layout.positions[4]),relativeDofs:Int32Array.of(0),unit:'mm'};
    const defs=[{...definition,multiplierDofs:Int32Array.of(1)},{...definition,multiplierDofs:Int32Array.of(0)}];
    const row={residual:0,jacobian:[1,.2],forceColumn:[-1,-.2],multiplierDerivative:1,multiplierJacobian:[.1],tolerance:1e-11},rows=[structuredClone(row),structuredClone(row)];
    const w=create(f.layout,f.cluster,defs);defs[0].multiplierDofs[0]=0;assert.ok(run(f,w,rows).converged);
    assert.throws(()=>run(f,w,[{...row,multiplierDofs:Int32Array.of(0)},row]),/Frozen local multiplier support/);
    assert.throws(()=>run(f,w,[{...row,multiplierJacobian:undefined},row]),/off-diagonal multiplier Jacobian/);
    assert.throws(()=>run(f,w,[{...row,multiplierJacobian:[NaN]},row]),/off-diagonal multiplier Jacobian/);
    assert.throws(()=>create(f.layout,f.cluster,defs),/diagonal/);
    assert.throws(()=>create(f.layout,f.cluster,[{...definition,multiplierDofs:Int32Array.of(1,1)},definition]),/unique/);
    assert.throws(()=>create(f.layout,f.cluster,[{...definition,multiplierDofs:Int32Array.of(2)},definition]),/in-range/);
    assert.throws(()=>create(f.layout,f.cluster,[{...definition,multiplierDofs:Int32Array.of(1)},{...definition,anchorNode:5}]),/one local contact anchor/);
});

test('one original common/rho/dual LU matches an independent dense oracle for GN and exact physical operators',()=>{
    for(const elementBackend of ['wasm','wasm-exact']) {
        const f=fixture({elementBackend}),{definitions,rows}=localRows(f),w=create(f.layout,f.cluster,definitions);
        const original=denseOriginal(f,definitions,rows),expected=denseSolve(original,f.fixed);
        const before={h:f.chain.hessian.slice(),g:f.commonResidual.slice(),cluster:structuredClone(f.cluster),rows:structuredClone(rows)};
        const result=run(f,w,rows),actual=allIncrement(result);
        assert.equal(result.converged,true,JSON.stringify(result.proof));vectorClose(actual,expected,3e-9);
        const permutation=[...w.common,...w.relative,...w.dual];
        original.A.forEach((row,i)=>row.forEach((v,j)=>{
            const a=permutation[i],b=permutation[j],value=b<w.packedLayout.starts[a]||b>w.packedLayout.ends[a]?0:w.originalMatrix[w.packedLayout.offsets[a]+b];
            close(value,v,2e-13);
        }));
        original.A.forEach((row,i)=>{
            const response=row.reduce((sum,v,j)=>sum+v*actual[j],0),residual=original.F[i]+response;
            close(result.originalLinearResidual[permutation[i]],residual,1e-10);
            if(i<f.fixed.length&&f.fixed[i]){assert.equal(result.commonIncrement[i],0);close(result.fixedReactionIncrement[i],response);close(result.fixedStationarity[i],residual);}
        });
        assert.ok(result.proof.commonForce<tolerance.force&&result.proof.relativeForce<tolerance.force&&result.proof.torque<tolerance.torque);
        assert.deepEqual(result.proof.constraints.map(c=>c.unit),['mm','N s']);assert.ok(result.proof.constraints.every(c=>c.converged));
        assert.equal(result.factorizations,1);assert.equal(result.linearSolves,1);assert.equal(result.certified,false);assert.equal(result.nonlinearStepAccepted,false);
        assert.deepEqual(f.chain.hessian,before.h);assert.deepEqual(f.commonResidual,before.g);assert.deepEqual(f.cluster,before.cluster);assert.deepEqual(rows,before.rows);
    }
});

test('the common diagnostic block is never added twice, even when diagnostic bytes are poisoned',()=>{
    const f=fixture(),w=create(f.layout,f.cluster),expected=denseSolve(denseOriginal(f),f.fixed),correct=allIncrement(run(f,w));
    vectorClose(correct,expected);
    const saved=f.cluster.common.hessian.slice();
    f.cluster.common.hessian.fill(NaN);f.cluster.common.gradient.fill(NaN);f.cluster.common.energy=NaN;
    vectorClose(allIncrement(run(f,w)),correct,1e-13);
    // Intentional wrong model: add affected common wire contributions again.
    const n=f.cluster.common.dofs.length,b=f.cluster.common.band;
    for(let i=0;i<n;i++)for(let j=Math.max(0,i-b+1);j<=i;j++)f.chain.hessian[packed(f.cluster.common.dofs[i],f.cluster.common.dofs[j],f.layout.band)]+=saved[i*b+i-j];
    const wrong=allIncrement(run(f,w));
    assert.ok(wrong.some((v,i)=>Math.abs(v-correct[i])>1e-3),'double-counting physical common inertia/stiffness must change the solution');
});

test('neighboring modes are solved jointly and deleting cross-mode blocks fails the full original equations',()=>{
    const f=fixture({curved:false}),original=denseOriginal(f),w=create(f.layout,f.cluster),correct=allIncrement(run(f,w));
    assert.ok(Math.abs(f.cluster.relative.hessian[packed(2,0,f.cluster.relative.band)])>1);
    const h=f.cluster.relative.hessian;
    for(let i=0;i<f.relativeResidual.length;i++)for(let j=Math.max(0,i-f.cluster.relative.band+1);j<i;j++)if(Math.floor(i/2)!==Math.floor(j/2))h[packed(i,j,f.cluster.relative.band)]=0;
    const wrong=allIncrement(run(f,w));
    assert.ok(wrong.some((v,i)=>Math.abs(v-correct[i])>1e-3));
    const relativeError=Math.max(...original.A.slice(original.n,original.n+original.r).map((row,i)=>Math.abs(original.F[original.n+i]+row.reduce((sum,v,j)=>sum+v*wrong[j],0))));
    assert.ok(relativeError>1,'independent mode solves must fail the coupled physical equation');
});

test('current H/C values are refreshed each direction while topology and frozen bases are checked',()=>{
    const f=fixture(),w=create(f.layout,f.cluster),first=allIncrement(run(f,w));
    const changed=structuredClone(f.cluster);changed.relative.hessian[0]*=1.7;changed.coupling.values[3]*=.6;f.cluster=changed;
    const expected=denseSolve(denseOriginal(f),f.fixed),actual=allIncrement(run(f,w));vectorClose(actual,expected);
    assert.ok(actual.some((v,i)=>Math.abs(v-first[i])>1e-5));
    assert.equal(run(f,w).factorizations,1,'a repeated direction factors the fresh original matrix');
    f.cluster.coupling.columns[0]++;assert.throws(()=>run(f,w),/frozen structure/);f.cluster.coupling.columns[0]--;
    f.cluster.modes[0].basis[0][0]+=.01;assert.throws(()=>run(f,w),/frozen transverse bases/);
});

test('fixed mixed supports preserve physical force-column asymmetry and full nonsymmetric geometric tangent',()=>{
    const f=fixture(),p=f.layout.positions[4],s=f.layout.spins.get('wire')[4];f.fixed[p]=1;
    const definitions=[{anchorNode:4,commonDofs:Int32Array.from([p,p+1,s]),relativeDofs:Int32Array.from([0,1]),unit:'mm'}];
    const tangent=new Float64Array(25);tangent[1*5+3]=.3;tangent[3*5+1]=-.8;tangent[4*5+4]=-400;
    const rows=[{residual:.003,jacobian:[7,2,.1,-3,.5],forceColumn:[-2,-1,.3,.7,-.2],geometricTangent:tangent,tolerance:1e-11}];
    const original=denseOriginal(f,definitions,rows),expected=denseSolve(original,f.fixed),w=create(f.layout,f.cluster,definitions),result=run(f,w,rows);
    assert.equal(result.converged,true,JSON.stringify(result.proof));vectorClose(allIncrement(result),expected);
    assert.equal(result.commonIncrement[p],0);assert.ok(Math.abs(result.fixedReactionIncrement[p])>1e-4);
    const physical=result.multiplierIncrement[0];rows[0].forceColumn=rows[0].jacobian.map(v=>-v);
    const wrong=run(f,w,rows);assert.ok(Math.abs(wrong.multiplierIncrement[0]-physical)>1e-3,'gap derivative cannot replace the signed physical force column');
});

test('boundary reactions act ON the chain and close an independently integrated translational impulse balance',()=>{
    // Two uniform linear material elements, both outer nodes prescribed.
    // A positive axial load acts on the free middle node. With consistent
    // mass, acceleration is triangular along the two physical elements.
    // This control derives momentum by integrating that velocity field,
    // independently of the assembled matrix or an assumed reaction sign.
    const f=fixture({count:3,nodes:[1],curved:false}),dt=.25,load=3.2;
    f.data.tools.forEach(tool=>{tool.angles.fill(0);tool.dsDx=1;tool.material=compileCompositeMaterial({EI1:2,EI2:3,GJ:4});});
    const inertia={dt,previousPositions:f.data.positions.map(p=>p.slice()),inertiaEdges:[0,1].map(edge=>({tools:f.data.tools.map(tool=>({
        id:tool.id,massPerMaterialLength:tool.id==='wire'?2.4:4.1,
        materialMap:{sStart:2*edge,dsDx:1,dsDt:[0,0]},oldMaterialVelocities:[[0,0,0],[0,0,0]]}))}))};
    assembleCompositeChain(f.data,f.chain);
    for(let edge=0;edge<2;edge++) {
        const local=assembleCompositeTranslationalInertia({coordinates:f.data.coordinates.slice(edge,edge+2),positions:f.data.positions.slice(edge,edge+2),
            previousPositions:inertia.previousPositions.slice(edge,edge+2),dt,tools:inertia.inertiaEdges[edge].tools});
        const dofs=[edge,edge+1].flatMap(node=>[f.layout.positions[node],f.layout.positions[node]+1,f.layout.positions[node]+2]);
        dofs.forEach((d,i)=>{f.chain.gradient[d]+=local.gradient[i];for(let j=0;j<=i;j++)f.chain.hessian[packed(d,dofs[j],f.layout.band)]+=local.hessian[i*6+j];});
    }
    f.cluster=assembleCompositeRelativeCluster({data:f.data,layout:f.layout,modes:f.modes,inertia});
    f.fixed.fill(1);const middle=f.layout.positions[1];f.fixed[middle]=0;
    f.commonResidual=f.chain.gradient.slice();f.commonResidual[middle]-=load;f.relativeResidual=f.cluster.relative.gradient.slice();
    const result=run(f,create(f.layout,f.cluster));assert.equal(result.converged,true);
    const velocity=Array.from(f.layout.positions,d=>result.commonIncrement[d]/dt);
    let momentum=0;
    for(let edge=0;edge<2;edge++)for(const material of inertia.inertiaEdges[edge].tools){
        const length=f.data.coordinates[edge+1]-f.data.coordinates[edge];
        momentum+=material.massPerMaterialLength*length*(velocity[edge]+velocity[edge+1])/2;
    }
    const momentumRate=momentum/dt,left=result.fixedReactionIncrement[f.layout.positions[0]],right=result.fixedReactionIncrement[f.layout.positions[2]];
    close(left,load/4,1e-12);close(right,load/4,1e-12);
    close(momentumRate,1.5*load,1e-12);close(momentumRate,load+left+right,1e-12);
    close(momentum,dt*(load+left+right),1e-12);
    assert.ok(Math.abs(momentumRate-(load-left-right))>load*.9,'opposite reactions fail physical impulse balance');
    close(result.fixedStationarity[f.layout.positions[0]],left,1e-12);close(result.fixedStationarity[f.layout.positions[2]],right,1e-12);
    // An existing load applied at a prescribed coordinate changes the TOTAL
    // boundary force, but not the direction or its reaction increment.
    const fixedLoad=.3;f.commonResidual[f.layout.positions[0]]-=fixedLoad;
    const loaded=run(f,create(f.layout,f.cluster));assert.equal(loaded.converged,true);
    close(loaded.fixedReactionIncrement[f.layout.positions[0]],left,1e-12);
    close(loaded.fixedStationarity[f.layout.positions[0]],left-fixedLoad,1e-12);
    close(momentumRate,load+fixedLoad+loaded.fixedStationarity[f.layout.positions[0]]+loaded.fixedStationarity[f.layout.positions[2]],1e-12);
});

test('fully prescribed incompatible rows retain their original residual and explicit row-specific gates',()=>{
    const f=fixture(),def={anchorNode:0,commonDofs:Int32Array.from([0,1]),relativeDofs:new Int32Array(),unit:'mm'};
    const rows=[{residual:2e-8,jacobian:[2,1],forceColumn:[-1,-.5],tolerance:1e-9}],w=create(f.layout,f.cluster,[def]);
    let result=run(f,w,rows);
    assert.equal(result.converged,false);assert.equal(result.heldMultiplierRows[0],1);assert.equal(result.multiplierIncrement[0],0);
    assert.equal(result.proof.constraints[0].residual,2e-8);assert.equal(result.proof.constraints[0].tolerance,1e-9);
    rows[0].residual=0;result=run(f,w,rows);assert.equal(result.converged,true);assert.equal(result.multiplierIncrement[0],0);
    // A caller-prepared open/release equation has an explicit dual derivative.
    rows[0].residual=.2;rows[0].jacobian=[0,0];rows[0].multiplierDerivative=-.5;
    result=run(f,w,rows);assert.equal(result.converged,true);assert.equal(result.heldMultiplierRows[0],0);close(result.multiplierIncrement[0],.4);
});

test('singular and stale/nonfinite inputs fail without regularization or a nonlinear certificate',()=>{
    const f=fixture(),w=create(f.layout,f.cluster);
    f.chain.hessianValid=false;assert.throws(()=>run(f,w),/fresh full common Hessian/);f.chain.hessianValid=true;
    f.chain.elementBackend='wasm-exact';assert.throws(()=>run(f,w),/Hessian kinds must agree/);f.chain.elementBackend='wasm';
    f.cluster.relative.hessian[0]=NaN;assert.throws(()=>run(f,w),/relative Hessian/);f.cluster.relative.hessian.fill(0);f.cluster.coupling.values.fill(0);
    f.relativeResidual[0]=1;
    const result=run(f,w);assert.equal(result.converged,false);assert.equal(result.factorizations,1);assert.equal(result.linearSolves,1);
    assert.equal(result.proof.relativeForce,Math.max(...f.relativeResidual.map(Math.abs)));assert.equal(result.certified,false);
    const defs=[{anchorNode:4,commonDofs:Int32Array.from([f.layout.positions[4]]),relativeDofs:Int32Array.from([0]),unit:'mm'}],v=create(f.layout,f.cluster,defs);
    assert.throws(()=>run(f,v,[{residual:0,jacobian:[1,1],tolerance:1e-9}]),/signed physical force column/);
    assert.throws(()=>run(f,v,[{residual:0,jacobian:[1,1],forceColumn:[1,1],tolerance:0}]),/positive/);
});
test('indexed coefficient scans reject nonfinite final entries and retries retain the independently solved original system',()=>{
    const f=fixture(),{definitions,rows}=localRows(f),w=create(f.layout,f.cluster,definitions);
    const original=denseOriginal(f,definitions,rows),expected=denseSolve(original,f.fixed);
    const checked=()=>{
        const result=run(f,w,rows);assert.equal(result.converged,true);
        vectorClose([...result.commonIncrement,...result.relativeIncrement,...result.multiplierIncrement],expected,2e-8);
        return [result.commonIncrement.slice(),result.relativeIncrement.slice(),result.multiplierIncrement.slice()];
    };
    const initial=checked();
    for(const field of ['jacobian','forceColumn','geometricTangent'])for(const bad of [NaN,Infinity,-Infinity]){
        const values=rows[0][field],last=values.length-1,saved=values[last];values[last]=bad;
        assert.throws(()=>run(f,w,rows),/finite entries/);values[last]=saved;assert.deepEqual(checked(),initial);
    }
});

test('unattainable original force/torque gates stay failed; each bounded correction refactors the same unshifted matrix',()=>{
    const f=fixture(),w=create(f.layout,f.cluster),originalSolve=w.lu.solve.bind(w.lu),calls=[];
    w.lu.solve=(matrix,rhs,scales,shift,direction)=>{calls.push({matrix:matrix.slice(),shift});return originalSolve(matrix,rhs,scales,shift,direction);};
    const result=solve(w,f.chain,{cluster:f.cluster,commonResidual:f.commonResidual,relativeResidual:f.relativeResidual,fixed:f.fixed,
        tolerances:{force:1e-30,torque:1e-30},maxCorrections:2});
    assert.equal(result.converged,false);assert.ok(result.proof.force>1e-30||result.proof.torque>1e-30);
    assert.equal(calls.length,3);assert.equal(result.factorizations,3);assert.equal(result.linearSolves,3);
    calls.forEach(call=>{assert.equal(call.shift,0);assert.deepEqual(call.matrix,calls[0].matrix);});
});

test('each correction RHS satisfies the independently contracted modified system while incompatible held rows remain visible',()=>{
    const f=fixture(),{definitions,rows}=localRows(f);
    definitions.push({anchorNode:0,commonDofs:Int32Array.from([0,1]),relativeDofs:new Int32Array(),unit:'mm'});
    rows.push({residual:2e-8,jacobian:[2,1],forceColumn:[-1,-.5],tolerance:1e-9});
    const w=create(f.layout,f.cluster,definitions),original=denseOriginal(f,definitions,rows);
    const native=w.lu.solve.bind(w.lu),calls=[];
    // Poison scratch from a previous solve; the first residual must be the
    // current equations at delta=0, not a retained operator response.
    w.increment.fill(71);w.linearResidual.fill(NaN);w.originalLinearResidual.fill(NaN);
    w.lu.solve=(matrix,rhs,scales,shift,direction)=>{
        const delta=w.increment.slice(),p=w.packedLayout;
        for(let i=0;i<w.count;i++) {
            let expected=w.residual[i];
            for(let j=p.starts[i];j<=p.ends[i];j++)expected+=matrix[p.offsets[i]+j]*delta[j];
            close(rhs[i],expected,1e-11);
            if(w.fixedMask[i])assert.equal(rhs[i],0);
        }
        assert.equal(rhs[w.dual.at(-1)],0,'held dual increment stays zero in the modified linear solve');
        calls.push(delta);return native(matrix,rhs,scales,shift,direction);
    };
    const result=solve(w,f.chain,{cluster:f.cluster,commonResidual:f.commonResidual,relativeResidual:f.relativeResidual,
        fixed:f.fixed,rows,tolerances:{force:1e-30,torque:1e-30},maxCorrections:2,eliminateZeroDuals:false});
    assert.equal(calls.length,3);assert.ok(calls[0].every(v=>v===0));
    assert.equal(result.converged,false);assert.equal(result.proof.constraints.at(-1).residual,2e-8);
    assert.equal(result.proof.constraints.at(-1).converged,false);
    const permutation=[...w.common,...w.relative,...w.dual],delta=allIncrement(result);
    original.A.forEach((row,i)=>{
        const expected=original.F[i]+row.reduce((sum,v,j)=>sum+v*delta[j],0);
        close(result.originalLinearResidual[permutation[i]],expected,1e-10);
    });
});

test('many modes and duals retain a bounded local band and linear matrix/factor storage',()=>{
    const sizes=[25,65,201],samples=sizes.map(count=>{
        const f=fixture({count,nodes:Array.from({length:count-4},(_,i)=>i+2),curved:false}),definitions=f.modes.map((m,i)=>({anchorNode:m.node,
            commonDofs:Int32Array.from([f.layout.positions[m.node]+1]),relativeDofs:Int32Array.from([2*i,2*i+1]),unit:'mm'}));
        const w=create(f.layout,f.cluster,definitions),rows=definitions.map(()=>({residual:.0001,jacobian:[1,2,1],forceColumn:[-1,-.5,-.5],tolerance:1e-10}));
        const result=run(f,w,rows);assert.equal(result.converged,true,JSON.stringify(result.proof));
        assert.ok(w.packedLayout.kl<=24);assert.ok(result.matrixEntries<49*result.count);assert.ok(result.factorEntries<75*result.count);
        assert.equal(result.maxRowsPerNode,1);return result;
    });
    assert.equal(samples[1].bandwidth,samples[2].bandwidth);
    assert.ok(samples[2].matrixEntries/samples[1].matrixEntries<3.3);assert.ok(samples[2].factorEntries/samples[1].factorEntries<3.3);
});

test('nonlocal or ambiguous supports are rejected instead of silently becoming a global dense solve',()=>{
    const f=fixture();
    assert.throws(()=>create(f.layout,f.cluster,[{anchorNode:0,commonDofs:Int32Array.from([0,f.layout.positions[7]]),relativeDofs:new Int32Array(),unit:'mm'}]),/two-edge/);
    assert.throws(()=>create(f.layout,f.cluster,[{anchorNode:4,commonDofs:[f.layout.positions[4]],relativeDofs:Int32Array.from([0]),unit:'mm'}]),/Int32Array/);
    assert.throws(()=>create(f.layout,f.cluster,[{anchorNode:4,commonDofs:Int32Array.from([20,20]),relativeDofs:new Int32Array(),unit:'mm'}]),/unique/);
    assert.throws(()=>create(f.layout,f.cluster,[{anchorNode:4,commonDofs:Int32Array.from([20]),relativeDofs:new Int32Array(),unit:''}]),/explicit unit/);
});

test('full 3D and mixed 2/3 coordinates interleave with local duals and match an independent dense direction',()=>{
    for(const dimensions of [[3,3,3],[2,3,2]])for(const elementBackend of ['wasm','wasm-exact']) {
        const f=fixture({dimensions,elementBackend}),definitions=f.cluster.modes.map(m=>({anchorNode:m.node,
            commonDofs:Int32Array.from([f.layout.positions[m.node]+1,f.layout.spins.get('wire')[m.node]]),relativeDofs:Int32Array.from(m.relativeDofs),unit:'mm'}));
        const rows=definitions.map((def,index)=>{
            const n=def.commonDofs.length+def.relativeDofs.length,tangent=new Float64Array(n*n);tangent[1]=.03;tangent[n]=-.01;
            return {residual:.003*(index+1),jacobian:Float64Array.from({length:n},(_,i)=>.2+Math.sin(i+.3)),
                forceColumn:Float64Array.from({length:n},(_,i)=>-.4+.3*Math.cos(i+.6)),multiplierDerivative:-.2,
                geometricTangent:tangent,tolerance:1e-11};
        });
        const w=create(f.layout,f.cluster,definitions),original=denseOriginal(f,definitions,rows),expected=denseSolve(original,f.fixed),result=run(f,w,rows);
        assert.equal(result.converged,true,JSON.stringify(result.proof));vectorClose(allIncrement(result),expected,3e-9);
        assert.equal(result.relativeIncrement.length,dimensions.reduce((s,d)=>s+d,0));
        let offset=0;dimensions.forEach((dim,i)=>{assert.equal(w.structure.modeOffsets[i],offset);offset+=dim;});
        assert.equal(w.structure.modeOffsets.at(-1),offset);assert.equal(result.fullRankOnRepresentedNodes,dimensions.every(d=>d===3));
        assert.equal(result.certified,false);assert.equal(result.nonlinearStepAccepted,false);
        const permutation=[...w.common,...w.relative,...w.dual];
        original.A.forEach((row,i)=>row.forEach((v,j)=>{const a=permutation[i],b=permutation[j];
            close(v,b<w.packedLayout.starts[a]||b>w.packedLayout.ends[a]?0:w.originalMatrix[w.packedLayout.offsets[a]+b],2e-13);
        }));
    }
});

test('direction workspace accepts structure-only metadata but rejects an unassembled operator and changed mode dimensions',()=>{
    const f=fixture({dimensions:[3,2,3]}),current=f.cluster;
    const structure=createCompositeRelativeClusterStructure({data:f.data,layout:f.layout,modes:f.modes,inertia:f.inertia});
    const w=create(f.layout,structure);assert.equal(structure.hessianValid,false);
    f.cluster=structure;assert.throws(()=>run(f,w),/fresh cluster/);
    f.cluster=current;assert.equal(run(f,w).converged,true);
    f.cluster=structuredClone(current);f.cluster.modes[0].basis.pop();assert.throws(()=>run(f,w),/frozen transverse bases/);
    f.cluster=structuredClone(current);f.cluster.modes[1].relativeDofs[0]--;assert.throws(()=>run(f,w),/frozen transverse bases/);
    const bad=structuredClone(current);bad.relative.dofCount--;assert.throws(()=>create(f.layout,bad),/sum of mode dimensions/);
});

test('many full three-coordinate modes keep bounded band and linear storage with one shared solve',()=>{
    const samples=[25,65,201].map(count=>{
        const nodes=Array.from({length:count-4},(_,i)=>i+2),f=fixture({count,nodes,dimensions:nodes.map(()=>3),curved:false});
        const definitions=f.cluster.modes.map(m=>({anchorNode:m.node,commonDofs:Int32Array.from([f.layout.positions[m.node]+1]),
            relativeDofs:Int32Array.from(m.relativeDofs),unit:'mm'}));
        const rows=definitions.map(()=>({residual:.0001,jacobian:[1,2,1,.5],forceColumn:[-1,-.5,-.5,-.25],tolerance:1e-10}));
        const w=create(f.layout,f.cluster,definitions),result=run(f,w,rows);
        assert.equal(result.converged,true,JSON.stringify(result.proof));assert.equal(result.fullRankOnRepresentedNodes,true);
        assert.ok(result.bandwidth<=28);assert.ok(result.matrixEntries<57*result.count);assert.ok(result.factorEntries<85*result.count);
        assert.equal(result.factorizations,1);return result;
    });
    assert.equal(samples[1].bandwidth,samples[2].bandwidth);
    assert.ok(samples[2].matrixEntries/samples[1].matrixEntries<3.3);assert.ok(samples[2].factorEntries/samples[1].factorEntries<3.3);
});

test('global endpoint modes and endpoint-local physical boundary rows match an independent dense system',()=>{
    for(const materialEnds of [false,true])for(const elementBackend of ['wasm','wasm-exact']) {
        const nodes=materialEnds?[1,4]:[0,6],edgeToolIds=materialEnds?Array.from({length:6},(_,e)=>e>=1&&e<4?['wire','catheter']:['catheter']):null;
        const f=fixture({count:7,nodes,dimensions:[3,3],edgeToolIds,elementBackend});
        nodes.forEach(node=>f.fixed.fill(1,f.layout.positions[node],f.layout.positions[node]+3));
        f.fixed[f.layout.spins.get('wire').find(d=>d>=0)]=1;
        const definitions=f.cluster.modes.map(m=>({anchorNode:m.node,
            commonDofs:Int32Array.from([f.layout.positions[m.node],f.layout.positions[m.node]+1,f.layout.positions[m.node]+2]),
            relativeDofs:Int32Array.from(m.relativeDofs),unit:'mm'}));
        const rows=definitions.map((def,index)=>({residual:.003*(index+1),jacobian:[1,.2,-.1,.3,-.7,.5],
            forceColumn:[-.5,.1,-.3,-.2,.8,.4],multiplierDerivative:-.1,tolerance:1e-11}));
        const original=denseOriginal(f,definitions,rows),expected=denseSolve(original,f.fixed),w=create(f.layout,f.cluster,definitions),r=run(f,w,rows);
        assert.equal(r.converged,true,JSON.stringify(r.proof));vectorClose(allIncrement(r),expected,3e-9);
        assert.ok(r.relativeIncrement.slice(0,3).some(v=>Math.abs(v)>1e-5));assert.ok(r.relativeIncrement.slice(3).some(v=>Math.abs(v)>1e-5));
        for(const dof of definitions.flatMap(d=>Array.from(d.commonDofs)))assert.equal(r.commonIncrement[dof],0);
        assert.ok(r.bandwidth<25,'distant endpoint modes do not turn the local solve into a global dense block');
    }
});

test('a held catheter permits free axial wire slip and an independent wire endpoint target while both tool lengths stay exact',()=>{
    const count=5,nodes=Array.from({length:count},(_,i)=>i),f=fixture({count,nodes,dimensions:nodes.map(()=>3),curved:false});
    const dt=.25,density=2.4,tipLoad=.2,feed=.01;
    f.data.tools.forEach(tool=>{tool.angles.fill(0);tool.dsDx=1;tool.material=compileCompositeMaterial({EI1:2,EI2:3,GJ:4});});
    f.inertia.dt=dt;f.inertia.previousPositions=f.data.positions.map(p=>p.slice());
    f.inertia.inertiaEdges.forEach((edge,e)=>edge.tools.forEach(tool=>{tool.materialMap={sStart:2*e,dsDx:1,dsDt:[0,0]};tool.oldMaterialVelocities=[[0,0,0],[0,0,0]];}));
    assembleCompositeChain(f.data,f.chain);
    f.inertia.inertiaEdges.forEach((edge,e)=>scatterCompositeTranslationalInertia(assembleCompositeTranslationalInertia({
        coordinates:f.data.coordinates.slice(e,e+2),positions:f.data.positions.slice(e,e+2),previousPositions:f.inertia.previousPositions.slice(e,e+2),dt,tools:edge.tools}),e,f.chain));
    f.cluster=assembleCompositeRelativeCluster({data:f.data,layout:f.layout,modes:f.modes,inertia:f.inertia});
    f.fixed.fill(0);f.layout.positions.forEach(d=>f.fixed.fill(1,d,d+3));for(const spins of f.layout.spins.values())f.fixed[spins[0]]=1;
    f.commonResidual=f.chain.gradient.slice();f.relativeResidual=f.cluster.relative.gradient.slice();
    assert.ok(f.commonResidual.every(v=>v===0)&&f.relativeResidual.every(v=>v===0));
    const lengths=createCompositeToolLengthWorkspace({layout:f.layout,modes:f.cluster.modes});
    const restLengths=new Map(f.data.tools.map(tool=>[tool.id,new Float64Array(count-1).fill(2)]));
    const toolPositions=new Map(f.data.tools.map(tool=>[tool.id,f.data.positions.map(p=>p.slice())]));
    evaluateCompositeToolLengths({toolPositions,restLengths,multipliers:new Float64Array(lengths.rows.length),tolerance:1e-10},lengths);
    // Frozen labels and u=0: axial motion is represented by physical material
    // displacement here, not applied a second time as through-mesh advection.
    const tip=f.cluster.modes.at(-1);f.commonResidual[f.layout.positions.at(-1)]-=tipLoad;
    tip.relativeDofs.forEach((d,j)=>f.relativeResidual[d]-=tip.basis[j][0]*tipLoad);
    const free=run(f,create(f.layout,f.cluster,lengths.rows),lengths.rows),mass=density*2*(count-1),freeDisplacement=tipLoad*dt*dt/mass;
    assert.equal(free.converged,true,JSON.stringify(free.proof));
    const physicalIncrement=(result,mode)=>[0,1,2].map(k=>result.commonIncrement[f.layout.positions[mode.node]+k]+mode.basis.reduce((sum,b,j)=>sum+b[k]*result.relativeIncrement[mode.relativeDofs[j]],0));
    f.cluster.modes.forEach(mode=>vectorClose(physicalIncrement(free,mode),[freeDisplacement,0,0],2e-10));
    assert.ok(freeDisplacement>0);assert.ok(Math.max(...free.fixedStationarity.map(Math.abs))<1e-9,'no artificial wire/catheter drag at the held common coordinates');
    // Prescribe the physical wire endpoint y=q+B*rho, while the catheter q
    // stays fixed. These are three local rows, not a fixed rho=0 attachment.
    f.commonResidual.fill(0);f.relativeResidual.fill(0);
    const base=f.cluster.modes[0],baseDofs=Int32Array.from([0,1,2]),relativeDofs=Int32Array.from(base.relativeDofs);
    const boundary=[0,1,2].map(axis=>{
        const jacobian=[...Array.from({length:3},(_,k)=>k===axis?1:0),...base.basis.map(b=>b[axis])];
        return {anchorNode:0,commonDofs:baseDofs,relativeDofs,unit:'mm',residual:axis===0?-feed:0,
            jacobian,forceColumn:jacobian.map(v=>-v),tolerance:1e-11};
    });
    const rows=[...lengths.rows,...boundary],prescribed=run(f,create(f.layout,f.cluster,rows),rows);
    assert.equal(prescribed.converged,true,JSON.stringify(prescribed.proof));
    f.cluster.modes.forEach(mode=>vectorClose(physicalIncrement(prescribed,mode),[feed,0,0],2e-10));
    f.layout.positions.forEach(d=>{assert.equal(prescribed.commonIncrement[d],0);assert.equal(prescribed.commonIncrement[d+1],0);assert.equal(prescribed.commonIncrement[d+2],0);});
    close(prescribed.multiplierIncrement[lengths.rows.length],mass*feed/(dt*dt),2e-10);
    assert.ok(Math.max(...prescribed.fixedStationarity.map(Math.abs))<1e-9);
    for(const mode of f.cluster.modes)toolPositions.get('wire')[mode.node]=f.data.positions[mode.node].map((v,k)=>v+physicalIncrement(prescribed,mode)[k]);
    const proof=evaluateCompositeToolLengths({toolPositions,restLengths,multipliers:new Float64Array(lengths.rows.length),tolerance:1e-10},lengths);
    assert.ok(proof.maximumResidual<1e-12,'both original finite material edge lengths remain exact after the rigid axial wire displacement');
    assert.equal(prescribed.nonlinearStepAccepted,false,'this integration control still does not accept a nonlinear dt');
});
