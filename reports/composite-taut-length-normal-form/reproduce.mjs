import fs from 'node:fs';
import path from 'node:path';
import os from 'node:os';
import assert from 'node:assert/strict';
import {pathToFileURL} from 'node:url';
import * as chain from '../../src/physics/kirchhoffCompositeChain.js';
import * as element from '../../src/physics/kirchhoffCompositeElement.js';
import * as current from '../../src/physics/kirchhoffCompositeJointTimeStep.js';
import {createCompositeJointMaterialHistory} from '../../src/physics/kirchhoffCompositeJointMaterialHistory.js';

const root=process.cwd(),temporary=fs.mkdtempSync(path.join(os.tmpdir(),'oet-taut-before-')),dt=1/120,ids=['wire','catheter'];
function fixture(api) {
    const coordinates=[0,1,2,3,4,5],positions=coordinates.map(x=>[x,0,0]),wire=coordinates.map(x=>[x,.03,0]),
        layout=chain.createCompositeChainLayout(coordinates.slice(1).map(()=>ids)),modes=coordinates.map((_,node)=>({node,basis:[[1,0,0],[0,1,0],[0,0,1]]})),
        tools=ids.map(id=>({id,dsDx:1,reference:element.captureCompositeReferenceFrames(id==='wire'?wire:positions,[0,0,1]),referenceTwists:new Float64Array(4),
            material:element.compileCompositeMaterial({EI1:id==='wire'?2:9,EI2:id==='wire'?3:12,GJ:id==='wire'?1:4})})),
        state=api.createCompositeJointTimeStepState({layout,coordinates,positions,tools,modes,relative:modes.flatMap(()=>[0,.03,0]),
            angles:new Map(ids.map(id=>[id,new Float64Array(5)])),restLengths:new Map(ids.map(id=>[id,new Float64Array(5).fill(1)])),
            inertiaGeometryByTool:new Map(ids.map(id=>[id,{}])),elasticityGeometry:'continuous-material-frame',lengthGeometry:'continuous-arclength',materialCoordinate:'reference-arclength'}),
        workspace=api.createCompositeJointTimeStepWorkspace(state),input={dt,torsionMode:'quasi-static',contacts:'none',workspace,inertia:inertiaFor(state),
            boundaries:{positions:ids.flatMap(toolId=>[0,1].map(node=>({toolId,node,value:state.toolPositions.get(toolId)[node]}))),spins:ids.map(toolId=>({toolId,edge:0,value:0}))},
            loads:{forces:[{toolId:'wire',node:5,value:[0,.004,.002]},{toolId:'catheter',node:5,value:[0,-.003,-.002]}]}};
    return {state,input};
}
function inertiaFor(state) {
    return {dt,previousPositions:structuredClone(state.toolPositions),inertiaEdges:state.layout.edgeToolIds.map((ids,e)=>({tools:ids.map(id=>({id,
        massPerMaterialLength:id==='wire'?.13:.24,materialMap:{sStart:20+state.coordinates[e],dsDx:1,dsDt:0},oldMaterialVelocities:[[0,0,0],[0,0,0]]}))}))};
}
const describe=r=>({accepted:r.accepted,status:r.status,step:r.state.step,time:r.state.time,diagnostics:r.diagnostics,
    balances:r.balances?[...r.balances]:null,tautLengthState:r.state.tautLengthState??null});
try {
    const file=path.join(temporary,'before-joint.mjs'),source=fs.readFileSync(new URL('./before-joint.js',import.meta.url),'utf8');
    fs.writeFileSync(file,source.replaceAll("from './","from '"+pathToFileURL(path.join(root,'src/physics')+'/').href));
    const previous=await import(pathToFileURL(file).href),a=fixture(previous),b=fixture(current),beforeSnapshot=structuredClone(a.state),afterSnapshot=structuredClone(b.state),
        before=previous.advanceCompositeJointTimeStep(a.state,a.input),after=current.advanceCompositeJointTimeStep(b.state,b.input);
    assert.equal(before.accepted,false);assert.equal(after.accepted,true);assert.deepEqual(a.state,beforeSnapshot);assert.deepEqual(b.state,afterSnapshot);
    const inertia=inertiaFor(after.state),history=createCompositeJointMaterialHistory({materialVelocities:after.state.materialVelocities}),
        prepared=history.prepare({coordinates:after.state.coordinates,inertiaEdges:inertia.inertiaEdges});
    for(let e=0;e<inertia.inertiaEdges.length;e++)for(const t of inertia.inertiaEdges[e].tools){delete t.oldMaterialVelocities;t.oldVelocityPieces=prepared.inertiaEdges[e].tools.find(v=>v.id===t.id).pieces;}
    const input={...b.input,inertia},snapshot=structuredClone(after.state),rejected=current.advanceCompositeJointTimeStep(after.state,{...input,budget:{directions:0}}),
        next=current.advanceCompositeJointTimeStep(after.state,input),cold=current.advanceCompositeJointTimeStep(after.state,{...input,workspace:null});
    assert.equal(rejected.accepted,false);assert.equal(next.accepted,true);assert.equal(cold.accepted,true);assert.deepEqual(next.state,cold.state);assert.deepEqual(next.boundaryForces,cold.boundaryForces);assert.deepEqual(after.state,snapshot);
    const result={scope:'old joint equations vs regular taut normal form on the same current material/inertia kernels; no timing or FPS claim',
        before:describe(before),after:describe(after),second:describe(next),inputUnchanged:true,retryMatchesCold:true};
    fs.writeFileSync('/tmp/oet-taut-length-reproduction.json',JSON.stringify(result,null,2)+'\n');
    console.log(JSON.stringify({before:before.status,after:after.status,second:next.status,tautRows:after.diagnostics.tautLengthRows,originalLengthError:after.diagnostics.certificate.length,forceError:after.diagnostics.certificate.force,retryMatchesCold:true}));
} finally {fs.rmSync(temporary,{recursive:true,force:true});}
