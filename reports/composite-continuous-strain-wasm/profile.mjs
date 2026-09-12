import path from 'node:path';
import {pathToFileURL} from 'node:url';
import assert from 'node:assert/strict';
const root=process.cwd();
async function api(directory) {
    const names=['Chain','Element','JointTimeStep','JointMaterialHistory'],all={};
    for(const name of names)Object.assign(all,await import(pathToFileURL(path.join(directory,'src/physics/kirchhoffComposite'+name+'.js')).href));
    return all;
}
const dt=1/120,ids=['wire','catheter'];
function setup(a,count,fixedNodes) {
    const height=count===3?.12:0,arc=height===0?1:.5*Math.sqrt(1+4*height*height)+Math.asinh(2*height)/(4*height),
        coordinates=Array.from({length:count},(_,j)=>arc*j),positions=coordinates.map((_,j)=>[j,height*j*(2-j),0]),
        toolPositions=id=>positions.map(p=>p.map((v,k)=>v+(id==='wire'&&k===1?.03:0))),
        tools=ids.map(id=>{const material=a.compileCompositeMaterial({EI1:id==='wire'?2:9,EI2:id==='wire'?3:12,GJ:id==='wire'?1:4});
            return {id,dsDx:1,reference:a.captureCompositeReferenceFrames(toolPositions(id),[0,0,1]),referenceTwists:new Float64Array(count-2),material,
                ...(height===0?{}:{materialAt:({coordinate})=>({...material,intrinsic:[-2*height/(1+4*height*height*(1-coordinate/arc)**2)/arc,0,0]})})};}),
        layout=a.createCompositeChainLayout(coordinates.slice(1).map(()=>ids)),modes=coordinates.map((_,node)=>({node,basis:[[1,0,0],[0,1,0],[0,0,1]]})),
        state=a.createCompositeJointTimeStepState({layout,coordinates,positions,tools,modes,relative:modes.flatMap(()=>[0,.03,0]),
            angles:new Map(ids.map(id=>[id,new Float64Array(count-1)])),restLengths:new Map(ids.map(id=>[id,new Float64Array(count-1).fill(arc)])),
            inertiaGeometryByTool:new Map(ids.map(id=>[id,{}])),elasticityGeometry:'continuous-material-frame',lengthGeometry:'continuous-arclength',materialCoordinate:'reference-arclength'}),
        workspace=a.createCompositeJointTimeStepWorkspace(state);
    function inertiaFor(current) {
        const inertia={dt,previousPositions:structuredClone(current.toolPositions),inertiaEdges:current.layout.edgeToolIds.map((ids,e)=>({tools:ids.map(id=>({id,
            massPerMaterialLength:id==='wire'?.13:.24,materialMap:{sStart:20+current.coordinates[e],dsDx:1,dsDt:0},oldMaterialVelocities:[[0,0,0],[0,0,0]]}))}))};
        if(current.step>0) {
            const h=a.createCompositeJointMaterialHistory({materialVelocities:current.materialVelocities}),prepared=h.prepare({coordinates:current.coordinates,inertiaEdges:inertia.inertiaEdges});
            for(let e=0;e<inertia.inertiaEdges.length;e++)for(const t of inertia.inertiaEdges[e].tools){delete t.oldMaterialVelocities;t.oldVelocityPieces=prepared.inertiaEdges[e].tools.find(v=>v.id===t.id).pieces;}
        }
        return inertia;
    }
    const input={dt,torsionMode:'quasi-static',contacts:'none',workspace,inertia:inertiaFor(state),
        boundaries:{positions:ids.flatMap(toolId=>fixedNodes.map(node=>({toolId,node,value:state.toolPositions.get(toolId)[node]}))),spins:ids.map(toolId=>({toolId,edge:0,value:0}))},
        loads:{forces:[{toolId:'wire',node:count-1,value:[0,.004,.002]},{toolId:'catheter',node:count-1,value:[0,-.003,-.002]}]}};
    return {state,input,workspace,inertiaFor,run:(current=state,own=input)=>a.advanceCompositeJointTimeStep(current,own)};
}

const a=await api(root),p=setup(a,6,[0,1]);
for(let j=0;j<240;j++){const r=p.run();assert.equal(r.accepted,true);}
console.log('240 accepted current joint steps');
