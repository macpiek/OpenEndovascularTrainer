import {createCompositeChainLayout} from '../../src/physics/kirchhoffCompositeChain.js';
import {captureCompositeReferenceFrames,compileCompositeMaterial} from '../../src/physics/kirchhoffCompositeElement.js';
import {createCompositeJointTimeStepState} from '../../src/physics/kirchhoffCompositeJointTimeStep.js';

// Synthetic material calibration; this is a whole-step mechanics/CPU fixture,
// not anatomy or a measured hydrophilic guidewire profile.
export function relativeSearchFixture({n=17,curved=false,lumen=false,offset=.03,wireSlope=0}={}) {
    if(curved&&wireSlope!==0)throw new RangeError('This fixture declares constant material slopes only');
    const positions=Array.from({length:n},(_,i)=>[2*i,curved?.04*Math.sin(i*.17):0,curved?.03*Math.cos(i*.13):0]);
    const coordinates=[0];for(let i=1;i<n;i++)coordinates.push(coordinates[i-1]+Math.hypot(...positions[i].map((v,k)=>v-positions[i-1][k])));
    const wire=positions.map(p=>[p[0]+.125,p[1]+offset+wireSlope*p[0],p[2]]),ids=['wire','catheter'],wireDsDx=Math.hypot(1,wireSlope);
    const layout=createCompositeChainLayout(Array.from({length:n-1},()=>ids));
    const modes=positions.map((_,node)=>({node,basis:[[1,0,0],[0,1,0],[0,0,1]]}));
    const angles=new Map(ids.map(id=>[id,new Float64Array(n-1)])),restLengths=new Map(ids.map(id=>[id,Float64Array.from(coordinates.slice(1),(x,i)=>(x-coordinates[i])*(id==='wire'?wireDsDx:1))]));
    const tools=ids.map(id=>({id,dsDx:id==='wire'?wireDsDx:1,reference:captureCompositeReferenceFrames(id==='wire'?wire:positions),referenceTwists:new Float64Array(n-2),
        material:compileCompositeMaterial(id==='wire'?{EI1:2,EI2:3,GJ:1}:{EI1:8,EI2:11,GJ:4})}));
    const state=createCompositeJointTimeStepState({layout,coordinates,positions,relative:wire.flatMap((p,i)=>p.map((v,k)=>v-positions[i][k])),
        modes,angles,tools,restLengths,materialCoordinate:'reference-arclength'});
    const contacts=lumen?{mode:'lumen-coulomb',chartId:'relative-search-lumen',forcePerLength:1,
        friction:{law:'coulomb',mu:[.3,.6],forcePerLength:5,materialPath:'linear-affine-maps'},
        pairs:Array.from({length:n-1},(_,edge)=>({id:`pair:${edge}`,innerToolId:'wire',outerToolId:'catheter',innerEdge:edge,outerEdge:edge,
            innerMaterialSegmentId:`wire:${edge}`,outerMaterialSegmentId:`catheter:${edge}`,lumenRadius:.4845,innerRadius:.444,
            quadrature:[.25,.75],openDistal:false,portalFilletRadius:0}))}:'none';
    return {state,contacts};
}

export function relativeSearchOptions(f,state,{relativeReduction='none',force=[.01,0,0],forceNode=state.layout.nodeCount-1,feed=0,spin=0,workspace,budget}={}) {
    return {dt:1/120,torsionMode:'quasi-static',contacts:f.contacts,relativeReduction,workspace,budget,
        inertia:{previousPositions:structuredClone(state.toolPositions),inertiaEdges:state.layout.edgeToolIds.map((ids,edge)=>({tools:ids.map(id=>({id,
            massPerMaterialLength:id==='wire'?.13:.24,
            materialMap:{sStart:20+state.coordinates[edge]*state.tools.find(t=>t.id===id).dsDx-(id==='wire'?feed*state.time:0),
                dsDx:state.tools.find(t=>t.id===id).dsDx,dsDt:id==='wire'?-feed:0},
            oldMaterialVelocities:(state.materialVelocities?.[edge].tools.find(t=>t.id===id).velocities??[[0,0,0],[0,0,0]]).map(v=>Array.from(v))}))}))},
        boundaries:{positions:[],spins:[{toolId:'wire',edge:0,value:spin},{toolId:'catheter',edge:0,value:-.4*spin}]},
        loads:{forces:[{toolId:'wire',node:forceNode,value:force}]}};
}
