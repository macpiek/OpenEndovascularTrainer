import assert from 'node:assert/strict';
import test from 'node:test';
import {BufferGeometry,Float32BufferAttribute,Vector3} from 'three';
import {MeshBVH} from 'three-mesh-bvh';
import {createContactResult} from '../src/physics/collision/vesselContactField.js';
import {createCompositeChainLayout} from '../src/physics/kirchhoffCompositeChain.js';
import {captureCompositeReferenceFrames} from '../src/physics/kirchhoffCompositeElement.js';
import {createCompositeContinuousGeometry} from '../src/physics/kirchhoffCompositeContinuousGeometry.js';
import {compositeContinuousFrameSupport} from '../src/physics/kirchhoffCompositeContinuousFrame.js';
import {createCompositeJointContinuousWallRows} from '../src/physics/kirchhoffCompositeJointContinuousWallRows.js';
import {createCompositeJointContinuousWallFrictionRows} from '../src/physics/kirchhoffCompositeJointContinuousWallFrictionRows.js';

const ids=['wire','catheter'],dot=(a,b)=>a.reduce((sum,v,k)=>sum+v*b[k],0),
    close=(a,b,t=2e-7)=>assert.ok(Number.isFinite(a)&&Number.isFinite(b)&&Math.abs(a-b)<=t,`${a} != ${b} (${t})`);
function fixture(negativePressure) {
    const mesh=new BufferGeometry();mesh.setAttribute('position',new Float32BufferAttribute([[0,0,0],[2,0,0],[0,2,0]].flat(),3));mesh.boundsTree=new MeshBVH(mesh);
    const probe=new Vector3(),target={point:new Vector3(),distance:Infinity,faceIndex:-1},field={fallbackGeometry:mesh,calls:0};
    field.querySphere=(position,radius,out=createContactResult())=>{
        field.calls++;mesh.boundsTree.closestPointToPoint(probe.fromArray(position),target);
        out.source='sparse-sdf-bvh';out.faceIndex=target.faceIndex;out.signedDistance=target.distance;out.signedGap=target.distance-radius;
        out.closestPoint.values.set(target.point.toArray());out.inward.values.set(position.map((v,k)=>(v-target.point.getComponent(k))/target.distance));return out;
    };
    const coordinates=[0,1,2,3],geometry=createCompositeContinuousGeometry({coordinates}),geometryByTool=new Map(ids.map(id=>[id,geometry])),
        materialSupports=ids.flatMap(toolId=>geometry.edges.map((_,edge)=>{const support=compositeContinuousFrameSupport(geometry,edge);
            return {toolId,edge,positionNodes:support.positionIndices,angleEdges:support.angleIndices};})),
        layout=createCompositeChainLayout(coordinates.slice(1).map(()=>ids),{positionSupports:geometry.edges.map(e=>e.nodeIndices),materialSupports}),
        basis=[[Math.SQRT1_2,Math.SQRT1_2,0],[-Math.SQRT1_2,Math.SQRT1_2,0],[0,0,1]],
        modes=coordinates.map((_,node)=>({node,basis:basis.map(b=>b.slice()),relativeDofs:[3*node,3*node+1,3*node+2]})),
        common=coordinates.map(i=>[.5+.2*i,-.4+.025*Math.sin(i),.7+.04*Math.cos(i)]),offset=[.013,.035,.125],
        oldWire=common.map(p=>p.map((v,k)=>v+offset[k])),oldToolPositions=new Map([['wire',oldWire],['catheter',common.map(p=>p.slice())]]),
        oldAngles=new Map(ids.map(id=>[id,coordinates.slice(1).map((_,j)=>(id==='wire'?.03:-.04)*j)])),
        state={layout,coordinates,modes,relativeToolId:'wire',elasticityGeometry:'continuous-material-frame',inertiaGeometryByTool:geometryByTool,
            toolPositions:oldToolPositions,angles:oldAngles,tools:ids.map(id=>({id,reference:captureCompositeReferenceFrames(oldToolPositions.get(id)),referenceTwists:[0,0]}))},
        candidate={positions:common.map((p,j)=>p.map((v,k)=>v+.001*(j+1)*(k+1))),relative:Float64Array.from(modes.flatMap((m,j)=>m.basis.map((b,a)=>dot(b,offset)+.0003*(j+1)*(a+1)))),
            modes,angles:new Map(ids.map(id=>[id,oldAngles.get(id).map((v,j)=>v+(id==='wire'?.013:-.017)*(j+1))])),inertiaGeometryByTool:geometryByTool},
        prepared={inertiaEdges:coordinates.slice(1).map((_,edge)=>({tools:ids.map(id=>({id,materialMap:{sStart:(id==='wire'?20:40)+edge,dsDx:id==='wire'?1.1:.9,
            dsDt:id==='wire'?[-.3,-.15]:[.18,.07]}}))}))},
        sites=ids.map(owner=>({id:`${owner}:1:.37`,owner,edge:1,fraction:.37,coordinate:1.37,radius:1})),
        normalWall={mode:'wall-normal',friction:'none',contactMode:'continuous-samples',pressureDiscretization:'original-resolution-C2-material-samples',
            chartId:'actual-triangle-edge-derivative-test',field,forcePerLength:3,pressureSites:sites},
        wall={...normalWall,mode:'wall-coulomb',friction:{law:'coulomb-static-kinetic',rateMode:'backward-euler-grid',slipModel:'implicit-backward-euler-surface-rate',finiteStepSlipKnown:false,
            forcePerLength:2.3,muByOwner:ids.map(owner=>({owner,muStatic:[.17,.17],muKinetic:[.17,.17]}))}},
        tolerances={force:1e-7,wallGap:1e-8,wallNcp:1e-8,wallWork:1e-9,linearConstraint:1e-10,frictionSlip:1e-8,frictionCone:1e-8,frictionWork:1e-8,frictionEquation:1e-8},
        normal=createCompositeJointContinuousWallRows({layout,coordinates,modes,wall:normalWall,geometryByTool,tolerances}),
        friction=createCompositeJointContinuousWallFrictionRows({state,candidate,prepared,normal,wall,dt:.02,tolerances,normalRowOffset:0,frictionRowOffset:2});
    normal.normalForces.set([1.3,negativePressure?-.7:.9]);friction.tractions.set([.6,-.4,-.7,.5]);
    const commonCount=layout.dofCount,relativeCount=candidate.relative.length,Q=commonCount+relativeCount,rows=[...normal.rows,...friction.rows],M=Q+rows.length;
    function sync() {
        return new Map(ids.map(id=>[id,candidate.positions.map((p,node)=>p.map((v,k)=>v+(id==='wire'?modes[node].basis.reduce((sum,b,a)=>sum+b[k]*candidate.relative[3*node+a],0):0)))]));
    }
    const initial=sync();normal.refresh({toolPositions:initial,commonResidual:new Float64Array(commonCount),relativeResidual:new Float64Array(relativeCount),order:'full'});friction.prepare({toolPositions:initial});
    function evaluate(order='gradient') {
        const toolPositions=sync(),cg=new Float64Array(commonCount),rg=new Float64Array(relativeCount);
        normal.refresh({toolPositions,commonResidual:cg,relativeResidual:rg,order});
        const proof=friction.refresh({toolPositions,commonResidual:cg,relativeResidual:rg,order}),residual=Float64Array.from([...cg,...rg,...rows.map(r=>r.residual)]),H=order==='full'?new Float64Array(M*M):null;
        if(H)rows.forEach((row,r)=>{
            const dofs=[...row.commonDofs,...Array.from(row.relativeDofs,d=>commonCount+d)],n=dofs.length;
            dofs.forEach((j,a)=>{
                H[(Q+r)*M+j]+=row.jacobian[a];H[j*M+Q+r]+=row.forceColumn[a];
                dofs.forEach((k,b)=>H[j*M+k]+=row.geometricTangent[a*n+b]);
            });
            H[(Q+r)*M+Q+r]+=row.multiplierDerivative;
            row.multiplierDofs?.forEach((d,j)=>H[(Q+r)*M+Q+d]+=row.multiplierJacobian[j]);
        });
        return {residual,H,proof,toolPositions};
    }
    function perturb(j,h) {
        if(j>=Q){const m=j-Q;if(m<2)normal.normalForces[m]+=h;else friction.tractions[m-2]+=h;return;}
        if(j>=commonCount){candidate.relative[j-commonCount]+=h;return;}
        for(let node=0;node<4;node++)if(j>=layout.positions[node]&&j<layout.positions[node]+3){candidate.positions[node][j-layout.positions[node]]+=h;return;}
        for(const [id,spins] of layout.spins){const edge=spins.indexOf(j);if(edge>=0){candidate.angles.get(id)[edge]+=h;return;}}
        throw Error('Unknown full joint column');
    }
    return {mesh,field,state,candidate,normal,friction,rows,M,Q,commonCount,evaluate,perturb};
}

for(const negativePressure of [false,true])test(`complete C2 wall contact block matches common/relative/angle/normal/traction finite differences${negativePressure?' with signed negative private pressure':''}`,context=>{
    const f=fixture(negativePressure);try {
        const base=f.evaluate('full'),h=1e-6;let maxEquilibrium=0,maxConstraint=0;
        assert.equal(f.Q,30);assert.equal(f.M,36);assert.equal(f.rows.length,6);
        assert.ok(f.normal.samples.every(s=>s.point.supported));
        assert.ok(f.normal.samples.every(s=>s.point.pointNormalDerivative.some(v=>Math.abs(v)>1e-3)),'Original triangle edge must produce changing normals');
        assert.ok(f.friction.rows.every(r=>r.relativeDofs.length===(r.toolId==='wire'?12:0)));
        assert.ok(f.friction.rows.every(r=>r.constraintSupport.positionNodes.length===4&&r.constraintSupport.angleEdges.length===3));
        assert.ok(base.proof.samples.every(s=>s.slip.every(Number.isFinite)&&s.finiteStepSlipKnown===false));
        for(let j=0;j<f.M;j++) {
            f.perturb(j,h);const plus=f.evaluate();f.perturb(j,-2*h);const minus=f.evaluate();f.perturb(j,h);
            for(let i=0;i<f.M;i++) {
                const fd=(plus.residual[i]-minus.residual[i])/(2*h),actual=base.H[i*f.M+j],error=Math.abs(fd-actual);
                if(i<f.Q)maxEquilibrium=Math.max(maxEquilibrium,error);else maxConstraint=Math.max(maxConstraint,error);
                assert.ok(Number.isFinite(fd)&&error<3e-7,`full wall block row ${i}, column ${j}: FD ${fd}, analytic ${actual}, error ${error}`);
            }
        }
        // Force/torque storage must be the negative of equilibrium residuals,
        // with the rotated relative basis applied to the wire alone.
        const restored=f.evaluate('full');
        for(let node=0;node<4;node++)for(let k=0;k<3;k++) {
            const force=ids.reduce((sum,id)=>sum+f.normal.nodalForces.get(id)[node][k]+f.friction.nodalForces.get(id)[node][k],0);
            close(restored.residual[f.state.layout.positions[node]+k],-force,3e-13);
        }
        for(const [id,spins] of f.state.layout.spins)spins.forEach((d,edge)=>close(restored.residual[d],-f.friction.spinTorques.get(id)[edge],3e-13));
        const wireRows=f.friction.rows.filter(r=>r.toolId==='wire');
        assert.ok(wireRows.some(r=>r.forceColumn.slice(r.commonDofs.length).some(v=>Math.abs(v)>1e-5)));
        if(negativePressure)assert.equal(restored.proof.converged,false);
        context.diagnostic(JSON.stringify({dofs:f.M,maxEquilibrium,maxConstraint,originalPointQueries:f.field.calls}));
    } finally {f.mesh.dispose();}
});
