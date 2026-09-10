import {evaluateCompositeContinuousFrame,createCompositeContinuousFrameWorkspace} from './kirchhoffCompositeContinuousFrame.js';

const finite=(x,name)=>{if(!Number.isFinite(x))throw new RangeError(`${name} must be finite`);return x;};
const vector=(v,n,name)=>{if(v?.length!==n)throw new RangeError(`${name} needs ${n} entries`);return Array.from(v,x=>finite(x,name));};
const cross=(a,b)=>[a[1]*b[2]-a[2]*b[1],a[2]*b[0]-a[0]*b[2],a[0]*b[1]-a[1]*b[0]],
    dot=(a,b)=>a.reduce((s,v,k)=>s+v*b[k],0),add=(a,b)=>a.map((v,k)=>v+b[k]),sub=(a,b)=>a.map((v,k)=>v-b[k]);

/** Work-conjugate surface rate and B/DB at an explicitly supplied WORLD point
 * on one physical C2/C1 tool. All position/spin support columns remain real
 * reactions. Feed is an explicit prescribed material-rate contribution.
 * The contact detector owns its foot, witness and tangent axes and must chain
 * their derivatives; this operator does not certify an old straight capsule.
 * No old point/frame jump is consumed as a classical angular velocity.
 */
export function evaluateCompositeContinuousSurfaceMotion(input,frame,workspace=createCompositeContinuousFrameWorkspace(frame)) {
    const {positions,angles,coordinate,materialMap,contact,wall,rateMode='instantaneous',dt,order='full'}=input;
    if(!['instantaneous','backward-euler-grid'].includes(rateMode))throw new RangeError('Explicit instantaneous or backward-euler-grid rates are required');
    if(rateMode==='backward-euler-grid'&&!(finite(dt,'dt')>0))throw new RangeError('Backward Euler rates require positive dt');
    const point=vector(contact?.point,3,'Physical world contact point'),axes=contact?.axes?.map(v=>vector(v,3,'World tangent axis')),
        wallVelocity=vector(wall?.velocity,3,'Explicit wall velocity');
    if(axes?.length!==2||axes.some(a=>Math.abs(dot(a,a)-1)>1e-10)||Math.abs(dot(axes[0],axes[1]))>1e-10)throw new RangeError('Two orthonormal world tangent axes are required');
    const state=evaluateCompositeContinuousFrame({positions,angles,coordinate,order},frame,workspace),N=state.configurationDofs,D=N+1,Q=10,full=order==='full',
        positionCount=frame.positionNodeIndices.length,angleCount=frame.angleEdgeIndices.length;
    let rates;
    if(rateMode==='instantaneous') {
        if(input.positionRates?.length!==positionCount)throw new RangeError('Physical grid rates must cover every local position column');
        rates=input.positionRates.flatMap(v=>vector(v,3,'Own grid velocity')).concat(vector(input.angleRates,angleCount,'Own unwrapped angle rates'));
    } else {
        if(input.positionRates!==undefined||input.angleRates!==undefined)throw new RangeError('Do not mix explicit and backward Euler grid rates');
        rates=positions.flatMap((v,j)=>v.map((x,k)=>(x-state.previousPositions[j][k])/dt)).concat(angles.map((x,j)=>(x-state.previousAngles[j])/dt));
    }
    vector(rates,N,'Own generalized rates');
    const dsDx=finite(materialMap?.dsDx,'Material metric'),sStart=finite(materialMap?.sStart,'Material start');
    if(!(dsDx>0))throw new RangeError('Material metric must be positive');
    const labelRates=typeof materialMap.dsDt==='number'?[finite(materialMap.dsDt,'Material rate'),materialMap.dsDt]:vector(materialMap.dsDt,2,'Material endpoint rates'),
        L=frame.coordinateInterval[1]-frame.coordinateInterval[0],fraction=(coordinate-frame.coordinateInterval[0])/L,
        u=-((1-fraction)*labelRates[0]+fraction*labelRates[1])/dsDx,ux=-(labelRates[1]-labelRates[0])/dsDx/L,
        lever=sub(point,state.position),J=j=>[0,1,2].map(k=>state.positionJacobian[k*D+j]),O=j=>[0,1,2].map(k=>state.angularRateMap[k*D+j]),
        H=(i,j)=>[0,1,2].map(k=>state.positionHessian[(k*D+i)*D+j]),DO=(i,j)=>[0,1,2].map(k=>state.angularRateMapDerivative[(k*D+i)*D+j]),
        spatialOmega=O(N),feedBase=add(J(N),cross(spatialOmega,lever)),vectors=Array.from({length:N},(_,j)=>add(J(j),cross(O(j),lever))),
        forceMap=Float64Array.from(vectors.flatMap(v=>axes.map(a=>dot(a,v)))),
        configurationDerivative=full?new Float64Array(N*2*N):null,queryDerivative=full?new Float64Array(N*2*Q):null,
        prescribedRate=axes.map(a=>u*dot(a,feedBase)-dot(a,wallVelocity)),prescribedConfigurationDerivative=full?new Float64Array(2*N):null,
        prescribedQueryDerivative=full?new Float64Array(2*Q):null,rateConfigurationDerivative=full?new Float64Array(2*N):null,rateQueryDerivative=full?new Float64Array(2*Q):null;
    if(full) {
        const basis=[[1,0,0],[0,1,0],[0,0,1]];
        for(let j=0;j<N;j++) {
            const omega=O(j);
            for(let l=0;l<D;l++) {
                const v=add(H(j,l),sub(cross(DO(j,l),lever),cross(omega,J(l))));
                for(let c=0;c<2;c++)if(l<N)configurationDerivative[(2*j+c)*N+l]=dot(axes[c],v);
                else queryDerivative[(2*j+c)*Q]=dot(axes[c],v);
            }
            for(let c=0;c<2;c++)for(let k=0;k<3;k++) {
                queryDerivative[(2*j+c)*Q+1+k]=dot(axes[c],cross(omega,basis[k]));
                queryDerivative[(2*j+c)*Q+4+3*c+k]=vectors[j][k];
            }
        }
        for(let l=0;l<D;l++) {
            const v=add(H(N,l),sub(cross(DO(N,l),lever),cross(spatialOmega,J(l))));
            for(let c=0;c<2;c++)if(l<N)prescribedConfigurationDerivative[c*N+l]=u*dot(axes[c],v);
            else prescribedQueryDerivative[c*Q]=ux*dot(axes[c],feedBase)+u*dot(axes[c],v);
        }
        for(let c=0;c<2;c++)for(let k=0;k<3;k++) {
            prescribedQueryDerivative[c*Q+1+k]=u*dot(axes[c],cross(spatialOmega,basis[k]));
            prescribedQueryDerivative[c*Q+4+3*c+k]=u*feedBase[k]-wallVelocity[k];
        }
        for(let c=0;c<2;c++)for(let l=0;l<N;l++)rateConfigurationDerivative[c*N+l]=prescribedConfigurationDerivative[c*N+l]
            +rates.reduce((sum,v,j)=>sum+configurationDerivative[(2*j+c)*N+l]*v,0)+(rateMode==='backward-euler-grid'?forceMap[2*l+c]/dt:0);
        for(let c=0;c<2;c++)for(let l=0;l<Q;l++)rateQueryDerivative[c*Q+l]=prescribedQueryDerivative[c*Q+l]
            +rates.reduce((sum,v,j)=>sum+queryDerivative[(2*j+c)*Q+l]*v,0);
    }
    const velocity=[0,1,2].map(k=>u*J(N)[k]+rates.reduce((sum,v,j)=>sum+v*state.positionJacobian[k*D+j],0)),
        omega=[0,1,2].map(k=>u*spatialOmega[k]+rates.reduce((sum,v,j)=>sum+v*state.angularRateMap[k*D+j],0)),
        surfaceVelocity=add(velocity,cross(omega,lever)),slipRate=axes.map((a,c)=>prescribedRate[c]+rates.reduce((sum,v,j)=>sum+forceMap[2*j+c]*v,0));
    for(const array of [rates,forceMap,prescribedRate,velocity,omega,surfaceVelocity,slipRate,...(full?[configurationDerivative,queryDerivative,prescribedConfigurationDerivative,prescribedQueryDerivative,rateConfigurationDerivative,rateQueryDerivative]:[])])
        if(!Array.from(array).every(Number.isFinite))throw new RangeError('Nonfinite continuous surface kinematics or force map');
    return {scope:'continuous-own-physical-surface-rate-and-force-map',toolId:frame.toolId,configurationColumns:frame.configurationColumns,configurationDofs:N,queryDofs:Q,
        forceMap,configurationDerivative,queryDerivative,prescribedRate,prescribedConfigurationDerivative,prescribedQueryDerivative,slipRate,
        rateConfigurationDerivative,rateQueryDerivative,velocity,omega,surfaceVelocity,lever,rates,position:state.position,directors:state.directors,
        materialLabel:finite(sStart+dsDx*L*fraction,'Current material label'),feed:u,order,rateMode,
        forceMapValid:true,derivativeValid:full,contactCertified:false,finiteStepSlipKnown:false,
        frameWorkspaceBytes:state.workspaceBytes,frameOperations:state.operations};
}
