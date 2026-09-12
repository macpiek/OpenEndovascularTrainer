import {compileCompositeMaterial} from './kirchhoffCompositeElement.js';
import {evaluateCompositeContinuousStrains,createCompositeContinuousFrameWorkspace} from './kirchhoffCompositeContinuousFrame.js';
import {compositeGaussRules as rules} from './kirchhoffCompositeQuadrature.js';
const positive=(v,name)=>{if(!Number.isFinite(v)||v<=0)throw new RangeError(`${name} must be positive and finite`);return v;};
const maxAbs=(a,start,end)=>{let value=0;for(let j=start;j<end;j++)value=Math.max(value,Math.abs(a[j]));return value;};
function ownMaterial(input) {
    const stiffness=input?.stiffness?.length===9?Array.from({length:3},(_,j)=>Array.from(input.stiffness.slice(3*j,3*j+3))):input?.stiffness;
    return compileCompositeMaterial({...input,stiffness});
}
function quadratureFailure(reason,details) {const error=new RangeError(reason);error.code='continuous-elastic-quadrature-budget';error.details=details;throw error;}

/** E = integral .5 (kappa-kappa0)^T K (kappa-kappa0) ds, using
 * Darboux strain of the SAME continuous material frame as surface motion.
 * Full tangents include strain Hessians, hence geometric stiffness. They are
 * not forced positive definite. Independent tools keep their own frame/K/k0.
 * Material providers must be immutable within a prepared step. Declared
 * material breaks and the director interpolation midpoint split quadrature.
 * Paired Gauss-4/Gauss-8 estimates control E, gradient and tangent separately;
 * exhausted depth/sample budgets reject instead of returning partial forces.
 */
export function createCompositeContinuousElasticEdge({frame,dsDx,material,materialAt,materialBreaks=[],quadrature={},workspace=createCompositeContinuousFrameWorkspace()}={}) {
    positive(dsDx,'Material reference metric');
    if(!Number.isInteger(frame?.configurationDofs)||frame.configurationDofs<1||frame.coordinateInterval?.length!==2)
        throw new RangeError('A prepared continuous material frame is required');
    if((material===undefined)===(materialAt===undefined)||materialAt!==undefined&&typeof materialAt!=='function')
        throw new RangeError('Exactly one material or immutable material provider is required');
    const N=frame.configurationDofs,[start,end]=frame.coordinateInterval,L=positive(end-start,'Physical edge length'),
        constant=material===undefined?null:ownMaterial(material),materials=new Map(),
        tolerance={energy:positive(quadrature.energy??1e-9,'Quadrature energy tolerance'),gradient:positive(quadrature.gradient??1e-8,'Quadrature gradient tolerance'),hessian:positive(quadrature.hessian??1e-7,'Quadrature Hessian tolerance')},
        maxDepth=quadrature.maxDepth??6,maxEvaluations=quadrature.maxEvaluations??4096;
    if(!Number.isInteger(maxDepth)||maxDepth<0||maxDepth>12||!Number.isInteger(maxEvaluations)||maxEvaluations<1||maxEvaluations>100000)
        throw new RangeError('Finite quadrature depth and evaluation budgets are required');
    if(!Array.isArray(materialBreaks)||materialBreaks.some(x=>!Number.isFinite(x)||x<=start||x>=end)||new Set(materialBreaks).size!==materialBreaks.length)
        throw new RangeError('Material breaks must be distinct interior physical coordinates');
    const cuts=[start,...new Set([start+L/2,...materialBreaks].sort((a,b)=>a-b)),end],
        output={scope:'continuous-material-frame-elastic-energy',configurationColumns:frame.configurationColumns,configurationDofs:N,
            energy:NaN,gradient:new Float64Array(N),hessian:new Float64Array(N*N),hessianValid:false,operatorReady:false,quadrature:null};
    let busy=false;
    function invalidate(){output.energy=NaN;output.gradient.fill(NaN);output.hessian.fill(NaN);output.hessianValid=output.operatorReady=false;output.quadrature=null;}
    function sampleMaterial(coordinate) {
        if(constant)return constant;
        if(!materials.has(coordinate)) {
            if(materials.size>=100000)quadratureFailure('Material quadrature cache budget exhausted',{toolId:frame.toolId,edge:frame.edge});
            materials.set(coordinate,ownMaterial(materialAt({toolId:frame.toolId,edge:frame.edge,coordinate,start,end})));
        }
        return materials.get(coordinate);
    }
    function evaluate({positions,angles},{order='full'}={}) {
        if(busy)throw new RangeError('Continuous elasticity workspace is busy');
        invalidate();busy=true;
        try {
            if(!['full','gradient'].includes(order))throw new RangeError('Continuous elasticity order must be full or gradient');
            const full=order==='full',size=1+N+(full?N*N:0),total=new Float64Array(size),errors=[0,0,0];
            let evaluations=0,intervals=0,deepest=0,workspaceBytes=0;
            function densities(a,b) {
                const coordinates=rules.flatMap(rule=>rule.points.map(f=>a+(b-a)*f));
                if(evaluations+coordinates.length>maxEvaluations)quadratureFailure('Continuous elastic quadrature evaluation budget exhausted',{evaluations,maxEvaluations,interval:[a,b]});
                const response=evaluateCompositeContinuousStrains({positions,angles,coordinates,dsDx,order:full?'full':'value'},frame,workspace);
                evaluations+=coordinates.length;
                return response.map((r,sample)=>{
                    workspaceBytes=Math.max(workspaceBytes,r.workspaceBytes);
                    const m=sampleMaterial(coordinates[sample]),error=r.strain.map((v,j)=>v-m.intrinsic[j]),K=m.stiffness,
                        moment=error.map((_,i)=>K[3*i]*error[0]+K[3*i+1]*error[1]+K[3*i+2]*error[2]),value=new Float64Array(size);
                    value[0]=.5*error.reduce((sum,v,j)=>sum+v*moment[j],0)+m.energyOffset;
                    for(let i=0;i<N;i++)for(let s=0;s<3;s++) {
                        const Ji=r.jacobian[s*N+i];value[1+i]+=Ji*moment[s];
                        if(full)for(let j=0;j<N;j++) {
                            let weighted=0;for(let t=0;t<3;t++)weighted+=K[3*s+t]*r.jacobian[t*N+j];
                            value[1+N+i*N+j]+=Ji*weighted+moment[s]*r.hessian[(s*N+i)*N+j];
                        }
                    }
                    if(!value.every(Number.isFinite))throw new RangeError('Nonfinite continuous elastic density');
                    return value;
                });
            }
            function integrate(a,b,depth) {
                deepest=Math.max(deepest,depth);
                const values=densities(a,b),sums=rules.map(()=>new Float64Array(size));let offset=0;
                rules.forEach((rule,index)=>{rule.weights.forEach((weight,j)=>{const value=values[offset+j],scale=(b-a)*dsDx*weight;for(let k=0;k<size;k++)sums[index][k]+=scale*value[k];});offset+=rule.points.length;});
                const difference=Float64Array.from(sums[1],(v,j)=>v-sums[0][j]),estimate=[Math.abs(difference[0]),maxAbs(difference,1,1+N),full?maxAbs(difference,1+N,size):0],fraction=(b-a)/L,
                    okay=estimate[0]<=fraction*tolerance.energy&&estimate[1]<=fraction*tolerance.gradient&&(!full||estimate[2]<=fraction*tolerance.hessian);
                if(!okay) {
                    if(depth===maxDepth)quadratureFailure('Continuous elastic quadrature depth exhausted',{depth,interval:[a,b],estimate,tolerance});
                    const middle=a+(b-a)/2;integrate(a,middle,depth+1);integrate(middle,b,depth+1);return;
                }
                for(let k=0;k<size;k++)total[k]+=sums[1][k];estimate.forEach((v,j)=>errors[j]+=v);intervals++;
            }
            for(let j=0;j<cuts.length-1;j++)integrate(cuts[j],cuts[j+1],0);
            output.energy=total[0];output.gradient.set(total.subarray(1,1+N));if(full)output.hessian.set(total.subarray(1+N));
            output.quadrature={evaluations,intervals,deepest,estimatedError:{energy:errors[0],gradient:errors[1],hessian:full?errors[2]:null},
                tolerance:{...tolerance},workspaceBytes,converged:true,method:'split-adaptive-Gauss-4-8',rigorousErrorBound:false};
            output.hessianValid=full;output.operatorReady=true;output.evaluationOrder=order;return output;
        } catch(error){invalidate();throw error;} finally {busy=false;}
    }
    invalidate();return Object.freeze({frame,evaluate,scope:output.scope,dsDx,coordinateInterval:frame.coordinateInterval,configurationDofs:N});
}
