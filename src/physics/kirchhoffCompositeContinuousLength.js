import {sampleCompositeContinuousBasis,compositeContinuousBezierControls} from './kirchhoffCompositeContinuousGeometry.js';
import {compositeGaussRules as rules} from './kirchhoffCompositeQuadrature.js';

const positive=(x,name)=>{if(!Number.isFinite(x)||x<=0)throw new RangeError(`${name} must be positive and finite`);return x;};
const dot=(a,b)=>a.reduce((s,v,k)=>s+v*b[k],0);
function compensatedAdd(values,compensation,index,value) {
    const y=value-compensation[index],sum=values[index]+y;
    compensation[index]=(sum-values[index])-y;values[index]=sum;
}
function failure(reason,details={}){const e=new RangeError(reason);e.code='continuous-length-refinement-required';e.details=details;throw e;}
function split(controls) {
    let row=controls.map(v=>v.slice());const left=[row[0]],right=[row.at(-1)];
    while(row.length>1){row=row.slice(1).map((v,j)=>v.map((x,k)=>(x+row[j][k])/2));left.push(row[0]);right.unshift(row.at(-1));}
    return [left,right];
}

/** Physical arclength of the same C2 polynomial curve as inertia/elasticity.
 * J = integral N'_i t dx; H = integral N'_i N'_j (I-tt^T)/|q'| dx.
 * Both are analytic configuration derivatives, with no differentiation of
 * finite differences or an endpoint chord. A Bernstein subdivision check
 * establishes nonzero tangent over the WHOLE interval, including between
 * quadrature sites. Preserving this integral does not impose pointwise
 * inextensibility inside a coarse element; speed bounds remain explicit.
 */
export function createCompositeContinuousLength({geometry,maxDepth=10,maxEvaluations=4096}={}) {
    const sample=sampleCompositeContinuousBasis(geometry,.5),n=sample.weights.length,N=3*n,L=geometry.coordinates[1]-geometry.coordinates[0],basisCache=new Map();
    if(!Number.isInteger(maxDepth)||maxDepth<0||maxDepth>16||!Number.isInteger(maxEvaluations)||maxEvaluations<1||maxEvaluations>100000)
        throw new RangeError('Finite continuous length quadrature budgets are required');
    const output={scope:'continuous-curve-arclength',length:NaN,gradient:new Float64Array(N),hessian:new Float64Array(N*N),
        hessianValid:false,operatorReady:false,quadrature:null,speedBounds:null,pointwiseInextensibility:false};
    let busy=false;
    function invalidate(){output.length=NaN;output.gradient.fill(NaN);output.hessian.fill(NaN);output.hessianValid=output.operatorReady=false;output.quadrature=output.speedBounds=null;}
    function evaluate(positions,{order='full',lengthTolerance=1e-12,gradientTolerance=1e-12,hessianTolerance=1e-11}={}) {
        if(busy)throw new RangeError('Continuous length operator is busy');invalidate();busy=true;
        try {
            if(!['full','gradient'].includes(order))throw new RangeError('Continuous length order must be full or gradient');
            const full=order==='full',tolerance=[positive(lengthTolerance,'Length quadrature tolerance'),positive(gradientTolerance,'Length gradient tolerance'),positive(hessianTolerance,'Length Hessian tolerance')];
            if(positions?.length!==n||positions.some(v=>v?.length!==3||!v.every(Number.isFinite)))throw new RangeError('Finite positions must follow the continuous node support');
            const p=positions.map(v=>Array.from(v)),controls=compositeContinuousBezierControls(geometry,p.map(v=>v.map((x,k)=>x-p[0][k]))),
                derivative=controls.slice(1).map((v,j)=>v.map((x,k)=>5*(x-controls[j][k])/L));
            let lower=Infinity,upper=0,regularityIntervals=0;
            function regularity(c,depth) {
                const halves=split(c),middle=halves[0].at(-1),norm=Math.hypot(...middle),bound=Math.max(...c.map(v=>Math.hypot(...v))),
                    projection=norm>0?Math.min(...c.map(v=>dot(v,middle)/norm)):0;
                if(projection>128*Number.EPSILON*bound){lower=Math.min(lower,projection);upper=Math.max(upper,bound);regularityIntervals++;return;}
                if(depth===16)failure('Continuous tangent regularity requires refinement',{edge:geometry.edge,depth});
                regularity(halves[0],depth+1);regularity(halves[1],depth+1);
            }
            regularity(derivative,0);
            const size=1+N+(full?N*N:0),total=new Float64Array(size),totalCompensation=new Float64Array(size),errors=[0,0,0];let evaluations=0,intervals=0,deepest=0;
            function integral(a,b,depth) {
                deepest=Math.max(deepest,depth);
                if(evaluations+12>maxEvaluations)failure('Continuous length quadrature evaluation budget exhausted',{evaluations,maxEvaluations});
                const sums=rules.map(()=>new Float64Array(size)),compensation=rules.map(()=>new Float64Array(size));
                rules.forEach((rule,r)=>rule.points.forEach((f,s)=>{
                    const at=a+(b-a)*f;
                    if(!basisCache.has(at)){
                        if(basisCache.size>=512)basisCache.clear();
                        const b=sampleCompositeContinuousBasis(geometry,at);basisCache.set(at,{anchor:b.anchor,first:b.first});
                    }
                    const basis=basisCache.get(at),d=[0,0,0],dc=[0,0,0];
                    for(let j=0;j<n;j++)if(j!==basis.anchor)for(let k=0;k<3;k++)compensatedAdd(d,dc,k,basis.first[j]*(p[j][k]-p[basis.anchor][k]));
                    const speed=positive(Math.hypot(...d),'Continuous tangent length'),t=d.map(v=>v/speed),w=(b-a)*L*rule.weights[s],out=sums[r];
                    compensatedAdd(out,compensation[r],0,w*speed);
                    for(let j=0;j<n;j++)for(let k=0;k<3;k++) {
                        const row=3*j+k;compensatedAdd(out,compensation[r],1+row,w*basis.first[j]*t[k]);
                        if(full)for(let i=0;i<n;i++)for(let l=0;l<3;l++)compensatedAdd(out,compensation[r],1+N+row*N+3*i+l,w*basis.first[j]*basis.first[i]/speed*((k===l?1:0)-t[k]*t[l]));
                    }
                }));
                evaluations+=12;
                const error=[0,0,0];
                for(let j=0;j<size;j++){const group=j===0?0:j<=N?1:2;error[group]=Math.max(error[group],Math.abs(sums[1][j]-sums[0][j]));}
                if(error.some((v,j)=>v>(b-a)*tolerance[j])) {
                    if(depth===maxDepth)failure('Continuous length quadrature depth exhausted',{edge:geometry.edge,depth,interval:[a,b],error,tolerance,
                        localTolerance:tolerance.map(v=>(b-a)*v),evaluations,
                        roundoffIndicators:[0,1,2].map(group=>32*Number.EPSILON*Math.max(0,...Array.from(sums[1]).filter((_,j)=>(j===0?0:j<=N?1:2)===group).map(Math.abs)))});
                    const middle=(a+b)/2;integral(a,middle,depth+1);integral(middle,b,depth+1);return;
                }
                for(let j=0;j<size;j++)compensatedAdd(total,totalCompensation,j,sums[1][j]);error.forEach((v,j)=>errors[j]+=v);intervals++;
            }
            integral(0,1,0);
            if(!total.every(Number.isFinite))throw new RangeError('Nonfinite continuous arclength response');
            output.length=positive(total[0],'Physical curve arclength');output.gradient.set(total.subarray(1,1+N));if(full)output.hessian.set(total.subarray(1+N));
            output.quadrature={evaluations,intervals,deepest,estimatedError:{length:errors[0],gradient:errors[1],hessian:full?errors[2]:null},
                tolerance:{length:tolerance[0],gradient:tolerance[1],hessian:full?tolerance[2]:null},summation:'compensated',rigorousErrorBound:false};
            output.speedBounds={lower,upper,regularityIntervals,wholeInterval:true};output.operatorReady=true;output.hessianValid=full;return output;
        }catch(error){invalidate();throw error;}finally{busy=false;}
    }
    invalidate();return Object.freeze({geometry,nodeIndices:geometry.nodeIndices,coordinateLength:L,evaluate,scope:output.scope,
        get diagnostics(){return {cachedBasisSamples:basisCache.size,basisCacheCapacity:512};}});
}
