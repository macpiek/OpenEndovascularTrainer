// The same bounded closest-segment algorithm and thresholds used by
// EndovascularPhysicsWorld.#closestSegmentParameters / #solveToolContact.
const EPS=1e-8,N=12,plans=new WeakMap();
const keys=['innerStart','innerEnd','outerStart','outerEnd'];
const dot=(a,b)=>a.reduce((s,v,i)=>s+v*b[i],0),sub=(a,b)=>a.map((v,i)=>v-b[i]);
const invalid=(out,reason)=>{out.supported=false;out.reason=reason;out.hessianValid=out.witnessJacobianValid=false;return out;};
const constant=v=>({v,d:new Float64Array(N)}),variable=(v,j)=>{const a=constant(v);a.d[j]=1;return a;};
const add=(a,b)=>({v:a.v+b.v,d:Float64Array.from(a.d,(v,j)=>v+b.d[j])}),neg=a=>({v:-a.v,d:Float64Array.from(a.d,v=>-v)}),
    minus=(a,b)=>add(a,neg(b)),mul=(a,b)=>({v:a.v*b.v,d:Float64Array.from(a.d,(v,j)=>v*b.v+a.v*b.d[j])}),
    div=(a,b)=>({v:a.v/b.v,d:Float64Array.from(a.d,(v,j)=>(v-(a.v/b.v)*b.d[j])/b.v)}),
    clamp=a=>a.v<=0?constant(0):a.v>=1?constant(1):a,
    sum=a=>a.reduce(add,constant(0)),adDot=(a,b)=>sum(a.map((v,i)=>mul(v,b[i]))),adSub=(a,b)=>a.map((v,i)=>minus(v,b[i]));

export function createCompositeExternalCapsuleGeometryWorkspace() {
    const out={supported:false,reason:'not-evaluated',dofCount:N,positions:Array.from({length:4},()=>new Float64Array(3)),
        innerPoint:new Float64Array(3),outerPoint:new Float64Array(3),normal:new Float64Array(3),physicalNormal:new Float64Array(3),
        innerWeights:new Float64Array(2),outerWeights:new Float64Array(2),innerTGradient:new Float64Array(N),outerTGradient:new Float64Array(N),
        normalJacobian:new Float64Array(3*N),gapJacobian:new Float64Array(N),normalForceColumn:new Float64Array(N),forceColumn:new Float64Array(N),
        normalDerivative:new Float64Array(N*N),rawContact:{kind:'external-capsule'}};
    plans.set(out,true);return out;
}

/** Branch derivative of the original affine capsule witness. Clamp equality
 * selects the constant endpoint member. Parallel pairs keep the original
 * deterministic endpoint chart. No finite differences, search expansion,
 * force projection or clearance/CCD certificate is introduced here.
 */
export function evaluateCompositeExternalCapsuleContact({input,order='full'},out) {
    if(!plans.has(out))throw new TypeError('Use an owned external capsule geometry workspace');
    invalid(out,'not-evaluated');
    for(const key of ['innerPoint','outerPoint','normal','physicalNormal','innerWeights','outerWeights','innerTGradient','outerTGradient','normalJacobian','gapJacobian','normalForceColumn','forceColumn','normalDerivative'])out[key].fill(NaN);
    out.gap=out.distance=out.innerT=out.outerT=NaN;out.openDistalExcluded=false;out.queryCount=1;
    if(!['full','gradient','witness'].includes(order))throw new RangeError('External capsule order must be full, gradient or witness');
    const positions=keys.map((key,i)=>{const v=input?.[key];if(v?.length!==3||!Array.from(v).every(Number.isFinite))throw new TypeError(`Finite ${key} required`);out.positions[i].set(v);return Array.from(v);});
    for(const key of ['innerRadius','outerRadius'])if(!(input[key]>=0)||!Number.isFinite(input[key]))throw new RangeError(`Nonnegative ${key} required`);
    const [A,B,C,D]=positions.map((p,i)=>p.map((v,k)=>variable(v,3*i+k))),u=adSub(B,A),v=adSub(D,C),w=adSub(A,C),
        aa=adDot(u,u),bb=adDot(u,v),cc=adDot(v,v),dd=adDot(u,w),ee=adDot(v,w),den=minus(mul(aa,cc),mul(bb,bb));
    if(!(aa.v>EPS&&cc.v>EPS))return invalid(out,'degenerate-native-edge');
    let s=den.v>EPS?clamp(div(minus(mul(bb,ee),mul(cc,dd)),den)):constant(0);
    const t=clamp(div(add(mul(bb,s),ee),cc));s=clamp(div(minus(mul(bb,t),dd),aa));
    const p=A.map((a,k)=>add(a,mul(s,u[k]))),q=C.map((c,k)=>add(c,mul(t,v[k]))),r=adSub(p,q),distance=Math.hypot(...r.map(x=>x.v));
    out.innerT=s.v;out.outerT=t.v;out.innerPoint.set(p.map(x=>x.v));out.outerPoint.set(q.map(x=>x.v));out.distance=distance;
    out.gap=distance-input.innerRadius-input.outerRadius;
    out.innerWeights.set([1-s.v,s.v]);out.outerWeights.set([1-t.v,t.v]);
    out.rawContact={kind:'external-capsule',innerT:s.v,outerT:t.v,distance,gap:out.gap,
        innerMaterialSegmentId:input.innerMaterialSegmentId,outerMaterialSegmentId:input.outerMaterialSegmentId};
    out.branchSignature=JSON.stringify(['original-native-capsule',den.v>EPS?'nonparallel':'parallel',s.v===0?'a0':s.v===1?'a1':'ai',t.v===0?'b0':t.v===1?'b1':'bi']);
    out.derivativeScope='original-native-capsule-clamped-one-sided';
    out.openDistalExcluded=!!input.openDistalB&&t.v>=1-1e-5&&dot(sub(Array.from(out.innerPoint),positions[3]),sub(positions[3],positions[2]))>0;
    if(!(distance>=EPS))return invalid(out,'coincident-native-capsule-witness');
    const normal=r.map(a=>a.v/distance),Dd=new Float64Array(N);
    for(let j=0;j<N;j++)for(let k=0;k<3;k++)Dd[j]+=normal[k]*r[k].d[j];
    out.normal.set(normal);out.physicalNormal.set(normal);out.gapJacobian.set(Dd);
    const weights=[minus(constant(1),s),s,neg(minus(constant(1),t)),neg(t)];
    for(let block=0;block<4;block++)for(let k=0;k<3;k++) {
        const i=3*block+k,Bn=weights[block].v*normal[k];out.normalForceColumn[i]=Bn;out.forceColumn[i]=-Bn;
        if(order==='full')for(let j=0;j<N;j++)out.normalDerivative[i*N+j]=weights[block].d[j]*normal[k]+weights[block].v*(r[k].d[j]-normal[k]*Dd[j])/distance;
    }
    if(order!=='gradient') {
        out.innerTGradient.set(s.d);out.outerTGradient.set(t.d);
        for(let k=0;k<3;k++)for(let j=0;j<N;j++)out.normalJacobian[k*N+j]=(r[k].d[j]-normal[k]*Dd[j])/distance;
    }
    out.supported=true;out.reason=null;out.hessianValid=order==='full';out.witnessJacobianValid=order!=='gradient';return out;
}
