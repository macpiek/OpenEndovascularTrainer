/** Outward intersection of an affine force path with the physical Coulomb
 * ellipse. The result is only a line-search proposal: neither endpoint nor
 * any force is projected, and the actual nonlinear slip/geometry must still
 * be evaluated before accepting a trial. This also applies to anisotropic
 * friction; being on its cone alone does NOT prove maximum dissipation.
 */
export function findCompositeFrictionConeCrossing({start,end,mu}) {
    if(!mu||mu.length!==2||!mu.every(v=>Number.isFinite(v)&&v>=0))throw new TypeError('Two nonnegative friction coefficients are required');
    for(const point of [start,end])if(!point||!Number.isFinite(point.Fn)||!point.traction||point.traction.length!==2||!point.traction.every(Number.isFinite))
        throw new TypeError('Finite normal and tangential force endpoints are required');
    if(start.Fn<0||end.Fn<=0||mu.every(v=>v===0))return null;
    const u=[0,0,start.Fn],v=[0,0,end.Fn];
    for(let k=0;k<2;k++) {
        // A disabled axis is exactly constrained to zero. Do not substitute
        // a small coefficient or infer a cone crossing for its violation.
        if(mu[k]===0){if(start.traction[k]!==0||end.traction[k]!==0)return null;}
        else {u[k]=start.traction[k]/mu[k];v[k]=end.traction[k]/mu[k];}
    }
    if(!u.every(Number.isFinite)||!v.every(Number.isFinite)||Math.hypot(v[0],v[1])<=v[2])return null;
    const scale=Math.max(...u.map(Math.abs),...v.map(Math.abs));
    u.forEach((x,k)=>u[k]=x/scale);v.forEach((x,k)=>v[k]=x/scale-u[k]);
    // ||Ft(t)/mu||^2 - Fn(t)^2 = A*t^2+B*t+C. Scale before
    // squaring and use the cancellation-resistant quadratic root pair.
    const A=v[0]**2+v[1]**2-v[2]**2,B=2*(u[0]*v[0]+u[1]*v[1]-u[2]*v[2]),C=u[0]**2+u[1]**2-u[2]**2;
    const discriminant=B*B-4*A*C;
    if(discriminant<0)return null;
    const q=-.5*(B+(B>=0?1:-1)*Math.sqrt(discriminant));
    const roots=A===0?[-C/B]:[q/A,C/q];
    let crossing=Infinity;
    for(const t of roots)if(t>0&&t<1&&B+2*A*t>0&&u[2]+t*v[2]>0)crossing=Math.min(crossing,t);
    return Number.isFinite(crossing)?crossing:null;
}

/** Safeguarded proposal after a rejected whole-system trial. Forces are
 * affine along that same Newton direction, so this needs no new mechanics
 * assembly and no frozen/approximate surface-slip derivative. Match samples
 * and their current law explicitly; unsupported paths use ordinary halving.
 */
export function proposeCompositeFrictionBacktrack(startSamples,endSamples,alpha) {
    if(!Number.isFinite(alpha)||alpha<=0||alpha>1)throw new RangeError('A trial step in (0,1] is required');
    if(!startSamples||!endSamples||startSamples.length!==endSamples.length)return null;
    let proposed=Infinity;
    for(let i=0;i<startSamples.length;i++) {
        const start=startSamples[i],end=endSamples[i],mu=start.mu;
        if(start.sampleId!==end.sampleId||!mu||!end.mu||mu.some((v,k)=>v!==end.mu[k]))continue;
        const crossing=findCompositeFrictionConeCrossing({start,end,mu});
        // Bound numerical contraction independently of physical tolerances.
        // Near either endpoint, retain the existing half-step backtracking.
        if(crossing!==null&&crossing>=.1&&crossing<=.9)proposed=Math.min(proposed,alpha*crossing);
    }
    return Number.isFinite(proposed)?proposed:null;
}
