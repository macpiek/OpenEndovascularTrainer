/** A two-coordinate transverse mode of ONE composite-chain cell.
 * The supplied quadratic is the local linearization of the two material
 * energies, inertia and loads, with the catheter axis held at the common q.
 * It is not a second full rod, a contact-geometry finder, or a certificate
 * for neglected nonlinear/higher modes. Overlapping modes must be assembled
 * as one coupled patch, not independently passed to this cell eliminator.
 */
function finite(values,length,name) {
    if(!values || values.length!==length || !values.every(Number.isFinite))
        throw new TypeError(`${name} needs ${length} finite values`);
}

function inverseSPD(a,b,d) {
    const l0=Math.sqrt(a),l1=b/l0,pivot=d-l1*l1;
    if(!(a>0) || !(pivot>32*Number.EPSILON*(Math.abs(d)+l1*l1)))
        throw new RangeError('A relative mode needs a resolved positive definite physical stiffness');
    const invD=1/pivot,invB=-l1/l0*invD,invA=1/a+(l1/l0)**2*invD;
    if(![invA,invB,invD].every(Number.isFinite)) throw new RangeError('Nonfinite relative compliance');
    return [invA,invB,invD];
}

/** Minimize r.y + .5 y.K.y over |y| <= clearance. Units: y and clearance
 * are lengths, r is force, K is force/length. The normal force is lambda*|y|
 * for this physical unilateral constraint, NEVER the common-axis reaction.
 * For an open gap it is exactly zero, even if the eliminated mode is loaded.
 */
export function solveCompositeClearanceCell({stiffness,gradient,clearance,
    forceTolerance=1e-10,gapTolerance=1e-12,maxIterations=80}) {
    finite(stiffness,3,'symmetric stiffness [K00,K01,K11]');finite(gradient,2,'relative gradient');
    if(!Number.isFinite(clearance)||clearance<=0) throw new RangeError('A positive physical clearance is required');
    if(!Number.isFinite(forceTolerance)||forceTolerance<=0||!Number.isFinite(gapTolerance)||gapTolerance<=0||
        !Number.isInteger(maxIterations)||maxIterations<1) throw new RangeError('Positive tolerances and an iteration budget are required');
    const [a,b,d]=stiffness,[r0,r1]=gradient;
    const solve=lambda=>{
        const inverse=inverseSPD(a+lambda,b,d+lambda);
        const y=[-inverse[0]*r0-inverse[1]*r1,-inverse[1]*r0-inverse[2]*r1];
        return {inverse,y,radius:Math.hypot(...y)};
    };
    let state=solve(0),lambda=0,iterations=0;
    const active=state.radius>clearance;
    if(active) {
        let lo=0,hi=Math.hypot(r0,r1)/clearance;
        if(!Number.isFinite(hi)) throw new RangeError('Nonfinite contact multiplier bracket');
        // The upper bound is feasible since K is positive definite.
        lambda=hi;state=solve(lambda);
        for(;iterations<maxIterations;iterations++) {
            const gap=clearance-state.radius;
            if(Math.abs(gap)<=gapTolerance && lambda*Math.abs(gap)<=forceTolerance) break;
            if(state.radius>clearance) lo=lambda; else hi=lambda;
            const [u,v]=state.y,[p,q,s]=state.inverse;
            const slope=-(u*(p*u+q*v)+v*(q*u+s*v))/state.radius;
            const trial=lambda-(state.radius-clearance)/slope;
            lambda=trial>lo&&trial<hi?trial:(lo+hi)/2;
            state=solve(lambda);
        }
    }
    const [x,y]=state.y,[p,q,s]=state.inverse;
    const normal=active?[x/state.radius,y/state.radius]:[0,0];
    const normalForce=active?lambda*state.radius:0;
    const contactForce=[-normalForce*normal[0],-normalForce*normal[1]];
    const stationarity=[a*x+b*y+r0-contactForce[0],b*x+d*y+r1-contactForce[1]];
    const gap=clearance-state.radius;
    // -dy/dr on the fixed contact branch: remove its normal component.
    let response=[p,q,s];
    if(active) {
        // In 2D the admissible tangent is one-dimensional. This equivalent
        // form avoids squaring tiny compliances and then dividing: that can
        // underflow and silently restore a forbidden radial response.
        const t0=-normal[1],t1=normal[0],scale=Math.max(Math.abs(a+lambda),Math.abs(b),Math.abs(d+lambda));
        const denominator=t0*((a+lambda)/scale*t0+b/scale*t1)+t1*(b/scale*t0+(d+lambda)/scale*t1);
        const compliance=(1/scale)/denominator;
        if(!(denominator>0)||!(compliance>0)||!Number.isFinite(compliance))
            throw new RangeError('Unresolved local contact tangent');
        response=[t0*t0*compliance,t0*t1*compliance,t1*t1*compliance];
    }
    const energyChange=r0*x+r1*y+.5*(a*x*x+2*b*x*y+d*y*y);
    const residual=Math.hypot(...stationarity),complementarity=normalForce*Math.abs(gap);
    if(![...state.y,...response,...contactForce,normalForce,energyChange,residual,complementarity,gap].every(Number.isFinite))
        throw new RangeError('Nonfinite local contact response');
    return {offset:state.y,normal,normalForce,contactForce,gap,active,lambda,response,energyChange,
        stationarity,residual,complementarity,iterations,
        converged:residual<=forceTolerance && gap>=-gapTolerance &&
            (!active || Math.abs(gap)<=gapTolerance && lambda*Math.abs(gap)<=forceTolerance),
        scope:'local-quadratic-cell'};
}

/** Local envelope correction for E(z,y)=E0(z)+r(z).y+.5 y.K.y, with K
 * frozen and dr/dz=coupling. Eliminates only these TWO internal coordinates.
 * Results scatter into the existing cell band. No global Schur complement.
 * At contact onset the envelope is C1; response is the chosen branch tangent.
 */
export function condenseCompositeClearanceCell({cell,coupling,commonDofCount}) {
    if(!cell?.converged) throw new RangeError('A converged physical clearance cell is required');
    if(!Number.isInteger(commonDofCount)||commonDofCount<1) throw new RangeError('A positive local common DOF count is required');
    finite(coupling,2*commonDofCount,'common-relative coupling');
    const gradient=new Float64Array(commonDofCount),hessian=new Float64Array(commonDofCount**2);
    const [p,q,s]=cell.response,[x,y]=cell.offset;
    for(let i=0;i<commonDofCount;i++) {
        const a=coupling[2*i],b=coupling[2*i+1];gradient[i]=a*x+b*y;
        for(let j=0;j<=i;j++) {
            const c=coupling[2*j],d=coupling[2*j+1];
            hessian[i*commonDofCount+j]=hessian[j*commonDofCount+i]=-(a*(p*c+q*d)+b*(q*c+s*d));
        }
    }
    if(!gradient.every(Number.isFinite)||!hessian.every(Number.isFinite)) throw new RangeError('Nonfinite local envelope correction');
    return {energy:cell.energyChange,gradient,hessian,normalForce:cell.normalForce,eliminatedDofCount:2};
}
