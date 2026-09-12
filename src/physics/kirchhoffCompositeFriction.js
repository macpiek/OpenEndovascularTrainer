import {projectKirchhoffSurfaceFriction} from './kirchhoffSurfaceFriction.js';
import {evaluateKirchhoffContinuousFrictionKKT} from './kirchhoffContinuousFrictionKKT.js';

function vector(value,n,name) {
    if(!value||value.length!==n||!value.every(Number.isFinite)) throw new TypeError(`${name} requires ${n} finite values`);
}

/** Derivative of Euclidean projection onto the fixed normal-force ellipse.
 * Outside a smooth ellipse only its tangent responds. The curvature form
 * avoids subtracting two nearly equal compliances or squaring tiny ones.
 */
function projectionTangent(z,p,axes) {
    const [a,b]=axes;
    if(a===0||b===0) return [a>0&&Math.abs(z[0])<=a?1:0,0,b>0&&Math.abs(z[1])<=b?1:0];
    if(Math.hypot(z[0]/a,z[1]/b)<=1)return [1,0,1];
    const scale=Math.max(a,b),ax=a/scale,by=b/scale,x=p[0]/scale,y=p[1]/scale;
    const nx=x/ax/ax,ny=y/by/by,normalLength=Math.hypot(nx,ny);
    if(!(normalLength>0)||!Number.isFinite(normalLength)) throw new RangeError('Unresolved ellipse normal');
    const t0=-ny/normalLength,t1=nx/normalLength;
    const curvature=(t0/ax)**2/normalLength+(t1/by)**2/normalLength;
    const distance=Math.hypot((z[0]-p[0])/scale,(z[1]-p[1])/scale);
    const denominator=1+distance*curvature;
    if(!(denominator>=1)||!Number.isFinite(denominator)) throw new RangeError('Unresolved friction projection tangent');
    return [t0*t0/denominator,t0*t1/denominator,t1*t1/denominator];
}

/** Fixed-load augmented Coulomb operator for a LOCAL contact patch.
 * traction is physical tangential force; slip is MATERIAL surface displacement
 * over the step, in two orthonormal tangent coordinates (both in length units).
 * Thus jacobian includes translational advection and each tool's independent
 * spin/lever arm. Neither centerline displacement alone nor a shared spin is
 * a substitute. normalForce must be the physical unilateral reaction.
 *
 * z=traction-penalty*slip, trialTraction=projection_ellipse(z).
 * Exact gradient at frozen normal load, tangent basis and affine slip map is
 * -J^T trialTraction. The PSD tangent is penalty*J^T DP J. No smoothing of
 * stick/slide or fictitious friction at zero normal load is introduced.
 * This is an inner operator: independently check original KKT and the entire
 * common force residual after updating normal and tangential reactions.
 */
export function evaluateCompositeFriction({traction,slip,jacobian,dofCount,normalForce,mu,penalty}) {
    vector(traction,2,'traction');vector(slip,2,'material slip');vector(mu,2,'friction coefficients');
    if(!Number.isInteger(dofCount)||dofCount<1)throw new RangeError('A local DOF count is required');
    vector(jacobian,2*dofCount,'row-major slip Jacobian');
    if(!Number.isFinite(normalForce)||normalForce<0||mu.some(x=>x<0)||!Number.isFinite(penalty)||penalty<=0)
        throw new RangeError('Physical normal force, nonnegative coefficients and positive penalty are required');
    const z=traction.map((v,i)=>v-penalty*slip[i]);
    vector(z,2,'trial force');
    const projection=projectKirchhoffSurfaceFriction(z,normalForce,mu),p=projection.lambda;
    const response=projectionTangent(z,p,projection.axes),[r00,r01,r11]=response;
    const gradient=new Float64Array(dofCount),hessian=new Float64Array(dofCount*dofCount);
    let energy=0;
    for(let i=0;i<2;i++) energy+=(p[i]-traction[i])/penalty*(p[i]/2+traction[i]/2)+(z[i]-p[i])/penalty*p[i];
    for(let i=0;i<dofCount;i++) {
        const a=jacobian[i],b=jacobian[dofCount+i];gradient[i]=-a*p[0]-b*p[1];
        for(let j=0;j<=i;j++) {
            const c=jacobian[j],d=jacobian[dofCount+j];
            hessian[i*dofCount+j]=hessian[j*dofCount+i]=penalty*(a*(r00*c+r01*d)+b*(r01*c+r11*d));
        }
    }
    if(!Number.isFinite(energy)||!gradient.every(Number.isFinite)||!hessian.every(Number.isFinite))throw new RangeError('Nonfinite friction operator');
    return {energy,gradient,hessian,trialTraction:p,projectionTangent:response,normalForce,
        scope:'fixed-physical-load-local-friction-operator'};
}

/** Original continuous maximum-dissipation and cone conditions, independent
 * of the augmented penalty. The existing physics residual is reused with
 * separate displacement, force-cone and dissipated-work tolerances.
 */
export function measureCompositeFriction({traction,slip,normalForce,mu,slipTolerance,coneTolerance,workTolerance}) {
    for(const value of [slipTolerance,coneTolerance,workTolerance])
        if(!Number.isFinite(value)||value<=0)throw new RangeError('Explicit positive physical tolerances are required');
    const kkt=evaluateKirchhoffContinuousFrictionKKT(traction,slip,normalForce,mu);
    const work=traction[0]*slip[0]+traction[1]*slip[1];
    const minimumWork=-normalForce*Math.hypot(mu[0]*slip[0],mu[1]*slip[1]);
    const workGap=Math.abs(work-minimumWork);
    return {work,minimumWork,workGap,slipResidual:kkt.residualMm,coneViolation:kkt.coneViolation,
        converged:Number.isFinite(workGap)&&kkt.residualMm<=slipTolerance&&kkt.coneViolation<=coneTolerance&&workGap<=workTolerance,
        scope:'original-local-Coulomb-KKT'};
}

const equationWorkspaces=new WeakMap();
const equationArrays=['residual','slipJacobian','tractionJacobian','normalDerivative','projectedForce','trialForce'];
function invalidateEquation(out,reason) {
    out.valid=out.operatorReady=false;out.certified=false;out.reason=reason;out.branch=null;
    out.normalForce=out.projectionLoad=out.penalty=NaN;
    for(const key of equationArrays)out[key].fill(NaN);
    return out;
}

/** Owned/reused two-coordinate constitutive equation buffers. No multiplier
 * history, physical cone/work certificate or complete solve is stored here.
 */
export function createCompositeFrictionEquationWorkspace() {
    const out={residual:new Float64Array(2),slipJacobian:new Float64Array(4),tractionJacobian:new Float64Array(4),
        normalDerivative:new Float64Array(2),projectedForce:new Float64Array(2),trialForce:new Float64Array(2),
        unit:'mm',scope:'private-normal-load-local-friction-nonlinear-equations'};
    equationWorkspaces.set(out,{projection:{lambda:out.projectedForce,axes:new Float64Array(2)}});
    return invalidateEquation(out,'not-evaluated');
}

/** r=(Ft-P_{max(Fn,0)*ellipse(mu)}(Ft-k*slip))/k, in mm.
 * dR/dslip=DPz, dR/dFt=(I-DPz)/k, dR/dFn=-DPFn/k.
 * k is the positive numerical N/mm penalty, not physical friction stiffness.
 *
 * Signed finite Fn is ONLY a private Newton extension. Inputs/Ft/history are
 * never projected in place; only the projection argument uses max(Fn,0).
 * Physical acceptance still requires literal Fn>=0 and the original KKT/work
 * gates in measureCompositeFriction. A small equation residual is not that
 * certificate, particularly at disabled axes or zero load.
 *
 * At Fn>0 use homogeneity DPFn=(P-DPz*z)/Fn with the existing fixed-load
 * tangent selection (interior at the stick boundary). At Fn<0 both projection
 * derivatives vanish. At Fn=0 use the right load derivative, the ellipse
 * support point mu_i²*z_i/|mu*z|; at z=0 choose the explicit zero apex member.
 * Zero coefficients disable their axes exactly, including the right limit.
 */
export function evaluateCompositeFrictionEquation(input,out) {
    const workspace=equationWorkspaces.get(out);
    if(!workspace)throw new TypeError('Use a prepared friction equation workspace');
    invalidateEquation(out,'not-evaluated');
    try {
        if(!input||typeof input!=='object')throw new TypeError('Explicit friction equation inputs are required');
        const {traction,slip,normalForce,mu,penalty}=input;
        vector(traction,2,'traction');vector(slip,2,'material slip');vector(mu,2,'friction coefficients');
        if(!Number.isFinite(normalForce)||mu.some(v=>v<0)||!Number.isFinite(penalty)||penalty<=0)
            throw new RangeError('Finite private normal force, nonnegative coefficients and positive penalty are required');
        const z=out.trialForce;
        for(let i=0;i<2;i++)z[i]=traction[i]-penalty*slip[i];
        vector(z,2,'trial force');
        const load=Math.max(normalForce,0),projection=projectKirchhoffSurfaceFriction(z,load,mu,workspace.projection),p=projection.lambda;
        let r00=0,r01=0,r11=0,dn0=0,dn1=0,branch;
        if(normalForce>0) {
            if(projection.axes.some((v,i)=>v===0&&mu[i]>0))throw new RangeError('Unresolved nonzero friction ellipse radius');
            [r00,r01,r11]=projectionTangent(z,p,projection.axes);
            dn0=(p[0]-r00*z[0]-r01*z[1])/normalForce;
            dn1=(p[1]-r01*z[0]-r11*z[1])/normalForce;
            // A disabled axis has identically zero projection, not a small
            // coefficient approximation or a tolerance-based force clamp.
            if(mu[0]===0)dn0=0;if(mu[1]===0)dn1=0;
            branch=projection.projected?'positive-load-projected':'positive-load-stick-or-boundary';
        } else if(normalForce===0) {
            const muScale=Math.max(...mu),zScale=Math.max(Math.abs(z[0]),Math.abs(z[1]));
            if(muScale>0&&zScale>0) {
                const w0=(mu[0]/muScale)*(z[0]/zScale),w1=(mu[1]/muScale)*(z[1]/zScale),length=Math.hypot(w0,w1);
                if(length>0){dn0=mu[0]*(w0/length);dn1=mu[1]*(w1/length);}
                else if(mu.some((v,i)=>v>0&&z[i]!==0))throw new RangeError('Unresolved zero-load friction support direction');
            }
            branch=zScale===0?'zero-load-apex':'zero-load-right-derivative';
        } else branch='negative-load-private-extension';
        out.slipJacobian.set([r00,r01,r01,r11]);
        out.tractionJacobian.set([(1-r00)/penalty,-r01/penalty,-r01/penalty,(1-r11)/penalty]);
        out.normalDerivative.set([dn0===0?0:-dn0/penalty,dn1===0?0:-dn1/penalty]);
        for(let i=0;i<2;i++)out.residual[i]=(traction[i]-p[i])/penalty;
        if(equationArrays.some(key=>!out[key].every(Number.isFinite)))throw new RangeError('Nonfinite friction equation or derivative');
        out.normalForce=normalForce;out.projectionLoad=load;out.penalty=penalty;out.branch=branch;
        out.valid=out.operatorReady=true;out.reason=null;
        return out;
    } catch(error) {invalidateEquation(out,error.message);throw error;}
}
