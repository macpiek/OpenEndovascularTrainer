import {createCompositeContinuousFrameWorkspace} from './kirchhoffCompositeContinuousFrame.js';
import {evaluateCompositeContinuousSurfaceMotion} from './kirchhoffCompositeContinuousSurfaceMotion.js';

const plans=new WeakMap(),dot=(a,b)=>a.reduce((sum,v,k)=>sum+v*b[k],0),
    cross=(a,b)=>[a[1]*b[2]-a[2]*b[1],a[2]*b[0]-a[0]*b[2],a[0]*b[1]-a[1]*b[0]],
    vector=(v,n,name)=>{if(v?.length!==n||!Array.from(v).every(Number.isFinite))throw new RangeError(`${name} needs ${n} finite entries`);return Array.from(v);},
    close=(a,b)=>Math.abs(a-b)<=256*Number.EPSILON*Math.max(1,Math.abs(a),Math.abs(b));

/** Physical tangential operator at one original-provider C2 wall sample.
 * The world seed is fixed for the lifetime of this local tangent chart. Its
 * projected axes vary with the actual wall normal, and their derivatives are
 * included. No surface certification or finite material-slip claim is made.
 */
export function createCompositeContinuousWallSurface({frame,site,seedAxis,wallVelocity=[0,0,0]}={}) {
    const workspace=createCompositeContinuousFrameWorkspace(frame),nodes=site?.nodeIndices?.slice(),weights=vector(site?.positionWeights,nodes?.length,'C2 sample weights');
    if(site?.owner!==frame.toolId||site?.edge!==frame.edge||!Number.isFinite(site?.fraction)||site.fraction<0||site.fraction>1||
        !nodes?.length||new Set(nodes).size!==nodes.length||nodes.some(node=>!frame.positionNodeIndices.includes(node))||!close(weights.reduce((s,v)=>s+v,0),1))
        throw new RangeError('The surface and wall sample must share their complete own C2 edge');
    let seed;
    if(seedAxis===undefined) {
        const normal=vector(site.normal,3,'Initial wall normal');let axis=0;
        for(let k=1;k<3;k++)if(Math.abs(normal[k])<Math.abs(normal[axis]))axis=k;
        seed=[0,0,0];seed[axis]=1;
    } else seed=vector(seedAxis,3,'Fixed world tangent seed');
    const seedLength=Math.hypot(...seed);if(!(seedLength>1e-12))throw new RangeError('A nonzero fixed world tangent seed is required');
    seed=seed.map(v=>v/seedLength);
    const N=frame.configurationDofs,positionMap=new Float64Array(3*N);
    nodes.forEach((node,i)=>{const j=frame.positionNodeIndices.indexOf(node);for(let k=0;k<3;k++)positionMap[k*N+3*j+k]=weights[i];});
    const handle=Object.freeze({scope:'continuous-C2-wall-tangential-operator',toolId:frame.toolId,edge:frame.edge,
        configurationDofs:N,configurationColumns:frame.configurationColumns,finiteStepSlipKnown:false});
    plans.set(handle,{frame,site,nodes,weights,positionMap,seed,workspace,velocity:vector(wallVelocity,3,'Explicit fixed wall velocity'),
        fraction:site.fraction,coordinate:frame.coordinateInterval[0]+site.fraction*(frame.coordinateInterval[1]-frame.coordinateInterval[0])});
    return handle;
}

/** Chain the provider's exact branch derivatives through the contact witness
 * w=x-d*n and axes a=normalize(seed-n*(n.seed)), b=n cross a.
 * B[j,c] maps every physical grid/spin rate to tangential relative velocity;
 * the mechanical residual uses -B*Ft. DB[(2*j+c),l] includes moving witness,
 * normal and axes, including remote material-director support columns.
 * slipIncrement is dt times an implicit backward-Euler rate surrogate. It is
 * deliberately not identified as a finite same-material contact displacement.
 */
export function evaluateCompositeContinuousWallSurface({positions,angles,materialMap,dt,order='full'},handle) {
    const p=plans.get(handle);if(!p)throw new TypeError('Use a compiled continuous wall surface');
    if(!['full','gradient'].includes(order))throw new RangeError('Wall surface order must be full or gradient');
    const {frame,site,N=frame.configurationDofs}=p,full=order==='full';
    if(!site.supported||(full&&!site.hessianValid))throw new RangeError('A current supported wall query with matching derivative order is required');
    if(site.owner!==frame.toolId||site.edge!==frame.edge||site.fraction!==p.fraction||site.nodeIndices?.length!==p.nodes.length||
        site.nodeIndices.some((node,j)=>node!==p.nodes[j])||site.positionWeights?.length!==p.weights.length||site.positionWeights.some((v,j)=>v!==p.weights[j]))
        throw new RangeError('The compiled C2 wall sample support changed');
    if(positions?.length!==frame.positionNodeIndices.length)throw new RangeError('Current positions must cover every physical frame support column');
    positions.forEach(v=>vector(v,3,'Current physical position'));
    const center=vector(site.position,3,'Current wall query center'),normal=vector(site.normal,3,'Current wall normal'),point=vector(site.closestPoint,3,'Current wall witness'),d=site.signedDistance;
    if(!Number.isFinite(d)||!close(dot(normal,normal),1))throw new RangeError('A finite signed wall distance and unit normal are required');
    for(let k=0;k<3;k++) {
        const anchor=positions[frame.positionNodeIndices.indexOf(p.nodes[0])][k],expected=anchor+p.nodes.slice(1).reduce((sum,node,j)=>
            sum+p.weights[j+1]*(positions[frame.positionNodeIndices.indexOf(node)][k]-anchor),0);
        if(!close(center[k],expected))throw new RangeError('Wall query is stale for the current C2 configuration');
        if(!close(point[k],center[k]-d*normal[k]))throw new RangeError('Wall witness does not match its signed-distance normal branch');
    }
    const seedDot=dot(p.seed,normal),projected=p.seed.map((v,k)=>v-seedDot*normal[k]),projectedLength=Math.hypot(...projected);
    if(!(projectedLength>1e-8))throw new RangeError('Wall tangent chart requires a different fixed world seed');
    const a=projected.map(v=>v/projectedLength),b=cross(normal,a),axes=[a,b],Q=10,
        queryConfigurationDerivative=full?new Float64Array(Q*N):null;
    if(full) {
        const G=vector(site.pointGapGradient,3,'Current world gap gradient'),Dn=vector(site.pointNormalDerivative,9,'Current world normal derivative');
        for(let l=0;l<N;l++) {
            const dx=[0,1,2].map(k=>p.positionMap[k*N+l]),dd=dot(G,dx),dn=[0,1,2].map(k=>dot(Dn.slice(3*k,3*k+3),dx)),
                dw=dx.map((v,k)=>v-normal[k]*dd-d*dn[k]),ds=dot(p.seed,dn),dp=dn.map((v,k)=>-seedDot*v-normal[k]*ds),
                projection=dot(a,dp),da=dp.map((v,k)=>(v-a[k]*projection)/projectedLength),
                left=cross(dn,a),right=cross(normal,da),db=left.map((v,k)=>v+right[k]);
            for(let k=0;k<3;k++) {
                queryConfigurationDerivative[(1+k)*N+l]=dw[k];
                queryConfigurationDerivative[(4+k)*N+l]=da[k];
                queryConfigurationDerivative[(7+k)*N+l]=db[k];
            }
        }
    }
    const motion=evaluateCompositeContinuousSurfaceMotion({positions,angles,coordinate:p.coordinate,materialMap,dt,rateMode:'backward-euler-grid',
        contact:{point,axes},wall:{velocity:p.velocity},order:full?'full':'value'},frame,p.workspace),
        configurationDerivative=full?new Float64Array(2*N*N):null,rateConfigurationDerivative=full?new Float64Array(2*N):null,
        slipDerivative=full?new Float64Array(2*N):null;
    if(full) {
        for(let row=0;row<2*N;row++)for(let l=0;l<N;l++) {
            let value=motion.configurationDerivative[row*N+l];
            for(let q=0;q<Q;q++)value+=motion.queryDerivative[row*Q+q]*queryConfigurationDerivative[q*N+l];
            configurationDerivative[row*N+l]=value;
        }
        for(let c=0;c<2;c++)for(let l=0;l<N;l++) {
            let value=motion.rateConfigurationDerivative[c*N+l];
            for(let q=0;q<Q;q++)value+=motion.rateQueryDerivative[c*Q+q]*queryConfigurationDerivative[q*N+l];
            rateConfigurationDerivative[c*N+l]=value;slipDerivative[c*N+l]=dt*value;
        }
    }
    const slipIncrement=Float64Array.from(motion.slipRate,v=>dt*v);
    for(const array of [slipIncrement,...(full?[configurationDerivative,rateConfigurationDerivative,slipDerivative,queryConfigurationDerivative]:[])])
        if(!array.every(Number.isFinite))throw new RangeError('Nonfinite wall surface chain derivative');
    return {scope:handle.scope,toolId:frame.toolId,configurationColumns:frame.configurationColumns,configurationDofs:N,
        forceMap:motion.forceMap,configurationDerivative,slipRate:motion.slipRate,rateConfigurationDerivative,slipIncrement,slipDerivative,
        contactPoint:point,axes,normal,queryConfigurationDerivative,position:motion.position,rates:motion.rates,prescribedRate:motion.prescribedRate,
        surfaceVelocity:motion.surfaceVelocity,omega:motion.omega,materialLabel:motion.materialLabel,feed:motion.feed,
        forceMapValid:true,derivativeValid:full,finiteStepSlipKnown:false,slipModel:'implicit-backward-euler-surface-rate',
        contactCertified:false,rateMode:'backward-euler-grid',order};
}
