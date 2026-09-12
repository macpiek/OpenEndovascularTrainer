import {sampleCompositeContinuousBasis} from './kirchhoffCompositeContinuousGeometry.js';
import {createCompositeSurfaceDifferentialArena as arena,compositeSurfaceDifferentialVectors as vectors} from './kirchhoffCompositeJointSurfaceMotion.js';
import {createCompositeStrainDifferentialTape} from './kirchhoffCompositeStrainDifferentialTape.js';

const plans=new WeakMap(),workspaces=new WeakMap();
const finite=(x,name)=>{if(!Number.isFinite(x))throw new RangeError(`${name} must be finite`);return x;};
const vector=(v,n,name)=>{if(v?.length!==n)throw new RangeError(`${name} needs ${n} entries`);return Array.from(v,x=>finite(x,name));};
const freeze=x=>{if(x&&typeof x==='object'){Object.values(x).forEach(freeze);Object.freeze(x);}return x;};
const dot=(a,b)=>a.reduce((s,v,k)=>s+v*b[k],0),cross=(a,b)=>[a[1]*b[2]-a[2]*b[1],a[2]*b[0]-a[0]*b[2],a[0]*b[1]-a[1]*b[0]],
    sub=(a,b)=>a.map((v,k)=>v-b[k]),unit=v=>{const l=Math.hypot(...v);if(!(l>1e-12))refine('degenerate-frame-tangent');return v.map(x=>x/l);};
function refine(reason,details={}){const error=new RangeError(reason);error.code='continuous-frame-refinement-required';error.details={reason,...details};throw error;}
function transport(d,a,b){const den=1+dot(a,b);if(!(den>1e-10))refine('antiparallel-frame-chart');const v=cross(a,b),w=cross(v,d),ww=cross(v,w);return d.map((x,k)=>x+w[k]+ww[k]/den);}
function phase(a,b,t){return Math.atan2(dot(t,cross(a,b)),dot(a,b));}

function slope(coordinates,index) {
    const n=coordinates.length;
    if(n===1)return {indices:[0],weights:[0]};
    if(n===2){const h=coordinates[1]-coordinates[0];return {indices:[0,1],weights:[-1/h,1/h]};}
    const indices=index===0?[0,1,2]:index===n-1?[n-3,n-2,n-1]:[index-1,index,index+1],
        origin=coordinates[indices[0]],h=coordinates[indices[2]]-origin,u=indices.map(j=>(coordinates[j]-origin)/h),at=(coordinates[index]-origin)/h;
    return {indices,weights:indices.map((_,j)=>{const others=[0,1,2].filter(k=>k!==j);return (2*at-u[others[0]]-u[others[1]])/((u[j]-u[others[0]])*(u[j]-u[others[1]]))/h;})};
}
function directorBasis(data,coordinate) {
    const {centers,slopes,region,angleIndices}=data,n=centers.length,size=angleIndices.length,
        weights=new Array(size).fill(0),first=new Array(size).fill(0),second=new Array(size).fill(0),
        add=(i,w,d=0,dd=0)=>{const j=angleIndices.indexOf(region[0]+i);if(j<0&&(w!==0||d!==0||dd!==0))throw new RangeError('Director support exceeds the compiled physical edge');if(j>=0){weights[j]+=w;first[j]+=d;second[j]+=dd;}};
    if(n===1)add(0,1);
    else if(coordinate<=centers[0]||coordinate>=centers[n-1]) {
        // A declared linear extension only inside the physical end half-edge.
        // It matches the same one-sided derivative at its endpoint sample.
        const i=coordinate<=centers[0]?0:n-1;add(i,1);slopes[i].indices.forEach((j,k)=>add(j,(coordinate-centers[i])*slopes[i].weights[k],slopes[i].weights[k]));
    } else {
        let i=0;while(coordinate>centers[i+1])i++;
        const h=centers[i+1]-centers[i],u=(coordinate-centers[i])/h,u2=u*u,u3=u2*u;
        add(i,2*u3-3*u2+1,(6*u2-6*u)/h,(12*u-6)/h/h);
        add(i+1,-2*u3+3*u2,(-6*u2+6*u)/h,(-12*u+6)/h/h);
        slopes[i].indices.forEach((j,k)=>add(j,h*(u3-2*u2+u)*slopes[i].weights[k],(3*u2-4*u+1)*slopes[i].weights[k],(6*u-4)/h*slopes[i].weights[k]));
        slopes[i+1].indices.forEach((j,k)=>add(j,h*(u3-u2)*slopes[i+1].weights[k],(3*u2-2*u)*slopes[i+1].weights[k],(6*u-2)/h*slopes[i+1].weights[k]));
    }
    for(const [row,total] of [[weights,1],[first,0],[second,0]])row[0]=total-row.slice(1).reduce((s,v)=>s+v,0);
    return {weights,first,second};
}

export function compositeContinuousFrameSupport(geometry,edge) {
    const own=geometry?.edges?.[edge];sampleCompositeContinuousBasis(own,.5);
    const region=own.region,angleIndices=Array.from({length:Math.min(region[1]-1,edge+2)-Math.max(region[0],edge-2)+1},(_,j)=>Math.max(region[0],edge-2)+j),
        positionIndices=[...new Set([...own.nodeIndices,...angleIndices.flatMap(i=>geometry.edges[i].nodeIndices)])].sort((a,b)=>a-b);
    return freeze({positionIndices,angleIndices});
}

/** C1 material director field on the SAME C2 position curve. Independent
 * unwrapped edge angles orient native physical samples before transporting
 * them to the continuous tangent. Cubic Hermite interpolation of those
 * physical vectors, followed by tangent projection, is objective and does not
 * depend on arbitrary reference-frame gauges. Shared slopes remove artificial
 * frame-rate seams; explicit physical interfaces retain their own traces.
 * This local chart cannot resolve a >=pi lifted twist between adjacent samples:
 * it requests refinement instead of replacing complete turns by their modulo.
 */
export function createCompositeContinuousFrame({geometry,edge,toolId,previousPositions,previousAngles,reference,referenceTwists}={}) {
    const own=geometry?.edges?.[edge];sampleCompositeContinuousBasis(own,.5);
    const x=geometry.coordinates,n=x.length,region=own.region;
    if(typeof toolId!=='string'||!toolId||previousPositions?.length!==n||previousAngles?.length!==n-1||reference?.length!==n-1||referenceTwists?.length!==n-2)
        throw new RangeError('Complete own geometry, reference frames, unwrapped angles and hinge lifts are required');
    const centers=Array.from({length:region[1]-region[0]},(_,j)=>{const i=region[0]+j;return x[i]+(x[i+1]-x[i])/2;}),slopes=centers.map((_,j)=>slope(centers,j)),
        {angleIndices,positionIndices}=compositeContinuousFrameSupport(geometry,edge),
        previous=positionIndices.map(i=>vector(previousPositions[i],3,'Own accepted position')),angles=angleIndices.map(i=>finite(previousAngles[i],'Own accepted unwrapped angle')),
        source=angleIndices.map(i=>{
            const f={tangent:vector(reference[i]?.tangent,3,'Own reference tangent'),director:vector(reference[i]?.director,3,'Own reference director')},
                t=unit(sub(previousPositions[i+1],previousPositions[i]));
            if(Math.hypot(...sub(t,f.tangent))>1e-10||Math.abs(dot(f.director,f.director)-1)>1e-10||Math.abs(dot(f.tangent,f.director))>1e-10)
                throw new RangeError('Own reference frame must belong to its accepted native physical edge');
            return f;
        }),twists=angleIndices.slice(0,-1).map((i,j)=>{
            const lift=finite(referenceTwists[i],'Own reference lift'),actual=phase(transport(source[j].director,source[j].tangent,source[j+1].tangent),source[j+1].director,source[j+1].tangent);
            if(Math.abs(Math.sin(lift-actual))>1e-10||Math.abs(Math.cos(lift-actual)-1)>1e-10)throw new RangeError('Own unwrapped reference lift disagrees with accepted physical frames');
            if(Math.abs(angles[j+1]-angles[j]+lift)>=Math.PI)refine('unresolved-accepted-spatial-winding',{toolId,edges:[i,i+1],twist:angles[j+1]-angles[j]+lift});
            return lift;
        });
    const configurationColumns=positionIndices.flatMap(node=>[0,1,2].map(component=>({kind:'position',toolId,node,component})))
        .concat(angleIndices.map(edge=>({kind:'angle',toolId,edge}))),N=configurationColumns.length;
    const handle=freeze({scope:'continuous-C1-physical-director-field',toolId,edge,positionNodeIndices:positionIndices,angleEdgeIndices:angleIndices,
        configurationColumns,configurationDofs:N,coordinateInterval:own.coordinates.slice(),contactCertified:false});
    plans.set(handle,{geometry,own,toolId,region,centers,slopes,angleIndices,positionIndices,previous,angles,source,twists,N});return handle;
}

export function createCompositeContinuousFrameWorkspace(frame=null) {
    const p=frame===null?null:plans.get(frame);if(frame!==null&&!p)throw new TypeError('Use a prepared continuous material frame');
    // One sequential arena per workspace, not one derivative arena per sample.
    const handle=Object.freeze({scope:'continuous-material-frame-scratch',configurationDofs:p?.N??null,shared:frame===null,
        get diagnostics(){const s=workspaces.get(handle);return {retainedBytes:(s.first?.data.buffer.byteLength??0)+(s.second?.data.buffer.byteLength??0)+(s.strainTape?.retainedBytes??0),
            nativeBuilds:s.nativeBuilds,samples:s.samples,arenaAllocations:s.arenaAllocations};}});
    workspaces.set(handle,{frame,first:p?arena(p.N+1,false,2048):null,second:null,strainTape:null,busy:false,nativeBuilds:0,samples:0,arenaAllocations:p?1:0});return handle;
}

function fieldArena(scratch,D,full) {
    const key=full?'second':'first',previous=scratch[key];
    if(previous?.dimension===D)return previous;
    const length=2048*(1+D+(full?D*D:0)),storage=previous&&previous.data.buffer.byteLength>=8*length?new Float64Array(previous.data.buffer):null;
    if(storage===null)scratch.arenaAllocations++;
    return scratch[key]=arena(D,full,2048,storage);
}

function strainTape(scratch,D) {
    const previous=scratch.strainTape;if(previous?.dimension===D)return previous;
    const tape=createCompositeStrainDifferentialTape(D,2048,previous?.storage);
    if(tape.storage!==previous?.storage)scratch.arenaAllocations++;
    return scratch.strainTape=tape;
}

/** Exact configuration derivatives of material Darboux strain. Spatial
 * derivatives are formed analytically before differentiation; no third-order
 * configuration tensor or numerical differences are needed for the Hessian.
 */
export function evaluateCompositeContinuousStrain(input,frame,workspace=createCompositeContinuousFrameWorkspace(frame)) {
    return evaluateContinuousField(input,frame,workspace,true);
}
export function evaluateCompositeContinuousStrains(input,frame,workspace=createCompositeContinuousFrameWorkspace(frame)) {
    if(!Array.isArray(input.coordinates)||!input.coordinates.length)throw new RangeError('A nonempty strain coordinate batch is required');
    return evaluateContinuousField(input,frame,workspace,true,input.coordinates);
}
/** Differentiate each scalar material energy directly. This retains the full
 * strain Hessian contribution while requiring one reverse sweep, instead of
 * recovering three strain Hessians and contracting them afterwards.
 */
export function evaluateCompositeContinuousElasticDensities(input,frame,workspace=createCompositeContinuousFrameWorkspace(frame)) {
    if(!Array.isArray(input.coordinates)||!input.coordinates.length||!Array.isArray(input.materials)||input.materials.length!==input.coordinates.length)
        throw new RangeError('Each elastic density coordinate needs an explicit material');
    const materials=input.materials.map(m=>({stiffness:vector(m?.stiffness,9,'Compiled material stiffness'),
        intrinsic:vector(m?.intrinsic,3,'Compiled intrinsic strain'),energyOffset:finite(m?.energyOffset,'Material energy offset')}));
    return evaluateContinuousField(input,frame,workspace,true,input.coordinates,materials);
}
export function evaluateCompositeContinuousFrame(input,frame,workspace=createCompositeContinuousFrameWorkspace(frame)) {
    return evaluateContinuousField(input,frame,workspace,false);
}
function evaluateContinuousField({positions,angles,coordinate,order='full',dsDx=1},frame,workspace,strainOnly,batch=null,materials=null) {
    const p=plans.get(frame),scratch=workspaces.get(workspace);
    if(!p||!scratch||(scratch.frame!==null&&scratch.frame!==frame)||scratch.busy)throw new TypeError('A free workspace for the prepared material frame is required');
    if(!['full','value'].includes(order))throw new RangeError('Continuous frame order must be full or value');
    const N=p.N,D=N+(strainOnly?0:1),full=order==='full';scratch.busy=true;
    try {
        if(positions?.length!==p.positionIndices.length||angles?.length!==p.angleIndices.length)throw new RangeError('Current pose must match the complete local physical support');
        const current=positions.map(v=>vector(v,3,'Current own position')),theta=vector(angles,angles.length,'Current own unwrapped angle');
        for(const x of batch??[coordinate]){finite(x,'Current physical coordinate');if(x<p.own.coordinates[0]||x>p.own.coordinates[1])throw new RangeError('Current coordinate leaves the prepared physical edge');}
        const a=strainOnly&&full?strainTape(scratch,D):fieldArena(scratch,D,full),v=vectors(a);a.reset();
        const q=current.map((point,j)=>point.map((x,k)=>a.variable(x,3*j+k))),spin=theta.map((x,j)=>a.variable(x,3*q.length+j)),nodeIndex=new Map(p.positionIndices.map((i,j)=>[i,j])),
            values=u=>u.map(at=>a.data[at]),jet=(value,first=0,second=0)=>{
                const at=a.constant(value);if(!strainOnly){a.data[at+1+N]=first;if(full)a.data[at+1+D+N*D+N]=second;}return at;
            };
        function geometryVector(e,basis,kind,coordinateDependent=false) {
            const anchor=q[nodeIndex.get(e.nodeIndices[basis.anchor])],total=kind==='weights'?1:0;
            let out=anchor.map(x=>a.scale(x,total));
            e.nodeIndices.forEach((node,j)=>{if(j===basis.anchor)return;
                const coefficient=coordinateDependent?jet(basis[kind][j],kind==='weights'?basis.first[j]:basis.second[j],kind==='weights'?basis.second[j]:basis.third[j]):a.constant(basis[kind][j]);
                out=v.add(out,v.times(v.sub(q[nodeIndex.get(node)],anchor),coefficient));
            });return out;
        }
        const native=[];scratch.nativeBuilds++;
        for(let j=0;j<p.angleIndices.length;j++) {
            const i=p.angleIndices[j],source=p.source[j],nativeT=v.unit(v.sub(q[nodeIndex.get(i+1)],q[nodeIndex.get(i)])),
                nativeD=v.transport(source.director.map(a.constant),source.tangent.map(a.constant),nativeT),
                curve=p.geometry.edges[i],curveT=v.unit(geometryVector(curve,sampleCompositeContinuousBasis(curve,.5),'first')),
                ref=v.transport(nativeD,nativeT,curveT),material=v.add(v.times(ref,a.cos(spin[j])),v.times(v.cross(curveT,ref),a.sin(spin[j])));
            native.push({nativeT,nativeD,material});
        }
        for(let j=0;j<native.length-1;j++) {
            const left=native[j],right=native[j+1],lt=values(left.nativeT),rt=values(right.nativeT),raw=phase(transport(values(left.nativeD),lt,rt),values(right.nativeD),rt),
                lift=raw+2*Math.PI*Math.round((p.twists[j]-raw)/(2*Math.PI)),twist=theta[j+1]-theta[j]+lift;
            if(Math.abs(twist)>=Math.PI)refine('unresolved-current-spatial-winding',{toolId:p.toolId,edges:[p.angleIndices[j],p.angleIndices[j+1]],twist});
        }
        function evaluateAt(coordinate,material=null) {
        scratch.samples++;
        const f=(coordinate-p.own.coordinates[0])/(p.own.coordinates[1]-p.own.coordinates[0]),basis=sampleCompositeContinuousBasis(p.own,f),
            center=strainOnly?null:geometryVector(p.own,basis,'weights',true),qx=geometryVector(p.own,basis,'first',true),tangent=v.unit(qx),
            shape=directorBasis(p,coordinate),anchor=native[0].material;
        let raw=anchor;
        for(let j=1;j<native.length;j++)raw=v.add(raw,v.times(v.sub(native[j].material,anchor),jet(shape.weights[j],shape.first[j],shape.second[j])));
        const projected=v.sub(raw,v.times(tangent,v.dot(tangent,raw)));
        if(!(Math.hypot(...values(projected))>1e-12))refine('singular-projected-director-chart',{toolId:p.toolId,edge:frame.edge});
        const d1=v.unit(projected),d2=v.cross(tangent,d1),triad=[d1,d2,tangent];
        if(strainOnly) {
            if(!(finite(dsDx,'Material reference metric')>0))throw new RangeError('Material reference metric must be positive');
            const normalizedDerivative=(value,direction,derivative)=>v.times(v.sub(derivative,v.times(direction,v.dot(direction,derivative))),a.reciprocal(a.sqrt(v.dot(value,value)))),
                qxx=geometryVector(p.own,basis,'second'),tx=normalizedDerivative(qx,tangent,qxx);
            let rawX=[0,0,0].map(a.constant);
            for(let j=1;j<native.length;j++)rawX=v.add(rawX,v.times(v.sub(native[j].material,anchor),a.constant(shape.first[j])));
            const projectedX=v.sub(v.sub(rawX,v.times(tx,v.dot(tangent,raw))),v.times(tangent,a.add(v.dot(tx,raw),v.dot(tangent,rawX)))),
                // For a right-handed orthonormal triad: k1=-d2.t_x,
                // k2=d1.t_x, tau=d2.d1_x. Since d2 is perpendicular
                // to d1, its contraction removes the normalization term.
                strain=[a.scale(v.dot(d2,tx),-1/dsDx),a.scale(v.dot(d1,tx),1/dsDx),
                    a.scale(a.mul(v.dot(d2,projectedX),a.reciprocal(a.sqrt(v.dot(projected,projected)))),1/dsDx)];
            if(material!==null) {
                const error=strain.map((at,j)=>a.sub(at,a.constant(material.intrinsic[j]))),K=material.stiffness,
                    moments=error.map((_,i)=>a.add(a.add(a.scale(error[0],K[3*i]),a.scale(error[1],K[3*i+1])),a.scale(error[2],K[3*i+2]))),
                    energy=a.add(a.scale(v.dot(error,moments),.5),a.constant(material.energyOffset));
                if(!a.finite())throw new RangeError('Nonfinite continuous elastic density');
                return {scope:'continuous-material-elastic-density',energy:a.data[energy],gradient:Float64Array.from(a.data.subarray(energy+1,energy+1+N)),
                    hessian:full?a.hessians([energy]):null,hessianValid:full,workspaceBytes:a.retainedBytes??a.data.buffer.byteLength};
            }
            if(!a.finite())throw new RangeError('Nonfinite continuous material strain derivatives');
            return {scope:'continuous-material-Darboux-strain',configurationColumns:frame.configurationColumns,configurationDofs:N,
                strain:values(strain),jacobian:Float64Array.from(strain.flatMap(at=>Array.from({length:N},(_,j)=>a.data[at+1+j]))),
                hessian:full?a.hessians(strain):null,
                hessianValid:full,order,coordinate,dsDx,derivativeCoordinates:'configuration-only',workspaceBytes:a.retainedBytes??a.data.buffer.byteLength,operations:a.used()};
        }
        if(!a.finite())throw new RangeError('Nonfinite continuous material frame derivatives');
        const derivative=(at,j)=>a.data[at+1+j],second=(at,i,j)=>a.data[at+1+D+i*D+j],
            angular=new Float64Array(3*D),angularDerivative=full?new Float64Array(3*D*D):null;
        for(let j=0;j<D;j++)for(const direction of triad) {
            const d=values(direction),dd=direction.map(at=>derivative(at,j)),omega=cross(d,dd);
            omega.forEach((x,k)=>angular[k*D+j]+=.5*x);
            if(full)for(let l=0;l<D;l++) {
                const first=cross(direction.map(at=>derivative(at,l)),dd),last=cross(d,direction.map(at=>second(at,j,l)));
                first.forEach((x,k)=>angularDerivative[(k*D+j)*D+l]+=.5*(x+last[k]));
            }
        }
        return {scope:frame.scope,toolId:p.toolId,edge:frame.edge,configurationColumns:frame.configurationColumns,configurationDofs:N,derivativeDofs:D,
            position:values(center),positionDx:values(qx),directors:triad.map(values),
            positionJacobian:Float64Array.from(center.flatMap(at=>Array.from({length:D},(_,j)=>derivative(at,j)))),
            positionHessian:full?Float64Array.from(center.flatMap(at=>Array.from({length:D*D},(_,j)=>second(at,Math.floor(j/D),j%D)))):null,
            frameJacobian:Float64Array.from(triad.flat().flatMap(at=>Array.from({length:D},(_,j)=>derivative(at,j)))),
            angularRateMap:angular,angularRateMapDerivative:angularDerivative,
            previousPositions:p.previous.map(v=>v.slice()),previousAngles:p.angles.slice(),
            order,forceKinematicsValid:true,derivativeValid:full,contactCertified:false,finiteStepSlipKnown:false,
            workspaceBytes:a.data.buffer.byteLength,operations:a.used(),spatialField:'C1-normalized-Hermite-physical-directors-on-C2-geometry'};
        }
        if(batch!==null){const checkpoint=a.checkpoint();return batch.map((x,j)=>{a.rewind(checkpoint);return evaluateAt(x,materials?.[j]??null);});}
        return evaluateAt(coordinate);
    } finally {scratch.busy=false;}
}
