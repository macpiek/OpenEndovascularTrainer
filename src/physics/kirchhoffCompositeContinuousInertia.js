import { sampleCompositeContinuousBasis } from './kirchhoffCompositeContinuousGeometry.js';

const finite = (x,name) => { if (!Number.isFinite(x)) throw new RangeError(`${name} must be finite`); return x; };
const positive = (x,name) => { if (!(finite(x,name)>0)) throw new RangeError(`${name} must be positive`); return x; };
const vector = (v,n,name) => { if(v?.length!==n)throw new RangeError(`${name} needs ${n} entries`);return Array.from(v,x=>finite(x,name)); };
const fractions = [.033765242898423975,.16939530676686776,.3806904069584015,.6193095930415985,.8306046932331322,.966234757101576];
const weights = [.08566224618958517,.1803807865240693,.23395696728634552,.23395696728634552,.1803807865240693,.08566224618958517];

function oldVelocity(controls, u) {
    return [0,1,2].map(k=>{
        const values=controls.map(v=>v[k]);
        for(let degree=5;degree>0;degree--)for(let i=0;i<degree;i++)values[i]=(1-u)*values[i]+u*values[i+1];
        return finite(values[0],'Old polynomial material velocity');
    });
}

/** Exact polynomial translational inertia for the shared C2 quintic position
 * field. The ALE physical velocity is N(q-q_old)/dt - (s_t/s_x) N_x q.
 * With affine maps/rates it has degree at most five; Gauss-6 integrates the
 * squared increment exactly on each OLD material-history polynomial piece.
 * Pieces retain their own intervals and discontinuities instead of averaging
 * old fields into two endpoint samples. No force or history transfer is implied.
 */
export function createCompositeContinuousMaterialInertiaEdge({ geometry, previousPositions, dt, tool } = {}) {
    const firstBasis=sampleCompositeContinuousBasis(geometry,0), n=firstBasis.weights.length, dofs=3*n,
        h=geometry.coordinates[1]-geometry.coordinates[0];
    positive(dt,'dt');
    if(typeof tool?.id!=='string'||!tool.id)throw new RangeError('An own physical tool id is required');
    if(previousPositions?.length!==n)throw new RangeError('Previous positions must follow the continuous local node support');
    const previous=previousPositions.map(v=>vector(v,3,'Own previous position')), map=tool.materialMap,
        dsDx=positive(map?.dsDx,'Material metric'), sStart=finite(map?.sStart,'Material start'), density=positive(tool.massPerMaterialLength,'Material density'),
        rates=typeof map.dsDt==='number'?[finite(map.dsDt,'Material rate'),map.dsDt]:vector(map.dsDt,2,'Material rates'),
        pieces=tool.oldVelocityPieces;
    if(tool.oldMaterialVelocities!==undefined||!Array.isArray(pieces)||!pieces.length)throw new RangeError('Continuous inertia requires explicit polynomial old material velocity pieces');
    let end=0;
    const owned=pieces.map(p=>{
        const interval=vector(p?.fractions,2,'Old velocity piece interval');
        if(interval[0]!==end||!(interval[1]>interval[0])||interval[1]>1)throw new RangeError('Old velocity pieces must cover [0,1] exactly without gaps or overlaps');
        let controls;
        if(p.interpretation==='quintic-bernstein-material-velocity'&&p.bernsteinVelocities?.length===6&&p.oldMaterialVelocities===undefined)
            controls=p.bernsteinVelocities.map(v=>vector(v,3,'Old material velocity control'));
        else if(p.oldMaterialVelocities?.length===2&&p.bernsteinVelocities===undefined&&(p.interpretation===undefined||p.interpretation==='physical-material-velocity')) {
            const endpoints=p.oldMaterialVelocities.map(v=>vector(v,3,'Old affine material velocity'));
            controls=Array.from({length:6},(_,j)=>endpoints[0].map((v,k)=>(1-j/5)*v+j/5*endpoints[1][k]));
        } else throw new RangeError('Each old velocity piece needs six explicit quintic Bernstein material velocity controls or a physical affine endpoint pair');
        end=interval[1];return {interval,controls};
    });
    if(end!==1)throw new RangeError('Old velocity pieces must cover [0,1] exactly');
    const prepared=[],hessian=new Float64Array(dofs*dofs),oldMomentum=[0,0,0];let oldKineticEnergy=0;
    for(const piece of owned)for(let i=0;i<6;i++) {
        const f=piece.interval[0]+fractions[i]*(piece.interval[1]-piece.interval[0]),basis=sampleCompositeContinuousBasis(geometry,f),
            u=-((1-f)*rates[0]+f*rates[1])/dsDx, coefficients=basis.weights.map((w,j)=>w/dt+u*basis.first[j]),
            massWeight=density*dsDx*h*(piece.interval[1]-piece.interval[0])*weights[i], old=oldVelocity(piece.controls,fractions[i]);
        if(!Number.isFinite(massWeight)||coefficients.some(v=>!Number.isFinite(v)))throw new RangeError('Nonfinite continuous inertia coefficients');
        old.forEach((v,k)=>{oldMomentum[k]+=massWeight*v;oldKineticEnergy+=.5*massWeight*v*v;});
        for(let j=0;j<n;j++)for(let l=0;l<n;l++)for(let k=0;k<3;k++)hessian[(3*j+k)*dofs+3*l+k]+=massWeight*coefficients[j]*coefficients[l];
        prepared.push({basis,fraction:f,u,coefficients,massWeight,old});
    }
    const mass=density*dsDx*h;
    if(![mass,oldKineticEnergy,...oldMomentum,...hessian].every(Number.isFinite))throw new RangeError('Nonfinite prepared continuous material inertia');
    const local={id:tool.id,mass,oldMomentum:oldMomentum.slice(),momentum:[0,0,0],samples:[]},
        output={scope:'prepared-one-material-continuous-quintic-inertia',tools:[local],mass,oldKineticEnergy,
            energy:NaN,kineticEnergy:NaN,gradient:new Float64Array(dofs),kineticGradient:new Float64Array(dofs),hessian:new Float64Array(dofs*dofs),
            momentumIncrement:new Float64Array(3),hessianValid:false,certified:false};
    local.samples=prepared.map(p=>({fraction:p.fraction,s:sStart+dsDx*h*p.fraction,u:p.u,massWeight:p.massWeight,
        coefficients:p.coefficients.slice(),oldMaterialVelocity:p.old.slice(),velocity:[NaN,NaN,NaN],velocityIncrement:[NaN,NaN,NaN]}));
    function invalidate() {
        output.hessianValid=false;output.energy=output.kineticEnergy=local.energy=local.kineticEnergy=NaN;
        for(const key of ['gradient','kineticGradient','hessian','momentumIncrement'])output[key].fill(NaN);
        local.momentum.fill(NaN);local.samples.forEach(s=>{s.velocity.fill(NaN);s.velocityIncrement.fill(NaN);});
    }
    function evaluate(positions,{order='full'}={}) {
        invalidate();
        try {
            if(!['full','gradient'].includes(order))throw new RangeError('Continuous inertia order must be full or gradient');
            if(positions?.length!==n)throw new RangeError('Current positions must follow the continuous local node support');
            const current=positions.map(v=>vector(v,3,'Own current position')),change=current.map((v,j)=>v.map((x,k)=>(x-previous[j][k])/dt));
            output.mass=local.mass=mass;output.oldKineticEnergy=oldKineticEnergy;local.oldMomentum=oldMomentum.slice();
            output.energy=output.kineticEnergy=0;output.gradient.fill(0);output.kineticGradient.fill(0);output.momentumIncrement.fill(0);local.momentum.fill(0);
            prepared.forEach((p,i)=>{
                const {basis:b,massWeight:m}=p,s=local.samples[i];
                s.fraction=p.fraction;s.s=sStart+dsDx*h*p.fraction;s.u=p.u;s.massWeight=m;s.coefficients=p.coefficients.slice();s.oldMaterialVelocity=p.old.slice();
                for(let k=0;k<3;k++) {
                    const anchor=current[b.anchor][k],anchorRate=change[b.anchor][k];
                    let qt=anchorRate,qx=0;
                    for(let j=0;j<n;j++)if(j!==b.anchor){qt+=b.weights[j]*(change[j][k]-anchorRate);qx+=b.first[j]*(current[j][k]-anchor);}
                    const velocity=qt+p.u*qx,increment=velocity-p.old[k];
                    s.velocity[k]=velocity;s.velocityIncrement[k]=increment;
                    output.energy+=.5*m*increment*increment;output.kineticEnergy+=.5*m*velocity*velocity;
                    local.momentum[k]+=m*velocity;output.momentumIncrement[k]+=m*increment;
                    for(let j=0;j<n;j++){output.gradient[3*j+k]+=m*p.coefficients[j]*increment;output.kineticGradient[3*j+k]+=m*p.coefficients[j]*velocity;}
                }
            });
            if(![output.energy,output.kineticEnergy,mass,oldKineticEnergy,...oldMomentum,...local.momentum,...output.gradient,...output.kineticGradient,...output.momentumIncrement,...hessian].every(Number.isFinite))
                throw new RangeError('Nonfinite continuous material inertia response');
            local.energy=output.energy;local.kineticEnergy=output.kineticEnergy;local.oldKineticEnergy=oldKineticEnergy;
            output.evaluationOrder=order;if(order==='full')output.hessian.set(hessian);output.hessianValid=order==='full';return output;
        } catch(error) {invalidate();throw error;}
    }
    invalidate();
    return Object.freeze({id:tool.id,dt,mass,coordinateLength:h,nodeIndices:geometry.nodeIndices,geometry,evaluate,
        scope:'prepared-one-material-continuous-quintic-inertia',includesAngularInertia:false,quadraturePoints:prepared.length,materialVelocityDegree:5});
}
