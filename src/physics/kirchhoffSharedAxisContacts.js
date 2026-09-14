import {sharedAxisOuterMaterialAt} from './kirchhoffSharedAxisNative.js';
import { createSharedAxisVesselDiscovery } from './kirchhoffSharedAxisVesselWitnesses.js';

const inactive=()=>({gap:1,jacobian:[0,0,0,0,0,0]});
const xyz=p=>Array.isArray(p)?p:[p.x,p.y,p.z];

/** The open introducer's radial lumen. There is no axial/friction equation.
 * Each spatial node is sampled once (the outgoing edge's first endpoint is
 * already the preceding edge's last endpoint). Vessel queries are shielded by the
 * material sheath interval, as in the existing application's collision mask.
 */
export function createSharedAxisContacts({sheath,contactField=null,localCoordinates=false}) {
    const worldStart=xyz(sheath.start),end=xyz(sheath.end),delta=end.map((v,i)=>v-worldStart[i]);
    const origin=localCoordinates?worldStart:[0,0,0],start=worldStart.map((v,i)=>v-origin[i]);
    const length=Math.hypot(...delta),axis=delta.map(v=>v/length),extension=sheath.proximalExtension??40;
    if (!(length>0) || !(sheath.innerRadius>0) || !(extension>=10)) throw new RangeError('Invalid shared axis introducer');
    const wallSamples=[1].map(endpoint=>({state,a,b,edge,radius,coordinateA,coordinateB,needHessian=true,sampleT})=>{
        if(coordinateA>=length)return inactive();
        const outletT=Math.min(1,(length-coordinateA)/(coordinateB-coordinateA));
        if(sampleT!==undefined&&sampleT>=outletT)return inactive();
        const t=Math.min(sampleT??1,outletT);
        if(state?.materials&&Number.isInteger(edge))radius=sharedAxisOuterMaterialAt(state,edge,t).body.radius;
        const localStart=state?.origin?worldStart.map((v,k)=>v-state.origin[k]):start;
        const p=a.map((v,k)=>(1-t)*v+t*b[k]),relative=p.map((v,i)=>v-localStart[i]),along=relative.reduce((v,x,i)=>v+x*axis[i],0);
        const radial=relative.map((v,i)=>v-along*axis[i]),rho=Math.hypot(...radial),clearance=sheath.innerRadius-radius;
        if(clearance<=0)throw new RangeError('Tool does not fit the introducer');
        if(rho<1e-10)return {gap:clearance,jacobian:[0,0,0,0,0,0]};
        const unit=radial.map(v=>v/rho),jacobian=[0,0,0,0,0,0],hessian=needHessian?new Float64Array(36):undefined;
        for(let i=0;i<6;i++) {
            const wi=i<3?1-t:t;jacobian[i]=-wi*unit[i%3];
            if(needHessian)for(let j=0;j<6;j++)hessian[i*6+j]=-wi*(j<3?1-t:t)*((i%3===j%3?1:0)-axis[i%3]*axis[j%3]-unit[i%3]*unit[j%3])/rho;
        }
        return {gap:clearance-rho,jacobian,hessian};
    });
    wallSamples[0].sharedAxisSheath=true;
    wallSamples[0].contactOutputOwned=true;
    if(contactField) wallSamples.push(createSharedAxisVesselDiscovery(contactField,length));
    return {rebaseNearTips:localCoordinates,startCoordinate:-extension,boundaryCoordinates:[0,length],wallSamples,length,origin,
        samplePosition:coordinate=>start.map((v,i)=>v+coordinate*axis[i])};
}
