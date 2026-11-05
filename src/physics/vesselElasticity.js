import {BALLOON_CONTACT} from '../devices/balloonMechanics.js';
// Reduced elastic foundation, in the rod solver's force/mm units. These are
// deliberately conservative simulation parameters, not measured tissue data.
export const DEFAULT_VESSEL_ELASTICITY=Object.freeze({compliance:1e-7,maxDisplacement:.35});

export function validateVesselElasticity(model) {
    if(model==null)return;
    if(!Number.isFinite(model.compliance)||model.compliance<=0||
        !Number.isFinite(model.maxDisplacement)||model.maxDisplacement<=0)
        throw new RangeError('Positive finite vessel compliance and displacement limit required');
}

// A progressively stiffening wall: d=C*lambda/(1+C*lambda/limit).
// The tangent belongs in the KKT system; its intercept must be removed from
// the geometric gap so that effectiveGap still evaluates the nonlinear law.
export function vesselElasticResponse(reaction,compliance,limit) {
    const force=Math.max(0,reaction),x=compliance*force/limit,denominator=1+x;
    const displacement=limit*x/denominator,tangent=compliance/(denominator*denominator);
    const integral=x<1e-3?x*x*(.5+x*(-2/3+x*(.75-.8*x))):Math.log1p(x)-x/denominator;
    return {displacement,tangent,energy:limit*limit/compliance*integral};
}

export function applyVesselElasticity(row,model,radiusOrBody) {
    const radius=typeof radiusOrBody==='number'?radiusOrBody:radiusOrBody.radius;
    const compressionLimit=radiusOrBody?.balloonCompressionLimit??0;
    row.geometricGap=undefined;row.wallDisplacement=undefined;row.wallElasticEnergy=undefined;row.balloonCompression=undefined;
    if(!model||!row.vesselElasticContact)return;
    // Keep the tool axis on the blood side of the reference surface. Existing
    // exact triangle/segment crossing certificates remain valid.
    const wallModel=compressionLimit>0?BALLOON_CONTACT:model;
    const limit=Math.min(wallModel.maxWallDisplacement??wallModel.maxDisplacement,.7*(radius-compressionLimit));
    const response=vesselElasticResponse(row.multiplier,wallModel.wallCompliance??wallModel.compliance,limit);
    const membrane=compressionLimit>0?vesselElasticResponse(row.multiplier,BALLOON_CONTACT.membraneCompliance,compressionLimit):null;
    if(membrane){
        row.balloonCompression=membrane.displacement;
        response.tangent+=membrane.tangent;response.energy+=membrane.energy;
    }
    row.geometricGap=row.gap;
    row.compliance=response.tangent;
    row.gap+=response.displacement+(membrane?.displacement??0)-response.tangent*row.multiplier;
    row.wallDisplacement=response.displacement;
    row.wallElasticEnergy=response.energy;
}

// Only accepted equilibria are published. Coordinates remain tied to the
// reference wall; neither failed trials nor display interpolation move the BVH.
export function captureVesselElasticPatches(state,rows,outerMaterialAt) {
    const patches=[];
    for(const row of rows) {
        if(!(row.wallDisplacement>1e-7)||!row.vesselElasticContact)continue;
        const t=row.witness.t,e=row.edge,normal=[0,1,2].map(k=>row.jacobian[k]+row.jacobian[k+3]);
        const body=outerMaterialAt(state,e,t,row.witness.owner).body,radius=body.radius;
        const distance=row.geometricGap+radius;
        const center=normal.map((n,k)=>(state.origin?.[k]??0)+(1-t)*state.positions[e][k]+t*state.positions[e+1][k]-n*distance);
        patches.push({center,normal:normal.map(n=>-n),displacement:row.wallDisplacement,
            width:body.balloonCompressionLimit?Math.max(3,Math.min(10,1.5*radius)):Math.max(2,4*radius)});
    }
    return patches;
}
