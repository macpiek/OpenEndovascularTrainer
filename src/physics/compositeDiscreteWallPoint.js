import {createContactResult} from './collision/vesselContactField.js';
import {initializeCompositeWallDifferentialRow,captureCompositeWallDifferentialContact,
    createCompositeWallDifferentialWorkspace,differentiateCompositeWallRow} from './kirchhoffCompositeWallDifferentialRows.js';

/** Fixed affine material site on one native chord. Its fraction never follows
 * the detector's minimum sample. This helper supplies local differentials,
 * not contact multipliers, capsule coverage or a physical-step certificate.
 */
export function createCompositeDiscreteWallPoint({fraction,edge=0,dofs=[0,1,2,3,4,5]}) {
    if(!Number.isFinite(fraction)||fraction<0||fraction>1||!Number.isInteger(edge)||edge<0||
        dofs?.length!==6||!Array.from(dofs).every(x=>Number.isInteger(x)&&x>=0))throw new RangeError('Valid fixed fraction, edge and six endpoint DOFs required');
    const localLayout={positions:[0,3]},row=initializeCompositeWallDifferentialRow({edge:0,included:true,role:'material-point'},localLayout);
    row.edge=edge;row.dofs=Int32Array.from(dofs);row.t=fraction;row.normal=new Float64Array(3);row.closestPoint=new Float64Array(3);row.point=new Float64Array(3);
    const local=initializeCompositeWallDifferentialRow({edge:0,included:true,role:'proximal',t:0},localLayout);
    local.normal=new Float64Array(3);local.closestPoint=new Float64Array(3);
    const contact=createContactResult(),scratch=createCompositeWallDifferentialWorkspace(),weights=[1-fraction,fraction];
    const invalidate=()=>{
        row.derivativeUnavailable=true;row.derivativeReason='not-evaluated';row.derivativeSource=null;
        row.gapJacobian.fill(NaN);row.forceColumn.fill(NaN);row.normalDerivative.fill(NaN);
    };
    function refresh({field,positions,radius,consumeQuery}) {
        invalidate();
        if(typeof field?.queryCapsuleCoordinates!=='function'||positions?.length!==2||positions.some(p=>p?.length!==3||!Array.from(p).every(Number.isFinite))||
            !Number.isFinite(radius)||radius<0||consumeQuery!==undefined&&typeof consumeQuery!=='function')throw new RangeError('Original field, finite chord and radius required');
        for(let k=0;k<3;k++)row.point[k]=fraction===0?positions[0][k]:fraction===1?positions[1][k]:positions[0][k]+fraction*(positions[1][k]-positions[0][k]);
        consumeQuery?.();
        const hit=field.queryCapsuleCoordinates(...row.point,...row.point,radius,contact);
        captureCompositeWallDifferentialContact(row,hit,radius);captureCompositeWallDifferentialContact(local,hit,radius);
        row.source=local.source=hit.source;row.gap=hit.signedGap;row.sampleCount=hit.capsuleSampleCount;row.faceIndex=hit.faceIndex;
        row.normal.set(hit.inward.values);local.normal.set(hit.inward.values);
        row.closestPoint.set(hit.closestPoint.values);local.closestPoint.set(hit.closestPoint.values);
        if(!Number.isFinite(row.gap)||!Number.isFinite(row.signedDistance)||!row.normal.every(Number.isFinite)||!row.closestPoint.every(Number.isFinite))throw new RangeError('Nonfinite original point contact');
        differentiateCompositeWallRow({field,positions:[row.point,row.point],row:local},scratch);
        row.derivativeSource=local.derivativeSource;row.derivativeReason=local.derivativeReason;
        if(local.derivativeUnavailable)return row;
        for(let a=0;a<2;a++)for(let i=0;i<3;i++) {
            row.gapJacobian[3*a+i]=weights[a]*local.gapJacobian[i];
            row.forceColumn[3*a+i]=weights[a]*local.forceColumn[i];
            for(let b=0;b<2;b++)for(let j=0;j<3;j++)row.normalDerivative[(3*a+i)*6+3*b+j]=weights[a]*weights[b]*local.normalDerivative[i*6+j];
        }
        row.derivativeUnavailable=false;return row;
    }
    return Object.freeze({fraction,edge,row,refresh});
}
