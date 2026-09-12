import {createCompositeDiscreteWallPoint} from './compositeDiscreteWallPoint.js';
import {createCompositeWallSdfBranchesWorkspace,evaluateCompositeWallSdfBranches,measureCompositeWallSdfBranches} from './kirchhoffCompositeWallSdfBranches.js';

/** Discover a bounded, proved seam around a freshly queried material point.
 * This only proposes a chart: installing rows and transferring reactions must
 * remain an atomic operation of the physical-step owner. No force is changed.
 */
export function discoverCompositeDiscreteWallPointSeam({field,original,radius,halfWidth}) {
    const h=field?.voxelSize;
    if(!Number.isFinite(h)||h<=0)return {supported:false,reason:'missing-sdf-grid'};
    const width=halfWidth??h*.04;
    if(!Number.isFinite(width)||width<=0||width>=h/2)throw new RangeError('Seam half width must be positive and smaller than half a voxel');
    const p=Array.from(original?.point??[]),contact=original?.rawContact;
    if(p.length!==3||!p.every(Number.isFinite)||contact?.source!=='sparse-sdf')return {supported:false,reason:'requires-original-sdf-point'};
    if(!field.sdfOrigin||field.sdfOrigin.length!==3)return {supported:false,reason:'missing-sdf-grid'};
    const grid=field.sdfGridCoordinates?Array.from(field.sdfGridCoordinates(p,contact)):p.map((v,k)=>(v-field.sdfOrigin[k])/h),
        candidates=grid.map((v,axis)=>({axis,gridIndex:Math.round(v),distance:Math.abs(v-Math.round(v))*h}))
            .filter(c=>c.distance<width).sort((a,b)=>a.distance-b.distance||a.axis-b.axis),attempts=[];
    for(const {axis,gridIndex} of candidates){
        const lower=p.map(v=>v-width),upper=p.map(v=>v+width);
        // Remain strictly within the two adjacent cells in transverse axes.
        // An actual edge/corner requires a separate multi-face chart.
        let transverse=true;
        for(let k=0;k<3;k++)if(k!==axis){
            const f=grid[k]-Math.floor(grid[k]),margin=Math.min(f,1-f)*h;
            if(margin===0){transverse=false;break;}
            const w=Math.min(width,margin*.5);lower[k]=p[k]-w;upper[k]=p[k]+w;
        }
        if(!transverse){attempts.push({axis,gridIndex,reason:'multiple-cell-faces'});continue;}
        const face={axis,gridIndex},domainBox={lower,upper},chart=evaluateCompositeWallSdfBranches({field,positions:[p],radius,contact,face,domainBox},createCompositeWallSdfBranchesWorkspace(1));
        if(chart.supported)return {supported:true,sdfSeam:{face,domainBox},selectedBranch:chart.selectedRow,onSeam:chart.onSeam};
        attempts.push({axis,gridIndex,reason:chart.reason});
    }
    return {supported:false,reason:candidates.length?'no-proved-local-seam':'no-nearby-cell-face',attempts};
}

/** Two proved local wall branches at ONE fixed material point. The original
 * detector row remains distinct; a continued polynomial is never relabelled
 * as a detector result. This component owns no physical step or force history.
 */
export function createCompositeDiscreteWallPointBranches(options) {
    const point=createCompositeDiscreteWallPoint(options),fraction=point.fraction,weights=[1-fraction,fraction],chart=createCompositeWallSdfBranchesWorkspace(1);
    const branches=[0,1].map(branchIndex=>({branchIndex,source:'sparse-sdf-cell-polynomial',role:'material-point-branch',edge:point.edge,fraction,
        supported:false,gap:NaN,signedDistance:NaN,normal:new Float64Array(3),wallPoint:new Float64Array(3),
        gapJacobian:new Float64Array(6),forceColumn:new Float64Array(6),normalDerivative:new Float64Array(36)}));
    let current=false;
    function invalidate(){current=false;for(const row of branches){row.supported=false;row.gap=row.signedDistance=NaN;for(const key of ['normal','wallPoint','gapJacobian','forceColumn','normalDerivative'])row[key].fill(NaN);}}
    function refresh({field,positions,radius,face,domainBox,consumeQuery}) {
        invalidate();
        const original=point.refresh({field,positions,radius,consumeQuery}),center=Array.from(original.point),result=evaluateCompositeWallSdfBranches({field,positions:[center],radius,face,domainBox,contact:original.rawContact},chart);
        if(!result.supported)return {supported:false,reason:result.reason,original,branches};
        for(const row of branches){
            const source=result.rows[row.branchIndex];row.gap=source.gap;row.signedDistance=source.signedDistance;row.normal.set(source.normal);
            row.wallPoint.set(center.map((v,k)=>v-source.signedDistance*source.normal[k]));
            for(let a=0;a<2;a++)for(let i=0;i<3;i++){
                row.gapJacobian[3*a+i]=weights[a]*source.gapJacobian[i];row.forceColumn[3*a+i]=weights[a]*source.forceColumn[i];
                for(let b=0;b<2;b++)for(let j=0;j<3;j++)row.normalDerivative[(3*a+i)*6+3*b+j]=weights[a]*weights[b]*source.normalDerivative[3*i+j];
            }
            row.supported=true;
        }
        current=true;return {supported:true,reason:null,original,branches,selectedBranch:result.selectedRow,onSeam:result.onSeam};
    }
    return Object.freeze({point,branches,refresh,measure(options){
        if(!current)throw new RangeError('A fresh supported branch evaluation is required');
        return measureCompositeWallSdfBranches(chart,options);
    }});
}
