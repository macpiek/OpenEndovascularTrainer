/** One unilateral bend limit on the common axis, independent of material count. */
const pull=[[-1,0],[1,-1],[0,1]];
function createBendWork(){return {u:new Array(3),v:new Array(3),gu:new Array(3),gv:new Array(3)};}
export function evaluateSharedAxisBendLimit({state,edge,needHessian=true,contactStorage=null}, angle) {
    // A contact record owns this scratch for one private timestep. Row banks
    // copy derivatives before retaining a measure; standalone calls still
    // return independent arrays. No pose or reaction is cached here.
    const work=contactStorage?(contactStorage.bendWork??=createBendWork()):createBendWork();
    const p=state.positions,{u,v,gu,gv}=work;
    for(let k=0;k<3;k++){u[k]=p[edge+1][k]-p[edge][k];v[k]=p[edge+2][k]-p[edge+1][k];}
    const l=Math.hypot(...u),m=Math.hypot(...v),scale=Math.min(state.coordinates[edge+1]-state.coordinates[edge],state.coordinates[edge+2]-state.coordinates[edge+1]);
    for(let k=0;k<3;k++){u[k]/=l;v[k]/=m;}
    let c=0;for(let k=0;k<3;k++)c+=u[k]*v[k];
    for(let i=0;i<3;i++) {
        gu[i]=(v[i]-c*u[i])/l;gv[i]=(u[i]-c*v[i])/m;
    }
    const out=contactStorage?.contact??{gap:0,jacobian:new Array(9)},jacobian=out.jacobian;
    out.gap=scale*(c-Math.cos(angle));out.hessian=undefined;
    if(!needHessian){
        for(let k=0;k<3;k++){jacobian[k]=-scale*gu[k];jacobian[3+k]=scale*(gu[k]-gv[k]);jacobian[6+k]=scale*gv[k];}
        return out;
    }
    const a=work.a??=new Array(9),b=work.b??=new Array(9);
    for(let i=0;i<3;i++)for(let j=0;j<3;j++){a[i*3+j]=((i===j?1:0)-u[i]*u[j])/l;b[i*3+j]=((i===j?1:0)-v[i]*v[j])/m;}
    const h=work.h??=new Float64Array(36);
    for(let i=0;i<3;i++)for(let j=0;j<3;j++) {
        h[i*6+j]=-(u[i]*gu[j]+gu[i]*u[j]+c*a[i*3+j])/l;
        h[(i+3)*6+j+3]=-(v[i]*gv[j]+gv[i]*v[j]+c*b[i*3+j])/m;
        let cross=0;for(let k=0;k<3;k++)cross+=a[i*3+k]*b[k*3+j];
        h[i*6+j+3]=h[(j+3)*6+i]=cross;
    }
    const hessian=contactStorage?(contactStorage.hessian??=new Float64Array(81)):new Float64Array(81);
    hessian.fill(0);
    for(let i=0;i<9;i++) {
        const node=Math.floor(i/3),axis=i%3;jacobian[i]=scale*(pull[node][0]*gu[axis]+pull[node][1]*gv[axis]);
        for(let j=0;j<9;j++)for(let x=0;x<2;x++)for(let y=0;y<2;y++)
            hessian[i*9+j]+=scale*pull[node][x]*pull[Math.floor(j/3)][y]*h[(3*x+axis)*6+3*y+j%3];
    }
    out.hessian=hessian;return out;
}
