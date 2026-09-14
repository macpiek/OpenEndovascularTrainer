/** One unilateral bend limit on the common axis, independent of material count. */
export function evaluateSharedAxisBendLimit({state,edge,needHessian=true}, angle) {
    const p=state.positions,u=p[edge+1].map((x,k)=>x-p[edge][k]),v=p[edge+2].map((x,k)=>x-p[edge+1][k]);
    const l=Math.hypot(...u),m=Math.hypot(...v),scale=Math.min(state.coordinates[edge+1]-state.coordinates[edge],state.coordinates[edge+2]-state.coordinates[edge+1]);
    for(let k=0;k<3;k++){u[k]/=l;v[k]/=m;}
    const c=u.reduce((s,x,k)=>s+x*v[k],0),a=[],b=[],gu=[],gv=[];
    for(let i=0;i<3;i++) {
        gu[i]=(v[i]-c*u[i])/l;gv[i]=(u[i]-c*v[i])/m;
        for(let j=0;j<3;j++){a[i*3+j]=((i===j?1:0)-u[i]*u[j])/l;b[i*3+j]=((i===j?1:0)-v[i]*v[j])/m;}
    }
    if(!needHessian)return {gap:scale*(c-Math.cos(angle)),jacobian:[...gu.map(v=>-scale*v),...gu.map((v,k)=>scale*(v-gv[k])),...gv.map(v=>scale*v)]};
    const h=new Float64Array(36);
    for(let i=0;i<3;i++)for(let j=0;j<3;j++) {
        h[i*6+j]=-(u[i]*gu[j]+gu[i]*u[j]+c*a[i*3+j])/l;
        h[(i+3)*6+j+3]=-(v[i]*gv[j]+gv[i]*v[j]+c*b[i*3+j])/m;
        let cross=0;for(let k=0;k<3;k++)cross+=a[i*3+k]*b[k*3+j];
        h[i*6+j+3]=h[(j+3)*6+i]=cross;
    }
    const pull=[[-1,0],[1,-1],[0,1]],jacobian=[],hessian=new Float64Array(81);
    for(let i=0;i<9;i++) {
        const node=Math.floor(i/3),axis=i%3;jacobian[i]=scale*(pull[node][0]*gu[axis]+pull[node][1]*gv[axis]);
        for(let j=0;j<9;j++)for(let x=0;x<2;x++)for(let y=0;y<2;y++)
            hessian[i*9+j]+=scale*pull[node][x]*pull[Math.floor(j/3)][y]*h[(3*x+axis)*6+3*y+j%3];
    }
    return {gap:scale*(c-Math.cos(angle)),jacobian,hessian};
}
