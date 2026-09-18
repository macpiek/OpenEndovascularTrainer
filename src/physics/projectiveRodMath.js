/** Small SPD band factor, owned by one PD step and reused for all three axes. */
export function factorProjectiveBand(terms, count, fixed) {
    const band=1+Math.max(0,...terms.map(t=>Math.max(...t.ids)-Math.min(...t.ids)));
    const a=new Float64Array(count*band),offset=Array.from({length:count},()=>[0,0,0]);
    for(const {ids,coefficients:c,weight:w} of terms)for(let i=0;i<ids.length;i++) {
        const row=ids[i];if(fixed.has(row))continue;
        for(let j=0;j<ids.length;j++) {
            const col=ids[j],v=w*c[i]*c[j];
            if(fixed.has(col)){for(let k=0;k<3;k++)offset[row][k]-=v*fixed.get(col)[k];}
            else if(row>=col)a[row*band+row-col]+=v;
        }
    }
    for(const i of fixed.keys())a[i*band]=1;
    for(let i=0;i<count;i++)for(let j=Math.max(0,i-band+1);j<=i;j++) {
        let v=a[i*band+i-j];
        for(let k=Math.max(0,i-band+1);k<j;k++)v-=a[i*band+i-k]*a[j*band+j-k];
        if(i===j){if(!(v>0&&Number.isFinite(v)))throw new Error('PD matrix is not positive definite');a[i*band]=Math.sqrt(v);}
        else a[i*band+i-j]=v/a[j*band];
    }
    return {band,solve(rhs) {
        const x=rhs.map((p,i)=>fixed.has(i)?fixed.get(i).slice():p.map((v,k)=>v+offset[i][k]));
        for(let i=0;i<count;i++)for(let k=0;k<3;k++) {
            for(let j=Math.max(0,i-band+1);j<i;j++)x[i][k]-=a[i*band+i-j]*x[j][k];
            x[i][k]/=a[i*band];
        }
        for(let i=count-1;i>=0;i--)for(let k=0;k<3;k++) {
            for(let j=i+1;j<Math.min(count,i+band);j++)x[i][k]-=a[j*band+j-i]*x[j][k];
            x[i][k]/=a[i*band];
        }
        return x;
    }};
}

export function quaternionColumns({x,y,z,w}) {
    return [[1-2*(y*y+z*z),2*(x*y+z*w),2*(x*z-y*w)],
        [2*(x*y-z*w),1-2*(x*x+z*z),2*(y*z+x*w)],
        [2*(x*z+y*w),2*(y*z-x*w),1-2*(x*x+y*y)]];
}

/** Exact proper-rotation Procrustes projection via the largest eigenvector of
 * Horn's symmetric quaternion matrix. Jacobi handles reflections and 180°;
 * power iteration alone can select the eigenvalue of largest absolute value. */
export function projectRotation(columns) {
    const b=Array.from({length:3},(_,i)=>columns.map(c=>c[i]));
    const [a,d,f]=[b[0][0],b[1][1],b[2][2]],xy=b[0][1]+b[1][0],xz=b[0][2]+b[2][0],yz=b[1][2]+b[2][1];
    const sx=b[2][1]-b[1][2],sy=b[0][2]-b[2][0],sz=b[1][0]-b[0][1];
    const m=[[a+d+f,sx,sy,sz],[sx,a-d-f,xy,xz],[sy,xy,d-a-f,yz],[sz,xz,yz,f-a-d]];
    const v=Array.from({length:4},(_,i)=>Array.from({length:4},(_,j)=>+(i===j)));
    for(let sweep=0;sweep<12;sweep++) {
        let largest=0;
        for(let p=0;p<4;p++)for(let q=p+1;q<4;q++) {
            const off=m[p][q];largest=Math.max(largest,Math.abs(off));if(Math.abs(off)<1e-14)continue;
            const angle=.5*Math.atan2(2*off,m[q][q]-m[p][p]),c=Math.cos(angle),s=Math.sin(angle);
            const pp=m[p][p],qq=m[q][q];
            m[p][p]=c*c*pp-2*s*c*off+s*s*qq;m[q][q]=s*s*pp+2*s*c*off+c*c*qq;m[p][q]=m[q][p]=0;
            for(let k=0;k<4;k++) {
                if(k!==p&&k!==q){const kp=m[k][p],kq=m[k][q];m[k][p]=m[p][k]=c*kp-s*kq;m[k][q]=m[q][k]=s*kp+c*kq;}
                const vp=v[k][p],vq=v[k][q];v[k][p]=c*vp-s*vq;v[k][q]=s*vp+c*vq;
            }
        }
        if(largest<1e-12)break;
    }
    let best=0;for(let i=1;i<4;i++)if(m[i][i]>m[best][best])best=i;
    const q={w:v[0][best],x:v[1][best],y:v[2][best],z:v[3][best]};
    return {quaternion:q,columns:quaternionColumns(q)};
}
