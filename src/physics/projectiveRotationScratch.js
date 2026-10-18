import {quaternionColumns} from './projectiveRodMath.js';

/** Same Horn/Jacobi arithmetic and sweep order as projectRotation. The two
 * 4x4 matrices belong to one caller, avoiding temporary matrices per frame.
 * Returned values own their storage; subsequent calls cannot change targets.
 */
export function createProjectRotationScratch() {
    const m=new Float64Array(16),v=new Float64Array(16);
    return columns=>{
        const [a,d,f]=[columns[0][0],columns[1][1],columns[2][2]];
        const xy=columns[1][0]+columns[0][1],xz=columns[2][0]+columns[0][2],yz=columns[2][1]+columns[1][2];
        const sx=columns[1][2]-columns[2][1],sy=columns[2][0]-columns[0][2],sz=columns[0][1]-columns[1][0];
        m[0]=a+d+f;m[1]=m[4]=sx;m[2]=m[8]=sy;m[3]=m[12]=sz;
        m[5]=a-d-f;m[6]=m[9]=xy;m[7]=m[13]=xz;
        m[10]=d-a-f;m[11]=m[14]=yz;m[15]=f-a-d;
        v.fill(0);v[0]=v[5]=v[10]=v[15]=1;
        for(let sweep=0;sweep<12;sweep++) {
            let largest=0;
            for(let p=0;p<4;p++)for(let q=p+1;q<4;q++) {
                const off=m[4*p+q];largest=Math.max(largest,Math.abs(off));if(Math.abs(off)<1e-14)continue;
                const angle=.5*Math.atan2(2*off,m[5*q]-m[5*p]),c=Math.cos(angle),s=Math.sin(angle);
                const pp=m[5*p],qq=m[5*q];
                m[5*p]=c*c*pp-2*s*c*off+s*s*qq;m[5*q]=s*s*pp+2*s*c*off+c*c*qq;m[4*p+q]=m[4*q+p]=0;
                for(let k=0;k<4;k++) {
                    if(k!==p&&k!==q){const kp=m[4*k+p],kq=m[4*k+q];m[4*k+p]=m[4*p+k]=c*kp-s*kq;m[4*k+q]=m[4*q+k]=s*kp+c*kq;}
                    const vp=v[4*k+p],vq=v[4*k+q];v[4*k+p]=c*vp-s*vq;v[4*k+q]=s*vp+c*vq;
                }
            }
            if(largest<1e-12)break;
        }
        let best=0;for(let i=1;i<4;i++)if(m[5*i]>m[5*best])best=i;
        const q={w:v[best],x:v[4+best],y:v[8+best],z:v[12+best]};
        return {quaternion:q,columns:quaternionColumns(q)};
    };
}
