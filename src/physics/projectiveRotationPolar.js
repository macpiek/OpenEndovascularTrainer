import {projectRotation,quaternionColumns} from './projectiveRodMath.js';

// For nonsingular, orientation-preserving directors the polar factor is the
// closest proper rotation. Newton's inverse-transpose iteration avoids the
// trigonometric Jacobi eigensolve in the usual near-rotation case. Reflections,
// ill-conditioned matrices and nonconverged iterations retain the reference
// Procrustes solver; no approximate frame is published on fast-path failure.
export function projectRotationPolar(columns) {
    let a=columns[0][0],b=columns[1][0],c=columns[2][0],
        d=columns[0][1],e=columns[1][1],f=columns[2][1],
        g=columns[0][2],h=columns[1][2],i=columns[2][2];
    const norm2=a*a+b*b+c*c+d*d+e*e+f*f+g*g+h*h+i*i;
    const determinant=a*(e*i-f*h)+b*(f*g-d*i)+c*(d*h-e*g);
    if(!(determinant>1e-6*(norm2/3)**1.5&&Number.isFinite(norm2)))return projectRotation(columns);
    let converged=false;
    for(let iteration=0;iteration<10;iteration++) {
        const ca=e*i-f*h,cb=f*g-d*i,cc=d*h-e*g,
            cd=c*h-b*i,ce=a*i-c*g,cf=b*g-a*h,
            cg=b*f-c*e,ch=c*d-a*f,ci=a*e-b*d;
        const inverse=1/(a*ca+b*cb+c*cc);
        const na=.5*(a+ca*inverse),nb=.5*(b+cb*inverse),nc=.5*(c+cc*inverse),
            nd=.5*(d+cd*inverse),ne=.5*(e+ce*inverse),nf=.5*(f+cf*inverse),
            ng=.5*(g+cg*inverse),nh=.5*(h+ch*inverse),ni=.5*(i+ci*inverse);
        const change=Math.max(Math.abs(na-a),Math.abs(nb-b),Math.abs(nc-c),Math.abs(nd-d),Math.abs(ne-e),
            Math.abs(nf-f),Math.abs(ng-g),Math.abs(nh-h),Math.abs(ni-i));
        a=na;b=nb;c=nc;d=nd;e=ne;f=nf;g=ng;h=nh;i=ni;
        if(change<1e-13){converged=true;break;}
    }
    if(!converged)return projectRotation(columns);
    let x,y,z,w;const trace=a+e+i;
    if(trace>0){const s=.5/Math.sqrt(trace+1);w=.25/s;x=(h-f)*s;y=(c-g)*s;z=(d-b)*s;}
    else if(a>e&&a>i){const s=2*Math.sqrt(1+a-e-i);w=(h-f)/s;x=.25*s;y=(b+d)/s;z=(c+g)/s;}
    else if(e>i){const s=2*Math.sqrt(1+e-a-i);w=(c-g)/s;x=(b+d)/s;y=.25*s;z=(f+h)/s;}
    else {const s=2*Math.sqrt(1+i-a-e);w=(d-b)/s;x=(c+g)/s;y=(f+h)/s;z=.25*s;}
    const inverse=1/Math.hypot(x,y,z,w),quaternion={x:x*inverse,y:y*inverse,z:z*inverse,w:w*inverse};
    return {quaternion,columns:quaternionColumns(quaternion)};
}
