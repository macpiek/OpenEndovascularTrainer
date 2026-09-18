// Finite-triangle region tests and barycentric arithmetic follow three.js
// Triangle (MIT; license in kirchhoffWallTriangleKernel.LICENSE). Preserve
// operation order: contact feature changes must not come from new rounding.
const caches=new WeakMap();
const MAX_FACES=2048;
function triangle(geometry,face,vertices) {
    let cache=caches.get(geometry);
    if(!cache){cache=new Map();caches.set(geometry,cache);}
    let entry=cache.get(face),same=!!entry;
    if(same)for(let i=0;i<9;i++)if(!Object.is(vertices[i],entry.vertices[i])){same=false;break;}
    if(same)return entry;
    if(!vertices.every(Number.isFinite))throw new RangeError('Triangle vertices must be finite');
    const [ax,ay,az,bx,by,bz,cx,cy,cz]=vertices;
    const ab=[bx-ax,by-ay,bz-az],ac=[cx-ax,cy-ay,cz-az],bc=[cx-bx,cy-by,cz-bz];
    const normal=[ab[1]*ac[2]-ab[2]*ac[1],ab[2]*ac[0]-ab[0]*ac[2],ab[0]*ac[1]-ab[1]*ac[0]];
    const norm2=normal[0]*normal[0]+normal[1]*normal[1]+normal[2]*normal[2];
    if(!(norm2>0)||!Number.isFinite(norm2))throw new RangeError('Degenerate triangle has no unique surface feature');
    const dot00=ac[0]*ac[0]+ac[1]*ac[1]+ac[2]*ac[2],dot01=ac[0]*ab[0]+ac[1]*ab[1]+ac[2]*ab[2];
    const dot11=ab[0]*ab[0]+ab[1]*ab[1]+ab[2]*ab[2],denom=dot00*dot11-dot01*dot01;
    entry={vertices:vertices.slice(),ab,ac,bc,normal,dot00,dot01,dot11,denom,invDenom:1/denom,key:`${face}:${vertices.join(',')}`};
    // Bound memory even if a simulation visits many different faces. Entries
    // contain numbers only, never references back to geometry or old states.
    if(cache.size>=MAX_FACES)cache.clear();
    cache.set(face,entry);return entry;
}
function barycentric(t,x,y,z,out) {
    if(t.denom===0){out.fill(0);return;}
    const px=x-t.vertices[0],py=y-t.vertices[1],pz=z-t.vertices[2],{ac,ab}=t;
    const dot02=ac[0]*px+ac[1]*py+ac[2]*pz,dot12=ab[0]*px+ab[1]*py+ab[2]*pz;
    const u=(t.dot11*dot02-t.dot01*dot12)*t.invDenom,v=(t.dot00*dot12-t.dot01*dot02)*t.invDenom;
    out[0]=1-u-v;out[1]=v;out[2]=u;
}
function closest(t,p,out) {
    const [ax,ay,az,bx,by,bz,cx,cy,cz]=t.vertices,{ab,ac,bc}=t;
    const apx=p[0]-ax,apy=p[1]-ay,apz=p[2]-az;
    const d1=ab[0]*apx+ab[1]*apy+ab[2]*apz,d2=ac[0]*apx+ac[1]*apy+ac[2]*apz;
    if(d1<=0&&d2<=0){out[0]=ax;out[1]=ay;out[2]=az;return;}
    const bpx=p[0]-bx,bpy=p[1]-by,bpz=p[2]-bz;
    const d3=ab[0]*bpx+ab[1]*bpy+ab[2]*bpz,d4=ac[0]*bpx+ac[1]*bpy+ac[2]*bpz;
    if(d3>=0&&d4<=d3){out[0]=bx;out[1]=by;out[2]=bz;return;}
    const vc=d1*d4-d3*d2;
    if(vc<=0&&d1>=0&&d3<=0) {
        const v=d1/(d1-d3);out[0]=ax+ab[0]*v;out[1]=ay+ab[1]*v;out[2]=az+ab[2]*v;return;
    }
    const cpx=p[0]-cx,cpy=p[1]-cy,cpz=p[2]-cz;
    const d5=ab[0]*cpx+ab[1]*cpy+ab[2]*cpz,d6=ac[0]*cpx+ac[1]*cpy+ac[2]*cpz;
    if(d6>=0&&d5<=d6){out[0]=cx;out[1]=cy;out[2]=cz;return;}
    const vb=d5*d2-d1*d6;
    if(vb<=0&&d2>=0&&d6<=0) {
        const w=d2/(d2-d6);out[0]=ax+ac[0]*w;out[1]=ay+ac[1]*w;out[2]=az+ac[2]*w;return;
    }
    const va=d3*d6-d5*d4;
    if(va<=0&&(d4-d3)>=0&&(d5-d6)>=0) {
        const w=(d4-d3)/((d4-d3)+(d5-d6));out[0]=bx+bc[0]*w;out[1]=by+bc[1]*w;out[2]=bz+bc[2]*w;return;
    }
    const denom=1/(va+vb+vc),v=vb*denom,w=vc*denom;
    out[0]=(ax+ab[0]*v)+ac[0]*w;out[1]=(ay+ab[1]*v)+ac[1]*w;out[2]=(az+ab[2]*v)+ac[2]*w;
}

/** Geometry/index/point validation and fresh vertex reads belong to the caller.
 * Cached values are shared across witnesses, but outputs remain caller-owned. */
export function evaluateKirchhoffWallTriangleKernel(geometry,faceIndex,point,out) {
    const t=triangle(geometry,faceIndex,out.triangleVertices),cp=out.closestPoint,bary=out.barycentric;
    closest(t,point,cp);
    const x=point[0]-t.vertices[0],y=point[1]-t.vertices[1],z=point[2]-t.vertices[2];
    if(t.normal[0]*x+t.normal[1]*y+t.normal[2]*z===0) {
        barycentric(t,...point,bary);
        if(bary[0]>=0&&bary[1]>=0&&bary[2]>=0)cp.set(point);
    }
    barycentric(t,cp[0],cp[1],cp[2],bary);
    if(!Number.isFinite(cp[0])||!Number.isFinite(cp[1])||!Number.isFinite(cp[2])||
        !Number.isFinite(bary[0])||!Number.isFinite(bary[1])||!Number.isFinite(bary[2]))throw new RangeError('Finite triangle evaluation failed');
    out.featureMask=0;let support=0;
    for(let i=0;i<3;i++)if(bary[i]>64*Number.EPSILON){out.featureMask|=1<<i;support++;}
    out.feature=support===3?'face':support===2?'edge':'vertex';
    const dx=point[0]-cp[0],dy=point[1]-cp[1],dz=point[2]-cp[2];
    out.distance=Math.sqrt(dx*dx+dy*dy+dz*dz);
    if(!Number.isFinite(out.distance))throw new RangeError('Finite triangle distance overflow');
    if(out.distance>0) {
        const inverse=1/out.distance;out.direction[0]=dx*inverse;out.direction[1]=dy*inverse;out.direction[2]=dz*inverse;
        out.normalDefined=true;
    }
    out.triangleKey=t.key;out.faceIndex=faceIndex;out._keyVertices.set(out.triangleVertices);
    // Keep the reference normal cache warm too when callers switch kernels.
    out._triangleNormal.set(t.normal[0],t.normal[1],t.normal[2]);out._normalKey=t.key;
    return out;
}
