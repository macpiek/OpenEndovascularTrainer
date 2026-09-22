import * as THREE from 'three';
export const NOSECONE_LENGTH_MM=60;
export const NOSECONE_BASE_RADIUS_MM=2.5;

/** Flexible tapered sleeve around the already deformable guidewire. Parallel
 * transport keeps cross-sections continuous on bends; only the unsupported
 * end extrapolates the wire's last tangent. No rigid cone transform is used. */
export function createFlexibleNoseGeometry(path,start,length=NOSECONE_LENGTH_MM) {
    const rows=Math.max(2,Math.ceil(length)),sides=16,positions=[],indices=[];
    const last=path.sample(path.length),lastDirection=last.clone().sub(path.sample(Math.max(0,path.length-1))).normalize();
    if(lastDirection.lengthSq()<1e-10)lastDirection.set(0,1,0);
    const sample=s=>s<=path.length?path.sample(s):last.clone().addScaledVector(lastDirection,s-path.length);
    let previousTangent=null,u=null;
    for(let i=0;i<=rows;i++) {
        const s=start+length*i/rows,p=sample(s),tangent=sample(s+.25).sub(sample(s-.25)).normalize();
        if(tangent.lengthSq()<1e-10)tangent.copy(lastDirection);
        if(u)u.applyQuaternion(new THREE.Quaternion().setFromUnitVectors(previousTangent,tangent));
        else u=new THREE.Vector3(Math.abs(tangent.z)<.9?0:1,0,Math.abs(tangent.z)<.9?1:0).cross(tangent).normalize();
        const v=tangent.clone().cross(u).normalize(),radius=.12+(NOSECONE_BASE_RADIUS_MM-.12)*Math.pow(1-i/rows,1.15);
        for(let j=0;j<sides;j++) {
            const angle=2*Math.PI*j/sides;
            positions.push(...p.clone().addScaledVector(u,radius*Math.cos(angle)).addScaledVector(v,radius*Math.sin(angle)).toArray());
            if(i<rows){const a=i*sides+j,b=i*sides+(j+1)%sides,c=a+sides,d=b+sides;indices.push(a,b,c,b,d,c);}
        }
        previousTangent=tangent;
    }
    for(const [row,reverse]of [[0,true],[rows,false]]) {
        const index=positions.length/3;positions.push(...sample(start+length*row/rows).toArray());
        for(let j=0;j<sides;j++){const a=row*sides+j,b=row*sides+(j+1)%sides;indices.push(index,...(reverse?[b,a]:[a,b]));}
    }
    const geometry=new THREE.BufferGeometry();geometry.setAttribute('position',new THREE.Float32BufferAttribute(positions,3));
    geometry.setIndex(indices);geometry.computeVertexNormals();geometry.computeBoundingSphere();
    return geometry;
}
