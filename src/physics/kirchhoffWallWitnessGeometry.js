import {Vector3,Triangle} from 'three';

export function createKirchhoffWallWitnessGeometryWorkspace() {
    const a=new Vector3(),b=new Vector3(),c=new Vector3();
    return {closestPoint:new Float64Array(3),barycentric:new Float64Array(3),direction:new Float64Array(3),
        triangleVertices:new Float64Array(9),triangleKey:null,faceIndex:-1,feature:null,
        distance:NaN,normalDefined:false,featureMask:0,
        _triangle:new Triangle(a,b,c),_vertices:[a,b,c],_point:new Vector3(),_closest:new Vector3(),_bary:new Vector3(),
        _ab:new Vector3(),_ac:new Vector3(),_cross:new Vector3(),_triangleNormal:new Vector3(),_normalKey:null,_keyVertices:new Float64Array(9).fill(NaN)};
}

/** Closest point on one RETAINED FINITE triangle, not its supporting plane.
 * faceIndex follows the direct MeshBVH geometry index contract. No global
 * winner query, signed-distance convention or reaction ownership is inferred.
 * Returned arrays belong to out and are reused on its next evaluation. The
 * immutable triangleKey includes the copied vertex values, not a live owner.
 */
export function evaluateKirchhoffWallWitnessGeometry({geometry,faceIndex,point,reuseTriangle=false},out=createKirchhoffWallWitnessGeometryWorkspace()) {
    out.normalDefined=false;out.feature=null;out.distance=NaN;out.direction.fill(NaN);
    const positions=geometry?.attributes?.position,index=geometry?.index,count=index?.count??positions?.count;
    if(!positions||positions.itemSize!==3||!Number.isInteger(count)||count%3!==0||
        !Number.isInteger(faceIndex)||faceIndex<0||faceIndex*3+2>=count)
        throw new RangeError('A valid finite triangle face index is required');
    if(geometry.boundsTree&&(geometry.boundsTree.geometry!==geometry||geometry.boundsTree.indirect))
        throw new TypeError('Retained faces require matching direct BVH geometry indices');
    if(!point||point.length!==3||!Number.isFinite(point[0])||!Number.isFinite(point[1])||!Number.isFinite(point[2]))throw new TypeError('A finite point is required');
    const vertices=out._vertices;
    for(let i=0;i<3;i++) {
        const at=index?index.getX(3*faceIndex+i):3*faceIndex+i;
        if(!Number.isInteger(at)||at<0||at>=positions.count)throw new RangeError('Invalid triangle vertex index');
        vertices[i].set(positions.getX(at),positions.getY(at),positions.getZ(at));
        if(reuseTriangle){const at=3*i;out.triangleVertices[at]=vertices[i].x;out.triangleVertices[at+1]=vertices[i].y;out.triangleVertices[at+2]=vertices[i].z;}
        else vertices[i].toArray(out.triangleVertices,3*i);
    }
    const sameTriangle=out.faceIndex===faceIndex&&!out.triangleVertices.some((v,i)=>v!==out._keyVertices[i]);
    const reuseNormal=reuseTriangle&&sameTriangle&&out._normalKey!==null&&out._normalKey===out.triangleKey;
    if(reuseNormal)out._cross.copy(out._triangleNormal);
    else {
        if(!out.triangleVertices.every(Number.isFinite))throw new RangeError('Triangle vertices must be finite');
        out._ab.subVectors(vertices[1],vertices[0]);out._ac.subVectors(vertices[2],vertices[0]);
        out._cross.crossVectors(out._ab,out._ac);
        if(!(out._cross.lengthSq()>0)||!Number.isFinite(out._cross.lengthSq()))throw new RangeError('Degenerate triangle has no unique surface feature');
    }
    if(!sameTriangle) {
        out.triangleKey=`${faceIndex}:${out.triangleVertices.join(',')}`;
        out._keyVertices.set(out.triangleVertices);
    }
    if(reuseTriangle&&!reuseNormal){out._triangleNormal.copy(out._cross);out._normalKey=out.triangleKey;}
    out.faceIndex=faceIndex;out._point.fromArray(point);
    out._triangle.closestPointToPoint(out._point,out._closest);
    // An exactly coplanar point in the finite triangle has exactly zero
    // distance; do not manufacture a direction from reconstruction roundoff.
    out._ab.subVectors(out._point,vertices[0]);
    if(out._cross.dot(out._ab)===0) {
        out._triangle.getBarycoord(out._point,out._bary);
        if(out._bary.x>=0&&out._bary.y>=0&&out._bary.z>=0)out._closest.copy(out._point);
    }
    out._triangle.getBarycoord(out._closest,out._bary);
    if(reuseTriangle) {
        out.closestPoint[0]=out._closest.x;out.closestPoint[1]=out._closest.y;out.closestPoint[2]=out._closest.z;
        out.barycentric[0]=out._bary.x;out.barycentric[1]=out._bary.y;out.barycentric[2]=out._bary.z;
    } else {out._closest.toArray(out.closestPoint);out._bary.toArray(out.barycentric);}
    if(!out.closestPoint.every(Number.isFinite)||!out.barycentric.every(Number.isFinite))throw new RangeError('Finite triangle evaluation failed');
    // Feature classification tolerates barycentric roundoff only; it does not
    // move the closest point or change its distance/normal.
    out.featureMask=0;let support=0;
    for(let i=0;i<3;i++)if(out.barycentric[i]>64*Number.EPSILON){out.featureMask|=1<<i;support++;}
    out.feature=support===3?'face':support===2?'edge':'vertex';
    out._cross.subVectors(out._point,out._closest);out.distance=out._cross.length();
    if(!Number.isFinite(out.distance))throw new RangeError('Finite triangle distance overflow');
    if(out.distance>0){
        out._cross.multiplyScalar(1/out.distance);
        if(reuseTriangle){out.direction[0]=out._cross.x;out.direction[1]=out._cross.y;out.direction[2]=out._cross.z;}
        else out._cross.toArray(out.direction);
        out.normalDefined=true;
    }
    return out;
}
