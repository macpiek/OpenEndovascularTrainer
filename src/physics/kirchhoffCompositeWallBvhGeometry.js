import {createCompositeWallGeometryWorkspace} from './kirchhoffCompositeWallGeometry.js';

const ROUND = 256 * Number.EPSILON;
// Existing VesselContactField threshold: at/below it BVH keeps the old normal.
const PROVIDER_NORMAL_EPSILON = 1e-8;
const dot = (a,b) => a[0]*b[0]+a[1]*b[1]+a[2]*b[2];
const sub = (a,b) => a.map((v,i)=>v-b[i]);
const cross = (a,b) => [a[1]*b[2]-a[2]*b[1],a[2]*b[0]-a[0]*b[2],a[0]*b[1]-a[1]*b[0]];
const unit = v => {const m=Math.hypot(...v);return v.map(x=>x/m);};
const fail = (out,reason) => {out.supported=false;out.reason=reason;return out;};
function finiteVector(v,n,name) {
    if(!v||v.length!==n)throw new TypeError(`${name} needs ${n} finite values`);
    for(let i=0;i<n;i++)if(!Number.isFinite(v[i]))throw new TypeError(`${name} must be finite`);
}
function triangle(geometry,face) {
    const p=geometry?.attributes?.position,index=geometry?.index,count=index?.count??p?.count;
    if(!p||p.itemSize!==3||typeof p.getX!=='function'||!Number.isInteger(count)||count%3!==0||
        !Number.isInteger(face)||face<0||3*face+2>=count)throw new RangeError('A valid direct geometry triangle index is required');
    const vertices=[];
    for(let i=0;i<3;i++){
        const at=index?index.getX(3*face+i):3*face+i;
        if(!Number.isInteger(at)||at<0||at>=p.count)throw new RangeError('Invalid triangle vertex index');
        const v=[p.getX(at),p.getY(at),p.getZ(at)];finiteVector(v,3,'triangle vertex');vertices.push(v);
    }
    return vertices;
}

/** Prove the Voronoi feature of an ALREADY reported closest point. This does
 * not search for another foot or issue a BVH query. Small tolerances below
 * bound floating-point predicate uncertainty; they never modify the contact.
 */
function featureAtFoot(vertices,point,foot) {
    const [a,b,c]=vertices,e0=sub(b,a),e1=sub(c,a),scale=Math.max(...e0.map(Math.abs),...e1.map(Math.abs));
    if(!(scale>0)||!Number.isFinite(scale))return {reason:'degenerate-triangle',onTriangle:false};
    const u=e0.map(v=>v/scale),v=e1.map(v=>v/scale),normal=cross(u,v),area2=dot(normal,normal);
    const uu=dot(u,u),uv=dot(u,v),vv=dot(v,v);
    if(!(area2>ROUND*(uu*vv+uv*uv)))return {reason:'unresolved-triangle-area',onTriangle:false};
    const planeNormal=unit(normal),r=sub(point,foot).map(x=>x/scale),s=sub(foot,a).map(x=>x/scale);
    const coordinateScale=Math.max(scale,...vertices.flat().map(Math.abs),...point.map(Math.abs),...foot.map(Math.abs));
    const positionBound=ROUND*(1+coordinateScale/scale);
    const baryBound=positionBound*(uu+vv+2*Math.abs(uv))/area2;
    if(!Number.isFinite(baryBound)||baryBound>=.125)return {reason:'unresolved-feature-predicates',onTriangle:false};
    const d0=dot(s,u),d1=dot(s,v),b1=(vv*d0-uv*d1)/area2,b2=(uu*d1-uv*d0)/area2;
    const barycentric=[1-b1-b2,b1,b2];
    if(Math.abs(dot(s,planeNormal))>positionBound*(1+Math.hypot(...s))||barycentric.some(x=>x < -baryBound))
        return {reason:'reported-foot-outside-triangle',onTriangle:false,planeNormal};
    const indices=barycentric.flatMap((x,i)=>x>baryBound?[i]:[]),P=new Float64Array(9);
    const dotBound=positionBound*(1+Math.hypot(...r))*(1+Math.sqrt(uu+vv));
    const result={vertices,barycentric,indices,planeNormal,projector:P,onTriangle:true,consistent:true,classical:false,
        positionBound:positionBound*scale,predicateBound:dotBound,scale,reason:null,type:null};
    if(indices.length===3){
        result.type='face';
        if(Math.abs(dot(r,u))>dotBound||Math.abs(dot(r,v))>dotBound)return {...result,consistent:false,reason:'foot-is-not-face-projection'};
        for(let i=0;i<3;i++)for(let j=0;j<3;j++)P[3*i+j]=(i===j?1:0)-planeNormal[i]*planeNormal[j];
        result.classical=true;result.margin=Math.min(...barycentric)-baryBound;
    }else if(indices.length===2){
        result.type='edge';const edge=unit(sub(vertices[indices[1]],vertices[indices[0]])),opposite=3-indices[0]-indices[1];
        if(Math.abs(dot(r,edge))>dotBound)return {...result,consistent:false,reason:'foot-is-not-edge-projection'};
        const towardFace=sub(vertices[opposite],foot).map(x=>x/scale),multiplier=-dot(r,towardFace);
        if(multiplier < -dotBound)return {...result,consistent:false,reason:'foot-is-not-closest-on-triangle'};
        for(let i=0;i<3;i++)for(let j=0;j<3;j++)P[3*i+j]=edge[i]*edge[j];
        result.classical=multiplier>dotBound;result.margin=multiplier-dotBound;
        if(!result.classical)result.reason='face-edge-transition';
    }else if(indices.length===1){
        result.type='vertex';const at=indices[0],multipliers=[];
        for(let i=0;i<3;i++)if(i!==at)multipliers.push(-dot(r,sub(vertices[i],vertices[at]).map(x=>x/scale)));
        if(multipliers.some(x=>x < -dotBound))return {...result,consistent:false,reason:'foot-is-not-closest-on-triangle'};
        result.classical=multipliers.every(x=>x>dotBound);result.margin=Math.min(...multipliers)-dotBound;
        if(!result.classical)result.reason='vertex-feature-transition';
    }else return {...result,consistent:false,reason:'unresolved-feature-dimension'};
    return result;
}

function interfaceKind(selected,neighbor) {
    const shared=[];
    for(const a of selected.vertices)if(neighbor.vertices.some(b=>a.every((v,i)=>v===b[i])))shared.push(a);
    const parallel=Math.abs(dot(selected.planeNormal,neighbor.planeNormal))>=1-ROUND*8;
    if(parallel&&shared.length===2){
        const other=vertices=>vertices.find(a=>!shared.some(b=>a.every((v,i)=>v===b[i])));
        const edge=sub(shared[1],shared[0]),left=cross(edge,sub(other(selected.vertices),shared[0])),right=cross(edge,sub(other(neighbor.vertices),shared[0]));
        if(dot(left,right)<-ROUND*Math.hypot(...left)*Math.hypot(...right))return 'soft-coplanar-interface';
        return 'overlapping-coplanar-interface';
    }
    if(!parallel&&shared.length===1&&selected.type==='vertex'&&neighbor.type==='vertex')return 'vertex-feature-interface';
    if(!parallel&&shared.length<2)return 'surface-intersection';
    return parallel?'coplanar-feature-interface':'crease-feature-interface';
}

export function createCompositeWallBvhGeometryWorkspace(pointCount=2) {
    return {...createCompositeWallGeometryWorkspace(pointCount),closestPoint:new Float64Array(3),
        providerFaceIndex:-1,featureType:null,featureVertexIndices:[],featureGeometryKey:null,barycentric:new Float64Array(3),
        featureMargin:null,classicalFeature:false,interfaceKind:null,localFacesChecked:0,
        globalWinnerCertified:false,derivativeScope:'fixed-selected-triangle-feature-sign-sample'};
}

/** Differentiates the selected BVH triangle feature, preserving the original
 * gap, sign, unit normal, foot, face index and capsule sample. No provider/BVH
 * query is made. G=B^T is established by the Euclidean feature proof, never
 * assumed as a fallback for sparse SDF or a retained near-zero normal.
 *
 * localFaceIndices can contain ALREADY KNOWN incident faces. Their common-foot
 * predicates distinguish soft coplanar interfaces and intersections from a
 * strict feature; no incidence search or global runner-up query is performed.
 * Single-triangle data cannot certify global closest-feature uniqueness.
 */
export function differentiateCompositeWallBvhContact({field,contact,positions,radius,localFaceIndices=[]},out) {
    fail(out,'not-evaluated');out.classicalFeature=false;out.interfaceKind=null;out.featureType=null;out.featureGeometryKey=null;out.localFacesChecked=0;
    if(!positions||positions.length!==out.pointCount)throw new RangeError('Point/capsule geometry must match the workspace');
    positions.forEach(p=>finiteVector(p,3,'position'));
    if(!Number.isFinite(radius)||radius<0)throw new RangeError('A finite nonnegative radius is required');
    if(!contact||!Number.isFinite(contact.signedDistance)||!Number.isFinite(contact.signedGap))throw new TypeError('A finite provider contact is required');
    out.source=contact.source;out.gap=contact.signedGap;out.signedDistance=contact.signedDistance;out.radius=radius;
    if(contact.source!=='sparse-sdf-bvh')return fail(out,`unsupported-source:${contact.source}`);
    const geometry=field?.fallbackGeometry;
    if(!geometry?.boundsTree||geometry.boundsTree.geometry!==geometry)return fail(out,'missing-matching-fallback-geometry');
    if(geometry.boundsTree.indirect)return fail(out,'unsupported-indirect-face-index-contract');
    finiteVector(contact.inward?.values,3,'provider normal');finiteVector(contact.closestPoint?.values,3,'provider closest point');
    const t=out.pointCount===1?0:contact.segmentT;
    if(!Number.isFinite(t)||t<0||t>1)throw new RangeError('The selected fraction must lie on its edge');
    out.sampleFraction=t;out.sampleCount=out.pointCount===1?0:contact.capsuleSampleCount;
    if(out.pointCount===2&&(!Number.isInteger(out.sampleCount)||out.sampleCount<1||Math.abs(t*out.sampleCount-Math.round(t*out.sampleCount))>ROUND))
        return fail(out,'unrecognized-capsule-sampling-branch');
    out.weights[0]=out.pointCount===1?1:1-t;if(out.pointCount===2)out.weights[1]=t;
    for(let i=0;i<3;i++){
        out.point[i]=out.pointCount===1||t===0?positions[0][i]:t===1?positions[1][i]:positions[0][i]+(positions[1][i]-positions[0][i])*t;
        out.normal[i]=contact.inward.values[i];out.closestPoint[i]=contact.closestPoint.values[i];
    }
    out.providerFaceIndex=contact.faceIndex;
    const vertices=triangle(geometry,contact.faceIndex),selected=featureAtFoot(vertices,Array.from(out.point),Array.from(out.closestPoint));
    if(!selected.onTriangle||!selected.consistent)return fail(out,selected.reason);
    out.featureType=selected.type;out.featureVertexIndices=[...selected.indices];out.barycentric.set(selected.barycentric);out.featureMargin=selected.margin;
    out.featureGeometryKey=selected.indices.map(i=>vertices[i].join(',')).sort().join(';');
    if(!Array.isArray(localFaceIndices)||localFaceIndices.some(i=>!Number.isInteger(i)))throw new TypeError('Local incident face indices must be an explicit list');
    let nonclassical=!selected.classical;
    for(const face of new Set(localFaceIndices))if(face!==contact.faceIndex){
        const neighbor=featureAtFoot(triangle(geometry,face),Array.from(out.point),Array.from(out.closestPoint));out.localFacesChecked++;
        if(!neighbor.onTriangle){
            if(neighbor.reason==='reported-foot-outside-triangle')continue;
            return fail(out,'unresolved-local-face-predicates');
        }
        if(!neighbor.consistent)return fail(out,'local-surface-proof-disagrees-with-provider');
        const sameProjector=selected.projector.every((v,i)=>Math.abs(v-neighbor.projector[i])<=ROUND*16);
        if(!selected.classical||!neighbor.classical||!sameProjector){
            nonclassical=true;out.interfaceKind=interfaceKind(selected,neighbor);
        }
    }
    const distance=Math.abs(contact.signedDistance),sign=contact.signedDistance>0?1:-1;
    if(!(distance>PROVIDER_NORMAL_EPSILON))return fail(out,'zero-distance-or-retained-provider-normal');
    if(nonclassical){out.interfaceKind??='unresolved-feature-interface';return fail(out,'nonclassical-feature-boundary');}
    const delta=Array.from(out.point,(v,i)=>v-out.closestPoint[i]),measuredDistance=Math.hypot(...delta);
    const lengthBound=ROUND*Math.max(distance,...out.point.map(Math.abs),...out.closestPoint.map(Math.abs),...vertices.flat().map(Math.abs));
    // Reject a poorly resolved direction before the comparison tolerance can
    // become broad enough to admit an unrelated (but unit) provider normal.
    if(lengthBound>=distance*.125)return fail(out,'unresolved-distance-direction');
    if(Math.abs(measuredDistance-distance)>lengthBound||Math.abs(out.gap-(contact.signedDistance-radius))>ROUND*Math.max(distance,radius))
        return fail(out,'contact-distance-does-not-match-feature');
    if(Math.abs(dot(out.normal,out.normal)-1)>ROUND*8||out.normal.some((v,i)=>Math.abs(v-sign*delta[i]/distance)>ROUND*8+lengthBound/distance))
        return fail(out,'normal-does-not-match-signed-feature-distance');
    out.branchSign=sign;out.gradientNorm=1;out.pointGapGradient.set(out.normal);
    out.pointGapHessian.fill(0);out.pointNormalJacobian.fill(0);
    if(selected.type!=='face')for(let i=0;i<3;i++)for(let j=0;j<3;j++){
        const value=sign/distance*((i===j?1:0)-selected.projector[3*i+j]-out.normal[i]*out.normal[j]);
        out.pointGapHessian[3*i+j]=out.pointNormalJacobian[3*i+j]=value;
    }
    const n=out.dofCount;
    for(let i=0;i<n;i++){
        const wi=out.weights[Math.floor(i/3)];out.gapGradient[i]=out.normalForceColumn[i]=wi*out.normal[i%3];
        for(let j=0;j<n;j++){
            const value=wi*out.weights[Math.floor(j/3)]*out.pointGapHessian[3*(i%3)+j%3];
            out.gapHessian[n*i+j]=out.normalForceJacobian[n*i+j]=value;
        }
    }
    if([out.gapGradient,out.gapHessian,out.normalForceJacobian].some(v=>!v.every(Number.isFinite)))return fail(out,'nonfinite-feature-derivative');
    out.branchSignature=`sparse-sdf-bvh:${selected.type}:${out.featureGeometryKey}:${sign}:${out.sampleCount}:${t}:${radius}`;
    out.classicalFeature=true;out.supported=true;out.reason=null;return out;
}

/** Exactly one original point/capsule query. Pass contactResult to reuse the
 * provider output. The differentiator above needs no query at all when the
 * collector already has the selected result.
 */
export function queryCompositeWallBvhPointGeometry({field,position,radius,contactResult,localFaceIndices},out=createCompositeWallBvhGeometryWorkspace(1)) {
    fail(out,'query-not-completed');finiteVector(position,3,'point');
    if(!Number.isFinite(radius)||radius<0)throw new RangeError('A finite nonnegative radius is required');
    if(typeof field?.querySphere!=='function')throw new TypeError('The original point provider is required');
    const contact=field.querySphere(position,radius,contactResult);
    return differentiateCompositeWallBvhContact({field,contact,positions:[position],radius,localFaceIndices},out);
}
export function queryCompositeWallBvhCapsuleGeometry({field,positions,radius,contactResult,localFaceIndices},out=createCompositeWallBvhGeometryWorkspace(2)) {
    fail(out,'query-not-completed');if(!positions||positions.length!==2)throw new RangeError('Two capsule endpoints are required');
    positions.forEach(p=>finiteVector(p,3,'endpoint'));
    if(!Number.isFinite(radius)||radius<0)throw new RangeError('A finite nonnegative radius is required');
    if(typeof field?.queryCapsuleCoordinates!=='function')throw new TypeError('The original capsule provider is required');
    const contact=field.queryCapsuleCoordinates(...positions[0],...positions[1],radius,contactResult);
    return differentiateCompositeWallBvhContact({field,contact,positions,radius,localFaceIndices},out);
}
