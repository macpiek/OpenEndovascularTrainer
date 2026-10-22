// Shared by the parenchymal renderer and the vascular containment checks.
export const RENAL_HALF_SIZE = Object.freeze([35,55,22]);
// A broad, rounded medial concavity around the pelvis, tapering into both
// poles. Keep the same field for the projected nephrogram and vessel bounds.
const NOTCH = Object.freeze({strength:1.05,offset:1.02,width:.55,height:.50});
export const RENAL_BODY_GLSL = `
float renalBodyLevel(vec3 p,float side){
    return dot(p,p)+${NOTCH.strength.toFixed(6)}*exp(
        -pow((p.x*side+${NOTCH.offset.toFixed(6)})/${NOTCH.width.toFixed(6)},2.)
        -pow(p.y/${NOTCH.height.toFixed(6)},2.));
}`;

/** Conservative upper bound over a sphere, including the hilar indentation.
 * A value below 1 means the entire sphere fits inside the rendered kidney.
 */
export function renalSphereLevelBound(point,radius,kidney) {
    return sphereLevelBound(point,radius,kidney,NOTCH);
}

function sphereLevelBound(point,radius,kidney,notch) {
    const p=[(point.x-kidney.center.x)*kidney.sign,point.y-kidney.center.y,point.z-kidney.center.z];
    const q=p.map((v,i)=>v/RENAL_HALF_SIZE[i]);
    const r=RENAL_HALF_SIZE.map(v=>radius/v);
    const ellipsoid=q.reduce((sum,v,i)=>sum+(Math.abs(v)+r[i])**2,0);
    const x=Math.max(0,Math.abs(q[0]+notch.offset)-r[0])/notch.width;
    const y=Math.max(0,Math.abs(q[1])-r[1])/notch.height;
    return ellipsoid+notch.strength*Math.exp(-x*x-y*y);
}
