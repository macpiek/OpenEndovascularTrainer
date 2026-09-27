import * as THREE from 'three';
import {fitExpandedGraft} from './stentGraftExpansion.js';

const cross=(a,b)=>a.x*b.y-a.y*b.x;
const smooth=t=>t*t*t*(10-15*t+6*t*t);

/** Intersect a ray with the actual fitted trunk polygon, not its nominal
 * circle. The old nominal circle re-expanded the crotch after vessel fitting,
 * creating a shelf even when every individual tube had a smooth taper.
 */
function boundaryHit(polygon,origin,direction) {
    let nearest=Infinity,hit=null;
    for(let j=0;j<polygon.length;j++) {
        const a=polygon[j],b=polygon[(j+1)%polygon.length],edge=b.clone().sub(a),delta=a.clone().sub(origin);
        const denominator=cross(direction,edge);
        if(Math.abs(denominator)<1e-10)continue;
        const t=cross(delta,edge)/denominator,u=cross(delta,direction)/denominator;
        if(t>=0&&u>=-1e-8&&u<=1+1e-8&&t<nearest){nearest=t;hit={t,j,u:THREE.MathUtils.clamp(u,0,1)};}
    }
    return hit;
}

/** Match complementary trunk sections to the two legs over 20 mm. The
 * boundary and its longitudinal tangent come from the wall-fitted trunk.
 * Nominal limb diameters and open distal gates are preserved.
 */
export function fitGraftJunction(parts,wallFit) {
    const [trunk,...legs]=parts,center=trunk.points.at(-1),frame=trunk.ringFrames.at(-1);
    const lateral=legs[1].points[0].clone().sub(legs[0].points[0]);
    lateral.addScaledVector(frame.tangent,-lateral.dot(frame.tangent));
    if(lateral.lengthSq()<1e-10)lateral.copy(frame.u);
    lateral.normalize();
    const up=frame.tangent.clone().cross(lateral).normalize();
    const end=[],previous=[],polygon=[];
    for(let j=0;j<trunk.sides;j++) {
        const p=new THREE.Vector3().fromArray(trunk.target,((trunk.rows-1)*trunk.sides+j)*3);
        end.push(p);previous.push(new THREE.Vector3().fromArray(trunk.target,((trunk.rows-2)*trunk.sides+j)*3));
        const relative=p.clone().sub(center);polygon.push(new THREE.Vector2(relative.dot(lateral),relative.dot(up)));
    }
    const min=Math.min(...polygon.map(p=>p.x)),max=Math.max(...polygon.map(p=>p.x));
    const r0=legs[0].rowRadii?.[0]??legs[0].radius,r1=legs[1].rowRadii?.[0]??legs[1].radius;
    const split=THREE.MathUtils.lerp(min,max,r0/(r0+r1));
    const axialStep=Math.max(1e-6,center.clone().sub(trunk.points.at(-2)).dot(frame.tangent));
    for(let leg=0;leg<2;leg++) {
        const part=legs[leg],sign=leg===0?-1:1;
        const origin=new THREE.Vector2(((leg===0?min:max)+split)/2,0);
        for(let row=0;row<part.rows;row++) {
            const t=THREE.MathUtils.clamp(part.path.coordinates[row]/20,0,1);
            if(t>=1)break;
            const blend=smooth(t),axial=part.points[row].clone().sub(center).dot(frame.tangent);
            for(let j=0;j<part.sides;j++) {
                const index=(row*part.sides+j)*3,p=new THREE.Vector3().fromArray(part.target,index);
                const dir=p.clone().sub(part.points[row]);
                const direction=new THREE.Vector2(dir.dot(lateral),dir.dot(up));
                if(direction.lengthSq()<1e-10)direction.set(Math.cos(j/part.sides*2*Math.PI),Math.sin(j/part.sides*2*Math.PI));
                direction.normalize();
                const hit=boundaryHit(polygon,origin,direction);
                if(!hit)continue;
                const seam=direction.x*sign<0?(split-origin.x)/direction.x:Infinity;
                let q;
                if(seam<hit.t) {
                    const point=origin.clone().addScaledVector(direction,seam);
                    q=center.clone().addScaledVector(lateral,point.x).addScaledVector(up,point.y).addScaledVector(frame.tangent,axial);
                } else {
                    const next=(hit.j+1)%trunk.sides;
                    q=end[hit.j].clone().lerp(end[next],hit.u);
                    const prev=previous[hit.j].clone().lerp(previous[next],hit.u);
                    q.addScaledVector(q.clone().sub(prev),axial/axialStep);
                }
                q.lerp(p,blend);
                wallFit.fit(q,part.points[row],.7).toArray(part.target,index);
            }
        }
    }
}

/** The catalogue crotch span is a width across two outlets, not a round
 * tube diameter. Reduced geometric approximation: blend a round inlet into
 * an oval spanning the two gates, with the depth of one gate. */
export function fitIIsTrunkSection(parts,wallFit,gateDiameter) {
    const [trunk,...legs]=parts;
    const lateral=legs[1].points[0].clone().sub(legs[0].points[0]);
    trunk.restRadii=new Float64Array(trunk.rows*trunk.sides);
    for(let i=0;i<trunk.rows;i++) {
        const frame=trunk.ringFrames[i],wide=lateral.clone().addScaledVector(frame.tangent,-lateral.dot(frame.tangent));
        if(wide.lengthSq()<1e-10)wide.copy(frame.u);wide.normalize();
        const depth=frame.tangent.clone().cross(wide).normalize();
        const blend=smooth(i/(trunk.rows-1)),a=trunk.rowRadii[i];
        const b=trunk.rowRadii[0]+(gateDiameter/2-trunk.rowRadii[0])*blend;
        for(let j=0;j<trunk.sides;j++) {
            const angle=j/trunk.sides*2*Math.PI;
            const dir=frame.u.clone().multiplyScalar(Math.cos(angle)).addScaledVector(frame.v,Math.sin(angle));
            trunk.restRadii[i*trunk.sides+j]=1/Math.sqrt((dir.dot(wide)/a)**2+(dir.dot(depth)/b)**2);
        }
    }
    fitExpandedGraft(trunk,wallFit,trunk.ringFrames);
}
