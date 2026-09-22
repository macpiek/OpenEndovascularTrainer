import {relaxGraftAxis,fitExpandedGraft} from './stentGraftExpansion.js';
import {graftWire,wireSegment,updateWire} from './stentGraftWire.js';
import * as THREE from 'three';

const segment=wireSegment;
const struts=graftWire;
// Interpolate the deformed fabric, so metal remains attached while the sheath
// passes each row, including the folded contralateral branch.
export function fabricPoint(part,s,angle) {
    const row=part.path.coordinates;
    let i=1;while(i<row.length-1&&row[i]<s)i++;
    const t=THREE.MathUtils.clamp((s-row[i-1])/Math.max(1e-9,row[i]-row[i-1]),0,1);
    const col=((angle/(2*Math.PI)*part.sides)%part.sides+part.sides)%part.sides;
    const j=Math.floor(col),u=col-j,p=part.mesh.geometry.attributes.position;
    const at=r=>new THREE.Vector3().fromBufferAttribute(p,r*part.sides+j)
        .lerp(new THREE.Vector3().fromBufferAttribute(p,r*part.sides+(j+1)%part.sides),u);
    return at(i-1).lerp(at(i),t);
}
export function createScaffold(part,material,markerMaterial) {
    const segments=[],length=part.path.length;
    for(let start=1;start<length-2;start+=9) {
        const height=Math.min(6,length-start-1),count=part.sides;
        for(let j=0;j<count;j++) {
            // Alternating valleys give the characteristic repeated M profile.
            const point=n=>({s:start+(n%2?height:n%4===2?height*.25:0),a:n/count*Math.PI*2});
            // Follow the circumference, rather than cutting a straight chord
            // through the fabric between peaks. Round line caps close the joins.
            const a=point(j),b=point(j+1),subdivisions=4;
            const at=t=>({s:THREE.MathUtils.lerp(a.s,b.s,t),a:THREE.MathUtils.lerp(a.a,b.a,t)});
            for(let k=0;k<subdivisions;k++)segments.push([at(k/subdivisions),at((k+1)/subdivisions)]);
        }
    }
    const metal=struts(segments.length,material);metal.name='nitinol-M-stents';
    const markers=struts(8,markerMaterial,.4);
    markers.material.depthTest=false;markers.renderOrder=20;
    markers.frustumCulled=false;markers.name='radiopaque-end-markers';
    part.scaffoldSegments=segments;part.markers=markers;part.rings=metal;
    return [metal,markers];
}
export function updateScaffold(part) {
    part.scaffoldSegments.forEach(([a,b],i)=>segment(part.rings,i,fabricPoint(part,a.s,a.a),fabricPoint(part,b.s,b.a)));
    updateWire(part.rings);
    for(let i=0;i<8;i++) {
        const start=i<4?0:Math.max(0,part.path.length-2.2),end=i<4?Math.min(2.2,part.path.length):part.path.length;
        segment(part.markers,i,fabricPoint(part,start,i%4*Math.PI/2),fabricPoint(part,end,i%4*Math.PI/2));
    }
    updateWire(part.markers);
    if(part.orientationMarker) {
        const points=[[.12,0],[-.13,0],[-.12,-1.4],[0,-2],[.12,-1.4],[.13,0],[.1,1.5],[0,2],[-.12,1.3]];
        for(let i=1;i<points.length;i++) {
            const at=([angle,s])=>fabricPoint(part,Math.min(part.path.length,8+s),Math.PI/2+angle);
            segment(part.orientationMarker,i-1,at(points[i-1]),at(points[i]));
        }
        updateWire(part.orientationMarker);
    }
}
export function createOrientationMarker(part,material) {
    part.orientationMarker=struts(8,material,.3);part.orientationMarker.name='e-orientation-marker';
    return part.orientationMarker;
}
const CROWN_SUBDIVISIONS=12;
export function createSuprarenalCrown(material) {
    const mesh=struts(12*(2*CROWN_SUBDIVISIONS+1),material,.055);mesh.name='suprarenal-capture-crown';return mesh;
}
export function updateSuprarenalCrown(device) {
    const part=device.parts[0],p=part.points[0],path=device.crownPath;
    const direction=p.clone().sub(part.points[1]).normalize();
    const captured=device.deliveryPath.sample(device.position+12),released=device.tipRelease>=1;
    const opening=THREE.MathUtils.clamp((device.sheathWithdrawal-device.coverLead)/2,0,1);
    if(device.crownOpening===opening&&device.crownReleased===released&&device.crownCaptured?.distanceToSquared(captured)<1e-12)return;
    device.crownCaptured=captured.clone();device.crownOpening=opening;device.crownReleased=released;
    // Fit the expanded crown as one coupled sleeve. Independent clipping of
    // its individual wire samples produced zigzags along otherwise straight legs.
    if(!device.expandedCrown) {
        const points=Array.from({length:CROWN_SUBDIVISIONS+1},(_,i)=>path.sample(path.length*i/CROWN_SUBDIVISIONS));
        relaxGraftAxis(points,device.wallFit);
        const sleeve={points,rows:points.length,sides:part.sides,radius:part.radius,target:new Float32Array(points.length*part.sides*3)};
        const frames=points.map((point,i)=>{
            const tangent=points[Math.min(points.length-1,i+1)].clone().sub(points[Math.max(0,i-1)]).normalize();
            const q=new THREE.Quaternion().setFromUnitVectors(direction,tangent);
            return {u:part.ringFrames[0].u.clone().applyQuaternion(q),v:part.ringFrames[0].v.clone().applyQuaternion(q),tangent};
        });
        fitExpandedGraft(sleeve,device.wallFit,frames);device.expandedCrown=sleeve;
    }
    const sleeve=device.expandedCrown;
    const expandedAt=(row,angle)=>{
        const col=((angle/(2*Math.PI)*part.sides)%part.sides+part.sides)%part.sides,j=Math.floor(col);
        return new THREE.Vector3().fromArray(sleeve.target,(row*part.sides+j)*3)
            .lerp(new THREE.Vector3().fromArray(sleeve.target,(row*part.sides+(j+1)%part.sides)*3),col-j);
    };
    let index=0;
    device.crownPolylines=[];
    for(let i=0;i<12;i++) {
        const a=i/12*Math.PI*2,b=(i+.5)/12*Math.PI*2,c=(i+1)/12*Math.PI*2;
        let peak;
        for(const angle of [a,c]) {
            const base=fabricPoint(part,0,angle).sub(p),line=[];
            for(let j=0;j<=CROWN_SUBDIVISIONS;j++) {
                const t=j/CROWN_SUBDIVISIONS,s=path.length*t,center=path.sample(s);
                const expanded=expandedAt(j,THREE.MathUtils.lerp(angle,b,t));
                // Keep the base attached to the actual fabric during release.
                expanded.addScaledVector(fabricPoint(part,0,angle).sub(expandedAt(0,angle)),1-t);
                // The captured crown fans toward the wire tip; its released
                // shape follows the curved vessel axis instead of a straight extrusion.
                const held=p.clone().lerp(captured,t).addScaledVector(base,1-t);
                // Capture travel is not radial deployment. Peaks remain held
                // until the latch releases, then spring to their wall-fit shape.
                const q=released?expanded:held;
                device.wallFit.fit(q,center,.4);line.push(q);
                if(j)segment(device.crown,index++,line[j-1],q);
            }
            device.crownPolylines.push(line);peak=line.at(-1);
        }
        // Keep the anchoring detail within the lumen as well as the crown tips.
        const center=path.sample(Math.max(0,path.length-1.5));
        const hook=device.wallFit.fit(peak.clone().addScaledVector(direction,-1.5),center,.4);
        segment(device.crown,index++,peak,hook);device.crownPolylines.push([peak,hook]);
    }
    device.crown.visible=opening>0;updateWire(device.crown);
}
