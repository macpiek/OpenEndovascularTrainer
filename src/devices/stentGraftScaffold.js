import {constrainSewnFabric} from './stentGraftSewnConstraints.js';
import {CROWN_SUBDIVISIONS,crownMaterial,inextensibleCrown} from './stentGraftCrownKinematics.js';
import {ringLayout,inextensibleRing,ringWave} from './stentGraftRingKinematics.js';
import {relaxGraftAxis,fitExpandedGraft} from './stentGraftExpansion.js';
import {graftWire,wireSegment,updateWire} from './stentGraftWire.js';
import * as THREE from 'three';
import {partExposure} from './stentGraftDeployment.js';

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
    const a=p.array,low=((i-1)*part.sides+j)*3,high=(i*part.sides+j)*3;
    const next=(j+1)%part.sides,lowNext=((i-1)*part.sides+next)*3,highNext=(i*part.sides+next)*3;
    const out=new THREE.Vector3();
    for(let c=0;c<3;c++) {
        const x=a[low+c]+(a[lowNext+c]-a[low+c])*u;
        const y=a[high+c]+(a[highNext+c]-a[high+c])*u;
        out.setComponent(c,x+(y-x)*t);
    }
    return out;
}
export function createScaffold(part,material,markerMaterial) {
    const segments=[],length=part.path.length;
    const nominal=s=>{
        if(part.nominalRadius)return typeof part.nominalRadius==='function'?part.nominalRadius(s):part.nominalRadius;
        if(!part.rowRadii)return part.radius;
        const row=part.path.coordinates;let i=1;while(i<row.length-1&&row[i]<s)i++;
        return THREE.MathUtils.lerp(part.rowRadii[i-1],part.rowRadii[i],(s-row[i-1])/(row[i]-row[i-1]));
    };
    part.scaffoldRings=ringLayout(length,part.sides,nominal,part.dimensionScale??1,part.scaffoldInset??0);
    for(const ring of part.scaffoldRings)for(let j=0;j<ring.samples;j++) {
        const at=k=>({s:ring.center+ring.height*ringWave(k/12,ring.proximal),a:k/ring.samples*Math.PI*2});
        segments.push([at(j),at(j+1)]);
    }
    const metal=struts(segments.length,material);metal.name='nitinol-M-stents';
    const markers=struts(Math.max(1,part.markerLayout?.length??0),markerMaterial,.3);
    markers.geometry.instanceCount=part.markerLayout?.length??0;
    markers.count=part.markerLayout?.length??0;
    markers.material.depthTest=false;markers.renderOrder=20;
    markers.frustumCulled=false;markers.name='radiopaque-end-markers';
    part.scaffoldSegments=segments;part.markers=markers;part.rings=metal;
    return [metal,markers];
}
export function updateScaffold(part,{constrain=true}={}) {
    let index=0;
    for(const ring of part.scaffoldRings) {
        const low=ring.center-ring.maxHeight/2,high=ring.center+ring.maxHeight/2;
        ring.packed=!part.exposure||!part.path.coordinates.some((s,i)=>s>=low-2&&s<=high+2&&part.exposure[i]>0);
    }
    if(constrain)constrainSewnFabric(part);
    // Sew metal just inside the fabric. Adjacent branches have a shared fabric
    // septum; putting both wires on that zero-thickness plane made them overlap.
    const positions=part.mesh.geometry.attributes.position;
    const centers=part.path.coordinates.map((_,i)=>{
        const c=new THREE.Vector3();
        for(let j=0;j<part.sides;j++) {
            const index=(i*part.sides+j)*3,a=positions.array;
            c.x+=a[index];c.y+=a[index+1];c.z+=a[index+2];
        }
        return c.multiplyScalar(1/part.sides);
    });
    const sample=(s,a)=>{
        const row=part.path.coordinates;let i=1;while(i<row.length-1&&row[i]<s)i++;
        const t=THREE.MathUtils.clamp((s-row[i-1])/(row[i]-row[i-1]),0,1);
        const c=centers[i-1],d=centers[i],p=fabricPoint(part,s,a);
        const x=c.x+(d.x-c.x)*t-p.x,y=c.y+(d.y-c.y)*t-p.y,z=c.z+(d.z-c.z)*t-p.z;
        const length=Math.sqrt(x*x+y*y+z*z),factor=(part.sewnParent ? .2 : .15)/(length||1);
        p.x+=x*factor;p.y+=y*factor;p.z+=z*factor;return p;
    };
    for(const ring of part.scaffoldRings) {
        // Fully settled sewn bands do not need another arc-length solve when
        // a different ring or a covered section of the implant moves.
        const row=part.path.coordinates,low=ring.center-ring.maxHeight/2,high=ring.center+ring.maxHeight/2;
        let first=0,last=row.length-1;
        while(first+1<row.length&&row[first+1]<low)first++;
        while(last>0&&row[last-1]>high)last--;
        const start=first*part.sides*3,end=(last+1)*part.sides*3;
        const band=positions.array.subarray(start,end);
        if(ring.fabricState?.length===band.length&&band.every((v,i)=>v===ring.fabricState[i])) {index+=ring.samples;continue;}
        ring.fabricState=band.slice();
        const points=inextensibleRing(ring,sample);
        for(let i=1;i<points.length;i++)segment(part.rings,index++,points[i-1],points[i]);
    }
    updateWire(part.rings);
    for(const [i,marker] of (part.markerLayout??[]).entries()) {
        segment(part.markers,i,fabricPoint(part,marker.start,marker.angle),fabricPoint(part,marker.end,marker.angle));
    }
    updateWire(part.markers);
    if(part.gateMarker) {
        const end=part.path.length;
        for(let i=0;i<part.gateMarker.count;i++)segment(part.gateMarker,i,fabricPoint(part,end,i/part.gateMarker.count*2*Math.PI),fabricPoint(part,end,(i+1)/part.gateMarker.count*2*Math.PI));
        updateWire(part.gateMarker);
    }
    if(part.orientationMarker) {
        const points=[[.12,0],[-.13,0],[-.12,-1.4],[0,-2],[.12,-1.4],[.13,0],[.1,1.5],[0,2],[-.12,1.3]];
        for(let i=1;i<points.length;i++) {
            const at=([angle,s])=>fabricPoint(part,Math.min(part.path.length,(2.75+s)*(part.dimensionScale??1)),Math.PI/2+angle);
            segment(part.orientationMarker,i-1,at(points[i-1]),at(points[i]));
        }
        updateWire(part.orientationMarker);
    }
}
export function createGateMarker(part,material) {
    part.gateMarker=struts(part.sides*4,material,.12);
    part.gateMarker.material.depthTest=false;part.gateMarker.renderOrder=20;
    part.gateMarker.name='contralateral-gate-marker';
    return part.gateMarker;
}
export function createOrientationMarker(part,material) {
    part.orientationMarker=struts(8,material,.25);part.orientationMarker.name='e-orientation-marker';
    part.orientationMarker.material.depthTest=false;part.orientationMarker.renderOrder=20;
    return part.orientationMarker;
}

export function createSuprarenalCrown(material) {
    const mesh=struts(12*(2*CROWN_SUBDIVISIONS+1),material,.035);mesh.name='suprarenal-capture-crown';return mesh;
}
export function updateSuprarenalCrown(device) {
    const part=device.parts[0],p=part.points[0],path=device.crownPath;
    const direction=p.clone().sub(part.points[1]).normalize();
    const captured=device.deliveryPath.sample(device.position+12*(device.dimensionScale??1)),released=device.tipRelease>=1;
    const opening=partExposure(device,part,0),rootRevision=part.mesh.geometry.attributes.position.version;
    if(device.crownRootRevision===rootRevision&&device.crownOpening===opening&&device.crownReleased===released&&device.crownCaptured?.distanceToSquared(captured)<1e-12)return;
    device.crownRootRevision=rootRevision;device.crownCaptured=captured.clone();device.crownOpening=opening;device.crownReleased=released;
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
    device.crownMaterial??=crownMaterial(device.diameter,device.dimensionScale??1);
    device.crownLengthError=0;
    device.crownPolylines=[];
    for(let i=0;i<12;i++) {
        const a=i/12*Math.PI*2,b=(i+.5)/12*Math.PI*2,c=(i+1)/12*Math.PI*2;
        const targets=[];
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

            }
            targets.push(line);
        }
        const fitted=inextensibleCrown(targets[0],targets[1],device.crownMaterial,
            (point,j)=>device.wallFit.fit(point,path.sample(path.length*j/CROWN_SUBDIVISIONS),.4),released?null:captured);
        device.crownLengthError=Math.max(device.crownLengthError,fitted.error);
        for(const line of [fitted.left,fitted.right]) {
            for(let j=1;j<line.length;j++)segment(device.crown,index++,line[j-1],line[j]);
            device.crownPolylines.push(line);
        }
        const peak=fitted.left.at(-1);
        const hook=peak.clone().addScaledVector(fitted.left.at(-2).clone().sub(peak).normalize(),device.crownMaterial.hookLength);
        segment(device.crown,index++,peak,hook);device.crownPolylines.push([peak,hook]);
    }
    device.crown.visible=opening>0;updateWire(device.crown);
}
