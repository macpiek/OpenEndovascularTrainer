import * as THREE from 'three';
import {createContactResult} from '../physics/collision/vesselContactField.js';
import {AORTIC_NECK,AORTIC_BIFURCATION,DevicePath,wireDevicePath,createAorticRoutes} from './stentGraftPaths.js';
import {StentGraftSurface} from './stentGraftSurface.js';

const fail=reason=>({ok:false,reason});
const success=()=>({ok:true});
const otherSide=side=>side==='right'?'left':'right';

/** Kinematic release with elastic contact against deployed fabric. The rod solver
 * owns contact reactions; sealed components remodel the contrast network. */
export class StentGraftSystem {
    constructor({readAccess,readAnatomy,sheaths}) {
        Object.assign(this,{readAccess,readAnatomy,sheaths});
        this.accesses={right:{device:null,message:''},left:{device:null,message:''}};
        this.committedSources={};
        this.surface=null;this.surfaces=[];
        this.implants=[];this.nextId=1;this.routes=null;
        this.group=new THREE.Group();this.group.name='stent-grafts';
        this.fabric=new THREE.Group();this.metal=new THREE.Group();this.delivery=new THREE.Group();
        this.group.add(this.fabric,this.metal,this.delivery);
        this.fabricMaterial=new THREE.MeshBasicMaterial({color:0xd9ece7,transparent:true,opacity:.26,side:THREE.DoubleSide,depthWrite:false});
        this.metalMaterial=new THREE.MeshBasicMaterial({color:0xffce69});
        this.deliveryMaterial=new THREE.MeshBasicMaterial({color:0xca8ce8});
        this.markerMaterial=new THREE.MeshBasicMaterial({color:0x57f3ee});
        this.contact=createContactResult();this.geometryRevision=0;
    }
    getPath(side){return this.committedSources[side]?.path ?? wireDevicePath(this.readAccess(side));}
    catheterPosition(side){return this.committedSources[side]?.catheterMm ?? this.readAccess(side).catheterMm;}
    ensureRoutes() {
        if(!this.routes) {
            const anatomy=this.readAnatomy();
            if(!anatomy?.centerlineBroadPhase?.segments?.length)return false;
            this.routes=createAorticRoutes(anatomy.centerlineBroadPhase.segments,this.sheaths);
        }return true;
    }
    load(side,type) {
        const access=this.accesses[side];
        if(access.device)return fail('Najpierw wycofaj i usuń bieżący system wprowadzający.');
        if(!['body','limb'].includes(type))return fail('Nieznany implant.');
        if(!this.ensureRoutes())return fail('Anatomia jest jeszcze wczytywana.');
        if(this.catheterPosition(side)>1)return fail('Wycofaj cewnik z tej koszulki przed wprowadzeniem stentgraftu.');
        if(type==='body'&&this.implants.some(i=>i.type==='body'))return fail('Korpus stentgraftu jest już rozłożony.');
        if(type==='body'&&Object.values(this.accesses).some(a=>a.device?.type==='body'))return fail('Korpus jest już załadowany w drugiej koszulce.');
        access.device={id:this.nextId++,type,side,position:0,target:0,phase:'loaded',deployment:0,diameter:type==='body'?28:14,length:80,parts:[],deliveryMesh:null};
        access.message='System wybrany. Wsuwaj po prowadniku sterowaniem cewnika (D / A).';return success();
    }
    availableGate(side) {
        return this.implants.find(i=>i.type==='body'&&i.side!==side&&i.deployment>=1&&!i.connectedLimbId)?.gate ?? null;
    }
    setPosition(side,value) {
        const d=this.accesses[side].device;if(!d||d.phase==='deploying')return;
        const limit=Math.max(0,this.getPath(side).length-12);
        d.target=THREE.MathUtils.clamp(Number(value)||0,0,limit);
    }
    setDiameter(side,value) {
        const d=this.accesses[side].device;
        if(d?.phase==='loaded')d.diameter=THREE.MathUtils.clamp(Number(value)||d.diameter,d.type==='body'?20:10,d.type==='body'?36:20);
    }
    positionAtTarget(side) {
        const d=this.accesses[side].device;if(!d||d.phase!=='loaded')return fail('Załaduj system wprowadzający.');
        const gate=this.availableGate(side),target=d.type==='body'?AORTIC_NECK:gate?.docking;
        if(!target)return fail('Brak dostępnej bramki korpusu.');
        const wire=this.getPath(side),nearest=wire.nearest(target);
        if(nearest.distance>18 || nearest.s>wire.length-12)return fail('Wprowadź prowadnik dalej, przez docelową szyję aorty lub bramkę.');
        this.setPosition(side,nearest.s);return success();
    }
    validation(side) {
        const d=this.accesses[side].device;if(!d||d.phase!=='loaded')return fail('Brak systemu gotowego do rozłożenia.');
        if(this.catheterPosition(side)>1)return fail('Wycofaj cewnik z tej koszulki.');
        const wire=this.getPath(side),nose=wire.sample(d.position);
        if(!nose||d.position<=.5||wire.length<d.position)return fail('Wprowadź system po prowadniku, aby go rozłożyć.');
        return success();
    }
    deploy(side) {
        const valid=this.validation(side);if(!valid.ok)return valid;
        const d=this.accesses[side].device,wire=this.getPath(side);
        const nose=wire.sample(d.position);
        d.target=d.position;
        const parts=[];
        if(d.type==='body') {
            const route=this.routes[side],contra=this.routes[otherSide(side)];
            const nearest=route.nearest(nose);
            const bif=route.nearest(AORTIC_BIFURCATION).s;
            const split=bif+8;
            // Outside the aortoiliac landing region, pose the same Y-shaped
            // template at the actual delivery nose instead of snapping to the
            // aorta or reversing the trunk toward the anatomical bifurcation.
            const freePlacement=nearest.s<=split+8||nearest.distance>14;
            const top=freePlacement?route.nearest(AORTIC_NECK).s:nearest.s;
            const trunk=route.section(top,split,2);
            const ipsi=route.section(split,Math.max(0,bif-65),2);
            const contraSplit=contra.nearest(route.sample(split)).s;
            const contraPath=contra.section(contraSplit,contraSplit-30,2);
            // Separate the two outlets within the distal trunk, then follow
            // each iliac route. The short outlet remains open for the extension.
            ipsi[0].x+=(side==='right'?-1:1)*3;
            contraPath[0].x+=(side==='right'?1:-1)*3;
            if(freePlacement) {
                const origin=trunk[0].clone(),from=origin.clone().sub(route.sample(top-2)).normalize();
                const to=nose.clone().sub(wire.sample(Math.max(0,d.position-2))).normalize();
                const rotation=from.lengthSq()&&to.lengthSq()?new THREE.Quaternion().setFromUnitVectors(from,to):new THREE.Quaternion();
                for(const points of [trunk,ipsi,contraPath])for(const point of points)
                    point.sub(origin).applyQuaternion(rotation).add(nose);
            }
            trunk[0].copy(nose);
            parts.push(this.buildPart(trunk,d.diameter/2));
            parts.push(this.buildPart(ipsi,7));parts.push(this.buildPart(contraPath,7));
            const gatePath=new DevicePath(contraPath);
            d.gate={side:otherSide(side),docking:gatePath.sample(gatePath.length-10),
                entry:gatePath.sample(gatePath.length),routeS:contraSplit-20,parent:d,freePlacement};
            const marker=new THREE.Mesh(new THREE.TorusGeometry(6.5,.6,6,24),this.markerMaterial);
            marker.position.copy(d.gate.entry);
            marker.quaternion.setFromUnitVectors(new THREE.Vector3(0,0,1),gatePath.sample(gatePath.length-2).sub(d.gate.entry).normalize());
            d.gate.marker=marker;marker.visible=false;this.metal.add(marker);
            d.parentId=null;
        } else {
            const gate=this.availableGate(side),route=this.routes[side];
            const connected=gate&&nose.distanceTo(gate.docking)<=10;
            const points=connected&&!gate.freePlacement
                ?route.section(gate.routeS,Math.max(0,gate.routeS-d.length),2)
                :wire.section(d.position,Math.max(0,d.position-d.length),2);
            points[0].copy(nose);
            parts.push(this.buildPart(points,d.diameter/2));
            d.parentId=connected?gate.parent.id:null;
            if(connected){gate.parent.connectedLimbId=d.id;gate.marker.material=this.metalMaterial;}
        }
        d.parts=parts;d.phase='deploying';d.deployment=0;
        this.implants.push(d);this.geometryRevision++;
        this.accesses[side].message='Rozkładanie stentgraftu… pozycja implantu jest zablokowana.';
        return success();
    }
    removeDelivery(side) {
        const access=this.accesses[side],d=access.device;
        if(!d)return success();
        if(d.phase==='deploying')return fail('Poczekaj na zakończenie rozkładania.');
        if(d.position>.5){d.target=0;return fail('System jest wycofywany. Usuń go po osiągnięciu 0 cm.');}
        if(d.deliveryMesh){this.delivery.remove(d.deliveryMesh);d.deliveryMesh.geometry.dispose();}
        if(d.noseMarker){this.delivery.remove(d.noseMarker);d.noseMarker.geometry.dispose();}
        access.device=null;access.message='System usunięty; rozłożony implant pozostaje w naczyniu.';return success();
    }
    updateAccess(side,dt,committedSource=null,command=null) {
        // Capture after the rod transaction commits. Cooperative Newton trials
        // can change feed coordinates before committing a matching wire pose.
        if(committedSource)this.committedSources[side]={path:wireDevicePath(committedSource),catheterMm:committedSource.catheterMm};
        const access=this.accesses[side],d=access.device;if(!d||!(dt>0))return;
        // Sample the catheter controls with the rod step; consume them only
        // after it commits. Rejected/suspended steps cannot advance delivery.
        if(command?.deviceId===d.id&&d.phase!=='deploying') {
            const advance=THREE.MathUtils.clamp(Number(command.advance)||0,-1,1);
            if(advance>0) {
                this.setPosition(side,d.position+25*dt*advance);
                d.target=Math.max(d.position,d.target);
            }
            else d.target=Math.max(0,d.position+25*dt*advance);
        }
        if(d.phase==='deploying') {
            d.deployment=Math.min(1,d.deployment+dt/3);
            for(const part of d.parts)this.expandPart(part,d.deployment);
            if(d.deployment===1){
                d.phase='deployed';if(d.gate)d.gate.marker.visible=true;
                access.message=d.type==='body'?'Korpus rozłożony. Otwarta bramka — dołącz nóżkę z przeciwnej koszulki.':
                    d.parentId!==null?'Nóżka połączona. Kontrast płynie światłem stentgraftu.':'Nóżka rozłożona bez połączenia z korpusem.';
                this.geometryRevision++;
                this.surface=new StentGraftSurface(this.implants.filter(i=>i.phase==='deployed'),this.geometryRevision);
                // Previous snapshots can still belong to a cooperative Newton step.
                this.surfaces.push(this.surface);
            }
        } else {
            const wire=this.getPath(side);
            const target=d.target;
            // A withdrawn guidewire cannot pull an implanted graft with it.
            if(target>d.position&&wire.length<target+12){access.message='Brak podparcia prowadnikiem — wsuwanie zatrzymane.';return;}
            const move=Math.sign(target-d.position)*Math.min(Math.abs(target-d.position),25*dt);
            if(this.catheterPosition(side)>1&&move>0){access.message='Wycofaj cewnik przed wsuwaniem systemu.';return;}
            d.position=Math.max(0,d.position+move);
        }
    }
    refreshDelivery(side) {
        const d=this.accesses[side].device;if(!d)return;
        if(d.position<1){if(d.deliveryMesh)d.deliveryMesh.visible=false;if(d.noseMarker)d.noseMarker.visible=false;return;}
        const wire=this.getPath(side);
        if(wire.length<d.position)return; // keep the last supported delivery pose
        const points=wire.section(Math.max(0,d.position-130),d.position,3);
        const curve=new THREE.CatmullRomCurve3(points,false,'centripetal');
        const geometry=new THREE.TubeGeometry(curve,Math.max(4,points.length*2),d.phase==='loaded'?1.8:1,8,false);
        if(!d.deliveryMesh){d.deliveryMesh=new THREE.Mesh(geometry,this.deliveryMaterial);this.delivery.add(d.deliveryMesh);}
        else {d.deliveryMesh.geometry.dispose();d.deliveryMesh.geometry=geometry;}
        d.deliveryMesh.visible=true;
        if(!d.noseMarker){d.noseMarker=new THREE.Mesh(new THREE.SphereGeometry(2.4,12,8),this.markerMaterial);this.delivery.add(d.noseMarker);}
        d.noseMarker.position.copy(points.at(-1));d.noseMarker.visible=true;
    }
    buildPart(points,radius) {
        const rows=points.length,sides=24,positions=new Float32Array(rows*sides*3),target=new Float32Array(positions.length),indices=[];
        const field=this.readAnatomy()?.contactField,wall=this.readAnatomy()?.geometry?.boundsTree;
        const ringFrames=[];
        for(let i=0;i<rows;i++) {
            const p=points[i],tangent=points[Math.min(rows-1,i+1)].clone().sub(points[Math.max(0,i-1)]).normalize();
            const axis=Math.abs(tangent.z)<.9?new THREE.Vector3(0,0,1):new THREE.Vector3(1,0,0);
            const u=axis.cross(tangent).normalize(),v=tangent.clone().cross(u).normalize();ringFrames.push({u,v});
            for(let j=0;j<sides;j++) {
                const angle=j/sides*Math.PI*2,dir=u.clone().multiplyScalar(Math.cos(angle)).addScaledVector(v,Math.sin(angle));
                const hit=wall?.raycastFirst(new THREE.Ray(p,dir),THREE.DoubleSide,0,radius+.3);
                let r=hit?Math.min(radius,Math.max(.15,hit.distance-.2)):radius;
                // Containment uses the same selected anatomy as the rod solver.
                if(field)for(let attempt=0;attempt<10;attempt++) {
                    const q=p.clone().addScaledVector(dir,r);
                    if(!field.querySphere(q,0,this.contact).violation)break;r*=.85;
                }
                const q=p.clone().addScaledVector(dir,r);q.toArray(target,(i*sides+j)*3);
                if(i<rows-1){const a=i*sides+j,b=i*sides+(j+1)%sides,c=a+sides,e=b+sides;indices.push(a,c,b,b,c,e);}
            }
        }
        const geometry=new THREE.BufferGeometry();geometry.setAttribute('position',new THREE.BufferAttribute(positions,3));geometry.setIndex(indices);
        const mesh=new THREE.Mesh(geometry,this.fabricMaterial);mesh.frustumCulled=false;this.fabric.add(mesh);
        const ringGeometry=new THREE.BufferGeometry(),ringPositions=new Float32Array(Math.ceil(rows/4)*sides*2*3);
        ringGeometry.setAttribute('position',new THREE.BufferAttribute(ringPositions,3));
        const rings=new THREE.LineSegments(ringGeometry,new THREE.LineBasicMaterial({color:0xffce69}));rings.frustumCulled=false;this.metal.add(rings);
        const part={points,rows,sides,target,mesh,rings,ringFrames};this.expandPart(part,0);return part;
    }
    expandPart(part,fraction) {
        const positions=part.mesh.geometry.attributes.position,ringPositions=part.rings.geometry.attributes.position;
        let ringIndex=0;
        for(let i=0;i<part.rows;i++) {
            const local=THREE.MathUtils.clamp((fraction*1.2-i/part.rows)*5,0,1),p=part.points[i];
            for(let j=0;j<part.sides;j++) {
                const k=(i*part.sides+j)*3,q=new THREE.Vector3().fromArray(part.target,k),dir=q.clone().sub(p);
                const initial=Math.min(1,1.3/Math.max(1e-6,dir.length()));
                q.copy(p).addScaledVector(dir,initial+(1-initial)*local);positions.setXYZ(i*part.sides+j,q.x,q.y,q.z);
            }
            if(i%4===0)for(let j=0;j<part.sides;j++)for(const n of [j,(j+1)%part.sides]) {
                const k=i*part.sides+n;ringPositions.setXYZ(ringIndex++,positions.getX(k),positions.getY(k),positions.getZ(k));
            }
        }
        positions.needsUpdate=true;ringPositions.needsUpdate=true;
    }
    setFluoroscopy(enabled){this.fabric.visible=!enabled;}
    snapshot(side) {
        const access=this.accesses[side],d=access.device;
        return {device:d,limit:Math.max(0,this.getPath(side).length-12),message:access.message,
            validation:d?.phase==='loaded'?this.validation(side):null,implants:this.implants};
    }
    dispose() {
        for(const surface of this.surfaces)surface.dispose();
        this.group.traverse(o=>{o.geometry?.dispose();if(o.isLineSegments)o.material.dispose();});
        this.fabricMaterial.dispose();this.metalMaterial.dispose();this.deliveryMaterial.dispose();this.markerMaterial.dispose();this.group.clear();
    }
}
