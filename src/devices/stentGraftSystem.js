import {relaxGraftAxis,fitExpandedGraft} from './stentGraftExpansion.js';
import {disposeWire} from './stentGraftWire.js';
import {mainBodyModel,bodyDimensions,proximalDiameters,distalDiameters} from './stentGraftModels.js';
import {captureGraftPose,rotateGraftPose} from './stentGraftRotation.js';
import {createFoldedGraftPreview,updateFoldedGraftPreview,disposeFoldedGraftPreview,setFoldedPreviewVisible} from './stentGraftFoldedPreview.js';
import * as THREE from 'three';
import {AORTIC_NECK,AORTIC_BIFURCATION,DevicePath,wireDevicePath,createAorticRoutes} from './stentGraftPaths.js';
import {createFlexibleNoseGeometry} from './stentGraftNose.js';
import {graftLumenSections} from './stentGraftLumenContact.js';
import {preparePartialSurface,partialSurfaceSnapshot,updatePartialSurfacePose} from './stentGraftPartialSurface.js';
import {StentGraftSurface} from './stentGraftSurface.js';
import {StentGraftWallFit} from './stentGraftWallFit.js';
import {initializeRelease,advanceRelease,rowExposure,deliveryNoseState,graftAttached,graftCoverWithdrawal} from './stentGraftDeployment.js';
import {createScaffold,updateScaffold,createSuprarenalCrown,updateSuprarenalCrown,createOrientationMarker} from './stentGraftScaffold.js';

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
        this.surface=null;this.mechanicalSurface=null;this.surfaces=[];
        this.implants=[];this.nextId=1;this.routes=null;
        this.group=new THREE.Group();this.group.name='stent-grafts';
        this.fabric=new THREE.Group();this.metal=new THREE.Group();this.delivery=new THREE.Group();
        this.group.add(this.fabric,this.metal,this.delivery);
        this.fabricMaterial=new THREE.MeshBasicMaterial({color:0xeee8dc,transparent:true,opacity:.38,side:THREE.DoubleSide,depthWrite:false});
        this.metalMaterial=new THREE.MeshBasicMaterial({color:0xc6ced5});
        this.deliveryMaterial=new THREE.MeshBasicMaterial({color:0xca8ce8});
        this.markerMaterial=new THREE.MeshBasicMaterial({color:0x57f3ee});
        this.geometryRevision=0;
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
    load(side,type,modelId) {
        const access=this.accesses[side];
        if(access.device)return fail('Najpierw wycofaj i usuń bieżący system wprowadzający.');
        if(!['body','limb'].includes(type))return fail('Nieznany implant.');
        if(!this.ensureRoutes())return fail('Anatomia jest jeszcze wczytywana.');
        if(this.catheterPosition(side)>1)return fail('Wycofaj cewnik z tej koszulki przed wprowadzeniem stentgraftu.');
        if(type==='body'&&this.implants.some(i=>i.type==='body'))return fail('Korpus stentgraftu jest już rozłożony.');
        if(type==='body'&&Object.values(this.accesses).some(a=>a.device?.type==='body'))return fail('Korpus jest już załadowany w drugiej koszulce.');
        const model=mainBodyModel(modelId);
        access.device={id:this.nextId++,type,side,position:0,target:0,phase:'loaded',deployment:0,
            modelId:model.id,diameter:type==='body'?28:14,distalDiameter:model.id==='iis-103'?14:16,
            length:type==='body'?model.length:80,deliveryRotation:0,graftRotation:0,parts:[],deliveryMesh:null};
        access.message='System wybrany. Wsuwaj po prowadniku sterowaniem cewnika (D / A).';return success();
    }
    availableGate(side) {
        return this.implants.find(i=>i.type==='body'&&i.side!==side&&i.deployment>=1&&!i.connectedLimbId)?.gate ?? null;
    }
    setPosition(side,value) {
        const d=this.accesses[side].device;if(!d)return;
        const limit=Math.max(0,this.getPath(side).length-12);
        d.target=THREE.MathUtils.clamp(Number(value)||0,0,limit);
    }
    setDiameter(side,value) {
        const d=this.accesses[side].device;
        if(d?.phase!=='loaded')return;
        const sizes=d.type==='body'?proximalDiameters(d.modelId):[10,13,14,16,20,24,28];
        d.diameter=sizes.reduce((a,b)=>Math.abs(b-value)<Math.abs(a-value)?b:a);
        if(d.type==='body'&&!distalDiameters(d.modelId,d.diameter).includes(d.distalDiameter))d.distalDiameter=distalDiameters(d.modelId,d.diameter)[0];
    }
    setDistalDiameter(side,value) {
        const d=this.accesses[side].device;
        if(d?.phase==='loaded'&&d.type==='body'&&distalDiameters(d.modelId,d.diameter).includes(Number(value)))d.distalDiameter=Number(value);
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
        disposeFoldedGraftPreview(d);
        const nose=wire.sample(d.position);
        d.target=d.position;d.implantPosition=d.position;
        const parts=[];
        this.wallFit=new StentGraftWallFit(this.readAnatomy());
        d.wallFit=this.wallFit;
        if(d.type==='body') {
            const route=this.routes[side],contra=this.routes[otherSide(side)];
            const nearest=route.nearest(nose);
            const bif=route.nearest(AORTIC_BIFURCATION).s;
            const dimensions=bodyDimensions(d);
            const freePlacement=nearest.s<=bif+16||!this.wallFit.connected(nearest.point,nose);
            const top=freePlacement?route.nearest(AORTIC_NECK).s:nearest.s;
            const split=top-dimensions.trunkLength;
            const trunk=route.section(top,split,2);
            const ipsi=route.section(split,split-dimensions.ipsiLength,2);
            const contraSplit=contra.nearest(route.sample(split)).s;
            const contraPath=contra.section(contraSplit,contraSplit-dimensions.contraLength,2);
            // Device crotch belongs to the selected model, not to the patient's
            // aortic bifurcation. Offset both outlets to form a visible short gate.
            const branchOffset=(points,sign)=>{
                for(let i=0;i<points.length;i++) {
                    const distance=i/(points.length-1)*(points===ipsi?dimensions.ipsiLength:dimensions.contraLength);
                    const own=route.sample(split-distance),opposite=contra.sample(contraSplit-distance);
                    // In the shared aortic lumen the two routes coincide. Keep
                    // the outlets side by side there, including the gate rim;
                    // taper the offset only as the iliac routes actually diverge.
                    const separation=Math.abs(opposite.x-own.x);
                    const shift=Math.max(0,(14.5-separation)/2);
                    points[i].x+=sign*shift;
                }
            };
            branchOffset(ipsi,side==='right'?-1:1);
            branchOffset(contraPath,side==='right'?1:-1);
            const overlap=trunk.at(-2).clone().sub(trunk.at(-1)).normalize().multiplyScalar(2);
            ipsi[0].add(overlap);contraPath[0].add(overlap);
            if(freePlacement) {
                const origin=trunk[0].clone(),from=origin.clone().sub(route.sample(top-2)).normalize();
                const to=nose.clone().sub(wire.sample(Math.max(0,d.position-2))).normalize();
                const rotation=from.lengthSq()&&to.lengthSq()?new THREE.Quaternion().setFromUnitVectors(from,to):new THREE.Quaternion();
                for(const points of [trunk,ipsi,contraPath])for(const point of points)
                    point.sub(origin).applyQuaternion(rotation).add(nose);
            }
            // A lateral guidewire position is not the released graft axis.
            // Pinning just this row to the wire produced a sharply tilted rim.
            if(freePlacement)trunk[0].copy(nose);
            d.crownPath=freePlacement
                ?new DevicePath(wire.section(d.position,d.position+12,1))
                :new DevicePath(route.section(top,Math.min(route.length,top+12),1));
            parts.push(this.buildPart(trunk,d.diameter/2));
            parts.push(this.buildPart(ipsi,d.distalDiameter/2));parts.push(this.buildPart(contraPath,7));
            for(const [i,part] of parts.entries()){
                const length=[dimensions.trunkLength,dimensions.ipsiLength,dimensions.contraLength][i];
                part.path=new DevicePath(part.points,part.points.map((_,j)=>length*j/(part.rows-1)));
            }
            const gatePath=new DevicePath(contraPath);
            d.gate={side:otherSide(side),docking:gatePath.sample(gatePath.length-10),
                entry:gatePath.sample(gatePath.length),routeS:contraSplit-dimensions.contraLength+10,parent:d,freePlacement};
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
        d.deliveryPath=new DevicePath(wire.points,wire.coordinates);
        initializeRelease(d);
        // Axis through the trunk and delivery-side route provides a stable
        // local frame for commanded roll before the fixation is detached.
        captureGraftPose(d,new DevicePath([...parts[0].points,...(parts[1]?.points.slice(1)??[])]));
        if(d.graftRotation)rotateGraftPose(d,d.graftRotation);
        d.contactFaces=preparePartialSurface(d);
        if(d.type==='body')this.metal.add(createOrientationMarker(parts[0],this.markerMaterial));
        for(const part of parts) {
            part.folded=part.path.coordinates.map(s=>d.deliveryPath.sample(d.position-part.releaseOffset-s));
            this.expandPart(part,d);
        }
        if(d.type==='body') {
            d.crown=createSuprarenalCrown(this.metalMaterial);this.metal.add(d.crown);updateSuprarenalCrown(d);
        }
        this.implants.push(d);this.geometryRevision++;
        this.accesses[side].message='Przytrzymaj „Zsuń koszulkę”, aby odsłaniać stentgraft.';
        return success();
    }
    removeDelivery(side) {
        const access=this.accesses[side],d=access.device;
        if(!d)return success();
        if(d.phase==='deploying')return fail('Poczekaj na zakończenie rozkładania.');
        if(d.position>.5){d.target=0;return fail('System jest wycofywany. Usuń go po osiągnięciu 0 cm.');}
        disposeFoldedGraftPreview(d);
        if(d.deliveryMesh){this.delivery.remove(d.deliveryMesh);d.deliveryMesh.geometry.dispose();}
        for(const key of ['noseMarker','sheathMarker','rotationMarker'])if(d[key]){this.delivery.remove(d[key]);d[key].geometry.dispose();}
        access.device=null;access.message='System usunięty; rozłożony implant pozostaje w naczyniu.';return success();
    }
    updateAccess(side,dt,committedSource=null,command=null) {
        // Capture after the rod transaction commits. Cooperative Newton trials
        // can change feed coordinates before committing a matching wire pose.
        if(committedSource)this.committedSources[side]={path:wireDevicePath(committedSource),catheterMm:committedSource.catheterMm};
        const access=this.accesses[side],d=access.device;if(!d||!(dt>0))return;
        // Sample the catheter controls with the rod step; consume them only
        // after it commits. Rejected/suspended steps cannot advance delivery.
        const mechanical=command?.deviceId===d.id&&Number.isFinite(command.mechanicalPosition);
        if(mechanical)d.position=d.target=Math.max(0,command.mechanicalPosition);
        if(!mechanical&&command?.deviceId===d.id) {
            const advance=THREE.MathUtils.clamp(Number(command.advance)||0,-1,1);
            if(advance>0) {
                this.setPosition(side,d.position+25*dt*advance);
                d.target=Math.max(d.position,d.target);
            }
            else d.target=Math.max(0,d.position+25*dt*advance);
        }
        if(!mechanical) {
            const wire=this.getPath(side);
            const target=d.target;
            // A withdrawn guidewire cannot pull an implanted graft with it.
            if(target>d.position&&wire.length<target+12){access.message='Brak podparcia prowadnikiem — wsuwanie zatrzymane.';return;}
            const move=Math.sign(target-d.position)*Math.min(Math.abs(target-d.position),25*dt);
            if(this.catheterPosition(side)>1&&move>0){access.message='Wycofaj cewnik przed wsuwaniem systemu.';return;}
            d.position=Math.max(0,d.position+move);
        }
        const control=command?.deviceId===d.id?command.release:null;
        if(command?.deviceId===d.id&&Number.isFinite(command.mechanicalRotation))d.deliveryRotation=command.mechanicalRotation;
        if(graftAttached(d)&&(Math.abs(d.deliveryRotation-d.graftRotation)>1e-9||
            d.parts.length&&Math.abs(d.position-d.implantPosition)>1e-9)) {
            if(d.parts.length) {
                rotateGraftPose(d,d.deliveryRotation,d.position);
                updatePartialSurfacePose(d);
                if(d.gate)d.gate.routeS=this.routes[otherSide(side)].nearest(d.gate.docking).s;
            } else d.graftRotation=d.deliveryRotation;
        }
        if(d.phase==='deployed')advanceRelease(d,dt,control);
        if(d.phase==='deploying') {
            advanceRelease(d,dt,control);
            d.deliveryPath=this.getPath(side);
            if(!graftAttached(d))d.detachedPath??=new DevicePath(d.deliveryPath.points,d.deliveryPath.coordinates);
            const graftPath=d.detachedPath??d.deliveryPath;
            for(const part of d.parts){
                const folded=part.path.coordinates.map(s=>graftPath.sample(d.implantPosition-part.releaseOffset-s));
                for(let i=0;i<part.rows;i++)if(part.exposure[i]<1 &&
                    [Math.max(0,i-1),i,Math.min(part.rows-1,i+1)].some(j=>folded[j].distanceToSquared(part.folded[j])>1e-12))part.exposure[i]=-1;
                part.folded=folded;this.expandPart(part,d);
            }
            if(d.crown)updateSuprarenalCrown(d);
            if(d.gate)d.gate.marker.visible=graftCoverWithdrawal(d)>=d.gateTravel;
            if(d.deployment===1){
                d.phase='deployed';if(d.gate)d.gate.marker.visible=true;
                access.message=d.type==='body'?'Korpus rozłożony. Otwarta bramka — dołącz nóżkę z przeciwnej koszulki.':
                    d.parentId!==null?'Nóżka połączona. Kontrast płynie światłem stentgraftu.':'Nóżka rozłożona bez połączenia z korpusem.';
                this.geometryRevision++;
                this.surface=new StentGraftSurface(this.implants.filter(i=>i.phase==='deployed'),this.geometryRevision);
                // Previous snapshots can still belong to a cooperative Newton step.
                this.surfaces.push(this.surface);
            }
            this.refreshMechanicalSurface();
        }
    }

    refreshMechanicalSurface() {
        const devices=this.implants.filter(d=>d.phase==='deploying');
        // Rebuild only when a complete fabric face becomes exposed, not on
        // every Newton step. Old snapshots remain immutable for suspended steps.
        const key=[this.surface?.revision??0,...devices.map(d=>`${d.id}:${d.poseRevision??0}:${d.contactFaces.filter(f=>f.distance<=graftCoverWithdrawal(d)-d.coverLead-2).length}`)].join('|');
        if(key===this.mechanicalSurfaceKey)return;
        this.mechanicalSurfaceKey=key;
        this.lumenSections={right:[],left:[]};
        for(const device of this.implants)this.lumenSections[device.side].push(...graftLumenSections(device));
        const previous=this.mechanicalSurface;
        this.mechanicalSurface=devices.length?partialSurfaceSnapshot(devices,this.surface,++this.geometryRevision):this.surface;
        if(previous&&previous!==this.surface&&!this.surfaces.includes(previous))previous.geometry.dispose();
    }
    mechanicalSurfaceForAccess(side) {
        const surface=this.mechanicalSurface??this.surface;
        return surface?{...surface,lumenSections:this.lumenSections?.[side]??[]}:null;
    }
    refreshDelivery(side) {
        const d=this.accesses[side].device;if(!d)return;
        if(d.position<1){setFoldedPreviewVisible(d.foldedPreview,false);for(const key of ['deliveryMesh','noseMarker','sheathMarker','rotationMarker'])if(d[key])d[key].visible=false;return;}
        const wire=d.phase==='deploying'?d.deliveryPath:this.getPath(side);
        if(wire.length<d.position)return;
        if(d.phase==='loaded'&&wire.points.length>=2){
            d.foldedPreview??=createFoldedGraftPreview(d,this);
            updateFoldedGraftPreview(d.foldedPreview,wire,d.position,d.graftRotation);
            setFoldedPreviewVisible(d.foldedPreview,true);
        }
        const lead=d.type==='body'?12:0;
        const tip=deliveryNoseState(d).position;
        const travel=d.sheathWithdrawal??0;
        const edge=Math.max(0,d.position+lead-travel);
        const start=Math.max(0,edge-160);
        const tube=(key,from,to,radius,material)=>{
            if(to-from<.05){if(d[key])d[key].visible=false;return;}
            const points=wire.section(from,to,3),curve=new THREE.CatmullRomCurve3(points,false,'centripetal');
            const geometry=new THREE.TubeGeometry(curve,Math.max(4,points.length*2),radius,10,false);
            if(!d[key]){d[key]=new THREE.Mesh(geometry,material);this.delivery.add(d[key]);}
            else {d[key].geometry.dispose();d[key].geometry=geometry;}d[key].visible=true;
        };
        tube('deliveryMesh',start,tip,.8,this.deliveryMaterial);
        // The radiolucent graft cover is invisible in both debug and X-ray.
        // Only its distal marker is rendered; the inner delivery shaft remains.
        if(!d.sheathMarker){d.sheathMarker=new THREE.Mesh(new THREE.TorusGeometry(d.type==='body'?3:2.4,.35,6,24),this.markerMaterial);this.delivery.add(d.sheathMarker);}
        d.sheathMarker.position.copy(wire.sample(edge));
        const tangent=wire.sample(Math.min(wire.length,edge+1)).sub(wire.sample(Math.max(0,edge-1))).normalize();
        d.sheathMarker.quaternion.setFromUnitVectors(new THREE.Vector3(0,0,1),tangent);d.sheathMarker.visible=edge>0;
        const noseGeometry=createFlexibleNoseGeometry(wire,tip);
        if(!d.noseMarker){d.noseMarker=new THREE.Mesh(noseGeometry,this.deliveryMaterial);this.delivery.add(d.noseMarker);}
        else {d.noseMarker.geometry.dispose();d.noseMarker.geometry=noseGeometry;}
        d.noseMarker.visible=true;
        // An eccentric marker makes shaft roll visible even after the implant
        // is fixed (the delivery shaft and nose themselves are circular).
        if(!d.rotationMarker){d.rotationMarker=new THREE.Mesh(new THREE.SphereGeometry(.45,8,6),this.markerMaterial);this.delivery.add(d.rotationMarker);}
        const noseTangent=wire.sample(tip).sub(wire.sample(Math.max(0,tip-1))).normalize();
        const radial=new THREE.Vector3(Math.abs(noseTangent.z)<.9?0:1,0,Math.abs(noseTangent.z)<.9?1:0).cross(noseTangent).normalize().applyAxisAngle(noseTangent,d.deliveryRotation);
        d.rotationMarker.position.copy(wire.sample(tip)).addScaledVector(radial,2.5);d.rotationMarker.visible=true;
    }
    buildPart(points,radius) {
        const rows=points.length,sides=24,positions=new Float32Array(rows*sides*3),target=new Float32Array(positions.length),indices=[];
        relaxGraftAxis(points,this.wallFit);
        for(let i=0;i<rows-1;i++)for(let j=0;j<sides;j++) {
            const a=i*sides+j,b=i*sides+(j+1)%sides,c=a+sides,e=b+sides;indices.push(a,c,b,b,c,e);
        }
        const geometry=new THREE.BufferGeometry();geometry.setAttribute('position',new THREE.BufferAttribute(positions,3));geometry.setIndex(indices);
        const mesh=new THREE.Mesh(geometry,this.fabricMaterial);mesh.frustumCulled=false;this.fabric.add(mesh);
        const part={points,rows,sides,radius,target,mesh,path:new DevicePath(points),exposure:new Float64Array(rows).fill(-1)};
        fitExpandedGraft(part,this.wallFit);
        part.path=new DevicePath(points);
        for(const object of createScaffold(part,this.metalMaterial,this.markerMaterial))this.metal.add(object);
        return part;
    }
    expandPart(part,device) {
        const positions=part.mesh.geometry.attributes.position;
        let changed=false;
        for(let i=0;i<part.rows;i++) {
            const local=rowExposure(device,part.releaseOffset+part.path.coordinates[i]),folded=part.folded[i];
            if(part.exposure[i]===local)continue;
            part.exposure[i]=local;changed=true;
            const tangent=part.folded[Math.min(part.rows-1,i+1)].clone().sub(part.folded[Math.max(0,i-1)]).normalize();
            if(tangent.lengthSq()<.01)tangent.copy(part.points[1]).sub(part.points[0]).normalize();
            const axis=Math.abs(tangent.z)<.9?new THREE.Vector3(0,0,1):new THREE.Vector3(1,0,0);
            const u=axis.cross(tangent).normalize(),v=tangent.clone().cross(u).normalize();
            for(let j=0;j<part.sides;j++) {
                const k=(i*part.sides+j)*3,angle=j/part.sides*Math.PI*2-(device.graftRotation??0);
                const initial=folded.clone().addScaledVector(u,1.7*Math.cos(angle)).addScaledVector(v,1.7*Math.sin(angle));
                const q=initial.lerp(new THREE.Vector3().fromArray(part.target,k),local);
                if(local<1)device.wallFit.fit(q,folded.clone().lerp(part.points[i],local),.7);
                positions.setXYZ(i*part.sides+j,q.x,q.y,q.z);
            }
        }
        if(changed){positions.needsUpdate=true;updateScaffold(part);}
    }
    setFluoroscopy(enabled){this.fabric.visible=!enabled;}
    snapshot(side) {
        const access=this.accesses[side],d=access.device;
        return {device:d,limit:Math.max(0,this.getPath(side).length-12),message:access.message,
            validation:d?.phase==='loaded'?this.validation(side):null,implants:this.implants};
    }
    dispose() {
        if(this.mechanicalSurface&&!this.surfaces.includes(this.mechanicalSurface))this.mechanicalSurface.geometry.dispose();
        for(const surface of this.surfaces)surface.dispose();
        this.group.traverse(o=>{o.geometry?.dispose();disposeWire(o);if(o.isInstancedMesh)o.dispose();if(o.isLineSegments)o.material.dispose();});
        this.fabricMaterial.dispose();this.metalMaterial.dispose();this.deliveryMaterial.dispose();this.markerMaterial.dispose();this.group.clear();
    }
}
