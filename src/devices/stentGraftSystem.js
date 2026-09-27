import {graftOwnedBranches} from './stentGraftBranchContact.js';
import {captureConfiguration,capturedRoot,captureLinks} from './stentGraftCapture.js';
import {calibrateStentGrafts} from './stentGraftCalibration.js';
import {graftDisplacementSampler} from './stentGraftCompliance.js';
import {fitGraftJunction,fitIIsTrunkSection} from './stentGraftJunction.js';
import {packedLayout,packedFrames,releaseSection,constrainReleaseCenters,separateReleaseBranches} from './stentGraftReleaseShape.js';
import {relaxGraftAxis,fitExpandedGraft,graftRestAxes} from './stentGraftExpansion.js';
import {disposeWire} from './stentGraftWire.js';
import {mainBodyModel,limbModel,LIMB_MODELS,LIMB_DISTAL_DIAMETERS,worldBodyDimensions,graftScale,nominalPartRadius,proximalDiameters,distalDiameters,deliveryRadiusMm} from './stentGraftModels.js';
import {captureGraftPose,rotateGraftPose} from './stentGraftRotation.js';
import {createFoldedGraftPreview,updateFoldedGraftPreview,disposeFoldedGraftPreview,setFoldedPreviewVisible} from './stentGraftFoldedPreview.js';
import * as THREE from 'three';
import {AORTIC_NECK,AORTIC_BIFURCATION,DevicePath,wireDevicePath,createAorticRoutes} from './stentGraftPaths.js';
import {createFlexibleNoseGeometry,NOSECONE_LENGTH_MM,NOSECONE_BASE_RADIUS_MM} from './stentGraftNose.js';
import {graftLumenSections} from './stentGraftLumenContact.js';
import {preparePartialSurface,partialSurfaceSnapshot,updatePartialSurfacePose} from './stentGraftPartialSurface.js';
import {StentGraftSurface} from './stentGraftSurface.js';
import {StentGraftWallFit} from './stentGraftWallFit.js';
import {initializeRelease,advanceRelease,partExposure,deliveryNoseState,graftAttached,graftFaceExposed} from './stentGraftDeployment.js';
import {createScaffold,updateScaffold,createSuprarenalCrown,updateSuprarenalCrown,createOrientationMarker,createGateMarker} from './stentGraftScaffold.js';

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
        this.contactPatches={right:[],left:[]};
        this.contactSignatures={right:"[]",left:"[]"};this.contactRevisions={right:0,left:0};
        this.surface=null;this.mechanicalSurface=null;this.surfaces=[];
        this.implants=[];this.nextId=1;this.routes=null;
        this.group=new THREE.Group();this.group.name='stent-grafts';
        this.fabric=new THREE.Group();this.metal=new THREE.Group();this.delivery=new THREE.Group();
        this.group.add(this.fabric,this.metal,this.delivery);
        this.fabricMaterial=new THREE.MeshBasicMaterial({color:0xeee8dc,transparent:true,opacity:.38,side:THREE.DoubleSide,depthWrite:false});
        this.metalMaterial=new THREE.MeshBasicMaterial({color:0xc6ced5});
        this.deliveryMaterial=new THREE.MeshBasicMaterial({color:0xca8ce8});
        this.noseMaterial=new THREE.MeshBasicMaterial({color:0xca8ce8,transparent:true,opacity:.3,depthWrite:false});
        this.noseProjectionMaterial=new THREE.MeshBasicMaterial({color:0xffffff,transparent:true,opacity:.16,depthWrite:false,toneMapped:false,side:THREE.FrontSide});
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
            this.calibration=calibrateStentGrafts(anatomy,this.routes.right);
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
        const model=type==='body'?mainBodyModel(modelId):limbModel(modelId);
        access.device={id:this.nextId++,type,side,position:0,target:0,phase:'loaded',deployment:0,
            dimensionScale:this.calibration?.worldUnitsPerMm??1,modelId:model.id,diameter:type==='body'?28:model.diameter,distalDiameter:type==='body'?(model.id==='iis-103'?14:16):model.distalDiameter,
            length:model.length,deliveryRotation:0,graftRotation:0,parts:[],deliveryMesh:null};
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
        if(d.type==='limb') {
            const distal=LIMB_DISTAL_DIAMETERS.reduce((a,b)=>Math.abs(b-value)<Math.abs(a-value)?b:a);
            const model=LIMB_MODELS.find(m=>m.distalDiameter===distal&&m.length===d.length);
            if(model)Object.assign(d,{modelId:model.id,diameter:model.diameter,distalDiameter:model.distalDiameter});
            return;
        }
        const sizes=proximalDiameters(d.modelId);
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
        d.target=d.position;d.implantPosition=d.position;d.deliveryWireReleased=false;
        const parts=[],scale=graftScale(d),length=d.length*scale;
        this.wallFit=new StentGraftWallFit(this.readAnatomy());
        d.wallFit=this.wallFit;
        if(d.type==='body') {
            const route=this.routes[side],contra=this.routes[otherSide(side)];
            const nearest=route.nearest(nose);
            const bif=route.nearest(AORTIC_BIFURCATION).s;
            const dimensions=worldBodyDimensions(d);
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
                    const shift=Math.max(0,(dimensions.crotchDiameter/2-separation)/2);
                    points[i].x+=sign*shift;
                }
            };
            branchOffset(ipsi,side==='right'?-1:1);
            branchOffset(contraPath,side==='right'?1:-1);
            const overlap=trunk.at(-2).clone().sub(trunk.at(-1)).normalize().multiplyScalar(2);
            ipsi[0].add(overlap);contraPath[0].add(overlap);
            graftRestAxes(trunk,ipsi,contraPath,dimensions,overlap,freePlacement?null:this.wallFit);
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
                ?new DevicePath(wire.section(d.position,d.position+12*scale,1))
                :new DevicePath(route.section(top,Math.min(route.length,top+12*scale),1));
            parts.push(this.buildPart(trunk,d.diameter*scale/2,dimensions.crotchDiameter/2));
            const gatePart=this.buildPart(contraPath,dimensions.gateDiameter/2);
            // The two sewn branches also constrain each other. Straightening
            // the long branch cannot make its axis collapse into the short gate.
            parts.push(this.buildPart(ipsi,d.distalDiameter*scale/2,dimensions.ipsiRootDiameter/2,dimensions.distalStraight,null,scale,
                {path:gatePart.path,radius:(dimensions.ipsiRootDiameter+dimensions.gateDiameter)/2}));
            parts.push(gatePart);
            for(const [i,part] of parts.entries()){
                const length=[dimensions.trunkLength,dimensions.ipsiLength,dimensions.contraLength][i];
                part.path=new DevicePath(part.points,part.points.map((_,j)=>length*j/(part.rows-1)));
            }
            if(d.modelId==='iis-103')fitIIsTrunkSection(parts,this.wallFit,dimensions.gateDiameter);
            fitGraftJunction(parts,this.wallFit);
            const gatePath=new DevicePath(contraPath);
            d.gate={side:otherSide(side),docking:gatePath.sample(gatePath.length-10*scale),
                entry:gatePath.sample(gatePath.length),routeS:contraSplit-dimensions.contraLength+10*scale,parent:d,freePlacement};
            d.gate.marker=createGateMarker(parts[2],this.markerMaterial);this.metal.add(d.gate.marker);
            d.parentId=null;
        } else {
            const gate=this.availableGate(side),route=this.routes[side];
            const connected=gate&&nose.distanceTo(gate.docking)<=10;
            const points=connected&&!gate.freePlacement
                ?route.section(gate.routeS,Math.max(0,gate.routeS-length),2)
                :wire.section(d.position,Math.max(0,d.position-length),2);
            points[0].copy(nose);
            const part=this.buildPart(points,d.diameter*scale/2,d.distalDiameter*scale/2,null,t=>nominalPartRadius(d,0,t*length));
            part.path=new DevicePath(part.points,part.points.map((_,i)=>length*i/(part.rows-1)));
            parts.push(part);
            d.parentId=connected?gate.parent.id:null;
            if(connected){gate.parent.connectedLimbId=d.id;gate.marker.material.color.copy(this.metalMaterial.color);}
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
        for(const [index,part] of parts.entries()) {
            part.dimensionScale=scale;
            // Keep the first branch wire off the sewn seam, while reserving
            // enough axial band for inextensible waves in the fitted gate.
            part.scaffoldInset=d.type==='body'&&index>0?.25*scale:0;
            part.nominalRadius=s=>nominalPartRadius(d,index,s);
            for(const object of createScaffold(part,this.metalMaterial,this.markerMaterial))this.metal.add(object);
            part.packedLayout=packedLayout(d.type,index,scale,d.side);
            part.folded=part.path.coordinates.map(s=>d.deliveryPath.sample(d.position-part.releaseOffset-s));
            this.expandPart(part,d);
        }
        separateReleaseBranches(d);
        for(const part of parts)updateScaffold(part);
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
        for(const key of ['noseMarker','noseBand','sheathMarker','rotationMarker'])if(d[key]){this.delivery.remove(d[key]);d[key].geometry.dispose();}
        access.device=null;access.message='System usunięty; rozłożony implant pozostaje w naczyniu.';return success();
    }
    updateAccess(side,dt,committedSource=null,command=null) {
        // Capture after the rod transaction commits. Cooperative Newton trials
        // can change feed coordinates before committing a matching wire pose.
        if(committedSource) {
            const path=wireDevicePath(committedSource);
            this.committedSources[side]={path,catheterMm:committedSource.catheterMm};
            // The original threading ceases when the wire is withdrawn from
            // the long outlet. A later wire is free to cannulate either portal.
            for(const implant of this.implants)if(implant.side===side&&implant.type==='body'&&!implant.deliveryWireReleased) {
                const part=implant.parts[1],entry=implant.implantPosition-part.releaseOffset-part.path.length;
                if(path.length<entry-2) {
                    implant.deliveryWireReleased=true;
                    this.mechanicalSurfaceKey=null;this.refreshMechanicalSurface();
                }
            }
        }
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
            // Once cloth leaves the cover, its release pose belongs to the
            // implant. Re-sampling it from the rod creates a feedback loop:
            // contact bends the rod, which bends the same contacting cloth.
            // Keep both position AND frame; deriving frames across a moving
            // covered/fixed exposed boundary would still twist released rings.
            for(const part of d.parts){
                const live=part.path.coordinates.map(s=>d.deliveryPath.sample(d.implantPosition-part.releaseOffset-s));
                const frames=packedFrames(live,d.graftRotation??0);
                part.foldedFrames??=packedFrames(part.folded,d.graftRotation??0);
                for(let i=0;i<part.rows;i++)if(part.exposure[i]<=0) {
                    if(part.folded[i].distanceToSquared(live[i])>1e-16||part.foldedFrames[i].u.distanceToSquared(frames[i].u)>1e-16)part.exposure[i]=-1;
                    part.folded[i]=live[i];part.foldedFrames[i]=frames[i];
                }
                this.expandPart(part,d);
            }
            separateReleaseBranches(d);
            for(const part of d.parts)updateScaffold(part);
            if(d.crown)updateSuprarenalCrown(d);
            // Collision faces and the owning lumen must follow the restrained
            // fabric, not the unrestrained expansion target behind it.
            const contactPoseKey=d.parts.map(part=>part.mesh.geometry.attributes.position.version).join('/');
            if(contactPoseKey!==d.contactPoseKey) {
                let changed=false;
                for(const part of d.parts) {
                    const firstCovered=part.exposure.findIndex(value=>value<=0),rows=firstCovered<0?part.rows:firstCovered;
                    const values=(part.contactBasePositions??part.mesh.geometry.attributes.position.array).subarray(0,rows*part.sides*3);
                    if(part.contactPosePositions?.length!==values.length||values.some((v,i)=>v!==part.contactPosePositions[i])) {
                        part.contactPosePositions=values.slice();changed=true;
                    }
                }
                if(changed){updatePartialSurfacePose(d,true);d.poseRevision=(d.poseRevision??0)+1;}
                d.contactPoseKey=contactPoseKey;
            }

            if(d.deployment===1&&d.releaseStage==='complete'){
                d.phase='deployed';if(d.gate)d.gate.marker.visible=true;
                access.message=d.type==='body'?'Korpus rozłożony. Otwarta bramka — dołącz nóżkę z przeciwnej koszulki.':
                    d.parentId!==null?'Nóżka połączona. Kontrast płynie światłem stentgraftu.':'Nóżka rozłożona bez połączenia z korpusem.';
                this.geometryRevision++;
                this.surface=new StentGraftSurface(this.implants.filter(i=>i.phase==='deployed'),this.geometryRevision);
                // Previous snapshots can still belong to a cooperative Newton step.
                this.surfaces.push(this.surface);
            }
            this.refreshMechanicalSurface();
            if(this.contactPatches.right.length||this.contactPatches.left.length)this.refreshContactIndentation();
        }
    }

    refreshMechanicalSurface() {
        const devices=this.implants.filter(d=>d.phase==='deploying');
        // Rebuild when exposed topology or actual fabric pose changes.
        // Old snapshots remain immutable for suspended Newton steps.
        const key=[this.surface?.revision??0,...devices.map(d=>`${d.id}:${d.poseRevision??0}:${d.gateOpening===1?1:0}:${d.contactFaces.filter(f=>graftFaceExposed(d,f)).length}`)].join('|');
        if(key===this.mechanicalSurfaceKey)return;
        this.mechanicalSurfaceKey=key;
        this.lumenSections={right:[],left:[]};
        this.ownedBranches={right:[],left:[]};
        for(const device of this.implants) {
            this.lumenSections[device.side].push(...graftLumenSections(device));
            this.ownedBranches[device.side].push(...graftOwnedBranches(device));
        }
        const previous=this.mechanicalSurface;
        this.mechanicalSurface=devices.length?partialSurfaceSnapshot(devices,this.surface,++this.geometryRevision):this.surface;
        if(previous&&previous!==this.surface&&!this.surfaces.includes(previous))previous.geometry.dispose();
    }
    captureForAccess(side) {
        const device=this.accesses[side].device;
        return device?captureLinks(device):null;
    }
    mechanicalSurfaceForAccess(side) {
        const surface=this.mechanicalSurface??this.surface;
        return surface?{...surface,revision:`${surface.revision}/${this.contactRevisions[otherSide(side)]}/${this.implants.map(d=>d.deliveryWireReleased?1:0).join('')}`,contains:surface.contains?.bind(surface),lumenSections:this.lumenSections?.[side]??[],ownedBranches:this.ownedBranches?.[side]??[],
            otherContactPatches:this.contactPatches[otherSide(side)],
            commitContactPatches:patches=>{
                const signature=JSON.stringify(patches.map(p=>[...p.point,...p.normal,p.depth,p.width].map(v=>Math.round(v*1e4))));
                if(signature===this.contactSignatures[side])return;
                this.contactSignatures[side]=signature;this.contactPatches[side]=patches;this.contactRevisions[side]++;
                this.refreshContactIndentation();
            }}:null;
    }
    refreshContactIndentation() {
        const patches=[...this.contactPatches.right,...this.contactPatches.left],point=new THREE.Vector3(),displace=graftDisplacementSampler(patches);
        for(const device of this.implants)for(const part of device.parts) {
            const positions=part.mesh.geometry.attributes.position;let changed=false;
            const capture=part===device.parts[0]?captureConfiguration(device):null;
            // Preserve the currently released cloth, including its sewn-axis
            // constraint. Clearing indentation must not jump back to the
            // unconstrained, fully expanded target during partial deployment.
            part.contactBasePositions??=positions.array.slice();
            for(let i=0;i<part.rows;i++)if(part.exposure[i]===1)for(let j=0;j<part.sides;j++) {
                const index=i*part.sides+j;
                point.fromArray(part.contactBasePositions,index*3);
                displace(point);
                if(capture&&i===0&&patches.length&&point.distanceToSquared(new THREE.Vector3().fromArray(part.contactBasePositions,index*3))>1e-20)
                    point.copy(capturedRoot(point,capture.latch,capture.material.armLength));
                if(positions.getX(index)!==Math.fround(point.x)||positions.getY(index)!==Math.fround(point.y)||positions.getZ(index)!==Math.fround(point.z)) {
                    positions.setXYZ(index,point.x,point.y,point.z);changed=true;
                }
            }
            if(changed){positions.needsUpdate=true;updateScaffold(part);}
            if(!patches.length)delete part.contactBasePositions;
        }
        for(const device of this.implants)if(device.crown)updateSuprarenalCrown(device);
    }
    refreshDelivery(side) {
        const d=this.accesses[side].device;if(!d)return;
        if(d.position<1){setFoldedPreviewVisible(d.foldedPreview,false);for(const key of ['deliveryMesh','noseMarker','noseBand','sheathMarker','rotationMarker'])if(d[key])d[key].visible=false;return;}
        const wire=d.phase==='deploying'?d.deliveryPath:this.getPath(side);
        if(wire.length<d.position)return;
        if(d.phase==='loaded'&&wire.points.length>=2){
            d.foldedPreview??=createFoldedGraftPreview(d,this);
            updateFoldedGraftPreview(d.foldedPreview,wire,d.position,d.graftRotation);
            setFoldedPreviewVisible(d.foldedPreview,true);
        }
        const lead=d.type==='body'?12*graftScale(d):0;
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
        const sheathRadius=deliveryRadiusMm(d);
        if(d.sheathMarker&&d.sheathMarker.geometry.parameters.radius!==sheathRadius) {
            d.sheathMarker.geometry.dispose();
            d.sheathMarker.geometry=new THREE.TorusGeometry(sheathRadius,.35,6,24);
        }
        if(!d.sheathMarker){d.sheathMarker=new THREE.Mesh(new THREE.TorusGeometry(sheathRadius,.35,6,24),this.markerMaterial);this.delivery.add(d.sheathMarker);}
        d.sheathMarker.position.copy(wire.sample(edge));
        const tangent=wire.sample(Math.min(wire.length,edge+1)).sub(wire.sample(Math.max(0,edge-1))).normalize();
        d.sheathMarker.quaternion.setFromUnitVectors(new THREE.Vector3(0,0,1),tangent);d.sheathMarker.visible=edge>0;
        const noseGeometry=createFlexibleNoseGeometry(wire,tip,NOSECONE_LENGTH_MM*graftScale(d),NOSECONE_BASE_RADIUS_MM*graftScale(d));
        if(!d.noseMarker){d.noseMarker=new THREE.Mesh(noseGeometry,this.noseMaterial);this.delivery.add(d.noseMarker);}
        else {d.noseMarker.geometry.dispose();d.noseMarker.geometry=noseGeometry;}
        d.noseMarker.userData.projectionMaterial=this.noseProjectionMaterial;
        d.noseMarker.visible=true;
        if(!d.noseBand){d.noseBand=new THREE.Mesh(new THREE.TorusGeometry(NOSECONE_BASE_RADIUS_MM*graftScale(d),.22,6,24),this.markerMaterial);this.delivery.add(d.noseBand);}
        d.noseBand.position.copy(wire.sample(tip));
        const noseAxis=wire.sample(tip).sub(wire.sample(Math.max(0,tip-1))).normalize();
        d.noseBand.quaternion.setFromUnitVectors(new THREE.Vector3(0,0,1),noseAxis);d.noseBand.visible=true;
        // An eccentric marker makes shaft roll visible even after the implant
        // is fixed (the delivery shaft and nose themselves are circular).
        if(!d.rotationMarker){d.rotationMarker=new THREE.Mesh(new THREE.SphereGeometry(.45,8,6),this.markerMaterial);this.delivery.add(d.rotationMarker);}
        const noseTangent=wire.sample(tip).sub(wire.sample(Math.max(0,tip-1))).normalize();
        const radial=new THREE.Vector3(Math.abs(noseTangent.z)<.9?0:1,0,Math.abs(noseTangent.z)<.9?1:0).cross(noseTangent).normalize().applyAxisAngle(noseTangent,d.deliveryRotation);
        d.rotationMarker.position.copy(wire.sample(tip)).addScaledVector(radial,2.5);d.rotationMarker.visible=true;
    }
    buildPart(points,radius,endRadius=radius,distalStraight=null,radiusProfile=null,dimensionScale=1,oppositeBranch=null) {
        const rows=points.length,sides=24,positions=new Float32Array(rows*sides*3),target=new Float32Array(positions.length),indices=[];
        relaxGraftAxis(points,this.wallFit,oppositeBranch);
        for(let i=0;i<rows-1;i++)for(let j=0;j<sides;j++) {
            const a=i*sides+j,b=i*sides+(j+1)%sides,c=a+sides,e=b+sides;indices.push(a,c,b,b,c,e);
        }
        const geometry=new THREE.BufferGeometry();geometry.setAttribute('position',new THREE.BufferAttribute(positions,3));geometry.setIndex(indices);
        const mesh=new THREE.Mesh(geometry,this.fabricMaterial);mesh.frustumCulled=false;this.fabric.add(mesh);
        const part={points,rows,sides,radius,target,mesh,path:new DevicePath(points),exposure:new Float64Array(rows).fill(-1)};
        part.rowRadii=Float64Array.from(points,(_,i)=>{
            if(radiusProfile)return radiusProfile(i/(rows-1));
            const t=distalStraight===null?i/(rows-1)
                :THREE.MathUtils.clamp((part.path.coordinates[i]-(part.path.length-distalStraight-10*dimensionScale))/(10*dimensionScale),0,1);
            const smooth=t*t*t*(10-15*t+6*t*t);
            return distalStraight===null?radius+(endRadius-radius)*smooth:endRadius+(radius-endRadius)*smooth;
        });
        fitExpandedGraft(part,this.wallFit);
        part.path=new DevicePath(points);
        return part;
    }
    expandPart(part,device) {
        part.releaseShapeChanged=false;
        const positions=part.mesh.geometry.attributes.position;
        let geometryChanged=false;
        const frames=part.foldedFrames??packedFrames(part.folded,device.graftRotation??0);
        const capture=part===device.parts[0]?captureConfiguration(device):null;
        if(capture) {
            const section=releaseSection(part,0,partExposure(device,part,0),frames[0],part.packedLayout);
            device.captureRestRoots=Array.from({length:part.sides},(_,j)=>section.point(j));
        }
        const rootShift=capture?device.captureRestRoots.map(p=>capturedRoot(p,capture.latch,capture.material.armLength).sub(p)):null;
        const openings=part.path.coordinates.map(s=>partExposure(device,part,s));
        const settled=!capture&&!part.wasCaptured&&!part.releasePoseDirty&&openings.every((v,i)=>v===part.exposure[i]||v===0);
        if(settled&&openings.every((v,i)=>v===part.exposure[i]))return;
        part.releaseShapeChanged=!settled;
        const sections=constrainReleaseCenters(part,openings.map((local,i)=>releaseSection(part,i,local,frames[i],part.packedLayout)),openings);
        for(let i=0;i<part.rows;i++) {
            const local=openings[i];
            if(settled&&local>0)continue;
            part.exposure[i]=local;
            const section=sections[i];
            for(let j=0;j<part.sides;j++) {
                const q=section.point(j);
                if(rootShift) {
                    const t=Math.max(0,1-part.path.coordinates[i]/(20*(device.dimensionScale??1)));
                    q.addScaledVector(rootShift[j],t*t*(3-2*t));
                }
                if(local>0&&local<1)device.wallFit.fit(q,section.center,.7);
                const index=i*part.sides+j;
                if(part.contactBasePositions)q.toArray(part.contactBasePositions,index*3);
                if(positions.getX(index)!==Math.fround(q.x)||positions.getY(index)!==Math.fround(q.y)||positions.getZ(index)!==Math.fround(q.z)) {
                    positions.setXYZ(index,q.x,q.y,q.z);geometryChanged=true;
                }
            }
        }
        part.wasCaptured=!!capture;part.releasePoseDirty=false;
        if(geometryChanged)positions.needsUpdate=true;

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
        this.noseMaterial.dispose();this.noseProjectionMaterial.dispose();this.fabricMaterial.dispose();this.metalMaterial.dispose();this.deliveryMaterial.dispose();this.markerMaterial.dispose();this.group.clear();
    }
}
