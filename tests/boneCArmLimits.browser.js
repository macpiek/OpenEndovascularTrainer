import * as T from 'three';
import { MeshBVH } from 'three-mesh-bvh';
import { createBoneModel } from '../src/boneModel.js';
import { generateVessel } from '../src/vesselGeometry.js';
import { applyAnteriorCArmPose } from '../src/carmPose.js';
import { applyCArmProjection, CARM_SOURCE_TO_ISOCENTER_MM } from '../src/imaging/carmProjection.js';
import { createBoneProjectionMaterial, BONE_DEPTH_SCALE } from '../src/imaging/boneProjection.js';
import { createAnatomyProjectionCamera } from '../src/imaging/anatomyProjectionCamera.js';

const result = document.querySelector('#results');
const query = new URLSearchParams(location.search);
const size = Number(query.get('size')) || 96;
const records = [], meshes = [], referenceMeshes = [];
const material = createBoneProjectionMaterial();
const scene = new T.Scene();
const camera = new T.PerspectiveCamera(45, 1, .1, 700);
const caster = new T.Raycaster(), ndc = new T.Vector2(), localDirection = new T.Vector3();
let renderer, target, pixels, anatomyProjection;
const assert = (value, message) => { if (!value) throw Error(message); };
const round = value => Number(value.toFixed(5));
const frame = () => new Promise(requestAnimationFrame);

function casesFor(sliders) {
    const cases = [];
    const heights = [sliders.carmZ.min, sliders.carmZ.value, sliders.carmZ.max];
    const fields = [{name:'Miednica',y:-390}, {name:'Brzuch',y:-200}, {name:'Klatka',y:100}];
    const add = (region, height, yaw, pitch, roll, extras = {}) => cases.push({
        region:region.name, x:0, y:region.y, height, yaw, pitch, roll,
        detector:sliders.carmDetDist.value, side:300, zoom:1, ...extras
    });
    if (query.get('suite')==='table') {
        for(const height of heights) for(const yaw of [50,-50])
            for(const region of fields) for(let x=-200;x<=200;x+=50)
                add(region,height,yaw,0,0,{x});
        return cases;
    }
    // Complete corner/centre combinations, including both height end stops.
    for (const region of fields) for (const height of heights)
        for (const yaw of [-90,0,90]) for (const pitch of [-45,0,45])
            for (const roll of [-90,0,90]) add(region,height,yaw,pitch,roll);
    // Oblique views and the originally reported detector-clipping pose.
    for (const region of fields) for (const height of heights)
        for (const yaw of [-60,30,60]) add(region,height,yaw,yaw===30?10:-35,-5);
    // Physical detector end stops and all selectable magnifications/panel sizes.
    for (const detector of [sliders.carmDetDist.min,sliders.carmDetDist.max])
        for (const height of [heights[0],heights[2]]) for (const side of [210,300])
            for (const zoom of [1,1.4,2.1])
                add(fields[1],height,30,10,-5,{detector,side,zoom});
    // Move vertically through the range at fixed extreme compound angles.
    for (const yaw of [-90,90]) for (let step=0;step<=10;step++)
        add(fields[1],heights[0]+(heights[2]-heights[0])*step/10,yaw,45,90);
    // A combined sweep also exercises intermediate angles/heights and returns
    // through AP, rather than checking only independent static end positions.
    for (let step=0;step<=60;step++) {
        const t=step/60;
        add(fields[1],heights[0]+(heights[2]-heights[0])*t,
            -90+180*t,45*Math.cos(t*Math.PI*2),-90+180*t);
    }
    return cases;
}

function oracle() {
    const expected = new Float32Array(size*size);
    const full = new Float32Array(size*size);
    let unmatched = 0, clipped = 0, maxIntersections = 0;
    const unmatchedExamples=[];
    for (let y=0;y<size;y++) for (let x=0;x<size;x++) {
        caster.setFromCamera(ndc.set((x+.5)/size*2-1,(y+.5)/size*2-1),camera);
        const dz = -localDirection.copy(caster.ray.direction).transformDirection(camera.matrixWorldInverse).z;
        const near = camera.near/dz, far = camera.far/dz;
        let length = 0, fullLength=0;
        for (const {tree} of referenceMeshes) {
            const hits = tree.raycast(caster.ray,T.DoubleSide).sort((a,b)=>a.distance-b.distance);
            const unique = [];
            for (const hit of hits) {
                const previous=unique.at(-1);
                // Only duplicate hits on the same-facing shared edge cancel.
                // Very close opposing faces can enclose a genuinely thin layer.
                if (!previous || hit.distance-previous.distance>1e-5 ||
                    Math.sign(hit.face.normal.dot(caster.ray.direction))!==
                    Math.sign(previous.face.normal.dot(caster.ray.direction))) unique.push(hit);
            }
            maxIntersections = Math.max(maxIntersections,unique.length);
            // A camera inside a solid starts with an exit, so include [0, exit].
            let previous = 0, depth = 0;
            if (unique.length && unique[0].face.normal.dot(caster.ray.direction)>0) depth=1;
            for (const hit of unique) {
                // Sum separate overlapping bones just like X-ray attenuation;
                // a union of their volumes would underestimate the reference.
                length += depth*Math.max(0,Math.min(hit.distance,far)-Math.max(previous,near));
                fullLength += depth*Math.max(0,hit.distance-Math.max(previous,near));
                if (depth>0 && previous<far && hit.distance>far) clipped++;
                if (hit.face.normal.dot(caster.ray.direction)<0) {
                    depth++;
                } else {
                    depth--;
                }
                previous=hit.distance;
            }
            if (depth!==0) {
                unmatched++;
                if(unmatchedExamples.length<4)unmatchedExamples.push({x,y,depth,
                    intersections:unique.map(hit=>({distance:hit.distance,
                        facing:hit.face.normal.dot(caster.ray.direction),face:hit.faceIndex}))});
            }
        }
        expected[x+y*size] = length;
        full[x+y*size] = fullLength;
    }
    return {expected,full,unmatched,unmatchedExamples,clipped,maxIntersections};
}

function draw(detectorClipping = query.get('projection')==='detector') {
    const projectionCamera=detectorClipping ? camera : anatomyProjection.update();
    renderer.setRenderTarget(target);
    renderer.setClearColor(0,0);
    renderer.clear();
    renderer.render(scene,projectionCamera);
    renderer.readRenderTargetPixels(target,0,0,size,size,pixels);
    assert(!renderer.getContext().isContextLost(),'Utrata kontekstu GPU');
    return Float32Array.from({length:size*size},(_,i)=>
        Math.max(0,T.DataUtils.fromHalfFloat(pixels[i*4+1])-T.DataUtils.fromHalfFloat(pixels[i*4]))/BONE_DEPTH_SCALE);
}

function compare(expected,actual) {
    const errors=[],lost=[],negative=[];
    let checked=0;
    for (let y=1;y<size-1;y++) for (let x=1;x<size-1;x++) {
        const i=x+y*size;
        // Ignore one-pixel silhouette differences; measure the solid interiors.
        if (expected[i]<=3 || [i-1,i+1,i-size,i+size].some(j=>expected[j]<=3)) continue;
        checked++;
        if (actual[i]<.1) lost.push({x,y,expected:round(expected[i]),actual:round(actual[i])});
        if (T.DataUtils.fromHalfFloat(pixels[i*4+1])<T.DataUtils.fromHalfFloat(pixels[i*4])) negative.push({x,y});
        errors.push(Math.abs(actual[i]-expected[i]));
    }
    errors.sort((a,b)=>a-b);
    return {checked,lost:lost.length,lostExamples:lost.slice(0,8),negative:negative.length,
        p95Mm:round(errors[Math.floor(errors.length*.95)]||0),maxMm:round(errors.at(-1)||0)};
}

function preview(values,caption) {
    const canvas=document.createElement('canvas');canvas.width=canvas.height=size;
    const context=canvas.getContext('2d'),data=context.createImageData(size,size);
    for(let y=0;y<size;y++)for(let x=0;x<size;x++){
        const grey=245*Math.exp(-values[x+y*size]*.018);
        data.data.set([grey,grey,grey,255],(x+(size-1-y)*size)*4);
    }
    context.putImageData(data,0,0);
    const figure=document.createElement('figure'),label=document.createElement('figcaption');
    label.textContent=caption;figure.append(canvas,label);document.querySelector('main').append(figure);
}

try {
    const app = new DOMParser().parseFromString(await (await fetch('/')).text(),'text/html');
    const sliders = Object.fromEntries(['carmZ','carmDetDist'].map(id=>[id,
        Object.fromEntries(['value','min','max'].map(key=>[key,Number(app.getElementById(id).getAttribute(key))]))]));
    const bone = createBoneModel();
    await bone.ready;
    assert(bone.group.children.length,'Nie wczytano modelu kości');
    const {branchPoint} = generateVessel().vessel;
    bone.group.position.set(branchPoint.x,branchPoint.y-60,branchPoint.z-50);
    scene.add(bone.group);scene.updateMatrixWorld(true);
    anatomyProjection=createAnatomyProjectionCamera(camera,bone.group);
    bone.group.traverse(mesh=>{
        if(!mesh.isMesh)return;
        mesh.material=material;meshes.push(mesh);
        const geometry=mesh.geometry.clone().applyMatrix4(mesh.matrixWorld);
        referenceMeshes.push({geometry,tree:new MeshBVH(geometry,{indirect:true})});
    });
    bone.material.dispose();
    renderer=new T.WebGLRenderer({antialias:false});renderer.setSize(size,size);
    target=new T.WebGLRenderTarget(size,size,{type:T.HalfFloatType,depthBuffer:false});
    pixels=new Uint16Array(size*size*4);
    let cases=casesFor(sliders).map((pose,caseIndex)=>({...pose,caseIndex}));
    if(query.has('case')) cases=query.get('case').split(',').map(index=>cases[Number(index)]);
    const started=performance.now();
    for (const [index,pose] of cases.entries()) {
        applyAnteriorCArmPose(camera,new T.Vector3(pose.x,pose.y,2*sliders.carmZ.value-pose.height),
            CARM_SOURCE_TO_ISOCENTER_MM,...[pose.yaw,pose.pitch,pose.roll].map(T.MathUtils.degToRad));
        applyCArmProjection(camera,CARM_SOURCE_TO_ISOCENTER_MM,pose.detector,pose.zoom,pose.side);
        result.textContent=`Test ${index+1}/${cases.length}: ${JSON.stringify(pose)}\nDotychczasowe ubytki: ${records.reduce((sum,r)=>sum+r.lost,0)}`;
        await frame();
        const reference=oracle(), actual=draw(), metrics=compare(reference.full,actual);
        const before=query.has('compare')?draw(true):null;
        const legacyLost=before?compare(reference.full,before).lost:null;
        const removedByDetector=reference.full.reduce((count,value,i)=>count+(value>3&&reference.expected[i]<.1),0);
        records.push({index:pose.caseIndex,...pose,...metrics,unmatched:reference.unmatched,clipped:reference.clipped,
            removedByDetector,legacyLost,unmatchedExamples:reference.unmatchedExamples,maxIntersections:reference.maxIntersections});
        if(query.has('case') || metrics.lost || [0,27,54,81,108,135,162,189,216,243].includes(index)) {
            if(before)preview(before,`Przed poprawką — ubytki: ${legacyLost} pikseli`);
            preview(actual,`${pose.region}: ${pose.yaw}° / ${pose.pitch}° / ${pose.roll}°, h=${pose.height}; ubytki ${metrics.lost}`);
            if(metrics.lost || query.has('case'))preview(reference.full,'Odniesienie: pełna kość w kierunku detektora');
        }
    }
    const summary={cases:records.length,checkedPixels:records.reduce((sum,r)=>sum+r.checked,0),
        lostPixels:records.reduce((sum,r)=>sum+r.lost,0),negativePixels:records.reduce((sum,r)=>sum+r.negative,0),
        legacyLostPixels:query.has('compare')?records.reduce((sum,r)=>sum+r.legacyLost,0):null,
        clippedIntersections:records.reduce((sum,r)=>sum+r.clipped,0),
        worstP95Mm:Math.max(...records.map(r=>r.p95Mm)),unmatchedRays:records.reduce((sum,r)=>sum+r.unmatched,0),
        seconds:round((performance.now()-started)/1000),size};
    const report={summary,sliders,projection:query.get('projection')||'anatomy',records};
    const reportElement=document.createElement('pre');reportElement.id='report';
    reportElement.textContent=JSON.stringify(report,null,2);
    const details=document.createElement('details');details.append(Object.assign(document.createElement('summary'),{textContent:'Pełne wyniki'}),reportElement);
    document.body.append(details);
    const download=document.querySelector('#download');download.hidden=false;
    download.onclick=()=>{const link=document.createElement('a');link.href=URL.createObjectURL(new Blob([reportElement.textContent],{type:'application/json'}));link.download='bone-carm-limits.json';link.click();setTimeout(()=>URL.revokeObjectURL(link.href),1000);};
    if(query.has('compare') && records.length===1) {
        const button=document.createElement('button');button.textContent='Pobierz porównanie PNG';
        button.onclick=()=>{
            const source=[...document.querySelectorAll('main canvas')].slice(0,2);
            const image=document.createElement('canvas');image.width=size*2+48;image.height=size+80;
            const ctx=image.getContext('2d');ctx.fillStyle='#17242b';ctx.fillRect(0,0,image.width,image.height);
            ctx.fillStyle='#edf3f6';ctx.font='20px sans-serif';
            for(let i=0;i<2;i++){
                ctx.fillText(i?'Po poprawce — pełna kość':'Przed poprawką — odcięta kość',16+i*(size+16),30);
                ctx.drawImage(source[i],16+i*(size+16),48);
            }
            image.toBlob(blob=>{const link=document.createElement('a');link.href=URL.createObjectURL(blob);
                link.download='bone-table-translation-comparison.png';link.click();
                setTimeout(()=>URL.revokeObjectURL(link.href),1000);});
        };
        download.after(button);
    }
    result.textContent=JSON.stringify(summary,null,2);
    assert(summary.checkedPixels>1000,'Za mało sprawdzonych pikseli kości');
    assert(summary.unmatchedRays===0,'Niezamknięta geometria w pomiarze odniesienia');
    assert(summary.lostPixels===0,'Wykryto znikające fragmenty kości');
    assert(summary.worstP95Mm<1.5,'Błąd grubości przekracza rozdzielczość bufora');
    result.textContent+='\nWSZYSTKIE POZYCJE ZALICZONE';document.body.dataset.status='passed';
} catch(error) {
    result.textContent+='\nFAIL: '+error.message;document.body.dataset.status='failed';console.error(error);
} finally {
    for(const mesh of meshes)mesh.geometry.dispose();
    for(const {geometry} of referenceMeshes)geometry.dispose();
    material.dispose();target?.dispose();renderer?.dispose();
}
