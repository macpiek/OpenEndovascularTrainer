import test from 'node:test';
import assert from 'node:assert/strict';
import * as THREE from 'three';
import {readFileSync} from 'node:fs';
import {setupCArmControls} from '../src/carmControls.js';
import {CARM_SOURCE_TO_ISOCENTER_MM} from '../src/imaging/carmProjection.js';
import {createRuntimeLifetime} from '../src/runtimeLifetime.js';
import {setMaxListeners} from 'node:events';

class Element extends EventTarget {
    constructor(value='0',min='-1000',max='1000') {
        super();Object.assign(this,{value,min,max,style:{},dataset:{},offsetWidth:120});
        this.classes=new Set();
        this.classList={add:c=>this.classes.add(c),remove:c=>this.classes.delete(c),toggle:()=>{}};
    }
    setPointerCapture() {} releasePointerCapture() {}
    blur() {}
    querySelector(selector) { return selector==='span' ? this.fieldLabel : null; }
    setAttribute(name,value) { this[name]=value; }
    getBoundingClientRect(){return {left:0,top:0,width:120,height:120};}
}

function fixture(run, {sourceRadius=500, sliders={}, withRuntime=false}={}) {
    const originals={document:globalThis.document,window:globalThis.window,requestAnimationFrame:globalThis.requestAnimationFrame};
    const elements=new Map();
    for(const id of ['carmX','carmY','carmZ','carmDetDist','carmYawReadout','carmPitchReadout','carmRollReadout',
        'carmAngleReset','carmLao30','carmRao30','carmRollLeft','carmRollRight','carmZUp','carmZDown',
        'angleJoystick','angleJoystickHandle','positionJoystick','positionJoystickHandle',
        'detectorZoomStatus','carmGeometryStatus','detectorSize'])elements.set(id,new Element());
    elements.get('carmDetDist').value='350';
    elements.get('detectorSize').value='300';
    for(const [id,attributes] of Object.entries(sliders))Object.assign(elements.get(id),attributes);
    const zoomButtons=['none','medium','maximum'].map(id=>{
        const button=new Element();button.dataset.detectorZoom=id;button.fieldLabel=new Element();return button;
    });
    elements.get('angleJoystickHandle').offsetWidth=20;
    elements.get('positionJoystickHandle').offsetWidth=20;
    const document=Object.assign(new EventTarget(),{visibilityState:'visible',
        getElementById:id=>elements.get(id)||null,querySelectorAll:()=>zoomButtons});
    const window=new EventTarget();let callback,now=performance.now();
    globalThis.document=document;globalThis.window=window;
    globalThis.requestAnimationFrame=fn=>{callback=fn;return 1;};
    let cancelledFrames=0;
    window.requestAnimationFrame=globalThis.requestAnimationFrame;
    window.cancelAnimationFrame=()=>{cancelledFrames++;};
    window.setTimeout=()=>1;window.clearTimeout=()=>{};
    const runtime=withRuntime?createRuntimeLifetime(window):null;
    if(runtime)setMaxListeners(0,runtime.signal);
    const camera=new THREE.PerspectiveCamera(45,1,.1,2000);
    try {
        const controls=setupCArmControls(camera,{branchPoint:{x:0,y:0,z:0}},sourceRadius,
            undefined,undefined,undefined,undefined,undefined,undefined,runtime);
        const frame=(dt=16.6667)=>{now+=dt;callback(now);};
        const event=(target,type,props={})=>{
            const e=new Event(type,{cancelable:true});Object.assign(e,{pointerId:1,...props});
            (typeof target==='string'?elements.get(target):target).dispatchEvent(e);
        };
        const yaw=()=>Math.atan2(camera.position.x,camera.position.z);
        const bounded=()=>{
            assert.ok(camera.position.z>=-1e-8,'source must never move behind the patient');
            assert.ok(Math.abs(yaw())<=Math.PI/2+1e-8);
            assert.ok(Math.abs(Math.asin(camera.position.y/500))<=Math.PI/4+1e-8);
            assert.ok(camera.quaternion.toArray().every(Number.isFinite));
        };
        frame();run({controls,camera,elements,window,document,event,frame,yaw,bounded,zoomButtons,
            runtime,cancelledFrames:()=>cancelledFrames});
    } finally {
        runtime?.dispose();
        for(const [key,value] of Object.entries(originals)) {
            if(value===undefined)delete globalThis[key];else globalThis[key]=value;
        }
    }
}

test('runtime disposal cancels C-arm animation and detaches held-motion controls',()=>fixture(({event,frame,yaw,runtime,cancelledFrames})=>{
    event('carmLao30','pointerdown');frame(100);
    assert.ok(yaw()>0);
    runtime.dispose();
    const before=yaw();
    event('carmRao30','pointerdown');frame(100);
    assert.equal(yaw(),before,'a late RAF or control event must not mutate a disposed camera');
    assert.equal(cancelledFrames(),1);
}, {withRuntime:true}));

test('reset cannot reverse direction or leave the C-arm range on a backward frame timestamp',()=>fixture(({event,frame,yaw,bounded})=>{
    event('carmLao30','pointerdown');for(let i=0;i<100;i++)frame();
    event('carmLao30','pointerup');
    assert.ok(Math.abs(yaw()-Math.PI/6)<1e-6);
    event('carmAngleReset','pointerdown');
    const before=yaw();frame(-5000);
    bounded();assert.ok(yaw()>=0 && yaw()<=before+1e-10,'reset must approach zero without reversing');
}));

test('AP starts and resets on the anterior side without mirroring the patient',()=>fixture(({controls,camera,event,frame})=>{
    const assertAnterior=()=>{
        camera.updateMatrixWorld(true);
        // Atlas frame: +Y cranial, +Z anterior, patient's left is +X.
        const anterior=new THREE.Vector3(0,0,40),posterior=new THREE.Vector3(0,0,-70);
        assert.ok(camera.position.distanceTo(anterior)<camera.position.distanceTo(posterior));
        assert.ok(camera.getWorldDirection(new THREE.Vector3()).z<-.999);
        assert.ok(new THREE.Vector3(20,0,0).project(camera).x>0,'patient left must appear on image right');
        assert.ok(new THREE.Vector3(0,20,0).project(camera).y>0,'cranial direction must remain up');
    };
    assertAnterior();
    event('carmLao30','pointerdown');for(let i=0;i<100;i++)frame();
    controls.reset();assertAnterior();
}));

test('LAO and RAO return monotonically to AP despite irregular render cadence',()=>fixture(({event,frame,yaw,bounded,elements})=>{
    for(const target of ['carmLao30','carmRao30']) {
        event(target,'pointerdown');for(let i=0;i<100;i++)frame();event(target,'pointerup');
        event('carmAngleReset','pointerdown');
        let previous=Math.abs(yaw());
        for(let i=0;i<120;i++) {
            frame(i===10?5000:i%2?33:8);
            bounded();assert.ok(Math.abs(yaw())<=previous+1e-10);
            assert.ok(previous-Math.abs(yaw())<=THREE.MathUtils.degToRad(2.4)+1e-10,'a delayed frame must not teleport the arm');
            previous=Math.abs(yaw());
        }
        event('carmAngleReset','pointerup');
        assert.ok(Math.abs(yaw())<1e-10);assert.equal(elements.get('carmYawReadout').textContent,'AP 0°');
    }
}));

test('reset owns all three rotations and returns from the limits to the original camera pose',()=>fixture(({event,frame,camera,bounded,elements,window})=>{
    const initialPosition=camera.position.clone(),initialRotation=camera.quaternion.clone();
    event('angleJoystick','pointerdown',{clientX:10,clientY:10});
    event('carmRollRight','mousedown');
    for(let i=0;i<400;i++){frame();bounded();}
    event('carmAngleReset','pointerdown');
    // A stale drag event must not compete with the reset command.
    event(window,'mousemove',{clientX:10,clientY:10});
    for(let i=0;i<400;i++){frame();bounded();}
    event('carmAngleReset','pointerup');
    assert.ok(camera.position.distanceTo(initialPosition)<1e-8);
    assert.ok(camera.quaternion.angleTo(initialRotation)<1e-7);
    assert.equal(elements.get('carmPitchReadout').textContent,'CRA 0°');
    assert.equal(elements.get('carmRollReadout').textContent,'Roll 0°');
}));

test('lost capture and hidden tab stop held movements without a jump on return',()=>fixture(({event,frame,yaw,document})=>{
    event('carmLao30','pointerdown');frame(100);
    event('carmLao30','lostpointercapture');
    const held=yaw();for(let i=0;i<20;i++)frame();assert.equal(yaw(),held);
    event('carmRao30','pointerdown');
    document.visibilityState='hidden';event(document,'visibilitychange');
    document.visibilityState='visible';event(document,'visibilitychange');
    frame(60000);assert.equal(yaw(),held);
}));

test('joystick returns from LAO 90 through AP to RAO without pitching or flipping',()=>fixture(({event,frame,yaw,camera,bounded,elements})=>{
    event('angleJoystick','pointerdown',{clientX:60,clientY:10});
    for(let i=0;i<400;i++)frame();
    assert.ok(Math.abs(yaw()-Math.PI/2)<1e-8,'joystick must reach LAO 90');
    assert.equal(elements.get('carmPitchReadout').textContent,'CRA 0°');
    event('angleJoystick','pointermove',{clientX:60,clientY:110});
    let previous=yaw(),rotation=camera.quaternion.clone(),crossedAP=false;
    for(let i=0;i<550;i++) {
        frame();bounded();
        const current=yaw();
        assert.ok(current<=previous+1e-10,'RAO command must decrease LAO continuously');
        if(previous>-Math.PI/2+1e-8)assert.ok(current<previous,'reverse must immediately move away from LAO stop');
        assert.ok(rotation.angleTo(camera.quaternion)<THREE.MathUtils.degToRad(1),'camera must not flip');
        assert.ok(Math.abs(camera.position.y)<1e-8,'LAO/RAO motion must not change pitch');
        crossedAP ||= previous>=0 && current<0;
        previous=current;rotation.copy(camera.quaternion);
    }
    assert.ok(crossedAP);assert.ok(Math.abs(yaw()+Math.PI/2)<1e-8);
    event('angleJoystick','pointermove',{clientX:60,clientY:10});frame();
    assert.ok(yaw()>-Math.PI/2,'reverse must also leave RAO 90 immediately');
    event('angleJoystick','pointerup');
}));

test('independent joystick axis controls only CRA and CAU, independently of LAO/RAO',()=>fixture(({event,frame,yaw,camera})=>{
    event('angleJoystick','pointerdown',{clientX:10,clientY:60});
    for(let i=0;i<220;i++)frame();
    assert.ok(Math.abs(yaw())<1e-8);assert.ok(Math.abs(Math.asin(camera.position.y/500)-Math.PI/4)<1e-8);
    event('angleJoystick','pointermove',{clientX:110,clientY:60});
    for(let i=0;i<400;i++)frame();
    assert.ok(Math.abs(yaw())<1e-8);assert.ok(Math.abs(Math.asin(camera.position.y/500)+Math.PI/4)<1e-8);
    event('angleJoystick','pointerup');
}));

// Exercise the actual application height limits/defaults, rather than a looser
// test-only range which could hide an oversized field at the end stops.
const html=readFileSync(new URL('../index.html',import.meta.url),'utf8');
const appSliders=Object.fromEntries(['carmX','carmY','carmZ','carmDetDist'].map(id=>{
    const input=html.match(new RegExp(`<input[^>]+id="${id}"[^>]*>`))[0];
    return [id,Object.fromEntries(['value','min','max'].map(name=>[name,input.match(new RegExp(`${name}="([^"]+)"`))[1]]))];
}));
const appGeometry={sourceRadius:CARM_SOURCE_TO_ISOCENTER_MM,sliders:appSliders};

function fieldAtPatientPlane(camera) {
    const patientPlane=new THREE.Plane(new THREE.Vector3(0,0,1),0);
    const raycaster=new THREE.Raycaster();
    const ends=[-0.88,0.88].map(x=>{
        raycaster.setFromCamera(new THREE.Vector2(x,0),camera);
        return raycaster.ray.intersectPlane(patientPlane,new THREE.Vector3());
    });
    return ends[0].distanceTo(ends[1]);
}

test('both height end stops keep the patient field inside either detector in all zoom modes',()=>fixture(({camera,event,frame,window,elements,zoomButtons})=>{
    const fields=[];
    for(const [button,expectedHeight] of [['carmZDown',-40],['carmZUp',180]]) {
        event(button,'mousedown');for(let i=0;i<500;i++)frame();event(window,'mouseup');
        assert.equal(Number(elements.get('carmZ').value),expectedHeight);
        const sourceDistance=camera.position.z;
        assert.ok(sourceDistance>0 && sourceDistance<camera.far,'patient stays between source and detector');
        for(const detectorSideMm of [300,210]) {
            elements.get('detectorSize').value=String(detectorSideMm);event('detectorSize','change');
            for(const [index,zoom] of [1,1.4,2.1].entries()) {
                event(zoomButtons[index],'click');
                const field=fieldAtPatientPlane(camera),detectorField=detectorSideMm/zoom;
                assert.ok(Math.abs(field-detectorField*sourceDistance/700)<1e-7,'field follows physical ray divergence');
                assert.ok(field<detectorField,'patient field cannot exceed the active detector');
                if(index===0 && detectorSideMm===300)fields.push(field);
            }
        }
    }
    assert.ok(Math.abs(fields[0]-218.5714285714)<1e-7,'lowest arm: 21.86 cm field at reference patient plane');
    assert.ok(Math.abs(fields[1]-124.2857142857)<1e-7,'highest arm: 12.43 cm field at reference patient plane');
},appGeometry));

test('detector distance changes the cone, zoom remains calibrated, and reset restores the original geometry',()=>fixture(({camera,event,controls,elements,zoomButtons})=>{
    const initialProjection=camera.projectionMatrix.clone(),initialPosition=camera.position.clone();
    elements.get('carmDetDist').value='600';event('carmDetDist','input');
    assert.equal(camera.far,950);
    assert.ok(camera.position.equals(initialPosition),'moving detector must not move the source');
    event(zoomButtons[1],'click');
    assert.ok(Math.abs(fieldAtPatientPlane(camera)-(300/1.4)*410/950)<1e-7);
    assert.match(elements.get('detectorZoomStatus').textContent,/21,4 × 21,4 cm detektora/);
    assert.match(elements.get('carmGeometryStatus').textContent,/Lampa–detektor: 95 cm/);
    controls.reset();
    assert.ok(camera.projectionMatrix.equals(initialProjection));
    assert.ok(camera.position.equals(initialPosition));
    assert.equal(camera.far,700);
    assert.match(elements.get('carmGeometryStatus').textContent,/15 × 15 cm/);
},appGeometry));

test('detector selection changes the physical field without changing pose or zoom and respects the camera lock',()=>fixture(({camera,event,controls,elements,zoomButtons})=>{
    assert.match(html,/<option value="300" selected>30 × 30 cm<\/option>/);
    assert.equal(elements.get('detectorSize').value,'300');
    assert.equal(controls.getDetectorZoomMode().detectorSideMm,300);
    event(zoomButtons[1],'click');
    const position=camera.position.clone(),rotation=camera.quaternion.clone();
    const field30=fieldAtPatientPlane(camera),revision=controls.getRevision();
    elements.get('detectorSize').value='210';event('detectorSize','change');
    assert.ok(Math.abs(fieldAtPatientPlane(camera)/field30-0.7)<1e-8);
    assert.ok(camera.position.equals(position));assert.ok(camera.quaternion.equals(rotation));
    assert.equal(camera.zoom,1.4);assert.equal(camera.far,700);
    assert.ok(controls.getRevision()>revision,'changing detector must invalidate the prior projection/capture');
    assert.match(elements.get('detectorZoomStatus').textContent,/15 × 15 cm detektora/);
    assert.deepEqual(zoomButtons.map(button=>button.fieldLabel.textContent),[
        '21 × 21 cm detektora','15 × 15 cm detektora','10 × 10 cm detektora'
    ]);
    controls.setLocked(true);
    assert.equal(elements.get('detectorSize').disabled,true);
    const projection=camera.projectionMatrix.clone();
    elements.get('detectorSize').value='300';event('detectorSize','change');
    assert.ok(camera.projectionMatrix.equals(projection),'locked geometry must not change');
    controls.setLocked(false);assert.equal(elements.get('detectorSize').disabled,false);
    controls.reset();
    assert.equal(elements.get('detectorSize').value,'300');
    assert.equal(controls.getDetectorZoomMode().id,'none');
    assert.equal(zoomButtons[0].fieldLabel.textContent,'30 × 30 cm detektora');
},appGeometry));
