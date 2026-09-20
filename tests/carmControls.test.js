import test from 'node:test';
import assert from 'node:assert/strict';
import * as THREE from 'three';
import {setupCArmControls} from '../src/carmControls.js';

class Element extends EventTarget {
    constructor(value='0',min='-1000',max='1000') {
        super();Object.assign(this,{value,min,max,style:{},dataset:{},offsetWidth:120});
        this.classes=new Set();
        this.classList={add:c=>this.classes.add(c),remove:c=>this.classes.delete(c),toggle:()=>{}};
    }
    setPointerCapture() {} releasePointerCapture() {}
    getBoundingClientRect(){return {left:0,top:0,width:120,height:120};}
}

function fixture(run) {
    const originals={document:globalThis.document,window:globalThis.window,requestAnimationFrame:globalThis.requestAnimationFrame};
    const elements=new Map();
    for(const id of ['carmX','carmY','carmZ','carmDetDist','carmYawReadout','carmPitchReadout','carmRollReadout',
        'carmAngleReset','carmLao30','carmRao30','carmRollLeft','carmRollRight','carmZUp','carmZDown',
        'angleJoystick','angleJoystickHandle','positionJoystick','positionJoystickHandle'])elements.set(id,new Element());
    elements.get('carmDetDist').value='350';
    elements.get('angleJoystickHandle').offsetWidth=20;
    elements.get('positionJoystickHandle').offsetWidth=20;
    const document=Object.assign(new EventTarget(),{visibilityState:'visible',
        getElementById:id=>elements.get(id)||null,querySelectorAll:()=>[]});
    const window=new EventTarget();let callback,now=performance.now();
    globalThis.document=document;globalThis.window=window;
    globalThis.requestAnimationFrame=fn=>{callback=fn;return 1;};
    const camera=new THREE.PerspectiveCamera(45,1,.1,2000);
    try {
        const controls=setupCArmControls(camera,{branchPoint:{x:0,y:0,z:0}},500);
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
        frame();run({controls,camera,elements,window,document,event,frame,yaw,bounded});
    } finally {
        for(const [key,value] of Object.entries(originals)) {
            if(value===undefined)delete globalThis[key];else globalThis[key]=value;
        }
    }
}

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
