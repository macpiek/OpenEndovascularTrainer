import test from 'node:test';
import assert from 'node:assert/strict';
import {StentGraftSystem} from '../src/devices/stentGraftSystem.js';
import {initStentGraftControls} from '../src/ui/stentGraftControls.js';

function systemFixture() {
    let length=100;
    const source=()=>({nodes:[{x:0,y:0,z:0},{x:0,y:length,z:0}],coordinate:i=>i*length,catheterMm:0});
    const system=new StentGraftSystem({readAccess:source,readAnatomy:()=>null,sheaths:{}});
    system.ensureRoutes=()=>true;
    return {system,setLength:value=>{length=value;}};
}

test('catheter commands move delivery only on committed time, stop on release and respect wire support',()=>{
    const {system,setLength}=systemFixture();
    try {
        setLength(0);assert.equal(system.load('right','body').ok,true);
        const d=system.accesses.right.device,command=advance=>({deviceId:d.id,advance});
        system.updateAccess('right',1,null,command(1));assert.equal(d.position,0);
        setLength(100);system.updateAccess('right',0,null,command(1));assert.equal(d.position,0);
        system.updateAccess('right',.1,null,command(1));assert.equal(d.position,2.5);
        system.updateAccess('right',1,null,command(0));assert.equal(d.position,2.5);
        system.updateAccess('right',.1,null,command(-1));assert.equal(d.position,0);
        for(let i=0;i<10;i++)system.updateAccess('right',1,null,command(1));
        assert.equal(d.position,88);
        setLength(20);system.updateAccess('right',1,null,command(1));assert.equal(d.position,88,'advance cannot retract an unsupported system');
        system.updateAccess('right',1,null,command(-1));assert.equal(d.position,63);
        const before=d.position;
        system.updateAccess('right',1,null,{deviceId:d.id+1,advance:1});assert.equal(d.position,before,'old commands cannot move a replacement device');
    } finally {system.dispose();}
});

class Element extends EventTarget {
    constructor(){super();this.value='';this.open=false;this.disabled=false;this.hidden=false;this.textContent='';}
    setAttribute(name,value){this[name]=value;}
    showModal(){this.open=true;} close(){this.open=false;}
    click(){this.dispatchEvent(new Event('click'));}
}
function controlsFixture() {
    const f=systemFixture(),elements=new Map();let side='right',tool='berenstein',released=0;
    const root=Object.assign(new EventTarget(),{defaultView:new EventTarget(),getElementById:id=>{if(!elements.has(id))elements.set(id,new Element());return elements.get(id);}});
    const ui={releaseToolInputs:()=>released++,getSelectedCatheterType:()=> 'berenstein',
        getSelectedCatheterTool:()=>tool,setSelectedCatheterTool:value=>{tool=value;},updateCatheterLength:()=>{}};
    const controls=initStentGraftControls({system:f.system,activeSide:()=>side,ui,root});
    return {...f,root,e:id=>root.getElementById(id),controls,ui,setSide:value=>{side=value;},get released(){return released;}};
}

test('selection opens a modal, cancel restores catheter, choose loads diameter and uses a separate deploy button',()=>{
    const f=controlsFixture();
    try {
        f.controls.selectTool('stentgraft');f.ui.setSelectedCatheterTool('stentgraft');
        assert.equal(f.e('stentGraftDialog').open,true);
        f.e('stentGraftCancel').click();assert.equal(f.ui.getSelectedCatheterTool(),'berenstein');
        assert.equal(f.system.accesses.right.device,null);
        f.controls.selectTool('stentgraft');f.ui.setSelectedCatheterTool('stentgraft');
        f.e('stentGraftDiameter').value='32';f.e('stentGraftLoad').click();
        assert.equal(f.e('stentGraftDialog').open,false);
        assert.equal(f.system.accesses.right.device.diameter,32);
        assert.equal(f.e('stentGraftDeploy').disabled,true,'selection must not deploy');
        assert.equal(f.system.accesses.right.device.phase,'loaded');
        assert.equal(f.e('catheterRotateLeft').disabled,false);
        f.setSide('left');f.ui.setSelectedCatheterTool('berenstein');f.controls.refresh();
        assert.equal(f.e('stentGraftToolControls').hidden,true);
        assert.equal(f.e('catheterRotateLeft').disabled,false);
        assert.equal(f.system.accesses.right.device.diameter,32,'switching access preserves the first device');
        assert.equal(f.system.accesses.left.device,null);
    } finally {f.system.dispose();}
});

test('failed selection stays in the modal, and changing catheter cannot remove an inserted device',()=>{
    const f=controlsFixture();
    try {
        f.controls.selectTool('stentgraft');f.ui.setSelectedCatheterTool('stentgraft');
        f.system.ensureRoutes=()=>false;f.e('stentGraftLoad').click();
        assert.equal(f.e('stentGraftDialog').open,true);assert.match(f.e('stentGraftDialogStatus').textContent,/Anatomia/);
        f.system.ensureRoutes=()=>true;f.e('stentGraftLoad').click();
        const d=f.system.accesses.right.device;d.position=20;d.target=20;
        assert.equal(f.controls.selectTool('pigtail'),false);assert.equal(f.system.accesses.right.device,d);
        d.position=0;assert.equal(f.controls.selectTool('pigtail'),true);assert.equal(f.system.accesses.right.device,null);
    } finally {f.system.dispose();}
});

test('hold controls stop on release, blur, access change and deployment stage changes',()=>{
    const f=controlsFixture();
    const event=(type,values={})=>Object.assign(new Event(type,{cancelable:true}),values);
    try {
        f.system.load('right','body');f.ui.setSelectedCatheterTool('stentgraft');
        const d=f.system.accesses.right.device;Object.assign(d,{position:30,target:30});
        f.system.deploy=()=>{Object.assign(d,{phase:'deploying',releaseStage:'sheath',sheathWithdrawal:0,sheathTravel:100,tipRelease:0});return {ok:true};};
        f.controls.refresh();const deploy=f.e('stentGraftDeploy'),tip=f.e('stentGraftReleaseTip');
        deploy.dispatchEvent(event('pointerdown',{button:0,pointerId:1}));
        assert.equal(f.controls.readRelease('right'),'sheath');assert.equal(f.controls.readRelease('left'),null);
        deploy.dispatchEvent(event('pointerup'));assert.equal(f.controls.readRelease('right'),null);
        deploy.dispatchEvent(event('keydown',{key:' '}));assert.equal(f.controls.readRelease('right'),'sheath');
        deploy.dispatchEvent(event('keyup',{key:' '}));assert.equal(f.controls.readRelease('right'),null);
        deploy.dispatchEvent(event('pointerdown',{button:0}));f.setSide('left');
        assert.equal(f.controls.readRelease('right'),null,'switching away cancels even before UI refresh');
        f.setSide('right');assert.equal(f.controls.readRelease('right'),null);
        deploy.dispatchEvent(event('pointerdown',{button:0}));d.releaseStage='tip';f.controls.refresh();
        assert.equal(f.controls.readRelease('right'),'sheath');assert.equal(deploy.disabled,false);assert.equal(tip.disabled,false);
        deploy.dispatchEvent(event('pointerup'));
        tip.dispatchEvent(event('pointerdown',{button:0}));assert.equal(f.controls.readRelease('right'),'tip');
        tip.dispatchEvent(event('blur'));assert.equal(f.controls.readRelease('right'),null);
        tip.click();assert.equal(f.controls.readRelease('right'),null,'a click does not start an autonomous animation');
    } finally {f.system.dispose();}
});

test('J/K cover direction and L capture are independent held commands, scoped to the selected access and tool',()=>{
    const f=controlsFixture(),event=(type,code,extra={})=>Object.assign(new Event(type,{cancelable:true}),{code,...extra});
    try {
        f.system.load('right','body');f.ui.setSelectedCatheterTool('stentgraft');
        const d=f.system.accesses.right.device;
        Object.assign(d,{position:30,target:30,phase:'deploying',releaseStage:'sheath',sheathWithdrawal:30,sheathTravel:100,tipRelease:0});f.controls.refresh();
        f.root.dispatchEvent(event('keydown','KeyJ'));assert.equal(f.controls.readRelease('right'),'sheath');
        f.root.dispatchEvent(event('keydown','KeyL'));assert.deepEqual(f.controls.readRelease('right'),{sheath:1,tip:true});
        f.root.dispatchEvent(event('keyup','KeyJ'));assert.equal(f.controls.readRelease('right'),'tip');
        f.root.dispatchEvent(event('keydown','KeyK'));assert.deepEqual(f.controls.readRelease('right'),{sheath:-1,tip:true});
        f.root.defaultView.dispatchEvent(new Event('blur'));assert.equal(f.controls.readRelease('right'),null);
        f.root.dispatchEvent(event('keydown','KeyJ',{ctrlKey:true}));assert.equal(f.controls.readRelease('right'),null);
        f.e('stentGraftDialog').open=true;f.root.dispatchEvent(event('keydown','KeyJ'));assert.equal(f.controls.readRelease('right'),null);f.e('stentGraftDialog').open=false;
        f.root.closest=()=>({});f.root.dispatchEvent(event('keydown','KeyJ'));assert.equal(f.controls.readRelease('right'),null,'typing into a field does not control the graft');delete f.root.closest;
        f.root.dispatchEvent(event('keydown','KeyK'));f.setSide('left');assert.equal(f.controls.readRelease('right'),null);
        f.setSide('right');f.ui.setSelectedCatheterTool('berenstein');f.root.dispatchEvent(event('keydown','KeyJ'));assert.equal(f.controls.readRelease('right'),null);
        f.ui.setSelectedCatheterTool('stentgraft');f.controls.dispose();f.root.dispatchEvent(event('keydown','KeyJ'));assert.equal(f.controls.readRelease('right'),null,'dispose removes keyboard listeners');
    } finally {f.system.dispose();}
});

test('nose button waits for fixation release, stops on pointer release and belongs to the active sheath',()=>{
    const f=controlsFixture(),event=type=>Object.assign(new Event(type,{cancelable:true}),{button:0});
    try {
        f.system.load('right','body');f.ui.setSelectedCatheterTool('stentgraft');
        const d=f.system.accesses.right.device;
        Object.assign(d,{position:80,target:80,phase:'deploying',releaseStage:'sheath',sheathWithdrawal:30,sheathTravel:100,tipRelease:0});
        const button=f.e('stentGraftRetractNose');f.controls.refresh();assert.equal(button.disabled,true);
        d.tipRelease=1;f.controls.refresh();assert.equal(button.disabled,false);
        button.dispatchEvent(event('pointerdown'));assert.equal(f.controls.readRelease('right'),'nose');
        button.dispatchEvent(event('pointerup'));assert.equal(f.controls.readRelease('right'),null);
        button.dispatchEvent(event('pointerdown'));f.setSide('left');assert.equal(f.controls.readRelease('right'),null);
        f.setSide('right');d.noseRetraction=33;f.controls.refresh();assert.equal(button.disabled,true,'stops when the cone reaches the sheath');
        assert.equal(d.sheathWithdrawal,30);
    } finally {f.controls.dispose();f.system.dispose();}
});

test('image cards select nominal main body length and discrete proximal/distal sizes',()=>{
    for(const [id,length] of [['iis-103',103],['ii-124',124],['ii-145',145],['ii-166',166]]) {
        const f=controlsFixture();
        try {
            f.controls.selectTool('stentgraft');f.ui.setSelectedCatheterTool('stentgraft');
            f.e(`graftModel-${id}`).click();assert.equal(f.e(`graftModel-${id}`)['aria-pressed'],'true');
            f.e('stentGraftDiameter').value='32';f.e('stentGraftDiameter').dispatchEvent(new Event('change'));
            f.e('stentGraftLoad').click();
            const d=f.system.accesses.right.device;
            assert.equal(d.modelId,id);assert.equal(d.length,length);assert.equal(d.diameter,32);
            assert.equal(d.distalDiameter,id==='iis-103'?14:16);
        } finally {f.controls.dispose();f.system.dispose();}
    }
});

test('deployment buttons preserve held movement and rotation; advance and withdraw remain enabled',()=>{
    const f=controlsFixture(),event=type=>Object.assign(new Event(type,{cancelable:true}),{button:0});
    try {
        f.system.load('right','body');f.ui.setSelectedCatheterTool('stentgraft');
        const d=f.system.accesses.right.device;
        Object.assign(d,{phase:'deploying',position:30,target:30,sheathWithdrawal:20,sheathTravel:100,tipRelease:0});
        f.controls.refresh();const released=f.released;
        assert.equal(f.e('catheterAdvance').disabled,false);assert.equal(f.e('catheterWithdraw').disabled,false);
        assert.equal(f.e('catheterRotateLeft').disabled,false);
        f.e('stentGraftDeploy').dispatchEvent(event('pointerdown'));
        assert.equal(f.released,released,'starting cover motion must not cancel A/D or Q/E');
        assert.equal(f.controls.readRelease('right'),'sheath');
        f.e('stentGraftDeploy').dispatchEvent(event('pointerup'));
        d.tipRelease=1;f.controls.refresh();assert.equal(f.e('catheterAdvance').disabled,false);
        assert.match(f.e('catheterRotateLeft').title,/implant pozostaje nieruchomy/);
    } finally {f.controls.dispose();f.system.dispose();}
});

test('N retracts the nose only after capture release and stops on keyup, blur or access change',()=>{
    const f=controlsFixture(),key=type=>Object.assign(new Event(type,{cancelable:true}),{code:'KeyN'});
    try {
        f.system.load('right','body');f.ui.setSelectedCatheterTool('stentgraft');
        const d=f.system.accesses.right.device;
        Object.assign(d,{position:80,target:80,phase:'deploying',releaseStage:'sheath',sheathWithdrawal:30,sheathTravel:100,tipRelease:0});
        f.controls.refresh();f.root.dispatchEvent(key('keydown'));assert.equal(f.controls.readRelease('right'),null);
        d.tipRelease=1;f.controls.refresh();
        f.root.dispatchEvent(key('keydown'));assert.equal(f.controls.readRelease('right'),'nose');
        f.root.dispatchEvent(key('keyup'));assert.equal(f.controls.readRelease('right'),null);
        f.root.dispatchEvent(key('keydown'));f.root.defaultView.dispatchEvent(new Event('blur'));assert.equal(f.controls.readRelease('right'),null);
        f.root.dispatchEvent(key('keydown'));f.setSide('left');assert.equal(f.controls.readRelease('right'),null);
        f.setSide('right');d.noseRetraction=33;f.controls.refresh();
        f.root.dispatchEvent(key('keydown'));assert.equal(f.controls.readRelease('right'),null,'docked nose cannot retract further');
    } finally {f.controls.dispose();f.system.dispose();}
});
