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
    const root={getElementById:id=>{if(!elements.has(id))elements.set(id,new Element());return elements.get(id);}};
    const ui={releaseToolInputs:()=>released++,getSelectedCatheterType:()=> 'berenstein',
        getSelectedCatheterTool:()=>tool,setSelectedCatheterTool:value=>{tool=value;},updateCatheterLength:()=>{}};
    const controls=initStentGraftControls({system:f.system,activeSide:()=>side,ui,root});
    return {...f,e:id=>root.getElementById(id),controls,ui,setSide:value=>{side=value;},get released(){return released;}};
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
        assert.equal(f.e('catheterRotateLeft').disabled,true);
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
