import {MAIN_BODY_MODELS,DEFAULT_MAIN_BODY,mainBodyModel,proximalDiameters,distalDiameters} from '../devices/stentGraftModels.js';
import {mainBodyCards} from './stentGraftModelCards.js';
import {deliveryNoseState,graftAttached} from '../devices/stentGraftDeployment.js';
export function initStentGraftControls({system,activeSide,ui,root=document}) {
    const el=id=>root.getElementById(id);
    const dialog=el('stentGraftDialog'),type=el('stentGraftType'),diameter=el('stentGraftDiameter');
    const deploy=el('stentGraftDeploy'),remove=el('stentGraftRemove'),configure=el('stentGraftConfigure');
    const tip=el('stentGraftReleaseTip'),resheath=el('stentGraftResheath');
    const nose=el('stentGraftRetractNose');
    const buttons={sheath:deploy,resheath,tip,nose},keys={KeyJ:'sheath',KeyK:'resheath',KeyL:'tip',KeyN:'nose'};
    const held=new Map(),listeners=[];
    let dialogSide=null,modelId=DEFAULT_MAIN_BODY;
    el('stentGraftModelCards').innerHTML=mainBodyCards();
    function listen(target,type,fn){target?.addEventListener?.(type,fn);listeners.push(()=>target?.removeEventListener?.(type,fn));}
    function paintHeld(){for(const [control,button] of Object.entries(buttons))button.setAttribute('aria-pressed',String([...held.values()].some(h=>h.control===control)));}
    function stopRelease(token){if(typeof token==='string')held.delete(token);else held.clear();paintHeld();}
    function readRelease(side) {
        if([...held.values()].some(h=>h.side!==activeSide()||system.accesses[h.side].device?.id!==h.id))stopRelease();
        const controls=new Set([...held.values()].filter(h=>h.side===side).map(h=>h.control));
        if(!controls.size)return null;
        if(controls.size===1)return [...controls][0];
        return {sheath:Number(controls.has('sheath'))-Number(controls.has('resheath')),tip:controls.has('tip'),...(controls.has('nose')?{nose:true}:{})};
    }
    const error=reason=>{el('stentGraftDialogStatus').textContent=reason;};
    const options=values=>values.map(value=>`<option value="${value}">${value} mm</option>`).join('');
    function updateDistal() {
        const distal=el('stentGraftDistalDiameter'),values=distalDiameters(modelId,Number(diameter.value)),previous=Number(distal.value);
        distal.innerHTML=options(values);distal.value=String(values.includes(previous)?previous:values.includes(16)?16:values[0]);
        el('stentGraftDiameterValue').textContent=`${diameter.value} mm`;
    }
    function diameterRange() {
        const body=type.value==='body',values=body?proximalDiameters(modelId):[10,13,14,16,20,24,28],previous=Number(diameter.value);
        diameter.innerHTML=options(values);diameter.value=String(values.includes(previous)?previous:body?28:14);
        el('stentGraftMainBodies').hidden=!body;el('stentGraftDistalField').hidden=!body;
        updateDistal();
        for(const model of MAIN_BODY_MODELS)el(`graftModel-${model.id}`).setAttribute('aria-pressed',String(model.id===modelId));
        el('stentGraftModelDescription').textContent=modelId==='iis-103'
            ?'Krótki korpus IIs, jak na schemacie: 103 mm, oba ujścia 14 mm. Ilustracja schematyczna.'
            :`Korpus II: ${mainBodyModel(modelId).length} mm, długa nóżka ipsilateralna i krótka bramka kontralateralna.`;
    }
    for(const model of MAIN_BODY_MODELS)listen(el(`graftModel-${model.id}`),'click',()=>{modelId=model.id;diameterRange();});
    function open() {
        if(system.accesses[activeSide()].device)return;
        ui.releaseToolInputs();dialogSide=activeSide();
        type.value=system.availableGate(dialogSide)?'limb':'body';diameterRange();error('');
        el('stentGraftAccess').textContent=`Koszulka: ${dialogSide==='right'?'prawa':'lewa'}`;
        dialog.showModal();
    }
    function close() {
        dialog.close();dialogSide=null;ui.releaseToolInputs();
        if(!system.accesses[activeSide()].device)ui.setSelectedCatheterTool(ui.getSelectedCatheterType());
        root.activeElement?.blur?.();refresh();
    }
    function selectTool(value) {
        if(value==='stentgraft'){open();return true;}
        const device=system.accesses[activeSide()].device;
        if(device&&(device.position>.5||device.phase==='deploying'))return false;
        const result=system.removeDelivery(activeSide());
        if(!result.ok){system.accesses[activeSide()].message=result.reason;return false;}
        return true;
    }
    configure.addEventListener('click',open);
    el('stentGraftCancel').addEventListener('click',close);
    dialog.addEventListener('cancel',event=>{event.preventDefault();close();});
    type.addEventListener('change',diameterRange);
    diameter.addEventListener('change',updateDistal);
    diameter.addEventListener('input',updateDistal);
    el('stentGraftLoad').addEventListener('click',()=>{
        if(dialogSide!==activeSide()){error('Koszulka została zmieniona. Otwórz wybór ponownie.');return;}
        const result=system.load(dialogSide,type.value,modelId);
        if(!result.ok){error(result.reason);return;}
        system.setDiameter(dialogSide,Number(diameter.value));
        system.setDistalDiameter(dialogSide,Number(el('stentGraftDistalDiameter').value));
        ui.setSelectedCatheterTool('stentgraft');close();
    });
    function start(control,token) {
        if(buttons[control].disabled||ui.getSelectedCatheterTool()!=='stentgraft')return false;
        const side=activeSide();
        if(system.accesses[side].device?.phase==='loaded') {
            const result=system.deploy(side);
            if(!result.ok){system.accesses[side].message=result.reason;refresh();return false;}
        }
        const d=system.accesses[side].device;
        if(!d||!['deploying','deployed'].includes(d.phase))return false;
        held.set(token,{side,id:d.id,control});paintHeld();refresh();return true;
    }
    for(const [control,button] of Object.entries(buttons)) {
        const token=`button-${control}`;
        const begin=event=>{
            if(event.repeat||event.type==='pointerdown'&&event.button!==0)return;
            if(event.type==='keydown'&&![' ','Enter'].includes(event.key))return;
            if(!start(control,token))return;
            event.preventDefault();
            if(event.type==='pointerdown')button.setPointerCapture?.(event.pointerId);
        };
        listen(button,'pointerdown',begin);listen(button,'keydown',begin);
        for(const type of ['pointerup','pointercancel','lostpointercapture','blur'])listen(button,type,()=>stopRelease(token));
        listen(button,'keyup',event=>{if([' ','Enter'].includes(event.key)){event.preventDefault();stopRelease(token);}});
        listen(button,'click',event=>event.preventDefault());
    }
    listen(root,'keydown',event=>{
        const control=keys[event.code];
        if(!control||event.repeat||event.ctrlKey||event.metaKey||event.altKey||dialog.open||ui.getSelectedCatheterTool()!=='stentgraft')return;
        if(event.target?.closest?.('input,select,textarea,[contenteditable]:not([contenteditable="false"])')||root.querySelector?.('dialog[open]'))return;
        if(start(control,`key-${event.code}`))event.preventDefault();
    });
    listen(root,'keyup',event=>{if(keys[event.code])stopRelease(`key-${event.code}`);});
    listen(root.defaultView,'blur',()=>stopRelease());
    listen(root,'visibilitychange',()=>{if(root.hidden)stopRelease();});
    remove.addEventListener('click',()=>{
        const side=activeSide(),result=system.removeDelivery(side);
        if(!result.ok)system.accesses[side].message=result.reason;
        else {ui.releaseToolInputs();ui.setSelectedCatheterTool(ui.getSelectedCatheterType());}
        refresh();
    });
    function refresh() {
        const side=activeSide(),state=system.snapshot(side),d=state.device;
        if(dialog.open&&dialogSide!==side)close();
        const selected=ui.getSelectedCatheterTool()==='stentgraft';
        el('stentGraftToolControls').hidden=!selected;
        readRelease(side);
        configure.disabled=!!d;
        deploy.disabled=d?.phase==='deploying'||d?.phase==='deployed'
            ?d.sheathWithdrawal>=d.sheathTravel:!state.validation?.ok;
        resheath.disabled=!d||!(d.sheathWithdrawal>0);
        tip.hidden=d?.type!=='body';
        tip.disabled=!d||d.phase==='deployed'||d.tipRelease>=1||(d.phase==='loaded'&&!state.validation?.ok);
        nose.disabled=!d||d.phase==='loaded'||!(d.tipRelease>=1)||deliveryNoseState(d).remaining<.001;
        nose.title=d?.type==='body'&&!(d.tipRelease>=1)?'Najpierw uwolnij mocowanie stentu nadnerkowego.':'Przytrzymaj N, aby ściągać nosecone do końca koszulki; puść, aby zatrzymać.';
        for(const [token,h] of held)if(buttons[h.control].disabled)stopRelease(token);
        remove.disabled=!d||d.phase==='deploying'||d.position>.5;
        el('stentGraftSelection').textContent=d?`${d.type==='body'?mainBodyModel(d.modelId).name:'Endurant II'} · ${d.type==='body'?'korpus rozwidlony':'nóżka kontralateralna'} · ${d.diameter}${d.type==='body'?' / '+d.distalDiameter:''} × ${d.length} mm`:'Wybierz implant';
        el('stentGraftDeployment').value=d?.deployment??0;
        el('stentGraftStatus').textContent=d&&d.phase!=='loaded'
            ?`${Math.round(d.deployment*100)}% · Koszulka: ${d.sheathWithdrawal.toFixed(1)} / ${d.sheathTravel.toFixed(1)} mm. ${d.phase==='deployed'?'Implant uwolniony; ruch koszulki nie składa go ponownie.':d.type==='body'?`Mocowanie: ${Math.round(d.tipRelease*100)}% uwolnione. Koszulka i mocowanie sterowane niezależnie.`:'Zsuwaj lub nasuwaj koszulkę; puszczenie zatrzymuje ruch.'}`
            :state.message||'Wybierz implant i wprowadź prowadnik.';
        if(d&&d.phase!=='loaded')el('stentGraftStatus').textContent+=` Nosecone do koszulki: ${deliveryNoseState(d).remaining.toFixed(1)} mm.`;
        el('stentGraftPlacement').textContent=state.validation?.reason??'';
        el('stentGraftImplants').textContent=state.implants.map(i=>
            `${i.type==='body'?'Korpus':'Nóżka'} · ${i.side==='right'?'prawa':'lewa'} · ${Math.round(i.deployment*100)}%`).join(' | ');
        for(const id of ['catheterRotateLeft','catheterRotateRight']) {
            el(id).disabled=selected&&!d;
            el(id).title=selected?(d&&!graftAttached(d)?'Obrót systemu wprowadzającego — implant pozostaje nieruchomy (Q / E)':'Obrót zespołu stentgraftu (Q / E)'):'Obrót cewnika (Q / E)';
        }
        if(selected&&d)el('stentGraftStatus').textContent+=!graftAttached(d)?' Q / E: obrót urządzenia, implant odłączony.':' Q / E: obrót zespołu z implantem.';
        for(const [id,verb] of [['catheterAdvance','Wsuń'],['catheterWithdraw','Wycofaj']]) {
            el(id).disabled=selected&&!d;
            el(id).setAttribute('aria-label',`${verb} ${selected?'stentgraft':'cewnik'} — przytrzymaj`);
        }
        if(selected)ui.updateCatheterLength(0);
    }
    refresh();return {refresh,selectTool,readRelease,dispose(){stopRelease();for(const remove of listeners)remove();}};
}
