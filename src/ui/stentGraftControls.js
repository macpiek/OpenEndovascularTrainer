export function initStentGraftControls({system,activeSide,ui,root=document}) {
    const el=id=>root.getElementById(id);
    const dialog=el('stentGraftDialog'),type=el('stentGraftType'),diameter=el('stentGraftDiameter');
    const deploy=el('stentGraftDeploy'),remove=el('stentGraftRemove'),configure=el('stentGraftConfigure');
    let dialogSide=null;
    const error=reason=>{el('stentGraftDialogStatus').textContent=reason;};
    function diameterRange() {
        diameter.min=type.value==='body'?'20':'10';diameter.max=type.value==='body'?'36':'20';
        diameter.value=type.value==='body'?'28':'14';
        el('stentGraftDiameterValue').textContent=`${diameter.value} mm`;
    }
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
    diameter.addEventListener('input',()=>{el('stentGraftDiameterValue').textContent=`${diameter.value} mm`;});
    el('stentGraftLoad').addEventListener('click',()=>{
        if(dialogSide!==activeSide()){error('Koszulka została zmieniona. Otwórz wybór ponownie.');return;}
        const result=system.load(dialogSide,type.value);
        if(!result.ok){error(result.reason);return;}
        system.setDiameter(dialogSide,Number(diameter.value));
        ui.setSelectedCatheterTool('stentgraft');close();
    });
    deploy.addEventListener('click',()=>{
        ui.releaseToolInputs();
        const side=activeSide(),result=system.deploy(side);
        if(!result.ok)system.accesses[side].message=result.reason;
        refresh();
    });
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
        configure.disabled=!!d;deploy.disabled=!state.validation?.ok;
        remove.disabled=!d||d.phase==='deploying'||d.position>.5;
        el('stentGraftSelection').textContent=d?`${d.type==='body'?'Korpus rozwidlony':'Nóżka kontralateralna'} · ${d.diameter} mm`:'Wybierz implant';
        el('stentGraftDeployment').value=d?.deployment??0;
        el('stentGraftStatus').textContent=d?.phase==='deploying'?`Rozkładanie: ${Math.round(d.deployment*100)}%`:
            state.message||'Wybierz implant i wprowadź prowadnik.';
        el('stentGraftPlacement').textContent=state.validation?.reason??'';
        el('stentGraftImplants').textContent=state.implants.map(i=>
            `${i.type==='body'?'Korpus':'Nóżka'} · ${i.side==='right'?'prawa':'lewa'} · ${Math.round(i.deployment*100)}%`).join(' | ');
        for(const id of ['catheterRotateLeft','catheterRotateRight'])el(id).disabled=selected;
        for(const [id,verb] of [['catheterAdvance','Wsuń'],['catheterWithdraw','Wycofaj']]) {
            el(id).disabled=selected&&(!d||d.phase==='deploying');
            el(id).setAttribute('aria-label',`${verb} ${selected?'stentgraft':'cewnik'} — przytrzymaj`);
        }
        if(selected)ui.updateCatheterLength(0);
    }
    refresh();return {refresh,selectTool};
}
