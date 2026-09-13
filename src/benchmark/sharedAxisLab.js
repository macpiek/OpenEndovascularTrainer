import { createSharedAxisContacts } from '../physics/kirchhoffSharedAxisContacts.js';
import { createSharedAxisNative, feedSharedAxisNative, rotateSharedAxisNative, relaxSharedAxisNative } from '../physics/kirchhoffSharedAxisNative.js';

const $ = id => document.getElementById(id);
const ids = ['wire', 'catheter'];
let state, spins = {wire:0, catheter:0};
const sheath={start:[0,0,0],end:[70,0,0],innerRadius:1.05,proximalExtension:40};
function labels() {
    for (const id of ids) {
        $(`${id}-feed-value`).textContent = `${(Number($(`${id}-feed`).value)/10).toFixed(1)} cm`;
        $(`${id}-spin-value`).textContent = `${$(`${id}-spin`).value}°`;
    }
}
function draw() {
    const points = state.positions;
    const xmin = Math.min(...points.map(p=>p[0])), xmax = Math.max($('sheath-mode').checked?90:0,...points.map(p=>p[0]));
    const extent = Math.max(20,...points.flatMap(p=>[Math.abs(p[1]),Math.abs(p[2])]))*1.15;
    const scale = Math.min(440/Math.max(1,xmax-xmin),210/(2*extent));
    for(const [plot,axis] of [['xy',1],['xz',2]]){
        $(plot).replaceChildren();
        if($('sheath-mode').checked){
            const sleeve=document.createElementNS('http://www.w3.org/2000/svg','rect');
            sleeve.setAttribute('x',30-xmin*scale);sleeve.setAttribute('y',122);sleeve.setAttribute('width',70*scale);sleeve.setAttribute('height',16);
            sleeve.setAttribute('fill','#6c819a');sleeve.setAttribute('opacity','.45');$(plot).append(sleeve);
        }
        for(const id of ['catheter','wire']){
            const material=state.materials.find(t=>t.spec.id===id),line=document.createElementNS('http://www.w3.org/2000/svg','polyline');
            line.setAttribute('points',points.slice(0,material.last+1).map(p=>`${30+(p[0]-xmin)*scale},${130-p[axis]*scale}`).join(' '));
            line.setAttribute('fill','none');line.setAttribute('stroke',id==='catheter'?'#61cbd6':'#ffce67');
            line.setAttribute('stroke-width',id==='catheter'?'7':'2');line.setAttribute('stroke-linejoin','round');$(plot).append(line);
        }
    }
}
function resultText(r) {
    return `${r.converged?'Równowaga osiągnięta':'Próba odrzucona: '+r.status}\n`+
        `${r.iterations} iteracji · ${r.ms.toFixed(2)} ms relaksacji · ${r.dofs} zmiennych · 0 kontaktów między narzędziami`+
        (r.residual?`\nBłąd długości: ${r.residual.length.toExponential(2)} mm · reszta sił: ${r.residual.force.toExponential(2)}`:'')+
        (r.error?`\n${r.error}`:'');
}
function reset() {
    const useSheath=$('sheath-mode').checked;
    const candidate=createSharedAxisNative({...useSheath?createSharedAxisContacts({sheath}):{},tools:[{id:'wire',insertion:useSheath?0:150,shaftStiffness:39,tipStiffness:30.7},
        {id:'catheter',insertion:useSheath?0:50,shaftStiffness:58.1,tipStiffness:87}]});
    for(const id of ids)$(`${id}-feed`).min=useSheath?0:10;
    const result=relaxSharedAxisNative(candidate);$('status').textContent=resultText(result);
    if(!result.converged)return;
    state=candidate;spins={wire:0,catheter:0};
    for(const id of ids){$(`${id}-feed`).value=state.materials.find(t=>t.spec.id===id).spec.insertion;$(`${id}-spin`).value=0;}
    labels();draw();
}
async function update() {
    const controls=[...document.querySelectorAll('input,button')];controls.forEach(c=>{c.disabled=true;});
    try {
        const targets=Object.fromEntries(ids.map(id=>[id,Number($(`${id}-feed`).value)]));
        const targetSpins=Object.fromEntries(ids.map(id=>[id,Number($(`${id}-spin`).value)]));
        const origins=Object.fromEntries(state.materials.map(t=>[t.spec.id,t.spec.insertion])),originSpins={...spins};
        const steps=Math.max(1,...ids.flatMap(id=>[Math.ceil(Math.abs(targets[id]-origins[id])/2),Math.ceil(Math.abs(targetSpins[id]-originSpins[id])/2)]));
        let result;
        for(let step=1;step<=steps;step++) {
            const nextSpins=Object.fromEntries(ids.map(id=>[id,originSpins[id]+(targetSpins[id]-originSpins[id])*step/steps]));
            const candidate=feedSharedAxisNative(state,Object.fromEntries(ids.map(id=>[id,origins[id]+(targets[id]-origins[id])*step/steps])));
            for(const id of ids)rotateSharedAxisNative(candidate,id,(nextSpins[id]-spins[id])*Math.PI/180);
            result=relaxSharedAxisNative(candidate);
            if(!result.converged)break;
            state=candidate;spins=nextSpins;
            if(step%10===0){draw();$('status').textContent=`Obliczanie: ${step}/${steps}`;await new Promise(resolve=>setTimeout(resolve,0));}
        }
        $('status').textContent=resultText(result)+(result.converged?'':'\nZatrzymano na ostatnim poprawnym stanie.');draw();
    } catch(error){$('status').textContent=`Próba odrzucona: ${error.message}. Zachowano ostatni poprawny stan.`;}
    for(const id of ids){$(`${id}-feed`).value=state.materials.find(t=>t.spec.id===id).spec.insertion;$(`${id}-spin`).value=spins[id];}
    labels();controls.forEach(c=>{c.disabled=false;});
}
for(const id of ids)for(const kind of ['feed','spin']){
    $(`${id}-${kind}`).addEventListener('input',labels);$(`${id}-${kind}`).addEventListener('change',update);
}
$('reset').addEventListener('click',reset);$('sheath-mode').addEventListener('change',reset);reset();
