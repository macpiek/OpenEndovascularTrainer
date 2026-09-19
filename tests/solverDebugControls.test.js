import test from 'node:test';
import assert from 'node:assert/strict';
import {solverDebugUrl} from '../src/ui/solverDebugControls.js';
import {createRodNodesDebug} from '../src/ui/rodNodesDebug.js';

test('solver navigation preserves unrelated URL parameters and selects a clean restart',()=>{
    const next=new URL(solverDebugUrl('http://localhost:5173/?foo=1&experimentalSplitMotion=1&coupledLinearSolver=axial-band#view','shared-axis-adaptive'));
    assert.equal(next.searchParams.get('coupledSolver'),'shared-axis-adaptive');
    assert.equal(next.searchParams.get('foo'),'1');assert.equal(next.hash,'#view');
    assert.equal(next.searchParams.get('solverDebug'),'1');assert.equal(next.searchParams.has('experimentalSplitMotion'),false);
    assert.throws(()=>solverDebugUrl(next.href,'unknown'),/Unknown/);
    assert.equal(new URL(solverDebugUrl(next.href,'shared-axis-projective',false)).searchParams.get('coupledSolver'),'shared-axis-projective');
    const enabled=solverDebugUrl(next.href,'shared-axis-adaptive',true);
    assert.equal(new URL(enabled).searchParams.get('modifiedNewton'),'1');
    assert.equal(new URL(solverDebugUrl(enabled,'shared-axis',false)).searchParams.has('modifiedNewton'),false);
});

test('node overlay uses accepted mechanical positions, supports legacy bodies and clears reset views',()=>{
    const debug=createRodNodesDebug();
    const body={jointStateView:{positions:Float64Array.from([1,2,3,10,20,30])},activeStart:0,activeEnd:0,x:[9],y:[8],z:[7]};
    debug.update(body,null,true);
    const wire=debug.group.children[1];
    assert.equal(wire.geometry.drawRange.count,2);
    assert.deepEqual(Array.from(wire.geometry.attributes.position.array.slice(0,6)),[1,2,3,10,20,30]);
    const attribute=wire.geometry.attributes.position;debug.update(body,null,true);
    assert.equal(wire.geometry.attributes.position,attribute);
    body.jointStateView=null;debug.update(body,null,true);
    assert.equal(wire.geometry.drawRange.count,1);
    assert.deepEqual(Array.from(attribute.array.slice(0,3)),[9,8,7]);
    debug.update(null,null,true);assert.equal(wire.geometry.drawRange.count,0);
    debug.update(body,null,false);assert.equal(debug.group.visible,false);debug.dispose();
});

test('contact pruning URL is explicit, reversible and independent of Newton mode',()=>{
    const base='http://localhost:5178/?foo=1&coupledSolver=shared-axis-adaptive';
    const enabled=new URL(solverDebugUrl(base,'shared-axis-adaptive',false,true));
    assert.equal(enabled.searchParams.get('pruneWitnesses'),'1');assert.equal(enabled.searchParams.get('foo'),'1');
    const disabled=new URL(solverDebugUrl(enabled.href,'shared-axis-adaptive',true,false));
    assert.equal(disabled.searchParams.has('pruneWitnesses'),false);assert.equal(disabled.searchParams.get('modifiedNewton'),'1');
});

test('fast Newton URL override is reversible without dropping the other solver controls',()=>{
    const base='http://localhost:5178/?coupledSolver=shared-axis-adaptive&foo=1';
    assert.equal(new URL(solverDebugUrl(base,'shared-axis-adaptive')).searchParams.has('fastNewton'),false);
    const enabled=new URL(solverDebugUrl(base,'shared-axis-adaptive',false,true,true));
    assert.equal(enabled.searchParams.get('fastNewton'),'1');assert.equal(enabled.searchParams.get('pruneWitnesses'),'1');
    const disabled=new URL(solverDebugUrl(enabled.href,'shared-axis-adaptive',true,true,false));
    assert.equal(disabled.searchParams.get('fastNewton'),'0');assert.equal(disabled.searchParams.get('modifiedNewton'),'1');
    assert.equal(disabled.searchParams.get('foo'),'1');assert.equal(disabled.searchParams.get('pruneWitnesses'),'1');
});

test('fast Newton checkbox defaults on, enables restart and is cleared when switching to projective dynamics',async()=>{
    const {initSolverDebugControls}=await import('../src/ui/solverDebugControls.js');
    const element=()=>({checked:false,disabled:false,listeners:{},addEventListener(name,fn){this.listeners[name]=fn;}});
    const select={...element(),append(){},ownerDocument:{createElement(){return {};}}},button=element(),fastNewtonToggle=element();let navigated;
    initSolverDebugControls({select,button,fastNewtonToggle,current:'shared-axis-adaptive',href:'http://localhost:5178/?pruneWitnesses=1',navigate:url=>navigated=url});
    assert.equal(button.disabled,true);assert.equal(fastNewtonToggle.checked,true);
    fastNewtonToggle.checked=false;fastNewtonToggle.listeners.change();assert.equal(button.disabled,false);
    button.listeners.click();assert.equal(new URL(navigated).searchParams.get('fastNewton'),'0');
    select.value='shared-axis-projective';select.listeners.change();assert.equal(fastNewtonToggle.disabled,true);
    button.listeners.click();assert.equal(new URL(navigated).searchParams.get('fastNewton'),'0');
});

test('predictive Newton defaults on and can restart independently of fast friction',async()=>{
    const {initSolverDebugControls}=await import('../src/ui/solverDebugControls.js');
    const element=()=>({checked:false,disabled:false,listeners:{},addEventListener(name,fn){this.listeners[name]=fn;}});
    const select={...element(),append(){},ownerDocument:{createElement(){return {};}}},button=element(),fastNewtonToggle=element(),predictiveNewtonToggle=element();let navigated;
    initSolverDebugControls({select,button,fastNewtonToggle,predictiveNewtonToggle,current:'shared-axis-adaptive',href:'http://localhost:5178/?pruneWitnesses=1',navigate:url=>navigated=url});
    assert.equal(predictiveNewtonToggle.checked,true);assert.equal(button.disabled,true);
    predictiveNewtonToggle.checked=false;predictiveNewtonToggle.listeners.change();assert.equal(button.disabled,false);
    button.listeners.click();const url=new URL(navigated);
    assert.equal(url.searchParams.get('predictiveNewton'),'0');assert.equal(url.searchParams.get('fastNewton'),'1');assert.equal(url.searchParams.get('pruneWitnesses'),'1');
    fastNewtonToggle.checked=false;fastNewtonToggle.listeners.change();assert.equal(predictiveNewtonToggle.disabled,true);
});
