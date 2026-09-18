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
