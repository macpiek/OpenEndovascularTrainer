import test from 'node:test';
import assert from 'node:assert/strict';
import {resolveAnatomyVariant,anatomyVariantUrl} from '../src/anatomyVariant.js';
import {initAnatomyControls} from '../src/ui/anatomyControls.js';

test('anatomy URLs preserve solver settings and can restore the baseline',()=>{
    const href='http://localhost:5178/?coupledSolver=shared-axis-realtime&fastNewton=1#scene';
    const next=new URL(anatomyVariantUrl(href,'infrarenal-aneurysm'));
    assert.equal(next.searchParams.get('coupledSolver'),'shared-axis-realtime');
    assert.equal(next.searchParams.get('fastNewton'),'1');
    assert.equal(next.searchParams.get('panel'),'anatomy');
    assert.equal(next.hash,'#scene');
    assert.equal(resolveAnatomyVariant(next.search).id,'infrarenal-aneurysm');
    assert.equal(resolveAnatomyVariant(new URL(anatomyVariantUrl(next.href,'baseline')).search).id,'baseline');
    assert.equal(resolveAnatomyVariant('?anatomy=constructor').id,'baseline');
    assert.throws(()=>anatomyVariantUrl(href,'invalid'));
});

test('anatomy selection previews the choice without resetting until Apply',()=>{
    const element=()=>({value:'',textContent:'',disabled:false,handlers:{},addEventListener(type,fn){this.handlers[type]=fn;}});
    const select=element(),button=element(),description=element(),status=element(),navigations=[];
    initAnatomyControls({select,button,description,status,href:'http://localhost:5178/',navigate:url=>navigations.push(url)});
    assert.equal(select.value,'baseline');assert.equal(button.disabled,true);
    select.value='infrarenal-aneurysm';select.handlers.change();
    assert.equal(button.disabled,false);assert.match(description.textContent,/50 mm/);
    assert.equal(navigations.length,0);
    button.handlers.click();
    assert.equal(button.disabled,true);assert.equal(navigations.length,1);
    assert.equal(resolveAnatomyVariant(new URL(navigations[0]).search).id,'infrarenal-aneurysm');
    initAnatomyControls({select,button,description,status,href:navigations[0],navigate:url=>navigations.push(url)});
    assert.equal(button.disabled,true);
    select.value='baseline';select.handlers.change();
    assert.match(button.textContent,/Przywróć/);
    button.handlers.click();assert.equal(resolveAnatomyVariant(new URL(navigations[1]).search).id,'baseline');
});
