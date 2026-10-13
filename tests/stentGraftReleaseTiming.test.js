import test from 'node:test';
import assert from 'node:assert/strict';
import {advanceRelease,sewnRingExposure} from '../src/devices/stentGraftDeployment.js';

function device(type='body') {
    const part=()=>({releaseOffset:0,path:{length:11},scaffoldRings:[{center:6,maxHeight:10}]});
    return {type,phase:'deploying',position:100,implantPosition:100,coverLead:0,
        sheathWithdrawal:10.8,sheathTravel:11.05,releaseLength:11,tipRelease:type==='body'?0:1,
        gateOpening:0,gateFreeTime:0,ipsiOpening:0,ipsiFreeTime:0,
        limbOpening:0,limbFreeTime:0,limbReleased:false,parts:type==='body'?[part(),part(),part()]:[part()]};
}

for(const type of ['body','limb'])for(const speed of [.25,.5,1])
test(`${type}: ring and outlet opening uses time after clearance at speed ${speed}, even when the sleeve stops`,()=>{
    const coarse=device(type),fine=device(type),dt=.1,control={sheath:speed};
    advanceRelease(coarse,dt,control);
    for(let i=0;i<6;i++)advanceRelease(fine,dt/6,control);
    const freeTime=dt-.2/(12*speed),t=45*freeTime,expected=t>=8?1:1-(1+t)*Math.exp(-t);
    for(let i=0;i<coarse.parts.length;i++) {
        const a=coarse.parts[i].scaffoldRings[0],b=fine.parts[i].scaffoldRings[0];
        assert.ok(Math.abs(a.freeTime-freeTime)<1e-12);
        assert.ok(Math.abs(a.opening-expected)<1e-12);
        assert.ok(Math.abs(a.opening-b.opening)<1e-12,'opening cannot depend on frame subdivision');
    }
    for(const prefix of type==='body'?['gate','ipsi']:['limb']) {
        assert.ok(Math.abs(coarse[`${prefix}Opening`]-expected)<1e-12);
        assert.ok(Math.abs(coarse[`${prefix}Opening`]-fine[`${prefix}Opening`])<1e-12);
    }
    const edge=coarse.sheathWithdrawal;
    advanceRelease(coarse,.25,null);
    assert.equal(coarse.sheathWithdrawal,edge);
    assert.ok(coarse.parts.every(p=>p.scaffoldRings[0].opening===1),'freed bands keep opening with a stationary sleeve');
});

test('ring-only relaxation reports a change, while a covered ring cannot expand with time',()=>{
    const d=device();d.parts=[d.parts[0]];
    d.sheathWithdrawal=5;
    advanceRelease(d,.01,null);
    const exposure=sewnRingExposure(d.parts[0],2);
    assert.equal(advanceRelease(d,.1,null),false);
    assert.equal(sewnRingExposure(d.parts[0],2),exposure);
    assert.equal(d.parts[0].scaffoldRings[0].opening,0);
    d.sheathWithdrawal=11;
    assert.equal(advanceRelease(d,.01,null),true);
    assert.equal(advanceRelease(d,.01,null),true,'stationary sleeve must not hide continuing ring expansion');
});
