import test from 'node:test';
import assert from 'node:assert/strict';
import {readFileSync} from 'node:fs';
import vm from 'node:vm';
const source=readFileSync(new URL('../src/simulator.js',import.meta.url),'utf8');
const fn=source.slice(source.indexOf('function sampleGuidewireContactMarkers()'),source.indexOf('function updateGuidewireResistance()'));

test('hidden wall overlay clears stale markers without resampling geometry; visible overlay still samples',()=>{
    for(const [fluoroscopy,wallContacts] of [[true,true],[false,false],[false,true]]) {
        let queries=0,clears=0;
        const marker=()=>({count:12,userData:{hasPoint:true},visible:true});
        const state={fluoroscopy,debugLayerVisibility:{wallContacts},ui:{updateGuidewireDiagnostics:()=>clears++},
            wallContactMarkers:marker(),wallBreachMarkers:marker(),wallWorstPointMarker:marker(),
            xpbdContactNormalLines:{geometry:{setDrawRange:()=>{}}},xpbdActiveBranchLines:{geometry:{setDrawRange:()=>{}}},
            guidewireTransport:{meshClearance:0,collectLumenDiagnostics:()=>{queries++;throw Error('sampled');}},vesselCollisionTarget:{},GUIDEWIRE_DIAGNOSTIC_CONTACT_BAND:1,CONTACT_MARKER_LIMIT:420};
        vm.createContext(state);vm.runInContext(fn,state);
        if(!fluoroscopy&&wallContacts){assert.throws(()=>state.sampleGuidewireContactMarkers(),/sampled/);assert.equal(queries,1);}
        else {state.sampleGuidewireContactMarkers();assert.equal(queries,0);assert.equal(clears,1);assert.equal(state.wallContactMarkers.count,0);assert.equal(state.wallBreachMarkers.count,0);assert.equal(state.wallWorstPointMarker.visible,false);}
    }
});
