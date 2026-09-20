import test from 'node:test';
import assert from 'node:assert/strict';
import * as THREE from 'three';
import {HybridContrastSystem} from '../src/contrast/hybridContrastSystem.js';
import {DsaRoadmapState} from '../src/imaging/dsaRoadmapState.js';

function contrastModel() {
    return new HybridContrastSystem({centerlineSegments:[{
        id:0,nodeStartId:0,nodeEndId:1,start:new THREE.Vector3(0,0,0),end:new THREE.Vector3(0,60,0),
        radiusStart:8,radiusEnd:8,safeRadius:8
    }]});
}

test('contrast clock counts only completed substeps, including idle flow and substep limits',()=>{
    const contrast=contrastModel();
    contrast.update(0);assert.equal(contrast.simulationTimeSeconds,0);
    contrast.update(1/60);assert.equal(contrast.simulationTimeSeconds,0);
    contrast.update(1/60);assert.ok(Math.abs(contrast.simulationTimeSeconds-1/30)<1e-10);
    contrast.update(10);
    assert.ok(Math.abs(contrast.simulationTimeSeconds-5/30)<1e-10,'unprocessed backlog must not advance DSA');
    assert.equal(contrast.getMetrics().simulationTimeSeconds,contrast.simulationTimeSeconds);
});

function recordAtSpeed(speed) {
    const contrast=contrastModel();
    const state=new DsaRoadmapState({nowMs:()=>contrast.simulationTimeSeconds*1000});
    state.startSequenceRecording();
    let backlog=0,lastPulse=-Infinity,finishedAtWallMs=null;
    // The display still pulses at 15 real Hz, but physics can be ten times slower.
    for(let frame=0;frame<15000;frame++) {
        const wallMs=frame*1000/120;
        backlog+=speed/120;
        while(backlog+1e-10>=1/60){contrast.update(1/60);backlog-=1/60;}
        if(wallMs-lastPulse+1e-8<1000/15)continue;
        lastPulse=wallMs;
        if(state.isMaskCaptureReady()) {state.markMaskCaptured(1);continue;}
        if(!state.isRecordingFrameDue({frameIntervalMs:1000/15}))continue;
        const result=state.appendRecordingFrame({frameIntervalMs:1000/15});
        assert.equal(result.ok,true);
        if(result.shouldStop) {
            state.finishSequenceRecording();finishedAtWallMs=wallMs;break;
        }
    }
    assert.notEqual(finishedAtWallMs,null);
    return {sequence:state.getSnapshot().sequences[0],finishedAtWallMs};
}

test('120-frame acquisition covers the same contrast time at normal and 10× slower physics',()=>{
    const normal=recordAtSpeed(1),slow=recordAtSpeed(.1);
    assert.equal(normal.sequence.frames.length,120);assert.equal(slow.sequence.frames.length,120);
    assert.ok(slow.finishedAtWallMs>normal.finishedAtWallMs*9);
    assert.ok(Math.abs(normal.sequence.endedAtMs-slow.sequence.endedAtMs)<70);
    assert.ok(slow.sequence.endedAtMs>=8600,'650 ms preparation plus 8 s acquisition');
    for(const sequence of [normal.sequence,slow.sequence])for(let i=1;i<sequence.frames.length;i++)
        assert.ok(sequence.frames[i].capturedAtMs-sequence.frames[i-1].capturedAtMs>=1000/15-1e-6);
});

test('stalled simulation cannot fill the archive or advance cine; seek, rate and invalidation share its clock',()=>{
    let simulatedMs=0;
    const state=new DsaRoadmapState({nowMs:()=>simulatedMs,preparationDelayMs:0});
    state.startSequenceRecording();state.markMaskCaptured(1);
    for(let i=0;i<200;i++)assert.equal(state.isRecordingFrameDue({frameIntervalMs:100}),false);
    assert.equal(state.appendRecordingFrame().ok,false);
    for(let i=1;i<=6;i++){simulatedMs=i*100;assert.equal(state.appendRecordingFrame().ok,true);}
    simulatedMs=650;state.invalidateForGeometryRevision(2);
    assert.equal(state.getSnapshot().sequences[0].endedAtMs,650);
    state.playCine();
    for(let i=0;i<100;i++)state.advanceCine();
    assert.equal(state.cineFrameIndex,0);
    simulatedMs+=200;state.advanceCine();assert.equal(state.cineFrameIndex,2);
    state.pauseCine();simulatedMs+=1000;state.advanceCine();assert.equal(state.cineFrameIndex,2);
    state.seekCineFrame(1);state.setCinePlaybackRate(2);state.playCine();
    simulatedMs+=100;state.advanceCine();assert.equal(state.cineFrameIndex,3);
    state.seekCineFrame(0);simulatedMs+=50;state.advanceCine();assert.equal(state.cineFrameIndex,1);
});
