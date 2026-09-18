import assert from 'node:assert/strict';
import {setImmediate as nextTurn} from 'node:timers/promises';

// A separate process makes forced GC available without requiring test-runner
// flags. Never inspect WeakRefs between collections: deref keeps the target
// alive through the current JavaScript job, independently of solver ownership.
assert.equal(typeof global.gc,'function','run this probe with --expose-gc');
const sourceRoot=new URL(process.argv[2]);
const {createSharedAxisNative,feedSharedAxisNative}=await import(new URL('src/physics/kirchhoffSharedAxisNative.js',sourceRoot));
const {stepSharedAxis}=await import(new URL('src/physics/kirchhoffSharedAxisTimeStep.js',sourceRoot));
const {defineKirchhoffMaterialProfile}=await import(new URL('src/physics/kirchhoffMaterialProfile.js',sourceRoot));
const {sharedAxisMaterialKernelWorkspace}=await import(new URL('src/physics/kirchhoffSharedAxisMaterialKernel.js',sourceRoot));
const wasmMaterial=process.argv[3]==='wasm',dynamic=wasmMaterial||process.argv[3]==='dynamic',count=dynamic?30:160;
const beam=defineKirchhoffMaterialProfile({id:'memory-lifecycle-beam',sampleEI1:()=>1e6,sampleGJ:()=>1e6/1.3});
const oldStates=[],oldPositions=[],oldBodies=[],oldMaterialMemories=[];
let factorizations=0;
function build() {
    let state=createSharedAxisNative({tools:[
        {id:'wire',insertion:30,type:beam},{id:'catheter',insertion:20,type:beam}
    ],startCoordinate:-10,maxBendAngle:Math.PI/4});
    for(let i=0;i<count;i++) {
        if(dynamic) {
            state.loads[state.layout.positions.at(-1)+1]=1;
            const result=stepSharedAxis(state,1/60,{liveWallNormalLoad:true,wasmMaterial,reuseConstraintWork:wasmMaterial,reuseMatrixAssembly:wasmMaterial,reuseRowBuffers:wasmMaterial,reuseMaterialScratch:wasmMaterial,lightweightFriction:wasmMaterial});
            assert.ok(result.converged,JSON.stringify(result));
            factorizations+=result.factorizations;
            if(wasmMaterial) {
                const w=sharedAxisMaterialKernelWorkspace(state.chain,state.materials.reduce((sum,m)=>sum+Math.max(0,m.last-1),0));
                oldMaterialMemories.push(new WeakRef(w.memory.buffer));
            }
        }
        oldStates.push(new WeakRef(state));
        oldPositions.push(new WeakRef(state.positions));
        oldBodies.push(new WeakRef(state.materials[0].body));
        state=feedSharedAxisNative(state,{wire:30+.02*(i+1),catheter:20+.01*(i+1)});
    }
    // Keep the current simulation alive explicitly. The regression was an
    // escaping bend-limit closure retaining all earlier constructor contexts.
    globalThis.currentMemoryProbeState=state;
    return {latest:new WeakRef(state),cache:state.mixed.activeWorkspaces?new WeakRef(state.mixed.activeWorkspaces):null};
}
async function collect() {
    for(let i=0;i<4;i++){await nextTurn();global.gc();}
}
const alive=refs=>refs.reduce((n,r)=>n+(r.deref()===undefined?0:1),0);
const {latest,cache}=build();
await collect();
const held={states:alive(oldStates),positions:alive(oldPositions),bodies:alive(oldBodies),materialMemories:alive(oldMaterialMemories),
    latestAlive:latest.deref()!==undefined,cacheSize:cache?.deref()?.size??0};
globalThis.currentMemoryProbeState=null;
await collect();
const released={states:alive(oldStates),positions:alive(oldPositions),bodies:alive(oldBodies),materialMemories:alive(oldMaterialMemories),
    latestAlive:latest.deref()!==undefined,cacheAlive:cache?.deref()!==undefined};
process.stdout.write(JSON.stringify({count,dynamic,factorizations,materialMemoryCount:oldMaterialMemories.length,held,released}));
