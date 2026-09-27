import test from 'node:test';
import assert from 'node:assert/strict';
import {previewFixture} from './helpers/stentGraftPreviewFixture.js';

for(const diameter of [23,28])for(const side of ['right','left'])test(`${side} / ${diameter} mm: exposed bifurcation cannot inherit shaft oscillations while the crown is captured`,()=>{
    const {system,device:d,sources}=previewFixture(side,'body',false);
    system.setDiameter(side,diameter);system.deploy(side);
    try {
        const first=d.parts[1].scaffoldRings[0];
        system.updateAccess(side,(d.coverLead+d.parts[1].releaseOffset+first.center+first.maxHeight/2+.1)/12,null,{deviceId:d.id,release:'sheath'});
        system.updateAccess(side,10,null,{deviceId:d.id});
        const part=d.parts[1],rows=Array.from(part.exposure).map((v,i)=>v>0?i:-1).filter(i=>i>=0);
        assert.ok(rows.length>1);assert.equal(d.tipRelease,0);
        // Establish release frames, then move the shaft without moving its handle.
        system.updateAccess(side,1/60,null,{deviceId:d.id});
        const before=Array.from(part.mesh.geometry.attributes.position.array),capture=system.captureForAccess(side);
        const original=sources[side].nodes.map(p=>p.clone());
        for(let step=0;step<60;step++) {
            sources[side].nodes.forEach((p,i)=>p.copy(original[i]).addScalar(.6*Math.sin(step)));
            system.updateAccess(side,1/60,null,{deviceId:d.id});
            for(const row of rows)assert.deepEqual(Array.from(part.mesh.geometry.attributes.position.array.slice(row*part.sides*3,(row+1)*part.sides*3)),before.slice(row*part.sides*3,(row+1)*part.sides*3));
        }
        assert.notDeepEqual(system.captureForAccess(side).offset,capture.offset,'capture must still transmit the changed shaft pose');
        assert.ok(system.mechanicalSurfaceForAccess(side).ownedBranches.length,'ipsilateral contact remains active');
    }finally{system.dispose();}
});

for(const diameter of [23,28])for(const side of ['right','left'])test(`${side} / ${diameter} mm: continuous sheath motion through bifurcation gives bounded exposed-row motion`,()=>{
    const {system,device:d,sources}=previewFixture(side,'body',false);
    system.setDiameter(side,diameter);system.deploy(side);
    try {
        const original=sources[side].nodes.map(p=>p.clone());
        system.updateAccess(side,55/12,null,{deviceId:d.id,release:'sheath'});
        let maximum=0;
        for(let step=0;step<150;step++) {
            const part=d.parts[1],old=Array.from(part.mesh.geometry.attributes.position.array),exposure=Array.from(part.exposure);
            sources[side].nodes.forEach((p,i)=>{p.copy(original[i]);p.x+=.5*Math.sin(step*.9);});
            system.updateAccess(side,1/60,null,{deviceId:d.id,release:'sheath'});
            const positions=part.mesh.geometry.attributes.position.array;
            for(let row=0;row<part.rows;row++)if(exposure[row]>0)for(let j=0;j<part.sides;j++) {
                const index=(row*part.sides+j)*3;
                maximum=Math.max(maximum,Math.hypot(...[0,1,2].map(k=>positions[index+k]-old[index+k])));
            }
        }
        assert.ok(maximum<1,`exposed fabric jumped ${maximum} mm in one 0.2 mm cover movement`);
        assert.equal(d.tipRelease,0);assert.ok(d.sheathWithdrawal>84.9);
    }finally{system.dispose();}
});

for(const diameter of [23,28])for(const side of ['right','left'])test(`${side} / ${diameter} mm: captured release anchors follow commanded roll and recapture follows the cover`,()=>{
    const {system,device:d,sources}=previewFixture(side,'body',false);
    system.setDiameter(side,diameter);system.deploy(side);
    try {
        const first=d.parts[1].scaffoldRings[0];
        system.updateAccess(side,(d.coverLead+d.parts[1].releaseOffset+first.center+first.maxHeight/2+.1)/12,null,{deviceId:d.id,release:'sheath'});
        system.updateAccess(side,10,null,{deviceId:d.id});
        system.updateAccess(side,1/60,null,{deviceId:d.id});
        const part=d.parts[1],old=part.folded.map(p=>p.clone()),frames=part.foldedFrames.map(f=>f.u.clone());
        const distance=old[0].distanceTo(old[1]);
        system.updateAccess(side,1/60,null,{deviceId:d.id,mechanicalRotation:.4});
        assert.ok(part.folded[0].distanceTo(old[0])>.01);
        assert.ok(Math.abs(part.folded[0].distanceTo(part.folded[1])-distance)<1e-8);
        assert.ok(part.foldedFrames[0].u.distanceTo(frames[0])>.01);
        system.updateAccess(side,1/60,null,{deviceId:d.id,mechanicalRotation:0});
        assert.ok(part.folded[0].distanceTo(old[0])<1e-8,'back-and-forth rotation must not accumulate anchor drift');
        system.updateAccess(side,5,null,{deviceId:d.id,release:'resheath'});
        sources[side].nodes.forEach(p=>p.x+=1);
        system.updateAccess(side,1/60,null,{deviceId:d.id});
        assert.equal(part.exposure[0],0);
        assert.ok(part.folded[0].distanceTo(old[0])>.9,'re-covered branch follows the delivery path again');
    }finally{system.dispose();}
});
