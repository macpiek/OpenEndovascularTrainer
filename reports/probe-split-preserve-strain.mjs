import {fixture,DT,xyz,axes,source} from '../tests/fixtures/splitMotionAnalyticWorld.js';
const { configureKirchhoffSplitBias } = await import(source('src/physics/kirchhoffSplitMotion.js'));
const { assembleKirchhoffDirect } = await import(source('src/physics/kirchhoffDirectSolver.js'));
const rows=[];
for(const materialMode of ['physical-compliance','preserve-strain']) {
    const f=fixture({wall:true,y:-.5});
    configureKirchhoffSplitBias(f.constraint,{materialMode});
    for(let i=0;i<f.wire.count;i++)f.wire.setNodePosition(i,i-1,-.5+.2*(i-1),0);
    f.wire.captureRestConfiguration();f.wire.copyCurrentToPrevious();
    const bodies=[f.wire,f.catheter];
    const alphaBefore=bodies.map(b=>assembleKirchhoffDirect(b,DT).alpha.slice());
    let reference;
    const solve=f.world.coupledSystem.solve;
    f.world.coupledSystem.solve=(c,dt,o)=>{
        if(c._splitMotion.phase==='bias'&&!reference)reference=c._splitMotion.materialStrainOffsets.map(a=>a.slice());
        return solve(c,dt,o);
    };
    f.world.stepFixed();
    const d=f.world.getStats().jointMotion;
    let strainDrift=0,alphaDrift=0;
    for(const [side,b]of bodies.entries()) {
        const m=assembleKirchhoffDirect(b,DT);
        for(let i=0;i<m.rowCount;i++) {
            strainDrift=Math.max(strainDrift,Math.abs(m.strain[i]-reference[side][i]));
            alphaDrift=Math.max(alphaDrift,Math.abs(m.alpha[i]-alphaBefore[side][i]));
        }
    }
    rows.push({materialMode,rawMinimumGap:Math.min(...f.wire.y.map((y,i)=>-y-f.wire.nodeRadius[i])),
        strainDrift,alphaDrift,position:xyz.map(a=>[...f.wire[a]]),
        velocity:axes.map(a=>[...f.wire['velocity'+a]]),omega:axes.map(a=>[...f.wire['angularVelocity'+a]]),
        physicalNormal:d.contacts.reduce((s,c)=>s+c.normalPhysical,0),
        physicalTangent:d.contacts.reduce((s,c)=>s+Math.hypot(...c.tangentPhysical),0),
        diagnostics:d});
}
console.log(JSON.stringify(rows,null,2));
