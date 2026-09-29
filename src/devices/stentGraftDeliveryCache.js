// Compare numerical path contents: committed solvers can reuse node objects,
// while getPath can allocate a new immutable path with identical coordinates.
export function unchangedDeliveryPose(device,wire,tip) {
    const head=[device.position,device.deliveryRotation,device.graftRotation,tip,
        device.sheathWithdrawal??0,device.phase==='loaded'?0:device.phase==='deploying'?1:2,
        device.diameter,device.distalDiameter,device.dimensionScale,device.length];
    const length=head.length+wire.points.length*4;
    const state=device.deliveryGeometryState;
    let same=!!state&&state.length===length;
    const next=same?state:new Float64Array(length);
    let cursor=0;
    const put=value=>{if(next[cursor]!==value)same=false;next[cursor++]=value;};
    for(const value of head)put(value);
    for(let i=0;i<wire.points.length;i++) {
        const p=wire.points[i];put(p.x);put(p.y);put(p.z);put(wire.coordinates[i]);
    }
    device.deliveryGeometryState=next;
    return same;
}
