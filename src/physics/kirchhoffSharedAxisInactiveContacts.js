// Private deferred geometry. A lower gap bound is usable only for an unloaded
// unilateral row. It never replaces a loaded force or an active Jacobian.
// Row banks own the saved pose: rejected trials and generator yields may move
// the state before a protected measure is consumed again.
const work = new WeakMap();

export function clearDeferredSharedAxisContact(row) {
    const w=work.get(row);if(w)w.pending=false;
}

export function deferSharedAxisContact(row,evaluate,input,lowerGap,cache=null,pose=null) {
    if(row.kind!=='wall'||row.multiplier!==0||!(lowerGap>0)||!Number.isFinite(lowerGap))
        throw new RangeError('Only certified separated zero-reaction contacts can be deferred');
    let w=work.get(row);
    if(!w) {
        w={pending:false,evaluate:null,input:{state:{origin:[0,0,0]},a:[0,0,0],b:[0,0,0],
            needHessian:false,reuseGeometry:true,contactStorage:{contact:{gap:NaN,jacobian:new Array(6)},hessian:null}}};
        work.set(row,w);
    }
    // Production shares one immutable position snapshot across all contacts
    // at this pose. Standalone callers retain the defensive copy contract.
    w.input.a=pose?pose.positions[row.edge]:input.a.slice();
    w.input.b=pose?pose.positions[row.edge+1]:input.b.slice();
    w.input.state.origin=pose?pose.origin:(input.state.origin??[0,0,0]).slice();
    w.input.radius=input.radius;w.input.state.geometryKey=input.state.geometryKey;w.cache=cache;w.evaluate=evaluate;w.pending=true;
    row.gap=lowerGap;row.jacobian.fill(0);row.geometricHessian=undefined;
}

export function materializeSharedAxisContact(row) {
    const w=work.get(row);if(!w?.pending)return row;
    const cached=w.cache?.key===w.input.state.geometryKey&&w.cache?.contact;
    if(w.cache&&!cached)w.cache.key=null;
    const contact=cached||w.evaluate(w.input);
    if(!Number.isFinite(contact?.gap)||contact.jacobian?.length!==row.dofs.length||!contact.jacobian.every(Number.isFinite))
        throw new RangeError('Deferred wall contact must supply exact finite geometry');
    if(w.cache&&!cached) {w.cache.key=w.input.state.geometryKey;w.cache.contact=contact;w.cache.withHessian=false;}
    row.gap=contact.gap;
    for(let k=0;k<row.jacobian.length;k++)row.jacobian[k]=contact.jacobian[k];
    w.pending=false;return row;
}

export function materializeSharedAxisContacts(rows) {
    for(const row of rows)materializeSharedAxisContact(row);
    return rows;
}

// For a witness at t in [0,1], |J dx| <= max(|dx_a|,|dx_b|).
// L1 endpoint norms conservatively bound the Euclidean norms, without sqrt.
// Check every new active-set direction, not just the accepted physical pose.
// If the bound cannot prove separation, resolve the ORIGINAL row before the
// ordinary violation/activation ranking. Never activate a surrogate row.
export function separatedSharedAxisContactDirection(row,increment) {
    if(!work.get(row)?.pending)return false;
    const d=row.dofs;
    const movement=Math.max(Math.abs(increment[d[0]])+Math.abs(increment[d[1]])+Math.abs(increment[d[2]]),
        Math.abs(increment[d[3]])+Math.abs(increment[d[4]])+Math.abs(increment[d[5]]));
    if(Number.isFinite(movement)&&row.gap>movement+256*Number.EPSILON*Math.max(1,movement))return true;
    materializeSharedAxisContact(row);return false;
}
