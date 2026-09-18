/** Experimental contact working-set reduction between accepted steps.
 * Keep the nearest retained surface at every material site, including ties,
 * all nonzero reactions, and every site with friction memory. Other surfaces
 * can return through the complete finite-mesh discovery before acceptance.
 *
 * This changes Newton's candidate inequalities/directions. It is NOT a
 * bitwise-equivalent cache optimization; nonlinear certificates stay intact.
 * No rows are removed during a solve, and the incoming state is read-only.
 */
export function prunableSharedAxisWitnesses(state) {
    const dropped=new Set();
    if(!state.acceptedWallGaps||!state.wallSamples.some(sample=>sample.sharedAxisCompleteDiscovery))return dropped;
    const closest=new Map(),history=new Set((state.wallFrictionHistory??[]).map(h=>h.id));
    const site=row=>`${row.edge}/${row.witness.t}/${row.witness.owner??''}`;
    for(const row of state.definitions)if(row.witness) {
        const gap=state.acceptedWallGaps.get(row.id),key=site(row);
        if(Number.isFinite(gap))closest.set(key,Math.min(closest.get(key)??Infinity,gap));
    }
    state.definitions.forEach((row,i)=>{
        if(!row.witness||state.multipliers[i]!==0||history.has(`wire/${row.id}`)||history.has(`catheter/${row.id}`))return;
        const gap=state.acceptedWallGaps.get(row.id);
        // Missing/nonfinite measurements cannot authorize pruning. Keeping
        // nearly tied faces preserves corner/edge normal-cone candidates.
        if(Number.isFinite(gap)&&gap>closest.get(site(row))+1e-10)dropped.add(row);
    });
    return dropped;
}
