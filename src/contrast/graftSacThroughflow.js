/** Steady flow on the native sac tree, with gate inflows and patent outlets.
 * Factor the tree once per geometry revision; implicit upwind transport then
 * costs one pass and remains positive even for tiny annular atlas cells. */
export function createSacThroughflow(cells, links, drains, feeds, discharge) {
    const count=cells.length,adj=Array.from({length:count},()=>[]);
    const diagonal=new Float64Array(count),rhs=new Float64Array(count);
    const pressure=new Float64Array(count),parent=new Int32Array(count).fill(-2);
    const parentG=new Float64Array(count),outgoing=Array.from({length:count},()=>[]);
    for(const link of links) {
        const a=cells[link.a],b=cells[link.b];
        const area=Math.min(a.volume/a.length,b.volume/b.length);
        const g=area*area/Math.max(1e-6,(a.length+b.length)*.5);
        adj[link.a].push({b:link.b,g});adj[link.b].push({b:link.a,g});
        diagonal[link.a]+=g;diagonal[link.b]+=g;
    }
    for(const d of drains) {
        d.conductance=d.weight*d.weight/Math.max(1e-6,cells[d.a].length);
        diagonal[d.a]+=d.conductance;
    }
    for(const f of feeds)rhs[f.inlet]+=f.q;
    // Root each connected component at an outlet. Components without any
    // outlet have no throughflow: a sealed/dead-end pocket is not erased.
    for(const drain of drains) {
        const root=drain.a;if(parent[root]!==-2)continue;
        const order=[root];parent[root]=-1;
        for(let i=0;i<order.length;i++)for(const {b,g} of adj[order[i]]) {
            if(parent[b]!==-2)continue;
            parent[b]=order[i];parentG[b]=g;order.push(b);
        }
        for(let i=order.length-1;i>0;i--) {
            const a=order[i],p=parent[a],ratio=parentG[a]/diagonal[a];
            diagonal[p]-=parentG[a]*ratio;rhs[p]+=rhs[a]*ratio;
        }
        pressure[root]=rhs[root]/diagonal[root];
        for(let i=1;i<order.length;i++) {
            const a=order[i];pressure[a]=(rhs[a]+parentG[a]*pressure[parent[a]])/diagonal[a];
        }
    }
    const outflow=new Float64Array(count),incoming=new Int32Array(count);
    for(const {a,b} of links) {
        const g=adj[a].find(v=>v.b===b).g;
        const q=g*(pressure[a]-pressure[b]);
        if(Math.abs(q)<1e-8)continue;
        const from=q>0?a:b,to=q>0?b:a,flow=Math.abs(q);
        outgoing[from].push({to,q:flow});outflow[from]+=flow;incoming[to]++;
    }
    for(const d of drains) {
        d.q=Math.max(0,d.conductance*pressure[d.a]);outflow[d.a]+=d.q;
        outgoing[d.a].push({to:-1,q:d.q,drain:d});
    }
    const order=[];
    for(let i=0;i<count;i++)if(!incoming[i])order.push(i);
    for(let i=0;i<order.length;i++)for(const edge of outgoing[order[i]])
        if(edge.to>=0&&!--incoming[edge.to])order.push(edge.to);
    return {outflow,update(dt) {
        for(const i of order) {
            if(!(outflow[i]>0))continue;
            const c=cells[i],before=c.entry.mass[c.cellIndex];
            const remaining=before/(1+dt*outflow[i]/c.volume);
            const amount=before-remaining;c.entry.mass[c.cellIndex]=remaining;
            for(const edge of outgoing[i]) {
                const transferred=amount*edge.q/outflow[i];
                if(edge.to>=0) {
                    const next=cells[edge.to];next.entry.mass[next.cellIndex]+=transferred;
                } else if(transferred>0)discharge(edge.drain,transferred);
            }
        }
    }};
}
