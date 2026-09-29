import {createSacThroughflow} from './graftSacThroughflow.js';

/** Separate finite-volume pool outside an uncompleted bifurcated graft.
 * Iodine enters at an exposed distal outlet (release frontier or open gate). Steady throughflow carries iodine to patent native outlets; symmetric exchange
 * also models mixing into and out of side pockets. This is a reduced
 * transport model, not a pressure/velocity solution of the aneurysm volume. */
export function createGraftSacTransport(network, remodeling) {
    const gates=remodeling.surface.openGates??[remodeling.surface.openGate].filter(Boolean);
    if(!gates.length||remodeling.surface.sealed)return null;
    const cells=[],byEdge=new Map(),links=[],drains=[];
    for(const [edgeIndex,entry] of remodeling.trapped) {
        const edge=network.edges[edgeIndex],indices=new Int32Array(edge.cellCount).fill(-1);
        for(let i=0;i<edge.cellCount;i++)if(entry.volumes[i]>1e-6) {
            indices[i]=cells.length;
            cells.push({edgeIndex,cellIndex:i,entry,volume:entry.volumes[i],length:edge.cellLength});
        }
        byEdge.set(edgeIndex,indices);
    }
    const connect=(a,b)=>{
        if(a<0||b<0)return;
        const x=cells[a],y=cells[b],length=(x.length+y.length)*.5;
        // Longitudinal recirculation/dispersion, in mm²/s. Geometry fixes the
        // travel distance, so filling cannot jump directly from neck to sac.
        links.push({a,b,q:120*Math.min(x.volume,y.volume)/Math.max(1,length*length)});
    };
    for(const [edgeIndex,indices] of byEdge) {
        const edge=network.edges[edgeIndex];
        for(let i=0;i<indices.length-1;i++)connect(indices[i],indices[i+1]);
        const last=indices.at(-1),children=network._preGraftGeometry[edgeIndex].children;
        for(let i=0;i<indices.length-1;i++)if(indices[i]>=0&&indices[i+1]<0)
            drains.push({a:indices[i],edgeIndex,cellIndex:i+1,weight:edge.areas[i+1]});
        if(last<0)continue;
        if(!children.length)drains.push({a:last,edgeIndex:-1,weight:edge.areas.at(-1)});
        for(const child of children) {
            const next=byEdge.get(child)?.[0]??-1;
            if(next>=0)connect(last,next);
            else if(!network.edges[child].transportExcluded)drains.push({a:last,edgeIndex:child,weight:network.edges[child].areas[0]});
        }
    }
    // Match the open gate to a cell in its own fabric part, not the ipsilateral
    // limb that can lie closer to the original anatomical centreline.
    const feeds=[];
    for(const gate of gates) {
        let donor=null,best=Infinity;
        for(const edge of network.edges)for(let i=0;i<edge.cellCount;i++) {
            const section=edge.graftSections?.[i];
            if(section?.part!==gate.part)continue;
            const distance=section.center.distanceToSquared(gate.center);
            if(distance<best){best=distance;donor={edgeIndex:edge.index,cellIndex:i};}
        }
        if(!donor)continue;
        // Two closely spaced outlets can map to the same finite-volume face.
        if(feeds.some(f=>f.donor.edgeIndex===donor.edgeIndex&&f.donor.cellIndex===donor.cellIndex))continue;
        let inlet=byEdge.get(donor.edgeIndex)?.[donor.cellIndex]??-1;
        if(inlet<0) {
            best=Infinity;
            for(let i=0;i<cells.length;i++) {
                const c=cells[i],e=network.edges[c.edgeIndex];
                if(!e.graftCovered)continue;
                const t=(c.cellIndex+.5)/e.cellCount;
                const distance=e.start.clone().lerp(e.end,t).distanceToSquared(gate.center);
                if(distance<best){best=distance;inlet=i;}
            }
        }
        if(inlet<0)continue;
        const edge=network.edges[donor.edgeIndex];
        feeds.push({donor,inlet,direction:edge.axis.dot(gate.normal)>=0?1:-1,fraction:1,q:Math.abs(edge.meanFlowMm3PerS)});
    }
    if(!feeds.length)return null;
    for(const link of links) {
        link.effectiveVolume=1/(1/cells[link.a].volume+1/cells[link.b].volume);
        link.decayRate=link.q/link.effectiveVolume;
    }
    const exchange=(link,dt)=>{
        const a=cells[link.a],b=cells[link.b];
        const difference=a.entry.mass[a.cellIndex]/a.volume-b.entry.mass[b.cellIndex]/b.volume;
        const amount=difference*link.effectiveVolume*(-Math.expm1(-link.decayRate*dt));
        a.entry.mass[a.cellIndex]-=amount;b.entry.mass[b.cellIndex]+=amount;
    };
    const throughflow=createSacThroughflow(cells,links,drains,feeds,(d,amount)=>{
        remodeling.trappedIodineMassMg-=amount;transport.drainedMassMg+=amount;
        if(d.edgeIndex<0)network.outletIodineMassMg+=amount;
        else network.depositIodine(d.edgeIndex,d.cellIndex??0,amount);
    });
    const transport={cells,byEdge,...feeds[0],feeds,receivedMassMg:0,drainedMassMg:0,
        divert(edgeIndex,cellIndex,sign,mass) {
            const feed=feeds.find(f=>edgeIndex===f.donor.edgeIndex&&cellIndex===f.donor.cellIndex&&sign===f.direction);
            if(!feed)return 0;
            const amount=mass*feed.fraction,c=cells[feed.inlet];c.entry.mass[c.cellIndex]+=amount;
            remodeling.trappedIodineMassMg+=amount;this.receivedMassMg+=amount;return amount;
        },
        update(dt) {
            if(!(remodeling.trappedIodineMassMg>1e-12))return;
            // Exact two-volume exchange stays positive even for tiny atlas
            // cells. Symmetric sweeps avoid directional bias and the thousands
            // of explicit substeps a global minimum-volume CFL would require.
            for(const link of links)exchange(link,dt*.5);
            throughflow.update(dt);
            for(let i=links.length-1;i>=0;i--)exchange(links[i],dt*.5);
        }
    };
    return transport;
}
