import {Vector2,Vector3} from 'three';

// One pair-of-pants surface. Split the outlet polygon, insert every material
// sample on its edges, and reuse those vertex IDs in all three strips. A
// positional constraint alone cannot sew polygons with different sampling.
export function sewnTopology(device) {
    if(device.sewnTopology)return device.sewnTopology;
    const [trunk,...legs]=device.parts,sides=trunk.sides,last=trunk.rows-1;
    const center=trunk.points[last],frame=trunk.ringFrames[last];
    const lateral=legs[1].points[0].clone().sub(legs[0].points[0]);
    lateral.addScaledVector(frame.tangent,-lateral.dot(frame.tangent)).normalize();
    if(lateral.lengthSq()<1e-12)lateral.copy(frame.u);
    const up=frame.tangent.clone().cross(lateral).normalize();
    const vertices=[],joint=[],registry=new Map();
    const binding=(part,entries)=>({part,indices:entries.map(([i])=>i),weights:entries.map(([,w])=>w)});
    const add=(part,entries,xy=null)=>{
        const sums=new Map();for(const [i,w]of entries)sums.set(i,(sums.get(i)??0)+w);
        entries=[...sums].filter(([,w])=>Math.abs(w)>1e-10).sort((a,b)=>a[0]-b[0]);
        const key=device.parts.indexOf(part)+':'+entries.map(([i,w])=>i+'@'+Math.round(w*1e8)).join(',');
        let id=registry.get(key);
        if(id===undefined){id=vertices.length;vertices.push(binding(part,entries));registry.set(key,id);}
        if(xy&&!joint.some(p=>p.id===id))joint.push({id,xy});
        return id;
    };
    const entries=id=>vertices[id].indices.map((i,k)=>[i,vertices[id].weights[k]]);
    const mix=(a,b,t)=>add(trunk,[...entries(a.id).map(([i,w])=>[i,w*(1-t)]),...entries(b.id).map(([i,w])=>[i,w*t])],a.xy.clone().lerp(b.xy,t));
    const outer=Array.from({length:sides},(_,j)=>{
        const index=last*sides+j,p=new Vector3().fromArray(trunk.target,index*3).sub(center),xy=new Vector2(p.dot(lateral),p.dot(up));
        return {id:add(trunk,[[index,1]],xy),xy};
    });
    // Wall fitting can collapse neighbouring samples in a deliberately
    // misplaced device. Keep material connectivity well-defined even when
    // the physical section is degenerate; never create undefined triangles.
    const area=outer.reduce((sum,a,j)=>{const b=outer[(j+1)%sides];return sum+a.xy.x*b.xy.y-a.xy.y*b.xy.x;},0);
    const collapsed=Math.abs(area)<1e-6||outer.some((a,j)=>a.xy.distanceToSquared(outer[(j+1)%sides].xy)<1e-12);
    if(collapsed)outer.forEach((p,j)=>{
        const angle=j/sides*Math.PI*2,radial=frame.u.clone().multiplyScalar(Math.cos(angle)).addScaledVector(frame.v,Math.sin(angle));
        p.xy.set(radial.dot(lateral),radial.dot(up));
    });
    const min=Math.min(...outer.map(p=>p.xy.x)),max=Math.max(...outer.map(p=>p.xy.x));
    const r0=legs[0].rowRadii[0],r1=legs[1].rowRadii[0],split=min+(max-min)*r0/(r0+r1);
    const clip=sign=>{
        const out=[];
        for(let j=0;j<sides;j++){
            const a=outer[j],b=outer[(j+1)%sides],insideA=(a.xy.x-split)*sign>=-1e-9,insideB=(b.xy.x-split)*sign>=-1e-9;
            if(insideA)out.push(a);
            if(insideA!==insideB){const t=(split-a.xy.x)/(b.xy.x-a.xy.x),id=mix(a,b,t);out.push({id,xy:a.xy.clone().lerp(b.xy,t)});}
        }
        return out;
    };
    const polygons=[clip(-1),clip(1)];
    const roots=legs.map((part,k)=>Array.from({length:part.sides},(_,j)=>{
        const p=new Vector3().fromArray(part.target,j*3).sub(center),xy=new Vector2(p.dot(lateral),p.dot(up));
        if(collapsed){
            const angle=j/part.sides*Math.PI*2,own=part.ringFrames[0];
            const radial=own.u.clone().multiplyScalar(Math.cos(angle)).addScaledVector(own.v,Math.sin(angle));
            xy.set((k===0?(min+split):(max+split))/2+radial.dot(lateral)*2,radial.dot(up)*2);
        }
        let best=null;
        for(let i=0;i<polygons[k].length;i++){
            const a=polygons[k][i],b=polygons[k][(i+1)%polygons[k].length],delta=b.xy.clone().sub(a.xy);
            const t=Math.max(0,Math.min(1,xy.clone().sub(a.xy).dot(delta)/Math.max(1e-20,delta.lengthSq()))),point=a.xy.clone().addScaledVector(delta,t),distance=xy.distanceToSquared(point);
            if(!best||distance<best.distance)best={a,b,t,distance};
        }
        return mix(best.a,best.b,best.t);
    }));
    const edge=(a,b)=>{
        const delta=b.xy.clone().sub(a.xy),length=delta.lengthSq();
        return joint.map(p=>({p,t:p.xy.clone().sub(a.xy).dot(delta)/length}))
            .filter(({p,t})=>t>=-1e-7&&t<=1+1e-7&&p.xy.distanceToSquared(a.xy.clone().addScaledVector(delta,t))<1e-12)
            .sort((a,b)=>a.t-b.t).map(({p})=>p.id);
    };
    const loops=polygons.map(poly=>poly.flatMap((a,i)=>edge(a,poly[(i+1)%poly.length]).slice(0,-1)));
    const faces=[],ports=[];
    const face=(part,row,a,b,c)=>{if([a,b,c].some(i=>!Number.isInteger(i)))throw new Error('Invalid sewn triangle');if(a!==b&&b!==c&&c!==a)faces.push({part,row,ids:[a,b,c]});};
    const original=(part,i)=>add(part,[[i,1]]);
    const bands=(part,start,end)=>{
        for(let r=start;r<end;r++)for(let j=0;j<part.sides;j++){
            const next=(j+1)%part.sides,a=original(part,r*part.sides+j),b=original(part,r*part.sides+next),c=original(part,(r+1)*part.sides+j),d=original(part,(r+1)*part.sides+next);
            face(part,r+1,a,c,b);face(part,r+1,b,c,d);
        }
    };
    bands(trunk,0,last-1);
    for(let j=0;j<sides;j++){
        const next=(j+1)%sides,a=original(trunk,(last-1)*sides+j),b=original(trunk,(last-1)*sides+next),path=edge(outer[j],outer[next]);
        face(trunk,last,a,path[0],b);
        for(let k=0;k<path.length-1;k++)face(trunk,last,b,path[k],path[k+1]);
    }
    legs.forEach((part,k)=>{
        const loop=loops[k],root=roots[k],n=loop.length;
        const total=root.reduce((sum,id,j)=>sum+(loop.indexOf(root[(j+1)%root.length])-loop.indexOf(id)+n)%n,0),direction=total<=n*1.5?1:-1;
        for(let j=0;j<part.sides;j++){
            const next=(j+1)%part.sides,bottom=original(part,part.sides+j),end=original(part,part.sides+next);
            let i=loop.indexOf(root[j]),stop=loop.indexOf(root[next]);
            if(i<0||stop<0)throw new Error('Missing sewn boundary vertex');
            for(let guard=0;i!==stop&&guard<n;guard++){
                const following=(i+direction+n)%n;face(part,1,loop[i],bottom,loop[following]);i=following;
            }
            face(part,1,loop[stop],bottom,end);
        }
        bands(part,1,part.rows-1);
    });
    ports.push({part:trunk,ids:Array.from({length:sides},(_,j)=>original(trunk,j)),proximal:true});
    legs.forEach(part=>ports.push({part,ids:Array.from({length:part.sides},(_,j)=>original(part,(part.rows-1)*part.sides+j)),proximal:false}));
    const bindings=roots.map(ids=>ids.map(id=>{
        const v=vertices[id],indices=Int32Array.from(v.indices,i=>i*3),weights=Float64Array.from(v.weights);
        return {indices,weights,entries:v.indices.map((i,k)=>[i,v.weights[k]])};
    }));
    return device.sewnTopology={parts:device.parts,vertices,faces,ports,bindings};
}
