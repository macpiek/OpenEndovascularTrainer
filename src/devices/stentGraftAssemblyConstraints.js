import {junctionBindings} from './stentGraftJunctionBindings.js';
import {constrainSewnFabric,sewnFabricProjector} from './stentGraftSewnConstraints.js';

// Eliminate the limb-mouth degrees of freedom: their material points belong
// to the trunk outlet. Cloth and wire gradients therefore act on the same
// particles, rather than alternating incompatible seam/metal projections.
export function constrainSewnAssembly(device) {
    if(device.type!=='body'||device.parts[0].exposure.at(-1)<=0){for(const p of device.parts)constrainSewnFabric(p);return;}
    const key=device.parts.map(p=>p.mesh.geometry.attributes.position.version).join('/');
    if(device.sewnAssemblyVersion===key)return;
    if(device.parts.some(p=>p.scaffoldRings.some(r=>!r.sewnReady)))return;
    const bindings=device.sewnJunctionBindings??=junctionBindings(device),parts=device.parts;
    parts.forEach(p=>sewnFabricProjector(p,true)); // refresh immutable material constraints
    const offsets=[];let count=0;
    for(const p of parts){offsets.push(count);count+=p.mesh.geometry.attributes.position.count;}
    const a=new Float64Array(count*3),mass=new Uint8Array(count);
    parts.forEach((p,k)=>{a.set(p.mesh.geometry.attributes.position.array,offsets[k]*3);for(let i=0;i<p.rows*p.sides;i++)mass[offsets[k]+i]=p.exposure[Math.floor(i/p.sides)]>0?1:0;});
    const mapping=(k,i)=>k>0&&i<parts[k].sides
        ?bindings[k-1][i].entries
        :[[offsets[k]+i,1]];
    const combine=entries=>{
        const map=new Map();for(const [i,w]of entries)map.set(i,(map.get(i)??0)+w);
        return [...map].filter(([,w])=>Math.abs(w)>1e-12);
    };
    let workspace=device.sewnAssemblyWorkspace;
    if(!workspace||workspace.sources.some((source,i)=>source!==parts[i].sewnConstraints)) {
    const constraints=[],direct=[];
    parts.forEach((p,k)=>{
        const centers=Array.from({length:p.rows},(_,r)=>combine(Array.from({length:p.sides},(_,j)=>mapping(k,r*p.sides+j).map(([i,w])=>[i,w/p.sides])).flat()));
        for(let stride=1;stride<p.rows;stride*=2)for(let r=stride;r<p.rows;r+=stride)constraints.push({length:p.path.coordinates[r]-p.path.coordinates[r-stride],anchors:[centers[r-stride],centers[r]],closed:false});
        for(const c of p.sewnConstraints.cloth) {
            if(k===0||c.i/3>=p.sides)direct.push({i:(offsets[k]*3+c.i),k:(offsets[k]*3+c.k),length:c.length,limitSq:c.limitSq});
            else constraints.push({length:c.length,anchors:[mapping(k,c.i/3),mapping(k,c.k/3)],closed:false});
        }
        for(const c of p.sewnConstraints.wire)constraints.push({length:c.length,closed:true,ring:c.ring,
            anchors:c.anchors.map(anchor=>combine(anchor.flatMap(([i,w])=>mapping(k,i).map(([n,v])=>[n,v*w]))))});
    });
    for(const c of constraints){
        if(!c.closed){
            const delta=combine([...c.anchors[0].map(([i,w])=>[i,-w]),...c.anchors[1]]);
            c.deltaIndices=Int32Array.from(delta,([i])=>i*3);c.deltaWeights=Float64Array.from(delta,([,w])=>w);
        }
        c.indices=Int32Array.from(new Set(c.anchors.flatMap(anchor=>anchor.map(([i])=>i*3))));
        c.points=new Float64Array(c.anchors.length*3);
        c.anchors=c.anchors.map(anchor=>({indices:Int32Array.from(anchor,([i])=>i*3),weights:Float64Array.from(anchor,([,w])=>w)}));
    }
    workspace=device.sewnAssemblyWorkspace={constraints,direct,sources:parts.map(p=>p.sewnConstraints),gradient:new Float64Array(a.length)};
    }
    const active=workspace.constraints.filter(c=>!c.ring?.packed&&c.indices.some(i=>mass[i/3]));
    const links=workspace.direct.filter(c=>mass[c.i/3]||mass[c.k/3]);
    const gradient=workspace.gradient;
    for(const c of active)if(!c.closed)c.denominator=c.deltaWeights.reduce((sum,w,n)=>sum+mass[c.deltaIndices[n]/3]*w*w,0);
    for(let pass=0;pass<100;pass++) {
        let error=0;
        for(const c of links) {
            const {i,k}=c,wi=mass[i/3],wk=mass[k/3],dx=a[k]-a[i],dy=a[k+1]-a[i+1],dz=a[k+2]-a[i+2],squared=dx*dx+dy*dy+dz*dz;
            if(squared<=c.limitSq)continue;
            const length=Math.sqrt(squared),excess=length-c.length,f=excess/(length*(wi+wk));error=Math.max(error,excess);
            if(wi){a[i]+=f*dx;a[i+1]+=f*dy;a[i+2]+=f*dz;}
            if(wk){a[k]-=f*dx;a[k+1]-=f*dy;a[k+2]-=f*dz;}
        }
        for(const c of active) {
            if(!c.closed){
                const indices=c.deltaIndices,weights=c.deltaWeights;let x=0,y=0,z=0;
                for(let n=0;n<indices.length;n++){const i=indices[n],w=weights[n];x+=w*a[i];y+=w*a[i+1];z+=w*a[i+2];}
                const length=Math.sqrt(x*x+y*y+z*z),excess=length-c.length;
                if(excess>1e-7&&c.denominator>1e-12){
                    error=Math.max(error,excess);const scale=excess/(length*c.denominator);
                    for(let n=0;n<indices.length;n++){const i=indices[n],f=scale*weights[n]*mass[i/3];a[i]-=f*x;a[i+1]-=f*y;a[i+2]-=f*z;}
                }
                continue;
            }
            for(const i of c.indices){gradient[i]=0;gradient[i+1]=0;gradient[i+2]=0;}
            const points=c.points;
            for(let j=0;j<c.anchors.length;j++) {
                const {indices,weights}=c.anchors[j];let x=0,y=0,z=0;
                for(let n=0;n<indices.length;n++){const i=indices[n],w=weights[n];x+=a[i]*w;y+=a[i+1]*w;z+=a[i+2]*w;}
                points[j*3]=x;points[j*3+1]=y;points[j*3+2]=z;
            }
            let length=0;
            for(let j=0;j<(c.closed?c.anchors.length:1);j++) {
                const n=(j+1)%c.anchors.length;
                let x=points[n*3]-points[j*3],y=points[n*3+1]-points[j*3+1],z=points[n*3+2]-points[j*3+2];
                const d=Math.sqrt(x*x+y*y+z*z);length+=d;if(d<1e-12)continue;x/=d;y/=d;z/=d;
                const from=c.anchors[j],to=c.anchors[n];
                for(let k=0;k<from.indices.length;k++){const i=from.indices[k],w=from.weights[k];gradient[i]-=w*x;gradient[i+1]-=w*y;gradient[i+2]-=w*z;}
                for(let k=0;k<to.indices.length;k++){const i=to.indices[k],w=to.weights[k];gradient[i]+=w*x;gradient[i+1]+=w*y;gradient[i+2]+=w*z;}
            }
            const excess=length-c.length;if(excess<=1e-7)continue;error=Math.max(error,excess);
            let denominator=0;for(const i of c.indices)if(mass[i/3])denominator+=gradient[i]**2+gradient[i+1]**2+gradient[i+2]**2;
            if(denominator<1e-12)continue;
            const scale=excess/denominator;
            for(const i of c.indices)if(mass[i/3]){a[i]-=scale*gradient[i];a[i+1]-=scale*gradient[i+1];a[i+2]-=scale*gradient[i+2];}
        }
        const capture=parts[0].sewnCapture;
        if(capture)for(let j=0;j<parts[0].sides;j++){
            const {latch,material}=capture,x=a[j*3]-latch.x,y=a[j*3+1]-latch.y,z=a[j*3+2]-latch.z,length=Math.hypot(x,y,z),scale=material.armLength/Math.max(1e-9,length);
            error=Math.max(error,Math.abs(length-material.armLength));a[j*3]=latch.x+x*scale;a[j*3+1]=latch.y+y*scale;a[j*3+2]=latch.z+z*scale;
        }
        if(error<1e-4)break;
    }
    parts.forEach((p,k)=>{
        const positions=p.mesh.geometry.attributes.position;
        for(let i=0;i<positions.count;i++)for(let q=0;q<3;q++){let value=0;for(const [n,w]of mapping(k,i))value+=a[n*3+q]*w;positions.array[i*3+q]=value;}
        positions.needsUpdate=true;if(p.contactBasePositions)p.contactBasePositions.set(positions.array);p.sewnConstraintAppliedVersion=positions.version;
    });
    device.sewnAssemblyVersion=parts.map(p=>p.mesh.geometry.attributes.position.version).join('/');
}
