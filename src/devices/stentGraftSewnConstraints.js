import {ringWave} from './stentGraftRingKinematics.js';

// Material attachments never slide along a stent band. Resolve excessive crown
// separation in the cloth before drawing an inextensible arm between them.
export function sewnFabricProjector(part,force=false) {
    const p=part.mesh.geometry.attributes.position,a=p.array,rows=part.path.coordinates,sides=part.sides;
    if(!part.target||!part.exposure||part.scaffoldRings.some(r=>!r.sewnReady))return;
    if(!force&&part.sewnConstraintAppliedVersion===p.version)return;
    if(!force&&part.exposure.every(value=>value<=0)){part.sewnConstraintAppliedVersion=p.version;return;}
    for(const ring of part.scaffoldRings) {
        const low=ring.center-ring.maxHeight/2,high=ring.center+ring.maxHeight/2;
        ring.packed=!part.path.coordinates.some((s,i)=>s>=low-2&&s<=high+2&&part.exposure[i]>0);
    }
    const weights=Uint8Array.from({length:p.count},(_,i)=>part.exposure[Math.floor(i/sides)]>0?1:0);
    const movable=i=>weights[i];
    const key=part.scaffoldRings.map(r=>r.attachmentHeight).join();
    if(part.sewnConstraintKey!==key) {
        part.sewnConstraintKey=key;
        const anchor=(s,j)=>{
            let row=1;while(row<rows.length-1&&rows[row]<s)row++;
            const t=Math.max(0,Math.min(1,(s-rows[row-1])/(rows[row]-rows[row-1])));
            return [[(row-1)*sides+j%sides,1-t],[row*sides+j%sides,t]];
        };
        // Rest lengths belong to the manufactured cloth, not to wall fitting
        // or to the current deployment target. Multi-scale chords propagate
        // tension along a long leg without hundreds of local-only sweeps.
        const metric=part.fabricMaterialCoordinates??=Float64Array.from(rows,(_,r)=>{
            let length=0;
            for(let k=1;k<=r;k++)length+=Math.hypot(rows[k]-rows[k-1],
                (part.rowRadii?.[k]??part.radius)-(part.rowRadii?.[k-1]??part.radius));
            return length;
        });
        const cloth=[];
        for(let stride=1;stride<part.rows;stride*=2)for(let row=0;row<part.rows-stride;row+=stride)for(let j=0;j<sides;j++) {
            const i=row*sides+j,k=(row+stride)*sides+j,length=metric[row+stride]-metric[row];
            cloth.push({i:i*3,k:k*3,length,limitSq:(length+1e-7)**2});
        }
        // Reserve two percent for the rounded bends between the sewn crowns.
        // The renderer restores the full wire length; this is not metal strain.
        const wire=part.scaffoldRings.map(ring=>({ring,length:ring.restLength*.98,
            anchors:Array.from({length:sides},(_,j)=>anchor(ring.center+ring.attachmentHeight*ringWave(j,ring.proximal),j))}));
        for(const c of wire) {
            c.indices=[...new Set([...Array.from({length:sides},(_,j)=>j),...c.anchors.flatMap(a=>a.map(([i])=>i))])];
            c.points=new Float64Array(sides*3);
            c.anchorIndices=new Int32Array(sides*2);
            c.anchorWeights=new Float64Array(sides*2);
            for(let j=0;j<sides;j++)for(let k=0;k<2;k++) {
                c.anchorIndices[j*2+k]=c.anchors[j][k][0]*3;
                c.anchorWeights[j*2+k]=c.anchors[j][k][1];
            }
        }
        part.sewnConstraints={cloth,wire};
    }
    const pinnedCenter=!part.sewnParent&&!part.sewnCapture&&part.scaffoldRings[0].opening<1?Array.from({length:3},(_,c)=>{
        let sum=0;for(let j=0;j<sides;j++)sum+=a[j*3+c];return sum/sides;
    }):null;
    let changed=false;
    const activeCloth=part.sewnConstraints.cloth.filter(c=>weights[c.i/3]||weights[c.k/3]);
    const activeWire=part.sewnConstraints.wire.filter(c=>!c.ring.packed);
    const project=c=>{
        const {i,k}=c,wi=weights[i/3],wk=weights[k/3],w=wi+wk;
        const dx=a[k]-a[i],dy=a[k+1]-a[i+1],dz=a[k+2]-a[i+2];
        const squared=dx*dx+dy*dy+dz*dz;if(squared<=c.limitSq)return 0;
        const length=Math.sqrt(squared),error=length-c.length;
        const f=error/(length*w);
        if(wi){a[i]+=f*dx;a[i+1]+=f*dy;a[i+2]+=f*dz;}
        if(wk){a[k]-=f*dx;a[k+1]-=f*dy;a[k+2]-=f*dz;}
        changed=true;return error;
    };
    const gradient=part.sewnGradient??=new Float64Array(a.length);
    const projectRing=c=>{
        if(c.ring.packed)return 0;
        for(const i of c.indices){gradient[i*3]=0;gradient[i*3+1]=0;gradient[i*3+2]=0;}
        const points=c.points,ai=c.anchorIndices,aw=c.anchorWeights;
        for(let j=0;j<sides;j++) {
            const i=ai[j*2],k=ai[j*2+1],u=aw[j*2],v=aw[j*2+1],p=j*3;
            points[p]=a[i]*u+a[k]*v;points[p+1]=a[i+1]*u+a[k+1]*v;points[p+2]=a[i+2]*u+a[k+2]*v;
        }
        let length=0;
        for(let j=0;j<sides;j++) {
            const next=(j+1)%sides,p=j*3,q=next*3;
            let x=points[q]-points[p],y=points[q+1]-points[p+1],z=points[q+2]-points[p+2];
            const distance=Math.sqrt(x*x+y*y+z*z);length+=distance;if(distance<1e-12)continue;
            x/=distance;y/=distance;z/=distance;
            for(let k=0;k<2;k++) {
                const i=ai[j*2+k],u=aw[j*2+k],n=ai[next*2+k],v=aw[next*2+k];
                gradient[i]-=u*x;gradient[i+1]-=u*y;gradient[i+2]-=u*z;
                gradient[n]+=v*x;gradient[n+1]+=v*y;gradient[n+2]+=v*z;
            }
        }
        const error=length-c.length;if(error<=1e-7)return 0;
        if(pinnedCenter)for(let k=0;k<3;k++) {
            let mean=0;for(let j=0;j<sides;j++)mean+=gradient[j*3+k]/sides;
            for(let j=0;j<sides;j++)gradient[j*3+k]-=mean;
        }
        let w=0;for(const i of c.indices)if(movable(i))w+=gradient[i*3]**2+gradient[i*3+1]**2+gradient[i*3+2]**2;
        if(w>1e-12) {
            const correction=error/w;
            for(const i of c.indices)if(movable(i)) {
                a[i*3]-=correction*gradient[i*3];a[i*3+1]-=correction*gradient[i*3+1];a[i*3+2]-=correction*gradient[i*3+2];
            }
            changed=true;
        }
        return error;
    };
    return {step() {
        let error=0;
        for(const c of activeCloth)error=Math.max(error,project(c));
        if(pinnedCenter)for(let c=0;c<3;c++) {
            let center=0;for(let j=0;j<sides;j++)center+=a[j*3+c]/sides;
            const shift=pinnedCenter[c]-center;
            for(let j=0;j<sides;j++)a[j*3+c]+=shift;
        }
        for(const c of activeWire)error=Math.max(error,projectRing(c));
        if(part.sewnCapture) {
            const {latch,material}=part.sewnCapture;
            for(let j=0;j<sides;j++) {
                const x=a[j*3]-latch.x,y=a[j*3+1]-latch.y,z=a[j*3+2]-latch.z;
                const length=Math.sqrt(x*x+y*y+z*z),scale=material.armLength/Math.max(1e-9,length);
                error=Math.max(error,Math.abs(length-material.armLength));
                a[j*3]=latch.x+x*scale;a[j*3+1]=latch.y+y*scale;a[j*3+2]=latch.z+z*scale;
            }
        }
        // Below Float32 precision at anatomical coordinates, further passes
        // cannot improve the geometry and only consume the frame budget.
        return error;
    },finish() {
        if(changed){p.needsUpdate=true;if(part.contactBasePositions)part.contactBasePositions.set(a);}
        part.sewnConstraintAppliedVersion=p.version;
    }};
}
export function constrainSewnFabric(part) {
    const projection=sewnFabricProjector(part);if(!projection)return;
    for(let pass=0;pass<100;pass++)if(projection.step()<1e-4)break;
    projection.finish();
}

