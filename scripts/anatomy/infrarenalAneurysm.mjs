// Atlas coordinates in mm. +Y is cranial; +Z is anterior.
export const INFRARENAL_ANEURYSM = Object.freeze({
    id:'infrarenal-aneurysm', targetDiameterMm:50,
    proximalY:-204, distalY:-273.77647399902344,
    sacDistalY:-253.77647399902344, proximalRampMm:16, referenceY:-230,
    distalExpansionScale:.53, distalBlendUpstreamMm:8,
    coreRadius:15, outerRadius:120,
    anteriorFraction:0.8,
    sourceSha256:'298e70f50d52fceb3c25cf13ae8df46ecd92c0bae36cea8fd91be236bc8b0aa2',
    renalOrigins:[[8.83759880065918,-188.27334594726562,-8.769051551818848],
        [7.808897018432617,-180.50462341308594,-10.288677215576172]],
    bifurcation:[-1.6286494731903076,-253.77647399902344,9.944806098937988]
});
const smoothstep = t => t*t*t*(10+t*(-15+6*t));
export function createInfrarenalDeformation({axis, referenceDiameterMm, ...overrides}) {
    const config={...INFRARENAL_ANEURYSM,...overrides};
    const gain=config.targetDiameterMm/referenceDiameterMm-1;
    if (!(referenceDiameterMm>0) || gain<=0) throw Error('Aneurysm must enlarge the baseline lumen');
    // Bound both radial expansion and anterior translation through the falloff.
    const translation=(config.targetDiameterMm-referenceDiameterMm)*.5*Math.abs(config.anteriorFraction);
    if ((gain*config.coreRadius+translation)*1.875/(config.outerRadius-config.coreRadius)>=1)
        throw Error('Aneurysm displacement would fold its surrounding transition');
    const path=axis.slice().sort((a,b)=>a[1]-b[1]);
    const centerAt = y => {
        let i=1;while(i<path.length-1 && path[i][1]<y)i++;
        const a=path[i-1],b=path[i],t=Math.max(0,Math.min(1,(y-a[1])/(b[1]-a[1])));
        return [a[0]+t*(b[0]-a[0]),y,a[2]+t*(b[2]-a[2])];
    };
    function move(x,y,z) {
        if(y<=config.distalY || y>=config.proximalY)return [x,y,z];
        const center=centerAt(y),dx=x-center[0],dz=z-center[2],r=Math.hypot(dx,dz);
        if(r>=config.outerRadius)return [x,y,z];
        // Maintain the sac down to the iliac bifurcation instead of narrowing
        // into a second aortic neck. Blend the junction into both iliac limbs.
        const proximalWeight=smoothstep(Math.min(1,(config.proximalY-y)/config.proximalRampMm));
        const distalWeight=smoothstep(Math.min(1,(y-config.distalY)/(config.sacDistalY-config.distalY)));
        const weight=proximalWeight*distalWeight;
        const falloff=r<=config.coreRadius ? 1 : 1-smoothstep((r-config.coreRadius)/(config.outerRadius-config.coreRadius));
        const distalBlend=smoothstep(Math.max(0,Math.min(1,(y-config.sacDistalY)/config.distalBlendUpstreamMm)));
        // Correct the wider junction locally, without introducing abrupt
        // displacement in the small neighboring branches.
        const junctionFalloff=1-smoothstep(Math.max(0,Math.min(1,(r-config.coreRadius)/15)));
        const localGain=gain*(1-(1-config.distalExpansionScale)*(1-distalBlend)*junctionFalloff);
        const radialGain=localGain*weight*falloff*Math.min(1,config.coreRadius/Math.max(r,1e-9));
        // Prefer anterior expansion so the posterior wall remains near its
        // original position against the spine. No skeleton geometry is moved.
        // Keep the bifurcation center fixed while widening its lumen.
        const anteriorWeight=smoothstep(Math.max(0,Math.min(1,(y-config.sacDistalY)/20)));
        const forward=(config.targetDiameterMm-referenceDiameterMm)*.5*config.anteriorFraction*weight*falloff*anteriorWeight;
        return [x+dx*radialGain,y,z+dz*radialGain+forward];
    }
    return {move,centerAt,config,gain};
}

export function axialSection(positions,y,center,maxRadius=18,selectionPositions=positions) {
    const points=[];
    for(let i=0;i<positions.length;i+=9) for(let j=0;j<3;j++) {
        const a=i+j*3,b=i+((j+1)%3)*3,dy=positions[b+1]-positions[a+1];
        if(Math.abs(dy)<1e-9)continue;
        const t=(y-positions[a+1])/dy;
        if(t<0||t>1)continue;
        const x=positions[a]+t*(positions[b]-positions[a]),z=positions[a+2]+t*(positions[b+2]-positions[a+2]);
        // Use the baseline contour to exclude nearby mesenteric branches,
        // including after their displacement by the expanding sac.
        const sx=selectionPositions[a]+t*(selectionPositions[b]-selectionPositions[a]);
        const sz=selectionPositions[a+2]+t*(selectionPositions[b+2]-selectionPositions[a+2]);
        if(Math.hypot(sx-center[0],sz-center[2])<maxRadius)points.push([x,z]);
    }
    let diameter=0;
    for(let angle=0;angle<Math.PI;angle+=Math.PI/360) {
        const c=Math.cos(angle),s=Math.sin(angle);let lo=Infinity,hi=-Infinity;
        for(const [x,z] of points){const p=x*c+z*s;lo=Math.min(lo,p);hi=Math.max(hi,p);}
        diameter=Math.max(diameter,hi-lo);
    }
    return {points,diameter};
}
