// Removing a spatial knot must not combine admissible bends into a forbidden
// bend before Newton starts. Retain only the old knots needed to repair that
// seed; feed reconsiders them rather than promoting permanent boundaries.
export function preserveSharedAxisBends(coordinates,positions,previous,maxBendAngle) {
    if(!previous||!Number.isFinite(maxBendAngle))return;
    const cosine=Math.cos(maxBendAngle);
    const violation=i=>{
        const a=positions[i-1],b=positions[i],c=positions[i+1];
        const ux=b[0]-a[0],uy=b[1]-a[1],uz=b[2]-a[2],vx=c[0]-b[0],vy=c[1]-b[1],vz=c[2]-b[2];
        const dot=(ux*vx+uy*vy+uz*vz)/(Math.hypot(ux,uy,uz)*Math.hypot(vx,vy,vz));
        return Math.max(0,cosine-dot-1e-12);
    };
    const score=()=>{let sum=0;for(let i=1;i+1<positions.length;i++){const v=violation(i);sum+=v*v;}return sum;};
    for(let pass=0;pass<previous.coordinates.length;pass++) {
        const candidates=new Set();
        for(let i=1;i+1<positions.length;i++) {
            if(!(violation(i)>0))continue;
            for(let j=1;j+1<previous.coordinates.length;j++) {
                const x=previous.coordinates[j];
                if(x>coordinates[i-1]&&x<coordinates[i+1]&&!coordinates.some(c=>Math.abs(c-x)<1e-9))candidates.add(j);
            }
        }
        if(!candidates.size)return;
        // Reinsert one knot at a time. Restoring every old sample in a bend
        // would retain a trail of short cells left by a moving material tip.
        let best=-1,bestScore=Infinity;
        for(const j of [...candidates].sort((a,b)=>a-b)) {
            const x=previous.coordinates[j],at=coordinates.findIndex(c=>c>x);
            coordinates.splice(at,0,x);positions.splice(at,0,previous.positions[j]);
            const next=score();coordinates.splice(at,1);positions.splice(at,1);
            if(next<bestScore){best=j;bestScore=next;}
        }
        if(best<0)return;
        const x=previous.coordinates[best],at=coordinates.findIndex(c=>c>x);
        coordinates.splice(at,0,x);positions.splice(at,0,previous.positions[best].slice());
    }
}
