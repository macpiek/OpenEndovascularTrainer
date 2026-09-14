// Geometry is immutable for a pose token. It contains no reactions: multiplier
// updates at an unchanged pose must still assemble fresh forces and tangents.
const poseGeometry = new WeakMap(), positionMasks = new WeakMap();

export function sharedAxisPositionDofMask(layout) {
    let mask=positionMasks.get(layout);
    if(!mask) {
        mask=new Uint8Array(layout.dofCount);
        for(const p of layout.positions)mask[p]=mask[p+1]=mask[p+2]=1;
        positionMasks.set(layout,mask);
    }
    return mask;
}

export function sharedAxisConstraintEdgeGeometry(s) {
    let cached=poseGeometry.get(s);
    if(s.geometryKey!==undefined&&cached?.key===s.geometryKey&&cached.positions===s.positions)return cached;
    const count=s.positions.length-1,lengths=new Float64Array(count),jacobians=new Array(count);
    for(let e=0;e<count;e++) {
        const a=s.positions[e],b=s.positions[e+1],x=b[0]-a[0],y=b[1]-a[1],z=b[2]-a[2];
        // Match Vector3.distanceTo/sub/multiplyScalar operation order exactly.
        const length=Math.sqrt(x*x+y*y+z*z),inverse=1/length,tx=x*inverse,ty=y*inverse,tz=z*inverse;
        lengths[e]=length;jacobians[e]=[-tx,-ty,-tz,tx,ty,tz];
    }
    cached={key:s.geometryKey,positions:s.positions,lengths,jacobians};
    poseGeometry.set(s,cached);return cached;
}

/** Assemble the existing physical rows without repeated vector construction.
 * The owner lookup is injected to avoid a cycle with Native. Returned rows and
 * reaction-dependent Hessians are fresh, so saved Newton measures remain valid.
 * Pose geometry is shared read-only; callers must not mutate row Jacobians.
 */
export function assembleSharedAxisConstraintRows(s,{withTangent=true,outerMaterialAt,retainWallHessians=false}={}) {
    const g=s.chain.gradient,geometry=sharedAxisConstraintEdgeGeometry(s),rows=new Array(s.definitions.length);
    for(let index=0;index<s.definitions.length;index++) {
        const def=s.definitions[index],e=def.edge,a=s.positions[e],b=s.positions[e+1],length=geometry.lengths[e];
        let gap,J,gapHessian=null;
        if(def.kind==='length') {
            gap=length-(s.coordinates[e+1]-s.coordinates[e]);J=geometry.jacobians[e];
        } else {
            const owner=outerMaterialAt(s,e,def.witness?.t??1,def.witness?.owner),needHessian=(withTangent||retainWallHessians)&&s.multipliers[index]!==0;
            const cache=(s.cacheMechanicalAssembly||retainWallHessians)?(s.wallGeometryCache??=new Map()):null,cached=cache?.get(def);
            let contact;
            if(cached?.key===s.geometryKey&&(!needHessian||cached.withHessian)) {
                contact=cached.contact;s.wallGeometryCacheHits=(s.wallGeometryCacheHits??0)+1;
            } else {
                const evaluate=def.evaluate??s.wallSamples[def.sample];
                contact=evaluate({state:s,a,b,edge:e,radius:owner.body.radius,owner:owner.spec.id,needHessian,sampleT:def.sampleT,
                    coordinateA:s.coordinates[e],coordinateB:s.coordinates[e+1]});
                if(cache&&contact) {
                    // Native evaluators return owned immutable-for-the-caller
                    // output. Custom evaluators may reuse scratch and still
                    // require a defensive copy before another call overwrites it.
                    if(!evaluate.contactOutputOwned)contact={...contact,jacobian:contact.jacobian?.slice(),hessian:contact.hessian?.slice()};
                    cache.set(def,{key:s.geometryKey,withHessian:needHessian,contact});
                }
            }
            if(!contact||!Number.isFinite(contact.gap)||contact.jacobian?.length!==def.dofs.length||!contact.jacobian.every(Number.isFinite))
                throw new RangeError('Wall sample must supply its signed gap and exact edge Jacobian');
            gap=contact.gap;J=contact.jacobian;gapHessian=contact.hessian??null;
        }
        const multiplier=s.multipliers[index],sign=def.kind==='wall'?-1:1;
        for(let i=0;i<def.dofs.length;i++)g[def.dofs[i]]+=sign*J[i]*multiplier;
        let geometricHessian;
        if(gapHessian&&multiplier!==0&&withTangent) {
            geometricHessian=new Float64Array(gapHessian.length);
            for(let i=0;i<gapHessian.length;i++)geometricHessian[i]=-multiplier*gapHessian[i];
        }
        if(withTangent&&def.kind==='length'&&multiplier!==0) {
            geometricHessian=new Float64Array(36);
            const scale=multiplier/length;
            for(let i=0;i<6;i++)for(let j=0;j<6;j++)
                geometricHessian[i*6+j]=scale*(i<3?-1:1)*(j<3?-1:1)*
                    ((i%3===j%3?1:0)-J[3+i%3]*J[3+j%3]);
        }
        rows[index]={...def,gap,jacobian:J,multiplier,penalty:1,geometricHessian};
    }
    return rows;
}
