import {clearDeferredSharedAxisContact,releaseDeferredSharedAxisContact,deferSharedAxisContact,materializeSharedAxisContacts} from './kirchhoffSharedAxisInactiveContacts.js';
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

export function sharedAxisConstraintEdgeGeometry(s,storage=null) {
    storage=storage?.geometryCache??storage;
    let cached=storage?storage.geometry:poseGeometry.get(s);
    if(s.geometryKey!==undefined&&cached?.key===s.geometryKey&&cached.positions===s.positions)return cached;
    const count=s.positions.length-1;
    if(!storage||!cached||cached.lengths.length!==count)cached={lengths:new Float64Array(count),jacobians:Array.from({length:count},()=>new Array(6))};
    const {lengths,jacobians}=cached;
    for(let e=0;e<count;e++) {
        const a=s.positions[e],b=s.positions[e+1],x=b[0]-a[0],y=b[1]-a[1],z=b[2]-a[2];
        // Match Vector3.distanceTo/sub/multiplyScalar operation order exactly.
        const length=Math.sqrt(x*x+y*y+z*z),inverse=1/length,tx=x*inverse,ty=y*inverse,tz=z*inverse;
        lengths[e]=length;
        const J=jacobians[e];J[0]=-tx;J[1]=-ty;J[2]=-tz;J[3]=tx;J[4]=ty;J[5]=tz;
    }
    cached.key=s.geometryKey;cached.positions=s.positions;
    if(storage)storage.geometry=cached;else poseGeometry.set(s,cached);return cached;
}

// A bank is writable only while no live Newton measure refers to its rows.
// Three banks cover base, uncorrected trial and corrected trial. Numeric rows
// are private to one nonlinear call. Contact scratch may span friction solves
// within a timestep: each bank copies its derivatives before retaining a row.
export function createSharedAxisConstraintRowPool(contacts=new Map()) {
    const banks=[],geometryCache={geometry:null,contactOutputs:[],contactOutputOwners:[]};
    return {banks,acquire(protectedRows=[]) {
        let bank=banks.find(b=>!protectedRows.includes(b.rows));
        if(!bank){bank={rows:[],rowBuffers:[],definitions:[],hessians:[],contacts,geometryCache};banks.push(bank);}
        return bank;
    },reset() {
        // Keep only numeric scratch between timesteps. No accepted state,
        // evaluator, geometry owner or deferred pose may survive the lease.
        for(const bank of banks) {
            for(let i=0;i<bank.rowBuffers.length;i++) {
                const row=bank.rowBuffers[i],def=bank.definitions[i];
                if(!row)continue;
                if(def)for(const key of Object.keys(def))row[key]=undefined;
                row.compliance=undefined;row.geometricHessian=undefined;
                row.extraForceDofs=undefined;row.extraForceJacobian=undefined;
                releaseDeferredSharedAxisContact(row);
            }
            bank.definitions.fill(undefined);bank.rows.length=0;
            if(bank.preparedContacts)bank.preparedContacts.length=0;
        }
        contacts.clear();
        geometryCache.contactOutputOwners.fill(undefined);
        if(geometryCache.geometry){geometryCache.geometry.key=null;geometryCache.geometry.positions=null;}
        geometryCache.contactPose=null;
    }};
}

let spareRowPool=null;
// A nonlinear generator can yield while another simulation runs. Lease the
// entire bank set until its timestep publishes exact contacts or rolls back.
// Keep at most one idle arena; overlapping solvers never share live rows.
export function leaseSharedAxisConstraintRowPool() {
    const pool=spareRowPool??createSharedAxisConstraintRowPool();spareRowPool=null;
    let released=false;
    return {pool,release() {
        if(released)return;released=true;pool.reset();
        spareRowPool??=pool;
    }};
}
export function snapshotSharedAxisConstraintMeasure(measure) {
    materializeSharedAxisContacts(measure.rows);
    return {...measure,rows:measure.rows.map(r=>({...r,jacobian:r.jacobian.slice(),geometricHessian:r.geometricHessian?.slice(),
        ...(r.extraForceDofs?{extraForceDofs:r.extraForceDofs.slice(),extraForceJacobian:r.extraForceJacobian.slice()}: {})}))};
}

function hessianBuffer(storage,index,count) {
    if(!storage)return new Float64Array(count);
    if(storage.hessians[index]?.length!==count)storage.hessians[index]=new Float64Array(count);
    return storage.hessians[index];
}

function finiteContact(contact,count) {
    const jacobian=contact?.jacobian;
    if(!Number.isFinite(contact?.gap)||jacobian?.length!==count)return false;
    // Avoid one callback invocation per coefficient in every Newton measure.
    // Reading every entry also rejects sparse arrays instead of letting
    // Array.every skip a missing derivative.
    for(let i=0;i<count;i++)if(!Number.isFinite(jacobian[i]))return false;
    return true;
}

/** Assemble the existing physical rows without repeated vector construction.
 * The owner lookup is injected to avoid a cycle with Native. Returned rows and
 * reaction-dependent Hessians are fresh unless the caller supplies a private
 * bank. Bank owners must protect every still-live Newton measure from reuse.
 * Default pose geometry is shared read-only; callers must not mutate it.
 * Native's private prepareOnly pass requires an owning contact cache and no
 * contact deferral. Its prepared results are consumed in the same synchronous
 * assembly, before any pose/reaction changes or cooperative yield.
 */
export function assembleSharedAxisConstraintRows(s,{withTangent=true,outerMaterialAt,retainWallHessians=false,reuseConstraintWork=false,storage=null,cullInactiveContacts=false,prepareOnly=null,prepared=null}={}) {
    if(prepared?.error&&prepared.index===undefined)throw prepared.error;
    const g=s.chain.gradient,geometry=sharedAxisConstraintEdgeGeometry(s,storage),rows=prepareOnly?null:(storage?.rows??new Array(s.definitions.length));
    if(rows)rows.length=s.definitions.length;
    for(let index=0;index<s.definitions.length;index++) {
        if(prepareOnly)prepareOnly.index=index;
        if(prepared?.error&&prepared.index===index)throw prepared.error;
        const def=s.definitions[index],e=def.edge,a=s.positions[e],b=s.positions[e+1],length=geometry.lengths[e];
        let gap,J,gapHessian=null,row;
        if(storage&&!prepareOnly) {
            row=rows[index]??storage.rowBuffers?.[index];
            if(storage.definitions[index]!==def) {
                const jacobian=row?.jacobian?.length===def.dofs.length?row.jacobian:new Array(def.dofs.length);
                const previous=storage.definitions[index];
                if(previous&&row)for(const key of Object.keys(previous))row[key]=undefined;
                storage.definitions[index]=def;
                // Keep hot numeric fields in a stable object layout across
                // length, sheath, bend and vessel rows. Optional metadata
                // should not change their layout in every active-set scan.
                row??={kind:undefined,edge:undefined,dofs:undefined,sample:undefined,sampleT:undefined,
                    id:undefined,subtype:undefined,witness:undefined,evaluate:undefined,compliance:undefined,
                    gap:0,jacobian,multiplier:0,penalty:1,geometricHessian:undefined,
                    extraForceDofs:undefined,extraForceJacobian:undefined};
                Object.assign(row,def);
                row.gap=0;row.jacobian=jacobian;row.multiplier=0;row.penalty=1;
                rows[index]=row;
                if(storage.rowBuffers)storage.rowBuffers[index]=row;
            }
            // No force column or Hessian from the preceding use may survive.
            clearDeferredSharedAxisContact(row);
            if(s.wallCompliance&&def.witness)row.compliance=s.wallCompliance;
            else row.compliance=undefined;
            row.geometricHessian=undefined;row.extraForceDofs=undefined;row.extraForceJacobian=undefined;
        }

        if(def.kind==='length') {
            gap=length-(s.coordinates[e+1]-s.coordinates[e]);J=geometry.jacobians[e];
        } else if(prepared?.contacts) {
            const contact=prepared.contacts[index];
            gap=contact.gap;J=contact.jacobian;gapHessian=contact.hessian??null;
        } else {
            const owner=outerMaterialAt(s,e,def.witness?.t??1,def.witness?.owner),needHessian=(withTangent||retainWallHessians)&&(s.multipliers[index]!==0||(s.primalCompliantContacts&&def.witness));
            const cache=(s.cacheMechanicalAssembly||retainWallHessians)?(storage?.contacts??(s.wallGeometryCache??=new Map())):null,cached=cache?.get(def);
            // A GN assembly without geometry caching must not overwrite the
            // retained Newton contact/Hessian needed if that method is retried.
            let record=storage&&cache?cache.get(def):null;
            if(storage&&cache&&!record){record={key:null,withHessian:false,contact:null,output:null};cache.set(def,record);}
            let contact;
            if(cached?.key===s.geometryKey&&(!needHessian||cached.withHessian)) {
                contact=cached.contact;s.wallGeometryCacheHits=(s.wallGeometryCacheHits??0)+1;
            } else {
                const evaluate=def.evaluate??s.wallSamples[def.sample];
                if(cullInactiveContacts&&storage&&s.geometryKey!==undefined&&s.multipliers[index]===0&&evaluate.inactiveClearance&&def.witness&&def.dofs.length===6) {
                    const input={state:s,a,b,radius:owner.body.radius};
                    const lowerGap=record?.lowerKey===s.geometryKey?record.lowerGap:evaluate.inactiveClearance(input);
                    if(record){record.lowerKey=s.geometryKey;record.lowerGap=lowerGap;}
                    if(lowerGap>1e-8&&Number.isFinite(lowerGap)) {
                        const holder=storage.geometryCache;
                        if(holder.contactPose?.key!==s.geometryKey)holder.contactPose={key:s.geometryKey,positions:s.positions.map(p=>p.slice()),origin:(s.origin??[0,0,0]).slice()};
                        row.multiplier=0;deferSharedAxisContact(row,evaluate,input,lowerGap,record,holder.contactPose);
                        continue;
                    }
                }
                if(record)record.key=null; // A throwing evaluation cannot leave a valid cache key.
                let contactStorage;
                if(record&&evaluate.contactStorageSupported) {
                    if(!record.output) {
                        // Only explicitly numeric native scratch can survive
                        // a lease. Custom evaluators may store arbitrary owners
                        // and retain their original per-step lifetime.
                        const outputs=evaluate.contactStorageReusable?storage?.geometryCache?.contactOutputs:null;
                        const owners=outputs?storage.geometryCache.contactOutputOwners:null;
                        // A caller may use a bank for different definitions
                        // before resetting the pool. Never overwrite another
                        // still-live record's geometry at the same row index.
                        const reusable=outputs&&(!owners[index]||owners[index]===def);
                        let output=reusable?outputs[index]:undefined;
                        if(output?.contact.jacobian.length!==def.dofs.length)
                            output={contact:{gap:NaN,jacobian:new Array(def.dofs.length),hessian:undefined},hessian:null};
                        if(reusable){outputs[index]=output;owners[index]=def;}
                        record.output=output;
                    }
                    contactStorage=record.output;
                }
                contact=evaluate({state:s,a,b,edge:e,radius:owner.body.radius,owner:owner.spec.id,needHessian,reuseGeometry:reuseConstraintWork,sampleT:def.sampleT,
                    coordinateA:s.coordinates[e],coordinateB:s.coordinates[e+1],contactStorage});
                if(cache&&contact) {
                    // Native evaluators return owned immutable-for-the-caller
                    // output. Custom evaluators may reuse scratch and still
                    // require a defensive copy before another call overwrites it.
                    if(!contactStorage&&!evaluate.contactOutputOwned)contact={...contact,jacobian:contact.jacobian?.slice(),hessian:contact.hessian?.slice()};
                    if(record){record.key=s.geometryKey;record.withHessian=needHessian;record.contact=contact;}
                    else cache.set(def,{key:s.geometryKey,withHessian:needHessian,contact});
                }
            }
            if(!finiteContact(contact,def.dofs.length)) {
                if(record)record.key=null;
                throw new RangeError('Wall sample must supply its signed gap and exact edge Jacobian');
            }
            if(prepareOnly)prepareOnly.contacts[index]=contact;
            gap=contact.gap;J=contact.jacobian;gapHessian=contact.hessian??null;
        }
        if(prepareOnly)continue;
        const multiplier=s.multipliers[index],sign=def.kind==='wall'?-1:1;
        if(multiplier!==0)for(let i=0;i<def.dofs.length;i++)g[def.dofs[i]]+=sign*J[i]*multiplier;
        let geometricHessian;
        if(gapHessian&&multiplier!==0&&withTangent) {
            geometricHessian=hessianBuffer(storage,index,gapHessian.length);
            for(let i=0;i<gapHessian.length;i++)geometricHessian[i]=-multiplier*gapHessian[i];
        }
        if(withTangent&&def.kind==='length'&&multiplier!==0) {
            geometricHessian=hessianBuffer(storage,index,36);
            const scale=multiplier/length;
            // The two endpoints contribute [P,-P;-P,P]. Negating the
            // completed product preserves the scalar sign/rounding (also
            // signed zero), while evaluating each 3x3 entry only once.
            for(let i=0;i<3;i++)for(let j=0;j<3;j++) {
                const at=i*6+j,value=scale*((i===j?1:0)-J[3+i]*J[3+j]);
                geometricHessian[at]=geometricHessian[at+21]=value;
                geometricHessian[at+3]=geometricHessian[at+18]=-value;
            }
        }
        if(storage) {
            row.gap=gap;row.multiplier=multiplier;row.geometricHessian=geometricHessian;
            for(let i=0;i<J.length;i++)row.jacobian[i]=J[i];
        } else {
            rows[index]={...def,gap,jacobian:J,multiplier,penalty:1,geometricHessian};
            if(s.wallCompliance&&def.witness)rows[index].compliance=s.wallCompliance;
        }
    }
    return rows;
}
