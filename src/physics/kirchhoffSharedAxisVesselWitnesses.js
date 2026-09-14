import {createSharedAxisDiscoveryCache} from './kirchhoffSharedAxisDiscoveryCache.js';
import { createContactResult } from './collision/vesselContactField.js';
import { createKirchhoffWallWitnessGeometryWorkspace, evaluateKirchhoffWallWitnessGeometry } from './kirchhoffWallWitnessGeometry.js';
import { extendSharedAxisNativeRows, iterateSharedAxisNative, sharedAxisOuterIntervals } from './kirchhoffSharedAxisNative.js';

const NEED_ROWS='shared-axis-wall-discovery';
const outsideError=message=>Object.assign(new Error(message),{code:'trial-outside-vessel'});
function gapForWitness(field,face,t,geometryScratch,{a,b,radius,state,needHessian=true}) {
    const query=geometryScratch.contactQuery;
    const point=query?.point??a.map((v,i)=>(1-t)*v+t*b[i]+(state.origin?.[i]??0));
    if(query) {
        for(let i=0;i<3;i++)point[i]=(1-t)*a[i]+t*b[i]+(state.origin?.[i]??0);
        query.geometry=field.fallbackGeometry;
    }
    const g=evaluateKirchhoffWallWitnessGeometry(query??{geometry:field.fallbackGeometry,faceIndex:face,point},geometryScratch);
    if(!g.normalDefined)throw outsideError('Retained vessel witness reached the surface');
    const n=g.direction,jacobian=query?new Array(6):[...n.map(v=>(1-t)*v),...n.map(v=>t*v)];
    if(query)for(let i=0;i<3;i++){jacobian[i]=(1-t)*n[i];jacobian[i+3]=t*n[i];}
    let hessian;
    if(needHessian&&g.feature!=='face') {
        const edge=geometryScratch.contactEdge??[0,0,0];edge.fill(0);
        if(g.feature==='edge') {
            const first=g.featureMask&1?0:1,second=g.featureMask&4?2:1;
            for(let a=0;a<3;a++)edge[a]=g.triangleVertices[second*3+a]-g.triangleVertices[first*3+a];
            const norm=Math.hypot(...edge);for(let a=0;a<3;a++)edge[a]/=norm;
        }
        hessian=new Float64Array(36);
        for(let i=0;i<6;i++)for(let j=0;j<6;j++)hessian[i*6+j]=(i<3?1-t:t)*(j<3?1-t:t)*
            ((i%3===j%3?1:0)-edge[i%3]*edge[j%3]-n[i%3]*n[j%3])/g.distance;
    }
    return {gap:g.distance-radius,jacobian,hessian};
}

// Keep witness geometry explicit so a failing state can be replayed without
// replaying hundreds of preceding feed steps or serializing closures.
export function createSharedAxisVesselWitness(field, definition,{reuseBuffers=true}={}) {
    const scratch=createKirchhoffWallWitnessGeometryWorkspace();
    const {face,t}=definition.witness;
    if(reuseBuffers){scratch.contactQuery={geometry:null,faceIndex:face,point:[0,0,0]};scratch.contactEdge=[0,0,0];}
    const evaluate=input=>gapForWitness(field,face,t,scratch,input);
    // Only temporary geometry uses scratch. Every returned derivative owns
    // its storage and remains valid across later trials, feed and rollback.
    evaluate.contactOutputOwned=reuseBuffers;
    return {...definition,evaluate};
}

/** Discover with the current mesh/BVH kernel, retain each finite surface
 * feature as a separate constraint. One nearest face cannot balance a force
 * in the normal cone of a mesh corner. Discovery is transactional: restart
 * from the incoming pose with extra geometry, never with a rejected force.
 */
export function createSharedAxisVesselDiscovery(field,sheathLength,{allSamples=true,queryReuse=true,indexedContacts=true,reuseContactBuffers=true}={}) {
    const clearance=createSharedAxisDiscoveryCache();
    const x=new Float64Array(2),y=x.slice(),z=x.slice(),r=x.slice(),out=createContactResult();
    const query=reuseContactBuffers?{a:[0,0,0],b:[0,0,0],sampleCount:0,geometryToken:null,gridToken:null,radius:0}:null;
    const sample=({state,a,b,edge,radius,coordinateA,coordinateB})=>{
        if(coordinateB<=sheathLength)return {gap:1,jacobian:[0,0,0,0,0,0]};
        const exposedStart=Math.max(0,(sheathLength-coordinateA)/(coordinateB-coordinateA));
        const origin=state.origin??[0,0,0];
        const intervals=state.materials?sharedAxisOuterIntervals(state,edge):[{start:0,end:1,material:{body:{radius}}}];
        for(const interval of intervals) {
            const start=Math.max(exposedStart,interval.start),end=interval.end;
            if(end<=start)continue;
            const radius=interval.material.body.radius;
            if(query) {
                for(let k=0;k<3;k++) {
                    query.a[k]=(1-start)*a[k]+start*b[k]+origin[k];
                    query.b[k]=(1-end)*a[k]+end*b[k]+origin[k];
                }
                x[0]=query.a[0];x[1]=query.b[0];y[0]=query.a[1];y[1]=query.b[1];z[0]=query.a[2];z[1]=query.b[2];
            } else {
                const clipped=a.map((v,k)=>(1-start)*v+start*b[k]),tip=a.map((v,k)=>(1-end)*v+end*b[k]);
                x[0]=clipped[0]+origin[0];x[1]=tip[0]+origin[0];y[0]=clipped[1]+origin[1];y[1]=tip[1]+origin[1];z[0]=clipped[2]+origin[2];z[1]=tip[2]+origin[2];
            }
            r.fill(radius);
            const length=(coordinateB-coordinateA)*(end-start),count=Math.max(1,Math.ceil(length/Math.max(field.voxelSize*4,Math.max(.5,radius))));
            const key=`${coordinateA}/${coordinateB}/${start}/${end}`,descriptor=query??{a:[x[0],y[0],z[0]],b:[x[1],y[1],z[1]],sampleCount:count,
                geometryToken:field.fallbackGeometry?.boundsTree,gridToken:key,radius};
            // lookup is synchronous; begin copies both endpoints and tokens
            // into its certificate before this descriptor can be reused.
            if(query){query.sampleCount=count;query.geometryToken=field.fallbackGeometry?.boundsTree;query.gridToken=key;query.radius=radius;}
            const proof=queryReuse&&allSamples?clearance.lookup(key,descriptor):null;
            if(proof?.skip)continue;
            // The initial sign is the same physical classification used by
            // the existing finite-mesh contact API. Subsequent movement must
            // remain within the exact surface-distance bound to reuse it.
            const certificate=queryReuse&&allSamples?clearance.begin(key,descriptor,{insideCertified:true}):null;
            let visited=0;
            const visit=(contact,localT)=>{
                visited++;certificate?.visit(contact,localT);
                if(contact.signedGap<.5) {
                    const t=start+(end-start)*localT,face=contact.faceIndex;
                    const owner=intervals.length>1?interval.material.spec.id:null;
                    const id=`vessel/${coordinateA}/${coordinateB}/${t}/${face}${owner?'/'+owner:''}`;
                    // Native rows are append-only within one state. Feed/replay
                    // rebuild their own index; pending discoveries stay separate
                    // until extendSharedAxisNativeRows accepts their topology.
                    const retained=indexedContacts&&state.definitionIds?state.definitionIds.has(id):state.definitions.some(r=>r.id===id);
                    if(!retained&&!state.pendingVesselRows?.has(id)) {
                        const pending=state.pendingVesselRows??=new Map();
                        pending.set(id,createSharedAxisVesselWitness(field,{kind:'wall',edge,id,witness:{face,t,...(owner?{owner}:{})},
                            dofs:[state.layout.positions[edge],state.layout.positions[edge+1]].flatMap(i=>[i,i+1,i+2])},{reuseBuffers:reuseContactBuffers}));
                    }
                }
                if(contact.signedDistance<=0)throw outsideError('Shared axis crossed the vessel surface');
            };
            try {
            const c=field.queryCapsuleSoA(x,y,z,r,0,out,-1,proof?.knownInside??false,false,-1,false,length,count,true,true,allSamples?visit:null);
            // Custom test/adapter fields may provide only the historical winner
            // API. Production visits every sphere already queried by its BVH.
            if(!visited)visit(c,c.segmentT);
            certificate?.commit();
            }catch(error){certificate?.abort();throw error;}

        }
        return {gap:1,jacobian:[0,0,0,0,0,0]};
    };
    sample.discoveryCache=clearance;
    return sample;
}

export function* iterateSharedAxisWithContacts(s,options={}) {
    const started=performance.now();let totalIterations=0,factorizations=0,backtracks=0,localRestarts=0,fullAssemblies=0,residualAssemblies=0;
    const timings={assemblyMs:0,linearMs:0,tangentAssemblyMs:0,residualAssemblyMs:0};
    for(let restarts=0;restarts<=64;restarts++) {
        const result=yield* iterateSharedAxisNative(s,options);
        localRestarts+=result.geometryRestarts??0;totalIterations+=result.iterations;factorizations+=result.factorizations;backtracks+=result.backtracks;
        for(const k of Object.keys(timings))timings[k]+=result.timings[k]??0;
        fullAssemblies+=result.fullAssemblies??0;residualAssemblies+=result.residualAssemblies??0;
        if(result.error===NEED_ROWS&&s.pendingVesselRows?.size&&restarts<64) {
            extendSharedAxisNativeRows(s,[...s.pendingVesselRows.values()]);s.pendingVesselRows.clear();continue;
        }
        return {...result,iterations:totalIterations,factorizations,backtracks,timings,fullAssemblies,residualAssemblies,geometryRestarts:restarts+localRestarts,ms:performance.now()-started};
    }
}

export function relaxSharedAxisWithContacts(s,options={}) {
    const iterator=iterateSharedAxisWithContacts(s,options);
    let next;do{next=iterator.next();}while(!next.done);
    return next.value;
}
