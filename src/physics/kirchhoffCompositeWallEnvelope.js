import {createCompositeWallWorkspace,canonicalizeCompositeWallReactions} from './kirchhoffCompositeWallContacts.js';
import {createContactResult} from './collision/vesselContactField.js';
import {initializeCompositeWallDifferentialRow,captureCompositeWallDifferentialContact,differentiateCompositeWallRow,
    requireCompositeWallDifferentialRows,isCompositeWallEndpointCombination} from './kirchhoffCompositeWallDifferentialRows.js';

/** Both endpoint sphere constraints and the original capsule minimum. A
 * capsule can have many simultaneous minimizers on a flat wall. Retaining
 * endpoint reactions avoids forcing a single arbitrary minimum's subgradient
 * to carry the load of the entire edge. The ORIGINAL capsule query remains
 * present and is checked on every trial. Exact local dependence is removed
 * only from the linear solve, with all original gaps still measured.
 */
export function createCompositeWallEnvelopeWorkspace(layout) {
    const w=createCompositeWallWorkspace(layout);
    w.rows=Array.from({length:3*(layout.nodeCount-1)},(_,index)=>initializeCompositeWallDifferentialRow({index,edge:Math.floor(index/3),
        role:['proximal','distal','capsule'][index%3],included:false,owner:null,gap:Infinity,t:index%3,
        normal:new Float64Array(3),closestPoint:new Float64Array(3)},layout));
    w.trialNormalForces=new Float64Array(w.rows.length);w.endpointScratch=createContactResult();
    return w;
}

export function refreshCompositeWallEnvelope({positions,contactOwners,field,lambdas},w) {
    if(positions.length!==w.layout.nodeCount||contactOwners.edges.length!==positions.length-1)
        throw new RangeError('Envelope geometry/ownership must match the frozen mesh');
    if(typeof field?.queryCapsuleCoordinates!=='function')throw new TypeError('Vessel capsule query interface required');
    if(positions.some(p=>p.length!==3||!p.every(Number.isFinite)))throw new TypeError('Finite positions required');
    w.queries=0;const cachedEndpoints=new Map();
    function read(row,result,t,radius) {
        if(!Number.isFinite(result.signedGap)||!Number.isFinite(t)||t<0||t>1||
            !result.inward.values.every(Number.isFinite)||Math.abs(Math.hypot(...result.inward.values)-1)>1e-8||
            !result.closestPoint.values.every(Number.isFinite))throw new RangeError('Finite wall gap, fraction and unit normal required');
        row.gap=result.signedGap;row.t=t;row.normal.set(result.inward.values);row.closestPoint.set(result.closestPoint.values);
        row.faceIndex=result.faceIndex;row.branchId=result.branchId;row.source=result.source;row.sampleCount=result.capsuleSampleCount;
        captureCompositeWallDifferentialContact(row,result,radius);
        differentiateCompositeWallRow({field,positions,row},w.differentialScratch);
    }
    for(let edge=0;edge<positions.length-1;edge++) {
        const owner=contactOwners.edges[edge],wall=owner.wall;
        if(owner.edge!==edge)throw new RangeError('Wall ownership must follow edges');
        const rows=w.rows.slice(3*edge,3*edge+3);
        for(const row of rows){row.included=wall!=null;row.owner=wall?.owner??null;}
        if(!wall){rows.forEach(row=>{row.gap=Infinity;differentiateCompositeWallRow({field,positions,row},w.differentialScratch);});continue;}
        if(!w.layout.edgeToolIds[edge].includes(wall.owner)||!Number.isFinite(wall.radius)||wall.radius<=0)
            throw new RangeError('A present outer material with positive radius is required');
        const a=positions[edge],b=positions[edge+1];
        for(let end=0;end<2;end++) {
            const node=edge+end,key=[node,wall.owner,wall.radius].join('|'),cached=cachedEndpoints.get(key);
            if(cached) {
                const row=rows[end];row.gap=cached.gap;row.t=end;row.normal.set(cached.normal);row.closestPoint.set(cached.closestPoint);
                for(const key of ['faceIndex','branchId','source','sampleCount'])row[key]=cached[key];
                captureCompositeWallDifferentialContact(row,cached.rawContact,wall.radius);
                differentiateCompositeWallRow({field,positions,row},w.differentialScratch);
            } else {
                const point=positions[node],result=field.queryCapsuleCoordinates(...point,...point,wall.radius,w.endpointScratch);
                w.queries++;read(rows[end],result,end,wall.radius);cachedEndpoints.set(key,rows[end]);
            }
        }
        const result=field.queryCapsuleCoordinates(...a,...b,wall.radius,w.scratch);w.queries++;read(rows[2],result,result.segmentT,wall.radius);
    }
    if(lambdas!==undefined)requireCompositeWallDifferentialRows(w,lambdas);
    return w;
}

/** An interior capsule's ORIGINAL g/G/B/DB must all equal its weighted
 * endpoint combination exactly, with matching source/owner/radius. Transfer
 * its physical force using precisely these weights, then
 * remove that dependent row from the mixed direction. This preserves total
 * force and moment. No tolerance, SVD or nearby-contact heuristic is used.
 * Canonical endpoint duplicates are combined after this local transfer.
 */
export function canonicalizeCompositeWallEnvelope(w,lambdas,owners) {
    requireCompositeWallDifferentialRows(w,lambdas);
    const dependent=new Set();
    for(let edge=0;edge<w.layout.nodeCount-1;edge++) {
        const [a,b,c]=w.rows.slice(3*edge,3*edge+3);
        if(!c.included||c.t===0||c.t===1)continue;
        if(!isCompositeWallEndpointCombination(a,b,c))continue;
        const force=lambdas[c.index];
        if(!Number.isFinite(force)||force<0)throw new RangeError('Physical envelope reaction must be finite and nonnegative');
        const first=lambdas[a.index]+(1-c.t)*force,second=lambdas[b.index]+c.t*force;
        if(!Number.isFinite(first)||!Number.isFinite(second))throw new RangeError('Physical envelope force overflowed');
        lambdas[a.index]=first;lambdas[b.index]=second;lambdas[c.index]=0;dependent.add(c.index);
    }
    return canonicalizeCompositeWallReactions(w,lambdas,owners).filter(index=>!dependent.has(index));
}
