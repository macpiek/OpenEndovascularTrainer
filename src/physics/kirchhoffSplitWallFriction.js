import { buildKirchhoffSurfaceFriction, evaluateKirchhoffSurfaceFrictionKKT } from './kirchhoffSurfaceFriction.js';
import { rotateVectorByQuaternion } from './discreteKirchhoffRod.js';
import { prepareKirchhoffWallFrictionEntry, recordKirchhoffWallFrictionApplication } from './kirchhoffWallFrictionMode.js';
const XYZ=['X','Y','Z'];
const wallKinds=new Set(['wall','split-point-wall','split-sweep']);
function fixedWitness() {
    const body={count:2,segmentCount:1,activeStart:0,activeEnd:1,nodeRadius:new Float64Array(2),inverseMass:new Float64Array(2),orientationControlSegment:-1};
    for(const a of XYZ) {
        body[a.toLowerCase()]=new Float64Array(2);body['previous'+a]=new Float64Array(2);
        body['velocity'+a]=new Float64Array(2);body['angularVelocity'+a]=new Float64Array(1);
    }
    for(const a of [...XYZ,'W'])for(const prefix of ['orientation','previousOrientation'])body[prefix+a]=new Float64Array([a==='W'?1:0]);
    for(let i=1;i<=3;i++)body['inverseInertia'+i]=new Float64Array(1);
    return body;
}
function normalLoad(row) {return row.kind==='wall'?row.owner.wallLambda[row.node]:row.lambda;}
function entryFor(batch,key,contact) {
    const pool=batch.pool??=new Map();let entry=pool.get(key);
    if(!entry) {
        const wall=fixedWitness();
        entry={wall,contact,record:{kind:'side',innerWeights:new Float64Array(2),outerWeights:new Float64Array([1,0]),
            _outerSegmentIndex:0,surfaceContactPoint:new Float64Array(3),surfaceAxialTangent:new Float64Array(3),normal:new Float64Array(3)},
            surface:{},constraint:{},rows:[{gradients:[]},{gradients:[]}],lambda:new Float64Array(2),residual:{}};
        pool.set(key,entry);
    }
    entry.contact=contact;
    return entry;
}
/** Static environment witness: its zero-mobility DOFs are omitted, while the
 * rod receives the SAME surface force and radius moment as the shared-point
 * surface law. Normal impulse is from the physical bank only. */
export function buildKirchhoffSplitWallFriction(joint,normalRows,dt,out={}) {
    out.rows??=[];out.groups??=[];out.entries??=[];
    out.rows.length=out.groups.length=out.entries.length=0;
    out.version=(out.version??0)+1;
    out.joint=joint;out.committed=false;out.rowOffset=0;
    const state=joint._splitMotion;if(!state||state.phase==='bias')return out;
    const contacts=state.wallFrictionContacts??=new Map();
    for(const normal of normalRows) {
        if(!wallKinds.has(normal.kind))continue;
        const side=normal.side,body=side?joint.outerBody:joint.innerBody;
        let mu=body.wallStaticFriction;
        if(!(mu>0||body.wallKineticFriction>0))continue;
        const key=`${normal.kind}:${side}:${normal.node}`;
        let contact=contacts.get(key);
        if(!contact) {
            contact={key,normalRow:normal,tangentLambda:new Float64Array(2),tangentU:new Float64Array([1,0,0]),tangentV:new Float64Array([0,0,1]),twistLambda:0,
                get normalLambda(){return normalLoad(this.normalRow);}};
            contacts.set(key,contact);
        }
        contact.normalRow=normal;
        const entry=entryFor(out,key,contact),r=entry.record;
        const segment=Math.min(normal.node,body.activeEnd-1),t=normal.kind==='wall'?body.wallT[normal.node]:Number(normal.node===body.activeEnd);
        r._innerSegmentIndex=segment;r.innerWeights[0]=1-t;r.innerWeights[1]=t;r.manifoldContact=contact;
        for(let a=0;a<3;a++)r.normal[a]=normal.kind==='wall'?body['wallNormal'+XYZ[a]][normal.node]:(normal.normal??normal.n)[a];
        const radius=Math.max(body.nodeRadius[segment],body.nodeRadius[segment+1]);
        const q={x:body.orientationX[segment],y:body.orientationY[segment],z:body.orientationZ[segment],w:body.orientationW[segment]};
        let tangent=rotateVectorByQuaternion(q,{x:0,y:0,z:1});
        let alignment=XYZ.reduce((sum,a,i)=>sum+tangent[a.toLowerCase()]*r.normal[i],0);
        if(Math.abs(alignment)>1-1e-8)tangent=rotateVectorByQuaternion(q,{x:1,y:0,z:0});
        for(let a=0;a<3;a++) {
            const axis=XYZ[a],center=(1-t)*body[axis.toLowerCase()][segment]+t*body[axis.toLowerCase()][segment+1];
            r.surfaceContactPoint[a]=center-radius*r.normal[a];r.surfaceAxialTangent[a]=tangent[axis.toLowerCase()];
            entry.wall[axis.toLowerCase()].fill(r.surfaceContactPoint[a]);entry.wall['previous'+axis].fill(r.surfaceContactPoint[a]);
        }
        entry.wall.z[1]+=1;entry.wall.previousZ[1]+=1;
        entry.normalRow=normal;entry.side=side;
        entry.normalIndex=normalRows.indexOf(normal);entry.baseNormalLambda=contact.normalLambda;
        mu=prepareKirchhoffWallFrictionEntry(joint,entry);
        Object.assign(entry.constraint,{innerBody:body,outerBody:entry.wall,axialFriction:mu,circumferentialFriction:mu,
            surfaceMotion:{dt,bodies:[state.bodies[side],entry.wall]}});
        const surface=buildKirchhoffSurfaceFriction(entry.constraint,r,dt,entry.surface);
        if(!surface.supported)throw new Error('Unsupported physical wall witness: '+surface.reason);
        entry.rowStart=out.rows.length;entry.normalRow=normal;entry.side=side;
        for(let axis=0;axis<2;axis++) {
            const source=surface.rows[axis],row=entry.rows[axis];
            row.kind='split-wall-friction';row.alpha=0;row.lambda=source.lambda;row.strain=source.strain;row.lower=-Infinity;row.upper=Infinity;
            row.gradients=source.gradients.filter(g=>g.side===0).map(g=>({side,dof:g.dof,value:g.value}));out.rows.push(row);
        }
        out.groups.push({kind:'coulomb-disk',mu:surface.group.mu,normalLambda:contact.normalLambda,normalContact:contact,normalRow:normal,
            rowIndices:[entry.rowStart,entry.rowStart+1]});out.entries.push(entry);
    }
    return out;
}
export function appendKirchhoffSplitWallFriction(batch,rows,groups) {
    batch.rowOffset=rows.length;
    rows.push(...batch.rows);
    for(const group of batch.groups)groups.push({...group,rowIndices:group.rowIndices.map(i=>i+batch.rowOffset)});
}
export function commitKirchhoffSplitWallFriction(batch,increments,scale) {
    if(batch.committed)throw new Error('Physical wall friction batch already committed');
    for(const e of batch.entries) {
        for(let axis=0;axis<2;axis++) {
            const delta=scale*increments[batch.rowOffset+e.rowStart+axis];
            if(!Number.isFinite(delta))throw new RangeError('Nonfinite physical wall friction increment');
            e.lambda[axis]=e.rows[axis].lambda+delta;
        }
    }
    for(const e of batch.entries) {
        recordKirchhoffWallFrictionApplication(e,scale*increments[e.normalIndex],
            e.lambda.map((value,axis)=>value-e.rows[axis].lambda),scale);
        e.contact.tangentLambda.set(e.lambda);e.contact.tangentU.set(e.surface.axes[0]);e.contact.tangentV.set(e.surface.axes[1]);
    }
    batch.committed=true;
}
export function measureKirchhoffSplitWallFriction(joint,normalRows,dt,out={}) {
    const batch=buildKirchhoffSplitWallFriction(joint,normalRows,dt,out._batch??={});
    out.maximumDisplacementResidualMm=out.maximumConeViolation=0;out.contactCount=batch.entries.length;
    for(const e of batch.entries) {
        const kkt=evaluateKirchhoffSurfaceFrictionKKT(e.surface.rows.map(r=>r.lambda),e.surface.rows.map(r=>r.strain),e.contact.normalLambda,e.surface.group.mu,e.residual);
        out.maximumDisplacementResidualMm=Math.max(out.maximumDisplacementResidualMm,kkt.residualMm);
        out.maximumConeViolation=Math.max(out.maximumConeViolation,kkt.coneViolation);
    }
    return out;
}
