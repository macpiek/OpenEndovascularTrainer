import {compositeContinuousBezierControls} from './kirchhoffCompositeContinuousGeometry.js';

const dot=(a,b)=>a.reduce((s,v,k)=>s+v*b[k],0),cross=(a,b)=>[a[1]*b[2]-a[2]*b[1],a[2]*b[0]-a[0]*b[2],a[0]*b[1]-a[1]*b[0]],
    sub=(a,b)=>a.map((v,k)=>v-b[k]),unit=v=>{const n=Math.hypot(...v);if(!(n>0))throw new RangeError('Nonzero taut direction required');return v.map(x=>x/n);},
    key=(id,node)=>JSON.stringify([id,node]),historyKey=(id,edge,node)=>JSON.stringify([id,edge,node]);

function fullShapeRank(geometry,other) {
    // The endpoint controls lie on the prescribed line already. Full rank of
    // the remaining nodal coefficients proves that the whole polynomial lies
    // on that line iff each remaining shape node lies on it. No sampling-only
    // replacement of the continuous constraint is used.
    const columns=other.map(node=>compositeContinuousBezierControls(geometry,geometry.nodeIndices.map(i=>[i===node?1:0,0,0])).map(p=>p[0])),basis=[];
    for(let row=1;row<5;row++) {
        const v=columns.map(c=>c[row]),scale=Math.hypot(...v);if(scale===0)continue;
        let residual=v.map(x=>x/scale);
        for(const q of basis){const p=dot(q,residual);residual=residual.map((x,j)=>x-p*q[j]);}
        const norm=Math.hypot(...residual);if(norm>128*Number.EPSILON)basis.push(residual.map(x=>x/norm));
    }
    if(basis.length!==other.length)throw new RangeError('Taut continuous shape needs a resolved coefficient rank');
}

/** Exact normal form for a continuous arc whose two prescribed endpoints
 * are EXACTLY one rest length apart. The triangle inequality makes its
 * feasible set a monotone straight curve. The singular scalar length row is
 * replaced by regular collinearity rows; the ORIGINAL arc and regular tangent
 * remain mandatory acceptance checks in JointTimeStep.
 *
 * psi = n . ((q1-q0) x (qj-q0)) / rest. Both endpoint derivatives are kept:
 * the internal reaction has zero resultant, and zero moment on the feasible
 * set. Its full signed Hessian enters the same common/relative solve.
 * The redundant axial multiplier uses zero gauge, as for fully prescribed
 * chord lengths. Transverse reactions are normals of the taut feasible set;
 * they must not be reported as a finite axial tension of the singular row.
 */
export function createCompositeTautLengthBlock({lengthRows,positionBoundaries,restLengths,history=[],tolerance,linearTolerance=tolerance}) {
    if(!(tolerance>0)||!Number.isFinite(tolerance)||!(linearTolerance>0)||!Number.isFinite(linearTolerance))throw new RangeError('Positive taut constraint tolerances required');
    const rows=[],blocks=[],replaced=new Set(),planeBases=new Map(),old=new Map(history.map(r=>[historyKey(r.toolId,r.edge,r.node),r.normalMultiplier]));
    for(const length of lengthRows) {
        if(length.lengthGeometry!=='continuous-arclength')continue;
        const {toolId,edge,nodeIndices:nodes}=length,left=positionBoundaries.get(key(toolId,edge)),right=positionBoundaries.get(key(toolId,edge+1));
        if(!left||!right)continue;
        const rest=restLengths.get(toolId)[edge],chord=sub(right.value,left.value),distance=Math.hypot(...chord);
        // A genuinely slack arc, however small the slack, remains the full
        // nonlinear length constraint. This is not a tolerance-based lock.
        if(distance!==rest)continue;
        const other=nodes.filter(node=>node!==edge&&node!==edge+1);fullShapeRank(length.compiled.geometry,other);
        const tangent=chord.map(v=>v/rest),axis=[0,1,2].reduce((best,k)=>Math.abs(tangent[k])<Math.abs(tangent[best])?k:best,0),
            seed=[0,0,0];seed[axis]=1;
        const n0=unit(cross(tangent,seed)),normals=[n0,cross(tangent,n0)],block={toolId,edge,rest,tangent,nodes,other,lengthIndex:length.index,rows:[]};
        blocks.push(block);replaced.add(length.index);
        for(const node of other) {
            const prescribed=positionBoundaries.get(key(toolId,node));
            if(prescribed) {
                if(Math.hypot(...cross(tangent,sub(prescribed.value,left.value)))>tolerance)throw new RangeError('Prescribed shape nodes conflict with a taut material arc');
                continue;
            }
            const identity=key(toolId,node),group=planeBases.get(identity)??{basis:[],origin:left.value.slice()},basis=group.basis;planeBases.set(identity,group);
            for(let axis=0;axis<2;axis++) {
                const normal=normals[axis],plane=cross(normal,tangent);let remainder=plane.slice(),target=dot(plane,sub(left.value,group.origin));
                for(const q of basis){const p=dot(q.plane,remainder);remainder=remainder.map((v,k)=>v-p*q.plane[k]);target-=p*q.target;}
                const norm=Math.hypot(...remainder);
                if(norm<=128*Number.EPSILON) {
                    if(Math.abs(target)>tolerance)throw new RangeError('Prescribed taut lines have incompatible intersections');
                    continue; // A reaction gauge; every omitted original arc remains checked.
                }
                basis.push({plane:remainder.map(v=>v/norm),target:target/norm});
                const size=length.terms.length,N=3*nodes.length,
                    a=nodes.map(i=>(i===edge+1?1:0)-(i===edge?1:0)),b=nodes.map(i=>(i===node?1:0)-(i===edge?1:0)),
                    H=new Float64Array(N*N),J=new Float64Array(N);
                for(let i=0;i<N;i++)for(let j=0;j<N;j++) {
                    const ei=[0,0,0],ej=[0,0,0];ei[i%3]=1;ej[j%3]=1;
                    H[i*N+j]=(a[Math.floor(i/3)]*b[Math.floor(j/3)]-b[Math.floor(i/3)]*a[Math.floor(j/3)])*dot(normal,cross(ei,ej))/rest;
                }
                const row={toolId,edge,node,axis,normal,rest,anchorNode:edge,unit:'mm',commonDofs:length.commonDofs,relativeDofs:length.relativeDofs,
                    constraintSupport:{kind:'continuous-taut-arclength',toolId,edge,nodeIndices:nodes.slice()},
                    jacobian:new Float64Array(size),forceColumn:new Float64Array(size),geometricTangent:new Float64Array(size*size),geometricTangentValid:false,
                    residual:0,multiplierDerivative:0,tolerance:linearTolerance,lambda:dot(old.get(historyKey(toolId,edge,node))??[0,0,0],normal),
                    terms:length.terms,nodes,a,b,H,J};
                rows.push(row);block.rows.push(row);
            }
        }
    }
    return {rows,blocks,replaced,
        refresh({toolPositions,commonResidual,relativeResidual,order}) {
            let maximumResidual=0,merit=0;
            for(const block of blocks) {
                const p=toolPositions.get(block.toolId),u=sub(p[block.edge+1],p[block.edge]);
                if(!(dot(u,block.tangent)>128*Number.EPSILON*block.rest))throw new RangeError('Taut arc left its prescribed chord chart');
                for(const node of block.other){const gap=Math.hypot(...cross(u,sub(p[node],p[block.edge])))/block.rest;maximumResidual=Math.max(maximumResidual,gap);merit+=(gap/tolerance)**2;}
            }
            for(const row of rows) {
                const p=toolPositions.get(row.toolId),u=sub(p[row.edge+1],p[row.edge]),v=sub(p[row.node],p[row.edge]),
                    du=cross(v,row.normal),dv=cross(row.normal,u),N=row.J.length,size=row.terms.length;
                row.residual=dot(row.normal,cross(u,v))/row.rest;
                for(let j=0;j<N;j++)row.J[j]=(row.a[Math.floor(j/3)]*du[j%3]+row.b[Math.floor(j/3)]*dv[j%3])/row.rest;
                for(let i=0;i<size;i++) {
                    const ti=row.terms[i];let value=0;for(let a=0;a<ti.length;a+=2)value+=ti[a+1]*row.J[ti[a]];
                    row.jacobian[i]=row.forceColumn[i]=value;
                    if(i<row.commonDofs.length)commonResidual[row.commonDofs[i]]+=row.lambda*value;
                    else relativeResidual[row.relativeDofs[i-row.commonDofs.length]]+=row.lambda*value;
                    if(order==='full')for(let j=0;j<size;j++) {
                        const tj=row.terms[j];let h=0;for(let a=0;a<ti.length;a+=2)for(let b=0;b<tj.length;b+=2)h+=ti[a+1]*tj[b+1]*row.H[ti[a]*N+tj[b]];
                        row.geometricTangent[i*size+j]=row.lambda*h;
                    }
                }
                row.geometricTangentValid=order==='full';
            }
            return {maximumResidual,merit,converged:maximumResidual<=tolerance};
        },
        commit() {
            const records=new Map();
            for(const row of rows) {
                const id=historyKey(row.toolId,row.edge,row.node);if(!records.has(id))records.set(id,{toolId:row.toolId,edge:row.edge,node:row.node,normalMultiplier:[0,0,0]});
                const value=records.get(id).normalMultiplier;row.normal.forEach((v,k)=>value[k]+=v*row.lambda);
            }
            return [...records.values()];
        }
    };
}
