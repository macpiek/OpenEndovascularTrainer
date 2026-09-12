import { condenseKirchhoffAxialBlock } from './kirchhoffAxialCondensation.js';
import { condenseKirchhoffAxialSparse } from './kirchhoffAxialSparseCondensation.js';
import { measureCoupledLoadKKT } from './kirchhoffCoupledLoadSolver.js';
import { assembleKirchhoffCoupledSystem } from './kirchhoffCoupledSystem.js';
import { partitionKirchhoffAxialSections } from './kirchhoffAxialSections.js';

/** Build directly from the original mechanical Jacobians, without first
 * constructing the two-body dual Gram or its common-relative basis. */
export function assembleKirchhoffAxialSystem(constraint, dt = 1 / 120, options = {}) {
    const source=assembleKirchhoffCoupledSystem(constraint,dt,
        {...options,jacobianOnly:true,includeAxialLayout:true});
    return assembleKirchhoffAxialReducedSystem(source,{matrixFormat:'general-band',sectionSpan:options.sectionSpan,sectionScope:options.sectionScope});
}

/** Local/reference mixed formulation of the existing Kirchhoff equations.
 * Soft material rows enter H exactly; zero-compliance and contact rows stay
 * explicit. Catheter translation offsets in overlap are statically eliminated,
 * not fixed to zero. All rotations and uncovered translations remain global.
 * Dense storage is an oracle; general-band assembles local sparse updates.
 */
export function assembleKirchhoffAxialReducedSystem(system, {matrixFormat = 'row-major',sectionSpan,sectionScope='catheter'} = {}) {
    if(!['row-major','general-band'].includes(matrixFormat)) throw new TypeError('Unknown axial matrix format');
    if(!['catheter','all'].includes(sectionScope))throw new TypeError('Unknown axial section scope');
    if(system.originalGroups) throw new TypeError('Use an unsolved native assembly so initial contact bounds and groups are preserved');
    const layout=system.axialLayout;
    if(!layout) throw new TypeError('Original system must include its axial layout');
    const snapshot={count:system.count,rhs:system.rhs.slice(),
        lower:system.lower.slice(),upper:system.upper.slice(),groups:structuredClone(system.groups),
        alpha:Float64Array.from(system.order,row=>system.rows[row].alpha),sizes:system.bodies.map(b=>b.count*6)};
    const dofs=[], maps=system.bodies.map(b=>new Int32Array(b.count*6).fill(-1));
    for(let side=0;side<system.bodies.length;side++) {
        const s=system.material[side];
        for(let dof=s.start*6;dof<(s.end+1)*6;dof++) if(s.weight[dof]>0) {
            maps[side][dof]=dofs.length; dofs.push({side,dof,weight:s.weight[dof]});
        }
    }
    const n=dofs.length, P=dofs.map((_,i)=>[[i,1]]), eliminated=new Set();
    for(const site of (layout.kind==='single'?[]:layout.sites)) if(site.wireSlots.length) for(let axis=0;axis<3;axis++) {
        const row=maps[1][site.nodeIndex*6+axis];
        if(row<0) continue;
        eliminated.add(row);
        for(let k=0;k<site.wireNodes.length;k++) {
            const column=maps[0][site.wireNodes[k]*6+axis];
            if(column>=0 && site.weights[k]!==0) P[row].push([column,site.weights[k]]);
        }
    }
    const originalRows=Array.from({length:system.count},()=>[]);
    dofs.forEach(({side,dof},i)=>{
        const column=system.columns[side][dof];
        for(let k=0;k<column.length;k+=2) originalRows[column[k]].push([i,column[k+1]]);
    });
    const gradients=originalRows.map(row=>{
        const values=new Map();
        for(const [i,v] of row) for(const [j,p] of P[i]) values.set(j,(values.get(j)??0)+v*p);
        return [...values].filter(([,v])=>v!==0).sort((a,b)=>a[0]-b[0]);
    });
    const hRows=Array.from({length:n},()=>new Map()), b=new Float64Array(n), remaining=[], soft=[];
    function addOuter(row,weight) {
        for(let a=0;a<row.length;a++) for(let c=0;c<=a;c++) {
            const [i,vi]=row[a],[j,vj]=row[c],value=weight*vi*vj;
            hRows[i].set(j,(hRows[i].get(j)??0)+value);
            if(i!==j) hRows[j].set(i,(hRows[j].get(i)??0)+value);
        }
    }
    dofs.forEach((d,i)=>addOuter(P[i],1/d.weight));
    for(let row=0;row<system.count;row++) {
        const source=system.rows[system.order[row]];
        if(source.kind==='material' && source.alpha>0 && system.lower[row]===-Infinity && system.upper[row]===Infinity) {
            soft.push(row); addOuter(gradients[row],1/source.alpha);
            for(const [i,v] of gradients[row]) b[i]+=v*system.rhs[row]/source.alpha;
        } else remaining.push(row);
    }
    const coordinates=system.bodies.map(body=>new Float64Array(body.count));
    if(layout.kind==='single') {
        if(system.bodies.length!==1) throw new TypeError('Single-axis layout requires one real body');
        layout.nodes.forEach((node,i)=>coordinates[0][node]=layout.coordinates[i]);
    } else {
        if(system.bodies.length!==2) throw new TypeError('Pair-axis layout requires two real bodies');
        layout.wireNodes.forEach((node,i)=>coordinates[0][node]=layout.wireAlignedCoordinates[i]);
        layout.sites.forEach(site=>coordinates[1][site.nodeIndex]=site.axialCoordinate);
    }
    const mixed=matrixFormat==='general-band'
        ? condenseKirchhoffAxialSparse({hRows,b,gradients,
            explicitRows:remaining.map(row=>({gradientIndex:row,alpha:snapshot.alpha[row],rhs:snapshot.rhs[row],
                lower:snapshot.lower[row],upper:snapshot.upper[row]})),
            eliminatedIndices:[...eliminated],axialCoordinates:dofs.map(({side,dof})=>coordinates[side][Math.floor(dof/6)])})
        : assembleDenseMixed(hRows,b,gradients,remaining,eliminated,snapshot);
    const {matrix,rhs,lower,upper,count}=mixed;
    const rowMap=new Int32Array(system.count).fill(-1);
    remaining.forEach((v,i)=>rowMap[v]=mixed.explicitIndices[i]);
    const groups=structuredClone(snapshot.groups).map(g=>{
        if(g.rows.some(r=>rowMap[r]<0) || g.normalRow!=null && rowMap[g.normalRow]<0)
            throw new Error('A contact law cannot be absorbed into material stiffness');
        return {...g,rows:g.rows.map(r=>rowMap[r]),normalRow:g.normalRow==null?undefined:rowMap[g.normalRow]};
    });
    let localSections;
    if(sectionSpan!==undefined) {
        if(matrixFormat!=='general-band')throw new TypeError('Local sections require general-band assembly');
        const candidates=dofs.flatMap((d,i)=>d.side===1&&mixed.primalSolutionIndices[i]>=0?[mixed.primalSolutionIndices[i]]:[]);
        remaining.forEach((row,i)=>{
            if(originalRows[row].some(([dof])=>dofs[dof].side===1))candidates.push(mixed.explicitIndices[i]);
        });
        localSections=partitionKirchhoffAxialSections(matrix,groups,mixed.coordinates,
            sectionScope==='all'?Array.from({length:count},(_,i)=>i):candidates,sectionSpan);
    }
    function recover(solution) {
        if(solution.length!==count || !solution.every(Number.isFinite)) throw new RangeError('Invalid mixed solution');
        const q=mixed.recover(solution);
        if(!q.every(Number.isFinite)) throw new RangeError('Axial reconstruction overflow');
        const correction=snapshot.sizes.map(size=>new Float64Array(size));
        dofs.forEach(({side,dof},i)=>{for(const [j,v] of P[i]) correction[side][dof]+=v*q[j];});
        const increment=new Float64Array(snapshot.count);
        remaining.forEach((row,i)=>increment[row]=solution[mixed.explicitIndices[i]]);
        for(const row of soft) {
            let value=snapshot.rhs[row];
            for(const [i,v] of gradients[row]) value-=v*q[i];
            increment[row]=value/snapshot.alpha[row];
        }
        if(!increment.every(Number.isFinite) || !correction.every(v=>v.every(Number.isFinite)))
            throw new RangeError('Axial reaction reconstruction overflow');
        let maximumMobilityError=0;
        const impulses=new Float64Array(n);
        originalRows.forEach((entries,row)=>{for(const [i,v] of entries) impulses[i]+=v*increment[row];});
        dofs.forEach(({side,dof,weight},i)=>{
            maximumMobilityError=Math.max(maximumMobilityError,Math.abs(correction[side][dof]-weight*impulses[i]));
        });
        const residual=snapshot.rhs.slice();
        // Certify the original dual equations directly through J W Jᵀ.
        // This avoids retaining a second band matrix just for the audit.
        originalRows.forEach((entries,row)=>{
            residual[row]-=snapshot.alpha[row]*increment[row];
            for(const [i,v] of entries) residual[row]-=v*dofs[i].weight*impulses[i];
        });
        if(!residual.every(Number.isFinite) || !Number.isFinite(maximumMobilityError))
            throw new RangeError('Original axial certificate overflow');
        const finalGroups=snapshot.groups.map(g=>({...g,radii:g.normalRow==null?g.radii:
            g.mu.map(mu=>mu*Math.max(0,g.normalLambda+increment[g.normalRow]))}));
        const originalKkt=measureCoupledLoadKKT(residual,increment,snapshot.lower,snapshot.upper,finalGroups);
        return {correction,increment,residual,maximumMobilityError,originalKkt};
    }
    return {matrix,rhs,lower,upper,count,groups,recover,matrixFormat,localSections,
        diagnostics:{originalDofs:n,retainedDofs:mixed.retainedIndices.length,softMaterialRows:soft.length,explicitRows:remaining.length,
            ...mixed.diagnostics}};
}

function assembleDenseMixed(hRows,b,gradients,remaining,eliminated,snapshot) {
    const n=b.length;
    const H=new Float64Array(n*n);
    hRows.forEach((row,i)=>{for(const [j,v] of row) H[i*n+j]=v;});
    const retained=Array.from({length:n},(_,i)=>i).filter(i=>!eliminated.has(i));
    const block=condenseKirchhoffAxialBlock({matrix:H,rhs:b,count:n,retainedIndices:retained});
    const nr=retained.length, ne=block.eliminatedIndices.length, nc=remaining.length, count=nr+nc;
    const retainedMap=new Int32Array(n).fill(-1), eliminatedMap=new Int32Array(n).fill(-1);
    retained.forEach((v,i)=>retainedMap[v]=i); block.eliminatedIndices.forEach((v,i)=>eliminatedMap[v]=i);
    const Je=[], Jr=[], responses=[];
    for(const row of remaining) {
        const e=new Float64Array(ne),r=new Float64Array(nr);
        for(const [i,v] of gradients[row]) {
            if(retainedMap[i]>=0) r[retainedMap[i]]=v;
            else e[eliminatedMap[i]]=v;
        }
        Je.push(e); Jr.push(r); responses.push(block.solveEliminated(e));
    }
    const free=block.solveEliminated(Float64Array.from(block.eliminatedIndices,i=>b[i]));
    const matrix=new Float64Array(count*count),rhs=new Float64Array(count);
    const lower=new Float64Array(count).fill(-Infinity),upper=new Float64Array(count).fill(Infinity);
    for(let i=0;i<nr;i++) {
        rhs[i]=block.rhs[i];
        for(let j=0;j<nr;j++) matrix[i*count+j]=block.matrix[i*nr+j];
    }
    for(let c=0;c<nc;c++) {
        const row=remaining[c],i=nr+c;
        rhs[i]=snapshot.rhs[row]; lower[i]=snapshot.lower[row];upper[i]=snapshot.upper[row];
        for(let e=0;e<ne;e++) rhs[i]-=Je[c][e]*free[e];
        for(let r=0;r<nr;r++) {
            let value=Jr[c][r];
            for(let e=0;e<ne;e++) value-=H[retained[r]*n+block.eliminatedIndices[e]]*responses[c][e];
            matrix[i*count+r]=value; matrix[r*count+i]=-value;
        }
        for(let d=0;d<nc;d++) {
            let value=c===d?snapshot.alpha[row]:0;
            for(let e=0;e<ne;e++) value+=Je[c][e]*responses[d][e];
            matrix[i*count+nr+d]=value;
        }
    }
    return {matrix,rhs,lower,upper,count,retainedIndices:retained,
        explicitIndices:remaining.map((_,i)=>nr+i),
        recover(solution) {
            const q=block.recover(solution.slice(0,nr));
            for(let e=0;e<ne;e++) for(let c=0;c<nc;c++) q[block.eliminatedIndices[e]]+=responses[c][e]*solution[nr+c];
            return q;
        }, diagnostics:{eliminatedSolver:block.diagnostics.eliminatedSolver,
            eliminatedFactorEntries:block.diagnostics.factorEntries,eliminatedOffsetDofs:ne}};
}
