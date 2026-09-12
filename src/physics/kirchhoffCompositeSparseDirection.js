import {createCoulombBandLU} from './kirchhoffCoulombBandLU.js';

/** Compile every ORIGINAL structural coefficient before any active-set or
 * zero-dual decision. The common/rho/dual ordering is unchanged. Numeric
 * zeros never shrink this pattern; both nonsymmetric directions and every
 * row's geometric tangent are represented explicitly.
 */
export function createCompositeSparseDirectionOperator(w) {
    const {count,common,relative,dual,layout,rows}=w;
    const sets=Array.from({length:count},(_,i)=>new Set([i]));
    const mark=(a,b)=>sets[a].add(b),pair=(a,b)=>{mark(a,b);mark(b,a);};
    for(let i=0;i<common.length;i++)for(let j=Math.max(0,i-layout.band+1);j<=i;j++)pair(common[i],common[j]);
    for(const scatter of [w.relativeScatter,w.couplingScatter])for(let i=0;i<scatter.length;i+=3)pair(scatter[i+1],scatter[i+2]);
    rows.forEach((row,i)=>{
        for(const a of row.mixedDofs){pair(a,dual[i]);for(const b of row.mixedDofs)mark(a,b);}
        row.multiplierDofs.forEach(j=>mark(dual[i],dual[j]));
    });
    const offsets=new Int32Array(count+1),columns=[],lookup=[];
    sets.forEach((set,i)=>{
        offsets[i]=columns.length;const map=new Map();
        for(const j of [...set].sort((a,b)=>a-b)){map.set(j,columns.length);columns.push(j);}
        lookup.push(map);
    });
    offsets[count]=columns.length;
    const at=(a,b)=>{const index=lookup[a].get(b);if(index===undefined)throw new RangeError('Coefficient lies outside the original sparse stencil');return index;};
    const commonScatter=[];
    for(let i=0;i<common.length;i++)for(let j=Math.max(0,i-layout.band+1);j<=i;j++)
        commonScatter.push(i*layout.band+i-j,at(common[i],common[j]),i===j?-1:at(common[j],common[i]));
    const scatterPlan=scatter=>{
        const plan=[];for(let i=0;i<scatter.length;i+=3){const a=scatter[i+1],b=scatter[i+2];plan.push(scatter[i],at(a,b),a===b?-1:at(b,a));}
        return Int32Array.from(plan);
    };
    const rowPlans=rows.map((row,i)=>({diagonal:at(dual[i],dual[i]),
        multiplierSlots:Int32Array.from(row.multiplierDofs,j=>at(dual[i],dual[j])),
        jacobianSlots:Int32Array.from(row.mixedDofs,d=>at(dual[i],d)),
        forceSlots:Int32Array.from(row.mixedDofs,d=>at(d,dual[i])),
        tangentSlots:Int32Array.from(Array.from(row.mixedDofs,a=>Array.from(row.mixedDofs,b=>at(a,b))).flat())}));
    const op={offsets,columns:Int32Array.from(columns),diagonal:Int32Array.from({length:count},(_,i)=>at(i,i)),rowPlans,
        commonScatter:Int32Array.from(commonScatter),relativeScatter:scatterPlan(w.relativeScatter),couplingScatter:scatterPlan(w.couplingScatter),
        original:new Float64Array(columns.length),values:new Float64Array(columns.length),generation:0};
    let originalView=null,numericalView=null,originalVersion=-1,numericalVersion=-1,nativeLU=null;
    function materialize(values,out) {
        out??=new Float64Array(w.packedLayout.entries);out.fill(0);
        for(let i=0;i<count;i++)for(let k=offsets[i];k<offsets[i+1];k++)out[w.packedLayout.offsets[i]+columns[k]]=values[k];
        return out;
    }
    // Explicit diagnostic/compatibility views are materialized on demand.
    // A compressed production solve never reads them. Kept references are
    // snapshots until the property is read again after the next assembly.
    Object.defineProperties(w,{
        originalMatrix:{get(){if(originalVersion!==op.generation){originalView=materialize(op.original,originalView);originalVersion=op.generation;}return originalView;}},
        matrix:{get(){if(numericalVersion!==op.generation){numericalView=materialize(op.values,numericalView);numericalVersion=op.generation;}return numericalView;}},
        lu:{get(){return nativeLU??=createCoulombBandLU(w.packedLayout,count);}},
    });
    Object.defineProperties(op,{
        materializedOriginalEntries:{get:()=>originalView?.length??0},
        materializedNumericalEntries:{get:()=>numericalView?.length??0},
        nativeFactorEntries:{get:()=>nativeLU?.diagnostics.factorEntries??0},
    });
    return op;
}
