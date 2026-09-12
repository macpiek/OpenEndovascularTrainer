import {createCoulombBandLayout} from './kirchhoffCoulombBandLU.js';

/** Promote every cross-section endpoint to a global boundary variable.
 * Friction Jacobian row envelopes include both tangents and their load, so
 * the partition remains valid when sliding/sticking switches within Newton.
 */
export function partitionKirchhoffAxialSections(matrix,groups,coordinates,candidates,span) {
    if(!Number.isFinite(span)||span<=0||!coordinates.every(Number.isFinite))throw new RangeError('Invalid section span or coordinates');
    const count=coordinates.length;
    createCoulombBandLayout(matrix,count,count,groups,'general-band');
    const labels=new Array(count).fill(null),boundary=new Set();
    for(const i of candidates){
        if(!Number.isInteger(i)||i<0||i>=count)throw new RangeError('Invalid local section candidate');
        labels[i]=Math.floor(coordinates[i]/span);
    }
    const edge=(i,j)=>{
        if(labels[i]!==null&&labels[j]!==null&&labels[i]!==labels[j]){boundary.add(i);boundary.add(j);}
    };
    for(let i=0;i<count;i++)for(let j=matrix.starts[i];j<=matrix.ends[i];j++)
        if(matrix.values[matrix.offsets[i]+j]!==0)edge(i,j);
    for(const group of groups) {
        const support=new Set(group.rows);
        if(group.normalRow!=null)support.add(group.normalRow);
        for(const i of group.rows)for(let j=matrix.starts[i];j<=matrix.ends[i];j++)
            if(matrix.values[matrix.offsets[i]+j]!==0)support.add(j);
        for(const i of group.rows)for(const j of support)edge(i,j);
    }
    const blocks=new Map();
    labels.forEach((label,i)=>{if(label===null||boundary.has(i))return;
        if(!blocks.has(label))blocks.set(label,[]);blocks.get(label).push(i);
    });
    return [...blocks.values()];
}
