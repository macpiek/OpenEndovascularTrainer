// Chooses a different direction strategy; never certifies or accepts a step.
// Compare best residuals in two adjacent windows so alternating wall sets
// cannot masquerade as progress. Storage is bounded and geometry-free.
export function createSharedAxisStagnationGuard(forceTolerance,lengthTolerance) {
    const half=8,window=2*half,history=new Float64Array(window*3);
    let count=0,rowCount=-1;
    return {observe(base) {
        if(rowCount!==base.rows.length){count=0;rowCount=base.rows.length;}
        const values=[base.force/forceTolerance,base.torque/forceTolerance,base.constraint/lengthTolerance];
        if(!values.every(v=>Number.isFinite(v)&&v>=0)){count=0;return null;}
        for(let k=0;k<3;k++)history[3*(count%window)+k]=values[k];
        count++;if(count<window)return null;
        const before=[Infinity,Infinity,Infinity],after=[Infinity,Infinity,Infinity];
        for(let i=0;i<window;i++)for(let k=0;k<3;k++) {
            const target=i<half?before:after;
            target[k]=Math.min(target[k],history[3*((count-window+i)%window)+k]);
        }
        // Do not interrupt near-tolerance settling. Any meaningful improvement
        // of a still-unsatisfied channel keeps the current strategy running.
        if(Math.max(...after)<=10)return null;
        for(let k=0;k<3;k++)if(before[k]>1&&after[k]<before[k]*.99)return null;
        return {window,before,after,minimumRelativeProgress:.01};
    }};
}
