// A direction-strategy guard, never an acceptance certificate. Store only a
// bounded recent history and require the pose, active contacts and all residual
// channels to recur. The caller rolls back and retries the full physical step.
export function createSharedAxisCycleGuard() {
    const capacity=4,history=new Array(capacity),streak=new Uint8Array(capacity);
    let count=0,rowCount=-1;
    const close=(a,b,absolute,relative=0)=>Number.isFinite(a)&&Number.isFinite(b)&&
        Math.abs(a-b)<=absolute+relative*Math.max(Math.abs(a),Math.abs(b));
    return {
        observe(s,base) {
            if(rowCount!==base.rows.length){count=0;streak.fill(0);rowCount=base.rows.length;}
            const size=s.positions.length*3+s.materials.reduce((n,m)=>n+4*m.body.orientationX.length,0);
            const matches=saved=>{
                if(!saved||saved.pose.length!==size)return false;
                if(!close(base.force,saved.force,1e-12,1e-8)||!close(base.torque,saved.torque,1e-12,1e-8)||
                    !close(base.constraint,saved.constraint,1e-12,1e-8)||!close(base.energy,saved.energy,1e-10,1e-12))return false;
                for(let i=0;i<rowCount;i++)if(saved.active[i]!==Number(base.rows[i].kind==='wall'&&base.rows[i].multiplier>0))return false;
                let at=0;
                for(const p of s.positions)for(const v of p)if(!close(v,saved.pose[at++],1e-8))return false;
                for(const m of s.materials)for(const k of ['X','Y','Z','W'])for(const v of m.body['orientation'+k])
                    if(!close(v,saved.pose[at++],1e-10))return false;
                return true;
            };
            let cycle=null;
            for(let period=1;period<=Math.min(capacity,count);period++) {
                streak[period-1]=matches(history[(count-period)%capacity])?streak[period-1]+1:0;
                // At least three matching comparisons, and two complete
                // periods: one accidental recurrence cannot stop a direction.
                if(!cycle&&streak[period-1]>=Math.max(3,2*period))cycle={period,repeatedComparisons:streak[period-1]};
            }
            const index=count%capacity;
            let saved=history[index];
            if(!saved||saved.pose.length!==size||saved.active.length!==rowCount)
                saved=history[index]={pose:new Float64Array(size),active:new Uint8Array(rowCount)};
            let at=0;
            for(const p of s.positions)for(const v of p)saved.pose[at++]=v;
            for(const m of s.materials)for(const k of ['X','Y','Z','W'])for(const v of m.body['orientation'+k])saved.pose[at++]=v;
            for(let i=0;i<rowCount;i++)saved.active[i]=Number(base.rows[i].kind==='wall'&&base.rows[i].multiplier>0);
            saved.force=base.force;saved.torque=base.torque;saved.constraint=base.constraint;saved.energy=base.energy;
            count++;return cycle;
        }
    };
}
