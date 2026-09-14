// Reusable copies of mutable input/presentation storage. Native solver states,
// geometry objects and caches are deliberately outside this transaction.
export function createPreparedInputCheckpoint() {
    const pools=new WeakMap(),records=[];
    return {
        capture(targets) {
            records.length=0;
            for(const target of targets) {
                if(!target)continue;
                let record=pools.get(target);
                if(!record){record={target,entries:new Map(),stamp:0};pools.set(target,record);}
                const {entries}=record;record.stamp++;
                for(const key of Object.keys(target)) {
                    const value=target[key];
                    let entry=entries.get(key);if(!entry){entry={};entries.set(key,entry);}
                    entry.value=value;entry.stamp=record.stamp;
                    if(ArrayBuffer.isView(value)&&!(value instanceof DataView)) {
                        let copy=entry.copy;
                        if(!copy||copy.constructor!==value.constructor||copy.length!==value.length)copy=value.slice();
                        else copy.set(value);
                        entry.copy=copy;
                    } else entry.copy=null;
                }
                for(const [key,entry] of entries)if(entry.stamp!==record.stamp)entries.delete(key);
                records.push(record);
            }
        },
        restore() {
            for(const {target,entries} of records) {
                for(const key of Object.keys(target))if(!entries.has(key))delete target[key];
                for(const [key,{value,copy}] of entries){target[key]=value;if(copy)value.set(copy);}
            }
        }
    };
}
