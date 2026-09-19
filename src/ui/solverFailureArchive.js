const DATABASE = 'oet-solver-failures';

/** Append individual replays rather than rewriting a growing sessionStorage
 * string. One transaction keeps the count and latest report consistent, even
 * when more than one simulator tab writes to the same archive. */
export function createSolverFailureArchive({indexedDB=globalThis.indexedDB,databaseName=DATABASE}={}) {
    let opening=null;
    function database() {
        if(!indexedDB)return Promise.reject(new Error('IndexedDB jest niedostępne'));
        if(!opening)opening=new Promise((resolve,reject)=>{
            const request=indexedDB.open(databaseName,1);let rejected=false;
            const fail=error=>{rejected=true;opening=null;reject(error);};
            request.onupgradeneeded=()=>{
                request.result.createObjectStore('reports',{keyPath:'failure.id'});
                request.result.createObjectStore('summary');
            };
            request.onerror=()=>fail(request.error);
            request.onblocked=()=>fail(new Error('Archiwum jest zablokowane przez inną kartę'));
            request.onsuccess=()=>{
                const db=request.result;
                if(rejected){db.close();return;}
                db.onversionchange=()=>{db.close();opening=null;};resolve(db);
            };
        });
        return opening;
    }
    async function transaction(stores,mode,work) {
        const db=await database();
        return new Promise((resolve,reject)=>{
            const tx=db.transaction(stores,mode);let result;
            tx.oncomplete=()=>resolve(result);
            tx.onabort=()=>reject(tx.error??new Error('Zapis archiwum przerwany'));
            tx.onerror=()=>{}; // onabort owns failure; no partial commit is published.
            try{work(tx,value=>{result=value;});}catch(error){tx.abort();reject(error);}
        });
    }
    return {
        load:()=>transaction(['summary'],'readonly',(tx,done)=>{
            const req=tx.objectStore('summary').get('state');
            req.onsuccess=()=>done(req.result??{count:0,last:null});
        }),
        append:report=>transaction(['reports','summary'],'readwrite',(tx,done)=>{
            const reports=tx.objectStore('reports'),summary=tx.objectStore('summary');
            const existing=reports.get(report.failure.id);
            existing.onsuccess=()=>{
                const req=summary.get('state');
                req.onsuccess=()=>{
                    let value=req.result??{count:0,last:null};
                    if(!existing.result){
                        value={count:value.count+1,last:report};reports.put(report);summary.put(value,'state');
                    }
                    done(value);
                };
            };
        }),
        readAll:()=>transaction(['reports'],'readonly',(tx,done)=>{
            const req=tx.objectStore('reports').getAll();req.onsuccess=()=>done(req.result);
        }),
        clear:()=>transaction(['reports','summary'],'readwrite',(tx,done)=>{
            tx.objectStore('reports').clear();tx.objectStore('summary').clear();done();
        })
    };
}
