import {pathToFileURL,fileURLToPath} from 'node:url';
// Includes the contrast snapshot requested by the synchronized commit; omitting
// that work hides the large Boolean-geometry stalls while contrast is visible.
const root=process.argv[2]??fileURLToPath(new URL('..',import.meta.url));
const {previewFixture}=await import(pathToFileURL(root+'/tests/helpers/stentGraftPreviewFixture.js'));
const rows=[];
for(let run=0;run<3;run++) {
    const {system,device}=previewFixture();const update=[],contrast=[];let faces=0;
    try {
        for(let i=0;i<210;i++) {
            const t=performance.now();
            system.updateAccess('right',1/20,null,{deviceId:device.id,release:'sheath'});
            const u=performance.now();
            const surface=system.getContrastSurface(1/20,true);
            const c=performance.now();
            if(i>=90){update.push(u-t);contrast.push(c-u);}
            faces=surface?.geometry?.attributes.position.count/3||0;
        }
        const summarize=values=>{const sorted=values.slice().sort((a,b)=>a-b);return {mean:values.reduce((a,b)=>a+b,0)/values.length,p95:sorted[Math.floor(.95*values.length)],max:sorted.at(-1)};};
        rows.push({run,steps:update.length,updateMs:summarize(update),contrastMs:summarize(contrast),totalMs:summarize(update.map((v,i)=>v+contrast[i])),faces});
    }finally{system.dispose();}
}
console.log(JSON.stringify(rows,null,2));
