import {pathToFileURL,fileURLToPath} from 'node:url';
// CPU of device updates and geometry; excludes the rod Newton solve and GPU rendering.
const root=process.argv[2]??fileURLToPath(new URL('..',import.meta.url)),{previewFixture}=await import(pathToFileURL(root+'/tests/helpers/stentGraftPreviewFixture.js'));
const result=[];
function run(label,count,fn){const times=[];for(let i=0;i<count;i++){const t=performance.now();fn(i);times.push(performance.now()-t);}times.sort((a,b)=>a-b);result.push({label,n:count,mean:times.reduce((a,b)=>a+b,0)/count,p95:times[Math.floor(count*.95)]});}
for(const type of ['body','limb']) {
 const {system,device:d}=previewFixture('right',type,false);
 system.refreshDelivery('right');
 run(type+' insertion',90,i=>{system.updateAccess('right',1/60,null,{deviceId:d.id,mechanicalPosition:d.position+.15});system.refreshDelivery('right');});
 run(type+' insertion idle',60,()=>{system.refreshDelivery('right');});
 system.deploy('right');
 for(let quarter=0;quarter<4;quarter++)run(type+' release '+quarter,30,()=>{system.updateAccess('right',d.sheathTravel/12/120,null,{deviceId:d.id,release:'sheath'});system.refreshDelivery('right');});
 run(type+' release idle',60,()=>{system.updateAccess('right',1/60);system.refreshDelivery('right');});
 system.dispose();
}
console.log(JSON.stringify(result,null,2));
