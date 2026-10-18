import {createSharedAxisWorkerEngine} from '../physics/kirchhoffSharedAxisWorkerEngine.js';
const engine=createSharedAxisWorkerEngine();
self.onmessage=({data:{epoch,id,operation,data}})=>{
    try {self.postMessage({epoch,id,value:engine.dispatch(operation,data)});}
    catch(error){self.postMessage({epoch,id,error:{message:error.message,stack:error.stack}});}
};
