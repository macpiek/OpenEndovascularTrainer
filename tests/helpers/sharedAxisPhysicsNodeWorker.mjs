import {parentPort} from 'node:worker_threads';
import {createSharedAxisWorkerEngine} from '../../src/physics/kirchhoffSharedAxisWorkerEngine.js';
const engine=createSharedAxisWorkerEngine();
parentPort.on('message',({epoch,id,operation,data})=>{
    try{parentPort.postMessage({epoch,id,value:engine.dispatch(operation,data)});}
    catch(error){parentPort.postMessage({epoch,id,error:{message:error.message,stack:error.stack}});}
});
