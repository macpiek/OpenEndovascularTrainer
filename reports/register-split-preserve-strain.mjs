// Test-only adapter: opt each existing split-World oracle fixture into the
// preserve-strain option without changing any oracle assertion or tolerance.
import { resolve } from 'node:path';
import { fileURLToPath, pathToFileURL } from 'node:url';
const root=resolve(process.env.OET_SPLIT_MOTION_SOURCE_ROOT??fileURLToPath(new URL('../',import.meta.url)));
const {EndovascularPhysicsWorld}=await import(pathToFileURL(resolve(root,'src/physics/endovascularPhysicsWorld.js')));
const {configureKirchhoffSplitBias}=await import(pathToFileURL(resolve(root,'src/physics/kirchhoffSplitMotion.js')));
const add=EndovascularPhysicsWorld.prototype.addContainment;
EndovascularPhysicsWorld.prototype.addContainment=function(...args) {
    const joint=add.apply(this,args);
    if(this.jointMotionMode==='split-physical-bias')configureKirchhoffSplitBias(joint,{materialMode:'preserve-strain'});
    return joint;
};
