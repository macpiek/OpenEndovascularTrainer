import {createCompositeJointTimeStepState} from './kirchhoffCompositeJointTimeStep.js';

const components=['x','y','z'];
function vector(value,name) {
    const result=components.map((key,i)=>value?.[key]??value?.[i]);
    if(!result.every(Number.isFinite))throw new TypeError(`${name} needs three finite coordinates`);
    return result;
}

/** One constant translation per application reset. The accepted native rod
 * stays in this frame for its whole lifetime; Float32 World publication is a
 * view and is never imported back into its physical pose or velocity history.
 * Anatomy queries still use the actual source provider and original BVH.
 */
export function createCompositeAppLocalFrame(world,coordinateOrigin) {
    const sheath=world.sheaths?.[0],origin=Object.freeze(vector(coordinateOrigin==='sheath-start'
        ?[sheath?.startX,sheath?.startY,sheath?.startZ]:coordinateOrigin,'Application coordinate origin'));
    const toLocal=p=>vector(p,'Global position').map((v,k)=>v-origin[k]);
    const toWorld=p=>vector(p,'Local position').map((v,k)=>v+origin[k]);
    const fieldViews=new WeakMap(),geometryViews=new WeakMap(),sheathViews=new WeakMap(),arrayViews=new WeakMap();
    function geometryView(source) {
        if(!source)return source;
        if(geometryViews.has(source))return geometryViews.get(source);
        const positions=new WeakMap(),trees=new WeakMap(),attributes=new WeakMap();let view;
        function positionView(p) {
            if(!p)return p;if(positions.has(p))return positions.get(p);
            // The original array/version remains the source provenance. Spatial
            // accessors expose translated vertices without making a new mesh.
            const methods=Object.fromEntries(['getX','getY','getZ'].map((key,k)=>[key,index=>p[key](index)-origin[k]]));
            const own=new Proxy(p,{get:(target,key)=>Object.hasOwn(methods,key)?methods[key]:Reflect.get(target,key,target)});
            positions.set(p,own);return own;
        }
        view=new Proxy(source,{get(target,key) {
            if(key==='attributes') {
                const sourceAttributes=target.attributes;if(!sourceAttributes)return sourceAttributes;
                if(!attributes.has(sourceAttributes))attributes.set(sourceAttributes,new Proxy(sourceAttributes,{get:(a,k)=>k==='position'?positionView(a.position):Reflect.get(a,k,a)}));
                return attributes.get(sourceAttributes);
            }
            if(key==='boundsTree') {
                const tree=target.boundsTree;if(!tree)return tree;
                if(!trees.has(tree))trees.set(tree,new Proxy(tree,{get:(t,k)=>k==='geometry'?(t.geometry===source?view:t.geometry):Reflect.get(t,k,t)}));
                return trees.get(tree);
            }
            return Reflect.get(target,key,target);
        }});geometryViews.set(source,view);return view;
    }
    function localContact(result) {
        // VesselContactField overwrites all these vectors on each query. Its
        // scalar distances, chosen triangle/sample and unit normals are kept.
        for(const key of ['point','target','closestPoint']) {
            const p=result?.[key]?.values;if(!p)continue;
            for(let k=0;k<3;k++)p[k]-=origin[k];
        }
        return result;
    }
    function fieldView(source) {
        if(!source)return source;if(fieldViews.has(source))return fieldViews.get(source);
        const methods=new Map(),vectors=new Map();
        const sdfGridCoordinates=(position,contact)=>{
            const sample=contact?.queryPoint;
            if(sample?.every(Number.isFinite)) {
                if(position.some((v,k)=>Math.abs(v-sample[k])>64*Number.EPSILON*Math.max(1,Math.abs(v),Math.abs(origin[k]))))
                    throw new RangeError('Original SDF sample does not belong to the current capsule');
                position=Array.from(sample);
            }
            return position.map((v,k)=>(v+origin[k]-source.sdfOrigin[k])/source.voxelSize);
        };
        const sdfFaceCoordinate=(axis,index)=>{
            if(!Number.isInteger(axis)||axis<0||axis>2||!Number.isSafeInteger(index))throw new RangeError('An SDF face needs an axis and integer grid index');
            // Form the actual provider plane first. Translating the grid
            // origin before adding index*h changes cancellation rounding.
            return (source.sdfOrigin[axis]+index*source.voxelSize)-origin[axis];
        };
        function method(key) {
            const original=source[key];if(typeof original!=='function')return original;
            const old=methods.get(key);if(old?.original===original)return old.value;
            let value;
            if(key==='certifyInsideBallCoordinates')value=(x,y,z,radius)=>original.call(source,x+origin[0],y+origin[1],z+origin[2],radius);
            else if(key==='querySphere')value=(position,radius,out)=>localContact(original.call(source,toWorld(position),radius,out));
            else if(key==='queryCapsuleCoordinates')value=(ax,ay,az,bx,by,bz,radius,out)=>localContact(original.call(source,
                ax+origin[0],ay+origin[1],az+origin[2],bx+origin[0],by+origin[1],bz+origin[2],radius,out));
            else if(key==='queryCapsule')value=(a,b,radius,out)=>localContact(original.call(source,toWorld(a),toWorld(b),radius,out));
            else value=original.bind(source);
            methods.set(key,{original,value});return value;
        }
        const view=new Proxy(source,{get(target,key) {
            if(key==='sdfGridCoordinates')return sdfGridCoordinates;
            if(key==='sdfFaceCoordinate')return sdfFaceCoordinate;
            if(key==='fallbackGeometry')return geometryView(target.fallbackGeometry);
            if(key==='sdfOrigin'||key==='broadPhaseOrigin') {
                const sourceVector=target[key];if(!sourceVector)return sourceVector;
                let local=vectors.get(key);if(!local){local=new Float64Array(3);vectors.set(key,local);}
                for(let k=0;k<3;k++)local[k]=sourceVector[k]-origin[k];return local;
            }
            const value=Reflect.get(target,key,target);return typeof value==='function'?method(key):value;
        }});fieldViews.set(source,view);return view;
    }
    function sheathView(source) {
        if(!sheathViews.has(source))sheathViews.set(source,new Proxy(source,{get:(target,key)=>{
            const k=['startX','startY','startZ'].indexOf(key);
            return k<0?Reflect.get(target,key,target):target[key]-origin[k];
        }}));return sheathViews.get(source);
    }
    const localWorld=new Proxy(world,{get(target,key) {
        if(key==='contactField')return fieldView(target.contactField);
        if(key==='sheaths') {
            const source=target.sheaths;if(!source)return source;
            let view=arrayViews.get(source);if(!view){view=[];arrayViews.set(source,view);}
            view.length=source.length;source.forEach((s,i)=>{view[i]=sheathView(s);});return view;
        }
        return Reflect.get(target,key,target);
    }});
    return Object.freeze({origin,world:localWorld,sourceWorld:world,toLocal,toWorld,
        initialState(state) {
            if(state.elasticityGeometry!=='native-discrete-rod'||state.continuousRateHistory!==undefined)
                throw new RangeError('Application local frame currently requires the native Joint state');
            if(['wallContactState','wallFrictionState','lumenContactState','lumenFrictionState','sheathContactState'].some(key=>state[key]!==undefined))
                throw new RangeError('Local origin must be installed at fresh initialization before contact history exists');
            const nativeRateHistory=structuredClone(state.nativeRateHistory);
            for(const tool of nativeRateHistory?.tools??[])for(const edge of tool.edges) {
                edge.positions=edge.positions.map(toLocal);
                if(edge.previousPositions)edge.previousPositions=edge.previousPositions.map(toLocal);
            }
            const local=createCompositeJointTimeStepState({...state,positions:state.positions.map(toLocal),nativeRateHistory});
            local.materialVelocities=structuredClone(state.materialVelocities);
            local.boundaryMultipliers=structuredClone(state.boundaryMultipliers);return local;
        },
        controls(input) {
            if(input.worldWallSurfacePosePaths!==undefined)
                throw new RangeError('Local native controls use own native rate history; external pose paths need an explicit frame adapter');
            return {...input,positionBoundaries:input.positionBoundaries?.map(row=>({...row,value:toLocal(row.value)}))};
        },
        wallConfiguration(config) {
            if(config===undefined)return undefined;
            return {...config,...(config.plane?{plane:{...config.plane,offset:config.plane.offset-config.plane.normal.reduce((sum,v,k)=>sum+v*origin[k],0)}}:{})};
        }
    });
}
