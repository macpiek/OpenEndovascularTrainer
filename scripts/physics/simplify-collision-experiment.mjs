/** Offline research mesh, in WORLD mm. Does not replace the shipped anatomy.
 * MESHOPT_MODULE=/absolute/path/meshopt_simplifier.js node ... OUTPUT.stl ERROR_MM
 * ErrorAbsolute is an approximate QEM metric, NOT a Hausdorff guarantee.
 * The bidirectional samples below are additional measurements, not a proof. */
import {readFileSync,writeFileSync} from 'node:fs';
import {createHash} from 'node:crypto';
import {pathToFileURL} from 'node:url';
import {BufferGeometry,BufferAttribute,Mesh,Vector3} from 'three';
import {STLLoader} from 'three/examples/jsm/loaders/STLLoader.js';
import {STLExporter} from 'three/examples/jsm/exporters/STLExporter.js';
import {MeshBVH} from 'three-mesh-bvh';
import {transformAortaGeometry} from '../../src/aortaTransform.js';
import {generateVessel} from '../../src/vesselGeometry.js';

const [output,errorText]=process.argv.slice(2),errorMm=Number(errorText);
if(!output||!(errorMm>0)||!process.env.MESHOPT_MODULE)throw new Error('Output, positive world-mm error and MESHOPT_MODULE required');
const moduleUrl=pathToFileURL(process.env.MESHOPT_MODULE);
const {MeshoptSimplifier:simplifier}=await import(moduleUrl.href);
const simplifierPackage=JSON.parse(readFileSync(new URL('./package.json',moduleUrl),'utf8'));
await simplifier.ready;
const source=readFileSync(new URL('../../res/Aorta_plain.stl',import.meta.url));
const original=new STLLoader().parse(source.buffer.slice(source.byteOffset,source.byteOffset+source.byteLength));
const transform=transformAortaGeometry(original,generateVessel(140,0).vessel);
const positions=original.attributes.position.array;
const weldMm=Number(process.env.WELD_MM??0);
if(!(weldMm>=0&&weldMm<=errorMm/10))throw new Error('Weld grid must be nonnegative and at most error/10');
let indices=simplifier.generatePositionRemap(positions,3);
const exactUniqueVertices=new Set(indices).size;
if(weldMm){
    const cells=new Map();indices=new Uint32Array(positions.length/3);
    for(let i=0;i<indices.length;i++){
        const key=[0,1,2].map(k=>Math.round(positions[3*i+k]/weldMm)).join('/');
        if(!cells.has(key))cells.set(key,i);
        indices[i]=cells.get(key);
    }
}
const weldedUniqueVertices=new Set(indices).size,filtered=[];
for(let i=0;i<indices.length;i+=3)if(indices[i]!==indices[i+1]&&indices[i]!==indices[i+2]&&indices[i+1]!==indices[i+2])filtered.push(indices[i],indices[i+1],indices[i+2]);
const removedDegenerateTriangles=(indices.length-filtered.length)/3;indices=new Uint32Array(filtered);
const [compactRemap,compactCount]=simplifier.compactMesh(indices),compactPositions=new Float32Array(compactCount*3);
for(let i=0;i<compactRemap.length;i++)if(compactRemap[i]!==0xffffffff)compactPositions.set(positions.subarray(i*3,i*3+3),compactRemap[i]*3);
const started=performance.now();
// Keep open boundaries and disconnected components; do not use Prune/Sloppy.
const chunkMm=Number(process.env.CHUNK_MM??0);
if(!(chunkMm>=0&&Number.isFinite(chunkMm)))throw new Error('Invalid chunk size');
let reduced,estimatedErrorMm,chunkCount=1;
if(!chunkMm)[reduced,estimatedErrorMm]=simplifier.simplify(indices,compactPositions,3,Math.floor(indices.length*.1/3)*3,errorMm,['LockBorder','ErrorAbsolute']);
else {
    // Local coordinates reduce cancellation in quadrics for the full, metre-
    // scale anatomy. LockBorder preserves shared edges between spatial chunks.
    const chunks=new Map();
    for(let i=0;i<indices.length;i+=3){
        const key=[0,1,2].map(k=>Math.floor((compactPositions[indices[i]*3+k]+compactPositions[indices[i+1]*3+k]+compactPositions[indices[i+2]*3+k])/(3*chunkMm))).join('/');
        if(!chunks.has(key))chunks.set(key,[]);
        chunks.get(key).push(indices[i],indices[i+1],indices[i+2]);
    }
    chunkCount=chunks.size;const combined=[];estimatedErrorMm=0;
    for(const list of chunks.values()){
        const localIndices=new Uint32Array(list),mapping=new Map(),inverse=[];
        for(let i=0;i<localIndices.length;i++){
            const global=localIndices[i];if(!mapping.has(global)){mapping.set(global,inverse.length);inverse.push(global);}
            localIndices[i]=mapping.get(global);
        }
        const localPositions=new Float32Array(inverse.length*3),origin=compactPositions.subarray(inverse[0]*3,inverse[0]*3+3);
        for(let i=0;i<inverse.length;i++)for(let k=0;k<3;k++)localPositions[i*3+k]=compactPositions[inverse[i]*3+k]-origin[k];
        const [result,error]=simplifier.simplify(localIndices,localPositions,3,Math.floor(localIndices.length*.1/3)*3,errorMm,['LockBorder','ErrorAbsolute']);
        estimatedErrorMm=Math.max(estimatedErrorMm,error);for(const i of result)combined.push(inverse[i]);
    }
    reduced=new Uint32Array(combined);
}
const simplifyMs=performance.now()-started;
const geometry=new BufferGeometry();geometry.setAttribute('position',new BufferAttribute(compactPositions,3));geometry.setIndex(new BufferAttribute(reduced,1));
const data=new STLExporter().parse(new Mesh(geometry),{binary:true});
writeFileSync(output,Buffer.from(data.buffer,data.byteOffset,data.byteLength));
original.boundsTree=new MeshBVH(original);geometry.boundsTree=new MeshBVH(geometry);
function surfaceSamples(from,to){
    const n=from.index.count/3,p=from.attributes.position,index=from.index;
    const point=new Vector3(),v=[new Vector3(),new Vector3(),new Vector3()],target={point:new Vector3()},distances=[];
    const started=performance.now();
    for(let f=0;f<n;f+=Math.max(1,Math.floor(n/20000))){
        for(let j=0;j<3;j++)v[j].fromBufferAttribute(p,index.getX(f*3+j));
        for(const weights of [[1,0,0],[0,1,0],[0,0,1],[.5,.5,0],[0,.5,.5],[.5,0,.5],[1/3,1/3,1/3]]){
            point.set(0,0,0);for(let j=0;j<3;j++)point.addScaledVector(v[j],weights[j]);
            distances.push(to.boundsTree.closestPointToPoint(point,target).distance);
        }
    }
    distances.sort((a,b)=>a-b);
    return {samples:distances.length,maxMm:distances.at(-1),p99Mm:distances[Math.floor(distances.length*.99)],rmsMm:Math.sqrt(distances.reduce((s,x)=>s+x*x,0)/distances.length),queryMs:performance.now()-started};
}
const report={generatorSha256:createHash('sha256').update(readFileSync(new URL(import.meta.url))).digest('hex'),simplifier:{name:simplifierPackage.name,version:simplifierPackage.version},sourceSha256:createHash('sha256').update(source).digest('hex'),meshSha256:createHash('sha256').update(readFileSync(output)).digest('hex'),coordinateSystem:'world-mm',transform,requestedErrorMm:errorMm,estimatedErrorMm,chunkMm,chunkCount,weldMm,exactUniqueVertices,weldedUniqueVertices,removedDegenerateTriangles,originalTriangles:positions.length/9,triangles:reduced.length/3,simplifyMs,originalToReduced:surfaceSamples(original,geometry),reducedToOriginal:surfaceSamples(geometry,original)};
writeFileSync(output+'.json',JSON.stringify(report,null,2));console.log(JSON.stringify(report,null,2));
