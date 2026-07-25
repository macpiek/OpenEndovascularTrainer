const an="srgb",Ui="srgb-linear",Ih="display-p3",fc="display-p3-linear",jo="linear",ye="srgb",$o="rec709";const ou="300 es";class Tr{addEventListener(t,e){this._listeners===void 0&&(this._listeners={});const n=this._listeners;n[t]===void 0&&(n[t]=[]),n[t].indexOf(e)===-1&&n[t].push(e)}hasEventListener(t,e){if(this._listeners===void 0)return!1;const n=this._listeners;return n[t]!==void 0&&n[t].indexOf(e)!==-1}removeEventListener(t,e){if(this._listeners===void 0)return;const i=this._listeners[t];if(i!==void 0){const s=i.indexOf(e);s!==-1&&i.splice(s,1)}}dispatchEvent(t){if(this._listeners===void 0)return;const n=this._listeners[t.type];if(n!==void 0){t.target=this;const i=n.slice(0);for(let s=0,a=i.length;s<a;s++)i[s].call(this,t);t.target=null}}}const cn=["00","01","02","03","04","05","06","07","08","09","0a","0b","0c","0d","0e","0f","10","11","12","13","14","15","16","17","18","19","1a","1b","1c","1d","1e","1f","20","21","22","23","24","25","26","27","28","29","2a","2b","2c","2d","2e","2f","30","31","32","33","34","35","36","37","38","39","3a","3b","3c","3d","3e","3f","40","41","42","43","44","45","46","47","48","49","4a","4b","4c","4d","4e","4f","50","51","52","53","54","55","56","57","58","59","5a","5b","5c","5d","5e","5f","60","61","62","63","64","65","66","67","68","69","6a","6b","6c","6d","6e","6f","70","71","72","73","74","75","76","77","78","79","7a","7b","7c","7d","7e","7f","80","81","82","83","84","85","86","87","88","89","8a","8b","8c","8d","8e","8f","90","91","92","93","94","95","96","97","98","99","9a","9b","9c","9d","9e","9f","a0","a1","a2","a3","a4","a5","a6","a7","a8","a9","aa","ab","ac","ad","ae","af","b0","b1","b2","b3","b4","b5","b6","b7","b8","b9","ba","bb","bc","bd","be","bf","c0","c1","c2","c3","c4","c5","c6","c7","c8","c9","ca","cb","cc","cd","ce","cf","d0","d1","d2","d3","d4","d5","d6","d7","d8","d9","da","db","dc","dd","de","df","e0","e1","e2","e3","e4","e5","e6","e7","e8","e9","ea","eb","ec","ed","ee","ef","f0","f1","f2","f3","f4","f5","f6","f7","f8","f9","fa","fb","fc","fd","fe","ff"];let cu=1234567;const va=Math.PI/180,Pa=180/Math.PI;function Ar(){const r=Math.random()*4294967295|0,t=Math.random()*4294967295|0,e=Math.random()*4294967295|0,n=Math.random()*4294967295|0;return(cn[r&255]+cn[r>>8&255]+cn[r>>16&255]+cn[r>>24&255]+"-"+cn[t&255]+cn[t>>8&255]+"-"+cn[t>>16&15|64]+cn[t>>24&255]+"-"+cn[e&63|128]+cn[e>>8&255]+"-"+cn[e>>16&255]+cn[e>>24&255]+cn[n&255]+cn[n>>8&255]+cn[n>>16&255]+cn[n>>24&255]).toLowerCase()}function We(r,t,e){return Math.max(t,Math.min(e,r))}function Nh(r,t){return(r%t+t)%t}function um(r,t,e,n,i){return n+(r-t)*(i-n)/(e-t)}function dm(r,t,e){return r!==t?(e-r)/(t-r):0}function Sa(r,t,e){return(1-e)*r+e*t}function fm(r,t,e,n){return Sa(r,t,1-Math.exp(-e*n))}function pm(r,t=1){return t-Math.abs(Nh(r,t*2)-t)}function mm(r,t,e){return r<=t?0:r>=e?1:(r=(r-t)/(e-t),r*r*(3-2*r))}function gm(r,t,e){return r<=t?0:r>=e?1:(r=(r-t)/(e-t),r*r*r*(r*(r*6-15)+10))}function xm(r,t){return r+Math.floor(Math.random()*(t-r+1))}function _m(r,t){return r+Math.random()*(t-r)}function vm(r){return r*(.5-Math.random())}function Sm(r){r!==void 0&&(cu=r);let t=cu+=1831565813;return t=Math.imul(t^t>>>15,t|1),t^=t+Math.imul(t^t>>>7,t|61),((t^t>>>14)>>>0)/4294967296}function Mm(r){return r*va}function ym(r){return r*Pa}function th(r){return(r&r-1)===0&&r!==0}function Em(r){return Math.pow(2,Math.ceil(Math.log(r)/Math.LN2))}function Ko(r){return Math.pow(2,Math.floor(Math.log(r)/Math.LN2))}function wm(r,t,e,n,i){const s=Math.cos,a=Math.sin,o=s(e/2),c=a(e/2),l=s((t+n)/2),h=a((t+n)/2),u=s((t-n)/2),d=a((t-n)/2),f=s((n-t)/2),g=a((n-t)/2);switch(i){case"XYX":r.set(o*h,c*u,c*d,o*l);break;case"YZY":r.set(c*d,o*h,c*u,o*l);break;case"ZXZ":r.set(c*u,c*d,o*h,o*l);break;case"XZX":r.set(o*h,c*g,c*f,o*l);break;case"YXY":r.set(c*f,o*h,c*g,o*l);break;case"ZYZ":r.set(c*g,c*f,o*h,o*l);break;default:console.warn("THREE.MathUtils: .setQuaternionFromProperEuler() encountered an unknown order: "+i)}}function ar(r,t){switch(t.constructor){case Float32Array:return r;case Uint32Array:return r/4294967295;case Uint16Array:return r/65535;case Uint8Array:return r/255;case Int32Array:return Math.max(r/2147483647,-1);case Int16Array:return Math.max(r/32767,-1);case Int8Array:return Math.max(r/127,-1);default:throw new Error("Invalid component type.")}}function mn(r,t){switch(t.constructor){case Float32Array:return r;case Uint32Array:return Math.round(r*4294967295);case Uint16Array:return Math.round(r*65535);case Uint8Array:return Math.round(r*255);case Int32Array:return Math.round(r*2147483647);case Int16Array:return Math.round(r*32767);case Int8Array:return Math.round(r*127);default:throw new Error("Invalid component type.")}}const he={DEG2RAD:va,RAD2DEG:Pa,generateUUID:Ar,clamp:We,euclideanModulo:Nh,mapLinear:um,inverseLerp:dm,lerp:Sa,damp:fm,pingpong:pm,smoothstep:mm,smootherstep:gm,randInt:xm,randFloat:_m,randFloatSpread:vm,seededRandom:Sm,degToRad:Mm,radToDeg:ym,isPowerOfTwo:th,ceilPowerOfTwo:Em,floorPowerOfTwo:Ko,setQuaternionFromProperEuler:wm,normalize:mn,denormalize:ar};class Mt{constructor(t=0,e=0){Mt.prototype.isVector2=!0,this.x=t,this.y=e}get width(){return this.x}set width(t){this.x=t}get height(){return this.y}set height(t){this.y=t}set(t,e){return this.x=t,this.y=e,this}setScalar(t){return this.x=t,this.y=t,this}setX(t){return this.x=t,this}setY(t){return this.y=t,this}setComponent(t,e){switch(t){case 0:this.x=e;break;case 1:this.y=e;break;default:throw new Error("index is out of range: "+t)}return this}getComponent(t){switch(t){case 0:return this.x;case 1:return this.y;default:throw new Error("index is out of range: "+t)}}clone(){return new this.constructor(this.x,this.y)}copy(t){return this.x=t.x,this.y=t.y,this}add(t){return this.x+=t.x,this.y+=t.y,this}addScalar(t){return this.x+=t,this.y+=t,this}addVectors(t,e){return this.x=t.x+e.x,this.y=t.y+e.y,this}addScaledVector(t,e){return this.x+=t.x*e,this.y+=t.y*e,this}sub(t){return this.x-=t.x,this.y-=t.y,this}subScalar(t){return this.x-=t,this.y-=t,this}subVectors(t,e){return this.x=t.x-e.x,this.y=t.y-e.y,this}multiply(t){return this.x*=t.x,this.y*=t.y,this}multiplyScalar(t){return this.x*=t,this.y*=t,this}divide(t){return this.x/=t.x,this.y/=t.y,this}divideScalar(t){return this.multiplyScalar(1/t)}applyMatrix3(t){const e=this.x,n=this.y,i=t.elements;return this.x=i[0]*e+i[3]*n+i[6],this.y=i[1]*e+i[4]*n+i[7],this}min(t){return this.x=Math.min(this.x,t.x),this.y=Math.min(this.y,t.y),this}max(t){return this.x=Math.max(this.x,t.x),this.y=Math.max(this.y,t.y),this}clamp(t,e){return this.x=Math.max(t.x,Math.min(e.x,this.x)),this.y=Math.max(t.y,Math.min(e.y,this.y)),this}clampScalar(t,e){return this.x=Math.max(t,Math.min(e,this.x)),this.y=Math.max(t,Math.min(e,this.y)),this}clampLength(t,e){const n=this.length();return this.divideScalar(n||1).multiplyScalar(Math.max(t,Math.min(e,n)))}floor(){return this.x=Math.floor(this.x),this.y=Math.floor(this.y),this}ceil(){return this.x=Math.ceil(this.x),this.y=Math.ceil(this.y),this}round(){return this.x=Math.round(this.x),this.y=Math.round(this.y),this}roundToZero(){return this.x=Math.trunc(this.x),this.y=Math.trunc(this.y),this}negate(){return this.x=-this.x,this.y=-this.y,this}dot(t){return this.x*t.x+this.y*t.y}cross(t){return this.x*t.y-this.y*t.x}lengthSq(){return this.x*this.x+this.y*this.y}length(){return Math.sqrt(this.x*this.x+this.y*this.y)}manhattanLength(){return Math.abs(this.x)+Math.abs(this.y)}normalize(){return this.divideScalar(this.length()||1)}angle(){return Math.atan2(-this.y,-this.x)+Math.PI}angleTo(t){const e=Math.sqrt(this.lengthSq()*t.lengthSq());if(e===0)return Math.PI/2;const n=this.dot(t)/e;return Math.acos(We(n,-1,1))}distanceTo(t){return Math.sqrt(this.distanceToSquared(t))}distanceToSquared(t){const e=this.x-t.x,n=this.y-t.y;return e*e+n*n}manhattanDistanceTo(t){return Math.abs(this.x-t.x)+Math.abs(this.y-t.y)}setLength(t){return this.normalize().multiplyScalar(t)}lerp(t,e){return this.x+=(t.x-this.x)*e,this.y+=(t.y-this.y)*e,this}lerpVectors(t,e,n){return this.x=t.x+(e.x-t.x)*n,this.y=t.y+(e.y-t.y)*n,this}equals(t){return t.x===this.x&&t.y===this.y}fromArray(t,e=0){return this.x=t[e],this.y=t[e+1],this}toArray(t=[],e=0){return t[e]=this.x,t[e+1]=this.y,t}fromBufferAttribute(t,e){return this.x=t.getX(e),this.y=t.getY(e),this}rotateAround(t,e){const n=Math.cos(e),i=Math.sin(e),s=this.x-t.x,a=this.y-t.y;return this.x=s*n-a*i+t.x,this.y=s*i+a*n+t.y,this}random(){return this.x=Math.random(),this.y=Math.random(),this}*[Symbol.iterator](){yield this.x,yield this.y}}class ne{constructor(t,e,n,i,s,a,o,c,l){ne.prototype.isMatrix3=!0,this.elements=[1,0,0,0,1,0,0,0,1],t!==void 0&&this.set(t,e,n,i,s,a,o,c,l)}set(t,e,n,i,s,a,o,c,l){const h=this.elements;return h[0]=t,h[1]=i,h[2]=o,h[3]=e,h[4]=s,h[5]=c,h[6]=n,h[7]=a,h[8]=l,this}identity(){return this.set(1,0,0,0,1,0,0,0,1),this}copy(t){const e=this.elements,n=t.elements;return e[0]=n[0],e[1]=n[1],e[2]=n[2],e[3]=n[3],e[4]=n[4],e[5]=n[5],e[6]=n[6],e[7]=n[7],e[8]=n[8],this}extractBasis(t,e,n){return t.setFromMatrix3Column(this,0),e.setFromMatrix3Column(this,1),n.setFromMatrix3Column(this,2),this}setFromMatrix4(t){const e=t.elements;return this.set(e[0],e[4],e[8],e[1],e[5],e[9],e[2],e[6],e[10]),this}multiply(t){return this.multiplyMatrices(this,t)}premultiply(t){return this.multiplyMatrices(t,this)}multiplyMatrices(t,e){const n=t.elements,i=e.elements,s=this.elements,a=n[0],o=n[3],c=n[6],l=n[1],h=n[4],u=n[7],d=n[2],f=n[5],g=n[8],x=i[0],m=i[3],p=i[6],_=i[1],v=i[4],S=i[7],M=i[2],y=i[5],w=i[8];return s[0]=a*x+o*_+c*M,s[3]=a*m+o*v+c*y,s[6]=a*p+o*S+c*w,s[1]=l*x+h*_+u*M,s[4]=l*m+h*v+u*y,s[7]=l*p+h*S+u*w,s[2]=d*x+f*_+g*M,s[5]=d*m+f*v+g*y,s[8]=d*p+f*S+g*w,this}multiplyScalar(t){const e=this.elements;return e[0]*=t,e[3]*=t,e[6]*=t,e[1]*=t,e[4]*=t,e[7]*=t,e[2]*=t,e[5]*=t,e[8]*=t,this}determinant(){const t=this.elements,e=t[0],n=t[1],i=t[2],s=t[3],a=t[4],o=t[5],c=t[6],l=t[7],h=t[8];return e*a*h-e*o*l-n*s*h+n*o*c+i*s*l-i*a*c}invert(){const t=this.elements,e=t[0],n=t[1],i=t[2],s=t[3],a=t[4],o=t[5],c=t[6],l=t[7],h=t[8],u=h*a-o*l,d=o*c-h*s,f=l*s-a*c,g=e*u+n*d+i*f;if(g===0)return this.set(0,0,0,0,0,0,0,0,0);const x=1/g;return t[0]=u*x,t[1]=(i*l-h*n)*x,t[2]=(o*n-i*a)*x,t[3]=d*x,t[4]=(h*e-i*c)*x,t[5]=(i*s-o*e)*x,t[6]=f*x,t[7]=(n*c-l*e)*x,t[8]=(a*e-n*s)*x,this}transpose(){let t;const e=this.elements;return t=e[1],e[1]=e[3],e[3]=t,t=e[2],e[2]=e[6],e[6]=t,t=e[5],e[5]=e[7],e[7]=t,this}getNormalMatrix(t){return this.setFromMatrix4(t).invert().transpose()}transposeIntoArray(t){const e=this.elements;return t[0]=e[0],t[1]=e[3],t[2]=e[6],t[3]=e[1],t[4]=e[4],t[5]=e[7],t[6]=e[2],t[7]=e[5],t[8]=e[8],this}setUvTransform(t,e,n,i,s,a,o){const c=Math.cos(s),l=Math.sin(s);return this.set(n*c,n*l,-n*(c*a+l*o)+a+t,-i*l,i*c,-i*(-l*a+c*o)+o+e,0,0,1),this}scale(t,e){return this.premultiply(bc.makeScale(t,e)),this}rotate(t){return this.premultiply(bc.makeRotation(-t)),this}translate(t,e){return this.premultiply(bc.makeTranslation(t,e)),this}makeTranslation(t,e){return t.isVector2?this.set(1,0,t.x,0,1,t.y,0,0,1):this.set(1,0,t,0,1,e,0,0,1),this}makeRotation(t){const e=Math.cos(t),n=Math.sin(t);return this.set(e,-n,0,n,e,0,0,0,1),this}makeScale(t,e){return this.set(t,0,0,0,e,0,0,0,1),this}equals(t){const e=this.elements,n=t.elements;for(let i=0;i<9;i++)if(e[i]!==n[i])return!1;return!0}fromArray(t,e=0){for(let n=0;n<9;n++)this.elements[n]=t[n+e];return this}toArray(t=[],e=0){const n=this.elements;return t[e]=n[0],t[e+1]=n[1],t[e+2]=n[2],t[e+3]=n[3],t[e+4]=n[4],t[e+5]=n[5],t[e+6]=n[6],t[e+7]=n[7],t[e+8]=n[8],t}clone(){return new this.constructor().fromArray(this.elements)}}const bc=new ne;function Bf(r){for(let t=r.length-1;t>=0;--t)if(r[t]>=65535)return!0;return!1}function Jo(r){return document.createElementNS("http://www.w3.org/1999/xhtml",r)}function Tm(){const r=Jo("canvas");return r.style.display="block",r}const lu={};function Ma(r){r in lu||(lu[r]=!0,console.warn(r))}const hu=new ne().set(.8224621,.177538,0,.0331941,.9668058,0,.0170827,.0723974,.9105199),uu=new ne().set(1.2249401,-.2249404,0,-.0420569,1.0420571,0,-.0196376,-.0786361,1.0982735),Ua={[Ui]:{transfer:jo,primaries:$o,toReference:r=>r,fromReference:r=>r},[an]:{transfer:ye,primaries:$o,toReference:r=>r.convertSRGBToLinear(),fromReference:r=>r.convertLinearToSRGB()},[fc]:{transfer:jo,primaries:"p3",toReference:r=>r.applyMatrix3(uu),fromReference:r=>r.applyMatrix3(hu)},[Ih]:{transfer:ye,primaries:"p3",toReference:r=>r.convertSRGBToLinear().applyMatrix3(uu),fromReference:r=>r.applyMatrix3(hu).convertLinearToSRGB()}},Am=new Set([Ui,fc]),pe={enabled:!0,_workingColorSpace:Ui,get workingColorSpace(){return this._workingColorSpace},set workingColorSpace(r){if(!Am.has(r))throw new Error(`Unsupported working color space, "${r}".`);this._workingColorSpace=r},convert:function(r,t,e){if(this.enabled===!1||t===e||!t||!e)return r;const n=Ua[t].toReference,i=Ua[e].fromReference;return i(n(r))},fromWorkingColorSpace:function(r,t){return this.convert(r,this._workingColorSpace,t)},toWorkingColorSpace:function(r,t){return this.convert(r,t,this._workingColorSpace)},getPrimaries:function(r){return Ua[r].primaries},getTransfer:function(r){return r===""?jo:Ua[r].transfer}};function mr(r){return r<.04045?r*.0773993808:Math.pow(r*.9478672986+.0521327014,2.4)}function Pc(r){return r<.0031308?r*12.92:1.055*Math.pow(r,.41666)-.055}let Is;class Uf{static getDataURL(t){if(/^data:/i.test(t.src)||typeof HTMLCanvasElement>"u")return t.src;let e;if(t instanceof HTMLCanvasElement)e=t;else{Is===void 0&&(Is=Jo("canvas")),Is.width=t.width,Is.height=t.height;const n=Is.getContext("2d");t instanceof ImageData?n.putImageData(t,0,0):n.drawImage(t,0,0,t.width,t.height),e=Is}return e.width>2048||e.height>2048?(console.warn("THREE.ImageUtils.getDataURL: Image converted to jpg for performance reasons",t),e.toDataURL("image/jpeg",.6)):e.toDataURL("image/png")}static sRGBToLinear(t){if(typeof HTMLImageElement<"u"&&t instanceof HTMLImageElement||typeof HTMLCanvasElement<"u"&&t instanceof HTMLCanvasElement||typeof ImageBitmap<"u"&&t instanceof ImageBitmap){const e=Jo("canvas");e.width=t.width,e.height=t.height;const n=e.getContext("2d");n.drawImage(t,0,0,t.width,t.height);const i=n.getImageData(0,0,t.width,t.height),s=i.data;for(let a=0;a<s.length;a++)s[a]=mr(s[a]/255)*255;return n.putImageData(i,0,0),e}else if(t.data){const e=t.data.slice(0);for(let n=0;n<e.length;n++)e instanceof Uint8Array||e instanceof Uint8ClampedArray?e[n]=Math.floor(mr(e[n]/255)*255):e[n]=mr(e[n]);return{data:e,width:t.width,height:t.height}}else return console.warn("THREE.ImageUtils.sRGBToLinear(): Unsupported image type. No color space conversion applied."),t}}let Cm=0;class zf{constructor(t=null){this.isSource=!0,Object.defineProperty(this,"id",{value:Cm++}),this.uuid=Ar(),this.data=t,this.version=0}set needsUpdate(t){t===!0&&this.version++}toJSON(t){const e=t===void 0||typeof t=="string";if(!e&&t.images[this.uuid]!==void 0)return t.images[this.uuid];const n={uuid:this.uuid,url:""},i=this.data;if(i!==null){let s;if(Array.isArray(i)){s=[];for(let a=0,o=i.length;a<o;a++)i[a].isDataTexture?s.push(Lc(i[a].image)):s.push(Lc(i[a]))}else s=Lc(i);n.url=s}return e||(t.images[this.uuid]=n),n}}function Lc(r){return typeof HTMLImageElement<"u"&&r instanceof HTMLImageElement||typeof HTMLCanvasElement<"u"&&r instanceof HTMLCanvasElement||typeof ImageBitmap<"u"&&r instanceof ImageBitmap?Uf.getDataURL(r):r.data?{data:Array.from(r.data),width:r.width,height:r.height,type:r.data.constructor.name}:(console.warn("THREE.Texture: Unable to serialize Texture."),{})}let Rm=0;class Fn extends Tr{constructor(t=Fn.DEFAULT_IMAGE,e=Fn.DEFAULT_MAPPING,n=1001,i=1001,s=1006,a=1008,o=1023,c=1009,l=Fn.DEFAULT_ANISOTROPY,h=""){super(),this.isTexture=!0,Object.defineProperty(this,"id",{value:Rm++}),this.uuid=Ar(),this.name="",this.source=new zf(t),this.mipmaps=[],this.mapping=e,this.channel=0,this.wrapS=n,this.wrapT=i,this.magFilter=s,this.minFilter=a,this.anisotropy=l,this.format=o,this.internalFormat=null,this.type=c,this.offset=new Mt(0,0),this.repeat=new Mt(1,1),this.center=new Mt(0,0),this.rotation=0,this.matrixAutoUpdate=!0,this.matrix=new ne,this.generateMipmaps=!0,this.premultiplyAlpha=!1,this.flipY=!0,this.unpackAlignment=4,typeof h=="string"?this.colorSpace=h:(Ma("THREE.Texture: Property .encoding has been replaced by .colorSpace."),this.colorSpace=h===3001?an:""),this.userData={},this.version=0,this.onUpdate=null,this.isRenderTargetTexture=!1,this.needsPMREMUpdate=!1}get image(){return this.source.data}set image(t=null){this.source.data=t}updateMatrix(){this.matrix.setUvTransform(this.offset.x,this.offset.y,this.repeat.x,this.repeat.y,this.rotation,this.center.x,this.center.y)}clone(){return new this.constructor().copy(this)}copy(t){return this.name=t.name,this.source=t.source,this.mipmaps=t.mipmaps.slice(0),this.mapping=t.mapping,this.channel=t.channel,this.wrapS=t.wrapS,this.wrapT=t.wrapT,this.magFilter=t.magFilter,this.minFilter=t.minFilter,this.anisotropy=t.anisotropy,this.format=t.format,this.internalFormat=t.internalFormat,this.type=t.type,this.offset.copy(t.offset),this.repeat.copy(t.repeat),this.center.copy(t.center),this.rotation=t.rotation,this.matrixAutoUpdate=t.matrixAutoUpdate,this.matrix.copy(t.matrix),this.generateMipmaps=t.generateMipmaps,this.premultiplyAlpha=t.premultiplyAlpha,this.flipY=t.flipY,this.unpackAlignment=t.unpackAlignment,this.colorSpace=t.colorSpace,this.userData=JSON.parse(JSON.stringify(t.userData)),this.needsUpdate=!0,this}toJSON(t){const e=t===void 0||typeof t=="string";if(!e&&t.textures[this.uuid]!==void 0)return t.textures[this.uuid];const n={metadata:{version:4.6,type:"Texture",generator:"Texture.toJSON"},uuid:this.uuid,name:this.name,image:this.source.toJSON(t).uuid,mapping:this.mapping,channel:this.channel,repeat:[this.repeat.x,this.repeat.y],offset:[this.offset.x,this.offset.y],center:[this.center.x,this.center.y],rotation:this.rotation,wrap:[this.wrapS,this.wrapT],format:this.format,internalFormat:this.internalFormat,type:this.type,colorSpace:this.colorSpace,minFilter:this.minFilter,magFilter:this.magFilter,anisotropy:this.anisotropy,flipY:this.flipY,generateMipmaps:this.generateMipmaps,premultiplyAlpha:this.premultiplyAlpha,unpackAlignment:this.unpackAlignment};return Object.keys(this.userData).length>0&&(n.userData=this.userData),e||(t.textures[this.uuid]=n),n}dispose(){this.dispatchEvent({type:"dispose"})}transformUv(t){if(this.mapping!==300)return t;if(t.applyMatrix3(this.matrix),t.x<0||t.x>1)switch(this.wrapS){case 1e3:t.x=t.x-Math.floor(t.x);break;case 1001:t.x=t.x<0?0:1;break;case 1002:Math.abs(Math.floor(t.x)%2)===1?t.x=Math.ceil(t.x)-t.x:t.x=t.x-Math.floor(t.x);break}if(t.y<0||t.y>1)switch(this.wrapT){case 1e3:t.y=t.y-Math.floor(t.y);break;case 1001:t.y=t.y<0?0:1;break;case 1002:Math.abs(Math.floor(t.y)%2)===1?t.y=Math.ceil(t.y)-t.y:t.y=t.y-Math.floor(t.y);break}return this.flipY&&(t.y=1-t.y),t}set needsUpdate(t){t===!0&&(this.version++,this.source.needsUpdate=!0)}get encoding(){return Ma("THREE.Texture: Property .encoding has been replaced by .colorSpace."),this.colorSpace===an?3001:3e3}set encoding(t){Ma("THREE.Texture: Property .encoding has been replaced by .colorSpace."),this.colorSpace=t===3001?an:""}}Fn.DEFAULT_IMAGE=null;Fn.DEFAULT_MAPPING=300;Fn.DEFAULT_ANISOTROPY=1;class en{constructor(t=0,e=0,n=0,i=1){en.prototype.isVector4=!0,this.x=t,this.y=e,this.z=n,this.w=i}get width(){return this.z}set width(t){this.z=t}get height(){return this.w}set height(t){this.w=t}set(t,e,n,i){return this.x=t,this.y=e,this.z=n,this.w=i,this}setScalar(t){return this.x=t,this.y=t,this.z=t,this.w=t,this}setX(t){return this.x=t,this}setY(t){return this.y=t,this}setZ(t){return this.z=t,this}setW(t){return this.w=t,this}setComponent(t,e){switch(t){case 0:this.x=e;break;case 1:this.y=e;break;case 2:this.z=e;break;case 3:this.w=e;break;default:throw new Error("index is out of range: "+t)}return this}getComponent(t){switch(t){case 0:return this.x;case 1:return this.y;case 2:return this.z;case 3:return this.w;default:throw new Error("index is out of range: "+t)}}clone(){return new this.constructor(this.x,this.y,this.z,this.w)}copy(t){return this.x=t.x,this.y=t.y,this.z=t.z,this.w=t.w!==void 0?t.w:1,this}add(t){return this.x+=t.x,this.y+=t.y,this.z+=t.z,this.w+=t.w,this}addScalar(t){return this.x+=t,this.y+=t,this.z+=t,this.w+=t,this}addVectors(t,e){return this.x=t.x+e.x,this.y=t.y+e.y,this.z=t.z+e.z,this.w=t.w+e.w,this}addScaledVector(t,e){return this.x+=t.x*e,this.y+=t.y*e,this.z+=t.z*e,this.w+=t.w*e,this}sub(t){return this.x-=t.x,this.y-=t.y,this.z-=t.z,this.w-=t.w,this}subScalar(t){return this.x-=t,this.y-=t,this.z-=t,this.w-=t,this}subVectors(t,e){return this.x=t.x-e.x,this.y=t.y-e.y,this.z=t.z-e.z,this.w=t.w-e.w,this}multiply(t){return this.x*=t.x,this.y*=t.y,this.z*=t.z,this.w*=t.w,this}multiplyScalar(t){return this.x*=t,this.y*=t,this.z*=t,this.w*=t,this}applyMatrix4(t){const e=this.x,n=this.y,i=this.z,s=this.w,a=t.elements;return this.x=a[0]*e+a[4]*n+a[8]*i+a[12]*s,this.y=a[1]*e+a[5]*n+a[9]*i+a[13]*s,this.z=a[2]*e+a[6]*n+a[10]*i+a[14]*s,this.w=a[3]*e+a[7]*n+a[11]*i+a[15]*s,this}divideScalar(t){return this.multiplyScalar(1/t)}setAxisAngleFromQuaternion(t){this.w=2*Math.acos(t.w);const e=Math.sqrt(1-t.w*t.w);return e<1e-4?(this.x=1,this.y=0,this.z=0):(this.x=t.x/e,this.y=t.y/e,this.z=t.z/e),this}setAxisAngleFromRotationMatrix(t){let e,n,i,s;const c=t.elements,l=c[0],h=c[4],u=c[8],d=c[1],f=c[5],g=c[9],x=c[2],m=c[6],p=c[10];if(Math.abs(h-d)<.01&&Math.abs(u-x)<.01&&Math.abs(g-m)<.01){if(Math.abs(h+d)<.1&&Math.abs(u+x)<.1&&Math.abs(g+m)<.1&&Math.abs(l+f+p-3)<.1)return this.set(1,0,0,0),this;e=Math.PI;const v=(l+1)/2,S=(f+1)/2,M=(p+1)/2,y=(h+d)/4,w=(u+x)/4,A=(g+m)/4;return v>S&&v>M?v<.01?(n=0,i=.707106781,s=.707106781):(n=Math.sqrt(v),i=y/n,s=w/n):S>M?S<.01?(n=.707106781,i=0,s=.707106781):(i=Math.sqrt(S),n=y/i,s=A/i):M<.01?(n=.707106781,i=.707106781,s=0):(s=Math.sqrt(M),n=w/s,i=A/s),this.set(n,i,s,e),this}let _=Math.sqrt((m-g)*(m-g)+(u-x)*(u-x)+(d-h)*(d-h));return Math.abs(_)<.001&&(_=1),this.x=(m-g)/_,this.y=(u-x)/_,this.z=(d-h)/_,this.w=Math.acos((l+f+p-1)/2),this}min(t){return this.x=Math.min(this.x,t.x),this.y=Math.min(this.y,t.y),this.z=Math.min(this.z,t.z),this.w=Math.min(this.w,t.w),this}max(t){return this.x=Math.max(this.x,t.x),this.y=Math.max(this.y,t.y),this.z=Math.max(this.z,t.z),this.w=Math.max(this.w,t.w),this}clamp(t,e){return this.x=Math.max(t.x,Math.min(e.x,this.x)),this.y=Math.max(t.y,Math.min(e.y,this.y)),this.z=Math.max(t.z,Math.min(e.z,this.z)),this.w=Math.max(t.w,Math.min(e.w,this.w)),this}clampScalar(t,e){return this.x=Math.max(t,Math.min(e,this.x)),this.y=Math.max(t,Math.min(e,this.y)),this.z=Math.max(t,Math.min(e,this.z)),this.w=Math.max(t,Math.min(e,this.w)),this}clampLength(t,e){const n=this.length();return this.divideScalar(n||1).multiplyScalar(Math.max(t,Math.min(e,n)))}floor(){return this.x=Math.floor(this.x),this.y=Math.floor(this.y),this.z=Math.floor(this.z),this.w=Math.floor(this.w),this}ceil(){return this.x=Math.ceil(this.x),this.y=Math.ceil(this.y),this.z=Math.ceil(this.z),this.w=Math.ceil(this.w),this}round(){return this.x=Math.round(this.x),this.y=Math.round(this.y),this.z=Math.round(this.z),this.w=Math.round(this.w),this}roundToZero(){return this.x=Math.trunc(this.x),this.y=Math.trunc(this.y),this.z=Math.trunc(this.z),this.w=Math.trunc(this.w),this}negate(){return this.x=-this.x,this.y=-this.y,this.z=-this.z,this.w=-this.w,this}dot(t){return this.x*t.x+this.y*t.y+this.z*t.z+this.w*t.w}lengthSq(){return this.x*this.x+this.y*this.y+this.z*this.z+this.w*this.w}length(){return Math.sqrt(this.x*this.x+this.y*this.y+this.z*this.z+this.w*this.w)}manhattanLength(){return Math.abs(this.x)+Math.abs(this.y)+Math.abs(this.z)+Math.abs(this.w)}normalize(){return this.divideScalar(this.length()||1)}setLength(t){return this.normalize().multiplyScalar(t)}lerp(t,e){return this.x+=(t.x-this.x)*e,this.y+=(t.y-this.y)*e,this.z+=(t.z-this.z)*e,this.w+=(t.w-this.w)*e,this}lerpVectors(t,e,n){return this.x=t.x+(e.x-t.x)*n,this.y=t.y+(e.y-t.y)*n,this.z=t.z+(e.z-t.z)*n,this.w=t.w+(e.w-t.w)*n,this}equals(t){return t.x===this.x&&t.y===this.y&&t.z===this.z&&t.w===this.w}fromArray(t,e=0){return this.x=t[e],this.y=t[e+1],this.z=t[e+2],this.w=t[e+3],this}toArray(t=[],e=0){return t[e]=this.x,t[e+1]=this.y,t[e+2]=this.z,t[e+3]=this.w,t}fromBufferAttribute(t,e){return this.x=t.getX(e),this.y=t.getY(e),this.z=t.getZ(e),this.w=t.getW(e),this}random(){return this.x=Math.random(),this.y=Math.random(),this.z=Math.random(),this.w=Math.random(),this}*[Symbol.iterator](){yield this.x,yield this.y,yield this.z,yield this.w}}class bm extends Tr{constructor(t=1,e=1,n={}){super(),this.isRenderTarget=!0,this.width=t,this.height=e,this.depth=1,this.scissor=new en(0,0,t,e),this.scissorTest=!1,this.viewport=new en(0,0,t,e);const i={width:t,height:e,depth:1};n.encoding!==void 0&&(Ma("THREE.WebGLRenderTarget: option.encoding has been replaced by option.colorSpace."),n.colorSpace=n.encoding===3001?an:""),n=Object.assign({generateMipmaps:!1,internalFormat:null,minFilter:1006,depthBuffer:!0,stencilBuffer:!1,depthTexture:null,samples:0},n),this.texture=new Fn(i,n.mapping,n.wrapS,n.wrapT,n.magFilter,n.minFilter,n.format,n.type,n.anisotropy,n.colorSpace),this.texture.isRenderTargetTexture=!0,this.texture.flipY=!1,this.texture.generateMipmaps=n.generateMipmaps,this.texture.internalFormat=n.internalFormat,this.depthBuffer=n.depthBuffer,this.stencilBuffer=n.stencilBuffer,this.depthTexture=n.depthTexture,this.samples=n.samples}setSize(t,e,n=1){(this.width!==t||this.height!==e||this.depth!==n)&&(this.width=t,this.height=e,this.depth=n,this.texture.image.width=t,this.texture.image.height=e,this.texture.image.depth=n,this.dispose()),this.viewport.set(0,0,t,e),this.scissor.set(0,0,t,e)}clone(){return new this.constructor().copy(this)}copy(t){this.width=t.width,this.height=t.height,this.depth=t.depth,this.scissor.copy(t.scissor),this.scissorTest=t.scissorTest,this.viewport.copy(t.viewport),this.texture=t.texture.clone(),this.texture.isRenderTargetTexture=!0;const e=Object.assign({},t.texture.image);return this.texture.source=new zf(e),this.depthBuffer=t.depthBuffer,this.stencilBuffer=t.stencilBuffer,t.depthTexture!==null&&(this.depthTexture=t.depthTexture.clone()),this.samples=t.samples,this}dispose(){this.dispatchEvent({type:"dispose"})}}class sn extends bm{constructor(t=1,e=1,n={}){super(t,e,n),this.isWebGLRenderTarget=!0}}class Of extends Fn{constructor(t=null,e=1,n=1,i=1){super(null),this.isDataArrayTexture=!0,this.image={data:t,width:e,height:n,depth:i},this.magFilter=1003,this.minFilter=1003,this.wrapR=1001,this.generateMipmaps=!1,this.flipY=!1,this.unpackAlignment=1}}class Pm extends Fn{constructor(t=null,e=1,n=1,i=1){super(null),this.isData3DTexture=!0,this.image={data:t,width:e,height:n,depth:i},this.magFilter=1003,this.minFilter=1003,this.wrapR=1001,this.generateMipmaps=!1,this.flipY=!1,this.unpackAlignment=1}}class ni{constructor(t=0,e=0,n=0,i=1){this.isQuaternion=!0,this._x=t,this._y=e,this._z=n,this._w=i}static slerpFlat(t,e,n,i,s,a,o){let c=n[i+0],l=n[i+1],h=n[i+2],u=n[i+3];const d=s[a+0],f=s[a+1],g=s[a+2],x=s[a+3];if(o===0){t[e+0]=c,t[e+1]=l,t[e+2]=h,t[e+3]=u;return}if(o===1){t[e+0]=d,t[e+1]=f,t[e+2]=g,t[e+3]=x;return}if(u!==x||c!==d||l!==f||h!==g){let m=1-o;const p=c*d+l*f+h*g+u*x,_=p>=0?1:-1,v=1-p*p;if(v>Number.EPSILON){const M=Math.sqrt(v),y=Math.atan2(M,p*_);m=Math.sin(m*y)/M,o=Math.sin(o*y)/M}const S=o*_;if(c=c*m+d*S,l=l*m+f*S,h=h*m+g*S,u=u*m+x*S,m===1-o){const M=1/Math.sqrt(c*c+l*l+h*h+u*u);c*=M,l*=M,h*=M,u*=M}}t[e]=c,t[e+1]=l,t[e+2]=h,t[e+3]=u}static multiplyQuaternionsFlat(t,e,n,i,s,a){const o=n[i],c=n[i+1],l=n[i+2],h=n[i+3],u=s[a],d=s[a+1],f=s[a+2],g=s[a+3];return t[e]=o*g+h*u+c*f-l*d,t[e+1]=c*g+h*d+l*u-o*f,t[e+2]=l*g+h*f+o*d-c*u,t[e+3]=h*g-o*u-c*d-l*f,t}get x(){return this._x}set x(t){this._x=t,this._onChangeCallback()}get y(){return this._y}set y(t){this._y=t,this._onChangeCallback()}get z(){return this._z}set z(t){this._z=t,this._onChangeCallback()}get w(){return this._w}set w(t){this._w=t,this._onChangeCallback()}set(t,e,n,i){return this._x=t,this._y=e,this._z=n,this._w=i,this._onChangeCallback(),this}clone(){return new this.constructor(this._x,this._y,this._z,this._w)}copy(t){return this._x=t.x,this._y=t.y,this._z=t.z,this._w=t.w,this._onChangeCallback(),this}setFromEuler(t,e=!0){const n=t._x,i=t._y,s=t._z,a=t._order,o=Math.cos,c=Math.sin,l=o(n/2),h=o(i/2),u=o(s/2),d=c(n/2),f=c(i/2),g=c(s/2);switch(a){case"XYZ":this._x=d*h*u+l*f*g,this._y=l*f*u-d*h*g,this._z=l*h*g+d*f*u,this._w=l*h*u-d*f*g;break;case"YXZ":this._x=d*h*u+l*f*g,this._y=l*f*u-d*h*g,this._z=l*h*g-d*f*u,this._w=l*h*u+d*f*g;break;case"ZXY":this._x=d*h*u-l*f*g,this._y=l*f*u+d*h*g,this._z=l*h*g+d*f*u,this._w=l*h*u-d*f*g;break;case"ZYX":this._x=d*h*u-l*f*g,this._y=l*f*u+d*h*g,this._z=l*h*g-d*f*u,this._w=l*h*u+d*f*g;break;case"YZX":this._x=d*h*u+l*f*g,this._y=l*f*u+d*h*g,this._z=l*h*g-d*f*u,this._w=l*h*u-d*f*g;break;case"XZY":this._x=d*h*u-l*f*g,this._y=l*f*u-d*h*g,this._z=l*h*g+d*f*u,this._w=l*h*u+d*f*g;break;default:console.warn("THREE.Quaternion: .setFromEuler() encountered an unknown order: "+a)}return e===!0&&this._onChangeCallback(),this}setFromAxisAngle(t,e){const n=e/2,i=Math.sin(n);return this._x=t.x*i,this._y=t.y*i,this._z=t.z*i,this._w=Math.cos(n),this._onChangeCallback(),this}setFromRotationMatrix(t){const e=t.elements,n=e[0],i=e[4],s=e[8],a=e[1],o=e[5],c=e[9],l=e[2],h=e[6],u=e[10],d=n+o+u;if(d>0){const f=.5/Math.sqrt(d+1);this._w=.25/f,this._x=(h-c)*f,this._y=(s-l)*f,this._z=(a-i)*f}else if(n>o&&n>u){const f=2*Math.sqrt(1+n-o-u);this._w=(h-c)/f,this._x=.25*f,this._y=(i+a)/f,this._z=(s+l)/f}else if(o>u){const f=2*Math.sqrt(1+o-n-u);this._w=(s-l)/f,this._x=(i+a)/f,this._y=.25*f,this._z=(c+h)/f}else{const f=2*Math.sqrt(1+u-n-o);this._w=(a-i)/f,this._x=(s+l)/f,this._y=(c+h)/f,this._z=.25*f}return this._onChangeCallback(),this}setFromUnitVectors(t,e){let n=t.dot(e)+1;return n<Number.EPSILON?(n=0,Math.abs(t.x)>Math.abs(t.z)?(this._x=-t.y,this._y=t.x,this._z=0,this._w=n):(this._x=0,this._y=-t.z,this._z=t.y,this._w=n)):(this._x=t.y*e.z-t.z*e.y,this._y=t.z*e.x-t.x*e.z,this._z=t.x*e.y-t.y*e.x,this._w=n),this.normalize()}angleTo(t){return 2*Math.acos(Math.abs(We(this.dot(t),-1,1)))}rotateTowards(t,e){const n=this.angleTo(t);if(n===0)return this;const i=Math.min(1,e/n);return this.slerp(t,i),this}identity(){return this.set(0,0,0,1)}invert(){return this.conjugate()}conjugate(){return this._x*=-1,this._y*=-1,this._z*=-1,this._onChangeCallback(),this}dot(t){return this._x*t._x+this._y*t._y+this._z*t._z+this._w*t._w}lengthSq(){return this._x*this._x+this._y*this._y+this._z*this._z+this._w*this._w}length(){return Math.sqrt(this._x*this._x+this._y*this._y+this._z*this._z+this._w*this._w)}normalize(){let t=this.length();return t===0?(this._x=0,this._y=0,this._z=0,this._w=1):(t=1/t,this._x=this._x*t,this._y=this._y*t,this._z=this._z*t,this._w=this._w*t),this._onChangeCallback(),this}multiply(t){return this.multiplyQuaternions(this,t)}premultiply(t){return this.multiplyQuaternions(t,this)}multiplyQuaternions(t,e){const n=t._x,i=t._y,s=t._z,a=t._w,o=e._x,c=e._y,l=e._z,h=e._w;return this._x=n*h+a*o+i*l-s*c,this._y=i*h+a*c+s*o-n*l,this._z=s*h+a*l+n*c-i*o,this._w=a*h-n*o-i*c-s*l,this._onChangeCallback(),this}slerp(t,e){if(e===0)return this;if(e===1)return this.copy(t);const n=this._x,i=this._y,s=this._z,a=this._w;let o=a*t._w+n*t._x+i*t._y+s*t._z;if(o<0?(this._w=-t._w,this._x=-t._x,this._y=-t._y,this._z=-t._z,o=-o):this.copy(t),o>=1)return this._w=a,this._x=n,this._y=i,this._z=s,this;const c=1-o*o;if(c<=Number.EPSILON){const f=1-e;return this._w=f*a+e*this._w,this._x=f*n+e*this._x,this._y=f*i+e*this._y,this._z=f*s+e*this._z,this.normalize(),this}const l=Math.sqrt(c),h=Math.atan2(l,o),u=Math.sin((1-e)*h)/l,d=Math.sin(e*h)/l;return this._w=a*u+this._w*d,this._x=n*u+this._x*d,this._y=i*u+this._y*d,this._z=s*u+this._z*d,this._onChangeCallback(),this}slerpQuaternions(t,e,n){return this.copy(t).slerp(e,n)}random(){const t=Math.random(),e=Math.sqrt(1-t),n=Math.sqrt(t),i=2*Math.PI*Math.random(),s=2*Math.PI*Math.random();return this.set(e*Math.cos(i),n*Math.sin(s),n*Math.cos(s),e*Math.sin(i))}equals(t){return t._x===this._x&&t._y===this._y&&t._z===this._z&&t._w===this._w}fromArray(t,e=0){return this._x=t[e],this._y=t[e+1],this._z=t[e+2],this._w=t[e+3],this._onChangeCallback(),this}toArray(t=[],e=0){return t[e]=this._x,t[e+1]=this._y,t[e+2]=this._z,t[e+3]=this._w,t}fromBufferAttribute(t,e){return this._x=t.getX(e),this._y=t.getY(e),this._z=t.getZ(e),this._w=t.getW(e),this._onChangeCallback(),this}toJSON(){return this.toArray()}_onChange(t){return this._onChangeCallback=t,this}_onChangeCallback(){}*[Symbol.iterator](){yield this._x,yield this._y,yield this._z,yield this._w}}class R{constructor(t=0,e=0,n=0){R.prototype.isVector3=!0,this.x=t,this.y=e,this.z=n}set(t,e,n){return n===void 0&&(n=this.z),this.x=t,this.y=e,this.z=n,this}setScalar(t){return this.x=t,this.y=t,this.z=t,this}setX(t){return this.x=t,this}setY(t){return this.y=t,this}setZ(t){return this.z=t,this}setComponent(t,e){switch(t){case 0:this.x=e;break;case 1:this.y=e;break;case 2:this.z=e;break;default:throw new Error("index is out of range: "+t)}return this}getComponent(t){switch(t){case 0:return this.x;case 1:return this.y;case 2:return this.z;default:throw new Error("index is out of range: "+t)}}clone(){return new this.constructor(this.x,this.y,this.z)}copy(t){return this.x=t.x,this.y=t.y,this.z=t.z,this}add(t){return this.x+=t.x,this.y+=t.y,this.z+=t.z,this}addScalar(t){return this.x+=t,this.y+=t,this.z+=t,this}addVectors(t,e){return this.x=t.x+e.x,this.y=t.y+e.y,this.z=t.z+e.z,this}addScaledVector(t,e){return this.x+=t.x*e,this.y+=t.y*e,this.z+=t.z*e,this}sub(t){return this.x-=t.x,this.y-=t.y,this.z-=t.z,this}subScalar(t){return this.x-=t,this.y-=t,this.z-=t,this}subVectors(t,e){return this.x=t.x-e.x,this.y=t.y-e.y,this.z=t.z-e.z,this}multiply(t){return this.x*=t.x,this.y*=t.y,this.z*=t.z,this}multiplyScalar(t){return this.x*=t,this.y*=t,this.z*=t,this}multiplyVectors(t,e){return this.x=t.x*e.x,this.y=t.y*e.y,this.z=t.z*e.z,this}applyEuler(t){return this.applyQuaternion(du.setFromEuler(t))}applyAxisAngle(t,e){return this.applyQuaternion(du.setFromAxisAngle(t,e))}applyMatrix3(t){const e=this.x,n=this.y,i=this.z,s=t.elements;return this.x=s[0]*e+s[3]*n+s[6]*i,this.y=s[1]*e+s[4]*n+s[7]*i,this.z=s[2]*e+s[5]*n+s[8]*i,this}applyNormalMatrix(t){return this.applyMatrix3(t).normalize()}applyMatrix4(t){const e=this.x,n=this.y,i=this.z,s=t.elements,a=1/(s[3]*e+s[7]*n+s[11]*i+s[15]);return this.x=(s[0]*e+s[4]*n+s[8]*i+s[12])*a,this.y=(s[1]*e+s[5]*n+s[9]*i+s[13])*a,this.z=(s[2]*e+s[6]*n+s[10]*i+s[14])*a,this}applyQuaternion(t){const e=this.x,n=this.y,i=this.z,s=t.x,a=t.y,o=t.z,c=t.w,l=2*(a*i-o*n),h=2*(o*e-s*i),u=2*(s*n-a*e);return this.x=e+c*l+a*u-o*h,this.y=n+c*h+o*l-s*u,this.z=i+c*u+s*h-a*l,this}project(t){return this.applyMatrix4(t.matrixWorldInverse).applyMatrix4(t.projectionMatrix)}unproject(t){return this.applyMatrix4(t.projectionMatrixInverse).applyMatrix4(t.matrixWorld)}transformDirection(t){const e=this.x,n=this.y,i=this.z,s=t.elements;return this.x=s[0]*e+s[4]*n+s[8]*i,this.y=s[1]*e+s[5]*n+s[9]*i,this.z=s[2]*e+s[6]*n+s[10]*i,this.normalize()}divide(t){return this.x/=t.x,this.y/=t.y,this.z/=t.z,this}divideScalar(t){return this.multiplyScalar(1/t)}min(t){return this.x=Math.min(this.x,t.x),this.y=Math.min(this.y,t.y),this.z=Math.min(this.z,t.z),this}max(t){return this.x=Math.max(this.x,t.x),this.y=Math.max(this.y,t.y),this.z=Math.max(this.z,t.z),this}clamp(t,e){return this.x=Math.max(t.x,Math.min(e.x,this.x)),this.y=Math.max(t.y,Math.min(e.y,this.y)),this.z=Math.max(t.z,Math.min(e.z,this.z)),this}clampScalar(t,e){return this.x=Math.max(t,Math.min(e,this.x)),this.y=Math.max(t,Math.min(e,this.y)),this.z=Math.max(t,Math.min(e,this.z)),this}clampLength(t,e){const n=this.length();return this.divideScalar(n||1).multiplyScalar(Math.max(t,Math.min(e,n)))}floor(){return this.x=Math.floor(this.x),this.y=Math.floor(this.y),this.z=Math.floor(this.z),this}ceil(){return this.x=Math.ceil(this.x),this.y=Math.ceil(this.y),this.z=Math.ceil(this.z),this}round(){return this.x=Math.round(this.x),this.y=Math.round(this.y),this.z=Math.round(this.z),this}roundToZero(){return this.x=Math.trunc(this.x),this.y=Math.trunc(this.y),this.z=Math.trunc(this.z),this}negate(){return this.x=-this.x,this.y=-this.y,this.z=-this.z,this}dot(t){return this.x*t.x+this.y*t.y+this.z*t.z}lengthSq(){return this.x*this.x+this.y*this.y+this.z*this.z}length(){return Math.sqrt(this.x*this.x+this.y*this.y+this.z*this.z)}manhattanLength(){return Math.abs(this.x)+Math.abs(this.y)+Math.abs(this.z)}normalize(){return this.divideScalar(this.length()||1)}setLength(t){return this.normalize().multiplyScalar(t)}lerp(t,e){return this.x+=(t.x-this.x)*e,this.y+=(t.y-this.y)*e,this.z+=(t.z-this.z)*e,this}lerpVectors(t,e,n){return this.x=t.x+(e.x-t.x)*n,this.y=t.y+(e.y-t.y)*n,this.z=t.z+(e.z-t.z)*n,this}cross(t){return this.crossVectors(this,t)}crossVectors(t,e){const n=t.x,i=t.y,s=t.z,a=e.x,o=e.y,c=e.z;return this.x=i*c-s*o,this.y=s*a-n*c,this.z=n*o-i*a,this}projectOnVector(t){const e=t.lengthSq();if(e===0)return this.set(0,0,0);const n=t.dot(this)/e;return this.copy(t).multiplyScalar(n)}projectOnPlane(t){return Dc.copy(this).projectOnVector(t),this.sub(Dc)}reflect(t){return this.sub(Dc.copy(t).multiplyScalar(2*this.dot(t)))}angleTo(t){const e=Math.sqrt(this.lengthSq()*t.lengthSq());if(e===0)return Math.PI/2;const n=this.dot(t)/e;return Math.acos(We(n,-1,1))}distanceTo(t){return Math.sqrt(this.distanceToSquared(t))}distanceToSquared(t){const e=this.x-t.x,n=this.y-t.y,i=this.z-t.z;return e*e+n*n+i*i}manhattanDistanceTo(t){return Math.abs(this.x-t.x)+Math.abs(this.y-t.y)+Math.abs(this.z-t.z)}setFromSpherical(t){return this.setFromSphericalCoords(t.radius,t.phi,t.theta)}setFromSphericalCoords(t,e,n){const i=Math.sin(e)*t;return this.x=i*Math.sin(n),this.y=Math.cos(e)*t,this.z=i*Math.cos(n),this}setFromCylindrical(t){return this.setFromCylindricalCoords(t.radius,t.theta,t.y)}setFromCylindricalCoords(t,e,n){return this.x=t*Math.sin(e),this.y=n,this.z=t*Math.cos(e),this}setFromMatrixPosition(t){const e=t.elements;return this.x=e[12],this.y=e[13],this.z=e[14],this}setFromMatrixScale(t){const e=this.setFromMatrixColumn(t,0).length(),n=this.setFromMatrixColumn(t,1).length(),i=this.setFromMatrixColumn(t,2).length();return this.x=e,this.y=n,this.z=i,this}setFromMatrixColumn(t,e){return this.fromArray(t.elements,e*4)}setFromMatrix3Column(t,e){return this.fromArray(t.elements,e*3)}setFromEuler(t){return this.x=t._x,this.y=t._y,this.z=t._z,this}setFromColor(t){return this.x=t.r,this.y=t.g,this.z=t.b,this}equals(t){return t.x===this.x&&t.y===this.y&&t.z===this.z}fromArray(t,e=0){return this.x=t[e],this.y=t[e+1],this.z=t[e+2],this}toArray(t=[],e=0){return t[e]=this.x,t[e+1]=this.y,t[e+2]=this.z,t}fromBufferAttribute(t,e){return this.x=t.getX(e),this.y=t.getY(e),this.z=t.getZ(e),this}random(){return this.x=Math.random(),this.y=Math.random(),this.z=Math.random(),this}randomDirection(){const t=(Math.random()-.5)*2,e=Math.random()*Math.PI*2,n=Math.sqrt(1-t**2);return this.x=n*Math.cos(e),this.y=n*Math.sin(e),this.z=t,this}*[Symbol.iterator](){yield this.x,yield this.y,yield this.z}}const Dc=new R,du=new ni;class rn{constructor(t=new R(1/0,1/0,1/0),e=new R(-1/0,-1/0,-1/0)){this.isBox3=!0,this.min=t,this.max=e}set(t,e){return this.min.copy(t),this.max.copy(e),this}setFromArray(t){this.makeEmpty();for(let e=0,n=t.length;e<n;e+=3)this.expandByPoint(jn.fromArray(t,e));return this}setFromBufferAttribute(t){this.makeEmpty();for(let e=0,n=t.count;e<n;e++)this.expandByPoint(jn.fromBufferAttribute(t,e));return this}setFromPoints(t){this.makeEmpty();for(let e=0,n=t.length;e<n;e++)this.expandByPoint(t[e]);return this}setFromCenterAndSize(t,e){const n=jn.copy(e).multiplyScalar(.5);return this.min.copy(t).sub(n),this.max.copy(t).add(n),this}setFromObject(t,e=!1){return this.makeEmpty(),this.expandByObject(t,e)}clone(){return new this.constructor().copy(this)}copy(t){return this.min.copy(t.min),this.max.copy(t.max),this}makeEmpty(){return this.min.x=this.min.y=this.min.z=1/0,this.max.x=this.max.y=this.max.z=-1/0,this}isEmpty(){return this.max.x<this.min.x||this.max.y<this.min.y||this.max.z<this.min.z}getCenter(t){return this.isEmpty()?t.set(0,0,0):t.addVectors(this.min,this.max).multiplyScalar(.5)}getSize(t){return this.isEmpty()?t.set(0,0,0):t.subVectors(this.max,this.min)}expandByPoint(t){return this.min.min(t),this.max.max(t),this}expandByVector(t){return this.min.sub(t),this.max.add(t),this}expandByScalar(t){return this.min.addScalar(-t),this.max.addScalar(t),this}expandByObject(t,e=!1){t.updateWorldMatrix(!1,!1);const n=t.geometry;if(n!==void 0){const s=n.getAttribute("position");if(e===!0&&s!==void 0&&t.isInstancedMesh!==!0)for(let a=0,o=s.count;a<o;a++)t.isMesh===!0?t.getVertexPosition(a,jn):jn.fromBufferAttribute(s,a),jn.applyMatrix4(t.matrixWorld),this.expandByPoint(jn);else t.boundingBox!==void 0?(t.boundingBox===null&&t.computeBoundingBox(),za.copy(t.boundingBox)):(n.boundingBox===null&&n.computeBoundingBox(),za.copy(n.boundingBox)),za.applyMatrix4(t.matrixWorld),this.union(za)}const i=t.children;for(let s=0,a=i.length;s<a;s++)this.expandByObject(i[s],e);return this}containsPoint(t){return!(t.x<this.min.x||t.x>this.max.x||t.y<this.min.y||t.y>this.max.y||t.z<this.min.z||t.z>this.max.z)}containsBox(t){return this.min.x<=t.min.x&&t.max.x<=this.max.x&&this.min.y<=t.min.y&&t.max.y<=this.max.y&&this.min.z<=t.min.z&&t.max.z<=this.max.z}getParameter(t,e){return e.set((t.x-this.min.x)/(this.max.x-this.min.x),(t.y-this.min.y)/(this.max.y-this.min.y),(t.z-this.min.z)/(this.max.z-this.min.z))}intersectsBox(t){return!(t.max.x<this.min.x||t.min.x>this.max.x||t.max.y<this.min.y||t.min.y>this.max.y||t.max.z<this.min.z||t.min.z>this.max.z)}intersectsSphere(t){return this.clampPoint(t.center,jn),jn.distanceToSquared(t.center)<=t.radius*t.radius}intersectsPlane(t){let e,n;return t.normal.x>0?(e=t.normal.x*this.min.x,n=t.normal.x*this.max.x):(e=t.normal.x*this.max.x,n=t.normal.x*this.min.x),t.normal.y>0?(e+=t.normal.y*this.min.y,n+=t.normal.y*this.max.y):(e+=t.normal.y*this.max.y,n+=t.normal.y*this.min.y),t.normal.z>0?(e+=t.normal.z*this.min.z,n+=t.normal.z*this.max.z):(e+=t.normal.z*this.max.z,n+=t.normal.z*this.min.z),e<=-t.constant&&n>=-t.constant}intersectsTriangle(t){if(this.isEmpty())return!1;this.getCenter(Dr),Oa.subVectors(this.max,Dr),Ns.subVectors(t.a,Dr),Fs.subVectors(t.b,Dr),Bs.subVectors(t.c,Dr),Wi.subVectors(Fs,Ns),Xi.subVectors(Bs,Fs),ls.subVectors(Ns,Bs);let e=[0,-Wi.z,Wi.y,0,-Xi.z,Xi.y,0,-ls.z,ls.y,Wi.z,0,-Wi.x,Xi.z,0,-Xi.x,ls.z,0,-ls.x,-Wi.y,Wi.x,0,-Xi.y,Xi.x,0,-ls.y,ls.x,0];return!Ic(e,Ns,Fs,Bs,Oa)||(e=[1,0,0,0,1,0,0,0,1],!Ic(e,Ns,Fs,Bs,Oa))?!1:(Ga.crossVectors(Wi,Xi),e=[Ga.x,Ga.y,Ga.z],Ic(e,Ns,Fs,Bs,Oa))}clampPoint(t,e){return e.copy(t).clamp(this.min,this.max)}distanceToPoint(t){return this.clampPoint(t,jn).distanceTo(t)}getBoundingSphere(t){return this.isEmpty()?t.makeEmpty():(this.getCenter(t.center),t.radius=this.getSize(jn).length()*.5),t}intersect(t){return this.min.max(t.min),this.max.min(t.max),this.isEmpty()&&this.makeEmpty(),this}union(t){return this.min.min(t.min),this.max.max(t.max),this}applyMatrix4(t){return this.isEmpty()?this:(Ei[0].set(this.min.x,this.min.y,this.min.z).applyMatrix4(t),Ei[1].set(this.min.x,this.min.y,this.max.z).applyMatrix4(t),Ei[2].set(this.min.x,this.max.y,this.min.z).applyMatrix4(t),Ei[3].set(this.min.x,this.max.y,this.max.z).applyMatrix4(t),Ei[4].set(this.max.x,this.min.y,this.min.z).applyMatrix4(t),Ei[5].set(this.max.x,this.min.y,this.max.z).applyMatrix4(t),Ei[6].set(this.max.x,this.max.y,this.min.z).applyMatrix4(t),Ei[7].set(this.max.x,this.max.y,this.max.z).applyMatrix4(t),this.setFromPoints(Ei),this)}translate(t){return this.min.add(t),this.max.add(t),this}equals(t){return t.min.equals(this.min)&&t.max.equals(this.max)}}const Ei=[new R,new R,new R,new R,new R,new R,new R,new R],jn=new R,za=new rn,Ns=new R,Fs=new R,Bs=new R,Wi=new R,Xi=new R,ls=new R,Dr=new R,Oa=new R,Ga=new R,hs=new R;function Ic(r,t,e,n,i){for(let s=0,a=r.length-3;s<=a;s+=3){hs.fromArray(r,s);const o=i.x*Math.abs(hs.x)+i.y*Math.abs(hs.y)+i.z*Math.abs(hs.z),c=t.dot(hs),l=e.dot(hs),h=n.dot(hs);if(Math.max(-Math.max(c,l,h),Math.min(c,l,h))>o)return!1}return!0}const Lm=new rn,Ir=new R,Nc=new R;class as{constructor(t=new R,e=-1){this.isSphere=!0,this.center=t,this.radius=e}set(t,e){return this.center.copy(t),this.radius=e,this}setFromPoints(t,e){const n=this.center;e!==void 0?n.copy(e):Lm.setFromPoints(t).getCenter(n);let i=0;for(let s=0,a=t.length;s<a;s++)i=Math.max(i,n.distanceToSquared(t[s]));return this.radius=Math.sqrt(i),this}copy(t){return this.center.copy(t.center),this.radius=t.radius,this}isEmpty(){return this.radius<0}makeEmpty(){return this.center.set(0,0,0),this.radius=-1,this}containsPoint(t){return t.distanceToSquared(this.center)<=this.radius*this.radius}distanceToPoint(t){return t.distanceTo(this.center)-this.radius}intersectsSphere(t){const e=this.radius+t.radius;return t.center.distanceToSquared(this.center)<=e*e}intersectsBox(t){return t.intersectsSphere(this)}intersectsPlane(t){return Math.abs(t.distanceToPoint(this.center))<=this.radius}clampPoint(t,e){const n=this.center.distanceToSquared(t);return e.copy(t),n>this.radius*this.radius&&(e.sub(this.center).normalize(),e.multiplyScalar(this.radius).add(this.center)),e}getBoundingBox(t){return this.isEmpty()?(t.makeEmpty(),t):(t.set(this.center,this.center),t.expandByScalar(this.radius),t)}applyMatrix4(t){return this.center.applyMatrix4(t),this.radius=this.radius*t.getMaxScaleOnAxis(),this}translate(t){return this.center.add(t),this}expandByPoint(t){if(this.isEmpty())return this.center.copy(t),this.radius=0,this;Ir.subVectors(t,this.center);const e=Ir.lengthSq();if(e>this.radius*this.radius){const n=Math.sqrt(e),i=(n-this.radius)*.5;this.center.addScaledVector(Ir,i/n),this.radius+=i}return this}union(t){return t.isEmpty()?this:this.isEmpty()?(this.copy(t),this):(this.center.equals(t.center)===!0?this.radius=Math.max(this.radius,t.radius):(Nc.subVectors(t.center,this.center).setLength(t.radius),this.expandByPoint(Ir.copy(t.center).add(Nc)),this.expandByPoint(Ir.copy(t.center).sub(Nc))),this)}equals(t){return t.center.equals(this.center)&&t.radius===this.radius}clone(){return new this.constructor().copy(this)}}const wi=new R,Fc=new R,Va=new R,Yi=new R,Bc=new R,ka=new R,Uc=new R;class Fh{constructor(t=new R,e=new R(0,0,-1)){this.origin=t,this.direction=e}set(t,e){return this.origin.copy(t),this.direction.copy(e),this}copy(t){return this.origin.copy(t.origin),this.direction.copy(t.direction),this}at(t,e){return e.copy(this.origin).addScaledVector(this.direction,t)}lookAt(t){return this.direction.copy(t).sub(this.origin).normalize(),this}recast(t){return this.origin.copy(this.at(t,wi)),this}closestPointToPoint(t,e){e.subVectors(t,this.origin);const n=e.dot(this.direction);return n<0?e.copy(this.origin):e.copy(this.origin).addScaledVector(this.direction,n)}distanceToPoint(t){return Math.sqrt(this.distanceSqToPoint(t))}distanceSqToPoint(t){const e=wi.subVectors(t,this.origin).dot(this.direction);return e<0?this.origin.distanceToSquared(t):(wi.copy(this.origin).addScaledVector(this.direction,e),wi.distanceToSquared(t))}distanceSqToSegment(t,e,n,i){Fc.copy(t).add(e).multiplyScalar(.5),Va.copy(e).sub(t).normalize(),Yi.copy(this.origin).sub(Fc);const s=t.distanceTo(e)*.5,a=-this.direction.dot(Va),o=Yi.dot(this.direction),c=-Yi.dot(Va),l=Yi.lengthSq(),h=Math.abs(1-a*a);let u,d,f,g;if(h>0)if(u=a*c-o,d=a*o-c,g=s*h,u>=0)if(d>=-g)if(d<=g){const x=1/h;u*=x,d*=x,f=u*(u+a*d+2*o)+d*(a*u+d+2*c)+l}else d=s,u=Math.max(0,-(a*d+o)),f=-u*u+d*(d+2*c)+l;else d=-s,u=Math.max(0,-(a*d+o)),f=-u*u+d*(d+2*c)+l;else d<=-g?(u=Math.max(0,-(-a*s+o)),d=u>0?-s:Math.min(Math.max(-s,-c),s),f=-u*u+d*(d+2*c)+l):d<=g?(u=0,d=Math.min(Math.max(-s,-c),s),f=d*(d+2*c)+l):(u=Math.max(0,-(a*s+o)),d=u>0?s:Math.min(Math.max(-s,-c),s),f=-u*u+d*(d+2*c)+l);else d=a>0?-s:s,u=Math.max(0,-(a*d+o)),f=-u*u+d*(d+2*c)+l;return n&&n.copy(this.origin).addScaledVector(this.direction,u),i&&i.copy(Fc).addScaledVector(Va,d),f}intersectSphere(t,e){wi.subVectors(t.center,this.origin);const n=wi.dot(this.direction),i=wi.dot(wi)-n*n,s=t.radius*t.radius;if(i>s)return null;const a=Math.sqrt(s-i),o=n-a,c=n+a;return c<0?null:o<0?this.at(c,e):this.at(o,e)}intersectsSphere(t){return this.distanceSqToPoint(t.center)<=t.radius*t.radius}distanceToPlane(t){const e=t.normal.dot(this.direction);if(e===0)return t.distanceToPoint(this.origin)===0?0:null;const n=-(this.origin.dot(t.normal)+t.constant)/e;return n>=0?n:null}intersectPlane(t,e){const n=this.distanceToPlane(t);return n===null?null:this.at(n,e)}intersectsPlane(t){const e=t.distanceToPoint(this.origin);return e===0||t.normal.dot(this.direction)*e<0}intersectBox(t,e){let n,i,s,a,o,c;const l=1/this.direction.x,h=1/this.direction.y,u=1/this.direction.z,d=this.origin;return l>=0?(n=(t.min.x-d.x)*l,i=(t.max.x-d.x)*l):(n=(t.max.x-d.x)*l,i=(t.min.x-d.x)*l),h>=0?(s=(t.min.y-d.y)*h,a=(t.max.y-d.y)*h):(s=(t.max.y-d.y)*h,a=(t.min.y-d.y)*h),n>a||s>i||((s>n||isNaN(n))&&(n=s),(a<i||isNaN(i))&&(i=a),u>=0?(o=(t.min.z-d.z)*u,c=(t.max.z-d.z)*u):(o=(t.max.z-d.z)*u,c=(t.min.z-d.z)*u),n>c||o>i)||((o>n||n!==n)&&(n=o),(c<i||i!==i)&&(i=c),i<0)?null:this.at(n>=0?n:i,e)}intersectsBox(t){return this.intersectBox(t,wi)!==null}intersectTriangle(t,e,n,i,s){Bc.subVectors(e,t),ka.subVectors(n,t),Uc.crossVectors(Bc,ka);let a=this.direction.dot(Uc),o;if(a>0){if(i)return null;o=1}else if(a<0)o=-1,a=-a;else return null;Yi.subVectors(this.origin,t);const c=o*this.direction.dot(ka.crossVectors(Yi,ka));if(c<0)return null;const l=o*this.direction.dot(Bc.cross(Yi));if(l<0||c+l>a)return null;const h=-o*Yi.dot(Uc);return h<0?null:this.at(h/a,s)}applyMatrix4(t){return this.origin.applyMatrix4(t),this.direction.transformDirection(t),this}equals(t){return t.origin.equals(this.origin)&&t.direction.equals(this.direction)}clone(){return new this.constructor().copy(this)}}class re{constructor(t,e,n,i,s,a,o,c,l,h,u,d,f,g,x,m){re.prototype.isMatrix4=!0,this.elements=[1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1],t!==void 0&&this.set(t,e,n,i,s,a,o,c,l,h,u,d,f,g,x,m)}set(t,e,n,i,s,a,o,c,l,h,u,d,f,g,x,m){const p=this.elements;return p[0]=t,p[4]=e,p[8]=n,p[12]=i,p[1]=s,p[5]=a,p[9]=o,p[13]=c,p[2]=l,p[6]=h,p[10]=u,p[14]=d,p[3]=f,p[7]=g,p[11]=x,p[15]=m,this}identity(){return this.set(1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1),this}clone(){return new re().fromArray(this.elements)}copy(t){const e=this.elements,n=t.elements;return e[0]=n[0],e[1]=n[1],e[2]=n[2],e[3]=n[3],e[4]=n[4],e[5]=n[5],e[6]=n[6],e[7]=n[7],e[8]=n[8],e[9]=n[9],e[10]=n[10],e[11]=n[11],e[12]=n[12],e[13]=n[13],e[14]=n[14],e[15]=n[15],this}copyPosition(t){const e=this.elements,n=t.elements;return e[12]=n[12],e[13]=n[13],e[14]=n[14],this}setFromMatrix3(t){const e=t.elements;return this.set(e[0],e[3],e[6],0,e[1],e[4],e[7],0,e[2],e[5],e[8],0,0,0,0,1),this}extractBasis(t,e,n){return t.setFromMatrixColumn(this,0),e.setFromMatrixColumn(this,1),n.setFromMatrixColumn(this,2),this}makeBasis(t,e,n){return this.set(t.x,e.x,n.x,0,t.y,e.y,n.y,0,t.z,e.z,n.z,0,0,0,0,1),this}extractRotation(t){const e=this.elements,n=t.elements,i=1/Us.setFromMatrixColumn(t,0).length(),s=1/Us.setFromMatrixColumn(t,1).length(),a=1/Us.setFromMatrixColumn(t,2).length();return e[0]=n[0]*i,e[1]=n[1]*i,e[2]=n[2]*i,e[3]=0,e[4]=n[4]*s,e[5]=n[5]*s,e[6]=n[6]*s,e[7]=0,e[8]=n[8]*a,e[9]=n[9]*a,e[10]=n[10]*a,e[11]=0,e[12]=0,e[13]=0,e[14]=0,e[15]=1,this}makeRotationFromEuler(t){const e=this.elements,n=t.x,i=t.y,s=t.z,a=Math.cos(n),o=Math.sin(n),c=Math.cos(i),l=Math.sin(i),h=Math.cos(s),u=Math.sin(s);if(t.order==="XYZ"){const d=a*h,f=a*u,g=o*h,x=o*u;e[0]=c*h,e[4]=-c*u,e[8]=l,e[1]=f+g*l,e[5]=d-x*l,e[9]=-o*c,e[2]=x-d*l,e[6]=g+f*l,e[10]=a*c}else if(t.order==="YXZ"){const d=c*h,f=c*u,g=l*h,x=l*u;e[0]=d+x*o,e[4]=g*o-f,e[8]=a*l,e[1]=a*u,e[5]=a*h,e[9]=-o,e[2]=f*o-g,e[6]=x+d*o,e[10]=a*c}else if(t.order==="ZXY"){const d=c*h,f=c*u,g=l*h,x=l*u;e[0]=d-x*o,e[4]=-a*u,e[8]=g+f*o,e[1]=f+g*o,e[5]=a*h,e[9]=x-d*o,e[2]=-a*l,e[6]=o,e[10]=a*c}else if(t.order==="ZYX"){const d=a*h,f=a*u,g=o*h,x=o*u;e[0]=c*h,e[4]=g*l-f,e[8]=d*l+x,e[1]=c*u,e[5]=x*l+d,e[9]=f*l-g,e[2]=-l,e[6]=o*c,e[10]=a*c}else if(t.order==="YZX"){const d=a*c,f=a*l,g=o*c,x=o*l;e[0]=c*h,e[4]=x-d*u,e[8]=g*u+f,e[1]=u,e[5]=a*h,e[9]=-o*h,e[2]=-l*h,e[6]=f*u+g,e[10]=d-x*u}else if(t.order==="XZY"){const d=a*c,f=a*l,g=o*c,x=o*l;e[0]=c*h,e[4]=-u,e[8]=l*h,e[1]=d*u+x,e[5]=a*h,e[9]=f*u-g,e[2]=g*u-f,e[6]=o*h,e[10]=x*u+d}return e[3]=0,e[7]=0,e[11]=0,e[12]=0,e[13]=0,e[14]=0,e[15]=1,this}makeRotationFromQuaternion(t){return this.compose(Dm,t,Im)}lookAt(t,e,n){const i=this.elements;return bn.subVectors(t,e),bn.lengthSq()===0&&(bn.z=1),bn.normalize(),qi.crossVectors(n,bn),qi.lengthSq()===0&&(Math.abs(n.z)===1?bn.x+=1e-4:bn.z+=1e-4,bn.normalize(),qi.crossVectors(n,bn)),qi.normalize(),Ha.crossVectors(bn,qi),i[0]=qi.x,i[4]=Ha.x,i[8]=bn.x,i[1]=qi.y,i[5]=Ha.y,i[9]=bn.y,i[2]=qi.z,i[6]=Ha.z,i[10]=bn.z,this}multiply(t){return this.multiplyMatrices(this,t)}premultiply(t){return this.multiplyMatrices(t,this)}multiplyMatrices(t,e){const n=t.elements,i=e.elements,s=this.elements,a=n[0],o=n[4],c=n[8],l=n[12],h=n[1],u=n[5],d=n[9],f=n[13],g=n[2],x=n[6],m=n[10],p=n[14],_=n[3],v=n[7],S=n[11],M=n[15],y=i[0],w=i[4],A=i[8],E=i[12],T=i[1],L=i[5],C=i[9],F=i[13],D=i[2],N=i[6],B=i[10],G=i[14],z=i[3],H=i[7],j=i[11],$=i[15];return s[0]=a*y+o*T+c*D+l*z,s[4]=a*w+o*L+c*N+l*H,s[8]=a*A+o*C+c*B+l*j,s[12]=a*E+o*F+c*G+l*$,s[1]=h*y+u*T+d*D+f*z,s[5]=h*w+u*L+d*N+f*H,s[9]=h*A+u*C+d*B+f*j,s[13]=h*E+u*F+d*G+f*$,s[2]=g*y+x*T+m*D+p*z,s[6]=g*w+x*L+m*N+p*H,s[10]=g*A+x*C+m*B+p*j,s[14]=g*E+x*F+m*G+p*$,s[3]=_*y+v*T+S*D+M*z,s[7]=_*w+v*L+S*N+M*H,s[11]=_*A+v*C+S*B+M*j,s[15]=_*E+v*F+S*G+M*$,this}multiplyScalar(t){const e=this.elements;return e[0]*=t,e[4]*=t,e[8]*=t,e[12]*=t,e[1]*=t,e[5]*=t,e[9]*=t,e[13]*=t,e[2]*=t,e[6]*=t,e[10]*=t,e[14]*=t,e[3]*=t,e[7]*=t,e[11]*=t,e[15]*=t,this}determinant(){const t=this.elements,e=t[0],n=t[4],i=t[8],s=t[12],a=t[1],o=t[5],c=t[9],l=t[13],h=t[2],u=t[6],d=t[10],f=t[14],g=t[3],x=t[7],m=t[11],p=t[15];return g*(+s*c*u-i*l*u-s*o*d+n*l*d+i*o*f-n*c*f)+x*(+e*c*f-e*l*d+s*a*d-i*a*f+i*l*h-s*c*h)+m*(+e*l*u-e*o*f-s*a*u+n*a*f+s*o*h-n*l*h)+p*(-i*o*h-e*c*u+e*o*d+i*a*u-n*a*d+n*c*h)}transpose(){const t=this.elements;let e;return e=t[1],t[1]=t[4],t[4]=e,e=t[2],t[2]=t[8],t[8]=e,e=t[6],t[6]=t[9],t[9]=e,e=t[3],t[3]=t[12],t[12]=e,e=t[7],t[7]=t[13],t[13]=e,e=t[11],t[11]=t[14],t[14]=e,this}setPosition(t,e,n){const i=this.elements;return t.isVector3?(i[12]=t.x,i[13]=t.y,i[14]=t.z):(i[12]=t,i[13]=e,i[14]=n),this}invert(){const t=this.elements,e=t[0],n=t[1],i=t[2],s=t[3],a=t[4],o=t[5],c=t[6],l=t[7],h=t[8],u=t[9],d=t[10],f=t[11],g=t[12],x=t[13],m=t[14],p=t[15],_=u*m*l-x*d*l+x*c*f-o*m*f-u*c*p+o*d*p,v=g*d*l-h*m*l-g*c*f+a*m*f+h*c*p-a*d*p,S=h*x*l-g*u*l+g*o*f-a*x*f-h*o*p+a*u*p,M=g*u*c-h*x*c-g*o*d+a*x*d+h*o*m-a*u*m,y=e*_+n*v+i*S+s*M;if(y===0)return this.set(0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0);const w=1/y;return t[0]=_*w,t[1]=(x*d*s-u*m*s-x*i*f+n*m*f+u*i*p-n*d*p)*w,t[2]=(o*m*s-x*c*s+x*i*l-n*m*l-o*i*p+n*c*p)*w,t[3]=(u*c*s-o*d*s-u*i*l+n*d*l+o*i*f-n*c*f)*w,t[4]=v*w,t[5]=(h*m*s-g*d*s+g*i*f-e*m*f-h*i*p+e*d*p)*w,t[6]=(g*c*s-a*m*s-g*i*l+e*m*l+a*i*p-e*c*p)*w,t[7]=(a*d*s-h*c*s+h*i*l-e*d*l-a*i*f+e*c*f)*w,t[8]=S*w,t[9]=(g*u*s-h*x*s-g*n*f+e*x*f+h*n*p-e*u*p)*w,t[10]=(a*x*s-g*o*s+g*n*l-e*x*l-a*n*p+e*o*p)*w,t[11]=(h*o*s-a*u*s-h*n*l+e*u*l+a*n*f-e*o*f)*w,t[12]=M*w,t[13]=(h*x*i-g*u*i+g*n*d-e*x*d-h*n*m+e*u*m)*w,t[14]=(g*o*i-a*x*i-g*n*c+e*x*c+a*n*m-e*o*m)*w,t[15]=(a*u*i-h*o*i+h*n*c-e*u*c-a*n*d+e*o*d)*w,this}scale(t){const e=this.elements,n=t.x,i=t.y,s=t.z;return e[0]*=n,e[4]*=i,e[8]*=s,e[1]*=n,e[5]*=i,e[9]*=s,e[2]*=n,e[6]*=i,e[10]*=s,e[3]*=n,e[7]*=i,e[11]*=s,this}getMaxScaleOnAxis(){const t=this.elements,e=t[0]*t[0]+t[1]*t[1]+t[2]*t[2],n=t[4]*t[4]+t[5]*t[5]+t[6]*t[6],i=t[8]*t[8]+t[9]*t[9]+t[10]*t[10];return Math.sqrt(Math.max(e,n,i))}makeTranslation(t,e,n){return t.isVector3?this.set(1,0,0,t.x,0,1,0,t.y,0,0,1,t.z,0,0,0,1):this.set(1,0,0,t,0,1,0,e,0,0,1,n,0,0,0,1),this}makeRotationX(t){const e=Math.cos(t),n=Math.sin(t);return this.set(1,0,0,0,0,e,-n,0,0,n,e,0,0,0,0,1),this}makeRotationY(t){const e=Math.cos(t),n=Math.sin(t);return this.set(e,0,n,0,0,1,0,0,-n,0,e,0,0,0,0,1),this}makeRotationZ(t){const e=Math.cos(t),n=Math.sin(t);return this.set(e,-n,0,0,n,e,0,0,0,0,1,0,0,0,0,1),this}makeRotationAxis(t,e){const n=Math.cos(e),i=Math.sin(e),s=1-n,a=t.x,o=t.y,c=t.z,l=s*a,h=s*o;return this.set(l*a+n,l*o-i*c,l*c+i*o,0,l*o+i*c,h*o+n,h*c-i*a,0,l*c-i*o,h*c+i*a,s*c*c+n,0,0,0,0,1),this}makeScale(t,e,n){return this.set(t,0,0,0,0,e,0,0,0,0,n,0,0,0,0,1),this}makeShear(t,e,n,i,s,a){return this.set(1,n,s,0,t,1,a,0,e,i,1,0,0,0,0,1),this}compose(t,e,n){const i=this.elements,s=e._x,a=e._y,o=e._z,c=e._w,l=s+s,h=a+a,u=o+o,d=s*l,f=s*h,g=s*u,x=a*h,m=a*u,p=o*u,_=c*l,v=c*h,S=c*u,M=n.x,y=n.y,w=n.z;return i[0]=(1-(x+p))*M,i[1]=(f+S)*M,i[2]=(g-v)*M,i[3]=0,i[4]=(f-S)*y,i[5]=(1-(d+p))*y,i[6]=(m+_)*y,i[7]=0,i[8]=(g+v)*w,i[9]=(m-_)*w,i[10]=(1-(d+x))*w,i[11]=0,i[12]=t.x,i[13]=t.y,i[14]=t.z,i[15]=1,this}decompose(t,e,n){const i=this.elements;let s=Us.set(i[0],i[1],i[2]).length();const a=Us.set(i[4],i[5],i[6]).length(),o=Us.set(i[8],i[9],i[10]).length();this.determinant()<0&&(s=-s),t.x=i[12],t.y=i[13],t.z=i[14],$n.copy(this);const l=1/s,h=1/a,u=1/o;return $n.elements[0]*=l,$n.elements[1]*=l,$n.elements[2]*=l,$n.elements[4]*=h,$n.elements[5]*=h,$n.elements[6]*=h,$n.elements[8]*=u,$n.elements[9]*=u,$n.elements[10]*=u,e.setFromRotationMatrix($n),n.x=s,n.y=a,n.z=o,this}makePerspective(t,e,n,i,s,a,o=2e3){const c=this.elements,l=2*s/(e-t),h=2*s/(n-i),u=(e+t)/(e-t),d=(n+i)/(n-i);let f,g;if(o===2e3)f=-(a+s)/(a-s),g=-2*a*s/(a-s);else if(o===2001)f=-a/(a-s),g=-a*s/(a-s);else throw new Error("THREE.Matrix4.makePerspective(): Invalid coordinate system: "+o);return c[0]=l,c[4]=0,c[8]=u,c[12]=0,c[1]=0,c[5]=h,c[9]=d,c[13]=0,c[2]=0,c[6]=0,c[10]=f,c[14]=g,c[3]=0,c[7]=0,c[11]=-1,c[15]=0,this}makeOrthographic(t,e,n,i,s,a,o=2e3){const c=this.elements,l=1/(e-t),h=1/(n-i),u=1/(a-s),d=(e+t)*l,f=(n+i)*h;let g,x;if(o===2e3)g=(a+s)*u,x=-2*u;else if(o===2001)g=s*u,x=-1*u;else throw new Error("THREE.Matrix4.makeOrthographic(): Invalid coordinate system: "+o);return c[0]=2*l,c[4]=0,c[8]=0,c[12]=-d,c[1]=0,c[5]=2*h,c[9]=0,c[13]=-f,c[2]=0,c[6]=0,c[10]=x,c[14]=-g,c[3]=0,c[7]=0,c[11]=0,c[15]=1,this}equals(t){const e=this.elements,n=t.elements;for(let i=0;i<16;i++)if(e[i]!==n[i])return!1;return!0}fromArray(t,e=0){for(let n=0;n<16;n++)this.elements[n]=t[n+e];return this}toArray(t=[],e=0){const n=this.elements;return t[e]=n[0],t[e+1]=n[1],t[e+2]=n[2],t[e+3]=n[3],t[e+4]=n[4],t[e+5]=n[5],t[e+6]=n[6],t[e+7]=n[7],t[e+8]=n[8],t[e+9]=n[9],t[e+10]=n[10],t[e+11]=n[11],t[e+12]=n[12],t[e+13]=n[13],t[e+14]=n[14],t[e+15]=n[15],t}}const Us=new R,$n=new re,Dm=new R(0,0,0),Im=new R(1,1,1),qi=new R,Ha=new R,bn=new R,fu=new re,pu=new ni;class un{constructor(t=0,e=0,n=0,i=un.DEFAULT_ORDER){this.isEuler=!0,this._x=t,this._y=e,this._z=n,this._order=i}get x(){return this._x}set x(t){this._x=t,this._onChangeCallback()}get y(){return this._y}set y(t){this._y=t,this._onChangeCallback()}get z(){return this._z}set z(t){this._z=t,this._onChangeCallback()}get order(){return this._order}set order(t){this._order=t,this._onChangeCallback()}set(t,e,n,i=this._order){return this._x=t,this._y=e,this._z=n,this._order=i,this._onChangeCallback(),this}clone(){return new this.constructor(this._x,this._y,this._z,this._order)}copy(t){return this._x=t._x,this._y=t._y,this._z=t._z,this._order=t._order,this._onChangeCallback(),this}setFromRotationMatrix(t,e=this._order,n=!0){const i=t.elements,s=i[0],a=i[4],o=i[8],c=i[1],l=i[5],h=i[9],u=i[2],d=i[6],f=i[10];switch(e){case"XYZ":this._y=Math.asin(We(o,-1,1)),Math.abs(o)<.9999999?(this._x=Math.atan2(-h,f),this._z=Math.atan2(-a,s)):(this._x=Math.atan2(d,l),this._z=0);break;case"YXZ":this._x=Math.asin(-We(h,-1,1)),Math.abs(h)<.9999999?(this._y=Math.atan2(o,f),this._z=Math.atan2(c,l)):(this._y=Math.atan2(-u,s),this._z=0);break;case"ZXY":this._x=Math.asin(We(d,-1,1)),Math.abs(d)<.9999999?(this._y=Math.atan2(-u,f),this._z=Math.atan2(-a,l)):(this._y=0,this._z=Math.atan2(c,s));break;case"ZYX":this._y=Math.asin(-We(u,-1,1)),Math.abs(u)<.9999999?(this._x=Math.atan2(d,f),this._z=Math.atan2(c,s)):(this._x=0,this._z=Math.atan2(-a,l));break;case"YZX":this._z=Math.asin(We(c,-1,1)),Math.abs(c)<.9999999?(this._x=Math.atan2(-h,l),this._y=Math.atan2(-u,s)):(this._x=0,this._y=Math.atan2(o,f));break;case"XZY":this._z=Math.asin(-We(a,-1,1)),Math.abs(a)<.9999999?(this._x=Math.atan2(d,l),this._y=Math.atan2(o,s)):(this._x=Math.atan2(-h,f),this._y=0);break;default:console.warn("THREE.Euler: .setFromRotationMatrix() encountered an unknown order: "+e)}return this._order=e,n===!0&&this._onChangeCallback(),this}setFromQuaternion(t,e,n){return fu.makeRotationFromQuaternion(t),this.setFromRotationMatrix(fu,e,n)}setFromVector3(t,e=this._order){return this.set(t.x,t.y,t.z,e)}reorder(t){return pu.setFromEuler(this),this.setFromQuaternion(pu,t)}equals(t){return t._x===this._x&&t._y===this._y&&t._z===this._z&&t._order===this._order}fromArray(t){return this._x=t[0],this._y=t[1],this._z=t[2],t[3]!==void 0&&(this._order=t[3]),this._onChangeCallback(),this}toArray(t=[],e=0){return t[e]=this._x,t[e+1]=this._y,t[e+2]=this._z,t[e+3]=this._order,t}_onChange(t){return this._onChangeCallback=t,this}_onChangeCallback(){}*[Symbol.iterator](){yield this._x,yield this._y,yield this._z,yield this._order}}un.DEFAULT_ORDER="XYZ";class Gf{constructor(){this.mask=1}set(t){this.mask=(1<<t|0)>>>0}enable(t){this.mask|=1<<t|0}enableAll(){this.mask=-1}toggle(t){this.mask^=1<<t|0}disable(t){this.mask&=~(1<<t|0)}disableAll(){this.mask=0}test(t){return(this.mask&t.mask)!==0}isEnabled(t){return(this.mask&(1<<t|0))!==0}}let Nm=0;const mu=new R,zs=new ni,Ti=new re,Wa=new R,Nr=new R,Fm=new R,Bm=new ni,gu=new R(1,0,0),xu=new R(0,1,0),_u=new R(0,0,1),Um={type:"added"},zm={type:"removed"};class nn extends Tr{constructor(){super(),this.isObject3D=!0,Object.defineProperty(this,"id",{value:Nm++}),this.uuid=Ar(),this.name="",this.type="Object3D",this.parent=null,this.children=[],this.up=nn.DEFAULT_UP.clone();const t=new R,e=new un,n=new ni,i=new R(1,1,1);function s(){n.setFromEuler(e,!1)}function a(){e.setFromQuaternion(n,void 0,!1)}e._onChange(s),n._onChange(a),Object.defineProperties(this,{position:{configurable:!0,enumerable:!0,value:t},rotation:{configurable:!0,enumerable:!0,value:e},quaternion:{configurable:!0,enumerable:!0,value:n},scale:{configurable:!0,enumerable:!0,value:i},modelViewMatrix:{value:new re},normalMatrix:{value:new ne}}),this.matrix=new re,this.matrixWorld=new re,this.matrixAutoUpdate=nn.DEFAULT_MATRIX_AUTO_UPDATE,this.matrixWorldAutoUpdate=nn.DEFAULT_MATRIX_WORLD_AUTO_UPDATE,this.matrixWorldNeedsUpdate=!1,this.layers=new Gf,this.visible=!0,this.castShadow=!1,this.receiveShadow=!1,this.frustumCulled=!0,this.renderOrder=0,this.animations=[],this.userData={}}onBeforeShadow(){}onAfterShadow(){}onBeforeRender(){}onAfterRender(){}applyMatrix4(t){this.matrixAutoUpdate&&this.updateMatrix(),this.matrix.premultiply(t),this.matrix.decompose(this.position,this.quaternion,this.scale)}applyQuaternion(t){return this.quaternion.premultiply(t),this}setRotationFromAxisAngle(t,e){this.quaternion.setFromAxisAngle(t,e)}setRotationFromEuler(t){this.quaternion.setFromEuler(t,!0)}setRotationFromMatrix(t){this.quaternion.setFromRotationMatrix(t)}setRotationFromQuaternion(t){this.quaternion.copy(t)}rotateOnAxis(t,e){return zs.setFromAxisAngle(t,e),this.quaternion.multiply(zs),this}rotateOnWorldAxis(t,e){return zs.setFromAxisAngle(t,e),this.quaternion.premultiply(zs),this}rotateX(t){return this.rotateOnAxis(gu,t)}rotateY(t){return this.rotateOnAxis(xu,t)}rotateZ(t){return this.rotateOnAxis(_u,t)}translateOnAxis(t,e){return mu.copy(t).applyQuaternion(this.quaternion),this.position.add(mu.multiplyScalar(e)),this}translateX(t){return this.translateOnAxis(gu,t)}translateY(t){return this.translateOnAxis(xu,t)}translateZ(t){return this.translateOnAxis(_u,t)}localToWorld(t){return this.updateWorldMatrix(!0,!1),t.applyMatrix4(this.matrixWorld)}worldToLocal(t){return this.updateWorldMatrix(!0,!1),t.applyMatrix4(Ti.copy(this.matrixWorld).invert())}lookAt(t,e,n){t.isVector3?Wa.copy(t):Wa.set(t,e,n);const i=this.parent;this.updateWorldMatrix(!0,!1),Nr.setFromMatrixPosition(this.matrixWorld),this.isCamera||this.isLight?Ti.lookAt(Nr,Wa,this.up):Ti.lookAt(Wa,Nr,this.up),this.quaternion.setFromRotationMatrix(Ti),i&&(Ti.extractRotation(i.matrixWorld),zs.setFromRotationMatrix(Ti),this.quaternion.premultiply(zs.invert()))}add(t){if(arguments.length>1){for(let e=0;e<arguments.length;e++)this.add(arguments[e]);return this}return t===this?(console.error("THREE.Object3D.add: object can't be added as a child of itself.",t),this):(t&&t.isObject3D?(t.parent!==null&&t.parent.remove(t),t.parent=this,this.children.push(t),t.dispatchEvent(Um)):console.error("THREE.Object3D.add: object not an instance of THREE.Object3D.",t),this)}remove(t){if(arguments.length>1){for(let n=0;n<arguments.length;n++)this.remove(arguments[n]);return this}const e=this.children.indexOf(t);return e!==-1&&(t.parent=null,this.children.splice(e,1),t.dispatchEvent(zm)),this}removeFromParent(){const t=this.parent;return t!==null&&t.remove(this),this}clear(){return this.remove(...this.children)}attach(t){return this.updateWorldMatrix(!0,!1),Ti.copy(this.matrixWorld).invert(),t.parent!==null&&(t.parent.updateWorldMatrix(!0,!1),Ti.multiply(t.parent.matrixWorld)),t.applyMatrix4(Ti),this.add(t),t.updateWorldMatrix(!1,!0),this}getObjectById(t){return this.getObjectByProperty("id",t)}getObjectByName(t){return this.getObjectByProperty("name",t)}getObjectByProperty(t,e){if(this[t]===e)return this;for(let n=0,i=this.children.length;n<i;n++){const a=this.children[n].getObjectByProperty(t,e);if(a!==void 0)return a}}getObjectsByProperty(t,e,n=[]){this[t]===e&&n.push(this);const i=this.children;for(let s=0,a=i.length;s<a;s++)i[s].getObjectsByProperty(t,e,n);return n}getWorldPosition(t){return this.updateWorldMatrix(!0,!1),t.setFromMatrixPosition(this.matrixWorld)}getWorldQuaternion(t){return this.updateWorldMatrix(!0,!1),this.matrixWorld.decompose(Nr,t,Fm),t}getWorldScale(t){return this.updateWorldMatrix(!0,!1),this.matrixWorld.decompose(Nr,Bm,t),t}getWorldDirection(t){this.updateWorldMatrix(!0,!1);const e=this.matrixWorld.elements;return t.set(e[8],e[9],e[10]).normalize()}raycast(){}traverse(t){t(this);const e=this.children;for(let n=0,i=e.length;n<i;n++)e[n].traverse(t)}traverseVisible(t){if(this.visible===!1)return;t(this);const e=this.children;for(let n=0,i=e.length;n<i;n++)e[n].traverseVisible(t)}traverseAncestors(t){const e=this.parent;e!==null&&(t(e),e.traverseAncestors(t))}updateMatrix(){this.matrix.compose(this.position,this.quaternion,this.scale),this.matrixWorldNeedsUpdate=!0}updateMatrixWorld(t){this.matrixAutoUpdate&&this.updateMatrix(),(this.matrixWorldNeedsUpdate||t)&&(this.parent===null?this.matrixWorld.copy(this.matrix):this.matrixWorld.multiplyMatrices(this.parent.matrixWorld,this.matrix),this.matrixWorldNeedsUpdate=!1,t=!0);const e=this.children;for(let n=0,i=e.length;n<i;n++){const s=e[n];(s.matrixWorldAutoUpdate===!0||t===!0)&&s.updateMatrixWorld(t)}}updateWorldMatrix(t,e){const n=this.parent;if(t===!0&&n!==null&&n.matrixWorldAutoUpdate===!0&&n.updateWorldMatrix(!0,!1),this.matrixAutoUpdate&&this.updateMatrix(),this.parent===null?this.matrixWorld.copy(this.matrix):this.matrixWorld.multiplyMatrices(this.parent.matrixWorld,this.matrix),e===!0){const i=this.children;for(let s=0,a=i.length;s<a;s++){const o=i[s];o.matrixWorldAutoUpdate===!0&&o.updateWorldMatrix(!1,!0)}}}toJSON(t){const e=t===void 0||typeof t=="string",n={};e&&(t={geometries:{},materials:{},textures:{},images:{},shapes:{},skeletons:{},animations:{},nodes:{}},n.metadata={version:4.6,type:"Object",generator:"Object3D.toJSON"});const i={};i.uuid=this.uuid,i.type=this.type,this.name!==""&&(i.name=this.name),this.castShadow===!0&&(i.castShadow=!0),this.receiveShadow===!0&&(i.receiveShadow=!0),this.visible===!1&&(i.visible=!1),this.frustumCulled===!1&&(i.frustumCulled=!1),this.renderOrder!==0&&(i.renderOrder=this.renderOrder),Object.keys(this.userData).length>0&&(i.userData=this.userData),i.layers=this.layers.mask,i.matrix=this.matrix.toArray(),i.up=this.up.toArray(),this.matrixAutoUpdate===!1&&(i.matrixAutoUpdate=!1),this.isInstancedMesh&&(i.type="InstancedMesh",i.count=this.count,i.instanceMatrix=this.instanceMatrix.toJSON(),this.instanceColor!==null&&(i.instanceColor=this.instanceColor.toJSON())),this.isBatchedMesh&&(i.type="BatchedMesh",i.perObjectFrustumCulled=this.perObjectFrustumCulled,i.sortObjects=this.sortObjects,i.drawRanges=this._drawRanges,i.reservedRanges=this._reservedRanges,i.visibility=this._visibility,i.active=this._active,i.bounds=this._bounds.map(o=>({boxInitialized:o.boxInitialized,boxMin:o.box.min.toArray(),boxMax:o.box.max.toArray(),sphereInitialized:o.sphereInitialized,sphereRadius:o.sphere.radius,sphereCenter:o.sphere.center.toArray()})),i.maxGeometryCount=this._maxGeometryCount,i.maxVertexCount=this._maxVertexCount,i.maxIndexCount=this._maxIndexCount,i.geometryInitialized=this._geometryInitialized,i.geometryCount=this._geometryCount,i.matricesTexture=this._matricesTexture.toJSON(t),this.boundingSphere!==null&&(i.boundingSphere={center:i.boundingSphere.center.toArray(),radius:i.boundingSphere.radius}),this.boundingBox!==null&&(i.boundingBox={min:i.boundingBox.min.toArray(),max:i.boundingBox.max.toArray()}));function s(o,c){return o[c.uuid]===void 0&&(o[c.uuid]=c.toJSON(t)),c.uuid}if(this.isScene)this.background&&(this.background.isColor?i.background=this.background.toJSON():this.background.isTexture&&(i.background=this.background.toJSON(t).uuid)),this.environment&&this.environment.isTexture&&this.environment.isRenderTargetTexture!==!0&&(i.environment=this.environment.toJSON(t).uuid);else if(this.isMesh||this.isLine||this.isPoints){i.geometry=s(t.geometries,this.geometry);const o=this.geometry.parameters;if(o!==void 0&&o.shapes!==void 0){const c=o.shapes;if(Array.isArray(c))for(let l=0,h=c.length;l<h;l++){const u=c[l];s(t.shapes,u)}else s(t.shapes,c)}}if(this.isSkinnedMesh&&(i.bindMode=this.bindMode,i.bindMatrix=this.bindMatrix.toArray(),this.skeleton!==void 0&&(s(t.skeletons,this.skeleton),i.skeleton=this.skeleton.uuid)),this.material!==void 0)if(Array.isArray(this.material)){const o=[];for(let c=0,l=this.material.length;c<l;c++)o.push(s(t.materials,this.material[c]));i.material=o}else i.material=s(t.materials,this.material);if(this.children.length>0){i.children=[];for(let o=0;o<this.children.length;o++)i.children.push(this.children[o].toJSON(t).object)}if(this.animations.length>0){i.animations=[];for(let o=0;o<this.animations.length;o++){const c=this.animations[o];i.animations.push(s(t.animations,c))}}if(e){const o=a(t.geometries),c=a(t.materials),l=a(t.textures),h=a(t.images),u=a(t.shapes),d=a(t.skeletons),f=a(t.animations),g=a(t.nodes);o.length>0&&(n.geometries=o),c.length>0&&(n.materials=c),l.length>0&&(n.textures=l),h.length>0&&(n.images=h),u.length>0&&(n.shapes=u),d.length>0&&(n.skeletons=d),f.length>0&&(n.animations=f),g.length>0&&(n.nodes=g)}return n.object=i,n;function a(o){const c=[];for(const l in o){const h=o[l];delete h.metadata,c.push(h)}return c}}clone(t){return new this.constructor().copy(this,t)}copy(t,e=!0){if(this.name=t.name,this.up.copy(t.up),this.position.copy(t.position),this.rotation.order=t.rotation.order,this.quaternion.copy(t.quaternion),this.scale.copy(t.scale),this.matrix.copy(t.matrix),this.matrixWorld.copy(t.matrixWorld),this.matrixAutoUpdate=t.matrixAutoUpdate,this.matrixWorldAutoUpdate=t.matrixWorldAutoUpdate,this.matrixWorldNeedsUpdate=t.matrixWorldNeedsUpdate,this.layers.mask=t.layers.mask,this.visible=t.visible,this.castShadow=t.castShadow,this.receiveShadow=t.receiveShadow,this.frustumCulled=t.frustumCulled,this.renderOrder=t.renderOrder,this.animations=t.animations.slice(),this.userData=JSON.parse(JSON.stringify(t.userData)),e===!0)for(let n=0;n<t.children.length;n++){const i=t.children[n];this.add(i.clone())}return this}}nn.DEFAULT_UP=new R(0,1,0);nn.DEFAULT_MATRIX_AUTO_UPDATE=!0;nn.DEFAULT_MATRIX_WORLD_AUTO_UPDATE=!0;const Kn=new R,Ai=new R,zc=new R,Ci=new R,Os=new R,Gs=new R,vu=new R,Oc=new R,Gc=new R,Vc=new R;let Xa=!1;class on{constructor(t=new R,e=new R,n=new R){this.a=t,this.b=e,this.c=n}static getNormal(t,e,n,i){i.subVectors(n,e),Kn.subVectors(t,e),i.cross(Kn);const s=i.lengthSq();return s>0?i.multiplyScalar(1/Math.sqrt(s)):i.set(0,0,0)}static getBarycoord(t,e,n,i,s){Kn.subVectors(i,e),Ai.subVectors(n,e),zc.subVectors(t,e);const a=Kn.dot(Kn),o=Kn.dot(Ai),c=Kn.dot(zc),l=Ai.dot(Ai),h=Ai.dot(zc),u=a*l-o*o;if(u===0)return s.set(0,0,0),null;const d=1/u,f=(l*c-o*h)*d,g=(a*h-o*c)*d;return s.set(1-f-g,g,f)}static containsPoint(t,e,n,i){return this.getBarycoord(t,e,n,i,Ci)===null?!1:Ci.x>=0&&Ci.y>=0&&Ci.x+Ci.y<=1}static getUV(t,e,n,i,s,a,o,c){return Xa===!1&&(console.warn("THREE.Triangle.getUV() has been renamed to THREE.Triangle.getInterpolation()."),Xa=!0),this.getInterpolation(t,e,n,i,s,a,o,c)}static getInterpolation(t,e,n,i,s,a,o,c){return this.getBarycoord(t,e,n,i,Ci)===null?(c.x=0,c.y=0,"z"in c&&(c.z=0),"w"in c&&(c.w=0),null):(c.setScalar(0),c.addScaledVector(s,Ci.x),c.addScaledVector(a,Ci.y),c.addScaledVector(o,Ci.z),c)}static isFrontFacing(t,e,n,i){return Kn.subVectors(n,e),Ai.subVectors(t,e),Kn.cross(Ai).dot(i)<0}set(t,e,n){return this.a.copy(t),this.b.copy(e),this.c.copy(n),this}setFromPointsAndIndices(t,e,n,i){return this.a.copy(t[e]),this.b.copy(t[n]),this.c.copy(t[i]),this}setFromAttributeAndIndices(t,e,n,i){return this.a.fromBufferAttribute(t,e),this.b.fromBufferAttribute(t,n),this.c.fromBufferAttribute(t,i),this}clone(){return new this.constructor().copy(this)}copy(t){return this.a.copy(t.a),this.b.copy(t.b),this.c.copy(t.c),this}getArea(){return Kn.subVectors(this.c,this.b),Ai.subVectors(this.a,this.b),Kn.cross(Ai).length()*.5}getMidpoint(t){return t.addVectors(this.a,this.b).add(this.c).multiplyScalar(1/3)}getNormal(t){return on.getNormal(this.a,this.b,this.c,t)}getPlane(t){return t.setFromCoplanarPoints(this.a,this.b,this.c)}getBarycoord(t,e){return on.getBarycoord(t,this.a,this.b,this.c,e)}getUV(t,e,n,i,s){return Xa===!1&&(console.warn("THREE.Triangle.getUV() has been renamed to THREE.Triangle.getInterpolation()."),Xa=!0),on.getInterpolation(t,this.a,this.b,this.c,e,n,i,s)}getInterpolation(t,e,n,i,s){return on.getInterpolation(t,this.a,this.b,this.c,e,n,i,s)}containsPoint(t){return on.containsPoint(t,this.a,this.b,this.c)}isFrontFacing(t){return on.isFrontFacing(this.a,this.b,this.c,t)}intersectsBox(t){return t.intersectsTriangle(this)}closestPointToPoint(t,e){const n=this.a,i=this.b,s=this.c;let a,o;Os.subVectors(i,n),Gs.subVectors(s,n),Oc.subVectors(t,n);const c=Os.dot(Oc),l=Gs.dot(Oc);if(c<=0&&l<=0)return e.copy(n);Gc.subVectors(t,i);const h=Os.dot(Gc),u=Gs.dot(Gc);if(h>=0&&u<=h)return e.copy(i);const d=c*u-h*l;if(d<=0&&c>=0&&h<=0)return a=c/(c-h),e.copy(n).addScaledVector(Os,a);Vc.subVectors(t,s);const f=Os.dot(Vc),g=Gs.dot(Vc);if(g>=0&&f<=g)return e.copy(s);const x=f*l-c*g;if(x<=0&&l>=0&&g<=0)return o=l/(l-g),e.copy(n).addScaledVector(Gs,o);const m=h*g-f*u;if(m<=0&&u-h>=0&&f-g>=0)return vu.subVectors(s,i),o=(u-h)/(u-h+(f-g)),e.copy(i).addScaledVector(vu,o);const p=1/(m+x+d);return a=x*p,o=d*p,e.copy(n).addScaledVector(Os,a).addScaledVector(Gs,o)}equals(t){return t.a.equals(this.a)&&t.b.equals(this.b)&&t.c.equals(this.c)}}const Vf={aliceblue:15792383,antiquewhite:16444375,aqua:65535,aquamarine:8388564,azure:15794175,beige:16119260,bisque:16770244,black:0,blanchedalmond:16772045,blue:255,blueviolet:9055202,brown:10824234,burlywood:14596231,cadetblue:6266528,chartreuse:8388352,chocolate:13789470,coral:16744272,cornflowerblue:6591981,cornsilk:16775388,crimson:14423100,cyan:65535,darkblue:139,darkcyan:35723,darkgoldenrod:12092939,darkgray:11119017,darkgreen:25600,darkgrey:11119017,darkkhaki:12433259,darkmagenta:9109643,darkolivegreen:5597999,darkorange:16747520,darkorchid:10040012,darkred:9109504,darksalmon:15308410,darkseagreen:9419919,darkslateblue:4734347,darkslategray:3100495,darkslategrey:3100495,darkturquoise:52945,darkviolet:9699539,deeppink:16716947,deepskyblue:49151,dimgray:6908265,dimgrey:6908265,dodgerblue:2003199,firebrick:11674146,floralwhite:16775920,forestgreen:2263842,fuchsia:16711935,gainsboro:14474460,ghostwhite:16316671,gold:16766720,goldenrod:14329120,gray:8421504,green:32768,greenyellow:11403055,grey:8421504,honeydew:15794160,hotpink:16738740,indianred:13458524,indigo:4915330,ivory:16777200,khaki:15787660,lavender:15132410,lavenderblush:16773365,lawngreen:8190976,lemonchiffon:16775885,lightblue:11393254,lightcoral:15761536,lightcyan:14745599,lightgoldenrodyellow:16448210,lightgray:13882323,lightgreen:9498256,lightgrey:13882323,lightpink:16758465,lightsalmon:16752762,lightseagreen:2142890,lightskyblue:8900346,lightslategray:7833753,lightslategrey:7833753,lightsteelblue:11584734,lightyellow:16777184,lime:65280,limegreen:3329330,linen:16445670,magenta:16711935,maroon:8388608,mediumaquamarine:6737322,mediumblue:205,mediumorchid:12211667,mediumpurple:9662683,mediumseagreen:3978097,mediumslateblue:8087790,mediumspringgreen:64154,mediumturquoise:4772300,mediumvioletred:13047173,midnightblue:1644912,mintcream:16121850,mistyrose:16770273,moccasin:16770229,navajowhite:16768685,navy:128,oldlace:16643558,olive:8421376,olivedrab:7048739,orange:16753920,orangered:16729344,orchid:14315734,palegoldenrod:15657130,palegreen:10025880,paleturquoise:11529966,palevioletred:14381203,papayawhip:16773077,peachpuff:16767673,peru:13468991,pink:16761035,plum:14524637,powderblue:11591910,purple:8388736,rebeccapurple:6697881,red:16711680,rosybrown:12357519,royalblue:4286945,saddlebrown:9127187,salmon:16416882,sandybrown:16032864,seagreen:3050327,seashell:16774638,sienna:10506797,silver:12632256,skyblue:8900331,slateblue:6970061,slategray:7372944,slategrey:7372944,snow:16775930,springgreen:65407,steelblue:4620980,tan:13808780,teal:32896,thistle:14204888,tomato:16737095,turquoise:4251856,violet:15631086,wheat:16113331,white:16777215,whitesmoke:16119285,yellow:16776960,yellowgreen:10145074},Zi={h:0,s:0,l:0},Ya={h:0,s:0,l:0};function kc(r,t,e){return e<0&&(e+=1),e>1&&(e-=1),e<1/6?r+(t-r)*6*e:e<1/2?t:e<2/3?r+(t-r)*6*(2/3-e):r}class Qt{constructor(t,e,n){return this.isColor=!0,this.r=1,this.g=1,this.b=1,this.set(t,e,n)}set(t,e,n){if(e===void 0&&n===void 0){const i=t;i&&i.isColor?this.copy(i):typeof i=="number"?this.setHex(i):typeof i=="string"&&this.setStyle(i)}else this.setRGB(t,e,n);return this}setScalar(t){return this.r=t,this.g=t,this.b=t,this}setHex(t,e=an){return t=Math.floor(t),this.r=(t>>16&255)/255,this.g=(t>>8&255)/255,this.b=(t&255)/255,pe.toWorkingColorSpace(this,e),this}setRGB(t,e,n,i=pe.workingColorSpace){return this.r=t,this.g=e,this.b=n,pe.toWorkingColorSpace(this,i),this}setHSL(t,e,n,i=pe.workingColorSpace){if(t=Nh(t,1),e=We(e,0,1),n=We(n,0,1),e===0)this.r=this.g=this.b=n;else{const s=n<=.5?n*(1+e):n+e-n*e,a=2*n-s;this.r=kc(a,s,t+1/3),this.g=kc(a,s,t),this.b=kc(a,s,t-1/3)}return pe.toWorkingColorSpace(this,i),this}setStyle(t,e=an){function n(s){s!==void 0&&parseFloat(s)<1&&console.warn("THREE.Color: Alpha component of "+t+" will be ignored.")}let i;if(i=/^(\w+)\(([^\)]*)\)/.exec(t)){let s;const a=i[1],o=i[2];switch(a){case"rgb":case"rgba":if(s=/^\s*(\d+)\s*,\s*(\d+)\s*,\s*(\d+)\s*(?:,\s*(\d*\.?\d+)\s*)?$/.exec(o))return n(s[4]),this.setRGB(Math.min(255,parseInt(s[1],10))/255,Math.min(255,parseInt(s[2],10))/255,Math.min(255,parseInt(s[3],10))/255,e);if(s=/^\s*(\d+)\%\s*,\s*(\d+)\%\s*,\s*(\d+)\%\s*(?:,\s*(\d*\.?\d+)\s*)?$/.exec(o))return n(s[4]),this.setRGB(Math.min(100,parseInt(s[1],10))/100,Math.min(100,parseInt(s[2],10))/100,Math.min(100,parseInt(s[3],10))/100,e);break;case"hsl":case"hsla":if(s=/^\s*(\d*\.?\d+)\s*,\s*(\d*\.?\d+)\%\s*,\s*(\d*\.?\d+)\%\s*(?:,\s*(\d*\.?\d+)\s*)?$/.exec(o))return n(s[4]),this.setHSL(parseFloat(s[1])/360,parseFloat(s[2])/100,parseFloat(s[3])/100,e);break;default:console.warn("THREE.Color: Unknown color model "+t)}}else if(i=/^\#([A-Fa-f\d]+)$/.exec(t)){const s=i[1],a=s.length;if(a===3)return this.setRGB(parseInt(s.charAt(0),16)/15,parseInt(s.charAt(1),16)/15,parseInt(s.charAt(2),16)/15,e);if(a===6)return this.setHex(parseInt(s,16),e);console.warn("THREE.Color: Invalid hex color "+t)}else if(t&&t.length>0)return this.setColorName(t,e);return this}setColorName(t,e=an){const n=Vf[t.toLowerCase()];return n!==void 0?this.setHex(n,e):console.warn("THREE.Color: Unknown color "+t),this}clone(){return new this.constructor(this.r,this.g,this.b)}copy(t){return this.r=t.r,this.g=t.g,this.b=t.b,this}copySRGBToLinear(t){return this.r=mr(t.r),this.g=mr(t.g),this.b=mr(t.b),this}copyLinearToSRGB(t){return this.r=Pc(t.r),this.g=Pc(t.g),this.b=Pc(t.b),this}convertSRGBToLinear(){return this.copySRGBToLinear(this),this}convertLinearToSRGB(){return this.copyLinearToSRGB(this),this}getHex(t=an){return pe.fromWorkingColorSpace(ln.copy(this),t),Math.round(We(ln.r*255,0,255))*65536+Math.round(We(ln.g*255,0,255))*256+Math.round(We(ln.b*255,0,255))}getHexString(t=an){return("000000"+this.getHex(t).toString(16)).slice(-6)}getHSL(t,e=pe.workingColorSpace){pe.fromWorkingColorSpace(ln.copy(this),e);const n=ln.r,i=ln.g,s=ln.b,a=Math.max(n,i,s),o=Math.min(n,i,s);let c,l;const h=(o+a)/2;if(o===a)c=0,l=0;else{const u=a-o;switch(l=h<=.5?u/(a+o):u/(2-a-o),a){case n:c=(i-s)/u+(i<s?6:0);break;case i:c=(s-n)/u+2;break;case s:c=(n-i)/u+4;break}c/=6}return t.h=c,t.s=l,t.l=h,t}getRGB(t,e=pe.workingColorSpace){return pe.fromWorkingColorSpace(ln.copy(this),e),t.r=ln.r,t.g=ln.g,t.b=ln.b,t}getStyle(t=an){pe.fromWorkingColorSpace(ln.copy(this),t);const e=ln.r,n=ln.g,i=ln.b;return t!==an?`color(${t} ${e.toFixed(3)} ${n.toFixed(3)} ${i.toFixed(3)})`:`rgb(${Math.round(e*255)},${Math.round(n*255)},${Math.round(i*255)})`}offsetHSL(t,e,n){return this.getHSL(Zi),this.setHSL(Zi.h+t,Zi.s+e,Zi.l+n)}add(t){return this.r+=t.r,this.g+=t.g,this.b+=t.b,this}addColors(t,e){return this.r=t.r+e.r,this.g=t.g+e.g,this.b=t.b+e.b,this}addScalar(t){return this.r+=t,this.g+=t,this.b+=t,this}sub(t){return this.r=Math.max(0,this.r-t.r),this.g=Math.max(0,this.g-t.g),this.b=Math.max(0,this.b-t.b),this}multiply(t){return this.r*=t.r,this.g*=t.g,this.b*=t.b,this}multiplyScalar(t){return this.r*=t,this.g*=t,this.b*=t,this}lerp(t,e){return this.r+=(t.r-this.r)*e,this.g+=(t.g-this.g)*e,this.b+=(t.b-this.b)*e,this}lerpColors(t,e,n){return this.r=t.r+(e.r-t.r)*n,this.g=t.g+(e.g-t.g)*n,this.b=t.b+(e.b-t.b)*n,this}lerpHSL(t,e){this.getHSL(Zi),t.getHSL(Ya);const n=Sa(Zi.h,Ya.h,e),i=Sa(Zi.s,Ya.s,e),s=Sa(Zi.l,Ya.l,e);return this.setHSL(n,i,s),this}setFromVector3(t){return this.r=t.x,this.g=t.y,this.b=t.z,this}applyMatrix3(t){const e=this.r,n=this.g,i=this.b,s=t.elements;return this.r=s[0]*e+s[3]*n+s[6]*i,this.g=s[1]*e+s[4]*n+s[7]*i,this.b=s[2]*e+s[5]*n+s[8]*i,this}equals(t){return t.r===this.r&&t.g===this.g&&t.b===this.b}fromArray(t,e=0){return this.r=t[e],this.g=t[e+1],this.b=t[e+2],this}toArray(t=[],e=0){return t[e]=this.r,t[e+1]=this.g,t[e+2]=this.b,t}fromBufferAttribute(t,e){return this.r=t.getX(e),this.g=t.getY(e),this.b=t.getZ(e),this}toJSON(){return this.getHex()}*[Symbol.iterator](){yield this.r,yield this.g,yield this.b}}const ln=new Qt;Qt.NAMES=Vf;let Om=0;class _i extends Tr{constructor(){super(),this.isMaterial=!0,Object.defineProperty(this,"id",{value:Om++}),this.uuid=Ar(),this.name="",this.type="Material",this.blending=1,this.side=0,this.vertexColors=!1,this.opacity=1,this.transparent=!1,this.alphaHash=!1,this.blendSrc=204,this.blendDst=205,this.blendEquation=100,this.blendSrcAlpha=null,this.blendDstAlpha=null,this.blendEquationAlpha=null,this.blendColor=new Qt(0,0,0),this.blendAlpha=0,this.depthFunc=3,this.depthTest=!0,this.depthWrite=!0,this.stencilWriteMask=255,this.stencilFunc=519,this.stencilRef=0,this.stencilFuncMask=255,this.stencilFail=7680,this.stencilZFail=7680,this.stencilZPass=7680,this.stencilWrite=!1,this.clippingPlanes=null,this.clipIntersection=!1,this.clipShadows=!1,this.shadowSide=null,this.colorWrite=!0,this.precision=null,this.polygonOffset=!1,this.polygonOffsetFactor=0,this.polygonOffsetUnits=0,this.dithering=!1,this.alphaToCoverage=!1,this.premultipliedAlpha=!1,this.forceSinglePass=!1,this.visible=!0,this.toneMapped=!0,this.userData={},this.version=0,this._alphaTest=0}get alphaTest(){return this._alphaTest}set alphaTest(t){this._alphaTest>0!=t>0&&this.version++,this._alphaTest=t}onBuild(){}onBeforeRender(){}onBeforeCompile(){}customProgramCacheKey(){return this.onBeforeCompile.toString()}setValues(t){if(t!==void 0)for(const e in t){const n=t[e];if(n===void 0){console.warn(`THREE.Material: parameter '${e}' has value of undefined.`);continue}const i=this[e];if(i===void 0){console.warn(`THREE.Material: '${e}' is not a property of THREE.${this.type}.`);continue}i&&i.isColor?i.set(n):i&&i.isVector3&&n&&n.isVector3?i.copy(n):this[e]=n}}toJSON(t){const e=t===void 0||typeof t=="string";e&&(t={textures:{},images:{}});const n={metadata:{version:4.6,type:"Material",generator:"Material.toJSON"}};n.uuid=this.uuid,n.type=this.type,this.name!==""&&(n.name=this.name),this.color&&this.color.isColor&&(n.color=this.color.getHex()),this.roughness!==void 0&&(n.roughness=this.roughness),this.metalness!==void 0&&(n.metalness=this.metalness),this.sheen!==void 0&&(n.sheen=this.sheen),this.sheenColor&&this.sheenColor.isColor&&(n.sheenColor=this.sheenColor.getHex()),this.sheenRoughness!==void 0&&(n.sheenRoughness=this.sheenRoughness),this.emissive&&this.emissive.isColor&&(n.emissive=this.emissive.getHex()),this.emissiveIntensity&&this.emissiveIntensity!==1&&(n.emissiveIntensity=this.emissiveIntensity),this.specular&&this.specular.isColor&&(n.specular=this.specular.getHex()),this.specularIntensity!==void 0&&(n.specularIntensity=this.specularIntensity),this.specularColor&&this.specularColor.isColor&&(n.specularColor=this.specularColor.getHex()),this.shininess!==void 0&&(n.shininess=this.shininess),this.clearcoat!==void 0&&(n.clearcoat=this.clearcoat),this.clearcoatRoughness!==void 0&&(n.clearcoatRoughness=this.clearcoatRoughness),this.clearcoatMap&&this.clearcoatMap.isTexture&&(n.clearcoatMap=this.clearcoatMap.toJSON(t).uuid),this.clearcoatRoughnessMap&&this.clearcoatRoughnessMap.isTexture&&(n.clearcoatRoughnessMap=this.clearcoatRoughnessMap.toJSON(t).uuid),this.clearcoatNormalMap&&this.clearcoatNormalMap.isTexture&&(n.clearcoatNormalMap=this.clearcoatNormalMap.toJSON(t).uuid,n.clearcoatNormalScale=this.clearcoatNormalScale.toArray()),this.iridescence!==void 0&&(n.iridescence=this.iridescence),this.iridescenceIOR!==void 0&&(n.iridescenceIOR=this.iridescenceIOR),this.iridescenceThicknessRange!==void 0&&(n.iridescenceThicknessRange=this.iridescenceThicknessRange),this.iridescenceMap&&this.iridescenceMap.isTexture&&(n.iridescenceMap=this.iridescenceMap.toJSON(t).uuid),this.iridescenceThicknessMap&&this.iridescenceThicknessMap.isTexture&&(n.iridescenceThicknessMap=this.iridescenceThicknessMap.toJSON(t).uuid),this.anisotropy!==void 0&&(n.anisotropy=this.anisotropy),this.anisotropyRotation!==void 0&&(n.anisotropyRotation=this.anisotropyRotation),this.anisotropyMap&&this.anisotropyMap.isTexture&&(n.anisotropyMap=this.anisotropyMap.toJSON(t).uuid),this.map&&this.map.isTexture&&(n.map=this.map.toJSON(t).uuid),this.matcap&&this.matcap.isTexture&&(n.matcap=this.matcap.toJSON(t).uuid),this.alphaMap&&this.alphaMap.isTexture&&(n.alphaMap=this.alphaMap.toJSON(t).uuid),this.lightMap&&this.lightMap.isTexture&&(n.lightMap=this.lightMap.toJSON(t).uuid,n.lightMapIntensity=this.lightMapIntensity),this.aoMap&&this.aoMap.isTexture&&(n.aoMap=this.aoMap.toJSON(t).uuid,n.aoMapIntensity=this.aoMapIntensity),this.bumpMap&&this.bumpMap.isTexture&&(n.bumpMap=this.bumpMap.toJSON(t).uuid,n.bumpScale=this.bumpScale),this.normalMap&&this.normalMap.isTexture&&(n.normalMap=this.normalMap.toJSON(t).uuid,n.normalMapType=this.normalMapType,n.normalScale=this.normalScale.toArray()),this.displacementMap&&this.displacementMap.isTexture&&(n.displacementMap=this.displacementMap.toJSON(t).uuid,n.displacementScale=this.displacementScale,n.displacementBias=this.displacementBias),this.roughnessMap&&this.roughnessMap.isTexture&&(n.roughnessMap=this.roughnessMap.toJSON(t).uuid),this.metalnessMap&&this.metalnessMap.isTexture&&(n.metalnessMap=this.metalnessMap.toJSON(t).uuid),this.emissiveMap&&this.emissiveMap.isTexture&&(n.emissiveMap=this.emissiveMap.toJSON(t).uuid),this.specularMap&&this.specularMap.isTexture&&(n.specularMap=this.specularMap.toJSON(t).uuid),this.specularIntensityMap&&this.specularIntensityMap.isTexture&&(n.specularIntensityMap=this.specularIntensityMap.toJSON(t).uuid),this.specularColorMap&&this.specularColorMap.isTexture&&(n.specularColorMap=this.specularColorMap.toJSON(t).uuid),this.envMap&&this.envMap.isTexture&&(n.envMap=this.envMap.toJSON(t).uuid,this.combine!==void 0&&(n.combine=this.combine)),this.envMapIntensity!==void 0&&(n.envMapIntensity=this.envMapIntensity),this.reflectivity!==void 0&&(n.reflectivity=this.reflectivity),this.refractionRatio!==void 0&&(n.refractionRatio=this.refractionRatio),this.gradientMap&&this.gradientMap.isTexture&&(n.gradientMap=this.gradientMap.toJSON(t).uuid),this.transmission!==void 0&&(n.transmission=this.transmission),this.transmissionMap&&this.transmissionMap.isTexture&&(n.transmissionMap=this.transmissionMap.toJSON(t).uuid),this.thickness!==void 0&&(n.thickness=this.thickness),this.thicknessMap&&this.thicknessMap.isTexture&&(n.thicknessMap=this.thicknessMap.toJSON(t).uuid),this.attenuationDistance!==void 0&&this.attenuationDistance!==1/0&&(n.attenuationDistance=this.attenuationDistance),this.attenuationColor!==void 0&&(n.attenuationColor=this.attenuationColor.getHex()),this.size!==void 0&&(n.size=this.size),this.shadowSide!==null&&(n.shadowSide=this.shadowSide),this.sizeAttenuation!==void 0&&(n.sizeAttenuation=this.sizeAttenuation),this.blending!==1&&(n.blending=this.blending),this.side!==0&&(n.side=this.side),this.vertexColors===!0&&(n.vertexColors=!0),this.opacity<1&&(n.opacity=this.opacity),this.transparent===!0&&(n.transparent=!0),this.blendSrc!==204&&(n.blendSrc=this.blendSrc),this.blendDst!==205&&(n.blendDst=this.blendDst),this.blendEquation!==100&&(n.blendEquation=this.blendEquation),this.blendSrcAlpha!==null&&(n.blendSrcAlpha=this.blendSrcAlpha),this.blendDstAlpha!==null&&(n.blendDstAlpha=this.blendDstAlpha),this.blendEquationAlpha!==null&&(n.blendEquationAlpha=this.blendEquationAlpha),this.blendColor&&this.blendColor.isColor&&(n.blendColor=this.blendColor.getHex()),this.blendAlpha!==0&&(n.blendAlpha=this.blendAlpha),this.depthFunc!==3&&(n.depthFunc=this.depthFunc),this.depthTest===!1&&(n.depthTest=this.depthTest),this.depthWrite===!1&&(n.depthWrite=this.depthWrite),this.colorWrite===!1&&(n.colorWrite=this.colorWrite),this.stencilWriteMask!==255&&(n.stencilWriteMask=this.stencilWriteMask),this.stencilFunc!==519&&(n.stencilFunc=this.stencilFunc),this.stencilRef!==0&&(n.stencilRef=this.stencilRef),this.stencilFuncMask!==255&&(n.stencilFuncMask=this.stencilFuncMask),this.stencilFail!==7680&&(n.stencilFail=this.stencilFail),this.stencilZFail!==7680&&(n.stencilZFail=this.stencilZFail),this.stencilZPass!==7680&&(n.stencilZPass=this.stencilZPass),this.stencilWrite===!0&&(n.stencilWrite=this.stencilWrite),this.rotation!==void 0&&this.rotation!==0&&(n.rotation=this.rotation),this.polygonOffset===!0&&(n.polygonOffset=!0),this.polygonOffsetFactor!==0&&(n.polygonOffsetFactor=this.polygonOffsetFactor),this.polygonOffsetUnits!==0&&(n.polygonOffsetUnits=this.polygonOffsetUnits),this.linewidth!==void 0&&this.linewidth!==1&&(n.linewidth=this.linewidth),this.dashSize!==void 0&&(n.dashSize=this.dashSize),this.gapSize!==void 0&&(n.gapSize=this.gapSize),this.scale!==void 0&&(n.scale=this.scale),this.dithering===!0&&(n.dithering=!0),this.alphaTest>0&&(n.alphaTest=this.alphaTest),this.alphaHash===!0&&(n.alphaHash=!0),this.alphaToCoverage===!0&&(n.alphaToCoverage=!0),this.premultipliedAlpha===!0&&(n.premultipliedAlpha=!0),this.forceSinglePass===!0&&(n.forceSinglePass=!0),this.wireframe===!0&&(n.wireframe=!0),this.wireframeLinewidth>1&&(n.wireframeLinewidth=this.wireframeLinewidth),this.wireframeLinecap!=="round"&&(n.wireframeLinecap=this.wireframeLinecap),this.wireframeLinejoin!=="round"&&(n.wireframeLinejoin=this.wireframeLinejoin),this.flatShading===!0&&(n.flatShading=!0),this.visible===!1&&(n.visible=!1),this.toneMapped===!1&&(n.toneMapped=!1),this.fog===!1&&(n.fog=!1),Object.keys(this.userData).length>0&&(n.userData=this.userData);function i(s){const a=[];for(const o in s){const c=s[o];delete c.metadata,a.push(c)}return a}if(e){const s=i(t.textures),a=i(t.images);s.length>0&&(n.textures=s),a.length>0&&(n.images=a)}return n}clone(){return new this.constructor().copy(this)}copy(t){this.name=t.name,this.blending=t.blending,this.side=t.side,this.vertexColors=t.vertexColors,this.opacity=t.opacity,this.transparent=t.transparent,this.blendSrc=t.blendSrc,this.blendDst=t.blendDst,this.blendEquation=t.blendEquation,this.blendSrcAlpha=t.blendSrcAlpha,this.blendDstAlpha=t.blendDstAlpha,this.blendEquationAlpha=t.blendEquationAlpha,this.blendColor.copy(t.blendColor),this.blendAlpha=t.blendAlpha,this.depthFunc=t.depthFunc,this.depthTest=t.depthTest,this.depthWrite=t.depthWrite,this.stencilWriteMask=t.stencilWriteMask,this.stencilFunc=t.stencilFunc,this.stencilRef=t.stencilRef,this.stencilFuncMask=t.stencilFuncMask,this.stencilFail=t.stencilFail,this.stencilZFail=t.stencilZFail,this.stencilZPass=t.stencilZPass,this.stencilWrite=t.stencilWrite;const e=t.clippingPlanes;let n=null;if(e!==null){const i=e.length;n=new Array(i);for(let s=0;s!==i;++s)n[s]=e[s].clone()}return this.clippingPlanes=n,this.clipIntersection=t.clipIntersection,this.clipShadows=t.clipShadows,this.shadowSide=t.shadowSide,this.colorWrite=t.colorWrite,this.precision=t.precision,this.polygonOffset=t.polygonOffset,this.polygonOffsetFactor=t.polygonOffsetFactor,this.polygonOffsetUnits=t.polygonOffsetUnits,this.dithering=t.dithering,this.alphaTest=t.alphaTest,this.alphaHash=t.alphaHash,this.alphaToCoverage=t.alphaToCoverage,this.premultipliedAlpha=t.premultipliedAlpha,this.forceSinglePass=t.forceSinglePass,this.visible=t.visible,this.toneMapped=t.toneMapped,this.userData=JSON.parse(JSON.stringify(t.userData)),this}dispose(){this.dispatchEvent({type:"dispose"})}set needsUpdate(t){t===!0&&this.version++}}class Fe extends _i{constructor(t){super(),this.isMeshBasicMaterial=!0,this.type="MeshBasicMaterial",this.color=new Qt(16777215),this.map=null,this.lightMap=null,this.lightMapIntensity=1,this.aoMap=null,this.aoMapIntensity=1,this.specularMap=null,this.alphaMap=null,this.envMap=null,this.combine=0,this.reflectivity=1,this.refractionRatio=.98,this.wireframe=!1,this.wireframeLinewidth=1,this.wireframeLinecap="round",this.wireframeLinejoin="round",this.fog=!0,this.setValues(t)}copy(t){return super.copy(t),this.color.copy(t.color),this.map=t.map,this.lightMap=t.lightMap,this.lightMapIntensity=t.lightMapIntensity,this.aoMap=t.aoMap,this.aoMapIntensity=t.aoMapIntensity,this.specularMap=t.specularMap,this.alphaMap=t.alphaMap,this.envMap=t.envMap,this.combine=t.combine,this.reflectivity=t.reflectivity,this.refractionRatio=t.refractionRatio,this.wireframe=t.wireframe,this.wireframeLinewidth=t.wireframeLinewidth,this.wireframeLinecap=t.wireframeLinecap,this.wireframeLinejoin=t.wireframeLinejoin,this.fog=t.fog,this}}const Oe=new R,qa=new Mt;class Be{constructor(t,e,n=!1){if(Array.isArray(t))throw new TypeError("THREE.BufferAttribute: array should be a Typed Array.");this.isBufferAttribute=!0,this.name="",this.array=t,this.itemSize=e,this.count=t!==void 0?t.length/e:0,this.normalized=n,this.usage=35044,this._updateRange={offset:0,count:-1},this.updateRanges=[],this.gpuType=1015,this.version=0}onUploadCallback(){}set needsUpdate(t){t===!0&&this.version++}get updateRange(){return console.warn("THREE.BufferAttribute: updateRange() is deprecated and will be removed in r169. Use addUpdateRange() instead."),this._updateRange}setUsage(t){return this.usage=t,this}addUpdateRange(t,e){this.updateRanges.push({start:t,count:e})}clearUpdateRanges(){this.updateRanges.length=0}copy(t){return this.name=t.name,this.array=new t.array.constructor(t.array),this.itemSize=t.itemSize,this.count=t.count,this.normalized=t.normalized,this.usage=t.usage,this.gpuType=t.gpuType,this}copyAt(t,e,n){t*=this.itemSize,n*=e.itemSize;for(let i=0,s=this.itemSize;i<s;i++)this.array[t+i]=e.array[n+i];return this}copyArray(t){return this.array.set(t),this}applyMatrix3(t){if(this.itemSize===2)for(let e=0,n=this.count;e<n;e++)qa.fromBufferAttribute(this,e),qa.applyMatrix3(t),this.setXY(e,qa.x,qa.y);else if(this.itemSize===3)for(let e=0,n=this.count;e<n;e++)Oe.fromBufferAttribute(this,e),Oe.applyMatrix3(t),this.setXYZ(e,Oe.x,Oe.y,Oe.z);return this}applyMatrix4(t){for(let e=0,n=this.count;e<n;e++)Oe.fromBufferAttribute(this,e),Oe.applyMatrix4(t),this.setXYZ(e,Oe.x,Oe.y,Oe.z);return this}applyNormalMatrix(t){for(let e=0,n=this.count;e<n;e++)Oe.fromBufferAttribute(this,e),Oe.applyNormalMatrix(t),this.setXYZ(e,Oe.x,Oe.y,Oe.z);return this}transformDirection(t){for(let e=0,n=this.count;e<n;e++)Oe.fromBufferAttribute(this,e),Oe.transformDirection(t),this.setXYZ(e,Oe.x,Oe.y,Oe.z);return this}set(t,e=0){return this.array.set(t,e),this}getComponent(t,e){let n=this.array[t*this.itemSize+e];return this.normalized&&(n=ar(n,this.array)),n}setComponent(t,e,n){return this.normalized&&(n=mn(n,this.array)),this.array[t*this.itemSize+e]=n,this}getX(t){let e=this.array[t*this.itemSize];return this.normalized&&(e=ar(e,this.array)),e}setX(t,e){return this.normalized&&(e=mn(e,this.array)),this.array[t*this.itemSize]=e,this}getY(t){let e=this.array[t*this.itemSize+1];return this.normalized&&(e=ar(e,this.array)),e}setY(t,e){return this.normalized&&(e=mn(e,this.array)),this.array[t*this.itemSize+1]=e,this}getZ(t){let e=this.array[t*this.itemSize+2];return this.normalized&&(e=ar(e,this.array)),e}setZ(t,e){return this.normalized&&(e=mn(e,this.array)),this.array[t*this.itemSize+2]=e,this}getW(t){let e=this.array[t*this.itemSize+3];return this.normalized&&(e=ar(e,this.array)),e}setW(t,e){return this.normalized&&(e=mn(e,this.array)),this.array[t*this.itemSize+3]=e,this}setXY(t,e,n){return t*=this.itemSize,this.normalized&&(e=mn(e,this.array),n=mn(n,this.array)),this.array[t+0]=e,this.array[t+1]=n,this}setXYZ(t,e,n,i){return t*=this.itemSize,this.normalized&&(e=mn(e,this.array),n=mn(n,this.array),i=mn(i,this.array)),this.array[t+0]=e,this.array[t+1]=n,this.array[t+2]=i,this}setXYZW(t,e,n,i,s){return t*=this.itemSize,this.normalized&&(e=mn(e,this.array),n=mn(n,this.array),i=mn(i,this.array),s=mn(s,this.array)),this.array[t+0]=e,this.array[t+1]=n,this.array[t+2]=i,this.array[t+3]=s,this}onUpload(t){return this.onUploadCallback=t,this}clone(){return new this.constructor(this.array,this.itemSize).copy(this)}toJSON(){const t={itemSize:this.itemSize,type:this.array.constructor.name,array:Array.from(this.array),normalized:this.normalized};return this.name!==""&&(t.name=this.name),this.usage!==35044&&(t.usage=this.usage),t}}class kf extends Be{constructor(t,e,n){super(new Uint16Array(t),e,n)}}class Hf extends Be{constructor(t,e,n){super(new Uint32Array(t),e,n)}}class ie extends Be{constructor(t,e,n){super(new Float32Array(t),e,n)}}let Gm=0;const On=new re,Hc=new nn,Vs=new R,Pn=new rn,Fr=new rn,Qe=new R;class me extends Tr{constructor(){super(),this.isBufferGeometry=!0,Object.defineProperty(this,"id",{value:Gm++}),this.uuid=Ar(),this.name="",this.type="BufferGeometry",this.index=null,this.attributes={},this.morphAttributes={},this.morphTargetsRelative=!1,this.groups=[],this.boundingBox=null,this.boundingSphere=null,this.drawRange={start:0,count:1/0},this.userData={}}getIndex(){return this.index}setIndex(t){return Array.isArray(t)?this.index=new(Bf(t)?Hf:kf)(t,1):this.index=t,this}getAttribute(t){return this.attributes[t]}setAttribute(t,e){return this.attributes[t]=e,this}deleteAttribute(t){return delete this.attributes[t],this}hasAttribute(t){return this.attributes[t]!==void 0}addGroup(t,e,n=0){this.groups.push({start:t,count:e,materialIndex:n})}clearGroups(){this.groups=[]}setDrawRange(t,e){this.drawRange.start=t,this.drawRange.count=e}applyMatrix4(t){const e=this.attributes.position;e!==void 0&&(e.applyMatrix4(t),e.needsUpdate=!0);const n=this.attributes.normal;if(n!==void 0){const s=new ne().getNormalMatrix(t);n.applyNormalMatrix(s),n.needsUpdate=!0}const i=this.attributes.tangent;return i!==void 0&&(i.transformDirection(t),i.needsUpdate=!0),this.boundingBox!==null&&this.computeBoundingBox(),this.boundingSphere!==null&&this.computeBoundingSphere(),this}applyQuaternion(t){return On.makeRotationFromQuaternion(t),this.applyMatrix4(On),this}rotateX(t){return On.makeRotationX(t),this.applyMatrix4(On),this}rotateY(t){return On.makeRotationY(t),this.applyMatrix4(On),this}rotateZ(t){return On.makeRotationZ(t),this.applyMatrix4(On),this}translate(t,e,n){return On.makeTranslation(t,e,n),this.applyMatrix4(On),this}scale(t,e,n){return On.makeScale(t,e,n),this.applyMatrix4(On),this}lookAt(t){return Hc.lookAt(t),Hc.updateMatrix(),this.applyMatrix4(Hc.matrix),this}center(){return this.computeBoundingBox(),this.boundingBox.getCenter(Vs).negate(),this.translate(Vs.x,Vs.y,Vs.z),this}setFromPoints(t){const e=[];for(let n=0,i=t.length;n<i;n++){const s=t[n];e.push(s.x,s.y,s.z||0)}return this.setAttribute("position",new ie(e,3)),this}computeBoundingBox(){this.boundingBox===null&&(this.boundingBox=new rn);const t=this.attributes.position,e=this.morphAttributes.position;if(t&&t.isGLBufferAttribute){console.error('THREE.BufferGeometry.computeBoundingBox(): GLBufferAttribute requires a manual bounding box. Alternatively set "mesh.frustumCulled" to "false".',this),this.boundingBox.set(new R(-1/0,-1/0,-1/0),new R(1/0,1/0,1/0));return}if(t!==void 0){if(this.boundingBox.setFromBufferAttribute(t),e)for(let n=0,i=e.length;n<i;n++){const s=e[n];Pn.setFromBufferAttribute(s),this.morphTargetsRelative?(Qe.addVectors(this.boundingBox.min,Pn.min),this.boundingBox.expandByPoint(Qe),Qe.addVectors(this.boundingBox.max,Pn.max),this.boundingBox.expandByPoint(Qe)):(this.boundingBox.expandByPoint(Pn.min),this.boundingBox.expandByPoint(Pn.max))}}else this.boundingBox.makeEmpty();(isNaN(this.boundingBox.min.x)||isNaN(this.boundingBox.min.y)||isNaN(this.boundingBox.min.z))&&console.error('THREE.BufferGeometry.computeBoundingBox(): Computed min/max have NaN values. The "position" attribute is likely to have NaN values.',this)}computeBoundingSphere(){this.boundingSphere===null&&(this.boundingSphere=new as);const t=this.attributes.position,e=this.morphAttributes.position;if(t&&t.isGLBufferAttribute){console.error('THREE.BufferGeometry.computeBoundingSphere(): GLBufferAttribute requires a manual bounding sphere. Alternatively set "mesh.frustumCulled" to "false".',this),this.boundingSphere.set(new R,1/0);return}if(t){const n=this.boundingSphere.center;if(Pn.setFromBufferAttribute(t),e)for(let s=0,a=e.length;s<a;s++){const o=e[s];Fr.setFromBufferAttribute(o),this.morphTargetsRelative?(Qe.addVectors(Pn.min,Fr.min),Pn.expandByPoint(Qe),Qe.addVectors(Pn.max,Fr.max),Pn.expandByPoint(Qe)):(Pn.expandByPoint(Fr.min),Pn.expandByPoint(Fr.max))}Pn.getCenter(n);let i=0;for(let s=0,a=t.count;s<a;s++)Qe.fromBufferAttribute(t,s),i=Math.max(i,n.distanceToSquared(Qe));if(e)for(let s=0,a=e.length;s<a;s++){const o=e[s],c=this.morphTargetsRelative;for(let l=0,h=o.count;l<h;l++)Qe.fromBufferAttribute(o,l),c&&(Vs.fromBufferAttribute(t,l),Qe.add(Vs)),i=Math.max(i,n.distanceToSquared(Qe))}this.boundingSphere.radius=Math.sqrt(i),isNaN(this.boundingSphere.radius)&&console.error('THREE.BufferGeometry.computeBoundingSphere(): Computed radius is NaN. The "position" attribute is likely to have NaN values.',this)}}computeTangents(){const t=this.index,e=this.attributes;if(t===null||e.position===void 0||e.normal===void 0||e.uv===void 0){console.error("THREE.BufferGeometry: .computeTangents() failed. Missing required attributes (index, position, normal or uv)");return}const n=t.array,i=e.position.array,s=e.normal.array,a=e.uv.array,o=i.length/3;this.hasAttribute("tangent")===!1&&this.setAttribute("tangent",new Be(new Float32Array(4*o),4));const c=this.getAttribute("tangent").array,l=[],h=[];for(let T=0;T<o;T++)l[T]=new R,h[T]=new R;const u=new R,d=new R,f=new R,g=new Mt,x=new Mt,m=new Mt,p=new R,_=new R;function v(T,L,C){u.fromArray(i,T*3),d.fromArray(i,L*3),f.fromArray(i,C*3),g.fromArray(a,T*2),x.fromArray(a,L*2),m.fromArray(a,C*2),d.sub(u),f.sub(u),x.sub(g),m.sub(g);const F=1/(x.x*m.y-m.x*x.y);isFinite(F)&&(p.copy(d).multiplyScalar(m.y).addScaledVector(f,-x.y).multiplyScalar(F),_.copy(f).multiplyScalar(x.x).addScaledVector(d,-m.x).multiplyScalar(F),l[T].add(p),l[L].add(p),l[C].add(p),h[T].add(_),h[L].add(_),h[C].add(_))}let S=this.groups;S.length===0&&(S=[{start:0,count:n.length}]);for(let T=0,L=S.length;T<L;++T){const C=S[T],F=C.start,D=C.count;for(let N=F,B=F+D;N<B;N+=3)v(n[N+0],n[N+1],n[N+2])}const M=new R,y=new R,w=new R,A=new R;function E(T){w.fromArray(s,T*3),A.copy(w);const L=l[T];M.copy(L),M.sub(w.multiplyScalar(w.dot(L))).normalize(),y.crossVectors(A,L);const F=y.dot(h[T])<0?-1:1;c[T*4]=M.x,c[T*4+1]=M.y,c[T*4+2]=M.z,c[T*4+3]=F}for(let T=0,L=S.length;T<L;++T){const C=S[T],F=C.start,D=C.count;for(let N=F,B=F+D;N<B;N+=3)E(n[N+0]),E(n[N+1]),E(n[N+2])}}computeVertexNormals(){const t=this.index,e=this.getAttribute("position");if(e!==void 0){let n=this.getAttribute("normal");if(n===void 0)n=new Be(new Float32Array(e.count*3),3),this.setAttribute("normal",n);else for(let d=0,f=n.count;d<f;d++)n.setXYZ(d,0,0,0);const i=new R,s=new R,a=new R,o=new R,c=new R,l=new R,h=new R,u=new R;if(t)for(let d=0,f=t.count;d<f;d+=3){const g=t.getX(d+0),x=t.getX(d+1),m=t.getX(d+2);i.fromBufferAttribute(e,g),s.fromBufferAttribute(e,x),a.fromBufferAttribute(e,m),h.subVectors(a,s),u.subVectors(i,s),h.cross(u),o.fromBufferAttribute(n,g),c.fromBufferAttribute(n,x),l.fromBufferAttribute(n,m),o.add(h),c.add(h),l.add(h),n.setXYZ(g,o.x,o.y,o.z),n.setXYZ(x,c.x,c.y,c.z),n.setXYZ(m,l.x,l.y,l.z)}else for(let d=0,f=e.count;d<f;d+=3)i.fromBufferAttribute(e,d+0),s.fromBufferAttribute(e,d+1),a.fromBufferAttribute(e,d+2),h.subVectors(a,s),u.subVectors(i,s),h.cross(u),n.setXYZ(d+0,h.x,h.y,h.z),n.setXYZ(d+1,h.x,h.y,h.z),n.setXYZ(d+2,h.x,h.y,h.z);this.normalizeNormals(),n.needsUpdate=!0}}normalizeNormals(){const t=this.attributes.normal;for(let e=0,n=t.count;e<n;e++)Qe.fromBufferAttribute(t,e),Qe.normalize(),t.setXYZ(e,Qe.x,Qe.y,Qe.z)}toNonIndexed(){function t(o,c){const l=o.array,h=o.itemSize,u=o.normalized,d=new l.constructor(c.length*h);let f=0,g=0;for(let x=0,m=c.length;x<m;x++){o.isInterleavedBufferAttribute?f=c[x]*o.data.stride+o.offset:f=c[x]*h;for(let p=0;p<h;p++)d[g++]=l[f++]}return new Be(d,h,u)}if(this.index===null)return console.warn("THREE.BufferGeometry.toNonIndexed(): BufferGeometry is already non-indexed."),this;const e=new me,n=this.index.array,i=this.attributes;for(const o in i){const c=i[o],l=t(c,n);e.setAttribute(o,l)}const s=this.morphAttributes;for(const o in s){const c=[],l=s[o];for(let h=0,u=l.length;h<u;h++){const d=l[h],f=t(d,n);c.push(f)}e.morphAttributes[o]=c}e.morphTargetsRelative=this.morphTargetsRelative;const a=this.groups;for(let o=0,c=a.length;o<c;o++){const l=a[o];e.addGroup(l.start,l.count,l.materialIndex)}return e}toJSON(){const t={metadata:{version:4.6,type:"BufferGeometry",generator:"BufferGeometry.toJSON"}};if(t.uuid=this.uuid,t.type=this.type,this.name!==""&&(t.name=this.name),Object.keys(this.userData).length>0&&(t.userData=this.userData),this.parameters!==void 0){const c=this.parameters;for(const l in c)c[l]!==void 0&&(t[l]=c[l]);return t}t.data={attributes:{}};const e=this.index;e!==null&&(t.data.index={type:e.array.constructor.name,array:Array.prototype.slice.call(e.array)});const n=this.attributes;for(const c in n){const l=n[c];t.data.attributes[c]=l.toJSON(t.data)}const i={};let s=!1;for(const c in this.morphAttributes){const l=this.morphAttributes[c],h=[];for(let u=0,d=l.length;u<d;u++){const f=l[u];h.push(f.toJSON(t.data))}h.length>0&&(i[c]=h,s=!0)}s&&(t.data.morphAttributes=i,t.data.morphTargetsRelative=this.morphTargetsRelative);const a=this.groups;a.length>0&&(t.data.groups=JSON.parse(JSON.stringify(a)));const o=this.boundingSphere;return o!==null&&(t.data.boundingSphere={center:o.center.toArray(),radius:o.radius}),t}clone(){return new this.constructor().copy(this)}copy(t){this.index=null,this.attributes={},this.morphAttributes={},this.groups=[],this.boundingBox=null,this.boundingSphere=null;const e={};this.name=t.name;const n=t.index;n!==null&&this.setIndex(n.clone(e));const i=t.attributes;for(const l in i){const h=i[l];this.setAttribute(l,h.clone(e))}const s=t.morphAttributes;for(const l in s){const h=[],u=s[l];for(let d=0,f=u.length;d<f;d++)h.push(u[d].clone(e));this.morphAttributes[l]=h}this.morphTargetsRelative=t.morphTargetsRelative;const a=t.groups;for(let l=0,h=a.length;l<h;l++){const u=a[l];this.addGroup(u.start,u.count,u.materialIndex)}const o=t.boundingBox;o!==null&&(this.boundingBox=o.clone());const c=t.boundingSphere;return c!==null&&(this.boundingSphere=c.clone()),this.drawRange.start=t.drawRange.start,this.drawRange.count=t.drawRange.count,this.userData=t.userData,this}dispose(){this.dispatchEvent({type:"dispose"})}}const Su=new re,us=new Fh,Za=new as,Mu=new R,ks=new R,Hs=new R,Ws=new R,Wc=new R,ja=new R,$a=new Mt,Ka=new Mt,Ja=new Mt,yu=new R,Eu=new R,wu=new R,Qa=new R,to=new R;class jt extends nn{constructor(t=new me,e=new Fe){super(),this.isMesh=!0,this.type="Mesh",this.geometry=t,this.material=e,this.updateMorphTargets()}copy(t,e){return super.copy(t,e),t.morphTargetInfluences!==void 0&&(this.morphTargetInfluences=t.morphTargetInfluences.slice()),t.morphTargetDictionary!==void 0&&(this.morphTargetDictionary=Object.assign({},t.morphTargetDictionary)),this.material=Array.isArray(t.material)?t.material.slice():t.material,this.geometry=t.geometry,this}updateMorphTargets(){const e=this.geometry.morphAttributes,n=Object.keys(e);if(n.length>0){const i=e[n[0]];if(i!==void 0){this.morphTargetInfluences=[],this.morphTargetDictionary={};for(let s=0,a=i.length;s<a;s++){const o=i[s].name||String(s);this.morphTargetInfluences.push(0),this.morphTargetDictionary[o]=s}}}}getVertexPosition(t,e){const n=this.geometry,i=n.attributes.position,s=n.morphAttributes.position,a=n.morphTargetsRelative;e.fromBufferAttribute(i,t);const o=this.morphTargetInfluences;if(s&&o){ja.set(0,0,0);for(let c=0,l=s.length;c<l;c++){const h=o[c],u=s[c];h!==0&&(Wc.fromBufferAttribute(u,t),a?ja.addScaledVector(Wc,h):ja.addScaledVector(Wc.sub(e),h))}e.add(ja)}return e}raycast(t,e){const n=this.geometry,i=this.material,s=this.matrixWorld;i!==void 0&&(n.boundingSphere===null&&n.computeBoundingSphere(),Za.copy(n.boundingSphere),Za.applyMatrix4(s),us.copy(t.ray).recast(t.near),!(Za.containsPoint(us.origin)===!1&&(us.intersectSphere(Za,Mu)===null||us.origin.distanceToSquared(Mu)>(t.far-t.near)**2))&&(Su.copy(s).invert(),us.copy(t.ray).applyMatrix4(Su),!(n.boundingBox!==null&&us.intersectsBox(n.boundingBox)===!1)&&this._computeIntersections(t,e,us)))}_computeIntersections(t,e,n){let i;const s=this.geometry,a=this.material,o=s.index,c=s.attributes.position,l=s.attributes.uv,h=s.attributes.uv1,u=s.attributes.normal,d=s.groups,f=s.drawRange;if(o!==null)if(Array.isArray(a))for(let g=0,x=d.length;g<x;g++){const m=d[g],p=a[m.materialIndex],_=Math.max(m.start,f.start),v=Math.min(o.count,Math.min(m.start+m.count,f.start+f.count));for(let S=_,M=v;S<M;S+=3){const y=o.getX(S),w=o.getX(S+1),A=o.getX(S+2);i=eo(this,p,t,n,l,h,u,y,w,A),i&&(i.faceIndex=Math.floor(S/3),i.face.materialIndex=m.materialIndex,e.push(i))}}else{const g=Math.max(0,f.start),x=Math.min(o.count,f.start+f.count);for(let m=g,p=x;m<p;m+=3){const _=o.getX(m),v=o.getX(m+1),S=o.getX(m+2);i=eo(this,a,t,n,l,h,u,_,v,S),i&&(i.faceIndex=Math.floor(m/3),e.push(i))}}else if(c!==void 0)if(Array.isArray(a))for(let g=0,x=d.length;g<x;g++){const m=d[g],p=a[m.materialIndex],_=Math.max(m.start,f.start),v=Math.min(c.count,Math.min(m.start+m.count,f.start+f.count));for(let S=_,M=v;S<M;S+=3){const y=S,w=S+1,A=S+2;i=eo(this,p,t,n,l,h,u,y,w,A),i&&(i.faceIndex=Math.floor(S/3),i.face.materialIndex=m.materialIndex,e.push(i))}}else{const g=Math.max(0,f.start),x=Math.min(c.count,f.start+f.count);for(let m=g,p=x;m<p;m+=3){const _=m,v=m+1,S=m+2;i=eo(this,a,t,n,l,h,u,_,v,S),i&&(i.faceIndex=Math.floor(m/3),e.push(i))}}}}function Vm(r,t,e,n,i,s,a,o){let c;if(t.side===1?c=n.intersectTriangle(a,s,i,!0,o):c=n.intersectTriangle(i,s,a,t.side===0,o),c===null)return null;to.copy(o),to.applyMatrix4(r.matrixWorld);const l=e.ray.origin.distanceTo(to);return l<e.near||l>e.far?null:{distance:l,point:to.clone(),object:r}}function eo(r,t,e,n,i,s,a,o,c,l){r.getVertexPosition(o,ks),r.getVertexPosition(c,Hs),r.getVertexPosition(l,Ws);const h=Vm(r,t,e,n,ks,Hs,Ws,Qa);if(h){i&&($a.fromBufferAttribute(i,o),Ka.fromBufferAttribute(i,c),Ja.fromBufferAttribute(i,l),h.uv=on.getInterpolation(Qa,ks,Hs,Ws,$a,Ka,Ja,new Mt)),s&&($a.fromBufferAttribute(s,o),Ka.fromBufferAttribute(s,c),Ja.fromBufferAttribute(s,l),h.uv1=on.getInterpolation(Qa,ks,Hs,Ws,$a,Ka,Ja,new Mt),h.uv2=h.uv1),a&&(yu.fromBufferAttribute(a,o),Eu.fromBufferAttribute(a,c),wu.fromBufferAttribute(a,l),h.normal=on.getInterpolation(Qa,ks,Hs,Ws,yu,Eu,wu,new R),h.normal.dot(n.direction)>0&&h.normal.multiplyScalar(-1));const u={a:o,b:c,c:l,normal:new R,materialIndex:0};on.getNormal(ks,Hs,Ws,u.normal),h.face=u}return h}class Dn extends me{constructor(t=1,e=1,n=1,i=1,s=1,a=1){super(),this.type="BoxGeometry",this.parameters={width:t,height:e,depth:n,widthSegments:i,heightSegments:s,depthSegments:a};const o=this;i=Math.floor(i),s=Math.floor(s),a=Math.floor(a);const c=[],l=[],h=[],u=[];let d=0,f=0;g("z","y","x",-1,-1,n,e,t,a,s,0),g("z","y","x",1,-1,n,e,-t,a,s,1),g("x","z","y",1,1,t,n,e,i,a,2),g("x","z","y",1,-1,t,n,-e,i,a,3),g("x","y","z",1,-1,t,e,n,i,s,4),g("x","y","z",-1,-1,t,e,-n,i,s,5),this.setIndex(c),this.setAttribute("position",new ie(l,3)),this.setAttribute("normal",new ie(h,3)),this.setAttribute("uv",new ie(u,2));function g(x,m,p,_,v,S,M,y,w,A,E){const T=S/w,L=M/A,C=S/2,F=M/2,D=y/2,N=w+1,B=A+1;let G=0,z=0;const H=new R;for(let j=0;j<B;j++){const $=j*L-F;for(let Q=0;Q<N;Q++){const V=Q*T-C;H[x]=V*_,H[m]=$*v,H[p]=D,l.push(H.x,H.y,H.z),H[x]=0,H[m]=0,H[p]=y>0?1:-1,h.push(H.x,H.y,H.z),u.push(Q/w),u.push(1-j/A),G+=1}}for(let j=0;j<A;j++)for(let $=0;$<w;$++){const Q=d+$+N*j,V=d+$+N*(j+1),K=d+($+1)+N*(j+1),nt=d+($+1)+N*j;c.push(Q,V,nt),c.push(V,K,nt),z+=6}o.addGroup(f,z,E),f+=z,d+=G}}copy(t){return super.copy(t),this.parameters=Object.assign({},t.parameters),this}static fromJSON(t){return new Dn(t.width,t.height,t.depth,t.widthSegments,t.heightSegments,t.depthSegments)}}function Sr(r){const t={};for(const e in r){t[e]={};for(const n in r[e]){const i=r[e][n];i&&(i.isColor||i.isMatrix3||i.isMatrix4||i.isVector2||i.isVector3||i.isVector4||i.isTexture||i.isQuaternion)?i.isRenderTargetTexture?(console.warn("UniformsUtils: Textures of render targets cannot be cloned via cloneUniforms() or mergeUniforms()."),t[e][n]=null):t[e][n]=i.clone():Array.isArray(i)?t[e][n]=i.slice():t[e][n]=i}}return t}function gn(r){const t={};for(let e=0;e<r.length;e++){const n=Sr(r[e]);for(const i in n)t[i]=n[i]}return t}function km(r){const t=[];for(let e=0;e<r.length;e++)t.push(r[e].clone());return t}function Wf(r){return r.getRenderTarget()===null?r.outputColorSpace:pe.workingColorSpace}const Hm={clone:Sr,merge:gn};var Wm=`void main() {
	gl_Position = projectionMatrix * modelViewMatrix * vec4( position, 1.0 );
}`,Xm=`void main() {
	gl_FragColor = vec4( 1.0, 0.0, 0.0, 1.0 );
}`;class Bn extends _i{constructor(t){super(),this.isShaderMaterial=!0,this.type="ShaderMaterial",this.defines={},this.uniforms={},this.uniformsGroups=[],this.vertexShader=Wm,this.fragmentShader=Xm,this.linewidth=1,this.wireframe=!1,this.wireframeLinewidth=1,this.fog=!1,this.lights=!1,this.clipping=!1,this.forceSinglePass=!0,this.extensions={derivatives:!1,fragDepth:!1,drawBuffers:!1,shaderTextureLOD:!1,clipCullDistance:!1},this.defaultAttributeValues={color:[1,1,1],uv:[0,0],uv1:[0,0]},this.index0AttributeName=void 0,this.uniformsNeedUpdate=!1,this.glslVersion=null,t!==void 0&&this.setValues(t)}copy(t){return super.copy(t),this.fragmentShader=t.fragmentShader,this.vertexShader=t.vertexShader,this.uniforms=Sr(t.uniforms),this.uniformsGroups=km(t.uniformsGroups),this.defines=Object.assign({},t.defines),this.wireframe=t.wireframe,this.wireframeLinewidth=t.wireframeLinewidth,this.fog=t.fog,this.lights=t.lights,this.clipping=t.clipping,this.extensions=Object.assign({},t.extensions),this.glslVersion=t.glslVersion,this}toJSON(t){const e=super.toJSON(t);e.glslVersion=this.glslVersion,e.uniforms={};for(const i in this.uniforms){const a=this.uniforms[i].value;a&&a.isTexture?e.uniforms[i]={type:"t",value:a.toJSON(t).uuid}:a&&a.isColor?e.uniforms[i]={type:"c",value:a.getHex()}:a&&a.isVector2?e.uniforms[i]={type:"v2",value:a.toArray()}:a&&a.isVector3?e.uniforms[i]={type:"v3",value:a.toArray()}:a&&a.isVector4?e.uniforms[i]={type:"v4",value:a.toArray()}:a&&a.isMatrix3?e.uniforms[i]={type:"m3",value:a.toArray()}:a&&a.isMatrix4?e.uniforms[i]={type:"m4",value:a.toArray()}:e.uniforms[i]={value:a}}Object.keys(this.defines).length>0&&(e.defines=this.defines),e.vertexShader=this.vertexShader,e.fragmentShader=this.fragmentShader,e.lights=this.lights,e.clipping=this.clipping;const n={};for(const i in this.extensions)this.extensions[i]===!0&&(n[i]=!0);return Object.keys(n).length>0&&(e.extensions=n),e}}class Xf extends nn{constructor(){super(),this.isCamera=!0,this.type="Camera",this.matrixWorldInverse=new re,this.projectionMatrix=new re,this.projectionMatrixInverse=new re,this.coordinateSystem=2e3}copy(t,e){return super.copy(t,e),this.matrixWorldInverse.copy(t.matrixWorldInverse),this.projectionMatrix.copy(t.projectionMatrix),this.projectionMatrixInverse.copy(t.projectionMatrixInverse),this.coordinateSystem=t.coordinateSystem,this}getWorldDirection(t){return super.getWorldDirection(t).negate()}updateMatrixWorld(t){super.updateMatrixWorld(t),this.matrixWorldInverse.copy(this.matrixWorld).invert()}updateWorldMatrix(t,e){super.updateWorldMatrix(t,e),this.matrixWorldInverse.copy(this.matrixWorld).invert()}clone(){return new this.constructor().copy(this)}}class In extends Xf{constructor(t=50,e=1,n=.1,i=2e3){super(),this.isPerspectiveCamera=!0,this.type="PerspectiveCamera",this.fov=t,this.zoom=1,this.near=n,this.far=i,this.focus=10,this.aspect=e,this.view=null,this.filmGauge=35,this.filmOffset=0,this.updateProjectionMatrix()}copy(t,e){return super.copy(t,e),this.fov=t.fov,this.zoom=t.zoom,this.near=t.near,this.far=t.far,this.focus=t.focus,this.aspect=t.aspect,this.view=t.view===null?null:Object.assign({},t.view),this.filmGauge=t.filmGauge,this.filmOffset=t.filmOffset,this}setFocalLength(t){const e=.5*this.getFilmHeight()/t;this.fov=Pa*2*Math.atan(e),this.updateProjectionMatrix()}getFocalLength(){const t=Math.tan(va*.5*this.fov);return .5*this.getFilmHeight()/t}getEffectiveFOV(){return Pa*2*Math.atan(Math.tan(va*.5*this.fov)/this.zoom)}getFilmWidth(){return this.filmGauge*Math.min(this.aspect,1)}getFilmHeight(){return this.filmGauge/Math.max(this.aspect,1)}setViewOffset(t,e,n,i,s,a){this.aspect=t/e,this.view===null&&(this.view={enabled:!0,fullWidth:1,fullHeight:1,offsetX:0,offsetY:0,width:1,height:1}),this.view.enabled=!0,this.view.fullWidth=t,this.view.fullHeight=e,this.view.offsetX=n,this.view.offsetY=i,this.view.width=s,this.view.height=a,this.updateProjectionMatrix()}clearViewOffset(){this.view!==null&&(this.view.enabled=!1),this.updateProjectionMatrix()}updateProjectionMatrix(){const t=this.near;let e=t*Math.tan(va*.5*this.fov)/this.zoom,n=2*e,i=this.aspect*n,s=-.5*i;const a=this.view;if(this.view!==null&&this.view.enabled){const c=a.fullWidth,l=a.fullHeight;s+=a.offsetX*i/c,e-=a.offsetY*n/l,i*=a.width/c,n*=a.height/l}const o=this.filmOffset;o!==0&&(s+=t*o/this.getFilmWidth()),this.projectionMatrix.makePerspective(s,s+i,e,e-n,t,this.far,this.coordinateSystem),this.projectionMatrixInverse.copy(this.projectionMatrix).invert()}toJSON(t){const e=super.toJSON(t);return e.object.fov=this.fov,e.object.zoom=this.zoom,e.object.near=this.near,e.object.far=this.far,e.object.focus=this.focus,e.object.aspect=this.aspect,this.view!==null&&(e.object.view=Object.assign({},this.view)),e.object.filmGauge=this.filmGauge,e.object.filmOffset=this.filmOffset,e}}const Xs=-90,Ys=1;class Ym extends nn{constructor(t,e,n){super(),this.type="CubeCamera",this.renderTarget=n,this.coordinateSystem=null,this.activeMipmapLevel=0;const i=new In(Xs,Ys,t,e);i.layers=this.layers,this.add(i);const s=new In(Xs,Ys,t,e);s.layers=this.layers,this.add(s);const a=new In(Xs,Ys,t,e);a.layers=this.layers,this.add(a);const o=new In(Xs,Ys,t,e);o.layers=this.layers,this.add(o);const c=new In(Xs,Ys,t,e);c.layers=this.layers,this.add(c);const l=new In(Xs,Ys,t,e);l.layers=this.layers,this.add(l)}updateCoordinateSystem(){const t=this.coordinateSystem,e=this.children.concat(),[n,i,s,a,o,c]=e;for(const l of e)this.remove(l);if(t===2e3)n.up.set(0,1,0),n.lookAt(1,0,0),i.up.set(0,1,0),i.lookAt(-1,0,0),s.up.set(0,0,-1),s.lookAt(0,1,0),a.up.set(0,0,1),a.lookAt(0,-1,0),o.up.set(0,1,0),o.lookAt(0,0,1),c.up.set(0,1,0),c.lookAt(0,0,-1);else if(t===2001)n.up.set(0,-1,0),n.lookAt(-1,0,0),i.up.set(0,-1,0),i.lookAt(1,0,0),s.up.set(0,0,1),s.lookAt(0,1,0),a.up.set(0,0,-1),a.lookAt(0,-1,0),o.up.set(0,-1,0),o.lookAt(0,0,1),c.up.set(0,-1,0),c.lookAt(0,0,-1);else throw new Error("THREE.CubeCamera.updateCoordinateSystem(): Invalid coordinate system: "+t);for(const l of e)this.add(l),l.updateMatrixWorld()}update(t,e){this.parent===null&&this.updateMatrixWorld();const{renderTarget:n,activeMipmapLevel:i}=this;this.coordinateSystem!==t.coordinateSystem&&(this.coordinateSystem=t.coordinateSystem,this.updateCoordinateSystem());const[s,a,o,c,l,h]=this.children,u=t.getRenderTarget(),d=t.getActiveCubeFace(),f=t.getActiveMipmapLevel(),g=t.xr.enabled;t.xr.enabled=!1;const x=n.texture.generateMipmaps;n.texture.generateMipmaps=!1,t.setRenderTarget(n,0,i),t.render(e,s),t.setRenderTarget(n,1,i),t.render(e,a),t.setRenderTarget(n,2,i),t.render(e,o),t.setRenderTarget(n,3,i),t.render(e,c),t.setRenderTarget(n,4,i),t.render(e,l),n.texture.generateMipmaps=x,t.setRenderTarget(n,5,i),t.render(e,h),t.setRenderTarget(u,d,f),t.xr.enabled=g,n.texture.needsPMREMUpdate=!0}}class Yf extends Fn{constructor(t,e,n,i,s,a,o,c,l,h){t=t!==void 0?t:[],e=e!==void 0?e:301,super(t,e,n,i,s,a,o,c,l,h),this.isCubeTexture=!0,this.flipY=!1}get images(){return this.image}set images(t){this.image=t}}class qm extends sn{constructor(t=1,e={}){super(t,t,e),this.isWebGLCubeRenderTarget=!0;const n={width:t,height:t,depth:1},i=[n,n,n,n,n,n];e.encoding!==void 0&&(Ma("THREE.WebGLCubeRenderTarget: option.encoding has been replaced by option.colorSpace."),e.colorSpace=e.encoding===3001?an:""),this.texture=new Yf(i,e.mapping,e.wrapS,e.wrapT,e.magFilter,e.minFilter,e.format,e.type,e.anisotropy,e.colorSpace),this.texture.isRenderTargetTexture=!0,this.texture.generateMipmaps=e.generateMipmaps!==void 0?e.generateMipmaps:!1,this.texture.minFilter=e.minFilter!==void 0?e.minFilter:1006}fromEquirectangularTexture(t,e){this.texture.type=e.type,this.texture.colorSpace=e.colorSpace,this.texture.generateMipmaps=e.generateMipmaps,this.texture.minFilter=e.minFilter,this.texture.magFilter=e.magFilter;const n={uniforms:{tEquirect:{value:null}},vertexShader:`

				varying vec3 vWorldDirection;

				vec3 transformDirection( in vec3 dir, in mat4 matrix ) {

					return normalize( ( matrix * vec4( dir, 0.0 ) ).xyz );

				}

				void main() {

					vWorldDirection = transformDirection( position, modelMatrix );

					#include <begin_vertex>
					#include <project_vertex>

				}
			`,fragmentShader:`

				uniform sampler2D tEquirect;

				varying vec3 vWorldDirection;

				#include <common>

				void main() {

					vec3 direction = normalize( vWorldDirection );

					vec2 sampleUV = equirectUv( direction );

					gl_FragColor = texture2D( tEquirect, sampleUV );

				}
			`},i=new Dn(5,5,5),s=new Bn({name:"CubemapFromEquirect",uniforms:Sr(n.uniforms),vertexShader:n.vertexShader,fragmentShader:n.fragmentShader,side:1,blending:0});s.uniforms.tEquirect.value=e;const a=new jt(i,s),o=e.minFilter;return e.minFilter===1008&&(e.minFilter=1006),new Ym(1,10,this).update(t,a),e.minFilter=o,a.geometry.dispose(),a.material.dispose(),this}clear(t,e,n,i){const s=t.getRenderTarget();for(let a=0;a<6;a++)t.setRenderTarget(this,a),t.clear(e,n,i);t.setRenderTarget(s)}}const Xc=new R,Zm=new R,jm=new ne;class Ii{constructor(t=new R(1,0,0),e=0){this.isPlane=!0,this.normal=t,this.constant=e}set(t,e){return this.normal.copy(t),this.constant=e,this}setComponents(t,e,n,i){return this.normal.set(t,e,n),this.constant=i,this}setFromNormalAndCoplanarPoint(t,e){return this.normal.copy(t),this.constant=-e.dot(this.normal),this}setFromCoplanarPoints(t,e,n){const i=Xc.subVectors(n,e).cross(Zm.subVectors(t,e)).normalize();return this.setFromNormalAndCoplanarPoint(i,t),this}copy(t){return this.normal.copy(t.normal),this.constant=t.constant,this}normalize(){const t=1/this.normal.length();return this.normal.multiplyScalar(t),this.constant*=t,this}negate(){return this.constant*=-1,this.normal.negate(),this}distanceToPoint(t){return this.normal.dot(t)+this.constant}distanceToSphere(t){return this.distanceToPoint(t.center)-t.radius}projectPoint(t,e){return e.copy(t).addScaledVector(this.normal,-this.distanceToPoint(t))}intersectLine(t,e){const n=t.delta(Xc),i=this.normal.dot(n);if(i===0)return this.distanceToPoint(t.start)===0?e.copy(t.start):null;const s=-(t.start.dot(this.normal)+this.constant)/i;return s<0||s>1?null:e.copy(t.start).addScaledVector(n,s)}intersectsLine(t){const e=this.distanceToPoint(t.start),n=this.distanceToPoint(t.end);return e<0&&n>0||n<0&&e>0}intersectsBox(t){return t.intersectsPlane(this)}intersectsSphere(t){return t.intersectsPlane(this)}coplanarPoint(t){return t.copy(this.normal).multiplyScalar(-this.constant)}applyMatrix4(t,e){const n=e||jm.getNormalMatrix(t),i=this.coplanarPoint(Xc).applyMatrix4(t),s=this.normal.applyMatrix3(n).normalize();return this.constant=-i.dot(s),this}translate(t){return this.constant-=t.dot(this.normal),this}equals(t){return t.normal.equals(this.normal)&&t.constant===this.constant}clone(){return new this.constructor().copy(this)}}const ds=new as,no=new R;class Bh{constructor(t=new Ii,e=new Ii,n=new Ii,i=new Ii,s=new Ii,a=new Ii){this.planes=[t,e,n,i,s,a]}set(t,e,n,i,s,a){const o=this.planes;return o[0].copy(t),o[1].copy(e),o[2].copy(n),o[3].copy(i),o[4].copy(s),o[5].copy(a),this}copy(t){const e=this.planes;for(let n=0;n<6;n++)e[n].copy(t.planes[n]);return this}setFromProjectionMatrix(t,e=2e3){const n=this.planes,i=t.elements,s=i[0],a=i[1],o=i[2],c=i[3],l=i[4],h=i[5],u=i[6],d=i[7],f=i[8],g=i[9],x=i[10],m=i[11],p=i[12],_=i[13],v=i[14],S=i[15];if(n[0].setComponents(c-s,d-l,m-f,S-p).normalize(),n[1].setComponents(c+s,d+l,m+f,S+p).normalize(),n[2].setComponents(c+a,d+h,m+g,S+_).normalize(),n[3].setComponents(c-a,d-h,m-g,S-_).normalize(),n[4].setComponents(c-o,d-u,m-x,S-v).normalize(),e===2e3)n[5].setComponents(c+o,d+u,m+x,S+v).normalize();else if(e===2001)n[5].setComponents(o,u,x,v).normalize();else throw new Error("THREE.Frustum.setFromProjectionMatrix(): Invalid coordinate system: "+e);return this}intersectsObject(t){if(t.boundingSphere!==void 0)t.boundingSphere===null&&t.computeBoundingSphere(),ds.copy(t.boundingSphere).applyMatrix4(t.matrixWorld);else{const e=t.geometry;e.boundingSphere===null&&e.computeBoundingSphere(),ds.copy(e.boundingSphere).applyMatrix4(t.matrixWorld)}return this.intersectsSphere(ds)}intersectsSprite(t){return ds.center.set(0,0,0),ds.radius=.7071067811865476,ds.applyMatrix4(t.matrixWorld),this.intersectsSphere(ds)}intersectsSphere(t){const e=this.planes,n=t.center,i=-t.radius;for(let s=0;s<6;s++)if(e[s].distanceToPoint(n)<i)return!1;return!0}intersectsBox(t){const e=this.planes;for(let n=0;n<6;n++){const i=e[n];if(no.x=i.normal.x>0?t.max.x:t.min.x,no.y=i.normal.y>0?t.max.y:t.min.y,no.z=i.normal.z>0?t.max.z:t.min.z,i.distanceToPoint(no)<0)return!1}return!0}containsPoint(t){const e=this.planes;for(let n=0;n<6;n++)if(e[n].distanceToPoint(t)<0)return!1;return!0}clone(){return new this.constructor().copy(this)}}function qf(){let r=null,t=!1,e=null,n=null;function i(s,a){e(s,a),n=r.requestAnimationFrame(i)}return{start:function(){t!==!0&&e!==null&&(n=r.requestAnimationFrame(i),t=!0)},stop:function(){r.cancelAnimationFrame(n),t=!1},setAnimationLoop:function(s){e=s},setContext:function(s){r=s}}}function $m(r,t){const e=t.isWebGL2,n=new WeakMap;function i(l,h){const u=l.array,d=l.usage,f=u.byteLength,g=r.createBuffer();r.bindBuffer(h,g),r.bufferData(h,u,d),l.onUploadCallback();let x;if(u instanceof Float32Array)x=r.FLOAT;else if(u instanceof Uint16Array)if(l.isFloat16BufferAttribute)if(e)x=r.HALF_FLOAT;else throw new Error("THREE.WebGLAttributes: Usage of Float16BufferAttribute requires WebGL2.");else x=r.UNSIGNED_SHORT;else if(u instanceof Int16Array)x=r.SHORT;else if(u instanceof Uint32Array)x=r.UNSIGNED_INT;else if(u instanceof Int32Array)x=r.INT;else if(u instanceof Int8Array)x=r.BYTE;else if(u instanceof Uint8Array)x=r.UNSIGNED_BYTE;else if(u instanceof Uint8ClampedArray)x=r.UNSIGNED_BYTE;else throw new Error("THREE.WebGLAttributes: Unsupported buffer data format: "+u);return{buffer:g,type:x,bytesPerElement:u.BYTES_PER_ELEMENT,version:l.version,size:f}}function s(l,h,u){const d=h.array,f=h._updateRange,g=h.updateRanges;if(r.bindBuffer(u,l),f.count===-1&&g.length===0&&r.bufferSubData(u,0,d),g.length!==0){for(let x=0,m=g.length;x<m;x++){const p=g[x];e?r.bufferSubData(u,p.start*d.BYTES_PER_ELEMENT,d,p.start,p.count):r.bufferSubData(u,p.start*d.BYTES_PER_ELEMENT,d.subarray(p.start,p.start+p.count))}h.clearUpdateRanges()}f.count!==-1&&(e?r.bufferSubData(u,f.offset*d.BYTES_PER_ELEMENT,d,f.offset,f.count):r.bufferSubData(u,f.offset*d.BYTES_PER_ELEMENT,d.subarray(f.offset,f.offset+f.count)),f.count=-1),h.onUploadCallback()}function a(l){return l.isInterleavedBufferAttribute&&(l=l.data),n.get(l)}function o(l){l.isInterleavedBufferAttribute&&(l=l.data);const h=n.get(l);h&&(r.deleteBuffer(h.buffer),n.delete(l))}function c(l,h){if(l.isGLBufferAttribute){const d=n.get(l);(!d||d.version<l.version)&&n.set(l,{buffer:l.buffer,type:l.type,bytesPerElement:l.elementSize,version:l.version});return}l.isInterleavedBufferAttribute&&(l=l.data);const u=n.get(l);if(u===void 0)n.set(l,i(l,h));else if(u.version<l.version){if(u.size!==l.array.byteLength)throw new Error("THREE.WebGLAttributes: The size of the buffer attribute's array buffer does not match the original size. Resizing buffer attributes is not supported.");s(u.buffer,l,h),u.version=l.version}}return{get:a,remove:o,update:c}}class pc extends me{constructor(t=1,e=1,n=1,i=1){super(),this.type="PlaneGeometry",this.parameters={width:t,height:e,widthSegments:n,heightSegments:i};const s=t/2,a=e/2,o=Math.floor(n),c=Math.floor(i),l=o+1,h=c+1,u=t/o,d=e/c,f=[],g=[],x=[],m=[];for(let p=0;p<h;p++){const _=p*d-a;for(let v=0;v<l;v++){const S=v*u-s;g.push(S,-_,0),x.push(0,0,1),m.push(v/o),m.push(1-p/c)}}for(let p=0;p<c;p++)for(let _=0;_<o;_++){const v=_+l*p,S=_+l*(p+1),M=_+1+l*(p+1),y=_+1+l*p;f.push(v,S,y),f.push(S,M,y)}this.setIndex(f),this.setAttribute("position",new ie(g,3)),this.setAttribute("normal",new ie(x,3)),this.setAttribute("uv",new ie(m,2))}copy(t){return super.copy(t),this.parameters=Object.assign({},t.parameters),this}static fromJSON(t){return new pc(t.width,t.height,t.widthSegments,t.heightSegments)}}var Km=`#ifdef USE_ALPHAHASH
	if ( diffuseColor.a < getAlphaHashThreshold( vPosition ) ) discard;
#endif`,Jm=`#ifdef USE_ALPHAHASH
	const float ALPHA_HASH_SCALE = 0.05;
	float hash2D( vec2 value ) {
		return fract( 1.0e4 * sin( 17.0 * value.x + 0.1 * value.y ) * ( 0.1 + abs( sin( 13.0 * value.y + value.x ) ) ) );
	}
	float hash3D( vec3 value ) {
		return hash2D( vec2( hash2D( value.xy ), value.z ) );
	}
	float getAlphaHashThreshold( vec3 position ) {
		float maxDeriv = max(
			length( dFdx( position.xyz ) ),
			length( dFdy( position.xyz ) )
		);
		float pixScale = 1.0 / ( ALPHA_HASH_SCALE * maxDeriv );
		vec2 pixScales = vec2(
			exp2( floor( log2( pixScale ) ) ),
			exp2( ceil( log2( pixScale ) ) )
		);
		vec2 alpha = vec2(
			hash3D( floor( pixScales.x * position.xyz ) ),
			hash3D( floor( pixScales.y * position.xyz ) )
		);
		float lerpFactor = fract( log2( pixScale ) );
		float x = ( 1.0 - lerpFactor ) * alpha.x + lerpFactor * alpha.y;
		float a = min( lerpFactor, 1.0 - lerpFactor );
		vec3 cases = vec3(
			x * x / ( 2.0 * a * ( 1.0 - a ) ),
			( x - 0.5 * a ) / ( 1.0 - a ),
			1.0 - ( ( 1.0 - x ) * ( 1.0 - x ) / ( 2.0 * a * ( 1.0 - a ) ) )
		);
		float threshold = ( x < ( 1.0 - a ) )
			? ( ( x < a ) ? cases.x : cases.y )
			: cases.z;
		return clamp( threshold , 1.0e-6, 1.0 );
	}
#endif`,Qm=`#ifdef USE_ALPHAMAP
	diffuseColor.a *= texture2D( alphaMap, vAlphaMapUv ).g;
#endif`,tg=`#ifdef USE_ALPHAMAP
	uniform sampler2D alphaMap;
#endif`,eg=`#ifdef USE_ALPHATEST
	if ( diffuseColor.a < alphaTest ) discard;
#endif`,ng=`#ifdef USE_ALPHATEST
	uniform float alphaTest;
#endif`,ig=`#ifdef USE_AOMAP
	float ambientOcclusion = ( texture2D( aoMap, vAoMapUv ).r - 1.0 ) * aoMapIntensity + 1.0;
	reflectedLight.indirectDiffuse *= ambientOcclusion;
	#if defined( USE_CLEARCOAT ) 
		clearcoatSpecularIndirect *= ambientOcclusion;
	#endif
	#if defined( USE_SHEEN ) 
		sheenSpecularIndirect *= ambientOcclusion;
	#endif
	#if defined( USE_ENVMAP ) && defined( STANDARD )
		float dotNV = saturate( dot( geometryNormal, geometryViewDir ) );
		reflectedLight.indirectSpecular *= computeSpecularOcclusion( dotNV, ambientOcclusion, material.roughness );
	#endif
#endif`,sg=`#ifdef USE_AOMAP
	uniform sampler2D aoMap;
	uniform float aoMapIntensity;
#endif`,rg=`#ifdef USE_BATCHING
	attribute float batchId;
	uniform highp sampler2D batchingTexture;
	mat4 getBatchingMatrix( const in float i ) {
		int size = textureSize( batchingTexture, 0 ).x;
		int j = int( i ) * 4;
		int x = j % size;
		int y = j / size;
		vec4 v1 = texelFetch( batchingTexture, ivec2( x, y ), 0 );
		vec4 v2 = texelFetch( batchingTexture, ivec2( x + 1, y ), 0 );
		vec4 v3 = texelFetch( batchingTexture, ivec2( x + 2, y ), 0 );
		vec4 v4 = texelFetch( batchingTexture, ivec2( x + 3, y ), 0 );
		return mat4( v1, v2, v3, v4 );
	}
#endif`,ag=`#ifdef USE_BATCHING
	mat4 batchingMatrix = getBatchingMatrix( batchId );
#endif`,og=`vec3 transformed = vec3( position );
#ifdef USE_ALPHAHASH
	vPosition = vec3( position );
#endif`,cg=`vec3 objectNormal = vec3( normal );
#ifdef USE_TANGENT
	vec3 objectTangent = vec3( tangent.xyz );
#endif`,lg=`float G_BlinnPhong_Implicit( ) {
	return 0.25;
}
float D_BlinnPhong( const in float shininess, const in float dotNH ) {
	return RECIPROCAL_PI * ( shininess * 0.5 + 1.0 ) * pow( dotNH, shininess );
}
vec3 BRDF_BlinnPhong( const in vec3 lightDir, const in vec3 viewDir, const in vec3 normal, const in vec3 specularColor, const in float shininess ) {
	vec3 halfDir = normalize( lightDir + viewDir );
	float dotNH = saturate( dot( normal, halfDir ) );
	float dotVH = saturate( dot( viewDir, halfDir ) );
	vec3 F = F_Schlick( specularColor, 1.0, dotVH );
	float G = G_BlinnPhong_Implicit( );
	float D = D_BlinnPhong( shininess, dotNH );
	return F * ( G * D );
} // validated`,hg=`#ifdef USE_IRIDESCENCE
	const mat3 XYZ_TO_REC709 = mat3(
		 3.2404542, -0.9692660,  0.0556434,
		-1.5371385,  1.8760108, -0.2040259,
		-0.4985314,  0.0415560,  1.0572252
	);
	vec3 Fresnel0ToIor( vec3 fresnel0 ) {
		vec3 sqrtF0 = sqrt( fresnel0 );
		return ( vec3( 1.0 ) + sqrtF0 ) / ( vec3( 1.0 ) - sqrtF0 );
	}
	vec3 IorToFresnel0( vec3 transmittedIor, float incidentIor ) {
		return pow2( ( transmittedIor - vec3( incidentIor ) ) / ( transmittedIor + vec3( incidentIor ) ) );
	}
	float IorToFresnel0( float transmittedIor, float incidentIor ) {
		return pow2( ( transmittedIor - incidentIor ) / ( transmittedIor + incidentIor ));
	}
	vec3 evalSensitivity( float OPD, vec3 shift ) {
		float phase = 2.0 * PI * OPD * 1.0e-9;
		vec3 val = vec3( 5.4856e-13, 4.4201e-13, 5.2481e-13 );
		vec3 pos = vec3( 1.6810e+06, 1.7953e+06, 2.2084e+06 );
		vec3 var = vec3( 4.3278e+09, 9.3046e+09, 6.6121e+09 );
		vec3 xyz = val * sqrt( 2.0 * PI * var ) * cos( pos * phase + shift ) * exp( - pow2( phase ) * var );
		xyz.x += 9.7470e-14 * sqrt( 2.0 * PI * 4.5282e+09 ) * cos( 2.2399e+06 * phase + shift[ 0 ] ) * exp( - 4.5282e+09 * pow2( phase ) );
		xyz /= 1.0685e-7;
		vec3 rgb = XYZ_TO_REC709 * xyz;
		return rgb;
	}
	vec3 evalIridescence( float outsideIOR, float eta2, float cosTheta1, float thinFilmThickness, vec3 baseF0 ) {
		vec3 I;
		float iridescenceIOR = mix( outsideIOR, eta2, smoothstep( 0.0, 0.03, thinFilmThickness ) );
		float sinTheta2Sq = pow2( outsideIOR / iridescenceIOR ) * ( 1.0 - pow2( cosTheta1 ) );
		float cosTheta2Sq = 1.0 - sinTheta2Sq;
		if ( cosTheta2Sq < 0.0 ) {
			return vec3( 1.0 );
		}
		float cosTheta2 = sqrt( cosTheta2Sq );
		float R0 = IorToFresnel0( iridescenceIOR, outsideIOR );
		float R12 = F_Schlick( R0, 1.0, cosTheta1 );
		float T121 = 1.0 - R12;
		float phi12 = 0.0;
		if ( iridescenceIOR < outsideIOR ) phi12 = PI;
		float phi21 = PI - phi12;
		vec3 baseIOR = Fresnel0ToIor( clamp( baseF0, 0.0, 0.9999 ) );		vec3 R1 = IorToFresnel0( baseIOR, iridescenceIOR );
		vec3 R23 = F_Schlick( R1, 1.0, cosTheta2 );
		vec3 phi23 = vec3( 0.0 );
		if ( baseIOR[ 0 ] < iridescenceIOR ) phi23[ 0 ] = PI;
		if ( baseIOR[ 1 ] < iridescenceIOR ) phi23[ 1 ] = PI;
		if ( baseIOR[ 2 ] < iridescenceIOR ) phi23[ 2 ] = PI;
		float OPD = 2.0 * iridescenceIOR * thinFilmThickness * cosTheta2;
		vec3 phi = vec3( phi21 ) + phi23;
		vec3 R123 = clamp( R12 * R23, 1e-5, 0.9999 );
		vec3 r123 = sqrt( R123 );
		vec3 Rs = pow2( T121 ) * R23 / ( vec3( 1.0 ) - R123 );
		vec3 C0 = R12 + Rs;
		I = C0;
		vec3 Cm = Rs - T121;
		for ( int m = 1; m <= 2; ++ m ) {
			Cm *= r123;
			vec3 Sm = 2.0 * evalSensitivity( float( m ) * OPD, float( m ) * phi );
			I += Cm * Sm;
		}
		return max( I, vec3( 0.0 ) );
	}
#endif`,ug=`#ifdef USE_BUMPMAP
	uniform sampler2D bumpMap;
	uniform float bumpScale;
	vec2 dHdxy_fwd() {
		vec2 dSTdx = dFdx( vBumpMapUv );
		vec2 dSTdy = dFdy( vBumpMapUv );
		float Hll = bumpScale * texture2D( bumpMap, vBumpMapUv ).x;
		float dBx = bumpScale * texture2D( bumpMap, vBumpMapUv + dSTdx ).x - Hll;
		float dBy = bumpScale * texture2D( bumpMap, vBumpMapUv + dSTdy ).x - Hll;
		return vec2( dBx, dBy );
	}
	vec3 perturbNormalArb( vec3 surf_pos, vec3 surf_norm, vec2 dHdxy, float faceDirection ) {
		vec3 vSigmaX = normalize( dFdx( surf_pos.xyz ) );
		vec3 vSigmaY = normalize( dFdy( surf_pos.xyz ) );
		vec3 vN = surf_norm;
		vec3 R1 = cross( vSigmaY, vN );
		vec3 R2 = cross( vN, vSigmaX );
		float fDet = dot( vSigmaX, R1 ) * faceDirection;
		vec3 vGrad = sign( fDet ) * ( dHdxy.x * R1 + dHdxy.y * R2 );
		return normalize( abs( fDet ) * surf_norm - vGrad );
	}
#endif`,dg=`#if NUM_CLIPPING_PLANES > 0
	vec4 plane;
	#pragma unroll_loop_start
	for ( int i = 0; i < UNION_CLIPPING_PLANES; i ++ ) {
		plane = clippingPlanes[ i ];
		if ( dot( vClipPosition, plane.xyz ) > plane.w ) discard;
	}
	#pragma unroll_loop_end
	#if UNION_CLIPPING_PLANES < NUM_CLIPPING_PLANES
		bool clipped = true;
		#pragma unroll_loop_start
		for ( int i = UNION_CLIPPING_PLANES; i < NUM_CLIPPING_PLANES; i ++ ) {
			plane = clippingPlanes[ i ];
			clipped = ( dot( vClipPosition, plane.xyz ) > plane.w ) && clipped;
		}
		#pragma unroll_loop_end
		if ( clipped ) discard;
	#endif
#endif`,fg=`#if NUM_CLIPPING_PLANES > 0
	varying vec3 vClipPosition;
	uniform vec4 clippingPlanes[ NUM_CLIPPING_PLANES ];
#endif`,pg=`#if NUM_CLIPPING_PLANES > 0
	varying vec3 vClipPosition;
#endif`,mg=`#if NUM_CLIPPING_PLANES > 0
	vClipPosition = - mvPosition.xyz;
#endif`,gg=`#if defined( USE_COLOR_ALPHA )
	diffuseColor *= vColor;
#elif defined( USE_COLOR )
	diffuseColor.rgb *= vColor;
#endif`,xg=`#if defined( USE_COLOR_ALPHA )
	varying vec4 vColor;
#elif defined( USE_COLOR )
	varying vec3 vColor;
#endif`,_g=`#if defined( USE_COLOR_ALPHA )
	varying vec4 vColor;
#elif defined( USE_COLOR ) || defined( USE_INSTANCING_COLOR )
	varying vec3 vColor;
#endif`,vg=`#if defined( USE_COLOR_ALPHA )
	vColor = vec4( 1.0 );
#elif defined( USE_COLOR ) || defined( USE_INSTANCING_COLOR )
	vColor = vec3( 1.0 );
#endif
#ifdef USE_COLOR
	vColor *= color;
#endif
#ifdef USE_INSTANCING_COLOR
	vColor.xyz *= instanceColor.xyz;
#endif`,Sg=`#define PI 3.141592653589793
#define PI2 6.283185307179586
#define PI_HALF 1.5707963267948966
#define RECIPROCAL_PI 0.3183098861837907
#define RECIPROCAL_PI2 0.15915494309189535
#define EPSILON 1e-6
#ifndef saturate
#define saturate( a ) clamp( a, 0.0, 1.0 )
#endif
#define whiteComplement( a ) ( 1.0 - saturate( a ) )
float pow2( const in float x ) { return x*x; }
vec3 pow2( const in vec3 x ) { return x*x; }
float pow3( const in float x ) { return x*x*x; }
float pow4( const in float x ) { float x2 = x*x; return x2*x2; }
float max3( const in vec3 v ) { return max( max( v.x, v.y ), v.z ); }
float average( const in vec3 v ) { return dot( v, vec3( 0.3333333 ) ); }
highp float rand( const in vec2 uv ) {
	const highp float a = 12.9898, b = 78.233, c = 43758.5453;
	highp float dt = dot( uv.xy, vec2( a,b ) ), sn = mod( dt, PI );
	return fract( sin( sn ) * c );
}
#ifdef HIGH_PRECISION
	float precisionSafeLength( vec3 v ) { return length( v ); }
#else
	float precisionSafeLength( vec3 v ) {
		float maxComponent = max3( abs( v ) );
		return length( v / maxComponent ) * maxComponent;
	}
#endif
struct IncidentLight {
	vec3 color;
	vec3 direction;
	bool visible;
};
struct ReflectedLight {
	vec3 directDiffuse;
	vec3 directSpecular;
	vec3 indirectDiffuse;
	vec3 indirectSpecular;
};
#ifdef USE_ALPHAHASH
	varying vec3 vPosition;
#endif
vec3 transformDirection( in vec3 dir, in mat4 matrix ) {
	return normalize( ( matrix * vec4( dir, 0.0 ) ).xyz );
}
vec3 inverseTransformDirection( in vec3 dir, in mat4 matrix ) {
	return normalize( ( vec4( dir, 0.0 ) * matrix ).xyz );
}
mat3 transposeMat3( const in mat3 m ) {
	mat3 tmp;
	tmp[ 0 ] = vec3( m[ 0 ].x, m[ 1 ].x, m[ 2 ].x );
	tmp[ 1 ] = vec3( m[ 0 ].y, m[ 1 ].y, m[ 2 ].y );
	tmp[ 2 ] = vec3( m[ 0 ].z, m[ 1 ].z, m[ 2 ].z );
	return tmp;
}
float luminance( const in vec3 rgb ) {
	const vec3 weights = vec3( 0.2126729, 0.7151522, 0.0721750 );
	return dot( weights, rgb );
}
bool isPerspectiveMatrix( mat4 m ) {
	return m[ 2 ][ 3 ] == - 1.0;
}
vec2 equirectUv( in vec3 dir ) {
	float u = atan( dir.z, dir.x ) * RECIPROCAL_PI2 + 0.5;
	float v = asin( clamp( dir.y, - 1.0, 1.0 ) ) * RECIPROCAL_PI + 0.5;
	return vec2( u, v );
}
vec3 BRDF_Lambert( const in vec3 diffuseColor ) {
	return RECIPROCAL_PI * diffuseColor;
}
vec3 F_Schlick( const in vec3 f0, const in float f90, const in float dotVH ) {
	float fresnel = exp2( ( - 5.55473 * dotVH - 6.98316 ) * dotVH );
	return f0 * ( 1.0 - fresnel ) + ( f90 * fresnel );
}
float F_Schlick( const in float f0, const in float f90, const in float dotVH ) {
	float fresnel = exp2( ( - 5.55473 * dotVH - 6.98316 ) * dotVH );
	return f0 * ( 1.0 - fresnel ) + ( f90 * fresnel );
} // validated`,Mg=`#ifdef ENVMAP_TYPE_CUBE_UV
	#define cubeUV_minMipLevel 4.0
	#define cubeUV_minTileSize 16.0
	float getFace( vec3 direction ) {
		vec3 absDirection = abs( direction );
		float face = - 1.0;
		if ( absDirection.x > absDirection.z ) {
			if ( absDirection.x > absDirection.y )
				face = direction.x > 0.0 ? 0.0 : 3.0;
			else
				face = direction.y > 0.0 ? 1.0 : 4.0;
		} else {
			if ( absDirection.z > absDirection.y )
				face = direction.z > 0.0 ? 2.0 : 5.0;
			else
				face = direction.y > 0.0 ? 1.0 : 4.0;
		}
		return face;
	}
	vec2 getUV( vec3 direction, float face ) {
		vec2 uv;
		if ( face == 0.0 ) {
			uv = vec2( direction.z, direction.y ) / abs( direction.x );
		} else if ( face == 1.0 ) {
			uv = vec2( - direction.x, - direction.z ) / abs( direction.y );
		} else if ( face == 2.0 ) {
			uv = vec2( - direction.x, direction.y ) / abs( direction.z );
		} else if ( face == 3.0 ) {
			uv = vec2( - direction.z, direction.y ) / abs( direction.x );
		} else if ( face == 4.0 ) {
			uv = vec2( - direction.x, direction.z ) / abs( direction.y );
		} else {
			uv = vec2( direction.x, direction.y ) / abs( direction.z );
		}
		return 0.5 * ( uv + 1.0 );
	}
	vec3 bilinearCubeUV( sampler2D envMap, vec3 direction, float mipInt ) {
		float face = getFace( direction );
		float filterInt = max( cubeUV_minMipLevel - mipInt, 0.0 );
		mipInt = max( mipInt, cubeUV_minMipLevel );
		float faceSize = exp2( mipInt );
		highp vec2 uv = getUV( direction, face ) * ( faceSize - 2.0 ) + 1.0;
		if ( face > 2.0 ) {
			uv.y += faceSize;
			face -= 3.0;
		}
		uv.x += face * faceSize;
		uv.x += filterInt * 3.0 * cubeUV_minTileSize;
		uv.y += 4.0 * ( exp2( CUBEUV_MAX_MIP ) - faceSize );
		uv.x *= CUBEUV_TEXEL_WIDTH;
		uv.y *= CUBEUV_TEXEL_HEIGHT;
		#ifdef texture2DGradEXT
			return texture2DGradEXT( envMap, uv, vec2( 0.0 ), vec2( 0.0 ) ).rgb;
		#else
			return texture2D( envMap, uv ).rgb;
		#endif
	}
	#define cubeUV_r0 1.0
	#define cubeUV_m0 - 2.0
	#define cubeUV_r1 0.8
	#define cubeUV_m1 - 1.0
	#define cubeUV_r4 0.4
	#define cubeUV_m4 2.0
	#define cubeUV_r5 0.305
	#define cubeUV_m5 3.0
	#define cubeUV_r6 0.21
	#define cubeUV_m6 4.0
	float roughnessToMip( float roughness ) {
		float mip = 0.0;
		if ( roughness >= cubeUV_r1 ) {
			mip = ( cubeUV_r0 - roughness ) * ( cubeUV_m1 - cubeUV_m0 ) / ( cubeUV_r0 - cubeUV_r1 ) + cubeUV_m0;
		} else if ( roughness >= cubeUV_r4 ) {
			mip = ( cubeUV_r1 - roughness ) * ( cubeUV_m4 - cubeUV_m1 ) / ( cubeUV_r1 - cubeUV_r4 ) + cubeUV_m1;
		} else if ( roughness >= cubeUV_r5 ) {
			mip = ( cubeUV_r4 - roughness ) * ( cubeUV_m5 - cubeUV_m4 ) / ( cubeUV_r4 - cubeUV_r5 ) + cubeUV_m4;
		} else if ( roughness >= cubeUV_r6 ) {
			mip = ( cubeUV_r5 - roughness ) * ( cubeUV_m6 - cubeUV_m5 ) / ( cubeUV_r5 - cubeUV_r6 ) + cubeUV_m5;
		} else {
			mip = - 2.0 * log2( 1.16 * roughness );		}
		return mip;
	}
	vec4 textureCubeUV( sampler2D envMap, vec3 sampleDir, float roughness ) {
		float mip = clamp( roughnessToMip( roughness ), cubeUV_m0, CUBEUV_MAX_MIP );
		float mipF = fract( mip );
		float mipInt = floor( mip );
		vec3 color0 = bilinearCubeUV( envMap, sampleDir, mipInt );
		if ( mipF == 0.0 ) {
			return vec4( color0, 1.0 );
		} else {
			vec3 color1 = bilinearCubeUV( envMap, sampleDir, mipInt + 1.0 );
			return vec4( mix( color0, color1, mipF ), 1.0 );
		}
	}
#endif`,yg=`vec3 transformedNormal = objectNormal;
#ifdef USE_TANGENT
	vec3 transformedTangent = objectTangent;
#endif
#ifdef USE_BATCHING
	mat3 bm = mat3( batchingMatrix );
	transformedNormal /= vec3( dot( bm[ 0 ], bm[ 0 ] ), dot( bm[ 1 ], bm[ 1 ] ), dot( bm[ 2 ], bm[ 2 ] ) );
	transformedNormal = bm * transformedNormal;
	#ifdef USE_TANGENT
		transformedTangent = bm * transformedTangent;
	#endif
#endif
#ifdef USE_INSTANCING
	mat3 im = mat3( instanceMatrix );
	transformedNormal /= vec3( dot( im[ 0 ], im[ 0 ] ), dot( im[ 1 ], im[ 1 ] ), dot( im[ 2 ], im[ 2 ] ) );
	transformedNormal = im * transformedNormal;
	#ifdef USE_TANGENT
		transformedTangent = im * transformedTangent;
	#endif
#endif
transformedNormal = normalMatrix * transformedNormal;
#ifdef FLIP_SIDED
	transformedNormal = - transformedNormal;
#endif
#ifdef USE_TANGENT
	transformedTangent = ( modelViewMatrix * vec4( transformedTangent, 0.0 ) ).xyz;
	#ifdef FLIP_SIDED
		transformedTangent = - transformedTangent;
	#endif
#endif`,Eg=`#ifdef USE_DISPLACEMENTMAP
	uniform sampler2D displacementMap;
	uniform float displacementScale;
	uniform float displacementBias;
#endif`,wg=`#ifdef USE_DISPLACEMENTMAP
	transformed += normalize( objectNormal ) * ( texture2D( displacementMap, vDisplacementMapUv ).x * displacementScale + displacementBias );
#endif`,Tg=`#ifdef USE_EMISSIVEMAP
	vec4 emissiveColor = texture2D( emissiveMap, vEmissiveMapUv );
	totalEmissiveRadiance *= emissiveColor.rgb;
#endif`,Ag=`#ifdef USE_EMISSIVEMAP
	uniform sampler2D emissiveMap;
#endif`,Cg="gl_FragColor = linearToOutputTexel( gl_FragColor );",Rg=`
const mat3 LINEAR_SRGB_TO_LINEAR_DISPLAY_P3 = mat3(
	vec3( 0.8224621, 0.177538, 0.0 ),
	vec3( 0.0331941, 0.9668058, 0.0 ),
	vec3( 0.0170827, 0.0723974, 0.9105199 )
);
const mat3 LINEAR_DISPLAY_P3_TO_LINEAR_SRGB = mat3(
	vec3( 1.2249401, - 0.2249404, 0.0 ),
	vec3( - 0.0420569, 1.0420571, 0.0 ),
	vec3( - 0.0196376, - 0.0786361, 1.0982735 )
);
vec4 LinearSRGBToLinearDisplayP3( in vec4 value ) {
	return vec4( value.rgb * LINEAR_SRGB_TO_LINEAR_DISPLAY_P3, value.a );
}
vec4 LinearDisplayP3ToLinearSRGB( in vec4 value ) {
	return vec4( value.rgb * LINEAR_DISPLAY_P3_TO_LINEAR_SRGB, value.a );
}
vec4 LinearTransferOETF( in vec4 value ) {
	return value;
}
vec4 sRGBTransferOETF( in vec4 value ) {
	return vec4( mix( pow( value.rgb, vec3( 0.41666 ) ) * 1.055 - vec3( 0.055 ), value.rgb * 12.92, vec3( lessThanEqual( value.rgb, vec3( 0.0031308 ) ) ) ), value.a );
}
vec4 LinearToLinear( in vec4 value ) {
	return value;
}
vec4 LinearTosRGB( in vec4 value ) {
	return sRGBTransferOETF( value );
}`,bg=`#ifdef USE_ENVMAP
	#ifdef ENV_WORLDPOS
		vec3 cameraToFrag;
		if ( isOrthographic ) {
			cameraToFrag = normalize( vec3( - viewMatrix[ 0 ][ 2 ], - viewMatrix[ 1 ][ 2 ], - viewMatrix[ 2 ][ 2 ] ) );
		} else {
			cameraToFrag = normalize( vWorldPosition - cameraPosition );
		}
		vec3 worldNormal = inverseTransformDirection( normal, viewMatrix );
		#ifdef ENVMAP_MODE_REFLECTION
			vec3 reflectVec = reflect( cameraToFrag, worldNormal );
		#else
			vec3 reflectVec = refract( cameraToFrag, worldNormal, refractionRatio );
		#endif
	#else
		vec3 reflectVec = vReflect;
	#endif
	#ifdef ENVMAP_TYPE_CUBE
		vec4 envColor = textureCube( envMap, vec3( flipEnvMap * reflectVec.x, reflectVec.yz ) );
	#else
		vec4 envColor = vec4( 0.0 );
	#endif
	#ifdef ENVMAP_BLENDING_MULTIPLY
		outgoingLight = mix( outgoingLight, outgoingLight * envColor.xyz, specularStrength * reflectivity );
	#elif defined( ENVMAP_BLENDING_MIX )
		outgoingLight = mix( outgoingLight, envColor.xyz, specularStrength * reflectivity );
	#elif defined( ENVMAP_BLENDING_ADD )
		outgoingLight += envColor.xyz * specularStrength * reflectivity;
	#endif
#endif`,Pg=`#ifdef USE_ENVMAP
	uniform float envMapIntensity;
	uniform float flipEnvMap;
	#ifdef ENVMAP_TYPE_CUBE
		uniform samplerCube envMap;
	#else
		uniform sampler2D envMap;
	#endif
	
#endif`,Lg=`#ifdef USE_ENVMAP
	uniform float reflectivity;
	#if defined( USE_BUMPMAP ) || defined( USE_NORMALMAP ) || defined( PHONG ) || defined( LAMBERT )
		#define ENV_WORLDPOS
	#endif
	#ifdef ENV_WORLDPOS
		varying vec3 vWorldPosition;
		uniform float refractionRatio;
	#else
		varying vec3 vReflect;
	#endif
#endif`,Dg=`#ifdef USE_ENVMAP
	#if defined( USE_BUMPMAP ) || defined( USE_NORMALMAP ) || defined( PHONG ) || defined( LAMBERT )
		#define ENV_WORLDPOS
	#endif
	#ifdef ENV_WORLDPOS
		
		varying vec3 vWorldPosition;
	#else
		varying vec3 vReflect;
		uniform float refractionRatio;
	#endif
#endif`,Ig=`#ifdef USE_ENVMAP
	#ifdef ENV_WORLDPOS
		vWorldPosition = worldPosition.xyz;
	#else
		vec3 cameraToVertex;
		if ( isOrthographic ) {
			cameraToVertex = normalize( vec3( - viewMatrix[ 0 ][ 2 ], - viewMatrix[ 1 ][ 2 ], - viewMatrix[ 2 ][ 2 ] ) );
		} else {
			cameraToVertex = normalize( worldPosition.xyz - cameraPosition );
		}
		vec3 worldNormal = inverseTransformDirection( transformedNormal, viewMatrix );
		#ifdef ENVMAP_MODE_REFLECTION
			vReflect = reflect( cameraToVertex, worldNormal );
		#else
			vReflect = refract( cameraToVertex, worldNormal, refractionRatio );
		#endif
	#endif
#endif`,Ng=`#ifdef USE_FOG
	vFogDepth = - mvPosition.z;
#endif`,Fg=`#ifdef USE_FOG
	varying float vFogDepth;
#endif`,Bg=`#ifdef USE_FOG
	#ifdef FOG_EXP2
		float fogFactor = 1.0 - exp( - fogDensity * fogDensity * vFogDepth * vFogDepth );
	#else
		float fogFactor = smoothstep( fogNear, fogFar, vFogDepth );
	#endif
	gl_FragColor.rgb = mix( gl_FragColor.rgb, fogColor, fogFactor );
#endif`,Ug=`#ifdef USE_FOG
	uniform vec3 fogColor;
	varying float vFogDepth;
	#ifdef FOG_EXP2
		uniform float fogDensity;
	#else
		uniform float fogNear;
		uniform float fogFar;
	#endif
#endif`,zg=`#ifdef USE_GRADIENTMAP
	uniform sampler2D gradientMap;
#endif
vec3 getGradientIrradiance( vec3 normal, vec3 lightDirection ) {
	float dotNL = dot( normal, lightDirection );
	vec2 coord = vec2( dotNL * 0.5 + 0.5, 0.0 );
	#ifdef USE_GRADIENTMAP
		return vec3( texture2D( gradientMap, coord ).r );
	#else
		vec2 fw = fwidth( coord ) * 0.5;
		return mix( vec3( 0.7 ), vec3( 1.0 ), smoothstep( 0.7 - fw.x, 0.7 + fw.x, coord.x ) );
	#endif
}`,Og=`#ifdef USE_LIGHTMAP
	vec4 lightMapTexel = texture2D( lightMap, vLightMapUv );
	vec3 lightMapIrradiance = lightMapTexel.rgb * lightMapIntensity;
	reflectedLight.indirectDiffuse += lightMapIrradiance;
#endif`,Gg=`#ifdef USE_LIGHTMAP
	uniform sampler2D lightMap;
	uniform float lightMapIntensity;
#endif`,Vg=`LambertMaterial material;
material.diffuseColor = diffuseColor.rgb;
material.specularStrength = specularStrength;`,kg=`varying vec3 vViewPosition;
struct LambertMaterial {
	vec3 diffuseColor;
	float specularStrength;
};
void RE_Direct_Lambert( const in IncidentLight directLight, const in vec3 geometryPosition, const in vec3 geometryNormal, const in vec3 geometryViewDir, const in vec3 geometryClearcoatNormal, const in LambertMaterial material, inout ReflectedLight reflectedLight ) {
	float dotNL = saturate( dot( geometryNormal, directLight.direction ) );
	vec3 irradiance = dotNL * directLight.color;
	reflectedLight.directDiffuse += irradiance * BRDF_Lambert( material.diffuseColor );
}
void RE_IndirectDiffuse_Lambert( const in vec3 irradiance, const in vec3 geometryPosition, const in vec3 geometryNormal, const in vec3 geometryViewDir, const in vec3 geometryClearcoatNormal, const in LambertMaterial material, inout ReflectedLight reflectedLight ) {
	reflectedLight.indirectDiffuse += irradiance * BRDF_Lambert( material.diffuseColor );
}
#define RE_Direct				RE_Direct_Lambert
#define RE_IndirectDiffuse		RE_IndirectDiffuse_Lambert`,Hg=`uniform bool receiveShadow;
uniform vec3 ambientLightColor;
#if defined( USE_LIGHT_PROBES )
	uniform vec3 lightProbe[ 9 ];
#endif
vec3 shGetIrradianceAt( in vec3 normal, in vec3 shCoefficients[ 9 ] ) {
	float x = normal.x, y = normal.y, z = normal.z;
	vec3 result = shCoefficients[ 0 ] * 0.886227;
	result += shCoefficients[ 1 ] * 2.0 * 0.511664 * y;
	result += shCoefficients[ 2 ] * 2.0 * 0.511664 * z;
	result += shCoefficients[ 3 ] * 2.0 * 0.511664 * x;
	result += shCoefficients[ 4 ] * 2.0 * 0.429043 * x * y;
	result += shCoefficients[ 5 ] * 2.0 * 0.429043 * y * z;
	result += shCoefficients[ 6 ] * ( 0.743125 * z * z - 0.247708 );
	result += shCoefficients[ 7 ] * 2.0 * 0.429043 * x * z;
	result += shCoefficients[ 8 ] * 0.429043 * ( x * x - y * y );
	return result;
}
vec3 getLightProbeIrradiance( const in vec3 lightProbe[ 9 ], const in vec3 normal ) {
	vec3 worldNormal = inverseTransformDirection( normal, viewMatrix );
	vec3 irradiance = shGetIrradianceAt( worldNormal, lightProbe );
	return irradiance;
}
vec3 getAmbientLightIrradiance( const in vec3 ambientLightColor ) {
	vec3 irradiance = ambientLightColor;
	return irradiance;
}
float getDistanceAttenuation( const in float lightDistance, const in float cutoffDistance, const in float decayExponent ) {
	#if defined ( LEGACY_LIGHTS )
		if ( cutoffDistance > 0.0 && decayExponent > 0.0 ) {
			return pow( saturate( - lightDistance / cutoffDistance + 1.0 ), decayExponent );
		}
		return 1.0;
	#else
		float distanceFalloff = 1.0 / max( pow( lightDistance, decayExponent ), 0.01 );
		if ( cutoffDistance > 0.0 ) {
			distanceFalloff *= pow2( saturate( 1.0 - pow4( lightDistance / cutoffDistance ) ) );
		}
		return distanceFalloff;
	#endif
}
float getSpotAttenuation( const in float coneCosine, const in float penumbraCosine, const in float angleCosine ) {
	return smoothstep( coneCosine, penumbraCosine, angleCosine );
}
#if NUM_DIR_LIGHTS > 0
	struct DirectionalLight {
		vec3 direction;
		vec3 color;
	};
	uniform DirectionalLight directionalLights[ NUM_DIR_LIGHTS ];
	void getDirectionalLightInfo( const in DirectionalLight directionalLight, out IncidentLight light ) {
		light.color = directionalLight.color;
		light.direction = directionalLight.direction;
		light.visible = true;
	}
#endif
#if NUM_POINT_LIGHTS > 0
	struct PointLight {
		vec3 position;
		vec3 color;
		float distance;
		float decay;
	};
	uniform PointLight pointLights[ NUM_POINT_LIGHTS ];
	void getPointLightInfo( const in PointLight pointLight, const in vec3 geometryPosition, out IncidentLight light ) {
		vec3 lVector = pointLight.position - geometryPosition;
		light.direction = normalize( lVector );
		float lightDistance = length( lVector );
		light.color = pointLight.color;
		light.color *= getDistanceAttenuation( lightDistance, pointLight.distance, pointLight.decay );
		light.visible = ( light.color != vec3( 0.0 ) );
	}
#endif
#if NUM_SPOT_LIGHTS > 0
	struct SpotLight {
		vec3 position;
		vec3 direction;
		vec3 color;
		float distance;
		float decay;
		float coneCos;
		float penumbraCos;
	};
	uniform SpotLight spotLights[ NUM_SPOT_LIGHTS ];
	void getSpotLightInfo( const in SpotLight spotLight, const in vec3 geometryPosition, out IncidentLight light ) {
		vec3 lVector = spotLight.position - geometryPosition;
		light.direction = normalize( lVector );
		float angleCos = dot( light.direction, spotLight.direction );
		float spotAttenuation = getSpotAttenuation( spotLight.coneCos, spotLight.penumbraCos, angleCos );
		if ( spotAttenuation > 0.0 ) {
			float lightDistance = length( lVector );
			light.color = spotLight.color * spotAttenuation;
			light.color *= getDistanceAttenuation( lightDistance, spotLight.distance, spotLight.decay );
			light.visible = ( light.color != vec3( 0.0 ) );
		} else {
			light.color = vec3( 0.0 );
			light.visible = false;
		}
	}
#endif
#if NUM_RECT_AREA_LIGHTS > 0
	struct RectAreaLight {
		vec3 color;
		vec3 position;
		vec3 halfWidth;
		vec3 halfHeight;
	};
	uniform sampler2D ltc_1;	uniform sampler2D ltc_2;
	uniform RectAreaLight rectAreaLights[ NUM_RECT_AREA_LIGHTS ];
#endif
#if NUM_HEMI_LIGHTS > 0
	struct HemisphereLight {
		vec3 direction;
		vec3 skyColor;
		vec3 groundColor;
	};
	uniform HemisphereLight hemisphereLights[ NUM_HEMI_LIGHTS ];
	vec3 getHemisphereLightIrradiance( const in HemisphereLight hemiLight, const in vec3 normal ) {
		float dotNL = dot( normal, hemiLight.direction );
		float hemiDiffuseWeight = 0.5 * dotNL + 0.5;
		vec3 irradiance = mix( hemiLight.groundColor, hemiLight.skyColor, hemiDiffuseWeight );
		return irradiance;
	}
#endif`,Wg=`#ifdef USE_ENVMAP
	vec3 getIBLIrradiance( const in vec3 normal ) {
		#ifdef ENVMAP_TYPE_CUBE_UV
			vec3 worldNormal = inverseTransformDirection( normal, viewMatrix );
			vec4 envMapColor = textureCubeUV( envMap, worldNormal, 1.0 );
			return PI * envMapColor.rgb * envMapIntensity;
		#else
			return vec3( 0.0 );
		#endif
	}
	vec3 getIBLRadiance( const in vec3 viewDir, const in vec3 normal, const in float roughness ) {
		#ifdef ENVMAP_TYPE_CUBE_UV
			vec3 reflectVec = reflect( - viewDir, normal );
			reflectVec = normalize( mix( reflectVec, normal, roughness * roughness) );
			reflectVec = inverseTransformDirection( reflectVec, viewMatrix );
			vec4 envMapColor = textureCubeUV( envMap, reflectVec, roughness );
			return envMapColor.rgb * envMapIntensity;
		#else
			return vec3( 0.0 );
		#endif
	}
	#ifdef USE_ANISOTROPY
		vec3 getIBLAnisotropyRadiance( const in vec3 viewDir, const in vec3 normal, const in float roughness, const in vec3 bitangent, const in float anisotropy ) {
			#ifdef ENVMAP_TYPE_CUBE_UV
				vec3 bentNormal = cross( bitangent, viewDir );
				bentNormal = normalize( cross( bentNormal, bitangent ) );
				bentNormal = normalize( mix( bentNormal, normal, pow2( pow2( 1.0 - anisotropy * ( 1.0 - roughness ) ) ) ) );
				return getIBLRadiance( viewDir, bentNormal, roughness );
			#else
				return vec3( 0.0 );
			#endif
		}
	#endif
#endif`,Xg=`ToonMaterial material;
material.diffuseColor = diffuseColor.rgb;`,Yg=`varying vec3 vViewPosition;
struct ToonMaterial {
	vec3 diffuseColor;
};
void RE_Direct_Toon( const in IncidentLight directLight, const in vec3 geometryPosition, const in vec3 geometryNormal, const in vec3 geometryViewDir, const in vec3 geometryClearcoatNormal, const in ToonMaterial material, inout ReflectedLight reflectedLight ) {
	vec3 irradiance = getGradientIrradiance( geometryNormal, directLight.direction ) * directLight.color;
	reflectedLight.directDiffuse += irradiance * BRDF_Lambert( material.diffuseColor );
}
void RE_IndirectDiffuse_Toon( const in vec3 irradiance, const in vec3 geometryPosition, const in vec3 geometryNormal, const in vec3 geometryViewDir, const in vec3 geometryClearcoatNormal, const in ToonMaterial material, inout ReflectedLight reflectedLight ) {
	reflectedLight.indirectDiffuse += irradiance * BRDF_Lambert( material.diffuseColor );
}
#define RE_Direct				RE_Direct_Toon
#define RE_IndirectDiffuse		RE_IndirectDiffuse_Toon`,qg=`BlinnPhongMaterial material;
material.diffuseColor = diffuseColor.rgb;
material.specularColor = specular;
material.specularShininess = shininess;
material.specularStrength = specularStrength;`,Zg=`varying vec3 vViewPosition;
struct BlinnPhongMaterial {
	vec3 diffuseColor;
	vec3 specularColor;
	float specularShininess;
	float specularStrength;
};
void RE_Direct_BlinnPhong( const in IncidentLight directLight, const in vec3 geometryPosition, const in vec3 geometryNormal, const in vec3 geometryViewDir, const in vec3 geometryClearcoatNormal, const in BlinnPhongMaterial material, inout ReflectedLight reflectedLight ) {
	float dotNL = saturate( dot( geometryNormal, directLight.direction ) );
	vec3 irradiance = dotNL * directLight.color;
	reflectedLight.directDiffuse += irradiance * BRDF_Lambert( material.diffuseColor );
	reflectedLight.directSpecular += irradiance * BRDF_BlinnPhong( directLight.direction, geometryViewDir, geometryNormal, material.specularColor, material.specularShininess ) * material.specularStrength;
}
void RE_IndirectDiffuse_BlinnPhong( const in vec3 irradiance, const in vec3 geometryPosition, const in vec3 geometryNormal, const in vec3 geometryViewDir, const in vec3 geometryClearcoatNormal, const in BlinnPhongMaterial material, inout ReflectedLight reflectedLight ) {
	reflectedLight.indirectDiffuse += irradiance * BRDF_Lambert( material.diffuseColor );
}
#define RE_Direct				RE_Direct_BlinnPhong
#define RE_IndirectDiffuse		RE_IndirectDiffuse_BlinnPhong`,jg=`PhysicalMaterial material;
material.diffuseColor = diffuseColor.rgb * ( 1.0 - metalnessFactor );
vec3 dxy = max( abs( dFdx( nonPerturbedNormal ) ), abs( dFdy( nonPerturbedNormal ) ) );
float geometryRoughness = max( max( dxy.x, dxy.y ), dxy.z );
material.roughness = max( roughnessFactor, 0.0525 );material.roughness += geometryRoughness;
material.roughness = min( material.roughness, 1.0 );
#ifdef IOR
	material.ior = ior;
	#ifdef USE_SPECULAR
		float specularIntensityFactor = specularIntensity;
		vec3 specularColorFactor = specularColor;
		#ifdef USE_SPECULAR_COLORMAP
			specularColorFactor *= texture2D( specularColorMap, vSpecularColorMapUv ).rgb;
		#endif
		#ifdef USE_SPECULAR_INTENSITYMAP
			specularIntensityFactor *= texture2D( specularIntensityMap, vSpecularIntensityMapUv ).a;
		#endif
		material.specularF90 = mix( specularIntensityFactor, 1.0, metalnessFactor );
	#else
		float specularIntensityFactor = 1.0;
		vec3 specularColorFactor = vec3( 1.0 );
		material.specularF90 = 1.0;
	#endif
	material.specularColor = mix( min( pow2( ( material.ior - 1.0 ) / ( material.ior + 1.0 ) ) * specularColorFactor, vec3( 1.0 ) ) * specularIntensityFactor, diffuseColor.rgb, metalnessFactor );
#else
	material.specularColor = mix( vec3( 0.04 ), diffuseColor.rgb, metalnessFactor );
	material.specularF90 = 1.0;
#endif
#ifdef USE_CLEARCOAT
	material.clearcoat = clearcoat;
	material.clearcoatRoughness = clearcoatRoughness;
	material.clearcoatF0 = vec3( 0.04 );
	material.clearcoatF90 = 1.0;
	#ifdef USE_CLEARCOATMAP
		material.clearcoat *= texture2D( clearcoatMap, vClearcoatMapUv ).x;
	#endif
	#ifdef USE_CLEARCOAT_ROUGHNESSMAP
		material.clearcoatRoughness *= texture2D( clearcoatRoughnessMap, vClearcoatRoughnessMapUv ).y;
	#endif
	material.clearcoat = saturate( material.clearcoat );	material.clearcoatRoughness = max( material.clearcoatRoughness, 0.0525 );
	material.clearcoatRoughness += geometryRoughness;
	material.clearcoatRoughness = min( material.clearcoatRoughness, 1.0 );
#endif
#ifdef USE_IRIDESCENCE
	material.iridescence = iridescence;
	material.iridescenceIOR = iridescenceIOR;
	#ifdef USE_IRIDESCENCEMAP
		material.iridescence *= texture2D( iridescenceMap, vIridescenceMapUv ).r;
	#endif
	#ifdef USE_IRIDESCENCE_THICKNESSMAP
		material.iridescenceThickness = (iridescenceThicknessMaximum - iridescenceThicknessMinimum) * texture2D( iridescenceThicknessMap, vIridescenceThicknessMapUv ).g + iridescenceThicknessMinimum;
	#else
		material.iridescenceThickness = iridescenceThicknessMaximum;
	#endif
#endif
#ifdef USE_SHEEN
	material.sheenColor = sheenColor;
	#ifdef USE_SHEEN_COLORMAP
		material.sheenColor *= texture2D( sheenColorMap, vSheenColorMapUv ).rgb;
	#endif
	material.sheenRoughness = clamp( sheenRoughness, 0.07, 1.0 );
	#ifdef USE_SHEEN_ROUGHNESSMAP
		material.sheenRoughness *= texture2D( sheenRoughnessMap, vSheenRoughnessMapUv ).a;
	#endif
#endif
#ifdef USE_ANISOTROPY
	#ifdef USE_ANISOTROPYMAP
		mat2 anisotropyMat = mat2( anisotropyVector.x, anisotropyVector.y, - anisotropyVector.y, anisotropyVector.x );
		vec3 anisotropyPolar = texture2D( anisotropyMap, vAnisotropyMapUv ).rgb;
		vec2 anisotropyV = anisotropyMat * normalize( 2.0 * anisotropyPolar.rg - vec2( 1.0 ) ) * anisotropyPolar.b;
	#else
		vec2 anisotropyV = anisotropyVector;
	#endif
	material.anisotropy = length( anisotropyV );
	if( material.anisotropy == 0.0 ) {
		anisotropyV = vec2( 1.0, 0.0 );
	} else {
		anisotropyV /= material.anisotropy;
		material.anisotropy = saturate( material.anisotropy );
	}
	material.alphaT = mix( pow2( material.roughness ), 1.0, pow2( material.anisotropy ) );
	material.anisotropyT = tbn[ 0 ] * anisotropyV.x + tbn[ 1 ] * anisotropyV.y;
	material.anisotropyB = tbn[ 1 ] * anisotropyV.x - tbn[ 0 ] * anisotropyV.y;
#endif`,$g=`struct PhysicalMaterial {
	vec3 diffuseColor;
	float roughness;
	vec3 specularColor;
	float specularF90;
	#ifdef USE_CLEARCOAT
		float clearcoat;
		float clearcoatRoughness;
		vec3 clearcoatF0;
		float clearcoatF90;
	#endif
	#ifdef USE_IRIDESCENCE
		float iridescence;
		float iridescenceIOR;
		float iridescenceThickness;
		vec3 iridescenceFresnel;
		vec3 iridescenceF0;
	#endif
	#ifdef USE_SHEEN
		vec3 sheenColor;
		float sheenRoughness;
	#endif
	#ifdef IOR
		float ior;
	#endif
	#ifdef USE_TRANSMISSION
		float transmission;
		float transmissionAlpha;
		float thickness;
		float attenuationDistance;
		vec3 attenuationColor;
	#endif
	#ifdef USE_ANISOTROPY
		float anisotropy;
		float alphaT;
		vec3 anisotropyT;
		vec3 anisotropyB;
	#endif
};
vec3 clearcoatSpecularDirect = vec3( 0.0 );
vec3 clearcoatSpecularIndirect = vec3( 0.0 );
vec3 sheenSpecularDirect = vec3( 0.0 );
vec3 sheenSpecularIndirect = vec3(0.0 );
vec3 Schlick_to_F0( const in vec3 f, const in float f90, const in float dotVH ) {
    float x = clamp( 1.0 - dotVH, 0.0, 1.0 );
    float x2 = x * x;
    float x5 = clamp( x * x2 * x2, 0.0, 0.9999 );
    return ( f - vec3( f90 ) * x5 ) / ( 1.0 - x5 );
}
float V_GGX_SmithCorrelated( const in float alpha, const in float dotNL, const in float dotNV ) {
	float a2 = pow2( alpha );
	float gv = dotNL * sqrt( a2 + ( 1.0 - a2 ) * pow2( dotNV ) );
	float gl = dotNV * sqrt( a2 + ( 1.0 - a2 ) * pow2( dotNL ) );
	return 0.5 / max( gv + gl, EPSILON );
}
float D_GGX( const in float alpha, const in float dotNH ) {
	float a2 = pow2( alpha );
	float denom = pow2( dotNH ) * ( a2 - 1.0 ) + 1.0;
	return RECIPROCAL_PI * a2 / pow2( denom );
}
#ifdef USE_ANISOTROPY
	float V_GGX_SmithCorrelated_Anisotropic( const in float alphaT, const in float alphaB, const in float dotTV, const in float dotBV, const in float dotTL, const in float dotBL, const in float dotNV, const in float dotNL ) {
		float gv = dotNL * length( vec3( alphaT * dotTV, alphaB * dotBV, dotNV ) );
		float gl = dotNV * length( vec3( alphaT * dotTL, alphaB * dotBL, dotNL ) );
		float v = 0.5 / ( gv + gl );
		return saturate(v);
	}
	float D_GGX_Anisotropic( const in float alphaT, const in float alphaB, const in float dotNH, const in float dotTH, const in float dotBH ) {
		float a2 = alphaT * alphaB;
		highp vec3 v = vec3( alphaB * dotTH, alphaT * dotBH, a2 * dotNH );
		highp float v2 = dot( v, v );
		float w2 = a2 / v2;
		return RECIPROCAL_PI * a2 * pow2 ( w2 );
	}
#endif
#ifdef USE_CLEARCOAT
	vec3 BRDF_GGX_Clearcoat( const in vec3 lightDir, const in vec3 viewDir, const in vec3 normal, const in PhysicalMaterial material) {
		vec3 f0 = material.clearcoatF0;
		float f90 = material.clearcoatF90;
		float roughness = material.clearcoatRoughness;
		float alpha = pow2( roughness );
		vec3 halfDir = normalize( lightDir + viewDir );
		float dotNL = saturate( dot( normal, lightDir ) );
		float dotNV = saturate( dot( normal, viewDir ) );
		float dotNH = saturate( dot( normal, halfDir ) );
		float dotVH = saturate( dot( viewDir, halfDir ) );
		vec3 F = F_Schlick( f0, f90, dotVH );
		float V = V_GGX_SmithCorrelated( alpha, dotNL, dotNV );
		float D = D_GGX( alpha, dotNH );
		return F * ( V * D );
	}
#endif
vec3 BRDF_GGX( const in vec3 lightDir, const in vec3 viewDir, const in vec3 normal, const in PhysicalMaterial material ) {
	vec3 f0 = material.specularColor;
	float f90 = material.specularF90;
	float roughness = material.roughness;
	float alpha = pow2( roughness );
	vec3 halfDir = normalize( lightDir + viewDir );
	float dotNL = saturate( dot( normal, lightDir ) );
	float dotNV = saturate( dot( normal, viewDir ) );
	float dotNH = saturate( dot( normal, halfDir ) );
	float dotVH = saturate( dot( viewDir, halfDir ) );
	vec3 F = F_Schlick( f0, f90, dotVH );
	#ifdef USE_IRIDESCENCE
		F = mix( F, material.iridescenceFresnel, material.iridescence );
	#endif
	#ifdef USE_ANISOTROPY
		float dotTL = dot( material.anisotropyT, lightDir );
		float dotTV = dot( material.anisotropyT, viewDir );
		float dotTH = dot( material.anisotropyT, halfDir );
		float dotBL = dot( material.anisotropyB, lightDir );
		float dotBV = dot( material.anisotropyB, viewDir );
		float dotBH = dot( material.anisotropyB, halfDir );
		float V = V_GGX_SmithCorrelated_Anisotropic( material.alphaT, alpha, dotTV, dotBV, dotTL, dotBL, dotNV, dotNL );
		float D = D_GGX_Anisotropic( material.alphaT, alpha, dotNH, dotTH, dotBH );
	#else
		float V = V_GGX_SmithCorrelated( alpha, dotNL, dotNV );
		float D = D_GGX( alpha, dotNH );
	#endif
	return F * ( V * D );
}
vec2 LTC_Uv( const in vec3 N, const in vec3 V, const in float roughness ) {
	const float LUT_SIZE = 64.0;
	const float LUT_SCALE = ( LUT_SIZE - 1.0 ) / LUT_SIZE;
	const float LUT_BIAS = 0.5 / LUT_SIZE;
	float dotNV = saturate( dot( N, V ) );
	vec2 uv = vec2( roughness, sqrt( 1.0 - dotNV ) );
	uv = uv * LUT_SCALE + LUT_BIAS;
	return uv;
}
float LTC_ClippedSphereFormFactor( const in vec3 f ) {
	float l = length( f );
	return max( ( l * l + f.z ) / ( l + 1.0 ), 0.0 );
}
vec3 LTC_EdgeVectorFormFactor( const in vec3 v1, const in vec3 v2 ) {
	float x = dot( v1, v2 );
	float y = abs( x );
	float a = 0.8543985 + ( 0.4965155 + 0.0145206 * y ) * y;
	float b = 3.4175940 + ( 4.1616724 + y ) * y;
	float v = a / b;
	float theta_sintheta = ( x > 0.0 ) ? v : 0.5 * inversesqrt( max( 1.0 - x * x, 1e-7 ) ) - v;
	return cross( v1, v2 ) * theta_sintheta;
}
vec3 LTC_Evaluate( const in vec3 N, const in vec3 V, const in vec3 P, const in mat3 mInv, const in vec3 rectCoords[ 4 ] ) {
	vec3 v1 = rectCoords[ 1 ] - rectCoords[ 0 ];
	vec3 v2 = rectCoords[ 3 ] - rectCoords[ 0 ];
	vec3 lightNormal = cross( v1, v2 );
	if( dot( lightNormal, P - rectCoords[ 0 ] ) < 0.0 ) return vec3( 0.0 );
	vec3 T1, T2;
	T1 = normalize( V - N * dot( V, N ) );
	T2 = - cross( N, T1 );
	mat3 mat = mInv * transposeMat3( mat3( T1, T2, N ) );
	vec3 coords[ 4 ];
	coords[ 0 ] = mat * ( rectCoords[ 0 ] - P );
	coords[ 1 ] = mat * ( rectCoords[ 1 ] - P );
	coords[ 2 ] = mat * ( rectCoords[ 2 ] - P );
	coords[ 3 ] = mat * ( rectCoords[ 3 ] - P );
	coords[ 0 ] = normalize( coords[ 0 ] );
	coords[ 1 ] = normalize( coords[ 1 ] );
	coords[ 2 ] = normalize( coords[ 2 ] );
	coords[ 3 ] = normalize( coords[ 3 ] );
	vec3 vectorFormFactor = vec3( 0.0 );
	vectorFormFactor += LTC_EdgeVectorFormFactor( coords[ 0 ], coords[ 1 ] );
	vectorFormFactor += LTC_EdgeVectorFormFactor( coords[ 1 ], coords[ 2 ] );
	vectorFormFactor += LTC_EdgeVectorFormFactor( coords[ 2 ], coords[ 3 ] );
	vectorFormFactor += LTC_EdgeVectorFormFactor( coords[ 3 ], coords[ 0 ] );
	float result = LTC_ClippedSphereFormFactor( vectorFormFactor );
	return vec3( result );
}
#if defined( USE_SHEEN )
float D_Charlie( float roughness, float dotNH ) {
	float alpha = pow2( roughness );
	float invAlpha = 1.0 / alpha;
	float cos2h = dotNH * dotNH;
	float sin2h = max( 1.0 - cos2h, 0.0078125 );
	return ( 2.0 + invAlpha ) * pow( sin2h, invAlpha * 0.5 ) / ( 2.0 * PI );
}
float V_Neubelt( float dotNV, float dotNL ) {
	return saturate( 1.0 / ( 4.0 * ( dotNL + dotNV - dotNL * dotNV ) ) );
}
vec3 BRDF_Sheen( const in vec3 lightDir, const in vec3 viewDir, const in vec3 normal, vec3 sheenColor, const in float sheenRoughness ) {
	vec3 halfDir = normalize( lightDir + viewDir );
	float dotNL = saturate( dot( normal, lightDir ) );
	float dotNV = saturate( dot( normal, viewDir ) );
	float dotNH = saturate( dot( normal, halfDir ) );
	float D = D_Charlie( sheenRoughness, dotNH );
	float V = V_Neubelt( dotNV, dotNL );
	return sheenColor * ( D * V );
}
#endif
float IBLSheenBRDF( const in vec3 normal, const in vec3 viewDir, const in float roughness ) {
	float dotNV = saturate( dot( normal, viewDir ) );
	float r2 = roughness * roughness;
	float a = roughness < 0.25 ? -339.2 * r2 + 161.4 * roughness - 25.9 : -8.48 * r2 + 14.3 * roughness - 9.95;
	float b = roughness < 0.25 ? 44.0 * r2 - 23.7 * roughness + 3.26 : 1.97 * r2 - 3.27 * roughness + 0.72;
	float DG = exp( a * dotNV + b ) + ( roughness < 0.25 ? 0.0 : 0.1 * ( roughness - 0.25 ) );
	return saturate( DG * RECIPROCAL_PI );
}
vec2 DFGApprox( const in vec3 normal, const in vec3 viewDir, const in float roughness ) {
	float dotNV = saturate( dot( normal, viewDir ) );
	const vec4 c0 = vec4( - 1, - 0.0275, - 0.572, 0.022 );
	const vec4 c1 = vec4( 1, 0.0425, 1.04, - 0.04 );
	vec4 r = roughness * c0 + c1;
	float a004 = min( r.x * r.x, exp2( - 9.28 * dotNV ) ) * r.x + r.y;
	vec2 fab = vec2( - 1.04, 1.04 ) * a004 + r.zw;
	return fab;
}
vec3 EnvironmentBRDF( const in vec3 normal, const in vec3 viewDir, const in vec3 specularColor, const in float specularF90, const in float roughness ) {
	vec2 fab = DFGApprox( normal, viewDir, roughness );
	return specularColor * fab.x + specularF90 * fab.y;
}
#ifdef USE_IRIDESCENCE
void computeMultiscatteringIridescence( const in vec3 normal, const in vec3 viewDir, const in vec3 specularColor, const in float specularF90, const in float iridescence, const in vec3 iridescenceF0, const in float roughness, inout vec3 singleScatter, inout vec3 multiScatter ) {
#else
void computeMultiscattering( const in vec3 normal, const in vec3 viewDir, const in vec3 specularColor, const in float specularF90, const in float roughness, inout vec3 singleScatter, inout vec3 multiScatter ) {
#endif
	vec2 fab = DFGApprox( normal, viewDir, roughness );
	#ifdef USE_IRIDESCENCE
		vec3 Fr = mix( specularColor, iridescenceF0, iridescence );
	#else
		vec3 Fr = specularColor;
	#endif
	vec3 FssEss = Fr * fab.x + specularF90 * fab.y;
	float Ess = fab.x + fab.y;
	float Ems = 1.0 - Ess;
	vec3 Favg = Fr + ( 1.0 - Fr ) * 0.047619;	vec3 Fms = FssEss * Favg / ( 1.0 - Ems * Favg );
	singleScatter += FssEss;
	multiScatter += Fms * Ems;
}
#if NUM_RECT_AREA_LIGHTS > 0
	void RE_Direct_RectArea_Physical( const in RectAreaLight rectAreaLight, const in vec3 geometryPosition, const in vec3 geometryNormal, const in vec3 geometryViewDir, const in vec3 geometryClearcoatNormal, const in PhysicalMaterial material, inout ReflectedLight reflectedLight ) {
		vec3 normal = geometryNormal;
		vec3 viewDir = geometryViewDir;
		vec3 position = geometryPosition;
		vec3 lightPos = rectAreaLight.position;
		vec3 halfWidth = rectAreaLight.halfWidth;
		vec3 halfHeight = rectAreaLight.halfHeight;
		vec3 lightColor = rectAreaLight.color;
		float roughness = material.roughness;
		vec3 rectCoords[ 4 ];
		rectCoords[ 0 ] = lightPos + halfWidth - halfHeight;		rectCoords[ 1 ] = lightPos - halfWidth - halfHeight;
		rectCoords[ 2 ] = lightPos - halfWidth + halfHeight;
		rectCoords[ 3 ] = lightPos + halfWidth + halfHeight;
		vec2 uv = LTC_Uv( normal, viewDir, roughness );
		vec4 t1 = texture2D( ltc_1, uv );
		vec4 t2 = texture2D( ltc_2, uv );
		mat3 mInv = mat3(
			vec3( t1.x, 0, t1.y ),
			vec3(    0, 1,    0 ),
			vec3( t1.z, 0, t1.w )
		);
		vec3 fresnel = ( material.specularColor * t2.x + ( vec3( 1.0 ) - material.specularColor ) * t2.y );
		reflectedLight.directSpecular += lightColor * fresnel * LTC_Evaluate( normal, viewDir, position, mInv, rectCoords );
		reflectedLight.directDiffuse += lightColor * material.diffuseColor * LTC_Evaluate( normal, viewDir, position, mat3( 1.0 ), rectCoords );
	}
#endif
void RE_Direct_Physical( const in IncidentLight directLight, const in vec3 geometryPosition, const in vec3 geometryNormal, const in vec3 geometryViewDir, const in vec3 geometryClearcoatNormal, const in PhysicalMaterial material, inout ReflectedLight reflectedLight ) {
	float dotNL = saturate( dot( geometryNormal, directLight.direction ) );
	vec3 irradiance = dotNL * directLight.color;
	#ifdef USE_CLEARCOAT
		float dotNLcc = saturate( dot( geometryClearcoatNormal, directLight.direction ) );
		vec3 ccIrradiance = dotNLcc * directLight.color;
		clearcoatSpecularDirect += ccIrradiance * BRDF_GGX_Clearcoat( directLight.direction, geometryViewDir, geometryClearcoatNormal, material );
	#endif
	#ifdef USE_SHEEN
		sheenSpecularDirect += irradiance * BRDF_Sheen( directLight.direction, geometryViewDir, geometryNormal, material.sheenColor, material.sheenRoughness );
	#endif
	reflectedLight.directSpecular += irradiance * BRDF_GGX( directLight.direction, geometryViewDir, geometryNormal, material );
	reflectedLight.directDiffuse += irradiance * BRDF_Lambert( material.diffuseColor );
}
void RE_IndirectDiffuse_Physical( const in vec3 irradiance, const in vec3 geometryPosition, const in vec3 geometryNormal, const in vec3 geometryViewDir, const in vec3 geometryClearcoatNormal, const in PhysicalMaterial material, inout ReflectedLight reflectedLight ) {
	reflectedLight.indirectDiffuse += irradiance * BRDF_Lambert( material.diffuseColor );
}
void RE_IndirectSpecular_Physical( const in vec3 radiance, const in vec3 irradiance, const in vec3 clearcoatRadiance, const in vec3 geometryPosition, const in vec3 geometryNormal, const in vec3 geometryViewDir, const in vec3 geometryClearcoatNormal, const in PhysicalMaterial material, inout ReflectedLight reflectedLight) {
	#ifdef USE_CLEARCOAT
		clearcoatSpecularIndirect += clearcoatRadiance * EnvironmentBRDF( geometryClearcoatNormal, geometryViewDir, material.clearcoatF0, material.clearcoatF90, material.clearcoatRoughness );
	#endif
	#ifdef USE_SHEEN
		sheenSpecularIndirect += irradiance * material.sheenColor * IBLSheenBRDF( geometryNormal, geometryViewDir, material.sheenRoughness );
	#endif
	vec3 singleScattering = vec3( 0.0 );
	vec3 multiScattering = vec3( 0.0 );
	vec3 cosineWeightedIrradiance = irradiance * RECIPROCAL_PI;
	#ifdef USE_IRIDESCENCE
		computeMultiscatteringIridescence( geometryNormal, geometryViewDir, material.specularColor, material.specularF90, material.iridescence, material.iridescenceFresnel, material.roughness, singleScattering, multiScattering );
	#else
		computeMultiscattering( geometryNormal, geometryViewDir, material.specularColor, material.specularF90, material.roughness, singleScattering, multiScattering );
	#endif
	vec3 totalScattering = singleScattering + multiScattering;
	vec3 diffuse = material.diffuseColor * ( 1.0 - max( max( totalScattering.r, totalScattering.g ), totalScattering.b ) );
	reflectedLight.indirectSpecular += radiance * singleScattering;
	reflectedLight.indirectSpecular += multiScattering * cosineWeightedIrradiance;
	reflectedLight.indirectDiffuse += diffuse * cosineWeightedIrradiance;
}
#define RE_Direct				RE_Direct_Physical
#define RE_Direct_RectArea		RE_Direct_RectArea_Physical
#define RE_IndirectDiffuse		RE_IndirectDiffuse_Physical
#define RE_IndirectSpecular		RE_IndirectSpecular_Physical
float computeSpecularOcclusion( const in float dotNV, const in float ambientOcclusion, const in float roughness ) {
	return saturate( pow( dotNV + ambientOcclusion, exp2( - 16.0 * roughness - 1.0 ) ) - 1.0 + ambientOcclusion );
}`,Kg=`
vec3 geometryPosition = - vViewPosition;
vec3 geometryNormal = normal;
vec3 geometryViewDir = ( isOrthographic ) ? vec3( 0, 0, 1 ) : normalize( vViewPosition );
vec3 geometryClearcoatNormal = vec3( 0.0 );
#ifdef USE_CLEARCOAT
	geometryClearcoatNormal = clearcoatNormal;
#endif
#ifdef USE_IRIDESCENCE
	float dotNVi = saturate( dot( normal, geometryViewDir ) );
	if ( material.iridescenceThickness == 0.0 ) {
		material.iridescence = 0.0;
	} else {
		material.iridescence = saturate( material.iridescence );
	}
	if ( material.iridescence > 0.0 ) {
		material.iridescenceFresnel = evalIridescence( 1.0, material.iridescenceIOR, dotNVi, material.iridescenceThickness, material.specularColor );
		material.iridescenceF0 = Schlick_to_F0( material.iridescenceFresnel, 1.0, dotNVi );
	}
#endif
IncidentLight directLight;
#if ( NUM_POINT_LIGHTS > 0 ) && defined( RE_Direct )
	PointLight pointLight;
	#if defined( USE_SHADOWMAP ) && NUM_POINT_LIGHT_SHADOWS > 0
	PointLightShadow pointLightShadow;
	#endif
	#pragma unroll_loop_start
	for ( int i = 0; i < NUM_POINT_LIGHTS; i ++ ) {
		pointLight = pointLights[ i ];
		getPointLightInfo( pointLight, geometryPosition, directLight );
		#if defined( USE_SHADOWMAP ) && ( UNROLLED_LOOP_INDEX < NUM_POINT_LIGHT_SHADOWS )
		pointLightShadow = pointLightShadows[ i ];
		directLight.color *= ( directLight.visible && receiveShadow ) ? getPointShadow( pointShadowMap[ i ], pointLightShadow.shadowMapSize, pointLightShadow.shadowBias, pointLightShadow.shadowRadius, vPointShadowCoord[ i ], pointLightShadow.shadowCameraNear, pointLightShadow.shadowCameraFar ) : 1.0;
		#endif
		RE_Direct( directLight, geometryPosition, geometryNormal, geometryViewDir, geometryClearcoatNormal, material, reflectedLight );
	}
	#pragma unroll_loop_end
#endif
#if ( NUM_SPOT_LIGHTS > 0 ) && defined( RE_Direct )
	SpotLight spotLight;
	vec4 spotColor;
	vec3 spotLightCoord;
	bool inSpotLightMap;
	#if defined( USE_SHADOWMAP ) && NUM_SPOT_LIGHT_SHADOWS > 0
	SpotLightShadow spotLightShadow;
	#endif
	#pragma unroll_loop_start
	for ( int i = 0; i < NUM_SPOT_LIGHTS; i ++ ) {
		spotLight = spotLights[ i ];
		getSpotLightInfo( spotLight, geometryPosition, directLight );
		#if ( UNROLLED_LOOP_INDEX < NUM_SPOT_LIGHT_SHADOWS_WITH_MAPS )
		#define SPOT_LIGHT_MAP_INDEX UNROLLED_LOOP_INDEX
		#elif ( UNROLLED_LOOP_INDEX < NUM_SPOT_LIGHT_SHADOWS )
		#define SPOT_LIGHT_MAP_INDEX NUM_SPOT_LIGHT_MAPS
		#else
		#define SPOT_LIGHT_MAP_INDEX ( UNROLLED_LOOP_INDEX - NUM_SPOT_LIGHT_SHADOWS + NUM_SPOT_LIGHT_SHADOWS_WITH_MAPS )
		#endif
		#if ( SPOT_LIGHT_MAP_INDEX < NUM_SPOT_LIGHT_MAPS )
			spotLightCoord = vSpotLightCoord[ i ].xyz / vSpotLightCoord[ i ].w;
			inSpotLightMap = all( lessThan( abs( spotLightCoord * 2. - 1. ), vec3( 1.0 ) ) );
			spotColor = texture2D( spotLightMap[ SPOT_LIGHT_MAP_INDEX ], spotLightCoord.xy );
			directLight.color = inSpotLightMap ? directLight.color * spotColor.rgb : directLight.color;
		#endif
		#undef SPOT_LIGHT_MAP_INDEX
		#if defined( USE_SHADOWMAP ) && ( UNROLLED_LOOP_INDEX < NUM_SPOT_LIGHT_SHADOWS )
		spotLightShadow = spotLightShadows[ i ];
		directLight.color *= ( directLight.visible && receiveShadow ) ? getShadow( spotShadowMap[ i ], spotLightShadow.shadowMapSize, spotLightShadow.shadowBias, spotLightShadow.shadowRadius, vSpotLightCoord[ i ] ) : 1.0;
		#endif
		RE_Direct( directLight, geometryPosition, geometryNormal, geometryViewDir, geometryClearcoatNormal, material, reflectedLight );
	}
	#pragma unroll_loop_end
#endif
#if ( NUM_DIR_LIGHTS > 0 ) && defined( RE_Direct )
	DirectionalLight directionalLight;
	#if defined( USE_SHADOWMAP ) && NUM_DIR_LIGHT_SHADOWS > 0
	DirectionalLightShadow directionalLightShadow;
	#endif
	#pragma unroll_loop_start
	for ( int i = 0; i < NUM_DIR_LIGHTS; i ++ ) {
		directionalLight = directionalLights[ i ];
		getDirectionalLightInfo( directionalLight, directLight );
		#if defined( USE_SHADOWMAP ) && ( UNROLLED_LOOP_INDEX < NUM_DIR_LIGHT_SHADOWS )
		directionalLightShadow = directionalLightShadows[ i ];
		directLight.color *= ( directLight.visible && receiveShadow ) ? getShadow( directionalShadowMap[ i ], directionalLightShadow.shadowMapSize, directionalLightShadow.shadowBias, directionalLightShadow.shadowRadius, vDirectionalShadowCoord[ i ] ) : 1.0;
		#endif
		RE_Direct( directLight, geometryPosition, geometryNormal, geometryViewDir, geometryClearcoatNormal, material, reflectedLight );
	}
	#pragma unroll_loop_end
#endif
#if ( NUM_RECT_AREA_LIGHTS > 0 ) && defined( RE_Direct_RectArea )
	RectAreaLight rectAreaLight;
	#pragma unroll_loop_start
	for ( int i = 0; i < NUM_RECT_AREA_LIGHTS; i ++ ) {
		rectAreaLight = rectAreaLights[ i ];
		RE_Direct_RectArea( rectAreaLight, geometryPosition, geometryNormal, geometryViewDir, geometryClearcoatNormal, material, reflectedLight );
	}
	#pragma unroll_loop_end
#endif
#if defined( RE_IndirectDiffuse )
	vec3 iblIrradiance = vec3( 0.0 );
	vec3 irradiance = getAmbientLightIrradiance( ambientLightColor );
	#if defined( USE_LIGHT_PROBES )
		irradiance += getLightProbeIrradiance( lightProbe, geometryNormal );
	#endif
	#if ( NUM_HEMI_LIGHTS > 0 )
		#pragma unroll_loop_start
		for ( int i = 0; i < NUM_HEMI_LIGHTS; i ++ ) {
			irradiance += getHemisphereLightIrradiance( hemisphereLights[ i ], geometryNormal );
		}
		#pragma unroll_loop_end
	#endif
#endif
#if defined( RE_IndirectSpecular )
	vec3 radiance = vec3( 0.0 );
	vec3 clearcoatRadiance = vec3( 0.0 );
#endif`,Jg=`#if defined( RE_IndirectDiffuse )
	#ifdef USE_LIGHTMAP
		vec4 lightMapTexel = texture2D( lightMap, vLightMapUv );
		vec3 lightMapIrradiance = lightMapTexel.rgb * lightMapIntensity;
		irradiance += lightMapIrradiance;
	#endif
	#if defined( USE_ENVMAP ) && defined( STANDARD ) && defined( ENVMAP_TYPE_CUBE_UV )
		iblIrradiance += getIBLIrradiance( geometryNormal );
	#endif
#endif
#if defined( USE_ENVMAP ) && defined( RE_IndirectSpecular )
	#ifdef USE_ANISOTROPY
		radiance += getIBLAnisotropyRadiance( geometryViewDir, geometryNormal, material.roughness, material.anisotropyB, material.anisotropy );
	#else
		radiance += getIBLRadiance( geometryViewDir, geometryNormal, material.roughness );
	#endif
	#ifdef USE_CLEARCOAT
		clearcoatRadiance += getIBLRadiance( geometryViewDir, geometryClearcoatNormal, material.clearcoatRoughness );
	#endif
#endif`,Qg=`#if defined( RE_IndirectDiffuse )
	RE_IndirectDiffuse( irradiance, geometryPosition, geometryNormal, geometryViewDir, geometryClearcoatNormal, material, reflectedLight );
#endif
#if defined( RE_IndirectSpecular )
	RE_IndirectSpecular( radiance, iblIrradiance, clearcoatRadiance, geometryPosition, geometryNormal, geometryViewDir, geometryClearcoatNormal, material, reflectedLight );
#endif`,t0=`#if defined( USE_LOGDEPTHBUF ) && defined( USE_LOGDEPTHBUF_EXT )
	gl_FragDepthEXT = vIsPerspective == 0.0 ? gl_FragCoord.z : log2( vFragDepth ) * logDepthBufFC * 0.5;
#endif`,e0=`#if defined( USE_LOGDEPTHBUF ) && defined( USE_LOGDEPTHBUF_EXT )
	uniform float logDepthBufFC;
	varying float vFragDepth;
	varying float vIsPerspective;
#endif`,n0=`#ifdef USE_LOGDEPTHBUF
	#ifdef USE_LOGDEPTHBUF_EXT
		varying float vFragDepth;
		varying float vIsPerspective;
	#else
		uniform float logDepthBufFC;
	#endif
#endif`,i0=`#ifdef USE_LOGDEPTHBUF
	#ifdef USE_LOGDEPTHBUF_EXT
		vFragDepth = 1.0 + gl_Position.w;
		vIsPerspective = float( isPerspectiveMatrix( projectionMatrix ) );
	#else
		if ( isPerspectiveMatrix( projectionMatrix ) ) {
			gl_Position.z = log2( max( EPSILON, gl_Position.w + 1.0 ) ) * logDepthBufFC - 1.0;
			gl_Position.z *= gl_Position.w;
		}
	#endif
#endif`,s0=`#ifdef USE_MAP
	vec4 sampledDiffuseColor = texture2D( map, vMapUv );
	#ifdef DECODE_VIDEO_TEXTURE
		sampledDiffuseColor = vec4( mix( pow( sampledDiffuseColor.rgb * 0.9478672986 + vec3( 0.0521327014 ), vec3( 2.4 ) ), sampledDiffuseColor.rgb * 0.0773993808, vec3( lessThanEqual( sampledDiffuseColor.rgb, vec3( 0.04045 ) ) ) ), sampledDiffuseColor.w );
	
	#endif
	diffuseColor *= sampledDiffuseColor;
#endif`,r0=`#ifdef USE_MAP
	uniform sampler2D map;
#endif`,a0=`#if defined( USE_MAP ) || defined( USE_ALPHAMAP )
	#if defined( USE_POINTS_UV )
		vec2 uv = vUv;
	#else
		vec2 uv = ( uvTransform * vec3( gl_PointCoord.x, 1.0 - gl_PointCoord.y, 1 ) ).xy;
	#endif
#endif
#ifdef USE_MAP
	diffuseColor *= texture2D( map, uv );
#endif
#ifdef USE_ALPHAMAP
	diffuseColor.a *= texture2D( alphaMap, uv ).g;
#endif`,o0=`#if defined( USE_POINTS_UV )
	varying vec2 vUv;
#else
	#if defined( USE_MAP ) || defined( USE_ALPHAMAP )
		uniform mat3 uvTransform;
	#endif
#endif
#ifdef USE_MAP
	uniform sampler2D map;
#endif
#ifdef USE_ALPHAMAP
	uniform sampler2D alphaMap;
#endif`,c0=`float metalnessFactor = metalness;
#ifdef USE_METALNESSMAP
	vec4 texelMetalness = texture2D( metalnessMap, vMetalnessMapUv );
	metalnessFactor *= texelMetalness.b;
#endif`,l0=`#ifdef USE_METALNESSMAP
	uniform sampler2D metalnessMap;
#endif`,h0=`#if defined( USE_MORPHCOLORS ) && defined( MORPHTARGETS_TEXTURE )
	vColor *= morphTargetBaseInfluence;
	for ( int i = 0; i < MORPHTARGETS_COUNT; i ++ ) {
		#if defined( USE_COLOR_ALPHA )
			if ( morphTargetInfluences[ i ] != 0.0 ) vColor += getMorph( gl_VertexID, i, 2 ) * morphTargetInfluences[ i ];
		#elif defined( USE_COLOR )
			if ( morphTargetInfluences[ i ] != 0.0 ) vColor += getMorph( gl_VertexID, i, 2 ).rgb * morphTargetInfluences[ i ];
		#endif
	}
#endif`,u0=`#ifdef USE_MORPHNORMALS
	objectNormal *= morphTargetBaseInfluence;
	#ifdef MORPHTARGETS_TEXTURE
		for ( int i = 0; i < MORPHTARGETS_COUNT; i ++ ) {
			if ( morphTargetInfluences[ i ] != 0.0 ) objectNormal += getMorph( gl_VertexID, i, 1 ).xyz * morphTargetInfluences[ i ];
		}
	#else
		objectNormal += morphNormal0 * morphTargetInfluences[ 0 ];
		objectNormal += morphNormal1 * morphTargetInfluences[ 1 ];
		objectNormal += morphNormal2 * morphTargetInfluences[ 2 ];
		objectNormal += morphNormal3 * morphTargetInfluences[ 3 ];
	#endif
#endif`,d0=`#ifdef USE_MORPHTARGETS
	uniform float morphTargetBaseInfluence;
	#ifdef MORPHTARGETS_TEXTURE
		uniform float morphTargetInfluences[ MORPHTARGETS_COUNT ];
		uniform sampler2DArray morphTargetsTexture;
		uniform ivec2 morphTargetsTextureSize;
		vec4 getMorph( const in int vertexIndex, const in int morphTargetIndex, const in int offset ) {
			int texelIndex = vertexIndex * MORPHTARGETS_TEXTURE_STRIDE + offset;
			int y = texelIndex / morphTargetsTextureSize.x;
			int x = texelIndex - y * morphTargetsTextureSize.x;
			ivec3 morphUV = ivec3( x, y, morphTargetIndex );
			return texelFetch( morphTargetsTexture, morphUV, 0 );
		}
	#else
		#ifndef USE_MORPHNORMALS
			uniform float morphTargetInfluences[ 8 ];
		#else
			uniform float morphTargetInfluences[ 4 ];
		#endif
	#endif
#endif`,f0=`#ifdef USE_MORPHTARGETS
	transformed *= morphTargetBaseInfluence;
	#ifdef MORPHTARGETS_TEXTURE
		for ( int i = 0; i < MORPHTARGETS_COUNT; i ++ ) {
			if ( morphTargetInfluences[ i ] != 0.0 ) transformed += getMorph( gl_VertexID, i, 0 ).xyz * morphTargetInfluences[ i ];
		}
	#else
		transformed += morphTarget0 * morphTargetInfluences[ 0 ];
		transformed += morphTarget1 * morphTargetInfluences[ 1 ];
		transformed += morphTarget2 * morphTargetInfluences[ 2 ];
		transformed += morphTarget3 * morphTargetInfluences[ 3 ];
		#ifndef USE_MORPHNORMALS
			transformed += morphTarget4 * morphTargetInfluences[ 4 ];
			transformed += morphTarget5 * morphTargetInfluences[ 5 ];
			transformed += morphTarget6 * morphTargetInfluences[ 6 ];
			transformed += morphTarget7 * morphTargetInfluences[ 7 ];
		#endif
	#endif
#endif`,p0=`float faceDirection = gl_FrontFacing ? 1.0 : - 1.0;
#ifdef FLAT_SHADED
	vec3 fdx = dFdx( vViewPosition );
	vec3 fdy = dFdy( vViewPosition );
	vec3 normal = normalize( cross( fdx, fdy ) );
#else
	vec3 normal = normalize( vNormal );
	#ifdef DOUBLE_SIDED
		normal *= faceDirection;
	#endif
#endif
#if defined( USE_NORMALMAP_TANGENTSPACE ) || defined( USE_CLEARCOAT_NORMALMAP ) || defined( USE_ANISOTROPY )
	#ifdef USE_TANGENT
		mat3 tbn = mat3( normalize( vTangent ), normalize( vBitangent ), normal );
	#else
		mat3 tbn = getTangentFrame( - vViewPosition, normal,
		#if defined( USE_NORMALMAP )
			vNormalMapUv
		#elif defined( USE_CLEARCOAT_NORMALMAP )
			vClearcoatNormalMapUv
		#else
			vUv
		#endif
		);
	#endif
	#if defined( DOUBLE_SIDED ) && ! defined( FLAT_SHADED )
		tbn[0] *= faceDirection;
		tbn[1] *= faceDirection;
	#endif
#endif
#ifdef USE_CLEARCOAT_NORMALMAP
	#ifdef USE_TANGENT
		mat3 tbn2 = mat3( normalize( vTangent ), normalize( vBitangent ), normal );
	#else
		mat3 tbn2 = getTangentFrame( - vViewPosition, normal, vClearcoatNormalMapUv );
	#endif
	#if defined( DOUBLE_SIDED ) && ! defined( FLAT_SHADED )
		tbn2[0] *= faceDirection;
		tbn2[1] *= faceDirection;
	#endif
#endif
vec3 nonPerturbedNormal = normal;`,m0=`#ifdef USE_NORMALMAP_OBJECTSPACE
	normal = texture2D( normalMap, vNormalMapUv ).xyz * 2.0 - 1.0;
	#ifdef FLIP_SIDED
		normal = - normal;
	#endif
	#ifdef DOUBLE_SIDED
		normal = normal * faceDirection;
	#endif
	normal = normalize( normalMatrix * normal );
#elif defined( USE_NORMALMAP_TANGENTSPACE )
	vec3 mapN = texture2D( normalMap, vNormalMapUv ).xyz * 2.0 - 1.0;
	mapN.xy *= normalScale;
	normal = normalize( tbn * mapN );
#elif defined( USE_BUMPMAP )
	normal = perturbNormalArb( - vViewPosition, normal, dHdxy_fwd(), faceDirection );
#endif`,g0=`#ifndef FLAT_SHADED
	varying vec3 vNormal;
	#ifdef USE_TANGENT
		varying vec3 vTangent;
		varying vec3 vBitangent;
	#endif
#endif`,x0=`#ifndef FLAT_SHADED
	varying vec3 vNormal;
	#ifdef USE_TANGENT
		varying vec3 vTangent;
		varying vec3 vBitangent;
	#endif
#endif`,_0=`#ifndef FLAT_SHADED
	vNormal = normalize( transformedNormal );
	#ifdef USE_TANGENT
		vTangent = normalize( transformedTangent );
		vBitangent = normalize( cross( vNormal, vTangent ) * tangent.w );
	#endif
#endif`,v0=`#ifdef USE_NORMALMAP
	uniform sampler2D normalMap;
	uniform vec2 normalScale;
#endif
#ifdef USE_NORMALMAP_OBJECTSPACE
	uniform mat3 normalMatrix;
#endif
#if ! defined ( USE_TANGENT ) && ( defined ( USE_NORMALMAP_TANGENTSPACE ) || defined ( USE_CLEARCOAT_NORMALMAP ) || defined( USE_ANISOTROPY ) )
	mat3 getTangentFrame( vec3 eye_pos, vec3 surf_norm, vec2 uv ) {
		vec3 q0 = dFdx( eye_pos.xyz );
		vec3 q1 = dFdy( eye_pos.xyz );
		vec2 st0 = dFdx( uv.st );
		vec2 st1 = dFdy( uv.st );
		vec3 N = surf_norm;
		vec3 q1perp = cross( q1, N );
		vec3 q0perp = cross( N, q0 );
		vec3 T = q1perp * st0.x + q0perp * st1.x;
		vec3 B = q1perp * st0.y + q0perp * st1.y;
		float det = max( dot( T, T ), dot( B, B ) );
		float scale = ( det == 0.0 ) ? 0.0 : inversesqrt( det );
		return mat3( T * scale, B * scale, N );
	}
#endif`,S0=`#ifdef USE_CLEARCOAT
	vec3 clearcoatNormal = nonPerturbedNormal;
#endif`,M0=`#ifdef USE_CLEARCOAT_NORMALMAP
	vec3 clearcoatMapN = texture2D( clearcoatNormalMap, vClearcoatNormalMapUv ).xyz * 2.0 - 1.0;
	clearcoatMapN.xy *= clearcoatNormalScale;
	clearcoatNormal = normalize( tbn2 * clearcoatMapN );
#endif`,y0=`#ifdef USE_CLEARCOATMAP
	uniform sampler2D clearcoatMap;
#endif
#ifdef USE_CLEARCOAT_NORMALMAP
	uniform sampler2D clearcoatNormalMap;
	uniform vec2 clearcoatNormalScale;
#endif
#ifdef USE_CLEARCOAT_ROUGHNESSMAP
	uniform sampler2D clearcoatRoughnessMap;
#endif`,E0=`#ifdef USE_IRIDESCENCEMAP
	uniform sampler2D iridescenceMap;
#endif
#ifdef USE_IRIDESCENCE_THICKNESSMAP
	uniform sampler2D iridescenceThicknessMap;
#endif`,w0=`#ifdef OPAQUE
diffuseColor.a = 1.0;
#endif
#ifdef USE_TRANSMISSION
diffuseColor.a *= material.transmissionAlpha;
#endif
gl_FragColor = vec4( outgoingLight, diffuseColor.a );`,T0=`vec3 packNormalToRGB( const in vec3 normal ) {
	return normalize( normal ) * 0.5 + 0.5;
}
vec3 unpackRGBToNormal( const in vec3 rgb ) {
	return 2.0 * rgb.xyz - 1.0;
}
const float PackUpscale = 256. / 255.;const float UnpackDownscale = 255. / 256.;
const vec3 PackFactors = vec3( 256. * 256. * 256., 256. * 256., 256. );
const vec4 UnpackFactors = UnpackDownscale / vec4( PackFactors, 1. );
const float ShiftRight8 = 1. / 256.;
vec4 packDepthToRGBA( const in float v ) {
	vec4 r = vec4( fract( v * PackFactors ), v );
	r.yzw -= r.xyz * ShiftRight8;	return r * PackUpscale;
}
float unpackRGBAToDepth( const in vec4 v ) {
	return dot( v, UnpackFactors );
}
vec2 packDepthToRG( in highp float v ) {
	return packDepthToRGBA( v ).yx;
}
float unpackRGToDepth( const in highp vec2 v ) {
	return unpackRGBAToDepth( vec4( v.xy, 0.0, 0.0 ) );
}
vec4 pack2HalfToRGBA( vec2 v ) {
	vec4 r = vec4( v.x, fract( v.x * 255.0 ), v.y, fract( v.y * 255.0 ) );
	return vec4( r.x - r.y / 255.0, r.y, r.z - r.w / 255.0, r.w );
}
vec2 unpackRGBATo2Half( vec4 v ) {
	return vec2( v.x + ( v.y / 255.0 ), v.z + ( v.w / 255.0 ) );
}
float viewZToOrthographicDepth( const in float viewZ, const in float near, const in float far ) {
	return ( viewZ + near ) / ( near - far );
}
float orthographicDepthToViewZ( const in float depth, const in float near, const in float far ) {
	return depth * ( near - far ) - near;
}
float viewZToPerspectiveDepth( const in float viewZ, const in float near, const in float far ) {
	return ( ( near + viewZ ) * far ) / ( ( far - near ) * viewZ );
}
float perspectiveDepthToViewZ( const in float depth, const in float near, const in float far ) {
	return ( near * far ) / ( ( far - near ) * depth - far );
}`,A0=`#ifdef PREMULTIPLIED_ALPHA
	gl_FragColor.rgb *= gl_FragColor.a;
#endif`,C0=`vec4 mvPosition = vec4( transformed, 1.0 );
#ifdef USE_BATCHING
	mvPosition = batchingMatrix * mvPosition;
#endif
#ifdef USE_INSTANCING
	mvPosition = instanceMatrix * mvPosition;
#endif
mvPosition = modelViewMatrix * mvPosition;
gl_Position = projectionMatrix * mvPosition;`,R0=`#ifdef DITHERING
	gl_FragColor.rgb = dithering( gl_FragColor.rgb );
#endif`,b0=`#ifdef DITHERING
	vec3 dithering( vec3 color ) {
		float grid_position = rand( gl_FragCoord.xy );
		vec3 dither_shift_RGB = vec3( 0.25 / 255.0, -0.25 / 255.0, 0.25 / 255.0 );
		dither_shift_RGB = mix( 2.0 * dither_shift_RGB, -2.0 * dither_shift_RGB, grid_position );
		return color + dither_shift_RGB;
	}
#endif`,P0=`float roughnessFactor = roughness;
#ifdef USE_ROUGHNESSMAP
	vec4 texelRoughness = texture2D( roughnessMap, vRoughnessMapUv );
	roughnessFactor *= texelRoughness.g;
#endif`,L0=`#ifdef USE_ROUGHNESSMAP
	uniform sampler2D roughnessMap;
#endif`,D0=`#if NUM_SPOT_LIGHT_COORDS > 0
	varying vec4 vSpotLightCoord[ NUM_SPOT_LIGHT_COORDS ];
#endif
#if NUM_SPOT_LIGHT_MAPS > 0
	uniform sampler2D spotLightMap[ NUM_SPOT_LIGHT_MAPS ];
#endif
#ifdef USE_SHADOWMAP
	#if NUM_DIR_LIGHT_SHADOWS > 0
		uniform sampler2D directionalShadowMap[ NUM_DIR_LIGHT_SHADOWS ];
		varying vec4 vDirectionalShadowCoord[ NUM_DIR_LIGHT_SHADOWS ];
		struct DirectionalLightShadow {
			float shadowBias;
			float shadowNormalBias;
			float shadowRadius;
			vec2 shadowMapSize;
		};
		uniform DirectionalLightShadow directionalLightShadows[ NUM_DIR_LIGHT_SHADOWS ];
	#endif
	#if NUM_SPOT_LIGHT_SHADOWS > 0
		uniform sampler2D spotShadowMap[ NUM_SPOT_LIGHT_SHADOWS ];
		struct SpotLightShadow {
			float shadowBias;
			float shadowNormalBias;
			float shadowRadius;
			vec2 shadowMapSize;
		};
		uniform SpotLightShadow spotLightShadows[ NUM_SPOT_LIGHT_SHADOWS ];
	#endif
	#if NUM_POINT_LIGHT_SHADOWS > 0
		uniform sampler2D pointShadowMap[ NUM_POINT_LIGHT_SHADOWS ];
		varying vec4 vPointShadowCoord[ NUM_POINT_LIGHT_SHADOWS ];
		struct PointLightShadow {
			float shadowBias;
			float shadowNormalBias;
			float shadowRadius;
			vec2 shadowMapSize;
			float shadowCameraNear;
			float shadowCameraFar;
		};
		uniform PointLightShadow pointLightShadows[ NUM_POINT_LIGHT_SHADOWS ];
	#endif
	float texture2DCompare( sampler2D depths, vec2 uv, float compare ) {
		return step( compare, unpackRGBAToDepth( texture2D( depths, uv ) ) );
	}
	vec2 texture2DDistribution( sampler2D shadow, vec2 uv ) {
		return unpackRGBATo2Half( texture2D( shadow, uv ) );
	}
	float VSMShadow (sampler2D shadow, vec2 uv, float compare ){
		float occlusion = 1.0;
		vec2 distribution = texture2DDistribution( shadow, uv );
		float hard_shadow = step( compare , distribution.x );
		if (hard_shadow != 1.0 ) {
			float distance = compare - distribution.x ;
			float variance = max( 0.00000, distribution.y * distribution.y );
			float softness_probability = variance / (variance + distance * distance );			softness_probability = clamp( ( softness_probability - 0.3 ) / ( 0.95 - 0.3 ), 0.0, 1.0 );			occlusion = clamp( max( hard_shadow, softness_probability ), 0.0, 1.0 );
		}
		return occlusion;
	}
	float getShadow( sampler2D shadowMap, vec2 shadowMapSize, float shadowBias, float shadowRadius, vec4 shadowCoord ) {
		float shadow = 1.0;
		shadowCoord.xyz /= shadowCoord.w;
		shadowCoord.z += shadowBias;
		bool inFrustum = shadowCoord.x >= 0.0 && shadowCoord.x <= 1.0 && shadowCoord.y >= 0.0 && shadowCoord.y <= 1.0;
		bool frustumTest = inFrustum && shadowCoord.z <= 1.0;
		if ( frustumTest ) {
		#if defined( SHADOWMAP_TYPE_PCF )
			vec2 texelSize = vec2( 1.0 ) / shadowMapSize;
			float dx0 = - texelSize.x * shadowRadius;
			float dy0 = - texelSize.y * shadowRadius;
			float dx1 = + texelSize.x * shadowRadius;
			float dy1 = + texelSize.y * shadowRadius;
			float dx2 = dx0 / 2.0;
			float dy2 = dy0 / 2.0;
			float dx3 = dx1 / 2.0;
			float dy3 = dy1 / 2.0;
			shadow = (
				texture2DCompare( shadowMap, shadowCoord.xy + vec2( dx0, dy0 ), shadowCoord.z ) +
				texture2DCompare( shadowMap, shadowCoord.xy + vec2( 0.0, dy0 ), shadowCoord.z ) +
				texture2DCompare( shadowMap, shadowCoord.xy + vec2( dx1, dy0 ), shadowCoord.z ) +
				texture2DCompare( shadowMap, shadowCoord.xy + vec2( dx2, dy2 ), shadowCoord.z ) +
				texture2DCompare( shadowMap, shadowCoord.xy + vec2( 0.0, dy2 ), shadowCoord.z ) +
				texture2DCompare( shadowMap, shadowCoord.xy + vec2( dx3, dy2 ), shadowCoord.z ) +
				texture2DCompare( shadowMap, shadowCoord.xy + vec2( dx0, 0.0 ), shadowCoord.z ) +
				texture2DCompare( shadowMap, shadowCoord.xy + vec2( dx2, 0.0 ), shadowCoord.z ) +
				texture2DCompare( shadowMap, shadowCoord.xy, shadowCoord.z ) +
				texture2DCompare( shadowMap, shadowCoord.xy + vec2( dx3, 0.0 ), shadowCoord.z ) +
				texture2DCompare( shadowMap, shadowCoord.xy + vec2( dx1, 0.0 ), shadowCoord.z ) +
				texture2DCompare( shadowMap, shadowCoord.xy + vec2( dx2, dy3 ), shadowCoord.z ) +
				texture2DCompare( shadowMap, shadowCoord.xy + vec2( 0.0, dy3 ), shadowCoord.z ) +
				texture2DCompare( shadowMap, shadowCoord.xy + vec2( dx3, dy3 ), shadowCoord.z ) +
				texture2DCompare( shadowMap, shadowCoord.xy + vec2( dx0, dy1 ), shadowCoord.z ) +
				texture2DCompare( shadowMap, shadowCoord.xy + vec2( 0.0, dy1 ), shadowCoord.z ) +
				texture2DCompare( shadowMap, shadowCoord.xy + vec2( dx1, dy1 ), shadowCoord.z )
			) * ( 1.0 / 17.0 );
		#elif defined( SHADOWMAP_TYPE_PCF_SOFT )
			vec2 texelSize = vec2( 1.0 ) / shadowMapSize;
			float dx = texelSize.x;
			float dy = texelSize.y;
			vec2 uv = shadowCoord.xy;
			vec2 f = fract( uv * shadowMapSize + 0.5 );
			uv -= f * texelSize;
			shadow = (
				texture2DCompare( shadowMap, uv, shadowCoord.z ) +
				texture2DCompare( shadowMap, uv + vec2( dx, 0.0 ), shadowCoord.z ) +
				texture2DCompare( shadowMap, uv + vec2( 0.0, dy ), shadowCoord.z ) +
				texture2DCompare( shadowMap, uv + texelSize, shadowCoord.z ) +
				mix( texture2DCompare( shadowMap, uv + vec2( -dx, 0.0 ), shadowCoord.z ),
					 texture2DCompare( shadowMap, uv + vec2( 2.0 * dx, 0.0 ), shadowCoord.z ),
					 f.x ) +
				mix( texture2DCompare( shadowMap, uv + vec2( -dx, dy ), shadowCoord.z ),
					 texture2DCompare( shadowMap, uv + vec2( 2.0 * dx, dy ), shadowCoord.z ),
					 f.x ) +
				mix( texture2DCompare( shadowMap, uv + vec2( 0.0, -dy ), shadowCoord.z ),
					 texture2DCompare( shadowMap, uv + vec2( 0.0, 2.0 * dy ), shadowCoord.z ),
					 f.y ) +
				mix( texture2DCompare( shadowMap, uv + vec2( dx, -dy ), shadowCoord.z ),
					 texture2DCompare( shadowMap, uv + vec2( dx, 2.0 * dy ), shadowCoord.z ),
					 f.y ) +
				mix( mix( texture2DCompare( shadowMap, uv + vec2( -dx, -dy ), shadowCoord.z ),
						  texture2DCompare( shadowMap, uv + vec2( 2.0 * dx, -dy ), shadowCoord.z ),
						  f.x ),
					 mix( texture2DCompare( shadowMap, uv + vec2( -dx, 2.0 * dy ), shadowCoord.z ),
						  texture2DCompare( shadowMap, uv + vec2( 2.0 * dx, 2.0 * dy ), shadowCoord.z ),
						  f.x ),
					 f.y )
			) * ( 1.0 / 9.0 );
		#elif defined( SHADOWMAP_TYPE_VSM )
			shadow = VSMShadow( shadowMap, shadowCoord.xy, shadowCoord.z );
		#else
			shadow = texture2DCompare( shadowMap, shadowCoord.xy, shadowCoord.z );
		#endif
		}
		return shadow;
	}
	vec2 cubeToUV( vec3 v, float texelSizeY ) {
		vec3 absV = abs( v );
		float scaleToCube = 1.0 / max( absV.x, max( absV.y, absV.z ) );
		absV *= scaleToCube;
		v *= scaleToCube * ( 1.0 - 2.0 * texelSizeY );
		vec2 planar = v.xy;
		float almostATexel = 1.5 * texelSizeY;
		float almostOne = 1.0 - almostATexel;
		if ( absV.z >= almostOne ) {
			if ( v.z > 0.0 )
				planar.x = 4.0 - v.x;
		} else if ( absV.x >= almostOne ) {
			float signX = sign( v.x );
			planar.x = v.z * signX + 2.0 * signX;
		} else if ( absV.y >= almostOne ) {
			float signY = sign( v.y );
			planar.x = v.x + 2.0 * signY + 2.0;
			planar.y = v.z * signY - 2.0;
		}
		return vec2( 0.125, 0.25 ) * planar + vec2( 0.375, 0.75 );
	}
	float getPointShadow( sampler2D shadowMap, vec2 shadowMapSize, float shadowBias, float shadowRadius, vec4 shadowCoord, float shadowCameraNear, float shadowCameraFar ) {
		vec2 texelSize = vec2( 1.0 ) / ( shadowMapSize * vec2( 4.0, 2.0 ) );
		vec3 lightToPosition = shadowCoord.xyz;
		float dp = ( length( lightToPosition ) - shadowCameraNear ) / ( shadowCameraFar - shadowCameraNear );		dp += shadowBias;
		vec3 bd3D = normalize( lightToPosition );
		#if defined( SHADOWMAP_TYPE_PCF ) || defined( SHADOWMAP_TYPE_PCF_SOFT ) || defined( SHADOWMAP_TYPE_VSM )
			vec2 offset = vec2( - 1, 1 ) * shadowRadius * texelSize.y;
			return (
				texture2DCompare( shadowMap, cubeToUV( bd3D + offset.xyy, texelSize.y ), dp ) +
				texture2DCompare( shadowMap, cubeToUV( bd3D + offset.yyy, texelSize.y ), dp ) +
				texture2DCompare( shadowMap, cubeToUV( bd3D + offset.xyx, texelSize.y ), dp ) +
				texture2DCompare( shadowMap, cubeToUV( bd3D + offset.yyx, texelSize.y ), dp ) +
				texture2DCompare( shadowMap, cubeToUV( bd3D, texelSize.y ), dp ) +
				texture2DCompare( shadowMap, cubeToUV( bd3D + offset.xxy, texelSize.y ), dp ) +
				texture2DCompare( shadowMap, cubeToUV( bd3D + offset.yxy, texelSize.y ), dp ) +
				texture2DCompare( shadowMap, cubeToUV( bd3D + offset.xxx, texelSize.y ), dp ) +
				texture2DCompare( shadowMap, cubeToUV( bd3D + offset.yxx, texelSize.y ), dp )
			) * ( 1.0 / 9.0 );
		#else
			return texture2DCompare( shadowMap, cubeToUV( bd3D, texelSize.y ), dp );
		#endif
	}
#endif`,I0=`#if NUM_SPOT_LIGHT_COORDS > 0
	uniform mat4 spotLightMatrix[ NUM_SPOT_LIGHT_COORDS ];
	varying vec4 vSpotLightCoord[ NUM_SPOT_LIGHT_COORDS ];
#endif
#ifdef USE_SHADOWMAP
	#if NUM_DIR_LIGHT_SHADOWS > 0
		uniform mat4 directionalShadowMatrix[ NUM_DIR_LIGHT_SHADOWS ];
		varying vec4 vDirectionalShadowCoord[ NUM_DIR_LIGHT_SHADOWS ];
		struct DirectionalLightShadow {
			float shadowBias;
			float shadowNormalBias;
			float shadowRadius;
			vec2 shadowMapSize;
		};
		uniform DirectionalLightShadow directionalLightShadows[ NUM_DIR_LIGHT_SHADOWS ];
	#endif
	#if NUM_SPOT_LIGHT_SHADOWS > 0
		struct SpotLightShadow {
			float shadowBias;
			float shadowNormalBias;
			float shadowRadius;
			vec2 shadowMapSize;
		};
		uniform SpotLightShadow spotLightShadows[ NUM_SPOT_LIGHT_SHADOWS ];
	#endif
	#if NUM_POINT_LIGHT_SHADOWS > 0
		uniform mat4 pointShadowMatrix[ NUM_POINT_LIGHT_SHADOWS ];
		varying vec4 vPointShadowCoord[ NUM_POINT_LIGHT_SHADOWS ];
		struct PointLightShadow {
			float shadowBias;
			float shadowNormalBias;
			float shadowRadius;
			vec2 shadowMapSize;
			float shadowCameraNear;
			float shadowCameraFar;
		};
		uniform PointLightShadow pointLightShadows[ NUM_POINT_LIGHT_SHADOWS ];
	#endif
#endif`,N0=`#if ( defined( USE_SHADOWMAP ) && ( NUM_DIR_LIGHT_SHADOWS > 0 || NUM_POINT_LIGHT_SHADOWS > 0 ) ) || ( NUM_SPOT_LIGHT_COORDS > 0 )
	vec3 shadowWorldNormal = inverseTransformDirection( transformedNormal, viewMatrix );
	vec4 shadowWorldPosition;
#endif
#if defined( USE_SHADOWMAP )
	#if NUM_DIR_LIGHT_SHADOWS > 0
		#pragma unroll_loop_start
		for ( int i = 0; i < NUM_DIR_LIGHT_SHADOWS; i ++ ) {
			shadowWorldPosition = worldPosition + vec4( shadowWorldNormal * directionalLightShadows[ i ].shadowNormalBias, 0 );
			vDirectionalShadowCoord[ i ] = directionalShadowMatrix[ i ] * shadowWorldPosition;
		}
		#pragma unroll_loop_end
	#endif
	#if NUM_POINT_LIGHT_SHADOWS > 0
		#pragma unroll_loop_start
		for ( int i = 0; i < NUM_POINT_LIGHT_SHADOWS; i ++ ) {
			shadowWorldPosition = worldPosition + vec4( shadowWorldNormal * pointLightShadows[ i ].shadowNormalBias, 0 );
			vPointShadowCoord[ i ] = pointShadowMatrix[ i ] * shadowWorldPosition;
		}
		#pragma unroll_loop_end
	#endif
#endif
#if NUM_SPOT_LIGHT_COORDS > 0
	#pragma unroll_loop_start
	for ( int i = 0; i < NUM_SPOT_LIGHT_COORDS; i ++ ) {
		shadowWorldPosition = worldPosition;
		#if ( defined( USE_SHADOWMAP ) && UNROLLED_LOOP_INDEX < NUM_SPOT_LIGHT_SHADOWS )
			shadowWorldPosition.xyz += shadowWorldNormal * spotLightShadows[ i ].shadowNormalBias;
		#endif
		vSpotLightCoord[ i ] = spotLightMatrix[ i ] * shadowWorldPosition;
	}
	#pragma unroll_loop_end
#endif`,F0=`float getShadowMask() {
	float shadow = 1.0;
	#ifdef USE_SHADOWMAP
	#if NUM_DIR_LIGHT_SHADOWS > 0
	DirectionalLightShadow directionalLight;
	#pragma unroll_loop_start
	for ( int i = 0; i < NUM_DIR_LIGHT_SHADOWS; i ++ ) {
		directionalLight = directionalLightShadows[ i ];
		shadow *= receiveShadow ? getShadow( directionalShadowMap[ i ], directionalLight.shadowMapSize, directionalLight.shadowBias, directionalLight.shadowRadius, vDirectionalShadowCoord[ i ] ) : 1.0;
	}
	#pragma unroll_loop_end
	#endif
	#if NUM_SPOT_LIGHT_SHADOWS > 0
	SpotLightShadow spotLight;
	#pragma unroll_loop_start
	for ( int i = 0; i < NUM_SPOT_LIGHT_SHADOWS; i ++ ) {
		spotLight = spotLightShadows[ i ];
		shadow *= receiveShadow ? getShadow( spotShadowMap[ i ], spotLight.shadowMapSize, spotLight.shadowBias, spotLight.shadowRadius, vSpotLightCoord[ i ] ) : 1.0;
	}
	#pragma unroll_loop_end
	#endif
	#if NUM_POINT_LIGHT_SHADOWS > 0
	PointLightShadow pointLight;
	#pragma unroll_loop_start
	for ( int i = 0; i < NUM_POINT_LIGHT_SHADOWS; i ++ ) {
		pointLight = pointLightShadows[ i ];
		shadow *= receiveShadow ? getPointShadow( pointShadowMap[ i ], pointLight.shadowMapSize, pointLight.shadowBias, pointLight.shadowRadius, vPointShadowCoord[ i ], pointLight.shadowCameraNear, pointLight.shadowCameraFar ) : 1.0;
	}
	#pragma unroll_loop_end
	#endif
	#endif
	return shadow;
}`,B0=`#ifdef USE_SKINNING
	mat4 boneMatX = getBoneMatrix( skinIndex.x );
	mat4 boneMatY = getBoneMatrix( skinIndex.y );
	mat4 boneMatZ = getBoneMatrix( skinIndex.z );
	mat4 boneMatW = getBoneMatrix( skinIndex.w );
#endif`,U0=`#ifdef USE_SKINNING
	uniform mat4 bindMatrix;
	uniform mat4 bindMatrixInverse;
	uniform highp sampler2D boneTexture;
	mat4 getBoneMatrix( const in float i ) {
		int size = textureSize( boneTexture, 0 ).x;
		int j = int( i ) * 4;
		int x = j % size;
		int y = j / size;
		vec4 v1 = texelFetch( boneTexture, ivec2( x, y ), 0 );
		vec4 v2 = texelFetch( boneTexture, ivec2( x + 1, y ), 0 );
		vec4 v3 = texelFetch( boneTexture, ivec2( x + 2, y ), 0 );
		vec4 v4 = texelFetch( boneTexture, ivec2( x + 3, y ), 0 );
		return mat4( v1, v2, v3, v4 );
	}
#endif`,z0=`#ifdef USE_SKINNING
	vec4 skinVertex = bindMatrix * vec4( transformed, 1.0 );
	vec4 skinned = vec4( 0.0 );
	skinned += boneMatX * skinVertex * skinWeight.x;
	skinned += boneMatY * skinVertex * skinWeight.y;
	skinned += boneMatZ * skinVertex * skinWeight.z;
	skinned += boneMatW * skinVertex * skinWeight.w;
	transformed = ( bindMatrixInverse * skinned ).xyz;
#endif`,O0=`#ifdef USE_SKINNING
	mat4 skinMatrix = mat4( 0.0 );
	skinMatrix += skinWeight.x * boneMatX;
	skinMatrix += skinWeight.y * boneMatY;
	skinMatrix += skinWeight.z * boneMatZ;
	skinMatrix += skinWeight.w * boneMatW;
	skinMatrix = bindMatrixInverse * skinMatrix * bindMatrix;
	objectNormal = vec4( skinMatrix * vec4( objectNormal, 0.0 ) ).xyz;
	#ifdef USE_TANGENT
		objectTangent = vec4( skinMatrix * vec4( objectTangent, 0.0 ) ).xyz;
	#endif
#endif`,G0=`float specularStrength;
#ifdef USE_SPECULARMAP
	vec4 texelSpecular = texture2D( specularMap, vSpecularMapUv );
	specularStrength = texelSpecular.r;
#else
	specularStrength = 1.0;
#endif`,V0=`#ifdef USE_SPECULARMAP
	uniform sampler2D specularMap;
#endif`,k0=`#if defined( TONE_MAPPING )
	gl_FragColor.rgb = toneMapping( gl_FragColor.rgb );
#endif`,H0=`#ifndef saturate
#define saturate( a ) clamp( a, 0.0, 1.0 )
#endif
uniform float toneMappingExposure;
vec3 LinearToneMapping( vec3 color ) {
	return saturate( toneMappingExposure * color );
}
vec3 ReinhardToneMapping( vec3 color ) {
	color *= toneMappingExposure;
	return saturate( color / ( vec3( 1.0 ) + color ) );
}
vec3 OptimizedCineonToneMapping( vec3 color ) {
	color *= toneMappingExposure;
	color = max( vec3( 0.0 ), color - 0.004 );
	return pow( ( color * ( 6.2 * color + 0.5 ) ) / ( color * ( 6.2 * color + 1.7 ) + 0.06 ), vec3( 2.2 ) );
}
vec3 RRTAndODTFit( vec3 v ) {
	vec3 a = v * ( v + 0.0245786 ) - 0.000090537;
	vec3 b = v * ( 0.983729 * v + 0.4329510 ) + 0.238081;
	return a / b;
}
vec3 ACESFilmicToneMapping( vec3 color ) {
	const mat3 ACESInputMat = mat3(
		vec3( 0.59719, 0.07600, 0.02840 ),		vec3( 0.35458, 0.90834, 0.13383 ),
		vec3( 0.04823, 0.01566, 0.83777 )
	);
	const mat3 ACESOutputMat = mat3(
		vec3(  1.60475, -0.10208, -0.00327 ),		vec3( -0.53108,  1.10813, -0.07276 ),
		vec3( -0.07367, -0.00605,  1.07602 )
	);
	color *= toneMappingExposure / 0.6;
	color = ACESInputMat * color;
	color = RRTAndODTFit( color );
	color = ACESOutputMat * color;
	return saturate( color );
}
const mat3 LINEAR_REC2020_TO_LINEAR_SRGB = mat3(
	vec3( 1.6605, - 0.1246, - 0.0182 ),
	vec3( - 0.5876, 1.1329, - 0.1006 ),
	vec3( - 0.0728, - 0.0083, 1.1187 )
);
const mat3 LINEAR_SRGB_TO_LINEAR_REC2020 = mat3(
	vec3( 0.6274, 0.0691, 0.0164 ),
	vec3( 0.3293, 0.9195, 0.0880 ),
	vec3( 0.0433, 0.0113, 0.8956 )
);
vec3 agxDefaultContrastApprox( vec3 x ) {
	vec3 x2 = x * x;
	vec3 x4 = x2 * x2;
	return + 15.5 * x4 * x2
		- 40.14 * x4 * x
		+ 31.96 * x4
		- 6.868 * x2 * x
		+ 0.4298 * x2
		+ 0.1191 * x
		- 0.00232;
}
vec3 AgXToneMapping( vec3 color ) {
	const mat3 AgXInsetMatrix = mat3(
		vec3( 0.856627153315983, 0.137318972929847, 0.11189821299995 ),
		vec3( 0.0951212405381588, 0.761241990602591, 0.0767994186031903 ),
		vec3( 0.0482516061458583, 0.101439036467562, 0.811302368396859 )
	);
	const mat3 AgXOutsetMatrix = mat3(
		vec3( 1.1271005818144368, - 0.1413297634984383, - 0.14132976349843826 ),
		vec3( - 0.11060664309660323, 1.157823702216272, - 0.11060664309660294 ),
		vec3( - 0.016493938717834573, - 0.016493938717834257, 1.2519364065950405 )
	);
	const float AgxMinEv = - 12.47393;	const float AgxMaxEv = 4.026069;
	color = LINEAR_SRGB_TO_LINEAR_REC2020 * color;
	color *= toneMappingExposure;
	color = AgXInsetMatrix * color;
	color = max( color, 1e-10 );	color = log2( color );
	color = ( color - AgxMinEv ) / ( AgxMaxEv - AgxMinEv );
	color = clamp( color, 0.0, 1.0 );
	color = agxDefaultContrastApprox( color );
	color = AgXOutsetMatrix * color;
	color = pow( max( vec3( 0.0 ), color ), vec3( 2.2 ) );
	color = LINEAR_REC2020_TO_LINEAR_SRGB * color;
	return color;
}
vec3 CustomToneMapping( vec3 color ) { return color; }`,W0=`#ifdef USE_TRANSMISSION
	material.transmission = transmission;
	material.transmissionAlpha = 1.0;
	material.thickness = thickness;
	material.attenuationDistance = attenuationDistance;
	material.attenuationColor = attenuationColor;
	#ifdef USE_TRANSMISSIONMAP
		material.transmission *= texture2D( transmissionMap, vTransmissionMapUv ).r;
	#endif
	#ifdef USE_THICKNESSMAP
		material.thickness *= texture2D( thicknessMap, vThicknessMapUv ).g;
	#endif
	vec3 pos = vWorldPosition;
	vec3 v = normalize( cameraPosition - pos );
	vec3 n = inverseTransformDirection( normal, viewMatrix );
	vec4 transmitted = getIBLVolumeRefraction(
		n, v, material.roughness, material.diffuseColor, material.specularColor, material.specularF90,
		pos, modelMatrix, viewMatrix, projectionMatrix, material.ior, material.thickness,
		material.attenuationColor, material.attenuationDistance );
	material.transmissionAlpha = mix( material.transmissionAlpha, transmitted.a, material.transmission );
	totalDiffuse = mix( totalDiffuse, transmitted.rgb, material.transmission );
#endif`,X0=`#ifdef USE_TRANSMISSION
	uniform float transmission;
	uniform float thickness;
	uniform float attenuationDistance;
	uniform vec3 attenuationColor;
	#ifdef USE_TRANSMISSIONMAP
		uniform sampler2D transmissionMap;
	#endif
	#ifdef USE_THICKNESSMAP
		uniform sampler2D thicknessMap;
	#endif
	uniform vec2 transmissionSamplerSize;
	uniform sampler2D transmissionSamplerMap;
	uniform mat4 modelMatrix;
	uniform mat4 projectionMatrix;
	varying vec3 vWorldPosition;
	float w0( float a ) {
		return ( 1.0 / 6.0 ) * ( a * ( a * ( - a + 3.0 ) - 3.0 ) + 1.0 );
	}
	float w1( float a ) {
		return ( 1.0 / 6.0 ) * ( a *  a * ( 3.0 * a - 6.0 ) + 4.0 );
	}
	float w2( float a ){
		return ( 1.0 / 6.0 ) * ( a * ( a * ( - 3.0 * a + 3.0 ) + 3.0 ) + 1.0 );
	}
	float w3( float a ) {
		return ( 1.0 / 6.0 ) * ( a * a * a );
	}
	float g0( float a ) {
		return w0( a ) + w1( a );
	}
	float g1( float a ) {
		return w2( a ) + w3( a );
	}
	float h0( float a ) {
		return - 1.0 + w1( a ) / ( w0( a ) + w1( a ) );
	}
	float h1( float a ) {
		return 1.0 + w3( a ) / ( w2( a ) + w3( a ) );
	}
	vec4 bicubic( sampler2D tex, vec2 uv, vec4 texelSize, float lod ) {
		uv = uv * texelSize.zw + 0.5;
		vec2 iuv = floor( uv );
		vec2 fuv = fract( uv );
		float g0x = g0( fuv.x );
		float g1x = g1( fuv.x );
		float h0x = h0( fuv.x );
		float h1x = h1( fuv.x );
		float h0y = h0( fuv.y );
		float h1y = h1( fuv.y );
		vec2 p0 = ( vec2( iuv.x + h0x, iuv.y + h0y ) - 0.5 ) * texelSize.xy;
		vec2 p1 = ( vec2( iuv.x + h1x, iuv.y + h0y ) - 0.5 ) * texelSize.xy;
		vec2 p2 = ( vec2( iuv.x + h0x, iuv.y + h1y ) - 0.5 ) * texelSize.xy;
		vec2 p3 = ( vec2( iuv.x + h1x, iuv.y + h1y ) - 0.5 ) * texelSize.xy;
		return g0( fuv.y ) * ( g0x * textureLod( tex, p0, lod ) + g1x * textureLod( tex, p1, lod ) ) +
			g1( fuv.y ) * ( g0x * textureLod( tex, p2, lod ) + g1x * textureLod( tex, p3, lod ) );
	}
	vec4 textureBicubic( sampler2D sampler, vec2 uv, float lod ) {
		vec2 fLodSize = vec2( textureSize( sampler, int( lod ) ) );
		vec2 cLodSize = vec2( textureSize( sampler, int( lod + 1.0 ) ) );
		vec2 fLodSizeInv = 1.0 / fLodSize;
		vec2 cLodSizeInv = 1.0 / cLodSize;
		vec4 fSample = bicubic( sampler, uv, vec4( fLodSizeInv, fLodSize ), floor( lod ) );
		vec4 cSample = bicubic( sampler, uv, vec4( cLodSizeInv, cLodSize ), ceil( lod ) );
		return mix( fSample, cSample, fract( lod ) );
	}
	vec3 getVolumeTransmissionRay( const in vec3 n, const in vec3 v, const in float thickness, const in float ior, const in mat4 modelMatrix ) {
		vec3 refractionVector = refract( - v, normalize( n ), 1.0 / ior );
		vec3 modelScale;
		modelScale.x = length( vec3( modelMatrix[ 0 ].xyz ) );
		modelScale.y = length( vec3( modelMatrix[ 1 ].xyz ) );
		modelScale.z = length( vec3( modelMatrix[ 2 ].xyz ) );
		return normalize( refractionVector ) * thickness * modelScale;
	}
	float applyIorToRoughness( const in float roughness, const in float ior ) {
		return roughness * clamp( ior * 2.0 - 2.0, 0.0, 1.0 );
	}
	vec4 getTransmissionSample( const in vec2 fragCoord, const in float roughness, const in float ior ) {
		float lod = log2( transmissionSamplerSize.x ) * applyIorToRoughness( roughness, ior );
		return textureBicubic( transmissionSamplerMap, fragCoord.xy, lod );
	}
	vec3 volumeAttenuation( const in float transmissionDistance, const in vec3 attenuationColor, const in float attenuationDistance ) {
		if ( isinf( attenuationDistance ) ) {
			return vec3( 1.0 );
		} else {
			vec3 attenuationCoefficient = -log( attenuationColor ) / attenuationDistance;
			vec3 transmittance = exp( - attenuationCoefficient * transmissionDistance );			return transmittance;
		}
	}
	vec4 getIBLVolumeRefraction( const in vec3 n, const in vec3 v, const in float roughness, const in vec3 diffuseColor,
		const in vec3 specularColor, const in float specularF90, const in vec3 position, const in mat4 modelMatrix,
		const in mat4 viewMatrix, const in mat4 projMatrix, const in float ior, const in float thickness,
		const in vec3 attenuationColor, const in float attenuationDistance ) {
		vec3 transmissionRay = getVolumeTransmissionRay( n, v, thickness, ior, modelMatrix );
		vec3 refractedRayExit = position + transmissionRay;
		vec4 ndcPos = projMatrix * viewMatrix * vec4( refractedRayExit, 1.0 );
		vec2 refractionCoords = ndcPos.xy / ndcPos.w;
		refractionCoords += 1.0;
		refractionCoords /= 2.0;
		vec4 transmittedLight = getTransmissionSample( refractionCoords, roughness, ior );
		vec3 transmittance = diffuseColor * volumeAttenuation( length( transmissionRay ), attenuationColor, attenuationDistance );
		vec3 attenuatedColor = transmittance * transmittedLight.rgb;
		vec3 F = EnvironmentBRDF( n, v, specularColor, specularF90, roughness );
		float transmittanceFactor = ( transmittance.r + transmittance.g + transmittance.b ) / 3.0;
		return vec4( ( 1.0 - F ) * attenuatedColor, 1.0 - ( 1.0 - transmittedLight.a ) * transmittanceFactor );
	}
#endif`,Y0=`#if defined( USE_UV ) || defined( USE_ANISOTROPY )
	varying vec2 vUv;
#endif
#ifdef USE_MAP
	varying vec2 vMapUv;
#endif
#ifdef USE_ALPHAMAP
	varying vec2 vAlphaMapUv;
#endif
#ifdef USE_LIGHTMAP
	varying vec2 vLightMapUv;
#endif
#ifdef USE_AOMAP
	varying vec2 vAoMapUv;
#endif
#ifdef USE_BUMPMAP
	varying vec2 vBumpMapUv;
#endif
#ifdef USE_NORMALMAP
	varying vec2 vNormalMapUv;
#endif
#ifdef USE_EMISSIVEMAP
	varying vec2 vEmissiveMapUv;
#endif
#ifdef USE_METALNESSMAP
	varying vec2 vMetalnessMapUv;
#endif
#ifdef USE_ROUGHNESSMAP
	varying vec2 vRoughnessMapUv;
#endif
#ifdef USE_ANISOTROPYMAP
	varying vec2 vAnisotropyMapUv;
#endif
#ifdef USE_CLEARCOATMAP
	varying vec2 vClearcoatMapUv;
#endif
#ifdef USE_CLEARCOAT_NORMALMAP
	varying vec2 vClearcoatNormalMapUv;
#endif
#ifdef USE_CLEARCOAT_ROUGHNESSMAP
	varying vec2 vClearcoatRoughnessMapUv;
#endif
#ifdef USE_IRIDESCENCEMAP
	varying vec2 vIridescenceMapUv;
#endif
#ifdef USE_IRIDESCENCE_THICKNESSMAP
	varying vec2 vIridescenceThicknessMapUv;
#endif
#ifdef USE_SHEEN_COLORMAP
	varying vec2 vSheenColorMapUv;
#endif
#ifdef USE_SHEEN_ROUGHNESSMAP
	varying vec2 vSheenRoughnessMapUv;
#endif
#ifdef USE_SPECULARMAP
	varying vec2 vSpecularMapUv;
#endif
#ifdef USE_SPECULAR_COLORMAP
	varying vec2 vSpecularColorMapUv;
#endif
#ifdef USE_SPECULAR_INTENSITYMAP
	varying vec2 vSpecularIntensityMapUv;
#endif
#ifdef USE_TRANSMISSIONMAP
	uniform mat3 transmissionMapTransform;
	varying vec2 vTransmissionMapUv;
#endif
#ifdef USE_THICKNESSMAP
	uniform mat3 thicknessMapTransform;
	varying vec2 vThicknessMapUv;
#endif`,q0=`#if defined( USE_UV ) || defined( USE_ANISOTROPY )
	varying vec2 vUv;
#endif
#ifdef USE_MAP
	uniform mat3 mapTransform;
	varying vec2 vMapUv;
#endif
#ifdef USE_ALPHAMAP
	uniform mat3 alphaMapTransform;
	varying vec2 vAlphaMapUv;
#endif
#ifdef USE_LIGHTMAP
	uniform mat3 lightMapTransform;
	varying vec2 vLightMapUv;
#endif
#ifdef USE_AOMAP
	uniform mat3 aoMapTransform;
	varying vec2 vAoMapUv;
#endif
#ifdef USE_BUMPMAP
	uniform mat3 bumpMapTransform;
	varying vec2 vBumpMapUv;
#endif
#ifdef USE_NORMALMAP
	uniform mat3 normalMapTransform;
	varying vec2 vNormalMapUv;
#endif
#ifdef USE_DISPLACEMENTMAP
	uniform mat3 displacementMapTransform;
	varying vec2 vDisplacementMapUv;
#endif
#ifdef USE_EMISSIVEMAP
	uniform mat3 emissiveMapTransform;
	varying vec2 vEmissiveMapUv;
#endif
#ifdef USE_METALNESSMAP
	uniform mat3 metalnessMapTransform;
	varying vec2 vMetalnessMapUv;
#endif
#ifdef USE_ROUGHNESSMAP
	uniform mat3 roughnessMapTransform;
	varying vec2 vRoughnessMapUv;
#endif
#ifdef USE_ANISOTROPYMAP
	uniform mat3 anisotropyMapTransform;
	varying vec2 vAnisotropyMapUv;
#endif
#ifdef USE_CLEARCOATMAP
	uniform mat3 clearcoatMapTransform;
	varying vec2 vClearcoatMapUv;
#endif
#ifdef USE_CLEARCOAT_NORMALMAP
	uniform mat3 clearcoatNormalMapTransform;
	varying vec2 vClearcoatNormalMapUv;
#endif
#ifdef USE_CLEARCOAT_ROUGHNESSMAP
	uniform mat3 clearcoatRoughnessMapTransform;
	varying vec2 vClearcoatRoughnessMapUv;
#endif
#ifdef USE_SHEEN_COLORMAP
	uniform mat3 sheenColorMapTransform;
	varying vec2 vSheenColorMapUv;
#endif
#ifdef USE_SHEEN_ROUGHNESSMAP
	uniform mat3 sheenRoughnessMapTransform;
	varying vec2 vSheenRoughnessMapUv;
#endif
#ifdef USE_IRIDESCENCEMAP
	uniform mat3 iridescenceMapTransform;
	varying vec2 vIridescenceMapUv;
#endif
#ifdef USE_IRIDESCENCE_THICKNESSMAP
	uniform mat3 iridescenceThicknessMapTransform;
	varying vec2 vIridescenceThicknessMapUv;
#endif
#ifdef USE_SPECULARMAP
	uniform mat3 specularMapTransform;
	varying vec2 vSpecularMapUv;
#endif
#ifdef USE_SPECULAR_COLORMAP
	uniform mat3 specularColorMapTransform;
	varying vec2 vSpecularColorMapUv;
#endif
#ifdef USE_SPECULAR_INTENSITYMAP
	uniform mat3 specularIntensityMapTransform;
	varying vec2 vSpecularIntensityMapUv;
#endif
#ifdef USE_TRANSMISSIONMAP
	uniform mat3 transmissionMapTransform;
	varying vec2 vTransmissionMapUv;
#endif
#ifdef USE_THICKNESSMAP
	uniform mat3 thicknessMapTransform;
	varying vec2 vThicknessMapUv;
#endif`,Z0=`#if defined( USE_UV ) || defined( USE_ANISOTROPY )
	vUv = vec3( uv, 1 ).xy;
#endif
#ifdef USE_MAP
	vMapUv = ( mapTransform * vec3( MAP_UV, 1 ) ).xy;
#endif
#ifdef USE_ALPHAMAP
	vAlphaMapUv = ( alphaMapTransform * vec3( ALPHAMAP_UV, 1 ) ).xy;
#endif
#ifdef USE_LIGHTMAP
	vLightMapUv = ( lightMapTransform * vec3( LIGHTMAP_UV, 1 ) ).xy;
#endif
#ifdef USE_AOMAP
	vAoMapUv = ( aoMapTransform * vec3( AOMAP_UV, 1 ) ).xy;
#endif
#ifdef USE_BUMPMAP
	vBumpMapUv = ( bumpMapTransform * vec3( BUMPMAP_UV, 1 ) ).xy;
#endif
#ifdef USE_NORMALMAP
	vNormalMapUv = ( normalMapTransform * vec3( NORMALMAP_UV, 1 ) ).xy;
#endif
#ifdef USE_DISPLACEMENTMAP
	vDisplacementMapUv = ( displacementMapTransform * vec3( DISPLACEMENTMAP_UV, 1 ) ).xy;
#endif
#ifdef USE_EMISSIVEMAP
	vEmissiveMapUv = ( emissiveMapTransform * vec3( EMISSIVEMAP_UV, 1 ) ).xy;
#endif
#ifdef USE_METALNESSMAP
	vMetalnessMapUv = ( metalnessMapTransform * vec3( METALNESSMAP_UV, 1 ) ).xy;
#endif
#ifdef USE_ROUGHNESSMAP
	vRoughnessMapUv = ( roughnessMapTransform * vec3( ROUGHNESSMAP_UV, 1 ) ).xy;
#endif
#ifdef USE_ANISOTROPYMAP
	vAnisotropyMapUv = ( anisotropyMapTransform * vec3( ANISOTROPYMAP_UV, 1 ) ).xy;
#endif
#ifdef USE_CLEARCOATMAP
	vClearcoatMapUv = ( clearcoatMapTransform * vec3( CLEARCOATMAP_UV, 1 ) ).xy;
#endif
#ifdef USE_CLEARCOAT_NORMALMAP
	vClearcoatNormalMapUv = ( clearcoatNormalMapTransform * vec3( CLEARCOAT_NORMALMAP_UV, 1 ) ).xy;
#endif
#ifdef USE_CLEARCOAT_ROUGHNESSMAP
	vClearcoatRoughnessMapUv = ( clearcoatRoughnessMapTransform * vec3( CLEARCOAT_ROUGHNESSMAP_UV, 1 ) ).xy;
#endif
#ifdef USE_IRIDESCENCEMAP
	vIridescenceMapUv = ( iridescenceMapTransform * vec3( IRIDESCENCEMAP_UV, 1 ) ).xy;
#endif
#ifdef USE_IRIDESCENCE_THICKNESSMAP
	vIridescenceThicknessMapUv = ( iridescenceThicknessMapTransform * vec3( IRIDESCENCE_THICKNESSMAP_UV, 1 ) ).xy;
#endif
#ifdef USE_SHEEN_COLORMAP
	vSheenColorMapUv = ( sheenColorMapTransform * vec3( SHEEN_COLORMAP_UV, 1 ) ).xy;
#endif
#ifdef USE_SHEEN_ROUGHNESSMAP
	vSheenRoughnessMapUv = ( sheenRoughnessMapTransform * vec3( SHEEN_ROUGHNESSMAP_UV, 1 ) ).xy;
#endif
#ifdef USE_SPECULARMAP
	vSpecularMapUv = ( specularMapTransform * vec3( SPECULARMAP_UV, 1 ) ).xy;
#endif
#ifdef USE_SPECULAR_COLORMAP
	vSpecularColorMapUv = ( specularColorMapTransform * vec3( SPECULAR_COLORMAP_UV, 1 ) ).xy;
#endif
#ifdef USE_SPECULAR_INTENSITYMAP
	vSpecularIntensityMapUv = ( specularIntensityMapTransform * vec3( SPECULAR_INTENSITYMAP_UV, 1 ) ).xy;
#endif
#ifdef USE_TRANSMISSIONMAP
	vTransmissionMapUv = ( transmissionMapTransform * vec3( TRANSMISSIONMAP_UV, 1 ) ).xy;
#endif
#ifdef USE_THICKNESSMAP
	vThicknessMapUv = ( thicknessMapTransform * vec3( THICKNESSMAP_UV, 1 ) ).xy;
#endif`,j0=`#if defined( USE_ENVMAP ) || defined( DISTANCE ) || defined ( USE_SHADOWMAP ) || defined ( USE_TRANSMISSION ) || NUM_SPOT_LIGHT_COORDS > 0
	vec4 worldPosition = vec4( transformed, 1.0 );
	#ifdef USE_BATCHING
		worldPosition = batchingMatrix * worldPosition;
	#endif
	#ifdef USE_INSTANCING
		worldPosition = instanceMatrix * worldPosition;
	#endif
	worldPosition = modelMatrix * worldPosition;
#endif`;const $0=`varying vec2 vUv;
uniform mat3 uvTransform;
void main() {
	vUv = ( uvTransform * vec3( uv, 1 ) ).xy;
	gl_Position = vec4( position.xy, 1.0, 1.0 );
}`,K0=`uniform sampler2D t2D;
uniform float backgroundIntensity;
varying vec2 vUv;
void main() {
	vec4 texColor = texture2D( t2D, vUv );
	#ifdef DECODE_VIDEO_TEXTURE
		texColor = vec4( mix( pow( texColor.rgb * 0.9478672986 + vec3( 0.0521327014 ), vec3( 2.4 ) ), texColor.rgb * 0.0773993808, vec3( lessThanEqual( texColor.rgb, vec3( 0.04045 ) ) ) ), texColor.w );
	#endif
	texColor.rgb *= backgroundIntensity;
	gl_FragColor = texColor;
	#include <tonemapping_fragment>
	#include <colorspace_fragment>
}`,J0=`varying vec3 vWorldDirection;
#include <common>
void main() {
	vWorldDirection = transformDirection( position, modelMatrix );
	#include <begin_vertex>
	#include <project_vertex>
	gl_Position.z = gl_Position.w;
}`,Q0=`#ifdef ENVMAP_TYPE_CUBE
	uniform samplerCube envMap;
#elif defined( ENVMAP_TYPE_CUBE_UV )
	uniform sampler2D envMap;
#endif
uniform float flipEnvMap;
uniform float backgroundBlurriness;
uniform float backgroundIntensity;
varying vec3 vWorldDirection;
#include <cube_uv_reflection_fragment>
void main() {
	#ifdef ENVMAP_TYPE_CUBE
		vec4 texColor = textureCube( envMap, vec3( flipEnvMap * vWorldDirection.x, vWorldDirection.yz ) );
	#elif defined( ENVMAP_TYPE_CUBE_UV )
		vec4 texColor = textureCubeUV( envMap, vWorldDirection, backgroundBlurriness );
	#else
		vec4 texColor = vec4( 0.0, 0.0, 0.0, 1.0 );
	#endif
	texColor.rgb *= backgroundIntensity;
	gl_FragColor = texColor;
	#include <tonemapping_fragment>
	#include <colorspace_fragment>
}`,tx=`varying vec3 vWorldDirection;
#include <common>
void main() {
	vWorldDirection = transformDirection( position, modelMatrix );
	#include <begin_vertex>
	#include <project_vertex>
	gl_Position.z = gl_Position.w;
}`,ex=`uniform samplerCube tCube;
uniform float tFlip;
uniform float opacity;
varying vec3 vWorldDirection;
void main() {
	vec4 texColor = textureCube( tCube, vec3( tFlip * vWorldDirection.x, vWorldDirection.yz ) );
	gl_FragColor = texColor;
	gl_FragColor.a *= opacity;
	#include <tonemapping_fragment>
	#include <colorspace_fragment>
}`,nx=`#include <common>
#include <batching_pars_vertex>
#include <uv_pars_vertex>
#include <displacementmap_pars_vertex>
#include <morphtarget_pars_vertex>
#include <skinning_pars_vertex>
#include <logdepthbuf_pars_vertex>
#include <clipping_planes_pars_vertex>
varying vec2 vHighPrecisionZW;
void main() {
	#include <uv_vertex>
	#include <batching_vertex>
	#include <skinbase_vertex>
	#ifdef USE_DISPLACEMENTMAP
		#include <beginnormal_vertex>
		#include <morphnormal_vertex>
		#include <skinnormal_vertex>
	#endif
	#include <begin_vertex>
	#include <morphtarget_vertex>
	#include <skinning_vertex>
	#include <displacementmap_vertex>
	#include <project_vertex>
	#include <logdepthbuf_vertex>
	#include <clipping_planes_vertex>
	vHighPrecisionZW = gl_Position.zw;
}`,ix=`#if DEPTH_PACKING == 3200
	uniform float opacity;
#endif
#include <common>
#include <packing>
#include <uv_pars_fragment>
#include <map_pars_fragment>
#include <alphamap_pars_fragment>
#include <alphatest_pars_fragment>
#include <alphahash_pars_fragment>
#include <logdepthbuf_pars_fragment>
#include <clipping_planes_pars_fragment>
varying vec2 vHighPrecisionZW;
void main() {
	#include <clipping_planes_fragment>
	vec4 diffuseColor = vec4( 1.0 );
	#if DEPTH_PACKING == 3200
		diffuseColor.a = opacity;
	#endif
	#include <map_fragment>
	#include <alphamap_fragment>
	#include <alphatest_fragment>
	#include <alphahash_fragment>
	#include <logdepthbuf_fragment>
	float fragCoordZ = 0.5 * vHighPrecisionZW[0] / vHighPrecisionZW[1] + 0.5;
	#if DEPTH_PACKING == 3200
		gl_FragColor = vec4( vec3( 1.0 - fragCoordZ ), opacity );
	#elif DEPTH_PACKING == 3201
		gl_FragColor = packDepthToRGBA( fragCoordZ );
	#endif
}`,sx=`#define DISTANCE
varying vec3 vWorldPosition;
#include <common>
#include <batching_pars_vertex>
#include <uv_pars_vertex>
#include <displacementmap_pars_vertex>
#include <morphtarget_pars_vertex>
#include <skinning_pars_vertex>
#include <clipping_planes_pars_vertex>
void main() {
	#include <uv_vertex>
	#include <batching_vertex>
	#include <skinbase_vertex>
	#ifdef USE_DISPLACEMENTMAP
		#include <beginnormal_vertex>
		#include <morphnormal_vertex>
		#include <skinnormal_vertex>
	#endif
	#include <begin_vertex>
	#include <morphtarget_vertex>
	#include <skinning_vertex>
	#include <displacementmap_vertex>
	#include <project_vertex>
	#include <worldpos_vertex>
	#include <clipping_planes_vertex>
	vWorldPosition = worldPosition.xyz;
}`,rx=`#define DISTANCE
uniform vec3 referencePosition;
uniform float nearDistance;
uniform float farDistance;
varying vec3 vWorldPosition;
#include <common>
#include <packing>
#include <uv_pars_fragment>
#include <map_pars_fragment>
#include <alphamap_pars_fragment>
#include <alphatest_pars_fragment>
#include <alphahash_pars_fragment>
#include <clipping_planes_pars_fragment>
void main () {
	#include <clipping_planes_fragment>
	vec4 diffuseColor = vec4( 1.0 );
	#include <map_fragment>
	#include <alphamap_fragment>
	#include <alphatest_fragment>
	#include <alphahash_fragment>
	float dist = length( vWorldPosition - referencePosition );
	dist = ( dist - nearDistance ) / ( farDistance - nearDistance );
	dist = saturate( dist );
	gl_FragColor = packDepthToRGBA( dist );
}`,ax=`varying vec3 vWorldDirection;
#include <common>
void main() {
	vWorldDirection = transformDirection( position, modelMatrix );
	#include <begin_vertex>
	#include <project_vertex>
}`,ox=`uniform sampler2D tEquirect;
varying vec3 vWorldDirection;
#include <common>
void main() {
	vec3 direction = normalize( vWorldDirection );
	vec2 sampleUV = equirectUv( direction );
	gl_FragColor = texture2D( tEquirect, sampleUV );
	#include <tonemapping_fragment>
	#include <colorspace_fragment>
}`,cx=`uniform float scale;
attribute float lineDistance;
varying float vLineDistance;
#include <common>
#include <uv_pars_vertex>
#include <color_pars_vertex>
#include <fog_pars_vertex>
#include <morphtarget_pars_vertex>
#include <logdepthbuf_pars_vertex>
#include <clipping_planes_pars_vertex>
void main() {
	vLineDistance = scale * lineDistance;
	#include <uv_vertex>
	#include <color_vertex>
	#include <morphcolor_vertex>
	#include <begin_vertex>
	#include <morphtarget_vertex>
	#include <project_vertex>
	#include <logdepthbuf_vertex>
	#include <clipping_planes_vertex>
	#include <fog_vertex>
}`,lx=`uniform vec3 diffuse;
uniform float opacity;
uniform float dashSize;
uniform float totalSize;
varying float vLineDistance;
#include <common>
#include <color_pars_fragment>
#include <uv_pars_fragment>
#include <map_pars_fragment>
#include <fog_pars_fragment>
#include <logdepthbuf_pars_fragment>
#include <clipping_planes_pars_fragment>
void main() {
	#include <clipping_planes_fragment>
	if ( mod( vLineDistance, totalSize ) > dashSize ) {
		discard;
	}
	vec3 outgoingLight = vec3( 0.0 );
	vec4 diffuseColor = vec4( diffuse, opacity );
	#include <logdepthbuf_fragment>
	#include <map_fragment>
	#include <color_fragment>
	outgoingLight = diffuseColor.rgb;
	#include <opaque_fragment>
	#include <tonemapping_fragment>
	#include <colorspace_fragment>
	#include <fog_fragment>
	#include <premultiplied_alpha_fragment>
}`,hx=`#include <common>
#include <batching_pars_vertex>
#include <uv_pars_vertex>
#include <envmap_pars_vertex>
#include <color_pars_vertex>
#include <fog_pars_vertex>
#include <morphtarget_pars_vertex>
#include <skinning_pars_vertex>
#include <logdepthbuf_pars_vertex>
#include <clipping_planes_pars_vertex>
void main() {
	#include <uv_vertex>
	#include <color_vertex>
	#include <morphcolor_vertex>
	#include <batching_vertex>
	#if defined ( USE_ENVMAP ) || defined ( USE_SKINNING )
		#include <beginnormal_vertex>
		#include <morphnormal_vertex>
		#include <skinbase_vertex>
		#include <skinnormal_vertex>
		#include <defaultnormal_vertex>
	#endif
	#include <begin_vertex>
	#include <morphtarget_vertex>
	#include <skinning_vertex>
	#include <project_vertex>
	#include <logdepthbuf_vertex>
	#include <clipping_planes_vertex>
	#include <worldpos_vertex>
	#include <envmap_vertex>
	#include <fog_vertex>
}`,ux=`uniform vec3 diffuse;
uniform float opacity;
#ifndef FLAT_SHADED
	varying vec3 vNormal;
#endif
#include <common>
#include <dithering_pars_fragment>
#include <color_pars_fragment>
#include <uv_pars_fragment>
#include <map_pars_fragment>
#include <alphamap_pars_fragment>
#include <alphatest_pars_fragment>
#include <alphahash_pars_fragment>
#include <aomap_pars_fragment>
#include <lightmap_pars_fragment>
#include <envmap_common_pars_fragment>
#include <envmap_pars_fragment>
#include <fog_pars_fragment>
#include <specularmap_pars_fragment>
#include <logdepthbuf_pars_fragment>
#include <clipping_planes_pars_fragment>
void main() {
	#include <clipping_planes_fragment>
	vec4 diffuseColor = vec4( diffuse, opacity );
	#include <logdepthbuf_fragment>
	#include <map_fragment>
	#include <color_fragment>
	#include <alphamap_fragment>
	#include <alphatest_fragment>
	#include <alphahash_fragment>
	#include <specularmap_fragment>
	ReflectedLight reflectedLight = ReflectedLight( vec3( 0.0 ), vec3( 0.0 ), vec3( 0.0 ), vec3( 0.0 ) );
	#ifdef USE_LIGHTMAP
		vec4 lightMapTexel = texture2D( lightMap, vLightMapUv );
		reflectedLight.indirectDiffuse += lightMapTexel.rgb * lightMapIntensity * RECIPROCAL_PI;
	#else
		reflectedLight.indirectDiffuse += vec3( 1.0 );
	#endif
	#include <aomap_fragment>
	reflectedLight.indirectDiffuse *= diffuseColor.rgb;
	vec3 outgoingLight = reflectedLight.indirectDiffuse;
	#include <envmap_fragment>
	#include <opaque_fragment>
	#include <tonemapping_fragment>
	#include <colorspace_fragment>
	#include <fog_fragment>
	#include <premultiplied_alpha_fragment>
	#include <dithering_fragment>
}`,dx=`#define LAMBERT
varying vec3 vViewPosition;
#include <common>
#include <batching_pars_vertex>
#include <uv_pars_vertex>
#include <displacementmap_pars_vertex>
#include <envmap_pars_vertex>
#include <color_pars_vertex>
#include <fog_pars_vertex>
#include <normal_pars_vertex>
#include <morphtarget_pars_vertex>
#include <skinning_pars_vertex>
#include <shadowmap_pars_vertex>
#include <logdepthbuf_pars_vertex>
#include <clipping_planes_pars_vertex>
void main() {
	#include <uv_vertex>
	#include <color_vertex>
	#include <morphcolor_vertex>
	#include <batching_vertex>
	#include <beginnormal_vertex>
	#include <morphnormal_vertex>
	#include <skinbase_vertex>
	#include <skinnormal_vertex>
	#include <defaultnormal_vertex>
	#include <normal_vertex>
	#include <begin_vertex>
	#include <morphtarget_vertex>
	#include <skinning_vertex>
	#include <displacementmap_vertex>
	#include <project_vertex>
	#include <logdepthbuf_vertex>
	#include <clipping_planes_vertex>
	vViewPosition = - mvPosition.xyz;
	#include <worldpos_vertex>
	#include <envmap_vertex>
	#include <shadowmap_vertex>
	#include <fog_vertex>
}`,fx=`#define LAMBERT
uniform vec3 diffuse;
uniform vec3 emissive;
uniform float opacity;
#include <common>
#include <packing>
#include <dithering_pars_fragment>
#include <color_pars_fragment>
#include <uv_pars_fragment>
#include <map_pars_fragment>
#include <alphamap_pars_fragment>
#include <alphatest_pars_fragment>
#include <alphahash_pars_fragment>
#include <aomap_pars_fragment>
#include <lightmap_pars_fragment>
#include <emissivemap_pars_fragment>
#include <envmap_common_pars_fragment>
#include <envmap_pars_fragment>
#include <fog_pars_fragment>
#include <bsdfs>
#include <lights_pars_begin>
#include <normal_pars_fragment>
#include <lights_lambert_pars_fragment>
#include <shadowmap_pars_fragment>
#include <bumpmap_pars_fragment>
#include <normalmap_pars_fragment>
#include <specularmap_pars_fragment>
#include <logdepthbuf_pars_fragment>
#include <clipping_planes_pars_fragment>
void main() {
	#include <clipping_planes_fragment>
	vec4 diffuseColor = vec4( diffuse, opacity );
	ReflectedLight reflectedLight = ReflectedLight( vec3( 0.0 ), vec3( 0.0 ), vec3( 0.0 ), vec3( 0.0 ) );
	vec3 totalEmissiveRadiance = emissive;
	#include <logdepthbuf_fragment>
	#include <map_fragment>
	#include <color_fragment>
	#include <alphamap_fragment>
	#include <alphatest_fragment>
	#include <alphahash_fragment>
	#include <specularmap_fragment>
	#include <normal_fragment_begin>
	#include <normal_fragment_maps>
	#include <emissivemap_fragment>
	#include <lights_lambert_fragment>
	#include <lights_fragment_begin>
	#include <lights_fragment_maps>
	#include <lights_fragment_end>
	#include <aomap_fragment>
	vec3 outgoingLight = reflectedLight.directDiffuse + reflectedLight.indirectDiffuse + totalEmissiveRadiance;
	#include <envmap_fragment>
	#include <opaque_fragment>
	#include <tonemapping_fragment>
	#include <colorspace_fragment>
	#include <fog_fragment>
	#include <premultiplied_alpha_fragment>
	#include <dithering_fragment>
}`,px=`#define MATCAP
varying vec3 vViewPosition;
#include <common>
#include <batching_pars_vertex>
#include <uv_pars_vertex>
#include <color_pars_vertex>
#include <displacementmap_pars_vertex>
#include <fog_pars_vertex>
#include <normal_pars_vertex>
#include <morphtarget_pars_vertex>
#include <skinning_pars_vertex>
#include <logdepthbuf_pars_vertex>
#include <clipping_planes_pars_vertex>
void main() {
	#include <uv_vertex>
	#include <color_vertex>
	#include <morphcolor_vertex>
	#include <batching_vertex>
	#include <beginnormal_vertex>
	#include <morphnormal_vertex>
	#include <skinbase_vertex>
	#include <skinnormal_vertex>
	#include <defaultnormal_vertex>
	#include <normal_vertex>
	#include <begin_vertex>
	#include <morphtarget_vertex>
	#include <skinning_vertex>
	#include <displacementmap_vertex>
	#include <project_vertex>
	#include <logdepthbuf_vertex>
	#include <clipping_planes_vertex>
	#include <fog_vertex>
	vViewPosition = - mvPosition.xyz;
}`,mx=`#define MATCAP
uniform vec3 diffuse;
uniform float opacity;
uniform sampler2D matcap;
varying vec3 vViewPosition;
#include <common>
#include <dithering_pars_fragment>
#include <color_pars_fragment>
#include <uv_pars_fragment>
#include <map_pars_fragment>
#include <alphamap_pars_fragment>
#include <alphatest_pars_fragment>
#include <alphahash_pars_fragment>
#include <fog_pars_fragment>
#include <normal_pars_fragment>
#include <bumpmap_pars_fragment>
#include <normalmap_pars_fragment>
#include <logdepthbuf_pars_fragment>
#include <clipping_planes_pars_fragment>
void main() {
	#include <clipping_planes_fragment>
	vec4 diffuseColor = vec4( diffuse, opacity );
	#include <logdepthbuf_fragment>
	#include <map_fragment>
	#include <color_fragment>
	#include <alphamap_fragment>
	#include <alphatest_fragment>
	#include <alphahash_fragment>
	#include <normal_fragment_begin>
	#include <normal_fragment_maps>
	vec3 viewDir = normalize( vViewPosition );
	vec3 x = normalize( vec3( viewDir.z, 0.0, - viewDir.x ) );
	vec3 y = cross( viewDir, x );
	vec2 uv = vec2( dot( x, normal ), dot( y, normal ) ) * 0.495 + 0.5;
	#ifdef USE_MATCAP
		vec4 matcapColor = texture2D( matcap, uv );
	#else
		vec4 matcapColor = vec4( vec3( mix( 0.2, 0.8, uv.y ) ), 1.0 );
	#endif
	vec3 outgoingLight = diffuseColor.rgb * matcapColor.rgb;
	#include <opaque_fragment>
	#include <tonemapping_fragment>
	#include <colorspace_fragment>
	#include <fog_fragment>
	#include <premultiplied_alpha_fragment>
	#include <dithering_fragment>
}`,gx=`#define NORMAL
#if defined( FLAT_SHADED ) || defined( USE_BUMPMAP ) || defined( USE_NORMALMAP_TANGENTSPACE )
	varying vec3 vViewPosition;
#endif
#include <common>
#include <batching_pars_vertex>
#include <uv_pars_vertex>
#include <displacementmap_pars_vertex>
#include <normal_pars_vertex>
#include <morphtarget_pars_vertex>
#include <skinning_pars_vertex>
#include <logdepthbuf_pars_vertex>
#include <clipping_planes_pars_vertex>
void main() {
	#include <uv_vertex>
	#include <batching_vertex>
	#include <beginnormal_vertex>
	#include <morphnormal_vertex>
	#include <skinbase_vertex>
	#include <skinnormal_vertex>
	#include <defaultnormal_vertex>
	#include <normal_vertex>
	#include <begin_vertex>
	#include <morphtarget_vertex>
	#include <skinning_vertex>
	#include <displacementmap_vertex>
	#include <project_vertex>
	#include <logdepthbuf_vertex>
	#include <clipping_planes_vertex>
#if defined( FLAT_SHADED ) || defined( USE_BUMPMAP ) || defined( USE_NORMALMAP_TANGENTSPACE )
	vViewPosition = - mvPosition.xyz;
#endif
}`,xx=`#define NORMAL
uniform float opacity;
#if defined( FLAT_SHADED ) || defined( USE_BUMPMAP ) || defined( USE_NORMALMAP_TANGENTSPACE )
	varying vec3 vViewPosition;
#endif
#include <packing>
#include <uv_pars_fragment>
#include <normal_pars_fragment>
#include <bumpmap_pars_fragment>
#include <normalmap_pars_fragment>
#include <logdepthbuf_pars_fragment>
#include <clipping_planes_pars_fragment>
void main() {
	#include <clipping_planes_fragment>
	#include <logdepthbuf_fragment>
	#include <normal_fragment_begin>
	#include <normal_fragment_maps>
	gl_FragColor = vec4( packNormalToRGB( normal ), opacity );
	#ifdef OPAQUE
		gl_FragColor.a = 1.0;
	#endif
}`,_x=`#define PHONG
varying vec3 vViewPosition;
#include <common>
#include <batching_pars_vertex>
#include <uv_pars_vertex>
#include <displacementmap_pars_vertex>
#include <envmap_pars_vertex>
#include <color_pars_vertex>
#include <fog_pars_vertex>
#include <normal_pars_vertex>
#include <morphtarget_pars_vertex>
#include <skinning_pars_vertex>
#include <shadowmap_pars_vertex>
#include <logdepthbuf_pars_vertex>
#include <clipping_planes_pars_vertex>
void main() {
	#include <uv_vertex>
	#include <color_vertex>
	#include <morphcolor_vertex>
	#include <batching_vertex>
	#include <beginnormal_vertex>
	#include <morphnormal_vertex>
	#include <skinbase_vertex>
	#include <skinnormal_vertex>
	#include <defaultnormal_vertex>
	#include <normal_vertex>
	#include <begin_vertex>
	#include <morphtarget_vertex>
	#include <skinning_vertex>
	#include <displacementmap_vertex>
	#include <project_vertex>
	#include <logdepthbuf_vertex>
	#include <clipping_planes_vertex>
	vViewPosition = - mvPosition.xyz;
	#include <worldpos_vertex>
	#include <envmap_vertex>
	#include <shadowmap_vertex>
	#include <fog_vertex>
}`,vx=`#define PHONG
uniform vec3 diffuse;
uniform vec3 emissive;
uniform vec3 specular;
uniform float shininess;
uniform float opacity;
#include <common>
#include <packing>
#include <dithering_pars_fragment>
#include <color_pars_fragment>
#include <uv_pars_fragment>
#include <map_pars_fragment>
#include <alphamap_pars_fragment>
#include <alphatest_pars_fragment>
#include <alphahash_pars_fragment>
#include <aomap_pars_fragment>
#include <lightmap_pars_fragment>
#include <emissivemap_pars_fragment>
#include <envmap_common_pars_fragment>
#include <envmap_pars_fragment>
#include <fog_pars_fragment>
#include <bsdfs>
#include <lights_pars_begin>
#include <normal_pars_fragment>
#include <lights_phong_pars_fragment>
#include <shadowmap_pars_fragment>
#include <bumpmap_pars_fragment>
#include <normalmap_pars_fragment>
#include <specularmap_pars_fragment>
#include <logdepthbuf_pars_fragment>
#include <clipping_planes_pars_fragment>
void main() {
	#include <clipping_planes_fragment>
	vec4 diffuseColor = vec4( diffuse, opacity );
	ReflectedLight reflectedLight = ReflectedLight( vec3( 0.0 ), vec3( 0.0 ), vec3( 0.0 ), vec3( 0.0 ) );
	vec3 totalEmissiveRadiance = emissive;
	#include <logdepthbuf_fragment>
	#include <map_fragment>
	#include <color_fragment>
	#include <alphamap_fragment>
	#include <alphatest_fragment>
	#include <alphahash_fragment>
	#include <specularmap_fragment>
	#include <normal_fragment_begin>
	#include <normal_fragment_maps>
	#include <emissivemap_fragment>
	#include <lights_phong_fragment>
	#include <lights_fragment_begin>
	#include <lights_fragment_maps>
	#include <lights_fragment_end>
	#include <aomap_fragment>
	vec3 outgoingLight = reflectedLight.directDiffuse + reflectedLight.indirectDiffuse + reflectedLight.directSpecular + reflectedLight.indirectSpecular + totalEmissiveRadiance;
	#include <envmap_fragment>
	#include <opaque_fragment>
	#include <tonemapping_fragment>
	#include <colorspace_fragment>
	#include <fog_fragment>
	#include <premultiplied_alpha_fragment>
	#include <dithering_fragment>
}`,Sx=`#define STANDARD
varying vec3 vViewPosition;
#ifdef USE_TRANSMISSION
	varying vec3 vWorldPosition;
#endif
#include <common>
#include <batching_pars_vertex>
#include <uv_pars_vertex>
#include <displacementmap_pars_vertex>
#include <color_pars_vertex>
#include <fog_pars_vertex>
#include <normal_pars_vertex>
#include <morphtarget_pars_vertex>
#include <skinning_pars_vertex>
#include <shadowmap_pars_vertex>
#include <logdepthbuf_pars_vertex>
#include <clipping_planes_pars_vertex>
void main() {
	#include <uv_vertex>
	#include <color_vertex>
	#include <morphcolor_vertex>
	#include <batching_vertex>
	#include <beginnormal_vertex>
	#include <morphnormal_vertex>
	#include <skinbase_vertex>
	#include <skinnormal_vertex>
	#include <defaultnormal_vertex>
	#include <normal_vertex>
	#include <begin_vertex>
	#include <morphtarget_vertex>
	#include <skinning_vertex>
	#include <displacementmap_vertex>
	#include <project_vertex>
	#include <logdepthbuf_vertex>
	#include <clipping_planes_vertex>
	vViewPosition = - mvPosition.xyz;
	#include <worldpos_vertex>
	#include <shadowmap_vertex>
	#include <fog_vertex>
#ifdef USE_TRANSMISSION
	vWorldPosition = worldPosition.xyz;
#endif
}`,Mx=`#define STANDARD
#ifdef PHYSICAL
	#define IOR
	#define USE_SPECULAR
#endif
uniform vec3 diffuse;
uniform vec3 emissive;
uniform float roughness;
uniform float metalness;
uniform float opacity;
#ifdef IOR
	uniform float ior;
#endif
#ifdef USE_SPECULAR
	uniform float specularIntensity;
	uniform vec3 specularColor;
	#ifdef USE_SPECULAR_COLORMAP
		uniform sampler2D specularColorMap;
	#endif
	#ifdef USE_SPECULAR_INTENSITYMAP
		uniform sampler2D specularIntensityMap;
	#endif
#endif
#ifdef USE_CLEARCOAT
	uniform float clearcoat;
	uniform float clearcoatRoughness;
#endif
#ifdef USE_IRIDESCENCE
	uniform float iridescence;
	uniform float iridescenceIOR;
	uniform float iridescenceThicknessMinimum;
	uniform float iridescenceThicknessMaximum;
#endif
#ifdef USE_SHEEN
	uniform vec3 sheenColor;
	uniform float sheenRoughness;
	#ifdef USE_SHEEN_COLORMAP
		uniform sampler2D sheenColorMap;
	#endif
	#ifdef USE_SHEEN_ROUGHNESSMAP
		uniform sampler2D sheenRoughnessMap;
	#endif
#endif
#ifdef USE_ANISOTROPY
	uniform vec2 anisotropyVector;
	#ifdef USE_ANISOTROPYMAP
		uniform sampler2D anisotropyMap;
	#endif
#endif
varying vec3 vViewPosition;
#include <common>
#include <packing>
#include <dithering_pars_fragment>
#include <color_pars_fragment>
#include <uv_pars_fragment>
#include <map_pars_fragment>
#include <alphamap_pars_fragment>
#include <alphatest_pars_fragment>
#include <alphahash_pars_fragment>
#include <aomap_pars_fragment>
#include <lightmap_pars_fragment>
#include <emissivemap_pars_fragment>
#include <iridescence_fragment>
#include <cube_uv_reflection_fragment>
#include <envmap_common_pars_fragment>
#include <envmap_physical_pars_fragment>
#include <fog_pars_fragment>
#include <lights_pars_begin>
#include <normal_pars_fragment>
#include <lights_physical_pars_fragment>
#include <transmission_pars_fragment>
#include <shadowmap_pars_fragment>
#include <bumpmap_pars_fragment>
#include <normalmap_pars_fragment>
#include <clearcoat_pars_fragment>
#include <iridescence_pars_fragment>
#include <roughnessmap_pars_fragment>
#include <metalnessmap_pars_fragment>
#include <logdepthbuf_pars_fragment>
#include <clipping_planes_pars_fragment>
void main() {
	#include <clipping_planes_fragment>
	vec4 diffuseColor = vec4( diffuse, opacity );
	ReflectedLight reflectedLight = ReflectedLight( vec3( 0.0 ), vec3( 0.0 ), vec3( 0.0 ), vec3( 0.0 ) );
	vec3 totalEmissiveRadiance = emissive;
	#include <logdepthbuf_fragment>
	#include <map_fragment>
	#include <color_fragment>
	#include <alphamap_fragment>
	#include <alphatest_fragment>
	#include <alphahash_fragment>
	#include <roughnessmap_fragment>
	#include <metalnessmap_fragment>
	#include <normal_fragment_begin>
	#include <normal_fragment_maps>
	#include <clearcoat_normal_fragment_begin>
	#include <clearcoat_normal_fragment_maps>
	#include <emissivemap_fragment>
	#include <lights_physical_fragment>
	#include <lights_fragment_begin>
	#include <lights_fragment_maps>
	#include <lights_fragment_end>
	#include <aomap_fragment>
	vec3 totalDiffuse = reflectedLight.directDiffuse + reflectedLight.indirectDiffuse;
	vec3 totalSpecular = reflectedLight.directSpecular + reflectedLight.indirectSpecular;
	#include <transmission_fragment>
	vec3 outgoingLight = totalDiffuse + totalSpecular + totalEmissiveRadiance;
	#ifdef USE_SHEEN
		float sheenEnergyComp = 1.0 - 0.157 * max3( material.sheenColor );
		outgoingLight = outgoingLight * sheenEnergyComp + sheenSpecularDirect + sheenSpecularIndirect;
	#endif
	#ifdef USE_CLEARCOAT
		float dotNVcc = saturate( dot( geometryClearcoatNormal, geometryViewDir ) );
		vec3 Fcc = F_Schlick( material.clearcoatF0, material.clearcoatF90, dotNVcc );
		outgoingLight = outgoingLight * ( 1.0 - material.clearcoat * Fcc ) + ( clearcoatSpecularDirect + clearcoatSpecularIndirect ) * material.clearcoat;
	#endif
	#include <opaque_fragment>
	#include <tonemapping_fragment>
	#include <colorspace_fragment>
	#include <fog_fragment>
	#include <premultiplied_alpha_fragment>
	#include <dithering_fragment>
}`,yx=`#define TOON
varying vec3 vViewPosition;
#include <common>
#include <batching_pars_vertex>
#include <uv_pars_vertex>
#include <displacementmap_pars_vertex>
#include <color_pars_vertex>
#include <fog_pars_vertex>
#include <normal_pars_vertex>
#include <morphtarget_pars_vertex>
#include <skinning_pars_vertex>
#include <shadowmap_pars_vertex>
#include <logdepthbuf_pars_vertex>
#include <clipping_planes_pars_vertex>
void main() {
	#include <uv_vertex>
	#include <color_vertex>
	#include <morphcolor_vertex>
	#include <batching_vertex>
	#include <beginnormal_vertex>
	#include <morphnormal_vertex>
	#include <skinbase_vertex>
	#include <skinnormal_vertex>
	#include <defaultnormal_vertex>
	#include <normal_vertex>
	#include <begin_vertex>
	#include <morphtarget_vertex>
	#include <skinning_vertex>
	#include <displacementmap_vertex>
	#include <project_vertex>
	#include <logdepthbuf_vertex>
	#include <clipping_planes_vertex>
	vViewPosition = - mvPosition.xyz;
	#include <worldpos_vertex>
	#include <shadowmap_vertex>
	#include <fog_vertex>
}`,Ex=`#define TOON
uniform vec3 diffuse;
uniform vec3 emissive;
uniform float opacity;
#include <common>
#include <packing>
#include <dithering_pars_fragment>
#include <color_pars_fragment>
#include <uv_pars_fragment>
#include <map_pars_fragment>
#include <alphamap_pars_fragment>
#include <alphatest_pars_fragment>
#include <alphahash_pars_fragment>
#include <aomap_pars_fragment>
#include <lightmap_pars_fragment>
#include <emissivemap_pars_fragment>
#include <gradientmap_pars_fragment>
#include <fog_pars_fragment>
#include <bsdfs>
#include <lights_pars_begin>
#include <normal_pars_fragment>
#include <lights_toon_pars_fragment>
#include <shadowmap_pars_fragment>
#include <bumpmap_pars_fragment>
#include <normalmap_pars_fragment>
#include <logdepthbuf_pars_fragment>
#include <clipping_planes_pars_fragment>
void main() {
	#include <clipping_planes_fragment>
	vec4 diffuseColor = vec4( diffuse, opacity );
	ReflectedLight reflectedLight = ReflectedLight( vec3( 0.0 ), vec3( 0.0 ), vec3( 0.0 ), vec3( 0.0 ) );
	vec3 totalEmissiveRadiance = emissive;
	#include <logdepthbuf_fragment>
	#include <map_fragment>
	#include <color_fragment>
	#include <alphamap_fragment>
	#include <alphatest_fragment>
	#include <alphahash_fragment>
	#include <normal_fragment_begin>
	#include <normal_fragment_maps>
	#include <emissivemap_fragment>
	#include <lights_toon_fragment>
	#include <lights_fragment_begin>
	#include <lights_fragment_maps>
	#include <lights_fragment_end>
	#include <aomap_fragment>
	vec3 outgoingLight = reflectedLight.directDiffuse + reflectedLight.indirectDiffuse + totalEmissiveRadiance;
	#include <opaque_fragment>
	#include <tonemapping_fragment>
	#include <colorspace_fragment>
	#include <fog_fragment>
	#include <premultiplied_alpha_fragment>
	#include <dithering_fragment>
}`,wx=`uniform float size;
uniform float scale;
#include <common>
#include <color_pars_vertex>
#include <fog_pars_vertex>
#include <morphtarget_pars_vertex>
#include <logdepthbuf_pars_vertex>
#include <clipping_planes_pars_vertex>
#ifdef USE_POINTS_UV
	varying vec2 vUv;
	uniform mat3 uvTransform;
#endif
void main() {
	#ifdef USE_POINTS_UV
		vUv = ( uvTransform * vec3( uv, 1 ) ).xy;
	#endif
	#include <color_vertex>
	#include <morphcolor_vertex>
	#include <begin_vertex>
	#include <morphtarget_vertex>
	#include <project_vertex>
	gl_PointSize = size;
	#ifdef USE_SIZEATTENUATION
		bool isPerspective = isPerspectiveMatrix( projectionMatrix );
		if ( isPerspective ) gl_PointSize *= ( scale / - mvPosition.z );
	#endif
	#include <logdepthbuf_vertex>
	#include <clipping_planes_vertex>
	#include <worldpos_vertex>
	#include <fog_vertex>
}`,Tx=`uniform vec3 diffuse;
uniform float opacity;
#include <common>
#include <color_pars_fragment>
#include <map_particle_pars_fragment>
#include <alphatest_pars_fragment>
#include <alphahash_pars_fragment>
#include <fog_pars_fragment>
#include <logdepthbuf_pars_fragment>
#include <clipping_planes_pars_fragment>
void main() {
	#include <clipping_planes_fragment>
	vec3 outgoingLight = vec3( 0.0 );
	vec4 diffuseColor = vec4( diffuse, opacity );
	#include <logdepthbuf_fragment>
	#include <map_particle_fragment>
	#include <color_fragment>
	#include <alphatest_fragment>
	#include <alphahash_fragment>
	outgoingLight = diffuseColor.rgb;
	#include <opaque_fragment>
	#include <tonemapping_fragment>
	#include <colorspace_fragment>
	#include <fog_fragment>
	#include <premultiplied_alpha_fragment>
}`,Ax=`#include <common>
#include <batching_pars_vertex>
#include <fog_pars_vertex>
#include <morphtarget_pars_vertex>
#include <skinning_pars_vertex>
#include <logdepthbuf_pars_vertex>
#include <shadowmap_pars_vertex>
void main() {
	#include <batching_vertex>
	#include <beginnormal_vertex>
	#include <morphnormal_vertex>
	#include <skinbase_vertex>
	#include <skinnormal_vertex>
	#include <defaultnormal_vertex>
	#include <begin_vertex>
	#include <morphtarget_vertex>
	#include <skinning_vertex>
	#include <project_vertex>
	#include <logdepthbuf_vertex>
	#include <worldpos_vertex>
	#include <shadowmap_vertex>
	#include <fog_vertex>
}`,Cx=`uniform vec3 color;
uniform float opacity;
#include <common>
#include <packing>
#include <fog_pars_fragment>
#include <bsdfs>
#include <lights_pars_begin>
#include <logdepthbuf_pars_fragment>
#include <shadowmap_pars_fragment>
#include <shadowmask_pars_fragment>
void main() {
	#include <logdepthbuf_fragment>
	gl_FragColor = vec4( color, opacity * ( 1.0 - getShadowMask() ) );
	#include <tonemapping_fragment>
	#include <colorspace_fragment>
	#include <fog_fragment>
}`,Rx=`uniform float rotation;
uniform vec2 center;
#include <common>
#include <uv_pars_vertex>
#include <fog_pars_vertex>
#include <logdepthbuf_pars_vertex>
#include <clipping_planes_pars_vertex>
void main() {
	#include <uv_vertex>
	vec4 mvPosition = modelViewMatrix * vec4( 0.0, 0.0, 0.0, 1.0 );
	vec2 scale;
	scale.x = length( vec3( modelMatrix[ 0 ].x, modelMatrix[ 0 ].y, modelMatrix[ 0 ].z ) );
	scale.y = length( vec3( modelMatrix[ 1 ].x, modelMatrix[ 1 ].y, modelMatrix[ 1 ].z ) );
	#ifndef USE_SIZEATTENUATION
		bool isPerspective = isPerspectiveMatrix( projectionMatrix );
		if ( isPerspective ) scale *= - mvPosition.z;
	#endif
	vec2 alignedPosition = ( position.xy - ( center - vec2( 0.5 ) ) ) * scale;
	vec2 rotatedPosition;
	rotatedPosition.x = cos( rotation ) * alignedPosition.x - sin( rotation ) * alignedPosition.y;
	rotatedPosition.y = sin( rotation ) * alignedPosition.x + cos( rotation ) * alignedPosition.y;
	mvPosition.xy += rotatedPosition;
	gl_Position = projectionMatrix * mvPosition;
	#include <logdepthbuf_vertex>
	#include <clipping_planes_vertex>
	#include <fog_vertex>
}`,bx=`uniform vec3 diffuse;
uniform float opacity;
#include <common>
#include <uv_pars_fragment>
#include <map_pars_fragment>
#include <alphamap_pars_fragment>
#include <alphatest_pars_fragment>
#include <alphahash_pars_fragment>
#include <fog_pars_fragment>
#include <logdepthbuf_pars_fragment>
#include <clipping_planes_pars_fragment>
void main() {
	#include <clipping_planes_fragment>
	vec3 outgoingLight = vec3( 0.0 );
	vec4 diffuseColor = vec4( diffuse, opacity );
	#include <logdepthbuf_fragment>
	#include <map_fragment>
	#include <alphamap_fragment>
	#include <alphatest_fragment>
	#include <alphahash_fragment>
	outgoingLight = diffuseColor.rgb;
	#include <opaque_fragment>
	#include <tonemapping_fragment>
	#include <colorspace_fragment>
	#include <fog_fragment>
}`,ee={alphahash_fragment:Km,alphahash_pars_fragment:Jm,alphamap_fragment:Qm,alphamap_pars_fragment:tg,alphatest_fragment:eg,alphatest_pars_fragment:ng,aomap_fragment:ig,aomap_pars_fragment:sg,batching_pars_vertex:rg,batching_vertex:ag,begin_vertex:og,beginnormal_vertex:cg,bsdfs:lg,iridescence_fragment:hg,bumpmap_pars_fragment:ug,clipping_planes_fragment:dg,clipping_planes_pars_fragment:fg,clipping_planes_pars_vertex:pg,clipping_planes_vertex:mg,color_fragment:gg,color_pars_fragment:xg,color_pars_vertex:_g,color_vertex:vg,common:Sg,cube_uv_reflection_fragment:Mg,defaultnormal_vertex:yg,displacementmap_pars_vertex:Eg,displacementmap_vertex:wg,emissivemap_fragment:Tg,emissivemap_pars_fragment:Ag,colorspace_fragment:Cg,colorspace_pars_fragment:Rg,envmap_fragment:bg,envmap_common_pars_fragment:Pg,envmap_pars_fragment:Lg,envmap_pars_vertex:Dg,envmap_physical_pars_fragment:Wg,envmap_vertex:Ig,fog_vertex:Ng,fog_pars_vertex:Fg,fog_fragment:Bg,fog_pars_fragment:Ug,gradientmap_pars_fragment:zg,lightmap_fragment:Og,lightmap_pars_fragment:Gg,lights_lambert_fragment:Vg,lights_lambert_pars_fragment:kg,lights_pars_begin:Hg,lights_toon_fragment:Xg,lights_toon_pars_fragment:Yg,lights_phong_fragment:qg,lights_phong_pars_fragment:Zg,lights_physical_fragment:jg,lights_physical_pars_fragment:$g,lights_fragment_begin:Kg,lights_fragment_maps:Jg,lights_fragment_end:Qg,logdepthbuf_fragment:t0,logdepthbuf_pars_fragment:e0,logdepthbuf_pars_vertex:n0,logdepthbuf_vertex:i0,map_fragment:s0,map_pars_fragment:r0,map_particle_fragment:a0,map_particle_pars_fragment:o0,metalnessmap_fragment:c0,metalnessmap_pars_fragment:l0,morphcolor_vertex:h0,morphnormal_vertex:u0,morphtarget_pars_vertex:d0,morphtarget_vertex:f0,normal_fragment_begin:p0,normal_fragment_maps:m0,normal_pars_fragment:g0,normal_pars_vertex:x0,normal_vertex:_0,normalmap_pars_fragment:v0,clearcoat_normal_fragment_begin:S0,clearcoat_normal_fragment_maps:M0,clearcoat_pars_fragment:y0,iridescence_pars_fragment:E0,opaque_fragment:w0,packing:T0,premultiplied_alpha_fragment:A0,project_vertex:C0,dithering_fragment:R0,dithering_pars_fragment:b0,roughnessmap_fragment:P0,roughnessmap_pars_fragment:L0,shadowmap_pars_fragment:D0,shadowmap_pars_vertex:I0,shadowmap_vertex:N0,shadowmask_pars_fragment:F0,skinbase_vertex:B0,skinning_pars_vertex:U0,skinning_vertex:z0,skinnormal_vertex:O0,specularmap_fragment:G0,specularmap_pars_fragment:V0,tonemapping_fragment:k0,tonemapping_pars_fragment:H0,transmission_fragment:W0,transmission_pars_fragment:X0,uv_pars_fragment:Y0,uv_pars_vertex:q0,uv_vertex:Z0,worldpos_vertex:j0,background_vert:$0,background_frag:K0,backgroundCube_vert:J0,backgroundCube_frag:Q0,cube_vert:tx,cube_frag:ex,depth_vert:nx,depth_frag:ix,distanceRGBA_vert:sx,distanceRGBA_frag:rx,equirect_vert:ax,equirect_frag:ox,linedashed_vert:cx,linedashed_frag:lx,meshbasic_vert:hx,meshbasic_frag:ux,meshlambert_vert:dx,meshlambert_frag:fx,meshmatcap_vert:px,meshmatcap_frag:mx,meshnormal_vert:gx,meshnormal_frag:xx,meshphong_vert:_x,meshphong_frag:vx,meshphysical_vert:Sx,meshphysical_frag:Mx,meshtoon_vert:yx,meshtoon_frag:Ex,points_vert:wx,points_frag:Tx,shadow_vert:Ax,shadow_frag:Cx,sprite_vert:Rx,sprite_frag:bx},pt={common:{diffuse:{value:new Qt(16777215)},opacity:{value:1},map:{value:null},mapTransform:{value:new ne},alphaMap:{value:null},alphaMapTransform:{value:new ne},alphaTest:{value:0}},specularmap:{specularMap:{value:null},specularMapTransform:{value:new ne}},envmap:{envMap:{value:null},flipEnvMap:{value:-1},reflectivity:{value:1},ior:{value:1.5},refractionRatio:{value:.98}},aomap:{aoMap:{value:null},aoMapIntensity:{value:1},aoMapTransform:{value:new ne}},lightmap:{lightMap:{value:null},lightMapIntensity:{value:1},lightMapTransform:{value:new ne}},bumpmap:{bumpMap:{value:null},bumpMapTransform:{value:new ne},bumpScale:{value:1}},normalmap:{normalMap:{value:null},normalMapTransform:{value:new ne},normalScale:{value:new Mt(1,1)}},displacementmap:{displacementMap:{value:null},displacementMapTransform:{value:new ne},displacementScale:{value:1},displacementBias:{value:0}},emissivemap:{emissiveMap:{value:null},emissiveMapTransform:{value:new ne}},metalnessmap:{metalnessMap:{value:null},metalnessMapTransform:{value:new ne}},roughnessmap:{roughnessMap:{value:null},roughnessMapTransform:{value:new ne}},gradientmap:{gradientMap:{value:null}},fog:{fogDensity:{value:25e-5},fogNear:{value:1},fogFar:{value:2e3},fogColor:{value:new Qt(16777215)}},lights:{ambientLightColor:{value:[]},lightProbe:{value:[]},directionalLights:{value:[],properties:{direction:{},color:{}}},directionalLightShadows:{value:[],properties:{shadowBias:{},shadowNormalBias:{},shadowRadius:{},shadowMapSize:{}}},directionalShadowMap:{value:[]},directionalShadowMatrix:{value:[]},spotLights:{value:[],properties:{color:{},position:{},direction:{},distance:{},coneCos:{},penumbraCos:{},decay:{}}},spotLightShadows:{value:[],properties:{shadowBias:{},shadowNormalBias:{},shadowRadius:{},shadowMapSize:{}}},spotLightMap:{value:[]},spotShadowMap:{value:[]},spotLightMatrix:{value:[]},pointLights:{value:[],properties:{color:{},position:{},decay:{},distance:{}}},pointLightShadows:{value:[],properties:{shadowBias:{},shadowNormalBias:{},shadowRadius:{},shadowMapSize:{},shadowCameraNear:{},shadowCameraFar:{}}},pointShadowMap:{value:[]},pointShadowMatrix:{value:[]},hemisphereLights:{value:[],properties:{direction:{},skyColor:{},groundColor:{}}},rectAreaLights:{value:[],properties:{color:{},position:{},width:{},height:{}}},ltc_1:{value:null},ltc_2:{value:null}},points:{diffuse:{value:new Qt(16777215)},opacity:{value:1},size:{value:1},scale:{value:1},map:{value:null},alphaMap:{value:null},alphaMapTransform:{value:new ne},alphaTest:{value:0},uvTransform:{value:new ne}},sprite:{diffuse:{value:new Qt(16777215)},opacity:{value:1},center:{value:new Mt(.5,.5)},rotation:{value:0},map:{value:null},mapTransform:{value:new ne},alphaMap:{value:null},alphaMapTransform:{value:new ne},alphaTest:{value:0}}},fi={basic:{uniforms:gn([pt.common,pt.specularmap,pt.envmap,pt.aomap,pt.lightmap,pt.fog]),vertexShader:ee.meshbasic_vert,fragmentShader:ee.meshbasic_frag},lambert:{uniforms:gn([pt.common,pt.specularmap,pt.envmap,pt.aomap,pt.lightmap,pt.emissivemap,pt.bumpmap,pt.normalmap,pt.displacementmap,pt.fog,pt.lights,{emissive:{value:new Qt(0)}}]),vertexShader:ee.meshlambert_vert,fragmentShader:ee.meshlambert_frag},phong:{uniforms:gn([pt.common,pt.specularmap,pt.envmap,pt.aomap,pt.lightmap,pt.emissivemap,pt.bumpmap,pt.normalmap,pt.displacementmap,pt.fog,pt.lights,{emissive:{value:new Qt(0)},specular:{value:new Qt(1118481)},shininess:{value:30}}]),vertexShader:ee.meshphong_vert,fragmentShader:ee.meshphong_frag},standard:{uniforms:gn([pt.common,pt.envmap,pt.aomap,pt.lightmap,pt.emissivemap,pt.bumpmap,pt.normalmap,pt.displacementmap,pt.roughnessmap,pt.metalnessmap,pt.fog,pt.lights,{emissive:{value:new Qt(0)},roughness:{value:1},metalness:{value:0},envMapIntensity:{value:1}}]),vertexShader:ee.meshphysical_vert,fragmentShader:ee.meshphysical_frag},toon:{uniforms:gn([pt.common,pt.aomap,pt.lightmap,pt.emissivemap,pt.bumpmap,pt.normalmap,pt.displacementmap,pt.gradientmap,pt.fog,pt.lights,{emissive:{value:new Qt(0)}}]),vertexShader:ee.meshtoon_vert,fragmentShader:ee.meshtoon_frag},matcap:{uniforms:gn([pt.common,pt.bumpmap,pt.normalmap,pt.displacementmap,pt.fog,{matcap:{value:null}}]),vertexShader:ee.meshmatcap_vert,fragmentShader:ee.meshmatcap_frag},points:{uniforms:gn([pt.points,pt.fog]),vertexShader:ee.points_vert,fragmentShader:ee.points_frag},dashed:{uniforms:gn([pt.common,pt.fog,{scale:{value:1},dashSize:{value:1},totalSize:{value:2}}]),vertexShader:ee.linedashed_vert,fragmentShader:ee.linedashed_frag},depth:{uniforms:gn([pt.common,pt.displacementmap]),vertexShader:ee.depth_vert,fragmentShader:ee.depth_frag},normal:{uniforms:gn([pt.common,pt.bumpmap,pt.normalmap,pt.displacementmap,{opacity:{value:1}}]),vertexShader:ee.meshnormal_vert,fragmentShader:ee.meshnormal_frag},sprite:{uniforms:gn([pt.sprite,pt.fog]),vertexShader:ee.sprite_vert,fragmentShader:ee.sprite_frag},background:{uniforms:{uvTransform:{value:new ne},t2D:{value:null},backgroundIntensity:{value:1}},vertexShader:ee.background_vert,fragmentShader:ee.background_frag},backgroundCube:{uniforms:{envMap:{value:null},flipEnvMap:{value:-1},backgroundBlurriness:{value:0},backgroundIntensity:{value:1}},vertexShader:ee.backgroundCube_vert,fragmentShader:ee.backgroundCube_frag},cube:{uniforms:{tCube:{value:null},tFlip:{value:-1},opacity:{value:1}},vertexShader:ee.cube_vert,fragmentShader:ee.cube_frag},equirect:{uniforms:{tEquirect:{value:null}},vertexShader:ee.equirect_vert,fragmentShader:ee.equirect_frag},distanceRGBA:{uniforms:gn([pt.common,pt.displacementmap,{referencePosition:{value:new R},nearDistance:{value:1},farDistance:{value:1e3}}]),vertexShader:ee.distanceRGBA_vert,fragmentShader:ee.distanceRGBA_frag},shadow:{uniforms:gn([pt.lights,pt.fog,{color:{value:new Qt(0)},opacity:{value:1}}]),vertexShader:ee.shadow_vert,fragmentShader:ee.shadow_frag}};fi.physical={uniforms:gn([fi.standard.uniforms,{clearcoat:{value:0},clearcoatMap:{value:null},clearcoatMapTransform:{value:new ne},clearcoatNormalMap:{value:null},clearcoatNormalMapTransform:{value:new ne},clearcoatNormalScale:{value:new Mt(1,1)},clearcoatRoughness:{value:0},clearcoatRoughnessMap:{value:null},clearcoatRoughnessMapTransform:{value:new ne},iridescence:{value:0},iridescenceMap:{value:null},iridescenceMapTransform:{value:new ne},iridescenceIOR:{value:1.3},iridescenceThicknessMinimum:{value:100},iridescenceThicknessMaximum:{value:400},iridescenceThicknessMap:{value:null},iridescenceThicknessMapTransform:{value:new ne},sheen:{value:0},sheenColor:{value:new Qt(0)},sheenColorMap:{value:null},sheenColorMapTransform:{value:new ne},sheenRoughness:{value:1},sheenRoughnessMap:{value:null},sheenRoughnessMapTransform:{value:new ne},transmission:{value:0},transmissionMap:{value:null},transmissionMapTransform:{value:new ne},transmissionSamplerSize:{value:new Mt},transmissionSamplerMap:{value:null},thickness:{value:0},thicknessMap:{value:null},thicknessMapTransform:{value:new ne},attenuationDistance:{value:0},attenuationColor:{value:new Qt(0)},specularColor:{value:new Qt(1,1,1)},specularColorMap:{value:null},specularColorMapTransform:{value:new ne},specularIntensity:{value:1},specularIntensityMap:{value:null},specularIntensityMapTransform:{value:new ne},anisotropyVector:{value:new Mt},anisotropyMap:{value:null},anisotropyMapTransform:{value:new ne}}]),vertexShader:ee.meshphysical_vert,fragmentShader:ee.meshphysical_frag};const io={r:0,b:0,g:0};function Px(r,t,e,n,i,s,a){const o=new Qt(0);let c=s===!0?0:1,l,h,u=null,d=0,f=null;function g(m,p){let _=!1,v=p.isScene===!0?p.background:null;v&&v.isTexture&&(v=(p.backgroundBlurriness>0?e:t).get(v)),v===null?x(o,c):v&&v.isColor&&(x(v,1),_=!0);const S=r.xr.getEnvironmentBlendMode();S==="additive"?n.buffers.color.setClear(0,0,0,1,a):S==="alpha-blend"&&n.buffers.color.setClear(0,0,0,0,a),(r.autoClear||_)&&r.clear(r.autoClearColor,r.autoClearDepth,r.autoClearStencil),v&&(v.isCubeTexture||v.mapping===306)?(h===void 0&&(h=new jt(new Dn(1,1,1),new Bn({name:"BackgroundCubeMaterial",uniforms:Sr(fi.backgroundCube.uniforms),vertexShader:fi.backgroundCube.vertexShader,fragmentShader:fi.backgroundCube.fragmentShader,side:1,depthTest:!1,depthWrite:!1,fog:!1})),h.geometry.deleteAttribute("normal"),h.geometry.deleteAttribute("uv"),h.onBeforeRender=function(M,y,w){this.matrixWorld.copyPosition(w.matrixWorld)},Object.defineProperty(h.material,"envMap",{get:function(){return this.uniforms.envMap.value}}),i.update(h)),h.material.uniforms.envMap.value=v,h.material.uniforms.flipEnvMap.value=v.isCubeTexture&&v.isRenderTargetTexture===!1?-1:1,h.material.uniforms.backgroundBlurriness.value=p.backgroundBlurriness,h.material.uniforms.backgroundIntensity.value=p.backgroundIntensity,h.material.toneMapped=pe.getTransfer(v.colorSpace)!==ye,(u!==v||d!==v.version||f!==r.toneMapping)&&(h.material.needsUpdate=!0,u=v,d=v.version,f=r.toneMapping),h.layers.enableAll(),m.unshift(h,h.geometry,h.material,0,0,null)):v&&v.isTexture&&(l===void 0&&(l=new jt(new pc(2,2),new Bn({name:"BackgroundMaterial",uniforms:Sr(fi.background.uniforms),vertexShader:fi.background.vertexShader,fragmentShader:fi.background.fragmentShader,side:0,depthTest:!1,depthWrite:!1,fog:!1})),l.geometry.deleteAttribute("normal"),Object.defineProperty(l.material,"map",{get:function(){return this.uniforms.t2D.value}}),i.update(l)),l.material.uniforms.t2D.value=v,l.material.uniforms.backgroundIntensity.value=p.backgroundIntensity,l.material.toneMapped=pe.getTransfer(v.colorSpace)!==ye,v.matrixAutoUpdate===!0&&v.updateMatrix(),l.material.uniforms.uvTransform.value.copy(v.matrix),(u!==v||d!==v.version||f!==r.toneMapping)&&(l.material.needsUpdate=!0,u=v,d=v.version,f=r.toneMapping),l.layers.enableAll(),m.unshift(l,l.geometry,l.material,0,0,null))}function x(m,p){m.getRGB(io,Wf(r)),n.buffers.color.setClear(io.r,io.g,io.b,p,a)}return{getClearColor:function(){return o},setClearColor:function(m,p=1){o.set(m),c=p,x(o,c)},getClearAlpha:function(){return c},setClearAlpha:function(m){c=m,x(o,c)},render:g}}function Lx(r,t,e,n){const i=r.getParameter(r.MAX_VERTEX_ATTRIBS),s=n.isWebGL2?null:t.get("OES_vertex_array_object"),a=n.isWebGL2||s!==null,o={},c=m(null);let l=c,h=!1;function u(D,N,B,G,z){let H=!1;if(a){const j=x(G,B,N);l!==j&&(l=j,f(l.object)),H=p(D,G,B,z),H&&_(D,G,B,z)}else{const j=N.wireframe===!0;(l.geometry!==G.id||l.program!==B.id||l.wireframe!==j)&&(l.geometry=G.id,l.program=B.id,l.wireframe=j,H=!0)}z!==null&&e.update(z,r.ELEMENT_ARRAY_BUFFER),(H||h)&&(h=!1,A(D,N,B,G),z!==null&&r.bindBuffer(r.ELEMENT_ARRAY_BUFFER,e.get(z).buffer))}function d(){return n.isWebGL2?r.createVertexArray():s.createVertexArrayOES()}function f(D){return n.isWebGL2?r.bindVertexArray(D):s.bindVertexArrayOES(D)}function g(D){return n.isWebGL2?r.deleteVertexArray(D):s.deleteVertexArrayOES(D)}function x(D,N,B){const G=B.wireframe===!0;let z=o[D.id];z===void 0&&(z={},o[D.id]=z);let H=z[N.id];H===void 0&&(H={},z[N.id]=H);let j=H[G];return j===void 0&&(j=m(d()),H[G]=j),j}function m(D){const N=[],B=[],G=[];for(let z=0;z<i;z++)N[z]=0,B[z]=0,G[z]=0;return{geometry:null,program:null,wireframe:!1,newAttributes:N,enabledAttributes:B,attributeDivisors:G,object:D,attributes:{},index:null}}function p(D,N,B,G){const z=l.attributes,H=N.attributes;let j=0;const $=B.getAttributes();for(const Q in $)if($[Q].location>=0){const K=z[Q];let nt=H[Q];if(nt===void 0&&(Q==="instanceMatrix"&&D.instanceMatrix&&(nt=D.instanceMatrix),Q==="instanceColor"&&D.instanceColor&&(nt=D.instanceColor)),K===void 0||K.attribute!==nt||nt&&K.data!==nt.data)return!0;j++}return l.attributesNum!==j||l.index!==G}function _(D,N,B,G){const z={},H=N.attributes;let j=0;const $=B.getAttributes();for(const Q in $)if($[Q].location>=0){let K=H[Q];K===void 0&&(Q==="instanceMatrix"&&D.instanceMatrix&&(K=D.instanceMatrix),Q==="instanceColor"&&D.instanceColor&&(K=D.instanceColor));const nt={};nt.attribute=K,K&&K.data&&(nt.data=K.data),z[Q]=nt,j++}l.attributes=z,l.attributesNum=j,l.index=G}function v(){const D=l.newAttributes;for(let N=0,B=D.length;N<B;N++)D[N]=0}function S(D){M(D,0)}function M(D,N){const B=l.newAttributes,G=l.enabledAttributes,z=l.attributeDivisors;B[D]=1,G[D]===0&&(r.enableVertexAttribArray(D),G[D]=1),z[D]!==N&&((n.isWebGL2?r:t.get("ANGLE_instanced_arrays"))[n.isWebGL2?"vertexAttribDivisor":"vertexAttribDivisorANGLE"](D,N),z[D]=N)}function y(){const D=l.newAttributes,N=l.enabledAttributes;for(let B=0,G=N.length;B<G;B++)N[B]!==D[B]&&(r.disableVertexAttribArray(B),N[B]=0)}function w(D,N,B,G,z,H,j){j===!0?r.vertexAttribIPointer(D,N,B,z,H):r.vertexAttribPointer(D,N,B,G,z,H)}function A(D,N,B,G){if(n.isWebGL2===!1&&(D.isInstancedMesh||G.isInstancedBufferGeometry)&&t.get("ANGLE_instanced_arrays")===null)return;v();const z=G.attributes,H=B.getAttributes(),j=N.defaultAttributeValues;for(const $ in H){const Q=H[$];if(Q.location>=0){let V=z[$];if(V===void 0&&($==="instanceMatrix"&&D.instanceMatrix&&(V=D.instanceMatrix),$==="instanceColor"&&D.instanceColor&&(V=D.instanceColor)),V!==void 0){const K=V.normalized,nt=V.itemSize,rt=e.get(V);if(rt===void 0)continue;const ot=rt.buffer,_t=rt.type,ht=rt.bytesPerElement,ut=n.isWebGL2===!0&&(_t===r.INT||_t===r.UNSIGNED_INT||V.gpuType===1013);if(V.isInterleavedBufferAttribute){const Ct=V.data,W=Ct.stride,Nt=V.offset;if(Ct.isInstancedInterleavedBuffer){for(let Tt=0;Tt<Q.locationSize;Tt++)M(Q.location+Tt,Ct.meshPerAttribute);D.isInstancedMesh!==!0&&G._maxInstanceCount===void 0&&(G._maxInstanceCount=Ct.meshPerAttribute*Ct.count)}else for(let Tt=0;Tt<Q.locationSize;Tt++)S(Q.location+Tt);r.bindBuffer(r.ARRAY_BUFFER,ot);for(let Tt=0;Tt<Q.locationSize;Tt++)w(Q.location+Tt,nt/Q.locationSize,_t,K,W*ht,(Nt+nt/Q.locationSize*Tt)*ht,ut)}else{if(V.isInstancedBufferAttribute){for(let Ct=0;Ct<Q.locationSize;Ct++)M(Q.location+Ct,V.meshPerAttribute);D.isInstancedMesh!==!0&&G._maxInstanceCount===void 0&&(G._maxInstanceCount=V.meshPerAttribute*V.count)}else for(let Ct=0;Ct<Q.locationSize;Ct++)S(Q.location+Ct);r.bindBuffer(r.ARRAY_BUFFER,ot);for(let Ct=0;Ct<Q.locationSize;Ct++)w(Q.location+Ct,nt/Q.locationSize,_t,K,nt*ht,nt/Q.locationSize*Ct*ht,ut)}}else if(j!==void 0){const K=j[$];if(K!==void 0)switch(K.length){case 2:r.vertexAttrib2fv(Q.location,K);break;case 3:r.vertexAttrib3fv(Q.location,K);break;case 4:r.vertexAttrib4fv(Q.location,K);break;default:r.vertexAttrib1fv(Q.location,K)}}}}y()}function E(){C();for(const D in o){const N=o[D];for(const B in N){const G=N[B];for(const z in G)g(G[z].object),delete G[z];delete N[B]}delete o[D]}}function T(D){if(o[D.id]===void 0)return;const N=o[D.id];for(const B in N){const G=N[B];for(const z in G)g(G[z].object),delete G[z];delete N[B]}delete o[D.id]}function L(D){for(const N in o){const B=o[N];if(B[D.id]===void 0)continue;const G=B[D.id];for(const z in G)g(G[z].object),delete G[z];delete B[D.id]}}function C(){F(),h=!0,l!==c&&(l=c,f(l.object))}function F(){c.geometry=null,c.program=null,c.wireframe=!1}return{setup:u,reset:C,resetDefaultState:F,dispose:E,releaseStatesOfGeometry:T,releaseStatesOfProgram:L,initAttributes:v,enableAttribute:S,disableUnusedAttributes:y}}function Dx(r,t,e,n){const i=n.isWebGL2;let s;function a(h){s=h}function o(h,u){r.drawArrays(s,h,u),e.update(u,s,1)}function c(h,u,d){if(d===0)return;let f,g;if(i)f=r,g="drawArraysInstanced";else if(f=t.get("ANGLE_instanced_arrays"),g="drawArraysInstancedANGLE",f===null){console.error("THREE.WebGLBufferRenderer: using THREE.InstancedBufferGeometry but hardware does not support extension ANGLE_instanced_arrays.");return}f[g](s,h,u,d),e.update(u,s,d)}function l(h,u,d){if(d===0)return;const f=t.get("WEBGL_multi_draw");if(f===null)for(let g=0;g<d;g++)this.render(h[g],u[g]);else{f.multiDrawArraysWEBGL(s,h,0,u,0,d);let g=0;for(let x=0;x<d;x++)g+=u[x];e.update(g,s,1)}}this.setMode=a,this.render=o,this.renderInstances=c,this.renderMultiDraw=l}function Ix(r,t,e){let n;function i(){if(n!==void 0)return n;if(t.has("EXT_texture_filter_anisotropic")===!0){const w=t.get("EXT_texture_filter_anisotropic");n=r.getParameter(w.MAX_TEXTURE_MAX_ANISOTROPY_EXT)}else n=0;return n}function s(w){if(w==="highp"){if(r.getShaderPrecisionFormat(r.VERTEX_SHADER,r.HIGH_FLOAT).precision>0&&r.getShaderPrecisionFormat(r.FRAGMENT_SHADER,r.HIGH_FLOAT).precision>0)return"highp";w="mediump"}return w==="mediump"&&r.getShaderPrecisionFormat(r.VERTEX_SHADER,r.MEDIUM_FLOAT).precision>0&&r.getShaderPrecisionFormat(r.FRAGMENT_SHADER,r.MEDIUM_FLOAT).precision>0?"mediump":"lowp"}const a=typeof WebGL2RenderingContext<"u"&&r.constructor.name==="WebGL2RenderingContext";let o=e.precision!==void 0?e.precision:"highp";const c=s(o);c!==o&&(console.warn("THREE.WebGLRenderer:",o,"not supported, using",c,"instead."),o=c);const l=a||t.has("WEBGL_draw_buffers"),h=e.logarithmicDepthBuffer===!0,u=r.getParameter(r.MAX_TEXTURE_IMAGE_UNITS),d=r.getParameter(r.MAX_VERTEX_TEXTURE_IMAGE_UNITS),f=r.getParameter(r.MAX_TEXTURE_SIZE),g=r.getParameter(r.MAX_CUBE_MAP_TEXTURE_SIZE),x=r.getParameter(r.MAX_VERTEX_ATTRIBS),m=r.getParameter(r.MAX_VERTEX_UNIFORM_VECTORS),p=r.getParameter(r.MAX_VARYING_VECTORS),_=r.getParameter(r.MAX_FRAGMENT_UNIFORM_VECTORS),v=d>0,S=a||t.has("OES_texture_float"),M=v&&S,y=a?r.getParameter(r.MAX_SAMPLES):0;return{isWebGL2:a,drawBuffers:l,getMaxAnisotropy:i,getMaxPrecision:s,precision:o,logarithmicDepthBuffer:h,maxTextures:u,maxVertexTextures:d,maxTextureSize:f,maxCubemapSize:g,maxAttributes:x,maxVertexUniforms:m,maxVaryings:p,maxFragmentUniforms:_,vertexTextures:v,floatFragmentTextures:S,floatVertexTextures:M,maxSamples:y}}function Nx(r){const t=this;let e=null,n=0,i=!1,s=!1;const a=new Ii,o=new ne,c={value:null,needsUpdate:!1};this.uniform=c,this.numPlanes=0,this.numIntersection=0,this.init=function(u,d){const f=u.length!==0||d||n!==0||i;return i=d,n=u.length,f},this.beginShadows=function(){s=!0,h(null)},this.endShadows=function(){s=!1},this.setGlobalState=function(u,d){e=h(u,d,0)},this.setState=function(u,d,f){const g=u.clippingPlanes,x=u.clipIntersection,m=u.clipShadows,p=r.get(u);if(!i||g===null||g.length===0||s&&!m)s?h(null):l();else{const _=s?0:n,v=_*4;let S=p.clippingState||null;c.value=S,S=h(g,d,v,f);for(let M=0;M!==v;++M)S[M]=e[M];p.clippingState=S,this.numIntersection=x?this.numPlanes:0,this.numPlanes+=_}};function l(){c.value!==e&&(c.value=e,c.needsUpdate=n>0),t.numPlanes=n,t.numIntersection=0}function h(u,d,f,g){const x=u!==null?u.length:0;let m=null;if(x!==0){if(m=c.value,g!==!0||m===null){const p=f+x*4,_=d.matrixWorldInverse;o.getNormalMatrix(_),(m===null||m.length<p)&&(m=new Float32Array(p));for(let v=0,S=f;v!==x;++v,S+=4)a.copy(u[v]).applyMatrix4(_,o),a.normal.toArray(m,S),m[S+3]=a.constant}c.value=m,c.needsUpdate=!0}return t.numPlanes=x,t.numIntersection=0,m}}function Fx(r){let t=new WeakMap;function e(a,o){return o===303?a.mapping=301:o===304&&(a.mapping=302),a}function n(a){if(a&&a.isTexture){const o=a.mapping;if(o===303||o===304)if(t.has(a)){const c=t.get(a).texture;return e(c,a.mapping)}else{const c=a.image;if(c&&c.height>0){const l=new qm(c.height/2);return l.fromEquirectangularTexture(r,a),t.set(a,l),a.addEventListener("dispose",i),e(l.texture,a.mapping)}else return null}}return a}function i(a){const o=a.target;o.removeEventListener("dispose",i);const c=t.get(o);c!==void 0&&(t.delete(o),c.dispose())}function s(){t=new WeakMap}return{get:n,dispose:s}}class Uh extends Xf{constructor(t=-1,e=1,n=1,i=-1,s=.1,a=2e3){super(),this.isOrthographicCamera=!0,this.type="OrthographicCamera",this.zoom=1,this.view=null,this.left=t,this.right=e,this.top=n,this.bottom=i,this.near=s,this.far=a,this.updateProjectionMatrix()}copy(t,e){return super.copy(t,e),this.left=t.left,this.right=t.right,this.top=t.top,this.bottom=t.bottom,this.near=t.near,this.far=t.far,this.zoom=t.zoom,this.view=t.view===null?null:Object.assign({},t.view),this}setViewOffset(t,e,n,i,s,a){this.view===null&&(this.view={enabled:!0,fullWidth:1,fullHeight:1,offsetX:0,offsetY:0,width:1,height:1}),this.view.enabled=!0,this.view.fullWidth=t,this.view.fullHeight=e,this.view.offsetX=n,this.view.offsetY=i,this.view.width=s,this.view.height=a,this.updateProjectionMatrix()}clearViewOffset(){this.view!==null&&(this.view.enabled=!1),this.updateProjectionMatrix()}updateProjectionMatrix(){const t=(this.right-this.left)/(2*this.zoom),e=(this.top-this.bottom)/(2*this.zoom),n=(this.right+this.left)/2,i=(this.top+this.bottom)/2;let s=n-t,a=n+t,o=i+e,c=i-e;if(this.view!==null&&this.view.enabled){const l=(this.right-this.left)/this.view.fullWidth/this.zoom,h=(this.top-this.bottom)/this.view.fullHeight/this.zoom;s+=l*this.view.offsetX,a=s+l*this.view.width,o-=h*this.view.offsetY,c=o-h*this.view.height}this.projectionMatrix.makeOrthographic(s,a,o,c,this.near,this.far,this.coordinateSystem),this.projectionMatrixInverse.copy(this.projectionMatrix).invert()}toJSON(t){const e=super.toJSON(t);return e.object.zoom=this.zoom,e.object.left=this.left,e.object.right=this.right,e.object.top=this.top,e.object.bottom=this.bottom,e.object.near=this.near,e.object.far=this.far,this.view!==null&&(e.object.view=Object.assign({},this.view)),e}}const hr=4,Tu=[.125,.215,.35,.446,.526,.582],Es=20,Yc=new Uh,Au=new Qt;let qc=null,Zc=0,jc=0;const Ss=(1+Math.sqrt(5))/2,qs=1/Ss,Cu=[new R(1,1,1),new R(-1,1,1),new R(1,1,-1),new R(-1,1,-1),new R(0,Ss,qs),new R(0,Ss,-qs),new R(qs,0,Ss),new R(-qs,0,Ss),new R(Ss,qs,0),new R(-Ss,qs,0)];class Ru{constructor(t){this._renderer=t,this._pingPongRenderTarget=null,this._lodMax=0,this._cubeSize=0,this._lodPlanes=[],this._sizeLods=[],this._sigmas=[],this._blurMaterial=null,this._cubemapMaterial=null,this._equirectMaterial=null,this._compileMaterial(this._blurMaterial)}fromScene(t,e=0,n=.1,i=100){qc=this._renderer.getRenderTarget(),Zc=this._renderer.getActiveCubeFace(),jc=this._renderer.getActiveMipmapLevel(),this._setSize(256);const s=this._allocateTargets();return s.depthBuffer=!0,this._sceneToCubeUV(t,n,i,s),e>0&&this._blur(s,0,0,e),this._applyPMREM(s),this._cleanup(s),s}fromEquirectangular(t,e=null){return this._fromTexture(t,e)}fromCubemap(t,e=null){return this._fromTexture(t,e)}compileCubemapShader(){this._cubemapMaterial===null&&(this._cubemapMaterial=Lu(),this._compileMaterial(this._cubemapMaterial))}compileEquirectangularShader(){this._equirectMaterial===null&&(this._equirectMaterial=Pu(),this._compileMaterial(this._equirectMaterial))}dispose(){this._dispose(),this._cubemapMaterial!==null&&this._cubemapMaterial.dispose(),this._equirectMaterial!==null&&this._equirectMaterial.dispose()}_setSize(t){this._lodMax=Math.floor(Math.log2(t)),this._cubeSize=Math.pow(2,this._lodMax)}_dispose(){this._blurMaterial!==null&&this._blurMaterial.dispose(),this._pingPongRenderTarget!==null&&this._pingPongRenderTarget.dispose();for(let t=0;t<this._lodPlanes.length;t++)this._lodPlanes[t].dispose()}_cleanup(t){this._renderer.setRenderTarget(qc,Zc,jc),t.scissorTest=!1,so(t,0,0,t.width,t.height)}_fromTexture(t,e){t.mapping===301||t.mapping===302?this._setSize(t.image.length===0?16:t.image[0].width||t.image[0].image.width):this._setSize(t.image.width/4),qc=this._renderer.getRenderTarget(),Zc=this._renderer.getActiveCubeFace(),jc=this._renderer.getActiveMipmapLevel();const n=e||this._allocateTargets();return this._textureToCubeUV(t,n),this._applyPMREM(n),this._cleanup(n),n}_allocateTargets(){const t=3*Math.max(this._cubeSize,112),e=4*this._cubeSize,n={magFilter:1006,minFilter:1006,generateMipmaps:!1,type:1016,format:1023,colorSpace:Ui,depthBuffer:!1},i=bu(t,e,n);if(this._pingPongRenderTarget===null||this._pingPongRenderTarget.width!==t||this._pingPongRenderTarget.height!==e){this._pingPongRenderTarget!==null&&this._dispose(),this._pingPongRenderTarget=bu(t,e,n);const{_lodMax:s}=this;({sizeLods:this._sizeLods,lodPlanes:this._lodPlanes,sigmas:this._sigmas}=Bx(s)),this._blurMaterial=Ux(s,t,e)}return i}_compileMaterial(t){const e=new jt(this._lodPlanes[0],t);this._renderer.compile(e,Yc)}_sceneToCubeUV(t,e,n,i){const o=new In(90,1,e,n),c=[1,-1,1,1,1,1],l=[1,1,1,-1,-1,-1],h=this._renderer,u=h.autoClear,d=h.toneMapping;h.getClearColor(Au),h.toneMapping=0,h.autoClear=!1;const f=new Fe({name:"PMREM.Background",side:1,depthWrite:!1,depthTest:!1}),g=new jt(new Dn,f);let x=!1;const m=t.background;m?m.isColor&&(f.color.copy(m),t.background=null,x=!0):(f.color.copy(Au),x=!0);for(let p=0;p<6;p++){const _=p%3;_===0?(o.up.set(0,c[p],0),o.lookAt(l[p],0,0)):_===1?(o.up.set(0,0,c[p]),o.lookAt(0,l[p],0)):(o.up.set(0,c[p],0),o.lookAt(0,0,l[p]));const v=this._cubeSize;so(i,_*v,p>2?v:0,v,v),h.setRenderTarget(i),x&&h.render(g,o),h.render(t,o)}g.geometry.dispose(),g.material.dispose(),h.toneMapping=d,h.autoClear=u,t.background=m}_textureToCubeUV(t,e){const n=this._renderer,i=t.mapping===301||t.mapping===302;i?(this._cubemapMaterial===null&&(this._cubemapMaterial=Lu()),this._cubemapMaterial.uniforms.flipEnvMap.value=t.isRenderTargetTexture===!1?-1:1):this._equirectMaterial===null&&(this._equirectMaterial=Pu());const s=i?this._cubemapMaterial:this._equirectMaterial,a=new jt(this._lodPlanes[0],s),o=s.uniforms;o.envMap.value=t;const c=this._cubeSize;so(e,0,0,3*c,2*c),n.setRenderTarget(e),n.render(a,Yc)}_applyPMREM(t){const e=this._renderer,n=e.autoClear;e.autoClear=!1;for(let i=1;i<this._lodPlanes.length;i++){const s=Math.sqrt(this._sigmas[i]*this._sigmas[i]-this._sigmas[i-1]*this._sigmas[i-1]),a=Cu[(i-1)%Cu.length];this._blur(t,i-1,i,s,a)}e.autoClear=n}_blur(t,e,n,i,s){const a=this._pingPongRenderTarget;this._halfBlur(t,a,e,n,i,"latitudinal",s),this._halfBlur(a,t,n,n,i,"longitudinal",s)}_halfBlur(t,e,n,i,s,a,o){const c=this._renderer,l=this._blurMaterial;a!=="latitudinal"&&a!=="longitudinal"&&console.error("blur direction must be either latitudinal or longitudinal!");const h=3,u=new jt(this._lodPlanes[i],l),d=l.uniforms,f=this._sizeLods[n]-1,g=isFinite(s)?Math.PI/(2*f):2*Math.PI/(2*Es-1),x=s/g,m=isFinite(s)?1+Math.floor(h*x):Es;m>Es&&console.warn(`sigmaRadians, ${s}, is too large and will clip, as it requested ${m} samples when the maximum is set to ${Es}`);const p=[];let _=0;for(let w=0;w<Es;++w){const A=w/x,E=Math.exp(-A*A/2);p.push(E),w===0?_+=E:w<m&&(_+=2*E)}for(let w=0;w<p.length;w++)p[w]=p[w]/_;d.envMap.value=t.texture,d.samples.value=m,d.weights.value=p,d.latitudinal.value=a==="latitudinal",o&&(d.poleAxis.value=o);const{_lodMax:v}=this;d.dTheta.value=g,d.mipInt.value=v-n;const S=this._sizeLods[i],M=3*S*(i>v-hr?i-v+hr:0),y=4*(this._cubeSize-S);so(e,M,y,3*S,2*S),c.setRenderTarget(e),c.render(u,Yc)}}function Bx(r){const t=[],e=[],n=[];let i=r;const s=r-hr+1+Tu.length;for(let a=0;a<s;a++){const o=Math.pow(2,i);e.push(o);let c=1/o;a>r-hr?c=Tu[a-r+hr-1]:a===0&&(c=0),n.push(c);const l=1/(o-2),h=-l,u=1+l,d=[h,h,u,h,u,u,h,h,u,u,h,u],f=6,g=6,x=3,m=2,p=1,_=new Float32Array(x*g*f),v=new Float32Array(m*g*f),S=new Float32Array(p*g*f);for(let y=0;y<f;y++){const w=y%3*2/3-1,A=y>2?0:-1,E=[w,A,0,w+2/3,A,0,w+2/3,A+1,0,w,A,0,w+2/3,A+1,0,w,A+1,0];_.set(E,x*g*y),v.set(d,m*g*y);const T=[y,y,y,y,y,y];S.set(T,p*g*y)}const M=new me;M.setAttribute("position",new Be(_,x)),M.setAttribute("uv",new Be(v,m)),M.setAttribute("faceIndex",new Be(S,p)),t.push(M),i>hr&&i--}return{lodPlanes:t,sizeLods:e,sigmas:n}}function bu(r,t,e){const n=new sn(r,t,e);return n.texture.mapping=306,n.texture.name="PMREM.cubeUv",n.scissorTest=!0,n}function so(r,t,e,n,i){r.viewport.set(t,e,n,i),r.scissor.set(t,e,n,i)}function Ux(r,t,e){const n=new Float32Array(Es),i=new R(0,1,0);return new Bn({name:"SphericalGaussianBlur",defines:{n:Es,CUBEUV_TEXEL_WIDTH:1/t,CUBEUV_TEXEL_HEIGHT:1/e,CUBEUV_MAX_MIP:`${r}.0`},uniforms:{envMap:{value:null},samples:{value:1},weights:{value:n},latitudinal:{value:!1},dTheta:{value:0},mipInt:{value:0},poleAxis:{value:i}},vertexShader:zh(),fragmentShader:`

			precision mediump float;
			precision mediump int;

			varying vec3 vOutputDirection;

			uniform sampler2D envMap;
			uniform int samples;
			uniform float weights[ n ];
			uniform bool latitudinal;
			uniform float dTheta;
			uniform float mipInt;
			uniform vec3 poleAxis;

			#define ENVMAP_TYPE_CUBE_UV
			#include <cube_uv_reflection_fragment>

			vec3 getSample( float theta, vec3 axis ) {

				float cosTheta = cos( theta );
				// Rodrigues' axis-angle rotation
				vec3 sampleDirection = vOutputDirection * cosTheta
					+ cross( axis, vOutputDirection ) * sin( theta )
					+ axis * dot( axis, vOutputDirection ) * ( 1.0 - cosTheta );

				return bilinearCubeUV( envMap, sampleDirection, mipInt );

			}

			void main() {

				vec3 axis = latitudinal ? poleAxis : cross( poleAxis, vOutputDirection );

				if ( all( equal( axis, vec3( 0.0 ) ) ) ) {

					axis = vec3( vOutputDirection.z, 0.0, - vOutputDirection.x );

				}

				axis = normalize( axis );

				gl_FragColor = vec4( 0.0, 0.0, 0.0, 1.0 );
				gl_FragColor.rgb += weights[ 0 ] * getSample( 0.0, axis );

				for ( int i = 1; i < n; i++ ) {

					if ( i >= samples ) {

						break;

					}

					float theta = dTheta * float( i );
					gl_FragColor.rgb += weights[ i ] * getSample( -1.0 * theta, axis );
					gl_FragColor.rgb += weights[ i ] * getSample( theta, axis );

				}

			}
		`,blending:0,depthTest:!1,depthWrite:!1})}function Pu(){return new Bn({name:"EquirectangularToCubeUV",uniforms:{envMap:{value:null}},vertexShader:zh(),fragmentShader:`

			precision mediump float;
			precision mediump int;

			varying vec3 vOutputDirection;

			uniform sampler2D envMap;

			#include <common>

			void main() {

				vec3 outputDirection = normalize( vOutputDirection );
				vec2 uv = equirectUv( outputDirection );

				gl_FragColor = vec4( texture2D ( envMap, uv ).rgb, 1.0 );

			}
		`,blending:0,depthTest:!1,depthWrite:!1})}function Lu(){return new Bn({name:"CubemapToCubeUV",uniforms:{envMap:{value:null},flipEnvMap:{value:-1}},vertexShader:zh(),fragmentShader:`

			precision mediump float;
			precision mediump int;

			uniform float flipEnvMap;

			varying vec3 vOutputDirection;

			uniform samplerCube envMap;

			void main() {

				gl_FragColor = textureCube( envMap, vec3( flipEnvMap * vOutputDirection.x, vOutputDirection.yz ) );

			}
		`,blending:0,depthTest:!1,depthWrite:!1})}function zh(){return`

		precision mediump float;
		precision mediump int;

		attribute float faceIndex;

		varying vec3 vOutputDirection;

		// RH coordinate system; PMREM face-indexing convention
		vec3 getDirection( vec2 uv, float face ) {

			uv = 2.0 * uv - 1.0;

			vec3 direction = vec3( uv, 1.0 );

			if ( face == 0.0 ) {

				direction = direction.zyx; // ( 1, v, u ) pos x

			} else if ( face == 1.0 ) {

				direction = direction.xzy;
				direction.xz *= -1.0; // ( -u, 1, -v ) pos y

			} else if ( face == 2.0 ) {

				direction.x *= -1.0; // ( -u, v, 1 ) pos z

			} else if ( face == 3.0 ) {

				direction = direction.zyx;
				direction.xz *= -1.0; // ( -1, v, -u ) neg x

			} else if ( face == 4.0 ) {

				direction = direction.xzy;
				direction.xy *= -1.0; // ( -u, -1, v ) neg y

			} else if ( face == 5.0 ) {

				direction.z *= -1.0; // ( u, v, -1 ) neg z

			}

			return direction;

		}

		void main() {

			vOutputDirection = getDirection( uv, faceIndex );
			gl_Position = vec4( position, 1.0 );

		}
	`}function zx(r){let t=new WeakMap,e=null;function n(o){if(o&&o.isTexture){const c=o.mapping,l=c===303||c===304,h=c===301||c===302;if(l||h)if(o.isRenderTargetTexture&&o.needsPMREMUpdate===!0){o.needsPMREMUpdate=!1;let u=t.get(o);return e===null&&(e=new Ru(r)),u=l?e.fromEquirectangular(o,u):e.fromCubemap(o,u),t.set(o,u),u.texture}else{if(t.has(o))return t.get(o).texture;{const u=o.image;if(l&&u&&u.height>0||h&&u&&i(u)){e===null&&(e=new Ru(r));const d=l?e.fromEquirectangular(o):e.fromCubemap(o);return t.set(o,d),o.addEventListener("dispose",s),d.texture}else return null}}}return o}function i(o){let c=0;const l=6;for(let h=0;h<l;h++)o[h]!==void 0&&c++;return c===l}function s(o){const c=o.target;c.removeEventListener("dispose",s);const l=t.get(c);l!==void 0&&(t.delete(c),l.dispose())}function a(){t=new WeakMap,e!==null&&(e.dispose(),e=null)}return{get:n,dispose:a}}function Ox(r){const t={};function e(n){if(t[n]!==void 0)return t[n];let i;switch(n){case"WEBGL_depth_texture":i=r.getExtension("WEBGL_depth_texture")||r.getExtension("MOZ_WEBGL_depth_texture")||r.getExtension("WEBKIT_WEBGL_depth_texture");break;case"EXT_texture_filter_anisotropic":i=r.getExtension("EXT_texture_filter_anisotropic")||r.getExtension("MOZ_EXT_texture_filter_anisotropic")||r.getExtension("WEBKIT_EXT_texture_filter_anisotropic");break;case"WEBGL_compressed_texture_s3tc":i=r.getExtension("WEBGL_compressed_texture_s3tc")||r.getExtension("MOZ_WEBGL_compressed_texture_s3tc")||r.getExtension("WEBKIT_WEBGL_compressed_texture_s3tc");break;case"WEBGL_compressed_texture_pvrtc":i=r.getExtension("WEBGL_compressed_texture_pvrtc")||r.getExtension("WEBKIT_WEBGL_compressed_texture_pvrtc");break;default:i=r.getExtension(n)}return t[n]=i,i}return{has:function(n){return e(n)!==null},init:function(n){n.isWebGL2?(e("EXT_color_buffer_float"),e("WEBGL_clip_cull_distance")):(e("WEBGL_depth_texture"),e("OES_texture_float"),e("OES_texture_half_float"),e("OES_texture_half_float_linear"),e("OES_standard_derivatives"),e("OES_element_index_uint"),e("OES_vertex_array_object"),e("ANGLE_instanced_arrays")),e("OES_texture_float_linear"),e("EXT_color_buffer_half_float"),e("WEBGL_multisampled_render_to_texture")},get:function(n){const i=e(n);return i===null&&console.warn("THREE.WebGLRenderer: "+n+" extension not supported."),i}}}function Gx(r,t,e,n){const i={},s=new WeakMap;function a(u){const d=u.target;d.index!==null&&t.remove(d.index);for(const g in d.attributes)t.remove(d.attributes[g]);for(const g in d.morphAttributes){const x=d.morphAttributes[g];for(let m=0,p=x.length;m<p;m++)t.remove(x[m])}d.removeEventListener("dispose",a),delete i[d.id];const f=s.get(d);f&&(t.remove(f),s.delete(d)),n.releaseStatesOfGeometry(d),d.isInstancedBufferGeometry===!0&&delete d._maxInstanceCount,e.memory.geometries--}function o(u,d){return i[d.id]===!0||(d.addEventListener("dispose",a),i[d.id]=!0,e.memory.geometries++),d}function c(u){const d=u.attributes;for(const g in d)t.update(d[g],r.ARRAY_BUFFER);const f=u.morphAttributes;for(const g in f){const x=f[g];for(let m=0,p=x.length;m<p;m++)t.update(x[m],r.ARRAY_BUFFER)}}function l(u){const d=[],f=u.index,g=u.attributes.position;let x=0;if(f!==null){const _=f.array;x=f.version;for(let v=0,S=_.length;v<S;v+=3){const M=_[v+0],y=_[v+1],w=_[v+2];d.push(M,y,y,w,w,M)}}else if(g!==void 0){const _=g.array;x=g.version;for(let v=0,S=_.length/3-1;v<S;v+=3){const M=v+0,y=v+1,w=v+2;d.push(M,y,y,w,w,M)}}else return;const m=new(Bf(d)?Hf:kf)(d,1);m.version=x;const p=s.get(u);p&&t.remove(p),s.set(u,m)}function h(u){const d=s.get(u);if(d){const f=u.index;f!==null&&d.version<f.version&&l(u)}else l(u);return s.get(u)}return{get:o,update:c,getWireframeAttribute:h}}function Vx(r,t,e,n){const i=n.isWebGL2;let s;function a(f){s=f}let o,c;function l(f){o=f.type,c=f.bytesPerElement}function h(f,g){r.drawElements(s,g,o,f*c),e.update(g,s,1)}function u(f,g,x){if(x===0)return;let m,p;if(i)m=r,p="drawElementsInstanced";else if(m=t.get("ANGLE_instanced_arrays"),p="drawElementsInstancedANGLE",m===null){console.error("THREE.WebGLIndexedBufferRenderer: using THREE.InstancedBufferGeometry but hardware does not support extension ANGLE_instanced_arrays.");return}m[p](s,g,o,f*c,x),e.update(g,s,x)}function d(f,g,x){if(x===0)return;const m=t.get("WEBGL_multi_draw");if(m===null)for(let p=0;p<x;p++)this.render(f[p]/c,g[p]);else{m.multiDrawElementsWEBGL(s,g,0,o,f,0,x);let p=0;for(let _=0;_<x;_++)p+=g[_];e.update(p,s,1)}}this.setMode=a,this.setIndex=l,this.render=h,this.renderInstances=u,this.renderMultiDraw=d}function kx(r){const t={geometries:0,textures:0},e={frame:0,calls:0,triangles:0,points:0,lines:0};function n(s,a,o){switch(e.calls++,a){case r.TRIANGLES:e.triangles+=o*(s/3);break;case r.LINES:e.lines+=o*(s/2);break;case r.LINE_STRIP:e.lines+=o*(s-1);break;case r.LINE_LOOP:e.lines+=o*s;break;case r.POINTS:e.points+=o*s;break;default:console.error("THREE.WebGLInfo: Unknown draw mode:",a);break}}function i(){e.calls=0,e.triangles=0,e.points=0,e.lines=0}return{memory:t,render:e,programs:null,autoReset:!0,reset:i,update:n}}function Hx(r,t){return r[0]-t[0]}function Wx(r,t){return Math.abs(t[1])-Math.abs(r[1])}function Xx(r,t,e){const n={},i=new Float32Array(8),s=new WeakMap,a=new en,o=[];for(let l=0;l<8;l++)o[l]=[l,0];function c(l,h,u){const d=l.morphTargetInfluences;if(t.isWebGL2===!0){const f=h.morphAttributes.position||h.morphAttributes.normal||h.morphAttributes.color,g=f!==void 0?f.length:0;let x=s.get(h);if(x===void 0||x.count!==g){let D=function(){C.dispose(),s.delete(h),h.removeEventListener("dispose",D)};x!==void 0&&x.texture.dispose();const _=h.morphAttributes.position!==void 0,v=h.morphAttributes.normal!==void 0,S=h.morphAttributes.color!==void 0,M=h.morphAttributes.position||[],y=h.morphAttributes.normal||[],w=h.morphAttributes.color||[];let A=0;_===!0&&(A=1),v===!0&&(A=2),S===!0&&(A=3);let E=h.attributes.position.count*A,T=1;E>t.maxTextureSize&&(T=Math.ceil(E/t.maxTextureSize),E=t.maxTextureSize);const L=new Float32Array(E*T*4*g),C=new Of(L,E,T,g);C.type=1015,C.needsUpdate=!0;const F=A*4;for(let N=0;N<g;N++){const B=M[N],G=y[N],z=w[N],H=E*T*4*N;for(let j=0;j<B.count;j++){const $=j*F;_===!0&&(a.fromBufferAttribute(B,j),L[H+$+0]=a.x,L[H+$+1]=a.y,L[H+$+2]=a.z,L[H+$+3]=0),v===!0&&(a.fromBufferAttribute(G,j),L[H+$+4]=a.x,L[H+$+5]=a.y,L[H+$+6]=a.z,L[H+$+7]=0),S===!0&&(a.fromBufferAttribute(z,j),L[H+$+8]=a.x,L[H+$+9]=a.y,L[H+$+10]=a.z,L[H+$+11]=z.itemSize===4?a.w:1)}}x={count:g,texture:C,size:new Mt(E,T)},s.set(h,x),h.addEventListener("dispose",D)}let m=0;for(let _=0;_<d.length;_++)m+=d[_];const p=h.morphTargetsRelative?1:1-m;u.getUniforms().setValue(r,"morphTargetBaseInfluence",p),u.getUniforms().setValue(r,"morphTargetInfluences",d),u.getUniforms().setValue(r,"morphTargetsTexture",x.texture,e),u.getUniforms().setValue(r,"morphTargetsTextureSize",x.size)}else{const f=d===void 0?0:d.length;let g=n[h.id];if(g===void 0||g.length!==f){g=[];for(let v=0;v<f;v++)g[v]=[v,0];n[h.id]=g}for(let v=0;v<f;v++){const S=g[v];S[0]=v,S[1]=d[v]}g.sort(Wx);for(let v=0;v<8;v++)v<f&&g[v][1]?(o[v][0]=g[v][0],o[v][1]=g[v][1]):(o[v][0]=Number.MAX_SAFE_INTEGER,o[v][1]=0);o.sort(Hx);const x=h.morphAttributes.position,m=h.morphAttributes.normal;let p=0;for(let v=0;v<8;v++){const S=o[v],M=S[0],y=S[1];M!==Number.MAX_SAFE_INTEGER&&y?(x&&h.getAttribute("morphTarget"+v)!==x[M]&&h.setAttribute("morphTarget"+v,x[M]),m&&h.getAttribute("morphNormal"+v)!==m[M]&&h.setAttribute("morphNormal"+v,m[M]),i[v]=y,p+=y):(x&&h.hasAttribute("morphTarget"+v)===!0&&h.deleteAttribute("morphTarget"+v),m&&h.hasAttribute("morphNormal"+v)===!0&&h.deleteAttribute("morphNormal"+v),i[v]=0)}const _=h.morphTargetsRelative?1:1-p;u.getUniforms().setValue(r,"morphTargetBaseInfluence",_),u.getUniforms().setValue(r,"morphTargetInfluences",i)}}return{update:c}}function Yx(r,t,e,n){let i=new WeakMap;function s(c){const l=n.render.frame,h=c.geometry,u=t.get(c,h);if(i.get(u)!==l&&(t.update(u),i.set(u,l)),c.isInstancedMesh&&(c.hasEventListener("dispose",o)===!1&&c.addEventListener("dispose",o),i.get(c)!==l&&(e.update(c.instanceMatrix,r.ARRAY_BUFFER),c.instanceColor!==null&&e.update(c.instanceColor,r.ARRAY_BUFFER),i.set(c,l))),c.isSkinnedMesh){const d=c.skeleton;i.get(d)!==l&&(d.update(),i.set(d,l))}return u}function a(){i=new WeakMap}function o(c){const l=c.target;l.removeEventListener("dispose",o),e.remove(l.instanceMatrix),l.instanceColor!==null&&e.remove(l.instanceColor)}return{update:s,dispose:a}}class Zf extends Fn{constructor(t,e,n,i,s,a,o,c,l,h){if(h=h!==void 0?h:1026,h!==1026&&h!==1027)throw new Error("DepthTexture format must be either THREE.DepthFormat or THREE.DepthStencilFormat");n===void 0&&h===1026&&(n=1014),n===void 0&&h===1027&&(n=1020),super(null,i,s,a,o,c,h,n,l),this.isDepthTexture=!0,this.image={width:t,height:e},this.magFilter=o!==void 0?o:1003,this.minFilter=c!==void 0?c:1003,this.flipY=!1,this.generateMipmaps=!1,this.compareFunction=null}copy(t){return super.copy(t),this.compareFunction=t.compareFunction,this}toJSON(t){const e=super.toJSON(t);return this.compareFunction!==null&&(e.compareFunction=this.compareFunction),e}}const jf=new Fn,$f=new Zf(1,1);$f.compareFunction=515;const Kf=new Of,Jf=new Pm,Qf=new Yf,Du=[],Iu=[],Nu=new Float32Array(16),Fu=new Float32Array(9),Bu=new Float32Array(4);function Cr(r,t,e){const n=r[0];if(n<=0||n>0)return r;const i=t*e;let s=Du[i];if(s===void 0&&(s=new Float32Array(i),Du[i]=s),t!==0){n.toArray(s,0);for(let a=1,o=0;a!==t;++a)o+=e,r[a].toArray(s,o)}return s}function $e(r,t){if(r.length!==t.length)return!1;for(let e=0,n=r.length;e<n;e++)if(r[e]!==t[e])return!1;return!0}function Ke(r,t){for(let e=0,n=t.length;e<n;e++)r[e]=t[e]}function mc(r,t){let e=Iu[t];e===void 0&&(e=new Int32Array(t),Iu[t]=e);for(let n=0;n!==t;++n)e[n]=r.allocateTextureUnit();return e}function qx(r,t){const e=this.cache;e[0]!==t&&(r.uniform1f(this.addr,t),e[0]=t)}function Zx(r,t){const e=this.cache;if(t.x!==void 0)(e[0]!==t.x||e[1]!==t.y)&&(r.uniform2f(this.addr,t.x,t.y),e[0]=t.x,e[1]=t.y);else{if($e(e,t))return;r.uniform2fv(this.addr,t),Ke(e,t)}}function jx(r,t){const e=this.cache;if(t.x!==void 0)(e[0]!==t.x||e[1]!==t.y||e[2]!==t.z)&&(r.uniform3f(this.addr,t.x,t.y,t.z),e[0]=t.x,e[1]=t.y,e[2]=t.z);else if(t.r!==void 0)(e[0]!==t.r||e[1]!==t.g||e[2]!==t.b)&&(r.uniform3f(this.addr,t.r,t.g,t.b),e[0]=t.r,e[1]=t.g,e[2]=t.b);else{if($e(e,t))return;r.uniform3fv(this.addr,t),Ke(e,t)}}function $x(r,t){const e=this.cache;if(t.x!==void 0)(e[0]!==t.x||e[1]!==t.y||e[2]!==t.z||e[3]!==t.w)&&(r.uniform4f(this.addr,t.x,t.y,t.z,t.w),e[0]=t.x,e[1]=t.y,e[2]=t.z,e[3]=t.w);else{if($e(e,t))return;r.uniform4fv(this.addr,t),Ke(e,t)}}function Kx(r,t){const e=this.cache,n=t.elements;if(n===void 0){if($e(e,t))return;r.uniformMatrix2fv(this.addr,!1,t),Ke(e,t)}else{if($e(e,n))return;Bu.set(n),r.uniformMatrix2fv(this.addr,!1,Bu),Ke(e,n)}}function Jx(r,t){const e=this.cache,n=t.elements;if(n===void 0){if($e(e,t))return;r.uniformMatrix3fv(this.addr,!1,t),Ke(e,t)}else{if($e(e,n))return;Fu.set(n),r.uniformMatrix3fv(this.addr,!1,Fu),Ke(e,n)}}function Qx(r,t){const e=this.cache,n=t.elements;if(n===void 0){if($e(e,t))return;r.uniformMatrix4fv(this.addr,!1,t),Ke(e,t)}else{if($e(e,n))return;Nu.set(n),r.uniformMatrix4fv(this.addr,!1,Nu),Ke(e,n)}}function t_(r,t){const e=this.cache;e[0]!==t&&(r.uniform1i(this.addr,t),e[0]=t)}function e_(r,t){const e=this.cache;if(t.x!==void 0)(e[0]!==t.x||e[1]!==t.y)&&(r.uniform2i(this.addr,t.x,t.y),e[0]=t.x,e[1]=t.y);else{if($e(e,t))return;r.uniform2iv(this.addr,t),Ke(e,t)}}function n_(r,t){const e=this.cache;if(t.x!==void 0)(e[0]!==t.x||e[1]!==t.y||e[2]!==t.z)&&(r.uniform3i(this.addr,t.x,t.y,t.z),e[0]=t.x,e[1]=t.y,e[2]=t.z);else{if($e(e,t))return;r.uniform3iv(this.addr,t),Ke(e,t)}}function i_(r,t){const e=this.cache;if(t.x!==void 0)(e[0]!==t.x||e[1]!==t.y||e[2]!==t.z||e[3]!==t.w)&&(r.uniform4i(this.addr,t.x,t.y,t.z,t.w),e[0]=t.x,e[1]=t.y,e[2]=t.z,e[3]=t.w);else{if($e(e,t))return;r.uniform4iv(this.addr,t),Ke(e,t)}}function s_(r,t){const e=this.cache;e[0]!==t&&(r.uniform1ui(this.addr,t),e[0]=t)}function r_(r,t){const e=this.cache;if(t.x!==void 0)(e[0]!==t.x||e[1]!==t.y)&&(r.uniform2ui(this.addr,t.x,t.y),e[0]=t.x,e[1]=t.y);else{if($e(e,t))return;r.uniform2uiv(this.addr,t),Ke(e,t)}}function a_(r,t){const e=this.cache;if(t.x!==void 0)(e[0]!==t.x||e[1]!==t.y||e[2]!==t.z)&&(r.uniform3ui(this.addr,t.x,t.y,t.z),e[0]=t.x,e[1]=t.y,e[2]=t.z);else{if($e(e,t))return;r.uniform3uiv(this.addr,t),Ke(e,t)}}function o_(r,t){const e=this.cache;if(t.x!==void 0)(e[0]!==t.x||e[1]!==t.y||e[2]!==t.z||e[3]!==t.w)&&(r.uniform4ui(this.addr,t.x,t.y,t.z,t.w),e[0]=t.x,e[1]=t.y,e[2]=t.z,e[3]=t.w);else{if($e(e,t))return;r.uniform4uiv(this.addr,t),Ke(e,t)}}function c_(r,t,e){const n=this.cache,i=e.allocateTextureUnit();n[0]!==i&&(r.uniform1i(this.addr,i),n[0]=i);const s=this.type===r.SAMPLER_2D_SHADOW?$f:jf;e.setTexture2D(t||s,i)}function l_(r,t,e){const n=this.cache,i=e.allocateTextureUnit();n[0]!==i&&(r.uniform1i(this.addr,i),n[0]=i),e.setTexture3D(t||Jf,i)}function h_(r,t,e){const n=this.cache,i=e.allocateTextureUnit();n[0]!==i&&(r.uniform1i(this.addr,i),n[0]=i),e.setTextureCube(t||Qf,i)}function u_(r,t,e){const n=this.cache,i=e.allocateTextureUnit();n[0]!==i&&(r.uniform1i(this.addr,i),n[0]=i),e.setTexture2DArray(t||Kf,i)}function d_(r){switch(r){case 5126:return qx;case 35664:return Zx;case 35665:return jx;case 35666:return $x;case 35674:return Kx;case 35675:return Jx;case 35676:return Qx;case 5124:case 35670:return t_;case 35667:case 35671:return e_;case 35668:case 35672:return n_;case 35669:case 35673:return i_;case 5125:return s_;case 36294:return r_;case 36295:return a_;case 36296:return o_;case 35678:case 36198:case 36298:case 36306:case 35682:return c_;case 35679:case 36299:case 36307:return l_;case 35680:case 36300:case 36308:case 36293:return h_;case 36289:case 36303:case 36311:case 36292:return u_}}function f_(r,t){r.uniform1fv(this.addr,t)}function p_(r,t){const e=Cr(t,this.size,2);r.uniform2fv(this.addr,e)}function m_(r,t){const e=Cr(t,this.size,3);r.uniform3fv(this.addr,e)}function g_(r,t){const e=Cr(t,this.size,4);r.uniform4fv(this.addr,e)}function x_(r,t){const e=Cr(t,this.size,4);r.uniformMatrix2fv(this.addr,!1,e)}function __(r,t){const e=Cr(t,this.size,9);r.uniformMatrix3fv(this.addr,!1,e)}function v_(r,t){const e=Cr(t,this.size,16);r.uniformMatrix4fv(this.addr,!1,e)}function S_(r,t){r.uniform1iv(this.addr,t)}function M_(r,t){r.uniform2iv(this.addr,t)}function y_(r,t){r.uniform3iv(this.addr,t)}function E_(r,t){r.uniform4iv(this.addr,t)}function w_(r,t){r.uniform1uiv(this.addr,t)}function T_(r,t){r.uniform2uiv(this.addr,t)}function A_(r,t){r.uniform3uiv(this.addr,t)}function C_(r,t){r.uniform4uiv(this.addr,t)}function R_(r,t,e){const n=this.cache,i=t.length,s=mc(e,i);$e(n,s)||(r.uniform1iv(this.addr,s),Ke(n,s));for(let a=0;a!==i;++a)e.setTexture2D(t[a]||jf,s[a])}function b_(r,t,e){const n=this.cache,i=t.length,s=mc(e,i);$e(n,s)||(r.uniform1iv(this.addr,s),Ke(n,s));for(let a=0;a!==i;++a)e.setTexture3D(t[a]||Jf,s[a])}function P_(r,t,e){const n=this.cache,i=t.length,s=mc(e,i);$e(n,s)||(r.uniform1iv(this.addr,s),Ke(n,s));for(let a=0;a!==i;++a)e.setTextureCube(t[a]||Qf,s[a])}function L_(r,t,e){const n=this.cache,i=t.length,s=mc(e,i);$e(n,s)||(r.uniform1iv(this.addr,s),Ke(n,s));for(let a=0;a!==i;++a)e.setTexture2DArray(t[a]||Kf,s[a])}function D_(r){switch(r){case 5126:return f_;case 35664:return p_;case 35665:return m_;case 35666:return g_;case 35674:return x_;case 35675:return __;case 35676:return v_;case 5124:case 35670:return S_;case 35667:case 35671:return M_;case 35668:case 35672:return y_;case 35669:case 35673:return E_;case 5125:return w_;case 36294:return T_;case 36295:return A_;case 36296:return C_;case 35678:case 36198:case 36298:case 36306:case 35682:return R_;case 35679:case 36299:case 36307:return b_;case 35680:case 36300:case 36308:case 36293:return P_;case 36289:case 36303:case 36311:case 36292:return L_}}class I_{constructor(t,e,n){this.id=t,this.addr=n,this.cache=[],this.type=e.type,this.setValue=d_(e.type)}}class N_{constructor(t,e,n){this.id=t,this.addr=n,this.cache=[],this.type=e.type,this.size=e.size,this.setValue=D_(e.type)}}class F_{constructor(t){this.id=t,this.seq=[],this.map={}}setValue(t,e,n){const i=this.seq;for(let s=0,a=i.length;s!==a;++s){const o=i[s];o.setValue(t,e[o.id],n)}}}const $c=/(\w+)(\])?(\[|\.)?/g;function Uu(r,t){r.seq.push(t),r.map[t.id]=t}function B_(r,t,e){const n=r.name,i=n.length;for($c.lastIndex=0;;){const s=$c.exec(n),a=$c.lastIndex;let o=s[1];const c=s[2]==="]",l=s[3];if(c&&(o=o|0),l===void 0||l==="["&&a+2===i){Uu(e,l===void 0?new I_(o,r,t):new N_(o,r,t));break}else{let u=e.map[o];u===void 0&&(u=new F_(o),Uu(e,u)),e=u}}}class Go{constructor(t,e){this.seq=[],this.map={};const n=t.getProgramParameter(e,t.ACTIVE_UNIFORMS);for(let i=0;i<n;++i){const s=t.getActiveUniform(e,i),a=t.getUniformLocation(e,s.name);B_(s,a,this)}}setValue(t,e,n,i){const s=this.map[e];s!==void 0&&s.setValue(t,n,i)}setOptional(t,e,n){const i=e[n];i!==void 0&&this.setValue(t,n,i)}static upload(t,e,n,i){for(let s=0,a=e.length;s!==a;++s){const o=e[s],c=n[o.id];c.needsUpdate!==!1&&o.setValue(t,c.value,i)}}static seqWithValue(t,e){const n=[];for(let i=0,s=t.length;i!==s;++i){const a=t[i];a.id in e&&n.push(a)}return n}}function zu(r,t,e){const n=r.createShader(t);return r.shaderSource(n,e),r.compileShader(n),n}const U_=37297;let z_=0;function O_(r,t){const e=r.split(`
`),n=[],i=Math.max(t-6,0),s=Math.min(t+6,e.length);for(let a=i;a<s;a++){const o=a+1;n.push(`${o===t?">":" "} ${o}: ${e[a]}`)}return n.join(`
`)}function G_(r){const t=pe.getPrimaries(pe.workingColorSpace),e=pe.getPrimaries(r);let n;switch(t===e?n="":t==="p3"&&e===$o?n="LinearDisplayP3ToLinearSRGB":t===$o&&e==="p3"&&(n="LinearSRGBToLinearDisplayP3"),r){case Ui:case fc:return[n,"LinearTransferOETF"];case an:case Ih:return[n,"sRGBTransferOETF"];default:return console.warn("THREE.WebGLProgram: Unsupported color space:",r),[n,"LinearTransferOETF"]}}function Ou(r,t,e){const n=r.getShaderParameter(t,r.COMPILE_STATUS),i=r.getShaderInfoLog(t).trim();if(n&&i==="")return"";const s=/ERROR: 0:(\d+)/.exec(i);if(s){const a=parseInt(s[1]);return e.toUpperCase()+`

`+i+`

`+O_(r.getShaderSource(t),a)}else return i}function V_(r,t){const e=G_(t);return`vec4 ${r}( vec4 value ) { return ${e[0]}( ${e[1]}( value ) ); }`}function k_(r,t){let e;switch(t){case 1:e="Linear";break;case 2:e="Reinhard";break;case 3:e="OptimizedCineon";break;case 4:e="ACESFilmic";break;case 6:e="AgX";break;case 5:e="Custom";break;default:console.warn("THREE.WebGLProgram: Unsupported toneMapping:",t),e="Linear"}return"vec3 "+r+"( vec3 color ) { return "+e+"ToneMapping( color ); }"}function H_(r){return[r.extensionDerivatives||r.envMapCubeUVHeight||r.bumpMap||r.normalMapTangentSpace||r.clearcoatNormalMap||r.flatShading||r.shaderID==="physical"?"#extension GL_OES_standard_derivatives : enable":"",(r.extensionFragDepth||r.logarithmicDepthBuffer)&&r.rendererExtensionFragDepth?"#extension GL_EXT_frag_depth : enable":"",r.extensionDrawBuffers&&r.rendererExtensionDrawBuffers?"#extension GL_EXT_draw_buffers : require":"",(r.extensionShaderTextureLOD||r.envMap||r.transmission)&&r.rendererExtensionShaderTextureLod?"#extension GL_EXT_shader_texture_lod : enable":""].filter(ur).join(`
`)}function W_(r){return[r.extensionClipCullDistance?"#extension GL_ANGLE_clip_cull_distance : require":""].filter(ur).join(`
`)}function X_(r){const t=[];for(const e in r){const n=r[e];n!==!1&&t.push("#define "+e+" "+n)}return t.join(`
`)}function Y_(r,t){const e={},n=r.getProgramParameter(t,r.ACTIVE_ATTRIBUTES);for(let i=0;i<n;i++){const s=r.getActiveAttrib(t,i),a=s.name;let o=1;s.type===r.FLOAT_MAT2&&(o=2),s.type===r.FLOAT_MAT3&&(o=3),s.type===r.FLOAT_MAT4&&(o=4),e[a]={type:s.type,location:r.getAttribLocation(t,a),locationSize:o}}return e}function ur(r){return r!==""}function Gu(r,t){const e=t.numSpotLightShadows+t.numSpotLightMaps-t.numSpotLightShadowsWithMaps;return r.replace(/NUM_DIR_LIGHTS/g,t.numDirLights).replace(/NUM_SPOT_LIGHTS/g,t.numSpotLights).replace(/NUM_SPOT_LIGHT_MAPS/g,t.numSpotLightMaps).replace(/NUM_SPOT_LIGHT_COORDS/g,e).replace(/NUM_RECT_AREA_LIGHTS/g,t.numRectAreaLights).replace(/NUM_POINT_LIGHTS/g,t.numPointLights).replace(/NUM_HEMI_LIGHTS/g,t.numHemiLights).replace(/NUM_DIR_LIGHT_SHADOWS/g,t.numDirLightShadows).replace(/NUM_SPOT_LIGHT_SHADOWS_WITH_MAPS/g,t.numSpotLightShadowsWithMaps).replace(/NUM_SPOT_LIGHT_SHADOWS/g,t.numSpotLightShadows).replace(/NUM_POINT_LIGHT_SHADOWS/g,t.numPointLightShadows)}function Vu(r,t){return r.replace(/NUM_CLIPPING_PLANES/g,t.numClippingPlanes).replace(/UNION_CLIPPING_PLANES/g,t.numClippingPlanes-t.numClipIntersection)}const q_=/^[ \t]*#include +<([\w\d./]+)>/gm;function eh(r){return r.replace(q_,j_)}const Z_=new Map([["encodings_fragment","colorspace_fragment"],["encodings_pars_fragment","colorspace_pars_fragment"],["output_fragment","opaque_fragment"]]);function j_(r,t){let e=ee[t];if(e===void 0){const n=Z_.get(t);if(n!==void 0)e=ee[n],console.warn('THREE.WebGLRenderer: Shader chunk "%s" has been deprecated. Use "%s" instead.',t,n);else throw new Error("Can not resolve #include <"+t+">")}return eh(e)}const $_=/#pragma unroll_loop_start\s+for\s*\(\s*int\s+i\s*=\s*(\d+)\s*;\s*i\s*<\s*(\d+)\s*;\s*i\s*\+\+\s*\)\s*{([\s\S]+?)}\s+#pragma unroll_loop_end/g;function ku(r){return r.replace($_,K_)}function K_(r,t,e,n){let i="";for(let s=parseInt(t);s<parseInt(e);s++)i+=n.replace(/\[\s*i\s*\]/g,"[ "+s+" ]").replace(/UNROLLED_LOOP_INDEX/g,s);return i}function Hu(r){let t="precision "+r.precision+` float;
precision `+r.precision+" int;";return r.precision==="highp"?t+=`
#define HIGH_PRECISION`:r.precision==="mediump"?t+=`
#define MEDIUM_PRECISION`:r.precision==="lowp"&&(t+=`
#define LOW_PRECISION`),t}function J_(r){let t="SHADOWMAP_TYPE_BASIC";return r.shadowMapType===1?t="SHADOWMAP_TYPE_PCF":r.shadowMapType===2?t="SHADOWMAP_TYPE_PCF_SOFT":r.shadowMapType===3&&(t="SHADOWMAP_TYPE_VSM"),t}function Q_(r){let t="ENVMAP_TYPE_CUBE";if(r.envMap)switch(r.envMapMode){case 301:case 302:t="ENVMAP_TYPE_CUBE";break;case 306:t="ENVMAP_TYPE_CUBE_UV";break}return t}function tv(r){let t="ENVMAP_MODE_REFLECTION";return r.envMap&&r.envMapMode===302&&(t="ENVMAP_MODE_REFRACTION"),t}function ev(r){let t="ENVMAP_BLENDING_NONE";if(r.envMap)switch(r.combine){case 0:t="ENVMAP_BLENDING_MULTIPLY";break;case 1:t="ENVMAP_BLENDING_MIX";break;case 2:t="ENVMAP_BLENDING_ADD";break}return t}function nv(r){const t=r.envMapCubeUVHeight;if(t===null)return null;const e=Math.log2(t)-2,n=1/t;return{texelWidth:1/(3*Math.max(Math.pow(2,e),112)),texelHeight:n,maxMip:e}}function iv(r,t,e,n){const i=r.getContext(),s=e.defines;let a=e.vertexShader,o=e.fragmentShader;const c=J_(e),l=Q_(e),h=tv(e),u=ev(e),d=nv(e),f=e.isWebGL2?"":H_(e),g=W_(e),x=X_(s),m=i.createProgram();let p,_,v=e.glslVersion?"#version "+e.glslVersion+`
`:"";e.isRawShaderMaterial?(p=["#define SHADER_TYPE "+e.shaderType,"#define SHADER_NAME "+e.shaderName,x].filter(ur).join(`
`),p.length>0&&(p+=`
`),_=[f,"#define SHADER_TYPE "+e.shaderType,"#define SHADER_NAME "+e.shaderName,x].filter(ur).join(`
`),_.length>0&&(_+=`
`)):(p=[Hu(e),"#define SHADER_TYPE "+e.shaderType,"#define SHADER_NAME "+e.shaderName,x,e.extensionClipCullDistance?"#define USE_CLIP_DISTANCE":"",e.batching?"#define USE_BATCHING":"",e.instancing?"#define USE_INSTANCING":"",e.instancingColor?"#define USE_INSTANCING_COLOR":"",e.useFog&&e.fog?"#define USE_FOG":"",e.useFog&&e.fogExp2?"#define FOG_EXP2":"",e.map?"#define USE_MAP":"",e.envMap?"#define USE_ENVMAP":"",e.envMap?"#define "+h:"",e.lightMap?"#define USE_LIGHTMAP":"",e.aoMap?"#define USE_AOMAP":"",e.bumpMap?"#define USE_BUMPMAP":"",e.normalMap?"#define USE_NORMALMAP":"",e.normalMapObjectSpace?"#define USE_NORMALMAP_OBJECTSPACE":"",e.normalMapTangentSpace?"#define USE_NORMALMAP_TANGENTSPACE":"",e.displacementMap?"#define USE_DISPLACEMENTMAP":"",e.emissiveMap?"#define USE_EMISSIVEMAP":"",e.anisotropy?"#define USE_ANISOTROPY":"",e.anisotropyMap?"#define USE_ANISOTROPYMAP":"",e.clearcoatMap?"#define USE_CLEARCOATMAP":"",e.clearcoatRoughnessMap?"#define USE_CLEARCOAT_ROUGHNESSMAP":"",e.clearcoatNormalMap?"#define USE_CLEARCOAT_NORMALMAP":"",e.iridescenceMap?"#define USE_IRIDESCENCEMAP":"",e.iridescenceThicknessMap?"#define USE_IRIDESCENCE_THICKNESSMAP":"",e.specularMap?"#define USE_SPECULARMAP":"",e.specularColorMap?"#define USE_SPECULAR_COLORMAP":"",e.specularIntensityMap?"#define USE_SPECULAR_INTENSITYMAP":"",e.roughnessMap?"#define USE_ROUGHNESSMAP":"",e.metalnessMap?"#define USE_METALNESSMAP":"",e.alphaMap?"#define USE_ALPHAMAP":"",e.alphaHash?"#define USE_ALPHAHASH":"",e.transmission?"#define USE_TRANSMISSION":"",e.transmissionMap?"#define USE_TRANSMISSIONMAP":"",e.thicknessMap?"#define USE_THICKNESSMAP":"",e.sheenColorMap?"#define USE_SHEEN_COLORMAP":"",e.sheenRoughnessMap?"#define USE_SHEEN_ROUGHNESSMAP":"",e.mapUv?"#define MAP_UV "+e.mapUv:"",e.alphaMapUv?"#define ALPHAMAP_UV "+e.alphaMapUv:"",e.lightMapUv?"#define LIGHTMAP_UV "+e.lightMapUv:"",e.aoMapUv?"#define AOMAP_UV "+e.aoMapUv:"",e.emissiveMapUv?"#define EMISSIVEMAP_UV "+e.emissiveMapUv:"",e.bumpMapUv?"#define BUMPMAP_UV "+e.bumpMapUv:"",e.normalMapUv?"#define NORMALMAP_UV "+e.normalMapUv:"",e.displacementMapUv?"#define DISPLACEMENTMAP_UV "+e.displacementMapUv:"",e.metalnessMapUv?"#define METALNESSMAP_UV "+e.metalnessMapUv:"",e.roughnessMapUv?"#define ROUGHNESSMAP_UV "+e.roughnessMapUv:"",e.anisotropyMapUv?"#define ANISOTROPYMAP_UV "+e.anisotropyMapUv:"",e.clearcoatMapUv?"#define CLEARCOATMAP_UV "+e.clearcoatMapUv:"",e.clearcoatNormalMapUv?"#define CLEARCOAT_NORMALMAP_UV "+e.clearcoatNormalMapUv:"",e.clearcoatRoughnessMapUv?"#define CLEARCOAT_ROUGHNESSMAP_UV "+e.clearcoatRoughnessMapUv:"",e.iridescenceMapUv?"#define IRIDESCENCEMAP_UV "+e.iridescenceMapUv:"",e.iridescenceThicknessMapUv?"#define IRIDESCENCE_THICKNESSMAP_UV "+e.iridescenceThicknessMapUv:"",e.sheenColorMapUv?"#define SHEEN_COLORMAP_UV "+e.sheenColorMapUv:"",e.sheenRoughnessMapUv?"#define SHEEN_ROUGHNESSMAP_UV "+e.sheenRoughnessMapUv:"",e.specularMapUv?"#define SPECULARMAP_UV "+e.specularMapUv:"",e.specularColorMapUv?"#define SPECULAR_COLORMAP_UV "+e.specularColorMapUv:"",e.specularIntensityMapUv?"#define SPECULAR_INTENSITYMAP_UV "+e.specularIntensityMapUv:"",e.transmissionMapUv?"#define TRANSMISSIONMAP_UV "+e.transmissionMapUv:"",e.thicknessMapUv?"#define THICKNESSMAP_UV "+e.thicknessMapUv:"",e.vertexTangents&&e.flatShading===!1?"#define USE_TANGENT":"",e.vertexColors?"#define USE_COLOR":"",e.vertexAlphas?"#define USE_COLOR_ALPHA":"",e.vertexUv1s?"#define USE_UV1":"",e.vertexUv2s?"#define USE_UV2":"",e.vertexUv3s?"#define USE_UV3":"",e.pointsUvs?"#define USE_POINTS_UV":"",e.flatShading?"#define FLAT_SHADED":"",e.skinning?"#define USE_SKINNING":"",e.morphTargets?"#define USE_MORPHTARGETS":"",e.morphNormals&&e.flatShading===!1?"#define USE_MORPHNORMALS":"",e.morphColors&&e.isWebGL2?"#define USE_MORPHCOLORS":"",e.morphTargetsCount>0&&e.isWebGL2?"#define MORPHTARGETS_TEXTURE":"",e.morphTargetsCount>0&&e.isWebGL2?"#define MORPHTARGETS_TEXTURE_STRIDE "+e.morphTextureStride:"",e.morphTargetsCount>0&&e.isWebGL2?"#define MORPHTARGETS_COUNT "+e.morphTargetsCount:"",e.doubleSided?"#define DOUBLE_SIDED":"",e.flipSided?"#define FLIP_SIDED":"",e.shadowMapEnabled?"#define USE_SHADOWMAP":"",e.shadowMapEnabled?"#define "+c:"",e.sizeAttenuation?"#define USE_SIZEATTENUATION":"",e.numLightProbes>0?"#define USE_LIGHT_PROBES":"",e.useLegacyLights?"#define LEGACY_LIGHTS":"",e.logarithmicDepthBuffer?"#define USE_LOGDEPTHBUF":"",e.logarithmicDepthBuffer&&e.rendererExtensionFragDepth?"#define USE_LOGDEPTHBUF_EXT":"","uniform mat4 modelMatrix;","uniform mat4 modelViewMatrix;","uniform mat4 projectionMatrix;","uniform mat4 viewMatrix;","uniform mat3 normalMatrix;","uniform vec3 cameraPosition;","uniform bool isOrthographic;","#ifdef USE_INSTANCING","	attribute mat4 instanceMatrix;","#endif","#ifdef USE_INSTANCING_COLOR","	attribute vec3 instanceColor;","#endif","attribute vec3 position;","attribute vec3 normal;","attribute vec2 uv;","#ifdef USE_UV1","	attribute vec2 uv1;","#endif","#ifdef USE_UV2","	attribute vec2 uv2;","#endif","#ifdef USE_UV3","	attribute vec2 uv3;","#endif","#ifdef USE_TANGENT","	attribute vec4 tangent;","#endif","#if defined( USE_COLOR_ALPHA )","	attribute vec4 color;","#elif defined( USE_COLOR )","	attribute vec3 color;","#endif","#if ( defined( USE_MORPHTARGETS ) && ! defined( MORPHTARGETS_TEXTURE ) )","	attribute vec3 morphTarget0;","	attribute vec3 morphTarget1;","	attribute vec3 morphTarget2;","	attribute vec3 morphTarget3;","	#ifdef USE_MORPHNORMALS","		attribute vec3 morphNormal0;","		attribute vec3 morphNormal1;","		attribute vec3 morphNormal2;","		attribute vec3 morphNormal3;","	#else","		attribute vec3 morphTarget4;","		attribute vec3 morphTarget5;","		attribute vec3 morphTarget6;","		attribute vec3 morphTarget7;","	#endif","#endif","#ifdef USE_SKINNING","	attribute vec4 skinIndex;","	attribute vec4 skinWeight;","#endif",`
`].filter(ur).join(`
`),_=[f,Hu(e),"#define SHADER_TYPE "+e.shaderType,"#define SHADER_NAME "+e.shaderName,x,e.useFog&&e.fog?"#define USE_FOG":"",e.useFog&&e.fogExp2?"#define FOG_EXP2":"",e.map?"#define USE_MAP":"",e.matcap?"#define USE_MATCAP":"",e.envMap?"#define USE_ENVMAP":"",e.envMap?"#define "+l:"",e.envMap?"#define "+h:"",e.envMap?"#define "+u:"",d?"#define CUBEUV_TEXEL_WIDTH "+d.texelWidth:"",d?"#define CUBEUV_TEXEL_HEIGHT "+d.texelHeight:"",d?"#define CUBEUV_MAX_MIP "+d.maxMip+".0":"",e.lightMap?"#define USE_LIGHTMAP":"",e.aoMap?"#define USE_AOMAP":"",e.bumpMap?"#define USE_BUMPMAP":"",e.normalMap?"#define USE_NORMALMAP":"",e.normalMapObjectSpace?"#define USE_NORMALMAP_OBJECTSPACE":"",e.normalMapTangentSpace?"#define USE_NORMALMAP_TANGENTSPACE":"",e.emissiveMap?"#define USE_EMISSIVEMAP":"",e.anisotropy?"#define USE_ANISOTROPY":"",e.anisotropyMap?"#define USE_ANISOTROPYMAP":"",e.clearcoat?"#define USE_CLEARCOAT":"",e.clearcoatMap?"#define USE_CLEARCOATMAP":"",e.clearcoatRoughnessMap?"#define USE_CLEARCOAT_ROUGHNESSMAP":"",e.clearcoatNormalMap?"#define USE_CLEARCOAT_NORMALMAP":"",e.iridescence?"#define USE_IRIDESCENCE":"",e.iridescenceMap?"#define USE_IRIDESCENCEMAP":"",e.iridescenceThicknessMap?"#define USE_IRIDESCENCE_THICKNESSMAP":"",e.specularMap?"#define USE_SPECULARMAP":"",e.specularColorMap?"#define USE_SPECULAR_COLORMAP":"",e.specularIntensityMap?"#define USE_SPECULAR_INTENSITYMAP":"",e.roughnessMap?"#define USE_ROUGHNESSMAP":"",e.metalnessMap?"#define USE_METALNESSMAP":"",e.alphaMap?"#define USE_ALPHAMAP":"",e.alphaTest?"#define USE_ALPHATEST":"",e.alphaHash?"#define USE_ALPHAHASH":"",e.sheen?"#define USE_SHEEN":"",e.sheenColorMap?"#define USE_SHEEN_COLORMAP":"",e.sheenRoughnessMap?"#define USE_SHEEN_ROUGHNESSMAP":"",e.transmission?"#define USE_TRANSMISSION":"",e.transmissionMap?"#define USE_TRANSMISSIONMAP":"",e.thicknessMap?"#define USE_THICKNESSMAP":"",e.vertexTangents&&e.flatShading===!1?"#define USE_TANGENT":"",e.vertexColors||e.instancingColor?"#define USE_COLOR":"",e.vertexAlphas?"#define USE_COLOR_ALPHA":"",e.vertexUv1s?"#define USE_UV1":"",e.vertexUv2s?"#define USE_UV2":"",e.vertexUv3s?"#define USE_UV3":"",e.pointsUvs?"#define USE_POINTS_UV":"",e.gradientMap?"#define USE_GRADIENTMAP":"",e.flatShading?"#define FLAT_SHADED":"",e.doubleSided?"#define DOUBLE_SIDED":"",e.flipSided?"#define FLIP_SIDED":"",e.shadowMapEnabled?"#define USE_SHADOWMAP":"",e.shadowMapEnabled?"#define "+c:"",e.premultipliedAlpha?"#define PREMULTIPLIED_ALPHA":"",e.numLightProbes>0?"#define USE_LIGHT_PROBES":"",e.useLegacyLights?"#define LEGACY_LIGHTS":"",e.decodeVideoTexture?"#define DECODE_VIDEO_TEXTURE":"",e.logarithmicDepthBuffer?"#define USE_LOGDEPTHBUF":"",e.logarithmicDepthBuffer&&e.rendererExtensionFragDepth?"#define USE_LOGDEPTHBUF_EXT":"","uniform mat4 viewMatrix;","uniform vec3 cameraPosition;","uniform bool isOrthographic;",e.toneMapping!==0?"#define TONE_MAPPING":"",e.toneMapping!==0?ee.tonemapping_pars_fragment:"",e.toneMapping!==0?k_("toneMapping",e.toneMapping):"",e.dithering?"#define DITHERING":"",e.opaque?"#define OPAQUE":"",ee.colorspace_pars_fragment,V_("linearToOutputTexel",e.outputColorSpace),e.useDepthPacking?"#define DEPTH_PACKING "+e.depthPacking:"",`
`].filter(ur).join(`
`)),a=eh(a),a=Gu(a,e),a=Vu(a,e),o=eh(o),o=Gu(o,e),o=Vu(o,e),a=ku(a),o=ku(o),e.isWebGL2&&e.isRawShaderMaterial!==!0&&(v=`#version 300 es
`,p=[g,"precision mediump sampler2DArray;","#define attribute in","#define varying out","#define texture2D texture"].join(`
`)+`
`+p,_=["precision mediump sampler2DArray;","#define varying in",e.glslVersion===ou?"":"layout(location = 0) out highp vec4 pc_fragColor;",e.glslVersion===ou?"":"#define gl_FragColor pc_fragColor","#define gl_FragDepthEXT gl_FragDepth","#define texture2D texture","#define textureCube texture","#define texture2DProj textureProj","#define texture2DLodEXT textureLod","#define texture2DProjLodEXT textureProjLod","#define textureCubeLodEXT textureLod","#define texture2DGradEXT textureGrad","#define texture2DProjGradEXT textureProjGrad","#define textureCubeGradEXT textureGrad"].join(`
`)+`
`+_);const S=v+p+a,M=v+_+o,y=zu(i,i.VERTEX_SHADER,S),w=zu(i,i.FRAGMENT_SHADER,M);i.attachShader(m,y),i.attachShader(m,w),e.index0AttributeName!==void 0?i.bindAttribLocation(m,0,e.index0AttributeName):e.morphTargets===!0&&i.bindAttribLocation(m,0,"position"),i.linkProgram(m);function A(C){if(r.debug.checkShaderErrors){const F=i.getProgramInfoLog(m).trim(),D=i.getShaderInfoLog(y).trim(),N=i.getShaderInfoLog(w).trim();let B=!0,G=!0;if(i.getProgramParameter(m,i.LINK_STATUS)===!1)if(B=!1,typeof r.debug.onShaderError=="function")r.debug.onShaderError(i,m,y,w);else{const z=Ou(i,y,"vertex"),H=Ou(i,w,"fragment");console.error("THREE.WebGLProgram: Shader Error "+i.getError()+" - VALIDATE_STATUS "+i.getProgramParameter(m,i.VALIDATE_STATUS)+`

Program Info Log: `+F+`
`+z+`
`+H)}else F!==""?console.warn("THREE.WebGLProgram: Program Info Log:",F):(D===""||N==="")&&(G=!1);G&&(C.diagnostics={runnable:B,programLog:F,vertexShader:{log:D,prefix:p},fragmentShader:{log:N,prefix:_}})}i.deleteShader(y),i.deleteShader(w),E=new Go(i,m),T=Y_(i,m)}let E;this.getUniforms=function(){return E===void 0&&A(this),E};let T;this.getAttributes=function(){return T===void 0&&A(this),T};let L=e.rendererExtensionParallelShaderCompile===!1;return this.isReady=function(){return L===!1&&(L=i.getProgramParameter(m,U_)),L},this.destroy=function(){n.releaseStatesOfProgram(this),i.deleteProgram(m),this.program=void 0},this.type=e.shaderType,this.name=e.shaderName,this.id=z_++,this.cacheKey=t,this.usedTimes=1,this.program=m,this.vertexShader=y,this.fragmentShader=w,this}let sv=0;class rv{constructor(){this.shaderCache=new Map,this.materialCache=new Map}update(t){const e=t.vertexShader,n=t.fragmentShader,i=this._getShaderStage(e),s=this._getShaderStage(n),a=this._getShaderCacheForMaterial(t);return a.has(i)===!1&&(a.add(i),i.usedTimes++),a.has(s)===!1&&(a.add(s),s.usedTimes++),this}remove(t){const e=this.materialCache.get(t);for(const n of e)n.usedTimes--,n.usedTimes===0&&this.shaderCache.delete(n.code);return this.materialCache.delete(t),this}getVertexShaderID(t){return this._getShaderStage(t.vertexShader).id}getFragmentShaderID(t){return this._getShaderStage(t.fragmentShader).id}dispose(){this.shaderCache.clear(),this.materialCache.clear()}_getShaderCacheForMaterial(t){const e=this.materialCache;let n=e.get(t);return n===void 0&&(n=new Set,e.set(t,n)),n}_getShaderStage(t){const e=this.shaderCache;let n=e.get(t);return n===void 0&&(n=new av(t),e.set(t,n)),n}}class av{constructor(t){this.id=sv++,this.code=t,this.usedTimes=0}}function ov(r,t,e,n,i,s,a){const o=new Gf,c=new rv,l=[],h=i.isWebGL2,u=i.logarithmicDepthBuffer,d=i.vertexTextures;let f=i.precision;const g={MeshDepthMaterial:"depth",MeshDistanceMaterial:"distanceRGBA",MeshNormalMaterial:"normal",MeshBasicMaterial:"basic",MeshLambertMaterial:"lambert",MeshPhongMaterial:"phong",MeshToonMaterial:"toon",MeshStandardMaterial:"physical",MeshPhysicalMaterial:"physical",MeshMatcapMaterial:"matcap",LineBasicMaterial:"basic",LineDashedMaterial:"dashed",PointsMaterial:"points",ShadowMaterial:"shadow",SpriteMaterial:"sprite"};function x(E){return E===0?"uv":`uv${E}`}function m(E,T,L,C,F){const D=C.fog,N=F.geometry,B=E.isMeshStandardMaterial?C.environment:null,G=(E.isMeshStandardMaterial?e:t).get(E.envMap||B),z=G&&G.mapping===306?G.image.height:null,H=g[E.type];E.precision!==null&&(f=i.getMaxPrecision(E.precision),f!==E.precision&&console.warn("THREE.WebGLProgram.getParameters:",E.precision,"not supported, using",f,"instead."));const j=N.morphAttributes.position||N.morphAttributes.normal||N.morphAttributes.color,$=j!==void 0?j.length:0;let Q=0;N.morphAttributes.position!==void 0&&(Q=1),N.morphAttributes.normal!==void 0&&(Q=2),N.morphAttributes.color!==void 0&&(Q=3);let V,K,nt,rt;if(H){const we=fi[H];V=we.vertexShader,K=we.fragmentShader}else V=E.vertexShader,K=E.fragmentShader,c.update(E),nt=c.getVertexShaderID(E),rt=c.getFragmentShaderID(E);const ot=r.getRenderTarget(),_t=F.isInstancedMesh===!0,ht=F.isBatchedMesh===!0,ut=!!E.map,Ct=!!E.matcap,W=!!G,Nt=!!E.aoMap,Tt=!!E.lightMap,Pt=!!E.bumpMap,yt=!!E.normalMap,Kt=!!E.displacementMap,bt=!!E.emissiveMap,I=!!E.metalnessMap,b=!!E.roughnessMap,X=E.anisotropy>0,et=E.clearcoat>0,J=E.iridescence>0,tt=E.sheen>0,St=E.transmission>0,lt=X&&!!E.anisotropyMap,gt=et&&!!E.clearcoatMap,At=et&&!!E.clearcoatNormalMap,Ht=et&&!!E.clearcoatRoughnessMap,it=J&&!!E.iridescenceMap,ce=J&&!!E.iridescenceThicknessMap,Jt=tt&&!!E.sheenColorMap,Gt=tt&&!!E.sheenRoughnessMap,Lt=!!E.specularMap,vt=!!E.specularColorMap,Ft=!!E.specularIntensityMap,te=St&&!!E.transmissionMap,le=St&&!!E.thicknessMap,Wt=!!E.gradientMap,ct=!!E.alphaMap,U=E.alphaTest>0,dt=!!E.alphaHash,ft=!!E.extensions,zt=!!N.attributes.uv1,Bt=!!N.attributes.uv2,de=!!N.attributes.uv3;let ue=0;return E.toneMapped&&(ot===null||ot.isXRRenderTarget===!0)&&(ue=r.toneMapping),{isWebGL2:h,shaderID:H,shaderType:E.type,shaderName:E.name,vertexShader:V,fragmentShader:K,defines:E.defines,customVertexShaderID:nt,customFragmentShaderID:rt,isRawShaderMaterial:E.isRawShaderMaterial===!0,glslVersion:E.glslVersion,precision:f,batching:ht,instancing:_t,instancingColor:_t&&F.instanceColor!==null,supportsVertexTextures:d,outputColorSpace:ot===null?r.outputColorSpace:ot.isXRRenderTarget===!0?ot.texture.colorSpace:Ui,map:ut,matcap:Ct,envMap:W,envMapMode:W&&G.mapping,envMapCubeUVHeight:z,aoMap:Nt,lightMap:Tt,bumpMap:Pt,normalMap:yt,displacementMap:d&&Kt,emissiveMap:bt,normalMapObjectSpace:yt&&E.normalMapType===1,normalMapTangentSpace:yt&&E.normalMapType===0,metalnessMap:I,roughnessMap:b,anisotropy:X,anisotropyMap:lt,clearcoat:et,clearcoatMap:gt,clearcoatNormalMap:At,clearcoatRoughnessMap:Ht,iridescence:J,iridescenceMap:it,iridescenceThicknessMap:ce,sheen:tt,sheenColorMap:Jt,sheenRoughnessMap:Gt,specularMap:Lt,specularColorMap:vt,specularIntensityMap:Ft,transmission:St,transmissionMap:te,thicknessMap:le,gradientMap:Wt,opaque:E.transparent===!1&&E.blending===1,alphaMap:ct,alphaTest:U,alphaHash:dt,combine:E.combine,mapUv:ut&&x(E.map.channel),aoMapUv:Nt&&x(E.aoMap.channel),lightMapUv:Tt&&x(E.lightMap.channel),bumpMapUv:Pt&&x(E.bumpMap.channel),normalMapUv:yt&&x(E.normalMap.channel),displacementMapUv:Kt&&x(E.displacementMap.channel),emissiveMapUv:bt&&x(E.emissiveMap.channel),metalnessMapUv:I&&x(E.metalnessMap.channel),roughnessMapUv:b&&x(E.roughnessMap.channel),anisotropyMapUv:lt&&x(E.anisotropyMap.channel),clearcoatMapUv:gt&&x(E.clearcoatMap.channel),clearcoatNormalMapUv:At&&x(E.clearcoatNormalMap.channel),clearcoatRoughnessMapUv:Ht&&x(E.clearcoatRoughnessMap.channel),iridescenceMapUv:it&&x(E.iridescenceMap.channel),iridescenceThicknessMapUv:ce&&x(E.iridescenceThicknessMap.channel),sheenColorMapUv:Jt&&x(E.sheenColorMap.channel),sheenRoughnessMapUv:Gt&&x(E.sheenRoughnessMap.channel),specularMapUv:Lt&&x(E.specularMap.channel),specularColorMapUv:vt&&x(E.specularColorMap.channel),specularIntensityMapUv:Ft&&x(E.specularIntensityMap.channel),transmissionMapUv:te&&x(E.transmissionMap.channel),thicknessMapUv:le&&x(E.thicknessMap.channel),alphaMapUv:ct&&x(E.alphaMap.channel),vertexTangents:!!N.attributes.tangent&&(yt||X),vertexColors:E.vertexColors,vertexAlphas:E.vertexColors===!0&&!!N.attributes.color&&N.attributes.color.itemSize===4,vertexUv1s:zt,vertexUv2s:Bt,vertexUv3s:de,pointsUvs:F.isPoints===!0&&!!N.attributes.uv&&(ut||ct),fog:!!D,useFog:E.fog===!0,fogExp2:D&&D.isFogExp2,flatShading:E.flatShading===!0,sizeAttenuation:E.sizeAttenuation===!0,logarithmicDepthBuffer:u,skinning:F.isSkinnedMesh===!0,morphTargets:N.morphAttributes.position!==void 0,morphNormals:N.morphAttributes.normal!==void 0,morphColors:N.morphAttributes.color!==void 0,morphTargetsCount:$,morphTextureStride:Q,numDirLights:T.directional.length,numPointLights:T.point.length,numSpotLights:T.spot.length,numSpotLightMaps:T.spotLightMap.length,numRectAreaLights:T.rectArea.length,numHemiLights:T.hemi.length,numDirLightShadows:T.directionalShadowMap.length,numPointLightShadows:T.pointShadowMap.length,numSpotLightShadows:T.spotShadowMap.length,numSpotLightShadowsWithMaps:T.numSpotLightShadowsWithMaps,numLightProbes:T.numLightProbes,numClippingPlanes:a.numPlanes,numClipIntersection:a.numIntersection,dithering:E.dithering,shadowMapEnabled:r.shadowMap.enabled&&L.length>0,shadowMapType:r.shadowMap.type,toneMapping:ue,useLegacyLights:r._useLegacyLights,decodeVideoTexture:ut&&E.map.isVideoTexture===!0&&pe.getTransfer(E.map.colorSpace)===ye,premultipliedAlpha:E.premultipliedAlpha,doubleSided:E.side===2,flipSided:E.side===1,useDepthPacking:E.depthPacking>=0,depthPacking:E.depthPacking||0,index0AttributeName:E.index0AttributeName,extensionDerivatives:ft&&E.extensions.derivatives===!0,extensionFragDepth:ft&&E.extensions.fragDepth===!0,extensionDrawBuffers:ft&&E.extensions.drawBuffers===!0,extensionShaderTextureLOD:ft&&E.extensions.shaderTextureLOD===!0,extensionClipCullDistance:ft&&E.extensions.clipCullDistance&&n.has("WEBGL_clip_cull_distance"),rendererExtensionFragDepth:h||n.has("EXT_frag_depth"),rendererExtensionDrawBuffers:h||n.has("WEBGL_draw_buffers"),rendererExtensionShaderTextureLod:h||n.has("EXT_shader_texture_lod"),rendererExtensionParallelShaderCompile:n.has("KHR_parallel_shader_compile"),customProgramCacheKey:E.customProgramCacheKey()}}function p(E){const T=[];if(E.shaderID?T.push(E.shaderID):(T.push(E.customVertexShaderID),T.push(E.customFragmentShaderID)),E.defines!==void 0)for(const L in E.defines)T.push(L),T.push(E.defines[L]);return E.isRawShaderMaterial===!1&&(_(T,E),v(T,E),T.push(r.outputColorSpace)),T.push(E.customProgramCacheKey),T.join()}function _(E,T){E.push(T.precision),E.push(T.outputColorSpace),E.push(T.envMapMode),E.push(T.envMapCubeUVHeight),E.push(T.mapUv),E.push(T.alphaMapUv),E.push(T.lightMapUv),E.push(T.aoMapUv),E.push(T.bumpMapUv),E.push(T.normalMapUv),E.push(T.displacementMapUv),E.push(T.emissiveMapUv),E.push(T.metalnessMapUv),E.push(T.roughnessMapUv),E.push(T.anisotropyMapUv),E.push(T.clearcoatMapUv),E.push(T.clearcoatNormalMapUv),E.push(T.clearcoatRoughnessMapUv),E.push(T.iridescenceMapUv),E.push(T.iridescenceThicknessMapUv),E.push(T.sheenColorMapUv),E.push(T.sheenRoughnessMapUv),E.push(T.specularMapUv),E.push(T.specularColorMapUv),E.push(T.specularIntensityMapUv),E.push(T.transmissionMapUv),E.push(T.thicknessMapUv),E.push(T.combine),E.push(T.fogExp2),E.push(T.sizeAttenuation),E.push(T.morphTargetsCount),E.push(T.morphAttributeCount),E.push(T.numDirLights),E.push(T.numPointLights),E.push(T.numSpotLights),E.push(T.numSpotLightMaps),E.push(T.numHemiLights),E.push(T.numRectAreaLights),E.push(T.numDirLightShadows),E.push(T.numPointLightShadows),E.push(T.numSpotLightShadows),E.push(T.numSpotLightShadowsWithMaps),E.push(T.numLightProbes),E.push(T.shadowMapType),E.push(T.toneMapping),E.push(T.numClippingPlanes),E.push(T.numClipIntersection),E.push(T.depthPacking)}function v(E,T){o.disableAll(),T.isWebGL2&&o.enable(0),T.supportsVertexTextures&&o.enable(1),T.instancing&&o.enable(2),T.instancingColor&&o.enable(3),T.matcap&&o.enable(4),T.envMap&&o.enable(5),T.normalMapObjectSpace&&o.enable(6),T.normalMapTangentSpace&&o.enable(7),T.clearcoat&&o.enable(8),T.iridescence&&o.enable(9),T.alphaTest&&o.enable(10),T.vertexColors&&o.enable(11),T.vertexAlphas&&o.enable(12),T.vertexUv1s&&o.enable(13),T.vertexUv2s&&o.enable(14),T.vertexUv3s&&o.enable(15),T.vertexTangents&&o.enable(16),T.anisotropy&&o.enable(17),T.alphaHash&&o.enable(18),T.batching&&o.enable(19),E.push(o.mask),o.disableAll(),T.fog&&o.enable(0),T.useFog&&o.enable(1),T.flatShading&&o.enable(2),T.logarithmicDepthBuffer&&o.enable(3),T.skinning&&o.enable(4),T.morphTargets&&o.enable(5),T.morphNormals&&o.enable(6),T.morphColors&&o.enable(7),T.premultipliedAlpha&&o.enable(8),T.shadowMapEnabled&&o.enable(9),T.useLegacyLights&&o.enable(10),T.doubleSided&&o.enable(11),T.flipSided&&o.enable(12),T.useDepthPacking&&o.enable(13),T.dithering&&o.enable(14),T.transmission&&o.enable(15),T.sheen&&o.enable(16),T.opaque&&o.enable(17),T.pointsUvs&&o.enable(18),T.decodeVideoTexture&&o.enable(19),E.push(o.mask)}function S(E){const T=g[E.type];let L;if(T){const C=fi[T];L=Hm.clone(C.uniforms)}else L=E.uniforms;return L}function M(E,T){let L;for(let C=0,F=l.length;C<F;C++){const D=l[C];if(D.cacheKey===T){L=D,++L.usedTimes;break}}return L===void 0&&(L=new iv(r,T,E,s),l.push(L)),L}function y(E){if(--E.usedTimes===0){const T=l.indexOf(E);l[T]=l[l.length-1],l.pop(),E.destroy()}}function w(E){c.remove(E)}function A(){c.dispose()}return{getParameters:m,getProgramCacheKey:p,getUniforms:S,acquireProgram:M,releaseProgram:y,releaseShaderCache:w,programs:l,dispose:A}}function cv(){let r=new WeakMap;function t(s){let a=r.get(s);return a===void 0&&(a={},r.set(s,a)),a}function e(s){r.delete(s)}function n(s,a,o){r.get(s)[a]=o}function i(){r=new WeakMap}return{get:t,remove:e,update:n,dispose:i}}function lv(r,t){return r.groupOrder!==t.groupOrder?r.groupOrder-t.groupOrder:r.renderOrder!==t.renderOrder?r.renderOrder-t.renderOrder:r.material.id!==t.material.id?r.material.id-t.material.id:r.z!==t.z?r.z-t.z:r.id-t.id}function Wu(r,t){return r.groupOrder!==t.groupOrder?r.groupOrder-t.groupOrder:r.renderOrder!==t.renderOrder?r.renderOrder-t.renderOrder:r.z!==t.z?t.z-r.z:r.id-t.id}function Xu(){const r=[];let t=0;const e=[],n=[],i=[];function s(){t=0,e.length=0,n.length=0,i.length=0}function a(u,d,f,g,x,m){let p=r[t];return p===void 0?(p={id:u.id,object:u,geometry:d,material:f,groupOrder:g,renderOrder:u.renderOrder,z:x,group:m},r[t]=p):(p.id=u.id,p.object=u,p.geometry=d,p.material=f,p.groupOrder=g,p.renderOrder=u.renderOrder,p.z=x,p.group=m),t++,p}function o(u,d,f,g,x,m){const p=a(u,d,f,g,x,m);f.transmission>0?n.push(p):f.transparent===!0?i.push(p):e.push(p)}function c(u,d,f,g,x,m){const p=a(u,d,f,g,x,m);f.transmission>0?n.unshift(p):f.transparent===!0?i.unshift(p):e.unshift(p)}function l(u,d){e.length>1&&e.sort(u||lv),n.length>1&&n.sort(d||Wu),i.length>1&&i.sort(d||Wu)}function h(){for(let u=t,d=r.length;u<d;u++){const f=r[u];if(f.id===null)break;f.id=null,f.object=null,f.geometry=null,f.material=null,f.group=null}}return{opaque:e,transmissive:n,transparent:i,init:s,push:o,unshift:c,finish:h,sort:l}}function hv(){let r=new WeakMap;function t(n,i){const s=r.get(n);let a;return s===void 0?(a=new Xu,r.set(n,[a])):i>=s.length?(a=new Xu,s.push(a)):a=s[i],a}function e(){r=new WeakMap}return{get:t,dispose:e}}function uv(){const r={};return{get:function(t){if(r[t.id]!==void 0)return r[t.id];let e;switch(t.type){case"DirectionalLight":e={direction:new R,color:new Qt};break;case"SpotLight":e={position:new R,direction:new R,color:new Qt,distance:0,coneCos:0,penumbraCos:0,decay:0};break;case"PointLight":e={position:new R,color:new Qt,distance:0,decay:0};break;case"HemisphereLight":e={direction:new R,skyColor:new Qt,groundColor:new Qt};break;case"RectAreaLight":e={color:new Qt,position:new R,halfWidth:new R,halfHeight:new R};break}return r[t.id]=e,e}}}function dv(){const r={};return{get:function(t){if(r[t.id]!==void 0)return r[t.id];let e;switch(t.type){case"DirectionalLight":e={shadowBias:0,shadowNormalBias:0,shadowRadius:1,shadowMapSize:new Mt};break;case"SpotLight":e={shadowBias:0,shadowNormalBias:0,shadowRadius:1,shadowMapSize:new Mt};break;case"PointLight":e={shadowBias:0,shadowNormalBias:0,shadowRadius:1,shadowMapSize:new Mt,shadowCameraNear:1,shadowCameraFar:1e3};break}return r[t.id]=e,e}}}let fv=0;function pv(r,t){return(t.castShadow?2:0)-(r.castShadow?2:0)+(t.map?1:0)-(r.map?1:0)}function mv(r,t){const e=new uv,n=dv(),i={version:0,hash:{directionalLength:-1,pointLength:-1,spotLength:-1,rectAreaLength:-1,hemiLength:-1,numDirectionalShadows:-1,numPointShadows:-1,numSpotShadows:-1,numSpotMaps:-1,numLightProbes:-1},ambient:[0,0,0],probe:[],directional:[],directionalShadow:[],directionalShadowMap:[],directionalShadowMatrix:[],spot:[],spotLightMap:[],spotShadow:[],spotShadowMap:[],spotLightMatrix:[],rectArea:[],rectAreaLTC1:null,rectAreaLTC2:null,point:[],pointShadow:[],pointShadowMap:[],pointShadowMatrix:[],hemi:[],numSpotLightShadowsWithMaps:0,numLightProbes:0};for(let h=0;h<9;h++)i.probe.push(new R);const s=new R,a=new re,o=new re;function c(h,u){let d=0,f=0,g=0;for(let C=0;C<9;C++)i.probe[C].set(0,0,0);let x=0,m=0,p=0,_=0,v=0,S=0,M=0,y=0,w=0,A=0,E=0;h.sort(pv);const T=u===!0?Math.PI:1;for(let C=0,F=h.length;C<F;C++){const D=h[C],N=D.color,B=D.intensity,G=D.distance,z=D.shadow&&D.shadow.map?D.shadow.map.texture:null;if(D.isAmbientLight)d+=N.r*B*T,f+=N.g*B*T,g+=N.b*B*T;else if(D.isLightProbe){for(let H=0;H<9;H++)i.probe[H].addScaledVector(D.sh.coefficients[H],B);E++}else if(D.isDirectionalLight){const H=e.get(D);if(H.color.copy(D.color).multiplyScalar(D.intensity*T),D.castShadow){const j=D.shadow,$=n.get(D);$.shadowBias=j.bias,$.shadowNormalBias=j.normalBias,$.shadowRadius=j.radius,$.shadowMapSize=j.mapSize,i.directionalShadow[x]=$,i.directionalShadowMap[x]=z,i.directionalShadowMatrix[x]=D.shadow.matrix,S++}i.directional[x]=H,x++}else if(D.isSpotLight){const H=e.get(D);H.position.setFromMatrixPosition(D.matrixWorld),H.color.copy(N).multiplyScalar(B*T),H.distance=G,H.coneCos=Math.cos(D.angle),H.penumbraCos=Math.cos(D.angle*(1-D.penumbra)),H.decay=D.decay,i.spot[p]=H;const j=D.shadow;if(D.map&&(i.spotLightMap[w]=D.map,w++,j.updateMatrices(D),D.castShadow&&A++),i.spotLightMatrix[p]=j.matrix,D.castShadow){const $=n.get(D);$.shadowBias=j.bias,$.shadowNormalBias=j.normalBias,$.shadowRadius=j.radius,$.shadowMapSize=j.mapSize,i.spotShadow[p]=$,i.spotShadowMap[p]=z,y++}p++}else if(D.isRectAreaLight){const H=e.get(D);H.color.copy(N).multiplyScalar(B),H.halfWidth.set(D.width*.5,0,0),H.halfHeight.set(0,D.height*.5,0),i.rectArea[_]=H,_++}else if(D.isPointLight){const H=e.get(D);if(H.color.copy(D.color).multiplyScalar(D.intensity*T),H.distance=D.distance,H.decay=D.decay,D.castShadow){const j=D.shadow,$=n.get(D);$.shadowBias=j.bias,$.shadowNormalBias=j.normalBias,$.shadowRadius=j.radius,$.shadowMapSize=j.mapSize,$.shadowCameraNear=j.camera.near,$.shadowCameraFar=j.camera.far,i.pointShadow[m]=$,i.pointShadowMap[m]=z,i.pointShadowMatrix[m]=D.shadow.matrix,M++}i.point[m]=H,m++}else if(D.isHemisphereLight){const H=e.get(D);H.skyColor.copy(D.color).multiplyScalar(B*T),H.groundColor.copy(D.groundColor).multiplyScalar(B*T),i.hemi[v]=H,v++}}_>0&&(t.isWebGL2?r.has("OES_texture_float_linear")===!0?(i.rectAreaLTC1=pt.LTC_FLOAT_1,i.rectAreaLTC2=pt.LTC_FLOAT_2):(i.rectAreaLTC1=pt.LTC_HALF_1,i.rectAreaLTC2=pt.LTC_HALF_2):r.has("OES_texture_float_linear")===!0?(i.rectAreaLTC1=pt.LTC_FLOAT_1,i.rectAreaLTC2=pt.LTC_FLOAT_2):r.has("OES_texture_half_float_linear")===!0?(i.rectAreaLTC1=pt.LTC_HALF_1,i.rectAreaLTC2=pt.LTC_HALF_2):console.error("THREE.WebGLRenderer: Unable to use RectAreaLight. Missing WebGL extensions.")),i.ambient[0]=d,i.ambient[1]=f,i.ambient[2]=g;const L=i.hash;(L.directionalLength!==x||L.pointLength!==m||L.spotLength!==p||L.rectAreaLength!==_||L.hemiLength!==v||L.numDirectionalShadows!==S||L.numPointShadows!==M||L.numSpotShadows!==y||L.numSpotMaps!==w||L.numLightProbes!==E)&&(i.directional.length=x,i.spot.length=p,i.rectArea.length=_,i.point.length=m,i.hemi.length=v,i.directionalShadow.length=S,i.directionalShadowMap.length=S,i.pointShadow.length=M,i.pointShadowMap.length=M,i.spotShadow.length=y,i.spotShadowMap.length=y,i.directionalShadowMatrix.length=S,i.pointShadowMatrix.length=M,i.spotLightMatrix.length=y+w-A,i.spotLightMap.length=w,i.numSpotLightShadowsWithMaps=A,i.numLightProbes=E,L.directionalLength=x,L.pointLength=m,L.spotLength=p,L.rectAreaLength=_,L.hemiLength=v,L.numDirectionalShadows=S,L.numPointShadows=M,L.numSpotShadows=y,L.numSpotMaps=w,L.numLightProbes=E,i.version=fv++)}function l(h,u){let d=0,f=0,g=0,x=0,m=0;const p=u.matrixWorldInverse;for(let _=0,v=h.length;_<v;_++){const S=h[_];if(S.isDirectionalLight){const M=i.directional[d];M.direction.setFromMatrixPosition(S.matrixWorld),s.setFromMatrixPosition(S.target.matrixWorld),M.direction.sub(s),M.direction.transformDirection(p),d++}else if(S.isSpotLight){const M=i.spot[g];M.position.setFromMatrixPosition(S.matrixWorld),M.position.applyMatrix4(p),M.direction.setFromMatrixPosition(S.matrixWorld),s.setFromMatrixPosition(S.target.matrixWorld),M.direction.sub(s),M.direction.transformDirection(p),g++}else if(S.isRectAreaLight){const M=i.rectArea[x];M.position.setFromMatrixPosition(S.matrixWorld),M.position.applyMatrix4(p),o.identity(),a.copy(S.matrixWorld),a.premultiply(p),o.extractRotation(a),M.halfWidth.set(S.width*.5,0,0),M.halfHeight.set(0,S.height*.5,0),M.halfWidth.applyMatrix4(o),M.halfHeight.applyMatrix4(o),x++}else if(S.isPointLight){const M=i.point[f];M.position.setFromMatrixPosition(S.matrixWorld),M.position.applyMatrix4(p),f++}else if(S.isHemisphereLight){const M=i.hemi[m];M.direction.setFromMatrixPosition(S.matrixWorld),M.direction.transformDirection(p),m++}}}return{setup:c,setupView:l,state:i}}function Yu(r,t){const e=new mv(r,t),n=[],i=[];function s(){n.length=0,i.length=0}function a(u){n.push(u)}function o(u){i.push(u)}function c(u){e.setup(n,u)}function l(u){e.setupView(n,u)}return{init:s,state:{lightsArray:n,shadowsArray:i,lights:e},setupLights:c,setupLightsView:l,pushLight:a,pushShadow:o}}function gv(r,t){let e=new WeakMap;function n(s,a=0){const o=e.get(s);let c;return o===void 0?(c=new Yu(r,t),e.set(s,[c])):a>=o.length?(c=new Yu(r,t),o.push(c)):c=o[a],c}function i(){e=new WeakMap}return{get:n,dispose:i}}class xv extends _i{constructor(t){super(),this.isMeshDepthMaterial=!0,this.type="MeshDepthMaterial",this.depthPacking=3200,this.map=null,this.alphaMap=null,this.displacementMap=null,this.displacementScale=1,this.displacementBias=0,this.wireframe=!1,this.wireframeLinewidth=1,this.setValues(t)}copy(t){return super.copy(t),this.depthPacking=t.depthPacking,this.map=t.map,this.alphaMap=t.alphaMap,this.displacementMap=t.displacementMap,this.displacementScale=t.displacementScale,this.displacementBias=t.displacementBias,this.wireframe=t.wireframe,this.wireframeLinewidth=t.wireframeLinewidth,this}}class _v extends _i{constructor(t){super(),this.isMeshDistanceMaterial=!0,this.type="MeshDistanceMaterial",this.map=null,this.alphaMap=null,this.displacementMap=null,this.displacementScale=1,this.displacementBias=0,this.setValues(t)}copy(t){return super.copy(t),this.map=t.map,this.alphaMap=t.alphaMap,this.displacementMap=t.displacementMap,this.displacementScale=t.displacementScale,this.displacementBias=t.displacementBias,this}}const vv=`void main() {
	gl_Position = vec4( position, 1.0 );
}`,Sv=`uniform sampler2D shadow_pass;
uniform vec2 resolution;
uniform float radius;
#include <packing>
void main() {
	const float samples = float( VSM_SAMPLES );
	float mean = 0.0;
	float squared_mean = 0.0;
	float uvStride = samples <= 1.0 ? 0.0 : 2.0 / ( samples - 1.0 );
	float uvStart = samples <= 1.0 ? 0.0 : - 1.0;
	for ( float i = 0.0; i < samples; i ++ ) {
		float uvOffset = uvStart + i * uvStride;
		#ifdef HORIZONTAL_PASS
			vec2 distribution = unpackRGBATo2Half( texture2D( shadow_pass, ( gl_FragCoord.xy + vec2( uvOffset, 0.0 ) * radius ) / resolution ) );
			mean += distribution.x;
			squared_mean += distribution.y * distribution.y + distribution.x * distribution.x;
		#else
			float depth = unpackRGBAToDepth( texture2D( shadow_pass, ( gl_FragCoord.xy + vec2( 0.0, uvOffset ) * radius ) / resolution ) );
			mean += depth;
			squared_mean += depth * depth;
		#endif
	}
	mean = mean / samples;
	squared_mean = squared_mean / samples;
	float std_dev = sqrt( squared_mean - mean * mean );
	gl_FragColor = pack2HalfToRGBA( vec2( mean, std_dev ) );
}`;function Mv(r,t,e){let n=new Bh;const i=new Mt,s=new Mt,a=new en,o=new xv({depthPacking:3201}),c=new _v,l={},h=e.maxTextureSize,u={0:1,1:0,2:2},d=new Bn({defines:{VSM_SAMPLES:8},uniforms:{shadow_pass:{value:null},resolution:{value:new Mt},radius:{value:4}},vertexShader:vv,fragmentShader:Sv}),f=d.clone();f.defines.HORIZONTAL_PASS=1;const g=new me;g.setAttribute("position",new Be(new Float32Array([-1,-1,.5,3,-1,.5,-1,3,.5]),3));const x=new jt(g,d),m=this;this.enabled=!1,this.autoUpdate=!0,this.needsUpdate=!1,this.type=1;let p=this.type;this.render=function(y,w,A){if(m.enabled===!1||m.autoUpdate===!1&&m.needsUpdate===!1||y.length===0)return;const E=r.getRenderTarget(),T=r.getActiveCubeFace(),L=r.getActiveMipmapLevel(),C=r.state;C.setBlending(0),C.buffers.color.setClear(1,1,1,1),C.buffers.depth.setTest(!0),C.setScissorTest(!1);const F=p!==3&&this.type===3,D=p===3&&this.type!==3;for(let N=0,B=y.length;N<B;N++){const G=y[N],z=G.shadow;if(z===void 0){console.warn("THREE.WebGLShadowMap:",G,"has no shadow.");continue}if(z.autoUpdate===!1&&z.needsUpdate===!1)continue;i.copy(z.mapSize);const H=z.getFrameExtents();if(i.multiply(H),s.copy(z.mapSize),(i.x>h||i.y>h)&&(i.x>h&&(s.x=Math.floor(h/H.x),i.x=s.x*H.x,z.mapSize.x=s.x),i.y>h&&(s.y=Math.floor(h/H.y),i.y=s.y*H.y,z.mapSize.y=s.y)),z.map===null||F===!0||D===!0){const $=this.type!==3?{minFilter:1003,magFilter:1003}:{};z.map!==null&&z.map.dispose(),z.map=new sn(i.x,i.y,$),z.map.texture.name=G.name+".shadowMap",z.camera.updateProjectionMatrix()}r.setRenderTarget(z.map),r.clear();const j=z.getViewportCount();for(let $=0;$<j;$++){const Q=z.getViewport($);a.set(s.x*Q.x,s.y*Q.y,s.x*Q.z,s.y*Q.w),C.viewport(a),z.updateMatrices(G,$),n=z.getFrustum(),S(w,A,z.camera,G,this.type)}z.isPointLightShadow!==!0&&this.type===3&&_(z,A),z.needsUpdate=!1}p=this.type,m.needsUpdate=!1,r.setRenderTarget(E,T,L)};function _(y,w){const A=t.update(x);d.defines.VSM_SAMPLES!==y.blurSamples&&(d.defines.VSM_SAMPLES=y.blurSamples,f.defines.VSM_SAMPLES=y.blurSamples,d.needsUpdate=!0,f.needsUpdate=!0),y.mapPass===null&&(y.mapPass=new sn(i.x,i.y)),d.uniforms.shadow_pass.value=y.map.texture,d.uniforms.resolution.value=y.mapSize,d.uniforms.radius.value=y.radius,r.setRenderTarget(y.mapPass),r.clear(),r.renderBufferDirect(w,null,A,d,x,null),f.uniforms.shadow_pass.value=y.mapPass.texture,f.uniforms.resolution.value=y.mapSize,f.uniforms.radius.value=y.radius,r.setRenderTarget(y.map),r.clear(),r.renderBufferDirect(w,null,A,f,x,null)}function v(y,w,A,E){let T=null;const L=A.isPointLight===!0?y.customDistanceMaterial:y.customDepthMaterial;if(L!==void 0)T=L;else if(T=A.isPointLight===!0?c:o,r.localClippingEnabled&&w.clipShadows===!0&&Array.isArray(w.clippingPlanes)&&w.clippingPlanes.length!==0||w.displacementMap&&w.displacementScale!==0||w.alphaMap&&w.alphaTest>0||w.map&&w.alphaTest>0){const C=T.uuid,F=w.uuid;let D=l[C];D===void 0&&(D={},l[C]=D);let N=D[F];N===void 0&&(N=T.clone(),D[F]=N,w.addEventListener("dispose",M)),T=N}if(T.visible=w.visible,T.wireframe=w.wireframe,E===3?T.side=w.shadowSide!==null?w.shadowSide:w.side:T.side=w.shadowSide!==null?w.shadowSide:u[w.side],T.alphaMap=w.alphaMap,T.alphaTest=w.alphaTest,T.map=w.map,T.clipShadows=w.clipShadows,T.clippingPlanes=w.clippingPlanes,T.clipIntersection=w.clipIntersection,T.displacementMap=w.displacementMap,T.displacementScale=w.displacementScale,T.displacementBias=w.displacementBias,T.wireframeLinewidth=w.wireframeLinewidth,T.linewidth=w.linewidth,A.isPointLight===!0&&T.isMeshDistanceMaterial===!0){const C=r.properties.get(T);C.light=A}return T}function S(y,w,A,E,T){if(y.visible===!1)return;if(y.layers.test(w.layers)&&(y.isMesh||y.isLine||y.isPoints)&&(y.castShadow||y.receiveShadow&&T===3)&&(!y.frustumCulled||n.intersectsObject(y))){y.modelViewMatrix.multiplyMatrices(A.matrixWorldInverse,y.matrixWorld);const F=t.update(y),D=y.material;if(Array.isArray(D)){const N=F.groups;for(let B=0,G=N.length;B<G;B++){const z=N[B],H=D[z.materialIndex];if(H&&H.visible){const j=v(y,H,E,T);y.onBeforeShadow(r,y,w,A,F,j,z),r.renderBufferDirect(A,null,F,j,y,z),y.onAfterShadow(r,y,w,A,F,j,z)}}}else if(D.visible){const N=v(y,D,E,T);y.onBeforeShadow(r,y,w,A,F,N,null),r.renderBufferDirect(A,null,F,N,y,null),y.onAfterShadow(r,y,w,A,F,N,null)}}const C=y.children;for(let F=0,D=C.length;F<D;F++)S(C[F],w,A,E,T)}function M(y){y.target.removeEventListener("dispose",M);for(const A in l){const E=l[A],T=y.target.uuid;T in E&&(E[T].dispose(),delete E[T])}}}function yv(r,t,e){const n=e.isWebGL2;function i(){let U=!1;const dt=new en;let ft=null;const zt=new en(0,0,0,0);return{setMask:function(Bt){ft!==Bt&&!U&&(r.colorMask(Bt,Bt,Bt,Bt),ft=Bt)},setLocked:function(Bt){U=Bt},setClear:function(Bt,de,ue,ve,we){we===!0&&(Bt*=ve,de*=ve,ue*=ve),dt.set(Bt,de,ue,ve),zt.equals(dt)===!1&&(r.clearColor(Bt,de,ue,ve),zt.copy(dt))},reset:function(){U=!1,ft=null,zt.set(-1,0,0,0)}}}function s(){let U=!1,dt=null,ft=null,zt=null;return{setTest:function(Bt){Bt?ht(r.DEPTH_TEST):ut(r.DEPTH_TEST)},setMask:function(Bt){dt!==Bt&&!U&&(r.depthMask(Bt),dt=Bt)},setFunc:function(Bt){if(ft!==Bt){switch(Bt){case 0:r.depthFunc(r.NEVER);break;case 1:r.depthFunc(r.ALWAYS);break;case 2:r.depthFunc(r.LESS);break;case 3:r.depthFunc(r.LEQUAL);break;case 4:r.depthFunc(r.EQUAL);break;case 5:r.depthFunc(r.GEQUAL);break;case 6:r.depthFunc(r.GREATER);break;case 7:r.depthFunc(r.NOTEQUAL);break;default:r.depthFunc(r.LEQUAL)}ft=Bt}},setLocked:function(Bt){U=Bt},setClear:function(Bt){zt!==Bt&&(r.clearDepth(Bt),zt=Bt)},reset:function(){U=!1,dt=null,ft=null,zt=null}}}function a(){let U=!1,dt=null,ft=null,zt=null,Bt=null,de=null,ue=null,ve=null,we=null;return{setTest:function(se){U||(se?ht(r.STENCIL_TEST):ut(r.STENCIL_TEST))},setMask:function(se){dt!==se&&!U&&(r.stencilMask(se),dt=se)},setFunc:function(se,Ce,Ye){(ft!==se||zt!==Ce||Bt!==Ye)&&(r.stencilFunc(se,Ce,Ye),ft=se,zt=Ce,Bt=Ye)},setOp:function(se,Ce,Ye){(de!==se||ue!==Ce||ve!==Ye)&&(r.stencilOp(se,Ce,Ye),de=se,ue=Ce,ve=Ye)},setLocked:function(se){U=se},setClear:function(se){we!==se&&(r.clearStencil(se),we=se)},reset:function(){U=!1,dt=null,ft=null,zt=null,Bt=null,de=null,ue=null,ve=null,we=null}}}const o=new i,c=new s,l=new a,h=new WeakMap,u=new WeakMap;let d={},f={},g=new WeakMap,x=[],m=null,p=!1,_=null,v=null,S=null,M=null,y=null,w=null,A=null,E=new Qt(0,0,0),T=0,L=!1,C=null,F=null,D=null,N=null,B=null;const G=r.getParameter(r.MAX_COMBINED_TEXTURE_IMAGE_UNITS);let z=!1,H=0;const j=r.getParameter(r.VERSION);j.indexOf("WebGL")!==-1?(H=parseFloat(/^WebGL (\d)/.exec(j)[1]),z=H>=1):j.indexOf("OpenGL ES")!==-1&&(H=parseFloat(/^OpenGL ES (\d)/.exec(j)[1]),z=H>=2);let $=null,Q={};const V=r.getParameter(r.SCISSOR_BOX),K=r.getParameter(r.VIEWPORT),nt=new en().fromArray(V),rt=new en().fromArray(K);function ot(U,dt,ft,zt){const Bt=new Uint8Array(4),de=r.createTexture();r.bindTexture(U,de),r.texParameteri(U,r.TEXTURE_MIN_FILTER,r.NEAREST),r.texParameteri(U,r.TEXTURE_MAG_FILTER,r.NEAREST);for(let ue=0;ue<ft;ue++)n&&(U===r.TEXTURE_3D||U===r.TEXTURE_2D_ARRAY)?r.texImage3D(dt,0,r.RGBA,1,1,zt,0,r.RGBA,r.UNSIGNED_BYTE,Bt):r.texImage2D(dt+ue,0,r.RGBA,1,1,0,r.RGBA,r.UNSIGNED_BYTE,Bt);return de}const _t={};_t[r.TEXTURE_2D]=ot(r.TEXTURE_2D,r.TEXTURE_2D,1),_t[r.TEXTURE_CUBE_MAP]=ot(r.TEXTURE_CUBE_MAP,r.TEXTURE_CUBE_MAP_POSITIVE_X,6),n&&(_t[r.TEXTURE_2D_ARRAY]=ot(r.TEXTURE_2D_ARRAY,r.TEXTURE_2D_ARRAY,1,1),_t[r.TEXTURE_3D]=ot(r.TEXTURE_3D,r.TEXTURE_3D,1,1)),o.setClear(0,0,0,1),c.setClear(1),l.setClear(0),ht(r.DEPTH_TEST),c.setFunc(3),bt(!1),I(1),ht(r.CULL_FACE),yt(0);function ht(U){d[U]!==!0&&(r.enable(U),d[U]=!0)}function ut(U){d[U]!==!1&&(r.disable(U),d[U]=!1)}function Ct(U,dt){return f[U]!==dt?(r.bindFramebuffer(U,dt),f[U]=dt,n&&(U===r.DRAW_FRAMEBUFFER&&(f[r.FRAMEBUFFER]=dt),U===r.FRAMEBUFFER&&(f[r.DRAW_FRAMEBUFFER]=dt)),!0):!1}function W(U,dt){let ft=x,zt=!1;if(U)if(ft=g.get(dt),ft===void 0&&(ft=[],g.set(dt,ft)),U.isWebGLMultipleRenderTargets){const Bt=U.texture;if(ft.length!==Bt.length||ft[0]!==r.COLOR_ATTACHMENT0){for(let de=0,ue=Bt.length;de<ue;de++)ft[de]=r.COLOR_ATTACHMENT0+de;ft.length=Bt.length,zt=!0}}else ft[0]!==r.COLOR_ATTACHMENT0&&(ft[0]=r.COLOR_ATTACHMENT0,zt=!0);else ft[0]!==r.BACK&&(ft[0]=r.BACK,zt=!0);zt&&(e.isWebGL2?r.drawBuffers(ft):t.get("WEBGL_draw_buffers").drawBuffersWEBGL(ft))}function Nt(U){return m!==U?(r.useProgram(U),m=U,!0):!1}const Tt={100:r.FUNC_ADD,101:r.FUNC_SUBTRACT,102:r.FUNC_REVERSE_SUBTRACT};if(n)Tt[103]=r.MIN,Tt[104]=r.MAX;else{const U=t.get("EXT_blend_minmax");U!==null&&(Tt[103]=U.MIN_EXT,Tt[104]=U.MAX_EXT)}const Pt={200:r.ZERO,201:r.ONE,202:r.SRC_COLOR,204:r.SRC_ALPHA,210:r.SRC_ALPHA_SATURATE,208:r.DST_COLOR,206:r.DST_ALPHA,203:r.ONE_MINUS_SRC_COLOR,205:r.ONE_MINUS_SRC_ALPHA,209:r.ONE_MINUS_DST_COLOR,207:r.ONE_MINUS_DST_ALPHA,211:r.CONSTANT_COLOR,212:r.ONE_MINUS_CONSTANT_COLOR,213:r.CONSTANT_ALPHA,214:r.ONE_MINUS_CONSTANT_ALPHA};function yt(U,dt,ft,zt,Bt,de,ue,ve,we,se){if(U===0){p===!0&&(ut(r.BLEND),p=!1);return}if(p===!1&&(ht(r.BLEND),p=!0),U!==5){if(U!==_||se!==L){if((v!==100||y!==100)&&(r.blendEquation(r.FUNC_ADD),v=100,y=100),se)switch(U){case 1:r.blendFuncSeparate(r.ONE,r.ONE_MINUS_SRC_ALPHA,r.ONE,r.ONE_MINUS_SRC_ALPHA);break;case 2:r.blendFunc(r.ONE,r.ONE);break;case 3:r.blendFuncSeparate(r.ZERO,r.ONE_MINUS_SRC_COLOR,r.ZERO,r.ONE);break;case 4:r.blendFuncSeparate(r.ZERO,r.SRC_COLOR,r.ZERO,r.SRC_ALPHA);break;default:console.error("THREE.WebGLState: Invalid blending: ",U);break}else switch(U){case 1:r.blendFuncSeparate(r.SRC_ALPHA,r.ONE_MINUS_SRC_ALPHA,r.ONE,r.ONE_MINUS_SRC_ALPHA);break;case 2:r.blendFunc(r.SRC_ALPHA,r.ONE);break;case 3:r.blendFuncSeparate(r.ZERO,r.ONE_MINUS_SRC_COLOR,r.ZERO,r.ONE);break;case 4:r.blendFunc(r.ZERO,r.SRC_COLOR);break;default:console.error("THREE.WebGLState: Invalid blending: ",U);break}S=null,M=null,w=null,A=null,E.set(0,0,0),T=0,_=U,L=se}return}Bt=Bt||dt,de=de||ft,ue=ue||zt,(dt!==v||Bt!==y)&&(r.blendEquationSeparate(Tt[dt],Tt[Bt]),v=dt,y=Bt),(ft!==S||zt!==M||de!==w||ue!==A)&&(r.blendFuncSeparate(Pt[ft],Pt[zt],Pt[de],Pt[ue]),S=ft,M=zt,w=de,A=ue),(ve.equals(E)===!1||we!==T)&&(r.blendColor(ve.r,ve.g,ve.b,we),E.copy(ve),T=we),_=U,L=!1}function Kt(U,dt){U.side===2?ut(r.CULL_FACE):ht(r.CULL_FACE);let ft=U.side===1;dt&&(ft=!ft),bt(ft),U.blending===1&&U.transparent===!1?yt(0):yt(U.blending,U.blendEquation,U.blendSrc,U.blendDst,U.blendEquationAlpha,U.blendSrcAlpha,U.blendDstAlpha,U.blendColor,U.blendAlpha,U.premultipliedAlpha),c.setFunc(U.depthFunc),c.setTest(U.depthTest),c.setMask(U.depthWrite),o.setMask(U.colorWrite);const zt=U.stencilWrite;l.setTest(zt),zt&&(l.setMask(U.stencilWriteMask),l.setFunc(U.stencilFunc,U.stencilRef,U.stencilFuncMask),l.setOp(U.stencilFail,U.stencilZFail,U.stencilZPass)),X(U.polygonOffset,U.polygonOffsetFactor,U.polygonOffsetUnits),U.alphaToCoverage===!0?ht(r.SAMPLE_ALPHA_TO_COVERAGE):ut(r.SAMPLE_ALPHA_TO_COVERAGE)}function bt(U){C!==U&&(U?r.frontFace(r.CW):r.frontFace(r.CCW),C=U)}function I(U){U!==0?(ht(r.CULL_FACE),U!==F&&(U===1?r.cullFace(r.BACK):U===2?r.cullFace(r.FRONT):r.cullFace(r.FRONT_AND_BACK))):ut(r.CULL_FACE),F=U}function b(U){U!==D&&(z&&r.lineWidth(U),D=U)}function X(U,dt,ft){U?(ht(r.POLYGON_OFFSET_FILL),(N!==dt||B!==ft)&&(r.polygonOffset(dt,ft),N=dt,B=ft)):ut(r.POLYGON_OFFSET_FILL)}function et(U){U?ht(r.SCISSOR_TEST):ut(r.SCISSOR_TEST)}function J(U){U===void 0&&(U=r.TEXTURE0+G-1),$!==U&&(r.activeTexture(U),$=U)}function tt(U,dt,ft){ft===void 0&&($===null?ft=r.TEXTURE0+G-1:ft=$);let zt=Q[ft];zt===void 0&&(zt={type:void 0,texture:void 0},Q[ft]=zt),(zt.type!==U||zt.texture!==dt)&&($!==ft&&(r.activeTexture(ft),$=ft),r.bindTexture(U,dt||_t[U]),zt.type=U,zt.texture=dt)}function St(){const U=Q[$];U!==void 0&&U.type!==void 0&&(r.bindTexture(U.type,null),U.type=void 0,U.texture=void 0)}function lt(){try{r.compressedTexImage2D.apply(r,arguments)}catch(U){console.error("THREE.WebGLState:",U)}}function gt(){try{r.compressedTexImage3D.apply(r,arguments)}catch(U){console.error("THREE.WebGLState:",U)}}function At(){try{r.texSubImage2D.apply(r,arguments)}catch(U){console.error("THREE.WebGLState:",U)}}function Ht(){try{r.texSubImage3D.apply(r,arguments)}catch(U){console.error("THREE.WebGLState:",U)}}function it(){try{r.compressedTexSubImage2D.apply(r,arguments)}catch(U){console.error("THREE.WebGLState:",U)}}function ce(){try{r.compressedTexSubImage3D.apply(r,arguments)}catch(U){console.error("THREE.WebGLState:",U)}}function Jt(){try{r.texStorage2D.apply(r,arguments)}catch(U){console.error("THREE.WebGLState:",U)}}function Gt(){try{r.texStorage3D.apply(r,arguments)}catch(U){console.error("THREE.WebGLState:",U)}}function Lt(){try{r.texImage2D.apply(r,arguments)}catch(U){console.error("THREE.WebGLState:",U)}}function vt(){try{r.texImage3D.apply(r,arguments)}catch(U){console.error("THREE.WebGLState:",U)}}function Ft(U){nt.equals(U)===!1&&(r.scissor(U.x,U.y,U.z,U.w),nt.copy(U))}function te(U){rt.equals(U)===!1&&(r.viewport(U.x,U.y,U.z,U.w),rt.copy(U))}function le(U,dt){let ft=u.get(dt);ft===void 0&&(ft=new WeakMap,u.set(dt,ft));let zt=ft.get(U);zt===void 0&&(zt=r.getUniformBlockIndex(dt,U.name),ft.set(U,zt))}function Wt(U,dt){const zt=u.get(dt).get(U);h.get(dt)!==zt&&(r.uniformBlockBinding(dt,zt,U.__bindingPointIndex),h.set(dt,zt))}function ct(){r.disable(r.BLEND),r.disable(r.CULL_FACE),r.disable(r.DEPTH_TEST),r.disable(r.POLYGON_OFFSET_FILL),r.disable(r.SCISSOR_TEST),r.disable(r.STENCIL_TEST),r.disable(r.SAMPLE_ALPHA_TO_COVERAGE),r.blendEquation(r.FUNC_ADD),r.blendFunc(r.ONE,r.ZERO),r.blendFuncSeparate(r.ONE,r.ZERO,r.ONE,r.ZERO),r.blendColor(0,0,0,0),r.colorMask(!0,!0,!0,!0),r.clearColor(0,0,0,0),r.depthMask(!0),r.depthFunc(r.LESS),r.clearDepth(1),r.stencilMask(4294967295),r.stencilFunc(r.ALWAYS,0,4294967295),r.stencilOp(r.KEEP,r.KEEP,r.KEEP),r.clearStencil(0),r.cullFace(r.BACK),r.frontFace(r.CCW),r.polygonOffset(0,0),r.activeTexture(r.TEXTURE0),r.bindFramebuffer(r.FRAMEBUFFER,null),n===!0&&(r.bindFramebuffer(r.DRAW_FRAMEBUFFER,null),r.bindFramebuffer(r.READ_FRAMEBUFFER,null)),r.useProgram(null),r.lineWidth(1),r.scissor(0,0,r.canvas.width,r.canvas.height),r.viewport(0,0,r.canvas.width,r.canvas.height),d={},$=null,Q={},f={},g=new WeakMap,x=[],m=null,p=!1,_=null,v=null,S=null,M=null,y=null,w=null,A=null,E=new Qt(0,0,0),T=0,L=!1,C=null,F=null,D=null,N=null,B=null,nt.set(0,0,r.canvas.width,r.canvas.height),rt.set(0,0,r.canvas.width,r.canvas.height),o.reset(),c.reset(),l.reset()}return{buffers:{color:o,depth:c,stencil:l},enable:ht,disable:ut,bindFramebuffer:Ct,drawBuffers:W,useProgram:Nt,setBlending:yt,setMaterial:Kt,setFlipSided:bt,setCullFace:I,setLineWidth:b,setPolygonOffset:X,setScissorTest:et,activeTexture:J,bindTexture:tt,unbindTexture:St,compressedTexImage2D:lt,compressedTexImage3D:gt,texImage2D:Lt,texImage3D:vt,updateUBOMapping:le,uniformBlockBinding:Wt,texStorage2D:Jt,texStorage3D:Gt,texSubImage2D:At,texSubImage3D:Ht,compressedTexSubImage2D:it,compressedTexSubImage3D:ce,scissor:Ft,viewport:te,reset:ct}}function Ev(r,t,e,n,i,s,a){const o=i.isWebGL2,c=t.has("WEBGL_multisampled_render_to_texture")?t.get("WEBGL_multisampled_render_to_texture"):null,l=typeof navigator>"u"?!1:/OculusBrowser/g.test(navigator.userAgent),h=new WeakMap;let u;const d=new WeakMap;let f=!1;try{f=typeof OffscreenCanvas<"u"&&new OffscreenCanvas(1,1).getContext("2d")!==null}catch{}function g(I,b){return f?new OffscreenCanvas(I,b):Jo("canvas")}function x(I,b,X,et){let J=1;if((I.width>et||I.height>et)&&(J=et/Math.max(I.width,I.height)),J<1||b===!0)if(typeof HTMLImageElement<"u"&&I instanceof HTMLImageElement||typeof HTMLCanvasElement<"u"&&I instanceof HTMLCanvasElement||typeof ImageBitmap<"u"&&I instanceof ImageBitmap){const tt=b?Ko:Math.floor,St=tt(J*I.width),lt=tt(J*I.height);u===void 0&&(u=g(St,lt));const gt=X?g(St,lt):u;return gt.width=St,gt.height=lt,gt.getContext("2d").drawImage(I,0,0,St,lt),console.warn("THREE.WebGLRenderer: Texture has been resized from ("+I.width+"x"+I.height+") to ("+St+"x"+lt+")."),gt}else return"data"in I&&console.warn("THREE.WebGLRenderer: Image in DataTexture is too big ("+I.width+"x"+I.height+")."),I;return I}function m(I){return th(I.width)&&th(I.height)}function p(I){return o?!1:I.wrapS!==1001||I.wrapT!==1001||I.minFilter!==1003&&I.minFilter!==1006}function _(I,b){return I.generateMipmaps&&b&&I.minFilter!==1003&&I.minFilter!==1006}function v(I){r.generateMipmap(I)}function S(I,b,X,et,J=!1){if(o===!1)return b;if(I!==null){if(r[I]!==void 0)return r[I];console.warn("THREE.WebGLRenderer: Attempt to use non-existing WebGL internal format '"+I+"'")}let tt=b;if(b===r.RED&&(X===r.FLOAT&&(tt=r.R32F),X===r.HALF_FLOAT&&(tt=r.R16F),X===r.UNSIGNED_BYTE&&(tt=r.R8)),b===r.RED_INTEGER&&(X===r.UNSIGNED_BYTE&&(tt=r.R8UI),X===r.UNSIGNED_SHORT&&(tt=r.R16UI),X===r.UNSIGNED_INT&&(tt=r.R32UI),X===r.BYTE&&(tt=r.R8I),X===r.SHORT&&(tt=r.R16I),X===r.INT&&(tt=r.R32I)),b===r.RG&&(X===r.FLOAT&&(tt=r.RG32F),X===r.HALF_FLOAT&&(tt=r.RG16F),X===r.UNSIGNED_BYTE&&(tt=r.RG8)),b===r.RGBA){const St=J?jo:pe.getTransfer(et);X===r.FLOAT&&(tt=r.RGBA32F),X===r.HALF_FLOAT&&(tt=r.RGBA16F),X===r.UNSIGNED_BYTE&&(tt=St===ye?r.SRGB8_ALPHA8:r.RGBA8),X===r.UNSIGNED_SHORT_4_4_4_4&&(tt=r.RGBA4),X===r.UNSIGNED_SHORT_5_5_5_1&&(tt=r.RGB5_A1)}return(tt===r.R16F||tt===r.R32F||tt===r.RG16F||tt===r.RG32F||tt===r.RGBA16F||tt===r.RGBA32F)&&t.get("EXT_color_buffer_float"),tt}function M(I,b,X){return _(I,X)===!0||I.isFramebufferTexture&&I.minFilter!==1003&&I.minFilter!==1006?Math.log2(Math.max(b.width,b.height))+1:I.mipmaps!==void 0&&I.mipmaps.length>0?I.mipmaps.length:I.isCompressedTexture&&Array.isArray(I.image)?b.mipmaps.length:1}function y(I){return I===1003||I===1004||I===1005?r.NEAREST:r.LINEAR}function w(I){const b=I.target;b.removeEventListener("dispose",w),E(b),b.isVideoTexture&&h.delete(b)}function A(I){const b=I.target;b.removeEventListener("dispose",A),L(b)}function E(I){const b=n.get(I);if(b.__webglInit===void 0)return;const X=I.source,et=d.get(X);if(et){const J=et[b.__cacheKey];J.usedTimes--,J.usedTimes===0&&T(I),Object.keys(et).length===0&&d.delete(X)}n.remove(I)}function T(I){const b=n.get(I);r.deleteTexture(b.__webglTexture);const X=I.source,et=d.get(X);delete et[b.__cacheKey],a.memory.textures--}function L(I){const b=I.texture,X=n.get(I),et=n.get(b);if(et.__webglTexture!==void 0&&(r.deleteTexture(et.__webglTexture),a.memory.textures--),I.depthTexture&&I.depthTexture.dispose(),I.isWebGLCubeRenderTarget)for(let J=0;J<6;J++){if(Array.isArray(X.__webglFramebuffer[J]))for(let tt=0;tt<X.__webglFramebuffer[J].length;tt++)r.deleteFramebuffer(X.__webglFramebuffer[J][tt]);else r.deleteFramebuffer(X.__webglFramebuffer[J]);X.__webglDepthbuffer&&r.deleteRenderbuffer(X.__webglDepthbuffer[J])}else{if(Array.isArray(X.__webglFramebuffer))for(let J=0;J<X.__webglFramebuffer.length;J++)r.deleteFramebuffer(X.__webglFramebuffer[J]);else r.deleteFramebuffer(X.__webglFramebuffer);if(X.__webglDepthbuffer&&r.deleteRenderbuffer(X.__webglDepthbuffer),X.__webglMultisampledFramebuffer&&r.deleteFramebuffer(X.__webglMultisampledFramebuffer),X.__webglColorRenderbuffer)for(let J=0;J<X.__webglColorRenderbuffer.length;J++)X.__webglColorRenderbuffer[J]&&r.deleteRenderbuffer(X.__webglColorRenderbuffer[J]);X.__webglDepthRenderbuffer&&r.deleteRenderbuffer(X.__webglDepthRenderbuffer)}if(I.isWebGLMultipleRenderTargets)for(let J=0,tt=b.length;J<tt;J++){const St=n.get(b[J]);St.__webglTexture&&(r.deleteTexture(St.__webglTexture),a.memory.textures--),n.remove(b[J])}n.remove(b),n.remove(I)}let C=0;function F(){C=0}function D(){const I=C;return I>=i.maxTextures&&console.warn("THREE.WebGLTextures: Trying to use "+I+" texture units while this GPU supports only "+i.maxTextures),C+=1,I}function N(I){const b=[];return b.push(I.wrapS),b.push(I.wrapT),b.push(I.wrapR||0),b.push(I.magFilter),b.push(I.minFilter),b.push(I.anisotropy),b.push(I.internalFormat),b.push(I.format),b.push(I.type),b.push(I.generateMipmaps),b.push(I.premultiplyAlpha),b.push(I.flipY),b.push(I.unpackAlignment),b.push(I.colorSpace),b.join()}function B(I,b){const X=n.get(I);if(I.isVideoTexture&&Kt(I),I.isRenderTargetTexture===!1&&I.version>0&&X.__version!==I.version){const et=I.image;if(et===null)console.warn("THREE.WebGLRenderer: Texture marked for update but no image data found.");else if(et.complete===!1)console.warn("THREE.WebGLRenderer: Texture marked for update but image is incomplete");else{nt(X,I,b);return}}e.bindTexture(r.TEXTURE_2D,X.__webglTexture,r.TEXTURE0+b)}function G(I,b){const X=n.get(I);if(I.version>0&&X.__version!==I.version){nt(X,I,b);return}e.bindTexture(r.TEXTURE_2D_ARRAY,X.__webglTexture,r.TEXTURE0+b)}function z(I,b){const X=n.get(I);if(I.version>0&&X.__version!==I.version){nt(X,I,b);return}e.bindTexture(r.TEXTURE_3D,X.__webglTexture,r.TEXTURE0+b)}function H(I,b){const X=n.get(I);if(I.version>0&&X.__version!==I.version){rt(X,I,b);return}e.bindTexture(r.TEXTURE_CUBE_MAP,X.__webglTexture,r.TEXTURE0+b)}const j={1e3:r.REPEAT,1001:r.CLAMP_TO_EDGE,1002:r.MIRRORED_REPEAT},$={1003:r.NEAREST,1004:r.NEAREST_MIPMAP_NEAREST,1005:r.NEAREST_MIPMAP_LINEAR,1006:r.LINEAR,1007:r.LINEAR_MIPMAP_NEAREST,1008:r.LINEAR_MIPMAP_LINEAR},Q={512:r.NEVER,519:r.ALWAYS,513:r.LESS,515:r.LEQUAL,514:r.EQUAL,518:r.GEQUAL,516:r.GREATER,517:r.NOTEQUAL};function V(I,b,X){if(X?(r.texParameteri(I,r.TEXTURE_WRAP_S,j[b.wrapS]),r.texParameteri(I,r.TEXTURE_WRAP_T,j[b.wrapT]),(I===r.TEXTURE_3D||I===r.TEXTURE_2D_ARRAY)&&r.texParameteri(I,r.TEXTURE_WRAP_R,j[b.wrapR]),r.texParameteri(I,r.TEXTURE_MAG_FILTER,$[b.magFilter]),r.texParameteri(I,r.TEXTURE_MIN_FILTER,$[b.minFilter])):(r.texParameteri(I,r.TEXTURE_WRAP_S,r.CLAMP_TO_EDGE),r.texParameteri(I,r.TEXTURE_WRAP_T,r.CLAMP_TO_EDGE),(I===r.TEXTURE_3D||I===r.TEXTURE_2D_ARRAY)&&r.texParameteri(I,r.TEXTURE_WRAP_R,r.CLAMP_TO_EDGE),(b.wrapS!==1001||b.wrapT!==1001)&&console.warn("THREE.WebGLRenderer: Texture is not power of two. Texture.wrapS and Texture.wrapT should be set to THREE.ClampToEdgeWrapping."),r.texParameteri(I,r.TEXTURE_MAG_FILTER,y(b.magFilter)),r.texParameteri(I,r.TEXTURE_MIN_FILTER,y(b.minFilter)),b.minFilter!==1003&&b.minFilter!==1006&&console.warn("THREE.WebGLRenderer: Texture is not power of two. Texture.minFilter should be set to THREE.NearestFilter or THREE.LinearFilter.")),b.compareFunction&&(r.texParameteri(I,r.TEXTURE_COMPARE_MODE,r.COMPARE_REF_TO_TEXTURE),r.texParameteri(I,r.TEXTURE_COMPARE_FUNC,Q[b.compareFunction])),t.has("EXT_texture_filter_anisotropic")===!0){const et=t.get("EXT_texture_filter_anisotropic");if(b.magFilter===1003||b.minFilter!==1005&&b.minFilter!==1008||b.type===1015&&t.has("OES_texture_float_linear")===!1||o===!1&&b.type===1016&&t.has("OES_texture_half_float_linear")===!1)return;(b.anisotropy>1||n.get(b).__currentAnisotropy)&&(r.texParameterf(I,et.TEXTURE_MAX_ANISOTROPY_EXT,Math.min(b.anisotropy,i.getMaxAnisotropy())),n.get(b).__currentAnisotropy=b.anisotropy)}}function K(I,b){let X=!1;I.__webglInit===void 0&&(I.__webglInit=!0,b.addEventListener("dispose",w));const et=b.source;let J=d.get(et);J===void 0&&(J={},d.set(et,J));const tt=N(b);if(tt!==I.__cacheKey){J[tt]===void 0&&(J[tt]={texture:r.createTexture(),usedTimes:0},a.memory.textures++,X=!0),J[tt].usedTimes++;const St=J[I.__cacheKey];St!==void 0&&(J[I.__cacheKey].usedTimes--,St.usedTimes===0&&T(b)),I.__cacheKey=tt,I.__webglTexture=J[tt].texture}return X}function nt(I,b,X){let et=r.TEXTURE_2D;(b.isDataArrayTexture||b.isCompressedArrayTexture)&&(et=r.TEXTURE_2D_ARRAY),b.isData3DTexture&&(et=r.TEXTURE_3D);const J=K(I,b),tt=b.source;e.bindTexture(et,I.__webglTexture,r.TEXTURE0+X);const St=n.get(tt);if(tt.version!==St.__version||J===!0){e.activeTexture(r.TEXTURE0+X);const lt=pe.getPrimaries(pe.workingColorSpace),gt=b.colorSpace===""?null:pe.getPrimaries(b.colorSpace),At=b.colorSpace===""||lt===gt?r.NONE:r.BROWSER_DEFAULT_WEBGL;r.pixelStorei(r.UNPACK_FLIP_Y_WEBGL,b.flipY),r.pixelStorei(r.UNPACK_PREMULTIPLY_ALPHA_WEBGL,b.premultiplyAlpha),r.pixelStorei(r.UNPACK_ALIGNMENT,b.unpackAlignment),r.pixelStorei(r.UNPACK_COLORSPACE_CONVERSION_WEBGL,At);const Ht=p(b)&&m(b.image)===!1;let it=x(b.image,Ht,!1,i.maxTextureSize);it=bt(b,it);const ce=m(it)||o,Jt=s.convert(b.format,b.colorSpace);let Gt=s.convert(b.type),Lt=S(b.internalFormat,Jt,Gt,b.colorSpace,b.isVideoTexture);V(et,b,ce);let vt;const Ft=b.mipmaps,te=o&&b.isVideoTexture!==!0&&Lt!==36196,le=St.__version===void 0||J===!0,Wt=M(b,it,ce);if(b.isDepthTexture)Lt=r.DEPTH_COMPONENT,o?b.type===1015?Lt=r.DEPTH_COMPONENT32F:b.type===1014?Lt=r.DEPTH_COMPONENT24:b.type===1020?Lt=r.DEPTH24_STENCIL8:Lt=r.DEPTH_COMPONENT16:b.type===1015&&console.error("WebGLRenderer: Floating point depth texture requires WebGL2."),b.format===1026&&Lt===r.DEPTH_COMPONENT&&b.type!==1012&&b.type!==1014&&(console.warn("THREE.WebGLRenderer: Use UnsignedShortType or UnsignedIntType for DepthFormat DepthTexture."),b.type=1014,Gt=s.convert(b.type)),b.format===1027&&Lt===r.DEPTH_COMPONENT&&(Lt=r.DEPTH_STENCIL,b.type!==1020&&(console.warn("THREE.WebGLRenderer: Use UnsignedInt248Type for DepthStencilFormat DepthTexture."),b.type=1020,Gt=s.convert(b.type))),le&&(te?e.texStorage2D(r.TEXTURE_2D,1,Lt,it.width,it.height):e.texImage2D(r.TEXTURE_2D,0,Lt,it.width,it.height,0,Jt,Gt,null));else if(b.isDataTexture)if(Ft.length>0&&ce){te&&le&&e.texStorage2D(r.TEXTURE_2D,Wt,Lt,Ft[0].width,Ft[0].height);for(let ct=0,U=Ft.length;ct<U;ct++)vt=Ft[ct],te?e.texSubImage2D(r.TEXTURE_2D,ct,0,0,vt.width,vt.height,Jt,Gt,vt.data):e.texImage2D(r.TEXTURE_2D,ct,Lt,vt.width,vt.height,0,Jt,Gt,vt.data);b.generateMipmaps=!1}else te?(le&&e.texStorage2D(r.TEXTURE_2D,Wt,Lt,it.width,it.height),e.texSubImage2D(r.TEXTURE_2D,0,0,0,it.width,it.height,Jt,Gt,it.data)):e.texImage2D(r.TEXTURE_2D,0,Lt,it.width,it.height,0,Jt,Gt,it.data);else if(b.isCompressedTexture)if(b.isCompressedArrayTexture){te&&le&&e.texStorage3D(r.TEXTURE_2D_ARRAY,Wt,Lt,Ft[0].width,Ft[0].height,it.depth);for(let ct=0,U=Ft.length;ct<U;ct++)vt=Ft[ct],b.format!==1023?Jt!==null?te?e.compressedTexSubImage3D(r.TEXTURE_2D_ARRAY,ct,0,0,0,vt.width,vt.height,it.depth,Jt,vt.data,0,0):e.compressedTexImage3D(r.TEXTURE_2D_ARRAY,ct,Lt,vt.width,vt.height,it.depth,0,vt.data,0,0):console.warn("THREE.WebGLRenderer: Attempt to load unsupported compressed texture format in .uploadTexture()"):te?e.texSubImage3D(r.TEXTURE_2D_ARRAY,ct,0,0,0,vt.width,vt.height,it.depth,Jt,Gt,vt.data):e.texImage3D(r.TEXTURE_2D_ARRAY,ct,Lt,vt.width,vt.height,it.depth,0,Jt,Gt,vt.data)}else{te&&le&&e.texStorage2D(r.TEXTURE_2D,Wt,Lt,Ft[0].width,Ft[0].height);for(let ct=0,U=Ft.length;ct<U;ct++)vt=Ft[ct],b.format!==1023?Jt!==null?te?e.compressedTexSubImage2D(r.TEXTURE_2D,ct,0,0,vt.width,vt.height,Jt,vt.data):e.compressedTexImage2D(r.TEXTURE_2D,ct,Lt,vt.width,vt.height,0,vt.data):console.warn("THREE.WebGLRenderer: Attempt to load unsupported compressed texture format in .uploadTexture()"):te?e.texSubImage2D(r.TEXTURE_2D,ct,0,0,vt.width,vt.height,Jt,Gt,vt.data):e.texImage2D(r.TEXTURE_2D,ct,Lt,vt.width,vt.height,0,Jt,Gt,vt.data)}else if(b.isDataArrayTexture)te?(le&&e.texStorage3D(r.TEXTURE_2D_ARRAY,Wt,Lt,it.width,it.height,it.depth),e.texSubImage3D(r.TEXTURE_2D_ARRAY,0,0,0,0,it.width,it.height,it.depth,Jt,Gt,it.data)):e.texImage3D(r.TEXTURE_2D_ARRAY,0,Lt,it.width,it.height,it.depth,0,Jt,Gt,it.data);else if(b.isData3DTexture)te?(le&&e.texStorage3D(r.TEXTURE_3D,Wt,Lt,it.width,it.height,it.depth),e.texSubImage3D(r.TEXTURE_3D,0,0,0,0,it.width,it.height,it.depth,Jt,Gt,it.data)):e.texImage3D(r.TEXTURE_3D,0,Lt,it.width,it.height,it.depth,0,Jt,Gt,it.data);else if(b.isFramebufferTexture){if(le)if(te)e.texStorage2D(r.TEXTURE_2D,Wt,Lt,it.width,it.height);else{let ct=it.width,U=it.height;for(let dt=0;dt<Wt;dt++)e.texImage2D(r.TEXTURE_2D,dt,Lt,ct,U,0,Jt,Gt,null),ct>>=1,U>>=1}}else if(Ft.length>0&&ce){te&&le&&e.texStorage2D(r.TEXTURE_2D,Wt,Lt,Ft[0].width,Ft[0].height);for(let ct=0,U=Ft.length;ct<U;ct++)vt=Ft[ct],te?e.texSubImage2D(r.TEXTURE_2D,ct,0,0,Jt,Gt,vt):e.texImage2D(r.TEXTURE_2D,ct,Lt,Jt,Gt,vt);b.generateMipmaps=!1}else te?(le&&e.texStorage2D(r.TEXTURE_2D,Wt,Lt,it.width,it.height),e.texSubImage2D(r.TEXTURE_2D,0,0,0,Jt,Gt,it)):e.texImage2D(r.TEXTURE_2D,0,Lt,Jt,Gt,it);_(b,ce)&&v(et),St.__version=tt.version,b.onUpdate&&b.onUpdate(b)}I.__version=b.version}function rt(I,b,X){if(b.image.length!==6)return;const et=K(I,b),J=b.source;e.bindTexture(r.TEXTURE_CUBE_MAP,I.__webglTexture,r.TEXTURE0+X);const tt=n.get(J);if(J.version!==tt.__version||et===!0){e.activeTexture(r.TEXTURE0+X);const St=pe.getPrimaries(pe.workingColorSpace),lt=b.colorSpace===""?null:pe.getPrimaries(b.colorSpace),gt=b.colorSpace===""||St===lt?r.NONE:r.BROWSER_DEFAULT_WEBGL;r.pixelStorei(r.UNPACK_FLIP_Y_WEBGL,b.flipY),r.pixelStorei(r.UNPACK_PREMULTIPLY_ALPHA_WEBGL,b.premultiplyAlpha),r.pixelStorei(r.UNPACK_ALIGNMENT,b.unpackAlignment),r.pixelStorei(r.UNPACK_COLORSPACE_CONVERSION_WEBGL,gt);const At=b.isCompressedTexture||b.image[0].isCompressedTexture,Ht=b.image[0]&&b.image[0].isDataTexture,it=[];for(let ct=0;ct<6;ct++)!At&&!Ht?it[ct]=x(b.image[ct],!1,!0,i.maxCubemapSize):it[ct]=Ht?b.image[ct].image:b.image[ct],it[ct]=bt(b,it[ct]);const ce=it[0],Jt=m(ce)||o,Gt=s.convert(b.format,b.colorSpace),Lt=s.convert(b.type),vt=S(b.internalFormat,Gt,Lt,b.colorSpace),Ft=o&&b.isVideoTexture!==!0,te=tt.__version===void 0||et===!0;let le=M(b,ce,Jt);V(r.TEXTURE_CUBE_MAP,b,Jt);let Wt;if(At){Ft&&te&&e.texStorage2D(r.TEXTURE_CUBE_MAP,le,vt,ce.width,ce.height);for(let ct=0;ct<6;ct++){Wt=it[ct].mipmaps;for(let U=0;U<Wt.length;U++){const dt=Wt[U];b.format!==1023?Gt!==null?Ft?e.compressedTexSubImage2D(r.TEXTURE_CUBE_MAP_POSITIVE_X+ct,U,0,0,dt.width,dt.height,Gt,dt.data):e.compressedTexImage2D(r.TEXTURE_CUBE_MAP_POSITIVE_X+ct,U,vt,dt.width,dt.height,0,dt.data):console.warn("THREE.WebGLRenderer: Attempt to load unsupported compressed texture format in .setTextureCube()"):Ft?e.texSubImage2D(r.TEXTURE_CUBE_MAP_POSITIVE_X+ct,U,0,0,dt.width,dt.height,Gt,Lt,dt.data):e.texImage2D(r.TEXTURE_CUBE_MAP_POSITIVE_X+ct,U,vt,dt.width,dt.height,0,Gt,Lt,dt.data)}}}else{Wt=b.mipmaps,Ft&&te&&(Wt.length>0&&le++,e.texStorage2D(r.TEXTURE_CUBE_MAP,le,vt,it[0].width,it[0].height));for(let ct=0;ct<6;ct++)if(Ht){Ft?e.texSubImage2D(r.TEXTURE_CUBE_MAP_POSITIVE_X+ct,0,0,0,it[ct].width,it[ct].height,Gt,Lt,it[ct].data):e.texImage2D(r.TEXTURE_CUBE_MAP_POSITIVE_X+ct,0,vt,it[ct].width,it[ct].height,0,Gt,Lt,it[ct].data);for(let U=0;U<Wt.length;U++){const ft=Wt[U].image[ct].image;Ft?e.texSubImage2D(r.TEXTURE_CUBE_MAP_POSITIVE_X+ct,U+1,0,0,ft.width,ft.height,Gt,Lt,ft.data):e.texImage2D(r.TEXTURE_CUBE_MAP_POSITIVE_X+ct,U+1,vt,ft.width,ft.height,0,Gt,Lt,ft.data)}}else{Ft?e.texSubImage2D(r.TEXTURE_CUBE_MAP_POSITIVE_X+ct,0,0,0,Gt,Lt,it[ct]):e.texImage2D(r.TEXTURE_CUBE_MAP_POSITIVE_X+ct,0,vt,Gt,Lt,it[ct]);for(let U=0;U<Wt.length;U++){const dt=Wt[U];Ft?e.texSubImage2D(r.TEXTURE_CUBE_MAP_POSITIVE_X+ct,U+1,0,0,Gt,Lt,dt.image[ct]):e.texImage2D(r.TEXTURE_CUBE_MAP_POSITIVE_X+ct,U+1,vt,Gt,Lt,dt.image[ct])}}}_(b,Jt)&&v(r.TEXTURE_CUBE_MAP),tt.__version=J.version,b.onUpdate&&b.onUpdate(b)}I.__version=b.version}function ot(I,b,X,et,J,tt){const St=s.convert(X.format,X.colorSpace),lt=s.convert(X.type),gt=S(X.internalFormat,St,lt,X.colorSpace);if(!n.get(b).__hasExternalTextures){const Ht=Math.max(1,b.width>>tt),it=Math.max(1,b.height>>tt);J===r.TEXTURE_3D||J===r.TEXTURE_2D_ARRAY?e.texImage3D(J,tt,gt,Ht,it,b.depth,0,St,lt,null):e.texImage2D(J,tt,gt,Ht,it,0,St,lt,null)}e.bindFramebuffer(r.FRAMEBUFFER,I),yt(b)?c.framebufferTexture2DMultisampleEXT(r.FRAMEBUFFER,et,J,n.get(X).__webglTexture,0,Pt(b)):(J===r.TEXTURE_2D||J>=r.TEXTURE_CUBE_MAP_POSITIVE_X&&J<=r.TEXTURE_CUBE_MAP_NEGATIVE_Z)&&r.framebufferTexture2D(r.FRAMEBUFFER,et,J,n.get(X).__webglTexture,tt),e.bindFramebuffer(r.FRAMEBUFFER,null)}function _t(I,b,X){if(r.bindRenderbuffer(r.RENDERBUFFER,I),b.depthBuffer&&!b.stencilBuffer){let et=o===!0?r.DEPTH_COMPONENT24:r.DEPTH_COMPONENT16;if(X||yt(b)){const J=b.depthTexture;J&&J.isDepthTexture&&(J.type===1015?et=r.DEPTH_COMPONENT32F:J.type===1014&&(et=r.DEPTH_COMPONENT24));const tt=Pt(b);yt(b)?c.renderbufferStorageMultisampleEXT(r.RENDERBUFFER,tt,et,b.width,b.height):r.renderbufferStorageMultisample(r.RENDERBUFFER,tt,et,b.width,b.height)}else r.renderbufferStorage(r.RENDERBUFFER,et,b.width,b.height);r.framebufferRenderbuffer(r.FRAMEBUFFER,r.DEPTH_ATTACHMENT,r.RENDERBUFFER,I)}else if(b.depthBuffer&&b.stencilBuffer){const et=Pt(b);X&&yt(b)===!1?r.renderbufferStorageMultisample(r.RENDERBUFFER,et,r.DEPTH24_STENCIL8,b.width,b.height):yt(b)?c.renderbufferStorageMultisampleEXT(r.RENDERBUFFER,et,r.DEPTH24_STENCIL8,b.width,b.height):r.renderbufferStorage(r.RENDERBUFFER,r.DEPTH_STENCIL,b.width,b.height),r.framebufferRenderbuffer(r.FRAMEBUFFER,r.DEPTH_STENCIL_ATTACHMENT,r.RENDERBUFFER,I)}else{const et=b.isWebGLMultipleRenderTargets===!0?b.texture:[b.texture];for(let J=0;J<et.length;J++){const tt=et[J],St=s.convert(tt.format,tt.colorSpace),lt=s.convert(tt.type),gt=S(tt.internalFormat,St,lt,tt.colorSpace),At=Pt(b);X&&yt(b)===!1?r.renderbufferStorageMultisample(r.RENDERBUFFER,At,gt,b.width,b.height):yt(b)?c.renderbufferStorageMultisampleEXT(r.RENDERBUFFER,At,gt,b.width,b.height):r.renderbufferStorage(r.RENDERBUFFER,gt,b.width,b.height)}}r.bindRenderbuffer(r.RENDERBUFFER,null)}function ht(I,b){if(b&&b.isWebGLCubeRenderTarget)throw new Error("Depth Texture with cube render targets is not supported");if(e.bindFramebuffer(r.FRAMEBUFFER,I),!(b.depthTexture&&b.depthTexture.isDepthTexture))throw new Error("renderTarget.depthTexture must be an instance of THREE.DepthTexture");(!n.get(b.depthTexture).__webglTexture||b.depthTexture.image.width!==b.width||b.depthTexture.image.height!==b.height)&&(b.depthTexture.image.width=b.width,b.depthTexture.image.height=b.height,b.depthTexture.needsUpdate=!0),B(b.depthTexture,0);const et=n.get(b.depthTexture).__webglTexture,J=Pt(b);if(b.depthTexture.format===1026)yt(b)?c.framebufferTexture2DMultisampleEXT(r.FRAMEBUFFER,r.DEPTH_ATTACHMENT,r.TEXTURE_2D,et,0,J):r.framebufferTexture2D(r.FRAMEBUFFER,r.DEPTH_ATTACHMENT,r.TEXTURE_2D,et,0);else if(b.depthTexture.format===1027)yt(b)?c.framebufferTexture2DMultisampleEXT(r.FRAMEBUFFER,r.DEPTH_STENCIL_ATTACHMENT,r.TEXTURE_2D,et,0,J):r.framebufferTexture2D(r.FRAMEBUFFER,r.DEPTH_STENCIL_ATTACHMENT,r.TEXTURE_2D,et,0);else throw new Error("Unknown depthTexture format")}function ut(I){const b=n.get(I),X=I.isWebGLCubeRenderTarget===!0;if(I.depthTexture&&!b.__autoAllocateDepthBuffer){if(X)throw new Error("target.depthTexture not supported in Cube render targets");ht(b.__webglFramebuffer,I)}else if(X){b.__webglDepthbuffer=[];for(let et=0;et<6;et++)e.bindFramebuffer(r.FRAMEBUFFER,b.__webglFramebuffer[et]),b.__webglDepthbuffer[et]=r.createRenderbuffer(),_t(b.__webglDepthbuffer[et],I,!1)}else e.bindFramebuffer(r.FRAMEBUFFER,b.__webglFramebuffer),b.__webglDepthbuffer=r.createRenderbuffer(),_t(b.__webglDepthbuffer,I,!1);e.bindFramebuffer(r.FRAMEBUFFER,null)}function Ct(I,b,X){const et=n.get(I);b!==void 0&&ot(et.__webglFramebuffer,I,I.texture,r.COLOR_ATTACHMENT0,r.TEXTURE_2D,0),X!==void 0&&ut(I)}function W(I){const b=I.texture,X=n.get(I),et=n.get(b);I.addEventListener("dispose",A),I.isWebGLMultipleRenderTargets!==!0&&(et.__webglTexture===void 0&&(et.__webglTexture=r.createTexture()),et.__version=b.version,a.memory.textures++);const J=I.isWebGLCubeRenderTarget===!0,tt=I.isWebGLMultipleRenderTargets===!0,St=m(I)||o;if(J){X.__webglFramebuffer=[];for(let lt=0;lt<6;lt++)if(o&&b.mipmaps&&b.mipmaps.length>0){X.__webglFramebuffer[lt]=[];for(let gt=0;gt<b.mipmaps.length;gt++)X.__webglFramebuffer[lt][gt]=r.createFramebuffer()}else X.__webglFramebuffer[lt]=r.createFramebuffer()}else{if(o&&b.mipmaps&&b.mipmaps.length>0){X.__webglFramebuffer=[];for(let lt=0;lt<b.mipmaps.length;lt++)X.__webglFramebuffer[lt]=r.createFramebuffer()}else X.__webglFramebuffer=r.createFramebuffer();if(tt)if(i.drawBuffers){const lt=I.texture;for(let gt=0,At=lt.length;gt<At;gt++){const Ht=n.get(lt[gt]);Ht.__webglTexture===void 0&&(Ht.__webglTexture=r.createTexture(),a.memory.textures++)}}else console.warn("THREE.WebGLRenderer: WebGLMultipleRenderTargets can only be used with WebGL2 or WEBGL_draw_buffers extension.");if(o&&I.samples>0&&yt(I)===!1){const lt=tt?b:[b];X.__webglMultisampledFramebuffer=r.createFramebuffer(),X.__webglColorRenderbuffer=[],e.bindFramebuffer(r.FRAMEBUFFER,X.__webglMultisampledFramebuffer);for(let gt=0;gt<lt.length;gt++){const At=lt[gt];X.__webglColorRenderbuffer[gt]=r.createRenderbuffer(),r.bindRenderbuffer(r.RENDERBUFFER,X.__webglColorRenderbuffer[gt]);const Ht=s.convert(At.format,At.colorSpace),it=s.convert(At.type),ce=S(At.internalFormat,Ht,it,At.colorSpace,I.isXRRenderTarget===!0),Jt=Pt(I);r.renderbufferStorageMultisample(r.RENDERBUFFER,Jt,ce,I.width,I.height),r.framebufferRenderbuffer(r.FRAMEBUFFER,r.COLOR_ATTACHMENT0+gt,r.RENDERBUFFER,X.__webglColorRenderbuffer[gt])}r.bindRenderbuffer(r.RENDERBUFFER,null),I.depthBuffer&&(X.__webglDepthRenderbuffer=r.createRenderbuffer(),_t(X.__webglDepthRenderbuffer,I,!0)),e.bindFramebuffer(r.FRAMEBUFFER,null)}}if(J){e.bindTexture(r.TEXTURE_CUBE_MAP,et.__webglTexture),V(r.TEXTURE_CUBE_MAP,b,St);for(let lt=0;lt<6;lt++)if(o&&b.mipmaps&&b.mipmaps.length>0)for(let gt=0;gt<b.mipmaps.length;gt++)ot(X.__webglFramebuffer[lt][gt],I,b,r.COLOR_ATTACHMENT0,r.TEXTURE_CUBE_MAP_POSITIVE_X+lt,gt);else ot(X.__webglFramebuffer[lt],I,b,r.COLOR_ATTACHMENT0,r.TEXTURE_CUBE_MAP_POSITIVE_X+lt,0);_(b,St)&&v(r.TEXTURE_CUBE_MAP),e.unbindTexture()}else if(tt){const lt=I.texture;for(let gt=0,At=lt.length;gt<At;gt++){const Ht=lt[gt],it=n.get(Ht);e.bindTexture(r.TEXTURE_2D,it.__webglTexture),V(r.TEXTURE_2D,Ht,St),ot(X.__webglFramebuffer,I,Ht,r.COLOR_ATTACHMENT0+gt,r.TEXTURE_2D,0),_(Ht,St)&&v(r.TEXTURE_2D)}e.unbindTexture()}else{let lt=r.TEXTURE_2D;if((I.isWebGL3DRenderTarget||I.isWebGLArrayRenderTarget)&&(o?lt=I.isWebGL3DRenderTarget?r.TEXTURE_3D:r.TEXTURE_2D_ARRAY:console.error("THREE.WebGLTextures: THREE.Data3DTexture and THREE.DataArrayTexture only supported with WebGL2.")),e.bindTexture(lt,et.__webglTexture),V(lt,b,St),o&&b.mipmaps&&b.mipmaps.length>0)for(let gt=0;gt<b.mipmaps.length;gt++)ot(X.__webglFramebuffer[gt],I,b,r.COLOR_ATTACHMENT0,lt,gt);else ot(X.__webglFramebuffer,I,b,r.COLOR_ATTACHMENT0,lt,0);_(b,St)&&v(lt),e.unbindTexture()}I.depthBuffer&&ut(I)}function Nt(I){const b=m(I)||o,X=I.isWebGLMultipleRenderTargets===!0?I.texture:[I.texture];for(let et=0,J=X.length;et<J;et++){const tt=X[et];if(_(tt,b)){const St=I.isWebGLCubeRenderTarget?r.TEXTURE_CUBE_MAP:r.TEXTURE_2D,lt=n.get(tt).__webglTexture;e.bindTexture(St,lt),v(St),e.unbindTexture()}}}function Tt(I){if(o&&I.samples>0&&yt(I)===!1){const b=I.isWebGLMultipleRenderTargets?I.texture:[I.texture],X=I.width,et=I.height;let J=r.COLOR_BUFFER_BIT;const tt=[],St=I.stencilBuffer?r.DEPTH_STENCIL_ATTACHMENT:r.DEPTH_ATTACHMENT,lt=n.get(I),gt=I.isWebGLMultipleRenderTargets===!0;if(gt)for(let At=0;At<b.length;At++)e.bindFramebuffer(r.FRAMEBUFFER,lt.__webglMultisampledFramebuffer),r.framebufferRenderbuffer(r.FRAMEBUFFER,r.COLOR_ATTACHMENT0+At,r.RENDERBUFFER,null),e.bindFramebuffer(r.FRAMEBUFFER,lt.__webglFramebuffer),r.framebufferTexture2D(r.DRAW_FRAMEBUFFER,r.COLOR_ATTACHMENT0+At,r.TEXTURE_2D,null,0);e.bindFramebuffer(r.READ_FRAMEBUFFER,lt.__webglMultisampledFramebuffer),e.bindFramebuffer(r.DRAW_FRAMEBUFFER,lt.__webglFramebuffer);for(let At=0;At<b.length;At++){tt.push(r.COLOR_ATTACHMENT0+At),I.depthBuffer&&tt.push(St);const Ht=lt.__ignoreDepthValues!==void 0?lt.__ignoreDepthValues:!1;if(Ht===!1&&(I.depthBuffer&&(J|=r.DEPTH_BUFFER_BIT),I.stencilBuffer&&(J|=r.STENCIL_BUFFER_BIT)),gt&&r.framebufferRenderbuffer(r.READ_FRAMEBUFFER,r.COLOR_ATTACHMENT0,r.RENDERBUFFER,lt.__webglColorRenderbuffer[At]),Ht===!0&&(r.invalidateFramebuffer(r.READ_FRAMEBUFFER,[St]),r.invalidateFramebuffer(r.DRAW_FRAMEBUFFER,[St])),gt){const it=n.get(b[At]).__webglTexture;r.framebufferTexture2D(r.DRAW_FRAMEBUFFER,r.COLOR_ATTACHMENT0,r.TEXTURE_2D,it,0)}r.blitFramebuffer(0,0,X,et,0,0,X,et,J,r.NEAREST),l&&r.invalidateFramebuffer(r.READ_FRAMEBUFFER,tt)}if(e.bindFramebuffer(r.READ_FRAMEBUFFER,null),e.bindFramebuffer(r.DRAW_FRAMEBUFFER,null),gt)for(let At=0;At<b.length;At++){e.bindFramebuffer(r.FRAMEBUFFER,lt.__webglMultisampledFramebuffer),r.framebufferRenderbuffer(r.FRAMEBUFFER,r.COLOR_ATTACHMENT0+At,r.RENDERBUFFER,lt.__webglColorRenderbuffer[At]);const Ht=n.get(b[At]).__webglTexture;e.bindFramebuffer(r.FRAMEBUFFER,lt.__webglFramebuffer),r.framebufferTexture2D(r.DRAW_FRAMEBUFFER,r.COLOR_ATTACHMENT0+At,r.TEXTURE_2D,Ht,0)}e.bindFramebuffer(r.DRAW_FRAMEBUFFER,lt.__webglMultisampledFramebuffer)}}function Pt(I){return Math.min(i.maxSamples,I.samples)}function yt(I){const b=n.get(I);return o&&I.samples>0&&t.has("WEBGL_multisampled_render_to_texture")===!0&&b.__useRenderToTexture!==!1}function Kt(I){const b=a.render.frame;h.get(I)!==b&&(h.set(I,b),I.update())}function bt(I,b){const X=I.colorSpace,et=I.format,J=I.type;return I.isCompressedTexture===!0||I.isVideoTexture===!0||I.format===1035||X!==Ui&&X!==""&&(pe.getTransfer(X)===ye?o===!1?t.has("EXT_sRGB")===!0&&et===1023?(I.format=1035,I.minFilter=1006,I.generateMipmaps=!1):b=Uf.sRGBToLinear(b):(et!==1023||J!==1009)&&console.warn("THREE.WebGLTextures: sRGB encoded textures have to use RGBAFormat and UnsignedByteType."):console.error("THREE.WebGLTextures: Unsupported texture color space:",X)),b}this.allocateTextureUnit=D,this.resetTextureUnits=F,this.setTexture2D=B,this.setTexture2DArray=G,this.setTexture3D=z,this.setTextureCube=H,this.rebindTextures=Ct,this.setupRenderTarget=W,this.updateRenderTargetMipmap=Nt,this.updateMultisampleRenderTarget=Tt,this.setupDepthRenderbuffer=ut,this.setupFrameBufferTexture=ot,this.useMultisampledRTT=yt}function wv(r,t,e){const n=e.isWebGL2;function i(s,a=""){let o;const c=pe.getTransfer(a);if(s===1009)return r.UNSIGNED_BYTE;if(s===1017)return r.UNSIGNED_SHORT_4_4_4_4;if(s===1018)return r.UNSIGNED_SHORT_5_5_5_1;if(s===1010)return r.BYTE;if(s===1011)return r.SHORT;if(s===1012)return r.UNSIGNED_SHORT;if(s===1013)return r.INT;if(s===1014)return r.UNSIGNED_INT;if(s===1015)return r.FLOAT;if(s===1016)return n?r.HALF_FLOAT:(o=t.get("OES_texture_half_float"),o!==null?o.HALF_FLOAT_OES:null);if(s===1021)return r.ALPHA;if(s===1023)return r.RGBA;if(s===1024)return r.LUMINANCE;if(s===1025)return r.LUMINANCE_ALPHA;if(s===1026)return r.DEPTH_COMPONENT;if(s===1027)return r.DEPTH_STENCIL;if(s===1035)return o=t.get("EXT_sRGB"),o!==null?o.SRGB_ALPHA_EXT:null;if(s===1028)return r.RED;if(s===1029)return r.RED_INTEGER;if(s===1030)return r.RG;if(s===1031)return r.RG_INTEGER;if(s===1033)return r.RGBA_INTEGER;if(s===33776||s===33777||s===33778||s===33779)if(c===ye)if(o=t.get("WEBGL_compressed_texture_s3tc_srgb"),o!==null){if(s===33776)return o.COMPRESSED_SRGB_S3TC_DXT1_EXT;if(s===33777)return o.COMPRESSED_SRGB_ALPHA_S3TC_DXT1_EXT;if(s===33778)return o.COMPRESSED_SRGB_ALPHA_S3TC_DXT3_EXT;if(s===33779)return o.COMPRESSED_SRGB_ALPHA_S3TC_DXT5_EXT}else return null;else if(o=t.get("WEBGL_compressed_texture_s3tc"),o!==null){if(s===33776)return o.COMPRESSED_RGB_S3TC_DXT1_EXT;if(s===33777)return o.COMPRESSED_RGBA_S3TC_DXT1_EXT;if(s===33778)return o.COMPRESSED_RGBA_S3TC_DXT3_EXT;if(s===33779)return o.COMPRESSED_RGBA_S3TC_DXT5_EXT}else return null;if(s===35840||s===35841||s===35842||s===35843)if(o=t.get("WEBGL_compressed_texture_pvrtc"),o!==null){if(s===35840)return o.COMPRESSED_RGB_PVRTC_4BPPV1_IMG;if(s===35841)return o.COMPRESSED_RGB_PVRTC_2BPPV1_IMG;if(s===35842)return o.COMPRESSED_RGBA_PVRTC_4BPPV1_IMG;if(s===35843)return o.COMPRESSED_RGBA_PVRTC_2BPPV1_IMG}else return null;if(s===36196)return o=t.get("WEBGL_compressed_texture_etc1"),o!==null?o.COMPRESSED_RGB_ETC1_WEBGL:null;if(s===37492||s===37496)if(o=t.get("WEBGL_compressed_texture_etc"),o!==null){if(s===37492)return c===ye?o.COMPRESSED_SRGB8_ETC2:o.COMPRESSED_RGB8_ETC2;if(s===37496)return c===ye?o.COMPRESSED_SRGB8_ALPHA8_ETC2_EAC:o.COMPRESSED_RGBA8_ETC2_EAC}else return null;if(s===37808||s===37809||s===37810||s===37811||s===37812||s===37813||s===37814||s===37815||s===37816||s===37817||s===37818||s===37819||s===37820||s===37821)if(o=t.get("WEBGL_compressed_texture_astc"),o!==null){if(s===37808)return c===ye?o.COMPRESSED_SRGB8_ALPHA8_ASTC_4x4_KHR:o.COMPRESSED_RGBA_ASTC_4x4_KHR;if(s===37809)return c===ye?o.COMPRESSED_SRGB8_ALPHA8_ASTC_5x4_KHR:o.COMPRESSED_RGBA_ASTC_5x4_KHR;if(s===37810)return c===ye?o.COMPRESSED_SRGB8_ALPHA8_ASTC_5x5_KHR:o.COMPRESSED_RGBA_ASTC_5x5_KHR;if(s===37811)return c===ye?o.COMPRESSED_SRGB8_ALPHA8_ASTC_6x5_KHR:o.COMPRESSED_RGBA_ASTC_6x5_KHR;if(s===37812)return c===ye?o.COMPRESSED_SRGB8_ALPHA8_ASTC_6x6_KHR:o.COMPRESSED_RGBA_ASTC_6x6_KHR;if(s===37813)return c===ye?o.COMPRESSED_SRGB8_ALPHA8_ASTC_8x5_KHR:o.COMPRESSED_RGBA_ASTC_8x5_KHR;if(s===37814)return c===ye?o.COMPRESSED_SRGB8_ALPHA8_ASTC_8x6_KHR:o.COMPRESSED_RGBA_ASTC_8x6_KHR;if(s===37815)return c===ye?o.COMPRESSED_SRGB8_ALPHA8_ASTC_8x8_KHR:o.COMPRESSED_RGBA_ASTC_8x8_KHR;if(s===37816)return c===ye?o.COMPRESSED_SRGB8_ALPHA8_ASTC_10x5_KHR:o.COMPRESSED_RGBA_ASTC_10x5_KHR;if(s===37817)return c===ye?o.COMPRESSED_SRGB8_ALPHA8_ASTC_10x6_KHR:o.COMPRESSED_RGBA_ASTC_10x6_KHR;if(s===37818)return c===ye?o.COMPRESSED_SRGB8_ALPHA8_ASTC_10x8_KHR:o.COMPRESSED_RGBA_ASTC_10x8_KHR;if(s===37819)return c===ye?o.COMPRESSED_SRGB8_ALPHA8_ASTC_10x10_KHR:o.COMPRESSED_RGBA_ASTC_10x10_KHR;if(s===37820)return c===ye?o.COMPRESSED_SRGB8_ALPHA8_ASTC_12x10_KHR:o.COMPRESSED_RGBA_ASTC_12x10_KHR;if(s===37821)return c===ye?o.COMPRESSED_SRGB8_ALPHA8_ASTC_12x12_KHR:o.COMPRESSED_RGBA_ASTC_12x12_KHR}else return null;if(s===36492||s===36494||s===36495)if(o=t.get("EXT_texture_compression_bptc"),o!==null){if(s===36492)return c===ye?o.COMPRESSED_SRGB_ALPHA_BPTC_UNORM_EXT:o.COMPRESSED_RGBA_BPTC_UNORM_EXT;if(s===36494)return o.COMPRESSED_RGB_BPTC_SIGNED_FLOAT_EXT;if(s===36495)return o.COMPRESSED_RGB_BPTC_UNSIGNED_FLOAT_EXT}else return null;if(s===36283||s===36284||s===36285||s===36286)if(o=t.get("EXT_texture_compression_rgtc"),o!==null){if(s===36492)return o.COMPRESSED_RED_RGTC1_EXT;if(s===36284)return o.COMPRESSED_SIGNED_RED_RGTC1_EXT;if(s===36285)return o.COMPRESSED_RED_GREEN_RGTC2_EXT;if(s===36286)return o.COMPRESSED_SIGNED_RED_GREEN_RGTC2_EXT}else return null;return s===1020?n?r.UNSIGNED_INT_24_8:(o=t.get("WEBGL_depth_texture"),o!==null?o.UNSIGNED_INT_24_8_WEBGL:null):r[s]!==void 0?r[s]:null}return{convert:i}}class Tv extends In{constructor(t=[]){super(),this.isArrayCamera=!0,this.cameras=t}}class Le extends nn{constructor(){super(),this.isGroup=!0,this.type="Group"}}const Av={type:"move"};class Kc{constructor(){this._targetRay=null,this._grip=null,this._hand=null}getHandSpace(){return this._hand===null&&(this._hand=new Le,this._hand.matrixAutoUpdate=!1,this._hand.visible=!1,this._hand.joints={},this._hand.inputState={pinching:!1}),this._hand}getTargetRaySpace(){return this._targetRay===null&&(this._targetRay=new Le,this._targetRay.matrixAutoUpdate=!1,this._targetRay.visible=!1,this._targetRay.hasLinearVelocity=!1,this._targetRay.linearVelocity=new R,this._targetRay.hasAngularVelocity=!1,this._targetRay.angularVelocity=new R),this._targetRay}getGripSpace(){return this._grip===null&&(this._grip=new Le,this._grip.matrixAutoUpdate=!1,this._grip.visible=!1,this._grip.hasLinearVelocity=!1,this._grip.linearVelocity=new R,this._grip.hasAngularVelocity=!1,this._grip.angularVelocity=new R),this._grip}dispatchEvent(t){return this._targetRay!==null&&this._targetRay.dispatchEvent(t),this._grip!==null&&this._grip.dispatchEvent(t),this._hand!==null&&this._hand.dispatchEvent(t),this}connect(t){if(t&&t.hand){const e=this._hand;if(e)for(const n of t.hand.values())this._getHandJoint(e,n)}return this.dispatchEvent({type:"connected",data:t}),this}disconnect(t){return this.dispatchEvent({type:"disconnected",data:t}),this._targetRay!==null&&(this._targetRay.visible=!1),this._grip!==null&&(this._grip.visible=!1),this._hand!==null&&(this._hand.visible=!1),this}update(t,e,n){let i=null,s=null,a=null;const o=this._targetRay,c=this._grip,l=this._hand;if(t&&e.session.visibilityState!=="visible-blurred"){if(l&&t.hand){a=!0;for(const x of t.hand.values()){const m=e.getJointPose(x,n),p=this._getHandJoint(l,x);m!==null&&(p.matrix.fromArray(m.transform.matrix),p.matrix.decompose(p.position,p.rotation,p.scale),p.matrixWorldNeedsUpdate=!0,p.jointRadius=m.radius),p.visible=m!==null}const h=l.joints["index-finger-tip"],u=l.joints["thumb-tip"],d=h.position.distanceTo(u.position),f=.02,g=.005;l.inputState.pinching&&d>f+g?(l.inputState.pinching=!1,this.dispatchEvent({type:"pinchend",handedness:t.handedness,target:this})):!l.inputState.pinching&&d<=f-g&&(l.inputState.pinching=!0,this.dispatchEvent({type:"pinchstart",handedness:t.handedness,target:this}))}else c!==null&&t.gripSpace&&(s=e.getPose(t.gripSpace,n),s!==null&&(c.matrix.fromArray(s.transform.matrix),c.matrix.decompose(c.position,c.rotation,c.scale),c.matrixWorldNeedsUpdate=!0,s.linearVelocity?(c.hasLinearVelocity=!0,c.linearVelocity.copy(s.linearVelocity)):c.hasLinearVelocity=!1,s.angularVelocity?(c.hasAngularVelocity=!0,c.angularVelocity.copy(s.angularVelocity)):c.hasAngularVelocity=!1));o!==null&&(i=e.getPose(t.targetRaySpace,n),i===null&&s!==null&&(i=s),i!==null&&(o.matrix.fromArray(i.transform.matrix),o.matrix.decompose(o.position,o.rotation,o.scale),o.matrixWorldNeedsUpdate=!0,i.linearVelocity?(o.hasLinearVelocity=!0,o.linearVelocity.copy(i.linearVelocity)):o.hasLinearVelocity=!1,i.angularVelocity?(o.hasAngularVelocity=!0,o.angularVelocity.copy(i.angularVelocity)):o.hasAngularVelocity=!1,this.dispatchEvent(Av)))}return o!==null&&(o.visible=i!==null),c!==null&&(c.visible=s!==null),l!==null&&(l.visible=a!==null),this}_getHandJoint(t,e){if(t.joints[e.jointName]===void 0){const n=new Le;n.matrixAutoUpdate=!1,n.visible=!1,t.joints[e.jointName]=n,t.add(n)}return t.joints[e.jointName]}}class Cv extends Tr{constructor(t,e){super();const n=this;let i=null,s=1,a=null,o="local-floor",c=1,l=null,h=null,u=null,d=null,f=null,g=null;const x=e.getContextAttributes();let m=null,p=null;const _=[],v=[],S=new Mt;let M=null;const y=new In;y.layers.enable(1),y.viewport=new en;const w=new In;w.layers.enable(2),w.viewport=new en;const A=[y,w],E=new Tv;E.layers.enable(1),E.layers.enable(2);let T=null,L=null;this.cameraAutoUpdate=!0,this.enabled=!1,this.isPresenting=!1,this.getController=function(V){let K=_[V];return K===void 0&&(K=new Kc,_[V]=K),K.getTargetRaySpace()},this.getControllerGrip=function(V){let K=_[V];return K===void 0&&(K=new Kc,_[V]=K),K.getGripSpace()},this.getHand=function(V){let K=_[V];return K===void 0&&(K=new Kc,_[V]=K),K.getHandSpace()};function C(V){const K=v.indexOf(V.inputSource);if(K===-1)return;const nt=_[K];nt!==void 0&&(nt.update(V.inputSource,V.frame,l||a),nt.dispatchEvent({type:V.type,data:V.inputSource}))}function F(){i.removeEventListener("select",C),i.removeEventListener("selectstart",C),i.removeEventListener("selectend",C),i.removeEventListener("squeeze",C),i.removeEventListener("squeezestart",C),i.removeEventListener("squeezeend",C),i.removeEventListener("end",F),i.removeEventListener("inputsourceschange",D);for(let V=0;V<_.length;V++){const K=v[V];K!==null&&(v[V]=null,_[V].disconnect(K))}T=null,L=null,t.setRenderTarget(m),f=null,d=null,u=null,i=null,p=null,Q.stop(),n.isPresenting=!1,t.setPixelRatio(M),t.setSize(S.width,S.height,!1),n.dispatchEvent({type:"sessionend"})}this.setFramebufferScaleFactor=function(V){s=V,n.isPresenting===!0&&console.warn("THREE.WebXRManager: Cannot change framebuffer scale while presenting.")},this.setReferenceSpaceType=function(V){o=V,n.isPresenting===!0&&console.warn("THREE.WebXRManager: Cannot change reference space type while presenting.")},this.getReferenceSpace=function(){return l||a},this.setReferenceSpace=function(V){l=V},this.getBaseLayer=function(){return d!==null?d:f},this.getBinding=function(){return u},this.getFrame=function(){return g},this.getSession=function(){return i},this.setSession=async function(V){if(i=V,i!==null){if(m=t.getRenderTarget(),i.addEventListener("select",C),i.addEventListener("selectstart",C),i.addEventListener("selectend",C),i.addEventListener("squeeze",C),i.addEventListener("squeezestart",C),i.addEventListener("squeezeend",C),i.addEventListener("end",F),i.addEventListener("inputsourceschange",D),x.xrCompatible!==!0&&await e.makeXRCompatible(),M=t.getPixelRatio(),t.getSize(S),i.renderState.layers===void 0||t.capabilities.isWebGL2===!1){const K={antialias:i.renderState.layers===void 0?x.antialias:!0,alpha:!0,depth:x.depth,stencil:x.stencil,framebufferScaleFactor:s};f=new XRWebGLLayer(i,e,K),i.updateRenderState({baseLayer:f}),t.setPixelRatio(1),t.setSize(f.framebufferWidth,f.framebufferHeight,!1),p=new sn(f.framebufferWidth,f.framebufferHeight,{format:1023,type:1009,colorSpace:t.outputColorSpace,stencilBuffer:x.stencil})}else{let K=null,nt=null,rt=null;x.depth&&(rt=x.stencil?e.DEPTH24_STENCIL8:e.DEPTH_COMPONENT24,K=x.stencil?1027:1026,nt=x.stencil?1020:1014);const ot={colorFormat:e.RGBA8,depthFormat:rt,scaleFactor:s};u=new XRWebGLBinding(i,e),d=u.createProjectionLayer(ot),i.updateRenderState({layers:[d]}),t.setPixelRatio(1),t.setSize(d.textureWidth,d.textureHeight,!1),p=new sn(d.textureWidth,d.textureHeight,{format:1023,type:1009,depthTexture:new Zf(d.textureWidth,d.textureHeight,nt,void 0,void 0,void 0,void 0,void 0,void 0,K),stencilBuffer:x.stencil,colorSpace:t.outputColorSpace,samples:x.antialias?4:0});const _t=t.properties.get(p);_t.__ignoreDepthValues=d.ignoreDepthValues}p.isXRRenderTarget=!0,this.setFoveation(c),l=null,a=await i.requestReferenceSpace(o),Q.setContext(i),Q.start(),n.isPresenting=!0,n.dispatchEvent({type:"sessionstart"})}},this.getEnvironmentBlendMode=function(){if(i!==null)return i.environmentBlendMode};function D(V){for(let K=0;K<V.removed.length;K++){const nt=V.removed[K],rt=v.indexOf(nt);rt>=0&&(v[rt]=null,_[rt].disconnect(nt))}for(let K=0;K<V.added.length;K++){const nt=V.added[K];let rt=v.indexOf(nt);if(rt===-1){for(let _t=0;_t<_.length;_t++)if(_t>=v.length){v.push(nt),rt=_t;break}else if(v[_t]===null){v[_t]=nt,rt=_t;break}if(rt===-1)break}const ot=_[rt];ot&&ot.connect(nt)}}const N=new R,B=new R;function G(V,K,nt){N.setFromMatrixPosition(K.matrixWorld),B.setFromMatrixPosition(nt.matrixWorld);const rt=N.distanceTo(B),ot=K.projectionMatrix.elements,_t=nt.projectionMatrix.elements,ht=ot[14]/(ot[10]-1),ut=ot[14]/(ot[10]+1),Ct=(ot[9]+1)/ot[5],W=(ot[9]-1)/ot[5],Nt=(ot[8]-1)/ot[0],Tt=(_t[8]+1)/_t[0],Pt=ht*Nt,yt=ht*Tt,Kt=rt/(-Nt+Tt),bt=Kt*-Nt;K.matrixWorld.decompose(V.position,V.quaternion,V.scale),V.translateX(bt),V.translateZ(Kt),V.matrixWorld.compose(V.position,V.quaternion,V.scale),V.matrixWorldInverse.copy(V.matrixWorld).invert();const I=ht+Kt,b=ut+Kt,X=Pt-bt,et=yt+(rt-bt),J=Ct*ut/b*I,tt=W*ut/b*I;V.projectionMatrix.makePerspective(X,et,J,tt,I,b),V.projectionMatrixInverse.copy(V.projectionMatrix).invert()}function z(V,K){K===null?V.matrixWorld.copy(V.matrix):V.matrixWorld.multiplyMatrices(K.matrixWorld,V.matrix),V.matrixWorldInverse.copy(V.matrixWorld).invert()}this.updateCamera=function(V){if(i===null)return;E.near=w.near=y.near=V.near,E.far=w.far=y.far=V.far,(T!==E.near||L!==E.far)&&(i.updateRenderState({depthNear:E.near,depthFar:E.far}),T=E.near,L=E.far);const K=V.parent,nt=E.cameras;z(E,K);for(let rt=0;rt<nt.length;rt++)z(nt[rt],K);nt.length===2?G(E,y,w):E.projectionMatrix.copy(y.projectionMatrix),H(V,E,K)};function H(V,K,nt){nt===null?V.matrix.copy(K.matrixWorld):(V.matrix.copy(nt.matrixWorld),V.matrix.invert(),V.matrix.multiply(K.matrixWorld)),V.matrix.decompose(V.position,V.quaternion,V.scale),V.updateMatrixWorld(!0),V.projectionMatrix.copy(K.projectionMatrix),V.projectionMatrixInverse.copy(K.projectionMatrixInverse),V.isPerspectiveCamera&&(V.fov=Pa*2*Math.atan(1/V.projectionMatrix.elements[5]),V.zoom=1)}this.getCamera=function(){return E},this.getFoveation=function(){if(!(d===null&&f===null))return c},this.setFoveation=function(V){c=V,d!==null&&(d.fixedFoveation=V),f!==null&&f.fixedFoveation!==void 0&&(f.fixedFoveation=V)};let j=null;function $(V,K){if(h=K.getViewerPose(l||a),g=K,h!==null){const nt=h.views;f!==null&&(t.setRenderTargetFramebuffer(p,f.framebuffer),t.setRenderTarget(p));let rt=!1;nt.length!==E.cameras.length&&(E.cameras.length=0,rt=!0);for(let ot=0;ot<nt.length;ot++){const _t=nt[ot];let ht=null;if(f!==null)ht=f.getViewport(_t);else{const Ct=u.getViewSubImage(d,_t);ht=Ct.viewport,ot===0&&(t.setRenderTargetTextures(p,Ct.colorTexture,d.ignoreDepthValues?void 0:Ct.depthStencilTexture),t.setRenderTarget(p))}let ut=A[ot];ut===void 0&&(ut=new In,ut.layers.enable(ot),ut.viewport=new en,A[ot]=ut),ut.matrix.fromArray(_t.transform.matrix),ut.matrix.decompose(ut.position,ut.quaternion,ut.scale),ut.projectionMatrix.fromArray(_t.projectionMatrix),ut.projectionMatrixInverse.copy(ut.projectionMatrix).invert(),ut.viewport.set(ht.x,ht.y,ht.width,ht.height),ot===0&&(E.matrix.copy(ut.matrix),E.matrix.decompose(E.position,E.quaternion,E.scale)),rt===!0&&E.cameras.push(ut)}}for(let nt=0;nt<_.length;nt++){const rt=v[nt],ot=_[nt];rt!==null&&ot!==void 0&&ot.update(rt,K,l||a)}j&&j(V,K),K.detectedPlanes&&n.dispatchEvent({type:"planesdetected",data:K}),g=null}const Q=new qf;Q.setAnimationLoop($),this.setAnimationLoop=function(V){j=V},this.dispose=function(){}}}function Rv(r,t){function e(m,p){m.matrixAutoUpdate===!0&&m.updateMatrix(),p.value.copy(m.matrix)}function n(m,p){p.color.getRGB(m.fogColor.value,Wf(r)),p.isFog?(m.fogNear.value=p.near,m.fogFar.value=p.far):p.isFogExp2&&(m.fogDensity.value=p.density)}function i(m,p,_,v,S){p.isMeshBasicMaterial||p.isMeshLambertMaterial?s(m,p):p.isMeshToonMaterial?(s(m,p),u(m,p)):p.isMeshPhongMaterial?(s(m,p),h(m,p)):p.isMeshStandardMaterial?(s(m,p),d(m,p),p.isMeshPhysicalMaterial&&f(m,p,S)):p.isMeshMatcapMaterial?(s(m,p),g(m,p)):p.isMeshDepthMaterial?s(m,p):p.isMeshDistanceMaterial?(s(m,p),x(m,p)):p.isMeshNormalMaterial?s(m,p):p.isLineBasicMaterial?(a(m,p),p.isLineDashedMaterial&&o(m,p)):p.isPointsMaterial?c(m,p,_,v):p.isSpriteMaterial?l(m,p):p.isShadowMaterial?(m.color.value.copy(p.color),m.opacity.value=p.opacity):p.isShaderMaterial&&(p.uniformsNeedUpdate=!1)}function s(m,p){m.opacity.value=p.opacity,p.color&&m.diffuse.value.copy(p.color),p.emissive&&m.emissive.value.copy(p.emissive).multiplyScalar(p.emissiveIntensity),p.map&&(m.map.value=p.map,e(p.map,m.mapTransform)),p.alphaMap&&(m.alphaMap.value=p.alphaMap,e(p.alphaMap,m.alphaMapTransform)),p.bumpMap&&(m.bumpMap.value=p.bumpMap,e(p.bumpMap,m.bumpMapTransform),m.bumpScale.value=p.bumpScale,p.side===1&&(m.bumpScale.value*=-1)),p.normalMap&&(m.normalMap.value=p.normalMap,e(p.normalMap,m.normalMapTransform),m.normalScale.value.copy(p.normalScale),p.side===1&&m.normalScale.value.negate()),p.displacementMap&&(m.displacementMap.value=p.displacementMap,e(p.displacementMap,m.displacementMapTransform),m.displacementScale.value=p.displacementScale,m.displacementBias.value=p.displacementBias),p.emissiveMap&&(m.emissiveMap.value=p.emissiveMap,e(p.emissiveMap,m.emissiveMapTransform)),p.specularMap&&(m.specularMap.value=p.specularMap,e(p.specularMap,m.specularMapTransform)),p.alphaTest>0&&(m.alphaTest.value=p.alphaTest);const _=t.get(p).envMap;if(_&&(m.envMap.value=_,m.flipEnvMap.value=_.isCubeTexture&&_.isRenderTargetTexture===!1?-1:1,m.reflectivity.value=p.reflectivity,m.ior.value=p.ior,m.refractionRatio.value=p.refractionRatio),p.lightMap){m.lightMap.value=p.lightMap;const v=r._useLegacyLights===!0?Math.PI:1;m.lightMapIntensity.value=p.lightMapIntensity*v,e(p.lightMap,m.lightMapTransform)}p.aoMap&&(m.aoMap.value=p.aoMap,m.aoMapIntensity.value=p.aoMapIntensity,e(p.aoMap,m.aoMapTransform))}function a(m,p){m.diffuse.value.copy(p.color),m.opacity.value=p.opacity,p.map&&(m.map.value=p.map,e(p.map,m.mapTransform))}function o(m,p){m.dashSize.value=p.dashSize,m.totalSize.value=p.dashSize+p.gapSize,m.scale.value=p.scale}function c(m,p,_,v){m.diffuse.value.copy(p.color),m.opacity.value=p.opacity,m.size.value=p.size*_,m.scale.value=v*.5,p.map&&(m.map.value=p.map,e(p.map,m.uvTransform)),p.alphaMap&&(m.alphaMap.value=p.alphaMap,e(p.alphaMap,m.alphaMapTransform)),p.alphaTest>0&&(m.alphaTest.value=p.alphaTest)}function l(m,p){m.diffuse.value.copy(p.color),m.opacity.value=p.opacity,m.rotation.value=p.rotation,p.map&&(m.map.value=p.map,e(p.map,m.mapTransform)),p.alphaMap&&(m.alphaMap.value=p.alphaMap,e(p.alphaMap,m.alphaMapTransform)),p.alphaTest>0&&(m.alphaTest.value=p.alphaTest)}function h(m,p){m.specular.value.copy(p.specular),m.shininess.value=Math.max(p.shininess,1e-4)}function u(m,p){p.gradientMap&&(m.gradientMap.value=p.gradientMap)}function d(m,p){m.metalness.value=p.metalness,p.metalnessMap&&(m.metalnessMap.value=p.metalnessMap,e(p.metalnessMap,m.metalnessMapTransform)),m.roughness.value=p.roughness,p.roughnessMap&&(m.roughnessMap.value=p.roughnessMap,e(p.roughnessMap,m.roughnessMapTransform)),t.get(p).envMap&&(m.envMapIntensity.value=p.envMapIntensity)}function f(m,p,_){m.ior.value=p.ior,p.sheen>0&&(m.sheenColor.value.copy(p.sheenColor).multiplyScalar(p.sheen),m.sheenRoughness.value=p.sheenRoughness,p.sheenColorMap&&(m.sheenColorMap.value=p.sheenColorMap,e(p.sheenColorMap,m.sheenColorMapTransform)),p.sheenRoughnessMap&&(m.sheenRoughnessMap.value=p.sheenRoughnessMap,e(p.sheenRoughnessMap,m.sheenRoughnessMapTransform))),p.clearcoat>0&&(m.clearcoat.value=p.clearcoat,m.clearcoatRoughness.value=p.clearcoatRoughness,p.clearcoatMap&&(m.clearcoatMap.value=p.clearcoatMap,e(p.clearcoatMap,m.clearcoatMapTransform)),p.clearcoatRoughnessMap&&(m.clearcoatRoughnessMap.value=p.clearcoatRoughnessMap,e(p.clearcoatRoughnessMap,m.clearcoatRoughnessMapTransform)),p.clearcoatNormalMap&&(m.clearcoatNormalMap.value=p.clearcoatNormalMap,e(p.clearcoatNormalMap,m.clearcoatNormalMapTransform),m.clearcoatNormalScale.value.copy(p.clearcoatNormalScale),p.side===1&&m.clearcoatNormalScale.value.negate())),p.iridescence>0&&(m.iridescence.value=p.iridescence,m.iridescenceIOR.value=p.iridescenceIOR,m.iridescenceThicknessMinimum.value=p.iridescenceThicknessRange[0],m.iridescenceThicknessMaximum.value=p.iridescenceThicknessRange[1],p.iridescenceMap&&(m.iridescenceMap.value=p.iridescenceMap,e(p.iridescenceMap,m.iridescenceMapTransform)),p.iridescenceThicknessMap&&(m.iridescenceThicknessMap.value=p.iridescenceThicknessMap,e(p.iridescenceThicknessMap,m.iridescenceThicknessMapTransform))),p.transmission>0&&(m.transmission.value=p.transmission,m.transmissionSamplerMap.value=_.texture,m.transmissionSamplerSize.value.set(_.width,_.height),p.transmissionMap&&(m.transmissionMap.value=p.transmissionMap,e(p.transmissionMap,m.transmissionMapTransform)),m.thickness.value=p.thickness,p.thicknessMap&&(m.thicknessMap.value=p.thicknessMap,e(p.thicknessMap,m.thicknessMapTransform)),m.attenuationDistance.value=p.attenuationDistance,m.attenuationColor.value.copy(p.attenuationColor)),p.anisotropy>0&&(m.anisotropyVector.value.set(p.anisotropy*Math.cos(p.anisotropyRotation),p.anisotropy*Math.sin(p.anisotropyRotation)),p.anisotropyMap&&(m.anisotropyMap.value=p.anisotropyMap,e(p.anisotropyMap,m.anisotropyMapTransform))),m.specularIntensity.value=p.specularIntensity,m.specularColor.value.copy(p.specularColor),p.specularColorMap&&(m.specularColorMap.value=p.specularColorMap,e(p.specularColorMap,m.specularColorMapTransform)),p.specularIntensityMap&&(m.specularIntensityMap.value=p.specularIntensityMap,e(p.specularIntensityMap,m.specularIntensityMapTransform))}function g(m,p){p.matcap&&(m.matcap.value=p.matcap)}function x(m,p){const _=t.get(p).light;m.referencePosition.value.setFromMatrixPosition(_.matrixWorld),m.nearDistance.value=_.shadow.camera.near,m.farDistance.value=_.shadow.camera.far}return{refreshFogUniforms:n,refreshMaterialUniforms:i}}function bv(r,t,e,n){let i={},s={},a=[];const o=e.isWebGL2?r.getParameter(r.MAX_UNIFORM_BUFFER_BINDINGS):0;function c(_,v){const S=v.program;n.uniformBlockBinding(_,S)}function l(_,v){let S=i[_.id];S===void 0&&(g(_),S=h(_),i[_.id]=S,_.addEventListener("dispose",m));const M=v.program;n.updateUBOMapping(_,M);const y=t.render.frame;s[_.id]!==y&&(d(_),s[_.id]=y)}function h(_){const v=u();_.__bindingPointIndex=v;const S=r.createBuffer(),M=_.__size,y=_.usage;return r.bindBuffer(r.UNIFORM_BUFFER,S),r.bufferData(r.UNIFORM_BUFFER,M,y),r.bindBuffer(r.UNIFORM_BUFFER,null),r.bindBufferBase(r.UNIFORM_BUFFER,v,S),S}function u(){for(let _=0;_<o;_++)if(a.indexOf(_)===-1)return a.push(_),_;return console.error("THREE.WebGLRenderer: Maximum number of simultaneously usable uniforms groups reached."),0}function d(_){const v=i[_.id],S=_.uniforms,M=_.__cache;r.bindBuffer(r.UNIFORM_BUFFER,v);for(let y=0,w=S.length;y<w;y++){const A=Array.isArray(S[y])?S[y]:[S[y]];for(let E=0,T=A.length;E<T;E++){const L=A[E];if(f(L,y,E,M)===!0){const C=L.__offset,F=Array.isArray(L.value)?L.value:[L.value];let D=0;for(let N=0;N<F.length;N++){const B=F[N],G=x(B);typeof B=="number"||typeof B=="boolean"?(L.__data[0]=B,r.bufferSubData(r.UNIFORM_BUFFER,C+D,L.__data)):B.isMatrix3?(L.__data[0]=B.elements[0],L.__data[1]=B.elements[1],L.__data[2]=B.elements[2],L.__data[3]=0,L.__data[4]=B.elements[3],L.__data[5]=B.elements[4],L.__data[6]=B.elements[5],L.__data[7]=0,L.__data[8]=B.elements[6],L.__data[9]=B.elements[7],L.__data[10]=B.elements[8],L.__data[11]=0):(B.toArray(L.__data,D),D+=G.storage/Float32Array.BYTES_PER_ELEMENT)}r.bufferSubData(r.UNIFORM_BUFFER,C,L.__data)}}}r.bindBuffer(r.UNIFORM_BUFFER,null)}function f(_,v,S,M){const y=_.value,w=v+"_"+S;if(M[w]===void 0)return typeof y=="number"||typeof y=="boolean"?M[w]=y:M[w]=y.clone(),!0;{const A=M[w];if(typeof y=="number"||typeof y=="boolean"){if(A!==y)return M[w]=y,!0}else if(A.equals(y)===!1)return A.copy(y),!0}return!1}function g(_){const v=_.uniforms;let S=0;const M=16;for(let w=0,A=v.length;w<A;w++){const E=Array.isArray(v[w])?v[w]:[v[w]];for(let T=0,L=E.length;T<L;T++){const C=E[T],F=Array.isArray(C.value)?C.value:[C.value];for(let D=0,N=F.length;D<N;D++){const B=F[D],G=x(B),z=S%M;z!==0&&M-z<G.boundary&&(S+=M-z),C.__data=new Float32Array(G.storage/Float32Array.BYTES_PER_ELEMENT),C.__offset=S,S+=G.storage}}}const y=S%M;return y>0&&(S+=M-y),_.__size=S,_.__cache={},this}function x(_){const v={boundary:0,storage:0};return typeof _=="number"||typeof _=="boolean"?(v.boundary=4,v.storage=4):_.isVector2?(v.boundary=8,v.storage=8):_.isVector3||_.isColor?(v.boundary=16,v.storage=12):_.isVector4?(v.boundary=16,v.storage=16):_.isMatrix3?(v.boundary=48,v.storage=48):_.isMatrix4?(v.boundary=64,v.storage=64):_.isTexture?console.warn("THREE.WebGLRenderer: Texture samplers can not be part of an uniforms group."):console.warn("THREE.WebGLRenderer: Unsupported uniform value type.",_),v}function m(_){const v=_.target;v.removeEventListener("dispose",m);const S=a.indexOf(v.__bindingPointIndex);a.splice(S,1),r.deleteBuffer(i[v.id]),delete i[v.id],delete s[v.id]}function p(){for(const _ in i)r.deleteBuffer(i[_]);a=[],i={},s={}}return{bind:c,update:l,dispose:p}}class Oh{constructor(t={}){const{canvas:e=Tm(),context:n=null,depth:i=!0,stencil:s=!0,alpha:a=!1,antialias:o=!1,premultipliedAlpha:c=!0,preserveDrawingBuffer:l=!1,powerPreference:h="default",failIfMajorPerformanceCaveat:u=!1}=t;this.isWebGLRenderer=!0;let d;n!==null?d=n.getContextAttributes().alpha:d=a;const f=new Uint32Array(4),g=new Int32Array(4);let x=null,m=null;const p=[],_=[];this.domElement=e,this.debug={checkShaderErrors:!0,onShaderError:null},this.autoClear=!0,this.autoClearColor=!0,this.autoClearDepth=!0,this.autoClearStencil=!0,this.sortObjects=!0,this.clippingPlanes=[],this.localClippingEnabled=!1,this._outputColorSpace=an,this._useLegacyLights=!1,this.toneMapping=0,this.toneMappingExposure=1;const v=this;let S=!1,M=0,y=0,w=null,A=-1,E=null;const T=new en,L=new en;let C=null;const F=new Qt(0);let D=0,N=e.width,B=e.height,G=1,z=null,H=null;const j=new en(0,0,N,B),$=new en(0,0,N,B);let Q=!1;const V=new Bh;let K=!1,nt=!1,rt=null;const ot=new re,_t=new Mt,ht=new R,ut={background:null,fog:null,environment:null,overrideMaterial:null,isScene:!0};function Ct(){return w===null?G:1}let W=n;function Nt(P,k){for(let Z=0;Z<P.length;Z++){const q=P[Z],Y=e.getContext(q,k);if(Y!==null)return Y}return null}try{const P={alpha:!0,depth:i,stencil:s,antialias:o,premultipliedAlpha:c,preserveDrawingBuffer:l,powerPreference:h,failIfMajorPerformanceCaveat:u};if("setAttribute"in e&&e.setAttribute("data-engine","three.js r160"),e.addEventListener("webglcontextlost",ct,!1),e.addEventListener("webglcontextrestored",U,!1),e.addEventListener("webglcontextcreationerror",dt,!1),W===null){const k=["webgl2","webgl","experimental-webgl"];if(v.isWebGL1Renderer===!0&&k.shift(),W=Nt(k,P),W===null)throw Nt(k)?new Error("Error creating WebGL context with your selected attributes."):new Error("Error creating WebGL context.")}typeof WebGLRenderingContext<"u"&&W instanceof WebGLRenderingContext&&console.warn("THREE.WebGLRenderer: WebGL 1 support was deprecated in r153 and will be removed in r163."),W.getShaderPrecisionFormat===void 0&&(W.getShaderPrecisionFormat=function(){return{rangeMin:1,rangeMax:1,precision:1}})}catch(P){throw console.error("THREE.WebGLRenderer: "+P.message),P}let Tt,Pt,yt,Kt,bt,I,b,X,et,J,tt,St,lt,gt,At,Ht,it,ce,Jt,Gt,Lt,vt,Ft,te;function le(){Tt=new Ox(W),Pt=new Ix(W,Tt,t),Tt.init(Pt),vt=new wv(W,Tt,Pt),yt=new yv(W,Tt,Pt),Kt=new kx(W),bt=new cv,I=new Ev(W,Tt,yt,bt,Pt,vt,Kt),b=new Fx(v),X=new zx(v),et=new $m(W,Pt),Ft=new Lx(W,Tt,et,Pt),J=new Gx(W,et,Kt,Ft),tt=new Yx(W,J,et,Kt),Jt=new Xx(W,Pt,I),Ht=new Nx(bt),St=new ov(v,b,X,Tt,Pt,Ft,Ht),lt=new Rv(v,bt),gt=new hv,At=new gv(Tt,Pt),ce=new Px(v,b,X,yt,tt,d,c),it=new Mv(v,tt,Pt),te=new bv(W,Kt,Pt,yt),Gt=new Dx(W,Tt,Kt,Pt),Lt=new Vx(W,Tt,Kt,Pt),Kt.programs=St.programs,v.capabilities=Pt,v.extensions=Tt,v.properties=bt,v.renderLists=gt,v.shadowMap=it,v.state=yt,v.info=Kt}le();const Wt=new Cv(v,W);this.xr=Wt,this.getContext=function(){return W},this.getContextAttributes=function(){return W.getContextAttributes()},this.forceContextLoss=function(){const P=Tt.get("WEBGL_lose_context");P&&P.loseContext()},this.forceContextRestore=function(){const P=Tt.get("WEBGL_lose_context");P&&P.restoreContext()},this.getPixelRatio=function(){return G},this.setPixelRatio=function(P){P!==void 0&&(G=P,this.setSize(N,B,!1))},this.getSize=function(P){return P.set(N,B)},this.setSize=function(P,k,Z=!0){if(Wt.isPresenting){console.warn("THREE.WebGLRenderer: Can't change size while VR device is presenting.");return}N=P,B=k,e.width=Math.floor(P*G),e.height=Math.floor(k*G),Z===!0&&(e.style.width=P+"px",e.style.height=k+"px"),this.setViewport(0,0,P,k)},this.getDrawingBufferSize=function(P){return P.set(N*G,B*G).floor()},this.setDrawingBufferSize=function(P,k,Z){N=P,B=k,G=Z,e.width=Math.floor(P*Z),e.height=Math.floor(k*Z),this.setViewport(0,0,P,k)},this.getCurrentViewport=function(P){return P.copy(T)},this.getViewport=function(P){return P.copy(j)},this.setViewport=function(P,k,Z,q){P.isVector4?j.set(P.x,P.y,P.z,P.w):j.set(P,k,Z,q),yt.viewport(T.copy(j).multiplyScalar(G).floor())},this.getScissor=function(P){return P.copy($)},this.setScissor=function(P,k,Z,q){P.isVector4?$.set(P.x,P.y,P.z,P.w):$.set(P,k,Z,q),yt.scissor(L.copy($).multiplyScalar(G).floor())},this.getScissorTest=function(){return Q},this.setScissorTest=function(P){yt.setScissorTest(Q=P)},this.setOpaqueSort=function(P){z=P},this.setTransparentSort=function(P){H=P},this.getClearColor=function(P){return P.copy(ce.getClearColor())},this.setClearColor=function(){ce.setClearColor.apply(ce,arguments)},this.getClearAlpha=function(){return ce.getClearAlpha()},this.setClearAlpha=function(){ce.setClearAlpha.apply(ce,arguments)},this.clear=function(P=!0,k=!0,Z=!0){let q=0;if(P){let Y=!1;if(w!==null){const at=w.texture.format;Y=at===1033||at===1031||at===1029}if(Y){const at=w.texture.type,Et=at===1009||at===1014||at===1012||at===1020||at===1017||at===1018,Dt=ce.getClearColor(),xt=ce.getClearAlpha(),Vt=Dt.r,kt=Dt.g,Ot=Dt.b;Et?(f[0]=Vt,f[1]=kt,f[2]=Ot,f[3]=xt,W.clearBufferuiv(W.COLOR,0,f)):(g[0]=Vt,g[1]=kt,g[2]=Ot,g[3]=xt,W.clearBufferiv(W.COLOR,0,g))}else q|=W.COLOR_BUFFER_BIT}k&&(q|=W.DEPTH_BUFFER_BIT),Z&&(q|=W.STENCIL_BUFFER_BIT,this.state.buffers.stencil.setMask(4294967295)),W.clear(q)},this.clearColor=function(){this.clear(!0,!1,!1)},this.clearDepth=function(){this.clear(!1,!0,!1)},this.clearStencil=function(){this.clear(!1,!1,!0)},this.dispose=function(){e.removeEventListener("webglcontextlost",ct,!1),e.removeEventListener("webglcontextrestored",U,!1),e.removeEventListener("webglcontextcreationerror",dt,!1),gt.dispose(),At.dispose(),bt.dispose(),b.dispose(),X.dispose(),tt.dispose(),Ft.dispose(),te.dispose(),St.dispose(),Wt.dispose(),Wt.removeEventListener("sessionstart",we),Wt.removeEventListener("sessionend",se),rt&&(rt.dispose(),rt=null),Ce.stop()};function ct(P){P.preventDefault(),console.log("THREE.WebGLRenderer: Context Lost."),S=!0}function U(){console.log("THREE.WebGLRenderer: Context Restored."),S=!1;const P=Kt.autoReset,k=it.enabled,Z=it.autoUpdate,q=it.needsUpdate,Y=it.type;le(),Kt.autoReset=P,it.enabled=k,it.autoUpdate=Z,it.needsUpdate=q,it.type=Y}function dt(P){console.error("THREE.WebGLRenderer: A WebGL context could not be created. Reason: ",P.statusMessage)}function ft(P){const k=P.target;k.removeEventListener("dispose",ft),zt(k)}function zt(P){Bt(P),bt.remove(P)}function Bt(P){const k=bt.get(P).programs;k!==void 0&&(k.forEach(function(Z){St.releaseProgram(Z)}),P.isShaderMaterial&&St.releaseShaderCache(P))}this.renderBufferDirect=function(P,k,Z,q,Y,at){k===null&&(k=ut);const Et=Y.isMesh&&Y.matrixWorld.determinant()<0,Dt=os(P,k,Z,q,Y);yt.setMaterial(q,Et);let xt=Z.index,Vt=1;if(q.wireframe===!0){if(xt=J.getWireframeAttribute(Z),xt===void 0)return;Vt=2}const kt=Z.drawRange,Ot=Z.attributes.position;let fe=kt.start*Vt,Je=(kt.start+kt.count)*Vt;at!==null&&(fe=Math.max(fe,at.start*Vt),Je=Math.min(Je,(at.start+at.count)*Vt)),xt!==null?(fe=Math.max(fe,0),Je=Math.min(Je,xt.count)):Ot!=null&&(fe=Math.max(fe,0),Je=Math.min(Je,Ot.count));const Re=Je-fe;if(Re<0||Re===1/0)return;Ft.setup(Y,q,Dt,Z,xt);let Zn,Se=Gt;if(xt!==null&&(Zn=et.get(xt),Se=Lt,Se.setIndex(Zn)),Y.isMesh)q.wireframe===!0?(yt.setLineWidth(q.wireframeLinewidth*Ct()),Se.setMode(W.LINES)):Se.setMode(W.TRIANGLES);else if(Y.isLine){let $t=q.linewidth;$t===void 0&&($t=1),yt.setLineWidth($t*Ct()),Y.isLineSegments?Se.setMode(W.LINES):Y.isLineLoop?Se.setMode(W.LINE_LOOP):Se.setMode(W.LINE_STRIP)}else Y.isPoints?Se.setMode(W.POINTS):Y.isSprite&&Se.setMode(W.TRIANGLES);if(Y.isBatchedMesh)Se.renderMultiDraw(Y._multiDrawStarts,Y._multiDrawCounts,Y._multiDrawCount);else if(Y.isInstancedMesh)Se.renderInstances(fe,Re,Y.count);else if(Z.isInstancedBufferGeometry){const $t=Z._maxInstanceCount!==void 0?Z._maxInstanceCount:1/0,ki=Math.min(Z.instanceCount,$t);Se.renderInstances(fe,Re,ki)}else Se.render(fe,Re)};function de(P,k,Z){P.transparent===!0&&P.side===2&&P.forceSinglePass===!1?(P.side=1,P.needsUpdate=!0,qe(P,k,Z),P.side=0,P.needsUpdate=!0,qe(P,k,Z),P.side=2):qe(P,k,Z)}this.compile=function(P,k,Z=null){Z===null&&(Z=P),m=At.get(Z),m.init(),_.push(m),Z.traverseVisible(function(Y){Y.isLight&&Y.layers.test(k.layers)&&(m.pushLight(Y),Y.castShadow&&m.pushShadow(Y))}),P!==Z&&P.traverseVisible(function(Y){Y.isLight&&Y.layers.test(k.layers)&&(m.pushLight(Y),Y.castShadow&&m.pushShadow(Y))}),m.setupLights(v._useLegacyLights);const q=new Set;return P.traverse(function(Y){const at=Y.material;if(at)if(Array.isArray(at))for(let Et=0;Et<at.length;Et++){const Dt=at[Et];de(Dt,Z,Y),q.add(Dt)}else de(at,Z,Y),q.add(at)}),_.pop(),m=null,q},this.compileAsync=function(P,k,Z=null){const q=this.compile(P,k,Z);return new Promise(Y=>{function at(){if(q.forEach(function(Et){bt.get(Et).currentProgram.isReady()&&q.delete(Et)}),q.size===0){Y(P);return}setTimeout(at,10)}Tt.get("KHR_parallel_shader_compile")!==null?at():setTimeout(at,10)})};let ue=null;function ve(P){ue&&ue(P)}function we(){Ce.stop()}function se(){Ce.start()}const Ce=new qf;Ce.setAnimationLoop(ve),typeof self<"u"&&Ce.setContext(self),this.setAnimationLoop=function(P){ue=P,Wt.setAnimationLoop(P),P===null?Ce.stop():Ce.start()},Wt.addEventListener("sessionstart",we),Wt.addEventListener("sessionend",se),this.render=function(P,k){if(k!==void 0&&k.isCamera!==!0){console.error("THREE.WebGLRenderer.render: camera is not an instance of THREE.Camera.");return}if(S===!0)return;P.matrixWorldAutoUpdate===!0&&P.updateMatrixWorld(),k.parent===null&&k.matrixWorldAutoUpdate===!0&&k.updateMatrixWorld(),Wt.enabled===!0&&Wt.isPresenting===!0&&(Wt.cameraAutoUpdate===!0&&Wt.updateCamera(k),k=Wt.getCamera()),P.isScene===!0&&P.onBeforeRender(v,P,k,w),m=At.get(P,_.length),m.init(),_.push(m),ot.multiplyMatrices(k.projectionMatrix,k.matrixWorldInverse),V.setFromProjectionMatrix(ot),nt=this.localClippingEnabled,K=Ht.init(this.clippingPlanes,nt),x=gt.get(P,p.length),x.init(),p.push(x),Ye(P,k,0,v.sortObjects),x.finish(),v.sortObjects===!0&&x.sort(z,H),this.info.render.frame++,K===!0&&Ht.beginShadows();const Z=m.state.shadowsArray;if(it.render(Z,P,k),K===!0&&Ht.endShadows(),this.info.autoReset===!0&&this.info.reset(),ce.render(x,P),m.setupLights(v._useLegacyLights),k.isArrayCamera){const q=k.cameras;for(let Y=0,at=q.length;Y<at;Y++){const Et=q[Y];Si(x,P,Et,Et.viewport)}}else Si(x,P,k);w!==null&&(I.updateMultisampleRenderTarget(w),I.updateRenderTargetMipmap(w)),P.isScene===!0&&P.onAfterRender(v,P,k),Ft.resetDefaultState(),A=-1,E=null,_.pop(),_.length>0?m=_[_.length-1]:m=null,p.pop(),p.length>0?x=p[p.length-1]:x=null};function Ye(P,k,Z,q){if(P.visible===!1)return;if(P.layers.test(k.layers)){if(P.isGroup)Z=P.renderOrder;else if(P.isLOD)P.autoUpdate===!0&&P.update(k);else if(P.isLight)m.pushLight(P),P.castShadow&&m.pushShadow(P);else if(P.isSprite){if(!P.frustumCulled||V.intersectsSprite(P)){q&&ht.setFromMatrixPosition(P.matrixWorld).applyMatrix4(ot);const Et=tt.update(P),Dt=P.material;Dt.visible&&x.push(P,Et,Dt,Z,ht.z,null)}}else if((P.isMesh||P.isLine||P.isPoints)&&(!P.frustumCulled||V.intersectsObject(P))){const Et=tt.update(P),Dt=P.material;if(q&&(P.boundingSphere!==void 0?(P.boundingSphere===null&&P.computeBoundingSphere(),ht.copy(P.boundingSphere.center)):(Et.boundingSphere===null&&Et.computeBoundingSphere(),ht.copy(Et.boundingSphere.center)),ht.applyMatrix4(P.matrixWorld).applyMatrix4(ot)),Array.isArray(Dt)){const xt=Et.groups;for(let Vt=0,kt=xt.length;Vt<kt;Vt++){const Ot=xt[Vt],fe=Dt[Ot.materialIndex];fe&&fe.visible&&x.push(P,Et,fe,Z,ht.z,Ot)}}else Dt.visible&&x.push(P,Et,Dt,Z,ht.z,null)}}const at=P.children;for(let Et=0,Dt=at.length;Et<Dt;Et++)Ye(at[Et],k,Z,q)}function Si(P,k,Z,q){const Y=P.opaque,at=P.transmissive,Et=P.transparent;m.setupLightsView(Z),K===!0&&Ht.setGlobalState(v.clippingPlanes,Z),at.length>0&&Ds(Y,at,k,Z),q&&yt.viewport(T.copy(q)),Y.length>0&&Sn(Y,k,Z),at.length>0&&Sn(at,k,Z),Et.length>0&&Sn(Et,k,Z),yt.buffers.depth.setTest(!0),yt.buffers.depth.setMask(!0),yt.buffers.color.setMask(!0),yt.setPolygonOffset(!1)}function Ds(P,k,Z,q){if((Z.isScene===!0?Z.overrideMaterial:null)!==null)return;const at=Pt.isWebGL2;rt===null&&(rt=new sn(1,1,{generateMipmaps:!0,type:Tt.has("EXT_color_buffer_half_float")?1016:1009,minFilter:1008,samples:at?4:0})),v.getDrawingBufferSize(_t),at?rt.setSize(_t.x,_t.y):rt.setSize(Ko(_t.x),Ko(_t.y));const Et=v.getRenderTarget();v.setRenderTarget(rt),v.getClearColor(F),D=v.getClearAlpha(),D<1&&v.setClearColor(16777215,.5),v.clear();const Dt=v.toneMapping;v.toneMapping=0,Sn(P,Z,q),I.updateMultisampleRenderTarget(rt),I.updateRenderTargetMipmap(rt);let xt=!1;for(let Vt=0,kt=k.length;Vt<kt;Vt++){const Ot=k[Vt],fe=Ot.object,Je=Ot.geometry,Re=Ot.material,Zn=Ot.group;if(Re.side===2&&fe.layers.test(q.layers)){const Se=Re.side;Re.side=1,Re.needsUpdate=!0,Rn(fe,Z,q,Je,Re,Zn),Re.side=Se,Re.needsUpdate=!0,xt=!0}}xt===!0&&(I.updateMultisampleRenderTarget(rt),I.updateRenderTargetMipmap(rt)),v.setRenderTarget(Et),v.setClearColor(F,D),v.toneMapping=Dt}function Sn(P,k,Z){const q=k.isScene===!0?k.overrideMaterial:null;for(let Y=0,at=P.length;Y<at;Y++){const Et=P[Y],Dt=Et.object,xt=Et.geometry,Vt=q===null?Et.material:q,kt=Et.group;Dt.layers.test(Z.layers)&&Rn(Dt,k,Z,xt,Vt,kt)}}function Rn(P,k,Z,q,Y,at){P.onBeforeRender(v,k,Z,q,Y,at),P.modelViewMatrix.multiplyMatrices(Z.matrixWorldInverse,P.matrixWorld),P.normalMatrix.getNormalMatrix(P.modelViewMatrix),Y.onBeforeRender(v,k,Z,q,P,at),Y.transparent===!0&&Y.side===2&&Y.forceSinglePass===!1?(Y.side=1,Y.needsUpdate=!0,v.renderBufferDirect(Z,k,q,Y,P,at),Y.side=0,Y.needsUpdate=!0,v.renderBufferDirect(Z,k,q,Y,P,at),Y.side=2):v.renderBufferDirect(Z,k,q,Y,P,at),P.onAfterRender(v,k,Z,q,Y,at)}function qe(P,k,Z){k.isScene!==!0&&(k=ut);const q=bt.get(P),Y=m.state.lights,at=m.state.shadowsArray,Et=Y.state.version,Dt=St.getParameters(P,Y.state,at,k,Z),xt=St.getProgramCacheKey(Dt);let Vt=q.programs;q.environment=P.isMeshStandardMaterial?k.environment:null,q.fog=k.fog,q.envMap=(P.isMeshStandardMaterial?X:b).get(P.envMap||q.environment),Vt===void 0&&(P.addEventListener("dispose",ft),Vt=new Map,q.programs=Vt);let kt=Vt.get(xt);if(kt!==void 0){if(q.currentProgram===kt&&q.lightsStateVersion===Et)return Vi(P,Dt),kt}else Dt.uniforms=St.getUniforms(P),P.onBuild(Z,Dt,v),P.onBeforeCompile(Dt,v),kt=St.acquireProgram(Dt,xt),Vt.set(xt,kt),q.uniforms=Dt.uniforms;const Ot=q.uniforms;return(!P.isShaderMaterial&&!P.isRawShaderMaterial||P.clipping===!0)&&(Ot.clippingPlanes=Ht.uniform),Vi(P,Dt),q.needsLights=Xt(P),q.lightsStateVersion=Et,q.needsLights&&(Ot.ambientLightColor.value=Y.state.ambient,Ot.lightProbe.value=Y.state.probe,Ot.directionalLights.value=Y.state.directional,Ot.directionalLightShadows.value=Y.state.directionalShadow,Ot.spotLights.value=Y.state.spot,Ot.spotLightShadows.value=Y.state.spotShadow,Ot.rectAreaLights.value=Y.state.rectArea,Ot.ltc_1.value=Y.state.rectAreaLTC1,Ot.ltc_2.value=Y.state.rectAreaLTC2,Ot.pointLights.value=Y.state.point,Ot.pointLightShadows.value=Y.state.pointShadow,Ot.hemisphereLights.value=Y.state.hemi,Ot.directionalShadowMap.value=Y.state.directionalShadowMap,Ot.directionalShadowMatrix.value=Y.state.directionalShadowMatrix,Ot.spotShadowMap.value=Y.state.spotShadowMap,Ot.spotLightMatrix.value=Y.state.spotLightMatrix,Ot.spotLightMap.value=Y.state.spotLightMap,Ot.pointShadowMap.value=Y.state.pointShadowMap,Ot.pointShadowMatrix.value=Y.state.pointShadowMatrix),q.currentProgram=kt,q.uniformsList=null,kt}function Un(P){if(P.uniformsList===null){const k=P.currentProgram.getUniforms();P.uniformsList=Go.seqWithValue(k.seq,P.uniforms)}return P.uniformsList}function Vi(P,k){const Z=bt.get(P);Z.outputColorSpace=k.outputColorSpace,Z.batching=k.batching,Z.instancing=k.instancing,Z.instancingColor=k.instancingColor,Z.skinning=k.skinning,Z.morphTargets=k.morphTargets,Z.morphNormals=k.morphNormals,Z.morphColors=k.morphColors,Z.morphTargetsCount=k.morphTargetsCount,Z.numClippingPlanes=k.numClippingPlanes,Z.numIntersection=k.numClipIntersection,Z.vertexAlphas=k.vertexAlphas,Z.vertexTangents=k.vertexTangents,Z.toneMapping=k.toneMapping}function os(P,k,Z,q,Y){k.isScene!==!0&&(k=ut),I.resetTextureUnits();const at=k.fog,Et=q.isMeshStandardMaterial?k.environment:null,Dt=w===null?v.outputColorSpace:w.isXRRenderTarget===!0?w.texture.colorSpace:Ui,xt=(q.isMeshStandardMaterial?X:b).get(q.envMap||Et),Vt=q.vertexColors===!0&&!!Z.attributes.color&&Z.attributes.color.itemSize===4,kt=!!Z.attributes.tangent&&(!!q.normalMap||q.anisotropy>0),Ot=!!Z.morphAttributes.position,fe=!!Z.morphAttributes.normal,Je=!!Z.morphAttributes.color;let Re=0;q.toneMapped&&(w===null||w.isXRRenderTarget===!0)&&(Re=v.toneMapping);const Zn=Z.morphAttributes.position||Z.morphAttributes.normal||Z.morphAttributes.color,Se=Zn!==void 0?Zn.length:0,$t=bt.get(q),ki=m.state.lights;if(K===!0&&(nt===!0||P!==E)){const Mn=P===E&&q.id===A;Ht.setState(q,P,Mn)}let Te=!1;q.version===$t.__version?($t.needsLights&&$t.lightsStateVersion!==ki.state.version||$t.outputColorSpace!==Dt||Y.isBatchedMesh&&$t.batching===!1||!Y.isBatchedMesh&&$t.batching===!0||Y.isInstancedMesh&&$t.instancing===!1||!Y.isInstancedMesh&&$t.instancing===!0||Y.isSkinnedMesh&&$t.skinning===!1||!Y.isSkinnedMesh&&$t.skinning===!0||Y.isInstancedMesh&&$t.instancingColor===!0&&Y.instanceColor===null||Y.isInstancedMesh&&$t.instancingColor===!1&&Y.instanceColor!==null||$t.envMap!==xt||q.fog===!0&&$t.fog!==at||$t.numClippingPlanes!==void 0&&($t.numClippingPlanes!==Ht.numPlanes||$t.numIntersection!==Ht.numIntersection)||$t.vertexAlphas!==Vt||$t.vertexTangents!==kt||$t.morphTargets!==Ot||$t.morphNormals!==fe||$t.morphColors!==Je||$t.toneMapping!==Re||Pt.isWebGL2===!0&&$t.morphTargetsCount!==Se)&&(Te=!0):(Te=!0,$t.__version=q.version);let Mi=$t.currentProgram;Te===!0&&(Mi=qe(q,k,Y));let Ba=!1,cs=!1,Hi=!1;const ze=Mi.getUniforms(),yi=$t.uniforms;if(yt.useProgram(Mi.program)&&(Ba=!0,cs=!0,Hi=!0),q.id!==A&&(A=q.id,cs=!0),Ba||E!==P){ze.setValue(W,"projectionMatrix",P.projectionMatrix),ze.setValue(W,"viewMatrix",P.matrixWorldInverse);const Mn=ze.map.cameraPosition;Mn!==void 0&&Mn.setValue(W,ht.setFromMatrixPosition(P.matrixWorld)),Pt.logarithmicDepthBuffer&&ze.setValue(W,"logDepthBufFC",2/(Math.log(P.far+1)/Math.LN2)),(q.isMeshPhongMaterial||q.isMeshToonMaterial||q.isMeshLambertMaterial||q.isMeshBasicMaterial||q.isMeshStandardMaterial||q.isShaderMaterial)&&ze.setValue(W,"isOrthographic",P.isOrthographicCamera===!0),E!==P&&(E=P,cs=!0,Hi=!0)}if(Y.isSkinnedMesh){ze.setOptional(W,Y,"bindMatrix"),ze.setOptional(W,Y,"bindMatrixInverse");const Mn=Y.skeleton;Mn&&(Pt.floatVertexTextures?(Mn.boneTexture===null&&Mn.computeBoneTexture(),ze.setValue(W,"boneTexture",Mn.boneTexture,I)):console.warn("THREE.WebGLRenderer: SkinnedMesh can only be used with WebGL 2. With WebGL 1 OES_texture_float and vertex textures support is required."))}Y.isBatchedMesh&&(ze.setOptional(W,Y,"batchingTexture"),ze.setValue(W,"batchingTexture",Y._matricesTexture,I));const Lr=Z.morphAttributes;if((Lr.position!==void 0||Lr.normal!==void 0||Lr.color!==void 0&&Pt.isWebGL2===!0)&&Jt.update(Y,Z,Mi),(cs||$t.receiveShadow!==Y.receiveShadow)&&($t.receiveShadow=Y.receiveShadow,ze.setValue(W,"receiveShadow",Y.receiveShadow)),q.isMeshGouraudMaterial&&q.envMap!==null&&(yi.envMap.value=xt,yi.flipEnvMap.value=xt.isCubeTexture&&xt.isRenderTargetTexture===!1?-1:1),cs&&(ze.setValue(W,"toneMappingExposure",v.toneMappingExposure),$t.needsLights&&st(yi,Hi),at&&q.fog===!0&&lt.refreshFogUniforms(yi,at),lt.refreshMaterialUniforms(yi,q,G,B,rt),Go.upload(W,Un($t),yi,I)),q.isShaderMaterial&&q.uniformsNeedUpdate===!0&&(Go.upload(W,Un($t),yi,I),q.uniformsNeedUpdate=!1),q.isSpriteMaterial&&ze.setValue(W,"center",Y.center),ze.setValue(W,"modelViewMatrix",Y.modelViewMatrix),ze.setValue(W,"normalMatrix",Y.normalMatrix),ze.setValue(W,"modelMatrix",Y.matrixWorld),q.isShaderMaterial||q.isRawShaderMaterial){const Mn=q.uniformsGroups;for(let O=0,mt=Mn.length;O<mt;O++)if(Pt.isWebGL2){const Ut=Mn[O];te.update(Ut,Mi),te.bind(Ut,Mi)}else console.warn("THREE.WebGLRenderer: Uniform Buffer Objects can only be used with WebGL 2.")}return Mi}function st(P,k){P.ambientLightColor.needsUpdate=k,P.lightProbe.needsUpdate=k,P.directionalLights.needsUpdate=k,P.directionalLightShadows.needsUpdate=k,P.pointLights.needsUpdate=k,P.pointLightShadows.needsUpdate=k,P.spotLights.needsUpdate=k,P.spotLightShadows.needsUpdate=k,P.rectAreaLights.needsUpdate=k,P.hemisphereLights.needsUpdate=k}function Xt(P){return P.isMeshLambertMaterial||P.isMeshToonMaterial||P.isMeshPhongMaterial||P.isMeshStandardMaterial||P.isShadowMaterial||P.isShaderMaterial&&P.lights===!0}this.getActiveCubeFace=function(){return M},this.getActiveMipmapLevel=function(){return y},this.getRenderTarget=function(){return w},this.setRenderTargetTextures=function(P,k,Z){bt.get(P.texture).__webglTexture=k,bt.get(P.depthTexture).__webglTexture=Z;const q=bt.get(P);q.__hasExternalTextures=!0,q.__hasExternalTextures&&(q.__autoAllocateDepthBuffer=Z===void 0,q.__autoAllocateDepthBuffer||Tt.has("WEBGL_multisampled_render_to_texture")===!0&&(console.warn("THREE.WebGLRenderer: Render-to-texture extension was disabled because an external texture was provided"),q.__useRenderToTexture=!1))},this.setRenderTargetFramebuffer=function(P,k){const Z=bt.get(P);Z.__webglFramebuffer=k,Z.__useDefaultFramebuffer=k===void 0},this.setRenderTarget=function(P,k=0,Z=0){w=P,M=k,y=Z;let q=!0,Y=null,at=!1,Et=!1;if(P){const xt=bt.get(P);xt.__useDefaultFramebuffer!==void 0?(yt.bindFramebuffer(W.FRAMEBUFFER,null),q=!1):xt.__webglFramebuffer===void 0?I.setupRenderTarget(P):xt.__hasExternalTextures&&I.rebindTextures(P,bt.get(P.texture).__webglTexture,bt.get(P.depthTexture).__webglTexture);const Vt=P.texture;(Vt.isData3DTexture||Vt.isDataArrayTexture||Vt.isCompressedArrayTexture)&&(Et=!0);const kt=bt.get(P).__webglFramebuffer;P.isWebGLCubeRenderTarget?(Array.isArray(kt[k])?Y=kt[k][Z]:Y=kt[k],at=!0):Pt.isWebGL2&&P.samples>0&&I.useMultisampledRTT(P)===!1?Y=bt.get(P).__webglMultisampledFramebuffer:Array.isArray(kt)?Y=kt[Z]:Y=kt,T.copy(P.viewport),L.copy(P.scissor),C=P.scissorTest}else T.copy(j).multiplyScalar(G).floor(),L.copy($).multiplyScalar(G).floor(),C=Q;if(yt.bindFramebuffer(W.FRAMEBUFFER,Y)&&Pt.drawBuffers&&q&&yt.drawBuffers(P,Y),yt.viewport(T),yt.scissor(L),yt.setScissorTest(C),at){const xt=bt.get(P.texture);W.framebufferTexture2D(W.FRAMEBUFFER,W.COLOR_ATTACHMENT0,W.TEXTURE_CUBE_MAP_POSITIVE_X+k,xt.__webglTexture,Z)}else if(Et){const xt=bt.get(P.texture),Vt=k||0;W.framebufferTextureLayer(W.FRAMEBUFFER,W.COLOR_ATTACHMENT0,xt.__webglTexture,Z||0,Vt)}A=-1},this.readRenderTargetPixels=function(P,k,Z,q,Y,at,Et){if(!(P&&P.isWebGLRenderTarget)){console.error("THREE.WebGLRenderer.readRenderTargetPixels: renderTarget is not THREE.WebGLRenderTarget.");return}let Dt=bt.get(P).__webglFramebuffer;if(P.isWebGLCubeRenderTarget&&Et!==void 0&&(Dt=Dt[Et]),Dt){yt.bindFramebuffer(W.FRAMEBUFFER,Dt);try{const xt=P.texture,Vt=xt.format,kt=xt.type;if(Vt!==1023&&vt.convert(Vt)!==W.getParameter(W.IMPLEMENTATION_COLOR_READ_FORMAT)){console.error("THREE.WebGLRenderer.readRenderTargetPixels: renderTarget is not in RGBA or implementation defined format.");return}const Ot=kt===1016&&(Tt.has("EXT_color_buffer_half_float")||Pt.isWebGL2&&Tt.has("EXT_color_buffer_float"));if(kt!==1009&&vt.convert(kt)!==W.getParameter(W.IMPLEMENTATION_COLOR_READ_TYPE)&&!(kt===1015&&(Pt.isWebGL2||Tt.has("OES_texture_float")||Tt.has("WEBGL_color_buffer_float")))&&!Ot){console.error("THREE.WebGLRenderer.readRenderTargetPixels: renderTarget is not in UnsignedByteType or implementation defined type.");return}k>=0&&k<=P.width-q&&Z>=0&&Z<=P.height-Y&&W.readPixels(k,Z,q,Y,vt.convert(Vt),vt.convert(kt),at)}finally{const xt=w!==null?bt.get(w).__webglFramebuffer:null;yt.bindFramebuffer(W.FRAMEBUFFER,xt)}}},this.copyFramebufferToTexture=function(P,k,Z=0){const q=Math.pow(2,-Z),Y=Math.floor(k.image.width*q),at=Math.floor(k.image.height*q);I.setTexture2D(k,0),W.copyTexSubImage2D(W.TEXTURE_2D,Z,0,0,P.x,P.y,Y,at),yt.unbindTexture()},this.copyTextureToTexture=function(P,k,Z,q=0){const Y=k.image.width,at=k.image.height,Et=vt.convert(Z.format),Dt=vt.convert(Z.type);I.setTexture2D(Z,0),W.pixelStorei(W.UNPACK_FLIP_Y_WEBGL,Z.flipY),W.pixelStorei(W.UNPACK_PREMULTIPLY_ALPHA_WEBGL,Z.premultiplyAlpha),W.pixelStorei(W.UNPACK_ALIGNMENT,Z.unpackAlignment),k.isDataTexture?W.texSubImage2D(W.TEXTURE_2D,q,P.x,P.y,Y,at,Et,Dt,k.image.data):k.isCompressedTexture?W.compressedTexSubImage2D(W.TEXTURE_2D,q,P.x,P.y,k.mipmaps[0].width,k.mipmaps[0].height,Et,k.mipmaps[0].data):W.texSubImage2D(W.TEXTURE_2D,q,P.x,P.y,Et,Dt,k.image),q===0&&Z.generateMipmaps&&W.generateMipmap(W.TEXTURE_2D),yt.unbindTexture()},this.copyTextureToTexture3D=function(P,k,Z,q,Y=0){if(v.isWebGL1Renderer){console.warn("THREE.WebGLRenderer.copyTextureToTexture3D: can only be used with WebGL2.");return}const at=P.max.x-P.min.x+1,Et=P.max.y-P.min.y+1,Dt=P.max.z-P.min.z+1,xt=vt.convert(q.format),Vt=vt.convert(q.type);let kt;if(q.isData3DTexture)I.setTexture3D(q,0),kt=W.TEXTURE_3D;else if(q.isDataArrayTexture||q.isCompressedArrayTexture)I.setTexture2DArray(q,0),kt=W.TEXTURE_2D_ARRAY;else{console.warn("THREE.WebGLRenderer.copyTextureToTexture3D: only supports THREE.DataTexture3D and THREE.DataTexture2DArray.");return}W.pixelStorei(W.UNPACK_FLIP_Y_WEBGL,q.flipY),W.pixelStorei(W.UNPACK_PREMULTIPLY_ALPHA_WEBGL,q.premultiplyAlpha),W.pixelStorei(W.UNPACK_ALIGNMENT,q.unpackAlignment);const Ot=W.getParameter(W.UNPACK_ROW_LENGTH),fe=W.getParameter(W.UNPACK_IMAGE_HEIGHT),Je=W.getParameter(W.UNPACK_SKIP_PIXELS),Re=W.getParameter(W.UNPACK_SKIP_ROWS),Zn=W.getParameter(W.UNPACK_SKIP_IMAGES),Se=Z.isCompressedTexture?Z.mipmaps[Y]:Z.image;W.pixelStorei(W.UNPACK_ROW_LENGTH,Se.width),W.pixelStorei(W.UNPACK_IMAGE_HEIGHT,Se.height),W.pixelStorei(W.UNPACK_SKIP_PIXELS,P.min.x),W.pixelStorei(W.UNPACK_SKIP_ROWS,P.min.y),W.pixelStorei(W.UNPACK_SKIP_IMAGES,P.min.z),Z.isDataTexture||Z.isData3DTexture?W.texSubImage3D(kt,Y,k.x,k.y,k.z,at,Et,Dt,xt,Vt,Se.data):Z.isCompressedArrayTexture?(console.warn("THREE.WebGLRenderer.copyTextureToTexture3D: untested support for compressed srcTexture."),W.compressedTexSubImage3D(kt,Y,k.x,k.y,k.z,at,Et,Dt,xt,Se.data)):W.texSubImage3D(kt,Y,k.x,k.y,k.z,at,Et,Dt,xt,Vt,Se),W.pixelStorei(W.UNPACK_ROW_LENGTH,Ot),W.pixelStorei(W.UNPACK_IMAGE_HEIGHT,fe),W.pixelStorei(W.UNPACK_SKIP_PIXELS,Je),W.pixelStorei(W.UNPACK_SKIP_ROWS,Re),W.pixelStorei(W.UNPACK_SKIP_IMAGES,Zn),Y===0&&q.generateMipmaps&&W.generateMipmap(kt),yt.unbindTexture()},this.initTexture=function(P){P.isCubeTexture?I.setTextureCube(P,0):P.isData3DTexture?I.setTexture3D(P,0):P.isDataArrayTexture||P.isCompressedArrayTexture?I.setTexture2DArray(P,0):I.setTexture2D(P,0),yt.unbindTexture()},this.resetState=function(){M=0,y=0,w=null,yt.reset(),Ft.reset()},typeof __THREE_DEVTOOLS__<"u"&&__THREE_DEVTOOLS__.dispatchEvent(new CustomEvent("observe",{detail:this}))}get coordinateSystem(){return 2e3}get outputColorSpace(){return this._outputColorSpace}set outputColorSpace(t){this._outputColorSpace=t;const e=this.getContext();e.drawingBufferColorSpace=t===Ih?"display-p3":"srgb",e.unpackColorSpace=pe.workingColorSpace===fc?"display-p3":"srgb"}get outputEncoding(){return console.warn("THREE.WebGLRenderer: Property .outputEncoding has been removed. Use .outputColorSpace instead."),this.outputColorSpace===an?3001:3e3}set outputEncoding(t){console.warn("THREE.WebGLRenderer: Property .outputEncoding has been removed. Use .outputColorSpace instead."),this.outputColorSpace=t===3001?an:Ui}get useLegacyLights(){return console.warn("THREE.WebGLRenderer: The property .useLegacyLights has been deprecated. Migrate your lighting according to the following guide: https://discourse.threejs.org/t/updates-to-lighting-in-three-js-r155/53733."),this._useLegacyLights}set useLegacyLights(t){console.warn("THREE.WebGLRenderer: The property .useLegacyLights has been deprecated. Migrate your lighting according to the following guide: https://discourse.threejs.org/t/updates-to-lighting-in-three-js-r155/53733."),this._useLegacyLights=t}}class Pv extends Oh{}Pv.prototype.isWebGL1Renderer=!0;class Rr extends nn{constructor(){super(),this.isScene=!0,this.type="Scene",this.background=null,this.environment=null,this.fog=null,this.backgroundBlurriness=0,this.backgroundIntensity=1,this.overrideMaterial=null,typeof __THREE_DEVTOOLS__<"u"&&__THREE_DEVTOOLS__.dispatchEvent(new CustomEvent("observe",{detail:this}))}copy(t,e){return super.copy(t,e),t.background!==null&&(this.background=t.background.clone()),t.environment!==null&&(this.environment=t.environment.clone()),t.fog!==null&&(this.fog=t.fog.clone()),this.backgroundBlurriness=t.backgroundBlurriness,this.backgroundIntensity=t.backgroundIntensity,t.overrideMaterial!==null&&(this.overrideMaterial=t.overrideMaterial.clone()),this.matrixAutoUpdate=t.matrixAutoUpdate,this}toJSON(t){const e=super.toJSON(t);return this.fog!==null&&(e.object.fog=this.fog.toJSON()),this.backgroundBlurriness>0&&(e.object.backgroundBlurriness=this.backgroundBlurriness),this.backgroundIntensity!==1&&(e.object.backgroundIntensity=this.backgroundIntensity),e}}class qu extends Be{constructor(t,e,n,i=1){super(t,e,n),this.isInstancedBufferAttribute=!0,this.meshPerAttribute=i}copy(t){return super.copy(t),this.meshPerAttribute=t.meshPerAttribute,this}toJSON(){const t=super.toJSON();return t.meshPerAttribute=this.meshPerAttribute,t.isInstancedBufferAttribute=!0,t}}const Zs=new re,Zu=new re,ro=[],ju=new rn,Lv=new re,Br=new jt,Ur=new as;class gc extends jt{constructor(t,e,n){super(t,e),this.isInstancedMesh=!0,this.instanceMatrix=new qu(new Float32Array(n*16),16),this.instanceColor=null,this.count=n,this.boundingBox=null,this.boundingSphere=null;for(let i=0;i<n;i++)this.setMatrixAt(i,Lv)}computeBoundingBox(){const t=this.geometry,e=this.count;this.boundingBox===null&&(this.boundingBox=new rn),t.boundingBox===null&&t.computeBoundingBox(),this.boundingBox.makeEmpty();for(let n=0;n<e;n++)this.getMatrixAt(n,Zs),ju.copy(t.boundingBox).applyMatrix4(Zs),this.boundingBox.union(ju)}computeBoundingSphere(){const t=this.geometry,e=this.count;this.boundingSphere===null&&(this.boundingSphere=new as),t.boundingSphere===null&&t.computeBoundingSphere(),this.boundingSphere.makeEmpty();for(let n=0;n<e;n++)this.getMatrixAt(n,Zs),Ur.copy(t.boundingSphere).applyMatrix4(Zs),this.boundingSphere.union(Ur)}copy(t,e){return super.copy(t,e),this.instanceMatrix.copy(t.instanceMatrix),t.instanceColor!==null&&(this.instanceColor=t.instanceColor.clone()),this.count=t.count,t.boundingBox!==null&&(this.boundingBox=t.boundingBox.clone()),t.boundingSphere!==null&&(this.boundingSphere=t.boundingSphere.clone()),this}getColorAt(t,e){e.fromArray(this.instanceColor.array,t*3)}getMatrixAt(t,e){e.fromArray(this.instanceMatrix.array,t*16)}raycast(t,e){const n=this.matrixWorld,i=this.count;if(Br.geometry=this.geometry,Br.material=this.material,Br.material!==void 0&&(this.boundingSphere===null&&this.computeBoundingSphere(),Ur.copy(this.boundingSphere),Ur.applyMatrix4(n),t.ray.intersectsSphere(Ur)!==!1))for(let s=0;s<i;s++){this.getMatrixAt(s,Zs),Zu.multiplyMatrices(n,Zs),Br.matrixWorld=Zu,Br.raycast(t,ro);for(let a=0,o=ro.length;a<o;a++){const c=ro[a];c.instanceId=s,c.object=this,e.push(c)}ro.length=0}}setColorAt(t,e){this.instanceColor===null&&(this.instanceColor=new qu(new Float32Array(this.instanceMatrix.count*3),3)),e.toArray(this.instanceColor.array,t*3)}setMatrixAt(t,e){e.toArray(this.instanceMatrix.array,t*16)}updateMorphTargets(){}dispose(){this.dispatchEvent({type:"dispose"})}}class gi extends _i{constructor(t){super(),this.isLineBasicMaterial=!0,this.type="LineBasicMaterial",this.color=new Qt(16777215),this.map=null,this.linewidth=1,this.linecap="round",this.linejoin="round",this.fog=!0,this.setValues(t)}copy(t){return super.copy(t),this.color.copy(t.color),this.map=t.map,this.linewidth=t.linewidth,this.linecap=t.linecap,this.linejoin=t.linejoin,this.fog=t.fog,this}}const $u=new R,Ku=new R,Ju=new re,Jc=new Fh,ao=new as;class Dv extends nn{constructor(t=new me,e=new gi){super(),this.isLine=!0,this.type="Line",this.geometry=t,this.material=e,this.updateMorphTargets()}copy(t,e){return super.copy(t,e),this.material=Array.isArray(t.material)?t.material.slice():t.material,this.geometry=t.geometry,this}computeLineDistances(){const t=this.geometry;if(t.index===null){const e=t.attributes.position,n=[0];for(let i=1,s=e.count;i<s;i++)$u.fromBufferAttribute(e,i-1),Ku.fromBufferAttribute(e,i),n[i]=n[i-1],n[i]+=$u.distanceTo(Ku);t.setAttribute("lineDistance",new ie(n,1))}else console.warn("THREE.Line.computeLineDistances(): Computation only possible with non-indexed BufferGeometry.");return this}raycast(t,e){const n=this.geometry,i=this.matrixWorld,s=t.params.Line.threshold,a=n.drawRange;if(n.boundingSphere===null&&n.computeBoundingSphere(),ao.copy(n.boundingSphere),ao.applyMatrix4(i),ao.radius+=s,t.ray.intersectsSphere(ao)===!1)return;Ju.copy(i).invert(),Jc.copy(t.ray).applyMatrix4(Ju);const o=s/((this.scale.x+this.scale.y+this.scale.z)/3),c=o*o,l=new R,h=new R,u=new R,d=new R,f=this.isLineSegments?2:1,g=n.index,m=n.attributes.position;if(g!==null){const p=Math.max(0,a.start),_=Math.min(g.count,a.start+a.count);for(let v=p,S=_-1;v<S;v+=f){const M=g.getX(v),y=g.getX(v+1);if(l.fromBufferAttribute(m,M),h.fromBufferAttribute(m,y),Jc.distanceSqToSegment(l,h,d,u)>c)continue;d.applyMatrix4(this.matrixWorld);const A=t.ray.origin.distanceTo(d);A<t.near||A>t.far||e.push({distance:A,point:u.clone().applyMatrix4(this.matrixWorld),index:v,face:null,faceIndex:null,object:this})}}else{const p=Math.max(0,a.start),_=Math.min(m.count,a.start+a.count);for(let v=p,S=_-1;v<S;v+=f){if(l.fromBufferAttribute(m,v),h.fromBufferAttribute(m,v+1),Jc.distanceSqToSegment(l,h,d,u)>c)continue;d.applyMatrix4(this.matrixWorld);const y=t.ray.origin.distanceTo(d);y<t.near||y>t.far||e.push({distance:y,point:u.clone().applyMatrix4(this.matrixWorld),index:v,face:null,faceIndex:null,object:this})}}}updateMorphTargets(){const e=this.geometry.morphAttributes,n=Object.keys(e);if(n.length>0){const i=e[n[0]];if(i!==void 0){this.morphTargetInfluences=[],this.morphTargetDictionary={};for(let s=0,a=i.length;s<a;s++){const o=i[s].name||String(s);this.morphTargetInfluences.push(0),this.morphTargetDictionary[o]=s}}}}}const Qu=new R,td=new R;class ss extends Dv{constructor(t,e){super(t,e),this.isLineSegments=!0,this.type="LineSegments"}computeLineDistances(){const t=this.geometry;if(t.index===null){const e=t.attributes.position,n=[];for(let i=0,s=e.count;i<s;i+=2)Qu.fromBufferAttribute(e,i),td.fromBufferAttribute(e,i+1),n[i]=i===0?0:n[i-1],n[i+1]=n[i]+Qu.distanceTo(td);t.setAttribute("lineDistance",new ie(n,1))}else console.warn("THREE.LineSegments.computeLineDistances(): Computation only possible with non-indexed BufferGeometry.");return this}}class ua extends _i{constructor(t){super(),this.isPointsMaterial=!0,this.type="PointsMaterial",this.color=new Qt(16777215),this.map=null,this.alphaMap=null,this.size=1,this.sizeAttenuation=!0,this.fog=!0,this.setValues(t)}copy(t){return super.copy(t),this.color.copy(t.color),this.map=t.map,this.alphaMap=t.alphaMap,this.size=t.size,this.sizeAttenuation=t.sizeAttenuation,this.fog=t.fog,this}}const ed=new re,nh=new Fh,oo=new as,co=new R;class Qc extends nn{constructor(t=new me,e=new ua){super(),this.isPoints=!0,this.type="Points",this.geometry=t,this.material=e,this.updateMorphTargets()}copy(t,e){return super.copy(t,e),this.material=Array.isArray(t.material)?t.material.slice():t.material,this.geometry=t.geometry,this}raycast(t,e){const n=this.geometry,i=this.matrixWorld,s=t.params.Points.threshold,a=n.drawRange;if(n.boundingSphere===null&&n.computeBoundingSphere(),oo.copy(n.boundingSphere),oo.applyMatrix4(i),oo.radius+=s,t.ray.intersectsSphere(oo)===!1)return;ed.copy(i).invert(),nh.copy(t.ray).applyMatrix4(ed);const o=s/((this.scale.x+this.scale.y+this.scale.z)/3),c=o*o,l=n.index,u=n.attributes.position;if(l!==null){const d=Math.max(0,a.start),f=Math.min(l.count,a.start+a.count);for(let g=d,x=f;g<x;g++){const m=l.getX(g);co.fromBufferAttribute(u,m),nd(co,m,c,i,t,e,this)}}else{const d=Math.max(0,a.start),f=Math.min(u.count,a.start+a.count);for(let g=d,x=f;g<x;g++)co.fromBufferAttribute(u,g),nd(co,g,c,i,t,e,this)}}updateMorphTargets(){const e=this.geometry.morphAttributes,n=Object.keys(e);if(n.length>0){const i=e[n[0]];if(i!==void 0){this.morphTargetInfluences=[],this.morphTargetDictionary={};for(let s=0,a=i.length;s<a;s++){const o=i[s].name||String(s);this.morphTargetInfluences.push(0),this.morphTargetDictionary[o]=s}}}}}function nd(r,t,e,n,i,s,a){const o=nh.distanceSqToPoint(r);if(o<e){const c=new R;nh.closestPointToPoint(r,c),c.applyMatrix4(n);const l=i.ray.origin.distanceTo(c);if(l<i.near||l>i.far)return;s.push({distance:l,distanceToRay:Math.sqrt(o),point:c,index:t,face:null,object:a})}}class vi{constructor(){this.type="Curve",this.arcLengthDivisions=200}getPoint(){return console.warn("THREE.Curve: .getPoint() not implemented."),null}getPointAt(t,e){const n=this.getUtoTmapping(t);return this.getPoint(n,e)}getPoints(t=5){const e=[];for(let n=0;n<=t;n++)e.push(this.getPoint(n/t));return e}getSpacedPoints(t=5){const e=[];for(let n=0;n<=t;n++)e.push(this.getPointAt(n/t));return e}getLength(){const t=this.getLengths();return t[t.length-1]}getLengths(t=this.arcLengthDivisions){if(this.cacheArcLengths&&this.cacheArcLengths.length===t+1&&!this.needsUpdate)return this.cacheArcLengths;this.needsUpdate=!1;const e=[];let n,i=this.getPoint(0),s=0;e.push(0);for(let a=1;a<=t;a++)n=this.getPoint(a/t),s+=n.distanceTo(i),e.push(s),i=n;return this.cacheArcLengths=e,e}updateArcLengths(){this.needsUpdate=!0,this.getLengths()}getUtoTmapping(t,e){const n=this.getLengths();let i=0;const s=n.length;let a;e?a=e:a=t*n[s-1];let o=0,c=s-1,l;for(;o<=c;)if(i=Math.floor(o+(c-o)/2),l=n[i]-a,l<0)o=i+1;else if(l>0)c=i-1;else{c=i;break}if(i=c,n[i]===a)return i/(s-1);const h=n[i],d=n[i+1]-h,f=(a-h)/d;return(i+f)/(s-1)}getTangent(t,e){let i=t-1e-4,s=t+1e-4;i<0&&(i=0),s>1&&(s=1);const a=this.getPoint(i),o=this.getPoint(s),c=e||(a.isVector2?new Mt:new R);return c.copy(o).sub(a).normalize(),c}getTangentAt(t,e){const n=this.getUtoTmapping(t);return this.getTangent(n,e)}computeFrenetFrames(t,e){const n=new R,i=[],s=[],a=[],o=new R,c=new re;for(let f=0;f<=t;f++){const g=f/t;i[f]=this.getTangentAt(g,new R)}s[0]=new R,a[0]=new R;let l=Number.MAX_VALUE;const h=Math.abs(i[0].x),u=Math.abs(i[0].y),d=Math.abs(i[0].z);h<=l&&(l=h,n.set(1,0,0)),u<=l&&(l=u,n.set(0,1,0)),d<=l&&n.set(0,0,1),o.crossVectors(i[0],n).normalize(),s[0].crossVectors(i[0],o),a[0].crossVectors(i[0],s[0]);for(let f=1;f<=t;f++){if(s[f]=s[f-1].clone(),a[f]=a[f-1].clone(),o.crossVectors(i[f-1],i[f]),o.length()>Number.EPSILON){o.normalize();const g=Math.acos(We(i[f-1].dot(i[f]),-1,1));s[f].applyMatrix4(c.makeRotationAxis(o,g))}a[f].crossVectors(i[f],s[f])}if(e===!0){let f=Math.acos(We(s[0].dot(s[t]),-1,1));f/=t,i[0].dot(o.crossVectors(s[0],s[t]))>0&&(f=-f);for(let g=1;g<=t;g++)s[g].applyMatrix4(c.makeRotationAxis(i[g],f*g)),a[g].crossVectors(i[g],s[g])}return{tangents:i,normals:s,binormals:a}}clone(){return new this.constructor().copy(this)}copy(t){return this.arcLengthDivisions=t.arcLengthDivisions,this}toJSON(){const t={metadata:{version:4.6,type:"Curve",generator:"Curve.toJSON"}};return t.arcLengthDivisions=this.arcLengthDivisions,t.type=this.type,t}fromJSON(t){return this.arcLengthDivisions=t.arcLengthDivisions,this}}class Gh extends vi{constructor(t=0,e=0,n=1,i=1,s=0,a=Math.PI*2,o=!1,c=0){super(),this.isEllipseCurve=!0,this.type="EllipseCurve",this.aX=t,this.aY=e,this.xRadius=n,this.yRadius=i,this.aStartAngle=s,this.aEndAngle=a,this.aClockwise=o,this.aRotation=c}getPoint(t,e){const n=e||new Mt,i=Math.PI*2;let s=this.aEndAngle-this.aStartAngle;const a=Math.abs(s)<Number.EPSILON;for(;s<0;)s+=i;for(;s>i;)s-=i;s<Number.EPSILON&&(a?s=0:s=i),this.aClockwise===!0&&!a&&(s===i?s=-i:s=s-i);const o=this.aStartAngle+t*s;let c=this.aX+this.xRadius*Math.cos(o),l=this.aY+this.yRadius*Math.sin(o);if(this.aRotation!==0){const h=Math.cos(this.aRotation),u=Math.sin(this.aRotation),d=c-this.aX,f=l-this.aY;c=d*h-f*u+this.aX,l=d*u+f*h+this.aY}return n.set(c,l)}copy(t){return super.copy(t),this.aX=t.aX,this.aY=t.aY,this.xRadius=t.xRadius,this.yRadius=t.yRadius,this.aStartAngle=t.aStartAngle,this.aEndAngle=t.aEndAngle,this.aClockwise=t.aClockwise,this.aRotation=t.aRotation,this}toJSON(){const t=super.toJSON();return t.aX=this.aX,t.aY=this.aY,t.xRadius=this.xRadius,t.yRadius=this.yRadius,t.aStartAngle=this.aStartAngle,t.aEndAngle=this.aEndAngle,t.aClockwise=this.aClockwise,t.aRotation=this.aRotation,t}fromJSON(t){return super.fromJSON(t),this.aX=t.aX,this.aY=t.aY,this.xRadius=t.xRadius,this.yRadius=t.yRadius,this.aStartAngle=t.aStartAngle,this.aEndAngle=t.aEndAngle,this.aClockwise=t.aClockwise,this.aRotation=t.aRotation,this}}class Iv extends Gh{constructor(t,e,n,i,s,a){super(t,e,n,n,i,s,a),this.isArcCurve=!0,this.type="ArcCurve"}}function Vh(){let r=0,t=0,e=0,n=0;function i(s,a,o,c){r=s,t=o,e=-3*s+3*a-2*o-c,n=2*s-2*a+o+c}return{initCatmullRom:function(s,a,o,c,l){i(a,o,l*(o-s),l*(c-a))},initNonuniformCatmullRom:function(s,a,o,c,l,h,u){let d=(a-s)/l-(o-s)/(l+h)+(o-a)/h,f=(o-a)/h-(c-a)/(h+u)+(c-o)/u;d*=h,f*=h,i(a,o,d,f)},calc:function(s){const a=s*s,o=a*s;return r+t*s+e*a+n*o}}}const lo=new R,tl=new Vh,el=new Vh,nl=new Vh;class kh extends vi{constructor(t=[],e=!1,n="centripetal",i=.5){super(),this.isCatmullRomCurve3=!0,this.type="CatmullRomCurve3",this.points=t,this.closed=e,this.curveType=n,this.tension=i}getPoint(t,e=new R){const n=e,i=this.points,s=i.length,a=(s-(this.closed?0:1))*t;let o=Math.floor(a),c=a-o;this.closed?o+=o>0?0:(Math.floor(Math.abs(o)/s)+1)*s:c===0&&o===s-1&&(o=s-2,c=1);let l,h;this.closed||o>0?l=i[(o-1)%s]:(lo.subVectors(i[0],i[1]).add(i[0]),l=lo);const u=i[o%s],d=i[(o+1)%s];if(this.closed||o+2<s?h=i[(o+2)%s]:(lo.subVectors(i[s-1],i[s-2]).add(i[s-1]),h=lo),this.curveType==="centripetal"||this.curveType==="chordal"){const f=this.curveType==="chordal"?.5:.25;let g=Math.pow(l.distanceToSquared(u),f),x=Math.pow(u.distanceToSquared(d),f),m=Math.pow(d.distanceToSquared(h),f);x<1e-4&&(x=1),g<1e-4&&(g=x),m<1e-4&&(m=x),tl.initNonuniformCatmullRom(l.x,u.x,d.x,h.x,g,x,m),el.initNonuniformCatmullRom(l.y,u.y,d.y,h.y,g,x,m),nl.initNonuniformCatmullRom(l.z,u.z,d.z,h.z,g,x,m)}else this.curveType==="catmullrom"&&(tl.initCatmullRom(l.x,u.x,d.x,h.x,this.tension),el.initCatmullRom(l.y,u.y,d.y,h.y,this.tension),nl.initCatmullRom(l.z,u.z,d.z,h.z,this.tension));return n.set(tl.calc(c),el.calc(c),nl.calc(c)),n}copy(t){super.copy(t),this.points=[];for(let e=0,n=t.points.length;e<n;e++){const i=t.points[e];this.points.push(i.clone())}return this.closed=t.closed,this.curveType=t.curveType,this.tension=t.tension,this}toJSON(){const t=super.toJSON();t.points=[];for(let e=0,n=this.points.length;e<n;e++){const i=this.points[e];t.points.push(i.toArray())}return t.closed=this.closed,t.curveType=this.curveType,t.tension=this.tension,t}fromJSON(t){super.fromJSON(t),this.points=[];for(let e=0,n=t.points.length;e<n;e++){const i=t.points[e];this.points.push(new R().fromArray(i))}return this.closed=t.closed,this.curveType=t.curveType,this.tension=t.tension,this}}function id(r,t,e,n,i){const s=(n-t)*.5,a=(i-e)*.5,o=r*r,c=r*o;return(2*e-2*n+s+a)*c+(-3*e+3*n-2*s-a)*o+s*r+e}function Nv(r,t){const e=1-r;return e*e*t}function Fv(r,t){return 2*(1-r)*r*t}function Bv(r,t){return r*r*t}function ya(r,t,e,n){return Nv(r,t)+Fv(r,e)+Bv(r,n)}function Uv(r,t){const e=1-r;return e*e*e*t}function zv(r,t){const e=1-r;return 3*e*e*r*t}function Ov(r,t){return 3*(1-r)*r*r*t}function Gv(r,t){return r*r*r*t}function Ea(r,t,e,n,i){return Uv(r,t)+zv(r,e)+Ov(r,n)+Gv(r,i)}class tp extends vi{constructor(t=new Mt,e=new Mt,n=new Mt,i=new Mt){super(),this.isCubicBezierCurve=!0,this.type="CubicBezierCurve",this.v0=t,this.v1=e,this.v2=n,this.v3=i}getPoint(t,e=new Mt){const n=e,i=this.v0,s=this.v1,a=this.v2,o=this.v3;return n.set(Ea(t,i.x,s.x,a.x,o.x),Ea(t,i.y,s.y,a.y,o.y)),n}copy(t){return super.copy(t),this.v0.copy(t.v0),this.v1.copy(t.v1),this.v2.copy(t.v2),this.v3.copy(t.v3),this}toJSON(){const t=super.toJSON();return t.v0=this.v0.toArray(),t.v1=this.v1.toArray(),t.v2=this.v2.toArray(),t.v3=this.v3.toArray(),t}fromJSON(t){return super.fromJSON(t),this.v0.fromArray(t.v0),this.v1.fromArray(t.v1),this.v2.fromArray(t.v2),this.v3.fromArray(t.v3),this}}class Vv extends vi{constructor(t=new R,e=new R,n=new R,i=new R){super(),this.isCubicBezierCurve3=!0,this.type="CubicBezierCurve3",this.v0=t,this.v1=e,this.v2=n,this.v3=i}getPoint(t,e=new R){const n=e,i=this.v0,s=this.v1,a=this.v2,o=this.v3;return n.set(Ea(t,i.x,s.x,a.x,o.x),Ea(t,i.y,s.y,a.y,o.y),Ea(t,i.z,s.z,a.z,o.z)),n}copy(t){return super.copy(t),this.v0.copy(t.v0),this.v1.copy(t.v1),this.v2.copy(t.v2),this.v3.copy(t.v3),this}toJSON(){const t=super.toJSON();return t.v0=this.v0.toArray(),t.v1=this.v1.toArray(),t.v2=this.v2.toArray(),t.v3=this.v3.toArray(),t}fromJSON(t){return super.fromJSON(t),this.v0.fromArray(t.v0),this.v1.fromArray(t.v1),this.v2.fromArray(t.v2),this.v3.fromArray(t.v3),this}}class ep extends vi{constructor(t=new Mt,e=new Mt){super(),this.isLineCurve=!0,this.type="LineCurve",this.v1=t,this.v2=e}getPoint(t,e=new Mt){const n=e;return t===1?n.copy(this.v2):(n.copy(this.v2).sub(this.v1),n.multiplyScalar(t).add(this.v1)),n}getPointAt(t,e){return this.getPoint(t,e)}getTangent(t,e=new Mt){return e.subVectors(this.v2,this.v1).normalize()}getTangentAt(t,e){return this.getTangent(t,e)}copy(t){return super.copy(t),this.v1.copy(t.v1),this.v2.copy(t.v2),this}toJSON(){const t=super.toJSON();return t.v1=this.v1.toArray(),t.v2=this.v2.toArray(),t}fromJSON(t){return super.fromJSON(t),this.v1.fromArray(t.v1),this.v2.fromArray(t.v2),this}}class np extends vi{constructor(t=new R,e=new R){super(),this.isLineCurve3=!0,this.type="LineCurve3",this.v1=t,this.v2=e}getPoint(t,e=new R){const n=e;return t===1?n.copy(this.v2):(n.copy(this.v2).sub(this.v1),n.multiplyScalar(t).add(this.v1)),n}getPointAt(t,e){return this.getPoint(t,e)}getTangent(t,e=new R){return e.subVectors(this.v2,this.v1).normalize()}getTangentAt(t,e){return this.getTangent(t,e)}copy(t){return super.copy(t),this.v1.copy(t.v1),this.v2.copy(t.v2),this}toJSON(){const t=super.toJSON();return t.v1=this.v1.toArray(),t.v2=this.v2.toArray(),t}fromJSON(t){return super.fromJSON(t),this.v1.fromArray(t.v1),this.v2.fromArray(t.v2),this}}class ip extends vi{constructor(t=new Mt,e=new Mt,n=new Mt){super(),this.isQuadraticBezierCurve=!0,this.type="QuadraticBezierCurve",this.v0=t,this.v1=e,this.v2=n}getPoint(t,e=new Mt){const n=e,i=this.v0,s=this.v1,a=this.v2;return n.set(ya(t,i.x,s.x,a.x),ya(t,i.y,s.y,a.y)),n}copy(t){return super.copy(t),this.v0.copy(t.v0),this.v1.copy(t.v1),this.v2.copy(t.v2),this}toJSON(){const t=super.toJSON();return t.v0=this.v0.toArray(),t.v1=this.v1.toArray(),t.v2=this.v2.toArray(),t}fromJSON(t){return super.fromJSON(t),this.v0.fromArray(t.v0),this.v1.fromArray(t.v1),this.v2.fromArray(t.v2),this}}class sp extends vi{constructor(t=new R,e=new R,n=new R){super(),this.isQuadraticBezierCurve3=!0,this.type="QuadraticBezierCurve3",this.v0=t,this.v1=e,this.v2=n}getPoint(t,e=new R){const n=e,i=this.v0,s=this.v1,a=this.v2;return n.set(ya(t,i.x,s.x,a.x),ya(t,i.y,s.y,a.y),ya(t,i.z,s.z,a.z)),n}copy(t){return super.copy(t),this.v0.copy(t.v0),this.v1.copy(t.v1),this.v2.copy(t.v2),this}toJSON(){const t=super.toJSON();return t.v0=this.v0.toArray(),t.v1=this.v1.toArray(),t.v2=this.v2.toArray(),t}fromJSON(t){return super.fromJSON(t),this.v0.fromArray(t.v0),this.v1.fromArray(t.v1),this.v2.fromArray(t.v2),this}}class rp extends vi{constructor(t=[]){super(),this.isSplineCurve=!0,this.type="SplineCurve",this.points=t}getPoint(t,e=new Mt){const n=e,i=this.points,s=(i.length-1)*t,a=Math.floor(s),o=s-a,c=i[a===0?a:a-1],l=i[a],h=i[a>i.length-2?i.length-1:a+1],u=i[a>i.length-3?i.length-1:a+2];return n.set(id(o,c.x,l.x,h.x,u.x),id(o,c.y,l.y,h.y,u.y)),n}copy(t){super.copy(t),this.points=[];for(let e=0,n=t.points.length;e<n;e++){const i=t.points[e];this.points.push(i.clone())}return this}toJSON(){const t=super.toJSON();t.points=[];for(let e=0,n=this.points.length;e<n;e++){const i=this.points[e];t.points.push(i.toArray())}return t}fromJSON(t){super.fromJSON(t),this.points=[];for(let e=0,n=t.points.length;e<n;e++){const i=t.points[e];this.points.push(new Mt().fromArray(i))}return this}}var ih=Object.freeze({__proto__:null,ArcCurve:Iv,CatmullRomCurve3:kh,CubicBezierCurve:tp,CubicBezierCurve3:Vv,EllipseCurve:Gh,LineCurve:ep,LineCurve3:np,QuadraticBezierCurve:ip,QuadraticBezierCurve3:sp,SplineCurve:rp});class kv extends vi{constructor(){super(),this.type="CurvePath",this.curves=[],this.autoClose=!1}add(t){this.curves.push(t)}closePath(){const t=this.curves[0].getPoint(0),e=this.curves[this.curves.length-1].getPoint(1);if(!t.equals(e)){const n=t.isVector2===!0?"LineCurve":"LineCurve3";this.curves.push(new ih[n](e,t))}return this}getPoint(t,e){const n=t*this.getLength(),i=this.getCurveLengths();let s=0;for(;s<i.length;){if(i[s]>=n){const a=i[s]-n,o=this.curves[s],c=o.getLength(),l=c===0?0:1-a/c;return o.getPointAt(l,e)}s++}return null}getLength(){const t=this.getCurveLengths();return t[t.length-1]}updateArcLengths(){this.needsUpdate=!0,this.cacheLengths=null,this.getCurveLengths()}getCurveLengths(){if(this.cacheLengths&&this.cacheLengths.length===this.curves.length)return this.cacheLengths;const t=[];let e=0;for(let n=0,i=this.curves.length;n<i;n++)e+=this.curves[n].getLength(),t.push(e);return this.cacheLengths=t,t}getSpacedPoints(t=40){const e=[];for(let n=0;n<=t;n++)e.push(this.getPoint(n/t));return this.autoClose&&e.push(e[0]),e}getPoints(t=12){const e=[];let n;for(let i=0,s=this.curves;i<s.length;i++){const a=s[i],o=a.isEllipseCurve?t*2:a.isLineCurve||a.isLineCurve3?1:a.isSplineCurve?t*a.points.length:t,c=a.getPoints(o);for(let l=0;l<c.length;l++){const h=c[l];n&&n.equals(h)||(e.push(h),n=h)}}return this.autoClose&&e.length>1&&!e[e.length-1].equals(e[0])&&e.push(e[0]),e}copy(t){super.copy(t),this.curves=[];for(let e=0,n=t.curves.length;e<n;e++){const i=t.curves[e];this.curves.push(i.clone())}return this.autoClose=t.autoClose,this}toJSON(){const t=super.toJSON();t.autoClose=this.autoClose,t.curves=[];for(let e=0,n=this.curves.length;e<n;e++){const i=this.curves[e];t.curves.push(i.toJSON())}return t}fromJSON(t){super.fromJSON(t),this.autoClose=t.autoClose,this.curves=[];for(let e=0,n=t.curves.length;e<n;e++){const i=t.curves[e];this.curves.push(new ih[i.type]().fromJSON(i))}return this}}class Hv extends kv{constructor(t){super(),this.type="Path",this.currentPoint=new Mt,t&&this.setFromPoints(t)}setFromPoints(t){this.moveTo(t[0].x,t[0].y);for(let e=1,n=t.length;e<n;e++)this.lineTo(t[e].x,t[e].y);return this}moveTo(t,e){return this.currentPoint.set(t,e),this}lineTo(t,e){const n=new ep(this.currentPoint.clone(),new Mt(t,e));return this.curves.push(n),this.currentPoint.set(t,e),this}quadraticCurveTo(t,e,n,i){const s=new ip(this.currentPoint.clone(),new Mt(t,e),new Mt(n,i));return this.curves.push(s),this.currentPoint.set(n,i),this}bezierCurveTo(t,e,n,i,s,a){const o=new tp(this.currentPoint.clone(),new Mt(t,e),new Mt(n,i),new Mt(s,a));return this.curves.push(o),this.currentPoint.set(s,a),this}splineThru(t){const e=[this.currentPoint.clone()].concat(t),n=new rp(e);return this.curves.push(n),this.currentPoint.copy(t[t.length-1]),this}arc(t,e,n,i,s,a){const o=this.currentPoint.x,c=this.currentPoint.y;return this.absarc(t+o,e+c,n,i,s,a),this}absarc(t,e,n,i,s,a){return this.absellipse(t,e,n,n,i,s,a),this}ellipse(t,e,n,i,s,a,o,c){const l=this.currentPoint.x,h=this.currentPoint.y;return this.absellipse(t+l,e+h,n,i,s,a,o,c),this}absellipse(t,e,n,i,s,a,o,c){const l=new Gh(t,e,n,i,s,a,o,c);if(this.curves.length>0){const u=l.getPoint(0);u.equals(this.currentPoint)||this.lineTo(u.x,u.y)}this.curves.push(l);const h=l.getPoint(1);return this.currentPoint.copy(h),this}copy(t){return super.copy(t),this.currentPoint.copy(t.currentPoint),this}toJSON(){const t=super.toJSON();return t.currentPoint=this.currentPoint.toArray(),t}fromJSON(t){return super.fromJSON(t),this.currentPoint.fromArray(t.currentPoint),this}}class Hh extends me{constructor(t=[new Mt(0,-.5),new Mt(.5,0),new Mt(0,.5)],e=12,n=0,i=Math.PI*2){super(),this.type="LatheGeometry",this.parameters={points:t,segments:e,phiStart:n,phiLength:i},e=Math.floor(e),i=We(i,0,Math.PI*2);const s=[],a=[],o=[],c=[],l=[],h=1/e,u=new R,d=new Mt,f=new R,g=new R,x=new R;let m=0,p=0;for(let _=0;_<=t.length-1;_++)switch(_){case 0:m=t[_+1].x-t[_].x,p=t[_+1].y-t[_].y,f.x=p*1,f.y=-m,f.z=p*0,x.copy(f),f.normalize(),c.push(f.x,f.y,f.z);break;case t.length-1:c.push(x.x,x.y,x.z);break;default:m=t[_+1].x-t[_].x,p=t[_+1].y-t[_].y,f.x=p*1,f.y=-m,f.z=p*0,g.copy(f),f.x+=x.x,f.y+=x.y,f.z+=x.z,f.normalize(),c.push(f.x,f.y,f.z),x.copy(g)}for(let _=0;_<=e;_++){const v=n+_*h*i,S=Math.sin(v),M=Math.cos(v);for(let y=0;y<=t.length-1;y++){u.x=t[y].x*S,u.y=t[y].y,u.z=t[y].x*M,a.push(u.x,u.y,u.z),d.x=_/e,d.y=y/(t.length-1),o.push(d.x,d.y);const w=c[3*y+0]*S,A=c[3*y+1],E=c[3*y+0]*M;l.push(w,A,E)}}for(let _=0;_<e;_++)for(let v=0;v<t.length-1;v++){const S=v+_*t.length,M=S,y=S+t.length,w=S+t.length+1,A=S+1;s.push(M,y,A),s.push(w,A,y)}this.setIndex(s),this.setAttribute("position",new ie(a,3)),this.setAttribute("uv",new ie(o,2)),this.setAttribute("normal",new ie(l,3))}copy(t){return super.copy(t),this.parameters=Object.assign({},t.parameters),this}static fromJSON(t){return new Hh(t.points,t.segments,t.phiStart,t.phiLength)}}class gr extends Hh{constructor(t=1,e=1,n=4,i=8){const s=new Hv;s.absarc(0,-e/2,t,Math.PI*1.5,0),s.absarc(0,e/2,t,0,Math.PI*.5),super(s.getPoints(n),i),this.type="CapsuleGeometry",this.parameters={radius:t,length:e,capSegments:n,radialSegments:i}}static fromJSON(t){return new gr(t.radius,t.length,t.capSegments,t.radialSegments)}}class Ps extends me{constructor(t=1,e=1,n=1,i=32,s=1,a=!1,o=0,c=Math.PI*2){super(),this.type="CylinderGeometry",this.parameters={radiusTop:t,radiusBottom:e,height:n,radialSegments:i,heightSegments:s,openEnded:a,thetaStart:o,thetaLength:c};const l=this;i=Math.floor(i),s=Math.floor(s);const h=[],u=[],d=[],f=[];let g=0;const x=[],m=n/2;let p=0;_(),a===!1&&(t>0&&v(!0),e>0&&v(!1)),this.setIndex(h),this.setAttribute("position",new ie(u,3)),this.setAttribute("normal",new ie(d,3)),this.setAttribute("uv",new ie(f,2));function _(){const S=new R,M=new R;let y=0;const w=(e-t)/n;for(let A=0;A<=s;A++){const E=[],T=A/s,L=T*(e-t)+t;for(let C=0;C<=i;C++){const F=C/i,D=F*c+o,N=Math.sin(D),B=Math.cos(D);M.x=L*N,M.y=-T*n+m,M.z=L*B,u.push(M.x,M.y,M.z),S.set(N,w,B).normalize(),d.push(S.x,S.y,S.z),f.push(F,1-T),E.push(g++)}x.push(E)}for(let A=0;A<i;A++)for(let E=0;E<s;E++){const T=x[E][A],L=x[E+1][A],C=x[E+1][A+1],F=x[E][A+1];h.push(T,L,F),h.push(L,C,F),y+=6}l.addGroup(p,y,0),p+=y}function v(S){const M=g,y=new Mt,w=new R;let A=0;const E=S===!0?t:e,T=S===!0?1:-1;for(let C=1;C<=i;C++)u.push(0,m*T,0),d.push(0,T,0),f.push(.5,.5),g++;const L=g;for(let C=0;C<=i;C++){const D=C/i*c+o,N=Math.cos(D),B=Math.sin(D);w.x=E*B,w.y=m*T,w.z=E*N,u.push(w.x,w.y,w.z),d.push(0,T,0),y.x=N*.5+.5,y.y=B*.5*T+.5,f.push(y.x,y.y),g++}for(let C=0;C<i;C++){const F=M+C,D=L+C;S===!0?h.push(D,D+1,F):h.push(D+1,D,F),A+=3}l.addGroup(p,A,S===!0?1:2),p+=A}}copy(t){return super.copy(t),this.parameters=Object.assign({},t.parameters),this}static fromJSON(t){return new Ps(t.radiusTop,t.radiusBottom,t.height,t.radialSegments,t.heightSegments,t.openEnded,t.thetaStart,t.thetaLength)}}class Wh extends me{constructor(t=.5,e=1,n=32,i=1,s=0,a=Math.PI*2){super(),this.type="RingGeometry",this.parameters={innerRadius:t,outerRadius:e,thetaSegments:n,phiSegments:i,thetaStart:s,thetaLength:a},n=Math.max(3,n),i=Math.max(1,i);const o=[],c=[],l=[],h=[];let u=t;const d=(e-t)/i,f=new R,g=new Mt;for(let x=0;x<=i;x++){for(let m=0;m<=n;m++){const p=s+m/n*a;f.x=u*Math.cos(p),f.y=u*Math.sin(p),c.push(f.x,f.y,f.z),l.push(0,0,1),g.x=(f.x/e+1)/2,g.y=(f.y/e+1)/2,h.push(g.x,g.y)}u+=d}for(let x=0;x<i;x++){const m=x*(n+1);for(let p=0;p<n;p++){const _=p+m,v=_,S=_+n+1,M=_+n+2,y=_+1;o.push(v,S,y),o.push(S,M,y)}}this.setIndex(o),this.setAttribute("position",new ie(c,3)),this.setAttribute("normal",new ie(l,3)),this.setAttribute("uv",new ie(h,2))}copy(t){return super.copy(t),this.parameters=Object.assign({},t.parameters),this}static fromJSON(t){return new Wh(t.innerRadius,t.outerRadius,t.thetaSegments,t.phiSegments,t.thetaStart,t.thetaLength)}}class ii extends me{constructor(t=1,e=32,n=16,i=0,s=Math.PI*2,a=0,o=Math.PI){super(),this.type="SphereGeometry",this.parameters={radius:t,widthSegments:e,heightSegments:n,phiStart:i,phiLength:s,thetaStart:a,thetaLength:o},e=Math.max(3,Math.floor(e)),n=Math.max(2,Math.floor(n));const c=Math.min(a+o,Math.PI);let l=0;const h=[],u=new R,d=new R,f=[],g=[],x=[],m=[];for(let p=0;p<=n;p++){const _=[],v=p/n;let S=0;p===0&&a===0?S=.5/e:p===n&&c===Math.PI&&(S=-.5/e);for(let M=0;M<=e;M++){const y=M/e;u.x=-t*Math.cos(i+y*s)*Math.sin(a+v*o),u.y=t*Math.cos(a+v*o),u.z=t*Math.sin(i+y*s)*Math.sin(a+v*o),g.push(u.x,u.y,u.z),d.copy(u).normalize(),x.push(d.x,d.y,d.z),m.push(y+S,1-v),_.push(l++)}h.push(_)}for(let p=0;p<n;p++)for(let _=0;_<e;_++){const v=h[p][_+1],S=h[p][_],M=h[p+1][_],y=h[p+1][_+1];(p!==0||a>0)&&f.push(v,S,y),(p!==n-1||c<Math.PI)&&f.push(S,M,y)}this.setIndex(f),this.setAttribute("position",new ie(g,3)),this.setAttribute("normal",new ie(x,3)),this.setAttribute("uv",new ie(m,2))}copy(t){return super.copy(t),this.parameters=Object.assign({},t.parameters),this}static fromJSON(t){return new ii(t.radius,t.widthSegments,t.heightSegments,t.phiStart,t.phiLength,t.thetaStart,t.thetaLength)}}class Xh extends me{constructor(t=1,e=.4,n=12,i=48,s=Math.PI*2){super(),this.type="TorusGeometry",this.parameters={radius:t,tube:e,radialSegments:n,tubularSegments:i,arc:s},n=Math.floor(n),i=Math.floor(i);const a=[],o=[],c=[],l=[],h=new R,u=new R,d=new R;for(let f=0;f<=n;f++)for(let g=0;g<=i;g++){const x=g/i*s,m=f/n*Math.PI*2;u.x=(t+e*Math.cos(m))*Math.cos(x),u.y=(t+e*Math.cos(m))*Math.sin(x),u.z=e*Math.sin(m),o.push(u.x,u.y,u.z),h.x=t*Math.cos(x),h.y=t*Math.sin(x),d.subVectors(u,h).normalize(),c.push(d.x,d.y,d.z),l.push(g/i),l.push(f/n)}for(let f=1;f<=n;f++)for(let g=1;g<=i;g++){const x=(i+1)*f+g-1,m=(i+1)*(f-1)+g-1,p=(i+1)*(f-1)+g,_=(i+1)*f+g;a.push(x,m,_),a.push(m,p,_)}this.setIndex(a),this.setAttribute("position",new ie(o,3)),this.setAttribute("normal",new ie(c,3)),this.setAttribute("uv",new ie(l,2))}copy(t){return super.copy(t),this.parameters=Object.assign({},t.parameters),this}static fromJSON(t){return new Xh(t.radius,t.tube,t.radialSegments,t.tubularSegments,t.arc)}}class xc extends me{constructor(t=new sp(new R(-1,-1,0),new R(-1,1,0),new R(1,1,0)),e=64,n=1,i=8,s=!1){super(),this.type="TubeGeometry",this.parameters={path:t,tubularSegments:e,radius:n,radialSegments:i,closed:s};const a=t.computeFrenetFrames(e,s);this.tangents=a.tangents,this.normals=a.normals,this.binormals=a.binormals;const o=new R,c=new R,l=new Mt;let h=new R;const u=[],d=[],f=[],g=[];x(),this.setIndex(g),this.setAttribute("position",new ie(u,3)),this.setAttribute("normal",new ie(d,3)),this.setAttribute("uv",new ie(f,2));function x(){for(let v=0;v<e;v++)m(v);m(s===!1?e:0),_(),p()}function m(v){h=t.getPointAt(v/e,h);const S=a.normals[v],M=a.binormals[v];for(let y=0;y<=i;y++){const w=y/i*Math.PI*2,A=Math.sin(w),E=-Math.cos(w);c.x=E*S.x+A*M.x,c.y=E*S.y+A*M.y,c.z=E*S.z+A*M.z,c.normalize(),d.push(c.x,c.y,c.z),o.x=h.x+n*c.x,o.y=h.y+n*c.y,o.z=h.z+n*c.z,u.push(o.x,o.y,o.z)}}function p(){for(let v=1;v<=e;v++)for(let S=1;S<=i;S++){const M=(i+1)*(v-1)+(S-1),y=(i+1)*v+(S-1),w=(i+1)*v+S,A=(i+1)*(v-1)+S;g.push(M,y,A),g.push(y,w,A)}}function _(){for(let v=0;v<=e;v++)for(let S=0;S<=i;S++)l.x=v/e,l.y=S/i,f.push(l.x,l.y)}}copy(t){return super.copy(t),this.parameters=Object.assign({},t.parameters),this}toJSON(){const t=super.toJSON();return t.path=this.parameters.path.toJSON(),t}static fromJSON(t){return new xc(new ih[t.path.type]().fromJSON(t.path),t.tubularSegments,t.radius,t.radialSegments,t.closed)}}class En extends _i{constructor(t){super(),this.isMeshStandardMaterial=!0,this.defines={STANDARD:""},this.type="MeshStandardMaterial",this.color=new Qt(16777215),this.roughness=1,this.metalness=0,this.map=null,this.lightMap=null,this.lightMapIntensity=1,this.aoMap=null,this.aoMapIntensity=1,this.emissive=new Qt(0),this.emissiveIntensity=1,this.emissiveMap=null,this.bumpMap=null,this.bumpScale=1,this.normalMap=null,this.normalMapType=0,this.normalScale=new Mt(1,1),this.displacementMap=null,this.displacementScale=1,this.displacementBias=0,this.roughnessMap=null,this.metalnessMap=null,this.alphaMap=null,this.envMap=null,this.envMapIntensity=1,this.wireframe=!1,this.wireframeLinewidth=1,this.wireframeLinecap="round",this.wireframeLinejoin="round",this.flatShading=!1,this.fog=!0,this.setValues(t)}copy(t){return super.copy(t),this.defines={STANDARD:""},this.color.copy(t.color),this.roughness=t.roughness,this.metalness=t.metalness,this.map=t.map,this.lightMap=t.lightMap,this.lightMapIntensity=t.lightMapIntensity,this.aoMap=t.aoMap,this.aoMapIntensity=t.aoMapIntensity,this.emissive.copy(t.emissive),this.emissiveMap=t.emissiveMap,this.emissiveIntensity=t.emissiveIntensity,this.bumpMap=t.bumpMap,this.bumpScale=t.bumpScale,this.normalMap=t.normalMap,this.normalMapType=t.normalMapType,this.normalScale.copy(t.normalScale),this.displacementMap=t.displacementMap,this.displacementScale=t.displacementScale,this.displacementBias=t.displacementBias,this.roughnessMap=t.roughnessMap,this.metalnessMap=t.metalnessMap,this.alphaMap=t.alphaMap,this.envMap=t.envMap,this.envMapIntensity=t.envMapIntensity,this.wireframe=t.wireframe,this.wireframeLinewidth=t.wireframeLinewidth,this.wireframeLinecap=t.wireframeLinecap,this.wireframeLinejoin=t.wireframeLinejoin,this.flatShading=t.flatShading,this.fog=t.fog,this}}class Wv extends _i{constructor(t){super(),this.isMeshPhongMaterial=!0,this.type="MeshPhongMaterial",this.color=new Qt(16777215),this.specular=new Qt(1118481),this.shininess=30,this.map=null,this.lightMap=null,this.lightMapIntensity=1,this.aoMap=null,this.aoMapIntensity=1,this.emissive=new Qt(0),this.emissiveIntensity=1,this.emissiveMap=null,this.bumpMap=null,this.bumpScale=1,this.normalMap=null,this.normalMapType=0,this.normalScale=new Mt(1,1),this.displacementMap=null,this.displacementScale=1,this.displacementBias=0,this.specularMap=null,this.alphaMap=null,this.envMap=null,this.combine=0,this.reflectivity=1,this.refractionRatio=.98,this.wireframe=!1,this.wireframeLinewidth=1,this.wireframeLinecap="round",this.wireframeLinejoin="round",this.flatShading=!1,this.fog=!0,this.setValues(t)}copy(t){return super.copy(t),this.color.copy(t.color),this.specular.copy(t.specular),this.shininess=t.shininess,this.map=t.map,this.lightMap=t.lightMap,this.lightMapIntensity=t.lightMapIntensity,this.aoMap=t.aoMap,this.aoMapIntensity=t.aoMapIntensity,this.emissive.copy(t.emissive),this.emissiveMap=t.emissiveMap,this.emissiveIntensity=t.emissiveIntensity,this.bumpMap=t.bumpMap,this.bumpScale=t.bumpScale,this.normalMap=t.normalMap,this.normalMapType=t.normalMapType,this.normalScale.copy(t.normalScale),this.displacementMap=t.displacementMap,this.displacementScale=t.displacementScale,this.displacementBias=t.displacementBias,this.specularMap=t.specularMap,this.alphaMap=t.alphaMap,this.envMap=t.envMap,this.combine=t.combine,this.reflectivity=t.reflectivity,this.refractionRatio=t.refractionRatio,this.wireframe=t.wireframe,this.wireframeLinewidth=t.wireframeLinewidth,this.wireframeLinecap=t.wireframeLinecap,this.wireframeLinejoin=t.wireframeLinejoin,this.flatShading=t.flatShading,this.fog=t.fog,this}}const sd={enabled:!1,files:{},add:function(r,t){this.enabled!==!1&&(this.files[r]=t)},get:function(r){if(this.enabled!==!1)return this.files[r]},remove:function(r){delete this.files[r]},clear:function(){this.files={}}};class Xv{constructor(t,e,n){const i=this;let s=!1,a=0,o=0,c;const l=[];this.onStart=void 0,this.onLoad=t,this.onProgress=e,this.onError=n,this.itemStart=function(h){o++,s===!1&&i.onStart!==void 0&&i.onStart(h,a,o),s=!0},this.itemEnd=function(h){a++,i.onProgress!==void 0&&i.onProgress(h,a,o),a===o&&(s=!1,i.onLoad!==void 0&&i.onLoad())},this.itemError=function(h){i.onError!==void 0&&i.onError(h)},this.resolveURL=function(h){return c?c(h):h},this.setURLModifier=function(h){return c=h,this},this.addHandler=function(h,u){return l.push(h,u),this},this.removeHandler=function(h){const u=l.indexOf(h);return u!==-1&&l.splice(u,2),this},this.getHandler=function(h){for(let u=0,d=l.length;u<d;u+=2){const f=l[u],g=l[u+1];if(f.global&&(f.lastIndex=0),f.test(h))return g}return null}}}const Yv=new Xv;class _c{constructor(t){this.manager=t!==void 0?t:Yv,this.crossOrigin="anonymous",this.withCredentials=!1,this.path="",this.resourcePath="",this.requestHeader={}}load(){}loadAsync(t,e){const n=this;return new Promise(function(i,s){n.load(t,i,e,s)})}parse(){}setCrossOrigin(t){return this.crossOrigin=t,this}setWithCredentials(t){return this.withCredentials=t,this}setPath(t){return this.path=t,this}setResourcePath(t){return this.resourcePath=t,this}setRequestHeader(t){return this.requestHeader=t,this}}_c.DEFAULT_MATERIAL_NAME="__DEFAULT";const Ri={};class qv extends Error{constructor(t,e){super(t),this.response=e}}class ap extends _c{constructor(t){super(t)}load(t,e,n,i){t===void 0&&(t=""),this.path!==void 0&&(t=this.path+t),t=this.manager.resolveURL(t);const s=sd.get(t);if(s!==void 0)return this.manager.itemStart(t),setTimeout(()=>{e&&e(s),this.manager.itemEnd(t)},0),s;if(Ri[t]!==void 0){Ri[t].push({onLoad:e,onProgress:n,onError:i});return}Ri[t]=[],Ri[t].push({onLoad:e,onProgress:n,onError:i});const a=new Request(t,{headers:new Headers(this.requestHeader),credentials:this.withCredentials?"include":"same-origin"}),o=this.mimeType,c=this.responseType;fetch(a).then(l=>{if(l.status===200||l.status===0){if(l.status===0&&console.warn("THREE.FileLoader: HTTP Status 0 received."),typeof ReadableStream>"u"||l.body===void 0||l.body.getReader===void 0)return l;const h=Ri[t],u=l.body.getReader(),d=l.headers.get("Content-Length")||l.headers.get("X-File-Size"),f=d?parseInt(d):0,g=f!==0;let x=0;const m=new ReadableStream({start(p){_();function _(){u.read().then(({done:v,value:S})=>{if(v)p.close();else{x+=S.byteLength;const M=new ProgressEvent("progress",{lengthComputable:g,loaded:x,total:f});for(let y=0,w=h.length;y<w;y++){const A=h[y];A.onProgress&&A.onProgress(M)}p.enqueue(S),_()}})}}});return new Response(m)}else throw new qv(`fetch for "${l.url}" responded with ${l.status}: ${l.statusText}`,l)}).then(l=>{switch(c){case"arraybuffer":return l.arrayBuffer();case"blob":return l.blob();case"document":return l.text().then(h=>new DOMParser().parseFromString(h,o));case"json":return l.json();default:if(o===void 0)return l.text();{const u=/charset="?([^;"\s]*)"?/i.exec(o),d=u&&u[1]?u[1].toLowerCase():void 0,f=new TextDecoder(d);return l.arrayBuffer().then(g=>f.decode(g))}}}).then(l=>{sd.add(t,l);const h=Ri[t];delete Ri[t];for(let u=0,d=h.length;u<d;u++){const f=h[u];f.onLoad&&f.onLoad(l)}}).catch(l=>{const h=Ri[t];if(h===void 0)throw this.manager.itemError(t),l;delete Ri[t];for(let u=0,d=h.length;u<d;u++){const f=h[u];f.onError&&f.onError(l)}this.manager.itemError(t)}).finally(()=>{this.manager.itemEnd(t)}),this.manager.itemStart(t)}setResponseType(t){return this.responseType=t,this}setMimeType(t){return this.mimeType=t,this}}class op extends nn{constructor(t,e=1){super(),this.isLight=!0,this.type="Light",this.color=new Qt(t),this.intensity=e}dispose(){}copy(t,e){return super.copy(t,e),this.color.copy(t.color),this.intensity=t.intensity,this}toJSON(t){const e=super.toJSON(t);return e.object.color=this.color.getHex(),e.object.intensity=this.intensity,this.groundColor!==void 0&&(e.object.groundColor=this.groundColor.getHex()),this.distance!==void 0&&(e.object.distance=this.distance),this.angle!==void 0&&(e.object.angle=this.angle),this.decay!==void 0&&(e.object.decay=this.decay),this.penumbra!==void 0&&(e.object.penumbra=this.penumbra),this.shadow!==void 0&&(e.object.shadow=this.shadow.toJSON()),e}}const il=new re,rd=new R,ad=new R;class Zv{constructor(t){this.camera=t,this.bias=0,this.normalBias=0,this.radius=1,this.blurSamples=8,this.mapSize=new Mt(512,512),this.map=null,this.mapPass=null,this.matrix=new re,this.autoUpdate=!0,this.needsUpdate=!1,this._frustum=new Bh,this._frameExtents=new Mt(1,1),this._viewportCount=1,this._viewports=[new en(0,0,1,1)]}getViewportCount(){return this._viewportCount}getFrustum(){return this._frustum}updateMatrices(t){const e=this.camera,n=this.matrix;rd.setFromMatrixPosition(t.matrixWorld),e.position.copy(rd),ad.setFromMatrixPosition(t.target.matrixWorld),e.lookAt(ad),e.updateMatrixWorld(),il.multiplyMatrices(e.projectionMatrix,e.matrixWorldInverse),this._frustum.setFromProjectionMatrix(il),n.set(.5,0,0,.5,0,.5,0,.5,0,0,.5,.5,0,0,0,1),n.multiply(il)}getViewport(t){return this._viewports[t]}getFrameExtents(){return this._frameExtents}dispose(){this.map&&this.map.dispose(),this.mapPass&&this.mapPass.dispose()}copy(t){return this.camera=t.camera.clone(),this.bias=t.bias,this.radius=t.radius,this.mapSize.copy(t.mapSize),this}clone(){return new this.constructor().copy(this)}toJSON(){const t={};return this.bias!==0&&(t.bias=this.bias),this.normalBias!==0&&(t.normalBias=this.normalBias),this.radius!==1&&(t.radius=this.radius),(this.mapSize.x!==512||this.mapSize.y!==512)&&(t.mapSize=this.mapSize.toArray()),t.camera=this.camera.toJSON(!1).object,delete t.camera.matrix,t}}class jv extends Zv{constructor(){super(new Uh(-5,5,5,-5,.5,500)),this.isDirectionalLightShadow=!0}}class od extends op{constructor(t,e){super(t,e),this.isDirectionalLight=!0,this.type="DirectionalLight",this.position.copy(nn.DEFAULT_UP),this.updateMatrix(),this.target=new nn,this.shadow=new jv}dispose(){this.shadow.dispose()}copy(t){return super.copy(t),this.target=t.target.clone(),this.shadow=t.shadow.clone(),this}}class $v extends op{constructor(t,e){super(t,e),this.isAmbientLight=!0,this.type="AmbientLight"}}class Kv{constructor(t=1,e=0,n=0){return this.radius=t,this.phi=e,this.theta=n,this}set(t,e,n){return this.radius=t,this.phi=e,this.theta=n,this}copy(t){return this.radius=t.radius,this.phi=t.phi,this.theta=t.theta,this}makeSafe(){return this.phi=Math.max(1e-6,Math.min(Math.PI-1e-6,this.phi)),this}setFromVector3(t){return this.setFromCartesianCoords(t.x,t.y,t.z)}setFromCartesianCoords(t,e,n){return this.radius=Math.sqrt(t*t+e*e+n*n),this.radius===0?(this.theta=0,this.phi=0):(this.theta=Math.atan2(t,n),this.phi=Math.acos(We(e/this.radius,-1,1))),this}clone(){return new this.constructor().copy(this)}}const cd=new R,ho=new R;class Fi{constructor(t=new R,e=new R){this.start=t,this.end=e}set(t,e){return this.start.copy(t),this.end.copy(e),this}copy(t){return this.start.copy(t.start),this.end.copy(t.end),this}getCenter(t){return t.addVectors(this.start,this.end).multiplyScalar(.5)}delta(t){return t.subVectors(this.end,this.start)}distanceSq(){return this.start.distanceToSquared(this.end)}distance(){return this.start.distanceTo(this.end)}at(t,e){return this.delta(e).multiplyScalar(t).add(this.start)}closestPointToPointParameter(t,e){cd.subVectors(t,this.start),ho.subVectors(this.end,this.start);const n=ho.dot(ho);let s=ho.dot(cd)/n;return e&&(s=We(s,0,1)),s}closestPointToPoint(t,e,n){const i=this.closestPointToPointParameter(t,e);return this.delta(n).multiplyScalar(i).add(this.start)}applyMatrix4(t){return this.start.applyMatrix4(t),this.end.applyMatrix4(t),this}equals(t){return t.start.equals(this.start)&&t.end.equals(this.end)}clone(){return new this.constructor().copy(this)}}class Jv extends ss{constructor(t=10,e=10,n=4473924,i=8947848){n=new Qt(n),i=new Qt(i);const s=e/2,a=t/e,o=t/2,c=[],l=[];for(let d=0,f=0,g=-o;d<=e;d++,g+=a){c.push(-o,0,g,o,0,g),c.push(g,0,-o,g,0,o);const x=d===s?n:i;x.toArray(l,f),f+=3,x.toArray(l,f),f+=3,x.toArray(l,f),f+=3,x.toArray(l,f),f+=3}const h=new me;h.setAttribute("position",new ie(c,3)),h.setAttribute("color",new ie(l,3));const u=new gi({vertexColors:!0,toneMapped:!1});super(h,u),this.type="GridHelper"}dispose(){this.geometry.dispose(),this.material.dispose()}}typeof __THREE_DEVTOOLS__<"u"&&__THREE_DEVTOOLS__.dispatchEvent(new CustomEvent("register",{detail:{revision:"160"}}));typeof window<"u"&&(window.__THREE__?console.warn("WARNING: Multiple instances of Three.js being imported."):window.__THREE__="160");function wa(r,t){return!r||r.length!==t?Array.from({length:t},()=>({x:0,y:0,z:0,active:!1})):r}function Vo(r,t){const e=wa(t,r.length),n=r.nodeStorage;if(n){const{x:i,y:s,z:a}=n;for(let o=0;o<r.length;o++){const c=e[o];c.x=i[o],c.y=s[o],c.z=a[o],c.active=!0}return e}for(let i=0;i<r.length;i++){const s=r[i],a=e[i];a.x=s.x,a.y=s.y,a.z=s.z,a.active=!0}return e}function ko(r){for(let t=0;t<r.length;t++){const e=r[t];e.x=0,e.y=0,e.z=0,e.active=!1}}function uo(r,t,e,n,i){const s=r[t];s.x+=e,s.y+=n,s.z+=i,s.active=!0}let cp=32,lp=0,Qv=8,hp=.006,up=.002;function ld(r){cp=r}function hd(r){lp=r}function sl(r,t){hp=r,up=t}const zr=1e-4,rl=1.25,fo=.92,al=.68,tn=.94,tS=2,eS=[.25,.5,.75],ud=[.25,.5,.75];function nS(r,t,e,n){const i={x:new Float64Array(r),y:new Float64Array(r),z:new Float64Array(r),vx:new Float64Array(r),vy:new Float64Array(r),vz:new Float64Array(r),fx:new Float64Array(r),fy:new Float64Array(r),fz:new Float64Array(r),kx:new Float64Array(r),ky:new Float64Array(r),kz:new Float64Array(r),mass:new Float64Array(r),bendingStiffness:new Float64Array(r),bendAngleLimit:new Float64Array(r),pinned:new Uint8Array(r)};return i.mass.fill(t),i.bendingStiffness.fill(e),i.bendAngleLimit.fill(n),i}class iS{constructor(t,e){this._storage=t,this.index=e}get x(){return this._storage.x[this.index]}set x(t){this._storage.x[this.index]=t}get y(){return this._storage.y[this.index]}set y(t){this._storage.y[this.index]=t}get z(){return this._storage.z[this.index]}set z(t){this._storage.z[this.index]=t}get vx(){return this._storage.vx[this.index]}set vx(t){this._storage.vx[this.index]=t}get vy(){return this._storage.vy[this.index]}set vy(t){this._storage.vy[this.index]=t}get vz(){return this._storage.vz[this.index]}set vz(t){this._storage.vz[this.index]=t}get fx(){return this._storage.fx[this.index]}set fx(t){this._storage.fx[this.index]=t}get fy(){return this._storage.fy[this.index]}set fy(t){this._storage.fy[this.index]=t}get fz(){return this._storage.fz[this.index]}set fz(t){this._storage.fz[this.index]=t}get mass(){return this._storage.mass[this.index]}set mass(t){this._storage.mass[this.index]=t}get bendingStiffness(){return this._storage.bendingStiffness[this.index]}set bendingStiffness(t){this._storage.bendingStiffness[this.index]=t}get bendAngleLimit(){return this._storage.bendAngleLimit[this.index]}set bendAngleLimit(t){this._storage.bendAngleLimit[this.index]=t}get kx(){return this._storage.kx[this.index]}set kx(t){this._storage.kx[this.index]=t}get ky(){return this._storage.ky[this.index]}set ky(t){this._storage.ky[this.index]=t}get kz(){return this._storage.kz[this.index]}set kz(t){this._storage.kz[this.index]=t}get pinned(){return this._storage.pinned[this.index]!==0}set pinned(t){this._storage.pinned[this.index]=t?1:0}}class sS{constructor(t,e,{mass:n=1,bendingStiffness:i=cp,smoothingIterations:s=lp,constraintIterations:a=Qv,bendingConstraintIterations:o=0,bendAngleLimit:c=50,bendProjectionStrength:l=.35,curvatureFlow:h=0,logger:u=null}={}){this.segmentLength=e,this.nodeStorage=nS(t,n,i,c),this.nodes=Array.from({length:t},(d,f)=>new iS(this.nodeStorage,f)),this.nodes.nodeStorage=this.nodeStorage,this.smoothingIterations=s,this.constraintIterations=a,this.bendingConstraintIterations=o,this.bendAngleLimit=c,this.bendProjectionStrength=l,this.curvatureFlow=h,this.logger=u,this.iteration=0,this.collisionPrevPositions=null,this._constraintPrevPositions=null,this._bendingCorrections=null,this._smoothPositions=null,this._tensionCorrections=null;for(let d=0;d<t;d++)this.nodeStorage.x[d]=d*e}storeCollisionPreviousPositions(){const{x:t,y:e,z:n}=this.nodeStorage;if(!this.collisionPrevPositions||this.collisionPrevPositions.length!==this.nodes.length){this.collisionPrevPositions=Array.from({length:this.nodes.length},(i,s)=>new R(t[s],e[s],n[s]));return}for(let i=0;i<this.nodes.length;i++)this.collisionPrevPositions[i].set(t[i],e[i],n[i])}computeLength(){const{x:t,y:e,z:n}=this.nodeStorage;let i=0;for(let s=0;s<this.nodes.length-1;s++)i+=Math.hypot(t[s+1]-t[s],e[s+1]-e[s],n[s+1]-n[s]);return i}averageCurvature(){const{kx:t,ky:e,kz:n}=this.nodeStorage;let i=0;for(let s=0;s<this.nodes.length;s++)i+=Math.hypot(t[s],e[s],n[s]);return i/this.nodes.length}bendAngleAt(t){if(t<=0||t>=this.nodes.length-1)return 0;const{x:e,y:n,z:i}=this.nodeStorage,s=e[t]-e[t-1],a=n[t]-n[t-1],o=i[t]-i[t-1],c=e[t+1]-e[t],l=n[t+1]-n[t],h=i[t+1]-i[t],u=Math.hypot(s,a,o),d=Math.hypot(c,l,h);if(u<1e-8||d<1e-8)return 0;const f=(s*c+a*l+o*h)/(u*d);return Math.acos(Math.max(-1,Math.min(1,f)))*180/Math.PI}resetForces(){this.nodeStorage.fx.fill(0),this.nodeStorage.fy.fill(0),this.nodeStorage.fz.fill(0)}updateCurvature(){const{x:t,y:e,z:n,kx:i,ky:s,kz:a}=this.nodeStorage,o=this.segmentLength*this.segmentLength;i.fill(0),s.fill(0),a.fill(0);for(let c=1;c<this.nodes.length-1;c++)i[c]=(t[c-1]-2*t[c]+t[c+1])/o,s[c]=(e[c-1]-2*e[c]+e[c+1])/o,a[c]=(n[c-1]-2*n[c]+n[c+1])/o}accumulateBendingForces(){const t=this.nodes.length;if(t<3)return;const{fx:e,fy:n,fz:i,kx:s,ky:a,kz:o,bendingStiffness:c}=this.nodeStorage;for(let l=1;l<t-1;l++){const h=s[l],u=a[l],d=o[l],f=c[l],x=1+(h*h+u*u+d*d),m=f*h*x,p=f*u*x,_=f*d*x,v=.4,S=.1;l>=2&&(e[l-2]+=S*m,n[l-2]+=S*p,i[l-2]+=S*_),l+2<t&&(e[l+2]+=S*m,n[l+2]+=S*p,i[l+2]+=S*_),e[l-1]+=v*m,n[l-1]+=v*p,i[l-1]+=v*_,e[l+1]+=v*m,n[l+1]+=v*p,i[l+1]+=v*_;const M=v*2+(l>=2?S:0)+(l+2<t?S:0);e[l]-=M*m,n[l]-=M*p,i[l]-=M*_}}integrate(t){const{x:e,y:n,z:i,vx:s,vy:a,vz:o,fx:c,fy:l,fz:h,mass:u,pinned:d}=this.nodeStorage;for(let f=0;f<this.nodes.length;f++){if(d[f]){s[f]=0,a[f]=0,o[f]=0;continue}s[f]+=c[f]/u[f]*t,a[f]+=l[f]/u[f]*t,o[f]+=h[f]/u[f]*t,e[f]+=s[f]*t,n[f]+=a[f]*t,i[f]+=o[f]*t}}projectSegmentLengthConstraints(t=this.constraintIterations){const e=this.segmentLength;for(let n=0;n<t;n++)for(let i=0;i<this.nodes.length-1;i++){const s=this.nodes[i],a=this.nodes[i+1];let o=a.x-s.x,c=a.y-s.y,l=a.z-s.z,h=Math.hypot(o,c,l);if(!h)continue;const u=(h-e)/h;s.pinned&&a.pinned||(s.pinned?(o*=u,c*=u,l*=u,a.x-=o,a.y-=c,a.z-=l):a.pinned?(o*=u,c*=u,l*=u,s.x+=o,s.y+=c,s.z+=l):(o*=u*.5,c*=u*.5,l*=u*.5,s.x+=o,s.y+=c,s.z+=l,a.x-=o,a.y-=c,a.z-=l))}}solveConstraints(t,e={}){const n=Vo(this.nodes,this._constraintPrevPositions);this._constraintPrevPositions=n;const i=e.applyBending??!0,s=e.velocityDamping??.92;if(this.projectSegmentLengthConstraints(),i){for(let c=1;c<this.nodes.length-1;c++){const l=this.nodes[c-1],h=this.nodes[c],u=this.nodes[c+1];if(h.pinned)continue;const d=(l.x+u.x)*.5,f=(l.y+u.y)*.5,g=(l.z+u.z)*.5,x=h.x-d,m=h.y-f,p=h.z-g,_=Math.min(1,h.bendingStiffness*t+this.curvatureFlow),v=x*_,S=m*_,M=p*_;h.x-=v,h.y-=S,h.z-=M}const o=e.bendingConstraintIterations??this.bendingConstraintIterations;o>0&&(this.projectBendingConstraints(o),this.projectSegmentLengthConstraints(Math.max(2,Math.ceil(this.constraintIterations*.5))))}this.smoothingIterations>0&&this.laplacianSmooth();const a=1/t;for(let o=0;o<this.nodes.length;o++){const c=this.nodes[o];if(c.pinned){c.vx=c.vy=c.vz=0;continue}c.vx=(c.x-n[o].x)*a*s,c.vy=(c.y-n[o].y)*a*s,c.vz=(c.z-n[o].z)*a*s}}projectBendingConstraints(t=this.bendingConstraintIterations){if(t<=0||this.nodes.length<3)return;const e=Math.max(0,this.bendAngleLimit),n=Math.max(0,Math.min(1,this.bendProjectionStrength));this._bendingCorrections=wa(this._bendingCorrections,this.nodes.length);for(let i=0;i<t;i++){const s=this._bendingCorrections;ko(s);for(let a=1;a<this.nodes.length-1;a++){const o=this.nodes[a];if(o.pinned)continue;const c=this.bendAngleAt(a);if(c<=e)continue;const l=this.nodes[a-1],h=this.nodes[a+1],u=Math.max(0,Math.min(1,(c-e)/Math.max(1,180-e))),d=n*(.35+.65*u),f=s[a];f.x=((l.x+h.x)*.5-o.x)*d,f.y=((l.y+h.y)*.5-o.y)*d,f.z=((l.z+h.z)*.5-o.z)*d,f.active=!0}for(let a=1;a<this.nodes.length-1;a++){const o=this.nodes[a],c=s[a];!c.active||o.pinned||(o.x+=c.x,o.y+=c.y,o.z+=c.z)}}}laplacianSmooth(){const t=this.nodes.length;if(!(t<3)){this._smoothPositions=wa(this._smoothPositions,t);for(let e=0;e<this.smoothingIterations;e++){const n=this._smoothPositions;ko(n);for(let i=1;i<t-1;i++){if(this.nodes[i].pinned)continue;const a=this.nodes[i-1],o=this.nodes[i+1],c=n[i];c.x=(a.x+o.x)*.5,c.y=(a.y+o.y)*.5,c.z=(a.z+o.z)*.5,c.active=!0}for(let i=1;i<t-1;i++){const s=this.nodes[i];if(s.pinned)continue;const a=n[i];a.active&&(s.x=a.x,s.y=a.y,s.z=a.z)}}}}straightenByTension(t=.2,e=1){const n=this.nodes.length;if(n<3||t<=0||e<=0)return;const i=Math.max(0,Math.min(1,t));this._tensionCorrections=wa(this._tensionCorrections,n);for(let s=0;s<e;s++){const a=this._tensionCorrections;ko(a);for(let o=1;o<n-1;o++){const c=this.nodes[o];if(c.pinned)continue;const l=this.nodes[o-1],h=this.nodes[o+1],u=a[o];u.x=((l.x+h.x)*.5-c.x)*i,u.y=((l.y+h.y)*.5-c.y)*i,u.z=((l.z+h.z)*.5-c.z)*i,u.active=!0}for(let o=1;o<n-1;o++){const c=this.nodes[o],l=a[o];!l.active||c.pinned||(c.x+=l.x,c.y+=l.y,c.z+=l.z,c.vx*=.65,c.vy*=.65,c.vz*=.65)}}}releaseFromVesselWall(t,e=.1,n=1){if(!t||e<=0||n<=0)return;const i=Math.max(0,Math.min(1,e));for(let s=0;s<n;s++)for(const a of this.nodes){if(a.pinned)continue;let o=null;for(const h of t){if(h.isSheath)continue;const u=h.end.x-h.start.x,d=h.end.y-h.start.y,f=h.end.z-h.start.z,g=u*u+d*d+f*f;if(!g)continue;const x=a.x-h.start.x,m=a.y-h.start.y,p=a.z-h.start.z,_=(x*u+m*d+p*f)/g,v=Math.max(0,Math.min(1,_)),S=h.start.x+u*v,M=h.start.y+d*v,y=h.start.z+f*v,w=a.x-S,A=a.y-M,E=a.z-y,T=Math.hypot(w,A,E),L=T/(h.radius||1),C=Math.abs(L-.75)+Math.max(0,Math.abs(_-.5)-.5);L<=1.25&&(!o||C<o.distanceScore)&&(o={cx:S,cy:M,cz:y,radialDist:T,normalized:L,distanceScore:C})}if(!o||o.radialDist<=this.segmentLength*.15)continue;const c=Math.max(0,Math.min(1,(o.normalized-.25)/.75)),l=i*c;a.x+=(o.cx-a.x)*l,a.y+=(o.cy-a.y)*l,a.z+=(o.cz-a.z)*l,a.vx*=.75,a.vy*=.75,a.vz*=.75}}applyWallResponse(t,e,n,i,s,a){const o=t.vx*e+t.vy*n+t.vz*i;let c=t.vx-o*e,l=t.vy-o*n,h=t.vz-o*i;const u=Math.sqrt(c*c+l*l+h*h),d=a?Math.abs(o)*t.mass/s:0,f=Math.max(0,t.fx*e+t.fy*n+t.fz*i)+d;if(f>0&&u>0){const g=hp*f*s/t.mass,x=up*f*s/t.mass;if(u<=g)c=0,l=0,h=0;else{const m=Math.max(0,u-x)/(u||1);c*=m,l*=m,h*=m}}t.vx=c,t.vy=l,t.vz=h}isPastOpenSheathEntrance(t,e){for(const n of e){if(!n.isSheath)continue;const i=n.end.x-n.start.x,s=n.end.y-n.start.y,a=n.end.z-n.start.z,o=i*i+s*s+a*a;if(!o)continue;const c=t.x-n.start.x,l=t.y-n.start.y,h=t.z-n.start.z,u=(c*i+l*s+h*a)/o;if(u>=-zr)continue;const d=c-i*u,f=l-s*u,g=h-a*u;if(Math.hypot(d,f,g)<=n.radius+this.segmentLength)return!0}return!1}isInsideSegmentVolume(t,e){return this.segmentVolumeContact(t,e).inside}segmentVolumeContact(t,e){let n=null;for(const i of e||[]){const s=i.end.x-i.start.x,a=i.end.y-i.start.y,o=i.end.z-i.start.z,c=s*s+a*a+o*o;if(!c)continue;const l=t.x-i.start.x,h=t.y-i.start.y,u=t.z-i.start.z,d=(l*s+h*a+u*o)/c,f=Math.max(0,Math.min(1,d)),g=i.start.x+s*f,x=i.start.y+a*f,m=i.start.z+o*f,p=t.x-g,_=t.y-x,v=t.z-m,S=Math.hypot(p,_,v),M=S-i.radius;if(M<=0)return{inside:!0,segment:i,outside:M,cx:g,cy:x,cz:m,rx:p,ry:_,rz:v,radialDist:S,rawT:d};(!n||M<n.outside)&&(n={inside:!1,segment:i,outside:M,cx:g,cy:x,cz:m,rx:p,ry:_,rz:v,radialDist:S,rawT:d})}return n||{inside:!1,outside:1/0}}collideWithSegments(t,e,n,i={}){const s=this.segmentVolumeContact(t,e);if(s.inside||!Number.isFinite(s.outside))return!1;if(i.localOnly){const h=i.contactBand??this.segmentLength*rl;if(s.outside>h||s.rawT<-zr||s.rawT>1+zr)return!1}const a=1/(s.radialDist||1),o=s.rx*a,c=s.ry*a,l=s.rz*a;return t.x=s.cx+o*s.segment.radius,t.y=s.cy+c*s.segment.radius,t.z=s.cz+l*s.segment.radius,this.applyWallResponse(t,o,c,l,n,!0),!0}collideRodSegmentsWithSegments(t,e,n={}){if(!t?.length)return;const i=n.segmentSamples||eS,s=n.contactBand??this.segmentLength*rl;for(let a=0;a<this.nodes.length-1;a++){const o=this.nodes[a],c=this.nodes[a+1];if(!(o.pinned&&c.pinned))for(const l of i){const h=1-l,u=l,d={x:o.x*h+c.x*u,y:o.y*h+c.y*u,z:o.z*h+c.z*u};if(this.isPastOpenSheathEntrance(d,t))continue;const f=this.segmentVolumeContact(d,t);if(f.inside||!Number.isFinite(f.outside)||n.localOnly&&(f.outside>s||f.rawT<-zr||f.rawT>1+zr))continue;const g=1/(f.radialDist||1),x=f.cx+f.rx*g*f.segment.radius,m=f.cy+f.ry*g*f.segment.radius,p=f.cz+f.rz*g*f.segment.radius,_=(x-d.x)*al,v=(m-d.y)*al,S=(p-d.z)*al,M=o.pinned?0:h,y=c.pinned?0:u,w=M*M+y*y;if(!(w<=1e-8)){if(!o.pinned){const A=M/w;o.x+=_*A,o.y+=v*A,o.z+=S*A,o.vx*=tn,o.vy*=tn,o.vz*=tn}if(!c.pinned){const A=y/w;c.x+=_*A,c.y+=v*A,c.z+=S*A,c.vx*=tn,c.vy*=tn,c.vz*=tn}}}}}collideWithMeshCollider(t,e,n,i=0,s=null){if(s&&e?.crossingContact){const c=e.crossingContact(s,t,i);if(c){t.x=c.target.x,t.y=c.target.y,t.z=c.target.z;const l=c.normal||new R(1,0,0);return this.applyWallResponse(t,l.x,l.y,l.z,n,!0),!0}}const a=e?.pointContact?.(t,i);if(!a?.violation)return!1;t.x=a.target.x,t.y=a.target.y,t.z=a.target.z;const o=a.normal||new R(1,0,0);return this.applyWallResponse(t,o.x,o.y,o.z,n,!0),!0}isPastOpenMeshOutlet(t,e={}){return Number.isFinite(e.openOutletY)&&t.y>e.openOutletY}meshContactAtPoint(t,e,n={}){const i=Math.max(0,n.clearance||0),s=new R,o=e.boundsTree.closestPointToPoint(t,{point:s})?.distance??t.distanceTo(s),c=typeof n.interiorDirection=="function"?n.interiorDirection(t,s).clone():new R().subVectors(t,s);c.lengthSq()<1e-8&&c.set(1,0,0),c.normalize();const l=new R().subVectors(t,s).dot(c);return{closest:s,interior:c,insideDepth:l,dist:o,clearance:i}}collideWithMesh(t,e,n,i={}){const s=new R(t.x,t.y,t.z),a=this.meshContactAtPoint(s,e,i),{closest:o,interior:c,insideDepth:l,dist:h,clearance:u}=a,d=Math.max(u+this.segmentLength*rl,this.segmentLength*1.5),f=-c.x,g=-c.y,x=-c.z;if(l<u)t.x=o.x+c.x*u,t.y=o.y+c.y*u,t.z=o.z+c.z*u,this.applyWallResponse(t,f,g,x,n,!0);else{if(h>d)return;const m=Math.max(0,t.fx*f+t.fy*g+t.fz*x),p=t.vx*t.vx+t.vy*t.vy+t.vz*t.vz-(t.vx*f+t.vy*g+t.vz*x)**2;m>0&&p>0&&this.applyWallResponse(t,f,g,x,n,!1)}}collideRodSegmentsWithMesh(t,e,n={},i=null){const s=n.segmentSamples||ud,a=Math.max(0,n.segmentClearance??Math.min(n.clearance||0,this.segmentLength*.06));for(let o=0;o<this.nodes.length-1;o++){const c=this.nodes[o],l=this.nodes[o+1];if(!(c.pinned&&l.pinned))for(const h of s){const u=1-h,d=h,f=new R(c.x*u+l.x*d,c.y*u+l.y*d,c.z*u+l.z*d),g={x:f.x,y:f.y,z:f.z};if(this.isPastOpenMeshOutlet(g,n)||i&&(this.isInsideSegmentVolume(g,i)||this.isPastOpenSheathEntrance(g,i)))continue;const x=this.meshContactAtPoint(f,t,n);if(x.insideDepth>=a)continue;const m=x.closest.x+x.interior.x*a,p=x.closest.y+x.interior.y*a,_=x.closest.z+x.interior.z*a,v=(m-f.x)*fo,S=(p-f.y)*fo,M=(_-f.z)*fo,y=c.pinned?0:u,w=l.pinned?0:d,A=y*y+w*w;if(!(A<=1e-8)){if(!c.pinned){const E=y/A;c.x+=v*E,c.y+=S*E,c.z+=M*E,c.vx*=tn,c.vy*=tn,c.vz*=tn}if(!l.pinned){const E=w/A;l.x+=v*E,l.y+=S*E,l.z+=M*E,l.vx*=tn,l.vy*=tn,l.vz*=tn}}}}}collideRodSegmentsWithMeshCollider(t,e,n={},i=null,s=null){const a=n.segmentSamples||ud,o=Math.max(0,n.segmentClearance??Math.min(n.clearance||0,this.segmentLength*.08)),c=this.segmentLength*4.5,l=new R,h=new R,u=new R,d=new R,f=(g,x,m,p,_)=>{h.subVectors(_,p),h.length()>c&&h.setLength(c),h.multiplyScalar(fo);const v=1-m,S=m,M=g.pinned?0:v,y=x.pinned?0:S,w=M*M+y*y;if(!(w<=1e-8)){if(!g.pinned){const A=M/w;g.x+=h.x*A,g.y+=h.y*A,g.z+=h.z*A,g.vx*=tn,g.vy*=tn,g.vz*=tn}if(!x.pinned){const A=y/w;x.x+=h.x*A,x.y+=h.y*A,x.z+=h.z*A,x.vx*=tn,x.vy*=tn,x.vz*=tn}}};for(let g=0;g<this.nodes.length-1;g++){const x=this.nodes[g],m=this.nodes[g+1];if(!(x.pinned&&m.pinned)){if(t.crossingContact){l.set((x.x+m.x)*.5,(x.y+m.y)*.5,(x.z+m.z)*.5);const p={x:l.x,y:l.y,z:l.z};if(this.isPastOpenMeshOutlet(p,n))continue;if(!(i&&(this.isInsideSegmentVolume(p,i)||this.isPastOpenSheathEntrance(p,i)))){u.set(x.x,x.y,x.z),d.set(m.x,m.y,m.z);const v=t.crossingContact(u,d,o);v&&v.t>.03&&v.t<.97&&f(x,m,v.t,v.point,v.target)}}for(const p of a){const _=1-p,v=p;l.set(x.x*_+m.x*v,x.y*_+m.y*v,x.z*_+m.z*v);const S={x:l.x,y:l.y,z:l.z};if(this.isPastOpenMeshOutlet(S,n)||i&&(this.isInsideSegmentVolume(S,i)||this.isPastOpenSheathEntrance(S,i)))continue;const M=t.pointContact(l,o);M?.violation&&f(x,m,p,l,M.target)}}}}collide(t,e=1){if(!t)return;const n=t.segments||null,i=t.meshCollider||t.lumenMeshCollider||null,s=t.collisionGeometry||(t.isBufferGeometry?t:t.geometry||t),a=this.collisionPrevPositions,o={clearance:t.guidewireClearance??t.collisionClearance??t.clearance??0,segmentClearance:t.guidewireSegmentClearance??t.segmentClearance,segmentSamples:t.guidewireSegmentSamples??t.segmentSamples,openOutletY:t.openOutletY,interiorDirection:t.interiorDirection||t.collisionInteriorDirection},c=!!i||!!s?.boundsTree,l={localOnly:c},h=Math.max(1,t.guidewireCollisionPasses??t.collisionPasses??(c?tS:4));if(!c&&!n)return;const u=()=>{for(let d=0;d<this.nodes.length;d++){const f=this.nodes[d];if(!f.pinned&&!(n&&(this.isInsideSegmentVolume(f,n)||this.isPastOpenSheathEntrance(f,n)||this.collideWithSegments(f,n,e,l)||!c))&&!this.isPastOpenMeshOutlet(f,o))if(i){const g=a?.[d]||null;this.collideWithMeshCollider(f,i,e,o.clearance,g)}else s&&s.boundsTree&&this.collideWithMesh(f,s,e,o)}};if(i){for(let d=0;d<h;d++)u(),n&&this.collideRodSegmentsWithSegments(n,e,l),this.collideRodSegmentsWithMeshCollider(i,e,o,n,a);u(),n&&this.collideRodSegmentsWithSegments(n,e,l)}else if(s&&s.boundsTree){for(let d=0;d<h;d++)u(),n&&this.collideRodSegmentsWithSegments(n,e,l),this.collideRodSegmentsWithMesh(s,e,o,n);u(),n&&this.collideRodSegmentsWithSegments(n,e,l)}else for(let d=0;d<h;d++)u(),n&&this.collideRodSegmentsWithSegments(n,e);this.smoothingIterations>0&&this.laplacianSmooth(e),this.storeCollisionPreviousPositions()}step(t){this.storeCollisionPreviousPositions(),this.resetForces(),this.updateCurvature(),this.accumulateBendingForces(),this.integrate(t),this.solveConstraints(t),this.iteration++,this.logger&&this.logger({iteration:this.iteration,curvature:this.averageCurvature(),length:this.computeLength()})}}function It(r,t,e){return Math.max(t,Math.min(e,r))}function Ge(r,t,e){const n=It((e-r)/Math.max(1e-6,t-r),0,1);return n*n*(3-2*n)}const bi=1.35,rS=.72,aS=2.4,oS=.42,cS=.035,lS=18,hS=5,uS=[.06,.12,.15,.25,.35,.45,.55,.65,.75,.85,.9,.94],dS=64,fS=.32,pS=0,mS=1,gS=.55,xS=.45,_S=2,vS=[0,.2,.4,.6,.8,1],SS=142,MS=0,yS=5,ES=10,wS=6,TS=2,AS=128,CS=.38,RS=2,bS=1.1,PS=.1,LS=170,DS=140,IS=3,NS=10,FS=108,BS=1,US=.34,zS=2,OS=96,GS=10,VS=.35,kS=18,ri=.001;function js(){return{query:{inward:{x:0,y:0,z:0},normal:{x:0,y:0,z:0},closestPoint:{x:0,y:0,z:0}},target:{x:0,y:0,z:0},closestPoint:{x:0,y:0,z:0},inward:{x:0,y:0,z:0},normal:{x:0,y:0,z:0}}}function $s(r,t){r.x=t.x,r.y=t.y,r.z=t.z}function dd(r,t,e){r.x+=t.x*e,r.y+=t.y*e,r.z+=t.z*e}function fd(r,t){return Math.hypot(r.x-t.x,r.y-t.y,r.z-t.z)}function pn(r,t){const e=Math.hypot(r.x,r.y,r.z);return e<1e-8?{...t}:{x:r.x/e,y:r.y/e,z:r.z/e}}function pd(r,t,e){return{x:r.x*(1-e)+t.x*e,y:r.y*(1-e)+t.y*e,z:r.z*(1-e)+t.z*e}}function Pi(){return globalThis.performance?.now?.()??Date.now()}function HS(){return{advanceMs:0,solveMs:0,projectMs:0,diagnosticMs:0,pointContactCount:0,diagnosticPointContactCount:0,projectGuidewireCalls:0,nodeProjectionCount:0,segmentProjectionCount:0,segmentSampleCount:0,solveIterations:0,moving:!1,boundaryDrivenFeed:!1,forceRelax:!1,foldGuarded:!1,stabilityRepaired:!1,withdrawalRelaxed:!1}}function WS(r){return r.advanceMs=0,r.solveMs=0,r.projectMs=0,r.diagnosticMs=0,r.pointContactCount=0,r.diagnosticPointContactCount=0,r.projectGuidewireCalls=0,r.nodeProjectionCount=0,r.segmentProjectionCount=0,r.segmentSampleCount=0,r.solveIterations=0,r.moving=!1,r.boundaryDrivenFeed=!1,r.forceRelax=!1,r.foldGuarded=!1,r.stabilityRepaired=!1,r.withdrawalRelaxed=!1,r}class XS{constructor({rod:t,segmentLength:e,guidewireLength:n,sheath:i,lumenSampler:s=null,advanceRate:a=44,minInsert:o=0,maxInsert:c=n,lumenClearance:l=rS,axialWindowScale:h=aS,straightening:u=oS,routeBlend:d=cS,relaxationIterations:f=lS,lengthIterations:g=hS,segmentSamples:x=uS,maxBendAngle:m=dS,bendLimitStrength:p=fS,bendLimitIterations:_=pS,segmentProjectionBlend:v=mS,maxSegmentProjectionStep:S=gS,meshClearance:M=xS,collisionProjectionRepeats:y=_S,foldAngle:w=SS,foldUntangleStrength:A=MS,foldUntangleWindow:E=yS,finalCollisionPasses:T=ES,finalLengthPasses:L=wS,finalProjectionPasses:C=TS,foldGuardAngle:F=AS,foldGuardStrength:D=CS,foldGuardPasses:N=RS,foldGuardCenterPull:B=bS,stabilityRepairSegmentError:G=PS,stabilityRepairBendAngle:z=LS,stabilityRepairTargetBendAngle:H=DS,stabilityRepairPasses:j=IS,stabilityRepairLengthIterations:$=NS,tipBacktrackAngle:Q=FS,tipBacktrackStrength:V=BS,withdrawalStraightening:K=US,withdrawalStraighteningPasses:nt=zS,withdrawalRelaxFrames:rt=OS,unsupportedBendRelaxAngle:ot=GS,unsupportedBendSupportBand:_t=VS,unsupportedBendRelaxFrames:ht=kS}){this.rod=t,this.segmentLength=e,this.guidewireLength=n,this.sheath=i,this.lumenSampler=typeof s=="function"?s:null,this.advanceRate=a,this.minInsert=o,this.maxInsert=c,this.lumenClearance=l,this.axialWindowScale=h,this.straightening=u,this.routeBlend=d,this.relaxationIterations=f,this.lengthIterations=g,this.segmentSamples=x,this.maxBendAngle=m,this.bendLimitStrength=p,this.bendLimitIterations=_,this.segmentProjectionBlend=v,this.maxSegmentProjectionStep=S,this.meshClearance=M,this.collisionProjectionRepeats=Math.max(1,Math.floor(y)),this.foldAngle=w,this.foldUntangleStrength=A,this.foldUntangleWindow=E,this.finalCollisionPasses=T,this.finalLengthPasses=L,this.finalProjectionPasses=C,this.foldGuardAngle=F,this.foldGuardStrength=D,this.foldGuardPasses=N,this.foldGuardCenterPull=B,this.stabilityRepairSegmentError=G,this.stabilityRepairBendAngle=z,this.stabilityRepairTargetBendAngle=H,this.stabilityRepairPasses=j,this.stabilityRepairLengthIterations=$,this.tipBacktrackAngle=Q,this.tipBacktrackStrength=V,this.withdrawalStraightening=K,this.withdrawalStraighteningPasses=nt,this.withdrawalRelaxFrames=rt,this.unsupportedBendRelaxAngle=ot,this.unsupportedBendSupportBand=_t,this.unsupportedBendRelaxFrames=ht,this.tailProgress=0,this.lastAdvanceDelta=0,this.settleFramesRemaining=0,this.withdrawalRelaxFramesRemaining=0,this.unsupportedBendRelaxFramesRemaining=0,this.unsupportedBendRelaxArmed=!0,this.contactPoints=[],this.breachPoints=[],this.previousPositions=null,this.performanceStats=HS(),this._advancePreviousPositions=null,this._solvePreviousPositions=null,this._straightenCorrections=null,this._spanCorrections=null,this._untangleCorrections=null,this._bendLimitCorrections=null,this._diagnosticContact=js(),this._supportContact=js(),this._projectContact=js(),this._slidePointContact=js(),this._slideTargetContact=js(),this._zeroVelocityContact=js(),this._projectNodePoint={x:0,y:0,z:0},this._convectSource={x:0,y:0,z:0},this._lumenConstraintState={projected:{x:0,y:0,z:0},radialMargin:0,axialOffset:0,axialWindow:0,breach:!1};const ut={x:i.end.x-i.start.x,y:i.end.y-i.start.y,z:i.end.z-i.start.z};this.sheathLength=Math.hypot(ut.x,ut.y,ut.z)||1,this.sheathDir=pn(ut,{x:1,y:0,z:0}),this.externalTailStart={x:i.start.x-this.sheathDir.x*n,y:i.start.y-this.sheathDir.y*n,z:i.start.z-this.sheathDir.z*n}}get progress(){return this.tailProgress}getPerformanceStats(){return{...this.performanceStats}}reset(){return this.tailProgress=this.minInsert,this.lastAdvanceDelta=0,this.settleFramesRemaining=0,this.withdrawalRelaxFramesRemaining=0,this.unsupportedBendRelaxFramesRemaining=0,this.unsupportedBendRelaxArmed=!0,this.contactPoints.length=0,this.breachPoints.length=0,this.initialize(),this}initialize(){const t=this.rod.nodes.nodeStorage;if(t){const{x:e,y:n,z:i,vx:s,vy:a,vz:o,pinned:c}=t;for(let l=0;l<this.rod.nodes.length;l++){const h=this.segmentLength*l;e[l]=this.externalTailStart.x+this.sheathDir.x*h,n[l]=this.externalTailStart.y+this.sheathDir.y*h,i[l]=this.externalTailStart.z+this.sheathDir.z*h,s[l]=0,a[l]=0,o[l]=0,c[l]=1}}else for(let e=0;e<this.rod.nodes.length;e++){const n=this.segmentLength*e,i=this.rod.nodes[e];i.x=this.externalTailStart.x+this.sheathDir.x*n,i.y=this.externalTailStart.y+this.sheathDir.y*n,i.z=this.externalTailStart.z+this.sheathDir.z*n,i.vx=i.vy=i.vz=0,i.pinned=!0}this.constrainSheath(),this.previousPositions=Vo(this.rod.nodes,this._advancePreviousPositions),this._advancePreviousPositions=this.previousPositions}insertedCoordinate(t){return this.segmentLength*t-this.guidewireLength+this.tailProgress}firstLumenNodeIndex(){return It(Math.ceil((this.sheathLength+this.guidewireLength-this.tailProgress)/this.segmentLength),0,this.rod.nodes.length)}firstInsertedNodeIndex(){return It(Math.ceil((this.guidewireLength-this.tailProgress)/this.segmentLength),0,this.rod.nodes.length)}#t(){return It(Math.floor((this.sheathLength+ri+this.guidewireLength-this.tailProgress)/this.segmentLength)+1,0,this.rod.nodes.length)}#e(){return Math.max(0,this.#t()-1)}sheathAxisPoint(t){return{x:this.sheath.start.x+this.sheathDir.x*t,y:this.sheath.start.y+this.sheathDir.y*t,z:this.sheath.start.z+this.sheathDir.z*t}}routeSample(t){return this.#n(t)?{point:this.sheathAxisPoint(t),tangent:{...this.sheathDir},radius:this.sheath.radius||2}:this.lumenSampler?this.lumenSampler(Math.max(0,t-this.sheathLength)):{point:this.sheathAxisPoint(t),tangent:{...this.sheathDir},radius:1/0}}constrainSheath(t=0){const e=this.rod.nodes.nodeStorage;if(e){const{x:n,y:i,z:s,vx:a,vy:o,vz:c,pinned:l}=e,h=this.#t();for(let u=0;u<h;u++){const d=this.insertedCoordinate(u);l[u]=1,n[u]=this.sheath.start.x+this.sheathDir.x*d,i[u]=this.sheath.start.y+this.sheathDir.y*d,s[u]=this.sheath.start.z+this.sheathDir.z*d,a[u]=this.sheathDir.x*t,o[u]=this.sheathDir.y*t,c[u]=this.sheathDir.z*t}l.fill(0,h);return}for(let n=0;n<this.rod.nodes.length;n++){const i=this.insertedCoordinate(n),s=this.rod.nodes[n],a=this.#n(i);s.pinned=a,a&&(s.x=this.sheath.start.x+this.sheathDir.x*i,s.y=this.sheath.start.y+this.sheathDir.y*i,s.z=this.sheath.start.z+this.sheathDir.z*i,s.vx=this.sheathDir.x*t,s.vy=this.sheathDir.y*t,s.vz=this.sheathDir.z*t)}}advance(t,e,n=null,{routeAssist:i=!0,boundaryDriven:s=!1}={}){WS(this.performanceStats);const a=Pi(),o=Vo(this.rod.nodes,this._advancePreviousPositions);this._advancePreviousPositions=o;const c=It(this.tailProgress+t*this.advanceRate*e,this.minInsert,this.maxInsert),l=c-this.tailProgress;this.tailProgress=c,this.lastAdvanceDelta=l,Math.abs(l)>1e-6&&(this.requestSettle(),this.unsupportedBendRelaxArmed=!0,this.unsupportedBendRelaxFramesRemaining=0),l<-1e-6&&(this.withdrawalRelaxFramesRemaining=Math.max(this.withdrawalRelaxFramesRemaining,Math.max(0,Math.floor(this.withdrawalRelaxFrames))));const h=l/Math.max(e,1e-6);return this.constrainSheath(h),Math.abs(l)>1e-6&&!s&&this.#r(l,o,e,n,i),this.previousPositions=o,this.performanceStats.advanceMs+=Pi()-a,this.performanceStats.moving=Math.abs(l)>1e-6,this.performanceStats.boundaryDrivenFeed=s&&Math.abs(l)>1e-6,l}requestSettle(t=48){this.settleFramesRemaining=Math.max(this.settleFramesRemaining,t)}solve(t,e=null,{iterations:n=this.relaxationIterations,forceRelax:i=!1}={}){const s=Pi();this.performanceStats.forceRelax=this.performanceStats.forceRelax||!!i;const a=Vo(this.rod.nodes,this._solvePreviousPositions);this._solvePreviousPositions=a,this.contactPoints.length=0,this.breachPoints.length=0,this.constrainSheath();const o=this.lastAdvanceDelta>1e-6,c=this.lastAdvanceDelta<-1e-6||this.withdrawalRelaxFramesRemaining>0;let l=!1;!o&&(this.unsupportedBendRelaxArmed||this.unsupportedBendRelaxFramesRemaining>0)&&(l=this.#T(e),l&&this.unsupportedBendRelaxArmed?(this.unsupportedBendRelaxFramesRemaining=Math.max(this.unsupportedBendRelaxFramesRemaining,Math.max(1,Math.floor(this.unsupportedBendRelaxFrames))),this.unsupportedBendRelaxArmed=!1):l||(this.unsupportedBendRelaxFramesRemaining=0,this.unsupportedBendRelaxArmed=!0));const h=l&&this.unsupportedBendRelaxFramesRemaining>0;if(!(i||Math.abs(this.lastAdvanceDelta)>1e-6||this.settleFramesRemaining>0||c||h)){this.#U(a,t,e),this.performanceStats.solveMs+=Pi()-s;return}Math.abs(this.lastAdvanceDelta)>1e-6&&(this.#o(),this.#g(e));const d=c||h;d&&(this.performanceStats.withdrawalRelaxed=this.#a(e,c?1:.72)||this.performanceStats.withdrawalRelaxed);const f=Math.max(1,n);this.performanceStats.solveIterations+=f;for(let x=0;x<f;x++){this.#p(x/f,e),d&&x<2&&(this.performanceStats.withdrawalRelaxed=this.#a(e,c?1:.72)||this.performanceStats.withdrawalRelaxed),this.#A();for(let m=0;m<this.collisionProjectionRepeats;m++)this.#v(),this.#s(this.lengthIterations),this.#l(e,!1);this.#s(2),this.#l(e,!1)}this.constrainSheath();for(let x=0;x<this.finalCollisionPasses;x++)this.#A(),this.#v(),this.#l(e,!1),this.#s(this.lengthIterations+2);this.#s(this.lengthIterations+4);for(let x=0;x<this.finalLengthPasses;x++)this.#l(e,!1),this.#s(5);for(let x=0;x<this.finalProjectionPasses;x++)this.#l(e,!1);this.#g(e),this.#s(Math.max(2,Math.ceil(this.lengthIterations*.4)));let g=this.#C();this.performanceStats.foldGuarded=this.performanceStats.foldGuarded||g,this.#l(e,!1),(g||this.#y(this.foldGuardAngle))&&(g=this.#C()||g,this.performanceStats.foldGuarded=this.performanceStats.foldGuarded||g,this.#s(this.lengthIterations+4),this.#l(e,!1),this.#s(4),this.#l(e,!1)),this.#w(e)&&(this.performanceStats.stabilityRepaired=!0,this.performanceStats.foldGuarded=!0),this.#U(a,t,e),Math.abs(this.lastAdvanceDelta)<=1e-6&&this.settleFramesRemaining>0&&this.settleFramesRemaining--,this.lastAdvanceDelta>=-1e-6&&this.withdrawalRelaxFramesRemaining>0&&this.withdrawalRelaxFramesRemaining--,this.unsupportedBendRelaxFramesRemaining>0&&(this.unsupportedBendRelaxFramesRemaining--,this.unsupportedBendRelaxFramesRemaining<=0&&!l&&(this.unsupportedBendRelaxArmed=!0)),this.performanceStats.solveMs+=Pi()-s}collectContactSamples(t=null,e=bi){const n=[],i=[],s=[0,.15,.35,.55,.75,.9,1],a=(c,l)=>{if(this.#n(l))return;const h=this.diagnosePoint(c,l,t,e);h.breach?this.#d(i,c):h.contact&&this.#d(n,c)};for(let c=0;c<this.rod.nodes.length-1;c++){const l=this.rod.nodes[c],h=this.rod.nodes[c+1];for(const u of s){const d=this.insertedCoordinate(c+u),f=pd(l,h,u);a(f,d)}}const o=this.rod.nodes[this.rod.nodes.length-1];return a(o,this.insertedCoordinate(this.rod.nodes.length-1)),{contacts:n,breaches:i}}collectLumenDiagnostics(t=null,{clearance:e=this.meshClearance,contactBand:n=bi,samples:i=vS,collectMarkers:s=!1,markerLimit:a=420}={}){const o=Pi();this.performanceStats.diagnosticMs=0,this.performanceStats.diagnosticPointContactCount=0;const c=t?.meshCollider||t?.lumenMeshCollider||null,l={checkedCount:0,contactCount:0,outsideCount:0,clearanceViolationCount:0,minSignedDistance:null,minClearanceMargin:null,worstPoint:null,worstInserted:null,maxSegmentError:0,maxBendAngle:0,clearance:e,contactBand:n,contacts:s?[]:null,breaches:s?[]:null};for(let h=0;h<this.rod.nodes.length-1;h++){const u=this.rod.nodes[h],d=this.rod.nodes[h+1];if(l.maxSegmentError=Math.max(l.maxSegmentError,Math.abs(fd(u,d)-this.segmentLength)),c?.pointContact)for(const f of i){const g=this.insertedCoordinate(h+f);if(this.#n(g))continue;const x=pd(u,d,f),m=this.#R(c,x,e,!0,this._diagnosticContact),p=Number.isFinite(m?.signedDistance)?m.signedDistance:null;if(!Number.isFinite(p))continue;l.checkedCount++,(l.minSignedDistance===null||p<l.minSignedDistance)&&(l.minSignedDistance=p,l.worstPoint={x:x.x,y:x.y,z:x.z},l.worstInserted=g);const _=p-e;(l.minClearanceMargin===null||_<l.minClearanceMargin)&&(l.minClearanceMargin=_),p<0?(l.outsideCount++,s&&this.#d(l.breaches,x,a)):p<=n&&(l.contactCount++,s&&this.#d(l.contacts,x,a)),p<e&&l.clearanceViolationCount++}}if(typeof this.rod.bendAngleAt=="function")for(let h=1;h<this.rod.nodes.length-1;h++){const u=this.insertedCoordinate(h);this.#n(u)||(l.maxBendAngle=Math.max(l.maxBendAngle,this.rod.bendAngleAt(h)||0))}return this.performanceStats.diagnosticMs=Pi()-o,l}diagnosePoint(t,e,n=null,i=bi){const s=this.lumenSampler?this.#I(t,e):null;let a=s?s.radialMargin<=i||Math.abs(s.axialOffset)>=s.axialWindow-i:!1,o=s?.breach||!1;const c=n?.meshCollider||n?.lumenMeshCollider||null;if(c?.pointContact&&!this.#n(e)){const l=this.#R(c,t,0,!0,this._diagnosticContact);o=o||!!l?.violation,a=a||!l?.violation&&Number.isFinite(l?.distance)&&l.distance<=i}return{contact:!o&&a,breach:o}}#r(t,e,n,i=null,s=!0){const a=t/this.segmentLength,o=1/Math.max(n,1e-6),c=s?this.#M(i):null,l=this.#t();for(let h=l;h<this.rod.nodes.length;h++){const u=this.rod.nodes[h],d=this.insertedCoordinate(h);if(this.#n(d))continue;const f=this.#f(e,h+a,s?i:null,this._convectSource,s);if(!s){const M=e[h];$s(u,f),u.vx=(f.x-M.x)*o*.2,u.vy=(f.y-M.y)*o*.2,u.vz=(f.z-M.z)*o*.2;continue}const g=this.lumenSampler?this.routeSample(d).point:f,x=d<this.sheathLength+this.segmentLength*2,m=this.lumenSampler?x?.64:.12:0,p={x:f.x*(1-m)+g.x*m,y:f.y*(1-m)+g.y*m,z:f.z*(1-m)+g.z*m},_=e[h],v=this.#_(_,{x:p.x-_.x,y:p.y-_.y,z:p.z-_.z},c);p.x=_.x+v.x,p.y=_.y+v.y,p.z=_.z+v.z;const S=this.#x(p,d,i,!1);$s(u,S),u.vx=(S.x-_.x)*o*.2,u.vy=(S.y-_.y)*o*.2,u.vz=(S.z-_.z)*o*.2}}#f(t,e,n=null,i={x:0,y:0,z:0},s=!0){const a=t.length-1;if(e<=0){const _=t[0];return i.x=_.x+this.sheathDir.x*e*this.segmentLength,i.y=_.y+this.sheathDir.y*e*this.segmentLength,i.z=_.z+this.sheathDir.z*e*this.segmentLength,i}if(e<a){const _=Math.floor(e),v=Math.min(a,_+1),S=e-_,M=t[_],y=t[v];return i.x=M.x+(y.x-M.x)*S,i.y=M.y+(y.y-M.y)*S,i.z=M.z+(y.z-M.z)*S,i}const o=t[a],c=t[Math.max(0,a-1)];let l=o.x-c.x,h=o.y-c.y,u=o.z-c.z;const d=Math.sqrt(l*l+h*h+u*u);if(d>1e-8)l/=d,h/=d,u/=d;else if(s){const _=this.routeSample(this.tailProgress).tangent;l=_.x,h=_.y,u=_.z}else l=this.sheathDir.x,h=this.sheathDir.y,u=this.sheathDir.z;let f=l,g=h,x=u;const m=s?this.#M(n):null;if(m){const _=pn(this.#_(o,{x:l,y:h,z:u},m),{x:l,y:h,z:u});f=_.x,g=_.y,x=_.z}const p=(e-a)*this.segmentLength;return i.x=o.x+f*p,i.y=o.y+g*p,i.z=o.z+x*p,i}#o(t=1){if(this.routeBlend<=0||!this.lumenSampler)return;const e=this.rod.nodes.nodeStorage;if(e){const{x:n,y:i,z:s,pinned:a}=e,o=this.#t();for(let c=o;c<this.rod.nodes.length;c++){const l=this.insertedCoordinate(c);if(this.#n(l))continue;const h=this.routeSample(l),u=Ge(this.sheathLength,this.sheathLength+this.segmentLength*8,l),d=this.routeBlend*t*(.35+.65*u);n[c]+=(h.point.x-n[c])*d,i[c]+=(h.point.y-i[c])*d,s[c]+=(h.point.z-s[c])*d}return}for(let n=0;n<this.rod.nodes.length;n++){const i=this.rod.nodes[n];if(i.pinned)continue;const s=this.insertedCoordinate(n);if(this.#n(s))continue;const a=this.routeSample(s),o=Ge(this.sheathLength,this.sheathLength+this.segmentLength*8,s),c=this.routeBlend*t*(.35+.65*o);i.x+=(a.point.x-i.x)*c,i.y+=(a.point.y-i.y)*c,i.z+=(a.point.z-i.z)*c}}#c(t,e=null){const n=this.rod.nodes[t];if(!n)return!0;const i=this.insertedCoordinate(t);if(this.#n(i))return!0;const s=this.#M(e);if(!s?.pointContact)return!1;const a=this.#R(s,n,this.meshClearance,!1,this._supportContact);return!!a?.violation||Number.isFinite(a?.signedDistance)&&a.signedDistance<=this.meshClearance+this.unsupportedBendSupportBand}#T(t=null){const e=Math.max(0,this.unsupportedBendRelaxAngle);if(e<=0||typeof this.rod.bendAngleAt!="function")return!1;for(let n=1;n<this.rod.nodes.length-1;n++){if(this.rod.nodes[n].pinned||this.insertedCoordinate(n)<=this.sheathLength+this.segmentLength+ri||(this.rod.bendAngleAt(n)||0)<=e)continue;const o=!this.#c(n+1,t),c=!this.#c(n,t);if(o||c)return!0}return!1}#h(t,e,n,i,s,a){let o=n;s?.pointContact&&!this.#n(e)&&(o=this.#_(t,o,s));const c=Math.hypot(o.x,o.y,o.z);if(c<=this.segmentLength*.002)return!1;if(c>a){const h=a/c;o={x:o.x*h,y:o.y*h,z:o.z*h}}const l=this.#x({x:t.x+o.x,y:t.y+o.y,z:t.z+o.z},e,i,!1);return $s(t,l),!0}#a(t=null,e=1){const n=Math.max(0,Math.floor(this.withdrawalStraighteningPasses)),i=It(this.withdrawalStraightening*e,0,1);if(n<=0||i<=0)return!1;const s=this.#M(t),a=this.withdrawalRelaxFramesRemaining>0?.55:.25,o=It(Math.abs(this.lastAdvanceDelta)/Math.max(1e-6,this.segmentLength*.25),a,1),c=this.segmentLength*.16,l=Math.max(1,Math.ceil(56/Math.max(1e-6,this.segmentLength)));let h=!1;for(let u=0;u<n;u++)for(let d=1;d<this.rod.nodes.length;d++){const f=this.rod.nodes[d];if(f.pinned||this.insertedCoordinate(d)<=this.sheathLength+this.segmentLength+ri)continue;const x=this.rod.nodes[d-1],m=this.rod.nodes[d-2],p=m?pn({x:x.x-m.x,y:x.y-m.y,z:x.z-m.z},this.sheathDir):this.sheathDir,_=pn({x:f.x-x.x,y:f.y-x.y,z:f.z-x.z},p),v=It(p.x*_.x+p.y*_.y+p.z*_.z,-1,1),S=It((1-v)/.28,0,1);if(S<=.001)continue;const M={x:x.x+p.x*this.segmentLength,y:x.y+p.y*this.segmentLength,z:x.z+p.z*this.segmentLength},y=i*o*(.25+.75*S);let w={x:(M.x-f.x)*y,y:(M.y-f.y)*y,z:(M.z-f.z)*y};const A=Math.hypot(w.x,w.y,w.z);if(!(A<=1e-8)){if(s?.pointContact){const E=this.#_(f,w,s),T=Math.hypot(E.x,E.y,E.z);if(T<A*.08){const L=pn({x:x.x-f.x,y:x.y-f.y,z:x.z-f.z},{x:-p.x,y:-p.y,z:-p.z}),C=Math.min(c,Math.max(A*.45,this.segmentLength*.035)),F=this.#_(f,{x:L.x*C,y:L.y*C,z:L.z*C},s);w=Math.hypot(F.x,F.y,F.z)>T?F:E}else w=E}for(let E=d;E<this.rod.nodes.length&&E<d+l;E++){const T=this.rod.nodes[E];if(!T||T.pinned)break;const L=this.insertedCoordinate(E);if(this.#n(L)||E>d&&this.#c(E,t))break;const C=this.#h(T,L,w,t,s,c);h=h||C}}}return h}#i(t){const e=wa(this[t],this.rod.nodes.length);return this[t]=e,ko(e),e}#p(t,e=null){const n=this.#i("_straightenCorrections"),i=.35+.65*Ge(0,1,t),s=this.#M(e),a=this.segmentLength*.18,o={x:0,y:0,z:0},c=this.rod.nodes.nodeStorage;if(c){const{x:u,y:d,z:f,pinned:g}=c,x=this.#t(),m=(_,v,S)=>{let M=S.x,y=S.y,w=S.z;if(s?.pointContact&&!this.#n(v)){const E=this.#_(this.rod.nodes[_],S,s);M=E.x,y=E.y,w=E.z}const A=Math.hypot(M,y,w);if(A>a){const E=a/A;M*=E,y*=E,w*=E}return o.x=u[_]+M,o.y=d[_]+y,o.z=f[_]+w,this.#x(o,v,e,!1)};for(let _=Math.max(1,x);_<this.rod.nodes.length-1;_++){if(g[_])continue;const v=this.insertedCoordinate(_),S=1-Ge(this.sheathLength,this.sheathLength+this.segmentLength*5,v),M=this.straightening*i*(1-S*.45),y=n[_];y.x=((u[_-1]+u[_+1])*.5-u[_])*M,y.y=((d[_-1]+d[_+1])*.5-d[_])*M,y.z=((f[_-1]+f[_+1])*.5-f[_])*M,y.active=!0}for(let _=Math.max(1,x);_<this.rod.nodes.length-1;_++){const v=n[_];if(!v.active||g[_])continue;const S=this.insertedCoordinate(_),M=m(_,S,v);u[_]=M.x,d[_]=M.y,f[_]=M.z}const p=[2,4,8,12];for(const _ of p){const v=this.#i("_spanCorrections"),S=this.straightening*.13/Math.sqrt(_);for(let M=Math.max(_,x);M<this.rod.nodes.length-_;M++){if(g[M])continue;const y=v[M];y.x=((u[M-_]+u[M+_])*.5-u[M])*S,y.y=((d[M-_]+d[M+_])*.5-d[M])*S,y.z=((f[M-_]+f[M+_])*.5-f[M])*S,y.active=!0}for(let M=Math.max(_,x);M<this.rod.nodes.length-_;M++){const y=v[M];if(!y.active||g[M])continue;const w=this.insertedCoordinate(M),A=m(M,w,y);u[M]=A.x,d[M]=A.y,f[M]=A.z}}return}const l=(u,d,f)=>{let g=f.x,x=f.y,m=f.z;if(s?.pointContact&&!this.#n(d)){const _=this.#_(u,f,s);g=_.x,x=_.y,m=_.z}const p=Math.hypot(g,x,m);if(p>a){const _=a/p;g*=_,x*=_,m*=_}return o.x=u.x+g,o.y=u.y+x,o.z=u.z+m,this.#x(o,d,e,!1)};for(let u=1;u<this.rod.nodes.length-1;u++){const d=this.rod.nodes[u];if(d.pinned)continue;const f=this.rod.nodes[u-1],g=this.rod.nodes[u+1],x=this.insertedCoordinate(u),m=1-Ge(this.sheathLength,this.sheathLength+this.segmentLength*5,x),p=this.straightening*i*(1-m*.45),_=n[u];_.x=((f.x+g.x)*.5-d.x)*p,_.y=((f.y+g.y)*.5-d.y)*p,_.z=((f.z+g.z)*.5-d.z)*p,_.active=!0}for(let u=1;u<this.rod.nodes.length-1;u++){const d=this.rod.nodes[u],f=n[u];if(!f.active||d.pinned)continue;const g=this.insertedCoordinate(u),x=l(d,g,f);$s(d,x)}const h=[2,4,8,12];for(const u of h){const d=this.#i("_spanCorrections"),f=this.straightening*.13/Math.sqrt(u);for(let g=u;g<this.rod.nodes.length-u;g++){const x=this.rod.nodes[g];if(x.pinned)continue;const m=this.rod.nodes[g-u],p=this.rod.nodes[g+u],_=d[g];_.x=((m.x+p.x)*.5-x.x)*f,_.y=((m.y+p.y)*.5-x.y)*f,_.z=((m.z+p.z)*.5-x.z)*f,_.active=!0}for(let g=u;g<this.rod.nodes.length-u;g++){const x=this.rod.nodes[g],m=d[g];if(!m.active||x.pinned)continue;const p=this.insertedCoordinate(g),_=l(x,p,m);$s(x,_)}}}#A(){if(this.foldUntangleStrength<=0||this.foldUntangleWindow<=0)return;const t=this.#i("_untangleCorrections"),e=It(this.foldAngle,1,179),n=Math.max(1,Math.floor(this.foldUntangleWindow)),i=It(this.foldUntangleStrength,0,1);for(let s=1;s<this.rod.nodes.length-1;s++){if(this.rod.nodes[s].pinned||this.insertedCoordinate(s)<=this.sheathLength+this.segmentLength+ri)continue;const c=this.rod.bendAngleAt?.(s)??0;if(c<=e)continue;const l=It((c-e)/Math.max(1,180-e),0,1);for(let h=-n;h<=n;h++){const u=s+h,d=this.rod.nodes[u];if(!d||d.pinned)continue;const f=this.insertedCoordinate(u);if(this.#n(f))continue;const g=1-Math.abs(h)/(n+1),x=i*l*g;if(x<=0)continue;const m=this.routeSample(f).point;uo(t,u,(m.x-d.x)*x,(m.y-d.y)*x,(m.z-d.z)*x)}}for(let s=1;s<this.rod.nodes.length-1;s++){const a=this.rod.nodes[s],o=t[s];!o.active||a.pinned||dd(a,o,1)}}#v(t=this.maxBendAngle,e=this.bendLimitStrength,n=this.bendLimitIterations,i=.45){if(n<=0||e<=0)return;const s=It(t,1,179),a=s*Math.PI/180,o=2*this.segmentLength*Math.cos(a*.5),c=It(e,0,1);for(let l=0;l<n;l++){const h=this.#i("_bendLimitCorrections");for(let u=1;u<this.rod.nodes.length-1;u++){const d=this.rod.nodes[u];if(d.pinned)continue;const f=this.insertedCoordinate(u);if(f<=this.sheathLength+this.segmentLength+ri)continue;const g=this.rod.bendAngleAt?.(u)??0;if(g<=s)continue;const x=this.rod.nodes[u-1],m=this.rod.nodes[u+1],p={x:m.x-x.x,y:m.y-x.y,z:m.z-x.z},_=Math.hypot(p.x,p.y,p.z),v=this.routeSample(f).tangent,S=pn(p,v),M=It((g-s)/Math.max(1,180-s),0,1),y=c*(.35+.65*M);if(_<o){const w=(o-_)*.5*y;x.pinned||uo(h,u-1,-S.x*w,-S.y*w,-S.z*w),m.pinned||uo(h,u+1,S.x*w,S.y*w,S.z*w)}uo(h,u,((x.x+m.x)*.5-d.x)*y*i,((x.y+m.y)*.5-d.y)*y*i,((x.z+m.z)*.5-d.z)*y*i)}for(let u=1;u<this.rod.nodes.length-1;u++){const d=this.rod.nodes[u],f=h[u];!f.active||d.pinned||dd(d,f,1)}}}#y(t){const e=It(t,1,179);for(let n=1;n<this.rod.nodes.length-1;n++){if(this.rod.nodes[n].pinned||this.insertedCoordinate(n)<=this.sheathLength+this.segmentLength+ri)continue;if((this.rod.bendAngleAt?.(n)??0)>e)return!0}return!1}#E(){let t=0;for(let e=0;e<this.rod.nodes.length-1;e++){const n=this.rod.nodes[e],i=this.rod.nodes[e+1];t=Math.max(t,Math.abs(fd(n,i)-this.segmentLength))}return t}#S(){let t=0;for(let e=1;e<this.rod.nodes.length-1;e++)this.rod.nodes[e].pinned||this.insertedCoordinate(e)<=this.sheathLength+this.segmentLength+ri||(t=Math.max(t,this.rod.bendAngleAt?.(e)??0));return t}#u(t){const e=Math.abs(t.x),n=Math.abs(t.y),i=e<.7?{x:1,y:0,z:0}:n<.7?{x:0,y:1,z:0}:{x:0,y:0,z:1};return pn({x:t.y*i.z-t.z*i.y,y:t.z*i.x-t.x*i.z,z:t.x*i.y-t.y*i.x},{x:1,y:0,z:0})}#m(t,e){const n=It(t,1,179)*Math.PI/180,i=Math.cos(n),s=Math.sin(n),a=It(e,0,1);if(!(a<=0))for(let o=1;o<this.rod.nodes.length-1;o++){const c=this.rod.nodes[o-1],l=this.rod.nodes[o],h=this.rod.nodes[o+1];if(l.pinned||h.pinned||this.insertedCoordinate(o)<=this.sheathLength+this.segmentLength+ri)continue;const d=pn({x:l.x-c.x,y:l.y-c.y,z:l.z-c.z},this.sheathDir),f={x:h.x-l.x,y:h.y-l.y,z:h.z-l.z},g=pn(f,d),x=d.x*g.x+d.y*g.y+d.z*g.z;if(x>=i)continue;let m={x:g.x-d.x*x,y:g.y-d.y*x,z:g.z-d.z*x};const p=Math.hypot(m.x,m.y,m.z);p<1e-8?m=this.#u(d):(m.x/=p,m.y/=p,m.z/=p);const _=pn({x:d.x*i+m.x*s,y:d.y*i+m.y*s,z:d.z*i+m.z*s},d),v={x:l.x+_.x*this.segmentLength,y:l.y+_.y*this.segmentLength,z:l.z+_.z*this.segmentLength};h.x+=(v.x-h.x)*a,h.y+=(v.y-h.y)*a,h.z+=(v.z-h.z)*a}}#g(t=null){const e=It(this.tipBacktrackStrength,0,1);if(e<=0||this.rod.nodes.length<3)return!1;const n=this.rod.nodes.length-1,i=n-1,s=n-2,a=this.rod.nodes[n],o=this.rod.nodes[i],c=this.rod.nodes[s];if(a.pinned||o.pinned||this.insertedCoordinate(i)<=this.sheathLength+this.segmentLength+ri)return!1;const h=It(this.tipBacktrackAngle,1,179)*Math.PI/180,u=Math.cos(h),d=Math.sin(h),f=pn({x:o.x-c.x,y:o.y-c.y,z:o.z-c.z},this.sheathDir),g=pn({x:a.x-o.x,y:a.y-o.y,z:a.z-o.z},f),x=f.x*g.x+f.y*g.y+f.z*g.z;if(x>=u)return!1;let m={x:g.x-f.x*x,y:g.y-f.y*x,z:g.z-f.z*x};const p=Math.hypot(m.x,m.y,m.z);p<1e-8?m=this.#u(f):(m.x/=p,m.y/=p,m.z/=p);const _=pn({x:f.x*u+m.x*d,y:f.y*u+m.y*d,z:f.z*u+m.z*d},f),v=this.#M(t);let S={x:o.x+_.x*this.segmentLength,y:o.y+_.y*this.segmentLength,z:o.z+_.z*this.segmentLength};if(v?.pointContact){const y=pn(this.#_(a,f,v),_),w=this.#x({x:a.x+y.x*this.segmentLength*.8,y:a.y+y.y*this.segmentLength*.8,z:a.z+y.z*this.segmentLength*.8},this.insertedCoordinate(n),t,!1),A=pn({x:w.x-o.x,y:w.y-o.y,z:w.z-o.z},_);f.x*A.x+f.y*A.y+f.z*A.z>x&&(S=w)}const M=this.#x(S,this.insertedCoordinate(n),t,!1);return a.x+=(M.x-a.x)*e,a.y+=(M.y-a.y)*e,a.z+=(M.z-a.z)*e,a.vx*=.2,a.vy*=.2,a.vz*=.2,!0}#C(){if(this.foldGuardPasses<=0||this.foldGuardStrength<=0||!this.#y(this.foldGuardAngle))return!1;const t=Math.max(1,Math.floor(this.foldGuardPasses));let e=!1;for(let n=0;n<t&&(e=!0,this.#m(this.foldGuardAngle,this.foldGuardStrength),this.#v(this.foldGuardAngle,this.foldGuardStrength,1,this.foldGuardCenterPull),this.#s(Math.max(3,Math.ceil(this.lengthIterations*.5))),this.#s(2),!!this.#y(this.foldGuardAngle));n++);return e}#w(t){const e=Math.max(0,Math.floor(this.stabilityRepairPasses));if(e<=0)return!1;const n=Math.max(1e-4,this.stabilityRepairSegmentError),i=It(this.stabilityRepairBendAngle,1,179),s=Math.max(0,Math.floor(this.stabilityRepairLengthIterations));let a=!1;for(let o=0;o<e;o++){const c=this.#E(),l=this.#S();if(c<=n&&l<=i)break;a=!0;const h=It(c/n-1,0,1),u=It((l-i)/Math.max(1,180-i),0,1),d=Math.max(h,u),f=It(Math.min(this.stabilityRepairTargetBendAngle,i),1,179),g=It(Math.max(this.foldGuardStrength,.72)*(.75+.25*d),0,1),x=Math.max(this.foldGuardCenterPull,1.1);this.#m(f,g),this.#v(f,g,2,x),this.#s(this.lengthIterations+s,t,!0),this.#l(t,!1),this.#v(f,g,1,x),this.#s(this.lengthIterations+Math.ceil(s*.5),t,!0),this.#l(t,!1),this.#s(Math.max(4,Math.ceil(this.lengthIterations*.5)),t,!0)}return a}#s(t,e=null,n=!1){const i=this.segmentLength,s=n?this.#M(e):null,a=!!s?.pointContact,o=this.#e(),c=this.rod.nodes.nodeStorage;if(!a&&c){const{x:l,y:h,z:u,pinned:d}=c;for(let f=0;f<t;f++){for(let g=o;g<this.rod.nodes.length-1;g++){const x=l[g+1]-l[g],m=h[g+1]-h[g],p=u[g+1]-u[g],_=Math.hypot(x,m,p);if(_<1e-8)continue;const v=(_-i)/_,S=d[g]?0:1,M=d[g+1]?0:1,y=S+M;if(y<=0)continue;const w=S/y,A=M/y;if(S){const E=v*w;l[g]+=x*E,h[g]+=m*E,u[g]+=p*E}if(M){const E=-v*A;l[g+1]+=x*E,h[g+1]+=m*E,u[g+1]+=p*E}}this.constrainSheath()}return}for(let l=0;l<t;l++){for(let h=o;h<this.rod.nodes.length-1;h++){const u=this.rod.nodes[h],d=this.rod.nodes[h+1],f=d.x-u.x,g=d.y-u.y,x=d.z-u.z,m=Math.hypot(f,g,x);if(m<1e-8)continue;const p=(m-i)/m,_=u.pinned?0:1,v=d.pinned?0:1,S=_+v;if(S<=0)continue;const M=_/S,y=v/S;if(!a){if(_){const w=p*M;u.x+=f*w,u.y+=g*w,u.z+=x*w}if(v){const w=-p*y;d.x+=f*w,d.y+=g*w,d.z+=x*w}continue}if(_){let w={x:f*p*M,y:g*p*M,z:x*p*M};this.#n(this.insertedCoordinate(h))||(w=this.#_(u,w,s)),u.x+=w.x,u.y+=w.y,u.z+=w.z}if(v){let w={x:-f*p*y,y:-g*p*y,z:-x*p*y};this.#n(this.insertedCoordinate(h+1))||(w=this.#_(d,w,s)),d.x+=w.x,d.y+=w.y,d.z+=w.z}}this.constrainSheath()}}#l(t,e){const n=Pi();this.performanceStats.projectGuidewireCalls++,this.#D(t,e),this.#O(t,e),this.#D(t,e),this.performanceStats.projectMs+=Pi()-n}#D(t,e){const n=this.#t(),i=this.rod.nodes.nodeStorage;if(i){const{x:s,y:a,z:o,pinned:c}=i,l=this._projectNodePoint;for(let h=n;h<this.rod.nodes.length;h++){if(c[h])continue;const u=this.insertedCoordinate(h);if(this.#n(u))continue;this.performanceStats.nodeProjectionCount++,l.x=s[h],l.y=a[h],l.z=o[h];const d=this.#x(l,u,t,e);s[h]=d.x,a[h]=d.y,o[h]=d.z}return}for(let s=n;s<this.rod.nodes.length;s++){const a=this.rod.nodes[s];if(a.pinned)continue;const o=this.insertedCoordinate(s);if(this.#n(o))continue;this.performanceStats.nodeProjectionCount++;const c=this.#x(a,o,t,e);$s(a,c)}}#O(t,e){const n={x:0,y:0,z:0},i=this.segmentLength*this.maxSegmentProjectionStep,s=this.#e(),a=this.rod.nodes.nodeStorage;if(a){const{x:o,y:c,z:l,pinned:h}=a;for(let u=s;u<this.rod.nodes.length-1;u++)if(!(h[u]&&h[u+1]))for(const d of this.segmentSamples){const f=this.insertedCoordinate(u+d);if(this.#n(f))continue;this.performanceStats.segmentSampleCount++;const g=h[u]?0:1-d,x=h[u+1]?0:d;n.x=o[u]*(1-d)+o[u+1]*d,n.y=c[u]*(1-d)+c[u+1]*d,n.z=l[u]*(1-d)+l[u+1]*d;const m=this.#x(n,f,t,e);let p=(m.x-n.x)*this.segmentProjectionBlend,_=(m.y-n.y)*this.segmentProjectionBlend,v=(m.z-n.z)*this.segmentProjectionBlend;const S=Math.hypot(p,_,v);if(S>i){const y=i/S;p*=y,_*=y,v*=y}const M=g*g+x*x;if(!(M<=1e-8)){if(this.performanceStats.segmentProjectionCount++,g){const y=g/M;o[u]+=p*y,c[u]+=_*y,l[u]+=v*y}if(x){const y=x/M;o[u+1]+=p*y,c[u+1]+=_*y,l[u+1]+=v*y}}}return}for(let o=s;o<this.rod.nodes.length-1;o++){const c=this.rod.nodes[o],l=this.rod.nodes[o+1];if(!(c.pinned&&l.pinned))for(const h of this.segmentSamples){const u=this.insertedCoordinate(o+h);if(this.#n(u))continue;this.performanceStats.segmentSampleCount++;const d=c.pinned?0:1-h,f=l.pinned?0:h;n.x=c.x*(1-h)+l.x*h,n.y=c.y*(1-h)+l.y*h,n.z=c.z*(1-h)+l.z*h;const g=this.#x(n,u,t,e);let x=(g.x-n.x)*this.segmentProjectionBlend,m=(g.y-n.y)*this.segmentProjectionBlend,p=(g.z-n.z)*this.segmentProjectionBlend;const _=Math.hypot(x,m,p);if(_>i){const S=i/_;x*=S,m*=S,p*=S}const v=d*d+f*f;if(!(v<=1e-8)){if(this.performanceStats.segmentProjectionCount++,d){const S=d/v;c.x+=x*S,c.y+=m*S,c.z+=p*S}if(f){const S=f/v;l.x+=x*S,l.y+=m*S,l.z+=p*S}}}}}#x(t,e,n,i){const s=n?.meshCollider||n?.lumenMeshCollider||null;if(s?.pointContact&&!this.#n(e)){let a=t;if(this.lumenSampler){const c=this.#I(t,e);a=c.projected,i&&(c.breach?this.#d(this.breachPoints,t):c.radialMargin<=bi&&this.#d(this.contactPoints,t))}const o=this.#R(s,a,this.meshClearance,!1,this._projectContact);return o?.violation&&o.target?(i&&(Number.isFinite(o.signedDistance)&&o.signedDistance<0?this.#d(this.breachPoints,a):this.#d(this.contactPoints,a)),{x:o.target.x,y:o.target.y,z:o.target.z}):(i&&Number.isFinite(o?.distance)&&o.distance<=bi&&this.#d(this.contactPoints,a),a)}return this.#B(t,e,i)}#M(t){return t?.meshCollider||t?.lumenMeshCollider||null}#R(t,e,n,i=!1,s=null){return i?this.performanceStats.diagnosticPointContactCount++:this.performanceStats.pointContactCount++,t.pointContact(e,n,s)}#_(t,e,n){if(!n?.pointContact)return{x:e.x,y:e.y,z:e.z};const i={x:t.x+e.x,y:t.y+e.y,z:t.z+e.z},s=this.#R(n,t,this.meshClearance,!1,this._slidePointContact),a=this.#R(n,i,this.meshClearance,!1,this._slideTargetContact),o=s?.violation||Number.isFinite(s?.signedDistance)&&s.signedDistance<=this.meshClearance+bi,c=a?.violation||Number.isFinite(a?.signedDistance)&&a.signedDistance<=this.meshClearance+bi;if(!o&&!c)return{x:e.x,y:e.y,z:e.z};const h=(a?.violation||c?a:s)?.normal||s?.normal||a?.normal,u=h?Math.hypot(h.x,h.y,h.z):0;if(u<1e-8)return{x:e.x,y:e.y,z:e.z};const d=h.x/u,f=h.y/u,g=h.z/u,x=e.x*d+e.y*f+e.z*g;return x<=0?{x:e.x,y:e.y,z:e.z}:{x:e.x-d*x,y:e.y-f*x,z:e.z-g*x}}#B(t,e,n){if(!this.lumenSampler)return t;const i=this.#I(t,e);return n&&(i.breach?this.#d(this.breachPoints,t):i.radialMargin<=bi&&this.#d(this.contactPoints,t)),i.projected}#I(t,e){const n=this.routeSample(e),i=Math.max(.5,(n.radius||1)-this.lumenClearance),s=n.tangent||this.sheathDir,a=Math.hypot(s.x,s.y,s.z),o=a<1e-8?this.sheathDir.x:s.x/a,c=a<1e-8?this.sheathDir.y:s.y/a,l=a<1e-8?this.sheathDir.z:s.z/a,h=t.x-n.point.x,u=t.y-n.point.y,d=t.z-n.point.z;let f=h*o+u*c+d*l,g=h-o*f,x=u-c*f,m=d-l*f,p=Math.hypot(g,x,m);const _=Math.max(this.segmentLength*.5,this.segmentLength*this.axialWindowScale),v=p>i+1e-4;if(p>i){const M=i/Math.max(1e-8,p);g*=M,x*=M,m*=M,p=i}f=It(f,-_,_);const S=this._lumenConstraintState;return S.projected.x=n.point.x+o*f+g,S.projected.y=n.point.y+c*f+x,S.projected.z=n.point.z+l*f+m,S.radialMargin=i-p,S.axialOffset=f,S.axialWindow=_,S.breach=v,S}#U(t,e,n=null){const i=1/Math.max(e,1e-6),s=n?.meshCollider||n?.lumenMeshCollider||null,a=this.rod.nodes.nodeStorage;if(a&&!s?.pointContact){const{x:o,y:c,z:l,vx:h,vy:u,vz:d}=a;for(let f=0;f<this.rod.nodes.length;f++){const g=o[f]-t[f].x,x=c[f]-t[f].y,m=l[f]-t[f].z,p=g*g+x*x+m*m>4e-4?.08:0;h[f]=g*i*p,u[f]=x*i*p,d[f]=m*i*p}return}for(let o=0;o<this.rod.nodes.length;o++){const c=this.rod.nodes[o],l=c.x-t[o].x,h=c.y-t[o].y,u=c.z-t[o].z,d=l*l+h*h+u*u>4e-4?.08:0;let f=l*i*d,g=h*i*d,x=u*i*d;const m=this.insertedCoordinate(o);if(s?.pointContact&&!this.#n(m)){const p=this.#R(s,c,this.meshClearance,!1,this._zeroVelocityContact),_=p?.normal;if((p?.violation||Number.isFinite(p?.signedDistance)&&p.signedDistance<=this.meshClearance+bi)&&_){const S=f*_.x+g*_.y+x*_.z;S>0&&(f-=_.x*S,g-=_.y*S,x-=_.z*S)}}c.vx=f,c.vy=g,c.vz=x}}#d(t,e,n=420){t.length>=n||t.push({x:e.x,y:e.y,z:e.z})}#n(t){return t<=this.sheathLength+ri}}const YS=16384,qS=64,ZS=20,jS=10,$S=30;function KS(r,{segmentLength:t=r.segmentLength,bodyBendingStiffness:e=YS,tipBendingStiffness:n=qS,softTipLength:i=ZS,bodyMaxBendAngle:s=jS,tipMaxBendAngle:a=$S}={}){const o=r.nodes.length-1;for(let c=0;c<r.nodes.length;c++){const h=(o-c)*t<i;r.nodes[c].bendingStiffness=h?n:e,r.nodes[c].bendAngleLimit=h?a:s}return r}const JS="OETCOLL1",Yh=8,ol=Yh+4,QS=1;new TextEncoder;const tM=new TextDecoder,eM={Float32Array,Uint32Array,Int16Array,Uint8Array,Int8Array};function md(r,t=8){return Math.ceil(r/t)*t}function nM(r){let t="";for(let e=0;e<Yh;e++)t+=String.fromCharCode(r[e]);return t}function dp(r){if(!(r instanceof ArrayBuffer))throw new TypeError("Collision asset must be an ArrayBuffer");if(r.byteLength<ol)throw new Error("Collision asset is truncated");const t=new Uint8Array(r),e=nM(t);if(e!==JS)throw new Error(`Unexpected collision asset magic: ${e}`);const n=new DataView(r).getUint32(Yh,!0),i=ol+n;if(i>r.byteLength)throw new Error("Collision asset manifest is truncated");const s=JSON.parse(tM.decode(t.subarray(ol,i)));if(s.version!==QS)throw new Error(`Unsupported collision asset version: ${s.version}`);const a={};let o=md(i);for(const c of s.sections||[]){const l=eM[c.type];if(!l)throw new Error(`Unsupported collision asset array type: ${c.type}`);const h=c.length*l.BYTES_PER_ELEMENT;if(o+h>r.byteLength)throw new Error(`Collision asset section is truncated: ${c.name}`);a[c.name]=new l(r,o,c.length),o=md(o+h)}return{metadata:s,arrays:a,buffer:r}}const Or=1e-8,Ki=0,Ho=1,cl=2,iM=3,sM=4,fp=5,hn=0,yn=1,fs=2;function Ks(r,t,e,n){return r.x=t,r.y=e,r.z=n,r}function gd(){const r=new Float64Array(6);return r[Ki]=-1/0,r[Ho]=1,r[fp]=-1,r}function pp(){return{inside:!1,signedDistance:-1/0,distance:1/0,inward:{x:1,y:0,z:0},normal:{x:-1,y:0,z:0},closestPoint:{x:0,y:0,z:0},lowerSliceIndex:-1,upperSliceIndex:-1}}class rM{constructor(t,e){this.metadata=t.lumen,this.sliceYs=e.lumenSliceYs,this.sliceContourOffsets=e.lumenSliceContourOffsets,this.contourPointOffsets=e.lumenContourPointOffsets,this.contourBounds=e.lumenContourBounds,this.contourSamples=e.lumenContourSamples,this.points=e.lumenPoints,this.pointQuantization=this.points instanceof Int16Array?this.metadata.pointQuantization||.02:1,this.axisBases=e.lumenAxisBases||new Float32Array([1,0,0,0,1,0,0,0,1]),this.axisSliceOffsets=e.lumenAxisSliceOffsets||new Uint32Array([0,this.sliceYs.length]),this.axisCount=Math.max(1,this.axisSliceOffsets.length-1),this._lower=gd(),this._upper=gd(),this._interval=new Float64Array(3),this._lastLower=new Int32Array(this.axisCount),this._lastUpper=new Int32Array(this.axisCount);for(let n=0;n<this.axisCount;n++){const i=this.axisSliceOffsets[n],s=this.axisSliceOffsets[n+1];this._lastLower[n]=i,this._lastUpper[n]=Math.min(i+1,Math.max(i,s-1))}}query(t,e=null){return this.queryCoordinates(t.x,t.y,t.z,e)}isInsideCoordinates(t,e,n){for(let i=0;i<this.axisCount;i++){const s=i*9,a=t*this.axisBases[s]+e*this.axisBases[s+1]+n*this.axisBases[s+2],o=t*this.axisBases[s+3]+e*this.axisBases[s+4]+n*this.axisBases[s+5],c=t*this.axisBases[s+6]+e*this.axisBases[s+7]+n*this.axisBases[s+8],l=this.#e(o,i);if(l[hn]<0||!this.#t(l[hn],a,c)&&(l[yn]===l[hn]||!this.#t(l[yn],a,c)))continue;const h=this.#r(l[hn],a,c,this._lower),u=l[yn]===l[hn]?h:this.#r(l[yn],a,c,this._upper),d=l[fs];if(h[Ki]*(1-d)+u[Ki]*d>=0)return!0}return!1}#t(t,e,n){const i=this.sliceContourOffsets[t],s=this.sliceContourOffsets[t+1];for(let a=i;a<s;a++){const o=a*4;if(e>=this.contourBounds[o]&&e<=this.contourBounds[o+1]&&n>=this.contourBounds[o+2]&&n<=this.contourBounds[o+3])return!0}return!1}queryCoordinates(t,e,n,i=null){const s=i||pp();if(!this.sliceYs.length)return s.inside=!1,s.signedDistance=-1/0,s.distance=1/0,Ks(s.inward,1,0,0),Ks(s.normal,-1,0,0),Ks(s.closestPoint,t,e,n),s.lowerSliceIndex=-1,s.upperSliceIndex=-1,s;let a=-1/0,o=1,c=0,l=0,h=-1,u=-1;for(let d=0;d<this.axisCount;d++){const f=d*9,g=t*this.axisBases[f]+e*this.axisBases[f+1]+n*this.axisBases[f+2],x=t*this.axisBases[f+3]+e*this.axisBases[f+4]+n*this.axisBases[f+5],m=t*this.axisBases[f+6]+e*this.axisBases[f+7]+n*this.axisBases[f+8],p=this.#e(x,d);if(p[hn]<0)continue;const _=this.#r(p[hn],g,m,this._lower),v=p[yn]===p[hn]?_:this.#r(p[yn],g,m,this._upper),S=p[fs],M=_[Ki]*(1-S)+v[Ki]*S;if(M<=a)continue;const y=Math.max(Or,Math.abs(this.sliceYs[p[yn]]-this.sliceYs[p[hn]])),w=p[yn]===p[hn]?0:Math.max(-.85,Math.min(.85,(v[Ki]-_[Ki])/y));let A=_[Ho]*(1-S)+v[Ho]*S,E=w,T=_[cl]*(1-S)+v[cl]*S;const L=Math.sqrt(A*A+E*E+T*T);L>Or?(A/=L,E/=L,T/=L):(A=1,E=0,T=0);let C=this.axisBases[f]*A+this.axisBases[f+3]*E+this.axisBases[f+6]*T,F=this.axisBases[f+1]*A+this.axisBases[f+4]*E+this.axisBases[f+7]*T,D=this.axisBases[f+2]*A+this.axisBases[f+5]*E+this.axisBases[f+8]*T;const N=Math.sqrt(C*C+F*F+D*D)||1;C/=N,F/=N,D/=N,a=M,o=C,c=F,l=D,h=p[hn],u=p[yn]}return s.inside=a>=0,s.signedDistance=a,s.distance=Math.abs(a),Ks(s.inward,o,c,l),Ks(s.normal,-o,-c,-l),Ks(s.closestPoint,t-o*a,e-c*a,n-l*a),s.lowerSliceIndex=h,s.upperSliceIndex=u,s}#e(t,e){const n=this._interval,i=this.axisSliceOffsets[e],s=this.axisSliceOffsets[e+1]-1;if(s<i)return n[hn]=-1,n[yn]=-1,n[fs]=0,n;if(s===i||t<=this.sliceYs[i])return this._lastLower[e]=i,this._lastUpper[e]=i,n[hn]=i,n[yn]=i,n[fs]=0,n;if(t>=this.sliceYs[s])return this._lastLower[e]=s,this._lastUpper[e]=s,n[hn]=s,n[yn]=s,n[fs]=0,n;let a=this._lastLower[e],o=this._lastUpper[e];if(a>=i&&o>a&&o<=s&&this.sliceYs[a]<=t&&t<=this.sliceYs[o]){const u=Math.max(Or,this.sliceYs[o]-this.sliceYs[a]);return n[hn]=a,n[yn]=o,n[fs]=(t-this.sliceYs[a])/u,n}let c=i,l=s;for(;l-c>1;){const u=Math.floor((c+l)*.5);this.sliceYs[u]<=t?c=u:l=u}a=c,o=l,this._lastLower[e]=a,this._lastUpper[e]=o;const h=Math.max(Or,this.sliceYs[o]-this.sliceYs[a]);return n[hn]=a,n[yn]=o,n[fs]=(t-this.sliceYs[a])/h,n}#r(t,e,n,i){let s=-1/0,a=1,o=0,c=e,l=n,h=-1;const u=this.sliceContourOffsets[t],d=this.sliceContourOffsets[t+1];for(let f=u;f<d;f++){const g=f*4;let x=0,m=0;if(e<this.contourBounds[g]?x=this.contourBounds[g]-e:e>this.contourBounds[g+1]&&(x=e-this.contourBounds[g+1]),n<this.contourBounds[g+2]?m=this.contourBounds[g+2]-n:n>this.contourBounds[g+3]&&(m=n-this.contourBounds[g+3]),Number.isFinite(s)&&s<0&&-Math.sqrt(x*x+m*m)<=s)continue;const p=this.contourPointOffsets[f],_=this.contourPointOffsets[f+1];if(_<=p)continue;let v=!1,S=e,M=n,y=1/0,w=_-1;for(let F=p;F<_;F++){const D=this.points[F*2]*this.pointQuantization,N=this.points[F*2+1]*this.pointQuantization,B=this.points[w*2]*this.pointQuantization,G=this.points[w*2+1]*this.pointQuantization;N>n!=G>n&&e<(B-D)*(n-N)/(G-N+1e-12)+D&&(v=!v);const z=B-D,H=G-N,j=z*z+H*H||1,$=Math.max(0,Math.min(1,((e-D)*z+(n-N)*H)/j)),Q=D+z*$,V=N+H*$,K=e-Q,nt=n-V,rt=K*K+nt*nt;rt<y&&(y=rt,S=Q,M=V),w=F}const A=Math.sqrt(y),E=v?A:-A;if(E<=s)continue;let T=v?e-S:S-e,L=v?n-M:M-n;const C=Math.sqrt(T*T+L*L);if(C>Or)T/=C,L/=C;else{T=this.contourSamples[f*2]-S,L=this.contourSamples[f*2+1]-M;const F=Math.sqrt(T*T+L*L)||1;T/=F,L/=F}s=E,a=T,o=L,c=S,l=M,h=f}return i[Ki]=s,i[Ho]=a,i[cl]=o,i[iM]=c,i[sM]=l,i[fp]=h,i}}function aM(r,t){return!r?.lumen||!t?.lumenSliceYs?.length?null:new rM(r,t)}const oM="centerline-safe-core",cM="sparse-sdf",lM="sparse-sdf-bvh",qh="fallback",hM="centerline-estimate",Jn=1e-8,da=1<<17,mp=da>>1,uM=mp-1,ll=200,Gr=65535,xd=0,_d=1,po=2,vd=3,mo=4,Sd=5,Md=6,yd=7,Ed=8,wd=9,hl=10,ul=11,Td=12,Ad=13,go=14,dM=15,Cd=0,Vr=1,fM=2,Rd=3,bd=4,Pd=5,xo=6,kr=0,Ld=1,Hr=2,Wr=3,dl=4,fl=5,pl=6,ml=7,Xr=8,Yr=9,qr=10,Zr=11,Ln=0,ai=1,oi=2,ci=3,jr=4;function pM(r,t,e){return Math.max(t,Math.min(e,r))}function li(r,t){r.inside=t.inside,r.violation=t.violation,r.conservative=t.conservative,r.source=t.source;const e=r.values,n=t.values;return e[0]=n[0],e[1]=n[1],e[2]=n[2],e[3]=n[3],e[4]=n[4],e[5]=n[5],e[6]=n[6],$r(r.point.values,t.point.values),$r(r.target.values,t.target.values),$r(r.closestPoint.values,t.closestPoint.values),$r(r.normal.values,t.normal.values),$r(r.inward.values,t.inward.values),r}function $r(r,t){r[0]=t[0],r[1]=t[1],r[2]=t[2]}class Kr{constructor(t=0,e=0,n=0){this.values=new Float64Array([t,e,n])}get x(){return this.values[0]}set x(t){this.values[0]=t}get y(){return this.values[1]}set y(t){this.values[1]=t}get z(){return this.values[2]}set z(t){this.values[2]=t}}class mM{constructor(){this.values=new Float64Array([-1/0,-1/0,1/0,1/0,-1,0,1]),this.inside=!1,this.violation=!1,this.conservative=!1,this.source=qh,this.point=new Kr,this.target=new Kr,this.closestPoint=new Kr,this.normal=new Kr(1,0,0),this.inward=new Kr(1,0,0)}get signedDistance(){return this.values[0]}set signedDistance(t){this.values[0]=t}get signedGap(){return this.values[1]}set signedGap(t){this.values[1]=t}get distance(){return this.values[2]}set distance(t){this.values[2]=t}get penetration(){return this.values[3]}set penetration(t){this.values[3]=t}get branchId(){return this.values[4]}set branchId(t){this.values[4]=t}get segmentT(){return this.values[5]}set segmentT(t){this.values[5]=t}get timeOfImpact(){return this.values[6]}set timeOfImpact(t){this.values[6]=t}}class gM{constructor(){this.values=new Float64Array(12),this.found=!1,this.branchId=-1,this.signedDistance=-1/0,this.safeDistance=-1/0,this.safeBranchId=-1,this.safeInwardX=1,this.nearestDistance=1/0,this.inwardX=1}get branchId(){return this.values[0]}set branchId(t){this.values[0]=t}get t(){return this.values[1]}set t(t){this.values[1]=t}get signedDistance(){return this.values[2]}set signedDistance(t){this.values[2]=t}get safeDistance(){return this.values[3]}set safeDistance(t){this.values[3]=t}get safeBranchId(){return this.values[4]}set safeBranchId(t){this.values[4]=t}get safeInwardX(){return this.values[5]}set safeInwardX(t){this.values[5]=t}get safeInwardY(){return this.values[6]}set safeInwardY(t){this.values[6]=t}get safeInwardZ(){return this.values[7]}set safeInwardZ(t){this.values[7]=t}get nearestDistance(){return this.values[8]}set nearestDistance(t){this.values[8]=t}get inwardX(){return this.values[9]}set inwardX(t){this.values[9]=t}get inwardY(){return this.values[10]}set inwardY(t){this.values[10]=t}get inwardZ(){return this.values[11]}set inwardZ(t){this.values[11]=t}}class xM{constructor(){this.values=new Float64Array([-1/0,1,0,0,-1]),this.conservative=!1,this.source=qh}get signedDistance(){return this.values[0]}set signedDistance(t){this.values[0]=t}get inwardX(){return this.values[1]}set inwardX(t){this.values[1]=t}get inwardY(){return this.values[2]}set inwardY(t){this.values[2]=t}get inwardZ(){return this.values[3]}set inwardZ(t){this.values[3]=t}get branchId(){return this.values[4]}set branchId(t){this.values[4]=t}}function ei(){return new mM}function _M(){return new Uint32Array(dM)}class vM{constructor(t,{fallbackCollider:e=null,fallbackGeometry:n=null,bvhValidationDistance:i=.85,capsuleBvhValidation:s=!0}={}){const a=t instanceof ArrayBuffer?dp(t):t;if(!a?.metadata||!a?.arrays)throw new TypeError("Decoded collision asset is required");this.metadata=a.metadata,this.arrays=a.arrays,this.fallbackCollider=e,this.fallbackGeometry=n,this.bvhValidationDistance=i,this.capsuleBvhValidationGap=s===!0?.05:Number.isFinite(s)?s:-1/0,this.packedLumenField=aM(a.metadata,a.arrays),this.centerline=a.arrays.centerlineSegments,this.centerlineStride=a.metadata.centerline.stride,this.broadPhaseOffsets=a.arrays.broadPhaseOffsets,this.broadPhaseIds=a.arrays.broadPhaseIds,this.sdfBrickKeys=a.arrays.sdfBrickKeys,this.sdfDistances=a.arrays.sdfDistances,this.sdfInsideBits=a.arrays.sdfInsideBits||null;const o=a.metadata.sdf;this.voxelSize=o.voxelSize,this.brickSize=o.brickSize,this.valuesPerBrick=this.brickSize**3,this.sdfQuantization=o.distanceQuantization??o.quantization,this.sdfOrigin=o.origin,this.sdfDimensions=o.dimensions;const c=this.sdfDimensions[0]*this.sdfDimensions[1]*this.sdfDimensions[2];if(this.sdfBrickKeys.length>=Gr)throw new RangeError("Sparse SDF has too many bricks for its runtime lookup");this.sdfBrickLookup=new Uint16Array(c),this.sdfBrickLookup.fill(Gr);for(let h=0;h<this.sdfBrickKeys.length;h++)this.sdfBrickLookup[this.sdfBrickKeys[h]]=h;this.signCacheKeyLow=new Int32Array(da),this.signCacheKeyHigh=new Int32Array(da),this.signCacheInside=new Uint8Array(da),this.signCacheValid=new Uint8Array(da),this.signCacheVictim=new Uint8Array(mp);const l=a.metadata.broadPhase;this.broadPhaseOrigin=l.origin,this.broadPhaseDimensions=l.dimensions,this.broadPhaseCellSize=l.cellSize,this._sdfCornerScratch=new Float64Array(8),this._capsuleCoordinateScratch=new Float64Array(7),this._centerlineQueryScratch=new Float64Array(3),this.runtimeBytes=a.metadata.decodedBytes+this.sdfBrickLookup.byteLength+this.signCacheKeyLow.byteLength+this.signCacheKeyHigh.byteLength+this.signCacheInside.byteLength+this.signCacheValid.byteLength+this.signCacheVictim.byteLength+this._sdfCornerScratch.byteLength+this._capsuleCoordinateScratch.byteLength+this._centerlineQueryScratch.byteLength,this.stats=_M(),this._centerlineState=new gM,this._distanceState=new xM,this._point={x:0,y:0,z:0},this._skipBvhValidation=!1,this._bvhPoint=new R,this._bvhClosest={point:new R,distance:1/0,faceIndex:-1},this._lumenQuery=pp(),this._capsuleContact=ei(),this._capsuleEndpointContact=ei(),this._capsuleEndpointX=NaN,this._capsuleEndpointY=NaN,this._capsuleEndpointZ=NaN,this._capsuleEndpointRadius=NaN,this._sweepContact=ei(),this._sweepProbe=ei(),this._fallbackContact={query:{inward:{x:0,y:0,z:0},normal:{x:0,y:0,z:0},closestPoint:{x:0,y:0,z:0}},target:{x:0,y:0,z:0},closestPoint:{x:0,y:0,z:0},inward:{x:0,y:0,z:0},normal:{x:0,y:0,z:0}}}resetStats(){this.stats.fill(0)}getStats(){const t=this.stats;return{pointQueries:t[xd],capsuleQueries:t[_d],capsuleSamples:t[po],sweepQueries:t[vd],sweepSamples:t[mo],batchQueries:t[Sd],safeCoreHits:t[Md],sdfHits:t[yd],bvhRefinements:t[Ed],signRefinements:t[wd],signCacheHits:t[hl],signCacheMisses:t[ul],fallbackHits:t[Td],centerlineEstimateHits:t[Ad],resultAllocations:t[go],runtimeBytes:this.runtimeBytes}}setFallbackCollider(t){this.fallbackCollider=t}setFallbackGeometry(t){this.fallbackGeometry=t}querySphere(t,e=0,n=null){n||this.stats[go]++;const i=n||ei(),s=t.x??t[0]??0,a=t.y??t[1]??0,o=t.z??t[2]??0;return this.#t(s,a,o,Math.max(0,e||0),i)}#t(t,e,n,i,s){this.stats[xd]++;const a=this.#r(t,e,n,i),o=a.values,c=o[Ln],l=c-i,h=Math.max(0,-l),u=l<0,d=o[ai],f=o[oi],g=o[ci],x=s.values;s.inside=c>=0,s.violation=u,s.conservative=a.conservative,x[Cd]=c,x[Vr]=l,x[fM]=Math.max(0,c),x[Rd]=h,x[bd]=o[jr],x[Pd]=0,x[xo]=u?0:1,s.source=a.source;const m=s.point.values;m[0]=t,m[1]=e,m[2]=n;const p=s.normal.values;p[0]=d,p[1]=f,p[2]=g;const _=s.inward.values;_[0]=d,_[1]=f,_[2]=g;const v=s.closestPoint.values;v[0]=t-d*c,v[1]=e-f*c,v[2]=n-g*c;const S=s.target.values;return S[0]=t+d*h,S[1]=e+f*h,S[2]=n+g*h,s}queryCapsule(t,e,n=0,i=null){const s=t.x??t[0]??0,a=t.y??t[1]??0,o=t.z??t[2]??0,c=e.x??e[0]??0,l=e.y??e[1]??0,h=e.z??e[2]??0;return this.queryCapsuleCoordinates(s,a,o,c,l,h,n,i)}queryCapsuleCoordinates(t,e,n,i,s,a,o=0,c=null){const l=this._capsuleCoordinateScratch;return l[0]=t,l[1]=e,l[2]=n,l[3]=i,l[4]=s,l[5]=a,l[6]=o,this.#e(c)}queryCapsuleSoA(t,e,n,i,s,a=null){const o=this._capsuleCoordinateScratch;return o[0]=t[s],o[1]=e[s],o[2]=n[s],o[3]=t[s+1],o[4]=e[s+1],o[5]=n[s+1],o[6]=Math.max(i[s],i[s+1]),this.#e(a)}#e(t){const e=this._capsuleCoordinateScratch,n=e[0],i=e[1],s=e[2],a=e[3],o=e[4],c=e[5],l=e[6];t||this.stats[go]++;const h=t||ei(),u=Math.max(0,l||0),d=a-n,f=o-i,g=c-s,x=Math.sqrt(d*d+f*f+g*g),m=Math.max(this.voxelSize*4,Math.max(.5,u)),p=Math.max(1,Math.ceil(x/m));let _=1/0,v=0,S=0;this.stats[_d]++,this._skipBvhValidation=!0;const M=n===this._capsuleEndpointX&&i===this._capsuleEndpointY&&s===this._capsuleEndpointZ&&u===this._capsuleEndpointRadius;let y;M?y=li(this._capsuleContact,this._capsuleEndpointContact):(y=this.#t(n,i,s,u,this._capsuleContact),this.stats[po]++),_=y.values[Vr];const w=_,A=y.inward.values[0],E=y.inward.values[1],T=y.inward.values[2];let L=w,C=1;li(h,y),x>Jn&&(y=this.#t(a,o,c,u,this._capsuleContact),this.stats[po]++,li(this._capsuleEndpointContact,y),this._capsuleEndpointX=a,this._capsuleEndpointY=o,this._capsuleEndpointZ=c,this._capsuleEndpointRadius=u,L=y.values[Vr],C=A*y.inward.values[0]+E*y.inward.values[1]+T*y.inward.values[2],L<_&&(_=L,v=1,S=p,li(h,y))),x<=Jn&&(li(this._capsuleEndpointContact,y),this._capsuleEndpointX=a,this._capsuleEndpointY=o,this._capsuleEndpointZ=c,this._capsuleEndpointRadius=u);const F=p>1&&(Math.min(w,L)<=this.voxelSize||C<.85);for(let D=1;F&&D<p;D++){const N=D/p;y=this.#t(n+d*N,i+f*N,s+g*N,u,this._capsuleContact),this.stats[po]++;const B=y.values[Vr];B<_&&(_=B,v=N,S=D,li(h,y))}return this._skipBvhValidation=!1,_<=this.capsuleBvhValidationGap&&(v=S/p,li(h,this.#t(n+d*v,i+f*v,s+g*v,u,this._capsuleContact))),h.values[Pd]=v,h}sweepSphere(t,e,n=0,i=null){i||this.stats[go]++;const s=i||ei(),a=t.x??t[0]??0,o=t.y??t[1]??0,c=t.z??t[2]??0,l=e.x??e[0]??0,h=e.y??e[1]??0,u=e.z??e[2]??0,d=l-a,f=h-o,g=u-c,x=Math.sqrt(d*d+f*f+g*g),m=Math.max(this.voxelSize*.5,Math.max(.1,n*.5)),p=Math.max(1,Math.ceil(x/m));this.stats[vd]++,this._point.x=a,this._point.y=o,this._point.z=c;let _=this.#t(this._point.x,this._point.y,this._point.z,Math.max(0,n||0),this._sweepContact);if(this.stats[mo]++,_.violation)return li(s,_),s.values[xo]=0,s;for(let v=1;v<=p;v++){const S=v/p;this._point.x=a+d*S,this._point.y=o+f*S,this._point.z=c+g*S;const M=this.#t(this._point.x,this._point.y,this._point.z,Math.max(0,n||0),this._sweepProbe);if(this.stats[mo]++,!M.violation){const A=_;_=M,this._sweepProbe=A;continue}let y=(v-1)/p,w=S;for(let A=0;A<7;A++){const E=(y+w)*.5;this._point.x=a+d*E,this._point.y=o+f*E,this._point.z=c+g*E;const T=this.#t(this._point.x,this._point.y,this._point.z,Math.max(0,n||0),this._sweepContact);this.stats[mo]++,T.violation?w=E:y=E}return this._point.x=a+d*w,this._point.y=o+f*w,this._point.z=c+g*w,li(s,this.#t(this._point.x,this._point.y,this._point.z,Math.max(0,n||0),this._sweepContact)),s.values[xo]=w,s}return this._point.x=l,this._point.y=h,this._point.z=u,li(s,this.#t(this._point.x,this._point.y,this._point.z,Math.max(0,n||0),this._sweepContact)),s.values[xo]=1,s}queryBatch(t,e,n,i){if(!i||i.signedGaps.length<n)throw new RangeError("Preallocated batch contact output is too small");this.stats[Sd]++;const s=this._capsuleContact;for(let a=0;a<n;a++){const o=a*3;this._point.x=t[o],this._point.y=t[o+1],this._point.z=t[o+2],this.#t(this._point.x,this._point.y,this._point.z,Math.max(0,e[a]||0),s),i.signedDistances[a]=s.values[Cd],i.signedGaps[a]=s.values[Vr],i.penetrations[a]=s.values[Rd],i.normals[o]=s.normal.values[0],i.normals[o+1]=s.normal.values[1],i.normals[o+2]=s.normal.values[2],i.targets[o]=s.target.values[0],i.targets[o+1]=s.target.values[1],i.targets[o+2]=s.target.values[2],i.branchIds[a]=s.values[bd],i.violations[a]=s.violation?1:0}return i.count=n,i}#r(t,e,n,i){const s=this.#f(t,e,n),a=s.values,o=this._distanceState,c=o.values;if(s.found&&a[Wr]>i+this.voxelSize*.25)return c[Ln]=a[Wr],c[ai]=a[fl],c[oi]=a[pl],c[ci]=a[ml],c[jr]=a[dl],o.conservative=!0,o.source=oM,this.stats[Md]++,o;if(this.#c(t,e,n,o))return Math.sqrt(c[ai]*c[ai]+c[oi]*c[oi]+c[ci]*c[ci])<Jn&&s.found&&(c[ai]=a[Yr],c[oi]=a[qr],c[ci]=a[Zr]),c[jr]=a[kr],o.conservative=!1,o.source=cM,this.stats[yd]++,this._skipBvhValidation||this.#h(t,e,n,i,o),o;if(this.fallbackCollider?.pointContact){this._point.x=t,this._point.y=e,this._point.z=n;const l=this.fallbackCollider.pointContact(this._point,0,this._fallbackContact);if(Number.isFinite(l?.signedDistance)){const h=l.inward||l.normal,u=h?.x||0,d=h?.y||0,f=h?.z||0,g=Math.sqrt(u*u+d*d+f*f);if(c[Ln]=l.signedDistance,g>Jn){const x=l.inward?1:-1;c[ai]=h.x/g*x,c[oi]=h.y/g*x,c[ci]=h.z/g*x}else c[ai]=a[Yr],c[oi]=a[qr],c[ci]=a[Zr];return c[jr]=a[kr],o.conservative=!1,o.source=qh,this.stats[Td]++,o}}return c[Ln]=a[Hr],c[ai]=a[Yr],c[oi]=a[qr],c[ci]=a[Zr],c[jr]=a[kr],o.conservative=!0,o.source=hM,this.stats[Ad]++,o}#f(t,e,n){const i=this._centerlineQueryScratch;i[0]=t,i[1]=e,i[2]=n;const s=this._centerlineState,a=s.values;s.found=!1,a[kr]=-1,a[Ld]=0,a[Hr]=-1/0,a[Wr]=-1/0,a[dl]=-1,a[fl]=1,a[pl]=0,a[ml]=0,a[Xr]=1/0,a[Yr]=1,a[qr]=0,a[Zr]=0;const o=Math.floor((t-this.broadPhaseOrigin[0])/this.broadPhaseCellSize),c=Math.floor((e-this.broadPhaseOrigin[1])/this.broadPhaseCellSize),l=Math.floor((n-this.broadPhaseOrigin[2])/this.broadPhaseCellSize);if(o>=0&&c>=0&&l>=0&&o<this.broadPhaseDimensions[0]&&c<this.broadPhaseDimensions[1]&&l<this.broadPhaseDimensions[2]){const h=o+this.broadPhaseDimensions[0]*(c+this.broadPhaseDimensions[1]*l),u=this.broadPhaseOffsets[h],d=this.broadPhaseOffsets[h+1];for(let f=u;f<d;f++)this.#o(this.broadPhaseIds[f],s)}if(!s.found){const h=this.centerline.length/this.centerlineStride;for(let u=0;u<h;u++)this.#o(u,s)}return s}#o(t,e){const n=this._centerlineQueryScratch,i=n[0],s=n[1],a=n[2],o=e.values,c=t*this.centerlineStride,l=this.centerline[c],h=this.centerline[c+1],u=this.centerline[c+2],d=this.centerline[c+3]-l,f=this.centerline[c+4]-h,g=this.centerline[c+5]-u,x=d*d+f*f+g*g,m=pM(((i-l)*d+(s-h)*f+(a-u)*g)/Math.max(Jn,x),0,1),p=l+d*m,_=h+f*m,v=u+g*m,S=p-i,M=_-s,y=v-a,w=Math.sqrt(S*S+M*M+y*y),A=this.centerline[c+6]*(1-m)+this.centerline[c+7]*m,E=this.centerline[c+8],T=A-w,L=E-w;o[Xr]=Math.min(o[Xr],w);let C,F,D;if(w>Jn)C=S/w,F=M/w,D=y/w;else{const N=Math.sqrt(x),B=N>Jn?d/N:0,G=N>Jn?f/N:1,z=N>Jn?g/N:0,H=Math.abs(G)<.85?0:1,j=Math.abs(G)<.85?1:0,$=-z*j,Q=z*H,V=B*j-G*H,K=Math.sqrt($*$+Q*Q+V*V)||1;C=$/K,F=Q/K,D=V/K}L>o[Wr]&&(o[Wr]=L,o[dl]=t,o[fl]=C,o[pl]=F,o[ml]=D),!(e.found&&T<=o[Hr])&&(e.found=!0,o[kr]=t,o[Ld]=m,o[Hr]=T,o[Yr]=C,o[qr]=F,o[Zr]=D)}#c(t,e,n,i){const s=i.values,a=this._centerlineState.values,o=this._sdfCornerScratch,c=(t-this.sdfOrigin[0])/this.voxelSize,l=(e-this.sdfOrigin[1])/this.voxelSize,h=(n-this.sdfOrigin[2])/this.voxelSize,u=Math.floor(c),d=Math.floor(l),f=Math.floor(h),g=c-u,x=l-d,m=h-f,p=this.brickSize,_=Math.floor(u/p),v=Math.floor(d/p),S=Math.floor(f/p),M=u-_*p,y=d-v*p,w=f-S*p,A=M>=0&&y>=0&&w>=0&&M+1<p&&y+1<p&&w+1<p&&_>=0&&v>=0&&S>=0&&_<this.sdfDimensions[0]&&v<this.sdfDimensions[1]&&S<this.sdfDimensions[2];let E=-1;if(A){const rt=_+this.sdfDimensions[0]*(v+this.sdfDimensions[1]*S),ot=this.sdfBrickLookup[rt];if(ot!==Gr){const _t=p,ht=p*p;E=ot*this.valuesPerBrick+M+_t*y+ht*w;const ut=this.sdfDistances,Ct=this.sdfQuantization;o[0]=ut[E]*Ct,o[1]=ut[E+1]*Ct,o[2]=ut[E+_t]*Ct,o[3]=ut[E+_t+1]*Ct,o[4]=ut[E+ht]*Ct,o[5]=ut[E+ht+1]*Ct,o[6]=ut[E+ht+_t]*Ct,o[7]=ut[E+ht+_t+1]*Ct}}if(E<0&&(this.#a(o,0,u,d,f),this.#a(o,1,u+1,d,f),this.#a(o,2,u,d+1,f),this.#a(o,3,u+1,d+1,f),this.#a(o,4,u,d,f+1),this.#a(o,5,u+1,d,f+1),this.#a(o,6,u,d+1,f+1),this.#a(o,7,u+1,d+1,f+1)),!Number.isFinite(o[0])||!Number.isFinite(o[1])||!Number.isFinite(o[2])||!Number.isFinite(o[3])||!Number.isFinite(o[4])||!Number.isFinite(o[5])||!Number.isFinite(o[6])||!Number.isFinite(o[7]))return!1;const T=o[0]+(o[1]-o[0])*g,L=o[2]+(o[3]-o[2])*g,C=o[4]+(o[5]-o[4])*g,F=o[6]+(o[7]-o[6])*g,D=T+(L-T)*x,N=C+(F-C)*x,B=D+(N-D)*m;let G;if(this.sdfInsideBits){let rt,ot,_t,ht,ut,Ct,W,Nt;if(E>=0){const X=p,et=p*p,J=this.sdfInsideBits,tt=E+1,St=E+X,lt=St+1,gt=E+et,At=gt+1,Ht=gt+X,it=Ht+1;rt=(J[E>>3]&1<<(E&7))!==0?1:0,ot=(J[tt>>3]&1<<(tt&7))!==0?1:0,_t=(J[St>>3]&1<<(St&7))!==0?1:0,ht=(J[lt>>3]&1<<(lt&7))!==0?1:0,ut=(J[gt>>3]&1<<(gt&7))!==0?1:0,Ct=(J[At>>3]&1<<(At&7))!==0?1:0,W=(J[Ht>>3]&1<<(Ht&7))!==0?1:0,Nt=(J[it>>3]&1<<(it&7))!==0?1:0}else rt=this.#i(u,d,f),ot=this.#i(u+1,d,f),_t=this.#i(u,d+1,f),ht=this.#i(u+1,d+1,f),ut=this.#i(u,d,f+1),Ct=this.#i(u+1,d,f+1),W=this.#i(u,d+1,f+1),Nt=this.#i(u+1,d+1,f+1);const Tt=rt+(ot-rt)*g,Pt=_t+(ht-_t)*g,yt=ut+(Ct-ut)*g,Kt=W+(Nt-W)*g,bt=Tt+(Pt-Tt)*x,I=yt+(Kt-yt)*x,b=rt+ot+_t+ht+ut+Ct+W+Nt;b>0&&b<8&&this.packedLumenField?(G=this.#T(t,e,n)?1:-1,this.stats[wd]++):G=bt+(I-bt)*m>=.5?1:-1,s[Ln]=B*G}else G=(this.packedLumenField?this.packedLumenField.queryCoordinates(t,e,n,this._lumenQuery).signedDistance:a[Hr])>=0?1:-1,s[Ln]=B*G;const z=(o[1]-o[0])*(1-x)+(o[3]-o[2])*x,H=(o[5]-o[4])*(1-x)+(o[7]-o[6])*x,j=(o[2]-o[0])*(1-g)+(o[3]-o[1])*g,$=(o[6]-o[4])*(1-g)+(o[7]-o[5])*g;let Q=(z*(1-m)+H*m)/this.voxelSize,V=(j*(1-m)+$*m)/this.voxelSize,K=(N-D)/this.voxelSize;const nt=Math.sqrt(Q*Q+V*V+K*K);return nt>Jn&&(Q/=nt,V/=nt,K/=nt),s[Ln]<0&&(a[Xr]<=.001||a[Xr]+.2<-s[Ln])&&(s[Ln]=-s[Ln],G=-G),s[ai]=Q*G,s[oi]=V*G,s[ci]=K*G,!0}#T(t,e,n){const i=Math.round((t-this.sdfOrigin[0])*ll),s=Math.round((e-this.sdfOrigin[1])*ll),a=Math.round((n-this.sdfOrigin[2])*ll);if(i<0||i>65535||s<0||s>131071||a<0||a>65535)return this.stats[ul]++,this.packedLumenField.isInsideCoordinates(t,e,n);const o=i&65535|(s&65535)<<16,c=s>>>16|a<<1,l=(Math.imul(i,73856093)^Math.imul(s,19349663)^Math.imul(a,83492791))&uM,h=l<<1,u=h+1;if(this.signCacheValid[h]&&this.signCacheKeyLow[h]===o&&this.signCacheKeyHigh[h]===c)return this.signCacheVictim[l]=1,this.stats[hl]++,this.signCacheInside[h]!==0;if(this.signCacheValid[u]&&this.signCacheKeyLow[u]===o&&this.signCacheKeyHigh[u]===c)return this.signCacheVictim[l]=0,this.stats[hl]++,this.signCacheInside[u]!==0;const d=this.packedLumenField.isInsideCoordinates(t,e,n);let f;return this.signCacheValid[h]?this.signCacheValid[u]?f=h+this.signCacheVictim[l]:f=u:f=h,this.signCacheKeyLow[f]=o,this.signCacheKeyHigh[f]=c,this.signCacheInside[f]=d?1:0,this.signCacheValid[f]=1,this.signCacheVictim[l]=f===h?1:0,this.stats[ul]++,d}#h(t,e,n,i,s){const a=this.fallbackGeometry?.boundsTree,o=s.values,c=o[Ln]-i,l=i>0?Math.min(this.bvhValidationDistance,.25):this.bvhValidationDistance;if(!a||Math.abs(c)>l&&(i<=0||c>=-.2))return!1;this._bvhPoint.set(t,e,n),this._bvhClosest.distance=1/0;const u=a.closestPointToPoint(this._bvhPoint,this._bvhClosest)?.distance??this._bvhPoint.distanceTo(this._bvhClosest.point);if(!Number.isFinite(u))return!1;const d=o[Ln]>=0?1:-1;return o[Ln]=u*d,u>Jn&&(o[ai]=(t-this._bvhClosest.point.x)/u*d,o[oi]=(e-this._bvhClosest.point.y)/u*d,o[ci]=(n-this._bvhClosest.point.z)/u*d),s.source=lM,this.stats[Ed]++,!0}#a(t,e,n,i,s){if(n<0||i<0||s<0){t[e]=NaN;return}const a=Math.floor(n/this.brickSize),o=Math.floor(i/this.brickSize),c=Math.floor(s/this.brickSize);if(a>=this.sdfDimensions[0]||o>=this.sdfDimensions[1]||c>=this.sdfDimensions[2]){t[e]=NaN;return}const l=a+this.sdfDimensions[0]*(o+this.sdfDimensions[1]*c),h=this.sdfBrickLookup[l];if(h===Gr){t[e]=NaN;return}const u=n-a*this.brickSize,d=i-o*this.brickSize,f=s-c*this.brickSize,g=u+this.brickSize*(d+this.brickSize*f),x=h*this.valuesPerBrick+g;t[e]=this.sdfDistances[x]*this.sdfQuantization}#i(t,e,n){if(t<0||e<0||n<0)return 0;const i=Math.floor(t/this.brickSize),s=Math.floor(e/this.brickSize),a=Math.floor(n/this.brickSize);if(i>=this.sdfDimensions[0]||s>=this.sdfDimensions[1]||a>=this.sdfDimensions[2])return 0;const o=i+this.sdfDimensions[0]*(s+this.sdfDimensions[1]*a),c=this.sdfBrickLookup[o];if(c===Gr)return 0;const l=t-i*this.brickSize,h=e-s*this.brickSize,u=n-a*this.brickSize,d=l+this.brickSize*(h+this.brickSize*u),f=c*this.valuesPerBrick+d;return(this.sdfInsideBits[f>>3]&1<<(f&7))!==0?1:0}}const SM=25.4,gp=1/3,xp=.035,_p=xp*SM,Mr=_p/2,MM=Mr*.5,vp=6,Sp=vp*gp,yM=Sp/2,Mp=1.8,yp=Mp/2,Ep=5,wp=Ep*gp,Qo=wp/2,Tp=.97,Ap=Tp/2,Cp=Qo*.78,Yt=1e-8,EM=1/120,Dd=1,_o=3,Id=4,Nd=5,Fd=16,Bd=.01;function ae(r,t,e){return Math.max(t,Math.min(e,r))}function ge(r,t,e){return Math.sqrt(r*r+t*t+e*e)}function Gn(){return globalThis.performance?.now?.()??Date.now()}function wM(r,t,e){if(!t)return 0;const n=Array.from(r.subarray(0,t));return n.sort((i,s)=>i-s),n[Math.min(n.length-1,Math.floor((n.length-1)*e))]}function Jr(r=512){return{samples:new Float32Array(r),cursor:0,count:0,recordedCount:0,total:0,last:0}}function Qr(r,t){r.last=t,r.total+=t,r.recordedCount++,r.samples[r.cursor]=t,r.cursor=(r.cursor+1)%r.samples.length,r.count=Math.min(r.samples.length,r.count+1)}function ta(r){return{lastMs:r.last,averageMs:r.recordedCount?r.total/r.recordedCount:0,p95Ms:wM(r.samples,r.count,.95)}}const vc=Object.freeze({guidewire:Object.freeze({id:"guidewire",radius:Mr,mass:1,stretchCompliance:2e-7,bendCompliance:2e-5,minBendComplianceScale:.001953125,maxBendAngle:135,foldLimitStrength:1,wallFriction:.006,linearDamping:.98,bendDamping:.06}),catheter:Object.freeze({id:"catheter",outerRadius:Qo,innerDiameter:Tp,innerRadius:Ap,radius:Qo,mass:1.4,stretchCompliance:1e-7,bendCompliance:1e-5,shapeCompliance:1e-4,maxBendAngle:35,foldLimitStrength:1,wallFriction:.06,lumenFriction:.04,linearDamping:.9,bendDamping:.68,maxSpeed:40,postStabilizationPasses:4}),sheath:Object.freeze({id:"sheath",outerRadius:1,innerDiameter:Mp,innerRadius:yp})});class Ud{constructor(t,e,n,i={}){if(!Number.isInteger(e)||e<2)throw new RangeError("A rod requires at least two nodes");this.id=t,this.count=e,this.segmentCount=e-1,this.segmentLength=n,this.radius=i.radius??.5,this.innerRadius=i.innerRadius??0,this.mass=i.mass??1,this.stretchCompliance=i.stretchCompliance??2e-7,this.bendCompliance=i.bendCompliance??.001,this.minBendComplianceScale=i.minBendComplianceScale??.125,this.shapeCompliance=i.shapeCompliance??5e-5,this.maxBendAngle=i.maxBendAngle??135,this.foldLimitStrength=i.foldLimitStrength??.7,this.wallCompliance=i.wallCompliance??0,this.wallFriction=i.wallFriction??.08,this.lumenFriction=i.lumenFriction??.04,this.linearDamping=i.linearDamping??.98,this.bendDamping=ae(i.bendDamping??0,0,1),this.maxSpeed=i.maxSpeed??1/0,this.postStabilizationPasses=Math.max(0,Math.floor(i.postStabilizationPasses??0)),this.sleepVelocity=i.sleepVelocity??.015,this.sleepFrames=i.sleepFrames??120,this.activeStart=0,this.activeEnd=e-1,this.collisionStartSegment=0,this.collisionEndSegment=e-2,this.sleepCounter=0,this.sleeping=!1,this.x=new Float32Array(e),this.y=new Float32Array(e),this.z=new Float32Array(e),this.previousX=new Float32Array(e),this.previousY=new Float32Array(e),this.previousZ=new Float32Array(e),this.velocityX=new Float32Array(e),this.velocityY=new Float32Array(e),this.velocityZ=new Float32Array(e),this.forceX=new Float32Array(e),this.forceY=new Float32Array(e),this.forceZ=new Float32Array(e),this.inverseMass=new Float32Array(e),this.nodeRadius=new Float32Array(e),this.pinned=new Uint8Array(e),this.controlEnabled=new Uint8Array(e),this.controlX=new Float32Array(e),this.controlY=new Float32Array(e),this.controlZ=new Float32Array(e),this.controlCompliance=new Float32Array(e),this.restShapeEnabled=new Uint8Array(e),this.restShapeX=new Float32Array(e),this.restShapeY=new Float32Array(e),this.restShapeZ=new Float32Array(e),this.restShapeCompliance=new Float32Array(e),this.restLength=new Float32Array(this.segmentCount),this.restBendChord=new Float32Array(e),this.lengthLambda=new Float32Array(this.segmentCount),this.lengthNormalX=new Float32Array(this.segmentCount),this.lengthNormalY=new Float32Array(this.segmentCount),this.lengthNormalZ=new Float32Array(this.segmentCount),this.lengthLower=new Float32Array(this.segmentCount),this.lengthUpper=new Float32Array(this.segmentCount),this.lengthRhs=new Float32Array(this.segmentCount),this.lengthSolution=new Float32Array(this.segmentCount),this.bendLambda=new Float32Array(e),this.bendComplianceByNode=new Float32Array(e),this.maxBendAngleByNode=new Float32Array(e),this.controlLambda=new Float32Array(e),this.shapeLambda=new Float32Array(e),this.wallLambda=new Float32Array(this.segmentCount),this.wallActive=new Uint8Array(this.segmentCount),this.wallT=new Float32Array(this.segmentCount),this.wallX=new Float32Array(this.segmentCount),this.wallY=new Float32Array(this.segmentCount),this.wallZ=new Float32Array(this.segmentCount),this.wallNormalX=new Float32Array(this.segmentCount),this.wallNormalY=new Float32Array(this.segmentCount),this.wallNormalZ=new Float32Array(this.segmentCount),this.wallBranchId=new Int32Array(this.segmentCount),this.wallGap=new Float32Array(this.segmentCount),this.wallQueryStartX=new Float32Array(this.segmentCount),this.wallQueryStartY=new Float32Array(this.segmentCount),this.wallQueryStartZ=new Float32Array(this.segmentCount),this.wallQueryEndX=new Float32Array(this.segmentCount),this.wallQueryEndY=new Float32Array(this.segmentCount),this.wallQueryEndZ=new Float32Array(this.segmentCount),this.wallCorrectionX=new Float32Array(e),this.wallCorrectionY=new Float32Array(e),this.wallCorrectionZ=new Float32Array(e),this.wallCorrectionWeight=new Float32Array(e),this.wallBranchId.fill(-1),this.wallGap.fill(1/0),this.nodeRadius.fill(this.radius),this.inverseMass.fill(1/Math.max(Yt,this.mass)),this.restLength.fill(n),this.bendComplianceByNode.fill(this.bendCompliance),this.maxBendAngleByNode.fill(this.maxBendAngle);for(let s=0;s<e;s++)this.x[s]=s*n;this.captureRestConfiguration(),this.copyCurrentToPrevious()}setNodePosition(t,e,n,i,s=!0){return this.x[t]=e,this.y[t]=n,this.z[t]=i,this.previousX[t]=e,this.previousY[t]=n,this.previousZ[t]=i,s&&(this.velocityX[t]=0,this.velocityY[t]=0,this.velocityZ[t]=0),this.wake(),this}setPinned(t,e=!0){return this.pinned[t]=e?1:0,this.inverseMass[t]=e?0:1/Math.max(Yt,this.mass),this.wake(),this}setActiveRange(t,e){const n=ae(Math.floor(t),0,this.count-1),i=ae(Math.ceil(e),n,this.count-1);if(n<this.activeStart)for(let s=n;s<this.activeStart;s++)this.previousX[s]=this.x[s],this.previousY[s]=this.y[s],this.previousZ[s]=this.z[s],this.velocityX[s]=0,this.velocityY[s]=0,this.velocityZ[s]=0;if(i>this.activeEnd)for(let s=this.activeEnd+1;s<=i;s++)this.previousX[s]=this.x[s],this.previousY[s]=this.y[s],this.previousZ[s]=this.z[s],this.velocityX[s]=0,this.velocityY[s]=0,this.velocityZ[s]=0;return(n!==this.activeStart||i!==this.activeEnd)&&this.wake(),this.activeStart=n,this.activeEnd=i,this}setCollisionRange(t,e){const n=Math.floor(t),i=Math.floor(e),s=ae(n,0,this.segmentCount-1);let a;return i<n||n>=this.segmentCount||i<0?a=s-1:a=ae(i,s,this.segmentCount-1),(s!==this.collisionStartSegment||a!==this.collisionEndSegment)&&this.wake(),this.collisionStartSegment=s,this.collisionEndSegment=a,this}setControlTarget(t,e,n,i,s=0){const a=Math.max(0,s),o=!this.controlEnabled[t]||Math.abs(this.controlX[t]-e)>1e-6||Math.abs(this.controlY[t]-n)>1e-6||Math.abs(this.controlZ[t]-i)>1e-6||Math.abs(this.controlCompliance[t]-a)>1e-10;return this.controlEnabled[t]=1,this.controlX[t]=e,this.controlY[t]=n,this.controlZ[t]=i,this.controlCompliance[t]=a,o&&(this.controlLambda[t]=0,this.wake()),this}clearControlTarget(t){return this.controlEnabled[t]&&this.wake(),this.controlEnabled[t]=0,this.controlLambda[t]=0,this}setRestShapeTarget(t,e,n,i,s=this.shapeCompliance){const a=Math.max(0,s),o=!this.restShapeEnabled[t]||Math.abs(this.restShapeX[t]-e)>1e-6||Math.abs(this.restShapeY[t]-n)>1e-6||Math.abs(this.restShapeZ[t]-i)>1e-6||Math.abs(this.restShapeCompliance[t]-a)>1e-10;return this.restShapeEnabled[t]=1,this.restShapeX[t]=e,this.restShapeY[t]=n,this.restShapeZ[t]=i,this.restShapeCompliance[t]=a,o&&(this.shapeLambda[t]=0,this.wake()),this}clearRestShapeTarget(t){return this.restShapeEnabled[t]&&this.wake(),this.restShapeEnabled[t]=0,this.shapeLambda[t]=0,this}captureRestConfiguration(){for(let t=0;t<this.segmentCount;t++)this.restLength[t]=ge(this.x[t+1]-this.x[t],this.y[t+1]-this.y[t],this.z[t+1]-this.z[t])||this.segmentLength;for(let t=1;t<this.count-1;t++)this.restBendChord[t]=ge(this.x[t+1]-this.x[t-1],this.y[t+1]-this.y[t-1],this.z[t+1]-this.z[t-1]);return this.lengthLambda.fill(0),this.bendLambda.fill(0),this}copyCurrentToPrevious(){this.previousX.set(this.x),this.previousY.set(this.y),this.previousZ.set(this.z)}wake(){this.sleeping=!1,this.sleepCounter=0}syncFromElasticRod(t,{resetVelocity:e=!1,preservePrevious:n=!1}={}){const i=t.nodeStorage,s=Math.min(this.count,t.nodes.length);let a=!1;for(let o=0;o<s;o++)a=a||Math.abs(this.x[o]-i.x[o])>1e-6||Math.abs(this.y[o]-i.y[o])>1e-6||Math.abs(this.z[o]-i.z[o])>1e-6||Math.abs(this.velocityX[o]-i.vx[o])>1e-5||Math.abs(this.velocityY[o]-i.vy[o])>1e-5||Math.abs(this.velocityZ[o]-i.vz[o])>1e-5,n&&(this.previousX[o]=this.x[o],this.previousY[o]=this.y[o],this.previousZ[o]=this.z[o]),this.x[o]=i.x[o],this.y[o]=i.y[o],this.z[o]=i.z[o],this.velocityX[o]=e?0:i.vx[o],this.velocityY[o]=e?0:i.vy[o],this.velocityZ[o]=e?0:i.vz[o],this.inverseMass[o]=i.pinned[o]?0:1/Math.max(Yt,i.mass[o]),this.pinned[o]=i.pinned[o],this.bendComplianceByNode[o]=ae(this.bendCompliance*32/Math.max(.1,i.bendingStiffness[o]),this.bendCompliance*this.minBendComplianceScale,this.bendCompliance*8),this.maxBendAngleByNode[o]=ae(i.bendAngleLimit?.[o]??this.maxBendAngle,1,179);return n||this.copyCurrentToPrevious(),a&&this.wake(),this}syncToElasticRod(t){const e=t.nodeStorage,n=Math.min(this.count,t.nodes.length);for(let i=0;i<n;i++)e.x[i]=this.x[i],e.y[i]=this.y[i],e.z[i]=this.z[i],e.vx[i]=this.velocityX[i],e.vy[i]=this.velocityY[i],e.vz[i]=this.velocityZ[i];return this}}class TM{constructor({contactField:t=null,fixedDt:e=EM,maxSubsteps:n=2,iterations:i=6,penetrationIterations:s=8,highPenetration:a=.15,contactActivation:o=.25}={}){this.contactField=t,this.fixedDt=e,this.maxSubsteps=n,this.iterations=i,this.penetrationIterations=s,this.highPenetration=a,this.contactActivation=o,this.accumulator=0,this.bodies=[],this.sheaths=[],this.containments=[],this.toolContacts=[],this.stepCount=0,this.contactCount=0,this.maxPenetration=0,this.settledMaxPenetration=0,this.settledContactBodyId=null,this.settledContactSegment=-1,this.settledContactT=0,this.settledContactX=0,this.settledContactY=0,this.settledContactZ=0,this.lastSubsteps=0,this.droppedTime=0,this._queryStart={x:0,y:0,z:0},this._queryEnd={x:0,y:0,z:0},this._segmentParameters={s:0,t:0},this._contact=ei(),this._sweep=ei(),this.timings={total:Jr(),integrate:Jr(),narrowPhase:Jr(),constraints:Jr(),velocity:Jr()}}createRod(t,e,n,i={}){const s=new Ud(t,e,n,i);return s.contactField=this.contactField,this.bodies.push(s),s}addRod(t){if(!(t instanceof Ud))throw new TypeError("EndovascularRodBody is required");return this.bodies.includes(t)||this.bodies.push(t),t}addSheath({id:t="sheath",start:e,end:n,innerRadius:i=vc.sheath.innerRadius,bodies:s=null}={}){const a=n.x-e.x,o=n.y-e.y,c=n.z-e.z,l=ge(a,o,c);if(l<Yt)throw new RangeError("Sheath axis must have positive length");const h={id:t,startX:e.x,startY:e.y,startZ:e.z,axisX:a/l,axisY:o/l,axisZ:c/l,length:l,innerRadius:i,bodies:s,lambdas:new Map};return this.sheaths.push(h),h}addContainment(t,e,{innerRadius:n=e.innerRadius,compliance:i=0,friction:s=e.lumenFriction,enabled:a=!0,openProximal:o=!0,openDistal:c=!0,searchWindow:l=10,outerStartNode:h=e.activeStart,startNode:u=t.activeStart,endNode:d=t.activeEnd,innerResponse:f=1,outerResponse:g=1,finalProjection:x="inner",outerFollowsInnerCenterline:m=!1,innerArcOffset:p=0,containedLength:_=1/0}={}){const v={innerBody:t,outerBody:e,innerRadius:n,compliance:i,friction:s,enabled:a,openProximal:o,openDistal:c,searchWindow:l,outerStartNode:h,startNode:u,endNode:d,innerResponse:ae(f,0,1),outerResponse:ae(g,0,1),finalProjection:x,outerFollowsInnerCenterline:m,innerArcOffset:p,containedLength:_,lambdas:new Float32Array(t.count),closestSegment:new Int32Array(t.count),_lastEnabled:a,_lastOuterStartNode:h,_lastStartNode:u,_lastEndNode:d,_lastInnerActiveStart:t.activeStart,_lastInnerActiveEnd:t.activeEnd,_lastOuterActiveStart:e.activeStart,_lastOuterActiveEnd:e.activeEnd};return v.closestSegment.fill(-1),this.containments.push(v),v}addToolContact(t,e,{compliance:n=0,friction:i=.06,enabled:s=!0,openDistalB:a=!1,startSegmentA:o=0,endSegmentA:c=t.segmentCount-1,startSegmentB:l=0,endSegmentB:h=e.segmentCount-1}={}){const u=t.segmentCount*e.segmentCount,d={bodyA:t,bodyB:e,compliance:n,friction:i,enabled:s,openDistalB:a,startSegmentA:o,endSegmentA:c,startSegmentB:l,endSegmentB:h,lambdas:new Float32Array(u),_lastEnabled:s,_lastStartSegmentA:o,_lastEndSegmentA:c,_lastStartSegmentB:l,_lastEndSegmentB:h};return this.toolContacts.push(d),d}advance(t,e=null){const n=Math.max(0,Math.min(.25,t));this.accumulator+=n;let i=0;for(;this.accumulator+Yt>=this.fixedDt&&i<this.maxSubsteps;)e?.(this.fixedDt,i),this.stepFixed(),this.accumulator-=this.fixedDt,i++;return this.accumulator>=this.fixedDt&&(this.droppedTime+=this.accumulator-this.accumulator%this.fixedDt,this.accumulator%=this.fixedDt),this.lastSubsteps=i,i}stepFixed(){const t=Gn();this.contactCount=0,this.maxPenetration=0;let e=Gn();for(let o=0;o<this.bodies.length;o++){const c=this.bodies[o];c.contactField=this.contactField,c.lengthLambda.fill(0),c.bendLambda.fill(0),c.controlLambda.fill(0),c.shapeLambda.fill(0),this.#t(c)}Qr(this.timings.integrate,Gn()-e),e=Gn();for(let o=0;o<this.bodies.length;o++)this.#e(this.bodies[o]);for(let o=0;o<this.bodies.length;o++)this.#r(this.bodies[o]);let n=Gn()-e;e=Gn();const i=this.maxPenetration>this.highPenetration?this.penetrationIterations:this.iterations;for(let o=0;o<i;o++){for(let c=0;c<this.sheaths.length;c++)this.#S(this.sheaths[c]);for(let c=0;c<this.bodies.length;c++)this.#c(this.bodies[c]);for(let c=0;c<this.bodies.length;c++)this.#T(this.bodies[c],(o&1)===1);for(let c=0;c<this.bodies.length;c++)this.#a(this.bodies[c]);for(let c=0;c<this.bodies.length;c++)this.#i(this.bodies[c]);for(let c=0;c<this.bodies.length;c++)this.#c(this.bodies[c]);for(let c=0;c<this.containments.length;c++)this.#A(this.containments[c]);for(let c=0;c<this.toolContacts.length;c++)this.#y(this.toolContacts[c]);for(let c=0;c<this.bodies.length;c++)this.#u(this.bodies[c]);for(let c=0;c<this.bodies.length;c++)this.#p(this.bodies[c])}for(let o=0;o<8;o++){for(let l=0;l<this.bodies.length;l++)this.#u(this.bodies[l]);for(let l=0;l<this.bodies.length;l++)this.#p(this.bodies[l]);for(let l=0;l<this.bodies.length;l++)this.#h(this.bodies[l]);let c=!0;for(let l=0;l<this.bodies.length;l++)c=c&&!this.#o(this.bodies[l],.002);if(c)break}for(let o=0;o<Fd;o++){let c=0;for(let l=0;l<this.bodies.length;l++)c=Math.max(c,this.#f(this.bodies[l]));if(c<=.02)break;for(let l=0;l<this.bodies.length;l++)this.#p(this.bodies[l]),this.#m(this.bodies[l]),o+1<Fd&&this.#h(this.bodies[l])}const s=this.containments.some(o=>o.enabled&&o.finalProjection!=="none"&&!o.outerFollowsInnerCenterline)?2:1;for(let o=0;o<s;o++)for(let c=0;c<this.containments.length;c++){const l=this.containments[c];!l.enabled||l.finalProjection==="none"||this.#A(l,{innerOnly:l.finalProjection!=="outer",outerOnly:l.finalProjection==="outer",applyFriction:!1})}for(let o=0;o<this.bodies.length;o++){const c=this.bodies[o];for(let l=0;l<c.postStabilizationPasses;l++)this.#c(c),this.#p(c),this.#h(c),this.#r(c),this.#u(c);this.#p(c)}Qr(this.timings.constraints,Gn()-e);const a=this.maxPenetration;e=Gn(),this.contactCount=0,this.maxPenetration=0,this.settledContactBodyId=null,this.settledContactSegment=-1;for(let o=0;o<this.bodies.length;o++)this.#f(this.bodies[o]);this.settledMaxPenetration=this.maxPenetration,this.maxPenetration=Math.max(a,this.settledMaxPenetration),n+=Gn()-e,Qr(this.timings.narrowPhase,n),e=Gn();for(let o=0;o<this.bodies.length;o++)this.#g(this.bodies[o]);for(let o=0;o<this.bodies.length;o++)this.#s(this.bodies[o]);for(let o=0;o<this.containments.length;o++)this.#w(this.containments[o]);for(let o=0;o<this.toolContacts.length;o++)this.#l(this.toolContacts[o]);for(let o=0;o<this.bodies.length;o++)this.#C(this.bodies[o]);Qr(this.timings.velocity,Gn()-e),this.stepCount++,Qr(this.timings.total,Gn()-t)}resetPerformanceStats(){this.contactCount=0,this.maxPenetration=0,this.settledMaxPenetration=0;for(const t of Object.values(this.timings))t.samples.fill(0),t.cursor=0,t.count=0,t.recordedCount=0,t.total=0,t.last=0}resetSimulationState(){this.accumulator=0,this.stepCount=0,this.lastSubsteps=0,this.droppedTime=0;for(const t of this.bodies)t.lengthLambda.fill(0),t.bendLambda.fill(0),t.controlLambda.fill(0),t.shapeLambda.fill(0),t.wallLambda.fill(0),t.wallActive.fill(0),t.wallBranchId.fill(-1),t.wallGap.fill(1/0),t.copyCurrentToPrevious(),t.wake();for(const t of this.sheaths)t.lambdas.clear();for(const t of this.containments)t.lambdas.fill(0),t.closestSegment.fill(-1),t._lastEnabled=t.enabled,t._lastOuterStartNode=t.outerStartNode,t._lastStartNode=t.startNode,t._lastEndNode=t.endNode,t._lastInnerActiveStart=t.innerBody.activeStart,t._lastInnerActiveEnd=t.innerBody.activeEnd,t._lastOuterActiveStart=t.outerBody.activeStart,t._lastOuterActiveEnd=t.outerBody.activeEnd;for(const t of this.toolContacts)t.lambdas.fill(0),t._lastEnabled=t.enabled,t._lastStartSegmentA=t.startSegmentA,t._lastEndSegmentA=t.endSegmentA,t._lastStartSegmentB=t.startSegmentB,t._lastEndSegmentB=t.endSegmentB;return this.resetPerformanceStats(),this}getStats(){const t=this.bodies.map(e=>this.#D(e));return{mode:"xpbd-contact-v1",fixedDt:this.fixedDt,steps:this.stepCount,lastSubsteps:this.lastSubsteps,droppedTime:this.droppedTime,contacts:this.contactCount,maxPenetration:this.maxPenetration,settledMaxPenetration:this.settledMaxPenetration,settledContact:{bodyId:this.settledContactBodyId,segment:this.settledContactSegment,t:this.settledContactT,x:this.settledContactX,y:this.settledContactY,z:this.settledContactZ},phases:{total:ta(this.timings.total),integrate:ta(this.timings.integrate),narrowPhase:ta(this.timings.narrowPhase),constraints:ta(this.timings.constraints),velocity:ta(this.timings.velocity)},bodies:t}}#t(t){if(t.sleeping)return;const e=this.fixedDt,n=e*e,i=t.activeStart,s=t.activeEnd;for(let a=i;a<=s;a++)t.previousX[a]=t.x[a],t.previousY[a]=t.y[a],t.previousZ[a]=t.z[a],!(t.inverseMass[a]<=0)&&(t.velocityX[a]*=t.linearDamping,t.velocityY[a]*=t.linearDamping,t.velocityZ[a]*=t.linearDamping,t.x[a]+=t.velocityX[a]*e+t.forceX[a]*t.inverseMass[a]*n,t.y[a]+=t.velocityY[a]*e+t.forceY[a]*t.inverseMass[a]*n,t.z[a]+=t.velocityZ[a]*e+t.forceZ[a]*t.inverseMass[a]*n);t.forceX.fill(0),t.forceY.fill(0),t.forceZ.fill(0)}#e(t){if(!this.contactField||t.sleeping||t.collisionEndSegment<t.collisionStartSegment)return;const e=Math.max(t.activeStart,t.collisionStartSegment),n=Math.min(t.activeEnd,t.collisionEndSegment+1);for(let i=e;i<=n;i++){const s=t.x[i]-t.previousX[i],a=t.y[i]-t.previousY[i],o=t.z[i]-t.previousZ[i],c=t.nodeRadius[i];if(s*s+a*a+o*o<=c*c*.25)continue;this._queryStart.x=t.previousX[i],this._queryStart.y=t.previousY[i],this._queryStart.z=t.previousZ[i],this._queryEnd.x=t.x[i],this._queryEnd.y=t.y[i],this._queryEnd.z=t.z[i];const l=this.contactField.sweepSphere(this._queryStart,this._queryEnd,c,this._sweep);if(!l.violation||l.timeOfImpact>=1)continue;const h=Math.max(0,l.timeOfImpact-.001);t.x[i]=t.previousX[i]+s*h+l.inward.x*.001,t.y[i]=t.previousY[i]+a*h+l.inward.y*.001,t.z[i]=t.previousZ[i]+o*h+l.inward.z*.001}}#r(t){if(!this.contactField||t.sleeping||t.collisionEndSegment<t.collisionStartSegment)return;const e=Math.max(t.activeStart,t.collisionStartSegment,0),n=Math.min(t.activeEnd,t.collisionEndSegment+1,t.segmentCount);for(let i=e;i<n;i++){const s=t.wallActive[i]!==0;if(t.wallActive[i]=0,s){const g=t.wallT[i],x=t.x[i]+(t.x[i+1]-t.x[i])*g,m=t.y[i]+(t.y[i+1]-t.y[i])*g,p=t.z[i]+(t.z[i+1]-t.z[i])*g,_=Math.max(t.nodeRadius[i],t.nodeRadius[i+1]),v=(x-t.wallX[i])*t.wallNormalX[i]+(m-t.wallY[i])*t.wallNormalY[i]+(p-t.wallZ[i])*t.wallNormalZ[i]-_;if(v<=this.contactActivation+.1){t.wallActive[i]=1,v<0&&(this.contactCount++,this.maxPenetration=Math.max(this.maxPenetration,-v));continue}}const a=t.wallGap[i];if(!s&&Number.isFinite(a)){const g=t.x[i]-t.wallQueryStartX[i],x=t.y[i]-t.wallQueryStartY[i],m=t.z[i]-t.wallQueryStartZ[i],p=t.x[i+1]-t.wallQueryEndX[i],_=t.y[i+1]-t.wallQueryEndY[i],v=t.z[i+1]-t.wallQueryEndZ[i],S=Math.sqrt(g*g+x*x+m*m),M=Math.sqrt(p*p+_*_+v*v);if(a-Math.max(S,M)>this.contactActivation){t.wallLambda[i]*=.5;continue}}let o;if(this.contactField.queryCapsuleSoA)o=this.contactField.queryCapsuleSoA(t.x,t.y,t.z,t.nodeRadius,i,this._contact);else{const g=Math.max(t.nodeRadius[i],t.nodeRadius[i+1]);this._queryStart.x=t.x[i],this._queryStart.y=t.y[i],this._queryStart.z=t.z[i],this._queryEnd.x=t.x[i+1],this._queryEnd.y=t.y[i+1],this._queryEnd.z=t.z[i+1],o=this.contactField.queryCapsule(this._queryStart,this._queryEnd,g,this._contact)}const c=o.values,l=c[Dd],h=c[Nd],u=c[Id],d=o.closestPoint.values,f=o.inward.values;if(t.wallGap[i]=l,t.wallQueryStartX[i]=t.x[i],t.wallQueryStartY[i]=t.y[i],t.wallQueryStartZ[i]=t.z[i],t.wallQueryEndX[i]=t.x[i+1],t.wallQueryEndY[i]=t.y[i+1],t.wallQueryEndZ[i]=t.z[i+1],l>this.contactActivation){t.wallLambda[i]*=.5;continue}t.wallBranchId[i]!==u&&(t.wallLambda[i]=0),t.wallActive[i]=1,t.wallT[i]=h,t.wallX[i]=d[0],t.wallY[i]=d[1],t.wallZ[i]=d[2],t.wallNormalX[i]=f[0],t.wallNormalY[i]=f[1],t.wallNormalZ[i]=f[2],t.wallBranchId[i]=u,o.violation&&(this.contactCount++,this.maxPenetration=Math.max(this.maxPenetration,c[_o]))}}#f(t){if(!this.contactField||t.sleeping||t.collisionEndSegment<t.collisionStartSegment)return 0;const e=Math.max(t.activeStart,t.collisionStartSegment,0),n=Math.min(t.activeEnd,t.collisionEndSegment+1,t.segmentCount);let i=0;for(let s=e;s<n;s++){if(!t.wallActive[s])continue;const a=t.wallGap[s];if(Number.isFinite(a)){const g=t.x[s]-t.wallQueryStartX[s],x=t.y[s]-t.wallQueryStartY[s],m=t.z[s]-t.wallQueryStartZ[s],p=t.x[s+1]-t.wallQueryEndX[s],_=t.y[s+1]-t.wallQueryEndY[s],v=t.z[s+1]-t.wallQueryEndZ[s],S=Math.sqrt(g*g+x*x+m*m),M=Math.sqrt(p*p+_*_+v*v),y=a-Math.max(S,M);if(y>.02){y>this.contactActivation&&(t.wallActive[s]=0,t.wallLambda[s]*=.5);continue}}let o;if(this.contactField.queryCapsuleSoA)o=this.contactField.queryCapsuleSoA(t.x,t.y,t.z,t.nodeRadius,s,this._contact);else{const g=Math.max(t.nodeRadius[s],t.nodeRadius[s+1]);this._queryStart.x=t.x[s],this._queryStart.y=t.y[s],this._queryStart.z=t.z[s],this._queryEnd.x=t.x[s+1],this._queryEnd.y=t.y[s+1],this._queryEnd.z=t.z[s+1],o=this.contactField.queryCapsule(this._queryStart,this._queryEnd,g,this._contact)}const c=o.values,l=c[Dd],h=c[Nd],u=c[Id],d=o.closestPoint.values,f=o.inward.values;if(t.wallGap[s]=l,t.wallQueryStartX[s]=t.x[s],t.wallQueryStartY[s]=t.y[s],t.wallQueryStartZ[s]=t.z[s],t.wallQueryEndX[s]=t.x[s+1],t.wallQueryEndY[s]=t.y[s+1],t.wallQueryEndZ[s]=t.z[s+1],l>this.contactActivation){t.wallActive[s]=0,t.wallLambda[s]*=.5;continue}t.wallBranchId[s]!==u&&(t.wallLambda[s]=0),t.wallT[s]=h,t.wallX[s]=d[0],t.wallY[s]=d[1],t.wallZ[s]=d[2],t.wallNormalX[s]=f[0],t.wallNormalY[s]=f[1],t.wallNormalZ[s]=f[2],t.wallBranchId[s]=u,o.violation&&(this.contactCount++,c[_o]>this.maxPenetration&&(this.settledContactBodyId=t.id,this.settledContactSegment=s,this.settledContactT=h,this.settledContactX=t.x[s]+(t.x[s+1]-t.x[s])*h,this.settledContactY=t.y[s]+(t.y[s+1]-t.y[s])*h,this.settledContactZ=t.z[s]+(t.z[s+1]-t.z[s])*h),this.maxPenetration=Math.max(this.maxPenetration,c[_o]),i=Math.max(i,c[_o]))}return i}#o(t,e){const n=Math.max(0,t.activeStart),i=Math.min(t.segmentCount,t.activeEnd);for(let s=n;s<i;s++){const a=ge(t.x[s+1]-t.x[s],t.y[s+1]-t.y[s],t.z[s+1]-t.z[s]);if(Math.abs(a-t.restLength[s])>t.restLength[s]*e)return!0}return!1}#c(t){if(t.sleeping)return;const e=this.fixedDt*this.fixedDt;for(let n=t.activeStart;n<=t.activeEnd;n++){if(!t.controlEnabled[n]||t.inverseMass[n]<=0)continue;const i=t.x[n]-t.controlX[n],s=t.y[n]-t.controlY[n],a=t.z[n]-t.controlZ[n],o=ge(i,s,a);if(o<Yt)continue;const c=t.controlCompliance[n]/e,l=(-o-c*t.controlLambda[n])/(t.inverseMass[n]+c);t.controlLambda[n]+=l;const h=l/o*t.inverseMass[n];t.x[n]+=i*h,t.y[n]+=s*h,t.z[n]+=a*h}}#T(t,e=!1){if(t.sleeping)return;const n=t.stretchCompliance/(this.fixedDt*this.fixedDt),i=Math.max(0,t.activeStart),s=Math.min(t.segmentCount,t.activeEnd);for(let a=e?s-1:i;e?a>=i:a<s;a+=e?-1:1){const o=t.x[a+1]-t.x[a],c=t.y[a+1]-t.y[a],l=t.z[a+1]-t.z[a],h=ge(o,c,l);if(h<Yt)continue;const u=t.inverseMass[a],d=t.inverseMass[a+1],f=u+d+n;if(f<Yt)continue;const x=(-(h-t.restLength[a])-n*t.lengthLambda[a])/f;t.lengthLambda[a]+=x;const m=o/h,p=c/h,_=l/h;t.x[a]-=m*x*u,t.y[a]-=p*x*u,t.z[a]-=_*x*u,t.x[a+1]+=m*x*d,t.y[a+1]+=p*x*d,t.z[a+1]+=_*x*d}}#h(t){if(t.sleeping)return;const e=Math.max(0,t.activeStart),i=Math.min(t.segmentCount,t.activeEnd)-e;if(i<=0)return;for(let a=0;a<i;a++){const o=e+a,c=t.x[o+1]-t.x[o],l=t.y[o+1]-t.y[o],h=t.z[o+1]-t.z[o],u=ge(c,l,h);u<Yt?(t.lengthNormalX[a]=1,t.lengthNormalY[a]=0,t.lengthNormalZ[a]=0,t.lengthRhs[a]=0):(t.lengthNormalX[a]=c/u,t.lengthNormalY[a]=l/u,t.lengthNormalZ[a]=h/u,t.lengthRhs[a]=-(u-t.restLength[o]))}for(let a=0;a<i;a++){const o=e+a;let c=0,l=0;a>0&&(c=-t.inverseMass[o]*(t.lengthNormalX[a]*t.lengthNormalX[a-1]+t.lengthNormalY[a]*t.lengthNormalY[a-1]+t.lengthNormalZ[a]*t.lengthNormalZ[a-1])),a+1<i&&(l=-t.inverseMass[o+1]*(t.lengthNormalX[a]*t.lengthNormalX[a+1]+t.lengthNormalY[a]*t.lengthNormalY[a+1]+t.lengthNormalZ[a]*t.lengthNormalZ[a+1])),t.lengthLower[a]=c,t.lengthUpper[a]=l,t.lengthSolution[a]=t.inverseMass[o]+t.inverseMass[o+1]}let s=Math.max(Yt,t.lengthSolution[0]);t.lengthUpper[0]/=s,t.lengthRhs[0]/=s;for(let a=1;a<i;a++)s=Math.max(Yt,t.lengthSolution[a]-t.lengthLower[a]*t.lengthUpper[a-1]),t.lengthUpper[a]=a+1<i?t.lengthUpper[a]/s:0,t.lengthRhs[a]=(t.lengthRhs[a]-t.lengthLower[a]*t.lengthRhs[a-1])/s;t.lengthSolution[i-1]=t.lengthRhs[i-1];for(let a=i-2;a>=0;a--)t.lengthSolution[a]=t.lengthRhs[a]-t.lengthUpper[a]*t.lengthSolution[a+1];for(let a=0;a<i;a++){const o=e+a,c=t.lengthSolution[a],l=t.lengthNormalX[a],h=t.lengthNormalY[a],u=t.lengthNormalZ[a];t.x[o]-=l*c*t.inverseMass[o],t.y[o]-=h*c*t.inverseMass[o],t.z[o]-=u*c*t.inverseMass[o],t.x[o+1]+=l*c*t.inverseMass[o+1],t.y[o+1]+=h*c*t.inverseMass[o+1],t.z[o+1]+=u*c*t.inverseMass[o+1]}}#a(t){if(t.sleeping||t.count<3)return;const e=Math.max(1,t.activeStart+1),n=Math.min(t.count-1,t.activeEnd);for(let i=e;i<n;i++){const s=i-1,a=i+1,o=t.x[a]-t.x[s],c=t.y[a]-t.y[s],l=t.z[a]-t.z[s],h=ge(o,c,l);if(h<Yt)continue;const u=t.inverseMass[s],d=t.inverseMass[a],f=t.bendComplianceByNode[i]/(this.fixedDt*this.fixedDt),g=u+d+f;if(g<Yt)continue;const m=(-(h-t.restBendChord[i])-f*t.bendLambda[i])/g;t.bendLambda[i]+=m;const p=o/h,_=c/h,v=l/h;t.x[s]-=p*m*u,t.y[s]-=_*m*u,t.z[s]-=v*m*u,t.x[a]+=p*m*d,t.y[a]+=_*m*d,t.z[a]+=v*m*d}}#i(t){if(t.sleeping)return;const e=this.fixedDt*this.fixedDt;for(let n=t.activeStart;n<=t.activeEnd;n++){if(!t.restShapeEnabled[n]||t.inverseMass[n]<=0)continue;const i=t.x[n]-t.restShapeX[n],s=t.y[n]-t.restShapeY[n],a=t.z[n]-t.restShapeZ[n],o=ge(i,s,a);if(o<Yt)continue;const c=t.restShapeCompliance[n]/e,l=(-o-c*t.shapeLambda[n])/(t.inverseMass[n]+c);t.shapeLambda[n]+=l;const h=l/o*t.inverseMass[n];t.x[n]+=i*h,t.y[n]+=s*h,t.z[n]+=a*h}}#p(t){if(t.sleeping||t.count<3||t.foldLimitStrength<=0)return;const e=Math.max(1,t.activeStart+1),n=Math.min(t.count-1,t.activeEnd);for(let i=e;i<n;i++){const s=ae(t.maxBendAngleByNode[i],1,179)*Math.PI/180,a=Math.cos(s),o=i-1,c=i+1,l=t.x[i]-t.x[o],h=t.y[i]-t.y[o],u=t.z[i]-t.z[o],d=t.x[c]-t.x[i],f=t.y[c]-t.y[i],g=t.z[c]-t.z[i],x=ge(l,h,u),m=ge(d,f,g);if(x<Yt||m<Yt)continue;const p=(l*d+h*f+u*g)/(x*m);if(p>=a)continue;if((o>0&&t.wallActive[o-1]||t.wallActive[o]||t.wallActive[i])&&t.inverseMass[i]>0){const N=Math.min(.72,t.foldLimitStrength*.62);t.x[i]+=((t.x[o]+t.x[c])*.5-t.x[i])*N,t.y[i]+=((t.y[o]+t.y[c])*.5-t.y[i])*N,t.z[i]+=((t.z[o]+t.z[c])*.5-t.z[i])*N;continue}let v=t.x[c]-t.x[o],S=t.y[c]-t.y[o],M=t.z[c]-t.z[o];const y=ge(v,S,M);y<Yt?(v=l/x,S=h/x,M=u/x):(v/=y,S/=y,M/=y);const A=Math.sqrt(Math.max(0,x*x+m*m+2*x*m*a))-y;if(A<=0)continue;const E=t.inverseMass[o],T=t.inverseMass[c],L=E+T;if(L<Yt)continue;const C=Math.min(A*t.foldLimitStrength,Math.min(x,m)*.35),F=C*E/L,D=C*T/L;if(t.x[o]-=v*F,t.y[o]-=S*F,t.z[o]-=M*F,t.x[c]+=v*D,t.y[c]+=S*D,t.z[c]+=M*D,t.inverseMass[i]>0){const N=t.foldLimitStrength*.45;if(t.x[i]+=((t.x[o]+t.x[c])*.5-t.x[i])*N,t.y[i]+=((t.y[o]+t.y[c])*.5-t.y[i])*N,t.z[i]+=((t.z[o]+t.z[c])*.5-t.z[i])*N,p<-.999&&y<Math.min(x,m)*.1){const B=l/x,G=h/x,z=u/x;let H,j,$;Math.abs(B)<.8?(H=0,j=z,$=-G):(H=-z,j=0,$=B);const Q=ge(H,j,$)||1,V=Math.min(x,m)*t.foldLimitStrength*.05;t.x[i]+=H/Q*V,t.y[i]+=j/Q*V,t.z[i]+=$/Q*V}}}}#A(t,{innerOnly:e=!1,outerOnly:n=!1,applyFriction:i=!0}={}){if((t.enabled!==t._lastEnabled||t.outerStartNode!==t._lastOuterStartNode||t.startNode!==t._lastStartNode||t.endNode!==t._lastEndNode||t.innerBody.activeStart!==t._lastInnerActiveStart||t.innerBody.activeEnd!==t._lastInnerActiveEnd||t.outerBody.activeStart!==t._lastOuterActiveStart||t.outerBody.activeEnd!==t._lastOuterActiveEnd)&&(t.lambdas.fill(0),t.closestSegment.fill(-1),t._lastEnabled=t.enabled,t._lastOuterStartNode=t.outerStartNode,t._lastStartNode=t.startNode,t._lastInnerActiveStart=t.innerBody.activeStart,t._lastInnerActiveEnd=t.innerBody.activeEnd,t._lastOuterActiveStart=t.outerBody.activeStart,t._lastOuterActiveEnd=t.outerBody.activeEnd),t._lastEndNode=t.endNode,!t.enabled)return;if(t.outerFollowsInnerCenterline){this.#v(t);return}const s=t.innerBody,a=t.outerBody,o=Math.max(0,t.innerRadius-s.radius),c=t.compliance/(this.fixedDt*this.fixedDt),l=ae(t.outerStartNode,a.activeStart,a.activeEnd),h=Math.min(a.activeEnd,a.segmentCount);if(h<=l)return;const u=ae(t.startNode,s.activeStart,s.activeEnd),d=ae(t.endNode,u,s.activeEnd);let f=l,g=l,x=0,m=a.restLength[l];for(let p=u;p<=d;p++){for(p>u&&(x+=s.restLength[p-1]);f<h-1&&m<x;)f++,m+=a.restLength[f];let _=Math.max(l,g,f-t.searchWindow),v=Math.min(h-1,f+t.searchWindow),S=1/0,M=-1,y=0,w=0,A=0,E=0;for(let Nt=_;Nt<=v;Nt++){const Tt=a.x[Nt],Pt=a.y[Nt],yt=a.z[Nt],Kt=a.x[Nt+1]-Tt,bt=a.y[Nt+1]-Pt,I=a.z[Nt+1]-yt,b=Kt*Kt+bt*bt+I*I,X=ae(((s.x[p]-Tt)*Kt+(s.y[p]-Pt)*bt+(s.z[p]-yt)*I)/Math.max(Yt,b),0,1),et=Tt+Kt*X,J=Pt+bt*X,tt=yt+I*X,St=s.x[p]-et,lt=s.y[p]-J,gt=s.z[p]-tt,At=St*St+lt*lt+gt*gt;At<S&&(S=At,M=Nt,y=X,w=et,A=J,E=tt)}if(M<0)continue;if(g=M,t.closestSegment[p]=M,t.openProximal&&M===l&&y<=1e-5){const Nt=a.x[l+1]-a.x[l],Tt=a.y[l+1]-a.y[l],Pt=a.z[l+1]-a.z[l];if((s.x[p]-a.x[l])*Nt+(s.y[p]-a.y[l])*Tt+(s.z[p]-a.z[l])*Pt<0)continue}if(t.openDistal&&M===h-1&&y>=1-1e-5){const Nt=a.x[h]-a.x[h-1],Tt=a.y[h]-a.y[h-1],Pt=a.z[h]-a.z[h-1];if((s.x[p]-a.x[h])*Nt+(s.y[p]-a.y[h])*Tt+(s.z[p]-a.z[h])*Pt>0)continue}const T=Math.sqrt(S);if(T<=o||T<Yt){t.lambdas[p]*=.8;continue}const L=(s.x[p]-w)/T,C=(s.y[p]-A)/T,F=(s.z[p]-E)/T,D=n?0:e?1:t.innerResponse,N=e?0:t.outerResponse,B=s.inverseMass[p]*D,G=1-y,z=y,H=a.inverseMass[M]*N*G*G,j=a.inverseMass[M+1]*N*z*z,$=B+H+j+c;if($<Yt)continue;const V=(-(o-T)-c*t.lambdas[p])/$;t.lambdas[p]+=V,s.x[p]-=L*V*B,s.y[p]-=C*V*B,s.z[p]-=F*V*B,a.x[M]+=L*V*a.inverseMass[M]*N*G,a.y[M]+=C*V*a.inverseMass[M]*N*G,a.z[M]+=F*V*a.inverseMass[M]*N*G,a.x[M+1]+=L*V*a.inverseMass[M+1]*N*z,a.y[M+1]+=C*V*a.inverseMass[M+1]*N*z,a.z[M+1]+=F*V*a.inverseMass[M+1]*N*z;const K=s.x[p]-s.previousX[p]-(a.x[M]-a.previousX[M])*G-(a.x[M+1]-a.previousX[M+1])*z,nt=s.y[p]-s.previousY[p]-(a.y[M]-a.previousY[M])*G-(a.y[M+1]-a.previousY[M+1])*z,rt=s.z[p]-s.previousZ[p]-(a.z[M]-a.previousZ[M])*G-(a.z[M+1]-a.previousZ[M+1])*z,ot=K*L+nt*C+rt*F;let _t=K-L*ot,ht=nt-C*ot,ut=rt-F*ot;const Ct=ge(_t,ht,ut),W=B+H+j;if(Ct>Yt&&W>Yt&&i&&t.friction>0){_t/=Ct,ht/=Ct,ut/=Ct;const Nt=-Math.min(Ct/W,t.friction*t.lambdas[p]);s.x[p]+=_t*Nt*B,s.y[p]+=ht*Nt*B,s.z[p]+=ut*Nt*B,a.x[M]-=_t*Nt*a.inverseMass[M]*N*G,a.y[M]-=ht*Nt*a.inverseMass[M]*N*G,a.z[M]-=ut*Nt*a.inverseMass[M]*N*G,a.x[M+1]-=_t*Nt*a.inverseMass[M+1]*N*z,a.y[M+1]-=ht*Nt*a.inverseMass[M+1]*N*z,a.z[M+1]-=ut*Nt*a.inverseMass[M+1]*N*z}}}#v(t){const e=t.innerBody,n=t.outerBody,i=ae(t.startNode,e.activeStart,e.activeEnd),s=ae(t.outerStartNode,n.activeStart,n.activeEnd);if(i>=e.activeEnd||s>n.activeEnd)return;let a=i,o=Math.max(0,t.innerArcOffset);i>e.activeStart&&(a=i-1,o-=e.restLength[a]);let c=0;const l=Math.max(0,t.containedLength);for(let h=s;h<=n.activeEnd&&!(c>l+1e-5);h++){for(;a<e.activeEnd-1&&o+e.restLength[a]<c;)o+=e.restLength[a],a++;const u=Math.max(Yt,e.restLength[a]),d=ae((c-o)/u,0,1),f=e.x[a]+(e.x[a+1]-e.x[a])*d,g=e.y[a]+(e.y[a+1]-e.y[a])*d,x=e.z[a]+(e.z[a+1]-e.z[a])*d;n.x[h]=f,n.y[h]=g,n.z[h]=x,h<n.activeEnd&&(c+=n.restLength[h])}}#y(t){if((t.enabled!==t._lastEnabled||t.startSegmentA!==t._lastStartSegmentA||t.endSegmentA!==t._lastEndSegmentA||t.startSegmentB!==t._lastStartSegmentB||t.endSegmentB!==t._lastEndSegmentB)&&(t.lambdas.fill(0),t._lastEnabled=t.enabled,t._lastStartSegmentA=t.startSegmentA,t._lastEndSegmentA=t.endSegmentA,t._lastStartSegmentB=t.startSegmentB,t._lastEndSegmentB=t.endSegmentB),!t.enabled)return;const e=t.bodyA,n=t.bodyB,i=t.compliance/(this.fixedDt*this.fixedDt),s=ae(t.startSegmentA,e.activeStart,e.segmentCount-1),a=ae(t.endSegmentA,s,Math.min(e.activeEnd-1,e.segmentCount-1)),o=ae(t.startSegmentB,n.activeStart,n.segmentCount-1),c=ae(t.endSegmentB,o,Math.min(n.activeEnd-1,n.segmentCount-1));for(let l=s;l<=a;l++)for(let h=o;h<=c;h++){const u=this.#E(e,l,n,h,this._segmentParameters),d=e.x[l]+(e.x[l+1]-e.x[l])*u.s,f=e.y[l]+(e.y[l+1]-e.y[l])*u.s,g=e.z[l]+(e.z[l+1]-e.z[l])*u.s,x=n.x[h]+(n.x[h+1]-n.x[h])*u.t,m=n.y[h]+(n.y[h+1]-n.y[h])*u.t,p=n.z[h]+(n.z[h+1]-n.z[h])*u.t;if(t.openDistalB&&h===c&&u.t>=1-1e-5){const ht=n.x[c+1]-n.x[c],ut=n.y[c+1]-n.y[c],Ct=n.z[c+1]-n.z[c];if((d-n.x[c+1])*ht+(f-n.y[c+1])*ut+(g-n.z[c+1])*Ct>0)continue}let _=d-x,v=f-m,S=g-p;const M=ge(_,v,S),y=Math.max(e.nodeRadius[l],e.nodeRadius[l+1])+Math.max(n.nodeRadius[h],n.nodeRadius[h+1]);if(M>=y||M<Yt)continue;_/=M,v/=M,S/=M;const w=1-u.s,A=u.s,E=1-u.t,T=u.t,L=e.inverseMass[l]*w*w,C=e.inverseMass[l+1]*A*A,F=n.inverseMass[h]*E*E,D=n.inverseMass[h+1]*T*T,N=L+C+F+D+i;if(N<Yt)continue;const B=l*n.segmentCount+h;let z=(-(M-y)-i*t.lambdas[B])/N;const H=Math.max(0,t.lambdas[B]+z);z=H-t.lambdas[B],t.lambdas[B]=H,e.x[l]+=_*z*e.inverseMass[l]*w,e.y[l]+=v*z*e.inverseMass[l]*w,e.z[l]+=S*z*e.inverseMass[l]*w,e.x[l+1]+=_*z*e.inverseMass[l+1]*A,e.y[l+1]+=v*z*e.inverseMass[l+1]*A,e.z[l+1]+=S*z*e.inverseMass[l+1]*A,n.x[h]-=_*z*n.inverseMass[h]*E,n.y[h]-=v*z*n.inverseMass[h]*E,n.z[h]-=S*z*n.inverseMass[h]*E,n.x[h+1]-=_*z*n.inverseMass[h+1]*T,n.y[h+1]-=v*z*n.inverseMass[h+1]*T,n.z[h+1]-=S*z*n.inverseMass[h+1]*T;const j=(e.x[l]-e.previousX[l])*w+(e.x[l+1]-e.previousX[l+1])*A-(n.x[h]-n.previousX[h])*E-(n.x[h+1]-n.previousX[h+1])*T,$=(e.y[l]-e.previousY[l])*w+(e.y[l+1]-e.previousY[l+1])*A-(n.y[h]-n.previousY[h])*E-(n.y[h+1]-n.previousY[h+1])*T,Q=(e.z[l]-e.previousZ[l])*w+(e.z[l+1]-e.previousZ[l+1])*A-(n.z[h]-n.previousZ[h])*E-(n.z[h+1]-n.previousZ[h+1])*T,V=j*_+$*v+Q*S;let K=j-_*V,nt=$-v*V,rt=Q-S*V;const ot=ge(K,nt,rt),_t=L+C+F+D;if(ot>Yt&&_t>Yt&&t.friction>0){K/=ot,nt/=ot,rt/=ot;const ht=-Math.min(ot/_t,t.friction*H);e.x[l]+=K*ht*e.inverseMass[l]*w,e.y[l]+=nt*ht*e.inverseMass[l]*w,e.z[l]+=rt*ht*e.inverseMass[l]*w,e.x[l+1]+=K*ht*e.inverseMass[l+1]*A,e.y[l+1]+=nt*ht*e.inverseMass[l+1]*A,e.z[l+1]+=rt*ht*e.inverseMass[l+1]*A,n.x[h]-=K*ht*n.inverseMass[h]*E,n.y[h]-=nt*ht*n.inverseMass[h]*E,n.z[h]-=rt*ht*n.inverseMass[h]*E,n.x[h+1]-=K*ht*n.inverseMass[h+1]*T,n.y[h+1]-=nt*ht*n.inverseMass[h+1]*T,n.z[h+1]-=rt*ht*n.inverseMass[h+1]*T}}}#E(t,e,n,i,s){const a=t.x[e+1]-t.x[e],o=t.y[e+1]-t.y[e],c=t.z[e+1]-t.z[e],l=n.x[i+1]-n.x[i],h=n.y[i+1]-n.y[i],u=n.z[i+1]-n.z[i],d=t.x[e]-n.x[i],f=t.y[e]-n.y[i],g=t.z[e]-n.z[i],x=a*a+o*o+c*c,m=a*l+o*h+c*u,p=l*l+h*h+u*u,_=a*d+o*f+c*g,v=l*d+h*f+u*g,S=x*p-m*m;let M=S>Yt?ae((m*v-p*_)/S,0,1):0,y=p>Yt?ae((m*M+v)/p,0,1):0;return x>Yt&&(M=ae((m*y-_)/x,0,1)),s.s=M,s.t=y,s}#S(t){for(let e=0;e<this.bodies.length;e++){const n=this.bodies[e];if(t.bodies&&!t.bodies.includes(n))continue;let i=t.lambdas.get(n);i||(i=new Float32Array(n.count),t.lambdas.set(n,i));for(let s=n.activeStart;s<=n.activeEnd;s++){const a=n.x[s]-t.startX,o=n.y[s]-t.startY,c=n.z[s]-t.startZ,l=a*t.axisX+o*t.axisY+c*t.axisZ;if(l<=0||l>=t.length){i[s]*=.8;continue}const h=t.startX+t.axisX*l,u=t.startY+t.axisY*l,d=t.startZ+t.axisZ*l,f=n.x[s]-h,g=n.y[s]-u,x=n.z[s]-d,m=ge(f,g,x),p=Math.max(0,t.innerRadius-n.nodeRadius[s]);if(m<=p||m<Yt){i[s]*=.8;continue}const _=n.inverseMass[s];if(_<=0)continue;const S=-(p-m)/_;i[s]+=S,n.x[s]-=f/m*S*_,n.y[s]-=g/m*S*_,n.z[s]-=x/m*S*_}}}#u(t){if(t.sleeping)return;const e=t.wallCompliance/(this.fixedDt*this.fixedDt),n=Math.max(0,t.activeStart,t.collisionStartSegment),i=Math.min(t.activeEnd,t.collisionEndSegment+1,t.segmentCount);for(let s=n;s<i;s++){if(!t.wallActive[s])continue;const a=t.wallT[s],o=1-a,c=a,l=t.x[s]*o+t.x[s+1]*c,h=t.y[s]*o+t.y[s+1]*c,u=t.z[s]*o+t.z[s+1]*c,d=t.wallNormalX[s],f=t.wallNormalY[s],g=t.wallNormalZ[s],x=Math.max(t.nodeRadius[s],t.nodeRadius[s+1]),m=(l-t.wallX[s])*d+(h-t.wallY[s])*f+(u-t.wallZ[s])*g-x;if(m>=0){t.wallLambda[s]*=.85;continue}const p=t.inverseMass[s]*o*o,_=t.inverseMass[s+1]*c*c,v=p+_+e;if(v<Yt)continue;let S=(-m-e*t.wallLambda[s])/v;const M=Math.max(0,t.wallLambda[s]+S);S=M-t.wallLambda[s],t.wallLambda[s]=M,t.x[s]+=d*S*t.inverseMass[s]*o,t.y[s]+=f*S*t.inverseMass[s]*o,t.z[s]+=g*S*t.inverseMass[s]*o,t.x[s+1]+=d*S*t.inverseMass[s+1]*c,t.y[s+1]+=f*S*t.inverseMass[s+1]*c,t.z[s+1]+=g*S*t.inverseMass[s+1]*c}}#m(t){if(t.sleeping)return;const e=t.wallCorrectionX,n=t.wallCorrectionY,i=t.wallCorrectionZ,s=t.wallCorrectionWeight;e.fill(0),n.fill(0),i.fill(0),s.fill(0);const a=Math.max(0,t.activeStart,t.collisionStartSegment),o=Math.min(t.activeEnd,t.collisionEndSegment+1,t.segmentCount);for(let l=a;l<o;l++){if(!t.wallActive[l])continue;const h=t.wallT[l],u=t.x[l]+(t.x[l+1]-t.x[l])*h,d=t.y[l]+(t.y[l+1]-t.y[l])*h,f=t.z[l]+(t.z[l+1]-t.z[l])*h,g=t.wallNormalX[l],x=t.wallNormalY[l],m=t.wallNormalZ[l],p=Math.max(t.nodeRadius[l],t.nodeRadius[l+1]),_=Math.max(0,Bd+p-((u-t.wallX[l])*g+(d-t.wallY[l])*x+(f-t.wallZ[l])*m));if(_<=.02)continue;const v=Math.max(.5,t.segmentLength),S=ae(Math.ceil(_/(v*.02)),4,32),M=l+h,y=Math.max(t.activeStart,Math.floor(M-S)),w=Math.min(t.activeEnd,Math.ceil(M+S));for(let A=y;A<=w;A++){if(t.inverseMass[A]<=0)continue;const E=Math.max(0,1-Math.abs(A-M)/(S+.5));e[A]+=g*_*E,n[A]+=x*_*E,i[A]+=m*_*E,s[A]+=E}}for(let l=t.activeStart;l<=t.activeEnd;l++){const h=s[l];h>Yt&&(e[l]/=h,n[l]/=h,i[l]/=h)}for(let l=0;l<28;l++){let h=!1;const u=(l&1)===1;for(let d=u?t.activeEnd-1:t.activeStart;u?d>=t.activeStart:d<t.activeEnd;d+=u?-1:1){const f=d+1,g=e[f]-e[d],x=n[f]-n[d],m=i[f]-i[d],p=g*g+x*x+m*m,_=Math.max(1e-5,t.restLength[d]*.02);if(p<=_*_)continue;const v=Math.sqrt(p),S=t.inverseMass[d],M=t.inverseMass[f],y=S+M;if(y<=Yt)continue;const w=(v-_)/v,A=w*S/y,E=w*M/y;e[d]+=g*A,n[d]+=x*A,i[d]+=m*A,e[f]-=g*E,n[f]-=x*E,i[f]-=m*E,h=!0}if(!h)break}for(let l=t.activeStart;l<=t.activeEnd;l++)t.x[l]+=e[l],t.y[l]+=n[l],t.z[l]+=i[l];const c=Math.min(o-1,t.activeEnd-1);if(c>=a&&t.wallActive[c]&&t.wallT[c]>.75&&t.inverseMass[c+1]>0){const l=t.wallT[c],h=t.x[c]+(t.x[c+1]-t.x[c])*l,u=t.y[c]+(t.y[c+1]-t.y[c])*l,d=t.z[c]+(t.z[c+1]-t.z[c])*l,f=t.wallNormalX[c],g=t.wallNormalY[c],x=t.wallNormalZ[c],m=Math.max(t.nodeRadius[c],t.nodeRadius[c+1]),p=Math.max(0,Bd+m-((h-t.wallX[c])*f+(u-t.wallY[c])*g+(d-t.wallZ[c])*x));if(p>0){const _=p/l;t.x[c+1]+=f*_,t.y[c+1]+=g*_,t.z[c+1]+=x*_}}}#g(t){if(t.sleeping)return;const e=1/this.fixedDt;let n=0;for(let i=t.activeStart;i<=t.activeEnd;i++){let s=t.x[i]-t.previousX[i],a=t.y[i]-t.previousY[i],o=t.z[i]-t.previousZ[i],c=0,l=0,h=0,u=0;i>0&&t.wallActive[i-1]&&(c+=t.wallLambda[i-1]*t.wallFriction,l+=t.wallNormalX[i-1],h+=t.wallNormalY[i-1],u+=t.wallNormalZ[i-1]),i<t.segmentCount&&t.wallActive[i]&&(c+=t.wallLambda[i]*t.wallFriction,l+=t.wallNormalX[i],h+=t.wallNormalY[i],u+=t.wallNormalZ[i]);const d=ge(l,h,u);if(d>Yt){l/=d,h/=d,u/=d;let f=s*l+a*h+o*u;f<0&&(s-=l*f,a-=h*f,o-=u*f,f=0);const g=s-l*f,x=a-h*f,m=o-u*f,p=ge(g,x,m);if(c>0&&p>Yt){const _=Math.min(p,c)/p;s-=g*_,a-=x*_,o-=m*_}}t.velocityX[i]=s*e,t.velocityY[i]=a*e,t.velocityZ[i]=o*e,n=Math.max(n,ge(t.velocityX[i],t.velocityY[i],t.velocityZ[i]))}n<t.sleepVelocity&&this.settledMaxPenetration<.01?t.sleepCounter++:t.sleepCounter=0,t.sleepCounter>=t.sleepFrames&&(t.sleeping=!0,t.velocityX.fill(0),t.velocityY.fill(0),t.velocityZ.fill(0))}#C(t){if(!(!Number.isFinite(t.maxSpeed)||t.maxSpeed<=0))for(let e=t.activeStart;e<=t.activeEnd;e++){const n=ge(t.velocityX[e],t.velocityY[e],t.velocityZ[e]);if(n<=t.maxSpeed||n<Yt)continue;const i=t.maxSpeed/n;t.velocityX[e]*=i,t.velocityY[e]*=i,t.velocityZ[e]*=i}}#w(t){if(!t.enabled||t.outerFollowsInnerCenterline)return;const e=t.innerBody,n=t.outerBody,i=Math.max(0,t.innerRadius-e.radius),s=ae(t.startNode,e.activeStart,e.activeEnd),a=ae(t.endNode,s,e.activeEnd),o=ae(t.outerStartNode,n.activeStart,n.activeEnd),c=Math.min(n.activeEnd,n.segmentCount);for(let l=s;l<=a;l++){const h=t.closestSegment[l];if(h<o||h>=c)continue;const u=n.x[h],d=n.y[h],f=n.z[h],g=n.x[h+1]-u,x=n.y[h+1]-d,m=n.z[h+1]-f,p=g*g+x*x+m*m,_=ae(((e.x[l]-u)*g+(e.y[l]-d)*x+(e.z[l]-f)*m)/Math.max(Yt,p),0,1),v=e.x[l]-(u+g*_),S=e.y[l]-(d+x*_),M=e.z[l]-(f+m*_),y=ge(v,S,M);if(y<Yt||y<i-.01&&t.lambdas[l]<=Yt)continue;const w=v/y,A=S/y,E=M/y,T=1-_,L=_,C=e.velocityX[l]-n.velocityX[h]*T-n.velocityX[h+1]*L,F=e.velocityY[l]-n.velocityY[h]*T-n.velocityY[h+1]*L,D=e.velocityZ[l]-n.velocityZ[h]*T-n.velocityZ[h+1]*L,N=C*w+F*A+D*E;if(N<=0)continue;const B=e.inverseMass[l]*t.innerResponse,G=n.inverseMass[h]*t.outerResponse*T*T,z=n.inverseMass[h+1]*t.outerResponse*L*L,H=B+G+z;if(H<Yt)continue;const j=N/H;e.velocityX[l]-=w*j*B,e.velocityY[l]-=A*j*B,e.velocityZ[l]-=E*j*B,n.velocityX[h]+=w*j*n.inverseMass[h]*t.outerResponse*T,n.velocityY[h]+=A*j*n.inverseMass[h]*t.outerResponse*T,n.velocityZ[h]+=E*j*n.inverseMass[h]*t.outerResponse*T,n.velocityX[h+1]+=w*j*n.inverseMass[h+1]*t.outerResponse*L,n.velocityY[h+1]+=A*j*n.inverseMass[h+1]*t.outerResponse*L,n.velocityZ[h+1]+=E*j*n.inverseMass[h+1]*t.outerResponse*L}}#s(t){if(t.sleeping||t.bendDamping<=0||t.count<3)return;const e=Math.max(1,t.activeStart+1),n=Math.min(t.count-1,t.activeEnd);for(let i=e;i<n;i++){if(t.inverseMass[i]<=0)continue;const s=t.x[i+1]-t.x[i-1],a=t.y[i+1]-t.y[i-1],o=t.z[i+1]-t.z[i-1],c=ge(s,a,o);if(c<Yt)continue;const l=s/c,h=a/c,u=o/c,d=(t.velocityX[i-1]+t.velocityX[i+1])*.5,f=(t.velocityY[i-1]+t.velocityY[i+1])*.5,g=(t.velocityZ[i-1]+t.velocityZ[i+1])*.5,x=t.velocityX[i]-d,m=t.velocityY[i]-f,p=t.velocityZ[i]-g,_=x*l+m*h+p*u;t.velocityX[i]-=(x-l*_)*t.bendDamping,t.velocityY[i]-=(m-h*_)*t.bendDamping,t.velocityZ[i]-=(p-u*_)*t.bendDamping}}#l(t){if(!t.enabled)return;const e=t.bodyA,n=t.bodyB,i=ae(t.startSegmentA,e.activeStart,e.segmentCount-1),s=ae(t.endSegmentA,i,Math.min(e.activeEnd-1,e.segmentCount-1)),a=ae(t.startSegmentB,n.activeStart,n.segmentCount-1),o=ae(t.endSegmentB,a,Math.min(n.activeEnd-1,n.segmentCount-1));for(let c=i;c<=s;c++)for(let l=a;l<=o;l++){const h=c*n.segmentCount+l;if(t.lambdas[h]<=Yt)continue;this.#E(e,c,n,l,this._segmentParameters);const u=this._segmentParameters.s,d=this._segmentParameters.t,f=1-u,g=u,x=1-d,m=d,p=e.x[c]*f+e.x[c+1]*g,_=e.y[c]*f+e.y[c+1]*g,v=e.z[c]*f+e.z[c+1]*g,S=n.x[l]*x+n.x[l+1]*m,M=n.y[l]*x+n.y[l+1]*m,y=n.z[l]*x+n.z[l+1]*m,w=p-S,A=_-M,E=v-y,T=ge(w,A,E);if(T<Yt)continue;const L=w/T,C=A/T,F=E/T,D=e.velocityX[c]*f+e.velocityX[c+1]*g-n.velocityX[l]*x-n.velocityX[l+1]*m,N=e.velocityY[c]*f+e.velocityY[c+1]*g-n.velocityY[l]*x-n.velocityY[l+1]*m,B=e.velocityZ[c]*f+e.velocityZ[c+1]*g-n.velocityZ[l]*x-n.velocityZ[l+1]*m,G=D*L+N*C+B*F;if(G>=0)continue;const z=e.inverseMass[c]*f*f,H=e.inverseMass[c+1]*g*g,j=n.inverseMass[l]*x*x,$=n.inverseMass[l+1]*m*m,Q=z+H+j+$;if(Q<Yt)continue;const V=-G/Q;e.velocityX[c]+=L*V*e.inverseMass[c]*f,e.velocityY[c]+=C*V*e.inverseMass[c]*f,e.velocityZ[c]+=F*V*e.inverseMass[c]*f,e.velocityX[c+1]+=L*V*e.inverseMass[c+1]*g,e.velocityY[c+1]+=C*V*e.inverseMass[c+1]*g,e.velocityZ[c+1]+=F*V*e.inverseMass[c+1]*g,n.velocityX[l]-=L*V*n.inverseMass[l]*x,n.velocityY[l]-=C*V*n.inverseMass[l]*x,n.velocityZ[l]-=F*V*n.inverseMass[l]*x,n.velocityX[l+1]-=L*V*n.inverseMass[l+1]*m,n.velocityY[l+1]-=C*V*n.inverseMass[l+1]*m,n.velocityZ[l+1]-=F*V*n.inverseMass[l+1]*m}}#D(t){let e=0,n=0,i=!0;for(let s=t.activeStart;s<Math.min(t.activeEnd,t.segmentCount);s++){const a=t.x[s+1]-t.x[s],o=t.y[s+1]-t.y[s],c=t.z[s+1]-t.z[s],l=ge(a,o,c);if(e=Math.max(e,Math.abs(l-t.restLength[s])/Math.max(Yt,t.restLength[s])),i=i&&Number.isFinite(l),s<=t.activeStart||s>=t.activeEnd-1)continue;const h=t.x[s]-t.x[s-1],u=t.y[s]-t.y[s-1],d=t.z[s]-t.z[s-1],f=t.x[s+1]-t.x[s],g=t.y[s+1]-t.y[s],x=t.z[s+1]-t.z[s],m=ge(h,u,d)*ge(f,g,x);m>Yt&&(n=Math.max(n,Math.acos(ae((h*f+u*g+d*x)/m,-1,1))))}return{id:t.id,sleeping:t.sleeping,finite:i,maxLengthError:e,maxBendAngleDegrees:n*180/Math.PI}}}function AM(r=140,t=0,e=null,n=yM){const o={radius:20,branchRadius:10,branchPoint:{x:0,y:-300,z:0},segments:[]},c={x:0,y:0,z:0},l={x:0,y:-300,z:0};o.main={start:c,end:l},o.segments.push({start:c,end:l,radius:20});function h(C){const F=Math.PI/6*C+t*C,D={x:o.branchPoint.x+Math.sin(F)*r,y:o.branchPoint.y-r,z:0};return{angle:F,end:D,length:r}}o.right=h(1),o.left=h(-1),o.segments.push({start:o.branchPoint,end:o.right.end,radius:10}),o.segments.push({start:o.branchPoint,end:o.left.end,radius:10});const u={x:-73,y:-383,z:14},d=new R(.24,.96,-.21).normalize(),f=o.left.length*.5,g=e??f,x={x:u.x-d.x*g,y:u.y-d.y*g,z:u.z-d.z*g},m={...u};o.sheath={start:x,end:m,radius:n,length:g,isSheath:!0},o.segments.push(o.sheath);for(const C of o.segments){const F=C.end.x-C.start.x,D=C.end.y-C.start.y,N=C.end.z-C.start.z,B=Math.sqrt(F*F+D*D+N*N)||1;C.length=B,C.volume=Math.PI*C.radius*C.radius*B}const p=new Map,_=[];function v(C){const F=`${C.x.toFixed(5)},${C.y.toFixed(5)},${C.z.toFixed(5)}`;if(p.has(F))return p.get(F);const D=_.length;return p.set(F,D),_.push({position:C,segments:[]}),D}o.segments.forEach((C,F)=>{C.startNode=v(C.start),C.endNode=v(C.end),_[C.startNode].segments.push(F),_[C.endNode].segments.push(F)}),o.nodes=_;const S=o.segments.map(()=>[]),M=o.segments.map(()=>null),y=1e-6,w=(C,F)=>Math.abs(C.x-F.x)<y&&Math.abs(C.y-F.y)<y&&Math.abs(C.z-F.z)<y;for(let C=0;C<o.segments.length;C++)for(let F=0;F<o.segments.length;F++)C!==F&&w(o.segments[C].end,o.segments[F].start)&&(S[C].push(F),M[F]=C);o.segmentGraph=S;for(let C=0;C<o.segments.length;C++)o.segments[C].parent=M[C];const A=85,E={},T=C=>{const F=C.end.x-C.start.x,D=C.end.y-C.start.y,N=C.end.z-C.start.z,B=Math.sqrt(F*F+D*D+N*N)||1;return{x:F/B,y:D/B,z:N/B}};function L(C,F){const D=o.segments[C],N=T(D);D.flowDir=N,D.flowSpeed=F;const B=S[C];if(E[C]={dir:N,speed:F,children:B},B.length){let G=0;for(const z of B)G+=o.segments[z].radius;for(const z of B){const H=F*(o.segments[z].radius/G);L(z,H)}}}for(let C=0;C<o.segments.length;C++)M[C]===null&&L(C,A);return o.flow=E,{vessel:o}}const CM=2,gl=1/15,RM=.58,bM=.29,ji=(r,t,e)=>Math.min(e,Math.max(t,r)),xl=(r,t,e)=>r+(t-r)*e;class PM{constructor(t,e,n,i,s={}){this.ecgCanvas=t,this.bpCanvas=e,this.hrElem=n,this.bpElem=i,this.spo2Elem=s.spo2Elem||null,this.mapElem=s.mapElem||null,this.rrElem=s.rrElem||null,this.rhythmElem=s.rhythmElem||null,this.clockElem=s.clockElem||null,this.ecgCtx=t.getContext("2d"),this.bpCtx=e.getContext("2d"),this.ecgCanvasState=this.#y("#020303","#000000"),this.bpCanvasState=this.#y("#030202","#000000"),this.baselineDash=[6,8],this.ecgSampleRate=250,this.bpSampleRate=50,this.ecgBufferLength=this.ecgSampleRate*10,this.bpBufferLength=this.bpSampleRate*10,this.ecgData=new Float32Array(this.ecgBufferLength),this.bpData=new Float32Array(this.bpBufferLength),this.bpData.fill(100),this.ecgCursor=0,this.bpCursor=0,this.drawAccumulator=gl,this.lastReadouts=Object.create(null),this.lastClockSecond=-1,this.clockLabel="00:00",this.time=0,this.cycleTime=0,this.variabilitySeed=Math.random()*Math.PI*2,this.baseHeartRate=75,this.heartRate=this.baseHeartRate,this.beatInterval=60/this.heartRate,this.ecgAccumulator=0,this.bpAccumulator=0,this.currentHR=this.heartRate,this.baselineSystolic=120,this.baselineDiastolic=80,this.waveSystolic=this.baselineSystolic,this.waveDiastolic=this.baselineDiastolic,this.systolic=120,this.diastolic=80,this.meanPressure=93,this.spo2=98,this.spo2Target=this.spo2,this.respiratoryRate=14,this.respiratoryRateTarget=this.respiratoryRate,this.bpMax=0,this.bpMin=1/0,this.ecgTemplate=this.#i(),this.ecgTemplateIndex=0,this.ecgSamplesSinceBeat=0,this.ecgSamplesToNextBeat=this.#o(),this.bpTemplate=this.#p()}setHeartRate(t){this.baseHeartRate=t,this.heartRate=t,this.beatInterval=60/this.heartRate,this.currentHR=t}update(t){this.ecgAccumulator+=t,this.bpAccumulator+=t,this.time+=t,this.cycleTime+=t;const e=1/this.ecgSampleRate;for(;this.ecgAccumulator>=e;){this.ecgAccumulator-=e;const i=this.#f();this.ecgData[this.ecgCursor]=i,this.ecgCursor=(this.ecgCursor+1)%this.ecgBufferLength}const n=1/this.bpSampleRate;for(;this.bpAccumulator>=n;){this.bpAccumulator-=n;const i=this.cycleTime/this.beatInterval%1,s=Math.floor(i*this.bpTemplate.length),a=this.#a(this.bpTemplate[s]);this.bpData[this.bpCursor]=a,this.bpCursor=(this.bpCursor+1)%this.bpBufferLength,a>this.bpMax&&(this.bpMax=a),a<this.bpMin&&(this.bpMin=a)}this.cycleTime>=this.beatInterval&&(this.currentHR=60/this.beatInterval,this.systolic=this.bpMax,this.diastolic=this.bpMin,this.meanPressure=this.diastolic+(this.systolic-this.diastolic)/3,this.cycleTime-=this.beatInterval,this.bpMax=0,this.bpMin=1/0,this.#r()),this.#c(t),this.drawAccumulator+=t,!(this.drawAccumulator<gl)&&(this.drawAccumulator%=gl,this.#t("hr",this.hrElem,Math.round(this.currentHR)),this.#e(),this.#t("spo2",this.spo2Elem,Math.round(this.spo2)),this.#t("map",this.mapElem,Math.round(this.meanPressure)),this.#t("rr",this.rrElem,Math.round(this.respiratoryRate)),this.#t("rhythm",this.rhythmElem,this.#w()),this.#t("clock",this.clockElem,this.#s()),this.#A(),this.#v())}#t(t,e,n){!e||this.lastReadouts[t]===n||(e.textContent=n,this.lastReadouts[t]=n)}#e(){if(!this.bpElem)return;const t=Math.round(this.systolic),e=Math.round(this.diastolic),n=t*256+e;this.lastReadouts.bp!==n&&(this.bpElem.textContent=`${t}/${e}`,this.lastReadouts.bp=n)}#r(){const t=Math.sin(this.time*.34+this.variabilitySeed),e=Math.sin(this.time*.11+this.variabilitySeed*.7),n=(Math.random()-.5)*1.8,i=(Math.random()-.5)*2.2;this.heartRate=ji(this.baseHeartRate+t*2.2+e*1.4+n,58,96),this.beatInterval=60/this.heartRate,this.ecgSamplesToNextBeat=this.#o(),this.currentHR=xl(this.currentHR,this.heartRate,.75),this.waveSystolic=ji(this.baselineSystolic+t*3.2+e*2+i,106,134),this.waveDiastolic=ji(this.baselineDiastolic+t*1.6+e*1.2+i*.45,68,88)}#f(){const t=this.ecgTemplateIndex<this.ecgTemplate.length?this.ecgTemplate[this.ecgTemplateIndex]:0;return this.ecgTemplateIndex+=1,this.ecgSamplesSinceBeat+=1,this.ecgSamplesSinceBeat>=this.ecgSamplesToNextBeat&&(this.ecgTemplateIndex=0,this.ecgSamplesSinceBeat=0,this.ecgSamplesToNextBeat=this.#o()),t}#o(){return Math.max(this.ecgTemplate?.length||1,Math.round(this.beatInterval*this.ecgSampleRate))}#c(t){const e=Math.sin(this.time*.31+this.variabilitySeed),n=Math.sin(this.time*.07+this.variabilitySeed*1.9),i=98+e*.9+n*.65,s=14+e*.9+n*.5;this.spo2Target=ji(i,96,100),this.respiratoryRateTarget=ji(s,11,18),this.spo2=xl(this.spo2,this.spo2Target,ji(t*1.4,0,1)),this.respiratoryRate=xl(this.respiratoryRate,this.respiratoryRateTarget,ji(t*.8,0,1))}#T(t){const e=(n,i,s)=>{const a=(t-n)/i;return s*Math.exp(-.5*a**2)};return e(.095,.022,.08)+e(.178,.009,-.12)+e(.198,.007,.82)+e(.222,.012,-.18)+e(.42,.062,.17)}#h(t){const i=1/(1+Math.exp(-(t-.11)/.018)),s=Math.exp(-Math.max(t-.16,0)/.36),a=-5.5*Math.exp(-.5*((t-.33)/.018)**2),o=3.2*Math.exp(-.5*((t-.37)/.026)**2);return 80+40*i*s+a+o}#a(t){const e=ji((t-80)/40,0,1.25);return this.waveDiastolic+e*(this.waveSystolic-this.waveDiastolic)}#i(){const e=Math.round(.62*this.ecgSampleRate),n=[];for(let i=0;i<e;i++)n.push(this.#T(i/this.ecgSampleRate));return n}#p(){const t=[];for(let e=0;e<this.bpSampleRate;e++){const n=e/this.bpSampleRate;t.push(this.#h(n))}return t}#A(){const t=this.ecgCtx,e=this.#E(this.ecgCanvas,t,this.ecgCanvasState),n=e.w,i=e.h,s=this.ecgData.length;this.#S(t,n,i,e.backgroundGradient);const a=i*RM,o=i*bM,c=Math.max(2,Math.min(s,Math.ceil(n)));this.#m(t,n,i,a,"rgba(82, 118, 102, 0.32)"),t.beginPath();for(let l=0;l<c;l++){const h=Math.floor(l*s/c),u=Math.max(h+1,Math.floor((l+1)*s/c));let d=0;for(let x=h;x<u;x++){let m=this.ecgCursor+x;m>=s&&(m-=s);const p=this.ecgData[m];Math.abs(p)>Math.abs(d)&&(d=p)}const f=l/(c-1)*n,g=a-d*o;l===0?t.moveTo(f,g):t.lineTo(f,g)}this.#g(t,"#39e75f",1.35),this.#C(t,n,i,e.markerGradient,"#39e75f")}#v(){const t=this.bpCtx,e=this.#E(this.bpCanvas,t,this.bpCanvasState),n=e.w,i=e.h,s=this.bpData.length,a=Math.max(2,Math.min(s,Math.ceil(n)));this.#S(t,n,i,e.backgroundGradient),this.#m(t,n,i,i-45/85*i,"rgba(120, 88, 88, 0.3)"),t.beginPath();for(let o=0;o<a;o++){const c=Math.floor(o*s/a),l=Math.max(c+1,Math.floor((o+1)*s/a));let h=0;for(let g=c;g<l;g++){let x=this.bpCursor+g;x>=s&&(x-=s),h+=this.bpData[x]}const u=h/(l-c),d=o/(a-1)*n,f=i-(u-55)/85*i;o===0?t.moveTo(d,f):t.lineTo(d,f)}this.#g(t,"#f04d4d",1.35),this.#C(t,n,i,e.markerGradient,"#f04d4d")}#y(t,e){return{w:0,h:0,dpr:0,topColor:t,bottomColor:e,backgroundGradient:null,markerGradient:null}}#E(t,e,n){const i=Math.max(1,t.clientWidth||t.width),s=Math.max(1,t.clientHeight||t.height),a=Math.min(window.devicePixelRatio||1,CM),o=Math.round(i*a),c=Math.round(s*a);if((t.width!==o||t.height!==c)&&(t.width=o,t.height=c),e.setTransform(a,0,0,a,0,0),n.w!==i||n.h!==s||n.dpr!==a||!n.backgroundGradient){const l=e.createLinearGradient(0,0,0,s);l.addColorStop(0,n.topColor),l.addColorStop(1,n.bottomColor);const h=i-10.5,u=e.createLinearGradient(h-20,0,h+4,0);u.addColorStop(0,"rgba(255,255,255,0)"),u.addColorStop(1,"rgba(210,220,218,0.12)"),n.w=i,n.h=s,n.dpr=a,n.backgroundGradient=l,n.markerGradient=u}return n}#S(t,e,n,i){t.clearRect(0,0,e,n),t.fillStyle=i,t.fillRect(0,0,e,n),this.#u(t,e,n)}#u(t,e,n){t.save(),t.strokeStyle="rgba(88, 112, 106, 0.16)",t.lineWidth=1;for(let i=.5;i<e;i+=32)t.beginPath(),t.moveTo(i,0),t.lineTo(i,n),t.stroke();for(let i=.5;i<n;i+=24)t.beginPath(),t.moveTo(0,i),t.lineTo(e,i),t.stroke();t.strokeStyle="rgba(88, 112, 106, 0.26)";for(let i=.5;i<e;i+=160)t.beginPath(),t.moveTo(i,0),t.lineTo(i,n),t.stroke();t.restore()}#m(t,e,n,i,s){t.save(),t.strokeStyle=s,t.setLineDash(this.baselineDash),t.lineWidth=1,t.beginPath(),t.moveTo(0,i),t.lineTo(e,i),t.stroke(),t.restore()}#g(t,e,n){t.save(),t.lineJoin="round",t.lineCap="round",t.strokeStyle=e,t.lineWidth=n,t.stroke(),t.restore()}#C(t,e,n,i,s){const a=e-10.5;t.save(),t.fillStyle=i,t.fillRect(Math.max(0,a-20),0,24,n),t.strokeStyle=s,t.globalAlpha=.85,t.lineWidth=1,t.beginPath(),t.moveTo(a,8),t.lineTo(a,n-8),t.stroke(),t.restore()}#w(){return this.currentHR>=105?"TACHY":this.currentHR<=50?"BRADY":this.meanPressure<65?"LOW MAP":"SINUS"}#s(){const t=Math.floor(this.time);if(t===this.lastClockSecond)return this.clockLabel;const e=Math.floor(t/60).toString().padStart(2,"0"),n=(t%60).toString().padStart(2,"0");return this.lastClockSecond=t,this.clockLabel=`${e}:${n}`,this.clockLabel}}function Ze(r,t,e,n,i,s=new un){const a=new jt(new Dn(r,t,e),n);return a.position.copy(i),a.rotation.copy(s),a}function hi(r,t,e,n,i,s=new un,a=40){const o=new jt(new Ps(r,t,e,a),n);return o.position.copy(i),o.rotation.copy(s),o}function zd(r,t,e,n,i=new un){const s=new jt(new gr(r,t,10,24),e);return s.position.copy(n),s.rotation.copy(i),s}function _l(r,t,e,n,i,s,a=0){const o=[];for(let l=0;l<=96;l++){const h=he.degToRad(t+(e-t)*l/96);o.push(new R(n,r*Math.sin(h),a+r*Math.cos(h)))}const c=new kh(o);return new jt(new xc(c,128,i,18,!1),s)}function LM(){const r=new Le,t=new Le,e=new Le,n=new Le,i=new R(10,22,0),s=new En({color:14542314,roughness:.42,metalness:.08}),a=new En({color:15922678,roughness:.36,metalness:.04}),o=new En({color:10332852,roughness:.48,metalness:.28}),c=new En({color:4739933,roughness:.62,metalness:.25}),l=new En({color:1383200,roughness:.75}),h=new En({color:15331571,roughness:.34}),u=new Fe({color:12773623}),d=new En({color:15002092,roughness:.4,metalness:.06}),f=new Fe({color:9559551,transparent:!0,opacity:.16,depthWrite:!1}),g=new Fe({color:4380671});r.add(Ze(118,12,58,l,new R(10,-105,-82))),r.add(Ze(78,52,62,s,new R(10,-72,-82))),r.add(Ze(84,18,62,c,new R(10,-103,-82))),r.add(hi(12,12,10,l,new R(-34,-108,-108),new un(Math.PI/2,0,0),32)),r.add(hi(12,12,10,l,new R(54,-108,-108),new un(Math.PI/2,0,0),32)),r.add(hi(10,10,8,l,new R(-34,-108,-56),new un(Math.PI/2,0,0),32)),r.add(hi(10,10,8,l,new R(54,-108,-56),new un(Math.PI/2,0,0),32)),r.add(hi(16,18,70,c,new R(10,-37,-82))),t.add(hi(12,14,96,o,new R(10,-14,-82))),t.add(Ze(62,24,52,s,new R(10,38,-82))),t.add(Ze(28,10,44,c,new R(10,24,-82))),t.add(zd(12,34,s,new R(10,37,-82),new un(0,0,Math.PI/2))),t.add(Ze(48,26,40,s,new R(10,37,-82))),t.add(Ze(54,18,28,s,new R(10,29,-86))),t.add(Ze(34,22,34,s,new R(10,20,-86))),t.add(hi(21,21,18,c,new R(10,26,-82),new un(Math.PI/2,0,0),48)),t.add(hi(25,25,18,c,new R(10,22,-86),new un(Math.PI/2,0,0),48)),t.add(Ze(46,34,24,c,new R(10,22,-86))),t.add(hi(5,6,28,o,new R(28,79,-82))),t.add(Ze(46,24,6,a,new R(28,96,-82),new un(he.degToRad(-8),0,0))),t.add(Ze(31,16,2,new Fe({color:1450543}),new R(28,96,-78))),t.add(zd(3.2,30,o,new R(38,19,-52),new un(Math.PI/2,0,0))),e.position.copy(i),t.add(e),r.add(t);const x=86,m=0,p=0,_=58,v=_l(x,_,360-_,m,6.2,a,p),S=_l(x+8,_+2,360-_-2,m-4.5,1.8,o,p),M=_l(x-8,_+2,360-_-2,m+4.5,1.8,o,p);e.add(v,S,M);const y=he.degToRad(_),w=x*Math.sin(y),A=-w,E=0,T=p+x*Math.cos(y),L=(T+E)*.5,C=Math.abs(E-T)+12;e.add(Ze(48,13,C,a,new R(m,w,L))),e.add(Ze(48,13,C,a,new R(m,A,L))),e.add(Ze(42,14,16,a,new R(m,w,T))),e.add(Ze(42,14,16,a,new R(m,A,T)));const F=Ze(50,16,42,h,new R(m,w,E));n.add(F);const D=Ze(40,2,34,u,new R(m,w-9,E));n.add(D);const N=Ze(58,22,44,d,new R(m,A,E));n.add(N);const B=Ze(36,9,26,c,new R(m,A+17,E));n.add(B);const G=hi(15,22,w-A-20,f,new R(m,0,E));n.add(G),e.add(n);const z=new jt(new Wh(7.5,9,48),g);return z.position.set(m,0,E),z.rotation.x=Math.PI/2,e.add(z),{group:r,gantryGroup:e,liftGroup:t,detectorAssembly:n}}function DM(){const r=new Le,t=new Le;r.userData.slideGroup=t;const e=new En({color:10134701,roughness:.45}),n=new En({color:14081507,roughness:.3,metalness:.15}),i=new En({color:5857899,roughness:.6,metalness:.2}),s=new En({color:2503490,roughness:.55}),a=new En({color:2064266,roughness:.7}),o=new En({color:14202011,roughness:.65}),c=new jt(new Dn(86,10,58),i);c.position.set(0,-88,0),r.add(c);const l=new jt(new Ps(8,10,76,32),i);l.position.set(0,-45,0),r.add(l);const h=new jt(new Dn(66,10,44),i);h.position.set(0,-8,0),r.add(h);const u=new jt(new Dn(230,8,58),e);u.position.set(0,0,0),t.add(u);const d=new jt(new Dn(218,5,48),s);d.position.set(0,6.5,0),t.add(d);const f=new jt(new Dn(224,3,3),n);f.position.set(0,7,-32),t.add(f);const g=f.clone();g.position.z=32,t.add(g);const x=new jt(new gr(17,64,10,22),a);x.rotation.z=Math.PI/2,x.position.set(16,23,0),t.add(x);const m=new jt(new Dn(62,8,42),a);m.position.set(14,20,0),t.add(m);const p=new jt(new ii(13,28,18),o);p.scale.set(1.05,.82,.9),p.position.set(-50,21,0),t.add(p);const _=new jt(new Dn(32,5,32),new En({color:15265522,roughness:.75}));_.position.set(-50,13,0),t.add(_);const v=new jt(new gr(7,62,8,16),a);v.rotation.z=Math.PI/2,v.position.set(70,18,-10),t.add(v);const S=v.clone();S.position.z=10,t.add(S);const M=new jt(new gr(4.5,58,8,14),o);M.rotation.z=Math.PI/2,M.position.set(4,17,-31),t.add(M);const y=M.clone();return y.position.z=31,t.add(y),r.add(t),r}let ti,dr,fr,fa,sh,Od,Wo,rh;const IM=new R(0,24,-30);function NM(){const r=document.getElementById("carm-preview");if(!r)return null;r.replaceChildren(),ti=new Rr,ti.background=new Qt(131843);const t=new $v(14542820,.72);ti.add(t);const e=new od(16777215,.85);e.position.set(120,180,160),ti.add(e);const n=new od(10140083,.24);n.position.set(-160,40,-130),ti.add(n);const i=r.clientWidth,s=r.clientHeight;dr=new In(39,i/s,.1,1e3),dr.position.set(268,146,289),dr.lookAt(IM),ti.add(dr),fr=new Oh({antialias:!0,alpha:!0}),fr.setSize(i,s),fr.setPixelRatio(Math.min(window.devicePixelRatio||1,2)),r.appendChild(fr.domElement);const a=new Jv(300,12,4741719,1910052);a.position.y=-94,ti.add(a),Wo=DM(),ti.add(Wo),fa=new Le;const{group:o,gantryGroup:c,liftGroup:l,detectorAssembly:h}=LM();return sh=c,Od=l,rh=h,fa.add(o),ti.add(fa),Rp(),{group:fa,gantry:sh,detectorAssembly:rh,lift:Od,table:Wo}}function Rp(){!fr||!ti||!dr||fr.render(ti,dr)}function FM(r,t,e,n,i,s,a,o,c=()=>{}){const l=document.getElementById("carmX"),h=document.getElementById("carmY"),u=document.getElementById("carmZ"),d=document.getElementById("carmDetDist"),f=document.getElementById("carmZUp"),g=document.getElementById("carmZDown"),x=document.getElementById("carmRollLeft"),m=document.getElementById("carmRollRight"),p=document.getElementById("carmAngleReset"),_=document.getElementById("carmLao30"),v=document.getElementById("carmRao30"),S=document.getElementById("carmYawReadout"),M=document.getElementById("carmPitchReadout"),y=document.getElementById("carmRollReadout");[l,h,u,d].filter(Boolean).forEach(st=>st.addEventListener("change",()=>st.blur()));let A=0,E=0,T=0,L=parseFloat(l.value),C=parseFloat(h.value),F=parseFloat(u.value),D=parseFloat(d.value),N=!1,B=0;const G=L,z=C,H=F,j=D,$=10,Q=new R(1,0,0),V=new R(0,0,1),K=new R(0,1,0),nt=new ni,rt=new ni,ot=new ni;function _t(){const st=H-(F-H);return new R(t.branchPoint.x+L,t.branchPoint.y+C,t.branchPoint.z+st)}function ht(st){return Math.round(he.radToDeg(st))}function ut(st){return st===0?"AP 0°":`${st>0?"LAO":"RAO"} ${Math.abs(st)}°`}function Ct(st){return st===0?"CRA 0°":`${st>0?"CRA":"CAU"} ${Math.abs(st)}°`}function W(){S&&(S.textContent=ut(ht(A))),M&&(M.textContent=Ct(ht(E))),y&&(y.textContent=`Roll ${ht(T)}°`)}function Nt(){const st=_t(),Xt=new R().setFromSpherical(new Kv(1,Math.PI/2-E,A)).normalize(),P=st.clone().addScaledVector(Xt,e),k=st.clone().addScaledVector(Xt,-D);r.position.copy(P),r.up.set(0,1,0),r.lookAt(k),r.rotateZ(T);const Z=L-G,q=C-z,Y=F-H;n&&n.position.set($,0,0),a&&(a.position.y=Y*.12),o&&(o.userData.slideGroup||o).position.set(q*.08,0,Z*.08),i&&(nt.setFromAxisAngle(Q,-A),rt.setFromAxisAngle(V,E),i.quaternion.copy(rt).multiply(nt)),s&&(ot.setFromAxisAngle(K,T),s.quaternion.copy(ot)),(n||i||s||a||o)&&c(),W(),B++}Nt(),l.addEventListener("input",st=>{N||(L=parseFloat(st.target.value),Nt())}),h.addEventListener("input",st=>{N||(C=parseFloat(st.target.value),Nt())}),u.addEventListener("input",st=>{N||(F=parseFloat(st.target.value),Nt())}),d.addEventListener("input",st=>{N||(D=parseFloat(st.target.value),Nt())});const Tt=document.getElementById("positionJoystick"),Pt=document.getElementById("positionJoystickHandle"),yt=document.getElementById("angleJoystick"),Kt=document.getElementById("angleJoystickHandle");let bt=0,I=0,b=0,X=!1,et=null,J=null;const tt=he.degToRad(90),St=he.degToRad(45),lt=he.degToRad(90),gt=he.degToRad(22),At=he.degToRad(18),Ht=he.degToRad(18),it=he.degToRad(24),ce=he.degToRad(24),Jt=.22;function Gt(st,Xt){const P=Math.abs(st);return P<Xt?0:Math.sign(st)*((P-Xt)/(1-Xt))}function Lt(st,Xt,P,k,{resetOnRelease:Z=!0}={}){if(!st||!Xt)return;const q=Xt.offsetWidth/2,Y=st.offsetWidth/2-q;let at=!1;const Et="transform 0.2s ease-out";function Dt(xt,Vt){if(N)return;const kt=st.getBoundingClientRect();let Ot=xt-kt.left-kt.width/2,fe=Vt-kt.top-kt.height/2;const Je=Math.hypot(Ot,fe);if(Je>Y){const Re=Y/Je;Ot*=Re,fe*=Re}Xt.style.transform=`translate(-50%, -50%) translate(${Ot}px, ${fe}px)`,P(Ot/Y,fe/Y)}st.addEventListener("mousedown",xt=>{N||(at=!0,Xt.style.transition="none",Dt(xt.clientX,xt.clientY))}),window.addEventListener("mousemove",xt=>{at&&Dt(xt.clientX,xt.clientY)}),window.addEventListener("mouseup",()=>{at&&(at=!1,Xt.style.transition=Et,Z&&(Xt.style.transform="translate(-50%, -50%)"),k())}),st.addEventListener("pointerdown",xt=>{N||(at=!0,st.setPointerCapture?.(xt.pointerId),Xt.style.transition="none",Dt(xt.clientX,xt.clientY))}),st.addEventListener("pointermove",xt=>{at&&Dt(xt.clientX,xt.clientY)}),st.addEventListener("pointerup",xt=>{at&&(at=!1,st.releasePointerCapture?.(xt.pointerId),Xt.style.transition=Et,Z&&(Xt.style.transform="translate(-50%, -50%)"),k())}),st.addEventListener("pointercancel",xt=>{at&&(at=!1,st.releasePointerCapture?.(xt.pointerId),Xt.style.transition=Et,Z&&(Xt.style.transform="translate(-50%, -50%)"),k())}),st.addEventListener("touchstart",xt=>{if(N)return;xt.preventDefault(),at=!0,Xt.style.transition="none";const Vt=xt.touches[0];Dt(Vt.clientX,Vt.clientY)}),window.addEventListener("touchmove",xt=>{if(!at)return;const Vt=xt.touches[0];Dt(Vt.clientX,Vt.clientY)},{passive:!1}),window.addEventListener("touchend",()=>{at&&(at=!1,Xt.style.transition=Et,Z&&(Xt.style.transform="translate(-50%, -50%)"),k())})}let vt=0,Ft=0,te=0;const le=parseFloat(l.min),Wt=parseFloat(l.max),ct=parseFloat(h.min),U=parseFloat(h.max),dt=parseFloat(u.min),ft=parseFloat(u.max),zt=(Wt-le)*.18,Bt=(U-ct)*.18,de=(ft-dt)*.18;let ue=performance.now();function ve(st,Xt){return Math.abs(st)<=Xt?0:st-Math.sign(st)*Xt}function we(st){const Xt=(st-ue)/1e3;if(ue=st,N){requestAnimationFrame(we);return}let P=!1;if((vt!==0||Ft!==0)&&(L=Math.min(Math.max(L+vt*zt*Xt,le),Wt),C=Math.min(Math.max(C+Ft*Bt*Xt,ct),U),l.value=Math.round(L),h.value=Math.round(C),P=!0),te!==0){const k=he.clamp(F+te*de*Xt,dt,ft);P=P||k!==F,F=k,u.value=Math.round(F)}if((I!==0||b!==0)&&(A=Math.min(Math.max(A+I*gt*Xt,-tt),tt),E=Math.min(Math.max(E+b*At*Xt,-St),St),P=!0),bt!==0&&(T=Math.min(Math.max(T+bt*Ht*Xt,-lt),lt),P=!0),X){I=0,b=0,et=null,J?.classList.remove("active"),J=null,bt=0;const k=it*Xt,Z=ve(A,k),q=ve(E,k),Y=ve(T,k);P=P||Z!==A||q!==E||Y!==T,A=Z,E=q,T=Y}if(et!==null){I=0;const k=ce*Xt,Z=et-A,q=Math.abs(Z)<=k?et:A+Math.sign(Z)*k;P=P||q!==A,A=he.clamp(q,-tt,tt)}P&&Nt(),requestAnimationFrame(we)}requestAnimationFrame(we);function se(st){N||(te=st)}function Ce(){te=0}f&&g&&(f.addEventListener("mousedown",()=>se(1)),g.addEventListener("mousedown",()=>se(-1)),window.addEventListener("mouseup",Ce),f.addEventListener("touchstart",st=>{st.preventDefault(),se(1)}),g.addEventListener("touchstart",st=>{st.preventDefault(),se(-1)}),window.addEventListener("touchend",Ce),window.addEventListener("touchcancel",Ce));function Ye(st){N||(bt=st)}function Si(){bt=0}x&&m&&(x.addEventListener("mousedown",()=>Ye(-1)),m.addEventListener("mousedown",()=>Ye(1)),window.addEventListener("mouseup",Si),x.addEventListener("touchstart",st=>{st.preventDefault(),Ye(-1)}),m.addEventListener("touchstart",st=>{st.preventDefault(),Ye(1)}),window.addEventListener("touchend",Si),window.addEventListener("touchcancel",Si));function Ds(st){st?.preventDefault?.(),!N&&(X=!0,I=0,b=0,bt=0,Kt&&(Kt.style.transition="transform 0.2s ease-out",Kt.style.transform="translate(-50%, -50%)"),p?.classList.add("active"))}function Sn(){X=!1,p?.classList.remove("active")}p&&(p.addEventListener("pointerdown",st=>{p.setPointerCapture?.(st.pointerId),Ds(st)}),p.addEventListener("pointerup",st=>{p.releasePointerCapture?.(st.pointerId),Sn()}),p.addEventListener("pointercancel",Sn),p.addEventListener("pointerleave",Sn),p.addEventListener("click",st=>st.preventDefault()),window.addEventListener("blur",Sn));function Rn(st,Xt,P){P?.preventDefault?.(),!N&&(X=!1,p?.classList.remove("active"),I=0,b=0,et=he.clamp(st,-tt,tt),J&&J!==Xt&&J.classList.remove("active"),J=Xt,J?.classList.add("active"),Kt&&(Kt.style.transition="transform 0.2s ease-out",Kt.style.transform="translate(-50%, -50%)"))}function qe(){et=null,J?.classList.remove("active"),J=null}function Un(st,Xt){st&&(st.addEventListener("pointerdown",P=>{st.setPointerCapture?.(P.pointerId),Rn(Xt,st,P)}),st.addEventListener("pointerup",P=>{st.releasePointerCapture?.(P.pointerId),qe()}),st.addEventListener("pointercancel",qe),st.addEventListener("pointerleave",qe),st.addEventListener("click",P=>P.preventDefault()))}Un(_,he.degToRad(30)),Un(v,he.degToRad(-30)),window.addEventListener("blur",qe),Lt(Tt,Pt,(st,Xt)=>{vt=-Xt,Ft=-st},()=>{vt=0,Ft=0}),Lt(yt,Kt,(st,Xt)=>{qe(),I=Gt(-Xt,Jt),b=Gt(-st,Jt)},()=>{I=0,b=0});function Vi(){vt=0,Ft=0,te=0,bt=0,I=0,b=0,X=!1,et=null,J?.classList.remove("active"),J=null,p?.classList.remove("active"),Pt&&(Pt.style.transform="translate(-50%, -50%)"),Kt&&(Kt.style.transform="translate(-50%, -50%)")}function os(){Vi(),A=0,E=0,T=0,L=G,C=z,F=H,D=j,l.value=String(Math.round(L)),h.value=String(Math.round(C)),u.value=String(Math.round(F)),d.value=String(Math.round(D)),Nt()}return{reset:os,getRevision:()=>B,setLocked(st){N=st===!0,N&&Vi()}}}class Gd{constructor({emptyThresholdCm:t=.05}={}){this.emptyThresholdCm=t,this.insertedCm=0,this.active=!1}updateLength(t){return this.insertedCm=Math.max(0,Number.isFinite(t)?t:0),this.insertedCm<=this.emptyThresholdCm&&(this.active=!1),this}toggle(){return this.active?this.active=!1:this.insertedCm>this.emptyThresholdCm&&(this.active=!0),this.active}cancel(){return this.active=!1,this}get command(){return this.active?-1:0}get disabled(){return!this.active&&this.insertedCm<=this.emptyThresholdCm}}function BM(r){const{camera:t,cameraRadius:e,vessel:n,voxelGroup:i,displayMaterial:s,blendMaterial:a,wireMaterial:o,onStartInjection:c,onStopInjection:l,onModeChange:h,onDebugLayerChange:u,onStartBrowserBenchmark:d,onStopBrowserBenchmark:f}=r,g=new PM(document.getElementById("ecgCanvas"),document.getElementById("bpCanvas"),document.getElementById("hrValue"),document.getElementById("bpValue"),{spo2Elem:document.getElementById("spo2Value"),mapElem:document.getElementById("mapValue"),rrElem:document.getElementById("rrValue"),rhythmElem:document.getElementById("monitorRhythm"),clockElem:document.getElementById("monitorClock")}),x=NM(),m=FM(t,n,e,x?.group||fa,x?.gantry||sh,x?.detectorAssembly||rh,x?.lift,x?.table||Wo,Rp),p=document.getElementById("stiffness"),_=document.getElementById("staticFriction"),v=document.getElementById("kineticFriction"),S=document.getElementById("smoothIterations"),M=document.getElementById("modeToggle"),y=document.getElementById("renderVoxels"),w=document.getElementById("showDebugStlModel"),A=document.getElementById("showDebugLumenCast"),E=document.getElementById("showDebugSections"),T=document.getElementById("showDebugCenterline"),L=document.getElementById("showDebugCapsules"),C=document.getElementById("injectContrast"),F=document.getElementById("stopInjection"),D=document.getElementById("injRate"),N=document.getElementById("injDuration"),B=document.getElementById("injVolume"),G=document.getElementById("autoExposureToggle"),z=document.getElementById("persistence"),H=document.getElementById("pulseRate"),j=document.getElementById("noiseLevel"),$=document.getElementById("scatterStrength"),Q=document.getElementById("collimation"),V=document.getElementById("imageBrightness"),K=document.getElementById("imageContrast"),nt=document.getElementById("edgeEnhancement"),rt=document.getElementById("boneVisibility"),ot=document.getElementById("opacityScale"),_t=document.getElementById("gain"),ht=document.getElementById("insertedLength"),ut=document.getElementById("catheterLength"),Ct=document.getElementById("guidewireAutoWithdraw"),W=document.getElementById("catheterAutoWithdraw"),Nt=document.getElementById("catheterAdvance"),Tt=document.getElementById("catheterWithdraw"),Pt=document.getElementById("catheterRotateLeft"),yt=document.getElementById("catheterRotateRight"),Kt=document.getElementById("catheterType"),bt=document.getElementById("guidewireType"),I=document.getElementById("catheterTypeStatus"),b=document.getElementById("guidewireTypeStatus"),X=document.getElementById("currentDose"),et=document.getElementById("currentKV"),J=document.getElementById("currentMA"),tt=document.getElementById("guidewireResistanceStatus"),St=document.getElementById("guidewireResistanceReason"),lt=document.getElementById("guidewireResistanceValue"),gt=document.getElementById("guidewireResistanceFill"),At=document.getElementById("guidewireDiagnostics"),Ht=document.getElementById("guidewireDiameter"),it=document.getElementById("sheathDiameter"),ce=document.getElementById("catheterDiameter"),Jt=document.getElementById("perfStats"),Gt=document.getElementById("runBrowserBenchmarkSmoke"),Lt=document.getElementById("runBrowserBenchmarkFull"),vt=document.getElementById("stopBrowserBenchmark"),Ft=document.getElementById("browserBenchmarkStatus"),te=document.getElementById("browserBenchmarkReport");Ht&&(Ht.textContent=`${xp.toFixed(3)}" · ${_p.toFixed(3)} mm`),it&&(it.textContent=`${vp}F · ${Sp.toFixed(3)} mm`),ce&&(ce.textContent=`${Ep}F · ${wp.toFixed(3)} mm`),y&&(i.visible=y.checked);const le={stlModel:w?.checked??!0,lumenCast:A?.checked??!1,sections:E?.checked??!1,centerline:T?.checked??!0,capsules:L?.checked??!1};function Wt(){typeof u=="function"&&u({...le})}w?.addEventListener("change",O=>{le.stlModel=O.target.checked,Wt()}),A?.addEventListener("change",O=>{le.lumenCast=O.target.checked,Wt()}),E?.addEventListener("change",O=>{le.sections=O.target.checked,Wt()}),T?.addEventListener("change",O=>{le.centerline=O.target.checked,Wt()}),L?.addEventListener("change",O=>{le.capsules=O.target.checked,Wt()}),Wt();let ct=0,U=0,dt=-1,ft=-1,zt=-1,Bt=-1,de=-1,ue="",ve="",we=null,se=null,Ce=-1,Ye="",Si=Kt?.value||"pigtail",Ds=bt?.value||"glidewire";const Sn=.05,Rn=new Gd({emptyThresholdCm:Sn}),qe=new Gd({emptyThresholdCm:Sn});function Un(O,mt){O&&(O.disabled=mt.disabled,O.classList.toggle("active",mt.active),O.setAttribute("aria-pressed",String(mt.active)),O.textContent=mt.active?"Zatrzymaj":"Wysuń")}function Vi(){Rn.cancel(),Un(Ct,Rn)}function os(){qe.cancel(),Un(W,qe)}function st(O,mt,Ut,Ue){if(O){O.disabled!==Ut&&(O.disabled=Ut);const zn=Ut?"Withdraw to 0 cm before changing selection":"";O.title!==zn&&(O.title=zn)}if(mt){const zn=Ut?`${Ue.toFixed(1)} cm inserted`:"Ready";mt.textContent!==zn&&(mt.textContent=zn),mt.classList.contains("locked")!==Ut&&mt.classList.toggle("locked",Ut)}}function Xt(){st(bt,b,ct>Sn,ct),st(Kt,I,U>Sn,U),Un(Ct,Rn),Un(W,qe)}Kt?.addEventListener("change",O=>{Si=O.target.value}),bt?.addEventListener("change",O=>{Ds=O.target.value}),Ct?.addEventListener("click",()=>{Rn.toggle(),Un(Ct,Rn)}),W?.addEventListener("click",()=>{qe.toggle(),Un(W,qe)}),Xt(),Gt?.addEventListener("click",()=>{typeof d=="function"&&d(5e3)}),Lt?.addEventListener("click",()=>{typeof d=="function"&&d(6e5)}),vt?.addEventListener("click",()=>{typeof f=="function"&&f()});const P=Array.from(document.querySelectorAll("[data-control-tab]")),k=Array.from(document.querySelectorAll("[data-control-panel]"));if(P.length&&k.length){const O=mt=>{P.forEach(Ut=>{const Ue=Ut.dataset.controlTab===mt;Ut.classList.toggle("active",Ue),Ut.setAttribute("aria-selected",Ue?"true":"false")}),k.forEach(Ut=>{Ut.classList.toggle("active",Ut.dataset.controlPanel===mt)})};P.forEach(mt=>{mt.addEventListener("click",()=>O(mt.dataset.controlTab))})}if([p,_,v,S,z,H,j,$,Q,V,K,nt,rt,ot,_t,B,D,N].filter(Boolean).forEach(O=>O.addEventListener("change",()=>O.blur())),y&&y.addEventListener("change",O=>{i.visible=O.target.checked}),document.querySelectorAll('#controls input[type="range"], #carm-controls input[type="range"]').forEach(O=>{const mt=O.nextElementSibling;if(!mt)return;const Ut=()=>{mt.textContent=O.value};Ut(),O.addEventListener("input",Ut)}),document.querySelectorAll(".section-header").forEach(O=>{O.addEventListener("click",()=>{const mt=O.nextElementSibling;O.classList.toggle("collapsed"),mt&&mt.classList.toggle("hidden")})}),G&&s.uniforms.autoExposureEnabled){let O=!!s.uniforms.autoExposureEnabled.value;const mt=()=>{s.uniforms.autoExposureEnabled.value=O,G.textContent=`Auto exposure: ${O?"On":"Off"}`,G.classList.toggle("active",O)};mt(),G.addEventListener("click",()=>{O=!O,mt(),G.blur()})}if(j&&(s.uniforms.noiseLevel.value=parseFloat(j.value),j.addEventListener("input",O=>{s.uniforms.noiseLevel.value=parseFloat(O.target.value)})),H&&s.uniforms.pulseRate){const O=mt=>{s.uniforms.pulseRate.value=parseFloat(mt.target.value)};s.uniforms.pulseRate.value=parseFloat(H.value),H.addEventListener("input",O),H.addEventListener("change",O)}if($&&s.uniforms.scatterStrength&&(s.uniforms.scatterStrength.value=parseFloat($.value),$.addEventListener("input",O=>{s.uniforms.scatterStrength.value=parseFloat(O.target.value)})),Q&&s.uniforms.collimation&&(s.uniforms.collimation.value=parseFloat(Q.value),Q.addEventListener("input",O=>{s.uniforms.collimation.value=parseFloat(O.target.value)})),V&&s.uniforms.imageBrightness&&(s.uniforms.imageBrightness.value=parseFloat(V.value),V.addEventListener("input",O=>{s.uniforms.imageBrightness.value=parseFloat(O.target.value)})),K&&s.uniforms.imageContrast&&(s.uniforms.imageContrast.value=parseFloat(K.value),K.addEventListener("input",O=>{s.uniforms.imageContrast.value=parseFloat(O.target.value)})),nt&&s.uniforms.edgeStrength&&(s.uniforms.edgeStrength.value=parseFloat(nt.value),nt.addEventListener("input",O=>{s.uniforms.edgeStrength.value=parseFloat(O.target.value)})),z&&(a.uniforms.decay.value=parseFloat(z.value),z.addEventListener("input",O=>{a.uniforms.decay.value=parseFloat(O.target.value)})),rt&&s.uniforms.boneOpacity&&(s.uniforms.boneOpacity.value=parseFloat(rt.value),rt.addEventListener("input",O=>{s.uniforms.boneOpacity.value=parseFloat(O.target.value)})),ot&&s.uniforms.contrastOpacity&&(s.uniforms.contrastOpacity.value=parseFloat(ot.value)/100,ot.addEventListener("input",O=>{s.uniforms.contrastOpacity.value=parseFloat(O.target.value)/100})),_t&&s.uniforms.contrastGain&&(s.uniforms.contrastGain.value=parseFloat(_t.value),_t.addEventListener("input",O=>{s.uniforms.contrastGain.value=parseFloat(O.target.value)})),p){let O=parseFloat(p.value);ld(O),p.addEventListener("input",mt=>{O=parseFloat(mt.target.value),ld(O)})}if(_&&v){let O=parseFloat(_.value),mt=parseFloat(v.value);sl(O,mt),_.addEventListener("input",Ut=>{O=parseFloat(Ut.target.value),sl(O,mt)}),v.addEventListener("input",Ut=>{mt=parseFloat(Ut.target.value),sl(O,mt)})}if(S){let O=parseInt(S.value);hd(O),S.addEventListener("input",mt=>{O=parseInt(mt.target.value),hd(O)})}let q=!0;if(M){const O=()=>{M.classList.toggle("fluoro-active",q),M.classList.toggle("debug-active",!q),M.setAttribute("aria-pressed",String(!q)),M.setAttribute("aria-label",`Current view: ${q?"fluoroscopy":"debug"}`)};O(),s.uniforms.fluoroscopy.value=!0,M.addEventListener("click",()=>{q=!q,s.uniforms.fluoroscopy.value=q,O(),o&&o.color.set(16777215),typeof h=="function"&&h(q)}),typeof h=="function"&&h(q)}let Y=0,at=0,Et=0;const Dt=O=>{O!==0&&os(),at=O},xt=()=>{at=0},Vt=O=>{Et=O},kt=()=>{Et=0};function Ot(O,mt,Ut){O&&(O.addEventListener("pointerdown",Ue=>{mt(),O.setPointerCapture?.(Ue.pointerId),Ue.preventDefault()}),O.addEventListener("pointerup",Ut),O.addEventListener("pointercancel",Ut),O.addEventListener("pointerleave",Ue=>{Ue.buttons===0&&Ut()}))}Ot(Nt,()=>Dt(1),xt),Ot(Tt,()=>Dt(-1),xt),Ot(Pt,()=>Vt(-1),kt),Ot(yt,()=>Vt(1),kt),document.addEventListener("keydown",O=>{if((O.code==="KeyW"||O.code==="ArrowUp")&&(Vi(),Y=1,O.preventDefault()),(O.code==="KeyS"||O.code==="ArrowDown")&&(Vi(),Y=-1,O.preventDefault()),O.code==="KeyD"&&(os(),at=1,O.preventDefault()),O.code==="KeyA"&&(os(),at=-1,O.preventDefault()),O.code==="KeyE"&&(Et=1,O.preventDefault()),O.code==="KeyQ"&&(Et=-1,O.preventDefault()),O.code==="KeyC"&&q){if(typeof c=="function"){const mt=parseFloat(D.value),Ut=parseFloat(N.value)/1e3,Ue=parseFloat(B.value);c({rate:mt,duration:Ut,volume:Ue})}O.preventDefault()}},!0),document.addEventListener("keyup",O=>{["KeyW","KeyS","ArrowUp","ArrowDown"].includes(O.code)&&(Y=0,O.preventDefault()),["KeyA","KeyD"].includes(O.code)&&(at=0,O.preventDefault()),["KeyQ","KeyE"].includes(O.code)&&(Et=0,O.preventDefault())},!0),window.addEventListener("blur",()=>{Y=0,at=0,Et=0}),C&&C.addEventListener("click",()=>{if(typeof c=="function"){const O=parseFloat(D.value),mt=parseFloat(N.value)/1e3,Ut=parseFloat(B.value);c({rate:O,duration:mt,volume:Ut})}}),F&&F.addEventListener("click",()=>{typeof l=="function"&&l()});function fe(O){ct=Math.max(0,O),Rn.updateLength(ct);const mt=Math.round(ct*10);if(mt===dt)return;dt=mt;const Ut=(mt/10).toFixed(1);ht&&(ht.textContent=`Wire ${Ut} cm`),Xt()}function Je(O){U=Math.max(0,O),qe.updateLength(U);const mt=Math.round(U*10);if(mt===ft)return;ft=mt;const Ut=(mt/10).toFixed(1);ut&&(ut.textContent=`Catheter ${Ut} cm`),Xt()}function Re(O){const mt=Math.round(O*10);if(mt===zt)return;zt=mt;const Ut=(mt/10).toFixed(1);X&&(X.textContent=`Contrast ${Ut} ml`)}function Zn(O,mt){const Ut=Math.round(O),Ue=Math.round(mt*10);et&&Ut!==Bt&&(ue=`${Ut} kV`,et.textContent=ue),J&&Ue!==de&&(ve=`${(Ue/10).toFixed(1)} mA`,J.textContent=ve),Bt=Ut,de=Ue}function Se(O,mt=""){if(!tt)return;if(O<.35){we!==!1&&(tt.classList.add("hidden"),tt.classList.remove("strong"),St&&(St.textContent="Opór na prowadniku"),lt&&(lt.textContent="0%"),gt&&(gt.style.width="0%"),we=!1,se=!1,Ce=0,Ye="");return}const Ut=Math.round(Math.max(0,Math.min(1,O))*100),Ue=O>.72,zn=mt||"Opór na prowadniku - cofnij lekko lub zmień kierunek.";we!==!0&&(tt.classList.remove("hidden"),we=!0),se!==Ue&&(tt.classList.toggle("strong",Ue),se=Ue),St&&Ye!==zn&&(St.textContent=zn,Ye=zn),Ce!==Ut&&(lt&&(lt.textContent=`${Ut}%`),gt&&(gt.style.width=`${Ut}%`),Ce=Ut)}function $t(O){return Number.isFinite(O)?Math.abs(O)<10?O.toFixed(2):O.toFixed(1):"--"}function ki(O){return Number.isFinite(O)?O<10?O.toFixed(2):O.toFixed(1):"--"}function Te(O){if(!O)return"";const mt=Number.isFinite(O.settledPenetration)?` | pen ${$t(O.settledPenetration)}/${$t(O.maximumPenetration)} mm`:"";return`
XPBD: adv ${ki(O.advanceMs)} / solve ${ki(O.solveMs)} / narrow ${ki(O.projectMs)} / dbg ${ki(O.diagnosticMs)} ms | q ${O.pointContactCount}+${O.diagnosticPointContactCount} | segS ${O.segmentSampleCount}${Number.isFinite(O.activeBranchCount)?` | br ${O.activeBranchCount}`:""}`+mt+`${O.foldGuarded?" | fold":""}${O.stabilityRepaired?" | repair":""}${O.withdrawalRelaxed?" | withdraw":""}`}function Mi(O=null){if(!At)return;if(At.classList.remove("warn","breach"),!O){At.textContent="GW STL: debug off";return}const mt=Te(O.performance);if(!O.checkedCount||!Number.isFinite(O.minSignedDistance)){At.textContent=`GW STL: no lumen samples${mt}`;return}At.classList.toggle("breach",O.outsideCount>0),At.classList.toggle("warn",O.outsideCount===0&&O.clearanceViolationCount>0),At.textContent=`GW STL: min ${$t(O.minSignedDistance)} mm / clr ${$t(O.clearance)} | out ${O.outsideCount} | near ${O.clearanceViolationCount} | seg ${$t(O.maxSegmentError)} | bend ${$t(O.maxBendAngle)} deg`+mt}function Ba(O){C&&C.disabled!==!!O&&(C.disabled=!!O)}function cs(O){F&&F.disabled!==!!O&&(F.disabled=!!O)}let Hi=0,ze=0;function yi(O){if(!Jt||(Hi+=O,ze++,Hi<.25))return;const mt=(ze/Math.max(1e-6,Hi)).toFixed(1);let Ut="N/A";performance.memory&&(Ut=(performance.memory.usedJSHeapSize/1048576).toFixed(1)+" MB"),Jt.textContent=`FPS: ${mt} | Mem: ${Ut}`,Hi=0,ze=0}function Lr(O,mt=null){const Ut=!!O?.running;if(Gt&&(Gt.disabled=Ut),Lt&&(Lt.disabled=Ut),vt&&(vt.disabled=!Ut),!Ft)return;if(Ft.classList.remove("passed","failed"),Ut){if(te&&(te.value="Running"),O.warmingUp){Ft.textContent="Warming up";return}const lm=Math.floor(O.elapsedMs/1e3),hm=Math.round(O.durationMs/1e3);Ft.textContent=`Running ${lm}/${hm} s · cycle ${O.cycleIndex+1}`;return}if(!mt?.frameCount){Ft.textContent="Idle",te&&(te.value="No report");return}const Ue=mt.browserAcceptance,zn=O.durationMs>=6e5&&O.elapsedMs>=6e5,au=zn&&!!Ue?.passed;Ft.classList.add(au?"passed":"failed"),Ft.textContent=`${zn?au?"PASS":"FAIL":"Smoke"} · ${mt.averageFps.toFixed(1)} FPS · 1% ${mt.onePercentLowFps.toFixed(1)} · pen ${mt.physicsEnvelope.maxPostStepPenetrationMm.toFixed(3)} mm`,te&&(te.value=JSON.stringify(mt))}function Mn(O){const mt=O===!0;mt&&m?.reset?.(),m?.setLocked?.(mt),document.body.classList.toggle("automated-benchmark-running",mt)}return{monitor:g,getAdvance:()=>Rn.active?Rn.command:Y,getCatheterAdvance:()=>qe.active?qe.command:at,getCatheterRotation:()=>Et,getSelectedCatheterType:()=>Si,getSelectedGuidewireType:()=>Ds,getFluoroscopy:()=>q,getDebugLayerState:()=>({...le}),updateInsertedLength:fe,updateCatheterLength:Je,updateDose:Re,updateXrayTechnique:Zn,updateGuidewireResistance:Se,updateGuidewireDiagnostics:Mi,setInjectButtonDisabled:Ba,setStopInjectionDisabled:cs,updatePerfStats:yi,updateBrowserBenchmarkStatus:Lr,setAutomatedBenchmarkMode:Mn,getCArmRevision:()=>m?.getRevision?.()??0}}const UM=/^[og]\s*(.+)?/,zM=/^mtllib /,OM=/^usemtl /,GM=/^usemap /,Vd=/\s+/,kd=new R,vl=new R,Hd=new R,Wd=new R,Vn=new R,vo=new Qt;function VM(){const r={objects:[],object:{},vertices:[],normals:[],colors:[],uvs:[],materials:{},materialLibraries:[],startObject:function(t,e){if(this.object&&this.object.fromDeclaration===!1){this.object.name=t,this.object.fromDeclaration=e!==!1;return}const n=this.object&&typeof this.object.currentMaterial=="function"?this.object.currentMaterial():void 0;if(this.object&&typeof this.object._finalize=="function"&&this.object._finalize(!0),this.object={name:t||"",fromDeclaration:e!==!1,geometry:{vertices:[],normals:[],colors:[],uvs:[],hasUVIndices:!1},materials:[],smooth:!0,startMaterial:function(i,s){const a=this._finalize(!1);a&&(a.inherited||a.groupCount<=0)&&this.materials.splice(a.index,1);const o={index:this.materials.length,name:i||"",mtllib:Array.isArray(s)&&s.length>0?s[s.length-1]:"",smooth:a!==void 0?a.smooth:this.smooth,groupStart:a!==void 0?a.groupEnd:0,groupEnd:-1,groupCount:-1,inherited:!1,clone:function(c){const l={index:typeof c=="number"?c:this.index,name:this.name,mtllib:this.mtllib,smooth:this.smooth,groupStart:0,groupEnd:-1,groupCount:-1,inherited:!1};return l.clone=this.clone.bind(l),l}};return this.materials.push(o),o},currentMaterial:function(){if(this.materials.length>0)return this.materials[this.materials.length-1]},_finalize:function(i){const s=this.currentMaterial();if(s&&s.groupEnd===-1&&(s.groupEnd=this.geometry.vertices.length/3,s.groupCount=s.groupEnd-s.groupStart,s.inherited=!1),i&&this.materials.length>1)for(let a=this.materials.length-1;a>=0;a--)this.materials[a].groupCount<=0&&this.materials.splice(a,1);return i&&this.materials.length===0&&this.materials.push({name:"",smooth:this.smooth}),s}},n&&n.name&&typeof n.clone=="function"){const i=n.clone(0);i.inherited=!0,this.object.materials.push(i)}this.objects.push(this.object)},finalize:function(){this.object&&typeof this.object._finalize=="function"&&this.object._finalize(!0)},parseVertexIndex:function(t,e){const n=parseInt(t,10);return(n>=0?n-1:n+e/3)*3},parseNormalIndex:function(t,e){const n=parseInt(t,10);return(n>=0?n-1:n+e/3)*3},parseUVIndex:function(t,e){const n=parseInt(t,10);return(n>=0?n-1:n+e/2)*2},addVertex:function(t,e,n){const i=this.vertices,s=this.object.geometry.vertices;s.push(i[t+0],i[t+1],i[t+2]),s.push(i[e+0],i[e+1],i[e+2]),s.push(i[n+0],i[n+1],i[n+2])},addVertexPoint:function(t){const e=this.vertices;this.object.geometry.vertices.push(e[t+0],e[t+1],e[t+2])},addVertexLine:function(t){const e=this.vertices;this.object.geometry.vertices.push(e[t+0],e[t+1],e[t+2])},addNormal:function(t,e,n){const i=this.normals,s=this.object.geometry.normals;s.push(i[t+0],i[t+1],i[t+2]),s.push(i[e+0],i[e+1],i[e+2]),s.push(i[n+0],i[n+1],i[n+2])},addFaceNormal:function(t,e,n){const i=this.vertices,s=this.object.geometry.normals;kd.fromArray(i,t),vl.fromArray(i,e),Hd.fromArray(i,n),Vn.subVectors(Hd,vl),Wd.subVectors(kd,vl),Vn.cross(Wd),Vn.normalize(),s.push(Vn.x,Vn.y,Vn.z),s.push(Vn.x,Vn.y,Vn.z),s.push(Vn.x,Vn.y,Vn.z)},addColor:function(t,e,n){const i=this.colors,s=this.object.geometry.colors;i[t]!==void 0&&s.push(i[t+0],i[t+1],i[t+2]),i[e]!==void 0&&s.push(i[e+0],i[e+1],i[e+2]),i[n]!==void 0&&s.push(i[n+0],i[n+1],i[n+2])},addUV:function(t,e,n){const i=this.uvs,s=this.object.geometry.uvs;s.push(i[t+0],i[t+1]),s.push(i[e+0],i[e+1]),s.push(i[n+0],i[n+1])},addDefaultUV:function(){const t=this.object.geometry.uvs;t.push(0,0),t.push(0,0),t.push(0,0)},addUVLine:function(t){const e=this.uvs;this.object.geometry.uvs.push(e[t+0],e[t+1])},addFace:function(t,e,n,i,s,a,o,c,l){const h=this.vertices.length;let u=this.parseVertexIndex(t,h),d=this.parseVertexIndex(e,h),f=this.parseVertexIndex(n,h);if(this.addVertex(u,d,f),this.addColor(u,d,f),o!==void 0&&o!==""){const g=this.normals.length;u=this.parseNormalIndex(o,g),d=this.parseNormalIndex(c,g),f=this.parseNormalIndex(l,g),this.addNormal(u,d,f)}else this.addFaceNormal(u,d,f);if(i!==void 0&&i!==""){const g=this.uvs.length;u=this.parseUVIndex(i,g),d=this.parseUVIndex(s,g),f=this.parseUVIndex(a,g),this.addUV(u,d,f),this.object.geometry.hasUVIndices=!0}else this.addDefaultUV()},addPointGeometry:function(t){this.object.geometry.type="Points";const e=this.vertices.length;for(let n=0,i=t.length;n<i;n++){const s=this.parseVertexIndex(t[n],e);this.addVertexPoint(s),this.addColor(s)}},addLineGeometry:function(t,e){this.object.geometry.type="Line";const n=this.vertices.length,i=this.uvs.length;for(let s=0,a=t.length;s<a;s++)this.addVertexLine(this.parseVertexIndex(t[s],n));for(let s=0,a=e.length;s<a;s++)this.addUVLine(this.parseUVIndex(e[s],i))}};return r.startObject("",!1),r}class kM extends _c{constructor(t){super(t),this.materials=null}load(t,e,n,i){const s=this,a=new ap(this.manager);a.setPath(this.path),a.setRequestHeader(this.requestHeader),a.setWithCredentials(this.withCredentials),a.load(t,function(o){try{e(s.parse(o))}catch(c){i?i(c):console.error(c),s.manager.itemError(t)}},n,i)}setMaterials(t){return this.materials=t,this}parse(t){const e=new VM;t.indexOf(`\r
`)!==-1&&(t=t.replace(/\r\n/g,`
`)),t.indexOf(`\\
`)!==-1&&(t=t.replace(/\\\n/g,""));const n=t.split(`
`);let i=[];for(let o=0,c=n.length;o<c;o++){const l=n[o].trimStart();if(l.length===0)continue;const h=l.charAt(0);if(h!=="#")if(h==="v"){const u=l.split(Vd);switch(u[0]){case"v":e.vertices.push(parseFloat(u[1]),parseFloat(u[2]),parseFloat(u[3])),u.length>=7?(vo.setRGB(parseFloat(u[4]),parseFloat(u[5]),parseFloat(u[6])).convertSRGBToLinear(),e.colors.push(vo.r,vo.g,vo.b)):e.colors.push(void 0,void 0,void 0);break;case"vn":e.normals.push(parseFloat(u[1]),parseFloat(u[2]),parseFloat(u[3]));break;case"vt":e.uvs.push(parseFloat(u[1]),parseFloat(u[2]));break}}else if(h==="f"){const d=l.slice(1).trim().split(Vd),f=[];for(let x=0,m=d.length;x<m;x++){const p=d[x];if(p.length>0){const _=p.split("/");f.push(_)}}const g=f[0];for(let x=1,m=f.length-1;x<m;x++){const p=f[x],_=f[x+1];e.addFace(g[0],p[0],_[0],g[1],p[1],_[1],g[2],p[2],_[2])}}else if(h==="l"){const u=l.substring(1).trim().split(" ");let d=[];const f=[];if(l.indexOf("/")===-1)d=u;else for(let g=0,x=u.length;g<x;g++){const m=u[g].split("/");m[0]!==""&&d.push(m[0]),m[1]!==""&&f.push(m[1])}e.addLineGeometry(d,f)}else if(h==="p"){const d=l.slice(1).trim().split(" ");e.addPointGeometry(d)}else if((i=UM.exec(l))!==null){const u=(" "+i[0].slice(1).trim()).slice(1);e.startObject(u)}else if(OM.test(l))e.object.startMaterial(l.substring(7).trim(),e.materialLibraries);else if(zM.test(l))e.materialLibraries.push(l.substring(7).trim());else if(GM.test(l))console.warn('THREE.OBJLoader: Rendering identifier "usemap" not supported. Textures must be defined in MTL files.');else if(h==="s"){if(i=l.split(" "),i.length>1){const d=i[1].trim().toLowerCase();e.object.smooth=d!=="0"&&d!=="off"}else e.object.smooth=!0;const u=e.object.currentMaterial();u&&(u.smooth=e.object.smooth)}else{if(l==="\0")continue;console.warn('THREE.OBJLoader: Unexpected line: "'+l+'"')}}e.finalize();const s=new Le;if(s.materialLibraries=[].concat(e.materialLibraries),!(e.objects.length===1&&e.objects[0].geometry.vertices.length===0)===!0)for(let o=0,c=e.objects.length;o<c;o++){const l=e.objects[o],h=l.geometry,u=l.materials,d=h.type==="Line",f=h.type==="Points";let g=!1;if(h.vertices.length===0)continue;const x=new me;x.setAttribute("position",new ie(h.vertices,3)),h.normals.length>0&&x.setAttribute("normal",new ie(h.normals,3)),h.colors.length>0&&(g=!0,x.setAttribute("color",new ie(h.colors,3))),h.hasUVIndices===!0&&x.setAttribute("uv",new ie(h.uvs,2));const m=[];for(let _=0,v=u.length;_<v;_++){const S=u[_],M=S.name+"_"+S.smooth+"_"+g;let y=e.materials[M];if(this.materials!==null){if(y=this.materials.create(S.name),d&&y&&!(y instanceof gi)){const w=new gi;_i.prototype.copy.call(w,y),w.color.copy(y.color),y=w}else if(f&&y&&!(y instanceof ua)){const w=new ua({size:10,sizeAttenuation:!1});_i.prototype.copy.call(w,y),w.color.copy(y.color),w.map=y.map,y=w}}y===void 0&&(d?y=new gi:f?y=new ua({size:1,sizeAttenuation:!1}):y=new Wv,y.name=S.name,y.flatShading=!S.smooth,y.vertexColors=g,e.materials[M]=y),m.push(y)}let p;if(m.length>1){for(let _=0,v=u.length;_<v;_++){const S=u[_];x.addGroup(S.groupStart,S.groupCount,_)}d?p=new ss(x,m):f?p=new Qc(x,m):p=new jt(x,m)}else d?p=new ss(x,m[0]):f?p=new Qc(x,m[0]):p=new jt(x,m[0]);p.name=l.name,s.add(p)}else if(e.vertices.length>0){const o=new ua({size:1,sizeAttenuation:!1}),c=new me;c.setAttribute("position",new ie(e.vertices,3)),e.colors.length>0&&e.colors[0]!==void 0&&(c.setAttribute("color",new ie(e.colors,3)),o.vertexColors=!0);const l=new Qc(c,o);s.add(l)}return s}}function HM({onLoaded:r,onError:t}={}){const e=new Fe({color:16777215,transparent:!0,opacity:.42,depthWrite:!1,depthTest:!1,blending:2,side:2}),n=new Le;return new kM().load("res/skeleton.obj",s=>{s.traverse(c=>{c.isMesh&&(c.material=e)});const o=new rn().setFromObject(s).getCenter(new R);s.position.sub(o),s.rotation.z=-Math.PI/3,s.scale.multiplyScalar(9),s.position.x-=1760,s.position.y-=300,s.position.z-=70,n.add(s),typeof r=="function"&&r({group:n,object:s,material:e})},void 0,s=>{console.warn("Failed to load skeleton OBJ model",s),typeof t=="function"&&t(s)}),{group:n,material:e}}const WM=new R(0,1,0);function ah(r){return new R(r.x,r.y,r.z)}function Xd(r,t){return he.clamp(Math.floor(r),0,Math.max(0,t-1))}function oh(r,t,e){const n=he.clamp((e-r)/Math.max(1e-6,t-r),0,1);return n*n*(3-2*n)}function Yd(r,t){return t<0||t>=r.cells?0:r.core[t]*.58+r.wall[t]*.72}function qd(r,t){return t<0||t>=r.cells?0:r.wall[t]}function XM(r,t){const e=Math.floor(t),n=t-e;return he.lerp(Yd(r,e),Yd(r,e+1),n)}function YM(r,t){const e=Math.floor(t),n=t-e;return he.lerp(qd(r,e),qd(r,e+1),n)}function qM(r,t){return r.segments.map((e,n)=>{const i=ah(e.start),s=ah(e.end),a=new R().subVectors(s,i),o=Math.max(1,a.length()),c=a.clone().normalize(),l=Math.max(2,Math.ceil(o/t)),h=o/l;return{sourceSegment:e,segmentIndex:n,start:i,end:s,dir:c,length:o,cells:l,cellLength:h,radius:e.radius,area:Math.PI*e.radius*e.radius,flowSpeed:e.flowSpeed||0,isSheath:!!e.isSheath,core:new Float32Array(l),wall:new Float32Array(l),nextCore:new Float32Array(l),nextWall:new Float32Array(l),orientation:new ni().setFromUnitVectors(WM,c)}})}class ZM{constructor(t,e=3.5){this.vessel=t,this.segments=qM(t,e),this.segmentGraph=t.segmentGraph||t.segments.map(()=>[]),this.outgoing=Array.from({length:this.segments.length*2},()=>({segmentIndex:-1,amount:0,wallShare:0,sourceArea:0})),this.outgoingCount=0,this.sheathSegmentIndex=t.segments.findIndex(n=>n.isSheath),this.time=0,this.totalSignal=0,this.lastInjectionTime=-1/0,this.coreSpeedScale=1.82,this.wallSpeedScale=1.24,this.wallExchange=4.6,this.axialDispersion=.46,this.clearance=.95,this.tailClearance=3.1}injectThroughSheath(t,e=0){if(t<=0)return;if(this.lastInjectionTime=this.time,this.sheathSegmentIndex>=0){const s=this.segments[this.sheathSegmentIndex];this.#e(s,s.cells-1,t*.06,.48)}const n=this.vessel.sheath?.end,i=n?this.#c(n,{excludeSheath:!0}):null;if(!i){const s=this.segments.find(a=>!a.isSheath);s&&this.#e(s,0,t,.85);return}this.#t(i,t*.92,e)}update(t){if(this.time+=t,this.totalSignal<=0)return;this.totalSignal=0;const e=1+.18*Math.sin(this.time*Math.PI*2.15);this.outgoingCount=0;for(let i=0;i<this.segments.length;i++){const s=this.segments[i];s.nextCore.set(s.core),s.nextWall.set(s.wall);const a=Math.min(.96,Math.max(0,s.flowSpeed*this.coreSpeedScale*e*t/s.cellLength)),o=Math.min(.78,Math.max(0,s.flowSpeed*this.wallSpeedScale*t/s.cellLength));this.#r(s,s.core,s.nextCore,a,1),this.#r(s,s.wall,s.nextWall,o,.35),s.core.set(s.nextCore),s.wall.set(s.nextWall),this.#f(s,t)}for(let i=0;i<this.outgoingCount;i++)this.#o(this.outgoing[i]);const n=this.time-this.lastInjectionTime>.38;for(let i=0;i<this.segments.length;i++){const s=this.segments[i];for(let a=0;a<s.cells;a++){const o=s.core[a]+s.wall[a]*.8,c=n?1-oh(.012,.13,o):0,l=Math.exp(-(this.clearance+c*this.tailClearance)*t),h=Math.exp(-(this.clearance*1.2+c*this.tailClearance*1.35)*t);s.core[a]*=l,s.wall[a]*=h,s.core[a]<4e-4&&(s.core[a]=0),s.wall[a]<4e-4&&(s.wall[a]=0),this.totalSignal+=s.core[a]+s.wall[a]*.8}}}hasVisibleContrast(t=.02){return this.totalSignal>t}#t(t,e,n){const i=t.segment,s=t.cellIndex,a=he.clamp(n/45,.35,1.35);this.#e(i,s,e*.25,.72);const o=s+1,c=Math.max(5,Math.min(o,Math.round(18*a)));let l=0;const h=[];for(let f=0;f<c;f++){const g=s-f;if(g<0)break;const x=Math.exp(-f/(5.5+a*4));h.push([g,x]),l+=x}for(const[f,g]of h)this.#e(i,f,e*.4*g/l,.64);const u=i.sourceSegment?.parent,d=Number.isInteger(u)?this.segments[u]:null;if(d){const f=Math.min(d.cells,Math.round(24*a));let g=0;const x=[];for(let m=0;m<f;m++){const p=d.cells-1-m,_=Math.exp(-m/8);x.push([p,_]),g+=_}for(const[m,p]of x)this.#e(d,m,e*.35*p/g,.58)}}#e(t,e,n,i){if(!t||n<=0)return;const s=Xd(e,t.cells),a=Math.max(1,t.area*t.cellLength),o=n*1e3/a;t.core[s]+=o*i,t.wall[s]+=o*(1-i),this.totalSignal+=o}#r(t,e,n,i,s){if(!(i<=0))for(let a=t.cells-1;a>=0;a--){const o=e[a]*i;if(n[a]-=o,a+1<t.cells)n[a+1]+=o;else if(o>0){const c=this.outgoing[this.outgoingCount++];c.segmentIndex=t.segmentIndex,c.amount=o,c.wallShare=s,c.sourceArea=t.area}}}#f(t,e){const n=he.clamp(this.wallExchange*e,0,.22),i=he.clamp(this.axialDispersion*e,0,.08);for(let s=0;s<t.cells;s++){const a=(t.core[s]-t.wall[s])*n;t.core[s]-=a,t.wall[s]+=a}if(!(i<=0||t.cells<3)){t.nextCore.set(t.core),t.nextWall.set(t.wall);for(let s=1;s<t.cells-1;s++)t.nextCore[s]+=(t.core[s-1]+t.core[s+1]-t.core[s]*2)*i,t.nextWall[s]+=(t.wall[s-1]+t.wall[s+1]-t.wall[s]*2)*i*1.35;t.core.set(t.nextCore),t.wall.set(t.nextWall)}}#o(t){const e=this.segmentGraph[t.segmentIndex]||[];if(!e.length)return;let n=0;for(let i=0;i<e.length;i++)n+=this.segments[e[i]]?.area||0;n<=0&&(n=e.length);for(let i=0;i<e.length;i++){const s=e[i],a=this.segments[s];if(!a)continue;const o=(a.area||1)/n,c=t.amount*o*t.sourceArea*a.cellLength/1e3;this.#e(a,0,c,.58+(1-t.wallShare)*.18)}}#c(t,{excludeSheath:e=!1}={}){const n=ah(t);let i=null;for(const s of this.segments){if(e&&s.isSheath)continue;const a=new R().subVectors(n,s.start),o=he.clamp(a.dot(s.dir),0,s.length),l=s.start.clone().addScaledVector(s.dir,o).distanceTo(n),h=Math.max(0,l-s.radius),u=Xd(o/s.cellLength,s.cells);(!i||h<i.score)&&(i={segment:s,segmentIndex:s.segmentIndex,cellIndex:u,score:h})}return i}}function jM(r,t=.015,e=!1,n=null){if(!r?.segments)return{mesh:n,count:0};const i=r.vessel?.geometry;if(!i?.attributes?.position)return{mesh:n,count:0};if(!n||!n.isMesh||n.userData.sourceGeometry!==i){KM(n);const c=i.clone(),l=c.attributes.position.count;c.setAttribute("color",new Be(new Float32Array(l*3),3));const h=new Fe({vertexColors:!0,transparent:!0,opacity:e?.78:.96,blending:2,depthTest:!1,depthWrite:!1,side:2,wireframe:e});n=new jt(c,h),n.frustumCulled=!1,n.userData.sourceGeometry=i,n.userData.influences=$M(r,c)}const s=n.geometry.attributes.color,a=n.userData.influences||[];let o=0;for(let c=0;c<s.count;c++){const l=a[c];let h=0,u=0;if(l?.length)for(const m of l){const p=r.segments[m.segmentIndex];h+=XM(p,m.cellFloat)*m.weight,u+=YM(p,m.cellFloat)*m.weight}const d=oh(t*.18,t*4.4,h),f=oh(t*.28,t*4,u),g=Math.max(d,f*.82),x=g>.018?Math.min(1,Math.pow(g,.78)*1.18):0;x>.02&&o++,s.setXYZ(c,x,x,x)}return s.needsUpdate=!0,n.visible=o>0,n.material.wireframe=e,n.material.opacity=e?.78:.96,{mesh:n,count:o}}function $M(r,t){const e=t.attributes.position,n=new R,i=new Array(e.count);for(let s=0;s<e.count;s++){n.fromBufferAttribute(e,s);const a=[];for(const l of r.segments){if(l.isSheath)continue;const u=new R().subVectors(n,l.start).dot(l.dir),d=he.clamp(u,0,l.length),f=l.start.clone().addScaledVector(l.dir,d),g=n.distanceTo(f),x=Math.max(0,-u,u-l.length),m=Math.abs(g-l.radius)+x*.45,p=Math.max(2,l.radius*.48),_=Math.exp(-(m*m)/(2*p*p));if(_<.02)continue;const v=he.clamp(d/l.cellLength-.5,0,l.cells-1);a.push({segmentIndex:l.segmentIndex,cellFloat:v,weight:_})}a.sort((l,h)=>h.weight-l.weight);const o=a.slice(0,3),c=o.reduce((l,h)=>l+h.weight,0);i[s]=c>0?o.map(l=>({...l,weight:l.weight/c})):[]}return i}function KM(r){if(r)if(r.isGroup)for(const t of r.children)t.geometry?.dispose?.(),t.material?.dispose?.();else r.geometry?.dispose?.(),r.material?.dispose?.()}const JM=8;function bp(r,{radius:t,samplesPerSegment:e=3,radialSegments:n=12,maxTubularSegments:i=900}={}){if(!Array.isArray(r)||r.length<2||!(t>0))return new me;const s=r.length===2?new np(r[0],r[1]):new kh(r,!1,"centripetal",.5),a=Math.min(i,Math.max(JM,Math.ceil((r.length-1)*e))),o=new xc(s,a,t,n,!1);return o.userData.smoothTube={sourcePointCount:r.length,tubularSegments:a,radialSegments:n},o}const ch=Qo,lh=7.2,hh=1.05,ws=lh*hh*Math.PI*2,So="pigtail",je="berenstein",Zd=Math.PI/4,pa=8,Ms=10,Mo=pa+Ms,Pp=16,yo=Pp+ws,ui=18,ps=4,ms=3.2,QM=150,ty=.24,Sl=.96,jd=.88,ey=18,ny=4,Ml=.04,yl=.08,iy=.22,sy=.075,El=72*Math.PI/180,ry=.36,$d=.55,Kd=1.2,ay=[.25,.5,.75],Jd=[2,4,7],oy=3,Qd=[2,4,8],cy=.24,ly=.085,wl=68*Math.PI/180,hy=.42,tf=1.15,Tl=1.7,uy=1.45,Al=7,dy=.22,fy=.42,Cl=.42,ef=2.4,py=2.6,my=3.2,gy=6e-7,xy=10,_y=5e-6,vy=20,Sy=1,nf=10,Rl=.025,My=ws,yy=8,Ey=24,wy=12,Ty=.35,Ay=.6,Cy=5e-6,bl=4e-6,Ry=1e-4,by=75e-7,Pl=.24,Ll=.3,Dl=.14,Py=20,Ly=8,Dy=4,Iy=.78,sf=1.2,Ny=90,Fy=52,By=32,Uy=Math.PI*.9,rf=ch*.72,zy=2.4,af=Cp*1.35;class Rt extends R{constructor(t=0,e=0,n=0){super(t,e,n),this._values=new Float64Array([this._initialX??t,this._initialY??e,this._initialZ??n])}get x(){return this._values?this._values[0]:this._initialX}set x(t){this._values?this._values[0]=t:this._initialX=t}get y(){return this._values?this._values[1]:this._initialY}set y(t){this._values?this._values[1]=t:this._initialY=t}get z(){return this._values?this._values[2]:this._initialZ}set z(t){this._values?this._values[2]=t:this._initialZ=t}}function De(r,t,e){return Math.sqrt(r*r+t*t+e*e)}function ea(r){return new Rt(r.x,r.y,r.z)}class Oy{constructor({wire:t,segmentLength:e,guidewireLength:n,tailProgressRef:i,vessel:s=null,maxLength:a=1e3}){this.wire=t,this.segmentLength=e,this.guidewireLength=n,this.tailProgressRef=i,this.vessel=s,this.vesselColliders=this.#rt(s),this.collisionMesh=null,this.sheathPath=this.#at(s?.sheath),this.maxLength=a,this.progress=0,this.guidewireInserted=0,this.previousGuidewireInserted=0,this.guidewireDelta=0,this.motionCommand=0,this.rotationCommand=0,this.rotation=0,this._pendingXpbdRotation=0,this._xpbdBerensteinTwisted=!1,this.type=So,this.pathSpacing=4,this.pathSamples=[],this._pathSamplePool=Array.from({length:Math.ceil(a/this.pathSpacing)+4},()=>({distance:0,point:new Rt})),this.freeNodes=[],this._nextFreeNodes=[],this._freeNodePool=[],this._freeNodeEpoch=0,this.freeRestDistances=new Float64Array(Math.ceil(a/ms)+2),this.freeRestDistanceCount=0,this.freeLength=0,this._physicsStepIndex=0,this.material=new Fe({color:16777215,depthTest:!1,transparent:!0,opacity:1}),this.tipMarkerMaterial=new Fe({color:16777215,depthTest:!1,transparent:!0,opacity:1}),this.maxRenderSegments=320,this.shaftMesh=new jt(new me,this.material),this.tipMarker=new jt(new Ps(af,af,zy,16,1,!1),this.tipMarkerMaterial),this.tipMarker.frustumCulled=!1,this.tipMarker.renderOrder=8,this.tipMarker.visible=!1,this.mesh=new Le,this.mesh.add(this.shaftMesh,this.tipMarker),this.mesh.frustumCulled=!1,this.mesh.renderOrder=7,this.mesh.visible=!1,this.physicsBody=null,this.physicsActiveCount=0,this.physicsLumenStartNode=0,this._xpbdLayoutX=null,this._xpbdLayoutY=null,this._xpbdLayoutZ=null,this._xpbdDriveX=null,this._xpbdDriveY=null,this._xpbdDriveZ=null,this._xpbdDriveInitialized=null,this._xpbdLayoutCount=0,this._xpbdProgress=0,this._xpbdYieldsToWall=!1,this._guidewireRelease=1,this.externalCollisionSolver=!1,this._renderPoints=[],this._tipMarkerPosition=new Rt,this._tipMarkerTangent=new Rt,this._tipMarkerUp=new Rt(0,1,0),this._shapeNormal=new Rt,this._pathTarget=new Rt,this._newNodeRest=new Rt,this._newNodePath=new Rt,this._newNodeGuide=new Rt,this._newNodePoint=new Rt,this._centerlinePoints=[],this._centerlineDistances=[],this._centerlinePointCount=0,this._deploymentStateScratch={pathEnd:0,supportEnd:0,freeLength:0},this._freeFrameScratch={supportTip:new Rt,beforeTip:new Rt,beforePlane:new Rt,tangent:new Rt,normal:new Rt},this._guideReleaseFrameScratch={supportTip:new Rt,beforeTip:new Rt,tangent:new Rt,normal:new Rt},this._planePreviousTangent=new Rt,this._planeCurvature=new Rt,this._planeHelper=new Rt,this._xpbdDriveDirection=new Rt,this._xpbdSoloTipTarget=new Rt,this._xpbdSoloTipTargetActive=!1,this._xpbdSoloTipControlIndex=-1}setType(t){const e=this.#ft(t);this.type!==e&&(this.#e(),this.type=e,this.#w(),this.freeRestDistanceCount=0,this.freeLength=0,this._physicsStepIndex=0,this.rotationCommand=0,this._pendingXpbdRotation=0,this._xpbdBerensteinTwisted=!1,this.physicsLumenStartNode=0,this._xpbdProgress=this.progress,this._xpbdYieldsToWall=!1,this._xpbdDriveInitialized?.fill(0),this.updateMesh())}dispose(){this.#e(),this.shaftMesh.geometry?.dispose?.(),this.tipMarker.geometry?.dispose?.(),this.material.dispose(),this.tipMarkerMaterial.dispose()}setExternalCollisionSolver(t=!0){return t||this.#e(),this.externalCollisionSolver=!!t,this}reset(){return this.#e(),this.progress=0,this.guidewireInserted=0,this.previousGuidewireInserted=0,this.guidewireDelta=0,this.motionCommand=0,this.rotationCommand=0,this.rotation=0,this._pendingXpbdRotation=0,this._xpbdBerensteinTwisted=!1,this.pathSamples.length=0,this.#w(),this.freeRestDistanceCount=0,this.freeLength=0,this._physicsStepIndex=0,this.physicsActiveCount=0,this._xpbdLayoutCount=0,this._xpbdProgress=0,this._xpbdYieldsToWall=!1,this._xpbdDriveInitialized?.fill(0),this._guidewireRelease=1,this.updateMesh(),this}syncXpbdBody(t,{shapeCompliance:e=t.shapeCompliance,targetSlewLimit:n=Sy,restLengthSlewLimit:i=.5,bendChordSlewLimit:s=1}={}){const a=this.#E(),o=Math.min(this._centerlinePointCount,t.count);let c=this.progress-this._xpbdProgress;if((this.physicsBody!==t||!this._xpbdLayoutX||this._xpbdLayoutX.length!==t.count)&&(this.physicsBody&&this.physicsBody!==t&&this.#e(this.physicsBody),this._xpbdLayoutX=new Float64Array(t.count),this._xpbdLayoutY=new Float64Array(t.count),this._xpbdLayoutZ=new Float64Array(t.count),this._xpbdDriveX=new Float32Array(t.count),this._xpbdDriveY=new Float32Array(t.count),this._xpbdDriveZ=new Float32Array(t.count),this._xpbdDriveInitialized=new Uint8Array(t.count),this._xpbdLayoutCount=0,this.physicsActiveCount=0,this._xpbdProgress=this.progress,c=0,this._xpbdYieldsToWall=!1),this.physicsBody=t,t.postStabilizationPasses=this.type===So?Ly:Dy,o<2){for(let p=0;p<this.physicsActiveCount;p++)t.clearRestShapeTarget(p);return t.setActiveRange(0,1),t.setCollisionRange(0,-1),this.physicsActiveCount=0,this._xpbdLayoutCount=0,this._xpbdProgress=this.progress,this._xpbdYieldsToWall=!1,this._xpbdDriveInitialized?.fill(0),this._pendingXpbdRotation=0,this.#e(t),0}const l=this.physicsActiveCount,h=this.externalCollisionSolver&&this.guidewireInserted<=ui;let u=-1,d=!1;if(l>0&&this._xpbdLayoutCount===l&&o===l+1){u=h&&this.freeNodes.length>=2?this.#o(o):this.freeNodes.length>=2?o-1:this.#a(a,o,l);for(let _=o-1;_>u;_--)this.#p(t,_,_-1);this.#A(t,a,u,o,e);const p=this.type===je||this.progress>=yo;h&&p&&u+1<o&&this.#c(t,a,u,e),d=!0}else if(o>1&&this._xpbdLayoutCount===l&&o===l-1){const p=h&&this.freeNodes.length>=2?this.#o(o):this.freeNodes.length>=2?l-1:this.#i(a,o,l);for(let _=p;_<o;_++)this.#p(t,_,_+1);d=!0}if(t.setActiveRange(0,o-1),(d||Math.abs(this.motionCommand)>0||Math.abs(this.guidewireDelta)>1e-5)&&t.wake(),d){h&&this.#f(t);for(let p=0;p<o-1;p++)t.restLength[p]=Math.max(.5,De(t.x[p+1]-t.x[p],t.y[p+1]-t.y[p],t.z[p+1]-t.z[p]));for(let p=1;p<o-1;p++)t.restBendChord[p]=De(t.x[p+1]-t.x[p-1],t.y[p+1]-t.y[p-1],t.z[p+1]-t.z[p-1])}let f=o-1;const g=this.vessel?.sheath;if(g){const p=g.end.x-g.start.x,_=g.end.y-g.start.y,v=g.end.z-g.start.z,S=De(p,_,v)||1,M=p/S,y=_/S,w=v/S;for(let A=0;A<o;A++){const E=a[A];if((E.x-g.start.x)*M+(E.y-g.start.y)*y+(E.z-g.start.z)*w>S+.25){f=Math.max(0,A-1);break}}}this.#t(t,o),h&&l>0&&c>0&&this.#T(t,f,o,Math.min(1,c));let x=!1;for(let p=Math.max(0,f);p<Math.min(t.segmentCount,o-1);p++)if(t.wallActive[p]){x=!0;break}const m=x&&!this._xpbdYieldsToWall;x&&(this._xpbdYieldsToWall=!0);for(let p=0;p<o;p++){const _=a[p],v=this._centerlineDistances[p]??1/0,S=this.#Q(v),M=this.#tt(v),y=this.guidewireInserted>ui?1-Ge(this.guidewireInserted+ps,this.guidewireInserted+Py,v):0,w=this._xpbdYieldsToWall,A=this.externalCollisionSolver&&p<=f,E=this.externalCollisionSolver&&p>f,T=p===u||u<0&&p>=l;if(T&&p!==u)if(l>0&&p>0){const B=a[p-1];let G=_.x-B.x,z=_.y-B.y,H=_.z-B.z,j=De(G,z,H);j<1e-6&&p>1&&(G=t.x[p-1]-t.x[p-2],z=t.y[p-1]-t.y[p-2],H=t.z[p-1]-t.z[p-2],j=De(G,z,H));const $=Math.max(.5,_.distanceTo(B)),Q=1/Math.max(1e-6,j);t.setNodePosition(p,t.x[p-1]+G*Q*$,t.y[p-1]+z*Q*$,t.z[p-1]+H*Q*$)}else t.setNodePosition(p,_.x,_.y,_.z);const L=S>Rl;if(A||L||E&&(!w||h)){const B=t.restShapeEnabled[p]===1,G=this.type===je&&this._xpbdBerensteinTwisted&&L&&!A&&B;let z=G||w&&!A&&B?t.restShapeX[p]:_.x,H=G||w&&!A&&B?t.restShapeY[p]:_.y,j=G||w&&!A&&B?t.restShapeZ[p]:_.z;if(l>0&&!A&&(T||!B||m))z=t.x[p],H=t.y[p],j=t.z[p];else if(!G&&(!w||A||L&&(h||this.type===je&&Math.abs(this.rotationCommand)>0)&&M>Rl)&&l>0&&B&&Number.isFinite(n)&&n>0){const Q=_.x-t.restShapeX[p],V=_.y-t.restShapeY[p],K=_.z-t.restShapeZ[p],nt=De(Q,V,K),rt=L?n*(.35+S*.65)*(w?.3:1):Math.max(1,n);if(nt>rt){const ot=rt/nt;z=t.restShapeX[p]+Q*ot,H=t.restShapeY[p]+V*ot,j=t.restShapeZ[p]+K*ot}}if(h&&w&&!A){const Q=z-t.x[p],V=H-t.y[p],K=j-t.z[p],nt=De(Q,V,K),rt=this.type===je?Ay:Ty;if(nt>rt){const ot=rt/nt;z=t.x[p]+Q*ot,H=t.y[p]+V*ot,j=t.z[p]+K*ot}}let $;!w||A?$=bl:L?$=e/Math.max(.25,S):$=Ry,G&&($=Math.min($,Cy)),$=bl+($-bl)*(1-y),t.setRestShapeTarget(p,z,H,j,$)}else t.clearRestShapeTarget(p);t.nodeRadius[p]=ch;const C=1-Ge(0,ui,this.guidewireInserted),F=t.bendCompliance+(Math.min(t.bendCompliance,gy)-t.bendCompliance)*C;t.bendComplianceByNode[p]=F+(_y-F)*M;const D=t.maxBendAngle+(Math.min(t.maxBendAngle,xy)-t.maxBendAngle)*C,N=this.type===je?vy:t.maxBendAngle;if(t.maxBendAngleByNode[p]=D+(N-D)*M,p>0){const B=a[p-1],G=Math.max(.5,_.distanceTo(B));l>0&&i>0?t.restLength[p-1]+=It(G-t.restLength[p-1],-i,i):t.restLength[p-1]=G}if(p>0&&p<o-1){const B=a[p-1].distanceTo(a[p+1]),G=a[p-1].distanceTo(_)+_.distanceTo(a[p+1]),H=w&&l>0&&M>Rl?De(t.x[p+1]-t.x[p-1],t.y[p+1]-t.y[p-1],t.z[p+1]-t.z[p-1]):h?G+(B-G)*S:B;l>0&&s>0?t.restBendChord[p]+=It(H-t.restBendChord[p],-s,s):t.restBendChord[p]=H}}for(let p=o;p<l;p++)t.clearRestShapeTarget(p);this.#r(t,o,h,c),t.setCollisionRange(f,o-2),this.physicsActiveCount=o;for(let p=0;p<o;p++)this._xpbdLayoutX[p]=a[p].x,this._xpbdLayoutY[p]=a[p].y,this._xpbdLayoutZ[p]=a[p].z;return this._xpbdLayoutCount=o,this._xpbdProgress=this.progress,o}#t(t,e){const n=this._pendingXpbdRotation;if(this._pendingXpbdRotation=0,this.type!==je||Math.abs(n)<1e-6||e<4)return;const i=Math.max(this.#b(),this.guidewireInserted,this.progress-Mo);let s=Math.max(1,e-1);for(let A=1;A<e;A++)if(!((this._centerlineDistances[A]??-1/0)<i)){s=A;break}if(s>=e-1)return;const a=Math.max(0,s-2),o=Math.min(e-1,s+1);let c=t.x[o]-t.x[a],l=t.y[o]-t.y[a],h=t.z[o]-t.z[a];const u=De(c,l,h);if(u<1e-6)return;c/=u,l/=u,h/=u,this._xpbdBerensteinTwisted=!0;const d=Math.cos(n),f=Math.sin(n),g=1-d,x=t.x[s],m=t.y[s],p=t.z[s],_=t.previousX[s],v=t.previousY[s],S=t.previousZ[s],M=t.restShapeEnabled[s]?t.restShapeX[s]:x,y=t.restShapeEnabled[s]?t.restShapeY[s]:m,w=t.restShapeEnabled[s]?t.restShapeZ[s]:p;if(this._xpbdSoloTipTargetActive){const A=this._xpbdSoloTipTarget.x-x,E=this._xpbdSoloTipTarget.y-m,T=this._xpbdSoloTipTarget.z-p,L=A*c+E*l+T*h;this._xpbdSoloTipTarget.set(x+A*d+(l*T-h*E)*f+c*L*g,m+E*d+(h*A-c*T)*f+l*L*g,p+T*d+(c*E-l*A)*f+h*L*g)}for(let A=s+1;A<e;A++){const E=t.x[A]-x,T=t.y[A]-m,L=t.z[A]-p,C=E*c+T*l+L*h;t.x[A]=x+E*d+(l*L-h*T)*f+c*C*g,t.y[A]=m+T*d+(h*E-c*L)*f+l*C*g,t.z[A]=p+L*d+(c*T-l*E)*f+h*C*g;const F=t.previousX[A]-_,D=t.previousY[A]-v,N=t.previousZ[A]-S,B=F*c+D*l+N*h;if(t.previousX[A]=_+F*d+(l*N-h*D)*f+c*B*g,t.previousY[A]=v+D*d+(h*F-c*N)*f+l*B*g,t.previousZ[A]=S+N*d+(c*D-l*F)*f+h*B*g,t.restShapeEnabled[A]){const $=t.restShapeX[A]-M,Q=t.restShapeY[A]-y,V=t.restShapeZ[A]-w,K=$*c+Q*l+V*h;t.restShapeX[A]=M+$*d+(l*V-h*Q)*f+c*K*g,t.restShapeY[A]=y+Q*d+(h*$-c*V)*f+l*K*g,t.restShapeZ[A]=w+V*d+(c*Q-l*$)*f+h*K*g}const G=t.velocityX[A],z=t.velocityY[A],H=t.velocityZ[A],j=G*c+z*l+H*h;t.velocityX[A]=G*d+(l*H-h*z)*f+c*j*g,t.velocityY[A]=z*d+(h*G-c*H)*f+l*j*g,t.velocityZ[A]=H*d+(c*z-l*G)*f+h*j*g,t.shapeLambda[A]=0,t.controlLambda[A]=0}t.wake()}#e(t=this.physicsBody){t&&this._xpbdSoloTipControlIndex>=0&&(t.clearControlTarget(this._xpbdSoloTipControlIndex),t.setPinned(this._xpbdSoloTipControlIndex,!1)),this._xpbdSoloTipTargetActive=!1,this._xpbdSoloTipControlIndex=-1}#r(t,e,n,i){const a=e-1;if(this._xpbdSoloTipControlIndex>=0&&this._xpbdSoloTipControlIndex!==a&&(t.clearControlTarget(this._xpbdSoloTipControlIndex),t.setPinned(this._xpbdSoloTipControlIndex,!1)),!n){this._xpbdSoloTipControlIndex>=0&&(t.clearControlTarget(this._xpbdSoloTipControlIndex),t.setPinned(this._xpbdSoloTipControlIndex,!1)),this._xpbdSoloTipTargetActive=!1,this._xpbdSoloTipControlIndex=-1;return}if(this._xpbdSoloTipTargetActive||(this._xpbdSoloTipTarget.set(t.x[a],t.y[a],t.z[a]),this._xpbdSoloTipTargetActive=!0),i>0&&a>0){const u=this.#h(t,a,t.x[a]-t.x[a-1],t.y[a]-t.y[a-1],t.z[a]-t.z[a-1]),d=u.length()||1;this._xpbdSoloTipTarget.addScaledVector(u,Math.min(.75,i)/d)}const o=this._xpbdSoloTipTarget.x-t.x[a],c=this._xpbdSoloTipTarget.y-t.y[a],l=this._xpbdSoloTipTarget.z-t.z[a],h=De(o,c,l);if(h>.6){const u=.6/h;this._xpbdSoloTipTarget.set(t.x[a]+o*u,t.y[a]+c*u,t.z[a]+l*u)}t.clearRestShapeTarget(a),t.setPinned(a,!1),t.setControlTarget(a,this._xpbdSoloTipTarget.x,this._xpbdSoloTipTarget.y,this._xpbdSoloTipTarget.z,by),this._xpbdSoloTipControlIndex=a}#f(t){t.wallLambda.fill(0),t.wallActive.fill(0),t.wallT.fill(0),t.wallX.fill(0),t.wallY.fill(0),t.wallZ.fill(0),t.wallNormalX.fill(0),t.wallNormalY.fill(0),t.wallNormalZ.fill(0),t.wallBranchId.fill(-1),t.wallGap.fill(1/0),t.wallQueryStartX.fill(0),t.wallQueryStartY.fill(0),t.wallQueryStartZ.fill(0),t.wallQueryEndX.fill(0),t.wallQueryEndY.fill(0),t.wallQueryEndZ.fill(0)}#o(t){const e=this.#b();for(let n=1;n<t;n++)if((this._centerlineDistances[n]??-1/0)>=e-.25)return n;return Math.max(1,t-1)}#c(t,e,n,i){const s=e[n],a=e[n+1];t.setNodePosition(n,s.x,s.y,s.z),t.setNodePosition(n+1,a.x,a.y,a.z),t.clearRestShapeTarget(n),t.setRestShapeTarget(n+1,a.x,a.y,a.z,i),this._xpbdDriveInitialized[n]=0,this._xpbdDriveInitialized[n+1]=0}#T(t,e,n,i){const s=Math.max(1,e+1),a=this.type===je?pa+Ms:ws,o=Math.max(this.#b(),this.progress-a);let c=n;for(let m=s;m<n;m++)if(!((this._centerlineDistances[m]??1/0)<o)){c=m;break}const l=Math.max(s,c-2),h=Math.min(n-1,c+1);let u=t.x[h]-t.x[l],d=t.y[h]-t.y[l],f=t.z[h]-t.z[l];const g=this.#h(t,c,u,d,f);u=g.x,d=g.y,f=g.z;const x=De(u,d,f)||1;for(let m=n-1;m>=s;m--){const p=m-1;let _,v,S,M;if(m>=c)_=u,v=d,S=f,M=x;else if(m+1<n){_=t.x[m+1]-t.x[p],v=t.y[m+1]-t.y[p],S=t.z[m+1]-t.z[p];const T=this.#h(t,m,_,v,S);_=T.x,v=T.y,S=T.z,M=De(_,v,S)}else _=t.x[m]-t.x[p],v=t.y[m]-t.y[p],S=t.z[m]-t.z[p],M=De(_,v,S);if(M<1e-6)continue;const y=i/M,w=_*y,A=v*y,E=S*y;t.restShapeEnabled[m]&&(t.restShapeX[m]+=w,t.restShapeY[m]+=A,t.restShapeZ[m]+=E)}}#h(t,e,n,i,s){const a=this._xpbdDriveDirection,o=Math.max(0,Math.min(t.count-1,e));let c=De(n,i,s);c<1e-6&&(this._xpbdDriveInitialized?.[o]?(n=this._xpbdDriveX[o],i=this._xpbdDriveY[o],s=this._xpbdDriveZ[o],c=1):(n=1,i=0,s=0,c=1)),n/=c,i/=c,s/=c,a.set(n,i,s);let l=!1;const h=t.contactField;if(typeof h?.getCenterlineTangent=="function"){const d=h.getCenterlineTangent(t.x[o],t.y[o],t.z[o]);d&&Number.isFinite(d.x)&&Number.isFinite(d.y)&&Number.isFinite(d.z)&&(a.set(d.x,d.y,d.z),l=a.lengthSq()>=1e-8)}else if(h?.centerline&&h.centerlineStride>=6){const d=h.centerline,f=h.centerlineStride,g=Math.floor(d.length/f);let x=-1;for(let m=0;m<=2&&x<0;m++){const p=Math.min(t.segmentCount-1,o+m),_=Math.max(0,o-1-m);t.wallActive[p]&&t.wallBranchId[p]>=0?x=t.wallBranchId[p]:t.wallActive[_]&&t.wallBranchId[_]>=0&&(x=t.wallBranchId[_])}if(x<0){const m=Math.max(0,Math.min(t.segmentCount-1,o-1));x=t.wallBranchId[m]}if(x>=0&&x<g){const m=x*f;a.set(d[m+3]-d[m],d[m+4]-d[m+1],d[m+5]-d[m+2]),l=a.lengthSq()>=1e-8}}l?(a.normalize(),a.x*n+a.y*i+a.z*s<0&&a.multiplyScalar(-1),a.set(n+(a.x-n)*Pl,i+(a.y-i)*Pl,s+(a.z-s)*Pl).normalize()):a.set(n,i,s);const u=o-1;if(u>=0&&this._xpbdDriveInitialized?.[u]){let d=this._xpbdDriveX[u],f=this._xpbdDriveY[u],g=this._xpbdDriveZ[u];a.x*d+a.y*f+a.z*g<0&&(d*=-1,f*=-1,g*=-1),a.set(a.x+(d-a.x)*Ll,a.y+(f-a.y)*Ll,a.z+(g-a.z)*Ll).normalize()}if(this._xpbdDriveInitialized?.[o]){let d=this._xpbdDriveX[o],f=this._xpbdDriveY[o],g=this._xpbdDriveZ[o];a.x*d+a.y*f+a.z*g<0&&(d*=-1,f*=-1,g*=-1),a.set(d+(a.x-d)*Dl,f+(a.y-f)*Dl,g+(a.z-g)*Dl).normalize()}return this._xpbdDriveX[o]=a.x,this._xpbdDriveY[o]=a.y,this._xpbdDriveZ[o]=a.z,this._xpbdDriveInitialized[o]=1,a}#a(t,e,n){let i=e-1,s=1/0;for(let a=0;a<e;a++){let o=0;for(let c=0;c<n;c++){const l=c<a?c:c+1,h=t[l],u=h.x-this._xpbdLayoutX[c],d=h.y-this._xpbdLayoutY[c],f=h.z-this._xpbdLayoutZ[c];o+=u*u+d*d+f*f}o<s&&(s=o,i=a)}return i}#i(t,e,n){let i=n-1,s=1/0;for(let a=0;a<n;a++){let o=0;for(let c=0;c<e;c++){const l=c<a?c:c+1,h=t[c],u=h.x-this._xpbdLayoutX[l],d=h.y-this._xpbdLayoutY[l],f=h.z-this._xpbdLayoutZ[l];o+=u*u+d*d+f*f}o<s&&(s=o,i=a)}return i}#p(t,e,n){t.x[e]=t.x[n],t.y[e]=t.y[n],t.z[e]=t.z[n],t.previousX[e]=t.previousX[n],t.previousY[e]=t.previousY[n],t.previousZ[e]=t.previousZ[n],t.velocityX[e]=t.velocityX[n],t.velocityY[e]=t.velocityY[n],t.velocityZ[e]=t.velocityZ[n],t.inverseMass[e]=t.inverseMass[n],t.nodeRadius[e]=t.nodeRadius[n],t.pinned[e]=t.pinned[n],t.bendComplianceByNode[e]=t.bendComplianceByNode[n],t.restShapeEnabled[e]=t.restShapeEnabled[n],t.restShapeX[e]=t.restShapeX[n],t.restShapeY[e]=t.restShapeY[n],t.restShapeZ[e]=t.restShapeZ[n],t.restShapeCompliance[e]=t.restShapeCompliance[n],this._xpbdDriveX[e]=this._xpbdDriveX[n],this._xpbdDriveY[e]=this._xpbdDriveY[n],this._xpbdDriveZ[e]=this._xpbdDriveZ[n],this._xpbdDriveInitialized[e]=this._xpbdDriveInitialized[n]}#A(t,e,n,i,s){if(n>0&&n+1<i){const a=e[n],o=e[n-1],c=e[n+1],l=a.distanceTo(o),h=a.distanceTo(c),u=l/Math.max(1e-6,l+h);t.x[n]=t.x[n-1]+(t.x[n+1]-t.x[n-1])*u,t.y[n]=t.y[n-1]+(t.y[n+1]-t.y[n-1])*u,t.z[n]=t.z[n-1]+(t.z[n+1]-t.z[n-1])*u,t.previousX[n]=t.previousX[n-1]+(t.previousX[n+1]-t.previousX[n-1])*u,t.previousY[n]=t.previousY[n-1]+(t.previousY[n+1]-t.previousY[n-1])*u,t.previousZ[n]=t.previousZ[n-1]+(t.previousZ[n+1]-t.previousZ[n-1])*u,t.velocityX[n]=t.velocityX[n-1]+(t.velocityX[n+1]-t.velocityX[n-1])*u,t.velocityY[n]=t.velocityY[n-1]+(t.velocityY[n+1]-t.velocityY[n-1])*u,t.velocityZ[n]=t.velocityZ[n-1]+(t.velocityZ[n+1]-t.velocityZ[n-1])*u}else if(n>0){const a=e[n],o=e[n-1],c=this.externalCollisionSolver&&(this.guidewireInserted<=ui||(this._centerlineDistances[n]??1/0)>this.guidewireInserted)&&n>1;let l=c?t.x[n-1]-t.x[n-2]:a.x-o.x,h=c?t.y[n-1]-t.y[n-2]:a.y-o.y,u=c?t.z[n-1]-t.z[n-2]:a.z-o.z,d=De(l,h,u);d<1e-6&&n>1&&(l=t.x[n-1]-t.x[n-2],h=t.y[n-1]-t.y[n-2],u=t.z[n-1]-t.z[n-2],d=De(l,h,u));const g=Math.max(.5,a.distanceTo(o))/Math.max(1e-6,d);t.x[n]=t.x[n-1]+l*g,t.y[n]=t.y[n-1]+h*g,t.z[n]=t.z[n-1]+u*g,t.previousX[n]=t.x[n],t.previousY[n]=t.y[n],t.previousZ[n]=t.z[n],t.velocityX[n]=0,t.velocityY[n]=0,t.velocityZ[n]=0}else t.setNodePosition(n,e[n].x,e[n].y,e[n].z);t.restShapeEnabled[n]=0,t.restShapeX[n]=t.x[n],t.restShapeY[n]=t.y[n],t.restShapeZ[n]=t.z[n],t.restShapeCompliance[n]=s,t.shapeLambda[n]=0,this._xpbdDriveInitialized[n]=0}setCollisionGeometry(t){const e=t?.geometry||t;if(!e?.boundsTree){this.collisionMesh=null;return}this.collisionMesh={geometry:e,meshCollider:t?.meshCollider||null,clearance:Math.max(ch*.7,t?.clearance||0),interiorDirection:t?.interiorDirection||t?.collisionInteriorDirection||null}}advance(t,e,n){this.motionCommand=t,this.previousGuidewireInserted=this.guidewireInserted,this.guidewireInserted=Math.max(0,n),this.guidewireDelta=this.guidewireInserted-this.previousGuidewireInserted;const i=t>0?Fy:By,s=It(this.progress+t*i*e,0,this.maxLength);s>this.progress?this.#W(Math.min(s,this.guidewireInserted)):s<this.progress&&this.#ct(s);const a=Math.min(s,this.guidewireInserted);(t!==0||this.guidewireDelta>0)&&a>ui&&this.#ot(a),this.progress=s}rotate(t,e){if(this.rotationCommand=t,!t)return;const n=t*Uy*e;this.rotation+=n,this.type===je&&this.externalCollisionSolver&&(this._pendingXpbdRotation+=n)}stepPhysics(t=1/60,{collisions:e=!0}={}){const n=this.#u();this.#j(t);const i=this._physicsStepIndex++;if((!this.externalCollisionSolver||(i&3)===0)&&this.#l(n.pathEnd),this.externalCollisionSolver){this.#v(n,t);return}if(n.freeLength<2||n.supportEnd<=0){this.#w(),this.freeRestDistanceCount=0,this.freeLength=0;return}const s=this.#m(n.supportEnd);if(this.#g(n,s),this.freeNodes.length<2)return;this.#s(n);const a=s.supportTip;for(let l=0;l<this.freeNodes.length;l++){const h=this.freeNodes[l];h.previousPos||=new Rt,h.shapeTarget||=new Rt,h.guideTarget||=new Rt,h.previousPos.copy(h.pos)}this.freeNodes[0].pos.copy(a),this.freeNodes[0].vel.set(0,0,0);for(let l=1;l<this.freeNodes.length;l++){const h=this.freeNodes[l];h.curl=Math.min(1,(h.curl??1)+ef*t);const u=Math.max(0,(h.distance??0)-(this.freeNodes[0].distance??0)),d=this.#z(u,s,n.freeLength,h.curl,h.shapeTarget),f=QM*t;h.vel.x+=(d.x-h.pos.x)*f,h.vel.y+=(d.y-h.pos.y)*f,h.vel.z+=(d.z-h.pos.z)*f,h.vel.multiplyScalar(jd),h.pos.addScaledVector(h.vel,t)}const o=e?ey:ny;for(let l=0;l<o;l++)this.freeNodes[0].pos.copy(a),this.#s(n),this.#I(),this.#d(s,n.freeLength),this.#U(s),this.#n(n.freeLength),this.#X(n.freeLength),e&&(this.#Y(),this.#q()),this.#I();const c=1/Math.max(1e-4,t);for(let l=1;l<this.freeNodes.length;l++){const h=this.freeNodes[l];h.vel.subVectors(h.pos,h.previousPos).multiplyScalar(c*jd)}this.freeNodes[0].vel.set(0,0,0)}#v(t,e){if(t.freeLength<2||t.supportEnd<=0){this.#w(),this.freeRestDistanceCount=0,this.freeLength=0;return}const n=this.#m(t.supportEnd);if(this.#g(t,n),this.freeNodes.length<2)return;this.freeNodes[0].pos.copy(n.supportTip),this.freeNodes[0].vel.set(0,0,0);const i=this.freeNodes[0].distance??t.supportEnd;for(let s=1;s<this.freeNodes.length;s++){const a=this.freeNodes[s];a.curl=Math.min(1,(a.curl??1)+ef*e);const o=Math.max(0,(a.distance??i)-i);a.pos.copy(this.#z(o,n,t.freeLength,1,a.shapeTarget)),a.vel.set(0,0,0)}}constrainGuidewire(t=1/60,{reactionScale:e=1}={}){if(this.progress<4)return;const n=this.#u();n.freeLength>=2&&this.freeNodes.length<2&&n.supportEnd>0&&this.#g(n,this.#m(n.supportEnd));const i=this.tailProgressRef(),s=this.freeNodes.length>=2?Math.max(n.pathEnd,this.progress):n.pathEnd,a=Math.min(this.progress,this.guidewireInserted,s);if(!(a<=0))for(let o=0;o<this.wire.nodes.length;o++){const c=this.#dt(o,i);if(c<=0||c>a)continue;const l=this.wire.nodes[o];if(l.pinned)continue;const h=this.#lt(c,n),u=l.x,d=l.y,f=l.z,x=(.6+Ge(0,this.segmentLength*1.5,c)*.4)*Iy,m=h.clone().sub(new Rt(u,d,f)),p=m.length();p>sf&&m.multiplyScalar(sf/p);const _=m.multiplyScalar(x);l.x=u+_.x,l.y=d+_.y,l.z=f+_.z;const v=1/Math.max(1e-4,t);l.vx=(l.x-u)*v*.25,l.vy=(l.y-d)*v*.25,l.vz=(l.z-f)*v*.25,this.#ht(c,_,e)}}updateMesh(){const t=this.physicsBody,e=t?null:this.#E(),n=t?this.physicsActiveCount:this._centerlinePointCount;if(n<2){this.mesh.visible=!1,this.tipMarker.visible=!1;return}const i=Math.min(n,this.maxRenderSegments+1);this._renderPoints.length=i;for(let a=0;a<i;a++){let o=this._renderPoints[a];o||(o=new R,this._renderPoints[a]=o),o.set(t?t.x[a]:e[a].x,t?t.y[a]:e[a].y,t?t.z[a]:e[a].z)}const s=this.shaftMesh.geometry;this.shaftMesh.geometry=bp(this._renderPoints,{radius:Cp,samplesPerSegment:3,radialSegments:14}),s.dispose(),this.#y(i),this.mesh.visible=!0}#y(t){const e=this.type===je?Mo:yo;let n=0;for(let i=t-1;i>0;i--){const s=this._renderPoints[i],a=this._renderPoints[i-1],o=s.distanceTo(a);if(o<1e-6)continue;if(n+o<e){n+=o;continue}const c=It((e-n)/o,0,1);this._tipMarkerPosition.copy(s).lerp(a,c),this._tipMarkerTangent.subVectors(s,a).normalize(),this.tipMarker.position.copy(this._tipMarkerPosition),this.tipMarker.quaternion.setFromUnitVectors(this._tipMarkerUp,this._tipMarkerTangent),this.tipMarker.userData.tipLengthMm=e,this.tipMarker.userData.catheterType=this.type,this.tipMarker.visible=!0;return}this.tipMarker.visible=!1}#E(){const t=this.#u(),e=this.sheathPath?Ny:0;if(this.physicsLumenStartNode=0,this._centerlinePointCount=0,t.pathEnd<=0&&e<=0)return this._centerlinePoints;const n=Math.max(0,t.supportEnd),i=n>0?It(Math.ceil(n/5),1,90):0,s=this._centerlinePoints;if(e>0){const c=It(Math.ceil(e/6),2,24);for(let l=0;l<=c;l++){const h=-e+e*l/c,u=this._centerlinePointCount++;this.#P(h,this.#S(u)),this._centerlineDistances[u]=h}this.physicsLumenStartNode=c}if(t.pathEnd<=0)return s;const a=this._centerlinePointCount?1:0;for(let c=a;c<=i;c++){const l=i>0?n*c/i:0,h=this._centerlinePointCount++;this.#P(l,this.#S(h)),this._centerlineDistances[h]=l}if(t.freeLength<2){if(t.pathEnd>n+.5){const c=this._centerlinePointCount++;this.#P(t.pathEnd,this.#S(c)),this._centerlineDistances[c]=t.pathEnd}return s}const o=this.#m(t.supportEnd);this.#g(t,o);for(let c=1;c<this.freeNodes.length;c++){const l=this._centerlinePointCount++;this.#S(l).copy(this.freeNodes[c].pos),this._centerlineDistances[l]=this.freeNodes[c].distance??t.supportEnd}return s}#S(t){let e=this._centerlinePoints[t];return e||(e=new Rt,this._centerlinePoints[t]=e),e}#u(){const t=this._deploymentStateScratch;if(this.progress<4)return t.pathEnd=0,t.supportEnd=0,t.freeLength=0,t;const e=this.#b(),n=Math.max(e,Math.min(this.progress,this.#F()));return t.pathEnd=n,t.supportEnd=n>0?e:0,t.freeLength=n>0?Math.max(0,this.progress-e):0,t}#m(t){const e=this._freeFrameScratch,n=this.#P(t,e.supportTip),i=this.#P(Math.max(0,t-10),e.beforeTip),s=this.#P(Math.max(0,t-28),e.beforePlane),a=e.tangent.subVectors(n,i);a.lengthSq()<1e-5&&a.set(0,1,0),a.normalize();const o=this.#ut(a,i,s,e.normal),c=this.type===je&&this.externalCollisionSolver&&this._xpbdBerensteinTwisted?0:this.rotation;return o.applyAxisAngle(a,c).normalize(),e}#g(t,e){const n=this.freeRestDistances;n[0]=t.supportEnd;let i=1,s=t.supportEnd;for(;s+ms<this.progress-.5;)s+=ms,n[i++]=s;this.progress>n[i-1]+.5&&(n[i++]=this.progress),this.freeRestDistanceCount=i;const a=this.freeNodes,o=this._nextFreeNodes;o.length=0;const c=++this._freeNodeEpoch;let l=0;for(let h=0;h<i;h++){const u=n[h],d=u-t.supportEnd;let f=-1,g=1/0;for(;l<a.length;){const m=Math.abs((a[l].distance??0)-u);if((l+1<a.length?Math.abs((a[l+1].distance??0)-u):1/0)>=m){f=l,g=m;break}l++}let x;if(f>=0&&g<=ms*.7)x=a[f],l=f+1;else{const m=this.guidewireDelta<-1e-4&&u>=this.guidewireInserted-ps&&u<=this.previousGuidewireInserted+ps,p=this.#z(d,e,t.freeLength,m?Cl:1,this._newNodeRest),_=this.#P(Math.min(u,this.#F()),this._newNodePath),v=this.guidewireInserted>ui&&u<=this.guidewireInserted+ps,S=this._newNodePoint;m?S.copy(_).lerp(p,Cl):v?S.copy(this.#L(u,this._newNodeGuide)).lerp(p,.28):S.copy(p);const M=this.externalCollisionSolver?S:this.#N(S).point;x=this.#C(M,u,m?Cl:1)}x._activeEpoch=c,x.distance=u,x.curl=x.curl??1,x.previousPos||=new Rt,x.shapeTarget||=new Rt,x.guideTarget||=new Rt,o.push(x)}for(let h=0;h<a.length;h++){const u=a[h];u._activeEpoch===c||u._pooled||(u._pooled=!0,this._freeNodePool.push(u))}this._nextFreeNodes=a,this.freeNodes=o,this.freeLength=t.freeLength,this.freeNodes[0]&&(this.freeNodes[0].pos.copy(e.supportTip),this.freeNodes[0].vel.set(0,0,0))}#C(t,e,n){const i=this._freeNodePool.pop()||{pos:new Rt,vel:new Rt,previousPos:new Rt,shapeTarget:new Rt,guideTarget:new Rt,distance:0,curl:1,_activeEpoch:0,_pooled:!1};return i._pooled=!1,i.pos.copy(t),i.vel.set(0,0,0),i.previousPos.copy(t),i.shapeTarget.copy(t),i.guideTarget.copy(t),i.distance=e,i.curl=n,i}#w(){for(let t=0;t<2;t++){const e=t===0?this.freeNodes:this._nextFreeNodes;for(let n=0;n<e.length;n++){const i=e[n];i._pooled||(i._pooled=!0,this._freeNodePool.push(i))}e.length=0}}#s(t){const e=Math.min(this.progress,this.guidewireInserted);if(e<=t.supportEnd+.5||this.freeNodes.length<2)return;const i=Math.abs(this.motionCommand)>0?fy:dy;for(let s=1;s<this.freeNodes.length;s++){const a=this.freeNodes[s].distance??t.supportEnd;if(a>e+ps)continue;const o=Ge(t.supportEnd,t.supportEnd+Al,a),c=1-Ge(e-Al,e+ps,a),l=this.#L(a,this.freeNodes[s].guideTarget),h=i*o*(.35+c*.65);this.freeNodes[s].pos.lerp(l,h),this.freeNodes[s].vel.multiplyScalar(1-h)}}#l(t){const e=this.sheathPath?.length||0;if(!(this.pathSamples.length<3||t<=e+this.pathSpacing*2))for(let n=0;n<oy;n++)this.#D(t,e),this.#O(t,e)}#D(t,e){const n=e+this.pathSpacing*1.5,i=1/Math.max(1e-8,this.pathSpacing*6.5),s=t-this.pathSpacing*4,a=1/Math.max(1e-8,this.pathSpacing*4);for(let o=1;o<this.pathSamples.length-1;o++){const c=this.pathSamples[o],l=Math.max(0,Math.min(1,(c.distance-n)*i)),h=Math.max(0,Math.min(1,(c.distance-s)*a)),u=l*l*(3-2*l),d=1-h*h*(3-2*h),f=u*(.35+d*.65);if(f<=.001)continue;const g=this.pathSamples[o-1].point._values,x=this.pathSamples[o+1].point._values;this.#M(c,(g[0]+x[0])*.5,(g[1]+x[1])*.5,(g[2]+x[2])*.5,cy*f)}for(let o=0;o<Qd.length;o++){const c=Qd[o];if(!(this.pathSamples.length<=c*2))for(let l=c;l<this.pathSamples.length-c;l++){const h=this.pathSamples[l],u=Math.max(0,Math.min(1,(h.distance-n)*i)),d=Math.max(0,Math.min(1,(h.distance-s)*a)),f=u*u*(3-2*u),g=1-d*d*(3-2*d),x=f*(.35+g*.65);if(x<=.001)continue;const m=this.pathSamples[l-c].point._values,p=this.pathSamples[l+c].point._values;this.#M(h,(m[0]+p[0])*.5,(m[1]+p[1])*.5,(m[2]+p[2])*.5,ly*x/Math.sqrt(c))}}}#O(t,e){const n=Math.cos(wl);for(let i=1;i<this.pathSamples.length-1;i++){const s=this.pathSamples[i],a=this.#x(s.distance,t,e);if(a<=.001)continue;const o=this.pathSamples[i-1].point,c=s.point,l=this.pathSamples[i+1].point,h=c.x-o.x,u=c.y-o.y,d=c.z-o.z,f=l.x-c.x,g=l.y-c.y,x=l.z-c.z,m=De(h,u,d),p=De(f,g,x);if(m<1e-5||p<1e-5)continue;const _=It((h*f+u*g+d*x)/(m*p),-1,1);if(_>=n)continue;const v=It((Math.acos(_)-wl)/(Math.PI-wl),0,1);this.#M(s,(o.x+l.x)*.5,(o.y+l.y)*.5,(o.z+l.z)*.5,hy*v*a)}}#x(t,e,n){if(t<=n+this.pathSpacing)return 0;const i=Ge(n+this.pathSpacing*1.5,n+this.pathSpacing*8,t),s=1-Ge(e-this.pathSpacing*4,e,t);return i*(.35+s*.65)}#M(t,e,n,i,s){const a=It(s,0,1),o=t.point._values;let c=(e-o[0])*a,l=(n-o[1])*a,h=(i-o[2])*a;const u=De(c,l,h);if(!(u<=1e-6)){if(u>tf){const d=tf/u;c*=d,l*=d,h*=d}if(this.externalCollisionSolver){o[0]+=c,o[1]+=l,o[2]+=h;return}this._pathTarget.set(o[0]+c,o[1]+l,o[2]+h),t.point.copy(this.#N(this._pathTarget).point)}}#R(t,e,n,i=1,s=new Rt){if(this.type===je)return this.#_(t,e,n,i,s);const a=Math.min(n,yo),o=Math.max(0,n-a);if(t<=o)return s.copy(e.supportTip).addScaledVector(e.tangent,t);const c=t-o,l=Math.min(a,Pp),h=It(i,0,1);if(c<=l||h<=.001)return s.copy(e.supportTip).addScaledVector(e.tangent,t);const u=Math.min(c-l,ws),d=lh/h,f=Math.min(hh*Math.PI*2,u/d);return s.copy(e.supportTip).addScaledVector(e.tangent,o+l+Math.sin(f)*d).addScaledVector(e.normal,(Math.cos(f)-1)*d)}#_(t,e,n,i=1,s=new Rt){const a=Math.min(n,Mo),o=Math.max(0,n-a);if(t<=o)return s.copy(e.supportTip).addScaledVector(e.tangent,t);const c=t-o,l=Math.min(a,pa);if(c<=l)return s.copy(e.supportTip).addScaledVector(e.tangent,t);const h=Zd*It(i,0,1);if(h<=.001)return s.copy(e.supportTip).addScaledVector(e.tangent,t);const u=this.#B(e,this._shapeNormal),d=Math.max(0,a-l),f=Math.min(Ms,Math.max(1e-4,d)),g=Math.min(c-l,f),x=h*It(g/f,0,1),m=f/h;s.copy(e.supportTip).addScaledVector(e.tangent,o+l).addScaledVector(e.tangent,Math.sin(x)*m).addScaledVector(u,(1-Math.cos(x))*m);const p=c-l-f;return p>0&&s.addScaledVector(e.tangent,Math.cos(h)*p).addScaledVector(u,Math.sin(h)*p),s}#B(t,e=new Rt){return e.copy(t.normal),e.z*=.18,e.addScaledVector(t.tangent,-e.dot(t.tangent)),e.lengthSq()<1e-6?e.copy(t.normal):e.normalize()}#I(){for(let t=1;t<this.freeNodes.length;t++){const e=this.freeNodes[t-1],n=this.freeNodes[t],i=Math.max(.5,(n.distance??0)-(e.distance??0)),s=n.pos.x-e.pos.x,a=n.pos.y-e.pos.y,o=n.pos.z-e.pos.z,c=De(s,a,o);if(c<1e-5)continue;const l=(c-i)/c;if(t===1)n.pos.x-=s*l,n.pos.y-=a*l,n.pos.z-=o*l;else{const h=l*.5;e.pos.x+=s*h,e.pos.y+=a*h,e.pos.z+=o*h,n.pos.x-=s*h,n.pos.y-=a*h,n.pos.z-=o*h}}}#U(t){if(this.freeNodes.length>1){const e=Math.max(.5,(this.freeNodes[1].distance??0)-(this.freeNodes[0].distance??0))||ms,n=this.freeNodes[1].pos;n.x+=(t.supportTip.x+t.tangent.x*e-n.x)*Sl,n.y+=(t.supportTip.y+t.tangent.y*e-n.y)*Sl,n.z+=(t.supportTip.z+t.tangent.z*e-n.z)*Sl}for(let e=2;e<this.freeNodes.length-1;e++){const n=this.freeNodes[e-1].pos,i=this.freeNodes[e+1].pos,s=this.freeNodes[e].pos;s.x+=((n.x+i.x)*.5-s.x)*Ml,s.y+=((n.y+i.y)*.5-s.y)*Ml,s.z+=((n.z+i.z)*.5-s.z)*Ml}}#d(t,e){for(let n=1;n<this.freeNodes.length;n++){const i=Math.max(0,(this.freeNodes[n].distance??0)-(this.freeNodes[0].distance??0)),s=this.#z(i,t,e,this.freeNodes[n].curl??1,this.freeNodes[n].shapeTarget),a=Ge(0,Math.max(ms,e),i),o=this.#J(i,e),c=this.type===So?2.4:1.2,l=ty*(.45+a*.55)*(1+o*c),h=It(l,0,.68),u=this.freeNodes[n].pos._values,d=s._values;u[0]+=(d[0]-u[0])*h,u[1]+=(d[1]-u[1])*h,u[2]+=(d[2]-u[2])*h}}#n(t){if(this.freeNodes.length<4)return;const e=this.freeNodes[0]?.distance??0,n=Math.max(0,t-this.#G(t)),i=Math.max(0,n-10),s=Math.max(1e-8,n+8-i);for(let a=1;a<this.freeNodes.length-1;a++){const o=Math.max(0,(this.freeNodes[a].distance??e)-e),c=Math.max(0,Math.min(1,(o-i)/s)),l=1-c*c*(3-2*c);if(l<=.001)continue;const h=this.freeNodes[a-1].pos._values,u=this.freeNodes[a+1].pos._values,d=this.freeNodes[a].pos._values,f=this.#V(this.freeNodes[a].distance),g=It(iy*l*f,0,1);d[0]+=((h[0]+u[0])*.5-d[0])*g,d[1]+=((h[1]+u[1])*.5-d[1])*g,d[2]+=((h[2]+u[2])*.5-d[2])*g}for(let a=0;a<Jd.length;a++){const o=Jd[a];if(!(this.freeNodes.length<=o*2))for(let c=o;c<this.freeNodes.length-o;c++){const l=Math.max(0,(this.freeNodes[c].distance??e)-e),h=Math.max(0,Math.min(1,(l-i)/s)),u=1-h*h*(3-2*h);if(u<=.001)continue;const d=this.freeNodes[c-o].pos._values,f=this.freeNodes[c+o].pos._values,g=this.#V(this.freeNodes[c].distance),x=sy*u*g/Math.sqrt(o),m=It(x,0,1),p=this.freeNodes[c].pos._values;p[0]+=((d[0]+f[0])*.5-p[0])*m,p[1]+=((d[1]+f[1])*.5-p[1])*m,p[2]+=((d[2]+f[2])*.5-p[2])*m}}}#X(t){if(this.freeNodes.length<3)return;const e=Math.cos(El);for(let n=1;n<this.freeNodes.length-1;n++){const i=this.freeNodes[n-1].pos,s=this.freeNodes[n].pos,a=this.freeNodes[n+1].pos,o=s.x-i.x,c=s.y-i.y,l=s.z-i.z,h=a.x-s.x,u=a.y-s.y,d=a.z-s.z,f=De(o,c,l),g=De(h,u,d);if(f<1e-5||g<1e-5)continue;const x=It((o*h+c*u+l*d)/(f*g),-1,1);if(x>=e)continue;const m=It((Math.acos(x)-El)/(Math.PI-El),0,1),p=this.#K(this.freeNodes[n],t),v=(this.#V(this.freeNodes[n].distance)-1)/Math.max(1e-6,Tl-1),S=1+(uy-1)*v,M=ry*S*m*(.28+p*.72),y=It(M,0,1);s.x+=((i.x+a.x)*.5-s.x)*y,s.y+=((i.y+a.y)*.5-s.y)*y,s.z+=((i.z+a.z)*.5-s.z)*y}}#Y(){for(let t=1;t<this.freeNodes.length;t++){const e=this.freeNodes[t],n=this.#N(e.pos);if(!n.collided)continue;e.pos.copy(n.point);const i=n.normal,s=e.vel.dot(i);s>0&&e.vel.addScaledVector(i,-s),e.vel.multiplyScalar(1-yl)}}#q(){if(!(this.freeNodes.length<2))for(let t=1;t<this.freeNodes.length;t++){const e=this.freeNodes[t-1],n=this.freeNodes[t];for(const i of ay){const s=e.pos.clone().lerp(n.pos,i),a=this.#N(s);if(!a.collided)continue;const o=a.point.sub(s),c=o.length();if(c<=1e-6)continue;c>Kd&&o.multiplyScalar(Kd/c);const l=t===1?0:1-i,h=t===1?1:i;e.pos.addScaledVector(o,$d*l),n.pos.addScaledVector(o,$d*h),e.vel.multiplyScalar(1-yl*l),n.vel.multiplyScalar(1-yl*h)}}}#z(t,e,n,i=1,s=new Rt){const a=this.#b()+t;if(this.guidewireInserted>ui){if(a<=this.guidewireInserted)return s.copy(this.#L(a,this._shapeNormal));const c=Math.max(0,this.progress-this.guidewireInserted),l=Math.min(c,a-this.guidewireInserted),h=this.#Z(e);return this.#$(l,h,c,i,s)}const o=this.#R(t,e,n,i,s);return this.externalCollisionSolver?o:this.#N(o).point}#Z(t){const e=this._guideReleaseFrameScratch;return this.#L(this.guidewireInserted,e.supportTip),this.#L(Math.max(this.#b(),this.guidewireInserted-10),e.beforeTip),e.tangent.subVectors(e.supportTip,e.beforeTip),e.tangent.lengthSq()<1e-6&&e.tangent.copy(t.tangent),e.tangent.normalize(),e.normal.copy(t.normal).addScaledVector(e.tangent,-t.normal.dot(e.tangent)),e.normal.lengthSq()<1e-6&&e.normal.copy(t.normal),e.normal.normalize(),e}#j(t){if(this.guidewireInserted<=ui){this._guidewireRelease=1;return}const e=Math.max(0,this.progress-this.guidewireInserted),n=this.type===je?pa+Ms:ws,i=Ge(0,n,e),s=i>=this._guidewireRelease?py:my;this._guidewireRelease+=It(i-this._guidewireRelease,-s*t,s*t)}#$(t,e,n,i,s){if(this.type===je){const d=Math.min(n,Ms),f=Math.max(0,n-d);if(t<=f||d<=1e-4)return s.copy(e.supportTip).addScaledVector(e.tangent,t);const g=this._guidewireRelease*It(i,0,1),x=Zd*g;if(x<=1e-4)return s.copy(e.supportTip).addScaledVector(e.tangent,t);const m=Ms/x,_=Math.min(t-f,d)/m,v=this.#B(e,this._shapeNormal);return s.copy(e.supportTip).addScaledVector(e.tangent,f+Math.sin(_)*m).addScaledVector(v,(1-Math.cos(_))*m)}const a=Math.min(n,ws),o=Math.max(0,n-a);if(t<=o||a<=1e-4)return s.copy(e.supportTip).addScaledVector(e.tangent,t);const c=this._guidewireRelease*It(i,0,1);if(c<=1e-4)return s.copy(e.supportTip).addScaledVector(e.tangent,t);const l=lh/c,h=Math.min(t-o,a),u=Math.min(hh*Math.PI*2,h/l);return s.copy(e.supportTip).addScaledVector(e.tangent,o+Math.sin(u)*l).addScaledVector(e.normal,(Math.cos(u)-1)*l)}#K(t,e){const n=this.freeNodes[0]?.distance??0,i=Math.max(0,(t.distance??n)-n),s=Math.max(0,e-this.#G(e));return 1-Ge(Math.max(0,s-10),s+8,i)}#J(t,e){const n=Math.max(0,e-this.#G(e));return Ge(n-2,n+10,t)}#Q(t){if(!Number.isFinite(t)||t<=0)return 0;const e=this.type===je?pa+Ms:ws,n=Math.max(this.#b(),this.progress-e),i=Ge(n-2,n+nf,t);if(this.guidewireInserted<=ui)return i;const s=Ge(this.guidewireInserted+.5,this.guidewireInserted+nf,t);return i*s}#tt(t){if(!Number.isFinite(t)||t<=0)return 0;const e=this.type===je?Ey:My,n=this.type===je?wy:yy,i=Math.max(this.#b(),this.progress-e);return Ge(i,i+n,t)}#G(t){const e=this.type===je?Mo:yo;return Math.min(t,e)}#V(t){if(this.guidewireInserted<=ui)return Tl;const e=Ge(this.guidewireInserted-Al,this.guidewireInserted+ps,t??this.progress);return 1+(Tl-1)*e}#N(t){if(this.collisionMesh)return this.#et(t,this.collisionMesh);let e=null;for(const n of this.vesselColliders){const i=n.type==="sphere"?this.#it(t,n):this.#nt(t,n);if(i.inside)return{point:t.clone(),normal:i.normal,collided:!1};(!e||i.distance<e.distance)&&(e=i)}return{point:e?.point||t.clone(),normal:e?.normal||new Rt(1,0,0),collided:!!e}}#et(t,e){if(e.meshCollider?.pointContact){const l=e.meshCollider.pointContact(t,e.clearance);return{point:l.violation?l.target.clone():t.clone(),normal:l.normal?.clone?.()||new Rt(1,0,0),collided:!!l.violation}}const n=new Rt,s=e.geometry.boundsTree.closestPointToPoint(t,{point:n})?.distance??t.distanceTo(n),a=Math.max(e.clearance+ms*1.5,e.clearance*2);if(s>a)return{point:t.clone(),normal:new Rt(1,0,0),collided:!1};const o=typeof e.interiorDirection=="function"?e.interiorDirection(t,n).clone():t.clone().sub(n);return o.lengthSq()<1e-8&&o.set(1,0,0),o.normalize(),t.clone().sub(n).dot(o)>=e.clearance?{point:t.clone(),normal:o.clone().multiplyScalar(-1),collided:!1}:{point:n.clone().addScaledVector(o,e.clearance),normal:o.clone().multiplyScalar(-1),collided:!0}}#nt(t,e){const n=new Rt().subVectors(t,e.start),i=It(n.dot(e.dir),0,e.length),s=e.start.clone().addScaledVector(e.dir,i),a=new Rt().subVectors(t,s),o=a.length(),c=Math.max(.6,e.radius-rf),l=o<=c,h=o>1e-6?a.multiplyScalar(1/o):this.#st(e.dir);if(l)return{inside:l,point:t.clone(),distance:0,normal:h};const u=s.addScaledVector(h,c);return{inside:!1,point:u,distance:t.distanceTo(u),normal:h}}#it(t,e){const n=new Rt().subVectors(t,e.center),i=n.length(),s=Math.max(.6,e.radius-rf),a=i<=s,o=i>1e-6?n.multiplyScalar(1/i):new Rt(1,0,0);if(a)return{inside:a,point:t.clone(),distance:0,normal:o};const c=e.center.clone().addScaledVector(o,s);return{inside:!1,point:c,distance:t.distanceTo(c),normal:o}}#st(t){const e=Math.abs(t.y)<.85?new Rt(0,1,0):new Rt(1,0,0);return new Rt().crossVectors(t,e).normalize()}#rt(t){if(!t?.segments)return[];const e=[],n=new Map,i=a=>`${a.x.toFixed(5)},${a.y.toFixed(5)},${a.z.toFixed(5)}`,s=(a,o)=>{const c=i(a),l=n.get(c);n.set(c,{point:a,radius:l?Math.max(l.radius,o):o})};for(const a of t.segments){const o=ea(a.start),c=ea(a.end),l=new Rt().subVectors(c,o),h=l.length();if(h<1e-6)continue;const u=l.multiplyScalar(1/h);e.push({type:"segment",start:o,end:c,dir:u,length:h,radius:a.radius||t.radius||10}),s(a.end,a.radius||t.radius||10),a.isSheath||s(a.start,a.radius||t.radius||10)}for(const{point:a,radius:o}of n.values())e.push({type:"sphere",center:ea(a),radius:o});return e}#at(t){if(!t?.start||!t?.end)return null;const e=ea(t.start),n=ea(t.end),i=new Rt().subVectors(n,e),s=i.length();return s<1e-6?null:(i.multiplyScalar(1/s),{start:e,end:n,dir:i,length:s})}#b(){return this.sheathPath?Math.min(this.progress,this.sheathPath.length):0}#H(t,e=new Rt){if(!this.sheathPath)return null;const n=It(t,0,this.sheathPath.length);return e.copy(this.sheathPath.start).addScaledVector(this.sheathPath.dir,n)}#W(t){const e=this.sheathPath?.length||0;if(t<=e+.5)return;this.pathSamples.length||this.#k(e);let n=this.#F();for(;n+this.pathSpacing<t;)n+=this.pathSpacing,this.#k(n);t>this.#F()+.5&&this.#k(t)}#k(t){const e=this.pathSamples.length;let n=this._pathSamplePool[e];return n||(n={distance:0,point:new Rt},this._pathSamplePool[e]=n),n.distance=t,this.#L(t,n.point),this.pathSamples[e]=n,n}#ot(t){const e=this.sheathPath?.length||0;if(!(t<=e+.5)){this.#W(t);for(let n=0;n<this.pathSamples.length;n++){const i=this.pathSamples[n];i.distance<=e+.5||i.distance>t+.5||this.#L(i.distance,i.point)}}}#ct(t){const e=this.sheathPath?.length||0,n=Math.max(t,e);for(;this.pathSamples.length>0&&this.pathSamples[this.pathSamples.length-1].distance>n;)this.pathSamples.pop();const i=this.pathSamples[this.pathSamples.length-1];i&&i.distance>t&&i.distance>e&&(i.distance=t)}#F(){const t=this.pathSamples[this.pathSamples.length-1];return Math.max(this.sheathPath?.length||0,t?t.distance:0)}#P(t,e=new Rt){const n=this.sheathPath?.length||0;if(this.sheathPath&&t<0)return e.copy(this.sheathPath.start).addScaledVector(this.sheathPath.dir,t);if(this.sheathPath&&t<=n+.5)return this.#H(t,e);if(!this.pathSamples.length){const a=this.#H(n,e);return a||this.#L(t,e)}const i=It(t,0,this.#F());let s=this.pathSamples[0];for(let a=1;a<this.pathSamples.length;a++){const o=this.pathSamples[a];if(o.distance>=i){const c=It((i-s.distance)/Math.max(1e-6,o.distance-s.distance),0,1);return e.copy(s.point).lerp(o.point,c)}s=o}return e.copy(s.point)}#lt(t,e=this.#u()){if(!this.freeNodes.length||t<=e.supportEnd+.5)return this.#P(t);const n=It(t,this.freeNodes[0].distance??e.supportEnd,this.progress);let i=this.freeNodes[0];for(let s=1;s<this.freeNodes.length;s++){const a=this.freeNodes[s];if((a.distance??n)>=n){const o=Math.max(1e-6,(a.distance??n)-(i.distance??n)),c=It((n-(i.distance??n))/o,0,1);return i.pos.clone().lerp(a.pos,c)}i=a}return i.pos.clone()}#ht(t,e,n=1){if(n<=0||!this.freeNodes.length||e.lengthSq()<1e-8||t<=(this.freeNodes[0].distance??0))return;let i=this.freeNodes[0];for(let a=1;a<this.freeNodes.length;a++){const o=this.freeNodes[a],c=i.distance??0,l=o.distance??c;if(t<=l+.5){const h=Math.max(1e-6,l-c),u=It((t-c)/h,0,1),d=e.clone().multiplyScalar(-.16*n),f=a===1?0:1-u,g=a===1?1:u;i.pos.addScaledVector(d,f),o.pos.addScaledVector(d,g),i.vel.addScaledVector(d,.18*f),o.vel.addScaledVector(d,.18*g);return}i=o}const s=this.freeNodes[this.freeNodes.length-1];s.pos.addScaledVector(e,-.16*n),s.vel.addScaledVector(e,-.18*n)}#ut(t,e,n,i=new Rt){const s=this._planePreviousTangent.subVectors(e,n);if(s.lengthSq()>1e-5){s.normalize();const c=this._planeCurvature.subVectors(t,s);if(c.addScaledVector(t,-c.dot(t)),c.lengthSq()>1e-5)return i.copy(c).normalize()}const a=Math.abs(t.y)<.85,o=this._planeHelper.set(a?0:1,a?1:0,0);return i.crossVectors(t,o).cross(t).normalize()}#L(t,e=new Rt){const n=this.tailProgressRef(),i=this.wire.nodes,s=It((t+this.guidewireLength-n)/this.segmentLength,0,i.length-1),a=Math.min(i.length-2,Math.floor(s)),o=s-a,c=i[a],l=i[a+1];return e.set(c.x+(l.x-c.x)*o,c.y+(l.y-c.y)*o,c.z+(l.z-c.z)*o)}#dt(t,e){return this.segmentLength*t-this.guidewireLength+e}#ft(t){return t===je||t==="bernstein"?je:So}}class Gy extends _c{constructor(t){super(t)}load(t,e,n,i){const s=this,a=new ap(this.manager);a.setPath(this.path),a.setResponseType("arraybuffer"),a.setRequestHeader(this.requestHeader),a.setWithCredentials(this.withCredentials),a.load(t,function(o){try{e(s.parse(o))}catch(c){i?i(c):console.error(c),s.manager.itemError(t)}},n,i)}parse(t){function e(l){const h=new DataView(l),u=32/8*3+32/8*3*3+16/8,d=h.getUint32(80,!0);if(80+32/8+d*u===h.byteLength)return!0;const g=[115,111,108,105,100];for(let x=0;x<5;x++)if(n(g,h,x))return!1;return!0}function n(l,h,u){for(let d=0,f=l.length;d<f;d++)if(l[d]!==h.getUint8(u+d))return!1;return!0}function i(l){const h=new DataView(l),u=h.getUint32(80,!0);let d,f,g,x=!1,m,p,_,v,S;for(let L=0;L<70;L++)h.getUint32(L,!1)==1129270351&&h.getUint8(L+4)==82&&h.getUint8(L+5)==61&&(x=!0,m=new Float32Array(u*3*3),p=h.getUint8(L+6)/255,_=h.getUint8(L+7)/255,v=h.getUint8(L+8)/255,S=h.getUint8(L+9)/255);const M=84,y=50,w=new me,A=new Float32Array(u*3*3),E=new Float32Array(u*3*3),T=new Qt;for(let L=0;L<u;L++){const C=M+L*y,F=h.getFloat32(C,!0),D=h.getFloat32(C+4,!0),N=h.getFloat32(C+8,!0);if(x){const B=h.getUint16(C+48,!0);(B&32768)===0?(d=(B&31)/31,f=(B>>5&31)/31,g=(B>>10&31)/31):(d=p,f=_,g=v)}for(let B=1;B<=3;B++){const G=C+B*12,z=L*3*3+(B-1)*3;A[z]=h.getFloat32(G,!0),A[z+1]=h.getFloat32(G+4,!0),A[z+2]=h.getFloat32(G+8,!0),E[z]=F,E[z+1]=D,E[z+2]=N,x&&(T.set(d,f,g).convertSRGBToLinear(),m[z]=T.r,m[z+1]=T.g,m[z+2]=T.b)}}return w.setAttribute("position",new Be(A,3)),w.setAttribute("normal",new Be(E,3)),x&&(w.setAttribute("color",new Be(m,3)),w.hasColors=!0,w.alpha=S),w}function s(l){const h=new me,u=/solid([\s\S]*?)endsolid/g,d=/facet([\s\S]*?)endfacet/g,f=/solid\s(.+)/;let g=0;const x=/[\s]+([+-]?(?:\d*)(?:\.\d*)?(?:[eE][+-]?\d+)?)/.source,m=new RegExp("vertex"+x+x+x,"g"),p=new RegExp("normal"+x+x+x,"g"),_=[],v=[],S=[],M=new R;let y,w=0,A=0,E=0;for(;(y=u.exec(l))!==null;){A=E;const T=y[0],L=(y=f.exec(T))!==null?y[1]:"";for(S.push(L);(y=d.exec(T))!==null;){let D=0,N=0;const B=y[0];for(;(y=p.exec(B))!==null;)M.x=parseFloat(y[1]),M.y=parseFloat(y[2]),M.z=parseFloat(y[3]),N++;for(;(y=m.exec(B))!==null;)_.push(parseFloat(y[1]),parseFloat(y[2]),parseFloat(y[3])),v.push(M.x,M.y,M.z),D++,E++;N!==1&&console.error("THREE.STLLoader: Something isn't right with the normal of face number "+g),D!==3&&console.error("THREE.STLLoader: Something isn't right with the vertices of face number "+g),g++}const C=A,F=E-A;h.userData.groupNames=S,h.addGroup(C,F,w),w++}return h.setAttribute("position",new ie(_,3)),h.setAttribute("normal",new ie(v,3)),h}function a(l){return typeof l!="string"?new TextDecoder().decode(l):l}function o(l){if(typeof l=="string"){const h=new Uint8Array(l.length);for(let u=0;u<l.length;u++)h[u]=l.charCodeAt(u)&255;return h.buffer||h}else return l}const c=o(t);return e(c)?i(c):s(a(t))}}const Lp=0,Vy=1,ky=2,of=2,Il=1.25,cf=1,Ta=32,Sc=65535,Hy=Math.pow(2,-24),Nl=Symbol("SKIP_GENERATION");function Wy(r){return r.index?r.index.count:r.attributes.position.count}function br(r){return Wy(r)/3}function Xy(r,t=ArrayBuffer){return r>65535?new Uint32Array(new t(4*r)):new Uint16Array(new t(2*r))}function Yy(r,t){if(!r.index){const e=r.attributes.position.count,n=t.useSharedArrayBuffer?SharedArrayBuffer:ArrayBuffer,i=Xy(e,n);r.setIndex(new Be(i,1));for(let s=0;s<e;s++)i[s]=s}}function Dp(r,t){const e=br(r),n=t||r.drawRange,i=n.start/3,s=(n.start+n.count)/3,a=Math.max(0,i),o=Math.min(e,s)-a;return[{offset:Math.floor(a),count:Math.floor(o)}]}function Ip(r,t){if(!r.groups||!r.groups.length)return Dp(r,t);const e=[],n=new Set,i=t||r.drawRange,s=i.start/3,a=(i.start+i.count)/3;for(const c of r.groups){const l=c.start/3,h=(c.start+c.count)/3;n.add(Math.max(s,l)),n.add(Math.min(a,h))}const o=Array.from(n.values()).sort((c,l)=>c-l);for(let c=0;c<o.length-1;c++){const l=o[c],h=o[c+1];e.push({offset:Math.floor(l),count:Math.floor(h-l)})}return e}function qy(r,t){const e=br(r),n=Ip(r,t).sort((a,o)=>a.offset-o.offset),i=n[n.length-1];i.count=Math.min(e-i.offset,i.count);let s=0;return n.forEach(({count:a})=>s+=a),e!==s}function Fl(r,t,e,n,i){let s=1/0,a=1/0,o=1/0,c=-1/0,l=-1/0,h=-1/0,u=1/0,d=1/0,f=1/0,g=-1/0,x=-1/0,m=-1/0;for(let p=t*6,_=(t+e)*6;p<_;p+=6){const v=r[p+0],S=r[p+1],M=v-S,y=v+S;M<s&&(s=M),y>c&&(c=y),v<u&&(u=v),v>g&&(g=v);const w=r[p+2],A=r[p+3],E=w-A,T=w+A;E<a&&(a=E),T>l&&(l=T),w<d&&(d=w),w>x&&(x=w);const L=r[p+4],C=r[p+5],F=L-C,D=L+C;F<o&&(o=F),D>h&&(h=D),L<f&&(f=L),L>m&&(m=L)}n[0]=s,n[1]=a,n[2]=o,n[3]=c,n[4]=l,n[5]=h,i[0]=u,i[1]=d,i[2]=f,i[3]=g,i[4]=x,i[5]=m}function Zy(r,t=null,e=null,n=null){const i=r.attributes.position,s=r.index?r.index.array:null,a=br(r),o=i.normalized;let c;t===null?(c=new Float32Array(a*6),e=0,n=a):(c=t,e=e||0,n=n||a);const l=i.array,h=i.offset||0;let u=3;i.isInterleavedBufferAttribute&&(u=i.data.stride);const d=["getX","getY","getZ"];for(let f=e;f<e+n;f++){const g=f*3,x=f*6;let m=g+0,p=g+1,_=g+2;s&&(m=s[m],p=s[p],_=s[_]),o||(m=m*u+h,p=p*u+h,_=_*u+h);for(let v=0;v<3;v++){let S,M,y;o?(S=i[d[v]](m),M=i[d[v]](p),y=i[d[v]](_)):(S=l[m+v],M=l[p+v],y=l[_+v]);let w=S;M<w&&(w=M),y<w&&(w=y);let A=S;M>A&&(A=M),y>A&&(A=y);const E=(A-w)/2,T=v*2;c[x+T+0]=w+E,c[x+T+1]=E+(Math.abs(w)+E)*Hy}}return c}function Ne(r,t,e){return e.min.x=t[r],e.min.y=t[r+1],e.min.z=t[r+2],e.max.x=t[r+3],e.max.y=t[r+4],e.max.z=t[r+5],e}function lf(r){let t=-1,e=-1/0;for(let n=0;n<3;n++){const i=r[n+3]-r[n];i>e&&(e=i,t=n)}return t}function hf(r,t){t.set(r)}function uf(r,t,e){let n,i;for(let s=0;s<3;s++){const a=s+3;n=r[s],i=t[s],e[s]=n<i?n:i,n=r[a],i=t[a],e[a]=n>i?n:i}}function Eo(r,t,e){for(let n=0;n<3;n++){const i=t[r+2*n],s=t[r+2*n+1],a=i-s,o=i+s;a<e[n]&&(e[n]=a),o>e[n+3]&&(e[n+3]=o)}}function na(r){const t=r[3]-r[0],e=r[4]-r[1],n=r[5]-r[2];return 2*(t*e+e*n+n*t)}const Di=32,jy=(r,t)=>r.candidate-t.candidate,$i=new Array(Di).fill().map(()=>({count:0,bounds:new Float32Array(6),rightCacheBounds:new Float32Array(6),leftCacheBounds:new Float32Array(6),candidate:0})),wo=new Float32Array(6);function $y(r,t,e,n,i,s){let a=-1,o=0;if(s===Lp)a=lf(t),a!==-1&&(o=(t[a]+t[a+3])/2);else if(s===Vy)a=lf(r),a!==-1&&(o=Ky(e,n,i,a));else if(s===ky){const c=na(r);let l=Il*i;const h=n*6,u=(n+i)*6;for(let d=0;d<3;d++){const f=t[d],m=(t[d+3]-f)/Di;if(i<Di/4){const p=[...$i];p.length=i;let _=0;for(let S=h;S<u;S+=6,_++){const M=p[_];M.candidate=e[S+2*d],M.count=0;const{bounds:y,leftCacheBounds:w,rightCacheBounds:A}=M;for(let E=0;E<3;E++)A[E]=1/0,A[E+3]=-1/0,w[E]=1/0,w[E+3]=-1/0,y[E]=1/0,y[E+3]=-1/0;Eo(S,e,y)}p.sort(jy);let v=i;for(let S=0;S<v;S++){const M=p[S];for(;S+1<v&&p[S+1].candidate===M.candidate;)p.splice(S+1,1),v--}for(let S=h;S<u;S+=6){const M=e[S+2*d];for(let y=0;y<v;y++){const w=p[y];M>=w.candidate?Eo(S,e,w.rightCacheBounds):(Eo(S,e,w.leftCacheBounds),w.count++)}}for(let S=0;S<v;S++){const M=p[S],y=M.count,w=i-M.count,A=M.leftCacheBounds,E=M.rightCacheBounds;let T=0;y!==0&&(T=na(A)/c);let L=0;w!==0&&(L=na(E)/c);const C=cf+Il*(T*y+L*w);C<l&&(a=d,l=C,o=M.candidate)}}else{for(let v=0;v<Di;v++){const S=$i[v];S.count=0,S.candidate=f+m+v*m;const M=S.bounds;for(let y=0;y<3;y++)M[y]=1/0,M[y+3]=-1/0}for(let v=h;v<u;v+=6){let y=~~((e[v+2*d]-f)/m);y>=Di&&(y=Di-1);const w=$i[y];w.count++,Eo(v,e,w.bounds)}const p=$i[Di-1];hf(p.bounds,p.rightCacheBounds);for(let v=Di-2;v>=0;v--){const S=$i[v],M=$i[v+1];uf(S.bounds,M.rightCacheBounds,S.rightCacheBounds)}let _=0;for(let v=0;v<Di-1;v++){const S=$i[v],M=S.count,y=S.bounds,A=$i[v+1].rightCacheBounds;M!==0&&(_===0?hf(y,wo):uf(y,wo,wo)),_+=M;let E=0,T=0;_!==0&&(E=na(wo)/c);const L=i-_;L!==0&&(T=na(A)/c);const C=cf+Il*(E*_+T*L);C<l&&(a=d,l=C,o=S.candidate)}}}}else console.warn(`MeshBVH: Invalid build strategy value ${s} used.`);return{axis:a,pos:o}}function Ky(r,t,e,n){let i=0;for(let s=t,a=t+e;s<a;s++)i+=r[s*6+n*2];return i/e}class Bl{constructor(){this.boundingData=new Float32Array(6)}}function Jy(r,t,e,n,i,s){let a=n,o=n+i-1;const c=s.pos,l=s.axis*2;for(;;){for(;a<=o&&e[a*6+l]<c;)a++;for(;a<=o&&e[o*6+l]>=c;)o--;if(a<o){for(let h=0;h<3;h++){let u=t[a*3+h];t[a*3+h]=t[o*3+h],t[o*3+h]=u}for(let h=0;h<6;h++){let u=e[a*6+h];e[a*6+h]=e[o*6+h],e[o*6+h]=u}a++,o--}else return a}}function Qy(r,t,e,n,i,s){let a=n,o=n+i-1;const c=s.pos,l=s.axis*2;for(;;){for(;a<=o&&e[a*6+l]<c;)a++;for(;a<=o&&e[o*6+l]>=c;)o--;if(a<o){let h=r[a];r[a]=r[o],r[o]=h;for(let u=0;u<6;u++){let d=e[a*6+u];e[a*6+u]=e[o*6+u],e[o*6+u]=d}a++,o--}else return a}}function wn(r,t){return t[r+15]===65535}function Nn(r,t){return t[r+6]}function kn(r,t){return t[r+14]}function Hn(r){return r+8}function Wn(r,t){return t[r+6]}function Np(r,t){return t[r+7]}let Fp,ma,Xo,Bp;const tE=Math.pow(2,32);function uh(r){return"count"in r?1:1+uh(r.left)+uh(r.right)}function eE(r,t,e){return Fp=new Float32Array(e),ma=new Uint32Array(e),Xo=new Uint16Array(e),Bp=new Uint8Array(e),dh(r,t)}function dh(r,t){const e=r/4,n=r/2,i="count"in t,s=t.boundingData;for(let a=0;a<6;a++)Fp[e+a]=s[a];if(i)if(t.buffer){const a=t.buffer;Bp.set(new Uint8Array(a),r);for(let o=r,c=r+a.byteLength;o<c;o+=Ta){const l=o/2;wn(l,Xo)||(ma[o/4+6]+=e)}return r+a.byteLength}else{const a=t.offset,o=t.count;return ma[e+6]=a,Xo[n+14]=o,Xo[n+15]=Sc,r+Ta}else{const a=t.left,o=t.right,c=t.splitAxis;let l;if(l=dh(r+Ta,a),l/4>tE)throw new Error("MeshBVH: Cannot store child pointer greater than 32 bits.");return ma[e+6]=l/4,l=dh(l,o),ma[e+7]=c,l}}function nE(r,t){const e=(r.index?r.index.count:r.attributes.position.count)/3,n=e>2**16,i=n?4:2,s=t?new SharedArrayBuffer(e*i):new ArrayBuffer(e*i),a=n?new Uint32Array(s):new Uint16Array(s);for(let o=0,c=a.length;o<c;o++)a[o]=o;return a}function iE(r,t,e,n,i){const{maxDepth:s,verbose:a,maxLeafTris:o,strategy:c,onProgress:l,indirect:h}=i,u=r._indirectBuffer,d=r.geometry,f=d.index?d.index.array:null,g=h?Qy:Jy,x=br(d),m=new Float32Array(6);let p=!1;const _=new Bl;return Fl(t,e,n,_.boundingData,m),S(_,e,n,m),_;function v(M){l&&l(M/x)}function S(M,y,w,A=null,E=0){if(!p&&E>=s&&(p=!0,a&&(console.warn(`MeshBVH: Max depth of ${s} reached when generating BVH. Consider increasing maxDepth.`),console.warn(d))),w<=o||E>=s)return v(y+w),M.offset=y,M.count=w,M;const T=$y(M.boundingData,A,t,y,w,c);if(T.axis===-1)return v(y+w),M.offset=y,M.count=w,M;const L=g(u,f,t,y,w,T);if(L===y||L===y+w)v(y+w),M.offset=y,M.count=w;else{M.splitAxis=T.axis;const C=new Bl,F=y,D=L-y;M.left=C,Fl(t,F,D,C.boundingData,m),S(C,F,D,m,E+1);const N=new Bl,B=L,G=w-D;M.right=N,Fl(t,B,G,N.boundingData,m),S(N,B,G,m,E+1)}return M}}function sE(r,t){const e=r.geometry;t.indirect&&(r._indirectBuffer=nE(e,t.useSharedArrayBuffer),qy(e,t.range)&&!t.verbose&&console.warn('MeshBVH: Provided geometry contains groups or a range that do not fully span the vertex contents while using the "indirect" option. BVH may incorrectly report intersections on unrendered portions of the geometry.')),r._indirectBuffer||Yy(e,t);const n=t.useSharedArrayBuffer?SharedArrayBuffer:ArrayBuffer,i=Zy(e),s=t.indirect?Dp(e,t.range):Ip(e,t.range);r._roots=s.map(a=>{const o=iE(r,i,a.offset,a.count,t),c=uh(o),l=new n(Ta*c);return eE(0,o,l),l})}class zi{constructor(){this.min=1/0,this.max=-1/0}setFromPointsField(t,e){let n=1/0,i=-1/0;for(let s=0,a=t.length;s<a;s++){const c=t[s][e];n=c<n?c:n,i=c>i?c:i}this.min=n,this.max=i}setFromPoints(t,e){let n=1/0,i=-1/0;for(let s=0,a=e.length;s<a;s++){const o=e[s],c=t.dot(o);n=c<n?c:n,i=c>i?c:i}this.min=n,this.max=i}isSeparated(t){return this.min>t.max||t.min>this.max}}zi.prototype.setFromBox=(function(){const r=new R;return function(e,n){const i=n.min,s=n.max;let a=1/0,o=-1/0;for(let c=0;c<=1;c++)for(let l=0;l<=1;l++)for(let h=0;h<=1;h++){r.x=i.x*c+s.x*(1-c),r.y=i.y*l+s.y*(1-l),r.z=i.z*h+s.z*(1-h);const u=e.dot(r);a=Math.min(u,a),o=Math.max(u,o)}this.min=a,this.max=o}})();const rE=(function(){const r=new R,t=new R,e=new R;return function(i,s,a){const o=i.start,c=r,l=s.start,h=t;e.subVectors(o,l),r.subVectors(i.end,i.start),t.subVectors(s.end,s.start);const u=e.dot(h),d=h.dot(c),f=h.dot(h),g=e.dot(c),m=c.dot(c)*f-d*d;let p,_;m!==0?p=(u*d-g*f)/m:p=0,_=(u+p*d)/f,a.x=p,a.y=_}})(),Zh=(function(){const r=new Mt,t=new R,e=new R;return function(i,s,a,o){rE(i,s,r);let c=r.x,l=r.y;if(c>=0&&c<=1&&l>=0&&l<=1){i.at(c,a),s.at(l,o);return}else if(c>=0&&c<=1){l<0?s.at(0,o):s.at(1,o),i.closestPointToPoint(o,!0,a);return}else if(l>=0&&l<=1){c<0?i.at(0,a):i.at(1,a),s.closestPointToPoint(a,!0,o);return}else{let h;c<0?h=i.start:h=i.end;let u;l<0?u=s.start:u=s.end;const d=t,f=e;if(i.closestPointToPoint(u,!0,t),s.closestPointToPoint(h,!0,e),d.distanceToSquared(u)<=f.distanceToSquared(h)){a.copy(d),o.copy(u);return}else{a.copy(h),o.copy(f);return}}}})(),aE=(function(){const r=new R,t=new R,e=new Ii,n=new Fi;return function(s,a){const{radius:o,center:c}=s,{a:l,b:h,c:u}=a;if(n.start=l,n.end=h,n.closestPointToPoint(c,!0,r).distanceTo(c)<=o||(n.start=l,n.end=u,n.closestPointToPoint(c,!0,r).distanceTo(c)<=o)||(n.start=h,n.end=u,n.closestPointToPoint(c,!0,r).distanceTo(c)<=o))return!0;const x=a.getPlane(e);if(Math.abs(x.distanceToPoint(c))<=o){const p=x.projectPoint(c,t);if(a.containsPoint(p))return!0}return!1}})(),oE=1e-15;function Ul(r){return Math.abs(r)<oE}class si extends on{constructor(...t){super(...t),this.isExtendedTriangle=!0,this.satAxes=new Array(4).fill().map(()=>new R),this.satBounds=new Array(4).fill().map(()=>new zi),this.points=[this.a,this.b,this.c],this.sphere=new as,this.plane=new Ii,this.needsUpdate=!0}intersectsSphere(t){return aE(t,this)}update(){const t=this.a,e=this.b,n=this.c,i=this.points,s=this.satAxes,a=this.satBounds,o=s[0],c=a[0];this.getNormal(o),c.setFromPoints(o,i);const l=s[1],h=a[1];l.subVectors(t,e),h.setFromPoints(l,i);const u=s[2],d=a[2];u.subVectors(e,n),d.setFromPoints(u,i);const f=s[3],g=a[3];f.subVectors(n,t),g.setFromPoints(f,i),this.sphere.setFromPoints(this.points),this.plane.setFromNormalAndCoplanarPoint(o,t),this.needsUpdate=!1}}si.prototype.closestPointToSegment=(function(){const r=new R,t=new R,e=new Fi;return function(i,s=null,a=null){const{start:o,end:c}=i,l=this.points;let h,u=1/0;for(let d=0;d<3;d++){const f=(d+1)%3;e.start.copy(l[d]),e.end.copy(l[f]),Zh(e,i,r,t),h=r.distanceToSquared(t),h<u&&(u=h,s&&s.copy(r),a&&a.copy(t))}return this.closestPointToPoint(o,r),h=o.distanceToSquared(r),h<u&&(u=h,s&&s.copy(r),a&&a.copy(o)),this.closestPointToPoint(c,r),h=c.distanceToSquared(r),h<u&&(u=h,s&&s.copy(r),a&&a.copy(c)),Math.sqrt(u)}})();si.prototype.intersectsTriangle=(function(){const r=new si,t=new Array(3),e=new Array(3),n=new zi,i=new zi,s=new R,a=new R,o=new R,c=new R,l=new R,h=new Fi,u=new Fi,d=new Fi,f=new R;function g(x,m,p){const _=x.points;let v=0,S=-1;for(let M=0;M<3;M++){const{start:y,end:w}=h;y.copy(_[M]),w.copy(_[(M+1)%3]),h.delta(a);const A=Ul(m.distanceToPoint(y));if(Ul(m.normal.dot(a))&&A){p.copy(h),v=2;break}const E=m.intersectLine(h,f);if(!E&&A&&f.copy(y),(E||A)&&!Ul(f.distanceTo(w))){if(v<=1)(v===1?p.start:p.end).copy(f),A&&(S=v);else if(v>=2){(S===1?p.start:p.end).copy(f),v=2;break}if(v++,v===2&&S===-1)break}}return v}return function(m,p=null,_=!1){this.needsUpdate&&this.update(),m.isExtendedTriangle?m.needsUpdate&&m.update():(r.copy(m),r.update(),m=r);const v=this.plane,S=m.plane;if(Math.abs(v.normal.dot(S.normal))>1-1e-10){const M=this.satBounds,y=this.satAxes;e[0]=m.a,e[1]=m.b,e[2]=m.c;for(let E=0;E<4;E++){const T=M[E],L=y[E];if(n.setFromPoints(L,e),T.isSeparated(n))return!1}const w=m.satBounds,A=m.satAxes;t[0]=this.a,t[1]=this.b,t[2]=this.c;for(let E=0;E<4;E++){const T=w[E],L=A[E];if(n.setFromPoints(L,t),T.isSeparated(n))return!1}for(let E=0;E<4;E++){const T=y[E];for(let L=0;L<4;L++){const C=A[L];if(s.crossVectors(T,C),n.setFromPoints(s,t),i.setFromPoints(s,e),n.isSeparated(i))return!1}}return p&&(_||console.warn("ExtendedTriangle.intersectsTriangle: Triangles are coplanar which does not support an output edge. Setting edge to 0, 0, 0."),p.start.set(0,0,0),p.end.set(0,0,0)),!0}else{const M=g(this,S,u);if(M===1&&m.containsPoint(u.end))return p&&(p.start.copy(u.end),p.end.copy(u.end)),!0;if(M!==2)return!1;const y=g(m,v,d);if(y===1&&this.containsPoint(d.end))return p&&(p.start.copy(d.end),p.end.copy(d.end)),!0;if(y!==2)return!1;if(u.delta(o),d.delta(c),o.dot(c)<0){let F=d.start;d.start=d.end,d.end=F}const w=u.start.dot(o),A=u.end.dot(o),E=d.start.dot(o),T=d.end.dot(o),L=A<E,C=w<T;return w!==T&&E!==A&&L===C?!1:(p&&(l.subVectors(u.start,d.start),l.dot(o)>0?p.start.copy(u.start):p.start.copy(d.start),l.subVectors(u.end,d.end),l.dot(o)<0?p.end.copy(u.end):p.end.copy(d.end)),!0)}}})();si.prototype.distanceToPoint=(function(){const r=new R;return function(e){return this.closestPointToPoint(e,r),e.distanceTo(r)}})();si.prototype.distanceToTriangle=(function(){const r=new R,t=new R,e=["a","b","c"],n=new Fi,i=new Fi;return function(a,o=null,c=null){const l=o||c?n:null;if(this.intersectsTriangle(a,l))return(o||c)&&(o&&l.getCenter(o),c&&l.getCenter(c)),0;let h=1/0;for(let u=0;u<3;u++){let d;const f=e[u],g=a[f];this.closestPointToPoint(g,r),d=g.distanceToSquared(r),d<h&&(h=d,o&&o.copy(r),c&&c.copy(g));const x=this[f];a.closestPointToPoint(x,r),d=x.distanceToSquared(r),d<h&&(h=d,o&&o.copy(x),c&&c.copy(r))}for(let u=0;u<3;u++){const d=e[u],f=e[(u+1)%3];n.set(this[d],this[f]);for(let g=0;g<3;g++){const x=e[g],m=e[(g+1)%3];i.set(a[x],a[m]),Zh(n,i,r,t);const p=r.distanceToSquared(t);p<h&&(h=p,o&&o.copy(r),c&&c.copy(t))}}return Math.sqrt(h)}})();class vn{constructor(t,e,n){this.isOrientedBox=!0,this.min=new R,this.max=new R,this.matrix=new re,this.invMatrix=new re,this.points=new Array(8).fill().map(()=>new R),this.satAxes=new Array(3).fill().map(()=>new R),this.satBounds=new Array(3).fill().map(()=>new zi),this.alignedSatBounds=new Array(3).fill().map(()=>new zi),this.needsUpdate=!1,t&&this.min.copy(t),e&&this.max.copy(e),n&&this.matrix.copy(n)}set(t,e,n){this.min.copy(t),this.max.copy(e),this.matrix.copy(n),this.needsUpdate=!0}copy(t){this.min.copy(t.min),this.max.copy(t.max),this.matrix.copy(t.matrix),this.needsUpdate=!0}}vn.prototype.update=(function(){return function(){const t=this.matrix,e=this.min,n=this.max,i=this.points;for(let l=0;l<=1;l++)for(let h=0;h<=1;h++)for(let u=0;u<=1;u++){const d=1*l|2*h|4*u,f=i[d];f.x=l?n.x:e.x,f.y=h?n.y:e.y,f.z=u?n.z:e.z,f.applyMatrix4(t)}const s=this.satBounds,a=this.satAxes,o=i[0];for(let l=0;l<3;l++){const h=a[l],u=s[l],d=1<<l,f=i[d];h.subVectors(o,f),u.setFromPoints(h,i)}const c=this.alignedSatBounds;c[0].setFromPointsField(i,"x"),c[1].setFromPointsField(i,"y"),c[2].setFromPointsField(i,"z"),this.invMatrix.copy(this.matrix).invert(),this.needsUpdate=!1}})();vn.prototype.intersectsBox=(function(){const r=new zi;return function(e){this.needsUpdate&&this.update();const n=e.min,i=e.max,s=this.satBounds,a=this.satAxes,o=this.alignedSatBounds;if(r.min=n.x,r.max=i.x,o[0].isSeparated(r)||(r.min=n.y,r.max=i.y,o[1].isSeparated(r))||(r.min=n.z,r.max=i.z,o[2].isSeparated(r)))return!1;for(let c=0;c<3;c++){const l=a[c],h=s[c];if(r.setFromBox(l,e),h.isSeparated(r))return!1}return!0}})();vn.prototype.intersectsTriangle=(function(){const r=new si,t=new Array(3),e=new zi,n=new zi,i=new R;return function(a){this.needsUpdate&&this.update(),a.isExtendedTriangle?a.needsUpdate&&a.update():(r.copy(a),r.update(),a=r);const o=this.satBounds,c=this.satAxes;t[0]=a.a,t[1]=a.b,t[2]=a.c;for(let d=0;d<3;d++){const f=o[d],g=c[d];if(e.setFromPoints(g,t),f.isSeparated(e))return!1}const l=a.satBounds,h=a.satAxes,u=this.points;for(let d=0;d<3;d++){const f=l[d],g=h[d];if(e.setFromPoints(g,u),f.isSeparated(e))return!1}for(let d=0;d<3;d++){const f=c[d];for(let g=0;g<4;g++){const x=h[g];if(i.crossVectors(f,x),e.setFromPoints(i,t),n.setFromPoints(i,u),e.isSeparated(n))return!1}}return!0}})();vn.prototype.closestPointToPoint=(function(){return function(t,e){return this.needsUpdate&&this.update(),e.copy(t).applyMatrix4(this.invMatrix).clamp(this.min,this.max).applyMatrix4(this.matrix),e}})();vn.prototype.distanceToPoint=(function(){const r=new R;return function(e){return this.closestPointToPoint(e,r),e.distanceTo(r)}})();vn.prototype.distanceToBox=(function(){const r=["x","y","z"],t=new Array(12).fill().map(()=>new Fi),e=new Array(12).fill().map(()=>new Fi),n=new R,i=new R;return function(a,o=0,c=null,l=null){if(this.needsUpdate&&this.update(),this.intersectsBox(a))return(c||l)&&(a.getCenter(i),this.closestPointToPoint(i,n),a.closestPointToPoint(n,i),c&&c.copy(n),l&&l.copy(i)),0;const h=o*o,u=a.min,d=a.max,f=this.points;let g=1/0;for(let m=0;m<8;m++){const p=f[m];i.copy(p).clamp(u,d);const _=p.distanceToSquared(i);if(_<g&&(g=_,c&&c.copy(p),l&&l.copy(i),_<h))return Math.sqrt(_)}let x=0;for(let m=0;m<3;m++)for(let p=0;p<=1;p++)for(let _=0;_<=1;_++){const v=(m+1)%3,S=(m+2)%3,M=p<<v|_<<S,y=1<<m|p<<v|_<<S,w=f[M],A=f[y];t[x].set(w,A);const T=r[m],L=r[v],C=r[S],F=e[x],D=F.start,N=F.end;D[T]=u[T],D[L]=p?u[L]:d[L],D[C]=_?u[C]:d[L],N[T]=d[T],N[L]=p?u[L]:d[L],N[C]=_?u[C]:d[L],x++}for(let m=0;m<=1;m++)for(let p=0;p<=1;p++)for(let _=0;_<=1;_++){i.x=m?d.x:u.x,i.y=p?d.y:u.y,i.z=_?d.z:u.z,this.closestPointToPoint(i,n);const v=i.distanceToSquared(n);if(v<g&&(g=v,c&&c.copy(n),l&&l.copy(i),v<h))return Math.sqrt(v)}for(let m=0;m<12;m++){const p=t[m];for(let _=0;_<12;_++){const v=e[_];Zh(p,v,n,i);const S=n.distanceToSquared(i);if(S<g&&(g=S,c&&c.copy(n),l&&l.copy(i),S<h))return Math.sqrt(S)}}return Math.sqrt(g)}})();class jh{constructor(t){this._getNewPrimitive=t,this._primitives=[]}getPrimitive(){const t=this._primitives;return t.length===0?this._getNewPrimitive():t.pop()}releasePrimitive(t){this._primitives.push(t)}}class cE extends jh{constructor(){super(()=>new si)}}const Xn=new cE;class lE{constructor(){this.float32Array=null,this.uint16Array=null,this.uint32Array=null;const t=[];let e=null;this.setBuffer=n=>{e&&t.push(e),e=n,this.float32Array=new Float32Array(n),this.uint16Array=new Uint16Array(n),this.uint32Array=new Uint32Array(n)},this.clearBuffer=()=>{e=null,this.float32Array=null,this.uint16Array=null,this.uint32Array=null,t.length!==0&&this.setBuffer(t.pop())}}}const Ee=new lE;let es,pr;const Js=[],To=new jh(()=>new rn);function hE(r,t,e,n,i,s){es=To.getPrimitive(),pr=To.getPrimitive(),Js.push(es,pr),Ee.setBuffer(r._roots[t]);const a=fh(0,r.geometry,e,n,i,s);Ee.clearBuffer(),To.releasePrimitive(es),To.releasePrimitive(pr),Js.pop(),Js.pop();const o=Js.length;return o>0&&(pr=Js[o-1],es=Js[o-2]),a}function fh(r,t,e,n,i=null,s=0,a=0){const{float32Array:o,uint16Array:c,uint32Array:l}=Ee;let h=r*2;if(wn(h,c)){const d=Nn(r,l),f=kn(h,c);return Ne(r,o,es),n(d,f,!1,a,s+r,es)}else{let T=function(C){const{uint16Array:F,uint32Array:D}=Ee;let N=C*2;for(;!wn(N,F);)C=Hn(C),N=C*2;return Nn(C,D)},L=function(C){const{uint16Array:F,uint32Array:D}=Ee;let N=C*2;for(;!wn(N,F);)C=Wn(C,D),N=C*2;return Nn(C,D)+kn(N,F)};const d=Hn(r),f=Wn(r,l);let g=d,x=f,m,p,_,v;if(i&&(_=es,v=pr,Ne(g,o,_),Ne(x,o,v),m=i(_),p=i(v),p<m)){g=f,x=d;const C=m;m=p,p=C,_=v}_||(_=es,Ne(g,o,_));const S=wn(g*2,c),M=e(_,S,m,a+1,s+g);let y;if(M===of){const C=T(g),D=L(g)-C;y=n(C,D,!0,a+1,s+g,_)}else y=M&&fh(g,t,e,n,i,s,a+1);if(y)return!0;v=pr,Ne(x,o,v);const w=wn(x*2,c),A=e(v,w,p,a+1,s+x);let E;if(A===of){const C=T(x),D=L(x)-C;E=n(C,D,!0,a+1,s+x,v)}else E=A&&fh(x,t,e,n,i,s,a+1);return!!E}}const ia=new R,zl=new R;function uE(r,t,e={},n=0,i=1/0){const s=n*n,a=i*i;let o=1/0,c=null;if(r.shapecast({boundsTraverseOrder:h=>(ia.copy(t).clamp(h.min,h.max),ia.distanceToSquared(t)),intersectsBounds:(h,u,d)=>d<o&&d<a,intersectsTriangle:(h,u)=>{h.closestPointToPoint(t,ia);const d=t.distanceToSquared(ia);return d<o&&(zl.copy(ia),o=d,c=u),d<s}}),o===1/0)return null;const l=Math.sqrt(o);return e.point?e.point.copy(zl):e.point=zl.clone(),e.distance=l,e.faceIndex=c,e}const dE=parseInt("160")>=169,gs=new R,xs=new R,_s=new R,Ao=new Mt,Co=new Mt,Ro=new Mt,df=new R,ff=new R,pf=new R,sa=new R;function fE(r,t,e,n,i,s,a,o){let c;if(s===1?c=r.intersectTriangle(n,e,t,!0,i):c=r.intersectTriangle(t,e,n,s!==2,i),c===null)return null;const l=r.origin.distanceTo(i);return l<a||l>o?null:{distance:l,point:i.clone()}}function pE(r,t,e,n,i,s,a,o,c,l,h){gs.fromBufferAttribute(t,s),xs.fromBufferAttribute(t,a),_s.fromBufferAttribute(t,o);const u=fE(r,gs,xs,_s,sa,c,l,h);if(u){const d=new R;on.getBarycoord(sa,gs,xs,_s,d),n&&(Ao.fromBufferAttribute(n,s),Co.fromBufferAttribute(n,a),Ro.fromBufferAttribute(n,o),u.uv=on.getInterpolation(sa,gs,xs,_s,Ao,Co,Ro,new Mt)),i&&(Ao.fromBufferAttribute(i,s),Co.fromBufferAttribute(i,a),Ro.fromBufferAttribute(i,o),u.uv1=on.getInterpolation(sa,gs,xs,_s,Ao,Co,Ro,new Mt)),e&&(df.fromBufferAttribute(e,s),ff.fromBufferAttribute(e,a),pf.fromBufferAttribute(e,o),u.normal=on.getInterpolation(sa,gs,xs,_s,df,ff,pf,new R),u.normal.dot(r.direction)>0&&u.normal.multiplyScalar(-1));const f={a:s,b:a,c:o,normal:new R,materialIndex:0};on.getNormal(gs,xs,_s,f.normal),u.face=f,u.faceIndex=s,dE&&(u.barycoord=d)}return u}function Mc(r,t,e,n,i,s,a){const o=n*3;let c=o+0,l=o+1,h=o+2;const u=r.index;r.index&&(c=u.getX(c),l=u.getX(l),h=u.getX(h));const{position:d,normal:f,uv:g,uv1:x}=r.attributes,m=pE(e,d,f,g,x,c,l,h,t,s,a);return m?(m.faceIndex=n,i&&i.push(m),m):null}function Xe(r,t,e,n){const i=r.a,s=r.b,a=r.c;let o=t,c=t+1,l=t+2;e&&(o=e.getX(o),c=e.getX(c),l=e.getX(l)),i.x=n.getX(o),i.y=n.getY(o),i.z=n.getZ(o),s.x=n.getX(c),s.y=n.getY(c),s.z=n.getZ(c),a.x=n.getX(l),a.y=n.getY(l),a.z=n.getZ(l)}function mE(r,t,e,n,i,s,a,o){const{geometry:c,_indirectBuffer:l}=r;for(let h=n,u=n+i;h<u;h++)Mc(c,t,e,h,s,a,o)}function gE(r,t,e,n,i,s,a){const{geometry:o,_indirectBuffer:c}=r;let l=1/0,h=null;for(let u=n,d=n+i;u<d;u++){let f;f=Mc(o,t,e,u,null,s,a),f&&f.distance<l&&(h=f,l=f.distance)}return h}function xE(r,t,e,n,i,s,a){const{geometry:o}=e,{index:c}=o,l=o.attributes.position;for(let h=r,u=t+r;h<u;h++){let d;if(d=h,Xe(a,d*3,c,l),a.needsUpdate=!0,n(a,d,i,s))return!0}return!1}function _E(r,t=null){t&&Array.isArray(t)&&(t=new Set(t));const e=r.geometry,n=e.index?e.index.array:null,i=e.attributes.position;let s,a,o,c,l=0;const h=r._roots;for(let d=0,f=h.length;d<f;d++)s=h[d],a=new Uint32Array(s),o=new Uint16Array(s),c=new Float32Array(s),u(0,l),l+=s.byteLength;function u(d,f,g=!1){const x=d*2;if(o[x+15]===Sc){const p=a[d+6],_=o[x+14];let v=1/0,S=1/0,M=1/0,y=-1/0,w=-1/0,A=-1/0;for(let E=3*p,T=3*(p+_);E<T;E++){let L=n[E];const C=i.getX(L),F=i.getY(L),D=i.getZ(L);C<v&&(v=C),C>y&&(y=C),F<S&&(S=F),F>w&&(w=F),D<M&&(M=D),D>A&&(A=D)}return c[d+0]!==v||c[d+1]!==S||c[d+2]!==M||c[d+3]!==y||c[d+4]!==w||c[d+5]!==A?(c[d+0]=v,c[d+1]=S,c[d+2]=M,c[d+3]=y,c[d+4]=w,c[d+5]=A,!0):!1}else{const p=d+8,_=a[d+6],v=p+f,S=_+f;let M=g,y=!1,w=!1;t?M||(y=t.has(v),w=t.has(S),M=!y&&!w):(y=!0,w=!0);const A=M||y,E=M||w;let T=!1;A&&(T=u(p,f,M));let L=!1;E&&(L=u(_,f,M));const C=T||L;if(C)for(let F=0;F<3;F++){const D=p+F,N=_+F,B=c[D],G=c[D+3],z=c[N],H=c[N+3];c[d+F]=B<z?B:z,c[d+F+3]=G>H?G:H}return C}}}function rs(r,t,e,n,i){let s,a,o,c,l,h;const u=1/e.direction.x,d=1/e.direction.y,f=1/e.direction.z,g=e.origin.x,x=e.origin.y,m=e.origin.z;let p=t[r],_=t[r+3],v=t[r+1],S=t[r+3+1],M=t[r+2],y=t[r+3+2];return u>=0?(s=(p-g)*u,a=(_-g)*u):(s=(_-g)*u,a=(p-g)*u),d>=0?(o=(v-x)*d,c=(S-x)*d):(o=(S-x)*d,c=(v-x)*d),s>c||o>a||((o>s||isNaN(s))&&(s=o),(c<a||isNaN(a))&&(a=c),f>=0?(l=(M-m)*f,h=(y-m)*f):(l=(y-m)*f,h=(M-m)*f),s>h||l>a)?!1:((l>s||s!==s)&&(s=l),(h<a||a!==a)&&(a=h),s<=i&&a>=n)}function vE(r,t,e,n,i,s,a,o){const{geometry:c,_indirectBuffer:l}=r;for(let h=n,u=n+i;h<u;h++){let d=l?l[h]:h;Mc(c,t,e,d,s,a,o)}}function SE(r,t,e,n,i,s,a){const{geometry:o,_indirectBuffer:c}=r;let l=1/0,h=null;for(let u=n,d=n+i;u<d;u++){let f;f=Mc(o,t,e,c?c[u]:u,null,s,a),f&&f.distance<l&&(h=f,l=f.distance)}return h}function ME(r,t,e,n,i,s,a){const{geometry:o}=e,{index:c}=o,l=o.attributes.position;for(let h=r,u=t+r;h<u;h++){let d;if(d=e.resolveTriangleIndex(h),Xe(a,d*3,c,l),a.needsUpdate=!0,n(a,d,i,s))return!0}return!1}function yE(r,t,e,n,i,s,a){Ee.setBuffer(r._roots[t]),ph(0,r,e,n,i,s,a),Ee.clearBuffer()}function ph(r,t,e,n,i,s,a){const{float32Array:o,uint16Array:c,uint32Array:l}=Ee,h=r*2;if(wn(h,c)){const d=Nn(r,l),f=kn(h,c);mE(t,e,n,d,f,i,s,a)}else{const d=Hn(r);rs(d,o,n,s,a)&&ph(d,t,e,n,i,s,a);const f=Wn(r,l);rs(f,o,n,s,a)&&ph(f,t,e,n,i,s,a)}}const EE=["x","y","z"];function wE(r,t,e,n,i,s){Ee.setBuffer(r._roots[t]);const a=mh(0,r,e,n,i,s);return Ee.clearBuffer(),a}function mh(r,t,e,n,i,s){const{float32Array:a,uint16Array:o,uint32Array:c}=Ee;let l=r*2;if(wn(l,o)){const u=Nn(r,c),d=kn(l,o);return gE(t,e,n,u,d,i,s)}else{const u=Np(r,c),d=EE[u],g=n.direction[d]>=0;let x,m;g?(x=Hn(r),m=Wn(r,c)):(x=Wn(r,c),m=Hn(r));const _=rs(x,a,n,i,s)?mh(x,t,e,n,i,s):null;if(_){const M=_.point[d];if(g?M<=a[m+u]:M>=a[m+u+3])return _}const S=rs(m,a,n,i,s)?mh(m,t,e,n,i,s):null;return _&&S?_.distance<=S.distance?_:S:_||S||null}}const bo=new rn,Qs=new si,tr=new si,ra=new re,mf=new vn,Po=new vn;function TE(r,t,e,n){Ee.setBuffer(r._roots[t]);const i=gh(0,r,e,n);return Ee.clearBuffer(),i}function gh(r,t,e,n,i=null){const{float32Array:s,uint16Array:a,uint32Array:o}=Ee;let c=r*2;if(i===null&&(e.boundingBox||e.computeBoundingBox(),mf.set(e.boundingBox.min,e.boundingBox.max,n),i=mf),wn(c,a)){const h=t.geometry,u=h.index,d=h.attributes.position,f=e.index,g=e.attributes.position,x=Nn(r,o),m=kn(c,a);if(ra.copy(n).invert(),e.boundsTree)return Ne(r,s,Po),Po.matrix.copy(ra),Po.needsUpdate=!0,e.boundsTree.shapecast({intersectsBounds:_=>Po.intersectsBox(_),intersectsTriangle:_=>{_.a.applyMatrix4(n),_.b.applyMatrix4(n),_.c.applyMatrix4(n),_.needsUpdate=!0;for(let v=x*3,S=(m+x)*3;v<S;v+=3)if(Xe(tr,v,u,d),tr.needsUpdate=!0,_.intersectsTriangle(tr))return!0;return!1}});for(let p=x*3,_=(m+x)*3;p<_;p+=3){Xe(Qs,p,u,d),Qs.a.applyMatrix4(ra),Qs.b.applyMatrix4(ra),Qs.c.applyMatrix4(ra),Qs.needsUpdate=!0;for(let v=0,S=f.count;v<S;v+=3)if(Xe(tr,v,f,g),tr.needsUpdate=!0,Qs.intersectsTriangle(tr))return!0}}else{const h=r+8,u=o[r+6];return Ne(h,s,bo),!!(i.intersectsBox(bo)&&gh(h,t,e,n,i)||(Ne(u,s,bo),i.intersectsBox(bo)&&gh(u,t,e,n,i)))}}const Lo=new re,Ol=new vn,aa=new vn,AE=new R,CE=new R,RE=new R,bE=new R;function PE(r,t,e,n={},i={},s=0,a=1/0){t.boundingBox||t.computeBoundingBox(),Ol.set(t.boundingBox.min,t.boundingBox.max,e),Ol.needsUpdate=!0;const o=r.geometry,c=o.attributes.position,l=o.index,h=t.attributes.position,u=t.index,d=Xn.getPrimitive(),f=Xn.getPrimitive();let g=AE,x=CE,m=null,p=null;i&&(m=RE,p=bE);let _=1/0,v=null,S=null;return Lo.copy(e).invert(),aa.matrix.copy(Lo),r.shapecast({boundsTraverseOrder:M=>Ol.distanceToBox(M),intersectsBounds:(M,y,w)=>w<_&&w<a?(y&&(aa.min.copy(M.min),aa.max.copy(M.max),aa.needsUpdate=!0),!0):!1,intersectsRange:(M,y)=>{if(t.boundsTree)return t.boundsTree.shapecast({boundsTraverseOrder:A=>aa.distanceToBox(A),intersectsBounds:(A,E,T)=>T<_&&T<a,intersectsRange:(A,E)=>{for(let T=A,L=A+E;T<L;T++){Xe(f,3*T,u,h),f.a.applyMatrix4(e),f.b.applyMatrix4(e),f.c.applyMatrix4(e),f.needsUpdate=!0;for(let C=M,F=M+y;C<F;C++){Xe(d,3*C,l,c),d.needsUpdate=!0;const D=d.distanceToTriangle(f,g,m);if(D<_&&(x.copy(g),p&&p.copy(m),_=D,v=C,S=T),D<s)return!0}}}});{const w=br(t);for(let A=0,E=w;A<E;A++){Xe(f,3*A,u,h),f.a.applyMatrix4(e),f.b.applyMatrix4(e),f.c.applyMatrix4(e),f.needsUpdate=!0;for(let T=M,L=M+y;T<L;T++){Xe(d,3*T,l,c),d.needsUpdate=!0;const C=d.distanceToTriangle(f,g,m);if(C<_&&(x.copy(g),p&&p.copy(m),_=C,v=T,S=A),C<s)return!0}}}}}),Xn.releasePrimitive(d),Xn.releasePrimitive(f),_===1/0?null:(n.point?n.point.copy(x):n.point=x.clone(),n.distance=_,n.faceIndex=v,i&&(i.point?i.point.copy(p):i.point=p.clone(),i.point.applyMatrix4(Lo),x.applyMatrix4(Lo),i.distance=x.sub(i.point).length(),i.faceIndex=S),n)}function LE(r,t=null){t&&Array.isArray(t)&&(t=new Set(t));const e=r.geometry,n=e.index?e.index.array:null,i=e.attributes.position;let s,a,o,c,l=0;const h=r._roots;for(let d=0,f=h.length;d<f;d++)s=h[d],a=new Uint32Array(s),o=new Uint16Array(s),c=new Float32Array(s),u(0,l),l+=s.byteLength;function u(d,f,g=!1){const x=d*2;if(o[x+15]===Sc){const p=a[d+6],_=o[x+14];let v=1/0,S=1/0,M=1/0,y=-1/0,w=-1/0,A=-1/0;for(let E=p,T=p+_;E<T;E++){const L=3*r.resolveTriangleIndex(E);for(let C=0;C<3;C++){let F=L+C;F=n?n[F]:F;const D=i.getX(F),N=i.getY(F),B=i.getZ(F);D<v&&(v=D),D>y&&(y=D),N<S&&(S=N),N>w&&(w=N),B<M&&(M=B),B>A&&(A=B)}}return c[d+0]!==v||c[d+1]!==S||c[d+2]!==M||c[d+3]!==y||c[d+4]!==w||c[d+5]!==A?(c[d+0]=v,c[d+1]=S,c[d+2]=M,c[d+3]=y,c[d+4]=w,c[d+5]=A,!0):!1}else{const p=d+8,_=a[d+6],v=p+f,S=_+f;let M=g,y=!1,w=!1;t?M||(y=t.has(v),w=t.has(S),M=!y&&!w):(y=!0,w=!0);const A=M||y,E=M||w;let T=!1;A&&(T=u(p,f,M));let L=!1;E&&(L=u(_,f,M));const C=T||L;if(C)for(let F=0;F<3;F++){const D=p+F,N=_+F,B=c[D],G=c[D+3],z=c[N],H=c[N+3];c[d+F]=B<z?B:z,c[d+F+3]=G>H?G:H}return C}}}function DE(r,t,e,n,i,s,a){Ee.setBuffer(r._roots[t]),xh(0,r,e,n,i,s,a),Ee.clearBuffer()}function xh(r,t,e,n,i,s,a){const{float32Array:o,uint16Array:c,uint32Array:l}=Ee,h=r*2;if(wn(h,c)){const d=Nn(r,l),f=kn(h,c);vE(t,e,n,d,f,i,s,a)}else{const d=Hn(r);rs(d,o,n,s,a)&&xh(d,t,e,n,i,s,a);const f=Wn(r,l);rs(f,o,n,s,a)&&xh(f,t,e,n,i,s,a)}}const IE=["x","y","z"];function NE(r,t,e,n,i,s){Ee.setBuffer(r._roots[t]);const a=_h(0,r,e,n,i,s);return Ee.clearBuffer(),a}function _h(r,t,e,n,i,s){const{float32Array:a,uint16Array:o,uint32Array:c}=Ee;let l=r*2;if(wn(l,o)){const u=Nn(r,c),d=kn(l,o);return SE(t,e,n,u,d,i,s)}else{const u=Np(r,c),d=IE[u],g=n.direction[d]>=0;let x,m;g?(x=Hn(r),m=Wn(r,c)):(x=Wn(r,c),m=Hn(r));const _=rs(x,a,n,i,s)?_h(x,t,e,n,i,s):null;if(_){const M=_.point[d];if(g?M<=a[m+u]:M>=a[m+u+3])return _}const S=rs(m,a,n,i,s)?_h(m,t,e,n,i,s):null;return _&&S?_.distance<=S.distance?_:S:_||S||null}}const Do=new rn,er=new si,nr=new si,oa=new re,gf=new vn,Io=new vn;function FE(r,t,e,n){Ee.setBuffer(r._roots[t]);const i=vh(0,r,e,n);return Ee.clearBuffer(),i}function vh(r,t,e,n,i=null){const{float32Array:s,uint16Array:a,uint32Array:o}=Ee;let c=r*2;if(i===null&&(e.boundingBox||e.computeBoundingBox(),gf.set(e.boundingBox.min,e.boundingBox.max,n),i=gf),wn(c,a)){const h=t.geometry,u=h.index,d=h.attributes.position,f=e.index,g=e.attributes.position,x=Nn(r,o),m=kn(c,a);if(oa.copy(n).invert(),e.boundsTree)return Ne(r,s,Io),Io.matrix.copy(oa),Io.needsUpdate=!0,e.boundsTree.shapecast({intersectsBounds:_=>Io.intersectsBox(_),intersectsTriangle:_=>{_.a.applyMatrix4(n),_.b.applyMatrix4(n),_.c.applyMatrix4(n),_.needsUpdate=!0;for(let v=x,S=m+x;v<S;v++)if(Xe(nr,3*t.resolveTriangleIndex(v),u,d),nr.needsUpdate=!0,_.intersectsTriangle(nr))return!0;return!1}});for(let p=x,_=m+x;p<_;p++){const v=t.resolveTriangleIndex(p);Xe(er,3*v,u,d),er.a.applyMatrix4(oa),er.b.applyMatrix4(oa),er.c.applyMatrix4(oa),er.needsUpdate=!0;for(let S=0,M=f.count;S<M;S+=3)if(Xe(nr,S,f,g),nr.needsUpdate=!0,er.intersectsTriangle(nr))return!0}}else{const h=r+8,u=o[r+6];return Ne(h,s,Do),!!(i.intersectsBox(Do)&&vh(h,t,e,n,i)||(Ne(u,s,Do),i.intersectsBox(Do)&&vh(u,t,e,n,i)))}}const No=new re,Gl=new vn,ca=new vn,BE=new R,UE=new R,zE=new R,OE=new R;function GE(r,t,e,n={},i={},s=0,a=1/0){t.boundingBox||t.computeBoundingBox(),Gl.set(t.boundingBox.min,t.boundingBox.max,e),Gl.needsUpdate=!0;const o=r.geometry,c=o.attributes.position,l=o.index,h=t.attributes.position,u=t.index,d=Xn.getPrimitive(),f=Xn.getPrimitive();let g=BE,x=UE,m=null,p=null;i&&(m=zE,p=OE);let _=1/0,v=null,S=null;return No.copy(e).invert(),ca.matrix.copy(No),r.shapecast({boundsTraverseOrder:M=>Gl.distanceToBox(M),intersectsBounds:(M,y,w)=>w<_&&w<a?(y&&(ca.min.copy(M.min),ca.max.copy(M.max),ca.needsUpdate=!0),!0):!1,intersectsRange:(M,y)=>{if(t.boundsTree){const w=t.boundsTree;return w.shapecast({boundsTraverseOrder:A=>ca.distanceToBox(A),intersectsBounds:(A,E,T)=>T<_&&T<a,intersectsRange:(A,E)=>{for(let T=A,L=A+E;T<L;T++){const C=w.resolveTriangleIndex(T);Xe(f,3*C,u,h),f.a.applyMatrix4(e),f.b.applyMatrix4(e),f.c.applyMatrix4(e),f.needsUpdate=!0;for(let F=M,D=M+y;F<D;F++){const N=r.resolveTriangleIndex(F);Xe(d,3*N,l,c),d.needsUpdate=!0;const B=d.distanceToTriangle(f,g,m);if(B<_&&(x.copy(g),p&&p.copy(m),_=B,v=F,S=T),B<s)return!0}}}})}else{const w=br(t);for(let A=0,E=w;A<E;A++){Xe(f,3*A,u,h),f.a.applyMatrix4(e),f.b.applyMatrix4(e),f.c.applyMatrix4(e),f.needsUpdate=!0;for(let T=M,L=M+y;T<L;T++){const C=r.resolveTriangleIndex(T);Xe(d,3*C,l,c),d.needsUpdate=!0;const F=d.distanceToTriangle(f,g,m);if(F<_&&(x.copy(g),p&&p.copy(m),_=F,v=T,S=A),F<s)return!0}}}}}),Xn.releasePrimitive(d),Xn.releasePrimitive(f),_===1/0?null:(n.point?n.point.copy(x):n.point=x.clone(),n.distance=_,n.faceIndex=v,i&&(i.point?i.point.copy(p):i.point=p.clone(),i.point.applyMatrix4(No),x.applyMatrix4(No),i.distance=x.sub(i.point).length(),i.faceIndex=S),n)}function VE(){return typeof SharedArrayBuffer<"u"}const Aa=new Ee.constructor,tc=new Ee.constructor,Ji=new jh(()=>new rn),ir=new rn,sr=new rn,Vl=new rn,kl=new rn;let Hl=!1;function kE(r,t,e,n){if(Hl)throw new Error("MeshBVH: Recursive calls to bvhcast not supported.");Hl=!0;const i=r._roots,s=t._roots;let a,o=0,c=0;const l=new re().copy(e).invert();for(let h=0,u=i.length;h<u;h++){Aa.setBuffer(i[h]),c=0;const d=Ji.getPrimitive();Ne(0,Aa.float32Array,d),d.applyMatrix4(l);for(let f=0,g=s.length;f<g&&(tc.setBuffer(s[f]),a=Qn(0,0,e,l,n,o,c,0,0,d),tc.clearBuffer(),c+=s[f].length,!a);f++);if(Ji.releasePrimitive(d),Aa.clearBuffer(),o+=i[h].length,a)break}return Hl=!1,a}function Qn(r,t,e,n,i,s=0,a=0,o=0,c=0,l=null,h=!1){let u,d;h?(u=tc,d=Aa):(u=Aa,d=tc);const f=u.float32Array,g=u.uint32Array,x=u.uint16Array,m=d.float32Array,p=d.uint32Array,_=d.uint16Array,v=r*2,S=t*2,M=wn(v,x),y=wn(S,_);let w=!1;if(y&&M)h?w=i(Nn(t,p),kn(t*2,_),Nn(r,g),kn(r*2,x),c,a+t,o,s+r):w=i(Nn(r,g),kn(r*2,x),Nn(t,p),kn(t*2,_),o,s+r,c,a+t);else if(y){const A=Ji.getPrimitive();Ne(t,m,A),A.applyMatrix4(e);const E=Hn(r),T=Wn(r,g);Ne(E,f,ir),Ne(T,f,sr);const L=A.intersectsBox(ir),C=A.intersectsBox(sr);w=L&&Qn(t,E,n,e,i,a,s,c,o+1,A,!h)||C&&Qn(t,T,n,e,i,a,s,c,o+1,A,!h),Ji.releasePrimitive(A)}else{const A=Hn(t),E=Wn(t,p);Ne(A,m,Vl),Ne(E,m,kl);const T=l.intersectsBox(Vl),L=l.intersectsBox(kl);if(T&&L)w=Qn(r,A,e,n,i,s,a,o,c+1,l,h)||Qn(r,E,e,n,i,s,a,o,c+1,l,h);else if(T)if(M)w=Qn(r,A,e,n,i,s,a,o,c+1,l,h);else{const C=Ji.getPrimitive();C.copy(Vl).applyMatrix4(e);const F=Hn(r),D=Wn(r,g);Ne(F,f,ir),Ne(D,f,sr);const N=C.intersectsBox(ir),B=C.intersectsBox(sr);w=N&&Qn(A,F,n,e,i,a,s,c,o+1,C,!h)||B&&Qn(A,D,n,e,i,a,s,c,o+1,C,!h),Ji.releasePrimitive(C)}else if(L)if(M)w=Qn(r,E,e,n,i,s,a,o,c+1,l,h);else{const C=Ji.getPrimitive();C.copy(kl).applyMatrix4(e);const F=Hn(r),D=Wn(r,g);Ne(F,f,ir),Ne(D,f,sr);const N=C.intersectsBox(ir),B=C.intersectsBox(sr);w=N&&Qn(E,F,n,e,i,a,s,c,o+1,C,!h)||B&&Qn(E,D,n,e,i,a,s,c,o+1,C,!h),Ji.releasePrimitive(C)}}return w}const Fo=new vn,xf=new rn,HE={strategy:Lp,maxDepth:40,maxLeafTris:10,useSharedArrayBuffer:!1,setBoundingBox:!0,onProgress:null,indirect:!1,verbose:!0,range:null};class $h{static serialize(t,e={}){e={cloneBuffers:!0,...e};const n=t.geometry,i=t._roots,s=t._indirectBuffer,a=n.getIndex();let o;return e.cloneBuffers?o={roots:i.map(c=>c.slice()),index:a?a.array.slice():null,indirectBuffer:s?s.slice():null}:o={roots:i,index:a?a.array:null,indirectBuffer:s},o}static deserialize(t,e,n={}){n={setIndex:!0,indirect:!!t.indirectBuffer,...n};const{index:i,roots:s,indirectBuffer:a}=t,o=new $h(e,{...n,[Nl]:!0});if(o._roots=s,o._indirectBuffer=a||null,n.setIndex){const c=e.getIndex();if(c===null){const l=new Be(t.index,1,!1);e.setIndex(l)}else c.array!==i&&(c.array.set(i),c.needsUpdate=!0)}return o}get indirect(){return!!this._indirectBuffer}constructor(t,e={}){if(t.isBufferGeometry){if(t.index&&t.index.isInterleavedBufferAttribute)throw new Error("MeshBVH: InterleavedBufferAttribute is not supported for the index attribute.")}else throw new Error("MeshBVH: Only BufferGeometries are supported.");if(e=Object.assign({...HE,[Nl]:!1},e),e.useSharedArrayBuffer&&!VE())throw new Error("MeshBVH: SharedArrayBuffer is not available.");this.geometry=t,this._roots=null,this._indirectBuffer=null,e[Nl]||(sE(this,e),!t.boundingBox&&e.setBoundingBox&&(t.boundingBox=this.getBoundingBox(new rn))),this.resolveTriangleIndex=e.indirect?n=>this._indirectBuffer[n]:n=>n}refit(t=null){return(this.indirect?LE:_E)(this,t)}traverse(t,e=0){const n=this._roots[e],i=new Uint32Array(n),s=new Uint16Array(n);a(0);function a(o,c=0){const l=o*2,h=s[l+15]===Sc;if(h){const u=i[o+6],d=s[l+14];t(c,h,new Float32Array(n,o*4,6),u,d)}else{const u=o+Ta/4,d=i[o+6],f=i[o+7];t(c,h,new Float32Array(n,o*4,6),f)||(a(u,c+1),a(d,c+1))}}}raycast(t,e=0,n=0,i=1/0){const s=this._roots,a=this.geometry,o=[],c=e.isMaterial,l=Array.isArray(e),h=a.groups,u=c?e.side:e,d=this.indirect?DE:yE;for(let f=0,g=s.length;f<g;f++){const x=l?e[h[f].materialIndex].side:u,m=o.length;if(d(this,f,x,t,o,n,i),l){const p=h[f].materialIndex;for(let _=m,v=o.length;_<v;_++)o[_].face.materialIndex=p}}return o}raycastFirst(t,e=0,n=0,i=1/0){const s=this._roots,a=this.geometry,o=e.isMaterial,c=Array.isArray(e);let l=null;const h=a.groups,u=o?e.side:e,d=this.indirect?NE:wE;for(let f=0,g=s.length;f<g;f++){const x=c?e[h[f].materialIndex].side:u,m=d(this,f,x,t,n,i);m!=null&&(l==null||m.distance<l.distance)&&(l=m,c&&(m.face.materialIndex=h[f].materialIndex))}return l}intersectsGeometry(t,e){let n=!1;const i=this._roots,s=this.indirect?FE:TE;for(let a=0,o=i.length;a<o&&(n=s(this,a,t,e),!n);a++);return n}shapecast(t){const e=Xn.getPrimitive(),n=this.indirect?ME:xE;let{boundsTraverseOrder:i,intersectsBounds:s,intersectsRange:a,intersectsTriangle:o}=t;if(a&&o){const u=a;a=(d,f,g,x,m)=>u(d,f,g,x,m)?!0:n(d,f,this,o,g,x,e)}else a||(o?a=(u,d,f,g)=>n(u,d,this,o,f,g,e):a=(u,d,f)=>f);let c=!1,l=0;const h=this._roots;for(let u=0,d=h.length;u<d;u++){const f=h[u];if(c=hE(this,u,s,a,i,l),c)break;l+=f.byteLength}return Xn.releasePrimitive(e),c}bvhcast(t,e,n){let{intersectsRanges:i,intersectsTriangles:s}=n;const a=Xn.getPrimitive(),o=this.geometry.index,c=this.geometry.attributes.position,l=this.indirect?g=>{const x=this.resolveTriangleIndex(g);Xe(a,x*3,o,c)}:g=>{Xe(a,g*3,o,c)},h=Xn.getPrimitive(),u=t.geometry.index,d=t.geometry.attributes.position,f=t.indirect?g=>{const x=t.resolveTriangleIndex(g);Xe(h,x*3,u,d)}:g=>{Xe(h,g*3,u,d)};if(s){const g=(x,m,p,_,v,S,M,y)=>{for(let w=p,A=p+_;w<A;w++){f(w),h.a.applyMatrix4(e),h.b.applyMatrix4(e),h.c.applyMatrix4(e),h.needsUpdate=!0;for(let E=x,T=x+m;E<T;E++)if(l(E),a.needsUpdate=!0,s(a,h,E,w,v,S,M,y))return!0}return!1};if(i){const x=i;i=function(m,p,_,v,S,M,y,w){return x(m,p,_,v,S,M,y,w)?!0:g(m,p,_,v,S,M,y,w)}}else i=g}return kE(this,t,e,i)}intersectsBox(t,e){return Fo.set(t.min,t.max,e),Fo.needsUpdate=!0,this.shapecast({intersectsBounds:n=>Fo.intersectsBox(n),intersectsTriangle:n=>Fo.intersectsTriangle(n)})}intersectsSphere(t){return this.shapecast({intersectsBounds:e=>t.intersectsBox(e),intersectsTriangle:e=>e.intersectsSphere(t)})}closestPointToGeometry(t,e,n={},i={},s=0,a=1/0){return(this.indirect?GE:PE)(this,t,e,n,i,s,a)}closestPointToPoint(t,e={},n=0,i=1/0){return uE(this,t,e,n,i)}getBoundingBox(t){return t.makeEmpty(),this._roots.forEach(n=>{Ne(0,new Float32Array(n),xf),t.union(xf)}),t}}const WE=new URL("/assets/Aorta_plain-_gXpsVDF.stl",import.meta.url).href,XE=new URL("/assets/Aorta_plain.collision-DFUYJYB3.bin",import.meta.url).href,YE=1.3,qE=40,ZE=1;function jE(r){const t=[];for(const i of r?.segments||[])i.isSheath||t.push(i.start.y,i.end.y);const e=Math.max(...t,0)+15,n=Math.min(...t,-420)-15;return{center:new R(r?.branchPoint?.x||0,(e+n)*.5+qE,r?.branchPoint?.z||0),length:Math.max(300,e-n)}}function $E(r,t){r.computeBoundingBox();const e=r.boundingBox.clone(),n=e.getSize(new R),i=e.getCenter(new R),s=jE(t),a=s.length*YE/Math.max(1e-6,n.z);return r.translate(-i.x,-i.y,-i.z),r.rotateX(-Math.PI/2),r.scale(a,a,a),r.translate(s.center.x,s.center.y,s.center.z),r.computeBoundingBox(),{version:ZE,rotationX:-Math.PI/2,scale:a,sourceCenter:i.toArray(),sourceSize:n.toArray(),targetCenter:s.center.toArray(),targetLength:s.length}}function KE(r){return globalThis.crypto.subtle.digest("SHA-256",r).then(t=>[...new Uint8Array(t)].map(e=>e.toString(16).padStart(2,"0")).join(""))}async function _f(r){const t=await fetch(r);if(!t.ok)throw new Error(`Failed to load ${r}: ${t.status} ${t.statusText}`);return t.arrayBuffer()}function JE(r,t,e){if(r.metadata.source?.stlSha256!==t)throw new Error("Aorta collision asset does not match Aorta_plain.stl; run npm run collision:build");const n=r.metadata.transform;if(n?.version!==e.version||Math.abs((n?.scale??1/0)-e.scale)>1e-7||Math.abs((n?.targetLength??1/0)-e.targetLength)>1e-6)throw new Error("Aorta collision asset transform is stale; run npm run collision:build")}function QE(r){const t=r.arrays.centerlineSegments,e=r.arrays.centerlineEdges,n=r.metadata.centerline.stride,i=[];for(let s=0;s<t.length/n;s++){const a=s*n,o=new R(t[a],t[a+1],t[a+2]),c=new R(t[a+3],t[a+4],t[a+5]),l=c.clone().sub(o),h=l.length();h>1e-8?l.multiplyScalar(1/h):l.set(0,1,0),i.push({id:s,start:o,end:c,axis:l,length:h,radiusStart:t[a+6],radiusEnd:t[a+7],safeRadius:t[a+8],nodeStartId:e[s*2],nodeEndId:e[s*2+1],source:"medial-slice-teasar",aabb:null})}return{type:"centerline-capsule-broadphase",source:"medial-slice-teasar",diagnostics:r.metadata.centerline.diagnostics,inflation:r.metadata.sdf.band,cellSize:r.metadata.broadPhase.cellSize,segments:i,contactField:r}}function t1(r,t=12e3){const e=r.arrays;if(!e.lumenSliceYs?.length)return new Float32Array;const n=e.lumenPoints instanceof Int16Array?r.metadata.lumen?.pointQuantization||.02:1,i=e.lumenAxisBases||new Float32Array([1,0,0,0,1,0,0,0,1]),s=e.lumenAxisSliceOffsets||new Uint32Array([0,e.lumenSliceYs.length]),a=e.lumenPoints.length/2,o=Math.max(1,Math.ceil(a/t)),c=[];let l=0;for(let h=0;h<s.length-1;h++){const u=h*9;for(let d=s[h];d<s[h+1];d++){const f=e.lumenSliceYs[d],g=e.lumenSliceContourOffsets[d],x=e.lumenSliceContourOffsets[d+1];for(let m=g;m<x;m++){const p=e.lumenContourPointOffsets[m],_=e.lumenContourPointOffsets[m+1];for(let v=p;v<_;v++,l++){if(l%o!==0)continue;const S=v+1<_?v+1:p,M=e.lumenPoints[v*2]*n,y=e.lumenPoints[v*2+1]*n,w=e.lumenPoints[S*2]*n,A=e.lumenPoints[S*2+1]*n;c.push(i[u]*M+i[u+3]*f+i[u+6]*y,i[u+1]*M+i[u+4]*f+i[u+7]*y,i[u+2]*M+i[u+5]*f+i[u+8]*y,i[u]*w+i[u+3]*f+i[u+6]*A,i[u+1]*w+i[u+4]*f+i[u+7]*A,i[u+2]*w+i[u+5]*f+i[u+8]*A)}}}}return new Float32Array(c)}function e1(r,t,e){const n=r.metadata;return{geometry:t,interiorSamples:[],lumenSlices:[],lumenField:r.packedLumenField,boundaryDebugSegments:new Float32Array,lumenContourDebugSegments:t1(r),centerlineSliceDebugSegments:null,centerlineExtraction:n.centerline.diagnostics,lumenCastGeometry:null,lumenCast:null,collisionAsset:n,diagnostics:{boundingBox:t.boundingBox.clone(),boundaryEdgeCount:0,degenerateTriangleCount:0,edgeCount:0,interiorSampleCount:0,lumenSliceCount:n.lumen.sliceCount,nonManifoldEdgeCount:0,size:t.boundingBox.getSize(new R),transform:e,triangleCount:n.source.triangleCount,vertexCount:t.attributes.position.count,source:"precompiled-collision-asset"}}}function n1(r,{onLoaded:t,onError:e}={}){const n=new Le;n.visible=!1;const i=new Fe({color:5213695,transparent:!0,opacity:.34,depthWrite:!1,side:2});return Promise.all([_f(WE),_f(XE)]).then(async([s,a])=>{const[o]=await Promise.all([KE(s)]),c=new Gy().parse(s),l=$E(c,r),h=dp(a);JE(h,o,l),c.computeVertexNormals(),c.computeBoundingSphere(),c.boundsTree=new $h(c);const u=new vM(h,{fallbackGeometry:c,bvhValidationDistance:.02,capsuleBvhValidation:-.1}),d=QE(u),f=e1(u,c,l),g=new jt(c,i);g.renderOrder=0,n.add(g),n.visible=!0;const x={geometry:c,contactField:u,meshCollider:i1(u),centerlineBroadPhase:d,clearance:.6,guidewireClearance:Mr,guidewireSegmentClearance:.12,guidewireCollisionPasses:3,guidewireSegmentSamples:[.2,.4,.6,.8],openOutletY:c.boundingBox.max.y-1,preprocessing:f};typeof t=="function"&&t({group:n,mesh:g,geometry:c,collision:x,preprocessing:f,scale:l.scale})}).catch(s=>{console.warn("Failed to load aorta STL model",s),typeof e=="function"&&e(s)}),{group:n,material:i}}function i1(r){const t=ei(),e=ei(),n=(o,c,l,h)=>(typeof o?.set=="function"?o.set(c,l,h):(o.x=c,o.y=l,o.z=h),o),i=(o,c)=>{const l=c||{target:new R,closestPoint:new R,inward:new R,normal:new R};return l.inside=o.inside,l.violation=o.violation,l.distance=Math.max(0,o.signedDistance),l.signedDistance=o.signedDistance,l.signedGap=o.signedGap,l.penetration=o.penetration,l.branchId=o.branchId,l.source=o.source,l.target=l.target||{},l.closestPoint=l.closestPoint||{},l.inward=l.inward||{},l.normal=l.normal||{},n(l.target,o.target.x,o.target.y,o.target.z),n(l.closestPoint,o.closestPoint.x,o.closestPoint.y,o.closestPoint.z),n(l.inward,o.inward.x,o.inward.y,o.inward.z),n(l.normal,-o.inward.x,-o.inward.y,-o.inward.z),l},s=(o,c=0,l=null)=>i(r.querySphere(o,c,t),l),a=(o,c,l=0)=>{const h=r.sweepSphere(o,c,l,e);return!h.violation&&h.timeOfImpact>=1?null:{penetration:h.penetration,point:new R(h.point.x,h.point.y,h.point.z),target:new R(h.target.x,h.target.y,h.target.z),normal:new R(-h.inward.x,-h.inward.y,-h.inward.z),inward:new R(h.inward.x,h.inward.y,h.inward.z),t:h.timeOfImpact,branchId:h.branchId}};return{geometry:r.fallbackGeometry,lumenField:r.packedLumenField,broadPhase:r,contactField:r,containsPoint:o=>!s(o,0,t).violation,pointContact:s,crossingContact:a,clearCache:()=>{}}}const s1=1.5,r1=900,a1=16773994,o1=16777215,c1=16732120,l1=1.05,h1=1.75,u1=3778303,d1=9427199;function f1(r,t=s1){return Number.isFinite(r)&&r>0?r:t}function p1(r,t){r.position.copy(t.start).lerp(t.end,.5),r.quaternion.setFromUnitVectors(new R(0,1,0),t.axis)}function m1(r,t,e,n){const i=new jt(new Ps(t.radiusEnd,t.radiusStart,t.length,18,1,!0),e);p1(i,t),i.renderOrder=4.5,i.userData.debugLayer="capsules",r.add(i);const s=new jt(new ii(t.radiusStart,16,8),n);s.position.copy(t.start),s.renderOrder=4.4,s.userData.debugLayer="capsules",r.add(s);const a=new jt(new ii(t.radiusEnd,16,8),n);a.position.copy(t.end),a.renderOrder=4.4,a.userData.debugLayer="capsules",r.add(a)}function Bo(r){return[Math.round(r.x*8),Math.round(r.y*8),Math.round(r.z*8)].join(",")}function Wl(r,t,e,n){let i=r.get(t);i||(i={point:new R,radius:0,degree:0,weight:0},r.set(t,i)),i.point.add(e),i.radius=Math.max(i.radius,f1(n)),i.degree++,i.weight++}function g1(r){const t=new Map;for(const e of r){if(e.nodeStartId!==void 0&&e.nodeStartId===e.nodeEndId){const s=e.start.clone().lerp(e.end,.5);Wl(t,`node:${e.nodeStartId}`,s,Math.max(e.radiusStart,e.radiusEnd));continue}const n=e.nodeStartId!==void 0?`node:${e.nodeStartId}:${Bo(e.start)}`:`point:${Bo(e.start)}`,i=e.nodeEndId!==void 0?`node:${e.nodeEndId}:${Bo(e.end)}`:`point:${Bo(e.end)}`;Wl(t,n,e.start,e.radiusStart),Wl(t,i,e.end,e.radiusEnd)}return[...t.values()].map(e=>({...e,point:e.point.multiplyScalar(1/Math.max(1,e.weight))}))}function vf(r,{radius:t,color:e,opacity:n,renderOrder:i}){if(!r.length)return null;const s=new gc(new ii(t,10,6),new Fe({color:e,transparent:!0,opacity:n,depthTest:!1,depthWrite:!1,toneMapped:!1}),r.length),a=new re,o=new R;for(let c=0;c<r.length;c++){const l=Math.max(.72,Math.min(1.65,r[c].radius*.16));o.setScalar(l),a.compose(r[c].point,new ni,o),s.setMatrixAt(c,a)}return s.frustumCulled=!1,s.instanceMatrix.needsUpdate=!0,s.renderOrder=i,s.userData.debugLayer="centerline",s}function x1(r,{maxCapsules:t=r1}={}){const e=new Le;if(!r?.segments?.length)return e;const n=r.segments,i=new Float32Array(n.length*6);for(let v=0;v<n.length;v++){const S=n[v];i[v*6]=S.start.x,i[v*6+1]=S.start.y,i[v*6+2]=S.start.z,i[v*6+3]=S.end.x,i[v*6+4]=S.end.y,i[v*6+5]=S.end.z}const s=new me;s.setAttribute("position",new Be(i,3));const a=new ss(s,new gi({color:a1,transparent:!0,opacity:.96,depthTest:!1,depthWrite:!1,toneMapped:!1}));a.frustumCulled=!1,a.renderOrder=9.7,a.userData.debugLayer="centerline",e.add(a);const o=g1(n),c=o.filter(v=>v.degree===2),l=o.filter(v=>v.degree!==2),h=vf(c,{radius:l1,color:o1,opacity:.92,renderOrder:9.78});h&&e.add(h);const u=vf(l,{radius:h1,color:c1,opacity:.98,renderOrder:9.82});u&&e.add(u);const d=Number.isFinite(t)&&t>0?Math.max(1,Math.ceil(r.segments.length/t)):1,f=r.segments.filter((v,S)=>S%d===0),g=new Fe({color:u1,transparent:!0,opacity:.105,depthTest:!0,depthWrite:!1,side:2,toneMapped:!1}),x=g.clone();x.opacity=.075;for(const v of f)m1(e,v,g,x);const m=new Float32Array(f.length*6);for(let v=0;v<f.length;v++){const S=f[v];m[v*6]=S.start.x,m[v*6+1]=S.start.y,m[v*6+2]=S.start.z,m[v*6+3]=S.end.x,m[v*6+4]=S.end.y,m[v*6+5]=S.end.z}const p=new me;p.setAttribute("position",new Be(m,3));const _=new ss(p,new gi({color:d1,transparent:!0,opacity:.38,depthTest:!1,depthWrite:!1,toneMapped:!1}));return _.frustumCulled=!1,_.renderOrder=9.55,_.userData.debugLayer="capsules",e.add(_),e.userData.broadPhase=r,e.userData.displayedSegmentCount=f.length,e.userData.centerlineNodeCount=o.length,e.userData.centerlineBranchNodeCount=l.length,e}const _1=`
// Fullscreen quad vertex shader.
// Passes through the quad UVs and positions to draw a screen-aligned quad.
varying vec2 vUv;
void main() {
    vUv = uv;
    gl_Position = vec4(position.xy, 0.0, 1.0);
}
`,v1=`
// Persistence blend (accumulation) shader.
// Combines the newly rendered frame (currentFrame) with the previous
// accumulated frame (previousFrame) using exponential decay. This
// produces a fluoroscopy-like persistence trail over time.
uniform sampler2D currentFrame;
uniform sampler2D previousFrame;
uniform float decay;
varying vec2 vUv;
void main() {
    vec4 prev = texture2D(previousFrame, vUv);
    vec4 curr = texture2D(currentFrame, vUv);
    gl_FragColor = curr + prev * decay;
}
`,S1=`
// Fullscreen quad vertex shader for thickness pass.
// Renders a screen-aligned quad; vUv is used to sample depth textures.
varying vec2 vUv;
void main() {
    vUv = uv;
    gl_Position = vec4(position.xy, 0.0, 1.0);
}
`,M1=`
// Thickness estimation shader.
// Front/back textures store linear camera-space depth normalized to camera far.
// Their difference is an approximation of X-ray path length through bone for
// the current C-arm projection.
uniform sampler2D frontDepth;
uniform sampler2D backDepth;
varying vec2 vUv;
void main() {
    float front = texture2D(frontDepth, vUv).r;
    float back = texture2D(backDepth, vUv).r;
    float thick = max(back - front, 0.0);

    gl_FragColor = vec4(vec3(clamp(thick, 0.0, 1.0)), 1.0);
}
`,y1=`
// Fullscreen quad vertex shader for the final display pass.
// Simply forwards UVs to the fragment shader.
varying vec2 vUv;
void main() {
    vUv = uv;
    gl_Position = vec4(position.xy, 0.0, 1.0);
}
`,E1=`
// Final display shader.
// - In fluoroscopy mode: composes anatomy thickness, iodine contrast, and
//   metallic devices as radiographic attenuation, then applies detector-style
//   windowing, edge enhancement, field falloff, and dose-dependent noise.
// - In wireframe mode: just visualizes the scene with edge-enhanced alpha.
uniform sampler2D uTexture;
uniform sampler2D contrastTexture;
uniform sampler2D thicknessTexture;
uniform sampler2D metalTexture;
uniform sampler2D catheterTexture;
uniform sampler2D sheathTexture;
uniform sampler2D boneTexture;
uniform vec3 gray;
uniform bool fluoroscopy;
uniform float time;
uniform float noiseLevel;
uniform float imageBrightness;
uniform float imageContrast;
uniform bool autoExposureEnabled;
uniform float autoExposureLevel;
uniform float pulseRate;
uniform float scatterStrength;
uniform float collimation;
uniform float boneOpacity;
uniform vec2 resolution;
uniform float edgeStrength;
uniform float contrastOpacity;
uniform float contrastGain;
varying vec2 vUv;

float saturate(float value) {
    return clamp(value, 0.0, 1.0);
}

float max3(vec3 value) {
    return max(max(value.r, value.g), value.b);
}

// Hash-based noise. Animated samples mimic quantum mottle; stable samples are
// reused for detector fixed-pattern noise.
float random(vec2 st) {
    return fract(sin(dot(st.xy, vec2(12.9898, 78.233))) * 43758.5453123);
}

float animatedNoise(vec2 st, float phase) {
    return random(st + vec2(phase * 17.37, phase * 5.91));
}

float sampleSignal(sampler2D source, vec2 uv) {
    return max3(texture2D(source, uv).rgb);
}

float contrastAt(vec2 uv) {
    float signal = sampleSignal(contrastTexture, uv);
    return saturate(1.0 - exp(-signal * max(0.0, contrastGain) * 1.45));
}

float metalAt(vec2 uv) {
    float center = sampleSignal(metalTexture, uv);
    return pow(saturate(center), 1.18);
}

float catheterAt(vec2 uv) {
    float signal = sampleSignal(catheterTexture, uv);
    return smoothstep(0.025, 0.48, signal);
}

float sheathAt(vec2 uv) {
    float signal = sampleSignal(sheathTexture, uv);
    return smoothstep(0.03, 0.32, signal);
}

vec4 boneProjectionSampleAt(vec2 uv) {
    return texture2D(boneTexture, uv);
}

float rawBoneThicknessFromSample(vec4 projectionSample) {
    return max(projectionSample.g - projectionSample.r, 0.0) * 10.2;
}

vec4 boneTransportAt(vec2 uv) {
    vec2 texel = 1.0 / resolution;
    vec4 center = boneProjectionSampleAt(uv);
    vec4 axial = (
        boneProjectionSampleAt(uv + texel * vec2(1.4, 0.0)) +
        boneProjectionSampleAt(uv + texel * vec2(-1.4, 0.0)) +
        boneProjectionSampleAt(uv + texel * vec2(0.0, 1.4)) +
        boneProjectionSampleAt(uv + texel * vec2(0.0, -1.4))
    ) * 0.04;
    vec4 diagonal = (
        boneProjectionSampleAt(uv + texel * vec2(1.7, 1.7)) +
        boneProjectionSampleAt(uv + texel * vec2(-1.7, 1.7)) +
        boneProjectionSampleAt(uv + texel * vec2(1.7, -1.7)) +
        boneProjectionSampleAt(uv + texel * vec2(-1.7, -1.7))
    ) * 0.015;
    return center * 0.78 + axial + diagonal;
}

float thicknessPathAt(vec2 uv) {
    float thickness = texture2D(thicknessTexture, uv).r;
    return saturate(thickness * 6.2);
}

vec4 boneLayerPathsAt(vec2 uv) {
    vec4 transport = boneTransportAt(uv);
    float rawThickness = max(transport.g - transport.r, 0.0) * 10.2;
    float totalPath = 1.0 - exp(-rawThickness * 0.5);
    float corticalPath = min(transport.b * 11.2, totalPath * 0.92);
    float corticalShare = saturate(corticalPath / max(totalPath, 0.001));
    float cancellousPath = max(totalPath - corticalPath * 0.8, 0.0) * mix(0.3, 0.54, corticalShare);
    float cancellousTexture = mix(0.72, 0.98, saturate(transport.a * 2.05));
    return vec4(totalPath, corticalPath, cancellousPath, cancellousTexture);
}

float bonePathAt(vec2 uv) {
    return saturate(boneLayerPathsAt(uv).x);
}

float corticalEdgeAt(vec2 uv) {
    vec2 texel = 1.0 / resolution;
    float thicknessL = thicknessPathAt(uv + texel * vec2(-1.0, 0.0));
    float thicknessR = thicknessPathAt(uv + texel * vec2(1.0, 0.0));
    float thicknessT = thicknessPathAt(uv + texel * vec2(0.0, -1.0));
    float thicknessB = thicknessPathAt(uv + texel * vec2(0.0, 1.0));
    float thicknessEdge = length(vec2(thicknessR - thicknessL, thicknessB - thicknessT));

    vec4 sampleL = boneProjectionSampleAt(uv + texel * vec2(-1.0, 0.0));
    vec4 sampleR = boneProjectionSampleAt(uv + texel * vec2(1.0, 0.0));
    vec4 sampleT = boneProjectionSampleAt(uv + texel * vec2(0.0, -1.0));
    vec4 sampleB = boneProjectionSampleAt(uv + texel * vec2(0.0, 1.0));
    float pathEdge = length(vec2(
        rawBoneThicknessFromSample(sampleR) - rawBoneThicknessFromSample(sampleL),
        rawBoneThicknessFromSample(sampleB) - rawBoneThicknessFromSample(sampleT)
    ));
    float cortexEdge = length(vec2(sampleR.b - sampleL.b, sampleB.b - sampleT.b));

    float depthEdge = smoothstep(0.02, 0.2, thicknessEdge);
    float transportEdge = smoothstep(0.006, 0.095, pathEdge);
    float corticalShellEdge = smoothstep(0.0025, 0.045, cortexEdge);
    return saturate(max(depthEdge * 0.34, max(transportEdge * 0.38, corticalShellEdge * 0.42)));
}

float attenuationAt(vec2 uv, vec4 bonePaths, float corticalEdge) {
    float boneVisibility = pow(saturate(boneOpacity), 0.55);
    float corticalAbsorption = pow(saturate(bonePaths.y * 1.6), 0.96) * 0.84;
    float edgeAbsorption = corticalEdge * 0.16;
    float cancellousAbsorption = pow(saturate(bonePaths.z), 0.82) * bonePaths.w * 0.34;
    float layeredAbsorption = pow(saturate(bonePaths.x), 0.72) * 0.66;
    float softBoneAbsorption = smoothstep(0.01, 0.72, bonePaths.x) * 0.48;
    float rawBoneSignal = corticalAbsorption + edgeAbsorption + cancellousAbsorption + layeredAbsorption + softBoneAbsorption;
    float boneSignal = 1.0 - exp(-rawBoneSignal * 1.08);
    float bone = boneSignal * 1.58 * boneVisibility;
    float iodine = contrastAt(uv) * saturate(contrastOpacity) * 3.25;
    float metal = metalAt(uv) * 5.25;
    float catheter = catheterAt(uv) * 0.28;
    float sheath = sheathAt(uv) * 0.42;

    // The accumulated visible frame creates detector persistence across the
    // full fluoroscopy image while current attenuation still leads the frame.
    float temporalTrace = smoothstep(0.025, 0.72, sampleSignal(uTexture, uv)) * 0.46;
    return max(0.0, bone + iodine + metal + catheter + sheath + temporalTrace);
}

float vignetteField(vec2 uv) {
    vec2 centered = uv * 2.0 - 1.0;
    centered.x *= resolution.x / max(1.0, resolution.y);
    float radius = length(centered);
    return 1.0 - 0.22 * smoothstep(0.28, 1.35, radius);
}

float patientBodyField(vec2 uv) {
    vec2 centered = uv * 2.0 - 1.0;
    centered.x *= resolution.x / max(1.0, resolution.y);
    float lowerBody = 1.0 - smoothstep(0.42, 1.18, length(centered * vec2(0.72, 1.05)));
    float trunk = 1.0 - smoothstep(0.35, 1.08, length((centered - vec2(0.0, -0.18)) * vec2(0.62, 1.35)));
    return saturate(max(lowerBody, trunk * 0.72));
}

float scatterFieldAt(vec2 uv, float attenuation, float bonePath) {
    float tissuePath = patientBodyField(uv);
    float projectedPath = saturate(thicknessPathAt(uv) * 0.55 + bonePath * 0.26 + tissuePath * 0.22);
    return saturate(scatterStrength * (projectedPath * 0.72 + attenuation * 0.08));
}

float collimatorMask(vec2 uv) {
    vec2 centered = uv * 2.0 - 1.0;
    float aspect = resolution.x / max(1.0, resolution.y);
    vec2 squareCoord = centered;
    if (aspect >= 1.0) {
        squareCoord.x *= aspect;
    } else {
        squareCoord.y /= max(0.001, aspect);
    }
    float crop = saturate(collimation);
    float halfSize = 1.0 - crop * 1.35;
    float softness = 0.022 + crop * 0.055;
    float maskX = 1.0 - smoothstep(halfSize, halfSize + softness, abs(squareCoord.x));
    float maskY = 1.0 - smoothstep(halfSize, halfSize + softness, abs(squareCoord.y));
    return saturate(maskX * maskY);
}

// Simple Sobel-like edge factor based on alpha channel of uTexture.
// Used to enhance edges in the displayed result.
float edgeFactor(vec2 uv) {
    vec2 texel = 1.0 / resolution;
    float tl = texture2D(uTexture, uv + texel * vec2(-1.0, -1.0)).a;
    float t  = texture2D(uTexture, uv + texel * vec2(0.0, -1.0)).a;
    float tr = texture2D(uTexture, uv + texel * vec2(1.0, -1.0)).a;
    float l  = texture2D(uTexture, uv + texel * vec2(-1.0, 0.0)).a;
    float r  = texture2D(uTexture, uv + texel * vec2(1.0, 0.0)).a;
    float bl = texture2D(uTexture, uv + texel * vec2(-1.0, 1.0)).a;
    float b  = texture2D(uTexture, uv + texel * vec2(0.0, 1.0)).a;
    float br = texture2D(uTexture, uv + texel * vec2(1.0, 1.0)).a;
    float gx = -tl - 2.0*l - bl + tr + 2.0*r + br;
    float gy = -tl - 2.0*t - tr + bl + 2.0*b + br;
    return length(vec2(gx, gy));
}
void main() {
    vec4 tex = texture2D(uTexture, vUv);
    if (fluoroscopy) {
        vec4 centerBonePaths = boneLayerPathsAt(vUv);
        float centerCorticalEdge = corticalEdgeAt(vUv);
        float centerAttenuation = attenuationAt(vUv, centerBonePaths, centerCorticalEdge);
        float localScatter = scatterFieldAt(vUv, centerAttenuation, centerBonePaths.x);
        float exposureLift = autoExposureEnabled ? autoExposureLevel : 0.0;

        // C-arm images are usually edge-enhanced after acquisition. Sharpen
        // attenuation before transmission. Screen-space derivatives preserve
        // local radiopaque borders without four full neighboring attenuation
        // evaluations per detector pixel.
        float scatterSoftenedEdge = mix(0.34, 0.16, localScatter);
        float sharpenedAttenuation = max(
            0.0,
            centerAttenuation + fwidth(centerAttenuation) * edgeStrength * scatterSoftenedEdge
        );

        float transmission = exp(-sharpenedAttenuation);
        float scatterFog = saturate(centerAttenuation * 0.025 + localScatter * 0.24);
        transmission = mix(transmission, 0.55 + exposureLift * 0.12, scatterFog);

        // Detector window/level with a soft shoulder. This keeps the air field
        // from becoming pure white and gives dense contrast a real black floor.
        float luma = pow(saturate(transmission), 0.88);
        luma = smoothstep(0.025, 0.975, luma);
        luma = mix(0.045, 0.72, luma);

        float field = vignetteField(vUv);
        float fixedPattern = (random(floor(vUv * resolution / 7.0)) - 0.5) * 0.012;
        float columnPattern = (random(vec2(floor(vUv.x * resolution.x / 3.0), 19.0)) - 0.5) * 0.004;
        float gridPattern =
            sin(vUv.x * resolution.x * 0.86) * 0.0008 +
            sin(vUv.y * resolution.y * 0.42) * 0.0006;
        float doseNoiseScale = sqrt(30.0 / clamp(pulseRate, 7.5, 30.0));
        float pulseIndex = floor(time * max(1.0, pulseRate));
        float pulseJitter = (random(vec2(pulseIndex, 37.0)) - 0.5) * 0.003 * doseNoiseScale;
        float stableMottle = random(floor(vUv * resolution / 2.0)) - 0.5;
        float animatedMottle = animatedNoise(vUv * resolution, pulseIndex * 0.73) - 0.5;
        float mottle = mix(stableMottle, animatedMottle, 0.32)
            * noiseLevel
            * doseNoiseScale
            * (0.08 + 0.18 * sqrt(max(luma, 0.0)));

        luma = saturate(luma * field + fixedPattern + columnPattern + gridPattern + mottle + pulseJitter);
        luma = mix(luma, 0.50 + (luma - 0.50) * 0.68, localScatter * 0.22);
        luma = saturate(luma + exposureLift);
        luma = saturate((luma - 0.5) * max(0.0, imageContrast) + 0.5 + imageBrightness);
        luma = mix(0.018, luma, collimatorMask(vUv));

        // Phosphor/detector response is slightly warm-neutral, not mathematically
        // flat grayscale. Keep it subtle so it still reads as fluoroscopy.
        vec3 detectorTint = vec3(0.992, 0.992, 0.988);
        gl_FragColor = vec4(gray * detectorTint * luma, 1.0);
    } else {
        // Debug mode: keep original color, use edge to boost alpha.
        float edge = edgeFactor(vUv) * edgeStrength;
        float alpha = clamp(tex.a + edge, 0.0, 1.0);
        gl_FragColor = vec4(tex.rgb, alpha);
    }
}
`,ec=600*1e3,yc=72*1e3;function w1(){return{guidewireAdvance:0,catheterAdvance:0,catheterRotation:0,catheterType:"pigtail"}}function Up(r){const t=Math.max(0,r);return Math.floor(t/yc)%2===0?"pigtail":"berenstein"}function Sf(r,t){const e=Math.max(0,r),n=e%yc;return t.guidewireAdvance=0,t.catheterAdvance=0,t.catheterRotation=0,t.catheterType=Up(e),n<15e3?t.guidewireAdvance=1:n<25e3?t.catheterAdvance=1:n<35e3?t.catheterRotation=Math.floor((n-25e3)/2500)%2===0?1:-1:n<52e3?(t.catheterAdvance=-1,t.catheterRotation=Math.floor((n-35e3)/2500)%2===0?-1:1):n<67e3&&(t.guidewireAdvance=-1),t}const Kh=2752468,T1=5213695,A1=6946702,C1=16751421,R1=11009884,b1=16732120,P1=16765514,L1=16724821,D1=16733695,Cs=420,zp=1/10,I1=1.85,N1=12,F1=3,Op=1/30,Gp=1/30,B1=new URLSearchParams(window.location.search).get("physics"),Tn=B1==="legacy"?"legacy":"xpbd-contact-v1",Vp=.1,kp=1e3,la=document.getElementById("loadingScreen"),Mf=document.getElementById("loadingMessage"),ns=new Set(["aorta","skeleton","firstFrame"]);let yf=!1,Ca=null;function Fa(r){Mf&&(Mf.textContent=r)}function Hp(){return!ns.has("aorta")&&!ns.has("skeleton")}function U1(){yf||!la||(yf=!0,Fa("Ready"),la.classList.add("is-hidden"),la.addEventListener("transitionend",()=>la.remove(),{once:!0}),setTimeout(()=>la.remove(),900))}function yr(r,t){ns.has(r)&&(ns.delete(r),r==="firstFrame"&&Ca&&(clearTimeout(Ca),Ca=null),t&&Fa(t),z1(),ns.size===0&&U1())}function Wp(r){yr(r,"Loading fallback view")}function z1(){!Hp()||!ns.has("firstFrame")||Ca||(Fa("Rendering first frame"),requestAnimationFrame(()=>yr("firstFrame","Ready")),Ca=setTimeout(()=>yr("firstFrame","Ready"),1800))}function Ef(){Hp()&&yr("firstFrame","Ready")}Fa("Preparing renderer");const O1=document.getElementById("sim"),qt=new Oh({canvas:O1,antialias:!0});qt.setSize(window.innerWidth,window.innerHeight);const nc=.85,G1=()=>Math.max(1,Math.round(window.innerWidth*nc)),V1=()=>Math.max(1,Math.round(window.innerHeight*nc)),Yn=G1(),qn=V1(),k1=qt.capabilities.isWebGL2?4:0,Ec={samples:k1},oe=new Rr;oe.background=new Qt(0);const ha=new Rr,Sh=new sn(Yn,qn,Ec),ic=new sn(Yn,qn),sc=new sn(Yn,qn,Ec),rc=new sn(Yn,qn,Ec),ac=new sn(Yn,qn,Ec),wc=new sn(Yn,qn,{type:1016}),Xp=new sn(Yn,qn),Yp=new sn(Yn,qn),oc=new sn(Yn,qn),cc=new sn(Yn,qn),Tc=new sn(Yn,qn);let Yo=Xp,Uo=Yp;const wf=new Float64Array(16),Tf=new Float64Array(16);let lc=!1;const qo=new Uh(-1,1,1,-1,0,1),Jh=new pc(2,2),hc=new Bn({uniforms:{currentFrame:{value:null},previousFrame:{value:null},decay:{value:.95}},vertexShader:_1,fragmentShader:v1}),H1=new jt(Jh,hc),qp=new Rr;qp.add(H1);function Zp(r){return new Bn({side:r,depthTest:!0,depthWrite:!0,uniforms:{cameraNear:{value:Vp},cameraFar:{value:kp}},vertexShader:`
            varying float vViewDepth;
            void main() {
                vec4 mvPosition = modelViewMatrix * vec4(position, 1.0);
                vViewDepth = -mvPosition.z;
                gl_Position = projectionMatrix * mvPosition;
            }
        `,fragmentShader:`
            uniform float cameraNear;
            uniform float cameraFar;
            varying float vViewDepth;
            void main() {
                float depth = clamp((vViewDepth - cameraNear) / max(1.0, cameraFar - cameraNear), 0.0, 1.0);
                gl_FragColor = vec4(vec3(depth), 1.0);
            }
        `})}const W1=Zp(0),X1=Zp(1),Mh=new Bn({uniforms:{frontDepth:{value:oc.texture},backDepth:{value:cc.texture}},vertexShader:S1,fragmentShader:M1}),Y1=new jt(Jh,Mh),jp=new Rr;jp.add(Y1);const q1=new Bn({transparent:!0,blending:5,blendEquation:100,blendSrc:201,blendDst:201,blendEquationAlpha:100,blendSrcAlpha:201,blendDstAlpha:201,side:2,depthTest:!1,depthWrite:!1,vertexShader:`
        varying vec3 vViewNormal;
        varying vec3 vViewPosition;
        varying vec3 vWorldPosition;
        void main() {
            vec4 mvPosition = modelViewMatrix * vec4(position, 1.0);
            vViewNormal = normalize(normalMatrix * normal);
            vViewPosition = mvPosition.xyz;
            vec4 worldPosition = modelMatrix * vec4(position, 1.0);
            vWorldPosition = worldPosition.xyz;
            gl_Position = projectionMatrix * mvPosition;
        }
    `,fragmentShader:`
        varying vec3 vViewNormal;
        varying vec3 vViewPosition;
        varying vec3 vWorldPosition;

        float hash(vec3 p) {
            return fract(sin(dot(p, vec3(17.13, 47.71, 91.37))) * 43758.5453);
        }

        float valueNoise(vec3 p) {
            vec3 i = floor(p);
            vec3 f = fract(p);
            f = f * f * (3.0 - 2.0 * f);
            float n000 = hash(i + vec3(0.0, 0.0, 0.0));
            float n100 = hash(i + vec3(1.0, 0.0, 0.0));
            float n010 = hash(i + vec3(0.0, 1.0, 0.0));
            float n110 = hash(i + vec3(1.0, 1.0, 0.0));
            float n001 = hash(i + vec3(0.0, 0.0, 1.0));
            float n101 = hash(i + vec3(1.0, 0.0, 1.0));
            float n011 = hash(i + vec3(0.0, 1.0, 1.0));
            float n111 = hash(i + vec3(1.0, 1.0, 1.0));
            float nx00 = mix(n000, n100, f.x);
            float nx10 = mix(n010, n110, f.x);
            float nx01 = mix(n001, n101, f.x);
            float nx11 = mix(n011, n111, f.x);
            float nxy0 = mix(nx00, nx10, f.y);
            float nxy1 = mix(nx01, nx11, f.y);
            return mix(nxy0, nxy1, f.z);
        }

        void main() {
            vec3 normal = normalize(vViewNormal);
            vec3 rayDir = normalize(-vViewPosition);
            float incidence = clamp(abs(dot(normal, rayDir)), 0.12, 1.0);
            float anglePath = clamp(pow(1.0 / incidence, 0.82), 1.0, 4.2);

            vec3 p = vWorldPosition * 0.035;
            float coarse = valueNoise(p);
            float fine = valueNoise(p * 2.8 + vec3(4.0, 11.0, 2.0));
            float trabeculae = smoothstep(0.42, 0.92, coarse * 0.62 + fine * 0.38);
            float marrowMottle = mix(0.72, 1.08, valueNoise(p * 1.35 + vec3(2.0, 7.0, 13.0)));

            float encodedDepth = length(vViewPosition) * 0.00072;
            float entryDepth = gl_FrontFacing ? encodedDepth : 0.0;
            float exitDepth = gl_FrontFacing ? 0.0 : encodedDepth;
            float grazingCortex = smoothstep(1.35, 3.8, anglePath);
            float corticalPath = anglePath * 0.0048 + grazingCortex * 0.036 + trabeculae * marrowMottle * 0.0009;
            float trabecularTexture = trabeculae * marrowMottle * 0.026;

            gl_FragColor = vec4(entryDepth, exitDepth, corticalPath, trabecularTexture);
        }
    `}),xn=new Bn({uniforms:{uTexture:{value:Yo.texture},contrastTexture:{value:ic.texture},thicknessTexture:{value:Tc.texture},metalTexture:{value:sc.texture},catheterTexture:{value:rc.texture},sheathTexture:{value:ac.texture},boneTexture:{value:wc.texture},gray:{value:new Qt(15461355)},fluoroscopy:{value:!1},time:{value:0},noiseLevel:{value:.1},imageBrightness:{value:.18},imageContrast:{value:1.33},autoExposureEnabled:{value:!1},autoExposureLevel:{value:0},pulseRate:{value:15},scatterStrength:{value:.45},collimation:{value:.08},boneOpacity:{value:.62},resolution:{value:new Mt(Yn,qn)},edgeStrength:{value:.1},contrastOpacity:{value:1},contrastGain:{value:5}},vertexShader:y1,fragmentShader:E1}),Z1=new jt(Jh,xn),yh=new Rr;yh.add(Z1);const $p=350,Ve=new In(45,window.innerWidth/window.innerHeight,Vp,kp);Ve.position.set(0,80,$p);oe.add(Ve);const Kp=-15;function Oi(r){return r.position.y+=Kp,r}let Pr;const{group:Rs}=HM({onLoaded:()=>{lc=!1,yr("skeleton",ns.has("aorta")?"Loading vessel model":"Rendering first frame")},onError:()=>Wp("skeleton")}),{vessel:fn}=AM(140,0);Pr=Oi(new Le);let _n=fn,xe=null,ke=null,_e=null,Ie=null,Gi=null,Qi=null,Ts=null,Ni=null,xr=null,ts=null;function Jp(r,t=1){const e=new R(r.start.x,r.start.y,r.start.z),n=new R(r.end.x,r.end.y,r.end.z),i=new R().subVectors(n,e),s=i.length(),a=r.radius*t,o=new Ps(a,a,s,18,1,!0);return o.applyQuaternion(new ni().setFromUnitVectors(new R(0,1,0),i.normalize())),o.translate((e.x+n.x)*.5,(e.y+n.y)*.5,(e.z+n.z)*.5),o}function j1(r){const t=Jp(r),e=new Fe({color:Kh,side:2,transparent:!0,opacity:.34,depthWrite:!1,depthTest:!1}),n=new jt(t,e);return n.renderOrder=6.6,n}function $1(r){const t=Jp(r),e=new Fe({color:16777215,side:2,transparent:!0,opacity:.065,depthTest:!1,depthWrite:!1}),n=new jt(t,e);return n.renderOrder=.7,n}function Af(r,{debugLayer:t=null,color:e=Kh,opacity:n=.24,renderOrder:i=3,depthTest:s=!0}={}){const a=new Fe({color:e,side:2,transparent:!0,opacity:n,depthWrite:!1,depthTest:s}),o=new jt(r,a);return o.renderOrder=i,t&&(o.userData.debugLayer=t),o}function K1(r,t){const e=r?.meshCollider||r?.lumenMeshCollider||null;if(!e?.pointContact||!t?.start||!t?.end)return null;const n=new R(t.start.x,t.start.y,t.start.z),i=new R(t.end.x,t.end.y,t.end.z),s=new R().subVectors(i,n),a=s.length();if(a<1e-6)return null;const o=d=>n.clone().addScaledVector(s,d),c=d=>e.pointContact(o(d),0)?.signedDistance??-1/0,l=Math.max(16,Math.ceil(a/2));let h=0,u=c(0);for(let d=1;d<=l;d++){const f=d/l,g=c(f);if(u<0&&g>=0){let x=h,m=f;for(let p=0;p<14;p++){const _=(x+m)*.5;c(_)>=0?m=_:x=_}return{point:o(m),tangent:s.normalize()}}h=f,u=g}return null}function J1(r,t){const e=K1(r,t),n=new Le;if(!e)return n;const i=new Fe({color:b1,transparent:!0,opacity:.95,depthTest:!1,depthWrite:!1,toneMapped:!1}),s=new jt(new ii(2.2,18,12),i);s.renderOrder=9.5,n.add(s);const a=new jt(new Xh(4.2,.32,8,32),i.clone());return a.quaternion.setFromUnitVectors(new R(0,0,1),e.tangent),a.renderOrder=9.4,n.add(a),n.position.copy(e.point),n.frustumCulled=!1,n}Pr.add(j1(fn.sheath));const As=$1(fn.sheath);As.visible=!0;Oi(As);oe.add(As);const pi=new Le;pi.visible=!1;Pr.add(pi);const _r={stlModel:!0,lumenCast:!1,sections:!1,centerline:!0,capsules:!1};function Qp(){pi.traverse(r=>{const t=r.userData?.debugLayer;!t||!(t in _r)||(r.visible=!!_r[t])})}Fa("Loading anatomy models");n1(fn,{onLoaded:({collision:r})=>{_n={...r,segments:[fn.sheath]},_e&&(_e.contactField=r.contactField),pi.clear(),pi.add(Af(r.geometry,{debugLayer:"stlModel",color:T1,opacity:.18,renderOrder:2.8})),r.preprocessing?.lumenCastGeometry&&pi.add(Af(r.preprocessing.lumenCastGeometry,{debugLayer:"lumenCast",color:Kh,opacity:.28,renderOrder:9.15,depthTest:!1})),pi.add(nw(r.preprocessing)),pi.add(x1(r.centerlineBroadPhase)),pi.add(J1(r,fn.sheath)),Qp(),ke?.requestSettle?.(90),xe?.setCollisionGeometry(r),yr("aorta",ns.has("skeleton")?"Loading skeleton model":"Rendering first frame")},onError:()=>{Wp("aorta")}});oe.add(Pr);Rs.position.set(fn.branchPoint.x,fn.branchPoint.y-60,fn.branchPoint.z-50);Rs.renderOrder=-1;oe.add(Rs);const Ra=new ZM(fn,3.5),di=Oi(new Le);oe.add(di);let Li=null,Xl=0,Yl=0;const Bi=5,Ac=201,vr=Bi*(Ac-1),Q1=44,xi=new sS(Ac,Bi,{constraintIterations:28});let Cc=0;const tw=vr,ew=0;function nw(r){const t=new Le;if(!r)return t;if(r.boundaryDebugSegments?.length){const i=new me;i.setAttribute("position",new Be(r.boundaryDebugSegments,3));const s=new ss(i,new gi({color:C1,transparent:!0,opacity:.85,depthTest:!1,depthWrite:!1,toneMapped:!1}));s.frustumCulled=!1,s.renderOrder=9,s.userData.debugLayer="sections",t.add(s)}const e=r.centerlineSliceDebugSegments?.length?r.centerlineSliceDebugSegments:r.lumenContourDebugSegments;if(e?.length){const i=new me;i.setAttribute("position",new Be(e,3));const s=new ss(i,new gi({color:R1,transparent:!0,opacity:.72,depthTest:!1,depthWrite:!1,toneMapped:!1}));s.frustumCulled=!1,s.renderOrder=8.5,s.userData.debugLayer="sections",t.add(s)}const n=r.interiorSamples||[];if(n.length){const i=new gc(new ii(1.15,10,6),new Fe({color:A1,transparent:!0,opacity:.82,depthTest:!1,depthWrite:!1,toneMapped:!1}),n.length),s=new re;for(let a=0;a<n.length;a++)s.makeTranslation(n[a].x,n[a].y,n[a].z),i.setMatrixAt(a,s);i.frustumCulled=!1,i.instanceMatrix.needsUpdate=!0,i.renderOrder=8,i.userData.debugLayer="sections",t.add(i)}return t}ke=new XS({rod:xi,segmentLength:Bi,guidewireLength:vr,sheath:fn.sheath,advanceRate:Q1,minInsert:ew,maxInsert:tw,lumenClearance:Mr,straightening:.72,routeBlend:0,relaxationIterations:6,lengthIterations:10,meshClearance:Mr,foldGuardAngle:166,foldGuardStrength:.62,foldGuardPasses:2,foldGuardCenterPull:1.25,stabilityRepairSegmentError:.09,stabilityRepairBendAngle:150,stabilityRepairTargetBendAngle:112,stabilityRepairPasses:3,stabilityRepairLengthIterations:10,tipBacktrackAngle:108,tipBacktrackStrength:1,segmentProjectionBlend:.48,maxSegmentProjectionStep:.32,collisionProjectionRepeats:1,segmentSamples:[.1,.24,.38,.52,.66,.8,.93],finalCollisionPasses:3,finalLengthPasses:2,finalProjectionPasses:2});KS(xi,{segmentLength:Bi});ke.initialize();Cc=ke.progress;let mi=!1,Eh=0,tm=2,wh=2,Cf=10,ba=0,Rf=0;const em=new Fe({color:16777215,depthTest:!1,depthWrite:!1,toneMapped:!1}),bf=new Bn({vertexShader:`
        varying float vWireProfile;
        void main() {
            mat4 instanceTransform = mat4(1.0);
            #ifdef USE_INSTANCING
                instanceTransform = instanceMatrix;
            #endif
            vec4 mvPosition = modelViewMatrix * instanceTransform * vec4(position, 1.0);
            vec3 viewNormal = normalize(normalMatrix * mat3(instanceTransform) * normal);
            vWireProfile = pow(clamp(abs(viewNormal.z), 0.0, 1.0), 0.75);
            gl_Position = projectionMatrix * mvPosition;
        }
    `,fragmentShader:`
        varying float vWireProfile;
        void main() {
            float profile = mix(0.06, 1.0, smoothstep(0.08, 0.94, vWireProfile));
            gl_FragColor = vec4(vec3(profile), 1.0);
        }
    `,depthTest:!1,depthWrite:!1,toneMapped:!1});let be=!0,An=null,Cn=null,He=null;const Pe=BM({camera:Ve,cameraRadius:$p,vessel:fn,voxelGroup:di,displayMaterial:xn,blendMaterial:hc,wireMaterial:em,onStartInjection:({rate:r,duration:t,volume:e})=>{mi||(mi=!0,Eh=0,wh=r,tm=t,Cf=e,ba=Cf)},onStopInjection:()=>{mi&&(mi=!1,ba=0)},onModeChange:r=>{be=r,Pr.visible=!be,As.visible=be,pi.visible=!be,An&&(An.visible=!be),Cn&&(Cn.visible=!be),He&&(He.visible=!be&&!!He.userData.hasPoint),Ni&&(Ni.visible=!be&&!!_r.capsules),Rs.visible=be,xn.uniforms.fluoroscopy.value=be},onDebugLayerChange:r=>{Object.assign(_r,r),Qp(),Ni&&(Ni.visible=!be&&!!_r.capsules)},onStartBrowserBenchmark:r=>cm({durationMs:r}),onStopBrowserBenchmark:()=>ru("ui")}),{monitor:iw}=Pe,La=new jt(new me,em);La.frustumCulled=!1;La.renderOrder=7;const Er=new Le;Er.add(La);Oi(Er);oe.add(Er);const Pf=Array.from({length:Ac},()=>new R),Lf=new re,sw=new ii(1.35,12,8),rw=new ii(2.1,12,8);An=new gc(sw,new Fe({color:P1,transparent:!0,opacity:.95,depthTest:!1,depthWrite:!1,toneMapped:!1}),Cs);An.instanceMatrix.setUsage(35048);An.count=0;An.visible=!0;An.frustumCulled=!1;An.renderOrder=6;Oi(An);oe.add(An);Cn=new gc(rw,new Fe({color:L1,transparent:!0,opacity:1,depthTest:!1,depthWrite:!1,toneMapped:!1}),Cs);Cn.instanceMatrix.setUsage(35048);Cn.count=0;Cn.visible=!0;Cn.frustumCulled=!1;Cn.renderOrder=7;Oi(Cn);oe.add(Cn);He=new jt(new ii(2.8,16,10),new Fe({color:D1,transparent:!0,opacity:1,depthTest:!1,depthWrite:!1,toneMapped:!1}));He.visible=!1;He.frustumCulled=!1;He.renderOrder=8;He.userData.hasPoint=!1;Oi(He);oe.add(He);function nm(r){const t=new Float32Array(Cs*6),e=new me;e.setAttribute("position",new Be(t,3)),e.setDrawRange(0,0);const n=new ss(e,new gi({color:r,transparent:!0,opacity:.95,depthTest:!1,depthWrite:!1,toneMapped:!1}));return n.frustumCulled=!1,n.renderOrder=9.9,n}Ni=new Le;xr=nm(2686935);ts=nm(16732120);Ni.add(xr,ts);Ni.visible=!be&&!!_r.capsules;Oi(Ni);oe.add(Ni);const aw={routeAssist:Tn==="legacy",boundaryDriven:Tn==="xpbd-contact-v1"},ow={resetVelocity:Tn!=="xpbd-contact-v1"},cw={collisions:!1},im={shapeCompliance:vc.catheter.shapeCompliance};xe=new Oy({wire:xi,segmentLength:Bi,guidewireLength:vr,tailProgressRef:()=>ke.progress,vessel:fn});xe.setExternalCollisionSolver(Tn==="xpbd-contact-v1");_n!==fn&&xe.setCollisionGeometry(_n);Oi(xe.mesh);oe.add(xe.mesh);_e=new TM({contactField:_n.contactField||null,fixedDt:1/120,maxSubsteps:2,iterations:6,penetrationIterations:8,highPenetration:.15,contactActivation:.2});Ie=_e.createRod("guidewire",Ac,Bi,{...vc.guidewire});Ie.syncFromElasticRod(xi);Gi=_e.createRod("catheter",320,4,{...vc.catheter});xe.syncXpbdBody(Gi,im);_e.addSheath({start:fn.sheath.start,end:fn.sheath.end,innerRadius:yp,bodies:[Ie,Gi]});Qi=_e.addContainment(Ie,Gi,{innerRadius:Ap,openProximal:!0,openDistal:!0,searchWindow:2,outerStartNode:xe.physicsLumenStartNode,innerResponse:0,outerResponse:1,finalProjection:"outer",outerFollowsInnerCenterline:!0,containedLength:0,enabled:!1});Ts=_e.addToolContact(Ie,Gi,{friction:.08,openDistalB:!0,enabled:!1});const Df=[Ie,Gi];globalThis.__OET_PHYSICS__={mode:Tn,world:_e,getStats:()=>_e.getStats()};const Ls=4e4,sm=yc*2,lw=60*1e3,Th=sm+lw,hw=610,rm=256,Qh=8,ga=new Float32Array(Ls),Ah=new Float32Array(hw),Ch=new Float32Array(Ls),uw=new Float32Array(Ls),Rh=new Float32Array(Ls),bh=new Float32Array(Ls),dn=new Float32Array(rm*Qh);let xa=0,bs=0,Da=0,Ia=0,tu=0,uc=0,wr=0,_a=0,Zo=0,dc=0,Ph=0,eu=0,nu=0,is=0,am=performance.now();const Zt={count:0,simulationSumMs:0,updateSumMs:0,renderSumMs:0,totalSumMs:0,maximumMs:0,simulationMaximumMs:0,updateMaximumMs:0,renderMaximumMs:0,lastSimulationMs:0,lastUpdateMs:0,lastRenderMs:0,lastTotalMs:0},Ae={supported:!1,samples:0,startBytes:null,minimumBytes:null,maximumBytes:null,endBytes:null},wt={running:!1,warmingUp:!1,durationMs:ec,warmupStartedAt:0,memorySettling:!1,startedAt:0,completedAt:0,simulationElapsedMs:0,stopReason:null,automated:!1},dw=["pointerdown","mousedown","touchstart","click","dblclick","wheel","keydown","input","change"];function fw(r){!wt.running||!wt.automated||(r.preventDefault(),r.stopImmediatePropagation())}for(const r of dw)window.addEventListener(r,fw,{capture:!0,passive:!1});const vs=w1(),Me={steps:0,maxPostStepPenetrationMm:0,maxPostStepPenetrationStep:-1,maxPostStepPenetrationBodyId:null,maxPostStepPenetrationSegment:-1,maxPostStepPenetrationT:0,maxPostStepPenetrationX:0,maxPostStepPenetrationY:0,maxPostStepPenetrationZ:0,maxTransientPenetrationMm:0,maxTransientPenetrationStep:-1,maxSegmentErrorPercent:0,maxSegmentErrorBodyId:null,maxSegmentErrorNodeIndex:-1,maxSegmentErrorStep:-1,maxBendAngleDegrees:0,maxBendBodyId:null,maxBendNodeIndex:-1,maxBendStep:-1,maxBendX:0,maxBendY:0,maxBendZ:0,finite:!0};let Na=null;function iu(){const r=performance.memory?.usedJSHeapSize;Number.isFinite(r)&&(Ae.supported=!0,Ae.samples===0?(Ae.startBytes=r,Ae.minimumBytes=r,Ae.maximumBytes=r):(Ae.minimumBytes=Math.min(Ae.minimumBytes,r),Ae.maximumBytes=Math.max(Ae.maximumBytes,r)),Ae.endBytes=r,Ae.samples++)}function pw(){Ae.supported=!1,Ae.samples=0,Ae.startBytes=null,Ae.minimumBytes=null,Ae.maximumBytes=null,Ae.endBytes=null,iu()}function mw(){return iu(),{...Ae,growthBytes:Ae.supported?Ae.endBytes-Ae.startBytes:null,rangeBytes:Ae.supported?Ae.maximumBytes-Ae.minimumBytes:null}}function su(){xa=0,bs=0,Da=0,Ia=0,tu=0,uc=0,wr=0,_a=0,Zo=0,dc=0,Ph=Pe.getCArmRevision?.()??0,eu=0,nu=0,is=document.hasFocus()?0:performance.now(),Zt.count=0,Zt.simulationSumMs=0,Zt.updateSumMs=0,Zt.renderSumMs=0,Zt.totalSumMs=0,Zt.maximumMs=0,Zt.simulationMaximumMs=0,Zt.updateMaximumMs=0,Zt.renderMaximumMs=0,Zt.lastSimulationMs=0,Zt.lastUpdateMs=0,Zt.lastRenderMs=0,Zt.lastTotalMs=0,pw(),Me.steps=0,Me.maxPostStepPenetrationMm=0,Me.maxPostStepPenetrationStep=-1,Me.maxPostStepPenetrationBodyId=null,Me.maxPostStepPenetrationSegment=-1,Me.maxTransientPenetrationMm=0,Me.maxTransientPenetrationStep=-1,Me.maxSegmentErrorPercent=0,Me.maxSegmentErrorBodyId=null,Me.maxSegmentErrorNodeIndex=-1,Me.maxSegmentErrorStep=-1,Me.maxBendAngleDegrees=0,Me.maxBendBodyId=null,Me.maxBendNodeIndex=-1,Me.maxBendStep=-1,Me.maxBendX=0,Me.maxBendY=0,Me.maxBendZ=0,Me.finite=!0,am=performance.now(),_e.resetPerformanceStats(),_n.contactField?.resetStats?.()}function gw(r){if(!(!Number.isFinite(r)||r<=0)){if(bs===ga.length?Da-=ga[xa]:bs++,ga[xa]=r,Da+=r,Ia=Math.max(Ia,r),_a+=r,Zo++,_a>=1e3&&(iu(),wr<Ah.length&&(Ah[wr++]=Zo*1e3/_a),_a=0,Zo=0),r>1e3/30&&(tu++,dc<rm)){const t=dc++*Qh;dn[t]=r,dn[t+1]=wt.running?performance.now()-wt.startedAt:-1,dn[t+2]=wt.simulationElapsedMs,dn[t+3]=performance.memory?.usedJSHeapSize??-1,dn[t+4]=Zt.lastSimulationMs,dn[t+5]=Zt.lastUpdateMs,dn[t+6]=Zt.lastRenderMs,dn[t+7]=Zt.lastTotalMs}r>50&&uc++,xa=(xa+1)%ga.length}}window.addEventListener("blur",()=>{!wt.running||wt.warmingUp||is>0||(eu++,is=performance.now())});window.addEventListener("focus",()=>{is<=0||(wt.running&&!wt.warmingUp&&(nu+=performance.now()-is),is=0)});function xw(){const r=Me;if(r.steps++,_e.settledMaxPenetration>r.maxPostStepPenetrationMm&&(r.maxPostStepPenetrationMm=_e.settledMaxPenetration,r.maxPostStepPenetrationStep=r.steps,r.maxPostStepPenetrationBodyId=_e.settledContactBodyId,r.maxPostStepPenetrationSegment=_e.settledContactSegment,r.maxPostStepPenetrationT=_e.settledContactT,r.maxPostStepPenetrationX=_e.settledContactX,r.maxPostStepPenetrationY=_e.settledContactY,r.maxPostStepPenetrationZ=_e.settledContactZ),_e.maxPenetration>r.maxTransientPenetrationMm&&(r.maxTransientPenetrationMm=_e.maxPenetration,r.maxTransientPenetrationStep=r.steps),!(r.steps!==1&&r.steps%30!==0))for(let t=0;t<Df.length;t++){const e=Df[t];if(!e)continue;const n=e.activeStart,i=Math.min(e.activeEnd,e.segmentCount);for(let s=n;s<=e.activeEnd;s++)r.finite=r.finite&&Number.isFinite(e.x[s])&&Number.isFinite(e.y[s])&&Number.isFinite(e.z[s])&&Number.isFinite(e.velocityX[s])&&Number.isFinite(e.velocityY[s])&&Number.isFinite(e.velocityZ[s]);for(let s=n;s<i;s++){const a=e.x[s+1]-e.x[s],o=e.y[s+1]-e.y[s],c=e.z[s+1]-e.z[s],l=Math.sqrt(a*a+o*o+c*c),h=Math.abs(l-e.restLength[s])/Math.max(1e-8,e.restLength[s])*100;if(h>r.maxSegmentErrorPercent&&(r.maxSegmentErrorPercent=h,r.maxSegmentErrorBodyId=e.id,r.maxSegmentErrorNodeIndex=s,r.maxSegmentErrorStep=r.steps),s<=n)continue;const u=e.x[s]-e.x[s-1],d=e.y[s]-e.y[s-1],f=e.z[s]-e.z[s-1],g=Math.sqrt(a*a+o*o+c*c)*Math.sqrt(u*u+d*d+f*f);if(g<=1e-8)continue;const x=he.clamp((a*u+o*d+c*f)/g,-1,1),m=Math.acos(x)*180/Math.PI;m>r.maxBendAngleDegrees&&(r.maxBendAngleDegrees=m,r.maxBendBodyId=e.id,r.maxBendNodeIndex=s,r.maxBendStep=r.steps,r.maxBendX=e.x[s],r.maxBendY=e.y[s],r.maxBendZ=e.z[s])}}}function _w(r){if(!bs)return 0;const t=Array.from(ga.subarray(0,bs));return t.sort((e,n)=>e-n),t[Math.min(t.length-1,Math.floor((t.length-1)*r))]}function vw(){if(!wr)return 0;const r=Array.from(Ah.subarray(0,wr));r.sort((n,i)=>n-i);const t=Math.max(1,Math.ceil(r.length*.01));let e=0;for(let n=0;n<t;n++)e+=r[n];return e/t}function Sw(){const r=[];for(let t=0;t<dc;t++){const e=t*Qh;r.push({frameMs:dn[e],elapsedMs:dn[e+1],simulationElapsedMs:dn[e+2],heapBytes:dn[e+3],previousFrameCpu:{simulationMs:dn[e+4],updateMs:dn[e+5],renderMs:dn[e+6],totalMs:dn[e+7]}})}return r}function If(r,t,e){if(!wt.running)return;const n=performance.now(),i=t-r,s=e-t,a=n-e,o=n-r,c=Zt.count;c<Ls&&(Ch[c]=i,uw[c]=s,Rh[c]=a,bh[c]=o),Zt.count++,Zt.simulationSumMs+=i,Zt.updateSumMs+=s,Zt.renderSumMs+=a,Zt.totalSumMs+=o,Zt.maximumMs=Math.max(Zt.maximumMs,o),Zt.simulationMaximumMs=Math.max(Zt.simulationMaximumMs,i),Zt.updateMaximumMs=Math.max(Zt.updateMaximumMs,s),Zt.renderMaximumMs=Math.max(Zt.renderMaximumMs,a),Zt.lastSimulationMs=i,Zt.lastUpdateMs=s,Zt.lastRenderMs=a,Zt.lastTotalMs=o}function rr(r,t){const e=Math.min(Zt.count,Ls);if(!e)return 0;const n=Array.from(r.subarray(0,e));return n.sort((i,s)=>i-s),n[Math.min(e-1,Math.floor((e-1)*t))]}function Mw(){const r=Zt.count||1;return{samples:Zt.count,simulationAverageMs:Zt.simulationSumMs/r,updateAverageMs:Zt.updateSumMs/r,renderAverageMs:Zt.renderSumMs/r,totalAverageMs:Zt.totalSumMs/r,simulationP95Ms:rr(Ch,.95),simulationP99Ms:rr(Ch,.99),renderP95Ms:rr(Rh,.95),renderP99Ms:rr(Rh,.99),totalP95Ms:rr(bh,.95),totalP99Ms:rr(bh,.99),simulationMaximumMs:Zt.simulationMaximumMs,updateMaximumMs:Zt.updateMaximumMs,renderMaximumMs:Zt.renderMaximumMs,maximumMs:Zt.maximumMs}}function Rc(){const r=performance.now(),t=wt.warmingUp?0:wt.running?Math.min(wt.durationMs,r-wt.startedAt):wt.completedAt>wt.startedAt?Math.min(wt.durationMs,wt.completedAt-wt.startedAt):0;return{running:wt.running,warmingUp:wt.warmingUp,warmupPhase:wt.warmingUp?wt.memorySettling?"memory-settle":"choreography":"complete",warmupElapsedMs:wt.warmingUp?Math.min(Th,r-wt.warmupStartedAt):Th,durationMs:wt.durationMs,elapsedMs:t,simulationElapsedMs:wt.simulationElapsedMs,progress:wt.durationMs>0?Math.min(1,t/wt.durationMs):0,cycleIndex:Math.floor(wt.simulationElapsedMs/yc),catheterType:Up(wt.simulationElapsedMs),stopReason:wt.stopReason,automated:wt.automated}}function Lh(){const r=performance.now(),t=_w(.99),e=vw(),n=_e.getStats(),i=_n.contactField?.getStats?.()||null,s=Rc(),a=!s.running&&s.durationMs>=ec&&s.elapsedMs>=ec,o=e>=55,c=uc===0,l=n.phases.total.averageMs<=4&&n.phases.total.p95Ms<=6,h=Me.maxPostStepPenetrationMm<=.2,u=Me.maxSegmentErrorPercent<=1,d=Me.maxBendAngleDegrees<150,f=Me.finite,g=Tn==="xpbd-contact-v1",x=!!_e.contactField,m=Math.max(0,(Pe.getCArmRevision?.()??Ph)-Ph),p=m===0,_=nu+(is>0?r-is:0),v=_<=100,S=mw(),M=!S.supported||S.growthBytes<=4*1024*1024&&S.rangeBytes<=8*1024*1024,y=i?.resultAllocations===0,w=(i?.runtimeBytes??1/0)<=32*1024*1024,A=Ia<100&&M&&y;return{mode:Tn,durationMs:performance.now()-am,frameCount:bs,averageFps:Da>0?bs*1e3/Da:0,onePercentLowFps:e,p99FrameMs:t,instantaneousP99Fps:t>0?1e3/t:0,fpsWindowCount:wr,maxFrameMs:Ia,longFrame33Count:tu,longFrame50Count:uc,longFrameEvents:Sw(),frameCpu:Mw(),physics:n,physicsEnvelope:{...Me},contactField:i,cameraProjectionChanges:m,heapBytes:S.endBytes,heap:S,pageState:{visibilityState:document.visibilityState,hasFocus:document.hasFocus(),focusLossCount:eu,focusLossMs:_},scenario:s,browserAcceptance:{durationPass:a,onePercentLowPass:o,noLongFramePass:c,noVisibleGcPausePass:A,physicsBudgetPass:l,narrowPhaseAllocationPass:y,memoryStabilityPass:M,runtimeAssetPass:w,penetrationPass:h,lengthPass:u,foldPass:d,finitePass:f,modePass:g,contactFieldPass:x,cameraStablePass:p,focusPass:v,passed:a&&o&&A&&l&&y&&M&&w&&h&&u&&d&&f&&g&&x&&p&&v}}}function ru(r="manual"){return r==="ui"&&wt.automated?Lh():(wt.running&&(wt.running=!1,wt.warmingUp=!1,wt.completedAt=performance.now(),wt.stopReason=r),Pe.setAutomatedBenchmarkMode?.(!1),Na=Lh(),Na)}function om({resetAccumulator:r=!0}={}){ke.reset(),Cc=ke.progress,xe.reset(),Ie.syncFromElasticRod(xi),xe.syncXpbdBody(Gi),Qi.enabled=!1,Ts.enabled=!1,_e.resetSimulationState(),r&&(cr=0)}function cm({durationMs:r=ec,automated:t=!1}={}){const e=Number(r);if(!Number.isFinite(e)||e<=0)throw new RangeError("Browser benchmark durationMs must be positive");if(!_e.contactField)throw new Error("Browser benchmark requires the precompiled vessel contact field");return om(),su(),wt.durationMs=e,wt.warmupStartedAt=performance.now(),wt.memorySettling=!1,wt.startedAt=0,wt.completedAt=0,wt.simulationElapsedMs=0,wt.stopReason=null,wt.automated=t===!0,Pe.setAutomatedBenchmarkMode?.(wt.automated),wt.running=!0,wt.warmingUp=!0,Na=null,Rc()}function yw(r){if(!wt.running)return null;const t=performance.now();if(wt.warmingUp){const i=t-wt.warmupStartedAt;if(i<sm){const s=Sf(wt.simulationElapsedMs,vs);return wt.simulationElapsedMs+=r*1e3,s}if(wt.memorySettling||(om({resetAccumulator:!1}),wt.memorySettling=!0,wt.simulationElapsedMs=0),i<Th)return vs.guidewireAdvance=0,vs.catheterAdvance=0,vs.catheterRotation=0,vs.catheterType="pigtail",vs;su(),wt.warmingUp=!1,wt.memorySettling=!1,wt.startedAt=performance.now(),wt.completedAt=0,wt.simulationElapsedMs=0}if(performance.now()-wt.startedAt>=wt.durationMs)return ru("duration"),null;const n=Sf(wt.simulationElapsedMs,vs);return wt.simulationElapsedMs+=r*1e3,n}globalThis.__OET_BENCHMARK__={reset:su,getReport:Lh,startScenario:cm,stopScenario:ru,getScenarioStatus:Rc,getLastScenarioReport:()=>Na};function Ew(r,t){const e=Tn==="legacy"?_n:null,n=ke.advance(r,t,e,aw);return Cc=ke.progress,n}function ww(){for(let t=0;t<xi.nodes.length;t++){const e=xi.nodes[t];Pf[t].set(e.x,e.y,e.z)}const r=La.geometry;La.geometry=bp(Pf,{radius:MM,samplesPerSegment:F1,radialSegments:N1}),r.dispose(),Er.visible=xi.nodes.length>1}function Tw(){if(!xr||!ts||Tn!=="xpbd-contact-v1")return{normalCount:0,branchCount:0};const r=xr.geometry.getAttribute("position"),t=ts.geometry.getAttribute("position"),e=r.array,n=t.array,i=_n.contactField,s=i?.centerline,a=i?.centerlineStride||0,o=a>0&&s?s.length/a:0;let c=ts.userData.seen;!c||c.length!==o?(c=new Uint8Array(o),ts.userData.seen=c):c.fill(0);let l=0,h=0;for(const u of[Ie,Gi]){if(!u)continue;const d=Math.min(u.segmentCount,u.activeEnd);for(let f=u.activeStart;f<d;f++){if(!u.wallActive[f])continue;if(l<Cs){const p=l*6,_=2.5+Math.min(4,u.wallLambda[f]*8);e[p]=u.wallX[f],e[p+1]=u.wallY[f],e[p+2]=u.wallZ[f],e[p+3]=u.wallX[f]+u.wallNormalX[f]*_,e[p+4]=u.wallY[f]+u.wallNormalY[f]*_,e[p+5]=u.wallZ[f]+u.wallNormalZ[f]*_,l++}const g=u.wallBranchId[f];if(g<0||g>=o||c[g]||h>=Cs)continue;c[g]=1;const x=g*a,m=h*6;for(let p=0;p<6;p++)n[m+p]=s[x+p];h++}}return xr.geometry.setDrawRange(0,l*2),ts.geometry.setDrawRange(0,h*2),r.needsUpdate=!0,t.needsUpdate=!0,{normalCount:l,branchCount:h}}function Aw(){if(be){Pe.updateGuidewireDiagnostics(null),An.count=0,Cn.count=0,He.userData.hasPoint=!1,He.visible=!1,xr?.geometry.setDrawRange(0,0),ts?.geometry.setDrawRange(0,0);return}const r=ke.collectLumenDiagnostics(_n,{clearance:ke.meshClearance,contactBand:I1,collectMarkers:!0,markerLimit:Cs});if(Tn==="xpbd-contact-v1"){const e=_e.getStats(),n=ke.getPerformanceStats(),i=Tw();r.performance={advanceMs:n.advanceMs,solveMs:e.phases.total.lastMs,projectMs:e.phases.narrowPhase.lastMs,diagnosticMs:0,pointContactCount:e.contacts,diagnosticPointContactCount:0,segmentSampleCount:_n.contactField?.getStats?.().capsuleSamples||0,activeBranchCount:i.branchCount,settledPenetration:e.settledMaxPenetration,maximumPenetration:e.maxPenetration}}else r.performance=ke.getPerformanceStats();Pe.updateGuidewireDiagnostics(r),r.worstPoint?(He.position.set(r.worstPoint.x,r.worstPoint.y+Kp,r.worstPoint.z),He.userData.hasPoint=!0,He.visible=!0):(He.userData.hasPoint=!1,He.visible=!1);const t=(e,n)=>{const i=Math.min(n.length,Cs);e.count=i;for(let s=0;s<i;s++){const a=n[s];Lf.makeTranslation(a.x,a.y,a.z),e.setMatrixAt(s,Lf)}e.instanceMatrix.needsUpdate=!0};t(An,r.contacts||[]),t(Cn,r.breaches||[])}function Cw(){if(Tn!=="xpbd-contact-v1"){Pe.updateGuidewireResistance(0,"");return}let r=0,t=0;for(let i=0;i<Ie.wallLambda.length;i++)Ie.wallActive[i]&&(r+=Ie.wallLambda[i],t++);const e=t?r/t:0,n=Math.max(0,Math.min(1,e/.08));Pe.updateGuidewireResistance(n,t?"Opór kontaktu prowadnika ze ścianą":"")}const or=Tn==="xpbd-contact-v1"?1/120:1/60;let Nf=performance.now(),cr=0,ql=-1/0,ys=0;const Zl=new R;let jl=zp,$l=Op,Kl=Gp,Jl=1/0;function Rw(r){const t=xn.uniforms,e=Math.min(1,Math.max(0,r)*1.35);if(!t.autoExposureEnabled.value){ys+=(0-ys)*Math.min(1,e*1.6),t.autoExposureLevel.value=ys;return}Ve.getWorldDirection(Zl);const n=Math.abs(Zl.x),i=Math.abs(Zl.y),s=Math.max(0,n-.1),a=he.clamp((t.collimation.value||0)/.45,0,1),o=1-a*.34,c=.012+s*.15+i*.035,l=he.clamp(c*o-a*.006,-.03,.18);ys+=(l-ys)*e,t.autoExposureLevel.value=ys}function Ff(){const r=xn.uniforms,t=he.clamp(r.pulseRate.value||15,7.5,30),e=r.autoExposureEnabled.value?he.clamp(ys/.18,0,1):.25,n=70+e*28,i=Math.pow(t/15,.72),a=1-he.clamp((r.collimation.value||0)/.45,0,1)*.42,o=(2.4+e*7.2)*i*a;Pe.updateXrayTechnique(n,o)}function bw(r=or){const t=yw(r),e=t?.guidewireAdvance??Pe.getAdvance(),n=t?.catheterAdvance??Pe.getCatheterAdvance(),i=t?.catheterRotation??Pe.getCatheterRotation();Ew(e,r);const s=Math.max(0,Cc);if(xe.setType(t?.catheterType??Pe.getSelectedCatheterType()),xe.advance(n,r,s),xe.rotate(i,r),Tn==="xpbd-contact-v1"){Ie.syncFromElasticRod(xi,ow),Ie.setActiveRange(Math.min(Ie.count-2,Math.max(0,ke.firstInsertedNodeIndex()-1)),Ie.count-1),Ie.setCollisionRange(Math.max(0,ke.firstLumenNodeIndex()-1),Ie.segmentCount-1),xe.stepPhysics(r,cw);const l=xe.syncXpbdBody(Gi,im);Qi.outerStartNode=xe.physicsLumenStartNode;const h=Math.max(0,Math.ceil((vr-s)/Bi)),u=Math.min(Ie.count-1,Math.floor((vr-s+xe.progress)/Bi));Qi.enabled=xe.progress>.5&&l>=2&&u>=h,Qi.startNode=h,Qi.endNode=Math.max(h,u),Qi.innerArcOffset=h*Bi-vr+s,Qi.containedLength=Math.min(xe.progress,s),Ie.nodeRadius.fill(Mr);const d=Math.max(0,l-2),f=Math.max(0,Math.min(Ie.segmentCount-1,u));Ts.enabled=!1,Ts.startSegmentA=f,Ts.endSegmentA=Math.min(Ie.activeEnd-1,f+16),Ts.startSegmentB=Math.max(0,d-8),Ts.endSegmentB=d,_e.stepFixed(),wt.running&&xw(),Ie.syncToElasticRod(xi)}else ke.solve(r,_n,{iterations:e===0?3:4}),xe.stepPhysics(r);const a=n!==0||i!==0,o=e!==0,c=xe.progress>4&&s>0;if(Tn==="legacy"&&(xe.constrainGuidewire(r,{reactionScale:o&&!a?.08:1}),o&&!a&&c&&(ke.solve(r,_n,{iterations:8,forceRelax:!0}),xe.constrainGuidewire(r,{reactionScale:.04}),ke.solve(r,_n,{iterations:5,forceRelax:!0})),a&&(ke.solve(r,_n,{iterations:10,forceRelax:!0}),xe.constrainGuidewire(r),ke.solve(r,_n,{iterations:8,forceRelax:!0}))),Cw(),Pe.updateInsertedLength(s/10),Pe.updateCatheterLength(xe.progress/10),mi){const l=Math.min(wh*r,ba);Ra.injectThroughSheath(l,wh),Rf+=l,Pe.updateDose(Rf),Eh+=r,ba-=l,(Eh>=tm||ba<=0)&&(mi=!1,Pe.setStopInjectionDisabled(!0))}Ra.update(r),iw.update(r)}const zo=[];function lr(r,t,e){zo.length=0;for(const n of r.children){if(n.isCamera)continue;!(n===e&&n.visible)&&n.visible&&(zo.push(n),n.visible=!1)}qt.render(r,t);for(let n=0;n<zo.length;n++)zo[n].visible=!0}const Oo=[],Ql=[];function Pw(){Ve.updateMatrixWorld(!0);const r=Ve.matrixWorld.elements,t=Ve.projectionMatrix.elements;let e=!lc;for(let n=0;n<16&&!e;n++)e=r[n]!==wf[n]||t[n]!==Tf[n];return e?(wf.set(r),Tf.set(t),lc=!0,!0):!1}function Lw(){Oo.length=0,Ql.length=0;for(const r of oe.children)r!==Rs&&!r.isCamera&&(Oo.push(r),Ql.push(r.visible),r.visible=!1);oe.overrideMaterial=W1,qt.setRenderTarget(oc),qt.clear(),qt.render(oe,Ve),oe.overrideMaterial=X1,qt.setRenderTarget(cc),qt.clear(),qt.render(oe,Ve),oe.overrideMaterial=null,qt.setRenderTarget(null);for(let r=0;r<Oo.length;r++)Oo[r].visible=Ql[r];Mh.uniforms.frontDepth.value=oc.texture,Mh.uniforms.backDepth.value=cc.texture,qt.setRenderTarget(Tc),qt.render(jp,qo),qt.setRenderTarget(null),qt.setRenderTarget(wc),qt.clear(),oe.overrideMaterial=q1,lr(oe,Ve,Rs),oe.overrideMaterial=null,qt.setRenderTarget(null)}function Dh(r){const t=performance.now(),e=r-Nf,n=Math.max(0,Math.min(.1,e/1e3));Nf=r,gw(e),cr+=n;let i=0;for(;cr+1e-9>=or&&i<2;)bw(or),cr-=or,i++;cr>=or&&(cr%=or);const s=performance.now();if($l+=n,$l>=Op&&($l=0,ww()),jl+=n,jl>=zp&&(jl=0,Aw()),Kl+=n,Kl>=Gp&&(Kl=0,xe.updateMesh()),mi||Ra.hasVisibleContrast()||Xl>0){Yl+=n;const l=mi?1/30:1/24;if(!Li||Yl>=l){Yl=0;const h=jM(Ra,.01,!be,Li);Xl=h.count,h.mesh&&h.mesh!==Li&&(Li&&di.remove(Li),Li=h.mesh,di.add(Li))}}else Li&&(Li.visible=!1);be&&di.parent!==ha?(oe.remove(di),ha.add(di)):!be&&di.parent!==oe&&(ha.remove(di),oe.add(di));const o=Xl>0||mi||Ra.hasVisibleContrast();if(Pr.visible=!be,As.visible=be,An&&(An.visible=!be),Cn&&(Cn.visible=!be),He&&(He.visible=!be&&!!He.userData.hasPoint),Rs.visible=be,Pe.setInjectButtonDisabled(o),Pe.setStopInjectionDisabled(!mi),Jl+=n,Jl>=.25){Jl=0;const l=Rc();Pe.updateBrowserBenchmarkStatus(l,l.running?null:Na)}const c=performance.now();if(be){Rw(n),Ff();const h=1e3/Math.max(1,xn.uniforms.pulseRate.value||15);if(!(r-ql>=h)){qt.setRenderTarget(null),qt.render(yh,qo),Pe.updatePerfStats(n),If(t,s,c),requestAnimationFrame(Dh);return}ql=r,Pw()&&Lw(),qt.setRenderTarget(ic),qt.setClearColor(0,0),qt.clear(),qt.render(ha,Ve),qt.setClearColor(0,1),qt.setRenderTarget(sc),qt.setClearColor(0,0),qt.clear(),oe.overrideMaterial=bf,lr(oe,Ve,Er),oe.overrideMaterial=null,qt.setClearColor(0,1),qt.setRenderTarget(rc),qt.setClearColor(0,0),qt.clear(),lr(oe,Ve,xe.mesh),qt.setClearColor(0,1),qt.setRenderTarget(ac),qt.setClearColor(0,0),qt.clear(),lr(oe,Ve,As),qt.setClearColor(0,1),qt.setRenderTarget(Sh),qt.clear(),lr(oe,Ve,As);const d=qt.autoClear;qt.autoClear=!1,oe.overrideMaterial=bf,lr(oe,Ve,Er),oe.overrideMaterial=null,qt.render(ha,Ve),qt.autoClear=d,hc.uniforms.currentFrame.value=Sh.texture,hc.uniforms.previousFrame.value=Yo.texture,qt.setRenderTarget(Uo),qt.render(qp,qo),qt.setRenderTarget(null),xn.uniforms.uTexture.value=Uo.texture,xn.uniforms.contrastTexture.value=ic.texture,xn.uniforms.thicknessTexture.value=Tc.texture,xn.uniforms.metalTexture.value=sc.texture,xn.uniforms.catheterTexture.value=rc.texture,xn.uniforms.sheathTexture.value=ac.texture,xn.uniforms.boneTexture.value=wc.texture,xn.uniforms.time.value=r*.001,qt.render(yh,qo),Ef();const f=Yo;Yo=Uo,Uo=f}else ql=-1/0,Ff(),qt.setRenderTarget(null),qt.render(oe,Ve),Ef();Pe.updatePerfStats(n),If(t,s,c),requestAnimationFrame(Dh)}requestAnimationFrame(Dh);window.addEventListener("resize",()=>{const r=window.innerWidth,t=window.innerHeight,e=Math.max(1,Math.round(r*nc)),n=Math.max(1,Math.round(t*nc));qt.setSize(r,t),Ve.aspect=r/t,Ve.updateProjectionMatrix(),Sh.setSize(e,n),ic.setSize(e,n),sc.setSize(e,n),rc.setSize(e,n),ac.setSize(e,n),wc.setSize(e,n),Xp.setSize(e,n),Yp.setSize(e,n),oc.setSize(e,n),cc.setSize(e,n),Tc.setSize(e,n),lc=!1,xn.uniforms.resolution.value.set(e,n)});
