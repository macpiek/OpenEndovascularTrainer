const nn="srgb",Ii="srgb-linear",Mh="display-p3",cc="display-p3-linear",Wo="linear",ye="srgb",Xo="rec709";const jh="300 es";class Sr{addEventListener(t,e){this._listeners===void 0&&(this._listeners={});const n=this._listeners;n[t]===void 0&&(n[t]=[]),n[t].indexOf(e)===-1&&n[t].push(e)}hasEventListener(t,e){if(this._listeners===void 0)return!1;const n=this._listeners;return n[t]!==void 0&&n[t].indexOf(e)!==-1}removeEventListener(t,e){if(this._listeners===void 0)return;const i=this._listeners[t];if(i!==void 0){const s=i.indexOf(e);s!==-1&&i.splice(s,1)}}dispatchEvent(t){if(this._listeners===void 0)return;const n=this._listeners[t.type];if(n!==void 0){t.target=this;const i=n.slice(0);for(let s=0,a=i.length;s<a;s++)i[s].call(this,t);t.target=null}}}const rn=["00","01","02","03","04","05","06","07","08","09","0a","0b","0c","0d","0e","0f","10","11","12","13","14","15","16","17","18","19","1a","1b","1c","1d","1e","1f","20","21","22","23","24","25","26","27","28","29","2a","2b","2c","2d","2e","2f","30","31","32","33","34","35","36","37","38","39","3a","3b","3c","3d","3e","3f","40","41","42","43","44","45","46","47","48","49","4a","4b","4c","4d","4e","4f","50","51","52","53","54","55","56","57","58","59","5a","5b","5c","5d","5e","5f","60","61","62","63","64","65","66","67","68","69","6a","6b","6c","6d","6e","6f","70","71","72","73","74","75","76","77","78","79","7a","7b","7c","7d","7e","7f","80","81","82","83","84","85","86","87","88","89","8a","8b","8c","8d","8e","8f","90","91","92","93","94","95","96","97","98","99","9a","9b","9c","9d","9e","9f","a0","a1","a2","a3","a4","a5","a6","a7","a8","a9","aa","ab","ac","ad","ae","af","b0","b1","b2","b3","b4","b5","b6","b7","b8","b9","ba","bb","bc","bd","be","bf","c0","c1","c2","c3","c4","c5","c6","c7","c8","c9","ca","cb","cc","cd","ce","cf","d0","d1","d2","d3","d4","d5","d6","d7","d8","d9","da","db","dc","dd","de","df","e0","e1","e2","e3","e4","e5","e6","e7","e8","e9","ea","eb","ec","ed","ee","ef","f0","f1","f2","f3","f4","f5","f6","f7","f8","f9","fa","fb","fc","fd","fe","ff"];let $h=1234567;const ma=Math.PI/180,Ta=180/Math.PI;function Mr(){const r=Math.random()*4294967295|0,t=Math.random()*4294967295|0,e=Math.random()*4294967295|0,n=Math.random()*4294967295|0;return(rn[r&255]+rn[r>>8&255]+rn[r>>16&255]+rn[r>>24&255]+"-"+rn[t&255]+rn[t>>8&255]+"-"+rn[t>>16&15|64]+rn[t>>24&255]+"-"+rn[e&63|128]+rn[e>>8&255]+"-"+rn[e>>16&255]+rn[e>>24&255]+rn[n&255]+rn[n>>8&255]+rn[n>>16&255]+rn[n>>24&255]).toLowerCase()}function ke(r,t,e){return Math.max(t,Math.min(e,r))}function yh(r,t){return(r%t+t)%t}function Jp(r,t,e,n,i){return n+(r-t)*(i-n)/(e-t)}function Qp(r,t,e){return r!==t?(e-r)/(t-r):0}function ga(r,t,e){return(1-e)*r+e*t}function tm(r,t,e,n){return ga(r,t,1-Math.exp(-e*n))}function em(r,t=1){return t-Math.abs(yh(r,t*2)-t)}function nm(r,t,e){return r<=t?0:r>=e?1:(r=(r-t)/(e-t),r*r*(3-2*r))}function im(r,t,e){return r<=t?0:r>=e?1:(r=(r-t)/(e-t),r*r*r*(r*(r*6-15)+10))}function sm(r,t){return r+Math.floor(Math.random()*(t-r+1))}function rm(r,t){return r+Math.random()*(t-r)}function am(r){return r*(.5-Math.random())}function om(r){r!==void 0&&($h=r);let t=$h+=1831565813;return t=Math.imul(t^t>>>15,t|1),t^=t+Math.imul(t^t>>>7,t|61),((t^t>>>14)>>>0)/4294967296}function cm(r){return r*ma}function lm(r){return r*Ta}function Vl(r){return(r&r-1)===0&&r!==0}function hm(r){return Math.pow(2,Math.ceil(Math.log(r)/Math.LN2))}function Yo(r){return Math.pow(2,Math.floor(Math.log(r)/Math.LN2))}function um(r,t,e,n,i){const s=Math.cos,a=Math.sin,o=s(e/2),c=a(e/2),l=s((t+n)/2),h=a((t+n)/2),u=s((t-n)/2),d=a((t-n)/2),f=s((n-t)/2),m=a((n-t)/2);switch(i){case"XYX":r.set(o*h,c*u,c*d,o*l);break;case"YZY":r.set(c*d,o*h,c*u,o*l);break;case"ZXZ":r.set(c*u,c*d,o*h,o*l);break;case"XZX":r.set(o*h,c*m,c*f,o*l);break;case"YXY":r.set(c*f,o*h,c*m,o*l);break;case"ZYZ":r.set(c*m,c*f,o*h,o*l);break;default:console.warn("THREE.MathUtils: .setQuaternionFromProperEuler() encountered an unknown order: "+i)}}function er(r,t){switch(t.constructor){case Float32Array:return r;case Uint32Array:return r/4294967295;case Uint16Array:return r/65535;case Uint8Array:return r/255;case Int32Array:return Math.max(r/2147483647,-1);case Int16Array:return Math.max(r/32767,-1);case Int8Array:return Math.max(r/127,-1);default:throw new Error("Invalid component type.")}}function dn(r,t){switch(t.constructor){case Float32Array:return r;case Uint32Array:return Math.round(r*4294967295);case Uint16Array:return Math.round(r*65535);case Uint8Array:return Math.round(r*255);case Int32Array:return Math.round(r*2147483647);case Int16Array:return Math.round(r*32767);case Int8Array:return Math.round(r*127);default:throw new Error("Invalid component type.")}}const ue={DEG2RAD:ma,RAD2DEG:Ta,generateUUID:Mr,clamp:ke,euclideanModulo:yh,mapLinear:Jp,inverseLerp:Qp,lerp:ga,damp:tm,pingpong:em,smoothstep:nm,smootherstep:im,randInt:sm,randFloat:rm,randFloatSpread:am,seededRandom:om,degToRad:cm,radToDeg:lm,isPowerOfTwo:Vl,ceilPowerOfTwo:hm,floorPowerOfTwo:Yo,setQuaternionFromProperEuler:um,normalize:dn,denormalize:er};class yt{constructor(t=0,e=0){yt.prototype.isVector2=!0,this.x=t,this.y=e}get width(){return this.x}set width(t){this.x=t}get height(){return this.y}set height(t){this.y=t}set(t,e){return this.x=t,this.y=e,this}setScalar(t){return this.x=t,this.y=t,this}setX(t){return this.x=t,this}setY(t){return this.y=t,this}setComponent(t,e){switch(t){case 0:this.x=e;break;case 1:this.y=e;break;default:throw new Error("index is out of range: "+t)}return this}getComponent(t){switch(t){case 0:return this.x;case 1:return this.y;default:throw new Error("index is out of range: "+t)}}clone(){return new this.constructor(this.x,this.y)}copy(t){return this.x=t.x,this.y=t.y,this}add(t){return this.x+=t.x,this.y+=t.y,this}addScalar(t){return this.x+=t,this.y+=t,this}addVectors(t,e){return this.x=t.x+e.x,this.y=t.y+e.y,this}addScaledVector(t,e){return this.x+=t.x*e,this.y+=t.y*e,this}sub(t){return this.x-=t.x,this.y-=t.y,this}subScalar(t){return this.x-=t,this.y-=t,this}subVectors(t,e){return this.x=t.x-e.x,this.y=t.y-e.y,this}multiply(t){return this.x*=t.x,this.y*=t.y,this}multiplyScalar(t){return this.x*=t,this.y*=t,this}divide(t){return this.x/=t.x,this.y/=t.y,this}divideScalar(t){return this.multiplyScalar(1/t)}applyMatrix3(t){const e=this.x,n=this.y,i=t.elements;return this.x=i[0]*e+i[3]*n+i[6],this.y=i[1]*e+i[4]*n+i[7],this}min(t){return this.x=Math.min(this.x,t.x),this.y=Math.min(this.y,t.y),this}max(t){return this.x=Math.max(this.x,t.x),this.y=Math.max(this.y,t.y),this}clamp(t,e){return this.x=Math.max(t.x,Math.min(e.x,this.x)),this.y=Math.max(t.y,Math.min(e.y,this.y)),this}clampScalar(t,e){return this.x=Math.max(t,Math.min(e,this.x)),this.y=Math.max(t,Math.min(e,this.y)),this}clampLength(t,e){const n=this.length();return this.divideScalar(n||1).multiplyScalar(Math.max(t,Math.min(e,n)))}floor(){return this.x=Math.floor(this.x),this.y=Math.floor(this.y),this}ceil(){return this.x=Math.ceil(this.x),this.y=Math.ceil(this.y),this}round(){return this.x=Math.round(this.x),this.y=Math.round(this.y),this}roundToZero(){return this.x=Math.trunc(this.x),this.y=Math.trunc(this.y),this}negate(){return this.x=-this.x,this.y=-this.y,this}dot(t){return this.x*t.x+this.y*t.y}cross(t){return this.x*t.y-this.y*t.x}lengthSq(){return this.x*this.x+this.y*this.y}length(){return Math.sqrt(this.x*this.x+this.y*this.y)}manhattanLength(){return Math.abs(this.x)+Math.abs(this.y)}normalize(){return this.divideScalar(this.length()||1)}angle(){return Math.atan2(-this.y,-this.x)+Math.PI}angleTo(t){const e=Math.sqrt(this.lengthSq()*t.lengthSq());if(e===0)return Math.PI/2;const n=this.dot(t)/e;return Math.acos(ke(n,-1,1))}distanceTo(t){return Math.sqrt(this.distanceToSquared(t))}distanceToSquared(t){const e=this.x-t.x,n=this.y-t.y;return e*e+n*n}manhattanDistanceTo(t){return Math.abs(this.x-t.x)+Math.abs(this.y-t.y)}setLength(t){return this.normalize().multiplyScalar(t)}lerp(t,e){return this.x+=(t.x-this.x)*e,this.y+=(t.y-this.y)*e,this}lerpVectors(t,e,n){return this.x=t.x+(e.x-t.x)*n,this.y=t.y+(e.y-t.y)*n,this}equals(t){return t.x===this.x&&t.y===this.y}fromArray(t,e=0){return this.x=t[e],this.y=t[e+1],this}toArray(t=[],e=0){return t[e]=this.x,t[e+1]=this.y,t}fromBufferAttribute(t,e){return this.x=t.getX(e),this.y=t.getY(e),this}rotateAround(t,e){const n=Math.cos(e),i=Math.sin(e),s=this.x-t.x,a=this.y-t.y;return this.x=s*n-a*i+t.x,this.y=s*i+a*n+t.y,this}random(){return this.x=Math.random(),this.y=Math.random(),this}*[Symbol.iterator](){yield this.x,yield this.y}}class ie{constructor(t,e,n,i,s,a,o,c,l){ie.prototype.isMatrix3=!0,this.elements=[1,0,0,0,1,0,0,0,1],t!==void 0&&this.set(t,e,n,i,s,a,o,c,l)}set(t,e,n,i,s,a,o,c,l){const h=this.elements;return h[0]=t,h[1]=i,h[2]=o,h[3]=e,h[4]=s,h[5]=c,h[6]=n,h[7]=a,h[8]=l,this}identity(){return this.set(1,0,0,0,1,0,0,0,1),this}copy(t){const e=this.elements,n=t.elements;return e[0]=n[0],e[1]=n[1],e[2]=n[2],e[3]=n[3],e[4]=n[4],e[5]=n[5],e[6]=n[6],e[7]=n[7],e[8]=n[8],this}extractBasis(t,e,n){return t.setFromMatrix3Column(this,0),e.setFromMatrix3Column(this,1),n.setFromMatrix3Column(this,2),this}setFromMatrix4(t){const e=t.elements;return this.set(e[0],e[4],e[8],e[1],e[5],e[9],e[2],e[6],e[10]),this}multiply(t){return this.multiplyMatrices(this,t)}premultiply(t){return this.multiplyMatrices(t,this)}multiplyMatrices(t,e){const n=t.elements,i=e.elements,s=this.elements,a=n[0],o=n[3],c=n[6],l=n[1],h=n[4],u=n[7],d=n[2],f=n[5],m=n[8],x=i[0],g=i[3],p=i[6],_=i[1],v=i[4],S=i[7],M=i[2],y=i[5],w=i[8];return s[0]=a*x+o*_+c*M,s[3]=a*g+o*v+c*y,s[6]=a*p+o*S+c*w,s[1]=l*x+h*_+u*M,s[4]=l*g+h*v+u*y,s[7]=l*p+h*S+u*w,s[2]=d*x+f*_+m*M,s[5]=d*g+f*v+m*y,s[8]=d*p+f*S+m*w,this}multiplyScalar(t){const e=this.elements;return e[0]*=t,e[3]*=t,e[6]*=t,e[1]*=t,e[4]*=t,e[7]*=t,e[2]*=t,e[5]*=t,e[8]*=t,this}determinant(){const t=this.elements,e=t[0],n=t[1],i=t[2],s=t[3],a=t[4],o=t[5],c=t[6],l=t[7],h=t[8];return e*a*h-e*o*l-n*s*h+n*o*c+i*s*l-i*a*c}invert(){const t=this.elements,e=t[0],n=t[1],i=t[2],s=t[3],a=t[4],o=t[5],c=t[6],l=t[7],h=t[8],u=h*a-o*l,d=o*c-h*s,f=l*s-a*c,m=e*u+n*d+i*f;if(m===0)return this.set(0,0,0,0,0,0,0,0,0);const x=1/m;return t[0]=u*x,t[1]=(i*l-h*n)*x,t[2]=(o*n-i*a)*x,t[3]=d*x,t[4]=(h*e-i*c)*x,t[5]=(i*s-o*e)*x,t[6]=f*x,t[7]=(n*c-l*e)*x,t[8]=(a*e-n*s)*x,this}transpose(){let t;const e=this.elements;return t=e[1],e[1]=e[3],e[3]=t,t=e[2],e[2]=e[6],e[6]=t,t=e[5],e[5]=e[7],e[7]=t,this}getNormalMatrix(t){return this.setFromMatrix4(t).invert().transpose()}transposeIntoArray(t){const e=this.elements;return t[0]=e[0],t[1]=e[3],t[2]=e[6],t[3]=e[1],t[4]=e[4],t[5]=e[7],t[6]=e[2],t[7]=e[5],t[8]=e[8],this}setUvTransform(t,e,n,i,s,a,o){const c=Math.cos(s),l=Math.sin(s);return this.set(n*c,n*l,-n*(c*a+l*o)+a+t,-i*l,i*c,-i*(-l*a+c*o)+o+e,0,0,1),this}scale(t,e){return this.premultiply(Mc.makeScale(t,e)),this}rotate(t){return this.premultiply(Mc.makeRotation(-t)),this}translate(t,e){return this.premultiply(Mc.makeTranslation(t,e)),this}makeTranslation(t,e){return t.isVector2?this.set(1,0,t.x,0,1,t.y,0,0,1):this.set(1,0,t,0,1,e,0,0,1),this}makeRotation(t){const e=Math.cos(t),n=Math.sin(t);return this.set(e,-n,0,n,e,0,0,0,1),this}makeScale(t,e){return this.set(t,0,0,0,e,0,0,0,1),this}equals(t){const e=this.elements,n=t.elements;for(let i=0;i<9;i++)if(e[i]!==n[i])return!1;return!0}fromArray(t,e=0){for(let n=0;n<9;n++)this.elements[n]=t[n+e];return this}toArray(t=[],e=0){const n=this.elements;return t[e]=n[0],t[e+1]=n[1],t[e+2]=n[2],t[e+3]=n[3],t[e+4]=n[4],t[e+5]=n[5],t[e+6]=n[6],t[e+7]=n[7],t[e+8]=n[8],t}clone(){return new this.constructor().fromArray(this.elements)}}const Mc=new ie;function Tf(r){for(let t=r.length-1;t>=0;--t)if(r[t]>=65535)return!0;return!1}function qo(r){return document.createElementNS("http://www.w3.org/1999/xhtml",r)}function dm(){const r=qo("canvas");return r.style.display="block",r}const Kh={};function xa(r){r in Kh||(Kh[r]=!0,console.warn(r))}const Jh=new ie().set(.8224621,.177538,0,.0331941,.9668058,0,.0170827,.0723974,.9105199),Qh=new ie().set(1.2249401,-.2249404,0,-.0420569,1.0420571,0,-.0196376,-.0786361,1.0982735),La={[Ii]:{transfer:Wo,primaries:Xo,toReference:r=>r,fromReference:r=>r},[nn]:{transfer:ye,primaries:Xo,toReference:r=>r.convertSRGBToLinear(),fromReference:r=>r.convertLinearToSRGB()},[cc]:{transfer:Wo,primaries:"p3",toReference:r=>r.applyMatrix3(Qh),fromReference:r=>r.applyMatrix3(Jh)},[Mh]:{transfer:ye,primaries:"p3",toReference:r=>r.convertSRGBToLinear().applyMatrix3(Qh),fromReference:r=>r.applyMatrix3(Jh).convertLinearToSRGB()}},fm=new Set([Ii,cc]),ge={enabled:!0,_workingColorSpace:Ii,get workingColorSpace(){return this._workingColorSpace},set workingColorSpace(r){if(!fm.has(r))throw new Error(`Unsupported working color space, "${r}".`);this._workingColorSpace=r},convert:function(r,t,e){if(this.enabled===!1||t===e||!t||!e)return r;const n=La[t].toReference,i=La[e].fromReference;return i(n(r))},fromWorkingColorSpace:function(r,t){return this.convert(r,this._workingColorSpace,t)},toWorkingColorSpace:function(r,t){return this.convert(r,t,this._workingColorSpace)},getPrimaries:function(r){return La[r].primaries},getTransfer:function(r){return r===""?Wo:La[r].transfer}};function hr(r){return r<.04045?r*.0773993808:Math.pow(r*.9478672986+.0521327014,2.4)}function yc(r){return r<.0031308?r*12.92:1.055*Math.pow(r,.41666)-.055}let bs;class Cf{static getDataURL(t){if(/^data:/i.test(t.src)||typeof HTMLCanvasElement>"u")return t.src;let e;if(t instanceof HTMLCanvasElement)e=t;else{bs===void 0&&(bs=qo("canvas")),bs.width=t.width,bs.height=t.height;const n=bs.getContext("2d");t instanceof ImageData?n.putImageData(t,0,0):n.drawImage(t,0,0,t.width,t.height),e=bs}return e.width>2048||e.height>2048?(console.warn("THREE.ImageUtils.getDataURL: Image converted to jpg for performance reasons",t),e.toDataURL("image/jpeg",.6)):e.toDataURL("image/png")}static sRGBToLinear(t){if(typeof HTMLImageElement<"u"&&t instanceof HTMLImageElement||typeof HTMLCanvasElement<"u"&&t instanceof HTMLCanvasElement||typeof ImageBitmap<"u"&&t instanceof ImageBitmap){const e=qo("canvas");e.width=t.width,e.height=t.height;const n=e.getContext("2d");n.drawImage(t,0,0,t.width,t.height);const i=n.getImageData(0,0,t.width,t.height),s=i.data;for(let a=0;a<s.length;a++)s[a]=hr(s[a]/255)*255;return n.putImageData(i,0,0),e}else if(t.data){const e=t.data.slice(0);for(let n=0;n<e.length;n++)e instanceof Uint8Array||e instanceof Uint8ClampedArray?e[n]=Math.floor(hr(e[n]/255)*255):e[n]=hr(e[n]);return{data:e,width:t.width,height:t.height}}else return console.warn("THREE.ImageUtils.sRGBToLinear(): Unsupported image type. No color space conversion applied."),t}}let pm=0;class bf{constructor(t=null){this.isSource=!0,Object.defineProperty(this,"id",{value:pm++}),this.uuid=Mr(),this.data=t,this.version=0}set needsUpdate(t){t===!0&&this.version++}toJSON(t){const e=t===void 0||typeof t=="string";if(!e&&t.images[this.uuid]!==void 0)return t.images[this.uuid];const n={uuid:this.uuid,url:""},i=this.data;if(i!==null){let s;if(Array.isArray(i)){s=[];for(let a=0,o=i.length;a<o;a++)i[a].isDataTexture?s.push(Ec(i[a].image)):s.push(Ec(i[a]))}else s=Ec(i);n.url=s}return e||(t.images[this.uuid]=n),n}}function Ec(r){return typeof HTMLImageElement<"u"&&r instanceof HTMLImageElement||typeof HTMLCanvasElement<"u"&&r instanceof HTMLCanvasElement||typeof ImageBitmap<"u"&&r instanceof ImageBitmap?Cf.getDataURL(r):r.data?{data:Array.from(r.data),width:r.width,height:r.height,type:r.data.constructor.name}:(console.warn("THREE.Texture: Unable to serialize Texture."),{})}let mm=0;class Nn extends Sr{constructor(t=Nn.DEFAULT_IMAGE,e=Nn.DEFAULT_MAPPING,n=1001,i=1001,s=1006,a=1008,o=1023,c=1009,l=Nn.DEFAULT_ANISOTROPY,h=""){super(),this.isTexture=!0,Object.defineProperty(this,"id",{value:mm++}),this.uuid=Mr(),this.name="",this.source=new bf(t),this.mipmaps=[],this.mapping=e,this.channel=0,this.wrapS=n,this.wrapT=i,this.magFilter=s,this.minFilter=a,this.anisotropy=l,this.format=o,this.internalFormat=null,this.type=c,this.offset=new yt(0,0),this.repeat=new yt(1,1),this.center=new yt(0,0),this.rotation=0,this.matrixAutoUpdate=!0,this.matrix=new ie,this.generateMipmaps=!0,this.premultiplyAlpha=!1,this.flipY=!0,this.unpackAlignment=4,typeof h=="string"?this.colorSpace=h:(xa("THREE.Texture: Property .encoding has been replaced by .colorSpace."),this.colorSpace=h===3001?nn:""),this.userData={},this.version=0,this.onUpdate=null,this.isRenderTargetTexture=!1,this.needsPMREMUpdate=!1}get image(){return this.source.data}set image(t=null){this.source.data=t}updateMatrix(){this.matrix.setUvTransform(this.offset.x,this.offset.y,this.repeat.x,this.repeat.y,this.rotation,this.center.x,this.center.y)}clone(){return new this.constructor().copy(this)}copy(t){return this.name=t.name,this.source=t.source,this.mipmaps=t.mipmaps.slice(0),this.mapping=t.mapping,this.channel=t.channel,this.wrapS=t.wrapS,this.wrapT=t.wrapT,this.magFilter=t.magFilter,this.minFilter=t.minFilter,this.anisotropy=t.anisotropy,this.format=t.format,this.internalFormat=t.internalFormat,this.type=t.type,this.offset.copy(t.offset),this.repeat.copy(t.repeat),this.center.copy(t.center),this.rotation=t.rotation,this.matrixAutoUpdate=t.matrixAutoUpdate,this.matrix.copy(t.matrix),this.generateMipmaps=t.generateMipmaps,this.premultiplyAlpha=t.premultiplyAlpha,this.flipY=t.flipY,this.unpackAlignment=t.unpackAlignment,this.colorSpace=t.colorSpace,this.userData=JSON.parse(JSON.stringify(t.userData)),this.needsUpdate=!0,this}toJSON(t){const e=t===void 0||typeof t=="string";if(!e&&t.textures[this.uuid]!==void 0)return t.textures[this.uuid];const n={metadata:{version:4.6,type:"Texture",generator:"Texture.toJSON"},uuid:this.uuid,name:this.name,image:this.source.toJSON(t).uuid,mapping:this.mapping,channel:this.channel,repeat:[this.repeat.x,this.repeat.y],offset:[this.offset.x,this.offset.y],center:[this.center.x,this.center.y],rotation:this.rotation,wrap:[this.wrapS,this.wrapT],format:this.format,internalFormat:this.internalFormat,type:this.type,colorSpace:this.colorSpace,minFilter:this.minFilter,magFilter:this.magFilter,anisotropy:this.anisotropy,flipY:this.flipY,generateMipmaps:this.generateMipmaps,premultiplyAlpha:this.premultiplyAlpha,unpackAlignment:this.unpackAlignment};return Object.keys(this.userData).length>0&&(n.userData=this.userData),e||(t.textures[this.uuid]=n),n}dispose(){this.dispatchEvent({type:"dispose"})}transformUv(t){if(this.mapping!==300)return t;if(t.applyMatrix3(this.matrix),t.x<0||t.x>1)switch(this.wrapS){case 1e3:t.x=t.x-Math.floor(t.x);break;case 1001:t.x=t.x<0?0:1;break;case 1002:Math.abs(Math.floor(t.x)%2)===1?t.x=Math.ceil(t.x)-t.x:t.x=t.x-Math.floor(t.x);break}if(t.y<0||t.y>1)switch(this.wrapT){case 1e3:t.y=t.y-Math.floor(t.y);break;case 1001:t.y=t.y<0?0:1;break;case 1002:Math.abs(Math.floor(t.y)%2)===1?t.y=Math.ceil(t.y)-t.y:t.y=t.y-Math.floor(t.y);break}return this.flipY&&(t.y=1-t.y),t}set needsUpdate(t){t===!0&&(this.version++,this.source.needsUpdate=!0)}get encoding(){return xa("THREE.Texture: Property .encoding has been replaced by .colorSpace."),this.colorSpace===nn?3001:3e3}set encoding(t){xa("THREE.Texture: Property .encoding has been replaced by .colorSpace."),this.colorSpace=t===3001?nn:""}}Nn.DEFAULT_IMAGE=null;Nn.DEFAULT_MAPPING=300;Nn.DEFAULT_ANISOTROPY=1;class Ke{constructor(t=0,e=0,n=0,i=1){Ke.prototype.isVector4=!0,this.x=t,this.y=e,this.z=n,this.w=i}get width(){return this.z}set width(t){this.z=t}get height(){return this.w}set height(t){this.w=t}set(t,e,n,i){return this.x=t,this.y=e,this.z=n,this.w=i,this}setScalar(t){return this.x=t,this.y=t,this.z=t,this.w=t,this}setX(t){return this.x=t,this}setY(t){return this.y=t,this}setZ(t){return this.z=t,this}setW(t){return this.w=t,this}setComponent(t,e){switch(t){case 0:this.x=e;break;case 1:this.y=e;break;case 2:this.z=e;break;case 3:this.w=e;break;default:throw new Error("index is out of range: "+t)}return this}getComponent(t){switch(t){case 0:return this.x;case 1:return this.y;case 2:return this.z;case 3:return this.w;default:throw new Error("index is out of range: "+t)}}clone(){return new this.constructor(this.x,this.y,this.z,this.w)}copy(t){return this.x=t.x,this.y=t.y,this.z=t.z,this.w=t.w!==void 0?t.w:1,this}add(t){return this.x+=t.x,this.y+=t.y,this.z+=t.z,this.w+=t.w,this}addScalar(t){return this.x+=t,this.y+=t,this.z+=t,this.w+=t,this}addVectors(t,e){return this.x=t.x+e.x,this.y=t.y+e.y,this.z=t.z+e.z,this.w=t.w+e.w,this}addScaledVector(t,e){return this.x+=t.x*e,this.y+=t.y*e,this.z+=t.z*e,this.w+=t.w*e,this}sub(t){return this.x-=t.x,this.y-=t.y,this.z-=t.z,this.w-=t.w,this}subScalar(t){return this.x-=t,this.y-=t,this.z-=t,this.w-=t,this}subVectors(t,e){return this.x=t.x-e.x,this.y=t.y-e.y,this.z=t.z-e.z,this.w=t.w-e.w,this}multiply(t){return this.x*=t.x,this.y*=t.y,this.z*=t.z,this.w*=t.w,this}multiplyScalar(t){return this.x*=t,this.y*=t,this.z*=t,this.w*=t,this}applyMatrix4(t){const e=this.x,n=this.y,i=this.z,s=this.w,a=t.elements;return this.x=a[0]*e+a[4]*n+a[8]*i+a[12]*s,this.y=a[1]*e+a[5]*n+a[9]*i+a[13]*s,this.z=a[2]*e+a[6]*n+a[10]*i+a[14]*s,this.w=a[3]*e+a[7]*n+a[11]*i+a[15]*s,this}divideScalar(t){return this.multiplyScalar(1/t)}setAxisAngleFromQuaternion(t){this.w=2*Math.acos(t.w);const e=Math.sqrt(1-t.w*t.w);return e<1e-4?(this.x=1,this.y=0,this.z=0):(this.x=t.x/e,this.y=t.y/e,this.z=t.z/e),this}setAxisAngleFromRotationMatrix(t){let e,n,i,s;const c=t.elements,l=c[0],h=c[4],u=c[8],d=c[1],f=c[5],m=c[9],x=c[2],g=c[6],p=c[10];if(Math.abs(h-d)<.01&&Math.abs(u-x)<.01&&Math.abs(m-g)<.01){if(Math.abs(h+d)<.1&&Math.abs(u+x)<.1&&Math.abs(m+g)<.1&&Math.abs(l+f+p-3)<.1)return this.set(1,0,0,0),this;e=Math.PI;const v=(l+1)/2,S=(f+1)/2,M=(p+1)/2,y=(h+d)/4,w=(u+x)/4,T=(m+g)/4;return v>S&&v>M?v<.01?(n=0,i=.707106781,s=.707106781):(n=Math.sqrt(v),i=y/n,s=w/n):S>M?S<.01?(n=.707106781,i=0,s=.707106781):(i=Math.sqrt(S),n=y/i,s=T/i):M<.01?(n=.707106781,i=.707106781,s=0):(s=Math.sqrt(M),n=w/s,i=T/s),this.set(n,i,s,e),this}let _=Math.sqrt((g-m)*(g-m)+(u-x)*(u-x)+(d-h)*(d-h));return Math.abs(_)<.001&&(_=1),this.x=(g-m)/_,this.y=(u-x)/_,this.z=(d-h)/_,this.w=Math.acos((l+f+p-1)/2),this}min(t){return this.x=Math.min(this.x,t.x),this.y=Math.min(this.y,t.y),this.z=Math.min(this.z,t.z),this.w=Math.min(this.w,t.w),this}max(t){return this.x=Math.max(this.x,t.x),this.y=Math.max(this.y,t.y),this.z=Math.max(this.z,t.z),this.w=Math.max(this.w,t.w),this}clamp(t,e){return this.x=Math.max(t.x,Math.min(e.x,this.x)),this.y=Math.max(t.y,Math.min(e.y,this.y)),this.z=Math.max(t.z,Math.min(e.z,this.z)),this.w=Math.max(t.w,Math.min(e.w,this.w)),this}clampScalar(t,e){return this.x=Math.max(t,Math.min(e,this.x)),this.y=Math.max(t,Math.min(e,this.y)),this.z=Math.max(t,Math.min(e,this.z)),this.w=Math.max(t,Math.min(e,this.w)),this}clampLength(t,e){const n=this.length();return this.divideScalar(n||1).multiplyScalar(Math.max(t,Math.min(e,n)))}floor(){return this.x=Math.floor(this.x),this.y=Math.floor(this.y),this.z=Math.floor(this.z),this.w=Math.floor(this.w),this}ceil(){return this.x=Math.ceil(this.x),this.y=Math.ceil(this.y),this.z=Math.ceil(this.z),this.w=Math.ceil(this.w),this}round(){return this.x=Math.round(this.x),this.y=Math.round(this.y),this.z=Math.round(this.z),this.w=Math.round(this.w),this}roundToZero(){return this.x=Math.trunc(this.x),this.y=Math.trunc(this.y),this.z=Math.trunc(this.z),this.w=Math.trunc(this.w),this}negate(){return this.x=-this.x,this.y=-this.y,this.z=-this.z,this.w=-this.w,this}dot(t){return this.x*t.x+this.y*t.y+this.z*t.z+this.w*t.w}lengthSq(){return this.x*this.x+this.y*this.y+this.z*this.z+this.w*this.w}length(){return Math.sqrt(this.x*this.x+this.y*this.y+this.z*this.z+this.w*this.w)}manhattanLength(){return Math.abs(this.x)+Math.abs(this.y)+Math.abs(this.z)+Math.abs(this.w)}normalize(){return this.divideScalar(this.length()||1)}setLength(t){return this.normalize().multiplyScalar(t)}lerp(t,e){return this.x+=(t.x-this.x)*e,this.y+=(t.y-this.y)*e,this.z+=(t.z-this.z)*e,this.w+=(t.w-this.w)*e,this}lerpVectors(t,e,n){return this.x=t.x+(e.x-t.x)*n,this.y=t.y+(e.y-t.y)*n,this.z=t.z+(e.z-t.z)*n,this.w=t.w+(e.w-t.w)*n,this}equals(t){return t.x===this.x&&t.y===this.y&&t.z===this.z&&t.w===this.w}fromArray(t,e=0){return this.x=t[e],this.y=t[e+1],this.z=t[e+2],this.w=t[e+3],this}toArray(t=[],e=0){return t[e]=this.x,t[e+1]=this.y,t[e+2]=this.z,t[e+3]=this.w,t}fromBufferAttribute(t,e){return this.x=t.getX(e),this.y=t.getY(e),this.z=t.getZ(e),this.w=t.getW(e),this}random(){return this.x=Math.random(),this.y=Math.random(),this.z=Math.random(),this.w=Math.random(),this}*[Symbol.iterator](){yield this.x,yield this.y,yield this.z,yield this.w}}class gm extends Sr{constructor(t=1,e=1,n={}){super(),this.isRenderTarget=!0,this.width=t,this.height=e,this.depth=1,this.scissor=new Ke(0,0,t,e),this.scissorTest=!1,this.viewport=new Ke(0,0,t,e);const i={width:t,height:e,depth:1};n.encoding!==void 0&&(xa("THREE.WebGLRenderTarget: option.encoding has been replaced by option.colorSpace."),n.colorSpace=n.encoding===3001?nn:""),n=Object.assign({generateMipmaps:!1,internalFormat:null,minFilter:1006,depthBuffer:!0,stencilBuffer:!1,depthTexture:null,samples:0},n),this.texture=new Nn(i,n.mapping,n.wrapS,n.wrapT,n.magFilter,n.minFilter,n.format,n.type,n.anisotropy,n.colorSpace),this.texture.isRenderTargetTexture=!0,this.texture.flipY=!1,this.texture.generateMipmaps=n.generateMipmaps,this.texture.internalFormat=n.internalFormat,this.depthBuffer=n.depthBuffer,this.stencilBuffer=n.stencilBuffer,this.depthTexture=n.depthTexture,this.samples=n.samples}setSize(t,e,n=1){(this.width!==t||this.height!==e||this.depth!==n)&&(this.width=t,this.height=e,this.depth=n,this.texture.image.width=t,this.texture.image.height=e,this.texture.image.depth=n,this.dispose()),this.viewport.set(0,0,t,e),this.scissor.set(0,0,t,e)}clone(){return new this.constructor().copy(this)}copy(t){this.width=t.width,this.height=t.height,this.depth=t.depth,this.scissor.copy(t.scissor),this.scissorTest=t.scissorTest,this.viewport.copy(t.viewport),this.texture=t.texture.clone(),this.texture.isRenderTargetTexture=!0;const e=Object.assign({},t.texture.image);return this.texture.source=new bf(e),this.depthBuffer=t.depthBuffer,this.stencilBuffer=t.stencilBuffer,t.depthTexture!==null&&(this.depthTexture=t.depthTexture.clone()),this.samples=t.samples,this}dispose(){this.dispatchEvent({type:"dispose"})}}class Qe extends gm{constructor(t=1,e=1,n={}){super(t,e,n),this.isWebGLRenderTarget=!0}}class Rf extends Nn{constructor(t=null,e=1,n=1,i=1){super(null),this.isDataArrayTexture=!0,this.image={data:t,width:e,height:n,depth:i},this.magFilter=1003,this.minFilter=1003,this.wrapR=1001,this.generateMipmaps=!1,this.flipY=!1,this.unpackAlignment=1}}class xm extends Nn{constructor(t=null,e=1,n=1,i=1){super(null),this.isData3DTexture=!0,this.image={data:t,width:e,height:n,depth:i},this.magFilter=1003,this.minFilter=1003,this.wrapR=1001,this.generateMipmaps=!1,this.flipY=!1,this.unpackAlignment=1}}class Fn{constructor(t=0,e=0,n=0,i=1){this.isQuaternion=!0,this._x=t,this._y=e,this._z=n,this._w=i}static slerpFlat(t,e,n,i,s,a,o){let c=n[i+0],l=n[i+1],h=n[i+2],u=n[i+3];const d=s[a+0],f=s[a+1],m=s[a+2],x=s[a+3];if(o===0){t[e+0]=c,t[e+1]=l,t[e+2]=h,t[e+3]=u;return}if(o===1){t[e+0]=d,t[e+1]=f,t[e+2]=m,t[e+3]=x;return}if(u!==x||c!==d||l!==f||h!==m){let g=1-o;const p=c*d+l*f+h*m+u*x,_=p>=0?1:-1,v=1-p*p;if(v>Number.EPSILON){const M=Math.sqrt(v),y=Math.atan2(M,p*_);g=Math.sin(g*y)/M,o=Math.sin(o*y)/M}const S=o*_;if(c=c*g+d*S,l=l*g+f*S,h=h*g+m*S,u=u*g+x*S,g===1-o){const M=1/Math.sqrt(c*c+l*l+h*h+u*u);c*=M,l*=M,h*=M,u*=M}}t[e]=c,t[e+1]=l,t[e+2]=h,t[e+3]=u}static multiplyQuaternionsFlat(t,e,n,i,s,a){const o=n[i],c=n[i+1],l=n[i+2],h=n[i+3],u=s[a],d=s[a+1],f=s[a+2],m=s[a+3];return t[e]=o*m+h*u+c*f-l*d,t[e+1]=c*m+h*d+l*u-o*f,t[e+2]=l*m+h*f+o*d-c*u,t[e+3]=h*m-o*u-c*d-l*f,t}get x(){return this._x}set x(t){this._x=t,this._onChangeCallback()}get y(){return this._y}set y(t){this._y=t,this._onChangeCallback()}get z(){return this._z}set z(t){this._z=t,this._onChangeCallback()}get w(){return this._w}set w(t){this._w=t,this._onChangeCallback()}set(t,e,n,i){return this._x=t,this._y=e,this._z=n,this._w=i,this._onChangeCallback(),this}clone(){return new this.constructor(this._x,this._y,this._z,this._w)}copy(t){return this._x=t.x,this._y=t.y,this._z=t.z,this._w=t.w,this._onChangeCallback(),this}setFromEuler(t,e=!0){const n=t._x,i=t._y,s=t._z,a=t._order,o=Math.cos,c=Math.sin,l=o(n/2),h=o(i/2),u=o(s/2),d=c(n/2),f=c(i/2),m=c(s/2);switch(a){case"XYZ":this._x=d*h*u+l*f*m,this._y=l*f*u-d*h*m,this._z=l*h*m+d*f*u,this._w=l*h*u-d*f*m;break;case"YXZ":this._x=d*h*u+l*f*m,this._y=l*f*u-d*h*m,this._z=l*h*m-d*f*u,this._w=l*h*u+d*f*m;break;case"ZXY":this._x=d*h*u-l*f*m,this._y=l*f*u+d*h*m,this._z=l*h*m+d*f*u,this._w=l*h*u-d*f*m;break;case"ZYX":this._x=d*h*u-l*f*m,this._y=l*f*u+d*h*m,this._z=l*h*m-d*f*u,this._w=l*h*u+d*f*m;break;case"YZX":this._x=d*h*u+l*f*m,this._y=l*f*u+d*h*m,this._z=l*h*m-d*f*u,this._w=l*h*u-d*f*m;break;case"XZY":this._x=d*h*u-l*f*m,this._y=l*f*u-d*h*m,this._z=l*h*m+d*f*u,this._w=l*h*u+d*f*m;break;default:console.warn("THREE.Quaternion: .setFromEuler() encountered an unknown order: "+a)}return e===!0&&this._onChangeCallback(),this}setFromAxisAngle(t,e){const n=e/2,i=Math.sin(n);return this._x=t.x*i,this._y=t.y*i,this._z=t.z*i,this._w=Math.cos(n),this._onChangeCallback(),this}setFromRotationMatrix(t){const e=t.elements,n=e[0],i=e[4],s=e[8],a=e[1],o=e[5],c=e[9],l=e[2],h=e[6],u=e[10],d=n+o+u;if(d>0){const f=.5/Math.sqrt(d+1);this._w=.25/f,this._x=(h-c)*f,this._y=(s-l)*f,this._z=(a-i)*f}else if(n>o&&n>u){const f=2*Math.sqrt(1+n-o-u);this._w=(h-c)/f,this._x=.25*f,this._y=(i+a)/f,this._z=(s+l)/f}else if(o>u){const f=2*Math.sqrt(1+o-n-u);this._w=(s-l)/f,this._x=(i+a)/f,this._y=.25*f,this._z=(c+h)/f}else{const f=2*Math.sqrt(1+u-n-o);this._w=(a-i)/f,this._x=(s+l)/f,this._y=(c+h)/f,this._z=.25*f}return this._onChangeCallback(),this}setFromUnitVectors(t,e){let n=t.dot(e)+1;return n<Number.EPSILON?(n=0,Math.abs(t.x)>Math.abs(t.z)?(this._x=-t.y,this._y=t.x,this._z=0,this._w=n):(this._x=0,this._y=-t.z,this._z=t.y,this._w=n)):(this._x=t.y*e.z-t.z*e.y,this._y=t.z*e.x-t.x*e.z,this._z=t.x*e.y-t.y*e.x,this._w=n),this.normalize()}angleTo(t){return 2*Math.acos(Math.abs(ke(this.dot(t),-1,1)))}rotateTowards(t,e){const n=this.angleTo(t);if(n===0)return this;const i=Math.min(1,e/n);return this.slerp(t,i),this}identity(){return this.set(0,0,0,1)}invert(){return this.conjugate()}conjugate(){return this._x*=-1,this._y*=-1,this._z*=-1,this._onChangeCallback(),this}dot(t){return this._x*t._x+this._y*t._y+this._z*t._z+this._w*t._w}lengthSq(){return this._x*this._x+this._y*this._y+this._z*this._z+this._w*this._w}length(){return Math.sqrt(this._x*this._x+this._y*this._y+this._z*this._z+this._w*this._w)}normalize(){let t=this.length();return t===0?(this._x=0,this._y=0,this._z=0,this._w=1):(t=1/t,this._x=this._x*t,this._y=this._y*t,this._z=this._z*t,this._w=this._w*t),this._onChangeCallback(),this}multiply(t){return this.multiplyQuaternions(this,t)}premultiply(t){return this.multiplyQuaternions(t,this)}multiplyQuaternions(t,e){const n=t._x,i=t._y,s=t._z,a=t._w,o=e._x,c=e._y,l=e._z,h=e._w;return this._x=n*h+a*o+i*l-s*c,this._y=i*h+a*c+s*o-n*l,this._z=s*h+a*l+n*c-i*o,this._w=a*h-n*o-i*c-s*l,this._onChangeCallback(),this}slerp(t,e){if(e===0)return this;if(e===1)return this.copy(t);const n=this._x,i=this._y,s=this._z,a=this._w;let o=a*t._w+n*t._x+i*t._y+s*t._z;if(o<0?(this._w=-t._w,this._x=-t._x,this._y=-t._y,this._z=-t._z,o=-o):this.copy(t),o>=1)return this._w=a,this._x=n,this._y=i,this._z=s,this;const c=1-o*o;if(c<=Number.EPSILON){const f=1-e;return this._w=f*a+e*this._w,this._x=f*n+e*this._x,this._y=f*i+e*this._y,this._z=f*s+e*this._z,this.normalize(),this}const l=Math.sqrt(c),h=Math.atan2(l,o),u=Math.sin((1-e)*h)/l,d=Math.sin(e*h)/l;return this._w=a*u+this._w*d,this._x=n*u+this._x*d,this._y=i*u+this._y*d,this._z=s*u+this._z*d,this._onChangeCallback(),this}slerpQuaternions(t,e,n){return this.copy(t).slerp(e,n)}random(){const t=Math.random(),e=Math.sqrt(1-t),n=Math.sqrt(t),i=2*Math.PI*Math.random(),s=2*Math.PI*Math.random();return this.set(e*Math.cos(i),n*Math.sin(s),n*Math.cos(s),e*Math.sin(i))}equals(t){return t._x===this._x&&t._y===this._y&&t._z===this._z&&t._w===this._w}fromArray(t,e=0){return this._x=t[e],this._y=t[e+1],this._z=t[e+2],this._w=t[e+3],this._onChangeCallback(),this}toArray(t=[],e=0){return t[e]=this._x,t[e+1]=this._y,t[e+2]=this._z,t[e+3]=this._w,t}fromBufferAttribute(t,e){return this._x=t.getX(e),this._y=t.getY(e),this._z=t.getZ(e),this._w=t.getW(e),this._onChangeCallback(),this}toJSON(){return this.toArray()}_onChange(t){return this._onChangeCallback=t,this}_onChangeCallback(){}*[Symbol.iterator](){yield this._x,yield this._y,yield this._z,yield this._w}}class C{constructor(t=0,e=0,n=0){C.prototype.isVector3=!0,this.x=t,this.y=e,this.z=n}set(t,e,n){return n===void 0&&(n=this.z),this.x=t,this.y=e,this.z=n,this}setScalar(t){return this.x=t,this.y=t,this.z=t,this}setX(t){return this.x=t,this}setY(t){return this.y=t,this}setZ(t){return this.z=t,this}setComponent(t,e){switch(t){case 0:this.x=e;break;case 1:this.y=e;break;case 2:this.z=e;break;default:throw new Error("index is out of range: "+t)}return this}getComponent(t){switch(t){case 0:return this.x;case 1:return this.y;case 2:return this.z;default:throw new Error("index is out of range: "+t)}}clone(){return new this.constructor(this.x,this.y,this.z)}copy(t){return this.x=t.x,this.y=t.y,this.z=t.z,this}add(t){return this.x+=t.x,this.y+=t.y,this.z+=t.z,this}addScalar(t){return this.x+=t,this.y+=t,this.z+=t,this}addVectors(t,e){return this.x=t.x+e.x,this.y=t.y+e.y,this.z=t.z+e.z,this}addScaledVector(t,e){return this.x+=t.x*e,this.y+=t.y*e,this.z+=t.z*e,this}sub(t){return this.x-=t.x,this.y-=t.y,this.z-=t.z,this}subScalar(t){return this.x-=t,this.y-=t,this.z-=t,this}subVectors(t,e){return this.x=t.x-e.x,this.y=t.y-e.y,this.z=t.z-e.z,this}multiply(t){return this.x*=t.x,this.y*=t.y,this.z*=t.z,this}multiplyScalar(t){return this.x*=t,this.y*=t,this.z*=t,this}multiplyVectors(t,e){return this.x=t.x*e.x,this.y=t.y*e.y,this.z=t.z*e.z,this}applyEuler(t){return this.applyQuaternion(tu.setFromEuler(t))}applyAxisAngle(t,e){return this.applyQuaternion(tu.setFromAxisAngle(t,e))}applyMatrix3(t){const e=this.x,n=this.y,i=this.z,s=t.elements;return this.x=s[0]*e+s[3]*n+s[6]*i,this.y=s[1]*e+s[4]*n+s[7]*i,this.z=s[2]*e+s[5]*n+s[8]*i,this}applyNormalMatrix(t){return this.applyMatrix3(t).normalize()}applyMatrix4(t){const e=this.x,n=this.y,i=this.z,s=t.elements,a=1/(s[3]*e+s[7]*n+s[11]*i+s[15]);return this.x=(s[0]*e+s[4]*n+s[8]*i+s[12])*a,this.y=(s[1]*e+s[5]*n+s[9]*i+s[13])*a,this.z=(s[2]*e+s[6]*n+s[10]*i+s[14])*a,this}applyQuaternion(t){const e=this.x,n=this.y,i=this.z,s=t.x,a=t.y,o=t.z,c=t.w,l=2*(a*i-o*n),h=2*(o*e-s*i),u=2*(s*n-a*e);return this.x=e+c*l+a*u-o*h,this.y=n+c*h+o*l-s*u,this.z=i+c*u+s*h-a*l,this}project(t){return this.applyMatrix4(t.matrixWorldInverse).applyMatrix4(t.projectionMatrix)}unproject(t){return this.applyMatrix4(t.projectionMatrixInverse).applyMatrix4(t.matrixWorld)}transformDirection(t){const e=this.x,n=this.y,i=this.z,s=t.elements;return this.x=s[0]*e+s[4]*n+s[8]*i,this.y=s[1]*e+s[5]*n+s[9]*i,this.z=s[2]*e+s[6]*n+s[10]*i,this.normalize()}divide(t){return this.x/=t.x,this.y/=t.y,this.z/=t.z,this}divideScalar(t){return this.multiplyScalar(1/t)}min(t){return this.x=Math.min(this.x,t.x),this.y=Math.min(this.y,t.y),this.z=Math.min(this.z,t.z),this}max(t){return this.x=Math.max(this.x,t.x),this.y=Math.max(this.y,t.y),this.z=Math.max(this.z,t.z),this}clamp(t,e){return this.x=Math.max(t.x,Math.min(e.x,this.x)),this.y=Math.max(t.y,Math.min(e.y,this.y)),this.z=Math.max(t.z,Math.min(e.z,this.z)),this}clampScalar(t,e){return this.x=Math.max(t,Math.min(e,this.x)),this.y=Math.max(t,Math.min(e,this.y)),this.z=Math.max(t,Math.min(e,this.z)),this}clampLength(t,e){const n=this.length();return this.divideScalar(n||1).multiplyScalar(Math.max(t,Math.min(e,n)))}floor(){return this.x=Math.floor(this.x),this.y=Math.floor(this.y),this.z=Math.floor(this.z),this}ceil(){return this.x=Math.ceil(this.x),this.y=Math.ceil(this.y),this.z=Math.ceil(this.z),this}round(){return this.x=Math.round(this.x),this.y=Math.round(this.y),this.z=Math.round(this.z),this}roundToZero(){return this.x=Math.trunc(this.x),this.y=Math.trunc(this.y),this.z=Math.trunc(this.z),this}negate(){return this.x=-this.x,this.y=-this.y,this.z=-this.z,this}dot(t){return this.x*t.x+this.y*t.y+this.z*t.z}lengthSq(){return this.x*this.x+this.y*this.y+this.z*this.z}length(){return Math.sqrt(this.x*this.x+this.y*this.y+this.z*this.z)}manhattanLength(){return Math.abs(this.x)+Math.abs(this.y)+Math.abs(this.z)}normalize(){return this.divideScalar(this.length()||1)}setLength(t){return this.normalize().multiplyScalar(t)}lerp(t,e){return this.x+=(t.x-this.x)*e,this.y+=(t.y-this.y)*e,this.z+=(t.z-this.z)*e,this}lerpVectors(t,e,n){return this.x=t.x+(e.x-t.x)*n,this.y=t.y+(e.y-t.y)*n,this.z=t.z+(e.z-t.z)*n,this}cross(t){return this.crossVectors(this,t)}crossVectors(t,e){const n=t.x,i=t.y,s=t.z,a=e.x,o=e.y,c=e.z;return this.x=i*c-s*o,this.y=s*a-n*c,this.z=n*o-i*a,this}projectOnVector(t){const e=t.lengthSq();if(e===0)return this.set(0,0,0);const n=t.dot(this)/e;return this.copy(t).multiplyScalar(n)}projectOnPlane(t){return wc.copy(this).projectOnVector(t),this.sub(wc)}reflect(t){return this.sub(wc.copy(t).multiplyScalar(2*this.dot(t)))}angleTo(t){const e=Math.sqrt(this.lengthSq()*t.lengthSq());if(e===0)return Math.PI/2;const n=this.dot(t)/e;return Math.acos(ke(n,-1,1))}distanceTo(t){return Math.sqrt(this.distanceToSquared(t))}distanceToSquared(t){const e=this.x-t.x,n=this.y-t.y,i=this.z-t.z;return e*e+n*n+i*i}manhattanDistanceTo(t){return Math.abs(this.x-t.x)+Math.abs(this.y-t.y)+Math.abs(this.z-t.z)}setFromSpherical(t){return this.setFromSphericalCoords(t.radius,t.phi,t.theta)}setFromSphericalCoords(t,e,n){const i=Math.sin(e)*t;return this.x=i*Math.sin(n),this.y=Math.cos(e)*t,this.z=i*Math.cos(n),this}setFromCylindrical(t){return this.setFromCylindricalCoords(t.radius,t.theta,t.y)}setFromCylindricalCoords(t,e,n){return this.x=t*Math.sin(e),this.y=n,this.z=t*Math.cos(e),this}setFromMatrixPosition(t){const e=t.elements;return this.x=e[12],this.y=e[13],this.z=e[14],this}setFromMatrixScale(t){const e=this.setFromMatrixColumn(t,0).length(),n=this.setFromMatrixColumn(t,1).length(),i=this.setFromMatrixColumn(t,2).length();return this.x=e,this.y=n,this.z=i,this}setFromMatrixColumn(t,e){return this.fromArray(t.elements,e*4)}setFromMatrix3Column(t,e){return this.fromArray(t.elements,e*3)}setFromEuler(t){return this.x=t._x,this.y=t._y,this.z=t._z,this}setFromColor(t){return this.x=t.r,this.y=t.g,this.z=t.b,this}equals(t){return t.x===this.x&&t.y===this.y&&t.z===this.z}fromArray(t,e=0){return this.x=t[e],this.y=t[e+1],this.z=t[e+2],this}toArray(t=[],e=0){return t[e]=this.x,t[e+1]=this.y,t[e+2]=this.z,t}fromBufferAttribute(t,e){return this.x=t.getX(e),this.y=t.getY(e),this.z=t.getZ(e),this}random(){return this.x=Math.random(),this.y=Math.random(),this.z=Math.random(),this}randomDirection(){const t=(Math.random()-.5)*2,e=Math.random()*Math.PI*2,n=Math.sqrt(1-t**2);return this.x=n*Math.cos(e),this.y=n*Math.sin(e),this.z=t,this}*[Symbol.iterator](){yield this.x,yield this.y,yield this.z}}const wc=new C,tu=new Fn;class tn{constructor(t=new C(1/0,1/0,1/0),e=new C(-1/0,-1/0,-1/0)){this.isBox3=!0,this.min=t,this.max=e}set(t,e){return this.min.copy(t),this.max.copy(e),this}setFromArray(t){this.makeEmpty();for(let e=0,n=t.length;e<n;e+=3)this.expandByPoint(qn.fromArray(t,e));return this}setFromBufferAttribute(t){this.makeEmpty();for(let e=0,n=t.count;e<n;e++)this.expandByPoint(qn.fromBufferAttribute(t,e));return this}setFromPoints(t){this.makeEmpty();for(let e=0,n=t.length;e<n;e++)this.expandByPoint(t[e]);return this}setFromCenterAndSize(t,e){const n=qn.copy(e).multiplyScalar(.5);return this.min.copy(t).sub(n),this.max.copy(t).add(n),this}setFromObject(t,e=!1){return this.makeEmpty(),this.expandByObject(t,e)}clone(){return new this.constructor().copy(this)}copy(t){return this.min.copy(t.min),this.max.copy(t.max),this}makeEmpty(){return this.min.x=this.min.y=this.min.z=1/0,this.max.x=this.max.y=this.max.z=-1/0,this}isEmpty(){return this.max.x<this.min.x||this.max.y<this.min.y||this.max.z<this.min.z}getCenter(t){return this.isEmpty()?t.set(0,0,0):t.addVectors(this.min,this.max).multiplyScalar(.5)}getSize(t){return this.isEmpty()?t.set(0,0,0):t.subVectors(this.max,this.min)}expandByPoint(t){return this.min.min(t),this.max.max(t),this}expandByVector(t){return this.min.sub(t),this.max.add(t),this}expandByScalar(t){return this.min.addScalar(-t),this.max.addScalar(t),this}expandByObject(t,e=!1){t.updateWorldMatrix(!1,!1);const n=t.geometry;if(n!==void 0){const s=n.getAttribute("position");if(e===!0&&s!==void 0&&t.isInstancedMesh!==!0)for(let a=0,o=s.count;a<o;a++)t.isMesh===!0?t.getVertexPosition(a,qn):qn.fromBufferAttribute(s,a),qn.applyMatrix4(t.matrixWorld),this.expandByPoint(qn);else t.boundingBox!==void 0?(t.boundingBox===null&&t.computeBoundingBox(),Da.copy(t.boundingBox)):(n.boundingBox===null&&n.computeBoundingBox(),Da.copy(n.boundingBox)),Da.applyMatrix4(t.matrixWorld),this.union(Da)}const i=t.children;for(let s=0,a=i.length;s<a;s++)this.expandByObject(i[s],e);return this}containsPoint(t){return!(t.x<this.min.x||t.x>this.max.x||t.y<this.min.y||t.y>this.max.y||t.z<this.min.z||t.z>this.max.z)}containsBox(t){return this.min.x<=t.min.x&&t.max.x<=this.max.x&&this.min.y<=t.min.y&&t.max.y<=this.max.y&&this.min.z<=t.min.z&&t.max.z<=this.max.z}getParameter(t,e){return e.set((t.x-this.min.x)/(this.max.x-this.min.x),(t.y-this.min.y)/(this.max.y-this.min.y),(t.z-this.min.z)/(this.max.z-this.min.z))}intersectsBox(t){return!(t.max.x<this.min.x||t.min.x>this.max.x||t.max.y<this.min.y||t.min.y>this.max.y||t.max.z<this.min.z||t.min.z>this.max.z)}intersectsSphere(t){return this.clampPoint(t.center,qn),qn.distanceToSquared(t.center)<=t.radius*t.radius}intersectsPlane(t){let e,n;return t.normal.x>0?(e=t.normal.x*this.min.x,n=t.normal.x*this.max.x):(e=t.normal.x*this.max.x,n=t.normal.x*this.min.x),t.normal.y>0?(e+=t.normal.y*this.min.y,n+=t.normal.y*this.max.y):(e+=t.normal.y*this.max.y,n+=t.normal.y*this.min.y),t.normal.z>0?(e+=t.normal.z*this.min.z,n+=t.normal.z*this.max.z):(e+=t.normal.z*this.max.z,n+=t.normal.z*this.min.z),e<=-t.constant&&n>=-t.constant}intersectsTriangle(t){if(this.isEmpty())return!1;this.getCenter(Rr),Ia.subVectors(this.max,Rr),Rs.subVectors(t.a,Rr),Ps.subVectors(t.b,Rr),Ls.subVectors(t.c,Rr),zi.subVectors(Ps,Rs),Oi.subVectors(Ls,Ps),ss.subVectors(Rs,Ls);let e=[0,-zi.z,zi.y,0,-Oi.z,Oi.y,0,-ss.z,ss.y,zi.z,0,-zi.x,Oi.z,0,-Oi.x,ss.z,0,-ss.x,-zi.y,zi.x,0,-Oi.y,Oi.x,0,-ss.y,ss.x,0];return!Ac(e,Rs,Ps,Ls,Ia)||(e=[1,0,0,0,1,0,0,0,1],!Ac(e,Rs,Ps,Ls,Ia))?!1:(Na.crossVectors(zi,Oi),e=[Na.x,Na.y,Na.z],Ac(e,Rs,Ps,Ls,Ia))}clampPoint(t,e){return e.copy(t).clamp(this.min,this.max)}distanceToPoint(t){return this.clampPoint(t,qn).distanceTo(t)}getBoundingSphere(t){return this.isEmpty()?t.makeEmpty():(this.getCenter(t.center),t.radius=this.getSize(qn).length()*.5),t}intersect(t){return this.min.max(t.min),this.max.min(t.max),this.isEmpty()&&this.makeEmpty(),this}union(t){return this.min.min(t.min),this.max.max(t.max),this}applyMatrix4(t){return this.isEmpty()?this:(vi[0].set(this.min.x,this.min.y,this.min.z).applyMatrix4(t),vi[1].set(this.min.x,this.min.y,this.max.z).applyMatrix4(t),vi[2].set(this.min.x,this.max.y,this.min.z).applyMatrix4(t),vi[3].set(this.min.x,this.max.y,this.max.z).applyMatrix4(t),vi[4].set(this.max.x,this.min.y,this.min.z).applyMatrix4(t),vi[5].set(this.max.x,this.min.y,this.max.z).applyMatrix4(t),vi[6].set(this.max.x,this.max.y,this.min.z).applyMatrix4(t),vi[7].set(this.max.x,this.max.y,this.max.z).applyMatrix4(t),this.setFromPoints(vi),this)}translate(t){return this.min.add(t),this.max.add(t),this}equals(t){return t.min.equals(this.min)&&t.max.equals(this.max)}}const vi=[new C,new C,new C,new C,new C,new C,new C,new C],qn=new C,Da=new tn,Rs=new C,Ps=new C,Ls=new C,zi=new C,Oi=new C,ss=new C,Rr=new C,Ia=new C,Na=new C,rs=new C;function Ac(r,t,e,n,i){for(let s=0,a=r.length-3;s<=a;s+=3){rs.fromArray(r,s);const o=i.x*Math.abs(rs.x)+i.y*Math.abs(rs.y)+i.z*Math.abs(rs.z),c=t.dot(rs),l=e.dot(rs),h=n.dot(rs);if(Math.max(-Math.max(c,l,h),Math.min(c,l,h))>o)return!1}return!0}const _m=new tn,Pr=new C,Tc=new C;class ns{constructor(t=new C,e=-1){this.isSphere=!0,this.center=t,this.radius=e}set(t,e){return this.center.copy(t),this.radius=e,this}setFromPoints(t,e){const n=this.center;e!==void 0?n.copy(e):_m.setFromPoints(t).getCenter(n);let i=0;for(let s=0,a=t.length;s<a;s++)i=Math.max(i,n.distanceToSquared(t[s]));return this.radius=Math.sqrt(i),this}copy(t){return this.center.copy(t.center),this.radius=t.radius,this}isEmpty(){return this.radius<0}makeEmpty(){return this.center.set(0,0,0),this.radius=-1,this}containsPoint(t){return t.distanceToSquared(this.center)<=this.radius*this.radius}distanceToPoint(t){return t.distanceTo(this.center)-this.radius}intersectsSphere(t){const e=this.radius+t.radius;return t.center.distanceToSquared(this.center)<=e*e}intersectsBox(t){return t.intersectsSphere(this)}intersectsPlane(t){return Math.abs(t.distanceToPoint(this.center))<=this.radius}clampPoint(t,e){const n=this.center.distanceToSquared(t);return e.copy(t),n>this.radius*this.radius&&(e.sub(this.center).normalize(),e.multiplyScalar(this.radius).add(this.center)),e}getBoundingBox(t){return this.isEmpty()?(t.makeEmpty(),t):(t.set(this.center,this.center),t.expandByScalar(this.radius),t)}applyMatrix4(t){return this.center.applyMatrix4(t),this.radius=this.radius*t.getMaxScaleOnAxis(),this}translate(t){return this.center.add(t),this}expandByPoint(t){if(this.isEmpty())return this.center.copy(t),this.radius=0,this;Pr.subVectors(t,this.center);const e=Pr.lengthSq();if(e>this.radius*this.radius){const n=Math.sqrt(e),i=(n-this.radius)*.5;this.center.addScaledVector(Pr,i/n),this.radius+=i}return this}union(t){return t.isEmpty()?this:this.isEmpty()?(this.copy(t),this):(this.center.equals(t.center)===!0?this.radius=Math.max(this.radius,t.radius):(Tc.subVectors(t.center,this.center).setLength(t.radius),this.expandByPoint(Pr.copy(t.center).add(Tc)),this.expandByPoint(Pr.copy(t.center).sub(Tc))),this)}equals(t){return t.center.equals(this.center)&&t.radius===this.radius}clone(){return new this.constructor().copy(this)}}const Si=new C,Cc=new C,Fa=new C,Gi=new C,bc=new C,Ua=new C,Rc=new C;class Eh{constructor(t=new C,e=new C(0,0,-1)){this.origin=t,this.direction=e}set(t,e){return this.origin.copy(t),this.direction.copy(e),this}copy(t){return this.origin.copy(t.origin),this.direction.copy(t.direction),this}at(t,e){return e.copy(this.origin).addScaledVector(this.direction,t)}lookAt(t){return this.direction.copy(t).sub(this.origin).normalize(),this}recast(t){return this.origin.copy(this.at(t,Si)),this}closestPointToPoint(t,e){e.subVectors(t,this.origin);const n=e.dot(this.direction);return n<0?e.copy(this.origin):e.copy(this.origin).addScaledVector(this.direction,n)}distanceToPoint(t){return Math.sqrt(this.distanceSqToPoint(t))}distanceSqToPoint(t){const e=Si.subVectors(t,this.origin).dot(this.direction);return e<0?this.origin.distanceToSquared(t):(Si.copy(this.origin).addScaledVector(this.direction,e),Si.distanceToSquared(t))}distanceSqToSegment(t,e,n,i){Cc.copy(t).add(e).multiplyScalar(.5),Fa.copy(e).sub(t).normalize(),Gi.copy(this.origin).sub(Cc);const s=t.distanceTo(e)*.5,a=-this.direction.dot(Fa),o=Gi.dot(this.direction),c=-Gi.dot(Fa),l=Gi.lengthSq(),h=Math.abs(1-a*a);let u,d,f,m;if(h>0)if(u=a*c-o,d=a*o-c,m=s*h,u>=0)if(d>=-m)if(d<=m){const x=1/h;u*=x,d*=x,f=u*(u+a*d+2*o)+d*(a*u+d+2*c)+l}else d=s,u=Math.max(0,-(a*d+o)),f=-u*u+d*(d+2*c)+l;else d=-s,u=Math.max(0,-(a*d+o)),f=-u*u+d*(d+2*c)+l;else d<=-m?(u=Math.max(0,-(-a*s+o)),d=u>0?-s:Math.min(Math.max(-s,-c),s),f=-u*u+d*(d+2*c)+l):d<=m?(u=0,d=Math.min(Math.max(-s,-c),s),f=d*(d+2*c)+l):(u=Math.max(0,-(a*s+o)),d=u>0?s:Math.min(Math.max(-s,-c),s),f=-u*u+d*(d+2*c)+l);else d=a>0?-s:s,u=Math.max(0,-(a*d+o)),f=-u*u+d*(d+2*c)+l;return n&&n.copy(this.origin).addScaledVector(this.direction,u),i&&i.copy(Cc).addScaledVector(Fa,d),f}intersectSphere(t,e){Si.subVectors(t.center,this.origin);const n=Si.dot(this.direction),i=Si.dot(Si)-n*n,s=t.radius*t.radius;if(i>s)return null;const a=Math.sqrt(s-i),o=n-a,c=n+a;return c<0?null:o<0?this.at(c,e):this.at(o,e)}intersectsSphere(t){return this.distanceSqToPoint(t.center)<=t.radius*t.radius}distanceToPlane(t){const e=t.normal.dot(this.direction);if(e===0)return t.distanceToPoint(this.origin)===0?0:null;const n=-(this.origin.dot(t.normal)+t.constant)/e;return n>=0?n:null}intersectPlane(t,e){const n=this.distanceToPlane(t);return n===null?null:this.at(n,e)}intersectsPlane(t){const e=t.distanceToPoint(this.origin);return e===0||t.normal.dot(this.direction)*e<0}intersectBox(t,e){let n,i,s,a,o,c;const l=1/this.direction.x,h=1/this.direction.y,u=1/this.direction.z,d=this.origin;return l>=0?(n=(t.min.x-d.x)*l,i=(t.max.x-d.x)*l):(n=(t.max.x-d.x)*l,i=(t.min.x-d.x)*l),h>=0?(s=(t.min.y-d.y)*h,a=(t.max.y-d.y)*h):(s=(t.max.y-d.y)*h,a=(t.min.y-d.y)*h),n>a||s>i||((s>n||isNaN(n))&&(n=s),(a<i||isNaN(i))&&(i=a),u>=0?(o=(t.min.z-d.z)*u,c=(t.max.z-d.z)*u):(o=(t.max.z-d.z)*u,c=(t.min.z-d.z)*u),n>c||o>i)||((o>n||n!==n)&&(n=o),(c<i||i!==i)&&(i=c),i<0)?null:this.at(n>=0?n:i,e)}intersectsBox(t){return this.intersectBox(t,Si)!==null}intersectTriangle(t,e,n,i,s){bc.subVectors(e,t),Ua.subVectors(n,t),Rc.crossVectors(bc,Ua);let a=this.direction.dot(Rc),o;if(a>0){if(i)return null;o=1}else if(a<0)o=-1,a=-a;else return null;Gi.subVectors(this.origin,t);const c=o*this.direction.dot(Ua.crossVectors(Gi,Ua));if(c<0)return null;const l=o*this.direction.dot(bc.cross(Gi));if(l<0||c+l>a)return null;const h=-o*Gi.dot(Rc);return h<0?null:this.at(h/a,s)}applyMatrix4(t){return this.origin.applyMatrix4(t),this.direction.transformDirection(t),this}equals(t){return t.origin.equals(this.origin)&&t.direction.equals(this.direction)}clone(){return new this.constructor().copy(this)}}class re{constructor(t,e,n,i,s,a,o,c,l,h,u,d,f,m,x,g){re.prototype.isMatrix4=!0,this.elements=[1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1],t!==void 0&&this.set(t,e,n,i,s,a,o,c,l,h,u,d,f,m,x,g)}set(t,e,n,i,s,a,o,c,l,h,u,d,f,m,x,g){const p=this.elements;return p[0]=t,p[4]=e,p[8]=n,p[12]=i,p[1]=s,p[5]=a,p[9]=o,p[13]=c,p[2]=l,p[6]=h,p[10]=u,p[14]=d,p[3]=f,p[7]=m,p[11]=x,p[15]=g,this}identity(){return this.set(1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1),this}clone(){return new re().fromArray(this.elements)}copy(t){const e=this.elements,n=t.elements;return e[0]=n[0],e[1]=n[1],e[2]=n[2],e[3]=n[3],e[4]=n[4],e[5]=n[5],e[6]=n[6],e[7]=n[7],e[8]=n[8],e[9]=n[9],e[10]=n[10],e[11]=n[11],e[12]=n[12],e[13]=n[13],e[14]=n[14],e[15]=n[15],this}copyPosition(t){const e=this.elements,n=t.elements;return e[12]=n[12],e[13]=n[13],e[14]=n[14],this}setFromMatrix3(t){const e=t.elements;return this.set(e[0],e[3],e[6],0,e[1],e[4],e[7],0,e[2],e[5],e[8],0,0,0,0,1),this}extractBasis(t,e,n){return t.setFromMatrixColumn(this,0),e.setFromMatrixColumn(this,1),n.setFromMatrixColumn(this,2),this}makeBasis(t,e,n){return this.set(t.x,e.x,n.x,0,t.y,e.y,n.y,0,t.z,e.z,n.z,0,0,0,0,1),this}extractRotation(t){const e=this.elements,n=t.elements,i=1/Ds.setFromMatrixColumn(t,0).length(),s=1/Ds.setFromMatrixColumn(t,1).length(),a=1/Ds.setFromMatrixColumn(t,2).length();return e[0]=n[0]*i,e[1]=n[1]*i,e[2]=n[2]*i,e[3]=0,e[4]=n[4]*s,e[5]=n[5]*s,e[6]=n[6]*s,e[7]=0,e[8]=n[8]*a,e[9]=n[9]*a,e[10]=n[10]*a,e[11]=0,e[12]=0,e[13]=0,e[14]=0,e[15]=1,this}makeRotationFromEuler(t){const e=this.elements,n=t.x,i=t.y,s=t.z,a=Math.cos(n),o=Math.sin(n),c=Math.cos(i),l=Math.sin(i),h=Math.cos(s),u=Math.sin(s);if(t.order==="XYZ"){const d=a*h,f=a*u,m=o*h,x=o*u;e[0]=c*h,e[4]=-c*u,e[8]=l,e[1]=f+m*l,e[5]=d-x*l,e[9]=-o*c,e[2]=x-d*l,e[6]=m+f*l,e[10]=a*c}else if(t.order==="YXZ"){const d=c*h,f=c*u,m=l*h,x=l*u;e[0]=d+x*o,e[4]=m*o-f,e[8]=a*l,e[1]=a*u,e[5]=a*h,e[9]=-o,e[2]=f*o-m,e[6]=x+d*o,e[10]=a*c}else if(t.order==="ZXY"){const d=c*h,f=c*u,m=l*h,x=l*u;e[0]=d-x*o,e[4]=-a*u,e[8]=m+f*o,e[1]=f+m*o,e[5]=a*h,e[9]=x-d*o,e[2]=-a*l,e[6]=o,e[10]=a*c}else if(t.order==="ZYX"){const d=a*h,f=a*u,m=o*h,x=o*u;e[0]=c*h,e[4]=m*l-f,e[8]=d*l+x,e[1]=c*u,e[5]=x*l+d,e[9]=f*l-m,e[2]=-l,e[6]=o*c,e[10]=a*c}else if(t.order==="YZX"){const d=a*c,f=a*l,m=o*c,x=o*l;e[0]=c*h,e[4]=x-d*u,e[8]=m*u+f,e[1]=u,e[5]=a*h,e[9]=-o*h,e[2]=-l*h,e[6]=f*u+m,e[10]=d-x*u}else if(t.order==="XZY"){const d=a*c,f=a*l,m=o*c,x=o*l;e[0]=c*h,e[4]=-u,e[8]=l*h,e[1]=d*u+x,e[5]=a*h,e[9]=f*u-m,e[2]=m*u-f,e[6]=o*h,e[10]=x*u+d}return e[3]=0,e[7]=0,e[11]=0,e[12]=0,e[13]=0,e[14]=0,e[15]=1,this}makeRotationFromQuaternion(t){return this.compose(vm,t,Sm)}lookAt(t,e,n){const i=this.elements;return bn.subVectors(t,e),bn.lengthSq()===0&&(bn.z=1),bn.normalize(),Vi.crossVectors(n,bn),Vi.lengthSq()===0&&(Math.abs(n.z)===1?bn.x+=1e-4:bn.z+=1e-4,bn.normalize(),Vi.crossVectors(n,bn)),Vi.normalize(),Ba.crossVectors(bn,Vi),i[0]=Vi.x,i[4]=Ba.x,i[8]=bn.x,i[1]=Vi.y,i[5]=Ba.y,i[9]=bn.y,i[2]=Vi.z,i[6]=Ba.z,i[10]=bn.z,this}multiply(t){return this.multiplyMatrices(this,t)}premultiply(t){return this.multiplyMatrices(t,this)}multiplyMatrices(t,e){const n=t.elements,i=e.elements,s=this.elements,a=n[0],o=n[4],c=n[8],l=n[12],h=n[1],u=n[5],d=n[9],f=n[13],m=n[2],x=n[6],g=n[10],p=n[14],_=n[3],v=n[7],S=n[11],M=n[15],y=i[0],w=i[4],T=i[8],E=i[12],A=i[1],D=i[5],b=i[9],F=i[13],L=i[2],N=i[6],U=i[10],k=i[14],O=i[3],H=i[7],j=i[11],J=i[15];return s[0]=a*y+o*A+c*L+l*O,s[4]=a*w+o*D+c*N+l*H,s[8]=a*T+o*b+c*U+l*j,s[12]=a*E+o*F+c*k+l*J,s[1]=h*y+u*A+d*L+f*O,s[5]=h*w+u*D+d*N+f*H,s[9]=h*T+u*b+d*U+f*j,s[13]=h*E+u*F+d*k+f*J,s[2]=m*y+x*A+g*L+p*O,s[6]=m*w+x*D+g*N+p*H,s[10]=m*T+x*b+g*U+p*j,s[14]=m*E+x*F+g*k+p*J,s[3]=_*y+v*A+S*L+M*O,s[7]=_*w+v*D+S*N+M*H,s[11]=_*T+v*b+S*U+M*j,s[15]=_*E+v*F+S*k+M*J,this}multiplyScalar(t){const e=this.elements;return e[0]*=t,e[4]*=t,e[8]*=t,e[12]*=t,e[1]*=t,e[5]*=t,e[9]*=t,e[13]*=t,e[2]*=t,e[6]*=t,e[10]*=t,e[14]*=t,e[3]*=t,e[7]*=t,e[11]*=t,e[15]*=t,this}determinant(){const t=this.elements,e=t[0],n=t[4],i=t[8],s=t[12],a=t[1],o=t[5],c=t[9],l=t[13],h=t[2],u=t[6],d=t[10],f=t[14],m=t[3],x=t[7],g=t[11],p=t[15];return m*(+s*c*u-i*l*u-s*o*d+n*l*d+i*o*f-n*c*f)+x*(+e*c*f-e*l*d+s*a*d-i*a*f+i*l*h-s*c*h)+g*(+e*l*u-e*o*f-s*a*u+n*a*f+s*o*h-n*l*h)+p*(-i*o*h-e*c*u+e*o*d+i*a*u-n*a*d+n*c*h)}transpose(){const t=this.elements;let e;return e=t[1],t[1]=t[4],t[4]=e,e=t[2],t[2]=t[8],t[8]=e,e=t[6],t[6]=t[9],t[9]=e,e=t[3],t[3]=t[12],t[12]=e,e=t[7],t[7]=t[13],t[13]=e,e=t[11],t[11]=t[14],t[14]=e,this}setPosition(t,e,n){const i=this.elements;return t.isVector3?(i[12]=t.x,i[13]=t.y,i[14]=t.z):(i[12]=t,i[13]=e,i[14]=n),this}invert(){const t=this.elements,e=t[0],n=t[1],i=t[2],s=t[3],a=t[4],o=t[5],c=t[6],l=t[7],h=t[8],u=t[9],d=t[10],f=t[11],m=t[12],x=t[13],g=t[14],p=t[15],_=u*g*l-x*d*l+x*c*f-o*g*f-u*c*p+o*d*p,v=m*d*l-h*g*l-m*c*f+a*g*f+h*c*p-a*d*p,S=h*x*l-m*u*l+m*o*f-a*x*f-h*o*p+a*u*p,M=m*u*c-h*x*c-m*o*d+a*x*d+h*o*g-a*u*g,y=e*_+n*v+i*S+s*M;if(y===0)return this.set(0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0);const w=1/y;return t[0]=_*w,t[1]=(x*d*s-u*g*s-x*i*f+n*g*f+u*i*p-n*d*p)*w,t[2]=(o*g*s-x*c*s+x*i*l-n*g*l-o*i*p+n*c*p)*w,t[3]=(u*c*s-o*d*s-u*i*l+n*d*l+o*i*f-n*c*f)*w,t[4]=v*w,t[5]=(h*g*s-m*d*s+m*i*f-e*g*f-h*i*p+e*d*p)*w,t[6]=(m*c*s-a*g*s-m*i*l+e*g*l+a*i*p-e*c*p)*w,t[7]=(a*d*s-h*c*s+h*i*l-e*d*l-a*i*f+e*c*f)*w,t[8]=S*w,t[9]=(m*u*s-h*x*s-m*n*f+e*x*f+h*n*p-e*u*p)*w,t[10]=(a*x*s-m*o*s+m*n*l-e*x*l-a*n*p+e*o*p)*w,t[11]=(h*o*s-a*u*s-h*n*l+e*u*l+a*n*f-e*o*f)*w,t[12]=M*w,t[13]=(h*x*i-m*u*i+m*n*d-e*x*d-h*n*g+e*u*g)*w,t[14]=(m*o*i-a*x*i-m*n*c+e*x*c+a*n*g-e*o*g)*w,t[15]=(a*u*i-h*o*i+h*n*c-e*u*c-a*n*d+e*o*d)*w,this}scale(t){const e=this.elements,n=t.x,i=t.y,s=t.z;return e[0]*=n,e[4]*=i,e[8]*=s,e[1]*=n,e[5]*=i,e[9]*=s,e[2]*=n,e[6]*=i,e[10]*=s,e[3]*=n,e[7]*=i,e[11]*=s,this}getMaxScaleOnAxis(){const t=this.elements,e=t[0]*t[0]+t[1]*t[1]+t[2]*t[2],n=t[4]*t[4]+t[5]*t[5]+t[6]*t[6],i=t[8]*t[8]+t[9]*t[9]+t[10]*t[10];return Math.sqrt(Math.max(e,n,i))}makeTranslation(t,e,n){return t.isVector3?this.set(1,0,0,t.x,0,1,0,t.y,0,0,1,t.z,0,0,0,1):this.set(1,0,0,t,0,1,0,e,0,0,1,n,0,0,0,1),this}makeRotationX(t){const e=Math.cos(t),n=Math.sin(t);return this.set(1,0,0,0,0,e,-n,0,0,n,e,0,0,0,0,1),this}makeRotationY(t){const e=Math.cos(t),n=Math.sin(t);return this.set(e,0,n,0,0,1,0,0,-n,0,e,0,0,0,0,1),this}makeRotationZ(t){const e=Math.cos(t),n=Math.sin(t);return this.set(e,-n,0,0,n,e,0,0,0,0,1,0,0,0,0,1),this}makeRotationAxis(t,e){const n=Math.cos(e),i=Math.sin(e),s=1-n,a=t.x,o=t.y,c=t.z,l=s*a,h=s*o;return this.set(l*a+n,l*o-i*c,l*c+i*o,0,l*o+i*c,h*o+n,h*c-i*a,0,l*c-i*o,h*c+i*a,s*c*c+n,0,0,0,0,1),this}makeScale(t,e,n){return this.set(t,0,0,0,0,e,0,0,0,0,n,0,0,0,0,1),this}makeShear(t,e,n,i,s,a){return this.set(1,n,s,0,t,1,a,0,e,i,1,0,0,0,0,1),this}compose(t,e,n){const i=this.elements,s=e._x,a=e._y,o=e._z,c=e._w,l=s+s,h=a+a,u=o+o,d=s*l,f=s*h,m=s*u,x=a*h,g=a*u,p=o*u,_=c*l,v=c*h,S=c*u,M=n.x,y=n.y,w=n.z;return i[0]=(1-(x+p))*M,i[1]=(f+S)*M,i[2]=(m-v)*M,i[3]=0,i[4]=(f-S)*y,i[5]=(1-(d+p))*y,i[6]=(g+_)*y,i[7]=0,i[8]=(m+v)*w,i[9]=(g-_)*w,i[10]=(1-(d+x))*w,i[11]=0,i[12]=t.x,i[13]=t.y,i[14]=t.z,i[15]=1,this}decompose(t,e,n){const i=this.elements;let s=Ds.set(i[0],i[1],i[2]).length();const a=Ds.set(i[4],i[5],i[6]).length(),o=Ds.set(i[8],i[9],i[10]).length();this.determinant()<0&&(s=-s),t.x=i[12],t.y=i[13],t.z=i[14],Zn.copy(this);const l=1/s,h=1/a,u=1/o;return Zn.elements[0]*=l,Zn.elements[1]*=l,Zn.elements[2]*=l,Zn.elements[4]*=h,Zn.elements[5]*=h,Zn.elements[6]*=h,Zn.elements[8]*=u,Zn.elements[9]*=u,Zn.elements[10]*=u,e.setFromRotationMatrix(Zn),n.x=s,n.y=a,n.z=o,this}makePerspective(t,e,n,i,s,a,o=2e3){const c=this.elements,l=2*s/(e-t),h=2*s/(n-i),u=(e+t)/(e-t),d=(n+i)/(n-i);let f,m;if(o===2e3)f=-(a+s)/(a-s),m=-2*a*s/(a-s);else if(o===2001)f=-a/(a-s),m=-a*s/(a-s);else throw new Error("THREE.Matrix4.makePerspective(): Invalid coordinate system: "+o);return c[0]=l,c[4]=0,c[8]=u,c[12]=0,c[1]=0,c[5]=h,c[9]=d,c[13]=0,c[2]=0,c[6]=0,c[10]=f,c[14]=m,c[3]=0,c[7]=0,c[11]=-1,c[15]=0,this}makeOrthographic(t,e,n,i,s,a,o=2e3){const c=this.elements,l=1/(e-t),h=1/(n-i),u=1/(a-s),d=(e+t)*l,f=(n+i)*h;let m,x;if(o===2e3)m=(a+s)*u,x=-2*u;else if(o===2001)m=s*u,x=-1*u;else throw new Error("THREE.Matrix4.makeOrthographic(): Invalid coordinate system: "+o);return c[0]=2*l,c[4]=0,c[8]=0,c[12]=-d,c[1]=0,c[5]=2*h,c[9]=0,c[13]=-f,c[2]=0,c[6]=0,c[10]=x,c[14]=-m,c[3]=0,c[7]=0,c[11]=0,c[15]=1,this}equals(t){const e=this.elements,n=t.elements;for(let i=0;i<16;i++)if(e[i]!==n[i])return!1;return!0}fromArray(t,e=0){for(let n=0;n<16;n++)this.elements[n]=t[n+e];return this}toArray(t=[],e=0){const n=this.elements;return t[e]=n[0],t[e+1]=n[1],t[e+2]=n[2],t[e+3]=n[3],t[e+4]=n[4],t[e+5]=n[5],t[e+6]=n[6],t[e+7]=n[7],t[e+8]=n[8],t[e+9]=n[9],t[e+10]=n[10],t[e+11]=n[11],t[e+12]=n[12],t[e+13]=n[13],t[e+14]=n[14],t[e+15]=n[15],t}}const Ds=new C,Zn=new re,vm=new C(0,0,0),Sm=new C(1,1,1),Vi=new C,Ba=new C,bn=new C,eu=new re,nu=new Fn;class cn{constructor(t=0,e=0,n=0,i=cn.DEFAULT_ORDER){this.isEuler=!0,this._x=t,this._y=e,this._z=n,this._order=i}get x(){return this._x}set x(t){this._x=t,this._onChangeCallback()}get y(){return this._y}set y(t){this._y=t,this._onChangeCallback()}get z(){return this._z}set z(t){this._z=t,this._onChangeCallback()}get order(){return this._order}set order(t){this._order=t,this._onChangeCallback()}set(t,e,n,i=this._order){return this._x=t,this._y=e,this._z=n,this._order=i,this._onChangeCallback(),this}clone(){return new this.constructor(this._x,this._y,this._z,this._order)}copy(t){return this._x=t._x,this._y=t._y,this._z=t._z,this._order=t._order,this._onChangeCallback(),this}setFromRotationMatrix(t,e=this._order,n=!0){const i=t.elements,s=i[0],a=i[4],o=i[8],c=i[1],l=i[5],h=i[9],u=i[2],d=i[6],f=i[10];switch(e){case"XYZ":this._y=Math.asin(ke(o,-1,1)),Math.abs(o)<.9999999?(this._x=Math.atan2(-h,f),this._z=Math.atan2(-a,s)):(this._x=Math.atan2(d,l),this._z=0);break;case"YXZ":this._x=Math.asin(-ke(h,-1,1)),Math.abs(h)<.9999999?(this._y=Math.atan2(o,f),this._z=Math.atan2(c,l)):(this._y=Math.atan2(-u,s),this._z=0);break;case"ZXY":this._x=Math.asin(ke(d,-1,1)),Math.abs(d)<.9999999?(this._y=Math.atan2(-u,f),this._z=Math.atan2(-a,l)):(this._y=0,this._z=Math.atan2(c,s));break;case"ZYX":this._y=Math.asin(-ke(u,-1,1)),Math.abs(u)<.9999999?(this._x=Math.atan2(d,f),this._z=Math.atan2(c,s)):(this._x=0,this._z=Math.atan2(-a,l));break;case"YZX":this._z=Math.asin(ke(c,-1,1)),Math.abs(c)<.9999999?(this._x=Math.atan2(-h,l),this._y=Math.atan2(-u,s)):(this._x=0,this._y=Math.atan2(o,f));break;case"XZY":this._z=Math.asin(-ke(a,-1,1)),Math.abs(a)<.9999999?(this._x=Math.atan2(d,l),this._y=Math.atan2(o,s)):(this._x=Math.atan2(-h,f),this._y=0);break;default:console.warn("THREE.Euler: .setFromRotationMatrix() encountered an unknown order: "+e)}return this._order=e,n===!0&&this._onChangeCallback(),this}setFromQuaternion(t,e,n){return eu.makeRotationFromQuaternion(t),this.setFromRotationMatrix(eu,e,n)}setFromVector3(t,e=this._order){return this.set(t.x,t.y,t.z,e)}reorder(t){return nu.setFromEuler(this),this.setFromQuaternion(nu,t)}equals(t){return t._x===this._x&&t._y===this._y&&t._z===this._z&&t._order===this._order}fromArray(t){return this._x=t[0],this._y=t[1],this._z=t[2],t[3]!==void 0&&(this._order=t[3]),this._onChangeCallback(),this}toArray(t=[],e=0){return t[e]=this._x,t[e+1]=this._y,t[e+2]=this._z,t[e+3]=this._order,t}_onChange(t){return this._onChangeCallback=t,this}_onChangeCallback(){}*[Symbol.iterator](){yield this._x,yield this._y,yield this._z,yield this._order}}cn.DEFAULT_ORDER="XYZ";class Pf{constructor(){this.mask=1}set(t){this.mask=(1<<t|0)>>>0}enable(t){this.mask|=1<<t|0}enableAll(){this.mask=-1}toggle(t){this.mask^=1<<t|0}disable(t){this.mask&=~(1<<t|0)}disableAll(){this.mask=0}test(t){return(this.mask&t.mask)!==0}isEnabled(t){return(this.mask&(1<<t|0))!==0}}let Mm=0;const iu=new C,Is=new Fn,Mi=new re,za=new C,Lr=new C,ym=new C,Em=new Fn,su=new C(1,0,0),ru=new C(0,1,0),au=new C(0,0,1),wm={type:"added"},Am={type:"removed"};class Je extends Sr{constructor(){super(),this.isObject3D=!0,Object.defineProperty(this,"id",{value:Mm++}),this.uuid=Mr(),this.name="",this.type="Object3D",this.parent=null,this.children=[],this.up=Je.DEFAULT_UP.clone();const t=new C,e=new cn,n=new Fn,i=new C(1,1,1);function s(){n.setFromEuler(e,!1)}function a(){e.setFromQuaternion(n,void 0,!1)}e._onChange(s),n._onChange(a),Object.defineProperties(this,{position:{configurable:!0,enumerable:!0,value:t},rotation:{configurable:!0,enumerable:!0,value:e},quaternion:{configurable:!0,enumerable:!0,value:n},scale:{configurable:!0,enumerable:!0,value:i},modelViewMatrix:{value:new re},normalMatrix:{value:new ie}}),this.matrix=new re,this.matrixWorld=new re,this.matrixAutoUpdate=Je.DEFAULT_MATRIX_AUTO_UPDATE,this.matrixWorldAutoUpdate=Je.DEFAULT_MATRIX_WORLD_AUTO_UPDATE,this.matrixWorldNeedsUpdate=!1,this.layers=new Pf,this.visible=!0,this.castShadow=!1,this.receiveShadow=!1,this.frustumCulled=!0,this.renderOrder=0,this.animations=[],this.userData={}}onBeforeShadow(){}onAfterShadow(){}onBeforeRender(){}onAfterRender(){}applyMatrix4(t){this.matrixAutoUpdate&&this.updateMatrix(),this.matrix.premultiply(t),this.matrix.decompose(this.position,this.quaternion,this.scale)}applyQuaternion(t){return this.quaternion.premultiply(t),this}setRotationFromAxisAngle(t,e){this.quaternion.setFromAxisAngle(t,e)}setRotationFromEuler(t){this.quaternion.setFromEuler(t,!0)}setRotationFromMatrix(t){this.quaternion.setFromRotationMatrix(t)}setRotationFromQuaternion(t){this.quaternion.copy(t)}rotateOnAxis(t,e){return Is.setFromAxisAngle(t,e),this.quaternion.multiply(Is),this}rotateOnWorldAxis(t,e){return Is.setFromAxisAngle(t,e),this.quaternion.premultiply(Is),this}rotateX(t){return this.rotateOnAxis(su,t)}rotateY(t){return this.rotateOnAxis(ru,t)}rotateZ(t){return this.rotateOnAxis(au,t)}translateOnAxis(t,e){return iu.copy(t).applyQuaternion(this.quaternion),this.position.add(iu.multiplyScalar(e)),this}translateX(t){return this.translateOnAxis(su,t)}translateY(t){return this.translateOnAxis(ru,t)}translateZ(t){return this.translateOnAxis(au,t)}localToWorld(t){return this.updateWorldMatrix(!0,!1),t.applyMatrix4(this.matrixWorld)}worldToLocal(t){return this.updateWorldMatrix(!0,!1),t.applyMatrix4(Mi.copy(this.matrixWorld).invert())}lookAt(t,e,n){t.isVector3?za.copy(t):za.set(t,e,n);const i=this.parent;this.updateWorldMatrix(!0,!1),Lr.setFromMatrixPosition(this.matrixWorld),this.isCamera||this.isLight?Mi.lookAt(Lr,za,this.up):Mi.lookAt(za,Lr,this.up),this.quaternion.setFromRotationMatrix(Mi),i&&(Mi.extractRotation(i.matrixWorld),Is.setFromRotationMatrix(Mi),this.quaternion.premultiply(Is.invert()))}add(t){if(arguments.length>1){for(let e=0;e<arguments.length;e++)this.add(arguments[e]);return this}return t===this?(console.error("THREE.Object3D.add: object can't be added as a child of itself.",t),this):(t&&t.isObject3D?(t.parent!==null&&t.parent.remove(t),t.parent=this,this.children.push(t),t.dispatchEvent(wm)):console.error("THREE.Object3D.add: object not an instance of THREE.Object3D.",t),this)}remove(t){if(arguments.length>1){for(let n=0;n<arguments.length;n++)this.remove(arguments[n]);return this}const e=this.children.indexOf(t);return e!==-1&&(t.parent=null,this.children.splice(e,1),t.dispatchEvent(Am)),this}removeFromParent(){const t=this.parent;return t!==null&&t.remove(this),this}clear(){return this.remove(...this.children)}attach(t){return this.updateWorldMatrix(!0,!1),Mi.copy(this.matrixWorld).invert(),t.parent!==null&&(t.parent.updateWorldMatrix(!0,!1),Mi.multiply(t.parent.matrixWorld)),t.applyMatrix4(Mi),this.add(t),t.updateWorldMatrix(!1,!0),this}getObjectById(t){return this.getObjectByProperty("id",t)}getObjectByName(t){return this.getObjectByProperty("name",t)}getObjectByProperty(t,e){if(this[t]===e)return this;for(let n=0,i=this.children.length;n<i;n++){const a=this.children[n].getObjectByProperty(t,e);if(a!==void 0)return a}}getObjectsByProperty(t,e,n=[]){this[t]===e&&n.push(this);const i=this.children;for(let s=0,a=i.length;s<a;s++)i[s].getObjectsByProperty(t,e,n);return n}getWorldPosition(t){return this.updateWorldMatrix(!0,!1),t.setFromMatrixPosition(this.matrixWorld)}getWorldQuaternion(t){return this.updateWorldMatrix(!0,!1),this.matrixWorld.decompose(Lr,t,ym),t}getWorldScale(t){return this.updateWorldMatrix(!0,!1),this.matrixWorld.decompose(Lr,Em,t),t}getWorldDirection(t){this.updateWorldMatrix(!0,!1);const e=this.matrixWorld.elements;return t.set(e[8],e[9],e[10]).normalize()}raycast(){}traverse(t){t(this);const e=this.children;for(let n=0,i=e.length;n<i;n++)e[n].traverse(t)}traverseVisible(t){if(this.visible===!1)return;t(this);const e=this.children;for(let n=0,i=e.length;n<i;n++)e[n].traverseVisible(t)}traverseAncestors(t){const e=this.parent;e!==null&&(t(e),e.traverseAncestors(t))}updateMatrix(){this.matrix.compose(this.position,this.quaternion,this.scale),this.matrixWorldNeedsUpdate=!0}updateMatrixWorld(t){this.matrixAutoUpdate&&this.updateMatrix(),(this.matrixWorldNeedsUpdate||t)&&(this.parent===null?this.matrixWorld.copy(this.matrix):this.matrixWorld.multiplyMatrices(this.parent.matrixWorld,this.matrix),this.matrixWorldNeedsUpdate=!1,t=!0);const e=this.children;for(let n=0,i=e.length;n<i;n++){const s=e[n];(s.matrixWorldAutoUpdate===!0||t===!0)&&s.updateMatrixWorld(t)}}updateWorldMatrix(t,e){const n=this.parent;if(t===!0&&n!==null&&n.matrixWorldAutoUpdate===!0&&n.updateWorldMatrix(!0,!1),this.matrixAutoUpdate&&this.updateMatrix(),this.parent===null?this.matrixWorld.copy(this.matrix):this.matrixWorld.multiplyMatrices(this.parent.matrixWorld,this.matrix),e===!0){const i=this.children;for(let s=0,a=i.length;s<a;s++){const o=i[s];o.matrixWorldAutoUpdate===!0&&o.updateWorldMatrix(!1,!0)}}}toJSON(t){const e=t===void 0||typeof t=="string",n={};e&&(t={geometries:{},materials:{},textures:{},images:{},shapes:{},skeletons:{},animations:{},nodes:{}},n.metadata={version:4.6,type:"Object",generator:"Object3D.toJSON"});const i={};i.uuid=this.uuid,i.type=this.type,this.name!==""&&(i.name=this.name),this.castShadow===!0&&(i.castShadow=!0),this.receiveShadow===!0&&(i.receiveShadow=!0),this.visible===!1&&(i.visible=!1),this.frustumCulled===!1&&(i.frustumCulled=!1),this.renderOrder!==0&&(i.renderOrder=this.renderOrder),Object.keys(this.userData).length>0&&(i.userData=this.userData),i.layers=this.layers.mask,i.matrix=this.matrix.toArray(),i.up=this.up.toArray(),this.matrixAutoUpdate===!1&&(i.matrixAutoUpdate=!1),this.isInstancedMesh&&(i.type="InstancedMesh",i.count=this.count,i.instanceMatrix=this.instanceMatrix.toJSON(),this.instanceColor!==null&&(i.instanceColor=this.instanceColor.toJSON())),this.isBatchedMesh&&(i.type="BatchedMesh",i.perObjectFrustumCulled=this.perObjectFrustumCulled,i.sortObjects=this.sortObjects,i.drawRanges=this._drawRanges,i.reservedRanges=this._reservedRanges,i.visibility=this._visibility,i.active=this._active,i.bounds=this._bounds.map(o=>({boxInitialized:o.boxInitialized,boxMin:o.box.min.toArray(),boxMax:o.box.max.toArray(),sphereInitialized:o.sphereInitialized,sphereRadius:o.sphere.radius,sphereCenter:o.sphere.center.toArray()})),i.maxGeometryCount=this._maxGeometryCount,i.maxVertexCount=this._maxVertexCount,i.maxIndexCount=this._maxIndexCount,i.geometryInitialized=this._geometryInitialized,i.geometryCount=this._geometryCount,i.matricesTexture=this._matricesTexture.toJSON(t),this.boundingSphere!==null&&(i.boundingSphere={center:i.boundingSphere.center.toArray(),radius:i.boundingSphere.radius}),this.boundingBox!==null&&(i.boundingBox={min:i.boundingBox.min.toArray(),max:i.boundingBox.max.toArray()}));function s(o,c){return o[c.uuid]===void 0&&(o[c.uuid]=c.toJSON(t)),c.uuid}if(this.isScene)this.background&&(this.background.isColor?i.background=this.background.toJSON():this.background.isTexture&&(i.background=this.background.toJSON(t).uuid)),this.environment&&this.environment.isTexture&&this.environment.isRenderTargetTexture!==!0&&(i.environment=this.environment.toJSON(t).uuid);else if(this.isMesh||this.isLine||this.isPoints){i.geometry=s(t.geometries,this.geometry);const o=this.geometry.parameters;if(o!==void 0&&o.shapes!==void 0){const c=o.shapes;if(Array.isArray(c))for(let l=0,h=c.length;l<h;l++){const u=c[l];s(t.shapes,u)}else s(t.shapes,c)}}if(this.isSkinnedMesh&&(i.bindMode=this.bindMode,i.bindMatrix=this.bindMatrix.toArray(),this.skeleton!==void 0&&(s(t.skeletons,this.skeleton),i.skeleton=this.skeleton.uuid)),this.material!==void 0)if(Array.isArray(this.material)){const o=[];for(let c=0,l=this.material.length;c<l;c++)o.push(s(t.materials,this.material[c]));i.material=o}else i.material=s(t.materials,this.material);if(this.children.length>0){i.children=[];for(let o=0;o<this.children.length;o++)i.children.push(this.children[o].toJSON(t).object)}if(this.animations.length>0){i.animations=[];for(let o=0;o<this.animations.length;o++){const c=this.animations[o];i.animations.push(s(t.animations,c))}}if(e){const o=a(t.geometries),c=a(t.materials),l=a(t.textures),h=a(t.images),u=a(t.shapes),d=a(t.skeletons),f=a(t.animations),m=a(t.nodes);o.length>0&&(n.geometries=o),c.length>0&&(n.materials=c),l.length>0&&(n.textures=l),h.length>0&&(n.images=h),u.length>0&&(n.shapes=u),d.length>0&&(n.skeletons=d),f.length>0&&(n.animations=f),m.length>0&&(n.nodes=m)}return n.object=i,n;function a(o){const c=[];for(const l in o){const h=o[l];delete h.metadata,c.push(h)}return c}}clone(t){return new this.constructor().copy(this,t)}copy(t,e=!0){if(this.name=t.name,this.up.copy(t.up),this.position.copy(t.position),this.rotation.order=t.rotation.order,this.quaternion.copy(t.quaternion),this.scale.copy(t.scale),this.matrix.copy(t.matrix),this.matrixWorld.copy(t.matrixWorld),this.matrixAutoUpdate=t.matrixAutoUpdate,this.matrixWorldAutoUpdate=t.matrixWorldAutoUpdate,this.matrixWorldNeedsUpdate=t.matrixWorldNeedsUpdate,this.layers.mask=t.layers.mask,this.visible=t.visible,this.castShadow=t.castShadow,this.receiveShadow=t.receiveShadow,this.frustumCulled=t.frustumCulled,this.renderOrder=t.renderOrder,this.animations=t.animations.slice(),this.userData=JSON.parse(JSON.stringify(t.userData)),e===!0)for(let n=0;n<t.children.length;n++){const i=t.children[n];this.add(i.clone())}return this}}Je.DEFAULT_UP=new C(0,1,0);Je.DEFAULT_MATRIX_AUTO_UPDATE=!0;Je.DEFAULT_MATRIX_WORLD_AUTO_UPDATE=!0;const jn=new C,yi=new C,Pc=new C,Ei=new C,Ns=new C,Fs=new C,ou=new C,Lc=new C,Dc=new C,Ic=new C;let Oa=!1;class sn{constructor(t=new C,e=new C,n=new C){this.a=t,this.b=e,this.c=n}static getNormal(t,e,n,i){i.subVectors(n,e),jn.subVectors(t,e),i.cross(jn);const s=i.lengthSq();return s>0?i.multiplyScalar(1/Math.sqrt(s)):i.set(0,0,0)}static getBarycoord(t,e,n,i,s){jn.subVectors(i,e),yi.subVectors(n,e),Pc.subVectors(t,e);const a=jn.dot(jn),o=jn.dot(yi),c=jn.dot(Pc),l=yi.dot(yi),h=yi.dot(Pc),u=a*l-o*o;if(u===0)return s.set(0,0,0),null;const d=1/u,f=(l*c-o*h)*d,m=(a*h-o*c)*d;return s.set(1-f-m,m,f)}static containsPoint(t,e,n,i){return this.getBarycoord(t,e,n,i,Ei)===null?!1:Ei.x>=0&&Ei.y>=0&&Ei.x+Ei.y<=1}static getUV(t,e,n,i,s,a,o,c){return Oa===!1&&(console.warn("THREE.Triangle.getUV() has been renamed to THREE.Triangle.getInterpolation()."),Oa=!0),this.getInterpolation(t,e,n,i,s,a,o,c)}static getInterpolation(t,e,n,i,s,a,o,c){return this.getBarycoord(t,e,n,i,Ei)===null?(c.x=0,c.y=0,"z"in c&&(c.z=0),"w"in c&&(c.w=0),null):(c.setScalar(0),c.addScaledVector(s,Ei.x),c.addScaledVector(a,Ei.y),c.addScaledVector(o,Ei.z),c)}static isFrontFacing(t,e,n,i){return jn.subVectors(n,e),yi.subVectors(t,e),jn.cross(yi).dot(i)<0}set(t,e,n){return this.a.copy(t),this.b.copy(e),this.c.copy(n),this}setFromPointsAndIndices(t,e,n,i){return this.a.copy(t[e]),this.b.copy(t[n]),this.c.copy(t[i]),this}setFromAttributeAndIndices(t,e,n,i){return this.a.fromBufferAttribute(t,e),this.b.fromBufferAttribute(t,n),this.c.fromBufferAttribute(t,i),this}clone(){return new this.constructor().copy(this)}copy(t){return this.a.copy(t.a),this.b.copy(t.b),this.c.copy(t.c),this}getArea(){return jn.subVectors(this.c,this.b),yi.subVectors(this.a,this.b),jn.cross(yi).length()*.5}getMidpoint(t){return t.addVectors(this.a,this.b).add(this.c).multiplyScalar(1/3)}getNormal(t){return sn.getNormal(this.a,this.b,this.c,t)}getPlane(t){return t.setFromCoplanarPoints(this.a,this.b,this.c)}getBarycoord(t,e){return sn.getBarycoord(t,this.a,this.b,this.c,e)}getUV(t,e,n,i,s){return Oa===!1&&(console.warn("THREE.Triangle.getUV() has been renamed to THREE.Triangle.getInterpolation()."),Oa=!0),sn.getInterpolation(t,this.a,this.b,this.c,e,n,i,s)}getInterpolation(t,e,n,i,s){return sn.getInterpolation(t,this.a,this.b,this.c,e,n,i,s)}containsPoint(t){return sn.containsPoint(t,this.a,this.b,this.c)}isFrontFacing(t){return sn.isFrontFacing(this.a,this.b,this.c,t)}intersectsBox(t){return t.intersectsTriangle(this)}closestPointToPoint(t,e){const n=this.a,i=this.b,s=this.c;let a,o;Ns.subVectors(i,n),Fs.subVectors(s,n),Lc.subVectors(t,n);const c=Ns.dot(Lc),l=Fs.dot(Lc);if(c<=0&&l<=0)return e.copy(n);Dc.subVectors(t,i);const h=Ns.dot(Dc),u=Fs.dot(Dc);if(h>=0&&u<=h)return e.copy(i);const d=c*u-h*l;if(d<=0&&c>=0&&h<=0)return a=c/(c-h),e.copy(n).addScaledVector(Ns,a);Ic.subVectors(t,s);const f=Ns.dot(Ic),m=Fs.dot(Ic);if(m>=0&&f<=m)return e.copy(s);const x=f*l-c*m;if(x<=0&&l>=0&&m<=0)return o=l/(l-m),e.copy(n).addScaledVector(Fs,o);const g=h*m-f*u;if(g<=0&&u-h>=0&&f-m>=0)return ou.subVectors(s,i),o=(u-h)/(u-h+(f-m)),e.copy(i).addScaledVector(ou,o);const p=1/(g+x+d);return a=x*p,o=d*p,e.copy(n).addScaledVector(Ns,a).addScaledVector(Fs,o)}equals(t){return t.a.equals(this.a)&&t.b.equals(this.b)&&t.c.equals(this.c)}}const Lf={aliceblue:15792383,antiquewhite:16444375,aqua:65535,aquamarine:8388564,azure:15794175,beige:16119260,bisque:16770244,black:0,blanchedalmond:16772045,blue:255,blueviolet:9055202,brown:10824234,burlywood:14596231,cadetblue:6266528,chartreuse:8388352,chocolate:13789470,coral:16744272,cornflowerblue:6591981,cornsilk:16775388,crimson:14423100,cyan:65535,darkblue:139,darkcyan:35723,darkgoldenrod:12092939,darkgray:11119017,darkgreen:25600,darkgrey:11119017,darkkhaki:12433259,darkmagenta:9109643,darkolivegreen:5597999,darkorange:16747520,darkorchid:10040012,darkred:9109504,darksalmon:15308410,darkseagreen:9419919,darkslateblue:4734347,darkslategray:3100495,darkslategrey:3100495,darkturquoise:52945,darkviolet:9699539,deeppink:16716947,deepskyblue:49151,dimgray:6908265,dimgrey:6908265,dodgerblue:2003199,firebrick:11674146,floralwhite:16775920,forestgreen:2263842,fuchsia:16711935,gainsboro:14474460,ghostwhite:16316671,gold:16766720,goldenrod:14329120,gray:8421504,green:32768,greenyellow:11403055,grey:8421504,honeydew:15794160,hotpink:16738740,indianred:13458524,indigo:4915330,ivory:16777200,khaki:15787660,lavender:15132410,lavenderblush:16773365,lawngreen:8190976,lemonchiffon:16775885,lightblue:11393254,lightcoral:15761536,lightcyan:14745599,lightgoldenrodyellow:16448210,lightgray:13882323,lightgreen:9498256,lightgrey:13882323,lightpink:16758465,lightsalmon:16752762,lightseagreen:2142890,lightskyblue:8900346,lightslategray:7833753,lightslategrey:7833753,lightsteelblue:11584734,lightyellow:16777184,lime:65280,limegreen:3329330,linen:16445670,magenta:16711935,maroon:8388608,mediumaquamarine:6737322,mediumblue:205,mediumorchid:12211667,mediumpurple:9662683,mediumseagreen:3978097,mediumslateblue:8087790,mediumspringgreen:64154,mediumturquoise:4772300,mediumvioletred:13047173,midnightblue:1644912,mintcream:16121850,mistyrose:16770273,moccasin:16770229,navajowhite:16768685,navy:128,oldlace:16643558,olive:8421376,olivedrab:7048739,orange:16753920,orangered:16729344,orchid:14315734,palegoldenrod:15657130,palegreen:10025880,paleturquoise:11529966,palevioletred:14381203,papayawhip:16773077,peachpuff:16767673,peru:13468991,pink:16761035,plum:14524637,powderblue:11591910,purple:8388736,rebeccapurple:6697881,red:16711680,rosybrown:12357519,royalblue:4286945,saddlebrown:9127187,salmon:16416882,sandybrown:16032864,seagreen:3050327,seashell:16774638,sienna:10506797,silver:12632256,skyblue:8900331,slateblue:6970061,slategray:7372944,slategrey:7372944,snow:16775930,springgreen:65407,steelblue:4620980,tan:13808780,teal:32896,thistle:14204888,tomato:16737095,turquoise:4251856,violet:15631086,wheat:16113331,white:16777215,whitesmoke:16119285,yellow:16776960,yellowgreen:10145074},ki={h:0,s:0,l:0},Ga={h:0,s:0,l:0};function Nc(r,t,e){return e<0&&(e+=1),e>1&&(e-=1),e<1/6?r+(t-r)*6*e:e<1/2?t:e<2/3?r+(t-r)*6*(2/3-e):r}class Kt{constructor(t,e,n){return this.isColor=!0,this.r=1,this.g=1,this.b=1,this.set(t,e,n)}set(t,e,n){if(e===void 0&&n===void 0){const i=t;i&&i.isColor?this.copy(i):typeof i=="number"?this.setHex(i):typeof i=="string"&&this.setStyle(i)}else this.setRGB(t,e,n);return this}setScalar(t){return this.r=t,this.g=t,this.b=t,this}setHex(t,e=nn){return t=Math.floor(t),this.r=(t>>16&255)/255,this.g=(t>>8&255)/255,this.b=(t&255)/255,ge.toWorkingColorSpace(this,e),this}setRGB(t,e,n,i=ge.workingColorSpace){return this.r=t,this.g=e,this.b=n,ge.toWorkingColorSpace(this,i),this}setHSL(t,e,n,i=ge.workingColorSpace){if(t=yh(t,1),e=ke(e,0,1),n=ke(n,0,1),e===0)this.r=this.g=this.b=n;else{const s=n<=.5?n*(1+e):n+e-n*e,a=2*n-s;this.r=Nc(a,s,t+1/3),this.g=Nc(a,s,t),this.b=Nc(a,s,t-1/3)}return ge.toWorkingColorSpace(this,i),this}setStyle(t,e=nn){function n(s){s!==void 0&&parseFloat(s)<1&&console.warn("THREE.Color: Alpha component of "+t+" will be ignored.")}let i;if(i=/^(\w+)\(([^\)]*)\)/.exec(t)){let s;const a=i[1],o=i[2];switch(a){case"rgb":case"rgba":if(s=/^\s*(\d+)\s*,\s*(\d+)\s*,\s*(\d+)\s*(?:,\s*(\d*\.?\d+)\s*)?$/.exec(o))return n(s[4]),this.setRGB(Math.min(255,parseInt(s[1],10))/255,Math.min(255,parseInt(s[2],10))/255,Math.min(255,parseInt(s[3],10))/255,e);if(s=/^\s*(\d+)\%\s*,\s*(\d+)\%\s*,\s*(\d+)\%\s*(?:,\s*(\d*\.?\d+)\s*)?$/.exec(o))return n(s[4]),this.setRGB(Math.min(100,parseInt(s[1],10))/100,Math.min(100,parseInt(s[2],10))/100,Math.min(100,parseInt(s[3],10))/100,e);break;case"hsl":case"hsla":if(s=/^\s*(\d*\.?\d+)\s*,\s*(\d*\.?\d+)\%\s*,\s*(\d*\.?\d+)\%\s*(?:,\s*(\d*\.?\d+)\s*)?$/.exec(o))return n(s[4]),this.setHSL(parseFloat(s[1])/360,parseFloat(s[2])/100,parseFloat(s[3])/100,e);break;default:console.warn("THREE.Color: Unknown color model "+t)}}else if(i=/^\#([A-Fa-f\d]+)$/.exec(t)){const s=i[1],a=s.length;if(a===3)return this.setRGB(parseInt(s.charAt(0),16)/15,parseInt(s.charAt(1),16)/15,parseInt(s.charAt(2),16)/15,e);if(a===6)return this.setHex(parseInt(s,16),e);console.warn("THREE.Color: Invalid hex color "+t)}else if(t&&t.length>0)return this.setColorName(t,e);return this}setColorName(t,e=nn){const n=Lf[t.toLowerCase()];return n!==void 0?this.setHex(n,e):console.warn("THREE.Color: Unknown color "+t),this}clone(){return new this.constructor(this.r,this.g,this.b)}copy(t){return this.r=t.r,this.g=t.g,this.b=t.b,this}copySRGBToLinear(t){return this.r=hr(t.r),this.g=hr(t.g),this.b=hr(t.b),this}copyLinearToSRGB(t){return this.r=yc(t.r),this.g=yc(t.g),this.b=yc(t.b),this}convertSRGBToLinear(){return this.copySRGBToLinear(this),this}convertLinearToSRGB(){return this.copyLinearToSRGB(this),this}getHex(t=nn){return ge.fromWorkingColorSpace(an.copy(this),t),Math.round(ke(an.r*255,0,255))*65536+Math.round(ke(an.g*255,0,255))*256+Math.round(ke(an.b*255,0,255))}getHexString(t=nn){return("000000"+this.getHex(t).toString(16)).slice(-6)}getHSL(t,e=ge.workingColorSpace){ge.fromWorkingColorSpace(an.copy(this),e);const n=an.r,i=an.g,s=an.b,a=Math.max(n,i,s),o=Math.min(n,i,s);let c,l;const h=(o+a)/2;if(o===a)c=0,l=0;else{const u=a-o;switch(l=h<=.5?u/(a+o):u/(2-a-o),a){case n:c=(i-s)/u+(i<s?6:0);break;case i:c=(s-n)/u+2;break;case s:c=(n-i)/u+4;break}c/=6}return t.h=c,t.s=l,t.l=h,t}getRGB(t,e=ge.workingColorSpace){return ge.fromWorkingColorSpace(an.copy(this),e),t.r=an.r,t.g=an.g,t.b=an.b,t}getStyle(t=nn){ge.fromWorkingColorSpace(an.copy(this),t);const e=an.r,n=an.g,i=an.b;return t!==nn?`color(${t} ${e.toFixed(3)} ${n.toFixed(3)} ${i.toFixed(3)})`:`rgb(${Math.round(e*255)},${Math.round(n*255)},${Math.round(i*255)})`}offsetHSL(t,e,n){return this.getHSL(ki),this.setHSL(ki.h+t,ki.s+e,ki.l+n)}add(t){return this.r+=t.r,this.g+=t.g,this.b+=t.b,this}addColors(t,e){return this.r=t.r+e.r,this.g=t.g+e.g,this.b=t.b+e.b,this}addScalar(t){return this.r+=t,this.g+=t,this.b+=t,this}sub(t){return this.r=Math.max(0,this.r-t.r),this.g=Math.max(0,this.g-t.g),this.b=Math.max(0,this.b-t.b),this}multiply(t){return this.r*=t.r,this.g*=t.g,this.b*=t.b,this}multiplyScalar(t){return this.r*=t,this.g*=t,this.b*=t,this}lerp(t,e){return this.r+=(t.r-this.r)*e,this.g+=(t.g-this.g)*e,this.b+=(t.b-this.b)*e,this}lerpColors(t,e,n){return this.r=t.r+(e.r-t.r)*n,this.g=t.g+(e.g-t.g)*n,this.b=t.b+(e.b-t.b)*n,this}lerpHSL(t,e){this.getHSL(ki),t.getHSL(Ga);const n=ga(ki.h,Ga.h,e),i=ga(ki.s,Ga.s,e),s=ga(ki.l,Ga.l,e);return this.setHSL(n,i,s),this}setFromVector3(t){return this.r=t.x,this.g=t.y,this.b=t.z,this}applyMatrix3(t){const e=this.r,n=this.g,i=this.b,s=t.elements;return this.r=s[0]*e+s[3]*n+s[6]*i,this.g=s[1]*e+s[4]*n+s[7]*i,this.b=s[2]*e+s[5]*n+s[8]*i,this}equals(t){return t.r===this.r&&t.g===this.g&&t.b===this.b}fromArray(t,e=0){return this.r=t[e],this.g=t[e+1],this.b=t[e+2],this}toArray(t=[],e=0){return t[e]=this.r,t[e+1]=this.g,t[e+2]=this.b,t}fromBufferAttribute(t,e){return this.r=t.getX(e),this.g=t.getY(e),this.b=t.getZ(e),this}toJSON(){return this.getHex()}*[Symbol.iterator](){yield this.r,yield this.g,yield this.b}}const an=new Kt;Kt.NAMES=Lf;let Tm=0;class pi extends Sr{constructor(){super(),this.isMaterial=!0,Object.defineProperty(this,"id",{value:Tm++}),this.uuid=Mr(),this.name="",this.type="Material",this.blending=1,this.side=0,this.vertexColors=!1,this.opacity=1,this.transparent=!1,this.alphaHash=!1,this.blendSrc=204,this.blendDst=205,this.blendEquation=100,this.blendSrcAlpha=null,this.blendDstAlpha=null,this.blendEquationAlpha=null,this.blendColor=new Kt(0,0,0),this.blendAlpha=0,this.depthFunc=3,this.depthTest=!0,this.depthWrite=!0,this.stencilWriteMask=255,this.stencilFunc=519,this.stencilRef=0,this.stencilFuncMask=255,this.stencilFail=7680,this.stencilZFail=7680,this.stencilZPass=7680,this.stencilWrite=!1,this.clippingPlanes=null,this.clipIntersection=!1,this.clipShadows=!1,this.shadowSide=null,this.colorWrite=!0,this.precision=null,this.polygonOffset=!1,this.polygonOffsetFactor=0,this.polygonOffsetUnits=0,this.dithering=!1,this.alphaToCoverage=!1,this.premultipliedAlpha=!1,this.forceSinglePass=!1,this.visible=!0,this.toneMapped=!0,this.userData={},this.version=0,this._alphaTest=0}get alphaTest(){return this._alphaTest}set alphaTest(t){this._alphaTest>0!=t>0&&this.version++,this._alphaTest=t}onBuild(){}onBeforeRender(){}onBeforeCompile(){}customProgramCacheKey(){return this.onBeforeCompile.toString()}setValues(t){if(t!==void 0)for(const e in t){const n=t[e];if(n===void 0){console.warn(`THREE.Material: parameter '${e}' has value of undefined.`);continue}const i=this[e];if(i===void 0){console.warn(`THREE.Material: '${e}' is not a property of THREE.${this.type}.`);continue}i&&i.isColor?i.set(n):i&&i.isVector3&&n&&n.isVector3?i.copy(n):this[e]=n}}toJSON(t){const e=t===void 0||typeof t=="string";e&&(t={textures:{},images:{}});const n={metadata:{version:4.6,type:"Material",generator:"Material.toJSON"}};n.uuid=this.uuid,n.type=this.type,this.name!==""&&(n.name=this.name),this.color&&this.color.isColor&&(n.color=this.color.getHex()),this.roughness!==void 0&&(n.roughness=this.roughness),this.metalness!==void 0&&(n.metalness=this.metalness),this.sheen!==void 0&&(n.sheen=this.sheen),this.sheenColor&&this.sheenColor.isColor&&(n.sheenColor=this.sheenColor.getHex()),this.sheenRoughness!==void 0&&(n.sheenRoughness=this.sheenRoughness),this.emissive&&this.emissive.isColor&&(n.emissive=this.emissive.getHex()),this.emissiveIntensity&&this.emissiveIntensity!==1&&(n.emissiveIntensity=this.emissiveIntensity),this.specular&&this.specular.isColor&&(n.specular=this.specular.getHex()),this.specularIntensity!==void 0&&(n.specularIntensity=this.specularIntensity),this.specularColor&&this.specularColor.isColor&&(n.specularColor=this.specularColor.getHex()),this.shininess!==void 0&&(n.shininess=this.shininess),this.clearcoat!==void 0&&(n.clearcoat=this.clearcoat),this.clearcoatRoughness!==void 0&&(n.clearcoatRoughness=this.clearcoatRoughness),this.clearcoatMap&&this.clearcoatMap.isTexture&&(n.clearcoatMap=this.clearcoatMap.toJSON(t).uuid),this.clearcoatRoughnessMap&&this.clearcoatRoughnessMap.isTexture&&(n.clearcoatRoughnessMap=this.clearcoatRoughnessMap.toJSON(t).uuid),this.clearcoatNormalMap&&this.clearcoatNormalMap.isTexture&&(n.clearcoatNormalMap=this.clearcoatNormalMap.toJSON(t).uuid,n.clearcoatNormalScale=this.clearcoatNormalScale.toArray()),this.iridescence!==void 0&&(n.iridescence=this.iridescence),this.iridescenceIOR!==void 0&&(n.iridescenceIOR=this.iridescenceIOR),this.iridescenceThicknessRange!==void 0&&(n.iridescenceThicknessRange=this.iridescenceThicknessRange),this.iridescenceMap&&this.iridescenceMap.isTexture&&(n.iridescenceMap=this.iridescenceMap.toJSON(t).uuid),this.iridescenceThicknessMap&&this.iridescenceThicknessMap.isTexture&&(n.iridescenceThicknessMap=this.iridescenceThicknessMap.toJSON(t).uuid),this.anisotropy!==void 0&&(n.anisotropy=this.anisotropy),this.anisotropyRotation!==void 0&&(n.anisotropyRotation=this.anisotropyRotation),this.anisotropyMap&&this.anisotropyMap.isTexture&&(n.anisotropyMap=this.anisotropyMap.toJSON(t).uuid),this.map&&this.map.isTexture&&(n.map=this.map.toJSON(t).uuid),this.matcap&&this.matcap.isTexture&&(n.matcap=this.matcap.toJSON(t).uuid),this.alphaMap&&this.alphaMap.isTexture&&(n.alphaMap=this.alphaMap.toJSON(t).uuid),this.lightMap&&this.lightMap.isTexture&&(n.lightMap=this.lightMap.toJSON(t).uuid,n.lightMapIntensity=this.lightMapIntensity),this.aoMap&&this.aoMap.isTexture&&(n.aoMap=this.aoMap.toJSON(t).uuid,n.aoMapIntensity=this.aoMapIntensity),this.bumpMap&&this.bumpMap.isTexture&&(n.bumpMap=this.bumpMap.toJSON(t).uuid,n.bumpScale=this.bumpScale),this.normalMap&&this.normalMap.isTexture&&(n.normalMap=this.normalMap.toJSON(t).uuid,n.normalMapType=this.normalMapType,n.normalScale=this.normalScale.toArray()),this.displacementMap&&this.displacementMap.isTexture&&(n.displacementMap=this.displacementMap.toJSON(t).uuid,n.displacementScale=this.displacementScale,n.displacementBias=this.displacementBias),this.roughnessMap&&this.roughnessMap.isTexture&&(n.roughnessMap=this.roughnessMap.toJSON(t).uuid),this.metalnessMap&&this.metalnessMap.isTexture&&(n.metalnessMap=this.metalnessMap.toJSON(t).uuid),this.emissiveMap&&this.emissiveMap.isTexture&&(n.emissiveMap=this.emissiveMap.toJSON(t).uuid),this.specularMap&&this.specularMap.isTexture&&(n.specularMap=this.specularMap.toJSON(t).uuid),this.specularIntensityMap&&this.specularIntensityMap.isTexture&&(n.specularIntensityMap=this.specularIntensityMap.toJSON(t).uuid),this.specularColorMap&&this.specularColorMap.isTexture&&(n.specularColorMap=this.specularColorMap.toJSON(t).uuid),this.envMap&&this.envMap.isTexture&&(n.envMap=this.envMap.toJSON(t).uuid,this.combine!==void 0&&(n.combine=this.combine)),this.envMapIntensity!==void 0&&(n.envMapIntensity=this.envMapIntensity),this.reflectivity!==void 0&&(n.reflectivity=this.reflectivity),this.refractionRatio!==void 0&&(n.refractionRatio=this.refractionRatio),this.gradientMap&&this.gradientMap.isTexture&&(n.gradientMap=this.gradientMap.toJSON(t).uuid),this.transmission!==void 0&&(n.transmission=this.transmission),this.transmissionMap&&this.transmissionMap.isTexture&&(n.transmissionMap=this.transmissionMap.toJSON(t).uuid),this.thickness!==void 0&&(n.thickness=this.thickness),this.thicknessMap&&this.thicknessMap.isTexture&&(n.thicknessMap=this.thicknessMap.toJSON(t).uuid),this.attenuationDistance!==void 0&&this.attenuationDistance!==1/0&&(n.attenuationDistance=this.attenuationDistance),this.attenuationColor!==void 0&&(n.attenuationColor=this.attenuationColor.getHex()),this.size!==void 0&&(n.size=this.size),this.shadowSide!==null&&(n.shadowSide=this.shadowSide),this.sizeAttenuation!==void 0&&(n.sizeAttenuation=this.sizeAttenuation),this.blending!==1&&(n.blending=this.blending),this.side!==0&&(n.side=this.side),this.vertexColors===!0&&(n.vertexColors=!0),this.opacity<1&&(n.opacity=this.opacity),this.transparent===!0&&(n.transparent=!0),this.blendSrc!==204&&(n.blendSrc=this.blendSrc),this.blendDst!==205&&(n.blendDst=this.blendDst),this.blendEquation!==100&&(n.blendEquation=this.blendEquation),this.blendSrcAlpha!==null&&(n.blendSrcAlpha=this.blendSrcAlpha),this.blendDstAlpha!==null&&(n.blendDstAlpha=this.blendDstAlpha),this.blendEquationAlpha!==null&&(n.blendEquationAlpha=this.blendEquationAlpha),this.blendColor&&this.blendColor.isColor&&(n.blendColor=this.blendColor.getHex()),this.blendAlpha!==0&&(n.blendAlpha=this.blendAlpha),this.depthFunc!==3&&(n.depthFunc=this.depthFunc),this.depthTest===!1&&(n.depthTest=this.depthTest),this.depthWrite===!1&&(n.depthWrite=this.depthWrite),this.colorWrite===!1&&(n.colorWrite=this.colorWrite),this.stencilWriteMask!==255&&(n.stencilWriteMask=this.stencilWriteMask),this.stencilFunc!==519&&(n.stencilFunc=this.stencilFunc),this.stencilRef!==0&&(n.stencilRef=this.stencilRef),this.stencilFuncMask!==255&&(n.stencilFuncMask=this.stencilFuncMask),this.stencilFail!==7680&&(n.stencilFail=this.stencilFail),this.stencilZFail!==7680&&(n.stencilZFail=this.stencilZFail),this.stencilZPass!==7680&&(n.stencilZPass=this.stencilZPass),this.stencilWrite===!0&&(n.stencilWrite=this.stencilWrite),this.rotation!==void 0&&this.rotation!==0&&(n.rotation=this.rotation),this.polygonOffset===!0&&(n.polygonOffset=!0),this.polygonOffsetFactor!==0&&(n.polygonOffsetFactor=this.polygonOffsetFactor),this.polygonOffsetUnits!==0&&(n.polygonOffsetUnits=this.polygonOffsetUnits),this.linewidth!==void 0&&this.linewidth!==1&&(n.linewidth=this.linewidth),this.dashSize!==void 0&&(n.dashSize=this.dashSize),this.gapSize!==void 0&&(n.gapSize=this.gapSize),this.scale!==void 0&&(n.scale=this.scale),this.dithering===!0&&(n.dithering=!0),this.alphaTest>0&&(n.alphaTest=this.alphaTest),this.alphaHash===!0&&(n.alphaHash=!0),this.alphaToCoverage===!0&&(n.alphaToCoverage=!0),this.premultipliedAlpha===!0&&(n.premultipliedAlpha=!0),this.forceSinglePass===!0&&(n.forceSinglePass=!0),this.wireframe===!0&&(n.wireframe=!0),this.wireframeLinewidth>1&&(n.wireframeLinewidth=this.wireframeLinewidth),this.wireframeLinecap!=="round"&&(n.wireframeLinecap=this.wireframeLinecap),this.wireframeLinejoin!=="round"&&(n.wireframeLinejoin=this.wireframeLinejoin),this.flatShading===!0&&(n.flatShading=!0),this.visible===!1&&(n.visible=!1),this.toneMapped===!1&&(n.toneMapped=!1),this.fog===!1&&(n.fog=!1),Object.keys(this.userData).length>0&&(n.userData=this.userData);function i(s){const a=[];for(const o in s){const c=s[o];delete c.metadata,a.push(c)}return a}if(e){const s=i(t.textures),a=i(t.images);s.length>0&&(n.textures=s),a.length>0&&(n.images=a)}return n}clone(){return new this.constructor().copy(this)}copy(t){this.name=t.name,this.blending=t.blending,this.side=t.side,this.vertexColors=t.vertexColors,this.opacity=t.opacity,this.transparent=t.transparent,this.blendSrc=t.blendSrc,this.blendDst=t.blendDst,this.blendEquation=t.blendEquation,this.blendSrcAlpha=t.blendSrcAlpha,this.blendDstAlpha=t.blendDstAlpha,this.blendEquationAlpha=t.blendEquationAlpha,this.blendColor.copy(t.blendColor),this.blendAlpha=t.blendAlpha,this.depthFunc=t.depthFunc,this.depthTest=t.depthTest,this.depthWrite=t.depthWrite,this.stencilWriteMask=t.stencilWriteMask,this.stencilFunc=t.stencilFunc,this.stencilRef=t.stencilRef,this.stencilFuncMask=t.stencilFuncMask,this.stencilFail=t.stencilFail,this.stencilZFail=t.stencilZFail,this.stencilZPass=t.stencilZPass,this.stencilWrite=t.stencilWrite;const e=t.clippingPlanes;let n=null;if(e!==null){const i=e.length;n=new Array(i);for(let s=0;s!==i;++s)n[s]=e[s].clone()}return this.clippingPlanes=n,this.clipIntersection=t.clipIntersection,this.clipShadows=t.clipShadows,this.shadowSide=t.shadowSide,this.colorWrite=t.colorWrite,this.precision=t.precision,this.polygonOffset=t.polygonOffset,this.polygonOffsetFactor=t.polygonOffsetFactor,this.polygonOffsetUnits=t.polygonOffsetUnits,this.dithering=t.dithering,this.alphaTest=t.alphaTest,this.alphaHash=t.alphaHash,this.alphaToCoverage=t.alphaToCoverage,this.premultipliedAlpha=t.premultipliedAlpha,this.forceSinglePass=t.forceSinglePass,this.visible=t.visible,this.toneMapped=t.toneMapped,this.userData=JSON.parse(JSON.stringify(t.userData)),this}dispose(){this.dispatchEvent({type:"dispose"})}set needsUpdate(t){t===!0&&this.version++}}class Ue extends pi{constructor(t){super(),this.isMeshBasicMaterial=!0,this.type="MeshBasicMaterial",this.color=new Kt(16777215),this.map=null,this.lightMap=null,this.lightMapIntensity=1,this.aoMap=null,this.aoMapIntensity=1,this.specularMap=null,this.alphaMap=null,this.envMap=null,this.combine=0,this.reflectivity=1,this.refractionRatio=.98,this.wireframe=!1,this.wireframeLinewidth=1,this.wireframeLinecap="round",this.wireframeLinejoin="round",this.fog=!0,this.setValues(t)}copy(t){return super.copy(t),this.color.copy(t.color),this.map=t.map,this.lightMap=t.lightMap,this.lightMapIntensity=t.lightMapIntensity,this.aoMap=t.aoMap,this.aoMapIntensity=t.aoMapIntensity,this.specularMap=t.specularMap,this.alphaMap=t.alphaMap,this.envMap=t.envMap,this.combine=t.combine,this.reflectivity=t.reflectivity,this.refractionRatio=t.refractionRatio,this.wireframe=t.wireframe,this.wireframeLinewidth=t.wireframeLinewidth,this.wireframeLinecap=t.wireframeLinecap,this.wireframeLinejoin=t.wireframeLinejoin,this.fog=t.fog,this}}const ze=new C,Va=new yt;class Fe{constructor(t,e,n=!1){if(Array.isArray(t))throw new TypeError("THREE.BufferAttribute: array should be a Typed Array.");this.isBufferAttribute=!0,this.name="",this.array=t,this.itemSize=e,this.count=t!==void 0?t.length/e:0,this.normalized=n,this.usage=35044,this._updateRange={offset:0,count:-1},this.updateRanges=[],this.gpuType=1015,this.version=0}onUploadCallback(){}set needsUpdate(t){t===!0&&this.version++}get updateRange(){return console.warn("THREE.BufferAttribute: updateRange() is deprecated and will be removed in r169. Use addUpdateRange() instead."),this._updateRange}setUsage(t){return this.usage=t,this}addUpdateRange(t,e){this.updateRanges.push({start:t,count:e})}clearUpdateRanges(){this.updateRanges.length=0}copy(t){return this.name=t.name,this.array=new t.array.constructor(t.array),this.itemSize=t.itemSize,this.count=t.count,this.normalized=t.normalized,this.usage=t.usage,this.gpuType=t.gpuType,this}copyAt(t,e,n){t*=this.itemSize,n*=e.itemSize;for(let i=0,s=this.itemSize;i<s;i++)this.array[t+i]=e.array[n+i];return this}copyArray(t){return this.array.set(t),this}applyMatrix3(t){if(this.itemSize===2)for(let e=0,n=this.count;e<n;e++)Va.fromBufferAttribute(this,e),Va.applyMatrix3(t),this.setXY(e,Va.x,Va.y);else if(this.itemSize===3)for(let e=0,n=this.count;e<n;e++)ze.fromBufferAttribute(this,e),ze.applyMatrix3(t),this.setXYZ(e,ze.x,ze.y,ze.z);return this}applyMatrix4(t){for(let e=0,n=this.count;e<n;e++)ze.fromBufferAttribute(this,e),ze.applyMatrix4(t),this.setXYZ(e,ze.x,ze.y,ze.z);return this}applyNormalMatrix(t){for(let e=0,n=this.count;e<n;e++)ze.fromBufferAttribute(this,e),ze.applyNormalMatrix(t),this.setXYZ(e,ze.x,ze.y,ze.z);return this}transformDirection(t){for(let e=0,n=this.count;e<n;e++)ze.fromBufferAttribute(this,e),ze.transformDirection(t),this.setXYZ(e,ze.x,ze.y,ze.z);return this}set(t,e=0){return this.array.set(t,e),this}getComponent(t,e){let n=this.array[t*this.itemSize+e];return this.normalized&&(n=er(n,this.array)),n}setComponent(t,e,n){return this.normalized&&(n=dn(n,this.array)),this.array[t*this.itemSize+e]=n,this}getX(t){let e=this.array[t*this.itemSize];return this.normalized&&(e=er(e,this.array)),e}setX(t,e){return this.normalized&&(e=dn(e,this.array)),this.array[t*this.itemSize]=e,this}getY(t){let e=this.array[t*this.itemSize+1];return this.normalized&&(e=er(e,this.array)),e}setY(t,e){return this.normalized&&(e=dn(e,this.array)),this.array[t*this.itemSize+1]=e,this}getZ(t){let e=this.array[t*this.itemSize+2];return this.normalized&&(e=er(e,this.array)),e}setZ(t,e){return this.normalized&&(e=dn(e,this.array)),this.array[t*this.itemSize+2]=e,this}getW(t){let e=this.array[t*this.itemSize+3];return this.normalized&&(e=er(e,this.array)),e}setW(t,e){return this.normalized&&(e=dn(e,this.array)),this.array[t*this.itemSize+3]=e,this}setXY(t,e,n){return t*=this.itemSize,this.normalized&&(e=dn(e,this.array),n=dn(n,this.array)),this.array[t+0]=e,this.array[t+1]=n,this}setXYZ(t,e,n,i){return t*=this.itemSize,this.normalized&&(e=dn(e,this.array),n=dn(n,this.array),i=dn(i,this.array)),this.array[t+0]=e,this.array[t+1]=n,this.array[t+2]=i,this}setXYZW(t,e,n,i,s){return t*=this.itemSize,this.normalized&&(e=dn(e,this.array),n=dn(n,this.array),i=dn(i,this.array),s=dn(s,this.array)),this.array[t+0]=e,this.array[t+1]=n,this.array[t+2]=i,this.array[t+3]=s,this}onUpload(t){return this.onUploadCallback=t,this}clone(){return new this.constructor(this.array,this.itemSize).copy(this)}toJSON(){const t={itemSize:this.itemSize,type:this.array.constructor.name,array:Array.from(this.array),normalized:this.normalized};return this.name!==""&&(t.name=this.name),this.usage!==35044&&(t.usage=this.usage),t}}class Df extends Fe{constructor(t,e,n){super(new Uint16Array(t),e,n)}}class If extends Fe{constructor(t,e,n){super(new Uint32Array(t),e,n)}}class se extends Fe{constructor(t,e,n){super(new Float32Array(t),e,n)}}let Cm=0;const Bn=new re,Fc=new Je,Us=new C,Rn=new tn,Dr=new tn,je=new C;class we extends Sr{constructor(){super(),this.isBufferGeometry=!0,Object.defineProperty(this,"id",{value:Cm++}),this.uuid=Mr(),this.name="",this.type="BufferGeometry",this.index=null,this.attributes={},this.morphAttributes={},this.morphTargetsRelative=!1,this.groups=[],this.boundingBox=null,this.boundingSphere=null,this.drawRange={start:0,count:1/0},this.userData={}}getIndex(){return this.index}setIndex(t){return Array.isArray(t)?this.index=new(Tf(t)?If:Df)(t,1):this.index=t,this}getAttribute(t){return this.attributes[t]}setAttribute(t,e){return this.attributes[t]=e,this}deleteAttribute(t){return delete this.attributes[t],this}hasAttribute(t){return this.attributes[t]!==void 0}addGroup(t,e,n=0){this.groups.push({start:t,count:e,materialIndex:n})}clearGroups(){this.groups=[]}setDrawRange(t,e){this.drawRange.start=t,this.drawRange.count=e}applyMatrix4(t){const e=this.attributes.position;e!==void 0&&(e.applyMatrix4(t),e.needsUpdate=!0);const n=this.attributes.normal;if(n!==void 0){const s=new ie().getNormalMatrix(t);n.applyNormalMatrix(s),n.needsUpdate=!0}const i=this.attributes.tangent;return i!==void 0&&(i.transformDirection(t),i.needsUpdate=!0),this.boundingBox!==null&&this.computeBoundingBox(),this.boundingSphere!==null&&this.computeBoundingSphere(),this}applyQuaternion(t){return Bn.makeRotationFromQuaternion(t),this.applyMatrix4(Bn),this}rotateX(t){return Bn.makeRotationX(t),this.applyMatrix4(Bn),this}rotateY(t){return Bn.makeRotationY(t),this.applyMatrix4(Bn),this}rotateZ(t){return Bn.makeRotationZ(t),this.applyMatrix4(Bn),this}translate(t,e,n){return Bn.makeTranslation(t,e,n),this.applyMatrix4(Bn),this}scale(t,e,n){return Bn.makeScale(t,e,n),this.applyMatrix4(Bn),this}lookAt(t){return Fc.lookAt(t),Fc.updateMatrix(),this.applyMatrix4(Fc.matrix),this}center(){return this.computeBoundingBox(),this.boundingBox.getCenter(Us).negate(),this.translate(Us.x,Us.y,Us.z),this}setFromPoints(t){const e=[];for(let n=0,i=t.length;n<i;n++){const s=t[n];e.push(s.x,s.y,s.z||0)}return this.setAttribute("position",new se(e,3)),this}computeBoundingBox(){this.boundingBox===null&&(this.boundingBox=new tn);const t=this.attributes.position,e=this.morphAttributes.position;if(t&&t.isGLBufferAttribute){console.error('THREE.BufferGeometry.computeBoundingBox(): GLBufferAttribute requires a manual bounding box. Alternatively set "mesh.frustumCulled" to "false".',this),this.boundingBox.set(new C(-1/0,-1/0,-1/0),new C(1/0,1/0,1/0));return}if(t!==void 0){if(this.boundingBox.setFromBufferAttribute(t),e)for(let n=0,i=e.length;n<i;n++){const s=e[n];Rn.setFromBufferAttribute(s),this.morphTargetsRelative?(je.addVectors(this.boundingBox.min,Rn.min),this.boundingBox.expandByPoint(je),je.addVectors(this.boundingBox.max,Rn.max),this.boundingBox.expandByPoint(je)):(this.boundingBox.expandByPoint(Rn.min),this.boundingBox.expandByPoint(Rn.max))}}else this.boundingBox.makeEmpty();(isNaN(this.boundingBox.min.x)||isNaN(this.boundingBox.min.y)||isNaN(this.boundingBox.min.z))&&console.error('THREE.BufferGeometry.computeBoundingBox(): Computed min/max have NaN values. The "position" attribute is likely to have NaN values.',this)}computeBoundingSphere(){this.boundingSphere===null&&(this.boundingSphere=new ns);const t=this.attributes.position,e=this.morphAttributes.position;if(t&&t.isGLBufferAttribute){console.error('THREE.BufferGeometry.computeBoundingSphere(): GLBufferAttribute requires a manual bounding sphere. Alternatively set "mesh.frustumCulled" to "false".',this),this.boundingSphere.set(new C,1/0);return}if(t){const n=this.boundingSphere.center;if(Rn.setFromBufferAttribute(t),e)for(let s=0,a=e.length;s<a;s++){const o=e[s];Dr.setFromBufferAttribute(o),this.morphTargetsRelative?(je.addVectors(Rn.min,Dr.min),Rn.expandByPoint(je),je.addVectors(Rn.max,Dr.max),Rn.expandByPoint(je)):(Rn.expandByPoint(Dr.min),Rn.expandByPoint(Dr.max))}Rn.getCenter(n);let i=0;for(let s=0,a=t.count;s<a;s++)je.fromBufferAttribute(t,s),i=Math.max(i,n.distanceToSquared(je));if(e)for(let s=0,a=e.length;s<a;s++){const o=e[s],c=this.morphTargetsRelative;for(let l=0,h=o.count;l<h;l++)je.fromBufferAttribute(o,l),c&&(Us.fromBufferAttribute(t,l),je.add(Us)),i=Math.max(i,n.distanceToSquared(je))}this.boundingSphere.radius=Math.sqrt(i),isNaN(this.boundingSphere.radius)&&console.error('THREE.BufferGeometry.computeBoundingSphere(): Computed radius is NaN. The "position" attribute is likely to have NaN values.',this)}}computeTangents(){const t=this.index,e=this.attributes;if(t===null||e.position===void 0||e.normal===void 0||e.uv===void 0){console.error("THREE.BufferGeometry: .computeTangents() failed. Missing required attributes (index, position, normal or uv)");return}const n=t.array,i=e.position.array,s=e.normal.array,a=e.uv.array,o=i.length/3;this.hasAttribute("tangent")===!1&&this.setAttribute("tangent",new Fe(new Float32Array(4*o),4));const c=this.getAttribute("tangent").array,l=[],h=[];for(let A=0;A<o;A++)l[A]=new C,h[A]=new C;const u=new C,d=new C,f=new C,m=new yt,x=new yt,g=new yt,p=new C,_=new C;function v(A,D,b){u.fromArray(i,A*3),d.fromArray(i,D*3),f.fromArray(i,b*3),m.fromArray(a,A*2),x.fromArray(a,D*2),g.fromArray(a,b*2),d.sub(u),f.sub(u),x.sub(m),g.sub(m);const F=1/(x.x*g.y-g.x*x.y);isFinite(F)&&(p.copy(d).multiplyScalar(g.y).addScaledVector(f,-x.y).multiplyScalar(F),_.copy(f).multiplyScalar(x.x).addScaledVector(d,-g.x).multiplyScalar(F),l[A].add(p),l[D].add(p),l[b].add(p),h[A].add(_),h[D].add(_),h[b].add(_))}let S=this.groups;S.length===0&&(S=[{start:0,count:n.length}]);for(let A=0,D=S.length;A<D;++A){const b=S[A],F=b.start,L=b.count;for(let N=F,U=F+L;N<U;N+=3)v(n[N+0],n[N+1],n[N+2])}const M=new C,y=new C,w=new C,T=new C;function E(A){w.fromArray(s,A*3),T.copy(w);const D=l[A];M.copy(D),M.sub(w.multiplyScalar(w.dot(D))).normalize(),y.crossVectors(T,D);const F=y.dot(h[A])<0?-1:1;c[A*4]=M.x,c[A*4+1]=M.y,c[A*4+2]=M.z,c[A*4+3]=F}for(let A=0,D=S.length;A<D;++A){const b=S[A],F=b.start,L=b.count;for(let N=F,U=F+L;N<U;N+=3)E(n[N+0]),E(n[N+1]),E(n[N+2])}}computeVertexNormals(){const t=this.index,e=this.getAttribute("position");if(e!==void 0){let n=this.getAttribute("normal");if(n===void 0)n=new Fe(new Float32Array(e.count*3),3),this.setAttribute("normal",n);else for(let d=0,f=n.count;d<f;d++)n.setXYZ(d,0,0,0);const i=new C,s=new C,a=new C,o=new C,c=new C,l=new C,h=new C,u=new C;if(t)for(let d=0,f=t.count;d<f;d+=3){const m=t.getX(d+0),x=t.getX(d+1),g=t.getX(d+2);i.fromBufferAttribute(e,m),s.fromBufferAttribute(e,x),a.fromBufferAttribute(e,g),h.subVectors(a,s),u.subVectors(i,s),h.cross(u),o.fromBufferAttribute(n,m),c.fromBufferAttribute(n,x),l.fromBufferAttribute(n,g),o.add(h),c.add(h),l.add(h),n.setXYZ(m,o.x,o.y,o.z),n.setXYZ(x,c.x,c.y,c.z),n.setXYZ(g,l.x,l.y,l.z)}else for(let d=0,f=e.count;d<f;d+=3)i.fromBufferAttribute(e,d+0),s.fromBufferAttribute(e,d+1),a.fromBufferAttribute(e,d+2),h.subVectors(a,s),u.subVectors(i,s),h.cross(u),n.setXYZ(d+0,h.x,h.y,h.z),n.setXYZ(d+1,h.x,h.y,h.z),n.setXYZ(d+2,h.x,h.y,h.z);this.normalizeNormals(),n.needsUpdate=!0}}normalizeNormals(){const t=this.attributes.normal;for(let e=0,n=t.count;e<n;e++)je.fromBufferAttribute(t,e),je.normalize(),t.setXYZ(e,je.x,je.y,je.z)}toNonIndexed(){function t(o,c){const l=o.array,h=o.itemSize,u=o.normalized,d=new l.constructor(c.length*h);let f=0,m=0;for(let x=0,g=c.length;x<g;x++){o.isInterleavedBufferAttribute?f=c[x]*o.data.stride+o.offset:f=c[x]*h;for(let p=0;p<h;p++)d[m++]=l[f++]}return new Fe(d,h,u)}if(this.index===null)return console.warn("THREE.BufferGeometry.toNonIndexed(): BufferGeometry is already non-indexed."),this;const e=new we,n=this.index.array,i=this.attributes;for(const o in i){const c=i[o],l=t(c,n);e.setAttribute(o,l)}const s=this.morphAttributes;for(const o in s){const c=[],l=s[o];for(let h=0,u=l.length;h<u;h++){const d=l[h],f=t(d,n);c.push(f)}e.morphAttributes[o]=c}e.morphTargetsRelative=this.morphTargetsRelative;const a=this.groups;for(let o=0,c=a.length;o<c;o++){const l=a[o];e.addGroup(l.start,l.count,l.materialIndex)}return e}toJSON(){const t={metadata:{version:4.6,type:"BufferGeometry",generator:"BufferGeometry.toJSON"}};if(t.uuid=this.uuid,t.type=this.type,this.name!==""&&(t.name=this.name),Object.keys(this.userData).length>0&&(t.userData=this.userData),this.parameters!==void 0){const c=this.parameters;for(const l in c)c[l]!==void 0&&(t[l]=c[l]);return t}t.data={attributes:{}};const e=this.index;e!==null&&(t.data.index={type:e.array.constructor.name,array:Array.prototype.slice.call(e.array)});const n=this.attributes;for(const c in n){const l=n[c];t.data.attributes[c]=l.toJSON(t.data)}const i={};let s=!1;for(const c in this.morphAttributes){const l=this.morphAttributes[c],h=[];for(let u=0,d=l.length;u<d;u++){const f=l[u];h.push(f.toJSON(t.data))}h.length>0&&(i[c]=h,s=!0)}s&&(t.data.morphAttributes=i,t.data.morphTargetsRelative=this.morphTargetsRelative);const a=this.groups;a.length>0&&(t.data.groups=JSON.parse(JSON.stringify(a)));const o=this.boundingSphere;return o!==null&&(t.data.boundingSphere={center:o.center.toArray(),radius:o.radius}),t}clone(){return new this.constructor().copy(this)}copy(t){this.index=null,this.attributes={},this.morphAttributes={},this.groups=[],this.boundingBox=null,this.boundingSphere=null;const e={};this.name=t.name;const n=t.index;n!==null&&this.setIndex(n.clone(e));const i=t.attributes;for(const l in i){const h=i[l];this.setAttribute(l,h.clone(e))}const s=t.morphAttributes;for(const l in s){const h=[],u=s[l];for(let d=0,f=u.length;d<f;d++)h.push(u[d].clone(e));this.morphAttributes[l]=h}this.morphTargetsRelative=t.morphTargetsRelative;const a=t.groups;for(let l=0,h=a.length;l<h;l++){const u=a[l];this.addGroup(u.start,u.count,u.materialIndex)}const o=t.boundingBox;o!==null&&(this.boundingBox=o.clone());const c=t.boundingSphere;return c!==null&&(this.boundingSphere=c.clone()),this.drawRange.start=t.drawRange.start,this.drawRange.count=t.drawRange.count,this.userData=t.userData,this}dispose(){this.dispatchEvent({type:"dispose"})}}const cu=new re,as=new Eh,ka=new ns,lu=new C,Bs=new C,zs=new C,Os=new C,Uc=new C,Ha=new C,Wa=new yt,Xa=new yt,Ya=new yt,hu=new C,uu=new C,du=new C,qa=new C,Za=new C;class $t extends Je{constructor(t=new we,e=new Ue){super(),this.isMesh=!0,this.type="Mesh",this.geometry=t,this.material=e,this.updateMorphTargets()}copy(t,e){return super.copy(t,e),t.morphTargetInfluences!==void 0&&(this.morphTargetInfluences=t.morphTargetInfluences.slice()),t.morphTargetDictionary!==void 0&&(this.morphTargetDictionary=Object.assign({},t.morphTargetDictionary)),this.material=Array.isArray(t.material)?t.material.slice():t.material,this.geometry=t.geometry,this}updateMorphTargets(){const e=this.geometry.morphAttributes,n=Object.keys(e);if(n.length>0){const i=e[n[0]];if(i!==void 0){this.morphTargetInfluences=[],this.morphTargetDictionary={};for(let s=0,a=i.length;s<a;s++){const o=i[s].name||String(s);this.morphTargetInfluences.push(0),this.morphTargetDictionary[o]=s}}}}getVertexPosition(t,e){const n=this.geometry,i=n.attributes.position,s=n.morphAttributes.position,a=n.morphTargetsRelative;e.fromBufferAttribute(i,t);const o=this.morphTargetInfluences;if(s&&o){Ha.set(0,0,0);for(let c=0,l=s.length;c<l;c++){const h=o[c],u=s[c];h!==0&&(Uc.fromBufferAttribute(u,t),a?Ha.addScaledVector(Uc,h):Ha.addScaledVector(Uc.sub(e),h))}e.add(Ha)}return e}raycast(t,e){const n=this.geometry,i=this.material,s=this.matrixWorld;i!==void 0&&(n.boundingSphere===null&&n.computeBoundingSphere(),ka.copy(n.boundingSphere),ka.applyMatrix4(s),as.copy(t.ray).recast(t.near),!(ka.containsPoint(as.origin)===!1&&(as.intersectSphere(ka,lu)===null||as.origin.distanceToSquared(lu)>(t.far-t.near)**2))&&(cu.copy(s).invert(),as.copy(t.ray).applyMatrix4(cu),!(n.boundingBox!==null&&as.intersectsBox(n.boundingBox)===!1)&&this._computeIntersections(t,e,as)))}_computeIntersections(t,e,n){let i;const s=this.geometry,a=this.material,o=s.index,c=s.attributes.position,l=s.attributes.uv,h=s.attributes.uv1,u=s.attributes.normal,d=s.groups,f=s.drawRange;if(o!==null)if(Array.isArray(a))for(let m=0,x=d.length;m<x;m++){const g=d[m],p=a[g.materialIndex],_=Math.max(g.start,f.start),v=Math.min(o.count,Math.min(g.start+g.count,f.start+f.count));for(let S=_,M=v;S<M;S+=3){const y=o.getX(S),w=o.getX(S+1),T=o.getX(S+2);i=ja(this,p,t,n,l,h,u,y,w,T),i&&(i.faceIndex=Math.floor(S/3),i.face.materialIndex=g.materialIndex,e.push(i))}}else{const m=Math.max(0,f.start),x=Math.min(o.count,f.start+f.count);for(let g=m,p=x;g<p;g+=3){const _=o.getX(g),v=o.getX(g+1),S=o.getX(g+2);i=ja(this,a,t,n,l,h,u,_,v,S),i&&(i.faceIndex=Math.floor(g/3),e.push(i))}}else if(c!==void 0)if(Array.isArray(a))for(let m=0,x=d.length;m<x;m++){const g=d[m],p=a[g.materialIndex],_=Math.max(g.start,f.start),v=Math.min(c.count,Math.min(g.start+g.count,f.start+f.count));for(let S=_,M=v;S<M;S+=3){const y=S,w=S+1,T=S+2;i=ja(this,p,t,n,l,h,u,y,w,T),i&&(i.faceIndex=Math.floor(S/3),i.face.materialIndex=g.materialIndex,e.push(i))}}else{const m=Math.max(0,f.start),x=Math.min(c.count,f.start+f.count);for(let g=m,p=x;g<p;g+=3){const _=g,v=g+1,S=g+2;i=ja(this,a,t,n,l,h,u,_,v,S),i&&(i.faceIndex=Math.floor(g/3),e.push(i))}}}}function bm(r,t,e,n,i,s,a,o){let c;if(t.side===1?c=n.intersectTriangle(a,s,i,!0,o):c=n.intersectTriangle(i,s,a,t.side===0,o),c===null)return null;Za.copy(o),Za.applyMatrix4(r.matrixWorld);const l=e.ray.origin.distanceTo(Za);return l<e.near||l>e.far?null:{distance:l,point:Za.clone(),object:r}}function ja(r,t,e,n,i,s,a,o,c,l){r.getVertexPosition(o,Bs),r.getVertexPosition(c,zs),r.getVertexPosition(l,Os);const h=bm(r,t,e,n,Bs,zs,Os,qa);if(h){i&&(Wa.fromBufferAttribute(i,o),Xa.fromBufferAttribute(i,c),Ya.fromBufferAttribute(i,l),h.uv=sn.getInterpolation(qa,Bs,zs,Os,Wa,Xa,Ya,new yt)),s&&(Wa.fromBufferAttribute(s,o),Xa.fromBufferAttribute(s,c),Ya.fromBufferAttribute(s,l),h.uv1=sn.getInterpolation(qa,Bs,zs,Os,Wa,Xa,Ya,new yt),h.uv2=h.uv1),a&&(hu.fromBufferAttribute(a,o),uu.fromBufferAttribute(a,c),du.fromBufferAttribute(a,l),h.normal=sn.getInterpolation(qa,Bs,zs,Os,hu,uu,du,new C),h.normal.dot(n.direction)>0&&h.normal.multiplyScalar(-1));const u={a:o,b:c,c:l,normal:new C,materialIndex:0};sn.getNormal(Bs,zs,Os,u.normal),h.face=u}return h}class Ln extends we{constructor(t=1,e=1,n=1,i=1,s=1,a=1){super(),this.type="BoxGeometry",this.parameters={width:t,height:e,depth:n,widthSegments:i,heightSegments:s,depthSegments:a};const o=this;i=Math.floor(i),s=Math.floor(s),a=Math.floor(a);const c=[],l=[],h=[],u=[];let d=0,f=0;m("z","y","x",-1,-1,n,e,t,a,s,0),m("z","y","x",1,-1,n,e,-t,a,s,1),m("x","z","y",1,1,t,n,e,i,a,2),m("x","z","y",1,-1,t,n,-e,i,a,3),m("x","y","z",1,-1,t,e,n,i,s,4),m("x","y","z",-1,-1,t,e,-n,i,s,5),this.setIndex(c),this.setAttribute("position",new se(l,3)),this.setAttribute("normal",new se(h,3)),this.setAttribute("uv",new se(u,2));function m(x,g,p,_,v,S,M,y,w,T,E){const A=S/w,D=M/T,b=S/2,F=M/2,L=y/2,N=w+1,U=T+1;let k=0,O=0;const H=new C;for(let j=0;j<U;j++){const J=j*D-F;for(let nt=0;nt<N;nt++){const V=nt*A-b;H[x]=V*_,H[g]=J*v,H[p]=L,l.push(H.x,H.y,H.z),H[x]=0,H[g]=0,H[p]=y>0?1:-1,h.push(H.x,H.y,H.z),u.push(nt/w),u.push(1-j/T),k+=1}}for(let j=0;j<T;j++)for(let J=0;J<w;J++){const nt=d+J+N*j,V=d+J+N*(j+1),$=d+(J+1)+N*(j+1),st=d+(J+1)+N*j;c.push(nt,V,st),c.push(V,$,st),O+=6}o.addGroup(f,O,E),f+=O,d+=k}}copy(t){return super.copy(t),this.parameters=Object.assign({},t.parameters),this}static fromJSON(t){return new Ln(t.width,t.height,t.depth,t.widthSegments,t.heightSegments,t.depthSegments)}}function mr(r){const t={};for(const e in r){t[e]={};for(const n in r[e]){const i=r[e][n];i&&(i.isColor||i.isMatrix3||i.isMatrix4||i.isVector2||i.isVector3||i.isVector4||i.isTexture||i.isQuaternion)?i.isRenderTargetTexture?(console.warn("UniformsUtils: Textures of render targets cannot be cloned via cloneUniforms() or mergeUniforms()."),t[e][n]=null):t[e][n]=i.clone():Array.isArray(i)?t[e][n]=i.slice():t[e][n]=i}}return t}function fn(r){const t={};for(let e=0;e<r.length;e++){const n=mr(r[e]);for(const i in n)t[i]=n[i]}return t}function Rm(r){const t=[];for(let e=0;e<r.length;e++)t.push(r[e].clone());return t}function Nf(r){return r.getRenderTarget()===null?r.outputColorSpace:ge.workingColorSpace}const Pm={clone:mr,merge:fn};var Lm=`void main() {
	gl_Position = projectionMatrix * modelViewMatrix * vec4( position, 1.0 );
}`,Dm=`void main() {
	gl_FragColor = vec4( 1.0, 0.0, 0.0, 1.0 );
}`;class Un extends pi{constructor(t){super(),this.isShaderMaterial=!0,this.type="ShaderMaterial",this.defines={},this.uniforms={},this.uniformsGroups=[],this.vertexShader=Lm,this.fragmentShader=Dm,this.linewidth=1,this.wireframe=!1,this.wireframeLinewidth=1,this.fog=!1,this.lights=!1,this.clipping=!1,this.forceSinglePass=!0,this.extensions={derivatives:!1,fragDepth:!1,drawBuffers:!1,shaderTextureLOD:!1,clipCullDistance:!1},this.defaultAttributeValues={color:[1,1,1],uv:[0,0],uv1:[0,0]},this.index0AttributeName=void 0,this.uniformsNeedUpdate=!1,this.glslVersion=null,t!==void 0&&this.setValues(t)}copy(t){return super.copy(t),this.fragmentShader=t.fragmentShader,this.vertexShader=t.vertexShader,this.uniforms=mr(t.uniforms),this.uniformsGroups=Rm(t.uniformsGroups),this.defines=Object.assign({},t.defines),this.wireframe=t.wireframe,this.wireframeLinewidth=t.wireframeLinewidth,this.fog=t.fog,this.lights=t.lights,this.clipping=t.clipping,this.extensions=Object.assign({},t.extensions),this.glslVersion=t.glslVersion,this}toJSON(t){const e=super.toJSON(t);e.glslVersion=this.glslVersion,e.uniforms={};for(const i in this.uniforms){const a=this.uniforms[i].value;a&&a.isTexture?e.uniforms[i]={type:"t",value:a.toJSON(t).uuid}:a&&a.isColor?e.uniforms[i]={type:"c",value:a.getHex()}:a&&a.isVector2?e.uniforms[i]={type:"v2",value:a.toArray()}:a&&a.isVector3?e.uniforms[i]={type:"v3",value:a.toArray()}:a&&a.isVector4?e.uniforms[i]={type:"v4",value:a.toArray()}:a&&a.isMatrix3?e.uniforms[i]={type:"m3",value:a.toArray()}:a&&a.isMatrix4?e.uniforms[i]={type:"m4",value:a.toArray()}:e.uniforms[i]={value:a}}Object.keys(this.defines).length>0&&(e.defines=this.defines),e.vertexShader=this.vertexShader,e.fragmentShader=this.fragmentShader,e.lights=this.lights,e.clipping=this.clipping;const n={};for(const i in this.extensions)this.extensions[i]===!0&&(n[i]=!0);return Object.keys(n).length>0&&(e.extensions=n),e}}class Ff extends Je{constructor(){super(),this.isCamera=!0,this.type="Camera",this.matrixWorldInverse=new re,this.projectionMatrix=new re,this.projectionMatrixInverse=new re,this.coordinateSystem=2e3}copy(t,e){return super.copy(t,e),this.matrixWorldInverse.copy(t.matrixWorldInverse),this.projectionMatrix.copy(t.projectionMatrix),this.projectionMatrixInverse.copy(t.projectionMatrixInverse),this.coordinateSystem=t.coordinateSystem,this}getWorldDirection(t){return super.getWorldDirection(t).negate()}updateMatrixWorld(t){super.updateMatrixWorld(t),this.matrixWorldInverse.copy(this.matrixWorld).invert()}updateWorldMatrix(t,e){super.updateWorldMatrix(t,e),this.matrixWorldInverse.copy(this.matrixWorld).invert()}clone(){return new this.constructor().copy(this)}}class Dn extends Ff{constructor(t=50,e=1,n=.1,i=2e3){super(),this.isPerspectiveCamera=!0,this.type="PerspectiveCamera",this.fov=t,this.zoom=1,this.near=n,this.far=i,this.focus=10,this.aspect=e,this.view=null,this.filmGauge=35,this.filmOffset=0,this.updateProjectionMatrix()}copy(t,e){return super.copy(t,e),this.fov=t.fov,this.zoom=t.zoom,this.near=t.near,this.far=t.far,this.focus=t.focus,this.aspect=t.aspect,this.view=t.view===null?null:Object.assign({},t.view),this.filmGauge=t.filmGauge,this.filmOffset=t.filmOffset,this}setFocalLength(t){const e=.5*this.getFilmHeight()/t;this.fov=Ta*2*Math.atan(e),this.updateProjectionMatrix()}getFocalLength(){const t=Math.tan(ma*.5*this.fov);return .5*this.getFilmHeight()/t}getEffectiveFOV(){return Ta*2*Math.atan(Math.tan(ma*.5*this.fov)/this.zoom)}getFilmWidth(){return this.filmGauge*Math.min(this.aspect,1)}getFilmHeight(){return this.filmGauge/Math.max(this.aspect,1)}setViewOffset(t,e,n,i,s,a){this.aspect=t/e,this.view===null&&(this.view={enabled:!0,fullWidth:1,fullHeight:1,offsetX:0,offsetY:0,width:1,height:1}),this.view.enabled=!0,this.view.fullWidth=t,this.view.fullHeight=e,this.view.offsetX=n,this.view.offsetY=i,this.view.width=s,this.view.height=a,this.updateProjectionMatrix()}clearViewOffset(){this.view!==null&&(this.view.enabled=!1),this.updateProjectionMatrix()}updateProjectionMatrix(){const t=this.near;let e=t*Math.tan(ma*.5*this.fov)/this.zoom,n=2*e,i=this.aspect*n,s=-.5*i;const a=this.view;if(this.view!==null&&this.view.enabled){const c=a.fullWidth,l=a.fullHeight;s+=a.offsetX*i/c,e-=a.offsetY*n/l,i*=a.width/c,n*=a.height/l}const o=this.filmOffset;o!==0&&(s+=t*o/this.getFilmWidth()),this.projectionMatrix.makePerspective(s,s+i,e,e-n,t,this.far,this.coordinateSystem),this.projectionMatrixInverse.copy(this.projectionMatrix).invert()}toJSON(t){const e=super.toJSON(t);return e.object.fov=this.fov,e.object.zoom=this.zoom,e.object.near=this.near,e.object.far=this.far,e.object.focus=this.focus,e.object.aspect=this.aspect,this.view!==null&&(e.object.view=Object.assign({},this.view)),e.object.filmGauge=this.filmGauge,e.object.filmOffset=this.filmOffset,e}}const Gs=-90,Vs=1;class Im extends Je{constructor(t,e,n){super(),this.type="CubeCamera",this.renderTarget=n,this.coordinateSystem=null,this.activeMipmapLevel=0;const i=new Dn(Gs,Vs,t,e);i.layers=this.layers,this.add(i);const s=new Dn(Gs,Vs,t,e);s.layers=this.layers,this.add(s);const a=new Dn(Gs,Vs,t,e);a.layers=this.layers,this.add(a);const o=new Dn(Gs,Vs,t,e);o.layers=this.layers,this.add(o);const c=new Dn(Gs,Vs,t,e);c.layers=this.layers,this.add(c);const l=new Dn(Gs,Vs,t,e);l.layers=this.layers,this.add(l)}updateCoordinateSystem(){const t=this.coordinateSystem,e=this.children.concat(),[n,i,s,a,o,c]=e;for(const l of e)this.remove(l);if(t===2e3)n.up.set(0,1,0),n.lookAt(1,0,0),i.up.set(0,1,0),i.lookAt(-1,0,0),s.up.set(0,0,-1),s.lookAt(0,1,0),a.up.set(0,0,1),a.lookAt(0,-1,0),o.up.set(0,1,0),o.lookAt(0,0,1),c.up.set(0,1,0),c.lookAt(0,0,-1);else if(t===2001)n.up.set(0,-1,0),n.lookAt(-1,0,0),i.up.set(0,-1,0),i.lookAt(1,0,0),s.up.set(0,0,1),s.lookAt(0,1,0),a.up.set(0,0,-1),a.lookAt(0,-1,0),o.up.set(0,-1,0),o.lookAt(0,0,1),c.up.set(0,-1,0),c.lookAt(0,0,-1);else throw new Error("THREE.CubeCamera.updateCoordinateSystem(): Invalid coordinate system: "+t);for(const l of e)this.add(l),l.updateMatrixWorld()}update(t,e){this.parent===null&&this.updateMatrixWorld();const{renderTarget:n,activeMipmapLevel:i}=this;this.coordinateSystem!==t.coordinateSystem&&(this.coordinateSystem=t.coordinateSystem,this.updateCoordinateSystem());const[s,a,o,c,l,h]=this.children,u=t.getRenderTarget(),d=t.getActiveCubeFace(),f=t.getActiveMipmapLevel(),m=t.xr.enabled;t.xr.enabled=!1;const x=n.texture.generateMipmaps;n.texture.generateMipmaps=!1,t.setRenderTarget(n,0,i),t.render(e,s),t.setRenderTarget(n,1,i),t.render(e,a),t.setRenderTarget(n,2,i),t.render(e,o),t.setRenderTarget(n,3,i),t.render(e,c),t.setRenderTarget(n,4,i),t.render(e,l),n.texture.generateMipmaps=x,t.setRenderTarget(n,5,i),t.render(e,h),t.setRenderTarget(u,d,f),t.xr.enabled=m,n.texture.needsPMREMUpdate=!0}}class Uf extends Nn{constructor(t,e,n,i,s,a,o,c,l,h){t=t!==void 0?t:[],e=e!==void 0?e:301,super(t,e,n,i,s,a,o,c,l,h),this.isCubeTexture=!0,this.flipY=!1}get images(){return this.image}set images(t){this.image=t}}class Nm extends Qe{constructor(t=1,e={}){super(t,t,e),this.isWebGLCubeRenderTarget=!0;const n={width:t,height:t,depth:1},i=[n,n,n,n,n,n];e.encoding!==void 0&&(xa("THREE.WebGLCubeRenderTarget: option.encoding has been replaced by option.colorSpace."),e.colorSpace=e.encoding===3001?nn:""),this.texture=new Uf(i,e.mapping,e.wrapS,e.wrapT,e.magFilter,e.minFilter,e.format,e.type,e.anisotropy,e.colorSpace),this.texture.isRenderTargetTexture=!0,this.texture.generateMipmaps=e.generateMipmaps!==void 0?e.generateMipmaps:!1,this.texture.minFilter=e.minFilter!==void 0?e.minFilter:1006}fromEquirectangularTexture(t,e){this.texture.type=e.type,this.texture.colorSpace=e.colorSpace,this.texture.generateMipmaps=e.generateMipmaps,this.texture.minFilter=e.minFilter,this.texture.magFilter=e.magFilter;const n={uniforms:{tEquirect:{value:null}},vertexShader:`

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
			`},i=new Ln(5,5,5),s=new Un({name:"CubemapFromEquirect",uniforms:mr(n.uniforms),vertexShader:n.vertexShader,fragmentShader:n.fragmentShader,side:1,blending:0});s.uniforms.tEquirect.value=e;const a=new $t(i,s),o=e.minFilter;return e.minFilter===1008&&(e.minFilter=1006),new Im(1,10,this).update(t,a),e.minFilter=o,a.geometry.dispose(),a.material.dispose(),this}clear(t,e,n,i){const s=t.getRenderTarget();for(let a=0;a<6;a++)t.setRenderTarget(this,a),t.clear(e,n,i);t.setRenderTarget(s)}}const Bc=new C,Fm=new C,Um=new ie;class Ri{constructor(t=new C(1,0,0),e=0){this.isPlane=!0,this.normal=t,this.constant=e}set(t,e){return this.normal.copy(t),this.constant=e,this}setComponents(t,e,n,i){return this.normal.set(t,e,n),this.constant=i,this}setFromNormalAndCoplanarPoint(t,e){return this.normal.copy(t),this.constant=-e.dot(this.normal),this}setFromCoplanarPoints(t,e,n){const i=Bc.subVectors(n,e).cross(Fm.subVectors(t,e)).normalize();return this.setFromNormalAndCoplanarPoint(i,t),this}copy(t){return this.normal.copy(t.normal),this.constant=t.constant,this}normalize(){const t=1/this.normal.length();return this.normal.multiplyScalar(t),this.constant*=t,this}negate(){return this.constant*=-1,this.normal.negate(),this}distanceToPoint(t){return this.normal.dot(t)+this.constant}distanceToSphere(t){return this.distanceToPoint(t.center)-t.radius}projectPoint(t,e){return e.copy(t).addScaledVector(this.normal,-this.distanceToPoint(t))}intersectLine(t,e){const n=t.delta(Bc),i=this.normal.dot(n);if(i===0)return this.distanceToPoint(t.start)===0?e.copy(t.start):null;const s=-(t.start.dot(this.normal)+this.constant)/i;return s<0||s>1?null:e.copy(t.start).addScaledVector(n,s)}intersectsLine(t){const e=this.distanceToPoint(t.start),n=this.distanceToPoint(t.end);return e<0&&n>0||n<0&&e>0}intersectsBox(t){return t.intersectsPlane(this)}intersectsSphere(t){return t.intersectsPlane(this)}coplanarPoint(t){return t.copy(this.normal).multiplyScalar(-this.constant)}applyMatrix4(t,e){const n=e||Um.getNormalMatrix(t),i=this.coplanarPoint(Bc).applyMatrix4(t),s=this.normal.applyMatrix3(n).normalize();return this.constant=-i.dot(s),this}translate(t){return this.constant-=t.dot(this.normal),this}equals(t){return t.normal.equals(this.normal)&&t.constant===this.constant}clone(){return new this.constructor().copy(this)}}const os=new ns,$a=new C;class wh{constructor(t=new Ri,e=new Ri,n=new Ri,i=new Ri,s=new Ri,a=new Ri){this.planes=[t,e,n,i,s,a]}set(t,e,n,i,s,a){const o=this.planes;return o[0].copy(t),o[1].copy(e),o[2].copy(n),o[3].copy(i),o[4].copy(s),o[5].copy(a),this}copy(t){const e=this.planes;for(let n=0;n<6;n++)e[n].copy(t.planes[n]);return this}setFromProjectionMatrix(t,e=2e3){const n=this.planes,i=t.elements,s=i[0],a=i[1],o=i[2],c=i[3],l=i[4],h=i[5],u=i[6],d=i[7],f=i[8],m=i[9],x=i[10],g=i[11],p=i[12],_=i[13],v=i[14],S=i[15];if(n[0].setComponents(c-s,d-l,g-f,S-p).normalize(),n[1].setComponents(c+s,d+l,g+f,S+p).normalize(),n[2].setComponents(c+a,d+h,g+m,S+_).normalize(),n[3].setComponents(c-a,d-h,g-m,S-_).normalize(),n[4].setComponents(c-o,d-u,g-x,S-v).normalize(),e===2e3)n[5].setComponents(c+o,d+u,g+x,S+v).normalize();else if(e===2001)n[5].setComponents(o,u,x,v).normalize();else throw new Error("THREE.Frustum.setFromProjectionMatrix(): Invalid coordinate system: "+e);return this}intersectsObject(t){if(t.boundingSphere!==void 0)t.boundingSphere===null&&t.computeBoundingSphere(),os.copy(t.boundingSphere).applyMatrix4(t.matrixWorld);else{const e=t.geometry;e.boundingSphere===null&&e.computeBoundingSphere(),os.copy(e.boundingSphere).applyMatrix4(t.matrixWorld)}return this.intersectsSphere(os)}intersectsSprite(t){return os.center.set(0,0,0),os.radius=.7071067811865476,os.applyMatrix4(t.matrixWorld),this.intersectsSphere(os)}intersectsSphere(t){const e=this.planes,n=t.center,i=-t.radius;for(let s=0;s<6;s++)if(e[s].distanceToPoint(n)<i)return!1;return!0}intersectsBox(t){const e=this.planes;for(let n=0;n<6;n++){const i=e[n];if($a.x=i.normal.x>0?t.max.x:t.min.x,$a.y=i.normal.y>0?t.max.y:t.min.y,$a.z=i.normal.z>0?t.max.z:t.min.z,i.distanceToPoint($a)<0)return!1}return!0}containsPoint(t){const e=this.planes;for(let n=0;n<6;n++)if(e[n].distanceToPoint(t)<0)return!1;return!0}clone(){return new this.constructor().copy(this)}}function Bf(){let r=null,t=!1,e=null,n=null;function i(s,a){e(s,a),n=r.requestAnimationFrame(i)}return{start:function(){t!==!0&&e!==null&&(n=r.requestAnimationFrame(i),t=!0)},stop:function(){r.cancelAnimationFrame(n),t=!1},setAnimationLoop:function(s){e=s},setContext:function(s){r=s}}}function Bm(r,t){const e=t.isWebGL2,n=new WeakMap;function i(l,h){const u=l.array,d=l.usage,f=u.byteLength,m=r.createBuffer();r.bindBuffer(h,m),r.bufferData(h,u,d),l.onUploadCallback();let x;if(u instanceof Float32Array)x=r.FLOAT;else if(u instanceof Uint16Array)if(l.isFloat16BufferAttribute)if(e)x=r.HALF_FLOAT;else throw new Error("THREE.WebGLAttributes: Usage of Float16BufferAttribute requires WebGL2.");else x=r.UNSIGNED_SHORT;else if(u instanceof Int16Array)x=r.SHORT;else if(u instanceof Uint32Array)x=r.UNSIGNED_INT;else if(u instanceof Int32Array)x=r.INT;else if(u instanceof Int8Array)x=r.BYTE;else if(u instanceof Uint8Array)x=r.UNSIGNED_BYTE;else if(u instanceof Uint8ClampedArray)x=r.UNSIGNED_BYTE;else throw new Error("THREE.WebGLAttributes: Unsupported buffer data format: "+u);return{buffer:m,type:x,bytesPerElement:u.BYTES_PER_ELEMENT,version:l.version,size:f}}function s(l,h,u){const d=h.array,f=h._updateRange,m=h.updateRanges;if(r.bindBuffer(u,l),f.count===-1&&m.length===0&&r.bufferSubData(u,0,d),m.length!==0){for(let x=0,g=m.length;x<g;x++){const p=m[x];e?r.bufferSubData(u,p.start*d.BYTES_PER_ELEMENT,d,p.start,p.count):r.bufferSubData(u,p.start*d.BYTES_PER_ELEMENT,d.subarray(p.start,p.start+p.count))}h.clearUpdateRanges()}f.count!==-1&&(e?r.bufferSubData(u,f.offset*d.BYTES_PER_ELEMENT,d,f.offset,f.count):r.bufferSubData(u,f.offset*d.BYTES_PER_ELEMENT,d.subarray(f.offset,f.offset+f.count)),f.count=-1),h.onUploadCallback()}function a(l){return l.isInterleavedBufferAttribute&&(l=l.data),n.get(l)}function o(l){l.isInterleavedBufferAttribute&&(l=l.data);const h=n.get(l);h&&(r.deleteBuffer(h.buffer),n.delete(l))}function c(l,h){if(l.isGLBufferAttribute){const d=n.get(l);(!d||d.version<l.version)&&n.set(l,{buffer:l.buffer,type:l.type,bytesPerElement:l.elementSize,version:l.version});return}l.isInterleavedBufferAttribute&&(l=l.data);const u=n.get(l);if(u===void 0)n.set(l,i(l,h));else if(u.version<l.version){if(u.size!==l.array.byteLength)throw new Error("THREE.WebGLAttributes: The size of the buffer attribute's array buffer does not match the original size. Resizing buffer attributes is not supported.");s(u.buffer,l,h),u.version=l.version}}return{get:a,remove:o,update:c}}class lc extends we{constructor(t=1,e=1,n=1,i=1){super(),this.type="PlaneGeometry",this.parameters={width:t,height:e,widthSegments:n,heightSegments:i};const s=t/2,a=e/2,o=Math.floor(n),c=Math.floor(i),l=o+1,h=c+1,u=t/o,d=e/c,f=[],m=[],x=[],g=[];for(let p=0;p<h;p++){const _=p*d-a;for(let v=0;v<l;v++){const S=v*u-s;m.push(S,-_,0),x.push(0,0,1),g.push(v/o),g.push(1-p/c)}}for(let p=0;p<c;p++)for(let _=0;_<o;_++){const v=_+l*p,S=_+l*(p+1),M=_+1+l*(p+1),y=_+1+l*p;f.push(v,S,y),f.push(S,M,y)}this.setIndex(f),this.setAttribute("position",new se(m,3)),this.setAttribute("normal",new se(x,3)),this.setAttribute("uv",new se(g,2))}copy(t){return super.copy(t),this.parameters=Object.assign({},t.parameters),this}static fromJSON(t){return new lc(t.width,t.height,t.widthSegments,t.heightSegments)}}var zm=`#ifdef USE_ALPHAHASH
	if ( diffuseColor.a < getAlphaHashThreshold( vPosition ) ) discard;
#endif`,Om=`#ifdef USE_ALPHAHASH
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
#endif`,Gm=`#ifdef USE_ALPHAMAP
	diffuseColor.a *= texture2D( alphaMap, vAlphaMapUv ).g;
#endif`,Vm=`#ifdef USE_ALPHAMAP
	uniform sampler2D alphaMap;
#endif`,km=`#ifdef USE_ALPHATEST
	if ( diffuseColor.a < alphaTest ) discard;
#endif`,Hm=`#ifdef USE_ALPHATEST
	uniform float alphaTest;
#endif`,Wm=`#ifdef USE_AOMAP
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
#endif`,Xm=`#ifdef USE_AOMAP
	uniform sampler2D aoMap;
	uniform float aoMapIntensity;
#endif`,Ym=`#ifdef USE_BATCHING
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
#endif`,qm=`#ifdef USE_BATCHING
	mat4 batchingMatrix = getBatchingMatrix( batchId );
#endif`,Zm=`vec3 transformed = vec3( position );
#ifdef USE_ALPHAHASH
	vPosition = vec3( position );
#endif`,jm=`vec3 objectNormal = vec3( normal );
#ifdef USE_TANGENT
	vec3 objectTangent = vec3( tangent.xyz );
#endif`,$m=`float G_BlinnPhong_Implicit( ) {
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
} // validated`,Km=`#ifdef USE_IRIDESCENCE
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
#endif`,Jm=`#ifdef USE_BUMPMAP
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
#endif`,Qm=`#if NUM_CLIPPING_PLANES > 0
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
#endif`,tg=`#if NUM_CLIPPING_PLANES > 0
	varying vec3 vClipPosition;
	uniform vec4 clippingPlanes[ NUM_CLIPPING_PLANES ];
#endif`,eg=`#if NUM_CLIPPING_PLANES > 0
	varying vec3 vClipPosition;
#endif`,ng=`#if NUM_CLIPPING_PLANES > 0
	vClipPosition = - mvPosition.xyz;
#endif`,ig=`#if defined( USE_COLOR_ALPHA )
	diffuseColor *= vColor;
#elif defined( USE_COLOR )
	diffuseColor.rgb *= vColor;
#endif`,sg=`#if defined( USE_COLOR_ALPHA )
	varying vec4 vColor;
#elif defined( USE_COLOR )
	varying vec3 vColor;
#endif`,rg=`#if defined( USE_COLOR_ALPHA )
	varying vec4 vColor;
#elif defined( USE_COLOR ) || defined( USE_INSTANCING_COLOR )
	varying vec3 vColor;
#endif`,ag=`#if defined( USE_COLOR_ALPHA )
	vColor = vec4( 1.0 );
#elif defined( USE_COLOR ) || defined( USE_INSTANCING_COLOR )
	vColor = vec3( 1.0 );
#endif
#ifdef USE_COLOR
	vColor *= color;
#endif
#ifdef USE_INSTANCING_COLOR
	vColor.xyz *= instanceColor.xyz;
#endif`,og=`#define PI 3.141592653589793
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
} // validated`,cg=`#ifdef ENVMAP_TYPE_CUBE_UV
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
#endif`,lg=`vec3 transformedNormal = objectNormal;
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
#endif`,hg=`#ifdef USE_DISPLACEMENTMAP
	uniform sampler2D displacementMap;
	uniform float displacementScale;
	uniform float displacementBias;
#endif`,ug=`#ifdef USE_DISPLACEMENTMAP
	transformed += normalize( objectNormal ) * ( texture2D( displacementMap, vDisplacementMapUv ).x * displacementScale + displacementBias );
#endif`,dg=`#ifdef USE_EMISSIVEMAP
	vec4 emissiveColor = texture2D( emissiveMap, vEmissiveMapUv );
	totalEmissiveRadiance *= emissiveColor.rgb;
#endif`,fg=`#ifdef USE_EMISSIVEMAP
	uniform sampler2D emissiveMap;
#endif`,pg="gl_FragColor = linearToOutputTexel( gl_FragColor );",mg=`
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
}`,gg=`#ifdef USE_ENVMAP
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
#endif`,xg=`#ifdef USE_ENVMAP
	uniform float envMapIntensity;
	uniform float flipEnvMap;
	#ifdef ENVMAP_TYPE_CUBE
		uniform samplerCube envMap;
	#else
		uniform sampler2D envMap;
	#endif
	
#endif`,_g=`#ifdef USE_ENVMAP
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
#endif`,vg=`#ifdef USE_ENVMAP
	#if defined( USE_BUMPMAP ) || defined( USE_NORMALMAP ) || defined( PHONG ) || defined( LAMBERT )
		#define ENV_WORLDPOS
	#endif
	#ifdef ENV_WORLDPOS
		
		varying vec3 vWorldPosition;
	#else
		varying vec3 vReflect;
		uniform float refractionRatio;
	#endif
#endif`,Sg=`#ifdef USE_ENVMAP
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
#endif`,Mg=`#ifdef USE_FOG
	vFogDepth = - mvPosition.z;
#endif`,yg=`#ifdef USE_FOG
	varying float vFogDepth;
#endif`,Eg=`#ifdef USE_FOG
	#ifdef FOG_EXP2
		float fogFactor = 1.0 - exp( - fogDensity * fogDensity * vFogDepth * vFogDepth );
	#else
		float fogFactor = smoothstep( fogNear, fogFar, vFogDepth );
	#endif
	gl_FragColor.rgb = mix( gl_FragColor.rgb, fogColor, fogFactor );
#endif`,wg=`#ifdef USE_FOG
	uniform vec3 fogColor;
	varying float vFogDepth;
	#ifdef FOG_EXP2
		uniform float fogDensity;
	#else
		uniform float fogNear;
		uniform float fogFar;
	#endif
#endif`,Ag=`#ifdef USE_GRADIENTMAP
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
}`,Tg=`#ifdef USE_LIGHTMAP
	vec4 lightMapTexel = texture2D( lightMap, vLightMapUv );
	vec3 lightMapIrradiance = lightMapTexel.rgb * lightMapIntensity;
	reflectedLight.indirectDiffuse += lightMapIrradiance;
#endif`,Cg=`#ifdef USE_LIGHTMAP
	uniform sampler2D lightMap;
	uniform float lightMapIntensity;
#endif`,bg=`LambertMaterial material;
material.diffuseColor = diffuseColor.rgb;
material.specularStrength = specularStrength;`,Rg=`varying vec3 vViewPosition;
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
#define RE_IndirectDiffuse		RE_IndirectDiffuse_Lambert`,Pg=`uniform bool receiveShadow;
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
#endif`,Lg=`#ifdef USE_ENVMAP
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
#endif`,Dg=`ToonMaterial material;
material.diffuseColor = diffuseColor.rgb;`,Ig=`varying vec3 vViewPosition;
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
#define RE_IndirectDiffuse		RE_IndirectDiffuse_Toon`,Ng=`BlinnPhongMaterial material;
material.diffuseColor = diffuseColor.rgb;
material.specularColor = specular;
material.specularShininess = shininess;
material.specularStrength = specularStrength;`,Fg=`varying vec3 vViewPosition;
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
#define RE_IndirectDiffuse		RE_IndirectDiffuse_BlinnPhong`,Ug=`PhysicalMaterial material;
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
#endif`,Bg=`struct PhysicalMaterial {
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
}`,zg=`
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
#endif`,Og=`#if defined( RE_IndirectDiffuse )
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
#endif`,Gg=`#if defined( RE_IndirectDiffuse )
	RE_IndirectDiffuse( irradiance, geometryPosition, geometryNormal, geometryViewDir, geometryClearcoatNormal, material, reflectedLight );
#endif
#if defined( RE_IndirectSpecular )
	RE_IndirectSpecular( radiance, iblIrradiance, clearcoatRadiance, geometryPosition, geometryNormal, geometryViewDir, geometryClearcoatNormal, material, reflectedLight );
#endif`,Vg=`#if defined( USE_LOGDEPTHBUF ) && defined( USE_LOGDEPTHBUF_EXT )
	gl_FragDepthEXT = vIsPerspective == 0.0 ? gl_FragCoord.z : log2( vFragDepth ) * logDepthBufFC * 0.5;
#endif`,kg=`#if defined( USE_LOGDEPTHBUF ) && defined( USE_LOGDEPTHBUF_EXT )
	uniform float logDepthBufFC;
	varying float vFragDepth;
	varying float vIsPerspective;
#endif`,Hg=`#ifdef USE_LOGDEPTHBUF
	#ifdef USE_LOGDEPTHBUF_EXT
		varying float vFragDepth;
		varying float vIsPerspective;
	#else
		uniform float logDepthBufFC;
	#endif
#endif`,Wg=`#ifdef USE_LOGDEPTHBUF
	#ifdef USE_LOGDEPTHBUF_EXT
		vFragDepth = 1.0 + gl_Position.w;
		vIsPerspective = float( isPerspectiveMatrix( projectionMatrix ) );
	#else
		if ( isPerspectiveMatrix( projectionMatrix ) ) {
			gl_Position.z = log2( max( EPSILON, gl_Position.w + 1.0 ) ) * logDepthBufFC - 1.0;
			gl_Position.z *= gl_Position.w;
		}
	#endif
#endif`,Xg=`#ifdef USE_MAP
	vec4 sampledDiffuseColor = texture2D( map, vMapUv );
	#ifdef DECODE_VIDEO_TEXTURE
		sampledDiffuseColor = vec4( mix( pow( sampledDiffuseColor.rgb * 0.9478672986 + vec3( 0.0521327014 ), vec3( 2.4 ) ), sampledDiffuseColor.rgb * 0.0773993808, vec3( lessThanEqual( sampledDiffuseColor.rgb, vec3( 0.04045 ) ) ) ), sampledDiffuseColor.w );
	
	#endif
	diffuseColor *= sampledDiffuseColor;
#endif`,Yg=`#ifdef USE_MAP
	uniform sampler2D map;
#endif`,qg=`#if defined( USE_MAP ) || defined( USE_ALPHAMAP )
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
#endif`,Zg=`#if defined( USE_POINTS_UV )
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
#endif`,jg=`float metalnessFactor = metalness;
#ifdef USE_METALNESSMAP
	vec4 texelMetalness = texture2D( metalnessMap, vMetalnessMapUv );
	metalnessFactor *= texelMetalness.b;
#endif`,$g=`#ifdef USE_METALNESSMAP
	uniform sampler2D metalnessMap;
#endif`,Kg=`#if defined( USE_MORPHCOLORS ) && defined( MORPHTARGETS_TEXTURE )
	vColor *= morphTargetBaseInfluence;
	for ( int i = 0; i < MORPHTARGETS_COUNT; i ++ ) {
		#if defined( USE_COLOR_ALPHA )
			if ( morphTargetInfluences[ i ] != 0.0 ) vColor += getMorph( gl_VertexID, i, 2 ) * morphTargetInfluences[ i ];
		#elif defined( USE_COLOR )
			if ( morphTargetInfluences[ i ] != 0.0 ) vColor += getMorph( gl_VertexID, i, 2 ).rgb * morphTargetInfluences[ i ];
		#endif
	}
#endif`,Jg=`#ifdef USE_MORPHNORMALS
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
#endif`,Qg=`#ifdef USE_MORPHTARGETS
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
#endif`,t0=`#ifdef USE_MORPHTARGETS
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
#endif`,e0=`float faceDirection = gl_FrontFacing ? 1.0 : - 1.0;
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
vec3 nonPerturbedNormal = normal;`,n0=`#ifdef USE_NORMALMAP_OBJECTSPACE
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
#endif`,i0=`#ifndef FLAT_SHADED
	varying vec3 vNormal;
	#ifdef USE_TANGENT
		varying vec3 vTangent;
		varying vec3 vBitangent;
	#endif
#endif`,s0=`#ifndef FLAT_SHADED
	varying vec3 vNormal;
	#ifdef USE_TANGENT
		varying vec3 vTangent;
		varying vec3 vBitangent;
	#endif
#endif`,r0=`#ifndef FLAT_SHADED
	vNormal = normalize( transformedNormal );
	#ifdef USE_TANGENT
		vTangent = normalize( transformedTangent );
		vBitangent = normalize( cross( vNormal, vTangent ) * tangent.w );
	#endif
#endif`,a0=`#ifdef USE_NORMALMAP
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
#endif`,o0=`#ifdef USE_CLEARCOAT
	vec3 clearcoatNormal = nonPerturbedNormal;
#endif`,c0=`#ifdef USE_CLEARCOAT_NORMALMAP
	vec3 clearcoatMapN = texture2D( clearcoatNormalMap, vClearcoatNormalMapUv ).xyz * 2.0 - 1.0;
	clearcoatMapN.xy *= clearcoatNormalScale;
	clearcoatNormal = normalize( tbn2 * clearcoatMapN );
#endif`,l0=`#ifdef USE_CLEARCOATMAP
	uniform sampler2D clearcoatMap;
#endif
#ifdef USE_CLEARCOAT_NORMALMAP
	uniform sampler2D clearcoatNormalMap;
	uniform vec2 clearcoatNormalScale;
#endif
#ifdef USE_CLEARCOAT_ROUGHNESSMAP
	uniform sampler2D clearcoatRoughnessMap;
#endif`,h0=`#ifdef USE_IRIDESCENCEMAP
	uniform sampler2D iridescenceMap;
#endif
#ifdef USE_IRIDESCENCE_THICKNESSMAP
	uniform sampler2D iridescenceThicknessMap;
#endif`,u0=`#ifdef OPAQUE
diffuseColor.a = 1.0;
#endif
#ifdef USE_TRANSMISSION
diffuseColor.a *= material.transmissionAlpha;
#endif
gl_FragColor = vec4( outgoingLight, diffuseColor.a );`,d0=`vec3 packNormalToRGB( const in vec3 normal ) {
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
}`,f0=`#ifdef PREMULTIPLIED_ALPHA
	gl_FragColor.rgb *= gl_FragColor.a;
#endif`,p0=`vec4 mvPosition = vec4( transformed, 1.0 );
#ifdef USE_BATCHING
	mvPosition = batchingMatrix * mvPosition;
#endif
#ifdef USE_INSTANCING
	mvPosition = instanceMatrix * mvPosition;
#endif
mvPosition = modelViewMatrix * mvPosition;
gl_Position = projectionMatrix * mvPosition;`,m0=`#ifdef DITHERING
	gl_FragColor.rgb = dithering( gl_FragColor.rgb );
#endif`,g0=`#ifdef DITHERING
	vec3 dithering( vec3 color ) {
		float grid_position = rand( gl_FragCoord.xy );
		vec3 dither_shift_RGB = vec3( 0.25 / 255.0, -0.25 / 255.0, 0.25 / 255.0 );
		dither_shift_RGB = mix( 2.0 * dither_shift_RGB, -2.0 * dither_shift_RGB, grid_position );
		return color + dither_shift_RGB;
	}
#endif`,x0=`float roughnessFactor = roughness;
#ifdef USE_ROUGHNESSMAP
	vec4 texelRoughness = texture2D( roughnessMap, vRoughnessMapUv );
	roughnessFactor *= texelRoughness.g;
#endif`,_0=`#ifdef USE_ROUGHNESSMAP
	uniform sampler2D roughnessMap;
#endif`,v0=`#if NUM_SPOT_LIGHT_COORDS > 0
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
#endif`,S0=`#if NUM_SPOT_LIGHT_COORDS > 0
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
#endif`,M0=`#if ( defined( USE_SHADOWMAP ) && ( NUM_DIR_LIGHT_SHADOWS > 0 || NUM_POINT_LIGHT_SHADOWS > 0 ) ) || ( NUM_SPOT_LIGHT_COORDS > 0 )
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
#endif`,y0=`float getShadowMask() {
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
}`,E0=`#ifdef USE_SKINNING
	mat4 boneMatX = getBoneMatrix( skinIndex.x );
	mat4 boneMatY = getBoneMatrix( skinIndex.y );
	mat4 boneMatZ = getBoneMatrix( skinIndex.z );
	mat4 boneMatW = getBoneMatrix( skinIndex.w );
#endif`,w0=`#ifdef USE_SKINNING
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
#endif`,A0=`#ifdef USE_SKINNING
	vec4 skinVertex = bindMatrix * vec4( transformed, 1.0 );
	vec4 skinned = vec4( 0.0 );
	skinned += boneMatX * skinVertex * skinWeight.x;
	skinned += boneMatY * skinVertex * skinWeight.y;
	skinned += boneMatZ * skinVertex * skinWeight.z;
	skinned += boneMatW * skinVertex * skinWeight.w;
	transformed = ( bindMatrixInverse * skinned ).xyz;
#endif`,T0=`#ifdef USE_SKINNING
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
#endif`,C0=`float specularStrength;
#ifdef USE_SPECULARMAP
	vec4 texelSpecular = texture2D( specularMap, vSpecularMapUv );
	specularStrength = texelSpecular.r;
#else
	specularStrength = 1.0;
#endif`,b0=`#ifdef USE_SPECULARMAP
	uniform sampler2D specularMap;
#endif`,R0=`#if defined( TONE_MAPPING )
	gl_FragColor.rgb = toneMapping( gl_FragColor.rgb );
#endif`,P0=`#ifndef saturate
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
vec3 CustomToneMapping( vec3 color ) { return color; }`,L0=`#ifdef USE_TRANSMISSION
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
#endif`,D0=`#ifdef USE_TRANSMISSION
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
#endif`,I0=`#if defined( USE_UV ) || defined( USE_ANISOTROPY )
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
#endif`,N0=`#if defined( USE_UV ) || defined( USE_ANISOTROPY )
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
#endif`,F0=`#if defined( USE_UV ) || defined( USE_ANISOTROPY )
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
#endif`,U0=`#if defined( USE_ENVMAP ) || defined( DISTANCE ) || defined ( USE_SHADOWMAP ) || defined ( USE_TRANSMISSION ) || NUM_SPOT_LIGHT_COORDS > 0
	vec4 worldPosition = vec4( transformed, 1.0 );
	#ifdef USE_BATCHING
		worldPosition = batchingMatrix * worldPosition;
	#endif
	#ifdef USE_INSTANCING
		worldPosition = instanceMatrix * worldPosition;
	#endif
	worldPosition = modelMatrix * worldPosition;
#endif`;const B0=`varying vec2 vUv;
uniform mat3 uvTransform;
void main() {
	vUv = ( uvTransform * vec3( uv, 1 ) ).xy;
	gl_Position = vec4( position.xy, 1.0, 1.0 );
}`,z0=`uniform sampler2D t2D;
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
}`,O0=`varying vec3 vWorldDirection;
#include <common>
void main() {
	vWorldDirection = transformDirection( position, modelMatrix );
	#include <begin_vertex>
	#include <project_vertex>
	gl_Position.z = gl_Position.w;
}`,G0=`#ifdef ENVMAP_TYPE_CUBE
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
}`,V0=`varying vec3 vWorldDirection;
#include <common>
void main() {
	vWorldDirection = transformDirection( position, modelMatrix );
	#include <begin_vertex>
	#include <project_vertex>
	gl_Position.z = gl_Position.w;
}`,k0=`uniform samplerCube tCube;
uniform float tFlip;
uniform float opacity;
varying vec3 vWorldDirection;
void main() {
	vec4 texColor = textureCube( tCube, vec3( tFlip * vWorldDirection.x, vWorldDirection.yz ) );
	gl_FragColor = texColor;
	gl_FragColor.a *= opacity;
	#include <tonemapping_fragment>
	#include <colorspace_fragment>
}`,H0=`#include <common>
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
}`,W0=`#if DEPTH_PACKING == 3200
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
}`,X0=`#define DISTANCE
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
}`,Y0=`#define DISTANCE
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
}`,q0=`varying vec3 vWorldDirection;
#include <common>
void main() {
	vWorldDirection = transformDirection( position, modelMatrix );
	#include <begin_vertex>
	#include <project_vertex>
}`,Z0=`uniform sampler2D tEquirect;
varying vec3 vWorldDirection;
#include <common>
void main() {
	vec3 direction = normalize( vWorldDirection );
	vec2 sampleUV = equirectUv( direction );
	gl_FragColor = texture2D( tEquirect, sampleUV );
	#include <tonemapping_fragment>
	#include <colorspace_fragment>
}`,j0=`uniform float scale;
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
}`,$0=`uniform vec3 diffuse;
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
}`,K0=`#include <common>
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
}`,J0=`uniform vec3 diffuse;
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
}`,Q0=`#define LAMBERT
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
}`,tx=`#define LAMBERT
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
}`,ex=`#define MATCAP
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
}`,nx=`#define MATCAP
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
}`,ix=`#define NORMAL
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
}`,sx=`#define NORMAL
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
}`,rx=`#define PHONG
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
}`,ax=`#define PHONG
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
}`,ox=`#define STANDARD
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
}`,cx=`#define STANDARD
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
}`,lx=`#define TOON
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
}`,hx=`#define TOON
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
}`,ux=`uniform float size;
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
}`,dx=`uniform vec3 diffuse;
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
}`,fx=`#include <common>
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
}`,px=`uniform vec3 color;
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
}`,mx=`uniform float rotation;
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
}`,gx=`uniform vec3 diffuse;
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
}`,ee={alphahash_fragment:zm,alphahash_pars_fragment:Om,alphamap_fragment:Gm,alphamap_pars_fragment:Vm,alphatest_fragment:km,alphatest_pars_fragment:Hm,aomap_fragment:Wm,aomap_pars_fragment:Xm,batching_pars_vertex:Ym,batching_vertex:qm,begin_vertex:Zm,beginnormal_vertex:jm,bsdfs:$m,iridescence_fragment:Km,bumpmap_pars_fragment:Jm,clipping_planes_fragment:Qm,clipping_planes_pars_fragment:tg,clipping_planes_pars_vertex:eg,clipping_planes_vertex:ng,color_fragment:ig,color_pars_fragment:sg,color_pars_vertex:rg,color_vertex:ag,common:og,cube_uv_reflection_fragment:cg,defaultnormal_vertex:lg,displacementmap_pars_vertex:hg,displacementmap_vertex:ug,emissivemap_fragment:dg,emissivemap_pars_fragment:fg,colorspace_fragment:pg,colorspace_pars_fragment:mg,envmap_fragment:gg,envmap_common_pars_fragment:xg,envmap_pars_fragment:_g,envmap_pars_vertex:vg,envmap_physical_pars_fragment:Lg,envmap_vertex:Sg,fog_vertex:Mg,fog_pars_vertex:yg,fog_fragment:Eg,fog_pars_fragment:wg,gradientmap_pars_fragment:Ag,lightmap_fragment:Tg,lightmap_pars_fragment:Cg,lights_lambert_fragment:bg,lights_lambert_pars_fragment:Rg,lights_pars_begin:Pg,lights_toon_fragment:Dg,lights_toon_pars_fragment:Ig,lights_phong_fragment:Ng,lights_phong_pars_fragment:Fg,lights_physical_fragment:Ug,lights_physical_pars_fragment:Bg,lights_fragment_begin:zg,lights_fragment_maps:Og,lights_fragment_end:Gg,logdepthbuf_fragment:Vg,logdepthbuf_pars_fragment:kg,logdepthbuf_pars_vertex:Hg,logdepthbuf_vertex:Wg,map_fragment:Xg,map_pars_fragment:Yg,map_particle_fragment:qg,map_particle_pars_fragment:Zg,metalnessmap_fragment:jg,metalnessmap_pars_fragment:$g,morphcolor_vertex:Kg,morphnormal_vertex:Jg,morphtarget_pars_vertex:Qg,morphtarget_vertex:t0,normal_fragment_begin:e0,normal_fragment_maps:n0,normal_pars_fragment:i0,normal_pars_vertex:s0,normal_vertex:r0,normalmap_pars_fragment:a0,clearcoat_normal_fragment_begin:o0,clearcoat_normal_fragment_maps:c0,clearcoat_pars_fragment:l0,iridescence_pars_fragment:h0,opaque_fragment:u0,packing:d0,premultiplied_alpha_fragment:f0,project_vertex:p0,dithering_fragment:m0,dithering_pars_fragment:g0,roughnessmap_fragment:x0,roughnessmap_pars_fragment:_0,shadowmap_pars_fragment:v0,shadowmap_pars_vertex:S0,shadowmap_vertex:M0,shadowmask_pars_fragment:y0,skinbase_vertex:E0,skinning_pars_vertex:w0,skinning_vertex:A0,skinnormal_vertex:T0,specularmap_fragment:C0,specularmap_pars_fragment:b0,tonemapping_fragment:R0,tonemapping_pars_fragment:P0,transmission_fragment:L0,transmission_pars_fragment:D0,uv_pars_fragment:I0,uv_pars_vertex:N0,uv_vertex:F0,worldpos_vertex:U0,background_vert:B0,background_frag:z0,backgroundCube_vert:O0,backgroundCube_frag:G0,cube_vert:V0,cube_frag:k0,depth_vert:H0,depth_frag:W0,distanceRGBA_vert:X0,distanceRGBA_frag:Y0,equirect_vert:q0,equirect_frag:Z0,linedashed_vert:j0,linedashed_frag:$0,meshbasic_vert:K0,meshbasic_frag:J0,meshlambert_vert:Q0,meshlambert_frag:tx,meshmatcap_vert:ex,meshmatcap_frag:nx,meshnormal_vert:ix,meshnormal_frag:sx,meshphong_vert:rx,meshphong_frag:ax,meshphysical_vert:ox,meshphysical_frag:cx,meshtoon_vert:lx,meshtoon_frag:hx,points_vert:ux,points_frag:dx,shadow_vert:fx,shadow_frag:px,sprite_vert:mx,sprite_frag:gx},pt={common:{diffuse:{value:new Kt(16777215)},opacity:{value:1},map:{value:null},mapTransform:{value:new ie},alphaMap:{value:null},alphaMapTransform:{value:new ie},alphaTest:{value:0}},specularmap:{specularMap:{value:null},specularMapTransform:{value:new ie}},envmap:{envMap:{value:null},flipEnvMap:{value:-1},reflectivity:{value:1},ior:{value:1.5},refractionRatio:{value:.98}},aomap:{aoMap:{value:null},aoMapIntensity:{value:1},aoMapTransform:{value:new ie}},lightmap:{lightMap:{value:null},lightMapIntensity:{value:1},lightMapTransform:{value:new ie}},bumpmap:{bumpMap:{value:null},bumpMapTransform:{value:new ie},bumpScale:{value:1}},normalmap:{normalMap:{value:null},normalMapTransform:{value:new ie},normalScale:{value:new yt(1,1)}},displacementmap:{displacementMap:{value:null},displacementMapTransform:{value:new ie},displacementScale:{value:1},displacementBias:{value:0}},emissivemap:{emissiveMap:{value:null},emissiveMapTransform:{value:new ie}},metalnessmap:{metalnessMap:{value:null},metalnessMapTransform:{value:new ie}},roughnessmap:{roughnessMap:{value:null},roughnessMapTransform:{value:new ie}},gradientmap:{gradientMap:{value:null}},fog:{fogDensity:{value:25e-5},fogNear:{value:1},fogFar:{value:2e3},fogColor:{value:new Kt(16777215)}},lights:{ambientLightColor:{value:[]},lightProbe:{value:[]},directionalLights:{value:[],properties:{direction:{},color:{}}},directionalLightShadows:{value:[],properties:{shadowBias:{},shadowNormalBias:{},shadowRadius:{},shadowMapSize:{}}},directionalShadowMap:{value:[]},directionalShadowMatrix:{value:[]},spotLights:{value:[],properties:{color:{},position:{},direction:{},distance:{},coneCos:{},penumbraCos:{},decay:{}}},spotLightShadows:{value:[],properties:{shadowBias:{},shadowNormalBias:{},shadowRadius:{},shadowMapSize:{}}},spotLightMap:{value:[]},spotShadowMap:{value:[]},spotLightMatrix:{value:[]},pointLights:{value:[],properties:{color:{},position:{},decay:{},distance:{}}},pointLightShadows:{value:[],properties:{shadowBias:{},shadowNormalBias:{},shadowRadius:{},shadowMapSize:{},shadowCameraNear:{},shadowCameraFar:{}}},pointShadowMap:{value:[]},pointShadowMatrix:{value:[]},hemisphereLights:{value:[],properties:{direction:{},skyColor:{},groundColor:{}}},rectAreaLights:{value:[],properties:{color:{},position:{},width:{},height:{}}},ltc_1:{value:null},ltc_2:{value:null}},points:{diffuse:{value:new Kt(16777215)},opacity:{value:1},size:{value:1},scale:{value:1},map:{value:null},alphaMap:{value:null},alphaMapTransform:{value:new ie},alphaTest:{value:0},uvTransform:{value:new ie}},sprite:{diffuse:{value:new Kt(16777215)},opacity:{value:1},center:{value:new yt(.5,.5)},rotation:{value:0},map:{value:null},mapTransform:{value:new ie},alphaMap:{value:null},alphaMapTransform:{value:new ie},alphaTest:{value:0}}},li={basic:{uniforms:fn([pt.common,pt.specularmap,pt.envmap,pt.aomap,pt.lightmap,pt.fog]),vertexShader:ee.meshbasic_vert,fragmentShader:ee.meshbasic_frag},lambert:{uniforms:fn([pt.common,pt.specularmap,pt.envmap,pt.aomap,pt.lightmap,pt.emissivemap,pt.bumpmap,pt.normalmap,pt.displacementmap,pt.fog,pt.lights,{emissive:{value:new Kt(0)}}]),vertexShader:ee.meshlambert_vert,fragmentShader:ee.meshlambert_frag},phong:{uniforms:fn([pt.common,pt.specularmap,pt.envmap,pt.aomap,pt.lightmap,pt.emissivemap,pt.bumpmap,pt.normalmap,pt.displacementmap,pt.fog,pt.lights,{emissive:{value:new Kt(0)},specular:{value:new Kt(1118481)},shininess:{value:30}}]),vertexShader:ee.meshphong_vert,fragmentShader:ee.meshphong_frag},standard:{uniforms:fn([pt.common,pt.envmap,pt.aomap,pt.lightmap,pt.emissivemap,pt.bumpmap,pt.normalmap,pt.displacementmap,pt.roughnessmap,pt.metalnessmap,pt.fog,pt.lights,{emissive:{value:new Kt(0)},roughness:{value:1},metalness:{value:0},envMapIntensity:{value:1}}]),vertexShader:ee.meshphysical_vert,fragmentShader:ee.meshphysical_frag},toon:{uniforms:fn([pt.common,pt.aomap,pt.lightmap,pt.emissivemap,pt.bumpmap,pt.normalmap,pt.displacementmap,pt.gradientmap,pt.fog,pt.lights,{emissive:{value:new Kt(0)}}]),vertexShader:ee.meshtoon_vert,fragmentShader:ee.meshtoon_frag},matcap:{uniforms:fn([pt.common,pt.bumpmap,pt.normalmap,pt.displacementmap,pt.fog,{matcap:{value:null}}]),vertexShader:ee.meshmatcap_vert,fragmentShader:ee.meshmatcap_frag},points:{uniforms:fn([pt.points,pt.fog]),vertexShader:ee.points_vert,fragmentShader:ee.points_frag},dashed:{uniforms:fn([pt.common,pt.fog,{scale:{value:1},dashSize:{value:1},totalSize:{value:2}}]),vertexShader:ee.linedashed_vert,fragmentShader:ee.linedashed_frag},depth:{uniforms:fn([pt.common,pt.displacementmap]),vertexShader:ee.depth_vert,fragmentShader:ee.depth_frag},normal:{uniforms:fn([pt.common,pt.bumpmap,pt.normalmap,pt.displacementmap,{opacity:{value:1}}]),vertexShader:ee.meshnormal_vert,fragmentShader:ee.meshnormal_frag},sprite:{uniforms:fn([pt.sprite,pt.fog]),vertexShader:ee.sprite_vert,fragmentShader:ee.sprite_frag},background:{uniforms:{uvTransform:{value:new ie},t2D:{value:null},backgroundIntensity:{value:1}},vertexShader:ee.background_vert,fragmentShader:ee.background_frag},backgroundCube:{uniforms:{envMap:{value:null},flipEnvMap:{value:-1},backgroundBlurriness:{value:0},backgroundIntensity:{value:1}},vertexShader:ee.backgroundCube_vert,fragmentShader:ee.backgroundCube_frag},cube:{uniforms:{tCube:{value:null},tFlip:{value:-1},opacity:{value:1}},vertexShader:ee.cube_vert,fragmentShader:ee.cube_frag},equirect:{uniforms:{tEquirect:{value:null}},vertexShader:ee.equirect_vert,fragmentShader:ee.equirect_frag},distanceRGBA:{uniforms:fn([pt.common,pt.displacementmap,{referencePosition:{value:new C},nearDistance:{value:1},farDistance:{value:1e3}}]),vertexShader:ee.distanceRGBA_vert,fragmentShader:ee.distanceRGBA_frag},shadow:{uniforms:fn([pt.lights,pt.fog,{color:{value:new Kt(0)},opacity:{value:1}}]),vertexShader:ee.shadow_vert,fragmentShader:ee.shadow_frag}};li.physical={uniforms:fn([li.standard.uniforms,{clearcoat:{value:0},clearcoatMap:{value:null},clearcoatMapTransform:{value:new ie},clearcoatNormalMap:{value:null},clearcoatNormalMapTransform:{value:new ie},clearcoatNormalScale:{value:new yt(1,1)},clearcoatRoughness:{value:0},clearcoatRoughnessMap:{value:null},clearcoatRoughnessMapTransform:{value:new ie},iridescence:{value:0},iridescenceMap:{value:null},iridescenceMapTransform:{value:new ie},iridescenceIOR:{value:1.3},iridescenceThicknessMinimum:{value:100},iridescenceThicknessMaximum:{value:400},iridescenceThicknessMap:{value:null},iridescenceThicknessMapTransform:{value:new ie},sheen:{value:0},sheenColor:{value:new Kt(0)},sheenColorMap:{value:null},sheenColorMapTransform:{value:new ie},sheenRoughness:{value:1},sheenRoughnessMap:{value:null},sheenRoughnessMapTransform:{value:new ie},transmission:{value:0},transmissionMap:{value:null},transmissionMapTransform:{value:new ie},transmissionSamplerSize:{value:new yt},transmissionSamplerMap:{value:null},thickness:{value:0},thicknessMap:{value:null},thicknessMapTransform:{value:new ie},attenuationDistance:{value:0},attenuationColor:{value:new Kt(0)},specularColor:{value:new Kt(1,1,1)},specularColorMap:{value:null},specularColorMapTransform:{value:new ie},specularIntensity:{value:1},specularIntensityMap:{value:null},specularIntensityMapTransform:{value:new ie},anisotropyVector:{value:new yt},anisotropyMap:{value:null},anisotropyMapTransform:{value:new ie}}]),vertexShader:ee.meshphysical_vert,fragmentShader:ee.meshphysical_frag};const Ka={r:0,b:0,g:0};function xx(r,t,e,n,i,s,a){const o=new Kt(0);let c=s===!0?0:1,l,h,u=null,d=0,f=null;function m(g,p){let _=!1,v=p.isScene===!0?p.background:null;v&&v.isTexture&&(v=(p.backgroundBlurriness>0?e:t).get(v)),v===null?x(o,c):v&&v.isColor&&(x(v,1),_=!0);const S=r.xr.getEnvironmentBlendMode();S==="additive"?n.buffers.color.setClear(0,0,0,1,a):S==="alpha-blend"&&n.buffers.color.setClear(0,0,0,0,a),(r.autoClear||_)&&r.clear(r.autoClearColor,r.autoClearDepth,r.autoClearStencil),v&&(v.isCubeTexture||v.mapping===306)?(h===void 0&&(h=new $t(new Ln(1,1,1),new Un({name:"BackgroundCubeMaterial",uniforms:mr(li.backgroundCube.uniforms),vertexShader:li.backgroundCube.vertexShader,fragmentShader:li.backgroundCube.fragmentShader,side:1,depthTest:!1,depthWrite:!1,fog:!1})),h.geometry.deleteAttribute("normal"),h.geometry.deleteAttribute("uv"),h.onBeforeRender=function(M,y,w){this.matrixWorld.copyPosition(w.matrixWorld)},Object.defineProperty(h.material,"envMap",{get:function(){return this.uniforms.envMap.value}}),i.update(h)),h.material.uniforms.envMap.value=v,h.material.uniforms.flipEnvMap.value=v.isCubeTexture&&v.isRenderTargetTexture===!1?-1:1,h.material.uniforms.backgroundBlurriness.value=p.backgroundBlurriness,h.material.uniforms.backgroundIntensity.value=p.backgroundIntensity,h.material.toneMapped=ge.getTransfer(v.colorSpace)!==ye,(u!==v||d!==v.version||f!==r.toneMapping)&&(h.material.needsUpdate=!0,u=v,d=v.version,f=r.toneMapping),h.layers.enableAll(),g.unshift(h,h.geometry,h.material,0,0,null)):v&&v.isTexture&&(l===void 0&&(l=new $t(new lc(2,2),new Un({name:"BackgroundMaterial",uniforms:mr(li.background.uniforms),vertexShader:li.background.vertexShader,fragmentShader:li.background.fragmentShader,side:0,depthTest:!1,depthWrite:!1,fog:!1})),l.geometry.deleteAttribute("normal"),Object.defineProperty(l.material,"map",{get:function(){return this.uniforms.t2D.value}}),i.update(l)),l.material.uniforms.t2D.value=v,l.material.uniforms.backgroundIntensity.value=p.backgroundIntensity,l.material.toneMapped=ge.getTransfer(v.colorSpace)!==ye,v.matrixAutoUpdate===!0&&v.updateMatrix(),l.material.uniforms.uvTransform.value.copy(v.matrix),(u!==v||d!==v.version||f!==r.toneMapping)&&(l.material.needsUpdate=!0,u=v,d=v.version,f=r.toneMapping),l.layers.enableAll(),g.unshift(l,l.geometry,l.material,0,0,null))}function x(g,p){g.getRGB(Ka,Nf(r)),n.buffers.color.setClear(Ka.r,Ka.g,Ka.b,p,a)}return{getClearColor:function(){return o},setClearColor:function(g,p=1){o.set(g),c=p,x(o,c)},getClearAlpha:function(){return c},setClearAlpha:function(g){c=g,x(o,c)},render:m}}function _x(r,t,e,n){const i=r.getParameter(r.MAX_VERTEX_ATTRIBS),s=n.isWebGL2?null:t.get("OES_vertex_array_object"),a=n.isWebGL2||s!==null,o={},c=g(null);let l=c,h=!1;function u(L,N,U,k,O){let H=!1;if(a){const j=x(k,U,N);l!==j&&(l=j,f(l.object)),H=p(L,k,U,O),H&&_(L,k,U,O)}else{const j=N.wireframe===!0;(l.geometry!==k.id||l.program!==U.id||l.wireframe!==j)&&(l.geometry=k.id,l.program=U.id,l.wireframe=j,H=!0)}O!==null&&e.update(O,r.ELEMENT_ARRAY_BUFFER),(H||h)&&(h=!1,T(L,N,U,k),O!==null&&r.bindBuffer(r.ELEMENT_ARRAY_BUFFER,e.get(O).buffer))}function d(){return n.isWebGL2?r.createVertexArray():s.createVertexArrayOES()}function f(L){return n.isWebGL2?r.bindVertexArray(L):s.bindVertexArrayOES(L)}function m(L){return n.isWebGL2?r.deleteVertexArray(L):s.deleteVertexArrayOES(L)}function x(L,N,U){const k=U.wireframe===!0;let O=o[L.id];O===void 0&&(O={},o[L.id]=O);let H=O[N.id];H===void 0&&(H={},O[N.id]=H);let j=H[k];return j===void 0&&(j=g(d()),H[k]=j),j}function g(L){const N=[],U=[],k=[];for(let O=0;O<i;O++)N[O]=0,U[O]=0,k[O]=0;return{geometry:null,program:null,wireframe:!1,newAttributes:N,enabledAttributes:U,attributeDivisors:k,object:L,attributes:{},index:null}}function p(L,N,U,k){const O=l.attributes,H=N.attributes;let j=0;const J=U.getAttributes();for(const nt in J)if(J[nt].location>=0){const $=O[nt];let st=H[nt];if(st===void 0&&(nt==="instanceMatrix"&&L.instanceMatrix&&(st=L.instanceMatrix),nt==="instanceColor"&&L.instanceColor&&(st=L.instanceColor)),$===void 0||$.attribute!==st||st&&$.data!==st.data)return!0;j++}return l.attributesNum!==j||l.index!==k}function _(L,N,U,k){const O={},H=N.attributes;let j=0;const J=U.getAttributes();for(const nt in J)if(J[nt].location>=0){let $=H[nt];$===void 0&&(nt==="instanceMatrix"&&L.instanceMatrix&&($=L.instanceMatrix),nt==="instanceColor"&&L.instanceColor&&($=L.instanceColor));const st={};st.attribute=$,$&&$.data&&(st.data=$.data),O[nt]=st,j++}l.attributes=O,l.attributesNum=j,l.index=k}function v(){const L=l.newAttributes;for(let N=0,U=L.length;N<U;N++)L[N]=0}function S(L){M(L,0)}function M(L,N){const U=l.newAttributes,k=l.enabledAttributes,O=l.attributeDivisors;U[L]=1,k[L]===0&&(r.enableVertexAttribArray(L),k[L]=1),O[L]!==N&&((n.isWebGL2?r:t.get("ANGLE_instanced_arrays"))[n.isWebGL2?"vertexAttribDivisor":"vertexAttribDivisorANGLE"](L,N),O[L]=N)}function y(){const L=l.newAttributes,N=l.enabledAttributes;for(let U=0,k=N.length;U<k;U++)N[U]!==L[U]&&(r.disableVertexAttribArray(U),N[U]=0)}function w(L,N,U,k,O,H,j){j===!0?r.vertexAttribIPointer(L,N,U,O,H):r.vertexAttribPointer(L,N,U,k,O,H)}function T(L,N,U,k){if(n.isWebGL2===!1&&(L.isInstancedMesh||k.isInstancedBufferGeometry)&&t.get("ANGLE_instanced_arrays")===null)return;v();const O=k.attributes,H=U.getAttributes(),j=N.defaultAttributeValues;for(const J in H){const nt=H[J];if(nt.location>=0){let V=O[J];if(V===void 0&&(J==="instanceMatrix"&&L.instanceMatrix&&(V=L.instanceMatrix),J==="instanceColor"&&L.instanceColor&&(V=L.instanceColor)),V!==void 0){const $=V.normalized,st=V.itemSize,at=e.get(V);if(at===void 0)continue;const lt=at.buffer,_t=at.type,ht=at.bytesPerElement,ut=n.isWebGL2===!0&&(_t===r.INT||_t===r.UNSIGNED_INT||V.gpuType===1013);if(V.isInterleavedBufferAttribute){const bt=V.data,Y=bt.stride,Nt=V.offset;if(bt.isInstancedInterleavedBuffer){for(let wt=0;wt<nt.locationSize;wt++)M(nt.location+wt,bt.meshPerAttribute);L.isInstancedMesh!==!0&&k._maxInstanceCount===void 0&&(k._maxInstanceCount=bt.meshPerAttribute*bt.count)}else for(let wt=0;wt<nt.locationSize;wt++)S(nt.location+wt);r.bindBuffer(r.ARRAY_BUFFER,lt);for(let wt=0;wt<nt.locationSize;wt++)w(nt.location+wt,st/nt.locationSize,_t,$,Y*ht,(Nt+st/nt.locationSize*wt)*ht,ut)}else{if(V.isInstancedBufferAttribute){for(let bt=0;bt<nt.locationSize;bt++)M(nt.location+bt,V.meshPerAttribute);L.isInstancedMesh!==!0&&k._maxInstanceCount===void 0&&(k._maxInstanceCount=V.meshPerAttribute*V.count)}else for(let bt=0;bt<nt.locationSize;bt++)S(nt.location+bt);r.bindBuffer(r.ARRAY_BUFFER,lt);for(let bt=0;bt<nt.locationSize;bt++)w(nt.location+bt,st/nt.locationSize,_t,$,st*ht,st/nt.locationSize*bt*ht,ut)}}else if(j!==void 0){const $=j[J];if($!==void 0)switch($.length){case 2:r.vertexAttrib2fv(nt.location,$);break;case 3:r.vertexAttrib3fv(nt.location,$);break;case 4:r.vertexAttrib4fv(nt.location,$);break;default:r.vertexAttrib1fv(nt.location,$)}}}}y()}function E(){b();for(const L in o){const N=o[L];for(const U in N){const k=N[U];for(const O in k)m(k[O].object),delete k[O];delete N[U]}delete o[L]}}function A(L){if(o[L.id]===void 0)return;const N=o[L.id];for(const U in N){const k=N[U];for(const O in k)m(k[O].object),delete k[O];delete N[U]}delete o[L.id]}function D(L){for(const N in o){const U=o[N];if(U[L.id]===void 0)continue;const k=U[L.id];for(const O in k)m(k[O].object),delete k[O];delete U[L.id]}}function b(){F(),h=!0,l!==c&&(l=c,f(l.object))}function F(){c.geometry=null,c.program=null,c.wireframe=!1}return{setup:u,reset:b,resetDefaultState:F,dispose:E,releaseStatesOfGeometry:A,releaseStatesOfProgram:D,initAttributes:v,enableAttribute:S,disableUnusedAttributes:y}}function vx(r,t,e,n){const i=n.isWebGL2;let s;function a(h){s=h}function o(h,u){r.drawArrays(s,h,u),e.update(u,s,1)}function c(h,u,d){if(d===0)return;let f,m;if(i)f=r,m="drawArraysInstanced";else if(f=t.get("ANGLE_instanced_arrays"),m="drawArraysInstancedANGLE",f===null){console.error("THREE.WebGLBufferRenderer: using THREE.InstancedBufferGeometry but hardware does not support extension ANGLE_instanced_arrays.");return}f[m](s,h,u,d),e.update(u,s,d)}function l(h,u,d){if(d===0)return;const f=t.get("WEBGL_multi_draw");if(f===null)for(let m=0;m<d;m++)this.render(h[m],u[m]);else{f.multiDrawArraysWEBGL(s,h,0,u,0,d);let m=0;for(let x=0;x<d;x++)m+=u[x];e.update(m,s,1)}}this.setMode=a,this.render=o,this.renderInstances=c,this.renderMultiDraw=l}function Sx(r,t,e){let n;function i(){if(n!==void 0)return n;if(t.has("EXT_texture_filter_anisotropic")===!0){const w=t.get("EXT_texture_filter_anisotropic");n=r.getParameter(w.MAX_TEXTURE_MAX_ANISOTROPY_EXT)}else n=0;return n}function s(w){if(w==="highp"){if(r.getShaderPrecisionFormat(r.VERTEX_SHADER,r.HIGH_FLOAT).precision>0&&r.getShaderPrecisionFormat(r.FRAGMENT_SHADER,r.HIGH_FLOAT).precision>0)return"highp";w="mediump"}return w==="mediump"&&r.getShaderPrecisionFormat(r.VERTEX_SHADER,r.MEDIUM_FLOAT).precision>0&&r.getShaderPrecisionFormat(r.FRAGMENT_SHADER,r.MEDIUM_FLOAT).precision>0?"mediump":"lowp"}const a=typeof WebGL2RenderingContext<"u"&&r.constructor.name==="WebGL2RenderingContext";let o=e.precision!==void 0?e.precision:"highp";const c=s(o);c!==o&&(console.warn("THREE.WebGLRenderer:",o,"not supported, using",c,"instead."),o=c);const l=a||t.has("WEBGL_draw_buffers"),h=e.logarithmicDepthBuffer===!0,u=r.getParameter(r.MAX_TEXTURE_IMAGE_UNITS),d=r.getParameter(r.MAX_VERTEX_TEXTURE_IMAGE_UNITS),f=r.getParameter(r.MAX_TEXTURE_SIZE),m=r.getParameter(r.MAX_CUBE_MAP_TEXTURE_SIZE),x=r.getParameter(r.MAX_VERTEX_ATTRIBS),g=r.getParameter(r.MAX_VERTEX_UNIFORM_VECTORS),p=r.getParameter(r.MAX_VARYING_VECTORS),_=r.getParameter(r.MAX_FRAGMENT_UNIFORM_VECTORS),v=d>0,S=a||t.has("OES_texture_float"),M=v&&S,y=a?r.getParameter(r.MAX_SAMPLES):0;return{isWebGL2:a,drawBuffers:l,getMaxAnisotropy:i,getMaxPrecision:s,precision:o,logarithmicDepthBuffer:h,maxTextures:u,maxVertexTextures:d,maxTextureSize:f,maxCubemapSize:m,maxAttributes:x,maxVertexUniforms:g,maxVaryings:p,maxFragmentUniforms:_,vertexTextures:v,floatFragmentTextures:S,floatVertexTextures:M,maxSamples:y}}function Mx(r){const t=this;let e=null,n=0,i=!1,s=!1;const a=new Ri,o=new ie,c={value:null,needsUpdate:!1};this.uniform=c,this.numPlanes=0,this.numIntersection=0,this.init=function(u,d){const f=u.length!==0||d||n!==0||i;return i=d,n=u.length,f},this.beginShadows=function(){s=!0,h(null)},this.endShadows=function(){s=!1},this.setGlobalState=function(u,d){e=h(u,d,0)},this.setState=function(u,d,f){const m=u.clippingPlanes,x=u.clipIntersection,g=u.clipShadows,p=r.get(u);if(!i||m===null||m.length===0||s&&!g)s?h(null):l();else{const _=s?0:n,v=_*4;let S=p.clippingState||null;c.value=S,S=h(m,d,v,f);for(let M=0;M!==v;++M)S[M]=e[M];p.clippingState=S,this.numIntersection=x?this.numPlanes:0,this.numPlanes+=_}};function l(){c.value!==e&&(c.value=e,c.needsUpdate=n>0),t.numPlanes=n,t.numIntersection=0}function h(u,d,f,m){const x=u!==null?u.length:0;let g=null;if(x!==0){if(g=c.value,m!==!0||g===null){const p=f+x*4,_=d.matrixWorldInverse;o.getNormalMatrix(_),(g===null||g.length<p)&&(g=new Float32Array(p));for(let v=0,S=f;v!==x;++v,S+=4)a.copy(u[v]).applyMatrix4(_,o),a.normal.toArray(g,S),g[S+3]=a.constant}c.value=g,c.needsUpdate=!0}return t.numPlanes=x,t.numIntersection=0,g}}function yx(r){let t=new WeakMap;function e(a,o){return o===303?a.mapping=301:o===304&&(a.mapping=302),a}function n(a){if(a&&a.isTexture){const o=a.mapping;if(o===303||o===304)if(t.has(a)){const c=t.get(a).texture;return e(c,a.mapping)}else{const c=a.image;if(c&&c.height>0){const l=new Nm(c.height/2);return l.fromEquirectangularTexture(r,a),t.set(a,l),a.addEventListener("dispose",i),e(l.texture,a.mapping)}else return null}}return a}function i(a){const o=a.target;o.removeEventListener("dispose",i);const c=t.get(o);c!==void 0&&(t.delete(o),c.dispose())}function s(){t=new WeakMap}return{get:n,dispose:s}}class Ah extends Ff{constructor(t=-1,e=1,n=1,i=-1,s=.1,a=2e3){super(),this.isOrthographicCamera=!0,this.type="OrthographicCamera",this.zoom=1,this.view=null,this.left=t,this.right=e,this.top=n,this.bottom=i,this.near=s,this.far=a,this.updateProjectionMatrix()}copy(t,e){return super.copy(t,e),this.left=t.left,this.right=t.right,this.top=t.top,this.bottom=t.bottom,this.near=t.near,this.far=t.far,this.zoom=t.zoom,this.view=t.view===null?null:Object.assign({},t.view),this}setViewOffset(t,e,n,i,s,a){this.view===null&&(this.view={enabled:!0,fullWidth:1,fullHeight:1,offsetX:0,offsetY:0,width:1,height:1}),this.view.enabled=!0,this.view.fullWidth=t,this.view.fullHeight=e,this.view.offsetX=n,this.view.offsetY=i,this.view.width=s,this.view.height=a,this.updateProjectionMatrix()}clearViewOffset(){this.view!==null&&(this.view.enabled=!1),this.updateProjectionMatrix()}updateProjectionMatrix(){const t=(this.right-this.left)/(2*this.zoom),e=(this.top-this.bottom)/(2*this.zoom),n=(this.right+this.left)/2,i=(this.top+this.bottom)/2;let s=n-t,a=n+t,o=i+e,c=i-e;if(this.view!==null&&this.view.enabled){const l=(this.right-this.left)/this.view.fullWidth/this.zoom,h=(this.top-this.bottom)/this.view.fullHeight/this.zoom;s+=l*this.view.offsetX,a=s+l*this.view.width,o-=h*this.view.offsetY,c=o-h*this.view.height}this.projectionMatrix.makeOrthographic(s,a,o,c,this.near,this.far,this.coordinateSystem),this.projectionMatrixInverse.copy(this.projectionMatrix).invert()}toJSON(t){const e=super.toJSON(t);return e.object.zoom=this.zoom,e.object.left=this.left,e.object.right=this.right,e.object.top=this.top,e.object.bottom=this.bottom,e.object.near=this.near,e.object.far=this.far,this.view!==null&&(e.object.view=Object.assign({},this.view)),e}}const rr=4,fu=[.125,.215,.35,.446,.526,.582],_s=20,zc=new Ah,pu=new Kt;let Oc=null,Gc=0,Vc=0;const gs=(1+Math.sqrt(5))/2,ks=1/gs,mu=[new C(1,1,1),new C(-1,1,1),new C(1,1,-1),new C(-1,1,-1),new C(0,gs,ks),new C(0,gs,-ks),new C(ks,0,gs),new C(-ks,0,gs),new C(gs,ks,0),new C(-gs,ks,0)];class gu{constructor(t){this._renderer=t,this._pingPongRenderTarget=null,this._lodMax=0,this._cubeSize=0,this._lodPlanes=[],this._sizeLods=[],this._sigmas=[],this._blurMaterial=null,this._cubemapMaterial=null,this._equirectMaterial=null,this._compileMaterial(this._blurMaterial)}fromScene(t,e=0,n=.1,i=100){Oc=this._renderer.getRenderTarget(),Gc=this._renderer.getActiveCubeFace(),Vc=this._renderer.getActiveMipmapLevel(),this._setSize(256);const s=this._allocateTargets();return s.depthBuffer=!0,this._sceneToCubeUV(t,n,i,s),e>0&&this._blur(s,0,0,e),this._applyPMREM(s),this._cleanup(s),s}fromEquirectangular(t,e=null){return this._fromTexture(t,e)}fromCubemap(t,e=null){return this._fromTexture(t,e)}compileCubemapShader(){this._cubemapMaterial===null&&(this._cubemapMaterial=vu(),this._compileMaterial(this._cubemapMaterial))}compileEquirectangularShader(){this._equirectMaterial===null&&(this._equirectMaterial=_u(),this._compileMaterial(this._equirectMaterial))}dispose(){this._dispose(),this._cubemapMaterial!==null&&this._cubemapMaterial.dispose(),this._equirectMaterial!==null&&this._equirectMaterial.dispose()}_setSize(t){this._lodMax=Math.floor(Math.log2(t)),this._cubeSize=Math.pow(2,this._lodMax)}_dispose(){this._blurMaterial!==null&&this._blurMaterial.dispose(),this._pingPongRenderTarget!==null&&this._pingPongRenderTarget.dispose();for(let t=0;t<this._lodPlanes.length;t++)this._lodPlanes[t].dispose()}_cleanup(t){this._renderer.setRenderTarget(Oc,Gc,Vc),t.scissorTest=!1,Ja(t,0,0,t.width,t.height)}_fromTexture(t,e){t.mapping===301||t.mapping===302?this._setSize(t.image.length===0?16:t.image[0].width||t.image[0].image.width):this._setSize(t.image.width/4),Oc=this._renderer.getRenderTarget(),Gc=this._renderer.getActiveCubeFace(),Vc=this._renderer.getActiveMipmapLevel();const n=e||this._allocateTargets();return this._textureToCubeUV(t,n),this._applyPMREM(n),this._cleanup(n),n}_allocateTargets(){const t=3*Math.max(this._cubeSize,112),e=4*this._cubeSize,n={magFilter:1006,minFilter:1006,generateMipmaps:!1,type:1016,format:1023,colorSpace:Ii,depthBuffer:!1},i=xu(t,e,n);if(this._pingPongRenderTarget===null||this._pingPongRenderTarget.width!==t||this._pingPongRenderTarget.height!==e){this._pingPongRenderTarget!==null&&this._dispose(),this._pingPongRenderTarget=xu(t,e,n);const{_lodMax:s}=this;({sizeLods:this._sizeLods,lodPlanes:this._lodPlanes,sigmas:this._sigmas}=Ex(s)),this._blurMaterial=wx(s,t,e)}return i}_compileMaterial(t){const e=new $t(this._lodPlanes[0],t);this._renderer.compile(e,zc)}_sceneToCubeUV(t,e,n,i){const o=new Dn(90,1,e,n),c=[1,-1,1,1,1,1],l=[1,1,1,-1,-1,-1],h=this._renderer,u=h.autoClear,d=h.toneMapping;h.getClearColor(pu),h.toneMapping=0,h.autoClear=!1;const f=new Ue({name:"PMREM.Background",side:1,depthWrite:!1,depthTest:!1}),m=new $t(new Ln,f);let x=!1;const g=t.background;g?g.isColor&&(f.color.copy(g),t.background=null,x=!0):(f.color.copy(pu),x=!0);for(let p=0;p<6;p++){const _=p%3;_===0?(o.up.set(0,c[p],0),o.lookAt(l[p],0,0)):_===1?(o.up.set(0,0,c[p]),o.lookAt(0,l[p],0)):(o.up.set(0,c[p],0),o.lookAt(0,0,l[p]));const v=this._cubeSize;Ja(i,_*v,p>2?v:0,v,v),h.setRenderTarget(i),x&&h.render(m,o),h.render(t,o)}m.geometry.dispose(),m.material.dispose(),h.toneMapping=d,h.autoClear=u,t.background=g}_textureToCubeUV(t,e){const n=this._renderer,i=t.mapping===301||t.mapping===302;i?(this._cubemapMaterial===null&&(this._cubemapMaterial=vu()),this._cubemapMaterial.uniforms.flipEnvMap.value=t.isRenderTargetTexture===!1?-1:1):this._equirectMaterial===null&&(this._equirectMaterial=_u());const s=i?this._cubemapMaterial:this._equirectMaterial,a=new $t(this._lodPlanes[0],s),o=s.uniforms;o.envMap.value=t;const c=this._cubeSize;Ja(e,0,0,3*c,2*c),n.setRenderTarget(e),n.render(a,zc)}_applyPMREM(t){const e=this._renderer,n=e.autoClear;e.autoClear=!1;for(let i=1;i<this._lodPlanes.length;i++){const s=Math.sqrt(this._sigmas[i]*this._sigmas[i]-this._sigmas[i-1]*this._sigmas[i-1]),a=mu[(i-1)%mu.length];this._blur(t,i-1,i,s,a)}e.autoClear=n}_blur(t,e,n,i,s){const a=this._pingPongRenderTarget;this._halfBlur(t,a,e,n,i,"latitudinal",s),this._halfBlur(a,t,n,n,i,"longitudinal",s)}_halfBlur(t,e,n,i,s,a,o){const c=this._renderer,l=this._blurMaterial;a!=="latitudinal"&&a!=="longitudinal"&&console.error("blur direction must be either latitudinal or longitudinal!");const h=3,u=new $t(this._lodPlanes[i],l),d=l.uniforms,f=this._sizeLods[n]-1,m=isFinite(s)?Math.PI/(2*f):2*Math.PI/(2*_s-1),x=s/m,g=isFinite(s)?1+Math.floor(h*x):_s;g>_s&&console.warn(`sigmaRadians, ${s}, is too large and will clip, as it requested ${g} samples when the maximum is set to ${_s}`);const p=[];let _=0;for(let w=0;w<_s;++w){const T=w/x,E=Math.exp(-T*T/2);p.push(E),w===0?_+=E:w<g&&(_+=2*E)}for(let w=0;w<p.length;w++)p[w]=p[w]/_;d.envMap.value=t.texture,d.samples.value=g,d.weights.value=p,d.latitudinal.value=a==="latitudinal",o&&(d.poleAxis.value=o);const{_lodMax:v}=this;d.dTheta.value=m,d.mipInt.value=v-n;const S=this._sizeLods[i],M=3*S*(i>v-rr?i-v+rr:0),y=4*(this._cubeSize-S);Ja(e,M,y,3*S,2*S),c.setRenderTarget(e),c.render(u,zc)}}function Ex(r){const t=[],e=[],n=[];let i=r;const s=r-rr+1+fu.length;for(let a=0;a<s;a++){const o=Math.pow(2,i);e.push(o);let c=1/o;a>r-rr?c=fu[a-r+rr-1]:a===0&&(c=0),n.push(c);const l=1/(o-2),h=-l,u=1+l,d=[h,h,u,h,u,u,h,h,u,u,h,u],f=6,m=6,x=3,g=2,p=1,_=new Float32Array(x*m*f),v=new Float32Array(g*m*f),S=new Float32Array(p*m*f);for(let y=0;y<f;y++){const w=y%3*2/3-1,T=y>2?0:-1,E=[w,T,0,w+2/3,T,0,w+2/3,T+1,0,w,T,0,w+2/3,T+1,0,w,T+1,0];_.set(E,x*m*y),v.set(d,g*m*y);const A=[y,y,y,y,y,y];S.set(A,p*m*y)}const M=new we;M.setAttribute("position",new Fe(_,x)),M.setAttribute("uv",new Fe(v,g)),M.setAttribute("faceIndex",new Fe(S,p)),t.push(M),i>rr&&i--}return{lodPlanes:t,sizeLods:e,sigmas:n}}function xu(r,t,e){const n=new Qe(r,t,e);return n.texture.mapping=306,n.texture.name="PMREM.cubeUv",n.scissorTest=!0,n}function Ja(r,t,e,n,i){r.viewport.set(t,e,n,i),r.scissor.set(t,e,n,i)}function wx(r,t,e){const n=new Float32Array(_s),i=new C(0,1,0);return new Un({name:"SphericalGaussianBlur",defines:{n:_s,CUBEUV_TEXEL_WIDTH:1/t,CUBEUV_TEXEL_HEIGHT:1/e,CUBEUV_MAX_MIP:`${r}.0`},uniforms:{envMap:{value:null},samples:{value:1},weights:{value:n},latitudinal:{value:!1},dTheta:{value:0},mipInt:{value:0},poleAxis:{value:i}},vertexShader:Th(),fragmentShader:`

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
		`,blending:0,depthTest:!1,depthWrite:!1})}function _u(){return new Un({name:"EquirectangularToCubeUV",uniforms:{envMap:{value:null}},vertexShader:Th(),fragmentShader:`

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
		`,blending:0,depthTest:!1,depthWrite:!1})}function vu(){return new Un({name:"CubemapToCubeUV",uniforms:{envMap:{value:null},flipEnvMap:{value:-1}},vertexShader:Th(),fragmentShader:`

			precision mediump float;
			precision mediump int;

			uniform float flipEnvMap;

			varying vec3 vOutputDirection;

			uniform samplerCube envMap;

			void main() {

				gl_FragColor = textureCube( envMap, vec3( flipEnvMap * vOutputDirection.x, vOutputDirection.yz ) );

			}
		`,blending:0,depthTest:!1,depthWrite:!1})}function Th(){return`

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
	`}function Ax(r){let t=new WeakMap,e=null;function n(o){if(o&&o.isTexture){const c=o.mapping,l=c===303||c===304,h=c===301||c===302;if(l||h)if(o.isRenderTargetTexture&&o.needsPMREMUpdate===!0){o.needsPMREMUpdate=!1;let u=t.get(o);return e===null&&(e=new gu(r)),u=l?e.fromEquirectangular(o,u):e.fromCubemap(o,u),t.set(o,u),u.texture}else{if(t.has(o))return t.get(o).texture;{const u=o.image;if(l&&u&&u.height>0||h&&u&&i(u)){e===null&&(e=new gu(r));const d=l?e.fromEquirectangular(o):e.fromCubemap(o);return t.set(o,d),o.addEventListener("dispose",s),d.texture}else return null}}}return o}function i(o){let c=0;const l=6;for(let h=0;h<l;h++)o[h]!==void 0&&c++;return c===l}function s(o){const c=o.target;c.removeEventListener("dispose",s);const l=t.get(c);l!==void 0&&(t.delete(c),l.dispose())}function a(){t=new WeakMap,e!==null&&(e.dispose(),e=null)}return{get:n,dispose:a}}function Tx(r){const t={};function e(n){if(t[n]!==void 0)return t[n];let i;switch(n){case"WEBGL_depth_texture":i=r.getExtension("WEBGL_depth_texture")||r.getExtension("MOZ_WEBGL_depth_texture")||r.getExtension("WEBKIT_WEBGL_depth_texture");break;case"EXT_texture_filter_anisotropic":i=r.getExtension("EXT_texture_filter_anisotropic")||r.getExtension("MOZ_EXT_texture_filter_anisotropic")||r.getExtension("WEBKIT_EXT_texture_filter_anisotropic");break;case"WEBGL_compressed_texture_s3tc":i=r.getExtension("WEBGL_compressed_texture_s3tc")||r.getExtension("MOZ_WEBGL_compressed_texture_s3tc")||r.getExtension("WEBKIT_WEBGL_compressed_texture_s3tc");break;case"WEBGL_compressed_texture_pvrtc":i=r.getExtension("WEBGL_compressed_texture_pvrtc")||r.getExtension("WEBKIT_WEBGL_compressed_texture_pvrtc");break;default:i=r.getExtension(n)}return t[n]=i,i}return{has:function(n){return e(n)!==null},init:function(n){n.isWebGL2?(e("EXT_color_buffer_float"),e("WEBGL_clip_cull_distance")):(e("WEBGL_depth_texture"),e("OES_texture_float"),e("OES_texture_half_float"),e("OES_texture_half_float_linear"),e("OES_standard_derivatives"),e("OES_element_index_uint"),e("OES_vertex_array_object"),e("ANGLE_instanced_arrays")),e("OES_texture_float_linear"),e("EXT_color_buffer_half_float"),e("WEBGL_multisampled_render_to_texture")},get:function(n){const i=e(n);return i===null&&console.warn("THREE.WebGLRenderer: "+n+" extension not supported."),i}}}function Cx(r,t,e,n){const i={},s=new WeakMap;function a(u){const d=u.target;d.index!==null&&t.remove(d.index);for(const m in d.attributes)t.remove(d.attributes[m]);for(const m in d.morphAttributes){const x=d.morphAttributes[m];for(let g=0,p=x.length;g<p;g++)t.remove(x[g])}d.removeEventListener("dispose",a),delete i[d.id];const f=s.get(d);f&&(t.remove(f),s.delete(d)),n.releaseStatesOfGeometry(d),d.isInstancedBufferGeometry===!0&&delete d._maxInstanceCount,e.memory.geometries--}function o(u,d){return i[d.id]===!0||(d.addEventListener("dispose",a),i[d.id]=!0,e.memory.geometries++),d}function c(u){const d=u.attributes;for(const m in d)t.update(d[m],r.ARRAY_BUFFER);const f=u.morphAttributes;for(const m in f){const x=f[m];for(let g=0,p=x.length;g<p;g++)t.update(x[g],r.ARRAY_BUFFER)}}function l(u){const d=[],f=u.index,m=u.attributes.position;let x=0;if(f!==null){const _=f.array;x=f.version;for(let v=0,S=_.length;v<S;v+=3){const M=_[v+0],y=_[v+1],w=_[v+2];d.push(M,y,y,w,w,M)}}else if(m!==void 0){const _=m.array;x=m.version;for(let v=0,S=_.length/3-1;v<S;v+=3){const M=v+0,y=v+1,w=v+2;d.push(M,y,y,w,w,M)}}else return;const g=new(Tf(d)?If:Df)(d,1);g.version=x;const p=s.get(u);p&&t.remove(p),s.set(u,g)}function h(u){const d=s.get(u);if(d){const f=u.index;f!==null&&d.version<f.version&&l(u)}else l(u);return s.get(u)}return{get:o,update:c,getWireframeAttribute:h}}function bx(r,t,e,n){const i=n.isWebGL2;let s;function a(f){s=f}let o,c;function l(f){o=f.type,c=f.bytesPerElement}function h(f,m){r.drawElements(s,m,o,f*c),e.update(m,s,1)}function u(f,m,x){if(x===0)return;let g,p;if(i)g=r,p="drawElementsInstanced";else if(g=t.get("ANGLE_instanced_arrays"),p="drawElementsInstancedANGLE",g===null){console.error("THREE.WebGLIndexedBufferRenderer: using THREE.InstancedBufferGeometry but hardware does not support extension ANGLE_instanced_arrays.");return}g[p](s,m,o,f*c,x),e.update(m,s,x)}function d(f,m,x){if(x===0)return;const g=t.get("WEBGL_multi_draw");if(g===null)for(let p=0;p<x;p++)this.render(f[p]/c,m[p]);else{g.multiDrawElementsWEBGL(s,m,0,o,f,0,x);let p=0;for(let _=0;_<x;_++)p+=m[_];e.update(p,s,1)}}this.setMode=a,this.setIndex=l,this.render=h,this.renderInstances=u,this.renderMultiDraw=d}function Rx(r){const t={geometries:0,textures:0},e={frame:0,calls:0,triangles:0,points:0,lines:0};function n(s,a,o){switch(e.calls++,a){case r.TRIANGLES:e.triangles+=o*(s/3);break;case r.LINES:e.lines+=o*(s/2);break;case r.LINE_STRIP:e.lines+=o*(s-1);break;case r.LINE_LOOP:e.lines+=o*s;break;case r.POINTS:e.points+=o*s;break;default:console.error("THREE.WebGLInfo: Unknown draw mode:",a);break}}function i(){e.calls=0,e.triangles=0,e.points=0,e.lines=0}return{memory:t,render:e,programs:null,autoReset:!0,reset:i,update:n}}function Px(r,t){return r[0]-t[0]}function Lx(r,t){return Math.abs(t[1])-Math.abs(r[1])}function Dx(r,t,e){const n={},i=new Float32Array(8),s=new WeakMap,a=new Ke,o=[];for(let l=0;l<8;l++)o[l]=[l,0];function c(l,h,u){const d=l.morphTargetInfluences;if(t.isWebGL2===!0){const f=h.morphAttributes.position||h.morphAttributes.normal||h.morphAttributes.color,m=f!==void 0?f.length:0;let x=s.get(h);if(x===void 0||x.count!==m){let L=function(){b.dispose(),s.delete(h),h.removeEventListener("dispose",L)};x!==void 0&&x.texture.dispose();const _=h.morphAttributes.position!==void 0,v=h.morphAttributes.normal!==void 0,S=h.morphAttributes.color!==void 0,M=h.morphAttributes.position||[],y=h.morphAttributes.normal||[],w=h.morphAttributes.color||[];let T=0;_===!0&&(T=1),v===!0&&(T=2),S===!0&&(T=3);let E=h.attributes.position.count*T,A=1;E>t.maxTextureSize&&(A=Math.ceil(E/t.maxTextureSize),E=t.maxTextureSize);const D=new Float32Array(E*A*4*m),b=new Rf(D,E,A,m);b.type=1015,b.needsUpdate=!0;const F=T*4;for(let N=0;N<m;N++){const U=M[N],k=y[N],O=w[N],H=E*A*4*N;for(let j=0;j<U.count;j++){const J=j*F;_===!0&&(a.fromBufferAttribute(U,j),D[H+J+0]=a.x,D[H+J+1]=a.y,D[H+J+2]=a.z,D[H+J+3]=0),v===!0&&(a.fromBufferAttribute(k,j),D[H+J+4]=a.x,D[H+J+5]=a.y,D[H+J+6]=a.z,D[H+J+7]=0),S===!0&&(a.fromBufferAttribute(O,j),D[H+J+8]=a.x,D[H+J+9]=a.y,D[H+J+10]=a.z,D[H+J+11]=O.itemSize===4?a.w:1)}}x={count:m,texture:b,size:new yt(E,A)},s.set(h,x),h.addEventListener("dispose",L)}let g=0;for(let _=0;_<d.length;_++)g+=d[_];const p=h.morphTargetsRelative?1:1-g;u.getUniforms().setValue(r,"morphTargetBaseInfluence",p),u.getUniforms().setValue(r,"morphTargetInfluences",d),u.getUniforms().setValue(r,"morphTargetsTexture",x.texture,e),u.getUniforms().setValue(r,"morphTargetsTextureSize",x.size)}else{const f=d===void 0?0:d.length;let m=n[h.id];if(m===void 0||m.length!==f){m=[];for(let v=0;v<f;v++)m[v]=[v,0];n[h.id]=m}for(let v=0;v<f;v++){const S=m[v];S[0]=v,S[1]=d[v]}m.sort(Lx);for(let v=0;v<8;v++)v<f&&m[v][1]?(o[v][0]=m[v][0],o[v][1]=m[v][1]):(o[v][0]=Number.MAX_SAFE_INTEGER,o[v][1]=0);o.sort(Px);const x=h.morphAttributes.position,g=h.morphAttributes.normal;let p=0;for(let v=0;v<8;v++){const S=o[v],M=S[0],y=S[1];M!==Number.MAX_SAFE_INTEGER&&y?(x&&h.getAttribute("morphTarget"+v)!==x[M]&&h.setAttribute("morphTarget"+v,x[M]),g&&h.getAttribute("morphNormal"+v)!==g[M]&&h.setAttribute("morphNormal"+v,g[M]),i[v]=y,p+=y):(x&&h.hasAttribute("morphTarget"+v)===!0&&h.deleteAttribute("morphTarget"+v),g&&h.hasAttribute("morphNormal"+v)===!0&&h.deleteAttribute("morphNormal"+v),i[v]=0)}const _=h.morphTargetsRelative?1:1-p;u.getUniforms().setValue(r,"morphTargetBaseInfluence",_),u.getUniforms().setValue(r,"morphTargetInfluences",i)}}return{update:c}}function Ix(r,t,e,n){let i=new WeakMap;function s(c){const l=n.render.frame,h=c.geometry,u=t.get(c,h);if(i.get(u)!==l&&(t.update(u),i.set(u,l)),c.isInstancedMesh&&(c.hasEventListener("dispose",o)===!1&&c.addEventListener("dispose",o),i.get(c)!==l&&(e.update(c.instanceMatrix,r.ARRAY_BUFFER),c.instanceColor!==null&&e.update(c.instanceColor,r.ARRAY_BUFFER),i.set(c,l))),c.isSkinnedMesh){const d=c.skeleton;i.get(d)!==l&&(d.update(),i.set(d,l))}return u}function a(){i=new WeakMap}function o(c){const l=c.target;l.removeEventListener("dispose",o),e.remove(l.instanceMatrix),l.instanceColor!==null&&e.remove(l.instanceColor)}return{update:s,dispose:a}}class zf extends Nn{constructor(t,e,n,i,s,a,o,c,l,h){if(h=h!==void 0?h:1026,h!==1026&&h!==1027)throw new Error("DepthTexture format must be either THREE.DepthFormat or THREE.DepthStencilFormat");n===void 0&&h===1026&&(n=1014),n===void 0&&h===1027&&(n=1020),super(null,i,s,a,o,c,h,n,l),this.isDepthTexture=!0,this.image={width:t,height:e},this.magFilter=o!==void 0?o:1003,this.minFilter=c!==void 0?c:1003,this.flipY=!1,this.generateMipmaps=!1,this.compareFunction=null}copy(t){return super.copy(t),this.compareFunction=t.compareFunction,this}toJSON(t){const e=super.toJSON(t);return this.compareFunction!==null&&(e.compareFunction=this.compareFunction),e}}const Of=new Nn,Gf=new zf(1,1);Gf.compareFunction=515;const Vf=new Rf,kf=new xm,Hf=new Uf,Su=[],Mu=[],yu=new Float32Array(16),Eu=new Float32Array(9),wu=new Float32Array(4);function yr(r,t,e){const n=r[0];if(n<=0||n>0)return r;const i=t*e;let s=Su[i];if(s===void 0&&(s=new Float32Array(i),Su[i]=s),t!==0){n.toArray(s,0);for(let a=1,o=0;a!==t;++a)o+=e,r[a].toArray(s,o)}return s}function Xe(r,t){if(r.length!==t.length)return!1;for(let e=0,n=r.length;e<n;e++)if(r[e]!==t[e])return!1;return!0}function Ye(r,t){for(let e=0,n=t.length;e<n;e++)r[e]=t[e]}function hc(r,t){let e=Mu[t];e===void 0&&(e=new Int32Array(t),Mu[t]=e);for(let n=0;n!==t;++n)e[n]=r.allocateTextureUnit();return e}function Nx(r,t){const e=this.cache;e[0]!==t&&(r.uniform1f(this.addr,t),e[0]=t)}function Fx(r,t){const e=this.cache;if(t.x!==void 0)(e[0]!==t.x||e[1]!==t.y)&&(r.uniform2f(this.addr,t.x,t.y),e[0]=t.x,e[1]=t.y);else{if(Xe(e,t))return;r.uniform2fv(this.addr,t),Ye(e,t)}}function Ux(r,t){const e=this.cache;if(t.x!==void 0)(e[0]!==t.x||e[1]!==t.y||e[2]!==t.z)&&(r.uniform3f(this.addr,t.x,t.y,t.z),e[0]=t.x,e[1]=t.y,e[2]=t.z);else if(t.r!==void 0)(e[0]!==t.r||e[1]!==t.g||e[2]!==t.b)&&(r.uniform3f(this.addr,t.r,t.g,t.b),e[0]=t.r,e[1]=t.g,e[2]=t.b);else{if(Xe(e,t))return;r.uniform3fv(this.addr,t),Ye(e,t)}}function Bx(r,t){const e=this.cache;if(t.x!==void 0)(e[0]!==t.x||e[1]!==t.y||e[2]!==t.z||e[3]!==t.w)&&(r.uniform4f(this.addr,t.x,t.y,t.z,t.w),e[0]=t.x,e[1]=t.y,e[2]=t.z,e[3]=t.w);else{if(Xe(e,t))return;r.uniform4fv(this.addr,t),Ye(e,t)}}function zx(r,t){const e=this.cache,n=t.elements;if(n===void 0){if(Xe(e,t))return;r.uniformMatrix2fv(this.addr,!1,t),Ye(e,t)}else{if(Xe(e,n))return;wu.set(n),r.uniformMatrix2fv(this.addr,!1,wu),Ye(e,n)}}function Ox(r,t){const e=this.cache,n=t.elements;if(n===void 0){if(Xe(e,t))return;r.uniformMatrix3fv(this.addr,!1,t),Ye(e,t)}else{if(Xe(e,n))return;Eu.set(n),r.uniformMatrix3fv(this.addr,!1,Eu),Ye(e,n)}}function Gx(r,t){const e=this.cache,n=t.elements;if(n===void 0){if(Xe(e,t))return;r.uniformMatrix4fv(this.addr,!1,t),Ye(e,t)}else{if(Xe(e,n))return;yu.set(n),r.uniformMatrix4fv(this.addr,!1,yu),Ye(e,n)}}function Vx(r,t){const e=this.cache;e[0]!==t&&(r.uniform1i(this.addr,t),e[0]=t)}function kx(r,t){const e=this.cache;if(t.x!==void 0)(e[0]!==t.x||e[1]!==t.y)&&(r.uniform2i(this.addr,t.x,t.y),e[0]=t.x,e[1]=t.y);else{if(Xe(e,t))return;r.uniform2iv(this.addr,t),Ye(e,t)}}function Hx(r,t){const e=this.cache;if(t.x!==void 0)(e[0]!==t.x||e[1]!==t.y||e[2]!==t.z)&&(r.uniform3i(this.addr,t.x,t.y,t.z),e[0]=t.x,e[1]=t.y,e[2]=t.z);else{if(Xe(e,t))return;r.uniform3iv(this.addr,t),Ye(e,t)}}function Wx(r,t){const e=this.cache;if(t.x!==void 0)(e[0]!==t.x||e[1]!==t.y||e[2]!==t.z||e[3]!==t.w)&&(r.uniform4i(this.addr,t.x,t.y,t.z,t.w),e[0]=t.x,e[1]=t.y,e[2]=t.z,e[3]=t.w);else{if(Xe(e,t))return;r.uniform4iv(this.addr,t),Ye(e,t)}}function Xx(r,t){const e=this.cache;e[0]!==t&&(r.uniform1ui(this.addr,t),e[0]=t)}function Yx(r,t){const e=this.cache;if(t.x!==void 0)(e[0]!==t.x||e[1]!==t.y)&&(r.uniform2ui(this.addr,t.x,t.y),e[0]=t.x,e[1]=t.y);else{if(Xe(e,t))return;r.uniform2uiv(this.addr,t),Ye(e,t)}}function qx(r,t){const e=this.cache;if(t.x!==void 0)(e[0]!==t.x||e[1]!==t.y||e[2]!==t.z)&&(r.uniform3ui(this.addr,t.x,t.y,t.z),e[0]=t.x,e[1]=t.y,e[2]=t.z);else{if(Xe(e,t))return;r.uniform3uiv(this.addr,t),Ye(e,t)}}function Zx(r,t){const e=this.cache;if(t.x!==void 0)(e[0]!==t.x||e[1]!==t.y||e[2]!==t.z||e[3]!==t.w)&&(r.uniform4ui(this.addr,t.x,t.y,t.z,t.w),e[0]=t.x,e[1]=t.y,e[2]=t.z,e[3]=t.w);else{if(Xe(e,t))return;r.uniform4uiv(this.addr,t),Ye(e,t)}}function jx(r,t,e){const n=this.cache,i=e.allocateTextureUnit();n[0]!==i&&(r.uniform1i(this.addr,i),n[0]=i);const s=this.type===r.SAMPLER_2D_SHADOW?Gf:Of;e.setTexture2D(t||s,i)}function $x(r,t,e){const n=this.cache,i=e.allocateTextureUnit();n[0]!==i&&(r.uniform1i(this.addr,i),n[0]=i),e.setTexture3D(t||kf,i)}function Kx(r,t,e){const n=this.cache,i=e.allocateTextureUnit();n[0]!==i&&(r.uniform1i(this.addr,i),n[0]=i),e.setTextureCube(t||Hf,i)}function Jx(r,t,e){const n=this.cache,i=e.allocateTextureUnit();n[0]!==i&&(r.uniform1i(this.addr,i),n[0]=i),e.setTexture2DArray(t||Vf,i)}function Qx(r){switch(r){case 5126:return Nx;case 35664:return Fx;case 35665:return Ux;case 35666:return Bx;case 35674:return zx;case 35675:return Ox;case 35676:return Gx;case 5124:case 35670:return Vx;case 35667:case 35671:return kx;case 35668:case 35672:return Hx;case 35669:case 35673:return Wx;case 5125:return Xx;case 36294:return Yx;case 36295:return qx;case 36296:return Zx;case 35678:case 36198:case 36298:case 36306:case 35682:return jx;case 35679:case 36299:case 36307:return $x;case 35680:case 36300:case 36308:case 36293:return Kx;case 36289:case 36303:case 36311:case 36292:return Jx}}function t_(r,t){r.uniform1fv(this.addr,t)}function e_(r,t){const e=yr(t,this.size,2);r.uniform2fv(this.addr,e)}function n_(r,t){const e=yr(t,this.size,3);r.uniform3fv(this.addr,e)}function i_(r,t){const e=yr(t,this.size,4);r.uniform4fv(this.addr,e)}function s_(r,t){const e=yr(t,this.size,4);r.uniformMatrix2fv(this.addr,!1,e)}function r_(r,t){const e=yr(t,this.size,9);r.uniformMatrix3fv(this.addr,!1,e)}function a_(r,t){const e=yr(t,this.size,16);r.uniformMatrix4fv(this.addr,!1,e)}function o_(r,t){r.uniform1iv(this.addr,t)}function c_(r,t){r.uniform2iv(this.addr,t)}function l_(r,t){r.uniform3iv(this.addr,t)}function h_(r,t){r.uniform4iv(this.addr,t)}function u_(r,t){r.uniform1uiv(this.addr,t)}function d_(r,t){r.uniform2uiv(this.addr,t)}function f_(r,t){r.uniform3uiv(this.addr,t)}function p_(r,t){r.uniform4uiv(this.addr,t)}function m_(r,t,e){const n=this.cache,i=t.length,s=hc(e,i);Xe(n,s)||(r.uniform1iv(this.addr,s),Ye(n,s));for(let a=0;a!==i;++a)e.setTexture2D(t[a]||Of,s[a])}function g_(r,t,e){const n=this.cache,i=t.length,s=hc(e,i);Xe(n,s)||(r.uniform1iv(this.addr,s),Ye(n,s));for(let a=0;a!==i;++a)e.setTexture3D(t[a]||kf,s[a])}function x_(r,t,e){const n=this.cache,i=t.length,s=hc(e,i);Xe(n,s)||(r.uniform1iv(this.addr,s),Ye(n,s));for(let a=0;a!==i;++a)e.setTextureCube(t[a]||Hf,s[a])}function __(r,t,e){const n=this.cache,i=t.length,s=hc(e,i);Xe(n,s)||(r.uniform1iv(this.addr,s),Ye(n,s));for(let a=0;a!==i;++a)e.setTexture2DArray(t[a]||Vf,s[a])}function v_(r){switch(r){case 5126:return t_;case 35664:return e_;case 35665:return n_;case 35666:return i_;case 35674:return s_;case 35675:return r_;case 35676:return a_;case 5124:case 35670:return o_;case 35667:case 35671:return c_;case 35668:case 35672:return l_;case 35669:case 35673:return h_;case 5125:return u_;case 36294:return d_;case 36295:return f_;case 36296:return p_;case 35678:case 36198:case 36298:case 36306:case 35682:return m_;case 35679:case 36299:case 36307:return g_;case 35680:case 36300:case 36308:case 36293:return x_;case 36289:case 36303:case 36311:case 36292:return __}}class S_{constructor(t,e,n){this.id=t,this.addr=n,this.cache=[],this.type=e.type,this.setValue=Qx(e.type)}}class M_{constructor(t,e,n){this.id=t,this.addr=n,this.cache=[],this.type=e.type,this.size=e.size,this.setValue=v_(e.type)}}class y_{constructor(t){this.id=t,this.seq=[],this.map={}}setValue(t,e,n){const i=this.seq;for(let s=0,a=i.length;s!==a;++s){const o=i[s];o.setValue(t,e[o.id],n)}}}const kc=/(\w+)(\])?(\[|\.)?/g;function Au(r,t){r.seq.push(t),r.map[t.id]=t}function E_(r,t,e){const n=r.name,i=n.length;for(kc.lastIndex=0;;){const s=kc.exec(n),a=kc.lastIndex;let o=s[1];const c=s[2]==="]",l=s[3];if(c&&(o=o|0),l===void 0||l==="["&&a+2===i){Au(e,l===void 0?new S_(o,r,t):new M_(o,r,t));break}else{let u=e.map[o];u===void 0&&(u=new y_(o),Au(e,u)),e=u}}}class No{constructor(t,e){this.seq=[],this.map={};const n=t.getProgramParameter(e,t.ACTIVE_UNIFORMS);for(let i=0;i<n;++i){const s=t.getActiveUniform(e,i),a=t.getUniformLocation(e,s.name);E_(s,a,this)}}setValue(t,e,n,i){const s=this.map[e];s!==void 0&&s.setValue(t,n,i)}setOptional(t,e,n){const i=e[n];i!==void 0&&this.setValue(t,n,i)}static upload(t,e,n,i){for(let s=0,a=e.length;s!==a;++s){const o=e[s],c=n[o.id];c.needsUpdate!==!1&&o.setValue(t,c.value,i)}}static seqWithValue(t,e){const n=[];for(let i=0,s=t.length;i!==s;++i){const a=t[i];a.id in e&&n.push(a)}return n}}function Tu(r,t,e){const n=r.createShader(t);return r.shaderSource(n,e),r.compileShader(n),n}const w_=37297;let A_=0;function T_(r,t){const e=r.split(`
`),n=[],i=Math.max(t-6,0),s=Math.min(t+6,e.length);for(let a=i;a<s;a++){const o=a+1;n.push(`${o===t?">":" "} ${o}: ${e[a]}`)}return n.join(`
`)}function C_(r){const t=ge.getPrimaries(ge.workingColorSpace),e=ge.getPrimaries(r);let n;switch(t===e?n="":t==="p3"&&e===Xo?n="LinearDisplayP3ToLinearSRGB":t===Xo&&e==="p3"&&(n="LinearSRGBToLinearDisplayP3"),r){case Ii:case cc:return[n,"LinearTransferOETF"];case nn:case Mh:return[n,"sRGBTransferOETF"];default:return console.warn("THREE.WebGLProgram: Unsupported color space:",r),[n,"LinearTransferOETF"]}}function Cu(r,t,e){const n=r.getShaderParameter(t,r.COMPILE_STATUS),i=r.getShaderInfoLog(t).trim();if(n&&i==="")return"";const s=/ERROR: 0:(\d+)/.exec(i);if(s){const a=parseInt(s[1]);return e.toUpperCase()+`

`+i+`

`+T_(r.getShaderSource(t),a)}else return i}function b_(r,t){const e=C_(t);return`vec4 ${r}( vec4 value ) { return ${e[0]}( ${e[1]}( value ) ); }`}function R_(r,t){let e;switch(t){case 1:e="Linear";break;case 2:e="Reinhard";break;case 3:e="OptimizedCineon";break;case 4:e="ACESFilmic";break;case 6:e="AgX";break;case 5:e="Custom";break;default:console.warn("THREE.WebGLProgram: Unsupported toneMapping:",t),e="Linear"}return"vec3 "+r+"( vec3 color ) { return "+e+"ToneMapping( color ); }"}function P_(r){return[r.extensionDerivatives||r.envMapCubeUVHeight||r.bumpMap||r.normalMapTangentSpace||r.clearcoatNormalMap||r.flatShading||r.shaderID==="physical"?"#extension GL_OES_standard_derivatives : enable":"",(r.extensionFragDepth||r.logarithmicDepthBuffer)&&r.rendererExtensionFragDepth?"#extension GL_EXT_frag_depth : enable":"",r.extensionDrawBuffers&&r.rendererExtensionDrawBuffers?"#extension GL_EXT_draw_buffers : require":"",(r.extensionShaderTextureLOD||r.envMap||r.transmission)&&r.rendererExtensionShaderTextureLod?"#extension GL_EXT_shader_texture_lod : enable":""].filter(ar).join(`
`)}function L_(r){return[r.extensionClipCullDistance?"#extension GL_ANGLE_clip_cull_distance : require":""].filter(ar).join(`
`)}function D_(r){const t=[];for(const e in r){const n=r[e];n!==!1&&t.push("#define "+e+" "+n)}return t.join(`
`)}function I_(r,t){const e={},n=r.getProgramParameter(t,r.ACTIVE_ATTRIBUTES);for(let i=0;i<n;i++){const s=r.getActiveAttrib(t,i),a=s.name;let o=1;s.type===r.FLOAT_MAT2&&(o=2),s.type===r.FLOAT_MAT3&&(o=3),s.type===r.FLOAT_MAT4&&(o=4),e[a]={type:s.type,location:r.getAttribLocation(t,a),locationSize:o}}return e}function ar(r){return r!==""}function bu(r,t){const e=t.numSpotLightShadows+t.numSpotLightMaps-t.numSpotLightShadowsWithMaps;return r.replace(/NUM_DIR_LIGHTS/g,t.numDirLights).replace(/NUM_SPOT_LIGHTS/g,t.numSpotLights).replace(/NUM_SPOT_LIGHT_MAPS/g,t.numSpotLightMaps).replace(/NUM_SPOT_LIGHT_COORDS/g,e).replace(/NUM_RECT_AREA_LIGHTS/g,t.numRectAreaLights).replace(/NUM_POINT_LIGHTS/g,t.numPointLights).replace(/NUM_HEMI_LIGHTS/g,t.numHemiLights).replace(/NUM_DIR_LIGHT_SHADOWS/g,t.numDirLightShadows).replace(/NUM_SPOT_LIGHT_SHADOWS_WITH_MAPS/g,t.numSpotLightShadowsWithMaps).replace(/NUM_SPOT_LIGHT_SHADOWS/g,t.numSpotLightShadows).replace(/NUM_POINT_LIGHT_SHADOWS/g,t.numPointLightShadows)}function Ru(r,t){return r.replace(/NUM_CLIPPING_PLANES/g,t.numClippingPlanes).replace(/UNION_CLIPPING_PLANES/g,t.numClippingPlanes-t.numClipIntersection)}const N_=/^[ \t]*#include +<([\w\d./]+)>/gm;function kl(r){return r.replace(N_,U_)}const F_=new Map([["encodings_fragment","colorspace_fragment"],["encodings_pars_fragment","colorspace_pars_fragment"],["output_fragment","opaque_fragment"]]);function U_(r,t){let e=ee[t];if(e===void 0){const n=F_.get(t);if(n!==void 0)e=ee[n],console.warn('THREE.WebGLRenderer: Shader chunk "%s" has been deprecated. Use "%s" instead.',t,n);else throw new Error("Can not resolve #include <"+t+">")}return kl(e)}const B_=/#pragma unroll_loop_start\s+for\s*\(\s*int\s+i\s*=\s*(\d+)\s*;\s*i\s*<\s*(\d+)\s*;\s*i\s*\+\+\s*\)\s*{([\s\S]+?)}\s+#pragma unroll_loop_end/g;function Pu(r){return r.replace(B_,z_)}function z_(r,t,e,n){let i="";for(let s=parseInt(t);s<parseInt(e);s++)i+=n.replace(/\[\s*i\s*\]/g,"[ "+s+" ]").replace(/UNROLLED_LOOP_INDEX/g,s);return i}function Lu(r){let t="precision "+r.precision+` float;
precision `+r.precision+" int;";return r.precision==="highp"?t+=`
#define HIGH_PRECISION`:r.precision==="mediump"?t+=`
#define MEDIUM_PRECISION`:r.precision==="lowp"&&(t+=`
#define LOW_PRECISION`),t}function O_(r){let t="SHADOWMAP_TYPE_BASIC";return r.shadowMapType===1?t="SHADOWMAP_TYPE_PCF":r.shadowMapType===2?t="SHADOWMAP_TYPE_PCF_SOFT":r.shadowMapType===3&&(t="SHADOWMAP_TYPE_VSM"),t}function G_(r){let t="ENVMAP_TYPE_CUBE";if(r.envMap)switch(r.envMapMode){case 301:case 302:t="ENVMAP_TYPE_CUBE";break;case 306:t="ENVMAP_TYPE_CUBE_UV";break}return t}function V_(r){let t="ENVMAP_MODE_REFLECTION";return r.envMap&&r.envMapMode===302&&(t="ENVMAP_MODE_REFRACTION"),t}function k_(r){let t="ENVMAP_BLENDING_NONE";if(r.envMap)switch(r.combine){case 0:t="ENVMAP_BLENDING_MULTIPLY";break;case 1:t="ENVMAP_BLENDING_MIX";break;case 2:t="ENVMAP_BLENDING_ADD";break}return t}function H_(r){const t=r.envMapCubeUVHeight;if(t===null)return null;const e=Math.log2(t)-2,n=1/t;return{texelWidth:1/(3*Math.max(Math.pow(2,e),112)),texelHeight:n,maxMip:e}}function W_(r,t,e,n){const i=r.getContext(),s=e.defines;let a=e.vertexShader,o=e.fragmentShader;const c=O_(e),l=G_(e),h=V_(e),u=k_(e),d=H_(e),f=e.isWebGL2?"":P_(e),m=L_(e),x=D_(s),g=i.createProgram();let p,_,v=e.glslVersion?"#version "+e.glslVersion+`
`:"";e.isRawShaderMaterial?(p=["#define SHADER_TYPE "+e.shaderType,"#define SHADER_NAME "+e.shaderName,x].filter(ar).join(`
`),p.length>0&&(p+=`
`),_=[f,"#define SHADER_TYPE "+e.shaderType,"#define SHADER_NAME "+e.shaderName,x].filter(ar).join(`
`),_.length>0&&(_+=`
`)):(p=[Lu(e),"#define SHADER_TYPE "+e.shaderType,"#define SHADER_NAME "+e.shaderName,x,e.extensionClipCullDistance?"#define USE_CLIP_DISTANCE":"",e.batching?"#define USE_BATCHING":"",e.instancing?"#define USE_INSTANCING":"",e.instancingColor?"#define USE_INSTANCING_COLOR":"",e.useFog&&e.fog?"#define USE_FOG":"",e.useFog&&e.fogExp2?"#define FOG_EXP2":"",e.map?"#define USE_MAP":"",e.envMap?"#define USE_ENVMAP":"",e.envMap?"#define "+h:"",e.lightMap?"#define USE_LIGHTMAP":"",e.aoMap?"#define USE_AOMAP":"",e.bumpMap?"#define USE_BUMPMAP":"",e.normalMap?"#define USE_NORMALMAP":"",e.normalMapObjectSpace?"#define USE_NORMALMAP_OBJECTSPACE":"",e.normalMapTangentSpace?"#define USE_NORMALMAP_TANGENTSPACE":"",e.displacementMap?"#define USE_DISPLACEMENTMAP":"",e.emissiveMap?"#define USE_EMISSIVEMAP":"",e.anisotropy?"#define USE_ANISOTROPY":"",e.anisotropyMap?"#define USE_ANISOTROPYMAP":"",e.clearcoatMap?"#define USE_CLEARCOATMAP":"",e.clearcoatRoughnessMap?"#define USE_CLEARCOAT_ROUGHNESSMAP":"",e.clearcoatNormalMap?"#define USE_CLEARCOAT_NORMALMAP":"",e.iridescenceMap?"#define USE_IRIDESCENCEMAP":"",e.iridescenceThicknessMap?"#define USE_IRIDESCENCE_THICKNESSMAP":"",e.specularMap?"#define USE_SPECULARMAP":"",e.specularColorMap?"#define USE_SPECULAR_COLORMAP":"",e.specularIntensityMap?"#define USE_SPECULAR_INTENSITYMAP":"",e.roughnessMap?"#define USE_ROUGHNESSMAP":"",e.metalnessMap?"#define USE_METALNESSMAP":"",e.alphaMap?"#define USE_ALPHAMAP":"",e.alphaHash?"#define USE_ALPHAHASH":"",e.transmission?"#define USE_TRANSMISSION":"",e.transmissionMap?"#define USE_TRANSMISSIONMAP":"",e.thicknessMap?"#define USE_THICKNESSMAP":"",e.sheenColorMap?"#define USE_SHEEN_COLORMAP":"",e.sheenRoughnessMap?"#define USE_SHEEN_ROUGHNESSMAP":"",e.mapUv?"#define MAP_UV "+e.mapUv:"",e.alphaMapUv?"#define ALPHAMAP_UV "+e.alphaMapUv:"",e.lightMapUv?"#define LIGHTMAP_UV "+e.lightMapUv:"",e.aoMapUv?"#define AOMAP_UV "+e.aoMapUv:"",e.emissiveMapUv?"#define EMISSIVEMAP_UV "+e.emissiveMapUv:"",e.bumpMapUv?"#define BUMPMAP_UV "+e.bumpMapUv:"",e.normalMapUv?"#define NORMALMAP_UV "+e.normalMapUv:"",e.displacementMapUv?"#define DISPLACEMENTMAP_UV "+e.displacementMapUv:"",e.metalnessMapUv?"#define METALNESSMAP_UV "+e.metalnessMapUv:"",e.roughnessMapUv?"#define ROUGHNESSMAP_UV "+e.roughnessMapUv:"",e.anisotropyMapUv?"#define ANISOTROPYMAP_UV "+e.anisotropyMapUv:"",e.clearcoatMapUv?"#define CLEARCOATMAP_UV "+e.clearcoatMapUv:"",e.clearcoatNormalMapUv?"#define CLEARCOAT_NORMALMAP_UV "+e.clearcoatNormalMapUv:"",e.clearcoatRoughnessMapUv?"#define CLEARCOAT_ROUGHNESSMAP_UV "+e.clearcoatRoughnessMapUv:"",e.iridescenceMapUv?"#define IRIDESCENCEMAP_UV "+e.iridescenceMapUv:"",e.iridescenceThicknessMapUv?"#define IRIDESCENCE_THICKNESSMAP_UV "+e.iridescenceThicknessMapUv:"",e.sheenColorMapUv?"#define SHEEN_COLORMAP_UV "+e.sheenColorMapUv:"",e.sheenRoughnessMapUv?"#define SHEEN_ROUGHNESSMAP_UV "+e.sheenRoughnessMapUv:"",e.specularMapUv?"#define SPECULARMAP_UV "+e.specularMapUv:"",e.specularColorMapUv?"#define SPECULAR_COLORMAP_UV "+e.specularColorMapUv:"",e.specularIntensityMapUv?"#define SPECULAR_INTENSITYMAP_UV "+e.specularIntensityMapUv:"",e.transmissionMapUv?"#define TRANSMISSIONMAP_UV "+e.transmissionMapUv:"",e.thicknessMapUv?"#define THICKNESSMAP_UV "+e.thicknessMapUv:"",e.vertexTangents&&e.flatShading===!1?"#define USE_TANGENT":"",e.vertexColors?"#define USE_COLOR":"",e.vertexAlphas?"#define USE_COLOR_ALPHA":"",e.vertexUv1s?"#define USE_UV1":"",e.vertexUv2s?"#define USE_UV2":"",e.vertexUv3s?"#define USE_UV3":"",e.pointsUvs?"#define USE_POINTS_UV":"",e.flatShading?"#define FLAT_SHADED":"",e.skinning?"#define USE_SKINNING":"",e.morphTargets?"#define USE_MORPHTARGETS":"",e.morphNormals&&e.flatShading===!1?"#define USE_MORPHNORMALS":"",e.morphColors&&e.isWebGL2?"#define USE_MORPHCOLORS":"",e.morphTargetsCount>0&&e.isWebGL2?"#define MORPHTARGETS_TEXTURE":"",e.morphTargetsCount>0&&e.isWebGL2?"#define MORPHTARGETS_TEXTURE_STRIDE "+e.morphTextureStride:"",e.morphTargetsCount>0&&e.isWebGL2?"#define MORPHTARGETS_COUNT "+e.morphTargetsCount:"",e.doubleSided?"#define DOUBLE_SIDED":"",e.flipSided?"#define FLIP_SIDED":"",e.shadowMapEnabled?"#define USE_SHADOWMAP":"",e.shadowMapEnabled?"#define "+c:"",e.sizeAttenuation?"#define USE_SIZEATTENUATION":"",e.numLightProbes>0?"#define USE_LIGHT_PROBES":"",e.useLegacyLights?"#define LEGACY_LIGHTS":"",e.logarithmicDepthBuffer?"#define USE_LOGDEPTHBUF":"",e.logarithmicDepthBuffer&&e.rendererExtensionFragDepth?"#define USE_LOGDEPTHBUF_EXT":"","uniform mat4 modelMatrix;","uniform mat4 modelViewMatrix;","uniform mat4 projectionMatrix;","uniform mat4 viewMatrix;","uniform mat3 normalMatrix;","uniform vec3 cameraPosition;","uniform bool isOrthographic;","#ifdef USE_INSTANCING","	attribute mat4 instanceMatrix;","#endif","#ifdef USE_INSTANCING_COLOR","	attribute vec3 instanceColor;","#endif","attribute vec3 position;","attribute vec3 normal;","attribute vec2 uv;","#ifdef USE_UV1","	attribute vec2 uv1;","#endif","#ifdef USE_UV2","	attribute vec2 uv2;","#endif","#ifdef USE_UV3","	attribute vec2 uv3;","#endif","#ifdef USE_TANGENT","	attribute vec4 tangent;","#endif","#if defined( USE_COLOR_ALPHA )","	attribute vec4 color;","#elif defined( USE_COLOR )","	attribute vec3 color;","#endif","#if ( defined( USE_MORPHTARGETS ) && ! defined( MORPHTARGETS_TEXTURE ) )","	attribute vec3 morphTarget0;","	attribute vec3 morphTarget1;","	attribute vec3 morphTarget2;","	attribute vec3 morphTarget3;","	#ifdef USE_MORPHNORMALS","		attribute vec3 morphNormal0;","		attribute vec3 morphNormal1;","		attribute vec3 morphNormal2;","		attribute vec3 morphNormal3;","	#else","		attribute vec3 morphTarget4;","		attribute vec3 morphTarget5;","		attribute vec3 morphTarget6;","		attribute vec3 morphTarget7;","	#endif","#endif","#ifdef USE_SKINNING","	attribute vec4 skinIndex;","	attribute vec4 skinWeight;","#endif",`
`].filter(ar).join(`
`),_=[f,Lu(e),"#define SHADER_TYPE "+e.shaderType,"#define SHADER_NAME "+e.shaderName,x,e.useFog&&e.fog?"#define USE_FOG":"",e.useFog&&e.fogExp2?"#define FOG_EXP2":"",e.map?"#define USE_MAP":"",e.matcap?"#define USE_MATCAP":"",e.envMap?"#define USE_ENVMAP":"",e.envMap?"#define "+l:"",e.envMap?"#define "+h:"",e.envMap?"#define "+u:"",d?"#define CUBEUV_TEXEL_WIDTH "+d.texelWidth:"",d?"#define CUBEUV_TEXEL_HEIGHT "+d.texelHeight:"",d?"#define CUBEUV_MAX_MIP "+d.maxMip+".0":"",e.lightMap?"#define USE_LIGHTMAP":"",e.aoMap?"#define USE_AOMAP":"",e.bumpMap?"#define USE_BUMPMAP":"",e.normalMap?"#define USE_NORMALMAP":"",e.normalMapObjectSpace?"#define USE_NORMALMAP_OBJECTSPACE":"",e.normalMapTangentSpace?"#define USE_NORMALMAP_TANGENTSPACE":"",e.emissiveMap?"#define USE_EMISSIVEMAP":"",e.anisotropy?"#define USE_ANISOTROPY":"",e.anisotropyMap?"#define USE_ANISOTROPYMAP":"",e.clearcoat?"#define USE_CLEARCOAT":"",e.clearcoatMap?"#define USE_CLEARCOATMAP":"",e.clearcoatRoughnessMap?"#define USE_CLEARCOAT_ROUGHNESSMAP":"",e.clearcoatNormalMap?"#define USE_CLEARCOAT_NORMALMAP":"",e.iridescence?"#define USE_IRIDESCENCE":"",e.iridescenceMap?"#define USE_IRIDESCENCEMAP":"",e.iridescenceThicknessMap?"#define USE_IRIDESCENCE_THICKNESSMAP":"",e.specularMap?"#define USE_SPECULARMAP":"",e.specularColorMap?"#define USE_SPECULAR_COLORMAP":"",e.specularIntensityMap?"#define USE_SPECULAR_INTENSITYMAP":"",e.roughnessMap?"#define USE_ROUGHNESSMAP":"",e.metalnessMap?"#define USE_METALNESSMAP":"",e.alphaMap?"#define USE_ALPHAMAP":"",e.alphaTest?"#define USE_ALPHATEST":"",e.alphaHash?"#define USE_ALPHAHASH":"",e.sheen?"#define USE_SHEEN":"",e.sheenColorMap?"#define USE_SHEEN_COLORMAP":"",e.sheenRoughnessMap?"#define USE_SHEEN_ROUGHNESSMAP":"",e.transmission?"#define USE_TRANSMISSION":"",e.transmissionMap?"#define USE_TRANSMISSIONMAP":"",e.thicknessMap?"#define USE_THICKNESSMAP":"",e.vertexTangents&&e.flatShading===!1?"#define USE_TANGENT":"",e.vertexColors||e.instancingColor?"#define USE_COLOR":"",e.vertexAlphas?"#define USE_COLOR_ALPHA":"",e.vertexUv1s?"#define USE_UV1":"",e.vertexUv2s?"#define USE_UV2":"",e.vertexUv3s?"#define USE_UV3":"",e.pointsUvs?"#define USE_POINTS_UV":"",e.gradientMap?"#define USE_GRADIENTMAP":"",e.flatShading?"#define FLAT_SHADED":"",e.doubleSided?"#define DOUBLE_SIDED":"",e.flipSided?"#define FLIP_SIDED":"",e.shadowMapEnabled?"#define USE_SHADOWMAP":"",e.shadowMapEnabled?"#define "+c:"",e.premultipliedAlpha?"#define PREMULTIPLIED_ALPHA":"",e.numLightProbes>0?"#define USE_LIGHT_PROBES":"",e.useLegacyLights?"#define LEGACY_LIGHTS":"",e.decodeVideoTexture?"#define DECODE_VIDEO_TEXTURE":"",e.logarithmicDepthBuffer?"#define USE_LOGDEPTHBUF":"",e.logarithmicDepthBuffer&&e.rendererExtensionFragDepth?"#define USE_LOGDEPTHBUF_EXT":"","uniform mat4 viewMatrix;","uniform vec3 cameraPosition;","uniform bool isOrthographic;",e.toneMapping!==0?"#define TONE_MAPPING":"",e.toneMapping!==0?ee.tonemapping_pars_fragment:"",e.toneMapping!==0?R_("toneMapping",e.toneMapping):"",e.dithering?"#define DITHERING":"",e.opaque?"#define OPAQUE":"",ee.colorspace_pars_fragment,b_("linearToOutputTexel",e.outputColorSpace),e.useDepthPacking?"#define DEPTH_PACKING "+e.depthPacking:"",`
`].filter(ar).join(`
`)),a=kl(a),a=bu(a,e),a=Ru(a,e),o=kl(o),o=bu(o,e),o=Ru(o,e),a=Pu(a),o=Pu(o),e.isWebGL2&&e.isRawShaderMaterial!==!0&&(v=`#version 300 es
`,p=[m,"precision mediump sampler2DArray;","#define attribute in","#define varying out","#define texture2D texture"].join(`
`)+`
`+p,_=["precision mediump sampler2DArray;","#define varying in",e.glslVersion===jh?"":"layout(location = 0) out highp vec4 pc_fragColor;",e.glslVersion===jh?"":"#define gl_FragColor pc_fragColor","#define gl_FragDepthEXT gl_FragDepth","#define texture2D texture","#define textureCube texture","#define texture2DProj textureProj","#define texture2DLodEXT textureLod","#define texture2DProjLodEXT textureProjLod","#define textureCubeLodEXT textureLod","#define texture2DGradEXT textureGrad","#define texture2DProjGradEXT textureProjGrad","#define textureCubeGradEXT textureGrad"].join(`
`)+`
`+_);const S=v+p+a,M=v+_+o,y=Tu(i,i.VERTEX_SHADER,S),w=Tu(i,i.FRAGMENT_SHADER,M);i.attachShader(g,y),i.attachShader(g,w),e.index0AttributeName!==void 0?i.bindAttribLocation(g,0,e.index0AttributeName):e.morphTargets===!0&&i.bindAttribLocation(g,0,"position"),i.linkProgram(g);function T(b){if(r.debug.checkShaderErrors){const F=i.getProgramInfoLog(g).trim(),L=i.getShaderInfoLog(y).trim(),N=i.getShaderInfoLog(w).trim();let U=!0,k=!0;if(i.getProgramParameter(g,i.LINK_STATUS)===!1)if(U=!1,typeof r.debug.onShaderError=="function")r.debug.onShaderError(i,g,y,w);else{const O=Cu(i,y,"vertex"),H=Cu(i,w,"fragment");console.error("THREE.WebGLProgram: Shader Error "+i.getError()+" - VALIDATE_STATUS "+i.getProgramParameter(g,i.VALIDATE_STATUS)+`

Program Info Log: `+F+`
`+O+`
`+H)}else F!==""?console.warn("THREE.WebGLProgram: Program Info Log:",F):(L===""||N==="")&&(k=!1);k&&(b.diagnostics={runnable:U,programLog:F,vertexShader:{log:L,prefix:p},fragmentShader:{log:N,prefix:_}})}i.deleteShader(y),i.deleteShader(w),E=new No(i,g),A=I_(i,g)}let E;this.getUniforms=function(){return E===void 0&&T(this),E};let A;this.getAttributes=function(){return A===void 0&&T(this),A};let D=e.rendererExtensionParallelShaderCompile===!1;return this.isReady=function(){return D===!1&&(D=i.getProgramParameter(g,w_)),D},this.destroy=function(){n.releaseStatesOfProgram(this),i.deleteProgram(g),this.program=void 0},this.type=e.shaderType,this.name=e.shaderName,this.id=A_++,this.cacheKey=t,this.usedTimes=1,this.program=g,this.vertexShader=y,this.fragmentShader=w,this}let X_=0;class Y_{constructor(){this.shaderCache=new Map,this.materialCache=new Map}update(t){const e=t.vertexShader,n=t.fragmentShader,i=this._getShaderStage(e),s=this._getShaderStage(n),a=this._getShaderCacheForMaterial(t);return a.has(i)===!1&&(a.add(i),i.usedTimes++),a.has(s)===!1&&(a.add(s),s.usedTimes++),this}remove(t){const e=this.materialCache.get(t);for(const n of e)n.usedTimes--,n.usedTimes===0&&this.shaderCache.delete(n.code);return this.materialCache.delete(t),this}getVertexShaderID(t){return this._getShaderStage(t.vertexShader).id}getFragmentShaderID(t){return this._getShaderStage(t.fragmentShader).id}dispose(){this.shaderCache.clear(),this.materialCache.clear()}_getShaderCacheForMaterial(t){const e=this.materialCache;let n=e.get(t);return n===void 0&&(n=new Set,e.set(t,n)),n}_getShaderStage(t){const e=this.shaderCache;let n=e.get(t);return n===void 0&&(n=new q_(t),e.set(t,n)),n}}class q_{constructor(t){this.id=X_++,this.code=t,this.usedTimes=0}}function Z_(r,t,e,n,i,s,a){const o=new Pf,c=new Y_,l=[],h=i.isWebGL2,u=i.logarithmicDepthBuffer,d=i.vertexTextures;let f=i.precision;const m={MeshDepthMaterial:"depth",MeshDistanceMaterial:"distanceRGBA",MeshNormalMaterial:"normal",MeshBasicMaterial:"basic",MeshLambertMaterial:"lambert",MeshPhongMaterial:"phong",MeshToonMaterial:"toon",MeshStandardMaterial:"physical",MeshPhysicalMaterial:"physical",MeshMatcapMaterial:"matcap",LineBasicMaterial:"basic",LineDashedMaterial:"dashed",PointsMaterial:"points",ShadowMaterial:"shadow",SpriteMaterial:"sprite"};function x(E){return E===0?"uv":`uv${E}`}function g(E,A,D,b,F){const L=b.fog,N=F.geometry,U=E.isMeshStandardMaterial?b.environment:null,k=(E.isMeshStandardMaterial?e:t).get(E.envMap||U),O=k&&k.mapping===306?k.image.height:null,H=m[E.type];E.precision!==null&&(f=i.getMaxPrecision(E.precision),f!==E.precision&&console.warn("THREE.WebGLProgram.getParameters:",E.precision,"not supported, using",f,"instead."));const j=N.morphAttributes.position||N.morphAttributes.normal||N.morphAttributes.color,J=j!==void 0?j.length:0;let nt=0;N.morphAttributes.position!==void 0&&(nt=1),N.morphAttributes.normal!==void 0&&(nt=2),N.morphAttributes.color!==void 0&&(nt=3);let V,$,st,at;if(H){const Ce=li[H];V=Ce.vertexShader,$=Ce.fragmentShader}else V=E.vertexShader,$=E.fragmentShader,c.update(E),st=c.getVertexShaderID(E),at=c.getFragmentShaderID(E);const lt=r.getRenderTarget(),_t=F.isInstancedMesh===!0,ht=F.isBatchedMesh===!0,ut=!!E.map,bt=!!E.matcap,Y=!!k,Nt=!!E.aoMap,wt=!!E.lightMap,Ct=!!E.bumpMap,vt=!!E.normalMap,Jt=!!E.displacementMap,Lt=!!E.emissiveMap,I=!!E.metalnessMap,R=!!E.roughnessMap,W=E.anisotropy>0,Q=E.clearcoat>0,K=E.iridescence>0,tt=E.sheen>0,Mt=E.transmission>0,rt=W&&!!E.anisotropyMap,gt=Q&&!!E.clearcoatMap,Pt=Q&&!!E.clearcoatNormalMap,Wt=Q&&!!E.clearcoatRoughnessMap,et=K&&!!E.iridescenceMap,oe=K&&!!E.iridescenceThicknessMap,jt=tt&&!!E.sheenColorMap,kt=tt&&!!E.sheenRoughnessMap,At=!!E.specularMap,xt=!!E.specularColorMap,Ft=!!E.specularIntensityMap,Qt=Mt&&!!E.transmissionMap,fe=Mt&&!!E.thicknessMap,qt=!!E.gradientMap,ot=!!E.alphaMap,B=E.alphaTest>0,dt=!!E.alphaHash,ft=!!E.extensions,Gt=!!N.attributes.uv1,Ut=!!N.attributes.uv2,de=!!N.attributes.uv3;let ce=0;return E.toneMapped&&(lt===null||lt.isXRRenderTarget===!0)&&(ce=r.toneMapping),{isWebGL2:h,shaderID:H,shaderType:E.type,shaderName:E.name,vertexShader:V,fragmentShader:$,defines:E.defines,customVertexShaderID:st,customFragmentShaderID:at,isRawShaderMaterial:E.isRawShaderMaterial===!0,glslVersion:E.glslVersion,precision:f,batching:ht,instancing:_t,instancingColor:_t&&F.instanceColor!==null,supportsVertexTextures:d,outputColorSpace:lt===null?r.outputColorSpace:lt.isXRRenderTarget===!0?lt.texture.colorSpace:Ii,map:ut,matcap:bt,envMap:Y,envMapMode:Y&&k.mapping,envMapCubeUVHeight:O,aoMap:Nt,lightMap:wt,bumpMap:Ct,normalMap:vt,displacementMap:d&&Jt,emissiveMap:Lt,normalMapObjectSpace:vt&&E.normalMapType===1,normalMapTangentSpace:vt&&E.normalMapType===0,metalnessMap:I,roughnessMap:R,anisotropy:W,anisotropyMap:rt,clearcoat:Q,clearcoatMap:gt,clearcoatNormalMap:Pt,clearcoatRoughnessMap:Wt,iridescence:K,iridescenceMap:et,iridescenceThicknessMap:oe,sheen:tt,sheenColorMap:jt,sheenRoughnessMap:kt,specularMap:At,specularColorMap:xt,specularIntensityMap:Ft,transmission:Mt,transmissionMap:Qt,thicknessMap:fe,gradientMap:qt,opaque:E.transparent===!1&&E.blending===1,alphaMap:ot,alphaTest:B,alphaHash:dt,combine:E.combine,mapUv:ut&&x(E.map.channel),aoMapUv:Nt&&x(E.aoMap.channel),lightMapUv:wt&&x(E.lightMap.channel),bumpMapUv:Ct&&x(E.bumpMap.channel),normalMapUv:vt&&x(E.normalMap.channel),displacementMapUv:Jt&&x(E.displacementMap.channel),emissiveMapUv:Lt&&x(E.emissiveMap.channel),metalnessMapUv:I&&x(E.metalnessMap.channel),roughnessMapUv:R&&x(E.roughnessMap.channel),anisotropyMapUv:rt&&x(E.anisotropyMap.channel),clearcoatMapUv:gt&&x(E.clearcoatMap.channel),clearcoatNormalMapUv:Pt&&x(E.clearcoatNormalMap.channel),clearcoatRoughnessMapUv:Wt&&x(E.clearcoatRoughnessMap.channel),iridescenceMapUv:et&&x(E.iridescenceMap.channel),iridescenceThicknessMapUv:oe&&x(E.iridescenceThicknessMap.channel),sheenColorMapUv:jt&&x(E.sheenColorMap.channel),sheenRoughnessMapUv:kt&&x(E.sheenRoughnessMap.channel),specularMapUv:At&&x(E.specularMap.channel),specularColorMapUv:xt&&x(E.specularColorMap.channel),specularIntensityMapUv:Ft&&x(E.specularIntensityMap.channel),transmissionMapUv:Qt&&x(E.transmissionMap.channel),thicknessMapUv:fe&&x(E.thicknessMap.channel),alphaMapUv:ot&&x(E.alphaMap.channel),vertexTangents:!!N.attributes.tangent&&(vt||W),vertexColors:E.vertexColors,vertexAlphas:E.vertexColors===!0&&!!N.attributes.color&&N.attributes.color.itemSize===4,vertexUv1s:Gt,vertexUv2s:Ut,vertexUv3s:de,pointsUvs:F.isPoints===!0&&!!N.attributes.uv&&(ut||ot),fog:!!L,useFog:E.fog===!0,fogExp2:L&&L.isFogExp2,flatShading:E.flatShading===!0,sizeAttenuation:E.sizeAttenuation===!0,logarithmicDepthBuffer:u,skinning:F.isSkinnedMesh===!0,morphTargets:N.morphAttributes.position!==void 0,morphNormals:N.morphAttributes.normal!==void 0,morphColors:N.morphAttributes.color!==void 0,morphTargetsCount:J,morphTextureStride:nt,numDirLights:A.directional.length,numPointLights:A.point.length,numSpotLights:A.spot.length,numSpotLightMaps:A.spotLightMap.length,numRectAreaLights:A.rectArea.length,numHemiLights:A.hemi.length,numDirLightShadows:A.directionalShadowMap.length,numPointLightShadows:A.pointShadowMap.length,numSpotLightShadows:A.spotShadowMap.length,numSpotLightShadowsWithMaps:A.numSpotLightShadowsWithMaps,numLightProbes:A.numLightProbes,numClippingPlanes:a.numPlanes,numClipIntersection:a.numIntersection,dithering:E.dithering,shadowMapEnabled:r.shadowMap.enabled&&D.length>0,shadowMapType:r.shadowMap.type,toneMapping:ce,useLegacyLights:r._useLegacyLights,decodeVideoTexture:ut&&E.map.isVideoTexture===!0&&ge.getTransfer(E.map.colorSpace)===ye,premultipliedAlpha:E.premultipliedAlpha,doubleSided:E.side===2,flipSided:E.side===1,useDepthPacking:E.depthPacking>=0,depthPacking:E.depthPacking||0,index0AttributeName:E.index0AttributeName,extensionDerivatives:ft&&E.extensions.derivatives===!0,extensionFragDepth:ft&&E.extensions.fragDepth===!0,extensionDrawBuffers:ft&&E.extensions.drawBuffers===!0,extensionShaderTextureLOD:ft&&E.extensions.shaderTextureLOD===!0,extensionClipCullDistance:ft&&E.extensions.clipCullDistance&&n.has("WEBGL_clip_cull_distance"),rendererExtensionFragDepth:h||n.has("EXT_frag_depth"),rendererExtensionDrawBuffers:h||n.has("WEBGL_draw_buffers"),rendererExtensionShaderTextureLod:h||n.has("EXT_shader_texture_lod"),rendererExtensionParallelShaderCompile:n.has("KHR_parallel_shader_compile"),customProgramCacheKey:E.customProgramCacheKey()}}function p(E){const A=[];if(E.shaderID?A.push(E.shaderID):(A.push(E.customVertexShaderID),A.push(E.customFragmentShaderID)),E.defines!==void 0)for(const D in E.defines)A.push(D),A.push(E.defines[D]);return E.isRawShaderMaterial===!1&&(_(A,E),v(A,E),A.push(r.outputColorSpace)),A.push(E.customProgramCacheKey),A.join()}function _(E,A){E.push(A.precision),E.push(A.outputColorSpace),E.push(A.envMapMode),E.push(A.envMapCubeUVHeight),E.push(A.mapUv),E.push(A.alphaMapUv),E.push(A.lightMapUv),E.push(A.aoMapUv),E.push(A.bumpMapUv),E.push(A.normalMapUv),E.push(A.displacementMapUv),E.push(A.emissiveMapUv),E.push(A.metalnessMapUv),E.push(A.roughnessMapUv),E.push(A.anisotropyMapUv),E.push(A.clearcoatMapUv),E.push(A.clearcoatNormalMapUv),E.push(A.clearcoatRoughnessMapUv),E.push(A.iridescenceMapUv),E.push(A.iridescenceThicknessMapUv),E.push(A.sheenColorMapUv),E.push(A.sheenRoughnessMapUv),E.push(A.specularMapUv),E.push(A.specularColorMapUv),E.push(A.specularIntensityMapUv),E.push(A.transmissionMapUv),E.push(A.thicknessMapUv),E.push(A.combine),E.push(A.fogExp2),E.push(A.sizeAttenuation),E.push(A.morphTargetsCount),E.push(A.morphAttributeCount),E.push(A.numDirLights),E.push(A.numPointLights),E.push(A.numSpotLights),E.push(A.numSpotLightMaps),E.push(A.numHemiLights),E.push(A.numRectAreaLights),E.push(A.numDirLightShadows),E.push(A.numPointLightShadows),E.push(A.numSpotLightShadows),E.push(A.numSpotLightShadowsWithMaps),E.push(A.numLightProbes),E.push(A.shadowMapType),E.push(A.toneMapping),E.push(A.numClippingPlanes),E.push(A.numClipIntersection),E.push(A.depthPacking)}function v(E,A){o.disableAll(),A.isWebGL2&&o.enable(0),A.supportsVertexTextures&&o.enable(1),A.instancing&&o.enable(2),A.instancingColor&&o.enable(3),A.matcap&&o.enable(4),A.envMap&&o.enable(5),A.normalMapObjectSpace&&o.enable(6),A.normalMapTangentSpace&&o.enable(7),A.clearcoat&&o.enable(8),A.iridescence&&o.enable(9),A.alphaTest&&o.enable(10),A.vertexColors&&o.enable(11),A.vertexAlphas&&o.enable(12),A.vertexUv1s&&o.enable(13),A.vertexUv2s&&o.enable(14),A.vertexUv3s&&o.enable(15),A.vertexTangents&&o.enable(16),A.anisotropy&&o.enable(17),A.alphaHash&&o.enable(18),A.batching&&o.enable(19),E.push(o.mask),o.disableAll(),A.fog&&o.enable(0),A.useFog&&o.enable(1),A.flatShading&&o.enable(2),A.logarithmicDepthBuffer&&o.enable(3),A.skinning&&o.enable(4),A.morphTargets&&o.enable(5),A.morphNormals&&o.enable(6),A.morphColors&&o.enable(7),A.premultipliedAlpha&&o.enable(8),A.shadowMapEnabled&&o.enable(9),A.useLegacyLights&&o.enable(10),A.doubleSided&&o.enable(11),A.flipSided&&o.enable(12),A.useDepthPacking&&o.enable(13),A.dithering&&o.enable(14),A.transmission&&o.enable(15),A.sheen&&o.enable(16),A.opaque&&o.enable(17),A.pointsUvs&&o.enable(18),A.decodeVideoTexture&&o.enable(19),E.push(o.mask)}function S(E){const A=m[E.type];let D;if(A){const b=li[A];D=Pm.clone(b.uniforms)}else D=E.uniforms;return D}function M(E,A){let D;for(let b=0,F=l.length;b<F;b++){const L=l[b];if(L.cacheKey===A){D=L,++D.usedTimes;break}}return D===void 0&&(D=new W_(r,A,E,s),l.push(D)),D}function y(E){if(--E.usedTimes===0){const A=l.indexOf(E);l[A]=l[l.length-1],l.pop(),E.destroy()}}function w(E){c.remove(E)}function T(){c.dispose()}return{getParameters:g,getProgramCacheKey:p,getUniforms:S,acquireProgram:M,releaseProgram:y,releaseShaderCache:w,programs:l,dispose:T}}function j_(){let r=new WeakMap;function t(s){let a=r.get(s);return a===void 0&&(a={},r.set(s,a)),a}function e(s){r.delete(s)}function n(s,a,o){r.get(s)[a]=o}function i(){r=new WeakMap}return{get:t,remove:e,update:n,dispose:i}}function $_(r,t){return r.groupOrder!==t.groupOrder?r.groupOrder-t.groupOrder:r.renderOrder!==t.renderOrder?r.renderOrder-t.renderOrder:r.material.id!==t.material.id?r.material.id-t.material.id:r.z!==t.z?r.z-t.z:r.id-t.id}function Du(r,t){return r.groupOrder!==t.groupOrder?r.groupOrder-t.groupOrder:r.renderOrder!==t.renderOrder?r.renderOrder-t.renderOrder:r.z!==t.z?t.z-r.z:r.id-t.id}function Iu(){const r=[];let t=0;const e=[],n=[],i=[];function s(){t=0,e.length=0,n.length=0,i.length=0}function a(u,d,f,m,x,g){let p=r[t];return p===void 0?(p={id:u.id,object:u,geometry:d,material:f,groupOrder:m,renderOrder:u.renderOrder,z:x,group:g},r[t]=p):(p.id=u.id,p.object=u,p.geometry=d,p.material=f,p.groupOrder=m,p.renderOrder=u.renderOrder,p.z=x,p.group=g),t++,p}function o(u,d,f,m,x,g){const p=a(u,d,f,m,x,g);f.transmission>0?n.push(p):f.transparent===!0?i.push(p):e.push(p)}function c(u,d,f,m,x,g){const p=a(u,d,f,m,x,g);f.transmission>0?n.unshift(p):f.transparent===!0?i.unshift(p):e.unshift(p)}function l(u,d){e.length>1&&e.sort(u||$_),n.length>1&&n.sort(d||Du),i.length>1&&i.sort(d||Du)}function h(){for(let u=t,d=r.length;u<d;u++){const f=r[u];if(f.id===null)break;f.id=null,f.object=null,f.geometry=null,f.material=null,f.group=null}}return{opaque:e,transmissive:n,transparent:i,init:s,push:o,unshift:c,finish:h,sort:l}}function K_(){let r=new WeakMap;function t(n,i){const s=r.get(n);let a;return s===void 0?(a=new Iu,r.set(n,[a])):i>=s.length?(a=new Iu,s.push(a)):a=s[i],a}function e(){r=new WeakMap}return{get:t,dispose:e}}function J_(){const r={};return{get:function(t){if(r[t.id]!==void 0)return r[t.id];let e;switch(t.type){case"DirectionalLight":e={direction:new C,color:new Kt};break;case"SpotLight":e={position:new C,direction:new C,color:new Kt,distance:0,coneCos:0,penumbraCos:0,decay:0};break;case"PointLight":e={position:new C,color:new Kt,distance:0,decay:0};break;case"HemisphereLight":e={direction:new C,skyColor:new Kt,groundColor:new Kt};break;case"RectAreaLight":e={color:new Kt,position:new C,halfWidth:new C,halfHeight:new C};break}return r[t.id]=e,e}}}function Q_(){const r={};return{get:function(t){if(r[t.id]!==void 0)return r[t.id];let e;switch(t.type){case"DirectionalLight":e={shadowBias:0,shadowNormalBias:0,shadowRadius:1,shadowMapSize:new yt};break;case"SpotLight":e={shadowBias:0,shadowNormalBias:0,shadowRadius:1,shadowMapSize:new yt};break;case"PointLight":e={shadowBias:0,shadowNormalBias:0,shadowRadius:1,shadowMapSize:new yt,shadowCameraNear:1,shadowCameraFar:1e3};break}return r[t.id]=e,e}}}let tv=0;function ev(r,t){return(t.castShadow?2:0)-(r.castShadow?2:0)+(t.map?1:0)-(r.map?1:0)}function nv(r,t){const e=new J_,n=Q_(),i={version:0,hash:{directionalLength:-1,pointLength:-1,spotLength:-1,rectAreaLength:-1,hemiLength:-1,numDirectionalShadows:-1,numPointShadows:-1,numSpotShadows:-1,numSpotMaps:-1,numLightProbes:-1},ambient:[0,0,0],probe:[],directional:[],directionalShadow:[],directionalShadowMap:[],directionalShadowMatrix:[],spot:[],spotLightMap:[],spotShadow:[],spotShadowMap:[],spotLightMatrix:[],rectArea:[],rectAreaLTC1:null,rectAreaLTC2:null,point:[],pointShadow:[],pointShadowMap:[],pointShadowMatrix:[],hemi:[],numSpotLightShadowsWithMaps:0,numLightProbes:0};for(let h=0;h<9;h++)i.probe.push(new C);const s=new C,a=new re,o=new re;function c(h,u){let d=0,f=0,m=0;for(let b=0;b<9;b++)i.probe[b].set(0,0,0);let x=0,g=0,p=0,_=0,v=0,S=0,M=0,y=0,w=0,T=0,E=0;h.sort(ev);const A=u===!0?Math.PI:1;for(let b=0,F=h.length;b<F;b++){const L=h[b],N=L.color,U=L.intensity,k=L.distance,O=L.shadow&&L.shadow.map?L.shadow.map.texture:null;if(L.isAmbientLight)d+=N.r*U*A,f+=N.g*U*A,m+=N.b*U*A;else if(L.isLightProbe){for(let H=0;H<9;H++)i.probe[H].addScaledVector(L.sh.coefficients[H],U);E++}else if(L.isDirectionalLight){const H=e.get(L);if(H.color.copy(L.color).multiplyScalar(L.intensity*A),L.castShadow){const j=L.shadow,J=n.get(L);J.shadowBias=j.bias,J.shadowNormalBias=j.normalBias,J.shadowRadius=j.radius,J.shadowMapSize=j.mapSize,i.directionalShadow[x]=J,i.directionalShadowMap[x]=O,i.directionalShadowMatrix[x]=L.shadow.matrix,S++}i.directional[x]=H,x++}else if(L.isSpotLight){const H=e.get(L);H.position.setFromMatrixPosition(L.matrixWorld),H.color.copy(N).multiplyScalar(U*A),H.distance=k,H.coneCos=Math.cos(L.angle),H.penumbraCos=Math.cos(L.angle*(1-L.penumbra)),H.decay=L.decay,i.spot[p]=H;const j=L.shadow;if(L.map&&(i.spotLightMap[w]=L.map,w++,j.updateMatrices(L),L.castShadow&&T++),i.spotLightMatrix[p]=j.matrix,L.castShadow){const J=n.get(L);J.shadowBias=j.bias,J.shadowNormalBias=j.normalBias,J.shadowRadius=j.radius,J.shadowMapSize=j.mapSize,i.spotShadow[p]=J,i.spotShadowMap[p]=O,y++}p++}else if(L.isRectAreaLight){const H=e.get(L);H.color.copy(N).multiplyScalar(U),H.halfWidth.set(L.width*.5,0,0),H.halfHeight.set(0,L.height*.5,0),i.rectArea[_]=H,_++}else if(L.isPointLight){const H=e.get(L);if(H.color.copy(L.color).multiplyScalar(L.intensity*A),H.distance=L.distance,H.decay=L.decay,L.castShadow){const j=L.shadow,J=n.get(L);J.shadowBias=j.bias,J.shadowNormalBias=j.normalBias,J.shadowRadius=j.radius,J.shadowMapSize=j.mapSize,J.shadowCameraNear=j.camera.near,J.shadowCameraFar=j.camera.far,i.pointShadow[g]=J,i.pointShadowMap[g]=O,i.pointShadowMatrix[g]=L.shadow.matrix,M++}i.point[g]=H,g++}else if(L.isHemisphereLight){const H=e.get(L);H.skyColor.copy(L.color).multiplyScalar(U*A),H.groundColor.copy(L.groundColor).multiplyScalar(U*A),i.hemi[v]=H,v++}}_>0&&(t.isWebGL2?r.has("OES_texture_float_linear")===!0?(i.rectAreaLTC1=pt.LTC_FLOAT_1,i.rectAreaLTC2=pt.LTC_FLOAT_2):(i.rectAreaLTC1=pt.LTC_HALF_1,i.rectAreaLTC2=pt.LTC_HALF_2):r.has("OES_texture_float_linear")===!0?(i.rectAreaLTC1=pt.LTC_FLOAT_1,i.rectAreaLTC2=pt.LTC_FLOAT_2):r.has("OES_texture_half_float_linear")===!0?(i.rectAreaLTC1=pt.LTC_HALF_1,i.rectAreaLTC2=pt.LTC_HALF_2):console.error("THREE.WebGLRenderer: Unable to use RectAreaLight. Missing WebGL extensions.")),i.ambient[0]=d,i.ambient[1]=f,i.ambient[2]=m;const D=i.hash;(D.directionalLength!==x||D.pointLength!==g||D.spotLength!==p||D.rectAreaLength!==_||D.hemiLength!==v||D.numDirectionalShadows!==S||D.numPointShadows!==M||D.numSpotShadows!==y||D.numSpotMaps!==w||D.numLightProbes!==E)&&(i.directional.length=x,i.spot.length=p,i.rectArea.length=_,i.point.length=g,i.hemi.length=v,i.directionalShadow.length=S,i.directionalShadowMap.length=S,i.pointShadow.length=M,i.pointShadowMap.length=M,i.spotShadow.length=y,i.spotShadowMap.length=y,i.directionalShadowMatrix.length=S,i.pointShadowMatrix.length=M,i.spotLightMatrix.length=y+w-T,i.spotLightMap.length=w,i.numSpotLightShadowsWithMaps=T,i.numLightProbes=E,D.directionalLength=x,D.pointLength=g,D.spotLength=p,D.rectAreaLength=_,D.hemiLength=v,D.numDirectionalShadows=S,D.numPointShadows=M,D.numSpotShadows=y,D.numSpotMaps=w,D.numLightProbes=E,i.version=tv++)}function l(h,u){let d=0,f=0,m=0,x=0,g=0;const p=u.matrixWorldInverse;for(let _=0,v=h.length;_<v;_++){const S=h[_];if(S.isDirectionalLight){const M=i.directional[d];M.direction.setFromMatrixPosition(S.matrixWorld),s.setFromMatrixPosition(S.target.matrixWorld),M.direction.sub(s),M.direction.transformDirection(p),d++}else if(S.isSpotLight){const M=i.spot[m];M.position.setFromMatrixPosition(S.matrixWorld),M.position.applyMatrix4(p),M.direction.setFromMatrixPosition(S.matrixWorld),s.setFromMatrixPosition(S.target.matrixWorld),M.direction.sub(s),M.direction.transformDirection(p),m++}else if(S.isRectAreaLight){const M=i.rectArea[x];M.position.setFromMatrixPosition(S.matrixWorld),M.position.applyMatrix4(p),o.identity(),a.copy(S.matrixWorld),a.premultiply(p),o.extractRotation(a),M.halfWidth.set(S.width*.5,0,0),M.halfHeight.set(0,S.height*.5,0),M.halfWidth.applyMatrix4(o),M.halfHeight.applyMatrix4(o),x++}else if(S.isPointLight){const M=i.point[f];M.position.setFromMatrixPosition(S.matrixWorld),M.position.applyMatrix4(p),f++}else if(S.isHemisphereLight){const M=i.hemi[g];M.direction.setFromMatrixPosition(S.matrixWorld),M.direction.transformDirection(p),g++}}}return{setup:c,setupView:l,state:i}}function Nu(r,t){const e=new nv(r,t),n=[],i=[];function s(){n.length=0,i.length=0}function a(u){n.push(u)}function o(u){i.push(u)}function c(u){e.setup(n,u)}function l(u){e.setupView(n,u)}return{init:s,state:{lightsArray:n,shadowsArray:i,lights:e},setupLights:c,setupLightsView:l,pushLight:a,pushShadow:o}}function iv(r,t){let e=new WeakMap;function n(s,a=0){const o=e.get(s);let c;return o===void 0?(c=new Nu(r,t),e.set(s,[c])):a>=o.length?(c=new Nu(r,t),o.push(c)):c=o[a],c}function i(){e=new WeakMap}return{get:n,dispose:i}}class sv extends pi{constructor(t){super(),this.isMeshDepthMaterial=!0,this.type="MeshDepthMaterial",this.depthPacking=3200,this.map=null,this.alphaMap=null,this.displacementMap=null,this.displacementScale=1,this.displacementBias=0,this.wireframe=!1,this.wireframeLinewidth=1,this.setValues(t)}copy(t){return super.copy(t),this.depthPacking=t.depthPacking,this.map=t.map,this.alphaMap=t.alphaMap,this.displacementMap=t.displacementMap,this.displacementScale=t.displacementScale,this.displacementBias=t.displacementBias,this.wireframe=t.wireframe,this.wireframeLinewidth=t.wireframeLinewidth,this}}class rv extends pi{constructor(t){super(),this.isMeshDistanceMaterial=!0,this.type="MeshDistanceMaterial",this.map=null,this.alphaMap=null,this.displacementMap=null,this.displacementScale=1,this.displacementBias=0,this.setValues(t)}copy(t){return super.copy(t),this.map=t.map,this.alphaMap=t.alphaMap,this.displacementMap=t.displacementMap,this.displacementScale=t.displacementScale,this.displacementBias=t.displacementBias,this}}const av=`void main() {
	gl_Position = vec4( position, 1.0 );
}`,ov=`uniform sampler2D shadow_pass;
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
}`;function cv(r,t,e){let n=new wh;const i=new yt,s=new yt,a=new Ke,o=new sv({depthPacking:3201}),c=new rv,l={},h=e.maxTextureSize,u={0:1,1:0,2:2},d=new Un({defines:{VSM_SAMPLES:8},uniforms:{shadow_pass:{value:null},resolution:{value:new yt},radius:{value:4}},vertexShader:av,fragmentShader:ov}),f=d.clone();f.defines.HORIZONTAL_PASS=1;const m=new we;m.setAttribute("position",new Fe(new Float32Array([-1,-1,.5,3,-1,.5,-1,3,.5]),3));const x=new $t(m,d),g=this;this.enabled=!1,this.autoUpdate=!0,this.needsUpdate=!1,this.type=1;let p=this.type;this.render=function(y,w,T){if(g.enabled===!1||g.autoUpdate===!1&&g.needsUpdate===!1||y.length===0)return;const E=r.getRenderTarget(),A=r.getActiveCubeFace(),D=r.getActiveMipmapLevel(),b=r.state;b.setBlending(0),b.buffers.color.setClear(1,1,1,1),b.buffers.depth.setTest(!0),b.setScissorTest(!1);const F=p!==3&&this.type===3,L=p===3&&this.type!==3;for(let N=0,U=y.length;N<U;N++){const k=y[N],O=k.shadow;if(O===void 0){console.warn("THREE.WebGLShadowMap:",k,"has no shadow.");continue}if(O.autoUpdate===!1&&O.needsUpdate===!1)continue;i.copy(O.mapSize);const H=O.getFrameExtents();if(i.multiply(H),s.copy(O.mapSize),(i.x>h||i.y>h)&&(i.x>h&&(s.x=Math.floor(h/H.x),i.x=s.x*H.x,O.mapSize.x=s.x),i.y>h&&(s.y=Math.floor(h/H.y),i.y=s.y*H.y,O.mapSize.y=s.y)),O.map===null||F===!0||L===!0){const J=this.type!==3?{minFilter:1003,magFilter:1003}:{};O.map!==null&&O.map.dispose(),O.map=new Qe(i.x,i.y,J),O.map.texture.name=k.name+".shadowMap",O.camera.updateProjectionMatrix()}r.setRenderTarget(O.map),r.clear();const j=O.getViewportCount();for(let J=0;J<j;J++){const nt=O.getViewport(J);a.set(s.x*nt.x,s.y*nt.y,s.x*nt.z,s.y*nt.w),b.viewport(a),O.updateMatrices(k,J),n=O.getFrustum(),S(w,T,O.camera,k,this.type)}O.isPointLightShadow!==!0&&this.type===3&&_(O,T),O.needsUpdate=!1}p=this.type,g.needsUpdate=!1,r.setRenderTarget(E,A,D)};function _(y,w){const T=t.update(x);d.defines.VSM_SAMPLES!==y.blurSamples&&(d.defines.VSM_SAMPLES=y.blurSamples,f.defines.VSM_SAMPLES=y.blurSamples,d.needsUpdate=!0,f.needsUpdate=!0),y.mapPass===null&&(y.mapPass=new Qe(i.x,i.y)),d.uniforms.shadow_pass.value=y.map.texture,d.uniforms.resolution.value=y.mapSize,d.uniforms.radius.value=y.radius,r.setRenderTarget(y.mapPass),r.clear(),r.renderBufferDirect(w,null,T,d,x,null),f.uniforms.shadow_pass.value=y.mapPass.texture,f.uniforms.resolution.value=y.mapSize,f.uniforms.radius.value=y.radius,r.setRenderTarget(y.map),r.clear(),r.renderBufferDirect(w,null,T,f,x,null)}function v(y,w,T,E){let A=null;const D=T.isPointLight===!0?y.customDistanceMaterial:y.customDepthMaterial;if(D!==void 0)A=D;else if(A=T.isPointLight===!0?c:o,r.localClippingEnabled&&w.clipShadows===!0&&Array.isArray(w.clippingPlanes)&&w.clippingPlanes.length!==0||w.displacementMap&&w.displacementScale!==0||w.alphaMap&&w.alphaTest>0||w.map&&w.alphaTest>0){const b=A.uuid,F=w.uuid;let L=l[b];L===void 0&&(L={},l[b]=L);let N=L[F];N===void 0&&(N=A.clone(),L[F]=N,w.addEventListener("dispose",M)),A=N}if(A.visible=w.visible,A.wireframe=w.wireframe,E===3?A.side=w.shadowSide!==null?w.shadowSide:w.side:A.side=w.shadowSide!==null?w.shadowSide:u[w.side],A.alphaMap=w.alphaMap,A.alphaTest=w.alphaTest,A.map=w.map,A.clipShadows=w.clipShadows,A.clippingPlanes=w.clippingPlanes,A.clipIntersection=w.clipIntersection,A.displacementMap=w.displacementMap,A.displacementScale=w.displacementScale,A.displacementBias=w.displacementBias,A.wireframeLinewidth=w.wireframeLinewidth,A.linewidth=w.linewidth,T.isPointLight===!0&&A.isMeshDistanceMaterial===!0){const b=r.properties.get(A);b.light=T}return A}function S(y,w,T,E,A){if(y.visible===!1)return;if(y.layers.test(w.layers)&&(y.isMesh||y.isLine||y.isPoints)&&(y.castShadow||y.receiveShadow&&A===3)&&(!y.frustumCulled||n.intersectsObject(y))){y.modelViewMatrix.multiplyMatrices(T.matrixWorldInverse,y.matrixWorld);const F=t.update(y),L=y.material;if(Array.isArray(L)){const N=F.groups;for(let U=0,k=N.length;U<k;U++){const O=N[U],H=L[O.materialIndex];if(H&&H.visible){const j=v(y,H,E,A);y.onBeforeShadow(r,y,w,T,F,j,O),r.renderBufferDirect(T,null,F,j,y,O),y.onAfterShadow(r,y,w,T,F,j,O)}}}else if(L.visible){const N=v(y,L,E,A);y.onBeforeShadow(r,y,w,T,F,N,null),r.renderBufferDirect(T,null,F,N,y,null),y.onAfterShadow(r,y,w,T,F,N,null)}}const b=y.children;for(let F=0,L=b.length;F<L;F++)S(b[F],w,T,E,A)}function M(y){y.target.removeEventListener("dispose",M);for(const T in l){const E=l[T],A=y.target.uuid;A in E&&(E[A].dispose(),delete E[A])}}}function lv(r,t,e){const n=e.isWebGL2;function i(){let B=!1;const dt=new Ke;let ft=null;const Gt=new Ke(0,0,0,0);return{setMask:function(Ut){ft!==Ut&&!B&&(r.colorMask(Ut,Ut,Ut,Ut),ft=Ut)},setLocked:function(Ut){B=Ut},setClear:function(Ut,de,ce,ve,Ce){Ce===!0&&(Ut*=ve,de*=ve,ce*=ve),dt.set(Ut,de,ce,ve),Gt.equals(dt)===!1&&(r.clearColor(Ut,de,ce,ve),Gt.copy(dt))},reset:function(){B=!1,ft=null,Gt.set(-1,0,0,0)}}}function s(){let B=!1,dt=null,ft=null,Gt=null;return{setTest:function(Ut){Ut?ht(r.DEPTH_TEST):ut(r.DEPTH_TEST)},setMask:function(Ut){dt!==Ut&&!B&&(r.depthMask(Ut),dt=Ut)},setFunc:function(Ut){if(ft!==Ut){switch(Ut){case 0:r.depthFunc(r.NEVER);break;case 1:r.depthFunc(r.ALWAYS);break;case 2:r.depthFunc(r.LESS);break;case 3:r.depthFunc(r.LEQUAL);break;case 4:r.depthFunc(r.EQUAL);break;case 5:r.depthFunc(r.GEQUAL);break;case 6:r.depthFunc(r.GREATER);break;case 7:r.depthFunc(r.NOTEQUAL);break;default:r.depthFunc(r.LEQUAL)}ft=Ut}},setLocked:function(Ut){B=Ut},setClear:function(Ut){Gt!==Ut&&(r.clearDepth(Ut),Gt=Ut)},reset:function(){B=!1,dt=null,ft=null,Gt=null}}}function a(){let B=!1,dt=null,ft=null,Gt=null,Ut=null,de=null,ce=null,ve=null,Ce=null;return{setTest:function(ae){B||(ae?ht(r.STENCIL_TEST):ut(r.STENCIL_TEST))},setMask:function(ae){dt!==ae&&!B&&(r.stencilMask(ae),dt=ae)},setFunc:function(ae,Le,qe){(ft!==ae||Gt!==Le||Ut!==qe)&&(r.stencilFunc(ae,Le,qe),ft=ae,Gt=Le,Ut=qe)},setOp:function(ae,Le,qe){(de!==ae||ce!==Le||ve!==qe)&&(r.stencilOp(ae,Le,qe),de=ae,ce=Le,ve=qe)},setLocked:function(ae){B=ae},setClear:function(ae){Ce!==ae&&(r.clearStencil(ae),Ce=ae)},reset:function(){B=!1,dt=null,ft=null,Gt=null,Ut=null,de=null,ce=null,ve=null,Ce=null}}}const o=new i,c=new s,l=new a,h=new WeakMap,u=new WeakMap;let d={},f={},m=new WeakMap,x=[],g=null,p=!1,_=null,v=null,S=null,M=null,y=null,w=null,T=null,E=new Kt(0,0,0),A=0,D=!1,b=null,F=null,L=null,N=null,U=null;const k=r.getParameter(r.MAX_COMBINED_TEXTURE_IMAGE_UNITS);let O=!1,H=0;const j=r.getParameter(r.VERSION);j.indexOf("WebGL")!==-1?(H=parseFloat(/^WebGL (\d)/.exec(j)[1]),O=H>=1):j.indexOf("OpenGL ES")!==-1&&(H=parseFloat(/^OpenGL ES (\d)/.exec(j)[1]),O=H>=2);let J=null,nt={};const V=r.getParameter(r.SCISSOR_BOX),$=r.getParameter(r.VIEWPORT),st=new Ke().fromArray(V),at=new Ke().fromArray($);function lt(B,dt,ft,Gt){const Ut=new Uint8Array(4),de=r.createTexture();r.bindTexture(B,de),r.texParameteri(B,r.TEXTURE_MIN_FILTER,r.NEAREST),r.texParameteri(B,r.TEXTURE_MAG_FILTER,r.NEAREST);for(let ce=0;ce<ft;ce++)n&&(B===r.TEXTURE_3D||B===r.TEXTURE_2D_ARRAY)?r.texImage3D(dt,0,r.RGBA,1,1,Gt,0,r.RGBA,r.UNSIGNED_BYTE,Ut):r.texImage2D(dt+ce,0,r.RGBA,1,1,0,r.RGBA,r.UNSIGNED_BYTE,Ut);return de}const _t={};_t[r.TEXTURE_2D]=lt(r.TEXTURE_2D,r.TEXTURE_2D,1),_t[r.TEXTURE_CUBE_MAP]=lt(r.TEXTURE_CUBE_MAP,r.TEXTURE_CUBE_MAP_POSITIVE_X,6),n&&(_t[r.TEXTURE_2D_ARRAY]=lt(r.TEXTURE_2D_ARRAY,r.TEXTURE_2D_ARRAY,1,1),_t[r.TEXTURE_3D]=lt(r.TEXTURE_3D,r.TEXTURE_3D,1,1)),o.setClear(0,0,0,1),c.setClear(1),l.setClear(0),ht(r.DEPTH_TEST),c.setFunc(3),Lt(!1),I(1),ht(r.CULL_FACE),vt(0);function ht(B){d[B]!==!0&&(r.enable(B),d[B]=!0)}function ut(B){d[B]!==!1&&(r.disable(B),d[B]=!1)}function bt(B,dt){return f[B]!==dt?(r.bindFramebuffer(B,dt),f[B]=dt,n&&(B===r.DRAW_FRAMEBUFFER&&(f[r.FRAMEBUFFER]=dt),B===r.FRAMEBUFFER&&(f[r.DRAW_FRAMEBUFFER]=dt)),!0):!1}function Y(B,dt){let ft=x,Gt=!1;if(B)if(ft=m.get(dt),ft===void 0&&(ft=[],m.set(dt,ft)),B.isWebGLMultipleRenderTargets){const Ut=B.texture;if(ft.length!==Ut.length||ft[0]!==r.COLOR_ATTACHMENT0){for(let de=0,ce=Ut.length;de<ce;de++)ft[de]=r.COLOR_ATTACHMENT0+de;ft.length=Ut.length,Gt=!0}}else ft[0]!==r.COLOR_ATTACHMENT0&&(ft[0]=r.COLOR_ATTACHMENT0,Gt=!0);else ft[0]!==r.BACK&&(ft[0]=r.BACK,Gt=!0);Gt&&(e.isWebGL2?r.drawBuffers(ft):t.get("WEBGL_draw_buffers").drawBuffersWEBGL(ft))}function Nt(B){return g!==B?(r.useProgram(B),g=B,!0):!1}const wt={100:r.FUNC_ADD,101:r.FUNC_SUBTRACT,102:r.FUNC_REVERSE_SUBTRACT};if(n)wt[103]=r.MIN,wt[104]=r.MAX;else{const B=t.get("EXT_blend_minmax");B!==null&&(wt[103]=B.MIN_EXT,wt[104]=B.MAX_EXT)}const Ct={200:r.ZERO,201:r.ONE,202:r.SRC_COLOR,204:r.SRC_ALPHA,210:r.SRC_ALPHA_SATURATE,208:r.DST_COLOR,206:r.DST_ALPHA,203:r.ONE_MINUS_SRC_COLOR,205:r.ONE_MINUS_SRC_ALPHA,209:r.ONE_MINUS_DST_COLOR,207:r.ONE_MINUS_DST_ALPHA,211:r.CONSTANT_COLOR,212:r.ONE_MINUS_CONSTANT_COLOR,213:r.CONSTANT_ALPHA,214:r.ONE_MINUS_CONSTANT_ALPHA};function vt(B,dt,ft,Gt,Ut,de,ce,ve,Ce,ae){if(B===0){p===!0&&(ut(r.BLEND),p=!1);return}if(p===!1&&(ht(r.BLEND),p=!0),B!==5){if(B!==_||ae!==D){if((v!==100||y!==100)&&(r.blendEquation(r.FUNC_ADD),v=100,y=100),ae)switch(B){case 1:r.blendFuncSeparate(r.ONE,r.ONE_MINUS_SRC_ALPHA,r.ONE,r.ONE_MINUS_SRC_ALPHA);break;case 2:r.blendFunc(r.ONE,r.ONE);break;case 3:r.blendFuncSeparate(r.ZERO,r.ONE_MINUS_SRC_COLOR,r.ZERO,r.ONE);break;case 4:r.blendFuncSeparate(r.ZERO,r.SRC_COLOR,r.ZERO,r.SRC_ALPHA);break;default:console.error("THREE.WebGLState: Invalid blending: ",B);break}else switch(B){case 1:r.blendFuncSeparate(r.SRC_ALPHA,r.ONE_MINUS_SRC_ALPHA,r.ONE,r.ONE_MINUS_SRC_ALPHA);break;case 2:r.blendFunc(r.SRC_ALPHA,r.ONE);break;case 3:r.blendFuncSeparate(r.ZERO,r.ONE_MINUS_SRC_COLOR,r.ZERO,r.ONE);break;case 4:r.blendFunc(r.ZERO,r.SRC_COLOR);break;default:console.error("THREE.WebGLState: Invalid blending: ",B);break}S=null,M=null,w=null,T=null,E.set(0,0,0),A=0,_=B,D=ae}return}Ut=Ut||dt,de=de||ft,ce=ce||Gt,(dt!==v||Ut!==y)&&(r.blendEquationSeparate(wt[dt],wt[Ut]),v=dt,y=Ut),(ft!==S||Gt!==M||de!==w||ce!==T)&&(r.blendFuncSeparate(Ct[ft],Ct[Gt],Ct[de],Ct[ce]),S=ft,M=Gt,w=de,T=ce),(ve.equals(E)===!1||Ce!==A)&&(r.blendColor(ve.r,ve.g,ve.b,Ce),E.copy(ve),A=Ce),_=B,D=!1}function Jt(B,dt){B.side===2?ut(r.CULL_FACE):ht(r.CULL_FACE);let ft=B.side===1;dt&&(ft=!ft),Lt(ft),B.blending===1&&B.transparent===!1?vt(0):vt(B.blending,B.blendEquation,B.blendSrc,B.blendDst,B.blendEquationAlpha,B.blendSrcAlpha,B.blendDstAlpha,B.blendColor,B.blendAlpha,B.premultipliedAlpha),c.setFunc(B.depthFunc),c.setTest(B.depthTest),c.setMask(B.depthWrite),o.setMask(B.colorWrite);const Gt=B.stencilWrite;l.setTest(Gt),Gt&&(l.setMask(B.stencilWriteMask),l.setFunc(B.stencilFunc,B.stencilRef,B.stencilFuncMask),l.setOp(B.stencilFail,B.stencilZFail,B.stencilZPass)),W(B.polygonOffset,B.polygonOffsetFactor,B.polygonOffsetUnits),B.alphaToCoverage===!0?ht(r.SAMPLE_ALPHA_TO_COVERAGE):ut(r.SAMPLE_ALPHA_TO_COVERAGE)}function Lt(B){b!==B&&(B?r.frontFace(r.CW):r.frontFace(r.CCW),b=B)}function I(B){B!==0?(ht(r.CULL_FACE),B!==F&&(B===1?r.cullFace(r.BACK):B===2?r.cullFace(r.FRONT):r.cullFace(r.FRONT_AND_BACK))):ut(r.CULL_FACE),F=B}function R(B){B!==L&&(O&&r.lineWidth(B),L=B)}function W(B,dt,ft){B?(ht(r.POLYGON_OFFSET_FILL),(N!==dt||U!==ft)&&(r.polygonOffset(dt,ft),N=dt,U=ft)):ut(r.POLYGON_OFFSET_FILL)}function Q(B){B?ht(r.SCISSOR_TEST):ut(r.SCISSOR_TEST)}function K(B){B===void 0&&(B=r.TEXTURE0+k-1),J!==B&&(r.activeTexture(B),J=B)}function tt(B,dt,ft){ft===void 0&&(J===null?ft=r.TEXTURE0+k-1:ft=J);let Gt=nt[ft];Gt===void 0&&(Gt={type:void 0,texture:void 0},nt[ft]=Gt),(Gt.type!==B||Gt.texture!==dt)&&(J!==ft&&(r.activeTexture(ft),J=ft),r.bindTexture(B,dt||_t[B]),Gt.type=B,Gt.texture=dt)}function Mt(){const B=nt[J];B!==void 0&&B.type!==void 0&&(r.bindTexture(B.type,null),B.type=void 0,B.texture=void 0)}function rt(){try{r.compressedTexImage2D.apply(r,arguments)}catch(B){console.error("THREE.WebGLState:",B)}}function gt(){try{r.compressedTexImage3D.apply(r,arguments)}catch(B){console.error("THREE.WebGLState:",B)}}function Pt(){try{r.texSubImage2D.apply(r,arguments)}catch(B){console.error("THREE.WebGLState:",B)}}function Wt(){try{r.texSubImage3D.apply(r,arguments)}catch(B){console.error("THREE.WebGLState:",B)}}function et(){try{r.compressedTexSubImage2D.apply(r,arguments)}catch(B){console.error("THREE.WebGLState:",B)}}function oe(){try{r.compressedTexSubImage3D.apply(r,arguments)}catch(B){console.error("THREE.WebGLState:",B)}}function jt(){try{r.texStorage2D.apply(r,arguments)}catch(B){console.error("THREE.WebGLState:",B)}}function kt(){try{r.texStorage3D.apply(r,arguments)}catch(B){console.error("THREE.WebGLState:",B)}}function At(){try{r.texImage2D.apply(r,arguments)}catch(B){console.error("THREE.WebGLState:",B)}}function xt(){try{r.texImage3D.apply(r,arguments)}catch(B){console.error("THREE.WebGLState:",B)}}function Ft(B){st.equals(B)===!1&&(r.scissor(B.x,B.y,B.z,B.w),st.copy(B))}function Qt(B){at.equals(B)===!1&&(r.viewport(B.x,B.y,B.z,B.w),at.copy(B))}function fe(B,dt){let ft=u.get(dt);ft===void 0&&(ft=new WeakMap,u.set(dt,ft));let Gt=ft.get(B);Gt===void 0&&(Gt=r.getUniformBlockIndex(dt,B.name),ft.set(B,Gt))}function qt(B,dt){const Gt=u.get(dt).get(B);h.get(dt)!==Gt&&(r.uniformBlockBinding(dt,Gt,B.__bindingPointIndex),h.set(dt,Gt))}function ot(){r.disable(r.BLEND),r.disable(r.CULL_FACE),r.disable(r.DEPTH_TEST),r.disable(r.POLYGON_OFFSET_FILL),r.disable(r.SCISSOR_TEST),r.disable(r.STENCIL_TEST),r.disable(r.SAMPLE_ALPHA_TO_COVERAGE),r.blendEquation(r.FUNC_ADD),r.blendFunc(r.ONE,r.ZERO),r.blendFuncSeparate(r.ONE,r.ZERO,r.ONE,r.ZERO),r.blendColor(0,0,0,0),r.colorMask(!0,!0,!0,!0),r.clearColor(0,0,0,0),r.depthMask(!0),r.depthFunc(r.LESS),r.clearDepth(1),r.stencilMask(4294967295),r.stencilFunc(r.ALWAYS,0,4294967295),r.stencilOp(r.KEEP,r.KEEP,r.KEEP),r.clearStencil(0),r.cullFace(r.BACK),r.frontFace(r.CCW),r.polygonOffset(0,0),r.activeTexture(r.TEXTURE0),r.bindFramebuffer(r.FRAMEBUFFER,null),n===!0&&(r.bindFramebuffer(r.DRAW_FRAMEBUFFER,null),r.bindFramebuffer(r.READ_FRAMEBUFFER,null)),r.useProgram(null),r.lineWidth(1),r.scissor(0,0,r.canvas.width,r.canvas.height),r.viewport(0,0,r.canvas.width,r.canvas.height),d={},J=null,nt={},f={},m=new WeakMap,x=[],g=null,p=!1,_=null,v=null,S=null,M=null,y=null,w=null,T=null,E=new Kt(0,0,0),A=0,D=!1,b=null,F=null,L=null,N=null,U=null,st.set(0,0,r.canvas.width,r.canvas.height),at.set(0,0,r.canvas.width,r.canvas.height),o.reset(),c.reset(),l.reset()}return{buffers:{color:o,depth:c,stencil:l},enable:ht,disable:ut,bindFramebuffer:bt,drawBuffers:Y,useProgram:Nt,setBlending:vt,setMaterial:Jt,setFlipSided:Lt,setCullFace:I,setLineWidth:R,setPolygonOffset:W,setScissorTest:Q,activeTexture:K,bindTexture:tt,unbindTexture:Mt,compressedTexImage2D:rt,compressedTexImage3D:gt,texImage2D:At,texImage3D:xt,updateUBOMapping:fe,uniformBlockBinding:qt,texStorage2D:jt,texStorage3D:kt,texSubImage2D:Pt,texSubImage3D:Wt,compressedTexSubImage2D:et,compressedTexSubImage3D:oe,scissor:Ft,viewport:Qt,reset:ot}}function hv(r,t,e,n,i,s,a){const o=i.isWebGL2,c=t.has("WEBGL_multisampled_render_to_texture")?t.get("WEBGL_multisampled_render_to_texture"):null,l=typeof navigator>"u"?!1:/OculusBrowser/g.test(navigator.userAgent),h=new WeakMap;let u;const d=new WeakMap;let f=!1;try{f=typeof OffscreenCanvas<"u"&&new OffscreenCanvas(1,1).getContext("2d")!==null}catch{}function m(I,R){return f?new OffscreenCanvas(I,R):qo("canvas")}function x(I,R,W,Q){let K=1;if((I.width>Q||I.height>Q)&&(K=Q/Math.max(I.width,I.height)),K<1||R===!0)if(typeof HTMLImageElement<"u"&&I instanceof HTMLImageElement||typeof HTMLCanvasElement<"u"&&I instanceof HTMLCanvasElement||typeof ImageBitmap<"u"&&I instanceof ImageBitmap){const tt=R?Yo:Math.floor,Mt=tt(K*I.width),rt=tt(K*I.height);u===void 0&&(u=m(Mt,rt));const gt=W?m(Mt,rt):u;return gt.width=Mt,gt.height=rt,gt.getContext("2d").drawImage(I,0,0,Mt,rt),console.warn("THREE.WebGLRenderer: Texture has been resized from ("+I.width+"x"+I.height+") to ("+Mt+"x"+rt+")."),gt}else return"data"in I&&console.warn("THREE.WebGLRenderer: Image in DataTexture is too big ("+I.width+"x"+I.height+")."),I;return I}function g(I){return Vl(I.width)&&Vl(I.height)}function p(I){return o?!1:I.wrapS!==1001||I.wrapT!==1001||I.minFilter!==1003&&I.minFilter!==1006}function _(I,R){return I.generateMipmaps&&R&&I.minFilter!==1003&&I.minFilter!==1006}function v(I){r.generateMipmap(I)}function S(I,R,W,Q,K=!1){if(o===!1)return R;if(I!==null){if(r[I]!==void 0)return r[I];console.warn("THREE.WebGLRenderer: Attempt to use non-existing WebGL internal format '"+I+"'")}let tt=R;if(R===r.RED&&(W===r.FLOAT&&(tt=r.R32F),W===r.HALF_FLOAT&&(tt=r.R16F),W===r.UNSIGNED_BYTE&&(tt=r.R8)),R===r.RED_INTEGER&&(W===r.UNSIGNED_BYTE&&(tt=r.R8UI),W===r.UNSIGNED_SHORT&&(tt=r.R16UI),W===r.UNSIGNED_INT&&(tt=r.R32UI),W===r.BYTE&&(tt=r.R8I),W===r.SHORT&&(tt=r.R16I),W===r.INT&&(tt=r.R32I)),R===r.RG&&(W===r.FLOAT&&(tt=r.RG32F),W===r.HALF_FLOAT&&(tt=r.RG16F),W===r.UNSIGNED_BYTE&&(tt=r.RG8)),R===r.RGBA){const Mt=K?Wo:ge.getTransfer(Q);W===r.FLOAT&&(tt=r.RGBA32F),W===r.HALF_FLOAT&&(tt=r.RGBA16F),W===r.UNSIGNED_BYTE&&(tt=Mt===ye?r.SRGB8_ALPHA8:r.RGBA8),W===r.UNSIGNED_SHORT_4_4_4_4&&(tt=r.RGBA4),W===r.UNSIGNED_SHORT_5_5_5_1&&(tt=r.RGB5_A1)}return(tt===r.R16F||tt===r.R32F||tt===r.RG16F||tt===r.RG32F||tt===r.RGBA16F||tt===r.RGBA32F)&&t.get("EXT_color_buffer_float"),tt}function M(I,R,W){return _(I,W)===!0||I.isFramebufferTexture&&I.minFilter!==1003&&I.minFilter!==1006?Math.log2(Math.max(R.width,R.height))+1:I.mipmaps!==void 0&&I.mipmaps.length>0?I.mipmaps.length:I.isCompressedTexture&&Array.isArray(I.image)?R.mipmaps.length:1}function y(I){return I===1003||I===1004||I===1005?r.NEAREST:r.LINEAR}function w(I){const R=I.target;R.removeEventListener("dispose",w),E(R),R.isVideoTexture&&h.delete(R)}function T(I){const R=I.target;R.removeEventListener("dispose",T),D(R)}function E(I){const R=n.get(I);if(R.__webglInit===void 0)return;const W=I.source,Q=d.get(W);if(Q){const K=Q[R.__cacheKey];K.usedTimes--,K.usedTimes===0&&A(I),Object.keys(Q).length===0&&d.delete(W)}n.remove(I)}function A(I){const R=n.get(I);r.deleteTexture(R.__webglTexture);const W=I.source,Q=d.get(W);delete Q[R.__cacheKey],a.memory.textures--}function D(I){const R=I.texture,W=n.get(I),Q=n.get(R);if(Q.__webglTexture!==void 0&&(r.deleteTexture(Q.__webglTexture),a.memory.textures--),I.depthTexture&&I.depthTexture.dispose(),I.isWebGLCubeRenderTarget)for(let K=0;K<6;K++){if(Array.isArray(W.__webglFramebuffer[K]))for(let tt=0;tt<W.__webglFramebuffer[K].length;tt++)r.deleteFramebuffer(W.__webglFramebuffer[K][tt]);else r.deleteFramebuffer(W.__webglFramebuffer[K]);W.__webglDepthbuffer&&r.deleteRenderbuffer(W.__webglDepthbuffer[K])}else{if(Array.isArray(W.__webglFramebuffer))for(let K=0;K<W.__webglFramebuffer.length;K++)r.deleteFramebuffer(W.__webglFramebuffer[K]);else r.deleteFramebuffer(W.__webglFramebuffer);if(W.__webglDepthbuffer&&r.deleteRenderbuffer(W.__webglDepthbuffer),W.__webglMultisampledFramebuffer&&r.deleteFramebuffer(W.__webglMultisampledFramebuffer),W.__webglColorRenderbuffer)for(let K=0;K<W.__webglColorRenderbuffer.length;K++)W.__webglColorRenderbuffer[K]&&r.deleteRenderbuffer(W.__webglColorRenderbuffer[K]);W.__webglDepthRenderbuffer&&r.deleteRenderbuffer(W.__webglDepthRenderbuffer)}if(I.isWebGLMultipleRenderTargets)for(let K=0,tt=R.length;K<tt;K++){const Mt=n.get(R[K]);Mt.__webglTexture&&(r.deleteTexture(Mt.__webglTexture),a.memory.textures--),n.remove(R[K])}n.remove(R),n.remove(I)}let b=0;function F(){b=0}function L(){const I=b;return I>=i.maxTextures&&console.warn("THREE.WebGLTextures: Trying to use "+I+" texture units while this GPU supports only "+i.maxTextures),b+=1,I}function N(I){const R=[];return R.push(I.wrapS),R.push(I.wrapT),R.push(I.wrapR||0),R.push(I.magFilter),R.push(I.minFilter),R.push(I.anisotropy),R.push(I.internalFormat),R.push(I.format),R.push(I.type),R.push(I.generateMipmaps),R.push(I.premultiplyAlpha),R.push(I.flipY),R.push(I.unpackAlignment),R.push(I.colorSpace),R.join()}function U(I,R){const W=n.get(I);if(I.isVideoTexture&&Jt(I),I.isRenderTargetTexture===!1&&I.version>0&&W.__version!==I.version){const Q=I.image;if(Q===null)console.warn("THREE.WebGLRenderer: Texture marked for update but no image data found.");else if(Q.complete===!1)console.warn("THREE.WebGLRenderer: Texture marked for update but image is incomplete");else{st(W,I,R);return}}e.bindTexture(r.TEXTURE_2D,W.__webglTexture,r.TEXTURE0+R)}function k(I,R){const W=n.get(I);if(I.version>0&&W.__version!==I.version){st(W,I,R);return}e.bindTexture(r.TEXTURE_2D_ARRAY,W.__webglTexture,r.TEXTURE0+R)}function O(I,R){const W=n.get(I);if(I.version>0&&W.__version!==I.version){st(W,I,R);return}e.bindTexture(r.TEXTURE_3D,W.__webglTexture,r.TEXTURE0+R)}function H(I,R){const W=n.get(I);if(I.version>0&&W.__version!==I.version){at(W,I,R);return}e.bindTexture(r.TEXTURE_CUBE_MAP,W.__webglTexture,r.TEXTURE0+R)}const j={1e3:r.REPEAT,1001:r.CLAMP_TO_EDGE,1002:r.MIRRORED_REPEAT},J={1003:r.NEAREST,1004:r.NEAREST_MIPMAP_NEAREST,1005:r.NEAREST_MIPMAP_LINEAR,1006:r.LINEAR,1007:r.LINEAR_MIPMAP_NEAREST,1008:r.LINEAR_MIPMAP_LINEAR},nt={512:r.NEVER,519:r.ALWAYS,513:r.LESS,515:r.LEQUAL,514:r.EQUAL,518:r.GEQUAL,516:r.GREATER,517:r.NOTEQUAL};function V(I,R,W){if(W?(r.texParameteri(I,r.TEXTURE_WRAP_S,j[R.wrapS]),r.texParameteri(I,r.TEXTURE_WRAP_T,j[R.wrapT]),(I===r.TEXTURE_3D||I===r.TEXTURE_2D_ARRAY)&&r.texParameteri(I,r.TEXTURE_WRAP_R,j[R.wrapR]),r.texParameteri(I,r.TEXTURE_MAG_FILTER,J[R.magFilter]),r.texParameteri(I,r.TEXTURE_MIN_FILTER,J[R.minFilter])):(r.texParameteri(I,r.TEXTURE_WRAP_S,r.CLAMP_TO_EDGE),r.texParameteri(I,r.TEXTURE_WRAP_T,r.CLAMP_TO_EDGE),(I===r.TEXTURE_3D||I===r.TEXTURE_2D_ARRAY)&&r.texParameteri(I,r.TEXTURE_WRAP_R,r.CLAMP_TO_EDGE),(R.wrapS!==1001||R.wrapT!==1001)&&console.warn("THREE.WebGLRenderer: Texture is not power of two. Texture.wrapS and Texture.wrapT should be set to THREE.ClampToEdgeWrapping."),r.texParameteri(I,r.TEXTURE_MAG_FILTER,y(R.magFilter)),r.texParameteri(I,r.TEXTURE_MIN_FILTER,y(R.minFilter)),R.minFilter!==1003&&R.minFilter!==1006&&console.warn("THREE.WebGLRenderer: Texture is not power of two. Texture.minFilter should be set to THREE.NearestFilter or THREE.LinearFilter.")),R.compareFunction&&(r.texParameteri(I,r.TEXTURE_COMPARE_MODE,r.COMPARE_REF_TO_TEXTURE),r.texParameteri(I,r.TEXTURE_COMPARE_FUNC,nt[R.compareFunction])),t.has("EXT_texture_filter_anisotropic")===!0){const Q=t.get("EXT_texture_filter_anisotropic");if(R.magFilter===1003||R.minFilter!==1005&&R.minFilter!==1008||R.type===1015&&t.has("OES_texture_float_linear")===!1||o===!1&&R.type===1016&&t.has("OES_texture_half_float_linear")===!1)return;(R.anisotropy>1||n.get(R).__currentAnisotropy)&&(r.texParameterf(I,Q.TEXTURE_MAX_ANISOTROPY_EXT,Math.min(R.anisotropy,i.getMaxAnisotropy())),n.get(R).__currentAnisotropy=R.anisotropy)}}function $(I,R){let W=!1;I.__webglInit===void 0&&(I.__webglInit=!0,R.addEventListener("dispose",w));const Q=R.source;let K=d.get(Q);K===void 0&&(K={},d.set(Q,K));const tt=N(R);if(tt!==I.__cacheKey){K[tt]===void 0&&(K[tt]={texture:r.createTexture(),usedTimes:0},a.memory.textures++,W=!0),K[tt].usedTimes++;const Mt=K[I.__cacheKey];Mt!==void 0&&(K[I.__cacheKey].usedTimes--,Mt.usedTimes===0&&A(R)),I.__cacheKey=tt,I.__webglTexture=K[tt].texture}return W}function st(I,R,W){let Q=r.TEXTURE_2D;(R.isDataArrayTexture||R.isCompressedArrayTexture)&&(Q=r.TEXTURE_2D_ARRAY),R.isData3DTexture&&(Q=r.TEXTURE_3D);const K=$(I,R),tt=R.source;e.bindTexture(Q,I.__webglTexture,r.TEXTURE0+W);const Mt=n.get(tt);if(tt.version!==Mt.__version||K===!0){e.activeTexture(r.TEXTURE0+W);const rt=ge.getPrimaries(ge.workingColorSpace),gt=R.colorSpace===""?null:ge.getPrimaries(R.colorSpace),Pt=R.colorSpace===""||rt===gt?r.NONE:r.BROWSER_DEFAULT_WEBGL;r.pixelStorei(r.UNPACK_FLIP_Y_WEBGL,R.flipY),r.pixelStorei(r.UNPACK_PREMULTIPLY_ALPHA_WEBGL,R.premultiplyAlpha),r.pixelStorei(r.UNPACK_ALIGNMENT,R.unpackAlignment),r.pixelStorei(r.UNPACK_COLORSPACE_CONVERSION_WEBGL,Pt);const Wt=p(R)&&g(R.image)===!1;let et=x(R.image,Wt,!1,i.maxTextureSize);et=Lt(R,et);const oe=g(et)||o,jt=s.convert(R.format,R.colorSpace);let kt=s.convert(R.type),At=S(R.internalFormat,jt,kt,R.colorSpace,R.isVideoTexture);V(Q,R,oe);let xt;const Ft=R.mipmaps,Qt=o&&R.isVideoTexture!==!0&&At!==36196,fe=Mt.__version===void 0||K===!0,qt=M(R,et,oe);if(R.isDepthTexture)At=r.DEPTH_COMPONENT,o?R.type===1015?At=r.DEPTH_COMPONENT32F:R.type===1014?At=r.DEPTH_COMPONENT24:R.type===1020?At=r.DEPTH24_STENCIL8:At=r.DEPTH_COMPONENT16:R.type===1015&&console.error("WebGLRenderer: Floating point depth texture requires WebGL2."),R.format===1026&&At===r.DEPTH_COMPONENT&&R.type!==1012&&R.type!==1014&&(console.warn("THREE.WebGLRenderer: Use UnsignedShortType or UnsignedIntType for DepthFormat DepthTexture."),R.type=1014,kt=s.convert(R.type)),R.format===1027&&At===r.DEPTH_COMPONENT&&(At=r.DEPTH_STENCIL,R.type!==1020&&(console.warn("THREE.WebGLRenderer: Use UnsignedInt248Type for DepthStencilFormat DepthTexture."),R.type=1020,kt=s.convert(R.type))),fe&&(Qt?e.texStorage2D(r.TEXTURE_2D,1,At,et.width,et.height):e.texImage2D(r.TEXTURE_2D,0,At,et.width,et.height,0,jt,kt,null));else if(R.isDataTexture)if(Ft.length>0&&oe){Qt&&fe&&e.texStorage2D(r.TEXTURE_2D,qt,At,Ft[0].width,Ft[0].height);for(let ot=0,B=Ft.length;ot<B;ot++)xt=Ft[ot],Qt?e.texSubImage2D(r.TEXTURE_2D,ot,0,0,xt.width,xt.height,jt,kt,xt.data):e.texImage2D(r.TEXTURE_2D,ot,At,xt.width,xt.height,0,jt,kt,xt.data);R.generateMipmaps=!1}else Qt?(fe&&e.texStorage2D(r.TEXTURE_2D,qt,At,et.width,et.height),e.texSubImage2D(r.TEXTURE_2D,0,0,0,et.width,et.height,jt,kt,et.data)):e.texImage2D(r.TEXTURE_2D,0,At,et.width,et.height,0,jt,kt,et.data);else if(R.isCompressedTexture)if(R.isCompressedArrayTexture){Qt&&fe&&e.texStorage3D(r.TEXTURE_2D_ARRAY,qt,At,Ft[0].width,Ft[0].height,et.depth);for(let ot=0,B=Ft.length;ot<B;ot++)xt=Ft[ot],R.format!==1023?jt!==null?Qt?e.compressedTexSubImage3D(r.TEXTURE_2D_ARRAY,ot,0,0,0,xt.width,xt.height,et.depth,jt,xt.data,0,0):e.compressedTexImage3D(r.TEXTURE_2D_ARRAY,ot,At,xt.width,xt.height,et.depth,0,xt.data,0,0):console.warn("THREE.WebGLRenderer: Attempt to load unsupported compressed texture format in .uploadTexture()"):Qt?e.texSubImage3D(r.TEXTURE_2D_ARRAY,ot,0,0,0,xt.width,xt.height,et.depth,jt,kt,xt.data):e.texImage3D(r.TEXTURE_2D_ARRAY,ot,At,xt.width,xt.height,et.depth,0,jt,kt,xt.data)}else{Qt&&fe&&e.texStorage2D(r.TEXTURE_2D,qt,At,Ft[0].width,Ft[0].height);for(let ot=0,B=Ft.length;ot<B;ot++)xt=Ft[ot],R.format!==1023?jt!==null?Qt?e.compressedTexSubImage2D(r.TEXTURE_2D,ot,0,0,xt.width,xt.height,jt,xt.data):e.compressedTexImage2D(r.TEXTURE_2D,ot,At,xt.width,xt.height,0,xt.data):console.warn("THREE.WebGLRenderer: Attempt to load unsupported compressed texture format in .uploadTexture()"):Qt?e.texSubImage2D(r.TEXTURE_2D,ot,0,0,xt.width,xt.height,jt,kt,xt.data):e.texImage2D(r.TEXTURE_2D,ot,At,xt.width,xt.height,0,jt,kt,xt.data)}else if(R.isDataArrayTexture)Qt?(fe&&e.texStorage3D(r.TEXTURE_2D_ARRAY,qt,At,et.width,et.height,et.depth),e.texSubImage3D(r.TEXTURE_2D_ARRAY,0,0,0,0,et.width,et.height,et.depth,jt,kt,et.data)):e.texImage3D(r.TEXTURE_2D_ARRAY,0,At,et.width,et.height,et.depth,0,jt,kt,et.data);else if(R.isData3DTexture)Qt?(fe&&e.texStorage3D(r.TEXTURE_3D,qt,At,et.width,et.height,et.depth),e.texSubImage3D(r.TEXTURE_3D,0,0,0,0,et.width,et.height,et.depth,jt,kt,et.data)):e.texImage3D(r.TEXTURE_3D,0,At,et.width,et.height,et.depth,0,jt,kt,et.data);else if(R.isFramebufferTexture){if(fe)if(Qt)e.texStorage2D(r.TEXTURE_2D,qt,At,et.width,et.height);else{let ot=et.width,B=et.height;for(let dt=0;dt<qt;dt++)e.texImage2D(r.TEXTURE_2D,dt,At,ot,B,0,jt,kt,null),ot>>=1,B>>=1}}else if(Ft.length>0&&oe){Qt&&fe&&e.texStorage2D(r.TEXTURE_2D,qt,At,Ft[0].width,Ft[0].height);for(let ot=0,B=Ft.length;ot<B;ot++)xt=Ft[ot],Qt?e.texSubImage2D(r.TEXTURE_2D,ot,0,0,jt,kt,xt):e.texImage2D(r.TEXTURE_2D,ot,At,jt,kt,xt);R.generateMipmaps=!1}else Qt?(fe&&e.texStorage2D(r.TEXTURE_2D,qt,At,et.width,et.height),e.texSubImage2D(r.TEXTURE_2D,0,0,0,jt,kt,et)):e.texImage2D(r.TEXTURE_2D,0,At,jt,kt,et);_(R,oe)&&v(Q),Mt.__version=tt.version,R.onUpdate&&R.onUpdate(R)}I.__version=R.version}function at(I,R,W){if(R.image.length!==6)return;const Q=$(I,R),K=R.source;e.bindTexture(r.TEXTURE_CUBE_MAP,I.__webglTexture,r.TEXTURE0+W);const tt=n.get(K);if(K.version!==tt.__version||Q===!0){e.activeTexture(r.TEXTURE0+W);const Mt=ge.getPrimaries(ge.workingColorSpace),rt=R.colorSpace===""?null:ge.getPrimaries(R.colorSpace),gt=R.colorSpace===""||Mt===rt?r.NONE:r.BROWSER_DEFAULT_WEBGL;r.pixelStorei(r.UNPACK_FLIP_Y_WEBGL,R.flipY),r.pixelStorei(r.UNPACK_PREMULTIPLY_ALPHA_WEBGL,R.premultiplyAlpha),r.pixelStorei(r.UNPACK_ALIGNMENT,R.unpackAlignment),r.pixelStorei(r.UNPACK_COLORSPACE_CONVERSION_WEBGL,gt);const Pt=R.isCompressedTexture||R.image[0].isCompressedTexture,Wt=R.image[0]&&R.image[0].isDataTexture,et=[];for(let ot=0;ot<6;ot++)!Pt&&!Wt?et[ot]=x(R.image[ot],!1,!0,i.maxCubemapSize):et[ot]=Wt?R.image[ot].image:R.image[ot],et[ot]=Lt(R,et[ot]);const oe=et[0],jt=g(oe)||o,kt=s.convert(R.format,R.colorSpace),At=s.convert(R.type),xt=S(R.internalFormat,kt,At,R.colorSpace),Ft=o&&R.isVideoTexture!==!0,Qt=tt.__version===void 0||Q===!0;let fe=M(R,oe,jt);V(r.TEXTURE_CUBE_MAP,R,jt);let qt;if(Pt){Ft&&Qt&&e.texStorage2D(r.TEXTURE_CUBE_MAP,fe,xt,oe.width,oe.height);for(let ot=0;ot<6;ot++){qt=et[ot].mipmaps;for(let B=0;B<qt.length;B++){const dt=qt[B];R.format!==1023?kt!==null?Ft?e.compressedTexSubImage2D(r.TEXTURE_CUBE_MAP_POSITIVE_X+ot,B,0,0,dt.width,dt.height,kt,dt.data):e.compressedTexImage2D(r.TEXTURE_CUBE_MAP_POSITIVE_X+ot,B,xt,dt.width,dt.height,0,dt.data):console.warn("THREE.WebGLRenderer: Attempt to load unsupported compressed texture format in .setTextureCube()"):Ft?e.texSubImage2D(r.TEXTURE_CUBE_MAP_POSITIVE_X+ot,B,0,0,dt.width,dt.height,kt,At,dt.data):e.texImage2D(r.TEXTURE_CUBE_MAP_POSITIVE_X+ot,B,xt,dt.width,dt.height,0,kt,At,dt.data)}}}else{qt=R.mipmaps,Ft&&Qt&&(qt.length>0&&fe++,e.texStorage2D(r.TEXTURE_CUBE_MAP,fe,xt,et[0].width,et[0].height));for(let ot=0;ot<6;ot++)if(Wt){Ft?e.texSubImage2D(r.TEXTURE_CUBE_MAP_POSITIVE_X+ot,0,0,0,et[ot].width,et[ot].height,kt,At,et[ot].data):e.texImage2D(r.TEXTURE_CUBE_MAP_POSITIVE_X+ot,0,xt,et[ot].width,et[ot].height,0,kt,At,et[ot].data);for(let B=0;B<qt.length;B++){const ft=qt[B].image[ot].image;Ft?e.texSubImage2D(r.TEXTURE_CUBE_MAP_POSITIVE_X+ot,B+1,0,0,ft.width,ft.height,kt,At,ft.data):e.texImage2D(r.TEXTURE_CUBE_MAP_POSITIVE_X+ot,B+1,xt,ft.width,ft.height,0,kt,At,ft.data)}}else{Ft?e.texSubImage2D(r.TEXTURE_CUBE_MAP_POSITIVE_X+ot,0,0,0,kt,At,et[ot]):e.texImage2D(r.TEXTURE_CUBE_MAP_POSITIVE_X+ot,0,xt,kt,At,et[ot]);for(let B=0;B<qt.length;B++){const dt=qt[B];Ft?e.texSubImage2D(r.TEXTURE_CUBE_MAP_POSITIVE_X+ot,B+1,0,0,kt,At,dt.image[ot]):e.texImage2D(r.TEXTURE_CUBE_MAP_POSITIVE_X+ot,B+1,xt,kt,At,dt.image[ot])}}}_(R,jt)&&v(r.TEXTURE_CUBE_MAP),tt.__version=K.version,R.onUpdate&&R.onUpdate(R)}I.__version=R.version}function lt(I,R,W,Q,K,tt){const Mt=s.convert(W.format,W.colorSpace),rt=s.convert(W.type),gt=S(W.internalFormat,Mt,rt,W.colorSpace);if(!n.get(R).__hasExternalTextures){const Wt=Math.max(1,R.width>>tt),et=Math.max(1,R.height>>tt);K===r.TEXTURE_3D||K===r.TEXTURE_2D_ARRAY?e.texImage3D(K,tt,gt,Wt,et,R.depth,0,Mt,rt,null):e.texImage2D(K,tt,gt,Wt,et,0,Mt,rt,null)}e.bindFramebuffer(r.FRAMEBUFFER,I),vt(R)?c.framebufferTexture2DMultisampleEXT(r.FRAMEBUFFER,Q,K,n.get(W).__webglTexture,0,Ct(R)):(K===r.TEXTURE_2D||K>=r.TEXTURE_CUBE_MAP_POSITIVE_X&&K<=r.TEXTURE_CUBE_MAP_NEGATIVE_Z)&&r.framebufferTexture2D(r.FRAMEBUFFER,Q,K,n.get(W).__webglTexture,tt),e.bindFramebuffer(r.FRAMEBUFFER,null)}function _t(I,R,W){if(r.bindRenderbuffer(r.RENDERBUFFER,I),R.depthBuffer&&!R.stencilBuffer){let Q=o===!0?r.DEPTH_COMPONENT24:r.DEPTH_COMPONENT16;if(W||vt(R)){const K=R.depthTexture;K&&K.isDepthTexture&&(K.type===1015?Q=r.DEPTH_COMPONENT32F:K.type===1014&&(Q=r.DEPTH_COMPONENT24));const tt=Ct(R);vt(R)?c.renderbufferStorageMultisampleEXT(r.RENDERBUFFER,tt,Q,R.width,R.height):r.renderbufferStorageMultisample(r.RENDERBUFFER,tt,Q,R.width,R.height)}else r.renderbufferStorage(r.RENDERBUFFER,Q,R.width,R.height);r.framebufferRenderbuffer(r.FRAMEBUFFER,r.DEPTH_ATTACHMENT,r.RENDERBUFFER,I)}else if(R.depthBuffer&&R.stencilBuffer){const Q=Ct(R);W&&vt(R)===!1?r.renderbufferStorageMultisample(r.RENDERBUFFER,Q,r.DEPTH24_STENCIL8,R.width,R.height):vt(R)?c.renderbufferStorageMultisampleEXT(r.RENDERBUFFER,Q,r.DEPTH24_STENCIL8,R.width,R.height):r.renderbufferStorage(r.RENDERBUFFER,r.DEPTH_STENCIL,R.width,R.height),r.framebufferRenderbuffer(r.FRAMEBUFFER,r.DEPTH_STENCIL_ATTACHMENT,r.RENDERBUFFER,I)}else{const Q=R.isWebGLMultipleRenderTargets===!0?R.texture:[R.texture];for(let K=0;K<Q.length;K++){const tt=Q[K],Mt=s.convert(tt.format,tt.colorSpace),rt=s.convert(tt.type),gt=S(tt.internalFormat,Mt,rt,tt.colorSpace),Pt=Ct(R);W&&vt(R)===!1?r.renderbufferStorageMultisample(r.RENDERBUFFER,Pt,gt,R.width,R.height):vt(R)?c.renderbufferStorageMultisampleEXT(r.RENDERBUFFER,Pt,gt,R.width,R.height):r.renderbufferStorage(r.RENDERBUFFER,gt,R.width,R.height)}}r.bindRenderbuffer(r.RENDERBUFFER,null)}function ht(I,R){if(R&&R.isWebGLCubeRenderTarget)throw new Error("Depth Texture with cube render targets is not supported");if(e.bindFramebuffer(r.FRAMEBUFFER,I),!(R.depthTexture&&R.depthTexture.isDepthTexture))throw new Error("renderTarget.depthTexture must be an instance of THREE.DepthTexture");(!n.get(R.depthTexture).__webglTexture||R.depthTexture.image.width!==R.width||R.depthTexture.image.height!==R.height)&&(R.depthTexture.image.width=R.width,R.depthTexture.image.height=R.height,R.depthTexture.needsUpdate=!0),U(R.depthTexture,0);const Q=n.get(R.depthTexture).__webglTexture,K=Ct(R);if(R.depthTexture.format===1026)vt(R)?c.framebufferTexture2DMultisampleEXT(r.FRAMEBUFFER,r.DEPTH_ATTACHMENT,r.TEXTURE_2D,Q,0,K):r.framebufferTexture2D(r.FRAMEBUFFER,r.DEPTH_ATTACHMENT,r.TEXTURE_2D,Q,0);else if(R.depthTexture.format===1027)vt(R)?c.framebufferTexture2DMultisampleEXT(r.FRAMEBUFFER,r.DEPTH_STENCIL_ATTACHMENT,r.TEXTURE_2D,Q,0,K):r.framebufferTexture2D(r.FRAMEBUFFER,r.DEPTH_STENCIL_ATTACHMENT,r.TEXTURE_2D,Q,0);else throw new Error("Unknown depthTexture format")}function ut(I){const R=n.get(I),W=I.isWebGLCubeRenderTarget===!0;if(I.depthTexture&&!R.__autoAllocateDepthBuffer){if(W)throw new Error("target.depthTexture not supported in Cube render targets");ht(R.__webglFramebuffer,I)}else if(W){R.__webglDepthbuffer=[];for(let Q=0;Q<6;Q++)e.bindFramebuffer(r.FRAMEBUFFER,R.__webglFramebuffer[Q]),R.__webglDepthbuffer[Q]=r.createRenderbuffer(),_t(R.__webglDepthbuffer[Q],I,!1)}else e.bindFramebuffer(r.FRAMEBUFFER,R.__webglFramebuffer),R.__webglDepthbuffer=r.createRenderbuffer(),_t(R.__webglDepthbuffer,I,!1);e.bindFramebuffer(r.FRAMEBUFFER,null)}function bt(I,R,W){const Q=n.get(I);R!==void 0&&lt(Q.__webglFramebuffer,I,I.texture,r.COLOR_ATTACHMENT0,r.TEXTURE_2D,0),W!==void 0&&ut(I)}function Y(I){const R=I.texture,W=n.get(I),Q=n.get(R);I.addEventListener("dispose",T),I.isWebGLMultipleRenderTargets!==!0&&(Q.__webglTexture===void 0&&(Q.__webglTexture=r.createTexture()),Q.__version=R.version,a.memory.textures++);const K=I.isWebGLCubeRenderTarget===!0,tt=I.isWebGLMultipleRenderTargets===!0,Mt=g(I)||o;if(K){W.__webglFramebuffer=[];for(let rt=0;rt<6;rt++)if(o&&R.mipmaps&&R.mipmaps.length>0){W.__webglFramebuffer[rt]=[];for(let gt=0;gt<R.mipmaps.length;gt++)W.__webglFramebuffer[rt][gt]=r.createFramebuffer()}else W.__webglFramebuffer[rt]=r.createFramebuffer()}else{if(o&&R.mipmaps&&R.mipmaps.length>0){W.__webglFramebuffer=[];for(let rt=0;rt<R.mipmaps.length;rt++)W.__webglFramebuffer[rt]=r.createFramebuffer()}else W.__webglFramebuffer=r.createFramebuffer();if(tt)if(i.drawBuffers){const rt=I.texture;for(let gt=0,Pt=rt.length;gt<Pt;gt++){const Wt=n.get(rt[gt]);Wt.__webglTexture===void 0&&(Wt.__webglTexture=r.createTexture(),a.memory.textures++)}}else console.warn("THREE.WebGLRenderer: WebGLMultipleRenderTargets can only be used with WebGL2 or WEBGL_draw_buffers extension.");if(o&&I.samples>0&&vt(I)===!1){const rt=tt?R:[R];W.__webglMultisampledFramebuffer=r.createFramebuffer(),W.__webglColorRenderbuffer=[],e.bindFramebuffer(r.FRAMEBUFFER,W.__webglMultisampledFramebuffer);for(let gt=0;gt<rt.length;gt++){const Pt=rt[gt];W.__webglColorRenderbuffer[gt]=r.createRenderbuffer(),r.bindRenderbuffer(r.RENDERBUFFER,W.__webglColorRenderbuffer[gt]);const Wt=s.convert(Pt.format,Pt.colorSpace),et=s.convert(Pt.type),oe=S(Pt.internalFormat,Wt,et,Pt.colorSpace,I.isXRRenderTarget===!0),jt=Ct(I);r.renderbufferStorageMultisample(r.RENDERBUFFER,jt,oe,I.width,I.height),r.framebufferRenderbuffer(r.FRAMEBUFFER,r.COLOR_ATTACHMENT0+gt,r.RENDERBUFFER,W.__webglColorRenderbuffer[gt])}r.bindRenderbuffer(r.RENDERBUFFER,null),I.depthBuffer&&(W.__webglDepthRenderbuffer=r.createRenderbuffer(),_t(W.__webglDepthRenderbuffer,I,!0)),e.bindFramebuffer(r.FRAMEBUFFER,null)}}if(K){e.bindTexture(r.TEXTURE_CUBE_MAP,Q.__webglTexture),V(r.TEXTURE_CUBE_MAP,R,Mt);for(let rt=0;rt<6;rt++)if(o&&R.mipmaps&&R.mipmaps.length>0)for(let gt=0;gt<R.mipmaps.length;gt++)lt(W.__webglFramebuffer[rt][gt],I,R,r.COLOR_ATTACHMENT0,r.TEXTURE_CUBE_MAP_POSITIVE_X+rt,gt);else lt(W.__webglFramebuffer[rt],I,R,r.COLOR_ATTACHMENT0,r.TEXTURE_CUBE_MAP_POSITIVE_X+rt,0);_(R,Mt)&&v(r.TEXTURE_CUBE_MAP),e.unbindTexture()}else if(tt){const rt=I.texture;for(let gt=0,Pt=rt.length;gt<Pt;gt++){const Wt=rt[gt],et=n.get(Wt);e.bindTexture(r.TEXTURE_2D,et.__webglTexture),V(r.TEXTURE_2D,Wt,Mt),lt(W.__webglFramebuffer,I,Wt,r.COLOR_ATTACHMENT0+gt,r.TEXTURE_2D,0),_(Wt,Mt)&&v(r.TEXTURE_2D)}e.unbindTexture()}else{let rt=r.TEXTURE_2D;if((I.isWebGL3DRenderTarget||I.isWebGLArrayRenderTarget)&&(o?rt=I.isWebGL3DRenderTarget?r.TEXTURE_3D:r.TEXTURE_2D_ARRAY:console.error("THREE.WebGLTextures: THREE.Data3DTexture and THREE.DataArrayTexture only supported with WebGL2.")),e.bindTexture(rt,Q.__webglTexture),V(rt,R,Mt),o&&R.mipmaps&&R.mipmaps.length>0)for(let gt=0;gt<R.mipmaps.length;gt++)lt(W.__webglFramebuffer[gt],I,R,r.COLOR_ATTACHMENT0,rt,gt);else lt(W.__webglFramebuffer,I,R,r.COLOR_ATTACHMENT0,rt,0);_(R,Mt)&&v(rt),e.unbindTexture()}I.depthBuffer&&ut(I)}function Nt(I){const R=g(I)||o,W=I.isWebGLMultipleRenderTargets===!0?I.texture:[I.texture];for(let Q=0,K=W.length;Q<K;Q++){const tt=W[Q];if(_(tt,R)){const Mt=I.isWebGLCubeRenderTarget?r.TEXTURE_CUBE_MAP:r.TEXTURE_2D,rt=n.get(tt).__webglTexture;e.bindTexture(Mt,rt),v(Mt),e.unbindTexture()}}}function wt(I){if(o&&I.samples>0&&vt(I)===!1){const R=I.isWebGLMultipleRenderTargets?I.texture:[I.texture],W=I.width,Q=I.height;let K=r.COLOR_BUFFER_BIT;const tt=[],Mt=I.stencilBuffer?r.DEPTH_STENCIL_ATTACHMENT:r.DEPTH_ATTACHMENT,rt=n.get(I),gt=I.isWebGLMultipleRenderTargets===!0;if(gt)for(let Pt=0;Pt<R.length;Pt++)e.bindFramebuffer(r.FRAMEBUFFER,rt.__webglMultisampledFramebuffer),r.framebufferRenderbuffer(r.FRAMEBUFFER,r.COLOR_ATTACHMENT0+Pt,r.RENDERBUFFER,null),e.bindFramebuffer(r.FRAMEBUFFER,rt.__webglFramebuffer),r.framebufferTexture2D(r.DRAW_FRAMEBUFFER,r.COLOR_ATTACHMENT0+Pt,r.TEXTURE_2D,null,0);e.bindFramebuffer(r.READ_FRAMEBUFFER,rt.__webglMultisampledFramebuffer),e.bindFramebuffer(r.DRAW_FRAMEBUFFER,rt.__webglFramebuffer);for(let Pt=0;Pt<R.length;Pt++){tt.push(r.COLOR_ATTACHMENT0+Pt),I.depthBuffer&&tt.push(Mt);const Wt=rt.__ignoreDepthValues!==void 0?rt.__ignoreDepthValues:!1;if(Wt===!1&&(I.depthBuffer&&(K|=r.DEPTH_BUFFER_BIT),I.stencilBuffer&&(K|=r.STENCIL_BUFFER_BIT)),gt&&r.framebufferRenderbuffer(r.READ_FRAMEBUFFER,r.COLOR_ATTACHMENT0,r.RENDERBUFFER,rt.__webglColorRenderbuffer[Pt]),Wt===!0&&(r.invalidateFramebuffer(r.READ_FRAMEBUFFER,[Mt]),r.invalidateFramebuffer(r.DRAW_FRAMEBUFFER,[Mt])),gt){const et=n.get(R[Pt]).__webglTexture;r.framebufferTexture2D(r.DRAW_FRAMEBUFFER,r.COLOR_ATTACHMENT0,r.TEXTURE_2D,et,0)}r.blitFramebuffer(0,0,W,Q,0,0,W,Q,K,r.NEAREST),l&&r.invalidateFramebuffer(r.READ_FRAMEBUFFER,tt)}if(e.bindFramebuffer(r.READ_FRAMEBUFFER,null),e.bindFramebuffer(r.DRAW_FRAMEBUFFER,null),gt)for(let Pt=0;Pt<R.length;Pt++){e.bindFramebuffer(r.FRAMEBUFFER,rt.__webglMultisampledFramebuffer),r.framebufferRenderbuffer(r.FRAMEBUFFER,r.COLOR_ATTACHMENT0+Pt,r.RENDERBUFFER,rt.__webglColorRenderbuffer[Pt]);const Wt=n.get(R[Pt]).__webglTexture;e.bindFramebuffer(r.FRAMEBUFFER,rt.__webglFramebuffer),r.framebufferTexture2D(r.DRAW_FRAMEBUFFER,r.COLOR_ATTACHMENT0+Pt,r.TEXTURE_2D,Wt,0)}e.bindFramebuffer(r.DRAW_FRAMEBUFFER,rt.__webglMultisampledFramebuffer)}}function Ct(I){return Math.min(i.maxSamples,I.samples)}function vt(I){const R=n.get(I);return o&&I.samples>0&&t.has("WEBGL_multisampled_render_to_texture")===!0&&R.__useRenderToTexture!==!1}function Jt(I){const R=a.render.frame;h.get(I)!==R&&(h.set(I,R),I.update())}function Lt(I,R){const W=I.colorSpace,Q=I.format,K=I.type;return I.isCompressedTexture===!0||I.isVideoTexture===!0||I.format===1035||W!==Ii&&W!==""&&(ge.getTransfer(W)===ye?o===!1?t.has("EXT_sRGB")===!0&&Q===1023?(I.format=1035,I.minFilter=1006,I.generateMipmaps=!1):R=Cf.sRGBToLinear(R):(Q!==1023||K!==1009)&&console.warn("THREE.WebGLTextures: sRGB encoded textures have to use RGBAFormat and UnsignedByteType."):console.error("THREE.WebGLTextures: Unsupported texture color space:",W)),R}this.allocateTextureUnit=L,this.resetTextureUnits=F,this.setTexture2D=U,this.setTexture2DArray=k,this.setTexture3D=O,this.setTextureCube=H,this.rebindTextures=bt,this.setupRenderTarget=Y,this.updateRenderTargetMipmap=Nt,this.updateMultisampleRenderTarget=wt,this.setupDepthRenderbuffer=ut,this.setupFrameBufferTexture=lt,this.useMultisampledRTT=vt}function uv(r,t,e){const n=e.isWebGL2;function i(s,a=""){let o;const c=ge.getTransfer(a);if(s===1009)return r.UNSIGNED_BYTE;if(s===1017)return r.UNSIGNED_SHORT_4_4_4_4;if(s===1018)return r.UNSIGNED_SHORT_5_5_5_1;if(s===1010)return r.BYTE;if(s===1011)return r.SHORT;if(s===1012)return r.UNSIGNED_SHORT;if(s===1013)return r.INT;if(s===1014)return r.UNSIGNED_INT;if(s===1015)return r.FLOAT;if(s===1016)return n?r.HALF_FLOAT:(o=t.get("OES_texture_half_float"),o!==null?o.HALF_FLOAT_OES:null);if(s===1021)return r.ALPHA;if(s===1023)return r.RGBA;if(s===1024)return r.LUMINANCE;if(s===1025)return r.LUMINANCE_ALPHA;if(s===1026)return r.DEPTH_COMPONENT;if(s===1027)return r.DEPTH_STENCIL;if(s===1035)return o=t.get("EXT_sRGB"),o!==null?o.SRGB_ALPHA_EXT:null;if(s===1028)return r.RED;if(s===1029)return r.RED_INTEGER;if(s===1030)return r.RG;if(s===1031)return r.RG_INTEGER;if(s===1033)return r.RGBA_INTEGER;if(s===33776||s===33777||s===33778||s===33779)if(c===ye)if(o=t.get("WEBGL_compressed_texture_s3tc_srgb"),o!==null){if(s===33776)return o.COMPRESSED_SRGB_S3TC_DXT1_EXT;if(s===33777)return o.COMPRESSED_SRGB_ALPHA_S3TC_DXT1_EXT;if(s===33778)return o.COMPRESSED_SRGB_ALPHA_S3TC_DXT3_EXT;if(s===33779)return o.COMPRESSED_SRGB_ALPHA_S3TC_DXT5_EXT}else return null;else if(o=t.get("WEBGL_compressed_texture_s3tc"),o!==null){if(s===33776)return o.COMPRESSED_RGB_S3TC_DXT1_EXT;if(s===33777)return o.COMPRESSED_RGBA_S3TC_DXT1_EXT;if(s===33778)return o.COMPRESSED_RGBA_S3TC_DXT3_EXT;if(s===33779)return o.COMPRESSED_RGBA_S3TC_DXT5_EXT}else return null;if(s===35840||s===35841||s===35842||s===35843)if(o=t.get("WEBGL_compressed_texture_pvrtc"),o!==null){if(s===35840)return o.COMPRESSED_RGB_PVRTC_4BPPV1_IMG;if(s===35841)return o.COMPRESSED_RGB_PVRTC_2BPPV1_IMG;if(s===35842)return o.COMPRESSED_RGBA_PVRTC_4BPPV1_IMG;if(s===35843)return o.COMPRESSED_RGBA_PVRTC_2BPPV1_IMG}else return null;if(s===36196)return o=t.get("WEBGL_compressed_texture_etc1"),o!==null?o.COMPRESSED_RGB_ETC1_WEBGL:null;if(s===37492||s===37496)if(o=t.get("WEBGL_compressed_texture_etc"),o!==null){if(s===37492)return c===ye?o.COMPRESSED_SRGB8_ETC2:o.COMPRESSED_RGB8_ETC2;if(s===37496)return c===ye?o.COMPRESSED_SRGB8_ALPHA8_ETC2_EAC:o.COMPRESSED_RGBA8_ETC2_EAC}else return null;if(s===37808||s===37809||s===37810||s===37811||s===37812||s===37813||s===37814||s===37815||s===37816||s===37817||s===37818||s===37819||s===37820||s===37821)if(o=t.get("WEBGL_compressed_texture_astc"),o!==null){if(s===37808)return c===ye?o.COMPRESSED_SRGB8_ALPHA8_ASTC_4x4_KHR:o.COMPRESSED_RGBA_ASTC_4x4_KHR;if(s===37809)return c===ye?o.COMPRESSED_SRGB8_ALPHA8_ASTC_5x4_KHR:o.COMPRESSED_RGBA_ASTC_5x4_KHR;if(s===37810)return c===ye?o.COMPRESSED_SRGB8_ALPHA8_ASTC_5x5_KHR:o.COMPRESSED_RGBA_ASTC_5x5_KHR;if(s===37811)return c===ye?o.COMPRESSED_SRGB8_ALPHA8_ASTC_6x5_KHR:o.COMPRESSED_RGBA_ASTC_6x5_KHR;if(s===37812)return c===ye?o.COMPRESSED_SRGB8_ALPHA8_ASTC_6x6_KHR:o.COMPRESSED_RGBA_ASTC_6x6_KHR;if(s===37813)return c===ye?o.COMPRESSED_SRGB8_ALPHA8_ASTC_8x5_KHR:o.COMPRESSED_RGBA_ASTC_8x5_KHR;if(s===37814)return c===ye?o.COMPRESSED_SRGB8_ALPHA8_ASTC_8x6_KHR:o.COMPRESSED_RGBA_ASTC_8x6_KHR;if(s===37815)return c===ye?o.COMPRESSED_SRGB8_ALPHA8_ASTC_8x8_KHR:o.COMPRESSED_RGBA_ASTC_8x8_KHR;if(s===37816)return c===ye?o.COMPRESSED_SRGB8_ALPHA8_ASTC_10x5_KHR:o.COMPRESSED_RGBA_ASTC_10x5_KHR;if(s===37817)return c===ye?o.COMPRESSED_SRGB8_ALPHA8_ASTC_10x6_KHR:o.COMPRESSED_RGBA_ASTC_10x6_KHR;if(s===37818)return c===ye?o.COMPRESSED_SRGB8_ALPHA8_ASTC_10x8_KHR:o.COMPRESSED_RGBA_ASTC_10x8_KHR;if(s===37819)return c===ye?o.COMPRESSED_SRGB8_ALPHA8_ASTC_10x10_KHR:o.COMPRESSED_RGBA_ASTC_10x10_KHR;if(s===37820)return c===ye?o.COMPRESSED_SRGB8_ALPHA8_ASTC_12x10_KHR:o.COMPRESSED_RGBA_ASTC_12x10_KHR;if(s===37821)return c===ye?o.COMPRESSED_SRGB8_ALPHA8_ASTC_12x12_KHR:o.COMPRESSED_RGBA_ASTC_12x12_KHR}else return null;if(s===36492||s===36494||s===36495)if(o=t.get("EXT_texture_compression_bptc"),o!==null){if(s===36492)return c===ye?o.COMPRESSED_SRGB_ALPHA_BPTC_UNORM_EXT:o.COMPRESSED_RGBA_BPTC_UNORM_EXT;if(s===36494)return o.COMPRESSED_RGB_BPTC_SIGNED_FLOAT_EXT;if(s===36495)return o.COMPRESSED_RGB_BPTC_UNSIGNED_FLOAT_EXT}else return null;if(s===36283||s===36284||s===36285||s===36286)if(o=t.get("EXT_texture_compression_rgtc"),o!==null){if(s===36492)return o.COMPRESSED_RED_RGTC1_EXT;if(s===36284)return o.COMPRESSED_SIGNED_RED_RGTC1_EXT;if(s===36285)return o.COMPRESSED_RED_GREEN_RGTC2_EXT;if(s===36286)return o.COMPRESSED_SIGNED_RED_GREEN_RGTC2_EXT}else return null;return s===1020?n?r.UNSIGNED_INT_24_8:(o=t.get("WEBGL_depth_texture"),o!==null?o.UNSIGNED_INT_24_8_WEBGL:null):r[s]!==void 0?r[s]:null}return{convert:i}}class dv extends Dn{constructor(t=[]){super(),this.isArrayCamera=!0,this.cameras=t}}class Ne extends Je{constructor(){super(),this.isGroup=!0,this.type="Group"}}const fv={type:"move"};class Hc{constructor(){this._targetRay=null,this._grip=null,this._hand=null}getHandSpace(){return this._hand===null&&(this._hand=new Ne,this._hand.matrixAutoUpdate=!1,this._hand.visible=!1,this._hand.joints={},this._hand.inputState={pinching:!1}),this._hand}getTargetRaySpace(){return this._targetRay===null&&(this._targetRay=new Ne,this._targetRay.matrixAutoUpdate=!1,this._targetRay.visible=!1,this._targetRay.hasLinearVelocity=!1,this._targetRay.linearVelocity=new C,this._targetRay.hasAngularVelocity=!1,this._targetRay.angularVelocity=new C),this._targetRay}getGripSpace(){return this._grip===null&&(this._grip=new Ne,this._grip.matrixAutoUpdate=!1,this._grip.visible=!1,this._grip.hasLinearVelocity=!1,this._grip.linearVelocity=new C,this._grip.hasAngularVelocity=!1,this._grip.angularVelocity=new C),this._grip}dispatchEvent(t){return this._targetRay!==null&&this._targetRay.dispatchEvent(t),this._grip!==null&&this._grip.dispatchEvent(t),this._hand!==null&&this._hand.dispatchEvent(t),this}connect(t){if(t&&t.hand){const e=this._hand;if(e)for(const n of t.hand.values())this._getHandJoint(e,n)}return this.dispatchEvent({type:"connected",data:t}),this}disconnect(t){return this.dispatchEvent({type:"disconnected",data:t}),this._targetRay!==null&&(this._targetRay.visible=!1),this._grip!==null&&(this._grip.visible=!1),this._hand!==null&&(this._hand.visible=!1),this}update(t,e,n){let i=null,s=null,a=null;const o=this._targetRay,c=this._grip,l=this._hand;if(t&&e.session.visibilityState!=="visible-blurred"){if(l&&t.hand){a=!0;for(const x of t.hand.values()){const g=e.getJointPose(x,n),p=this._getHandJoint(l,x);g!==null&&(p.matrix.fromArray(g.transform.matrix),p.matrix.decompose(p.position,p.rotation,p.scale),p.matrixWorldNeedsUpdate=!0,p.jointRadius=g.radius),p.visible=g!==null}const h=l.joints["index-finger-tip"],u=l.joints["thumb-tip"],d=h.position.distanceTo(u.position),f=.02,m=.005;l.inputState.pinching&&d>f+m?(l.inputState.pinching=!1,this.dispatchEvent({type:"pinchend",handedness:t.handedness,target:this})):!l.inputState.pinching&&d<=f-m&&(l.inputState.pinching=!0,this.dispatchEvent({type:"pinchstart",handedness:t.handedness,target:this}))}else c!==null&&t.gripSpace&&(s=e.getPose(t.gripSpace,n),s!==null&&(c.matrix.fromArray(s.transform.matrix),c.matrix.decompose(c.position,c.rotation,c.scale),c.matrixWorldNeedsUpdate=!0,s.linearVelocity?(c.hasLinearVelocity=!0,c.linearVelocity.copy(s.linearVelocity)):c.hasLinearVelocity=!1,s.angularVelocity?(c.hasAngularVelocity=!0,c.angularVelocity.copy(s.angularVelocity)):c.hasAngularVelocity=!1));o!==null&&(i=e.getPose(t.targetRaySpace,n),i===null&&s!==null&&(i=s),i!==null&&(o.matrix.fromArray(i.transform.matrix),o.matrix.decompose(o.position,o.rotation,o.scale),o.matrixWorldNeedsUpdate=!0,i.linearVelocity?(o.hasLinearVelocity=!0,o.linearVelocity.copy(i.linearVelocity)):o.hasLinearVelocity=!1,i.angularVelocity?(o.hasAngularVelocity=!0,o.angularVelocity.copy(i.angularVelocity)):o.hasAngularVelocity=!1,this.dispatchEvent(fv)))}return o!==null&&(o.visible=i!==null),c!==null&&(c.visible=s!==null),l!==null&&(l.visible=a!==null),this}_getHandJoint(t,e){if(t.joints[e.jointName]===void 0){const n=new Ne;n.matrixAutoUpdate=!1,n.visible=!1,t.joints[e.jointName]=n,t.add(n)}return t.joints[e.jointName]}}class pv extends Sr{constructor(t,e){super();const n=this;let i=null,s=1,a=null,o="local-floor",c=1,l=null,h=null,u=null,d=null,f=null,m=null;const x=e.getContextAttributes();let g=null,p=null;const _=[],v=[],S=new yt;let M=null;const y=new Dn;y.layers.enable(1),y.viewport=new Ke;const w=new Dn;w.layers.enable(2),w.viewport=new Ke;const T=[y,w],E=new dv;E.layers.enable(1),E.layers.enable(2);let A=null,D=null;this.cameraAutoUpdate=!0,this.enabled=!1,this.isPresenting=!1,this.getController=function(V){let $=_[V];return $===void 0&&($=new Hc,_[V]=$),$.getTargetRaySpace()},this.getControllerGrip=function(V){let $=_[V];return $===void 0&&($=new Hc,_[V]=$),$.getGripSpace()},this.getHand=function(V){let $=_[V];return $===void 0&&($=new Hc,_[V]=$),$.getHandSpace()};function b(V){const $=v.indexOf(V.inputSource);if($===-1)return;const st=_[$];st!==void 0&&(st.update(V.inputSource,V.frame,l||a),st.dispatchEvent({type:V.type,data:V.inputSource}))}function F(){i.removeEventListener("select",b),i.removeEventListener("selectstart",b),i.removeEventListener("selectend",b),i.removeEventListener("squeeze",b),i.removeEventListener("squeezestart",b),i.removeEventListener("squeezeend",b),i.removeEventListener("end",F),i.removeEventListener("inputsourceschange",L);for(let V=0;V<_.length;V++){const $=v[V];$!==null&&(v[V]=null,_[V].disconnect($))}A=null,D=null,t.setRenderTarget(g),f=null,d=null,u=null,i=null,p=null,nt.stop(),n.isPresenting=!1,t.setPixelRatio(M),t.setSize(S.width,S.height,!1),n.dispatchEvent({type:"sessionend"})}this.setFramebufferScaleFactor=function(V){s=V,n.isPresenting===!0&&console.warn("THREE.WebXRManager: Cannot change framebuffer scale while presenting.")},this.setReferenceSpaceType=function(V){o=V,n.isPresenting===!0&&console.warn("THREE.WebXRManager: Cannot change reference space type while presenting.")},this.getReferenceSpace=function(){return l||a},this.setReferenceSpace=function(V){l=V},this.getBaseLayer=function(){return d!==null?d:f},this.getBinding=function(){return u},this.getFrame=function(){return m},this.getSession=function(){return i},this.setSession=async function(V){if(i=V,i!==null){if(g=t.getRenderTarget(),i.addEventListener("select",b),i.addEventListener("selectstart",b),i.addEventListener("selectend",b),i.addEventListener("squeeze",b),i.addEventListener("squeezestart",b),i.addEventListener("squeezeend",b),i.addEventListener("end",F),i.addEventListener("inputsourceschange",L),x.xrCompatible!==!0&&await e.makeXRCompatible(),M=t.getPixelRatio(),t.getSize(S),i.renderState.layers===void 0||t.capabilities.isWebGL2===!1){const $={antialias:i.renderState.layers===void 0?x.antialias:!0,alpha:!0,depth:x.depth,stencil:x.stencil,framebufferScaleFactor:s};f=new XRWebGLLayer(i,e,$),i.updateRenderState({baseLayer:f}),t.setPixelRatio(1),t.setSize(f.framebufferWidth,f.framebufferHeight,!1),p=new Qe(f.framebufferWidth,f.framebufferHeight,{format:1023,type:1009,colorSpace:t.outputColorSpace,stencilBuffer:x.stencil})}else{let $=null,st=null,at=null;x.depth&&(at=x.stencil?e.DEPTH24_STENCIL8:e.DEPTH_COMPONENT24,$=x.stencil?1027:1026,st=x.stencil?1020:1014);const lt={colorFormat:e.RGBA8,depthFormat:at,scaleFactor:s};u=new XRWebGLBinding(i,e),d=u.createProjectionLayer(lt),i.updateRenderState({layers:[d]}),t.setPixelRatio(1),t.setSize(d.textureWidth,d.textureHeight,!1),p=new Qe(d.textureWidth,d.textureHeight,{format:1023,type:1009,depthTexture:new zf(d.textureWidth,d.textureHeight,st,void 0,void 0,void 0,void 0,void 0,void 0,$),stencilBuffer:x.stencil,colorSpace:t.outputColorSpace,samples:x.antialias?4:0});const _t=t.properties.get(p);_t.__ignoreDepthValues=d.ignoreDepthValues}p.isXRRenderTarget=!0,this.setFoveation(c),l=null,a=await i.requestReferenceSpace(o),nt.setContext(i),nt.start(),n.isPresenting=!0,n.dispatchEvent({type:"sessionstart"})}},this.getEnvironmentBlendMode=function(){if(i!==null)return i.environmentBlendMode};function L(V){for(let $=0;$<V.removed.length;$++){const st=V.removed[$],at=v.indexOf(st);at>=0&&(v[at]=null,_[at].disconnect(st))}for(let $=0;$<V.added.length;$++){const st=V.added[$];let at=v.indexOf(st);if(at===-1){for(let _t=0;_t<_.length;_t++)if(_t>=v.length){v.push(st),at=_t;break}else if(v[_t]===null){v[_t]=st,at=_t;break}if(at===-1)break}const lt=_[at];lt&&lt.connect(st)}}const N=new C,U=new C;function k(V,$,st){N.setFromMatrixPosition($.matrixWorld),U.setFromMatrixPosition(st.matrixWorld);const at=N.distanceTo(U),lt=$.projectionMatrix.elements,_t=st.projectionMatrix.elements,ht=lt[14]/(lt[10]-1),ut=lt[14]/(lt[10]+1),bt=(lt[9]+1)/lt[5],Y=(lt[9]-1)/lt[5],Nt=(lt[8]-1)/lt[0],wt=(_t[8]+1)/_t[0],Ct=ht*Nt,vt=ht*wt,Jt=at/(-Nt+wt),Lt=Jt*-Nt;$.matrixWorld.decompose(V.position,V.quaternion,V.scale),V.translateX(Lt),V.translateZ(Jt),V.matrixWorld.compose(V.position,V.quaternion,V.scale),V.matrixWorldInverse.copy(V.matrixWorld).invert();const I=ht+Jt,R=ut+Jt,W=Ct-Lt,Q=vt+(at-Lt),K=bt*ut/R*I,tt=Y*ut/R*I;V.projectionMatrix.makePerspective(W,Q,K,tt,I,R),V.projectionMatrixInverse.copy(V.projectionMatrix).invert()}function O(V,$){$===null?V.matrixWorld.copy(V.matrix):V.matrixWorld.multiplyMatrices($.matrixWorld,V.matrix),V.matrixWorldInverse.copy(V.matrixWorld).invert()}this.updateCamera=function(V){if(i===null)return;E.near=w.near=y.near=V.near,E.far=w.far=y.far=V.far,(A!==E.near||D!==E.far)&&(i.updateRenderState({depthNear:E.near,depthFar:E.far}),A=E.near,D=E.far);const $=V.parent,st=E.cameras;O(E,$);for(let at=0;at<st.length;at++)O(st[at],$);st.length===2?k(E,y,w):E.projectionMatrix.copy(y.projectionMatrix),H(V,E,$)};function H(V,$,st){st===null?V.matrix.copy($.matrixWorld):(V.matrix.copy(st.matrixWorld),V.matrix.invert(),V.matrix.multiply($.matrixWorld)),V.matrix.decompose(V.position,V.quaternion,V.scale),V.updateMatrixWorld(!0),V.projectionMatrix.copy($.projectionMatrix),V.projectionMatrixInverse.copy($.projectionMatrixInverse),V.isPerspectiveCamera&&(V.fov=Ta*2*Math.atan(1/V.projectionMatrix.elements[5]),V.zoom=1)}this.getCamera=function(){return E},this.getFoveation=function(){if(!(d===null&&f===null))return c},this.setFoveation=function(V){c=V,d!==null&&(d.fixedFoveation=V),f!==null&&f.fixedFoveation!==void 0&&(f.fixedFoveation=V)};let j=null;function J(V,$){if(h=$.getViewerPose(l||a),m=$,h!==null){const st=h.views;f!==null&&(t.setRenderTargetFramebuffer(p,f.framebuffer),t.setRenderTarget(p));let at=!1;st.length!==E.cameras.length&&(E.cameras.length=0,at=!0);for(let lt=0;lt<st.length;lt++){const _t=st[lt];let ht=null;if(f!==null)ht=f.getViewport(_t);else{const bt=u.getViewSubImage(d,_t);ht=bt.viewport,lt===0&&(t.setRenderTargetTextures(p,bt.colorTexture,d.ignoreDepthValues?void 0:bt.depthStencilTexture),t.setRenderTarget(p))}let ut=T[lt];ut===void 0&&(ut=new Dn,ut.layers.enable(lt),ut.viewport=new Ke,T[lt]=ut),ut.matrix.fromArray(_t.transform.matrix),ut.matrix.decompose(ut.position,ut.quaternion,ut.scale),ut.projectionMatrix.fromArray(_t.projectionMatrix),ut.projectionMatrixInverse.copy(ut.projectionMatrix).invert(),ut.viewport.set(ht.x,ht.y,ht.width,ht.height),lt===0&&(E.matrix.copy(ut.matrix),E.matrix.decompose(E.position,E.quaternion,E.scale)),at===!0&&E.cameras.push(ut)}}for(let st=0;st<_.length;st++){const at=v[st],lt=_[st];at!==null&&lt!==void 0&&lt.update(at,$,l||a)}j&&j(V,$),$.detectedPlanes&&n.dispatchEvent({type:"planesdetected",data:$}),m=null}const nt=new Bf;nt.setAnimationLoop(J),this.setAnimationLoop=function(V){j=V},this.dispose=function(){}}}function mv(r,t){function e(g,p){g.matrixAutoUpdate===!0&&g.updateMatrix(),p.value.copy(g.matrix)}function n(g,p){p.color.getRGB(g.fogColor.value,Nf(r)),p.isFog?(g.fogNear.value=p.near,g.fogFar.value=p.far):p.isFogExp2&&(g.fogDensity.value=p.density)}function i(g,p,_,v,S){p.isMeshBasicMaterial||p.isMeshLambertMaterial?s(g,p):p.isMeshToonMaterial?(s(g,p),u(g,p)):p.isMeshPhongMaterial?(s(g,p),h(g,p)):p.isMeshStandardMaterial?(s(g,p),d(g,p),p.isMeshPhysicalMaterial&&f(g,p,S)):p.isMeshMatcapMaterial?(s(g,p),m(g,p)):p.isMeshDepthMaterial?s(g,p):p.isMeshDistanceMaterial?(s(g,p),x(g,p)):p.isMeshNormalMaterial?s(g,p):p.isLineBasicMaterial?(a(g,p),p.isLineDashedMaterial&&o(g,p)):p.isPointsMaterial?c(g,p,_,v):p.isSpriteMaterial?l(g,p):p.isShadowMaterial?(g.color.value.copy(p.color),g.opacity.value=p.opacity):p.isShaderMaterial&&(p.uniformsNeedUpdate=!1)}function s(g,p){g.opacity.value=p.opacity,p.color&&g.diffuse.value.copy(p.color),p.emissive&&g.emissive.value.copy(p.emissive).multiplyScalar(p.emissiveIntensity),p.map&&(g.map.value=p.map,e(p.map,g.mapTransform)),p.alphaMap&&(g.alphaMap.value=p.alphaMap,e(p.alphaMap,g.alphaMapTransform)),p.bumpMap&&(g.bumpMap.value=p.bumpMap,e(p.bumpMap,g.bumpMapTransform),g.bumpScale.value=p.bumpScale,p.side===1&&(g.bumpScale.value*=-1)),p.normalMap&&(g.normalMap.value=p.normalMap,e(p.normalMap,g.normalMapTransform),g.normalScale.value.copy(p.normalScale),p.side===1&&g.normalScale.value.negate()),p.displacementMap&&(g.displacementMap.value=p.displacementMap,e(p.displacementMap,g.displacementMapTransform),g.displacementScale.value=p.displacementScale,g.displacementBias.value=p.displacementBias),p.emissiveMap&&(g.emissiveMap.value=p.emissiveMap,e(p.emissiveMap,g.emissiveMapTransform)),p.specularMap&&(g.specularMap.value=p.specularMap,e(p.specularMap,g.specularMapTransform)),p.alphaTest>0&&(g.alphaTest.value=p.alphaTest);const _=t.get(p).envMap;if(_&&(g.envMap.value=_,g.flipEnvMap.value=_.isCubeTexture&&_.isRenderTargetTexture===!1?-1:1,g.reflectivity.value=p.reflectivity,g.ior.value=p.ior,g.refractionRatio.value=p.refractionRatio),p.lightMap){g.lightMap.value=p.lightMap;const v=r._useLegacyLights===!0?Math.PI:1;g.lightMapIntensity.value=p.lightMapIntensity*v,e(p.lightMap,g.lightMapTransform)}p.aoMap&&(g.aoMap.value=p.aoMap,g.aoMapIntensity.value=p.aoMapIntensity,e(p.aoMap,g.aoMapTransform))}function a(g,p){g.diffuse.value.copy(p.color),g.opacity.value=p.opacity,p.map&&(g.map.value=p.map,e(p.map,g.mapTransform))}function o(g,p){g.dashSize.value=p.dashSize,g.totalSize.value=p.dashSize+p.gapSize,g.scale.value=p.scale}function c(g,p,_,v){g.diffuse.value.copy(p.color),g.opacity.value=p.opacity,g.size.value=p.size*_,g.scale.value=v*.5,p.map&&(g.map.value=p.map,e(p.map,g.uvTransform)),p.alphaMap&&(g.alphaMap.value=p.alphaMap,e(p.alphaMap,g.alphaMapTransform)),p.alphaTest>0&&(g.alphaTest.value=p.alphaTest)}function l(g,p){g.diffuse.value.copy(p.color),g.opacity.value=p.opacity,g.rotation.value=p.rotation,p.map&&(g.map.value=p.map,e(p.map,g.mapTransform)),p.alphaMap&&(g.alphaMap.value=p.alphaMap,e(p.alphaMap,g.alphaMapTransform)),p.alphaTest>0&&(g.alphaTest.value=p.alphaTest)}function h(g,p){g.specular.value.copy(p.specular),g.shininess.value=Math.max(p.shininess,1e-4)}function u(g,p){p.gradientMap&&(g.gradientMap.value=p.gradientMap)}function d(g,p){g.metalness.value=p.metalness,p.metalnessMap&&(g.metalnessMap.value=p.metalnessMap,e(p.metalnessMap,g.metalnessMapTransform)),g.roughness.value=p.roughness,p.roughnessMap&&(g.roughnessMap.value=p.roughnessMap,e(p.roughnessMap,g.roughnessMapTransform)),t.get(p).envMap&&(g.envMapIntensity.value=p.envMapIntensity)}function f(g,p,_){g.ior.value=p.ior,p.sheen>0&&(g.sheenColor.value.copy(p.sheenColor).multiplyScalar(p.sheen),g.sheenRoughness.value=p.sheenRoughness,p.sheenColorMap&&(g.sheenColorMap.value=p.sheenColorMap,e(p.sheenColorMap,g.sheenColorMapTransform)),p.sheenRoughnessMap&&(g.sheenRoughnessMap.value=p.sheenRoughnessMap,e(p.sheenRoughnessMap,g.sheenRoughnessMapTransform))),p.clearcoat>0&&(g.clearcoat.value=p.clearcoat,g.clearcoatRoughness.value=p.clearcoatRoughness,p.clearcoatMap&&(g.clearcoatMap.value=p.clearcoatMap,e(p.clearcoatMap,g.clearcoatMapTransform)),p.clearcoatRoughnessMap&&(g.clearcoatRoughnessMap.value=p.clearcoatRoughnessMap,e(p.clearcoatRoughnessMap,g.clearcoatRoughnessMapTransform)),p.clearcoatNormalMap&&(g.clearcoatNormalMap.value=p.clearcoatNormalMap,e(p.clearcoatNormalMap,g.clearcoatNormalMapTransform),g.clearcoatNormalScale.value.copy(p.clearcoatNormalScale),p.side===1&&g.clearcoatNormalScale.value.negate())),p.iridescence>0&&(g.iridescence.value=p.iridescence,g.iridescenceIOR.value=p.iridescenceIOR,g.iridescenceThicknessMinimum.value=p.iridescenceThicknessRange[0],g.iridescenceThicknessMaximum.value=p.iridescenceThicknessRange[1],p.iridescenceMap&&(g.iridescenceMap.value=p.iridescenceMap,e(p.iridescenceMap,g.iridescenceMapTransform)),p.iridescenceThicknessMap&&(g.iridescenceThicknessMap.value=p.iridescenceThicknessMap,e(p.iridescenceThicknessMap,g.iridescenceThicknessMapTransform))),p.transmission>0&&(g.transmission.value=p.transmission,g.transmissionSamplerMap.value=_.texture,g.transmissionSamplerSize.value.set(_.width,_.height),p.transmissionMap&&(g.transmissionMap.value=p.transmissionMap,e(p.transmissionMap,g.transmissionMapTransform)),g.thickness.value=p.thickness,p.thicknessMap&&(g.thicknessMap.value=p.thicknessMap,e(p.thicknessMap,g.thicknessMapTransform)),g.attenuationDistance.value=p.attenuationDistance,g.attenuationColor.value.copy(p.attenuationColor)),p.anisotropy>0&&(g.anisotropyVector.value.set(p.anisotropy*Math.cos(p.anisotropyRotation),p.anisotropy*Math.sin(p.anisotropyRotation)),p.anisotropyMap&&(g.anisotropyMap.value=p.anisotropyMap,e(p.anisotropyMap,g.anisotropyMapTransform))),g.specularIntensity.value=p.specularIntensity,g.specularColor.value.copy(p.specularColor),p.specularColorMap&&(g.specularColorMap.value=p.specularColorMap,e(p.specularColorMap,g.specularColorMapTransform)),p.specularIntensityMap&&(g.specularIntensityMap.value=p.specularIntensityMap,e(p.specularIntensityMap,g.specularIntensityMapTransform))}function m(g,p){p.matcap&&(g.matcap.value=p.matcap)}function x(g,p){const _=t.get(p).light;g.referencePosition.value.setFromMatrixPosition(_.matrixWorld),g.nearDistance.value=_.shadow.camera.near,g.farDistance.value=_.shadow.camera.far}return{refreshFogUniforms:n,refreshMaterialUniforms:i}}function gv(r,t,e,n){let i={},s={},a=[];const o=e.isWebGL2?r.getParameter(r.MAX_UNIFORM_BUFFER_BINDINGS):0;function c(_,v){const S=v.program;n.uniformBlockBinding(_,S)}function l(_,v){let S=i[_.id];S===void 0&&(m(_),S=h(_),i[_.id]=S,_.addEventListener("dispose",g));const M=v.program;n.updateUBOMapping(_,M);const y=t.render.frame;s[_.id]!==y&&(d(_),s[_.id]=y)}function h(_){const v=u();_.__bindingPointIndex=v;const S=r.createBuffer(),M=_.__size,y=_.usage;return r.bindBuffer(r.UNIFORM_BUFFER,S),r.bufferData(r.UNIFORM_BUFFER,M,y),r.bindBuffer(r.UNIFORM_BUFFER,null),r.bindBufferBase(r.UNIFORM_BUFFER,v,S),S}function u(){for(let _=0;_<o;_++)if(a.indexOf(_)===-1)return a.push(_),_;return console.error("THREE.WebGLRenderer: Maximum number of simultaneously usable uniforms groups reached."),0}function d(_){const v=i[_.id],S=_.uniforms,M=_.__cache;r.bindBuffer(r.UNIFORM_BUFFER,v);for(let y=0,w=S.length;y<w;y++){const T=Array.isArray(S[y])?S[y]:[S[y]];for(let E=0,A=T.length;E<A;E++){const D=T[E];if(f(D,y,E,M)===!0){const b=D.__offset,F=Array.isArray(D.value)?D.value:[D.value];let L=0;for(let N=0;N<F.length;N++){const U=F[N],k=x(U);typeof U=="number"||typeof U=="boolean"?(D.__data[0]=U,r.bufferSubData(r.UNIFORM_BUFFER,b+L,D.__data)):U.isMatrix3?(D.__data[0]=U.elements[0],D.__data[1]=U.elements[1],D.__data[2]=U.elements[2],D.__data[3]=0,D.__data[4]=U.elements[3],D.__data[5]=U.elements[4],D.__data[6]=U.elements[5],D.__data[7]=0,D.__data[8]=U.elements[6],D.__data[9]=U.elements[7],D.__data[10]=U.elements[8],D.__data[11]=0):(U.toArray(D.__data,L),L+=k.storage/Float32Array.BYTES_PER_ELEMENT)}r.bufferSubData(r.UNIFORM_BUFFER,b,D.__data)}}}r.bindBuffer(r.UNIFORM_BUFFER,null)}function f(_,v,S,M){const y=_.value,w=v+"_"+S;if(M[w]===void 0)return typeof y=="number"||typeof y=="boolean"?M[w]=y:M[w]=y.clone(),!0;{const T=M[w];if(typeof y=="number"||typeof y=="boolean"){if(T!==y)return M[w]=y,!0}else if(T.equals(y)===!1)return T.copy(y),!0}return!1}function m(_){const v=_.uniforms;let S=0;const M=16;for(let w=0,T=v.length;w<T;w++){const E=Array.isArray(v[w])?v[w]:[v[w]];for(let A=0,D=E.length;A<D;A++){const b=E[A],F=Array.isArray(b.value)?b.value:[b.value];for(let L=0,N=F.length;L<N;L++){const U=F[L],k=x(U),O=S%M;O!==0&&M-O<k.boundary&&(S+=M-O),b.__data=new Float32Array(k.storage/Float32Array.BYTES_PER_ELEMENT),b.__offset=S,S+=k.storage}}}const y=S%M;return y>0&&(S+=M-y),_.__size=S,_.__cache={},this}function x(_){const v={boundary:0,storage:0};return typeof _=="number"||typeof _=="boolean"?(v.boundary=4,v.storage=4):_.isVector2?(v.boundary=8,v.storage=8):_.isVector3||_.isColor?(v.boundary=16,v.storage=12):_.isVector4?(v.boundary=16,v.storage=16):_.isMatrix3?(v.boundary=48,v.storage=48):_.isMatrix4?(v.boundary=64,v.storage=64):_.isTexture?console.warn("THREE.WebGLRenderer: Texture samplers can not be part of an uniforms group."):console.warn("THREE.WebGLRenderer: Unsupported uniform value type.",_),v}function g(_){const v=_.target;v.removeEventListener("dispose",g);const S=a.indexOf(v.__bindingPointIndex);a.splice(S,1),r.deleteBuffer(i[v.id]),delete i[v.id],delete s[v.id]}function p(){for(const _ in i)r.deleteBuffer(i[_]);a=[],i={},s={}}return{bind:c,update:l,dispose:p}}class Ch{constructor(t={}){const{canvas:e=dm(),context:n=null,depth:i=!0,stencil:s=!0,alpha:a=!1,antialias:o=!1,premultipliedAlpha:c=!0,preserveDrawingBuffer:l=!1,powerPreference:h="default",failIfMajorPerformanceCaveat:u=!1}=t;this.isWebGLRenderer=!0;let d;n!==null?d=n.getContextAttributes().alpha:d=a;const f=new Uint32Array(4),m=new Int32Array(4);let x=null,g=null;const p=[],_=[];this.domElement=e,this.debug={checkShaderErrors:!0,onShaderError:null},this.autoClear=!0,this.autoClearColor=!0,this.autoClearDepth=!0,this.autoClearStencil=!0,this.sortObjects=!0,this.clippingPlanes=[],this.localClippingEnabled=!1,this._outputColorSpace=nn,this._useLegacyLights=!1,this.toneMapping=0,this.toneMappingExposure=1;const v=this;let S=!1,M=0,y=0,w=null,T=-1,E=null;const A=new Ke,D=new Ke;let b=null;const F=new Kt(0);let L=0,N=e.width,U=e.height,k=1,O=null,H=null;const j=new Ke(0,0,N,U),J=new Ke(0,0,N,U);let nt=!1;const V=new wh;let $=!1,st=!1,at=null;const lt=new re,_t=new yt,ht=new C,ut={background:null,fog:null,environment:null,overrideMaterial:null,isScene:!0};function bt(){return w===null?k:1}let Y=n;function Nt(P,G){for(let q=0;q<P.length;q++){const Z=P[q],X=e.getContext(Z,G);if(X!==null)return X}return null}try{const P={alpha:!0,depth:i,stencil:s,antialias:o,premultipliedAlpha:c,preserveDrawingBuffer:l,powerPreference:h,failIfMajorPerformanceCaveat:u};if("setAttribute"in e&&e.setAttribute("data-engine","three.js r160"),e.addEventListener("webglcontextlost",ot,!1),e.addEventListener("webglcontextrestored",B,!1),e.addEventListener("webglcontextcreationerror",dt,!1),Y===null){const G=["webgl2","webgl","experimental-webgl"];if(v.isWebGL1Renderer===!0&&G.shift(),Y=Nt(G,P),Y===null)throw Nt(G)?new Error("Error creating WebGL context with your selected attributes."):new Error("Error creating WebGL context.")}typeof WebGLRenderingContext<"u"&&Y instanceof WebGLRenderingContext&&console.warn("THREE.WebGLRenderer: WebGL 1 support was deprecated in r153 and will be removed in r163."),Y.getShaderPrecisionFormat===void 0&&(Y.getShaderPrecisionFormat=function(){return{rangeMin:1,rangeMax:1,precision:1}})}catch(P){throw console.error("THREE.WebGLRenderer: "+P.message),P}let wt,Ct,vt,Jt,Lt,I,R,W,Q,K,tt,Mt,rt,gt,Pt,Wt,et,oe,jt,kt,At,xt,Ft,Qt;function fe(){wt=new Tx(Y),Ct=new Sx(Y,wt,t),wt.init(Ct),xt=new uv(Y,wt,Ct),vt=new lv(Y,wt,Ct),Jt=new Rx(Y),Lt=new j_,I=new hv(Y,wt,vt,Lt,Ct,xt,Jt),R=new yx(v),W=new Ax(v),Q=new Bm(Y,Ct),Ft=new _x(Y,wt,Q,Ct),K=new Cx(Y,Q,Jt,Ft),tt=new Ix(Y,K,Q,Jt),jt=new Dx(Y,Ct,I),Wt=new Mx(Lt),Mt=new Z_(v,R,W,wt,Ct,Ft,Wt),rt=new mv(v,Lt),gt=new K_,Pt=new iv(wt,Ct),oe=new xx(v,R,W,vt,tt,d,c),et=new cv(v,tt,Ct),Qt=new gv(Y,Jt,Ct,vt),kt=new vx(Y,wt,Jt,Ct),At=new bx(Y,wt,Jt,Ct),Jt.programs=Mt.programs,v.capabilities=Ct,v.extensions=wt,v.properties=Lt,v.renderLists=gt,v.shadowMap=et,v.state=vt,v.info=Jt}fe();const qt=new pv(v,Y);this.xr=qt,this.getContext=function(){return Y},this.getContextAttributes=function(){return Y.getContextAttributes()},this.forceContextLoss=function(){const P=wt.get("WEBGL_lose_context");P&&P.loseContext()},this.forceContextRestore=function(){const P=wt.get("WEBGL_lose_context");P&&P.restoreContext()},this.getPixelRatio=function(){return k},this.setPixelRatio=function(P){P!==void 0&&(k=P,this.setSize(N,U,!1))},this.getSize=function(P){return P.set(N,U)},this.setSize=function(P,G,q=!0){if(qt.isPresenting){console.warn("THREE.WebGLRenderer: Can't change size while VR device is presenting.");return}N=P,U=G,e.width=Math.floor(P*k),e.height=Math.floor(G*k),q===!0&&(e.style.width=P+"px",e.style.height=G+"px"),this.setViewport(0,0,P,G)},this.getDrawingBufferSize=function(P){return P.set(N*k,U*k).floor()},this.setDrawingBufferSize=function(P,G,q){N=P,U=G,k=q,e.width=Math.floor(P*q),e.height=Math.floor(G*q),this.setViewport(0,0,P,G)},this.getCurrentViewport=function(P){return P.copy(A)},this.getViewport=function(P){return P.copy(j)},this.setViewport=function(P,G,q,Z){P.isVector4?j.set(P.x,P.y,P.z,P.w):j.set(P,G,q,Z),vt.viewport(A.copy(j).multiplyScalar(k).floor())},this.getScissor=function(P){return P.copy(J)},this.setScissor=function(P,G,q,Z){P.isVector4?J.set(P.x,P.y,P.z,P.w):J.set(P,G,q,Z),vt.scissor(D.copy(J).multiplyScalar(k).floor())},this.getScissorTest=function(){return nt},this.setScissorTest=function(P){vt.setScissorTest(nt=P)},this.setOpaqueSort=function(P){O=P},this.setTransparentSort=function(P){H=P},this.getClearColor=function(P){return P.copy(oe.getClearColor())},this.setClearColor=function(){oe.setClearColor.apply(oe,arguments)},this.getClearAlpha=function(){return oe.getClearAlpha()},this.setClearAlpha=function(){oe.setClearAlpha.apply(oe,arguments)},this.clear=function(P=!0,G=!0,q=!0){let Z=0;if(P){let X=!1;if(w!==null){const ct=w.texture.format;X=ct===1033||ct===1031||ct===1029}if(X){const ct=w.texture.type,Tt=ct===1009||ct===1014||ct===1012||ct===1020||ct===1017||ct===1018,Dt=oe.getClearColor(),St=oe.getClearAlpha(),Ht=Dt.r,Bt=Dt.g,Vt=Dt.b;Tt?(f[0]=Ht,f[1]=Bt,f[2]=Vt,f[3]=St,Y.clearBufferuiv(Y.COLOR,0,f)):(m[0]=Ht,m[1]=Bt,m[2]=Vt,m[3]=St,Y.clearBufferiv(Y.COLOR,0,m))}else Z|=Y.COLOR_BUFFER_BIT}G&&(Z|=Y.DEPTH_BUFFER_BIT),q&&(Z|=Y.STENCIL_BUFFER_BIT,this.state.buffers.stencil.setMask(4294967295)),Y.clear(Z)},this.clearColor=function(){this.clear(!0,!1,!1)},this.clearDepth=function(){this.clear(!1,!0,!1)},this.clearStencil=function(){this.clear(!1,!1,!0)},this.dispose=function(){e.removeEventListener("webglcontextlost",ot,!1),e.removeEventListener("webglcontextrestored",B,!1),e.removeEventListener("webglcontextcreationerror",dt,!1),gt.dispose(),Pt.dispose(),Lt.dispose(),R.dispose(),W.dispose(),tt.dispose(),Ft.dispose(),Qt.dispose(),Mt.dispose(),qt.dispose(),qt.removeEventListener("sessionstart",Ce),qt.removeEventListener("sessionend",ae),at&&(at.dispose(),at=null),Le.stop()};function ot(P){P.preventDefault(),console.log("THREE.WebGLRenderer: Context Lost."),S=!0}function B(){console.log("THREE.WebGLRenderer: Context Restored."),S=!1;const P=Jt.autoReset,G=et.enabled,q=et.autoUpdate,Z=et.needsUpdate,X=et.type;fe(),Jt.autoReset=P,et.enabled=G,et.autoUpdate=q,et.needsUpdate=Z,et.type=X}function dt(P){console.error("THREE.WebGLRenderer: A WebGL context could not be created. Reason: ",P.statusMessage)}function ft(P){const G=P.target;G.removeEventListener("dispose",ft),Gt(G)}function Gt(P){Ut(P),Lt.remove(P)}function Ut(P){const G=Lt.get(P).programs;G!==void 0&&(G.forEach(function(q){Mt.releaseProgram(q)}),P.isShaderMaterial&&Mt.releaseShaderCache(P))}this.renderBufferDirect=function(P,G,q,Z,X,ct){G===null&&(G=ut);const Tt=X.isMesh&&X.matrixWorld.determinant()<0,Dt=xi(P,G,q,Z,X);vt.setMaterial(Z,Tt);let St=q.index,Ht=1;if(Z.wireframe===!0){if(St=K.getWireframeAttribute(q),St===void 0)return;Ht=2}const Bt=q.drawRange,Vt=q.attributes.position;let pe=Bt.start*Ht,Ze=(Bt.start+Bt.count)*Ht;ct!==null&&(pe=Math.max(pe,ct.start*Ht),Ze=Math.min(Ze,(ct.start+ct.count)*Ht)),St!==null?(pe=Math.max(pe,0),Ze=Math.min(Ze,St.count)):Vt!=null&&(pe=Math.max(pe,0),Ze=Math.min(Ze,Vt.count));const be=Ze-pe;if(be<0||be===1/0)return;Ft.setup(X,Z,Dt,q,St);let Yn,me=kt;if(St!==null&&(Yn=Q.get(St),me=At,me.setIndex(Yn)),X.isMesh)Z.wireframe===!0?(vt.setLineWidth(Z.wireframeLinewidth*bt()),me.setMode(Y.LINES)):me.setMode(Y.TRIANGLES);else if(X.isLine){let te=Z.linewidth;te===void 0&&(te=1),vt.setLineWidth(te*bt()),X.isLineSegments?me.setMode(Y.LINES):X.isLineLoop?me.setMode(Y.LINE_LOOP):me.setMode(Y.LINE_STRIP)}else X.isPoints?me.setMode(Y.POINTS):X.isSprite&&me.setMode(Y.TRIANGLES);if(X.isBatchedMesh)me.renderMultiDraw(X._multiDrawStarts,X._multiDrawCounts,X._multiDrawCount);else if(X.isInstancedMesh)me.renderInstances(pe,be,X.count);else if(q.isInstancedBufferGeometry){const te=q._maxInstanceCount!==void 0?q._maxInstanceCount:1/0,Cr=Math.min(q.instanceCount,te);me.renderInstances(pe,be,Cr)}else me.render(pe,be)};function de(P,G,q){P.transparent===!0&&P.side===2&&P.forceSinglePass===!1?(P.side=1,P.needsUpdate=!0,Cn(P,G,q),P.side=0,P.needsUpdate=!0,Cn(P,G,q),P.side=2):Cn(P,G,q)}this.compile=function(P,G,q=null){q===null&&(q=P),g=Pt.get(q),g.init(),_.push(g),q.traverseVisible(function(X){X.isLight&&X.layers.test(G.layers)&&(g.pushLight(X),X.castShadow&&g.pushShadow(X))}),P!==q&&P.traverseVisible(function(X){X.isLight&&X.layers.test(G.layers)&&(g.pushLight(X),X.castShadow&&g.pushShadow(X))}),g.setupLights(v._useLegacyLights);const Z=new Set;return P.traverse(function(X){const ct=X.material;if(ct)if(Array.isArray(ct))for(let Tt=0;Tt<ct.length;Tt++){const Dt=ct[Tt];de(Dt,q,X),Z.add(Dt)}else de(ct,q,X),Z.add(ct)}),_.pop(),g=null,Z},this.compileAsync=function(P,G,q=null){const Z=this.compile(P,G,q);return new Promise(X=>{function ct(){if(Z.forEach(function(Tt){Lt.get(Tt).currentProgram.isReady()&&Z.delete(Tt)}),Z.size===0){X(P);return}setTimeout(ct,10)}wt.get("KHR_parallel_shader_compile")!==null?ct():setTimeout(ct,10)})};let ce=null;function ve(P){ce&&ce(P)}function Ce(){Le.stop()}function ae(){Le.start()}const Le=new Bf;Le.setAnimationLoop(ve),typeof self<"u"&&Le.setContext(self),this.setAnimationLoop=function(P){ce=P,qt.setAnimationLoop(P),P===null?Le.stop():Le.start()},qt.addEventListener("sessionstart",Ce),qt.addEventListener("sessionend",ae),this.render=function(P,G){if(G!==void 0&&G.isCamera!==!0){console.error("THREE.WebGLRenderer.render: camera is not an instance of THREE.Camera.");return}if(S===!0)return;P.matrixWorldAutoUpdate===!0&&P.updateMatrixWorld(),G.parent===null&&G.matrixWorldAutoUpdate===!0&&G.updateMatrixWorld(),qt.enabled===!0&&qt.isPresenting===!0&&(qt.cameraAutoUpdate===!0&&qt.updateCamera(G),G=qt.getCamera()),P.isScene===!0&&P.onBeforeRender(v,P,G,w),g=Pt.get(P,_.length),g.init(),_.push(g),lt.multiplyMatrices(G.projectionMatrix,G.matrixWorldInverse),V.setFromProjectionMatrix(lt),st=this.localClippingEnabled,$=Wt.init(this.clippingPlanes,st),x=gt.get(P,p.length),x.init(),p.push(x),qe(P,G,0,v.sortObjects),x.finish(),v.sortObjects===!0&&x.sort(O,H),this.info.render.frame++,$===!0&&Wt.beginShadows();const q=g.state.shadowsArray;if(et.render(q,P,G),$===!0&&Wt.endShadows(),this.info.autoReset===!0&&this.info.reset(),oe.render(x,P),g.setupLights(v._useLegacyLights),G.isArrayCamera){const Z=G.cameras;for(let X=0,ct=Z.length;X<ct;X++){const Tt=Z[X];gi(x,P,Tt,Tt.viewport)}}else gi(x,P,G);w!==null&&(I.updateMultisampleRenderTarget(w),I.updateRenderTargetMipmap(w)),P.isScene===!0&&P.onAfterRender(v,P,G),Ft.resetDefaultState(),T=-1,E=null,_.pop(),_.length>0?g=_[_.length-1]:g=null,p.pop(),p.length>0?x=p[p.length-1]:x=null};function qe(P,G,q,Z){if(P.visible===!1)return;if(P.layers.test(G.layers)){if(P.isGroup)q=P.renderOrder;else if(P.isLOD)P.autoUpdate===!0&&P.update(G);else if(P.isLight)g.pushLight(P),P.castShadow&&g.pushShadow(P);else if(P.isSprite){if(!P.frustumCulled||V.intersectsSprite(P)){Z&&ht.setFromMatrixPosition(P.matrixWorld).applyMatrix4(lt);const Tt=tt.update(P),Dt=P.material;Dt.visible&&x.push(P,Tt,Dt,q,ht.z,null)}}else if((P.isMesh||P.isLine||P.isPoints)&&(!P.frustumCulled||V.intersectsObject(P))){const Tt=tt.update(P),Dt=P.material;if(Z&&(P.boundingSphere!==void 0?(P.boundingSphere===null&&P.computeBoundingSphere(),ht.copy(P.boundingSphere.center)):(Tt.boundingSphere===null&&Tt.computeBoundingSphere(),ht.copy(Tt.boundingSphere.center)),ht.applyMatrix4(P.matrixWorld).applyMatrix4(lt)),Array.isArray(Dt)){const St=Tt.groups;for(let Ht=0,Bt=St.length;Ht<Bt;Ht++){const Vt=St[Ht],pe=Dt[Vt.materialIndex];pe&&pe.visible&&x.push(P,Tt,pe,q,ht.z,Vt)}}else Dt.visible&&x.push(P,Tt,Dt,q,ht.z,null)}}const ct=P.children;for(let Tt=0,Dt=ct.length;Tt<Dt;Tt++)qe(ct[Tt],G,q,Z)}function gi(P,G,q,Z){const X=P.opaque,ct=P.transmissive,Tt=P.transparent;g.setupLightsView(q),$===!0&&Wt.setGlobalState(v.clippingPlanes,q),ct.length>0&&As(X,ct,G,q),Z&&vt.viewport(A.copy(Z)),X.length>0&&Tn(X,G,q),ct.length>0&&Tn(ct,G,q),Tt.length>0&&Tn(Tt,G,q),vt.buffers.depth.setTest(!0),vt.buffers.depth.setMask(!0),vt.buffers.color.setMask(!0),vt.setPolygonOffset(!1)}function As(P,G,q,Z){if((q.isScene===!0?q.overrideMaterial:null)!==null)return;const ct=Ct.isWebGL2;at===null&&(at=new Qe(1,1,{generateMipmaps:!0,type:wt.has("EXT_color_buffer_half_float")?1016:1009,minFilter:1008,samples:ct?4:0})),v.getDrawingBufferSize(_t),ct?at.setSize(_t.x,_t.y):at.setSize(Yo(_t.x),Yo(_t.y));const Tt=v.getRenderTarget();v.setRenderTarget(at),v.getClearColor(F),L=v.getClearAlpha(),L<1&&v.setClearColor(16777215,.5),v.clear();const Dt=v.toneMapping;v.toneMapping=0,Tn(P,q,Z),I.updateMultisampleRenderTarget(at),I.updateRenderTargetMipmap(at);let St=!1;for(let Ht=0,Bt=G.length;Ht<Bt;Ht++){const Vt=G[Ht],pe=Vt.object,Ze=Vt.geometry,be=Vt.material,Yn=Vt.group;if(be.side===2&&pe.layers.test(Z.layers)){const me=be.side;be.side=1,be.needsUpdate=!0,Bi(pe,q,Z,Ze,be,Yn),be.side=me,be.needsUpdate=!0,St=!0}}St===!0&&(I.updateMultisampleRenderTarget(at),I.updateRenderTargetMipmap(at)),v.setRenderTarget(Tt),v.setClearColor(F,L),v.toneMapping=Dt}function Tn(P,G,q){const Z=G.isScene===!0?G.overrideMaterial:null;for(let X=0,ct=P.length;X<ct;X++){const Tt=P[X],Dt=Tt.object,St=Tt.geometry,Ht=Z===null?Tt.material:Z,Bt=Tt.group;Dt.layers.test(q.layers)&&Bi(Dt,G,q,St,Ht,Bt)}}function Bi(P,G,q,Z,X,ct){P.onBeforeRender(v,G,q,Z,X,ct),P.modelViewMatrix.multiplyMatrices(q.matrixWorldInverse,P.matrixWorld),P.normalMatrix.getNormalMatrix(P.modelViewMatrix),X.onBeforeRender(v,G,q,Z,P,ct),X.transparent===!0&&X.side===2&&X.forceSinglePass===!1?(X.side=1,X.needsUpdate=!0,v.renderBufferDirect(q,G,Z,X,P,ct),X.side=0,X.needsUpdate=!0,v.renderBufferDirect(q,G,Z,X,P,ct),X.side=2):v.renderBufferDirect(q,G,Z,X,P,ct),P.onAfterRender(v,G,q,Z,X,ct)}function Cn(P,G,q){G.isScene!==!0&&(G=ut);const Z=Lt.get(P),X=g.state.lights,ct=g.state.shadowsArray,Tt=X.state.version,Dt=Mt.getParameters(P,X.state,ct,G,q),St=Mt.getProgramCacheKey(Dt);let Ht=Z.programs;Z.environment=P.isMeshStandardMaterial?G.environment:null,Z.fog=G.fog,Z.envMap=(P.isMeshStandardMaterial?W:R).get(P.envMap||Z.environment),Ht===void 0&&(P.addEventListener("dispose",ft),Ht=new Map,Z.programs=Ht);let Bt=Ht.get(St);if(Bt!==void 0){if(Z.currentProgram===Bt&&Z.lightsStateVersion===Tt)return en(P,Dt),Bt}else Dt.uniforms=Mt.getUniforms(P),P.onBuild(q,Dt,v),P.onBeforeCompile(Dt,v),Bt=Mt.acquireProgram(Dt,St),Ht.set(St,Bt),Z.uniforms=Dt.uniforms;const Vt=Z.uniforms;return(!P.isShaderMaterial&&!P.isRawShaderMaterial||P.clipping===!0)&&(Vt.clippingPlanes=Wt.uniform),en(P,Dt),Z.needsLights=Ot(P),Z.lightsStateVersion=Tt,Z.needsLights&&(Vt.ambientLightColor.value=X.state.ambient,Vt.lightProbe.value=X.state.probe,Vt.directionalLights.value=X.state.directional,Vt.directionalLightShadows.value=X.state.directionalShadow,Vt.spotLights.value=X.state.spot,Vt.spotLightShadows.value=X.state.spotShadow,Vt.rectAreaLights.value=X.state.rectArea,Vt.ltc_1.value=X.state.rectAreaLTC1,Vt.ltc_2.value=X.state.rectAreaLTC2,Vt.pointLights.value=X.state.point,Vt.pointLightShadows.value=X.state.pointShadow,Vt.hemisphereLights.value=X.state.hemi,Vt.directionalShadowMap.value=X.state.directionalShadowMap,Vt.directionalShadowMatrix.value=X.state.directionalShadowMatrix,Vt.spotShadowMap.value=X.state.spotShadowMap,Vt.spotLightMatrix.value=X.state.spotLightMatrix,Vt.spotLightMap.value=X.state.spotLightMap,Vt.pointShadowMap.value=X.state.pointShadowMap,Vt.pointShadowMatrix.value=X.state.pointShadowMatrix),Z.currentProgram=Bt,Z.uniformsList=null,Bt}function Ts(P){if(P.uniformsList===null){const G=P.currentProgram.getUniforms();P.uniformsList=No.seqWithValue(G.seq,P.uniforms)}return P.uniformsList}function en(P,G){const q=Lt.get(P);q.outputColorSpace=G.outputColorSpace,q.batching=G.batching,q.instancing=G.instancing,q.instancingColor=G.instancingColor,q.skinning=G.skinning,q.morphTargets=G.morphTargets,q.morphNormals=G.morphNormals,q.morphColors=G.morphColors,q.morphTargetsCount=G.morphTargetsCount,q.numClippingPlanes=G.numClippingPlanes,q.numIntersection=G.numClipIntersection,q.vertexAlphas=G.vertexAlphas,q.vertexTangents=G.vertexTangents,q.toneMapping=G.toneMapping}function xi(P,G,q,Z,X){G.isScene!==!0&&(G=ut),I.resetTextureUnits();const ct=G.fog,Tt=Z.isMeshStandardMaterial?G.environment:null,Dt=w===null?v.outputColorSpace:w.isXRRenderTarget===!0?w.texture.colorSpace:Ii,St=(Z.isMeshStandardMaterial?W:R).get(Z.envMap||Tt),Ht=Z.vertexColors===!0&&!!q.attributes.color&&q.attributes.color.itemSize===4,Bt=!!q.attributes.tangent&&(!!Z.normalMap||Z.anisotropy>0),Vt=!!q.morphAttributes.position,pe=!!q.morphAttributes.normal,Ze=!!q.morphAttributes.color;let be=0;Z.toneMapped&&(w===null||w.isXRRenderTarget===!0)&&(be=v.toneMapping);const Yn=q.morphAttributes.position||q.morphAttributes.normal||q.morphAttributes.color,me=Yn!==void 0?Yn.length:0,te=Lt.get(Z),Cr=g.state.lights;if($===!0&&(st===!0||P!==E)){const _n=P===E&&Z.id===T;Wt.setState(Z,P,_n)}let Ae=!1;Z.version===te.__version?(te.needsLights&&te.lightsStateVersion!==Cr.state.version||te.outputColorSpace!==Dt||X.isBatchedMesh&&te.batching===!1||!X.isBatchedMesh&&te.batching===!0||X.isInstancedMesh&&te.instancing===!1||!X.isInstancedMesh&&te.instancing===!0||X.isSkinnedMesh&&te.skinning===!1||!X.isSkinnedMesh&&te.skinning===!0||X.isInstancedMesh&&te.instancingColor===!0&&X.instanceColor===null||X.isInstancedMesh&&te.instancingColor===!1&&X.instanceColor!==null||te.envMap!==St||Z.fog===!0&&te.fog!==ct||te.numClippingPlanes!==void 0&&(te.numClippingPlanes!==Wt.numPlanes||te.numIntersection!==Wt.numIntersection)||te.vertexAlphas!==Ht||te.vertexTangents!==Bt||te.morphTargets!==Vt||te.morphNormals!==pe||te.morphColors!==Ze||te.toneMapping!==be||Ct.isWebGL2===!0&&te.morphTargetsCount!==me)&&(Ae=!0):(Ae=!0,te.__version=Z.version);let _i=te.currentProgram;Ae===!0&&(_i=Cn(Z,G,X));let z=!1,mt=!1,It=!1;const ne=_i.getUniforms(),Be=te.uniforms;if(vt.useProgram(_i.program)&&(z=!0,mt=!0,It=!0),Z.id!==T&&(T=Z.id,mt=!0),z||E!==P){ne.setValue(Y,"projectionMatrix",P.projectionMatrix),ne.setValue(Y,"viewMatrix",P.matrixWorldInverse);const _n=ne.map.cameraPosition;_n!==void 0&&_n.setValue(Y,ht.setFromMatrixPosition(P.matrixWorld)),Ct.logarithmicDepthBuffer&&ne.setValue(Y,"logDepthBufFC",2/(Math.log(P.far+1)/Math.LN2)),(Z.isMeshPhongMaterial||Z.isMeshToonMaterial||Z.isMeshLambertMaterial||Z.isMeshBasicMaterial||Z.isMeshStandardMaterial||Z.isShaderMaterial)&&ne.setValue(Y,"isOrthographic",P.isOrthographicCamera===!0),E!==P&&(E=P,mt=!0,It=!0)}if(X.isSkinnedMesh){ne.setOptional(Y,X,"bindMatrix"),ne.setOptional(Y,X,"bindMatrixInverse");const _n=X.skeleton;_n&&(Ct.floatVertexTextures?(_n.boneTexture===null&&_n.computeBoneTexture(),ne.setValue(Y,"boneTexture",_n.boneTexture,I)):console.warn("THREE.WebGLRenderer: SkinnedMesh can only be used with WebGL 2. With WebGL 1 OES_texture_float and vertex textures support is required."))}X.isBatchedMesh&&(ne.setOptional(Y,X,"batchingTexture"),ne.setValue(Y,"batchingTexture",X._matricesTexture,I));const Cs=q.morphAttributes;if((Cs.position!==void 0||Cs.normal!==void 0||Cs.color!==void 0&&Ct.isWebGL2===!0)&&jt.update(X,q,_i),(mt||te.receiveShadow!==X.receiveShadow)&&(te.receiveShadow=X.receiveShadow,ne.setValue(Y,"receiveShadow",X.receiveShadow)),Z.isMeshGouraudMaterial&&Z.envMap!==null&&(Be.envMap.value=St,Be.flipEnvMap.value=St.isCubeTexture&&St.isRenderTargetTexture===!1?-1:1),mt&&(ne.setValue(Y,"toneMappingExposure",v.toneMappingExposure),te.needsLights&&it(Be,It),ct&&Z.fog===!0&&rt.refreshFogUniforms(Be,ct),rt.refreshMaterialUniforms(Be,Z,k,U,at),No.upload(Y,Ts(te),Be,I)),Z.isShaderMaterial&&Z.uniformsNeedUpdate===!0&&(No.upload(Y,Ts(te),Be,I),Z.uniformsNeedUpdate=!1),Z.isSpriteMaterial&&ne.setValue(Y,"center",X.center),ne.setValue(Y,"modelViewMatrix",X.modelViewMatrix),ne.setValue(Y,"normalMatrix",X.normalMatrix),ne.setValue(Y,"modelMatrix",X.matrixWorld),Z.isShaderMaterial||Z.isRawShaderMaterial){const _n=Z.uniformsGroups;for(let br=0,Kp=_n.length;br<Kp;br++)if(Ct.isWebGL2){const Zh=_n[br];Qt.update(Zh,_i),Qt.bind(Zh,_i)}else console.warn("THREE.WebGLRenderer: Uniform Buffer Objects can only be used with WebGL 2.")}return _i}function it(P,G){P.ambientLightColor.needsUpdate=G,P.lightProbe.needsUpdate=G,P.directionalLights.needsUpdate=G,P.directionalLightShadows.needsUpdate=G,P.pointLights.needsUpdate=G,P.pointLightShadows.needsUpdate=G,P.spotLights.needsUpdate=G,P.spotLightShadows.needsUpdate=G,P.rectAreaLights.needsUpdate=G,P.hemisphereLights.needsUpdate=G}function Ot(P){return P.isMeshLambertMaterial||P.isMeshToonMaterial||P.isMeshPhongMaterial||P.isMeshStandardMaterial||P.isShadowMaterial||P.isShaderMaterial&&P.lights===!0}this.getActiveCubeFace=function(){return M},this.getActiveMipmapLevel=function(){return y},this.getRenderTarget=function(){return w},this.setRenderTargetTextures=function(P,G,q){Lt.get(P.texture).__webglTexture=G,Lt.get(P.depthTexture).__webglTexture=q;const Z=Lt.get(P);Z.__hasExternalTextures=!0,Z.__hasExternalTextures&&(Z.__autoAllocateDepthBuffer=q===void 0,Z.__autoAllocateDepthBuffer||wt.has("WEBGL_multisampled_render_to_texture")===!0&&(console.warn("THREE.WebGLRenderer: Render-to-texture extension was disabled because an external texture was provided"),Z.__useRenderToTexture=!1))},this.setRenderTargetFramebuffer=function(P,G){const q=Lt.get(P);q.__webglFramebuffer=G,q.__useDefaultFramebuffer=G===void 0},this.setRenderTarget=function(P,G=0,q=0){w=P,M=G,y=q;let Z=!0,X=null,ct=!1,Tt=!1;if(P){const St=Lt.get(P);St.__useDefaultFramebuffer!==void 0?(vt.bindFramebuffer(Y.FRAMEBUFFER,null),Z=!1):St.__webglFramebuffer===void 0?I.setupRenderTarget(P):St.__hasExternalTextures&&I.rebindTextures(P,Lt.get(P.texture).__webglTexture,Lt.get(P.depthTexture).__webglTexture);const Ht=P.texture;(Ht.isData3DTexture||Ht.isDataArrayTexture||Ht.isCompressedArrayTexture)&&(Tt=!0);const Bt=Lt.get(P).__webglFramebuffer;P.isWebGLCubeRenderTarget?(Array.isArray(Bt[G])?X=Bt[G][q]:X=Bt[G],ct=!0):Ct.isWebGL2&&P.samples>0&&I.useMultisampledRTT(P)===!1?X=Lt.get(P).__webglMultisampledFramebuffer:Array.isArray(Bt)?X=Bt[q]:X=Bt,A.copy(P.viewport),D.copy(P.scissor),b=P.scissorTest}else A.copy(j).multiplyScalar(k).floor(),D.copy(J).multiplyScalar(k).floor(),b=nt;if(vt.bindFramebuffer(Y.FRAMEBUFFER,X)&&Ct.drawBuffers&&Z&&vt.drawBuffers(P,X),vt.viewport(A),vt.scissor(D),vt.setScissorTest(b),ct){const St=Lt.get(P.texture);Y.framebufferTexture2D(Y.FRAMEBUFFER,Y.COLOR_ATTACHMENT0,Y.TEXTURE_CUBE_MAP_POSITIVE_X+G,St.__webglTexture,q)}else if(Tt){const St=Lt.get(P.texture),Ht=G||0;Y.framebufferTextureLayer(Y.FRAMEBUFFER,Y.COLOR_ATTACHMENT0,St.__webglTexture,q||0,Ht)}T=-1},this.readRenderTargetPixels=function(P,G,q,Z,X,ct,Tt){if(!(P&&P.isWebGLRenderTarget)){console.error("THREE.WebGLRenderer.readRenderTargetPixels: renderTarget is not THREE.WebGLRenderTarget.");return}let Dt=Lt.get(P).__webglFramebuffer;if(P.isWebGLCubeRenderTarget&&Tt!==void 0&&(Dt=Dt[Tt]),Dt){vt.bindFramebuffer(Y.FRAMEBUFFER,Dt);try{const St=P.texture,Ht=St.format,Bt=St.type;if(Ht!==1023&&xt.convert(Ht)!==Y.getParameter(Y.IMPLEMENTATION_COLOR_READ_FORMAT)){console.error("THREE.WebGLRenderer.readRenderTargetPixels: renderTarget is not in RGBA or implementation defined format.");return}const Vt=Bt===1016&&(wt.has("EXT_color_buffer_half_float")||Ct.isWebGL2&&wt.has("EXT_color_buffer_float"));if(Bt!==1009&&xt.convert(Bt)!==Y.getParameter(Y.IMPLEMENTATION_COLOR_READ_TYPE)&&!(Bt===1015&&(Ct.isWebGL2||wt.has("OES_texture_float")||wt.has("WEBGL_color_buffer_float")))&&!Vt){console.error("THREE.WebGLRenderer.readRenderTargetPixels: renderTarget is not in UnsignedByteType or implementation defined type.");return}G>=0&&G<=P.width-Z&&q>=0&&q<=P.height-X&&Y.readPixels(G,q,Z,X,xt.convert(Ht),xt.convert(Bt),ct)}finally{const St=w!==null?Lt.get(w).__webglFramebuffer:null;vt.bindFramebuffer(Y.FRAMEBUFFER,St)}}},this.copyFramebufferToTexture=function(P,G,q=0){const Z=Math.pow(2,-q),X=Math.floor(G.image.width*Z),ct=Math.floor(G.image.height*Z);I.setTexture2D(G,0),Y.copyTexSubImage2D(Y.TEXTURE_2D,q,0,0,P.x,P.y,X,ct),vt.unbindTexture()},this.copyTextureToTexture=function(P,G,q,Z=0){const X=G.image.width,ct=G.image.height,Tt=xt.convert(q.format),Dt=xt.convert(q.type);I.setTexture2D(q,0),Y.pixelStorei(Y.UNPACK_FLIP_Y_WEBGL,q.flipY),Y.pixelStorei(Y.UNPACK_PREMULTIPLY_ALPHA_WEBGL,q.premultiplyAlpha),Y.pixelStorei(Y.UNPACK_ALIGNMENT,q.unpackAlignment),G.isDataTexture?Y.texSubImage2D(Y.TEXTURE_2D,Z,P.x,P.y,X,ct,Tt,Dt,G.image.data):G.isCompressedTexture?Y.compressedTexSubImage2D(Y.TEXTURE_2D,Z,P.x,P.y,G.mipmaps[0].width,G.mipmaps[0].height,Tt,G.mipmaps[0].data):Y.texSubImage2D(Y.TEXTURE_2D,Z,P.x,P.y,Tt,Dt,G.image),Z===0&&q.generateMipmaps&&Y.generateMipmap(Y.TEXTURE_2D),vt.unbindTexture()},this.copyTextureToTexture3D=function(P,G,q,Z,X=0){if(v.isWebGL1Renderer){console.warn("THREE.WebGLRenderer.copyTextureToTexture3D: can only be used with WebGL2.");return}const ct=P.max.x-P.min.x+1,Tt=P.max.y-P.min.y+1,Dt=P.max.z-P.min.z+1,St=xt.convert(Z.format),Ht=xt.convert(Z.type);let Bt;if(Z.isData3DTexture)I.setTexture3D(Z,0),Bt=Y.TEXTURE_3D;else if(Z.isDataArrayTexture||Z.isCompressedArrayTexture)I.setTexture2DArray(Z,0),Bt=Y.TEXTURE_2D_ARRAY;else{console.warn("THREE.WebGLRenderer.copyTextureToTexture3D: only supports THREE.DataTexture3D and THREE.DataTexture2DArray.");return}Y.pixelStorei(Y.UNPACK_FLIP_Y_WEBGL,Z.flipY),Y.pixelStorei(Y.UNPACK_PREMULTIPLY_ALPHA_WEBGL,Z.premultiplyAlpha),Y.pixelStorei(Y.UNPACK_ALIGNMENT,Z.unpackAlignment);const Vt=Y.getParameter(Y.UNPACK_ROW_LENGTH),pe=Y.getParameter(Y.UNPACK_IMAGE_HEIGHT),Ze=Y.getParameter(Y.UNPACK_SKIP_PIXELS),be=Y.getParameter(Y.UNPACK_SKIP_ROWS),Yn=Y.getParameter(Y.UNPACK_SKIP_IMAGES),me=q.isCompressedTexture?q.mipmaps[X]:q.image;Y.pixelStorei(Y.UNPACK_ROW_LENGTH,me.width),Y.pixelStorei(Y.UNPACK_IMAGE_HEIGHT,me.height),Y.pixelStorei(Y.UNPACK_SKIP_PIXELS,P.min.x),Y.pixelStorei(Y.UNPACK_SKIP_ROWS,P.min.y),Y.pixelStorei(Y.UNPACK_SKIP_IMAGES,P.min.z),q.isDataTexture||q.isData3DTexture?Y.texSubImage3D(Bt,X,G.x,G.y,G.z,ct,Tt,Dt,St,Ht,me.data):q.isCompressedArrayTexture?(console.warn("THREE.WebGLRenderer.copyTextureToTexture3D: untested support for compressed srcTexture."),Y.compressedTexSubImage3D(Bt,X,G.x,G.y,G.z,ct,Tt,Dt,St,me.data)):Y.texSubImage3D(Bt,X,G.x,G.y,G.z,ct,Tt,Dt,St,Ht,me),Y.pixelStorei(Y.UNPACK_ROW_LENGTH,Vt),Y.pixelStorei(Y.UNPACK_IMAGE_HEIGHT,pe),Y.pixelStorei(Y.UNPACK_SKIP_PIXELS,Ze),Y.pixelStorei(Y.UNPACK_SKIP_ROWS,be),Y.pixelStorei(Y.UNPACK_SKIP_IMAGES,Yn),X===0&&Z.generateMipmaps&&Y.generateMipmap(Bt),vt.unbindTexture()},this.initTexture=function(P){P.isCubeTexture?I.setTextureCube(P,0):P.isData3DTexture?I.setTexture3D(P,0):P.isDataArrayTexture||P.isCompressedArrayTexture?I.setTexture2DArray(P,0):I.setTexture2D(P,0),vt.unbindTexture()},this.resetState=function(){M=0,y=0,w=null,vt.reset(),Ft.reset()},typeof __THREE_DEVTOOLS__<"u"&&__THREE_DEVTOOLS__.dispatchEvent(new CustomEvent("observe",{detail:this}))}get coordinateSystem(){return 2e3}get outputColorSpace(){return this._outputColorSpace}set outputColorSpace(t){this._outputColorSpace=t;const e=this.getContext();e.drawingBufferColorSpace=t===Mh?"display-p3":"srgb",e.unpackColorSpace=ge.workingColorSpace===cc?"display-p3":"srgb"}get outputEncoding(){return console.warn("THREE.WebGLRenderer: Property .outputEncoding has been removed. Use .outputColorSpace instead."),this.outputColorSpace===nn?3001:3e3}set outputEncoding(t){console.warn("THREE.WebGLRenderer: Property .outputEncoding has been removed. Use .outputColorSpace instead."),this.outputColorSpace=t===3001?nn:Ii}get useLegacyLights(){return console.warn("THREE.WebGLRenderer: The property .useLegacyLights has been deprecated. Migrate your lighting according to the following guide: https://discourse.threejs.org/t/updates-to-lighting-in-three-js-r155/53733."),this._useLegacyLights}set useLegacyLights(t){console.warn("THREE.WebGLRenderer: The property .useLegacyLights has been deprecated. Migrate your lighting according to the following guide: https://discourse.threejs.org/t/updates-to-lighting-in-three-js-r155/53733."),this._useLegacyLights=t}}class xv extends Ch{}xv.prototype.isWebGL1Renderer=!0;class Er extends Je{constructor(){super(),this.isScene=!0,this.type="Scene",this.background=null,this.environment=null,this.fog=null,this.backgroundBlurriness=0,this.backgroundIntensity=1,this.overrideMaterial=null,typeof __THREE_DEVTOOLS__<"u"&&__THREE_DEVTOOLS__.dispatchEvent(new CustomEvent("observe",{detail:this}))}copy(t,e){return super.copy(t,e),t.background!==null&&(this.background=t.background.clone()),t.environment!==null&&(this.environment=t.environment.clone()),t.fog!==null&&(this.fog=t.fog.clone()),this.backgroundBlurriness=t.backgroundBlurriness,this.backgroundIntensity=t.backgroundIntensity,t.overrideMaterial!==null&&(this.overrideMaterial=t.overrideMaterial.clone()),this.matrixAutoUpdate=t.matrixAutoUpdate,this}toJSON(t){const e=super.toJSON(t);return this.fog!==null&&(e.object.fog=this.fog.toJSON()),this.backgroundBlurriness>0&&(e.object.backgroundBlurriness=this.backgroundBlurriness),this.backgroundIntensity!==1&&(e.object.backgroundIntensity=this.backgroundIntensity),e}}class Fu extends Fe{constructor(t,e,n,i=1){super(t,e,n),this.isInstancedBufferAttribute=!0,this.meshPerAttribute=i}copy(t){return super.copy(t),this.meshPerAttribute=t.meshPerAttribute,this}toJSON(){const t=super.toJSON();return t.meshPerAttribute=this.meshPerAttribute,t.isInstancedBufferAttribute=!0,t}}const Hs=new re,Uu=new re,Qa=[],Bu=new tn,_v=new re,Ir=new $t,Nr=new ns;class wr extends $t{constructor(t,e,n){super(t,e),this.isInstancedMesh=!0,this.instanceMatrix=new Fu(new Float32Array(n*16),16),this.instanceColor=null,this.count=n,this.boundingBox=null,this.boundingSphere=null;for(let i=0;i<n;i++)this.setMatrixAt(i,_v)}computeBoundingBox(){const t=this.geometry,e=this.count;this.boundingBox===null&&(this.boundingBox=new tn),t.boundingBox===null&&t.computeBoundingBox(),this.boundingBox.makeEmpty();for(let n=0;n<e;n++)this.getMatrixAt(n,Hs),Bu.copy(t.boundingBox).applyMatrix4(Hs),this.boundingBox.union(Bu)}computeBoundingSphere(){const t=this.geometry,e=this.count;this.boundingSphere===null&&(this.boundingSphere=new ns),t.boundingSphere===null&&t.computeBoundingSphere(),this.boundingSphere.makeEmpty();for(let n=0;n<e;n++)this.getMatrixAt(n,Hs),Nr.copy(t.boundingSphere).applyMatrix4(Hs),this.boundingSphere.union(Nr)}copy(t,e){return super.copy(t,e),this.instanceMatrix.copy(t.instanceMatrix),t.instanceColor!==null&&(this.instanceColor=t.instanceColor.clone()),this.count=t.count,t.boundingBox!==null&&(this.boundingBox=t.boundingBox.clone()),t.boundingSphere!==null&&(this.boundingSphere=t.boundingSphere.clone()),this}getColorAt(t,e){e.fromArray(this.instanceColor.array,t*3)}getMatrixAt(t,e){e.fromArray(this.instanceMatrix.array,t*16)}raycast(t,e){const n=this.matrixWorld,i=this.count;if(Ir.geometry=this.geometry,Ir.material=this.material,Ir.material!==void 0&&(this.boundingSphere===null&&this.computeBoundingSphere(),Nr.copy(this.boundingSphere),Nr.applyMatrix4(n),t.ray.intersectsSphere(Nr)!==!1))for(let s=0;s<i;s++){this.getMatrixAt(s,Hs),Uu.multiplyMatrices(n,Hs),Ir.matrixWorld=Uu,Ir.raycast(t,Qa);for(let a=0,o=Qa.length;a<o;a++){const c=Qa[a];c.instanceId=s,c.object=this,e.push(c)}Qa.length=0}}setColorAt(t,e){this.instanceColor===null&&(this.instanceColor=new Fu(new Float32Array(this.instanceMatrix.count*3),3)),e.toArray(this.instanceColor.array,t*3)}setMatrixAt(t,e){e.toArray(this.instanceMatrix.array,t*16)}updateMorphTargets(){}dispose(){this.dispatchEvent({type:"dispose"})}}class di extends pi{constructor(t){super(),this.isLineBasicMaterial=!0,this.type="LineBasicMaterial",this.color=new Kt(16777215),this.map=null,this.linewidth=1,this.linecap="round",this.linejoin="round",this.fog=!0,this.setValues(t)}copy(t){return super.copy(t),this.color.copy(t.color),this.map=t.map,this.linewidth=t.linewidth,this.linecap=t.linecap,this.linejoin=t.linejoin,this.fog=t.fog,this}}const zu=new C,Ou=new C,Gu=new re,Wc=new Eh,to=new ns;class vv extends Je{constructor(t=new we,e=new di){super(),this.isLine=!0,this.type="Line",this.geometry=t,this.material=e,this.updateMorphTargets()}copy(t,e){return super.copy(t,e),this.material=Array.isArray(t.material)?t.material.slice():t.material,this.geometry=t.geometry,this}computeLineDistances(){const t=this.geometry;if(t.index===null){const e=t.attributes.position,n=[0];for(let i=1,s=e.count;i<s;i++)zu.fromBufferAttribute(e,i-1),Ou.fromBufferAttribute(e,i),n[i]=n[i-1],n[i]+=zu.distanceTo(Ou);t.setAttribute("lineDistance",new se(n,1))}else console.warn("THREE.Line.computeLineDistances(): Computation only possible with non-indexed BufferGeometry.");return this}raycast(t,e){const n=this.geometry,i=this.matrixWorld,s=t.params.Line.threshold,a=n.drawRange;if(n.boundingSphere===null&&n.computeBoundingSphere(),to.copy(n.boundingSphere),to.applyMatrix4(i),to.radius+=s,t.ray.intersectsSphere(to)===!1)return;Gu.copy(i).invert(),Wc.copy(t.ray).applyMatrix4(Gu);const o=s/((this.scale.x+this.scale.y+this.scale.z)/3),c=o*o,l=new C,h=new C,u=new C,d=new C,f=this.isLineSegments?2:1,m=n.index,g=n.attributes.position;if(m!==null){const p=Math.max(0,a.start),_=Math.min(m.count,a.start+a.count);for(let v=p,S=_-1;v<S;v+=f){const M=m.getX(v),y=m.getX(v+1);if(l.fromBufferAttribute(g,M),h.fromBufferAttribute(g,y),Wc.distanceSqToSegment(l,h,d,u)>c)continue;d.applyMatrix4(this.matrixWorld);const T=t.ray.origin.distanceTo(d);T<t.near||T>t.far||e.push({distance:T,point:u.clone().applyMatrix4(this.matrixWorld),index:v,face:null,faceIndex:null,object:this})}}else{const p=Math.max(0,a.start),_=Math.min(g.count,a.start+a.count);for(let v=p,S=_-1;v<S;v+=f){if(l.fromBufferAttribute(g,v),h.fromBufferAttribute(g,v+1),Wc.distanceSqToSegment(l,h,d,u)>c)continue;d.applyMatrix4(this.matrixWorld);const y=t.ray.origin.distanceTo(d);y<t.near||y>t.far||e.push({distance:y,point:u.clone().applyMatrix4(this.matrixWorld),index:v,face:null,faceIndex:null,object:this})}}}updateMorphTargets(){const e=this.geometry.morphAttributes,n=Object.keys(e);if(n.length>0){const i=e[n[0]];if(i!==void 0){this.morphTargetInfluences=[],this.morphTargetDictionary={};for(let s=0,a=i.length;s<a;s++){const o=i[s].name||String(s);this.morphTargetInfluences.push(0),this.morphTargetDictionary[o]=s}}}}}const Vu=new C,ku=new C;class ts extends vv{constructor(t,e){super(t,e),this.isLineSegments=!0,this.type="LineSegments"}computeLineDistances(){const t=this.geometry;if(t.index===null){const e=t.attributes.position,n=[];for(let i=0,s=e.count;i<s;i+=2)Vu.fromBufferAttribute(e,i),ku.fromBufferAttribute(e,i+1),n[i]=i===0?0:n[i-1],n[i+1]=n[i]+Vu.distanceTo(ku);t.setAttribute("lineDistance",new se(n,1))}else console.warn("THREE.LineSegments.computeLineDistances(): Computation only possible with non-indexed BufferGeometry.");return this}}class ca extends pi{constructor(t){super(),this.isPointsMaterial=!0,this.type="PointsMaterial",this.color=new Kt(16777215),this.map=null,this.alphaMap=null,this.size=1,this.sizeAttenuation=!0,this.fog=!0,this.setValues(t)}copy(t){return super.copy(t),this.color.copy(t.color),this.map=t.map,this.alphaMap=t.alphaMap,this.size=t.size,this.sizeAttenuation=t.sizeAttenuation,this.fog=t.fog,this}}const Hu=new re,Hl=new Eh,eo=new ns,no=new C;class Xc extends Je{constructor(t=new we,e=new ca){super(),this.isPoints=!0,this.type="Points",this.geometry=t,this.material=e,this.updateMorphTargets()}copy(t,e){return super.copy(t,e),this.material=Array.isArray(t.material)?t.material.slice():t.material,this.geometry=t.geometry,this}raycast(t,e){const n=this.geometry,i=this.matrixWorld,s=t.params.Points.threshold,a=n.drawRange;if(n.boundingSphere===null&&n.computeBoundingSphere(),eo.copy(n.boundingSphere),eo.applyMatrix4(i),eo.radius+=s,t.ray.intersectsSphere(eo)===!1)return;Hu.copy(i).invert(),Hl.copy(t.ray).applyMatrix4(Hu);const o=s/((this.scale.x+this.scale.y+this.scale.z)/3),c=o*o,l=n.index,u=n.attributes.position;if(l!==null){const d=Math.max(0,a.start),f=Math.min(l.count,a.start+a.count);for(let m=d,x=f;m<x;m++){const g=l.getX(m);no.fromBufferAttribute(u,g),Wu(no,g,c,i,t,e,this)}}else{const d=Math.max(0,a.start),f=Math.min(u.count,a.start+a.count);for(let m=d,x=f;m<x;m++)no.fromBufferAttribute(u,m),Wu(no,m,c,i,t,e,this)}}updateMorphTargets(){const e=this.geometry.morphAttributes,n=Object.keys(e);if(n.length>0){const i=e[n[0]];if(i!==void 0){this.morphTargetInfluences=[],this.morphTargetDictionary={};for(let s=0,a=i.length;s<a;s++){const o=i[s].name||String(s);this.morphTargetInfluences.push(0),this.morphTargetDictionary[o]=s}}}}}function Wu(r,t,e,n,i,s,a){const o=Hl.distanceSqToPoint(r);if(o<e){const c=new C;Hl.closestPointToPoint(r,c),c.applyMatrix4(n);const l=i.ray.origin.distanceTo(c);if(l<i.near||l>i.far)return;s.push({distance:l,distanceToRay:Math.sqrt(o),point:c,index:t,face:null,object:a})}}class mi{constructor(){this.type="Curve",this.arcLengthDivisions=200}getPoint(){return console.warn("THREE.Curve: .getPoint() not implemented."),null}getPointAt(t,e){const n=this.getUtoTmapping(t);return this.getPoint(n,e)}getPoints(t=5){const e=[];for(let n=0;n<=t;n++)e.push(this.getPoint(n/t));return e}getSpacedPoints(t=5){const e=[];for(let n=0;n<=t;n++)e.push(this.getPointAt(n/t));return e}getLength(){const t=this.getLengths();return t[t.length-1]}getLengths(t=this.arcLengthDivisions){if(this.cacheArcLengths&&this.cacheArcLengths.length===t+1&&!this.needsUpdate)return this.cacheArcLengths;this.needsUpdate=!1;const e=[];let n,i=this.getPoint(0),s=0;e.push(0);for(let a=1;a<=t;a++)n=this.getPoint(a/t),s+=n.distanceTo(i),e.push(s),i=n;return this.cacheArcLengths=e,e}updateArcLengths(){this.needsUpdate=!0,this.getLengths()}getUtoTmapping(t,e){const n=this.getLengths();let i=0;const s=n.length;let a;e?a=e:a=t*n[s-1];let o=0,c=s-1,l;for(;o<=c;)if(i=Math.floor(o+(c-o)/2),l=n[i]-a,l<0)o=i+1;else if(l>0)c=i-1;else{c=i;break}if(i=c,n[i]===a)return i/(s-1);const h=n[i],d=n[i+1]-h,f=(a-h)/d;return(i+f)/(s-1)}getTangent(t,e){let i=t-1e-4,s=t+1e-4;i<0&&(i=0),s>1&&(s=1);const a=this.getPoint(i),o=this.getPoint(s),c=e||(a.isVector2?new yt:new C);return c.copy(o).sub(a).normalize(),c}getTangentAt(t,e){const n=this.getUtoTmapping(t);return this.getTangent(n,e)}computeFrenetFrames(t,e){const n=new C,i=[],s=[],a=[],o=new C,c=new re;for(let f=0;f<=t;f++){const m=f/t;i[f]=this.getTangentAt(m,new C)}s[0]=new C,a[0]=new C;let l=Number.MAX_VALUE;const h=Math.abs(i[0].x),u=Math.abs(i[0].y),d=Math.abs(i[0].z);h<=l&&(l=h,n.set(1,0,0)),u<=l&&(l=u,n.set(0,1,0)),d<=l&&n.set(0,0,1),o.crossVectors(i[0],n).normalize(),s[0].crossVectors(i[0],o),a[0].crossVectors(i[0],s[0]);for(let f=1;f<=t;f++){if(s[f]=s[f-1].clone(),a[f]=a[f-1].clone(),o.crossVectors(i[f-1],i[f]),o.length()>Number.EPSILON){o.normalize();const m=Math.acos(ke(i[f-1].dot(i[f]),-1,1));s[f].applyMatrix4(c.makeRotationAxis(o,m))}a[f].crossVectors(i[f],s[f])}if(e===!0){let f=Math.acos(ke(s[0].dot(s[t]),-1,1));f/=t,i[0].dot(o.crossVectors(s[0],s[t]))>0&&(f=-f);for(let m=1;m<=t;m++)s[m].applyMatrix4(c.makeRotationAxis(i[m],f*m)),a[m].crossVectors(i[m],s[m])}return{tangents:i,normals:s,binormals:a}}clone(){return new this.constructor().copy(this)}copy(t){return this.arcLengthDivisions=t.arcLengthDivisions,this}toJSON(){const t={metadata:{version:4.6,type:"Curve",generator:"Curve.toJSON"}};return t.arcLengthDivisions=this.arcLengthDivisions,t.type=this.type,t}fromJSON(t){return this.arcLengthDivisions=t.arcLengthDivisions,this}}class bh extends mi{constructor(t=0,e=0,n=1,i=1,s=0,a=Math.PI*2,o=!1,c=0){super(),this.isEllipseCurve=!0,this.type="EllipseCurve",this.aX=t,this.aY=e,this.xRadius=n,this.yRadius=i,this.aStartAngle=s,this.aEndAngle=a,this.aClockwise=o,this.aRotation=c}getPoint(t,e){const n=e||new yt,i=Math.PI*2;let s=this.aEndAngle-this.aStartAngle;const a=Math.abs(s)<Number.EPSILON;for(;s<0;)s+=i;for(;s>i;)s-=i;s<Number.EPSILON&&(a?s=0:s=i),this.aClockwise===!0&&!a&&(s===i?s=-i:s=s-i);const o=this.aStartAngle+t*s;let c=this.aX+this.xRadius*Math.cos(o),l=this.aY+this.yRadius*Math.sin(o);if(this.aRotation!==0){const h=Math.cos(this.aRotation),u=Math.sin(this.aRotation),d=c-this.aX,f=l-this.aY;c=d*h-f*u+this.aX,l=d*u+f*h+this.aY}return n.set(c,l)}copy(t){return super.copy(t),this.aX=t.aX,this.aY=t.aY,this.xRadius=t.xRadius,this.yRadius=t.yRadius,this.aStartAngle=t.aStartAngle,this.aEndAngle=t.aEndAngle,this.aClockwise=t.aClockwise,this.aRotation=t.aRotation,this}toJSON(){const t=super.toJSON();return t.aX=this.aX,t.aY=this.aY,t.xRadius=this.xRadius,t.yRadius=this.yRadius,t.aStartAngle=this.aStartAngle,t.aEndAngle=this.aEndAngle,t.aClockwise=this.aClockwise,t.aRotation=this.aRotation,t}fromJSON(t){return super.fromJSON(t),this.aX=t.aX,this.aY=t.aY,this.xRadius=t.xRadius,this.yRadius=t.yRadius,this.aStartAngle=t.aStartAngle,this.aEndAngle=t.aEndAngle,this.aClockwise=t.aClockwise,this.aRotation=t.aRotation,this}}class Sv extends bh{constructor(t,e,n,i,s,a){super(t,e,n,n,i,s,a),this.isArcCurve=!0,this.type="ArcCurve"}}function Rh(){let r=0,t=0,e=0,n=0;function i(s,a,o,c){r=s,t=o,e=-3*s+3*a-2*o-c,n=2*s-2*a+o+c}return{initCatmullRom:function(s,a,o,c,l){i(a,o,l*(o-s),l*(c-a))},initNonuniformCatmullRom:function(s,a,o,c,l,h,u){let d=(a-s)/l-(o-s)/(l+h)+(o-a)/h,f=(o-a)/h-(c-a)/(h+u)+(c-o)/u;d*=h,f*=h,i(a,o,d,f)},calc:function(s){const a=s*s,o=a*s;return r+t*s+e*a+n*o}}}const io=new C,Yc=new Rh,qc=new Rh,Zc=new Rh;class Wf extends mi{constructor(t=[],e=!1,n="centripetal",i=.5){super(),this.isCatmullRomCurve3=!0,this.type="CatmullRomCurve3",this.points=t,this.closed=e,this.curveType=n,this.tension=i}getPoint(t,e=new C){const n=e,i=this.points,s=i.length,a=(s-(this.closed?0:1))*t;let o=Math.floor(a),c=a-o;this.closed?o+=o>0?0:(Math.floor(Math.abs(o)/s)+1)*s:c===0&&o===s-1&&(o=s-2,c=1);let l,h;this.closed||o>0?l=i[(o-1)%s]:(io.subVectors(i[0],i[1]).add(i[0]),l=io);const u=i[o%s],d=i[(o+1)%s];if(this.closed||o+2<s?h=i[(o+2)%s]:(io.subVectors(i[s-1],i[s-2]).add(i[s-1]),h=io),this.curveType==="centripetal"||this.curveType==="chordal"){const f=this.curveType==="chordal"?.5:.25;let m=Math.pow(l.distanceToSquared(u),f),x=Math.pow(u.distanceToSquared(d),f),g=Math.pow(d.distanceToSquared(h),f);x<1e-4&&(x=1),m<1e-4&&(m=x),g<1e-4&&(g=x),Yc.initNonuniformCatmullRom(l.x,u.x,d.x,h.x,m,x,g),qc.initNonuniformCatmullRom(l.y,u.y,d.y,h.y,m,x,g),Zc.initNonuniformCatmullRom(l.z,u.z,d.z,h.z,m,x,g)}else this.curveType==="catmullrom"&&(Yc.initCatmullRom(l.x,u.x,d.x,h.x,this.tension),qc.initCatmullRom(l.y,u.y,d.y,h.y,this.tension),Zc.initCatmullRom(l.z,u.z,d.z,h.z,this.tension));return n.set(Yc.calc(c),qc.calc(c),Zc.calc(c)),n}copy(t){super.copy(t),this.points=[];for(let e=0,n=t.points.length;e<n;e++){const i=t.points[e];this.points.push(i.clone())}return this.closed=t.closed,this.curveType=t.curveType,this.tension=t.tension,this}toJSON(){const t=super.toJSON();t.points=[];for(let e=0,n=this.points.length;e<n;e++){const i=this.points[e];t.points.push(i.toArray())}return t.closed=this.closed,t.curveType=this.curveType,t.tension=this.tension,t}fromJSON(t){super.fromJSON(t),this.points=[];for(let e=0,n=t.points.length;e<n;e++){const i=t.points[e];this.points.push(new C().fromArray(i))}return this.closed=t.closed,this.curveType=t.curveType,this.tension=t.tension,this}}function Xu(r,t,e,n,i){const s=(n-t)*.5,a=(i-e)*.5,o=r*r,c=r*o;return(2*e-2*n+s+a)*c+(-3*e+3*n-2*s-a)*o+s*r+e}function Mv(r,t){const e=1-r;return e*e*t}function yv(r,t){return 2*(1-r)*r*t}function Ev(r,t){return r*r*t}function _a(r,t,e,n){return Mv(r,t)+yv(r,e)+Ev(r,n)}function wv(r,t){const e=1-r;return e*e*e*t}function Av(r,t){const e=1-r;return 3*e*e*r*t}function Tv(r,t){return 3*(1-r)*r*r*t}function Cv(r,t){return r*r*r*t}function va(r,t,e,n,i){return wv(r,t)+Av(r,e)+Tv(r,n)+Cv(r,i)}class Xf extends mi{constructor(t=new yt,e=new yt,n=new yt,i=new yt){super(),this.isCubicBezierCurve=!0,this.type="CubicBezierCurve",this.v0=t,this.v1=e,this.v2=n,this.v3=i}getPoint(t,e=new yt){const n=e,i=this.v0,s=this.v1,a=this.v2,o=this.v3;return n.set(va(t,i.x,s.x,a.x,o.x),va(t,i.y,s.y,a.y,o.y)),n}copy(t){return super.copy(t),this.v0.copy(t.v0),this.v1.copy(t.v1),this.v2.copy(t.v2),this.v3.copy(t.v3),this}toJSON(){const t=super.toJSON();return t.v0=this.v0.toArray(),t.v1=this.v1.toArray(),t.v2=this.v2.toArray(),t.v3=this.v3.toArray(),t}fromJSON(t){return super.fromJSON(t),this.v0.fromArray(t.v0),this.v1.fromArray(t.v1),this.v2.fromArray(t.v2),this.v3.fromArray(t.v3),this}}class bv extends mi{constructor(t=new C,e=new C,n=new C,i=new C){super(),this.isCubicBezierCurve3=!0,this.type="CubicBezierCurve3",this.v0=t,this.v1=e,this.v2=n,this.v3=i}getPoint(t,e=new C){const n=e,i=this.v0,s=this.v1,a=this.v2,o=this.v3;return n.set(va(t,i.x,s.x,a.x,o.x),va(t,i.y,s.y,a.y,o.y),va(t,i.z,s.z,a.z,o.z)),n}copy(t){return super.copy(t),this.v0.copy(t.v0),this.v1.copy(t.v1),this.v2.copy(t.v2),this.v3.copy(t.v3),this}toJSON(){const t=super.toJSON();return t.v0=this.v0.toArray(),t.v1=this.v1.toArray(),t.v2=this.v2.toArray(),t.v3=this.v3.toArray(),t}fromJSON(t){return super.fromJSON(t),this.v0.fromArray(t.v0),this.v1.fromArray(t.v1),this.v2.fromArray(t.v2),this.v3.fromArray(t.v3),this}}class Yf extends mi{constructor(t=new yt,e=new yt){super(),this.isLineCurve=!0,this.type="LineCurve",this.v1=t,this.v2=e}getPoint(t,e=new yt){const n=e;return t===1?n.copy(this.v2):(n.copy(this.v2).sub(this.v1),n.multiplyScalar(t).add(this.v1)),n}getPointAt(t,e){return this.getPoint(t,e)}getTangent(t,e=new yt){return e.subVectors(this.v2,this.v1).normalize()}getTangentAt(t,e){return this.getTangent(t,e)}copy(t){return super.copy(t),this.v1.copy(t.v1),this.v2.copy(t.v2),this}toJSON(){const t=super.toJSON();return t.v1=this.v1.toArray(),t.v2=this.v2.toArray(),t}fromJSON(t){return super.fromJSON(t),this.v1.fromArray(t.v1),this.v2.fromArray(t.v2),this}}class Rv extends mi{constructor(t=new C,e=new C){super(),this.isLineCurve3=!0,this.type="LineCurve3",this.v1=t,this.v2=e}getPoint(t,e=new C){const n=e;return t===1?n.copy(this.v2):(n.copy(this.v2).sub(this.v1),n.multiplyScalar(t).add(this.v1)),n}getPointAt(t,e){return this.getPoint(t,e)}getTangent(t,e=new C){return e.subVectors(this.v2,this.v1).normalize()}getTangentAt(t,e){return this.getTangent(t,e)}copy(t){return super.copy(t),this.v1.copy(t.v1),this.v2.copy(t.v2),this}toJSON(){const t=super.toJSON();return t.v1=this.v1.toArray(),t.v2=this.v2.toArray(),t}fromJSON(t){return super.fromJSON(t),this.v1.fromArray(t.v1),this.v2.fromArray(t.v2),this}}class qf extends mi{constructor(t=new yt,e=new yt,n=new yt){super(),this.isQuadraticBezierCurve=!0,this.type="QuadraticBezierCurve",this.v0=t,this.v1=e,this.v2=n}getPoint(t,e=new yt){const n=e,i=this.v0,s=this.v1,a=this.v2;return n.set(_a(t,i.x,s.x,a.x),_a(t,i.y,s.y,a.y)),n}copy(t){return super.copy(t),this.v0.copy(t.v0),this.v1.copy(t.v1),this.v2.copy(t.v2),this}toJSON(){const t=super.toJSON();return t.v0=this.v0.toArray(),t.v1=this.v1.toArray(),t.v2=this.v2.toArray(),t}fromJSON(t){return super.fromJSON(t),this.v0.fromArray(t.v0),this.v1.fromArray(t.v1),this.v2.fromArray(t.v2),this}}class Zf extends mi{constructor(t=new C,e=new C,n=new C){super(),this.isQuadraticBezierCurve3=!0,this.type="QuadraticBezierCurve3",this.v0=t,this.v1=e,this.v2=n}getPoint(t,e=new C){const n=e,i=this.v0,s=this.v1,a=this.v2;return n.set(_a(t,i.x,s.x,a.x),_a(t,i.y,s.y,a.y),_a(t,i.z,s.z,a.z)),n}copy(t){return super.copy(t),this.v0.copy(t.v0),this.v1.copy(t.v1),this.v2.copy(t.v2),this}toJSON(){const t=super.toJSON();return t.v0=this.v0.toArray(),t.v1=this.v1.toArray(),t.v2=this.v2.toArray(),t}fromJSON(t){return super.fromJSON(t),this.v0.fromArray(t.v0),this.v1.fromArray(t.v1),this.v2.fromArray(t.v2),this}}class jf extends mi{constructor(t=[]){super(),this.isSplineCurve=!0,this.type="SplineCurve",this.points=t}getPoint(t,e=new yt){const n=e,i=this.points,s=(i.length-1)*t,a=Math.floor(s),o=s-a,c=i[a===0?a:a-1],l=i[a],h=i[a>i.length-2?i.length-1:a+1],u=i[a>i.length-3?i.length-1:a+2];return n.set(Xu(o,c.x,l.x,h.x,u.x),Xu(o,c.y,l.y,h.y,u.y)),n}copy(t){super.copy(t),this.points=[];for(let e=0,n=t.points.length;e<n;e++){const i=t.points[e];this.points.push(i.clone())}return this}toJSON(){const t=super.toJSON();t.points=[];for(let e=0,n=this.points.length;e<n;e++){const i=this.points[e];t.points.push(i.toArray())}return t}fromJSON(t){super.fromJSON(t),this.points=[];for(let e=0,n=t.points.length;e<n;e++){const i=t.points[e];this.points.push(new yt().fromArray(i))}return this}}var Wl=Object.freeze({__proto__:null,ArcCurve:Sv,CatmullRomCurve3:Wf,CubicBezierCurve:Xf,CubicBezierCurve3:bv,EllipseCurve:bh,LineCurve:Yf,LineCurve3:Rv,QuadraticBezierCurve:qf,QuadraticBezierCurve3:Zf,SplineCurve:jf});class Pv extends mi{constructor(){super(),this.type="CurvePath",this.curves=[],this.autoClose=!1}add(t){this.curves.push(t)}closePath(){const t=this.curves[0].getPoint(0),e=this.curves[this.curves.length-1].getPoint(1);if(!t.equals(e)){const n=t.isVector2===!0?"LineCurve":"LineCurve3";this.curves.push(new Wl[n](e,t))}return this}getPoint(t,e){const n=t*this.getLength(),i=this.getCurveLengths();let s=0;for(;s<i.length;){if(i[s]>=n){const a=i[s]-n,o=this.curves[s],c=o.getLength(),l=c===0?0:1-a/c;return o.getPointAt(l,e)}s++}return null}getLength(){const t=this.getCurveLengths();return t[t.length-1]}updateArcLengths(){this.needsUpdate=!0,this.cacheLengths=null,this.getCurveLengths()}getCurveLengths(){if(this.cacheLengths&&this.cacheLengths.length===this.curves.length)return this.cacheLengths;const t=[];let e=0;for(let n=0,i=this.curves.length;n<i;n++)e+=this.curves[n].getLength(),t.push(e);return this.cacheLengths=t,t}getSpacedPoints(t=40){const e=[];for(let n=0;n<=t;n++)e.push(this.getPoint(n/t));return this.autoClose&&e.push(e[0]),e}getPoints(t=12){const e=[];let n;for(let i=0,s=this.curves;i<s.length;i++){const a=s[i],o=a.isEllipseCurve?t*2:a.isLineCurve||a.isLineCurve3?1:a.isSplineCurve?t*a.points.length:t,c=a.getPoints(o);for(let l=0;l<c.length;l++){const h=c[l];n&&n.equals(h)||(e.push(h),n=h)}}return this.autoClose&&e.length>1&&!e[e.length-1].equals(e[0])&&e.push(e[0]),e}copy(t){super.copy(t),this.curves=[];for(let e=0,n=t.curves.length;e<n;e++){const i=t.curves[e];this.curves.push(i.clone())}return this.autoClose=t.autoClose,this}toJSON(){const t=super.toJSON();t.autoClose=this.autoClose,t.curves=[];for(let e=0,n=this.curves.length;e<n;e++){const i=this.curves[e];t.curves.push(i.toJSON())}return t}fromJSON(t){super.fromJSON(t),this.autoClose=t.autoClose,this.curves=[];for(let e=0,n=t.curves.length;e<n;e++){const i=t.curves[e];this.curves.push(new Wl[i.type]().fromJSON(i))}return this}}class Lv extends Pv{constructor(t){super(),this.type="Path",this.currentPoint=new yt,t&&this.setFromPoints(t)}setFromPoints(t){this.moveTo(t[0].x,t[0].y);for(let e=1,n=t.length;e<n;e++)this.lineTo(t[e].x,t[e].y);return this}moveTo(t,e){return this.currentPoint.set(t,e),this}lineTo(t,e){const n=new Yf(this.currentPoint.clone(),new yt(t,e));return this.curves.push(n),this.currentPoint.set(t,e),this}quadraticCurveTo(t,e,n,i){const s=new qf(this.currentPoint.clone(),new yt(t,e),new yt(n,i));return this.curves.push(s),this.currentPoint.set(n,i),this}bezierCurveTo(t,e,n,i,s,a){const o=new Xf(this.currentPoint.clone(),new yt(t,e),new yt(n,i),new yt(s,a));return this.curves.push(o),this.currentPoint.set(s,a),this}splineThru(t){const e=[this.currentPoint.clone()].concat(t),n=new jf(e);return this.curves.push(n),this.currentPoint.copy(t[t.length-1]),this}arc(t,e,n,i,s,a){const o=this.currentPoint.x,c=this.currentPoint.y;return this.absarc(t+o,e+c,n,i,s,a),this}absarc(t,e,n,i,s,a){return this.absellipse(t,e,n,n,i,s,a),this}ellipse(t,e,n,i,s,a,o,c){const l=this.currentPoint.x,h=this.currentPoint.y;return this.absellipse(t+l,e+h,n,i,s,a,o,c),this}absellipse(t,e,n,i,s,a,o,c){const l=new bh(t,e,n,i,s,a,o,c);if(this.curves.length>0){const u=l.getPoint(0);u.equals(this.currentPoint)||this.lineTo(u.x,u.y)}this.curves.push(l);const h=l.getPoint(1);return this.currentPoint.copy(h),this}copy(t){return super.copy(t),this.currentPoint.copy(t.currentPoint),this}toJSON(){const t=super.toJSON();return t.currentPoint=this.currentPoint.toArray(),t}fromJSON(t){return super.fromJSON(t),this.currentPoint.fromArray(t.currentPoint),this}}class Ph extends we{constructor(t=[new yt(0,-.5),new yt(.5,0),new yt(0,.5)],e=12,n=0,i=Math.PI*2){super(),this.type="LatheGeometry",this.parameters={points:t,segments:e,phiStart:n,phiLength:i},e=Math.floor(e),i=ke(i,0,Math.PI*2);const s=[],a=[],o=[],c=[],l=[],h=1/e,u=new C,d=new yt,f=new C,m=new C,x=new C;let g=0,p=0;for(let _=0;_<=t.length-1;_++)switch(_){case 0:g=t[_+1].x-t[_].x,p=t[_+1].y-t[_].y,f.x=p*1,f.y=-g,f.z=p*0,x.copy(f),f.normalize(),c.push(f.x,f.y,f.z);break;case t.length-1:c.push(x.x,x.y,x.z);break;default:g=t[_+1].x-t[_].x,p=t[_+1].y-t[_].y,f.x=p*1,f.y=-g,f.z=p*0,m.copy(f),f.x+=x.x,f.y+=x.y,f.z+=x.z,f.normalize(),c.push(f.x,f.y,f.z),x.copy(m)}for(let _=0;_<=e;_++){const v=n+_*h*i,S=Math.sin(v),M=Math.cos(v);for(let y=0;y<=t.length-1;y++){u.x=t[y].x*S,u.y=t[y].y,u.z=t[y].x*M,a.push(u.x,u.y,u.z),d.x=_/e,d.y=y/(t.length-1),o.push(d.x,d.y);const w=c[3*y+0]*S,T=c[3*y+1],E=c[3*y+0]*M;l.push(w,T,E)}}for(let _=0;_<e;_++)for(let v=0;v<t.length-1;v++){const S=v+_*t.length,M=S,y=S+t.length,w=S+t.length+1,T=S+1;s.push(M,y,T),s.push(w,T,y)}this.setIndex(s),this.setAttribute("position",new se(a,3)),this.setAttribute("uv",new se(o,2)),this.setAttribute("normal",new se(l,3))}copy(t){return super.copy(t),this.parameters=Object.assign({},t.parameters),this}static fromJSON(t){return new Ph(t.points,t.segments,t.phiStart,t.phiLength)}}class ur extends Ph{constructor(t=1,e=1,n=4,i=8){const s=new Lv;s.absarc(0,-e/2,t,Math.PI*1.5,0),s.absarc(0,e/2,t,0,Math.PI*.5),super(s.getPoints(n),i),this.type="CapsuleGeometry",this.parameters={radius:t,length:e,capSegments:n,radialSegments:i}}static fromJSON(t){return new ur(t.radius,t.length,t.capSegments,t.radialSegments)}}class is extends we{constructor(t=1,e=1,n=1,i=32,s=1,a=!1,o=0,c=Math.PI*2){super(),this.type="CylinderGeometry",this.parameters={radiusTop:t,radiusBottom:e,height:n,radialSegments:i,heightSegments:s,openEnded:a,thetaStart:o,thetaLength:c};const l=this;i=Math.floor(i),s=Math.floor(s);const h=[],u=[],d=[],f=[];let m=0;const x=[],g=n/2;let p=0;_(),a===!1&&(t>0&&v(!0),e>0&&v(!1)),this.setIndex(h),this.setAttribute("position",new se(u,3)),this.setAttribute("normal",new se(d,3)),this.setAttribute("uv",new se(f,2));function _(){const S=new C,M=new C;let y=0;const w=(e-t)/n;for(let T=0;T<=s;T++){const E=[],A=T/s,D=A*(e-t)+t;for(let b=0;b<=i;b++){const F=b/i,L=F*c+o,N=Math.sin(L),U=Math.cos(L);M.x=D*N,M.y=-A*n+g,M.z=D*U,u.push(M.x,M.y,M.z),S.set(N,w,U).normalize(),d.push(S.x,S.y,S.z),f.push(F,1-A),E.push(m++)}x.push(E)}for(let T=0;T<i;T++)for(let E=0;E<s;E++){const A=x[E][T],D=x[E+1][T],b=x[E+1][T+1],F=x[E][T+1];h.push(A,D,F),h.push(D,b,F),y+=6}l.addGroup(p,y,0),p+=y}function v(S){const M=m,y=new yt,w=new C;let T=0;const E=S===!0?t:e,A=S===!0?1:-1;for(let b=1;b<=i;b++)u.push(0,g*A,0),d.push(0,A,0),f.push(.5,.5),m++;const D=m;for(let b=0;b<=i;b++){const L=b/i*c+o,N=Math.cos(L),U=Math.sin(L);w.x=E*U,w.y=g*A,w.z=E*N,u.push(w.x,w.y,w.z),d.push(0,A,0),y.x=N*.5+.5,y.y=U*.5*A+.5,f.push(y.x,y.y),m++}for(let b=0;b<i;b++){const F=M+b,L=D+b;S===!0?h.push(L,L+1,F):h.push(L+1,L,F),T+=3}l.addGroup(p,T,S===!0?1:2),p+=T}}copy(t){return super.copy(t),this.parameters=Object.assign({},t.parameters),this}static fromJSON(t){return new is(t.radiusTop,t.radiusBottom,t.height,t.radialSegments,t.heightSegments,t.openEnded,t.thetaStart,t.thetaLength)}}class Lh extends we{constructor(t=.5,e=1,n=32,i=1,s=0,a=Math.PI*2){super(),this.type="RingGeometry",this.parameters={innerRadius:t,outerRadius:e,thetaSegments:n,phiSegments:i,thetaStart:s,thetaLength:a},n=Math.max(3,n),i=Math.max(1,i);const o=[],c=[],l=[],h=[];let u=t;const d=(e-t)/i,f=new C,m=new yt;for(let x=0;x<=i;x++){for(let g=0;g<=n;g++){const p=s+g/n*a;f.x=u*Math.cos(p),f.y=u*Math.sin(p),c.push(f.x,f.y,f.z),l.push(0,0,1),m.x=(f.x/e+1)/2,m.y=(f.y/e+1)/2,h.push(m.x,m.y)}u+=d}for(let x=0;x<i;x++){const g=x*(n+1);for(let p=0;p<n;p++){const _=p+g,v=_,S=_+n+1,M=_+n+2,y=_+1;o.push(v,S,y),o.push(S,M,y)}}this.setIndex(o),this.setAttribute("position",new se(c,3)),this.setAttribute("normal",new se(l,3)),this.setAttribute("uv",new se(h,2))}copy(t){return super.copy(t),this.parameters=Object.assign({},t.parameters),this}static fromJSON(t){return new Lh(t.innerRadius,t.outerRadius,t.thetaSegments,t.phiSegments,t.thetaStart,t.thetaLength)}}class ti extends we{constructor(t=1,e=32,n=16,i=0,s=Math.PI*2,a=0,o=Math.PI){super(),this.type="SphereGeometry",this.parameters={radius:t,widthSegments:e,heightSegments:n,phiStart:i,phiLength:s,thetaStart:a,thetaLength:o},e=Math.max(3,Math.floor(e)),n=Math.max(2,Math.floor(n));const c=Math.min(a+o,Math.PI);let l=0;const h=[],u=new C,d=new C,f=[],m=[],x=[],g=[];for(let p=0;p<=n;p++){const _=[],v=p/n;let S=0;p===0&&a===0?S=.5/e:p===n&&c===Math.PI&&(S=-.5/e);for(let M=0;M<=e;M++){const y=M/e;u.x=-t*Math.cos(i+y*s)*Math.sin(a+v*o),u.y=t*Math.cos(a+v*o),u.z=t*Math.sin(i+y*s)*Math.sin(a+v*o),m.push(u.x,u.y,u.z),d.copy(u).normalize(),x.push(d.x,d.y,d.z),g.push(y+S,1-v),_.push(l++)}h.push(_)}for(let p=0;p<n;p++)for(let _=0;_<e;_++){const v=h[p][_+1],S=h[p][_],M=h[p+1][_],y=h[p+1][_+1];(p!==0||a>0)&&f.push(v,S,y),(p!==n-1||c<Math.PI)&&f.push(S,M,y)}this.setIndex(f),this.setAttribute("position",new se(m,3)),this.setAttribute("normal",new se(x,3)),this.setAttribute("uv",new se(g,2))}copy(t){return super.copy(t),this.parameters=Object.assign({},t.parameters),this}static fromJSON(t){return new ti(t.radius,t.widthSegments,t.heightSegments,t.phiStart,t.phiLength,t.thetaStart,t.thetaLength)}}class Dh extends we{constructor(t=1,e=.4,n=12,i=48,s=Math.PI*2){super(),this.type="TorusGeometry",this.parameters={radius:t,tube:e,radialSegments:n,tubularSegments:i,arc:s},n=Math.floor(n),i=Math.floor(i);const a=[],o=[],c=[],l=[],h=new C,u=new C,d=new C;for(let f=0;f<=n;f++)for(let m=0;m<=i;m++){const x=m/i*s,g=f/n*Math.PI*2;u.x=(t+e*Math.cos(g))*Math.cos(x),u.y=(t+e*Math.cos(g))*Math.sin(x),u.z=e*Math.sin(g),o.push(u.x,u.y,u.z),h.x=t*Math.cos(x),h.y=t*Math.sin(x),d.subVectors(u,h).normalize(),c.push(d.x,d.y,d.z),l.push(m/i),l.push(f/n)}for(let f=1;f<=n;f++)for(let m=1;m<=i;m++){const x=(i+1)*f+m-1,g=(i+1)*(f-1)+m-1,p=(i+1)*(f-1)+m,_=(i+1)*f+m;a.push(x,g,_),a.push(g,p,_)}this.setIndex(a),this.setAttribute("position",new se(o,3)),this.setAttribute("normal",new se(c,3)),this.setAttribute("uv",new se(l,2))}copy(t){return super.copy(t),this.parameters=Object.assign({},t.parameters),this}static fromJSON(t){return new Dh(t.radius,t.tube,t.radialSegments,t.tubularSegments,t.arc)}}class Ih extends we{constructor(t=new Zf(new C(-1,-1,0),new C(-1,1,0),new C(1,1,0)),e=64,n=1,i=8,s=!1){super(),this.type="TubeGeometry",this.parameters={path:t,tubularSegments:e,radius:n,radialSegments:i,closed:s};const a=t.computeFrenetFrames(e,s);this.tangents=a.tangents,this.normals=a.normals,this.binormals=a.binormals;const o=new C,c=new C,l=new yt;let h=new C;const u=[],d=[],f=[],m=[];x(),this.setIndex(m),this.setAttribute("position",new se(u,3)),this.setAttribute("normal",new se(d,3)),this.setAttribute("uv",new se(f,2));function x(){for(let v=0;v<e;v++)g(v);g(s===!1?e:0),_(),p()}function g(v){h=t.getPointAt(v/e,h);const S=a.normals[v],M=a.binormals[v];for(let y=0;y<=i;y++){const w=y/i*Math.PI*2,T=Math.sin(w),E=-Math.cos(w);c.x=E*S.x+T*M.x,c.y=E*S.y+T*M.y,c.z=E*S.z+T*M.z,c.normalize(),d.push(c.x,c.y,c.z),o.x=h.x+n*c.x,o.y=h.y+n*c.y,o.z=h.z+n*c.z,u.push(o.x,o.y,o.z)}}function p(){for(let v=1;v<=e;v++)for(let S=1;S<=i;S++){const M=(i+1)*(v-1)+(S-1),y=(i+1)*v+(S-1),w=(i+1)*v+S,T=(i+1)*(v-1)+S;m.push(M,y,T),m.push(y,w,T)}}function _(){for(let v=0;v<=e;v++)for(let S=0;S<=i;S++)l.x=v/e,l.y=S/i,f.push(l.x,l.y)}}copy(t){return super.copy(t),this.parameters=Object.assign({},t.parameters),this}toJSON(){const t=super.toJSON();return t.path=this.parameters.path.toJSON(),t}static fromJSON(t){return new Ih(new Wl[t.path.type]().fromJSON(t.path),t.tubularSegments,t.radius,t.radialSegments,t.closed)}}class Mn extends pi{constructor(t){super(),this.isMeshStandardMaterial=!0,this.defines={STANDARD:""},this.type="MeshStandardMaterial",this.color=new Kt(16777215),this.roughness=1,this.metalness=0,this.map=null,this.lightMap=null,this.lightMapIntensity=1,this.aoMap=null,this.aoMapIntensity=1,this.emissive=new Kt(0),this.emissiveIntensity=1,this.emissiveMap=null,this.bumpMap=null,this.bumpScale=1,this.normalMap=null,this.normalMapType=0,this.normalScale=new yt(1,1),this.displacementMap=null,this.displacementScale=1,this.displacementBias=0,this.roughnessMap=null,this.metalnessMap=null,this.alphaMap=null,this.envMap=null,this.envMapIntensity=1,this.wireframe=!1,this.wireframeLinewidth=1,this.wireframeLinecap="round",this.wireframeLinejoin="round",this.flatShading=!1,this.fog=!0,this.setValues(t)}copy(t){return super.copy(t),this.defines={STANDARD:""},this.color.copy(t.color),this.roughness=t.roughness,this.metalness=t.metalness,this.map=t.map,this.lightMap=t.lightMap,this.lightMapIntensity=t.lightMapIntensity,this.aoMap=t.aoMap,this.aoMapIntensity=t.aoMapIntensity,this.emissive.copy(t.emissive),this.emissiveMap=t.emissiveMap,this.emissiveIntensity=t.emissiveIntensity,this.bumpMap=t.bumpMap,this.bumpScale=t.bumpScale,this.normalMap=t.normalMap,this.normalMapType=t.normalMapType,this.normalScale.copy(t.normalScale),this.displacementMap=t.displacementMap,this.displacementScale=t.displacementScale,this.displacementBias=t.displacementBias,this.roughnessMap=t.roughnessMap,this.metalnessMap=t.metalnessMap,this.alphaMap=t.alphaMap,this.envMap=t.envMap,this.envMapIntensity=t.envMapIntensity,this.wireframe=t.wireframe,this.wireframeLinewidth=t.wireframeLinewidth,this.wireframeLinecap=t.wireframeLinecap,this.wireframeLinejoin=t.wireframeLinejoin,this.flatShading=t.flatShading,this.fog=t.fog,this}}class Dv extends pi{constructor(t){super(),this.isMeshPhongMaterial=!0,this.type="MeshPhongMaterial",this.color=new Kt(16777215),this.specular=new Kt(1118481),this.shininess=30,this.map=null,this.lightMap=null,this.lightMapIntensity=1,this.aoMap=null,this.aoMapIntensity=1,this.emissive=new Kt(0),this.emissiveIntensity=1,this.emissiveMap=null,this.bumpMap=null,this.bumpScale=1,this.normalMap=null,this.normalMapType=0,this.normalScale=new yt(1,1),this.displacementMap=null,this.displacementScale=1,this.displacementBias=0,this.specularMap=null,this.alphaMap=null,this.envMap=null,this.combine=0,this.reflectivity=1,this.refractionRatio=.98,this.wireframe=!1,this.wireframeLinewidth=1,this.wireframeLinecap="round",this.wireframeLinejoin="round",this.flatShading=!1,this.fog=!0,this.setValues(t)}copy(t){return super.copy(t),this.color.copy(t.color),this.specular.copy(t.specular),this.shininess=t.shininess,this.map=t.map,this.lightMap=t.lightMap,this.lightMapIntensity=t.lightMapIntensity,this.aoMap=t.aoMap,this.aoMapIntensity=t.aoMapIntensity,this.emissive.copy(t.emissive),this.emissiveMap=t.emissiveMap,this.emissiveIntensity=t.emissiveIntensity,this.bumpMap=t.bumpMap,this.bumpScale=t.bumpScale,this.normalMap=t.normalMap,this.normalMapType=t.normalMapType,this.normalScale.copy(t.normalScale),this.displacementMap=t.displacementMap,this.displacementScale=t.displacementScale,this.displacementBias=t.displacementBias,this.specularMap=t.specularMap,this.alphaMap=t.alphaMap,this.envMap=t.envMap,this.combine=t.combine,this.reflectivity=t.reflectivity,this.refractionRatio=t.refractionRatio,this.wireframe=t.wireframe,this.wireframeLinewidth=t.wireframeLinewidth,this.wireframeLinecap=t.wireframeLinecap,this.wireframeLinejoin=t.wireframeLinejoin,this.flatShading=t.flatShading,this.fog=t.fog,this}}const Yu={enabled:!1,files:{},add:function(r,t){this.enabled!==!1&&(this.files[r]=t)},get:function(r){if(this.enabled!==!1)return this.files[r]},remove:function(r){delete this.files[r]},clear:function(){this.files={}}};class Iv{constructor(t,e,n){const i=this;let s=!1,a=0,o=0,c;const l=[];this.onStart=void 0,this.onLoad=t,this.onProgress=e,this.onError=n,this.itemStart=function(h){o++,s===!1&&i.onStart!==void 0&&i.onStart(h,a,o),s=!0},this.itemEnd=function(h){a++,i.onProgress!==void 0&&i.onProgress(h,a,o),a===o&&(s=!1,i.onLoad!==void 0&&i.onLoad())},this.itemError=function(h){i.onError!==void 0&&i.onError(h)},this.resolveURL=function(h){return c?c(h):h},this.setURLModifier=function(h){return c=h,this},this.addHandler=function(h,u){return l.push(h,u),this},this.removeHandler=function(h){const u=l.indexOf(h);return u!==-1&&l.splice(u,2),this},this.getHandler=function(h){for(let u=0,d=l.length;u<d;u+=2){const f=l[u],m=l[u+1];if(f.global&&(f.lastIndex=0),f.test(h))return m}return null}}}const Nv=new Iv;class uc{constructor(t){this.manager=t!==void 0?t:Nv,this.crossOrigin="anonymous",this.withCredentials=!1,this.path="",this.resourcePath="",this.requestHeader={}}load(){}loadAsync(t,e){const n=this;return new Promise(function(i,s){n.load(t,i,e,s)})}parse(){}setCrossOrigin(t){return this.crossOrigin=t,this}setWithCredentials(t){return this.withCredentials=t,this}setPath(t){return this.path=t,this}setResourcePath(t){return this.resourcePath=t,this}setRequestHeader(t){return this.requestHeader=t,this}}uc.DEFAULT_MATERIAL_NAME="__DEFAULT";const wi={};class Fv extends Error{constructor(t,e){super(t),this.response=e}}class $f extends uc{constructor(t){super(t)}load(t,e,n,i){t===void 0&&(t=""),this.path!==void 0&&(t=this.path+t),t=this.manager.resolveURL(t);const s=Yu.get(t);if(s!==void 0)return this.manager.itemStart(t),setTimeout(()=>{e&&e(s),this.manager.itemEnd(t)},0),s;if(wi[t]!==void 0){wi[t].push({onLoad:e,onProgress:n,onError:i});return}wi[t]=[],wi[t].push({onLoad:e,onProgress:n,onError:i});const a=new Request(t,{headers:new Headers(this.requestHeader),credentials:this.withCredentials?"include":"same-origin"}),o=this.mimeType,c=this.responseType;fetch(a).then(l=>{if(l.status===200||l.status===0){if(l.status===0&&console.warn("THREE.FileLoader: HTTP Status 0 received."),typeof ReadableStream>"u"||l.body===void 0||l.body.getReader===void 0)return l;const h=wi[t],u=l.body.getReader(),d=l.headers.get("Content-Length")||l.headers.get("X-File-Size"),f=d?parseInt(d):0,m=f!==0;let x=0;const g=new ReadableStream({start(p){_();function _(){u.read().then(({done:v,value:S})=>{if(v)p.close();else{x+=S.byteLength;const M=new ProgressEvent("progress",{lengthComputable:m,loaded:x,total:f});for(let y=0,w=h.length;y<w;y++){const T=h[y];T.onProgress&&T.onProgress(M)}p.enqueue(S),_()}})}}});return new Response(g)}else throw new Fv(`fetch for "${l.url}" responded with ${l.status}: ${l.statusText}`,l)}).then(l=>{switch(c){case"arraybuffer":return l.arrayBuffer();case"blob":return l.blob();case"document":return l.text().then(h=>new DOMParser().parseFromString(h,o));case"json":return l.json();default:if(o===void 0)return l.text();{const u=/charset="?([^;"\s]*)"?/i.exec(o),d=u&&u[1]?u[1].toLowerCase():void 0,f=new TextDecoder(d);return l.arrayBuffer().then(m=>f.decode(m))}}}).then(l=>{Yu.add(t,l);const h=wi[t];delete wi[t];for(let u=0,d=h.length;u<d;u++){const f=h[u];f.onLoad&&f.onLoad(l)}}).catch(l=>{const h=wi[t];if(h===void 0)throw this.manager.itemError(t),l;delete wi[t];for(let u=0,d=h.length;u<d;u++){const f=h[u];f.onError&&f.onError(l)}this.manager.itemError(t)}).finally(()=>{this.manager.itemEnd(t)}),this.manager.itemStart(t)}setResponseType(t){return this.responseType=t,this}setMimeType(t){return this.mimeType=t,this}}class Kf extends Je{constructor(t,e=1){super(),this.isLight=!0,this.type="Light",this.color=new Kt(t),this.intensity=e}dispose(){}copy(t,e){return super.copy(t,e),this.color.copy(t.color),this.intensity=t.intensity,this}toJSON(t){const e=super.toJSON(t);return e.object.color=this.color.getHex(),e.object.intensity=this.intensity,this.groundColor!==void 0&&(e.object.groundColor=this.groundColor.getHex()),this.distance!==void 0&&(e.object.distance=this.distance),this.angle!==void 0&&(e.object.angle=this.angle),this.decay!==void 0&&(e.object.decay=this.decay),this.penumbra!==void 0&&(e.object.penumbra=this.penumbra),this.shadow!==void 0&&(e.object.shadow=this.shadow.toJSON()),e}}const jc=new re,qu=new C,Zu=new C;class Uv{constructor(t){this.camera=t,this.bias=0,this.normalBias=0,this.radius=1,this.blurSamples=8,this.mapSize=new yt(512,512),this.map=null,this.mapPass=null,this.matrix=new re,this.autoUpdate=!0,this.needsUpdate=!1,this._frustum=new wh,this._frameExtents=new yt(1,1),this._viewportCount=1,this._viewports=[new Ke(0,0,1,1)]}getViewportCount(){return this._viewportCount}getFrustum(){return this._frustum}updateMatrices(t){const e=this.camera,n=this.matrix;qu.setFromMatrixPosition(t.matrixWorld),e.position.copy(qu),Zu.setFromMatrixPosition(t.target.matrixWorld),e.lookAt(Zu),e.updateMatrixWorld(),jc.multiplyMatrices(e.projectionMatrix,e.matrixWorldInverse),this._frustum.setFromProjectionMatrix(jc),n.set(.5,0,0,.5,0,.5,0,.5,0,0,.5,.5,0,0,0,1),n.multiply(jc)}getViewport(t){return this._viewports[t]}getFrameExtents(){return this._frameExtents}dispose(){this.map&&this.map.dispose(),this.mapPass&&this.mapPass.dispose()}copy(t){return this.camera=t.camera.clone(),this.bias=t.bias,this.radius=t.radius,this.mapSize.copy(t.mapSize),this}clone(){return new this.constructor().copy(this)}toJSON(){const t={};return this.bias!==0&&(t.bias=this.bias),this.normalBias!==0&&(t.normalBias=this.normalBias),this.radius!==1&&(t.radius=this.radius),(this.mapSize.x!==512||this.mapSize.y!==512)&&(t.mapSize=this.mapSize.toArray()),t.camera=this.camera.toJSON(!1).object,delete t.camera.matrix,t}}class Bv extends Uv{constructor(){super(new Ah(-5,5,5,-5,.5,500)),this.isDirectionalLightShadow=!0}}class ju extends Kf{constructor(t,e){super(t,e),this.isDirectionalLight=!0,this.type="DirectionalLight",this.position.copy(Je.DEFAULT_UP),this.updateMatrix(),this.target=new Je,this.shadow=new Bv}dispose(){this.shadow.dispose()}copy(t){return super.copy(t),this.target=t.target.clone(),this.shadow=t.shadow.clone(),this}}class zv extends Kf{constructor(t,e){super(t,e),this.isAmbientLight=!0,this.type="AmbientLight"}}class Ov{constructor(t=1,e=0,n=0){return this.radius=t,this.phi=e,this.theta=n,this}set(t,e,n){return this.radius=t,this.phi=e,this.theta=n,this}copy(t){return this.radius=t.radius,this.phi=t.phi,this.theta=t.theta,this}makeSafe(){return this.phi=Math.max(1e-6,Math.min(Math.PI-1e-6,this.phi)),this}setFromVector3(t){return this.setFromCartesianCoords(t.x,t.y,t.z)}setFromCartesianCoords(t,e,n){return this.radius=Math.sqrt(t*t+e*e+n*n),this.radius===0?(this.theta=0,this.phi=0):(this.theta=Math.atan2(t,n),this.phi=Math.acos(ke(e/this.radius,-1,1))),this}clone(){return new this.constructor().copy(this)}}const $u=new C,so=new C;class Li{constructor(t=new C,e=new C){this.start=t,this.end=e}set(t,e){return this.start.copy(t),this.end.copy(e),this}copy(t){return this.start.copy(t.start),this.end.copy(t.end),this}getCenter(t){return t.addVectors(this.start,this.end).multiplyScalar(.5)}delta(t){return t.subVectors(this.end,this.start)}distanceSq(){return this.start.distanceToSquared(this.end)}distance(){return this.start.distanceTo(this.end)}at(t,e){return this.delta(e).multiplyScalar(t).add(this.start)}closestPointToPointParameter(t,e){$u.subVectors(t,this.start),so.subVectors(this.end,this.start);const n=so.dot(so);let s=so.dot($u)/n;return e&&(s=ke(s,0,1)),s}closestPointToPoint(t,e,n){const i=this.closestPointToPointParameter(t,e);return this.delta(n).multiplyScalar(i).add(this.start)}applyMatrix4(t){return this.start.applyMatrix4(t),this.end.applyMatrix4(t),this}equals(t){return t.start.equals(this.start)&&t.end.equals(this.end)}clone(){return new this.constructor().copy(this)}}class Gv extends ts{constructor(t=10,e=10,n=4473924,i=8947848){n=new Kt(n),i=new Kt(i);const s=e/2,a=t/e,o=t/2,c=[],l=[];for(let d=0,f=0,m=-o;d<=e;d++,m+=a){c.push(-o,0,m,o,0,m),c.push(m,0,-o,m,0,o);const x=d===s?n:i;x.toArray(l,f),f+=3,x.toArray(l,f),f+=3,x.toArray(l,f),f+=3,x.toArray(l,f),f+=3}const h=new we;h.setAttribute("position",new se(c,3)),h.setAttribute("color",new se(l,3));const u=new di({vertexColors:!0,toneMapped:!1});super(h,u),this.type="GridHelper"}dispose(){this.geometry.dispose(),this.material.dispose()}}typeof __THREE_DEVTOOLS__<"u"&&__THREE_DEVTOOLS__.dispatchEvent(new CustomEvent("register",{detail:{revision:"160"}}));typeof window<"u"&&(window.__THREE__?console.warn("WARNING: Multiple instances of Three.js being imported."):window.__THREE__="160");function Sa(r,t){return!r||r.length!==t?Array.from({length:t},()=>({x:0,y:0,z:0,active:!1})):r}function Fo(r,t){const e=Sa(t,r.length),n=r.nodeStorage;if(n){const{x:i,y:s,z:a}=n;for(let o=0;o<r.length;o++){const c=e[o];c.x=i[o],c.y=s[o],c.z=a[o],c.active=!0}return e}for(let i=0;i<r.length;i++){const s=r[i],a=e[i];a.x=s.x,a.y=s.y,a.z=s.z,a.active=!0}return e}function Uo(r){for(let t=0;t<r.length;t++){const e=r[t];e.x=0,e.y=0,e.z=0,e.active=!1}}function ro(r,t,e,n,i){const s=r[t];s.x+=e,s.y+=n,s.z+=i,s.active=!0}let Jf=32,Qf=0,Vv=8,tp=.006,ep=.002;function Ku(r){Jf=r}function Ju(r){Qf=r}function $c(r,t){tp=r,ep=t}const Fr=1e-4,Kc=1.25,ao=.92,Jc=.68,$e=.94,kv=2,Hv=[.25,.5,.75],Qu=[.25,.5,.75];function Wv(r,t,e,n){const i={x:new Float64Array(r),y:new Float64Array(r),z:new Float64Array(r),vx:new Float64Array(r),vy:new Float64Array(r),vz:new Float64Array(r),fx:new Float64Array(r),fy:new Float64Array(r),fz:new Float64Array(r),kx:new Float64Array(r),ky:new Float64Array(r),kz:new Float64Array(r),mass:new Float64Array(r),bendingStiffness:new Float64Array(r),bendAngleLimit:new Float64Array(r),pinned:new Uint8Array(r)};return i.mass.fill(t),i.bendingStiffness.fill(e),i.bendAngleLimit.fill(n),i}class Xv{constructor(t,e){this._storage=t,this.index=e}get x(){return this._storage.x[this.index]}set x(t){this._storage.x[this.index]=t}get y(){return this._storage.y[this.index]}set y(t){this._storage.y[this.index]=t}get z(){return this._storage.z[this.index]}set z(t){this._storage.z[this.index]=t}get vx(){return this._storage.vx[this.index]}set vx(t){this._storage.vx[this.index]=t}get vy(){return this._storage.vy[this.index]}set vy(t){this._storage.vy[this.index]=t}get vz(){return this._storage.vz[this.index]}set vz(t){this._storage.vz[this.index]=t}get fx(){return this._storage.fx[this.index]}set fx(t){this._storage.fx[this.index]=t}get fy(){return this._storage.fy[this.index]}set fy(t){this._storage.fy[this.index]=t}get fz(){return this._storage.fz[this.index]}set fz(t){this._storage.fz[this.index]=t}get mass(){return this._storage.mass[this.index]}set mass(t){this._storage.mass[this.index]=t}get bendingStiffness(){return this._storage.bendingStiffness[this.index]}set bendingStiffness(t){this._storage.bendingStiffness[this.index]=t}get bendAngleLimit(){return this._storage.bendAngleLimit[this.index]}set bendAngleLimit(t){this._storage.bendAngleLimit[this.index]=t}get kx(){return this._storage.kx[this.index]}set kx(t){this._storage.kx[this.index]=t}get ky(){return this._storage.ky[this.index]}set ky(t){this._storage.ky[this.index]=t}get kz(){return this._storage.kz[this.index]}set kz(t){this._storage.kz[this.index]=t}get pinned(){return this._storage.pinned[this.index]!==0}set pinned(t){this._storage.pinned[this.index]=t?1:0}}class Yv{constructor(t,e,{mass:n=1,bendingStiffness:i=Jf,smoothingIterations:s=Qf,constraintIterations:a=Vv,bendingConstraintIterations:o=0,bendAngleLimit:c=50,bendProjectionStrength:l=.35,curvatureFlow:h=0,logger:u=null}={}){this.segmentLength=e,this.nodeStorage=Wv(t,n,i,c),this.nodes=Array.from({length:t},(d,f)=>new Xv(this.nodeStorage,f)),this.nodes.nodeStorage=this.nodeStorage,this.smoothingIterations=s,this.constraintIterations=a,this.bendingConstraintIterations=o,this.bendAngleLimit=c,this.bendProjectionStrength=l,this.curvatureFlow=h,this.logger=u,this.iteration=0,this.collisionPrevPositions=null,this._constraintPrevPositions=null,this._bendingCorrections=null,this._smoothPositions=null,this._tensionCorrections=null;for(let d=0;d<t;d++)this.nodeStorage.x[d]=d*e}storeCollisionPreviousPositions(){const{x:t,y:e,z:n}=this.nodeStorage;if(!this.collisionPrevPositions||this.collisionPrevPositions.length!==this.nodes.length){this.collisionPrevPositions=Array.from({length:this.nodes.length},(i,s)=>new C(t[s],e[s],n[s]));return}for(let i=0;i<this.nodes.length;i++)this.collisionPrevPositions[i].set(t[i],e[i],n[i])}computeLength(){const{x:t,y:e,z:n}=this.nodeStorage;let i=0;for(let s=0;s<this.nodes.length-1;s++)i+=Math.hypot(t[s+1]-t[s],e[s+1]-e[s],n[s+1]-n[s]);return i}averageCurvature(){const{kx:t,ky:e,kz:n}=this.nodeStorage;let i=0;for(let s=0;s<this.nodes.length;s++)i+=Math.hypot(t[s],e[s],n[s]);return i/this.nodes.length}bendAngleAt(t){if(t<=0||t>=this.nodes.length-1)return 0;const{x:e,y:n,z:i}=this.nodeStorage,s=e[t]-e[t-1],a=n[t]-n[t-1],o=i[t]-i[t-1],c=e[t+1]-e[t],l=n[t+1]-n[t],h=i[t+1]-i[t],u=Math.hypot(s,a,o),d=Math.hypot(c,l,h);if(u<1e-8||d<1e-8)return 0;const f=(s*c+a*l+o*h)/(u*d);return Math.acos(Math.max(-1,Math.min(1,f)))*180/Math.PI}resetForces(){this.nodeStorage.fx.fill(0),this.nodeStorage.fy.fill(0),this.nodeStorage.fz.fill(0)}updateCurvature(){const{x:t,y:e,z:n,kx:i,ky:s,kz:a}=this.nodeStorage,o=this.segmentLength*this.segmentLength;i.fill(0),s.fill(0),a.fill(0);for(let c=1;c<this.nodes.length-1;c++)i[c]=(t[c-1]-2*t[c]+t[c+1])/o,s[c]=(e[c-1]-2*e[c]+e[c+1])/o,a[c]=(n[c-1]-2*n[c]+n[c+1])/o}accumulateBendingForces(){const t=this.nodes.length;if(t<3)return;const{fx:e,fy:n,fz:i,kx:s,ky:a,kz:o,bendingStiffness:c}=this.nodeStorage;for(let l=1;l<t-1;l++){const h=s[l],u=a[l],d=o[l],f=c[l],x=1+(h*h+u*u+d*d),g=f*h*x,p=f*u*x,_=f*d*x,v=.4,S=.1;l>=2&&(e[l-2]+=S*g,n[l-2]+=S*p,i[l-2]+=S*_),l+2<t&&(e[l+2]+=S*g,n[l+2]+=S*p,i[l+2]+=S*_),e[l-1]+=v*g,n[l-1]+=v*p,i[l-1]+=v*_,e[l+1]+=v*g,n[l+1]+=v*p,i[l+1]+=v*_;const M=v*2+(l>=2?S:0)+(l+2<t?S:0);e[l]-=M*g,n[l]-=M*p,i[l]-=M*_}}integrate(t){const{x:e,y:n,z:i,vx:s,vy:a,vz:o,fx:c,fy:l,fz:h,mass:u,pinned:d}=this.nodeStorage;for(let f=0;f<this.nodes.length;f++){if(d[f]){s[f]=0,a[f]=0,o[f]=0;continue}s[f]+=c[f]/u[f]*t,a[f]+=l[f]/u[f]*t,o[f]+=h[f]/u[f]*t,e[f]+=s[f]*t,n[f]+=a[f]*t,i[f]+=o[f]*t}}projectSegmentLengthConstraints(t=this.constraintIterations){const e=this.segmentLength;for(let n=0;n<t;n++)for(let i=0;i<this.nodes.length-1;i++){const s=this.nodes[i],a=this.nodes[i+1];let o=a.x-s.x,c=a.y-s.y,l=a.z-s.z,h=Math.hypot(o,c,l);if(!h)continue;const u=(h-e)/h;s.pinned&&a.pinned||(s.pinned?(o*=u,c*=u,l*=u,a.x-=o,a.y-=c,a.z-=l):a.pinned?(o*=u,c*=u,l*=u,s.x+=o,s.y+=c,s.z+=l):(o*=u*.5,c*=u*.5,l*=u*.5,s.x+=o,s.y+=c,s.z+=l,a.x-=o,a.y-=c,a.z-=l))}}solveConstraints(t,e={}){const n=Fo(this.nodes,this._constraintPrevPositions);this._constraintPrevPositions=n;const i=e.applyBending??!0,s=e.velocityDamping??.92;if(this.projectSegmentLengthConstraints(),i){for(let c=1;c<this.nodes.length-1;c++){const l=this.nodes[c-1],h=this.nodes[c],u=this.nodes[c+1];if(h.pinned)continue;const d=(l.x+u.x)*.5,f=(l.y+u.y)*.5,m=(l.z+u.z)*.5,x=h.x-d,g=h.y-f,p=h.z-m,_=Math.min(1,h.bendingStiffness*t+this.curvatureFlow),v=x*_,S=g*_,M=p*_;h.x-=v,h.y-=S,h.z-=M}const o=e.bendingConstraintIterations??this.bendingConstraintIterations;o>0&&(this.projectBendingConstraints(o),this.projectSegmentLengthConstraints(Math.max(2,Math.ceil(this.constraintIterations*.5))))}this.smoothingIterations>0&&this.laplacianSmooth();const a=1/t;for(let o=0;o<this.nodes.length;o++){const c=this.nodes[o];if(c.pinned){c.vx=c.vy=c.vz=0;continue}c.vx=(c.x-n[o].x)*a*s,c.vy=(c.y-n[o].y)*a*s,c.vz=(c.z-n[o].z)*a*s}}projectBendingConstraints(t=this.bendingConstraintIterations){if(t<=0||this.nodes.length<3)return;const e=Math.max(0,this.bendAngleLimit),n=Math.max(0,Math.min(1,this.bendProjectionStrength));this._bendingCorrections=Sa(this._bendingCorrections,this.nodes.length);for(let i=0;i<t;i++){const s=this._bendingCorrections;Uo(s);for(let a=1;a<this.nodes.length-1;a++){const o=this.nodes[a];if(o.pinned)continue;const c=this.bendAngleAt(a);if(c<=e)continue;const l=this.nodes[a-1],h=this.nodes[a+1],u=Math.max(0,Math.min(1,(c-e)/Math.max(1,180-e))),d=n*(.35+.65*u),f=s[a];f.x=((l.x+h.x)*.5-o.x)*d,f.y=((l.y+h.y)*.5-o.y)*d,f.z=((l.z+h.z)*.5-o.z)*d,f.active=!0}for(let a=1;a<this.nodes.length-1;a++){const o=this.nodes[a],c=s[a];!c.active||o.pinned||(o.x+=c.x,o.y+=c.y,o.z+=c.z)}}}laplacianSmooth(){const t=this.nodes.length;if(!(t<3)){this._smoothPositions=Sa(this._smoothPositions,t);for(let e=0;e<this.smoothingIterations;e++){const n=this._smoothPositions;Uo(n);for(let i=1;i<t-1;i++){if(this.nodes[i].pinned)continue;const a=this.nodes[i-1],o=this.nodes[i+1],c=n[i];c.x=(a.x+o.x)*.5,c.y=(a.y+o.y)*.5,c.z=(a.z+o.z)*.5,c.active=!0}for(let i=1;i<t-1;i++){const s=this.nodes[i];if(s.pinned)continue;const a=n[i];a.active&&(s.x=a.x,s.y=a.y,s.z=a.z)}}}}straightenByTension(t=.2,e=1){const n=this.nodes.length;if(n<3||t<=0||e<=0)return;const i=Math.max(0,Math.min(1,t));this._tensionCorrections=Sa(this._tensionCorrections,n);for(let s=0;s<e;s++){const a=this._tensionCorrections;Uo(a);for(let o=1;o<n-1;o++){const c=this.nodes[o];if(c.pinned)continue;const l=this.nodes[o-1],h=this.nodes[o+1],u=a[o];u.x=((l.x+h.x)*.5-c.x)*i,u.y=((l.y+h.y)*.5-c.y)*i,u.z=((l.z+h.z)*.5-c.z)*i,u.active=!0}for(let o=1;o<n-1;o++){const c=this.nodes[o],l=a[o];!l.active||c.pinned||(c.x+=l.x,c.y+=l.y,c.z+=l.z,c.vx*=.65,c.vy*=.65,c.vz*=.65)}}}releaseFromVesselWall(t,e=.1,n=1){if(!t||e<=0||n<=0)return;const i=Math.max(0,Math.min(1,e));for(let s=0;s<n;s++)for(const a of this.nodes){if(a.pinned)continue;let o=null;for(const h of t){if(h.isSheath)continue;const u=h.end.x-h.start.x,d=h.end.y-h.start.y,f=h.end.z-h.start.z,m=u*u+d*d+f*f;if(!m)continue;const x=a.x-h.start.x,g=a.y-h.start.y,p=a.z-h.start.z,_=(x*u+g*d+p*f)/m,v=Math.max(0,Math.min(1,_)),S=h.start.x+u*v,M=h.start.y+d*v,y=h.start.z+f*v,w=a.x-S,T=a.y-M,E=a.z-y,A=Math.hypot(w,T,E),D=A/(h.radius||1),b=Math.abs(D-.75)+Math.max(0,Math.abs(_-.5)-.5);D<=1.25&&(!o||b<o.distanceScore)&&(o={cx:S,cy:M,cz:y,radialDist:A,normalized:D,distanceScore:b})}if(!o||o.radialDist<=this.segmentLength*.15)continue;const c=Math.max(0,Math.min(1,(o.normalized-.25)/.75)),l=i*c;a.x+=(o.cx-a.x)*l,a.y+=(o.cy-a.y)*l,a.z+=(o.cz-a.z)*l,a.vx*=.75,a.vy*=.75,a.vz*=.75}}applyWallResponse(t,e,n,i,s,a){const o=t.vx*e+t.vy*n+t.vz*i;let c=t.vx-o*e,l=t.vy-o*n,h=t.vz-o*i;const u=Math.sqrt(c*c+l*l+h*h),d=a?Math.abs(o)*t.mass/s:0,f=Math.max(0,t.fx*e+t.fy*n+t.fz*i)+d;if(f>0&&u>0){const m=tp*f*s/t.mass,x=ep*f*s/t.mass;if(u<=m)c=0,l=0,h=0;else{const g=Math.max(0,u-x)/(u||1);c*=g,l*=g,h*=g}}t.vx=c,t.vy=l,t.vz=h}isPastOpenSheathEntrance(t,e){for(const n of e){if(!n.isSheath)continue;const i=n.end.x-n.start.x,s=n.end.y-n.start.y,a=n.end.z-n.start.z,o=i*i+s*s+a*a;if(!o)continue;const c=t.x-n.start.x,l=t.y-n.start.y,h=t.z-n.start.z,u=(c*i+l*s+h*a)/o;if(u>=-Fr)continue;const d=c-i*u,f=l-s*u,m=h-a*u;if(Math.hypot(d,f,m)<=n.radius+this.segmentLength)return!0}return!1}isInsideSegmentVolume(t,e){return this.segmentVolumeContact(t,e).inside}segmentVolumeContact(t,e){let n=null;for(const i of e||[]){const s=i.end.x-i.start.x,a=i.end.y-i.start.y,o=i.end.z-i.start.z,c=s*s+a*a+o*o;if(!c)continue;const l=t.x-i.start.x,h=t.y-i.start.y,u=t.z-i.start.z,d=(l*s+h*a+u*o)/c,f=Math.max(0,Math.min(1,d)),m=i.start.x+s*f,x=i.start.y+a*f,g=i.start.z+o*f,p=t.x-m,_=t.y-x,v=t.z-g,S=Math.hypot(p,_,v),M=S-i.radius;if(M<=0)return{inside:!0,segment:i,outside:M,cx:m,cy:x,cz:g,rx:p,ry:_,rz:v,radialDist:S,rawT:d};(!n||M<n.outside)&&(n={inside:!1,segment:i,outside:M,cx:m,cy:x,cz:g,rx:p,ry:_,rz:v,radialDist:S,rawT:d})}return n||{inside:!1,outside:1/0}}collideWithSegments(t,e,n,i={}){const s=this.segmentVolumeContact(t,e);if(s.inside||!Number.isFinite(s.outside))return!1;if(i.localOnly){const h=i.contactBand??this.segmentLength*Kc;if(s.outside>h||s.rawT<-Fr||s.rawT>1+Fr)return!1}const a=1/(s.radialDist||1),o=s.rx*a,c=s.ry*a,l=s.rz*a;return t.x=s.cx+o*s.segment.radius,t.y=s.cy+c*s.segment.radius,t.z=s.cz+l*s.segment.radius,this.applyWallResponse(t,o,c,l,n,!0),!0}collideRodSegmentsWithSegments(t,e,n={}){if(!t?.length)return;const i=n.segmentSamples||Hv,s=n.contactBand??this.segmentLength*Kc;for(let a=0;a<this.nodes.length-1;a++){const o=this.nodes[a],c=this.nodes[a+1];if(!(o.pinned&&c.pinned))for(const l of i){const h=1-l,u=l,d={x:o.x*h+c.x*u,y:o.y*h+c.y*u,z:o.z*h+c.z*u};if(this.isPastOpenSheathEntrance(d,t))continue;const f=this.segmentVolumeContact(d,t);if(f.inside||!Number.isFinite(f.outside)||n.localOnly&&(f.outside>s||f.rawT<-Fr||f.rawT>1+Fr))continue;const m=1/(f.radialDist||1),x=f.cx+f.rx*m*f.segment.radius,g=f.cy+f.ry*m*f.segment.radius,p=f.cz+f.rz*m*f.segment.radius,_=(x-d.x)*Jc,v=(g-d.y)*Jc,S=(p-d.z)*Jc,M=o.pinned?0:h,y=c.pinned?0:u,w=M*M+y*y;if(!(w<=1e-8)){if(!o.pinned){const T=M/w;o.x+=_*T,o.y+=v*T,o.z+=S*T,o.vx*=$e,o.vy*=$e,o.vz*=$e}if(!c.pinned){const T=y/w;c.x+=_*T,c.y+=v*T,c.z+=S*T,c.vx*=$e,c.vy*=$e,c.vz*=$e}}}}}collideWithMeshCollider(t,e,n,i=0,s=null){if(s&&e?.crossingContact){const c=e.crossingContact(s,t,i);if(c){t.x=c.target.x,t.y=c.target.y,t.z=c.target.z;const l=c.normal||new C(1,0,0);return this.applyWallResponse(t,l.x,l.y,l.z,n,!0),!0}}const a=e?.pointContact?.(t,i);if(!a?.violation)return!1;t.x=a.target.x,t.y=a.target.y,t.z=a.target.z;const o=a.normal||new C(1,0,0);return this.applyWallResponse(t,o.x,o.y,o.z,n,!0),!0}isPastOpenMeshOutlet(t,e={}){return Number.isFinite(e.openOutletY)&&t.y>e.openOutletY}meshContactAtPoint(t,e,n={}){const i=Math.max(0,n.clearance||0),s=new C,o=e.boundsTree.closestPointToPoint(t,{point:s})?.distance??t.distanceTo(s),c=typeof n.interiorDirection=="function"?n.interiorDirection(t,s).clone():new C().subVectors(t,s);c.lengthSq()<1e-8&&c.set(1,0,0),c.normalize();const l=new C().subVectors(t,s).dot(c);return{closest:s,interior:c,insideDepth:l,dist:o,clearance:i}}collideWithMesh(t,e,n,i={}){const s=new C(t.x,t.y,t.z),a=this.meshContactAtPoint(s,e,i),{closest:o,interior:c,insideDepth:l,dist:h,clearance:u}=a,d=Math.max(u+this.segmentLength*Kc,this.segmentLength*1.5),f=-c.x,m=-c.y,x=-c.z;if(l<u)t.x=o.x+c.x*u,t.y=o.y+c.y*u,t.z=o.z+c.z*u,this.applyWallResponse(t,f,m,x,n,!0);else{if(h>d)return;const g=Math.max(0,t.fx*f+t.fy*m+t.fz*x),p=t.vx*t.vx+t.vy*t.vy+t.vz*t.vz-(t.vx*f+t.vy*m+t.vz*x)**2;g>0&&p>0&&this.applyWallResponse(t,f,m,x,n,!1)}}collideRodSegmentsWithMesh(t,e,n={},i=null){const s=n.segmentSamples||Qu,a=Math.max(0,n.segmentClearance??Math.min(n.clearance||0,this.segmentLength*.06));for(let o=0;o<this.nodes.length-1;o++){const c=this.nodes[o],l=this.nodes[o+1];if(!(c.pinned&&l.pinned))for(const h of s){const u=1-h,d=h,f=new C(c.x*u+l.x*d,c.y*u+l.y*d,c.z*u+l.z*d),m={x:f.x,y:f.y,z:f.z};if(this.isPastOpenMeshOutlet(m,n)||i&&(this.isInsideSegmentVolume(m,i)||this.isPastOpenSheathEntrance(m,i)))continue;const x=this.meshContactAtPoint(f,t,n);if(x.insideDepth>=a)continue;const g=x.closest.x+x.interior.x*a,p=x.closest.y+x.interior.y*a,_=x.closest.z+x.interior.z*a,v=(g-f.x)*ao,S=(p-f.y)*ao,M=(_-f.z)*ao,y=c.pinned?0:u,w=l.pinned?0:d,T=y*y+w*w;if(!(T<=1e-8)){if(!c.pinned){const E=y/T;c.x+=v*E,c.y+=S*E,c.z+=M*E,c.vx*=$e,c.vy*=$e,c.vz*=$e}if(!l.pinned){const E=w/T;l.x+=v*E,l.y+=S*E,l.z+=M*E,l.vx*=$e,l.vy*=$e,l.vz*=$e}}}}}collideRodSegmentsWithMeshCollider(t,e,n={},i=null,s=null){const a=n.segmentSamples||Qu,o=Math.max(0,n.segmentClearance??Math.min(n.clearance||0,this.segmentLength*.08)),c=this.segmentLength*4.5,l=new C,h=new C,u=new C,d=new C,f=(m,x,g,p,_)=>{h.subVectors(_,p),h.length()>c&&h.setLength(c),h.multiplyScalar(ao);const v=1-g,S=g,M=m.pinned?0:v,y=x.pinned?0:S,w=M*M+y*y;if(!(w<=1e-8)){if(!m.pinned){const T=M/w;m.x+=h.x*T,m.y+=h.y*T,m.z+=h.z*T,m.vx*=$e,m.vy*=$e,m.vz*=$e}if(!x.pinned){const T=y/w;x.x+=h.x*T,x.y+=h.y*T,x.z+=h.z*T,x.vx*=$e,x.vy*=$e,x.vz*=$e}}};for(let m=0;m<this.nodes.length-1;m++){const x=this.nodes[m],g=this.nodes[m+1];if(!(x.pinned&&g.pinned)){if(t.crossingContact){l.set((x.x+g.x)*.5,(x.y+g.y)*.5,(x.z+g.z)*.5);const p={x:l.x,y:l.y,z:l.z};if(this.isPastOpenMeshOutlet(p,n))continue;if(!(i&&(this.isInsideSegmentVolume(p,i)||this.isPastOpenSheathEntrance(p,i)))){u.set(x.x,x.y,x.z),d.set(g.x,g.y,g.z);const v=t.crossingContact(u,d,o);v&&v.t>.03&&v.t<.97&&f(x,g,v.t,v.point,v.target)}}for(const p of a){const _=1-p,v=p;l.set(x.x*_+g.x*v,x.y*_+g.y*v,x.z*_+g.z*v);const S={x:l.x,y:l.y,z:l.z};if(this.isPastOpenMeshOutlet(S,n)||i&&(this.isInsideSegmentVolume(S,i)||this.isPastOpenSheathEntrance(S,i)))continue;const M=t.pointContact(l,o);M?.violation&&f(x,g,p,l,M.target)}}}}collide(t,e=1){if(!t)return;const n=t.segments||null,i=t.meshCollider||t.lumenMeshCollider||null,s=t.collisionGeometry||(t.isBufferGeometry?t:t.geometry||t),a=this.collisionPrevPositions,o={clearance:t.guidewireClearance??t.collisionClearance??t.clearance??0,segmentClearance:t.guidewireSegmentClearance??t.segmentClearance,segmentSamples:t.guidewireSegmentSamples??t.segmentSamples,openOutletY:t.openOutletY,interiorDirection:t.interiorDirection||t.collisionInteriorDirection},c=!!i||!!s?.boundsTree,l={localOnly:c},h=Math.max(1,t.guidewireCollisionPasses??t.collisionPasses??(c?kv:4));if(!c&&!n)return;const u=()=>{for(let d=0;d<this.nodes.length;d++){const f=this.nodes[d];if(!f.pinned&&!(n&&(this.isInsideSegmentVolume(f,n)||this.isPastOpenSheathEntrance(f,n)||this.collideWithSegments(f,n,e,l)||!c))&&!this.isPastOpenMeshOutlet(f,o))if(i){const m=a?.[d]||null;this.collideWithMeshCollider(f,i,e,o.clearance,m)}else s&&s.boundsTree&&this.collideWithMesh(f,s,e,o)}};if(i){for(let d=0;d<h;d++)u(),n&&this.collideRodSegmentsWithSegments(n,e,l),this.collideRodSegmentsWithMeshCollider(i,e,o,n,a);u(),n&&this.collideRodSegmentsWithSegments(n,e,l)}else if(s&&s.boundsTree){for(let d=0;d<h;d++)u(),n&&this.collideRodSegmentsWithSegments(n,e,l),this.collideRodSegmentsWithMesh(s,e,o,n);u(),n&&this.collideRodSegmentsWithSegments(n,e,l)}else for(let d=0;d<h;d++)u(),n&&this.collideRodSegmentsWithSegments(n,e);this.smoothingIterations>0&&this.laplacianSmooth(e),this.storeCollisionPreviousPositions()}step(t){this.storeCollisionPreviousPositions(),this.resetForces(),this.updateCurvature(),this.accumulateBendingForces(),this.integrate(t),this.solveConstraints(t),this.iteration++,this.logger&&this.logger({iteration:this.iteration,curvature:this.averageCurvature(),length:this.computeLength()})}}function zt(r,t,e){return Math.max(t,Math.min(e,r))}function pn(r,t,e){const n=zt((e-r)/Math.max(1e-6,t-r),0,1);return n*n*(3-2*n)}const Ai=1.35,qv=.72,Zv=2.4,jv=.42,$v=.035,Kv=18,Jv=5,Qv=[.06,.12,.15,.25,.35,.45,.55,.65,.75,.85,.9,.94],tS=64,eS=.32,nS=0,iS=1,sS=.55,rS=.45,aS=2,oS=[0,.2,.4,.6,.8,1],cS=142,lS=0,hS=5,uS=10,dS=6,fS=2,pS=128,mS=.38,gS=2,xS=1.1,_S=.1,vS=170,SS=140,MS=3,yS=10,ES=108,wS=1,AS=.34,TS=2,CS=96,bS=10,RS=.35,PS=18,ni=.001;function Ws(){return{query:{inward:{x:0,y:0,z:0},normal:{x:0,y:0,z:0},closestPoint:{x:0,y:0,z:0}},target:{x:0,y:0,z:0},closestPoint:{x:0,y:0,z:0},inward:{x:0,y:0,z:0},normal:{x:0,y:0,z:0}}}function Xs(r,t){r.x=t.x,r.y=t.y,r.z=t.z}function td(r,t,e){r.x+=t.x*e,r.y+=t.y*e,r.z+=t.z*e}function ed(r,t){return Math.hypot(r.x-t.x,r.y-t.y,r.z-t.z)}function un(r,t){const e=Math.hypot(r.x,r.y,r.z);return e<1e-8?{...t}:{x:r.x/e,y:r.y/e,z:r.z/e}}function nd(r,t,e){return{x:r.x*(1-e)+t.x*e,y:r.y*(1-e)+t.y*e,z:r.z*(1-e)+t.z*e}}function Ti(){return globalThis.performance?.now?.()??Date.now()}function LS(){return{advanceMs:0,solveMs:0,projectMs:0,diagnosticMs:0,pointContactCount:0,diagnosticPointContactCount:0,projectGuidewireCalls:0,nodeProjectionCount:0,segmentProjectionCount:0,segmentSampleCount:0,solveIterations:0,moving:!1,boundaryDrivenFeed:!1,forceRelax:!1,foldGuarded:!1,stabilityRepaired:!1,withdrawalRelaxed:!1}}function DS(r){return r.advanceMs=0,r.solveMs=0,r.projectMs=0,r.diagnosticMs=0,r.pointContactCount=0,r.diagnosticPointContactCount=0,r.projectGuidewireCalls=0,r.nodeProjectionCount=0,r.segmentProjectionCount=0,r.segmentSampleCount=0,r.solveIterations=0,r.moving=!1,r.boundaryDrivenFeed=!1,r.forceRelax=!1,r.foldGuarded=!1,r.stabilityRepaired=!1,r.withdrawalRelaxed=!1,r}class IS{constructor({rod:t,segmentLength:e,guidewireLength:n,sheath:i,lumenSampler:s=null,advanceRate:a=44,minInsert:o=0,maxInsert:c=n,lumenClearance:l=qv,axialWindowScale:h=Zv,straightening:u=jv,routeBlend:d=$v,relaxationIterations:f=Kv,lengthIterations:m=Jv,segmentSamples:x=Qv,maxBendAngle:g=tS,bendLimitStrength:p=eS,bendLimitIterations:_=nS,segmentProjectionBlend:v=iS,maxSegmentProjectionStep:S=sS,meshClearance:M=rS,collisionProjectionRepeats:y=aS,foldAngle:w=cS,foldUntangleStrength:T=lS,foldUntangleWindow:E=hS,finalCollisionPasses:A=uS,finalLengthPasses:D=dS,finalProjectionPasses:b=fS,foldGuardAngle:F=pS,foldGuardStrength:L=mS,foldGuardPasses:N=gS,foldGuardCenterPull:U=xS,stabilityRepairSegmentError:k=_S,stabilityRepairBendAngle:O=vS,stabilityRepairTargetBendAngle:H=SS,stabilityRepairPasses:j=MS,stabilityRepairLengthIterations:J=yS,tipBacktrackAngle:nt=ES,tipBacktrackStrength:V=wS,withdrawalStraightening:$=AS,withdrawalStraighteningPasses:st=TS,withdrawalRelaxFrames:at=CS,unsupportedBendRelaxAngle:lt=bS,unsupportedBendSupportBand:_t=RS,unsupportedBendRelaxFrames:ht=PS}){this.rod=t,this.segmentLength=e,this.guidewireLength=n,this.sheath=i,this.lumenSampler=typeof s=="function"?s:null,this.advanceRate=a,this.minInsert=o,this.maxInsert=c,this.lumenClearance=l,this.axialWindowScale=h,this.straightening=u,this.routeBlend=d,this.relaxationIterations=f,this.lengthIterations=m,this.segmentSamples=x,this.maxBendAngle=g,this.bendLimitStrength=p,this.bendLimitIterations=_,this.segmentProjectionBlend=v,this.maxSegmentProjectionStep=S,this.meshClearance=M,this.collisionProjectionRepeats=Math.max(1,Math.floor(y)),this.foldAngle=w,this.foldUntangleStrength=T,this.foldUntangleWindow=E,this.finalCollisionPasses=A,this.finalLengthPasses=D,this.finalProjectionPasses=b,this.foldGuardAngle=F,this.foldGuardStrength=L,this.foldGuardPasses=N,this.foldGuardCenterPull=U,this.stabilityRepairSegmentError=k,this.stabilityRepairBendAngle=O,this.stabilityRepairTargetBendAngle=H,this.stabilityRepairPasses=j,this.stabilityRepairLengthIterations=J,this.tipBacktrackAngle=nt,this.tipBacktrackStrength=V,this.withdrawalStraightening=$,this.withdrawalStraighteningPasses=st,this.withdrawalRelaxFrames=at,this.unsupportedBendRelaxAngle=lt,this.unsupportedBendSupportBand=_t,this.unsupportedBendRelaxFrames=ht,this.tailProgress=0,this.lastAdvanceDelta=0,this.settleFramesRemaining=0,this.withdrawalRelaxFramesRemaining=0,this.unsupportedBendRelaxFramesRemaining=0,this.unsupportedBendRelaxArmed=!0,this.contactPoints=[],this.breachPoints=[],this.previousPositions=null,this.performanceStats=LS(),this._advancePreviousPositions=null,this._solvePreviousPositions=null,this._straightenCorrections=null,this._spanCorrections=null,this._untangleCorrections=null,this._bendLimitCorrections=null,this._diagnosticContact=Ws(),this._supportContact=Ws(),this._projectContact=Ws(),this._slidePointContact=Ws(),this._slideTargetContact=Ws(),this._zeroVelocityContact=Ws(),this._projectNodePoint={x:0,y:0,z:0},this._convectSource={x:0,y:0,z:0},this._lumenConstraintState={projected:{x:0,y:0,z:0},radialMargin:0,axialOffset:0,axialWindow:0,breach:!1};const ut={x:i.end.x-i.start.x,y:i.end.y-i.start.y,z:i.end.z-i.start.z};this.sheathLength=Math.hypot(ut.x,ut.y,ut.z)||1,this.sheathDir=un(ut,{x:1,y:0,z:0}),this.externalTailStart={x:i.start.x-this.sheathDir.x*n,y:i.start.y-this.sheathDir.y*n,z:i.start.z-this.sheathDir.z*n}}get progress(){return this.tailProgress}getPerformanceStats(){return{...this.performanceStats}}reset(){return this.tailProgress=this.minInsert,this.lastAdvanceDelta=0,this.settleFramesRemaining=0,this.withdrawalRelaxFramesRemaining=0,this.unsupportedBendRelaxFramesRemaining=0,this.unsupportedBendRelaxArmed=!0,this.contactPoints.length=0,this.breachPoints.length=0,this.initialize(),this}initialize(){const t=this.rod.nodes.nodeStorage;if(t){const{x:e,y:n,z:i,vx:s,vy:a,vz:o,pinned:c}=t;for(let l=0;l<this.rod.nodes.length;l++){const h=this.segmentLength*l;e[l]=this.externalTailStart.x+this.sheathDir.x*h,n[l]=this.externalTailStart.y+this.sheathDir.y*h,i[l]=this.externalTailStart.z+this.sheathDir.z*h,s[l]=0,a[l]=0,o[l]=0,c[l]=1}}else for(let e=0;e<this.rod.nodes.length;e++){const n=this.segmentLength*e,i=this.rod.nodes[e];i.x=this.externalTailStart.x+this.sheathDir.x*n,i.y=this.externalTailStart.y+this.sheathDir.y*n,i.z=this.externalTailStart.z+this.sheathDir.z*n,i.vx=i.vy=i.vz=0,i.pinned=!0}this.constrainSheath(),this.previousPositions=Fo(this.rod.nodes,this._advancePreviousPositions),this._advancePreviousPositions=this.previousPositions}insertedCoordinate(t){return this.segmentLength*t-this.guidewireLength+this.tailProgress}firstLumenNodeIndex(){return zt(Math.ceil((this.sheathLength+this.guidewireLength-this.tailProgress)/this.segmentLength),0,this.rod.nodes.length)}firstInsertedNodeIndex(){return zt(Math.ceil((this.guidewireLength-this.tailProgress)/this.segmentLength),0,this.rod.nodes.length)}#t(){return zt(Math.floor((this.sheathLength+ni+this.guidewireLength-this.tailProgress)/this.segmentLength)+1,0,this.rod.nodes.length)}#i(){return Math.max(0,this.#t()-1)}sheathAxisPoint(t){return{x:this.sheath.start.x+this.sheathDir.x*t,y:this.sheath.start.y+this.sheathDir.y*t,z:this.sheath.start.z+this.sheathDir.z*t}}routeSample(t){return this.#e(t)?{point:this.sheathAxisPoint(t),tangent:{...this.sheathDir},radius:this.sheath.radius||2}:this.lumenSampler?this.lumenSampler(Math.max(0,t-this.sheathLength)):{point:this.sheathAxisPoint(t),tangent:{...this.sheathDir},radius:1/0}}constrainSheath(t=0){const e=this.rod.nodes.nodeStorage;if(e){const{x:n,y:i,z:s,vx:a,vy:o,vz:c,pinned:l}=e,h=this.#t();for(let u=0;u<h;u++){const d=this.insertedCoordinate(u);l[u]=1,n[u]=this.sheath.start.x+this.sheathDir.x*d,i[u]=this.sheath.start.y+this.sheathDir.y*d,s[u]=this.sheath.start.z+this.sheathDir.z*d,a[u]=this.sheathDir.x*t,o[u]=this.sheathDir.y*t,c[u]=this.sheathDir.z*t}l.fill(0,h);return}for(let n=0;n<this.rod.nodes.length;n++){const i=this.insertedCoordinate(n),s=this.rod.nodes[n],a=this.#e(i);s.pinned=a,a&&(s.x=this.sheath.start.x+this.sheathDir.x*i,s.y=this.sheath.start.y+this.sheathDir.y*i,s.z=this.sheath.start.z+this.sheathDir.z*i,s.vx=this.sheathDir.x*t,s.vy=this.sheathDir.y*t,s.vz=this.sheathDir.z*t)}}advance(t,e,n=null,{routeAssist:i=!0,boundaryDriven:s=!1}={}){DS(this.performanceStats);const a=Ti(),o=Fo(this.rod.nodes,this._advancePreviousPositions);this._advancePreviousPositions=o;const c=zt(this.tailProgress+t*this.advanceRate*e,this.minInsert,this.maxInsert),l=c-this.tailProgress;this.tailProgress=c,this.lastAdvanceDelta=l,Math.abs(l)>1e-6&&(this.requestSettle(),this.unsupportedBendRelaxArmed=!0,this.unsupportedBendRelaxFramesRemaining=0),l<-1e-6&&(this.withdrawalRelaxFramesRemaining=Math.max(this.withdrawalRelaxFramesRemaining,Math.max(0,Math.floor(this.withdrawalRelaxFrames))));const h=l/Math.max(e,1e-6);return this.constrainSheath(h),Math.abs(l)>1e-6&&!s&&this.#a(l,o,e,n,i),this.previousPositions=o,this.performanceStats.advanceMs+=Ti()-a,this.performanceStats.moving=Math.abs(l)>1e-6,this.performanceStats.boundaryDrivenFeed=s&&Math.abs(l)>1e-6,l}requestSettle(t=48){this.settleFramesRemaining=Math.max(this.settleFramesRemaining,t)}solve(t,e=null,{iterations:n=this.relaxationIterations,forceRelax:i=!1}={}){const s=Ti();this.performanceStats.forceRelax=this.performanceStats.forceRelax||!!i;const a=Fo(this.rod.nodes,this._solvePreviousPositions);this._solvePreviousPositions=a,this.contactPoints.length=0,this.breachPoints.length=0,this.constrainSheath();const o=this.lastAdvanceDelta>1e-6,c=this.lastAdvanceDelta<-1e-6||this.withdrawalRelaxFramesRemaining>0;let l=!1;!o&&(this.unsupportedBendRelaxArmed||this.unsupportedBendRelaxFramesRemaining>0)&&(l=this.#p(e),l&&this.unsupportedBendRelaxArmed?(this.unsupportedBendRelaxFramesRemaining=Math.max(this.unsupportedBendRelaxFramesRemaining,Math.max(1,Math.floor(this.unsupportedBendRelaxFrames))),this.unsupportedBendRelaxArmed=!1):l||(this.unsupportedBendRelaxFramesRemaining=0,this.unsupportedBendRelaxArmed=!0));const h=l&&this.unsupportedBendRelaxFramesRemaining>0;if(!(i||Math.abs(this.lastAdvanceDelta)>1e-6||this.settleFramesRemaining>0||c||h)){this.#F(a,t,e),this.performanceStats.solveMs+=Ti()-s;return}Math.abs(this.lastAdvanceDelta)>1e-6&&(this.#c(),this.#E(e));const d=c||h;d&&(this.performanceStats.withdrawalRelaxed=this.#s(e,c?1:.72)||this.performanceStats.withdrawalRelaxed);const f=Math.max(1,n);this.performanceStats.solveIterations+=f;for(let x=0;x<f;x++){this.#S(x/f,e),d&&x<2&&(this.performanceStats.withdrawalRelaxed=this.#s(e,c?1:.72)||this.performanceStats.withdrawalRelaxed),this.#m();for(let g=0;g<this.collisionProjectionRepeats;g++)this.#g(),this.#r(this.lengthIterations),this.#h(e,!1);this.#r(2),this.#h(e,!1)}this.constrainSheath();for(let x=0;x<this.finalCollisionPasses;x++)this.#m(),this.#g(),this.#h(e,!1),this.#r(this.lengthIterations+2);this.#r(this.lengthIterations+4);for(let x=0;x<this.finalLengthPasses;x++)this.#h(e,!1),this.#r(5);for(let x=0;x<this.finalProjectionPasses;x++)this.#h(e,!1);this.#E(e),this.#r(Math.max(2,Math.ceil(this.lengthIterations*.4)));let m=this.#w();this.performanceStats.foldGuarded=this.performanceStats.foldGuarded||m,this.#h(e,!1),(m||this.#v(this.foldGuardAngle))&&(m=this.#w()||m,this.performanceStats.foldGuarded=this.performanceStats.foldGuarded||m,this.#r(this.lengthIterations+4),this.#h(e,!1),this.#r(4),this.#h(e,!1)),this.#b(e)&&(this.performanceStats.stabilityRepaired=!0,this.performanceStats.foldGuarded=!0),this.#F(a,t,e),Math.abs(this.lastAdvanceDelta)<=1e-6&&this.settleFramesRemaining>0&&this.settleFramesRemaining--,this.lastAdvanceDelta>=-1e-6&&this.withdrawalRelaxFramesRemaining>0&&this.withdrawalRelaxFramesRemaining--,this.unsupportedBendRelaxFramesRemaining>0&&(this.unsupportedBendRelaxFramesRemaining--,this.unsupportedBendRelaxFramesRemaining<=0&&!l&&(this.unsupportedBendRelaxArmed=!0)),this.performanceStats.solveMs+=Ti()-s}collectContactSamples(t=null,e=Ai){const n=[],i=[],s=[0,.15,.35,.55,.75,.9,1],a=(c,l)=>{if(this.#e(l))return;const h=this.diagnosePoint(c,l,t,e);h.breach?this.#d(i,c):h.contact&&this.#d(n,c)};for(let c=0;c<this.rod.nodes.length-1;c++){const l=this.rod.nodes[c],h=this.rod.nodes[c+1];for(const u of s){const d=this.insertedCoordinate(c+u),f=nd(l,h,u);a(f,d)}}const o=this.rod.nodes[this.rod.nodes.length-1];return a(o,this.insertedCoordinate(this.rod.nodes.length-1)),{contacts:n,breaches:i}}collectLumenDiagnostics(t=null,{clearance:e=this.meshClearance,contactBand:n=Ai,samples:i=oS,collectMarkers:s=!1,markerLimit:a=420}={}){const o=Ti();this.performanceStats.diagnosticMs=0,this.performanceStats.diagnosticPointContactCount=0;const c=t?.meshCollider||t?.lumenMeshCollider||null,l={checkedCount:0,contactCount:0,outsideCount:0,clearanceViolationCount:0,minSignedDistance:null,minClearanceMargin:null,worstPoint:null,worstInserted:null,maxSegmentError:0,maxBendAngle:0,clearance:e,contactBand:n,contacts:s?[]:null,breaches:s?[]:null};for(let h=0;h<this.rod.nodes.length-1;h++){const u=this.rod.nodes[h],d=this.rod.nodes[h+1];if(l.maxSegmentError=Math.max(l.maxSegmentError,Math.abs(ed(u,d)-this.segmentLength)),c?.pointContact)for(const f of i){const m=this.insertedCoordinate(h+f);if(this.#e(m))continue;const x=nd(u,d,f),g=this.#T(c,x,e,!0,this._diagnosticContact),p=Number.isFinite(g?.signedDistance)?g.signedDistance:null;if(!Number.isFinite(p))continue;l.checkedCount++,(l.minSignedDistance===null||p<l.minSignedDistance)&&(l.minSignedDistance=p,l.worstPoint={x:x.x,y:x.y,z:x.z},l.worstInserted=m);const _=p-e;(l.minClearanceMargin===null||_<l.minClearanceMargin)&&(l.minClearanceMargin=_),p<0?(l.outsideCount++,s&&this.#d(l.breaches,x,a)):p<=n&&(l.contactCount++,s&&this.#d(l.contacts,x,a)),p<e&&l.clearanceViolationCount++}}if(typeof this.rod.bendAngleAt=="function")for(let h=1;h<this.rod.nodes.length-1;h++){const u=this.insertedCoordinate(h);this.#e(u)||(l.maxBendAngle=Math.max(l.maxBendAngle,this.rod.bendAngleAt(h)||0))}return this.performanceStats.diagnosticMs=Ti()-o,l}diagnosePoint(t,e,n=null,i=Ai){const s=this.lumenSampler?this.#L(t,e):null;let a=s?s.radialMargin<=i||Math.abs(s.axialOffset)>=s.axialWindow-i:!1,o=s?.breach||!1;const c=n?.meshCollider||n?.lumenMeshCollider||null;if(c?.pointContact&&!this.#e(e)){const l=this.#T(c,t,0,!0,this._diagnosticContact);o=o||!!l?.violation,a=a||!l?.violation&&Number.isFinite(l?.distance)&&l.distance<=i}return{contact:!o&&a,breach:o}}#a(t,e,n,i=null,s=!0){const a=t/this.segmentLength,o=1/Math.max(n,1e-6),c=s?this.#A(i):null,l=this.#t();for(let h=l;h<this.rod.nodes.length;h++){const u=this.rod.nodes[h],d=this.insertedCoordinate(h);if(this.#e(d))continue;const f=this.#f(e,h+a,s?i:null,this._convectSource,s);if(!s){const M=e[h];Xs(u,f),u.vx=(f.x-M.x)*o*.2,u.vy=(f.y-M.y)*o*.2,u.vz=(f.z-M.z)*o*.2;continue}const m=this.lumenSampler?this.routeSample(d).point:f,x=d<this.sheathLength+this.segmentLength*2,g=this.lumenSampler?x?.64:.12:0,p={x:f.x*(1-g)+m.x*g,y:f.y*(1-g)+m.y*g,z:f.z*(1-g)+m.z*g},_=e[h],v=this.#o(_,{x:p.x-_.x,y:p.y-_.y,z:p.z-_.z},c);p.x=_.x+v.x,p.y=_.y+v.y,p.z=_.z+v.z;const S=this.#x(p,d,i,!1);Xs(u,S),u.vx=(S.x-_.x)*o*.2,u.vy=(S.y-_.y)*o*.2,u.vz=(S.z-_.z)*o*.2}}#f(t,e,n=null,i={x:0,y:0,z:0},s=!0){const a=t.length-1;if(e<=0){const _=t[0];return i.x=_.x+this.sheathDir.x*e*this.segmentLength,i.y=_.y+this.sheathDir.y*e*this.segmentLength,i.z=_.z+this.sheathDir.z*e*this.segmentLength,i}if(e<a){const _=Math.floor(e),v=Math.min(a,_+1),S=e-_,M=t[_],y=t[v];return i.x=M.x+(y.x-M.x)*S,i.y=M.y+(y.y-M.y)*S,i.z=M.z+(y.z-M.z)*S,i}const o=t[a],c=t[Math.max(0,a-1)];let l=o.x-c.x,h=o.y-c.y,u=o.z-c.z;const d=Math.sqrt(l*l+h*h+u*u);if(d>1e-8)l/=d,h/=d,u/=d;else if(s){const _=this.routeSample(this.tailProgress).tangent;l=_.x,h=_.y,u=_.z}else l=this.sheathDir.x,h=this.sheathDir.y,u=this.sheathDir.z;let f=l,m=h,x=u;const g=s?this.#A(n):null;if(g){const _=un(this.#o(o,{x:l,y:h,z:u},g),{x:l,y:h,z:u});f=_.x,m=_.y,x=_.z}const p=(e-a)*this.segmentLength;return i.x=o.x+f*p,i.y=o.y+m*p,i.z=o.z+x*p,i}#c(t=1){if(this.routeBlend<=0||!this.lumenSampler)return;const e=this.rod.nodes.nodeStorage;if(e){const{x:n,y:i,z:s,pinned:a}=e,o=this.#t();for(let c=o;c<this.rod.nodes.length;c++){const l=this.insertedCoordinate(c);if(this.#e(l))continue;const h=this.routeSample(l),u=pn(this.sheathLength,this.sheathLength+this.segmentLength*8,l),d=this.routeBlend*t*(.35+.65*u);n[c]+=(h.point.x-n[c])*d,i[c]+=(h.point.y-i[c])*d,s[c]+=(h.point.z-s[c])*d}return}for(let n=0;n<this.rod.nodes.length;n++){const i=this.rod.nodes[n];if(i.pinned)continue;const s=this.insertedCoordinate(n);if(this.#e(s))continue;const a=this.routeSample(s),o=pn(this.sheathLength,this.sheathLength+this.segmentLength*8,s),c=this.routeBlend*t*(.35+.65*o);i.x+=(a.point.x-i.x)*c,i.y+=(a.point.y-i.y)*c,i.z+=(a.point.z-i.z)*c}}#l(t,e=null){const n=this.rod.nodes[t];if(!n)return!0;const i=this.insertedCoordinate(t);if(this.#e(i))return!0;const s=this.#A(e);if(!s?.pointContact)return!1;const a=this.#T(s,n,this.meshClearance,!1,this._supportContact);return!!a?.violation||Number.isFinite(a?.signedDistance)&&a.signedDistance<=this.meshClearance+this.unsupportedBendSupportBand}#p(t=null){const e=Math.max(0,this.unsupportedBendRelaxAngle);if(e<=0||typeof this.rod.bendAngleAt!="function")return!1;for(let n=1;n<this.rod.nodes.length-1;n++){if(this.rod.nodes[n].pinned||this.insertedCoordinate(n)<=this.sheathLength+this.segmentLength+ni||(this.rod.bendAngleAt(n)||0)<=e)continue;const o=!this.#l(n+1,t),c=!this.#l(n,t);if(o||c)return!0}return!1}#u(t,e,n,i,s,a){let o=n;s?.pointContact&&!this.#e(e)&&(o=this.#o(t,o,s));const c=Math.hypot(o.x,o.y,o.z);if(c<=this.segmentLength*.002)return!1;if(c>a){const h=a/c;o={x:o.x*h,y:o.y*h,z:o.z*h}}const l=this.#x({x:t.x+o.x,y:t.y+o.y,z:t.z+o.z},e,i,!1);return Xs(t,l),!0}#s(t=null,e=1){const n=Math.max(0,Math.floor(this.withdrawalStraighteningPasses)),i=zt(this.withdrawalStraightening*e,0,1);if(n<=0||i<=0)return!1;const s=this.#A(t),a=this.withdrawalRelaxFramesRemaining>0?.55:.25,o=zt(Math.abs(this.lastAdvanceDelta)/Math.max(1e-6,this.segmentLength*.25),a,1),c=this.segmentLength*.16,l=Math.max(1,Math.ceil(56/Math.max(1e-6,this.segmentLength)));let h=!1;for(let u=0;u<n;u++)for(let d=1;d<this.rod.nodes.length;d++){const f=this.rod.nodes[d];if(f.pinned||this.insertedCoordinate(d)<=this.sheathLength+this.segmentLength+ni)continue;const x=this.rod.nodes[d-1],g=this.rod.nodes[d-2],p=g?un({x:x.x-g.x,y:x.y-g.y,z:x.z-g.z},this.sheathDir):this.sheathDir,_=un({x:f.x-x.x,y:f.y-x.y,z:f.z-x.z},p),v=zt(p.x*_.x+p.y*_.y+p.z*_.z,-1,1),S=zt((1-v)/.28,0,1);if(S<=.001)continue;const M={x:x.x+p.x*this.segmentLength,y:x.y+p.y*this.segmentLength,z:x.z+p.z*this.segmentLength},y=i*o*(.25+.75*S);let w={x:(M.x-f.x)*y,y:(M.y-f.y)*y,z:(M.z-f.z)*y};const T=Math.hypot(w.x,w.y,w.z);if(!(T<=1e-8)){if(s?.pointContact){const E=this.#o(f,w,s),A=Math.hypot(E.x,E.y,E.z);if(A<T*.08){const D=un({x:x.x-f.x,y:x.y-f.y,z:x.z-f.z},{x:-p.x,y:-p.y,z:-p.z}),b=Math.min(c,Math.max(T*.45,this.segmentLength*.035)),F=this.#o(f,{x:D.x*b,y:D.y*b,z:D.z*b},s);w=Math.hypot(F.x,F.y,F.z)>A?F:E}else w=E}for(let E=d;E<this.rod.nodes.length&&E<d+l;E++){const A=this.rod.nodes[E];if(!A||A.pinned)break;const D=this.insertedCoordinate(E);if(this.#e(D)||E>d&&this.#l(E,t))break;const b=this.#u(A,D,w,t,s,c);h=h||b}}}return h}#n(t){const e=Sa(this[t],this.rod.nodes.length);return this[t]=e,Uo(e),e}#S(t,e=null){const n=this.#n("_straightenCorrections"),i=.35+.65*pn(0,1,t),s=this.#A(e),a=this.segmentLength*.18,o={x:0,y:0,z:0},c=this.rod.nodes.nodeStorage;if(c){const{x:u,y:d,z:f,pinned:m}=c,x=this.#t(),g=(_,v,S)=>{let M=S.x,y=S.y,w=S.z;if(s?.pointContact&&!this.#e(v)){const E=this.#o(this.rod.nodes[_],S,s);M=E.x,y=E.y,w=E.z}const T=Math.hypot(M,y,w);if(T>a){const E=a/T;M*=E,y*=E,w*=E}return o.x=u[_]+M,o.y=d[_]+y,o.z=f[_]+w,this.#x(o,v,e,!1)};for(let _=Math.max(1,x);_<this.rod.nodes.length-1;_++){if(m[_])continue;const v=this.insertedCoordinate(_),S=1-pn(this.sheathLength,this.sheathLength+this.segmentLength*5,v),M=this.straightening*i*(1-S*.45),y=n[_];y.x=((u[_-1]+u[_+1])*.5-u[_])*M,y.y=((d[_-1]+d[_+1])*.5-d[_])*M,y.z=((f[_-1]+f[_+1])*.5-f[_])*M,y.active=!0}for(let _=Math.max(1,x);_<this.rod.nodes.length-1;_++){const v=n[_];if(!v.active||m[_])continue;const S=this.insertedCoordinate(_),M=g(_,S,v);u[_]=M.x,d[_]=M.y,f[_]=M.z}const p=[2,4,8,12];for(const _ of p){const v=this.#n("_spanCorrections"),S=this.straightening*.13/Math.sqrt(_);for(let M=Math.max(_,x);M<this.rod.nodes.length-_;M++){if(m[M])continue;const y=v[M];y.x=((u[M-_]+u[M+_])*.5-u[M])*S,y.y=((d[M-_]+d[M+_])*.5-d[M])*S,y.z=((f[M-_]+f[M+_])*.5-f[M])*S,y.active=!0}for(let M=Math.max(_,x);M<this.rod.nodes.length-_;M++){const y=v[M];if(!y.active||m[M])continue;const w=this.insertedCoordinate(M),T=g(M,w,y);u[M]=T.x,d[M]=T.y,f[M]=T.z}}return}const l=(u,d,f)=>{let m=f.x,x=f.y,g=f.z;if(s?.pointContact&&!this.#e(d)){const _=this.#o(u,f,s);m=_.x,x=_.y,g=_.z}const p=Math.hypot(m,x,g);if(p>a){const _=a/p;m*=_,x*=_,g*=_}return o.x=u.x+m,o.y=u.y+x,o.z=u.z+g,this.#x(o,d,e,!1)};for(let u=1;u<this.rod.nodes.length-1;u++){const d=this.rod.nodes[u];if(d.pinned)continue;const f=this.rod.nodes[u-1],m=this.rod.nodes[u+1],x=this.insertedCoordinate(u),g=1-pn(this.sheathLength,this.sheathLength+this.segmentLength*5,x),p=this.straightening*i*(1-g*.45),_=n[u];_.x=((f.x+m.x)*.5-d.x)*p,_.y=((f.y+m.y)*.5-d.y)*p,_.z=((f.z+m.z)*.5-d.z)*p,_.active=!0}for(let u=1;u<this.rod.nodes.length-1;u++){const d=this.rod.nodes[u],f=n[u];if(!f.active||d.pinned)continue;const m=this.insertedCoordinate(u),x=l(d,m,f);Xs(d,x)}const h=[2,4,8,12];for(const u of h){const d=this.#n("_spanCorrections"),f=this.straightening*.13/Math.sqrt(u);for(let m=u;m<this.rod.nodes.length-u;m++){const x=this.rod.nodes[m];if(x.pinned)continue;const g=this.rod.nodes[m-u],p=this.rod.nodes[m+u],_=d[m];_.x=((g.x+p.x)*.5-x.x)*f,_.y=((g.y+p.y)*.5-x.y)*f,_.z=((g.z+p.z)*.5-x.z)*f,_.active=!0}for(let m=u;m<this.rod.nodes.length-u;m++){const x=this.rod.nodes[m],g=d[m];if(!g.active||x.pinned)continue;const p=this.insertedCoordinate(m),_=l(x,p,g);Xs(x,_)}}}#m(){if(this.foldUntangleStrength<=0||this.foldUntangleWindow<=0)return;const t=this.#n("_untangleCorrections"),e=zt(this.foldAngle,1,179),n=Math.max(1,Math.floor(this.foldUntangleWindow)),i=zt(this.foldUntangleStrength,0,1);for(let s=1;s<this.rod.nodes.length-1;s++){if(this.rod.nodes[s].pinned||this.insertedCoordinate(s)<=this.sheathLength+this.segmentLength+ni)continue;const c=this.rod.bendAngleAt?.(s)??0;if(c<=e)continue;const l=zt((c-e)/Math.max(1,180-e),0,1);for(let h=-n;h<=n;h++){const u=s+h,d=this.rod.nodes[u];if(!d||d.pinned)continue;const f=this.insertedCoordinate(u);if(this.#e(f))continue;const m=1-Math.abs(h)/(n+1),x=i*l*m;if(x<=0)continue;const g=this.routeSample(f).point;ro(t,u,(g.x-d.x)*x,(g.y-d.y)*x,(g.z-d.z)*x)}}for(let s=1;s<this.rod.nodes.length-1;s++){const a=this.rod.nodes[s],o=t[s];!o.active||a.pinned||td(a,o,1)}}#g(t=this.maxBendAngle,e=this.bendLimitStrength,n=this.bendLimitIterations,i=.45){if(n<=0||e<=0)return;const s=zt(t,1,179),a=s*Math.PI/180,o=2*this.segmentLength*Math.cos(a*.5),c=zt(e,0,1);for(let l=0;l<n;l++){const h=this.#n("_bendLimitCorrections");for(let u=1;u<this.rod.nodes.length-1;u++){const d=this.rod.nodes[u];if(d.pinned)continue;const f=this.insertedCoordinate(u);if(f<=this.sheathLength+this.segmentLength+ni)continue;const m=this.rod.bendAngleAt?.(u)??0;if(m<=s)continue;const x=this.rod.nodes[u-1],g=this.rod.nodes[u+1],p={x:g.x-x.x,y:g.y-x.y,z:g.z-x.z},_=Math.hypot(p.x,p.y,p.z),v=this.routeSample(f).tangent,S=un(p,v),M=zt((m-s)/Math.max(1,180-s),0,1),y=c*(.35+.65*M);if(_<o){const w=(o-_)*.5*y;x.pinned||ro(h,u-1,-S.x*w,-S.y*w,-S.z*w),g.pinned||ro(h,u+1,S.x*w,S.y*w,S.z*w)}ro(h,u,((x.x+g.x)*.5-d.x)*y*i,((x.y+g.y)*.5-d.y)*y*i,((x.z+g.z)*.5-d.z)*y*i)}for(let u=1;u<this.rod.nodes.length-1;u++){const d=this.rod.nodes[u],f=h[u];!f.active||d.pinned||td(d,f,1)}}}#v(t){const e=zt(t,1,179);for(let n=1;n<this.rod.nodes.length-1;n++){if(this.rod.nodes[n].pinned||this.insertedCoordinate(n)<=this.sheathLength+this.segmentLength+ni)continue;if((this.rod.bendAngleAt?.(n)??0)>e)return!0}return!1}#M(){let t=0;for(let e=0;e<this.rod.nodes.length-1;e++){const n=this.rod.nodes[e],i=this.rod.nodes[e+1];t=Math.max(t,Math.abs(ed(n,i)-this.segmentLength))}return t}#C(){let t=0;for(let e=1;e<this.rod.nodes.length-1;e++)this.rod.nodes[e].pinned||this.insertedCoordinate(e)<=this.sheathLength+this.segmentLength+ni||(t=Math.max(t,this.rod.bendAngleAt?.(e)??0));return t}#y(t){const e=Math.abs(t.x),n=Math.abs(t.y),i=e<.7?{x:1,y:0,z:0}:n<.7?{x:0,y:1,z:0}:{x:0,y:0,z:1};return un({x:t.y*i.z-t.z*i.y,y:t.z*i.x-t.x*i.z,z:t.x*i.y-t.y*i.x},{x:1,y:0,z:0})}#_(t,e){const n=zt(t,1,179)*Math.PI/180,i=Math.cos(n),s=Math.sin(n),a=zt(e,0,1);if(!(a<=0))for(let o=1;o<this.rod.nodes.length-1;o++){const c=this.rod.nodes[o-1],l=this.rod.nodes[o],h=this.rod.nodes[o+1];if(l.pinned||h.pinned||this.insertedCoordinate(o)<=this.sheathLength+this.segmentLength+ni)continue;const d=un({x:l.x-c.x,y:l.y-c.y,z:l.z-c.z},this.sheathDir),f={x:h.x-l.x,y:h.y-l.y,z:h.z-l.z},m=un(f,d),x=d.x*m.x+d.y*m.y+d.z*m.z;if(x>=i)continue;let g={x:m.x-d.x*x,y:m.y-d.y*x,z:m.z-d.z*x};const p=Math.hypot(g.x,g.y,g.z);p<1e-8?g=this.#y(d):(g.x/=p,g.y/=p,g.z/=p);const _=un({x:d.x*i+g.x*s,y:d.y*i+g.y*s,z:d.z*i+g.z*s},d),v={x:l.x+_.x*this.segmentLength,y:l.y+_.y*this.segmentLength,z:l.z+_.z*this.segmentLength};h.x+=(v.x-h.x)*a,h.y+=(v.y-h.y)*a,h.z+=(v.z-h.z)*a}}#E(t=null){const e=zt(this.tipBacktrackStrength,0,1);if(e<=0||this.rod.nodes.length<3)return!1;const n=this.rod.nodes.length-1,i=n-1,s=n-2,a=this.rod.nodes[n],o=this.rod.nodes[i],c=this.rod.nodes[s];if(a.pinned||o.pinned||this.insertedCoordinate(i)<=this.sheathLength+this.segmentLength+ni)return!1;const h=zt(this.tipBacktrackAngle,1,179)*Math.PI/180,u=Math.cos(h),d=Math.sin(h),f=un({x:o.x-c.x,y:o.y-c.y,z:o.z-c.z},this.sheathDir),m=un({x:a.x-o.x,y:a.y-o.y,z:a.z-o.z},f),x=f.x*m.x+f.y*m.y+f.z*m.z;if(x>=u)return!1;let g={x:m.x-f.x*x,y:m.y-f.y*x,z:m.z-f.z*x};const p=Math.hypot(g.x,g.y,g.z);p<1e-8?g=this.#y(f):(g.x/=p,g.y/=p,g.z/=p);const _=un({x:f.x*u+g.x*d,y:f.y*u+g.y*d,z:f.z*u+g.z*d},f),v=this.#A(t);let S={x:o.x+_.x*this.segmentLength,y:o.y+_.y*this.segmentLength,z:o.z+_.z*this.segmentLength};if(v?.pointContact){const y=un(this.#o(a,f,v),_),w=this.#x({x:a.x+y.x*this.segmentLength*.8,y:a.y+y.y*this.segmentLength*.8,z:a.z+y.z*this.segmentLength*.8},this.insertedCoordinate(n),t,!1),T=un({x:w.x-o.x,y:w.y-o.y,z:w.z-o.z},_);f.x*T.x+f.y*T.y+f.z*T.z>x&&(S=w)}const M=this.#x(S,this.insertedCoordinate(n),t,!1);return a.x+=(M.x-a.x)*e,a.y+=(M.y-a.y)*e,a.z+=(M.z-a.z)*e,a.vx*=.2,a.vy*=.2,a.vz*=.2,!0}#w(){if(this.foldGuardPasses<=0||this.foldGuardStrength<=0||!this.#v(this.foldGuardAngle))return!1;const t=Math.max(1,Math.floor(this.foldGuardPasses));let e=!1;for(let n=0;n<t&&(e=!0,this.#_(this.foldGuardAngle,this.foldGuardStrength),this.#g(this.foldGuardAngle,this.foldGuardStrength,1,this.foldGuardCenterPull),this.#r(Math.max(3,Math.ceil(this.lengthIterations*.5))),this.#r(2),!!this.#v(this.foldGuardAngle));n++);return e}#b(t){const e=Math.max(0,Math.floor(this.stabilityRepairPasses));if(e<=0)return!1;const n=Math.max(1e-4,this.stabilityRepairSegmentError),i=zt(this.stabilityRepairBendAngle,1,179),s=Math.max(0,Math.floor(this.stabilityRepairLengthIterations));let a=!1;for(let o=0;o<e;o++){const c=this.#M(),l=this.#C();if(c<=n&&l<=i)break;a=!0;const h=zt(c/n-1,0,1),u=zt((l-i)/Math.max(1,180-i),0,1),d=Math.max(h,u),f=zt(Math.min(this.stabilityRepairTargetBendAngle,i),1,179),m=zt(Math.max(this.foldGuardStrength,.72)*(.75+.25*d),0,1),x=Math.max(this.foldGuardCenterPull,1.1);this.#_(f,m),this.#g(f,m,2,x),this.#r(this.lengthIterations+s,t,!0),this.#h(t,!1),this.#g(f,m,1,x),this.#r(this.lengthIterations+Math.ceil(s*.5),t,!0),this.#h(t,!1),this.#r(Math.max(4,Math.ceil(this.lengthIterations*.5)),t,!0)}return a}#r(t,e=null,n=!1){const i=this.segmentLength,s=n?this.#A(e):null,a=!!s?.pointContact,o=this.#i(),c=this.rod.nodes.nodeStorage;if(!a&&c){const{x:l,y:h,z:u,pinned:d}=c;for(let f=0;f<t;f++){for(let m=o;m<this.rod.nodes.length-1;m++){const x=l[m+1]-l[m],g=h[m+1]-h[m],p=u[m+1]-u[m],_=Math.hypot(x,g,p);if(_<1e-8)continue;const v=(_-i)/_,S=d[m]?0:1,M=d[m+1]?0:1,y=S+M;if(y<=0)continue;const w=S/y,T=M/y;if(S){const E=v*w;l[m]+=x*E,h[m]+=g*E,u[m]+=p*E}if(M){const E=-v*T;l[m+1]+=x*E,h[m+1]+=g*E,u[m+1]+=p*E}}this.constrainSheath()}return}for(let l=0;l<t;l++){for(let h=o;h<this.rod.nodes.length-1;h++){const u=this.rod.nodes[h],d=this.rod.nodes[h+1],f=d.x-u.x,m=d.y-u.y,x=d.z-u.z,g=Math.hypot(f,m,x);if(g<1e-8)continue;const p=(g-i)/g,_=u.pinned?0:1,v=d.pinned?0:1,S=_+v;if(S<=0)continue;const M=_/S,y=v/S;if(!a){if(_){const w=p*M;u.x+=f*w,u.y+=m*w,u.z+=x*w}if(v){const w=-p*y;d.x+=f*w,d.y+=m*w,d.z+=x*w}continue}if(_){let w={x:f*p*M,y:m*p*M,z:x*p*M};this.#e(this.insertedCoordinate(h))||(w=this.#o(u,w,s)),u.x+=w.x,u.y+=w.y,u.z+=w.z}if(v){let w={x:-f*p*y,y:-m*p*y,z:-x*p*y};this.#e(this.insertedCoordinate(h+1))||(w=this.#o(d,w,s)),d.x+=w.x,d.y+=w.y,d.z+=w.z}}this.constrainSheath()}}#h(t,e){const n=Ti();this.performanceStats.projectGuidewireCalls++,this.#N(t,e),this.#U(t,e),this.#N(t,e),this.performanceStats.projectMs+=Ti()-n}#N(t,e){const n=this.#t(),i=this.rod.nodes.nodeStorage;if(i){const{x:s,y:a,z:o,pinned:c}=i,l=this._projectNodePoint;for(let h=n;h<this.rod.nodes.length;h++){if(c[h])continue;const u=this.insertedCoordinate(h);if(this.#e(u))continue;this.performanceStats.nodeProjectionCount++,l.x=s[h],l.y=a[h],l.z=o[h];const d=this.#x(l,u,t,e);s[h]=d.x,a[h]=d.y,o[h]=d.z}return}for(let s=n;s<this.rod.nodes.length;s++){const a=this.rod.nodes[s];if(a.pinned)continue;const o=this.insertedCoordinate(s);if(this.#e(o))continue;this.performanceStats.nodeProjectionCount++;const c=this.#x(a,o,t,e);Xs(a,c)}}#U(t,e){const n={x:0,y:0,z:0},i=this.segmentLength*this.maxSegmentProjectionStep,s=this.#i(),a=this.rod.nodes.nodeStorage;if(a){const{x:o,y:c,z:l,pinned:h}=a;for(let u=s;u<this.rod.nodes.length-1;u++)if(!(h[u]&&h[u+1]))for(const d of this.segmentSamples){const f=this.insertedCoordinate(u+d);if(this.#e(f))continue;this.performanceStats.segmentSampleCount++;const m=h[u]?0:1-d,x=h[u+1]?0:d;n.x=o[u]*(1-d)+o[u+1]*d,n.y=c[u]*(1-d)+c[u+1]*d,n.z=l[u]*(1-d)+l[u+1]*d;const g=this.#x(n,f,t,e);let p=(g.x-n.x)*this.segmentProjectionBlend,_=(g.y-n.y)*this.segmentProjectionBlend,v=(g.z-n.z)*this.segmentProjectionBlend;const S=Math.hypot(p,_,v);if(S>i){const y=i/S;p*=y,_*=y,v*=y}const M=m*m+x*x;if(!(M<=1e-8)){if(this.performanceStats.segmentProjectionCount++,m){const y=m/M;o[u]+=p*y,c[u]+=_*y,l[u]+=v*y}if(x){const y=x/M;o[u+1]+=p*y,c[u+1]+=_*y,l[u+1]+=v*y}}}return}for(let o=s;o<this.rod.nodes.length-1;o++){const c=this.rod.nodes[o],l=this.rod.nodes[o+1];if(!(c.pinned&&l.pinned))for(const h of this.segmentSamples){const u=this.insertedCoordinate(o+h);if(this.#e(u))continue;this.performanceStats.segmentSampleCount++;const d=c.pinned?0:1-h,f=l.pinned?0:h;n.x=c.x*(1-h)+l.x*h,n.y=c.y*(1-h)+l.y*h,n.z=c.z*(1-h)+l.z*h;const m=this.#x(n,u,t,e);let x=(m.x-n.x)*this.segmentProjectionBlend,g=(m.y-n.y)*this.segmentProjectionBlend,p=(m.z-n.z)*this.segmentProjectionBlend;const _=Math.hypot(x,g,p);if(_>i){const S=i/_;x*=S,g*=S,p*=S}const v=d*d+f*f;if(!(v<=1e-8)){if(this.performanceStats.segmentProjectionCount++,d){const S=d/v;c.x+=x*S,c.y+=g*S,c.z+=p*S}if(f){const S=f/v;l.x+=x*S,l.y+=g*S,l.z+=p*S}}}}}#x(t,e,n,i){const s=n?.meshCollider||n?.lumenMeshCollider||null;if(s?.pointContact&&!this.#e(e)){let a=t;if(this.lumenSampler){const c=this.#L(t,e);a=c.projected,i&&(c.breach?this.#d(this.breachPoints,t):c.radialMargin<=Ai&&this.#d(this.contactPoints,t))}const o=this.#T(s,a,this.meshClearance,!1,this._projectContact);return o?.violation&&o.target?(i&&(Number.isFinite(o.signedDistance)&&o.signedDistance<0?this.#d(this.breachPoints,a):this.#d(this.contactPoints,a)),{x:o.target.x,y:o.target.y,z:o.target.z}):(i&&Number.isFinite(o?.distance)&&o.distance<=Ai&&this.#d(this.contactPoints,a),a)}return this.#B(t,e,i)}#A(t){return t?.meshCollider||t?.lumenMeshCollider||null}#T(t,e,n,i=!1,s=null){return i?this.performanceStats.diagnosticPointContactCount++:this.performanceStats.pointContactCount++,t.pointContact(e,n,s)}#o(t,e,n){if(!n?.pointContact)return{x:e.x,y:e.y,z:e.z};const i={x:t.x+e.x,y:t.y+e.y,z:t.z+e.z},s=this.#T(n,t,this.meshClearance,!1,this._slidePointContact),a=this.#T(n,i,this.meshClearance,!1,this._slideTargetContact),o=s?.violation||Number.isFinite(s?.signedDistance)&&s.signedDistance<=this.meshClearance+Ai,c=a?.violation||Number.isFinite(a?.signedDistance)&&a.signedDistance<=this.meshClearance+Ai;if(!o&&!c)return{x:e.x,y:e.y,z:e.z};const h=(a?.violation||c?a:s)?.normal||s?.normal||a?.normal,u=h?Math.hypot(h.x,h.y,h.z):0;if(u<1e-8)return{x:e.x,y:e.y,z:e.z};const d=h.x/u,f=h.y/u,m=h.z/u,x=e.x*d+e.y*f+e.z*m;return x<=0?{x:e.x,y:e.y,z:e.z}:{x:e.x-d*x,y:e.y-f*x,z:e.z-m*x}}#B(t,e,n){if(!this.lumenSampler)return t;const i=this.#L(t,e);return n&&(i.breach?this.#d(this.breachPoints,t):i.radialMargin<=Ai&&this.#d(this.contactPoints,t)),i.projected}#L(t,e){const n=this.routeSample(e),i=Math.max(.5,(n.radius||1)-this.lumenClearance),s=n.tangent||this.sheathDir,a=Math.hypot(s.x,s.y,s.z),o=a<1e-8?this.sheathDir.x:s.x/a,c=a<1e-8?this.sheathDir.y:s.y/a,l=a<1e-8?this.sheathDir.z:s.z/a,h=t.x-n.point.x,u=t.y-n.point.y,d=t.z-n.point.z;let f=h*o+u*c+d*l,m=h-o*f,x=u-c*f,g=d-l*f,p=Math.hypot(m,x,g);const _=Math.max(this.segmentLength*.5,this.segmentLength*this.axialWindowScale),v=p>i+1e-4;if(p>i){const M=i/Math.max(1e-8,p);m*=M,x*=M,g*=M,p=i}f=zt(f,-_,_);const S=this._lumenConstraintState;return S.projected.x=n.point.x+o*f+m,S.projected.y=n.point.y+c*f+x,S.projected.z=n.point.z+l*f+g,S.radialMargin=i-p,S.axialOffset=f,S.axialWindow=_,S.breach=v,S}#F(t,e,n=null){const i=1/Math.max(e,1e-6),s=n?.meshCollider||n?.lumenMeshCollider||null,a=this.rod.nodes.nodeStorage;if(a&&!s?.pointContact){const{x:o,y:c,z:l,vx:h,vy:u,vz:d}=a;for(let f=0;f<this.rod.nodes.length;f++){const m=o[f]-t[f].x,x=c[f]-t[f].y,g=l[f]-t[f].z,p=m*m+x*x+g*g>4e-4?.08:0;h[f]=m*i*p,u[f]=x*i*p,d[f]=g*i*p}return}for(let o=0;o<this.rod.nodes.length;o++){const c=this.rod.nodes[o],l=c.x-t[o].x,h=c.y-t[o].y,u=c.z-t[o].z,d=l*l+h*h+u*u>4e-4?.08:0;let f=l*i*d,m=h*i*d,x=u*i*d;const g=this.insertedCoordinate(o);if(s?.pointContact&&!this.#e(g)){const p=this.#T(s,c,this.meshClearance,!1,this._zeroVelocityContact),_=p?.normal;if((p?.violation||Number.isFinite(p?.signedDistance)&&p.signedDistance<=this.meshClearance+Ai)&&_){const S=f*_.x+m*_.y+x*_.z;S>0&&(f-=_.x*S,m-=_.y*S,x-=_.z*S)}}c.vx=f,c.vy=m,c.vz=x}}#d(t,e,n=420){t.length>=n||t.push({x:e.x,y:e.y,z:e.z})}#e(t){return t<=this.sheathLength+ni}}const NS=16384,FS=64,US=20,BS=10,zS=30;function OS(r,{segmentLength:t=r.segmentLength,bodyBendingStiffness:e=NS,tipBendingStiffness:n=FS,softTipLength:i=US,bodyMaxBendAngle:s=BS,tipMaxBendAngle:a=zS}={}){const o=r.nodes.length-1;for(let c=0;c<r.nodes.length;c++){const h=(o-c)*t<i;r.nodes[c].bendingStiffness=h?n:e,r.nodes[c].bendAngleLimit=h?a:s}return r}const GS="OETCOLL1",Nh=8,Qc=Nh+4,VS=1;new TextEncoder;const kS=new TextDecoder,HS={Float32Array,Uint32Array,Int16Array,Uint8Array,Int8Array};function id(r,t=8){return Math.ceil(r/t)*t}function WS(r){let t="";for(let e=0;e<Nh;e++)t+=String.fromCharCode(r[e]);return t}function np(r){if(!(r instanceof ArrayBuffer))throw new TypeError("Collision asset must be an ArrayBuffer");if(r.byteLength<Qc)throw new Error("Collision asset is truncated");const t=new Uint8Array(r),e=WS(t);if(e!==GS)throw new Error(`Unexpected collision asset magic: ${e}`);const n=new DataView(r).getUint32(Nh,!0),i=Qc+n;if(i>r.byteLength)throw new Error("Collision asset manifest is truncated");const s=JSON.parse(kS.decode(t.subarray(Qc,i)));if(s.version!==VS)throw new Error(`Unsupported collision asset version: ${s.version}`);const a={};let o=id(i);for(const c of s.sections||[]){const l=HS[c.type];if(!l)throw new Error(`Unsupported collision asset array type: ${c.type}`);const h=c.length*l.BYTES_PER_ELEMENT;if(o+h>r.byteLength)throw new Error(`Collision asset section is truncated: ${c.name}`);a[c.name]=new l(r,o,c.length),o=id(o+h)}return{metadata:s,arrays:a,buffer:r}}const Ur=1e-8,Yi=0,Bo=1,tl=2,XS=3,YS=4,ip=5,on=0,vn=1,cs=2;function Ys(r,t,e,n){return r.x=t,r.y=e,r.z=n,r}function sd(){const r=new Float64Array(6);return r[Yi]=-1/0,r[Bo]=1,r[ip]=-1,r}function sp(){return{inside:!1,signedDistance:-1/0,distance:1/0,inward:{x:1,y:0,z:0},normal:{x:-1,y:0,z:0},closestPoint:{x:0,y:0,z:0},lowerSliceIndex:-1,upperSliceIndex:-1}}class qS{constructor(t,e){this.metadata=t.lumen,this.sliceYs=e.lumenSliceYs,this.sliceContourOffsets=e.lumenSliceContourOffsets,this.contourPointOffsets=e.lumenContourPointOffsets,this.contourBounds=e.lumenContourBounds,this.contourSamples=e.lumenContourSamples,this.points=e.lumenPoints,this.pointQuantization=this.points instanceof Int16Array?this.metadata.pointQuantization||.02:1,this.axisBases=e.lumenAxisBases||new Float32Array([1,0,0,0,1,0,0,0,1]),this.axisSliceOffsets=e.lumenAxisSliceOffsets||new Uint32Array([0,this.sliceYs.length]),this.axisCount=Math.max(1,this.axisSliceOffsets.length-1),this._lower=sd(),this._upper=sd(),this._interval=new Float64Array(3),this._lastLower=new Int32Array(this.axisCount),this._lastUpper=new Int32Array(this.axisCount);for(let n=0;n<this.axisCount;n++){const i=this.axisSliceOffsets[n],s=this.axisSliceOffsets[n+1];this._lastLower[n]=i,this._lastUpper[n]=Math.min(i+1,Math.max(i,s-1))}}query(t,e=null){return this.queryCoordinates(t.x,t.y,t.z,e)}isInsideCoordinates(t,e,n){for(let i=0;i<this.axisCount;i++){const s=i*9,a=t*this.axisBases[s]+e*this.axisBases[s+1]+n*this.axisBases[s+2],o=t*this.axisBases[s+3]+e*this.axisBases[s+4]+n*this.axisBases[s+5],c=t*this.axisBases[s+6]+e*this.axisBases[s+7]+n*this.axisBases[s+8],l=this.#i(o,i);if(l[on]<0||!this.#t(l[on],a,c)&&(l[vn]===l[on]||!this.#t(l[vn],a,c)))continue;const h=this.#a(l[on],a,c,this._lower),u=l[vn]===l[on]?h:this.#a(l[vn],a,c,this._upper),d=l[cs];if(h[Yi]*(1-d)+u[Yi]*d>=0)return!0}return!1}#t(t,e,n){const i=this.sliceContourOffsets[t],s=this.sliceContourOffsets[t+1];for(let a=i;a<s;a++){const o=a*4;if(e>=this.contourBounds[o]&&e<=this.contourBounds[o+1]&&n>=this.contourBounds[o+2]&&n<=this.contourBounds[o+3])return!0}return!1}queryCoordinates(t,e,n,i=null){const s=i||sp();if(!this.sliceYs.length)return s.inside=!1,s.signedDistance=-1/0,s.distance=1/0,Ys(s.inward,1,0,0),Ys(s.normal,-1,0,0),Ys(s.closestPoint,t,e,n),s.lowerSliceIndex=-1,s.upperSliceIndex=-1,s;let a=-1/0,o=1,c=0,l=0,h=-1,u=-1;for(let d=0;d<this.axisCount;d++){const f=d*9,m=t*this.axisBases[f]+e*this.axisBases[f+1]+n*this.axisBases[f+2],x=t*this.axisBases[f+3]+e*this.axisBases[f+4]+n*this.axisBases[f+5],g=t*this.axisBases[f+6]+e*this.axisBases[f+7]+n*this.axisBases[f+8],p=this.#i(x,d);if(p[on]<0)continue;const _=this.#a(p[on],m,g,this._lower),v=p[vn]===p[on]?_:this.#a(p[vn],m,g,this._upper),S=p[cs],M=_[Yi]*(1-S)+v[Yi]*S;if(M<=a)continue;const y=Math.max(Ur,Math.abs(this.sliceYs[p[vn]]-this.sliceYs[p[on]])),w=p[vn]===p[on]?0:Math.max(-.85,Math.min(.85,(v[Yi]-_[Yi])/y));let T=_[Bo]*(1-S)+v[Bo]*S,E=w,A=_[tl]*(1-S)+v[tl]*S;const D=Math.sqrt(T*T+E*E+A*A);D>Ur?(T/=D,E/=D,A/=D):(T=1,E=0,A=0);let b=this.axisBases[f]*T+this.axisBases[f+3]*E+this.axisBases[f+6]*A,F=this.axisBases[f+1]*T+this.axisBases[f+4]*E+this.axisBases[f+7]*A,L=this.axisBases[f+2]*T+this.axisBases[f+5]*E+this.axisBases[f+8]*A;const N=Math.sqrt(b*b+F*F+L*L)||1;b/=N,F/=N,L/=N,a=M,o=b,c=F,l=L,h=p[on],u=p[vn]}return s.inside=a>=0,s.signedDistance=a,s.distance=Math.abs(a),Ys(s.inward,o,c,l),Ys(s.normal,-o,-c,-l),Ys(s.closestPoint,t-o*a,e-c*a,n-l*a),s.lowerSliceIndex=h,s.upperSliceIndex=u,s}#i(t,e){const n=this._interval,i=this.axisSliceOffsets[e],s=this.axisSliceOffsets[e+1]-1;if(s<i)return n[on]=-1,n[vn]=-1,n[cs]=0,n;if(s===i||t<=this.sliceYs[i])return this._lastLower[e]=i,this._lastUpper[e]=i,n[on]=i,n[vn]=i,n[cs]=0,n;if(t>=this.sliceYs[s])return this._lastLower[e]=s,this._lastUpper[e]=s,n[on]=s,n[vn]=s,n[cs]=0,n;let a=this._lastLower[e],o=this._lastUpper[e];if(a>=i&&o>a&&o<=s&&this.sliceYs[a]<=t&&t<=this.sliceYs[o]){const u=Math.max(Ur,this.sliceYs[o]-this.sliceYs[a]);return n[on]=a,n[vn]=o,n[cs]=(t-this.sliceYs[a])/u,n}let c=i,l=s;for(;l-c>1;){const u=Math.floor((c+l)*.5);this.sliceYs[u]<=t?c=u:l=u}a=c,o=l,this._lastLower[e]=a,this._lastUpper[e]=o;const h=Math.max(Ur,this.sliceYs[o]-this.sliceYs[a]);return n[on]=a,n[vn]=o,n[cs]=(t-this.sliceYs[a])/h,n}#a(t,e,n,i){let s=-1/0,a=1,o=0,c=e,l=n,h=-1;const u=this.sliceContourOffsets[t],d=this.sliceContourOffsets[t+1];for(let f=u;f<d;f++){const m=f*4;let x=0,g=0;if(e<this.contourBounds[m]?x=this.contourBounds[m]-e:e>this.contourBounds[m+1]&&(x=e-this.contourBounds[m+1]),n<this.contourBounds[m+2]?g=this.contourBounds[m+2]-n:n>this.contourBounds[m+3]&&(g=n-this.contourBounds[m+3]),Number.isFinite(s)&&s<0&&-Math.sqrt(x*x+g*g)<=s)continue;const p=this.contourPointOffsets[f],_=this.contourPointOffsets[f+1];if(_<=p)continue;let v=!1,S=e,M=n,y=1/0,w=_-1;for(let F=p;F<_;F++){const L=this.points[F*2]*this.pointQuantization,N=this.points[F*2+1]*this.pointQuantization,U=this.points[w*2]*this.pointQuantization,k=this.points[w*2+1]*this.pointQuantization;N>n!=k>n&&e<(U-L)*(n-N)/(k-N+1e-12)+L&&(v=!v);const O=U-L,H=k-N,j=O*O+H*H||1,J=Math.max(0,Math.min(1,((e-L)*O+(n-N)*H)/j)),nt=L+O*J,V=N+H*J,$=e-nt,st=n-V,at=$*$+st*st;at<y&&(y=at,S=nt,M=V),w=F}const T=Math.sqrt(y),E=v?T:-T;if(E<=s)continue;let A=v?e-S:S-e,D=v?n-M:M-n;const b=Math.sqrt(A*A+D*D);if(b>Ur)A/=b,D/=b;else{A=this.contourSamples[f*2]-S,D=this.contourSamples[f*2+1]-M;const F=Math.sqrt(A*A+D*D)||1;A/=F,D/=F}s=E,a=A,o=D,c=S,l=M,h=f}return i[Yi]=s,i[Bo]=a,i[tl]=o,i[XS]=c,i[YS]=l,i[ip]=h,i}}function ZS(r,t){return!r?.lumen||!t?.lumenSliceYs?.length?null:new qS(r,t)}const jS="centerline-safe-core",$S="sparse-sdf",KS="sparse-sdf-bvh",Fh="fallback",JS="centerline-estimate",$n=1e-8,la=1<<17,rp=la>>1,QS=rp-1,el=200,Br=65535,rd=0,ad=1,oo=2,od=3,co=4,cd=5,ld=6,hd=7,ud=8,dd=9,nl=10,il=11,fd=12,pd=13,lo=14,tM=15,md=0,zr=1,eM=2,gd=3,xd=4,_d=5,ho=6,Or=0,vd=1,Gr=2,Vr=3,sl=4,rl=5,al=6,ol=7,kr=8,Hr=9,Wr=10,Xr=11,Pn=0,ii=1,si=2,ri=3,Yr=4;function nM(r,t,e){return Math.max(t,Math.min(e,r))}function ai(r,t){r.inside=t.inside,r.violation=t.violation,r.conservative=t.conservative,r.source=t.source;const e=r.values,n=t.values;return e[0]=n[0],e[1]=n[1],e[2]=n[2],e[3]=n[3],e[4]=n[4],e[5]=n[5],e[6]=n[6],qr(r.point.values,t.point.values),qr(r.target.values,t.target.values),qr(r.closestPoint.values,t.closestPoint.values),qr(r.normal.values,t.normal.values),qr(r.inward.values,t.inward.values),r}function qr(r,t){r[0]=t[0],r[1]=t[1],r[2]=t[2]}class Zr{constructor(t=0,e=0,n=0){this.values=new Float64Array([t,e,n])}get x(){return this.values[0]}set x(t){this.values[0]=t}get y(){return this.values[1]}set y(t){this.values[1]=t}get z(){return this.values[2]}set z(t){this.values[2]=t}}class iM{constructor(){this.values=new Float64Array([-1/0,-1/0,1/0,1/0,-1,0,1]),this.inside=!1,this.violation=!1,this.conservative=!1,this.source=Fh,this.point=new Zr,this.target=new Zr,this.closestPoint=new Zr,this.normal=new Zr(1,0,0),this.inward=new Zr(1,0,0)}get signedDistance(){return this.values[0]}set signedDistance(t){this.values[0]=t}get signedGap(){return this.values[1]}set signedGap(t){this.values[1]=t}get distance(){return this.values[2]}set distance(t){this.values[2]=t}get penetration(){return this.values[3]}set penetration(t){this.values[3]=t}get branchId(){return this.values[4]}set branchId(t){this.values[4]=t}get segmentT(){return this.values[5]}set segmentT(t){this.values[5]=t}get timeOfImpact(){return this.values[6]}set timeOfImpact(t){this.values[6]=t}}class sM{constructor(){this.values=new Float64Array(12),this.found=!1,this.branchId=-1,this.signedDistance=-1/0,this.safeDistance=-1/0,this.safeBranchId=-1,this.safeInwardX=1,this.nearestDistance=1/0,this.inwardX=1}get branchId(){return this.values[0]}set branchId(t){this.values[0]=t}get t(){return this.values[1]}set t(t){this.values[1]=t}get signedDistance(){return this.values[2]}set signedDistance(t){this.values[2]=t}get safeDistance(){return this.values[3]}set safeDistance(t){this.values[3]=t}get safeBranchId(){return this.values[4]}set safeBranchId(t){this.values[4]=t}get safeInwardX(){return this.values[5]}set safeInwardX(t){this.values[5]=t}get safeInwardY(){return this.values[6]}set safeInwardY(t){this.values[6]=t}get safeInwardZ(){return this.values[7]}set safeInwardZ(t){this.values[7]=t}get nearestDistance(){return this.values[8]}set nearestDistance(t){this.values[8]=t}get inwardX(){return this.values[9]}set inwardX(t){this.values[9]=t}get inwardY(){return this.values[10]}set inwardY(t){this.values[10]=t}get inwardZ(){return this.values[11]}set inwardZ(t){this.values[11]=t}}class rM{constructor(){this.values=new Float64Array([-1/0,1,0,0,-1]),this.conservative=!1,this.source=Fh}get signedDistance(){return this.values[0]}set signedDistance(t){this.values[0]=t}get inwardX(){return this.values[1]}set inwardX(t){this.values[1]=t}get inwardY(){return this.values[2]}set inwardY(t){this.values[2]=t}get inwardZ(){return this.values[3]}set inwardZ(t){this.values[3]=t}get branchId(){return this.values[4]}set branchId(t){this.values[4]=t}}function Qn(){return new iM}function aM(){return new Uint32Array(tM)}class oM{constructor(t,{fallbackCollider:e=null,fallbackGeometry:n=null,bvhValidationDistance:i=.85,capsuleBvhValidation:s=!0}={}){const a=t instanceof ArrayBuffer?np(t):t;if(!a?.metadata||!a?.arrays)throw new TypeError("Decoded collision asset is required");this.metadata=a.metadata,this.arrays=a.arrays,this.fallbackCollider=e,this.fallbackGeometry=n,this.bvhValidationDistance=i,this.capsuleBvhValidationGap=s===!0?.05:Number.isFinite(s)?s:-1/0,this.packedLumenField=ZS(a.metadata,a.arrays),this.centerline=a.arrays.centerlineSegments,this.centerlineStride=a.metadata.centerline.stride,this.broadPhaseOffsets=a.arrays.broadPhaseOffsets,this.broadPhaseIds=a.arrays.broadPhaseIds,this.sdfBrickKeys=a.arrays.sdfBrickKeys,this.sdfDistances=a.arrays.sdfDistances,this.sdfInsideBits=a.arrays.sdfInsideBits||null;const o=a.metadata.sdf;this.voxelSize=o.voxelSize,this.brickSize=o.brickSize,this.valuesPerBrick=this.brickSize**3,this.sdfQuantization=o.distanceQuantization??o.quantization,this.sdfOrigin=o.origin,this.sdfDimensions=o.dimensions;const c=this.sdfDimensions[0]*this.sdfDimensions[1]*this.sdfDimensions[2];if(this.sdfBrickKeys.length>=Br)throw new RangeError("Sparse SDF has too many bricks for its runtime lookup");this.sdfBrickLookup=new Uint16Array(c),this.sdfBrickLookup.fill(Br);for(let h=0;h<this.sdfBrickKeys.length;h++)this.sdfBrickLookup[this.sdfBrickKeys[h]]=h;this.signCacheKeyLow=new Int32Array(la),this.signCacheKeyHigh=new Int32Array(la),this.signCacheInside=new Uint8Array(la),this.signCacheValid=new Uint8Array(la),this.signCacheVictim=new Uint8Array(rp);const l=a.metadata.broadPhase;this.broadPhaseOrigin=l.origin,this.broadPhaseDimensions=l.dimensions,this.broadPhaseCellSize=l.cellSize,this._sdfCornerScratch=new Float64Array(8),this._capsuleCoordinateScratch=new Float64Array(7),this._centerlineQueryScratch=new Float64Array(3),this.runtimeBytes=a.metadata.decodedBytes+this.sdfBrickLookup.byteLength+this.signCacheKeyLow.byteLength+this.signCacheKeyHigh.byteLength+this.signCacheInside.byteLength+this.signCacheValid.byteLength+this.signCacheVictim.byteLength+this._sdfCornerScratch.byteLength+this._capsuleCoordinateScratch.byteLength+this._centerlineQueryScratch.byteLength,this.stats=aM(),this._centerlineState=new sM,this._distanceState=new rM,this._point={x:0,y:0,z:0},this._skipBvhValidation=!1,this._bvhPoint=new C,this._bvhClosest={point:new C,distance:1/0,faceIndex:-1},this._lumenQuery=sp(),this._capsuleContact=Qn(),this._capsuleEndpointContact=Qn(),this._capsuleEndpointX=NaN,this._capsuleEndpointY=NaN,this._capsuleEndpointZ=NaN,this._capsuleEndpointRadius=NaN,this._sweepContact=Qn(),this._sweepProbe=Qn(),this._fallbackContact={query:{inward:{x:0,y:0,z:0},normal:{x:0,y:0,z:0},closestPoint:{x:0,y:0,z:0}},target:{x:0,y:0,z:0},closestPoint:{x:0,y:0,z:0},inward:{x:0,y:0,z:0},normal:{x:0,y:0,z:0}}}resetStats(){this.stats.fill(0)}getStats(){const t=this.stats;return{pointQueries:t[rd],capsuleQueries:t[ad],capsuleSamples:t[oo],sweepQueries:t[od],sweepSamples:t[co],batchQueries:t[cd],safeCoreHits:t[ld],sdfHits:t[hd],bvhRefinements:t[ud],signRefinements:t[dd],signCacheHits:t[nl],signCacheMisses:t[il],fallbackHits:t[fd],centerlineEstimateHits:t[pd],resultAllocations:t[lo],runtimeBytes:this.runtimeBytes}}setFallbackCollider(t){this.fallbackCollider=t}setFallbackGeometry(t){this.fallbackGeometry=t}querySphere(t,e=0,n=null){n||this.stats[lo]++;const i=n||Qn(),s=t.x??t[0]??0,a=t.y??t[1]??0,o=t.z??t[2]??0;return this.#t(s,a,o,Math.max(0,e||0),i)}#t(t,e,n,i,s){this.stats[rd]++;const a=this.#a(t,e,n,i),o=a.values,c=o[Pn],l=c-i,h=Math.max(0,-l),u=l<0,d=o[ii],f=o[si],m=o[ri],x=s.values;s.inside=c>=0,s.violation=u,s.conservative=a.conservative,x[md]=c,x[zr]=l,x[eM]=Math.max(0,c),x[gd]=h,x[xd]=o[Yr],x[_d]=0,x[ho]=u?0:1,s.source=a.source;const g=s.point.values;g[0]=t,g[1]=e,g[2]=n;const p=s.normal.values;p[0]=d,p[1]=f,p[2]=m;const _=s.inward.values;_[0]=d,_[1]=f,_[2]=m;const v=s.closestPoint.values;v[0]=t-d*c,v[1]=e-f*c,v[2]=n-m*c;const S=s.target.values;return S[0]=t+d*h,S[1]=e+f*h,S[2]=n+m*h,s}queryCapsule(t,e,n=0,i=null){const s=t.x??t[0]??0,a=t.y??t[1]??0,o=t.z??t[2]??0,c=e.x??e[0]??0,l=e.y??e[1]??0,h=e.z??e[2]??0;return this.queryCapsuleCoordinates(s,a,o,c,l,h,n,i)}queryCapsuleCoordinates(t,e,n,i,s,a,o=0,c=null){const l=this._capsuleCoordinateScratch;return l[0]=t,l[1]=e,l[2]=n,l[3]=i,l[4]=s,l[5]=a,l[6]=o,this.#i(c)}queryCapsuleSoA(t,e,n,i,s,a=null){const o=this._capsuleCoordinateScratch;return o[0]=t[s],o[1]=e[s],o[2]=n[s],o[3]=t[s+1],o[4]=e[s+1],o[5]=n[s+1],o[6]=Math.max(i[s],i[s+1]),this.#i(a)}#i(t){const e=this._capsuleCoordinateScratch,n=e[0],i=e[1],s=e[2],a=e[3],o=e[4],c=e[5],l=e[6];t||this.stats[lo]++;const h=t||Qn(),u=Math.max(0,l||0),d=a-n,f=o-i,m=c-s,x=Math.sqrt(d*d+f*f+m*m),g=Math.max(this.voxelSize*4,Math.max(.5,u)),p=Math.max(1,Math.ceil(x/g));let _=1/0,v=0,S=0;this.stats[ad]++,this._skipBvhValidation=!0;const M=n===this._capsuleEndpointX&&i===this._capsuleEndpointY&&s===this._capsuleEndpointZ&&u===this._capsuleEndpointRadius;let y;M?y=ai(this._capsuleContact,this._capsuleEndpointContact):(y=this.#t(n,i,s,u,this._capsuleContact),this.stats[oo]++),_=y.values[zr];const w=_,T=y.inward.values[0],E=y.inward.values[1],A=y.inward.values[2];let D=w,b=1;ai(h,y),x>$n&&(y=this.#t(a,o,c,u,this._capsuleContact),this.stats[oo]++,ai(this._capsuleEndpointContact,y),this._capsuleEndpointX=a,this._capsuleEndpointY=o,this._capsuleEndpointZ=c,this._capsuleEndpointRadius=u,D=y.values[zr],b=T*y.inward.values[0]+E*y.inward.values[1]+A*y.inward.values[2],D<_&&(_=D,v=1,S=p,ai(h,y))),x<=$n&&(ai(this._capsuleEndpointContact,y),this._capsuleEndpointX=a,this._capsuleEndpointY=o,this._capsuleEndpointZ=c,this._capsuleEndpointRadius=u);const F=p>1&&(Math.min(w,D)<=this.voxelSize||b<.85);for(let L=1;F&&L<p;L++){const N=L/p;y=this.#t(n+d*N,i+f*N,s+m*N,u,this._capsuleContact),this.stats[oo]++;const U=y.values[zr];U<_&&(_=U,v=N,S=L,ai(h,y))}return this._skipBvhValidation=!1,_<=this.capsuleBvhValidationGap&&(v=S/p,ai(h,this.#t(n+d*v,i+f*v,s+m*v,u,this._capsuleContact))),h.values[_d]=v,h}sweepSphere(t,e,n=0,i=null){i||this.stats[lo]++;const s=i||Qn(),a=t.x??t[0]??0,o=t.y??t[1]??0,c=t.z??t[2]??0,l=e.x??e[0]??0,h=e.y??e[1]??0,u=e.z??e[2]??0,d=l-a,f=h-o,m=u-c,x=Math.sqrt(d*d+f*f+m*m),g=Math.max(this.voxelSize*.5,Math.max(.1,n*.5)),p=Math.max(1,Math.ceil(x/g));this.stats[od]++,this._point.x=a,this._point.y=o,this._point.z=c;let _=this.#t(this._point.x,this._point.y,this._point.z,Math.max(0,n||0),this._sweepContact);if(this.stats[co]++,_.violation)return ai(s,_),s.values[ho]=0,s;for(let v=1;v<=p;v++){const S=v/p;this._point.x=a+d*S,this._point.y=o+f*S,this._point.z=c+m*S;const M=this.#t(this._point.x,this._point.y,this._point.z,Math.max(0,n||0),this._sweepProbe);if(this.stats[co]++,!M.violation){const T=_;_=M,this._sweepProbe=T;continue}let y=(v-1)/p,w=S;for(let T=0;T<7;T++){const E=(y+w)*.5;this._point.x=a+d*E,this._point.y=o+f*E,this._point.z=c+m*E;const A=this.#t(this._point.x,this._point.y,this._point.z,Math.max(0,n||0),this._sweepContact);this.stats[co]++,A.violation?w=E:y=E}return this._point.x=a+d*w,this._point.y=o+f*w,this._point.z=c+m*w,ai(s,this.#t(this._point.x,this._point.y,this._point.z,Math.max(0,n||0),this._sweepContact)),s.values[ho]=w,s}return this._point.x=l,this._point.y=h,this._point.z=u,ai(s,this.#t(this._point.x,this._point.y,this._point.z,Math.max(0,n||0),this._sweepContact)),s.values[ho]=1,s}queryBatch(t,e,n,i){if(!i||i.signedGaps.length<n)throw new RangeError("Preallocated batch contact output is too small");this.stats[cd]++;const s=this._capsuleContact;for(let a=0;a<n;a++){const o=a*3;this._point.x=t[o],this._point.y=t[o+1],this._point.z=t[o+2],this.#t(this._point.x,this._point.y,this._point.z,Math.max(0,e[a]||0),s),i.signedDistances[a]=s.values[md],i.signedGaps[a]=s.values[zr],i.penetrations[a]=s.values[gd],i.normals[o]=s.normal.values[0],i.normals[o+1]=s.normal.values[1],i.normals[o+2]=s.normal.values[2],i.targets[o]=s.target.values[0],i.targets[o+1]=s.target.values[1],i.targets[o+2]=s.target.values[2],i.branchIds[a]=s.values[xd],i.violations[a]=s.violation?1:0}return i.count=n,i}#a(t,e,n,i){const s=this.#f(t,e,n),a=s.values,o=this._distanceState,c=o.values;if(s.found&&a[Vr]>i+this.voxelSize*.25)return c[Pn]=a[Vr],c[ii]=a[rl],c[si]=a[al],c[ri]=a[ol],c[Yr]=a[sl],o.conservative=!0,o.source=jS,this.stats[ld]++,o;if(this.#l(t,e,n,o))return Math.sqrt(c[ii]*c[ii]+c[si]*c[si]+c[ri]*c[ri])<$n&&s.found&&(c[ii]=a[Hr],c[si]=a[Wr],c[ri]=a[Xr]),c[Yr]=a[Or],o.conservative=!1,o.source=$S,this.stats[hd]++,this._skipBvhValidation||this.#u(t,e,n,i,o),o;if(this.fallbackCollider?.pointContact){this._point.x=t,this._point.y=e,this._point.z=n;const l=this.fallbackCollider.pointContact(this._point,0,this._fallbackContact);if(Number.isFinite(l?.signedDistance)){const h=l.inward||l.normal,u=h?.x||0,d=h?.y||0,f=h?.z||0,m=Math.sqrt(u*u+d*d+f*f);if(c[Pn]=l.signedDistance,m>$n){const x=l.inward?1:-1;c[ii]=h.x/m*x,c[si]=h.y/m*x,c[ri]=h.z/m*x}else c[ii]=a[Hr],c[si]=a[Wr],c[ri]=a[Xr];return c[Yr]=a[Or],o.conservative=!1,o.source=Fh,this.stats[fd]++,o}}return c[Pn]=a[Gr],c[ii]=a[Hr],c[si]=a[Wr],c[ri]=a[Xr],c[Yr]=a[Or],o.conservative=!0,o.source=JS,this.stats[pd]++,o}#f(t,e,n){const i=this._centerlineQueryScratch;i[0]=t,i[1]=e,i[2]=n;const s=this._centerlineState,a=s.values;s.found=!1,a[Or]=-1,a[vd]=0,a[Gr]=-1/0,a[Vr]=-1/0,a[sl]=-1,a[rl]=1,a[al]=0,a[ol]=0,a[kr]=1/0,a[Hr]=1,a[Wr]=0,a[Xr]=0;const o=Math.floor((t-this.broadPhaseOrigin[0])/this.broadPhaseCellSize),c=Math.floor((e-this.broadPhaseOrigin[1])/this.broadPhaseCellSize),l=Math.floor((n-this.broadPhaseOrigin[2])/this.broadPhaseCellSize);if(o>=0&&c>=0&&l>=0&&o<this.broadPhaseDimensions[0]&&c<this.broadPhaseDimensions[1]&&l<this.broadPhaseDimensions[2]){const h=o+this.broadPhaseDimensions[0]*(c+this.broadPhaseDimensions[1]*l),u=this.broadPhaseOffsets[h],d=this.broadPhaseOffsets[h+1];for(let f=u;f<d;f++)this.#c(this.broadPhaseIds[f],s)}if(!s.found){const h=this.centerline.length/this.centerlineStride;for(let u=0;u<h;u++)this.#c(u,s)}return s}#c(t,e){const n=this._centerlineQueryScratch,i=n[0],s=n[1],a=n[2],o=e.values,c=t*this.centerlineStride,l=this.centerline[c],h=this.centerline[c+1],u=this.centerline[c+2],d=this.centerline[c+3]-l,f=this.centerline[c+4]-h,m=this.centerline[c+5]-u,x=d*d+f*f+m*m,g=nM(((i-l)*d+(s-h)*f+(a-u)*m)/Math.max($n,x),0,1),p=l+d*g,_=h+f*g,v=u+m*g,S=p-i,M=_-s,y=v-a,w=Math.sqrt(S*S+M*M+y*y),T=this.centerline[c+6]*(1-g)+this.centerline[c+7]*g,E=this.centerline[c+8],A=T-w,D=E-w;o[kr]=Math.min(o[kr],w);let b,F,L;if(w>$n)b=S/w,F=M/w,L=y/w;else{const N=Math.sqrt(x),U=N>$n?d/N:0,k=N>$n?f/N:1,O=N>$n?m/N:0,H=Math.abs(k)<.85?0:1,j=Math.abs(k)<.85?1:0,J=-O*j,nt=O*H,V=U*j-k*H,$=Math.sqrt(J*J+nt*nt+V*V)||1;b=J/$,F=nt/$,L=V/$}D>o[Vr]&&(o[Vr]=D,o[sl]=t,o[rl]=b,o[al]=F,o[ol]=L),!(e.found&&A<=o[Gr])&&(e.found=!0,o[Or]=t,o[vd]=g,o[Gr]=A,o[Hr]=b,o[Wr]=F,o[Xr]=L)}#l(t,e,n,i){const s=i.values,a=this._centerlineState.values,o=this._sdfCornerScratch,c=(t-this.sdfOrigin[0])/this.voxelSize,l=(e-this.sdfOrigin[1])/this.voxelSize,h=(n-this.sdfOrigin[2])/this.voxelSize,u=Math.floor(c),d=Math.floor(l),f=Math.floor(h),m=c-u,x=l-d,g=h-f,p=this.brickSize,_=Math.floor(u/p),v=Math.floor(d/p),S=Math.floor(f/p),M=u-_*p,y=d-v*p,w=f-S*p,T=M>=0&&y>=0&&w>=0&&M+1<p&&y+1<p&&w+1<p&&_>=0&&v>=0&&S>=0&&_<this.sdfDimensions[0]&&v<this.sdfDimensions[1]&&S<this.sdfDimensions[2];let E=-1;if(T){const at=_+this.sdfDimensions[0]*(v+this.sdfDimensions[1]*S),lt=this.sdfBrickLookup[at];if(lt!==Br){const _t=p,ht=p*p;E=lt*this.valuesPerBrick+M+_t*y+ht*w;const ut=this.sdfDistances,bt=this.sdfQuantization;o[0]=ut[E]*bt,o[1]=ut[E+1]*bt,o[2]=ut[E+_t]*bt,o[3]=ut[E+_t+1]*bt,o[4]=ut[E+ht]*bt,o[5]=ut[E+ht+1]*bt,o[6]=ut[E+ht+_t]*bt,o[7]=ut[E+ht+_t+1]*bt}}if(E<0&&(this.#s(o,0,u,d,f),this.#s(o,1,u+1,d,f),this.#s(o,2,u,d+1,f),this.#s(o,3,u+1,d+1,f),this.#s(o,4,u,d,f+1),this.#s(o,5,u+1,d,f+1),this.#s(o,6,u,d+1,f+1),this.#s(o,7,u+1,d+1,f+1)),!Number.isFinite(o[0])||!Number.isFinite(o[1])||!Number.isFinite(o[2])||!Number.isFinite(o[3])||!Number.isFinite(o[4])||!Number.isFinite(o[5])||!Number.isFinite(o[6])||!Number.isFinite(o[7]))return!1;const A=o[0]+(o[1]-o[0])*m,D=o[2]+(o[3]-o[2])*m,b=o[4]+(o[5]-o[4])*m,F=o[6]+(o[7]-o[6])*m,L=A+(D-A)*x,N=b+(F-b)*x,U=L+(N-L)*g;let k;if(this.sdfInsideBits){let at,lt,_t,ht,ut,bt,Y,Nt;if(E>=0){const W=p,Q=p*p,K=this.sdfInsideBits,tt=E+1,Mt=E+W,rt=Mt+1,gt=E+Q,Pt=gt+1,Wt=gt+W,et=Wt+1;at=(K[E>>3]&1<<(E&7))!==0?1:0,lt=(K[tt>>3]&1<<(tt&7))!==0?1:0,_t=(K[Mt>>3]&1<<(Mt&7))!==0?1:0,ht=(K[rt>>3]&1<<(rt&7))!==0?1:0,ut=(K[gt>>3]&1<<(gt&7))!==0?1:0,bt=(K[Pt>>3]&1<<(Pt&7))!==0?1:0,Y=(K[Wt>>3]&1<<(Wt&7))!==0?1:0,Nt=(K[et>>3]&1<<(et&7))!==0?1:0}else at=this.#n(u,d,f),lt=this.#n(u+1,d,f),_t=this.#n(u,d+1,f),ht=this.#n(u+1,d+1,f),ut=this.#n(u,d,f+1),bt=this.#n(u+1,d,f+1),Y=this.#n(u,d+1,f+1),Nt=this.#n(u+1,d+1,f+1);const wt=at+(lt-at)*m,Ct=_t+(ht-_t)*m,vt=ut+(bt-ut)*m,Jt=Y+(Nt-Y)*m,Lt=wt+(Ct-wt)*x,I=vt+(Jt-vt)*x,R=at+lt+_t+ht+ut+bt+Y+Nt;R>0&&R<8&&this.packedLumenField?(k=this.#p(t,e,n)?1:-1,this.stats[dd]++):k=Lt+(I-Lt)*g>=.5?1:-1,s[Pn]=U*k}else k=(this.packedLumenField?this.packedLumenField.queryCoordinates(t,e,n,this._lumenQuery).signedDistance:a[Gr])>=0?1:-1,s[Pn]=U*k;const O=(o[1]-o[0])*(1-x)+(o[3]-o[2])*x,H=(o[5]-o[4])*(1-x)+(o[7]-o[6])*x,j=(o[2]-o[0])*(1-m)+(o[3]-o[1])*m,J=(o[6]-o[4])*(1-m)+(o[7]-o[5])*m;let nt=(O*(1-g)+H*g)/this.voxelSize,V=(j*(1-g)+J*g)/this.voxelSize,$=(N-L)/this.voxelSize;const st=Math.sqrt(nt*nt+V*V+$*$);return st>$n&&(nt/=st,V/=st,$/=st),s[Pn]<0&&(a[kr]<=.001||a[kr]+.2<-s[Pn])&&(s[Pn]=-s[Pn],k=-k),s[ii]=nt*k,s[si]=V*k,s[ri]=$*k,!0}#p(t,e,n){const i=Math.round((t-this.sdfOrigin[0])*el),s=Math.round((e-this.sdfOrigin[1])*el),a=Math.round((n-this.sdfOrigin[2])*el);if(i<0||i>65535||s<0||s>131071||a<0||a>65535)return this.stats[il]++,this.packedLumenField.isInsideCoordinates(t,e,n);const o=i&65535|(s&65535)<<16,c=s>>>16|a<<1,l=(Math.imul(i,73856093)^Math.imul(s,19349663)^Math.imul(a,83492791))&QS,h=l<<1,u=h+1;if(this.signCacheValid[h]&&this.signCacheKeyLow[h]===o&&this.signCacheKeyHigh[h]===c)return this.signCacheVictim[l]=1,this.stats[nl]++,this.signCacheInside[h]!==0;if(this.signCacheValid[u]&&this.signCacheKeyLow[u]===o&&this.signCacheKeyHigh[u]===c)return this.signCacheVictim[l]=0,this.stats[nl]++,this.signCacheInside[u]!==0;const d=this.packedLumenField.isInsideCoordinates(t,e,n);let f;return this.signCacheValid[h]?this.signCacheValid[u]?f=h+this.signCacheVictim[l]:f=u:f=h,this.signCacheKeyLow[f]=o,this.signCacheKeyHigh[f]=c,this.signCacheInside[f]=d?1:0,this.signCacheValid[f]=1,this.signCacheVictim[l]=f===h?1:0,this.stats[il]++,d}#u(t,e,n,i,s){const a=this.fallbackGeometry?.boundsTree,o=s.values,c=o[Pn]-i,l=i>0?Math.min(this.bvhValidationDistance,.25):this.bvhValidationDistance;if(!a||Math.abs(c)>l&&(i<=0||c>=-.2))return!1;this._bvhPoint.set(t,e,n),this._bvhClosest.distance=1/0;const u=a.closestPointToPoint(this._bvhPoint,this._bvhClosest)?.distance??this._bvhPoint.distanceTo(this._bvhClosest.point);if(!Number.isFinite(u))return!1;const d=o[Pn]>=0?1:-1;return o[Pn]=u*d,u>$n&&(o[ii]=(t-this._bvhClosest.point.x)/u*d,o[si]=(e-this._bvhClosest.point.y)/u*d,o[ri]=(n-this._bvhClosest.point.z)/u*d),s.source=KS,this.stats[ud]++,!0}#s(t,e,n,i,s){if(n<0||i<0||s<0){t[e]=NaN;return}const a=Math.floor(n/this.brickSize),o=Math.floor(i/this.brickSize),c=Math.floor(s/this.brickSize);if(a>=this.sdfDimensions[0]||o>=this.sdfDimensions[1]||c>=this.sdfDimensions[2]){t[e]=NaN;return}const l=a+this.sdfDimensions[0]*(o+this.sdfDimensions[1]*c),h=this.sdfBrickLookup[l];if(h===Br){t[e]=NaN;return}const u=n-a*this.brickSize,d=i-o*this.brickSize,f=s-c*this.brickSize,m=u+this.brickSize*(d+this.brickSize*f),x=h*this.valuesPerBrick+m;t[e]=this.sdfDistances[x]*this.sdfQuantization}#n(t,e,n){if(t<0||e<0||n<0)return 0;const i=Math.floor(t/this.brickSize),s=Math.floor(e/this.brickSize),a=Math.floor(n/this.brickSize);if(i>=this.sdfDimensions[0]||s>=this.sdfDimensions[1]||a>=this.sdfDimensions[2])return 0;const o=i+this.sdfDimensions[0]*(s+this.sdfDimensions[1]*a),c=this.sdfBrickLookup[o];if(c===Br)return 0;const l=t-i*this.brickSize,h=e-s*this.brickSize,u=n-a*this.brickSize,d=l+this.brickSize*(h+this.brickSize*u),f=c*this.valuesPerBrick+d;return(this.sdfInsideBits[f>>3]&1<<(f&7))!==0?1:0}}const cM=25.4,ap=1/3,op=.035,cp=op*cM,gr=cp/2,Xl=gr*.5,lp=6,hp=lp*ap,lM=hp/2,up=1.8,dp=up/2,fp=5,pp=fp*ap,Zo=pp/2,mp=.97,gp=mp/2,cl=Zo*.78,Yt=1e-8,hM=1/120,Sd=1,uo=3,Md=4,yd=5,Ed=16,wd=.01;function le(r,t,e){return Math.max(t,Math.min(e,r))}function Se(r,t,e){return Math.sqrt(r*r+t*t+e*e)}function zn(){return globalThis.performance?.now?.()??Date.now()}function uM(r,t,e){if(!t)return 0;const n=Array.from(r.subarray(0,t));return n.sort((i,s)=>i-s),n[Math.min(n.length-1,Math.floor((n.length-1)*e))]}function jr(r=512){return{samples:new Float32Array(r),cursor:0,count:0,recordedCount:0,total:0,last:0}}function $r(r,t){r.last=t,r.total+=t,r.recordedCount++,r.samples[r.cursor]=t,r.cursor=(r.cursor+1)%r.samples.length,r.count=Math.min(r.samples.length,r.count+1)}function Kr(r){return{lastMs:r.last,averageMs:r.recordedCount?r.total/r.recordedCount:0,p95Ms:uM(r.samples,r.count,.95)}}const dc=Object.freeze({guidewire:Object.freeze({id:"guidewire",radius:gr,mass:1,stretchCompliance:2e-7,bendCompliance:2e-5,minBendComplianceScale:.001953125,maxBendAngle:135,foldLimitStrength:1,wallFriction:.006,linearDamping:.98,bendDamping:.06}),catheter:Object.freeze({id:"catheter",outerRadius:Zo,innerDiameter:mp,innerRadius:gp,radius:Zo,mass:1.4,stretchCompliance:1e-7,bendCompliance:8e-5,shapeCompliance:2e-4,maxBendAngle:45,foldLimitStrength:1,wallFriction:.12,lumenFriction:.04,linearDamping:.96,bendDamping:.32}),sheath:Object.freeze({id:"sheath",outerRadius:1,innerDiameter:up,innerRadius:dp})});class Ad{constructor(t,e,n,i={}){if(!Number.isInteger(e)||e<2)throw new RangeError("A rod requires at least two nodes");this.id=t,this.count=e,this.segmentCount=e-1,this.segmentLength=n,this.radius=i.radius??.5,this.innerRadius=i.innerRadius??0,this.mass=i.mass??1,this.stretchCompliance=i.stretchCompliance??2e-7,this.bendCompliance=i.bendCompliance??.001,this.minBendComplianceScale=i.minBendComplianceScale??.125,this.shapeCompliance=i.shapeCompliance??5e-5,this.maxBendAngle=i.maxBendAngle??135,this.foldLimitStrength=i.foldLimitStrength??.7,this.wallCompliance=i.wallCompliance??0,this.wallFriction=i.wallFriction??.08,this.lumenFriction=i.lumenFriction??.04,this.linearDamping=i.linearDamping??.98,this.bendDamping=le(i.bendDamping??0,0,1),this.sleepVelocity=i.sleepVelocity??.015,this.sleepFrames=i.sleepFrames??120,this.activeStart=0,this.activeEnd=e-1,this.collisionStartSegment=0,this.collisionEndSegment=e-2,this.sleepCounter=0,this.sleeping=!1,this.x=new Float32Array(e),this.y=new Float32Array(e),this.z=new Float32Array(e),this.previousX=new Float32Array(e),this.previousY=new Float32Array(e),this.previousZ=new Float32Array(e),this.velocityX=new Float32Array(e),this.velocityY=new Float32Array(e),this.velocityZ=new Float32Array(e),this.forceX=new Float32Array(e),this.forceY=new Float32Array(e),this.forceZ=new Float32Array(e),this.inverseMass=new Float32Array(e),this.nodeRadius=new Float32Array(e),this.pinned=new Uint8Array(e),this.controlEnabled=new Uint8Array(e),this.controlX=new Float32Array(e),this.controlY=new Float32Array(e),this.controlZ=new Float32Array(e),this.controlCompliance=new Float32Array(e),this.restShapeEnabled=new Uint8Array(e),this.restShapeX=new Float32Array(e),this.restShapeY=new Float32Array(e),this.restShapeZ=new Float32Array(e),this.restShapeCompliance=new Float32Array(e),this.restLength=new Float32Array(this.segmentCount),this.restBendChord=new Float32Array(e),this.lengthLambda=new Float32Array(this.segmentCount),this.lengthNormalX=new Float32Array(this.segmentCount),this.lengthNormalY=new Float32Array(this.segmentCount),this.lengthNormalZ=new Float32Array(this.segmentCount),this.lengthLower=new Float32Array(this.segmentCount),this.lengthUpper=new Float32Array(this.segmentCount),this.lengthRhs=new Float32Array(this.segmentCount),this.lengthSolution=new Float32Array(this.segmentCount),this.bendLambda=new Float32Array(e),this.bendComplianceByNode=new Float32Array(e),this.maxBendAngleByNode=new Float32Array(e),this.controlLambda=new Float32Array(e),this.shapeLambda=new Float32Array(e),this.wallLambda=new Float32Array(this.segmentCount),this.wallActive=new Uint8Array(this.segmentCount),this.wallT=new Float32Array(this.segmentCount),this.wallX=new Float32Array(this.segmentCount),this.wallY=new Float32Array(this.segmentCount),this.wallZ=new Float32Array(this.segmentCount),this.wallNormalX=new Float32Array(this.segmentCount),this.wallNormalY=new Float32Array(this.segmentCount),this.wallNormalZ=new Float32Array(this.segmentCount),this.wallBranchId=new Int32Array(this.segmentCount),this.wallGap=new Float32Array(this.segmentCount),this.wallQueryStartX=new Float32Array(this.segmentCount),this.wallQueryStartY=new Float32Array(this.segmentCount),this.wallQueryStartZ=new Float32Array(this.segmentCount),this.wallQueryEndX=new Float32Array(this.segmentCount),this.wallQueryEndY=new Float32Array(this.segmentCount),this.wallQueryEndZ=new Float32Array(this.segmentCount),this.wallCorrectionX=new Float32Array(e),this.wallCorrectionY=new Float32Array(e),this.wallCorrectionZ=new Float32Array(e),this.wallCorrectionWeight=new Float32Array(e),this.wallBranchId.fill(-1),this.wallGap.fill(1/0),this.nodeRadius.fill(this.radius),this.inverseMass.fill(1/Math.max(Yt,this.mass)),this.restLength.fill(n),this.bendComplianceByNode.fill(this.bendCompliance),this.maxBendAngleByNode.fill(this.maxBendAngle);for(let s=0;s<e;s++)this.x[s]=s*n;this.captureRestConfiguration(),this.copyCurrentToPrevious()}setNodePosition(t,e,n,i,s=!0){return this.x[t]=e,this.y[t]=n,this.z[t]=i,this.previousX[t]=e,this.previousY[t]=n,this.previousZ[t]=i,s&&(this.velocityX[t]=0,this.velocityY[t]=0,this.velocityZ[t]=0),this.wake(),this}setPinned(t,e=!0){return this.pinned[t]=e?1:0,this.inverseMass[t]=e?0:1/Math.max(Yt,this.mass),this.wake(),this}setActiveRange(t,e){const n=le(Math.floor(t),0,this.count-1),i=le(Math.ceil(e),n,this.count-1);if(n<this.activeStart)for(let s=n;s<this.activeStart;s++)this.previousX[s]=this.x[s],this.previousY[s]=this.y[s],this.previousZ[s]=this.z[s],this.velocityX[s]=0,this.velocityY[s]=0,this.velocityZ[s]=0;if(i>this.activeEnd)for(let s=this.activeEnd+1;s<=i;s++)this.previousX[s]=this.x[s],this.previousY[s]=this.y[s],this.previousZ[s]=this.z[s],this.velocityX[s]=0,this.velocityY[s]=0,this.velocityZ[s]=0;return(n!==this.activeStart||i!==this.activeEnd)&&this.wake(),this.activeStart=n,this.activeEnd=i,this}setCollisionRange(t,e){const n=Math.floor(t),i=Math.floor(e),s=le(n,0,this.segmentCount-1);let a;return i<n||n>=this.segmentCount||i<0?a=s-1:a=le(i,s,this.segmentCount-1),(s!==this.collisionStartSegment||a!==this.collisionEndSegment)&&this.wake(),this.collisionStartSegment=s,this.collisionEndSegment=a,this}setControlTarget(t,e,n,i,s=0){const a=Math.max(0,s),o=!this.controlEnabled[t]||Math.abs(this.controlX[t]-e)>1e-6||Math.abs(this.controlY[t]-n)>1e-6||Math.abs(this.controlZ[t]-i)>1e-6||Math.abs(this.controlCompliance[t]-a)>1e-10;return this.controlEnabled[t]=1,this.controlX[t]=e,this.controlY[t]=n,this.controlZ[t]=i,this.controlCompliance[t]=a,o&&(this.controlLambda[t]=0,this.wake()),this}clearControlTarget(t){return this.controlEnabled[t]&&this.wake(),this.controlEnabled[t]=0,this.controlLambda[t]=0,this}setRestShapeTarget(t,e,n,i,s=this.shapeCompliance){const a=Math.max(0,s),o=!this.restShapeEnabled[t]||Math.abs(this.restShapeX[t]-e)>1e-6||Math.abs(this.restShapeY[t]-n)>1e-6||Math.abs(this.restShapeZ[t]-i)>1e-6||Math.abs(this.restShapeCompliance[t]-a)>1e-10;return this.restShapeEnabled[t]=1,this.restShapeX[t]=e,this.restShapeY[t]=n,this.restShapeZ[t]=i,this.restShapeCompliance[t]=a,o&&(this.shapeLambda[t]=0,this.wake()),this}clearRestShapeTarget(t){return this.restShapeEnabled[t]&&this.wake(),this.restShapeEnabled[t]=0,this.shapeLambda[t]=0,this}captureRestConfiguration(){for(let t=0;t<this.segmentCount;t++)this.restLength[t]=Se(this.x[t+1]-this.x[t],this.y[t+1]-this.y[t],this.z[t+1]-this.z[t])||this.segmentLength;for(let t=1;t<this.count-1;t++)this.restBendChord[t]=Se(this.x[t+1]-this.x[t-1],this.y[t+1]-this.y[t-1],this.z[t+1]-this.z[t-1]);return this.lengthLambda.fill(0),this.bendLambda.fill(0),this}copyCurrentToPrevious(){this.previousX.set(this.x),this.previousY.set(this.y),this.previousZ.set(this.z)}wake(){this.sleeping=!1,this.sleepCounter=0}syncFromElasticRod(t,{resetVelocity:e=!1,preservePrevious:n=!1}={}){const i=t.nodeStorage,s=Math.min(this.count,t.nodes.length);let a=!1;for(let o=0;o<s;o++)a=a||Math.abs(this.x[o]-i.x[o])>1e-6||Math.abs(this.y[o]-i.y[o])>1e-6||Math.abs(this.z[o]-i.z[o])>1e-6||Math.abs(this.velocityX[o]-i.vx[o])>1e-5||Math.abs(this.velocityY[o]-i.vy[o])>1e-5||Math.abs(this.velocityZ[o]-i.vz[o])>1e-5,n&&(this.previousX[o]=this.x[o],this.previousY[o]=this.y[o],this.previousZ[o]=this.z[o]),this.x[o]=i.x[o],this.y[o]=i.y[o],this.z[o]=i.z[o],this.velocityX[o]=e?0:i.vx[o],this.velocityY[o]=e?0:i.vy[o],this.velocityZ[o]=e?0:i.vz[o],this.inverseMass[o]=i.pinned[o]?0:1/Math.max(Yt,i.mass[o]),this.pinned[o]=i.pinned[o],this.bendComplianceByNode[o]=le(this.bendCompliance*32/Math.max(.1,i.bendingStiffness[o]),this.bendCompliance*this.minBendComplianceScale,this.bendCompliance*8),this.maxBendAngleByNode[o]=le(i.bendAngleLimit?.[o]??this.maxBendAngle,1,179);return n||this.copyCurrentToPrevious(),a&&this.wake(),this}syncToElasticRod(t){const e=t.nodeStorage,n=Math.min(this.count,t.nodes.length);for(let i=0;i<n;i++)e.x[i]=this.x[i],e.y[i]=this.y[i],e.z[i]=this.z[i],e.vx[i]=this.velocityX[i],e.vy[i]=this.velocityY[i],e.vz[i]=this.velocityZ[i];return this}}class dM{constructor({contactField:t=null,fixedDt:e=hM,maxSubsteps:n=2,iterations:i=6,penetrationIterations:s=8,highPenetration:a=.15,contactActivation:o=.25}={}){this.contactField=t,this.fixedDt=e,this.maxSubsteps=n,this.iterations=i,this.penetrationIterations=s,this.highPenetration=a,this.contactActivation=o,this.accumulator=0,this.bodies=[],this.sheaths=[],this.containments=[],this.toolContacts=[],this.stepCount=0,this.contactCount=0,this.maxPenetration=0,this.settledMaxPenetration=0,this.settledContactBodyId=null,this.settledContactSegment=-1,this.settledContactT=0,this.settledContactX=0,this.settledContactY=0,this.settledContactZ=0,this.lastSubsteps=0,this.droppedTime=0,this._queryStart={x:0,y:0,z:0},this._queryEnd={x:0,y:0,z:0},this._segmentParameters={s:0,t:0},this._contact=Qn(),this._sweep=Qn(),this.timings={total:jr(),integrate:jr(),narrowPhase:jr(),constraints:jr(),velocity:jr()}}createRod(t,e,n,i={}){const s=new Ad(t,e,n,i);return this.bodies.push(s),s}addRod(t){if(!(t instanceof Ad))throw new TypeError("EndovascularRodBody is required");return this.bodies.includes(t)||this.bodies.push(t),t}addSheath({id:t="sheath",start:e,end:n,innerRadius:i=dc.sheath.innerRadius,bodies:s=null}={}){const a=n.x-e.x,o=n.y-e.y,c=n.z-e.z,l=Se(a,o,c);if(l<Yt)throw new RangeError("Sheath axis must have positive length");const h={id:t,startX:e.x,startY:e.y,startZ:e.z,axisX:a/l,axisY:o/l,axisZ:c/l,length:l,innerRadius:i,bodies:s,lambdas:new Map};return this.sheaths.push(h),h}addContainment(t,e,{innerRadius:n=e.innerRadius,compliance:i=0,friction:s=e.lumenFriction,enabled:a=!0,openProximal:o=!0,openDistal:c=!0,searchWindow:l=10,outerStartNode:h=e.activeStart,startNode:u=t.activeStart,endNode:d=t.activeEnd,innerResponse:f=1,outerResponse:m=1,finalProjection:x="inner",outerFollowsInnerCenterline:g=!1,innerArcOffset:p=0,containedLength:_=1/0}={}){const v={innerBody:t,outerBody:e,innerRadius:n,compliance:i,friction:s,enabled:a,openProximal:o,openDistal:c,searchWindow:l,outerStartNode:h,startNode:u,endNode:d,innerResponse:le(f,0,1),outerResponse:le(m,0,1),finalProjection:x,outerFollowsInnerCenterline:g,innerArcOffset:p,containedLength:_,lambdas:new Float32Array(t.count),closestSegment:new Int32Array(t.count),_lastEnabled:a,_lastOuterStartNode:h,_lastStartNode:u,_lastEndNode:d,_lastInnerActiveStart:t.activeStart,_lastInnerActiveEnd:t.activeEnd,_lastOuterActiveStart:e.activeStart,_lastOuterActiveEnd:e.activeEnd};return v.closestSegment.fill(-1),this.containments.push(v),v}addToolContact(t,e,{compliance:n=0,friction:i=.06,enabled:s=!0,openDistalB:a=!1,startSegmentA:o=0,endSegmentA:c=t.segmentCount-1,startSegmentB:l=0,endSegmentB:h=e.segmentCount-1}={}){const u=t.segmentCount*e.segmentCount,d={bodyA:t,bodyB:e,compliance:n,friction:i,enabled:s,openDistalB:a,startSegmentA:o,endSegmentA:c,startSegmentB:l,endSegmentB:h,lambdas:new Float32Array(u),_lastEnabled:s,_lastStartSegmentA:o,_lastEndSegmentA:c,_lastStartSegmentB:l,_lastEndSegmentB:h};return this.toolContacts.push(d),d}advance(t,e=null){const n=Math.max(0,Math.min(.25,t));this.accumulator+=n;let i=0;for(;this.accumulator+Yt>=this.fixedDt&&i<this.maxSubsteps;)e?.(this.fixedDt,i),this.stepFixed(),this.accumulator-=this.fixedDt,i++;return this.accumulator>=this.fixedDt&&(this.droppedTime+=this.accumulator-this.accumulator%this.fixedDt,this.accumulator%=this.fixedDt),this.lastSubsteps=i,i}stepFixed(){const t=zn();this.contactCount=0,this.maxPenetration=0;let e=zn();for(let o=0;o<this.bodies.length;o++){const c=this.bodies[o];c.lengthLambda.fill(0),c.bendLambda.fill(0),c.controlLambda.fill(0),c.shapeLambda.fill(0),this.#t(c)}$r(this.timings.integrate,zn()-e),e=zn();for(let o=0;o<this.bodies.length;o++)this.#i(this.bodies[o]);for(let o=0;o<this.bodies.length;o++)this.#a(this.bodies[o]);let n=zn()-e;e=zn();const i=this.maxPenetration>this.highPenetration?this.penetrationIterations:this.iterations;for(let o=0;o<i;o++){for(let c=0;c<this.sheaths.length;c++)this.#C(this.sheaths[c]);for(let c=0;c<this.bodies.length;c++)this.#l(this.bodies[c]);for(let c=0;c<this.bodies.length;c++)this.#p(this.bodies[c],(o&1)===1);for(let c=0;c<this.bodies.length;c++)this.#s(this.bodies[c]);for(let c=0;c<this.bodies.length;c++)this.#n(this.bodies[c]);for(let c=0;c<this.containments.length;c++)this.#m(this.containments[c]);for(let c=0;c<this.toolContacts.length;c++)this.#v(this.toolContacts[c]);for(let c=0;c<this.bodies.length;c++)this.#y(this.bodies[c]);for(let c=0;c<this.bodies.length;c++)this.#S(this.bodies[c])}for(let o=0;o<8;o++){for(let l=0;l<this.bodies.length;l++)this.#y(this.bodies[l]);for(let l=0;l<this.bodies.length;l++)this.#S(this.bodies[l]);for(let l=0;l<this.bodies.length;l++)this.#u(this.bodies[l]);let c=!0;for(let l=0;l<this.bodies.length;l++)c=c&&!this.#c(this.bodies[l],.002);if(c)break}for(let o=0;o<Ed;o++){let c=0;for(let l=0;l<this.bodies.length;l++)c=Math.max(c,this.#f(this.bodies[l]));if(c<=.02)break;for(let l=0;l<this.bodies.length;l++)this.#S(this.bodies[l]),this.#_(this.bodies[l]),o+1<Ed&&this.#u(this.bodies[l])}const s=this.containments.some(o=>o.enabled&&o.finalProjection!=="none"&&!o.outerFollowsInnerCenterline)?2:1;for(let o=0;o<s;o++)for(let c=0;c<this.containments.length;c++){const l=this.containments[c];!l.enabled||l.finalProjection==="none"||this.#m(l,{innerOnly:l.finalProjection!=="outer",outerOnly:l.finalProjection==="outer",applyFriction:!1})}$r(this.timings.constraints,zn()-e);const a=this.maxPenetration;e=zn(),this.contactCount=0,this.maxPenetration=0,this.settledContactBodyId=null,this.settledContactSegment=-1;for(let o=0;o<this.bodies.length;o++)this.#f(this.bodies[o]);this.settledMaxPenetration=this.maxPenetration,this.maxPenetration=Math.max(a,this.settledMaxPenetration),n+=zn()-e,$r(this.timings.narrowPhase,n),e=zn();for(let o=0;o<this.bodies.length;o++)this.#E(this.bodies[o]);for(let o=0;o<this.bodies.length;o++)this.#b(this.bodies[o]);for(let o=0;o<this.containments.length;o++)this.#w(this.containments[o]);for(let o=0;o<this.toolContacts.length;o++)this.#r(this.toolContacts[o]);$r(this.timings.velocity,zn()-e),this.stepCount++,$r(this.timings.total,zn()-t)}resetPerformanceStats(){this.contactCount=0,this.maxPenetration=0,this.settledMaxPenetration=0;for(const t of Object.values(this.timings))t.samples.fill(0),t.cursor=0,t.count=0,t.recordedCount=0,t.total=0,t.last=0}resetSimulationState(){this.accumulator=0,this.stepCount=0,this.lastSubsteps=0,this.droppedTime=0;for(const t of this.bodies)t.lengthLambda.fill(0),t.bendLambda.fill(0),t.controlLambda.fill(0),t.shapeLambda.fill(0),t.wallLambda.fill(0),t.wallActive.fill(0),t.wallBranchId.fill(-1),t.wallGap.fill(1/0),t.copyCurrentToPrevious(),t.wake();for(const t of this.sheaths)t.lambdas.clear();for(const t of this.containments)t.lambdas.fill(0),t.closestSegment.fill(-1),t._lastEnabled=t.enabled,t._lastOuterStartNode=t.outerStartNode,t._lastStartNode=t.startNode,t._lastEndNode=t.endNode,t._lastInnerActiveStart=t.innerBody.activeStart,t._lastInnerActiveEnd=t.innerBody.activeEnd,t._lastOuterActiveStart=t.outerBody.activeStart,t._lastOuterActiveEnd=t.outerBody.activeEnd;for(const t of this.toolContacts)t.lambdas.fill(0),t._lastEnabled=t.enabled,t._lastStartSegmentA=t.startSegmentA,t._lastEndSegmentA=t.endSegmentA,t._lastStartSegmentB=t.startSegmentB,t._lastEndSegmentB=t.endSegmentB;return this.resetPerformanceStats(),this}getStats(){const t=this.bodies.map(e=>this.#h(e));return{mode:"xpbd-contact-v1",fixedDt:this.fixedDt,steps:this.stepCount,lastSubsteps:this.lastSubsteps,droppedTime:this.droppedTime,contacts:this.contactCount,maxPenetration:this.maxPenetration,settledMaxPenetration:this.settledMaxPenetration,settledContact:{bodyId:this.settledContactBodyId,segment:this.settledContactSegment,t:this.settledContactT,x:this.settledContactX,y:this.settledContactY,z:this.settledContactZ},phases:{total:Kr(this.timings.total),integrate:Kr(this.timings.integrate),narrowPhase:Kr(this.timings.narrowPhase),constraints:Kr(this.timings.constraints),velocity:Kr(this.timings.velocity)},bodies:t}}#t(t){if(t.sleeping)return;const e=this.fixedDt,n=e*e,i=t.activeStart,s=t.activeEnd;for(let a=i;a<=s;a++)t.previousX[a]=t.x[a],t.previousY[a]=t.y[a],t.previousZ[a]=t.z[a],!(t.inverseMass[a]<=0)&&(t.velocityX[a]*=t.linearDamping,t.velocityY[a]*=t.linearDamping,t.velocityZ[a]*=t.linearDamping,t.x[a]+=t.velocityX[a]*e+t.forceX[a]*t.inverseMass[a]*n,t.y[a]+=t.velocityY[a]*e+t.forceY[a]*t.inverseMass[a]*n,t.z[a]+=t.velocityZ[a]*e+t.forceZ[a]*t.inverseMass[a]*n);t.forceX.fill(0),t.forceY.fill(0),t.forceZ.fill(0)}#i(t){if(!this.contactField||t.sleeping||t.collisionEndSegment<t.collisionStartSegment)return;const e=Math.max(t.activeStart,t.collisionStartSegment),n=Math.min(t.activeEnd,t.collisionEndSegment+1);for(let i=e;i<=n;i++){const s=t.x[i]-t.previousX[i],a=t.y[i]-t.previousY[i],o=t.z[i]-t.previousZ[i],c=t.nodeRadius[i];if(s*s+a*a+o*o<=c*c*.25)continue;this._queryStart.x=t.previousX[i],this._queryStart.y=t.previousY[i],this._queryStart.z=t.previousZ[i],this._queryEnd.x=t.x[i],this._queryEnd.y=t.y[i],this._queryEnd.z=t.z[i];const l=this.contactField.sweepSphere(this._queryStart,this._queryEnd,c,this._sweep);if(!l.violation||l.timeOfImpact>=1)continue;const h=Math.max(0,l.timeOfImpact-.001);t.x[i]=t.previousX[i]+s*h+l.inward.x*.001,t.y[i]=t.previousY[i]+a*h+l.inward.y*.001,t.z[i]=t.previousZ[i]+o*h+l.inward.z*.001}}#a(t){if(!this.contactField||t.sleeping||t.collisionEndSegment<t.collisionStartSegment)return;const e=Math.max(t.activeStart,t.collisionStartSegment,0),n=Math.min(t.activeEnd,t.collisionEndSegment+1,t.segmentCount);for(let i=e;i<n;i++){const s=t.wallActive[i]!==0;if(t.wallActive[i]=0,s){const m=t.wallT[i],x=t.x[i]+(t.x[i+1]-t.x[i])*m,g=t.y[i]+(t.y[i+1]-t.y[i])*m,p=t.z[i]+(t.z[i+1]-t.z[i])*m,_=Math.max(t.nodeRadius[i],t.nodeRadius[i+1]),v=(x-t.wallX[i])*t.wallNormalX[i]+(g-t.wallY[i])*t.wallNormalY[i]+(p-t.wallZ[i])*t.wallNormalZ[i]-_;if(v<=this.contactActivation+.1){t.wallActive[i]=1,v<0&&(this.contactCount++,this.maxPenetration=Math.max(this.maxPenetration,-v));continue}}const a=t.wallGap[i];if(!s&&Number.isFinite(a)){const m=t.x[i]-t.wallQueryStartX[i],x=t.y[i]-t.wallQueryStartY[i],g=t.z[i]-t.wallQueryStartZ[i],p=t.x[i+1]-t.wallQueryEndX[i],_=t.y[i+1]-t.wallQueryEndY[i],v=t.z[i+1]-t.wallQueryEndZ[i],S=Math.sqrt(m*m+x*x+g*g),M=Math.sqrt(p*p+_*_+v*v);if(a-Math.max(S,M)>this.contactActivation){t.wallLambda[i]*=.5;continue}}let o;if(this.contactField.queryCapsuleSoA)o=this.contactField.queryCapsuleSoA(t.x,t.y,t.z,t.nodeRadius,i,this._contact);else{const m=Math.max(t.nodeRadius[i],t.nodeRadius[i+1]);this._queryStart.x=t.x[i],this._queryStart.y=t.y[i],this._queryStart.z=t.z[i],this._queryEnd.x=t.x[i+1],this._queryEnd.y=t.y[i+1],this._queryEnd.z=t.z[i+1],o=this.contactField.queryCapsule(this._queryStart,this._queryEnd,m,this._contact)}const c=o.values,l=c[Sd],h=c[yd],u=c[Md],d=o.closestPoint.values,f=o.inward.values;if(t.wallGap[i]=l,t.wallQueryStartX[i]=t.x[i],t.wallQueryStartY[i]=t.y[i],t.wallQueryStartZ[i]=t.z[i],t.wallQueryEndX[i]=t.x[i+1],t.wallQueryEndY[i]=t.y[i+1],t.wallQueryEndZ[i]=t.z[i+1],l>this.contactActivation){t.wallLambda[i]*=.5;continue}t.wallBranchId[i]!==u&&(t.wallLambda[i]=0),t.wallActive[i]=1,t.wallT[i]=h,t.wallX[i]=d[0],t.wallY[i]=d[1],t.wallZ[i]=d[2],t.wallNormalX[i]=f[0],t.wallNormalY[i]=f[1],t.wallNormalZ[i]=f[2],t.wallBranchId[i]=u,o.violation&&(this.contactCount++,this.maxPenetration=Math.max(this.maxPenetration,c[uo]))}}#f(t){if(!this.contactField||t.sleeping||t.collisionEndSegment<t.collisionStartSegment)return 0;const e=Math.max(t.activeStart,t.collisionStartSegment,0),n=Math.min(t.activeEnd,t.collisionEndSegment+1,t.segmentCount);let i=0;for(let s=e;s<n;s++){if(!t.wallActive[s])continue;const a=t.wallGap[s];if(Number.isFinite(a)){const m=t.x[s]-t.wallQueryStartX[s],x=t.y[s]-t.wallQueryStartY[s],g=t.z[s]-t.wallQueryStartZ[s],p=t.x[s+1]-t.wallQueryEndX[s],_=t.y[s+1]-t.wallQueryEndY[s],v=t.z[s+1]-t.wallQueryEndZ[s],S=Math.sqrt(m*m+x*x+g*g),M=Math.sqrt(p*p+_*_+v*v),y=a-Math.max(S,M);if(y>.02){y>this.contactActivation&&(t.wallActive[s]=0,t.wallLambda[s]*=.5);continue}}let o;if(this.contactField.queryCapsuleSoA)o=this.contactField.queryCapsuleSoA(t.x,t.y,t.z,t.nodeRadius,s,this._contact);else{const m=Math.max(t.nodeRadius[s],t.nodeRadius[s+1]);this._queryStart.x=t.x[s],this._queryStart.y=t.y[s],this._queryStart.z=t.z[s],this._queryEnd.x=t.x[s+1],this._queryEnd.y=t.y[s+1],this._queryEnd.z=t.z[s+1],o=this.contactField.queryCapsule(this._queryStart,this._queryEnd,m,this._contact)}const c=o.values,l=c[Sd],h=c[yd],u=c[Md],d=o.closestPoint.values,f=o.inward.values;if(t.wallGap[s]=l,t.wallQueryStartX[s]=t.x[s],t.wallQueryStartY[s]=t.y[s],t.wallQueryStartZ[s]=t.z[s],t.wallQueryEndX[s]=t.x[s+1],t.wallQueryEndY[s]=t.y[s+1],t.wallQueryEndZ[s]=t.z[s+1],l>this.contactActivation){t.wallActive[s]=0,t.wallLambda[s]*=.5;continue}t.wallBranchId[s]!==u&&(t.wallLambda[s]=0),t.wallT[s]=h,t.wallX[s]=d[0],t.wallY[s]=d[1],t.wallZ[s]=d[2],t.wallNormalX[s]=f[0],t.wallNormalY[s]=f[1],t.wallNormalZ[s]=f[2],t.wallBranchId[s]=u,o.violation&&(this.contactCount++,c[uo]>this.maxPenetration&&(this.settledContactBodyId=t.id,this.settledContactSegment=s,this.settledContactT=h,this.settledContactX=t.x[s]+(t.x[s+1]-t.x[s])*h,this.settledContactY=t.y[s]+(t.y[s+1]-t.y[s])*h,this.settledContactZ=t.z[s]+(t.z[s+1]-t.z[s])*h),this.maxPenetration=Math.max(this.maxPenetration,c[uo]),i=Math.max(i,c[uo]))}return i}#c(t,e){const n=Math.max(0,t.activeStart),i=Math.min(t.segmentCount,t.activeEnd);for(let s=n;s<i;s++){const a=Se(t.x[s+1]-t.x[s],t.y[s+1]-t.y[s],t.z[s+1]-t.z[s]);if(Math.abs(a-t.restLength[s])>t.restLength[s]*e)return!0}return!1}#l(t){if(t.sleeping)return;const e=this.fixedDt*this.fixedDt;for(let n=t.activeStart;n<=t.activeEnd;n++){if(!t.controlEnabled[n]||t.inverseMass[n]<=0)continue;const i=t.x[n]-t.controlX[n],s=t.y[n]-t.controlY[n],a=t.z[n]-t.controlZ[n],o=Se(i,s,a);if(o<Yt)continue;const c=t.controlCompliance[n]/e,l=(-o-c*t.controlLambda[n])/(t.inverseMass[n]+c);t.controlLambda[n]+=l;const h=l/o*t.inverseMass[n];t.x[n]+=i*h,t.y[n]+=s*h,t.z[n]+=a*h}}#p(t,e=!1){if(t.sleeping)return;const n=t.stretchCompliance/(this.fixedDt*this.fixedDt),i=Math.max(0,t.activeStart),s=Math.min(t.segmentCount,t.activeEnd);for(let a=e?s-1:i;e?a>=i:a<s;a+=e?-1:1){const o=t.x[a+1]-t.x[a],c=t.y[a+1]-t.y[a],l=t.z[a+1]-t.z[a],h=Se(o,c,l);if(h<Yt)continue;const u=t.inverseMass[a],d=t.inverseMass[a+1],f=u+d+n;if(f<Yt)continue;const x=(-(h-t.restLength[a])-n*t.lengthLambda[a])/f;t.lengthLambda[a]+=x;const g=o/h,p=c/h,_=l/h;t.x[a]-=g*x*u,t.y[a]-=p*x*u,t.z[a]-=_*x*u,t.x[a+1]+=g*x*d,t.y[a+1]+=p*x*d,t.z[a+1]+=_*x*d}}#u(t){if(t.sleeping)return;const e=Math.max(0,t.activeStart),i=Math.min(t.segmentCount,t.activeEnd)-e;if(i<=0)return;for(let a=0;a<i;a++){const o=e+a,c=t.x[o+1]-t.x[o],l=t.y[o+1]-t.y[o],h=t.z[o+1]-t.z[o],u=Se(c,l,h);u<Yt?(t.lengthNormalX[a]=1,t.lengthNormalY[a]=0,t.lengthNormalZ[a]=0,t.lengthRhs[a]=0):(t.lengthNormalX[a]=c/u,t.lengthNormalY[a]=l/u,t.lengthNormalZ[a]=h/u,t.lengthRhs[a]=-(u-t.restLength[o]))}for(let a=0;a<i;a++){const o=e+a;let c=0,l=0;a>0&&(c=-t.inverseMass[o]*(t.lengthNormalX[a]*t.lengthNormalX[a-1]+t.lengthNormalY[a]*t.lengthNormalY[a-1]+t.lengthNormalZ[a]*t.lengthNormalZ[a-1])),a+1<i&&(l=-t.inverseMass[o+1]*(t.lengthNormalX[a]*t.lengthNormalX[a+1]+t.lengthNormalY[a]*t.lengthNormalY[a+1]+t.lengthNormalZ[a]*t.lengthNormalZ[a+1])),t.lengthLower[a]=c,t.lengthUpper[a]=l,t.lengthSolution[a]=t.inverseMass[o]+t.inverseMass[o+1]}let s=Math.max(Yt,t.lengthSolution[0]);t.lengthUpper[0]/=s,t.lengthRhs[0]/=s;for(let a=1;a<i;a++)s=Math.max(Yt,t.lengthSolution[a]-t.lengthLower[a]*t.lengthUpper[a-1]),t.lengthUpper[a]=a+1<i?t.lengthUpper[a]/s:0,t.lengthRhs[a]=(t.lengthRhs[a]-t.lengthLower[a]*t.lengthRhs[a-1])/s;t.lengthSolution[i-1]=t.lengthRhs[i-1];for(let a=i-2;a>=0;a--)t.lengthSolution[a]=t.lengthRhs[a]-t.lengthUpper[a]*t.lengthSolution[a+1];for(let a=0;a<i;a++){const o=e+a,c=t.lengthSolution[a],l=t.lengthNormalX[a],h=t.lengthNormalY[a],u=t.lengthNormalZ[a];t.x[o]-=l*c*t.inverseMass[o],t.y[o]-=h*c*t.inverseMass[o],t.z[o]-=u*c*t.inverseMass[o],t.x[o+1]+=l*c*t.inverseMass[o+1],t.y[o+1]+=h*c*t.inverseMass[o+1],t.z[o+1]+=u*c*t.inverseMass[o+1]}}#s(t){if(t.sleeping||t.count<3)return;const e=Math.max(1,t.activeStart+1),n=Math.min(t.count-1,t.activeEnd);for(let i=e;i<n;i++){const s=i-1,a=i+1,o=t.x[a]-t.x[s],c=t.y[a]-t.y[s],l=t.z[a]-t.z[s],h=Se(o,c,l);if(h<Yt)continue;const u=t.inverseMass[s],d=t.inverseMass[a],f=t.bendComplianceByNode[i]/(this.fixedDt*this.fixedDt),m=u+d+f;if(m<Yt)continue;const g=(-(h-t.restBendChord[i])-f*t.bendLambda[i])/m;t.bendLambda[i]+=g;const p=o/h,_=c/h,v=l/h;t.x[s]-=p*g*u,t.y[s]-=_*g*u,t.z[s]-=v*g*u,t.x[a]+=p*g*d,t.y[a]+=_*g*d,t.z[a]+=v*g*d}}#n(t){if(t.sleeping)return;const e=this.fixedDt*this.fixedDt;for(let n=t.activeStart;n<=t.activeEnd;n++){if(!t.restShapeEnabled[n]||t.inverseMass[n]<=0)continue;const i=t.x[n]-t.restShapeX[n],s=t.y[n]-t.restShapeY[n],a=t.z[n]-t.restShapeZ[n],o=Se(i,s,a);if(o<Yt)continue;const c=t.restShapeCompliance[n]/e,l=(-o-c*t.shapeLambda[n])/(t.inverseMass[n]+c);t.shapeLambda[n]+=l;const h=l/o*t.inverseMass[n];t.x[n]+=i*h,t.y[n]+=s*h,t.z[n]+=a*h}}#S(t){if(t.sleeping||t.count<3||t.foldLimitStrength<=0)return;const e=Math.max(1,t.activeStart+1),n=Math.min(t.count-1,t.activeEnd);for(let i=e;i<n;i++){const s=le(t.maxBendAngleByNode[i],1,179)*Math.PI/180,a=Math.cos(s),o=i-1,c=i+1,l=t.x[i]-t.x[o],h=t.y[i]-t.y[o],u=t.z[i]-t.z[o],d=t.x[c]-t.x[i],f=t.y[c]-t.y[i],m=t.z[c]-t.z[i],x=Se(l,h,u),g=Se(d,f,m);if(x<Yt||g<Yt)continue;const p=(l*d+h*f+u*m)/(x*g);if(p>=a)continue;let _=t.x[c]-t.x[o],v=t.y[c]-t.y[o],S=t.z[c]-t.z[o];const M=Se(_,v,S);M<Yt?(_=l/x,v=h/x,S=u/x):(_/=M,v/=M,S/=M);const w=Math.sqrt(Math.max(0,x*x+g*g+2*x*g*a))-M;if(w<=0)continue;const T=t.inverseMass[o],E=t.inverseMass[c],A=T+E;if(A<Yt)continue;const D=Math.min(w*t.foldLimitStrength,Math.min(x,g)*.35),b=D*T/A,F=D*E/A;if(t.x[o]-=_*b,t.y[o]-=v*b,t.z[o]-=S*b,t.x[c]+=_*F,t.y[c]+=v*F,t.z[c]+=S*F,t.inverseMass[i]>0){const L=t.foldLimitStrength*.45;if(t.x[i]+=((t.x[o]+t.x[c])*.5-t.x[i])*L,t.y[i]+=((t.y[o]+t.y[c])*.5-t.y[i])*L,t.z[i]+=((t.z[o]+t.z[c])*.5-t.z[i])*L,p<-.999&&M<Math.min(x,g)*.1){const N=l/x,U=h/x,k=u/x;let O,H,j;Math.abs(N)<.8?(O=0,H=k,j=-U):(O=-k,H=0,j=N);const J=Se(O,H,j)||1,nt=Math.min(x,g)*t.foldLimitStrength*.05;t.x[i]+=O/J*nt,t.y[i]+=H/J*nt,t.z[i]+=j/J*nt}}}}#m(t,{innerOnly:e=!1,outerOnly:n=!1,applyFriction:i=!0}={}){if((t.enabled!==t._lastEnabled||t.outerStartNode!==t._lastOuterStartNode||t.startNode!==t._lastStartNode||t.endNode!==t._lastEndNode||t.innerBody.activeStart!==t._lastInnerActiveStart||t.innerBody.activeEnd!==t._lastInnerActiveEnd||t.outerBody.activeStart!==t._lastOuterActiveStart||t.outerBody.activeEnd!==t._lastOuterActiveEnd)&&(t.lambdas.fill(0),t.closestSegment.fill(-1),t._lastEnabled=t.enabled,t._lastOuterStartNode=t.outerStartNode,t._lastStartNode=t.startNode,t._lastInnerActiveStart=t.innerBody.activeStart,t._lastInnerActiveEnd=t.innerBody.activeEnd,t._lastOuterActiveStart=t.outerBody.activeStart,t._lastOuterActiveEnd=t.outerBody.activeEnd),t._lastEndNode=t.endNode,!t.enabled)return;if(t.outerFollowsInnerCenterline){this.#g(t);return}const s=t.innerBody,a=t.outerBody,o=Math.max(0,t.innerRadius-s.radius),c=t.compliance/(this.fixedDt*this.fixedDt),l=le(t.outerStartNode,a.activeStart,a.activeEnd),h=Math.min(a.activeEnd,a.segmentCount);if(h<=l)return;const u=le(t.startNode,s.activeStart,s.activeEnd),d=le(t.endNode,u,s.activeEnd);let f=l,m=l,x=0,g=a.restLength[l];for(let p=u;p<=d;p++){for(p>u&&(x+=s.restLength[p-1]);f<h-1&&g<x;)f++,g+=a.restLength[f];let _=Math.max(l,m,f-t.searchWindow),v=Math.min(h-1,f+t.searchWindow),S=1/0,M=-1,y=0,w=0,T=0,E=0;for(let Nt=_;Nt<=v;Nt++){const wt=a.x[Nt],Ct=a.y[Nt],vt=a.z[Nt],Jt=a.x[Nt+1]-wt,Lt=a.y[Nt+1]-Ct,I=a.z[Nt+1]-vt,R=Jt*Jt+Lt*Lt+I*I,W=le(((s.x[p]-wt)*Jt+(s.y[p]-Ct)*Lt+(s.z[p]-vt)*I)/Math.max(Yt,R),0,1),Q=wt+Jt*W,K=Ct+Lt*W,tt=vt+I*W,Mt=s.x[p]-Q,rt=s.y[p]-K,gt=s.z[p]-tt,Pt=Mt*Mt+rt*rt+gt*gt;Pt<S&&(S=Pt,M=Nt,y=W,w=Q,T=K,E=tt)}if(M<0)continue;if(m=M,t.closestSegment[p]=M,t.openProximal&&M===l&&y<=1e-5){const Nt=a.x[l+1]-a.x[l],wt=a.y[l+1]-a.y[l],Ct=a.z[l+1]-a.z[l];if((s.x[p]-a.x[l])*Nt+(s.y[p]-a.y[l])*wt+(s.z[p]-a.z[l])*Ct<0)continue}if(t.openDistal&&M===h-1&&y>=1-1e-5){const Nt=a.x[h]-a.x[h-1],wt=a.y[h]-a.y[h-1],Ct=a.z[h]-a.z[h-1];if((s.x[p]-a.x[h])*Nt+(s.y[p]-a.y[h])*wt+(s.z[p]-a.z[h])*Ct>0)continue}const A=Math.sqrt(S);if(A<=o||A<Yt){t.lambdas[p]*=.8;continue}const D=(s.x[p]-w)/A,b=(s.y[p]-T)/A,F=(s.z[p]-E)/A,L=n?0:e?1:t.innerResponse,N=e?0:t.outerResponse,U=s.inverseMass[p]*L,k=1-y,O=y,H=a.inverseMass[M]*N*k*k,j=a.inverseMass[M+1]*N*O*O,J=U+H+j+c;if(J<Yt)continue;const V=(-(o-A)-c*t.lambdas[p])/J;t.lambdas[p]+=V,s.x[p]-=D*V*U,s.y[p]-=b*V*U,s.z[p]-=F*V*U,a.x[M]+=D*V*a.inverseMass[M]*N*k,a.y[M]+=b*V*a.inverseMass[M]*N*k,a.z[M]+=F*V*a.inverseMass[M]*N*k,a.x[M+1]+=D*V*a.inverseMass[M+1]*N*O,a.y[M+1]+=b*V*a.inverseMass[M+1]*N*O,a.z[M+1]+=F*V*a.inverseMass[M+1]*N*O;const $=s.x[p]-s.previousX[p]-(a.x[M]-a.previousX[M])*k-(a.x[M+1]-a.previousX[M+1])*O,st=s.y[p]-s.previousY[p]-(a.y[M]-a.previousY[M])*k-(a.y[M+1]-a.previousY[M+1])*O,at=s.z[p]-s.previousZ[p]-(a.z[M]-a.previousZ[M])*k-(a.z[M+1]-a.previousZ[M+1])*O,lt=$*D+st*b+at*F;let _t=$-D*lt,ht=st-b*lt,ut=at-F*lt;const bt=Se(_t,ht,ut),Y=U+H+j;if(bt>Yt&&Y>Yt&&i&&t.friction>0){_t/=bt,ht/=bt,ut/=bt;const Nt=-Math.min(bt/Y,t.friction*t.lambdas[p]);s.x[p]+=_t*Nt*U,s.y[p]+=ht*Nt*U,s.z[p]+=ut*Nt*U,a.x[M]-=_t*Nt*a.inverseMass[M]*N*k,a.y[M]-=ht*Nt*a.inverseMass[M]*N*k,a.z[M]-=ut*Nt*a.inverseMass[M]*N*k,a.x[M+1]-=_t*Nt*a.inverseMass[M+1]*N*O,a.y[M+1]-=ht*Nt*a.inverseMass[M+1]*N*O,a.z[M+1]-=ut*Nt*a.inverseMass[M+1]*N*O}}}#g(t){const e=t.innerBody,n=t.outerBody,i=le(t.startNode,e.activeStart,e.activeEnd),s=le(t.outerStartNode,n.activeStart,n.activeEnd);if(i>=e.activeEnd||s>n.activeEnd)return;let a=i,o=Math.max(0,t.innerArcOffset);i>e.activeStart&&(a=i-1,o-=e.restLength[a]);let c=0;const l=Math.max(0,t.containedLength);for(let h=s;h<=n.activeEnd&&!(c>l+1e-5);h++){for(;a<e.activeEnd-1&&o+e.restLength[a]<c;)o+=e.restLength[a],a++;const u=Math.max(Yt,e.restLength[a]),d=le((c-o)/u,0,1),f=e.x[a]+(e.x[a+1]-e.x[a])*d,m=e.y[a]+(e.y[a+1]-e.y[a])*d,x=e.z[a]+(e.z[a+1]-e.z[a])*d;n.x[h]=f,n.y[h]=m,n.z[h]=x,h<n.activeEnd&&(c+=n.restLength[h])}}#v(t){if((t.enabled!==t._lastEnabled||t.startSegmentA!==t._lastStartSegmentA||t.endSegmentA!==t._lastEndSegmentA||t.startSegmentB!==t._lastStartSegmentB||t.endSegmentB!==t._lastEndSegmentB)&&(t.lambdas.fill(0),t._lastEnabled=t.enabled,t._lastStartSegmentA=t.startSegmentA,t._lastEndSegmentA=t.endSegmentA,t._lastStartSegmentB=t.startSegmentB,t._lastEndSegmentB=t.endSegmentB),!t.enabled)return;const e=t.bodyA,n=t.bodyB,i=t.compliance/(this.fixedDt*this.fixedDt),s=le(t.startSegmentA,e.activeStart,e.segmentCount-1),a=le(t.endSegmentA,s,Math.min(e.activeEnd-1,e.segmentCount-1)),o=le(t.startSegmentB,n.activeStart,n.segmentCount-1),c=le(t.endSegmentB,o,Math.min(n.activeEnd-1,n.segmentCount-1));for(let l=s;l<=a;l++)for(let h=o;h<=c;h++){const u=this.#M(e,l,n,h,this._segmentParameters),d=e.x[l]+(e.x[l+1]-e.x[l])*u.s,f=e.y[l]+(e.y[l+1]-e.y[l])*u.s,m=e.z[l]+(e.z[l+1]-e.z[l])*u.s,x=n.x[h]+(n.x[h+1]-n.x[h])*u.t,g=n.y[h]+(n.y[h+1]-n.y[h])*u.t,p=n.z[h]+(n.z[h+1]-n.z[h])*u.t;if(t.openDistalB&&h===c&&u.t>=1-1e-5){const ht=n.x[c+1]-n.x[c],ut=n.y[c+1]-n.y[c],bt=n.z[c+1]-n.z[c];if((d-n.x[c+1])*ht+(f-n.y[c+1])*ut+(m-n.z[c+1])*bt>0)continue}let _=d-x,v=f-g,S=m-p;const M=Se(_,v,S),y=Math.max(e.nodeRadius[l],e.nodeRadius[l+1])+Math.max(n.nodeRadius[h],n.nodeRadius[h+1]);if(M>=y||M<Yt)continue;_/=M,v/=M,S/=M;const w=1-u.s,T=u.s,E=1-u.t,A=u.t,D=e.inverseMass[l]*w*w,b=e.inverseMass[l+1]*T*T,F=n.inverseMass[h]*E*E,L=n.inverseMass[h+1]*A*A,N=D+b+F+L+i;if(N<Yt)continue;const U=l*n.segmentCount+h;let O=(-(M-y)-i*t.lambdas[U])/N;const H=Math.max(0,t.lambdas[U]+O);O=H-t.lambdas[U],t.lambdas[U]=H,e.x[l]+=_*O*e.inverseMass[l]*w,e.y[l]+=v*O*e.inverseMass[l]*w,e.z[l]+=S*O*e.inverseMass[l]*w,e.x[l+1]+=_*O*e.inverseMass[l+1]*T,e.y[l+1]+=v*O*e.inverseMass[l+1]*T,e.z[l+1]+=S*O*e.inverseMass[l+1]*T,n.x[h]-=_*O*n.inverseMass[h]*E,n.y[h]-=v*O*n.inverseMass[h]*E,n.z[h]-=S*O*n.inverseMass[h]*E,n.x[h+1]-=_*O*n.inverseMass[h+1]*A,n.y[h+1]-=v*O*n.inverseMass[h+1]*A,n.z[h+1]-=S*O*n.inverseMass[h+1]*A;const j=(e.x[l]-e.previousX[l])*w+(e.x[l+1]-e.previousX[l+1])*T-(n.x[h]-n.previousX[h])*E-(n.x[h+1]-n.previousX[h+1])*A,J=(e.y[l]-e.previousY[l])*w+(e.y[l+1]-e.previousY[l+1])*T-(n.y[h]-n.previousY[h])*E-(n.y[h+1]-n.previousY[h+1])*A,nt=(e.z[l]-e.previousZ[l])*w+(e.z[l+1]-e.previousZ[l+1])*T-(n.z[h]-n.previousZ[h])*E-(n.z[h+1]-n.previousZ[h+1])*A,V=j*_+J*v+nt*S;let $=j-_*V,st=J-v*V,at=nt-S*V;const lt=Se($,st,at),_t=D+b+F+L;if(lt>Yt&&_t>Yt&&t.friction>0){$/=lt,st/=lt,at/=lt;const ht=-Math.min(lt/_t,t.friction*H);e.x[l]+=$*ht*e.inverseMass[l]*w,e.y[l]+=st*ht*e.inverseMass[l]*w,e.z[l]+=at*ht*e.inverseMass[l]*w,e.x[l+1]+=$*ht*e.inverseMass[l+1]*T,e.y[l+1]+=st*ht*e.inverseMass[l+1]*T,e.z[l+1]+=at*ht*e.inverseMass[l+1]*T,n.x[h]-=$*ht*n.inverseMass[h]*E,n.y[h]-=st*ht*n.inverseMass[h]*E,n.z[h]-=at*ht*n.inverseMass[h]*E,n.x[h+1]-=$*ht*n.inverseMass[h+1]*A,n.y[h+1]-=st*ht*n.inverseMass[h+1]*A,n.z[h+1]-=at*ht*n.inverseMass[h+1]*A}}}#M(t,e,n,i,s){const a=t.x[e+1]-t.x[e],o=t.y[e+1]-t.y[e],c=t.z[e+1]-t.z[e],l=n.x[i+1]-n.x[i],h=n.y[i+1]-n.y[i],u=n.z[i+1]-n.z[i],d=t.x[e]-n.x[i],f=t.y[e]-n.y[i],m=t.z[e]-n.z[i],x=a*a+o*o+c*c,g=a*l+o*h+c*u,p=l*l+h*h+u*u,_=a*d+o*f+c*m,v=l*d+h*f+u*m,S=x*p-g*g;let M=S>Yt?le((g*v-p*_)/S,0,1):0,y=p>Yt?le((g*M+v)/p,0,1):0;return x>Yt&&(M=le((g*y-_)/x,0,1)),s.s=M,s.t=y,s}#C(t){for(let e=0;e<this.bodies.length;e++){const n=this.bodies[e];if(t.bodies&&!t.bodies.includes(n))continue;let i=t.lambdas.get(n);i||(i=new Float32Array(n.count),t.lambdas.set(n,i));for(let s=n.activeStart;s<=n.activeEnd;s++){const a=n.x[s]-t.startX,o=n.y[s]-t.startY,c=n.z[s]-t.startZ,l=a*t.axisX+o*t.axisY+c*t.axisZ;if(l<=0||l>=t.length){i[s]*=.8;continue}const h=t.startX+t.axisX*l,u=t.startY+t.axisY*l,d=t.startZ+t.axisZ*l,f=n.x[s]-h,m=n.y[s]-u,x=n.z[s]-d,g=Se(f,m,x),p=Math.max(0,t.innerRadius-n.nodeRadius[s]);if(g<=p||g<Yt){i[s]*=.8;continue}const _=n.inverseMass[s];if(_<=0)continue;const S=-(p-g)/_;i[s]+=S,n.x[s]-=f/g*S*_,n.y[s]-=m/g*S*_,n.z[s]-=x/g*S*_}}}#y(t){if(t.sleeping)return;const e=t.wallCompliance/(this.fixedDt*this.fixedDt),n=Math.max(0,t.activeStart,t.collisionStartSegment),i=Math.min(t.activeEnd,t.collisionEndSegment+1,t.segmentCount);for(let s=n;s<i;s++){if(!t.wallActive[s])continue;const a=t.wallT[s],o=1-a,c=a,l=t.x[s]*o+t.x[s+1]*c,h=t.y[s]*o+t.y[s+1]*c,u=t.z[s]*o+t.z[s+1]*c,d=t.wallNormalX[s],f=t.wallNormalY[s],m=t.wallNormalZ[s],x=Math.max(t.nodeRadius[s],t.nodeRadius[s+1]),g=(l-t.wallX[s])*d+(h-t.wallY[s])*f+(u-t.wallZ[s])*m-x;if(g>=0){t.wallLambda[s]*=.85;continue}const p=t.inverseMass[s]*o*o,_=t.inverseMass[s+1]*c*c,v=p+_+e;if(v<Yt)continue;let S=(-g-e*t.wallLambda[s])/v;const M=Math.max(0,t.wallLambda[s]+S);S=M-t.wallLambda[s],t.wallLambda[s]=M,t.x[s]+=d*S*t.inverseMass[s]*o,t.y[s]+=f*S*t.inverseMass[s]*o,t.z[s]+=m*S*t.inverseMass[s]*o,t.x[s+1]+=d*S*t.inverseMass[s+1]*c,t.y[s+1]+=f*S*t.inverseMass[s+1]*c,t.z[s+1]+=m*S*t.inverseMass[s+1]*c}}#_(t){if(t.sleeping)return;const e=t.wallCorrectionX,n=t.wallCorrectionY,i=t.wallCorrectionZ,s=t.wallCorrectionWeight;e.fill(0),n.fill(0),i.fill(0),s.fill(0);const a=Math.max(0,t.activeStart,t.collisionStartSegment),o=Math.min(t.activeEnd,t.collisionEndSegment+1,t.segmentCount);for(let l=a;l<o;l++){if(!t.wallActive[l])continue;const h=t.wallT[l],u=t.x[l]+(t.x[l+1]-t.x[l])*h,d=t.y[l]+(t.y[l+1]-t.y[l])*h,f=t.z[l]+(t.z[l+1]-t.z[l])*h,m=t.wallNormalX[l],x=t.wallNormalY[l],g=t.wallNormalZ[l],p=Math.max(t.nodeRadius[l],t.nodeRadius[l+1]),_=Math.max(0,wd+p-((u-t.wallX[l])*m+(d-t.wallY[l])*x+(f-t.wallZ[l])*g));if(_<=.02)continue;const v=Math.max(.5,t.segmentLength),S=le(Math.ceil(_/(v*.02)),4,32),M=l+h,y=Math.max(t.activeStart,Math.floor(M-S)),w=Math.min(t.activeEnd,Math.ceil(M+S));for(let T=y;T<=w;T++){if(t.inverseMass[T]<=0)continue;const E=Math.max(0,1-Math.abs(T-M)/(S+.5));e[T]+=m*_*E,n[T]+=x*_*E,i[T]+=g*_*E,s[T]+=E}}for(let l=t.activeStart;l<=t.activeEnd;l++){const h=s[l];h>Yt&&(e[l]/=h,n[l]/=h,i[l]/=h)}for(let l=0;l<28;l++){let h=!1;const u=(l&1)===1;for(let d=u?t.activeEnd-1:t.activeStart;u?d>=t.activeStart:d<t.activeEnd;d+=u?-1:1){const f=d+1,m=e[f]-e[d],x=n[f]-n[d],g=i[f]-i[d],p=m*m+x*x+g*g,_=Math.max(1e-5,t.restLength[d]*.02);if(p<=_*_)continue;const v=Math.sqrt(p),S=t.inverseMass[d],M=t.inverseMass[f],y=S+M;if(y<=Yt)continue;const w=(v-_)/v,T=w*S/y,E=w*M/y;e[d]+=m*T,n[d]+=x*T,i[d]+=g*T,e[f]-=m*E,n[f]-=x*E,i[f]-=g*E,h=!0}if(!h)break}for(let l=t.activeStart;l<=t.activeEnd;l++)t.x[l]+=e[l],t.y[l]+=n[l],t.z[l]+=i[l];const c=Math.min(o-1,t.activeEnd-1);if(c>=a&&t.wallActive[c]&&t.wallT[c]>.75&&t.inverseMass[c+1]>0){const l=t.wallT[c],h=t.x[c]+(t.x[c+1]-t.x[c])*l,u=t.y[c]+(t.y[c+1]-t.y[c])*l,d=t.z[c]+(t.z[c+1]-t.z[c])*l,f=t.wallNormalX[c],m=t.wallNormalY[c],x=t.wallNormalZ[c],g=Math.max(t.nodeRadius[c],t.nodeRadius[c+1]),p=Math.max(0,wd+g-((h-t.wallX[c])*f+(u-t.wallY[c])*m+(d-t.wallZ[c])*x));if(p>0){const _=p/l;t.x[c+1]+=f*_,t.y[c+1]+=m*_,t.z[c+1]+=x*_}}}#E(t){if(t.sleeping)return;const e=1/this.fixedDt;let n=0;for(let i=t.activeStart;i<=t.activeEnd;i++){let s=t.x[i]-t.previousX[i],a=t.y[i]-t.previousY[i],o=t.z[i]-t.previousZ[i],c=0,l=0,h=0,u=0;i>0&&t.wallActive[i-1]&&(c+=t.wallLambda[i-1]*t.wallFriction,l+=t.wallNormalX[i-1],h+=t.wallNormalY[i-1],u+=t.wallNormalZ[i-1]),i<t.segmentCount&&t.wallActive[i]&&(c+=t.wallLambda[i]*t.wallFriction,l+=t.wallNormalX[i],h+=t.wallNormalY[i],u+=t.wallNormalZ[i]);const d=Se(l,h,u);if(d>Yt){l/=d,h/=d,u/=d;let f=s*l+a*h+o*u;f<0&&(s-=l*f,a-=h*f,o-=u*f,f=0);const m=s-l*f,x=a-h*f,g=o-u*f,p=Se(m,x,g);if(c>0&&p>Yt){const _=Math.min(p,c)/p;s-=m*_,a-=x*_,o-=g*_}}t.velocityX[i]=s*e,t.velocityY[i]=a*e,t.velocityZ[i]=o*e,n=Math.max(n,Se(t.velocityX[i],t.velocityY[i],t.velocityZ[i]))}n<t.sleepVelocity&&this.settledMaxPenetration<.01?t.sleepCounter++:t.sleepCounter=0,t.sleepCounter>=t.sleepFrames&&(t.sleeping=!0,t.velocityX.fill(0),t.velocityY.fill(0),t.velocityZ.fill(0))}#w(t){if(!t.enabled||t.outerFollowsInnerCenterline)return;const e=t.innerBody,n=t.outerBody,i=Math.max(0,t.innerRadius-e.radius),s=le(t.startNode,e.activeStart,e.activeEnd),a=le(t.endNode,s,e.activeEnd),o=le(t.outerStartNode,n.activeStart,n.activeEnd),c=Math.min(n.activeEnd,n.segmentCount);for(let l=s;l<=a;l++){const h=t.closestSegment[l];if(h<o||h>=c)continue;const u=n.x[h],d=n.y[h],f=n.z[h],m=n.x[h+1]-u,x=n.y[h+1]-d,g=n.z[h+1]-f,p=m*m+x*x+g*g,_=le(((e.x[l]-u)*m+(e.y[l]-d)*x+(e.z[l]-f)*g)/Math.max(Yt,p),0,1),v=e.x[l]-(u+m*_),S=e.y[l]-(d+x*_),M=e.z[l]-(f+g*_),y=Se(v,S,M);if(y<Yt||y<i-.01&&t.lambdas[l]<=Yt)continue;const w=v/y,T=S/y,E=M/y,A=1-_,D=_,b=e.velocityX[l]-n.velocityX[h]*A-n.velocityX[h+1]*D,F=e.velocityY[l]-n.velocityY[h]*A-n.velocityY[h+1]*D,L=e.velocityZ[l]-n.velocityZ[h]*A-n.velocityZ[h+1]*D,N=b*w+F*T+L*E;if(N<=0)continue;const U=e.inverseMass[l]*t.innerResponse,k=n.inverseMass[h]*t.outerResponse*A*A,O=n.inverseMass[h+1]*t.outerResponse*D*D,H=U+k+O;if(H<Yt)continue;const j=N/H;e.velocityX[l]-=w*j*U,e.velocityY[l]-=T*j*U,e.velocityZ[l]-=E*j*U,n.velocityX[h]+=w*j*n.inverseMass[h]*t.outerResponse*A,n.velocityY[h]+=T*j*n.inverseMass[h]*t.outerResponse*A,n.velocityZ[h]+=E*j*n.inverseMass[h]*t.outerResponse*A,n.velocityX[h+1]+=w*j*n.inverseMass[h+1]*t.outerResponse*D,n.velocityY[h+1]+=T*j*n.inverseMass[h+1]*t.outerResponse*D,n.velocityZ[h+1]+=E*j*n.inverseMass[h+1]*t.outerResponse*D}}#b(t){if(t.sleeping||t.bendDamping<=0||t.count<3)return;const e=Math.max(1,t.activeStart+1),n=Math.min(t.count-1,t.activeEnd);for(let i=e;i<n;i++){if(t.inverseMass[i]<=0)continue;const s=t.x[i+1]-t.x[i-1],a=t.y[i+1]-t.y[i-1],o=t.z[i+1]-t.z[i-1],c=Se(s,a,o);if(c<Yt)continue;const l=s/c,h=a/c,u=o/c,d=(t.velocityX[i-1]+t.velocityX[i+1])*.5,f=(t.velocityY[i-1]+t.velocityY[i+1])*.5,m=(t.velocityZ[i-1]+t.velocityZ[i+1])*.5,x=t.velocityX[i]-d,g=t.velocityY[i]-f,p=t.velocityZ[i]-m,_=x*l+g*h+p*u;t.velocityX[i]-=(x-l*_)*t.bendDamping,t.velocityY[i]-=(g-h*_)*t.bendDamping,t.velocityZ[i]-=(p-u*_)*t.bendDamping}}#r(t){if(!t.enabled)return;const e=t.bodyA,n=t.bodyB,i=le(t.startSegmentA,e.activeStart,e.segmentCount-1),s=le(t.endSegmentA,i,Math.min(e.activeEnd-1,e.segmentCount-1)),a=le(t.startSegmentB,n.activeStart,n.segmentCount-1),o=le(t.endSegmentB,a,Math.min(n.activeEnd-1,n.segmentCount-1));for(let c=i;c<=s;c++)for(let l=a;l<=o;l++){const h=c*n.segmentCount+l;if(t.lambdas[h]<=Yt)continue;this.#M(e,c,n,l,this._segmentParameters);const u=this._segmentParameters.s,d=this._segmentParameters.t,f=1-u,m=u,x=1-d,g=d,p=e.x[c]*f+e.x[c+1]*m,_=e.y[c]*f+e.y[c+1]*m,v=e.z[c]*f+e.z[c+1]*m,S=n.x[l]*x+n.x[l+1]*g,M=n.y[l]*x+n.y[l+1]*g,y=n.z[l]*x+n.z[l+1]*g,w=p-S,T=_-M,E=v-y,A=Se(w,T,E);if(A<Yt)continue;const D=w/A,b=T/A,F=E/A,L=e.velocityX[c]*f+e.velocityX[c+1]*m-n.velocityX[l]*x-n.velocityX[l+1]*g,N=e.velocityY[c]*f+e.velocityY[c+1]*m-n.velocityY[l]*x-n.velocityY[l+1]*g,U=e.velocityZ[c]*f+e.velocityZ[c+1]*m-n.velocityZ[l]*x-n.velocityZ[l+1]*g,k=L*D+N*b+U*F;if(k>=0)continue;const O=e.inverseMass[c]*f*f,H=e.inverseMass[c+1]*m*m,j=n.inverseMass[l]*x*x,J=n.inverseMass[l+1]*g*g,nt=O+H+j+J;if(nt<Yt)continue;const V=-k/nt;e.velocityX[c]+=D*V*e.inverseMass[c]*f,e.velocityY[c]+=b*V*e.inverseMass[c]*f,e.velocityZ[c]+=F*V*e.inverseMass[c]*f,e.velocityX[c+1]+=D*V*e.inverseMass[c+1]*m,e.velocityY[c+1]+=b*V*e.inverseMass[c+1]*m,e.velocityZ[c+1]+=F*V*e.inverseMass[c+1]*m,n.velocityX[l]-=D*V*n.inverseMass[l]*x,n.velocityY[l]-=b*V*n.inverseMass[l]*x,n.velocityZ[l]-=F*V*n.inverseMass[l]*x,n.velocityX[l+1]-=D*V*n.inverseMass[l+1]*g,n.velocityY[l+1]-=b*V*n.inverseMass[l+1]*g,n.velocityZ[l+1]-=F*V*n.inverseMass[l+1]*g}}#h(t){let e=0,n=0,i=!0;for(let s=t.activeStart;s<Math.min(t.activeEnd,t.segmentCount);s++){const a=t.x[s+1]-t.x[s],o=t.y[s+1]-t.y[s],c=t.z[s+1]-t.z[s],l=Se(a,o,c);if(e=Math.max(e,Math.abs(l-t.restLength[s])/Math.max(Yt,t.restLength[s])),i=i&&Number.isFinite(l),s<=t.activeStart||s>=t.activeEnd-1)continue;const h=t.x[s]-t.x[s-1],u=t.y[s]-t.y[s-1],d=t.z[s]-t.z[s-1],f=t.x[s+1]-t.x[s],m=t.y[s+1]-t.y[s],x=t.z[s+1]-t.z[s],g=Se(h,u,d)*Se(f,m,x);g>Yt&&(n=Math.max(n,Math.acos(le((h*f+u*m+d*x)/g,-1,1))))}return{id:t.id,sleeping:t.sleeping,finite:i,maxLengthError:e,maxBendAngleDegrees:n*180/Math.PI}}}function fM(r=140,t=0,e=null,n=lM){const o={radius:20,branchRadius:10,branchPoint:{x:0,y:-300,z:0},segments:[]},c={x:0,y:0,z:0},l={x:0,y:-300,z:0};o.main={start:c,end:l},o.segments.push({start:c,end:l,radius:20});function h(b){const F=Math.PI/6*b+t*b,L={x:o.branchPoint.x+Math.sin(F)*r,y:o.branchPoint.y-r,z:0};return{angle:F,end:L,length:r}}o.right=h(1),o.left=h(-1),o.segments.push({start:o.branchPoint,end:o.right.end,radius:10}),o.segments.push({start:o.branchPoint,end:o.left.end,radius:10});const u={x:-73,y:-383,z:14},d=new C(.24,.96,-.21).normalize(),f=o.left.length*.5,m=e??f,x={x:u.x-d.x*m,y:u.y-d.y*m,z:u.z-d.z*m},g={...u};o.sheath={start:x,end:g,radius:n,length:m,isSheath:!0},o.segments.push(o.sheath);for(const b of o.segments){const F=b.end.x-b.start.x,L=b.end.y-b.start.y,N=b.end.z-b.start.z,U=Math.sqrt(F*F+L*L+N*N)||1;b.length=U,b.volume=Math.PI*b.radius*b.radius*U}const p=new Map,_=[];function v(b){const F=`${b.x.toFixed(5)},${b.y.toFixed(5)},${b.z.toFixed(5)}`;if(p.has(F))return p.get(F);const L=_.length;return p.set(F,L),_.push({position:b,segments:[]}),L}o.segments.forEach((b,F)=>{b.startNode=v(b.start),b.endNode=v(b.end),_[b.startNode].segments.push(F),_[b.endNode].segments.push(F)}),o.nodes=_;const S=o.segments.map(()=>[]),M=o.segments.map(()=>null),y=1e-6,w=(b,F)=>Math.abs(b.x-F.x)<y&&Math.abs(b.y-F.y)<y&&Math.abs(b.z-F.z)<y;for(let b=0;b<o.segments.length;b++)for(let F=0;F<o.segments.length;F++)b!==F&&w(o.segments[b].end,o.segments[F].start)&&(S[b].push(F),M[F]=b);o.segmentGraph=S;for(let b=0;b<o.segments.length;b++)o.segments[b].parent=M[b];const T=85,E={},A=b=>{const F=b.end.x-b.start.x,L=b.end.y-b.start.y,N=b.end.z-b.start.z,U=Math.sqrt(F*F+L*L+N*N)||1;return{x:F/U,y:L/U,z:N/U}};function D(b,F){const L=o.segments[b],N=A(L);L.flowDir=N,L.flowSpeed=F;const U=S[b];if(E[b]={dir:N,speed:F,children:U},U.length){let k=0;for(const O of U)k+=o.segments[O].radius;for(const O of U){const H=F*(o.segments[O].radius/k);D(O,H)}}}for(let b=0;b<o.segments.length;b++)M[b]===null&&D(b,T);return o.flow=E,{vessel:o}}const pM=2,ll=1/15,mM=.58,gM=.29,Hi=(r,t,e)=>Math.min(e,Math.max(t,r)),hl=(r,t,e)=>r+(t-r)*e;class xM{constructor(t,e,n,i,s={}){this.ecgCanvas=t,this.bpCanvas=e,this.hrElem=n,this.bpElem=i,this.spo2Elem=s.spo2Elem||null,this.mapElem=s.mapElem||null,this.rrElem=s.rrElem||null,this.rhythmElem=s.rhythmElem||null,this.clockElem=s.clockElem||null,this.ecgCtx=t.getContext("2d"),this.bpCtx=e.getContext("2d"),this.ecgCanvasState=this.#v("#020303","#000000"),this.bpCanvasState=this.#v("#030202","#000000"),this.baselineDash=[6,8],this.ecgSampleRate=250,this.bpSampleRate=50,this.ecgBufferLength=this.ecgSampleRate*10,this.bpBufferLength=this.bpSampleRate*10,this.ecgData=new Float32Array(this.ecgBufferLength),this.bpData=new Float32Array(this.bpBufferLength),this.bpData.fill(100),this.ecgCursor=0,this.bpCursor=0,this.drawAccumulator=ll,this.lastReadouts=Object.create(null),this.lastClockSecond=-1,this.clockLabel="00:00",this.time=0,this.cycleTime=0,this.variabilitySeed=Math.random()*Math.PI*2,this.baseHeartRate=75,this.heartRate=this.baseHeartRate,this.beatInterval=60/this.heartRate,this.ecgAccumulator=0,this.bpAccumulator=0,this.currentHR=this.heartRate,this.baselineSystolic=120,this.baselineDiastolic=80,this.waveSystolic=this.baselineSystolic,this.waveDiastolic=this.baselineDiastolic,this.systolic=120,this.diastolic=80,this.meanPressure=93,this.spo2=98,this.spo2Target=this.spo2,this.respiratoryRate=14,this.respiratoryRateTarget=this.respiratoryRate,this.bpMax=0,this.bpMin=1/0,this.ecgTemplate=this.#n(),this.ecgTemplateIndex=0,this.ecgSamplesSinceBeat=0,this.ecgSamplesToNextBeat=this.#c(),this.bpTemplate=this.#S()}setHeartRate(t){this.baseHeartRate=t,this.heartRate=t,this.beatInterval=60/this.heartRate,this.currentHR=t}update(t){this.ecgAccumulator+=t,this.bpAccumulator+=t,this.time+=t,this.cycleTime+=t;const e=1/this.ecgSampleRate;for(;this.ecgAccumulator>=e;){this.ecgAccumulator-=e;const i=this.#f();this.ecgData[this.ecgCursor]=i,this.ecgCursor=(this.ecgCursor+1)%this.ecgBufferLength}const n=1/this.bpSampleRate;for(;this.bpAccumulator>=n;){this.bpAccumulator-=n;const i=this.cycleTime/this.beatInterval%1,s=Math.floor(i*this.bpTemplate.length),a=this.#s(this.bpTemplate[s]);this.bpData[this.bpCursor]=a,this.bpCursor=(this.bpCursor+1)%this.bpBufferLength,a>this.bpMax&&(this.bpMax=a),a<this.bpMin&&(this.bpMin=a)}this.cycleTime>=this.beatInterval&&(this.currentHR=60/this.beatInterval,this.systolic=this.bpMax,this.diastolic=this.bpMin,this.meanPressure=this.diastolic+(this.systolic-this.diastolic)/3,this.cycleTime-=this.beatInterval,this.bpMax=0,this.bpMin=1/0,this.#a()),this.#l(t),this.drawAccumulator+=t,!(this.drawAccumulator<ll)&&(this.drawAccumulator%=ll,this.#t("hr",this.hrElem,Math.round(this.currentHR)),this.#i(),this.#t("spo2",this.spo2Elem,Math.round(this.spo2)),this.#t("map",this.mapElem,Math.round(this.meanPressure)),this.#t("rr",this.rrElem,Math.round(this.respiratoryRate)),this.#t("rhythm",this.rhythmElem,this.#b()),this.#t("clock",this.clockElem,this.#r()),this.#m(),this.#g())}#t(t,e,n){!e||this.lastReadouts[t]===n||(e.textContent=n,this.lastReadouts[t]=n)}#i(){if(!this.bpElem)return;const t=Math.round(this.systolic),e=Math.round(this.diastolic),n=t*256+e;this.lastReadouts.bp!==n&&(this.bpElem.textContent=`${t}/${e}`,this.lastReadouts.bp=n)}#a(){const t=Math.sin(this.time*.34+this.variabilitySeed),e=Math.sin(this.time*.11+this.variabilitySeed*.7),n=(Math.random()-.5)*1.8,i=(Math.random()-.5)*2.2;this.heartRate=Hi(this.baseHeartRate+t*2.2+e*1.4+n,58,96),this.beatInterval=60/this.heartRate,this.ecgSamplesToNextBeat=this.#c(),this.currentHR=hl(this.currentHR,this.heartRate,.75),this.waveSystolic=Hi(this.baselineSystolic+t*3.2+e*2+i,106,134),this.waveDiastolic=Hi(this.baselineDiastolic+t*1.6+e*1.2+i*.45,68,88)}#f(){const t=this.ecgTemplateIndex<this.ecgTemplate.length?this.ecgTemplate[this.ecgTemplateIndex]:0;return this.ecgTemplateIndex+=1,this.ecgSamplesSinceBeat+=1,this.ecgSamplesSinceBeat>=this.ecgSamplesToNextBeat&&(this.ecgTemplateIndex=0,this.ecgSamplesSinceBeat=0,this.ecgSamplesToNextBeat=this.#c()),t}#c(){return Math.max(this.ecgTemplate?.length||1,Math.round(this.beatInterval*this.ecgSampleRate))}#l(t){const e=Math.sin(this.time*.31+this.variabilitySeed),n=Math.sin(this.time*.07+this.variabilitySeed*1.9),i=98+e*.9+n*.65,s=14+e*.9+n*.5;this.spo2Target=Hi(i,96,100),this.respiratoryRateTarget=Hi(s,11,18),this.spo2=hl(this.spo2,this.spo2Target,Hi(t*1.4,0,1)),this.respiratoryRate=hl(this.respiratoryRate,this.respiratoryRateTarget,Hi(t*.8,0,1))}#p(t){const e=(n,i,s)=>{const a=(t-n)/i;return s*Math.exp(-.5*a**2)};return e(.095,.022,.08)+e(.178,.009,-.12)+e(.198,.007,.82)+e(.222,.012,-.18)+e(.42,.062,.17)}#u(t){const i=1/(1+Math.exp(-(t-.11)/.018)),s=Math.exp(-Math.max(t-.16,0)/.36),a=-5.5*Math.exp(-.5*((t-.33)/.018)**2),o=3.2*Math.exp(-.5*((t-.37)/.026)**2);return 80+40*i*s+a+o}#s(t){const e=Hi((t-80)/40,0,1.25);return this.waveDiastolic+e*(this.waveSystolic-this.waveDiastolic)}#n(){const e=Math.round(.62*this.ecgSampleRate),n=[];for(let i=0;i<e;i++)n.push(this.#p(i/this.ecgSampleRate));return n}#S(){const t=[];for(let e=0;e<this.bpSampleRate;e++){const n=e/this.bpSampleRate;t.push(this.#u(n))}return t}#m(){const t=this.ecgCtx,e=this.#M(this.ecgCanvas,t,this.ecgCanvasState),n=e.w,i=e.h,s=this.ecgData.length;this.#C(t,n,i,e.backgroundGradient);const a=i*mM,o=i*gM,c=Math.max(2,Math.min(s,Math.ceil(n)));this.#_(t,n,i,a,"rgba(82, 118, 102, 0.32)"),t.beginPath();for(let l=0;l<c;l++){const h=Math.floor(l*s/c),u=Math.max(h+1,Math.floor((l+1)*s/c));let d=0;for(let x=h;x<u;x++){let g=this.ecgCursor+x;g>=s&&(g-=s);const p=this.ecgData[g];Math.abs(p)>Math.abs(d)&&(d=p)}const f=l/(c-1)*n,m=a-d*o;l===0?t.moveTo(f,m):t.lineTo(f,m)}this.#E(t,"#39e75f",1.35),this.#w(t,n,i,e.markerGradient,"#39e75f")}#g(){const t=this.bpCtx,e=this.#M(this.bpCanvas,t,this.bpCanvasState),n=e.w,i=e.h,s=this.bpData.length,a=Math.max(2,Math.min(s,Math.ceil(n)));this.#C(t,n,i,e.backgroundGradient),this.#_(t,n,i,i-45/85*i,"rgba(120, 88, 88, 0.3)"),t.beginPath();for(let o=0;o<a;o++){const c=Math.floor(o*s/a),l=Math.max(c+1,Math.floor((o+1)*s/a));let h=0;for(let m=c;m<l;m++){let x=this.bpCursor+m;x>=s&&(x-=s),h+=this.bpData[x]}const u=h/(l-c),d=o/(a-1)*n,f=i-(u-55)/85*i;o===0?t.moveTo(d,f):t.lineTo(d,f)}this.#E(t,"#f04d4d",1.35),this.#w(t,n,i,e.markerGradient,"#f04d4d")}#v(t,e){return{w:0,h:0,dpr:0,topColor:t,bottomColor:e,backgroundGradient:null,markerGradient:null}}#M(t,e,n){const i=Math.max(1,t.clientWidth||t.width),s=Math.max(1,t.clientHeight||t.height),a=Math.min(window.devicePixelRatio||1,pM),o=Math.round(i*a),c=Math.round(s*a);if((t.width!==o||t.height!==c)&&(t.width=o,t.height=c),e.setTransform(a,0,0,a,0,0),n.w!==i||n.h!==s||n.dpr!==a||!n.backgroundGradient){const l=e.createLinearGradient(0,0,0,s);l.addColorStop(0,n.topColor),l.addColorStop(1,n.bottomColor);const h=i-10.5,u=e.createLinearGradient(h-20,0,h+4,0);u.addColorStop(0,"rgba(255,255,255,0)"),u.addColorStop(1,"rgba(210,220,218,0.12)"),n.w=i,n.h=s,n.dpr=a,n.backgroundGradient=l,n.markerGradient=u}return n}#C(t,e,n,i){t.clearRect(0,0,e,n),t.fillStyle=i,t.fillRect(0,0,e,n),this.#y(t,e,n)}#y(t,e,n){t.save(),t.strokeStyle="rgba(88, 112, 106, 0.16)",t.lineWidth=1;for(let i=.5;i<e;i+=32)t.beginPath(),t.moveTo(i,0),t.lineTo(i,n),t.stroke();for(let i=.5;i<n;i+=24)t.beginPath(),t.moveTo(0,i),t.lineTo(e,i),t.stroke();t.strokeStyle="rgba(88, 112, 106, 0.26)";for(let i=.5;i<e;i+=160)t.beginPath(),t.moveTo(i,0),t.lineTo(i,n),t.stroke();t.restore()}#_(t,e,n,i,s){t.save(),t.strokeStyle=s,t.setLineDash(this.baselineDash),t.lineWidth=1,t.beginPath(),t.moveTo(0,i),t.lineTo(e,i),t.stroke(),t.restore()}#E(t,e,n){t.save(),t.lineJoin="round",t.lineCap="round",t.strokeStyle=e,t.lineWidth=n,t.stroke(),t.restore()}#w(t,e,n,i,s){const a=e-10.5;t.save(),t.fillStyle=i,t.fillRect(Math.max(0,a-20),0,24,n),t.strokeStyle=s,t.globalAlpha=.85,t.lineWidth=1,t.beginPath(),t.moveTo(a,8),t.lineTo(a,n-8),t.stroke(),t.restore()}#b(){return this.currentHR>=105?"TACHY":this.currentHR<=50?"BRADY":this.meanPressure<65?"LOW MAP":"SINUS"}#r(){const t=Math.floor(this.time);if(t===this.lastClockSecond)return this.clockLabel;const e=Math.floor(t/60).toString().padStart(2,"0"),n=(t%60).toString().padStart(2,"0");return this.lastClockSecond=t,this.clockLabel=`${e}:${n}`,this.clockLabel}}function We(r,t,e,n,i,s=new cn){const a=new $t(new Ln(r,t,e),n);return a.position.copy(i),a.rotation.copy(s),a}function oi(r,t,e,n,i,s=new cn,a=40){const o=new $t(new is(r,t,e,a),n);return o.position.copy(i),o.rotation.copy(s),o}function Td(r,t,e,n,i=new cn){const s=new $t(new ur(r,t,10,24),e);return s.position.copy(n),s.rotation.copy(i),s}function ul(r,t,e,n,i,s,a=0){const o=[];for(let l=0;l<=96;l++){const h=ue.degToRad(t+(e-t)*l/96);o.push(new C(n,r*Math.sin(h),a+r*Math.cos(h)))}const c=new Wf(o);return new $t(new Ih(c,128,i,18,!1),s)}function _M(){const r=new Ne,t=new Ne,e=new Ne,n=new Ne,i=new C(10,22,0),s=new Mn({color:14542314,roughness:.42,metalness:.08}),a=new Mn({color:15922678,roughness:.36,metalness:.04}),o=new Mn({color:10332852,roughness:.48,metalness:.28}),c=new Mn({color:4739933,roughness:.62,metalness:.25}),l=new Mn({color:1383200,roughness:.75}),h=new Mn({color:15331571,roughness:.34}),u=new Ue({color:12773623}),d=new Mn({color:15002092,roughness:.4,metalness:.06}),f=new Ue({color:9559551,transparent:!0,opacity:.16,depthWrite:!1}),m=new Ue({color:4380671});r.add(We(118,12,58,l,new C(10,-105,-82))),r.add(We(78,52,62,s,new C(10,-72,-82))),r.add(We(84,18,62,c,new C(10,-103,-82))),r.add(oi(12,12,10,l,new C(-34,-108,-108),new cn(Math.PI/2,0,0),32)),r.add(oi(12,12,10,l,new C(54,-108,-108),new cn(Math.PI/2,0,0),32)),r.add(oi(10,10,8,l,new C(-34,-108,-56),new cn(Math.PI/2,0,0),32)),r.add(oi(10,10,8,l,new C(54,-108,-56),new cn(Math.PI/2,0,0),32)),r.add(oi(16,18,70,c,new C(10,-37,-82))),t.add(oi(12,14,96,o,new C(10,-14,-82))),t.add(We(62,24,52,s,new C(10,38,-82))),t.add(We(28,10,44,c,new C(10,24,-82))),t.add(Td(12,34,s,new C(10,37,-82),new cn(0,0,Math.PI/2))),t.add(We(48,26,40,s,new C(10,37,-82))),t.add(We(54,18,28,s,new C(10,29,-86))),t.add(We(34,22,34,s,new C(10,20,-86))),t.add(oi(21,21,18,c,new C(10,26,-82),new cn(Math.PI/2,0,0),48)),t.add(oi(25,25,18,c,new C(10,22,-86),new cn(Math.PI/2,0,0),48)),t.add(We(46,34,24,c,new C(10,22,-86))),t.add(oi(5,6,28,o,new C(28,79,-82))),t.add(We(46,24,6,a,new C(28,96,-82),new cn(ue.degToRad(-8),0,0))),t.add(We(31,16,2,new Ue({color:1450543}),new C(28,96,-78))),t.add(Td(3.2,30,o,new C(38,19,-52),new cn(Math.PI/2,0,0))),e.position.copy(i),t.add(e),r.add(t);const x=86,g=0,p=0,_=58,v=ul(x,_,360-_,g,6.2,a,p),S=ul(x+8,_+2,360-_-2,g-4.5,1.8,o,p),M=ul(x-8,_+2,360-_-2,g+4.5,1.8,o,p);e.add(v,S,M);const y=ue.degToRad(_),w=x*Math.sin(y),T=-w,E=0,A=p+x*Math.cos(y),D=(A+E)*.5,b=Math.abs(E-A)+12;e.add(We(48,13,b,a,new C(g,w,D))),e.add(We(48,13,b,a,new C(g,T,D))),e.add(We(42,14,16,a,new C(g,w,A))),e.add(We(42,14,16,a,new C(g,T,A)));const F=We(50,16,42,h,new C(g,w,E));n.add(F);const L=We(40,2,34,u,new C(g,w-9,E));n.add(L);const N=We(58,22,44,d,new C(g,T,E));n.add(N);const U=We(36,9,26,c,new C(g,T+17,E));n.add(U);const k=oi(15,22,w-T-20,f,new C(g,0,E));n.add(k),e.add(n);const O=new $t(new Lh(7.5,9,48),m);return O.position.set(g,0,E),O.rotation.x=Math.PI/2,e.add(O),{group:r,gantryGroup:e,liftGroup:t,detectorAssembly:n}}function vM(){const r=new Ne,t=new Ne;r.userData.slideGroup=t;const e=new Mn({color:10134701,roughness:.45}),n=new Mn({color:14081507,roughness:.3,metalness:.15}),i=new Mn({color:5857899,roughness:.6,metalness:.2}),s=new Mn({color:2503490,roughness:.55}),a=new Mn({color:2064266,roughness:.7}),o=new Mn({color:14202011,roughness:.65}),c=new $t(new Ln(86,10,58),i);c.position.set(0,-88,0),r.add(c);const l=new $t(new is(8,10,76,32),i);l.position.set(0,-45,0),r.add(l);const h=new $t(new Ln(66,10,44),i);h.position.set(0,-8,0),r.add(h);const u=new $t(new Ln(230,8,58),e);u.position.set(0,0,0),t.add(u);const d=new $t(new Ln(218,5,48),s);d.position.set(0,6.5,0),t.add(d);const f=new $t(new Ln(224,3,3),n);f.position.set(0,7,-32),t.add(f);const m=f.clone();m.position.z=32,t.add(m);const x=new $t(new ur(17,64,10,22),a);x.rotation.z=Math.PI/2,x.position.set(16,23,0),t.add(x);const g=new $t(new Ln(62,8,42),a);g.position.set(14,20,0),t.add(g);const p=new $t(new ti(13,28,18),o);p.scale.set(1.05,.82,.9),p.position.set(-50,21,0),t.add(p);const _=new $t(new Ln(32,5,32),new Mn({color:15265522,roughness:.75}));_.position.set(-50,13,0),t.add(_);const v=new $t(new ur(7,62,8,16),a);v.rotation.z=Math.PI/2,v.position.set(70,18,-10),t.add(v);const S=v.clone();S.position.z=10,t.add(S);const M=new $t(new ur(4.5,58,8,14),o);M.rotation.z=Math.PI/2,M.position.set(4,17,-31),t.add(M);const y=M.clone();return y.position.z=31,t.add(y),r.add(t),r}let Jn,or,cr,ha,Yl,Cd,zo,ql;const SM=new C(0,24,-30);function MM(){const r=document.getElementById("carm-preview");if(!r)return null;r.replaceChildren(),Jn=new Er,Jn.background=new Kt(131843);const t=new zv(14542820,.72);Jn.add(t);const e=new ju(16777215,.85);e.position.set(120,180,160),Jn.add(e);const n=new ju(10140083,.24);n.position.set(-160,40,-130),Jn.add(n);const i=r.clientWidth,s=r.clientHeight;or=new Dn(39,i/s,.1,1e3),or.position.set(268,146,289),or.lookAt(SM),Jn.add(or),cr=new Ch({antialias:!0,alpha:!0}),cr.setSize(i,s),cr.setPixelRatio(Math.min(window.devicePixelRatio||1,2)),r.appendChild(cr.domElement);const a=new Gv(300,12,4741719,1910052);a.position.y=-94,Jn.add(a),zo=vM(),Jn.add(zo),ha=new Ne;const{group:o,gantryGroup:c,liftGroup:l,detectorAssembly:h}=_M();return Yl=c,Cd=l,ql=h,ha.add(o),Jn.add(ha),xp(),{group:ha,gantry:Yl,detectorAssembly:ql,lift:Cd,table:zo}}function xp(){!cr||!Jn||!or||cr.render(Jn,or)}function yM(r,t,e,n,i,s,a,o,c=()=>{}){const l=document.getElementById("carmX"),h=document.getElementById("carmY"),u=document.getElementById("carmZ"),d=document.getElementById("carmDetDist"),f=document.getElementById("carmZUp"),m=document.getElementById("carmZDown"),x=document.getElementById("carmRollLeft"),g=document.getElementById("carmRollRight"),p=document.getElementById("carmAngleReset"),_=document.getElementById("carmLao30"),v=document.getElementById("carmRao30"),S=document.getElementById("carmYawReadout"),M=document.getElementById("carmPitchReadout"),y=document.getElementById("carmRollReadout");[l,h,u,d].filter(Boolean).forEach(it=>it.addEventListener("change",()=>it.blur()));let T=0,E=0,A=0,D=parseFloat(l.value),b=parseFloat(h.value),F=parseFloat(u.value),L=parseFloat(d.value),N=!1,U=0;const k=D,O=b,H=F,j=L,J=10,nt=new C(1,0,0),V=new C(0,0,1),$=new C(0,1,0),st=new Fn,at=new Fn,lt=new Fn;function _t(){const it=H-(F-H);return new C(t.branchPoint.x+D,t.branchPoint.y+b,t.branchPoint.z+it)}function ht(it){return Math.round(ue.radToDeg(it))}function ut(it){return it===0?"AP 0°":`${it>0?"LAO":"RAO"} ${Math.abs(it)}°`}function bt(it){return it===0?"CRA 0°":`${it>0?"CRA":"CAU"} ${Math.abs(it)}°`}function Y(){S&&(S.textContent=ut(ht(T))),M&&(M.textContent=bt(ht(E))),y&&(y.textContent=`Roll ${ht(A)}°`)}function Nt(){const it=_t(),Ot=new C().setFromSpherical(new Ov(1,Math.PI/2-E,T)).normalize(),P=it.clone().addScaledVector(Ot,e),G=it.clone().addScaledVector(Ot,-L);r.position.copy(P),r.up.set(0,1,0),r.lookAt(G),r.rotateZ(A);const q=D-k,Z=b-O,X=F-H;n&&n.position.set(J,0,0),a&&(a.position.y=X*.12),o&&(o.userData.slideGroup||o).position.set(Z*.08,0,q*.08),i&&(st.setFromAxisAngle(nt,-T),at.setFromAxisAngle(V,E),i.quaternion.copy(at).multiply(st)),s&&(lt.setFromAxisAngle($,A),s.quaternion.copy(lt)),(n||i||s||a||o)&&c(),Y(),U++}Nt(),l.addEventListener("input",it=>{N||(D=parseFloat(it.target.value),Nt())}),h.addEventListener("input",it=>{N||(b=parseFloat(it.target.value),Nt())}),u.addEventListener("input",it=>{N||(F=parseFloat(it.target.value),Nt())}),d.addEventListener("input",it=>{N||(L=parseFloat(it.target.value),Nt())});const wt=document.getElementById("positionJoystick"),Ct=document.getElementById("positionJoystickHandle"),vt=document.getElementById("angleJoystick"),Jt=document.getElementById("angleJoystickHandle");let Lt=0,I=0,R=0,W=!1,Q=null,K=null;const tt=ue.degToRad(90),Mt=ue.degToRad(45),rt=ue.degToRad(90),gt=ue.degToRad(22),Pt=ue.degToRad(18),Wt=ue.degToRad(18),et=ue.degToRad(24),oe=ue.degToRad(24),jt=.22;function kt(it,Ot){const P=Math.abs(it);return P<Ot?0:Math.sign(it)*((P-Ot)/(1-Ot))}function At(it,Ot,P,G,{resetOnRelease:q=!0}={}){if(!it||!Ot)return;const Z=Ot.offsetWidth/2,X=it.offsetWidth/2-Z;let ct=!1;const Tt="transform 0.2s ease-out";function Dt(St,Ht){if(N)return;const Bt=it.getBoundingClientRect();let Vt=St-Bt.left-Bt.width/2,pe=Ht-Bt.top-Bt.height/2;const Ze=Math.hypot(Vt,pe);if(Ze>X){const be=X/Ze;Vt*=be,pe*=be}Ot.style.transform=`translate(-50%, -50%) translate(${Vt}px, ${pe}px)`,P(Vt/X,pe/X)}it.addEventListener("mousedown",St=>{N||(ct=!0,Ot.style.transition="none",Dt(St.clientX,St.clientY))}),window.addEventListener("mousemove",St=>{ct&&Dt(St.clientX,St.clientY)}),window.addEventListener("mouseup",()=>{ct&&(ct=!1,Ot.style.transition=Tt,q&&(Ot.style.transform="translate(-50%, -50%)"),G())}),it.addEventListener("pointerdown",St=>{N||(ct=!0,it.setPointerCapture?.(St.pointerId),Ot.style.transition="none",Dt(St.clientX,St.clientY))}),it.addEventListener("pointermove",St=>{ct&&Dt(St.clientX,St.clientY)}),it.addEventListener("pointerup",St=>{ct&&(ct=!1,it.releasePointerCapture?.(St.pointerId),Ot.style.transition=Tt,q&&(Ot.style.transform="translate(-50%, -50%)"),G())}),it.addEventListener("pointercancel",St=>{ct&&(ct=!1,it.releasePointerCapture?.(St.pointerId),Ot.style.transition=Tt,q&&(Ot.style.transform="translate(-50%, -50%)"),G())}),it.addEventListener("touchstart",St=>{if(N)return;St.preventDefault(),ct=!0,Ot.style.transition="none";const Ht=St.touches[0];Dt(Ht.clientX,Ht.clientY)}),window.addEventListener("touchmove",St=>{if(!ct)return;const Ht=St.touches[0];Dt(Ht.clientX,Ht.clientY)},{passive:!1}),window.addEventListener("touchend",()=>{ct&&(ct=!1,Ot.style.transition=Tt,q&&(Ot.style.transform="translate(-50%, -50%)"),G())})}let xt=0,Ft=0,Qt=0;const fe=parseFloat(l.min),qt=parseFloat(l.max),ot=parseFloat(h.min),B=parseFloat(h.max),dt=parseFloat(u.min),ft=parseFloat(u.max),Gt=(qt-fe)*.18,Ut=(B-ot)*.18,de=(ft-dt)*.18;let ce=performance.now();function ve(it,Ot){return Math.abs(it)<=Ot?0:it-Math.sign(it)*Ot}function Ce(it){const Ot=(it-ce)/1e3;if(ce=it,N){requestAnimationFrame(Ce);return}let P=!1;if((xt!==0||Ft!==0)&&(D=Math.min(Math.max(D+xt*Gt*Ot,fe),qt),b=Math.min(Math.max(b+Ft*Ut*Ot,ot),B),l.value=Math.round(D),h.value=Math.round(b),P=!0),Qt!==0){const G=ue.clamp(F+Qt*de*Ot,dt,ft);P=P||G!==F,F=G,u.value=Math.round(F)}if((I!==0||R!==0)&&(T=Math.min(Math.max(T+I*gt*Ot,-tt),tt),E=Math.min(Math.max(E+R*Pt*Ot,-Mt),Mt),P=!0),Lt!==0&&(A=Math.min(Math.max(A+Lt*Wt*Ot,-rt),rt),P=!0),W){I=0,R=0,Q=null,K?.classList.remove("active"),K=null,Lt=0;const G=et*Ot,q=ve(T,G),Z=ve(E,G),X=ve(A,G);P=P||q!==T||Z!==E||X!==A,T=q,E=Z,A=X}if(Q!==null){I=0;const G=oe*Ot,q=Q-T,Z=Math.abs(q)<=G?Q:T+Math.sign(q)*G;P=P||Z!==T,T=ue.clamp(Z,-tt,tt)}P&&Nt(),requestAnimationFrame(Ce)}requestAnimationFrame(Ce);function ae(it){N||(Qt=it)}function Le(){Qt=0}f&&m&&(f.addEventListener("mousedown",()=>ae(1)),m.addEventListener("mousedown",()=>ae(-1)),window.addEventListener("mouseup",Le),f.addEventListener("touchstart",it=>{it.preventDefault(),ae(1)}),m.addEventListener("touchstart",it=>{it.preventDefault(),ae(-1)}),window.addEventListener("touchend",Le),window.addEventListener("touchcancel",Le));function qe(it){N||(Lt=it)}function gi(){Lt=0}x&&g&&(x.addEventListener("mousedown",()=>qe(-1)),g.addEventListener("mousedown",()=>qe(1)),window.addEventListener("mouseup",gi),x.addEventListener("touchstart",it=>{it.preventDefault(),qe(-1)}),g.addEventListener("touchstart",it=>{it.preventDefault(),qe(1)}),window.addEventListener("touchend",gi),window.addEventListener("touchcancel",gi));function As(it){it?.preventDefault?.(),!N&&(W=!0,I=0,R=0,Lt=0,Jt&&(Jt.style.transition="transform 0.2s ease-out",Jt.style.transform="translate(-50%, -50%)"),p?.classList.add("active"))}function Tn(){W=!1,p?.classList.remove("active")}p&&(p.addEventListener("pointerdown",it=>{p.setPointerCapture?.(it.pointerId),As(it)}),p.addEventListener("pointerup",it=>{p.releasePointerCapture?.(it.pointerId),Tn()}),p.addEventListener("pointercancel",Tn),p.addEventListener("pointerleave",Tn),p.addEventListener("click",it=>it.preventDefault()),window.addEventListener("blur",Tn));function Bi(it,Ot,P){P?.preventDefault?.(),!N&&(W=!1,p?.classList.remove("active"),I=0,R=0,Q=ue.clamp(it,-tt,tt),K&&K!==Ot&&K.classList.remove("active"),K=Ot,K?.classList.add("active"),Jt&&(Jt.style.transition="transform 0.2s ease-out",Jt.style.transform="translate(-50%, -50%)"))}function Cn(){Q=null,K?.classList.remove("active"),K=null}function Ts(it,Ot){it&&(it.addEventListener("pointerdown",P=>{it.setPointerCapture?.(P.pointerId),Bi(Ot,it,P)}),it.addEventListener("pointerup",P=>{it.releasePointerCapture?.(P.pointerId),Cn()}),it.addEventListener("pointercancel",Cn),it.addEventListener("pointerleave",Cn),it.addEventListener("click",P=>P.preventDefault()))}Ts(_,ue.degToRad(30)),Ts(v,ue.degToRad(-30)),window.addEventListener("blur",Cn),At(wt,Ct,(it,Ot)=>{xt=-Ot,Ft=-it},()=>{xt=0,Ft=0}),At(vt,Jt,(it,Ot)=>{Cn(),I=kt(-Ot,jt),R=kt(-it,jt)},()=>{I=0,R=0});function en(){xt=0,Ft=0,Qt=0,Lt=0,I=0,R=0,W=!1,Q=null,K?.classList.remove("active"),K=null,p?.classList.remove("active"),Ct&&(Ct.style.transform="translate(-50%, -50%)"),Jt&&(Jt.style.transform="translate(-50%, -50%)")}function xi(){en(),T=0,E=0,A=0,D=k,b=O,F=H,L=j,l.value=String(Math.round(D)),h.value=String(Math.round(b)),u.value=String(Math.round(F)),d.value=String(Math.round(L)),Nt()}return{reset:xi,getRevision:()=>U,setLocked(it){N=it===!0,N&&en()}}}function EM(r){const{camera:t,cameraRadius:e,vessel:n,voxelGroup:i,displayMaterial:s,blendMaterial:a,wireMaterial:o,onStartInjection:c,onStopInjection:l,onModeChange:h,onDebugLayerChange:u,onStartBrowserBenchmark:d,onStopBrowserBenchmark:f}=r,m=new xM(document.getElementById("ecgCanvas"),document.getElementById("bpCanvas"),document.getElementById("hrValue"),document.getElementById("bpValue"),{spo2Elem:document.getElementById("spo2Value"),mapElem:document.getElementById("mapValue"),rrElem:document.getElementById("rrValue"),rhythmElem:document.getElementById("monitorRhythm"),clockElem:document.getElementById("monitorClock")}),x=MM(),g=yM(t,n,e,x?.group||ha,x?.gantry||Yl,x?.detectorAssembly||ql,x?.lift,x?.table||zo,xp),p=document.getElementById("stiffness"),_=document.getElementById("staticFriction"),v=document.getElementById("kineticFriction"),S=document.getElementById("smoothIterations"),M=document.getElementById("modeToggle"),y=document.getElementById("renderVoxels"),w=document.getElementById("showDebugStlModel"),T=document.getElementById("showDebugLumenCast"),E=document.getElementById("showDebugSections"),A=document.getElementById("showDebugCenterline"),D=document.getElementById("showDebugCapsules"),b=document.getElementById("injectContrast"),F=document.getElementById("stopInjection"),L=document.getElementById("injRate"),N=document.getElementById("injDuration"),U=document.getElementById("injVolume"),k=document.getElementById("autoExposureToggle"),O=document.getElementById("persistence"),H=document.getElementById("pulseRate"),j=document.getElementById("noiseLevel"),J=document.getElementById("scatterStrength"),nt=document.getElementById("collimation"),V=document.getElementById("imageBrightness"),$=document.getElementById("imageContrast"),st=document.getElementById("edgeEnhancement"),at=document.getElementById("boneVisibility"),lt=document.getElementById("opacityScale"),_t=document.getElementById("gain"),ht=document.getElementById("insertedLength"),ut=document.getElementById("catheterLength"),bt=document.getElementById("catheterAdvance"),Y=document.getElementById("catheterWithdraw"),Nt=document.getElementById("catheterRotateLeft"),wt=document.getElementById("catheterRotateRight"),Ct=document.getElementById("catheterType"),vt=document.getElementById("guidewireType"),Jt=document.getElementById("catheterTypeStatus"),Lt=document.getElementById("guidewireTypeStatus"),I=document.getElementById("currentDose"),R=document.getElementById("currentKV"),W=document.getElementById("currentMA"),Q=document.getElementById("guidewireResistanceStatus"),K=document.getElementById("guidewireResistanceReason"),tt=document.getElementById("guidewireResistanceValue"),Mt=document.getElementById("guidewireResistanceFill"),rt=document.getElementById("guidewireDiagnostics"),gt=document.getElementById("guidewireDiameter"),Pt=document.getElementById("sheathDiameter"),Wt=document.getElementById("catheterDiameter"),et=document.getElementById("perfStats"),oe=document.getElementById("runBrowserBenchmarkSmoke"),jt=document.getElementById("runBrowserBenchmarkFull"),kt=document.getElementById("stopBrowserBenchmark"),At=document.getElementById("browserBenchmarkStatus"),xt=document.getElementById("browserBenchmarkReport");gt&&(gt.textContent=`${op.toFixed(3)}" · ${cp.toFixed(3)} mm`),Pt&&(Pt.textContent=`${lp}F · ${hp.toFixed(3)} mm`),Wt&&(Wt.textContent=`${fp}F · ${pp.toFixed(3)} mm`),y&&(i.visible=y.checked);const Ft={stlModel:w?.checked??!0,lumenCast:T?.checked??!1,sections:E?.checked??!1,centerline:A?.checked??!0,capsules:D?.checked??!1};function Qt(){typeof u=="function"&&u({...Ft})}w?.addEventListener("change",z=>{Ft.stlModel=z.target.checked,Qt()}),T?.addEventListener("change",z=>{Ft.lumenCast=z.target.checked,Qt()}),E?.addEventListener("change",z=>{Ft.sections=z.target.checked,Qt()}),A?.addEventListener("change",z=>{Ft.centerline=z.target.checked,Qt()}),D?.addEventListener("change",z=>{Ft.capsules=z.target.checked,Qt()}),Qt();let fe=0,qt=0,ot=-1,B=-1,dt=-1,ft=-1,Gt=-1,Ut="",de="",ce=null,ve=null,Ce=-1,ae="",Le=Ct?.value||"pigtail",qe=vt?.value||"glidewire";const gi=.05;function As(z,mt,It,ne){if(z){z.disabled!==It&&(z.disabled=It);const Be=It?"Withdraw to 0 cm before changing selection":"";z.title!==Be&&(z.title=Be)}if(mt){const Be=It?`${ne.toFixed(1)} cm inserted`:"Ready";mt.textContent!==Be&&(mt.textContent=Be),mt.classList.contains("locked")!==It&&mt.classList.toggle("locked",It)}}function Tn(){As(vt,Lt,fe>gi,fe),As(Ct,Jt,qt>gi,qt)}Ct?.addEventListener("change",z=>{Le=z.target.value}),vt?.addEventListener("change",z=>{qe=z.target.value}),Tn(),oe?.addEventListener("click",()=>{typeof d=="function"&&d(5e3)}),jt?.addEventListener("click",()=>{typeof d=="function"&&d(6e5)}),kt?.addEventListener("click",()=>{typeof f=="function"&&f()});const Bi=Array.from(document.querySelectorAll("[data-control-tab]")),Cn=Array.from(document.querySelectorAll("[data-control-panel]"));if(Bi.length&&Cn.length){const z=mt=>{Bi.forEach(It=>{const ne=It.dataset.controlTab===mt;It.classList.toggle("active",ne),It.setAttribute("aria-selected",ne?"true":"false")}),Cn.forEach(It=>{It.classList.toggle("active",It.dataset.controlPanel===mt)})};Bi.forEach(mt=>{mt.addEventListener("click",()=>z(mt.dataset.controlTab))})}if([p,_,v,S,O,H,j,J,nt,V,$,st,at,lt,_t,U,L,N].filter(Boolean).forEach(z=>z.addEventListener("change",()=>z.blur())),y&&y.addEventListener("change",z=>{i.visible=z.target.checked}),document.querySelectorAll('#controls input[type="range"], #carm-controls input[type="range"]').forEach(z=>{const mt=z.nextElementSibling;if(!mt)return;const It=()=>{mt.textContent=z.value};It(),z.addEventListener("input",It)}),document.querySelectorAll(".section-header").forEach(z=>{z.addEventListener("click",()=>{const mt=z.nextElementSibling;z.classList.toggle("collapsed"),mt&&mt.classList.toggle("hidden")})}),k&&s.uniforms.autoExposureEnabled){let z=!!s.uniforms.autoExposureEnabled.value;const mt=()=>{s.uniforms.autoExposureEnabled.value=z,k.textContent=`Auto exposure: ${z?"On":"Off"}`,k.classList.toggle("active",z)};mt(),k.addEventListener("click",()=>{z=!z,mt(),k.blur()})}if(j&&(s.uniforms.noiseLevel.value=parseFloat(j.value),j.addEventListener("input",z=>{s.uniforms.noiseLevel.value=parseFloat(z.target.value)})),H&&s.uniforms.pulseRate){const z=mt=>{s.uniforms.pulseRate.value=parseFloat(mt.target.value)};s.uniforms.pulseRate.value=parseFloat(H.value),H.addEventListener("input",z),H.addEventListener("change",z)}if(J&&s.uniforms.scatterStrength&&(s.uniforms.scatterStrength.value=parseFloat(J.value),J.addEventListener("input",z=>{s.uniforms.scatterStrength.value=parseFloat(z.target.value)})),nt&&s.uniforms.collimation&&(s.uniforms.collimation.value=parseFloat(nt.value),nt.addEventListener("input",z=>{s.uniforms.collimation.value=parseFloat(z.target.value)})),V&&s.uniforms.imageBrightness&&(s.uniforms.imageBrightness.value=parseFloat(V.value),V.addEventListener("input",z=>{s.uniforms.imageBrightness.value=parseFloat(z.target.value)})),$&&s.uniforms.imageContrast&&(s.uniforms.imageContrast.value=parseFloat($.value),$.addEventListener("input",z=>{s.uniforms.imageContrast.value=parseFloat(z.target.value)})),st&&s.uniforms.edgeStrength&&(s.uniforms.edgeStrength.value=parseFloat(st.value),st.addEventListener("input",z=>{s.uniforms.edgeStrength.value=parseFloat(z.target.value)})),O&&(a.uniforms.decay.value=parseFloat(O.value),O.addEventListener("input",z=>{a.uniforms.decay.value=parseFloat(z.target.value)})),at&&s.uniforms.boneOpacity&&(s.uniforms.boneOpacity.value=parseFloat(at.value),at.addEventListener("input",z=>{s.uniforms.boneOpacity.value=parseFloat(z.target.value)})),lt&&s.uniforms.contrastOpacity&&(s.uniforms.contrastOpacity.value=parseFloat(lt.value)/100,lt.addEventListener("input",z=>{s.uniforms.contrastOpacity.value=parseFloat(z.target.value)/100})),_t&&s.uniforms.contrastGain&&(s.uniforms.contrastGain.value=parseFloat(_t.value),_t.addEventListener("input",z=>{s.uniforms.contrastGain.value=parseFloat(z.target.value)})),p){let z=parseFloat(p.value);Ku(z),p.addEventListener("input",mt=>{z=parseFloat(mt.target.value),Ku(z)})}if(_&&v){let z=parseFloat(_.value),mt=parseFloat(v.value);$c(z,mt),_.addEventListener("input",It=>{z=parseFloat(It.target.value),$c(z,mt)}),v.addEventListener("input",It=>{mt=parseFloat(It.target.value),$c(z,mt)})}if(S){let z=parseInt(S.value);Ju(z),S.addEventListener("input",mt=>{z=parseInt(mt.target.value),Ju(z)})}let en=!0;if(M){const z=()=>{M.classList.toggle("fluoro-active",en),M.classList.toggle("debug-active",!en),M.setAttribute("aria-pressed",String(!en)),M.setAttribute("aria-label",`Current view: ${en?"fluoroscopy":"debug"}`)};z(),s.uniforms.fluoroscopy.value=!0,M.addEventListener("click",()=>{en=!en,s.uniforms.fluoroscopy.value=en,z(),o&&o.color.set(16777215),typeof h=="function"&&h(en)}),typeof h=="function"&&h(en)}let xi=0,it=0,Ot=0;const P=z=>{it=z},G=()=>{it=0},q=z=>{Ot=z},Z=()=>{Ot=0};function X(z,mt,It){z&&(z.addEventListener("pointerdown",ne=>{mt(),z.setPointerCapture?.(ne.pointerId),ne.preventDefault()}),z.addEventListener("pointerup",It),z.addEventListener("pointercancel",It),z.addEventListener("pointerleave",ne=>{ne.buttons===0&&It()}))}X(bt,()=>P(1),G),X(Y,()=>P(-1),G),X(Nt,()=>q(-1),Z),X(wt,()=>q(1),Z),document.addEventListener("keydown",z=>{if((z.code==="KeyW"||z.code==="ArrowUp")&&(xi=1,z.preventDefault()),(z.code==="KeyS"||z.code==="ArrowDown")&&(xi=-1,z.preventDefault()),z.code==="KeyD"&&(it=1,z.preventDefault()),z.code==="KeyA"&&(it=-1,z.preventDefault()),z.code==="KeyE"&&(Ot=1,z.preventDefault()),z.code==="KeyQ"&&(Ot=-1,z.preventDefault()),z.code==="KeyC"&&en){if(typeof c=="function"){const mt=parseFloat(L.value),It=parseFloat(N.value)/1e3,ne=parseFloat(U.value);c({rate:mt,duration:It,volume:ne})}z.preventDefault()}},!0),document.addEventListener("keyup",z=>{["KeyW","KeyS","ArrowUp","ArrowDown"].includes(z.code)&&(xi=0,z.preventDefault()),["KeyA","KeyD"].includes(z.code)&&(it=0,z.preventDefault()),["KeyQ","KeyE"].includes(z.code)&&(Ot=0,z.preventDefault())},!0),window.addEventListener("blur",()=>{xi=0,it=0,Ot=0}),b&&b.addEventListener("click",()=>{if(typeof c=="function"){const z=parseFloat(L.value),mt=parseFloat(N.value)/1e3,It=parseFloat(U.value);c({rate:z,duration:mt,volume:It})}}),F&&F.addEventListener("click",()=>{typeof l=="function"&&l()});function ct(z){fe=Math.max(0,z);const mt=Math.round(fe*10);if(mt===ot)return;ot=mt;const It=(mt/10).toFixed(1);ht&&(ht.textContent=`Wire ${It} cm`),Tn()}function Tt(z){qt=Math.max(0,z);const mt=Math.round(qt*10);if(mt===B)return;B=mt;const It=(mt/10).toFixed(1);ut&&(ut.textContent=`Catheter ${It} cm`),Tn()}function Dt(z){const mt=Math.round(z*10);if(mt===dt)return;dt=mt;const It=(mt/10).toFixed(1);I&&(I.textContent=`Contrast ${It} ml`)}function St(z,mt){const It=Math.round(z),ne=Math.round(mt*10);R&&It!==ft&&(Ut=`${It} kV`,R.textContent=Ut),W&&ne!==Gt&&(de=`${(ne/10).toFixed(1)} mA`,W.textContent=de),ft=It,Gt=ne}function Ht(z,mt=""){if(!Q)return;if(z<.35){ce!==!1&&(Q.classList.add("hidden"),Q.classList.remove("strong"),K&&(K.textContent="Opór na prowadniku"),tt&&(tt.textContent="0%"),Mt&&(Mt.style.width="0%"),ce=!1,ve=!1,Ce=0,ae="");return}const It=Math.round(Math.max(0,Math.min(1,z))*100),ne=z>.72,Be=mt||"Opór na prowadniku - cofnij lekko lub zmień kierunek.";ce!==!0&&(Q.classList.remove("hidden"),ce=!0),ve!==ne&&(Q.classList.toggle("strong",ne),ve=ne),K&&ae!==Be&&(K.textContent=Be,ae=Be),Ce!==It&&(tt&&(tt.textContent=`${It}%`),Mt&&(Mt.style.width=`${It}%`),Ce=It)}function Bt(z){return Number.isFinite(z)?Math.abs(z)<10?z.toFixed(2):z.toFixed(1):"--"}function Vt(z){return Number.isFinite(z)?z<10?z.toFixed(2):z.toFixed(1):"--"}function pe(z){if(!z)return"";const mt=Number.isFinite(z.settledPenetration)?` | pen ${Bt(z.settledPenetration)}/${Bt(z.maximumPenetration)} mm`:"";return`
XPBD: adv ${Vt(z.advanceMs)} / solve ${Vt(z.solveMs)} / narrow ${Vt(z.projectMs)} / dbg ${Vt(z.diagnosticMs)} ms | q ${z.pointContactCount}+${z.diagnosticPointContactCount} | segS ${z.segmentSampleCount}${Number.isFinite(z.activeBranchCount)?` | br ${z.activeBranchCount}`:""}`+mt+`${z.foldGuarded?" | fold":""}${z.stabilityRepaired?" | repair":""}${z.withdrawalRelaxed?" | withdraw":""}`}function Ze(z=null){if(!rt)return;if(rt.classList.remove("warn","breach"),!z){rt.textContent="GW STL: debug off";return}const mt=pe(z.performance);if(!z.checkedCount||!Number.isFinite(z.minSignedDistance)){rt.textContent=`GW STL: no lumen samples${mt}`;return}rt.classList.toggle("breach",z.outsideCount>0),rt.classList.toggle("warn",z.outsideCount===0&&z.clearanceViolationCount>0),rt.textContent=`GW STL: min ${Bt(z.minSignedDistance)} mm / clr ${Bt(z.clearance)} | out ${z.outsideCount} | near ${z.clearanceViolationCount} | seg ${Bt(z.maxSegmentError)} | bend ${Bt(z.maxBendAngle)} deg`+mt}function be(z){b&&b.disabled!==!!z&&(b.disabled=!!z)}function Yn(z){F&&F.disabled!==!!z&&(F.disabled=!!z)}let me=0,te=0;function Cr(z){if(!et||(me+=z,te++,me<.25))return;const mt=(te/Math.max(1e-6,me)).toFixed(1);let It="N/A";performance.memory&&(It=(performance.memory.usedJSHeapSize/1048576).toFixed(1)+" MB"),et.textContent=`FPS: ${mt} | Mem: ${It}`,me=0,te=0}function Ae(z,mt=null){const It=!!z?.running;if(oe&&(oe.disabled=It),jt&&(jt.disabled=It),kt&&(kt.disabled=!It),!At)return;if(At.classList.remove("passed","failed"),It){if(xt&&(xt.value="Running"),z.warmingUp){At.textContent="Warming up";return}const _n=Math.floor(z.elapsedMs/1e3),br=Math.round(z.durationMs/1e3);At.textContent=`Running ${_n}/${br} s · cycle ${z.cycleIndex+1}`;return}if(!mt?.frameCount){At.textContent="Idle",xt&&(xt.value="No report");return}const ne=mt.browserAcceptance,Be=z.durationMs>=6e5&&z.elapsedMs>=6e5,Cs=Be&&!!ne?.passed;At.classList.add(Cs?"passed":"failed"),At.textContent=`${Be?Cs?"PASS":"FAIL":"Smoke"} · ${mt.averageFps.toFixed(1)} FPS · 1% ${mt.onePercentLowFps.toFixed(1)} · pen ${mt.physicsEnvelope.maxPostStepPenetrationMm.toFixed(3)} mm`,xt&&(xt.value=JSON.stringify(mt))}function _i(z){const mt=z===!0;mt&&g?.reset?.(),g?.setLocked?.(mt),document.body.classList.toggle("automated-benchmark-running",mt)}return{monitor:m,getAdvance:()=>xi,getCatheterAdvance:()=>it,getCatheterRotation:()=>Ot,getSelectedCatheterType:()=>Le,getSelectedGuidewireType:()=>qe,getFluoroscopy:()=>en,getDebugLayerState:()=>({...Ft}),updateInsertedLength:ct,updateCatheterLength:Tt,updateDose:Dt,updateXrayTechnique:St,updateGuidewireResistance:Ht,updateGuidewireDiagnostics:Ze,setInjectButtonDisabled:be,setStopInjectionDisabled:Yn,updatePerfStats:Cr,updateBrowserBenchmarkStatus:Ae,setAutomatedBenchmarkMode:_i,getCArmRevision:()=>g?.getRevision?.()??0}}const wM=/^[og]\s*(.+)?/,AM=/^mtllib /,TM=/^usemtl /,CM=/^usemap /,bd=/\s+/,Rd=new C,dl=new C,Pd=new C,Ld=new C,On=new C,fo=new Kt;function bM(){const r={objects:[],object:{},vertices:[],normals:[],colors:[],uvs:[],materials:{},materialLibraries:[],startObject:function(t,e){if(this.object&&this.object.fromDeclaration===!1){this.object.name=t,this.object.fromDeclaration=e!==!1;return}const n=this.object&&typeof this.object.currentMaterial=="function"?this.object.currentMaterial():void 0;if(this.object&&typeof this.object._finalize=="function"&&this.object._finalize(!0),this.object={name:t||"",fromDeclaration:e!==!1,geometry:{vertices:[],normals:[],colors:[],uvs:[],hasUVIndices:!1},materials:[],smooth:!0,startMaterial:function(i,s){const a=this._finalize(!1);a&&(a.inherited||a.groupCount<=0)&&this.materials.splice(a.index,1);const o={index:this.materials.length,name:i||"",mtllib:Array.isArray(s)&&s.length>0?s[s.length-1]:"",smooth:a!==void 0?a.smooth:this.smooth,groupStart:a!==void 0?a.groupEnd:0,groupEnd:-1,groupCount:-1,inherited:!1,clone:function(c){const l={index:typeof c=="number"?c:this.index,name:this.name,mtllib:this.mtllib,smooth:this.smooth,groupStart:0,groupEnd:-1,groupCount:-1,inherited:!1};return l.clone=this.clone.bind(l),l}};return this.materials.push(o),o},currentMaterial:function(){if(this.materials.length>0)return this.materials[this.materials.length-1]},_finalize:function(i){const s=this.currentMaterial();if(s&&s.groupEnd===-1&&(s.groupEnd=this.geometry.vertices.length/3,s.groupCount=s.groupEnd-s.groupStart,s.inherited=!1),i&&this.materials.length>1)for(let a=this.materials.length-1;a>=0;a--)this.materials[a].groupCount<=0&&this.materials.splice(a,1);return i&&this.materials.length===0&&this.materials.push({name:"",smooth:this.smooth}),s}},n&&n.name&&typeof n.clone=="function"){const i=n.clone(0);i.inherited=!0,this.object.materials.push(i)}this.objects.push(this.object)},finalize:function(){this.object&&typeof this.object._finalize=="function"&&this.object._finalize(!0)},parseVertexIndex:function(t,e){const n=parseInt(t,10);return(n>=0?n-1:n+e/3)*3},parseNormalIndex:function(t,e){const n=parseInt(t,10);return(n>=0?n-1:n+e/3)*3},parseUVIndex:function(t,e){const n=parseInt(t,10);return(n>=0?n-1:n+e/2)*2},addVertex:function(t,e,n){const i=this.vertices,s=this.object.geometry.vertices;s.push(i[t+0],i[t+1],i[t+2]),s.push(i[e+0],i[e+1],i[e+2]),s.push(i[n+0],i[n+1],i[n+2])},addVertexPoint:function(t){const e=this.vertices;this.object.geometry.vertices.push(e[t+0],e[t+1],e[t+2])},addVertexLine:function(t){const e=this.vertices;this.object.geometry.vertices.push(e[t+0],e[t+1],e[t+2])},addNormal:function(t,e,n){const i=this.normals,s=this.object.geometry.normals;s.push(i[t+0],i[t+1],i[t+2]),s.push(i[e+0],i[e+1],i[e+2]),s.push(i[n+0],i[n+1],i[n+2])},addFaceNormal:function(t,e,n){const i=this.vertices,s=this.object.geometry.normals;Rd.fromArray(i,t),dl.fromArray(i,e),Pd.fromArray(i,n),On.subVectors(Pd,dl),Ld.subVectors(Rd,dl),On.cross(Ld),On.normalize(),s.push(On.x,On.y,On.z),s.push(On.x,On.y,On.z),s.push(On.x,On.y,On.z)},addColor:function(t,e,n){const i=this.colors,s=this.object.geometry.colors;i[t]!==void 0&&s.push(i[t+0],i[t+1],i[t+2]),i[e]!==void 0&&s.push(i[e+0],i[e+1],i[e+2]),i[n]!==void 0&&s.push(i[n+0],i[n+1],i[n+2])},addUV:function(t,e,n){const i=this.uvs,s=this.object.geometry.uvs;s.push(i[t+0],i[t+1]),s.push(i[e+0],i[e+1]),s.push(i[n+0],i[n+1])},addDefaultUV:function(){const t=this.object.geometry.uvs;t.push(0,0),t.push(0,0),t.push(0,0)},addUVLine:function(t){const e=this.uvs;this.object.geometry.uvs.push(e[t+0],e[t+1])},addFace:function(t,e,n,i,s,a,o,c,l){const h=this.vertices.length;let u=this.parseVertexIndex(t,h),d=this.parseVertexIndex(e,h),f=this.parseVertexIndex(n,h);if(this.addVertex(u,d,f),this.addColor(u,d,f),o!==void 0&&o!==""){const m=this.normals.length;u=this.parseNormalIndex(o,m),d=this.parseNormalIndex(c,m),f=this.parseNormalIndex(l,m),this.addNormal(u,d,f)}else this.addFaceNormal(u,d,f);if(i!==void 0&&i!==""){const m=this.uvs.length;u=this.parseUVIndex(i,m),d=this.parseUVIndex(s,m),f=this.parseUVIndex(a,m),this.addUV(u,d,f),this.object.geometry.hasUVIndices=!0}else this.addDefaultUV()},addPointGeometry:function(t){this.object.geometry.type="Points";const e=this.vertices.length;for(let n=0,i=t.length;n<i;n++){const s=this.parseVertexIndex(t[n],e);this.addVertexPoint(s),this.addColor(s)}},addLineGeometry:function(t,e){this.object.geometry.type="Line";const n=this.vertices.length,i=this.uvs.length;for(let s=0,a=t.length;s<a;s++)this.addVertexLine(this.parseVertexIndex(t[s],n));for(let s=0,a=e.length;s<a;s++)this.addUVLine(this.parseUVIndex(e[s],i))}};return r.startObject("",!1),r}class RM extends uc{constructor(t){super(t),this.materials=null}load(t,e,n,i){const s=this,a=new $f(this.manager);a.setPath(this.path),a.setRequestHeader(this.requestHeader),a.setWithCredentials(this.withCredentials),a.load(t,function(o){try{e(s.parse(o))}catch(c){i?i(c):console.error(c),s.manager.itemError(t)}},n,i)}setMaterials(t){return this.materials=t,this}parse(t){const e=new bM;t.indexOf(`\r
`)!==-1&&(t=t.replace(/\r\n/g,`
`)),t.indexOf(`\\
`)!==-1&&(t=t.replace(/\\\n/g,""));const n=t.split(`
`);let i=[];for(let o=0,c=n.length;o<c;o++){const l=n[o].trimStart();if(l.length===0)continue;const h=l.charAt(0);if(h!=="#")if(h==="v"){const u=l.split(bd);switch(u[0]){case"v":e.vertices.push(parseFloat(u[1]),parseFloat(u[2]),parseFloat(u[3])),u.length>=7?(fo.setRGB(parseFloat(u[4]),parseFloat(u[5]),parseFloat(u[6])).convertSRGBToLinear(),e.colors.push(fo.r,fo.g,fo.b)):e.colors.push(void 0,void 0,void 0);break;case"vn":e.normals.push(parseFloat(u[1]),parseFloat(u[2]),parseFloat(u[3]));break;case"vt":e.uvs.push(parseFloat(u[1]),parseFloat(u[2]));break}}else if(h==="f"){const d=l.slice(1).trim().split(bd),f=[];for(let x=0,g=d.length;x<g;x++){const p=d[x];if(p.length>0){const _=p.split("/");f.push(_)}}const m=f[0];for(let x=1,g=f.length-1;x<g;x++){const p=f[x],_=f[x+1];e.addFace(m[0],p[0],_[0],m[1],p[1],_[1],m[2],p[2],_[2])}}else if(h==="l"){const u=l.substring(1).trim().split(" ");let d=[];const f=[];if(l.indexOf("/")===-1)d=u;else for(let m=0,x=u.length;m<x;m++){const g=u[m].split("/");g[0]!==""&&d.push(g[0]),g[1]!==""&&f.push(g[1])}e.addLineGeometry(d,f)}else if(h==="p"){const d=l.slice(1).trim().split(" ");e.addPointGeometry(d)}else if((i=wM.exec(l))!==null){const u=(" "+i[0].slice(1).trim()).slice(1);e.startObject(u)}else if(TM.test(l))e.object.startMaterial(l.substring(7).trim(),e.materialLibraries);else if(AM.test(l))e.materialLibraries.push(l.substring(7).trim());else if(CM.test(l))console.warn('THREE.OBJLoader: Rendering identifier "usemap" not supported. Textures must be defined in MTL files.');else if(h==="s"){if(i=l.split(" "),i.length>1){const d=i[1].trim().toLowerCase();e.object.smooth=d!=="0"&&d!=="off"}else e.object.smooth=!0;const u=e.object.currentMaterial();u&&(u.smooth=e.object.smooth)}else{if(l==="\0")continue;console.warn('THREE.OBJLoader: Unexpected line: "'+l+'"')}}e.finalize();const s=new Ne;if(s.materialLibraries=[].concat(e.materialLibraries),!(e.objects.length===1&&e.objects[0].geometry.vertices.length===0)===!0)for(let o=0,c=e.objects.length;o<c;o++){const l=e.objects[o],h=l.geometry,u=l.materials,d=h.type==="Line",f=h.type==="Points";let m=!1;if(h.vertices.length===0)continue;const x=new we;x.setAttribute("position",new se(h.vertices,3)),h.normals.length>0&&x.setAttribute("normal",new se(h.normals,3)),h.colors.length>0&&(m=!0,x.setAttribute("color",new se(h.colors,3))),h.hasUVIndices===!0&&x.setAttribute("uv",new se(h.uvs,2));const g=[];for(let _=0,v=u.length;_<v;_++){const S=u[_],M=S.name+"_"+S.smooth+"_"+m;let y=e.materials[M];if(this.materials!==null){if(y=this.materials.create(S.name),d&&y&&!(y instanceof di)){const w=new di;pi.prototype.copy.call(w,y),w.color.copy(y.color),y=w}else if(f&&y&&!(y instanceof ca)){const w=new ca({size:10,sizeAttenuation:!1});pi.prototype.copy.call(w,y),w.color.copy(y.color),w.map=y.map,y=w}}y===void 0&&(d?y=new di:f?y=new ca({size:1,sizeAttenuation:!1}):y=new Dv,y.name=S.name,y.flatShading=!S.smooth,y.vertexColors=m,e.materials[M]=y),g.push(y)}let p;if(g.length>1){for(let _=0,v=u.length;_<v;_++){const S=u[_];x.addGroup(S.groupStart,S.groupCount,_)}d?p=new ts(x,g):f?p=new Xc(x,g):p=new $t(x,g)}else d?p=new ts(x,g[0]):f?p=new Xc(x,g[0]):p=new $t(x,g[0]);p.name=l.name,s.add(p)}else if(e.vertices.length>0){const o=new ca({size:1,sizeAttenuation:!1}),c=new we;c.setAttribute("position",new se(e.vertices,3)),e.colors.length>0&&e.colors[0]!==void 0&&(c.setAttribute("color",new se(e.colors,3)),o.vertexColors=!0);const l=new Xc(c,o);s.add(l)}return s}}function PM({onLoaded:r,onError:t}={}){const e=new Ue({color:16777215,transparent:!0,opacity:.42,depthWrite:!1,depthTest:!1,blending:2,side:2}),n=new Ne;return new RM().load("res/skeleton.obj",s=>{s.traverse(c=>{c.isMesh&&(c.material=e)});const o=new tn().setFromObject(s).getCenter(new C);s.position.sub(o),s.rotation.z=-Math.PI/3,s.scale.multiplyScalar(9),s.position.x-=1760,s.position.y-=300,s.position.z-=70,n.add(s),typeof r=="function"&&r({group:n,object:s,material:e})},void 0,s=>{console.warn("Failed to load skeleton OBJ model",s),typeof t=="function"&&t(s)}),{group:n,material:e}}const LM=new C(0,1,0);function Zl(r){return new C(r.x,r.y,r.z)}function Dd(r,t){return ue.clamp(Math.floor(r),0,Math.max(0,t-1))}function jl(r,t,e){const n=ue.clamp((e-r)/Math.max(1e-6,t-r),0,1);return n*n*(3-2*n)}function Id(r,t){return t<0||t>=r.cells?0:r.core[t]*.58+r.wall[t]*.72}function Nd(r,t){return t<0||t>=r.cells?0:r.wall[t]}function DM(r,t){const e=Math.floor(t),n=t-e;return ue.lerp(Id(r,e),Id(r,e+1),n)}function IM(r,t){const e=Math.floor(t),n=t-e;return ue.lerp(Nd(r,e),Nd(r,e+1),n)}function NM(r,t){return r.segments.map((e,n)=>{const i=Zl(e.start),s=Zl(e.end),a=new C().subVectors(s,i),o=Math.max(1,a.length()),c=a.clone().normalize(),l=Math.max(2,Math.ceil(o/t)),h=o/l;return{sourceSegment:e,segmentIndex:n,start:i,end:s,dir:c,length:o,cells:l,cellLength:h,radius:e.radius,area:Math.PI*e.radius*e.radius,flowSpeed:e.flowSpeed||0,isSheath:!!e.isSheath,core:new Float32Array(l),wall:new Float32Array(l),nextCore:new Float32Array(l),nextWall:new Float32Array(l),orientation:new Fn().setFromUnitVectors(LM,c)}})}class FM{constructor(t,e=3.5){this.vessel=t,this.segments=NM(t,e),this.segmentGraph=t.segmentGraph||t.segments.map(()=>[]),this.outgoing=Array.from({length:this.segments.length*2},()=>({segmentIndex:-1,amount:0,wallShare:0,sourceArea:0})),this.outgoingCount=0,this.sheathSegmentIndex=t.segments.findIndex(n=>n.isSheath),this.time=0,this.totalSignal=0,this.lastInjectionTime=-1/0,this.coreSpeedScale=1.82,this.wallSpeedScale=1.24,this.wallExchange=4.6,this.axialDispersion=.46,this.clearance=.95,this.tailClearance=3.1}injectThroughSheath(t,e=0){if(t<=0)return;if(this.lastInjectionTime=this.time,this.sheathSegmentIndex>=0){const s=this.segments[this.sheathSegmentIndex];this.#i(s,s.cells-1,t*.06,.48)}const n=this.vessel.sheath?.end,i=n?this.#l(n,{excludeSheath:!0}):null;if(!i){const s=this.segments.find(a=>!a.isSheath);s&&this.#i(s,0,t,.85);return}this.#t(i,t*.92,e)}update(t){if(this.time+=t,this.totalSignal<=0)return;this.totalSignal=0;const e=1+.18*Math.sin(this.time*Math.PI*2.15);this.outgoingCount=0;for(let i=0;i<this.segments.length;i++){const s=this.segments[i];s.nextCore.set(s.core),s.nextWall.set(s.wall);const a=Math.min(.96,Math.max(0,s.flowSpeed*this.coreSpeedScale*e*t/s.cellLength)),o=Math.min(.78,Math.max(0,s.flowSpeed*this.wallSpeedScale*t/s.cellLength));this.#a(s,s.core,s.nextCore,a,1),this.#a(s,s.wall,s.nextWall,o,.35),s.core.set(s.nextCore),s.wall.set(s.nextWall),this.#f(s,t)}for(let i=0;i<this.outgoingCount;i++)this.#c(this.outgoing[i]);const n=this.time-this.lastInjectionTime>.38;for(let i=0;i<this.segments.length;i++){const s=this.segments[i];for(let a=0;a<s.cells;a++){const o=s.core[a]+s.wall[a]*.8,c=n?1-jl(.012,.13,o):0,l=Math.exp(-(this.clearance+c*this.tailClearance)*t),h=Math.exp(-(this.clearance*1.2+c*this.tailClearance*1.35)*t);s.core[a]*=l,s.wall[a]*=h,s.core[a]<4e-4&&(s.core[a]=0),s.wall[a]<4e-4&&(s.wall[a]=0),this.totalSignal+=s.core[a]+s.wall[a]*.8}}}hasVisibleContrast(t=.02){return this.totalSignal>t}#t(t,e,n){const i=t.segment,s=t.cellIndex,a=ue.clamp(n/45,.35,1.35);this.#i(i,s,e*.25,.72);const o=s+1,c=Math.max(5,Math.min(o,Math.round(18*a)));let l=0;const h=[];for(let f=0;f<c;f++){const m=s-f;if(m<0)break;const x=Math.exp(-f/(5.5+a*4));h.push([m,x]),l+=x}for(const[f,m]of h)this.#i(i,f,e*.4*m/l,.64);const u=i.sourceSegment?.parent,d=Number.isInteger(u)?this.segments[u]:null;if(d){const f=Math.min(d.cells,Math.round(24*a));let m=0;const x=[];for(let g=0;g<f;g++){const p=d.cells-1-g,_=Math.exp(-g/8);x.push([p,_]),m+=_}for(const[g,p]of x)this.#i(d,g,e*.35*p/m,.58)}}#i(t,e,n,i){if(!t||n<=0)return;const s=Dd(e,t.cells),a=Math.max(1,t.area*t.cellLength),o=n*1e3/a;t.core[s]+=o*i,t.wall[s]+=o*(1-i),this.totalSignal+=o}#a(t,e,n,i,s){if(!(i<=0))for(let a=t.cells-1;a>=0;a--){const o=e[a]*i;if(n[a]-=o,a+1<t.cells)n[a+1]+=o;else if(o>0){const c=this.outgoing[this.outgoingCount++];c.segmentIndex=t.segmentIndex,c.amount=o,c.wallShare=s,c.sourceArea=t.area}}}#f(t,e){const n=ue.clamp(this.wallExchange*e,0,.22),i=ue.clamp(this.axialDispersion*e,0,.08);for(let s=0;s<t.cells;s++){const a=(t.core[s]-t.wall[s])*n;t.core[s]-=a,t.wall[s]+=a}if(!(i<=0||t.cells<3)){t.nextCore.set(t.core),t.nextWall.set(t.wall);for(let s=1;s<t.cells-1;s++)t.nextCore[s]+=(t.core[s-1]+t.core[s+1]-t.core[s]*2)*i,t.nextWall[s]+=(t.wall[s-1]+t.wall[s+1]-t.wall[s]*2)*i*1.35;t.core.set(t.nextCore),t.wall.set(t.nextWall)}}#c(t){const e=this.segmentGraph[t.segmentIndex]||[];if(!e.length)return;let n=0;for(let i=0;i<e.length;i++)n+=this.segments[e[i]]?.area||0;n<=0&&(n=e.length);for(let i=0;i<e.length;i++){const s=e[i],a=this.segments[s];if(!a)continue;const o=(a.area||1)/n,c=t.amount*o*t.sourceArea*a.cellLength/1e3;this.#i(a,0,c,.58+(1-t.wallShare)*.18)}}#l(t,{excludeSheath:e=!1}={}){const n=Zl(t);let i=null;for(const s of this.segments){if(e&&s.isSheath)continue;const a=new C().subVectors(n,s.start),o=ue.clamp(a.dot(s.dir),0,s.length),l=s.start.clone().addScaledVector(s.dir,o).distanceTo(n),h=Math.max(0,l-s.radius),u=Dd(o/s.cellLength,s.cells);(!i||h<i.score)&&(i={segment:s,segmentIndex:s.segmentIndex,cellIndex:u,score:h})}return i}}function UM(r,t=.015,e=!1,n=null){if(!r?.segments)return{mesh:n,count:0};const i=r.vessel?.geometry;if(!i?.attributes?.position)return{mesh:n,count:0};if(!n||!n.isMesh||n.userData.sourceGeometry!==i){zM(n);const c=i.clone(),l=c.attributes.position.count;c.setAttribute("color",new Fe(new Float32Array(l*3),3));const h=new Ue({vertexColors:!0,transparent:!0,opacity:e?.78:.96,blending:2,depthTest:!1,depthWrite:!1,side:2,wireframe:e});n=new $t(c,h),n.frustumCulled=!1,n.userData.sourceGeometry=i,n.userData.influences=BM(r,c)}const s=n.geometry.attributes.color,a=n.userData.influences||[];let o=0;for(let c=0;c<s.count;c++){const l=a[c];let h=0,u=0;if(l?.length)for(const g of l){const p=r.segments[g.segmentIndex];h+=DM(p,g.cellFloat)*g.weight,u+=IM(p,g.cellFloat)*g.weight}const d=jl(t*.18,t*4.4,h),f=jl(t*.28,t*4,u),m=Math.max(d,f*.82),x=m>.018?Math.min(1,Math.pow(m,.78)*1.18):0;x>.02&&o++,s.setXYZ(c,x,x,x)}return s.needsUpdate=!0,n.visible=o>0,n.material.wireframe=e,n.material.opacity=e?.78:.96,{mesh:n,count:o}}function BM(r,t){const e=t.attributes.position,n=new C,i=new Array(e.count);for(let s=0;s<e.count;s++){n.fromBufferAttribute(e,s);const a=[];for(const l of r.segments){if(l.isSheath)continue;const u=new C().subVectors(n,l.start).dot(l.dir),d=ue.clamp(u,0,l.length),f=l.start.clone().addScaledVector(l.dir,d),m=n.distanceTo(f),x=Math.max(0,-u,u-l.length),g=Math.abs(m-l.radius)+x*.45,p=Math.max(2,l.radius*.48),_=Math.exp(-(g*g)/(2*p*p));if(_<.02)continue;const v=ue.clamp(d/l.cellLength-.5,0,l.cells-1);a.push({segmentIndex:l.segmentIndex,cellFloat:v,weight:_})}a.sort((l,h)=>h.weight-l.weight);const o=a.slice(0,3),c=o.reduce((l,h)=>l+h.weight,0);i[s]=c>0?o.map(l=>({...l,weight:l.weight/c})):[]}return i}function zM(r){if(r)if(r.isGroup)for(const t of r.children)t.geometry?.dispose?.(),t.material?.dispose?.();else r.geometry?.dispose?.(),r.material?.dispose?.()}const $l=Zo,Kl=7.2,Jl=1.05,Oo=Kl*Jl*Math.PI*2,Fd="pigtail",ls="berenstein",Ud=Math.PI/4,Bd=8,po=10,OM=48,_p=16,mo=_p+Oo,Wi=18,hs=4,us=3.2,GM=76,VM=.105,fl=.96,zd=.88,kM=18,HM=4,pl=.085,ml=.08,WM=.22,XM=.075,gl=72*Math.PI/180,YM=.36,Od=.55,Gd=1.2,qM=[.25,.5,.75],Vd=[2,4,7],ZM=3,kd=[2,4,8],jM=.24,$M=.085,xl=68*Math.PI/180,KM=.42,Hd=1.15,_l=1.7,JM=1.45,vl=7,QM=.22,ty=.42,Sl=.42,Wd=.82,ey=.65,ny=1.5,iy=5e-5,sy=.78,Xd=1.2,ry=90,ay=52,oy=32,cy=Math.PI*.9,Yd=$l*.72;class Rt extends C{constructor(t=0,e=0,n=0){super(t,e,n),this._values=new Float64Array([this._initialX??t,this._initialY??e,this._initialZ??n])}get x(){return this._values?this._values[0]:this._initialX}set x(t){this._values?this._values[0]=t:this._initialX=t}get y(){return this._values?this._values[1]:this._initialY}set y(t){this._values?this._values[1]=t:this._initialY=t}get z(){return this._values?this._values[2]:this._initialZ}set z(t){this._values?this._values[2]=t:this._initialZ=t}}function Sn(r,t,e){return Math.sqrt(r*r+t*t+e*e)}function Jr(r){return new Rt(r.x,r.y,r.z)}class ly{constructor({wire:t,segmentLength:e,guidewireLength:n,tailProgressRef:i,vessel:s=null,maxLength:a=1e3}){this.wire=t,this.segmentLength=e,this.guidewireLength=n,this.tailProgressRef=i,this.vessel=s,this.vesselColliders=this.#Z(s),this.collisionMesh=null,this.sheathPath=this.#j(s?.sheath),this.maxLength=a,this.progress=0,this.guidewireInserted=0,this.previousGuidewireInserted=0,this.guidewireDelta=0,this.motionCommand=0,this.rotation=0,this.type=Fd,this.pathSpacing=4,this.pathSamples=[],this._pathSamplePool=Array.from({length:Math.ceil(a/this.pathSpacing)+4},()=>({distance:0,point:new Rt})),this.freeNodes=[],this._nextFreeNodes=[],this._freeNodePool=[],this._freeNodeEpoch=0,this.freeRestDistances=new Float64Array(Math.ceil(a/us)+2),this.freeRestDistanceCount=0,this.freeLength=0,this._physicsStepIndex=0,this.material=new Ue({color:16777215,depthTest:!1,transparent:!0,opacity:1}),this.maxRenderSegments=320,this.mesh=new wr(new is(cl,cl,1,10,1,!1),this.material,this.maxRenderSegments),this.mesh.instanceMatrix.setUsage(35048),this.mesh.count=0,this.mesh.frustumCulled=!1,this.mesh.renderOrder=7,this.mesh.visible=!1,this.physicsBody=null,this.physicsActiveCount=0,this.physicsLumenStartNode=0,this._xpbdLayoutX=null,this._xpbdLayoutY=null,this._xpbdLayoutZ=null,this._xpbdLayoutCount=0,this._guidewireRelease=1,this.externalCollisionSolver=!1,this._renderAxis=new Rt,this._renderMidpoint=new Rt,this._renderUp=new Rt(0,1,0),this._renderQuaternion=new Fn,this._renderScale=new Rt(1,1,1),this._renderMatrix=new re,this._shapeNormal=new Rt,this._pathTarget=new Rt,this._newNodeRest=new Rt,this._newNodePath=new Rt,this._newNodeGuide=new Rt,this._newNodePoint=new Rt,this._centerlinePoints=[],this._centerlineDistances=[],this._centerlinePointCount=0,this._deploymentStateScratch={pathEnd:0,supportEnd:0,freeLength:0},this._freeFrameScratch={supportTip:new Rt,beforeTip:new Rt,beforePlane:new Rt,tangent:new Rt,normal:new Rt},this._guideReleaseFrameScratch={supportTip:new Rt,beforeTip:new Rt,tangent:new Rt,normal:new Rt},this._planePreviousTangent=new Rt,this._planeCurvature=new Rt,this._planeHelper=new Rt}setType(t){const e=this.#nt(t);this.type!==e&&(this.type=e,this.#m(),this.freeRestDistanceCount=0,this.freeLength=0,this._physicsStepIndex=0,this.physicsLumenStartNode=0,this.updateMesh())}dispose(){this.mesh.geometry?.dispose?.(),this.material.dispose()}setExternalCollisionSolver(t=!0){return this.externalCollisionSolver=!!t,this}reset(){return this.progress=0,this.guidewireInserted=0,this.previousGuidewireInserted=0,this.guidewireDelta=0,this.motionCommand=0,this.rotation=0,this.pathSamples.length=0,this.#m(),this.freeRestDistanceCount=0,this.freeLength=0,this._physicsStepIndex=0,this.physicsActiveCount=0,this._xpbdLayoutCount=0,this._guidewireRelease=1,this.updateMesh(),this}syncXpbdBody(t,{shapeCompliance:e=t.shapeCompliance,targetSlewLimit:n=1,restLengthSlewLimit:i=.5,bendChordSlewLimit:s=1}={}){const a=this.#l(),o=Math.min(this._centerlinePointCount,t.count);if((this.physicsBody!==t||!this._xpbdLayoutX||this._xpbdLayoutX.length!==t.count)&&(this._xpbdLayoutX=new Float64Array(t.count),this._xpbdLayoutY=new Float64Array(t.count),this._xpbdLayoutZ=new Float64Array(t.count),this._xpbdLayoutCount=0,this.physicsActiveCount=0),this.physicsBody=t,o<2){for(let m=0;m<this.physicsActiveCount;m++)t.clearRestShapeTarget(m);return t.setActiveRange(0,1),t.setCollisionRange(0,-1),this.physicsActiveCount=0,this._xpbdLayoutCount=0,0}const c=this.physicsActiveCount;let l=-1,h=!1;if(c>0&&this._xpbdLayoutCount===c&&o===c+1){l=this.freeNodes.length>=2?o-1:this.#t(a,o,c);for(let m=o-1;m>l;m--)this.#a(t,m,m-1);this.#f(t,a,l,o,e),h=!0}else if(o>1&&this._xpbdLayoutCount===c&&o===c-1){const m=this.freeNodes.length>=2?c-1:this.#i(a,o,c);for(let x=m;x<o;x++)this.#a(t,x,x+1);h=!0}if(t.setActiveRange(0,o-1),h){for(let m=0;m<o-1;m++)t.restLength[m]=Math.max(.5,Sn(t.x[m+1]-t.x[m],t.y[m+1]-t.y[m],t.z[m+1]-t.z[m]));for(let m=1;m<o-1;m++)t.restBendChord[m]=Sn(t.x[m+1]-t.x[m-1],t.y[m+1]-t.y[m-1],t.z[m+1]-t.z[m-1])}let u=o-1;const d=this.vessel?.sheath;if(d){const m=d.end.x-d.start.x,x=d.end.y-d.start.y,g=d.end.z-d.start.z,p=Sn(m,x,g)||1,_=m/p,v=x/p,S=g/p;for(let M=0;M<o;M++){const y=a[M];if((y.x-d.start.x)*_+(y.y-d.start.y)*v+(y.z-d.start.z)*S>p+.25){u=Math.max(0,M-1);break}}}const f=this.externalCollisionSolver&&this.guidewireInserted<=Wi;for(let m=0;m<o;m++){const x=a[m],g=this._centerlineDistances[m]??1/0,p=this.guidewireInserted>Wi&&g<=this.guidewireInserted+hs,_=m===l||l<0&&m>=c;if(_&&m!==l)if(c>0&&m>0){const y=a[m-1];let w=x.x-y.x,T=x.y-y.y,E=x.z-y.z,A=Sn(w,T,E);A<1e-6&&m>1&&(w=t.x[m-1]-t.x[m-2],T=t.y[m-1]-t.y[m-2],E=t.z[m-1]-t.z[m-2],A=Sn(w,T,E));const D=Math.max(.5,x.distanceTo(y)),b=1/Math.max(1e-6,A);t.setNodePosition(m,t.x[m-1]+w*b*D,t.y[m-1]+T*b*D,t.z[m-1]+E*b*D)}else t.setNodePosition(m,x.x,x.y,x.z);let v=x.x,S=x.y,M=x.z;if(c>0&&_)v=t.x[m],S=t.y[m],M=t.z[m];else if(c>0&&t.restShapeEnabled[m]&&Number.isFinite(n)&&n>0){const y=x.x-t.restShapeX[m],w=x.y-t.restShapeY[m],T=x.z-t.restShapeZ[m],E=Sn(y,w,T);if(E>n){const A=n/E;v=t.restShapeX[m]+y*A,S=t.restShapeY[m]+w*A,M=t.restShapeZ[m]+T*A}}if(m>u&&!p?t.clearRestShapeTarget(m):t.setRestShapeTarget(m,v,S,M,e),t.nodeRadius[m]=$l,t.bendComplianceByNode[m]=f?Math.min(t.bendCompliance,iy):t.bendCompliance,m>0){const y=a[m-1],w=Math.max(.5,x.distanceTo(y));c>0&&i>0?t.restLength[m-1]+=zt(w-t.restLength[m-1],-i,i):t.restLength[m-1]=w}if(m>0&&m<o-1){const y=f?a[m-1].distanceTo(x)+x.distanceTo(a[m+1]):a[m-1].distanceTo(a[m+1]);c>0&&s>0?t.restBendChord[m]+=zt(y-t.restBendChord[m],-s,s):t.restBendChord[m]=y}}for(let m=o;m<c;m++)t.clearRestShapeTarget(m);t.setCollisionRange(u,o-2),this.physicsActiveCount=o;for(let m=0;m<o;m++)this._xpbdLayoutX[m]=a[m].x,this._xpbdLayoutY[m]=a[m].y,this._xpbdLayoutZ[m]=a[m].z;return this._xpbdLayoutCount=o,o}#t(t,e,n){let i=e-1,s=1/0;for(let a=0;a<e;a++){let o=0;for(let c=0;c<n;c++){const l=c<a?c:c+1,h=t[l],u=h.x-this._xpbdLayoutX[c],d=h.y-this._xpbdLayoutY[c],f=h.z-this._xpbdLayoutZ[c];o+=u*u+d*d+f*f}o<s&&(s=o,i=a)}return i}#i(t,e,n){let i=n-1,s=1/0;for(let a=0;a<n;a++){let o=0;for(let c=0;c<e;c++){const l=c<a?c:c+1,h=t[c],u=h.x-this._xpbdLayoutX[l],d=h.y-this._xpbdLayoutY[l],f=h.z-this._xpbdLayoutZ[l];o+=u*u+d*d+f*f}o<s&&(s=o,i=a)}return i}#a(t,e,n){t.x[e]=t.x[n],t.y[e]=t.y[n],t.z[e]=t.z[n],t.previousX[e]=t.previousX[n],t.previousY[e]=t.previousY[n],t.previousZ[e]=t.previousZ[n],t.velocityX[e]=t.velocityX[n],t.velocityY[e]=t.velocityY[n],t.velocityZ[e]=t.velocityZ[n],t.inverseMass[e]=t.inverseMass[n],t.nodeRadius[e]=t.nodeRadius[n],t.pinned[e]=t.pinned[n],t.bendComplianceByNode[e]=t.bendComplianceByNode[n],t.restShapeEnabled[e]=t.restShapeEnabled[n],t.restShapeX[e]=t.restShapeX[n],t.restShapeY[e]=t.restShapeY[n],t.restShapeZ[e]=t.restShapeZ[n],t.restShapeCompliance[e]=t.restShapeCompliance[n]}#f(t,e,n,i,s){if(n>0&&n+1<i){const a=e[n],o=e[n-1],c=e[n+1],l=a.distanceTo(o),h=a.distanceTo(c),u=l/Math.max(1e-6,l+h);t.x[n]=t.x[n-1]+(t.x[n+1]-t.x[n-1])*u,t.y[n]=t.y[n-1]+(t.y[n+1]-t.y[n-1])*u,t.z[n]=t.z[n-1]+(t.z[n+1]-t.z[n-1])*u,t.previousX[n]=t.previousX[n-1]+(t.previousX[n+1]-t.previousX[n-1])*u,t.previousY[n]=t.previousY[n-1]+(t.previousY[n+1]-t.previousY[n-1])*u,t.previousZ[n]=t.previousZ[n-1]+(t.previousZ[n+1]-t.previousZ[n-1])*u,t.velocityX[n]=t.velocityX[n-1]+(t.velocityX[n+1]-t.velocityX[n-1])*u,t.velocityY[n]=t.velocityY[n-1]+(t.velocityY[n+1]-t.velocityY[n-1])*u,t.velocityZ[n]=t.velocityZ[n-1]+(t.velocityZ[n+1]-t.velocityZ[n-1])*u}else if(n>0){const a=e[n],o=e[n-1],c=this.externalCollisionSolver&&(this.guidewireInserted<=Wi||(this._centerlineDistances[n]??1/0)>this.guidewireInserted)&&n>1;let l=c?t.x[n-1]-t.x[n-2]:a.x-o.x,h=c?t.y[n-1]-t.y[n-2]:a.y-o.y,u=c?t.z[n-1]-t.z[n-2]:a.z-o.z,d=Sn(l,h,u);d<1e-6&&n>1&&(l=t.x[n-1]-t.x[n-2],h=t.y[n-1]-t.y[n-2],u=t.z[n-1]-t.z[n-2],d=Sn(l,h,u));const m=Math.max(.5,a.distanceTo(o))/Math.max(1e-6,d);t.x[n]=t.x[n-1]+l*m,t.y[n]=t.y[n-1]+h*m,t.z[n]=t.z[n-1]+u*m,t.previousX[n]=t.x[n],t.previousY[n]=t.y[n],t.previousZ[n]=t.z[n],t.velocityX[n]=0,t.velocityY[n]=0,t.velocityZ[n]=0}else t.setNodePosition(n,e[n].x,e[n].y,e[n].z);t.restShapeEnabled[n]=1,t.restShapeX[n]=t.x[n],t.restShapeY[n]=t.y[n],t.restShapeZ[n]=t.z[n],t.restShapeCompliance[n]=s,t.shapeLambda[n]=0}setCollisionGeometry(t){const e=t?.geometry||t;if(!e?.boundsTree){this.collisionMesh=null;return}this.collisionMesh={geometry:e,meshCollider:t?.meshCollider||null,clearance:Math.max($l*.7,t?.clearance||0),interiorDirection:t?.interiorDirection||t?.collisionInteriorDirection||null}}advance(t,e,n){this.motionCommand=t,this.previousGuidewireInserted=this.guidewireInserted,this.guidewireInserted=Math.max(0,n),this.guidewireDelta=this.guidewireInserted-this.previousGuidewireInserted;const i=t>0?ay:oy,s=zt(this.progress+t*i*e,0,this.maxLength);s>this.progress?this.#H(Math.min(s,this.guidewireInserted)):s<this.progress&&this.#K(s);const a=Math.min(s,this.guidewireInserted);(t!==0||this.guidewireDelta>0)&&a>Wi&&this.#$(a),this.progress=s}rotate(t,e){t&&(this.rotation+=t*cy*e)}stepPhysics(t=1/60,{collisions:e=!0}={}){const n=this.#u();this.#L(t);const i=this._physicsStepIndex++;if((!this.externalCollisionSolver||(i&3)===0)&&this.#v(n.pathEnd),this.externalCollisionSolver){this.#c(n,t);return}if(n.freeLength<2||n.supportEnd<=0){this.#m(),this.freeRestDistanceCount=0,this.freeLength=0;return}const s=this.#s(n.supportEnd);if(this.#n(n,s),this.freeNodes.length<2)return;this.#g(n);const a=s.supportTip;for(let l=0;l<this.freeNodes.length;l++){const h=this.freeNodes[l];h.previousPos||=new Rt,h.shapeTarget||=new Rt,h.guideTarget||=new Rt,h.previousPos.copy(h.pos)}this.freeNodes[0].pos.copy(a),this.freeNodes[0].vel.set(0,0,0);for(let l=1;l<this.freeNodes.length;l++){const h=this.freeNodes[l];h.curl=Math.min(1,(h.curl??1)+Wd*t);const u=Math.max(0,(h.distance??0)-(this.freeNodes[0].distance??0)),d=this.#o(u,s,n.freeLength,h.curl,h.shapeTarget),f=GM*t;h.vel.x+=(d.x-h.pos.x)*f,h.vel.y+=(d.y-h.pos.y)*f,h.vel.z+=(d.z-h.pos.z)*f,h.vel.multiplyScalar(zd),h.pos.addScaledVector(h.vel,t)}const o=e?kM:HM;for(let l=0;l<o;l++)this.freeNodes[0].pos.copy(a),this.#g(n),this.#r(),this.#N(s,n.freeLength),this.#h(s),this.#U(n.freeLength),this.#x(n.freeLength),e&&(this.#A(),this.#T()),this.#r();const c=1/Math.max(1e-4,t);for(let l=1;l<this.freeNodes.length;l++){const h=this.freeNodes[l];h.vel.subVectors(h.pos,h.previousPos).multiplyScalar(c*zd)}this.freeNodes[0].vel.set(0,0,0)}#c(t,e){if(t.freeLength<2||t.supportEnd<=0){this.#m(),this.freeRestDistanceCount=0,this.freeLength=0;return}const n=this.#s(t.supportEnd);if(this.#n(t,n),this.freeNodes.length<2)return;this.freeNodes[0].pos.copy(n.supportTip),this.freeNodes[0].vel.set(0,0,0);const i=this.freeNodes[0].distance??t.supportEnd;for(let s=1;s<this.freeNodes.length;s++){const a=this.freeNodes[s];a.curl=Math.min(1,(a.curl??1)+Wd*e);const o=Math.max(0,(a.distance??i)-i);a.pos.copy(this.#o(o,n,t.freeLength,1,a.shapeTarget)),a.vel.set(0,0,0)}}constrainGuidewire(t=1/60,{reactionScale:e=1}={}){if(this.progress<4)return;const n=this.#u();n.freeLength>=2&&this.freeNodes.length<2&&n.supportEnd>0&&this.#n(n,this.#s(n.supportEnd));const i=this.tailProgressRef(),s=this.freeNodes.length>=2?Math.max(n.pathEnd,this.progress):n.pathEnd,a=Math.min(this.progress,this.guidewireInserted,s);if(!(a<=0))for(let o=0;o<this.wire.nodes.length;o++){const c=this.#et(o,i);if(c<=0||c>a)continue;const l=this.wire.nodes[o];if(l.pinned)continue;const h=this.#J(c,n),u=l.x,d=l.y,f=l.z,x=(.6+pn(0,this.segmentLength*1.5,c)*.4)*sy,g=h.clone().sub(new Rt(u,d,f)),p=g.length();p>Xd&&g.multiplyScalar(Xd/p);const _=g.multiplyScalar(x);l.x=u+_.x,l.y=d+_.y,l.z=f+_.z;const v=1/Math.max(1e-4,t);l.vx=(l.x-u)*v*.25,l.vy=(l.y-d)*v*.25,l.vz=(l.z-f)*v*.25,this.#Q(c,_,e)}}updateMesh(){const t=this.physicsBody,e=t?null:this.#l(),n=t?this.physicsActiveCount:this._centerlinePointCount;if(n<2){this.mesh.count=0,this.mesh.visible=!1;return}const i=Math.min(n-1,this.maxRenderSegments);let s=0;for(let a=0;a<i;a++){const o=t?t.x[a]:e[a].x,c=t?t.y[a]:e[a].y,l=t?t.z[a]:e[a].z,h=t?t.x[a+1]:e[a+1].x,u=t?t.y[a+1]:e[a+1].y,d=t?t.z[a+1]:e[a+1].z;this._renderAxis.set(h-o,u-c,d-l);const f=this._renderAxis.length();f<1e-6||(this._renderAxis.multiplyScalar(1/f),this._renderMidpoint.set((o+h)*.5,(c+u)*.5,(l+d)*.5),this._renderQuaternion.setFromUnitVectors(this._renderUp,this._renderAxis),this._renderScale.set(1,f+cl*.65,1),this._renderMatrix.compose(this._renderMidpoint,this._renderQuaternion,this._renderScale),this.mesh.setMatrixAt(s++,this._renderMatrix))}this.mesh.count=s,this.mesh.instanceMatrix.needsUpdate=!0,this.mesh.visible=s>0}#l(){const t=this.#u(),e=this.sheathPath?ry:0;if(this.physicsLumenStartNode=0,this._centerlinePointCount=0,t.pathEnd<=0&&e<=0)return this._centerlinePoints;const n=Math.max(0,t.supportEnd),i=n>0?zt(Math.ceil(n/5),1,90):0,s=this._centerlinePoints;if(e>0){const c=zt(Math.ceil(e/6),2,24);for(let l=0;l<=c;l++){const h=-e+e*l/c,u=this._centerlinePointCount++;this.#R(h,this.#p(u)),this._centerlineDistances[u]=h}this.physicsLumenStartNode=c}if(t.pathEnd<=0)return s;const a=this._centerlinePointCount?1:0;for(let c=a;c<=i;c++){const l=i>0?n*c/i:0,h=this._centerlinePointCount++;this.#R(l,this.#p(h)),this._centerlineDistances[h]=l}if(t.freeLength<2){if(t.pathEnd>n+.5){const c=this._centerlinePointCount++;this.#R(t.pathEnd,this.#p(c)),this._centerlineDistances[c]=t.pathEnd}return s}const o=this.#s(t.supportEnd);this.#n(t,o);for(let c=1;c<this.freeNodes.length;c++){const l=this._centerlinePointCount++;this.#p(l).copy(this.freeNodes[c].pos),this._centerlineDistances[l]=this.freeNodes[c].distance??t.supportEnd}return s}#p(t){let e=this._centerlinePoints[t];return e||(e=new Rt,this._centerlinePoints[t]=e),e}#u(){const t=this._deploymentStateScratch;if(this.progress<4)return t.pathEnd=0,t.supportEnd=0,t.freeLength=0,t;const e=this.#O(),n=Math.max(e,Math.min(this.progress,this.#I()));return t.pathEnd=n,t.supportEnd=n>0?e:0,t.freeLength=n>0?Math.max(0,this.progress-e):0,t}#s(t){const e=this._freeFrameScratch,n=this.#R(t,e.supportTip),i=this.#R(Math.max(0,t-10),e.beforeTip),s=this.#R(Math.max(0,t-28),e.beforePlane),a=e.tangent.subVectors(n,i);return a.lengthSq()<1e-5&&a.set(0,1,0),a.normalize(),this.#tt(a,i,s,e.normal).applyAxisAngle(a,this.rotation).normalize(),e}#n(t,e){const n=this.freeRestDistances;n[0]=t.supportEnd;let i=1,s=t.supportEnd;for(;s+us<this.progress-.5;)s+=us,n[i++]=s;this.progress>n[i-1]+.5&&(n[i++]=this.progress),this.freeRestDistanceCount=i;const a=this.freeNodes,o=this._nextFreeNodes;o.length=0;const c=++this._freeNodeEpoch;let l=0;for(let h=0;h<i;h++){const u=n[h],d=u-t.supportEnd;let f=-1,m=1/0;for(;l<a.length;){const g=Math.abs((a[l].distance??0)-u);if((l+1<a.length?Math.abs((a[l+1].distance??0)-u):1/0)>=g){f=l,m=g;break}l++}let x;if(f>=0&&m<=us*.7)x=a[f],l=f+1;else{const g=this.guidewireDelta<-1e-4&&u>=this.guidewireInserted-hs&&u<=this.previousGuidewireInserted+hs,p=this.#o(d,e,t.freeLength,g?Sl:1,this._newNodeRest),_=this.#R(Math.min(u,this.#I()),this._newNodePath),v=this.guidewireInserted>Wi&&u<=this.guidewireInserted+hs,S=this._newNodePoint;g?S.copy(_).lerp(p,Sl):v?S.copy(this.#P(u,this._newNodeGuide)).lerp(p,.28):S.copy(p);const M=this.externalCollisionSolver?S:this.#D(S).point;x=this.#S(M,u,g?Sl:1)}x._activeEpoch=c,x.distance=u,x.curl=x.curl??1,x.previousPos||=new Rt,x.shapeTarget||=new Rt,x.guideTarget||=new Rt,o.push(x)}for(let h=0;h<a.length;h++){const u=a[h];u._activeEpoch===c||u._pooled||(u._pooled=!0,this._freeNodePool.push(u))}this._nextFreeNodes=a,this.freeNodes=o,this.freeLength=t.freeLength,this.freeNodes[0]&&(this.freeNodes[0].pos.copy(e.supportTip),this.freeNodes[0].vel.set(0,0,0))}#S(t,e,n){const i=this._freeNodePool.pop()||{pos:new Rt,vel:new Rt,previousPos:new Rt,shapeTarget:new Rt,guideTarget:new Rt,distance:0,curl:1,_activeEpoch:0,_pooled:!1};return i._pooled=!1,i.pos.copy(t),i.vel.set(0,0,0),i.previousPos.copy(t),i.shapeTarget.copy(t),i.guideTarget.copy(t),i.distance=e,i.curl=n,i}#m(){for(let t=0;t<2;t++){const e=t===0?this.freeNodes:this._nextFreeNodes;for(let n=0;n<e.length;n++){const i=e[n];i._pooled||(i._pooled=!0,this._freeNodePool.push(i))}e.length=0}}#g(t){const e=Math.min(this.progress,this.guidewireInserted);if(e<=t.supportEnd+.5||this.freeNodes.length<2)return;const i=Math.abs(this.motionCommand)>0?ty:QM;for(let s=1;s<this.freeNodes.length;s++){const a=this.freeNodes[s].distance??t.supportEnd;if(a>e+hs)continue;const o=pn(t.supportEnd,t.supportEnd+vl,a),c=1-pn(e-vl,e+hs,a),l=this.#P(a,this.freeNodes[s].guideTarget),h=i*o*(.35+c*.65);this.freeNodes[s].pos.lerp(l,h),this.freeNodes[s].vel.multiplyScalar(1-h)}}#v(t){const e=this.sheathPath?.length||0;if(!(this.pathSamples.length<3||t<=e+this.pathSpacing*2))for(let n=0;n<ZM;n++)this.#M(t,e),this.#C(t,e)}#M(t,e){const n=e+this.pathSpacing*1.5,i=1/Math.max(1e-8,this.pathSpacing*6.5),s=t-this.pathSpacing*4,a=1/Math.max(1e-8,this.pathSpacing*4);for(let o=1;o<this.pathSamples.length-1;o++){const c=this.pathSamples[o],l=Math.max(0,Math.min(1,(c.distance-n)*i)),h=Math.max(0,Math.min(1,(c.distance-s)*a)),u=l*l*(3-2*l),d=1-h*h*(3-2*h),f=u*(.35+d*.65);if(f<=.001)continue;const m=this.pathSamples[o-1].point._values,x=this.pathSamples[o+1].point._values;this.#_(c,(m[0]+x[0])*.5,(m[1]+x[1])*.5,(m[2]+x[2])*.5,jM*f)}for(let o=0;o<kd.length;o++){const c=kd[o];if(!(this.pathSamples.length<=c*2))for(let l=c;l<this.pathSamples.length-c;l++){const h=this.pathSamples[l],u=Math.max(0,Math.min(1,(h.distance-n)*i)),d=Math.max(0,Math.min(1,(h.distance-s)*a)),f=u*u*(3-2*u),m=1-d*d*(3-2*d),x=f*(.35+m*.65);if(x<=.001)continue;const g=this.pathSamples[l-c].point._values,p=this.pathSamples[l+c].point._values;this.#_(h,(g[0]+p[0])*.5,(g[1]+p[1])*.5,(g[2]+p[2])*.5,$M*x/Math.sqrt(c))}}}#C(t,e){const n=Math.cos(xl);for(let i=1;i<this.pathSamples.length-1;i++){const s=this.pathSamples[i],a=this.#y(s.distance,t,e);if(a<=.001)continue;const o=this.pathSamples[i-1].point,c=s.point,l=this.pathSamples[i+1].point,h=c.x-o.x,u=c.y-o.y,d=c.z-o.z,f=l.x-c.x,m=l.y-c.y,x=l.z-c.z,g=Sn(h,u,d),p=Sn(f,m,x);if(g<1e-5||p<1e-5)continue;const _=zt((h*f+u*m+d*x)/(g*p),-1,1);if(_>=n)continue;const v=zt((Math.acos(_)-xl)/(Math.PI-xl),0,1);this.#_(s,(o.x+l.x)*.5,(o.y+l.y)*.5,(o.z+l.z)*.5,KM*v*a)}}#y(t,e,n){if(t<=n+this.pathSpacing)return 0;const i=pn(n+this.pathSpacing*1.5,n+this.pathSpacing*8,t),s=1-pn(e-this.pathSpacing*4,e,t);return i*(.35+s*.65)}#_(t,e,n,i,s){const a=zt(s,0,1),o=t.point._values;let c=(e-o[0])*a,l=(n-o[1])*a,h=(i-o[2])*a;const u=Sn(c,l,h);if(!(u<=1e-6)){if(u>Hd){const d=Hd/u;c*=d,l*=d,h*=d}if(this.externalCollisionSolver){o[0]+=c,o[1]+=l,o[2]+=h;return}this._pathTarget.set(o[0]+c,o[1]+l,o[2]+h),t.point.copy(this.#D(this._pathTarget).point)}}#E(t,e,n,i=1,s=new Rt){if(this.type===ls)return this.#w(t,e,n,i,s);const a=Math.min(n,mo),o=Math.max(0,n-a);if(t<=o)return s.copy(e.supportTip).addScaledVector(e.tangent,t);const c=t-o,l=Math.min(a,_p),h=zt(i,0,1);if(c<=l||h<=.001)return s.copy(e.supportTip).addScaledVector(e.tangent,t);const u=Math.min(c-l,Oo),d=Kl/h,f=Math.min(Jl*Math.PI*2,u/d);return s.copy(e.supportTip).addScaledVector(e.tangent,o+l+Math.sin(f)*d).addScaledVector(e.normal,(Math.cos(f)-1)*d)}#w(t,e,n,i=1,s=new Rt){const a=Math.min(n,mo),o=Math.max(0,n-a);if(t<=o)return s.copy(e.supportTip).addScaledVector(e.tangent,t);const c=t-o,l=Math.min(a,Bd);if(c<=l)return s.copy(e.supportTip).addScaledVector(e.tangent,t);const h=Ud*zt(i,0,1);if(h<=.001)return s.copy(e.supportTip).addScaledVector(e.tangent,t);const u=this.#b(e,this._shapeNormal),d=Math.max(0,a-l),f=Math.min(po,Math.max(1e-4,d)),m=Math.min(c-l,f),x=h*zt(m/f,0,1),g=f/h;s.copy(e.supportTip).addScaledVector(e.tangent,o+l).addScaledVector(e.tangent,Math.sin(x)*g).addScaledVector(u,(1-Math.cos(x))*g);const p=c-l-f;return p>0&&s.addScaledVector(e.tangent,Math.cos(h)*p).addScaledVector(u,Math.sin(h)*p),s}#b(t,e=new Rt){return e.copy(t.normal),e.z*=.18,e.addScaledVector(t.tangent,-e.dot(t.tangent)),e.lengthSq()<1e-6?e.copy(t.normal):e.normalize()}#r(){for(let t=1;t<this.freeNodes.length;t++){const e=this.freeNodes[t-1],n=this.freeNodes[t],i=Math.max(.5,(n.distance??0)-(e.distance??0)),s=n.pos.x-e.pos.x,a=n.pos.y-e.pos.y,o=n.pos.z-e.pos.z,c=Sn(s,a,o);if(c<1e-5)continue;const l=(c-i)/c;if(t===1)n.pos.x-=s*l,n.pos.y-=a*l,n.pos.z-=o*l;else{const h=l*.5;e.pos.x+=s*h,e.pos.y+=a*h,e.pos.z+=o*h,n.pos.x-=s*h,n.pos.y-=a*h,n.pos.z-=o*h}}}#h(t){if(this.freeNodes.length>1){const e=Math.max(.5,(this.freeNodes[1].distance??0)-(this.freeNodes[0].distance??0))||us,n=this.freeNodes[1].pos;n.x+=(t.supportTip.x+t.tangent.x*e-n.x)*fl,n.y+=(t.supportTip.y+t.tangent.y*e-n.y)*fl,n.z+=(t.supportTip.z+t.tangent.z*e-n.z)*fl}for(let e=2;e<this.freeNodes.length-1;e++){const n=this.freeNodes[e-1].pos,i=this.freeNodes[e+1].pos,s=this.freeNodes[e].pos;s.x+=((n.x+i.x)*.5-s.x)*pl,s.y+=((n.y+i.y)*.5-s.y)*pl,s.z+=((n.z+i.z)*.5-s.z)*pl}}#N(t,e){for(let n=1;n<this.freeNodes.length;n++){const i=Math.max(0,(this.freeNodes[n].distance??0)-(this.freeNodes[0].distance??0)),s=this.#o(i,t,e,this.freeNodes[n].curl??1,this.freeNodes[n].shapeTarget),a=pn(0,Math.max(us,e),i),o=this.type===ls?this.#e(i,e):0,c=VM*(.45+a*.55)*(1+o*1.2),l=zt(c,0,.42),h=this.freeNodes[n].pos._values,u=s._values;h[0]+=(u[0]-h[0])*l,h[1]+=(u[1]-h[1])*l,h[2]+=(u[2]-h[2])*l}}#U(t){if(this.freeNodes.length<4)return;const e=this.freeNodes[0]?.distance??0,n=Math.max(0,t-Math.min(t,mo)),i=Math.max(0,n-10),s=Math.max(1e-8,n+8-i);for(let a=1;a<this.freeNodes.length-1;a++){const o=Math.max(0,(this.freeNodes[a].distance??e)-e),c=Math.max(0,Math.min(1,(o-i)/s)),l=1-c*c*(3-2*c);if(l<=.001)continue;const h=this.freeNodes[a-1].pos._values,u=this.freeNodes[a+1].pos._values,d=this.freeNodes[a].pos._values,f=this.#z(this.freeNodes[a].distance),m=zt(WM*l*f,0,1);d[0]+=((h[0]+u[0])*.5-d[0])*m,d[1]+=((h[1]+u[1])*.5-d[1])*m,d[2]+=((h[2]+u[2])*.5-d[2])*m}for(let a=0;a<Vd.length;a++){const o=Vd[a];if(!(this.freeNodes.length<=o*2))for(let c=o;c<this.freeNodes.length-o;c++){const l=Math.max(0,(this.freeNodes[c].distance??e)-e),h=Math.max(0,Math.min(1,(l-i)/s)),u=1-h*h*(3-2*h);if(u<=.001)continue;const d=this.freeNodes[c-o].pos._values,f=this.freeNodes[c+o].pos._values,m=this.#z(this.freeNodes[c].distance),x=XM*u*m/Math.sqrt(o),g=zt(x,0,1),p=this.freeNodes[c].pos._values;p[0]+=((d[0]+f[0])*.5-p[0])*g,p[1]+=((d[1]+f[1])*.5-p[1])*g,p[2]+=((d[2]+f[2])*.5-p[2])*g}}}#x(t){if(this.freeNodes.length<3)return;const e=Math.cos(gl);for(let n=1;n<this.freeNodes.length-1;n++){const i=this.freeNodes[n-1].pos,s=this.freeNodes[n].pos,a=this.freeNodes[n+1].pos,o=s.x-i.x,c=s.y-i.y,l=s.z-i.z,h=a.x-s.x,u=a.y-s.y,d=a.z-s.z,f=Sn(o,c,l),m=Sn(h,u,d);if(f<1e-5||m<1e-5)continue;const x=zt((o*h+c*u+l*d)/(f*m),-1,1);if(x>=e)continue;const g=zt((Math.acos(x)-gl)/(Math.PI-gl),0,1),p=this.#d(this.freeNodes[n],t),v=(this.#z(this.freeNodes[n].distance)-1)/Math.max(1e-6,_l-1),S=1+(JM-1)*v,M=YM*S*g*(.28+p*.72),y=zt(M,0,1);s.x+=((i.x+a.x)*.5-s.x)*y,s.y+=((i.y+a.y)*.5-s.y)*y,s.z+=((i.z+a.z)*.5-s.z)*y}}#A(){for(let t=1;t<this.freeNodes.length;t++){const e=this.freeNodes[t],n=this.#D(e.pos);if(!n.collided)continue;e.pos.copy(n.point);const i=n.normal,s=e.vel.dot(i);s>0&&e.vel.addScaledVector(i,-s),e.vel.multiplyScalar(1-ml)}}#T(){if(!(this.freeNodes.length<2))for(let t=1;t<this.freeNodes.length;t++){const e=this.freeNodes[t-1],n=this.freeNodes[t];for(const i of qM){const s=e.pos.clone().lerp(n.pos,i),a=this.#D(s);if(!a.collided)continue;const o=a.point.sub(s),c=o.length();if(c<=1e-6)continue;c>Gd&&o.multiplyScalar(Gd/c);const l=t===1?0:1-i,h=t===1?1:i;e.pos.addScaledVector(o,Od*l),n.pos.addScaledVector(o,Od*h),e.vel.multiplyScalar(1-ml*l),n.vel.multiplyScalar(1-ml*h)}}}#o(t,e,n,i=1,s=new Rt){const a=this.#O()+t;if(this.guidewireInserted>Wi){if(a<=this.guidewireInserted)return s.copy(this.#P(a,this._shapeNormal));const c=Math.max(0,this.progress-this.guidewireInserted),l=Math.min(c,a-this.guidewireInserted),h=this.#B(e);return this.#F(l,h,c,i,s)}const o=this.#E(t,e,n,i,s);return this.externalCollisionSolver?o:this.#D(o).point}#B(t){const e=this._guideReleaseFrameScratch;return this.#P(this.guidewireInserted,e.supportTip),this.#P(Math.max(this.#O(),this.guidewireInserted-10),e.beforeTip),e.tangent.subVectors(e.supportTip,e.beforeTip),e.tangent.lengthSq()<1e-6&&e.tangent.copy(t.tangent),e.tangent.normalize(),e.normal.copy(t.normal).addScaledVector(e.tangent,-t.normal.dot(e.tangent)),e.normal.lengthSq()<1e-6&&e.normal.copy(t.normal),e.normal.normalize(),e}#L(t){if(this.guidewireInserted<=Wi){this._guidewireRelease=1;return}const e=Math.max(0,this.progress-this.guidewireInserted),n=this.type===ls?Bd+po:Oo,i=pn(0,n,e),s=i>=this._guidewireRelease?ey:ny;this._guidewireRelease+=zt(i-this._guidewireRelease,-s*t,s*t)}#F(t,e,n,i,s){if(this.type===ls){const d=Math.min(n,po),f=Math.max(0,n-d);if(t<=f||d<=1e-4)return s.copy(e.supportTip).addScaledVector(e.tangent,t);const m=this._guidewireRelease*zt(i,0,1),x=Ud*m;if(x<=1e-4)return s.copy(e.supportTip).addScaledVector(e.tangent,t);const g=po/x,_=Math.min(t-f,d)/g,v=this.#b(e,this._shapeNormal);return s.copy(e.supportTip).addScaledVector(e.tangent,f+Math.sin(_)*g).addScaledVector(v,(1-Math.cos(_))*g)}const a=Math.min(n,Oo),o=Math.max(0,n-a);if(t<=o||a<=1e-4)return s.copy(e.supportTip).addScaledVector(e.tangent,t);const c=this._guidewireRelease*zt(i,0,1);if(c<=1e-4)return s.copy(e.supportTip).addScaledVector(e.tangent,t);const l=Kl/c,h=Math.min(t-o,a),u=Math.min(Jl*Math.PI*2,h/l);return s.copy(e.supportTip).addScaledVector(e.tangent,o+Math.sin(u)*l).addScaledVector(e.normal,(Math.cos(u)-1)*l)}#d(t,e){const n=this.freeNodes[0]?.distance??0,i=Math.max(0,(t.distance??n)-n),s=Math.max(0,e-this.#V(e));return 1-pn(Math.max(0,s-10),s+8,i)}#e(t,e){const n=Math.max(0,e-this.#V(e));return pn(n-2,n+10,t)}#V(t){const e=this.type===ls?OM:mo;return Math.min(t,e)}#z(t){if(this.guidewireInserted<=Wi)return _l;const e=pn(this.guidewireInserted-vl,this.guidewireInserted+hs,t??this.progress);return 1+(_l-1)*e}#D(t){if(this.collisionMesh)return this.#W(t,this.collisionMesh);let e=null;for(const n of this.vesselColliders){const i=n.type==="sphere"?this.#Y(t,n):this.#X(t,n);if(i.inside)return{point:t.clone(),normal:i.normal,collided:!1};(!e||i.distance<e.distance)&&(e=i)}return{point:e?.point||t.clone(),normal:e?.normal||new Rt(1,0,0),collided:!!e}}#W(t,e){if(e.meshCollider?.pointContact){const l=e.meshCollider.pointContact(t,e.clearance);return{point:l.violation?l.target.clone():t.clone(),normal:l.normal?.clone?.()||new Rt(1,0,0),collided:!!l.violation}}const n=new Rt,s=e.geometry.boundsTree.closestPointToPoint(t,{point:n})?.distance??t.distanceTo(n),a=Math.max(e.clearance+us*1.5,e.clearance*2);if(s>a)return{point:t.clone(),normal:new Rt(1,0,0),collided:!1};const o=typeof e.interiorDirection=="function"?e.interiorDirection(t,n).clone():t.clone().sub(n);return o.lengthSq()<1e-8&&o.set(1,0,0),o.normalize(),t.clone().sub(n).dot(o)>=e.clearance?{point:t.clone(),normal:o.clone().multiplyScalar(-1),collided:!1}:{point:n.clone().addScaledVector(o,e.clearance),normal:o.clone().multiplyScalar(-1),collided:!0}}#X(t,e){const n=new Rt().subVectors(t,e.start),i=zt(n.dot(e.dir),0,e.length),s=e.start.clone().addScaledVector(e.dir,i),a=new Rt().subVectors(t,s),o=a.length(),c=Math.max(.6,e.radius-Yd),l=o<=c,h=o>1e-6?a.multiplyScalar(1/o):this.#q(e.dir);if(l)return{inside:l,point:t.clone(),distance:0,normal:h};const u=s.addScaledVector(h,c);return{inside:!1,point:u,distance:t.distanceTo(u),normal:h}}#Y(t,e){const n=new Rt().subVectors(t,e.center),i=n.length(),s=Math.max(.6,e.radius-Yd),a=i<=s,o=i>1e-6?n.multiplyScalar(1/i):new Rt(1,0,0);if(a)return{inside:a,point:t.clone(),distance:0,normal:o};const c=e.center.clone().addScaledVector(o,s);return{inside:!1,point:c,distance:t.distanceTo(c),normal:o}}#q(t){const e=Math.abs(t.y)<.85?new Rt(0,1,0):new Rt(1,0,0);return new Rt().crossVectors(t,e).normalize()}#Z(t){if(!t?.segments)return[];const e=[],n=new Map,i=a=>`${a.x.toFixed(5)},${a.y.toFixed(5)},${a.z.toFixed(5)}`,s=(a,o)=>{const c=i(a),l=n.get(c);n.set(c,{point:a,radius:l?Math.max(l.radius,o):o})};for(const a of t.segments){const o=Jr(a.start),c=Jr(a.end),l=new Rt().subVectors(c,o),h=l.length();if(h<1e-6)continue;const u=l.multiplyScalar(1/h);e.push({type:"segment",start:o,end:c,dir:u,length:h,radius:a.radius||t.radius||10}),s(a.end,a.radius||t.radius||10),a.isSheath||s(a.start,a.radius||t.radius||10)}for(const{point:a,radius:o}of n.values())e.push({type:"sphere",center:Jr(a),radius:o});return e}#j(t){if(!t?.start||!t?.end)return null;const e=Jr(t.start),n=Jr(t.end),i=new Rt().subVectors(n,e),s=i.length();return s<1e-6?null:(i.multiplyScalar(1/s),{start:e,end:n,dir:i,length:s})}#O(){return this.sheathPath?Math.min(this.progress,this.sheathPath.length):0}#k(t,e=new Rt){if(!this.sheathPath)return null;const n=zt(t,0,this.sheathPath.length);return e.copy(this.sheathPath.start).addScaledVector(this.sheathPath.dir,n)}#H(t){const e=this.sheathPath?.length||0;if(t<=e+.5)return;this.pathSamples.length||this.#G(e);let n=this.#I();for(;n+this.pathSpacing<t;)n+=this.pathSpacing,this.#G(n);t>this.#I()+.5&&this.#G(t)}#G(t){const e=this.pathSamples.length;let n=this._pathSamplePool[e];return n||(n={distance:0,point:new Rt},this._pathSamplePool[e]=n),n.distance=t,this.#P(t,n.point),this.pathSamples[e]=n,n}#$(t){const e=this.sheathPath?.length||0;if(!(t<=e+.5)){this.#H(t);for(let n=0;n<this.pathSamples.length;n++){const i=this.pathSamples[n];i.distance<=e+.5||i.distance>t+.5||this.#P(i.distance,i.point)}}}#K(t){const e=this.sheathPath?.length||0,n=Math.max(t,e);for(;this.pathSamples.length>0&&this.pathSamples[this.pathSamples.length-1].distance>n;)this.pathSamples.pop();const i=this.pathSamples[this.pathSamples.length-1];i&&i.distance>t&&i.distance>e&&(i.distance=t)}#I(){const t=this.pathSamples[this.pathSamples.length-1];return Math.max(this.sheathPath?.length||0,t?t.distance:0)}#R(t,e=new Rt){const n=this.sheathPath?.length||0;if(this.sheathPath&&t<0)return e.copy(this.sheathPath.start).addScaledVector(this.sheathPath.dir,t);if(this.sheathPath&&t<=n+.5)return this.#k(t,e);if(!this.pathSamples.length){const a=this.#k(n,e);return a||this.#P(t,e)}const i=zt(t,0,this.#I());let s=this.pathSamples[0];for(let a=1;a<this.pathSamples.length;a++){const o=this.pathSamples[a];if(o.distance>=i){const c=zt((i-s.distance)/Math.max(1e-6,o.distance-s.distance),0,1);return e.copy(s.point).lerp(o.point,c)}s=o}return e.copy(s.point)}#J(t,e=this.#u()){if(!this.freeNodes.length||t<=e.supportEnd+.5)return this.#R(t);const n=zt(t,this.freeNodes[0].distance??e.supportEnd,this.progress);let i=this.freeNodes[0];for(let s=1;s<this.freeNodes.length;s++){const a=this.freeNodes[s];if((a.distance??n)>=n){const o=Math.max(1e-6,(a.distance??n)-(i.distance??n)),c=zt((n-(i.distance??n))/o,0,1);return i.pos.clone().lerp(a.pos,c)}i=a}return i.pos.clone()}#Q(t,e,n=1){if(n<=0||!this.freeNodes.length||e.lengthSq()<1e-8||t<=(this.freeNodes[0].distance??0))return;let i=this.freeNodes[0];for(let a=1;a<this.freeNodes.length;a++){const o=this.freeNodes[a],c=i.distance??0,l=o.distance??c;if(t<=l+.5){const h=Math.max(1e-6,l-c),u=zt((t-c)/h,0,1),d=e.clone().multiplyScalar(-.16*n),f=a===1?0:1-u,m=a===1?1:u;i.pos.addScaledVector(d,f),o.pos.addScaledVector(d,m),i.vel.addScaledVector(d,.18*f),o.vel.addScaledVector(d,.18*m);return}i=o}const s=this.freeNodes[this.freeNodes.length-1];s.pos.addScaledVector(e,-.16*n),s.vel.addScaledVector(e,-.18*n)}#tt(t,e,n,i=new Rt){const s=this._planePreviousTangent.subVectors(e,n);if(s.lengthSq()>1e-5){s.normalize();const c=this._planeCurvature.subVectors(t,s);if(c.addScaledVector(t,-c.dot(t)),c.lengthSq()>1e-5)return i.copy(c).normalize()}const a=Math.abs(t.y)<.85,o=this._planeHelper.set(a?0:1,a?1:0,0);return i.crossVectors(t,o).cross(t).normalize()}#P(t,e=new Rt){const n=this.tailProgressRef(),i=this.wire.nodes,s=zt((t+this.guidewireLength-n)/this.segmentLength,0,i.length-1),a=Math.min(i.length-2,Math.floor(s)),o=s-a,c=i[a],l=i[a+1];return e.set(c.x+(l.x-c.x)*o,c.y+(l.y-c.y)*o,c.z+(l.z-c.z)*o)}#et(t,e){return this.segmentLength*t-this.guidewireLength+e}#nt(t){return t===ls||t==="bernstein"?ls:Fd}}class hy extends uc{constructor(t){super(t)}load(t,e,n,i){const s=this,a=new $f(this.manager);a.setPath(this.path),a.setResponseType("arraybuffer"),a.setRequestHeader(this.requestHeader),a.setWithCredentials(this.withCredentials),a.load(t,function(o){try{e(s.parse(o))}catch(c){i?i(c):console.error(c),s.manager.itemError(t)}},n,i)}parse(t){function e(l){const h=new DataView(l),u=32/8*3+32/8*3*3+16/8,d=h.getUint32(80,!0);if(80+32/8+d*u===h.byteLength)return!0;const m=[115,111,108,105,100];for(let x=0;x<5;x++)if(n(m,h,x))return!1;return!0}function n(l,h,u){for(let d=0,f=l.length;d<f;d++)if(l[d]!==h.getUint8(u+d))return!1;return!0}function i(l){const h=new DataView(l),u=h.getUint32(80,!0);let d,f,m,x=!1,g,p,_,v,S;for(let D=0;D<70;D++)h.getUint32(D,!1)==1129270351&&h.getUint8(D+4)==82&&h.getUint8(D+5)==61&&(x=!0,g=new Float32Array(u*3*3),p=h.getUint8(D+6)/255,_=h.getUint8(D+7)/255,v=h.getUint8(D+8)/255,S=h.getUint8(D+9)/255);const M=84,y=50,w=new we,T=new Float32Array(u*3*3),E=new Float32Array(u*3*3),A=new Kt;for(let D=0;D<u;D++){const b=M+D*y,F=h.getFloat32(b,!0),L=h.getFloat32(b+4,!0),N=h.getFloat32(b+8,!0);if(x){const U=h.getUint16(b+48,!0);(U&32768)===0?(d=(U&31)/31,f=(U>>5&31)/31,m=(U>>10&31)/31):(d=p,f=_,m=v)}for(let U=1;U<=3;U++){const k=b+U*12,O=D*3*3+(U-1)*3;T[O]=h.getFloat32(k,!0),T[O+1]=h.getFloat32(k+4,!0),T[O+2]=h.getFloat32(k+8,!0),E[O]=F,E[O+1]=L,E[O+2]=N,x&&(A.set(d,f,m).convertSRGBToLinear(),g[O]=A.r,g[O+1]=A.g,g[O+2]=A.b)}}return w.setAttribute("position",new Fe(T,3)),w.setAttribute("normal",new Fe(E,3)),x&&(w.setAttribute("color",new Fe(g,3)),w.hasColors=!0,w.alpha=S),w}function s(l){const h=new we,u=/solid([\s\S]*?)endsolid/g,d=/facet([\s\S]*?)endfacet/g,f=/solid\s(.+)/;let m=0;const x=/[\s]+([+-]?(?:\d*)(?:\.\d*)?(?:[eE][+-]?\d+)?)/.source,g=new RegExp("vertex"+x+x+x,"g"),p=new RegExp("normal"+x+x+x,"g"),_=[],v=[],S=[],M=new C;let y,w=0,T=0,E=0;for(;(y=u.exec(l))!==null;){T=E;const A=y[0],D=(y=f.exec(A))!==null?y[1]:"";for(S.push(D);(y=d.exec(A))!==null;){let L=0,N=0;const U=y[0];for(;(y=p.exec(U))!==null;)M.x=parseFloat(y[1]),M.y=parseFloat(y[2]),M.z=parseFloat(y[3]),N++;for(;(y=g.exec(U))!==null;)_.push(parseFloat(y[1]),parseFloat(y[2]),parseFloat(y[3])),v.push(M.x,M.y,M.z),L++,E++;N!==1&&console.error("THREE.STLLoader: Something isn't right with the normal of face number "+m),L!==3&&console.error("THREE.STLLoader: Something isn't right with the vertices of face number "+m),m++}const b=T,F=E-T;h.userData.groupNames=S,h.addGroup(b,F,w),w++}return h.setAttribute("position",new se(_,3)),h.setAttribute("normal",new se(v,3)),h}function a(l){return typeof l!="string"?new TextDecoder().decode(l):l}function o(l){if(typeof l=="string"){const h=new Uint8Array(l.length);for(let u=0;u<l.length;u++)h[u]=l.charCodeAt(u)&255;return h.buffer||h}else return l}const c=o(t);return e(c)?i(c):s(a(t))}}const vp=0,uy=1,dy=2,qd=2,Ml=1.25,Zd=1,Ma=32,fc=65535,fy=Math.pow(2,-24),yl=Symbol("SKIP_GENERATION");function py(r){return r.index?r.index.count:r.attributes.position.count}function Ar(r){return py(r)/3}function my(r,t=ArrayBuffer){return r>65535?new Uint32Array(new t(4*r)):new Uint16Array(new t(2*r))}function gy(r,t){if(!r.index){const e=r.attributes.position.count,n=t.useSharedArrayBuffer?SharedArrayBuffer:ArrayBuffer,i=my(e,n);r.setIndex(new Fe(i,1));for(let s=0;s<e;s++)i[s]=s}}function Sp(r,t){const e=Ar(r),n=t||r.drawRange,i=n.start/3,s=(n.start+n.count)/3,a=Math.max(0,i),o=Math.min(e,s)-a;return[{offset:Math.floor(a),count:Math.floor(o)}]}function Mp(r,t){if(!r.groups||!r.groups.length)return Sp(r,t);const e=[],n=new Set,i=t||r.drawRange,s=i.start/3,a=(i.start+i.count)/3;for(const c of r.groups){const l=c.start/3,h=(c.start+c.count)/3;n.add(Math.max(s,l)),n.add(Math.min(a,h))}const o=Array.from(n.values()).sort((c,l)=>c-l);for(let c=0;c<o.length-1;c++){const l=o[c],h=o[c+1];e.push({offset:Math.floor(l),count:Math.floor(h-l)})}return e}function xy(r,t){const e=Ar(r),n=Mp(r,t).sort((a,o)=>a.offset-o.offset),i=n[n.length-1];i.count=Math.min(e-i.offset,i.count);let s=0;return n.forEach(({count:a})=>s+=a),e!==s}function El(r,t,e,n,i){let s=1/0,a=1/0,o=1/0,c=-1/0,l=-1/0,h=-1/0,u=1/0,d=1/0,f=1/0,m=-1/0,x=-1/0,g=-1/0;for(let p=t*6,_=(t+e)*6;p<_;p+=6){const v=r[p+0],S=r[p+1],M=v-S,y=v+S;M<s&&(s=M),y>c&&(c=y),v<u&&(u=v),v>m&&(m=v);const w=r[p+2],T=r[p+3],E=w-T,A=w+T;E<a&&(a=E),A>l&&(l=A),w<d&&(d=w),w>x&&(x=w);const D=r[p+4],b=r[p+5],F=D-b,L=D+b;F<o&&(o=F),L>h&&(h=L),D<f&&(f=D),D>g&&(g=D)}n[0]=s,n[1]=a,n[2]=o,n[3]=c,n[4]=l,n[5]=h,i[0]=u,i[1]=d,i[2]=f,i[3]=m,i[4]=x,i[5]=g}function _y(r,t=null,e=null,n=null){const i=r.attributes.position,s=r.index?r.index.array:null,a=Ar(r),o=i.normalized;let c;t===null?(c=new Float32Array(a*6),e=0,n=a):(c=t,e=e||0,n=n||a);const l=i.array,h=i.offset||0;let u=3;i.isInterleavedBufferAttribute&&(u=i.data.stride);const d=["getX","getY","getZ"];for(let f=e;f<e+n;f++){const m=f*3,x=f*6;let g=m+0,p=m+1,_=m+2;s&&(g=s[g],p=s[p],_=s[_]),o||(g=g*u+h,p=p*u+h,_=_*u+h);for(let v=0;v<3;v++){let S,M,y;o?(S=i[d[v]](g),M=i[d[v]](p),y=i[d[v]](_)):(S=l[g+v],M=l[p+v],y=l[_+v]);let w=S;M<w&&(w=M),y<w&&(w=y);let T=S;M>T&&(T=M),y>T&&(T=y);const E=(T-w)/2,A=v*2;c[x+A+0]=w+E,c[x+A+1]=E+(Math.abs(w)+E)*fy}}return c}function Ie(r,t,e){return e.min.x=t[r],e.min.y=t[r+1],e.min.z=t[r+2],e.max.x=t[r+3],e.max.y=t[r+4],e.max.z=t[r+5],e}function jd(r){let t=-1,e=-1/0;for(let n=0;n<3;n++){const i=r[n+3]-r[n];i>e&&(e=i,t=n)}return t}function $d(r,t){t.set(r)}function Kd(r,t,e){let n,i;for(let s=0;s<3;s++){const a=s+3;n=r[s],i=t[s],e[s]=n<i?n:i,n=r[a],i=t[a],e[a]=n>i?n:i}}function go(r,t,e){for(let n=0;n<3;n++){const i=t[r+2*n],s=t[r+2*n+1],a=i-s,o=i+s;a<e[n]&&(e[n]=a),o>e[n+3]&&(e[n+3]=o)}}function Qr(r){const t=r[3]-r[0],e=r[4]-r[1],n=r[5]-r[2];return 2*(t*e+e*n+n*t)}const bi=32,vy=(r,t)=>r.candidate-t.candidate,Xi=new Array(bi).fill().map(()=>({count:0,bounds:new Float32Array(6),rightCacheBounds:new Float32Array(6),leftCacheBounds:new Float32Array(6),candidate:0})),xo=new Float32Array(6);function Sy(r,t,e,n,i,s){let a=-1,o=0;if(s===vp)a=jd(t),a!==-1&&(o=(t[a]+t[a+3])/2);else if(s===uy)a=jd(r),a!==-1&&(o=My(e,n,i,a));else if(s===dy){const c=Qr(r);let l=Ml*i;const h=n*6,u=(n+i)*6;for(let d=0;d<3;d++){const f=t[d],g=(t[d+3]-f)/bi;if(i<bi/4){const p=[...Xi];p.length=i;let _=0;for(let S=h;S<u;S+=6,_++){const M=p[_];M.candidate=e[S+2*d],M.count=0;const{bounds:y,leftCacheBounds:w,rightCacheBounds:T}=M;for(let E=0;E<3;E++)T[E]=1/0,T[E+3]=-1/0,w[E]=1/0,w[E+3]=-1/0,y[E]=1/0,y[E+3]=-1/0;go(S,e,y)}p.sort(vy);let v=i;for(let S=0;S<v;S++){const M=p[S];for(;S+1<v&&p[S+1].candidate===M.candidate;)p.splice(S+1,1),v--}for(let S=h;S<u;S+=6){const M=e[S+2*d];for(let y=0;y<v;y++){const w=p[y];M>=w.candidate?go(S,e,w.rightCacheBounds):(go(S,e,w.leftCacheBounds),w.count++)}}for(let S=0;S<v;S++){const M=p[S],y=M.count,w=i-M.count,T=M.leftCacheBounds,E=M.rightCacheBounds;let A=0;y!==0&&(A=Qr(T)/c);let D=0;w!==0&&(D=Qr(E)/c);const b=Zd+Ml*(A*y+D*w);b<l&&(a=d,l=b,o=M.candidate)}}else{for(let v=0;v<bi;v++){const S=Xi[v];S.count=0,S.candidate=f+g+v*g;const M=S.bounds;for(let y=0;y<3;y++)M[y]=1/0,M[y+3]=-1/0}for(let v=h;v<u;v+=6){let y=~~((e[v+2*d]-f)/g);y>=bi&&(y=bi-1);const w=Xi[y];w.count++,go(v,e,w.bounds)}const p=Xi[bi-1];$d(p.bounds,p.rightCacheBounds);for(let v=bi-2;v>=0;v--){const S=Xi[v],M=Xi[v+1];Kd(S.bounds,M.rightCacheBounds,S.rightCacheBounds)}let _=0;for(let v=0;v<bi-1;v++){const S=Xi[v],M=S.count,y=S.bounds,T=Xi[v+1].rightCacheBounds;M!==0&&(_===0?$d(y,xo):Kd(y,xo,xo)),_+=M;let E=0,A=0;_!==0&&(E=Qr(xo)/c);const D=i-_;D!==0&&(A=Qr(T)/c);const b=Zd+Ml*(E*_+A*D);b<l&&(a=d,l=b,o=S.candidate)}}}}else console.warn(`MeshBVH: Invalid build strategy value ${s} used.`);return{axis:a,pos:o}}function My(r,t,e,n){let i=0;for(let s=t,a=t+e;s<a;s++)i+=r[s*6+n*2];return i/e}class wl{constructor(){this.boundingData=new Float32Array(6)}}function yy(r,t,e,n,i,s){let a=n,o=n+i-1;const c=s.pos,l=s.axis*2;for(;;){for(;a<=o&&e[a*6+l]<c;)a++;for(;a<=o&&e[o*6+l]>=c;)o--;if(a<o){for(let h=0;h<3;h++){let u=t[a*3+h];t[a*3+h]=t[o*3+h],t[o*3+h]=u}for(let h=0;h<6;h++){let u=e[a*6+h];e[a*6+h]=e[o*6+h],e[o*6+h]=u}a++,o--}else return a}}function Ey(r,t,e,n,i,s){let a=n,o=n+i-1;const c=s.pos,l=s.axis*2;for(;;){for(;a<=o&&e[a*6+l]<c;)a++;for(;a<=o&&e[o*6+l]>=c;)o--;if(a<o){let h=r[a];r[a]=r[o],r[o]=h;for(let u=0;u<6;u++){let d=e[a*6+u];e[a*6+u]=e[o*6+u],e[o*6+u]=d}a++,o--}else return a}}function yn(r,t){return t[r+15]===65535}function In(r,t){return t[r+6]}function Gn(r,t){return t[r+14]}function Vn(r){return r+8}function kn(r,t){return t[r+6]}function yp(r,t){return t[r+7]}let Ep,ua,Go,wp;const wy=Math.pow(2,32);function Ql(r){return"count"in r?1:1+Ql(r.left)+Ql(r.right)}function Ay(r,t,e){return Ep=new Float32Array(e),ua=new Uint32Array(e),Go=new Uint16Array(e),wp=new Uint8Array(e),th(r,t)}function th(r,t){const e=r/4,n=r/2,i="count"in t,s=t.boundingData;for(let a=0;a<6;a++)Ep[e+a]=s[a];if(i)if(t.buffer){const a=t.buffer;wp.set(new Uint8Array(a),r);for(let o=r,c=r+a.byteLength;o<c;o+=Ma){const l=o/2;yn(l,Go)||(ua[o/4+6]+=e)}return r+a.byteLength}else{const a=t.offset,o=t.count;return ua[e+6]=a,Go[n+14]=o,Go[n+15]=fc,r+Ma}else{const a=t.left,o=t.right,c=t.splitAxis;let l;if(l=th(r+Ma,a),l/4>wy)throw new Error("MeshBVH: Cannot store child pointer greater than 32 bits.");return ua[e+6]=l/4,l=th(l,o),ua[e+7]=c,l}}function Ty(r,t){const e=(r.index?r.index.count:r.attributes.position.count)/3,n=e>2**16,i=n?4:2,s=t?new SharedArrayBuffer(e*i):new ArrayBuffer(e*i),a=n?new Uint32Array(s):new Uint16Array(s);for(let o=0,c=a.length;o<c;o++)a[o]=o;return a}function Cy(r,t,e,n,i){const{maxDepth:s,verbose:a,maxLeafTris:o,strategy:c,onProgress:l,indirect:h}=i,u=r._indirectBuffer,d=r.geometry,f=d.index?d.index.array:null,m=h?Ey:yy,x=Ar(d),g=new Float32Array(6);let p=!1;const _=new wl;return El(t,e,n,_.boundingData,g),S(_,e,n,g),_;function v(M){l&&l(M/x)}function S(M,y,w,T=null,E=0){if(!p&&E>=s&&(p=!0,a&&(console.warn(`MeshBVH: Max depth of ${s} reached when generating BVH. Consider increasing maxDepth.`),console.warn(d))),w<=o||E>=s)return v(y+w),M.offset=y,M.count=w,M;const A=Sy(M.boundingData,T,t,y,w,c);if(A.axis===-1)return v(y+w),M.offset=y,M.count=w,M;const D=m(u,f,t,y,w,A);if(D===y||D===y+w)v(y+w),M.offset=y,M.count=w;else{M.splitAxis=A.axis;const b=new wl,F=y,L=D-y;M.left=b,El(t,F,L,b.boundingData,g),S(b,F,L,g,E+1);const N=new wl,U=D,k=w-L;M.right=N,El(t,U,k,N.boundingData,g),S(N,U,k,g,E+1)}return M}}function by(r,t){const e=r.geometry;t.indirect&&(r._indirectBuffer=Ty(e,t.useSharedArrayBuffer),xy(e,t.range)&&!t.verbose&&console.warn('MeshBVH: Provided geometry contains groups or a range that do not fully span the vertex contents while using the "indirect" option. BVH may incorrectly report intersections on unrendered portions of the geometry.')),r._indirectBuffer||gy(e,t);const n=t.useSharedArrayBuffer?SharedArrayBuffer:ArrayBuffer,i=_y(e),s=t.indirect?Sp(e,t.range):Mp(e,t.range);r._roots=s.map(a=>{const o=Cy(r,i,a.offset,a.count,t),c=Ql(o),l=new n(Ma*c);return Ay(0,o,l),l})}class Ni{constructor(){this.min=1/0,this.max=-1/0}setFromPointsField(t,e){let n=1/0,i=-1/0;for(let s=0,a=t.length;s<a;s++){const c=t[s][e];n=c<n?c:n,i=c>i?c:i}this.min=n,this.max=i}setFromPoints(t,e){let n=1/0,i=-1/0;for(let s=0,a=e.length;s<a;s++){const o=e[s],c=t.dot(o);n=c<n?c:n,i=c>i?c:i}this.min=n,this.max=i}isSeparated(t){return this.min>t.max||t.min>this.max}}Ni.prototype.setFromBox=(function(){const r=new C;return function(e,n){const i=n.min,s=n.max;let a=1/0,o=-1/0;for(let c=0;c<=1;c++)for(let l=0;l<=1;l++)for(let h=0;h<=1;h++){r.x=i.x*c+s.x*(1-c),r.y=i.y*l+s.y*(1-l),r.z=i.z*h+s.z*(1-h);const u=e.dot(r);a=Math.min(u,a),o=Math.max(u,o)}this.min=a,this.max=o}})();const Ry=(function(){const r=new C,t=new C,e=new C;return function(i,s,a){const o=i.start,c=r,l=s.start,h=t;e.subVectors(o,l),r.subVectors(i.end,i.start),t.subVectors(s.end,s.start);const u=e.dot(h),d=h.dot(c),f=h.dot(h),m=e.dot(c),g=c.dot(c)*f-d*d;let p,_;g!==0?p=(u*d-m*f)/g:p=0,_=(u+p*d)/f,a.x=p,a.y=_}})(),Uh=(function(){const r=new yt,t=new C,e=new C;return function(i,s,a,o){Ry(i,s,r);let c=r.x,l=r.y;if(c>=0&&c<=1&&l>=0&&l<=1){i.at(c,a),s.at(l,o);return}else if(c>=0&&c<=1){l<0?s.at(0,o):s.at(1,o),i.closestPointToPoint(o,!0,a);return}else if(l>=0&&l<=1){c<0?i.at(0,a):i.at(1,a),s.closestPointToPoint(a,!0,o);return}else{let h;c<0?h=i.start:h=i.end;let u;l<0?u=s.start:u=s.end;const d=t,f=e;if(i.closestPointToPoint(u,!0,t),s.closestPointToPoint(h,!0,e),d.distanceToSquared(u)<=f.distanceToSquared(h)){a.copy(d),o.copy(u);return}else{a.copy(h),o.copy(f);return}}}})(),Py=(function(){const r=new C,t=new C,e=new Ri,n=new Li;return function(s,a){const{radius:o,center:c}=s,{a:l,b:h,c:u}=a;if(n.start=l,n.end=h,n.closestPointToPoint(c,!0,r).distanceTo(c)<=o||(n.start=l,n.end=u,n.closestPointToPoint(c,!0,r).distanceTo(c)<=o)||(n.start=h,n.end=u,n.closestPointToPoint(c,!0,r).distanceTo(c)<=o))return!0;const x=a.getPlane(e);if(Math.abs(x.distanceToPoint(c))<=o){const p=x.projectPoint(c,t);if(a.containsPoint(p))return!0}return!1}})(),Ly=1e-15;function Al(r){return Math.abs(r)<Ly}class ei extends sn{constructor(...t){super(...t),this.isExtendedTriangle=!0,this.satAxes=new Array(4).fill().map(()=>new C),this.satBounds=new Array(4).fill().map(()=>new Ni),this.points=[this.a,this.b,this.c],this.sphere=new ns,this.plane=new Ri,this.needsUpdate=!0}intersectsSphere(t){return Py(t,this)}update(){const t=this.a,e=this.b,n=this.c,i=this.points,s=this.satAxes,a=this.satBounds,o=s[0],c=a[0];this.getNormal(o),c.setFromPoints(o,i);const l=s[1],h=a[1];l.subVectors(t,e),h.setFromPoints(l,i);const u=s[2],d=a[2];u.subVectors(e,n),d.setFromPoints(u,i);const f=s[3],m=a[3];f.subVectors(n,t),m.setFromPoints(f,i),this.sphere.setFromPoints(this.points),this.plane.setFromNormalAndCoplanarPoint(o,t),this.needsUpdate=!1}}ei.prototype.closestPointToSegment=(function(){const r=new C,t=new C,e=new Li;return function(i,s=null,a=null){const{start:o,end:c}=i,l=this.points;let h,u=1/0;for(let d=0;d<3;d++){const f=(d+1)%3;e.start.copy(l[d]),e.end.copy(l[f]),Uh(e,i,r,t),h=r.distanceToSquared(t),h<u&&(u=h,s&&s.copy(r),a&&a.copy(t))}return this.closestPointToPoint(o,r),h=o.distanceToSquared(r),h<u&&(u=h,s&&s.copy(r),a&&a.copy(o)),this.closestPointToPoint(c,r),h=c.distanceToSquared(r),h<u&&(u=h,s&&s.copy(r),a&&a.copy(c)),Math.sqrt(u)}})();ei.prototype.intersectsTriangle=(function(){const r=new ei,t=new Array(3),e=new Array(3),n=new Ni,i=new Ni,s=new C,a=new C,o=new C,c=new C,l=new C,h=new Li,u=new Li,d=new Li,f=new C;function m(x,g,p){const _=x.points;let v=0,S=-1;for(let M=0;M<3;M++){const{start:y,end:w}=h;y.copy(_[M]),w.copy(_[(M+1)%3]),h.delta(a);const T=Al(g.distanceToPoint(y));if(Al(g.normal.dot(a))&&T){p.copy(h),v=2;break}const E=g.intersectLine(h,f);if(!E&&T&&f.copy(y),(E||T)&&!Al(f.distanceTo(w))){if(v<=1)(v===1?p.start:p.end).copy(f),T&&(S=v);else if(v>=2){(S===1?p.start:p.end).copy(f),v=2;break}if(v++,v===2&&S===-1)break}}return v}return function(g,p=null,_=!1){this.needsUpdate&&this.update(),g.isExtendedTriangle?g.needsUpdate&&g.update():(r.copy(g),r.update(),g=r);const v=this.plane,S=g.plane;if(Math.abs(v.normal.dot(S.normal))>1-1e-10){const M=this.satBounds,y=this.satAxes;e[0]=g.a,e[1]=g.b,e[2]=g.c;for(let E=0;E<4;E++){const A=M[E],D=y[E];if(n.setFromPoints(D,e),A.isSeparated(n))return!1}const w=g.satBounds,T=g.satAxes;t[0]=this.a,t[1]=this.b,t[2]=this.c;for(let E=0;E<4;E++){const A=w[E],D=T[E];if(n.setFromPoints(D,t),A.isSeparated(n))return!1}for(let E=0;E<4;E++){const A=y[E];for(let D=0;D<4;D++){const b=T[D];if(s.crossVectors(A,b),n.setFromPoints(s,t),i.setFromPoints(s,e),n.isSeparated(i))return!1}}return p&&(_||console.warn("ExtendedTriangle.intersectsTriangle: Triangles are coplanar which does not support an output edge. Setting edge to 0, 0, 0."),p.start.set(0,0,0),p.end.set(0,0,0)),!0}else{const M=m(this,S,u);if(M===1&&g.containsPoint(u.end))return p&&(p.start.copy(u.end),p.end.copy(u.end)),!0;if(M!==2)return!1;const y=m(g,v,d);if(y===1&&this.containsPoint(d.end))return p&&(p.start.copy(d.end),p.end.copy(d.end)),!0;if(y!==2)return!1;if(u.delta(o),d.delta(c),o.dot(c)<0){let F=d.start;d.start=d.end,d.end=F}const w=u.start.dot(o),T=u.end.dot(o),E=d.start.dot(o),A=d.end.dot(o),D=T<E,b=w<A;return w!==A&&E!==T&&D===b?!1:(p&&(l.subVectors(u.start,d.start),l.dot(o)>0?p.start.copy(u.start):p.start.copy(d.start),l.subVectors(u.end,d.end),l.dot(o)<0?p.end.copy(u.end):p.end.copy(d.end)),!0)}}})();ei.prototype.distanceToPoint=(function(){const r=new C;return function(e){return this.closestPointToPoint(e,r),e.distanceTo(r)}})();ei.prototype.distanceToTriangle=(function(){const r=new C,t=new C,e=["a","b","c"],n=new Li,i=new Li;return function(a,o=null,c=null){const l=o||c?n:null;if(this.intersectsTriangle(a,l))return(o||c)&&(o&&l.getCenter(o),c&&l.getCenter(c)),0;let h=1/0;for(let u=0;u<3;u++){let d;const f=e[u],m=a[f];this.closestPointToPoint(m,r),d=m.distanceToSquared(r),d<h&&(h=d,o&&o.copy(r),c&&c.copy(m));const x=this[f];a.closestPointToPoint(x,r),d=x.distanceToSquared(r),d<h&&(h=d,o&&o.copy(x),c&&c.copy(r))}for(let u=0;u<3;u++){const d=e[u],f=e[(u+1)%3];n.set(this[d],this[f]);for(let m=0;m<3;m++){const x=e[m],g=e[(m+1)%3];i.set(a[x],a[g]),Uh(n,i,r,t);const p=r.distanceToSquared(t);p<h&&(h=p,o&&o.copy(r),c&&c.copy(t))}}return Math.sqrt(h)}})();class xn{constructor(t,e,n){this.isOrientedBox=!0,this.min=new C,this.max=new C,this.matrix=new re,this.invMatrix=new re,this.points=new Array(8).fill().map(()=>new C),this.satAxes=new Array(3).fill().map(()=>new C),this.satBounds=new Array(3).fill().map(()=>new Ni),this.alignedSatBounds=new Array(3).fill().map(()=>new Ni),this.needsUpdate=!1,t&&this.min.copy(t),e&&this.max.copy(e),n&&this.matrix.copy(n)}set(t,e,n){this.min.copy(t),this.max.copy(e),this.matrix.copy(n),this.needsUpdate=!0}copy(t){this.min.copy(t.min),this.max.copy(t.max),this.matrix.copy(t.matrix),this.needsUpdate=!0}}xn.prototype.update=(function(){return function(){const t=this.matrix,e=this.min,n=this.max,i=this.points;for(let l=0;l<=1;l++)for(let h=0;h<=1;h++)for(let u=0;u<=1;u++){const d=1*l|2*h|4*u,f=i[d];f.x=l?n.x:e.x,f.y=h?n.y:e.y,f.z=u?n.z:e.z,f.applyMatrix4(t)}const s=this.satBounds,a=this.satAxes,o=i[0];for(let l=0;l<3;l++){const h=a[l],u=s[l],d=1<<l,f=i[d];h.subVectors(o,f),u.setFromPoints(h,i)}const c=this.alignedSatBounds;c[0].setFromPointsField(i,"x"),c[1].setFromPointsField(i,"y"),c[2].setFromPointsField(i,"z"),this.invMatrix.copy(this.matrix).invert(),this.needsUpdate=!1}})();xn.prototype.intersectsBox=(function(){const r=new Ni;return function(e){this.needsUpdate&&this.update();const n=e.min,i=e.max,s=this.satBounds,a=this.satAxes,o=this.alignedSatBounds;if(r.min=n.x,r.max=i.x,o[0].isSeparated(r)||(r.min=n.y,r.max=i.y,o[1].isSeparated(r))||(r.min=n.z,r.max=i.z,o[2].isSeparated(r)))return!1;for(let c=0;c<3;c++){const l=a[c],h=s[c];if(r.setFromBox(l,e),h.isSeparated(r))return!1}return!0}})();xn.prototype.intersectsTriangle=(function(){const r=new ei,t=new Array(3),e=new Ni,n=new Ni,i=new C;return function(a){this.needsUpdate&&this.update(),a.isExtendedTriangle?a.needsUpdate&&a.update():(r.copy(a),r.update(),a=r);const o=this.satBounds,c=this.satAxes;t[0]=a.a,t[1]=a.b,t[2]=a.c;for(let d=0;d<3;d++){const f=o[d],m=c[d];if(e.setFromPoints(m,t),f.isSeparated(e))return!1}const l=a.satBounds,h=a.satAxes,u=this.points;for(let d=0;d<3;d++){const f=l[d],m=h[d];if(e.setFromPoints(m,u),f.isSeparated(e))return!1}for(let d=0;d<3;d++){const f=c[d];for(let m=0;m<4;m++){const x=h[m];if(i.crossVectors(f,x),e.setFromPoints(i,t),n.setFromPoints(i,u),e.isSeparated(n))return!1}}return!0}})();xn.prototype.closestPointToPoint=(function(){return function(t,e){return this.needsUpdate&&this.update(),e.copy(t).applyMatrix4(this.invMatrix).clamp(this.min,this.max).applyMatrix4(this.matrix),e}})();xn.prototype.distanceToPoint=(function(){const r=new C;return function(e){return this.closestPointToPoint(e,r),e.distanceTo(r)}})();xn.prototype.distanceToBox=(function(){const r=["x","y","z"],t=new Array(12).fill().map(()=>new Li),e=new Array(12).fill().map(()=>new Li),n=new C,i=new C;return function(a,o=0,c=null,l=null){if(this.needsUpdate&&this.update(),this.intersectsBox(a))return(c||l)&&(a.getCenter(i),this.closestPointToPoint(i,n),a.closestPointToPoint(n,i),c&&c.copy(n),l&&l.copy(i)),0;const h=o*o,u=a.min,d=a.max,f=this.points;let m=1/0;for(let g=0;g<8;g++){const p=f[g];i.copy(p).clamp(u,d);const _=p.distanceToSquared(i);if(_<m&&(m=_,c&&c.copy(p),l&&l.copy(i),_<h))return Math.sqrt(_)}let x=0;for(let g=0;g<3;g++)for(let p=0;p<=1;p++)for(let _=0;_<=1;_++){const v=(g+1)%3,S=(g+2)%3,M=p<<v|_<<S,y=1<<g|p<<v|_<<S,w=f[M],T=f[y];t[x].set(w,T);const A=r[g],D=r[v],b=r[S],F=e[x],L=F.start,N=F.end;L[A]=u[A],L[D]=p?u[D]:d[D],L[b]=_?u[b]:d[D],N[A]=d[A],N[D]=p?u[D]:d[D],N[b]=_?u[b]:d[D],x++}for(let g=0;g<=1;g++)for(let p=0;p<=1;p++)for(let _=0;_<=1;_++){i.x=g?d.x:u.x,i.y=p?d.y:u.y,i.z=_?d.z:u.z,this.closestPointToPoint(i,n);const v=i.distanceToSquared(n);if(v<m&&(m=v,c&&c.copy(n),l&&l.copy(i),v<h))return Math.sqrt(v)}for(let g=0;g<12;g++){const p=t[g];for(let _=0;_<12;_++){const v=e[_];Uh(p,v,n,i);const S=n.distanceToSquared(i);if(S<m&&(m=S,c&&c.copy(n),l&&l.copy(i),S<h))return Math.sqrt(S)}}return Math.sqrt(m)}})();class Bh{constructor(t){this._getNewPrimitive=t,this._primitives=[]}getPrimitive(){const t=this._primitives;return t.length===0?this._getNewPrimitive():t.pop()}releasePrimitive(t){this._primitives.push(t)}}class Dy extends Bh{constructor(){super(()=>new ei)}}const Hn=new Dy;class Iy{constructor(){this.float32Array=null,this.uint16Array=null,this.uint32Array=null;const t=[];let e=null;this.setBuffer=n=>{e&&t.push(e),e=n,this.float32Array=new Float32Array(n),this.uint16Array=new Uint16Array(n),this.uint32Array=new Uint32Array(n)},this.clearBuffer=()=>{e=null,this.float32Array=null,this.uint16Array=null,this.uint32Array=null,t.length!==0&&this.setBuffer(t.pop())}}}const Ee=new Iy;let $i,lr;const qs=[],_o=new Bh(()=>new tn);function Ny(r,t,e,n,i,s){$i=_o.getPrimitive(),lr=_o.getPrimitive(),qs.push($i,lr),Ee.setBuffer(r._roots[t]);const a=eh(0,r.geometry,e,n,i,s);Ee.clearBuffer(),_o.releasePrimitive($i),_o.releasePrimitive(lr),qs.pop(),qs.pop();const o=qs.length;return o>0&&(lr=qs[o-1],$i=qs[o-2]),a}function eh(r,t,e,n,i=null,s=0,a=0){const{float32Array:o,uint16Array:c,uint32Array:l}=Ee;let h=r*2;if(yn(h,c)){const d=In(r,l),f=Gn(h,c);return Ie(r,o,$i),n(d,f,!1,a,s+r,$i)}else{let A=function(b){const{uint16Array:F,uint32Array:L}=Ee;let N=b*2;for(;!yn(N,F);)b=Vn(b),N=b*2;return In(b,L)},D=function(b){const{uint16Array:F,uint32Array:L}=Ee;let N=b*2;for(;!yn(N,F);)b=kn(b,L),N=b*2;return In(b,L)+Gn(N,F)};const d=Vn(r),f=kn(r,l);let m=d,x=f,g,p,_,v;if(i&&(_=$i,v=lr,Ie(m,o,_),Ie(x,o,v),g=i(_),p=i(v),p<g)){m=f,x=d;const b=g;g=p,p=b,_=v}_||(_=$i,Ie(m,o,_));const S=yn(m*2,c),M=e(_,S,g,a+1,s+m);let y;if(M===qd){const b=A(m),L=D(m)-b;y=n(b,L,!0,a+1,s+m,_)}else y=M&&eh(m,t,e,n,i,s,a+1);if(y)return!0;v=lr,Ie(x,o,v);const w=yn(x*2,c),T=e(v,w,p,a+1,s+x);let E;if(T===qd){const b=A(x),L=D(x)-b;E=n(b,L,!0,a+1,s+x,v)}else E=T&&eh(x,t,e,n,i,s,a+1);return!!E}}const ta=new C,Tl=new C;function Fy(r,t,e={},n=0,i=1/0){const s=n*n,a=i*i;let o=1/0,c=null;if(r.shapecast({boundsTraverseOrder:h=>(ta.copy(t).clamp(h.min,h.max),ta.distanceToSquared(t)),intersectsBounds:(h,u,d)=>d<o&&d<a,intersectsTriangle:(h,u)=>{h.closestPointToPoint(t,ta);const d=t.distanceToSquared(ta);return d<o&&(Tl.copy(ta),o=d,c=u),d<s}}),o===1/0)return null;const l=Math.sqrt(o);return e.point?e.point.copy(Tl):e.point=Tl.clone(),e.distance=l,e.faceIndex=c,e}const Uy=parseInt("160")>=169,ds=new C,fs=new C,ps=new C,vo=new yt,So=new yt,Mo=new yt,Jd=new C,Qd=new C,tf=new C,ea=new C;function By(r,t,e,n,i,s,a,o){let c;if(s===1?c=r.intersectTriangle(n,e,t,!0,i):c=r.intersectTriangle(t,e,n,s!==2,i),c===null)return null;const l=r.origin.distanceTo(i);return l<a||l>o?null:{distance:l,point:i.clone()}}function zy(r,t,e,n,i,s,a,o,c,l,h){ds.fromBufferAttribute(t,s),fs.fromBufferAttribute(t,a),ps.fromBufferAttribute(t,o);const u=By(r,ds,fs,ps,ea,c,l,h);if(u){const d=new C;sn.getBarycoord(ea,ds,fs,ps,d),n&&(vo.fromBufferAttribute(n,s),So.fromBufferAttribute(n,a),Mo.fromBufferAttribute(n,o),u.uv=sn.getInterpolation(ea,ds,fs,ps,vo,So,Mo,new yt)),i&&(vo.fromBufferAttribute(i,s),So.fromBufferAttribute(i,a),Mo.fromBufferAttribute(i,o),u.uv1=sn.getInterpolation(ea,ds,fs,ps,vo,So,Mo,new yt)),e&&(Jd.fromBufferAttribute(e,s),Qd.fromBufferAttribute(e,a),tf.fromBufferAttribute(e,o),u.normal=sn.getInterpolation(ea,ds,fs,ps,Jd,Qd,tf,new C),u.normal.dot(r.direction)>0&&u.normal.multiplyScalar(-1));const f={a:s,b:a,c:o,normal:new C,materialIndex:0};sn.getNormal(ds,fs,ps,f.normal),u.face=f,u.faceIndex=s,Uy&&(u.barycoord=d)}return u}function pc(r,t,e,n,i,s,a){const o=n*3;let c=o+0,l=o+1,h=o+2;const u=r.index;r.index&&(c=u.getX(c),l=u.getX(l),h=u.getX(h));const{position:d,normal:f,uv:m,uv1:x}=r.attributes,g=zy(e,d,f,m,x,c,l,h,t,s,a);return g?(g.faceIndex=n,i&&i.push(g),g):null}function He(r,t,e,n){const i=r.a,s=r.b,a=r.c;let o=t,c=t+1,l=t+2;e&&(o=e.getX(o),c=e.getX(c),l=e.getX(l)),i.x=n.getX(o),i.y=n.getY(o),i.z=n.getZ(o),s.x=n.getX(c),s.y=n.getY(c),s.z=n.getZ(c),a.x=n.getX(l),a.y=n.getY(l),a.z=n.getZ(l)}function Oy(r,t,e,n,i,s,a,o){const{geometry:c,_indirectBuffer:l}=r;for(let h=n,u=n+i;h<u;h++)pc(c,t,e,h,s,a,o)}function Gy(r,t,e,n,i,s,a){const{geometry:o,_indirectBuffer:c}=r;let l=1/0,h=null;for(let u=n,d=n+i;u<d;u++){let f;f=pc(o,t,e,u,null,s,a),f&&f.distance<l&&(h=f,l=f.distance)}return h}function Vy(r,t,e,n,i,s,a){const{geometry:o}=e,{index:c}=o,l=o.attributes.position;for(let h=r,u=t+r;h<u;h++){let d;if(d=h,He(a,d*3,c,l),a.needsUpdate=!0,n(a,d,i,s))return!0}return!1}function ky(r,t=null){t&&Array.isArray(t)&&(t=new Set(t));const e=r.geometry,n=e.index?e.index.array:null,i=e.attributes.position;let s,a,o,c,l=0;const h=r._roots;for(let d=0,f=h.length;d<f;d++)s=h[d],a=new Uint32Array(s),o=new Uint16Array(s),c=new Float32Array(s),u(0,l),l+=s.byteLength;function u(d,f,m=!1){const x=d*2;if(o[x+15]===fc){const p=a[d+6],_=o[x+14];let v=1/0,S=1/0,M=1/0,y=-1/0,w=-1/0,T=-1/0;for(let E=3*p,A=3*(p+_);E<A;E++){let D=n[E];const b=i.getX(D),F=i.getY(D),L=i.getZ(D);b<v&&(v=b),b>y&&(y=b),F<S&&(S=F),F>w&&(w=F),L<M&&(M=L),L>T&&(T=L)}return c[d+0]!==v||c[d+1]!==S||c[d+2]!==M||c[d+3]!==y||c[d+4]!==w||c[d+5]!==T?(c[d+0]=v,c[d+1]=S,c[d+2]=M,c[d+3]=y,c[d+4]=w,c[d+5]=T,!0):!1}else{const p=d+8,_=a[d+6],v=p+f,S=_+f;let M=m,y=!1,w=!1;t?M||(y=t.has(v),w=t.has(S),M=!y&&!w):(y=!0,w=!0);const T=M||y,E=M||w;let A=!1;T&&(A=u(p,f,M));let D=!1;E&&(D=u(_,f,M));const b=A||D;if(b)for(let F=0;F<3;F++){const L=p+F,N=_+F,U=c[L],k=c[L+3],O=c[N],H=c[N+3];c[d+F]=U<O?U:O,c[d+F+3]=k>H?k:H}return b}}}function es(r,t,e,n,i){let s,a,o,c,l,h;const u=1/e.direction.x,d=1/e.direction.y,f=1/e.direction.z,m=e.origin.x,x=e.origin.y,g=e.origin.z;let p=t[r],_=t[r+3],v=t[r+1],S=t[r+3+1],M=t[r+2],y=t[r+3+2];return u>=0?(s=(p-m)*u,a=(_-m)*u):(s=(_-m)*u,a=(p-m)*u),d>=0?(o=(v-x)*d,c=(S-x)*d):(o=(S-x)*d,c=(v-x)*d),s>c||o>a||((o>s||isNaN(s))&&(s=o),(c<a||isNaN(a))&&(a=c),f>=0?(l=(M-g)*f,h=(y-g)*f):(l=(y-g)*f,h=(M-g)*f),s>h||l>a)?!1:((l>s||s!==s)&&(s=l),(h<a||a!==a)&&(a=h),s<=i&&a>=n)}function Hy(r,t,e,n,i,s,a,o){const{geometry:c,_indirectBuffer:l}=r;for(let h=n,u=n+i;h<u;h++){let d=l?l[h]:h;pc(c,t,e,d,s,a,o)}}function Wy(r,t,e,n,i,s,a){const{geometry:o,_indirectBuffer:c}=r;let l=1/0,h=null;for(let u=n,d=n+i;u<d;u++){let f;f=pc(o,t,e,c?c[u]:u,null,s,a),f&&f.distance<l&&(h=f,l=f.distance)}return h}function Xy(r,t,e,n,i,s,a){const{geometry:o}=e,{index:c}=o,l=o.attributes.position;for(let h=r,u=t+r;h<u;h++){let d;if(d=e.resolveTriangleIndex(h),He(a,d*3,c,l),a.needsUpdate=!0,n(a,d,i,s))return!0}return!1}function Yy(r,t,e,n,i,s,a){Ee.setBuffer(r._roots[t]),nh(0,r,e,n,i,s,a),Ee.clearBuffer()}function nh(r,t,e,n,i,s,a){const{float32Array:o,uint16Array:c,uint32Array:l}=Ee,h=r*2;if(yn(h,c)){const d=In(r,l),f=Gn(h,c);Oy(t,e,n,d,f,i,s,a)}else{const d=Vn(r);es(d,o,n,s,a)&&nh(d,t,e,n,i,s,a);const f=kn(r,l);es(f,o,n,s,a)&&nh(f,t,e,n,i,s,a)}}const qy=["x","y","z"];function Zy(r,t,e,n,i,s){Ee.setBuffer(r._roots[t]);const a=ih(0,r,e,n,i,s);return Ee.clearBuffer(),a}function ih(r,t,e,n,i,s){const{float32Array:a,uint16Array:o,uint32Array:c}=Ee;let l=r*2;if(yn(l,o)){const u=In(r,c),d=Gn(l,o);return Gy(t,e,n,u,d,i,s)}else{const u=yp(r,c),d=qy[u],m=n.direction[d]>=0;let x,g;m?(x=Vn(r),g=kn(r,c)):(x=kn(r,c),g=Vn(r));const _=es(x,a,n,i,s)?ih(x,t,e,n,i,s):null;if(_){const M=_.point[d];if(m?M<=a[g+u]:M>=a[g+u+3])return _}const S=es(g,a,n,i,s)?ih(g,t,e,n,i,s):null;return _&&S?_.distance<=S.distance?_:S:_||S||null}}const yo=new tn,Zs=new ei,js=new ei,na=new re,ef=new xn,Eo=new xn;function jy(r,t,e,n){Ee.setBuffer(r._roots[t]);const i=sh(0,r,e,n);return Ee.clearBuffer(),i}function sh(r,t,e,n,i=null){const{float32Array:s,uint16Array:a,uint32Array:o}=Ee;let c=r*2;if(i===null&&(e.boundingBox||e.computeBoundingBox(),ef.set(e.boundingBox.min,e.boundingBox.max,n),i=ef),yn(c,a)){const h=t.geometry,u=h.index,d=h.attributes.position,f=e.index,m=e.attributes.position,x=In(r,o),g=Gn(c,a);if(na.copy(n).invert(),e.boundsTree)return Ie(r,s,Eo),Eo.matrix.copy(na),Eo.needsUpdate=!0,e.boundsTree.shapecast({intersectsBounds:_=>Eo.intersectsBox(_),intersectsTriangle:_=>{_.a.applyMatrix4(n),_.b.applyMatrix4(n),_.c.applyMatrix4(n),_.needsUpdate=!0;for(let v=x*3,S=(g+x)*3;v<S;v+=3)if(He(js,v,u,d),js.needsUpdate=!0,_.intersectsTriangle(js))return!0;return!1}});for(let p=x*3,_=(g+x)*3;p<_;p+=3){He(Zs,p,u,d),Zs.a.applyMatrix4(na),Zs.b.applyMatrix4(na),Zs.c.applyMatrix4(na),Zs.needsUpdate=!0;for(let v=0,S=f.count;v<S;v+=3)if(He(js,v,f,m),js.needsUpdate=!0,Zs.intersectsTriangle(js))return!0}}else{const h=r+8,u=o[r+6];return Ie(h,s,yo),!!(i.intersectsBox(yo)&&sh(h,t,e,n,i)||(Ie(u,s,yo),i.intersectsBox(yo)&&sh(u,t,e,n,i)))}}const wo=new re,Cl=new xn,ia=new xn,$y=new C,Ky=new C,Jy=new C,Qy=new C;function tE(r,t,e,n={},i={},s=0,a=1/0){t.boundingBox||t.computeBoundingBox(),Cl.set(t.boundingBox.min,t.boundingBox.max,e),Cl.needsUpdate=!0;const o=r.geometry,c=o.attributes.position,l=o.index,h=t.attributes.position,u=t.index,d=Hn.getPrimitive(),f=Hn.getPrimitive();let m=$y,x=Ky,g=null,p=null;i&&(g=Jy,p=Qy);let _=1/0,v=null,S=null;return wo.copy(e).invert(),ia.matrix.copy(wo),r.shapecast({boundsTraverseOrder:M=>Cl.distanceToBox(M),intersectsBounds:(M,y,w)=>w<_&&w<a?(y&&(ia.min.copy(M.min),ia.max.copy(M.max),ia.needsUpdate=!0),!0):!1,intersectsRange:(M,y)=>{if(t.boundsTree)return t.boundsTree.shapecast({boundsTraverseOrder:T=>ia.distanceToBox(T),intersectsBounds:(T,E,A)=>A<_&&A<a,intersectsRange:(T,E)=>{for(let A=T,D=T+E;A<D;A++){He(f,3*A,u,h),f.a.applyMatrix4(e),f.b.applyMatrix4(e),f.c.applyMatrix4(e),f.needsUpdate=!0;for(let b=M,F=M+y;b<F;b++){He(d,3*b,l,c),d.needsUpdate=!0;const L=d.distanceToTriangle(f,m,g);if(L<_&&(x.copy(m),p&&p.copy(g),_=L,v=b,S=A),L<s)return!0}}}});{const w=Ar(t);for(let T=0,E=w;T<E;T++){He(f,3*T,u,h),f.a.applyMatrix4(e),f.b.applyMatrix4(e),f.c.applyMatrix4(e),f.needsUpdate=!0;for(let A=M,D=M+y;A<D;A++){He(d,3*A,l,c),d.needsUpdate=!0;const b=d.distanceToTriangle(f,m,g);if(b<_&&(x.copy(m),p&&p.copy(g),_=b,v=A,S=T),b<s)return!0}}}}}),Hn.releasePrimitive(d),Hn.releasePrimitive(f),_===1/0?null:(n.point?n.point.copy(x):n.point=x.clone(),n.distance=_,n.faceIndex=v,i&&(i.point?i.point.copy(p):i.point=p.clone(),i.point.applyMatrix4(wo),x.applyMatrix4(wo),i.distance=x.sub(i.point).length(),i.faceIndex=S),n)}function eE(r,t=null){t&&Array.isArray(t)&&(t=new Set(t));const e=r.geometry,n=e.index?e.index.array:null,i=e.attributes.position;let s,a,o,c,l=0;const h=r._roots;for(let d=0,f=h.length;d<f;d++)s=h[d],a=new Uint32Array(s),o=new Uint16Array(s),c=new Float32Array(s),u(0,l),l+=s.byteLength;function u(d,f,m=!1){const x=d*2;if(o[x+15]===fc){const p=a[d+6],_=o[x+14];let v=1/0,S=1/0,M=1/0,y=-1/0,w=-1/0,T=-1/0;for(let E=p,A=p+_;E<A;E++){const D=3*r.resolveTriangleIndex(E);for(let b=0;b<3;b++){let F=D+b;F=n?n[F]:F;const L=i.getX(F),N=i.getY(F),U=i.getZ(F);L<v&&(v=L),L>y&&(y=L),N<S&&(S=N),N>w&&(w=N),U<M&&(M=U),U>T&&(T=U)}}return c[d+0]!==v||c[d+1]!==S||c[d+2]!==M||c[d+3]!==y||c[d+4]!==w||c[d+5]!==T?(c[d+0]=v,c[d+1]=S,c[d+2]=M,c[d+3]=y,c[d+4]=w,c[d+5]=T,!0):!1}else{const p=d+8,_=a[d+6],v=p+f,S=_+f;let M=m,y=!1,w=!1;t?M||(y=t.has(v),w=t.has(S),M=!y&&!w):(y=!0,w=!0);const T=M||y,E=M||w;let A=!1;T&&(A=u(p,f,M));let D=!1;E&&(D=u(_,f,M));const b=A||D;if(b)for(let F=0;F<3;F++){const L=p+F,N=_+F,U=c[L],k=c[L+3],O=c[N],H=c[N+3];c[d+F]=U<O?U:O,c[d+F+3]=k>H?k:H}return b}}}function nE(r,t,e,n,i,s,a){Ee.setBuffer(r._roots[t]),rh(0,r,e,n,i,s,a),Ee.clearBuffer()}function rh(r,t,e,n,i,s,a){const{float32Array:o,uint16Array:c,uint32Array:l}=Ee,h=r*2;if(yn(h,c)){const d=In(r,l),f=Gn(h,c);Hy(t,e,n,d,f,i,s,a)}else{const d=Vn(r);es(d,o,n,s,a)&&rh(d,t,e,n,i,s,a);const f=kn(r,l);es(f,o,n,s,a)&&rh(f,t,e,n,i,s,a)}}const iE=["x","y","z"];function sE(r,t,e,n,i,s){Ee.setBuffer(r._roots[t]);const a=ah(0,r,e,n,i,s);return Ee.clearBuffer(),a}function ah(r,t,e,n,i,s){const{float32Array:a,uint16Array:o,uint32Array:c}=Ee;let l=r*2;if(yn(l,o)){const u=In(r,c),d=Gn(l,o);return Wy(t,e,n,u,d,i,s)}else{const u=yp(r,c),d=iE[u],m=n.direction[d]>=0;let x,g;m?(x=Vn(r),g=kn(r,c)):(x=kn(r,c),g=Vn(r));const _=es(x,a,n,i,s)?ah(x,t,e,n,i,s):null;if(_){const M=_.point[d];if(m?M<=a[g+u]:M>=a[g+u+3])return _}const S=es(g,a,n,i,s)?ah(g,t,e,n,i,s):null;return _&&S?_.distance<=S.distance?_:S:_||S||null}}const Ao=new tn,$s=new ei,Ks=new ei,sa=new re,nf=new xn,To=new xn;function rE(r,t,e,n){Ee.setBuffer(r._roots[t]);const i=oh(0,r,e,n);return Ee.clearBuffer(),i}function oh(r,t,e,n,i=null){const{float32Array:s,uint16Array:a,uint32Array:o}=Ee;let c=r*2;if(i===null&&(e.boundingBox||e.computeBoundingBox(),nf.set(e.boundingBox.min,e.boundingBox.max,n),i=nf),yn(c,a)){const h=t.geometry,u=h.index,d=h.attributes.position,f=e.index,m=e.attributes.position,x=In(r,o),g=Gn(c,a);if(sa.copy(n).invert(),e.boundsTree)return Ie(r,s,To),To.matrix.copy(sa),To.needsUpdate=!0,e.boundsTree.shapecast({intersectsBounds:_=>To.intersectsBox(_),intersectsTriangle:_=>{_.a.applyMatrix4(n),_.b.applyMatrix4(n),_.c.applyMatrix4(n),_.needsUpdate=!0;for(let v=x,S=g+x;v<S;v++)if(He(Ks,3*t.resolveTriangleIndex(v),u,d),Ks.needsUpdate=!0,_.intersectsTriangle(Ks))return!0;return!1}});for(let p=x,_=g+x;p<_;p++){const v=t.resolveTriangleIndex(p);He($s,3*v,u,d),$s.a.applyMatrix4(sa),$s.b.applyMatrix4(sa),$s.c.applyMatrix4(sa),$s.needsUpdate=!0;for(let S=0,M=f.count;S<M;S+=3)if(He(Ks,S,f,m),Ks.needsUpdate=!0,$s.intersectsTriangle(Ks))return!0}}else{const h=r+8,u=o[r+6];return Ie(h,s,Ao),!!(i.intersectsBox(Ao)&&oh(h,t,e,n,i)||(Ie(u,s,Ao),i.intersectsBox(Ao)&&oh(u,t,e,n,i)))}}const Co=new re,bl=new xn,ra=new xn,aE=new C,oE=new C,cE=new C,lE=new C;function hE(r,t,e,n={},i={},s=0,a=1/0){t.boundingBox||t.computeBoundingBox(),bl.set(t.boundingBox.min,t.boundingBox.max,e),bl.needsUpdate=!0;const o=r.geometry,c=o.attributes.position,l=o.index,h=t.attributes.position,u=t.index,d=Hn.getPrimitive(),f=Hn.getPrimitive();let m=aE,x=oE,g=null,p=null;i&&(g=cE,p=lE);let _=1/0,v=null,S=null;return Co.copy(e).invert(),ra.matrix.copy(Co),r.shapecast({boundsTraverseOrder:M=>bl.distanceToBox(M),intersectsBounds:(M,y,w)=>w<_&&w<a?(y&&(ra.min.copy(M.min),ra.max.copy(M.max),ra.needsUpdate=!0),!0):!1,intersectsRange:(M,y)=>{if(t.boundsTree){const w=t.boundsTree;return w.shapecast({boundsTraverseOrder:T=>ra.distanceToBox(T),intersectsBounds:(T,E,A)=>A<_&&A<a,intersectsRange:(T,E)=>{for(let A=T,D=T+E;A<D;A++){const b=w.resolveTriangleIndex(A);He(f,3*b,u,h),f.a.applyMatrix4(e),f.b.applyMatrix4(e),f.c.applyMatrix4(e),f.needsUpdate=!0;for(let F=M,L=M+y;F<L;F++){const N=r.resolveTriangleIndex(F);He(d,3*N,l,c),d.needsUpdate=!0;const U=d.distanceToTriangle(f,m,g);if(U<_&&(x.copy(m),p&&p.copy(g),_=U,v=F,S=A),U<s)return!0}}}})}else{const w=Ar(t);for(let T=0,E=w;T<E;T++){He(f,3*T,u,h),f.a.applyMatrix4(e),f.b.applyMatrix4(e),f.c.applyMatrix4(e),f.needsUpdate=!0;for(let A=M,D=M+y;A<D;A++){const b=r.resolveTriangleIndex(A);He(d,3*b,l,c),d.needsUpdate=!0;const F=d.distanceToTriangle(f,m,g);if(F<_&&(x.copy(m),p&&p.copy(g),_=F,v=A,S=T),F<s)return!0}}}}}),Hn.releasePrimitive(d),Hn.releasePrimitive(f),_===1/0?null:(n.point?n.point.copy(x):n.point=x.clone(),n.distance=_,n.faceIndex=v,i&&(i.point?i.point.copy(p):i.point=p.clone(),i.point.applyMatrix4(Co),x.applyMatrix4(Co),i.distance=x.sub(i.point).length(),i.faceIndex=S),n)}function uE(){return typeof SharedArrayBuffer<"u"}const ya=new Ee.constructor,jo=new Ee.constructor,qi=new Bh(()=>new tn),Js=new tn,Qs=new tn,Rl=new tn,Pl=new tn;let Ll=!1;function dE(r,t,e,n){if(Ll)throw new Error("MeshBVH: Recursive calls to bvhcast not supported.");Ll=!0;const i=r._roots,s=t._roots;let a,o=0,c=0;const l=new re().copy(e).invert();for(let h=0,u=i.length;h<u;h++){ya.setBuffer(i[h]),c=0;const d=qi.getPrimitive();Ie(0,ya.float32Array,d),d.applyMatrix4(l);for(let f=0,m=s.length;f<m&&(jo.setBuffer(s[f]),a=Kn(0,0,e,l,n,o,c,0,0,d),jo.clearBuffer(),c+=s[f].length,!a);f++);if(qi.releasePrimitive(d),ya.clearBuffer(),o+=i[h].length,a)break}return Ll=!1,a}function Kn(r,t,e,n,i,s=0,a=0,o=0,c=0,l=null,h=!1){let u,d;h?(u=jo,d=ya):(u=ya,d=jo);const f=u.float32Array,m=u.uint32Array,x=u.uint16Array,g=d.float32Array,p=d.uint32Array,_=d.uint16Array,v=r*2,S=t*2,M=yn(v,x),y=yn(S,_);let w=!1;if(y&&M)h?w=i(In(t,p),Gn(t*2,_),In(r,m),Gn(r*2,x),c,a+t,o,s+r):w=i(In(r,m),Gn(r*2,x),In(t,p),Gn(t*2,_),o,s+r,c,a+t);else if(y){const T=qi.getPrimitive();Ie(t,g,T),T.applyMatrix4(e);const E=Vn(r),A=kn(r,m);Ie(E,f,Js),Ie(A,f,Qs);const D=T.intersectsBox(Js),b=T.intersectsBox(Qs);w=D&&Kn(t,E,n,e,i,a,s,c,o+1,T,!h)||b&&Kn(t,A,n,e,i,a,s,c,o+1,T,!h),qi.releasePrimitive(T)}else{const T=Vn(t),E=kn(t,p);Ie(T,g,Rl),Ie(E,g,Pl);const A=l.intersectsBox(Rl),D=l.intersectsBox(Pl);if(A&&D)w=Kn(r,T,e,n,i,s,a,o,c+1,l,h)||Kn(r,E,e,n,i,s,a,o,c+1,l,h);else if(A)if(M)w=Kn(r,T,e,n,i,s,a,o,c+1,l,h);else{const b=qi.getPrimitive();b.copy(Rl).applyMatrix4(e);const F=Vn(r),L=kn(r,m);Ie(F,f,Js),Ie(L,f,Qs);const N=b.intersectsBox(Js),U=b.intersectsBox(Qs);w=N&&Kn(T,F,n,e,i,a,s,c,o+1,b,!h)||U&&Kn(T,L,n,e,i,a,s,c,o+1,b,!h),qi.releasePrimitive(b)}else if(D)if(M)w=Kn(r,E,e,n,i,s,a,o,c+1,l,h);else{const b=qi.getPrimitive();b.copy(Pl).applyMatrix4(e);const F=Vn(r),L=kn(r,m);Ie(F,f,Js),Ie(L,f,Qs);const N=b.intersectsBox(Js),U=b.intersectsBox(Qs);w=N&&Kn(E,F,n,e,i,a,s,c,o+1,b,!h)||U&&Kn(E,L,n,e,i,a,s,c,o+1,b,!h),qi.releasePrimitive(b)}}return w}const bo=new xn,sf=new tn,fE={strategy:vp,maxDepth:40,maxLeafTris:10,useSharedArrayBuffer:!1,setBoundingBox:!0,onProgress:null,indirect:!1,verbose:!0,range:null};class zh{static serialize(t,e={}){e={cloneBuffers:!0,...e};const n=t.geometry,i=t._roots,s=t._indirectBuffer,a=n.getIndex();let o;return e.cloneBuffers?o={roots:i.map(c=>c.slice()),index:a?a.array.slice():null,indirectBuffer:s?s.slice():null}:o={roots:i,index:a?a.array:null,indirectBuffer:s},o}static deserialize(t,e,n={}){n={setIndex:!0,indirect:!!t.indirectBuffer,...n};const{index:i,roots:s,indirectBuffer:a}=t,o=new zh(e,{...n,[yl]:!0});if(o._roots=s,o._indirectBuffer=a||null,n.setIndex){const c=e.getIndex();if(c===null){const l=new Fe(t.index,1,!1);e.setIndex(l)}else c.array!==i&&(c.array.set(i),c.needsUpdate=!0)}return o}get indirect(){return!!this._indirectBuffer}constructor(t,e={}){if(t.isBufferGeometry){if(t.index&&t.index.isInterleavedBufferAttribute)throw new Error("MeshBVH: InterleavedBufferAttribute is not supported for the index attribute.")}else throw new Error("MeshBVH: Only BufferGeometries are supported.");if(e=Object.assign({...fE,[yl]:!1},e),e.useSharedArrayBuffer&&!uE())throw new Error("MeshBVH: SharedArrayBuffer is not available.");this.geometry=t,this._roots=null,this._indirectBuffer=null,e[yl]||(by(this,e),!t.boundingBox&&e.setBoundingBox&&(t.boundingBox=this.getBoundingBox(new tn))),this.resolveTriangleIndex=e.indirect?n=>this._indirectBuffer[n]:n=>n}refit(t=null){return(this.indirect?eE:ky)(this,t)}traverse(t,e=0){const n=this._roots[e],i=new Uint32Array(n),s=new Uint16Array(n);a(0);function a(o,c=0){const l=o*2,h=s[l+15]===fc;if(h){const u=i[o+6],d=s[l+14];t(c,h,new Float32Array(n,o*4,6),u,d)}else{const u=o+Ma/4,d=i[o+6],f=i[o+7];t(c,h,new Float32Array(n,o*4,6),f)||(a(u,c+1),a(d,c+1))}}}raycast(t,e=0,n=0,i=1/0){const s=this._roots,a=this.geometry,o=[],c=e.isMaterial,l=Array.isArray(e),h=a.groups,u=c?e.side:e,d=this.indirect?nE:Yy;for(let f=0,m=s.length;f<m;f++){const x=l?e[h[f].materialIndex].side:u,g=o.length;if(d(this,f,x,t,o,n,i),l){const p=h[f].materialIndex;for(let _=g,v=o.length;_<v;_++)o[_].face.materialIndex=p}}return o}raycastFirst(t,e=0,n=0,i=1/0){const s=this._roots,a=this.geometry,o=e.isMaterial,c=Array.isArray(e);let l=null;const h=a.groups,u=o?e.side:e,d=this.indirect?sE:Zy;for(let f=0,m=s.length;f<m;f++){const x=c?e[h[f].materialIndex].side:u,g=d(this,f,x,t,n,i);g!=null&&(l==null||g.distance<l.distance)&&(l=g,c&&(g.face.materialIndex=h[f].materialIndex))}return l}intersectsGeometry(t,e){let n=!1;const i=this._roots,s=this.indirect?rE:jy;for(let a=0,o=i.length;a<o&&(n=s(this,a,t,e),!n);a++);return n}shapecast(t){const e=Hn.getPrimitive(),n=this.indirect?Xy:Vy;let{boundsTraverseOrder:i,intersectsBounds:s,intersectsRange:a,intersectsTriangle:o}=t;if(a&&o){const u=a;a=(d,f,m,x,g)=>u(d,f,m,x,g)?!0:n(d,f,this,o,m,x,e)}else a||(o?a=(u,d,f,m)=>n(u,d,this,o,f,m,e):a=(u,d,f)=>f);let c=!1,l=0;const h=this._roots;for(let u=0,d=h.length;u<d;u++){const f=h[u];if(c=Ny(this,u,s,a,i,l),c)break;l+=f.byteLength}return Hn.releasePrimitive(e),c}bvhcast(t,e,n){let{intersectsRanges:i,intersectsTriangles:s}=n;const a=Hn.getPrimitive(),o=this.geometry.index,c=this.geometry.attributes.position,l=this.indirect?m=>{const x=this.resolveTriangleIndex(m);He(a,x*3,o,c)}:m=>{He(a,m*3,o,c)},h=Hn.getPrimitive(),u=t.geometry.index,d=t.geometry.attributes.position,f=t.indirect?m=>{const x=t.resolveTriangleIndex(m);He(h,x*3,u,d)}:m=>{He(h,m*3,u,d)};if(s){const m=(x,g,p,_,v,S,M,y)=>{for(let w=p,T=p+_;w<T;w++){f(w),h.a.applyMatrix4(e),h.b.applyMatrix4(e),h.c.applyMatrix4(e),h.needsUpdate=!0;for(let E=x,A=x+g;E<A;E++)if(l(E),a.needsUpdate=!0,s(a,h,E,w,v,S,M,y))return!0}return!1};if(i){const x=i;i=function(g,p,_,v,S,M,y,w){return x(g,p,_,v,S,M,y,w)?!0:m(g,p,_,v,S,M,y,w)}}else i=m}return dE(this,t,e,i)}intersectsBox(t,e){return bo.set(t.min,t.max,e),bo.needsUpdate=!0,this.shapecast({intersectsBounds:n=>bo.intersectsBox(n),intersectsTriangle:n=>bo.intersectsTriangle(n)})}intersectsSphere(t){return this.shapecast({intersectsBounds:e=>t.intersectsBox(e),intersectsTriangle:e=>e.intersectsSphere(t)})}closestPointToGeometry(t,e,n={},i={},s=0,a=1/0){return(this.indirect?hE:tE)(this,t,e,n,i,s,a)}closestPointToPoint(t,e={},n=0,i=1/0){return Fy(this,t,e,n,i)}getBoundingBox(t){return t.makeEmpty(),this._roots.forEach(n=>{Ie(0,new Float32Array(n),sf),t.union(sf)}),t}}const pE=new URL("/assets/Aorta_plain-_gXpsVDF.stl",import.meta.url).href,mE=new URL("/assets/Aorta_plain.collision-DFUYJYB3.bin",import.meta.url).href,gE=1.3,xE=40,_E=1;function vE(r){const t=[];for(const i of r?.segments||[])i.isSheath||t.push(i.start.y,i.end.y);const e=Math.max(...t,0)+15,n=Math.min(...t,-420)-15;return{center:new C(r?.branchPoint?.x||0,(e+n)*.5+xE,r?.branchPoint?.z||0),length:Math.max(300,e-n)}}function SE(r,t){r.computeBoundingBox();const e=r.boundingBox.clone(),n=e.getSize(new C),i=e.getCenter(new C),s=vE(t),a=s.length*gE/Math.max(1e-6,n.z);return r.translate(-i.x,-i.y,-i.z),r.rotateX(-Math.PI/2),r.scale(a,a,a),r.translate(s.center.x,s.center.y,s.center.z),r.computeBoundingBox(),{version:_E,rotationX:-Math.PI/2,scale:a,sourceCenter:i.toArray(),sourceSize:n.toArray(),targetCenter:s.center.toArray(),targetLength:s.length}}function ME(r){return globalThis.crypto.subtle.digest("SHA-256",r).then(t=>[...new Uint8Array(t)].map(e=>e.toString(16).padStart(2,"0")).join(""))}async function rf(r){const t=await fetch(r);if(!t.ok)throw new Error(`Failed to load ${r}: ${t.status} ${t.statusText}`);return t.arrayBuffer()}function yE(r,t,e){if(r.metadata.source?.stlSha256!==t)throw new Error("Aorta collision asset does not match Aorta_plain.stl; run npm run collision:build");const n=r.metadata.transform;if(n?.version!==e.version||Math.abs((n?.scale??1/0)-e.scale)>1e-7||Math.abs((n?.targetLength??1/0)-e.targetLength)>1e-6)throw new Error("Aorta collision asset transform is stale; run npm run collision:build")}function EE(r){const t=r.arrays.centerlineSegments,e=r.arrays.centerlineEdges,n=r.metadata.centerline.stride,i=[];for(let s=0;s<t.length/n;s++){const a=s*n,o=new C(t[a],t[a+1],t[a+2]),c=new C(t[a+3],t[a+4],t[a+5]),l=c.clone().sub(o),h=l.length();h>1e-8?l.multiplyScalar(1/h):l.set(0,1,0),i.push({id:s,start:o,end:c,axis:l,length:h,radiusStart:t[a+6],radiusEnd:t[a+7],safeRadius:t[a+8],nodeStartId:e[s*2],nodeEndId:e[s*2+1],source:"medial-slice-teasar",aabb:null})}return{type:"centerline-capsule-broadphase",source:"medial-slice-teasar",diagnostics:r.metadata.centerline.diagnostics,inflation:r.metadata.sdf.band,cellSize:r.metadata.broadPhase.cellSize,segments:i,contactField:r}}function wE(r,t=12e3){const e=r.arrays;if(!e.lumenSliceYs?.length)return new Float32Array;const n=e.lumenPoints instanceof Int16Array?r.metadata.lumen?.pointQuantization||.02:1,i=e.lumenAxisBases||new Float32Array([1,0,0,0,1,0,0,0,1]),s=e.lumenAxisSliceOffsets||new Uint32Array([0,e.lumenSliceYs.length]),a=e.lumenPoints.length/2,o=Math.max(1,Math.ceil(a/t)),c=[];let l=0;for(let h=0;h<s.length-1;h++){const u=h*9;for(let d=s[h];d<s[h+1];d++){const f=e.lumenSliceYs[d],m=e.lumenSliceContourOffsets[d],x=e.lumenSliceContourOffsets[d+1];for(let g=m;g<x;g++){const p=e.lumenContourPointOffsets[g],_=e.lumenContourPointOffsets[g+1];for(let v=p;v<_;v++,l++){if(l%o!==0)continue;const S=v+1<_?v+1:p,M=e.lumenPoints[v*2]*n,y=e.lumenPoints[v*2+1]*n,w=e.lumenPoints[S*2]*n,T=e.lumenPoints[S*2+1]*n;c.push(i[u]*M+i[u+3]*f+i[u+6]*y,i[u+1]*M+i[u+4]*f+i[u+7]*y,i[u+2]*M+i[u+5]*f+i[u+8]*y,i[u]*w+i[u+3]*f+i[u+6]*T,i[u+1]*w+i[u+4]*f+i[u+7]*T,i[u+2]*w+i[u+5]*f+i[u+8]*T)}}}}return new Float32Array(c)}function AE(r,t,e){const n=r.metadata;return{geometry:t,interiorSamples:[],lumenSlices:[],lumenField:r.packedLumenField,boundaryDebugSegments:new Float32Array,lumenContourDebugSegments:wE(r),centerlineSliceDebugSegments:null,centerlineExtraction:n.centerline.diagnostics,lumenCastGeometry:null,lumenCast:null,collisionAsset:n,diagnostics:{boundingBox:t.boundingBox.clone(),boundaryEdgeCount:0,degenerateTriangleCount:0,edgeCount:0,interiorSampleCount:0,lumenSliceCount:n.lumen.sliceCount,nonManifoldEdgeCount:0,size:t.boundingBox.getSize(new C),transform:e,triangleCount:n.source.triangleCount,vertexCount:t.attributes.position.count,source:"precompiled-collision-asset"}}}function TE(r,{onLoaded:t,onError:e}={}){const n=new Ne;n.visible=!1;const i=new Ue({color:5213695,transparent:!0,opacity:.34,depthWrite:!1,side:2});return Promise.all([rf(pE),rf(mE)]).then(async([s,a])=>{const[o]=await Promise.all([ME(s)]),c=new hy().parse(s),l=SE(c,r),h=np(a);yE(h,o,l),c.computeVertexNormals(),c.computeBoundingSphere(),c.boundsTree=new zh(c);const u=new oM(h,{fallbackGeometry:c,bvhValidationDistance:.02,capsuleBvhValidation:-.1}),d=EE(u),f=AE(u,c,l),m=new $t(c,i);m.renderOrder=0,n.add(m),n.visible=!0;const x={geometry:c,contactField:u,meshCollider:CE(u),centerlineBroadPhase:d,clearance:.6,guidewireClearance:gr,guidewireSegmentClearance:.12,guidewireCollisionPasses:3,guidewireSegmentSamples:[.2,.4,.6,.8],openOutletY:c.boundingBox.max.y-1,preprocessing:f};typeof t=="function"&&t({group:n,mesh:m,geometry:c,collision:x,preprocessing:f,scale:l.scale})}).catch(s=>{console.warn("Failed to load aorta STL model",s),typeof e=="function"&&e(s)}),{group:n,material:i}}function CE(r){const t=Qn(),e=Qn(),n=(o,c,l,h)=>(typeof o?.set=="function"?o.set(c,l,h):(o.x=c,o.y=l,o.z=h),o),i=(o,c)=>{const l=c||{target:new C,closestPoint:new C,inward:new C,normal:new C};return l.inside=o.inside,l.violation=o.violation,l.distance=Math.max(0,o.signedDistance),l.signedDistance=o.signedDistance,l.signedGap=o.signedGap,l.penetration=o.penetration,l.branchId=o.branchId,l.source=o.source,l.target=l.target||{},l.closestPoint=l.closestPoint||{},l.inward=l.inward||{},l.normal=l.normal||{},n(l.target,o.target.x,o.target.y,o.target.z),n(l.closestPoint,o.closestPoint.x,o.closestPoint.y,o.closestPoint.z),n(l.inward,o.inward.x,o.inward.y,o.inward.z),n(l.normal,-o.inward.x,-o.inward.y,-o.inward.z),l},s=(o,c=0,l=null)=>i(r.querySphere(o,c,t),l),a=(o,c,l=0)=>{const h=r.sweepSphere(o,c,l,e);return!h.violation&&h.timeOfImpact>=1?null:{penetration:h.penetration,point:new C(h.point.x,h.point.y,h.point.z),target:new C(h.target.x,h.target.y,h.target.z),normal:new C(-h.inward.x,-h.inward.y,-h.inward.z),inward:new C(h.inward.x,h.inward.y,h.inward.z),t:h.timeOfImpact,branchId:h.branchId}};return{geometry:r.fallbackGeometry,lumenField:r.packedLumenField,broadPhase:r,contactField:r,containsPoint:o=>!s(o,0,t).violation,pointContact:s,crossingContact:a,clearCache:()=>{}}}const bE=1.5,RE=900,PE=16773994,LE=16777215,DE=16732120,IE=1.05,NE=1.75,FE=3778303,UE=9427199;function BE(r,t=bE){return Number.isFinite(r)&&r>0?r:t}function zE(r,t){r.position.copy(t.start).lerp(t.end,.5),r.quaternion.setFromUnitVectors(new C(0,1,0),t.axis)}function OE(r,t,e,n){const i=new $t(new is(t.radiusEnd,t.radiusStart,t.length,18,1,!0),e);zE(i,t),i.renderOrder=4.5,i.userData.debugLayer="capsules",r.add(i);const s=new $t(new ti(t.radiusStart,16,8),n);s.position.copy(t.start),s.renderOrder=4.4,s.userData.debugLayer="capsules",r.add(s);const a=new $t(new ti(t.radiusEnd,16,8),n);a.position.copy(t.end),a.renderOrder=4.4,a.userData.debugLayer="capsules",r.add(a)}function Ro(r){return[Math.round(r.x*8),Math.round(r.y*8),Math.round(r.z*8)].join(",")}function Dl(r,t,e,n){let i=r.get(t);i||(i={point:new C,radius:0,degree:0,weight:0},r.set(t,i)),i.point.add(e),i.radius=Math.max(i.radius,BE(n)),i.degree++,i.weight++}function GE(r){const t=new Map;for(const e of r){if(e.nodeStartId!==void 0&&e.nodeStartId===e.nodeEndId){const s=e.start.clone().lerp(e.end,.5);Dl(t,`node:${e.nodeStartId}`,s,Math.max(e.radiusStart,e.radiusEnd));continue}const n=e.nodeStartId!==void 0?`node:${e.nodeStartId}:${Ro(e.start)}`:`point:${Ro(e.start)}`,i=e.nodeEndId!==void 0?`node:${e.nodeEndId}:${Ro(e.end)}`:`point:${Ro(e.end)}`;Dl(t,n,e.start,e.radiusStart),Dl(t,i,e.end,e.radiusEnd)}return[...t.values()].map(e=>({...e,point:e.point.multiplyScalar(1/Math.max(1,e.weight))}))}function af(r,{radius:t,color:e,opacity:n,renderOrder:i}){if(!r.length)return null;const s=new wr(new ti(t,10,6),new Ue({color:e,transparent:!0,opacity:n,depthTest:!1,depthWrite:!1,toneMapped:!1}),r.length),a=new re,o=new C;for(let c=0;c<r.length;c++){const l=Math.max(.72,Math.min(1.65,r[c].radius*.16));o.setScalar(l),a.compose(r[c].point,new Fn,o),s.setMatrixAt(c,a)}return s.frustumCulled=!1,s.instanceMatrix.needsUpdate=!0,s.renderOrder=i,s.userData.debugLayer="centerline",s}function VE(r,{maxCapsules:t=RE}={}){const e=new Ne;if(!r?.segments?.length)return e;const n=r.segments,i=new Float32Array(n.length*6);for(let v=0;v<n.length;v++){const S=n[v];i[v*6]=S.start.x,i[v*6+1]=S.start.y,i[v*6+2]=S.start.z,i[v*6+3]=S.end.x,i[v*6+4]=S.end.y,i[v*6+5]=S.end.z}const s=new we;s.setAttribute("position",new Fe(i,3));const a=new ts(s,new di({color:PE,transparent:!0,opacity:.96,depthTest:!1,depthWrite:!1,toneMapped:!1}));a.frustumCulled=!1,a.renderOrder=9.7,a.userData.debugLayer="centerline",e.add(a);const o=GE(n),c=o.filter(v=>v.degree===2),l=o.filter(v=>v.degree!==2),h=af(c,{radius:IE,color:LE,opacity:.92,renderOrder:9.78});h&&e.add(h);const u=af(l,{radius:NE,color:DE,opacity:.98,renderOrder:9.82});u&&e.add(u);const d=Number.isFinite(t)&&t>0?Math.max(1,Math.ceil(r.segments.length/t)):1,f=r.segments.filter((v,S)=>S%d===0),m=new Ue({color:FE,transparent:!0,opacity:.105,depthTest:!0,depthWrite:!1,side:2,toneMapped:!1}),x=m.clone();x.opacity=.075;for(const v of f)OE(e,v,m,x);const g=new Float32Array(f.length*6);for(let v=0;v<f.length;v++){const S=f[v];g[v*6]=S.start.x,g[v*6+1]=S.start.y,g[v*6+2]=S.start.z,g[v*6+3]=S.end.x,g[v*6+4]=S.end.y,g[v*6+5]=S.end.z}const p=new we;p.setAttribute("position",new Fe(g,3));const _=new ts(p,new di({color:UE,transparent:!0,opacity:.38,depthTest:!1,depthWrite:!1,toneMapped:!1}));return _.frustumCulled=!1,_.renderOrder=9.55,_.userData.debugLayer="capsules",e.add(_),e.userData.broadPhase=r,e.userData.displayedSegmentCount=f.length,e.userData.centerlineNodeCount=o.length,e.userData.centerlineBranchNodeCount=l.length,e}const kE=`
// Fullscreen quad vertex shader.
// Passes through the quad UVs and positions to draw a screen-aligned quad.
varying vec2 vUv;
void main() {
    vUv = uv;
    gl_Position = vec4(position.xy, 0.0, 1.0);
}
`,HE=`
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
`,WE=`
// Fullscreen quad vertex shader for thickness pass.
// Renders a screen-aligned quad; vUv is used to sample depth textures.
varying vec2 vUv;
void main() {
    vUv = uv;
    gl_Position = vec4(position.xy, 0.0, 1.0);
}
`,XE=`
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
`,YE=`
// Fullscreen quad vertex shader for the final display pass.
// Simply forwards UVs to the fragment shader.
varying vec2 vUv;
void main() {
    vUv = uv;
    gl_Position = vec4(position.xy, 0.0, 1.0);
}
`,qE=`
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
`,$o=600*1e3,mc=72*1e3;function ZE(){return{guidewireAdvance:0,catheterAdvance:0,catheterRotation:0,catheterType:"pigtail"}}function Ap(r){const t=Math.max(0,r);return Math.floor(t/mc)%2===0?"pigtail":"berenstein"}function of(r,t){const e=Math.max(0,r),n=e%mc;return t.guidewireAdvance=0,t.catheterAdvance=0,t.catheterRotation=0,t.catheterType=Ap(e),n<15e3?t.guidewireAdvance=1:n<25e3?t.catheterAdvance=1:n<35e3?t.catheterRotation=Math.floor((n-25e3)/2500)%2===0?1:-1:n<52e3?(t.catheterAdvance=-1,t.catheterRotation=Math.floor((n-35e3)/2500)%2===0?-1:1):n<67e3&&(t.guidewireAdvance=-1),t}const Oh=2752468,jE=5213695,$E=6946702,KE=16751421,JE=11009884,QE=16732120,t1=16765514,e1=16724821,n1=16733695,Ms=420,Tp=1/10,i1=1.85,s1=32,r1=Xl*1.35,Cp=1/30,a1=new URLSearchParams(window.location.search).get("physics"),En=a1==="legacy"?"legacy":"xpbd-contact-v1",bp=.1,Rp=1e3,aa=document.getElementById("loadingScreen"),cf=document.getElementById("loadingMessage"),Ki=new Set(["aorta","skeleton","firstFrame"]);let lf=!1,Ea=null;function Pa(r){cf&&(cf.textContent=r)}function Pp(){return!Ki.has("aorta")&&!Ki.has("skeleton")}function o1(){lf||!aa||(lf=!0,Pa("Ready"),aa.classList.add("is-hidden"),aa.addEventListener("transitionend",()=>aa.remove(),{once:!0}),setTimeout(()=>aa.remove(),900))}function xr(r,t){Ki.has(r)&&(Ki.delete(r),r==="firstFrame"&&Ea&&(clearTimeout(Ea),Ea=null),t&&Pa(t),c1(),Ki.size===0&&o1())}function Lp(r){xr(r,"Loading fallback view")}function c1(){!Pp()||!Ki.has("firstFrame")||Ea||(Pa("Rendering first frame"),requestAnimationFrame(()=>xr("firstFrame","Ready")),Ea=setTimeout(()=>xr("firstFrame","Ready"),1800))}function hf(){Pp()&&xr("firstFrame","Ready")}Pa("Preparing renderer");const l1=document.getElementById("sim"),Xt=new Ch({canvas:l1,antialias:!0});Xt.setSize(window.innerWidth,window.innerHeight);const Ko=.64,h1=()=>Math.max(1,Math.round(window.innerWidth*Ko)),u1=()=>Math.max(1,Math.round(window.innerHeight*Ko)),Wn=h1(),Xn=u1(),d1=Xt.capabilities.isWebGL2?2:0,Dp={samples:d1},he=new Er;he.background=new Kt(0);const oa=new Er,ch=new Qe(Wn,Xn,Dp),Jo=new Qe(Wn,Xn),Qo=new Qe(Wn,Xn,Dp),tc=new Qe(Wn,Xn),ec=new Qe(Wn,Xn),gc=new Qe(Wn,Xn,{type:1016}),Ip=new Qe(Wn,Xn),Np=new Qe(Wn,Xn),nc=new Qe(Wn,Xn),ic=new Qe(Wn,Xn),xc=new Qe(Wn,Xn);let Vo=Ip,Po=Np;const uf=new Float64Array(16),df=new Float64Array(16);let sc=!1;const ko=new Ah(-1,1,1,-1,0,1),Gh=new lc(2,2),rc=new Un({uniforms:{currentFrame:{value:null},previousFrame:{value:null},decay:{value:.95}},vertexShader:kE,fragmentShader:HE}),f1=new $t(Gh,rc),Fp=new Er;Fp.add(f1);function Up(r){return new Un({side:r,depthTest:!0,depthWrite:!0,uniforms:{cameraNear:{value:bp},cameraFar:{value:Rp}},vertexShader:`
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
        `})}const p1=Up(0),m1=Up(1),lh=new Un({uniforms:{frontDepth:{value:nc.texture},backDepth:{value:ic.texture}},vertexShader:WE,fragmentShader:XE}),g1=new $t(Gh,lh),Bp=new Er;Bp.add(g1);const x1=new Un({transparent:!0,blending:5,blendEquation:100,blendSrc:201,blendDst:201,blendEquationAlpha:100,blendSrcAlpha:201,blendDstAlpha:201,side:2,depthTest:!1,depthWrite:!1,vertexShader:`
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
    `}),mn=new Un({uniforms:{uTexture:{value:Vo.texture},contrastTexture:{value:Jo.texture},thicknessTexture:{value:xc.texture},metalTexture:{value:Qo.texture},catheterTexture:{value:tc.texture},sheathTexture:{value:ec.texture},boneTexture:{value:gc.texture},gray:{value:new Kt(15461355)},fluoroscopy:{value:!1},time:{value:0},noiseLevel:{value:.1},imageBrightness:{value:.18},imageContrast:{value:1.33},autoExposureEnabled:{value:!1},autoExposureLevel:{value:0},pulseRate:{value:15},scatterStrength:{value:.45},collimation:{value:.08},boneOpacity:{value:.62},resolution:{value:new yt(Wn,Xn)},edgeStrength:{value:.1},contrastOpacity:{value:1},contrastGain:{value:5}},vertexShader:YE,fragmentShader:qE}),_1=new $t(Gh,mn),hh=new Er;hh.add(_1);const zp=350,Oe=new Dn(45,window.innerWidth/window.innerHeight,bp,Rp);Oe.position.set(0,80,zp);he.add(Oe);const Op=-15;function Fi(r){return r.position.y+=Op,r}let Tr;const{group:ys}=PM({onLoaded:()=>{sc=!1,xr("skeleton",Ki.has("aorta")?"Loading vessel model":"Rendering first frame")},onError:()=>Lp("skeleton")}),{vessel:hn}=fM(140,0);Tr=Fi(new Ne);let gn=hn,xe=null,Ge=null,_e=null,De=null,Ui=null,Zi=null,vs=null,Pi=null,dr=null,ji=null;function Gp(r,t=1){const e=new C(r.start.x,r.start.y,r.start.z),n=new C(r.end.x,r.end.y,r.end.z),i=new C().subVectors(n,e),s=i.length(),a=r.radius*t,o=new is(a,a,s,18,1,!0);return o.applyQuaternion(new Fn().setFromUnitVectors(new C(0,1,0),i.normalize())),o.translate((e.x+n.x)*.5,(e.y+n.y)*.5,(e.z+n.z)*.5),o}function v1(r){const t=Gp(r),e=new Ue({color:Oh,side:2,transparent:!0,opacity:.34,depthWrite:!1,depthTest:!1}),n=new $t(t,e);return n.renderOrder=6.6,n}function S1(r){const t=Gp(r),e=new Ue({color:16777215,side:2,transparent:!0,opacity:.065,depthTest:!1,depthWrite:!1}),n=new $t(t,e);return n.renderOrder=.7,n}function ff(r,{debugLayer:t=null,color:e=Oh,opacity:n=.24,renderOrder:i=3,depthTest:s=!0}={}){const a=new Ue({color:e,side:2,transparent:!0,opacity:n,depthWrite:!1,depthTest:s}),o=new $t(r,a);return o.renderOrder=i,t&&(o.userData.debugLayer=t),o}function M1(r,t){const e=r?.meshCollider||r?.lumenMeshCollider||null;if(!e?.pointContact||!t?.start||!t?.end)return null;const n=new C(t.start.x,t.start.y,t.start.z),i=new C(t.end.x,t.end.y,t.end.z),s=new C().subVectors(i,n),a=s.length();if(a<1e-6)return null;const o=d=>n.clone().addScaledVector(s,d),c=d=>e.pointContact(o(d),0)?.signedDistance??-1/0,l=Math.max(16,Math.ceil(a/2));let h=0,u=c(0);for(let d=1;d<=l;d++){const f=d/l,m=c(f);if(u<0&&m>=0){let x=h,g=f;for(let p=0;p<14;p++){const _=(x+g)*.5;c(_)>=0?g=_:x=_}return{point:o(g),tangent:s.normalize()}}h=f,u=m}return null}function y1(r,t){const e=M1(r,t),n=new Ne;if(!e)return n;const i=new Ue({color:QE,transparent:!0,opacity:.95,depthTest:!1,depthWrite:!1,toneMapped:!1}),s=new $t(new ti(2.2,18,12),i);s.renderOrder=9.5,n.add(s);const a=new $t(new Dh(4.2,.32,8,32),i.clone());return a.quaternion.setFromUnitVectors(new C(0,0,1),e.tangent),a.renderOrder=9.4,n.add(a),n.position.copy(e.point),n.frustumCulled=!1,n}Tr.add(v1(hn.sheath));const Ss=S1(hn.sheath);Ss.visible=!0;Fi(Ss);he.add(Ss);const hi=new Ne;hi.visible=!1;Tr.add(hi);const fr={stlModel:!0,lumenCast:!1,sections:!1,centerline:!0,capsules:!1};function Vp(){hi.traverse(r=>{const t=r.userData?.debugLayer;!t||!(t in fr)||(r.visible=!!fr[t])})}Pa("Loading anatomy models");TE(hn,{onLoaded:({collision:r})=>{gn={...r,segments:[hn.sheath]},_e&&(_e.contactField=r.contactField),hi.clear(),hi.add(ff(r.geometry,{debugLayer:"stlModel",color:jE,opacity:.18,renderOrder:2.8})),r.preprocessing?.lumenCastGeometry&&hi.add(ff(r.preprocessing.lumenCastGeometry,{debugLayer:"lumenCast",color:Oh,opacity:.28,renderOrder:9.15,depthTest:!1})),hi.add(T1(r.preprocessing)),hi.add(VE(r.centerlineBroadPhase)),hi.add(y1(r,hn.sheath)),Vp(),Ge?.requestSettle?.(90),xe?.setCollisionGeometry(r),xr("aorta",Ki.has("skeleton")?"Loading skeleton model":"Rendering first frame")},onError:()=>{Lp("aorta")}});he.add(Tr);ys.position.set(hn.branchPoint.x,hn.branchPoint.y-60,hn.branchPoint.z-50);ys.renderOrder=-1;he.add(ys);const wa=new FM(hn,3.5),ci=Fi(new Ne);he.add(ci);let Ci=null,Il=0,Nl=0;const Di=5,_c=201,pr=Di*(_c-1),E1=44,fi=new Yv(_c,Di,{constraintIterations:28});let vc=0;const w1=pr,A1=0;function T1(r){const t=new Ne;if(!r)return t;if(r.boundaryDebugSegments?.length){const i=new we;i.setAttribute("position",new Fe(r.boundaryDebugSegments,3));const s=new ts(i,new di({color:KE,transparent:!0,opacity:.85,depthTest:!1,depthWrite:!1,toneMapped:!1}));s.frustumCulled=!1,s.renderOrder=9,s.userData.debugLayer="sections",t.add(s)}const e=r.centerlineSliceDebugSegments?.length?r.centerlineSliceDebugSegments:r.lumenContourDebugSegments;if(e?.length){const i=new we;i.setAttribute("position",new Fe(e,3));const s=new ts(i,new di({color:JE,transparent:!0,opacity:.72,depthTest:!1,depthWrite:!1,toneMapped:!1}));s.frustumCulled=!1,s.renderOrder=8.5,s.userData.debugLayer="sections",t.add(s)}const n=r.interiorSamples||[];if(n.length){const i=new wr(new ti(1.15,10,6),new Ue({color:$E,transparent:!0,opacity:.82,depthTest:!1,depthWrite:!1,toneMapped:!1}),n.length),s=new re;for(let a=0;a<n.length;a++)s.makeTranslation(n[a].x,n[a].y,n[a].z),i.setMatrixAt(a,s);i.frustumCulled=!1,i.instanceMatrix.needsUpdate=!0,i.renderOrder=8,i.userData.debugLayer="sections",t.add(i)}return t}Ge=new IS({rod:fi,segmentLength:Di,guidewireLength:pr,sheath:hn.sheath,advanceRate:E1,minInsert:A1,maxInsert:w1,lumenClearance:gr,straightening:.72,routeBlend:0,relaxationIterations:6,lengthIterations:10,meshClearance:gr,foldGuardAngle:166,foldGuardStrength:.62,foldGuardPasses:2,foldGuardCenterPull:1.25,stabilityRepairSegmentError:.09,stabilityRepairBendAngle:150,stabilityRepairTargetBendAngle:112,stabilityRepairPasses:3,stabilityRepairLengthIterations:10,tipBacktrackAngle:108,tipBacktrackStrength:1,segmentProjectionBlend:.48,maxSegmentProjectionStep:.32,collisionProjectionRepeats:1,segmentSamples:[.1,.24,.38,.52,.66,.8,.93],finalCollisionPasses:3,finalLengthPasses:2,finalProjectionPasses:2});OS(fi,{segmentLength:Di});Ge.initialize();vc=Ge.progress;let ui=!1,uh=0,kp=2,dh=2,pf=10,Aa=0,mf=0;const Hp=new Ue({color:16777215,depthTest:!1,depthWrite:!1,toneMapped:!1}),gf=new Un({vertexShader:`
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
    `,depthTest:!1,depthWrite:!1,toneMapped:!1});let Re=!0,wn=null,An=null,Ve=null;const Pe=EM({camera:Oe,cameraRadius:zp,vessel:hn,voxelGroup:ci,displayMaterial:mn,blendMaterial:rc,wireMaterial:Hp,onStartInjection:({rate:r,duration:t,volume:e})=>{ui||(ui=!0,uh=0,dh=r,kp=t,pf=e,Aa=pf)},onStopInjection:()=>{ui&&(ui=!1,Aa=0)},onModeChange:r=>{Re=r,Tr.visible=!Re,Ss.visible=Re,hi.visible=!Re,wn&&(wn.visible=!Re),An&&(An.visible=!Re),Ve&&(Ve.visible=!Re&&!!Ve.userData.hasPoint),Pi&&(Pi.visible=!Re&&!!fr.capsules),ys.visible=Re,mn.uniforms.fluoroscopy.value=Re},onDebugLayerChange:r=>{Object.assign(fr,r),Vp(),Pi&&(Pi.visible=!Re&&!!fr.capsules)},onStartBrowserBenchmark:r=>$p({durationMs:r}),onStopBrowserBenchmark:()=>qh("ui")}),{monitor:C1}=Pe,b1=new is(Xl,Xl,1,s1,1,!1),Ji=new wr(b1,Hp,_c-1);Ji.instanceMatrix.setUsage(35048);Ji.frustumCulled=!1;Ji.renderOrder=7;Ji.count=0;const _r=new Ne;_r.add(Ji);Fi(_r);he.add(_r);const xf=new re,_f=new Fn,Lo=new C,vf=new C,Sf=new C(1,1,1),R1=new C(0,1,0),Mf=new re,P1=new ti(1.35,12,8),L1=new ti(2.1,12,8);wn=new wr(P1,new Ue({color:t1,transparent:!0,opacity:.95,depthTest:!1,depthWrite:!1,toneMapped:!1}),Ms);wn.instanceMatrix.setUsage(35048);wn.count=0;wn.visible=!0;wn.frustumCulled=!1;wn.renderOrder=6;Fi(wn);he.add(wn);An=new wr(L1,new Ue({color:e1,transparent:!0,opacity:1,depthTest:!1,depthWrite:!1,toneMapped:!1}),Ms);An.instanceMatrix.setUsage(35048);An.count=0;An.visible=!0;An.frustumCulled=!1;An.renderOrder=7;Fi(An);he.add(An);Ve=new $t(new ti(2.8,16,10),new Ue({color:n1,transparent:!0,opacity:1,depthTest:!1,depthWrite:!1,toneMapped:!1}));Ve.visible=!1;Ve.frustumCulled=!1;Ve.renderOrder=8;Ve.userData.hasPoint=!1;Fi(Ve);he.add(Ve);function Wp(r){const t=new Float32Array(Ms*6),e=new we;e.setAttribute("position",new Fe(t,3)),e.setDrawRange(0,0);const n=new ts(e,new di({color:r,transparent:!0,opacity:.95,depthTest:!1,depthWrite:!1,toneMapped:!1}));return n.frustumCulled=!1,n.renderOrder=9.9,n}Pi=new Ne;dr=Wp(2686935);ji=Wp(16732120);Pi.add(dr,ji);Pi.visible=!Re&&!!fr.capsules;Fi(Pi);he.add(Pi);const D1={routeAssist:En==="legacy",boundaryDriven:En==="xpbd-contact-v1"},I1={resetVelocity:En!=="xpbd-contact-v1"},N1={collisions:!1},Xp={shapeCompliance:dc.catheter.shapeCompliance};xe=new ly({wire:fi,segmentLength:Di,guidewireLength:pr,tailProgressRef:()=>Ge.progress,vessel:hn});xe.setExternalCollisionSolver(En==="xpbd-contact-v1");gn!==hn&&xe.setCollisionGeometry(gn);Fi(xe.mesh);he.add(xe.mesh);_e=new dM({contactField:gn.contactField||null,fixedDt:1/120,maxSubsteps:2,iterations:6,penetrationIterations:8,highPenetration:.15,contactActivation:.2});De=_e.createRod("guidewire",_c,Di,{...dc.guidewire});De.syncFromElasticRod(fi);Ui=_e.createRod("catheter",320,4,{...dc.catheter});xe.syncXpbdBody(Ui,Xp);_e.addSheath({start:hn.sheath.start,end:hn.sheath.end,innerRadius:dp,bodies:[De,Ui]});Zi=_e.addContainment(De,Ui,{innerRadius:gp,openProximal:!0,openDistal:!0,searchWindow:2,outerStartNode:xe.physicsLumenStartNode,innerResponse:0,outerResponse:1,finalProjection:"outer",outerFollowsInnerCenterline:!0,containedLength:0,enabled:!1});vs=_e.addToolContact(De,Ui,{friction:.08,openDistalB:!0,enabled:!1});const yf=[De,Ui];globalThis.__OET_PHYSICS__={mode:En,world:_e,getStats:()=>_e.getStats()};const ws=4e4,Yp=mc*2,F1=60*1e3,fh=Yp+F1,U1=610,qp=256,Vh=8,da=new Float32Array(ws),ph=new Float32Array(U1),mh=new Float32Array(ws),B1=new Float32Array(ws),gh=new Float32Array(ws),xh=new Float32Array(ws),ln=new Float32Array(qp*Vh);let fa=0,Es=0,Ca=0,ba=0,kh=0,ac=0,vr=0,pa=0,Ho=0,oc=0,_h=0,Hh=0,Wh=0,Qi=0,Zp=performance.now();const Zt={count:0,simulationSumMs:0,updateSumMs:0,renderSumMs:0,totalSumMs:0,maximumMs:0,simulationMaximumMs:0,updateMaximumMs:0,renderMaximumMs:0,lastSimulationMs:0,lastUpdateMs:0,lastRenderMs:0,lastTotalMs:0},Te={supported:!1,samples:0,startBytes:null,minimumBytes:null,maximumBytes:null,endBytes:null},Et={running:!1,warmingUp:!1,durationMs:$o,warmupStartedAt:0,memorySettling:!1,startedAt:0,completedAt:0,simulationElapsedMs:0,stopReason:null,automated:!1},z1=["pointerdown","mousedown","touchstart","click","dblclick","wheel","keydown","input","change"];function O1(r){!Et.running||!Et.automated||(r.preventDefault(),r.stopImmediatePropagation())}for(const r of z1)window.addEventListener(r,O1,{capture:!0,passive:!1});const ms=ZE(),Me={steps:0,maxPostStepPenetrationMm:0,maxPostStepPenetrationStep:-1,maxPostStepPenetrationBodyId:null,maxPostStepPenetrationSegment:-1,maxPostStepPenetrationT:0,maxPostStepPenetrationX:0,maxPostStepPenetrationY:0,maxPostStepPenetrationZ:0,maxTransientPenetrationMm:0,maxTransientPenetrationStep:-1,maxSegmentErrorPercent:0,maxSegmentErrorBodyId:null,maxSegmentErrorNodeIndex:-1,maxSegmentErrorStep:-1,maxBendAngleDegrees:0,maxBendBodyId:null,maxBendNodeIndex:-1,maxBendStep:-1,maxBendX:0,maxBendY:0,maxBendZ:0,finite:!0};let Ra=null;function Xh(){const r=performance.memory?.usedJSHeapSize;Number.isFinite(r)&&(Te.supported=!0,Te.samples===0?(Te.startBytes=r,Te.minimumBytes=r,Te.maximumBytes=r):(Te.minimumBytes=Math.min(Te.minimumBytes,r),Te.maximumBytes=Math.max(Te.maximumBytes,r)),Te.endBytes=r,Te.samples++)}function G1(){Te.supported=!1,Te.samples=0,Te.startBytes=null,Te.minimumBytes=null,Te.maximumBytes=null,Te.endBytes=null,Xh()}function V1(){return Xh(),{...Te,growthBytes:Te.supported?Te.endBytes-Te.startBytes:null,rangeBytes:Te.supported?Te.maximumBytes-Te.minimumBytes:null}}function Yh(){fa=0,Es=0,Ca=0,ba=0,kh=0,ac=0,vr=0,pa=0,Ho=0,oc=0,_h=Pe.getCArmRevision?.()??0,Hh=0,Wh=0,Qi=document.hasFocus()?0:performance.now(),Zt.count=0,Zt.simulationSumMs=0,Zt.updateSumMs=0,Zt.renderSumMs=0,Zt.totalSumMs=0,Zt.maximumMs=0,Zt.simulationMaximumMs=0,Zt.updateMaximumMs=0,Zt.renderMaximumMs=0,Zt.lastSimulationMs=0,Zt.lastUpdateMs=0,Zt.lastRenderMs=0,Zt.lastTotalMs=0,G1(),Me.steps=0,Me.maxPostStepPenetrationMm=0,Me.maxPostStepPenetrationStep=-1,Me.maxPostStepPenetrationBodyId=null,Me.maxPostStepPenetrationSegment=-1,Me.maxTransientPenetrationMm=0,Me.maxTransientPenetrationStep=-1,Me.maxSegmentErrorPercent=0,Me.maxSegmentErrorBodyId=null,Me.maxSegmentErrorNodeIndex=-1,Me.maxSegmentErrorStep=-1,Me.maxBendAngleDegrees=0,Me.maxBendBodyId=null,Me.maxBendNodeIndex=-1,Me.maxBendStep=-1,Me.maxBendX=0,Me.maxBendY=0,Me.maxBendZ=0,Me.finite=!0,Zp=performance.now(),_e.resetPerformanceStats(),gn.contactField?.resetStats?.()}function k1(r){if(!(!Number.isFinite(r)||r<=0)){if(Es===da.length?Ca-=da[fa]:Es++,da[fa]=r,Ca+=r,ba=Math.max(ba,r),pa+=r,Ho++,pa>=1e3&&(Xh(),vr<ph.length&&(ph[vr++]=Ho*1e3/pa),pa=0,Ho=0),r>1e3/30&&(kh++,oc<qp)){const t=oc++*Vh;ln[t]=r,ln[t+1]=Et.running?performance.now()-Et.startedAt:-1,ln[t+2]=Et.simulationElapsedMs,ln[t+3]=performance.memory?.usedJSHeapSize??-1,ln[t+4]=Zt.lastSimulationMs,ln[t+5]=Zt.lastUpdateMs,ln[t+6]=Zt.lastRenderMs,ln[t+7]=Zt.lastTotalMs}r>50&&ac++,fa=(fa+1)%da.length}}window.addEventListener("blur",()=>{!Et.running||Et.warmingUp||Qi>0||(Hh++,Qi=performance.now())});window.addEventListener("focus",()=>{Qi<=0||(Et.running&&!Et.warmingUp&&(Wh+=performance.now()-Qi),Qi=0)});function H1(){const r=Me;if(r.steps++,_e.settledMaxPenetration>r.maxPostStepPenetrationMm&&(r.maxPostStepPenetrationMm=_e.settledMaxPenetration,r.maxPostStepPenetrationStep=r.steps,r.maxPostStepPenetrationBodyId=_e.settledContactBodyId,r.maxPostStepPenetrationSegment=_e.settledContactSegment,r.maxPostStepPenetrationT=_e.settledContactT,r.maxPostStepPenetrationX=_e.settledContactX,r.maxPostStepPenetrationY=_e.settledContactY,r.maxPostStepPenetrationZ=_e.settledContactZ),_e.maxPenetration>r.maxTransientPenetrationMm&&(r.maxTransientPenetrationMm=_e.maxPenetration,r.maxTransientPenetrationStep=r.steps),!(r.steps!==1&&r.steps%30!==0))for(let t=0;t<yf.length;t++){const e=yf[t];if(!e)continue;const n=e.activeStart,i=Math.min(e.activeEnd,e.segmentCount);for(let s=n;s<=e.activeEnd;s++)r.finite=r.finite&&Number.isFinite(e.x[s])&&Number.isFinite(e.y[s])&&Number.isFinite(e.z[s])&&Number.isFinite(e.velocityX[s])&&Number.isFinite(e.velocityY[s])&&Number.isFinite(e.velocityZ[s]);for(let s=n;s<i;s++){const a=e.x[s+1]-e.x[s],o=e.y[s+1]-e.y[s],c=e.z[s+1]-e.z[s],l=Math.sqrt(a*a+o*o+c*c),h=Math.abs(l-e.restLength[s])/Math.max(1e-8,e.restLength[s])*100;if(h>r.maxSegmentErrorPercent&&(r.maxSegmentErrorPercent=h,r.maxSegmentErrorBodyId=e.id,r.maxSegmentErrorNodeIndex=s,r.maxSegmentErrorStep=r.steps),s<=n)continue;const u=e.x[s]-e.x[s-1],d=e.y[s]-e.y[s-1],f=e.z[s]-e.z[s-1],m=Math.sqrt(a*a+o*o+c*c)*Math.sqrt(u*u+d*d+f*f);if(m<=1e-8)continue;const x=ue.clamp((a*u+o*d+c*f)/m,-1,1),g=Math.acos(x)*180/Math.PI;g>r.maxBendAngleDegrees&&(r.maxBendAngleDegrees=g,r.maxBendBodyId=e.id,r.maxBendNodeIndex=s,r.maxBendStep=r.steps,r.maxBendX=e.x[s],r.maxBendY=e.y[s],r.maxBendZ=e.z[s])}}}function W1(r){if(!Es)return 0;const t=Array.from(da.subarray(0,Es));return t.sort((e,n)=>e-n),t[Math.min(t.length-1,Math.floor((t.length-1)*r))]}function X1(){if(!vr)return 0;const r=Array.from(ph.subarray(0,vr));r.sort((n,i)=>n-i);const t=Math.max(1,Math.ceil(r.length*.01));let e=0;for(let n=0;n<t;n++)e+=r[n];return e/t}function Y1(){const r=[];for(let t=0;t<oc;t++){const e=t*Vh;r.push({frameMs:ln[e],elapsedMs:ln[e+1],simulationElapsedMs:ln[e+2],heapBytes:ln[e+3],previousFrameCpu:{simulationMs:ln[e+4],updateMs:ln[e+5],renderMs:ln[e+6],totalMs:ln[e+7]}})}return r}function Ef(r,t,e){if(!Et.running)return;const n=performance.now(),i=t-r,s=e-t,a=n-e,o=n-r,c=Zt.count;c<ws&&(mh[c]=i,B1[c]=s,gh[c]=a,xh[c]=o),Zt.count++,Zt.simulationSumMs+=i,Zt.updateSumMs+=s,Zt.renderSumMs+=a,Zt.totalSumMs+=o,Zt.maximumMs=Math.max(Zt.maximumMs,o),Zt.simulationMaximumMs=Math.max(Zt.simulationMaximumMs,i),Zt.updateMaximumMs=Math.max(Zt.updateMaximumMs,s),Zt.renderMaximumMs=Math.max(Zt.renderMaximumMs,a),Zt.lastSimulationMs=i,Zt.lastUpdateMs=s,Zt.lastRenderMs=a,Zt.lastTotalMs=o}function tr(r,t){const e=Math.min(Zt.count,ws);if(!e)return 0;const n=Array.from(r.subarray(0,e));return n.sort((i,s)=>i-s),n[Math.min(e-1,Math.floor((e-1)*t))]}function q1(){const r=Zt.count||1;return{samples:Zt.count,simulationAverageMs:Zt.simulationSumMs/r,updateAverageMs:Zt.updateSumMs/r,renderAverageMs:Zt.renderSumMs/r,totalAverageMs:Zt.totalSumMs/r,simulationP95Ms:tr(mh,.95),simulationP99Ms:tr(mh,.99),renderP95Ms:tr(gh,.95),renderP99Ms:tr(gh,.99),totalP95Ms:tr(xh,.95),totalP99Ms:tr(xh,.99),simulationMaximumMs:Zt.simulationMaximumMs,updateMaximumMs:Zt.updateMaximumMs,renderMaximumMs:Zt.renderMaximumMs,maximumMs:Zt.maximumMs}}function Sc(){const r=performance.now(),t=Et.warmingUp?0:Et.running?Math.min(Et.durationMs,r-Et.startedAt):Et.completedAt>Et.startedAt?Math.min(Et.durationMs,Et.completedAt-Et.startedAt):0;return{running:Et.running,warmingUp:Et.warmingUp,warmupPhase:Et.warmingUp?Et.memorySettling?"memory-settle":"choreography":"complete",warmupElapsedMs:Et.warmingUp?Math.min(fh,r-Et.warmupStartedAt):fh,durationMs:Et.durationMs,elapsedMs:t,simulationElapsedMs:Et.simulationElapsedMs,progress:Et.durationMs>0?Math.min(1,t/Et.durationMs):0,cycleIndex:Math.floor(Et.simulationElapsedMs/mc),catheterType:Ap(Et.simulationElapsedMs),stopReason:Et.stopReason,automated:Et.automated}}function vh(){const r=performance.now(),t=W1(.99),e=X1(),n=_e.getStats(),i=gn.contactField?.getStats?.()||null,s=Sc(),a=!s.running&&s.durationMs>=$o&&s.elapsedMs>=$o,o=e>=55,c=ac===0,l=n.phases.total.averageMs<=4&&n.phases.total.p95Ms<=6,h=Me.maxPostStepPenetrationMm<=.2,u=Me.maxSegmentErrorPercent<=1,d=Me.maxBendAngleDegrees<150,f=Me.finite,m=En==="xpbd-contact-v1",x=!!_e.contactField,g=Math.max(0,(Pe.getCArmRevision?.()??_h)-_h),p=g===0,_=Wh+(Qi>0?r-Qi:0),v=_<=100,S=V1(),M=!S.supported||S.growthBytes<=4*1024*1024&&S.rangeBytes<=8*1024*1024,y=i?.resultAllocations===0,w=(i?.runtimeBytes??1/0)<=32*1024*1024,T=ba<100&&M&&y;return{mode:En,durationMs:performance.now()-Zp,frameCount:Es,averageFps:Ca>0?Es*1e3/Ca:0,onePercentLowFps:e,p99FrameMs:t,instantaneousP99Fps:t>0?1e3/t:0,fpsWindowCount:vr,maxFrameMs:ba,longFrame33Count:kh,longFrame50Count:ac,longFrameEvents:Y1(),frameCpu:q1(),physics:n,physicsEnvelope:{...Me},contactField:i,cameraProjectionChanges:g,heapBytes:S.endBytes,heap:S,pageState:{visibilityState:document.visibilityState,hasFocus:document.hasFocus(),focusLossCount:Hh,focusLossMs:_},scenario:s,browserAcceptance:{durationPass:a,onePercentLowPass:o,noLongFramePass:c,noVisibleGcPausePass:T,physicsBudgetPass:l,narrowPhaseAllocationPass:y,memoryStabilityPass:M,runtimeAssetPass:w,penetrationPass:h,lengthPass:u,foldPass:d,finitePass:f,modePass:m,contactFieldPass:x,cameraStablePass:p,focusPass:v,passed:a&&o&&T&&l&&y&&M&&w&&h&&u&&d&&f&&m&&x&&p&&v}}}function qh(r="manual"){return r==="ui"&&Et.automated?vh():(Et.running&&(Et.running=!1,Et.warmingUp=!1,Et.completedAt=performance.now(),Et.stopReason=r),Pe.setAutomatedBenchmarkMode?.(!1),Ra=vh(),Ra)}function jp({resetAccumulator:r=!0}={}){Ge.reset(),vc=Ge.progress,xe.reset(),De.syncFromElasticRod(fi),xe.syncXpbdBody(Ui),Zi.enabled=!1,vs.enabled=!1,_e.resetSimulationState(),r&&(ir=0)}function $p({durationMs:r=$o,automated:t=!1}={}){const e=Number(r);if(!Number.isFinite(e)||e<=0)throw new RangeError("Browser benchmark durationMs must be positive");if(!_e.contactField)throw new Error("Browser benchmark requires the precompiled vessel contact field");return jp(),Yh(),Et.durationMs=e,Et.warmupStartedAt=performance.now(),Et.memorySettling=!1,Et.startedAt=0,Et.completedAt=0,Et.simulationElapsedMs=0,Et.stopReason=null,Et.automated=t===!0,Pe.setAutomatedBenchmarkMode?.(Et.automated),Et.running=!0,Et.warmingUp=!0,Ra=null,Sc()}function Z1(r){if(!Et.running)return null;const t=performance.now();if(Et.warmingUp){const i=t-Et.warmupStartedAt;if(i<Yp){const s=of(Et.simulationElapsedMs,ms);return Et.simulationElapsedMs+=r*1e3,s}if(Et.memorySettling||(jp({resetAccumulator:!1}),Et.memorySettling=!0,Et.simulationElapsedMs=0),i<fh)return ms.guidewireAdvance=0,ms.catheterAdvance=0,ms.catheterRotation=0,ms.catheterType="pigtail",ms;Yh(),Et.warmingUp=!1,Et.memorySettling=!1,Et.startedAt=performance.now(),Et.completedAt=0,Et.simulationElapsedMs=0}if(performance.now()-Et.startedAt>=Et.durationMs)return qh("duration"),null;const n=of(Et.simulationElapsedMs,ms);return Et.simulationElapsedMs+=r*1e3,n}globalThis.__OET_BENCHMARK__={reset:Yh,getReport:vh,startScenario:$p,stopScenario:qh,getScenarioStatus:Sc,getLastScenarioReport:()=>Ra};function j1(r,t){const e=En==="legacy"?gn:null,n=Ge.advance(r,t,e,D1);return vc=Ge.progress,n}function $1(){let r=0;for(let t=0;t<fi.nodes.length-1;t++){const e=fi.nodes[t],n=fi.nodes[t+1];Lo.set(n.x-e.x,n.y-e.y,n.z-e.z);const i=Lo.length();i<1e-6||(Lo.multiplyScalar(1/i),vf.set((e.x+n.x)*.5,(e.y+n.y)*.5,(e.z+n.z)*.5),_f.setFromUnitVectors(R1,Lo),Sf.set(1,i+r1,1),xf.compose(vf,_f,Sf),Ji.setMatrixAt(r,xf),r++)}Ji.count=r,Ji.instanceMatrix.needsUpdate=!0,_r.visible=r>0}function K1(){if(!dr||!ji||En!=="xpbd-contact-v1")return{normalCount:0,branchCount:0};const r=dr.geometry.getAttribute("position"),t=ji.geometry.getAttribute("position"),e=r.array,n=t.array,i=gn.contactField,s=i?.centerline,a=i?.centerlineStride||0,o=a>0&&s?s.length/a:0;let c=ji.userData.seen;!c||c.length!==o?(c=new Uint8Array(o),ji.userData.seen=c):c.fill(0);let l=0,h=0;for(const u of[De,Ui]){if(!u)continue;const d=Math.min(u.segmentCount,u.activeEnd);for(let f=u.activeStart;f<d;f++){if(!u.wallActive[f])continue;if(l<Ms){const p=l*6,_=2.5+Math.min(4,u.wallLambda[f]*8);e[p]=u.wallX[f],e[p+1]=u.wallY[f],e[p+2]=u.wallZ[f],e[p+3]=u.wallX[f]+u.wallNormalX[f]*_,e[p+4]=u.wallY[f]+u.wallNormalY[f]*_,e[p+5]=u.wallZ[f]+u.wallNormalZ[f]*_,l++}const m=u.wallBranchId[f];if(m<0||m>=o||c[m]||h>=Ms)continue;c[m]=1;const x=m*a,g=h*6;for(let p=0;p<6;p++)n[g+p]=s[x+p];h++}}return dr.geometry.setDrawRange(0,l*2),ji.geometry.setDrawRange(0,h*2),r.needsUpdate=!0,t.needsUpdate=!0,{normalCount:l,branchCount:h}}function J1(){if(Re){Pe.updateGuidewireDiagnostics(null),wn.count=0,An.count=0,Ve.userData.hasPoint=!1,Ve.visible=!1,dr?.geometry.setDrawRange(0,0),ji?.geometry.setDrawRange(0,0);return}const r=Ge.collectLumenDiagnostics(gn,{clearance:Ge.meshClearance,contactBand:i1,collectMarkers:!0,markerLimit:Ms});if(En==="xpbd-contact-v1"){const e=_e.getStats(),n=Ge.getPerformanceStats(),i=K1();r.performance={advanceMs:n.advanceMs,solveMs:e.phases.total.lastMs,projectMs:e.phases.narrowPhase.lastMs,diagnosticMs:0,pointContactCount:e.contacts,diagnosticPointContactCount:0,segmentSampleCount:gn.contactField?.getStats?.().capsuleSamples||0,activeBranchCount:i.branchCount,settledPenetration:e.settledMaxPenetration,maximumPenetration:e.maxPenetration}}else r.performance=Ge.getPerformanceStats();Pe.updateGuidewireDiagnostics(r),r.worstPoint?(Ve.position.set(r.worstPoint.x,r.worstPoint.y+Op,r.worstPoint.z),Ve.userData.hasPoint=!0,Ve.visible=!0):(Ve.userData.hasPoint=!1,Ve.visible=!1);const t=(e,n)=>{const i=Math.min(n.length,Ms);e.count=i;for(let s=0;s<i;s++){const a=n[s];Mf.makeTranslation(a.x,a.y,a.z),e.setMatrixAt(s,Mf)}e.instanceMatrix.needsUpdate=!0};t(wn,r.contacts||[]),t(An,r.breaches||[])}function Q1(){if(En!=="xpbd-contact-v1"){Pe.updateGuidewireResistance(0,"");return}let r=0,t=0;for(let i=0;i<De.wallLambda.length;i++)De.wallActive[i]&&(r+=De.wallLambda[i],t++);const e=t?r/t:0,n=Math.max(0,Math.min(1,e/.08));Pe.updateGuidewireResistance(n,t?"Opór kontaktu prowadnika ze ścianą":"")}const nr=En==="xpbd-contact-v1"?1/120:1/60;let wf=performance.now(),ir=0,Fl=-1/0,xs=0;const Ul=new C;let Bl=Tp,zl=Cp,Ol=1/0;function tw(r){const t=mn.uniforms,e=Math.min(1,Math.max(0,r)*1.35);if(!t.autoExposureEnabled.value){xs+=(0-xs)*Math.min(1,e*1.6),t.autoExposureLevel.value=xs;return}Oe.getWorldDirection(Ul);const n=Math.abs(Ul.x),i=Math.abs(Ul.y),s=Math.max(0,n-.1),a=ue.clamp((t.collimation.value||0)/.45,0,1),o=1-a*.34,c=.012+s*.15+i*.035,l=ue.clamp(c*o-a*.006,-.03,.18);xs+=(l-xs)*e,t.autoExposureLevel.value=xs}function Af(){const r=mn.uniforms,t=ue.clamp(r.pulseRate.value||15,7.5,30),e=r.autoExposureEnabled.value?ue.clamp(xs/.18,0,1):.25,n=70+e*28,i=Math.pow(t/15,.72),a=1-ue.clamp((r.collimation.value||0)/.45,0,1)*.42,o=(2.4+e*7.2)*i*a;Pe.updateXrayTechnique(n,o)}function ew(r=nr){const t=Z1(r),e=t?.guidewireAdvance??Pe.getAdvance(),n=t?.catheterAdvance??Pe.getCatheterAdvance(),i=t?.catheterRotation??Pe.getCatheterRotation();j1(e,r);const s=Math.max(0,vc);if(xe.setType(t?.catheterType??Pe.getSelectedCatheterType()),xe.advance(n,r,s),xe.rotate(i,r),En==="xpbd-contact-v1"){De.syncFromElasticRod(fi,I1),De.setActiveRange(Math.min(De.count-2,Math.max(0,Ge.firstInsertedNodeIndex()-1)),De.count-1),De.setCollisionRange(Math.max(0,Ge.firstLumenNodeIndex()-1),De.segmentCount-1),xe.stepPhysics(r,N1);const l=xe.syncXpbdBody(Ui,Xp);Zi.outerStartNode=xe.physicsLumenStartNode;const h=Math.max(0,Math.ceil((pr-s)/Di)),u=Math.min(De.count-1,Math.floor((pr-s+xe.progress)/Di));Zi.enabled=xe.progress>.5&&l>=2&&u>=h,Zi.startNode=h,Zi.endNode=Math.max(h,u),Zi.innerArcOffset=h*Di-pr+s,Zi.containedLength=Math.min(xe.progress,s),De.nodeRadius.fill(gr);const d=Math.max(0,l-2),f=Math.max(0,Math.min(De.segmentCount-1,u));vs.enabled=!1,vs.startSegmentA=f,vs.endSegmentA=Math.min(De.activeEnd-1,f+16),vs.startSegmentB=Math.max(0,d-8),vs.endSegmentB=d,_e.stepFixed(),Et.running&&H1(),De.syncToElasticRod(fi)}else Ge.solve(r,gn,{iterations:e===0?3:4}),xe.stepPhysics(r);const a=n!==0||i!==0,o=e!==0,c=xe.progress>4&&s>0;if(En==="legacy"&&(xe.constrainGuidewire(r,{reactionScale:o&&!a?.08:1}),o&&!a&&c&&(Ge.solve(r,gn,{iterations:8,forceRelax:!0}),xe.constrainGuidewire(r,{reactionScale:.04}),Ge.solve(r,gn,{iterations:5,forceRelax:!0})),a&&(Ge.solve(r,gn,{iterations:10,forceRelax:!0}),xe.constrainGuidewire(r),Ge.solve(r,gn,{iterations:8,forceRelax:!0}))),Q1(),Pe.updateInsertedLength(s/10),Pe.updateCatheterLength(xe.progress/10),ui){const l=Math.min(dh*r,Aa);wa.injectThroughSheath(l,dh),mf+=l,Pe.updateDose(mf),uh+=r,Aa-=l,(uh>=kp||Aa<=0)&&(ui=!1,Pe.setStopInjectionDisabled(!0))}wa.update(r),C1.update(r)}const Do=[];function sr(r,t,e){Do.length=0;for(const n of r.children){if(n.isCamera)continue;!(n===e&&n.visible)&&n.visible&&(Do.push(n),n.visible=!1)}Xt.render(r,t);for(let n=0;n<Do.length;n++)Do[n].visible=!0}const Io=[],Gl=[];function nw(){Oe.updateMatrixWorld(!0);const r=Oe.matrixWorld.elements,t=Oe.projectionMatrix.elements;let e=!sc;for(let n=0;n<16&&!e;n++)e=r[n]!==uf[n]||t[n]!==df[n];return e?(uf.set(r),df.set(t),sc=!0,!0):!1}function iw(){Io.length=0,Gl.length=0;for(const r of he.children)r!==ys&&!r.isCamera&&(Io.push(r),Gl.push(r.visible),r.visible=!1);he.overrideMaterial=p1,Xt.setRenderTarget(nc),Xt.clear(),Xt.render(he,Oe),he.overrideMaterial=m1,Xt.setRenderTarget(ic),Xt.clear(),Xt.render(he,Oe),he.overrideMaterial=null,Xt.setRenderTarget(null);for(let r=0;r<Io.length;r++)Io[r].visible=Gl[r];lh.uniforms.frontDepth.value=nc.texture,lh.uniforms.backDepth.value=ic.texture,Xt.setRenderTarget(xc),Xt.render(Bp,ko),Xt.setRenderTarget(null),Xt.setRenderTarget(gc),Xt.clear(),he.overrideMaterial=x1,sr(he,Oe,ys),he.overrideMaterial=null,Xt.setRenderTarget(null)}function Sh(r){const t=performance.now(),e=r-wf,n=Math.max(0,Math.min(.1,e/1e3));wf=r,k1(e),ir+=n;let i=0;for(;ir+1e-9>=nr&&i<2;)ew(nr),ir-=nr,i++;ir>=nr&&(ir%=nr);const s=performance.now();if($1(),Bl+=n,Bl>=Tp&&(Bl=0,J1()),zl+=n,zl>=Cp&&(zl=0,xe.updateMesh()),ui||wa.hasVisibleContrast()||Il>0){Nl+=n;const l=ui?1/30:1/24;if(!Ci||Nl>=l){Nl=0;const h=UM(wa,.01,!Re,Ci);Il=h.count,h.mesh&&h.mesh!==Ci&&(Ci&&ci.remove(Ci),Ci=h.mesh,ci.add(Ci))}}else Ci&&(Ci.visible=!1);Re&&ci.parent!==oa?(he.remove(ci),oa.add(ci)):!Re&&ci.parent!==he&&(oa.remove(ci),he.add(ci));const o=Il>0||ui||wa.hasVisibleContrast();if(Tr.visible=!Re,Ss.visible=Re,wn&&(wn.visible=!Re),An&&(An.visible=!Re),Ve&&(Ve.visible=!Re&&!!Ve.userData.hasPoint),ys.visible=Re,Pe.setInjectButtonDisabled(o),Pe.setStopInjectionDisabled(!ui),Ol+=n,Ol>=.25){Ol=0;const l=Sc();Pe.updateBrowserBenchmarkStatus(l,l.running?null:Ra)}const c=performance.now();if(Re){tw(n),Af();const h=1e3/Math.max(1,mn.uniforms.pulseRate.value||15);if(!(r-Fl>=h)){Xt.setRenderTarget(null),Xt.render(hh,ko),Pe.updatePerfStats(n),Ef(t,s,c),requestAnimationFrame(Sh);return}Fl=r,nw()&&iw(),Xt.setRenderTarget(Jo),Xt.setClearColor(0,0),Xt.clear(),Xt.render(oa,Oe),Xt.setClearColor(0,1),Xt.setRenderTarget(Qo),Xt.setClearColor(0,0),Xt.clear(),he.overrideMaterial=gf,sr(he,Oe,_r),he.overrideMaterial=null,Xt.setClearColor(0,1),Xt.setRenderTarget(tc),Xt.setClearColor(0,0),Xt.clear(),sr(he,Oe,xe.mesh),Xt.setClearColor(0,1),Xt.setRenderTarget(ec),Xt.setClearColor(0,0),Xt.clear(),sr(he,Oe,Ss),Xt.setClearColor(0,1),Xt.setRenderTarget(ch),Xt.clear(),sr(he,Oe,Ss);const d=Xt.autoClear;Xt.autoClear=!1,he.overrideMaterial=gf,sr(he,Oe,_r),he.overrideMaterial=null,Xt.render(oa,Oe),Xt.autoClear=d,rc.uniforms.currentFrame.value=ch.texture,rc.uniforms.previousFrame.value=Vo.texture,Xt.setRenderTarget(Po),Xt.render(Fp,ko),Xt.setRenderTarget(null),mn.uniforms.uTexture.value=Po.texture,mn.uniforms.contrastTexture.value=Jo.texture,mn.uniforms.thicknessTexture.value=xc.texture,mn.uniforms.metalTexture.value=Qo.texture,mn.uniforms.catheterTexture.value=tc.texture,mn.uniforms.sheathTexture.value=ec.texture,mn.uniforms.boneTexture.value=gc.texture,mn.uniforms.time.value=r*.001,Xt.render(hh,ko),hf();const f=Vo;Vo=Po,Po=f}else Fl=-1/0,Af(),Xt.setRenderTarget(null),Xt.render(he,Oe),hf();Pe.updatePerfStats(n),Ef(t,s,c),requestAnimationFrame(Sh)}requestAnimationFrame(Sh);window.addEventListener("resize",()=>{const r=window.innerWidth,t=window.innerHeight,e=Math.max(1,Math.round(r*Ko)),n=Math.max(1,Math.round(t*Ko));Xt.setSize(r,t),Oe.aspect=r/t,Oe.updateProjectionMatrix(),ch.setSize(e,n),Jo.setSize(e,n),Qo.setSize(e,n),tc.setSize(e,n),ec.setSize(e,n),gc.setSize(e,n),Ip.setSize(e,n),Np.setSize(e,n),nc.setSize(e,n),ic.setSize(e,n),xc.setSize(e,n),sc=!1,mn.uniforms.resolution.value.set(e,n)});
