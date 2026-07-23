(function(){const t=document.createElement("link").relList;if(t&&t.supports&&t.supports("modulepreload"))return;for(const i of document.querySelectorAll('link[rel="modulepreload"]'))n(i);new MutationObserver(i=>{for(const s of i)if(s.type==="childList")for(const a of s.addedNodes)a.tagName==="LINK"&&a.rel==="modulepreload"&&n(a)}).observe(document,{childList:!0,subtree:!0});function e(i){const s={};return i.integrity&&(s.integrity=i.integrity),i.referrerPolicy&&(s.referrerPolicy=i.referrerPolicy),i.crossOrigin==="use-credentials"?s.credentials="include":i.crossOrigin==="anonymous"?s.credentials="omit":s.credentials="same-origin",s}function n(i){if(i.ep)return;i.ep=!0;const s=e(i);fetch(i.href,s)}})();const Bc="160",Ag=0,Vu=1,Tg=2,qp=1,bg=2,Ni=3,oi=0,dn=1,Ke=2,os=0,Tr=1,gc=2,ku=3,Hu=4,Zp=5,Bi=100,Cg=101,Rg=102,Wu=103,Xu=104,Pg=200,Lo=201,Lg=202,Ig=203,wh=204,Ah=205,Dg=206,Ng=207,Fg=208,Ug=209,zg=210,Bg=211,Og=212,Gg=213,Vg=214,kg=0,Hg=1,Wg=2,xc=3,Xg=4,Yg=5,qg=6,Zg=7,lu=0,jg=1,$g=2,as=0,Kg=1,Jg=2,Qg=3,t0=4,e0=5,n0=6,jp=300,Ir=301,Dr=302,Th=303,bh=304,Oc=306,Ch=1e3,si=1001,Rh=1002,_n=1003,Yu=1004,Qc=1005,Vn=1006,i0=1007,$o=1008,cs=1009,s0=1010,r0=1011,hu=1012,$p=1013,ns=1014,is=1015,Nr=1016,Kp=1017,Jp=1018,Is=1020,o0=1021,ri=1023,a0=1024,c0=1025,Ds=1026,Fr=1027,l0=1028,Qp=1029,h0=1030,tm=1031,em=1033,tl=33776,el=33777,nl=33778,il=33779,qu=35840,Zu=35841,ju=35842,$u=35843,nm=36196,Ku=37492,Ju=37496,Qu=37808,td=37809,ed=37810,nd=37811,id=37812,sd=37813,rd=37814,od=37815,ad=37816,cd=37817,ld=37818,hd=37819,ud=37820,dd=37821,sl=36492,fd=36494,pd=36495,u0=36283,md=36284,gd=36285,xd=36286,im=3e3,Ns=3001,d0=3200,f0=3201,uu=0,p0=1,kn="",sn="srgb",Vi="srgb-linear",du="display-p3",Gc="display-p3-linear",_c="linear",Se="srgb",vc="rec709",Sc="p3",ks=7680,_d=519,m0=512,g0=513,x0=514,sm=515,_0=516,v0=517,S0=518,M0=519,vd=35044,Vc=35048,Sd="300 es",Ph=1035,Oi=2e3,Mc=2001;class Or{addEventListener(t,e){this._listeners===void 0&&(this._listeners={});const n=this._listeners;n[t]===void 0&&(n[t]=[]),n[t].indexOf(e)===-1&&n[t].push(e)}hasEventListener(t,e){if(this._listeners===void 0)return!1;const n=this._listeners;return n[t]!==void 0&&n[t].indexOf(e)!==-1}removeEventListener(t,e){if(this._listeners===void 0)return;const i=this._listeners[t];if(i!==void 0){const s=i.indexOf(e);s!==-1&&i.splice(s,1)}}dispatchEvent(t){if(this._listeners===void 0)return;const n=this._listeners[t.type];if(n!==void 0){t.target=this;const i=n.slice(0);for(let s=0,a=i.length;s<a;s++)i[s].call(this,t);t.target=null}}}const on=["00","01","02","03","04","05","06","07","08","09","0a","0b","0c","0d","0e","0f","10","11","12","13","14","15","16","17","18","19","1a","1b","1c","1d","1e","1f","20","21","22","23","24","25","26","27","28","29","2a","2b","2c","2d","2e","2f","30","31","32","33","34","35","36","37","38","39","3a","3b","3c","3d","3e","3f","40","41","42","43","44","45","46","47","48","49","4a","4b","4c","4d","4e","4f","50","51","52","53","54","55","56","57","58","59","5a","5b","5c","5d","5e","5f","60","61","62","63","64","65","66","67","68","69","6a","6b","6c","6d","6e","6f","70","71","72","73","74","75","76","77","78","79","7a","7b","7c","7d","7e","7f","80","81","82","83","84","85","86","87","88","89","8a","8b","8c","8d","8e","8f","90","91","92","93","94","95","96","97","98","99","9a","9b","9c","9d","9e","9f","a0","a1","a2","a3","a4","a5","a6","a7","a8","a9","aa","ab","ac","ad","ae","af","b0","b1","b2","b3","b4","b5","b6","b7","b8","b9","ba","bb","bc","bd","be","bf","c0","c1","c2","c3","c4","c5","c6","c7","c8","c9","ca","cb","cc","cd","ce","cf","d0","d1","d2","d3","d4","d5","d6","d7","d8","d9","da","db","dc","dd","de","df","e0","e1","e2","e3","e4","e5","e6","e7","e8","e9","ea","eb","ec","ed","ee","ef","f0","f1","f2","f3","f4","f5","f6","f7","f8","f9","fa","fb","fc","fd","fe","ff"];let Md=1234567;const Oo=Math.PI/180,Ko=180/Math.PI;function Gr(){const r=Math.random()*4294967295|0,t=Math.random()*4294967295|0,e=Math.random()*4294967295|0,n=Math.random()*4294967295|0;return(on[r&255]+on[r>>8&255]+on[r>>16&255]+on[r>>24&255]+"-"+on[t&255]+on[t>>8&255]+"-"+on[t>>16&15|64]+on[t>>24&255]+"-"+on[e&63|128]+on[e>>8&255]+"-"+on[e>>16&255]+on[e>>24&255]+on[n&255]+on[n>>8&255]+on[n>>16&255]+on[n>>24&255]).toLowerCase()}function Ge(r,t,e){return Math.max(t,Math.min(e,r))}function fu(r,t){return(r%t+t)%t}function y0(r,t,e,n,i){return n+(r-t)*(i-n)/(e-t)}function E0(r,t,e){return r!==t?(e-r)/(t-r):0}function Go(r,t,e){return(1-e)*r+e*t}function w0(r,t,e,n){return Go(r,t,1-Math.exp(-e*n))}function A0(r,t=1){return t-Math.abs(fu(r,t*2)-t)}function T0(r,t,e){return r<=t?0:r>=e?1:(r=(r-t)/(e-t),r*r*(3-2*r))}function b0(r,t,e){return r<=t?0:r>=e?1:(r=(r-t)/(e-t),r*r*r*(r*(r*6-15)+10))}function C0(r,t){return r+Math.floor(Math.random()*(t-r+1))}function R0(r,t){return r+Math.random()*(t-r)}function P0(r){return r*(.5-Math.random())}function L0(r){r!==void 0&&(Md=r);let t=Md+=1831565813;return t=Math.imul(t^t>>>15,t|1),t^=t+Math.imul(t^t>>>7,t|61),((t^t>>>14)>>>0)/4294967296}function I0(r){return r*Oo}function D0(r){return r*Ko}function Lh(r){return(r&r-1)===0&&r!==0}function N0(r){return Math.pow(2,Math.ceil(Math.log(r)/Math.LN2))}function yc(r){return Math.pow(2,Math.floor(Math.log(r)/Math.LN2))}function F0(r,t,e,n,i){const s=Math.cos,a=Math.sin,o=s(e/2),c=a(e/2),l=s((t+n)/2),h=a((t+n)/2),u=s((t-n)/2),f=a((t-n)/2),d=s((n-t)/2),g=a((n-t)/2);switch(i){case"XYX":r.set(o*h,c*u,c*f,o*l);break;case"YZY":r.set(c*f,o*h,c*u,o*l);break;case"ZXZ":r.set(c*u,c*f,o*h,o*l);break;case"XZX":r.set(o*h,c*g,c*d,o*l);break;case"YXY":r.set(c*d,o*h,c*g,o*l);break;case"ZYZ":r.set(c*g,c*d,o*h,o*l);break;default:console.warn("THREE.MathUtils: .setQuaternionFromProperEuler() encountered an unknown order: "+i)}}function gr(r,t){switch(t.constructor){case Float32Array:return r;case Uint32Array:return r/4294967295;case Uint16Array:return r/65535;case Uint8Array:return r/255;case Int32Array:return Math.max(r/2147483647,-1);case Int16Array:return Math.max(r/32767,-1);case Int8Array:return Math.max(r/127,-1);default:throw new Error("Invalid component type.")}}function mn(r,t){switch(t.constructor){case Float32Array:return r;case Uint32Array:return Math.round(r*4294967295);case Uint16Array:return Math.round(r*65535);case Uint8Array:return Math.round(r*255);case Int32Array:return Math.round(r*2147483647);case Int16Array:return Math.round(r*32767);case Int8Array:return Math.round(r*127);default:throw new Error("Invalid component type.")}}const he={DEG2RAD:Oo,RAD2DEG:Ko,generateUUID:Gr,clamp:Ge,euclideanModulo:fu,mapLinear:y0,inverseLerp:E0,lerp:Go,damp:w0,pingpong:A0,smoothstep:T0,smootherstep:b0,randInt:C0,randFloat:R0,randFloatSpread:P0,seededRandom:L0,degToRad:I0,radToDeg:D0,isPowerOfTwo:Lh,ceilPowerOfTwo:N0,floorPowerOfTwo:yc,setQuaternionFromProperEuler:F0,normalize:mn,denormalize:gr};class St{constructor(t=0,e=0){St.prototype.isVector2=!0,this.x=t,this.y=e}get width(){return this.x}set width(t){this.x=t}get height(){return this.y}set height(t){this.y=t}set(t,e){return this.x=t,this.y=e,this}setScalar(t){return this.x=t,this.y=t,this}setX(t){return this.x=t,this}setY(t){return this.y=t,this}setComponent(t,e){switch(t){case 0:this.x=e;break;case 1:this.y=e;break;default:throw new Error("index is out of range: "+t)}return this}getComponent(t){switch(t){case 0:return this.x;case 1:return this.y;default:throw new Error("index is out of range: "+t)}}clone(){return new this.constructor(this.x,this.y)}copy(t){return this.x=t.x,this.y=t.y,this}add(t){return this.x+=t.x,this.y+=t.y,this}addScalar(t){return this.x+=t,this.y+=t,this}addVectors(t,e){return this.x=t.x+e.x,this.y=t.y+e.y,this}addScaledVector(t,e){return this.x+=t.x*e,this.y+=t.y*e,this}sub(t){return this.x-=t.x,this.y-=t.y,this}subScalar(t){return this.x-=t,this.y-=t,this}subVectors(t,e){return this.x=t.x-e.x,this.y=t.y-e.y,this}multiply(t){return this.x*=t.x,this.y*=t.y,this}multiplyScalar(t){return this.x*=t,this.y*=t,this}divide(t){return this.x/=t.x,this.y/=t.y,this}divideScalar(t){return this.multiplyScalar(1/t)}applyMatrix3(t){const e=this.x,n=this.y,i=t.elements;return this.x=i[0]*e+i[3]*n+i[6],this.y=i[1]*e+i[4]*n+i[7],this}min(t){return this.x=Math.min(this.x,t.x),this.y=Math.min(this.y,t.y),this}max(t){return this.x=Math.max(this.x,t.x),this.y=Math.max(this.y,t.y),this}clamp(t,e){return this.x=Math.max(t.x,Math.min(e.x,this.x)),this.y=Math.max(t.y,Math.min(e.y,this.y)),this}clampScalar(t,e){return this.x=Math.max(t,Math.min(e,this.x)),this.y=Math.max(t,Math.min(e,this.y)),this}clampLength(t,e){const n=this.length();return this.divideScalar(n||1).multiplyScalar(Math.max(t,Math.min(e,n)))}floor(){return this.x=Math.floor(this.x),this.y=Math.floor(this.y),this}ceil(){return this.x=Math.ceil(this.x),this.y=Math.ceil(this.y),this}round(){return this.x=Math.round(this.x),this.y=Math.round(this.y),this}roundToZero(){return this.x=Math.trunc(this.x),this.y=Math.trunc(this.y),this}negate(){return this.x=-this.x,this.y=-this.y,this}dot(t){return this.x*t.x+this.y*t.y}cross(t){return this.x*t.y-this.y*t.x}lengthSq(){return this.x*this.x+this.y*this.y}length(){return Math.sqrt(this.x*this.x+this.y*this.y)}manhattanLength(){return Math.abs(this.x)+Math.abs(this.y)}normalize(){return this.divideScalar(this.length()||1)}angle(){return Math.atan2(-this.y,-this.x)+Math.PI}angleTo(t){const e=Math.sqrt(this.lengthSq()*t.lengthSq());if(e===0)return Math.PI/2;const n=this.dot(t)/e;return Math.acos(Ge(n,-1,1))}distanceTo(t){return Math.sqrt(this.distanceToSquared(t))}distanceToSquared(t){const e=this.x-t.x,n=this.y-t.y;return e*e+n*n}manhattanDistanceTo(t){return Math.abs(this.x-t.x)+Math.abs(this.y-t.y)}setLength(t){return this.normalize().multiplyScalar(t)}lerp(t,e){return this.x+=(t.x-this.x)*e,this.y+=(t.y-this.y)*e,this}lerpVectors(t,e,n){return this.x=t.x+(e.x-t.x)*n,this.y=t.y+(e.y-t.y)*n,this}equals(t){return t.x===this.x&&t.y===this.y}fromArray(t,e=0){return this.x=t[e],this.y=t[e+1],this}toArray(t=[],e=0){return t[e]=this.x,t[e+1]=this.y,t}fromBufferAttribute(t,e){return this.x=t.getX(e),this.y=t.getY(e),this}rotateAround(t,e){const n=Math.cos(e),i=Math.sin(e),s=this.x-t.x,a=this.y-t.y;return this.x=s*n-a*i+t.x,this.y=s*i+a*n+t.y,this}random(){return this.x=Math.random(),this.y=Math.random(),this}*[Symbol.iterator](){yield this.x,yield this.y}}class ne{constructor(t,e,n,i,s,a,o,c,l){ne.prototype.isMatrix3=!0,this.elements=[1,0,0,0,1,0,0,0,1],t!==void 0&&this.set(t,e,n,i,s,a,o,c,l)}set(t,e,n,i,s,a,o,c,l){const h=this.elements;return h[0]=t,h[1]=i,h[2]=o,h[3]=e,h[4]=s,h[5]=c,h[6]=n,h[7]=a,h[8]=l,this}identity(){return this.set(1,0,0,0,1,0,0,0,1),this}copy(t){const e=this.elements,n=t.elements;return e[0]=n[0],e[1]=n[1],e[2]=n[2],e[3]=n[3],e[4]=n[4],e[5]=n[5],e[6]=n[6],e[7]=n[7],e[8]=n[8],this}extractBasis(t,e,n){return t.setFromMatrix3Column(this,0),e.setFromMatrix3Column(this,1),n.setFromMatrix3Column(this,2),this}setFromMatrix4(t){const e=t.elements;return this.set(e[0],e[4],e[8],e[1],e[5],e[9],e[2],e[6],e[10]),this}multiply(t){return this.multiplyMatrices(this,t)}premultiply(t){return this.multiplyMatrices(t,this)}multiplyMatrices(t,e){const n=t.elements,i=e.elements,s=this.elements,a=n[0],o=n[3],c=n[6],l=n[1],h=n[4],u=n[7],f=n[2],d=n[5],g=n[8],x=i[0],m=i[3],p=i[6],_=i[1],v=i[4],S=i[7],y=i[2],M=i[5],w=i[8];return s[0]=a*x+o*_+c*y,s[3]=a*m+o*v+c*M,s[6]=a*p+o*S+c*w,s[1]=l*x+h*_+u*y,s[4]=l*m+h*v+u*M,s[7]=l*p+h*S+u*w,s[2]=f*x+d*_+g*y,s[5]=f*m+d*v+g*M,s[8]=f*p+d*S+g*w,this}multiplyScalar(t){const e=this.elements;return e[0]*=t,e[3]*=t,e[6]*=t,e[1]*=t,e[4]*=t,e[7]*=t,e[2]*=t,e[5]*=t,e[8]*=t,this}determinant(){const t=this.elements,e=t[0],n=t[1],i=t[2],s=t[3],a=t[4],o=t[5],c=t[6],l=t[7],h=t[8];return e*a*h-e*o*l-n*s*h+n*o*c+i*s*l-i*a*c}invert(){const t=this.elements,e=t[0],n=t[1],i=t[2],s=t[3],a=t[4],o=t[5],c=t[6],l=t[7],h=t[8],u=h*a-o*l,f=o*c-h*s,d=l*s-a*c,g=e*u+n*f+i*d;if(g===0)return this.set(0,0,0,0,0,0,0,0,0);const x=1/g;return t[0]=u*x,t[1]=(i*l-h*n)*x,t[2]=(o*n-i*a)*x,t[3]=f*x,t[4]=(h*e-i*c)*x,t[5]=(i*s-o*e)*x,t[6]=d*x,t[7]=(n*c-l*e)*x,t[8]=(a*e-n*s)*x,this}transpose(){let t;const e=this.elements;return t=e[1],e[1]=e[3],e[3]=t,t=e[2],e[2]=e[6],e[6]=t,t=e[5],e[5]=e[7],e[7]=t,this}getNormalMatrix(t){return this.setFromMatrix4(t).invert().transpose()}transposeIntoArray(t){const e=this.elements;return t[0]=e[0],t[1]=e[3],t[2]=e[6],t[3]=e[1],t[4]=e[4],t[5]=e[7],t[6]=e[2],t[7]=e[5],t[8]=e[8],this}setUvTransform(t,e,n,i,s,a,o){const c=Math.cos(s),l=Math.sin(s);return this.set(n*c,n*l,-n*(c*a+l*o)+a+t,-i*l,i*c,-i*(-l*a+c*o)+o+e,0,0,1),this}scale(t,e){return this.premultiply(rl.makeScale(t,e)),this}rotate(t){return this.premultiply(rl.makeRotation(-t)),this}translate(t,e){return this.premultiply(rl.makeTranslation(t,e)),this}makeTranslation(t,e){return t.isVector2?this.set(1,0,t.x,0,1,t.y,0,0,1):this.set(1,0,t,0,1,e,0,0,1),this}makeRotation(t){const e=Math.cos(t),n=Math.sin(t);return this.set(e,-n,0,n,e,0,0,0,1),this}makeScale(t,e){return this.set(t,0,0,0,e,0,0,0,1),this}equals(t){const e=this.elements,n=t.elements;for(let i=0;i<9;i++)if(e[i]!==n[i])return!1;return!0}fromArray(t,e=0){for(let n=0;n<9;n++)this.elements[n]=t[n+e];return this}toArray(t=[],e=0){const n=this.elements;return t[e]=n[0],t[e+1]=n[1],t[e+2]=n[2],t[e+3]=n[3],t[e+4]=n[4],t[e+5]=n[5],t[e+6]=n[6],t[e+7]=n[7],t[e+8]=n[8],t}clone(){return new this.constructor().fromArray(this.elements)}}const rl=new ne;function rm(r){for(let t=r.length-1;t>=0;--t)if(r[t]>=65535)return!0;return!1}function Ec(r){return document.createElementNS("http://www.w3.org/1999/xhtml",r)}function U0(){const r=Ec("canvas");return r.style.display="block",r}const yd={};function Vo(r){r in yd||(yd[r]=!0,console.warn(r))}const Ed=new ne().set(.8224621,.177538,0,.0331941,.9668058,0,.0170827,.0723974,.9105199),wd=new ne().set(1.2249401,-.2249404,0,-.0420569,1.0420571,0,-.0196376,-.0786361,1.0982735),ra={[Vi]:{transfer:_c,primaries:vc,toReference:r=>r,fromReference:r=>r},[sn]:{transfer:Se,primaries:vc,toReference:r=>r.convertSRGBToLinear(),fromReference:r=>r.convertLinearToSRGB()},[Gc]:{transfer:_c,primaries:Sc,toReference:r=>r.applyMatrix3(wd),fromReference:r=>r.applyMatrix3(Ed)},[du]:{transfer:Se,primaries:Sc,toReference:r=>r.convertSRGBToLinear().applyMatrix3(wd),fromReference:r=>r.applyMatrix3(Ed).convertLinearToSRGB()}},z0=new Set([Vi,Gc]),me={enabled:!0,_workingColorSpace:Vi,get workingColorSpace(){return this._workingColorSpace},set workingColorSpace(r){if(!z0.has(r))throw new Error(`Unsupported working color space, "${r}".`);this._workingColorSpace=r},convert:function(r,t,e){if(this.enabled===!1||t===e||!t||!e)return r;const n=ra[t].toReference,i=ra[e].fromReference;return i(n(r))},fromWorkingColorSpace:function(r,t){return this.convert(r,this._workingColorSpace,t)},toWorkingColorSpace:function(r,t){return this.convert(r,t,this._workingColorSpace)},getPrimaries:function(r){return ra[r].primaries},getTransfer:function(r){return r===kn?_c:ra[r].transfer}};function br(r){return r<.04045?r*.0773993808:Math.pow(r*.9478672986+.0521327014,2.4)}function ol(r){return r<.0031308?r*12.92:1.055*Math.pow(r,.41666)-.055}let Hs;class om{static getDataURL(t){if(/^data:/i.test(t.src)||typeof HTMLCanvasElement>"u")return t.src;let e;if(t instanceof HTMLCanvasElement)e=t;else{Hs===void 0&&(Hs=Ec("canvas")),Hs.width=t.width,Hs.height=t.height;const n=Hs.getContext("2d");t instanceof ImageData?n.putImageData(t,0,0):n.drawImage(t,0,0,t.width,t.height),e=Hs}return e.width>2048||e.height>2048?(console.warn("THREE.ImageUtils.getDataURL: Image converted to jpg for performance reasons",t),e.toDataURL("image/jpeg",.6)):e.toDataURL("image/png")}static sRGBToLinear(t){if(typeof HTMLImageElement<"u"&&t instanceof HTMLImageElement||typeof HTMLCanvasElement<"u"&&t instanceof HTMLCanvasElement||typeof ImageBitmap<"u"&&t instanceof ImageBitmap){const e=Ec("canvas");e.width=t.width,e.height=t.height;const n=e.getContext("2d");n.drawImage(t,0,0,t.width,t.height);const i=n.getImageData(0,0,t.width,t.height),s=i.data;for(let a=0;a<s.length;a++)s[a]=br(s[a]/255)*255;return n.putImageData(i,0,0),e}else if(t.data){const e=t.data.slice(0);for(let n=0;n<e.length;n++)e instanceof Uint8Array||e instanceof Uint8ClampedArray?e[n]=Math.floor(br(e[n]/255)*255):e[n]=br(e[n]);return{data:e,width:t.width,height:t.height}}else return console.warn("THREE.ImageUtils.sRGBToLinear(): Unsupported image type. No color space conversion applied."),t}}let B0=0;class am{constructor(t=null){this.isSource=!0,Object.defineProperty(this,"id",{value:B0++}),this.uuid=Gr(),this.data=t,this.version=0}set needsUpdate(t){t===!0&&this.version++}toJSON(t){const e=t===void 0||typeof t=="string";if(!e&&t.images[this.uuid]!==void 0)return t.images[this.uuid];const n={uuid:this.uuid,url:""},i=this.data;if(i!==null){let s;if(Array.isArray(i)){s=[];for(let a=0,o=i.length;a<o;a++)i[a].isDataTexture?s.push(al(i[a].image)):s.push(al(i[a]))}else s=al(i);n.url=s}return e||(t.images[this.uuid]=n),n}}function al(r){return typeof HTMLImageElement<"u"&&r instanceof HTMLImageElement||typeof HTMLCanvasElement<"u"&&r instanceof HTMLCanvasElement||typeof ImageBitmap<"u"&&r instanceof ImageBitmap?om.getDataURL(r):r.data?{data:Array.from(r.data),width:r.width,height:r.height,type:r.data.constructor.name}:(console.warn("THREE.Texture: Unable to serialize Texture."),{})}let O0=0;class Dn extends Or{constructor(t=Dn.DEFAULT_IMAGE,e=Dn.DEFAULT_MAPPING,n=si,i=si,s=Vn,a=$o,o=ri,c=cs,l=Dn.DEFAULT_ANISOTROPY,h=kn){super(),this.isTexture=!0,Object.defineProperty(this,"id",{value:O0++}),this.uuid=Gr(),this.name="",this.source=new am(t),this.mipmaps=[],this.mapping=e,this.channel=0,this.wrapS=n,this.wrapT=i,this.magFilter=s,this.minFilter=a,this.anisotropy=l,this.format=o,this.internalFormat=null,this.type=c,this.offset=new St(0,0),this.repeat=new St(1,1),this.center=new St(0,0),this.rotation=0,this.matrixAutoUpdate=!0,this.matrix=new ne,this.generateMipmaps=!0,this.premultiplyAlpha=!1,this.flipY=!0,this.unpackAlignment=4,typeof h=="string"?this.colorSpace=h:(Vo("THREE.Texture: Property .encoding has been replaced by .colorSpace."),this.colorSpace=h===Ns?sn:kn),this.userData={},this.version=0,this.onUpdate=null,this.isRenderTargetTexture=!1,this.needsPMREMUpdate=!1}get image(){return this.source.data}set image(t=null){this.source.data=t}updateMatrix(){this.matrix.setUvTransform(this.offset.x,this.offset.y,this.repeat.x,this.repeat.y,this.rotation,this.center.x,this.center.y)}clone(){return new this.constructor().copy(this)}copy(t){return this.name=t.name,this.source=t.source,this.mipmaps=t.mipmaps.slice(0),this.mapping=t.mapping,this.channel=t.channel,this.wrapS=t.wrapS,this.wrapT=t.wrapT,this.magFilter=t.magFilter,this.minFilter=t.minFilter,this.anisotropy=t.anisotropy,this.format=t.format,this.internalFormat=t.internalFormat,this.type=t.type,this.offset.copy(t.offset),this.repeat.copy(t.repeat),this.center.copy(t.center),this.rotation=t.rotation,this.matrixAutoUpdate=t.matrixAutoUpdate,this.matrix.copy(t.matrix),this.generateMipmaps=t.generateMipmaps,this.premultiplyAlpha=t.premultiplyAlpha,this.flipY=t.flipY,this.unpackAlignment=t.unpackAlignment,this.colorSpace=t.colorSpace,this.userData=JSON.parse(JSON.stringify(t.userData)),this.needsUpdate=!0,this}toJSON(t){const e=t===void 0||typeof t=="string";if(!e&&t.textures[this.uuid]!==void 0)return t.textures[this.uuid];const n={metadata:{version:4.6,type:"Texture",generator:"Texture.toJSON"},uuid:this.uuid,name:this.name,image:this.source.toJSON(t).uuid,mapping:this.mapping,channel:this.channel,repeat:[this.repeat.x,this.repeat.y],offset:[this.offset.x,this.offset.y],center:[this.center.x,this.center.y],rotation:this.rotation,wrap:[this.wrapS,this.wrapT],format:this.format,internalFormat:this.internalFormat,type:this.type,colorSpace:this.colorSpace,minFilter:this.minFilter,magFilter:this.magFilter,anisotropy:this.anisotropy,flipY:this.flipY,generateMipmaps:this.generateMipmaps,premultiplyAlpha:this.premultiplyAlpha,unpackAlignment:this.unpackAlignment};return Object.keys(this.userData).length>0&&(n.userData=this.userData),e||(t.textures[this.uuid]=n),n}dispose(){this.dispatchEvent({type:"dispose"})}transformUv(t){if(this.mapping!==jp)return t;if(t.applyMatrix3(this.matrix),t.x<0||t.x>1)switch(this.wrapS){case Ch:t.x=t.x-Math.floor(t.x);break;case si:t.x=t.x<0?0:1;break;case Rh:Math.abs(Math.floor(t.x)%2)===1?t.x=Math.ceil(t.x)-t.x:t.x=t.x-Math.floor(t.x);break}if(t.y<0||t.y>1)switch(this.wrapT){case Ch:t.y=t.y-Math.floor(t.y);break;case si:t.y=t.y<0?0:1;break;case Rh:Math.abs(Math.floor(t.y)%2)===1?t.y=Math.ceil(t.y)-t.y:t.y=t.y-Math.floor(t.y);break}return this.flipY&&(t.y=1-t.y),t}set needsUpdate(t){t===!0&&(this.version++,this.source.needsUpdate=!0)}get encoding(){return Vo("THREE.Texture: Property .encoding has been replaced by .colorSpace."),this.colorSpace===sn?Ns:im}set encoding(t){Vo("THREE.Texture: Property .encoding has been replaced by .colorSpace."),this.colorSpace=t===Ns?sn:kn}}Dn.DEFAULT_IMAGE=null;Dn.DEFAULT_MAPPING=jp;Dn.DEFAULT_ANISOTROPY=1;class Je{constructor(t=0,e=0,n=0,i=1){Je.prototype.isVector4=!0,this.x=t,this.y=e,this.z=n,this.w=i}get width(){return this.z}set width(t){this.z=t}get height(){return this.w}set height(t){this.w=t}set(t,e,n,i){return this.x=t,this.y=e,this.z=n,this.w=i,this}setScalar(t){return this.x=t,this.y=t,this.z=t,this.w=t,this}setX(t){return this.x=t,this}setY(t){return this.y=t,this}setZ(t){return this.z=t,this}setW(t){return this.w=t,this}setComponent(t,e){switch(t){case 0:this.x=e;break;case 1:this.y=e;break;case 2:this.z=e;break;case 3:this.w=e;break;default:throw new Error("index is out of range: "+t)}return this}getComponent(t){switch(t){case 0:return this.x;case 1:return this.y;case 2:return this.z;case 3:return this.w;default:throw new Error("index is out of range: "+t)}}clone(){return new this.constructor(this.x,this.y,this.z,this.w)}copy(t){return this.x=t.x,this.y=t.y,this.z=t.z,this.w=t.w!==void 0?t.w:1,this}add(t){return this.x+=t.x,this.y+=t.y,this.z+=t.z,this.w+=t.w,this}addScalar(t){return this.x+=t,this.y+=t,this.z+=t,this.w+=t,this}addVectors(t,e){return this.x=t.x+e.x,this.y=t.y+e.y,this.z=t.z+e.z,this.w=t.w+e.w,this}addScaledVector(t,e){return this.x+=t.x*e,this.y+=t.y*e,this.z+=t.z*e,this.w+=t.w*e,this}sub(t){return this.x-=t.x,this.y-=t.y,this.z-=t.z,this.w-=t.w,this}subScalar(t){return this.x-=t,this.y-=t,this.z-=t,this.w-=t,this}subVectors(t,e){return this.x=t.x-e.x,this.y=t.y-e.y,this.z=t.z-e.z,this.w=t.w-e.w,this}multiply(t){return this.x*=t.x,this.y*=t.y,this.z*=t.z,this.w*=t.w,this}multiplyScalar(t){return this.x*=t,this.y*=t,this.z*=t,this.w*=t,this}applyMatrix4(t){const e=this.x,n=this.y,i=this.z,s=this.w,a=t.elements;return this.x=a[0]*e+a[4]*n+a[8]*i+a[12]*s,this.y=a[1]*e+a[5]*n+a[9]*i+a[13]*s,this.z=a[2]*e+a[6]*n+a[10]*i+a[14]*s,this.w=a[3]*e+a[7]*n+a[11]*i+a[15]*s,this}divideScalar(t){return this.multiplyScalar(1/t)}setAxisAngleFromQuaternion(t){this.w=2*Math.acos(t.w);const e=Math.sqrt(1-t.w*t.w);return e<1e-4?(this.x=1,this.y=0,this.z=0):(this.x=t.x/e,this.y=t.y/e,this.z=t.z/e),this}setAxisAngleFromRotationMatrix(t){let e,n,i,s;const c=t.elements,l=c[0],h=c[4],u=c[8],f=c[1],d=c[5],g=c[9],x=c[2],m=c[6],p=c[10];if(Math.abs(h-f)<.01&&Math.abs(u-x)<.01&&Math.abs(g-m)<.01){if(Math.abs(h+f)<.1&&Math.abs(u+x)<.1&&Math.abs(g+m)<.1&&Math.abs(l+d+p-3)<.1)return this.set(1,0,0,0),this;e=Math.PI;const v=(l+1)/2,S=(d+1)/2,y=(p+1)/2,M=(h+f)/4,w=(u+x)/4,T=(g+m)/4;return v>S&&v>y?v<.01?(n=0,i=.707106781,s=.707106781):(n=Math.sqrt(v),i=M/n,s=w/n):S>y?S<.01?(n=.707106781,i=0,s=.707106781):(i=Math.sqrt(S),n=M/i,s=T/i):y<.01?(n=.707106781,i=.707106781,s=0):(s=Math.sqrt(y),n=w/s,i=T/s),this.set(n,i,s,e),this}let _=Math.sqrt((m-g)*(m-g)+(u-x)*(u-x)+(f-h)*(f-h));return Math.abs(_)<.001&&(_=1),this.x=(m-g)/_,this.y=(u-x)/_,this.z=(f-h)/_,this.w=Math.acos((l+d+p-1)/2),this}min(t){return this.x=Math.min(this.x,t.x),this.y=Math.min(this.y,t.y),this.z=Math.min(this.z,t.z),this.w=Math.min(this.w,t.w),this}max(t){return this.x=Math.max(this.x,t.x),this.y=Math.max(this.y,t.y),this.z=Math.max(this.z,t.z),this.w=Math.max(this.w,t.w),this}clamp(t,e){return this.x=Math.max(t.x,Math.min(e.x,this.x)),this.y=Math.max(t.y,Math.min(e.y,this.y)),this.z=Math.max(t.z,Math.min(e.z,this.z)),this.w=Math.max(t.w,Math.min(e.w,this.w)),this}clampScalar(t,e){return this.x=Math.max(t,Math.min(e,this.x)),this.y=Math.max(t,Math.min(e,this.y)),this.z=Math.max(t,Math.min(e,this.z)),this.w=Math.max(t,Math.min(e,this.w)),this}clampLength(t,e){const n=this.length();return this.divideScalar(n||1).multiplyScalar(Math.max(t,Math.min(e,n)))}floor(){return this.x=Math.floor(this.x),this.y=Math.floor(this.y),this.z=Math.floor(this.z),this.w=Math.floor(this.w),this}ceil(){return this.x=Math.ceil(this.x),this.y=Math.ceil(this.y),this.z=Math.ceil(this.z),this.w=Math.ceil(this.w),this}round(){return this.x=Math.round(this.x),this.y=Math.round(this.y),this.z=Math.round(this.z),this.w=Math.round(this.w),this}roundToZero(){return this.x=Math.trunc(this.x),this.y=Math.trunc(this.y),this.z=Math.trunc(this.z),this.w=Math.trunc(this.w),this}negate(){return this.x=-this.x,this.y=-this.y,this.z=-this.z,this.w=-this.w,this}dot(t){return this.x*t.x+this.y*t.y+this.z*t.z+this.w*t.w}lengthSq(){return this.x*this.x+this.y*this.y+this.z*this.z+this.w*this.w}length(){return Math.sqrt(this.x*this.x+this.y*this.y+this.z*this.z+this.w*this.w)}manhattanLength(){return Math.abs(this.x)+Math.abs(this.y)+Math.abs(this.z)+Math.abs(this.w)}normalize(){return this.divideScalar(this.length()||1)}setLength(t){return this.normalize().multiplyScalar(t)}lerp(t,e){return this.x+=(t.x-this.x)*e,this.y+=(t.y-this.y)*e,this.z+=(t.z-this.z)*e,this.w+=(t.w-this.w)*e,this}lerpVectors(t,e,n){return this.x=t.x+(e.x-t.x)*n,this.y=t.y+(e.y-t.y)*n,this.z=t.z+(e.z-t.z)*n,this.w=t.w+(e.w-t.w)*n,this}equals(t){return t.x===this.x&&t.y===this.y&&t.z===this.z&&t.w===this.w}fromArray(t,e=0){return this.x=t[e],this.y=t[e+1],this.z=t[e+2],this.w=t[e+3],this}toArray(t=[],e=0){return t[e]=this.x,t[e+1]=this.y,t[e+2]=this.z,t[e+3]=this.w,t}fromBufferAttribute(t,e){return this.x=t.getX(e),this.y=t.getY(e),this.z=t.getZ(e),this.w=t.getW(e),this}random(){return this.x=Math.random(),this.y=Math.random(),this.z=Math.random(),this.w=Math.random(),this}*[Symbol.iterator](){yield this.x,yield this.y,yield this.z,yield this.w}}class G0 extends Or{constructor(t=1,e=1,n={}){super(),this.isRenderTarget=!0,this.width=t,this.height=e,this.depth=1,this.scissor=new Je(0,0,t,e),this.scissorTest=!1,this.viewport=new Je(0,0,t,e);const i={width:t,height:e,depth:1};n.encoding!==void 0&&(Vo("THREE.WebGLRenderTarget: option.encoding has been replaced by option.colorSpace."),n.colorSpace=n.encoding===Ns?sn:kn),n=Object.assign({generateMipmaps:!1,internalFormat:null,minFilter:Vn,depthBuffer:!0,stencilBuffer:!1,depthTexture:null,samples:0},n),this.texture=new Dn(i,n.mapping,n.wrapS,n.wrapT,n.magFilter,n.minFilter,n.format,n.type,n.anisotropy,n.colorSpace),this.texture.isRenderTargetTexture=!0,this.texture.flipY=!1,this.texture.generateMipmaps=n.generateMipmaps,this.texture.internalFormat=n.internalFormat,this.depthBuffer=n.depthBuffer,this.stencilBuffer=n.stencilBuffer,this.depthTexture=n.depthTexture,this.samples=n.samples}setSize(t,e,n=1){(this.width!==t||this.height!==e||this.depth!==n)&&(this.width=t,this.height=e,this.depth=n,this.texture.image.width=t,this.texture.image.height=e,this.texture.image.depth=n,this.dispose()),this.viewport.set(0,0,t,e),this.scissor.set(0,0,t,e)}clone(){return new this.constructor().copy(this)}copy(t){this.width=t.width,this.height=t.height,this.depth=t.depth,this.scissor.copy(t.scissor),this.scissorTest=t.scissorTest,this.viewport.copy(t.viewport),this.texture=t.texture.clone(),this.texture.isRenderTargetTexture=!0;const e=Object.assign({},t.texture.image);return this.texture.source=new am(e),this.depthBuffer=t.depthBuffer,this.stencilBuffer=t.stencilBuffer,t.depthTexture!==null&&(this.depthTexture=t.depthTexture.clone()),this.samples=t.samples,this}dispose(){this.dispatchEvent({type:"dispose"})}}class tn extends G0{constructor(t=1,e=1,n={}){super(t,e,n),this.isWebGLRenderTarget=!0}}class cm extends Dn{constructor(t=null,e=1,n=1,i=1){super(null),this.isDataArrayTexture=!0,this.image={data:t,width:e,height:n,depth:i},this.magFilter=_n,this.minFilter=_n,this.wrapR=si,this.generateMipmaps=!1,this.flipY=!1,this.unpackAlignment=1}}class V0 extends Dn{constructor(t=null,e=1,n=1,i=1){super(null),this.isData3DTexture=!0,this.image={data:t,width:e,height:n,depth:i},this.magFilter=_n,this.minFilter=_n,this.wrapR=si,this.generateMipmaps=!1,this.flipY=!1,this.unpackAlignment=1}}class Nn{constructor(t=0,e=0,n=0,i=1){this.isQuaternion=!0,this._x=t,this._y=e,this._z=n,this._w=i}static slerpFlat(t,e,n,i,s,a,o){let c=n[i+0],l=n[i+1],h=n[i+2],u=n[i+3];const f=s[a+0],d=s[a+1],g=s[a+2],x=s[a+3];if(o===0){t[e+0]=c,t[e+1]=l,t[e+2]=h,t[e+3]=u;return}if(o===1){t[e+0]=f,t[e+1]=d,t[e+2]=g,t[e+3]=x;return}if(u!==x||c!==f||l!==d||h!==g){let m=1-o;const p=c*f+l*d+h*g+u*x,_=p>=0?1:-1,v=1-p*p;if(v>Number.EPSILON){const y=Math.sqrt(v),M=Math.atan2(y,p*_);m=Math.sin(m*M)/y,o=Math.sin(o*M)/y}const S=o*_;if(c=c*m+f*S,l=l*m+d*S,h=h*m+g*S,u=u*m+x*S,m===1-o){const y=1/Math.sqrt(c*c+l*l+h*h+u*u);c*=y,l*=y,h*=y,u*=y}}t[e]=c,t[e+1]=l,t[e+2]=h,t[e+3]=u}static multiplyQuaternionsFlat(t,e,n,i,s,a){const o=n[i],c=n[i+1],l=n[i+2],h=n[i+3],u=s[a],f=s[a+1],d=s[a+2],g=s[a+3];return t[e]=o*g+h*u+c*d-l*f,t[e+1]=c*g+h*f+l*u-o*d,t[e+2]=l*g+h*d+o*f-c*u,t[e+3]=h*g-o*u-c*f-l*d,t}get x(){return this._x}set x(t){this._x=t,this._onChangeCallback()}get y(){return this._y}set y(t){this._y=t,this._onChangeCallback()}get z(){return this._z}set z(t){this._z=t,this._onChangeCallback()}get w(){return this._w}set w(t){this._w=t,this._onChangeCallback()}set(t,e,n,i){return this._x=t,this._y=e,this._z=n,this._w=i,this._onChangeCallback(),this}clone(){return new this.constructor(this._x,this._y,this._z,this._w)}copy(t){return this._x=t.x,this._y=t.y,this._z=t.z,this._w=t.w,this._onChangeCallback(),this}setFromEuler(t,e=!0){const n=t._x,i=t._y,s=t._z,a=t._order,o=Math.cos,c=Math.sin,l=o(n/2),h=o(i/2),u=o(s/2),f=c(n/2),d=c(i/2),g=c(s/2);switch(a){case"XYZ":this._x=f*h*u+l*d*g,this._y=l*d*u-f*h*g,this._z=l*h*g+f*d*u,this._w=l*h*u-f*d*g;break;case"YXZ":this._x=f*h*u+l*d*g,this._y=l*d*u-f*h*g,this._z=l*h*g-f*d*u,this._w=l*h*u+f*d*g;break;case"ZXY":this._x=f*h*u-l*d*g,this._y=l*d*u+f*h*g,this._z=l*h*g+f*d*u,this._w=l*h*u-f*d*g;break;case"ZYX":this._x=f*h*u-l*d*g,this._y=l*d*u+f*h*g,this._z=l*h*g-f*d*u,this._w=l*h*u+f*d*g;break;case"YZX":this._x=f*h*u+l*d*g,this._y=l*d*u+f*h*g,this._z=l*h*g-f*d*u,this._w=l*h*u-f*d*g;break;case"XZY":this._x=f*h*u-l*d*g,this._y=l*d*u-f*h*g,this._z=l*h*g+f*d*u,this._w=l*h*u+f*d*g;break;default:console.warn("THREE.Quaternion: .setFromEuler() encountered an unknown order: "+a)}return e===!0&&this._onChangeCallback(),this}setFromAxisAngle(t,e){const n=e/2,i=Math.sin(n);return this._x=t.x*i,this._y=t.y*i,this._z=t.z*i,this._w=Math.cos(n),this._onChangeCallback(),this}setFromRotationMatrix(t){const e=t.elements,n=e[0],i=e[4],s=e[8],a=e[1],o=e[5],c=e[9],l=e[2],h=e[6],u=e[10],f=n+o+u;if(f>0){const d=.5/Math.sqrt(f+1);this._w=.25/d,this._x=(h-c)*d,this._y=(s-l)*d,this._z=(a-i)*d}else if(n>o&&n>u){const d=2*Math.sqrt(1+n-o-u);this._w=(h-c)/d,this._x=.25*d,this._y=(i+a)/d,this._z=(s+l)/d}else if(o>u){const d=2*Math.sqrt(1+o-n-u);this._w=(s-l)/d,this._x=(i+a)/d,this._y=.25*d,this._z=(c+h)/d}else{const d=2*Math.sqrt(1+u-n-o);this._w=(a-i)/d,this._x=(s+l)/d,this._y=(c+h)/d,this._z=.25*d}return this._onChangeCallback(),this}setFromUnitVectors(t,e){let n=t.dot(e)+1;return n<Number.EPSILON?(n=0,Math.abs(t.x)>Math.abs(t.z)?(this._x=-t.y,this._y=t.x,this._z=0,this._w=n):(this._x=0,this._y=-t.z,this._z=t.y,this._w=n)):(this._x=t.y*e.z-t.z*e.y,this._y=t.z*e.x-t.x*e.z,this._z=t.x*e.y-t.y*e.x,this._w=n),this.normalize()}angleTo(t){return 2*Math.acos(Math.abs(Ge(this.dot(t),-1,1)))}rotateTowards(t,e){const n=this.angleTo(t);if(n===0)return this;const i=Math.min(1,e/n);return this.slerp(t,i),this}identity(){return this.set(0,0,0,1)}invert(){return this.conjugate()}conjugate(){return this._x*=-1,this._y*=-1,this._z*=-1,this._onChangeCallback(),this}dot(t){return this._x*t._x+this._y*t._y+this._z*t._z+this._w*t._w}lengthSq(){return this._x*this._x+this._y*this._y+this._z*this._z+this._w*this._w}length(){return Math.sqrt(this._x*this._x+this._y*this._y+this._z*this._z+this._w*this._w)}normalize(){let t=this.length();return t===0?(this._x=0,this._y=0,this._z=0,this._w=1):(t=1/t,this._x=this._x*t,this._y=this._y*t,this._z=this._z*t,this._w=this._w*t),this._onChangeCallback(),this}multiply(t){return this.multiplyQuaternions(this,t)}premultiply(t){return this.multiplyQuaternions(t,this)}multiplyQuaternions(t,e){const n=t._x,i=t._y,s=t._z,a=t._w,o=e._x,c=e._y,l=e._z,h=e._w;return this._x=n*h+a*o+i*l-s*c,this._y=i*h+a*c+s*o-n*l,this._z=s*h+a*l+n*c-i*o,this._w=a*h-n*o-i*c-s*l,this._onChangeCallback(),this}slerp(t,e){if(e===0)return this;if(e===1)return this.copy(t);const n=this._x,i=this._y,s=this._z,a=this._w;let o=a*t._w+n*t._x+i*t._y+s*t._z;if(o<0?(this._w=-t._w,this._x=-t._x,this._y=-t._y,this._z=-t._z,o=-o):this.copy(t),o>=1)return this._w=a,this._x=n,this._y=i,this._z=s,this;const c=1-o*o;if(c<=Number.EPSILON){const d=1-e;return this._w=d*a+e*this._w,this._x=d*n+e*this._x,this._y=d*i+e*this._y,this._z=d*s+e*this._z,this.normalize(),this}const l=Math.sqrt(c),h=Math.atan2(l,o),u=Math.sin((1-e)*h)/l,f=Math.sin(e*h)/l;return this._w=a*u+this._w*f,this._x=n*u+this._x*f,this._y=i*u+this._y*f,this._z=s*u+this._z*f,this._onChangeCallback(),this}slerpQuaternions(t,e,n){return this.copy(t).slerp(e,n)}random(){const t=Math.random(),e=Math.sqrt(1-t),n=Math.sqrt(t),i=2*Math.PI*Math.random(),s=2*Math.PI*Math.random();return this.set(e*Math.cos(i),n*Math.sin(s),n*Math.cos(s),e*Math.sin(i))}equals(t){return t._x===this._x&&t._y===this._y&&t._z===this._z&&t._w===this._w}fromArray(t,e=0){return this._x=t[e],this._y=t[e+1],this._z=t[e+2],this._w=t[e+3],this._onChangeCallback(),this}toArray(t=[],e=0){return t[e]=this._x,t[e+1]=this._y,t[e+2]=this._z,t[e+3]=this._w,t}fromBufferAttribute(t,e){return this._x=t.getX(e),this._y=t.getY(e),this._z=t.getZ(e),this._w=t.getW(e),this._onChangeCallback(),this}toJSON(){return this.toArray()}_onChange(t){return this._onChangeCallback=t,this}_onChangeCallback(){}*[Symbol.iterator](){yield this._x,yield this._y,yield this._z,yield this._w}}class b{constructor(t=0,e=0,n=0){b.prototype.isVector3=!0,this.x=t,this.y=e,this.z=n}set(t,e,n){return n===void 0&&(n=this.z),this.x=t,this.y=e,this.z=n,this}setScalar(t){return this.x=t,this.y=t,this.z=t,this}setX(t){return this.x=t,this}setY(t){return this.y=t,this}setZ(t){return this.z=t,this}setComponent(t,e){switch(t){case 0:this.x=e;break;case 1:this.y=e;break;case 2:this.z=e;break;default:throw new Error("index is out of range: "+t)}return this}getComponent(t){switch(t){case 0:return this.x;case 1:return this.y;case 2:return this.z;default:throw new Error("index is out of range: "+t)}}clone(){return new this.constructor(this.x,this.y,this.z)}copy(t){return this.x=t.x,this.y=t.y,this.z=t.z,this}add(t){return this.x+=t.x,this.y+=t.y,this.z+=t.z,this}addScalar(t){return this.x+=t,this.y+=t,this.z+=t,this}addVectors(t,e){return this.x=t.x+e.x,this.y=t.y+e.y,this.z=t.z+e.z,this}addScaledVector(t,e){return this.x+=t.x*e,this.y+=t.y*e,this.z+=t.z*e,this}sub(t){return this.x-=t.x,this.y-=t.y,this.z-=t.z,this}subScalar(t){return this.x-=t,this.y-=t,this.z-=t,this}subVectors(t,e){return this.x=t.x-e.x,this.y=t.y-e.y,this.z=t.z-e.z,this}multiply(t){return this.x*=t.x,this.y*=t.y,this.z*=t.z,this}multiplyScalar(t){return this.x*=t,this.y*=t,this.z*=t,this}multiplyVectors(t,e){return this.x=t.x*e.x,this.y=t.y*e.y,this.z=t.z*e.z,this}applyEuler(t){return this.applyQuaternion(Ad.setFromEuler(t))}applyAxisAngle(t,e){return this.applyQuaternion(Ad.setFromAxisAngle(t,e))}applyMatrix3(t){const e=this.x,n=this.y,i=this.z,s=t.elements;return this.x=s[0]*e+s[3]*n+s[6]*i,this.y=s[1]*e+s[4]*n+s[7]*i,this.z=s[2]*e+s[5]*n+s[8]*i,this}applyNormalMatrix(t){return this.applyMatrix3(t).normalize()}applyMatrix4(t){const e=this.x,n=this.y,i=this.z,s=t.elements,a=1/(s[3]*e+s[7]*n+s[11]*i+s[15]);return this.x=(s[0]*e+s[4]*n+s[8]*i+s[12])*a,this.y=(s[1]*e+s[5]*n+s[9]*i+s[13])*a,this.z=(s[2]*e+s[6]*n+s[10]*i+s[14])*a,this}applyQuaternion(t){const e=this.x,n=this.y,i=this.z,s=t.x,a=t.y,o=t.z,c=t.w,l=2*(a*i-o*n),h=2*(o*e-s*i),u=2*(s*n-a*e);return this.x=e+c*l+a*u-o*h,this.y=n+c*h+o*l-s*u,this.z=i+c*u+s*h-a*l,this}project(t){return this.applyMatrix4(t.matrixWorldInverse).applyMatrix4(t.projectionMatrix)}unproject(t){return this.applyMatrix4(t.projectionMatrixInverse).applyMatrix4(t.matrixWorld)}transformDirection(t){const e=this.x,n=this.y,i=this.z,s=t.elements;return this.x=s[0]*e+s[4]*n+s[8]*i,this.y=s[1]*e+s[5]*n+s[9]*i,this.z=s[2]*e+s[6]*n+s[10]*i,this.normalize()}divide(t){return this.x/=t.x,this.y/=t.y,this.z/=t.z,this}divideScalar(t){return this.multiplyScalar(1/t)}min(t){return this.x=Math.min(this.x,t.x),this.y=Math.min(this.y,t.y),this.z=Math.min(this.z,t.z),this}max(t){return this.x=Math.max(this.x,t.x),this.y=Math.max(this.y,t.y),this.z=Math.max(this.z,t.z),this}clamp(t,e){return this.x=Math.max(t.x,Math.min(e.x,this.x)),this.y=Math.max(t.y,Math.min(e.y,this.y)),this.z=Math.max(t.z,Math.min(e.z,this.z)),this}clampScalar(t,e){return this.x=Math.max(t,Math.min(e,this.x)),this.y=Math.max(t,Math.min(e,this.y)),this.z=Math.max(t,Math.min(e,this.z)),this}clampLength(t,e){const n=this.length();return this.divideScalar(n||1).multiplyScalar(Math.max(t,Math.min(e,n)))}floor(){return this.x=Math.floor(this.x),this.y=Math.floor(this.y),this.z=Math.floor(this.z),this}ceil(){return this.x=Math.ceil(this.x),this.y=Math.ceil(this.y),this.z=Math.ceil(this.z),this}round(){return this.x=Math.round(this.x),this.y=Math.round(this.y),this.z=Math.round(this.z),this}roundToZero(){return this.x=Math.trunc(this.x),this.y=Math.trunc(this.y),this.z=Math.trunc(this.z),this}negate(){return this.x=-this.x,this.y=-this.y,this.z=-this.z,this}dot(t){return this.x*t.x+this.y*t.y+this.z*t.z}lengthSq(){return this.x*this.x+this.y*this.y+this.z*this.z}length(){return Math.sqrt(this.x*this.x+this.y*this.y+this.z*this.z)}manhattanLength(){return Math.abs(this.x)+Math.abs(this.y)+Math.abs(this.z)}normalize(){return this.divideScalar(this.length()||1)}setLength(t){return this.normalize().multiplyScalar(t)}lerp(t,e){return this.x+=(t.x-this.x)*e,this.y+=(t.y-this.y)*e,this.z+=(t.z-this.z)*e,this}lerpVectors(t,e,n){return this.x=t.x+(e.x-t.x)*n,this.y=t.y+(e.y-t.y)*n,this.z=t.z+(e.z-t.z)*n,this}cross(t){return this.crossVectors(this,t)}crossVectors(t,e){const n=t.x,i=t.y,s=t.z,a=e.x,o=e.y,c=e.z;return this.x=i*c-s*o,this.y=s*a-n*c,this.z=n*o-i*a,this}projectOnVector(t){const e=t.lengthSq();if(e===0)return this.set(0,0,0);const n=t.dot(this)/e;return this.copy(t).multiplyScalar(n)}projectOnPlane(t){return cl.copy(this).projectOnVector(t),this.sub(cl)}reflect(t){return this.sub(cl.copy(t).multiplyScalar(2*this.dot(t)))}angleTo(t){const e=Math.sqrt(this.lengthSq()*t.lengthSq());if(e===0)return Math.PI/2;const n=this.dot(t)/e;return Math.acos(Ge(n,-1,1))}distanceTo(t){return Math.sqrt(this.distanceToSquared(t))}distanceToSquared(t){const e=this.x-t.x,n=this.y-t.y,i=this.z-t.z;return e*e+n*n+i*i}manhattanDistanceTo(t){return Math.abs(this.x-t.x)+Math.abs(this.y-t.y)+Math.abs(this.z-t.z)}setFromSpherical(t){return this.setFromSphericalCoords(t.radius,t.phi,t.theta)}setFromSphericalCoords(t,e,n){const i=Math.sin(e)*t;return this.x=i*Math.sin(n),this.y=Math.cos(e)*t,this.z=i*Math.cos(n),this}setFromCylindrical(t){return this.setFromCylindricalCoords(t.radius,t.theta,t.y)}setFromCylindricalCoords(t,e,n){return this.x=t*Math.sin(e),this.y=n,this.z=t*Math.cos(e),this}setFromMatrixPosition(t){const e=t.elements;return this.x=e[12],this.y=e[13],this.z=e[14],this}setFromMatrixScale(t){const e=this.setFromMatrixColumn(t,0).length(),n=this.setFromMatrixColumn(t,1).length(),i=this.setFromMatrixColumn(t,2).length();return this.x=e,this.y=n,this.z=i,this}setFromMatrixColumn(t,e){return this.fromArray(t.elements,e*4)}setFromMatrix3Column(t,e){return this.fromArray(t.elements,e*3)}setFromEuler(t){return this.x=t._x,this.y=t._y,this.z=t._z,this}setFromColor(t){return this.x=t.r,this.y=t.g,this.z=t.b,this}equals(t){return t.x===this.x&&t.y===this.y&&t.z===this.z}fromArray(t,e=0){return this.x=t[e],this.y=t[e+1],this.z=t[e+2],this}toArray(t=[],e=0){return t[e]=this.x,t[e+1]=this.y,t[e+2]=this.z,t}fromBufferAttribute(t,e){return this.x=t.getX(e),this.y=t.getY(e),this.z=t.getZ(e),this}random(){return this.x=Math.random(),this.y=Math.random(),this.z=Math.random(),this}randomDirection(){const t=(Math.random()-.5)*2,e=Math.random()*Math.PI*2,n=Math.sqrt(1-t**2);return this.x=n*Math.cos(e),this.y=n*Math.sin(e),this.z=t,this}*[Symbol.iterator](){yield this.x,yield this.y,yield this.z}}const cl=new b,Ad=new Nn;class en{constructor(t=new b(1/0,1/0,1/0),e=new b(-1/0,-1/0,-1/0)){this.isBox3=!0,this.min=t,this.max=e}set(t,e){return this.min.copy(t),this.max.copy(e),this}setFromArray(t){this.makeEmpty();for(let e=0,n=t.length;e<n;e+=3)this.expandByPoint(Jn.fromArray(t,e));return this}setFromBufferAttribute(t){this.makeEmpty();for(let e=0,n=t.count;e<n;e++)this.expandByPoint(Jn.fromBufferAttribute(t,e));return this}setFromPoints(t){this.makeEmpty();for(let e=0,n=t.length;e<n;e++)this.expandByPoint(t[e]);return this}setFromCenterAndSize(t,e){const n=Jn.copy(e).multiplyScalar(.5);return this.min.copy(t).sub(n),this.max.copy(t).add(n),this}setFromObject(t,e=!1){return this.makeEmpty(),this.expandByObject(t,e)}clone(){return new this.constructor().copy(this)}copy(t){return this.min.copy(t.min),this.max.copy(t.max),this}makeEmpty(){return this.min.x=this.min.y=this.min.z=1/0,this.max.x=this.max.y=this.max.z=-1/0,this}isEmpty(){return this.max.x<this.min.x||this.max.y<this.min.y||this.max.z<this.min.z}getCenter(t){return this.isEmpty()?t.set(0,0,0):t.addVectors(this.min,this.max).multiplyScalar(.5)}getSize(t){return this.isEmpty()?t.set(0,0,0):t.subVectors(this.max,this.min)}expandByPoint(t){return this.min.min(t),this.max.max(t),this}expandByVector(t){return this.min.sub(t),this.max.add(t),this}expandByScalar(t){return this.min.addScalar(-t),this.max.addScalar(t),this}expandByObject(t,e=!1){t.updateWorldMatrix(!1,!1);const n=t.geometry;if(n!==void 0){const s=n.getAttribute("position");if(e===!0&&s!==void 0&&t.isInstancedMesh!==!0)for(let a=0,o=s.count;a<o;a++)t.isMesh===!0?t.getVertexPosition(a,Jn):Jn.fromBufferAttribute(s,a),Jn.applyMatrix4(t.matrixWorld),this.expandByPoint(Jn);else t.boundingBox!==void 0?(t.boundingBox===null&&t.computeBoundingBox(),oa.copy(t.boundingBox)):(n.boundingBox===null&&n.computeBoundingBox(),oa.copy(n.boundingBox)),oa.applyMatrix4(t.matrixWorld),this.union(oa)}const i=t.children;for(let s=0,a=i.length;s<a;s++)this.expandByObject(i[s],e);return this}containsPoint(t){return!(t.x<this.min.x||t.x>this.max.x||t.y<this.min.y||t.y>this.max.y||t.z<this.min.z||t.z>this.max.z)}containsBox(t){return this.min.x<=t.min.x&&t.max.x<=this.max.x&&this.min.y<=t.min.y&&t.max.y<=this.max.y&&this.min.z<=t.min.z&&t.max.z<=this.max.z}getParameter(t,e){return e.set((t.x-this.min.x)/(this.max.x-this.min.x),(t.y-this.min.y)/(this.max.y-this.min.y),(t.z-this.min.z)/(this.max.z-this.min.z))}intersectsBox(t){return!(t.max.x<this.min.x||t.min.x>this.max.x||t.max.y<this.min.y||t.min.y>this.max.y||t.max.z<this.min.z||t.min.z>this.max.z)}intersectsSphere(t){return this.clampPoint(t.center,Jn),Jn.distanceToSquared(t.center)<=t.radius*t.radius}intersectsPlane(t){let e,n;return t.normal.x>0?(e=t.normal.x*this.min.x,n=t.normal.x*this.max.x):(e=t.normal.x*this.max.x,n=t.normal.x*this.min.x),t.normal.y>0?(e+=t.normal.y*this.min.y,n+=t.normal.y*this.max.y):(e+=t.normal.y*this.max.y,n+=t.normal.y*this.min.y),t.normal.z>0?(e+=t.normal.z*this.min.z,n+=t.normal.z*this.max.z):(e+=t.normal.z*this.max.z,n+=t.normal.z*this.min.z),e<=-t.constant&&n>=-t.constant}intersectsTriangle(t){if(this.isEmpty())return!1;this.getCenter(Zr),aa.subVectors(this.max,Zr),Ws.subVectors(t.a,Zr),Xs.subVectors(t.b,Zr),Ys.subVectors(t.c,Zr),Xi.subVectors(Xs,Ws),Yi.subVectors(Ys,Xs),xs.subVectors(Ws,Ys);let e=[0,-Xi.z,Xi.y,0,-Yi.z,Yi.y,0,-xs.z,xs.y,Xi.z,0,-Xi.x,Yi.z,0,-Yi.x,xs.z,0,-xs.x,-Xi.y,Xi.x,0,-Yi.y,Yi.x,0,-xs.y,xs.x,0];return!ll(e,Ws,Xs,Ys,aa)||(e=[1,0,0,0,1,0,0,0,1],!ll(e,Ws,Xs,Ys,aa))?!1:(ca.crossVectors(Xi,Yi),e=[ca.x,ca.y,ca.z],ll(e,Ws,Xs,Ys,aa))}clampPoint(t,e){return e.copy(t).clamp(this.min,this.max)}distanceToPoint(t){return this.clampPoint(t,Jn).distanceTo(t)}getBoundingSphere(t){return this.isEmpty()?t.makeEmpty():(this.getCenter(t.center),t.radius=this.getSize(Jn).length()*.5),t}intersect(t){return this.min.max(t.min),this.max.min(t.max),this.isEmpty()&&this.makeEmpty(),this}union(t){return this.min.min(t.min),this.max.max(t.max),this}applyMatrix4(t){return this.isEmpty()?this:(Ai[0].set(this.min.x,this.min.y,this.min.z).applyMatrix4(t),Ai[1].set(this.min.x,this.min.y,this.max.z).applyMatrix4(t),Ai[2].set(this.min.x,this.max.y,this.min.z).applyMatrix4(t),Ai[3].set(this.min.x,this.max.y,this.max.z).applyMatrix4(t),Ai[4].set(this.max.x,this.min.y,this.min.z).applyMatrix4(t),Ai[5].set(this.max.x,this.min.y,this.max.z).applyMatrix4(t),Ai[6].set(this.max.x,this.max.y,this.min.z).applyMatrix4(t),Ai[7].set(this.max.x,this.max.y,this.max.z).applyMatrix4(t),this.setFromPoints(Ai),this)}translate(t){return this.min.add(t),this.max.add(t),this}equals(t){return t.min.equals(this.min)&&t.max.equals(this.max)}}const Ai=[new b,new b,new b,new b,new b,new b,new b,new b],Jn=new b,oa=new en,Ws=new b,Xs=new b,Ys=new b,Xi=new b,Yi=new b,xs=new b,Zr=new b,aa=new b,ca=new b,_s=new b;function ll(r,t,e,n,i){for(let s=0,a=r.length-3;s<=a;s+=3){_s.fromArray(r,s);const o=i.x*Math.abs(_s.x)+i.y*Math.abs(_s.y)+i.z*Math.abs(_s.z),c=t.dot(_s),l=e.dot(_s),h=n.dot(_s);if(Math.max(-Math.max(c,l,h),Math.min(c,l,h))>o)return!1}return!0}const k0=new en,jr=new b,hl=new b;class ms{constructor(t=new b,e=-1){this.isSphere=!0,this.center=t,this.radius=e}set(t,e){return this.center.copy(t),this.radius=e,this}setFromPoints(t,e){const n=this.center;e!==void 0?n.copy(e):k0.setFromPoints(t).getCenter(n);let i=0;for(let s=0,a=t.length;s<a;s++)i=Math.max(i,n.distanceToSquared(t[s]));return this.radius=Math.sqrt(i),this}copy(t){return this.center.copy(t.center),this.radius=t.radius,this}isEmpty(){return this.radius<0}makeEmpty(){return this.center.set(0,0,0),this.radius=-1,this}containsPoint(t){return t.distanceToSquared(this.center)<=this.radius*this.radius}distanceToPoint(t){return t.distanceTo(this.center)-this.radius}intersectsSphere(t){const e=this.radius+t.radius;return t.center.distanceToSquared(this.center)<=e*e}intersectsBox(t){return t.intersectsSphere(this)}intersectsPlane(t){return Math.abs(t.distanceToPoint(this.center))<=this.radius}clampPoint(t,e){const n=this.center.distanceToSquared(t);return e.copy(t),n>this.radius*this.radius&&(e.sub(this.center).normalize(),e.multiplyScalar(this.radius).add(this.center)),e}getBoundingBox(t){return this.isEmpty()?(t.makeEmpty(),t):(t.set(this.center,this.center),t.expandByScalar(this.radius),t)}applyMatrix4(t){return this.center.applyMatrix4(t),this.radius=this.radius*t.getMaxScaleOnAxis(),this}translate(t){return this.center.add(t),this}expandByPoint(t){if(this.isEmpty())return this.center.copy(t),this.radius=0,this;jr.subVectors(t,this.center);const e=jr.lengthSq();if(e>this.radius*this.radius){const n=Math.sqrt(e),i=(n-this.radius)*.5;this.center.addScaledVector(jr,i/n),this.radius+=i}return this}union(t){return t.isEmpty()?this:this.isEmpty()?(this.copy(t),this):(this.center.equals(t.center)===!0?this.radius=Math.max(this.radius,t.radius):(hl.subVectors(t.center,this.center).setLength(t.radius),this.expandByPoint(jr.copy(t.center).add(hl)),this.expandByPoint(jr.copy(t.center).sub(hl))),this)}equals(t){return t.center.equals(this.center)&&t.radius===this.radius}clone(){return new this.constructor().copy(this)}}const Ti=new b,ul=new b,la=new b,qi=new b,dl=new b,ha=new b,fl=new b;class pu{constructor(t=new b,e=new b(0,0,-1)){this.origin=t,this.direction=e}set(t,e){return this.origin.copy(t),this.direction.copy(e),this}copy(t){return this.origin.copy(t.origin),this.direction.copy(t.direction),this}at(t,e){return e.copy(this.origin).addScaledVector(this.direction,t)}lookAt(t){return this.direction.copy(t).sub(this.origin).normalize(),this}recast(t){return this.origin.copy(this.at(t,Ti)),this}closestPointToPoint(t,e){e.subVectors(t,this.origin);const n=e.dot(this.direction);return n<0?e.copy(this.origin):e.copy(this.origin).addScaledVector(this.direction,n)}distanceToPoint(t){return Math.sqrt(this.distanceSqToPoint(t))}distanceSqToPoint(t){const e=Ti.subVectors(t,this.origin).dot(this.direction);return e<0?this.origin.distanceToSquared(t):(Ti.copy(this.origin).addScaledVector(this.direction,e),Ti.distanceToSquared(t))}distanceSqToSegment(t,e,n,i){ul.copy(t).add(e).multiplyScalar(.5),la.copy(e).sub(t).normalize(),qi.copy(this.origin).sub(ul);const s=t.distanceTo(e)*.5,a=-this.direction.dot(la),o=qi.dot(this.direction),c=-qi.dot(la),l=qi.lengthSq(),h=Math.abs(1-a*a);let u,f,d,g;if(h>0)if(u=a*c-o,f=a*o-c,g=s*h,u>=0)if(f>=-g)if(f<=g){const x=1/h;u*=x,f*=x,d=u*(u+a*f+2*o)+f*(a*u+f+2*c)+l}else f=s,u=Math.max(0,-(a*f+o)),d=-u*u+f*(f+2*c)+l;else f=-s,u=Math.max(0,-(a*f+o)),d=-u*u+f*(f+2*c)+l;else f<=-g?(u=Math.max(0,-(-a*s+o)),f=u>0?-s:Math.min(Math.max(-s,-c),s),d=-u*u+f*(f+2*c)+l):f<=g?(u=0,f=Math.min(Math.max(-s,-c),s),d=f*(f+2*c)+l):(u=Math.max(0,-(a*s+o)),f=u>0?s:Math.min(Math.max(-s,-c),s),d=-u*u+f*(f+2*c)+l);else f=a>0?-s:s,u=Math.max(0,-(a*f+o)),d=-u*u+f*(f+2*c)+l;return n&&n.copy(this.origin).addScaledVector(this.direction,u),i&&i.copy(ul).addScaledVector(la,f),d}intersectSphere(t,e){Ti.subVectors(t.center,this.origin);const n=Ti.dot(this.direction),i=Ti.dot(Ti)-n*n,s=t.radius*t.radius;if(i>s)return null;const a=Math.sqrt(s-i),o=n-a,c=n+a;return c<0?null:o<0?this.at(c,e):this.at(o,e)}intersectsSphere(t){return this.distanceSqToPoint(t.center)<=t.radius*t.radius}distanceToPlane(t){const e=t.normal.dot(this.direction);if(e===0)return t.distanceToPoint(this.origin)===0?0:null;const n=-(this.origin.dot(t.normal)+t.constant)/e;return n>=0?n:null}intersectPlane(t,e){const n=this.distanceToPlane(t);return n===null?null:this.at(n,e)}intersectsPlane(t){const e=t.distanceToPoint(this.origin);return e===0||t.normal.dot(this.direction)*e<0}intersectBox(t,e){let n,i,s,a,o,c;const l=1/this.direction.x,h=1/this.direction.y,u=1/this.direction.z,f=this.origin;return l>=0?(n=(t.min.x-f.x)*l,i=(t.max.x-f.x)*l):(n=(t.max.x-f.x)*l,i=(t.min.x-f.x)*l),h>=0?(s=(t.min.y-f.y)*h,a=(t.max.y-f.y)*h):(s=(t.max.y-f.y)*h,a=(t.min.y-f.y)*h),n>a||s>i||((s>n||isNaN(n))&&(n=s),(a<i||isNaN(i))&&(i=a),u>=0?(o=(t.min.z-f.z)*u,c=(t.max.z-f.z)*u):(o=(t.max.z-f.z)*u,c=(t.min.z-f.z)*u),n>c||o>i)||((o>n||n!==n)&&(n=o),(c<i||i!==i)&&(i=c),i<0)?null:this.at(n>=0?n:i,e)}intersectsBox(t){return this.intersectBox(t,Ti)!==null}intersectTriangle(t,e,n,i,s){dl.subVectors(e,t),ha.subVectors(n,t),fl.crossVectors(dl,ha);let a=this.direction.dot(fl),o;if(a>0){if(i)return null;o=1}else if(a<0)o=-1,a=-a;else return null;qi.subVectors(this.origin,t);const c=o*this.direction.dot(ha.crossVectors(qi,ha));if(c<0)return null;const l=o*this.direction.dot(dl.cross(qi));if(l<0||c+l>a)return null;const h=-o*qi.dot(fl);return h<0?null:this.at(h/a,s)}applyMatrix4(t){return this.origin.applyMatrix4(t),this.direction.transformDirection(t),this}equals(t){return t.origin.equals(this.origin)&&t.direction.equals(this.direction)}clone(){return new this.constructor().copy(this)}}class se{constructor(t,e,n,i,s,a,o,c,l,h,u,f,d,g,x,m){se.prototype.isMatrix4=!0,this.elements=[1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1],t!==void 0&&this.set(t,e,n,i,s,a,o,c,l,h,u,f,d,g,x,m)}set(t,e,n,i,s,a,o,c,l,h,u,f,d,g,x,m){const p=this.elements;return p[0]=t,p[4]=e,p[8]=n,p[12]=i,p[1]=s,p[5]=a,p[9]=o,p[13]=c,p[2]=l,p[6]=h,p[10]=u,p[14]=f,p[3]=d,p[7]=g,p[11]=x,p[15]=m,this}identity(){return this.set(1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1),this}clone(){return new se().fromArray(this.elements)}copy(t){const e=this.elements,n=t.elements;return e[0]=n[0],e[1]=n[1],e[2]=n[2],e[3]=n[3],e[4]=n[4],e[5]=n[5],e[6]=n[6],e[7]=n[7],e[8]=n[8],e[9]=n[9],e[10]=n[10],e[11]=n[11],e[12]=n[12],e[13]=n[13],e[14]=n[14],e[15]=n[15],this}copyPosition(t){const e=this.elements,n=t.elements;return e[12]=n[12],e[13]=n[13],e[14]=n[14],this}setFromMatrix3(t){const e=t.elements;return this.set(e[0],e[3],e[6],0,e[1],e[4],e[7],0,e[2],e[5],e[8],0,0,0,0,1),this}extractBasis(t,e,n){return t.setFromMatrixColumn(this,0),e.setFromMatrixColumn(this,1),n.setFromMatrixColumn(this,2),this}makeBasis(t,e,n){return this.set(t.x,e.x,n.x,0,t.y,e.y,n.y,0,t.z,e.z,n.z,0,0,0,0,1),this}extractRotation(t){const e=this.elements,n=t.elements,i=1/qs.setFromMatrixColumn(t,0).length(),s=1/qs.setFromMatrixColumn(t,1).length(),a=1/qs.setFromMatrixColumn(t,2).length();return e[0]=n[0]*i,e[1]=n[1]*i,e[2]=n[2]*i,e[3]=0,e[4]=n[4]*s,e[5]=n[5]*s,e[6]=n[6]*s,e[7]=0,e[8]=n[8]*a,e[9]=n[9]*a,e[10]=n[10]*a,e[11]=0,e[12]=0,e[13]=0,e[14]=0,e[15]=1,this}makeRotationFromEuler(t){const e=this.elements,n=t.x,i=t.y,s=t.z,a=Math.cos(n),o=Math.sin(n),c=Math.cos(i),l=Math.sin(i),h=Math.cos(s),u=Math.sin(s);if(t.order==="XYZ"){const f=a*h,d=a*u,g=o*h,x=o*u;e[0]=c*h,e[4]=-c*u,e[8]=l,e[1]=d+g*l,e[5]=f-x*l,e[9]=-o*c,e[2]=x-f*l,e[6]=g+d*l,e[10]=a*c}else if(t.order==="YXZ"){const f=c*h,d=c*u,g=l*h,x=l*u;e[0]=f+x*o,e[4]=g*o-d,e[8]=a*l,e[1]=a*u,e[5]=a*h,e[9]=-o,e[2]=d*o-g,e[6]=x+f*o,e[10]=a*c}else if(t.order==="ZXY"){const f=c*h,d=c*u,g=l*h,x=l*u;e[0]=f-x*o,e[4]=-a*u,e[8]=g+d*o,e[1]=d+g*o,e[5]=a*h,e[9]=x-f*o,e[2]=-a*l,e[6]=o,e[10]=a*c}else if(t.order==="ZYX"){const f=a*h,d=a*u,g=o*h,x=o*u;e[0]=c*h,e[4]=g*l-d,e[8]=f*l+x,e[1]=c*u,e[5]=x*l+f,e[9]=d*l-g,e[2]=-l,e[6]=o*c,e[10]=a*c}else if(t.order==="YZX"){const f=a*c,d=a*l,g=o*c,x=o*l;e[0]=c*h,e[4]=x-f*u,e[8]=g*u+d,e[1]=u,e[5]=a*h,e[9]=-o*h,e[2]=-l*h,e[6]=d*u+g,e[10]=f-x*u}else if(t.order==="XZY"){const f=a*c,d=a*l,g=o*c,x=o*l;e[0]=c*h,e[4]=-u,e[8]=l*h,e[1]=f*u+x,e[5]=a*h,e[9]=d*u-g,e[2]=g*u-d,e[6]=o*h,e[10]=x*u+f}return e[3]=0,e[7]=0,e[11]=0,e[12]=0,e[13]=0,e[14]=0,e[15]=1,this}makeRotationFromQuaternion(t){return this.compose(H0,t,W0)}lookAt(t,e,n){const i=this.elements;return bn.subVectors(t,e),bn.lengthSq()===0&&(bn.z=1),bn.normalize(),Zi.crossVectors(n,bn),Zi.lengthSq()===0&&(Math.abs(n.z)===1?bn.x+=1e-4:bn.z+=1e-4,bn.normalize(),Zi.crossVectors(n,bn)),Zi.normalize(),ua.crossVectors(bn,Zi),i[0]=Zi.x,i[4]=ua.x,i[8]=bn.x,i[1]=Zi.y,i[5]=ua.y,i[9]=bn.y,i[2]=Zi.z,i[6]=ua.z,i[10]=bn.z,this}multiply(t){return this.multiplyMatrices(this,t)}premultiply(t){return this.multiplyMatrices(t,this)}multiplyMatrices(t,e){const n=t.elements,i=e.elements,s=this.elements,a=n[0],o=n[4],c=n[8],l=n[12],h=n[1],u=n[5],f=n[9],d=n[13],g=n[2],x=n[6],m=n[10],p=n[14],_=n[3],v=n[7],S=n[11],y=n[15],M=i[0],w=i[4],T=i[8],E=i[12],A=i[1],D=i[5],R=i[9],N=i[13],L=i[2],F=i[6],z=i[10],q=i[14],O=i[3],Y=i[7],K=i[11],J=i[15];return s[0]=a*M+o*A+c*L+l*O,s[4]=a*w+o*D+c*F+l*Y,s[8]=a*T+o*R+c*z+l*K,s[12]=a*E+o*N+c*q+l*J,s[1]=h*M+u*A+f*L+d*O,s[5]=h*w+u*D+f*F+d*Y,s[9]=h*T+u*R+f*z+d*K,s[13]=h*E+u*N+f*q+d*J,s[2]=g*M+x*A+m*L+p*O,s[6]=g*w+x*D+m*F+p*Y,s[10]=g*T+x*R+m*z+p*K,s[14]=g*E+x*N+m*q+p*J,s[3]=_*M+v*A+S*L+y*O,s[7]=_*w+v*D+S*F+y*Y,s[11]=_*T+v*R+S*z+y*K,s[15]=_*E+v*N+S*q+y*J,this}multiplyScalar(t){const e=this.elements;return e[0]*=t,e[4]*=t,e[8]*=t,e[12]*=t,e[1]*=t,e[5]*=t,e[9]*=t,e[13]*=t,e[2]*=t,e[6]*=t,e[10]*=t,e[14]*=t,e[3]*=t,e[7]*=t,e[11]*=t,e[15]*=t,this}determinant(){const t=this.elements,e=t[0],n=t[4],i=t[8],s=t[12],a=t[1],o=t[5],c=t[9],l=t[13],h=t[2],u=t[6],f=t[10],d=t[14],g=t[3],x=t[7],m=t[11],p=t[15];return g*(+s*c*u-i*l*u-s*o*f+n*l*f+i*o*d-n*c*d)+x*(+e*c*d-e*l*f+s*a*f-i*a*d+i*l*h-s*c*h)+m*(+e*l*u-e*o*d-s*a*u+n*a*d+s*o*h-n*l*h)+p*(-i*o*h-e*c*u+e*o*f+i*a*u-n*a*f+n*c*h)}transpose(){const t=this.elements;let e;return e=t[1],t[1]=t[4],t[4]=e,e=t[2],t[2]=t[8],t[8]=e,e=t[6],t[6]=t[9],t[9]=e,e=t[3],t[3]=t[12],t[12]=e,e=t[7],t[7]=t[13],t[13]=e,e=t[11],t[11]=t[14],t[14]=e,this}setPosition(t,e,n){const i=this.elements;return t.isVector3?(i[12]=t.x,i[13]=t.y,i[14]=t.z):(i[12]=t,i[13]=e,i[14]=n),this}invert(){const t=this.elements,e=t[0],n=t[1],i=t[2],s=t[3],a=t[4],o=t[5],c=t[6],l=t[7],h=t[8],u=t[9],f=t[10],d=t[11],g=t[12],x=t[13],m=t[14],p=t[15],_=u*m*l-x*f*l+x*c*d-o*m*d-u*c*p+o*f*p,v=g*f*l-h*m*l-g*c*d+a*m*d+h*c*p-a*f*p,S=h*x*l-g*u*l+g*o*d-a*x*d-h*o*p+a*u*p,y=g*u*c-h*x*c-g*o*f+a*x*f+h*o*m-a*u*m,M=e*_+n*v+i*S+s*y;if(M===0)return this.set(0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0);const w=1/M;return t[0]=_*w,t[1]=(x*f*s-u*m*s-x*i*d+n*m*d+u*i*p-n*f*p)*w,t[2]=(o*m*s-x*c*s+x*i*l-n*m*l-o*i*p+n*c*p)*w,t[3]=(u*c*s-o*f*s-u*i*l+n*f*l+o*i*d-n*c*d)*w,t[4]=v*w,t[5]=(h*m*s-g*f*s+g*i*d-e*m*d-h*i*p+e*f*p)*w,t[6]=(g*c*s-a*m*s-g*i*l+e*m*l+a*i*p-e*c*p)*w,t[7]=(a*f*s-h*c*s+h*i*l-e*f*l-a*i*d+e*c*d)*w,t[8]=S*w,t[9]=(g*u*s-h*x*s-g*n*d+e*x*d+h*n*p-e*u*p)*w,t[10]=(a*x*s-g*o*s+g*n*l-e*x*l-a*n*p+e*o*p)*w,t[11]=(h*o*s-a*u*s-h*n*l+e*u*l+a*n*d-e*o*d)*w,t[12]=y*w,t[13]=(h*x*i-g*u*i+g*n*f-e*x*f-h*n*m+e*u*m)*w,t[14]=(g*o*i-a*x*i-g*n*c+e*x*c+a*n*m-e*o*m)*w,t[15]=(a*u*i-h*o*i+h*n*c-e*u*c-a*n*f+e*o*f)*w,this}scale(t){const e=this.elements,n=t.x,i=t.y,s=t.z;return e[0]*=n,e[4]*=i,e[8]*=s,e[1]*=n,e[5]*=i,e[9]*=s,e[2]*=n,e[6]*=i,e[10]*=s,e[3]*=n,e[7]*=i,e[11]*=s,this}getMaxScaleOnAxis(){const t=this.elements,e=t[0]*t[0]+t[1]*t[1]+t[2]*t[2],n=t[4]*t[4]+t[5]*t[5]+t[6]*t[6],i=t[8]*t[8]+t[9]*t[9]+t[10]*t[10];return Math.sqrt(Math.max(e,n,i))}makeTranslation(t,e,n){return t.isVector3?this.set(1,0,0,t.x,0,1,0,t.y,0,0,1,t.z,0,0,0,1):this.set(1,0,0,t,0,1,0,e,0,0,1,n,0,0,0,1),this}makeRotationX(t){const e=Math.cos(t),n=Math.sin(t);return this.set(1,0,0,0,0,e,-n,0,0,n,e,0,0,0,0,1),this}makeRotationY(t){const e=Math.cos(t),n=Math.sin(t);return this.set(e,0,n,0,0,1,0,0,-n,0,e,0,0,0,0,1),this}makeRotationZ(t){const e=Math.cos(t),n=Math.sin(t);return this.set(e,-n,0,0,n,e,0,0,0,0,1,0,0,0,0,1),this}makeRotationAxis(t,e){const n=Math.cos(e),i=Math.sin(e),s=1-n,a=t.x,o=t.y,c=t.z,l=s*a,h=s*o;return this.set(l*a+n,l*o-i*c,l*c+i*o,0,l*o+i*c,h*o+n,h*c-i*a,0,l*c-i*o,h*c+i*a,s*c*c+n,0,0,0,0,1),this}makeScale(t,e,n){return this.set(t,0,0,0,0,e,0,0,0,0,n,0,0,0,0,1),this}makeShear(t,e,n,i,s,a){return this.set(1,n,s,0,t,1,a,0,e,i,1,0,0,0,0,1),this}compose(t,e,n){const i=this.elements,s=e._x,a=e._y,o=e._z,c=e._w,l=s+s,h=a+a,u=o+o,f=s*l,d=s*h,g=s*u,x=a*h,m=a*u,p=o*u,_=c*l,v=c*h,S=c*u,y=n.x,M=n.y,w=n.z;return i[0]=(1-(x+p))*y,i[1]=(d+S)*y,i[2]=(g-v)*y,i[3]=0,i[4]=(d-S)*M,i[5]=(1-(f+p))*M,i[6]=(m+_)*M,i[7]=0,i[8]=(g+v)*w,i[9]=(m-_)*w,i[10]=(1-(f+x))*w,i[11]=0,i[12]=t.x,i[13]=t.y,i[14]=t.z,i[15]=1,this}decompose(t,e,n){const i=this.elements;let s=qs.set(i[0],i[1],i[2]).length();const a=qs.set(i[4],i[5],i[6]).length(),o=qs.set(i[8],i[9],i[10]).length();this.determinant()<0&&(s=-s),t.x=i[12],t.y=i[13],t.z=i[14],Qn.copy(this);const l=1/s,h=1/a,u=1/o;return Qn.elements[0]*=l,Qn.elements[1]*=l,Qn.elements[2]*=l,Qn.elements[4]*=h,Qn.elements[5]*=h,Qn.elements[6]*=h,Qn.elements[8]*=u,Qn.elements[9]*=u,Qn.elements[10]*=u,e.setFromRotationMatrix(Qn),n.x=s,n.y=a,n.z=o,this}makePerspective(t,e,n,i,s,a,o=Oi){const c=this.elements,l=2*s/(e-t),h=2*s/(n-i),u=(e+t)/(e-t),f=(n+i)/(n-i);let d,g;if(o===Oi)d=-(a+s)/(a-s),g=-2*a*s/(a-s);else if(o===Mc)d=-a/(a-s),g=-a*s/(a-s);else throw new Error("THREE.Matrix4.makePerspective(): Invalid coordinate system: "+o);return c[0]=l,c[4]=0,c[8]=u,c[12]=0,c[1]=0,c[5]=h,c[9]=f,c[13]=0,c[2]=0,c[6]=0,c[10]=d,c[14]=g,c[3]=0,c[7]=0,c[11]=-1,c[15]=0,this}makeOrthographic(t,e,n,i,s,a,o=Oi){const c=this.elements,l=1/(e-t),h=1/(n-i),u=1/(a-s),f=(e+t)*l,d=(n+i)*h;let g,x;if(o===Oi)g=(a+s)*u,x=-2*u;else if(o===Mc)g=s*u,x=-1*u;else throw new Error("THREE.Matrix4.makeOrthographic(): Invalid coordinate system: "+o);return c[0]=2*l,c[4]=0,c[8]=0,c[12]=-f,c[1]=0,c[5]=2*h,c[9]=0,c[13]=-d,c[2]=0,c[6]=0,c[10]=x,c[14]=-g,c[3]=0,c[7]=0,c[11]=0,c[15]=1,this}equals(t){const e=this.elements,n=t.elements;for(let i=0;i<16;i++)if(e[i]!==n[i])return!1;return!0}fromArray(t,e=0){for(let n=0;n<16;n++)this.elements[n]=t[n+e];return this}toArray(t=[],e=0){const n=this.elements;return t[e]=n[0],t[e+1]=n[1],t[e+2]=n[2],t[e+3]=n[3],t[e+4]=n[4],t[e+5]=n[5],t[e+6]=n[6],t[e+7]=n[7],t[e+8]=n[8],t[e+9]=n[9],t[e+10]=n[10],t[e+11]=n[11],t[e+12]=n[12],t[e+13]=n[13],t[e+14]=n[14],t[e+15]=n[15],t}}const qs=new b,Qn=new se,H0=new b(0,0,0),W0=new b(1,1,1),Zi=new b,ua=new b,bn=new b,Td=new se,bd=new Nn;class hn{constructor(t=0,e=0,n=0,i=hn.DEFAULT_ORDER){this.isEuler=!0,this._x=t,this._y=e,this._z=n,this._order=i}get x(){return this._x}set x(t){this._x=t,this._onChangeCallback()}get y(){return this._y}set y(t){this._y=t,this._onChangeCallback()}get z(){return this._z}set z(t){this._z=t,this._onChangeCallback()}get order(){return this._order}set order(t){this._order=t,this._onChangeCallback()}set(t,e,n,i=this._order){return this._x=t,this._y=e,this._z=n,this._order=i,this._onChangeCallback(),this}clone(){return new this.constructor(this._x,this._y,this._z,this._order)}copy(t){return this._x=t._x,this._y=t._y,this._z=t._z,this._order=t._order,this._onChangeCallback(),this}setFromRotationMatrix(t,e=this._order,n=!0){const i=t.elements,s=i[0],a=i[4],o=i[8],c=i[1],l=i[5],h=i[9],u=i[2],f=i[6],d=i[10];switch(e){case"XYZ":this._y=Math.asin(Ge(o,-1,1)),Math.abs(o)<.9999999?(this._x=Math.atan2(-h,d),this._z=Math.atan2(-a,s)):(this._x=Math.atan2(f,l),this._z=0);break;case"YXZ":this._x=Math.asin(-Ge(h,-1,1)),Math.abs(h)<.9999999?(this._y=Math.atan2(o,d),this._z=Math.atan2(c,l)):(this._y=Math.atan2(-u,s),this._z=0);break;case"ZXY":this._x=Math.asin(Ge(f,-1,1)),Math.abs(f)<.9999999?(this._y=Math.atan2(-u,d),this._z=Math.atan2(-a,l)):(this._y=0,this._z=Math.atan2(c,s));break;case"ZYX":this._y=Math.asin(-Ge(u,-1,1)),Math.abs(u)<.9999999?(this._x=Math.atan2(f,d),this._z=Math.atan2(c,s)):(this._x=0,this._z=Math.atan2(-a,l));break;case"YZX":this._z=Math.asin(Ge(c,-1,1)),Math.abs(c)<.9999999?(this._x=Math.atan2(-h,l),this._y=Math.atan2(-u,s)):(this._x=0,this._y=Math.atan2(o,d));break;case"XZY":this._z=Math.asin(-Ge(a,-1,1)),Math.abs(a)<.9999999?(this._x=Math.atan2(f,l),this._y=Math.atan2(o,s)):(this._x=Math.atan2(-h,d),this._y=0);break;default:console.warn("THREE.Euler: .setFromRotationMatrix() encountered an unknown order: "+e)}return this._order=e,n===!0&&this._onChangeCallback(),this}setFromQuaternion(t,e,n){return Td.makeRotationFromQuaternion(t),this.setFromRotationMatrix(Td,e,n)}setFromVector3(t,e=this._order){return this.set(t.x,t.y,t.z,e)}reorder(t){return bd.setFromEuler(this),this.setFromQuaternion(bd,t)}equals(t){return t._x===this._x&&t._y===this._y&&t._z===this._z&&t._order===this._order}fromArray(t){return this._x=t[0],this._y=t[1],this._z=t[2],t[3]!==void 0&&(this._order=t[3]),this._onChangeCallback(),this}toArray(t=[],e=0){return t[e]=this._x,t[e+1]=this._y,t[e+2]=this._z,t[e+3]=this._order,t}_onChange(t){return this._onChangeCallback=t,this}_onChangeCallback(){}*[Symbol.iterator](){yield this._x,yield this._y,yield this._z,yield this._order}}hn.DEFAULT_ORDER="XYZ";class lm{constructor(){this.mask=1}set(t){this.mask=(1<<t|0)>>>0}enable(t){this.mask|=1<<t|0}enableAll(){this.mask=-1}toggle(t){this.mask^=1<<t|0}disable(t){this.mask&=~(1<<t|0)}disableAll(){this.mask=0}test(t){return(this.mask&t.mask)!==0}isEnabled(t){return(this.mask&(1<<t|0))!==0}}let X0=0;const Cd=new b,Zs=new Nn,bi=new se,da=new b,$r=new b,Y0=new b,q0=new Nn,Rd=new b(1,0,0),Pd=new b(0,1,0),Ld=new b(0,0,1),Z0={type:"added"},j0={type:"removed"};class Qe extends Or{constructor(){super(),this.isObject3D=!0,Object.defineProperty(this,"id",{value:X0++}),this.uuid=Gr(),this.name="",this.type="Object3D",this.parent=null,this.children=[],this.up=Qe.DEFAULT_UP.clone();const t=new b,e=new hn,n=new Nn,i=new b(1,1,1);function s(){n.setFromEuler(e,!1)}function a(){e.setFromQuaternion(n,void 0,!1)}e._onChange(s),n._onChange(a),Object.defineProperties(this,{position:{configurable:!0,enumerable:!0,value:t},rotation:{configurable:!0,enumerable:!0,value:e},quaternion:{configurable:!0,enumerable:!0,value:n},scale:{configurable:!0,enumerable:!0,value:i},modelViewMatrix:{value:new se},normalMatrix:{value:new ne}}),this.matrix=new se,this.matrixWorld=new se,this.matrixAutoUpdate=Qe.DEFAULT_MATRIX_AUTO_UPDATE,this.matrixWorldAutoUpdate=Qe.DEFAULT_MATRIX_WORLD_AUTO_UPDATE,this.matrixWorldNeedsUpdate=!1,this.layers=new lm,this.visible=!0,this.castShadow=!1,this.receiveShadow=!1,this.frustumCulled=!0,this.renderOrder=0,this.animations=[],this.userData={}}onBeforeShadow(){}onAfterShadow(){}onBeforeRender(){}onAfterRender(){}applyMatrix4(t){this.matrixAutoUpdate&&this.updateMatrix(),this.matrix.premultiply(t),this.matrix.decompose(this.position,this.quaternion,this.scale)}applyQuaternion(t){return this.quaternion.premultiply(t),this}setRotationFromAxisAngle(t,e){this.quaternion.setFromAxisAngle(t,e)}setRotationFromEuler(t){this.quaternion.setFromEuler(t,!0)}setRotationFromMatrix(t){this.quaternion.setFromRotationMatrix(t)}setRotationFromQuaternion(t){this.quaternion.copy(t)}rotateOnAxis(t,e){return Zs.setFromAxisAngle(t,e),this.quaternion.multiply(Zs),this}rotateOnWorldAxis(t,e){return Zs.setFromAxisAngle(t,e),this.quaternion.premultiply(Zs),this}rotateX(t){return this.rotateOnAxis(Rd,t)}rotateY(t){return this.rotateOnAxis(Pd,t)}rotateZ(t){return this.rotateOnAxis(Ld,t)}translateOnAxis(t,e){return Cd.copy(t).applyQuaternion(this.quaternion),this.position.add(Cd.multiplyScalar(e)),this}translateX(t){return this.translateOnAxis(Rd,t)}translateY(t){return this.translateOnAxis(Pd,t)}translateZ(t){return this.translateOnAxis(Ld,t)}localToWorld(t){return this.updateWorldMatrix(!0,!1),t.applyMatrix4(this.matrixWorld)}worldToLocal(t){return this.updateWorldMatrix(!0,!1),t.applyMatrix4(bi.copy(this.matrixWorld).invert())}lookAt(t,e,n){t.isVector3?da.copy(t):da.set(t,e,n);const i=this.parent;this.updateWorldMatrix(!0,!1),$r.setFromMatrixPosition(this.matrixWorld),this.isCamera||this.isLight?bi.lookAt($r,da,this.up):bi.lookAt(da,$r,this.up),this.quaternion.setFromRotationMatrix(bi),i&&(bi.extractRotation(i.matrixWorld),Zs.setFromRotationMatrix(bi),this.quaternion.premultiply(Zs.invert()))}add(t){if(arguments.length>1){for(let e=0;e<arguments.length;e++)this.add(arguments[e]);return this}return t===this?(console.error("THREE.Object3D.add: object can't be added as a child of itself.",t),this):(t&&t.isObject3D?(t.parent!==null&&t.parent.remove(t),t.parent=this,this.children.push(t),t.dispatchEvent(Z0)):console.error("THREE.Object3D.add: object not an instance of THREE.Object3D.",t),this)}remove(t){if(arguments.length>1){for(let n=0;n<arguments.length;n++)this.remove(arguments[n]);return this}const e=this.children.indexOf(t);return e!==-1&&(t.parent=null,this.children.splice(e,1),t.dispatchEvent(j0)),this}removeFromParent(){const t=this.parent;return t!==null&&t.remove(this),this}clear(){return this.remove(...this.children)}attach(t){return this.updateWorldMatrix(!0,!1),bi.copy(this.matrixWorld).invert(),t.parent!==null&&(t.parent.updateWorldMatrix(!0,!1),bi.multiply(t.parent.matrixWorld)),t.applyMatrix4(bi),this.add(t),t.updateWorldMatrix(!1,!0),this}getObjectById(t){return this.getObjectByProperty("id",t)}getObjectByName(t){return this.getObjectByProperty("name",t)}getObjectByProperty(t,e){if(this[t]===e)return this;for(let n=0,i=this.children.length;n<i;n++){const a=this.children[n].getObjectByProperty(t,e);if(a!==void 0)return a}}getObjectsByProperty(t,e,n=[]){this[t]===e&&n.push(this);const i=this.children;for(let s=0,a=i.length;s<a;s++)i[s].getObjectsByProperty(t,e,n);return n}getWorldPosition(t){return this.updateWorldMatrix(!0,!1),t.setFromMatrixPosition(this.matrixWorld)}getWorldQuaternion(t){return this.updateWorldMatrix(!0,!1),this.matrixWorld.decompose($r,t,Y0),t}getWorldScale(t){return this.updateWorldMatrix(!0,!1),this.matrixWorld.decompose($r,q0,t),t}getWorldDirection(t){this.updateWorldMatrix(!0,!1);const e=this.matrixWorld.elements;return t.set(e[8],e[9],e[10]).normalize()}raycast(){}traverse(t){t(this);const e=this.children;for(let n=0,i=e.length;n<i;n++)e[n].traverse(t)}traverseVisible(t){if(this.visible===!1)return;t(this);const e=this.children;for(let n=0,i=e.length;n<i;n++)e[n].traverseVisible(t)}traverseAncestors(t){const e=this.parent;e!==null&&(t(e),e.traverseAncestors(t))}updateMatrix(){this.matrix.compose(this.position,this.quaternion,this.scale),this.matrixWorldNeedsUpdate=!0}updateMatrixWorld(t){this.matrixAutoUpdate&&this.updateMatrix(),(this.matrixWorldNeedsUpdate||t)&&(this.parent===null?this.matrixWorld.copy(this.matrix):this.matrixWorld.multiplyMatrices(this.parent.matrixWorld,this.matrix),this.matrixWorldNeedsUpdate=!1,t=!0);const e=this.children;for(let n=0,i=e.length;n<i;n++){const s=e[n];(s.matrixWorldAutoUpdate===!0||t===!0)&&s.updateMatrixWorld(t)}}updateWorldMatrix(t,e){const n=this.parent;if(t===!0&&n!==null&&n.matrixWorldAutoUpdate===!0&&n.updateWorldMatrix(!0,!1),this.matrixAutoUpdate&&this.updateMatrix(),this.parent===null?this.matrixWorld.copy(this.matrix):this.matrixWorld.multiplyMatrices(this.parent.matrixWorld,this.matrix),e===!0){const i=this.children;for(let s=0,a=i.length;s<a;s++){const o=i[s];o.matrixWorldAutoUpdate===!0&&o.updateWorldMatrix(!1,!0)}}}toJSON(t){const e=t===void 0||typeof t=="string",n={};e&&(t={geometries:{},materials:{},textures:{},images:{},shapes:{},skeletons:{},animations:{},nodes:{}},n.metadata={version:4.6,type:"Object",generator:"Object3D.toJSON"});const i={};i.uuid=this.uuid,i.type=this.type,this.name!==""&&(i.name=this.name),this.castShadow===!0&&(i.castShadow=!0),this.receiveShadow===!0&&(i.receiveShadow=!0),this.visible===!1&&(i.visible=!1),this.frustumCulled===!1&&(i.frustumCulled=!1),this.renderOrder!==0&&(i.renderOrder=this.renderOrder),Object.keys(this.userData).length>0&&(i.userData=this.userData),i.layers=this.layers.mask,i.matrix=this.matrix.toArray(),i.up=this.up.toArray(),this.matrixAutoUpdate===!1&&(i.matrixAutoUpdate=!1),this.isInstancedMesh&&(i.type="InstancedMesh",i.count=this.count,i.instanceMatrix=this.instanceMatrix.toJSON(),this.instanceColor!==null&&(i.instanceColor=this.instanceColor.toJSON())),this.isBatchedMesh&&(i.type="BatchedMesh",i.perObjectFrustumCulled=this.perObjectFrustumCulled,i.sortObjects=this.sortObjects,i.drawRanges=this._drawRanges,i.reservedRanges=this._reservedRanges,i.visibility=this._visibility,i.active=this._active,i.bounds=this._bounds.map(o=>({boxInitialized:o.boxInitialized,boxMin:o.box.min.toArray(),boxMax:o.box.max.toArray(),sphereInitialized:o.sphereInitialized,sphereRadius:o.sphere.radius,sphereCenter:o.sphere.center.toArray()})),i.maxGeometryCount=this._maxGeometryCount,i.maxVertexCount=this._maxVertexCount,i.maxIndexCount=this._maxIndexCount,i.geometryInitialized=this._geometryInitialized,i.geometryCount=this._geometryCount,i.matricesTexture=this._matricesTexture.toJSON(t),this.boundingSphere!==null&&(i.boundingSphere={center:i.boundingSphere.center.toArray(),radius:i.boundingSphere.radius}),this.boundingBox!==null&&(i.boundingBox={min:i.boundingBox.min.toArray(),max:i.boundingBox.max.toArray()}));function s(o,c){return o[c.uuid]===void 0&&(o[c.uuid]=c.toJSON(t)),c.uuid}if(this.isScene)this.background&&(this.background.isColor?i.background=this.background.toJSON():this.background.isTexture&&(i.background=this.background.toJSON(t).uuid)),this.environment&&this.environment.isTexture&&this.environment.isRenderTargetTexture!==!0&&(i.environment=this.environment.toJSON(t).uuid);else if(this.isMesh||this.isLine||this.isPoints){i.geometry=s(t.geometries,this.geometry);const o=this.geometry.parameters;if(o!==void 0&&o.shapes!==void 0){const c=o.shapes;if(Array.isArray(c))for(let l=0,h=c.length;l<h;l++){const u=c[l];s(t.shapes,u)}else s(t.shapes,c)}}if(this.isSkinnedMesh&&(i.bindMode=this.bindMode,i.bindMatrix=this.bindMatrix.toArray(),this.skeleton!==void 0&&(s(t.skeletons,this.skeleton),i.skeleton=this.skeleton.uuid)),this.material!==void 0)if(Array.isArray(this.material)){const o=[];for(let c=0,l=this.material.length;c<l;c++)o.push(s(t.materials,this.material[c]));i.material=o}else i.material=s(t.materials,this.material);if(this.children.length>0){i.children=[];for(let o=0;o<this.children.length;o++)i.children.push(this.children[o].toJSON(t).object)}if(this.animations.length>0){i.animations=[];for(let o=0;o<this.animations.length;o++){const c=this.animations[o];i.animations.push(s(t.animations,c))}}if(e){const o=a(t.geometries),c=a(t.materials),l=a(t.textures),h=a(t.images),u=a(t.shapes),f=a(t.skeletons),d=a(t.animations),g=a(t.nodes);o.length>0&&(n.geometries=o),c.length>0&&(n.materials=c),l.length>0&&(n.textures=l),h.length>0&&(n.images=h),u.length>0&&(n.shapes=u),f.length>0&&(n.skeletons=f),d.length>0&&(n.animations=d),g.length>0&&(n.nodes=g)}return n.object=i,n;function a(o){const c=[];for(const l in o){const h=o[l];delete h.metadata,c.push(h)}return c}}clone(t){return new this.constructor().copy(this,t)}copy(t,e=!0){if(this.name=t.name,this.up.copy(t.up),this.position.copy(t.position),this.rotation.order=t.rotation.order,this.quaternion.copy(t.quaternion),this.scale.copy(t.scale),this.matrix.copy(t.matrix),this.matrixWorld.copy(t.matrixWorld),this.matrixAutoUpdate=t.matrixAutoUpdate,this.matrixWorldAutoUpdate=t.matrixWorldAutoUpdate,this.matrixWorldNeedsUpdate=t.matrixWorldNeedsUpdate,this.layers.mask=t.layers.mask,this.visible=t.visible,this.castShadow=t.castShadow,this.receiveShadow=t.receiveShadow,this.frustumCulled=t.frustumCulled,this.renderOrder=t.renderOrder,this.animations=t.animations.slice(),this.userData=JSON.parse(JSON.stringify(t.userData)),e===!0)for(let n=0;n<t.children.length;n++){const i=t.children[n];this.add(i.clone())}return this}}Qe.DEFAULT_UP=new b(0,1,0);Qe.DEFAULT_MATRIX_AUTO_UPDATE=!0;Qe.DEFAULT_MATRIX_WORLD_AUTO_UPDATE=!0;const ti=new b,Ci=new b,pl=new b,Ri=new b,js=new b,$s=new b,Id=new b,ml=new b,gl=new b,xl=new b;let fa=!1;class rn{constructor(t=new b,e=new b,n=new b){this.a=t,this.b=e,this.c=n}static getNormal(t,e,n,i){i.subVectors(n,e),ti.subVectors(t,e),i.cross(ti);const s=i.lengthSq();return s>0?i.multiplyScalar(1/Math.sqrt(s)):i.set(0,0,0)}static getBarycoord(t,e,n,i,s){ti.subVectors(i,e),Ci.subVectors(n,e),pl.subVectors(t,e);const a=ti.dot(ti),o=ti.dot(Ci),c=ti.dot(pl),l=Ci.dot(Ci),h=Ci.dot(pl),u=a*l-o*o;if(u===0)return s.set(0,0,0),null;const f=1/u,d=(l*c-o*h)*f,g=(a*h-o*c)*f;return s.set(1-d-g,g,d)}static containsPoint(t,e,n,i){return this.getBarycoord(t,e,n,i,Ri)===null?!1:Ri.x>=0&&Ri.y>=0&&Ri.x+Ri.y<=1}static getUV(t,e,n,i,s,a,o,c){return fa===!1&&(console.warn("THREE.Triangle.getUV() has been renamed to THREE.Triangle.getInterpolation()."),fa=!0),this.getInterpolation(t,e,n,i,s,a,o,c)}static getInterpolation(t,e,n,i,s,a,o,c){return this.getBarycoord(t,e,n,i,Ri)===null?(c.x=0,c.y=0,"z"in c&&(c.z=0),"w"in c&&(c.w=0),null):(c.setScalar(0),c.addScaledVector(s,Ri.x),c.addScaledVector(a,Ri.y),c.addScaledVector(o,Ri.z),c)}static isFrontFacing(t,e,n,i){return ti.subVectors(n,e),Ci.subVectors(t,e),ti.cross(Ci).dot(i)<0}set(t,e,n){return this.a.copy(t),this.b.copy(e),this.c.copy(n),this}setFromPointsAndIndices(t,e,n,i){return this.a.copy(t[e]),this.b.copy(t[n]),this.c.copy(t[i]),this}setFromAttributeAndIndices(t,e,n,i){return this.a.fromBufferAttribute(t,e),this.b.fromBufferAttribute(t,n),this.c.fromBufferAttribute(t,i),this}clone(){return new this.constructor().copy(this)}copy(t){return this.a.copy(t.a),this.b.copy(t.b),this.c.copy(t.c),this}getArea(){return ti.subVectors(this.c,this.b),Ci.subVectors(this.a,this.b),ti.cross(Ci).length()*.5}getMidpoint(t){return t.addVectors(this.a,this.b).add(this.c).multiplyScalar(1/3)}getNormal(t){return rn.getNormal(this.a,this.b,this.c,t)}getPlane(t){return t.setFromCoplanarPoints(this.a,this.b,this.c)}getBarycoord(t,e){return rn.getBarycoord(t,this.a,this.b,this.c,e)}getUV(t,e,n,i,s){return fa===!1&&(console.warn("THREE.Triangle.getUV() has been renamed to THREE.Triangle.getInterpolation()."),fa=!0),rn.getInterpolation(t,this.a,this.b,this.c,e,n,i,s)}getInterpolation(t,e,n,i,s){return rn.getInterpolation(t,this.a,this.b,this.c,e,n,i,s)}containsPoint(t){return rn.containsPoint(t,this.a,this.b,this.c)}isFrontFacing(t){return rn.isFrontFacing(this.a,this.b,this.c,t)}intersectsBox(t){return t.intersectsTriangle(this)}closestPointToPoint(t,e){const n=this.a,i=this.b,s=this.c;let a,o;js.subVectors(i,n),$s.subVectors(s,n),ml.subVectors(t,n);const c=js.dot(ml),l=$s.dot(ml);if(c<=0&&l<=0)return e.copy(n);gl.subVectors(t,i);const h=js.dot(gl),u=$s.dot(gl);if(h>=0&&u<=h)return e.copy(i);const f=c*u-h*l;if(f<=0&&c>=0&&h<=0)return a=c/(c-h),e.copy(n).addScaledVector(js,a);xl.subVectors(t,s);const d=js.dot(xl),g=$s.dot(xl);if(g>=0&&d<=g)return e.copy(s);const x=d*l-c*g;if(x<=0&&l>=0&&g<=0)return o=l/(l-g),e.copy(n).addScaledVector($s,o);const m=h*g-d*u;if(m<=0&&u-h>=0&&d-g>=0)return Id.subVectors(s,i),o=(u-h)/(u-h+(d-g)),e.copy(i).addScaledVector(Id,o);const p=1/(m+x+f);return a=x*p,o=f*p,e.copy(n).addScaledVector(js,a).addScaledVector($s,o)}equals(t){return t.a.equals(this.a)&&t.b.equals(this.b)&&t.c.equals(this.c)}}const hm={aliceblue:15792383,antiquewhite:16444375,aqua:65535,aquamarine:8388564,azure:15794175,beige:16119260,bisque:16770244,black:0,blanchedalmond:16772045,blue:255,blueviolet:9055202,brown:10824234,burlywood:14596231,cadetblue:6266528,chartreuse:8388352,chocolate:13789470,coral:16744272,cornflowerblue:6591981,cornsilk:16775388,crimson:14423100,cyan:65535,darkblue:139,darkcyan:35723,darkgoldenrod:12092939,darkgray:11119017,darkgreen:25600,darkgrey:11119017,darkkhaki:12433259,darkmagenta:9109643,darkolivegreen:5597999,darkorange:16747520,darkorchid:10040012,darkred:9109504,darksalmon:15308410,darkseagreen:9419919,darkslateblue:4734347,darkslategray:3100495,darkslategrey:3100495,darkturquoise:52945,darkviolet:9699539,deeppink:16716947,deepskyblue:49151,dimgray:6908265,dimgrey:6908265,dodgerblue:2003199,firebrick:11674146,floralwhite:16775920,forestgreen:2263842,fuchsia:16711935,gainsboro:14474460,ghostwhite:16316671,gold:16766720,goldenrod:14329120,gray:8421504,green:32768,greenyellow:11403055,grey:8421504,honeydew:15794160,hotpink:16738740,indianred:13458524,indigo:4915330,ivory:16777200,khaki:15787660,lavender:15132410,lavenderblush:16773365,lawngreen:8190976,lemonchiffon:16775885,lightblue:11393254,lightcoral:15761536,lightcyan:14745599,lightgoldenrodyellow:16448210,lightgray:13882323,lightgreen:9498256,lightgrey:13882323,lightpink:16758465,lightsalmon:16752762,lightseagreen:2142890,lightskyblue:8900346,lightslategray:7833753,lightslategrey:7833753,lightsteelblue:11584734,lightyellow:16777184,lime:65280,limegreen:3329330,linen:16445670,magenta:16711935,maroon:8388608,mediumaquamarine:6737322,mediumblue:205,mediumorchid:12211667,mediumpurple:9662683,mediumseagreen:3978097,mediumslateblue:8087790,mediumspringgreen:64154,mediumturquoise:4772300,mediumvioletred:13047173,midnightblue:1644912,mintcream:16121850,mistyrose:16770273,moccasin:16770229,navajowhite:16768685,navy:128,oldlace:16643558,olive:8421376,olivedrab:7048739,orange:16753920,orangered:16729344,orchid:14315734,palegoldenrod:15657130,palegreen:10025880,paleturquoise:11529966,palevioletred:14381203,papayawhip:16773077,peachpuff:16767673,peru:13468991,pink:16761035,plum:14524637,powderblue:11591910,purple:8388736,rebeccapurple:6697881,red:16711680,rosybrown:12357519,royalblue:4286945,saddlebrown:9127187,salmon:16416882,sandybrown:16032864,seagreen:3050327,seashell:16774638,sienna:10506797,silver:12632256,skyblue:8900331,slateblue:6970061,slategray:7372944,slategrey:7372944,snow:16775930,springgreen:65407,steelblue:4620980,tan:13808780,teal:32896,thistle:14204888,tomato:16737095,turquoise:4251856,violet:15631086,wheat:16113331,white:16777215,whitesmoke:16119285,yellow:16776960,yellowgreen:10145074},ji={h:0,s:0,l:0},pa={h:0,s:0,l:0};function _l(r,t,e){return e<0&&(e+=1),e>1&&(e-=1),e<1/6?r+(t-r)*6*e:e<1/2?t:e<2/3?r+(t-r)*6*(2/3-e):r}class jt{constructor(t,e,n){return this.isColor=!0,this.r=1,this.g=1,this.b=1,this.set(t,e,n)}set(t,e,n){if(e===void 0&&n===void 0){const i=t;i&&i.isColor?this.copy(i):typeof i=="number"?this.setHex(i):typeof i=="string"&&this.setStyle(i)}else this.setRGB(t,e,n);return this}setScalar(t){return this.r=t,this.g=t,this.b=t,this}setHex(t,e=sn){return t=Math.floor(t),this.r=(t>>16&255)/255,this.g=(t>>8&255)/255,this.b=(t&255)/255,me.toWorkingColorSpace(this,e),this}setRGB(t,e,n,i=me.workingColorSpace){return this.r=t,this.g=e,this.b=n,me.toWorkingColorSpace(this,i),this}setHSL(t,e,n,i=me.workingColorSpace){if(t=fu(t,1),e=Ge(e,0,1),n=Ge(n,0,1),e===0)this.r=this.g=this.b=n;else{const s=n<=.5?n*(1+e):n+e-n*e,a=2*n-s;this.r=_l(a,s,t+1/3),this.g=_l(a,s,t),this.b=_l(a,s,t-1/3)}return me.toWorkingColorSpace(this,i),this}setStyle(t,e=sn){function n(s){s!==void 0&&parseFloat(s)<1&&console.warn("THREE.Color: Alpha component of "+t+" will be ignored.")}let i;if(i=/^(\w+)\(([^\)]*)\)/.exec(t)){let s;const a=i[1],o=i[2];switch(a){case"rgb":case"rgba":if(s=/^\s*(\d+)\s*,\s*(\d+)\s*,\s*(\d+)\s*(?:,\s*(\d*\.?\d+)\s*)?$/.exec(o))return n(s[4]),this.setRGB(Math.min(255,parseInt(s[1],10))/255,Math.min(255,parseInt(s[2],10))/255,Math.min(255,parseInt(s[3],10))/255,e);if(s=/^\s*(\d+)\%\s*,\s*(\d+)\%\s*,\s*(\d+)\%\s*(?:,\s*(\d*\.?\d+)\s*)?$/.exec(o))return n(s[4]),this.setRGB(Math.min(100,parseInt(s[1],10))/100,Math.min(100,parseInt(s[2],10))/100,Math.min(100,parseInt(s[3],10))/100,e);break;case"hsl":case"hsla":if(s=/^\s*(\d*\.?\d+)\s*,\s*(\d*\.?\d+)\%\s*,\s*(\d*\.?\d+)\%\s*(?:,\s*(\d*\.?\d+)\s*)?$/.exec(o))return n(s[4]),this.setHSL(parseFloat(s[1])/360,parseFloat(s[2])/100,parseFloat(s[3])/100,e);break;default:console.warn("THREE.Color: Unknown color model "+t)}}else if(i=/^\#([A-Fa-f\d]+)$/.exec(t)){const s=i[1],a=s.length;if(a===3)return this.setRGB(parseInt(s.charAt(0),16)/15,parseInt(s.charAt(1),16)/15,parseInt(s.charAt(2),16)/15,e);if(a===6)return this.setHex(parseInt(s,16),e);console.warn("THREE.Color: Invalid hex color "+t)}else if(t&&t.length>0)return this.setColorName(t,e);return this}setColorName(t,e=sn){const n=hm[t.toLowerCase()];return n!==void 0?this.setHex(n,e):console.warn("THREE.Color: Unknown color "+t),this}clone(){return new this.constructor(this.r,this.g,this.b)}copy(t){return this.r=t.r,this.g=t.g,this.b=t.b,this}copySRGBToLinear(t){return this.r=br(t.r),this.g=br(t.g),this.b=br(t.b),this}copyLinearToSRGB(t){return this.r=ol(t.r),this.g=ol(t.g),this.b=ol(t.b),this}convertSRGBToLinear(){return this.copySRGBToLinear(this),this}convertLinearToSRGB(){return this.copyLinearToSRGB(this),this}getHex(t=sn){return me.fromWorkingColorSpace(an.copy(this),t),Math.round(Ge(an.r*255,0,255))*65536+Math.round(Ge(an.g*255,0,255))*256+Math.round(Ge(an.b*255,0,255))}getHexString(t=sn){return("000000"+this.getHex(t).toString(16)).slice(-6)}getHSL(t,e=me.workingColorSpace){me.fromWorkingColorSpace(an.copy(this),e);const n=an.r,i=an.g,s=an.b,a=Math.max(n,i,s),o=Math.min(n,i,s);let c,l;const h=(o+a)/2;if(o===a)c=0,l=0;else{const u=a-o;switch(l=h<=.5?u/(a+o):u/(2-a-o),a){case n:c=(i-s)/u+(i<s?6:0);break;case i:c=(s-n)/u+2;break;case s:c=(n-i)/u+4;break}c/=6}return t.h=c,t.s=l,t.l=h,t}getRGB(t,e=me.workingColorSpace){return me.fromWorkingColorSpace(an.copy(this),e),t.r=an.r,t.g=an.g,t.b=an.b,t}getStyle(t=sn){me.fromWorkingColorSpace(an.copy(this),t);const e=an.r,n=an.g,i=an.b;return t!==sn?`color(${t} ${e.toFixed(3)} ${n.toFixed(3)} ${i.toFixed(3)})`:`rgb(${Math.round(e*255)},${Math.round(n*255)},${Math.round(i*255)})`}offsetHSL(t,e,n){return this.getHSL(ji),this.setHSL(ji.h+t,ji.s+e,ji.l+n)}add(t){return this.r+=t.r,this.g+=t.g,this.b+=t.b,this}addColors(t,e){return this.r=t.r+e.r,this.g=t.g+e.g,this.b=t.b+e.b,this}addScalar(t){return this.r+=t,this.g+=t,this.b+=t,this}sub(t){return this.r=Math.max(0,this.r-t.r),this.g=Math.max(0,this.g-t.g),this.b=Math.max(0,this.b-t.b),this}multiply(t){return this.r*=t.r,this.g*=t.g,this.b*=t.b,this}multiplyScalar(t){return this.r*=t,this.g*=t,this.b*=t,this}lerp(t,e){return this.r+=(t.r-this.r)*e,this.g+=(t.g-this.g)*e,this.b+=(t.b-this.b)*e,this}lerpColors(t,e,n){return this.r=t.r+(e.r-t.r)*n,this.g=t.g+(e.g-t.g)*n,this.b=t.b+(e.b-t.b)*n,this}lerpHSL(t,e){this.getHSL(ji),t.getHSL(pa);const n=Go(ji.h,pa.h,e),i=Go(ji.s,pa.s,e),s=Go(ji.l,pa.l,e);return this.setHSL(n,i,s),this}setFromVector3(t){return this.r=t.x,this.g=t.y,this.b=t.z,this}applyMatrix3(t){const e=this.r,n=this.g,i=this.b,s=t.elements;return this.r=s[0]*e+s[3]*n+s[6]*i,this.g=s[1]*e+s[4]*n+s[7]*i,this.b=s[2]*e+s[5]*n+s[8]*i,this}equals(t){return t.r===this.r&&t.g===this.g&&t.b===this.b}fromArray(t,e=0){return this.r=t[e],this.g=t[e+1],this.b=t[e+2],this}toArray(t=[],e=0){return t[e]=this.r,t[e+1]=this.g,t[e+2]=this.b,t}fromBufferAttribute(t,e){return this.r=t.getX(e),this.g=t.getY(e),this.b=t.getZ(e),this}toJSON(){return this.getHex()}*[Symbol.iterator](){yield this.r,yield this.g,yield this.b}}const an=new jt;jt.NAMES=hm;let $0=0;class Si extends Or{constructor(){super(),this.isMaterial=!0,Object.defineProperty(this,"id",{value:$0++}),this.uuid=Gr(),this.name="",this.type="Material",this.blending=Tr,this.side=oi,this.vertexColors=!1,this.opacity=1,this.transparent=!1,this.alphaHash=!1,this.blendSrc=wh,this.blendDst=Ah,this.blendEquation=Bi,this.blendSrcAlpha=null,this.blendDstAlpha=null,this.blendEquationAlpha=null,this.blendColor=new jt(0,0,0),this.blendAlpha=0,this.depthFunc=xc,this.depthTest=!0,this.depthWrite=!0,this.stencilWriteMask=255,this.stencilFunc=_d,this.stencilRef=0,this.stencilFuncMask=255,this.stencilFail=ks,this.stencilZFail=ks,this.stencilZPass=ks,this.stencilWrite=!1,this.clippingPlanes=null,this.clipIntersection=!1,this.clipShadows=!1,this.shadowSide=null,this.colorWrite=!0,this.precision=null,this.polygonOffset=!1,this.polygonOffsetFactor=0,this.polygonOffsetUnits=0,this.dithering=!1,this.alphaToCoverage=!1,this.premultipliedAlpha=!1,this.forceSinglePass=!1,this.visible=!0,this.toneMapped=!0,this.userData={},this.version=0,this._alphaTest=0}get alphaTest(){return this._alphaTest}set alphaTest(t){this._alphaTest>0!=t>0&&this.version++,this._alphaTest=t}onBuild(){}onBeforeRender(){}onBeforeCompile(){}customProgramCacheKey(){return this.onBeforeCompile.toString()}setValues(t){if(t!==void 0)for(const e in t){const n=t[e];if(n===void 0){console.warn(`THREE.Material: parameter '${e}' has value of undefined.`);continue}const i=this[e];if(i===void 0){console.warn(`THREE.Material: '${e}' is not a property of THREE.${this.type}.`);continue}i&&i.isColor?i.set(n):i&&i.isVector3&&n&&n.isVector3?i.copy(n):this[e]=n}}toJSON(t){const e=t===void 0||typeof t=="string";e&&(t={textures:{},images:{}});const n={metadata:{version:4.6,type:"Material",generator:"Material.toJSON"}};n.uuid=this.uuid,n.type=this.type,this.name!==""&&(n.name=this.name),this.color&&this.color.isColor&&(n.color=this.color.getHex()),this.roughness!==void 0&&(n.roughness=this.roughness),this.metalness!==void 0&&(n.metalness=this.metalness),this.sheen!==void 0&&(n.sheen=this.sheen),this.sheenColor&&this.sheenColor.isColor&&(n.sheenColor=this.sheenColor.getHex()),this.sheenRoughness!==void 0&&(n.sheenRoughness=this.sheenRoughness),this.emissive&&this.emissive.isColor&&(n.emissive=this.emissive.getHex()),this.emissiveIntensity&&this.emissiveIntensity!==1&&(n.emissiveIntensity=this.emissiveIntensity),this.specular&&this.specular.isColor&&(n.specular=this.specular.getHex()),this.specularIntensity!==void 0&&(n.specularIntensity=this.specularIntensity),this.specularColor&&this.specularColor.isColor&&(n.specularColor=this.specularColor.getHex()),this.shininess!==void 0&&(n.shininess=this.shininess),this.clearcoat!==void 0&&(n.clearcoat=this.clearcoat),this.clearcoatRoughness!==void 0&&(n.clearcoatRoughness=this.clearcoatRoughness),this.clearcoatMap&&this.clearcoatMap.isTexture&&(n.clearcoatMap=this.clearcoatMap.toJSON(t).uuid),this.clearcoatRoughnessMap&&this.clearcoatRoughnessMap.isTexture&&(n.clearcoatRoughnessMap=this.clearcoatRoughnessMap.toJSON(t).uuid),this.clearcoatNormalMap&&this.clearcoatNormalMap.isTexture&&(n.clearcoatNormalMap=this.clearcoatNormalMap.toJSON(t).uuid,n.clearcoatNormalScale=this.clearcoatNormalScale.toArray()),this.iridescence!==void 0&&(n.iridescence=this.iridescence),this.iridescenceIOR!==void 0&&(n.iridescenceIOR=this.iridescenceIOR),this.iridescenceThicknessRange!==void 0&&(n.iridescenceThicknessRange=this.iridescenceThicknessRange),this.iridescenceMap&&this.iridescenceMap.isTexture&&(n.iridescenceMap=this.iridescenceMap.toJSON(t).uuid),this.iridescenceThicknessMap&&this.iridescenceThicknessMap.isTexture&&(n.iridescenceThicknessMap=this.iridescenceThicknessMap.toJSON(t).uuid),this.anisotropy!==void 0&&(n.anisotropy=this.anisotropy),this.anisotropyRotation!==void 0&&(n.anisotropyRotation=this.anisotropyRotation),this.anisotropyMap&&this.anisotropyMap.isTexture&&(n.anisotropyMap=this.anisotropyMap.toJSON(t).uuid),this.map&&this.map.isTexture&&(n.map=this.map.toJSON(t).uuid),this.matcap&&this.matcap.isTexture&&(n.matcap=this.matcap.toJSON(t).uuid),this.alphaMap&&this.alphaMap.isTexture&&(n.alphaMap=this.alphaMap.toJSON(t).uuid),this.lightMap&&this.lightMap.isTexture&&(n.lightMap=this.lightMap.toJSON(t).uuid,n.lightMapIntensity=this.lightMapIntensity),this.aoMap&&this.aoMap.isTexture&&(n.aoMap=this.aoMap.toJSON(t).uuid,n.aoMapIntensity=this.aoMapIntensity),this.bumpMap&&this.bumpMap.isTexture&&(n.bumpMap=this.bumpMap.toJSON(t).uuid,n.bumpScale=this.bumpScale),this.normalMap&&this.normalMap.isTexture&&(n.normalMap=this.normalMap.toJSON(t).uuid,n.normalMapType=this.normalMapType,n.normalScale=this.normalScale.toArray()),this.displacementMap&&this.displacementMap.isTexture&&(n.displacementMap=this.displacementMap.toJSON(t).uuid,n.displacementScale=this.displacementScale,n.displacementBias=this.displacementBias),this.roughnessMap&&this.roughnessMap.isTexture&&(n.roughnessMap=this.roughnessMap.toJSON(t).uuid),this.metalnessMap&&this.metalnessMap.isTexture&&(n.metalnessMap=this.metalnessMap.toJSON(t).uuid),this.emissiveMap&&this.emissiveMap.isTexture&&(n.emissiveMap=this.emissiveMap.toJSON(t).uuid),this.specularMap&&this.specularMap.isTexture&&(n.specularMap=this.specularMap.toJSON(t).uuid),this.specularIntensityMap&&this.specularIntensityMap.isTexture&&(n.specularIntensityMap=this.specularIntensityMap.toJSON(t).uuid),this.specularColorMap&&this.specularColorMap.isTexture&&(n.specularColorMap=this.specularColorMap.toJSON(t).uuid),this.envMap&&this.envMap.isTexture&&(n.envMap=this.envMap.toJSON(t).uuid,this.combine!==void 0&&(n.combine=this.combine)),this.envMapIntensity!==void 0&&(n.envMapIntensity=this.envMapIntensity),this.reflectivity!==void 0&&(n.reflectivity=this.reflectivity),this.refractionRatio!==void 0&&(n.refractionRatio=this.refractionRatio),this.gradientMap&&this.gradientMap.isTexture&&(n.gradientMap=this.gradientMap.toJSON(t).uuid),this.transmission!==void 0&&(n.transmission=this.transmission),this.transmissionMap&&this.transmissionMap.isTexture&&(n.transmissionMap=this.transmissionMap.toJSON(t).uuid),this.thickness!==void 0&&(n.thickness=this.thickness),this.thicknessMap&&this.thicknessMap.isTexture&&(n.thicknessMap=this.thicknessMap.toJSON(t).uuid),this.attenuationDistance!==void 0&&this.attenuationDistance!==1/0&&(n.attenuationDistance=this.attenuationDistance),this.attenuationColor!==void 0&&(n.attenuationColor=this.attenuationColor.getHex()),this.size!==void 0&&(n.size=this.size),this.shadowSide!==null&&(n.shadowSide=this.shadowSide),this.sizeAttenuation!==void 0&&(n.sizeAttenuation=this.sizeAttenuation),this.blending!==Tr&&(n.blending=this.blending),this.side!==oi&&(n.side=this.side),this.vertexColors===!0&&(n.vertexColors=!0),this.opacity<1&&(n.opacity=this.opacity),this.transparent===!0&&(n.transparent=!0),this.blendSrc!==wh&&(n.blendSrc=this.blendSrc),this.blendDst!==Ah&&(n.blendDst=this.blendDst),this.blendEquation!==Bi&&(n.blendEquation=this.blendEquation),this.blendSrcAlpha!==null&&(n.blendSrcAlpha=this.blendSrcAlpha),this.blendDstAlpha!==null&&(n.blendDstAlpha=this.blendDstAlpha),this.blendEquationAlpha!==null&&(n.blendEquationAlpha=this.blendEquationAlpha),this.blendColor&&this.blendColor.isColor&&(n.blendColor=this.blendColor.getHex()),this.blendAlpha!==0&&(n.blendAlpha=this.blendAlpha),this.depthFunc!==xc&&(n.depthFunc=this.depthFunc),this.depthTest===!1&&(n.depthTest=this.depthTest),this.depthWrite===!1&&(n.depthWrite=this.depthWrite),this.colorWrite===!1&&(n.colorWrite=this.colorWrite),this.stencilWriteMask!==255&&(n.stencilWriteMask=this.stencilWriteMask),this.stencilFunc!==_d&&(n.stencilFunc=this.stencilFunc),this.stencilRef!==0&&(n.stencilRef=this.stencilRef),this.stencilFuncMask!==255&&(n.stencilFuncMask=this.stencilFuncMask),this.stencilFail!==ks&&(n.stencilFail=this.stencilFail),this.stencilZFail!==ks&&(n.stencilZFail=this.stencilZFail),this.stencilZPass!==ks&&(n.stencilZPass=this.stencilZPass),this.stencilWrite===!0&&(n.stencilWrite=this.stencilWrite),this.rotation!==void 0&&this.rotation!==0&&(n.rotation=this.rotation),this.polygonOffset===!0&&(n.polygonOffset=!0),this.polygonOffsetFactor!==0&&(n.polygonOffsetFactor=this.polygonOffsetFactor),this.polygonOffsetUnits!==0&&(n.polygonOffsetUnits=this.polygonOffsetUnits),this.linewidth!==void 0&&this.linewidth!==1&&(n.linewidth=this.linewidth),this.dashSize!==void 0&&(n.dashSize=this.dashSize),this.gapSize!==void 0&&(n.gapSize=this.gapSize),this.scale!==void 0&&(n.scale=this.scale),this.dithering===!0&&(n.dithering=!0),this.alphaTest>0&&(n.alphaTest=this.alphaTest),this.alphaHash===!0&&(n.alphaHash=!0),this.alphaToCoverage===!0&&(n.alphaToCoverage=!0),this.premultipliedAlpha===!0&&(n.premultipliedAlpha=!0),this.forceSinglePass===!0&&(n.forceSinglePass=!0),this.wireframe===!0&&(n.wireframe=!0),this.wireframeLinewidth>1&&(n.wireframeLinewidth=this.wireframeLinewidth),this.wireframeLinecap!=="round"&&(n.wireframeLinecap=this.wireframeLinecap),this.wireframeLinejoin!=="round"&&(n.wireframeLinejoin=this.wireframeLinejoin),this.flatShading===!0&&(n.flatShading=!0),this.visible===!1&&(n.visible=!1),this.toneMapped===!1&&(n.toneMapped=!1),this.fog===!1&&(n.fog=!1),Object.keys(this.userData).length>0&&(n.userData=this.userData);function i(s){const a=[];for(const o in s){const c=s[o];delete c.metadata,a.push(c)}return a}if(e){const s=i(t.textures),a=i(t.images);s.length>0&&(n.textures=s),a.length>0&&(n.images=a)}return n}clone(){return new this.constructor().copy(this)}copy(t){this.name=t.name,this.blending=t.blending,this.side=t.side,this.vertexColors=t.vertexColors,this.opacity=t.opacity,this.transparent=t.transparent,this.blendSrc=t.blendSrc,this.blendDst=t.blendDst,this.blendEquation=t.blendEquation,this.blendSrcAlpha=t.blendSrcAlpha,this.blendDstAlpha=t.blendDstAlpha,this.blendEquationAlpha=t.blendEquationAlpha,this.blendColor.copy(t.blendColor),this.blendAlpha=t.blendAlpha,this.depthFunc=t.depthFunc,this.depthTest=t.depthTest,this.depthWrite=t.depthWrite,this.stencilWriteMask=t.stencilWriteMask,this.stencilFunc=t.stencilFunc,this.stencilRef=t.stencilRef,this.stencilFuncMask=t.stencilFuncMask,this.stencilFail=t.stencilFail,this.stencilZFail=t.stencilZFail,this.stencilZPass=t.stencilZPass,this.stencilWrite=t.stencilWrite;const e=t.clippingPlanes;let n=null;if(e!==null){const i=e.length;n=new Array(i);for(let s=0;s!==i;++s)n[s]=e[s].clone()}return this.clippingPlanes=n,this.clipIntersection=t.clipIntersection,this.clipShadows=t.clipShadows,this.shadowSide=t.shadowSide,this.colorWrite=t.colorWrite,this.precision=t.precision,this.polygonOffset=t.polygonOffset,this.polygonOffsetFactor=t.polygonOffsetFactor,this.polygonOffsetUnits=t.polygonOffsetUnits,this.dithering=t.dithering,this.alphaTest=t.alphaTest,this.alphaHash=t.alphaHash,this.alphaToCoverage=t.alphaToCoverage,this.premultipliedAlpha=t.premultipliedAlpha,this.forceSinglePass=t.forceSinglePass,this.visible=t.visible,this.toneMapped=t.toneMapped,this.userData=JSON.parse(JSON.stringify(t.userData)),this}dispose(){this.dispatchEvent({type:"dispose"})}set needsUpdate(t){t===!0&&this.version++}}class Fe extends Si{constructor(t){super(),this.isMeshBasicMaterial=!0,this.type="MeshBasicMaterial",this.color=new jt(16777215),this.map=null,this.lightMap=null,this.lightMapIntensity=1,this.aoMap=null,this.aoMapIntensity=1,this.specularMap=null,this.alphaMap=null,this.envMap=null,this.combine=lu,this.reflectivity=1,this.refractionRatio=.98,this.wireframe=!1,this.wireframeLinewidth=1,this.wireframeLinecap="round",this.wireframeLinejoin="round",this.fog=!0,this.setValues(t)}copy(t){return super.copy(t),this.color.copy(t.color),this.map=t.map,this.lightMap=t.lightMap,this.lightMapIntensity=t.lightMapIntensity,this.aoMap=t.aoMap,this.aoMapIntensity=t.aoMapIntensity,this.specularMap=t.specularMap,this.alphaMap=t.alphaMap,this.envMap=t.envMap,this.combine=t.combine,this.reflectivity=t.reflectivity,this.refractionRatio=t.refractionRatio,this.wireframe=t.wireframe,this.wireframeLinewidth=t.wireframeLinewidth,this.wireframeLinecap=t.wireframeLinecap,this.wireframeLinejoin=t.wireframeLinejoin,this.fog=t.fog,this}}const ze=new b,ma=new St;class Ne{constructor(t,e,n=!1){if(Array.isArray(t))throw new TypeError("THREE.BufferAttribute: array should be a Typed Array.");this.isBufferAttribute=!0,this.name="",this.array=t,this.itemSize=e,this.count=t!==void 0?t.length/e:0,this.normalized=n,this.usage=vd,this._updateRange={offset:0,count:-1},this.updateRanges=[],this.gpuType=is,this.version=0}onUploadCallback(){}set needsUpdate(t){t===!0&&this.version++}get updateRange(){return console.warn("THREE.BufferAttribute: updateRange() is deprecated and will be removed in r169. Use addUpdateRange() instead."),this._updateRange}setUsage(t){return this.usage=t,this}addUpdateRange(t,e){this.updateRanges.push({start:t,count:e})}clearUpdateRanges(){this.updateRanges.length=0}copy(t){return this.name=t.name,this.array=new t.array.constructor(t.array),this.itemSize=t.itemSize,this.count=t.count,this.normalized=t.normalized,this.usage=t.usage,this.gpuType=t.gpuType,this}copyAt(t,e,n){t*=this.itemSize,n*=e.itemSize;for(let i=0,s=this.itemSize;i<s;i++)this.array[t+i]=e.array[n+i];return this}copyArray(t){return this.array.set(t),this}applyMatrix3(t){if(this.itemSize===2)for(let e=0,n=this.count;e<n;e++)ma.fromBufferAttribute(this,e),ma.applyMatrix3(t),this.setXY(e,ma.x,ma.y);else if(this.itemSize===3)for(let e=0,n=this.count;e<n;e++)ze.fromBufferAttribute(this,e),ze.applyMatrix3(t),this.setXYZ(e,ze.x,ze.y,ze.z);return this}applyMatrix4(t){for(let e=0,n=this.count;e<n;e++)ze.fromBufferAttribute(this,e),ze.applyMatrix4(t),this.setXYZ(e,ze.x,ze.y,ze.z);return this}applyNormalMatrix(t){for(let e=0,n=this.count;e<n;e++)ze.fromBufferAttribute(this,e),ze.applyNormalMatrix(t),this.setXYZ(e,ze.x,ze.y,ze.z);return this}transformDirection(t){for(let e=0,n=this.count;e<n;e++)ze.fromBufferAttribute(this,e),ze.transformDirection(t),this.setXYZ(e,ze.x,ze.y,ze.z);return this}set(t,e=0){return this.array.set(t,e),this}getComponent(t,e){let n=this.array[t*this.itemSize+e];return this.normalized&&(n=gr(n,this.array)),n}setComponent(t,e,n){return this.normalized&&(n=mn(n,this.array)),this.array[t*this.itemSize+e]=n,this}getX(t){let e=this.array[t*this.itemSize];return this.normalized&&(e=gr(e,this.array)),e}setX(t,e){return this.normalized&&(e=mn(e,this.array)),this.array[t*this.itemSize]=e,this}getY(t){let e=this.array[t*this.itemSize+1];return this.normalized&&(e=gr(e,this.array)),e}setY(t,e){return this.normalized&&(e=mn(e,this.array)),this.array[t*this.itemSize+1]=e,this}getZ(t){let e=this.array[t*this.itemSize+2];return this.normalized&&(e=gr(e,this.array)),e}setZ(t,e){return this.normalized&&(e=mn(e,this.array)),this.array[t*this.itemSize+2]=e,this}getW(t){let e=this.array[t*this.itemSize+3];return this.normalized&&(e=gr(e,this.array)),e}setW(t,e){return this.normalized&&(e=mn(e,this.array)),this.array[t*this.itemSize+3]=e,this}setXY(t,e,n){return t*=this.itemSize,this.normalized&&(e=mn(e,this.array),n=mn(n,this.array)),this.array[t+0]=e,this.array[t+1]=n,this}setXYZ(t,e,n,i){return t*=this.itemSize,this.normalized&&(e=mn(e,this.array),n=mn(n,this.array),i=mn(i,this.array)),this.array[t+0]=e,this.array[t+1]=n,this.array[t+2]=i,this}setXYZW(t,e,n,i,s){return t*=this.itemSize,this.normalized&&(e=mn(e,this.array),n=mn(n,this.array),i=mn(i,this.array),s=mn(s,this.array)),this.array[t+0]=e,this.array[t+1]=n,this.array[t+2]=i,this.array[t+3]=s,this}onUpload(t){return this.onUploadCallback=t,this}clone(){return new this.constructor(this.array,this.itemSize).copy(this)}toJSON(){const t={itemSize:this.itemSize,type:this.array.constructor.name,array:Array.from(this.array),normalized:this.normalized};return this.name!==""&&(t.name=this.name),this.usage!==vd&&(t.usage=this.usage),t}}class um extends Ne{constructor(t,e,n){super(new Uint16Array(t),e,n)}}class dm extends Ne{constructor(t,e,n){super(new Uint32Array(t),e,n)}}class ie extends Ne{constructor(t,e,n){super(new Float32Array(t),e,n)}}let K0=0;const Bn=new se,vl=new Qe,Ks=new b,Cn=new en,Kr=new en,je=new b;class ye extends Or{constructor(){super(),this.isBufferGeometry=!0,Object.defineProperty(this,"id",{value:K0++}),this.uuid=Gr(),this.name="",this.type="BufferGeometry",this.index=null,this.attributes={},this.morphAttributes={},this.morphTargetsRelative=!1,this.groups=[],this.boundingBox=null,this.boundingSphere=null,this.drawRange={start:0,count:1/0},this.userData={}}getIndex(){return this.index}setIndex(t){return Array.isArray(t)?this.index=new(rm(t)?dm:um)(t,1):this.index=t,this}getAttribute(t){return this.attributes[t]}setAttribute(t,e){return this.attributes[t]=e,this}deleteAttribute(t){return delete this.attributes[t],this}hasAttribute(t){return this.attributes[t]!==void 0}addGroup(t,e,n=0){this.groups.push({start:t,count:e,materialIndex:n})}clearGroups(){this.groups=[]}setDrawRange(t,e){this.drawRange.start=t,this.drawRange.count=e}applyMatrix4(t){const e=this.attributes.position;e!==void 0&&(e.applyMatrix4(t),e.needsUpdate=!0);const n=this.attributes.normal;if(n!==void 0){const s=new ne().getNormalMatrix(t);n.applyNormalMatrix(s),n.needsUpdate=!0}const i=this.attributes.tangent;return i!==void 0&&(i.transformDirection(t),i.needsUpdate=!0),this.boundingBox!==null&&this.computeBoundingBox(),this.boundingSphere!==null&&this.computeBoundingSphere(),this}applyQuaternion(t){return Bn.makeRotationFromQuaternion(t),this.applyMatrix4(Bn),this}rotateX(t){return Bn.makeRotationX(t),this.applyMatrix4(Bn),this}rotateY(t){return Bn.makeRotationY(t),this.applyMatrix4(Bn),this}rotateZ(t){return Bn.makeRotationZ(t),this.applyMatrix4(Bn),this}translate(t,e,n){return Bn.makeTranslation(t,e,n),this.applyMatrix4(Bn),this}scale(t,e,n){return Bn.makeScale(t,e,n),this.applyMatrix4(Bn),this}lookAt(t){return vl.lookAt(t),vl.updateMatrix(),this.applyMatrix4(vl.matrix),this}center(){return this.computeBoundingBox(),this.boundingBox.getCenter(Ks).negate(),this.translate(Ks.x,Ks.y,Ks.z),this}setFromPoints(t){const e=[];for(let n=0,i=t.length;n<i;n++){const s=t[n];e.push(s.x,s.y,s.z||0)}return this.setAttribute("position",new ie(e,3)),this}computeBoundingBox(){this.boundingBox===null&&(this.boundingBox=new en);const t=this.attributes.position,e=this.morphAttributes.position;if(t&&t.isGLBufferAttribute){console.error('THREE.BufferGeometry.computeBoundingBox(): GLBufferAttribute requires a manual bounding box. Alternatively set "mesh.frustumCulled" to "false".',this),this.boundingBox.set(new b(-1/0,-1/0,-1/0),new b(1/0,1/0,1/0));return}if(t!==void 0){if(this.boundingBox.setFromBufferAttribute(t),e)for(let n=0,i=e.length;n<i;n++){const s=e[n];Cn.setFromBufferAttribute(s),this.morphTargetsRelative?(je.addVectors(this.boundingBox.min,Cn.min),this.boundingBox.expandByPoint(je),je.addVectors(this.boundingBox.max,Cn.max),this.boundingBox.expandByPoint(je)):(this.boundingBox.expandByPoint(Cn.min),this.boundingBox.expandByPoint(Cn.max))}}else this.boundingBox.makeEmpty();(isNaN(this.boundingBox.min.x)||isNaN(this.boundingBox.min.y)||isNaN(this.boundingBox.min.z))&&console.error('THREE.BufferGeometry.computeBoundingBox(): Computed min/max have NaN values. The "position" attribute is likely to have NaN values.',this)}computeBoundingSphere(){this.boundingSphere===null&&(this.boundingSphere=new ms);const t=this.attributes.position,e=this.morphAttributes.position;if(t&&t.isGLBufferAttribute){console.error('THREE.BufferGeometry.computeBoundingSphere(): GLBufferAttribute requires a manual bounding sphere. Alternatively set "mesh.frustumCulled" to "false".',this),this.boundingSphere.set(new b,1/0);return}if(t){const n=this.boundingSphere.center;if(Cn.setFromBufferAttribute(t),e)for(let s=0,a=e.length;s<a;s++){const o=e[s];Kr.setFromBufferAttribute(o),this.morphTargetsRelative?(je.addVectors(Cn.min,Kr.min),Cn.expandByPoint(je),je.addVectors(Cn.max,Kr.max),Cn.expandByPoint(je)):(Cn.expandByPoint(Kr.min),Cn.expandByPoint(Kr.max))}Cn.getCenter(n);let i=0;for(let s=0,a=t.count;s<a;s++)je.fromBufferAttribute(t,s),i=Math.max(i,n.distanceToSquared(je));if(e)for(let s=0,a=e.length;s<a;s++){const o=e[s],c=this.morphTargetsRelative;for(let l=0,h=o.count;l<h;l++)je.fromBufferAttribute(o,l),c&&(Ks.fromBufferAttribute(t,l),je.add(Ks)),i=Math.max(i,n.distanceToSquared(je))}this.boundingSphere.radius=Math.sqrt(i),isNaN(this.boundingSphere.radius)&&console.error('THREE.BufferGeometry.computeBoundingSphere(): Computed radius is NaN. The "position" attribute is likely to have NaN values.',this)}}computeTangents(){const t=this.index,e=this.attributes;if(t===null||e.position===void 0||e.normal===void 0||e.uv===void 0){console.error("THREE.BufferGeometry: .computeTangents() failed. Missing required attributes (index, position, normal or uv)");return}const n=t.array,i=e.position.array,s=e.normal.array,a=e.uv.array,o=i.length/3;this.hasAttribute("tangent")===!1&&this.setAttribute("tangent",new Ne(new Float32Array(4*o),4));const c=this.getAttribute("tangent").array,l=[],h=[];for(let A=0;A<o;A++)l[A]=new b,h[A]=new b;const u=new b,f=new b,d=new b,g=new St,x=new St,m=new St,p=new b,_=new b;function v(A,D,R){u.fromArray(i,A*3),f.fromArray(i,D*3),d.fromArray(i,R*3),g.fromArray(a,A*2),x.fromArray(a,D*2),m.fromArray(a,R*2),f.sub(u),d.sub(u),x.sub(g),m.sub(g);const N=1/(x.x*m.y-m.x*x.y);isFinite(N)&&(p.copy(f).multiplyScalar(m.y).addScaledVector(d,-x.y).multiplyScalar(N),_.copy(d).multiplyScalar(x.x).addScaledVector(f,-m.x).multiplyScalar(N),l[A].add(p),l[D].add(p),l[R].add(p),h[A].add(_),h[D].add(_),h[R].add(_))}let S=this.groups;S.length===0&&(S=[{start:0,count:n.length}]);for(let A=0,D=S.length;A<D;++A){const R=S[A],N=R.start,L=R.count;for(let F=N,z=N+L;F<z;F+=3)v(n[F+0],n[F+1],n[F+2])}const y=new b,M=new b,w=new b,T=new b;function E(A){w.fromArray(s,A*3),T.copy(w);const D=l[A];y.copy(D),y.sub(w.multiplyScalar(w.dot(D))).normalize(),M.crossVectors(T,D);const N=M.dot(h[A])<0?-1:1;c[A*4]=y.x,c[A*4+1]=y.y,c[A*4+2]=y.z,c[A*4+3]=N}for(let A=0,D=S.length;A<D;++A){const R=S[A],N=R.start,L=R.count;for(let F=N,z=N+L;F<z;F+=3)E(n[F+0]),E(n[F+1]),E(n[F+2])}}computeVertexNormals(){const t=this.index,e=this.getAttribute("position");if(e!==void 0){let n=this.getAttribute("normal");if(n===void 0)n=new Ne(new Float32Array(e.count*3),3),this.setAttribute("normal",n);else for(let f=0,d=n.count;f<d;f++)n.setXYZ(f,0,0,0);const i=new b,s=new b,a=new b,o=new b,c=new b,l=new b,h=new b,u=new b;if(t)for(let f=0,d=t.count;f<d;f+=3){const g=t.getX(f+0),x=t.getX(f+1),m=t.getX(f+2);i.fromBufferAttribute(e,g),s.fromBufferAttribute(e,x),a.fromBufferAttribute(e,m),h.subVectors(a,s),u.subVectors(i,s),h.cross(u),o.fromBufferAttribute(n,g),c.fromBufferAttribute(n,x),l.fromBufferAttribute(n,m),o.add(h),c.add(h),l.add(h),n.setXYZ(g,o.x,o.y,o.z),n.setXYZ(x,c.x,c.y,c.z),n.setXYZ(m,l.x,l.y,l.z)}else for(let f=0,d=e.count;f<d;f+=3)i.fromBufferAttribute(e,f+0),s.fromBufferAttribute(e,f+1),a.fromBufferAttribute(e,f+2),h.subVectors(a,s),u.subVectors(i,s),h.cross(u),n.setXYZ(f+0,h.x,h.y,h.z),n.setXYZ(f+1,h.x,h.y,h.z),n.setXYZ(f+2,h.x,h.y,h.z);this.normalizeNormals(),n.needsUpdate=!0}}normalizeNormals(){const t=this.attributes.normal;for(let e=0,n=t.count;e<n;e++)je.fromBufferAttribute(t,e),je.normalize(),t.setXYZ(e,je.x,je.y,je.z)}toNonIndexed(){function t(o,c){const l=o.array,h=o.itemSize,u=o.normalized,f=new l.constructor(c.length*h);let d=0,g=0;for(let x=0,m=c.length;x<m;x++){o.isInterleavedBufferAttribute?d=c[x]*o.data.stride+o.offset:d=c[x]*h;for(let p=0;p<h;p++)f[g++]=l[d++]}return new Ne(f,h,u)}if(this.index===null)return console.warn("THREE.BufferGeometry.toNonIndexed(): BufferGeometry is already non-indexed."),this;const e=new ye,n=this.index.array,i=this.attributes;for(const o in i){const c=i[o],l=t(c,n);e.setAttribute(o,l)}const s=this.morphAttributes;for(const o in s){const c=[],l=s[o];for(let h=0,u=l.length;h<u;h++){const f=l[h],d=t(f,n);c.push(d)}e.morphAttributes[o]=c}e.morphTargetsRelative=this.morphTargetsRelative;const a=this.groups;for(let o=0,c=a.length;o<c;o++){const l=a[o];e.addGroup(l.start,l.count,l.materialIndex)}return e}toJSON(){const t={metadata:{version:4.6,type:"BufferGeometry",generator:"BufferGeometry.toJSON"}};if(t.uuid=this.uuid,t.type=this.type,this.name!==""&&(t.name=this.name),Object.keys(this.userData).length>0&&(t.userData=this.userData),this.parameters!==void 0){const c=this.parameters;for(const l in c)c[l]!==void 0&&(t[l]=c[l]);return t}t.data={attributes:{}};const e=this.index;e!==null&&(t.data.index={type:e.array.constructor.name,array:Array.prototype.slice.call(e.array)});const n=this.attributes;for(const c in n){const l=n[c];t.data.attributes[c]=l.toJSON(t.data)}const i={};let s=!1;for(const c in this.morphAttributes){const l=this.morphAttributes[c],h=[];for(let u=0,f=l.length;u<f;u++){const d=l[u];h.push(d.toJSON(t.data))}h.length>0&&(i[c]=h,s=!0)}s&&(t.data.morphAttributes=i,t.data.morphTargetsRelative=this.morphTargetsRelative);const a=this.groups;a.length>0&&(t.data.groups=JSON.parse(JSON.stringify(a)));const o=this.boundingSphere;return o!==null&&(t.data.boundingSphere={center:o.center.toArray(),radius:o.radius}),t}clone(){return new this.constructor().copy(this)}copy(t){this.index=null,this.attributes={},this.morphAttributes={},this.groups=[],this.boundingBox=null,this.boundingSphere=null;const e={};this.name=t.name;const n=t.index;n!==null&&this.setIndex(n.clone(e));const i=t.attributes;for(const l in i){const h=i[l];this.setAttribute(l,h.clone(e))}const s=t.morphAttributes;for(const l in s){const h=[],u=s[l];for(let f=0,d=u.length;f<d;f++)h.push(u[f].clone(e));this.morphAttributes[l]=h}this.morphTargetsRelative=t.morphTargetsRelative;const a=t.groups;for(let l=0,h=a.length;l<h;l++){const u=a[l];this.addGroup(u.start,u.count,u.materialIndex)}const o=t.boundingBox;o!==null&&(this.boundingBox=o.clone());const c=t.boundingSphere;return c!==null&&(this.boundingSphere=c.clone()),this.drawRange.start=t.drawRange.start,this.drawRange.count=t.drawRange.count,this.userData=t.userData,this}dispose(){this.dispatchEvent({type:"dispose"})}}const Dd=new se,vs=new pu,ga=new ms,Nd=new b,Js=new b,Qs=new b,tr=new b,Sl=new b,xa=new b,_a=new St,va=new St,Sa=new St,Fd=new b,Ud=new b,zd=new b,Ma=new b,ya=new b;class Zt extends Qe{constructor(t=new ye,e=new Fe){super(),this.isMesh=!0,this.type="Mesh",this.geometry=t,this.material=e,this.updateMorphTargets()}copy(t,e){return super.copy(t,e),t.morphTargetInfluences!==void 0&&(this.morphTargetInfluences=t.morphTargetInfluences.slice()),t.morphTargetDictionary!==void 0&&(this.morphTargetDictionary=Object.assign({},t.morphTargetDictionary)),this.material=Array.isArray(t.material)?t.material.slice():t.material,this.geometry=t.geometry,this}updateMorphTargets(){const e=this.geometry.morphAttributes,n=Object.keys(e);if(n.length>0){const i=e[n[0]];if(i!==void 0){this.morphTargetInfluences=[],this.morphTargetDictionary={};for(let s=0,a=i.length;s<a;s++){const o=i[s].name||String(s);this.morphTargetInfluences.push(0),this.morphTargetDictionary[o]=s}}}}getVertexPosition(t,e){const n=this.geometry,i=n.attributes.position,s=n.morphAttributes.position,a=n.morphTargetsRelative;e.fromBufferAttribute(i,t);const o=this.morphTargetInfluences;if(s&&o){xa.set(0,0,0);for(let c=0,l=s.length;c<l;c++){const h=o[c],u=s[c];h!==0&&(Sl.fromBufferAttribute(u,t),a?xa.addScaledVector(Sl,h):xa.addScaledVector(Sl.sub(e),h))}e.add(xa)}return e}raycast(t,e){const n=this.geometry,i=this.material,s=this.matrixWorld;i!==void 0&&(n.boundingSphere===null&&n.computeBoundingSphere(),ga.copy(n.boundingSphere),ga.applyMatrix4(s),vs.copy(t.ray).recast(t.near),!(ga.containsPoint(vs.origin)===!1&&(vs.intersectSphere(ga,Nd)===null||vs.origin.distanceToSquared(Nd)>(t.far-t.near)**2))&&(Dd.copy(s).invert(),vs.copy(t.ray).applyMatrix4(Dd),!(n.boundingBox!==null&&vs.intersectsBox(n.boundingBox)===!1)&&this._computeIntersections(t,e,vs)))}_computeIntersections(t,e,n){let i;const s=this.geometry,a=this.material,o=s.index,c=s.attributes.position,l=s.attributes.uv,h=s.attributes.uv1,u=s.attributes.normal,f=s.groups,d=s.drawRange;if(o!==null)if(Array.isArray(a))for(let g=0,x=f.length;g<x;g++){const m=f[g],p=a[m.materialIndex],_=Math.max(m.start,d.start),v=Math.min(o.count,Math.min(m.start+m.count,d.start+d.count));for(let S=_,y=v;S<y;S+=3){const M=o.getX(S),w=o.getX(S+1),T=o.getX(S+2);i=Ea(this,p,t,n,l,h,u,M,w,T),i&&(i.faceIndex=Math.floor(S/3),i.face.materialIndex=m.materialIndex,e.push(i))}}else{const g=Math.max(0,d.start),x=Math.min(o.count,d.start+d.count);for(let m=g,p=x;m<p;m+=3){const _=o.getX(m),v=o.getX(m+1),S=o.getX(m+2);i=Ea(this,a,t,n,l,h,u,_,v,S),i&&(i.faceIndex=Math.floor(m/3),e.push(i))}}else if(c!==void 0)if(Array.isArray(a))for(let g=0,x=f.length;g<x;g++){const m=f[g],p=a[m.materialIndex],_=Math.max(m.start,d.start),v=Math.min(c.count,Math.min(m.start+m.count,d.start+d.count));for(let S=_,y=v;S<y;S+=3){const M=S,w=S+1,T=S+2;i=Ea(this,p,t,n,l,h,u,M,w,T),i&&(i.faceIndex=Math.floor(S/3),i.face.materialIndex=m.materialIndex,e.push(i))}}else{const g=Math.max(0,d.start),x=Math.min(c.count,d.start+d.count);for(let m=g,p=x;m<p;m+=3){const _=m,v=m+1,S=m+2;i=Ea(this,a,t,n,l,h,u,_,v,S),i&&(i.faceIndex=Math.floor(m/3),e.push(i))}}}}function J0(r,t,e,n,i,s,a,o){let c;if(t.side===dn?c=n.intersectTriangle(a,s,i,!0,o):c=n.intersectTriangle(i,s,a,t.side===oi,o),c===null)return null;ya.copy(o),ya.applyMatrix4(r.matrixWorld);const l=e.ray.origin.distanceTo(ya);return l<e.near||l>e.far?null:{distance:l,point:ya.clone(),object:r}}function Ea(r,t,e,n,i,s,a,o,c,l){r.getVertexPosition(o,Js),r.getVertexPosition(c,Qs),r.getVertexPosition(l,tr);const h=J0(r,t,e,n,Js,Qs,tr,Ma);if(h){i&&(_a.fromBufferAttribute(i,o),va.fromBufferAttribute(i,c),Sa.fromBufferAttribute(i,l),h.uv=rn.getInterpolation(Ma,Js,Qs,tr,_a,va,Sa,new St)),s&&(_a.fromBufferAttribute(s,o),va.fromBufferAttribute(s,c),Sa.fromBufferAttribute(s,l),h.uv1=rn.getInterpolation(Ma,Js,Qs,tr,_a,va,Sa,new St),h.uv2=h.uv1),a&&(Fd.fromBufferAttribute(a,o),Ud.fromBufferAttribute(a,c),zd.fromBufferAttribute(a,l),h.normal=rn.getInterpolation(Ma,Js,Qs,tr,Fd,Ud,zd,new b),h.normal.dot(n.direction)>0&&h.normal.multiplyScalar(-1));const u={a:o,b:c,c:l,normal:new b,materialIndex:0};rn.getNormal(Js,Qs,tr,u.normal),h.face=u}return h}class Pn extends ye{constructor(t=1,e=1,n=1,i=1,s=1,a=1){super(),this.type="BoxGeometry",this.parameters={width:t,height:e,depth:n,widthSegments:i,heightSegments:s,depthSegments:a};const o=this;i=Math.floor(i),s=Math.floor(s),a=Math.floor(a);const c=[],l=[],h=[],u=[];let f=0,d=0;g("z","y","x",-1,-1,n,e,t,a,s,0),g("z","y","x",1,-1,n,e,-t,a,s,1),g("x","z","y",1,1,t,n,e,i,a,2),g("x","z","y",1,-1,t,n,-e,i,a,3),g("x","y","z",1,-1,t,e,n,i,s,4),g("x","y","z",-1,-1,t,e,-n,i,s,5),this.setIndex(c),this.setAttribute("position",new ie(l,3)),this.setAttribute("normal",new ie(h,3)),this.setAttribute("uv",new ie(u,2));function g(x,m,p,_,v,S,y,M,w,T,E){const A=S/w,D=y/T,R=S/2,N=y/2,L=M/2,F=w+1,z=T+1;let q=0,O=0;const Y=new b;for(let K=0;K<z;K++){const J=K*D-N;for(let it=0;it<F;it++){const X=it*A-R;Y[x]=X*_,Y[m]=J*v,Y[p]=L,l.push(Y.x,Y.y,Y.z),Y[x]=0,Y[m]=0,Y[p]=M>0?1:-1,h.push(Y.x,Y.y,Y.z),u.push(it/w),u.push(1-K/T),q+=1}}for(let K=0;K<T;K++)for(let J=0;J<w;J++){const it=f+J+F*K,X=f+J+F*(K+1),j=f+(J+1)+F*(K+1),nt=f+(J+1)+F*K;c.push(it,X,nt),c.push(X,j,nt),O+=6}o.addGroup(d,O,E),d+=O,f+=q}}copy(t){return super.copy(t),this.parameters=Object.assign({},t.parameters),this}static fromJSON(t){return new Pn(t.width,t.height,t.depth,t.widthSegments,t.heightSegments,t.depthSegments)}}function Ur(r){const t={};for(const e in r){t[e]={};for(const n in r[e]){const i=r[e][n];i&&(i.isColor||i.isMatrix3||i.isMatrix4||i.isVector2||i.isVector3||i.isVector4||i.isTexture||i.isQuaternion)?i.isRenderTargetTexture?(console.warn("UniformsUtils: Textures of render targets cannot be cloned via cloneUniforms() or mergeUniforms()."),t[e][n]=null):t[e][n]=i.clone():Array.isArray(i)?t[e][n]=i.slice():t[e][n]=i}}return t}function gn(r){const t={};for(let e=0;e<r.length;e++){const n=Ur(r[e]);for(const i in n)t[i]=n[i]}return t}function Q0(r){const t=[];for(let e=0;e<r.length;e++)t.push(r[e].clone());return t}function fm(r){return r.getRenderTarget()===null?r.outputColorSpace:me.workingColorSpace}const tx={clone:Ur,merge:gn};var ex=`void main() {
	gl_Position = projectionMatrix * modelViewMatrix * vec4( position, 1.0 );
}`,nx=`void main() {
	gl_FragColor = vec4( 1.0, 0.0, 0.0, 1.0 );
}`;class Fn extends Si{constructor(t){super(),this.isShaderMaterial=!0,this.type="ShaderMaterial",this.defines={},this.uniforms={},this.uniformsGroups=[],this.vertexShader=ex,this.fragmentShader=nx,this.linewidth=1,this.wireframe=!1,this.wireframeLinewidth=1,this.fog=!1,this.lights=!1,this.clipping=!1,this.forceSinglePass=!0,this.extensions={derivatives:!1,fragDepth:!1,drawBuffers:!1,shaderTextureLOD:!1,clipCullDistance:!1},this.defaultAttributeValues={color:[1,1,1],uv:[0,0],uv1:[0,0]},this.index0AttributeName=void 0,this.uniformsNeedUpdate=!1,this.glslVersion=null,t!==void 0&&this.setValues(t)}copy(t){return super.copy(t),this.fragmentShader=t.fragmentShader,this.vertexShader=t.vertexShader,this.uniforms=Ur(t.uniforms),this.uniformsGroups=Q0(t.uniformsGroups),this.defines=Object.assign({},t.defines),this.wireframe=t.wireframe,this.wireframeLinewidth=t.wireframeLinewidth,this.fog=t.fog,this.lights=t.lights,this.clipping=t.clipping,this.extensions=Object.assign({},t.extensions),this.glslVersion=t.glslVersion,this}toJSON(t){const e=super.toJSON(t);e.glslVersion=this.glslVersion,e.uniforms={};for(const i in this.uniforms){const a=this.uniforms[i].value;a&&a.isTexture?e.uniforms[i]={type:"t",value:a.toJSON(t).uuid}:a&&a.isColor?e.uniforms[i]={type:"c",value:a.getHex()}:a&&a.isVector2?e.uniforms[i]={type:"v2",value:a.toArray()}:a&&a.isVector3?e.uniforms[i]={type:"v3",value:a.toArray()}:a&&a.isVector4?e.uniforms[i]={type:"v4",value:a.toArray()}:a&&a.isMatrix3?e.uniforms[i]={type:"m3",value:a.toArray()}:a&&a.isMatrix4?e.uniforms[i]={type:"m4",value:a.toArray()}:e.uniforms[i]={value:a}}Object.keys(this.defines).length>0&&(e.defines=this.defines),e.vertexShader=this.vertexShader,e.fragmentShader=this.fragmentShader,e.lights=this.lights,e.clipping=this.clipping;const n={};for(const i in this.extensions)this.extensions[i]===!0&&(n[i]=!0);return Object.keys(n).length>0&&(e.extensions=n),e}}class pm extends Qe{constructor(){super(),this.isCamera=!0,this.type="Camera",this.matrixWorldInverse=new se,this.projectionMatrix=new se,this.projectionMatrixInverse=new se,this.coordinateSystem=Oi}copy(t,e){return super.copy(t,e),this.matrixWorldInverse.copy(t.matrixWorldInverse),this.projectionMatrix.copy(t.projectionMatrix),this.projectionMatrixInverse.copy(t.projectionMatrixInverse),this.coordinateSystem=t.coordinateSystem,this}getWorldDirection(t){return super.getWorldDirection(t).negate()}updateMatrixWorld(t){super.updateMatrixWorld(t),this.matrixWorldInverse.copy(this.matrixWorld).invert()}updateWorldMatrix(t,e){super.updateWorldMatrix(t,e),this.matrixWorldInverse.copy(this.matrixWorld).invert()}clone(){return new this.constructor().copy(this)}}class Ln extends pm{constructor(t=50,e=1,n=.1,i=2e3){super(),this.isPerspectiveCamera=!0,this.type="PerspectiveCamera",this.fov=t,this.zoom=1,this.near=n,this.far=i,this.focus=10,this.aspect=e,this.view=null,this.filmGauge=35,this.filmOffset=0,this.updateProjectionMatrix()}copy(t,e){return super.copy(t,e),this.fov=t.fov,this.zoom=t.zoom,this.near=t.near,this.far=t.far,this.focus=t.focus,this.aspect=t.aspect,this.view=t.view===null?null:Object.assign({},t.view),this.filmGauge=t.filmGauge,this.filmOffset=t.filmOffset,this}setFocalLength(t){const e=.5*this.getFilmHeight()/t;this.fov=Ko*2*Math.atan(e),this.updateProjectionMatrix()}getFocalLength(){const t=Math.tan(Oo*.5*this.fov);return .5*this.getFilmHeight()/t}getEffectiveFOV(){return Ko*2*Math.atan(Math.tan(Oo*.5*this.fov)/this.zoom)}getFilmWidth(){return this.filmGauge*Math.min(this.aspect,1)}getFilmHeight(){return this.filmGauge/Math.max(this.aspect,1)}setViewOffset(t,e,n,i,s,a){this.aspect=t/e,this.view===null&&(this.view={enabled:!0,fullWidth:1,fullHeight:1,offsetX:0,offsetY:0,width:1,height:1}),this.view.enabled=!0,this.view.fullWidth=t,this.view.fullHeight=e,this.view.offsetX=n,this.view.offsetY=i,this.view.width=s,this.view.height=a,this.updateProjectionMatrix()}clearViewOffset(){this.view!==null&&(this.view.enabled=!1),this.updateProjectionMatrix()}updateProjectionMatrix(){const t=this.near;let e=t*Math.tan(Oo*.5*this.fov)/this.zoom,n=2*e,i=this.aspect*n,s=-.5*i;const a=this.view;if(this.view!==null&&this.view.enabled){const c=a.fullWidth,l=a.fullHeight;s+=a.offsetX*i/c,e-=a.offsetY*n/l,i*=a.width/c,n*=a.height/l}const o=this.filmOffset;o!==0&&(s+=t*o/this.getFilmWidth()),this.projectionMatrix.makePerspective(s,s+i,e,e-n,t,this.far,this.coordinateSystem),this.projectionMatrixInverse.copy(this.projectionMatrix).invert()}toJSON(t){const e=super.toJSON(t);return e.object.fov=this.fov,e.object.zoom=this.zoom,e.object.near=this.near,e.object.far=this.far,e.object.focus=this.focus,e.object.aspect=this.aspect,this.view!==null&&(e.object.view=Object.assign({},this.view)),e.object.filmGauge=this.filmGauge,e.object.filmOffset=this.filmOffset,e}}const er=-90,nr=1;class ix extends Qe{constructor(t,e,n){super(),this.type="CubeCamera",this.renderTarget=n,this.coordinateSystem=null,this.activeMipmapLevel=0;const i=new Ln(er,nr,t,e);i.layers=this.layers,this.add(i);const s=new Ln(er,nr,t,e);s.layers=this.layers,this.add(s);const a=new Ln(er,nr,t,e);a.layers=this.layers,this.add(a);const o=new Ln(er,nr,t,e);o.layers=this.layers,this.add(o);const c=new Ln(er,nr,t,e);c.layers=this.layers,this.add(c);const l=new Ln(er,nr,t,e);l.layers=this.layers,this.add(l)}updateCoordinateSystem(){const t=this.coordinateSystem,e=this.children.concat(),[n,i,s,a,o,c]=e;for(const l of e)this.remove(l);if(t===Oi)n.up.set(0,1,0),n.lookAt(1,0,0),i.up.set(0,1,0),i.lookAt(-1,0,0),s.up.set(0,0,-1),s.lookAt(0,1,0),a.up.set(0,0,1),a.lookAt(0,-1,0),o.up.set(0,1,0),o.lookAt(0,0,1),c.up.set(0,1,0),c.lookAt(0,0,-1);else if(t===Mc)n.up.set(0,-1,0),n.lookAt(-1,0,0),i.up.set(0,-1,0),i.lookAt(1,0,0),s.up.set(0,0,1),s.lookAt(0,1,0),a.up.set(0,0,-1),a.lookAt(0,-1,0),o.up.set(0,-1,0),o.lookAt(0,0,1),c.up.set(0,-1,0),c.lookAt(0,0,-1);else throw new Error("THREE.CubeCamera.updateCoordinateSystem(): Invalid coordinate system: "+t);for(const l of e)this.add(l),l.updateMatrixWorld()}update(t,e){this.parent===null&&this.updateMatrixWorld();const{renderTarget:n,activeMipmapLevel:i}=this;this.coordinateSystem!==t.coordinateSystem&&(this.coordinateSystem=t.coordinateSystem,this.updateCoordinateSystem());const[s,a,o,c,l,h]=this.children,u=t.getRenderTarget(),f=t.getActiveCubeFace(),d=t.getActiveMipmapLevel(),g=t.xr.enabled;t.xr.enabled=!1;const x=n.texture.generateMipmaps;n.texture.generateMipmaps=!1,t.setRenderTarget(n,0,i),t.render(e,s),t.setRenderTarget(n,1,i),t.render(e,a),t.setRenderTarget(n,2,i),t.render(e,o),t.setRenderTarget(n,3,i),t.render(e,c),t.setRenderTarget(n,4,i),t.render(e,l),n.texture.generateMipmaps=x,t.setRenderTarget(n,5,i),t.render(e,h),t.setRenderTarget(u,f,d),t.xr.enabled=g,n.texture.needsPMREMUpdate=!0}}class mm extends Dn{constructor(t,e,n,i,s,a,o,c,l,h){t=t!==void 0?t:[],e=e!==void 0?e:Ir,super(t,e,n,i,s,a,o,c,l,h),this.isCubeTexture=!0,this.flipY=!1}get images(){return this.image}set images(t){this.image=t}}class sx extends tn{constructor(t=1,e={}){super(t,t,e),this.isWebGLCubeRenderTarget=!0;const n={width:t,height:t,depth:1},i=[n,n,n,n,n,n];e.encoding!==void 0&&(Vo("THREE.WebGLCubeRenderTarget: option.encoding has been replaced by option.colorSpace."),e.colorSpace=e.encoding===Ns?sn:kn),this.texture=new mm(i,e.mapping,e.wrapS,e.wrapT,e.magFilter,e.minFilter,e.format,e.type,e.anisotropy,e.colorSpace),this.texture.isRenderTargetTexture=!0,this.texture.generateMipmaps=e.generateMipmaps!==void 0?e.generateMipmaps:!1,this.texture.minFilter=e.minFilter!==void 0?e.minFilter:Vn}fromEquirectangularTexture(t,e){this.texture.type=e.type,this.texture.colorSpace=e.colorSpace,this.texture.generateMipmaps=e.generateMipmaps,this.texture.minFilter=e.minFilter,this.texture.magFilter=e.magFilter;const n={uniforms:{tEquirect:{value:null}},vertexShader:`

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
			`},i=new Pn(5,5,5),s=new Fn({name:"CubemapFromEquirect",uniforms:Ur(n.uniforms),vertexShader:n.vertexShader,fragmentShader:n.fragmentShader,side:dn,blending:os});s.uniforms.tEquirect.value=e;const a=new Zt(i,s),o=e.minFilter;return e.minFilter===$o&&(e.minFilter=Vn),new ix(1,10,this).update(t,a),e.minFilter=o,a.geometry.dispose(),a.material.dispose(),this}clear(t,e,n,i){const s=t.getRenderTarget();for(let a=0;a<6;a++)t.setRenderTarget(this,a),t.clear(e,n,i);t.setRenderTarget(s)}}const Ml=new b,rx=new b,ox=new ne;class zi{constructor(t=new b(1,0,0),e=0){this.isPlane=!0,this.normal=t,this.constant=e}set(t,e){return this.normal.copy(t),this.constant=e,this}setComponents(t,e,n,i){return this.normal.set(t,e,n),this.constant=i,this}setFromNormalAndCoplanarPoint(t,e){return this.normal.copy(t),this.constant=-e.dot(this.normal),this}setFromCoplanarPoints(t,e,n){const i=Ml.subVectors(n,e).cross(rx.subVectors(t,e)).normalize();return this.setFromNormalAndCoplanarPoint(i,t),this}copy(t){return this.normal.copy(t.normal),this.constant=t.constant,this}normalize(){const t=1/this.normal.length();return this.normal.multiplyScalar(t),this.constant*=t,this}negate(){return this.constant*=-1,this.normal.negate(),this}distanceToPoint(t){return this.normal.dot(t)+this.constant}distanceToSphere(t){return this.distanceToPoint(t.center)-t.radius}projectPoint(t,e){return e.copy(t).addScaledVector(this.normal,-this.distanceToPoint(t))}intersectLine(t,e){const n=t.delta(Ml),i=this.normal.dot(n);if(i===0)return this.distanceToPoint(t.start)===0?e.copy(t.start):null;const s=-(t.start.dot(this.normal)+this.constant)/i;return s<0||s>1?null:e.copy(t.start).addScaledVector(n,s)}intersectsLine(t){const e=this.distanceToPoint(t.start),n=this.distanceToPoint(t.end);return e<0&&n>0||n<0&&e>0}intersectsBox(t){return t.intersectsPlane(this)}intersectsSphere(t){return t.intersectsPlane(this)}coplanarPoint(t){return t.copy(this.normal).multiplyScalar(-this.constant)}applyMatrix4(t,e){const n=e||ox.getNormalMatrix(t),i=this.coplanarPoint(Ml).applyMatrix4(t),s=this.normal.applyMatrix3(n).normalize();return this.constant=-i.dot(s),this}translate(t){return this.constant-=t.dot(this.normal),this}equals(t){return t.normal.equals(this.normal)&&t.constant===this.constant}clone(){return new this.constructor().copy(this)}}const Ss=new ms,wa=new b;class mu{constructor(t=new zi,e=new zi,n=new zi,i=new zi,s=new zi,a=new zi){this.planes=[t,e,n,i,s,a]}set(t,e,n,i,s,a){const o=this.planes;return o[0].copy(t),o[1].copy(e),o[2].copy(n),o[3].copy(i),o[4].copy(s),o[5].copy(a),this}copy(t){const e=this.planes;for(let n=0;n<6;n++)e[n].copy(t.planes[n]);return this}setFromProjectionMatrix(t,e=Oi){const n=this.planes,i=t.elements,s=i[0],a=i[1],o=i[2],c=i[3],l=i[4],h=i[5],u=i[6],f=i[7],d=i[8],g=i[9],x=i[10],m=i[11],p=i[12],_=i[13],v=i[14],S=i[15];if(n[0].setComponents(c-s,f-l,m-d,S-p).normalize(),n[1].setComponents(c+s,f+l,m+d,S+p).normalize(),n[2].setComponents(c+a,f+h,m+g,S+_).normalize(),n[3].setComponents(c-a,f-h,m-g,S-_).normalize(),n[4].setComponents(c-o,f-u,m-x,S-v).normalize(),e===Oi)n[5].setComponents(c+o,f+u,m+x,S+v).normalize();else if(e===Mc)n[5].setComponents(o,u,x,v).normalize();else throw new Error("THREE.Frustum.setFromProjectionMatrix(): Invalid coordinate system: "+e);return this}intersectsObject(t){if(t.boundingSphere!==void 0)t.boundingSphere===null&&t.computeBoundingSphere(),Ss.copy(t.boundingSphere).applyMatrix4(t.matrixWorld);else{const e=t.geometry;e.boundingSphere===null&&e.computeBoundingSphere(),Ss.copy(e.boundingSphere).applyMatrix4(t.matrixWorld)}return this.intersectsSphere(Ss)}intersectsSprite(t){return Ss.center.set(0,0,0),Ss.radius=.7071067811865476,Ss.applyMatrix4(t.matrixWorld),this.intersectsSphere(Ss)}intersectsSphere(t){const e=this.planes,n=t.center,i=-t.radius;for(let s=0;s<6;s++)if(e[s].distanceToPoint(n)<i)return!1;return!0}intersectsBox(t){const e=this.planes;for(let n=0;n<6;n++){const i=e[n];if(wa.x=i.normal.x>0?t.max.x:t.min.x,wa.y=i.normal.y>0?t.max.y:t.min.y,wa.z=i.normal.z>0?t.max.z:t.min.z,i.distanceToPoint(wa)<0)return!1}return!0}containsPoint(t){const e=this.planes;for(let n=0;n<6;n++)if(e[n].distanceToPoint(t)<0)return!1;return!0}clone(){return new this.constructor().copy(this)}}function gm(){let r=null,t=!1,e=null,n=null;function i(s,a){e(s,a),n=r.requestAnimationFrame(i)}return{start:function(){t!==!0&&e!==null&&(n=r.requestAnimationFrame(i),t=!0)},stop:function(){r.cancelAnimationFrame(n),t=!1},setAnimationLoop:function(s){e=s},setContext:function(s){r=s}}}function ax(r,t){const e=t.isWebGL2,n=new WeakMap;function i(l,h){const u=l.array,f=l.usage,d=u.byteLength,g=r.createBuffer();r.bindBuffer(h,g),r.bufferData(h,u,f),l.onUploadCallback();let x;if(u instanceof Float32Array)x=r.FLOAT;else if(u instanceof Uint16Array)if(l.isFloat16BufferAttribute)if(e)x=r.HALF_FLOAT;else throw new Error("THREE.WebGLAttributes: Usage of Float16BufferAttribute requires WebGL2.");else x=r.UNSIGNED_SHORT;else if(u instanceof Int16Array)x=r.SHORT;else if(u instanceof Uint32Array)x=r.UNSIGNED_INT;else if(u instanceof Int32Array)x=r.INT;else if(u instanceof Int8Array)x=r.BYTE;else if(u instanceof Uint8Array)x=r.UNSIGNED_BYTE;else if(u instanceof Uint8ClampedArray)x=r.UNSIGNED_BYTE;else throw new Error("THREE.WebGLAttributes: Unsupported buffer data format: "+u);return{buffer:g,type:x,bytesPerElement:u.BYTES_PER_ELEMENT,version:l.version,size:d}}function s(l,h,u){const f=h.array,d=h._updateRange,g=h.updateRanges;if(r.bindBuffer(u,l),d.count===-1&&g.length===0&&r.bufferSubData(u,0,f),g.length!==0){for(let x=0,m=g.length;x<m;x++){const p=g[x];e?r.bufferSubData(u,p.start*f.BYTES_PER_ELEMENT,f,p.start,p.count):r.bufferSubData(u,p.start*f.BYTES_PER_ELEMENT,f.subarray(p.start,p.start+p.count))}h.clearUpdateRanges()}d.count!==-1&&(e?r.bufferSubData(u,d.offset*f.BYTES_PER_ELEMENT,f,d.offset,d.count):r.bufferSubData(u,d.offset*f.BYTES_PER_ELEMENT,f.subarray(d.offset,d.offset+d.count)),d.count=-1),h.onUploadCallback()}function a(l){return l.isInterleavedBufferAttribute&&(l=l.data),n.get(l)}function o(l){l.isInterleavedBufferAttribute&&(l=l.data);const h=n.get(l);h&&(r.deleteBuffer(h.buffer),n.delete(l))}function c(l,h){if(l.isGLBufferAttribute){const f=n.get(l);(!f||f.version<l.version)&&n.set(l,{buffer:l.buffer,type:l.type,bytesPerElement:l.elementSize,version:l.version});return}l.isInterleavedBufferAttribute&&(l=l.data);const u=n.get(l);if(u===void 0)n.set(l,i(l,h));else if(u.version<l.version){if(u.size!==l.array.byteLength)throw new Error("THREE.WebGLAttributes: The size of the buffer attribute's array buffer does not match the original size. Resizing buffer attributes is not supported.");s(u.buffer,l,h),u.version=l.version}}return{get:a,remove:o,update:c}}class kc extends ye{constructor(t=1,e=1,n=1,i=1){super(),this.type="PlaneGeometry",this.parameters={width:t,height:e,widthSegments:n,heightSegments:i};const s=t/2,a=e/2,o=Math.floor(n),c=Math.floor(i),l=o+1,h=c+1,u=t/o,f=e/c,d=[],g=[],x=[],m=[];for(let p=0;p<h;p++){const _=p*f-a;for(let v=0;v<l;v++){const S=v*u-s;g.push(S,-_,0),x.push(0,0,1),m.push(v/o),m.push(1-p/c)}}for(let p=0;p<c;p++)for(let _=0;_<o;_++){const v=_+l*p,S=_+l*(p+1),y=_+1+l*(p+1),M=_+1+l*p;d.push(v,S,M),d.push(S,y,M)}this.setIndex(d),this.setAttribute("position",new ie(g,3)),this.setAttribute("normal",new ie(x,3)),this.setAttribute("uv",new ie(m,2))}copy(t){return super.copy(t),this.parameters=Object.assign({},t.parameters),this}static fromJSON(t){return new kc(t.width,t.height,t.widthSegments,t.heightSegments)}}var cx=`#ifdef USE_ALPHAHASH
	if ( diffuseColor.a < getAlphaHashThreshold( vPosition ) ) discard;
#endif`,lx=`#ifdef USE_ALPHAHASH
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
#endif`,hx=`#ifdef USE_ALPHAMAP
	diffuseColor.a *= texture2D( alphaMap, vAlphaMapUv ).g;
#endif`,ux=`#ifdef USE_ALPHAMAP
	uniform sampler2D alphaMap;
#endif`,dx=`#ifdef USE_ALPHATEST
	if ( diffuseColor.a < alphaTest ) discard;
#endif`,fx=`#ifdef USE_ALPHATEST
	uniform float alphaTest;
#endif`,px=`#ifdef USE_AOMAP
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
#endif`,mx=`#ifdef USE_AOMAP
	uniform sampler2D aoMap;
	uniform float aoMapIntensity;
#endif`,gx=`#ifdef USE_BATCHING
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
#endif`,xx=`#ifdef USE_BATCHING
	mat4 batchingMatrix = getBatchingMatrix( batchId );
#endif`,_x=`vec3 transformed = vec3( position );
#ifdef USE_ALPHAHASH
	vPosition = vec3( position );
#endif`,vx=`vec3 objectNormal = vec3( normal );
#ifdef USE_TANGENT
	vec3 objectTangent = vec3( tangent.xyz );
#endif`,Sx=`float G_BlinnPhong_Implicit( ) {
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
} // validated`,Mx=`#ifdef USE_IRIDESCENCE
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
#endif`,yx=`#ifdef USE_BUMPMAP
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
#endif`,Ex=`#if NUM_CLIPPING_PLANES > 0
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
#endif`,wx=`#if NUM_CLIPPING_PLANES > 0
	varying vec3 vClipPosition;
	uniform vec4 clippingPlanes[ NUM_CLIPPING_PLANES ];
#endif`,Ax=`#if NUM_CLIPPING_PLANES > 0
	varying vec3 vClipPosition;
#endif`,Tx=`#if NUM_CLIPPING_PLANES > 0
	vClipPosition = - mvPosition.xyz;
#endif`,bx=`#if defined( USE_COLOR_ALPHA )
	diffuseColor *= vColor;
#elif defined( USE_COLOR )
	diffuseColor.rgb *= vColor;
#endif`,Cx=`#if defined( USE_COLOR_ALPHA )
	varying vec4 vColor;
#elif defined( USE_COLOR )
	varying vec3 vColor;
#endif`,Rx=`#if defined( USE_COLOR_ALPHA )
	varying vec4 vColor;
#elif defined( USE_COLOR ) || defined( USE_INSTANCING_COLOR )
	varying vec3 vColor;
#endif`,Px=`#if defined( USE_COLOR_ALPHA )
	vColor = vec4( 1.0 );
#elif defined( USE_COLOR ) || defined( USE_INSTANCING_COLOR )
	vColor = vec3( 1.0 );
#endif
#ifdef USE_COLOR
	vColor *= color;
#endif
#ifdef USE_INSTANCING_COLOR
	vColor.xyz *= instanceColor.xyz;
#endif`,Lx=`#define PI 3.141592653589793
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
} // validated`,Ix=`#ifdef ENVMAP_TYPE_CUBE_UV
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
#endif`,Dx=`vec3 transformedNormal = objectNormal;
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
#endif`,Nx=`#ifdef USE_DISPLACEMENTMAP
	uniform sampler2D displacementMap;
	uniform float displacementScale;
	uniform float displacementBias;
#endif`,Fx=`#ifdef USE_DISPLACEMENTMAP
	transformed += normalize( objectNormal ) * ( texture2D( displacementMap, vDisplacementMapUv ).x * displacementScale + displacementBias );
#endif`,Ux=`#ifdef USE_EMISSIVEMAP
	vec4 emissiveColor = texture2D( emissiveMap, vEmissiveMapUv );
	totalEmissiveRadiance *= emissiveColor.rgb;
#endif`,zx=`#ifdef USE_EMISSIVEMAP
	uniform sampler2D emissiveMap;
#endif`,Bx="gl_FragColor = linearToOutputTexel( gl_FragColor );",Ox=`
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
}`,Gx=`#ifdef USE_ENVMAP
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
#endif`,Vx=`#ifdef USE_ENVMAP
	uniform float envMapIntensity;
	uniform float flipEnvMap;
	#ifdef ENVMAP_TYPE_CUBE
		uniform samplerCube envMap;
	#else
		uniform sampler2D envMap;
	#endif
	
#endif`,kx=`#ifdef USE_ENVMAP
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
#endif`,Hx=`#ifdef USE_ENVMAP
	#if defined( USE_BUMPMAP ) || defined( USE_NORMALMAP ) || defined( PHONG ) || defined( LAMBERT )
		#define ENV_WORLDPOS
	#endif
	#ifdef ENV_WORLDPOS
		
		varying vec3 vWorldPosition;
	#else
		varying vec3 vReflect;
		uniform float refractionRatio;
	#endif
#endif`,Wx=`#ifdef USE_ENVMAP
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
#endif`,Xx=`#ifdef USE_FOG
	vFogDepth = - mvPosition.z;
#endif`,Yx=`#ifdef USE_FOG
	varying float vFogDepth;
#endif`,qx=`#ifdef USE_FOG
	#ifdef FOG_EXP2
		float fogFactor = 1.0 - exp( - fogDensity * fogDensity * vFogDepth * vFogDepth );
	#else
		float fogFactor = smoothstep( fogNear, fogFar, vFogDepth );
	#endif
	gl_FragColor.rgb = mix( gl_FragColor.rgb, fogColor, fogFactor );
#endif`,Zx=`#ifdef USE_FOG
	uniform vec3 fogColor;
	varying float vFogDepth;
	#ifdef FOG_EXP2
		uniform float fogDensity;
	#else
		uniform float fogNear;
		uniform float fogFar;
	#endif
#endif`,jx=`#ifdef USE_GRADIENTMAP
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
}`,$x=`#ifdef USE_LIGHTMAP
	vec4 lightMapTexel = texture2D( lightMap, vLightMapUv );
	vec3 lightMapIrradiance = lightMapTexel.rgb * lightMapIntensity;
	reflectedLight.indirectDiffuse += lightMapIrradiance;
#endif`,Kx=`#ifdef USE_LIGHTMAP
	uniform sampler2D lightMap;
	uniform float lightMapIntensity;
#endif`,Jx=`LambertMaterial material;
material.diffuseColor = diffuseColor.rgb;
material.specularStrength = specularStrength;`,Qx=`varying vec3 vViewPosition;
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
#define RE_IndirectDiffuse		RE_IndirectDiffuse_Lambert`,t_=`uniform bool receiveShadow;
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
#endif`,e_=`#ifdef USE_ENVMAP
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
#endif`,n_=`ToonMaterial material;
material.diffuseColor = diffuseColor.rgb;`,i_=`varying vec3 vViewPosition;
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
#define RE_IndirectDiffuse		RE_IndirectDiffuse_Toon`,s_=`BlinnPhongMaterial material;
material.diffuseColor = diffuseColor.rgb;
material.specularColor = specular;
material.specularShininess = shininess;
material.specularStrength = specularStrength;`,r_=`varying vec3 vViewPosition;
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
#define RE_IndirectDiffuse		RE_IndirectDiffuse_BlinnPhong`,o_=`PhysicalMaterial material;
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
#endif`,a_=`struct PhysicalMaterial {
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
}`,c_=`
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
#endif`,l_=`#if defined( RE_IndirectDiffuse )
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
#endif`,h_=`#if defined( RE_IndirectDiffuse )
	RE_IndirectDiffuse( irradiance, geometryPosition, geometryNormal, geometryViewDir, geometryClearcoatNormal, material, reflectedLight );
#endif
#if defined( RE_IndirectSpecular )
	RE_IndirectSpecular( radiance, iblIrradiance, clearcoatRadiance, geometryPosition, geometryNormal, geometryViewDir, geometryClearcoatNormal, material, reflectedLight );
#endif`,u_=`#if defined( USE_LOGDEPTHBUF ) && defined( USE_LOGDEPTHBUF_EXT )
	gl_FragDepthEXT = vIsPerspective == 0.0 ? gl_FragCoord.z : log2( vFragDepth ) * logDepthBufFC * 0.5;
#endif`,d_=`#if defined( USE_LOGDEPTHBUF ) && defined( USE_LOGDEPTHBUF_EXT )
	uniform float logDepthBufFC;
	varying float vFragDepth;
	varying float vIsPerspective;
#endif`,f_=`#ifdef USE_LOGDEPTHBUF
	#ifdef USE_LOGDEPTHBUF_EXT
		varying float vFragDepth;
		varying float vIsPerspective;
	#else
		uniform float logDepthBufFC;
	#endif
#endif`,p_=`#ifdef USE_LOGDEPTHBUF
	#ifdef USE_LOGDEPTHBUF_EXT
		vFragDepth = 1.0 + gl_Position.w;
		vIsPerspective = float( isPerspectiveMatrix( projectionMatrix ) );
	#else
		if ( isPerspectiveMatrix( projectionMatrix ) ) {
			gl_Position.z = log2( max( EPSILON, gl_Position.w + 1.0 ) ) * logDepthBufFC - 1.0;
			gl_Position.z *= gl_Position.w;
		}
	#endif
#endif`,m_=`#ifdef USE_MAP
	vec4 sampledDiffuseColor = texture2D( map, vMapUv );
	#ifdef DECODE_VIDEO_TEXTURE
		sampledDiffuseColor = vec4( mix( pow( sampledDiffuseColor.rgb * 0.9478672986 + vec3( 0.0521327014 ), vec3( 2.4 ) ), sampledDiffuseColor.rgb * 0.0773993808, vec3( lessThanEqual( sampledDiffuseColor.rgb, vec3( 0.04045 ) ) ) ), sampledDiffuseColor.w );
	
	#endif
	diffuseColor *= sampledDiffuseColor;
#endif`,g_=`#ifdef USE_MAP
	uniform sampler2D map;
#endif`,x_=`#if defined( USE_MAP ) || defined( USE_ALPHAMAP )
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
#endif`,__=`#if defined( USE_POINTS_UV )
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
#endif`,v_=`float metalnessFactor = metalness;
#ifdef USE_METALNESSMAP
	vec4 texelMetalness = texture2D( metalnessMap, vMetalnessMapUv );
	metalnessFactor *= texelMetalness.b;
#endif`,S_=`#ifdef USE_METALNESSMAP
	uniform sampler2D metalnessMap;
#endif`,M_=`#if defined( USE_MORPHCOLORS ) && defined( MORPHTARGETS_TEXTURE )
	vColor *= morphTargetBaseInfluence;
	for ( int i = 0; i < MORPHTARGETS_COUNT; i ++ ) {
		#if defined( USE_COLOR_ALPHA )
			if ( morphTargetInfluences[ i ] != 0.0 ) vColor += getMorph( gl_VertexID, i, 2 ) * morphTargetInfluences[ i ];
		#elif defined( USE_COLOR )
			if ( morphTargetInfluences[ i ] != 0.0 ) vColor += getMorph( gl_VertexID, i, 2 ).rgb * morphTargetInfluences[ i ];
		#endif
	}
#endif`,y_=`#ifdef USE_MORPHNORMALS
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
#endif`,E_=`#ifdef USE_MORPHTARGETS
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
#endif`,w_=`#ifdef USE_MORPHTARGETS
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
#endif`,A_=`float faceDirection = gl_FrontFacing ? 1.0 : - 1.0;
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
vec3 nonPerturbedNormal = normal;`,T_=`#ifdef USE_NORMALMAP_OBJECTSPACE
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
#endif`,b_=`#ifndef FLAT_SHADED
	varying vec3 vNormal;
	#ifdef USE_TANGENT
		varying vec3 vTangent;
		varying vec3 vBitangent;
	#endif
#endif`,C_=`#ifndef FLAT_SHADED
	varying vec3 vNormal;
	#ifdef USE_TANGENT
		varying vec3 vTangent;
		varying vec3 vBitangent;
	#endif
#endif`,R_=`#ifndef FLAT_SHADED
	vNormal = normalize( transformedNormal );
	#ifdef USE_TANGENT
		vTangent = normalize( transformedTangent );
		vBitangent = normalize( cross( vNormal, vTangent ) * tangent.w );
	#endif
#endif`,P_=`#ifdef USE_NORMALMAP
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
#endif`,L_=`#ifdef USE_CLEARCOAT
	vec3 clearcoatNormal = nonPerturbedNormal;
#endif`,I_=`#ifdef USE_CLEARCOAT_NORMALMAP
	vec3 clearcoatMapN = texture2D( clearcoatNormalMap, vClearcoatNormalMapUv ).xyz * 2.0 - 1.0;
	clearcoatMapN.xy *= clearcoatNormalScale;
	clearcoatNormal = normalize( tbn2 * clearcoatMapN );
#endif`,D_=`#ifdef USE_CLEARCOATMAP
	uniform sampler2D clearcoatMap;
#endif
#ifdef USE_CLEARCOAT_NORMALMAP
	uniform sampler2D clearcoatNormalMap;
	uniform vec2 clearcoatNormalScale;
#endif
#ifdef USE_CLEARCOAT_ROUGHNESSMAP
	uniform sampler2D clearcoatRoughnessMap;
#endif`,N_=`#ifdef USE_IRIDESCENCEMAP
	uniform sampler2D iridescenceMap;
#endif
#ifdef USE_IRIDESCENCE_THICKNESSMAP
	uniform sampler2D iridescenceThicknessMap;
#endif`,F_=`#ifdef OPAQUE
diffuseColor.a = 1.0;
#endif
#ifdef USE_TRANSMISSION
diffuseColor.a *= material.transmissionAlpha;
#endif
gl_FragColor = vec4( outgoingLight, diffuseColor.a );`,U_=`vec3 packNormalToRGB( const in vec3 normal ) {
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
}`,z_=`#ifdef PREMULTIPLIED_ALPHA
	gl_FragColor.rgb *= gl_FragColor.a;
#endif`,B_=`vec4 mvPosition = vec4( transformed, 1.0 );
#ifdef USE_BATCHING
	mvPosition = batchingMatrix * mvPosition;
#endif
#ifdef USE_INSTANCING
	mvPosition = instanceMatrix * mvPosition;
#endif
mvPosition = modelViewMatrix * mvPosition;
gl_Position = projectionMatrix * mvPosition;`,O_=`#ifdef DITHERING
	gl_FragColor.rgb = dithering( gl_FragColor.rgb );
#endif`,G_=`#ifdef DITHERING
	vec3 dithering( vec3 color ) {
		float grid_position = rand( gl_FragCoord.xy );
		vec3 dither_shift_RGB = vec3( 0.25 / 255.0, -0.25 / 255.0, 0.25 / 255.0 );
		dither_shift_RGB = mix( 2.0 * dither_shift_RGB, -2.0 * dither_shift_RGB, grid_position );
		return color + dither_shift_RGB;
	}
#endif`,V_=`float roughnessFactor = roughness;
#ifdef USE_ROUGHNESSMAP
	vec4 texelRoughness = texture2D( roughnessMap, vRoughnessMapUv );
	roughnessFactor *= texelRoughness.g;
#endif`,k_=`#ifdef USE_ROUGHNESSMAP
	uniform sampler2D roughnessMap;
#endif`,H_=`#if NUM_SPOT_LIGHT_COORDS > 0
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
#endif`,W_=`#if NUM_SPOT_LIGHT_COORDS > 0
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
#endif`,X_=`#if ( defined( USE_SHADOWMAP ) && ( NUM_DIR_LIGHT_SHADOWS > 0 || NUM_POINT_LIGHT_SHADOWS > 0 ) ) || ( NUM_SPOT_LIGHT_COORDS > 0 )
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
#endif`,Y_=`float getShadowMask() {
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
}`,q_=`#ifdef USE_SKINNING
	mat4 boneMatX = getBoneMatrix( skinIndex.x );
	mat4 boneMatY = getBoneMatrix( skinIndex.y );
	mat4 boneMatZ = getBoneMatrix( skinIndex.z );
	mat4 boneMatW = getBoneMatrix( skinIndex.w );
#endif`,Z_=`#ifdef USE_SKINNING
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
#endif`,j_=`#ifdef USE_SKINNING
	vec4 skinVertex = bindMatrix * vec4( transformed, 1.0 );
	vec4 skinned = vec4( 0.0 );
	skinned += boneMatX * skinVertex * skinWeight.x;
	skinned += boneMatY * skinVertex * skinWeight.y;
	skinned += boneMatZ * skinVertex * skinWeight.z;
	skinned += boneMatW * skinVertex * skinWeight.w;
	transformed = ( bindMatrixInverse * skinned ).xyz;
#endif`,$_=`#ifdef USE_SKINNING
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
#endif`,K_=`float specularStrength;
#ifdef USE_SPECULARMAP
	vec4 texelSpecular = texture2D( specularMap, vSpecularMapUv );
	specularStrength = texelSpecular.r;
#else
	specularStrength = 1.0;
#endif`,J_=`#ifdef USE_SPECULARMAP
	uniform sampler2D specularMap;
#endif`,Q_=`#if defined( TONE_MAPPING )
	gl_FragColor.rgb = toneMapping( gl_FragColor.rgb );
#endif`,tv=`#ifndef saturate
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
vec3 CustomToneMapping( vec3 color ) { return color; }`,ev=`#ifdef USE_TRANSMISSION
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
#endif`,nv=`#ifdef USE_TRANSMISSION
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
#endif`,iv=`#if defined( USE_UV ) || defined( USE_ANISOTROPY )
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
#endif`,sv=`#if defined( USE_UV ) || defined( USE_ANISOTROPY )
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
#endif`,rv=`#if defined( USE_UV ) || defined( USE_ANISOTROPY )
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
#endif`,ov=`#if defined( USE_ENVMAP ) || defined( DISTANCE ) || defined ( USE_SHADOWMAP ) || defined ( USE_TRANSMISSION ) || NUM_SPOT_LIGHT_COORDS > 0
	vec4 worldPosition = vec4( transformed, 1.0 );
	#ifdef USE_BATCHING
		worldPosition = batchingMatrix * worldPosition;
	#endif
	#ifdef USE_INSTANCING
		worldPosition = instanceMatrix * worldPosition;
	#endif
	worldPosition = modelMatrix * worldPosition;
#endif`;const av=`varying vec2 vUv;
uniform mat3 uvTransform;
void main() {
	vUv = ( uvTransform * vec3( uv, 1 ) ).xy;
	gl_Position = vec4( position.xy, 1.0, 1.0 );
}`,cv=`uniform sampler2D t2D;
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
}`,lv=`varying vec3 vWorldDirection;
#include <common>
void main() {
	vWorldDirection = transformDirection( position, modelMatrix );
	#include <begin_vertex>
	#include <project_vertex>
	gl_Position.z = gl_Position.w;
}`,hv=`#ifdef ENVMAP_TYPE_CUBE
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
}`,uv=`varying vec3 vWorldDirection;
#include <common>
void main() {
	vWorldDirection = transformDirection( position, modelMatrix );
	#include <begin_vertex>
	#include <project_vertex>
	gl_Position.z = gl_Position.w;
}`,dv=`uniform samplerCube tCube;
uniform float tFlip;
uniform float opacity;
varying vec3 vWorldDirection;
void main() {
	vec4 texColor = textureCube( tCube, vec3( tFlip * vWorldDirection.x, vWorldDirection.yz ) );
	gl_FragColor = texColor;
	gl_FragColor.a *= opacity;
	#include <tonemapping_fragment>
	#include <colorspace_fragment>
}`,fv=`#include <common>
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
}`,pv=`#if DEPTH_PACKING == 3200
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
}`,mv=`#define DISTANCE
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
}`,gv=`#define DISTANCE
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
}`,xv=`varying vec3 vWorldDirection;
#include <common>
void main() {
	vWorldDirection = transformDirection( position, modelMatrix );
	#include <begin_vertex>
	#include <project_vertex>
}`,_v=`uniform sampler2D tEquirect;
varying vec3 vWorldDirection;
#include <common>
void main() {
	vec3 direction = normalize( vWorldDirection );
	vec2 sampleUV = equirectUv( direction );
	gl_FragColor = texture2D( tEquirect, sampleUV );
	#include <tonemapping_fragment>
	#include <colorspace_fragment>
}`,vv=`uniform float scale;
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
}`,Sv=`uniform vec3 diffuse;
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
}`,Mv=`#include <common>
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
}`,yv=`uniform vec3 diffuse;
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
}`,Ev=`#define LAMBERT
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
}`,wv=`#define LAMBERT
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
}`,Av=`#define MATCAP
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
}`,Tv=`#define MATCAP
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
}`,bv=`#define NORMAL
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
}`,Cv=`#define NORMAL
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
}`,Rv=`#define PHONG
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
}`,Pv=`#define PHONG
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
}`,Lv=`#define STANDARD
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
}`,Iv=`#define STANDARD
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
}`,Dv=`#define TOON
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
}`,Nv=`#define TOON
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
}`,Fv=`uniform float size;
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
}`,Uv=`uniform vec3 diffuse;
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
}`,zv=`#include <common>
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
}`,Bv=`uniform vec3 color;
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
}`,Ov=`uniform float rotation;
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
}`,Gv=`uniform vec3 diffuse;
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
}`,Qt={alphahash_fragment:cx,alphahash_pars_fragment:lx,alphamap_fragment:hx,alphamap_pars_fragment:ux,alphatest_fragment:dx,alphatest_pars_fragment:fx,aomap_fragment:px,aomap_pars_fragment:mx,batching_pars_vertex:gx,batching_vertex:xx,begin_vertex:_x,beginnormal_vertex:vx,bsdfs:Sx,iridescence_fragment:Mx,bumpmap_pars_fragment:yx,clipping_planes_fragment:Ex,clipping_planes_pars_fragment:wx,clipping_planes_pars_vertex:Ax,clipping_planes_vertex:Tx,color_fragment:bx,color_pars_fragment:Cx,color_pars_vertex:Rx,color_vertex:Px,common:Lx,cube_uv_reflection_fragment:Ix,defaultnormal_vertex:Dx,displacementmap_pars_vertex:Nx,displacementmap_vertex:Fx,emissivemap_fragment:Ux,emissivemap_pars_fragment:zx,colorspace_fragment:Bx,colorspace_pars_fragment:Ox,envmap_fragment:Gx,envmap_common_pars_fragment:Vx,envmap_pars_fragment:kx,envmap_pars_vertex:Hx,envmap_physical_pars_fragment:e_,envmap_vertex:Wx,fog_vertex:Xx,fog_pars_vertex:Yx,fog_fragment:qx,fog_pars_fragment:Zx,gradientmap_pars_fragment:jx,lightmap_fragment:$x,lightmap_pars_fragment:Kx,lights_lambert_fragment:Jx,lights_lambert_pars_fragment:Qx,lights_pars_begin:t_,lights_toon_fragment:n_,lights_toon_pars_fragment:i_,lights_phong_fragment:s_,lights_phong_pars_fragment:r_,lights_physical_fragment:o_,lights_physical_pars_fragment:a_,lights_fragment_begin:c_,lights_fragment_maps:l_,lights_fragment_end:h_,logdepthbuf_fragment:u_,logdepthbuf_pars_fragment:d_,logdepthbuf_pars_vertex:f_,logdepthbuf_vertex:p_,map_fragment:m_,map_pars_fragment:g_,map_particle_fragment:x_,map_particle_pars_fragment:__,metalnessmap_fragment:v_,metalnessmap_pars_fragment:S_,morphcolor_vertex:M_,morphnormal_vertex:y_,morphtarget_pars_vertex:E_,morphtarget_vertex:w_,normal_fragment_begin:A_,normal_fragment_maps:T_,normal_pars_fragment:b_,normal_pars_vertex:C_,normal_vertex:R_,normalmap_pars_fragment:P_,clearcoat_normal_fragment_begin:L_,clearcoat_normal_fragment_maps:I_,clearcoat_pars_fragment:D_,iridescence_pars_fragment:N_,opaque_fragment:F_,packing:U_,premultiplied_alpha_fragment:z_,project_vertex:B_,dithering_fragment:O_,dithering_pars_fragment:G_,roughnessmap_fragment:V_,roughnessmap_pars_fragment:k_,shadowmap_pars_fragment:H_,shadowmap_pars_vertex:W_,shadowmap_vertex:X_,shadowmask_pars_fragment:Y_,skinbase_vertex:q_,skinning_pars_vertex:Z_,skinning_vertex:j_,skinnormal_vertex:$_,specularmap_fragment:K_,specularmap_pars_fragment:J_,tonemapping_fragment:Q_,tonemapping_pars_fragment:tv,transmission_fragment:ev,transmission_pars_fragment:nv,uv_pars_fragment:iv,uv_pars_vertex:sv,uv_vertex:rv,worldpos_vertex:ov,background_vert:av,background_frag:cv,backgroundCube_vert:lv,backgroundCube_frag:hv,cube_vert:uv,cube_frag:dv,depth_vert:fv,depth_frag:pv,distanceRGBA_vert:mv,distanceRGBA_frag:gv,equirect_vert:xv,equirect_frag:_v,linedashed_vert:vv,linedashed_frag:Sv,meshbasic_vert:Mv,meshbasic_frag:yv,meshlambert_vert:Ev,meshlambert_frag:wv,meshmatcap_vert:Av,meshmatcap_frag:Tv,meshnormal_vert:bv,meshnormal_frag:Cv,meshphong_vert:Rv,meshphong_frag:Pv,meshphysical_vert:Lv,meshphysical_frag:Iv,meshtoon_vert:Dv,meshtoon_frag:Nv,points_vert:Fv,points_frag:Uv,shadow_vert:zv,shadow_frag:Bv,sprite_vert:Ov,sprite_frag:Gv},mt={common:{diffuse:{value:new jt(16777215)},opacity:{value:1},map:{value:null},mapTransform:{value:new ne},alphaMap:{value:null},alphaMapTransform:{value:new ne},alphaTest:{value:0}},specularmap:{specularMap:{value:null},specularMapTransform:{value:new ne}},envmap:{envMap:{value:null},flipEnvMap:{value:-1},reflectivity:{value:1},ior:{value:1.5},refractionRatio:{value:.98}},aomap:{aoMap:{value:null},aoMapIntensity:{value:1},aoMapTransform:{value:new ne}},lightmap:{lightMap:{value:null},lightMapIntensity:{value:1},lightMapTransform:{value:new ne}},bumpmap:{bumpMap:{value:null},bumpMapTransform:{value:new ne},bumpScale:{value:1}},normalmap:{normalMap:{value:null},normalMapTransform:{value:new ne},normalScale:{value:new St(1,1)}},displacementmap:{displacementMap:{value:null},displacementMapTransform:{value:new ne},displacementScale:{value:1},displacementBias:{value:0}},emissivemap:{emissiveMap:{value:null},emissiveMapTransform:{value:new ne}},metalnessmap:{metalnessMap:{value:null},metalnessMapTransform:{value:new ne}},roughnessmap:{roughnessMap:{value:null},roughnessMapTransform:{value:new ne}},gradientmap:{gradientMap:{value:null}},fog:{fogDensity:{value:25e-5},fogNear:{value:1},fogFar:{value:2e3},fogColor:{value:new jt(16777215)}},lights:{ambientLightColor:{value:[]},lightProbe:{value:[]},directionalLights:{value:[],properties:{direction:{},color:{}}},directionalLightShadows:{value:[],properties:{shadowBias:{},shadowNormalBias:{},shadowRadius:{},shadowMapSize:{}}},directionalShadowMap:{value:[]},directionalShadowMatrix:{value:[]},spotLights:{value:[],properties:{color:{},position:{},direction:{},distance:{},coneCos:{},penumbraCos:{},decay:{}}},spotLightShadows:{value:[],properties:{shadowBias:{},shadowNormalBias:{},shadowRadius:{},shadowMapSize:{}}},spotLightMap:{value:[]},spotShadowMap:{value:[]},spotLightMatrix:{value:[]},pointLights:{value:[],properties:{color:{},position:{},decay:{},distance:{}}},pointLightShadows:{value:[],properties:{shadowBias:{},shadowNormalBias:{},shadowRadius:{},shadowMapSize:{},shadowCameraNear:{},shadowCameraFar:{}}},pointShadowMap:{value:[]},pointShadowMatrix:{value:[]},hemisphereLights:{value:[],properties:{direction:{},skyColor:{},groundColor:{}}},rectAreaLights:{value:[],properties:{color:{},position:{},width:{},height:{}}},ltc_1:{value:null},ltc_2:{value:null}},points:{diffuse:{value:new jt(16777215)},opacity:{value:1},size:{value:1},scale:{value:1},map:{value:null},alphaMap:{value:null},alphaMapTransform:{value:new ne},alphaTest:{value:0},uvTransform:{value:new ne}},sprite:{diffuse:{value:new jt(16777215)},opacity:{value:1},center:{value:new St(.5,.5)},rotation:{value:0},map:{value:null},mapTransform:{value:new ne},alphaMap:{value:null},alphaMapTransform:{value:new ne},alphaTest:{value:0}}},gi={basic:{uniforms:gn([mt.common,mt.specularmap,mt.envmap,mt.aomap,mt.lightmap,mt.fog]),vertexShader:Qt.meshbasic_vert,fragmentShader:Qt.meshbasic_frag},lambert:{uniforms:gn([mt.common,mt.specularmap,mt.envmap,mt.aomap,mt.lightmap,mt.emissivemap,mt.bumpmap,mt.normalmap,mt.displacementmap,mt.fog,mt.lights,{emissive:{value:new jt(0)}}]),vertexShader:Qt.meshlambert_vert,fragmentShader:Qt.meshlambert_frag},phong:{uniforms:gn([mt.common,mt.specularmap,mt.envmap,mt.aomap,mt.lightmap,mt.emissivemap,mt.bumpmap,mt.normalmap,mt.displacementmap,mt.fog,mt.lights,{emissive:{value:new jt(0)},specular:{value:new jt(1118481)},shininess:{value:30}}]),vertexShader:Qt.meshphong_vert,fragmentShader:Qt.meshphong_frag},standard:{uniforms:gn([mt.common,mt.envmap,mt.aomap,mt.lightmap,mt.emissivemap,mt.bumpmap,mt.normalmap,mt.displacementmap,mt.roughnessmap,mt.metalnessmap,mt.fog,mt.lights,{emissive:{value:new jt(0)},roughness:{value:1},metalness:{value:0},envMapIntensity:{value:1}}]),vertexShader:Qt.meshphysical_vert,fragmentShader:Qt.meshphysical_frag},toon:{uniforms:gn([mt.common,mt.aomap,mt.lightmap,mt.emissivemap,mt.bumpmap,mt.normalmap,mt.displacementmap,mt.gradientmap,mt.fog,mt.lights,{emissive:{value:new jt(0)}}]),vertexShader:Qt.meshtoon_vert,fragmentShader:Qt.meshtoon_frag},matcap:{uniforms:gn([mt.common,mt.bumpmap,mt.normalmap,mt.displacementmap,mt.fog,{matcap:{value:null}}]),vertexShader:Qt.meshmatcap_vert,fragmentShader:Qt.meshmatcap_frag},points:{uniforms:gn([mt.points,mt.fog]),vertexShader:Qt.points_vert,fragmentShader:Qt.points_frag},dashed:{uniforms:gn([mt.common,mt.fog,{scale:{value:1},dashSize:{value:1},totalSize:{value:2}}]),vertexShader:Qt.linedashed_vert,fragmentShader:Qt.linedashed_frag},depth:{uniforms:gn([mt.common,mt.displacementmap]),vertexShader:Qt.depth_vert,fragmentShader:Qt.depth_frag},normal:{uniforms:gn([mt.common,mt.bumpmap,mt.normalmap,mt.displacementmap,{opacity:{value:1}}]),vertexShader:Qt.meshnormal_vert,fragmentShader:Qt.meshnormal_frag},sprite:{uniforms:gn([mt.sprite,mt.fog]),vertexShader:Qt.sprite_vert,fragmentShader:Qt.sprite_frag},background:{uniforms:{uvTransform:{value:new ne},t2D:{value:null},backgroundIntensity:{value:1}},vertexShader:Qt.background_vert,fragmentShader:Qt.background_frag},backgroundCube:{uniforms:{envMap:{value:null},flipEnvMap:{value:-1},backgroundBlurriness:{value:0},backgroundIntensity:{value:1}},vertexShader:Qt.backgroundCube_vert,fragmentShader:Qt.backgroundCube_frag},cube:{uniforms:{tCube:{value:null},tFlip:{value:-1},opacity:{value:1}},vertexShader:Qt.cube_vert,fragmentShader:Qt.cube_frag},equirect:{uniforms:{tEquirect:{value:null}},vertexShader:Qt.equirect_vert,fragmentShader:Qt.equirect_frag},distanceRGBA:{uniforms:gn([mt.common,mt.displacementmap,{referencePosition:{value:new b},nearDistance:{value:1},farDistance:{value:1e3}}]),vertexShader:Qt.distanceRGBA_vert,fragmentShader:Qt.distanceRGBA_frag},shadow:{uniforms:gn([mt.lights,mt.fog,{color:{value:new jt(0)},opacity:{value:1}}]),vertexShader:Qt.shadow_vert,fragmentShader:Qt.shadow_frag}};gi.physical={uniforms:gn([gi.standard.uniforms,{clearcoat:{value:0},clearcoatMap:{value:null},clearcoatMapTransform:{value:new ne},clearcoatNormalMap:{value:null},clearcoatNormalMapTransform:{value:new ne},clearcoatNormalScale:{value:new St(1,1)},clearcoatRoughness:{value:0},clearcoatRoughnessMap:{value:null},clearcoatRoughnessMapTransform:{value:new ne},iridescence:{value:0},iridescenceMap:{value:null},iridescenceMapTransform:{value:new ne},iridescenceIOR:{value:1.3},iridescenceThicknessMinimum:{value:100},iridescenceThicknessMaximum:{value:400},iridescenceThicknessMap:{value:null},iridescenceThicknessMapTransform:{value:new ne},sheen:{value:0},sheenColor:{value:new jt(0)},sheenColorMap:{value:null},sheenColorMapTransform:{value:new ne},sheenRoughness:{value:1},sheenRoughnessMap:{value:null},sheenRoughnessMapTransform:{value:new ne},transmission:{value:0},transmissionMap:{value:null},transmissionMapTransform:{value:new ne},transmissionSamplerSize:{value:new St},transmissionSamplerMap:{value:null},thickness:{value:0},thicknessMap:{value:null},thicknessMapTransform:{value:new ne},attenuationDistance:{value:0},attenuationColor:{value:new jt(0)},specularColor:{value:new jt(1,1,1)},specularColorMap:{value:null},specularColorMapTransform:{value:new ne},specularIntensity:{value:1},specularIntensityMap:{value:null},specularIntensityMapTransform:{value:new ne},anisotropyVector:{value:new St},anisotropyMap:{value:null},anisotropyMapTransform:{value:new ne}}]),vertexShader:Qt.meshphysical_vert,fragmentShader:Qt.meshphysical_frag};const Aa={r:0,b:0,g:0};function Vv(r,t,e,n,i,s,a){const o=new jt(0);let c=s===!0?0:1,l,h,u=null,f=0,d=null;function g(m,p){let _=!1,v=p.isScene===!0?p.background:null;v&&v.isTexture&&(v=(p.backgroundBlurriness>0?e:t).get(v)),v===null?x(o,c):v&&v.isColor&&(x(v,1),_=!0);const S=r.xr.getEnvironmentBlendMode();S==="additive"?n.buffers.color.setClear(0,0,0,1,a):S==="alpha-blend"&&n.buffers.color.setClear(0,0,0,0,a),(r.autoClear||_)&&r.clear(r.autoClearColor,r.autoClearDepth,r.autoClearStencil),v&&(v.isCubeTexture||v.mapping===Oc)?(h===void 0&&(h=new Zt(new Pn(1,1,1),new Fn({name:"BackgroundCubeMaterial",uniforms:Ur(gi.backgroundCube.uniforms),vertexShader:gi.backgroundCube.vertexShader,fragmentShader:gi.backgroundCube.fragmentShader,side:dn,depthTest:!1,depthWrite:!1,fog:!1})),h.geometry.deleteAttribute("normal"),h.geometry.deleteAttribute("uv"),h.onBeforeRender=function(y,M,w){this.matrixWorld.copyPosition(w.matrixWorld)},Object.defineProperty(h.material,"envMap",{get:function(){return this.uniforms.envMap.value}}),i.update(h)),h.material.uniforms.envMap.value=v,h.material.uniforms.flipEnvMap.value=v.isCubeTexture&&v.isRenderTargetTexture===!1?-1:1,h.material.uniforms.backgroundBlurriness.value=p.backgroundBlurriness,h.material.uniforms.backgroundIntensity.value=p.backgroundIntensity,h.material.toneMapped=me.getTransfer(v.colorSpace)!==Se,(u!==v||f!==v.version||d!==r.toneMapping)&&(h.material.needsUpdate=!0,u=v,f=v.version,d=r.toneMapping),h.layers.enableAll(),m.unshift(h,h.geometry,h.material,0,0,null)):v&&v.isTexture&&(l===void 0&&(l=new Zt(new kc(2,2),new Fn({name:"BackgroundMaterial",uniforms:Ur(gi.background.uniforms),vertexShader:gi.background.vertexShader,fragmentShader:gi.background.fragmentShader,side:oi,depthTest:!1,depthWrite:!1,fog:!1})),l.geometry.deleteAttribute("normal"),Object.defineProperty(l.material,"map",{get:function(){return this.uniforms.t2D.value}}),i.update(l)),l.material.uniforms.t2D.value=v,l.material.uniforms.backgroundIntensity.value=p.backgroundIntensity,l.material.toneMapped=me.getTransfer(v.colorSpace)!==Se,v.matrixAutoUpdate===!0&&v.updateMatrix(),l.material.uniforms.uvTransform.value.copy(v.matrix),(u!==v||f!==v.version||d!==r.toneMapping)&&(l.material.needsUpdate=!0,u=v,f=v.version,d=r.toneMapping),l.layers.enableAll(),m.unshift(l,l.geometry,l.material,0,0,null))}function x(m,p){m.getRGB(Aa,fm(r)),n.buffers.color.setClear(Aa.r,Aa.g,Aa.b,p,a)}return{getClearColor:function(){return o},setClearColor:function(m,p=1){o.set(m),c=p,x(o,c)},getClearAlpha:function(){return c},setClearAlpha:function(m){c=m,x(o,c)},render:g}}function kv(r,t,e,n){const i=r.getParameter(r.MAX_VERTEX_ATTRIBS),s=n.isWebGL2?null:t.get("OES_vertex_array_object"),a=n.isWebGL2||s!==null,o={},c=m(null);let l=c,h=!1;function u(L,F,z,q,O){let Y=!1;if(a){const K=x(q,z,F);l!==K&&(l=K,d(l.object)),Y=p(L,q,z,O),Y&&_(L,q,z,O)}else{const K=F.wireframe===!0;(l.geometry!==q.id||l.program!==z.id||l.wireframe!==K)&&(l.geometry=q.id,l.program=z.id,l.wireframe=K,Y=!0)}O!==null&&e.update(O,r.ELEMENT_ARRAY_BUFFER),(Y||h)&&(h=!1,T(L,F,z,q),O!==null&&r.bindBuffer(r.ELEMENT_ARRAY_BUFFER,e.get(O).buffer))}function f(){return n.isWebGL2?r.createVertexArray():s.createVertexArrayOES()}function d(L){return n.isWebGL2?r.bindVertexArray(L):s.bindVertexArrayOES(L)}function g(L){return n.isWebGL2?r.deleteVertexArray(L):s.deleteVertexArrayOES(L)}function x(L,F,z){const q=z.wireframe===!0;let O=o[L.id];O===void 0&&(O={},o[L.id]=O);let Y=O[F.id];Y===void 0&&(Y={},O[F.id]=Y);let K=Y[q];return K===void 0&&(K=m(f()),Y[q]=K),K}function m(L){const F=[],z=[],q=[];for(let O=0;O<i;O++)F[O]=0,z[O]=0,q[O]=0;return{geometry:null,program:null,wireframe:!1,newAttributes:F,enabledAttributes:z,attributeDivisors:q,object:L,attributes:{},index:null}}function p(L,F,z,q){const O=l.attributes,Y=F.attributes;let K=0;const J=z.getAttributes();for(const it in J)if(J[it].location>=0){const j=O[it];let nt=Y[it];if(nt===void 0&&(it==="instanceMatrix"&&L.instanceMatrix&&(nt=L.instanceMatrix),it==="instanceColor"&&L.instanceColor&&(nt=L.instanceColor)),j===void 0||j.attribute!==nt||nt&&j.data!==nt.data)return!0;K++}return l.attributesNum!==K||l.index!==q}function _(L,F,z,q){const O={},Y=F.attributes;let K=0;const J=z.getAttributes();for(const it in J)if(J[it].location>=0){let j=Y[it];j===void 0&&(it==="instanceMatrix"&&L.instanceMatrix&&(j=L.instanceMatrix),it==="instanceColor"&&L.instanceColor&&(j=L.instanceColor));const nt={};nt.attribute=j,j&&j.data&&(nt.data=j.data),O[it]=nt,K++}l.attributes=O,l.attributesNum=K,l.index=q}function v(){const L=l.newAttributes;for(let F=0,z=L.length;F<z;F++)L[F]=0}function S(L){y(L,0)}function y(L,F){const z=l.newAttributes,q=l.enabledAttributes,O=l.attributeDivisors;z[L]=1,q[L]===0&&(r.enableVertexAttribArray(L),q[L]=1),O[L]!==F&&((n.isWebGL2?r:t.get("ANGLE_instanced_arrays"))[n.isWebGL2?"vertexAttribDivisor":"vertexAttribDivisorANGLE"](L,F),O[L]=F)}function M(){const L=l.newAttributes,F=l.enabledAttributes;for(let z=0,q=F.length;z<q;z++)F[z]!==L[z]&&(r.disableVertexAttribArray(z),F[z]=0)}function w(L,F,z,q,O,Y,K){K===!0?r.vertexAttribIPointer(L,F,z,O,Y):r.vertexAttribPointer(L,F,z,q,O,Y)}function T(L,F,z,q){if(n.isWebGL2===!1&&(L.isInstancedMesh||q.isInstancedBufferGeometry)&&t.get("ANGLE_instanced_arrays")===null)return;v();const O=q.attributes,Y=z.getAttributes(),K=F.defaultAttributeValues;for(const J in Y){const it=Y[J];if(it.location>=0){let X=O[J];if(X===void 0&&(J==="instanceMatrix"&&L.instanceMatrix&&(X=L.instanceMatrix),J==="instanceColor"&&L.instanceColor&&(X=L.instanceColor)),X!==void 0){const j=X.normalized,nt=X.itemSize,ot=e.get(X);if(ot===void 0)continue;const ut=ot.buffer,rt=ot.type,ht=ot.bytesPerElement,dt=n.isWebGL2===!0&&(rt===r.INT||rt===r.UNSIGNED_INT||X.gpuType===$p);if(X.isInterleavedBufferAttribute){const bt=X.data,V=bt.stride,re=X.offset;if(bt.isInstancedInterleavedBuffer){for(let Tt=0;Tt<it.locationSize;Tt++)y(it.location+Tt,bt.meshPerAttribute);L.isInstancedMesh!==!0&&q._maxInstanceCount===void 0&&(q._maxInstanceCount=bt.meshPerAttribute*bt.count)}else for(let Tt=0;Tt<it.locationSize;Tt++)S(it.location+Tt);r.bindBuffer(r.ARRAY_BUFFER,ut);for(let Tt=0;Tt<it.locationSize;Tt++)w(it.location+Tt,nt/it.locationSize,rt,j,V*ht,(re+nt/it.locationSize*Tt)*ht,dt)}else{if(X.isInstancedBufferAttribute){for(let bt=0;bt<it.locationSize;bt++)y(it.location+bt,X.meshPerAttribute);L.isInstancedMesh!==!0&&q._maxInstanceCount===void 0&&(q._maxInstanceCount=X.meshPerAttribute*X.count)}else for(let bt=0;bt<it.locationSize;bt++)S(it.location+bt);r.bindBuffer(r.ARRAY_BUFFER,ut);for(let bt=0;bt<it.locationSize;bt++)w(it.location+bt,nt/it.locationSize,rt,j,nt*ht,nt/it.locationSize*bt*ht,dt)}}else if(K!==void 0){const j=K[J];if(j!==void 0)switch(j.length){case 2:r.vertexAttrib2fv(it.location,j);break;case 3:r.vertexAttrib3fv(it.location,j);break;case 4:r.vertexAttrib4fv(it.location,j);break;default:r.vertexAttrib1fv(it.location,j)}}}}M()}function E(){R();for(const L in o){const F=o[L];for(const z in F){const q=F[z];for(const O in q)g(q[O].object),delete q[O];delete F[z]}delete o[L]}}function A(L){if(o[L.id]===void 0)return;const F=o[L.id];for(const z in F){const q=F[z];for(const O in q)g(q[O].object),delete q[O];delete F[z]}delete o[L.id]}function D(L){for(const F in o){const z=o[F];if(z[L.id]===void 0)continue;const q=z[L.id];for(const O in q)g(q[O].object),delete q[O];delete z[L.id]}}function R(){N(),h=!0,l!==c&&(l=c,d(l.object))}function N(){c.geometry=null,c.program=null,c.wireframe=!1}return{setup:u,reset:R,resetDefaultState:N,dispose:E,releaseStatesOfGeometry:A,releaseStatesOfProgram:D,initAttributes:v,enableAttribute:S,disableUnusedAttributes:M}}function Hv(r,t,e,n){const i=n.isWebGL2;let s;function a(h){s=h}function o(h,u){r.drawArrays(s,h,u),e.update(u,s,1)}function c(h,u,f){if(f===0)return;let d,g;if(i)d=r,g="drawArraysInstanced";else if(d=t.get("ANGLE_instanced_arrays"),g="drawArraysInstancedANGLE",d===null){console.error("THREE.WebGLBufferRenderer: using THREE.InstancedBufferGeometry but hardware does not support extension ANGLE_instanced_arrays.");return}d[g](s,h,u,f),e.update(u,s,f)}function l(h,u,f){if(f===0)return;const d=t.get("WEBGL_multi_draw");if(d===null)for(let g=0;g<f;g++)this.render(h[g],u[g]);else{d.multiDrawArraysWEBGL(s,h,0,u,0,f);let g=0;for(let x=0;x<f;x++)g+=u[x];e.update(g,s,1)}}this.setMode=a,this.render=o,this.renderInstances=c,this.renderMultiDraw=l}function Wv(r,t,e){let n;function i(){if(n!==void 0)return n;if(t.has("EXT_texture_filter_anisotropic")===!0){const w=t.get("EXT_texture_filter_anisotropic");n=r.getParameter(w.MAX_TEXTURE_MAX_ANISOTROPY_EXT)}else n=0;return n}function s(w){if(w==="highp"){if(r.getShaderPrecisionFormat(r.VERTEX_SHADER,r.HIGH_FLOAT).precision>0&&r.getShaderPrecisionFormat(r.FRAGMENT_SHADER,r.HIGH_FLOAT).precision>0)return"highp";w="mediump"}return w==="mediump"&&r.getShaderPrecisionFormat(r.VERTEX_SHADER,r.MEDIUM_FLOAT).precision>0&&r.getShaderPrecisionFormat(r.FRAGMENT_SHADER,r.MEDIUM_FLOAT).precision>0?"mediump":"lowp"}const a=typeof WebGL2RenderingContext<"u"&&r.constructor.name==="WebGL2RenderingContext";let o=e.precision!==void 0?e.precision:"highp";const c=s(o);c!==o&&(console.warn("THREE.WebGLRenderer:",o,"not supported, using",c,"instead."),o=c);const l=a||t.has("WEBGL_draw_buffers"),h=e.logarithmicDepthBuffer===!0,u=r.getParameter(r.MAX_TEXTURE_IMAGE_UNITS),f=r.getParameter(r.MAX_VERTEX_TEXTURE_IMAGE_UNITS),d=r.getParameter(r.MAX_TEXTURE_SIZE),g=r.getParameter(r.MAX_CUBE_MAP_TEXTURE_SIZE),x=r.getParameter(r.MAX_VERTEX_ATTRIBS),m=r.getParameter(r.MAX_VERTEX_UNIFORM_VECTORS),p=r.getParameter(r.MAX_VARYING_VECTORS),_=r.getParameter(r.MAX_FRAGMENT_UNIFORM_VECTORS),v=f>0,S=a||t.has("OES_texture_float"),y=v&&S,M=a?r.getParameter(r.MAX_SAMPLES):0;return{isWebGL2:a,drawBuffers:l,getMaxAnisotropy:i,getMaxPrecision:s,precision:o,logarithmicDepthBuffer:h,maxTextures:u,maxVertexTextures:f,maxTextureSize:d,maxCubemapSize:g,maxAttributes:x,maxVertexUniforms:m,maxVaryings:p,maxFragmentUniforms:_,vertexTextures:v,floatFragmentTextures:S,floatVertexTextures:y,maxSamples:M}}function Xv(r){const t=this;let e=null,n=0,i=!1,s=!1;const a=new zi,o=new ne,c={value:null,needsUpdate:!1};this.uniform=c,this.numPlanes=0,this.numIntersection=0,this.init=function(u,f){const d=u.length!==0||f||n!==0||i;return i=f,n=u.length,d},this.beginShadows=function(){s=!0,h(null)},this.endShadows=function(){s=!1},this.setGlobalState=function(u,f){e=h(u,f,0)},this.setState=function(u,f,d){const g=u.clippingPlanes,x=u.clipIntersection,m=u.clipShadows,p=r.get(u);if(!i||g===null||g.length===0||s&&!m)s?h(null):l();else{const _=s?0:n,v=_*4;let S=p.clippingState||null;c.value=S,S=h(g,f,v,d);for(let y=0;y!==v;++y)S[y]=e[y];p.clippingState=S,this.numIntersection=x?this.numPlanes:0,this.numPlanes+=_}};function l(){c.value!==e&&(c.value=e,c.needsUpdate=n>0),t.numPlanes=n,t.numIntersection=0}function h(u,f,d,g){const x=u!==null?u.length:0;let m=null;if(x!==0){if(m=c.value,g!==!0||m===null){const p=d+x*4,_=f.matrixWorldInverse;o.getNormalMatrix(_),(m===null||m.length<p)&&(m=new Float32Array(p));for(let v=0,S=d;v!==x;++v,S+=4)a.copy(u[v]).applyMatrix4(_,o),a.normal.toArray(m,S),m[S+3]=a.constant}c.value=m,c.needsUpdate=!0}return t.numPlanes=x,t.numIntersection=0,m}}function Yv(r){let t=new WeakMap;function e(a,o){return o===Th?a.mapping=Ir:o===bh&&(a.mapping=Dr),a}function n(a){if(a&&a.isTexture){const o=a.mapping;if(o===Th||o===bh)if(t.has(a)){const c=t.get(a).texture;return e(c,a.mapping)}else{const c=a.image;if(c&&c.height>0){const l=new sx(c.height/2);return l.fromEquirectangularTexture(r,a),t.set(a,l),a.addEventListener("dispose",i),e(l.texture,a.mapping)}else return null}}return a}function i(a){const o=a.target;o.removeEventListener("dispose",i);const c=t.get(o);c!==void 0&&(t.delete(o),c.dispose())}function s(){t=new WeakMap}return{get:n,dispose:s}}class gu extends pm{constructor(t=-1,e=1,n=1,i=-1,s=.1,a=2e3){super(),this.isOrthographicCamera=!0,this.type="OrthographicCamera",this.zoom=1,this.view=null,this.left=t,this.right=e,this.top=n,this.bottom=i,this.near=s,this.far=a,this.updateProjectionMatrix()}copy(t,e){return super.copy(t,e),this.left=t.left,this.right=t.right,this.top=t.top,this.bottom=t.bottom,this.near=t.near,this.far=t.far,this.zoom=t.zoom,this.view=t.view===null?null:Object.assign({},t.view),this}setViewOffset(t,e,n,i,s,a){this.view===null&&(this.view={enabled:!0,fullWidth:1,fullHeight:1,offsetX:0,offsetY:0,width:1,height:1}),this.view.enabled=!0,this.view.fullWidth=t,this.view.fullHeight=e,this.view.offsetX=n,this.view.offsetY=i,this.view.width=s,this.view.height=a,this.updateProjectionMatrix()}clearViewOffset(){this.view!==null&&(this.view.enabled=!1),this.updateProjectionMatrix()}updateProjectionMatrix(){const t=(this.right-this.left)/(2*this.zoom),e=(this.top-this.bottom)/(2*this.zoom),n=(this.right+this.left)/2,i=(this.top+this.bottom)/2;let s=n-t,a=n+t,o=i+e,c=i-e;if(this.view!==null&&this.view.enabled){const l=(this.right-this.left)/this.view.fullWidth/this.zoom,h=(this.top-this.bottom)/this.view.fullHeight/this.zoom;s+=l*this.view.offsetX,a=s+l*this.view.width,o-=h*this.view.offsetY,c=o-h*this.view.height}this.projectionMatrix.makeOrthographic(s,a,o,c,this.near,this.far,this.coordinateSystem),this.projectionMatrixInverse.copy(this.projectionMatrix).invert()}toJSON(t){const e=super.toJSON(t);return e.object.zoom=this.zoom,e.object.left=this.left,e.object.right=this.right,e.object.top=this.top,e.object.bottom=this.bottom,e.object.near=this.near,e.object.far=this.far,this.view!==null&&(e.object.view=Object.assign({},this.view)),e}}const Sr=4,Bd=[.125,.215,.35,.446,.526,.582],Ps=20,yl=new gu,Od=new jt;let El=null,wl=0,Al=0;const Cs=(1+Math.sqrt(5))/2,ir=1/Cs,Gd=[new b(1,1,1),new b(-1,1,1),new b(1,1,-1),new b(-1,1,-1),new b(0,Cs,ir),new b(0,Cs,-ir),new b(ir,0,Cs),new b(-ir,0,Cs),new b(Cs,ir,0),new b(-Cs,ir,0)];class Vd{constructor(t){this._renderer=t,this._pingPongRenderTarget=null,this._lodMax=0,this._cubeSize=0,this._lodPlanes=[],this._sizeLods=[],this._sigmas=[],this._blurMaterial=null,this._cubemapMaterial=null,this._equirectMaterial=null,this._compileMaterial(this._blurMaterial)}fromScene(t,e=0,n=.1,i=100){El=this._renderer.getRenderTarget(),wl=this._renderer.getActiveCubeFace(),Al=this._renderer.getActiveMipmapLevel(),this._setSize(256);const s=this._allocateTargets();return s.depthBuffer=!0,this._sceneToCubeUV(t,n,i,s),e>0&&this._blur(s,0,0,e),this._applyPMREM(s),this._cleanup(s),s}fromEquirectangular(t,e=null){return this._fromTexture(t,e)}fromCubemap(t,e=null){return this._fromTexture(t,e)}compileCubemapShader(){this._cubemapMaterial===null&&(this._cubemapMaterial=Wd(),this._compileMaterial(this._cubemapMaterial))}compileEquirectangularShader(){this._equirectMaterial===null&&(this._equirectMaterial=Hd(),this._compileMaterial(this._equirectMaterial))}dispose(){this._dispose(),this._cubemapMaterial!==null&&this._cubemapMaterial.dispose(),this._equirectMaterial!==null&&this._equirectMaterial.dispose()}_setSize(t){this._lodMax=Math.floor(Math.log2(t)),this._cubeSize=Math.pow(2,this._lodMax)}_dispose(){this._blurMaterial!==null&&this._blurMaterial.dispose(),this._pingPongRenderTarget!==null&&this._pingPongRenderTarget.dispose();for(let t=0;t<this._lodPlanes.length;t++)this._lodPlanes[t].dispose()}_cleanup(t){this._renderer.setRenderTarget(El,wl,Al),t.scissorTest=!1,Ta(t,0,0,t.width,t.height)}_fromTexture(t,e){t.mapping===Ir||t.mapping===Dr?this._setSize(t.image.length===0?16:t.image[0].width||t.image[0].image.width):this._setSize(t.image.width/4),El=this._renderer.getRenderTarget(),wl=this._renderer.getActiveCubeFace(),Al=this._renderer.getActiveMipmapLevel();const n=e||this._allocateTargets();return this._textureToCubeUV(t,n),this._applyPMREM(n),this._cleanup(n),n}_allocateTargets(){const t=3*Math.max(this._cubeSize,112),e=4*this._cubeSize,n={magFilter:Vn,minFilter:Vn,generateMipmaps:!1,type:Nr,format:ri,colorSpace:Vi,depthBuffer:!1},i=kd(t,e,n);if(this._pingPongRenderTarget===null||this._pingPongRenderTarget.width!==t||this._pingPongRenderTarget.height!==e){this._pingPongRenderTarget!==null&&this._dispose(),this._pingPongRenderTarget=kd(t,e,n);const{_lodMax:s}=this;({sizeLods:this._sizeLods,lodPlanes:this._lodPlanes,sigmas:this._sigmas}=qv(s)),this._blurMaterial=Zv(s,t,e)}return i}_compileMaterial(t){const e=new Zt(this._lodPlanes[0],t);this._renderer.compile(e,yl)}_sceneToCubeUV(t,e,n,i){const o=new Ln(90,1,e,n),c=[1,-1,1,1,1,1],l=[1,1,1,-1,-1,-1],h=this._renderer,u=h.autoClear,f=h.toneMapping;h.getClearColor(Od),h.toneMapping=as,h.autoClear=!1;const d=new Fe({name:"PMREM.Background",side:dn,depthWrite:!1,depthTest:!1}),g=new Zt(new Pn,d);let x=!1;const m=t.background;m?m.isColor&&(d.color.copy(m),t.background=null,x=!0):(d.color.copy(Od),x=!0);for(let p=0;p<6;p++){const _=p%3;_===0?(o.up.set(0,c[p],0),o.lookAt(l[p],0,0)):_===1?(o.up.set(0,0,c[p]),o.lookAt(0,l[p],0)):(o.up.set(0,c[p],0),o.lookAt(0,0,l[p]));const v=this._cubeSize;Ta(i,_*v,p>2?v:0,v,v),h.setRenderTarget(i),x&&h.render(g,o),h.render(t,o)}g.geometry.dispose(),g.material.dispose(),h.toneMapping=f,h.autoClear=u,t.background=m}_textureToCubeUV(t,e){const n=this._renderer,i=t.mapping===Ir||t.mapping===Dr;i?(this._cubemapMaterial===null&&(this._cubemapMaterial=Wd()),this._cubemapMaterial.uniforms.flipEnvMap.value=t.isRenderTargetTexture===!1?-1:1):this._equirectMaterial===null&&(this._equirectMaterial=Hd());const s=i?this._cubemapMaterial:this._equirectMaterial,a=new Zt(this._lodPlanes[0],s),o=s.uniforms;o.envMap.value=t;const c=this._cubeSize;Ta(e,0,0,3*c,2*c),n.setRenderTarget(e),n.render(a,yl)}_applyPMREM(t){const e=this._renderer,n=e.autoClear;e.autoClear=!1;for(let i=1;i<this._lodPlanes.length;i++){const s=Math.sqrt(this._sigmas[i]*this._sigmas[i]-this._sigmas[i-1]*this._sigmas[i-1]),a=Gd[(i-1)%Gd.length];this._blur(t,i-1,i,s,a)}e.autoClear=n}_blur(t,e,n,i,s){const a=this._pingPongRenderTarget;this._halfBlur(t,a,e,n,i,"latitudinal",s),this._halfBlur(a,t,n,n,i,"longitudinal",s)}_halfBlur(t,e,n,i,s,a,o){const c=this._renderer,l=this._blurMaterial;a!=="latitudinal"&&a!=="longitudinal"&&console.error("blur direction must be either latitudinal or longitudinal!");const h=3,u=new Zt(this._lodPlanes[i],l),f=l.uniforms,d=this._sizeLods[n]-1,g=isFinite(s)?Math.PI/(2*d):2*Math.PI/(2*Ps-1),x=s/g,m=isFinite(s)?1+Math.floor(h*x):Ps;m>Ps&&console.warn(`sigmaRadians, ${s}, is too large and will clip, as it requested ${m} samples when the maximum is set to ${Ps}`);const p=[];let _=0;for(let w=0;w<Ps;++w){const T=w/x,E=Math.exp(-T*T/2);p.push(E),w===0?_+=E:w<m&&(_+=2*E)}for(let w=0;w<p.length;w++)p[w]=p[w]/_;f.envMap.value=t.texture,f.samples.value=m,f.weights.value=p,f.latitudinal.value=a==="latitudinal",o&&(f.poleAxis.value=o);const{_lodMax:v}=this;f.dTheta.value=g,f.mipInt.value=v-n;const S=this._sizeLods[i],y=3*S*(i>v-Sr?i-v+Sr:0),M=4*(this._cubeSize-S);Ta(e,y,M,3*S,2*S),c.setRenderTarget(e),c.render(u,yl)}}function qv(r){const t=[],e=[],n=[];let i=r;const s=r-Sr+1+Bd.length;for(let a=0;a<s;a++){const o=Math.pow(2,i);e.push(o);let c=1/o;a>r-Sr?c=Bd[a-r+Sr-1]:a===0&&(c=0),n.push(c);const l=1/(o-2),h=-l,u=1+l,f=[h,h,u,h,u,u,h,h,u,u,h,u],d=6,g=6,x=3,m=2,p=1,_=new Float32Array(x*g*d),v=new Float32Array(m*g*d),S=new Float32Array(p*g*d);for(let M=0;M<d;M++){const w=M%3*2/3-1,T=M>2?0:-1,E=[w,T,0,w+2/3,T,0,w+2/3,T+1,0,w,T,0,w+2/3,T+1,0,w,T+1,0];_.set(E,x*g*M),v.set(f,m*g*M);const A=[M,M,M,M,M,M];S.set(A,p*g*M)}const y=new ye;y.setAttribute("position",new Ne(_,x)),y.setAttribute("uv",new Ne(v,m)),y.setAttribute("faceIndex",new Ne(S,p)),t.push(y),i>Sr&&i--}return{lodPlanes:t,sizeLods:e,sigmas:n}}function kd(r,t,e){const n=new tn(r,t,e);return n.texture.mapping=Oc,n.texture.name="PMREM.cubeUv",n.scissorTest=!0,n}function Ta(r,t,e,n,i){r.viewport.set(t,e,n,i),r.scissor.set(t,e,n,i)}function Zv(r,t,e){const n=new Float32Array(Ps),i=new b(0,1,0);return new Fn({name:"SphericalGaussianBlur",defines:{n:Ps,CUBEUV_TEXEL_WIDTH:1/t,CUBEUV_TEXEL_HEIGHT:1/e,CUBEUV_MAX_MIP:`${r}.0`},uniforms:{envMap:{value:null},samples:{value:1},weights:{value:n},latitudinal:{value:!1},dTheta:{value:0},mipInt:{value:0},poleAxis:{value:i}},vertexShader:xu(),fragmentShader:`

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
		`,blending:os,depthTest:!1,depthWrite:!1})}function Hd(){return new Fn({name:"EquirectangularToCubeUV",uniforms:{envMap:{value:null}},vertexShader:xu(),fragmentShader:`

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
		`,blending:os,depthTest:!1,depthWrite:!1})}function Wd(){return new Fn({name:"CubemapToCubeUV",uniforms:{envMap:{value:null},flipEnvMap:{value:-1}},vertexShader:xu(),fragmentShader:`

			precision mediump float;
			precision mediump int;

			uniform float flipEnvMap;

			varying vec3 vOutputDirection;

			uniform samplerCube envMap;

			void main() {

				gl_FragColor = textureCube( envMap, vec3( flipEnvMap * vOutputDirection.x, vOutputDirection.yz ) );

			}
		`,blending:os,depthTest:!1,depthWrite:!1})}function xu(){return`

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
	`}function jv(r){let t=new WeakMap,e=null;function n(o){if(o&&o.isTexture){const c=o.mapping,l=c===Th||c===bh,h=c===Ir||c===Dr;if(l||h)if(o.isRenderTargetTexture&&o.needsPMREMUpdate===!0){o.needsPMREMUpdate=!1;let u=t.get(o);return e===null&&(e=new Vd(r)),u=l?e.fromEquirectangular(o,u):e.fromCubemap(o,u),t.set(o,u),u.texture}else{if(t.has(o))return t.get(o).texture;{const u=o.image;if(l&&u&&u.height>0||h&&u&&i(u)){e===null&&(e=new Vd(r));const f=l?e.fromEquirectangular(o):e.fromCubemap(o);return t.set(o,f),o.addEventListener("dispose",s),f.texture}else return null}}}return o}function i(o){let c=0;const l=6;for(let h=0;h<l;h++)o[h]!==void 0&&c++;return c===l}function s(o){const c=o.target;c.removeEventListener("dispose",s);const l=t.get(c);l!==void 0&&(t.delete(c),l.dispose())}function a(){t=new WeakMap,e!==null&&(e.dispose(),e=null)}return{get:n,dispose:a}}function $v(r){const t={};function e(n){if(t[n]!==void 0)return t[n];let i;switch(n){case"WEBGL_depth_texture":i=r.getExtension("WEBGL_depth_texture")||r.getExtension("MOZ_WEBGL_depth_texture")||r.getExtension("WEBKIT_WEBGL_depth_texture");break;case"EXT_texture_filter_anisotropic":i=r.getExtension("EXT_texture_filter_anisotropic")||r.getExtension("MOZ_EXT_texture_filter_anisotropic")||r.getExtension("WEBKIT_EXT_texture_filter_anisotropic");break;case"WEBGL_compressed_texture_s3tc":i=r.getExtension("WEBGL_compressed_texture_s3tc")||r.getExtension("MOZ_WEBGL_compressed_texture_s3tc")||r.getExtension("WEBKIT_WEBGL_compressed_texture_s3tc");break;case"WEBGL_compressed_texture_pvrtc":i=r.getExtension("WEBGL_compressed_texture_pvrtc")||r.getExtension("WEBKIT_WEBGL_compressed_texture_pvrtc");break;default:i=r.getExtension(n)}return t[n]=i,i}return{has:function(n){return e(n)!==null},init:function(n){n.isWebGL2?(e("EXT_color_buffer_float"),e("WEBGL_clip_cull_distance")):(e("WEBGL_depth_texture"),e("OES_texture_float"),e("OES_texture_half_float"),e("OES_texture_half_float_linear"),e("OES_standard_derivatives"),e("OES_element_index_uint"),e("OES_vertex_array_object"),e("ANGLE_instanced_arrays")),e("OES_texture_float_linear"),e("EXT_color_buffer_half_float"),e("WEBGL_multisampled_render_to_texture")},get:function(n){const i=e(n);return i===null&&console.warn("THREE.WebGLRenderer: "+n+" extension not supported."),i}}}function Kv(r,t,e,n){const i={},s=new WeakMap;function a(u){const f=u.target;f.index!==null&&t.remove(f.index);for(const g in f.attributes)t.remove(f.attributes[g]);for(const g in f.morphAttributes){const x=f.morphAttributes[g];for(let m=0,p=x.length;m<p;m++)t.remove(x[m])}f.removeEventListener("dispose",a),delete i[f.id];const d=s.get(f);d&&(t.remove(d),s.delete(f)),n.releaseStatesOfGeometry(f),f.isInstancedBufferGeometry===!0&&delete f._maxInstanceCount,e.memory.geometries--}function o(u,f){return i[f.id]===!0||(f.addEventListener("dispose",a),i[f.id]=!0,e.memory.geometries++),f}function c(u){const f=u.attributes;for(const g in f)t.update(f[g],r.ARRAY_BUFFER);const d=u.morphAttributes;for(const g in d){const x=d[g];for(let m=0,p=x.length;m<p;m++)t.update(x[m],r.ARRAY_BUFFER)}}function l(u){const f=[],d=u.index,g=u.attributes.position;let x=0;if(d!==null){const _=d.array;x=d.version;for(let v=0,S=_.length;v<S;v+=3){const y=_[v+0],M=_[v+1],w=_[v+2];f.push(y,M,M,w,w,y)}}else if(g!==void 0){const _=g.array;x=g.version;for(let v=0,S=_.length/3-1;v<S;v+=3){const y=v+0,M=v+1,w=v+2;f.push(y,M,M,w,w,y)}}else return;const m=new(rm(f)?dm:um)(f,1);m.version=x;const p=s.get(u);p&&t.remove(p),s.set(u,m)}function h(u){const f=s.get(u);if(f){const d=u.index;d!==null&&f.version<d.version&&l(u)}else l(u);return s.get(u)}return{get:o,update:c,getWireframeAttribute:h}}function Jv(r,t,e,n){const i=n.isWebGL2;let s;function a(d){s=d}let o,c;function l(d){o=d.type,c=d.bytesPerElement}function h(d,g){r.drawElements(s,g,o,d*c),e.update(g,s,1)}function u(d,g,x){if(x===0)return;let m,p;if(i)m=r,p="drawElementsInstanced";else if(m=t.get("ANGLE_instanced_arrays"),p="drawElementsInstancedANGLE",m===null){console.error("THREE.WebGLIndexedBufferRenderer: using THREE.InstancedBufferGeometry but hardware does not support extension ANGLE_instanced_arrays.");return}m[p](s,g,o,d*c,x),e.update(g,s,x)}function f(d,g,x){if(x===0)return;const m=t.get("WEBGL_multi_draw");if(m===null)for(let p=0;p<x;p++)this.render(d[p]/c,g[p]);else{m.multiDrawElementsWEBGL(s,g,0,o,d,0,x);let p=0;for(let _=0;_<x;_++)p+=g[_];e.update(p,s,1)}}this.setMode=a,this.setIndex=l,this.render=h,this.renderInstances=u,this.renderMultiDraw=f}function Qv(r){const t={geometries:0,textures:0},e={frame:0,calls:0,triangles:0,points:0,lines:0};function n(s,a,o){switch(e.calls++,a){case r.TRIANGLES:e.triangles+=o*(s/3);break;case r.LINES:e.lines+=o*(s/2);break;case r.LINE_STRIP:e.lines+=o*(s-1);break;case r.LINE_LOOP:e.lines+=o*s;break;case r.POINTS:e.points+=o*s;break;default:console.error("THREE.WebGLInfo: Unknown draw mode:",a);break}}function i(){e.calls=0,e.triangles=0,e.points=0,e.lines=0}return{memory:t,render:e,programs:null,autoReset:!0,reset:i,update:n}}function tS(r,t){return r[0]-t[0]}function eS(r,t){return Math.abs(t[1])-Math.abs(r[1])}function nS(r,t,e){const n={},i=new Float32Array(8),s=new WeakMap,a=new Je,o=[];for(let l=0;l<8;l++)o[l]=[l,0];function c(l,h,u){const f=l.morphTargetInfluences;if(t.isWebGL2===!0){const d=h.morphAttributes.position||h.morphAttributes.normal||h.morphAttributes.color,g=d!==void 0?d.length:0;let x=s.get(h);if(x===void 0||x.count!==g){let L=function(){R.dispose(),s.delete(h),h.removeEventListener("dispose",L)};x!==void 0&&x.texture.dispose();const _=h.morphAttributes.position!==void 0,v=h.morphAttributes.normal!==void 0,S=h.morphAttributes.color!==void 0,y=h.morphAttributes.position||[],M=h.morphAttributes.normal||[],w=h.morphAttributes.color||[];let T=0;_===!0&&(T=1),v===!0&&(T=2),S===!0&&(T=3);let E=h.attributes.position.count*T,A=1;E>t.maxTextureSize&&(A=Math.ceil(E/t.maxTextureSize),E=t.maxTextureSize);const D=new Float32Array(E*A*4*g),R=new cm(D,E,A,g);R.type=is,R.needsUpdate=!0;const N=T*4;for(let F=0;F<g;F++){const z=y[F],q=M[F],O=w[F],Y=E*A*4*F;for(let K=0;K<z.count;K++){const J=K*N;_===!0&&(a.fromBufferAttribute(z,K),D[Y+J+0]=a.x,D[Y+J+1]=a.y,D[Y+J+2]=a.z,D[Y+J+3]=0),v===!0&&(a.fromBufferAttribute(q,K),D[Y+J+4]=a.x,D[Y+J+5]=a.y,D[Y+J+6]=a.z,D[Y+J+7]=0),S===!0&&(a.fromBufferAttribute(O,K),D[Y+J+8]=a.x,D[Y+J+9]=a.y,D[Y+J+10]=a.z,D[Y+J+11]=O.itemSize===4?a.w:1)}}x={count:g,texture:R,size:new St(E,A)},s.set(h,x),h.addEventListener("dispose",L)}let m=0;for(let _=0;_<f.length;_++)m+=f[_];const p=h.morphTargetsRelative?1:1-m;u.getUniforms().setValue(r,"morphTargetBaseInfluence",p),u.getUniforms().setValue(r,"morphTargetInfluences",f),u.getUniforms().setValue(r,"morphTargetsTexture",x.texture,e),u.getUniforms().setValue(r,"morphTargetsTextureSize",x.size)}else{const d=f===void 0?0:f.length;let g=n[h.id];if(g===void 0||g.length!==d){g=[];for(let v=0;v<d;v++)g[v]=[v,0];n[h.id]=g}for(let v=0;v<d;v++){const S=g[v];S[0]=v,S[1]=f[v]}g.sort(eS);for(let v=0;v<8;v++)v<d&&g[v][1]?(o[v][0]=g[v][0],o[v][1]=g[v][1]):(o[v][0]=Number.MAX_SAFE_INTEGER,o[v][1]=0);o.sort(tS);const x=h.morphAttributes.position,m=h.morphAttributes.normal;let p=0;for(let v=0;v<8;v++){const S=o[v],y=S[0],M=S[1];y!==Number.MAX_SAFE_INTEGER&&M?(x&&h.getAttribute("morphTarget"+v)!==x[y]&&h.setAttribute("morphTarget"+v,x[y]),m&&h.getAttribute("morphNormal"+v)!==m[y]&&h.setAttribute("morphNormal"+v,m[y]),i[v]=M,p+=M):(x&&h.hasAttribute("morphTarget"+v)===!0&&h.deleteAttribute("morphTarget"+v),m&&h.hasAttribute("morphNormal"+v)===!0&&h.deleteAttribute("morphNormal"+v),i[v]=0)}const _=h.morphTargetsRelative?1:1-p;u.getUniforms().setValue(r,"morphTargetBaseInfluence",_),u.getUniforms().setValue(r,"morphTargetInfluences",i)}}return{update:c}}function iS(r,t,e,n){let i=new WeakMap;function s(c){const l=n.render.frame,h=c.geometry,u=t.get(c,h);if(i.get(u)!==l&&(t.update(u),i.set(u,l)),c.isInstancedMesh&&(c.hasEventListener("dispose",o)===!1&&c.addEventListener("dispose",o),i.get(c)!==l&&(e.update(c.instanceMatrix,r.ARRAY_BUFFER),c.instanceColor!==null&&e.update(c.instanceColor,r.ARRAY_BUFFER),i.set(c,l))),c.isSkinnedMesh){const f=c.skeleton;i.get(f)!==l&&(f.update(),i.set(f,l))}return u}function a(){i=new WeakMap}function o(c){const l=c.target;l.removeEventListener("dispose",o),e.remove(l.instanceMatrix),l.instanceColor!==null&&e.remove(l.instanceColor)}return{update:s,dispose:a}}class xm extends Dn{constructor(t,e,n,i,s,a,o,c,l,h){if(h=h!==void 0?h:Ds,h!==Ds&&h!==Fr)throw new Error("DepthTexture format must be either THREE.DepthFormat or THREE.DepthStencilFormat");n===void 0&&h===Ds&&(n=ns),n===void 0&&h===Fr&&(n=Is),super(null,i,s,a,o,c,h,n,l),this.isDepthTexture=!0,this.image={width:t,height:e},this.magFilter=o!==void 0?o:_n,this.minFilter=c!==void 0?c:_n,this.flipY=!1,this.generateMipmaps=!1,this.compareFunction=null}copy(t){return super.copy(t),this.compareFunction=t.compareFunction,this}toJSON(t){const e=super.toJSON(t);return this.compareFunction!==null&&(e.compareFunction=this.compareFunction),e}}const _m=new Dn,vm=new xm(1,1);vm.compareFunction=sm;const Sm=new cm,Mm=new V0,ym=new mm,Xd=[],Yd=[],qd=new Float32Array(16),Zd=new Float32Array(9),jd=new Float32Array(4);function Vr(r,t,e){const n=r[0];if(n<=0||n>0)return r;const i=t*e;let s=Xd[i];if(s===void 0&&(s=new Float32Array(i),Xd[i]=s),t!==0){n.toArray(s,0);for(let a=1,o=0;a!==t;++a)o+=e,r[a].toArray(s,o)}return s}function Xe(r,t){if(r.length!==t.length)return!1;for(let e=0,n=r.length;e<n;e++)if(r[e]!==t[e])return!1;return!0}function Ye(r,t){for(let e=0,n=t.length;e<n;e++)r[e]=t[e]}function Hc(r,t){let e=Yd[t];e===void 0&&(e=new Int32Array(t),Yd[t]=e);for(let n=0;n!==t;++n)e[n]=r.allocateTextureUnit();return e}function sS(r,t){const e=this.cache;e[0]!==t&&(r.uniform1f(this.addr,t),e[0]=t)}function rS(r,t){const e=this.cache;if(t.x!==void 0)(e[0]!==t.x||e[1]!==t.y)&&(r.uniform2f(this.addr,t.x,t.y),e[0]=t.x,e[1]=t.y);else{if(Xe(e,t))return;r.uniform2fv(this.addr,t),Ye(e,t)}}function oS(r,t){const e=this.cache;if(t.x!==void 0)(e[0]!==t.x||e[1]!==t.y||e[2]!==t.z)&&(r.uniform3f(this.addr,t.x,t.y,t.z),e[0]=t.x,e[1]=t.y,e[2]=t.z);else if(t.r!==void 0)(e[0]!==t.r||e[1]!==t.g||e[2]!==t.b)&&(r.uniform3f(this.addr,t.r,t.g,t.b),e[0]=t.r,e[1]=t.g,e[2]=t.b);else{if(Xe(e,t))return;r.uniform3fv(this.addr,t),Ye(e,t)}}function aS(r,t){const e=this.cache;if(t.x!==void 0)(e[0]!==t.x||e[1]!==t.y||e[2]!==t.z||e[3]!==t.w)&&(r.uniform4f(this.addr,t.x,t.y,t.z,t.w),e[0]=t.x,e[1]=t.y,e[2]=t.z,e[3]=t.w);else{if(Xe(e,t))return;r.uniform4fv(this.addr,t),Ye(e,t)}}function cS(r,t){const e=this.cache,n=t.elements;if(n===void 0){if(Xe(e,t))return;r.uniformMatrix2fv(this.addr,!1,t),Ye(e,t)}else{if(Xe(e,n))return;jd.set(n),r.uniformMatrix2fv(this.addr,!1,jd),Ye(e,n)}}function lS(r,t){const e=this.cache,n=t.elements;if(n===void 0){if(Xe(e,t))return;r.uniformMatrix3fv(this.addr,!1,t),Ye(e,t)}else{if(Xe(e,n))return;Zd.set(n),r.uniformMatrix3fv(this.addr,!1,Zd),Ye(e,n)}}function hS(r,t){const e=this.cache,n=t.elements;if(n===void 0){if(Xe(e,t))return;r.uniformMatrix4fv(this.addr,!1,t),Ye(e,t)}else{if(Xe(e,n))return;qd.set(n),r.uniformMatrix4fv(this.addr,!1,qd),Ye(e,n)}}function uS(r,t){const e=this.cache;e[0]!==t&&(r.uniform1i(this.addr,t),e[0]=t)}function dS(r,t){const e=this.cache;if(t.x!==void 0)(e[0]!==t.x||e[1]!==t.y)&&(r.uniform2i(this.addr,t.x,t.y),e[0]=t.x,e[1]=t.y);else{if(Xe(e,t))return;r.uniform2iv(this.addr,t),Ye(e,t)}}function fS(r,t){const e=this.cache;if(t.x!==void 0)(e[0]!==t.x||e[1]!==t.y||e[2]!==t.z)&&(r.uniform3i(this.addr,t.x,t.y,t.z),e[0]=t.x,e[1]=t.y,e[2]=t.z);else{if(Xe(e,t))return;r.uniform3iv(this.addr,t),Ye(e,t)}}function pS(r,t){const e=this.cache;if(t.x!==void 0)(e[0]!==t.x||e[1]!==t.y||e[2]!==t.z||e[3]!==t.w)&&(r.uniform4i(this.addr,t.x,t.y,t.z,t.w),e[0]=t.x,e[1]=t.y,e[2]=t.z,e[3]=t.w);else{if(Xe(e,t))return;r.uniform4iv(this.addr,t),Ye(e,t)}}function mS(r,t){const e=this.cache;e[0]!==t&&(r.uniform1ui(this.addr,t),e[0]=t)}function gS(r,t){const e=this.cache;if(t.x!==void 0)(e[0]!==t.x||e[1]!==t.y)&&(r.uniform2ui(this.addr,t.x,t.y),e[0]=t.x,e[1]=t.y);else{if(Xe(e,t))return;r.uniform2uiv(this.addr,t),Ye(e,t)}}function xS(r,t){const e=this.cache;if(t.x!==void 0)(e[0]!==t.x||e[1]!==t.y||e[2]!==t.z)&&(r.uniform3ui(this.addr,t.x,t.y,t.z),e[0]=t.x,e[1]=t.y,e[2]=t.z);else{if(Xe(e,t))return;r.uniform3uiv(this.addr,t),Ye(e,t)}}function _S(r,t){const e=this.cache;if(t.x!==void 0)(e[0]!==t.x||e[1]!==t.y||e[2]!==t.z||e[3]!==t.w)&&(r.uniform4ui(this.addr,t.x,t.y,t.z,t.w),e[0]=t.x,e[1]=t.y,e[2]=t.z,e[3]=t.w);else{if(Xe(e,t))return;r.uniform4uiv(this.addr,t),Ye(e,t)}}function vS(r,t,e){const n=this.cache,i=e.allocateTextureUnit();n[0]!==i&&(r.uniform1i(this.addr,i),n[0]=i);const s=this.type===r.SAMPLER_2D_SHADOW?vm:_m;e.setTexture2D(t||s,i)}function SS(r,t,e){const n=this.cache,i=e.allocateTextureUnit();n[0]!==i&&(r.uniform1i(this.addr,i),n[0]=i),e.setTexture3D(t||Mm,i)}function MS(r,t,e){const n=this.cache,i=e.allocateTextureUnit();n[0]!==i&&(r.uniform1i(this.addr,i),n[0]=i),e.setTextureCube(t||ym,i)}function yS(r,t,e){const n=this.cache,i=e.allocateTextureUnit();n[0]!==i&&(r.uniform1i(this.addr,i),n[0]=i),e.setTexture2DArray(t||Sm,i)}function ES(r){switch(r){case 5126:return sS;case 35664:return rS;case 35665:return oS;case 35666:return aS;case 35674:return cS;case 35675:return lS;case 35676:return hS;case 5124:case 35670:return uS;case 35667:case 35671:return dS;case 35668:case 35672:return fS;case 35669:case 35673:return pS;case 5125:return mS;case 36294:return gS;case 36295:return xS;case 36296:return _S;case 35678:case 36198:case 36298:case 36306:case 35682:return vS;case 35679:case 36299:case 36307:return SS;case 35680:case 36300:case 36308:case 36293:return MS;case 36289:case 36303:case 36311:case 36292:return yS}}function wS(r,t){r.uniform1fv(this.addr,t)}function AS(r,t){const e=Vr(t,this.size,2);r.uniform2fv(this.addr,e)}function TS(r,t){const e=Vr(t,this.size,3);r.uniform3fv(this.addr,e)}function bS(r,t){const e=Vr(t,this.size,4);r.uniform4fv(this.addr,e)}function CS(r,t){const e=Vr(t,this.size,4);r.uniformMatrix2fv(this.addr,!1,e)}function RS(r,t){const e=Vr(t,this.size,9);r.uniformMatrix3fv(this.addr,!1,e)}function PS(r,t){const e=Vr(t,this.size,16);r.uniformMatrix4fv(this.addr,!1,e)}function LS(r,t){r.uniform1iv(this.addr,t)}function IS(r,t){r.uniform2iv(this.addr,t)}function DS(r,t){r.uniform3iv(this.addr,t)}function NS(r,t){r.uniform4iv(this.addr,t)}function FS(r,t){r.uniform1uiv(this.addr,t)}function US(r,t){r.uniform2uiv(this.addr,t)}function zS(r,t){r.uniform3uiv(this.addr,t)}function BS(r,t){r.uniform4uiv(this.addr,t)}function OS(r,t,e){const n=this.cache,i=t.length,s=Hc(e,i);Xe(n,s)||(r.uniform1iv(this.addr,s),Ye(n,s));for(let a=0;a!==i;++a)e.setTexture2D(t[a]||_m,s[a])}function GS(r,t,e){const n=this.cache,i=t.length,s=Hc(e,i);Xe(n,s)||(r.uniform1iv(this.addr,s),Ye(n,s));for(let a=0;a!==i;++a)e.setTexture3D(t[a]||Mm,s[a])}function VS(r,t,e){const n=this.cache,i=t.length,s=Hc(e,i);Xe(n,s)||(r.uniform1iv(this.addr,s),Ye(n,s));for(let a=0;a!==i;++a)e.setTextureCube(t[a]||ym,s[a])}function kS(r,t,e){const n=this.cache,i=t.length,s=Hc(e,i);Xe(n,s)||(r.uniform1iv(this.addr,s),Ye(n,s));for(let a=0;a!==i;++a)e.setTexture2DArray(t[a]||Sm,s[a])}function HS(r){switch(r){case 5126:return wS;case 35664:return AS;case 35665:return TS;case 35666:return bS;case 35674:return CS;case 35675:return RS;case 35676:return PS;case 5124:case 35670:return LS;case 35667:case 35671:return IS;case 35668:case 35672:return DS;case 35669:case 35673:return NS;case 5125:return FS;case 36294:return US;case 36295:return zS;case 36296:return BS;case 35678:case 36198:case 36298:case 36306:case 35682:return OS;case 35679:case 36299:case 36307:return GS;case 35680:case 36300:case 36308:case 36293:return VS;case 36289:case 36303:case 36311:case 36292:return kS}}class WS{constructor(t,e,n){this.id=t,this.addr=n,this.cache=[],this.type=e.type,this.setValue=ES(e.type)}}class XS{constructor(t,e,n){this.id=t,this.addr=n,this.cache=[],this.type=e.type,this.size=e.size,this.setValue=HS(e.type)}}class YS{constructor(t){this.id=t,this.seq=[],this.map={}}setValue(t,e,n){const i=this.seq;for(let s=0,a=i.length;s!==a;++s){const o=i[s];o.setValue(t,e[o.id],n)}}}const Tl=/(\w+)(\])?(\[|\.)?/g;function $d(r,t){r.seq.push(t),r.map[t.id]=t}function qS(r,t,e){const n=r.name,i=n.length;for(Tl.lastIndex=0;;){const s=Tl.exec(n),a=Tl.lastIndex;let o=s[1];const c=s[2]==="]",l=s[3];if(c&&(o=o|0),l===void 0||l==="["&&a+2===i){$d(e,l===void 0?new WS(o,r,t):new XS(o,r,t));break}else{let u=e.map[o];u===void 0&&(u=new YS(o),$d(e,u)),e=u}}}class ac{constructor(t,e){this.seq=[],this.map={};const n=t.getProgramParameter(e,t.ACTIVE_UNIFORMS);for(let i=0;i<n;++i){const s=t.getActiveUniform(e,i),a=t.getUniformLocation(e,s.name);qS(s,a,this)}}setValue(t,e,n,i){const s=this.map[e];s!==void 0&&s.setValue(t,n,i)}setOptional(t,e,n){const i=e[n];i!==void 0&&this.setValue(t,n,i)}static upload(t,e,n,i){for(let s=0,a=e.length;s!==a;++s){const o=e[s],c=n[o.id];c.needsUpdate!==!1&&o.setValue(t,c.value,i)}}static seqWithValue(t,e){const n=[];for(let i=0,s=t.length;i!==s;++i){const a=t[i];a.id in e&&n.push(a)}return n}}function Kd(r,t,e){const n=r.createShader(t);return r.shaderSource(n,e),r.compileShader(n),n}const ZS=37297;let jS=0;function $S(r,t){const e=r.split(`
`),n=[],i=Math.max(t-6,0),s=Math.min(t+6,e.length);for(let a=i;a<s;a++){const o=a+1;n.push(`${o===t?">":" "} ${o}: ${e[a]}`)}return n.join(`
`)}function KS(r){const t=me.getPrimaries(me.workingColorSpace),e=me.getPrimaries(r);let n;switch(t===e?n="":t===Sc&&e===vc?n="LinearDisplayP3ToLinearSRGB":t===vc&&e===Sc&&(n="LinearSRGBToLinearDisplayP3"),r){case Vi:case Gc:return[n,"LinearTransferOETF"];case sn:case du:return[n,"sRGBTransferOETF"];default:return console.warn("THREE.WebGLProgram: Unsupported color space:",r),[n,"LinearTransferOETF"]}}function Jd(r,t,e){const n=r.getShaderParameter(t,r.COMPILE_STATUS),i=r.getShaderInfoLog(t).trim();if(n&&i==="")return"";const s=/ERROR: 0:(\d+)/.exec(i);if(s){const a=parseInt(s[1]);return e.toUpperCase()+`

`+i+`

`+$S(r.getShaderSource(t),a)}else return i}function JS(r,t){const e=KS(t);return`vec4 ${r}( vec4 value ) { return ${e[0]}( ${e[1]}( value ) ); }`}function QS(r,t){let e;switch(t){case Kg:e="Linear";break;case Jg:e="Reinhard";break;case Qg:e="OptimizedCineon";break;case t0:e="ACESFilmic";break;case n0:e="AgX";break;case e0:e="Custom";break;default:console.warn("THREE.WebGLProgram: Unsupported toneMapping:",t),e="Linear"}return"vec3 "+r+"( vec3 color ) { return "+e+"ToneMapping( color ); }"}function tM(r){return[r.extensionDerivatives||r.envMapCubeUVHeight||r.bumpMap||r.normalMapTangentSpace||r.clearcoatNormalMap||r.flatShading||r.shaderID==="physical"?"#extension GL_OES_standard_derivatives : enable":"",(r.extensionFragDepth||r.logarithmicDepthBuffer)&&r.rendererExtensionFragDepth?"#extension GL_EXT_frag_depth : enable":"",r.extensionDrawBuffers&&r.rendererExtensionDrawBuffers?"#extension GL_EXT_draw_buffers : require":"",(r.extensionShaderTextureLOD||r.envMap||r.transmission)&&r.rendererExtensionShaderTextureLod?"#extension GL_EXT_shader_texture_lod : enable":""].filter(Mr).join(`
`)}function eM(r){return[r.extensionClipCullDistance?"#extension GL_ANGLE_clip_cull_distance : require":""].filter(Mr).join(`
`)}function nM(r){const t=[];for(const e in r){const n=r[e];n!==!1&&t.push("#define "+e+" "+n)}return t.join(`
`)}function iM(r,t){const e={},n=r.getProgramParameter(t,r.ACTIVE_ATTRIBUTES);for(let i=0;i<n;i++){const s=r.getActiveAttrib(t,i),a=s.name;let o=1;s.type===r.FLOAT_MAT2&&(o=2),s.type===r.FLOAT_MAT3&&(o=3),s.type===r.FLOAT_MAT4&&(o=4),e[a]={type:s.type,location:r.getAttribLocation(t,a),locationSize:o}}return e}function Mr(r){return r!==""}function Qd(r,t){const e=t.numSpotLightShadows+t.numSpotLightMaps-t.numSpotLightShadowsWithMaps;return r.replace(/NUM_DIR_LIGHTS/g,t.numDirLights).replace(/NUM_SPOT_LIGHTS/g,t.numSpotLights).replace(/NUM_SPOT_LIGHT_MAPS/g,t.numSpotLightMaps).replace(/NUM_SPOT_LIGHT_COORDS/g,e).replace(/NUM_RECT_AREA_LIGHTS/g,t.numRectAreaLights).replace(/NUM_POINT_LIGHTS/g,t.numPointLights).replace(/NUM_HEMI_LIGHTS/g,t.numHemiLights).replace(/NUM_DIR_LIGHT_SHADOWS/g,t.numDirLightShadows).replace(/NUM_SPOT_LIGHT_SHADOWS_WITH_MAPS/g,t.numSpotLightShadowsWithMaps).replace(/NUM_SPOT_LIGHT_SHADOWS/g,t.numSpotLightShadows).replace(/NUM_POINT_LIGHT_SHADOWS/g,t.numPointLightShadows)}function tf(r,t){return r.replace(/NUM_CLIPPING_PLANES/g,t.numClippingPlanes).replace(/UNION_CLIPPING_PLANES/g,t.numClippingPlanes-t.numClipIntersection)}const sM=/^[ \t]*#include +<([\w\d./]+)>/gm;function Ih(r){return r.replace(sM,oM)}const rM=new Map([["encodings_fragment","colorspace_fragment"],["encodings_pars_fragment","colorspace_pars_fragment"],["output_fragment","opaque_fragment"]]);function oM(r,t){let e=Qt[t];if(e===void 0){const n=rM.get(t);if(n!==void 0)e=Qt[n],console.warn('THREE.WebGLRenderer: Shader chunk "%s" has been deprecated. Use "%s" instead.',t,n);else throw new Error("Can not resolve #include <"+t+">")}return Ih(e)}const aM=/#pragma unroll_loop_start\s+for\s*\(\s*int\s+i\s*=\s*(\d+)\s*;\s*i\s*<\s*(\d+)\s*;\s*i\s*\+\+\s*\)\s*{([\s\S]+?)}\s+#pragma unroll_loop_end/g;function ef(r){return r.replace(aM,cM)}function cM(r,t,e,n){let i="";for(let s=parseInt(t);s<parseInt(e);s++)i+=n.replace(/\[\s*i\s*\]/g,"[ "+s+" ]").replace(/UNROLLED_LOOP_INDEX/g,s);return i}function nf(r){let t="precision "+r.precision+` float;
precision `+r.precision+" int;";return r.precision==="highp"?t+=`
#define HIGH_PRECISION`:r.precision==="mediump"?t+=`
#define MEDIUM_PRECISION`:r.precision==="lowp"&&(t+=`
#define LOW_PRECISION`),t}function lM(r){let t="SHADOWMAP_TYPE_BASIC";return r.shadowMapType===qp?t="SHADOWMAP_TYPE_PCF":r.shadowMapType===bg?t="SHADOWMAP_TYPE_PCF_SOFT":r.shadowMapType===Ni&&(t="SHADOWMAP_TYPE_VSM"),t}function hM(r){let t="ENVMAP_TYPE_CUBE";if(r.envMap)switch(r.envMapMode){case Ir:case Dr:t="ENVMAP_TYPE_CUBE";break;case Oc:t="ENVMAP_TYPE_CUBE_UV";break}return t}function uM(r){let t="ENVMAP_MODE_REFLECTION";return r.envMap&&r.envMapMode===Dr&&(t="ENVMAP_MODE_REFRACTION"),t}function dM(r){let t="ENVMAP_BLENDING_NONE";if(r.envMap)switch(r.combine){case lu:t="ENVMAP_BLENDING_MULTIPLY";break;case jg:t="ENVMAP_BLENDING_MIX";break;case $g:t="ENVMAP_BLENDING_ADD";break}return t}function fM(r){const t=r.envMapCubeUVHeight;if(t===null)return null;const e=Math.log2(t)-2,n=1/t;return{texelWidth:1/(3*Math.max(Math.pow(2,e),112)),texelHeight:n,maxMip:e}}function pM(r,t,e,n){const i=r.getContext(),s=e.defines;let a=e.vertexShader,o=e.fragmentShader;const c=lM(e),l=hM(e),h=uM(e),u=dM(e),f=fM(e),d=e.isWebGL2?"":tM(e),g=eM(e),x=nM(s),m=i.createProgram();let p,_,v=e.glslVersion?"#version "+e.glslVersion+`
`:"";e.isRawShaderMaterial?(p=["#define SHADER_TYPE "+e.shaderType,"#define SHADER_NAME "+e.shaderName,x].filter(Mr).join(`
`),p.length>0&&(p+=`
`),_=[d,"#define SHADER_TYPE "+e.shaderType,"#define SHADER_NAME "+e.shaderName,x].filter(Mr).join(`
`),_.length>0&&(_+=`
`)):(p=[nf(e),"#define SHADER_TYPE "+e.shaderType,"#define SHADER_NAME "+e.shaderName,x,e.extensionClipCullDistance?"#define USE_CLIP_DISTANCE":"",e.batching?"#define USE_BATCHING":"",e.instancing?"#define USE_INSTANCING":"",e.instancingColor?"#define USE_INSTANCING_COLOR":"",e.useFog&&e.fog?"#define USE_FOG":"",e.useFog&&e.fogExp2?"#define FOG_EXP2":"",e.map?"#define USE_MAP":"",e.envMap?"#define USE_ENVMAP":"",e.envMap?"#define "+h:"",e.lightMap?"#define USE_LIGHTMAP":"",e.aoMap?"#define USE_AOMAP":"",e.bumpMap?"#define USE_BUMPMAP":"",e.normalMap?"#define USE_NORMALMAP":"",e.normalMapObjectSpace?"#define USE_NORMALMAP_OBJECTSPACE":"",e.normalMapTangentSpace?"#define USE_NORMALMAP_TANGENTSPACE":"",e.displacementMap?"#define USE_DISPLACEMENTMAP":"",e.emissiveMap?"#define USE_EMISSIVEMAP":"",e.anisotropy?"#define USE_ANISOTROPY":"",e.anisotropyMap?"#define USE_ANISOTROPYMAP":"",e.clearcoatMap?"#define USE_CLEARCOATMAP":"",e.clearcoatRoughnessMap?"#define USE_CLEARCOAT_ROUGHNESSMAP":"",e.clearcoatNormalMap?"#define USE_CLEARCOAT_NORMALMAP":"",e.iridescenceMap?"#define USE_IRIDESCENCEMAP":"",e.iridescenceThicknessMap?"#define USE_IRIDESCENCE_THICKNESSMAP":"",e.specularMap?"#define USE_SPECULARMAP":"",e.specularColorMap?"#define USE_SPECULAR_COLORMAP":"",e.specularIntensityMap?"#define USE_SPECULAR_INTENSITYMAP":"",e.roughnessMap?"#define USE_ROUGHNESSMAP":"",e.metalnessMap?"#define USE_METALNESSMAP":"",e.alphaMap?"#define USE_ALPHAMAP":"",e.alphaHash?"#define USE_ALPHAHASH":"",e.transmission?"#define USE_TRANSMISSION":"",e.transmissionMap?"#define USE_TRANSMISSIONMAP":"",e.thicknessMap?"#define USE_THICKNESSMAP":"",e.sheenColorMap?"#define USE_SHEEN_COLORMAP":"",e.sheenRoughnessMap?"#define USE_SHEEN_ROUGHNESSMAP":"",e.mapUv?"#define MAP_UV "+e.mapUv:"",e.alphaMapUv?"#define ALPHAMAP_UV "+e.alphaMapUv:"",e.lightMapUv?"#define LIGHTMAP_UV "+e.lightMapUv:"",e.aoMapUv?"#define AOMAP_UV "+e.aoMapUv:"",e.emissiveMapUv?"#define EMISSIVEMAP_UV "+e.emissiveMapUv:"",e.bumpMapUv?"#define BUMPMAP_UV "+e.bumpMapUv:"",e.normalMapUv?"#define NORMALMAP_UV "+e.normalMapUv:"",e.displacementMapUv?"#define DISPLACEMENTMAP_UV "+e.displacementMapUv:"",e.metalnessMapUv?"#define METALNESSMAP_UV "+e.metalnessMapUv:"",e.roughnessMapUv?"#define ROUGHNESSMAP_UV "+e.roughnessMapUv:"",e.anisotropyMapUv?"#define ANISOTROPYMAP_UV "+e.anisotropyMapUv:"",e.clearcoatMapUv?"#define CLEARCOATMAP_UV "+e.clearcoatMapUv:"",e.clearcoatNormalMapUv?"#define CLEARCOAT_NORMALMAP_UV "+e.clearcoatNormalMapUv:"",e.clearcoatRoughnessMapUv?"#define CLEARCOAT_ROUGHNESSMAP_UV "+e.clearcoatRoughnessMapUv:"",e.iridescenceMapUv?"#define IRIDESCENCEMAP_UV "+e.iridescenceMapUv:"",e.iridescenceThicknessMapUv?"#define IRIDESCENCE_THICKNESSMAP_UV "+e.iridescenceThicknessMapUv:"",e.sheenColorMapUv?"#define SHEEN_COLORMAP_UV "+e.sheenColorMapUv:"",e.sheenRoughnessMapUv?"#define SHEEN_ROUGHNESSMAP_UV "+e.sheenRoughnessMapUv:"",e.specularMapUv?"#define SPECULARMAP_UV "+e.specularMapUv:"",e.specularColorMapUv?"#define SPECULAR_COLORMAP_UV "+e.specularColorMapUv:"",e.specularIntensityMapUv?"#define SPECULAR_INTENSITYMAP_UV "+e.specularIntensityMapUv:"",e.transmissionMapUv?"#define TRANSMISSIONMAP_UV "+e.transmissionMapUv:"",e.thicknessMapUv?"#define THICKNESSMAP_UV "+e.thicknessMapUv:"",e.vertexTangents&&e.flatShading===!1?"#define USE_TANGENT":"",e.vertexColors?"#define USE_COLOR":"",e.vertexAlphas?"#define USE_COLOR_ALPHA":"",e.vertexUv1s?"#define USE_UV1":"",e.vertexUv2s?"#define USE_UV2":"",e.vertexUv3s?"#define USE_UV3":"",e.pointsUvs?"#define USE_POINTS_UV":"",e.flatShading?"#define FLAT_SHADED":"",e.skinning?"#define USE_SKINNING":"",e.morphTargets?"#define USE_MORPHTARGETS":"",e.morphNormals&&e.flatShading===!1?"#define USE_MORPHNORMALS":"",e.morphColors&&e.isWebGL2?"#define USE_MORPHCOLORS":"",e.morphTargetsCount>0&&e.isWebGL2?"#define MORPHTARGETS_TEXTURE":"",e.morphTargetsCount>0&&e.isWebGL2?"#define MORPHTARGETS_TEXTURE_STRIDE "+e.morphTextureStride:"",e.morphTargetsCount>0&&e.isWebGL2?"#define MORPHTARGETS_COUNT "+e.morphTargetsCount:"",e.doubleSided?"#define DOUBLE_SIDED":"",e.flipSided?"#define FLIP_SIDED":"",e.shadowMapEnabled?"#define USE_SHADOWMAP":"",e.shadowMapEnabled?"#define "+c:"",e.sizeAttenuation?"#define USE_SIZEATTENUATION":"",e.numLightProbes>0?"#define USE_LIGHT_PROBES":"",e.useLegacyLights?"#define LEGACY_LIGHTS":"",e.logarithmicDepthBuffer?"#define USE_LOGDEPTHBUF":"",e.logarithmicDepthBuffer&&e.rendererExtensionFragDepth?"#define USE_LOGDEPTHBUF_EXT":"","uniform mat4 modelMatrix;","uniform mat4 modelViewMatrix;","uniform mat4 projectionMatrix;","uniform mat4 viewMatrix;","uniform mat3 normalMatrix;","uniform vec3 cameraPosition;","uniform bool isOrthographic;","#ifdef USE_INSTANCING","	attribute mat4 instanceMatrix;","#endif","#ifdef USE_INSTANCING_COLOR","	attribute vec3 instanceColor;","#endif","attribute vec3 position;","attribute vec3 normal;","attribute vec2 uv;","#ifdef USE_UV1","	attribute vec2 uv1;","#endif","#ifdef USE_UV2","	attribute vec2 uv2;","#endif","#ifdef USE_UV3","	attribute vec2 uv3;","#endif","#ifdef USE_TANGENT","	attribute vec4 tangent;","#endif","#if defined( USE_COLOR_ALPHA )","	attribute vec4 color;","#elif defined( USE_COLOR )","	attribute vec3 color;","#endif","#if ( defined( USE_MORPHTARGETS ) && ! defined( MORPHTARGETS_TEXTURE ) )","	attribute vec3 morphTarget0;","	attribute vec3 morphTarget1;","	attribute vec3 morphTarget2;","	attribute vec3 morphTarget3;","	#ifdef USE_MORPHNORMALS","		attribute vec3 morphNormal0;","		attribute vec3 morphNormal1;","		attribute vec3 morphNormal2;","		attribute vec3 morphNormal3;","	#else","		attribute vec3 morphTarget4;","		attribute vec3 morphTarget5;","		attribute vec3 morphTarget6;","		attribute vec3 morphTarget7;","	#endif","#endif","#ifdef USE_SKINNING","	attribute vec4 skinIndex;","	attribute vec4 skinWeight;","#endif",`
`].filter(Mr).join(`
`),_=[d,nf(e),"#define SHADER_TYPE "+e.shaderType,"#define SHADER_NAME "+e.shaderName,x,e.useFog&&e.fog?"#define USE_FOG":"",e.useFog&&e.fogExp2?"#define FOG_EXP2":"",e.map?"#define USE_MAP":"",e.matcap?"#define USE_MATCAP":"",e.envMap?"#define USE_ENVMAP":"",e.envMap?"#define "+l:"",e.envMap?"#define "+h:"",e.envMap?"#define "+u:"",f?"#define CUBEUV_TEXEL_WIDTH "+f.texelWidth:"",f?"#define CUBEUV_TEXEL_HEIGHT "+f.texelHeight:"",f?"#define CUBEUV_MAX_MIP "+f.maxMip+".0":"",e.lightMap?"#define USE_LIGHTMAP":"",e.aoMap?"#define USE_AOMAP":"",e.bumpMap?"#define USE_BUMPMAP":"",e.normalMap?"#define USE_NORMALMAP":"",e.normalMapObjectSpace?"#define USE_NORMALMAP_OBJECTSPACE":"",e.normalMapTangentSpace?"#define USE_NORMALMAP_TANGENTSPACE":"",e.emissiveMap?"#define USE_EMISSIVEMAP":"",e.anisotropy?"#define USE_ANISOTROPY":"",e.anisotropyMap?"#define USE_ANISOTROPYMAP":"",e.clearcoat?"#define USE_CLEARCOAT":"",e.clearcoatMap?"#define USE_CLEARCOATMAP":"",e.clearcoatRoughnessMap?"#define USE_CLEARCOAT_ROUGHNESSMAP":"",e.clearcoatNormalMap?"#define USE_CLEARCOAT_NORMALMAP":"",e.iridescence?"#define USE_IRIDESCENCE":"",e.iridescenceMap?"#define USE_IRIDESCENCEMAP":"",e.iridescenceThicknessMap?"#define USE_IRIDESCENCE_THICKNESSMAP":"",e.specularMap?"#define USE_SPECULARMAP":"",e.specularColorMap?"#define USE_SPECULAR_COLORMAP":"",e.specularIntensityMap?"#define USE_SPECULAR_INTENSITYMAP":"",e.roughnessMap?"#define USE_ROUGHNESSMAP":"",e.metalnessMap?"#define USE_METALNESSMAP":"",e.alphaMap?"#define USE_ALPHAMAP":"",e.alphaTest?"#define USE_ALPHATEST":"",e.alphaHash?"#define USE_ALPHAHASH":"",e.sheen?"#define USE_SHEEN":"",e.sheenColorMap?"#define USE_SHEEN_COLORMAP":"",e.sheenRoughnessMap?"#define USE_SHEEN_ROUGHNESSMAP":"",e.transmission?"#define USE_TRANSMISSION":"",e.transmissionMap?"#define USE_TRANSMISSIONMAP":"",e.thicknessMap?"#define USE_THICKNESSMAP":"",e.vertexTangents&&e.flatShading===!1?"#define USE_TANGENT":"",e.vertexColors||e.instancingColor?"#define USE_COLOR":"",e.vertexAlphas?"#define USE_COLOR_ALPHA":"",e.vertexUv1s?"#define USE_UV1":"",e.vertexUv2s?"#define USE_UV2":"",e.vertexUv3s?"#define USE_UV3":"",e.pointsUvs?"#define USE_POINTS_UV":"",e.gradientMap?"#define USE_GRADIENTMAP":"",e.flatShading?"#define FLAT_SHADED":"",e.doubleSided?"#define DOUBLE_SIDED":"",e.flipSided?"#define FLIP_SIDED":"",e.shadowMapEnabled?"#define USE_SHADOWMAP":"",e.shadowMapEnabled?"#define "+c:"",e.premultipliedAlpha?"#define PREMULTIPLIED_ALPHA":"",e.numLightProbes>0?"#define USE_LIGHT_PROBES":"",e.useLegacyLights?"#define LEGACY_LIGHTS":"",e.decodeVideoTexture?"#define DECODE_VIDEO_TEXTURE":"",e.logarithmicDepthBuffer?"#define USE_LOGDEPTHBUF":"",e.logarithmicDepthBuffer&&e.rendererExtensionFragDepth?"#define USE_LOGDEPTHBUF_EXT":"","uniform mat4 viewMatrix;","uniform vec3 cameraPosition;","uniform bool isOrthographic;",e.toneMapping!==as?"#define TONE_MAPPING":"",e.toneMapping!==as?Qt.tonemapping_pars_fragment:"",e.toneMapping!==as?QS("toneMapping",e.toneMapping):"",e.dithering?"#define DITHERING":"",e.opaque?"#define OPAQUE":"",Qt.colorspace_pars_fragment,JS("linearToOutputTexel",e.outputColorSpace),e.useDepthPacking?"#define DEPTH_PACKING "+e.depthPacking:"",`
`].filter(Mr).join(`
`)),a=Ih(a),a=Qd(a,e),a=tf(a,e),o=Ih(o),o=Qd(o,e),o=tf(o,e),a=ef(a),o=ef(o),e.isWebGL2&&e.isRawShaderMaterial!==!0&&(v=`#version 300 es
`,p=[g,"precision mediump sampler2DArray;","#define attribute in","#define varying out","#define texture2D texture"].join(`
`)+`
`+p,_=["precision mediump sampler2DArray;","#define varying in",e.glslVersion===Sd?"":"layout(location = 0) out highp vec4 pc_fragColor;",e.glslVersion===Sd?"":"#define gl_FragColor pc_fragColor","#define gl_FragDepthEXT gl_FragDepth","#define texture2D texture","#define textureCube texture","#define texture2DProj textureProj","#define texture2DLodEXT textureLod","#define texture2DProjLodEXT textureProjLod","#define textureCubeLodEXT textureLod","#define texture2DGradEXT textureGrad","#define texture2DProjGradEXT textureProjGrad","#define textureCubeGradEXT textureGrad"].join(`
`)+`
`+_);const S=v+p+a,y=v+_+o,M=Kd(i,i.VERTEX_SHADER,S),w=Kd(i,i.FRAGMENT_SHADER,y);i.attachShader(m,M),i.attachShader(m,w),e.index0AttributeName!==void 0?i.bindAttribLocation(m,0,e.index0AttributeName):e.morphTargets===!0&&i.bindAttribLocation(m,0,"position"),i.linkProgram(m);function T(R){if(r.debug.checkShaderErrors){const N=i.getProgramInfoLog(m).trim(),L=i.getShaderInfoLog(M).trim(),F=i.getShaderInfoLog(w).trim();let z=!0,q=!0;if(i.getProgramParameter(m,i.LINK_STATUS)===!1)if(z=!1,typeof r.debug.onShaderError=="function")r.debug.onShaderError(i,m,M,w);else{const O=Jd(i,M,"vertex"),Y=Jd(i,w,"fragment");console.error("THREE.WebGLProgram: Shader Error "+i.getError()+" - VALIDATE_STATUS "+i.getProgramParameter(m,i.VALIDATE_STATUS)+`

Program Info Log: `+N+`
`+O+`
`+Y)}else N!==""?console.warn("THREE.WebGLProgram: Program Info Log:",N):(L===""||F==="")&&(q=!1);q&&(R.diagnostics={runnable:z,programLog:N,vertexShader:{log:L,prefix:p},fragmentShader:{log:F,prefix:_}})}i.deleteShader(M),i.deleteShader(w),E=new ac(i,m),A=iM(i,m)}let E;this.getUniforms=function(){return E===void 0&&T(this),E};let A;this.getAttributes=function(){return A===void 0&&T(this),A};let D=e.rendererExtensionParallelShaderCompile===!1;return this.isReady=function(){return D===!1&&(D=i.getProgramParameter(m,ZS)),D},this.destroy=function(){n.releaseStatesOfProgram(this),i.deleteProgram(m),this.program=void 0},this.type=e.shaderType,this.name=e.shaderName,this.id=jS++,this.cacheKey=t,this.usedTimes=1,this.program=m,this.vertexShader=M,this.fragmentShader=w,this}let mM=0;class gM{constructor(){this.shaderCache=new Map,this.materialCache=new Map}update(t){const e=t.vertexShader,n=t.fragmentShader,i=this._getShaderStage(e),s=this._getShaderStage(n),a=this._getShaderCacheForMaterial(t);return a.has(i)===!1&&(a.add(i),i.usedTimes++),a.has(s)===!1&&(a.add(s),s.usedTimes++),this}remove(t){const e=this.materialCache.get(t);for(const n of e)n.usedTimes--,n.usedTimes===0&&this.shaderCache.delete(n.code);return this.materialCache.delete(t),this}getVertexShaderID(t){return this._getShaderStage(t.vertexShader).id}getFragmentShaderID(t){return this._getShaderStage(t.fragmentShader).id}dispose(){this.shaderCache.clear(),this.materialCache.clear()}_getShaderCacheForMaterial(t){const e=this.materialCache;let n=e.get(t);return n===void 0&&(n=new Set,e.set(t,n)),n}_getShaderStage(t){const e=this.shaderCache;let n=e.get(t);return n===void 0&&(n=new xM(t),e.set(t,n)),n}}class xM{constructor(t){this.id=mM++,this.code=t,this.usedTimes=0}}function _M(r,t,e,n,i,s,a){const o=new lm,c=new gM,l=[],h=i.isWebGL2,u=i.logarithmicDepthBuffer,f=i.vertexTextures;let d=i.precision;const g={MeshDepthMaterial:"depth",MeshDistanceMaterial:"distanceRGBA",MeshNormalMaterial:"normal",MeshBasicMaterial:"basic",MeshLambertMaterial:"lambert",MeshPhongMaterial:"phong",MeshToonMaterial:"toon",MeshStandardMaterial:"physical",MeshPhysicalMaterial:"physical",MeshMatcapMaterial:"matcap",LineBasicMaterial:"basic",LineDashedMaterial:"dashed",PointsMaterial:"points",ShadowMaterial:"shadow",SpriteMaterial:"sprite"};function x(E){return E===0?"uv":`uv${E}`}function m(E,A,D,R,N){const L=R.fog,F=N.geometry,z=E.isMeshStandardMaterial?R.environment:null,q=(E.isMeshStandardMaterial?e:t).get(E.envMap||z),O=q&&q.mapping===Oc?q.image.height:null,Y=g[E.type];E.precision!==null&&(d=i.getMaxPrecision(E.precision),d!==E.precision&&console.warn("THREE.WebGLProgram.getParameters:",E.precision,"not supported, using",d,"instead."));const K=F.morphAttributes.position||F.morphAttributes.normal||F.morphAttributes.color,J=K!==void 0?K.length:0;let it=0;F.morphAttributes.position!==void 0&&(it=1),F.morphAttributes.normal!==void 0&&(it=2),F.morphAttributes.color!==void 0&&(it=3);let X,j,nt,ot;if(Y){const Ae=gi[Y];X=Ae.vertexShader,j=Ae.fragmentShader}else X=E.vertexShader,j=E.fragmentShader,c.update(E),nt=c.getVertexShaderID(E),ot=c.getFragmentShaderID(E);const ut=r.getRenderTarget(),rt=N.isInstancedMesh===!0,ht=N.isBatchedMesh===!0,dt=!!E.map,bt=!!E.matcap,V=!!q,re=!!E.aoMap,Tt=!!E.lightMap,It=!!E.bumpMap,vt=!!E.normalMap,Jt=!!E.displacementMap,Rt=!!E.emissiveMap,I=!!E.metalnessMap,C=!!E.roughnessMap,H=E.anisotropy>0,Q=E.clearcoat>0,$=E.iridescence>0,tt=E.sheen>0,Et=E.transmission>0,at=H&&!!E.anisotropyMap,xt=Q&&!!E.clearcoatMap,Pt=Q&&!!E.clearcoatNormalMap,Ht=Q&&!!E.clearcoatRoughnessMap,et=$&&!!E.iridescenceMap,ae=$&&!!E.iridescenceThicknessMap,qt=tt&&!!E.sheenColorMap,Gt=tt&&!!E.sheenRoughnessMap,wt=!!E.specularMap,gt=!!E.specularColorMap,Dt=!!E.specularIntensityMap,$t=Et&&!!E.transmissionMap,de=Et&&!!E.thicknessMap,Xt=!!E.gradientMap,ct=!!E.alphaMap,U=E.alphaTest>0,ft=!!E.alphaHash,pt=!!E.extensions,zt=!!F.attributes.uv1,Nt=!!F.attributes.uv2,ue=!!F.attributes.uv3;let ce=as;return E.toneMapped&&(ut===null||ut.isXRRenderTarget===!0)&&(ce=r.toneMapping),{isWebGL2:h,shaderID:Y,shaderType:E.type,shaderName:E.name,vertexShader:X,fragmentShader:j,defines:E.defines,customVertexShaderID:nt,customFragmentShaderID:ot,isRawShaderMaterial:E.isRawShaderMaterial===!0,glslVersion:E.glslVersion,precision:d,batching:ht,instancing:rt,instancingColor:rt&&N.instanceColor!==null,supportsVertexTextures:f,outputColorSpace:ut===null?r.outputColorSpace:ut.isXRRenderTarget===!0?ut.texture.colorSpace:Vi,map:dt,matcap:bt,envMap:V,envMapMode:V&&q.mapping,envMapCubeUVHeight:O,aoMap:re,lightMap:Tt,bumpMap:It,normalMap:vt,displacementMap:f&&Jt,emissiveMap:Rt,normalMapObjectSpace:vt&&E.normalMapType===p0,normalMapTangentSpace:vt&&E.normalMapType===uu,metalnessMap:I,roughnessMap:C,anisotropy:H,anisotropyMap:at,clearcoat:Q,clearcoatMap:xt,clearcoatNormalMap:Pt,clearcoatRoughnessMap:Ht,iridescence:$,iridescenceMap:et,iridescenceThicknessMap:ae,sheen:tt,sheenColorMap:qt,sheenRoughnessMap:Gt,specularMap:wt,specularColorMap:gt,specularIntensityMap:Dt,transmission:Et,transmissionMap:$t,thicknessMap:de,gradientMap:Xt,opaque:E.transparent===!1&&E.blending===Tr,alphaMap:ct,alphaTest:U,alphaHash:ft,combine:E.combine,mapUv:dt&&x(E.map.channel),aoMapUv:re&&x(E.aoMap.channel),lightMapUv:Tt&&x(E.lightMap.channel),bumpMapUv:It&&x(E.bumpMap.channel),normalMapUv:vt&&x(E.normalMap.channel),displacementMapUv:Jt&&x(E.displacementMap.channel),emissiveMapUv:Rt&&x(E.emissiveMap.channel),metalnessMapUv:I&&x(E.metalnessMap.channel),roughnessMapUv:C&&x(E.roughnessMap.channel),anisotropyMapUv:at&&x(E.anisotropyMap.channel),clearcoatMapUv:xt&&x(E.clearcoatMap.channel),clearcoatNormalMapUv:Pt&&x(E.clearcoatNormalMap.channel),clearcoatRoughnessMapUv:Ht&&x(E.clearcoatRoughnessMap.channel),iridescenceMapUv:et&&x(E.iridescenceMap.channel),iridescenceThicknessMapUv:ae&&x(E.iridescenceThicknessMap.channel),sheenColorMapUv:qt&&x(E.sheenColorMap.channel),sheenRoughnessMapUv:Gt&&x(E.sheenRoughnessMap.channel),specularMapUv:wt&&x(E.specularMap.channel),specularColorMapUv:gt&&x(E.specularColorMap.channel),specularIntensityMapUv:Dt&&x(E.specularIntensityMap.channel),transmissionMapUv:$t&&x(E.transmissionMap.channel),thicknessMapUv:de&&x(E.thicknessMap.channel),alphaMapUv:ct&&x(E.alphaMap.channel),vertexTangents:!!F.attributes.tangent&&(vt||H),vertexColors:E.vertexColors,vertexAlphas:E.vertexColors===!0&&!!F.attributes.color&&F.attributes.color.itemSize===4,vertexUv1s:zt,vertexUv2s:Nt,vertexUv3s:ue,pointsUvs:N.isPoints===!0&&!!F.attributes.uv&&(dt||ct),fog:!!L,useFog:E.fog===!0,fogExp2:L&&L.isFogExp2,flatShading:E.flatShading===!0,sizeAttenuation:E.sizeAttenuation===!0,logarithmicDepthBuffer:u,skinning:N.isSkinnedMesh===!0,morphTargets:F.morphAttributes.position!==void 0,morphNormals:F.morphAttributes.normal!==void 0,morphColors:F.morphAttributes.color!==void 0,morphTargetsCount:J,morphTextureStride:it,numDirLights:A.directional.length,numPointLights:A.point.length,numSpotLights:A.spot.length,numSpotLightMaps:A.spotLightMap.length,numRectAreaLights:A.rectArea.length,numHemiLights:A.hemi.length,numDirLightShadows:A.directionalShadowMap.length,numPointLightShadows:A.pointShadowMap.length,numSpotLightShadows:A.spotShadowMap.length,numSpotLightShadowsWithMaps:A.numSpotLightShadowsWithMaps,numLightProbes:A.numLightProbes,numClippingPlanes:a.numPlanes,numClipIntersection:a.numIntersection,dithering:E.dithering,shadowMapEnabled:r.shadowMap.enabled&&D.length>0,shadowMapType:r.shadowMap.type,toneMapping:ce,useLegacyLights:r._useLegacyLights,decodeVideoTexture:dt&&E.map.isVideoTexture===!0&&me.getTransfer(E.map.colorSpace)===Se,premultipliedAlpha:E.premultipliedAlpha,doubleSided:E.side===Ke,flipSided:E.side===dn,useDepthPacking:E.depthPacking>=0,depthPacking:E.depthPacking||0,index0AttributeName:E.index0AttributeName,extensionDerivatives:pt&&E.extensions.derivatives===!0,extensionFragDepth:pt&&E.extensions.fragDepth===!0,extensionDrawBuffers:pt&&E.extensions.drawBuffers===!0,extensionShaderTextureLOD:pt&&E.extensions.shaderTextureLOD===!0,extensionClipCullDistance:pt&&E.extensions.clipCullDistance&&n.has("WEBGL_clip_cull_distance"),rendererExtensionFragDepth:h||n.has("EXT_frag_depth"),rendererExtensionDrawBuffers:h||n.has("WEBGL_draw_buffers"),rendererExtensionShaderTextureLod:h||n.has("EXT_shader_texture_lod"),rendererExtensionParallelShaderCompile:n.has("KHR_parallel_shader_compile"),customProgramCacheKey:E.customProgramCacheKey()}}function p(E){const A=[];if(E.shaderID?A.push(E.shaderID):(A.push(E.customVertexShaderID),A.push(E.customFragmentShaderID)),E.defines!==void 0)for(const D in E.defines)A.push(D),A.push(E.defines[D]);return E.isRawShaderMaterial===!1&&(_(A,E),v(A,E),A.push(r.outputColorSpace)),A.push(E.customProgramCacheKey),A.join()}function _(E,A){E.push(A.precision),E.push(A.outputColorSpace),E.push(A.envMapMode),E.push(A.envMapCubeUVHeight),E.push(A.mapUv),E.push(A.alphaMapUv),E.push(A.lightMapUv),E.push(A.aoMapUv),E.push(A.bumpMapUv),E.push(A.normalMapUv),E.push(A.displacementMapUv),E.push(A.emissiveMapUv),E.push(A.metalnessMapUv),E.push(A.roughnessMapUv),E.push(A.anisotropyMapUv),E.push(A.clearcoatMapUv),E.push(A.clearcoatNormalMapUv),E.push(A.clearcoatRoughnessMapUv),E.push(A.iridescenceMapUv),E.push(A.iridescenceThicknessMapUv),E.push(A.sheenColorMapUv),E.push(A.sheenRoughnessMapUv),E.push(A.specularMapUv),E.push(A.specularColorMapUv),E.push(A.specularIntensityMapUv),E.push(A.transmissionMapUv),E.push(A.thicknessMapUv),E.push(A.combine),E.push(A.fogExp2),E.push(A.sizeAttenuation),E.push(A.morphTargetsCount),E.push(A.morphAttributeCount),E.push(A.numDirLights),E.push(A.numPointLights),E.push(A.numSpotLights),E.push(A.numSpotLightMaps),E.push(A.numHemiLights),E.push(A.numRectAreaLights),E.push(A.numDirLightShadows),E.push(A.numPointLightShadows),E.push(A.numSpotLightShadows),E.push(A.numSpotLightShadowsWithMaps),E.push(A.numLightProbes),E.push(A.shadowMapType),E.push(A.toneMapping),E.push(A.numClippingPlanes),E.push(A.numClipIntersection),E.push(A.depthPacking)}function v(E,A){o.disableAll(),A.isWebGL2&&o.enable(0),A.supportsVertexTextures&&o.enable(1),A.instancing&&o.enable(2),A.instancingColor&&o.enable(3),A.matcap&&o.enable(4),A.envMap&&o.enable(5),A.normalMapObjectSpace&&o.enable(6),A.normalMapTangentSpace&&o.enable(7),A.clearcoat&&o.enable(8),A.iridescence&&o.enable(9),A.alphaTest&&o.enable(10),A.vertexColors&&o.enable(11),A.vertexAlphas&&o.enable(12),A.vertexUv1s&&o.enable(13),A.vertexUv2s&&o.enable(14),A.vertexUv3s&&o.enable(15),A.vertexTangents&&o.enable(16),A.anisotropy&&o.enable(17),A.alphaHash&&o.enable(18),A.batching&&o.enable(19),E.push(o.mask),o.disableAll(),A.fog&&o.enable(0),A.useFog&&o.enable(1),A.flatShading&&o.enable(2),A.logarithmicDepthBuffer&&o.enable(3),A.skinning&&o.enable(4),A.morphTargets&&o.enable(5),A.morphNormals&&o.enable(6),A.morphColors&&o.enable(7),A.premultipliedAlpha&&o.enable(8),A.shadowMapEnabled&&o.enable(9),A.useLegacyLights&&o.enable(10),A.doubleSided&&o.enable(11),A.flipSided&&o.enable(12),A.useDepthPacking&&o.enable(13),A.dithering&&o.enable(14),A.transmission&&o.enable(15),A.sheen&&o.enable(16),A.opaque&&o.enable(17),A.pointsUvs&&o.enable(18),A.decodeVideoTexture&&o.enable(19),E.push(o.mask)}function S(E){const A=g[E.type];let D;if(A){const R=gi[A];D=tx.clone(R.uniforms)}else D=E.uniforms;return D}function y(E,A){let D;for(let R=0,N=l.length;R<N;R++){const L=l[R];if(L.cacheKey===A){D=L,++D.usedTimes;break}}return D===void 0&&(D=new pM(r,A,E,s),l.push(D)),D}function M(E){if(--E.usedTimes===0){const A=l.indexOf(E);l[A]=l[l.length-1],l.pop(),E.destroy()}}function w(E){c.remove(E)}function T(){c.dispose()}return{getParameters:m,getProgramCacheKey:p,getUniforms:S,acquireProgram:y,releaseProgram:M,releaseShaderCache:w,programs:l,dispose:T}}function vM(){let r=new WeakMap;function t(s){let a=r.get(s);return a===void 0&&(a={},r.set(s,a)),a}function e(s){r.delete(s)}function n(s,a,o){r.get(s)[a]=o}function i(){r=new WeakMap}return{get:t,remove:e,update:n,dispose:i}}function SM(r,t){return r.groupOrder!==t.groupOrder?r.groupOrder-t.groupOrder:r.renderOrder!==t.renderOrder?r.renderOrder-t.renderOrder:r.material.id!==t.material.id?r.material.id-t.material.id:r.z!==t.z?r.z-t.z:r.id-t.id}function sf(r,t){return r.groupOrder!==t.groupOrder?r.groupOrder-t.groupOrder:r.renderOrder!==t.renderOrder?r.renderOrder-t.renderOrder:r.z!==t.z?t.z-r.z:r.id-t.id}function rf(){const r=[];let t=0;const e=[],n=[],i=[];function s(){t=0,e.length=0,n.length=0,i.length=0}function a(u,f,d,g,x,m){let p=r[t];return p===void 0?(p={id:u.id,object:u,geometry:f,material:d,groupOrder:g,renderOrder:u.renderOrder,z:x,group:m},r[t]=p):(p.id=u.id,p.object=u,p.geometry=f,p.material=d,p.groupOrder=g,p.renderOrder=u.renderOrder,p.z=x,p.group=m),t++,p}function o(u,f,d,g,x,m){const p=a(u,f,d,g,x,m);d.transmission>0?n.push(p):d.transparent===!0?i.push(p):e.push(p)}function c(u,f,d,g,x,m){const p=a(u,f,d,g,x,m);d.transmission>0?n.unshift(p):d.transparent===!0?i.unshift(p):e.unshift(p)}function l(u,f){e.length>1&&e.sort(u||SM),n.length>1&&n.sort(f||sf),i.length>1&&i.sort(f||sf)}function h(){for(let u=t,f=r.length;u<f;u++){const d=r[u];if(d.id===null)break;d.id=null,d.object=null,d.geometry=null,d.material=null,d.group=null}}return{opaque:e,transmissive:n,transparent:i,init:s,push:o,unshift:c,finish:h,sort:l}}function MM(){let r=new WeakMap;function t(n,i){const s=r.get(n);let a;return s===void 0?(a=new rf,r.set(n,[a])):i>=s.length?(a=new rf,s.push(a)):a=s[i],a}function e(){r=new WeakMap}return{get:t,dispose:e}}function yM(){const r={};return{get:function(t){if(r[t.id]!==void 0)return r[t.id];let e;switch(t.type){case"DirectionalLight":e={direction:new b,color:new jt};break;case"SpotLight":e={position:new b,direction:new b,color:new jt,distance:0,coneCos:0,penumbraCos:0,decay:0};break;case"PointLight":e={position:new b,color:new jt,distance:0,decay:0};break;case"HemisphereLight":e={direction:new b,skyColor:new jt,groundColor:new jt};break;case"RectAreaLight":e={color:new jt,position:new b,halfWidth:new b,halfHeight:new b};break}return r[t.id]=e,e}}}function EM(){const r={};return{get:function(t){if(r[t.id]!==void 0)return r[t.id];let e;switch(t.type){case"DirectionalLight":e={shadowBias:0,shadowNormalBias:0,shadowRadius:1,shadowMapSize:new St};break;case"SpotLight":e={shadowBias:0,shadowNormalBias:0,shadowRadius:1,shadowMapSize:new St};break;case"PointLight":e={shadowBias:0,shadowNormalBias:0,shadowRadius:1,shadowMapSize:new St,shadowCameraNear:1,shadowCameraFar:1e3};break}return r[t.id]=e,e}}}let wM=0;function AM(r,t){return(t.castShadow?2:0)-(r.castShadow?2:0)+(t.map?1:0)-(r.map?1:0)}function TM(r,t){const e=new yM,n=EM(),i={version:0,hash:{directionalLength:-1,pointLength:-1,spotLength:-1,rectAreaLength:-1,hemiLength:-1,numDirectionalShadows:-1,numPointShadows:-1,numSpotShadows:-1,numSpotMaps:-1,numLightProbes:-1},ambient:[0,0,0],probe:[],directional:[],directionalShadow:[],directionalShadowMap:[],directionalShadowMatrix:[],spot:[],spotLightMap:[],spotShadow:[],spotShadowMap:[],spotLightMatrix:[],rectArea:[],rectAreaLTC1:null,rectAreaLTC2:null,point:[],pointShadow:[],pointShadowMap:[],pointShadowMatrix:[],hemi:[],numSpotLightShadowsWithMaps:0,numLightProbes:0};for(let h=0;h<9;h++)i.probe.push(new b);const s=new b,a=new se,o=new se;function c(h,u){let f=0,d=0,g=0;for(let R=0;R<9;R++)i.probe[R].set(0,0,0);let x=0,m=0,p=0,_=0,v=0,S=0,y=0,M=0,w=0,T=0,E=0;h.sort(AM);const A=u===!0?Math.PI:1;for(let R=0,N=h.length;R<N;R++){const L=h[R],F=L.color,z=L.intensity,q=L.distance,O=L.shadow&&L.shadow.map?L.shadow.map.texture:null;if(L.isAmbientLight)f+=F.r*z*A,d+=F.g*z*A,g+=F.b*z*A;else if(L.isLightProbe){for(let Y=0;Y<9;Y++)i.probe[Y].addScaledVector(L.sh.coefficients[Y],z);E++}else if(L.isDirectionalLight){const Y=e.get(L);if(Y.color.copy(L.color).multiplyScalar(L.intensity*A),L.castShadow){const K=L.shadow,J=n.get(L);J.shadowBias=K.bias,J.shadowNormalBias=K.normalBias,J.shadowRadius=K.radius,J.shadowMapSize=K.mapSize,i.directionalShadow[x]=J,i.directionalShadowMap[x]=O,i.directionalShadowMatrix[x]=L.shadow.matrix,S++}i.directional[x]=Y,x++}else if(L.isSpotLight){const Y=e.get(L);Y.position.setFromMatrixPosition(L.matrixWorld),Y.color.copy(F).multiplyScalar(z*A),Y.distance=q,Y.coneCos=Math.cos(L.angle),Y.penumbraCos=Math.cos(L.angle*(1-L.penumbra)),Y.decay=L.decay,i.spot[p]=Y;const K=L.shadow;if(L.map&&(i.spotLightMap[w]=L.map,w++,K.updateMatrices(L),L.castShadow&&T++),i.spotLightMatrix[p]=K.matrix,L.castShadow){const J=n.get(L);J.shadowBias=K.bias,J.shadowNormalBias=K.normalBias,J.shadowRadius=K.radius,J.shadowMapSize=K.mapSize,i.spotShadow[p]=J,i.spotShadowMap[p]=O,M++}p++}else if(L.isRectAreaLight){const Y=e.get(L);Y.color.copy(F).multiplyScalar(z),Y.halfWidth.set(L.width*.5,0,0),Y.halfHeight.set(0,L.height*.5,0),i.rectArea[_]=Y,_++}else if(L.isPointLight){const Y=e.get(L);if(Y.color.copy(L.color).multiplyScalar(L.intensity*A),Y.distance=L.distance,Y.decay=L.decay,L.castShadow){const K=L.shadow,J=n.get(L);J.shadowBias=K.bias,J.shadowNormalBias=K.normalBias,J.shadowRadius=K.radius,J.shadowMapSize=K.mapSize,J.shadowCameraNear=K.camera.near,J.shadowCameraFar=K.camera.far,i.pointShadow[m]=J,i.pointShadowMap[m]=O,i.pointShadowMatrix[m]=L.shadow.matrix,y++}i.point[m]=Y,m++}else if(L.isHemisphereLight){const Y=e.get(L);Y.skyColor.copy(L.color).multiplyScalar(z*A),Y.groundColor.copy(L.groundColor).multiplyScalar(z*A),i.hemi[v]=Y,v++}}_>0&&(t.isWebGL2?r.has("OES_texture_float_linear")===!0?(i.rectAreaLTC1=mt.LTC_FLOAT_1,i.rectAreaLTC2=mt.LTC_FLOAT_2):(i.rectAreaLTC1=mt.LTC_HALF_1,i.rectAreaLTC2=mt.LTC_HALF_2):r.has("OES_texture_float_linear")===!0?(i.rectAreaLTC1=mt.LTC_FLOAT_1,i.rectAreaLTC2=mt.LTC_FLOAT_2):r.has("OES_texture_half_float_linear")===!0?(i.rectAreaLTC1=mt.LTC_HALF_1,i.rectAreaLTC2=mt.LTC_HALF_2):console.error("THREE.WebGLRenderer: Unable to use RectAreaLight. Missing WebGL extensions.")),i.ambient[0]=f,i.ambient[1]=d,i.ambient[2]=g;const D=i.hash;(D.directionalLength!==x||D.pointLength!==m||D.spotLength!==p||D.rectAreaLength!==_||D.hemiLength!==v||D.numDirectionalShadows!==S||D.numPointShadows!==y||D.numSpotShadows!==M||D.numSpotMaps!==w||D.numLightProbes!==E)&&(i.directional.length=x,i.spot.length=p,i.rectArea.length=_,i.point.length=m,i.hemi.length=v,i.directionalShadow.length=S,i.directionalShadowMap.length=S,i.pointShadow.length=y,i.pointShadowMap.length=y,i.spotShadow.length=M,i.spotShadowMap.length=M,i.directionalShadowMatrix.length=S,i.pointShadowMatrix.length=y,i.spotLightMatrix.length=M+w-T,i.spotLightMap.length=w,i.numSpotLightShadowsWithMaps=T,i.numLightProbes=E,D.directionalLength=x,D.pointLength=m,D.spotLength=p,D.rectAreaLength=_,D.hemiLength=v,D.numDirectionalShadows=S,D.numPointShadows=y,D.numSpotShadows=M,D.numSpotMaps=w,D.numLightProbes=E,i.version=wM++)}function l(h,u){let f=0,d=0,g=0,x=0,m=0;const p=u.matrixWorldInverse;for(let _=0,v=h.length;_<v;_++){const S=h[_];if(S.isDirectionalLight){const y=i.directional[f];y.direction.setFromMatrixPosition(S.matrixWorld),s.setFromMatrixPosition(S.target.matrixWorld),y.direction.sub(s),y.direction.transformDirection(p),f++}else if(S.isSpotLight){const y=i.spot[g];y.position.setFromMatrixPosition(S.matrixWorld),y.position.applyMatrix4(p),y.direction.setFromMatrixPosition(S.matrixWorld),s.setFromMatrixPosition(S.target.matrixWorld),y.direction.sub(s),y.direction.transformDirection(p),g++}else if(S.isRectAreaLight){const y=i.rectArea[x];y.position.setFromMatrixPosition(S.matrixWorld),y.position.applyMatrix4(p),o.identity(),a.copy(S.matrixWorld),a.premultiply(p),o.extractRotation(a),y.halfWidth.set(S.width*.5,0,0),y.halfHeight.set(0,S.height*.5,0),y.halfWidth.applyMatrix4(o),y.halfHeight.applyMatrix4(o),x++}else if(S.isPointLight){const y=i.point[d];y.position.setFromMatrixPosition(S.matrixWorld),y.position.applyMatrix4(p),d++}else if(S.isHemisphereLight){const y=i.hemi[m];y.direction.setFromMatrixPosition(S.matrixWorld),y.direction.transformDirection(p),m++}}}return{setup:c,setupView:l,state:i}}function of(r,t){const e=new TM(r,t),n=[],i=[];function s(){n.length=0,i.length=0}function a(u){n.push(u)}function o(u){i.push(u)}function c(u){e.setup(n,u)}function l(u){e.setupView(n,u)}return{init:s,state:{lightsArray:n,shadowsArray:i,lights:e},setupLights:c,setupLightsView:l,pushLight:a,pushShadow:o}}function bM(r,t){let e=new WeakMap;function n(s,a=0){const o=e.get(s);let c;return o===void 0?(c=new of(r,t),e.set(s,[c])):a>=o.length?(c=new of(r,t),o.push(c)):c=o[a],c}function i(){e=new WeakMap}return{get:n,dispose:i}}class CM extends Si{constructor(t){super(),this.isMeshDepthMaterial=!0,this.type="MeshDepthMaterial",this.depthPacking=d0,this.map=null,this.alphaMap=null,this.displacementMap=null,this.displacementScale=1,this.displacementBias=0,this.wireframe=!1,this.wireframeLinewidth=1,this.setValues(t)}copy(t){return super.copy(t),this.depthPacking=t.depthPacking,this.map=t.map,this.alphaMap=t.alphaMap,this.displacementMap=t.displacementMap,this.displacementScale=t.displacementScale,this.displacementBias=t.displacementBias,this.wireframe=t.wireframe,this.wireframeLinewidth=t.wireframeLinewidth,this}}class RM extends Si{constructor(t){super(),this.isMeshDistanceMaterial=!0,this.type="MeshDistanceMaterial",this.map=null,this.alphaMap=null,this.displacementMap=null,this.displacementScale=1,this.displacementBias=0,this.setValues(t)}copy(t){return super.copy(t),this.map=t.map,this.alphaMap=t.alphaMap,this.displacementMap=t.displacementMap,this.displacementScale=t.displacementScale,this.displacementBias=t.displacementBias,this}}const PM=`void main() {
	gl_Position = vec4( position, 1.0 );
}`,LM=`uniform sampler2D shadow_pass;
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
}`;function IM(r,t,e){let n=new mu;const i=new St,s=new St,a=new Je,o=new CM({depthPacking:f0}),c=new RM,l={},h=e.maxTextureSize,u={[oi]:dn,[dn]:oi,[Ke]:Ke},f=new Fn({defines:{VSM_SAMPLES:8},uniforms:{shadow_pass:{value:null},resolution:{value:new St},radius:{value:4}},vertexShader:PM,fragmentShader:LM}),d=f.clone();d.defines.HORIZONTAL_PASS=1;const g=new ye;g.setAttribute("position",new Ne(new Float32Array([-1,-1,.5,3,-1,.5,-1,3,.5]),3));const x=new Zt(g,f),m=this;this.enabled=!1,this.autoUpdate=!0,this.needsUpdate=!1,this.type=qp;let p=this.type;this.render=function(M,w,T){if(m.enabled===!1||m.autoUpdate===!1&&m.needsUpdate===!1||M.length===0)return;const E=r.getRenderTarget(),A=r.getActiveCubeFace(),D=r.getActiveMipmapLevel(),R=r.state;R.setBlending(os),R.buffers.color.setClear(1,1,1,1),R.buffers.depth.setTest(!0),R.setScissorTest(!1);const N=p!==Ni&&this.type===Ni,L=p===Ni&&this.type!==Ni;for(let F=0,z=M.length;F<z;F++){const q=M[F],O=q.shadow;if(O===void 0){console.warn("THREE.WebGLShadowMap:",q,"has no shadow.");continue}if(O.autoUpdate===!1&&O.needsUpdate===!1)continue;i.copy(O.mapSize);const Y=O.getFrameExtents();if(i.multiply(Y),s.copy(O.mapSize),(i.x>h||i.y>h)&&(i.x>h&&(s.x=Math.floor(h/Y.x),i.x=s.x*Y.x,O.mapSize.x=s.x),i.y>h&&(s.y=Math.floor(h/Y.y),i.y=s.y*Y.y,O.mapSize.y=s.y)),O.map===null||N===!0||L===!0){const J=this.type!==Ni?{minFilter:_n,magFilter:_n}:{};O.map!==null&&O.map.dispose(),O.map=new tn(i.x,i.y,J),O.map.texture.name=q.name+".shadowMap",O.camera.updateProjectionMatrix()}r.setRenderTarget(O.map),r.clear();const K=O.getViewportCount();for(let J=0;J<K;J++){const it=O.getViewport(J);a.set(s.x*it.x,s.y*it.y,s.x*it.z,s.y*it.w),R.viewport(a),O.updateMatrices(q,J),n=O.getFrustum(),S(w,T,O.camera,q,this.type)}O.isPointLightShadow!==!0&&this.type===Ni&&_(O,T),O.needsUpdate=!1}p=this.type,m.needsUpdate=!1,r.setRenderTarget(E,A,D)};function _(M,w){const T=t.update(x);f.defines.VSM_SAMPLES!==M.blurSamples&&(f.defines.VSM_SAMPLES=M.blurSamples,d.defines.VSM_SAMPLES=M.blurSamples,f.needsUpdate=!0,d.needsUpdate=!0),M.mapPass===null&&(M.mapPass=new tn(i.x,i.y)),f.uniforms.shadow_pass.value=M.map.texture,f.uniforms.resolution.value=M.mapSize,f.uniforms.radius.value=M.radius,r.setRenderTarget(M.mapPass),r.clear(),r.renderBufferDirect(w,null,T,f,x,null),d.uniforms.shadow_pass.value=M.mapPass.texture,d.uniforms.resolution.value=M.mapSize,d.uniforms.radius.value=M.radius,r.setRenderTarget(M.map),r.clear(),r.renderBufferDirect(w,null,T,d,x,null)}function v(M,w,T,E){let A=null;const D=T.isPointLight===!0?M.customDistanceMaterial:M.customDepthMaterial;if(D!==void 0)A=D;else if(A=T.isPointLight===!0?c:o,r.localClippingEnabled&&w.clipShadows===!0&&Array.isArray(w.clippingPlanes)&&w.clippingPlanes.length!==0||w.displacementMap&&w.displacementScale!==0||w.alphaMap&&w.alphaTest>0||w.map&&w.alphaTest>0){const R=A.uuid,N=w.uuid;let L=l[R];L===void 0&&(L={},l[R]=L);let F=L[N];F===void 0&&(F=A.clone(),L[N]=F,w.addEventListener("dispose",y)),A=F}if(A.visible=w.visible,A.wireframe=w.wireframe,E===Ni?A.side=w.shadowSide!==null?w.shadowSide:w.side:A.side=w.shadowSide!==null?w.shadowSide:u[w.side],A.alphaMap=w.alphaMap,A.alphaTest=w.alphaTest,A.map=w.map,A.clipShadows=w.clipShadows,A.clippingPlanes=w.clippingPlanes,A.clipIntersection=w.clipIntersection,A.displacementMap=w.displacementMap,A.displacementScale=w.displacementScale,A.displacementBias=w.displacementBias,A.wireframeLinewidth=w.wireframeLinewidth,A.linewidth=w.linewidth,T.isPointLight===!0&&A.isMeshDistanceMaterial===!0){const R=r.properties.get(A);R.light=T}return A}function S(M,w,T,E,A){if(M.visible===!1)return;if(M.layers.test(w.layers)&&(M.isMesh||M.isLine||M.isPoints)&&(M.castShadow||M.receiveShadow&&A===Ni)&&(!M.frustumCulled||n.intersectsObject(M))){M.modelViewMatrix.multiplyMatrices(T.matrixWorldInverse,M.matrixWorld);const N=t.update(M),L=M.material;if(Array.isArray(L)){const F=N.groups;for(let z=0,q=F.length;z<q;z++){const O=F[z],Y=L[O.materialIndex];if(Y&&Y.visible){const K=v(M,Y,E,A);M.onBeforeShadow(r,M,w,T,N,K,O),r.renderBufferDirect(T,null,N,K,M,O),M.onAfterShadow(r,M,w,T,N,K,O)}}}else if(L.visible){const F=v(M,L,E,A);M.onBeforeShadow(r,M,w,T,N,F,null),r.renderBufferDirect(T,null,N,F,M,null),M.onAfterShadow(r,M,w,T,N,F,null)}}const R=M.children;for(let N=0,L=R.length;N<L;N++)S(R[N],w,T,E,A)}function y(M){M.target.removeEventListener("dispose",y);for(const T in l){const E=l[T],A=M.target.uuid;A in E&&(E[A].dispose(),delete E[A])}}}function DM(r,t,e){const n=e.isWebGL2;function i(){let U=!1;const ft=new Je;let pt=null;const zt=new Je(0,0,0,0);return{setMask:function(Nt){pt!==Nt&&!U&&(r.colorMask(Nt,Nt,Nt,Nt),pt=Nt)},setLocked:function(Nt){U=Nt},setClear:function(Nt,ue,ce,_e,Ae){Ae===!0&&(Nt*=_e,ue*=_e,ce*=_e),ft.set(Nt,ue,ce,_e),zt.equals(ft)===!1&&(r.clearColor(Nt,ue,ce,_e),zt.copy(ft))},reset:function(){U=!1,pt=null,zt.set(-1,0,0,0)}}}function s(){let U=!1,ft=null,pt=null,zt=null;return{setTest:function(Nt){Nt?ht(r.DEPTH_TEST):dt(r.DEPTH_TEST)},setMask:function(Nt){ft!==Nt&&!U&&(r.depthMask(Nt),ft=Nt)},setFunc:function(Nt){if(pt!==Nt){switch(Nt){case kg:r.depthFunc(r.NEVER);break;case Hg:r.depthFunc(r.ALWAYS);break;case Wg:r.depthFunc(r.LESS);break;case xc:r.depthFunc(r.LEQUAL);break;case Xg:r.depthFunc(r.EQUAL);break;case Yg:r.depthFunc(r.GEQUAL);break;case qg:r.depthFunc(r.GREATER);break;case Zg:r.depthFunc(r.NOTEQUAL);break;default:r.depthFunc(r.LEQUAL)}pt=Nt}},setLocked:function(Nt){U=Nt},setClear:function(Nt){zt!==Nt&&(r.clearDepth(Nt),zt=Nt)},reset:function(){U=!1,ft=null,pt=null,zt=null}}}function a(){let U=!1,ft=null,pt=null,zt=null,Nt=null,ue=null,ce=null,_e=null,Ae=null;return{setTest:function(oe){U||(oe?ht(r.STENCIL_TEST):dt(r.STENCIL_TEST))},setMask:function(oe){ft!==oe&&!U&&(r.stencilMask(oe),ft=oe)},setFunc:function(oe,Pe,qe){(pt!==oe||zt!==Pe||Nt!==qe)&&(r.stencilFunc(oe,Pe,qe),pt=oe,zt=Pe,Nt=qe)},setOp:function(oe,Pe,qe){(ue!==oe||ce!==Pe||_e!==qe)&&(r.stencilOp(oe,Pe,qe),ue=oe,ce=Pe,_e=qe)},setLocked:function(oe){U=oe},setClear:function(oe){Ae!==oe&&(r.clearStencil(oe),Ae=oe)},reset:function(){U=!1,ft=null,pt=null,zt=null,Nt=null,ue=null,ce=null,_e=null,Ae=null}}}const o=new i,c=new s,l=new a,h=new WeakMap,u=new WeakMap;let f={},d={},g=new WeakMap,x=[],m=null,p=!1,_=null,v=null,S=null,y=null,M=null,w=null,T=null,E=new jt(0,0,0),A=0,D=!1,R=null,N=null,L=null,F=null,z=null;const q=r.getParameter(r.MAX_COMBINED_TEXTURE_IMAGE_UNITS);let O=!1,Y=0;const K=r.getParameter(r.VERSION);K.indexOf("WebGL")!==-1?(Y=parseFloat(/^WebGL (\d)/.exec(K)[1]),O=Y>=1):K.indexOf("OpenGL ES")!==-1&&(Y=parseFloat(/^OpenGL ES (\d)/.exec(K)[1]),O=Y>=2);let J=null,it={};const X=r.getParameter(r.SCISSOR_BOX),j=r.getParameter(r.VIEWPORT),nt=new Je().fromArray(X),ot=new Je().fromArray(j);function ut(U,ft,pt,zt){const Nt=new Uint8Array(4),ue=r.createTexture();r.bindTexture(U,ue),r.texParameteri(U,r.TEXTURE_MIN_FILTER,r.NEAREST),r.texParameteri(U,r.TEXTURE_MAG_FILTER,r.NEAREST);for(let ce=0;ce<pt;ce++)n&&(U===r.TEXTURE_3D||U===r.TEXTURE_2D_ARRAY)?r.texImage3D(ft,0,r.RGBA,1,1,zt,0,r.RGBA,r.UNSIGNED_BYTE,Nt):r.texImage2D(ft+ce,0,r.RGBA,1,1,0,r.RGBA,r.UNSIGNED_BYTE,Nt);return ue}const rt={};rt[r.TEXTURE_2D]=ut(r.TEXTURE_2D,r.TEXTURE_2D,1),rt[r.TEXTURE_CUBE_MAP]=ut(r.TEXTURE_CUBE_MAP,r.TEXTURE_CUBE_MAP_POSITIVE_X,6),n&&(rt[r.TEXTURE_2D_ARRAY]=ut(r.TEXTURE_2D_ARRAY,r.TEXTURE_2D_ARRAY,1,1),rt[r.TEXTURE_3D]=ut(r.TEXTURE_3D,r.TEXTURE_3D,1,1)),o.setClear(0,0,0,1),c.setClear(1),l.setClear(0),ht(r.DEPTH_TEST),c.setFunc(xc),Rt(!1),I(Vu),ht(r.CULL_FACE),vt(os);function ht(U){f[U]!==!0&&(r.enable(U),f[U]=!0)}function dt(U){f[U]!==!1&&(r.disable(U),f[U]=!1)}function bt(U,ft){return d[U]!==ft?(r.bindFramebuffer(U,ft),d[U]=ft,n&&(U===r.DRAW_FRAMEBUFFER&&(d[r.FRAMEBUFFER]=ft),U===r.FRAMEBUFFER&&(d[r.DRAW_FRAMEBUFFER]=ft)),!0):!1}function V(U,ft){let pt=x,zt=!1;if(U)if(pt=g.get(ft),pt===void 0&&(pt=[],g.set(ft,pt)),U.isWebGLMultipleRenderTargets){const Nt=U.texture;if(pt.length!==Nt.length||pt[0]!==r.COLOR_ATTACHMENT0){for(let ue=0,ce=Nt.length;ue<ce;ue++)pt[ue]=r.COLOR_ATTACHMENT0+ue;pt.length=Nt.length,zt=!0}}else pt[0]!==r.COLOR_ATTACHMENT0&&(pt[0]=r.COLOR_ATTACHMENT0,zt=!0);else pt[0]!==r.BACK&&(pt[0]=r.BACK,zt=!0);zt&&(e.isWebGL2?r.drawBuffers(pt):t.get("WEBGL_draw_buffers").drawBuffersWEBGL(pt))}function re(U){return m!==U?(r.useProgram(U),m=U,!0):!1}const Tt={[Bi]:r.FUNC_ADD,[Cg]:r.FUNC_SUBTRACT,[Rg]:r.FUNC_REVERSE_SUBTRACT};if(n)Tt[Wu]=r.MIN,Tt[Xu]=r.MAX;else{const U=t.get("EXT_blend_minmax");U!==null&&(Tt[Wu]=U.MIN_EXT,Tt[Xu]=U.MAX_EXT)}const It={[Pg]:r.ZERO,[Lo]:r.ONE,[Lg]:r.SRC_COLOR,[wh]:r.SRC_ALPHA,[zg]:r.SRC_ALPHA_SATURATE,[Fg]:r.DST_COLOR,[Dg]:r.DST_ALPHA,[Ig]:r.ONE_MINUS_SRC_COLOR,[Ah]:r.ONE_MINUS_SRC_ALPHA,[Ug]:r.ONE_MINUS_DST_COLOR,[Ng]:r.ONE_MINUS_DST_ALPHA,[Bg]:r.CONSTANT_COLOR,[Og]:r.ONE_MINUS_CONSTANT_COLOR,[Gg]:r.CONSTANT_ALPHA,[Vg]:r.ONE_MINUS_CONSTANT_ALPHA};function vt(U,ft,pt,zt,Nt,ue,ce,_e,Ae,oe){if(U===os){p===!0&&(dt(r.BLEND),p=!1);return}if(p===!1&&(ht(r.BLEND),p=!0),U!==Zp){if(U!==_||oe!==D){if((v!==Bi||M!==Bi)&&(r.blendEquation(r.FUNC_ADD),v=Bi,M=Bi),oe)switch(U){case Tr:r.blendFuncSeparate(r.ONE,r.ONE_MINUS_SRC_ALPHA,r.ONE,r.ONE_MINUS_SRC_ALPHA);break;case gc:r.blendFunc(r.ONE,r.ONE);break;case ku:r.blendFuncSeparate(r.ZERO,r.ONE_MINUS_SRC_COLOR,r.ZERO,r.ONE);break;case Hu:r.blendFuncSeparate(r.ZERO,r.SRC_COLOR,r.ZERO,r.SRC_ALPHA);break;default:console.error("THREE.WebGLState: Invalid blending: ",U);break}else switch(U){case Tr:r.blendFuncSeparate(r.SRC_ALPHA,r.ONE_MINUS_SRC_ALPHA,r.ONE,r.ONE_MINUS_SRC_ALPHA);break;case gc:r.blendFunc(r.SRC_ALPHA,r.ONE);break;case ku:r.blendFuncSeparate(r.ZERO,r.ONE_MINUS_SRC_COLOR,r.ZERO,r.ONE);break;case Hu:r.blendFunc(r.ZERO,r.SRC_COLOR);break;default:console.error("THREE.WebGLState: Invalid blending: ",U);break}S=null,y=null,w=null,T=null,E.set(0,0,0),A=0,_=U,D=oe}return}Nt=Nt||ft,ue=ue||pt,ce=ce||zt,(ft!==v||Nt!==M)&&(r.blendEquationSeparate(Tt[ft],Tt[Nt]),v=ft,M=Nt),(pt!==S||zt!==y||ue!==w||ce!==T)&&(r.blendFuncSeparate(It[pt],It[zt],It[ue],It[ce]),S=pt,y=zt,w=ue,T=ce),(_e.equals(E)===!1||Ae!==A)&&(r.blendColor(_e.r,_e.g,_e.b,Ae),E.copy(_e),A=Ae),_=U,D=!1}function Jt(U,ft){U.side===Ke?dt(r.CULL_FACE):ht(r.CULL_FACE);let pt=U.side===dn;ft&&(pt=!pt),Rt(pt),U.blending===Tr&&U.transparent===!1?vt(os):vt(U.blending,U.blendEquation,U.blendSrc,U.blendDst,U.blendEquationAlpha,U.blendSrcAlpha,U.blendDstAlpha,U.blendColor,U.blendAlpha,U.premultipliedAlpha),c.setFunc(U.depthFunc),c.setTest(U.depthTest),c.setMask(U.depthWrite),o.setMask(U.colorWrite);const zt=U.stencilWrite;l.setTest(zt),zt&&(l.setMask(U.stencilWriteMask),l.setFunc(U.stencilFunc,U.stencilRef,U.stencilFuncMask),l.setOp(U.stencilFail,U.stencilZFail,U.stencilZPass)),H(U.polygonOffset,U.polygonOffsetFactor,U.polygonOffsetUnits),U.alphaToCoverage===!0?ht(r.SAMPLE_ALPHA_TO_COVERAGE):dt(r.SAMPLE_ALPHA_TO_COVERAGE)}function Rt(U){R!==U&&(U?r.frontFace(r.CW):r.frontFace(r.CCW),R=U)}function I(U){U!==Ag?(ht(r.CULL_FACE),U!==N&&(U===Vu?r.cullFace(r.BACK):U===Tg?r.cullFace(r.FRONT):r.cullFace(r.FRONT_AND_BACK))):dt(r.CULL_FACE),N=U}function C(U){U!==L&&(O&&r.lineWidth(U),L=U)}function H(U,ft,pt){U?(ht(r.POLYGON_OFFSET_FILL),(F!==ft||z!==pt)&&(r.polygonOffset(ft,pt),F=ft,z=pt)):dt(r.POLYGON_OFFSET_FILL)}function Q(U){U?ht(r.SCISSOR_TEST):dt(r.SCISSOR_TEST)}function $(U){U===void 0&&(U=r.TEXTURE0+q-1),J!==U&&(r.activeTexture(U),J=U)}function tt(U,ft,pt){pt===void 0&&(J===null?pt=r.TEXTURE0+q-1:pt=J);let zt=it[pt];zt===void 0&&(zt={type:void 0,texture:void 0},it[pt]=zt),(zt.type!==U||zt.texture!==ft)&&(J!==pt&&(r.activeTexture(pt),J=pt),r.bindTexture(U,ft||rt[U]),zt.type=U,zt.texture=ft)}function Et(){const U=it[J];U!==void 0&&U.type!==void 0&&(r.bindTexture(U.type,null),U.type=void 0,U.texture=void 0)}function at(){try{r.compressedTexImage2D.apply(r,arguments)}catch(U){console.error("THREE.WebGLState:",U)}}function xt(){try{r.compressedTexImage3D.apply(r,arguments)}catch(U){console.error("THREE.WebGLState:",U)}}function Pt(){try{r.texSubImage2D.apply(r,arguments)}catch(U){console.error("THREE.WebGLState:",U)}}function Ht(){try{r.texSubImage3D.apply(r,arguments)}catch(U){console.error("THREE.WebGLState:",U)}}function et(){try{r.compressedTexSubImage2D.apply(r,arguments)}catch(U){console.error("THREE.WebGLState:",U)}}function ae(){try{r.compressedTexSubImage3D.apply(r,arguments)}catch(U){console.error("THREE.WebGLState:",U)}}function qt(){try{r.texStorage2D.apply(r,arguments)}catch(U){console.error("THREE.WebGLState:",U)}}function Gt(){try{r.texStorage3D.apply(r,arguments)}catch(U){console.error("THREE.WebGLState:",U)}}function wt(){try{r.texImage2D.apply(r,arguments)}catch(U){console.error("THREE.WebGLState:",U)}}function gt(){try{r.texImage3D.apply(r,arguments)}catch(U){console.error("THREE.WebGLState:",U)}}function Dt(U){nt.equals(U)===!1&&(r.scissor(U.x,U.y,U.z,U.w),nt.copy(U))}function $t(U){ot.equals(U)===!1&&(r.viewport(U.x,U.y,U.z,U.w),ot.copy(U))}function de(U,ft){let pt=u.get(ft);pt===void 0&&(pt=new WeakMap,u.set(ft,pt));let zt=pt.get(U);zt===void 0&&(zt=r.getUniformBlockIndex(ft,U.name),pt.set(U,zt))}function Xt(U,ft){const zt=u.get(ft).get(U);h.get(ft)!==zt&&(r.uniformBlockBinding(ft,zt,U.__bindingPointIndex),h.set(ft,zt))}function ct(){r.disable(r.BLEND),r.disable(r.CULL_FACE),r.disable(r.DEPTH_TEST),r.disable(r.POLYGON_OFFSET_FILL),r.disable(r.SCISSOR_TEST),r.disable(r.STENCIL_TEST),r.disable(r.SAMPLE_ALPHA_TO_COVERAGE),r.blendEquation(r.FUNC_ADD),r.blendFunc(r.ONE,r.ZERO),r.blendFuncSeparate(r.ONE,r.ZERO,r.ONE,r.ZERO),r.blendColor(0,0,0,0),r.colorMask(!0,!0,!0,!0),r.clearColor(0,0,0,0),r.depthMask(!0),r.depthFunc(r.LESS),r.clearDepth(1),r.stencilMask(4294967295),r.stencilFunc(r.ALWAYS,0,4294967295),r.stencilOp(r.KEEP,r.KEEP,r.KEEP),r.clearStencil(0),r.cullFace(r.BACK),r.frontFace(r.CCW),r.polygonOffset(0,0),r.activeTexture(r.TEXTURE0),r.bindFramebuffer(r.FRAMEBUFFER,null),n===!0&&(r.bindFramebuffer(r.DRAW_FRAMEBUFFER,null),r.bindFramebuffer(r.READ_FRAMEBUFFER,null)),r.useProgram(null),r.lineWidth(1),r.scissor(0,0,r.canvas.width,r.canvas.height),r.viewport(0,0,r.canvas.width,r.canvas.height),f={},J=null,it={},d={},g=new WeakMap,x=[],m=null,p=!1,_=null,v=null,S=null,y=null,M=null,w=null,T=null,E=new jt(0,0,0),A=0,D=!1,R=null,N=null,L=null,F=null,z=null,nt.set(0,0,r.canvas.width,r.canvas.height),ot.set(0,0,r.canvas.width,r.canvas.height),o.reset(),c.reset(),l.reset()}return{buffers:{color:o,depth:c,stencil:l},enable:ht,disable:dt,bindFramebuffer:bt,drawBuffers:V,useProgram:re,setBlending:vt,setMaterial:Jt,setFlipSided:Rt,setCullFace:I,setLineWidth:C,setPolygonOffset:H,setScissorTest:Q,activeTexture:$,bindTexture:tt,unbindTexture:Et,compressedTexImage2D:at,compressedTexImage3D:xt,texImage2D:wt,texImage3D:gt,updateUBOMapping:de,uniformBlockBinding:Xt,texStorage2D:qt,texStorage3D:Gt,texSubImage2D:Pt,texSubImage3D:Ht,compressedTexSubImage2D:et,compressedTexSubImage3D:ae,scissor:Dt,viewport:$t,reset:ct}}function NM(r,t,e,n,i,s,a){const o=i.isWebGL2,c=t.has("WEBGL_multisampled_render_to_texture")?t.get("WEBGL_multisampled_render_to_texture"):null,l=typeof navigator>"u"?!1:/OculusBrowser/g.test(navigator.userAgent),h=new WeakMap;let u;const f=new WeakMap;let d=!1;try{d=typeof OffscreenCanvas<"u"&&new OffscreenCanvas(1,1).getContext("2d")!==null}catch{}function g(I,C){return d?new OffscreenCanvas(I,C):Ec("canvas")}function x(I,C,H,Q){let $=1;if((I.width>Q||I.height>Q)&&($=Q/Math.max(I.width,I.height)),$<1||C===!0)if(typeof HTMLImageElement<"u"&&I instanceof HTMLImageElement||typeof HTMLCanvasElement<"u"&&I instanceof HTMLCanvasElement||typeof ImageBitmap<"u"&&I instanceof ImageBitmap){const tt=C?yc:Math.floor,Et=tt($*I.width),at=tt($*I.height);u===void 0&&(u=g(Et,at));const xt=H?g(Et,at):u;return xt.width=Et,xt.height=at,xt.getContext("2d").drawImage(I,0,0,Et,at),console.warn("THREE.WebGLRenderer: Texture has been resized from ("+I.width+"x"+I.height+") to ("+Et+"x"+at+")."),xt}else return"data"in I&&console.warn("THREE.WebGLRenderer: Image in DataTexture is too big ("+I.width+"x"+I.height+")."),I;return I}function m(I){return Lh(I.width)&&Lh(I.height)}function p(I){return o?!1:I.wrapS!==si||I.wrapT!==si||I.minFilter!==_n&&I.minFilter!==Vn}function _(I,C){return I.generateMipmaps&&C&&I.minFilter!==_n&&I.minFilter!==Vn}function v(I){r.generateMipmap(I)}function S(I,C,H,Q,$=!1){if(o===!1)return C;if(I!==null){if(r[I]!==void 0)return r[I];console.warn("THREE.WebGLRenderer: Attempt to use non-existing WebGL internal format '"+I+"'")}let tt=C;if(C===r.RED&&(H===r.FLOAT&&(tt=r.R32F),H===r.HALF_FLOAT&&(tt=r.R16F),H===r.UNSIGNED_BYTE&&(tt=r.R8)),C===r.RED_INTEGER&&(H===r.UNSIGNED_BYTE&&(tt=r.R8UI),H===r.UNSIGNED_SHORT&&(tt=r.R16UI),H===r.UNSIGNED_INT&&(tt=r.R32UI),H===r.BYTE&&(tt=r.R8I),H===r.SHORT&&(tt=r.R16I),H===r.INT&&(tt=r.R32I)),C===r.RG&&(H===r.FLOAT&&(tt=r.RG32F),H===r.HALF_FLOAT&&(tt=r.RG16F),H===r.UNSIGNED_BYTE&&(tt=r.RG8)),C===r.RGBA){const Et=$?_c:me.getTransfer(Q);H===r.FLOAT&&(tt=r.RGBA32F),H===r.HALF_FLOAT&&(tt=r.RGBA16F),H===r.UNSIGNED_BYTE&&(tt=Et===Se?r.SRGB8_ALPHA8:r.RGBA8),H===r.UNSIGNED_SHORT_4_4_4_4&&(tt=r.RGBA4),H===r.UNSIGNED_SHORT_5_5_5_1&&(tt=r.RGB5_A1)}return(tt===r.R16F||tt===r.R32F||tt===r.RG16F||tt===r.RG32F||tt===r.RGBA16F||tt===r.RGBA32F)&&t.get("EXT_color_buffer_float"),tt}function y(I,C,H){return _(I,H)===!0||I.isFramebufferTexture&&I.minFilter!==_n&&I.minFilter!==Vn?Math.log2(Math.max(C.width,C.height))+1:I.mipmaps!==void 0&&I.mipmaps.length>0?I.mipmaps.length:I.isCompressedTexture&&Array.isArray(I.image)?C.mipmaps.length:1}function M(I){return I===_n||I===Yu||I===Qc?r.NEAREST:r.LINEAR}function w(I){const C=I.target;C.removeEventListener("dispose",w),E(C),C.isVideoTexture&&h.delete(C)}function T(I){const C=I.target;C.removeEventListener("dispose",T),D(C)}function E(I){const C=n.get(I);if(C.__webglInit===void 0)return;const H=I.source,Q=f.get(H);if(Q){const $=Q[C.__cacheKey];$.usedTimes--,$.usedTimes===0&&A(I),Object.keys(Q).length===0&&f.delete(H)}n.remove(I)}function A(I){const C=n.get(I);r.deleteTexture(C.__webglTexture);const H=I.source,Q=f.get(H);delete Q[C.__cacheKey],a.memory.textures--}function D(I){const C=I.texture,H=n.get(I),Q=n.get(C);if(Q.__webglTexture!==void 0&&(r.deleteTexture(Q.__webglTexture),a.memory.textures--),I.depthTexture&&I.depthTexture.dispose(),I.isWebGLCubeRenderTarget)for(let $=0;$<6;$++){if(Array.isArray(H.__webglFramebuffer[$]))for(let tt=0;tt<H.__webglFramebuffer[$].length;tt++)r.deleteFramebuffer(H.__webglFramebuffer[$][tt]);else r.deleteFramebuffer(H.__webglFramebuffer[$]);H.__webglDepthbuffer&&r.deleteRenderbuffer(H.__webglDepthbuffer[$])}else{if(Array.isArray(H.__webglFramebuffer))for(let $=0;$<H.__webglFramebuffer.length;$++)r.deleteFramebuffer(H.__webglFramebuffer[$]);else r.deleteFramebuffer(H.__webglFramebuffer);if(H.__webglDepthbuffer&&r.deleteRenderbuffer(H.__webglDepthbuffer),H.__webglMultisampledFramebuffer&&r.deleteFramebuffer(H.__webglMultisampledFramebuffer),H.__webglColorRenderbuffer)for(let $=0;$<H.__webglColorRenderbuffer.length;$++)H.__webglColorRenderbuffer[$]&&r.deleteRenderbuffer(H.__webglColorRenderbuffer[$]);H.__webglDepthRenderbuffer&&r.deleteRenderbuffer(H.__webglDepthRenderbuffer)}if(I.isWebGLMultipleRenderTargets)for(let $=0,tt=C.length;$<tt;$++){const Et=n.get(C[$]);Et.__webglTexture&&(r.deleteTexture(Et.__webglTexture),a.memory.textures--),n.remove(C[$])}n.remove(C),n.remove(I)}let R=0;function N(){R=0}function L(){const I=R;return I>=i.maxTextures&&console.warn("THREE.WebGLTextures: Trying to use "+I+" texture units while this GPU supports only "+i.maxTextures),R+=1,I}function F(I){const C=[];return C.push(I.wrapS),C.push(I.wrapT),C.push(I.wrapR||0),C.push(I.magFilter),C.push(I.minFilter),C.push(I.anisotropy),C.push(I.internalFormat),C.push(I.format),C.push(I.type),C.push(I.generateMipmaps),C.push(I.premultiplyAlpha),C.push(I.flipY),C.push(I.unpackAlignment),C.push(I.colorSpace),C.join()}function z(I,C){const H=n.get(I);if(I.isVideoTexture&&Jt(I),I.isRenderTargetTexture===!1&&I.version>0&&H.__version!==I.version){const Q=I.image;if(Q===null)console.warn("THREE.WebGLRenderer: Texture marked for update but no image data found.");else if(Q.complete===!1)console.warn("THREE.WebGLRenderer: Texture marked for update but image is incomplete");else{nt(H,I,C);return}}e.bindTexture(r.TEXTURE_2D,H.__webglTexture,r.TEXTURE0+C)}function q(I,C){const H=n.get(I);if(I.version>0&&H.__version!==I.version){nt(H,I,C);return}e.bindTexture(r.TEXTURE_2D_ARRAY,H.__webglTexture,r.TEXTURE0+C)}function O(I,C){const H=n.get(I);if(I.version>0&&H.__version!==I.version){nt(H,I,C);return}e.bindTexture(r.TEXTURE_3D,H.__webglTexture,r.TEXTURE0+C)}function Y(I,C){const H=n.get(I);if(I.version>0&&H.__version!==I.version){ot(H,I,C);return}e.bindTexture(r.TEXTURE_CUBE_MAP,H.__webglTexture,r.TEXTURE0+C)}const K={[Ch]:r.REPEAT,[si]:r.CLAMP_TO_EDGE,[Rh]:r.MIRRORED_REPEAT},J={[_n]:r.NEAREST,[Yu]:r.NEAREST_MIPMAP_NEAREST,[Qc]:r.NEAREST_MIPMAP_LINEAR,[Vn]:r.LINEAR,[i0]:r.LINEAR_MIPMAP_NEAREST,[$o]:r.LINEAR_MIPMAP_LINEAR},it={[m0]:r.NEVER,[M0]:r.ALWAYS,[g0]:r.LESS,[sm]:r.LEQUAL,[x0]:r.EQUAL,[S0]:r.GEQUAL,[_0]:r.GREATER,[v0]:r.NOTEQUAL};function X(I,C,H){if(H?(r.texParameteri(I,r.TEXTURE_WRAP_S,K[C.wrapS]),r.texParameteri(I,r.TEXTURE_WRAP_T,K[C.wrapT]),(I===r.TEXTURE_3D||I===r.TEXTURE_2D_ARRAY)&&r.texParameteri(I,r.TEXTURE_WRAP_R,K[C.wrapR]),r.texParameteri(I,r.TEXTURE_MAG_FILTER,J[C.magFilter]),r.texParameteri(I,r.TEXTURE_MIN_FILTER,J[C.minFilter])):(r.texParameteri(I,r.TEXTURE_WRAP_S,r.CLAMP_TO_EDGE),r.texParameteri(I,r.TEXTURE_WRAP_T,r.CLAMP_TO_EDGE),(I===r.TEXTURE_3D||I===r.TEXTURE_2D_ARRAY)&&r.texParameteri(I,r.TEXTURE_WRAP_R,r.CLAMP_TO_EDGE),(C.wrapS!==si||C.wrapT!==si)&&console.warn("THREE.WebGLRenderer: Texture is not power of two. Texture.wrapS and Texture.wrapT should be set to THREE.ClampToEdgeWrapping."),r.texParameteri(I,r.TEXTURE_MAG_FILTER,M(C.magFilter)),r.texParameteri(I,r.TEXTURE_MIN_FILTER,M(C.minFilter)),C.minFilter!==_n&&C.minFilter!==Vn&&console.warn("THREE.WebGLRenderer: Texture is not power of two. Texture.minFilter should be set to THREE.NearestFilter or THREE.LinearFilter.")),C.compareFunction&&(r.texParameteri(I,r.TEXTURE_COMPARE_MODE,r.COMPARE_REF_TO_TEXTURE),r.texParameteri(I,r.TEXTURE_COMPARE_FUNC,it[C.compareFunction])),t.has("EXT_texture_filter_anisotropic")===!0){const Q=t.get("EXT_texture_filter_anisotropic");if(C.magFilter===_n||C.minFilter!==Qc&&C.minFilter!==$o||C.type===is&&t.has("OES_texture_float_linear")===!1||o===!1&&C.type===Nr&&t.has("OES_texture_half_float_linear")===!1)return;(C.anisotropy>1||n.get(C).__currentAnisotropy)&&(r.texParameterf(I,Q.TEXTURE_MAX_ANISOTROPY_EXT,Math.min(C.anisotropy,i.getMaxAnisotropy())),n.get(C).__currentAnisotropy=C.anisotropy)}}function j(I,C){let H=!1;I.__webglInit===void 0&&(I.__webglInit=!0,C.addEventListener("dispose",w));const Q=C.source;let $=f.get(Q);$===void 0&&($={},f.set(Q,$));const tt=F(C);if(tt!==I.__cacheKey){$[tt]===void 0&&($[tt]={texture:r.createTexture(),usedTimes:0},a.memory.textures++,H=!0),$[tt].usedTimes++;const Et=$[I.__cacheKey];Et!==void 0&&($[I.__cacheKey].usedTimes--,Et.usedTimes===0&&A(C)),I.__cacheKey=tt,I.__webglTexture=$[tt].texture}return H}function nt(I,C,H){let Q=r.TEXTURE_2D;(C.isDataArrayTexture||C.isCompressedArrayTexture)&&(Q=r.TEXTURE_2D_ARRAY),C.isData3DTexture&&(Q=r.TEXTURE_3D);const $=j(I,C),tt=C.source;e.bindTexture(Q,I.__webglTexture,r.TEXTURE0+H);const Et=n.get(tt);if(tt.version!==Et.__version||$===!0){e.activeTexture(r.TEXTURE0+H);const at=me.getPrimaries(me.workingColorSpace),xt=C.colorSpace===kn?null:me.getPrimaries(C.colorSpace),Pt=C.colorSpace===kn||at===xt?r.NONE:r.BROWSER_DEFAULT_WEBGL;r.pixelStorei(r.UNPACK_FLIP_Y_WEBGL,C.flipY),r.pixelStorei(r.UNPACK_PREMULTIPLY_ALPHA_WEBGL,C.premultiplyAlpha),r.pixelStorei(r.UNPACK_ALIGNMENT,C.unpackAlignment),r.pixelStorei(r.UNPACK_COLORSPACE_CONVERSION_WEBGL,Pt);const Ht=p(C)&&m(C.image)===!1;let et=x(C.image,Ht,!1,i.maxTextureSize);et=Rt(C,et);const ae=m(et)||o,qt=s.convert(C.format,C.colorSpace);let Gt=s.convert(C.type),wt=S(C.internalFormat,qt,Gt,C.colorSpace,C.isVideoTexture);X(Q,C,ae);let gt;const Dt=C.mipmaps,$t=o&&C.isVideoTexture!==!0&&wt!==nm,de=Et.__version===void 0||$===!0,Xt=y(C,et,ae);if(C.isDepthTexture)wt=r.DEPTH_COMPONENT,o?C.type===is?wt=r.DEPTH_COMPONENT32F:C.type===ns?wt=r.DEPTH_COMPONENT24:C.type===Is?wt=r.DEPTH24_STENCIL8:wt=r.DEPTH_COMPONENT16:C.type===is&&console.error("WebGLRenderer: Floating point depth texture requires WebGL2."),C.format===Ds&&wt===r.DEPTH_COMPONENT&&C.type!==hu&&C.type!==ns&&(console.warn("THREE.WebGLRenderer: Use UnsignedShortType or UnsignedIntType for DepthFormat DepthTexture."),C.type=ns,Gt=s.convert(C.type)),C.format===Fr&&wt===r.DEPTH_COMPONENT&&(wt=r.DEPTH_STENCIL,C.type!==Is&&(console.warn("THREE.WebGLRenderer: Use UnsignedInt248Type for DepthStencilFormat DepthTexture."),C.type=Is,Gt=s.convert(C.type))),de&&($t?e.texStorage2D(r.TEXTURE_2D,1,wt,et.width,et.height):e.texImage2D(r.TEXTURE_2D,0,wt,et.width,et.height,0,qt,Gt,null));else if(C.isDataTexture)if(Dt.length>0&&ae){$t&&de&&e.texStorage2D(r.TEXTURE_2D,Xt,wt,Dt[0].width,Dt[0].height);for(let ct=0,U=Dt.length;ct<U;ct++)gt=Dt[ct],$t?e.texSubImage2D(r.TEXTURE_2D,ct,0,0,gt.width,gt.height,qt,Gt,gt.data):e.texImage2D(r.TEXTURE_2D,ct,wt,gt.width,gt.height,0,qt,Gt,gt.data);C.generateMipmaps=!1}else $t?(de&&e.texStorage2D(r.TEXTURE_2D,Xt,wt,et.width,et.height),e.texSubImage2D(r.TEXTURE_2D,0,0,0,et.width,et.height,qt,Gt,et.data)):e.texImage2D(r.TEXTURE_2D,0,wt,et.width,et.height,0,qt,Gt,et.data);else if(C.isCompressedTexture)if(C.isCompressedArrayTexture){$t&&de&&e.texStorage3D(r.TEXTURE_2D_ARRAY,Xt,wt,Dt[0].width,Dt[0].height,et.depth);for(let ct=0,U=Dt.length;ct<U;ct++)gt=Dt[ct],C.format!==ri?qt!==null?$t?e.compressedTexSubImage3D(r.TEXTURE_2D_ARRAY,ct,0,0,0,gt.width,gt.height,et.depth,qt,gt.data,0,0):e.compressedTexImage3D(r.TEXTURE_2D_ARRAY,ct,wt,gt.width,gt.height,et.depth,0,gt.data,0,0):console.warn("THREE.WebGLRenderer: Attempt to load unsupported compressed texture format in .uploadTexture()"):$t?e.texSubImage3D(r.TEXTURE_2D_ARRAY,ct,0,0,0,gt.width,gt.height,et.depth,qt,Gt,gt.data):e.texImage3D(r.TEXTURE_2D_ARRAY,ct,wt,gt.width,gt.height,et.depth,0,qt,Gt,gt.data)}else{$t&&de&&e.texStorage2D(r.TEXTURE_2D,Xt,wt,Dt[0].width,Dt[0].height);for(let ct=0,U=Dt.length;ct<U;ct++)gt=Dt[ct],C.format!==ri?qt!==null?$t?e.compressedTexSubImage2D(r.TEXTURE_2D,ct,0,0,gt.width,gt.height,qt,gt.data):e.compressedTexImage2D(r.TEXTURE_2D,ct,wt,gt.width,gt.height,0,gt.data):console.warn("THREE.WebGLRenderer: Attempt to load unsupported compressed texture format in .uploadTexture()"):$t?e.texSubImage2D(r.TEXTURE_2D,ct,0,0,gt.width,gt.height,qt,Gt,gt.data):e.texImage2D(r.TEXTURE_2D,ct,wt,gt.width,gt.height,0,qt,Gt,gt.data)}else if(C.isDataArrayTexture)$t?(de&&e.texStorage3D(r.TEXTURE_2D_ARRAY,Xt,wt,et.width,et.height,et.depth),e.texSubImage3D(r.TEXTURE_2D_ARRAY,0,0,0,0,et.width,et.height,et.depth,qt,Gt,et.data)):e.texImage3D(r.TEXTURE_2D_ARRAY,0,wt,et.width,et.height,et.depth,0,qt,Gt,et.data);else if(C.isData3DTexture)$t?(de&&e.texStorage3D(r.TEXTURE_3D,Xt,wt,et.width,et.height,et.depth),e.texSubImage3D(r.TEXTURE_3D,0,0,0,0,et.width,et.height,et.depth,qt,Gt,et.data)):e.texImage3D(r.TEXTURE_3D,0,wt,et.width,et.height,et.depth,0,qt,Gt,et.data);else if(C.isFramebufferTexture){if(de)if($t)e.texStorage2D(r.TEXTURE_2D,Xt,wt,et.width,et.height);else{let ct=et.width,U=et.height;for(let ft=0;ft<Xt;ft++)e.texImage2D(r.TEXTURE_2D,ft,wt,ct,U,0,qt,Gt,null),ct>>=1,U>>=1}}else if(Dt.length>0&&ae){$t&&de&&e.texStorage2D(r.TEXTURE_2D,Xt,wt,Dt[0].width,Dt[0].height);for(let ct=0,U=Dt.length;ct<U;ct++)gt=Dt[ct],$t?e.texSubImage2D(r.TEXTURE_2D,ct,0,0,qt,Gt,gt):e.texImage2D(r.TEXTURE_2D,ct,wt,qt,Gt,gt);C.generateMipmaps=!1}else $t?(de&&e.texStorage2D(r.TEXTURE_2D,Xt,wt,et.width,et.height),e.texSubImage2D(r.TEXTURE_2D,0,0,0,qt,Gt,et)):e.texImage2D(r.TEXTURE_2D,0,wt,qt,Gt,et);_(C,ae)&&v(Q),Et.__version=tt.version,C.onUpdate&&C.onUpdate(C)}I.__version=C.version}function ot(I,C,H){if(C.image.length!==6)return;const Q=j(I,C),$=C.source;e.bindTexture(r.TEXTURE_CUBE_MAP,I.__webglTexture,r.TEXTURE0+H);const tt=n.get($);if($.version!==tt.__version||Q===!0){e.activeTexture(r.TEXTURE0+H);const Et=me.getPrimaries(me.workingColorSpace),at=C.colorSpace===kn?null:me.getPrimaries(C.colorSpace),xt=C.colorSpace===kn||Et===at?r.NONE:r.BROWSER_DEFAULT_WEBGL;r.pixelStorei(r.UNPACK_FLIP_Y_WEBGL,C.flipY),r.pixelStorei(r.UNPACK_PREMULTIPLY_ALPHA_WEBGL,C.premultiplyAlpha),r.pixelStorei(r.UNPACK_ALIGNMENT,C.unpackAlignment),r.pixelStorei(r.UNPACK_COLORSPACE_CONVERSION_WEBGL,xt);const Pt=C.isCompressedTexture||C.image[0].isCompressedTexture,Ht=C.image[0]&&C.image[0].isDataTexture,et=[];for(let ct=0;ct<6;ct++)!Pt&&!Ht?et[ct]=x(C.image[ct],!1,!0,i.maxCubemapSize):et[ct]=Ht?C.image[ct].image:C.image[ct],et[ct]=Rt(C,et[ct]);const ae=et[0],qt=m(ae)||o,Gt=s.convert(C.format,C.colorSpace),wt=s.convert(C.type),gt=S(C.internalFormat,Gt,wt,C.colorSpace),Dt=o&&C.isVideoTexture!==!0,$t=tt.__version===void 0||Q===!0;let de=y(C,ae,qt);X(r.TEXTURE_CUBE_MAP,C,qt);let Xt;if(Pt){Dt&&$t&&e.texStorage2D(r.TEXTURE_CUBE_MAP,de,gt,ae.width,ae.height);for(let ct=0;ct<6;ct++){Xt=et[ct].mipmaps;for(let U=0;U<Xt.length;U++){const ft=Xt[U];C.format!==ri?Gt!==null?Dt?e.compressedTexSubImage2D(r.TEXTURE_CUBE_MAP_POSITIVE_X+ct,U,0,0,ft.width,ft.height,Gt,ft.data):e.compressedTexImage2D(r.TEXTURE_CUBE_MAP_POSITIVE_X+ct,U,gt,ft.width,ft.height,0,ft.data):console.warn("THREE.WebGLRenderer: Attempt to load unsupported compressed texture format in .setTextureCube()"):Dt?e.texSubImage2D(r.TEXTURE_CUBE_MAP_POSITIVE_X+ct,U,0,0,ft.width,ft.height,Gt,wt,ft.data):e.texImage2D(r.TEXTURE_CUBE_MAP_POSITIVE_X+ct,U,gt,ft.width,ft.height,0,Gt,wt,ft.data)}}}else{Xt=C.mipmaps,Dt&&$t&&(Xt.length>0&&de++,e.texStorage2D(r.TEXTURE_CUBE_MAP,de,gt,et[0].width,et[0].height));for(let ct=0;ct<6;ct++)if(Ht){Dt?e.texSubImage2D(r.TEXTURE_CUBE_MAP_POSITIVE_X+ct,0,0,0,et[ct].width,et[ct].height,Gt,wt,et[ct].data):e.texImage2D(r.TEXTURE_CUBE_MAP_POSITIVE_X+ct,0,gt,et[ct].width,et[ct].height,0,Gt,wt,et[ct].data);for(let U=0;U<Xt.length;U++){const pt=Xt[U].image[ct].image;Dt?e.texSubImage2D(r.TEXTURE_CUBE_MAP_POSITIVE_X+ct,U+1,0,0,pt.width,pt.height,Gt,wt,pt.data):e.texImage2D(r.TEXTURE_CUBE_MAP_POSITIVE_X+ct,U+1,gt,pt.width,pt.height,0,Gt,wt,pt.data)}}else{Dt?e.texSubImage2D(r.TEXTURE_CUBE_MAP_POSITIVE_X+ct,0,0,0,Gt,wt,et[ct]):e.texImage2D(r.TEXTURE_CUBE_MAP_POSITIVE_X+ct,0,gt,Gt,wt,et[ct]);for(let U=0;U<Xt.length;U++){const ft=Xt[U];Dt?e.texSubImage2D(r.TEXTURE_CUBE_MAP_POSITIVE_X+ct,U+1,0,0,Gt,wt,ft.image[ct]):e.texImage2D(r.TEXTURE_CUBE_MAP_POSITIVE_X+ct,U+1,gt,Gt,wt,ft.image[ct])}}}_(C,qt)&&v(r.TEXTURE_CUBE_MAP),tt.__version=$.version,C.onUpdate&&C.onUpdate(C)}I.__version=C.version}function ut(I,C,H,Q,$,tt){const Et=s.convert(H.format,H.colorSpace),at=s.convert(H.type),xt=S(H.internalFormat,Et,at,H.colorSpace);if(!n.get(C).__hasExternalTextures){const Ht=Math.max(1,C.width>>tt),et=Math.max(1,C.height>>tt);$===r.TEXTURE_3D||$===r.TEXTURE_2D_ARRAY?e.texImage3D($,tt,xt,Ht,et,C.depth,0,Et,at,null):e.texImage2D($,tt,xt,Ht,et,0,Et,at,null)}e.bindFramebuffer(r.FRAMEBUFFER,I),vt(C)?c.framebufferTexture2DMultisampleEXT(r.FRAMEBUFFER,Q,$,n.get(H).__webglTexture,0,It(C)):($===r.TEXTURE_2D||$>=r.TEXTURE_CUBE_MAP_POSITIVE_X&&$<=r.TEXTURE_CUBE_MAP_NEGATIVE_Z)&&r.framebufferTexture2D(r.FRAMEBUFFER,Q,$,n.get(H).__webglTexture,tt),e.bindFramebuffer(r.FRAMEBUFFER,null)}function rt(I,C,H){if(r.bindRenderbuffer(r.RENDERBUFFER,I),C.depthBuffer&&!C.stencilBuffer){let Q=o===!0?r.DEPTH_COMPONENT24:r.DEPTH_COMPONENT16;if(H||vt(C)){const $=C.depthTexture;$&&$.isDepthTexture&&($.type===is?Q=r.DEPTH_COMPONENT32F:$.type===ns&&(Q=r.DEPTH_COMPONENT24));const tt=It(C);vt(C)?c.renderbufferStorageMultisampleEXT(r.RENDERBUFFER,tt,Q,C.width,C.height):r.renderbufferStorageMultisample(r.RENDERBUFFER,tt,Q,C.width,C.height)}else r.renderbufferStorage(r.RENDERBUFFER,Q,C.width,C.height);r.framebufferRenderbuffer(r.FRAMEBUFFER,r.DEPTH_ATTACHMENT,r.RENDERBUFFER,I)}else if(C.depthBuffer&&C.stencilBuffer){const Q=It(C);H&&vt(C)===!1?r.renderbufferStorageMultisample(r.RENDERBUFFER,Q,r.DEPTH24_STENCIL8,C.width,C.height):vt(C)?c.renderbufferStorageMultisampleEXT(r.RENDERBUFFER,Q,r.DEPTH24_STENCIL8,C.width,C.height):r.renderbufferStorage(r.RENDERBUFFER,r.DEPTH_STENCIL,C.width,C.height),r.framebufferRenderbuffer(r.FRAMEBUFFER,r.DEPTH_STENCIL_ATTACHMENT,r.RENDERBUFFER,I)}else{const Q=C.isWebGLMultipleRenderTargets===!0?C.texture:[C.texture];for(let $=0;$<Q.length;$++){const tt=Q[$],Et=s.convert(tt.format,tt.colorSpace),at=s.convert(tt.type),xt=S(tt.internalFormat,Et,at,tt.colorSpace),Pt=It(C);H&&vt(C)===!1?r.renderbufferStorageMultisample(r.RENDERBUFFER,Pt,xt,C.width,C.height):vt(C)?c.renderbufferStorageMultisampleEXT(r.RENDERBUFFER,Pt,xt,C.width,C.height):r.renderbufferStorage(r.RENDERBUFFER,xt,C.width,C.height)}}r.bindRenderbuffer(r.RENDERBUFFER,null)}function ht(I,C){if(C&&C.isWebGLCubeRenderTarget)throw new Error("Depth Texture with cube render targets is not supported");if(e.bindFramebuffer(r.FRAMEBUFFER,I),!(C.depthTexture&&C.depthTexture.isDepthTexture))throw new Error("renderTarget.depthTexture must be an instance of THREE.DepthTexture");(!n.get(C.depthTexture).__webglTexture||C.depthTexture.image.width!==C.width||C.depthTexture.image.height!==C.height)&&(C.depthTexture.image.width=C.width,C.depthTexture.image.height=C.height,C.depthTexture.needsUpdate=!0),z(C.depthTexture,0);const Q=n.get(C.depthTexture).__webglTexture,$=It(C);if(C.depthTexture.format===Ds)vt(C)?c.framebufferTexture2DMultisampleEXT(r.FRAMEBUFFER,r.DEPTH_ATTACHMENT,r.TEXTURE_2D,Q,0,$):r.framebufferTexture2D(r.FRAMEBUFFER,r.DEPTH_ATTACHMENT,r.TEXTURE_2D,Q,0);else if(C.depthTexture.format===Fr)vt(C)?c.framebufferTexture2DMultisampleEXT(r.FRAMEBUFFER,r.DEPTH_STENCIL_ATTACHMENT,r.TEXTURE_2D,Q,0,$):r.framebufferTexture2D(r.FRAMEBUFFER,r.DEPTH_STENCIL_ATTACHMENT,r.TEXTURE_2D,Q,0);else throw new Error("Unknown depthTexture format")}function dt(I){const C=n.get(I),H=I.isWebGLCubeRenderTarget===!0;if(I.depthTexture&&!C.__autoAllocateDepthBuffer){if(H)throw new Error("target.depthTexture not supported in Cube render targets");ht(C.__webglFramebuffer,I)}else if(H){C.__webglDepthbuffer=[];for(let Q=0;Q<6;Q++)e.bindFramebuffer(r.FRAMEBUFFER,C.__webglFramebuffer[Q]),C.__webglDepthbuffer[Q]=r.createRenderbuffer(),rt(C.__webglDepthbuffer[Q],I,!1)}else e.bindFramebuffer(r.FRAMEBUFFER,C.__webglFramebuffer),C.__webglDepthbuffer=r.createRenderbuffer(),rt(C.__webglDepthbuffer,I,!1);e.bindFramebuffer(r.FRAMEBUFFER,null)}function bt(I,C,H){const Q=n.get(I);C!==void 0&&ut(Q.__webglFramebuffer,I,I.texture,r.COLOR_ATTACHMENT0,r.TEXTURE_2D,0),H!==void 0&&dt(I)}function V(I){const C=I.texture,H=n.get(I),Q=n.get(C);I.addEventListener("dispose",T),I.isWebGLMultipleRenderTargets!==!0&&(Q.__webglTexture===void 0&&(Q.__webglTexture=r.createTexture()),Q.__version=C.version,a.memory.textures++);const $=I.isWebGLCubeRenderTarget===!0,tt=I.isWebGLMultipleRenderTargets===!0,Et=m(I)||o;if($){H.__webglFramebuffer=[];for(let at=0;at<6;at++)if(o&&C.mipmaps&&C.mipmaps.length>0){H.__webglFramebuffer[at]=[];for(let xt=0;xt<C.mipmaps.length;xt++)H.__webglFramebuffer[at][xt]=r.createFramebuffer()}else H.__webglFramebuffer[at]=r.createFramebuffer()}else{if(o&&C.mipmaps&&C.mipmaps.length>0){H.__webglFramebuffer=[];for(let at=0;at<C.mipmaps.length;at++)H.__webglFramebuffer[at]=r.createFramebuffer()}else H.__webglFramebuffer=r.createFramebuffer();if(tt)if(i.drawBuffers){const at=I.texture;for(let xt=0,Pt=at.length;xt<Pt;xt++){const Ht=n.get(at[xt]);Ht.__webglTexture===void 0&&(Ht.__webglTexture=r.createTexture(),a.memory.textures++)}}else console.warn("THREE.WebGLRenderer: WebGLMultipleRenderTargets can only be used with WebGL2 or WEBGL_draw_buffers extension.");if(o&&I.samples>0&&vt(I)===!1){const at=tt?C:[C];H.__webglMultisampledFramebuffer=r.createFramebuffer(),H.__webglColorRenderbuffer=[],e.bindFramebuffer(r.FRAMEBUFFER,H.__webglMultisampledFramebuffer);for(let xt=0;xt<at.length;xt++){const Pt=at[xt];H.__webglColorRenderbuffer[xt]=r.createRenderbuffer(),r.bindRenderbuffer(r.RENDERBUFFER,H.__webglColorRenderbuffer[xt]);const Ht=s.convert(Pt.format,Pt.colorSpace),et=s.convert(Pt.type),ae=S(Pt.internalFormat,Ht,et,Pt.colorSpace,I.isXRRenderTarget===!0),qt=It(I);r.renderbufferStorageMultisample(r.RENDERBUFFER,qt,ae,I.width,I.height),r.framebufferRenderbuffer(r.FRAMEBUFFER,r.COLOR_ATTACHMENT0+xt,r.RENDERBUFFER,H.__webglColorRenderbuffer[xt])}r.bindRenderbuffer(r.RENDERBUFFER,null),I.depthBuffer&&(H.__webglDepthRenderbuffer=r.createRenderbuffer(),rt(H.__webglDepthRenderbuffer,I,!0)),e.bindFramebuffer(r.FRAMEBUFFER,null)}}if($){e.bindTexture(r.TEXTURE_CUBE_MAP,Q.__webglTexture),X(r.TEXTURE_CUBE_MAP,C,Et);for(let at=0;at<6;at++)if(o&&C.mipmaps&&C.mipmaps.length>0)for(let xt=0;xt<C.mipmaps.length;xt++)ut(H.__webglFramebuffer[at][xt],I,C,r.COLOR_ATTACHMENT0,r.TEXTURE_CUBE_MAP_POSITIVE_X+at,xt);else ut(H.__webglFramebuffer[at],I,C,r.COLOR_ATTACHMENT0,r.TEXTURE_CUBE_MAP_POSITIVE_X+at,0);_(C,Et)&&v(r.TEXTURE_CUBE_MAP),e.unbindTexture()}else if(tt){const at=I.texture;for(let xt=0,Pt=at.length;xt<Pt;xt++){const Ht=at[xt],et=n.get(Ht);e.bindTexture(r.TEXTURE_2D,et.__webglTexture),X(r.TEXTURE_2D,Ht,Et),ut(H.__webglFramebuffer,I,Ht,r.COLOR_ATTACHMENT0+xt,r.TEXTURE_2D,0),_(Ht,Et)&&v(r.TEXTURE_2D)}e.unbindTexture()}else{let at=r.TEXTURE_2D;if((I.isWebGL3DRenderTarget||I.isWebGLArrayRenderTarget)&&(o?at=I.isWebGL3DRenderTarget?r.TEXTURE_3D:r.TEXTURE_2D_ARRAY:console.error("THREE.WebGLTextures: THREE.Data3DTexture and THREE.DataArrayTexture only supported with WebGL2.")),e.bindTexture(at,Q.__webglTexture),X(at,C,Et),o&&C.mipmaps&&C.mipmaps.length>0)for(let xt=0;xt<C.mipmaps.length;xt++)ut(H.__webglFramebuffer[xt],I,C,r.COLOR_ATTACHMENT0,at,xt);else ut(H.__webglFramebuffer,I,C,r.COLOR_ATTACHMENT0,at,0);_(C,Et)&&v(at),e.unbindTexture()}I.depthBuffer&&dt(I)}function re(I){const C=m(I)||o,H=I.isWebGLMultipleRenderTargets===!0?I.texture:[I.texture];for(let Q=0,$=H.length;Q<$;Q++){const tt=H[Q];if(_(tt,C)){const Et=I.isWebGLCubeRenderTarget?r.TEXTURE_CUBE_MAP:r.TEXTURE_2D,at=n.get(tt).__webglTexture;e.bindTexture(Et,at),v(Et),e.unbindTexture()}}}function Tt(I){if(o&&I.samples>0&&vt(I)===!1){const C=I.isWebGLMultipleRenderTargets?I.texture:[I.texture],H=I.width,Q=I.height;let $=r.COLOR_BUFFER_BIT;const tt=[],Et=I.stencilBuffer?r.DEPTH_STENCIL_ATTACHMENT:r.DEPTH_ATTACHMENT,at=n.get(I),xt=I.isWebGLMultipleRenderTargets===!0;if(xt)for(let Pt=0;Pt<C.length;Pt++)e.bindFramebuffer(r.FRAMEBUFFER,at.__webglMultisampledFramebuffer),r.framebufferRenderbuffer(r.FRAMEBUFFER,r.COLOR_ATTACHMENT0+Pt,r.RENDERBUFFER,null),e.bindFramebuffer(r.FRAMEBUFFER,at.__webglFramebuffer),r.framebufferTexture2D(r.DRAW_FRAMEBUFFER,r.COLOR_ATTACHMENT0+Pt,r.TEXTURE_2D,null,0);e.bindFramebuffer(r.READ_FRAMEBUFFER,at.__webglMultisampledFramebuffer),e.bindFramebuffer(r.DRAW_FRAMEBUFFER,at.__webglFramebuffer);for(let Pt=0;Pt<C.length;Pt++){tt.push(r.COLOR_ATTACHMENT0+Pt),I.depthBuffer&&tt.push(Et);const Ht=at.__ignoreDepthValues!==void 0?at.__ignoreDepthValues:!1;if(Ht===!1&&(I.depthBuffer&&($|=r.DEPTH_BUFFER_BIT),I.stencilBuffer&&($|=r.STENCIL_BUFFER_BIT)),xt&&r.framebufferRenderbuffer(r.READ_FRAMEBUFFER,r.COLOR_ATTACHMENT0,r.RENDERBUFFER,at.__webglColorRenderbuffer[Pt]),Ht===!0&&(r.invalidateFramebuffer(r.READ_FRAMEBUFFER,[Et]),r.invalidateFramebuffer(r.DRAW_FRAMEBUFFER,[Et])),xt){const et=n.get(C[Pt]).__webglTexture;r.framebufferTexture2D(r.DRAW_FRAMEBUFFER,r.COLOR_ATTACHMENT0,r.TEXTURE_2D,et,0)}r.blitFramebuffer(0,0,H,Q,0,0,H,Q,$,r.NEAREST),l&&r.invalidateFramebuffer(r.READ_FRAMEBUFFER,tt)}if(e.bindFramebuffer(r.READ_FRAMEBUFFER,null),e.bindFramebuffer(r.DRAW_FRAMEBUFFER,null),xt)for(let Pt=0;Pt<C.length;Pt++){e.bindFramebuffer(r.FRAMEBUFFER,at.__webglMultisampledFramebuffer),r.framebufferRenderbuffer(r.FRAMEBUFFER,r.COLOR_ATTACHMENT0+Pt,r.RENDERBUFFER,at.__webglColorRenderbuffer[Pt]);const Ht=n.get(C[Pt]).__webglTexture;e.bindFramebuffer(r.FRAMEBUFFER,at.__webglFramebuffer),r.framebufferTexture2D(r.DRAW_FRAMEBUFFER,r.COLOR_ATTACHMENT0+Pt,r.TEXTURE_2D,Ht,0)}e.bindFramebuffer(r.DRAW_FRAMEBUFFER,at.__webglMultisampledFramebuffer)}}function It(I){return Math.min(i.maxSamples,I.samples)}function vt(I){const C=n.get(I);return o&&I.samples>0&&t.has("WEBGL_multisampled_render_to_texture")===!0&&C.__useRenderToTexture!==!1}function Jt(I){const C=a.render.frame;h.get(I)!==C&&(h.set(I,C),I.update())}function Rt(I,C){const H=I.colorSpace,Q=I.format,$=I.type;return I.isCompressedTexture===!0||I.isVideoTexture===!0||I.format===Ph||H!==Vi&&H!==kn&&(me.getTransfer(H)===Se?o===!1?t.has("EXT_sRGB")===!0&&Q===ri?(I.format=Ph,I.minFilter=Vn,I.generateMipmaps=!1):C=om.sRGBToLinear(C):(Q!==ri||$!==cs)&&console.warn("THREE.WebGLTextures: sRGB encoded textures have to use RGBAFormat and UnsignedByteType."):console.error("THREE.WebGLTextures: Unsupported texture color space:",H)),C}this.allocateTextureUnit=L,this.resetTextureUnits=N,this.setTexture2D=z,this.setTexture2DArray=q,this.setTexture3D=O,this.setTextureCube=Y,this.rebindTextures=bt,this.setupRenderTarget=V,this.updateRenderTargetMipmap=re,this.updateMultisampleRenderTarget=Tt,this.setupDepthRenderbuffer=dt,this.setupFrameBufferTexture=ut,this.useMultisampledRTT=vt}function FM(r,t,e){const n=e.isWebGL2;function i(s,a=kn){let o;const c=me.getTransfer(a);if(s===cs)return r.UNSIGNED_BYTE;if(s===Kp)return r.UNSIGNED_SHORT_4_4_4_4;if(s===Jp)return r.UNSIGNED_SHORT_5_5_5_1;if(s===s0)return r.BYTE;if(s===r0)return r.SHORT;if(s===hu)return r.UNSIGNED_SHORT;if(s===$p)return r.INT;if(s===ns)return r.UNSIGNED_INT;if(s===is)return r.FLOAT;if(s===Nr)return n?r.HALF_FLOAT:(o=t.get("OES_texture_half_float"),o!==null?o.HALF_FLOAT_OES:null);if(s===o0)return r.ALPHA;if(s===ri)return r.RGBA;if(s===a0)return r.LUMINANCE;if(s===c0)return r.LUMINANCE_ALPHA;if(s===Ds)return r.DEPTH_COMPONENT;if(s===Fr)return r.DEPTH_STENCIL;if(s===Ph)return o=t.get("EXT_sRGB"),o!==null?o.SRGB_ALPHA_EXT:null;if(s===l0)return r.RED;if(s===Qp)return r.RED_INTEGER;if(s===h0)return r.RG;if(s===tm)return r.RG_INTEGER;if(s===em)return r.RGBA_INTEGER;if(s===tl||s===el||s===nl||s===il)if(c===Se)if(o=t.get("WEBGL_compressed_texture_s3tc_srgb"),o!==null){if(s===tl)return o.COMPRESSED_SRGB_S3TC_DXT1_EXT;if(s===el)return o.COMPRESSED_SRGB_ALPHA_S3TC_DXT1_EXT;if(s===nl)return o.COMPRESSED_SRGB_ALPHA_S3TC_DXT3_EXT;if(s===il)return o.COMPRESSED_SRGB_ALPHA_S3TC_DXT5_EXT}else return null;else if(o=t.get("WEBGL_compressed_texture_s3tc"),o!==null){if(s===tl)return o.COMPRESSED_RGB_S3TC_DXT1_EXT;if(s===el)return o.COMPRESSED_RGBA_S3TC_DXT1_EXT;if(s===nl)return o.COMPRESSED_RGBA_S3TC_DXT3_EXT;if(s===il)return o.COMPRESSED_RGBA_S3TC_DXT5_EXT}else return null;if(s===qu||s===Zu||s===ju||s===$u)if(o=t.get("WEBGL_compressed_texture_pvrtc"),o!==null){if(s===qu)return o.COMPRESSED_RGB_PVRTC_4BPPV1_IMG;if(s===Zu)return o.COMPRESSED_RGB_PVRTC_2BPPV1_IMG;if(s===ju)return o.COMPRESSED_RGBA_PVRTC_4BPPV1_IMG;if(s===$u)return o.COMPRESSED_RGBA_PVRTC_2BPPV1_IMG}else return null;if(s===nm)return o=t.get("WEBGL_compressed_texture_etc1"),o!==null?o.COMPRESSED_RGB_ETC1_WEBGL:null;if(s===Ku||s===Ju)if(o=t.get("WEBGL_compressed_texture_etc"),o!==null){if(s===Ku)return c===Se?o.COMPRESSED_SRGB8_ETC2:o.COMPRESSED_RGB8_ETC2;if(s===Ju)return c===Se?o.COMPRESSED_SRGB8_ALPHA8_ETC2_EAC:o.COMPRESSED_RGBA8_ETC2_EAC}else return null;if(s===Qu||s===td||s===ed||s===nd||s===id||s===sd||s===rd||s===od||s===ad||s===cd||s===ld||s===hd||s===ud||s===dd)if(o=t.get("WEBGL_compressed_texture_astc"),o!==null){if(s===Qu)return c===Se?o.COMPRESSED_SRGB8_ALPHA8_ASTC_4x4_KHR:o.COMPRESSED_RGBA_ASTC_4x4_KHR;if(s===td)return c===Se?o.COMPRESSED_SRGB8_ALPHA8_ASTC_5x4_KHR:o.COMPRESSED_RGBA_ASTC_5x4_KHR;if(s===ed)return c===Se?o.COMPRESSED_SRGB8_ALPHA8_ASTC_5x5_KHR:o.COMPRESSED_RGBA_ASTC_5x5_KHR;if(s===nd)return c===Se?o.COMPRESSED_SRGB8_ALPHA8_ASTC_6x5_KHR:o.COMPRESSED_RGBA_ASTC_6x5_KHR;if(s===id)return c===Se?o.COMPRESSED_SRGB8_ALPHA8_ASTC_6x6_KHR:o.COMPRESSED_RGBA_ASTC_6x6_KHR;if(s===sd)return c===Se?o.COMPRESSED_SRGB8_ALPHA8_ASTC_8x5_KHR:o.COMPRESSED_RGBA_ASTC_8x5_KHR;if(s===rd)return c===Se?o.COMPRESSED_SRGB8_ALPHA8_ASTC_8x6_KHR:o.COMPRESSED_RGBA_ASTC_8x6_KHR;if(s===od)return c===Se?o.COMPRESSED_SRGB8_ALPHA8_ASTC_8x8_KHR:o.COMPRESSED_RGBA_ASTC_8x8_KHR;if(s===ad)return c===Se?o.COMPRESSED_SRGB8_ALPHA8_ASTC_10x5_KHR:o.COMPRESSED_RGBA_ASTC_10x5_KHR;if(s===cd)return c===Se?o.COMPRESSED_SRGB8_ALPHA8_ASTC_10x6_KHR:o.COMPRESSED_RGBA_ASTC_10x6_KHR;if(s===ld)return c===Se?o.COMPRESSED_SRGB8_ALPHA8_ASTC_10x8_KHR:o.COMPRESSED_RGBA_ASTC_10x8_KHR;if(s===hd)return c===Se?o.COMPRESSED_SRGB8_ALPHA8_ASTC_10x10_KHR:o.COMPRESSED_RGBA_ASTC_10x10_KHR;if(s===ud)return c===Se?o.COMPRESSED_SRGB8_ALPHA8_ASTC_12x10_KHR:o.COMPRESSED_RGBA_ASTC_12x10_KHR;if(s===dd)return c===Se?o.COMPRESSED_SRGB8_ALPHA8_ASTC_12x12_KHR:o.COMPRESSED_RGBA_ASTC_12x12_KHR}else return null;if(s===sl||s===fd||s===pd)if(o=t.get("EXT_texture_compression_bptc"),o!==null){if(s===sl)return c===Se?o.COMPRESSED_SRGB_ALPHA_BPTC_UNORM_EXT:o.COMPRESSED_RGBA_BPTC_UNORM_EXT;if(s===fd)return o.COMPRESSED_RGB_BPTC_SIGNED_FLOAT_EXT;if(s===pd)return o.COMPRESSED_RGB_BPTC_UNSIGNED_FLOAT_EXT}else return null;if(s===u0||s===md||s===gd||s===xd)if(o=t.get("EXT_texture_compression_rgtc"),o!==null){if(s===sl)return o.COMPRESSED_RED_RGTC1_EXT;if(s===md)return o.COMPRESSED_SIGNED_RED_RGTC1_EXT;if(s===gd)return o.COMPRESSED_RED_GREEN_RGTC2_EXT;if(s===xd)return o.COMPRESSED_SIGNED_RED_GREEN_RGTC2_EXT}else return null;return s===Is?n?r.UNSIGNED_INT_24_8:(o=t.get("WEBGL_depth_texture"),o!==null?o.UNSIGNED_INT_24_8_WEBGL:null):r[s]!==void 0?r[s]:null}return{convert:i}}class UM extends Ln{constructor(t=[]){super(),this.isArrayCamera=!0,this.cameras=t}}class De extends Qe{constructor(){super(),this.isGroup=!0,this.type="Group"}}const zM={type:"move"};class bl{constructor(){this._targetRay=null,this._grip=null,this._hand=null}getHandSpace(){return this._hand===null&&(this._hand=new De,this._hand.matrixAutoUpdate=!1,this._hand.visible=!1,this._hand.joints={},this._hand.inputState={pinching:!1}),this._hand}getTargetRaySpace(){return this._targetRay===null&&(this._targetRay=new De,this._targetRay.matrixAutoUpdate=!1,this._targetRay.visible=!1,this._targetRay.hasLinearVelocity=!1,this._targetRay.linearVelocity=new b,this._targetRay.hasAngularVelocity=!1,this._targetRay.angularVelocity=new b),this._targetRay}getGripSpace(){return this._grip===null&&(this._grip=new De,this._grip.matrixAutoUpdate=!1,this._grip.visible=!1,this._grip.hasLinearVelocity=!1,this._grip.linearVelocity=new b,this._grip.hasAngularVelocity=!1,this._grip.angularVelocity=new b),this._grip}dispatchEvent(t){return this._targetRay!==null&&this._targetRay.dispatchEvent(t),this._grip!==null&&this._grip.dispatchEvent(t),this._hand!==null&&this._hand.dispatchEvent(t),this}connect(t){if(t&&t.hand){const e=this._hand;if(e)for(const n of t.hand.values())this._getHandJoint(e,n)}return this.dispatchEvent({type:"connected",data:t}),this}disconnect(t){return this.dispatchEvent({type:"disconnected",data:t}),this._targetRay!==null&&(this._targetRay.visible=!1),this._grip!==null&&(this._grip.visible=!1),this._hand!==null&&(this._hand.visible=!1),this}update(t,e,n){let i=null,s=null,a=null;const o=this._targetRay,c=this._grip,l=this._hand;if(t&&e.session.visibilityState!=="visible-blurred"){if(l&&t.hand){a=!0;for(const x of t.hand.values()){const m=e.getJointPose(x,n),p=this._getHandJoint(l,x);m!==null&&(p.matrix.fromArray(m.transform.matrix),p.matrix.decompose(p.position,p.rotation,p.scale),p.matrixWorldNeedsUpdate=!0,p.jointRadius=m.radius),p.visible=m!==null}const h=l.joints["index-finger-tip"],u=l.joints["thumb-tip"],f=h.position.distanceTo(u.position),d=.02,g=.005;l.inputState.pinching&&f>d+g?(l.inputState.pinching=!1,this.dispatchEvent({type:"pinchend",handedness:t.handedness,target:this})):!l.inputState.pinching&&f<=d-g&&(l.inputState.pinching=!0,this.dispatchEvent({type:"pinchstart",handedness:t.handedness,target:this}))}else c!==null&&t.gripSpace&&(s=e.getPose(t.gripSpace,n),s!==null&&(c.matrix.fromArray(s.transform.matrix),c.matrix.decompose(c.position,c.rotation,c.scale),c.matrixWorldNeedsUpdate=!0,s.linearVelocity?(c.hasLinearVelocity=!0,c.linearVelocity.copy(s.linearVelocity)):c.hasLinearVelocity=!1,s.angularVelocity?(c.hasAngularVelocity=!0,c.angularVelocity.copy(s.angularVelocity)):c.hasAngularVelocity=!1));o!==null&&(i=e.getPose(t.targetRaySpace,n),i===null&&s!==null&&(i=s),i!==null&&(o.matrix.fromArray(i.transform.matrix),o.matrix.decompose(o.position,o.rotation,o.scale),o.matrixWorldNeedsUpdate=!0,i.linearVelocity?(o.hasLinearVelocity=!0,o.linearVelocity.copy(i.linearVelocity)):o.hasLinearVelocity=!1,i.angularVelocity?(o.hasAngularVelocity=!0,o.angularVelocity.copy(i.angularVelocity)):o.hasAngularVelocity=!1,this.dispatchEvent(zM)))}return o!==null&&(o.visible=i!==null),c!==null&&(c.visible=s!==null),l!==null&&(l.visible=a!==null),this}_getHandJoint(t,e){if(t.joints[e.jointName]===void 0){const n=new De;n.matrixAutoUpdate=!1,n.visible=!1,t.joints[e.jointName]=n,t.add(n)}return t.joints[e.jointName]}}class BM extends Or{constructor(t,e){super();const n=this;let i=null,s=1,a=null,o="local-floor",c=1,l=null,h=null,u=null,f=null,d=null,g=null;const x=e.getContextAttributes();let m=null,p=null;const _=[],v=[],S=new St;let y=null;const M=new Ln;M.layers.enable(1),M.viewport=new Je;const w=new Ln;w.layers.enable(2),w.viewport=new Je;const T=[M,w],E=new UM;E.layers.enable(1),E.layers.enable(2);let A=null,D=null;this.cameraAutoUpdate=!0,this.enabled=!1,this.isPresenting=!1,this.getController=function(X){let j=_[X];return j===void 0&&(j=new bl,_[X]=j),j.getTargetRaySpace()},this.getControllerGrip=function(X){let j=_[X];return j===void 0&&(j=new bl,_[X]=j),j.getGripSpace()},this.getHand=function(X){let j=_[X];return j===void 0&&(j=new bl,_[X]=j),j.getHandSpace()};function R(X){const j=v.indexOf(X.inputSource);if(j===-1)return;const nt=_[j];nt!==void 0&&(nt.update(X.inputSource,X.frame,l||a),nt.dispatchEvent({type:X.type,data:X.inputSource}))}function N(){i.removeEventListener("select",R),i.removeEventListener("selectstart",R),i.removeEventListener("selectend",R),i.removeEventListener("squeeze",R),i.removeEventListener("squeezestart",R),i.removeEventListener("squeezeend",R),i.removeEventListener("end",N),i.removeEventListener("inputsourceschange",L);for(let X=0;X<_.length;X++){const j=v[X];j!==null&&(v[X]=null,_[X].disconnect(j))}A=null,D=null,t.setRenderTarget(m),d=null,f=null,u=null,i=null,p=null,it.stop(),n.isPresenting=!1,t.setPixelRatio(y),t.setSize(S.width,S.height,!1),n.dispatchEvent({type:"sessionend"})}this.setFramebufferScaleFactor=function(X){s=X,n.isPresenting===!0&&console.warn("THREE.WebXRManager: Cannot change framebuffer scale while presenting.")},this.setReferenceSpaceType=function(X){o=X,n.isPresenting===!0&&console.warn("THREE.WebXRManager: Cannot change reference space type while presenting.")},this.getReferenceSpace=function(){return l||a},this.setReferenceSpace=function(X){l=X},this.getBaseLayer=function(){return f!==null?f:d},this.getBinding=function(){return u},this.getFrame=function(){return g},this.getSession=function(){return i},this.setSession=async function(X){if(i=X,i!==null){if(m=t.getRenderTarget(),i.addEventListener("select",R),i.addEventListener("selectstart",R),i.addEventListener("selectend",R),i.addEventListener("squeeze",R),i.addEventListener("squeezestart",R),i.addEventListener("squeezeend",R),i.addEventListener("end",N),i.addEventListener("inputsourceschange",L),x.xrCompatible!==!0&&await e.makeXRCompatible(),y=t.getPixelRatio(),t.getSize(S),i.renderState.layers===void 0||t.capabilities.isWebGL2===!1){const j={antialias:i.renderState.layers===void 0?x.antialias:!0,alpha:!0,depth:x.depth,stencil:x.stencil,framebufferScaleFactor:s};d=new XRWebGLLayer(i,e,j),i.updateRenderState({baseLayer:d}),t.setPixelRatio(1),t.setSize(d.framebufferWidth,d.framebufferHeight,!1),p=new tn(d.framebufferWidth,d.framebufferHeight,{format:ri,type:cs,colorSpace:t.outputColorSpace,stencilBuffer:x.stencil})}else{let j=null,nt=null,ot=null;x.depth&&(ot=x.stencil?e.DEPTH24_STENCIL8:e.DEPTH_COMPONENT24,j=x.stencil?Fr:Ds,nt=x.stencil?Is:ns);const ut={colorFormat:e.RGBA8,depthFormat:ot,scaleFactor:s};u=new XRWebGLBinding(i,e),f=u.createProjectionLayer(ut),i.updateRenderState({layers:[f]}),t.setPixelRatio(1),t.setSize(f.textureWidth,f.textureHeight,!1),p=new tn(f.textureWidth,f.textureHeight,{format:ri,type:cs,depthTexture:new xm(f.textureWidth,f.textureHeight,nt,void 0,void 0,void 0,void 0,void 0,void 0,j),stencilBuffer:x.stencil,colorSpace:t.outputColorSpace,samples:x.antialias?4:0});const rt=t.properties.get(p);rt.__ignoreDepthValues=f.ignoreDepthValues}p.isXRRenderTarget=!0,this.setFoveation(c),l=null,a=await i.requestReferenceSpace(o),it.setContext(i),it.start(),n.isPresenting=!0,n.dispatchEvent({type:"sessionstart"})}},this.getEnvironmentBlendMode=function(){if(i!==null)return i.environmentBlendMode};function L(X){for(let j=0;j<X.removed.length;j++){const nt=X.removed[j],ot=v.indexOf(nt);ot>=0&&(v[ot]=null,_[ot].disconnect(nt))}for(let j=0;j<X.added.length;j++){const nt=X.added[j];let ot=v.indexOf(nt);if(ot===-1){for(let rt=0;rt<_.length;rt++)if(rt>=v.length){v.push(nt),ot=rt;break}else if(v[rt]===null){v[rt]=nt,ot=rt;break}if(ot===-1)break}const ut=_[ot];ut&&ut.connect(nt)}}const F=new b,z=new b;function q(X,j,nt){F.setFromMatrixPosition(j.matrixWorld),z.setFromMatrixPosition(nt.matrixWorld);const ot=F.distanceTo(z),ut=j.projectionMatrix.elements,rt=nt.projectionMatrix.elements,ht=ut[14]/(ut[10]-1),dt=ut[14]/(ut[10]+1),bt=(ut[9]+1)/ut[5],V=(ut[9]-1)/ut[5],re=(ut[8]-1)/ut[0],Tt=(rt[8]+1)/rt[0],It=ht*re,vt=ht*Tt,Jt=ot/(-re+Tt),Rt=Jt*-re;j.matrixWorld.decompose(X.position,X.quaternion,X.scale),X.translateX(Rt),X.translateZ(Jt),X.matrixWorld.compose(X.position,X.quaternion,X.scale),X.matrixWorldInverse.copy(X.matrixWorld).invert();const I=ht+Jt,C=dt+Jt,H=It-Rt,Q=vt+(ot-Rt),$=bt*dt/C*I,tt=V*dt/C*I;X.projectionMatrix.makePerspective(H,Q,$,tt,I,C),X.projectionMatrixInverse.copy(X.projectionMatrix).invert()}function O(X,j){j===null?X.matrixWorld.copy(X.matrix):X.matrixWorld.multiplyMatrices(j.matrixWorld,X.matrix),X.matrixWorldInverse.copy(X.matrixWorld).invert()}this.updateCamera=function(X){if(i===null)return;E.near=w.near=M.near=X.near,E.far=w.far=M.far=X.far,(A!==E.near||D!==E.far)&&(i.updateRenderState({depthNear:E.near,depthFar:E.far}),A=E.near,D=E.far);const j=X.parent,nt=E.cameras;O(E,j);for(let ot=0;ot<nt.length;ot++)O(nt[ot],j);nt.length===2?q(E,M,w):E.projectionMatrix.copy(M.projectionMatrix),Y(X,E,j)};function Y(X,j,nt){nt===null?X.matrix.copy(j.matrixWorld):(X.matrix.copy(nt.matrixWorld),X.matrix.invert(),X.matrix.multiply(j.matrixWorld)),X.matrix.decompose(X.position,X.quaternion,X.scale),X.updateMatrixWorld(!0),X.projectionMatrix.copy(j.projectionMatrix),X.projectionMatrixInverse.copy(j.projectionMatrixInverse),X.isPerspectiveCamera&&(X.fov=Ko*2*Math.atan(1/X.projectionMatrix.elements[5]),X.zoom=1)}this.getCamera=function(){return E},this.getFoveation=function(){if(!(f===null&&d===null))return c},this.setFoveation=function(X){c=X,f!==null&&(f.fixedFoveation=X),d!==null&&d.fixedFoveation!==void 0&&(d.fixedFoveation=X)};let K=null;function J(X,j){if(h=j.getViewerPose(l||a),g=j,h!==null){const nt=h.views;d!==null&&(t.setRenderTargetFramebuffer(p,d.framebuffer),t.setRenderTarget(p));let ot=!1;nt.length!==E.cameras.length&&(E.cameras.length=0,ot=!0);for(let ut=0;ut<nt.length;ut++){const rt=nt[ut];let ht=null;if(d!==null)ht=d.getViewport(rt);else{const bt=u.getViewSubImage(f,rt);ht=bt.viewport,ut===0&&(t.setRenderTargetTextures(p,bt.colorTexture,f.ignoreDepthValues?void 0:bt.depthStencilTexture),t.setRenderTarget(p))}let dt=T[ut];dt===void 0&&(dt=new Ln,dt.layers.enable(ut),dt.viewport=new Je,T[ut]=dt),dt.matrix.fromArray(rt.transform.matrix),dt.matrix.decompose(dt.position,dt.quaternion,dt.scale),dt.projectionMatrix.fromArray(rt.projectionMatrix),dt.projectionMatrixInverse.copy(dt.projectionMatrix).invert(),dt.viewport.set(ht.x,ht.y,ht.width,ht.height),ut===0&&(E.matrix.copy(dt.matrix),E.matrix.decompose(E.position,E.quaternion,E.scale)),ot===!0&&E.cameras.push(dt)}}for(let nt=0;nt<_.length;nt++){const ot=v[nt],ut=_[nt];ot!==null&&ut!==void 0&&ut.update(ot,j,l||a)}K&&K(X,j),j.detectedPlanes&&n.dispatchEvent({type:"planesdetected",data:j}),g=null}const it=new gm;it.setAnimationLoop(J),this.setAnimationLoop=function(X){K=X},this.dispose=function(){}}}function OM(r,t){function e(m,p){m.matrixAutoUpdate===!0&&m.updateMatrix(),p.value.copy(m.matrix)}function n(m,p){p.color.getRGB(m.fogColor.value,fm(r)),p.isFog?(m.fogNear.value=p.near,m.fogFar.value=p.far):p.isFogExp2&&(m.fogDensity.value=p.density)}function i(m,p,_,v,S){p.isMeshBasicMaterial||p.isMeshLambertMaterial?s(m,p):p.isMeshToonMaterial?(s(m,p),u(m,p)):p.isMeshPhongMaterial?(s(m,p),h(m,p)):p.isMeshStandardMaterial?(s(m,p),f(m,p),p.isMeshPhysicalMaterial&&d(m,p,S)):p.isMeshMatcapMaterial?(s(m,p),g(m,p)):p.isMeshDepthMaterial?s(m,p):p.isMeshDistanceMaterial?(s(m,p),x(m,p)):p.isMeshNormalMaterial?s(m,p):p.isLineBasicMaterial?(a(m,p),p.isLineDashedMaterial&&o(m,p)):p.isPointsMaterial?c(m,p,_,v):p.isSpriteMaterial?l(m,p):p.isShadowMaterial?(m.color.value.copy(p.color),m.opacity.value=p.opacity):p.isShaderMaterial&&(p.uniformsNeedUpdate=!1)}function s(m,p){m.opacity.value=p.opacity,p.color&&m.diffuse.value.copy(p.color),p.emissive&&m.emissive.value.copy(p.emissive).multiplyScalar(p.emissiveIntensity),p.map&&(m.map.value=p.map,e(p.map,m.mapTransform)),p.alphaMap&&(m.alphaMap.value=p.alphaMap,e(p.alphaMap,m.alphaMapTransform)),p.bumpMap&&(m.bumpMap.value=p.bumpMap,e(p.bumpMap,m.bumpMapTransform),m.bumpScale.value=p.bumpScale,p.side===dn&&(m.bumpScale.value*=-1)),p.normalMap&&(m.normalMap.value=p.normalMap,e(p.normalMap,m.normalMapTransform),m.normalScale.value.copy(p.normalScale),p.side===dn&&m.normalScale.value.negate()),p.displacementMap&&(m.displacementMap.value=p.displacementMap,e(p.displacementMap,m.displacementMapTransform),m.displacementScale.value=p.displacementScale,m.displacementBias.value=p.displacementBias),p.emissiveMap&&(m.emissiveMap.value=p.emissiveMap,e(p.emissiveMap,m.emissiveMapTransform)),p.specularMap&&(m.specularMap.value=p.specularMap,e(p.specularMap,m.specularMapTransform)),p.alphaTest>0&&(m.alphaTest.value=p.alphaTest);const _=t.get(p).envMap;if(_&&(m.envMap.value=_,m.flipEnvMap.value=_.isCubeTexture&&_.isRenderTargetTexture===!1?-1:1,m.reflectivity.value=p.reflectivity,m.ior.value=p.ior,m.refractionRatio.value=p.refractionRatio),p.lightMap){m.lightMap.value=p.lightMap;const v=r._useLegacyLights===!0?Math.PI:1;m.lightMapIntensity.value=p.lightMapIntensity*v,e(p.lightMap,m.lightMapTransform)}p.aoMap&&(m.aoMap.value=p.aoMap,m.aoMapIntensity.value=p.aoMapIntensity,e(p.aoMap,m.aoMapTransform))}function a(m,p){m.diffuse.value.copy(p.color),m.opacity.value=p.opacity,p.map&&(m.map.value=p.map,e(p.map,m.mapTransform))}function o(m,p){m.dashSize.value=p.dashSize,m.totalSize.value=p.dashSize+p.gapSize,m.scale.value=p.scale}function c(m,p,_,v){m.diffuse.value.copy(p.color),m.opacity.value=p.opacity,m.size.value=p.size*_,m.scale.value=v*.5,p.map&&(m.map.value=p.map,e(p.map,m.uvTransform)),p.alphaMap&&(m.alphaMap.value=p.alphaMap,e(p.alphaMap,m.alphaMapTransform)),p.alphaTest>0&&(m.alphaTest.value=p.alphaTest)}function l(m,p){m.diffuse.value.copy(p.color),m.opacity.value=p.opacity,m.rotation.value=p.rotation,p.map&&(m.map.value=p.map,e(p.map,m.mapTransform)),p.alphaMap&&(m.alphaMap.value=p.alphaMap,e(p.alphaMap,m.alphaMapTransform)),p.alphaTest>0&&(m.alphaTest.value=p.alphaTest)}function h(m,p){m.specular.value.copy(p.specular),m.shininess.value=Math.max(p.shininess,1e-4)}function u(m,p){p.gradientMap&&(m.gradientMap.value=p.gradientMap)}function f(m,p){m.metalness.value=p.metalness,p.metalnessMap&&(m.metalnessMap.value=p.metalnessMap,e(p.metalnessMap,m.metalnessMapTransform)),m.roughness.value=p.roughness,p.roughnessMap&&(m.roughnessMap.value=p.roughnessMap,e(p.roughnessMap,m.roughnessMapTransform)),t.get(p).envMap&&(m.envMapIntensity.value=p.envMapIntensity)}function d(m,p,_){m.ior.value=p.ior,p.sheen>0&&(m.sheenColor.value.copy(p.sheenColor).multiplyScalar(p.sheen),m.sheenRoughness.value=p.sheenRoughness,p.sheenColorMap&&(m.sheenColorMap.value=p.sheenColorMap,e(p.sheenColorMap,m.sheenColorMapTransform)),p.sheenRoughnessMap&&(m.sheenRoughnessMap.value=p.sheenRoughnessMap,e(p.sheenRoughnessMap,m.sheenRoughnessMapTransform))),p.clearcoat>0&&(m.clearcoat.value=p.clearcoat,m.clearcoatRoughness.value=p.clearcoatRoughness,p.clearcoatMap&&(m.clearcoatMap.value=p.clearcoatMap,e(p.clearcoatMap,m.clearcoatMapTransform)),p.clearcoatRoughnessMap&&(m.clearcoatRoughnessMap.value=p.clearcoatRoughnessMap,e(p.clearcoatRoughnessMap,m.clearcoatRoughnessMapTransform)),p.clearcoatNormalMap&&(m.clearcoatNormalMap.value=p.clearcoatNormalMap,e(p.clearcoatNormalMap,m.clearcoatNormalMapTransform),m.clearcoatNormalScale.value.copy(p.clearcoatNormalScale),p.side===dn&&m.clearcoatNormalScale.value.negate())),p.iridescence>0&&(m.iridescence.value=p.iridescence,m.iridescenceIOR.value=p.iridescenceIOR,m.iridescenceThicknessMinimum.value=p.iridescenceThicknessRange[0],m.iridescenceThicknessMaximum.value=p.iridescenceThicknessRange[1],p.iridescenceMap&&(m.iridescenceMap.value=p.iridescenceMap,e(p.iridescenceMap,m.iridescenceMapTransform)),p.iridescenceThicknessMap&&(m.iridescenceThicknessMap.value=p.iridescenceThicknessMap,e(p.iridescenceThicknessMap,m.iridescenceThicknessMapTransform))),p.transmission>0&&(m.transmission.value=p.transmission,m.transmissionSamplerMap.value=_.texture,m.transmissionSamplerSize.value.set(_.width,_.height),p.transmissionMap&&(m.transmissionMap.value=p.transmissionMap,e(p.transmissionMap,m.transmissionMapTransform)),m.thickness.value=p.thickness,p.thicknessMap&&(m.thicknessMap.value=p.thicknessMap,e(p.thicknessMap,m.thicknessMapTransform)),m.attenuationDistance.value=p.attenuationDistance,m.attenuationColor.value.copy(p.attenuationColor)),p.anisotropy>0&&(m.anisotropyVector.value.set(p.anisotropy*Math.cos(p.anisotropyRotation),p.anisotropy*Math.sin(p.anisotropyRotation)),p.anisotropyMap&&(m.anisotropyMap.value=p.anisotropyMap,e(p.anisotropyMap,m.anisotropyMapTransform))),m.specularIntensity.value=p.specularIntensity,m.specularColor.value.copy(p.specularColor),p.specularColorMap&&(m.specularColorMap.value=p.specularColorMap,e(p.specularColorMap,m.specularColorMapTransform)),p.specularIntensityMap&&(m.specularIntensityMap.value=p.specularIntensityMap,e(p.specularIntensityMap,m.specularIntensityMapTransform))}function g(m,p){p.matcap&&(m.matcap.value=p.matcap)}function x(m,p){const _=t.get(p).light;m.referencePosition.value.setFromMatrixPosition(_.matrixWorld),m.nearDistance.value=_.shadow.camera.near,m.farDistance.value=_.shadow.camera.far}return{refreshFogUniforms:n,refreshMaterialUniforms:i}}function GM(r,t,e,n){let i={},s={},a=[];const o=e.isWebGL2?r.getParameter(r.MAX_UNIFORM_BUFFER_BINDINGS):0;function c(_,v){const S=v.program;n.uniformBlockBinding(_,S)}function l(_,v){let S=i[_.id];S===void 0&&(g(_),S=h(_),i[_.id]=S,_.addEventListener("dispose",m));const y=v.program;n.updateUBOMapping(_,y);const M=t.render.frame;s[_.id]!==M&&(f(_),s[_.id]=M)}function h(_){const v=u();_.__bindingPointIndex=v;const S=r.createBuffer(),y=_.__size,M=_.usage;return r.bindBuffer(r.UNIFORM_BUFFER,S),r.bufferData(r.UNIFORM_BUFFER,y,M),r.bindBuffer(r.UNIFORM_BUFFER,null),r.bindBufferBase(r.UNIFORM_BUFFER,v,S),S}function u(){for(let _=0;_<o;_++)if(a.indexOf(_)===-1)return a.push(_),_;return console.error("THREE.WebGLRenderer: Maximum number of simultaneously usable uniforms groups reached."),0}function f(_){const v=i[_.id],S=_.uniforms,y=_.__cache;r.bindBuffer(r.UNIFORM_BUFFER,v);for(let M=0,w=S.length;M<w;M++){const T=Array.isArray(S[M])?S[M]:[S[M]];for(let E=0,A=T.length;E<A;E++){const D=T[E];if(d(D,M,E,y)===!0){const R=D.__offset,N=Array.isArray(D.value)?D.value:[D.value];let L=0;for(let F=0;F<N.length;F++){const z=N[F],q=x(z);typeof z=="number"||typeof z=="boolean"?(D.__data[0]=z,r.bufferSubData(r.UNIFORM_BUFFER,R+L,D.__data)):z.isMatrix3?(D.__data[0]=z.elements[0],D.__data[1]=z.elements[1],D.__data[2]=z.elements[2],D.__data[3]=0,D.__data[4]=z.elements[3],D.__data[5]=z.elements[4],D.__data[6]=z.elements[5],D.__data[7]=0,D.__data[8]=z.elements[6],D.__data[9]=z.elements[7],D.__data[10]=z.elements[8],D.__data[11]=0):(z.toArray(D.__data,L),L+=q.storage/Float32Array.BYTES_PER_ELEMENT)}r.bufferSubData(r.UNIFORM_BUFFER,R,D.__data)}}}r.bindBuffer(r.UNIFORM_BUFFER,null)}function d(_,v,S,y){const M=_.value,w=v+"_"+S;if(y[w]===void 0)return typeof M=="number"||typeof M=="boolean"?y[w]=M:y[w]=M.clone(),!0;{const T=y[w];if(typeof M=="number"||typeof M=="boolean"){if(T!==M)return y[w]=M,!0}else if(T.equals(M)===!1)return T.copy(M),!0}return!1}function g(_){const v=_.uniforms;let S=0;const y=16;for(let w=0,T=v.length;w<T;w++){const E=Array.isArray(v[w])?v[w]:[v[w]];for(let A=0,D=E.length;A<D;A++){const R=E[A],N=Array.isArray(R.value)?R.value:[R.value];for(let L=0,F=N.length;L<F;L++){const z=N[L],q=x(z),O=S%y;O!==0&&y-O<q.boundary&&(S+=y-O),R.__data=new Float32Array(q.storage/Float32Array.BYTES_PER_ELEMENT),R.__offset=S,S+=q.storage}}}const M=S%y;return M>0&&(S+=y-M),_.__size=S,_.__cache={},this}function x(_){const v={boundary:0,storage:0};return typeof _=="number"||typeof _=="boolean"?(v.boundary=4,v.storage=4):_.isVector2?(v.boundary=8,v.storage=8):_.isVector3||_.isColor?(v.boundary=16,v.storage=12):_.isVector4?(v.boundary=16,v.storage=16):_.isMatrix3?(v.boundary=48,v.storage=48):_.isMatrix4?(v.boundary=64,v.storage=64):_.isTexture?console.warn("THREE.WebGLRenderer: Texture samplers can not be part of an uniforms group."):console.warn("THREE.WebGLRenderer: Unsupported uniform value type.",_),v}function m(_){const v=_.target;v.removeEventListener("dispose",m);const S=a.indexOf(v.__bindingPointIndex);a.splice(S,1),r.deleteBuffer(i[v.id]),delete i[v.id],delete s[v.id]}function p(){for(const _ in i)r.deleteBuffer(i[_]);a=[],i={},s={}}return{bind:c,update:l,dispose:p}}class _u{constructor(t={}){const{canvas:e=U0(),context:n=null,depth:i=!0,stencil:s=!0,alpha:a=!1,antialias:o=!1,premultipliedAlpha:c=!0,preserveDrawingBuffer:l=!1,powerPreference:h="default",failIfMajorPerformanceCaveat:u=!1}=t;this.isWebGLRenderer=!0;let f;n!==null?f=n.getContextAttributes().alpha:f=a;const d=new Uint32Array(4),g=new Int32Array(4);let x=null,m=null;const p=[],_=[];this.domElement=e,this.debug={checkShaderErrors:!0,onShaderError:null},this.autoClear=!0,this.autoClearColor=!0,this.autoClearDepth=!0,this.autoClearStencil=!0,this.sortObjects=!0,this.clippingPlanes=[],this.localClippingEnabled=!1,this._outputColorSpace=sn,this._useLegacyLights=!1,this.toneMapping=as,this.toneMappingExposure=1;const v=this;let S=!1,y=0,M=0,w=null,T=-1,E=null;const A=new Je,D=new Je;let R=null;const N=new jt(0);let L=0,F=e.width,z=e.height,q=1,O=null,Y=null;const K=new Je(0,0,F,z),J=new Je(0,0,F,z);let it=!1;const X=new mu;let j=!1,nt=!1,ot=null;const ut=new se,rt=new St,ht=new b,dt={background:null,fog:null,environment:null,overrideMaterial:null,isScene:!0};function bt(){return w===null?q:1}let V=n;function re(P,G){for(let W=0;W<P.length;W++){const Z=P[W],k=e.getContext(Z,G);if(k!==null)return k}return null}try{const P={alpha:!0,depth:i,stencil:s,antialias:o,premultipliedAlpha:c,preserveDrawingBuffer:l,powerPreference:h,failIfMajorPerformanceCaveat:u};if("setAttribute"in e&&e.setAttribute("data-engine",`three.js r${Bc}`),e.addEventListener("webglcontextlost",ct,!1),e.addEventListener("webglcontextrestored",U,!1),e.addEventListener("webglcontextcreationerror",ft,!1),V===null){const G=["webgl2","webgl","experimental-webgl"];if(v.isWebGL1Renderer===!0&&G.shift(),V=re(G,P),V===null)throw re(G)?new Error("Error creating WebGL context with your selected attributes."):new Error("Error creating WebGL context.")}typeof WebGLRenderingContext<"u"&&V instanceof WebGLRenderingContext&&console.warn("THREE.WebGLRenderer: WebGL 1 support was deprecated in r153 and will be removed in r163."),V.getShaderPrecisionFormat===void 0&&(V.getShaderPrecisionFormat=function(){return{rangeMin:1,rangeMax:1,precision:1}})}catch(P){throw console.error("THREE.WebGLRenderer: "+P.message),P}let Tt,It,vt,Jt,Rt,I,C,H,Q,$,tt,Et,at,xt,Pt,Ht,et,ae,qt,Gt,wt,gt,Dt,$t;function de(){Tt=new $v(V),It=new Wv(V,Tt,t),Tt.init(It),gt=new FM(V,Tt,It),vt=new DM(V,Tt,It),Jt=new Qv(V),Rt=new vM,I=new NM(V,Tt,vt,Rt,It,gt,Jt),C=new Yv(v),H=new jv(v),Q=new ax(V,It),Dt=new kv(V,Tt,Q,It),$=new Kv(V,Q,Jt,Dt),tt=new iS(V,$,Q,Jt),qt=new nS(V,It,I),Ht=new Xv(Rt),Et=new _M(v,C,H,Tt,It,Dt,Ht),at=new OM(v,Rt),xt=new MM,Pt=new bM(Tt,It),ae=new Vv(v,C,H,vt,tt,f,c),et=new IM(v,tt,It),$t=new GM(V,Jt,It,vt),Gt=new Hv(V,Tt,Jt,It),wt=new Jv(V,Tt,Jt,It),Jt.programs=Et.programs,v.capabilities=It,v.extensions=Tt,v.properties=Rt,v.renderLists=xt,v.shadowMap=et,v.state=vt,v.info=Jt}de();const Xt=new BM(v,V);this.xr=Xt,this.getContext=function(){return V},this.getContextAttributes=function(){return V.getContextAttributes()},this.forceContextLoss=function(){const P=Tt.get("WEBGL_lose_context");P&&P.loseContext()},this.forceContextRestore=function(){const P=Tt.get("WEBGL_lose_context");P&&P.restoreContext()},this.getPixelRatio=function(){return q},this.setPixelRatio=function(P){P!==void 0&&(q=P,this.setSize(F,z,!1))},this.getSize=function(P){return P.set(F,z)},this.setSize=function(P,G,W=!0){if(Xt.isPresenting){console.warn("THREE.WebGLRenderer: Can't change size while VR device is presenting.");return}F=P,z=G,e.width=Math.floor(P*q),e.height=Math.floor(G*q),W===!0&&(e.style.width=P+"px",e.style.height=G+"px"),this.setViewport(0,0,P,G)},this.getDrawingBufferSize=function(P){return P.set(F*q,z*q).floor()},this.setDrawingBufferSize=function(P,G,W){F=P,z=G,q=W,e.width=Math.floor(P*W),e.height=Math.floor(G*W),this.setViewport(0,0,P,G)},this.getCurrentViewport=function(P){return P.copy(A)},this.getViewport=function(P){return P.copy(K)},this.setViewport=function(P,G,W,Z){P.isVector4?K.set(P.x,P.y,P.z,P.w):K.set(P,G,W,Z),vt.viewport(A.copy(K).multiplyScalar(q).floor())},this.getScissor=function(P){return P.copy(J)},this.setScissor=function(P,G,W,Z){P.isVector4?J.set(P.x,P.y,P.z,P.w):J.set(P,G,W,Z),vt.scissor(D.copy(J).multiplyScalar(q).floor())},this.getScissorTest=function(){return it},this.setScissorTest=function(P){vt.setScissorTest(it=P)},this.setOpaqueSort=function(P){O=P},this.setTransparentSort=function(P){Y=P},this.getClearColor=function(P){return P.copy(ae.getClearColor())},this.setClearColor=function(){ae.setClearColor.apply(ae,arguments)},this.getClearAlpha=function(){return ae.getClearAlpha()},this.setClearAlpha=function(){ae.setClearAlpha.apply(ae,arguments)},this.clear=function(P=!0,G=!0,W=!0){let Z=0;if(P){let k=!1;if(w!==null){const lt=w.texture.format;k=lt===em||lt===tm||lt===Qp}if(k){const lt=w.texture.type,At=lt===cs||lt===ns||lt===hu||lt===Is||lt===Kp||lt===Jp,Ct=ae.getClearColor(),_t=ae.getClearAlpha(),Vt=Ct.r,Ft=Ct.g,Bt=Ct.b;At?(d[0]=Vt,d[1]=Ft,d[2]=Bt,d[3]=_t,V.clearBufferuiv(V.COLOR,0,d)):(g[0]=Vt,g[1]=Ft,g[2]=Bt,g[3]=_t,V.clearBufferiv(V.COLOR,0,g))}else Z|=V.COLOR_BUFFER_BIT}G&&(Z|=V.DEPTH_BUFFER_BIT),W&&(Z|=V.STENCIL_BUFFER_BIT,this.state.buffers.stencil.setMask(4294967295)),V.clear(Z)},this.clearColor=function(){this.clear(!0,!1,!1)},this.clearDepth=function(){this.clear(!1,!0,!1)},this.clearStencil=function(){this.clear(!1,!1,!0)},this.dispose=function(){e.removeEventListener("webglcontextlost",ct,!1),e.removeEventListener("webglcontextrestored",U,!1),e.removeEventListener("webglcontextcreationerror",ft,!1),xt.dispose(),Pt.dispose(),Rt.dispose(),C.dispose(),H.dispose(),tt.dispose(),Dt.dispose(),$t.dispose(),Et.dispose(),Xt.dispose(),Xt.removeEventListener("sessionstart",Ae),Xt.removeEventListener("sessionend",oe),ot&&(ot.dispose(),ot=null),Pe.stop()};function ct(P){P.preventDefault(),console.log("THREE.WebGLRenderer: Context Lost."),S=!0}function U(){console.log("THREE.WebGLRenderer: Context Restored."),S=!1;const P=Jt.autoReset,G=et.enabled,W=et.autoUpdate,Z=et.needsUpdate,k=et.type;de(),Jt.autoReset=P,et.enabled=G,et.autoUpdate=W,et.needsUpdate=Z,et.type=k}function ft(P){console.error("THREE.WebGLRenderer: A WebGL context could not be created. Reason: ",P.statusMessage)}function pt(P){const G=P.target;G.removeEventListener("dispose",pt),zt(G)}function zt(P){Nt(P),Rt.remove(P)}function Nt(P){const G=Rt.get(P).programs;G!==void 0&&(G.forEach(function(W){Et.releaseProgram(W)}),P.isShaderMaterial&&Et.releaseShaderCache(P))}this.renderBufferDirect=function(P,G,W,Z,k,lt){G===null&&(G=dt);const At=k.isMesh&&k.matrixWorld.determinant()<0,Ct=Ei(P,G,W,Z,k);vt.setMaterial(Z,At);let _t=W.index,Vt=1;if(Z.wireframe===!0){if(_t=$.getWireframeAttribute(W),_t===void 0)return;Vt=2}const Ft=W.drawRange,Bt=W.attributes.position;let fe=Ft.start*Vt,Ze=(Ft.start+Ft.count)*Vt;lt!==null&&(fe=Math.max(fe,lt.start*Vt),Ze=Math.min(Ze,(lt.start+lt.count)*Vt)),_t!==null?(fe=Math.max(fe,0),Ze=Math.min(Ze,_t.count)):Bt!=null&&(fe=Math.max(fe,0),Ze=Math.min(Ze,Bt.count));const Te=Ze-fe;if(Te<0||Te===1/0)return;Dt.setup(k,Z,Ct,W,_t);let Kn,pe=Gt;if(_t!==null&&(Kn=Q.get(_t),pe=wt,pe.setIndex(Kn)),k.isMesh)Z.wireframe===!0?(vt.setLineWidth(Z.wireframeLinewidth*bt()),pe.setMode(V.LINES)):pe.setMode(V.TRIANGLES);else if(k.isLine){let Kt=Z.linewidth;Kt===void 0&&(Kt=1),vt.setLineWidth(Kt*bt()),k.isLineSegments?pe.setMode(V.LINES):k.isLineLoop?pe.setMode(V.LINE_LOOP):pe.setMode(V.LINE_STRIP)}else k.isPoints?pe.setMode(V.POINTS):k.isSprite&&pe.setMode(V.TRIANGLES);if(k.isBatchedMesh)pe.renderMultiDraw(k._multiDrawStarts,k._multiDrawCounts,k._multiDrawCount);else if(k.isInstancedMesh)pe.renderInstances(fe,Te,k.count);else if(W.isInstancedBufferGeometry){const Kt=W._maxInstanceCount!==void 0?W._maxInstanceCount:1/0,Yr=Math.min(W.instanceCount,Kt);pe.renderInstances(fe,Te,Yr)}else pe.render(fe,Te)};function ue(P,G,W){P.transparent===!0&&P.side===Ke&&P.forceSinglePass===!1?(P.side=dn,P.needsUpdate=!0,Tn(P,G,W),P.side=oi,P.needsUpdate=!0,Tn(P,G,W),P.side=Ke):Tn(P,G,W)}this.compile=function(P,G,W=null){W===null&&(W=P),m=Pt.get(W),m.init(),_.push(m),W.traverseVisible(function(k){k.isLight&&k.layers.test(G.layers)&&(m.pushLight(k),k.castShadow&&m.pushShadow(k))}),P!==W&&P.traverseVisible(function(k){k.isLight&&k.layers.test(G.layers)&&(m.pushLight(k),k.castShadow&&m.pushShadow(k))}),m.setupLights(v._useLegacyLights);const Z=new Set;return P.traverse(function(k){const lt=k.material;if(lt)if(Array.isArray(lt))for(let At=0;At<lt.length;At++){const Ct=lt[At];ue(Ct,W,k),Z.add(Ct)}else ue(lt,W,k),Z.add(lt)}),_.pop(),m=null,Z},this.compileAsync=function(P,G,W=null){const Z=this.compile(P,G,W);return new Promise(k=>{function lt(){if(Z.forEach(function(At){Rt.get(At).currentProgram.isReady()&&Z.delete(At)}),Z.size===0){k(P);return}setTimeout(lt,10)}Tt.get("KHR_parallel_shader_compile")!==null?lt():setTimeout(lt,10)})};let ce=null;function _e(P){ce&&ce(P)}function Ae(){Pe.stop()}function oe(){Pe.start()}const Pe=new gm;Pe.setAnimationLoop(_e),typeof self<"u"&&Pe.setContext(self),this.setAnimationLoop=function(P){ce=P,Xt.setAnimationLoop(P),P===null?Pe.stop():Pe.start()},Xt.addEventListener("sessionstart",Ae),Xt.addEventListener("sessionend",oe),this.render=function(P,G){if(G!==void 0&&G.isCamera!==!0){console.error("THREE.WebGLRenderer.render: camera is not an instance of THREE.Camera.");return}if(S===!0)return;P.matrixWorldAutoUpdate===!0&&P.updateMatrixWorld(),G.parent===null&&G.matrixWorldAutoUpdate===!0&&G.updateMatrixWorld(),Xt.enabled===!0&&Xt.isPresenting===!0&&(Xt.cameraAutoUpdate===!0&&Xt.updateCamera(G),G=Xt.getCamera()),P.isScene===!0&&P.onBeforeRender(v,P,G,w),m=Pt.get(P,_.length),m.init(),_.push(m),ut.multiplyMatrices(G.projectionMatrix,G.matrixWorldInverse),X.setFromProjectionMatrix(ut),nt=this.localClippingEnabled,j=Ht.init(this.clippingPlanes,nt),x=xt.get(P,p.length),x.init(),p.push(x),qe(P,G,0,v.sortObjects),x.finish(),v.sortObjects===!0&&x.sort(O,Y),this.info.render.frame++,j===!0&&Ht.beginShadows();const W=m.state.shadowsArray;if(et.render(W,P,G),j===!0&&Ht.endShadows(),this.info.autoReset===!0&&this.info.reset(),ae.render(x,P),m.setupLights(v._useLegacyLights),G.isArrayCamera){const Z=G.cameras;for(let k=0,lt=Z.length;k<lt;k++){const At=Z[k];yi(x,P,At,At.viewport)}}else yi(x,P,G);w!==null&&(I.updateMultisampleRenderTarget(w),I.updateRenderTargetMipmap(w)),P.isScene===!0&&P.onAfterRender(v,P,G),Dt.resetDefaultState(),T=-1,E=null,_.pop(),_.length>0?m=_[_.length-1]:m=null,p.pop(),p.length>0?x=p[p.length-1]:x=null};function qe(P,G,W,Z){if(P.visible===!1)return;if(P.layers.test(G.layers)){if(P.isGroup)W=P.renderOrder;else if(P.isLOD)P.autoUpdate===!0&&P.update(G);else if(P.isLight)m.pushLight(P),P.castShadow&&m.pushShadow(P);else if(P.isSprite){if(!P.frustumCulled||X.intersectsSprite(P)){Z&&ht.setFromMatrixPosition(P.matrixWorld).applyMatrix4(ut);const At=tt.update(P),Ct=P.material;Ct.visible&&x.push(P,At,Ct,W,ht.z,null)}}else if((P.isMesh||P.isLine||P.isPoints)&&(!P.frustumCulled||X.intersectsObject(P))){const At=tt.update(P),Ct=P.material;if(Z&&(P.boundingSphere!==void 0?(P.boundingSphere===null&&P.computeBoundingSphere(),ht.copy(P.boundingSphere.center)):(At.boundingSphere===null&&At.computeBoundingSphere(),ht.copy(At.boundingSphere.center)),ht.applyMatrix4(P.matrixWorld).applyMatrix4(ut)),Array.isArray(Ct)){const _t=At.groups;for(let Vt=0,Ft=_t.length;Vt<Ft;Vt++){const Bt=_t[Vt],fe=Ct[Bt.materialIndex];fe&&fe.visible&&x.push(P,At,fe,W,ht.z,Bt)}}else Ct.visible&&x.push(P,At,Ct,W,ht.z,null)}}const lt=P.children;for(let At=0,Ct=lt.length;At<Ct;At++)qe(lt[At],G,W,Z)}function yi(P,G,W,Z){const k=P.opaque,lt=P.transmissive,At=P.transparent;m.setupLightsView(W),j===!0&&Ht.setGlobalState(v.clippingPlanes,W),lt.length>0&&Os(k,lt,G,W),Z&&vt.viewport(A.copy(Z)),k.length>0&&An(k,G,W),lt.length>0&&An(lt,G,W),At.length>0&&An(At,G,W),vt.buffers.depth.setTest(!0),vt.buffers.depth.setMask(!0),vt.buffers.color.setMask(!0),vt.setPolygonOffset(!1)}function Os(P,G,W,Z){if((W.isScene===!0?W.overrideMaterial:null)!==null)return;const lt=It.isWebGL2;ot===null&&(ot=new tn(1,1,{generateMipmaps:!0,type:Tt.has("EXT_color_buffer_half_float")?Nr:cs,minFilter:$o,samples:lt?4:0})),v.getDrawingBufferSize(rt),lt?ot.setSize(rt.x,rt.y):ot.setSize(yc(rt.x),yc(rt.y));const At=v.getRenderTarget();v.setRenderTarget(ot),v.getClearColor(N),L=v.getClearAlpha(),L<1&&v.setClearColor(16777215,.5),v.clear();const Ct=v.toneMapping;v.toneMapping=as,An(P,W,Z),I.updateMultisampleRenderTarget(ot),I.updateRenderTargetMipmap(ot);let _t=!1;for(let Vt=0,Ft=G.length;Vt<Ft;Vt++){const Bt=G[Vt],fe=Bt.object,Ze=Bt.geometry,Te=Bt.material,Kn=Bt.group;if(Te.side===Ke&&fe.layers.test(Z.layers)){const pe=Te.side;Te.side=dn,Te.needsUpdate=!0,Wi(fe,W,Z,Ze,Te,Kn),Te.side=pe,Te.needsUpdate=!0,_t=!0}}_t===!0&&(I.updateMultisampleRenderTarget(ot),I.updateRenderTargetMipmap(ot)),v.setRenderTarget(At),v.setClearColor(N,L),v.toneMapping=Ct}function An(P,G,W){const Z=G.isScene===!0?G.overrideMaterial:null;for(let k=0,lt=P.length;k<lt;k++){const At=P[k],Ct=At.object,_t=At.geometry,Vt=Z===null?At.material:Z,Ft=At.group;Ct.layers.test(W.layers)&&Wi(Ct,G,W,_t,Vt,Ft)}}function Wi(P,G,W,Z,k,lt){P.onBeforeRender(v,G,W,Z,k,lt),P.modelViewMatrix.multiplyMatrices(W.matrixWorldInverse,P.matrixWorld),P.normalMatrix.getNormalMatrix(P.modelViewMatrix),k.onBeforeRender(v,G,W,Z,P,lt),k.transparent===!0&&k.side===Ke&&k.forceSinglePass===!1?(k.side=dn,k.needsUpdate=!0,v.renderBufferDirect(W,G,Z,k,P,lt),k.side=oi,k.needsUpdate=!0,v.renderBufferDirect(W,G,Z,k,P,lt),k.side=Ke):v.renderBufferDirect(W,G,Z,k,P,lt),P.onAfterRender(v,G,W,Z,k,lt)}function Tn(P,G,W){G.isScene!==!0&&(G=dt);const Z=Rt.get(P),k=m.state.lights,lt=m.state.shadowsArray,At=k.state.version,Ct=Et.getParameters(P,k.state,lt,G,W),_t=Et.getProgramCacheKey(Ct);let Vt=Z.programs;Z.environment=P.isMeshStandardMaterial?G.environment:null,Z.fog=G.fog,Z.envMap=(P.isMeshStandardMaterial?H:C).get(P.envMap||Z.environment),Vt===void 0&&(P.addEventListener("dispose",pt),Vt=new Map,Z.programs=Vt);let Ft=Vt.get(_t);if(Ft!==void 0){if(Z.currentProgram===Ft&&Z.lightsStateVersion===At)return nn(P,Ct),Ft}else Ct.uniforms=Et.getUniforms(P),P.onBuild(W,Ct,v),P.onBeforeCompile(Ct,v),Ft=Et.acquireProgram(Ct,_t),Vt.set(_t,Ft),Z.uniforms=Ct.uniforms;const Bt=Z.uniforms;return(!P.isShaderMaterial&&!P.isRawShaderMaterial||P.clipping===!0)&&(Bt.clippingPlanes=Ht.uniform),nn(P,Ct),Z.needsLights=Ut(P),Z.lightsStateVersion=At,Z.needsLights&&(Bt.ambientLightColor.value=k.state.ambient,Bt.lightProbe.value=k.state.probe,Bt.directionalLights.value=k.state.directional,Bt.directionalLightShadows.value=k.state.directionalShadow,Bt.spotLights.value=k.state.spot,Bt.spotLightShadows.value=k.state.spotShadow,Bt.rectAreaLights.value=k.state.rectArea,Bt.ltc_1.value=k.state.rectAreaLTC1,Bt.ltc_2.value=k.state.rectAreaLTC2,Bt.pointLights.value=k.state.point,Bt.pointLightShadows.value=k.state.pointShadow,Bt.hemisphereLights.value=k.state.hemi,Bt.directionalShadowMap.value=k.state.directionalShadowMap,Bt.directionalShadowMatrix.value=k.state.directionalShadowMatrix,Bt.spotShadowMap.value=k.state.spotShadowMap,Bt.spotLightMatrix.value=k.state.spotLightMatrix,Bt.spotLightMap.value=k.state.spotLightMap,Bt.pointShadowMap.value=k.state.pointShadowMap,Bt.pointShadowMatrix.value=k.state.pointShadowMatrix),Z.currentProgram=Ft,Z.uniformsList=null,Ft}function Gs(P){if(P.uniformsList===null){const G=P.currentProgram.getUniforms();P.uniformsList=ac.seqWithValue(G.seq,P.uniforms)}return P.uniformsList}function nn(P,G){const W=Rt.get(P);W.outputColorSpace=G.outputColorSpace,W.batching=G.batching,W.instancing=G.instancing,W.instancingColor=G.instancingColor,W.skinning=G.skinning,W.morphTargets=G.morphTargets,W.morphNormals=G.morphNormals,W.morphColors=G.morphColors,W.morphTargetsCount=G.morphTargetsCount,W.numClippingPlanes=G.numClippingPlanes,W.numIntersection=G.numClipIntersection,W.vertexAlphas=G.vertexAlphas,W.vertexTangents=G.vertexTangents,W.toneMapping=G.toneMapping}function Ei(P,G,W,Z,k){G.isScene!==!0&&(G=dt),I.resetTextureUnits();const lt=G.fog,At=Z.isMeshStandardMaterial?G.environment:null,Ct=w===null?v.outputColorSpace:w.isXRRenderTarget===!0?w.texture.colorSpace:Vi,_t=(Z.isMeshStandardMaterial?H:C).get(Z.envMap||At),Vt=Z.vertexColors===!0&&!!W.attributes.color&&W.attributes.color.itemSize===4,Ft=!!W.attributes.tangent&&(!!Z.normalMap||Z.anisotropy>0),Bt=!!W.morphAttributes.position,fe=!!W.morphAttributes.normal,Ze=!!W.morphAttributes.color;let Te=as;Z.toneMapped&&(w===null||w.isXRRenderTarget===!0)&&(Te=v.toneMapping);const Kn=W.morphAttributes.position||W.morphAttributes.normal||W.morphAttributes.color,pe=Kn!==void 0?Kn.length:0,Kt=Rt.get(Z),Yr=m.state.lights;if(j===!0&&(nt===!0||P!==E)){const Mn=P===E&&Z.id===T;Ht.setState(Z,P,Mn)}let Ee=!1;Z.version===Kt.__version?(Kt.needsLights&&Kt.lightsStateVersion!==Yr.state.version||Kt.outputColorSpace!==Ct||k.isBatchedMesh&&Kt.batching===!1||!k.isBatchedMesh&&Kt.batching===!0||k.isInstancedMesh&&Kt.instancing===!1||!k.isInstancedMesh&&Kt.instancing===!0||k.isSkinnedMesh&&Kt.skinning===!1||!k.isSkinnedMesh&&Kt.skinning===!0||k.isInstancedMesh&&Kt.instancingColor===!0&&k.instanceColor===null||k.isInstancedMesh&&Kt.instancingColor===!1&&k.instanceColor!==null||Kt.envMap!==_t||Z.fog===!0&&Kt.fog!==lt||Kt.numClippingPlanes!==void 0&&(Kt.numClippingPlanes!==Ht.numPlanes||Kt.numIntersection!==Ht.numIntersection)||Kt.vertexAlphas!==Vt||Kt.vertexTangents!==Ft||Kt.morphTargets!==Bt||Kt.morphNormals!==fe||Kt.morphColors!==Ze||Kt.toneMapping!==Te||It.isWebGL2===!0&&Kt.morphTargetsCount!==pe)&&(Ee=!0):(Ee=!0,Kt.__version=Z.version);let wi=Kt.currentProgram;Ee===!0&&(wi=Tn(Z,G,k));let B=!1,Mt=!1,Ot=!1;const te=wi.getUniforms(),Ue=Kt.uniforms;if(vt.useProgram(wi.program)&&(B=!0,Mt=!0,Ot=!0),Z.id!==T&&(T=Z.id,Mt=!0),B||E!==P){te.setValue(V,"projectionMatrix",P.projectionMatrix),te.setValue(V,"viewMatrix",P.matrixWorldInverse);const Mn=te.map.cameraPosition;Mn!==void 0&&Mn.setValue(V,ht.setFromMatrixPosition(P.matrixWorld)),It.logarithmicDepthBuffer&&te.setValue(V,"logDepthBufFC",2/(Math.log(P.far+1)/Math.LN2)),(Z.isMeshPhongMaterial||Z.isMeshToonMaterial||Z.isMeshLambertMaterial||Z.isMeshBasicMaterial||Z.isMeshStandardMaterial||Z.isShaderMaterial)&&te.setValue(V,"isOrthographic",P.isOrthographicCamera===!0),E!==P&&(E=P,Mt=!0,Ot=!0)}if(k.isSkinnedMesh){te.setOptional(V,k,"bindMatrix"),te.setOptional(V,k,"bindMatrixInverse");const Mn=k.skeleton;Mn&&(It.floatVertexTextures?(Mn.boneTexture===null&&Mn.computeBoneTexture(),te.setValue(V,"boneTexture",Mn.boneTexture,I)):console.warn("THREE.WebGLRenderer: SkinnedMesh can only be used with WebGL 2. With WebGL 1 OES_texture_float and vertex textures support is required."))}k.isBatchedMesh&&(te.setOptional(V,k,"batchingTexture"),te.setValue(V,"batchingTexture",k._matricesTexture,I));const Vs=W.morphAttributes;if((Vs.position!==void 0||Vs.normal!==void 0||Vs.color!==void 0&&It.isWebGL2===!0)&&qt.update(k,W,wi),(Mt||Kt.receiveShadow!==k.receiveShadow)&&(Kt.receiveShadow=k.receiveShadow,te.setValue(V,"receiveShadow",k.receiveShadow)),Z.isMeshGouraudMaterial&&Z.envMap!==null&&(Ue.envMap.value=_t,Ue.flipEnvMap.value=_t.isCubeTexture&&_t.isRenderTargetTexture===!1?-1:1),Mt&&(te.setValue(V,"toneMappingExposure",v.toneMappingExposure),Kt.needsLights&&st(Ue,Ot),lt&&Z.fog===!0&&at.refreshFogUniforms(Ue,lt),at.refreshMaterialUniforms(Ue,Z,q,z,ot),ac.upload(V,Gs(Kt),Ue,I)),Z.isShaderMaterial&&Z.uniformsNeedUpdate===!0&&(ac.upload(V,Gs(Kt),Ue,I),Z.uniformsNeedUpdate=!1),Z.isSpriteMaterial&&te.setValue(V,"center",k.center),te.setValue(V,"modelViewMatrix",k.modelViewMatrix),te.setValue(V,"normalMatrix",k.normalMatrix),te.setValue(V,"modelMatrix",k.matrixWorld),Z.isShaderMaterial||Z.isRawShaderMaterial){const Mn=Z.uniformsGroups;for(let qr=0,wg=Mn.length;qr<wg;qr++)if(It.isWebGL2){const Gu=Mn[qr];$t.update(Gu,wi),$t.bind(Gu,wi)}else console.warn("THREE.WebGLRenderer: Uniform Buffer Objects can only be used with WebGL 2.")}return wi}function st(P,G){P.ambientLightColor.needsUpdate=G,P.lightProbe.needsUpdate=G,P.directionalLights.needsUpdate=G,P.directionalLightShadows.needsUpdate=G,P.pointLights.needsUpdate=G,P.pointLightShadows.needsUpdate=G,P.spotLights.needsUpdate=G,P.spotLightShadows.needsUpdate=G,P.rectAreaLights.needsUpdate=G,P.hemisphereLights.needsUpdate=G}function Ut(P){return P.isMeshLambertMaterial||P.isMeshToonMaterial||P.isMeshPhongMaterial||P.isMeshStandardMaterial||P.isShadowMaterial||P.isShaderMaterial&&P.lights===!0}this.getActiveCubeFace=function(){return y},this.getActiveMipmapLevel=function(){return M},this.getRenderTarget=function(){return w},this.setRenderTargetTextures=function(P,G,W){Rt.get(P.texture).__webglTexture=G,Rt.get(P.depthTexture).__webglTexture=W;const Z=Rt.get(P);Z.__hasExternalTextures=!0,Z.__hasExternalTextures&&(Z.__autoAllocateDepthBuffer=W===void 0,Z.__autoAllocateDepthBuffer||Tt.has("WEBGL_multisampled_render_to_texture")===!0&&(console.warn("THREE.WebGLRenderer: Render-to-texture extension was disabled because an external texture was provided"),Z.__useRenderToTexture=!1))},this.setRenderTargetFramebuffer=function(P,G){const W=Rt.get(P);W.__webglFramebuffer=G,W.__useDefaultFramebuffer=G===void 0},this.setRenderTarget=function(P,G=0,W=0){w=P,y=G,M=W;let Z=!0,k=null,lt=!1,At=!1;if(P){const _t=Rt.get(P);_t.__useDefaultFramebuffer!==void 0?(vt.bindFramebuffer(V.FRAMEBUFFER,null),Z=!1):_t.__webglFramebuffer===void 0?I.setupRenderTarget(P):_t.__hasExternalTextures&&I.rebindTextures(P,Rt.get(P.texture).__webglTexture,Rt.get(P.depthTexture).__webglTexture);const Vt=P.texture;(Vt.isData3DTexture||Vt.isDataArrayTexture||Vt.isCompressedArrayTexture)&&(At=!0);const Ft=Rt.get(P).__webglFramebuffer;P.isWebGLCubeRenderTarget?(Array.isArray(Ft[G])?k=Ft[G][W]:k=Ft[G],lt=!0):It.isWebGL2&&P.samples>0&&I.useMultisampledRTT(P)===!1?k=Rt.get(P).__webglMultisampledFramebuffer:Array.isArray(Ft)?k=Ft[W]:k=Ft,A.copy(P.viewport),D.copy(P.scissor),R=P.scissorTest}else A.copy(K).multiplyScalar(q).floor(),D.copy(J).multiplyScalar(q).floor(),R=it;if(vt.bindFramebuffer(V.FRAMEBUFFER,k)&&It.drawBuffers&&Z&&vt.drawBuffers(P,k),vt.viewport(A),vt.scissor(D),vt.setScissorTest(R),lt){const _t=Rt.get(P.texture);V.framebufferTexture2D(V.FRAMEBUFFER,V.COLOR_ATTACHMENT0,V.TEXTURE_CUBE_MAP_POSITIVE_X+G,_t.__webglTexture,W)}else if(At){const _t=Rt.get(P.texture),Vt=G||0;V.framebufferTextureLayer(V.FRAMEBUFFER,V.COLOR_ATTACHMENT0,_t.__webglTexture,W||0,Vt)}T=-1},this.readRenderTargetPixels=function(P,G,W,Z,k,lt,At){if(!(P&&P.isWebGLRenderTarget)){console.error("THREE.WebGLRenderer.readRenderTargetPixels: renderTarget is not THREE.WebGLRenderTarget.");return}let Ct=Rt.get(P).__webglFramebuffer;if(P.isWebGLCubeRenderTarget&&At!==void 0&&(Ct=Ct[At]),Ct){vt.bindFramebuffer(V.FRAMEBUFFER,Ct);try{const _t=P.texture,Vt=_t.format,Ft=_t.type;if(Vt!==ri&&gt.convert(Vt)!==V.getParameter(V.IMPLEMENTATION_COLOR_READ_FORMAT)){console.error("THREE.WebGLRenderer.readRenderTargetPixels: renderTarget is not in RGBA or implementation defined format.");return}const Bt=Ft===Nr&&(Tt.has("EXT_color_buffer_half_float")||It.isWebGL2&&Tt.has("EXT_color_buffer_float"));if(Ft!==cs&&gt.convert(Ft)!==V.getParameter(V.IMPLEMENTATION_COLOR_READ_TYPE)&&!(Ft===is&&(It.isWebGL2||Tt.has("OES_texture_float")||Tt.has("WEBGL_color_buffer_float")))&&!Bt){console.error("THREE.WebGLRenderer.readRenderTargetPixels: renderTarget is not in UnsignedByteType or implementation defined type.");return}G>=0&&G<=P.width-Z&&W>=0&&W<=P.height-k&&V.readPixels(G,W,Z,k,gt.convert(Vt),gt.convert(Ft),lt)}finally{const _t=w!==null?Rt.get(w).__webglFramebuffer:null;vt.bindFramebuffer(V.FRAMEBUFFER,_t)}}},this.copyFramebufferToTexture=function(P,G,W=0){const Z=Math.pow(2,-W),k=Math.floor(G.image.width*Z),lt=Math.floor(G.image.height*Z);I.setTexture2D(G,0),V.copyTexSubImage2D(V.TEXTURE_2D,W,0,0,P.x,P.y,k,lt),vt.unbindTexture()},this.copyTextureToTexture=function(P,G,W,Z=0){const k=G.image.width,lt=G.image.height,At=gt.convert(W.format),Ct=gt.convert(W.type);I.setTexture2D(W,0),V.pixelStorei(V.UNPACK_FLIP_Y_WEBGL,W.flipY),V.pixelStorei(V.UNPACK_PREMULTIPLY_ALPHA_WEBGL,W.premultiplyAlpha),V.pixelStorei(V.UNPACK_ALIGNMENT,W.unpackAlignment),G.isDataTexture?V.texSubImage2D(V.TEXTURE_2D,Z,P.x,P.y,k,lt,At,Ct,G.image.data):G.isCompressedTexture?V.compressedTexSubImage2D(V.TEXTURE_2D,Z,P.x,P.y,G.mipmaps[0].width,G.mipmaps[0].height,At,G.mipmaps[0].data):V.texSubImage2D(V.TEXTURE_2D,Z,P.x,P.y,At,Ct,G.image),Z===0&&W.generateMipmaps&&V.generateMipmap(V.TEXTURE_2D),vt.unbindTexture()},this.copyTextureToTexture3D=function(P,G,W,Z,k=0){if(v.isWebGL1Renderer){console.warn("THREE.WebGLRenderer.copyTextureToTexture3D: can only be used with WebGL2.");return}const lt=P.max.x-P.min.x+1,At=P.max.y-P.min.y+1,Ct=P.max.z-P.min.z+1,_t=gt.convert(Z.format),Vt=gt.convert(Z.type);let Ft;if(Z.isData3DTexture)I.setTexture3D(Z,0),Ft=V.TEXTURE_3D;else if(Z.isDataArrayTexture||Z.isCompressedArrayTexture)I.setTexture2DArray(Z,0),Ft=V.TEXTURE_2D_ARRAY;else{console.warn("THREE.WebGLRenderer.copyTextureToTexture3D: only supports THREE.DataTexture3D and THREE.DataTexture2DArray.");return}V.pixelStorei(V.UNPACK_FLIP_Y_WEBGL,Z.flipY),V.pixelStorei(V.UNPACK_PREMULTIPLY_ALPHA_WEBGL,Z.premultiplyAlpha),V.pixelStorei(V.UNPACK_ALIGNMENT,Z.unpackAlignment);const Bt=V.getParameter(V.UNPACK_ROW_LENGTH),fe=V.getParameter(V.UNPACK_IMAGE_HEIGHT),Ze=V.getParameter(V.UNPACK_SKIP_PIXELS),Te=V.getParameter(V.UNPACK_SKIP_ROWS),Kn=V.getParameter(V.UNPACK_SKIP_IMAGES),pe=W.isCompressedTexture?W.mipmaps[k]:W.image;V.pixelStorei(V.UNPACK_ROW_LENGTH,pe.width),V.pixelStorei(V.UNPACK_IMAGE_HEIGHT,pe.height),V.pixelStorei(V.UNPACK_SKIP_PIXELS,P.min.x),V.pixelStorei(V.UNPACK_SKIP_ROWS,P.min.y),V.pixelStorei(V.UNPACK_SKIP_IMAGES,P.min.z),W.isDataTexture||W.isData3DTexture?V.texSubImage3D(Ft,k,G.x,G.y,G.z,lt,At,Ct,_t,Vt,pe.data):W.isCompressedArrayTexture?(console.warn("THREE.WebGLRenderer.copyTextureToTexture3D: untested support for compressed srcTexture."),V.compressedTexSubImage3D(Ft,k,G.x,G.y,G.z,lt,At,Ct,_t,pe.data)):V.texSubImage3D(Ft,k,G.x,G.y,G.z,lt,At,Ct,_t,Vt,pe),V.pixelStorei(V.UNPACK_ROW_LENGTH,Bt),V.pixelStorei(V.UNPACK_IMAGE_HEIGHT,fe),V.pixelStorei(V.UNPACK_SKIP_PIXELS,Ze),V.pixelStorei(V.UNPACK_SKIP_ROWS,Te),V.pixelStorei(V.UNPACK_SKIP_IMAGES,Kn),k===0&&Z.generateMipmaps&&V.generateMipmap(Ft),vt.unbindTexture()},this.initTexture=function(P){P.isCubeTexture?I.setTextureCube(P,0):P.isData3DTexture?I.setTexture3D(P,0):P.isDataArrayTexture||P.isCompressedArrayTexture?I.setTexture2DArray(P,0):I.setTexture2D(P,0),vt.unbindTexture()},this.resetState=function(){y=0,M=0,w=null,vt.reset(),Dt.reset()},typeof __THREE_DEVTOOLS__<"u"&&__THREE_DEVTOOLS__.dispatchEvent(new CustomEvent("observe",{detail:this}))}get coordinateSystem(){return Oi}get outputColorSpace(){return this._outputColorSpace}set outputColorSpace(t){this._outputColorSpace=t;const e=this.getContext();e.drawingBufferColorSpace=t===du?"display-p3":"srgb",e.unpackColorSpace=me.workingColorSpace===Gc?"display-p3":"srgb"}get outputEncoding(){return console.warn("THREE.WebGLRenderer: Property .outputEncoding has been removed. Use .outputColorSpace instead."),this.outputColorSpace===sn?Ns:im}set outputEncoding(t){console.warn("THREE.WebGLRenderer: Property .outputEncoding has been removed. Use .outputColorSpace instead."),this.outputColorSpace=t===Ns?sn:Vi}get useLegacyLights(){return console.warn("THREE.WebGLRenderer: The property .useLegacyLights has been deprecated. Migrate your lighting according to the following guide: https://discourse.threejs.org/t/updates-to-lighting-in-three-js-r155/53733."),this._useLegacyLights}set useLegacyLights(t){console.warn("THREE.WebGLRenderer: The property .useLegacyLights has been deprecated. Migrate your lighting according to the following guide: https://discourse.threejs.org/t/updates-to-lighting-in-three-js-r155/53733."),this._useLegacyLights=t}}class VM extends _u{}VM.prototype.isWebGL1Renderer=!0;class kr extends Qe{constructor(){super(),this.isScene=!0,this.type="Scene",this.background=null,this.environment=null,this.fog=null,this.backgroundBlurriness=0,this.backgroundIntensity=1,this.overrideMaterial=null,typeof __THREE_DEVTOOLS__<"u"&&__THREE_DEVTOOLS__.dispatchEvent(new CustomEvent("observe",{detail:this}))}copy(t,e){return super.copy(t,e),t.background!==null&&(this.background=t.background.clone()),t.environment!==null&&(this.environment=t.environment.clone()),t.fog!==null&&(this.fog=t.fog.clone()),this.backgroundBlurriness=t.backgroundBlurriness,this.backgroundIntensity=t.backgroundIntensity,t.overrideMaterial!==null&&(this.overrideMaterial=t.overrideMaterial.clone()),this.matrixAutoUpdate=t.matrixAutoUpdate,this}toJSON(t){const e=super.toJSON(t);return this.fog!==null&&(e.object.fog=this.fog.toJSON()),this.backgroundBlurriness>0&&(e.object.backgroundBlurriness=this.backgroundBlurriness),this.backgroundIntensity!==1&&(e.object.backgroundIntensity=this.backgroundIntensity),e}}class af extends Ne{constructor(t,e,n,i=1){super(t,e,n),this.isInstancedBufferAttribute=!0,this.meshPerAttribute=i}copy(t){return super.copy(t),this.meshPerAttribute=t.meshPerAttribute,this}toJSON(){const t=super.toJSON();return t.meshPerAttribute=this.meshPerAttribute,t.isInstancedBufferAttribute=!0,t}}const sr=new se,cf=new se,ba=[],lf=new en,kM=new se,Jr=new Zt,Qr=new ms;class Hr extends Zt{constructor(t,e,n){super(t,e),this.isInstancedMesh=!0,this.instanceMatrix=new af(new Float32Array(n*16),16),this.instanceColor=null,this.count=n,this.boundingBox=null,this.boundingSphere=null;for(let i=0;i<n;i++)this.setMatrixAt(i,kM)}computeBoundingBox(){const t=this.geometry,e=this.count;this.boundingBox===null&&(this.boundingBox=new en),t.boundingBox===null&&t.computeBoundingBox(),this.boundingBox.makeEmpty();for(let n=0;n<e;n++)this.getMatrixAt(n,sr),lf.copy(t.boundingBox).applyMatrix4(sr),this.boundingBox.union(lf)}computeBoundingSphere(){const t=this.geometry,e=this.count;this.boundingSphere===null&&(this.boundingSphere=new ms),t.boundingSphere===null&&t.computeBoundingSphere(),this.boundingSphere.makeEmpty();for(let n=0;n<e;n++)this.getMatrixAt(n,sr),Qr.copy(t.boundingSphere).applyMatrix4(sr),this.boundingSphere.union(Qr)}copy(t,e){return super.copy(t,e),this.instanceMatrix.copy(t.instanceMatrix),t.instanceColor!==null&&(this.instanceColor=t.instanceColor.clone()),this.count=t.count,t.boundingBox!==null&&(this.boundingBox=t.boundingBox.clone()),t.boundingSphere!==null&&(this.boundingSphere=t.boundingSphere.clone()),this}getColorAt(t,e){e.fromArray(this.instanceColor.array,t*3)}getMatrixAt(t,e){e.fromArray(this.instanceMatrix.array,t*16)}raycast(t,e){const n=this.matrixWorld,i=this.count;if(Jr.geometry=this.geometry,Jr.material=this.material,Jr.material!==void 0&&(this.boundingSphere===null&&this.computeBoundingSphere(),Qr.copy(this.boundingSphere),Qr.applyMatrix4(n),t.ray.intersectsSphere(Qr)!==!1))for(let s=0;s<i;s++){this.getMatrixAt(s,sr),cf.multiplyMatrices(n,sr),Jr.matrixWorld=cf,Jr.raycast(t,ba);for(let a=0,o=ba.length;a<o;a++){const c=ba[a];c.instanceId=s,c.object=this,e.push(c)}ba.length=0}}setColorAt(t,e){this.instanceColor===null&&(this.instanceColor=new af(new Float32Array(this.instanceMatrix.count*3),3)),e.toArray(this.instanceColor.array,t*3)}setMatrixAt(t,e){e.toArray(this.instanceMatrix.array,t*16)}updateMorphTargets(){}dispose(){this.dispatchEvent({type:"dispose"})}}class vi extends Si{constructor(t){super(),this.isLineBasicMaterial=!0,this.type="LineBasicMaterial",this.color=new jt(16777215),this.map=null,this.linewidth=1,this.linecap="round",this.linejoin="round",this.fog=!0,this.setValues(t)}copy(t){return super.copy(t),this.color.copy(t.color),this.map=t.map,this.linewidth=t.linewidth,this.linecap=t.linecap,this.linejoin=t.linejoin,this.fog=t.fog,this}}const hf=new b,uf=new b,df=new se,Cl=new pu,Ca=new ms;class HM extends Qe{constructor(t=new ye,e=new vi){super(),this.isLine=!0,this.type="Line",this.geometry=t,this.material=e,this.updateMorphTargets()}copy(t,e){return super.copy(t,e),this.material=Array.isArray(t.material)?t.material.slice():t.material,this.geometry=t.geometry,this}computeLineDistances(){const t=this.geometry;if(t.index===null){const e=t.attributes.position,n=[0];for(let i=1,s=e.count;i<s;i++)hf.fromBufferAttribute(e,i-1),uf.fromBufferAttribute(e,i),n[i]=n[i-1],n[i]+=hf.distanceTo(uf);t.setAttribute("lineDistance",new ie(n,1))}else console.warn("THREE.Line.computeLineDistances(): Computation only possible with non-indexed BufferGeometry.");return this}raycast(t,e){const n=this.geometry,i=this.matrixWorld,s=t.params.Line.threshold,a=n.drawRange;if(n.boundingSphere===null&&n.computeBoundingSphere(),Ca.copy(n.boundingSphere),Ca.applyMatrix4(i),Ca.radius+=s,t.ray.intersectsSphere(Ca)===!1)return;df.copy(i).invert(),Cl.copy(t.ray).applyMatrix4(df);const o=s/((this.scale.x+this.scale.y+this.scale.z)/3),c=o*o,l=new b,h=new b,u=new b,f=new b,d=this.isLineSegments?2:1,g=n.index,m=n.attributes.position;if(g!==null){const p=Math.max(0,a.start),_=Math.min(g.count,a.start+a.count);for(let v=p,S=_-1;v<S;v+=d){const y=g.getX(v),M=g.getX(v+1);if(l.fromBufferAttribute(m,y),h.fromBufferAttribute(m,M),Cl.distanceSqToSegment(l,h,f,u)>c)continue;f.applyMatrix4(this.matrixWorld);const T=t.ray.origin.distanceTo(f);T<t.near||T>t.far||e.push({distance:T,point:u.clone().applyMatrix4(this.matrixWorld),index:v,face:null,faceIndex:null,object:this})}}else{const p=Math.max(0,a.start),_=Math.min(m.count,a.start+a.count);for(let v=p,S=_-1;v<S;v+=d){if(l.fromBufferAttribute(m,v),h.fromBufferAttribute(m,v+1),Cl.distanceSqToSegment(l,h,f,u)>c)continue;f.applyMatrix4(this.matrixWorld);const M=t.ray.origin.distanceTo(f);M<t.near||M>t.far||e.push({distance:M,point:u.clone().applyMatrix4(this.matrixWorld),index:v,face:null,faceIndex:null,object:this})}}}updateMorphTargets(){const e=this.geometry.morphAttributes,n=Object.keys(e);if(n.length>0){const i=e[n[0]];if(i!==void 0){this.morphTargetInfluences=[],this.morphTargetDictionary={};for(let s=0,a=i.length;s<a;s++){const o=i[s].name||String(s);this.morphTargetInfluences.push(0),this.morphTargetDictionary[o]=s}}}}}const ff=new b,pf=new b;class ds extends HM{constructor(t,e){super(t,e),this.isLineSegments=!0,this.type="LineSegments"}computeLineDistances(){const t=this.geometry;if(t.index===null){const e=t.attributes.position,n=[];for(let i=0,s=e.count;i<s;i+=2)ff.fromBufferAttribute(e,i),pf.fromBufferAttribute(e,i+1),n[i]=i===0?0:n[i-1],n[i+1]=n[i]+ff.distanceTo(pf);t.setAttribute("lineDistance",new ie(n,1))}else console.warn("THREE.LineSegments.computeLineDistances(): Computation only possible with non-indexed BufferGeometry.");return this}}class Io extends Si{constructor(t){super(),this.isPointsMaterial=!0,this.type="PointsMaterial",this.color=new jt(16777215),this.map=null,this.alphaMap=null,this.size=1,this.sizeAttenuation=!0,this.fog=!0,this.setValues(t)}copy(t){return super.copy(t),this.color.copy(t.color),this.map=t.map,this.alphaMap=t.alphaMap,this.size=t.size,this.sizeAttenuation=t.sizeAttenuation,this.fog=t.fog,this}}const mf=new se,Dh=new pu,Ra=new ms,Pa=new b;class Rl extends Qe{constructor(t=new ye,e=new Io){super(),this.isPoints=!0,this.type="Points",this.geometry=t,this.material=e,this.updateMorphTargets()}copy(t,e){return super.copy(t,e),this.material=Array.isArray(t.material)?t.material.slice():t.material,this.geometry=t.geometry,this}raycast(t,e){const n=this.geometry,i=this.matrixWorld,s=t.params.Points.threshold,a=n.drawRange;if(n.boundingSphere===null&&n.computeBoundingSphere(),Ra.copy(n.boundingSphere),Ra.applyMatrix4(i),Ra.radius+=s,t.ray.intersectsSphere(Ra)===!1)return;mf.copy(i).invert(),Dh.copy(t.ray).applyMatrix4(mf);const o=s/((this.scale.x+this.scale.y+this.scale.z)/3),c=o*o,l=n.index,u=n.attributes.position;if(l!==null){const f=Math.max(0,a.start),d=Math.min(l.count,a.start+a.count);for(let g=f,x=d;g<x;g++){const m=l.getX(g);Pa.fromBufferAttribute(u,m),gf(Pa,m,c,i,t,e,this)}}else{const f=Math.max(0,a.start),d=Math.min(u.count,a.start+a.count);for(let g=f,x=d;g<x;g++)Pa.fromBufferAttribute(u,g),gf(Pa,g,c,i,t,e,this)}}updateMorphTargets(){const e=this.geometry.morphAttributes,n=Object.keys(e);if(n.length>0){const i=e[n[0]];if(i!==void 0){this.morphTargetInfluences=[],this.morphTargetDictionary={};for(let s=0,a=i.length;s<a;s++){const o=i[s].name||String(s);this.morphTargetInfluences.push(0),this.morphTargetDictionary[o]=s}}}}}function gf(r,t,e,n,i,s,a){const o=Dh.distanceSqToPoint(r);if(o<e){const c=new b;Dh.closestPointToPoint(r,c),c.applyMatrix4(n);const l=i.ray.origin.distanceTo(c);if(l<i.near||l>i.far)return;s.push({distance:l,distanceToRay:Math.sqrt(o),point:c,index:t,face:null,object:a})}}class Mi{constructor(){this.type="Curve",this.arcLengthDivisions=200}getPoint(){return console.warn("THREE.Curve: .getPoint() not implemented."),null}getPointAt(t,e){const n=this.getUtoTmapping(t);return this.getPoint(n,e)}getPoints(t=5){const e=[];for(let n=0;n<=t;n++)e.push(this.getPoint(n/t));return e}getSpacedPoints(t=5){const e=[];for(let n=0;n<=t;n++)e.push(this.getPointAt(n/t));return e}getLength(){const t=this.getLengths();return t[t.length-1]}getLengths(t=this.arcLengthDivisions){if(this.cacheArcLengths&&this.cacheArcLengths.length===t+1&&!this.needsUpdate)return this.cacheArcLengths;this.needsUpdate=!1;const e=[];let n,i=this.getPoint(0),s=0;e.push(0);for(let a=1;a<=t;a++)n=this.getPoint(a/t),s+=n.distanceTo(i),e.push(s),i=n;return this.cacheArcLengths=e,e}updateArcLengths(){this.needsUpdate=!0,this.getLengths()}getUtoTmapping(t,e){const n=this.getLengths();let i=0;const s=n.length;let a;e?a=e:a=t*n[s-1];let o=0,c=s-1,l;for(;o<=c;)if(i=Math.floor(o+(c-o)/2),l=n[i]-a,l<0)o=i+1;else if(l>0)c=i-1;else{c=i;break}if(i=c,n[i]===a)return i/(s-1);const h=n[i],f=n[i+1]-h,d=(a-h)/f;return(i+d)/(s-1)}getTangent(t,e){let i=t-1e-4,s=t+1e-4;i<0&&(i=0),s>1&&(s=1);const a=this.getPoint(i),o=this.getPoint(s),c=e||(a.isVector2?new St:new b);return c.copy(o).sub(a).normalize(),c}getTangentAt(t,e){const n=this.getUtoTmapping(t);return this.getTangent(n,e)}computeFrenetFrames(t,e){const n=new b,i=[],s=[],a=[],o=new b,c=new se;for(let d=0;d<=t;d++){const g=d/t;i[d]=this.getTangentAt(g,new b)}s[0]=new b,a[0]=new b;let l=Number.MAX_VALUE;const h=Math.abs(i[0].x),u=Math.abs(i[0].y),f=Math.abs(i[0].z);h<=l&&(l=h,n.set(1,0,0)),u<=l&&(l=u,n.set(0,1,0)),f<=l&&n.set(0,0,1),o.crossVectors(i[0],n).normalize(),s[0].crossVectors(i[0],o),a[0].crossVectors(i[0],s[0]);for(let d=1;d<=t;d++){if(s[d]=s[d-1].clone(),a[d]=a[d-1].clone(),o.crossVectors(i[d-1],i[d]),o.length()>Number.EPSILON){o.normalize();const g=Math.acos(Ge(i[d-1].dot(i[d]),-1,1));s[d].applyMatrix4(c.makeRotationAxis(o,g))}a[d].crossVectors(i[d],s[d])}if(e===!0){let d=Math.acos(Ge(s[0].dot(s[t]),-1,1));d/=t,i[0].dot(o.crossVectors(s[0],s[t]))>0&&(d=-d);for(let g=1;g<=t;g++)s[g].applyMatrix4(c.makeRotationAxis(i[g],d*g)),a[g].crossVectors(i[g],s[g])}return{tangents:i,normals:s,binormals:a}}clone(){return new this.constructor().copy(this)}copy(t){return this.arcLengthDivisions=t.arcLengthDivisions,this}toJSON(){const t={metadata:{version:4.6,type:"Curve",generator:"Curve.toJSON"}};return t.arcLengthDivisions=this.arcLengthDivisions,t.type=this.type,t}fromJSON(t){return this.arcLengthDivisions=t.arcLengthDivisions,this}}class vu extends Mi{constructor(t=0,e=0,n=1,i=1,s=0,a=Math.PI*2,o=!1,c=0){super(),this.isEllipseCurve=!0,this.type="EllipseCurve",this.aX=t,this.aY=e,this.xRadius=n,this.yRadius=i,this.aStartAngle=s,this.aEndAngle=a,this.aClockwise=o,this.aRotation=c}getPoint(t,e){const n=e||new St,i=Math.PI*2;let s=this.aEndAngle-this.aStartAngle;const a=Math.abs(s)<Number.EPSILON;for(;s<0;)s+=i;for(;s>i;)s-=i;s<Number.EPSILON&&(a?s=0:s=i),this.aClockwise===!0&&!a&&(s===i?s=-i:s=s-i);const o=this.aStartAngle+t*s;let c=this.aX+this.xRadius*Math.cos(o),l=this.aY+this.yRadius*Math.sin(o);if(this.aRotation!==0){const h=Math.cos(this.aRotation),u=Math.sin(this.aRotation),f=c-this.aX,d=l-this.aY;c=f*h-d*u+this.aX,l=f*u+d*h+this.aY}return n.set(c,l)}copy(t){return super.copy(t),this.aX=t.aX,this.aY=t.aY,this.xRadius=t.xRadius,this.yRadius=t.yRadius,this.aStartAngle=t.aStartAngle,this.aEndAngle=t.aEndAngle,this.aClockwise=t.aClockwise,this.aRotation=t.aRotation,this}toJSON(){const t=super.toJSON();return t.aX=this.aX,t.aY=this.aY,t.xRadius=this.xRadius,t.yRadius=this.yRadius,t.aStartAngle=this.aStartAngle,t.aEndAngle=this.aEndAngle,t.aClockwise=this.aClockwise,t.aRotation=this.aRotation,t}fromJSON(t){return super.fromJSON(t),this.aX=t.aX,this.aY=t.aY,this.xRadius=t.xRadius,this.yRadius=t.yRadius,this.aStartAngle=t.aStartAngle,this.aEndAngle=t.aEndAngle,this.aClockwise=t.aClockwise,this.aRotation=t.aRotation,this}}class WM extends vu{constructor(t,e,n,i,s,a){super(t,e,n,n,i,s,a),this.isArcCurve=!0,this.type="ArcCurve"}}function Su(){let r=0,t=0,e=0,n=0;function i(s,a,o,c){r=s,t=o,e=-3*s+3*a-2*o-c,n=2*s-2*a+o+c}return{initCatmullRom:function(s,a,o,c,l){i(a,o,l*(o-s),l*(c-a))},initNonuniformCatmullRom:function(s,a,o,c,l,h,u){let f=(a-s)/l-(o-s)/(l+h)+(o-a)/h,d=(o-a)/h-(c-a)/(h+u)+(c-o)/u;f*=h,d*=h,i(a,o,f,d)},calc:function(s){const a=s*s,o=a*s;return r+t*s+e*a+n*o}}}const La=new b,Pl=new Su,Ll=new Su,Il=new Su;class Em extends Mi{constructor(t=[],e=!1,n="centripetal",i=.5){super(),this.isCatmullRomCurve3=!0,this.type="CatmullRomCurve3",this.points=t,this.closed=e,this.curveType=n,this.tension=i}getPoint(t,e=new b){const n=e,i=this.points,s=i.length,a=(s-(this.closed?0:1))*t;let o=Math.floor(a),c=a-o;this.closed?o+=o>0?0:(Math.floor(Math.abs(o)/s)+1)*s:c===0&&o===s-1&&(o=s-2,c=1);let l,h;this.closed||o>0?l=i[(o-1)%s]:(La.subVectors(i[0],i[1]).add(i[0]),l=La);const u=i[o%s],f=i[(o+1)%s];if(this.closed||o+2<s?h=i[(o+2)%s]:(La.subVectors(i[s-1],i[s-2]).add(i[s-1]),h=La),this.curveType==="centripetal"||this.curveType==="chordal"){const d=this.curveType==="chordal"?.5:.25;let g=Math.pow(l.distanceToSquared(u),d),x=Math.pow(u.distanceToSquared(f),d),m=Math.pow(f.distanceToSquared(h),d);x<1e-4&&(x=1),g<1e-4&&(g=x),m<1e-4&&(m=x),Pl.initNonuniformCatmullRom(l.x,u.x,f.x,h.x,g,x,m),Ll.initNonuniformCatmullRom(l.y,u.y,f.y,h.y,g,x,m),Il.initNonuniformCatmullRom(l.z,u.z,f.z,h.z,g,x,m)}else this.curveType==="catmullrom"&&(Pl.initCatmullRom(l.x,u.x,f.x,h.x,this.tension),Ll.initCatmullRom(l.y,u.y,f.y,h.y,this.tension),Il.initCatmullRom(l.z,u.z,f.z,h.z,this.tension));return n.set(Pl.calc(c),Ll.calc(c),Il.calc(c)),n}copy(t){super.copy(t),this.points=[];for(let e=0,n=t.points.length;e<n;e++){const i=t.points[e];this.points.push(i.clone())}return this.closed=t.closed,this.curveType=t.curveType,this.tension=t.tension,this}toJSON(){const t=super.toJSON();t.points=[];for(let e=0,n=this.points.length;e<n;e++){const i=this.points[e];t.points.push(i.toArray())}return t.closed=this.closed,t.curveType=this.curveType,t.tension=this.tension,t}fromJSON(t){super.fromJSON(t),this.points=[];for(let e=0,n=t.points.length;e<n;e++){const i=t.points[e];this.points.push(new b().fromArray(i))}return this.closed=t.closed,this.curveType=t.curveType,this.tension=t.tension,this}}function xf(r,t,e,n,i){const s=(n-t)*.5,a=(i-e)*.5,o=r*r,c=r*o;return(2*e-2*n+s+a)*c+(-3*e+3*n-2*s-a)*o+s*r+e}function XM(r,t){const e=1-r;return e*e*t}function YM(r,t){return 2*(1-r)*r*t}function qM(r,t){return r*r*t}function ko(r,t,e,n){return XM(r,t)+YM(r,e)+qM(r,n)}function ZM(r,t){const e=1-r;return e*e*e*t}function jM(r,t){const e=1-r;return 3*e*e*r*t}function $M(r,t){return 3*(1-r)*r*r*t}function KM(r,t){return r*r*r*t}function Ho(r,t,e,n,i){return ZM(r,t)+jM(r,e)+$M(r,n)+KM(r,i)}class wm extends Mi{constructor(t=new St,e=new St,n=new St,i=new St){super(),this.isCubicBezierCurve=!0,this.type="CubicBezierCurve",this.v0=t,this.v1=e,this.v2=n,this.v3=i}getPoint(t,e=new St){const n=e,i=this.v0,s=this.v1,a=this.v2,o=this.v3;return n.set(Ho(t,i.x,s.x,a.x,o.x),Ho(t,i.y,s.y,a.y,o.y)),n}copy(t){return super.copy(t),this.v0.copy(t.v0),this.v1.copy(t.v1),this.v2.copy(t.v2),this.v3.copy(t.v3),this}toJSON(){const t=super.toJSON();return t.v0=this.v0.toArray(),t.v1=this.v1.toArray(),t.v2=this.v2.toArray(),t.v3=this.v3.toArray(),t}fromJSON(t){return super.fromJSON(t),this.v0.fromArray(t.v0),this.v1.fromArray(t.v1),this.v2.fromArray(t.v2),this.v3.fromArray(t.v3),this}}class JM extends Mi{constructor(t=new b,e=new b,n=new b,i=new b){super(),this.isCubicBezierCurve3=!0,this.type="CubicBezierCurve3",this.v0=t,this.v1=e,this.v2=n,this.v3=i}getPoint(t,e=new b){const n=e,i=this.v0,s=this.v1,a=this.v2,o=this.v3;return n.set(Ho(t,i.x,s.x,a.x,o.x),Ho(t,i.y,s.y,a.y,o.y),Ho(t,i.z,s.z,a.z,o.z)),n}copy(t){return super.copy(t),this.v0.copy(t.v0),this.v1.copy(t.v1),this.v2.copy(t.v2),this.v3.copy(t.v3),this}toJSON(){const t=super.toJSON();return t.v0=this.v0.toArray(),t.v1=this.v1.toArray(),t.v2=this.v2.toArray(),t.v3=this.v3.toArray(),t}fromJSON(t){return super.fromJSON(t),this.v0.fromArray(t.v0),this.v1.fromArray(t.v1),this.v2.fromArray(t.v2),this.v3.fromArray(t.v3),this}}class Am extends Mi{constructor(t=new St,e=new St){super(),this.isLineCurve=!0,this.type="LineCurve",this.v1=t,this.v2=e}getPoint(t,e=new St){const n=e;return t===1?n.copy(this.v2):(n.copy(this.v2).sub(this.v1),n.multiplyScalar(t).add(this.v1)),n}getPointAt(t,e){return this.getPoint(t,e)}getTangent(t,e=new St){return e.subVectors(this.v2,this.v1).normalize()}getTangentAt(t,e){return this.getTangent(t,e)}copy(t){return super.copy(t),this.v1.copy(t.v1),this.v2.copy(t.v2),this}toJSON(){const t=super.toJSON();return t.v1=this.v1.toArray(),t.v2=this.v2.toArray(),t}fromJSON(t){return super.fromJSON(t),this.v1.fromArray(t.v1),this.v2.fromArray(t.v2),this}}class QM extends Mi{constructor(t=new b,e=new b){super(),this.isLineCurve3=!0,this.type="LineCurve3",this.v1=t,this.v2=e}getPoint(t,e=new b){const n=e;return t===1?n.copy(this.v2):(n.copy(this.v2).sub(this.v1),n.multiplyScalar(t).add(this.v1)),n}getPointAt(t,e){return this.getPoint(t,e)}getTangent(t,e=new b){return e.subVectors(this.v2,this.v1).normalize()}getTangentAt(t,e){return this.getTangent(t,e)}copy(t){return super.copy(t),this.v1.copy(t.v1),this.v2.copy(t.v2),this}toJSON(){const t=super.toJSON();return t.v1=this.v1.toArray(),t.v2=this.v2.toArray(),t}fromJSON(t){return super.fromJSON(t),this.v1.fromArray(t.v1),this.v2.fromArray(t.v2),this}}class Tm extends Mi{constructor(t=new St,e=new St,n=new St){super(),this.isQuadraticBezierCurve=!0,this.type="QuadraticBezierCurve",this.v0=t,this.v1=e,this.v2=n}getPoint(t,e=new St){const n=e,i=this.v0,s=this.v1,a=this.v2;return n.set(ko(t,i.x,s.x,a.x),ko(t,i.y,s.y,a.y)),n}copy(t){return super.copy(t),this.v0.copy(t.v0),this.v1.copy(t.v1),this.v2.copy(t.v2),this}toJSON(){const t=super.toJSON();return t.v0=this.v0.toArray(),t.v1=this.v1.toArray(),t.v2=this.v2.toArray(),t}fromJSON(t){return super.fromJSON(t),this.v0.fromArray(t.v0),this.v1.fromArray(t.v1),this.v2.fromArray(t.v2),this}}class bm extends Mi{constructor(t=new b,e=new b,n=new b){super(),this.isQuadraticBezierCurve3=!0,this.type="QuadraticBezierCurve3",this.v0=t,this.v1=e,this.v2=n}getPoint(t,e=new b){const n=e,i=this.v0,s=this.v1,a=this.v2;return n.set(ko(t,i.x,s.x,a.x),ko(t,i.y,s.y,a.y),ko(t,i.z,s.z,a.z)),n}copy(t){return super.copy(t),this.v0.copy(t.v0),this.v1.copy(t.v1),this.v2.copy(t.v2),this}toJSON(){const t=super.toJSON();return t.v0=this.v0.toArray(),t.v1=this.v1.toArray(),t.v2=this.v2.toArray(),t}fromJSON(t){return super.fromJSON(t),this.v0.fromArray(t.v0),this.v1.fromArray(t.v1),this.v2.fromArray(t.v2),this}}class Cm extends Mi{constructor(t=[]){super(),this.isSplineCurve=!0,this.type="SplineCurve",this.points=t}getPoint(t,e=new St){const n=e,i=this.points,s=(i.length-1)*t,a=Math.floor(s),o=s-a,c=i[a===0?a:a-1],l=i[a],h=i[a>i.length-2?i.length-1:a+1],u=i[a>i.length-3?i.length-1:a+2];return n.set(xf(o,c.x,l.x,h.x,u.x),xf(o,c.y,l.y,h.y,u.y)),n}copy(t){super.copy(t),this.points=[];for(let e=0,n=t.points.length;e<n;e++){const i=t.points[e];this.points.push(i.clone())}return this}toJSON(){const t=super.toJSON();t.points=[];for(let e=0,n=this.points.length;e<n;e++){const i=this.points[e];t.points.push(i.toArray())}return t}fromJSON(t){super.fromJSON(t),this.points=[];for(let e=0,n=t.points.length;e<n;e++){const i=t.points[e];this.points.push(new St().fromArray(i))}return this}}var Nh=Object.freeze({__proto__:null,ArcCurve:WM,CatmullRomCurve3:Em,CubicBezierCurve:wm,CubicBezierCurve3:JM,EllipseCurve:vu,LineCurve:Am,LineCurve3:QM,QuadraticBezierCurve:Tm,QuadraticBezierCurve3:bm,SplineCurve:Cm});class ty extends Mi{constructor(){super(),this.type="CurvePath",this.curves=[],this.autoClose=!1}add(t){this.curves.push(t)}closePath(){const t=this.curves[0].getPoint(0),e=this.curves[this.curves.length-1].getPoint(1);if(!t.equals(e)){const n=t.isVector2===!0?"LineCurve":"LineCurve3";this.curves.push(new Nh[n](e,t))}return this}getPoint(t,e){const n=t*this.getLength(),i=this.getCurveLengths();let s=0;for(;s<i.length;){if(i[s]>=n){const a=i[s]-n,o=this.curves[s],c=o.getLength(),l=c===0?0:1-a/c;return o.getPointAt(l,e)}s++}return null}getLength(){const t=this.getCurveLengths();return t[t.length-1]}updateArcLengths(){this.needsUpdate=!0,this.cacheLengths=null,this.getCurveLengths()}getCurveLengths(){if(this.cacheLengths&&this.cacheLengths.length===this.curves.length)return this.cacheLengths;const t=[];let e=0;for(let n=0,i=this.curves.length;n<i;n++)e+=this.curves[n].getLength(),t.push(e);return this.cacheLengths=t,t}getSpacedPoints(t=40){const e=[];for(let n=0;n<=t;n++)e.push(this.getPoint(n/t));return this.autoClose&&e.push(e[0]),e}getPoints(t=12){const e=[];let n;for(let i=0,s=this.curves;i<s.length;i++){const a=s[i],o=a.isEllipseCurve?t*2:a.isLineCurve||a.isLineCurve3?1:a.isSplineCurve?t*a.points.length:t,c=a.getPoints(o);for(let l=0;l<c.length;l++){const h=c[l];n&&n.equals(h)||(e.push(h),n=h)}}return this.autoClose&&e.length>1&&!e[e.length-1].equals(e[0])&&e.push(e[0]),e}copy(t){super.copy(t),this.curves=[];for(let e=0,n=t.curves.length;e<n;e++){const i=t.curves[e];this.curves.push(i.clone())}return this.autoClose=t.autoClose,this}toJSON(){const t=super.toJSON();t.autoClose=this.autoClose,t.curves=[];for(let e=0,n=this.curves.length;e<n;e++){const i=this.curves[e];t.curves.push(i.toJSON())}return t}fromJSON(t){super.fromJSON(t),this.autoClose=t.autoClose,this.curves=[];for(let e=0,n=t.curves.length;e<n;e++){const i=t.curves[e];this.curves.push(new Nh[i.type]().fromJSON(i))}return this}}class ey extends ty{constructor(t){super(),this.type="Path",this.currentPoint=new St,t&&this.setFromPoints(t)}setFromPoints(t){this.moveTo(t[0].x,t[0].y);for(let e=1,n=t.length;e<n;e++)this.lineTo(t[e].x,t[e].y);return this}moveTo(t,e){return this.currentPoint.set(t,e),this}lineTo(t,e){const n=new Am(this.currentPoint.clone(),new St(t,e));return this.curves.push(n),this.currentPoint.set(t,e),this}quadraticCurveTo(t,e,n,i){const s=new Tm(this.currentPoint.clone(),new St(t,e),new St(n,i));return this.curves.push(s),this.currentPoint.set(n,i),this}bezierCurveTo(t,e,n,i,s,a){const o=new wm(this.currentPoint.clone(),new St(t,e),new St(n,i),new St(s,a));return this.curves.push(o),this.currentPoint.set(s,a),this}splineThru(t){const e=[this.currentPoint.clone()].concat(t),n=new Cm(e);return this.curves.push(n),this.currentPoint.copy(t[t.length-1]),this}arc(t,e,n,i,s,a){const o=this.currentPoint.x,c=this.currentPoint.y;return this.absarc(t+o,e+c,n,i,s,a),this}absarc(t,e,n,i,s,a){return this.absellipse(t,e,n,n,i,s,a),this}ellipse(t,e,n,i,s,a,o,c){const l=this.currentPoint.x,h=this.currentPoint.y;return this.absellipse(t+l,e+h,n,i,s,a,o,c),this}absellipse(t,e,n,i,s,a,o,c){const l=new vu(t,e,n,i,s,a,o,c);if(this.curves.length>0){const u=l.getPoint(0);u.equals(this.currentPoint)||this.lineTo(u.x,u.y)}this.curves.push(l);const h=l.getPoint(1);return this.currentPoint.copy(h),this}copy(t){return super.copy(t),this.currentPoint.copy(t.currentPoint),this}toJSON(){const t=super.toJSON();return t.currentPoint=this.currentPoint.toArray(),t}fromJSON(t){return super.fromJSON(t),this.currentPoint.fromArray(t.currentPoint),this}}class Mu extends ye{constructor(t=[new St(0,-.5),new St(.5,0),new St(0,.5)],e=12,n=0,i=Math.PI*2){super(),this.type="LatheGeometry",this.parameters={points:t,segments:e,phiStart:n,phiLength:i},e=Math.floor(e),i=Ge(i,0,Math.PI*2);const s=[],a=[],o=[],c=[],l=[],h=1/e,u=new b,f=new St,d=new b,g=new b,x=new b;let m=0,p=0;for(let _=0;_<=t.length-1;_++)switch(_){case 0:m=t[_+1].x-t[_].x,p=t[_+1].y-t[_].y,d.x=p*1,d.y=-m,d.z=p*0,x.copy(d),d.normalize(),c.push(d.x,d.y,d.z);break;case t.length-1:c.push(x.x,x.y,x.z);break;default:m=t[_+1].x-t[_].x,p=t[_+1].y-t[_].y,d.x=p*1,d.y=-m,d.z=p*0,g.copy(d),d.x+=x.x,d.y+=x.y,d.z+=x.z,d.normalize(),c.push(d.x,d.y,d.z),x.copy(g)}for(let _=0;_<=e;_++){const v=n+_*h*i,S=Math.sin(v),y=Math.cos(v);for(let M=0;M<=t.length-1;M++){u.x=t[M].x*S,u.y=t[M].y,u.z=t[M].x*y,a.push(u.x,u.y,u.z),f.x=_/e,f.y=M/(t.length-1),o.push(f.x,f.y);const w=c[3*M+0]*S,T=c[3*M+1],E=c[3*M+0]*y;l.push(w,T,E)}}for(let _=0;_<e;_++)for(let v=0;v<t.length-1;v++){const S=v+_*t.length,y=S,M=S+t.length,w=S+t.length+1,T=S+1;s.push(y,M,T),s.push(w,T,M)}this.setIndex(s),this.setAttribute("position",new ie(a,3)),this.setAttribute("uv",new ie(o,2)),this.setAttribute("normal",new ie(l,3))}copy(t){return super.copy(t),this.parameters=Object.assign({},t.parameters),this}static fromJSON(t){return new Mu(t.points,t.segments,t.phiStart,t.phiLength)}}class Cr extends Mu{constructor(t=1,e=1,n=4,i=8){const s=new ey;s.absarc(0,-e/2,t,Math.PI*1.5,0),s.absarc(0,e/2,t,0,Math.PI*.5),super(s.getPoints(n),i),this.type="CapsuleGeometry",this.parameters={radius:t,length:e,capSegments:n,radialSegments:i}}static fromJSON(t){return new Cr(t.radius,t.length,t.capSegments,t.radialSegments)}}class gs extends ye{constructor(t=1,e=1,n=1,i=32,s=1,a=!1,o=0,c=Math.PI*2){super(),this.type="CylinderGeometry",this.parameters={radiusTop:t,radiusBottom:e,height:n,radialSegments:i,heightSegments:s,openEnded:a,thetaStart:o,thetaLength:c};const l=this;i=Math.floor(i),s=Math.floor(s);const h=[],u=[],f=[],d=[];let g=0;const x=[],m=n/2;let p=0;_(),a===!1&&(t>0&&v(!0),e>0&&v(!1)),this.setIndex(h),this.setAttribute("position",new ie(u,3)),this.setAttribute("normal",new ie(f,3)),this.setAttribute("uv",new ie(d,2));function _(){const S=new b,y=new b;let M=0;const w=(e-t)/n;for(let T=0;T<=s;T++){const E=[],A=T/s,D=A*(e-t)+t;for(let R=0;R<=i;R++){const N=R/i,L=N*c+o,F=Math.sin(L),z=Math.cos(L);y.x=D*F,y.y=-A*n+m,y.z=D*z,u.push(y.x,y.y,y.z),S.set(F,w,z).normalize(),f.push(S.x,S.y,S.z),d.push(N,1-A),E.push(g++)}x.push(E)}for(let T=0;T<i;T++)for(let E=0;E<s;E++){const A=x[E][T],D=x[E+1][T],R=x[E+1][T+1],N=x[E][T+1];h.push(A,D,N),h.push(D,R,N),M+=6}l.addGroup(p,M,0),p+=M}function v(S){const y=g,M=new St,w=new b;let T=0;const E=S===!0?t:e,A=S===!0?1:-1;for(let R=1;R<=i;R++)u.push(0,m*A,0),f.push(0,A,0),d.push(.5,.5),g++;const D=g;for(let R=0;R<=i;R++){const L=R/i*c+o,F=Math.cos(L),z=Math.sin(L);w.x=E*z,w.y=m*A,w.z=E*F,u.push(w.x,w.y,w.z),f.push(0,A,0),M.x=F*.5+.5,M.y=z*.5*A+.5,d.push(M.x,M.y),g++}for(let R=0;R<i;R++){const N=y+R,L=D+R;S===!0?h.push(L,L+1,N):h.push(L+1,L,N),T+=3}l.addGroup(p,T,S===!0?1:2),p+=T}}copy(t){return super.copy(t),this.parameters=Object.assign({},t.parameters),this}static fromJSON(t){return new gs(t.radiusTop,t.radiusBottom,t.height,t.radialSegments,t.heightSegments,t.openEnded,t.thetaStart,t.thetaLength)}}class yu extends ye{constructor(t=.5,e=1,n=32,i=1,s=0,a=Math.PI*2){super(),this.type="RingGeometry",this.parameters={innerRadius:t,outerRadius:e,thetaSegments:n,phiSegments:i,thetaStart:s,thetaLength:a},n=Math.max(3,n),i=Math.max(1,i);const o=[],c=[],l=[],h=[];let u=t;const f=(e-t)/i,d=new b,g=new St;for(let x=0;x<=i;x++){for(let m=0;m<=n;m++){const p=s+m/n*a;d.x=u*Math.cos(p),d.y=u*Math.sin(p),c.push(d.x,d.y,d.z),l.push(0,0,1),g.x=(d.x/e+1)/2,g.y=(d.y/e+1)/2,h.push(g.x,g.y)}u+=f}for(let x=0;x<i;x++){const m=x*(n+1);for(let p=0;p<n;p++){const _=p+m,v=_,S=_+n+1,y=_+n+2,M=_+1;o.push(v,S,M),o.push(S,y,M)}}this.setIndex(o),this.setAttribute("position",new ie(c,3)),this.setAttribute("normal",new ie(l,3)),this.setAttribute("uv",new ie(h,2))}copy(t){return super.copy(t),this.parameters=Object.assign({},t.parameters),this}static fromJSON(t){return new yu(t.innerRadius,t.outerRadius,t.thetaSegments,t.phiSegments,t.thetaStart,t.thetaLength)}}class ai extends ye{constructor(t=1,e=32,n=16,i=0,s=Math.PI*2,a=0,o=Math.PI){super(),this.type="SphereGeometry",this.parameters={radius:t,widthSegments:e,heightSegments:n,phiStart:i,phiLength:s,thetaStart:a,thetaLength:o},e=Math.max(3,Math.floor(e)),n=Math.max(2,Math.floor(n));const c=Math.min(a+o,Math.PI);let l=0;const h=[],u=new b,f=new b,d=[],g=[],x=[],m=[];for(let p=0;p<=n;p++){const _=[],v=p/n;let S=0;p===0&&a===0?S=.5/e:p===n&&c===Math.PI&&(S=-.5/e);for(let y=0;y<=e;y++){const M=y/e;u.x=-t*Math.cos(i+M*s)*Math.sin(a+v*o),u.y=t*Math.cos(a+v*o),u.z=t*Math.sin(i+M*s)*Math.sin(a+v*o),g.push(u.x,u.y,u.z),f.copy(u).normalize(),x.push(f.x,f.y,f.z),m.push(M+S,1-v),_.push(l++)}h.push(_)}for(let p=0;p<n;p++)for(let _=0;_<e;_++){const v=h[p][_+1],S=h[p][_],y=h[p+1][_],M=h[p+1][_+1];(p!==0||a>0)&&d.push(v,S,M),(p!==n-1||c<Math.PI)&&d.push(S,y,M)}this.setIndex(d),this.setAttribute("position",new ie(g,3)),this.setAttribute("normal",new ie(x,3)),this.setAttribute("uv",new ie(m,2))}copy(t){return super.copy(t),this.parameters=Object.assign({},t.parameters),this}static fromJSON(t){return new ai(t.radius,t.widthSegments,t.heightSegments,t.phiStart,t.phiLength,t.thetaStart,t.thetaLength)}}class Eu extends ye{constructor(t=1,e=.4,n=12,i=48,s=Math.PI*2){super(),this.type="TorusGeometry",this.parameters={radius:t,tube:e,radialSegments:n,tubularSegments:i,arc:s},n=Math.floor(n),i=Math.floor(i);const a=[],o=[],c=[],l=[],h=new b,u=new b,f=new b;for(let d=0;d<=n;d++)for(let g=0;g<=i;g++){const x=g/i*s,m=d/n*Math.PI*2;u.x=(t+e*Math.cos(m))*Math.cos(x),u.y=(t+e*Math.cos(m))*Math.sin(x),u.z=e*Math.sin(m),o.push(u.x,u.y,u.z),h.x=t*Math.cos(x),h.y=t*Math.sin(x),f.subVectors(u,h).normalize(),c.push(f.x,f.y,f.z),l.push(g/i),l.push(d/n)}for(let d=1;d<=n;d++)for(let g=1;g<=i;g++){const x=(i+1)*d+g-1,m=(i+1)*(d-1)+g-1,p=(i+1)*(d-1)+g,_=(i+1)*d+g;a.push(x,m,_),a.push(m,p,_)}this.setIndex(a),this.setAttribute("position",new ie(o,3)),this.setAttribute("normal",new ie(c,3)),this.setAttribute("uv",new ie(l,2))}copy(t){return super.copy(t),this.parameters=Object.assign({},t.parameters),this}static fromJSON(t){return new Eu(t.radius,t.tube,t.radialSegments,t.tubularSegments,t.arc)}}class wu extends ye{constructor(t=new bm(new b(-1,-1,0),new b(-1,1,0),new b(1,1,0)),e=64,n=1,i=8,s=!1){super(),this.type="TubeGeometry",this.parameters={path:t,tubularSegments:e,radius:n,radialSegments:i,closed:s};const a=t.computeFrenetFrames(e,s);this.tangents=a.tangents,this.normals=a.normals,this.binormals=a.binormals;const o=new b,c=new b,l=new St;let h=new b;const u=[],f=[],d=[],g=[];x(),this.setIndex(g),this.setAttribute("position",new ie(u,3)),this.setAttribute("normal",new ie(f,3)),this.setAttribute("uv",new ie(d,2));function x(){for(let v=0;v<e;v++)m(v);m(s===!1?e:0),_(),p()}function m(v){h=t.getPointAt(v/e,h);const S=a.normals[v],y=a.binormals[v];for(let M=0;M<=i;M++){const w=M/i*Math.PI*2,T=Math.sin(w),E=-Math.cos(w);c.x=E*S.x+T*y.x,c.y=E*S.y+T*y.y,c.z=E*S.z+T*y.z,c.normalize(),f.push(c.x,c.y,c.z),o.x=h.x+n*c.x,o.y=h.y+n*c.y,o.z=h.z+n*c.z,u.push(o.x,o.y,o.z)}}function p(){for(let v=1;v<=e;v++)for(let S=1;S<=i;S++){const y=(i+1)*(v-1)+(S-1),M=(i+1)*v+(S-1),w=(i+1)*v+S,T=(i+1)*(v-1)+S;g.push(y,M,T),g.push(M,w,T)}}function _(){for(let v=0;v<=e;v++)for(let S=0;S<=i;S++)l.x=v/e,l.y=S/i,d.push(l.x,l.y)}}copy(t){return super.copy(t),this.parameters=Object.assign({},t.parameters),this}toJSON(){const t=super.toJSON();return t.path=this.parameters.path.toJSON(),t}static fromJSON(t){return new wu(new Nh[t.path.type]().fromJSON(t.path),t.tubularSegments,t.radius,t.radialSegments,t.closed)}}class En extends Si{constructor(t){super(),this.isMeshStandardMaterial=!0,this.defines={STANDARD:""},this.type="MeshStandardMaterial",this.color=new jt(16777215),this.roughness=1,this.metalness=0,this.map=null,this.lightMap=null,this.lightMapIntensity=1,this.aoMap=null,this.aoMapIntensity=1,this.emissive=new jt(0),this.emissiveIntensity=1,this.emissiveMap=null,this.bumpMap=null,this.bumpScale=1,this.normalMap=null,this.normalMapType=uu,this.normalScale=new St(1,1),this.displacementMap=null,this.displacementScale=1,this.displacementBias=0,this.roughnessMap=null,this.metalnessMap=null,this.alphaMap=null,this.envMap=null,this.envMapIntensity=1,this.wireframe=!1,this.wireframeLinewidth=1,this.wireframeLinecap="round",this.wireframeLinejoin="round",this.flatShading=!1,this.fog=!0,this.setValues(t)}copy(t){return super.copy(t),this.defines={STANDARD:""},this.color.copy(t.color),this.roughness=t.roughness,this.metalness=t.metalness,this.map=t.map,this.lightMap=t.lightMap,this.lightMapIntensity=t.lightMapIntensity,this.aoMap=t.aoMap,this.aoMapIntensity=t.aoMapIntensity,this.emissive.copy(t.emissive),this.emissiveMap=t.emissiveMap,this.emissiveIntensity=t.emissiveIntensity,this.bumpMap=t.bumpMap,this.bumpScale=t.bumpScale,this.normalMap=t.normalMap,this.normalMapType=t.normalMapType,this.normalScale.copy(t.normalScale),this.displacementMap=t.displacementMap,this.displacementScale=t.displacementScale,this.displacementBias=t.displacementBias,this.roughnessMap=t.roughnessMap,this.metalnessMap=t.metalnessMap,this.alphaMap=t.alphaMap,this.envMap=t.envMap,this.envMapIntensity=t.envMapIntensity,this.wireframe=t.wireframe,this.wireframeLinewidth=t.wireframeLinewidth,this.wireframeLinecap=t.wireframeLinecap,this.wireframeLinejoin=t.wireframeLinejoin,this.flatShading=t.flatShading,this.fog=t.fog,this}}class ny extends Si{constructor(t){super(),this.isMeshPhongMaterial=!0,this.type="MeshPhongMaterial",this.color=new jt(16777215),this.specular=new jt(1118481),this.shininess=30,this.map=null,this.lightMap=null,this.lightMapIntensity=1,this.aoMap=null,this.aoMapIntensity=1,this.emissive=new jt(0),this.emissiveIntensity=1,this.emissiveMap=null,this.bumpMap=null,this.bumpScale=1,this.normalMap=null,this.normalMapType=uu,this.normalScale=new St(1,1),this.displacementMap=null,this.displacementScale=1,this.displacementBias=0,this.specularMap=null,this.alphaMap=null,this.envMap=null,this.combine=lu,this.reflectivity=1,this.refractionRatio=.98,this.wireframe=!1,this.wireframeLinewidth=1,this.wireframeLinecap="round",this.wireframeLinejoin="round",this.flatShading=!1,this.fog=!0,this.setValues(t)}copy(t){return super.copy(t),this.color.copy(t.color),this.specular.copy(t.specular),this.shininess=t.shininess,this.map=t.map,this.lightMap=t.lightMap,this.lightMapIntensity=t.lightMapIntensity,this.aoMap=t.aoMap,this.aoMapIntensity=t.aoMapIntensity,this.emissive.copy(t.emissive),this.emissiveMap=t.emissiveMap,this.emissiveIntensity=t.emissiveIntensity,this.bumpMap=t.bumpMap,this.bumpScale=t.bumpScale,this.normalMap=t.normalMap,this.normalMapType=t.normalMapType,this.normalScale.copy(t.normalScale),this.displacementMap=t.displacementMap,this.displacementScale=t.displacementScale,this.displacementBias=t.displacementBias,this.specularMap=t.specularMap,this.alphaMap=t.alphaMap,this.envMap=t.envMap,this.combine=t.combine,this.reflectivity=t.reflectivity,this.refractionRatio=t.refractionRatio,this.wireframe=t.wireframe,this.wireframeLinewidth=t.wireframeLinewidth,this.wireframeLinecap=t.wireframeLinecap,this.wireframeLinejoin=t.wireframeLinejoin,this.flatShading=t.flatShading,this.fog=t.fog,this}}const _f={enabled:!1,files:{},add:function(r,t){this.enabled!==!1&&(this.files[r]=t)},get:function(r){if(this.enabled!==!1)return this.files[r]},remove:function(r){delete this.files[r]},clear:function(){this.files={}}};class iy{constructor(t,e,n){const i=this;let s=!1,a=0,o=0,c;const l=[];this.onStart=void 0,this.onLoad=t,this.onProgress=e,this.onError=n,this.itemStart=function(h){o++,s===!1&&i.onStart!==void 0&&i.onStart(h,a,o),s=!0},this.itemEnd=function(h){a++,i.onProgress!==void 0&&i.onProgress(h,a,o),a===o&&(s=!1,i.onLoad!==void 0&&i.onLoad())},this.itemError=function(h){i.onError!==void 0&&i.onError(h)},this.resolveURL=function(h){return c?c(h):h},this.setURLModifier=function(h){return c=h,this},this.addHandler=function(h,u){return l.push(h,u),this},this.removeHandler=function(h){const u=l.indexOf(h);return u!==-1&&l.splice(u,2),this},this.getHandler=function(h){for(let u=0,f=l.length;u<f;u+=2){const d=l[u],g=l[u+1];if(d.global&&(d.lastIndex=0),d.test(h))return g}return null}}}const sy=new iy;class Wc{constructor(t){this.manager=t!==void 0?t:sy,this.crossOrigin="anonymous",this.withCredentials=!1,this.path="",this.resourcePath="",this.requestHeader={}}load(){}loadAsync(t,e){const n=this;return new Promise(function(i,s){n.load(t,i,e,s)})}parse(){}setCrossOrigin(t){return this.crossOrigin=t,this}setWithCredentials(t){return this.withCredentials=t,this}setPath(t){return this.path=t,this}setResourcePath(t){return this.resourcePath=t,this}setRequestHeader(t){return this.requestHeader=t,this}}Wc.DEFAULT_MATERIAL_NAME="__DEFAULT";const Pi={};class ry extends Error{constructor(t,e){super(t),this.response=e}}class Rm extends Wc{constructor(t){super(t)}load(t,e,n,i){t===void 0&&(t=""),this.path!==void 0&&(t=this.path+t),t=this.manager.resolveURL(t);const s=_f.get(t);if(s!==void 0)return this.manager.itemStart(t),setTimeout(()=>{e&&e(s),this.manager.itemEnd(t)},0),s;if(Pi[t]!==void 0){Pi[t].push({onLoad:e,onProgress:n,onError:i});return}Pi[t]=[],Pi[t].push({onLoad:e,onProgress:n,onError:i});const a=new Request(t,{headers:new Headers(this.requestHeader),credentials:this.withCredentials?"include":"same-origin"}),o=this.mimeType,c=this.responseType;fetch(a).then(l=>{if(l.status===200||l.status===0){if(l.status===0&&console.warn("THREE.FileLoader: HTTP Status 0 received."),typeof ReadableStream>"u"||l.body===void 0||l.body.getReader===void 0)return l;const h=Pi[t],u=l.body.getReader(),f=l.headers.get("Content-Length")||l.headers.get("X-File-Size"),d=f?parseInt(f):0,g=d!==0;let x=0;const m=new ReadableStream({start(p){_();function _(){u.read().then(({done:v,value:S})=>{if(v)p.close();else{x+=S.byteLength;const y=new ProgressEvent("progress",{lengthComputable:g,loaded:x,total:d});for(let M=0,w=h.length;M<w;M++){const T=h[M];T.onProgress&&T.onProgress(y)}p.enqueue(S),_()}})}}});return new Response(m)}else throw new ry(`fetch for "${l.url}" responded with ${l.status}: ${l.statusText}`,l)}).then(l=>{switch(c){case"arraybuffer":return l.arrayBuffer();case"blob":return l.blob();case"document":return l.text().then(h=>new DOMParser().parseFromString(h,o));case"json":return l.json();default:if(o===void 0)return l.text();{const u=/charset="?([^;"\s]*)"?/i.exec(o),f=u&&u[1]?u[1].toLowerCase():void 0,d=new TextDecoder(f);return l.arrayBuffer().then(g=>d.decode(g))}}}).then(l=>{_f.add(t,l);const h=Pi[t];delete Pi[t];for(let u=0,f=h.length;u<f;u++){const d=h[u];d.onLoad&&d.onLoad(l)}}).catch(l=>{const h=Pi[t];if(h===void 0)throw this.manager.itemError(t),l;delete Pi[t];for(let u=0,f=h.length;u<f;u++){const d=h[u];d.onError&&d.onError(l)}this.manager.itemError(t)}).finally(()=>{this.manager.itemEnd(t)}),this.manager.itemStart(t)}setResponseType(t){return this.responseType=t,this}setMimeType(t){return this.mimeType=t,this}}class Pm extends Qe{constructor(t,e=1){super(),this.isLight=!0,this.type="Light",this.color=new jt(t),this.intensity=e}dispose(){}copy(t,e){return super.copy(t,e),this.color.copy(t.color),this.intensity=t.intensity,this}toJSON(t){const e=super.toJSON(t);return e.object.color=this.color.getHex(),e.object.intensity=this.intensity,this.groundColor!==void 0&&(e.object.groundColor=this.groundColor.getHex()),this.distance!==void 0&&(e.object.distance=this.distance),this.angle!==void 0&&(e.object.angle=this.angle),this.decay!==void 0&&(e.object.decay=this.decay),this.penumbra!==void 0&&(e.object.penumbra=this.penumbra),this.shadow!==void 0&&(e.object.shadow=this.shadow.toJSON()),e}}const Dl=new se,vf=new b,Sf=new b;class oy{constructor(t){this.camera=t,this.bias=0,this.normalBias=0,this.radius=1,this.blurSamples=8,this.mapSize=new St(512,512),this.map=null,this.mapPass=null,this.matrix=new se,this.autoUpdate=!0,this.needsUpdate=!1,this._frustum=new mu,this._frameExtents=new St(1,1),this._viewportCount=1,this._viewports=[new Je(0,0,1,1)]}getViewportCount(){return this._viewportCount}getFrustum(){return this._frustum}updateMatrices(t){const e=this.camera,n=this.matrix;vf.setFromMatrixPosition(t.matrixWorld),e.position.copy(vf),Sf.setFromMatrixPosition(t.target.matrixWorld),e.lookAt(Sf),e.updateMatrixWorld(),Dl.multiplyMatrices(e.projectionMatrix,e.matrixWorldInverse),this._frustum.setFromProjectionMatrix(Dl),n.set(.5,0,0,.5,0,.5,0,.5,0,0,.5,.5,0,0,0,1),n.multiply(Dl)}getViewport(t){return this._viewports[t]}getFrameExtents(){return this._frameExtents}dispose(){this.map&&this.map.dispose(),this.mapPass&&this.mapPass.dispose()}copy(t){return this.camera=t.camera.clone(),this.bias=t.bias,this.radius=t.radius,this.mapSize.copy(t.mapSize),this}clone(){return new this.constructor().copy(this)}toJSON(){const t={};return this.bias!==0&&(t.bias=this.bias),this.normalBias!==0&&(t.normalBias=this.normalBias),this.radius!==1&&(t.radius=this.radius),(this.mapSize.x!==512||this.mapSize.y!==512)&&(t.mapSize=this.mapSize.toArray()),t.camera=this.camera.toJSON(!1).object,delete t.camera.matrix,t}}class ay extends oy{constructor(){super(new gu(-5,5,5,-5,.5,500)),this.isDirectionalLightShadow=!0}}class Mf extends Pm{constructor(t,e){super(t,e),this.isDirectionalLight=!0,this.type="DirectionalLight",this.position.copy(Qe.DEFAULT_UP),this.updateMatrix(),this.target=new Qe,this.shadow=new ay}dispose(){this.shadow.dispose()}copy(t){return super.copy(t),this.target=t.target.clone(),this.shadow=t.shadow.clone(),this}}class cy extends Pm{constructor(t,e){super(t,e),this.isAmbientLight=!0,this.type="AmbientLight"}}class ly{constructor(t=1,e=0,n=0){return this.radius=t,this.phi=e,this.theta=n,this}set(t,e,n){return this.radius=t,this.phi=e,this.theta=n,this}copy(t){return this.radius=t.radius,this.phi=t.phi,this.theta=t.theta,this}makeSafe(){return this.phi=Math.max(1e-6,Math.min(Math.PI-1e-6,this.phi)),this}setFromVector3(t){return this.setFromCartesianCoords(t.x,t.y,t.z)}setFromCartesianCoords(t,e,n){return this.radius=Math.sqrt(t*t+e*e+n*n),this.radius===0?(this.theta=0,this.phi=0):(this.theta=Math.atan2(t,n),this.phi=Math.acos(Ge(e/this.radius,-1,1))),this}clone(){return new this.constructor().copy(this)}}const yf=new b,Ia=new b;class Gi{constructor(t=new b,e=new b){this.start=t,this.end=e}set(t,e){return this.start.copy(t),this.end.copy(e),this}copy(t){return this.start.copy(t.start),this.end.copy(t.end),this}getCenter(t){return t.addVectors(this.start,this.end).multiplyScalar(.5)}delta(t){return t.subVectors(this.end,this.start)}distanceSq(){return this.start.distanceToSquared(this.end)}distance(){return this.start.distanceTo(this.end)}at(t,e){return this.delta(e).multiplyScalar(t).add(this.start)}closestPointToPointParameter(t,e){yf.subVectors(t,this.start),Ia.subVectors(this.end,this.start);const n=Ia.dot(Ia);let s=Ia.dot(yf)/n;return e&&(s=Ge(s,0,1)),s}closestPointToPoint(t,e,n){const i=this.closestPointToPointParameter(t,e);return this.delta(n).multiplyScalar(i).add(this.start)}applyMatrix4(t){return this.start.applyMatrix4(t),this.end.applyMatrix4(t),this}equals(t){return t.start.equals(this.start)&&t.end.equals(this.end)}clone(){return new this.constructor().copy(this)}}class hy extends ds{constructor(t=10,e=10,n=4473924,i=8947848){n=new jt(n),i=new jt(i);const s=e/2,a=t/e,o=t/2,c=[],l=[];for(let f=0,d=0,g=-o;f<=e;f++,g+=a){c.push(-o,0,g,o,0,g),c.push(g,0,-o,g,0,o);const x=f===s?n:i;x.toArray(l,d),d+=3,x.toArray(l,d),d+=3,x.toArray(l,d),d+=3,x.toArray(l,d),d+=3}const h=new ye;h.setAttribute("position",new ie(c,3)),h.setAttribute("color",new ie(l,3));const u=new vi({vertexColors:!0,toneMapped:!1});super(h,u),this.type="GridHelper"}dispose(){this.geometry.dispose(),this.material.dispose()}}typeof __THREE_DEVTOOLS__<"u"&&__THREE_DEVTOOLS__.dispatchEvent(new CustomEvent("register",{detail:{revision:Bc}}));typeof window<"u"&&(window.__THREE__?console.warn("WARNING: Multiple instances of Three.js being imported."):window.__THREE__=Bc);function Wo(r,t){return!r||r.length!==t?Array.from({length:t},()=>({x:0,y:0,z:0,active:!1})):r}function cc(r,t){const e=Wo(t,r.length),n=r.nodeStorage;if(n){const{x:i,y:s,z:a}=n;for(let o=0;o<r.length;o++){const c=e[o];c.x=i[o],c.y=s[o],c.z=a[o],c.active=!0}return e}for(let i=0;i<r.length;i++){const s=r[i],a=e[i];a.x=s.x,a.y=s.y,a.z=s.z,a.active=!0}return e}function lc(r){for(let t=0;t<r.length;t++){const e=r[t];e.x=0,e.y=0,e.z=0,e.active=!1}}function Da(r,t,e,n,i){const s=r[t];s.x+=e,s.y+=n,s.z+=i,s.active=!0}let uy=32,dy=0,fy=8,py=.006,my=.002;const to=1e-4,Nl=1.25,Na=.92,Fl=.68,$e=.94,gy=2,xy=[.25,.5,.75],Ef=[.25,.5,.75];function _y(r,t,e){const n={x:new Float64Array(r),y:new Float64Array(r),z:new Float64Array(r),vx:new Float64Array(r),vy:new Float64Array(r),vz:new Float64Array(r),fx:new Float64Array(r),fy:new Float64Array(r),fz:new Float64Array(r),kx:new Float64Array(r),ky:new Float64Array(r),kz:new Float64Array(r),mass:new Float64Array(r),bendingStiffness:new Float64Array(r),pinned:new Uint8Array(r)};return n.mass.fill(t),n.bendingStiffness.fill(e),n}class vy{constructor(t,e){this._storage=t,this.index=e}get x(){return this._storage.x[this.index]}set x(t){this._storage.x[this.index]=t}get y(){return this._storage.y[this.index]}set y(t){this._storage.y[this.index]=t}get z(){return this._storage.z[this.index]}set z(t){this._storage.z[this.index]=t}get vx(){return this._storage.vx[this.index]}set vx(t){this._storage.vx[this.index]=t}get vy(){return this._storage.vy[this.index]}set vy(t){this._storage.vy[this.index]=t}get vz(){return this._storage.vz[this.index]}set vz(t){this._storage.vz[this.index]=t}get fx(){return this._storage.fx[this.index]}set fx(t){this._storage.fx[this.index]=t}get fy(){return this._storage.fy[this.index]}set fy(t){this._storage.fy[this.index]=t}get fz(){return this._storage.fz[this.index]}set fz(t){this._storage.fz[this.index]=t}get mass(){return this._storage.mass[this.index]}set mass(t){this._storage.mass[this.index]=t}get bendingStiffness(){return this._storage.bendingStiffness[this.index]}set bendingStiffness(t){this._storage.bendingStiffness[this.index]=t}get kx(){return this._storage.kx[this.index]}set kx(t){this._storage.kx[this.index]=t}get ky(){return this._storage.ky[this.index]}set ky(t){this._storage.ky[this.index]=t}get kz(){return this._storage.kz[this.index]}set kz(t){this._storage.kz[this.index]=t}get pinned(){return this._storage.pinned[this.index]!==0}set pinned(t){this._storage.pinned[this.index]=t?1:0}}class Sy{constructor(t,e,{mass:n=1,bendingStiffness:i=uy,smoothingIterations:s=dy,constraintIterations:a=fy,bendingConstraintIterations:o=0,bendAngleLimit:c=50,bendProjectionStrength:l=.35,curvatureFlow:h=0,logger:u=null}={}){this.segmentLength=e,this.nodeStorage=_y(t,n,i),this.nodes=Array.from({length:t},(f,d)=>new vy(this.nodeStorage,d)),this.nodes.nodeStorage=this.nodeStorage,this.smoothingIterations=s,this.constraintIterations=a,this.bendingConstraintIterations=o,this.bendAngleLimit=c,this.bendProjectionStrength=l,this.curvatureFlow=h,this.logger=u,this.iteration=0,this.collisionPrevPositions=null,this._constraintPrevPositions=null,this._bendingCorrections=null,this._smoothPositions=null,this._tensionCorrections=null;for(let f=0;f<t;f++)this.nodeStorage.x[f]=f*e}storeCollisionPreviousPositions(){const{x:t,y:e,z:n}=this.nodeStorage;if(!this.collisionPrevPositions||this.collisionPrevPositions.length!==this.nodes.length){this.collisionPrevPositions=Array.from({length:this.nodes.length},(i,s)=>new b(t[s],e[s],n[s]));return}for(let i=0;i<this.nodes.length;i++)this.collisionPrevPositions[i].set(t[i],e[i],n[i])}computeLength(){const{x:t,y:e,z:n}=this.nodeStorage;let i=0;for(let s=0;s<this.nodes.length-1;s++)i+=Math.hypot(t[s+1]-t[s],e[s+1]-e[s],n[s+1]-n[s]);return i}averageCurvature(){const{kx:t,ky:e,kz:n}=this.nodeStorage;let i=0;for(let s=0;s<this.nodes.length;s++)i+=Math.hypot(t[s],e[s],n[s]);return i/this.nodes.length}bendAngleAt(t){if(t<=0||t>=this.nodes.length-1)return 0;const{x:e,y:n,z:i}=this.nodeStorage,s=e[t]-e[t-1],a=n[t]-n[t-1],o=i[t]-i[t-1],c=e[t+1]-e[t],l=n[t+1]-n[t],h=i[t+1]-i[t],u=Math.hypot(s,a,o),f=Math.hypot(c,l,h);if(u<1e-8||f<1e-8)return 0;const d=(s*c+a*l+o*h)/(u*f);return Math.acos(Math.max(-1,Math.min(1,d)))*180/Math.PI}resetForces(){this.nodeStorage.fx.fill(0),this.nodeStorage.fy.fill(0),this.nodeStorage.fz.fill(0)}updateCurvature(){const{x:t,y:e,z:n,kx:i,ky:s,kz:a}=this.nodeStorage,o=this.segmentLength*this.segmentLength;i.fill(0),s.fill(0),a.fill(0);for(let c=1;c<this.nodes.length-1;c++)i[c]=(t[c-1]-2*t[c]+t[c+1])/o,s[c]=(e[c-1]-2*e[c]+e[c+1])/o,a[c]=(n[c-1]-2*n[c]+n[c+1])/o}accumulateBendingForces(){const t=this.nodes.length;if(t<3)return;const{fx:e,fy:n,fz:i,kx:s,ky:a,kz:o,bendingStiffness:c}=this.nodeStorage;for(let l=1;l<t-1;l++){const h=s[l],u=a[l],f=o[l],d=c[l],x=1+(h*h+u*u+f*f),m=d*h*x,p=d*u*x,_=d*f*x,v=.4,S=.1;l>=2&&(e[l-2]+=S*m,n[l-2]+=S*p,i[l-2]+=S*_),l+2<t&&(e[l+2]+=S*m,n[l+2]+=S*p,i[l+2]+=S*_),e[l-1]+=v*m,n[l-1]+=v*p,i[l-1]+=v*_,e[l+1]+=v*m,n[l+1]+=v*p,i[l+1]+=v*_;const y=v*2+(l>=2?S:0)+(l+2<t?S:0);e[l]-=y*m,n[l]-=y*p,i[l]-=y*_}}integrate(t){const{x:e,y:n,z:i,vx:s,vy:a,vz:o,fx:c,fy:l,fz:h,mass:u,pinned:f}=this.nodeStorage;for(let d=0;d<this.nodes.length;d++){if(f[d]){s[d]=0,a[d]=0,o[d]=0;continue}s[d]+=c[d]/u[d]*t,a[d]+=l[d]/u[d]*t,o[d]+=h[d]/u[d]*t,e[d]+=s[d]*t,n[d]+=a[d]*t,i[d]+=o[d]*t}}projectSegmentLengthConstraints(t=this.constraintIterations){const e=this.segmentLength;for(let n=0;n<t;n++)for(let i=0;i<this.nodes.length-1;i++){const s=this.nodes[i],a=this.nodes[i+1];let o=a.x-s.x,c=a.y-s.y,l=a.z-s.z,h=Math.hypot(o,c,l);if(!h)continue;const u=(h-e)/h;s.pinned&&a.pinned||(s.pinned?(o*=u,c*=u,l*=u,a.x-=o,a.y-=c,a.z-=l):a.pinned?(o*=u,c*=u,l*=u,s.x+=o,s.y+=c,s.z+=l):(o*=u*.5,c*=u*.5,l*=u*.5,s.x+=o,s.y+=c,s.z+=l,a.x-=o,a.y-=c,a.z-=l))}}solveConstraints(t,e={}){const n=cc(this.nodes,this._constraintPrevPositions);this._constraintPrevPositions=n;const i=e.applyBending??!0,s=e.velocityDamping??.92;if(this.projectSegmentLengthConstraints(),i){for(let c=1;c<this.nodes.length-1;c++){const l=this.nodes[c-1],h=this.nodes[c],u=this.nodes[c+1];if(h.pinned)continue;const f=(l.x+u.x)*.5,d=(l.y+u.y)*.5,g=(l.z+u.z)*.5,x=h.x-f,m=h.y-d,p=h.z-g,_=Math.min(1,h.bendingStiffness*t+this.curvatureFlow),v=x*_,S=m*_,y=p*_;h.x-=v,h.y-=S,h.z-=y}const o=e.bendingConstraintIterations??this.bendingConstraintIterations;o>0&&(this.projectBendingConstraints(o),this.projectSegmentLengthConstraints(Math.max(2,Math.ceil(this.constraintIterations*.5))))}this.smoothingIterations>0&&this.laplacianSmooth();const a=1/t;for(let o=0;o<this.nodes.length;o++){const c=this.nodes[o];if(c.pinned){c.vx=c.vy=c.vz=0;continue}c.vx=(c.x-n[o].x)*a*s,c.vy=(c.y-n[o].y)*a*s,c.vz=(c.z-n[o].z)*a*s}}projectBendingConstraints(t=this.bendingConstraintIterations){if(t<=0||this.nodes.length<3)return;const e=Math.max(0,this.bendAngleLimit),n=Math.max(0,Math.min(1,this.bendProjectionStrength));this._bendingCorrections=Wo(this._bendingCorrections,this.nodes.length);for(let i=0;i<t;i++){const s=this._bendingCorrections;lc(s);for(let a=1;a<this.nodes.length-1;a++){const o=this.nodes[a];if(o.pinned)continue;const c=this.bendAngleAt(a);if(c<=e)continue;const l=this.nodes[a-1],h=this.nodes[a+1],u=Math.max(0,Math.min(1,(c-e)/Math.max(1,180-e))),f=n*(.35+.65*u),d=s[a];d.x=((l.x+h.x)*.5-o.x)*f,d.y=((l.y+h.y)*.5-o.y)*f,d.z=((l.z+h.z)*.5-o.z)*f,d.active=!0}for(let a=1;a<this.nodes.length-1;a++){const o=this.nodes[a],c=s[a];!c.active||o.pinned||(o.x+=c.x,o.y+=c.y,o.z+=c.z)}}}laplacianSmooth(){const t=this.nodes.length;if(!(t<3)){this._smoothPositions=Wo(this._smoothPositions,t);for(let e=0;e<this.smoothingIterations;e++){const n=this._smoothPositions;lc(n);for(let i=1;i<t-1;i++){if(this.nodes[i].pinned)continue;const a=this.nodes[i-1],o=this.nodes[i+1],c=n[i];c.x=(a.x+o.x)*.5,c.y=(a.y+o.y)*.5,c.z=(a.z+o.z)*.5,c.active=!0}for(let i=1;i<t-1;i++){const s=this.nodes[i];if(s.pinned)continue;const a=n[i];a.active&&(s.x=a.x,s.y=a.y,s.z=a.z)}}}}straightenByTension(t=.2,e=1){const n=this.nodes.length;if(n<3||t<=0||e<=0)return;const i=Math.max(0,Math.min(1,t));this._tensionCorrections=Wo(this._tensionCorrections,n);for(let s=0;s<e;s++){const a=this._tensionCorrections;lc(a);for(let o=1;o<n-1;o++){const c=this.nodes[o];if(c.pinned)continue;const l=this.nodes[o-1],h=this.nodes[o+1],u=a[o];u.x=((l.x+h.x)*.5-c.x)*i,u.y=((l.y+h.y)*.5-c.y)*i,u.z=((l.z+h.z)*.5-c.z)*i,u.active=!0}for(let o=1;o<n-1;o++){const c=this.nodes[o],l=a[o];!l.active||c.pinned||(c.x+=l.x,c.y+=l.y,c.z+=l.z,c.vx*=.65,c.vy*=.65,c.vz*=.65)}}}releaseFromVesselWall(t,e=.1,n=1){if(!t||e<=0||n<=0)return;const i=Math.max(0,Math.min(1,e));for(let s=0;s<n;s++)for(const a of this.nodes){if(a.pinned)continue;let o=null;for(const h of t){if(h.isSheath)continue;const u=h.end.x-h.start.x,f=h.end.y-h.start.y,d=h.end.z-h.start.z,g=u*u+f*f+d*d;if(!g)continue;const x=a.x-h.start.x,m=a.y-h.start.y,p=a.z-h.start.z,_=(x*u+m*f+p*d)/g,v=Math.max(0,Math.min(1,_)),S=h.start.x+u*v,y=h.start.y+f*v,M=h.start.z+d*v,w=a.x-S,T=a.y-y,E=a.z-M,A=Math.hypot(w,T,E),D=A/(h.radius||1),R=Math.abs(D-.75)+Math.max(0,Math.abs(_-.5)-.5);D<=1.25&&(!o||R<o.distanceScore)&&(o={cx:S,cy:y,cz:M,radialDist:A,normalized:D,distanceScore:R})}if(!o||o.radialDist<=this.segmentLength*.15)continue;const c=Math.max(0,Math.min(1,(o.normalized-.25)/.75)),l=i*c;a.x+=(o.cx-a.x)*l,a.y+=(o.cy-a.y)*l,a.z+=(o.cz-a.z)*l,a.vx*=.75,a.vy*=.75,a.vz*=.75}}applyWallResponse(t,e,n,i,s,a){const o=t.vx*e+t.vy*n+t.vz*i;let c=t.vx-o*e,l=t.vy-o*n,h=t.vz-o*i;const u=Math.sqrt(c*c+l*l+h*h),f=a?Math.abs(o)*t.mass/s:0,d=Math.max(0,t.fx*e+t.fy*n+t.fz*i)+f;if(d>0&&u>0){const g=py*d*s/t.mass,x=my*d*s/t.mass;if(u<=g)c=0,l=0,h=0;else{const m=Math.max(0,u-x)/(u||1);c*=m,l*=m,h*=m}}t.vx=c,t.vy=l,t.vz=h}isPastOpenSheathEntrance(t,e){for(const n of e){if(!n.isSheath)continue;const i=n.end.x-n.start.x,s=n.end.y-n.start.y,a=n.end.z-n.start.z,o=i*i+s*s+a*a;if(!o)continue;const c=t.x-n.start.x,l=t.y-n.start.y,h=t.z-n.start.z,u=(c*i+l*s+h*a)/o;if(u>=-to)continue;const f=c-i*u,d=l-s*u,g=h-a*u;if(Math.hypot(f,d,g)<=n.radius+this.segmentLength)return!0}return!1}isInsideSegmentVolume(t,e){return this.segmentVolumeContact(t,e).inside}segmentVolumeContact(t,e){let n=null;for(const i of e||[]){const s=i.end.x-i.start.x,a=i.end.y-i.start.y,o=i.end.z-i.start.z,c=s*s+a*a+o*o;if(!c)continue;const l=t.x-i.start.x,h=t.y-i.start.y,u=t.z-i.start.z,f=(l*s+h*a+u*o)/c,d=Math.max(0,Math.min(1,f)),g=i.start.x+s*d,x=i.start.y+a*d,m=i.start.z+o*d,p=t.x-g,_=t.y-x,v=t.z-m,S=Math.hypot(p,_,v),y=S-i.radius;if(y<=0)return{inside:!0,segment:i,outside:y,cx:g,cy:x,cz:m,rx:p,ry:_,rz:v,radialDist:S,rawT:f};(!n||y<n.outside)&&(n={inside:!1,segment:i,outside:y,cx:g,cy:x,cz:m,rx:p,ry:_,rz:v,radialDist:S,rawT:f})}return n||{inside:!1,outside:1/0}}collideWithSegments(t,e,n,i={}){const s=this.segmentVolumeContact(t,e);if(s.inside||!Number.isFinite(s.outside))return!1;if(i.localOnly){const h=i.contactBand??this.segmentLength*Nl;if(s.outside>h||s.rawT<-to||s.rawT>1+to)return!1}const a=1/(s.radialDist||1),o=s.rx*a,c=s.ry*a,l=s.rz*a;return t.x=s.cx+o*s.segment.radius,t.y=s.cy+c*s.segment.radius,t.z=s.cz+l*s.segment.radius,this.applyWallResponse(t,o,c,l,n,!0),!0}collideRodSegmentsWithSegments(t,e,n={}){if(!t?.length)return;const i=n.segmentSamples||xy,s=n.contactBand??this.segmentLength*Nl;for(let a=0;a<this.nodes.length-1;a++){const o=this.nodes[a],c=this.nodes[a+1];if(!(o.pinned&&c.pinned))for(const l of i){const h=1-l,u=l,f={x:o.x*h+c.x*u,y:o.y*h+c.y*u,z:o.z*h+c.z*u};if(this.isPastOpenSheathEntrance(f,t))continue;const d=this.segmentVolumeContact(f,t);if(d.inside||!Number.isFinite(d.outside)||n.localOnly&&(d.outside>s||d.rawT<-to||d.rawT>1+to))continue;const g=1/(d.radialDist||1),x=d.cx+d.rx*g*d.segment.radius,m=d.cy+d.ry*g*d.segment.radius,p=d.cz+d.rz*g*d.segment.radius,_=(x-f.x)*Fl,v=(m-f.y)*Fl,S=(p-f.z)*Fl,y=o.pinned?0:h,M=c.pinned?0:u,w=y*y+M*M;if(!(w<=1e-8)){if(!o.pinned){const T=y/w;o.x+=_*T,o.y+=v*T,o.z+=S*T,o.vx*=$e,o.vy*=$e,o.vz*=$e}if(!c.pinned){const T=M/w;c.x+=_*T,c.y+=v*T,c.z+=S*T,c.vx*=$e,c.vy*=$e,c.vz*=$e}}}}}collideWithMeshCollider(t,e,n,i=0,s=null){if(s&&e?.crossingContact){const c=e.crossingContact(s,t,i);if(c){t.x=c.target.x,t.y=c.target.y,t.z=c.target.z;const l=c.normal||new b(1,0,0);return this.applyWallResponse(t,l.x,l.y,l.z,n,!0),!0}}const a=e?.pointContact?.(t,i);if(!a?.violation)return!1;t.x=a.target.x,t.y=a.target.y,t.z=a.target.z;const o=a.normal||new b(1,0,0);return this.applyWallResponse(t,o.x,o.y,o.z,n,!0),!0}isPastOpenMeshOutlet(t,e={}){return Number.isFinite(e.openOutletY)&&t.y>e.openOutletY}meshContactAtPoint(t,e,n={}){const i=Math.max(0,n.clearance||0),s=new b,o=e.boundsTree.closestPointToPoint(t,{point:s})?.distance??t.distanceTo(s),c=typeof n.interiorDirection=="function"?n.interiorDirection(t,s).clone():new b().subVectors(t,s);c.lengthSq()<1e-8&&c.set(1,0,0),c.normalize();const l=new b().subVectors(t,s).dot(c);return{closest:s,interior:c,insideDepth:l,dist:o,clearance:i}}collideWithMesh(t,e,n,i={}){const s=new b(t.x,t.y,t.z),a=this.meshContactAtPoint(s,e,i),{closest:o,interior:c,insideDepth:l,dist:h,clearance:u}=a,f=Math.max(u+this.segmentLength*Nl,this.segmentLength*1.5),d=-c.x,g=-c.y,x=-c.z;if(l<u)t.x=o.x+c.x*u,t.y=o.y+c.y*u,t.z=o.z+c.z*u,this.applyWallResponse(t,d,g,x,n,!0);else{if(h>f)return;const m=Math.max(0,t.fx*d+t.fy*g+t.fz*x),p=t.vx*t.vx+t.vy*t.vy+t.vz*t.vz-(t.vx*d+t.vy*g+t.vz*x)**2;m>0&&p>0&&this.applyWallResponse(t,d,g,x,n,!1)}}collideRodSegmentsWithMesh(t,e,n={},i=null){const s=n.segmentSamples||Ef,a=Math.max(0,n.segmentClearance??Math.min(n.clearance||0,this.segmentLength*.06));for(let o=0;o<this.nodes.length-1;o++){const c=this.nodes[o],l=this.nodes[o+1];if(!(c.pinned&&l.pinned))for(const h of s){const u=1-h,f=h,d=new b(c.x*u+l.x*f,c.y*u+l.y*f,c.z*u+l.z*f),g={x:d.x,y:d.y,z:d.z};if(this.isPastOpenMeshOutlet(g,n)||i&&(this.isInsideSegmentVolume(g,i)||this.isPastOpenSheathEntrance(g,i)))continue;const x=this.meshContactAtPoint(d,t,n);if(x.insideDepth>=a)continue;const m=x.closest.x+x.interior.x*a,p=x.closest.y+x.interior.y*a,_=x.closest.z+x.interior.z*a,v=(m-d.x)*Na,S=(p-d.y)*Na,y=(_-d.z)*Na,M=c.pinned?0:u,w=l.pinned?0:f,T=M*M+w*w;if(!(T<=1e-8)){if(!c.pinned){const E=M/T;c.x+=v*E,c.y+=S*E,c.z+=y*E,c.vx*=$e,c.vy*=$e,c.vz*=$e}if(!l.pinned){const E=w/T;l.x+=v*E,l.y+=S*E,l.z+=y*E,l.vx*=$e,l.vy*=$e,l.vz*=$e}}}}}collideRodSegmentsWithMeshCollider(t,e,n={},i=null,s=null){const a=n.segmentSamples||Ef,o=Math.max(0,n.segmentClearance??Math.min(n.clearance||0,this.segmentLength*.08)),c=this.segmentLength*4.5,l=new b,h=new b,u=new b,f=new b,d=(g,x,m,p,_)=>{h.subVectors(_,p),h.length()>c&&h.setLength(c),h.multiplyScalar(Na);const v=1-m,S=m,y=g.pinned?0:v,M=x.pinned?0:S,w=y*y+M*M;if(!(w<=1e-8)){if(!g.pinned){const T=y/w;g.x+=h.x*T,g.y+=h.y*T,g.z+=h.z*T,g.vx*=$e,g.vy*=$e,g.vz*=$e}if(!x.pinned){const T=M/w;x.x+=h.x*T,x.y+=h.y*T,x.z+=h.z*T,x.vx*=$e,x.vy*=$e,x.vz*=$e}}};for(let g=0;g<this.nodes.length-1;g++){const x=this.nodes[g],m=this.nodes[g+1];if(!(x.pinned&&m.pinned)){if(t.crossingContact){l.set((x.x+m.x)*.5,(x.y+m.y)*.5,(x.z+m.z)*.5);const p={x:l.x,y:l.y,z:l.z};if(this.isPastOpenMeshOutlet(p,n))continue;if(!(i&&(this.isInsideSegmentVolume(p,i)||this.isPastOpenSheathEntrance(p,i)))){u.set(x.x,x.y,x.z),f.set(m.x,m.y,m.z);const v=t.crossingContact(u,f,o);v&&v.t>.03&&v.t<.97&&d(x,m,v.t,v.point,v.target)}}for(const p of a){const _=1-p,v=p;l.set(x.x*_+m.x*v,x.y*_+m.y*v,x.z*_+m.z*v);const S={x:l.x,y:l.y,z:l.z};if(this.isPastOpenMeshOutlet(S,n)||i&&(this.isInsideSegmentVolume(S,i)||this.isPastOpenSheathEntrance(S,i)))continue;const y=t.pointContact(l,o);y?.violation&&d(x,m,p,l,y.target)}}}}collide(t,e=1){if(!t)return;const n=t.segments||null,i=t.meshCollider||t.lumenMeshCollider||null,s=t.collisionGeometry||(t.isBufferGeometry?t:t.geometry||t),a=this.collisionPrevPositions,o={clearance:t.guidewireClearance??t.collisionClearance??t.clearance??0,segmentClearance:t.guidewireSegmentClearance??t.segmentClearance,segmentSamples:t.guidewireSegmentSamples??t.segmentSamples,openOutletY:t.openOutletY,interiorDirection:t.interiorDirection||t.collisionInteriorDirection},c=!!i||!!s?.boundsTree,l={localOnly:c},h=Math.max(1,t.guidewireCollisionPasses??t.collisionPasses??(c?gy:4));if(!c&&!n)return;const u=()=>{for(let f=0;f<this.nodes.length;f++){const d=this.nodes[f];if(!d.pinned&&!(n&&(this.isInsideSegmentVolume(d,n)||this.isPastOpenSheathEntrance(d,n)||this.collideWithSegments(d,n,e,l)||!c))&&!this.isPastOpenMeshOutlet(d,o))if(i){const g=a?.[f]||null;this.collideWithMeshCollider(d,i,e,o.clearance,g)}else s&&s.boundsTree&&this.collideWithMesh(d,s,e,o)}};if(i){for(let f=0;f<h;f++)u(),n&&this.collideRodSegmentsWithSegments(n,e,l),this.collideRodSegmentsWithMeshCollider(i,e,o,n,a);u(),n&&this.collideRodSegmentsWithSegments(n,e,l)}else if(s&&s.boundsTree){for(let f=0;f<h;f++)u(),n&&this.collideRodSegmentsWithSegments(n,e,l),this.collideRodSegmentsWithMesh(s,e,o,n);u(),n&&this.collideRodSegmentsWithSegments(n,e,l)}else for(let f=0;f<h;f++)u(),n&&this.collideRodSegmentsWithSegments(n,e);this.smoothingIterations>0&&this.laplacianSmooth(e),this.storeCollisionPreviousPositions()}step(t){this.storeCollisionPreviousPositions(),this.resetForces(),this.updateCurvature(),this.accumulateBendingForces(),this.integrate(t),this.solveConstraints(t),this.iteration++,this.logger&&this.logger({iteration:this.iteration,curvature:this.averageCurvature(),length:this.computeLength()})}}function kt(r,t,e){return Math.max(t,Math.min(e,r))}function ln(r,t,e){const n=kt((e-r)/Math.max(1e-6,t-r),0,1);return n*n*(3-2*n)}const Li=1.35,My=.72,yy=2.4,Ey=.42,wy=.035,Ay=18,Ty=5,by=[.06,.12,.15,.25,.35,.45,.55,.65,.75,.85,.9,.94],Cy=64,Ry=.32,Py=0,Ly=1,Iy=.55,Dy=.45,Ny=2,Fy=[0,.2,.4,.6,.8,1],Uy=142,zy=0,By=5,Oy=10,Gy=6,Vy=2,ky=128,Hy=.38,Wy=2,Xy=1.1,Yy=.1,qy=170,Zy=140,jy=3,$y=10,Ky=108,Jy=1,Qy=.34,tE=2,eE=96,nE=10,iE=.35,sE=18,li=.001;function rr(){return{query:{inward:{x:0,y:0,z:0},normal:{x:0,y:0,z:0},closestPoint:{x:0,y:0,z:0}},target:{x:0,y:0,z:0},closestPoint:{x:0,y:0,z:0},inward:{x:0,y:0,z:0},normal:{x:0,y:0,z:0}}}function or(r,t){r.x=t.x,r.y=t.y,r.z=t.z}function wf(r,t,e){r.x+=t.x*e,r.y+=t.y*e,r.z+=t.z*e}function Af(r,t){return Math.hypot(r.x-t.x,r.y-t.y,r.z-t.z)}function pn(r,t){const e=Math.hypot(r.x,r.y,r.z);return e<1e-8?{...t}:{x:r.x/e,y:r.y/e,z:r.z/e}}function Tf(r,t,e){return{x:r.x*(1-e)+t.x*e,y:r.y*(1-e)+t.y*e,z:r.z*(1-e)+t.z*e}}function Ii(){return globalThis.performance?.now?.()??Date.now()}function rE(){return{advanceMs:0,solveMs:0,projectMs:0,diagnosticMs:0,pointContactCount:0,diagnosticPointContactCount:0,projectGuidewireCalls:0,nodeProjectionCount:0,segmentProjectionCount:0,segmentSampleCount:0,solveIterations:0,moving:!1,forceRelax:!1,foldGuarded:!1,stabilityRepaired:!1,withdrawalRelaxed:!1}}function oE(r){return r.advanceMs=0,r.solveMs=0,r.projectMs=0,r.diagnosticMs=0,r.pointContactCount=0,r.diagnosticPointContactCount=0,r.projectGuidewireCalls=0,r.nodeProjectionCount=0,r.segmentProjectionCount=0,r.segmentSampleCount=0,r.solveIterations=0,r.moving=!1,r.forceRelax=!1,r.foldGuarded=!1,r.stabilityRepaired=!1,r.withdrawalRelaxed=!1,r}class aE{constructor({rod:t,segmentLength:e,guidewireLength:n,sheath:i,lumenSampler:s=null,advanceRate:a=44,minInsert:o=0,maxInsert:c=n,lumenClearance:l=My,axialWindowScale:h=yy,straightening:u=Ey,routeBlend:f=wy,relaxationIterations:d=Ay,lengthIterations:g=Ty,segmentSamples:x=by,maxBendAngle:m=Cy,bendLimitStrength:p=Ry,bendLimitIterations:_=Py,segmentProjectionBlend:v=Ly,maxSegmentProjectionStep:S=Iy,meshClearance:y=Dy,collisionProjectionRepeats:M=Ny,foldAngle:w=Uy,foldUntangleStrength:T=zy,foldUntangleWindow:E=By,finalCollisionPasses:A=Oy,finalLengthPasses:D=Gy,finalProjectionPasses:R=Vy,foldGuardAngle:N=ky,foldGuardStrength:L=Hy,foldGuardPasses:F=Wy,foldGuardCenterPull:z=Xy,stabilityRepairSegmentError:q=Yy,stabilityRepairBendAngle:O=qy,stabilityRepairTargetBendAngle:Y=Zy,stabilityRepairPasses:K=jy,stabilityRepairLengthIterations:J=$y,tipBacktrackAngle:it=Ky,tipBacktrackStrength:X=Jy,withdrawalStraightening:j=Qy,withdrawalStraighteningPasses:nt=tE,withdrawalRelaxFrames:ot=eE,unsupportedBendRelaxAngle:ut=nE,unsupportedBendSupportBand:rt=iE,unsupportedBendRelaxFrames:ht=sE}){this.rod=t,this.segmentLength=e,this.guidewireLength=n,this.sheath=i,this.lumenSampler=typeof s=="function"?s:null,this.advanceRate=a,this.minInsert=o,this.maxInsert=c,this.lumenClearance=l,this.axialWindowScale=h,this.straightening=u,this.routeBlend=f,this.relaxationIterations=d,this.lengthIterations=g,this.segmentSamples=x,this.maxBendAngle=m,this.bendLimitStrength=p,this.bendLimitIterations=_,this.segmentProjectionBlend=v,this.maxSegmentProjectionStep=S,this.meshClearance=y,this.collisionProjectionRepeats=Math.max(1,Math.floor(M)),this.foldAngle=w,this.foldUntangleStrength=T,this.foldUntangleWindow=E,this.finalCollisionPasses=A,this.finalLengthPasses=D,this.finalProjectionPasses=R,this.foldGuardAngle=N,this.foldGuardStrength=L,this.foldGuardPasses=F,this.foldGuardCenterPull=z,this.stabilityRepairSegmentError=q,this.stabilityRepairBendAngle=O,this.stabilityRepairTargetBendAngle=Y,this.stabilityRepairPasses=K,this.stabilityRepairLengthIterations=J,this.tipBacktrackAngle=it,this.tipBacktrackStrength=X,this.withdrawalStraightening=j,this.withdrawalStraighteningPasses=nt,this.withdrawalRelaxFrames=ot,this.unsupportedBendRelaxAngle=ut,this.unsupportedBendSupportBand=rt,this.unsupportedBendRelaxFrames=ht,this.tailProgress=0,this.lastAdvanceDelta=0,this.settleFramesRemaining=0,this.withdrawalRelaxFramesRemaining=0,this.unsupportedBendRelaxFramesRemaining=0,this.unsupportedBendRelaxArmed=!0,this.contactPoints=[],this.breachPoints=[],this.previousPositions=null,this.performanceStats=rE(),this._advancePreviousPositions=null,this._solvePreviousPositions=null,this._straightenCorrections=null,this._spanCorrections=null,this._untangleCorrections=null,this._bendLimitCorrections=null,this._diagnosticContact=rr(),this._supportContact=rr(),this._projectContact=rr(),this._slidePointContact=rr(),this._slideTargetContact=rr(),this._zeroVelocityContact=rr(),this._projectNodePoint={x:0,y:0,z:0},this._convectSource={x:0,y:0,z:0},this._lumenConstraintState={projected:{x:0,y:0,z:0},radialMargin:0,axialOffset:0,axialWindow:0,breach:!1};const dt={x:i.end.x-i.start.x,y:i.end.y-i.start.y,z:i.end.z-i.start.z};this.sheathLength=Math.hypot(dt.x,dt.y,dt.z)||1,this.sheathDir=pn(dt,{x:1,y:0,z:0}),this.externalTailStart={x:i.start.x-this.sheathDir.x*n,y:i.start.y-this.sheathDir.y*n,z:i.start.z-this.sheathDir.z*n}}get progress(){return this.tailProgress}getPerformanceStats(){return{...this.performanceStats}}reset(){return this.tailProgress=this.minInsert,this.lastAdvanceDelta=0,this.settleFramesRemaining=0,this.withdrawalRelaxFramesRemaining=0,this.unsupportedBendRelaxFramesRemaining=0,this.unsupportedBendRelaxArmed=!0,this.contactPoints.length=0,this.breachPoints.length=0,this.initialize(),this}initialize(){const t=this.rod.nodes.nodeStorage;if(t){const{x:e,y:n,z:i,vx:s,vy:a,vz:o,pinned:c}=t;for(let l=0;l<this.rod.nodes.length;l++){const h=this.segmentLength*l;e[l]=this.externalTailStart.x+this.sheathDir.x*h,n[l]=this.externalTailStart.y+this.sheathDir.y*h,i[l]=this.externalTailStart.z+this.sheathDir.z*h,s[l]=0,a[l]=0,o[l]=0,c[l]=1}}else for(let e=0;e<this.rod.nodes.length;e++){const n=this.segmentLength*e,i=this.rod.nodes[e];i.x=this.externalTailStart.x+this.sheathDir.x*n,i.y=this.externalTailStart.y+this.sheathDir.y*n,i.z=this.externalTailStart.z+this.sheathDir.z*n,i.vx=i.vy=i.vz=0,i.pinned=!0}this.constrainSheath(),this.previousPositions=cc(this.rod.nodes,this._advancePreviousPositions),this._advancePreviousPositions=this.previousPositions}insertedCoordinate(t){return this.segmentLength*t-this.guidewireLength+this.tailProgress}firstLumenNodeIndex(){return kt(Math.ceil((this.sheathLength+this.guidewireLength-this.tailProgress)/this.segmentLength),0,this.rod.nodes.length)}firstInsertedNodeIndex(){return kt(Math.ceil((this.guidewireLength-this.tailProgress)/this.segmentLength),0,this.rod.nodes.length)}#t(){return kt(Math.floor((this.sheathLength+li+this.guidewireLength-this.tailProgress)/this.segmentLength)+1,0,this.rod.nodes.length)}#n(){return Math.max(0,this.#t()-1)}sheathAxisPoint(t){return{x:this.sheath.start.x+this.sheathDir.x*t,y:this.sheath.start.y+this.sheathDir.y*t,z:this.sheath.start.z+this.sheathDir.z*t}}routeSample(t){return this.#e(t)?{point:this.sheathAxisPoint(t),tangent:{...this.sheathDir},radius:this.sheath.radius||2}:this.lumenSampler?this.lumenSampler(Math.max(0,t-this.sheathLength)):{point:this.sheathAxisPoint(t),tangent:{...this.sheathDir},radius:1/0}}constrainSheath(t=0){const e=this.rod.nodes.nodeStorage;if(e){const{x:n,y:i,z:s,vx:a,vy:o,vz:c,pinned:l}=e,h=this.#t();for(let u=0;u<h;u++){const f=this.insertedCoordinate(u);l[u]=1,n[u]=this.sheath.start.x+this.sheathDir.x*f,i[u]=this.sheath.start.y+this.sheathDir.y*f,s[u]=this.sheath.start.z+this.sheathDir.z*f,a[u]=this.sheathDir.x*t,o[u]=this.sheathDir.y*t,c[u]=this.sheathDir.z*t}l.fill(0,h);return}for(let n=0;n<this.rod.nodes.length;n++){const i=this.insertedCoordinate(n),s=this.rod.nodes[n],a=this.#e(i);s.pinned=a,a&&(s.x=this.sheath.start.x+this.sheathDir.x*i,s.y=this.sheath.start.y+this.sheathDir.y*i,s.z=this.sheath.start.z+this.sheathDir.z*i,s.vx=this.sheathDir.x*t,s.vy=this.sheathDir.y*t,s.vz=this.sheathDir.z*t)}}advance(t,e,n=null,{routeAssist:i=!0}={}){oE(this.performanceStats);const s=Ii(),a=cc(this.rod.nodes,this._advancePreviousPositions);this._advancePreviousPositions=a;const o=kt(this.tailProgress+t*this.advanceRate*e,this.minInsert,this.maxInsert),c=o-this.tailProgress;this.tailProgress=o,this.lastAdvanceDelta=c,Math.abs(c)>1e-6&&(this.requestSettle(),this.unsupportedBendRelaxArmed=!0,this.unsupportedBendRelaxFramesRemaining=0),c<-1e-6&&(this.withdrawalRelaxFramesRemaining=Math.max(this.withdrawalRelaxFramesRemaining,Math.max(0,Math.floor(this.withdrawalRelaxFrames))));const l=c/Math.max(e,1e-6);return this.constrainSheath(l),Math.abs(c)>1e-6&&this.#i(c,a,e,n,i),this.previousPositions=a,this.performanceStats.advanceMs+=Ii()-s,this.performanceStats.moving=Math.abs(c)>1e-6,c}requestSettle(t=48){this.settleFramesRemaining=Math.max(this.settleFramesRemaining,t)}solve(t,e=null,{iterations:n=this.relaxationIterations,forceRelax:i=!1}={}){const s=Ii();this.performanceStats.forceRelax=this.performanceStats.forceRelax||!!i;const a=cc(this.rod.nodes,this._solvePreviousPositions);this._solvePreviousPositions=a,this.contactPoints.length=0,this.breachPoints.length=0,this.constrainSheath();const o=this.lastAdvanceDelta>1e-6,c=this.lastAdvanceDelta<-1e-6||this.withdrawalRelaxFramesRemaining>0;let l=!1;!o&&(this.unsupportedBendRelaxArmed||this.unsupportedBendRelaxFramesRemaining>0)&&(l=this.#m(e),l&&this.unsupportedBendRelaxArmed?(this.unsupportedBendRelaxFramesRemaining=Math.max(this.unsupportedBendRelaxFramesRemaining,Math.max(1,Math.floor(this.unsupportedBendRelaxFrames))),this.unsupportedBendRelaxArmed=!1):l||(this.unsupportedBendRelaxFramesRemaining=0,this.unsupportedBendRelaxArmed=!0));const h=l&&this.unsupportedBendRelaxFramesRemaining>0;if(!(i||Math.abs(this.lastAdvanceDelta)>1e-6||this.settleFramesRemaining>0||c||h)){this.#N(a,t,e),this.performanceStats.solveMs+=Ii()-s;return}Math.abs(this.lastAdvanceDelta)>1e-6&&(this.#a(),this.#w(e));const f=c||h;f&&(this.performanceStats.withdrawalRelaxed=this.#r(e,c?1:.72)||this.performanceStats.withdrawalRelaxed);const d=Math.max(1,n);this.performanceStats.solveIterations+=d;for(let x=0;x<d;x++){this.#S(x/d,e),f&&x<2&&(this.performanceStats.withdrawalRelaxed=this.#r(e,c?1:.72)||this.performanceStats.withdrawalRelaxed),this.#A();for(let m=0;m<this.collisionProjectionRepeats;m++)this.#u(),this.#o(this.lengthIterations),this.#f(e,!1);this.#o(2),this.#f(e,!1)}this.constrainSheath();for(let x=0;x<this.finalCollisionPasses;x++)this.#A(),this.#u(),this.#f(e,!1),this.#o(this.lengthIterations+2);this.#o(this.lengthIterations+4);for(let x=0;x<this.finalLengthPasses;x++)this.#f(e,!1),this.#o(5);for(let x=0;x<this.finalProjectionPasses;x++)this.#f(e,!1);this.#w(e),this.#o(Math.max(2,Math.ceil(this.lengthIterations*.4)));let g=this.#C();this.performanceStats.foldGuarded=this.performanceStats.foldGuarded||g,this.#f(e,!1),(g||this.#_(this.foldGuardAngle))&&(g=this.#C()||g,this.performanceStats.foldGuarded=this.performanceStats.foldGuarded||g,this.#o(this.lengthIterations+4),this.#f(e,!1),this.#o(4),this.#f(e,!1)),this.#L(e)&&(this.performanceStats.stabilityRepaired=!0,this.performanceStats.foldGuarded=!0),this.#N(a,t,e),Math.abs(this.lastAdvanceDelta)<=1e-6&&this.settleFramesRemaining>0&&this.settleFramesRemaining--,this.lastAdvanceDelta>=-1e-6&&this.withdrawalRelaxFramesRemaining>0&&this.withdrawalRelaxFramesRemaining--,this.unsupportedBendRelaxFramesRemaining>0&&(this.unsupportedBendRelaxFramesRemaining--,this.unsupportedBendRelaxFramesRemaining<=0&&!l&&(this.unsupportedBendRelaxArmed=!0)),this.performanceStats.solveMs+=Ii()-s}collectContactSamples(t=null,e=Li){const n=[],i=[],s=[0,.15,.35,.55,.75,.9,1],a=(c,l)=>{if(this.#e(l))return;const h=this.diagnosePoint(c,l,t,e);h.breach?this.#d(i,c):h.contact&&this.#d(n,c)};for(let c=0;c<this.rod.nodes.length-1;c++){const l=this.rod.nodes[c],h=this.rod.nodes[c+1];for(const u of s){const f=this.insertedCoordinate(c+u),d=Tf(l,h,u);a(d,f)}}const o=this.rod.nodes[this.rod.nodes.length-1];return a(o,this.insertedCoordinate(this.rod.nodes.length-1)),{contacts:n,breaches:i}}collectLumenDiagnostics(t=null,{clearance:e=this.meshClearance,contactBand:n=Li,samples:i=Fy,collectMarkers:s=!1,markerLimit:a=420}={}){const o=Ii();this.performanceStats.diagnosticMs=0,this.performanceStats.diagnosticPointContactCount=0;const c=t?.meshCollider||t?.lumenMeshCollider||null,l={checkedCount:0,contactCount:0,outsideCount:0,clearanceViolationCount:0,minSignedDistance:null,minClearanceMargin:null,worstPoint:null,worstInserted:null,maxSegmentError:0,maxBendAngle:0,clearance:e,contactBand:n,contacts:s?[]:null,breaches:s?[]:null};for(let h=0;h<this.rod.nodes.length-1;h++){const u=this.rod.nodes[h],f=this.rod.nodes[h+1];if(l.maxSegmentError=Math.max(l.maxSegmentError,Math.abs(Af(u,f)-this.segmentLength)),c?.pointContact)for(const d of i){const g=this.insertedCoordinate(h+d);if(this.#e(g))continue;const x=Tf(u,f,d),m=this.#x(c,x,e,!0,this._diagnosticContact),p=Number.isFinite(m?.signedDistance)?m.signedDistance:null;if(!Number.isFinite(p))continue;l.checkedCount++,(l.minSignedDistance===null||p<l.minSignedDistance)&&(l.minSignedDistance=p,l.worstPoint={x:x.x,y:x.y,z:x.z},l.worstInserted=g);const _=p-e;(l.minClearanceMargin===null||_<l.minClearanceMargin)&&(l.minClearanceMargin=_),p<0?(l.outsideCount++,s&&this.#d(l.breaches,x,a)):p<=n&&(l.contactCount++,s&&this.#d(l.contacts,x,a)),p<e&&l.clearanceViolationCount++}}if(typeof this.rod.bendAngleAt=="function")for(let h=1;h<this.rod.nodes.length-1;h++){const u=this.insertedCoordinate(h);this.#e(u)||(l.maxBendAngle=Math.max(l.maxBendAngle,this.rod.bendAngleAt(h)||0))}return this.performanceStats.diagnosticMs=Ii()-o,l}diagnosePoint(t,e,n=null,i=Li){const s=this.lumenSampler?this.#I(t,e):null;let a=s?s.radialMargin<=i||Math.abs(s.axialOffset)>=s.axialWindow-i:!1,o=s?.breach||!1;const c=n?.meshCollider||n?.lumenMeshCollider||null;if(c?.pointContact&&!this.#e(e)){const l=this.#x(c,t,0,!0,this._diagnosticContact);o=o||!!l?.violation,a=a||!l?.violation&&Number.isFinite(l?.distance)&&l.distance<=i}return{contact:!o&&a,breach:o}}#i(t,e,n,i=null,s=!0){const a=t/this.segmentLength,o=1/Math.max(n,1e-6),c=s?this.#v(i):null,l=this.#t();for(let h=l;h<this.rod.nodes.length;h++){const u=this.rod.nodes[h],f=this.insertedCoordinate(h);if(this.#e(f))continue;const d=this.#l(e,h+a,s?i:null,this._convectSource,s);if(!s){const y=e[h];or(u,d),u.vx=(d.x-y.x)*o*.2,u.vy=(d.y-y.y)*o*.2,u.vz=(d.z-y.z)*o*.2;continue}const g=this.lumenSampler?this.routeSample(f).point:d,x=f<this.sheathLength+this.segmentLength*2,m=this.lumenSampler?x?.64:.12:0,p={x:d.x*(1-m)+g.x*m,y:d.y*(1-m)+g.y*m,z:d.z*(1-m)+g.z*m},_=e[h],v=this.#c(_,{x:p.x-_.x,y:p.y-_.y,z:p.z-_.z},c);p.x=_.x+v.x,p.y=_.y+v.y,p.z=_.z+v.z;const S=this.#p(p,f,i,!1);or(u,S),u.vx=(S.x-_.x)*o*.2,u.vy=(S.y-_.y)*o*.2,u.vz=(S.z-_.z)*o*.2}}#l(t,e,n=null,i={x:0,y:0,z:0},s=!0){const a=t.length-1;if(e<=0){const _=t[0];return i.x=_.x+this.sheathDir.x*e*this.segmentLength,i.y=_.y+this.sheathDir.y*e*this.segmentLength,i.z=_.z+this.sheathDir.z*e*this.segmentLength,i}if(e<a){const _=Math.floor(e),v=Math.min(a,_+1),S=e-_,y=t[_],M=t[v];return i.x=y.x+(M.x-y.x)*S,i.y=y.y+(M.y-y.y)*S,i.z=y.z+(M.z-y.z)*S,i}const o=t[a],c=t[Math.max(0,a-1)];let l=o.x-c.x,h=o.y-c.y,u=o.z-c.z;const f=Math.sqrt(l*l+h*h+u*u);if(f>1e-8)l/=f,h/=f,u/=f;else if(s){const _=this.routeSample(this.tailProgress).tangent;l=_.x,h=_.y,u=_.z}else l=this.sheathDir.x,h=this.sheathDir.y,u=this.sheathDir.z;let d=l,g=h,x=u;const m=s?this.#v(n):null;if(m){const _=pn(this.#c(o,{x:l,y:h,z:u},m),{x:l,y:h,z:u});d=_.x,g=_.y,x=_.z}const p=(e-a)*this.segmentLength;return i.x=o.x+d*p,i.y=o.y+g*p,i.z=o.z+x*p,i}#a(t=1){if(this.routeBlend<=0||!this.lumenSampler)return;const e=this.rod.nodes.nodeStorage;if(e){const{x:n,y:i,z:s,pinned:a}=e,o=this.#t();for(let c=o;c<this.rod.nodes.length;c++){const l=this.insertedCoordinate(c);if(this.#e(l))continue;const h=this.routeSample(l),u=ln(this.sheathLength,this.sheathLength+this.segmentLength*8,l),f=this.routeBlend*t*(.35+.65*u);n[c]+=(h.point.x-n[c])*f,i[c]+=(h.point.y-i[c])*f,s[c]+=(h.point.z-s[c])*f}return}for(let n=0;n<this.rod.nodes.length;n++){const i=this.rod.nodes[n];if(i.pinned)continue;const s=this.insertedCoordinate(n);if(this.#e(s))continue;const a=this.routeSample(s),o=ln(this.sheathLength,this.sheathLength+this.segmentLength*8,s),c=this.routeBlend*t*(.35+.65*o);i.x+=(a.point.x-i.x)*c,i.y+=(a.point.y-i.y)*c,i.z+=(a.point.z-i.z)*c}}#h(t,e=null){const n=this.rod.nodes[t];if(!n)return!0;const i=this.insertedCoordinate(t);if(this.#e(i))return!0;const s=this.#v(e);if(!s?.pointContact)return!1;const a=this.#x(s,n,this.meshClearance,!1,this._supportContact);return!!a?.violation||Number.isFinite(a?.signedDistance)&&a.signedDistance<=this.meshClearance+this.unsupportedBendSupportBand}#m(t=null){const e=Math.max(0,this.unsupportedBendRelaxAngle);if(e<=0||typeof this.rod.bendAngleAt!="function")return!1;for(let n=1;n<this.rod.nodes.length-1;n++){if(this.rod.nodes[n].pinned||this.insertedCoordinate(n)<=this.sheathLength+this.segmentLength+li||(this.rod.bendAngleAt(n)||0)<=e)continue;const o=!this.#h(n+1,t),c=!this.#h(n,t);if(o||c)return!0}return!1}#g(t,e,n,i,s,a){let o=n;s?.pointContact&&!this.#e(e)&&(o=this.#c(t,o,s));const c=Math.hypot(o.x,o.y,o.z);if(c<=this.segmentLength*.002)return!1;if(c>a){const h=a/c;o={x:o.x*h,y:o.y*h,z:o.z*h}}const l=this.#p({x:t.x+o.x,y:t.y+o.y,z:t.z+o.z},e,i,!1);return or(t,l),!0}#r(t=null,e=1){const n=Math.max(0,Math.floor(this.withdrawalStraighteningPasses)),i=kt(this.withdrawalStraightening*e,0,1);if(n<=0||i<=0)return!1;const s=this.#v(t),a=this.withdrawalRelaxFramesRemaining>0?.55:.25,o=kt(Math.abs(this.lastAdvanceDelta)/Math.max(1e-6,this.segmentLength*.25),a,1),c=this.segmentLength*.16,l=Math.max(1,Math.ceil(56/Math.max(1e-6,this.segmentLength)));let h=!1;for(let u=0;u<n;u++)for(let f=1;f<this.rod.nodes.length;f++){const d=this.rod.nodes[f];if(d.pinned||this.insertedCoordinate(f)<=this.sheathLength+this.segmentLength+li)continue;const x=this.rod.nodes[f-1],m=this.rod.nodes[f-2],p=m?pn({x:x.x-m.x,y:x.y-m.y,z:x.z-m.z},this.sheathDir):this.sheathDir,_=pn({x:d.x-x.x,y:d.y-x.y,z:d.z-x.z},p),v=kt(p.x*_.x+p.y*_.y+p.z*_.z,-1,1),S=kt((1-v)/.28,0,1);if(S<=.001)continue;const y={x:x.x+p.x*this.segmentLength,y:x.y+p.y*this.segmentLength,z:x.z+p.z*this.segmentLength},M=i*o*(.25+.75*S);let w={x:(y.x-d.x)*M,y:(y.y-d.y)*M,z:(y.z-d.z)*M};const T=Math.hypot(w.x,w.y,w.z);if(!(T<=1e-8)){if(s?.pointContact){const E=this.#c(d,w,s),A=Math.hypot(E.x,E.y,E.z);if(A<T*.08){const D=pn({x:x.x-d.x,y:x.y-d.y,z:x.z-d.z},{x:-p.x,y:-p.y,z:-p.z}),R=Math.min(c,Math.max(T*.45,this.segmentLength*.035)),N=this.#c(d,{x:D.x*R,y:D.y*R,z:D.z*R},s);w=Math.hypot(N.x,N.y,N.z)>A?N:E}else w=E}for(let E=f;E<this.rod.nodes.length&&E<f+l;E++){const A=this.rod.nodes[E];if(!A||A.pinned)break;const D=this.insertedCoordinate(E);if(this.#e(D)||E>f&&this.#h(E,t))break;const R=this.#g(A,D,w,t,s,c);h=h||R}}}return h}#s(t){const e=Wo(this[t],this.rod.nodes.length);return this[t]=e,lc(e),e}#S(t,e=null){const n=this.#s("_straightenCorrections"),i=.35+.65*ln(0,1,t),s=this.#v(e),a=this.segmentLength*.18,o={x:0,y:0,z:0},c=this.rod.nodes.nodeStorage;if(c){const{x:u,y:f,z:d,pinned:g}=c,x=this.#t(),m=(_,v,S)=>{let y=S.x,M=S.y,w=S.z;if(s?.pointContact&&!this.#e(v)){const E=this.#c(this.rod.nodes[_],S,s);y=E.x,M=E.y,w=E.z}const T=Math.hypot(y,M,w);if(T>a){const E=a/T;y*=E,M*=E,w*=E}return o.x=u[_]+y,o.y=f[_]+M,o.z=d[_]+w,this.#p(o,v,e,!1)};for(let _=Math.max(1,x);_<this.rod.nodes.length-1;_++){if(g[_])continue;const v=this.insertedCoordinate(_),S=1-ln(this.sheathLength,this.sheathLength+this.segmentLength*5,v),y=this.straightening*i*(1-S*.45),M=n[_];M.x=((u[_-1]+u[_+1])*.5-u[_])*y,M.y=((f[_-1]+f[_+1])*.5-f[_])*y,M.z=((d[_-1]+d[_+1])*.5-d[_])*y,M.active=!0}for(let _=Math.max(1,x);_<this.rod.nodes.length-1;_++){const v=n[_];if(!v.active||g[_])continue;const S=this.insertedCoordinate(_),y=m(_,S,v);u[_]=y.x,f[_]=y.y,d[_]=y.z}const p=[2,4,8,12];for(const _ of p){const v=this.#s("_spanCorrections"),S=this.straightening*.13/Math.sqrt(_);for(let y=Math.max(_,x);y<this.rod.nodes.length-_;y++){if(g[y])continue;const M=v[y];M.x=((u[y-_]+u[y+_])*.5-u[y])*S,M.y=((f[y-_]+f[y+_])*.5-f[y])*S,M.z=((d[y-_]+d[y+_])*.5-d[y])*S,M.active=!0}for(let y=Math.max(_,x);y<this.rod.nodes.length-_;y++){const M=v[y];if(!M.active||g[y])continue;const w=this.insertedCoordinate(y),T=m(y,w,M);u[y]=T.x,f[y]=T.y,d[y]=T.z}}return}const l=(u,f,d)=>{let g=d.x,x=d.y,m=d.z;if(s?.pointContact&&!this.#e(f)){const _=this.#c(u,d,s);g=_.x,x=_.y,m=_.z}const p=Math.hypot(g,x,m);if(p>a){const _=a/p;g*=_,x*=_,m*=_}return o.x=u.x+g,o.y=u.y+x,o.z=u.z+m,this.#p(o,f,e,!1)};for(let u=1;u<this.rod.nodes.length-1;u++){const f=this.rod.nodes[u];if(f.pinned)continue;const d=this.rod.nodes[u-1],g=this.rod.nodes[u+1],x=this.insertedCoordinate(u),m=1-ln(this.sheathLength,this.sheathLength+this.segmentLength*5,x),p=this.straightening*i*(1-m*.45),_=n[u];_.x=((d.x+g.x)*.5-f.x)*p,_.y=((d.y+g.y)*.5-f.y)*p,_.z=((d.z+g.z)*.5-f.z)*p,_.active=!0}for(let u=1;u<this.rod.nodes.length-1;u++){const f=this.rod.nodes[u],d=n[u];if(!d.active||f.pinned)continue;const g=this.insertedCoordinate(u),x=l(f,g,d);or(f,x)}const h=[2,4,8,12];for(const u of h){const f=this.#s("_spanCorrections"),d=this.straightening*.13/Math.sqrt(u);for(let g=u;g<this.rod.nodes.length-u;g++){const x=this.rod.nodes[g];if(x.pinned)continue;const m=this.rod.nodes[g-u],p=this.rod.nodes[g+u],_=f[g];_.x=((m.x+p.x)*.5-x.x)*d,_.y=((m.y+p.y)*.5-x.y)*d,_.z=((m.z+p.z)*.5-x.z)*d,_.active=!0}for(let g=u;g<this.rod.nodes.length-u;g++){const x=this.rod.nodes[g],m=f[g];if(!m.active||x.pinned)continue;const p=this.insertedCoordinate(g),_=l(x,p,m);or(x,_)}}}#A(){if(this.foldUntangleStrength<=0||this.foldUntangleWindow<=0)return;const t=this.#s("_untangleCorrections"),e=kt(this.foldAngle,1,179),n=Math.max(1,Math.floor(this.foldUntangleWindow)),i=kt(this.foldUntangleStrength,0,1);for(let s=1;s<this.rod.nodes.length-1;s++){if(this.rod.nodes[s].pinned||this.insertedCoordinate(s)<=this.sheathLength+this.segmentLength+li)continue;const c=this.rod.bendAngleAt?.(s)??0;if(c<=e)continue;const l=kt((c-e)/Math.max(1,180-e),0,1);for(let h=-n;h<=n;h++){const u=s+h,f=this.rod.nodes[u];if(!f||f.pinned)continue;const d=this.insertedCoordinate(u);if(this.#e(d))continue;const g=1-Math.abs(h)/(n+1),x=i*l*g;if(x<=0)continue;const m=this.routeSample(d).point;Da(t,u,(m.x-f.x)*x,(m.y-f.y)*x,(m.z-f.z)*x)}}for(let s=1;s<this.rod.nodes.length-1;s++){const a=this.rod.nodes[s],o=t[s];!o.active||a.pinned||wf(a,o,1)}}#u(t=this.maxBendAngle,e=this.bendLimitStrength,n=this.bendLimitIterations,i=.45){if(n<=0||e<=0)return;const s=kt(t,1,179),a=s*Math.PI/180,o=2*this.segmentLength*Math.cos(a*.5),c=kt(e,0,1);for(let l=0;l<n;l++){const h=this.#s("_bendLimitCorrections");for(let u=1;u<this.rod.nodes.length-1;u++){const f=this.rod.nodes[u];if(f.pinned)continue;const d=this.insertedCoordinate(u);if(d<=this.sheathLength+this.segmentLength+li)continue;const g=this.rod.bendAngleAt?.(u)??0;if(g<=s)continue;const x=this.rod.nodes[u-1],m=this.rod.nodes[u+1],p={x:m.x-x.x,y:m.y-x.y,z:m.z-x.z},_=Math.hypot(p.x,p.y,p.z),v=this.routeSample(d).tangent,S=pn(p,v),y=kt((g-s)/Math.max(1,180-s),0,1),M=c*(.35+.65*y);if(_<o){const w=(o-_)*.5*M;x.pinned||Da(h,u-1,-S.x*w,-S.y*w,-S.z*w),m.pinned||Da(h,u+1,S.x*w,S.y*w,S.z*w)}Da(h,u,((x.x+m.x)*.5-f.x)*M*i,((x.y+m.y)*.5-f.y)*M*i,((x.z+m.z)*.5-f.z)*M*i)}for(let u=1;u<this.rod.nodes.length-1;u++){const f=this.rod.nodes[u],d=h[u];!d.active||f.pinned||wf(f,d,1)}}}#_(t){const e=kt(t,1,179);for(let n=1;n<this.rod.nodes.length-1;n++){if(this.rod.nodes[n].pinned||this.insertedCoordinate(n)<=this.sheathLength+this.segmentLength+li)continue;if((this.rod.bendAngleAt?.(n)??0)>e)return!0}return!1}#T(){let t=0;for(let e=0;e<this.rod.nodes.length-1;e++){const n=this.rod.nodes[e],i=this.rod.nodes[e+1];t=Math.max(t,Math.abs(Af(n,i)-this.segmentLength))}return t}#M(){let t=0;for(let e=1;e<this.rod.nodes.length-1;e++)this.rod.nodes[e].pinned||this.insertedCoordinate(e)<=this.sheathLength+this.segmentLength+li||(t=Math.max(t,this.rod.bendAngleAt?.(e)??0));return t}#y(t){const e=Math.abs(t.x),n=Math.abs(t.y),i=e<.7?{x:1,y:0,z:0}:n<.7?{x:0,y:1,z:0}:{x:0,y:0,z:1};return pn({x:t.y*i.z-t.z*i.y,y:t.z*i.x-t.x*i.z,z:t.x*i.y-t.y*i.x},{x:1,y:0,z:0})}#E(t,e){const n=kt(t,1,179)*Math.PI/180,i=Math.cos(n),s=Math.sin(n),a=kt(e,0,1);if(!(a<=0))for(let o=1;o<this.rod.nodes.length-1;o++){const c=this.rod.nodes[o-1],l=this.rod.nodes[o],h=this.rod.nodes[o+1];if(l.pinned||h.pinned||this.insertedCoordinate(o)<=this.sheathLength+this.segmentLength+li)continue;const f=pn({x:l.x-c.x,y:l.y-c.y,z:l.z-c.z},this.sheathDir),d={x:h.x-l.x,y:h.y-l.y,z:h.z-l.z},g=pn(d,f),x=f.x*g.x+f.y*g.y+f.z*g.z;if(x>=i)continue;let m={x:g.x-f.x*x,y:g.y-f.y*x,z:g.z-f.z*x};const p=Math.hypot(m.x,m.y,m.z);p<1e-8?m=this.#y(f):(m.x/=p,m.y/=p,m.z/=p);const _=pn({x:f.x*i+m.x*s,y:f.y*i+m.y*s,z:f.z*i+m.z*s},f),v={x:l.x+_.x*this.segmentLength,y:l.y+_.y*this.segmentLength,z:l.z+_.z*this.segmentLength};h.x+=(v.x-h.x)*a,h.y+=(v.y-h.y)*a,h.z+=(v.z-h.z)*a}}#w(t=null){const e=kt(this.tipBacktrackStrength,0,1);if(e<=0||this.rod.nodes.length<3)return!1;const n=this.rod.nodes.length-1,i=n-1,s=n-2,a=this.rod.nodes[n],o=this.rod.nodes[i],c=this.rod.nodes[s];if(a.pinned||o.pinned||this.insertedCoordinate(i)<=this.sheathLength+this.segmentLength+li)return!1;const h=kt(this.tipBacktrackAngle,1,179)*Math.PI/180,u=Math.cos(h),f=Math.sin(h),d=pn({x:o.x-c.x,y:o.y-c.y,z:o.z-c.z},this.sheathDir),g=pn({x:a.x-o.x,y:a.y-o.y,z:a.z-o.z},d),x=d.x*g.x+d.y*g.y+d.z*g.z;if(x>=u)return!1;let m={x:g.x-d.x*x,y:g.y-d.y*x,z:g.z-d.z*x};const p=Math.hypot(m.x,m.y,m.z);p<1e-8?m=this.#y(d):(m.x/=p,m.y/=p,m.z/=p);const _=pn({x:d.x*u+m.x*f,y:d.y*u+m.y*f,z:d.z*u+m.z*f},d),v=this.#v(t);let S={x:o.x+_.x*this.segmentLength,y:o.y+_.y*this.segmentLength,z:o.z+_.z*this.segmentLength};if(v?.pointContact){const M=pn(this.#c(a,d,v),_),w=this.#p({x:a.x+M.x*this.segmentLength*.8,y:a.y+M.y*this.segmentLength*.8,z:a.z+M.z*this.segmentLength*.8},this.insertedCoordinate(n),t,!1),T=pn({x:w.x-o.x,y:w.y-o.y,z:w.z-o.z},_);d.x*T.x+d.y*T.y+d.z*T.z>x&&(S=w)}const y=this.#p(S,this.insertedCoordinate(n),t,!1);return a.x+=(y.x-a.x)*e,a.y+=(y.y-a.y)*e,a.z+=(y.z-a.z)*e,a.vx*=.2,a.vy*=.2,a.vz*=.2,!0}#C(){if(this.foldGuardPasses<=0||this.foldGuardStrength<=0||!this.#_(this.foldGuardAngle))return!1;const t=Math.max(1,Math.floor(this.foldGuardPasses));let e=!1;for(let n=0;n<t&&(e=!0,this.#E(this.foldGuardAngle,this.foldGuardStrength),this.#u(this.foldGuardAngle,this.foldGuardStrength,1,this.foldGuardCenterPull),this.#o(Math.max(3,Math.ceil(this.lengthIterations*.5))),this.#o(2),!!this.#_(this.foldGuardAngle));n++);return e}#L(t){const e=Math.max(0,Math.floor(this.stabilityRepairPasses));if(e<=0)return!1;const n=Math.max(1e-4,this.stabilityRepairSegmentError),i=kt(this.stabilityRepairBendAngle,1,179),s=Math.max(0,Math.floor(this.stabilityRepairLengthIterations));let a=!1;for(let o=0;o<e;o++){const c=this.#T(),l=this.#M();if(c<=n&&l<=i)break;a=!0;const h=kt(c/n-1,0,1),u=kt((l-i)/Math.max(1,180-i),0,1),f=Math.max(h,u),d=kt(Math.min(this.stabilityRepairTargetBendAngle,i),1,179),g=kt(Math.max(this.foldGuardStrength,.72)*(.75+.25*f),0,1),x=Math.max(this.foldGuardCenterPull,1.1);this.#E(d,g),this.#u(d,g,2,x),this.#o(this.lengthIterations+s,t,!0),this.#f(t,!1),this.#u(d,g,1,x),this.#o(this.lengthIterations+Math.ceil(s*.5),t,!0),this.#f(t,!1),this.#o(Math.max(4,Math.ceil(this.lengthIterations*.5)),t,!0)}return a}#o(t,e=null,n=!1){const i=this.segmentLength,s=n?this.#v(e):null,a=!!s?.pointContact,o=this.#n(),c=this.rod.nodes.nodeStorage;if(!a&&c){const{x:l,y:h,z:u,pinned:f}=c;for(let d=0;d<t;d++){for(let g=o;g<this.rod.nodes.length-1;g++){const x=l[g+1]-l[g],m=h[g+1]-h[g],p=u[g+1]-u[g],_=Math.hypot(x,m,p);if(_<1e-8)continue;const v=(_-i)/_,S=f[g]?0:1,y=f[g+1]?0:1,M=S+y;if(M<=0)continue;const w=S/M,T=y/M;if(S){const E=v*w;l[g]+=x*E,h[g]+=m*E,u[g]+=p*E}if(y){const E=-v*T;l[g+1]+=x*E,h[g+1]+=m*E,u[g+1]+=p*E}}this.constrainSheath()}return}for(let l=0;l<t;l++){for(let h=o;h<this.rod.nodes.length-1;h++){const u=this.rod.nodes[h],f=this.rod.nodes[h+1],d=f.x-u.x,g=f.y-u.y,x=f.z-u.z,m=Math.hypot(d,g,x);if(m<1e-8)continue;const p=(m-i)/m,_=u.pinned?0:1,v=f.pinned?0:1,S=_+v;if(S<=0)continue;const y=_/S,M=v/S;if(!a){if(_){const w=p*y;u.x+=d*w,u.y+=g*w,u.z+=x*w}if(v){const w=-p*M;f.x+=d*w,f.y+=g*w,f.z+=x*w}continue}if(_){let w={x:d*p*y,y:g*p*y,z:x*p*y};this.#e(this.insertedCoordinate(h))||(w=this.#c(u,w,s)),u.x+=w.x,u.y+=w.y,u.z+=w.z}if(v){let w={x:-d*p*M,y:-g*p*M,z:-x*p*M};this.#e(this.insertedCoordinate(h+1))||(w=this.#c(f,w,s)),f.x+=w.x,f.y+=w.y,f.z+=w.z}}this.constrainSheath()}}#f(t,e){const n=Ii();this.performanceStats.projectGuidewireCalls++,this.#R(t,e),this.#F(t,e),this.#R(t,e),this.performanceStats.projectMs+=Ii()-n}#R(t,e){const n=this.#t(),i=this.rod.nodes.nodeStorage;if(i){const{x:s,y:a,z:o,pinned:c}=i,l=this._projectNodePoint;for(let h=n;h<this.rod.nodes.length;h++){if(c[h])continue;const u=this.insertedCoordinate(h);if(this.#e(u))continue;this.performanceStats.nodeProjectionCount++,l.x=s[h],l.y=a[h],l.z=o[h];const f=this.#p(l,u,t,e);s[h]=f.x,a[h]=f.y,o[h]=f.z}return}for(let s=n;s<this.rod.nodes.length;s++){const a=this.rod.nodes[s];if(a.pinned)continue;const o=this.insertedCoordinate(s);if(this.#e(o))continue;this.performanceStats.nodeProjectionCount++;const c=this.#p(a,o,t,e);or(a,c)}}#F(t,e){const n={x:0,y:0,z:0},i=this.segmentLength*this.maxSegmentProjectionStep,s=this.#n(),a=this.rod.nodes.nodeStorage;if(a){const{x:o,y:c,z:l,pinned:h}=a;for(let u=s;u<this.rod.nodes.length-1;u++)if(!(h[u]&&h[u+1]))for(const f of this.segmentSamples){const d=this.insertedCoordinate(u+f);if(this.#e(d))continue;this.performanceStats.segmentSampleCount++;const g=h[u]?0:1-f,x=h[u+1]?0:f;n.x=o[u]*(1-f)+o[u+1]*f,n.y=c[u]*(1-f)+c[u+1]*f,n.z=l[u]*(1-f)+l[u+1]*f;const m=this.#p(n,d,t,e);let p=(m.x-n.x)*this.segmentProjectionBlend,_=(m.y-n.y)*this.segmentProjectionBlend,v=(m.z-n.z)*this.segmentProjectionBlend;const S=Math.hypot(p,_,v);if(S>i){const M=i/S;p*=M,_*=M,v*=M}const y=g*g+x*x;if(!(y<=1e-8)){if(this.performanceStats.segmentProjectionCount++,g){const M=g/y;o[u]+=p*M,c[u]+=_*M,l[u]+=v*M}if(x){const M=x/y;o[u+1]+=p*M,c[u+1]+=_*M,l[u+1]+=v*M}}}return}for(let o=s;o<this.rod.nodes.length-1;o++){const c=this.rod.nodes[o],l=this.rod.nodes[o+1];if(!(c.pinned&&l.pinned))for(const h of this.segmentSamples){const u=this.insertedCoordinate(o+h);if(this.#e(u))continue;this.performanceStats.segmentSampleCount++;const f=c.pinned?0:1-h,d=l.pinned?0:h;n.x=c.x*(1-h)+l.x*h,n.y=c.y*(1-h)+l.y*h,n.z=c.z*(1-h)+l.z*h;const g=this.#p(n,u,t,e);let x=(g.x-n.x)*this.segmentProjectionBlend,m=(g.y-n.y)*this.segmentProjectionBlend,p=(g.z-n.z)*this.segmentProjectionBlend;const _=Math.hypot(x,m,p);if(_>i){const S=i/_;x*=S,m*=S,p*=S}const v=f*f+d*d;if(!(v<=1e-8)){if(this.performanceStats.segmentProjectionCount++,f){const S=f/v;c.x+=x*S,c.y+=m*S,c.z+=p*S}if(d){const S=d/v;l.x+=x*S,l.y+=m*S,l.z+=p*S}}}}}#p(t,e,n,i){const s=n?.meshCollider||n?.lumenMeshCollider||null;if(s?.pointContact&&!this.#e(e)){let a=t;if(this.lumenSampler){const c=this.#I(t,e);a=c.projected,i&&(c.breach?this.#d(this.breachPoints,t):c.radialMargin<=Li&&this.#d(this.contactPoints,t))}const o=this.#x(s,a,this.meshClearance,!1,this._projectContact);return o?.violation&&o.target?(i&&(Number.isFinite(o.signedDistance)&&o.signedDistance<0?this.#d(this.breachPoints,a):this.#d(this.contactPoints,a)),{x:o.target.x,y:o.target.y,z:o.target.z}):(i&&Number.isFinite(o?.distance)&&o.distance<=Li&&this.#d(this.contactPoints,a),a)}return this.#U(t,e,i)}#v(t){return t?.meshCollider||t?.lumenMeshCollider||null}#x(t,e,n,i=!1,s=null){return i?this.performanceStats.diagnosticPointContactCount++:this.performanceStats.pointContactCount++,t.pointContact(e,n,s)}#c(t,e,n){if(!n?.pointContact)return{x:e.x,y:e.y,z:e.z};const i={x:t.x+e.x,y:t.y+e.y,z:t.z+e.z},s=this.#x(n,t,this.meshClearance,!1,this._slidePointContact),a=this.#x(n,i,this.meshClearance,!1,this._slideTargetContact),o=s?.violation||Number.isFinite(s?.signedDistance)&&s.signedDistance<=this.meshClearance+Li,c=a?.violation||Number.isFinite(a?.signedDistance)&&a.signedDistance<=this.meshClearance+Li;if(!o&&!c)return{x:e.x,y:e.y,z:e.z};const h=(a?.violation||c?a:s)?.normal||s?.normal||a?.normal,u=h?Math.hypot(h.x,h.y,h.z):0;if(u<1e-8)return{x:e.x,y:e.y,z:e.z};const f=h.x/u,d=h.y/u,g=h.z/u,x=e.x*f+e.y*d+e.z*g;return x<=0?{x:e.x,y:e.y,z:e.z}:{x:e.x-f*x,y:e.y-d*x,z:e.z-g*x}}#U(t,e,n){if(!this.lumenSampler)return t;const i=this.#I(t,e);return n&&(i.breach?this.#d(this.breachPoints,t):i.radialMargin<=Li&&this.#d(this.contactPoints,t)),i.projected}#I(t,e){const n=this.routeSample(e),i=Math.max(.5,(n.radius||1)-this.lumenClearance),s=n.tangent||this.sheathDir,a=Math.hypot(s.x,s.y,s.z),o=a<1e-8?this.sheathDir.x:s.x/a,c=a<1e-8?this.sheathDir.y:s.y/a,l=a<1e-8?this.sheathDir.z:s.z/a,h=t.x-n.point.x,u=t.y-n.point.y,f=t.z-n.point.z;let d=h*o+u*c+f*l,g=h-o*d,x=u-c*d,m=f-l*d,p=Math.hypot(g,x,m);const _=Math.max(this.segmentLength*.5,this.segmentLength*this.axialWindowScale),v=p>i+1e-4;if(p>i){const y=i/Math.max(1e-8,p);g*=y,x*=y,m*=y,p=i}d=kt(d,-_,_);const S=this._lumenConstraintState;return S.projected.x=n.point.x+o*d+g,S.projected.y=n.point.y+c*d+x,S.projected.z=n.point.z+l*d+m,S.radialMargin=i-p,S.axialOffset=d,S.axialWindow=_,S.breach=v,S}#N(t,e,n=null){const i=1/Math.max(e,1e-6),s=n?.meshCollider||n?.lumenMeshCollider||null,a=this.rod.nodes.nodeStorage;if(a&&!s?.pointContact){const{x:o,y:c,z:l,vx:h,vy:u,vz:f}=a;for(let d=0;d<this.rod.nodes.length;d++){const g=o[d]-t[d].x,x=c[d]-t[d].y,m=l[d]-t[d].z,p=g*g+x*x+m*m>4e-4?.08:0;h[d]=g*i*p,u[d]=x*i*p,f[d]=m*i*p}return}for(let o=0;o<this.rod.nodes.length;o++){const c=this.rod.nodes[o],l=c.x-t[o].x,h=c.y-t[o].y,u=c.z-t[o].z,f=l*l+h*h+u*u>4e-4?.08:0;let d=l*i*f,g=h*i*f,x=u*i*f;const m=this.insertedCoordinate(o);if(s?.pointContact&&!this.#e(m)){const p=this.#x(s,c,this.meshClearance,!1,this._zeroVelocityContact),_=p?.normal;if((p?.violation||Number.isFinite(p?.signedDistance)&&p.signedDistance<=this.meshClearance+Li)&&_){const S=d*_.x+g*_.y+x*_.z;S>0&&(d-=_.x*S,g-=_.y*S,x-=_.z*S)}}c.vx=d,c.vy=g,c.vz=x}}#d(t,e,n=420){t.length>=n||t.push({x:e.x,y:e.y,z:e.z})}#e(t){return t<=this.sheathLength+li}}const cE="OETCOLL1",Au=8,Ul=Au+4,lE=1;new TextEncoder;const hE=new TextDecoder,uE={Float32Array,Uint32Array,Int16Array,Uint8Array,Int8Array};function bf(r,t=8){return Math.ceil(r/t)*t}function dE(r){let t="";for(let e=0;e<Au;e++)t+=String.fromCharCode(r[e]);return t}function Lm(r){if(!(r instanceof ArrayBuffer))throw new TypeError("Collision asset must be an ArrayBuffer");if(r.byteLength<Ul)throw new Error("Collision asset is truncated");const t=new Uint8Array(r),e=dE(t);if(e!==cE)throw new Error(`Unexpected collision asset magic: ${e}`);const n=new DataView(r).getUint32(Au,!0),i=Ul+n;if(i>r.byteLength)throw new Error("Collision asset manifest is truncated");const s=JSON.parse(hE.decode(t.subarray(Ul,i)));if(s.version!==lE)throw new Error(`Unsupported collision asset version: ${s.version}`);const a={};let o=bf(i);for(const c of s.sections||[]){const l=uE[c.type];if(!l)throw new Error(`Unsupported collision asset array type: ${c.type}`);const h=c.length*l.BYTES_PER_ELEMENT;if(o+h>r.byteLength)throw new Error(`Collision asset section is truncated: ${c.name}`);a[c.name]=new l(r,o,c.length),o=bf(o+h)}return{metadata:s,arrays:a,buffer:r}}const eo=1e-8,Qi=0,hc=1,zl=2,fE=3,pE=4,Im=5,cn=0,yn=1,Ms=2;function ar(r,t,e,n){return r.x=t,r.y=e,r.z=n,r}function Cf(){const r=new Float64Array(6);return r[Qi]=-1/0,r[hc]=1,r[Im]=-1,r}function Dm(){return{inside:!1,signedDistance:-1/0,distance:1/0,inward:{x:1,y:0,z:0},normal:{x:-1,y:0,z:0},closestPoint:{x:0,y:0,z:0},lowerSliceIndex:-1,upperSliceIndex:-1}}class mE{constructor(t,e){this.metadata=t.lumen,this.sliceYs=e.lumenSliceYs,this.sliceContourOffsets=e.lumenSliceContourOffsets,this.contourPointOffsets=e.lumenContourPointOffsets,this.contourBounds=e.lumenContourBounds,this.contourSamples=e.lumenContourSamples,this.points=e.lumenPoints,this.pointQuantization=this.points instanceof Int16Array?this.metadata.pointQuantization||.02:1,this.axisBases=e.lumenAxisBases||new Float32Array([1,0,0,0,1,0,0,0,1]),this.axisSliceOffsets=e.lumenAxisSliceOffsets||new Uint32Array([0,this.sliceYs.length]),this.axisCount=Math.max(1,this.axisSliceOffsets.length-1),this._lower=Cf(),this._upper=Cf(),this._interval=new Float64Array(3),this._lastLower=new Int32Array(this.axisCount),this._lastUpper=new Int32Array(this.axisCount);for(let n=0;n<this.axisCount;n++){const i=this.axisSliceOffsets[n],s=this.axisSliceOffsets[n+1];this._lastLower[n]=i,this._lastUpper[n]=Math.min(i+1,Math.max(i,s-1))}}query(t,e=null){return this.queryCoordinates(t.x,t.y,t.z,e)}isInsideCoordinates(t,e,n){for(let i=0;i<this.axisCount;i++){const s=i*9,a=t*this.axisBases[s]+e*this.axisBases[s+1]+n*this.axisBases[s+2],o=t*this.axisBases[s+3]+e*this.axisBases[s+4]+n*this.axisBases[s+5],c=t*this.axisBases[s+6]+e*this.axisBases[s+7]+n*this.axisBases[s+8],l=this.#n(o,i);if(l[cn]<0||!this.#t(l[cn],a,c)&&(l[yn]===l[cn]||!this.#t(l[yn],a,c)))continue;const h=this.#i(l[cn],a,c,this._lower),u=l[yn]===l[cn]?h:this.#i(l[yn],a,c,this._upper),f=l[Ms];if(h[Qi]*(1-f)+u[Qi]*f>=0)return!0}return!1}#t(t,e,n){const i=this.sliceContourOffsets[t],s=this.sliceContourOffsets[t+1];for(let a=i;a<s;a++){const o=a*4;if(e>=this.contourBounds[o]&&e<=this.contourBounds[o+1]&&n>=this.contourBounds[o+2]&&n<=this.contourBounds[o+3])return!0}return!1}queryCoordinates(t,e,n,i=null){const s=i||Dm();if(!this.sliceYs.length)return s.inside=!1,s.signedDistance=-1/0,s.distance=1/0,ar(s.inward,1,0,0),ar(s.normal,-1,0,0),ar(s.closestPoint,t,e,n),s.lowerSliceIndex=-1,s.upperSliceIndex=-1,s;let a=-1/0,o=1,c=0,l=0,h=-1,u=-1;for(let f=0;f<this.axisCount;f++){const d=f*9,g=t*this.axisBases[d]+e*this.axisBases[d+1]+n*this.axisBases[d+2],x=t*this.axisBases[d+3]+e*this.axisBases[d+4]+n*this.axisBases[d+5],m=t*this.axisBases[d+6]+e*this.axisBases[d+7]+n*this.axisBases[d+8],p=this.#n(x,f);if(p[cn]<0)continue;const _=this.#i(p[cn],g,m,this._lower),v=p[yn]===p[cn]?_:this.#i(p[yn],g,m,this._upper),S=p[Ms],y=_[Qi]*(1-S)+v[Qi]*S;if(y<=a)continue;const M=Math.max(eo,Math.abs(this.sliceYs[p[yn]]-this.sliceYs[p[cn]])),w=p[yn]===p[cn]?0:Math.max(-.85,Math.min(.85,(v[Qi]-_[Qi])/M));let T=_[hc]*(1-S)+v[hc]*S,E=w,A=_[zl]*(1-S)+v[zl]*S;const D=Math.sqrt(T*T+E*E+A*A);D>eo?(T/=D,E/=D,A/=D):(T=1,E=0,A=0);let R=this.axisBases[d]*T+this.axisBases[d+3]*E+this.axisBases[d+6]*A,N=this.axisBases[d+1]*T+this.axisBases[d+4]*E+this.axisBases[d+7]*A,L=this.axisBases[d+2]*T+this.axisBases[d+5]*E+this.axisBases[d+8]*A;const F=Math.sqrt(R*R+N*N+L*L)||1;R/=F,N/=F,L/=F,a=y,o=R,c=N,l=L,h=p[cn],u=p[yn]}return s.inside=a>=0,s.signedDistance=a,s.distance=Math.abs(a),ar(s.inward,o,c,l),ar(s.normal,-o,-c,-l),ar(s.closestPoint,t-o*a,e-c*a,n-l*a),s.lowerSliceIndex=h,s.upperSliceIndex=u,s}#n(t,e){const n=this._interval,i=this.axisSliceOffsets[e],s=this.axisSliceOffsets[e+1]-1;if(s<i)return n[cn]=-1,n[yn]=-1,n[Ms]=0,n;if(s===i||t<=this.sliceYs[i])return this._lastLower[e]=i,this._lastUpper[e]=i,n[cn]=i,n[yn]=i,n[Ms]=0,n;if(t>=this.sliceYs[s])return this._lastLower[e]=s,this._lastUpper[e]=s,n[cn]=s,n[yn]=s,n[Ms]=0,n;let a=this._lastLower[e],o=this._lastUpper[e];if(a>=i&&o>a&&o<=s&&this.sliceYs[a]<=t&&t<=this.sliceYs[o]){const u=Math.max(eo,this.sliceYs[o]-this.sliceYs[a]);return n[cn]=a,n[yn]=o,n[Ms]=(t-this.sliceYs[a])/u,n}let c=i,l=s;for(;l-c>1;){const u=Math.floor((c+l)*.5);this.sliceYs[u]<=t?c=u:l=u}a=c,o=l,this._lastLower[e]=a,this._lastUpper[e]=o;const h=Math.max(eo,this.sliceYs[o]-this.sliceYs[a]);return n[cn]=a,n[yn]=o,n[Ms]=(t-this.sliceYs[a])/h,n}#i(t,e,n,i){let s=-1/0,a=1,o=0,c=e,l=n,h=-1;const u=this.sliceContourOffsets[t],f=this.sliceContourOffsets[t+1];for(let d=u;d<f;d++){const g=d*4;let x=0,m=0;if(e<this.contourBounds[g]?x=this.contourBounds[g]-e:e>this.contourBounds[g+1]&&(x=e-this.contourBounds[g+1]),n<this.contourBounds[g+2]?m=this.contourBounds[g+2]-n:n>this.contourBounds[g+3]&&(m=n-this.contourBounds[g+3]),Number.isFinite(s)&&s<0&&-Math.sqrt(x*x+m*m)<=s)continue;const p=this.contourPointOffsets[d],_=this.contourPointOffsets[d+1];if(_<=p)continue;let v=!1,S=e,y=n,M=1/0,w=_-1;for(let N=p;N<_;N++){const L=this.points[N*2]*this.pointQuantization,F=this.points[N*2+1]*this.pointQuantization,z=this.points[w*2]*this.pointQuantization,q=this.points[w*2+1]*this.pointQuantization;F>n!=q>n&&e<(z-L)*(n-F)/(q-F+1e-12)+L&&(v=!v);const O=z-L,Y=q-F,K=O*O+Y*Y||1,J=Math.max(0,Math.min(1,((e-L)*O+(n-F)*Y)/K)),it=L+O*J,X=F+Y*J,j=e-it,nt=n-X,ot=j*j+nt*nt;ot<M&&(M=ot,S=it,y=X),w=N}const T=Math.sqrt(M),E=v?T:-T;if(E<=s)continue;let A=v?e-S:S-e,D=v?n-y:y-n;const R=Math.sqrt(A*A+D*D);if(R>eo)A/=R,D/=R;else{A=this.contourSamples[d*2]-S,D=this.contourSamples[d*2+1]-y;const N=Math.sqrt(A*A+D*D)||1;A/=N,D/=N}s=E,a=A,o=D,c=S,l=y,h=d}return i[Qi]=s,i[hc]=a,i[zl]=o,i[fE]=c,i[pE]=l,i[Im]=h,i}}function gE(r,t){return!r?.lumen||!t?.lumenSliceYs?.length?null:new mE(r,t)}const xE="fallback";let no=class{constructor(t=0,e=0,n=0){this.values=new Float64Array([t,e,n])}get x(){return this.values[0]}set x(t){this.values[0]=t}get y(){return this.values[1]}set y(t){this.values[1]=t}get z(){return this.values[2]}set z(t){this.values[2]=t}},_E=class{constructor(){this.values=new Float64Array([-1/0,-1/0,1/0,1/0,-1,0,1]),this.inside=!1,this.violation=!1,this.conservative=!1,this.source=xE,this.point=new no,this.target=new no,this.closestPoint=new no,this.normal=new no(1,0,0),this.inward=new no(1,0,0)}get signedDistance(){return this.values[0]}set signedDistance(t){this.values[0]=t}get signedGap(){return this.values[1]}set signedGap(t){this.values[1]=t}get distance(){return this.values[2]}set distance(t){this.values[2]=t}get penetration(){return this.values[3]}set penetration(t){this.values[3]=t}get branchId(){return this.values[4]}set branchId(t){this.values[4]=t}get segmentT(){return this.values[5]}set segmentT(t){this.values[5]=t}get timeOfImpact(){return this.values[6]}set timeOfImpact(t){this.values[6]=t}};function Rf(){return new _E}const vE=25.4,Nm=1/3,Fm=.035,Um=Fm*vE,Jo=Um/2,Fh=Jo*.5,zm=6,Bm=zm*Nm,SE=Bm/2,Om=1.8,Gm=Om/2,Vm=5,km=Vm*Nm,wc=km/2,Hm=.97,Wm=Hm/2,Bl=wc*.78,ee=1e-8,ME=1/120,Pf=1,Fa=3,Lf=4,If=5,Df=5,yE=.01;function He(r,t,e){return Math.max(t,Math.min(e,r))}function be(r,t,e){return Math.sqrt(r*r+t*t+e*e)}function On(){return globalThis.performance?.now?.()??Date.now()}function EE(r,t,e){if(!t)return 0;const n=Array.from(r.subarray(0,t));return n.sort((i,s)=>i-s),n[Math.min(n.length-1,Math.floor((n.length-1)*e))]}function io(r=512){return{samples:new Float32Array(r),cursor:0,count:0,recordedCount:0,total:0,last:0}}function so(r,t){r.last=t,r.total+=t,r.recordedCount++,r.samples[r.cursor]=t,r.cursor=(r.cursor+1)%r.samples.length,r.count=Math.min(r.samples.length,r.count+1)}function ro(r){return{lastMs:r.last,averageMs:r.recordedCount?r.total/r.recordedCount:0,p95Ms:EE(r.samples,r.count,.95)}}const Tu=Object.freeze({guidewire:Object.freeze({id:"guidewire",radius:Jo,mass:1,stretchCompliance:2e-7,bendCompliance:2e-5,maxBendAngle:135,foldLimitStrength:.7,wallFriction:.08}),catheter:Object.freeze({id:"catheter",outerRadius:wc,innerDiameter:Hm,innerRadius:Wm,radius:wc,mass:1.4,stretchCompliance:1e-7,bendCompliance:4e-4,maxBendAngle:120,foldLimitStrength:.75,wallFriction:.12,lumenFriction:.04}),sheath:Object.freeze({id:"sheath",outerRadius:1,innerDiameter:Om,innerRadius:Gm})});class Nf{constructor(t,e,n,i={}){if(!Number.isInteger(e)||e<2)throw new RangeError("A rod requires at least two nodes");this.id=t,this.count=e,this.segmentCount=e-1,this.segmentLength=n,this.radius=i.radius??.5,this.innerRadius=i.innerRadius??0,this.mass=i.mass??1,this.stretchCompliance=i.stretchCompliance??2e-7,this.bendCompliance=i.bendCompliance??.001,this.shapeCompliance=i.shapeCompliance??5e-5,this.maxBendAngle=i.maxBendAngle??135,this.foldLimitStrength=i.foldLimitStrength??.7,this.wallCompliance=i.wallCompliance??0,this.wallFriction=i.wallFriction??.08,this.lumenFriction=i.lumenFriction??.04,this.linearDamping=i.linearDamping??.98,this.sleepVelocity=i.sleepVelocity??.015,this.sleepFrames=i.sleepFrames??120,this.activeStart=0,this.activeEnd=e-1,this.collisionStartSegment=0,this.collisionEndSegment=e-2,this.sleepCounter=0,this.sleeping=!1,this.x=new Float32Array(e),this.y=new Float32Array(e),this.z=new Float32Array(e),this.previousX=new Float32Array(e),this.previousY=new Float32Array(e),this.previousZ=new Float32Array(e),this.velocityX=new Float32Array(e),this.velocityY=new Float32Array(e),this.velocityZ=new Float32Array(e),this.forceX=new Float32Array(e),this.forceY=new Float32Array(e),this.forceZ=new Float32Array(e),this.inverseMass=new Float32Array(e),this.nodeRadius=new Float32Array(e),this.pinned=new Uint8Array(e),this.controlEnabled=new Uint8Array(e),this.controlX=new Float32Array(e),this.controlY=new Float32Array(e),this.controlZ=new Float32Array(e),this.controlCompliance=new Float32Array(e),this.restShapeEnabled=new Uint8Array(e),this.restShapeX=new Float32Array(e),this.restShapeY=new Float32Array(e),this.restShapeZ=new Float32Array(e),this.restShapeCompliance=new Float32Array(e),this.restLength=new Float32Array(this.segmentCount),this.restBendChord=new Float32Array(e),this.lengthLambda=new Float32Array(this.segmentCount),this.lengthNormalX=new Float32Array(this.segmentCount),this.lengthNormalY=new Float32Array(this.segmentCount),this.lengthNormalZ=new Float32Array(this.segmentCount),this.lengthLower=new Float32Array(this.segmentCount),this.lengthUpper=new Float32Array(this.segmentCount),this.lengthRhs=new Float32Array(this.segmentCount),this.lengthSolution=new Float32Array(this.segmentCount),this.bendLambda=new Float32Array(e),this.bendComplianceByNode=new Float32Array(e),this.controlLambda=new Float32Array(e),this.shapeLambda=new Float32Array(e),this.wallLambda=new Float32Array(this.segmentCount),this.wallActive=new Uint8Array(this.segmentCount),this.wallT=new Float32Array(this.segmentCount),this.wallX=new Float32Array(this.segmentCount),this.wallY=new Float32Array(this.segmentCount),this.wallZ=new Float32Array(this.segmentCount),this.wallNormalX=new Float32Array(this.segmentCount),this.wallNormalY=new Float32Array(this.segmentCount),this.wallNormalZ=new Float32Array(this.segmentCount),this.wallBranchId=new Int32Array(this.segmentCount),this.wallGap=new Float32Array(this.segmentCount),this.wallQueryStartX=new Float32Array(this.segmentCount),this.wallQueryStartY=new Float32Array(this.segmentCount),this.wallQueryStartZ=new Float32Array(this.segmentCount),this.wallQueryEndX=new Float32Array(this.segmentCount),this.wallQueryEndY=new Float32Array(this.segmentCount),this.wallQueryEndZ=new Float32Array(this.segmentCount),this.wallCorrectionX=new Float32Array(e),this.wallCorrectionY=new Float32Array(e),this.wallCorrectionZ=new Float32Array(e),this.wallCorrectionWeight=new Float32Array(e),this.wallBranchId.fill(-1),this.wallGap.fill(1/0),this.nodeRadius.fill(this.radius),this.inverseMass.fill(1/Math.max(ee,this.mass)),this.restLength.fill(n),this.bendComplianceByNode.fill(this.bendCompliance);for(let s=0;s<e;s++)this.x[s]=s*n;this.captureRestConfiguration(),this.copyCurrentToPrevious()}setNodePosition(t,e,n,i,s=!0){return this.x[t]=e,this.y[t]=n,this.z[t]=i,this.previousX[t]=e,this.previousY[t]=n,this.previousZ[t]=i,s&&(this.velocityX[t]=0,this.velocityY[t]=0,this.velocityZ[t]=0),this.wake(),this}setPinned(t,e=!0){return this.pinned[t]=e?1:0,this.inverseMass[t]=e?0:1/Math.max(ee,this.mass),this.wake(),this}setActiveRange(t,e){const n=He(Math.floor(t),0,this.count-1),i=He(Math.ceil(e),n,this.count-1);if(n<this.activeStart)for(let s=n;s<this.activeStart;s++)this.previousX[s]=this.x[s],this.previousY[s]=this.y[s],this.previousZ[s]=this.z[s],this.velocityX[s]=0,this.velocityY[s]=0,this.velocityZ[s]=0;if(i>this.activeEnd)for(let s=this.activeEnd+1;s<=i;s++)this.previousX[s]=this.x[s],this.previousY[s]=this.y[s],this.previousZ[s]=this.z[s],this.velocityX[s]=0,this.velocityY[s]=0,this.velocityZ[s]=0;return(n!==this.activeStart||i!==this.activeEnd)&&this.wake(),this.activeStart=n,this.activeEnd=i,this}setCollisionRange(t,e){const n=Math.floor(t),i=Math.floor(e),s=He(n,0,this.segmentCount-1);let a;return i<n||n>=this.segmentCount||i<0?a=s-1:a=He(i,s,this.segmentCount-1),(s!==this.collisionStartSegment||a!==this.collisionEndSegment)&&this.wake(),this.collisionStartSegment=s,this.collisionEndSegment=a,this}setControlTarget(t,e,n,i,s=0){const a=Math.max(0,s),o=!this.controlEnabled[t]||Math.abs(this.controlX[t]-e)>1e-6||Math.abs(this.controlY[t]-n)>1e-6||Math.abs(this.controlZ[t]-i)>1e-6||Math.abs(this.controlCompliance[t]-a)>1e-10;return this.controlEnabled[t]=1,this.controlX[t]=e,this.controlY[t]=n,this.controlZ[t]=i,this.controlCompliance[t]=a,o&&(this.controlLambda[t]=0,this.wake()),this}clearControlTarget(t){return this.controlEnabled[t]&&this.wake(),this.controlEnabled[t]=0,this.controlLambda[t]=0,this}setRestShapeTarget(t,e,n,i,s=this.shapeCompliance){const a=Math.max(0,s),o=!this.restShapeEnabled[t]||Math.abs(this.restShapeX[t]-e)>1e-6||Math.abs(this.restShapeY[t]-n)>1e-6||Math.abs(this.restShapeZ[t]-i)>1e-6||Math.abs(this.restShapeCompliance[t]-a)>1e-10;return this.restShapeEnabled[t]=1,this.restShapeX[t]=e,this.restShapeY[t]=n,this.restShapeZ[t]=i,this.restShapeCompliance[t]=a,o&&(this.shapeLambda[t]=0,this.wake()),this}clearRestShapeTarget(t){return this.restShapeEnabled[t]&&this.wake(),this.restShapeEnabled[t]=0,this.shapeLambda[t]=0,this}captureRestConfiguration(){for(let t=0;t<this.segmentCount;t++)this.restLength[t]=be(this.x[t+1]-this.x[t],this.y[t+1]-this.y[t],this.z[t+1]-this.z[t])||this.segmentLength;for(let t=1;t<this.count-1;t++)this.restBendChord[t]=be(this.x[t+1]-this.x[t-1],this.y[t+1]-this.y[t-1],this.z[t+1]-this.z[t-1]);return this.lengthLambda.fill(0),this.bendLambda.fill(0),this}copyCurrentToPrevious(){this.previousX.set(this.x),this.previousY.set(this.y),this.previousZ.set(this.z)}wake(){this.sleeping=!1,this.sleepCounter=0}syncFromElasticRod(t,{resetVelocity:e=!1,preservePrevious:n=!1}={}){const i=t.nodeStorage,s=Math.min(this.count,t.nodes.length);let a=!1;for(let o=0;o<s;o++)a=a||Math.abs(this.x[o]-i.x[o])>1e-6||Math.abs(this.y[o]-i.y[o])>1e-6||Math.abs(this.z[o]-i.z[o])>1e-6||Math.abs(this.velocityX[o]-i.vx[o])>1e-5||Math.abs(this.velocityY[o]-i.vy[o])>1e-5||Math.abs(this.velocityZ[o]-i.vz[o])>1e-5,n&&(this.previousX[o]=this.x[o],this.previousY[o]=this.y[o],this.previousZ[o]=this.z[o]),this.x[o]=i.x[o],this.y[o]=i.y[o],this.z[o]=i.z[o],this.velocityX[o]=e?0:i.vx[o],this.velocityY[o]=e?0:i.vy[o],this.velocityZ[o]=e?0:i.vz[o],this.inverseMass[o]=i.pinned[o]?0:1/Math.max(ee,i.mass[o]),this.pinned[o]=i.pinned[o],this.bendComplianceByNode[o]=He(this.bendCompliance*32/Math.max(.1,i.bendingStiffness[o]),this.bendCompliance*.125,this.bendCompliance*8);return n||this.copyCurrentToPrevious(),a&&this.wake(),this}syncToElasticRod(t){const e=t.nodeStorage,n=Math.min(this.count,t.nodes.length);for(let i=0;i<n;i++)e.x[i]=this.x[i],e.y[i]=this.y[i],e.z[i]=this.z[i],e.vx[i]=this.velocityX[i],e.vy[i]=this.velocityY[i],e.vz[i]=this.velocityZ[i];return this}}class wE{constructor({contactField:t=null,fixedDt:e=ME,maxSubsteps:n=2,iterations:i=6,penetrationIterations:s=8,highPenetration:a=.15,contactActivation:o=.25}={}){this.contactField=t,this.fixedDt=e,this.maxSubsteps=n,this.iterations=i,this.penetrationIterations=s,this.highPenetration=a,this.contactActivation=o,this.accumulator=0,this.bodies=[],this.sheaths=[],this.containments=[],this.toolContacts=[],this.stepCount=0,this.contactCount=0,this.maxPenetration=0,this.settledMaxPenetration=0,this.settledContactBodyId=null,this.settledContactSegment=-1,this.settledContactT=0,this.settledContactX=0,this.settledContactY=0,this.settledContactZ=0,this.lastSubsteps=0,this.droppedTime=0,this._queryStart={x:0,y:0,z:0},this._queryEnd={x:0,y:0,z:0},this._segmentParameters={s:0,t:0},this._contact=Rf(),this._sweep=Rf(),this.timings={total:io(),integrate:io(),narrowPhase:io(),constraints:io(),velocity:io()}}createRod(t,e,n,i={}){const s=new Nf(t,e,n,i);return this.bodies.push(s),s}addRod(t){if(!(t instanceof Nf))throw new TypeError("EndovascularRodBody is required");return this.bodies.includes(t)||this.bodies.push(t),t}addSheath({id:t="sheath",start:e,end:n,innerRadius:i=Tu.sheath.innerRadius,bodies:s=null}={}){const a=n.x-e.x,o=n.y-e.y,c=n.z-e.z,l=be(a,o,c);if(l<ee)throw new RangeError("Sheath axis must have positive length");const h={id:t,startX:e.x,startY:e.y,startZ:e.z,axisX:a/l,axisY:o/l,axisZ:c/l,length:l,innerRadius:i,bodies:s,lambdas:new Map};return this.sheaths.push(h),h}addContainment(t,e,{innerRadius:n=e.innerRadius,compliance:i=0,friction:s=e.lumenFriction,enabled:a=!0,openProximal:o=!0,openDistal:c=!0,searchWindow:l=10,outerStartNode:h=e.activeStart,startNode:u=t.activeStart,endNode:f=t.activeEnd}={}){const d={innerBody:t,outerBody:e,innerRadius:n,compliance:i,friction:s,enabled:a,openProximal:o,openDistal:c,searchWindow:l,outerStartNode:h,startNode:u,endNode:f,lambdas:new Float32Array(t.count),closestSegment:new Int32Array(t.count),_lastEnabled:a,_lastOuterStartNode:h,_lastStartNode:u,_lastEndNode:f};return d.closestSegment.fill(-1),this.containments.push(d),d}addToolContact(t,e,{compliance:n=0,friction:i=.06,enabled:s=!0,openDistalB:a=!1,startSegmentA:o=0,endSegmentA:c=t.segmentCount-1,startSegmentB:l=0,endSegmentB:h=e.segmentCount-1}={}){const u=t.segmentCount*e.segmentCount,f={bodyA:t,bodyB:e,compliance:n,friction:i,enabled:s,openDistalB:a,startSegmentA:o,endSegmentA:c,startSegmentB:l,endSegmentB:h,lambdas:new Float32Array(u),_lastEnabled:s,_lastStartSegmentA:o,_lastEndSegmentA:c,_lastStartSegmentB:l,_lastEndSegmentB:h};return this.toolContacts.push(f),f}advance(t,e=null){const n=Math.max(0,Math.min(.25,t));this.accumulator+=n;let i=0;for(;this.accumulator+ee>=this.fixedDt&&i<this.maxSubsteps;)e?.(this.fixedDt,i),this.stepFixed(),this.accumulator-=this.fixedDt,i++;return this.accumulator>=this.fixedDt&&(this.droppedTime+=this.accumulator-this.accumulator%this.fixedDt,this.accumulator%=this.fixedDt),this.lastSubsteps=i,i}stepFixed(){const t=On();this.contactCount=0,this.maxPenetration=0;let e=On();for(let a=0;a<this.bodies.length;a++){const o=this.bodies[a];o.lengthLambda.fill(0),o.bendLambda.fill(0),o.controlLambda.fill(0),o.shapeLambda.fill(0),this.#t(o)}so(this.timings.integrate,On()-e),e=On();for(let a=0;a<this.bodies.length;a++)this.#n(this.bodies[a]);for(let a=0;a<this.bodies.length;a++)this.#i(this.bodies[a]);let n=On()-e;e=On();const i=this.maxPenetration>this.highPenetration?this.penetrationIterations:this.iterations;for(let a=0;a<i;a++){for(let o=0;o<this.sheaths.length;o++)this.#T(this.sheaths[o]);for(let o=0;o<this.bodies.length;o++)this.#h(this.bodies[o]);for(let o=0;o<this.bodies.length;o++)this.#m(this.bodies[o],(a&1)===1);for(let o=0;o<this.bodies.length;o++)this.#r(this.bodies[o]);for(let o=0;o<this.bodies.length;o++)this.#s(this.bodies[o]);for(let o=0;o<this.containments.length;o++)this.#A(this.containments[o]);for(let o=0;o<this.toolContacts.length;o++)this.#u(this.toolContacts[o]);for(let o=0;o<this.bodies.length;o++)this.#M(this.bodies[o]);for(let o=0;o<this.bodies.length;o++)this.#S(this.bodies[o])}for(let a=0;a<8;a++){for(let c=0;c<this.bodies.length;c++)this.#M(this.bodies[c]);for(let c=0;c<this.bodies.length;c++)this.#S(this.bodies[c]);for(let c=0;c<this.bodies.length;c++)this.#g(this.bodies[c]);let o=!0;for(let c=0;c<this.bodies.length;c++)o=o&&!this.#a(this.bodies[c],.002);if(o)break}for(let a=0;a<Df;a++){let o=0;for(let c=0;c<this.bodies.length;c++)o=Math.max(o,this.#l(this.bodies[c]));if(o<=.02)break;for(let c=0;c<this.bodies.length;c++)this.#S(this.bodies[c]),this.#y(this.bodies[c]),a+1<Df&&this.#g(this.bodies[c])}so(this.timings.constraints,On()-e);const s=this.maxPenetration;e=On(),this.contactCount=0,this.maxPenetration=0,this.settledContactBodyId=null,this.settledContactSegment=-1;for(let a=0;a<this.bodies.length;a++)this.#l(this.bodies[a]);this.settledMaxPenetration=this.maxPenetration,this.maxPenetration=Math.max(s,this.settledMaxPenetration),n+=On()-e,so(this.timings.narrowPhase,n),e=On();for(let a=0;a<this.bodies.length;a++)this.#E(this.bodies[a]);so(this.timings.velocity,On()-e),this.stepCount++,so(this.timings.total,On()-t)}resetPerformanceStats(){this.contactCount=0,this.maxPenetration=0,this.settledMaxPenetration=0;for(const t of Object.values(this.timings))t.samples.fill(0),t.cursor=0,t.count=0,t.recordedCount=0,t.total=0,t.last=0}resetSimulationState(){this.accumulator=0,this.stepCount=0,this.lastSubsteps=0,this.droppedTime=0;for(const t of this.bodies)t.lengthLambda.fill(0),t.bendLambda.fill(0),t.controlLambda.fill(0),t.shapeLambda.fill(0),t.wallLambda.fill(0),t.wallActive.fill(0),t.wallBranchId.fill(-1),t.wallGap.fill(1/0),t.copyCurrentToPrevious(),t.wake();for(const t of this.sheaths)t.lambdas.clear();for(const t of this.containments)t.lambdas.fill(0),t.closestSegment.fill(-1),t._lastEnabled=t.enabled,t._lastOuterStartNode=t.outerStartNode,t._lastStartNode=t.startNode,t._lastEndNode=t.endNode;for(const t of this.toolContacts)t.lambdas.fill(0),t._lastEnabled=t.enabled,t._lastStartSegmentA=t.startSegmentA,t._lastEndSegmentA=t.endSegmentA,t._lastStartSegmentB=t.startSegmentB,t._lastEndSegmentB=t.endSegmentB;return this.resetPerformanceStats(),this}getStats(){const t=this.bodies.map(e=>this.#w(e));return{mode:"xpbd-contact-v1",fixedDt:this.fixedDt,steps:this.stepCount,lastSubsteps:this.lastSubsteps,droppedTime:this.droppedTime,contacts:this.contactCount,maxPenetration:this.maxPenetration,settledMaxPenetration:this.settledMaxPenetration,settledContact:{bodyId:this.settledContactBodyId,segment:this.settledContactSegment,t:this.settledContactT,x:this.settledContactX,y:this.settledContactY,z:this.settledContactZ},phases:{total:ro(this.timings.total),integrate:ro(this.timings.integrate),narrowPhase:ro(this.timings.narrowPhase),constraints:ro(this.timings.constraints),velocity:ro(this.timings.velocity)},bodies:t}}#t(t){if(t.sleeping)return;const e=this.fixedDt,n=e*e,i=t.activeStart,s=t.activeEnd;for(let a=i;a<=s;a++)t.previousX[a]=t.x[a],t.previousY[a]=t.y[a],t.previousZ[a]=t.z[a],!(t.inverseMass[a]<=0)&&(t.velocityX[a]*=t.linearDamping,t.velocityY[a]*=t.linearDamping,t.velocityZ[a]*=t.linearDamping,t.x[a]+=t.velocityX[a]*e+t.forceX[a]*t.inverseMass[a]*n,t.y[a]+=t.velocityY[a]*e+t.forceY[a]*t.inverseMass[a]*n,t.z[a]+=t.velocityZ[a]*e+t.forceZ[a]*t.inverseMass[a]*n);t.forceX.fill(0),t.forceY.fill(0),t.forceZ.fill(0)}#n(t){if(!this.contactField||t.sleeping||t.collisionEndSegment<t.collisionStartSegment)return;const e=Math.max(t.activeStart,t.collisionStartSegment),n=Math.min(t.activeEnd,t.collisionEndSegment+1);for(let i=e;i<=n;i++){const s=t.x[i]-t.previousX[i],a=t.y[i]-t.previousY[i],o=t.z[i]-t.previousZ[i],c=t.nodeRadius[i];if(s*s+a*a+o*o<=c*c*.25)continue;this._queryStart.x=t.previousX[i],this._queryStart.y=t.previousY[i],this._queryStart.z=t.previousZ[i],this._queryEnd.x=t.x[i],this._queryEnd.y=t.y[i],this._queryEnd.z=t.z[i];const l=this.contactField.sweepSphere(this._queryStart,this._queryEnd,c,this._sweep);if(!l.violation||l.timeOfImpact>=1)continue;const h=Math.max(0,l.timeOfImpact-.001);t.x[i]=t.previousX[i]+s*h+l.inward.x*.001,t.y[i]=t.previousY[i]+a*h+l.inward.y*.001,t.z[i]=t.previousZ[i]+o*h+l.inward.z*.001}}#i(t){if(!this.contactField||t.sleeping||t.collisionEndSegment<t.collisionStartSegment)return;const e=Math.max(t.activeStart,t.collisionStartSegment,0),n=Math.min(t.activeEnd,t.collisionEndSegment+1,t.segmentCount);for(let i=e;i<n;i++){const s=t.wallActive[i]!==0;if(t.wallActive[i]=0,s){const g=t.wallT[i],x=t.x[i]+(t.x[i+1]-t.x[i])*g,m=t.y[i]+(t.y[i+1]-t.y[i])*g,p=t.z[i]+(t.z[i+1]-t.z[i])*g,_=Math.max(t.nodeRadius[i],t.nodeRadius[i+1]),v=(x-t.wallX[i])*t.wallNormalX[i]+(m-t.wallY[i])*t.wallNormalY[i]+(p-t.wallZ[i])*t.wallNormalZ[i]-_;if(v<=this.contactActivation+.1){t.wallActive[i]=1,v<0&&(this.contactCount++,this.maxPenetration=Math.max(this.maxPenetration,-v));continue}}const a=t.wallGap[i];if(!s&&Number.isFinite(a)){const g=t.x[i]-t.wallQueryStartX[i],x=t.y[i]-t.wallQueryStartY[i],m=t.z[i]-t.wallQueryStartZ[i],p=t.x[i+1]-t.wallQueryEndX[i],_=t.y[i+1]-t.wallQueryEndY[i],v=t.z[i+1]-t.wallQueryEndZ[i],S=Math.sqrt(g*g+x*x+m*m),y=Math.sqrt(p*p+_*_+v*v);if(a-Math.max(S,y)>this.contactActivation){t.wallLambda[i]*=.5;continue}}let o;if(this.contactField.queryCapsuleSoA)o=this.contactField.queryCapsuleSoA(t.x,t.y,t.z,t.nodeRadius,i,this._contact);else{const g=Math.max(t.nodeRadius[i],t.nodeRadius[i+1]);this._queryStart.x=t.x[i],this._queryStart.y=t.y[i],this._queryStart.z=t.z[i],this._queryEnd.x=t.x[i+1],this._queryEnd.y=t.y[i+1],this._queryEnd.z=t.z[i+1],o=this.contactField.queryCapsule(this._queryStart,this._queryEnd,g,this._contact)}const c=o.values,l=c[Pf],h=c[If],u=c[Lf],f=o.closestPoint.values,d=o.inward.values;if(t.wallGap[i]=l,t.wallQueryStartX[i]=t.x[i],t.wallQueryStartY[i]=t.y[i],t.wallQueryStartZ[i]=t.z[i],t.wallQueryEndX[i]=t.x[i+1],t.wallQueryEndY[i]=t.y[i+1],t.wallQueryEndZ[i]=t.z[i+1],l>this.contactActivation){t.wallLambda[i]*=.5;continue}t.wallBranchId[i]!==u&&(t.wallLambda[i]=0),t.wallActive[i]=1,t.wallT[i]=h,t.wallX[i]=f[0],t.wallY[i]=f[1],t.wallZ[i]=f[2],t.wallNormalX[i]=d[0],t.wallNormalY[i]=d[1],t.wallNormalZ[i]=d[2],t.wallBranchId[i]=u,o.violation&&(this.contactCount++,this.maxPenetration=Math.max(this.maxPenetration,c[Fa]))}}#l(t){if(!this.contactField||t.sleeping||t.collisionEndSegment<t.collisionStartSegment)return 0;const e=Math.max(t.activeStart,t.collisionStartSegment,0),n=Math.min(t.activeEnd,t.collisionEndSegment+1,t.segmentCount);let i=0;for(let s=e;s<n;s++){if(!t.wallActive[s])continue;const a=t.wallGap[s];if(Number.isFinite(a)){const g=t.x[s]-t.wallQueryStartX[s],x=t.y[s]-t.wallQueryStartY[s],m=t.z[s]-t.wallQueryStartZ[s],p=t.x[s+1]-t.wallQueryEndX[s],_=t.y[s+1]-t.wallQueryEndY[s],v=t.z[s+1]-t.wallQueryEndZ[s],S=Math.sqrt(g*g+x*x+m*m),y=Math.sqrt(p*p+_*_+v*v),M=a-Math.max(S,y);if(M>.02){M>this.contactActivation&&(t.wallActive[s]=0,t.wallLambda[s]*=.5);continue}}let o;if(this.contactField.queryCapsuleSoA)o=this.contactField.queryCapsuleSoA(t.x,t.y,t.z,t.nodeRadius,s,this._contact);else{const g=Math.max(t.nodeRadius[s],t.nodeRadius[s+1]);this._queryStart.x=t.x[s],this._queryStart.y=t.y[s],this._queryStart.z=t.z[s],this._queryEnd.x=t.x[s+1],this._queryEnd.y=t.y[s+1],this._queryEnd.z=t.z[s+1],o=this.contactField.queryCapsule(this._queryStart,this._queryEnd,g,this._contact)}const c=o.values,l=c[Pf],h=c[If],u=c[Lf],f=o.closestPoint.values,d=o.inward.values;if(t.wallGap[s]=l,t.wallQueryStartX[s]=t.x[s],t.wallQueryStartY[s]=t.y[s],t.wallQueryStartZ[s]=t.z[s],t.wallQueryEndX[s]=t.x[s+1],t.wallQueryEndY[s]=t.y[s+1],t.wallQueryEndZ[s]=t.z[s+1],l>this.contactActivation){t.wallActive[s]=0,t.wallLambda[s]*=.5;continue}t.wallBranchId[s]!==u&&(t.wallLambda[s]=0),t.wallT[s]=h,t.wallX[s]=f[0],t.wallY[s]=f[1],t.wallZ[s]=f[2],t.wallNormalX[s]=d[0],t.wallNormalY[s]=d[1],t.wallNormalZ[s]=d[2],t.wallBranchId[s]=u,o.violation&&(this.contactCount++,c[Fa]>this.maxPenetration&&(this.settledContactBodyId=t.id,this.settledContactSegment=s,this.settledContactT=h,this.settledContactX=t.x[s]+(t.x[s+1]-t.x[s])*h,this.settledContactY=t.y[s]+(t.y[s+1]-t.y[s])*h,this.settledContactZ=t.z[s]+(t.z[s+1]-t.z[s])*h),this.maxPenetration=Math.max(this.maxPenetration,c[Fa]),i=Math.max(i,c[Fa]))}return i}#a(t,e){const n=Math.max(0,t.activeStart),i=Math.min(t.segmentCount,t.activeEnd);for(let s=n;s<i;s++){const a=be(t.x[s+1]-t.x[s],t.y[s+1]-t.y[s],t.z[s+1]-t.z[s]);if(Math.abs(a-t.restLength[s])>t.restLength[s]*e)return!0}return!1}#h(t){if(t.sleeping)return;const e=this.fixedDt*this.fixedDt;for(let n=t.activeStart;n<=t.activeEnd;n++){if(!t.controlEnabled[n]||t.inverseMass[n]<=0)continue;const i=t.x[n]-t.controlX[n],s=t.y[n]-t.controlY[n],a=t.z[n]-t.controlZ[n],o=be(i,s,a);if(o<ee)continue;const c=t.controlCompliance[n]/e,l=(-o-c*t.controlLambda[n])/(t.inverseMass[n]+c);t.controlLambda[n]+=l;const h=l/o*t.inverseMass[n];t.x[n]+=i*h,t.y[n]+=s*h,t.z[n]+=a*h}}#m(t,e=!1){if(t.sleeping)return;const n=t.stretchCompliance/(this.fixedDt*this.fixedDt),i=Math.max(0,t.activeStart),s=Math.min(t.segmentCount,t.activeEnd);for(let a=e?s-1:i;e?a>=i:a<s;a+=e?-1:1){const o=t.x[a+1]-t.x[a],c=t.y[a+1]-t.y[a],l=t.z[a+1]-t.z[a],h=be(o,c,l);if(h<ee)continue;const u=t.inverseMass[a],f=t.inverseMass[a+1],d=u+f+n;if(d<ee)continue;const x=(-(h-t.restLength[a])-n*t.lengthLambda[a])/d;t.lengthLambda[a]+=x;const m=o/h,p=c/h,_=l/h;t.x[a]-=m*x*u,t.y[a]-=p*x*u,t.z[a]-=_*x*u,t.x[a+1]+=m*x*f,t.y[a+1]+=p*x*f,t.z[a+1]+=_*x*f}}#g(t){if(t.sleeping)return;const e=Math.max(0,t.activeStart),i=Math.min(t.segmentCount,t.activeEnd)-e;if(i<=0)return;for(let a=0;a<i;a++){const o=e+a,c=t.x[o+1]-t.x[o],l=t.y[o+1]-t.y[o],h=t.z[o+1]-t.z[o],u=be(c,l,h);u<ee?(t.lengthNormalX[a]=1,t.lengthNormalY[a]=0,t.lengthNormalZ[a]=0,t.lengthRhs[a]=0):(t.lengthNormalX[a]=c/u,t.lengthNormalY[a]=l/u,t.lengthNormalZ[a]=h/u,t.lengthRhs[a]=-(u-t.restLength[o]))}for(let a=0;a<i;a++){const o=e+a;let c=0,l=0;a>0&&(c=-t.inverseMass[o]*(t.lengthNormalX[a]*t.lengthNormalX[a-1]+t.lengthNormalY[a]*t.lengthNormalY[a-1]+t.lengthNormalZ[a]*t.lengthNormalZ[a-1])),a+1<i&&(l=-t.inverseMass[o+1]*(t.lengthNormalX[a]*t.lengthNormalX[a+1]+t.lengthNormalY[a]*t.lengthNormalY[a+1]+t.lengthNormalZ[a]*t.lengthNormalZ[a+1])),t.lengthLower[a]=c,t.lengthUpper[a]=l,t.lengthSolution[a]=t.inverseMass[o]+t.inverseMass[o+1]}let s=Math.max(ee,t.lengthSolution[0]);t.lengthUpper[0]/=s,t.lengthRhs[0]/=s;for(let a=1;a<i;a++)s=Math.max(ee,t.lengthSolution[a]-t.lengthLower[a]*t.lengthUpper[a-1]),t.lengthUpper[a]=a+1<i?t.lengthUpper[a]/s:0,t.lengthRhs[a]=(t.lengthRhs[a]-t.lengthLower[a]*t.lengthRhs[a-1])/s;t.lengthSolution[i-1]=t.lengthRhs[i-1];for(let a=i-2;a>=0;a--)t.lengthSolution[a]=t.lengthRhs[a]-t.lengthUpper[a]*t.lengthSolution[a+1];for(let a=0;a<i;a++){const o=e+a,c=t.lengthSolution[a],l=t.lengthNormalX[a],h=t.lengthNormalY[a],u=t.lengthNormalZ[a];t.x[o]-=l*c*t.inverseMass[o],t.y[o]-=h*c*t.inverseMass[o],t.z[o]-=u*c*t.inverseMass[o],t.x[o+1]+=l*c*t.inverseMass[o+1],t.y[o+1]+=h*c*t.inverseMass[o+1],t.z[o+1]+=u*c*t.inverseMass[o+1]}}#r(t){if(t.sleeping||t.count<3)return;const e=Math.max(1,t.activeStart+1),n=Math.min(t.count-1,t.activeEnd);for(let i=e;i<n;i++){const s=i-1,a=i+1,o=t.x[a]-t.x[s],c=t.y[a]-t.y[s],l=t.z[a]-t.z[s],h=be(o,c,l);if(h<ee)continue;const u=t.inverseMass[s],f=t.inverseMass[a],d=t.bendComplianceByNode[i]/(this.fixedDt*this.fixedDt),g=u+f+d;if(g<ee)continue;const m=(-(h-t.restBendChord[i])-d*t.bendLambda[i])/g;t.bendLambda[i]+=m;const p=o/h,_=c/h,v=l/h;t.x[s]-=p*m*u,t.y[s]-=_*m*u,t.z[s]-=v*m*u,t.x[a]+=p*m*f,t.y[a]+=_*m*f,t.z[a]+=v*m*f}}#s(t){if(t.sleeping)return;const e=this.fixedDt*this.fixedDt;for(let n=t.activeStart;n<=t.activeEnd;n++){if(!t.restShapeEnabled[n]||t.inverseMass[n]<=0)continue;const i=t.x[n]-t.restShapeX[n],s=t.y[n]-t.restShapeY[n],a=t.z[n]-t.restShapeZ[n],o=be(i,s,a);if(o<ee)continue;const c=t.restShapeCompliance[n]/e,l=(-o-c*t.shapeLambda[n])/(t.inverseMass[n]+c);t.shapeLambda[n]+=l;const h=l/o*t.inverseMass[n];t.x[n]+=i*h,t.y[n]+=s*h,t.z[n]+=a*h}}#S(t){if(t.sleeping||t.count<3||t.foldLimitStrength<=0)return;const e=Math.max(1,t.activeStart+1),n=Math.min(t.count-1,t.activeEnd),i=He(t.maxBendAngle,1,179)*Math.PI/180,s=Math.cos(i);for(let a=e;a<n;a++){const o=a-1,c=a+1,l=t.x[a]-t.x[o],h=t.y[a]-t.y[o],u=t.z[a]-t.z[o],f=t.x[c]-t.x[a],d=t.y[c]-t.y[a],g=t.z[c]-t.z[a],x=be(l,h,u),m=be(f,d,g);if(x<ee||m<ee)continue;const p=(l*f+h*d+u*g)/(x*m);if(p>=s)continue;let _=t.x[c]-t.x[o],v=t.y[c]-t.y[o],S=t.z[c]-t.z[o];const y=be(_,v,S);y<ee?(_=l/x,v=h/x,S=u/x):(_/=y,v/=y,S/=y);const w=Math.sqrt(Math.max(0,x*x+m*m+2*x*m*s))-y;if(w<=0)continue;const T=t.inverseMass[o],E=t.inverseMass[c],A=T+E;if(A<ee)continue;const D=w*t.foldLimitStrength,R=D*T/A,N=D*E/A;if(t.x[o]-=_*R,t.y[o]-=v*R,t.z[o]-=S*R,t.x[c]+=_*N,t.y[c]+=v*N,t.z[c]+=S*N,t.inverseMass[a]>0){const L=t.foldLimitStrength*.45;if(t.x[a]+=((t.x[o]+t.x[c])*.5-t.x[a])*L,t.y[a]+=((t.y[o]+t.y[c])*.5-t.y[a])*L,t.z[a]+=((t.z[o]+t.z[c])*.5-t.z[a])*L,p<-.999&&y<Math.min(x,m)*.1){const F=l/x,z=h/x,q=u/x;let O,Y,K;Math.abs(F)<.8?(O=0,Y=q,K=-z):(O=-q,Y=0,K=F);const J=be(O,Y,K)||1,it=Math.min(x,m)*t.foldLimitStrength*.05;t.x[a]+=O/J*it,t.y[a]+=Y/J*it,t.z[a]+=K/J*it}}}}#A(t){if((t.enabled!==t._lastEnabled||t.outerStartNode!==t._lastOuterStartNode||t.startNode!==t._lastStartNode||t.endNode!==t._lastEndNode)&&(t.lambdas.fill(0),t.closestSegment.fill(-1),t._lastEnabled=t.enabled,t._lastOuterStartNode=t.outerStartNode,t._lastStartNode=t.startNode),t._lastEndNode=t.endNode,!t.enabled)return;const e=t.innerBody,n=t.outerBody,i=Math.max(0,t.innerRadius-e.radius),s=t.compliance/(this.fixedDt*this.fixedDt),a=He(t.outerStartNode,n.activeStart,n.activeEnd),o=Math.min(n.activeEnd,n.segmentCount);if(o<=a)return;const c=He(t.startNode,e.activeStart,e.activeEnd),l=He(t.endNode,c,e.activeEnd);let h=a,u=0,f=n.restLength[a];for(let d=c;d<=l;d++){for(d>c&&(u+=e.restLength[d-1]);h<o-1&&f<u;)h++,f+=n.restLength[h];let g=Math.max(a,h-t.searchWindow),x=Math.min(o-1,h+t.searchWindow);const m=t.closestSegment[d];if(m>=a&&m<o){const rt=Math.abs(m-h)<=t.searchWindow?m:h;g=Math.max(a,rt-1),x=Math.min(o-1,rt+1)}let p=1/0,_=-1,v=0,S=0,y=0,M=0;for(let rt=g;rt<=x;rt++){const ht=n.x[rt],dt=n.y[rt],bt=n.z[rt],V=n.x[rt+1]-ht,re=n.y[rt+1]-dt,Tt=n.z[rt+1]-bt,It=V*V+re*re+Tt*Tt,vt=He(((e.x[d]-ht)*V+(e.y[d]-dt)*re+(e.z[d]-bt)*Tt)/Math.max(ee,It),0,1),Jt=ht+V*vt,Rt=dt+re*vt,I=bt+Tt*vt,C=e.x[d]-Jt,H=e.y[d]-Rt,Q=e.z[d]-I,$=C*C+H*H+Q*Q;$<p&&(p=$,_=rt,v=vt,S=Jt,y=Rt,M=I)}if(_<0)continue;if(t.closestSegment[d]=_,t.openProximal&&_===a&&v<=1e-5){const rt=n.x[a+1]-n.x[a],ht=n.y[a+1]-n.y[a],dt=n.z[a+1]-n.z[a];if((e.x[d]-n.x[a])*rt+(e.y[d]-n.y[a])*ht+(e.z[d]-n.z[a])*dt<0)continue}if(t.openDistal&&_===o-1&&v>=1-1e-5){const rt=n.x[o]-n.x[o-1],ht=n.y[o]-n.y[o-1],dt=n.z[o]-n.z[o-1];if((e.x[d]-n.x[o])*rt+(e.y[d]-n.y[o])*ht+(e.z[d]-n.z[o])*dt>0)continue}const w=Math.sqrt(p);if(w<=i||w<ee){t.lambdas[d]*=.8;continue}const T=(e.x[d]-S)/w,E=(e.y[d]-y)/w,A=(e.z[d]-M)/w,D=e.inverseMass[d],R=1-v,N=v,L=n.inverseMass[_]*R*R,F=n.inverseMass[_+1]*N*N,z=D+L+F+s;if(z<ee)continue;const O=(-(i-w)-s*t.lambdas[d])/z;t.lambdas[d]+=O,e.x[d]-=T*O*D,e.y[d]-=E*O*D,e.z[d]-=A*O*D,n.x[_]+=T*O*n.inverseMass[_]*R,n.y[_]+=E*O*n.inverseMass[_]*R,n.z[_]+=A*O*n.inverseMass[_]*R,n.x[_+1]+=T*O*n.inverseMass[_+1]*N,n.y[_+1]+=E*O*n.inverseMass[_+1]*N,n.z[_+1]+=A*O*n.inverseMass[_+1]*N;const Y=e.x[d]-e.previousX[d]-(n.x[_]-n.previousX[_])*R-(n.x[_+1]-n.previousX[_+1])*N,K=e.y[d]-e.previousY[d]-(n.y[_]-n.previousY[_])*R-(n.y[_+1]-n.previousY[_+1])*N,J=e.z[d]-e.previousZ[d]-(n.z[_]-n.previousZ[_])*R-(n.z[_+1]-n.previousZ[_+1])*N,it=Y*T+K*E+J*A;let X=Y-T*it,j=K-E*it,nt=J-A*it;const ot=be(X,j,nt),ut=D+L+F;if(ot>ee&&ut>ee&&t.friction>0){X/=ot,j/=ot,nt/=ot;const rt=-Math.min(ot/ut,t.friction*t.lambdas[d]);e.x[d]+=X*rt*D,e.y[d]+=j*rt*D,e.z[d]+=nt*rt*D,n.x[_]-=X*rt*n.inverseMass[_]*R,n.y[_]-=j*rt*n.inverseMass[_]*R,n.z[_]-=nt*rt*n.inverseMass[_]*R,n.x[_+1]-=X*rt*n.inverseMass[_+1]*N,n.y[_+1]-=j*rt*n.inverseMass[_+1]*N,n.z[_+1]-=nt*rt*n.inverseMass[_+1]*N}}}#u(t){if((t.enabled!==t._lastEnabled||t.startSegmentA!==t._lastStartSegmentA||t.endSegmentA!==t._lastEndSegmentA||t.startSegmentB!==t._lastStartSegmentB||t.endSegmentB!==t._lastEndSegmentB)&&(t.lambdas.fill(0),t._lastEnabled=t.enabled,t._lastStartSegmentA=t.startSegmentA,t._lastEndSegmentA=t.endSegmentA,t._lastStartSegmentB=t.startSegmentB,t._lastEndSegmentB=t.endSegmentB),!t.enabled)return;const e=t.bodyA,n=t.bodyB,i=t.compliance/(this.fixedDt*this.fixedDt),s=He(t.startSegmentA,e.activeStart,e.segmentCount-1),a=He(t.endSegmentA,s,Math.min(e.activeEnd-1,e.segmentCount-1)),o=He(t.startSegmentB,n.activeStart,n.segmentCount-1),c=He(t.endSegmentB,o,Math.min(n.activeEnd-1,n.segmentCount-1));for(let l=s;l<=a;l++)for(let h=o;h<=c;h++){const u=this.#_(e,l,n,h,this._segmentParameters),f=e.x[l]+(e.x[l+1]-e.x[l])*u.s,d=e.y[l]+(e.y[l+1]-e.y[l])*u.s,g=e.z[l]+(e.z[l+1]-e.z[l])*u.s,x=n.x[h]+(n.x[h+1]-n.x[h])*u.t,m=n.y[h]+(n.y[h+1]-n.y[h])*u.t,p=n.z[h]+(n.z[h+1]-n.z[h])*u.t;if(t.openDistalB&&h===c&&u.t>=1-1e-5){const ht=n.x[c+1]-n.x[c],dt=n.y[c+1]-n.y[c],bt=n.z[c+1]-n.z[c];if((f-n.x[c+1])*ht+(d-n.y[c+1])*dt+(g-n.z[c+1])*bt>0)continue}let _=f-x,v=d-m,S=g-p;const y=be(_,v,S),M=Math.max(e.nodeRadius[l],e.nodeRadius[l+1])+Math.max(n.nodeRadius[h],n.nodeRadius[h+1]);if(y>=M||y<ee)continue;_/=y,v/=y,S/=y;const w=1-u.s,T=u.s,E=1-u.t,A=u.t,D=e.inverseMass[l]*w*w,R=e.inverseMass[l+1]*T*T,N=n.inverseMass[h]*E*E,L=n.inverseMass[h+1]*A*A,F=D+R+N+L+i;if(F<ee)continue;const z=l*n.segmentCount+h;let O=(-(y-M)-i*t.lambdas[z])/F;const Y=Math.max(0,t.lambdas[z]+O);O=Y-t.lambdas[z],t.lambdas[z]=Y,e.x[l]+=_*O*e.inverseMass[l]*w,e.y[l]+=v*O*e.inverseMass[l]*w,e.z[l]+=S*O*e.inverseMass[l]*w,e.x[l+1]+=_*O*e.inverseMass[l+1]*T,e.y[l+1]+=v*O*e.inverseMass[l+1]*T,e.z[l+1]+=S*O*e.inverseMass[l+1]*T,n.x[h]-=_*O*n.inverseMass[h]*E,n.y[h]-=v*O*n.inverseMass[h]*E,n.z[h]-=S*O*n.inverseMass[h]*E,n.x[h+1]-=_*O*n.inverseMass[h+1]*A,n.y[h+1]-=v*O*n.inverseMass[h+1]*A,n.z[h+1]-=S*O*n.inverseMass[h+1]*A;const K=(e.x[l]-e.previousX[l])*w+(e.x[l+1]-e.previousX[l+1])*T-(n.x[h]-n.previousX[h])*E-(n.x[h+1]-n.previousX[h+1])*A,J=(e.y[l]-e.previousY[l])*w+(e.y[l+1]-e.previousY[l+1])*T-(n.y[h]-n.previousY[h])*E-(n.y[h+1]-n.previousY[h+1])*A,it=(e.z[l]-e.previousZ[l])*w+(e.z[l+1]-e.previousZ[l+1])*T-(n.z[h]-n.previousZ[h])*E-(n.z[h+1]-n.previousZ[h+1])*A,X=K*_+J*v+it*S;let j=K-_*X,nt=J-v*X,ot=it-S*X;const ut=be(j,nt,ot),rt=D+R+N+L;if(ut>ee&&rt>ee&&t.friction>0){j/=ut,nt/=ut,ot/=ut;const ht=-Math.min(ut/rt,t.friction*Y);e.x[l]+=j*ht*e.inverseMass[l]*w,e.y[l]+=nt*ht*e.inverseMass[l]*w,e.z[l]+=ot*ht*e.inverseMass[l]*w,e.x[l+1]+=j*ht*e.inverseMass[l+1]*T,e.y[l+1]+=nt*ht*e.inverseMass[l+1]*T,e.z[l+1]+=ot*ht*e.inverseMass[l+1]*T,n.x[h]-=j*ht*n.inverseMass[h]*E,n.y[h]-=nt*ht*n.inverseMass[h]*E,n.z[h]-=ot*ht*n.inverseMass[h]*E,n.x[h+1]-=j*ht*n.inverseMass[h+1]*A,n.y[h+1]-=nt*ht*n.inverseMass[h+1]*A,n.z[h+1]-=ot*ht*n.inverseMass[h+1]*A}}}#_(t,e,n,i,s){const a=t.x[e+1]-t.x[e],o=t.y[e+1]-t.y[e],c=t.z[e+1]-t.z[e],l=n.x[i+1]-n.x[i],h=n.y[i+1]-n.y[i],u=n.z[i+1]-n.z[i],f=t.x[e]-n.x[i],d=t.y[e]-n.y[i],g=t.z[e]-n.z[i],x=a*a+o*o+c*c,m=a*l+o*h+c*u,p=l*l+h*h+u*u,_=a*f+o*d+c*g,v=l*f+h*d+u*g,S=x*p-m*m;let y=S>ee?He((m*v-p*_)/S,0,1):0,M=p>ee?He((m*y+v)/p,0,1):0;return x>ee&&(y=He((m*M-_)/x,0,1)),s.s=y,s.t=M,s}#T(t){for(let e=0;e<this.bodies.length;e++){const n=this.bodies[e];if(t.bodies&&!t.bodies.includes(n))continue;let i=t.lambdas.get(n);i||(i=new Float32Array(n.count),t.lambdas.set(n,i));for(let s=n.activeStart;s<=n.activeEnd;s++){const a=n.x[s]-t.startX,o=n.y[s]-t.startY,c=n.z[s]-t.startZ,l=a*t.axisX+o*t.axisY+c*t.axisZ;if(l<=0||l>=t.length){i[s]*=.8;continue}const h=t.startX+t.axisX*l,u=t.startY+t.axisY*l,f=t.startZ+t.axisZ*l,d=n.x[s]-h,g=n.y[s]-u,x=n.z[s]-f,m=be(d,g,x),p=Math.max(0,t.innerRadius-n.nodeRadius[s]);if(m<=p||m<ee){i[s]*=.8;continue}const _=n.inverseMass[s];if(_<=0)continue;const S=-(p-m)/_;i[s]+=S,n.x[s]-=d/m*S*_,n.y[s]-=g/m*S*_,n.z[s]-=x/m*S*_}}}#M(t){if(t.sleeping)return;const e=t.wallCompliance/(this.fixedDt*this.fixedDt),n=Math.max(0,t.activeStart,t.collisionStartSegment),i=Math.min(t.activeEnd,t.collisionEndSegment+1,t.segmentCount);for(let s=n;s<i;s++){if(!t.wallActive[s])continue;const a=t.wallT[s],o=1-a,c=a,l=t.x[s]*o+t.x[s+1]*c,h=t.y[s]*o+t.y[s+1]*c,u=t.z[s]*o+t.z[s+1]*c,f=t.wallNormalX[s],d=t.wallNormalY[s],g=t.wallNormalZ[s],x=Math.max(t.nodeRadius[s],t.nodeRadius[s+1]),m=(l-t.wallX[s])*f+(h-t.wallY[s])*d+(u-t.wallZ[s])*g-x;if(m>=0){t.wallLambda[s]*=.85;continue}const p=t.inverseMass[s]*o*o,_=t.inverseMass[s+1]*c*c,v=p+_+e;if(v<ee)continue;let S=(-m-e*t.wallLambda[s])/v;const y=Math.max(0,t.wallLambda[s]+S);S=y-t.wallLambda[s],t.wallLambda[s]=y,t.x[s]+=f*S*t.inverseMass[s]*o,t.y[s]+=d*S*t.inverseMass[s]*o,t.z[s]+=g*S*t.inverseMass[s]*o,t.x[s+1]+=f*S*t.inverseMass[s+1]*c,t.y[s+1]+=d*S*t.inverseMass[s+1]*c,t.z[s+1]+=g*S*t.inverseMass[s+1]*c}}#y(t){if(t.sleeping)return;const e=t.wallCorrectionX,n=t.wallCorrectionY,i=t.wallCorrectionZ,s=t.wallCorrectionWeight;e.fill(0),n.fill(0),i.fill(0),s.fill(0);const a=Math.max(0,t.activeStart,t.collisionStartSegment),o=Math.min(t.activeEnd,t.collisionEndSegment+1,t.segmentCount);for(let c=a;c<o;c++){if(!t.wallActive[c])continue;const l=t.wallT[c],h=t.x[c]+(t.x[c+1]-t.x[c])*l,u=t.y[c]+(t.y[c+1]-t.y[c])*l,f=t.z[c]+(t.z[c+1]-t.z[c])*l,d=t.wallNormalX[c],g=t.wallNormalY[c],x=t.wallNormalZ[c],m=Math.max(t.nodeRadius[c],t.nodeRadius[c+1]),p=Math.max(0,yE+m-((h-t.wallX[c])*d+(u-t.wallY[c])*g+(f-t.wallZ[c])*x));if(p<=.02)continue;const _=Math.max(.5,t.segmentLength),v=He(Math.ceil(p/(_*.002)),4,96),S=c+l,y=Math.max(t.activeStart,Math.floor(S-v)),M=Math.min(t.activeEnd,Math.ceil(S+v));for(let w=y;w<=M;w++){if(t.inverseMass[w]<=0)continue;const T=Math.max(0,1-Math.abs(w-S)/(v+.5));e[w]+=d*p*T,n[w]+=g*p*T,i[w]+=x*p*T,s[w]+=T}}for(let c=t.activeStart;c<=t.activeEnd;c++){const l=s[c];l>ee&&(e[c]/=l,n[c]/=l,i[c]/=l)}for(let c=0;c<28;c++){let l=!1;const h=(c&1)===1;for(let u=h?t.activeEnd-1:t.activeStart;h?u>=t.activeStart:u<t.activeEnd;u+=h?-1:1){const f=u+1,d=e[f]-e[u],g=n[f]-n[u],x=i[f]-i[u],m=d*d+g*g+x*x,p=Math.max(1e-5,t.restLength[u]*.002);if(m<=p*p)continue;const _=Math.sqrt(m),v=t.inverseMass[u],S=t.inverseMass[f],y=v+S;if(y<=ee)continue;const M=(_-p)/_,w=M*v/y,T=M*S/y;e[u]+=d*w,n[u]+=g*w,i[u]+=x*w,e[f]-=d*T,n[f]-=g*T,i[f]-=x*T,l=!0}if(!l)break}for(let c=t.activeStart;c<=t.activeEnd;c++)t.x[c]+=e[c],t.y[c]+=n[c],t.z[c]+=i[c]}#E(t){if(t.sleeping)return;const e=1/this.fixedDt;let n=0;for(let i=t.activeStart;i<=t.activeEnd;i++){let s=t.x[i]-t.previousX[i],a=t.y[i]-t.previousY[i],o=t.z[i]-t.previousZ[i],c=0,l=0,h=0,u=0;i>0&&t.wallActive[i-1]&&(c+=t.wallLambda[i-1]*t.wallFriction,l+=t.wallNormalX[i-1],h+=t.wallNormalY[i-1],u+=t.wallNormalZ[i-1]),i<t.segmentCount&&t.wallActive[i]&&(c+=t.wallLambda[i]*t.wallFriction,l+=t.wallNormalX[i],h+=t.wallNormalY[i],u+=t.wallNormalZ[i]);const f=be(l,h,u);if(c>0&&f>ee){l/=f,h/=f,u/=f;const d=s*l+a*h+o*u,g=s-l*d,x=a-h*d,m=o-u*d,p=be(g,x,m);if(p>ee){const _=Math.min(p,c)/p;s-=g*_,a-=x*_,o-=m*_}}t.velocityX[i]=s*e,t.velocityY[i]=a*e,t.velocityZ[i]=o*e,n=Math.max(n,be(t.velocityX[i],t.velocityY[i],t.velocityZ[i]))}n<t.sleepVelocity&&this.settledMaxPenetration<.01?t.sleepCounter++:t.sleepCounter=0,t.sleepCounter>=t.sleepFrames&&(t.sleeping=!0,t.velocityX.fill(0),t.velocityY.fill(0),t.velocityZ.fill(0))}#w(t){let e=0,n=0,i=!0;for(let s=t.activeStart;s<Math.min(t.activeEnd,t.segmentCount);s++){const a=t.x[s+1]-t.x[s],o=t.y[s+1]-t.y[s],c=t.z[s+1]-t.z[s],l=be(a,o,c);if(e=Math.max(e,Math.abs(l-t.restLength[s])/Math.max(ee,t.restLength[s])),i=i&&Number.isFinite(l),s<=t.activeStart||s>=t.activeEnd-1)continue;const h=t.x[s]-t.x[s-1],u=t.y[s]-t.y[s-1],f=t.z[s]-t.z[s-1],d=t.x[s+1]-t.x[s],g=t.y[s+1]-t.y[s],x=t.z[s+1]-t.z[s],m=be(h,u,f)*be(d,g,x);m>ee&&(n=Math.max(n,Math.acos(He((h*d+u*g+f*x)/m,-1,1))))}return{id:t.id,sleeping:t.sleeping,finite:i,maxLengthError:e,maxBendAngleDegrees:n*180/Math.PI}}}function AE(r=140,t=0,e=null,n=SE){const o={radius:20,branchRadius:10,branchPoint:{x:0,y:-300,z:0},segments:[]},c={x:0,y:0,z:0},l={x:0,y:-300,z:0};o.main={start:c,end:l},o.segments.push({start:c,end:l,radius:20});function h(R){const N=Math.PI/6*R+t*R,L={x:o.branchPoint.x+Math.sin(N)*r,y:o.branchPoint.y-r,z:0};return{angle:N,end:L,length:r}}o.right=h(1),o.left=h(-1),o.segments.push({start:o.branchPoint,end:o.right.end,radius:10}),o.segments.push({start:o.branchPoint,end:o.left.end,radius:10});const u={x:-73,y:-383,z:14},f=new b(.24,.96,-.21).normalize(),d=o.left.length*.5,g=e??d,x={x:u.x-f.x*g,y:u.y-f.y*g,z:u.z-f.z*g},m={...u};o.sheath={start:x,end:m,radius:n,length:g,isSheath:!0},o.segments.push(o.sheath);for(const R of o.segments){const N=R.end.x-R.start.x,L=R.end.y-R.start.y,F=R.end.z-R.start.z,z=Math.sqrt(N*N+L*L+F*F)||1;R.length=z,R.volume=Math.PI*R.radius*R.radius*z}const p=new Map,_=[];function v(R){const N=`${R.x.toFixed(5)},${R.y.toFixed(5)},${R.z.toFixed(5)}`;if(p.has(N))return p.get(N);const L=_.length;return p.set(N,L),_.push({position:R,segments:[]}),L}o.segments.forEach((R,N)=>{R.startNode=v(R.start),R.endNode=v(R.end),_[R.startNode].segments.push(N),_[R.endNode].segments.push(N)}),o.nodes=_;const S=o.segments.map(()=>[]),y=o.segments.map(()=>null),M=1e-6,w=(R,N)=>Math.abs(R.x-N.x)<M&&Math.abs(R.y-N.y)<M&&Math.abs(R.z-N.z)<M;for(let R=0;R<o.segments.length;R++)for(let N=0;N<o.segments.length;N++)R!==N&&w(o.segments[R].end,o.segments[N].start)&&(S[R].push(N),y[N]=R);o.segmentGraph=S;for(let R=0;R<o.segments.length;R++)o.segments[R].parent=y[R];const T=85,E={},A=R=>{const N=R.end.x-R.start.x,L=R.end.y-R.start.y,F=R.end.z-R.start.z,z=Math.sqrt(N*N+L*L+F*F)||1;return{x:N/z,y:L/z,z:F/z}};function D(R,N){const L=o.segments[R],F=A(L);L.flowDir=F,L.flowSpeed=N;const z=S[R];if(E[R]={dir:F,speed:N,children:z},z.length){let q=0;for(const O of z)q+=o.segments[O].radius;for(const O of z){const Y=N*(o.segments[O].radius/q);D(O,Y)}}}for(let R=0;R<o.segments.length;R++)y[R]===null&&D(R,T);return o.flow=E,{vessel:o}}const TE=2,Ol=1/15,bE=.58,CE=.29,$i=(r,t,e)=>Math.min(e,Math.max(t,r)),Gl=(r,t,e)=>r+(t-r)*e;class RE{constructor(t,e,n,i,s={}){this.ecgCanvas=t,this.bpCanvas=e,this.hrElem=n,this.bpElem=i,this.spo2Elem=s.spo2Elem||null,this.mapElem=s.mapElem||null,this.rrElem=s.rrElem||null,this.rhythmElem=s.rhythmElem||null,this.clockElem=s.clockElem||null,this.ecgCtx=t.getContext("2d"),this.bpCtx=e.getContext("2d"),this.ecgCanvasState=this.#_("#020303","#000000"),this.bpCanvasState=this.#_("#030202","#000000"),this.baselineDash=[6,8],this.ecgSampleRate=250,this.bpSampleRate=50,this.ecgBufferLength=this.ecgSampleRate*10,this.bpBufferLength=this.bpSampleRate*10,this.ecgData=new Float32Array(this.ecgBufferLength),this.bpData=new Float32Array(this.bpBufferLength),this.bpData.fill(100),this.ecgCursor=0,this.bpCursor=0,this.drawAccumulator=Ol,this.lastReadouts=Object.create(null),this.lastClockSecond=-1,this.clockLabel="00:00",this.time=0,this.cycleTime=0,this.variabilitySeed=Math.random()*Math.PI*2,this.baseHeartRate=75,this.heartRate=this.baseHeartRate,this.beatInterval=60/this.heartRate,this.ecgAccumulator=0,this.bpAccumulator=0,this.currentHR=this.heartRate,this.baselineSystolic=120,this.baselineDiastolic=80,this.waveSystolic=this.baselineSystolic,this.waveDiastolic=this.baselineDiastolic,this.systolic=120,this.diastolic=80,this.meanPressure=93,this.spo2=98,this.spo2Target=this.spo2,this.respiratoryRate=14,this.respiratoryRateTarget=this.respiratoryRate,this.bpMax=0,this.bpMin=1/0,this.ecgTemplate=this.#s(),this.ecgTemplateIndex=0,this.ecgSamplesSinceBeat=0,this.ecgSamplesToNextBeat=this.#a(),this.bpTemplate=this.#S()}setHeartRate(t){this.baseHeartRate=t,this.heartRate=t,this.beatInterval=60/this.heartRate,this.currentHR=t}update(t){this.ecgAccumulator+=t,this.bpAccumulator+=t,this.time+=t,this.cycleTime+=t;const e=1/this.ecgSampleRate;for(;this.ecgAccumulator>=e;){this.ecgAccumulator-=e;const i=this.#l();this.ecgData[this.ecgCursor]=i,this.ecgCursor=(this.ecgCursor+1)%this.ecgBufferLength}const n=1/this.bpSampleRate;for(;this.bpAccumulator>=n;){this.bpAccumulator-=n;const i=this.cycleTime/this.beatInterval%1,s=Math.floor(i*this.bpTemplate.length),a=this.#r(this.bpTemplate[s]);this.bpData[this.bpCursor]=a,this.bpCursor=(this.bpCursor+1)%this.bpBufferLength,a>this.bpMax&&(this.bpMax=a),a<this.bpMin&&(this.bpMin=a)}this.cycleTime>=this.beatInterval&&(this.currentHR=60/this.beatInterval,this.systolic=this.bpMax,this.diastolic=this.bpMin,this.meanPressure=this.diastolic+(this.systolic-this.diastolic)/3,this.cycleTime-=this.beatInterval,this.bpMax=0,this.bpMin=1/0,this.#i()),this.#h(t),this.drawAccumulator+=t,!(this.drawAccumulator<Ol)&&(this.drawAccumulator%=Ol,this.#t("hr",this.hrElem,Math.round(this.currentHR)),this.#n(),this.#t("spo2",this.spo2Elem,Math.round(this.spo2)),this.#t("map",this.mapElem,Math.round(this.meanPressure)),this.#t("rr",this.rrElem,Math.round(this.respiratoryRate)),this.#t("rhythm",this.rhythmElem,this.#L()),this.#t("clock",this.clockElem,this.#o()),this.#A(),this.#u())}#t(t,e,n){!e||this.lastReadouts[t]===n||(e.textContent=n,this.lastReadouts[t]=n)}#n(){if(!this.bpElem)return;const t=Math.round(this.systolic),e=Math.round(this.diastolic),n=t*256+e;this.lastReadouts.bp!==n&&(this.bpElem.textContent=`${t}/${e}`,this.lastReadouts.bp=n)}#i(){const t=Math.sin(this.time*.34+this.variabilitySeed),e=Math.sin(this.time*.11+this.variabilitySeed*.7),n=(Math.random()-.5)*1.8,i=(Math.random()-.5)*2.2;this.heartRate=$i(this.baseHeartRate+t*2.2+e*1.4+n,58,96),this.beatInterval=60/this.heartRate,this.ecgSamplesToNextBeat=this.#a(),this.currentHR=Gl(this.currentHR,this.heartRate,.75),this.waveSystolic=$i(this.baselineSystolic+t*3.2+e*2+i,106,134),this.waveDiastolic=$i(this.baselineDiastolic+t*1.6+e*1.2+i*.45,68,88)}#l(){const t=this.ecgTemplateIndex<this.ecgTemplate.length?this.ecgTemplate[this.ecgTemplateIndex]:0;return this.ecgTemplateIndex+=1,this.ecgSamplesSinceBeat+=1,this.ecgSamplesSinceBeat>=this.ecgSamplesToNextBeat&&(this.ecgTemplateIndex=0,this.ecgSamplesSinceBeat=0,this.ecgSamplesToNextBeat=this.#a()),t}#a(){return Math.max(this.ecgTemplate?.length||1,Math.round(this.beatInterval*this.ecgSampleRate))}#h(t){const e=Math.sin(this.time*.31+this.variabilitySeed),n=Math.sin(this.time*.07+this.variabilitySeed*1.9),i=98+e*.9+n*.65,s=14+e*.9+n*.5;this.spo2Target=$i(i,96,100),this.respiratoryRateTarget=$i(s,11,18),this.spo2=Gl(this.spo2,this.spo2Target,$i(t*1.4,0,1)),this.respiratoryRate=Gl(this.respiratoryRate,this.respiratoryRateTarget,$i(t*.8,0,1))}#m(t){const e=(n,i,s)=>{const a=(t-n)/i;return s*Math.exp(-.5*a**2)};return e(.095,.022,.08)+e(.178,.009,-.12)+e(.198,.007,.82)+e(.222,.012,-.18)+e(.42,.062,.17)}#g(t){const i=1/(1+Math.exp(-(t-.11)/.018)),s=Math.exp(-Math.max(t-.16,0)/.36),a=-5.5*Math.exp(-.5*((t-.33)/.018)**2),o=3.2*Math.exp(-.5*((t-.37)/.026)**2);return 80+40*i*s+a+o}#r(t){const e=$i((t-80)/40,0,1.25);return this.waveDiastolic+e*(this.waveSystolic-this.waveDiastolic)}#s(){const e=Math.round(.62*this.ecgSampleRate),n=[];for(let i=0;i<e;i++)n.push(this.#m(i/this.ecgSampleRate));return n}#S(){const t=[];for(let e=0;e<this.bpSampleRate;e++){const n=e/this.bpSampleRate;t.push(this.#g(n))}return t}#A(){const t=this.ecgCtx,e=this.#T(this.ecgCanvas,t,this.ecgCanvasState),n=e.w,i=e.h,s=this.ecgData.length;this.#M(t,n,i,e.backgroundGradient);const a=i*bE,o=i*CE,c=Math.max(2,Math.min(s,Math.ceil(n)));this.#E(t,n,i,a,"rgba(82, 118, 102, 0.32)"),t.beginPath();for(let l=0;l<c;l++){const h=Math.floor(l*s/c),u=Math.max(h+1,Math.floor((l+1)*s/c));let f=0;for(let x=h;x<u;x++){let m=this.ecgCursor+x;m>=s&&(m-=s);const p=this.ecgData[m];Math.abs(p)>Math.abs(f)&&(f=p)}const d=l/(c-1)*n,g=a-f*o;l===0?t.moveTo(d,g):t.lineTo(d,g)}this.#w(t,"#39e75f",1.35),this.#C(t,n,i,e.markerGradient,"#39e75f")}#u(){const t=this.bpCtx,e=this.#T(this.bpCanvas,t,this.bpCanvasState),n=e.w,i=e.h,s=this.bpData.length,a=Math.max(2,Math.min(s,Math.ceil(n)));this.#M(t,n,i,e.backgroundGradient),this.#E(t,n,i,i-45/85*i,"rgba(120, 88, 88, 0.3)"),t.beginPath();for(let o=0;o<a;o++){const c=Math.floor(o*s/a),l=Math.max(c+1,Math.floor((o+1)*s/a));let h=0;for(let g=c;g<l;g++){let x=this.bpCursor+g;x>=s&&(x-=s),h+=this.bpData[x]}const u=h/(l-c),f=o/(a-1)*n,d=i-(u-55)/85*i;o===0?t.moveTo(f,d):t.lineTo(f,d)}this.#w(t,"#f04d4d",1.35),this.#C(t,n,i,e.markerGradient,"#f04d4d")}#_(t,e){return{w:0,h:0,dpr:0,topColor:t,bottomColor:e,backgroundGradient:null,markerGradient:null}}#T(t,e,n){const i=Math.max(1,t.clientWidth||t.width),s=Math.max(1,t.clientHeight||t.height),a=Math.min(window.devicePixelRatio||1,TE),o=Math.round(i*a),c=Math.round(s*a);if((t.width!==o||t.height!==c)&&(t.width=o,t.height=c),e.setTransform(a,0,0,a,0,0),n.w!==i||n.h!==s||n.dpr!==a||!n.backgroundGradient){const l=e.createLinearGradient(0,0,0,s);l.addColorStop(0,n.topColor),l.addColorStop(1,n.bottomColor);const h=i-10.5,u=e.createLinearGradient(h-20,0,h+4,0);u.addColorStop(0,"rgba(255,255,255,0)"),u.addColorStop(1,"rgba(210,220,218,0.12)"),n.w=i,n.h=s,n.dpr=a,n.backgroundGradient=l,n.markerGradient=u}return n}#M(t,e,n,i){t.clearRect(0,0,e,n),t.fillStyle=i,t.fillRect(0,0,e,n),this.#y(t,e,n)}#y(t,e,n){t.save(),t.strokeStyle="rgba(88, 112, 106, 0.16)",t.lineWidth=1;for(let i=.5;i<e;i+=32)t.beginPath(),t.moveTo(i,0),t.lineTo(i,n),t.stroke();for(let i=.5;i<n;i+=24)t.beginPath(),t.moveTo(0,i),t.lineTo(e,i),t.stroke();t.strokeStyle="rgba(88, 112, 106, 0.26)";for(let i=.5;i<e;i+=160)t.beginPath(),t.moveTo(i,0),t.lineTo(i,n),t.stroke();t.restore()}#E(t,e,n,i,s){t.save(),t.strokeStyle=s,t.setLineDash(this.baselineDash),t.lineWidth=1,t.beginPath(),t.moveTo(0,i),t.lineTo(e,i),t.stroke(),t.restore()}#w(t,e,n){t.save(),t.lineJoin="round",t.lineCap="round",t.strokeStyle=e,t.lineWidth=n,t.stroke(),t.restore()}#C(t,e,n,i,s){const a=e-10.5;t.save(),t.fillStyle=i,t.fillRect(Math.max(0,a-20),0,24,n),t.strokeStyle=s,t.globalAlpha=.85,t.lineWidth=1,t.beginPath(),t.moveTo(a,8),t.lineTo(a,n-8),t.stroke(),t.restore()}#L(){return this.currentHR>=105?"TACHY":this.currentHR<=50?"BRADY":this.meanPressure<65?"LOW MAP":"SINUS"}#o(){const t=Math.floor(this.time);if(t===this.lastClockSecond)return this.clockLabel;const e=Math.floor(t/60).toString().padStart(2,"0"),n=(t%60).toString().padStart(2,"0");return this.lastClockSecond=t,this.clockLabel=`${e}:${n}`,this.clockLabel}}function ke(r,t,e,n,i,s=new hn){const a=new Zt(new Pn(r,t,e),n);return a.position.copy(i),a.rotation.copy(s),a}function hi(r,t,e,n,i,s=new hn,a=40){const o=new Zt(new gs(r,t,e,a),n);return o.position.copy(i),o.rotation.copy(s),o}function Ff(r,t,e,n,i=new hn){const s=new Zt(new Cr(r,t,10,24),e);return s.position.copy(n),s.rotation.copy(i),s}function Vl(r,t,e,n,i,s,a=0){const o=[];for(let l=0;l<=96;l++){const h=he.degToRad(t+(e-t)*l/96);o.push(new b(n,r*Math.sin(h),a+r*Math.cos(h)))}const c=new Em(o);return new Zt(new wu(c,128,i,18,!1),s)}function PE(){const r=new De,t=new De,e=new De,n=new De,i=new b(10,22,0),s=new En({color:14542314,roughness:.42,metalness:.08}),a=new En({color:15922678,roughness:.36,metalness:.04}),o=new En({color:10332852,roughness:.48,metalness:.28}),c=new En({color:4739933,roughness:.62,metalness:.25}),l=new En({color:1383200,roughness:.75}),h=new En({color:15331571,roughness:.34}),u=new Fe({color:12773623}),f=new En({color:15002092,roughness:.4,metalness:.06}),d=new Fe({color:9559551,transparent:!0,opacity:.16,depthWrite:!1}),g=new Fe({color:4380671});r.add(ke(118,12,58,l,new b(10,-105,-82))),r.add(ke(78,52,62,s,new b(10,-72,-82))),r.add(ke(84,18,62,c,new b(10,-103,-82))),r.add(hi(12,12,10,l,new b(-34,-108,-108),new hn(Math.PI/2,0,0),32)),r.add(hi(12,12,10,l,new b(54,-108,-108),new hn(Math.PI/2,0,0),32)),r.add(hi(10,10,8,l,new b(-34,-108,-56),new hn(Math.PI/2,0,0),32)),r.add(hi(10,10,8,l,new b(54,-108,-56),new hn(Math.PI/2,0,0),32)),r.add(hi(16,18,70,c,new b(10,-37,-82))),t.add(hi(12,14,96,o,new b(10,-14,-82))),t.add(ke(62,24,52,s,new b(10,38,-82))),t.add(ke(28,10,44,c,new b(10,24,-82))),t.add(Ff(12,34,s,new b(10,37,-82),new hn(0,0,Math.PI/2))),t.add(ke(48,26,40,s,new b(10,37,-82))),t.add(ke(54,18,28,s,new b(10,29,-86))),t.add(ke(34,22,34,s,new b(10,20,-86))),t.add(hi(21,21,18,c,new b(10,26,-82),new hn(Math.PI/2,0,0),48)),t.add(hi(25,25,18,c,new b(10,22,-86),new hn(Math.PI/2,0,0),48)),t.add(ke(46,34,24,c,new b(10,22,-86))),t.add(hi(5,6,28,o,new b(28,79,-82))),t.add(ke(46,24,6,a,new b(28,96,-82),new hn(he.degToRad(-8),0,0))),t.add(ke(31,16,2,new Fe({color:1450543}),new b(28,96,-78))),t.add(Ff(3.2,30,o,new b(38,19,-52),new hn(Math.PI/2,0,0))),e.position.copy(i),t.add(e),r.add(t);const x=86,m=0,p=0,_=58,v=Vl(x,_,360-_,m,6.2,a,p),S=Vl(x+8,_+2,360-_-2,m-4.5,1.8,o,p),y=Vl(x-8,_+2,360-_-2,m+4.5,1.8,o,p);e.add(v,S,y);const M=he.degToRad(_),w=x*Math.sin(M),T=-w,E=0,A=p+x*Math.cos(M),D=(A+E)*.5,R=Math.abs(E-A)+12;e.add(ke(48,13,R,a,new b(m,w,D))),e.add(ke(48,13,R,a,new b(m,T,D))),e.add(ke(42,14,16,a,new b(m,w,A))),e.add(ke(42,14,16,a,new b(m,T,A)));const N=ke(50,16,42,h,new b(m,w,E));n.add(N);const L=ke(40,2,34,u,new b(m,w-9,E));n.add(L);const F=ke(58,22,44,f,new b(m,T,E));n.add(F);const z=ke(36,9,26,c,new b(m,T+17,E));n.add(z);const q=hi(15,22,w-T-20,d,new b(m,0,E));n.add(q),e.add(n);const O=new Zt(new yu(7.5,9,48),g);return O.position.set(m,0,E),O.rotation.x=Math.PI/2,e.add(O),{group:r,gantryGroup:e,liftGroup:t,detectorAssembly:n}}function LE(){const r=new De,t=new De;r.userData.slideGroup=t;const e=new En({color:10134701,roughness:.45}),n=new En({color:14081507,roughness:.3,metalness:.15}),i=new En({color:5857899,roughness:.6,metalness:.2}),s=new En({color:2503490,roughness:.55}),a=new En({color:2064266,roughness:.7}),o=new En({color:14202011,roughness:.65}),c=new Zt(new Pn(86,10,58),i);c.position.set(0,-88,0),r.add(c);const l=new Zt(new gs(8,10,76,32),i);l.position.set(0,-45,0),r.add(l);const h=new Zt(new Pn(66,10,44),i);h.position.set(0,-8,0),r.add(h);const u=new Zt(new Pn(230,8,58),e);u.position.set(0,0,0),t.add(u);const f=new Zt(new Pn(218,5,48),s);f.position.set(0,6.5,0),t.add(f);const d=new Zt(new Pn(224,3,3),n);d.position.set(0,7,-32),t.add(d);const g=d.clone();g.position.z=32,t.add(g);const x=new Zt(new Cr(17,64,10,22),a);x.rotation.z=Math.PI/2,x.position.set(16,23,0),t.add(x);const m=new Zt(new Pn(62,8,42),a);m.position.set(14,20,0),t.add(m);const p=new Zt(new ai(13,28,18),o);p.scale.set(1.05,.82,.9),p.position.set(-50,21,0),t.add(p);const _=new Zt(new Pn(32,5,32),new En({color:15265522,roughness:.75}));_.position.set(-50,13,0),t.add(_);const v=new Zt(new Cr(7,62,8,16),a);v.rotation.z=Math.PI/2,v.position.set(70,18,-10),t.add(v);const S=v.clone();S.position.z=10,t.add(S);const y=new Zt(new Cr(4.5,58,8,14),o);y.rotation.z=Math.PI/2,y.position.set(4,17,-31),t.add(y);const M=y.clone();return M.position.z=31,t.add(M),r.add(t),r}let ii,yr,Er,Do,Uh,Uf,uc,zh;const IE=new b(0,24,-30);function DE(){const r=document.getElementById("carm-preview");if(!r)return null;r.replaceChildren(),ii=new kr,ii.background=new jt(131843);const t=new cy(14542820,.72);ii.add(t);const e=new Mf(16777215,.85);e.position.set(120,180,160),ii.add(e);const n=new Mf(10140083,.24);n.position.set(-160,40,-130),ii.add(n);const i=r.clientWidth,s=r.clientHeight;yr=new Ln(39,i/s,.1,1e3),yr.position.set(268,146,289),yr.lookAt(IE),ii.add(yr),Er=new _u({antialias:!0,alpha:!0}),Er.setSize(i,s),Er.setPixelRatio(Math.min(window.devicePixelRatio||1,2)),r.appendChild(Er.domElement);const a=new hy(300,12,4741719,1910052);a.position.y=-94,ii.add(a),uc=LE(),ii.add(uc),Do=new De;const{group:o,gantryGroup:c,liftGroup:l,detectorAssembly:h}=PE();return Uh=c,Uf=l,zh=h,Do.add(o),ii.add(Do),Xm(),{group:Do,gantry:Uh,detectorAssembly:zh,lift:Uf,table:uc}}function Xm(){!Er||!ii||!yr||Er.render(ii,yr)}function NE(r,t,e,n,i,s,a,o,c=()=>{}){const l=document.getElementById("carmX"),h=document.getElementById("carmY"),u=document.getElementById("carmZ"),f=document.getElementById("carmDetDist"),d=document.getElementById("carmZUp"),g=document.getElementById("carmZDown"),x=document.getElementById("carmRollLeft"),m=document.getElementById("carmRollRight"),p=document.getElementById("carmAngleReset"),_=document.getElementById("carmLao30"),v=document.getElementById("carmRao30"),S=document.getElementById("carmYawReadout"),y=document.getElementById("carmPitchReadout"),M=document.getElementById("carmRollReadout");[l,h,u,f].filter(Boolean).forEach(st=>st.addEventListener("change",()=>st.blur()));let T=0,E=0,A=0,D=parseFloat(l.value),R=parseFloat(h.value),N=parseFloat(u.value),L=parseFloat(f.value),F=!1,z=0;const q=D,O=R,Y=N,K=L,J=10,it=new b(1,0,0),X=new b(0,0,1),j=new b(0,1,0),nt=new Nn,ot=new Nn,ut=new Nn;function rt(){const st=Y-(N-Y);return new b(t.branchPoint.x+D,t.branchPoint.y+R,t.branchPoint.z+st)}function ht(st){return Math.round(he.radToDeg(st))}function dt(st){return st===0?"AP 0°":`${st>0?"LAO":"RAO"} ${Math.abs(st)}°`}function bt(st){return st===0?"CRA 0°":`${st>0?"CRA":"CAU"} ${Math.abs(st)}°`}function V(){S&&(S.textContent=dt(ht(T))),y&&(y.textContent=bt(ht(E))),M&&(M.textContent=`Roll ${ht(A)}°`)}function re(){const st=rt(),Ut=new b().setFromSpherical(new ly(1,Math.PI/2-E,T)).normalize(),P=st.clone().addScaledVector(Ut,e),G=st.clone().addScaledVector(Ut,-L);r.position.copy(P),r.up.set(0,1,0),r.lookAt(G),r.rotateZ(A);const W=D-q,Z=R-O,k=N-Y;n&&n.position.set(J,0,0),a&&(a.position.y=k*.12),o&&(o.userData.slideGroup||o).position.set(Z*.08,0,W*.08),i&&(nt.setFromAxisAngle(it,-T),ot.setFromAxisAngle(X,E),i.quaternion.copy(ot).multiply(nt)),s&&(ut.setFromAxisAngle(j,A),s.quaternion.copy(ut)),(n||i||s||a||o)&&c(),V(),z++}re(),l.addEventListener("input",st=>{F||(D=parseFloat(st.target.value),re())}),h.addEventListener("input",st=>{F||(R=parseFloat(st.target.value),re())}),u.addEventListener("input",st=>{F||(N=parseFloat(st.target.value),re())}),f.addEventListener("input",st=>{F||(L=parseFloat(st.target.value),re())});const Tt=document.getElementById("positionJoystick"),It=document.getElementById("positionJoystickHandle"),vt=document.getElementById("angleJoystick"),Jt=document.getElementById("angleJoystickHandle");let Rt=0,I=0,C=0,H=!1,Q=null,$=null;const tt=he.degToRad(90),Et=he.degToRad(45),at=he.degToRad(90),xt=he.degToRad(22),Pt=he.degToRad(18),Ht=he.degToRad(18),et=he.degToRad(24),ae=he.degToRad(24),qt=.22;function Gt(st,Ut){const P=Math.abs(st);return P<Ut?0:Math.sign(st)*((P-Ut)/(1-Ut))}function wt(st,Ut,P,G,{resetOnRelease:W=!0}={}){if(!st||!Ut)return;const Z=Ut.offsetWidth/2,k=st.offsetWidth/2-Z;let lt=!1;const At="transform 0.2s ease-out";function Ct(_t,Vt){if(F)return;const Ft=st.getBoundingClientRect();let Bt=_t-Ft.left-Ft.width/2,fe=Vt-Ft.top-Ft.height/2;const Ze=Math.hypot(Bt,fe);if(Ze>k){const Te=k/Ze;Bt*=Te,fe*=Te}Ut.style.transform=`translate(-50%, -50%) translate(${Bt}px, ${fe}px)`,P(Bt/k,fe/k)}st.addEventListener("mousedown",_t=>{F||(lt=!0,Ut.style.transition="none",Ct(_t.clientX,_t.clientY))}),window.addEventListener("mousemove",_t=>{lt&&Ct(_t.clientX,_t.clientY)}),window.addEventListener("mouseup",()=>{lt&&(lt=!1,Ut.style.transition=At,W&&(Ut.style.transform="translate(-50%, -50%)"),G())}),st.addEventListener("pointerdown",_t=>{F||(lt=!0,st.setPointerCapture?.(_t.pointerId),Ut.style.transition="none",Ct(_t.clientX,_t.clientY))}),st.addEventListener("pointermove",_t=>{lt&&Ct(_t.clientX,_t.clientY)}),st.addEventListener("pointerup",_t=>{lt&&(lt=!1,st.releasePointerCapture?.(_t.pointerId),Ut.style.transition=At,W&&(Ut.style.transform="translate(-50%, -50%)"),G())}),st.addEventListener("pointercancel",_t=>{lt&&(lt=!1,st.releasePointerCapture?.(_t.pointerId),Ut.style.transition=At,W&&(Ut.style.transform="translate(-50%, -50%)"),G())}),st.addEventListener("touchstart",_t=>{if(F)return;_t.preventDefault(),lt=!0,Ut.style.transition="none";const Vt=_t.touches[0];Ct(Vt.clientX,Vt.clientY)}),window.addEventListener("touchmove",_t=>{if(!lt)return;const Vt=_t.touches[0];Ct(Vt.clientX,Vt.clientY)},{passive:!1}),window.addEventListener("touchend",()=>{lt&&(lt=!1,Ut.style.transition=At,W&&(Ut.style.transform="translate(-50%, -50%)"),G())})}let gt=0,Dt=0,$t=0;const de=parseFloat(l.min),Xt=parseFloat(l.max),ct=parseFloat(h.min),U=parseFloat(h.max),ft=parseFloat(u.min),pt=parseFloat(u.max),zt=(Xt-de)*.18,Nt=(U-ct)*.18,ue=(pt-ft)*.18;let ce=performance.now();function _e(st,Ut){return Math.abs(st)<=Ut?0:st-Math.sign(st)*Ut}function Ae(st){const Ut=(st-ce)/1e3;if(ce=st,F){requestAnimationFrame(Ae);return}let P=!1;if((gt!==0||Dt!==0)&&(D=Math.min(Math.max(D+gt*zt*Ut,de),Xt),R=Math.min(Math.max(R+Dt*Nt*Ut,ct),U),l.value=Math.round(D),h.value=Math.round(R),P=!0),$t!==0){const G=he.clamp(N+$t*ue*Ut,ft,pt);P=P||G!==N,N=G,u.value=Math.round(N)}if((I!==0||C!==0)&&(T=Math.min(Math.max(T+I*xt*Ut,-tt),tt),E=Math.min(Math.max(E+C*Pt*Ut,-Et),Et),P=!0),Rt!==0&&(A=Math.min(Math.max(A+Rt*Ht*Ut,-at),at),P=!0),H){I=0,C=0,Q=null,$?.classList.remove("active"),$=null,Rt=0;const G=et*Ut,W=_e(T,G),Z=_e(E,G),k=_e(A,G);P=P||W!==T||Z!==E||k!==A,T=W,E=Z,A=k}if(Q!==null){I=0;const G=ae*Ut,W=Q-T,Z=Math.abs(W)<=G?Q:T+Math.sign(W)*G;P=P||Z!==T,T=he.clamp(Z,-tt,tt)}P&&re(),requestAnimationFrame(Ae)}requestAnimationFrame(Ae);function oe(st){F||($t=st)}function Pe(){$t=0}d&&g&&(d.addEventListener("mousedown",()=>oe(1)),g.addEventListener("mousedown",()=>oe(-1)),window.addEventListener("mouseup",Pe),d.addEventListener("touchstart",st=>{st.preventDefault(),oe(1)}),g.addEventListener("touchstart",st=>{st.preventDefault(),oe(-1)}),window.addEventListener("touchend",Pe),window.addEventListener("touchcancel",Pe));function qe(st){F||(Rt=st)}function yi(){Rt=0}x&&m&&(x.addEventListener("mousedown",()=>qe(-1)),m.addEventListener("mousedown",()=>qe(1)),window.addEventListener("mouseup",yi),x.addEventListener("touchstart",st=>{st.preventDefault(),qe(-1)}),m.addEventListener("touchstart",st=>{st.preventDefault(),qe(1)}),window.addEventListener("touchend",yi),window.addEventListener("touchcancel",yi));function Os(st){st?.preventDefault?.(),!F&&(H=!0,I=0,C=0,Rt=0,Jt&&(Jt.style.transition="transform 0.2s ease-out",Jt.style.transform="translate(-50%, -50%)"),p?.classList.add("active"))}function An(){H=!1,p?.classList.remove("active")}p&&(p.addEventListener("pointerdown",st=>{p.setPointerCapture?.(st.pointerId),Os(st)}),p.addEventListener("pointerup",st=>{p.releasePointerCapture?.(st.pointerId),An()}),p.addEventListener("pointercancel",An),p.addEventListener("pointerleave",An),p.addEventListener("click",st=>st.preventDefault()),window.addEventListener("blur",An));function Wi(st,Ut,P){P?.preventDefault?.(),!F&&(H=!1,p?.classList.remove("active"),I=0,C=0,Q=he.clamp(st,-tt,tt),$&&$!==Ut&&$.classList.remove("active"),$=Ut,$?.classList.add("active"),Jt&&(Jt.style.transition="transform 0.2s ease-out",Jt.style.transform="translate(-50%, -50%)"))}function Tn(){Q=null,$?.classList.remove("active"),$=null}function Gs(st,Ut){st&&(st.addEventListener("pointerdown",P=>{st.setPointerCapture?.(P.pointerId),Wi(Ut,st,P)}),st.addEventListener("pointerup",P=>{st.releasePointerCapture?.(P.pointerId),Tn()}),st.addEventListener("pointercancel",Tn),st.addEventListener("pointerleave",Tn),st.addEventListener("click",P=>P.preventDefault()))}Gs(_,he.degToRad(30)),Gs(v,he.degToRad(-30)),window.addEventListener("blur",Tn),wt(Tt,It,(st,Ut)=>{gt=-Ut,Dt=-st},()=>{gt=0,Dt=0}),wt(vt,Jt,(st,Ut)=>{Tn(),I=Gt(-Ut,qt),C=Gt(-st,qt)},()=>{I=0,C=0});function nn(){gt=0,Dt=0,$t=0,Rt=0,I=0,C=0,H=!1,Q=null,$?.classList.remove("active"),$=null,p?.classList.remove("active"),It&&(It.style.transform="translate(-50%, -50%)"),Jt&&(Jt.style.transform="translate(-50%, -50%)")}function Ei(){nn(),T=0,E=0,A=0,D=q,R=O,N=Y,L=K,l.value=String(Math.round(D)),h.value=String(Math.round(R)),u.value=String(Math.round(N)),f.value=String(Math.round(L)),re()}return{reset:Ei,getRevision:()=>z,setLocked(st){F=st===!0,F&&nn()}}}function FE(r){const{camera:t,cameraRadius:e,vessel:n,voxelGroup:i,displayMaterial:s,blendMaterial:a,wireMaterial:o,onStartInjection:c,onStopInjection:l,onModeChange:h,onDebugLayerChange:u,onStartBrowserBenchmark:f,onStopBrowserBenchmark:d}=r,g=new RE(document.getElementById("ecgCanvas"),document.getElementById("bpCanvas"),document.getElementById("hrValue"),document.getElementById("bpValue"),{spo2Elem:document.getElementById("spo2Value"),mapElem:document.getElementById("mapValue"),rrElem:document.getElementById("rrValue"),rhythmElem:document.getElementById("monitorRhythm"),clockElem:document.getElementById("monitorClock")}),x=DE(),m=NE(t,n,e,x?.group||Do,x?.gantry||Uh,x?.detectorAssembly||zh,x?.lift,x?.table||uc,Xm),p=document.getElementById("stiffness"),_=document.getElementById("staticFriction"),v=document.getElementById("kineticFriction"),S=document.getElementById("smoothIterations"),y=document.getElementById("modeToggle"),M=document.getElementById("renderVoxels"),w=document.getElementById("showDebugStlModel"),T=document.getElementById("showDebugLumenCast"),E=document.getElementById("showDebugSections"),A=document.getElementById("showDebugCenterline"),D=document.getElementById("showDebugCapsules"),R=document.getElementById("injectContrast"),N=document.getElementById("stopInjection"),L=document.getElementById("injRate"),F=document.getElementById("injDuration"),z=document.getElementById("injVolume"),q=document.getElementById("autoExposureToggle"),O=document.getElementById("persistence"),Y=document.getElementById("pulseRate"),K=document.getElementById("noiseLevel"),J=document.getElementById("scatterStrength"),it=document.getElementById("collimation"),X=document.getElementById("imageBrightness"),j=document.getElementById("imageContrast"),nt=document.getElementById("edgeEnhancement"),ot=document.getElementById("boneVisibility"),ut=document.getElementById("opacityScale"),rt=document.getElementById("gain"),ht=document.getElementById("insertedLength"),dt=document.getElementById("catheterLength"),bt=document.getElementById("catheterAdvance"),V=document.getElementById("catheterWithdraw"),re=document.getElementById("catheterRotateLeft"),Tt=document.getElementById("catheterRotateRight"),It=document.getElementById("catheterType"),vt=document.getElementById("guidewireType"),Jt=document.getElementById("catheterTypeStatus"),Rt=document.getElementById("guidewireTypeStatus"),I=document.getElementById("currentDose"),C=document.getElementById("currentKV"),H=document.getElementById("currentMA"),Q=document.getElementById("guidewireResistanceStatus"),$=document.getElementById("guidewireResistanceReason"),tt=document.getElementById("guidewireResistanceValue"),Et=document.getElementById("guidewireResistanceFill"),at=document.getElementById("guidewireDiagnostics"),xt=document.getElementById("guidewireDiameter"),Pt=document.getElementById("sheathDiameter"),Ht=document.getElementById("catheterDiameter"),et=document.getElementById("perfStats"),ae=document.getElementById("runBrowserBenchmarkSmoke"),qt=document.getElementById("runBrowserBenchmarkFull"),Gt=document.getElementById("stopBrowserBenchmark"),wt=document.getElementById("browserBenchmarkStatus"),gt=document.getElementById("browserBenchmarkReport");xt&&(xt.textContent=`${Fm.toFixed(3)}" · ${Um.toFixed(3)} mm`),Pt&&(Pt.textContent=`${zm}F · ${Bm.toFixed(3)} mm`),Ht&&(Ht.textContent=`${Vm}F · ${km.toFixed(3)} mm`),M&&(i.visible=M.checked);const Dt={stlModel:w?.checked??!0,lumenCast:T?.checked??!1,sections:E?.checked??!1,centerline:A?.checked??!0,capsules:D?.checked??!1};function $t(){typeof u=="function"&&u({...Dt})}w?.addEventListener("change",B=>{Dt.stlModel=B.target.checked,$t()}),T?.addEventListener("change",B=>{Dt.lumenCast=B.target.checked,$t()}),E?.addEventListener("change",B=>{Dt.sections=B.target.checked,$t()}),A?.addEventListener("change",B=>{Dt.centerline=B.target.checked,$t()}),D?.addEventListener("change",B=>{Dt.capsules=B.target.checked,$t()}),$t();let de=0,Xt=0,ct=-1,U=-1,ft=-1,pt=-1,zt=-1,Nt="",ue="",ce=null,_e=null,Ae=-1,oe="",Pe=It?.value||"pigtail",qe=vt?.value||"glidewire";const yi=.05;function Os(B,Mt,Ot,te){if(B){B.disabled!==Ot&&(B.disabled=Ot);const Ue=Ot?"Withdraw to 0 cm before changing selection":"";B.title!==Ue&&(B.title=Ue)}if(Mt){const Ue=Ot?`${te.toFixed(1)} cm inserted`:"Ready";Mt.textContent!==Ue&&(Mt.textContent=Ue),Mt.classList.contains("locked")!==Ot&&Mt.classList.toggle("locked",Ot)}}function An(){Os(vt,Rt,de>yi,de),Os(It,Jt,Xt>yi,Xt)}It?.addEventListener("change",B=>{Pe=B.target.value}),vt?.addEventListener("change",B=>{qe=B.target.value}),An(),ae?.addEventListener("click",()=>{typeof f=="function"&&f(5e3)}),qt?.addEventListener("click",()=>{typeof f=="function"&&f(6e5)}),Gt?.addEventListener("click",()=>{typeof d=="function"&&d()});const Wi=Array.from(document.querySelectorAll("[data-control-tab]")),Tn=Array.from(document.querySelectorAll("[data-control-panel]"));if(Wi.length&&Tn.length){const B=Mt=>{Wi.forEach(Ot=>{const te=Ot.dataset.controlTab===Mt;Ot.classList.toggle("active",te),Ot.setAttribute("aria-selected",te?"true":"false")}),Tn.forEach(Ot=>{Ot.classList.toggle("active",Ot.dataset.controlPanel===Mt)})};Wi.forEach(Mt=>{Mt.addEventListener("click",()=>B(Mt.dataset.controlTab))})}if([p,_,v,S,O,Y,K,J,it,X,j,nt,ot,ut,rt,z,L,F].filter(Boolean).forEach(B=>B.addEventListener("change",()=>B.blur())),M&&M.addEventListener("change",B=>{i.visible=B.target.checked}),document.querySelectorAll('#controls input[type="range"], #carm-controls input[type="range"]').forEach(B=>{const Mt=B.nextElementSibling;if(!Mt)return;const Ot=()=>{Mt.textContent=B.value};Ot(),B.addEventListener("input",Ot)}),document.querySelectorAll(".section-header").forEach(B=>{B.addEventListener("click",()=>{const Mt=B.nextElementSibling;B.classList.toggle("collapsed"),Mt&&Mt.classList.toggle("hidden")})}),q&&s.uniforms.autoExposureEnabled){let B=!!s.uniforms.autoExposureEnabled.value;const Mt=()=>{s.uniforms.autoExposureEnabled.value=B,q.textContent=`Auto exposure: ${B?"On":"Off"}`,q.classList.toggle("active",B)};Mt(),q.addEventListener("click",()=>{B=!B,Mt(),q.blur()})}if(K&&(s.uniforms.noiseLevel.value=parseFloat(K.value),K.addEventListener("input",B=>{s.uniforms.noiseLevel.value=parseFloat(B.target.value)})),Y&&s.uniforms.pulseRate){const B=Mt=>{s.uniforms.pulseRate.value=parseFloat(Mt.target.value)};s.uniforms.pulseRate.value=parseFloat(Y.value),Y.addEventListener("input",B),Y.addEventListener("change",B)}J&&s.uniforms.scatterStrength&&(s.uniforms.scatterStrength.value=parseFloat(J.value),J.addEventListener("input",B=>{s.uniforms.scatterStrength.value=parseFloat(B.target.value)})),it&&s.uniforms.collimation&&(s.uniforms.collimation.value=parseFloat(it.value),it.addEventListener("input",B=>{s.uniforms.collimation.value=parseFloat(B.target.value)})),X&&s.uniforms.imageBrightness&&(s.uniforms.imageBrightness.value=parseFloat(X.value),X.addEventListener("input",B=>{s.uniforms.imageBrightness.value=parseFloat(B.target.value)})),j&&s.uniforms.imageContrast&&(s.uniforms.imageContrast.value=parseFloat(j.value),j.addEventListener("input",B=>{s.uniforms.imageContrast.value=parseFloat(B.target.value)})),nt&&s.uniforms.edgeStrength&&(s.uniforms.edgeStrength.value=parseFloat(nt.value),nt.addEventListener("input",B=>{s.uniforms.edgeStrength.value=parseFloat(B.target.value)})),O&&(a.uniforms.decay.value=parseFloat(O.value),O.addEventListener("input",B=>{a.uniforms.decay.value=parseFloat(B.target.value)})),ot&&s.uniforms.boneOpacity&&(s.uniforms.boneOpacity.value=parseFloat(ot.value),ot.addEventListener("input",B=>{s.uniforms.boneOpacity.value=parseFloat(B.target.value)})),ut&&s.uniforms.contrastOpacity&&(s.uniforms.contrastOpacity.value=parseFloat(ut.value)/100,ut.addEventListener("input",B=>{s.uniforms.contrastOpacity.value=parseFloat(B.target.value)/100})),rt&&s.uniforms.contrastGain&&(s.uniforms.contrastGain.value=parseFloat(rt.value),rt.addEventListener("input",B=>{s.uniforms.contrastGain.value=parseFloat(B.target.value)})),p&&(parseFloat(p.value),p.addEventListener("input",B=>{parseFloat(B.target.value)})),_&&v&&(parseFloat(_.value),parseFloat(v.value),_.addEventListener("input",B=>{parseFloat(B.target.value)}),v.addEventListener("input",B=>{parseFloat(B.target.value)})),S&&(parseInt(S.value),S.addEventListener("input",B=>{parseInt(B.target.value)}));let nn=!0;if(y){const B=()=>{y.classList.toggle("fluoro-active",nn),y.classList.toggle("debug-active",!nn),y.setAttribute("aria-pressed",String(!nn)),y.setAttribute("aria-label",`Current view: ${nn?"fluoroscopy":"debug"}`)};B(),s.uniforms.fluoroscopy.value=!0,y.addEventListener("click",()=>{nn=!nn,s.uniforms.fluoroscopy.value=nn,B(),o&&o.color.set(16777215),typeof h=="function"&&h(nn)}),typeof h=="function"&&h(nn)}let Ei=0,st=0,Ut=0;const P=B=>{st=B},G=()=>{st=0},W=B=>{Ut=B},Z=()=>{Ut=0};function k(B,Mt,Ot){B&&(B.addEventListener("pointerdown",te=>{Mt(),B.setPointerCapture?.(te.pointerId),te.preventDefault()}),B.addEventListener("pointerup",Ot),B.addEventListener("pointercancel",Ot),B.addEventListener("pointerleave",te=>{te.buttons===0&&Ot()}))}k(bt,()=>P(1),G),k(V,()=>P(-1),G),k(re,()=>W(-1),Z),k(Tt,()=>W(1),Z),document.addEventListener("keydown",B=>{if((B.code==="KeyW"||B.code==="ArrowUp")&&(Ei=1,B.preventDefault()),(B.code==="KeyS"||B.code==="ArrowDown")&&(Ei=-1,B.preventDefault()),B.code==="KeyD"&&(st=1,B.preventDefault()),B.code==="KeyA"&&(st=-1,B.preventDefault()),B.code==="KeyE"&&(Ut=1,B.preventDefault()),B.code==="KeyQ"&&(Ut=-1,B.preventDefault()),B.code==="KeyC"&&nn){if(typeof c=="function"){const Mt=parseFloat(L.value),Ot=parseFloat(F.value)/1e3,te=parseFloat(z.value);c({rate:Mt,duration:Ot,volume:te})}B.preventDefault()}},!0),document.addEventListener("keyup",B=>{["KeyW","KeyS","ArrowUp","ArrowDown"].includes(B.code)&&(Ei=0,B.preventDefault()),["KeyA","KeyD"].includes(B.code)&&(st=0,B.preventDefault()),["KeyQ","KeyE"].includes(B.code)&&(Ut=0,B.preventDefault())},!0),window.addEventListener("blur",()=>{Ei=0,st=0,Ut=0}),R&&R.addEventListener("click",()=>{if(typeof c=="function"){const B=parseFloat(L.value),Mt=parseFloat(F.value)/1e3,Ot=parseFloat(z.value);c({rate:B,duration:Mt,volume:Ot})}}),N&&N.addEventListener("click",()=>{typeof l=="function"&&l()});function lt(B){de=Math.max(0,B);const Mt=Math.round(de*10);if(Mt===ct)return;ct=Mt;const Ot=(Mt/10).toFixed(1);ht&&(ht.textContent=`Wire ${Ot} cm`),An()}function At(B){Xt=Math.max(0,B);const Mt=Math.round(Xt*10);if(Mt===U)return;U=Mt;const Ot=(Mt/10).toFixed(1);dt&&(dt.textContent=`Catheter ${Ot} cm`),An()}function Ct(B){const Mt=Math.round(B*10);if(Mt===ft)return;ft=Mt;const Ot=(Mt/10).toFixed(1);I&&(I.textContent=`Contrast ${Ot} ml`)}function _t(B,Mt){const Ot=Math.round(B),te=Math.round(Mt*10);C&&Ot!==pt&&(Nt=`${Ot} kV`,C.textContent=Nt),H&&te!==zt&&(ue=`${(te/10).toFixed(1)} mA`,H.textContent=ue),pt=Ot,zt=te}function Vt(B,Mt=""){if(!Q)return;if(B<.35){ce!==!1&&(Q.classList.add("hidden"),Q.classList.remove("strong"),$&&($.textContent="Opór na prowadniku"),tt&&(tt.textContent="0%"),Et&&(Et.style.width="0%"),ce=!1,_e=!1,Ae=0,oe="");return}const Ot=Math.round(Math.max(0,Math.min(1,B))*100),te=B>.72,Ue=Mt||"Opór na prowadniku - cofnij lekko lub zmień kierunek.";ce!==!0&&(Q.classList.remove("hidden"),ce=!0),_e!==te&&(Q.classList.toggle("strong",te),_e=te),$&&oe!==Ue&&($.textContent=Ue,oe=Ue),Ae!==Ot&&(tt&&(tt.textContent=`${Ot}%`),Et&&(Et.style.width=`${Ot}%`),Ae=Ot)}function Ft(B){return Number.isFinite(B)?Math.abs(B)<10?B.toFixed(2):B.toFixed(1):"--"}function Bt(B){return Number.isFinite(B)?B<10?B.toFixed(2):B.toFixed(1):"--"}function fe(B){if(!B)return"";const Mt=Number.isFinite(B.settledPenetration)?` | pen ${Ft(B.settledPenetration)}/${Ft(B.maximumPenetration)} mm`:"";return`
XPBD: adv ${Bt(B.advanceMs)} / solve ${Bt(B.solveMs)} / narrow ${Bt(B.projectMs)} / dbg ${Bt(B.diagnosticMs)} ms | q ${B.pointContactCount}+${B.diagnosticPointContactCount} | segS ${B.segmentSampleCount}${Number.isFinite(B.activeBranchCount)?` | br ${B.activeBranchCount}`:""}`+Mt+`${B.foldGuarded?" | fold":""}${B.stabilityRepaired?" | repair":""}${B.withdrawalRelaxed?" | withdraw":""}`}function Ze(B=null){if(!at)return;if(at.classList.remove("warn","breach"),!B){at.textContent="GW STL: debug off";return}const Mt=fe(B.performance);if(!B.checkedCount||!Number.isFinite(B.minSignedDistance)){at.textContent=`GW STL: no lumen samples${Mt}`;return}at.classList.toggle("breach",B.outsideCount>0),at.classList.toggle("warn",B.outsideCount===0&&B.clearanceViolationCount>0),at.textContent=`GW STL: min ${Ft(B.minSignedDistance)} mm / clr ${Ft(B.clearance)} | out ${B.outsideCount} | near ${B.clearanceViolationCount} | seg ${Ft(B.maxSegmentError)} | bend ${Ft(B.maxBendAngle)} deg`+Mt}function Te(B){R&&R.disabled!==!!B&&(R.disabled=!!B)}function Kn(B){N&&N.disabled!==!!B&&(N.disabled=!!B)}let pe=0,Kt=0;function Yr(B){if(!et||(pe+=B,Kt++,pe<.25))return;const Mt=(Kt/Math.max(1e-6,pe)).toFixed(1);let Ot="N/A";performance.memory&&(Ot=(performance.memory.usedJSHeapSize/1048576).toFixed(1)+" MB"),et.textContent=`FPS: ${Mt} | Mem: ${Ot}`,pe=0,Kt=0}function Ee(B,Mt=null){const Ot=!!B?.running;if(ae&&(ae.disabled=Ot),qt&&(qt.disabled=Ot),Gt&&(Gt.disabled=!Ot),!wt)return;if(wt.classList.remove("passed","failed"),Ot){if(gt&&(gt.value="Running"),B.warmingUp){wt.textContent="Warming up";return}const Mn=Math.floor(B.elapsedMs/1e3),qr=Math.round(B.durationMs/1e3);wt.textContent=`Running ${Mn}/${qr} s · cycle ${B.cycleIndex+1}`;return}if(!Mt?.frameCount){wt.textContent="Idle",gt&&(gt.value="No report");return}const te=Mt.browserAcceptance,Ue=B.durationMs>=6e5&&B.elapsedMs>=6e5,Vs=Ue&&!!te?.passed;wt.classList.add(Vs?"passed":"failed"),wt.textContent=`${Ue?Vs?"PASS":"FAIL":"Smoke"} · ${Mt.averageFps.toFixed(1)} FPS · 1% ${Mt.onePercentLowFps.toFixed(1)} · pen ${Mt.physicsEnvelope.maxPostStepPenetrationMm.toFixed(3)} mm`,gt&&(gt.value=JSON.stringify(Mt))}function wi(B){const Mt=B===!0;Mt&&m?.reset?.(),m?.setLocked?.(Mt),document.body.classList.toggle("automated-benchmark-running",Mt)}return{monitor:g,getAdvance:()=>Ei,getCatheterAdvance:()=>st,getCatheterRotation:()=>Ut,getSelectedCatheterType:()=>Pe,getSelectedGuidewireType:()=>qe,getFluoroscopy:()=>nn,getDebugLayerState:()=>({...Dt}),updateInsertedLength:lt,updateCatheterLength:At,updateDose:Ct,updateXrayTechnique:_t,updateGuidewireResistance:Vt,updateGuidewireDiagnostics:Ze,setInjectButtonDisabled:Te,setStopInjectionDisabled:Kn,updatePerfStats:Yr,updateBrowserBenchmarkStatus:Ee,setAutomatedBenchmarkMode:wi,getCArmRevision:()=>m?.getRevision?.()??0}}const UE=/^[og]\s*(.+)?/,zE=/^mtllib /,BE=/^usemtl /,OE=/^usemap /,zf=/\s+/,Bf=new b,kl=new b,Of=new b,Gf=new b,Gn=new b,Ua=new jt;function GE(){const r={objects:[],object:{},vertices:[],normals:[],colors:[],uvs:[],materials:{},materialLibraries:[],startObject:function(t,e){if(this.object&&this.object.fromDeclaration===!1){this.object.name=t,this.object.fromDeclaration=e!==!1;return}const n=this.object&&typeof this.object.currentMaterial=="function"?this.object.currentMaterial():void 0;if(this.object&&typeof this.object._finalize=="function"&&this.object._finalize(!0),this.object={name:t||"",fromDeclaration:e!==!1,geometry:{vertices:[],normals:[],colors:[],uvs:[],hasUVIndices:!1},materials:[],smooth:!0,startMaterial:function(i,s){const a=this._finalize(!1);a&&(a.inherited||a.groupCount<=0)&&this.materials.splice(a.index,1);const o={index:this.materials.length,name:i||"",mtllib:Array.isArray(s)&&s.length>0?s[s.length-1]:"",smooth:a!==void 0?a.smooth:this.smooth,groupStart:a!==void 0?a.groupEnd:0,groupEnd:-1,groupCount:-1,inherited:!1,clone:function(c){const l={index:typeof c=="number"?c:this.index,name:this.name,mtllib:this.mtllib,smooth:this.smooth,groupStart:0,groupEnd:-1,groupCount:-1,inherited:!1};return l.clone=this.clone.bind(l),l}};return this.materials.push(o),o},currentMaterial:function(){if(this.materials.length>0)return this.materials[this.materials.length-1]},_finalize:function(i){const s=this.currentMaterial();if(s&&s.groupEnd===-1&&(s.groupEnd=this.geometry.vertices.length/3,s.groupCount=s.groupEnd-s.groupStart,s.inherited=!1),i&&this.materials.length>1)for(let a=this.materials.length-1;a>=0;a--)this.materials[a].groupCount<=0&&this.materials.splice(a,1);return i&&this.materials.length===0&&this.materials.push({name:"",smooth:this.smooth}),s}},n&&n.name&&typeof n.clone=="function"){const i=n.clone(0);i.inherited=!0,this.object.materials.push(i)}this.objects.push(this.object)},finalize:function(){this.object&&typeof this.object._finalize=="function"&&this.object._finalize(!0)},parseVertexIndex:function(t,e){const n=parseInt(t,10);return(n>=0?n-1:n+e/3)*3},parseNormalIndex:function(t,e){const n=parseInt(t,10);return(n>=0?n-1:n+e/3)*3},parseUVIndex:function(t,e){const n=parseInt(t,10);return(n>=0?n-1:n+e/2)*2},addVertex:function(t,e,n){const i=this.vertices,s=this.object.geometry.vertices;s.push(i[t+0],i[t+1],i[t+2]),s.push(i[e+0],i[e+1],i[e+2]),s.push(i[n+0],i[n+1],i[n+2])},addVertexPoint:function(t){const e=this.vertices;this.object.geometry.vertices.push(e[t+0],e[t+1],e[t+2])},addVertexLine:function(t){const e=this.vertices;this.object.geometry.vertices.push(e[t+0],e[t+1],e[t+2])},addNormal:function(t,e,n){const i=this.normals,s=this.object.geometry.normals;s.push(i[t+0],i[t+1],i[t+2]),s.push(i[e+0],i[e+1],i[e+2]),s.push(i[n+0],i[n+1],i[n+2])},addFaceNormal:function(t,e,n){const i=this.vertices,s=this.object.geometry.normals;Bf.fromArray(i,t),kl.fromArray(i,e),Of.fromArray(i,n),Gn.subVectors(Of,kl),Gf.subVectors(Bf,kl),Gn.cross(Gf),Gn.normalize(),s.push(Gn.x,Gn.y,Gn.z),s.push(Gn.x,Gn.y,Gn.z),s.push(Gn.x,Gn.y,Gn.z)},addColor:function(t,e,n){const i=this.colors,s=this.object.geometry.colors;i[t]!==void 0&&s.push(i[t+0],i[t+1],i[t+2]),i[e]!==void 0&&s.push(i[e+0],i[e+1],i[e+2]),i[n]!==void 0&&s.push(i[n+0],i[n+1],i[n+2])},addUV:function(t,e,n){const i=this.uvs,s=this.object.geometry.uvs;s.push(i[t+0],i[t+1]),s.push(i[e+0],i[e+1]),s.push(i[n+0],i[n+1])},addDefaultUV:function(){const t=this.object.geometry.uvs;t.push(0,0),t.push(0,0),t.push(0,0)},addUVLine:function(t){const e=this.uvs;this.object.geometry.uvs.push(e[t+0],e[t+1])},addFace:function(t,e,n,i,s,a,o,c,l){const h=this.vertices.length;let u=this.parseVertexIndex(t,h),f=this.parseVertexIndex(e,h),d=this.parseVertexIndex(n,h);if(this.addVertex(u,f,d),this.addColor(u,f,d),o!==void 0&&o!==""){const g=this.normals.length;u=this.parseNormalIndex(o,g),f=this.parseNormalIndex(c,g),d=this.parseNormalIndex(l,g),this.addNormal(u,f,d)}else this.addFaceNormal(u,f,d);if(i!==void 0&&i!==""){const g=this.uvs.length;u=this.parseUVIndex(i,g),f=this.parseUVIndex(s,g),d=this.parseUVIndex(a,g),this.addUV(u,f,d),this.object.geometry.hasUVIndices=!0}else this.addDefaultUV()},addPointGeometry:function(t){this.object.geometry.type="Points";const e=this.vertices.length;for(let n=0,i=t.length;n<i;n++){const s=this.parseVertexIndex(t[n],e);this.addVertexPoint(s),this.addColor(s)}},addLineGeometry:function(t,e){this.object.geometry.type="Line";const n=this.vertices.length,i=this.uvs.length;for(let s=0,a=t.length;s<a;s++)this.addVertexLine(this.parseVertexIndex(t[s],n));for(let s=0,a=e.length;s<a;s++)this.addUVLine(this.parseUVIndex(e[s],i))}};return r.startObject("",!1),r}class VE extends Wc{constructor(t){super(t),this.materials=null}load(t,e,n,i){const s=this,a=new Rm(this.manager);a.setPath(this.path),a.setRequestHeader(this.requestHeader),a.setWithCredentials(this.withCredentials),a.load(t,function(o){try{e(s.parse(o))}catch(c){i?i(c):console.error(c),s.manager.itemError(t)}},n,i)}setMaterials(t){return this.materials=t,this}parse(t){const e=new GE;t.indexOf(`\r
`)!==-1&&(t=t.replace(/\r\n/g,`
`)),t.indexOf(`\\
`)!==-1&&(t=t.replace(/\\\n/g,""));const n=t.split(`
`);let i=[];for(let o=0,c=n.length;o<c;o++){const l=n[o].trimStart();if(l.length===0)continue;const h=l.charAt(0);if(h!=="#")if(h==="v"){const u=l.split(zf);switch(u[0]){case"v":e.vertices.push(parseFloat(u[1]),parseFloat(u[2]),parseFloat(u[3])),u.length>=7?(Ua.setRGB(parseFloat(u[4]),parseFloat(u[5]),parseFloat(u[6])).convertSRGBToLinear(),e.colors.push(Ua.r,Ua.g,Ua.b)):e.colors.push(void 0,void 0,void 0);break;case"vn":e.normals.push(parseFloat(u[1]),parseFloat(u[2]),parseFloat(u[3]));break;case"vt":e.uvs.push(parseFloat(u[1]),parseFloat(u[2]));break}}else if(h==="f"){const f=l.slice(1).trim().split(zf),d=[];for(let x=0,m=f.length;x<m;x++){const p=f[x];if(p.length>0){const _=p.split("/");d.push(_)}}const g=d[0];for(let x=1,m=d.length-1;x<m;x++){const p=d[x],_=d[x+1];e.addFace(g[0],p[0],_[0],g[1],p[1],_[1],g[2],p[2],_[2])}}else if(h==="l"){const u=l.substring(1).trim().split(" ");let f=[];const d=[];if(l.indexOf("/")===-1)f=u;else for(let g=0,x=u.length;g<x;g++){const m=u[g].split("/");m[0]!==""&&f.push(m[0]),m[1]!==""&&d.push(m[1])}e.addLineGeometry(f,d)}else if(h==="p"){const f=l.slice(1).trim().split(" ");e.addPointGeometry(f)}else if((i=UE.exec(l))!==null){const u=(" "+i[0].slice(1).trim()).slice(1);e.startObject(u)}else if(BE.test(l))e.object.startMaterial(l.substring(7).trim(),e.materialLibraries);else if(zE.test(l))e.materialLibraries.push(l.substring(7).trim());else if(OE.test(l))console.warn('THREE.OBJLoader: Rendering identifier "usemap" not supported. Textures must be defined in MTL files.');else if(h==="s"){if(i=l.split(" "),i.length>1){const f=i[1].trim().toLowerCase();e.object.smooth=f!=="0"&&f!=="off"}else e.object.smooth=!0;const u=e.object.currentMaterial();u&&(u.smooth=e.object.smooth)}else{if(l==="\0")continue;console.warn('THREE.OBJLoader: Unexpected line: "'+l+'"')}}e.finalize();const s=new De;if(s.materialLibraries=[].concat(e.materialLibraries),!(e.objects.length===1&&e.objects[0].geometry.vertices.length===0)===!0)for(let o=0,c=e.objects.length;o<c;o++){const l=e.objects[o],h=l.geometry,u=l.materials,f=h.type==="Line",d=h.type==="Points";let g=!1;if(h.vertices.length===0)continue;const x=new ye;x.setAttribute("position",new ie(h.vertices,3)),h.normals.length>0&&x.setAttribute("normal",new ie(h.normals,3)),h.colors.length>0&&(g=!0,x.setAttribute("color",new ie(h.colors,3))),h.hasUVIndices===!0&&x.setAttribute("uv",new ie(h.uvs,2));const m=[];for(let _=0,v=u.length;_<v;_++){const S=u[_],y=S.name+"_"+S.smooth+"_"+g;let M=e.materials[y];if(this.materials!==null){if(M=this.materials.create(S.name),f&&M&&!(M instanceof vi)){const w=new vi;Si.prototype.copy.call(w,M),w.color.copy(M.color),M=w}else if(d&&M&&!(M instanceof Io)){const w=new Io({size:10,sizeAttenuation:!1});Si.prototype.copy.call(w,M),w.color.copy(M.color),w.map=M.map,M=w}}M===void 0&&(f?M=new vi:d?M=new Io({size:1,sizeAttenuation:!1}):M=new ny,M.name=S.name,M.flatShading=!S.smooth,M.vertexColors=g,e.materials[y]=M),m.push(M)}let p;if(m.length>1){for(let _=0,v=u.length;_<v;_++){const S=u[_];x.addGroup(S.groupStart,S.groupCount,_)}f?p=new ds(x,m):d?p=new Rl(x,m):p=new Zt(x,m)}else f?p=new ds(x,m[0]):d?p=new Rl(x,m[0]):p=new Zt(x,m[0]);p.name=l.name,s.add(p)}else if(e.vertices.length>0){const o=new Io({size:1,sizeAttenuation:!1}),c=new ye;c.setAttribute("position",new ie(e.vertices,3)),e.colors.length>0&&e.colors[0]!==void 0&&(c.setAttribute("color",new ie(e.colors,3)),o.vertexColors=!0);const l=new Rl(c,o);s.add(l)}return s}}function kE({onLoaded:r,onError:t}={}){const e=new Fe({color:16777215,transparent:!0,opacity:.42,depthWrite:!1,depthTest:!1,blending:gc,side:Ke}),n=new De;return new VE().load("res/skeleton.obj",s=>{s.traverse(c=>{c.isMesh&&(c.material=e)});const o=new en().setFromObject(s).getCenter(new b);s.position.sub(o),s.rotation.z=-Math.PI/3,s.scale.multiplyScalar(9),s.position.x-=1760,s.position.y-=300,s.position.z-=70,n.add(s),typeof r=="function"&&r({group:n,object:s,material:e})},void 0,s=>{console.warn("Failed to load skeleton OBJ model",s),typeof t=="function"&&t(s)}),{group:n,material:e}}const HE=new b(0,1,0);function Bh(r){return new b(r.x,r.y,r.z)}function Vf(r,t){return he.clamp(Math.floor(r),0,Math.max(0,t-1))}function Oh(r,t,e){const n=he.clamp((e-r)/Math.max(1e-6,t-r),0,1);return n*n*(3-2*n)}function kf(r,t){return t<0||t>=r.cells?0:r.core[t]*.58+r.wall[t]*.72}function Hf(r,t){return t<0||t>=r.cells?0:r.wall[t]}function WE(r,t){const e=Math.floor(t),n=t-e;return he.lerp(kf(r,e),kf(r,e+1),n)}function XE(r,t){const e=Math.floor(t),n=t-e;return he.lerp(Hf(r,e),Hf(r,e+1),n)}function YE(r,t){return r.segments.map((e,n)=>{const i=Bh(e.start),s=Bh(e.end),a=new b().subVectors(s,i),o=Math.max(1,a.length()),c=a.clone().normalize(),l=Math.max(2,Math.ceil(o/t)),h=o/l;return{sourceSegment:e,segmentIndex:n,start:i,end:s,dir:c,length:o,cells:l,cellLength:h,radius:e.radius,area:Math.PI*e.radius*e.radius,flowSpeed:e.flowSpeed||0,isSheath:!!e.isSheath,core:new Float32Array(l),wall:new Float32Array(l),nextCore:new Float32Array(l),nextWall:new Float32Array(l),orientation:new Nn().setFromUnitVectors(HE,c)}})}class qE{constructor(t,e=3.5){this.vessel=t,this.segments=YE(t,e),this.segmentGraph=t.segmentGraph||t.segments.map(()=>[]),this.outgoing=Array.from({length:this.segments.length*2},()=>({segmentIndex:-1,amount:0,wallShare:0,sourceArea:0})),this.outgoingCount=0,this.sheathSegmentIndex=t.segments.findIndex(n=>n.isSheath),this.time=0,this.totalSignal=0,this.lastInjectionTime=-1/0,this.coreSpeedScale=1.82,this.wallSpeedScale=1.24,this.wallExchange=4.6,this.axialDispersion=.46,this.clearance=.95,this.tailClearance=3.1}injectThroughSheath(t,e=0){if(t<=0)return;if(this.lastInjectionTime=this.time,this.sheathSegmentIndex>=0){const s=this.segments[this.sheathSegmentIndex];this.#n(s,s.cells-1,t*.06,.48)}const n=this.vessel.sheath?.end,i=n?this.#h(n,{excludeSheath:!0}):null;if(!i){const s=this.segments.find(a=>!a.isSheath);s&&this.#n(s,0,t,.85);return}this.#t(i,t*.92,e)}update(t){if(this.time+=t,this.totalSignal<=0)return;this.totalSignal=0;const e=1+.18*Math.sin(this.time*Math.PI*2.15);this.outgoingCount=0;for(let i=0;i<this.segments.length;i++){const s=this.segments[i];s.nextCore.set(s.core),s.nextWall.set(s.wall);const a=Math.min(.96,Math.max(0,s.flowSpeed*this.coreSpeedScale*e*t/s.cellLength)),o=Math.min(.78,Math.max(0,s.flowSpeed*this.wallSpeedScale*t/s.cellLength));this.#i(s,s.core,s.nextCore,a,1),this.#i(s,s.wall,s.nextWall,o,.35),s.core.set(s.nextCore),s.wall.set(s.nextWall),this.#l(s,t)}for(let i=0;i<this.outgoingCount;i++)this.#a(this.outgoing[i]);const n=this.time-this.lastInjectionTime>.38;for(let i=0;i<this.segments.length;i++){const s=this.segments[i];for(let a=0;a<s.cells;a++){const o=s.core[a]+s.wall[a]*.8,c=n?1-Oh(.012,.13,o):0,l=Math.exp(-(this.clearance+c*this.tailClearance)*t),h=Math.exp(-(this.clearance*1.2+c*this.tailClearance*1.35)*t);s.core[a]*=l,s.wall[a]*=h,s.core[a]<4e-4&&(s.core[a]=0),s.wall[a]<4e-4&&(s.wall[a]=0),this.totalSignal+=s.core[a]+s.wall[a]*.8}}}hasVisibleContrast(t=.02){return this.totalSignal>t}#t(t,e,n){const i=t.segment,s=t.cellIndex,a=he.clamp(n/45,.35,1.35);this.#n(i,s,e*.25,.72);const o=s+1,c=Math.max(5,Math.min(o,Math.round(18*a)));let l=0;const h=[];for(let d=0;d<c;d++){const g=s-d;if(g<0)break;const x=Math.exp(-d/(5.5+a*4));h.push([g,x]),l+=x}for(const[d,g]of h)this.#n(i,d,e*.4*g/l,.64);const u=i.sourceSegment?.parent,f=Number.isInteger(u)?this.segments[u]:null;if(f){const d=Math.min(f.cells,Math.round(24*a));let g=0;const x=[];for(let m=0;m<d;m++){const p=f.cells-1-m,_=Math.exp(-m/8);x.push([p,_]),g+=_}for(const[m,p]of x)this.#n(f,m,e*.35*p/g,.58)}}#n(t,e,n,i){if(!t||n<=0)return;const s=Vf(e,t.cells),a=Math.max(1,t.area*t.cellLength),o=n*1e3/a;t.core[s]+=o*i,t.wall[s]+=o*(1-i),this.totalSignal+=o}#i(t,e,n,i,s){if(!(i<=0))for(let a=t.cells-1;a>=0;a--){const o=e[a]*i;if(n[a]-=o,a+1<t.cells)n[a+1]+=o;else if(o>0){const c=this.outgoing[this.outgoingCount++];c.segmentIndex=t.segmentIndex,c.amount=o,c.wallShare=s,c.sourceArea=t.area}}}#l(t,e){const n=he.clamp(this.wallExchange*e,0,.22),i=he.clamp(this.axialDispersion*e,0,.08);for(let s=0;s<t.cells;s++){const a=(t.core[s]-t.wall[s])*n;t.core[s]-=a,t.wall[s]+=a}if(!(i<=0||t.cells<3)){t.nextCore.set(t.core),t.nextWall.set(t.wall);for(let s=1;s<t.cells-1;s++)t.nextCore[s]+=(t.core[s-1]+t.core[s+1]-t.core[s]*2)*i,t.nextWall[s]+=(t.wall[s-1]+t.wall[s+1]-t.wall[s]*2)*i*1.35;t.core.set(t.nextCore),t.wall.set(t.nextWall)}}#a(t){const e=this.segmentGraph[t.segmentIndex]||[];if(!e.length)return;let n=0;for(let i=0;i<e.length;i++)n+=this.segments[e[i]]?.area||0;n<=0&&(n=e.length);for(let i=0;i<e.length;i++){const s=e[i],a=this.segments[s];if(!a)continue;const o=(a.area||1)/n,c=t.amount*o*t.sourceArea*a.cellLength/1e3;this.#n(a,0,c,.58+(1-t.wallShare)*.18)}}#h(t,{excludeSheath:e=!1}={}){const n=Bh(t);let i=null;for(const s of this.segments){if(e&&s.isSheath)continue;const a=new b().subVectors(n,s.start),o=he.clamp(a.dot(s.dir),0,s.length),l=s.start.clone().addScaledVector(s.dir,o).distanceTo(n),h=Math.max(0,l-s.radius),u=Vf(o/s.cellLength,s.cells);(!i||h<i.score)&&(i={segment:s,segmentIndex:s.segmentIndex,cellIndex:u,score:h})}return i}}function ZE(r,t=.015,e=!1,n=null){if(!r?.segments)return{mesh:n,count:0};const i=r.vessel?.geometry;if(!i?.attributes?.position)return{mesh:n,count:0};if(!n||!n.isMesh||n.userData.sourceGeometry!==i){$E(n);const c=i.clone(),l=c.attributes.position.count;c.setAttribute("color",new Ne(new Float32Array(l*3),3));const h=new Fe({vertexColors:!0,transparent:!0,opacity:e?.78:.96,blending:gc,depthTest:!1,depthWrite:!1,side:Ke,wireframe:e});n=new Zt(c,h),n.frustumCulled=!1,n.userData.sourceGeometry=i,n.userData.influences=jE(r,c)}const s=n.geometry.attributes.color,a=n.userData.influences||[];let o=0;for(let c=0;c<s.count;c++){const l=a[c];let h=0,u=0;if(l?.length)for(const m of l){const p=r.segments[m.segmentIndex];h+=WE(p,m.cellFloat)*m.weight,u+=XE(p,m.cellFloat)*m.weight}const f=Oh(t*.18,t*4.4,h),d=Oh(t*.28,t*4,u),g=Math.max(f,d*.82),x=g>.018?Math.min(1,Math.pow(g,.78)*1.18):0;x>.02&&o++,s.setXYZ(c,x,x,x)}return s.needsUpdate=!0,n.visible=o>0,n.material.wireframe=e,n.material.opacity=e?.78:.96,{mesh:n,count:o}}function jE(r,t){const e=t.attributes.position,n=new b,i=new Array(e.count);for(let s=0;s<e.count;s++){n.fromBufferAttribute(e,s);const a=[];for(const l of r.segments){if(l.isSheath)continue;const u=new b().subVectors(n,l.start).dot(l.dir),f=he.clamp(u,0,l.length),d=l.start.clone().addScaledVector(l.dir,f),g=n.distanceTo(d),x=Math.max(0,-u,u-l.length),m=Math.abs(g-l.radius)+x*.45,p=Math.max(2,l.radius*.48),_=Math.exp(-(m*m)/(2*p*p));if(_<.02)continue;const v=he.clamp(f/l.cellLength-.5,0,l.cells-1);a.push({segmentIndex:l.segmentIndex,cellFloat:v,weight:_})}a.sort((l,h)=>h.weight-l.weight);const o=a.slice(0,3),c=o.reduce((l,h)=>l+h.weight,0);i[s]=c>0?o.map(l=>({...l,weight:l.weight/c})):[]}return i}function $E(r){if(r)if(r.isGroup)for(const t of r.children)t.geometry?.dispose?.(),t.material?.dispose?.();else r.geometry?.dispose?.(),r.material?.dispose?.()}const Gh=wc,Wf=7.2,KE=1.05,Xf="pigtail",oo="berenstein",JE=Math.PI/4,QE=8,t1=10,e1=48,ao=48,Hl=16,za=18,Ki=4,ys=3.2,n1=76,i1=.105,Wl=.96,Yf=.88,s1=18,r1=4,Xl=.085,Yl=.08,o1=.22,a1=.075,ql=72*Math.PI/180,c1=.36,qf=.55,Zf=1.2,l1=[.25,.5,.75],jf=[2,4,7],h1=3,$f=[2,4,8],u1=.24,d1=.085,Zl=68*Math.PI/180,f1=.42,Kf=1.15,jl=1.7,p1=1.45,Ba=7,m1=.22,g1=.42,$l=.42,x1=.82,_1=.78,Jf=1.2,v1=90,S1=52,M1=32,y1=Math.PI*.9,Qf=Gh*.72;class Lt extends b{constructor(t=0,e=0,n=0){super(t,e,n),this._values=new Float64Array([this._initialX??t,this._initialY??e,this._initialZ??n])}get x(){return this._values?this._values[0]:this._initialX}set x(t){this._values?this._values[0]=t:this._initialX=t}get y(){return this._values?this._values[1]:this._initialY}set y(t){this._values?this._values[1]=t:this._initialY=t}get z(){return this._values?this._values[2]:this._initialZ}set z(t){this._values?this._values[2]=t:this._initialZ=t}}function Es(r,t,e){return Math.sqrt(r*r+t*t+e*e)}function co(r){return new Lt(r.x,r.y,r.z)}class E1{constructor({wire:t,segmentLength:e,guidewireLength:n,tailProgressRef:i,vessel:s=null,maxLength:a=1e3}){this.wire=t,this.segmentLength=e,this.guidewireLength=n,this.tailProgressRef=i,this.vessel=s,this.vesselColliders=this.#e(s),this.collisionMesh=null,this.sheathPath=this.#V(s?.sheath),this.maxLength=a,this.progress=0,this.guidewireInserted=0,this.previousGuidewireInserted=0,this.guidewireDelta=0,this.motionCommand=0,this.rotation=0,this.type=Xf,this.pathSpacing=4,this.pathSamples=[],this._pathSamplePool=Array.from({length:Math.ceil(a/this.pathSpacing)+4},()=>({distance:0,point:new Lt})),this.freeNodes=[],this._nextFreeNodes=[],this._freeNodePool=[],this._freeNodeEpoch=0,this.freeRestDistances=new Float64Array(Math.ceil(a/ys)+2),this.freeRestDistanceCount=0,this.freeLength=0,this._physicsStepIndex=0,this.material=new Fe({color:16777215,depthTest:!1,transparent:!0,opacity:1}),this.maxRenderSegments=320,this.mesh=new Hr(new gs(Bl,Bl,1,10,1,!1),this.material,this.maxRenderSegments),this.mesh.instanceMatrix.setUsage(Vc),this.mesh.count=0,this.mesh.frustumCulled=!1,this.mesh.renderOrder=7,this.mesh.visible=!1,this.physicsBody=null,this.physicsActiveCount=0,this.physicsLumenStartNode=0,this.externalCollisionSolver=!1,this._renderAxis=new Lt,this._renderMidpoint=new Lt,this._renderUp=new Lt(0,1,0),this._renderQuaternion=new Nn,this._renderScale=new Lt(1,1,1),this._renderMatrix=new se,this._shapeNormal=new Lt,this._pathTarget=new Lt,this._newNodeRest=new Lt,this._newNodePath=new Lt,this._newNodeGuide=new Lt,this._newNodePoint=new Lt,this._centerlinePoints=[],this._centerlinePointCount=0,this._deploymentStateScratch={pathEnd:0,supportEnd:0,freeLength:0},this._freeFrameScratch={supportTip:new Lt,beforeTip:new Lt,beforePlane:new Lt,tangent:new Lt,normal:new Lt},this._planePreviousTangent=new Lt,this._planeCurvature=new Lt,this._planeHelper=new Lt}setType(t){const e=this.#Z(t);this.type!==e&&(this.type=e,this.#m(),this.freeRestDistanceCount=0,this.freeLength=0,this._physicsStepIndex=0,this.physicsLumenStartNode=0,this.updateMesh())}dispose(){this.mesh.geometry?.dispose?.(),this.material.dispose()}setExternalCollisionSolver(t=!0){return this.externalCollisionSolver=!!t,this}reset(){return this.progress=0,this.guidewireInserted=0,this.previousGuidewireInserted=0,this.guidewireDelta=0,this.motionCommand=0,this.rotation=0,this.pathSamples.length=0,this.#m(),this.freeRestDistanceCount=0,this.freeLength=0,this._physicsStepIndex=0,this.updateMesh(),this}syncXpbdBody(t,{shapeCompliance:e=2e-4}={}){const n=this.#t(),i=Math.min(this._centerlinePointCount,t.count);if(this.physicsBody=t,i<2){for(let c=0;c<this.physicsActiveCount;c++)t.clearRestShapeTarget(c);return t.setActiveRange(0,1),t.setCollisionRange(0,-1),this.physicsActiveCount=0,0}const s=this.physicsActiveCount;t.setActiveRange(0,i-1);for(let c=0;c<i;c++){const l=n[c];if(c>=s&&t.setNodePosition(c,l.x,l.y,l.z),t.setRestShapeTarget(c,l.x,l.y,l.z,e),t.nodeRadius[c]=Gh,c>0){const h=n[c-1];t.restLength[c-1]=Math.max(.5,l.distanceTo(h))}c>0&&c<i-1&&(t.restBendChord[c]=n[c-1].distanceTo(n[c+1]))}for(let c=i;c<s;c++)t.clearRestShapeTarget(c);let a=i-1;const o=this.vessel?.sheath;if(o){const c=o.end.x-o.start.x,l=o.end.y-o.start.y,h=o.end.z-o.start.z,u=Es(c,l,h)||1,f=c/u,d=l/u,g=h/u;for(let x=0;x<i;x++){const m=n[x];if((m.x-o.start.x)*f+(m.y-o.start.y)*d+(m.z-o.start.z)*g>u+.25){a=Math.max(0,x-1);break}}}return t.setCollisionRange(a,i-2),this.physicsActiveCount=i,i}setCollisionGeometry(t){const e=t?.geometry||t;if(!e?.boundsTree){this.collisionMesh=null;return}this.collisionMesh={geometry:e,meshCollider:t?.meshCollider||null,clearance:Math.max(Gh*.7,t?.clearance||0),interiorDirection:t?.interiorDirection||t?.collisionInteriorDirection||null}}advance(t,e,n){this.motionCommand=t,this.previousGuidewireInserted=this.guidewireInserted,this.guidewireInserted=Math.max(0,n),this.guidewireDelta=this.guidewireInserted-this.previousGuidewireInserted;const i=t>0?S1:M1,s=kt(this.progress+t*i*e,0,this.maxLength);s>this.progress?this.#G(Math.min(s,this.guidewireInserted)):s<this.progress&&this.#H(s);const a=Math.min(s,this.guidewireInserted);(t!==0||this.guidewireDelta>0)&&a>za&&this.#k(a),this.progress=s}rotate(t,e){t&&(this.rotation+=t*y1*e)}stepPhysics(t=1/60,{collisions:e=!0}={}){const n=this.#i(),i=this._physicsStepIndex++;if((!this.externalCollisionSolver||(i&3)===0)&&this.#r(n.pathEnd),this.externalCollisionSolver&&(i&1)===1)return;if(n.freeLength<2||n.supportEnd<=0){this.#m(),this.freeRestDistanceCount=0,this.freeLength=0;return}const s=this.#l(n.supportEnd);if(this.#a(n,s),this.freeNodes.length<2)return;this.#g(n);const a=s.supportTip;for(let l=0;l<this.freeNodes.length;l++){const h=this.freeNodes[l];h.previousPos||=new Lt,h.shapeTarget||=new Lt,h.guideTarget||=new Lt,h.previousPos.copy(h.pos)}this.freeNodes[0].pos.copy(a),this.freeNodes[0].vel.set(0,0,0);for(let l=1;l<this.freeNodes.length;l++){const h=this.freeNodes[l];h.curl=Math.min(1,(h.curl??1)+x1*t);const u=Math.max(0,(h.distance??0)-(this.freeNodes[0].distance??0)),f=this.#R(u,s,n.freeLength,h.curl,h.shapeTarget),d=n1*t;h.vel.x+=(f.x-h.pos.x)*d,h.vel.y+=(f.y-h.pos.y)*d,h.vel.z+=(f.z-h.pos.z)*d,h.vel.multiplyScalar(Yf),h.pos.addScaledVector(h.vel,t)}const o=e?s1:r1;for(let l=0;l<o;l++)this.freeNodes[0].pos.copy(a),this.#g(n),this.#y(),this.#w(s,n.freeLength),this.#E(s),this.#C(n.freeLength),this.#L(n.freeLength),e&&(this.#o(),this.#f()),this.#y();const c=1/Math.max(1e-4,t);for(let l=1;l<this.freeNodes.length;l++){const h=this.freeNodes[l];h.vel.subVectors(h.pos,h.previousPos).multiplyScalar(c*Yf)}this.freeNodes[0].vel.set(0,0,0)}constrainGuidewire(t=1/60,{reactionScale:e=1}={}){if(this.progress<4)return;const n=this.#i();n.freeLength>=2&&this.freeNodes.length<2&&n.supportEnd>0&&this.#a(n,this.#l(n.supportEnd));const i=this.tailProgressRef(),s=this.freeNodes.length>=2?Math.max(n.pathEnd,this.progress):n.pathEnd,a=Math.min(this.progress,this.guidewireInserted,s);if(!(a<=0))for(let o=0;o<this.wire.nodes.length;o++){const c=this.#q(o,i);if(c<=0||c>a)continue;const l=this.wire.nodes[o];if(l.pinned)continue;const h=this.#W(c,n),u=l.x,f=l.y,d=l.z,x=(.6+ln(0,this.segmentLength*1.5,c)*.4)*_1,m=h.clone().sub(new Lt(u,f,d)),p=m.length();p>Jf&&m.multiplyScalar(Jf/p);const _=m.multiplyScalar(x);l.x=u+_.x,l.y=f+_.y,l.z=d+_.z;const v=1/Math.max(1e-4,t);l.vx=(l.x-u)*v*.25,l.vy=(l.y-f)*v*.25,l.vz=(l.z-d)*v*.25,this.#X(c,_,e)}}updateMesh(){const t=this.physicsBody,e=t?null:this.#t(),n=t?this.physicsActiveCount:this._centerlinePointCount;if(n<2){this.mesh.count=0,this.mesh.visible=!1;return}const i=Math.min(n-1,this.maxRenderSegments);let s=0;for(let a=0;a<i;a++){const o=t?t.x[a]:e[a].x,c=t?t.y[a]:e[a].y,l=t?t.z[a]:e[a].z,h=t?t.x[a+1]:e[a+1].x,u=t?t.y[a+1]:e[a+1].y,f=t?t.z[a+1]:e[a+1].z;this._renderAxis.set(h-o,u-c,f-l);const d=this._renderAxis.length();d<1e-6||(this._renderAxis.multiplyScalar(1/d),this._renderMidpoint.set((o+h)*.5,(c+u)*.5,(l+f)*.5),this._renderQuaternion.setFromUnitVectors(this._renderUp,this._renderAxis),this._renderScale.set(1,d+Bl*.65,1),this._renderMatrix.compose(this._renderMidpoint,this._renderQuaternion,this._renderScale),this.mesh.setMatrixAt(s++,this._renderMatrix))}this.mesh.count=s,this.mesh.instanceMatrix.needsUpdate=!0,this.mesh.visible=s>0}#t(){const t=this.#i(),e=this.sheathPath?v1:0;if(this.physicsLumenStartNode=0,this._centerlinePointCount=0,t.pathEnd<=0&&e<=0)return this._centerlinePoints;const n=Math.max(0,t.supportEnd),i=n>0?kt(Math.ceil(n/5),1,90):0,s=this._centerlinePoints;if(e>0){const c=kt(Math.ceil(e/6),2,24);for(let l=0;l<=c;l++){const h=-e+e*l/c;this.#b(h,this.#n(this._centerlinePointCount++))}this.physicsLumenStartNode=c}if(t.pathEnd<=0)return s;const a=this._centerlinePointCount?1:0;for(let c=a;c<=i;c++){const l=i>0?n*c/i:0;this.#b(l,this.#n(this._centerlinePointCount++))}if(t.freeLength<2)return t.pathEnd>n+.5&&this.#b(t.pathEnd,this.#n(this._centerlinePointCount++)),s;const o=this.#l(t.supportEnd);this.#a(t,o);for(let c=1;c<this.freeNodes.length;c++)this.#n(this._centerlinePointCount++).copy(this.freeNodes[c].pos);return s}#n(t){let e=this._centerlinePoints[t];return e||(e=new Lt,this._centerlinePoints[t]=e),e}#i(){const t=this._deploymentStateScratch;if(this.progress<4)return t.pathEnd=0,t.supportEnd=0,t.freeLength=0,t;const e=this.#B(),n=Math.max(e,Math.min(this.progress,this.#D()));return t.pathEnd=n,t.supportEnd=n>0?e:0,t.freeLength=n>0?Math.max(0,this.progress-e):0,t}#l(t){const e=this._freeFrameScratch,n=this.#b(t,e.supportTip),i=this.#b(Math.max(0,t-10),e.beforeTip),s=this.#b(Math.max(0,t-28),e.beforePlane),a=e.tangent.subVectors(n,i);return a.lengthSq()<1e-5&&a.set(0,1,0),a.normalize(),this.#Y(a,i,s,e.normal).applyAxisAngle(a,this.rotation).normalize(),e}#a(t,e){const n=this.freeRestDistances;n[0]=t.supportEnd;let i=1,s=t.supportEnd;for(;s+ys<this.progress-.5;)s+=ys,n[i++]=s;this.progress>n[i-1]+.5&&(n[i++]=this.progress),this.freeRestDistanceCount=i;const a=this.freeNodes,o=this._nextFreeNodes;o.length=0;const c=++this._freeNodeEpoch;let l=0;for(let h=0;h<i;h++){const u=n[h],f=u-t.supportEnd;let d=-1,g=1/0;for(;l<a.length;){const m=Math.abs((a[l].distance??0)-u);if((l+1<a.length?Math.abs((a[l+1].distance??0)-u):1/0)>=m){d=l,g=m;break}l++}let x;if(d>=0&&g<=ys*.7)x=a[d],l=d+1;else{const m=this.guidewireDelta<-1e-4&&u>=this.guidewireInserted-Ki&&u<=this.previousGuidewireInserted+Ki,p=this.#R(f,e,t.freeLength,m?$l:1,this._newNodeRest),_=this.#b(Math.min(u,this.#D()),this._newNodePath),v=this.guidewireInserted>za&&u<=this.guidewireInserted+Ki,S=this._newNodePoint;m?S.copy(_).lerp(p,$l):v?S.copy(this.#P(u,this._newNodeGuide)).lerp(p,.28):S.copy(p);const y=this.externalCollisionSolver?S:this.#c(S).point;x=this.#h(y,u,m?$l:1)}x._activeEpoch=c,x.distance=u,x.curl=x.curl??1,x.previousPos||=new Lt,x.shapeTarget||=new Lt,x.guideTarget||=new Lt,o.push(x)}for(let h=0;h<a.length;h++){const u=a[h];u._activeEpoch===c||u._pooled||(u._pooled=!0,this._freeNodePool.push(u))}this._nextFreeNodes=a,this.freeNodes=o,this.freeLength=t.freeLength,this.freeNodes[0]&&(this.freeNodes[0].pos.copy(e.supportTip),this.freeNodes[0].vel.set(0,0,0))}#h(t,e,n){const i=this._freeNodePool.pop()||{pos:new Lt,vel:new Lt,previousPos:new Lt,shapeTarget:new Lt,guideTarget:new Lt,distance:0,curl:1,_activeEpoch:0,_pooled:!1};return i._pooled=!1,i.pos.copy(t),i.vel.set(0,0,0),i.previousPos.copy(t),i.shapeTarget.copy(t),i.guideTarget.copy(t),i.distance=e,i.curl=n,i}#m(){for(let t=0;t<2;t++){const e=t===0?this.freeNodes:this._nextFreeNodes;for(let n=0;n<e.length;n++){const i=e[n];i._pooled||(i._pooled=!0,this._freeNodePool.push(i))}e.length=0}}#g(t){const e=Math.min(this.progress,this.guidewireInserted);if(e<=t.supportEnd+.5||this.freeNodes.length<2)return;const i=Math.abs(this.motionCommand)>0?g1:m1;for(let s=1;s<this.freeNodes.length;s++){const a=this.freeNodes[s].distance??t.supportEnd;if(a>e+Ki)continue;const o=ln(t.supportEnd,t.supportEnd+Ba,a),c=1-ln(e-Ba,e+Ki,a),l=this.#P(a,this.freeNodes[s].guideTarget),h=i*o*(.35+c*.65);this.freeNodes[s].pos.lerp(l,h),this.freeNodes[s].vel.multiplyScalar(1-h)}}#r(t){const e=this.sheathPath?.length||0;if(!(this.pathSamples.length<3||t<=e+this.pathSpacing*2))for(let n=0;n<h1;n++)this.#s(t,e),this.#S(t,e)}#s(t,e){const n=e+this.pathSpacing*1.5,i=1/Math.max(1e-8,this.pathSpacing*6.5),s=t-this.pathSpacing*4,a=1/Math.max(1e-8,this.pathSpacing*4);for(let o=1;o<this.pathSamples.length-1;o++){const c=this.pathSamples[o],l=Math.max(0,Math.min(1,(c.distance-n)*i)),h=Math.max(0,Math.min(1,(c.distance-s)*a)),u=l*l*(3-2*l),f=1-h*h*(3-2*h),d=u*(.35+f*.65);if(d<=.001)continue;const g=this.pathSamples[o-1].point._values,x=this.pathSamples[o+1].point._values;this.#u(c,(g[0]+x[0])*.5,(g[1]+x[1])*.5,(g[2]+x[2])*.5,u1*d)}for(let o=0;o<$f.length;o++){const c=$f[o];if(!(this.pathSamples.length<=c*2))for(let l=c;l<this.pathSamples.length-c;l++){const h=this.pathSamples[l],u=Math.max(0,Math.min(1,(h.distance-n)*i)),f=Math.max(0,Math.min(1,(h.distance-s)*a)),d=u*u*(3-2*u),g=1-f*f*(3-2*f),x=d*(.35+g*.65);if(x<=.001)continue;const m=this.pathSamples[l-c].point._values,p=this.pathSamples[l+c].point._values;this.#u(h,(m[0]+p[0])*.5,(m[1]+p[1])*.5,(m[2]+p[2])*.5,d1*x/Math.sqrt(c))}}}#S(t,e){const n=Math.cos(Zl);for(let i=1;i<this.pathSamples.length-1;i++){const s=this.pathSamples[i],a=this.#A(s.distance,t,e);if(a<=.001)continue;const o=this.pathSamples[i-1].point,c=s.point,l=this.pathSamples[i+1].point,h=c.x-o.x,u=c.y-o.y,f=c.z-o.z,d=l.x-c.x,g=l.y-c.y,x=l.z-c.z,m=Es(h,u,f),p=Es(d,g,x);if(m<1e-5||p<1e-5)continue;const _=kt((h*d+u*g+f*x)/(m*p),-1,1);if(_>=n)continue;const v=kt((Math.acos(_)-Zl)/(Math.PI-Zl),0,1);this.#u(s,(o.x+l.x)*.5,(o.y+l.y)*.5,(o.z+l.z)*.5,f1*v*a)}}#A(t,e,n){if(t<=n+this.pathSpacing)return 0;const i=ln(n+this.pathSpacing*1.5,n+this.pathSpacing*8,t),s=1-ln(e-this.pathSpacing*4,e,t);return i*(.35+s*.65)}#u(t,e,n,i,s){const a=kt(s,0,1),o=t.point._values;let c=(e-o[0])*a,l=(n-o[1])*a,h=(i-o[2])*a;const u=Es(c,l,h);if(!(u<=1e-6)){if(u>Kf){const f=Kf/u;c*=f,l*=f,h*=f}if(this.externalCollisionSolver){o[0]+=c,o[1]+=l,o[2]+=h;return}this._pathTarget.set(o[0]+c,o[1]+l,o[2]+h),t.point.copy(this.#c(this._pathTarget).point)}}#_(t,e,n,i=1,s=new Lt){if(this.type===oo)return this.#T(t,e,n,i,s);const a=Math.min(n,ao),o=Math.max(0,n-a);if(t<=o)return s.copy(e.supportTip).addScaledVector(e.tangent,t);const c=t-o,l=Math.max(0,a-Hl),h=ln(0,ao-Hl,l)*i,u=Math.min(a,Hl+l*.18);if(c<=u||h<=.001)return s.copy(e.supportTip).addScaledVector(e.tangent,t);const f=Math.max(1e-4,a-u),d=kt((c-u)/f,0,1),g=KE*Math.PI*2*h*d,x=1-d,m=Wf*h*(.72+x*.28);return s.copy(e.supportTip).addScaledVector(e.tangent,o+u+Math.sin(g)*m).addScaledVector(e.normal,-Wf*h+Math.cos(g)*m)}#T(t,e,n,i=1,s=new Lt){const a=Math.min(n,ao),o=Math.max(0,n-a);if(t<=o)return s.copy(e.supportTip).addScaledVector(e.tangent,t);const c=t-o,l=Math.min(a,QE);if(c<=l)return s.copy(e.supportTip).addScaledVector(e.tangent,t);const h=JE*kt(i,0,1);if(h<=.001)return s.copy(e.supportTip).addScaledVector(e.tangent,t);const u=this.#M(e,this._shapeNormal),f=Math.max(0,a-l),d=Math.min(t1,Math.max(1e-4,f)),g=Math.min(c-l,d),x=h*kt(g/d,0,1),m=d/h;s.copy(e.supportTip).addScaledVector(e.tangent,o+l).addScaledVector(e.tangent,Math.sin(x)*m).addScaledVector(u,(1-Math.cos(x))*m);const p=c-l-d;return p>0&&s.addScaledVector(e.tangent,Math.cos(h)*p).addScaledVector(u,Math.sin(h)*p),s}#M(t,e=new Lt){return e.copy(t.normal),e.z*=.18,e.addScaledVector(t.tangent,-e.dot(t.tangent)),e.lengthSq()<1e-6?e.copy(t.normal):e.normalize()}#y(){for(let t=1;t<this.freeNodes.length;t++){const e=this.freeNodes[t-1],n=this.freeNodes[t],i=Math.max(.5,(n.distance??0)-(e.distance??0)),s=n.pos.x-e.pos.x,a=n.pos.y-e.pos.y,o=n.pos.z-e.pos.z,c=Es(s,a,o);if(c<1e-5)continue;const l=(c-i)/c;if(t===1)n.pos.x-=s*l,n.pos.y-=a*l,n.pos.z-=o*l;else{const h=l*.5;e.pos.x+=s*h,e.pos.y+=a*h,e.pos.z+=o*h,n.pos.x-=s*h,n.pos.y-=a*h,n.pos.z-=o*h}}}#E(t){if(this.freeNodes.length>1){const e=Math.max(.5,(this.freeNodes[1].distance??0)-(this.freeNodes[0].distance??0))||ys,n=this.freeNodes[1].pos;n.x+=(t.supportTip.x+t.tangent.x*e-n.x)*Wl,n.y+=(t.supportTip.y+t.tangent.y*e-n.y)*Wl,n.z+=(t.supportTip.z+t.tangent.z*e-n.z)*Wl}for(let e=2;e<this.freeNodes.length-1;e++){const n=this.freeNodes[e-1].pos,i=this.freeNodes[e+1].pos,s=this.freeNodes[e].pos;s.x+=((n.x+i.x)*.5-s.x)*Xl,s.y+=((n.y+i.y)*.5-s.y)*Xl,s.z+=((n.z+i.z)*.5-s.z)*Xl}}#w(t,e){for(let n=1;n<this.freeNodes.length;n++){const i=Math.max(0,(this.freeNodes[n].distance??0)-(this.freeNodes[0].distance??0)),s=this.#R(i,t,e,this.freeNodes[n].curl??1,this.freeNodes[n].shapeTarget),a=ln(0,Math.max(ys,e),i),o=this.type===oo?this.#p(i,e):0,c=i1*(.45+a*.55)*(1+o*1.2),l=kt(c,0,.42),h=this.freeNodes[n].pos._values,u=s._values;h[0]+=(u[0]-h[0])*l,h[1]+=(u[1]-h[1])*l,h[2]+=(u[2]-h[2])*l}}#C(t){if(this.freeNodes.length<4)return;const e=this.freeNodes[0]?.distance??0,n=Math.max(0,t-Math.min(t,ao)),i=Math.max(0,n-10),s=Math.max(1e-8,n+8-i);for(let a=1;a<this.freeNodes.length-1;a++){const o=Math.max(0,(this.freeNodes[a].distance??e)-e),c=Math.max(0,Math.min(1,(o-i)/s)),l=1-c*c*(3-2*c);if(l<=.001)continue;const h=this.freeNodes[a-1].pos._values,u=this.freeNodes[a+1].pos._values,f=this.freeNodes[a].pos._values,d=this.#x(this.freeNodes[a].distance),g=kt(o1*l*d,0,1);f[0]+=((h[0]+u[0])*.5-f[0])*g,f[1]+=((h[1]+u[1])*.5-f[1])*g,f[2]+=((h[2]+u[2])*.5-f[2])*g}for(let a=0;a<jf.length;a++){const o=jf[a];if(!(this.freeNodes.length<=o*2))for(let c=o;c<this.freeNodes.length-o;c++){const l=Math.max(0,(this.freeNodes[c].distance??e)-e),h=Math.max(0,Math.min(1,(l-i)/s)),u=1-h*h*(3-2*h);if(u<=.001)continue;const f=this.freeNodes[c-o].pos._values,d=this.freeNodes[c+o].pos._values,g=this.#x(this.freeNodes[c].distance),x=a1*u*g/Math.sqrt(o),m=kt(x,0,1),p=this.freeNodes[c].pos._values;p[0]+=((f[0]+d[0])*.5-p[0])*m,p[1]+=((f[1]+d[1])*.5-p[1])*m,p[2]+=((f[2]+d[2])*.5-p[2])*m}}}#L(t){if(this.freeNodes.length<3)return;const e=Math.cos(ql);for(let n=1;n<this.freeNodes.length-1;n++){const i=this.freeNodes[n-1].pos,s=this.freeNodes[n].pos,a=this.freeNodes[n+1].pos,o=s.x-i.x,c=s.y-i.y,l=s.z-i.z,h=a.x-s.x,u=a.y-s.y,f=a.z-s.z,d=Es(o,c,l),g=Es(h,u,f);if(d<1e-5||g<1e-5)continue;const x=kt((o*h+c*u+l*f)/(d*g),-1,1);if(x>=e)continue;const m=kt((Math.acos(x)-ql)/(Math.PI-ql),0,1),p=this.#F(this.freeNodes[n],t),v=(this.#x(this.freeNodes[n].distance)-1)/Math.max(1e-6,jl-1),S=1+(p1-1)*v,y=c1*S*m*(.28+p*.72),M=kt(y,0,1);s.x+=((i.x+a.x)*.5-s.x)*M,s.y+=((i.y+a.y)*.5-s.y)*M,s.z+=((i.z+a.z)*.5-s.z)*M}}#o(){for(let t=1;t<this.freeNodes.length;t++){const e=this.freeNodes[t],n=this.#c(e.pos);if(!n.collided)continue;e.pos.copy(n.point);const i=n.normal,s=e.vel.dot(i);s>0&&e.vel.addScaledVector(i,-s),e.vel.multiplyScalar(1-Yl)}}#f(){if(!(this.freeNodes.length<2))for(let t=1;t<this.freeNodes.length;t++){const e=this.freeNodes[t-1],n=this.freeNodes[t];for(const i of l1){const s=e.pos.clone().lerp(n.pos,i),a=this.#c(s);if(!a.collided)continue;const o=a.point.sub(s),c=o.length();if(c<=1e-6)continue;c>Zf&&o.multiplyScalar(Zf/c);const l=t===1?0:1-i,h=t===1?1:i;e.pos.addScaledVector(o,qf*l),n.pos.addScaledVector(o,qf*h),e.vel.multiplyScalar(1-Yl*l),n.vel.multiplyScalar(1-Yl*h)}}}#R(t,e,n,i=1,s=new Lt){const a=this.#_(t,e,n,i,s),o=this.#B()+t;if(this.guidewireInserted>za&&o<=this.guidewireInserted+Ki){const c=1-ln(this.guidewireInserted-Ba,this.guidewireInserted+Ki,o),l=this.#P(o,this._shapeNormal);a.lerp(l,.97*c)}return this.externalCollisionSolver?a:this.#c(a).point}#F(t,e){const n=this.freeNodes[0]?.distance??0,i=Math.max(0,(t.distance??n)-n),s=Math.max(0,e-this.#v(e));return 1-ln(Math.max(0,s-10),s+8,i)}#p(t,e){const n=Math.max(0,e-this.#v(e));return ln(n-2,n+10,t)}#v(t){const e=this.type===oo?e1:ao;return Math.min(t,e)}#x(t){if(this.guidewireInserted<=za)return jl;const e=ln(this.guidewireInserted-Ba,this.guidewireInserted+Ki,t??this.progress);return 1+(jl-1)*e}#c(t){if(this.collisionMesh)return this.#U(t,this.collisionMesh);let e=null;for(const n of this.vesselColliders){const i=n.type==="sphere"?this.#N(t,n):this.#I(t,n);if(i.inside)return{point:t.clone(),normal:i.normal,collided:!1};(!e||i.distance<e.distance)&&(e=i)}return{point:e?.point||t.clone(),normal:e?.normal||new Lt(1,0,0),collided:!!e}}#U(t,e){if(e.meshCollider?.pointContact){const l=e.meshCollider.pointContact(t,e.clearance);return{point:l.violation?l.target.clone():t.clone(),normal:l.normal?.clone?.()||new Lt(1,0,0),collided:!!l.violation}}const n=new Lt,s=e.geometry.boundsTree.closestPointToPoint(t,{point:n})?.distance??t.distanceTo(n),a=Math.max(e.clearance+ys*1.5,e.clearance*2);if(s>a)return{point:t.clone(),normal:new Lt(1,0,0),collided:!1};const o=typeof e.interiorDirection=="function"?e.interiorDirection(t,n).clone():t.clone().sub(n);return o.lengthSq()<1e-8&&o.set(1,0,0),o.normalize(),t.clone().sub(n).dot(o)>=e.clearance?{point:t.clone(),normal:o.clone().multiplyScalar(-1),collided:!1}:{point:n.clone().addScaledVector(o,e.clearance),normal:o.clone().multiplyScalar(-1),collided:!0}}#I(t,e){const n=new Lt().subVectors(t,e.start),i=kt(n.dot(e.dir),0,e.length),s=e.start.clone().addScaledVector(e.dir,i),a=new Lt().subVectors(t,s),o=a.length(),c=Math.max(.6,e.radius-Qf),l=o<=c,h=o>1e-6?a.multiplyScalar(1/o):this.#d(e.dir);if(l)return{inside:l,point:t.clone(),distance:0,normal:h};const u=s.addScaledVector(h,c);return{inside:!1,point:u,distance:t.distanceTo(u),normal:h}}#N(t,e){const n=new Lt().subVectors(t,e.center),i=n.length(),s=Math.max(.6,e.radius-Qf),a=i<=s,o=i>1e-6?n.multiplyScalar(1/i):new Lt(1,0,0);if(a)return{inside:a,point:t.clone(),distance:0,normal:o};const c=e.center.clone().addScaledVector(o,s);return{inside:!1,point:c,distance:t.distanceTo(c),normal:o}}#d(t){const e=Math.abs(t.y)<.85?new Lt(0,1,0):new Lt(1,0,0);return new Lt().crossVectors(t,e).normalize()}#e(t){if(!t?.segments)return[];const e=[],n=new Map,i=a=>`${a.x.toFixed(5)},${a.y.toFixed(5)},${a.z.toFixed(5)}`,s=(a,o)=>{const c=i(a),l=n.get(c);n.set(c,{point:a,radius:l?Math.max(l.radius,o):o})};for(const a of t.segments){const o=co(a.start),c=co(a.end),l=new Lt().subVectors(c,o),h=l.length();if(h<1e-6)continue;const u=l.multiplyScalar(1/h);e.push({type:"segment",start:o,end:c,dir:u,length:h,radius:a.radius||t.radius||10}),s(a.end,a.radius||t.radius||10),a.isSheath||s(a.start,a.radius||t.radius||10)}for(const{point:a,radius:o}of n.values())e.push({type:"sphere",center:co(a),radius:o});return e}#V(t){if(!t?.start||!t?.end)return null;const e=co(t.start),n=co(t.end),i=new Lt().subVectors(n,e),s=i.length();return s<1e-6?null:(i.multiplyScalar(1/s),{start:e,end:n,dir:i,length:s})}#B(){return this.sheathPath?Math.min(this.progress,this.sheathPath.length):0}#O(t,e=new Lt){if(!this.sheathPath)return null;const n=kt(t,0,this.sheathPath.length);return e.copy(this.sheathPath.start).addScaledVector(this.sheathPath.dir,n)}#G(t){const e=this.sheathPath?.length||0;if(t<=e+.5)return;this.pathSamples.length||this.#z(e);let n=this.#D();for(;n+this.pathSpacing<t;)n+=this.pathSpacing,this.#z(n);t>this.#D()+.5&&this.#z(t)}#z(t){const e=this.pathSamples.length;let n=this._pathSamplePool[e];return n||(n={distance:0,point:new Lt},this._pathSamplePool[e]=n),n.distance=t,this.#P(t,n.point),this.pathSamples[e]=n,n}#k(t){const e=this.sheathPath?.length||0;if(!(t<=e+.5)){this.#G(t);for(let n=0;n<this.pathSamples.length;n++){const i=this.pathSamples[n];i.distance<=e+.5||i.distance>t+.5||this.#P(i.distance,i.point)}}}#H(t){const e=this.sheathPath?.length||0,n=Math.max(t,e);for(;this.pathSamples.length>0&&this.pathSamples[this.pathSamples.length-1].distance>n;)this.pathSamples.pop();const i=this.pathSamples[this.pathSamples.length-1];i&&i.distance>t&&i.distance>e&&(i.distance=t)}#D(){const t=this.pathSamples[this.pathSamples.length-1];return Math.max(this.sheathPath?.length||0,t?t.distance:0)}#b(t,e=new Lt){const n=this.sheathPath?.length||0;if(this.sheathPath&&t<0)return e.copy(this.sheathPath.start).addScaledVector(this.sheathPath.dir,t);if(this.sheathPath&&t<=n+.5)return this.#O(t,e);if(!this.pathSamples.length){const a=this.#O(n,e);return a||this.#P(t,e)}const i=kt(t,0,this.#D());let s=this.pathSamples[0];for(let a=1;a<this.pathSamples.length;a++){const o=this.pathSamples[a];if(o.distance>=i){const c=kt((i-s.distance)/Math.max(1e-6,o.distance-s.distance),0,1);return e.copy(s.point).lerp(o.point,c)}s=o}return e.copy(s.point)}#W(t,e=this.#i()){if(!this.freeNodes.length||t<=e.supportEnd+.5)return this.#b(t);const n=kt(t,this.freeNodes[0].distance??e.supportEnd,this.progress);let i=this.freeNodes[0];for(let s=1;s<this.freeNodes.length;s++){const a=this.freeNodes[s];if((a.distance??n)>=n){const o=Math.max(1e-6,(a.distance??n)-(i.distance??n)),c=kt((n-(i.distance??n))/o,0,1);return i.pos.clone().lerp(a.pos,c)}i=a}return i.pos.clone()}#X(t,e,n=1){if(n<=0||!this.freeNodes.length||e.lengthSq()<1e-8||t<=(this.freeNodes[0].distance??0))return;let i=this.freeNodes[0];for(let a=1;a<this.freeNodes.length;a++){const o=this.freeNodes[a],c=i.distance??0,l=o.distance??c;if(t<=l+.5){const h=Math.max(1e-6,l-c),u=kt((t-c)/h,0,1),f=e.clone().multiplyScalar(-.16*n),d=a===1?0:1-u,g=a===1?1:u;i.pos.addScaledVector(f,d),o.pos.addScaledVector(f,g),i.vel.addScaledVector(f,.18*d),o.vel.addScaledVector(f,.18*g);return}i=o}const s=this.freeNodes[this.freeNodes.length-1];s.pos.addScaledVector(e,-.16*n),s.vel.addScaledVector(e,-.18*n)}#Y(t,e,n,i=new Lt){const s=this._planePreviousTangent.subVectors(e,n);if(s.lengthSq()>1e-5){s.normalize();const c=this._planeCurvature.subVectors(t,s);if(c.addScaledVector(t,-c.dot(t)),c.lengthSq()>1e-5)return i.copy(c).normalize()}const a=Math.abs(t.y)<.85,o=this._planeHelper.set(a?0:1,a?1:0,0);return i.crossVectors(t,o).cross(t).normalize()}#P(t,e=new Lt){const n=this.tailProgressRef(),i=this.wire.nodes,s=kt((t+this.guidewireLength-n)/this.segmentLength,0,i.length-1),a=Math.min(i.length-2,Math.floor(s)),o=s-a,c=i[a],l=i[a+1];return e.set(c.x+(l.x-c.x)*o,c.y+(l.y-c.y)*o,c.z+(l.z-c.z)*o)}#q(t,e){return this.segmentLength*t-this.guidewireLength+e}#Z(t){return t===oo||t==="bernstein"?oo:Xf}}class w1 extends Wc{constructor(t){super(t)}load(t,e,n,i){const s=this,a=new Rm(this.manager);a.setPath(this.path),a.setResponseType("arraybuffer"),a.setRequestHeader(this.requestHeader),a.setWithCredentials(this.withCredentials),a.load(t,function(o){try{e(s.parse(o))}catch(c){i?i(c):console.error(c),s.manager.itemError(t)}},n,i)}parse(t){function e(l){const h=new DataView(l),u=32/8*3+32/8*3*3+16/8,f=h.getUint32(80,!0);if(80+32/8+f*u===h.byteLength)return!0;const g=[115,111,108,105,100];for(let x=0;x<5;x++)if(n(g,h,x))return!1;return!0}function n(l,h,u){for(let f=0,d=l.length;f<d;f++)if(l[f]!==h.getUint8(u+f))return!1;return!0}function i(l){const h=new DataView(l),u=h.getUint32(80,!0);let f,d,g,x=!1,m,p,_,v,S;for(let D=0;D<70;D++)h.getUint32(D,!1)==1129270351&&h.getUint8(D+4)==82&&h.getUint8(D+5)==61&&(x=!0,m=new Float32Array(u*3*3),p=h.getUint8(D+6)/255,_=h.getUint8(D+7)/255,v=h.getUint8(D+8)/255,S=h.getUint8(D+9)/255);const y=84,M=50,w=new ye,T=new Float32Array(u*3*3),E=new Float32Array(u*3*3),A=new jt;for(let D=0;D<u;D++){const R=y+D*M,N=h.getFloat32(R,!0),L=h.getFloat32(R+4,!0),F=h.getFloat32(R+8,!0);if(x){const z=h.getUint16(R+48,!0);(z&32768)===0?(f=(z&31)/31,d=(z>>5&31)/31,g=(z>>10&31)/31):(f=p,d=_,g=v)}for(let z=1;z<=3;z++){const q=R+z*12,O=D*3*3+(z-1)*3;T[O]=h.getFloat32(q,!0),T[O+1]=h.getFloat32(q+4,!0),T[O+2]=h.getFloat32(q+8,!0),E[O]=N,E[O+1]=L,E[O+2]=F,x&&(A.set(f,d,g).convertSRGBToLinear(),m[O]=A.r,m[O+1]=A.g,m[O+2]=A.b)}}return w.setAttribute("position",new Ne(T,3)),w.setAttribute("normal",new Ne(E,3)),x&&(w.setAttribute("color",new Ne(m,3)),w.hasColors=!0,w.alpha=S),w}function s(l){const h=new ye,u=/solid([\s\S]*?)endsolid/g,f=/facet([\s\S]*?)endfacet/g,d=/solid\s(.+)/;let g=0;const x=/[\s]+([+-]?(?:\d*)(?:\.\d*)?(?:[eE][+-]?\d+)?)/.source,m=new RegExp("vertex"+x+x+x,"g"),p=new RegExp("normal"+x+x+x,"g"),_=[],v=[],S=[],y=new b;let M,w=0,T=0,E=0;for(;(M=u.exec(l))!==null;){T=E;const A=M[0],D=(M=d.exec(A))!==null?M[1]:"";for(S.push(D);(M=f.exec(A))!==null;){let L=0,F=0;const z=M[0];for(;(M=p.exec(z))!==null;)y.x=parseFloat(M[1]),y.y=parseFloat(M[2]),y.z=parseFloat(M[3]),F++;for(;(M=m.exec(z))!==null;)_.push(parseFloat(M[1]),parseFloat(M[2]),parseFloat(M[3])),v.push(y.x,y.y,y.z),L++,E++;F!==1&&console.error("THREE.STLLoader: Something isn't right with the normal of face number "+g),L!==3&&console.error("THREE.STLLoader: Something isn't right with the vertices of face number "+g),g++}const R=T,N=E-T;h.userData.groupNames=S,h.addGroup(R,N,w),w++}return h.setAttribute("position",new ie(_,3)),h.setAttribute("normal",new ie(v,3)),h}function a(l){return typeof l!="string"?new TextDecoder().decode(l):l}function o(l){if(typeof l=="string"){const h=new Uint8Array(l.length);for(let u=0;u<l.length;u++)h[u]=l.charCodeAt(u)&255;return h.buffer||h}else return l}const c=o(t);return e(c)?i(c):s(a(t))}}const Ym=0,A1=1,T1=2,tp=2,Kl=1.25,ep=1,Xo=32,Xc=65535,b1=Math.pow(2,-24),Jl=Symbol("SKIP_GENERATION");function C1(r){return r.index?r.index.count:r.attributes.position.count}function Wr(r){return C1(r)/3}function R1(r,t=ArrayBuffer){return r>65535?new Uint32Array(new t(4*r)):new Uint16Array(new t(2*r))}function P1(r,t){if(!r.index){const e=r.attributes.position.count,n=t.useSharedArrayBuffer?SharedArrayBuffer:ArrayBuffer,i=R1(e,n);r.setIndex(new Ne(i,1));for(let s=0;s<e;s++)i[s]=s}}function qm(r,t){const e=Wr(r),n=t||r.drawRange,i=n.start/3,s=(n.start+n.count)/3,a=Math.max(0,i),o=Math.min(e,s)-a;return[{offset:Math.floor(a),count:Math.floor(o)}]}function Zm(r,t){if(!r.groups||!r.groups.length)return qm(r,t);const e=[],n=new Set,i=t||r.drawRange,s=i.start/3,a=(i.start+i.count)/3;for(const c of r.groups){const l=c.start/3,h=(c.start+c.count)/3;n.add(Math.max(s,l)),n.add(Math.min(a,h))}const o=Array.from(n.values()).sort((c,l)=>c-l);for(let c=0;c<o.length-1;c++){const l=o[c],h=o[c+1];e.push({offset:Math.floor(l),count:Math.floor(h-l)})}return e}function L1(r,t){const e=Wr(r),n=Zm(r,t).sort((a,o)=>a.offset-o.offset),i=n[n.length-1];i.count=Math.min(e-i.offset,i.count);let s=0;return n.forEach(({count:a})=>s+=a),e!==s}function Ql(r,t,e,n,i){let s=1/0,a=1/0,o=1/0,c=-1/0,l=-1/0,h=-1/0,u=1/0,f=1/0,d=1/0,g=-1/0,x=-1/0,m=-1/0;for(let p=t*6,_=(t+e)*6;p<_;p+=6){const v=r[p+0],S=r[p+1],y=v-S,M=v+S;y<s&&(s=y),M>c&&(c=M),v<u&&(u=v),v>g&&(g=v);const w=r[p+2],T=r[p+3],E=w-T,A=w+T;E<a&&(a=E),A>l&&(l=A),w<f&&(f=w),w>x&&(x=w);const D=r[p+4],R=r[p+5],N=D-R,L=D+R;N<o&&(o=N),L>h&&(h=L),D<d&&(d=D),D>m&&(m=D)}n[0]=s,n[1]=a,n[2]=o,n[3]=c,n[4]=l,n[5]=h,i[0]=u,i[1]=f,i[2]=d,i[3]=g,i[4]=x,i[5]=m}function I1(r,t=null,e=null,n=null){const i=r.attributes.position,s=r.index?r.index.array:null,a=Wr(r),o=i.normalized;let c;t===null?(c=new Float32Array(a*6),e=0,n=a):(c=t,e=e||0,n=n||a);const l=i.array,h=i.offset||0;let u=3;i.isInterleavedBufferAttribute&&(u=i.data.stride);const f=["getX","getY","getZ"];for(let d=e;d<e+n;d++){const g=d*3,x=d*6;let m=g+0,p=g+1,_=g+2;s&&(m=s[m],p=s[p],_=s[_]),o||(m=m*u+h,p=p*u+h,_=_*u+h);for(let v=0;v<3;v++){let S,y,M;o?(S=i[f[v]](m),y=i[f[v]](p),M=i[f[v]](_)):(S=l[m+v],y=l[p+v],M=l[_+v]);let w=S;y<w&&(w=y),M<w&&(w=M);let T=S;y>T&&(T=y),M>T&&(T=M);const E=(T-w)/2,A=v*2;c[x+A+0]=w+E,c[x+A+1]=E+(Math.abs(w)+E)*b1}}return c}function Ie(r,t,e){return e.min.x=t[r],e.min.y=t[r+1],e.min.z=t[r+2],e.max.x=t[r+3],e.max.y=t[r+4],e.max.z=t[r+5],e}function np(r){let t=-1,e=-1/0;for(let n=0;n<3;n++){const i=r[n+3]-r[n];i>e&&(e=i,t=n)}return t}function ip(r,t){t.set(r)}function sp(r,t,e){let n,i;for(let s=0;s<3;s++){const a=s+3;n=r[s],i=t[s],e[s]=n<i?n:i,n=r[a],i=t[a],e[a]=n>i?n:i}}function Oa(r,t,e){for(let n=0;n<3;n++){const i=t[r+2*n],s=t[r+2*n+1],a=i-s,o=i+s;a<e[n]&&(e[n]=a),o>e[n+3]&&(e[n+3]=o)}}function lo(r){const t=r[3]-r[0],e=r[4]-r[1],n=r[5]-r[2];return 2*(t*e+e*n+n*t)}const Fi=32,D1=(r,t)=>r.candidate-t.candidate,Ji=new Array(Fi).fill().map(()=>({count:0,bounds:new Float32Array(6),rightCacheBounds:new Float32Array(6),leftCacheBounds:new Float32Array(6),candidate:0})),Ga=new Float32Array(6);function N1(r,t,e,n,i,s){let a=-1,o=0;if(s===Ym)a=np(t),a!==-1&&(o=(t[a]+t[a+3])/2);else if(s===A1)a=np(r),a!==-1&&(o=F1(e,n,i,a));else if(s===T1){const c=lo(r);let l=Kl*i;const h=n*6,u=(n+i)*6;for(let f=0;f<3;f++){const d=t[f],m=(t[f+3]-d)/Fi;if(i<Fi/4){const p=[...Ji];p.length=i;let _=0;for(let S=h;S<u;S+=6,_++){const y=p[_];y.candidate=e[S+2*f],y.count=0;const{bounds:M,leftCacheBounds:w,rightCacheBounds:T}=y;for(let E=0;E<3;E++)T[E]=1/0,T[E+3]=-1/0,w[E]=1/0,w[E+3]=-1/0,M[E]=1/0,M[E+3]=-1/0;Oa(S,e,M)}p.sort(D1);let v=i;for(let S=0;S<v;S++){const y=p[S];for(;S+1<v&&p[S+1].candidate===y.candidate;)p.splice(S+1,1),v--}for(let S=h;S<u;S+=6){const y=e[S+2*f];for(let M=0;M<v;M++){const w=p[M];y>=w.candidate?Oa(S,e,w.rightCacheBounds):(Oa(S,e,w.leftCacheBounds),w.count++)}}for(let S=0;S<v;S++){const y=p[S],M=y.count,w=i-y.count,T=y.leftCacheBounds,E=y.rightCacheBounds;let A=0;M!==0&&(A=lo(T)/c);let D=0;w!==0&&(D=lo(E)/c);const R=ep+Kl*(A*M+D*w);R<l&&(a=f,l=R,o=y.candidate)}}else{for(let v=0;v<Fi;v++){const S=Ji[v];S.count=0,S.candidate=d+m+v*m;const y=S.bounds;for(let M=0;M<3;M++)y[M]=1/0,y[M+3]=-1/0}for(let v=h;v<u;v+=6){let M=~~((e[v+2*f]-d)/m);M>=Fi&&(M=Fi-1);const w=Ji[M];w.count++,Oa(v,e,w.bounds)}const p=Ji[Fi-1];ip(p.bounds,p.rightCacheBounds);for(let v=Fi-2;v>=0;v--){const S=Ji[v],y=Ji[v+1];sp(S.bounds,y.rightCacheBounds,S.rightCacheBounds)}let _=0;for(let v=0;v<Fi-1;v++){const S=Ji[v],y=S.count,M=S.bounds,T=Ji[v+1].rightCacheBounds;y!==0&&(_===0?ip(M,Ga):sp(M,Ga,Ga)),_+=y;let E=0,A=0;_!==0&&(E=lo(Ga)/c);const D=i-_;D!==0&&(A=lo(T)/c);const R=ep+Kl*(E*_+A*D);R<l&&(a=f,l=R,o=S.candidate)}}}}else console.warn(`MeshBVH: Invalid build strategy value ${s} used.`);return{axis:a,pos:o}}function F1(r,t,e,n){let i=0;for(let s=t,a=t+e;s<a;s++)i+=r[s*6+n*2];return i/e}class th{constructor(){this.boundingData=new Float32Array(6)}}function U1(r,t,e,n,i,s){let a=n,o=n+i-1;const c=s.pos,l=s.axis*2;for(;;){for(;a<=o&&e[a*6+l]<c;)a++;for(;a<=o&&e[o*6+l]>=c;)o--;if(a<o){for(let h=0;h<3;h++){let u=t[a*3+h];t[a*3+h]=t[o*3+h],t[o*3+h]=u}for(let h=0;h<6;h++){let u=e[a*6+h];e[a*6+h]=e[o*6+h],e[o*6+h]=u}a++,o--}else return a}}function z1(r,t,e,n,i,s){let a=n,o=n+i-1;const c=s.pos,l=s.axis*2;for(;;){for(;a<=o&&e[a*6+l]<c;)a++;for(;a<=o&&e[o*6+l]>=c;)o--;if(a<o){let h=r[a];r[a]=r[o],r[o]=h;for(let u=0;u<6;u++){let f=e[a*6+u];e[a*6+u]=e[o*6+u],e[o*6+u]=f}a++,o--}else return a}}function wn(r,t){return t[r+15]===65535}function In(r,t){return t[r+6]}function Hn(r,t){return t[r+14]}function Wn(r){return r+8}function Xn(r,t){return t[r+6]}function jm(r,t){return t[r+7]}let $m,No,dc,Km;const B1=Math.pow(2,32);function Vh(r){return"count"in r?1:1+Vh(r.left)+Vh(r.right)}function O1(r,t,e){return $m=new Float32Array(e),No=new Uint32Array(e),dc=new Uint16Array(e),Km=new Uint8Array(e),kh(r,t)}function kh(r,t){const e=r/4,n=r/2,i="count"in t,s=t.boundingData;for(let a=0;a<6;a++)$m[e+a]=s[a];if(i)if(t.buffer){const a=t.buffer;Km.set(new Uint8Array(a),r);for(let o=r,c=r+a.byteLength;o<c;o+=Xo){const l=o/2;wn(l,dc)||(No[o/4+6]+=e)}return r+a.byteLength}else{const a=t.offset,o=t.count;return No[e+6]=a,dc[n+14]=o,dc[n+15]=Xc,r+Xo}else{const a=t.left,o=t.right,c=t.splitAxis;let l;if(l=kh(r+Xo,a),l/4>B1)throw new Error("MeshBVH: Cannot store child pointer greater than 32 bits.");return No[e+6]=l/4,l=kh(l,o),No[e+7]=c,l}}function G1(r,t){const e=(r.index?r.index.count:r.attributes.position.count)/3,n=e>2**16,i=n?4:2,s=t?new SharedArrayBuffer(e*i):new ArrayBuffer(e*i),a=n?new Uint32Array(s):new Uint16Array(s);for(let o=0,c=a.length;o<c;o++)a[o]=o;return a}function V1(r,t,e,n,i){const{maxDepth:s,verbose:a,maxLeafTris:o,strategy:c,onProgress:l,indirect:h}=i,u=r._indirectBuffer,f=r.geometry,d=f.index?f.index.array:null,g=h?z1:U1,x=Wr(f),m=new Float32Array(6);let p=!1;const _=new th;return Ql(t,e,n,_.boundingData,m),S(_,e,n,m),_;function v(y){l&&l(y/x)}function S(y,M,w,T=null,E=0){if(!p&&E>=s&&(p=!0,a&&(console.warn(`MeshBVH: Max depth of ${s} reached when generating BVH. Consider increasing maxDepth.`),console.warn(f))),w<=o||E>=s)return v(M+w),y.offset=M,y.count=w,y;const A=N1(y.boundingData,T,t,M,w,c);if(A.axis===-1)return v(M+w),y.offset=M,y.count=w,y;const D=g(u,d,t,M,w,A);if(D===M||D===M+w)v(M+w),y.offset=M,y.count=w;else{y.splitAxis=A.axis;const R=new th,N=M,L=D-M;y.left=R,Ql(t,N,L,R.boundingData,m),S(R,N,L,m,E+1);const F=new th,z=D,q=w-L;y.right=F,Ql(t,z,q,F.boundingData,m),S(F,z,q,m,E+1)}return y}}function k1(r,t){const e=r.geometry;t.indirect&&(r._indirectBuffer=G1(e,t.useSharedArrayBuffer),L1(e,t.range)&&!t.verbose&&console.warn('MeshBVH: Provided geometry contains groups or a range that do not fully span the vertex contents while using the "indirect" option. BVH may incorrectly report intersections on unrendered portions of the geometry.')),r._indirectBuffer||P1(e,t);const n=t.useSharedArrayBuffer?SharedArrayBuffer:ArrayBuffer,i=I1(e),s=t.indirect?qm(e,t.range):Zm(e,t.range);r._roots=s.map(a=>{const o=V1(r,i,a.offset,a.count,t),c=Vh(o),l=new n(Xo*c);return O1(0,o,l),l})}class ki{constructor(){this.min=1/0,this.max=-1/0}setFromPointsField(t,e){let n=1/0,i=-1/0;for(let s=0,a=t.length;s<a;s++){const c=t[s][e];n=c<n?c:n,i=c>i?c:i}this.min=n,this.max=i}setFromPoints(t,e){let n=1/0,i=-1/0;for(let s=0,a=e.length;s<a;s++){const o=e[s],c=t.dot(o);n=c<n?c:n,i=c>i?c:i}this.min=n,this.max=i}isSeparated(t){return this.min>t.max||t.min>this.max}}ki.prototype.setFromBox=(function(){const r=new b;return function(e,n){const i=n.min,s=n.max;let a=1/0,o=-1/0;for(let c=0;c<=1;c++)for(let l=0;l<=1;l++)for(let h=0;h<=1;h++){r.x=i.x*c+s.x*(1-c),r.y=i.y*l+s.y*(1-l),r.z=i.z*h+s.z*(1-h);const u=e.dot(r);a=Math.min(u,a),o=Math.max(u,o)}this.min=a,this.max=o}})();const H1=(function(){const r=new b,t=new b,e=new b;return function(i,s,a){const o=i.start,c=r,l=s.start,h=t;e.subVectors(o,l),r.subVectors(i.end,i.start),t.subVectors(s.end,s.start);const u=e.dot(h),f=h.dot(c),d=h.dot(h),g=e.dot(c),m=c.dot(c)*d-f*f;let p,_;m!==0?p=(u*f-g*d)/m:p=0,_=(u+p*f)/d,a.x=p,a.y=_}})(),bu=(function(){const r=new St,t=new b,e=new b;return function(i,s,a,o){H1(i,s,r);let c=r.x,l=r.y;if(c>=0&&c<=1&&l>=0&&l<=1){i.at(c,a),s.at(l,o);return}else if(c>=0&&c<=1){l<0?s.at(0,o):s.at(1,o),i.closestPointToPoint(o,!0,a);return}else if(l>=0&&l<=1){c<0?i.at(0,a):i.at(1,a),s.closestPointToPoint(a,!0,o);return}else{let h;c<0?h=i.start:h=i.end;let u;l<0?u=s.start:u=s.end;const f=t,d=e;if(i.closestPointToPoint(u,!0,t),s.closestPointToPoint(h,!0,e),f.distanceToSquared(u)<=d.distanceToSquared(h)){a.copy(f),o.copy(u);return}else{a.copy(h),o.copy(d);return}}}})(),W1=(function(){const r=new b,t=new b,e=new zi,n=new Gi;return function(s,a){const{radius:o,center:c}=s,{a:l,b:h,c:u}=a;if(n.start=l,n.end=h,n.closestPointToPoint(c,!0,r).distanceTo(c)<=o||(n.start=l,n.end=u,n.closestPointToPoint(c,!0,r).distanceTo(c)<=o)||(n.start=h,n.end=u,n.closestPointToPoint(c,!0,r).distanceTo(c)<=o))return!0;const x=a.getPlane(e);if(Math.abs(x.distanceToPoint(c))<=o){const p=x.projectPoint(c,t);if(a.containsPoint(p))return!0}return!1}})(),X1=1e-15;function eh(r){return Math.abs(r)<X1}class ci extends rn{constructor(...t){super(...t),this.isExtendedTriangle=!0,this.satAxes=new Array(4).fill().map(()=>new b),this.satBounds=new Array(4).fill().map(()=>new ki),this.points=[this.a,this.b,this.c],this.sphere=new ms,this.plane=new zi,this.needsUpdate=!0}intersectsSphere(t){return W1(t,this)}update(){const t=this.a,e=this.b,n=this.c,i=this.points,s=this.satAxes,a=this.satBounds,o=s[0],c=a[0];this.getNormal(o),c.setFromPoints(o,i);const l=s[1],h=a[1];l.subVectors(t,e),h.setFromPoints(l,i);const u=s[2],f=a[2];u.subVectors(e,n),f.setFromPoints(u,i);const d=s[3],g=a[3];d.subVectors(n,t),g.setFromPoints(d,i),this.sphere.setFromPoints(this.points),this.plane.setFromNormalAndCoplanarPoint(o,t),this.needsUpdate=!1}}ci.prototype.closestPointToSegment=(function(){const r=new b,t=new b,e=new Gi;return function(i,s=null,a=null){const{start:o,end:c}=i,l=this.points;let h,u=1/0;for(let f=0;f<3;f++){const d=(f+1)%3;e.start.copy(l[f]),e.end.copy(l[d]),bu(e,i,r,t),h=r.distanceToSquared(t),h<u&&(u=h,s&&s.copy(r),a&&a.copy(t))}return this.closestPointToPoint(o,r),h=o.distanceToSquared(r),h<u&&(u=h,s&&s.copy(r),a&&a.copy(o)),this.closestPointToPoint(c,r),h=c.distanceToSquared(r),h<u&&(u=h,s&&s.copy(r),a&&a.copy(c)),Math.sqrt(u)}})();ci.prototype.intersectsTriangle=(function(){const r=new ci,t=new Array(3),e=new Array(3),n=new ki,i=new ki,s=new b,a=new b,o=new b,c=new b,l=new b,h=new Gi,u=new Gi,f=new Gi,d=new b;function g(x,m,p){const _=x.points;let v=0,S=-1;for(let y=0;y<3;y++){const{start:M,end:w}=h;M.copy(_[y]),w.copy(_[(y+1)%3]),h.delta(a);const T=eh(m.distanceToPoint(M));if(eh(m.normal.dot(a))&&T){p.copy(h),v=2;break}const E=m.intersectLine(h,d);if(!E&&T&&d.copy(M),(E||T)&&!eh(d.distanceTo(w))){if(v<=1)(v===1?p.start:p.end).copy(d),T&&(S=v);else if(v>=2){(S===1?p.start:p.end).copy(d),v=2;break}if(v++,v===2&&S===-1)break}}return v}return function(m,p=null,_=!1){this.needsUpdate&&this.update(),m.isExtendedTriangle?m.needsUpdate&&m.update():(r.copy(m),r.update(),m=r);const v=this.plane,S=m.plane;if(Math.abs(v.normal.dot(S.normal))>1-1e-10){const y=this.satBounds,M=this.satAxes;e[0]=m.a,e[1]=m.b,e[2]=m.c;for(let E=0;E<4;E++){const A=y[E],D=M[E];if(n.setFromPoints(D,e),A.isSeparated(n))return!1}const w=m.satBounds,T=m.satAxes;t[0]=this.a,t[1]=this.b,t[2]=this.c;for(let E=0;E<4;E++){const A=w[E],D=T[E];if(n.setFromPoints(D,t),A.isSeparated(n))return!1}for(let E=0;E<4;E++){const A=M[E];for(let D=0;D<4;D++){const R=T[D];if(s.crossVectors(A,R),n.setFromPoints(s,t),i.setFromPoints(s,e),n.isSeparated(i))return!1}}return p&&(_||console.warn("ExtendedTriangle.intersectsTriangle: Triangles are coplanar which does not support an output edge. Setting edge to 0, 0, 0."),p.start.set(0,0,0),p.end.set(0,0,0)),!0}else{const y=g(this,S,u);if(y===1&&m.containsPoint(u.end))return p&&(p.start.copy(u.end),p.end.copy(u.end)),!0;if(y!==2)return!1;const M=g(m,v,f);if(M===1&&this.containsPoint(f.end))return p&&(p.start.copy(f.end),p.end.copy(f.end)),!0;if(M!==2)return!1;if(u.delta(o),f.delta(c),o.dot(c)<0){let N=f.start;f.start=f.end,f.end=N}const w=u.start.dot(o),T=u.end.dot(o),E=f.start.dot(o),A=f.end.dot(o),D=T<E,R=w<A;return w!==A&&E!==T&&D===R?!1:(p&&(l.subVectors(u.start,f.start),l.dot(o)>0?p.start.copy(u.start):p.start.copy(f.start),l.subVectors(u.end,f.end),l.dot(o)<0?p.end.copy(u.end):p.end.copy(f.end)),!0)}}})();ci.prototype.distanceToPoint=(function(){const r=new b;return function(e){return this.closestPointToPoint(e,r),e.distanceTo(r)}})();ci.prototype.distanceToTriangle=(function(){const r=new b,t=new b,e=["a","b","c"],n=new Gi,i=new Gi;return function(a,o=null,c=null){const l=o||c?n:null;if(this.intersectsTriangle(a,l))return(o||c)&&(o&&l.getCenter(o),c&&l.getCenter(c)),0;let h=1/0;for(let u=0;u<3;u++){let f;const d=e[u],g=a[d];this.closestPointToPoint(g,r),f=g.distanceToSquared(r),f<h&&(h=f,o&&o.copy(r),c&&c.copy(g));const x=this[d];a.closestPointToPoint(x,r),f=x.distanceToSquared(r),f<h&&(h=f,o&&o.copy(x),c&&c.copy(r))}for(let u=0;u<3;u++){const f=e[u],d=e[(u+1)%3];n.set(this[f],this[d]);for(let g=0;g<3;g++){const x=e[g],m=e[(g+1)%3];i.set(a[x],a[m]),bu(n,i,r,t);const p=r.distanceToSquared(t);p<h&&(h=p,o&&o.copy(r),c&&c.copy(t))}}return Math.sqrt(h)}})();class Sn{constructor(t,e,n){this.isOrientedBox=!0,this.min=new b,this.max=new b,this.matrix=new se,this.invMatrix=new se,this.points=new Array(8).fill().map(()=>new b),this.satAxes=new Array(3).fill().map(()=>new b),this.satBounds=new Array(3).fill().map(()=>new ki),this.alignedSatBounds=new Array(3).fill().map(()=>new ki),this.needsUpdate=!1,t&&this.min.copy(t),e&&this.max.copy(e),n&&this.matrix.copy(n)}set(t,e,n){this.min.copy(t),this.max.copy(e),this.matrix.copy(n),this.needsUpdate=!0}copy(t){this.min.copy(t.min),this.max.copy(t.max),this.matrix.copy(t.matrix),this.needsUpdate=!0}}Sn.prototype.update=(function(){return function(){const t=this.matrix,e=this.min,n=this.max,i=this.points;for(let l=0;l<=1;l++)for(let h=0;h<=1;h++)for(let u=0;u<=1;u++){const f=1*l|2*h|4*u,d=i[f];d.x=l?n.x:e.x,d.y=h?n.y:e.y,d.z=u?n.z:e.z,d.applyMatrix4(t)}const s=this.satBounds,a=this.satAxes,o=i[0];for(let l=0;l<3;l++){const h=a[l],u=s[l],f=1<<l,d=i[f];h.subVectors(o,d),u.setFromPoints(h,i)}const c=this.alignedSatBounds;c[0].setFromPointsField(i,"x"),c[1].setFromPointsField(i,"y"),c[2].setFromPointsField(i,"z"),this.invMatrix.copy(this.matrix).invert(),this.needsUpdate=!1}})();Sn.prototype.intersectsBox=(function(){const r=new ki;return function(e){this.needsUpdate&&this.update();const n=e.min,i=e.max,s=this.satBounds,a=this.satAxes,o=this.alignedSatBounds;if(r.min=n.x,r.max=i.x,o[0].isSeparated(r)||(r.min=n.y,r.max=i.y,o[1].isSeparated(r))||(r.min=n.z,r.max=i.z,o[2].isSeparated(r)))return!1;for(let c=0;c<3;c++){const l=a[c],h=s[c];if(r.setFromBox(l,e),h.isSeparated(r))return!1}return!0}})();Sn.prototype.intersectsTriangle=(function(){const r=new ci,t=new Array(3),e=new ki,n=new ki,i=new b;return function(a){this.needsUpdate&&this.update(),a.isExtendedTriangle?a.needsUpdate&&a.update():(r.copy(a),r.update(),a=r);const o=this.satBounds,c=this.satAxes;t[0]=a.a,t[1]=a.b,t[2]=a.c;for(let f=0;f<3;f++){const d=o[f],g=c[f];if(e.setFromPoints(g,t),d.isSeparated(e))return!1}const l=a.satBounds,h=a.satAxes,u=this.points;for(let f=0;f<3;f++){const d=l[f],g=h[f];if(e.setFromPoints(g,u),d.isSeparated(e))return!1}for(let f=0;f<3;f++){const d=c[f];for(let g=0;g<4;g++){const x=h[g];if(i.crossVectors(d,x),e.setFromPoints(i,t),n.setFromPoints(i,u),e.isSeparated(n))return!1}}return!0}})();Sn.prototype.closestPointToPoint=(function(){return function(t,e){return this.needsUpdate&&this.update(),e.copy(t).applyMatrix4(this.invMatrix).clamp(this.min,this.max).applyMatrix4(this.matrix),e}})();Sn.prototype.distanceToPoint=(function(){const r=new b;return function(e){return this.closestPointToPoint(e,r),e.distanceTo(r)}})();Sn.prototype.distanceToBox=(function(){const r=["x","y","z"],t=new Array(12).fill().map(()=>new Gi),e=new Array(12).fill().map(()=>new Gi),n=new b,i=new b;return function(a,o=0,c=null,l=null){if(this.needsUpdate&&this.update(),this.intersectsBox(a))return(c||l)&&(a.getCenter(i),this.closestPointToPoint(i,n),a.closestPointToPoint(n,i),c&&c.copy(n),l&&l.copy(i)),0;const h=o*o,u=a.min,f=a.max,d=this.points;let g=1/0;for(let m=0;m<8;m++){const p=d[m];i.copy(p).clamp(u,f);const _=p.distanceToSquared(i);if(_<g&&(g=_,c&&c.copy(p),l&&l.copy(i),_<h))return Math.sqrt(_)}let x=0;for(let m=0;m<3;m++)for(let p=0;p<=1;p++)for(let _=0;_<=1;_++){const v=(m+1)%3,S=(m+2)%3,y=p<<v|_<<S,M=1<<m|p<<v|_<<S,w=d[y],T=d[M];t[x].set(w,T);const A=r[m],D=r[v],R=r[S],N=e[x],L=N.start,F=N.end;L[A]=u[A],L[D]=p?u[D]:f[D],L[R]=_?u[R]:f[D],F[A]=f[A],F[D]=p?u[D]:f[D],F[R]=_?u[R]:f[D],x++}for(let m=0;m<=1;m++)for(let p=0;p<=1;p++)for(let _=0;_<=1;_++){i.x=m?f.x:u.x,i.y=p?f.y:u.y,i.z=_?f.z:u.z,this.closestPointToPoint(i,n);const v=i.distanceToSquared(n);if(v<g&&(g=v,c&&c.copy(n),l&&l.copy(i),v<h))return Math.sqrt(v)}for(let m=0;m<12;m++){const p=t[m];for(let _=0;_<12;_++){const v=e[_];bu(p,v,n,i);const S=n.distanceToSquared(i);if(S<g&&(g=S,c&&c.copy(n),l&&l.copy(i),S<h))return Math.sqrt(S)}}return Math.sqrt(g)}})();class Cu{constructor(t){this._getNewPrimitive=t,this._primitives=[]}getPrimitive(){const t=this._primitives;return t.length===0?this._getNewPrimitive():t.pop()}releasePrimitive(t){this._primitives.push(t)}}class Y1 extends Cu{constructor(){super(()=>new ci)}}const Yn=new Y1;class q1{constructor(){this.float32Array=null,this.uint16Array=null,this.uint32Array=null;const t=[];let e=null;this.setBuffer=n=>{e&&t.push(e),e=n,this.float32Array=new Float32Array(n),this.uint16Array=new Uint16Array(n),this.uint32Array=new Uint32Array(n)},this.clearBuffer=()=>{e=null,this.float32Array=null,this.uint16Array=null,this.uint32Array=null,t.length!==0&&this.setBuffer(t.pop())}}}const Me=new q1;let ss,wr;const cr=[],Va=new Cu(()=>new en);function Z1(r,t,e,n,i,s){ss=Va.getPrimitive(),wr=Va.getPrimitive(),cr.push(ss,wr),Me.setBuffer(r._roots[t]);const a=Hh(0,r.geometry,e,n,i,s);Me.clearBuffer(),Va.releasePrimitive(ss),Va.releasePrimitive(wr),cr.pop(),cr.pop();const o=cr.length;return o>0&&(wr=cr[o-1],ss=cr[o-2]),a}function Hh(r,t,e,n,i=null,s=0,a=0){const{float32Array:o,uint16Array:c,uint32Array:l}=Me;let h=r*2;if(wn(h,c)){const f=In(r,l),d=Hn(h,c);return Ie(r,o,ss),n(f,d,!1,a,s+r,ss)}else{let A=function(R){const{uint16Array:N,uint32Array:L}=Me;let F=R*2;for(;!wn(F,N);)R=Wn(R),F=R*2;return In(R,L)},D=function(R){const{uint16Array:N,uint32Array:L}=Me;let F=R*2;for(;!wn(F,N);)R=Xn(R,L),F=R*2;return In(R,L)+Hn(F,N)};const f=Wn(r),d=Xn(r,l);let g=f,x=d,m,p,_,v;if(i&&(_=ss,v=wr,Ie(g,o,_),Ie(x,o,v),m=i(_),p=i(v),p<m)){g=d,x=f;const R=m;m=p,p=R,_=v}_||(_=ss,Ie(g,o,_));const S=wn(g*2,c),y=e(_,S,m,a+1,s+g);let M;if(y===tp){const R=A(g),L=D(g)-R;M=n(R,L,!0,a+1,s+g,_)}else M=y&&Hh(g,t,e,n,i,s,a+1);if(M)return!0;v=wr,Ie(x,o,v);const w=wn(x*2,c),T=e(v,w,p,a+1,s+x);let E;if(T===tp){const R=A(x),L=D(x)-R;E=n(R,L,!0,a+1,s+x,v)}else E=T&&Hh(x,t,e,n,i,s,a+1);return!!E}}const ho=new b,nh=new b;function j1(r,t,e={},n=0,i=1/0){const s=n*n,a=i*i;let o=1/0,c=null;if(r.shapecast({boundsTraverseOrder:h=>(ho.copy(t).clamp(h.min,h.max),ho.distanceToSquared(t)),intersectsBounds:(h,u,f)=>f<o&&f<a,intersectsTriangle:(h,u)=>{h.closestPointToPoint(t,ho);const f=t.distanceToSquared(ho);return f<o&&(nh.copy(ho),o=f,c=u),f<s}}),o===1/0)return null;const l=Math.sqrt(o);return e.point?e.point.copy(nh):e.point=nh.clone(),e.distance=l,e.faceIndex=c,e}const $1=parseInt(Bc)>=169,ws=new b,As=new b,Ts=new b,ka=new St,Ha=new St,Wa=new St,rp=new b,op=new b,ap=new b,uo=new b;function K1(r,t,e,n,i,s,a,o){let c;if(s===dn?c=r.intersectTriangle(n,e,t,!0,i):c=r.intersectTriangle(t,e,n,s!==Ke,i),c===null)return null;const l=r.origin.distanceTo(i);return l<a||l>o?null:{distance:l,point:i.clone()}}function J1(r,t,e,n,i,s,a,o,c,l,h){ws.fromBufferAttribute(t,s),As.fromBufferAttribute(t,a),Ts.fromBufferAttribute(t,o);const u=K1(r,ws,As,Ts,uo,c,l,h);if(u){const f=new b;rn.getBarycoord(uo,ws,As,Ts,f),n&&(ka.fromBufferAttribute(n,s),Ha.fromBufferAttribute(n,a),Wa.fromBufferAttribute(n,o),u.uv=rn.getInterpolation(uo,ws,As,Ts,ka,Ha,Wa,new St)),i&&(ka.fromBufferAttribute(i,s),Ha.fromBufferAttribute(i,a),Wa.fromBufferAttribute(i,o),u.uv1=rn.getInterpolation(uo,ws,As,Ts,ka,Ha,Wa,new St)),e&&(rp.fromBufferAttribute(e,s),op.fromBufferAttribute(e,a),ap.fromBufferAttribute(e,o),u.normal=rn.getInterpolation(uo,ws,As,Ts,rp,op,ap,new b),u.normal.dot(r.direction)>0&&u.normal.multiplyScalar(-1));const d={a:s,b:a,c:o,normal:new b,materialIndex:0};rn.getNormal(ws,As,Ts,d.normal),u.face=d,u.faceIndex=s,$1&&(u.barycoord=f)}return u}function Yc(r,t,e,n,i,s,a){const o=n*3;let c=o+0,l=o+1,h=o+2;const u=r.index;r.index&&(c=u.getX(c),l=u.getX(l),h=u.getX(h));const{position:f,normal:d,uv:g,uv1:x}=r.attributes,m=J1(e,f,d,g,x,c,l,h,t,s,a);return m?(m.faceIndex=n,i&&i.push(m),m):null}function Ve(r,t,e,n){const i=r.a,s=r.b,a=r.c;let o=t,c=t+1,l=t+2;e&&(o=e.getX(o),c=e.getX(c),l=e.getX(l)),i.x=n.getX(o),i.y=n.getY(o),i.z=n.getZ(o),s.x=n.getX(c),s.y=n.getY(c),s.z=n.getZ(c),a.x=n.getX(l),a.y=n.getY(l),a.z=n.getZ(l)}function Q1(r,t,e,n,i,s,a,o){const{geometry:c,_indirectBuffer:l}=r;for(let h=n,u=n+i;h<u;h++)Yc(c,t,e,h,s,a,o)}function tw(r,t,e,n,i,s,a){const{geometry:o,_indirectBuffer:c}=r;let l=1/0,h=null;for(let u=n,f=n+i;u<f;u++){let d;d=Yc(o,t,e,u,null,s,a),d&&d.distance<l&&(h=d,l=d.distance)}return h}function ew(r,t,e,n,i,s,a){const{geometry:o}=e,{index:c}=o,l=o.attributes.position;for(let h=r,u=t+r;h<u;h++){let f;if(f=h,Ve(a,f*3,c,l),a.needsUpdate=!0,n(a,f,i,s))return!0}return!1}function nw(r,t=null){t&&Array.isArray(t)&&(t=new Set(t));const e=r.geometry,n=e.index?e.index.array:null,i=e.attributes.position;let s,a,o,c,l=0;const h=r._roots;for(let f=0,d=h.length;f<d;f++)s=h[f],a=new Uint32Array(s),o=new Uint16Array(s),c=new Float32Array(s),u(0,l),l+=s.byteLength;function u(f,d,g=!1){const x=f*2;if(o[x+15]===Xc){const p=a[f+6],_=o[x+14];let v=1/0,S=1/0,y=1/0,M=-1/0,w=-1/0,T=-1/0;for(let E=3*p,A=3*(p+_);E<A;E++){let D=n[E];const R=i.getX(D),N=i.getY(D),L=i.getZ(D);R<v&&(v=R),R>M&&(M=R),N<S&&(S=N),N>w&&(w=N),L<y&&(y=L),L>T&&(T=L)}return c[f+0]!==v||c[f+1]!==S||c[f+2]!==y||c[f+3]!==M||c[f+4]!==w||c[f+5]!==T?(c[f+0]=v,c[f+1]=S,c[f+2]=y,c[f+3]=M,c[f+4]=w,c[f+5]=T,!0):!1}else{const p=f+8,_=a[f+6],v=p+d,S=_+d;let y=g,M=!1,w=!1;t?y||(M=t.has(v),w=t.has(S),y=!M&&!w):(M=!0,w=!0);const T=y||M,E=y||w;let A=!1;T&&(A=u(p,d,y));let D=!1;E&&(D=u(_,d,y));const R=A||D;if(R)for(let N=0;N<3;N++){const L=p+N,F=_+N,z=c[L],q=c[L+3],O=c[F],Y=c[F+3];c[f+N]=z<O?z:O,c[f+N+3]=q>Y?q:Y}return R}}}function fs(r,t,e,n,i){let s,a,o,c,l,h;const u=1/e.direction.x,f=1/e.direction.y,d=1/e.direction.z,g=e.origin.x,x=e.origin.y,m=e.origin.z;let p=t[r],_=t[r+3],v=t[r+1],S=t[r+3+1],y=t[r+2],M=t[r+3+2];return u>=0?(s=(p-g)*u,a=(_-g)*u):(s=(_-g)*u,a=(p-g)*u),f>=0?(o=(v-x)*f,c=(S-x)*f):(o=(S-x)*f,c=(v-x)*f),s>c||o>a||((o>s||isNaN(s))&&(s=o),(c<a||isNaN(a))&&(a=c),d>=0?(l=(y-m)*d,h=(M-m)*d):(l=(M-m)*d,h=(y-m)*d),s>h||l>a)?!1:((l>s||s!==s)&&(s=l),(h<a||a!==a)&&(a=h),s<=i&&a>=n)}function iw(r,t,e,n,i,s,a,o){const{geometry:c,_indirectBuffer:l}=r;for(let h=n,u=n+i;h<u;h++){let f=l?l[h]:h;Yc(c,t,e,f,s,a,o)}}function sw(r,t,e,n,i,s,a){const{geometry:o,_indirectBuffer:c}=r;let l=1/0,h=null;for(let u=n,f=n+i;u<f;u++){let d;d=Yc(o,t,e,c?c[u]:u,null,s,a),d&&d.distance<l&&(h=d,l=d.distance)}return h}function rw(r,t,e,n,i,s,a){const{geometry:o}=e,{index:c}=o,l=o.attributes.position;for(let h=r,u=t+r;h<u;h++){let f;if(f=e.resolveTriangleIndex(h),Ve(a,f*3,c,l),a.needsUpdate=!0,n(a,f,i,s))return!0}return!1}function ow(r,t,e,n,i,s,a){Me.setBuffer(r._roots[t]),Wh(0,r,e,n,i,s,a),Me.clearBuffer()}function Wh(r,t,e,n,i,s,a){const{float32Array:o,uint16Array:c,uint32Array:l}=Me,h=r*2;if(wn(h,c)){const f=In(r,l),d=Hn(h,c);Q1(t,e,n,f,d,i,s,a)}else{const f=Wn(r);fs(f,o,n,s,a)&&Wh(f,t,e,n,i,s,a);const d=Xn(r,l);fs(d,o,n,s,a)&&Wh(d,t,e,n,i,s,a)}}const aw=["x","y","z"];function cw(r,t,e,n,i,s){Me.setBuffer(r._roots[t]);const a=Xh(0,r,e,n,i,s);return Me.clearBuffer(),a}function Xh(r,t,e,n,i,s){const{float32Array:a,uint16Array:o,uint32Array:c}=Me;let l=r*2;if(wn(l,o)){const u=In(r,c),f=Hn(l,o);return tw(t,e,n,u,f,i,s)}else{const u=jm(r,c),f=aw[u],g=n.direction[f]>=0;let x,m;g?(x=Wn(r),m=Xn(r,c)):(x=Xn(r,c),m=Wn(r));const _=fs(x,a,n,i,s)?Xh(x,t,e,n,i,s):null;if(_){const y=_.point[f];if(g?y<=a[m+u]:y>=a[m+u+3])return _}const S=fs(m,a,n,i,s)?Xh(m,t,e,n,i,s):null;return _&&S?_.distance<=S.distance?_:S:_||S||null}}const Xa=new en,lr=new ci,hr=new ci,fo=new se,cp=new Sn,Ya=new Sn;function lw(r,t,e,n){Me.setBuffer(r._roots[t]);const i=Yh(0,r,e,n);return Me.clearBuffer(),i}function Yh(r,t,e,n,i=null){const{float32Array:s,uint16Array:a,uint32Array:o}=Me;let c=r*2;if(i===null&&(e.boundingBox||e.computeBoundingBox(),cp.set(e.boundingBox.min,e.boundingBox.max,n),i=cp),wn(c,a)){const h=t.geometry,u=h.index,f=h.attributes.position,d=e.index,g=e.attributes.position,x=In(r,o),m=Hn(c,a);if(fo.copy(n).invert(),e.boundsTree)return Ie(r,s,Ya),Ya.matrix.copy(fo),Ya.needsUpdate=!0,e.boundsTree.shapecast({intersectsBounds:_=>Ya.intersectsBox(_),intersectsTriangle:_=>{_.a.applyMatrix4(n),_.b.applyMatrix4(n),_.c.applyMatrix4(n),_.needsUpdate=!0;for(let v=x*3,S=(m+x)*3;v<S;v+=3)if(Ve(hr,v,u,f),hr.needsUpdate=!0,_.intersectsTriangle(hr))return!0;return!1}});for(let p=x*3,_=(m+x)*3;p<_;p+=3){Ve(lr,p,u,f),lr.a.applyMatrix4(fo),lr.b.applyMatrix4(fo),lr.c.applyMatrix4(fo),lr.needsUpdate=!0;for(let v=0,S=d.count;v<S;v+=3)if(Ve(hr,v,d,g),hr.needsUpdate=!0,lr.intersectsTriangle(hr))return!0}}else{const h=r+8,u=o[r+6];return Ie(h,s,Xa),!!(i.intersectsBox(Xa)&&Yh(h,t,e,n,i)||(Ie(u,s,Xa),i.intersectsBox(Xa)&&Yh(u,t,e,n,i)))}}const qa=new se,ih=new Sn,po=new Sn,hw=new b,uw=new b,dw=new b,fw=new b;function pw(r,t,e,n={},i={},s=0,a=1/0){t.boundingBox||t.computeBoundingBox(),ih.set(t.boundingBox.min,t.boundingBox.max,e),ih.needsUpdate=!0;const o=r.geometry,c=o.attributes.position,l=o.index,h=t.attributes.position,u=t.index,f=Yn.getPrimitive(),d=Yn.getPrimitive();let g=hw,x=uw,m=null,p=null;i&&(m=dw,p=fw);let _=1/0,v=null,S=null;return qa.copy(e).invert(),po.matrix.copy(qa),r.shapecast({boundsTraverseOrder:y=>ih.distanceToBox(y),intersectsBounds:(y,M,w)=>w<_&&w<a?(M&&(po.min.copy(y.min),po.max.copy(y.max),po.needsUpdate=!0),!0):!1,intersectsRange:(y,M)=>{if(t.boundsTree)return t.boundsTree.shapecast({boundsTraverseOrder:T=>po.distanceToBox(T),intersectsBounds:(T,E,A)=>A<_&&A<a,intersectsRange:(T,E)=>{for(let A=T,D=T+E;A<D;A++){Ve(d,3*A,u,h),d.a.applyMatrix4(e),d.b.applyMatrix4(e),d.c.applyMatrix4(e),d.needsUpdate=!0;for(let R=y,N=y+M;R<N;R++){Ve(f,3*R,l,c),f.needsUpdate=!0;const L=f.distanceToTriangle(d,g,m);if(L<_&&(x.copy(g),p&&p.copy(m),_=L,v=R,S=A),L<s)return!0}}}});{const w=Wr(t);for(let T=0,E=w;T<E;T++){Ve(d,3*T,u,h),d.a.applyMatrix4(e),d.b.applyMatrix4(e),d.c.applyMatrix4(e),d.needsUpdate=!0;for(let A=y,D=y+M;A<D;A++){Ve(f,3*A,l,c),f.needsUpdate=!0;const R=f.distanceToTriangle(d,g,m);if(R<_&&(x.copy(g),p&&p.copy(m),_=R,v=A,S=T),R<s)return!0}}}}}),Yn.releasePrimitive(f),Yn.releasePrimitive(d),_===1/0?null:(n.point?n.point.copy(x):n.point=x.clone(),n.distance=_,n.faceIndex=v,i&&(i.point?i.point.copy(p):i.point=p.clone(),i.point.applyMatrix4(qa),x.applyMatrix4(qa),i.distance=x.sub(i.point).length(),i.faceIndex=S),n)}function mw(r,t=null){t&&Array.isArray(t)&&(t=new Set(t));const e=r.geometry,n=e.index?e.index.array:null,i=e.attributes.position;let s,a,o,c,l=0;const h=r._roots;for(let f=0,d=h.length;f<d;f++)s=h[f],a=new Uint32Array(s),o=new Uint16Array(s),c=new Float32Array(s),u(0,l),l+=s.byteLength;function u(f,d,g=!1){const x=f*2;if(o[x+15]===Xc){const p=a[f+6],_=o[x+14];let v=1/0,S=1/0,y=1/0,M=-1/0,w=-1/0,T=-1/0;for(let E=p,A=p+_;E<A;E++){const D=3*r.resolveTriangleIndex(E);for(let R=0;R<3;R++){let N=D+R;N=n?n[N]:N;const L=i.getX(N),F=i.getY(N),z=i.getZ(N);L<v&&(v=L),L>M&&(M=L),F<S&&(S=F),F>w&&(w=F),z<y&&(y=z),z>T&&(T=z)}}return c[f+0]!==v||c[f+1]!==S||c[f+2]!==y||c[f+3]!==M||c[f+4]!==w||c[f+5]!==T?(c[f+0]=v,c[f+1]=S,c[f+2]=y,c[f+3]=M,c[f+4]=w,c[f+5]=T,!0):!1}else{const p=f+8,_=a[f+6],v=p+d,S=_+d;let y=g,M=!1,w=!1;t?y||(M=t.has(v),w=t.has(S),y=!M&&!w):(M=!0,w=!0);const T=y||M,E=y||w;let A=!1;T&&(A=u(p,d,y));let D=!1;E&&(D=u(_,d,y));const R=A||D;if(R)for(let N=0;N<3;N++){const L=p+N,F=_+N,z=c[L],q=c[L+3],O=c[F],Y=c[F+3];c[f+N]=z<O?z:O,c[f+N+3]=q>Y?q:Y}return R}}}function gw(r,t,e,n,i,s,a){Me.setBuffer(r._roots[t]),qh(0,r,e,n,i,s,a),Me.clearBuffer()}function qh(r,t,e,n,i,s,a){const{float32Array:o,uint16Array:c,uint32Array:l}=Me,h=r*2;if(wn(h,c)){const f=In(r,l),d=Hn(h,c);iw(t,e,n,f,d,i,s,a)}else{const f=Wn(r);fs(f,o,n,s,a)&&qh(f,t,e,n,i,s,a);const d=Xn(r,l);fs(d,o,n,s,a)&&qh(d,t,e,n,i,s,a)}}const xw=["x","y","z"];function _w(r,t,e,n,i,s){Me.setBuffer(r._roots[t]);const a=Zh(0,r,e,n,i,s);return Me.clearBuffer(),a}function Zh(r,t,e,n,i,s){const{float32Array:a,uint16Array:o,uint32Array:c}=Me;let l=r*2;if(wn(l,o)){const u=In(r,c),f=Hn(l,o);return sw(t,e,n,u,f,i,s)}else{const u=jm(r,c),f=xw[u],g=n.direction[f]>=0;let x,m;g?(x=Wn(r),m=Xn(r,c)):(x=Xn(r,c),m=Wn(r));const _=fs(x,a,n,i,s)?Zh(x,t,e,n,i,s):null;if(_){const y=_.point[f];if(g?y<=a[m+u]:y>=a[m+u+3])return _}const S=fs(m,a,n,i,s)?Zh(m,t,e,n,i,s):null;return _&&S?_.distance<=S.distance?_:S:_||S||null}}const Za=new en,ur=new ci,dr=new ci,mo=new se,lp=new Sn,ja=new Sn;function vw(r,t,e,n){Me.setBuffer(r._roots[t]);const i=jh(0,r,e,n);return Me.clearBuffer(),i}function jh(r,t,e,n,i=null){const{float32Array:s,uint16Array:a,uint32Array:o}=Me;let c=r*2;if(i===null&&(e.boundingBox||e.computeBoundingBox(),lp.set(e.boundingBox.min,e.boundingBox.max,n),i=lp),wn(c,a)){const h=t.geometry,u=h.index,f=h.attributes.position,d=e.index,g=e.attributes.position,x=In(r,o),m=Hn(c,a);if(mo.copy(n).invert(),e.boundsTree)return Ie(r,s,ja),ja.matrix.copy(mo),ja.needsUpdate=!0,e.boundsTree.shapecast({intersectsBounds:_=>ja.intersectsBox(_),intersectsTriangle:_=>{_.a.applyMatrix4(n),_.b.applyMatrix4(n),_.c.applyMatrix4(n),_.needsUpdate=!0;for(let v=x,S=m+x;v<S;v++)if(Ve(dr,3*t.resolveTriangleIndex(v),u,f),dr.needsUpdate=!0,_.intersectsTriangle(dr))return!0;return!1}});for(let p=x,_=m+x;p<_;p++){const v=t.resolveTriangleIndex(p);Ve(ur,3*v,u,f),ur.a.applyMatrix4(mo),ur.b.applyMatrix4(mo),ur.c.applyMatrix4(mo),ur.needsUpdate=!0;for(let S=0,y=d.count;S<y;S+=3)if(Ve(dr,S,d,g),dr.needsUpdate=!0,ur.intersectsTriangle(dr))return!0}}else{const h=r+8,u=o[r+6];return Ie(h,s,Za),!!(i.intersectsBox(Za)&&jh(h,t,e,n,i)||(Ie(u,s,Za),i.intersectsBox(Za)&&jh(u,t,e,n,i)))}}const $a=new se,sh=new Sn,go=new Sn,Sw=new b,Mw=new b,yw=new b,Ew=new b;function ww(r,t,e,n={},i={},s=0,a=1/0){t.boundingBox||t.computeBoundingBox(),sh.set(t.boundingBox.min,t.boundingBox.max,e),sh.needsUpdate=!0;const o=r.geometry,c=o.attributes.position,l=o.index,h=t.attributes.position,u=t.index,f=Yn.getPrimitive(),d=Yn.getPrimitive();let g=Sw,x=Mw,m=null,p=null;i&&(m=yw,p=Ew);let _=1/0,v=null,S=null;return $a.copy(e).invert(),go.matrix.copy($a),r.shapecast({boundsTraverseOrder:y=>sh.distanceToBox(y),intersectsBounds:(y,M,w)=>w<_&&w<a?(M&&(go.min.copy(y.min),go.max.copy(y.max),go.needsUpdate=!0),!0):!1,intersectsRange:(y,M)=>{if(t.boundsTree){const w=t.boundsTree;return w.shapecast({boundsTraverseOrder:T=>go.distanceToBox(T),intersectsBounds:(T,E,A)=>A<_&&A<a,intersectsRange:(T,E)=>{for(let A=T,D=T+E;A<D;A++){const R=w.resolveTriangleIndex(A);Ve(d,3*R,u,h),d.a.applyMatrix4(e),d.b.applyMatrix4(e),d.c.applyMatrix4(e),d.needsUpdate=!0;for(let N=y,L=y+M;N<L;N++){const F=r.resolveTriangleIndex(N);Ve(f,3*F,l,c),f.needsUpdate=!0;const z=f.distanceToTriangle(d,g,m);if(z<_&&(x.copy(g),p&&p.copy(m),_=z,v=N,S=A),z<s)return!0}}}})}else{const w=Wr(t);for(let T=0,E=w;T<E;T++){Ve(d,3*T,u,h),d.a.applyMatrix4(e),d.b.applyMatrix4(e),d.c.applyMatrix4(e),d.needsUpdate=!0;for(let A=y,D=y+M;A<D;A++){const R=r.resolveTriangleIndex(A);Ve(f,3*R,l,c),f.needsUpdate=!0;const N=f.distanceToTriangle(d,g,m);if(N<_&&(x.copy(g),p&&p.copy(m),_=N,v=A,S=T),N<s)return!0}}}}}),Yn.releasePrimitive(f),Yn.releasePrimitive(d),_===1/0?null:(n.point?n.point.copy(x):n.point=x.clone(),n.distance=_,n.faceIndex=v,i&&(i.point?i.point.copy(p):i.point=p.clone(),i.point.applyMatrix4($a),x.applyMatrix4($a),i.distance=x.sub(i.point).length(),i.faceIndex=S),n)}function Aw(){return typeof SharedArrayBuffer<"u"}const Yo=new Me.constructor,Ac=new Me.constructor,ts=new Cu(()=>new en),fr=new en,pr=new en,rh=new en,oh=new en;let ah=!1;function Tw(r,t,e,n){if(ah)throw new Error("MeshBVH: Recursive calls to bvhcast not supported.");ah=!0;const i=r._roots,s=t._roots;let a,o=0,c=0;const l=new se().copy(e).invert();for(let h=0,u=i.length;h<u;h++){Yo.setBuffer(i[h]),c=0;const f=ts.getPrimitive();Ie(0,Yo.float32Array,f),f.applyMatrix4(l);for(let d=0,g=s.length;d<g&&(Ac.setBuffer(s[d]),a=ni(0,0,e,l,n,o,c,0,0,f),Ac.clearBuffer(),c+=s[d].length,!a);d++);if(ts.releasePrimitive(f),Yo.clearBuffer(),o+=i[h].length,a)break}return ah=!1,a}function ni(r,t,e,n,i,s=0,a=0,o=0,c=0,l=null,h=!1){let u,f;h?(u=Ac,f=Yo):(u=Yo,f=Ac);const d=u.float32Array,g=u.uint32Array,x=u.uint16Array,m=f.float32Array,p=f.uint32Array,_=f.uint16Array,v=r*2,S=t*2,y=wn(v,x),M=wn(S,_);let w=!1;if(M&&y)h?w=i(In(t,p),Hn(t*2,_),In(r,g),Hn(r*2,x),c,a+t,o,s+r):w=i(In(r,g),Hn(r*2,x),In(t,p),Hn(t*2,_),o,s+r,c,a+t);else if(M){const T=ts.getPrimitive();Ie(t,m,T),T.applyMatrix4(e);const E=Wn(r),A=Xn(r,g);Ie(E,d,fr),Ie(A,d,pr);const D=T.intersectsBox(fr),R=T.intersectsBox(pr);w=D&&ni(t,E,n,e,i,a,s,c,o+1,T,!h)||R&&ni(t,A,n,e,i,a,s,c,o+1,T,!h),ts.releasePrimitive(T)}else{const T=Wn(t),E=Xn(t,p);Ie(T,m,rh),Ie(E,m,oh);const A=l.intersectsBox(rh),D=l.intersectsBox(oh);if(A&&D)w=ni(r,T,e,n,i,s,a,o,c+1,l,h)||ni(r,E,e,n,i,s,a,o,c+1,l,h);else if(A)if(y)w=ni(r,T,e,n,i,s,a,o,c+1,l,h);else{const R=ts.getPrimitive();R.copy(rh).applyMatrix4(e);const N=Wn(r),L=Xn(r,g);Ie(N,d,fr),Ie(L,d,pr);const F=R.intersectsBox(fr),z=R.intersectsBox(pr);w=F&&ni(T,N,n,e,i,a,s,c,o+1,R,!h)||z&&ni(T,L,n,e,i,a,s,c,o+1,R,!h),ts.releasePrimitive(R)}else if(D)if(y)w=ni(r,E,e,n,i,s,a,o,c+1,l,h);else{const R=ts.getPrimitive();R.copy(oh).applyMatrix4(e);const N=Wn(r),L=Xn(r,g);Ie(N,d,fr),Ie(L,d,pr);const F=R.intersectsBox(fr),z=R.intersectsBox(pr);w=F&&ni(E,N,n,e,i,a,s,c,o+1,R,!h)||z&&ni(E,L,n,e,i,a,s,c,o+1,R,!h),ts.releasePrimitive(R)}}return w}const Ka=new Sn,hp=new en,bw={strategy:Ym,maxDepth:40,maxLeafTris:10,useSharedArrayBuffer:!1,setBoundingBox:!0,onProgress:null,indirect:!1,verbose:!0,range:null};class Ru{static serialize(t,e={}){e={cloneBuffers:!0,...e};const n=t.geometry,i=t._roots,s=t._indirectBuffer,a=n.getIndex();let o;return e.cloneBuffers?o={roots:i.map(c=>c.slice()),index:a?a.array.slice():null,indirectBuffer:s?s.slice():null}:o={roots:i,index:a?a.array:null,indirectBuffer:s},o}static deserialize(t,e,n={}){n={setIndex:!0,indirect:!!t.indirectBuffer,...n};const{index:i,roots:s,indirectBuffer:a}=t,o=new Ru(e,{...n,[Jl]:!0});if(o._roots=s,o._indirectBuffer=a||null,n.setIndex){const c=e.getIndex();if(c===null){const l=new Ne(t.index,1,!1);e.setIndex(l)}else c.array!==i&&(c.array.set(i),c.needsUpdate=!0)}return o}get indirect(){return!!this._indirectBuffer}constructor(t,e={}){if(t.isBufferGeometry){if(t.index&&t.index.isInterleavedBufferAttribute)throw new Error("MeshBVH: InterleavedBufferAttribute is not supported for the index attribute.")}else throw new Error("MeshBVH: Only BufferGeometries are supported.");if(e=Object.assign({...bw,[Jl]:!1},e),e.useSharedArrayBuffer&&!Aw())throw new Error("MeshBVH: SharedArrayBuffer is not available.");this.geometry=t,this._roots=null,this._indirectBuffer=null,e[Jl]||(k1(this,e),!t.boundingBox&&e.setBoundingBox&&(t.boundingBox=this.getBoundingBox(new en))),this.resolveTriangleIndex=e.indirect?n=>this._indirectBuffer[n]:n=>n}refit(t=null){return(this.indirect?mw:nw)(this,t)}traverse(t,e=0){const n=this._roots[e],i=new Uint32Array(n),s=new Uint16Array(n);a(0);function a(o,c=0){const l=o*2,h=s[l+15]===Xc;if(h){const u=i[o+6],f=s[l+14];t(c,h,new Float32Array(n,o*4,6),u,f)}else{const u=o+Xo/4,f=i[o+6],d=i[o+7];t(c,h,new Float32Array(n,o*4,6),d)||(a(u,c+1),a(f,c+1))}}}raycast(t,e=oi,n=0,i=1/0){const s=this._roots,a=this.geometry,o=[],c=e.isMaterial,l=Array.isArray(e),h=a.groups,u=c?e.side:e,f=this.indirect?gw:ow;for(let d=0,g=s.length;d<g;d++){const x=l?e[h[d].materialIndex].side:u,m=o.length;if(f(this,d,x,t,o,n,i),l){const p=h[d].materialIndex;for(let _=m,v=o.length;_<v;_++)o[_].face.materialIndex=p}}return o}raycastFirst(t,e=oi,n=0,i=1/0){const s=this._roots,a=this.geometry,o=e.isMaterial,c=Array.isArray(e);let l=null;const h=a.groups,u=o?e.side:e,f=this.indirect?_w:cw;for(let d=0,g=s.length;d<g;d++){const x=c?e[h[d].materialIndex].side:u,m=f(this,d,x,t,n,i);m!=null&&(l==null||m.distance<l.distance)&&(l=m,c&&(m.face.materialIndex=h[d].materialIndex))}return l}intersectsGeometry(t,e){let n=!1;const i=this._roots,s=this.indirect?vw:lw;for(let a=0,o=i.length;a<o&&(n=s(this,a,t,e),!n);a++);return n}shapecast(t){const e=Yn.getPrimitive(),n=this.indirect?rw:ew;let{boundsTraverseOrder:i,intersectsBounds:s,intersectsRange:a,intersectsTriangle:o}=t;if(a&&o){const u=a;a=(f,d,g,x,m)=>u(f,d,g,x,m)?!0:n(f,d,this,o,g,x,e)}else a||(o?a=(u,f,d,g)=>n(u,f,this,o,d,g,e):a=(u,f,d)=>d);let c=!1,l=0;const h=this._roots;for(let u=0,f=h.length;u<f;u++){const d=h[u];if(c=Z1(this,u,s,a,i,l),c)break;l+=d.byteLength}return Yn.releasePrimitive(e),c}bvhcast(t,e,n){let{intersectsRanges:i,intersectsTriangles:s}=n;const a=Yn.getPrimitive(),o=this.geometry.index,c=this.geometry.attributes.position,l=this.indirect?g=>{const x=this.resolveTriangleIndex(g);Ve(a,x*3,o,c)}:g=>{Ve(a,g*3,o,c)},h=Yn.getPrimitive(),u=t.geometry.index,f=t.geometry.attributes.position,d=t.indirect?g=>{const x=t.resolveTriangleIndex(g);Ve(h,x*3,u,f)}:g=>{Ve(h,g*3,u,f)};if(s){const g=(x,m,p,_,v,S,y,M)=>{for(let w=p,T=p+_;w<T;w++){d(w),h.a.applyMatrix4(e),h.b.applyMatrix4(e),h.c.applyMatrix4(e),h.needsUpdate=!0;for(let E=x,A=x+m;E<A;E++)if(l(E),a.needsUpdate=!0,s(a,h,E,w,v,S,y,M))return!0}return!1};if(i){const x=i;i=function(m,p,_,v,S,y,M,w){return x(m,p,_,v,S,y,M,w)?!0:g(m,p,_,v,S,y,M,w)}}else i=g}return Tw(this,t,e,i)}intersectsBox(t,e){return Ka.set(t.min,t.max,e),Ka.needsUpdate=!0,this.shapecast({intersectsBounds:n=>Ka.intersectsBox(n),intersectsTriangle:n=>Ka.intersectsTriangle(n)})}intersectsSphere(t){return this.shapecast({intersectsBounds:e=>t.intersectsBox(e),intersectsTriangle:e=>e.intersectsSphere(t)})}closestPointToGeometry(t,e,n={},i={},s=0,a=1/0){return(this.indirect?ww:pw)(this,t,e,n,i,s,a)}closestPointToPoint(t,e={},n=0,i=1/0){return j1(this,t,e,n,i)}getBoundingBox(t){return t.makeEmpty(),this._roots.forEach(n=>{Ie(0,new Float32Array(n),hp),t.union(hp)}),t}}const Cw=new URL("/assets/Aorta_plain-_gXpsVDF.stl",import.meta.url).href,Rw=new URL("/assets/Aorta_plain.collision-DFUYJYB3.bin",import.meta.url).href,Pw=1.3,Lw=40,Iw=1;function Dw(r){const t=[];for(const i of r?.segments||[])i.isSheath||t.push(i.start.y,i.end.y);const e=Math.max(...t,0)+15,n=Math.min(...t,-420)-15;return{center:new b(r?.branchPoint?.x||0,(e+n)*.5+Lw,r?.branchPoint?.z||0),length:Math.max(300,e-n)}}function Nw(r,t){r.computeBoundingBox();const e=r.boundingBox.clone(),n=e.getSize(new b),i=e.getCenter(new b),s=Dw(t),a=s.length*Pw/Math.max(1e-6,n.z);return r.translate(-i.x,-i.y,-i.z),r.rotateX(-Math.PI/2),r.scale(a,a,a),r.translate(s.center.x,s.center.y,s.center.z),r.computeBoundingBox(),{version:Iw,rotationX:-Math.PI/2,scale:a,sourceCenter:i.toArray(),sourceSize:n.toArray(),targetCenter:s.center.toArray(),targetLength:s.length}}const Fw="centerline-safe-core",Uw="sparse-sdf",zw="sparse-sdf-bvh",Pu="fallback",Bw="centerline-estimate",ei=1e-8,Fo=1<<17,Jm=Fo>>1,Ow=Jm-1,ch=200,xo=65535,up=0,dp=1,Ja=2,fp=3,Qa=4,pp=5,mp=6,gp=7,xp=8,_p=9,lh=10,hh=11,vp=12,Sp=13,tc=14,Gw=15,Mp=0,_o=1,Vw=2,yp=3,Ep=4,wp=5,ec=6,vo=0,Ap=1,So=2,Mo=3,uh=4,dh=5,fh=6,ph=7,yo=8,Eo=9,wo=10,Ao=11,Rn=0,ui=1,di=2,fi=3,To=4;function kw(r,t,e){return Math.max(t,Math.min(e,r))}function pi(r,t){r.inside=t.inside,r.violation=t.violation,r.conservative=t.conservative,r.source=t.source;const e=r.values,n=t.values;return e[0]=n[0],e[1]=n[1],e[2]=n[2],e[3]=n[3],e[4]=n[4],e[5]=n[5],e[6]=n[6],bo(r.point.values,t.point.values),bo(r.target.values,t.target.values),bo(r.closestPoint.values,t.closestPoint.values),bo(r.normal.values,t.normal.values),bo(r.inward.values,t.inward.values),r}function bo(r,t){r[0]=t[0],r[1]=t[1],r[2]=t[2]}class Co{constructor(t=0,e=0,n=0){this.values=new Float64Array([t,e,n])}get x(){return this.values[0]}set x(t){this.values[0]=t}get y(){return this.values[1]}set y(t){this.values[1]=t}get z(){return this.values[2]}set z(t){this.values[2]=t}}class Hw{constructor(){this.values=new Float64Array([-1/0,-1/0,1/0,1/0,-1,0,1]),this.inside=!1,this.violation=!1,this.conservative=!1,this.source=Pu,this.point=new Co,this.target=new Co,this.closestPoint=new Co,this.normal=new Co(1,0,0),this.inward=new Co(1,0,0)}get signedDistance(){return this.values[0]}set signedDistance(t){this.values[0]=t}get signedGap(){return this.values[1]}set signedGap(t){this.values[1]=t}get distance(){return this.values[2]}set distance(t){this.values[2]=t}get penetration(){return this.values[3]}set penetration(t){this.values[3]=t}get branchId(){return this.values[4]}set branchId(t){this.values[4]=t}get segmentT(){return this.values[5]}set segmentT(t){this.values[5]=t}get timeOfImpact(){return this.values[6]}set timeOfImpact(t){this.values[6]=t}}class Ww{constructor(){this.values=new Float64Array(12),this.found=!1,this.branchId=-1,this.signedDistance=-1/0,this.safeDistance=-1/0,this.safeBranchId=-1,this.safeInwardX=1,this.nearestDistance=1/0,this.inwardX=1}get branchId(){return this.values[0]}set branchId(t){this.values[0]=t}get t(){return this.values[1]}set t(t){this.values[1]=t}get signedDistance(){return this.values[2]}set signedDistance(t){this.values[2]=t}get safeDistance(){return this.values[3]}set safeDistance(t){this.values[3]=t}get safeBranchId(){return this.values[4]}set safeBranchId(t){this.values[4]=t}get safeInwardX(){return this.values[5]}set safeInwardX(t){this.values[5]=t}get safeInwardY(){return this.values[6]}set safeInwardY(t){this.values[6]=t}get safeInwardZ(){return this.values[7]}set safeInwardZ(t){this.values[7]=t}get nearestDistance(){return this.values[8]}set nearestDistance(t){this.values[8]=t}get inwardX(){return this.values[9]}set inwardX(t){this.values[9]=t}get inwardY(){return this.values[10]}set inwardY(t){this.values[10]=t}get inwardZ(){return this.values[11]}set inwardZ(t){this.values[11]=t}}class Xw{constructor(){this.values=new Float64Array([-1/0,1,0,0,-1]),this.conservative=!1,this.source=Pu}get signedDistance(){return this.values[0]}set signedDistance(t){this.values[0]=t}get inwardX(){return this.values[1]}set inwardX(t){this.values[1]=t}get inwardY(){return this.values[2]}set inwardY(t){this.values[2]=t}get inwardZ(){return this.values[3]}set inwardZ(t){this.values[3]=t}get branchId(){return this.values[4]}set branchId(t){this.values[4]=t}}function Ui(){return new Hw}function Yw(){return new Uint32Array(Gw)}class qw{constructor(t,{fallbackCollider:e=null,fallbackGeometry:n=null,bvhValidationDistance:i=.85,capsuleBvhValidation:s=!0}={}){const a=t instanceof ArrayBuffer?Lm(t):t;if(!a?.metadata||!a?.arrays)throw new TypeError("Decoded collision asset is required");this.metadata=a.metadata,this.arrays=a.arrays,this.fallbackCollider=e,this.fallbackGeometry=n,this.bvhValidationDistance=i,this.capsuleBvhValidationGap=s===!0?.05:Number.isFinite(s)?s:-1/0,this.packedLumenField=gE(a.metadata,a.arrays),this.centerline=a.arrays.centerlineSegments,this.centerlineStride=a.metadata.centerline.stride,this.broadPhaseOffsets=a.arrays.broadPhaseOffsets,this.broadPhaseIds=a.arrays.broadPhaseIds,this.sdfBrickKeys=a.arrays.sdfBrickKeys,this.sdfDistances=a.arrays.sdfDistances,this.sdfInsideBits=a.arrays.sdfInsideBits||null;const o=a.metadata.sdf;this.voxelSize=o.voxelSize,this.brickSize=o.brickSize,this.valuesPerBrick=this.brickSize**3,this.sdfQuantization=o.distanceQuantization??o.quantization,this.sdfOrigin=o.origin,this.sdfDimensions=o.dimensions;const c=this.sdfDimensions[0]*this.sdfDimensions[1]*this.sdfDimensions[2];if(this.sdfBrickKeys.length>=xo)throw new RangeError("Sparse SDF has too many bricks for its runtime lookup");this.sdfBrickLookup=new Uint16Array(c),this.sdfBrickLookup.fill(xo);for(let h=0;h<this.sdfBrickKeys.length;h++)this.sdfBrickLookup[this.sdfBrickKeys[h]]=h;this.signCacheKeyLow=new Int32Array(Fo),this.signCacheKeyHigh=new Int32Array(Fo),this.signCacheInside=new Uint8Array(Fo),this.signCacheValid=new Uint8Array(Fo),this.signCacheVictim=new Uint8Array(Jm);const l=a.metadata.broadPhase;this.broadPhaseOrigin=l.origin,this.broadPhaseDimensions=l.dimensions,this.broadPhaseCellSize=l.cellSize,this._sdfCornerScratch=new Float64Array(8),this._capsuleCoordinateScratch=new Float64Array(7),this._centerlineQueryScratch=new Float64Array(3),this.runtimeBytes=a.metadata.decodedBytes+this.sdfBrickLookup.byteLength+this.signCacheKeyLow.byteLength+this.signCacheKeyHigh.byteLength+this.signCacheInside.byteLength+this.signCacheValid.byteLength+this.signCacheVictim.byteLength+this._sdfCornerScratch.byteLength+this._capsuleCoordinateScratch.byteLength+this._centerlineQueryScratch.byteLength,this.stats=Yw(),this._centerlineState=new Ww,this._distanceState=new Xw,this._point={x:0,y:0,z:0},this._skipBvhValidation=!1,this._bvhPoint=new b,this._bvhClosest={point:new b,distance:1/0,faceIndex:-1},this._lumenQuery=Dm(),this._capsuleContact=Ui(),this._capsuleEndpointContact=Ui(),this._capsuleEndpointX=NaN,this._capsuleEndpointY=NaN,this._capsuleEndpointZ=NaN,this._capsuleEndpointRadius=NaN,this._sweepContact=Ui(),this._sweepProbe=Ui(),this._fallbackContact={query:{inward:{x:0,y:0,z:0},normal:{x:0,y:0,z:0},closestPoint:{x:0,y:0,z:0}},target:{x:0,y:0,z:0},closestPoint:{x:0,y:0,z:0},inward:{x:0,y:0,z:0},normal:{x:0,y:0,z:0}}}resetStats(){this.stats.fill(0)}getStats(){const t=this.stats;return{pointQueries:t[up],capsuleQueries:t[dp],capsuleSamples:t[Ja],sweepQueries:t[fp],sweepSamples:t[Qa],batchQueries:t[pp],safeCoreHits:t[mp],sdfHits:t[gp],bvhRefinements:t[xp],signRefinements:t[_p],signCacheHits:t[lh],signCacheMisses:t[hh],fallbackHits:t[vp],centerlineEstimateHits:t[Sp],resultAllocations:t[tc],runtimeBytes:this.runtimeBytes}}setFallbackCollider(t){this.fallbackCollider=t}setFallbackGeometry(t){this.fallbackGeometry=t}querySphere(t,e=0,n=null){n||this.stats[tc]++;const i=n||Ui(),s=t.x??t[0]??0,a=t.y??t[1]??0,o=t.z??t[2]??0;return this.#t(s,a,o,Math.max(0,e||0),i)}#t(t,e,n,i,s){this.stats[up]++;const a=this.#i(t,e,n,i),o=a.values,c=o[Rn],l=c-i,h=Math.max(0,-l),u=l<0,f=o[ui],d=o[di],g=o[fi],x=s.values;s.inside=c>=0,s.violation=u,s.conservative=a.conservative,x[Mp]=c,x[_o]=l,x[Vw]=Math.max(0,c),x[yp]=h,x[Ep]=o[To],x[wp]=0,x[ec]=u?0:1,s.source=a.source;const m=s.point.values;m[0]=t,m[1]=e,m[2]=n;const p=s.normal.values;p[0]=f,p[1]=d,p[2]=g;const _=s.inward.values;_[0]=f,_[1]=d,_[2]=g;const v=s.closestPoint.values;v[0]=t-f*c,v[1]=e-d*c,v[2]=n-g*c;const S=s.target.values;return S[0]=t+f*h,S[1]=e+d*h,S[2]=n+g*h,s}queryCapsule(t,e,n=0,i=null){const s=t.x??t[0]??0,a=t.y??t[1]??0,o=t.z??t[2]??0,c=e.x??e[0]??0,l=e.y??e[1]??0,h=e.z??e[2]??0;return this.queryCapsuleCoordinates(s,a,o,c,l,h,n,i)}queryCapsuleCoordinates(t,e,n,i,s,a,o=0,c=null){const l=this._capsuleCoordinateScratch;return l[0]=t,l[1]=e,l[2]=n,l[3]=i,l[4]=s,l[5]=a,l[6]=o,this.#n(c)}queryCapsuleSoA(t,e,n,i,s,a=null){const o=this._capsuleCoordinateScratch;return o[0]=t[s],o[1]=e[s],o[2]=n[s],o[3]=t[s+1],o[4]=e[s+1],o[5]=n[s+1],o[6]=Math.max(i[s],i[s+1]),this.#n(a)}#n(t){const e=this._capsuleCoordinateScratch,n=e[0],i=e[1],s=e[2],a=e[3],o=e[4],c=e[5],l=e[6];t||this.stats[tc]++;const h=t||Ui(),u=Math.max(0,l||0),f=a-n,d=o-i,g=c-s,x=Math.sqrt(f*f+d*d+g*g),m=Math.max(this.voxelSize*4,Math.max(.5,u)),p=Math.max(1,Math.ceil(x/m));let _=1/0,v=0,S=0;this.stats[dp]++,this._skipBvhValidation=!0;const y=n===this._capsuleEndpointX&&i===this._capsuleEndpointY&&s===this._capsuleEndpointZ&&u===this._capsuleEndpointRadius;let M;y?M=pi(this._capsuleContact,this._capsuleEndpointContact):(M=this.#t(n,i,s,u,this._capsuleContact),this.stats[Ja]++),_=M.values[_o];const w=_,T=M.inward.values[0],E=M.inward.values[1],A=M.inward.values[2];let D=w,R=1;pi(h,M),x>ei&&(M=this.#t(a,o,c,u,this._capsuleContact),this.stats[Ja]++,pi(this._capsuleEndpointContact,M),this._capsuleEndpointX=a,this._capsuleEndpointY=o,this._capsuleEndpointZ=c,this._capsuleEndpointRadius=u,D=M.values[_o],R=T*M.inward.values[0]+E*M.inward.values[1]+A*M.inward.values[2],D<_&&(_=D,v=1,S=p,pi(h,M))),x<=ei&&(pi(this._capsuleEndpointContact,M),this._capsuleEndpointX=a,this._capsuleEndpointY=o,this._capsuleEndpointZ=c,this._capsuleEndpointRadius=u);const N=p>1&&(Math.min(w,D)<=this.voxelSize||R<.85);for(let L=1;N&&L<p;L++){const F=L/p;M=this.#t(n+f*F,i+d*F,s+g*F,u,this._capsuleContact),this.stats[Ja]++;const z=M.values[_o];z<_&&(_=z,v=F,S=L,pi(h,M))}return this._skipBvhValidation=!1,_<=this.capsuleBvhValidationGap&&(v=S/p,pi(h,this.#t(n+f*v,i+d*v,s+g*v,u,this._capsuleContact))),h.values[wp]=v,h}sweepSphere(t,e,n=0,i=null){i||this.stats[tc]++;const s=i||Ui(),a=t.x??t[0]??0,o=t.y??t[1]??0,c=t.z??t[2]??0,l=e.x??e[0]??0,h=e.y??e[1]??0,u=e.z??e[2]??0,f=l-a,d=h-o,g=u-c,x=Math.sqrt(f*f+d*d+g*g),m=Math.max(this.voxelSize*.5,Math.max(.1,n*.5)),p=Math.max(1,Math.ceil(x/m));this.stats[fp]++,this._point.x=a,this._point.y=o,this._point.z=c;let _=this.#t(this._point.x,this._point.y,this._point.z,Math.max(0,n||0),this._sweepContact);if(this.stats[Qa]++,_.violation)return pi(s,_),s.values[ec]=0,s;for(let v=1;v<=p;v++){const S=v/p;this._point.x=a+f*S,this._point.y=o+d*S,this._point.z=c+g*S;const y=this.#t(this._point.x,this._point.y,this._point.z,Math.max(0,n||0),this._sweepProbe);if(this.stats[Qa]++,!y.violation){const T=_;_=y,this._sweepProbe=T;continue}let M=(v-1)/p,w=S;for(let T=0;T<7;T++){const E=(M+w)*.5;this._point.x=a+f*E,this._point.y=o+d*E,this._point.z=c+g*E;const A=this.#t(this._point.x,this._point.y,this._point.z,Math.max(0,n||0),this._sweepContact);this.stats[Qa]++,A.violation?w=E:M=E}return this._point.x=a+f*w,this._point.y=o+d*w,this._point.z=c+g*w,pi(s,this.#t(this._point.x,this._point.y,this._point.z,Math.max(0,n||0),this._sweepContact)),s.values[ec]=w,s}return this._point.x=l,this._point.y=h,this._point.z=u,pi(s,this.#t(this._point.x,this._point.y,this._point.z,Math.max(0,n||0),this._sweepContact)),s.values[ec]=1,s}queryBatch(t,e,n,i){if(!i||i.signedGaps.length<n)throw new RangeError("Preallocated batch contact output is too small");this.stats[pp]++;const s=this._capsuleContact;for(let a=0;a<n;a++){const o=a*3;this._point.x=t[o],this._point.y=t[o+1],this._point.z=t[o+2],this.#t(this._point.x,this._point.y,this._point.z,Math.max(0,e[a]||0),s),i.signedDistances[a]=s.values[Mp],i.signedGaps[a]=s.values[_o],i.penetrations[a]=s.values[yp],i.normals[o]=s.normal.values[0],i.normals[o+1]=s.normal.values[1],i.normals[o+2]=s.normal.values[2],i.targets[o]=s.target.values[0],i.targets[o+1]=s.target.values[1],i.targets[o+2]=s.target.values[2],i.branchIds[a]=s.values[Ep],i.violations[a]=s.violation?1:0}return i.count=n,i}#i(t,e,n,i){const s=this.#l(t,e,n),a=s.values,o=this._distanceState,c=o.values;if(s.found&&a[Mo]>i+this.voxelSize*.25)return c[Rn]=a[Mo],c[ui]=a[dh],c[di]=a[fh],c[fi]=a[ph],c[To]=a[uh],o.conservative=!0,o.source=Fw,this.stats[mp]++,o;if(this.#h(t,e,n,o))return Math.sqrt(c[ui]*c[ui]+c[di]*c[di]+c[fi]*c[fi])<ei&&s.found&&(c[ui]=a[Eo],c[di]=a[wo],c[fi]=a[Ao]),c[To]=a[vo],o.conservative=!1,o.source=Uw,this.stats[gp]++,this._skipBvhValidation||this.#g(t,e,n,i,o),o;if(this.fallbackCollider?.pointContact){this._point.x=t,this._point.y=e,this._point.z=n;const l=this.fallbackCollider.pointContact(this._point,0,this._fallbackContact);if(Number.isFinite(l?.signedDistance)){const h=l.inward||l.normal,u=h?.x||0,f=h?.y||0,d=h?.z||0,g=Math.sqrt(u*u+f*f+d*d);if(c[Rn]=l.signedDistance,g>ei){const x=l.inward?1:-1;c[ui]=h.x/g*x,c[di]=h.y/g*x,c[fi]=h.z/g*x}else c[ui]=a[Eo],c[di]=a[wo],c[fi]=a[Ao];return c[To]=a[vo],o.conservative=!1,o.source=Pu,this.stats[vp]++,o}}return c[Rn]=a[So],c[ui]=a[Eo],c[di]=a[wo],c[fi]=a[Ao],c[To]=a[vo],o.conservative=!0,o.source=Bw,this.stats[Sp]++,o}#l(t,e,n){const i=this._centerlineQueryScratch;i[0]=t,i[1]=e,i[2]=n;const s=this._centerlineState,a=s.values;s.found=!1,a[vo]=-1,a[Ap]=0,a[So]=-1/0,a[Mo]=-1/0,a[uh]=-1,a[dh]=1,a[fh]=0,a[ph]=0,a[yo]=1/0,a[Eo]=1,a[wo]=0,a[Ao]=0;const o=Math.floor((t-this.broadPhaseOrigin[0])/this.broadPhaseCellSize),c=Math.floor((e-this.broadPhaseOrigin[1])/this.broadPhaseCellSize),l=Math.floor((n-this.broadPhaseOrigin[2])/this.broadPhaseCellSize);if(o>=0&&c>=0&&l>=0&&o<this.broadPhaseDimensions[0]&&c<this.broadPhaseDimensions[1]&&l<this.broadPhaseDimensions[2]){const h=o+this.broadPhaseDimensions[0]*(c+this.broadPhaseDimensions[1]*l),u=this.broadPhaseOffsets[h],f=this.broadPhaseOffsets[h+1];for(let d=u;d<f;d++)this.#a(this.broadPhaseIds[d],s)}if(!s.found){const h=this.centerline.length/this.centerlineStride;for(let u=0;u<h;u++)this.#a(u,s)}return s}#a(t,e){const n=this._centerlineQueryScratch,i=n[0],s=n[1],a=n[2],o=e.values,c=t*this.centerlineStride,l=this.centerline[c],h=this.centerline[c+1],u=this.centerline[c+2],f=this.centerline[c+3]-l,d=this.centerline[c+4]-h,g=this.centerline[c+5]-u,x=f*f+d*d+g*g,m=kw(((i-l)*f+(s-h)*d+(a-u)*g)/Math.max(ei,x),0,1),p=l+f*m,_=h+d*m,v=u+g*m,S=p-i,y=_-s,M=v-a,w=Math.sqrt(S*S+y*y+M*M),T=this.centerline[c+6]*(1-m)+this.centerline[c+7]*m,E=this.centerline[c+8],A=T-w,D=E-w;o[yo]=Math.min(o[yo],w);let R,N,L;if(w>ei)R=S/w,N=y/w,L=M/w;else{const F=Math.sqrt(x),z=F>ei?f/F:0,q=F>ei?d/F:1,O=F>ei?g/F:0,Y=Math.abs(q)<.85?0:1,K=Math.abs(q)<.85?1:0,J=-O*K,it=O*Y,X=z*K-q*Y,j=Math.sqrt(J*J+it*it+X*X)||1;R=J/j,N=it/j,L=X/j}D>o[Mo]&&(o[Mo]=D,o[uh]=t,o[dh]=R,o[fh]=N,o[ph]=L),!(e.found&&A<=o[So])&&(e.found=!0,o[vo]=t,o[Ap]=m,o[So]=A,o[Eo]=R,o[wo]=N,o[Ao]=L)}#h(t,e,n,i){const s=i.values,a=this._centerlineState.values,o=this._sdfCornerScratch,c=(t-this.sdfOrigin[0])/this.voxelSize,l=(e-this.sdfOrigin[1])/this.voxelSize,h=(n-this.sdfOrigin[2])/this.voxelSize,u=Math.floor(c),f=Math.floor(l),d=Math.floor(h),g=c-u,x=l-f,m=h-d,p=this.brickSize,_=Math.floor(u/p),v=Math.floor(f/p),S=Math.floor(d/p),y=u-_*p,M=f-v*p,w=d-S*p,T=y>=0&&M>=0&&w>=0&&y+1<p&&M+1<p&&w+1<p&&_>=0&&v>=0&&S>=0&&_<this.sdfDimensions[0]&&v<this.sdfDimensions[1]&&S<this.sdfDimensions[2];let E=-1;if(T){const ot=_+this.sdfDimensions[0]*(v+this.sdfDimensions[1]*S),ut=this.sdfBrickLookup[ot];if(ut!==xo){const rt=p,ht=p*p;E=ut*this.valuesPerBrick+y+rt*M+ht*w;const dt=this.sdfDistances,bt=this.sdfQuantization;o[0]=dt[E]*bt,o[1]=dt[E+1]*bt,o[2]=dt[E+rt]*bt,o[3]=dt[E+rt+1]*bt,o[4]=dt[E+ht]*bt,o[5]=dt[E+ht+1]*bt,o[6]=dt[E+ht+rt]*bt,o[7]=dt[E+ht+rt+1]*bt}}if(E<0&&(this.#r(o,0,u,f,d),this.#r(o,1,u+1,f,d),this.#r(o,2,u,f+1,d),this.#r(o,3,u+1,f+1,d),this.#r(o,4,u,f,d+1),this.#r(o,5,u+1,f,d+1),this.#r(o,6,u,f+1,d+1),this.#r(o,7,u+1,f+1,d+1)),!Number.isFinite(o[0])||!Number.isFinite(o[1])||!Number.isFinite(o[2])||!Number.isFinite(o[3])||!Number.isFinite(o[4])||!Number.isFinite(o[5])||!Number.isFinite(o[6])||!Number.isFinite(o[7]))return!1;const A=o[0]+(o[1]-o[0])*g,D=o[2]+(o[3]-o[2])*g,R=o[4]+(o[5]-o[4])*g,N=o[6]+(o[7]-o[6])*g,L=A+(D-A)*x,F=R+(N-R)*x,z=L+(F-L)*m;let q;if(this.sdfInsideBits){let ot,ut,rt,ht,dt,bt,V,re;if(E>=0){const H=p,Q=p*p,$=this.sdfInsideBits,tt=E+1,Et=E+H,at=Et+1,xt=E+Q,Pt=xt+1,Ht=xt+H,et=Ht+1;ot=($[E>>3]&1<<(E&7))!==0?1:0,ut=($[tt>>3]&1<<(tt&7))!==0?1:0,rt=($[Et>>3]&1<<(Et&7))!==0?1:0,ht=($[at>>3]&1<<(at&7))!==0?1:0,dt=($[xt>>3]&1<<(xt&7))!==0?1:0,bt=($[Pt>>3]&1<<(Pt&7))!==0?1:0,V=($[Ht>>3]&1<<(Ht&7))!==0?1:0,re=($[et>>3]&1<<(et&7))!==0?1:0}else ot=this.#s(u,f,d),ut=this.#s(u+1,f,d),rt=this.#s(u,f+1,d),ht=this.#s(u+1,f+1,d),dt=this.#s(u,f,d+1),bt=this.#s(u+1,f,d+1),V=this.#s(u,f+1,d+1),re=this.#s(u+1,f+1,d+1);const Tt=ot+(ut-ot)*g,It=rt+(ht-rt)*g,vt=dt+(bt-dt)*g,Jt=V+(re-V)*g,Rt=Tt+(It-Tt)*x,I=vt+(Jt-vt)*x,C=ot+ut+rt+ht+dt+bt+V+re;C>0&&C<8&&this.packedLumenField?(q=this.#m(t,e,n)?1:-1,this.stats[_p]++):q=Rt+(I-Rt)*m>=.5?1:-1,s[Rn]=z*q}else q=(this.packedLumenField?this.packedLumenField.queryCoordinates(t,e,n,this._lumenQuery).signedDistance:a[So])>=0?1:-1,s[Rn]=z*q;const O=(o[1]-o[0])*(1-x)+(o[3]-o[2])*x,Y=(o[5]-o[4])*(1-x)+(o[7]-o[6])*x,K=(o[2]-o[0])*(1-g)+(o[3]-o[1])*g,J=(o[6]-o[4])*(1-g)+(o[7]-o[5])*g;let it=(O*(1-m)+Y*m)/this.voxelSize,X=(K*(1-m)+J*m)/this.voxelSize,j=(F-L)/this.voxelSize;const nt=Math.sqrt(it*it+X*X+j*j);return nt>ei&&(it/=nt,X/=nt,j/=nt),s[Rn]<0&&(a[yo]<=.001||a[yo]+.2<-s[Rn])&&(s[Rn]=-s[Rn],q=-q),s[ui]=it*q,s[di]=X*q,s[fi]=j*q,!0}#m(t,e,n){const i=Math.round((t-this.sdfOrigin[0])*ch),s=Math.round((e-this.sdfOrigin[1])*ch),a=Math.round((n-this.sdfOrigin[2])*ch);if(i<0||i>65535||s<0||s>131071||a<0||a>65535)return this.stats[hh]++,this.packedLumenField.isInsideCoordinates(t,e,n);const o=i&65535|(s&65535)<<16,c=s>>>16|a<<1,l=(Math.imul(i,73856093)^Math.imul(s,19349663)^Math.imul(a,83492791))&Ow,h=l<<1,u=h+1;if(this.signCacheValid[h]&&this.signCacheKeyLow[h]===o&&this.signCacheKeyHigh[h]===c)return this.signCacheVictim[l]=1,this.stats[lh]++,this.signCacheInside[h]!==0;if(this.signCacheValid[u]&&this.signCacheKeyLow[u]===o&&this.signCacheKeyHigh[u]===c)return this.signCacheVictim[l]=0,this.stats[lh]++,this.signCacheInside[u]!==0;const f=this.packedLumenField.isInsideCoordinates(t,e,n);let d;return this.signCacheValid[h]?this.signCacheValid[u]?d=h+this.signCacheVictim[l]:d=u:d=h,this.signCacheKeyLow[d]=o,this.signCacheKeyHigh[d]=c,this.signCacheInside[d]=f?1:0,this.signCacheValid[d]=1,this.signCacheVictim[l]=d===h?1:0,this.stats[hh]++,f}#g(t,e,n,i,s){const a=this.fallbackGeometry?.boundsTree,o=s.values,c=o[Rn]-i,l=i>0?Math.min(this.bvhValidationDistance,.25):this.bvhValidationDistance;if(!a||Math.abs(c)>l&&(i<=0||c>=-.2))return!1;this._bvhPoint.set(t,e,n),this._bvhClosest.distance=1/0;const u=a.closestPointToPoint(this._bvhPoint,this._bvhClosest)?.distance??this._bvhPoint.distanceTo(this._bvhClosest.point);if(!Number.isFinite(u))return!1;const f=o[Rn]>=0?1:-1;return o[Rn]=u*f,u>ei&&(o[ui]=(t-this._bvhClosest.point.x)/u*f,o[di]=(e-this._bvhClosest.point.y)/u*f,o[fi]=(n-this._bvhClosest.point.z)/u*f),s.source=zw,this.stats[xp]++,!0}#r(t,e,n,i,s){if(n<0||i<0||s<0){t[e]=NaN;return}const a=Math.floor(n/this.brickSize),o=Math.floor(i/this.brickSize),c=Math.floor(s/this.brickSize);if(a>=this.sdfDimensions[0]||o>=this.sdfDimensions[1]||c>=this.sdfDimensions[2]){t[e]=NaN;return}const l=a+this.sdfDimensions[0]*(o+this.sdfDimensions[1]*c),h=this.sdfBrickLookup[l];if(h===xo){t[e]=NaN;return}const u=n-a*this.brickSize,f=i-o*this.brickSize,d=s-c*this.brickSize,g=u+this.brickSize*(f+this.brickSize*d),x=h*this.valuesPerBrick+g;t[e]=this.sdfDistances[x]*this.sdfQuantization}#s(t,e,n){if(t<0||e<0||n<0)return 0;const i=Math.floor(t/this.brickSize),s=Math.floor(e/this.brickSize),a=Math.floor(n/this.brickSize);if(i>=this.sdfDimensions[0]||s>=this.sdfDimensions[1]||a>=this.sdfDimensions[2])return 0;const o=i+this.sdfDimensions[0]*(s+this.sdfDimensions[1]*a),c=this.sdfBrickLookup[o];if(c===xo)return 0;const l=t-i*this.brickSize,h=e-s*this.brickSize,u=n-a*this.brickSize,f=l+this.brickSize*(h+this.brickSize*u),d=c*this.valuesPerBrick+f;return(this.sdfInsideBits[d>>3]&1<<(d&7))!==0?1:0}}function Zw(r){return globalThis.crypto.subtle.digest("SHA-256",r).then(t=>[...new Uint8Array(t)].map(e=>e.toString(16).padStart(2,"0")).join(""))}async function Tp(r){const t=await fetch(r);if(!t.ok)throw new Error(`Failed to load ${r}: ${t.status} ${t.statusText}`);return t.arrayBuffer()}function jw(r,t,e){if(r.metadata.source?.stlSha256!==t)throw new Error("Aorta collision asset does not match Aorta_plain.stl; run npm run collision:build");const n=r.metadata.transform;if(n?.version!==e.version||Math.abs((n?.scale??1/0)-e.scale)>1e-7||Math.abs((n?.targetLength??1/0)-e.targetLength)>1e-6)throw new Error("Aorta collision asset transform is stale; run npm run collision:build")}function $w(r){const t=r.arrays.centerlineSegments,e=r.arrays.centerlineEdges,n=r.metadata.centerline.stride,i=[];for(let s=0;s<t.length/n;s++){const a=s*n,o=new b(t[a],t[a+1],t[a+2]),c=new b(t[a+3],t[a+4],t[a+5]),l=c.clone().sub(o),h=l.length();h>1e-8?l.multiplyScalar(1/h):l.set(0,1,0),i.push({id:s,start:o,end:c,axis:l,length:h,radiusStart:t[a+6],radiusEnd:t[a+7],safeRadius:t[a+8],nodeStartId:e[s*2],nodeEndId:e[s*2+1],source:"medial-slice-teasar",aabb:null})}return{type:"centerline-capsule-broadphase",source:"medial-slice-teasar",diagnostics:r.metadata.centerline.diagnostics,inflation:r.metadata.sdf.band,cellSize:r.metadata.broadPhase.cellSize,segments:i,contactField:r}}function Kw(r,t=12e3){const e=r.arrays;if(!e.lumenSliceYs?.length)return new Float32Array;const n=e.lumenPoints instanceof Int16Array?r.metadata.lumen?.pointQuantization||.02:1,i=e.lumenAxisBases||new Float32Array([1,0,0,0,1,0,0,0,1]),s=e.lumenAxisSliceOffsets||new Uint32Array([0,e.lumenSliceYs.length]),a=e.lumenPoints.length/2,o=Math.max(1,Math.ceil(a/t)),c=[];let l=0;for(let h=0;h<s.length-1;h++){const u=h*9;for(let f=s[h];f<s[h+1];f++){const d=e.lumenSliceYs[f],g=e.lumenSliceContourOffsets[f],x=e.lumenSliceContourOffsets[f+1];for(let m=g;m<x;m++){const p=e.lumenContourPointOffsets[m],_=e.lumenContourPointOffsets[m+1];for(let v=p;v<_;v++,l++){if(l%o!==0)continue;const S=v+1<_?v+1:p,y=e.lumenPoints[v*2]*n,M=e.lumenPoints[v*2+1]*n,w=e.lumenPoints[S*2]*n,T=e.lumenPoints[S*2+1]*n;c.push(i[u]*y+i[u+3]*d+i[u+6]*M,i[u+1]*y+i[u+4]*d+i[u+7]*M,i[u+2]*y+i[u+5]*d+i[u+8]*M,i[u]*w+i[u+3]*d+i[u+6]*T,i[u+1]*w+i[u+4]*d+i[u+7]*T,i[u+2]*w+i[u+5]*d+i[u+8]*T)}}}}return new Float32Array(c)}function Jw(r,t,e){const n=r.metadata;return{geometry:t,interiorSamples:[],lumenSlices:[],lumenField:r.packedLumenField,boundaryDebugSegments:new Float32Array,lumenContourDebugSegments:Kw(r),centerlineSliceDebugSegments:null,centerlineExtraction:n.centerline.diagnostics,lumenCastGeometry:null,lumenCast:null,collisionAsset:n,diagnostics:{boundingBox:t.boundingBox.clone(),boundaryEdgeCount:0,degenerateTriangleCount:0,edgeCount:0,interiorSampleCount:0,lumenSliceCount:n.lumen.sliceCount,nonManifoldEdgeCount:0,size:t.boundingBox.getSize(new b),transform:e,triangleCount:n.source.triangleCount,vertexCount:t.attributes.position.count,source:"precompiled-collision-asset"}}}function Qw(r,{onLoaded:t,onError:e}={}){const n=new De;n.visible=!1;const i=new Fe({color:5213695,transparent:!0,opacity:.34,depthWrite:!1,side:Ke});return Promise.all([Tp(Cw),Tp(Rw)]).then(async([s,a])=>{const[o]=await Promise.all([Zw(s)]),c=new w1().parse(s),l=Nw(c,r),h=Lm(a);jw(h,o,l),c.computeVertexNormals(),c.computeBoundingSphere(),c.boundsTree=new Ru(c);const u=new qw(h,{fallbackGeometry:c,bvhValidationDistance:.02,capsuleBvhValidation:-.1}),f=$w(u),d=Jw(u,c,l),g=new Zt(c,i);g.renderOrder=0,n.add(g),n.visible=!0;const x={geometry:c,contactField:u,meshCollider:tA(u),centerlineBroadPhase:f,clearance:.6,guidewireClearance:Jo,guidewireSegmentClearance:.12,guidewireCollisionPasses:3,guidewireSegmentSamples:[.2,.4,.6,.8],openOutletY:c.boundingBox.max.y-1,preprocessing:d};typeof t=="function"&&t({group:n,mesh:g,geometry:c,collision:x,preprocessing:d,scale:l.scale})}).catch(s=>{console.warn("Failed to load aorta STL model",s),typeof e=="function"&&e(s)}),{group:n,material:i}}function tA(r){const t=Ui(),e=Ui(),n=(o,c,l,h)=>(typeof o?.set=="function"?o.set(c,l,h):(o.x=c,o.y=l,o.z=h),o),i=(o,c)=>{const l=c||{target:new b,closestPoint:new b,inward:new b,normal:new b};return l.inside=o.inside,l.violation=o.violation,l.distance=Math.max(0,o.signedDistance),l.signedDistance=o.signedDistance,l.signedGap=o.signedGap,l.penetration=o.penetration,l.branchId=o.branchId,l.source=o.source,l.target=l.target||{},l.closestPoint=l.closestPoint||{},l.inward=l.inward||{},l.normal=l.normal||{},n(l.target,o.target.x,o.target.y,o.target.z),n(l.closestPoint,o.closestPoint.x,o.closestPoint.y,o.closestPoint.z),n(l.inward,o.inward.x,o.inward.y,o.inward.z),n(l.normal,-o.inward.x,-o.inward.y,-o.inward.z),l},s=(o,c=0,l=null)=>i(r.querySphere(o,c,t),l),a=(o,c,l=0)=>{const h=r.sweepSphere(o,c,l,e);return!h.violation&&h.timeOfImpact>=1?null:{penetration:h.penetration,point:new b(h.point.x,h.point.y,h.point.z),target:new b(h.target.x,h.target.y,h.target.z),normal:new b(-h.inward.x,-h.inward.y,-h.inward.z),inward:new b(h.inward.x,h.inward.y,h.inward.z),t:h.timeOfImpact,branchId:h.branchId}};return{geometry:r.fallbackGeometry,lumenField:r.packedLumenField,broadPhase:r,contactField:r,containsPoint:o=>!s(o,0,t).violation,pointContact:s,crossingContact:a,clearCache:()=>{}}}const eA=1.5,nA=900,iA=16773994,sA=16777215,rA=16732120,oA=1.05,aA=1.75,cA=3778303,lA=9427199;function hA(r,t=eA){return Number.isFinite(r)&&r>0?r:t}function uA(r,t){r.position.copy(t.start).lerp(t.end,.5),r.quaternion.setFromUnitVectors(new b(0,1,0),t.axis)}function dA(r,t,e,n){const i=new Zt(new gs(t.radiusEnd,t.radiusStart,t.length,18,1,!0),e);uA(i,t),i.renderOrder=4.5,i.userData.debugLayer="capsules",r.add(i);const s=new Zt(new ai(t.radiusStart,16,8),n);s.position.copy(t.start),s.renderOrder=4.4,s.userData.debugLayer="capsules",r.add(s);const a=new Zt(new ai(t.radiusEnd,16,8),n);a.position.copy(t.end),a.renderOrder=4.4,a.userData.debugLayer="capsules",r.add(a)}function nc(r){return[Math.round(r.x*8),Math.round(r.y*8),Math.round(r.z*8)].join(",")}function mh(r,t,e,n){let i=r.get(t);i||(i={point:new b,radius:0,degree:0,weight:0},r.set(t,i)),i.point.add(e),i.radius=Math.max(i.radius,hA(n)),i.degree++,i.weight++}function fA(r){const t=new Map;for(const e of r){if(e.nodeStartId!==void 0&&e.nodeStartId===e.nodeEndId){const s=e.start.clone().lerp(e.end,.5);mh(t,`node:${e.nodeStartId}`,s,Math.max(e.radiusStart,e.radiusEnd));continue}const n=e.nodeStartId!==void 0?`node:${e.nodeStartId}:${nc(e.start)}`:`point:${nc(e.start)}`,i=e.nodeEndId!==void 0?`node:${e.nodeEndId}:${nc(e.end)}`:`point:${nc(e.end)}`;mh(t,n,e.start,e.radiusStart),mh(t,i,e.end,e.radiusEnd)}return[...t.values()].map(e=>({...e,point:e.point.multiplyScalar(1/Math.max(1,e.weight))}))}function bp(r,{radius:t,color:e,opacity:n,renderOrder:i}){if(!r.length)return null;const s=new Hr(new ai(t,10,6),new Fe({color:e,transparent:!0,opacity:n,depthTest:!1,depthWrite:!1,toneMapped:!1}),r.length),a=new se,o=new b;for(let c=0;c<r.length;c++){const l=Math.max(.72,Math.min(1.65,r[c].radius*.16));o.setScalar(l),a.compose(r[c].point,new Nn,o),s.setMatrixAt(c,a)}return s.frustumCulled=!1,s.instanceMatrix.needsUpdate=!0,s.renderOrder=i,s.userData.debugLayer="centerline",s}function pA(r,{maxCapsules:t=nA}={}){const e=new De;if(!r?.segments?.length)return e;const n=r.segments,i=new Float32Array(n.length*6);for(let v=0;v<n.length;v++){const S=n[v];i[v*6]=S.start.x,i[v*6+1]=S.start.y,i[v*6+2]=S.start.z,i[v*6+3]=S.end.x,i[v*6+4]=S.end.y,i[v*6+5]=S.end.z}const s=new ye;s.setAttribute("position",new Ne(i,3));const a=new ds(s,new vi({color:iA,transparent:!0,opacity:.96,depthTest:!1,depthWrite:!1,toneMapped:!1}));a.frustumCulled=!1,a.renderOrder=9.7,a.userData.debugLayer="centerline",e.add(a);const o=fA(n),c=o.filter(v=>v.degree===2),l=o.filter(v=>v.degree!==2),h=bp(c,{radius:oA,color:sA,opacity:.92,renderOrder:9.78});h&&e.add(h);const u=bp(l,{radius:aA,color:rA,opacity:.98,renderOrder:9.82});u&&e.add(u);const f=Number.isFinite(t)&&t>0?Math.max(1,Math.ceil(r.segments.length/t)):1,d=r.segments.filter((v,S)=>S%f===0),g=new Fe({color:cA,transparent:!0,opacity:.105,depthTest:!0,depthWrite:!1,side:Ke,toneMapped:!1}),x=g.clone();x.opacity=.075;for(const v of d)dA(e,v,g,x);const m=new Float32Array(d.length*6);for(let v=0;v<d.length;v++){const S=d[v];m[v*6]=S.start.x,m[v*6+1]=S.start.y,m[v*6+2]=S.start.z,m[v*6+3]=S.end.x,m[v*6+4]=S.end.y,m[v*6+5]=S.end.z}const p=new ye;p.setAttribute("position",new Ne(m,3));const _=new ds(p,new vi({color:lA,transparent:!0,opacity:.38,depthTest:!1,depthWrite:!1,toneMapped:!1}));return _.frustumCulled=!1,_.renderOrder=9.55,_.userData.debugLayer="capsules",e.add(_),e.userData.broadPhase=r,e.userData.displayedSegmentCount=d.length,e.userData.centerlineNodeCount=o.length,e.userData.centerlineBranchNodeCount=l.length,e}const mA=`
// Fullscreen quad vertex shader.
// Passes through the quad UVs and positions to draw a screen-aligned quad.
varying vec2 vUv;
void main() {
    vUv = uv;
    gl_Position = vec4(position.xy, 0.0, 1.0);
}
`,gA=`
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
`,xA=`
// Fullscreen quad vertex shader for thickness pass.
// Renders a screen-aligned quad; vUv is used to sample depth textures.
varying vec2 vUv;
void main() {
    vUv = uv;
    gl_Position = vec4(position.xy, 0.0, 1.0);
}
`,_A=`
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
`,vA=`
// Fullscreen quad vertex shader for the final display pass.
// Simply forwards UVs to the fragment shader.
varying vec2 vUv;
void main() {
    vUv = uv;
    gl_Position = vec4(position.xy, 0.0, 1.0);
}
`,SA=`
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
`,Tc=600*1e3,qc=72*1e3;function MA(){return{guidewireAdvance:0,catheterAdvance:0,catheterRotation:0,catheterType:"pigtail"}}function Qm(r){const t=Math.max(0,r);return Math.floor(t/qc)%2===0?"pigtail":"berenstein"}function Cp(r,t){const e=Math.max(0,r),n=e%qc;return t.guidewireAdvance=0,t.catheterAdvance=0,t.catheterRotation=0,t.catheterType=Qm(e),n<15e3?t.guidewireAdvance=1:n<25e3?t.catheterAdvance=1:n<35e3?t.catheterRotation=Math.floor((n-25e3)/2500)%2===0?1:-1:n<52e3?(t.catheterAdvance=-1,t.catheterRotation=Math.floor((n-35e3)/2500)%2===0?-1:1):n<67e3&&(t.guidewireAdvance=-1),t}const Lu=2752468,yA=5213695,EA=6946702,wA=16751421,AA=11009884,TA=16732120,bA=16765514,CA=16724821,RA=16733695,Fs=420,tg=1/10,PA=1.85,LA=32,IA=Fh*1.35,eg=1/30,DA=new URLSearchParams(window.location.search).get("physics"),Zn=DA==="legacy"?"legacy":"xpbd-contact-v1",ng=.1,ig=1e3,Ro=document.getElementById("loadingScreen"),Rp=document.getElementById("loadingMessage"),ls=new Set(["aorta","skeleton","firstFrame"]);let Pp=!1,qo=null;function sa(r){Rp&&(Rp.textContent=r)}function sg(){return!ls.has("aorta")&&!ls.has("skeleton")}function NA(){Pp||!Ro||(Pp=!0,sa("Ready"),Ro.classList.add("is-hidden"),Ro.addEventListener("transitionend",()=>Ro.remove(),{once:!0}),setTimeout(()=>Ro.remove(),900))}function zr(r,t){ls.has(r)&&(ls.delete(r),r==="firstFrame"&&qo&&(clearTimeout(qo),qo=null),t&&sa(t),FA(),ls.size===0&&NA())}function rg(r){zr(r,"Loading fallback view")}function FA(){!sg()||!ls.has("firstFrame")||qo||(sa("Rendering first frame"),requestAnimationFrame(()=>zr("firstFrame","Ready")),qo=setTimeout(()=>zr("firstFrame","Ready"),1800))}function Lp(){sg()&&zr("firstFrame","Ready")}sa("Preparing renderer");const UA=document.getElementById("sim"),Wt=new _u({canvas:UA,antialias:!0});Wt.setSize(window.innerWidth,window.innerHeight);const bc=.64,zA=()=>Math.max(1,Math.round(window.innerWidth*bc)),BA=()=>Math.max(1,Math.round(window.innerHeight*bc)),jn=zA(),$n=BA(),OA=Wt.capabilities.isWebGL2?2:0,og={samples:OA},le=new kr;le.background=new jt(0);const Po=new kr,$h=new tn(jn,$n,og),Cc=new tn(jn,$n),Rc=new tn(jn,$n,og),Pc=new tn(jn,$n),Lc=new tn(jn,$n),Zc=new tn(jn,$n,{type:Nr}),ag=new tn(jn,$n),cg=new tn(jn,$n),Ic=new tn(jn,$n),Dc=new tn(jn,$n),jc=new tn(jn,$n);let fc=ag,ic=cg;const Ip=new Float64Array(16),Dp=new Float64Array(16);let Nc=!1;const pc=new gu(-1,1,1,-1,0,1),Iu=new kc(2,2),Fc=new Fn({uniforms:{currentFrame:{value:null},previousFrame:{value:null},decay:{value:.95}},vertexShader:mA,fragmentShader:gA}),GA=new Zt(Iu,Fc),lg=new kr;lg.add(GA);function hg(r){return new Fn({side:r,depthTest:!0,depthWrite:!0,uniforms:{cameraNear:{value:ng},cameraFar:{value:ig}},vertexShader:`
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
        `})}const VA=hg(oi),kA=hg(dn),Kh=new Fn({uniforms:{frontDepth:{value:Ic.texture},backDepth:{value:Dc.texture}},vertexShader:xA,fragmentShader:_A}),HA=new Zt(Iu,Kh),ug=new kr;ug.add(HA);const WA=new Fn({transparent:!0,blending:Zp,blendEquation:Bi,blendSrc:Lo,blendDst:Lo,blendEquationAlpha:Bi,blendSrcAlpha:Lo,blendDstAlpha:Lo,side:Ke,depthTest:!1,depthWrite:!1,vertexShader:`
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
    `}),xn=new Fn({uniforms:{uTexture:{value:fc.texture},contrastTexture:{value:Cc.texture},thicknessTexture:{value:jc.texture},metalTexture:{value:Rc.texture},catheterTexture:{value:Pc.texture},sheathTexture:{value:Lc.texture},boneTexture:{value:Zc.texture},gray:{value:new jt(15461355)},fluoroscopy:{value:!1},time:{value:0},noiseLevel:{value:.1},imageBrightness:{value:.18},imageContrast:{value:1.33},autoExposureEnabled:{value:!1},autoExposureLevel:{value:0},pulseRate:{value:15},scatterStrength:{value:.45},collimation:{value:.08},boneOpacity:{value:.62},resolution:{value:new St(jn,$n)},edgeStrength:{value:.1},contrastOpacity:{value:1},contrastGain:{value:5}},vertexShader:vA,fragmentShader:SA}),XA=new Zt(Iu,xn),Jh=new kr;Jh.add(XA);const dg=350,Be=new Ln(45,window.innerWidth/window.innerHeight,ng,ig);Be.position.set(0,80,dg);le.add(Be);let Xr;const{group:Us}=kE({onLoaded:()=>{Nc=!1,zr("skeleton",ls.has("aorta")?"Loading vessel model":"Rendering first frame")},onError:()=>rg("skeleton")}),{vessel:fn}=AE(140,0);Xr=new De;let vn=fn,ge=null,Oe=null,xe=null,Le=null,Hi=null,Ar=null,Ls=null,rs=null,Rr=null,es=null;function fg(r,t=1){const e=new b(r.start.x,r.start.y,r.start.z),n=new b(r.end.x,r.end.y,r.end.z),i=new b().subVectors(n,e),s=i.length(),a=r.radius*t,o=new gs(a,a,s,18,1,!0);return o.applyQuaternion(new Nn().setFromUnitVectors(new b(0,1,0),i.normalize())),o.translate((e.x+n.x)*.5,(e.y+n.y)*.5,(e.z+n.z)*.5),o}function YA(r){const t=fg(r),e=new Fe({color:Lu,side:Ke,transparent:!0,opacity:.34,depthWrite:!1,depthTest:!1}),n=new Zt(t,e);return n.renderOrder=6.6,n}function qA(r){const t=fg(r),e=new Fe({color:16777215,side:Ke,transparent:!0,opacity:.065,depthTest:!1,depthWrite:!1}),n=new Zt(t,e);return n.renderOrder=.7,n}function Np(r,{debugLayer:t=null,color:e=Lu,opacity:n=.24,renderOrder:i=3,depthTest:s=!0}={}){const a=new Fe({color:e,side:Ke,transparent:!0,opacity:n,depthWrite:!1,depthTest:s}),o=new Zt(r,a);return o.renderOrder=i,t&&(o.userData.debugLayer=t),o}function ZA(r,t){const e=r?.meshCollider||r?.lumenMeshCollider||null;if(!e?.pointContact||!t?.start||!t?.end)return null;const n=new b(t.start.x,t.start.y,t.start.z),i=new b(t.end.x,t.end.y,t.end.z),s=new b().subVectors(i,n),a=s.length();if(a<1e-6)return null;const o=f=>n.clone().addScaledVector(s,f),c=f=>e.pointContact(o(f),0)?.signedDistance??-1/0,l=Math.max(16,Math.ceil(a/2));let h=0,u=c(0);for(let f=1;f<=l;f++){const d=f/l,g=c(d);if(u<0&&g>=0){let x=h,m=d;for(let p=0;p<14;p++){const _=(x+m)*.5;c(_)>=0?m=_:x=_}return{point:o(m),tangent:s.normalize()}}h=d,u=g}return null}function jA(r,t){const e=ZA(r,t),n=new De;if(!e)return n;const i=new Fe({color:TA,transparent:!0,opacity:.95,depthTest:!1,depthWrite:!1,toneMapped:!1}),s=new Zt(new ai(2.2,18,12),i);s.renderOrder=9.5,n.add(s);const a=new Zt(new Eu(4.2,.32,8,32),i.clone());return a.quaternion.setFromUnitVectors(new b(0,0,1),e.tangent),a.renderOrder=9.4,n.add(a),n.position.copy(e.point),n.frustumCulled=!1,n}Xr.add(YA(fn.sheath));const Pr=qA(fn.sheath);Pr.visible=!0;le.add(Pr);const xi=new De;xi.visible=!1;Xr.add(xi);const Lr={stlModel:!0,lumenCast:!1,sections:!1,centerline:!0,capsules:!1};function pg(){xi.traverse(r=>{const t=r.userData?.debugLayer;!t||!(t in Lr)||(r.visible=!!Lr[t])})}sa("Loading anatomy models");Qw(fn,{onLoaded:({collision:r})=>{vn={...r,segments:[fn.sheath]},xe&&(xe.contactField=r.contactField),xi.clear(),xi.add(Np(r.geometry,{debugLayer:"stlModel",color:yA,opacity:.18,renderOrder:2.8})),r.preprocessing?.lumenCastGeometry&&xi.add(Np(r.preprocessing.lumenCastGeometry,{debugLayer:"lumenCast",color:Lu,opacity:.28,renderOrder:9.15,depthTest:!1})),xi.add(iT(r.preprocessing)),xi.add(pA(r.centerlineBroadPhase)),xi.add(jA(r,fn.sheath)),pg(),Oe?.requestSettle?.(90),ge?.setCollisionGeometry(r),zr("aorta",ls.has("skeleton")?"Loading skeleton model":"Rendering first frame")},onError:()=>{rg("aorta")}});le.add(Xr);Us.position.set(fn.branchPoint.x,fn.branchPoint.y-60,fn.branchPoint.z-50);Us.renderOrder=-1;le.add(Us);const Zo=new qE(fn,3.5),mi=new De;le.add(mi);let Di=null,gh=0,xh=0;const ps=5,$c=201,Qo=ps*($c-1),$A=44,KA=32,JA=8,QA=105,tT=24,qn=new Sy($c,ps,{constraintIterations:28});let Kc=0;const eT=Qo,nT=0;function iT(r){const t=new De;if(!r)return t;if(r.boundaryDebugSegments?.length){const i=new ye;i.setAttribute("position",new Ne(r.boundaryDebugSegments,3));const s=new ds(i,new vi({color:wA,transparent:!0,opacity:.85,depthTest:!1,depthWrite:!1,toneMapped:!1}));s.frustumCulled=!1,s.renderOrder=9,s.userData.debugLayer="sections",t.add(s)}const e=r.centerlineSliceDebugSegments?.length?r.centerlineSliceDebugSegments:r.lumenContourDebugSegments;if(e?.length){const i=new ye;i.setAttribute("position",new Ne(e,3));const s=new ds(i,new vi({color:AA,transparent:!0,opacity:.72,depthTest:!1,depthWrite:!1,toneMapped:!1}));s.frustumCulled=!1,s.renderOrder=8.5,s.userData.debugLayer="sections",t.add(s)}const n=r.interiorSamples||[];if(n.length){const i=new Hr(new ai(1.15,10,6),new Fe({color:EA,transparent:!0,opacity:.82,depthTest:!1,depthWrite:!1,toneMapped:!1}),n.length),s=new se;for(let a=0;a<n.length;a++)s.makeTranslation(n[a].x,n[a].y,n[a].z),i.setMatrixAt(a,s);i.frustumCulled=!1,i.instanceMatrix.needsUpdate=!0,i.renderOrder=8,i.userData.debugLayer="sections",t.add(i)}return t}Oe=new aE({rod:qn,segmentLength:ps,guidewireLength:Qo,sheath:fn.sheath,advanceRate:$A,minInsert:nT,maxInsert:eT,lumenClearance:Jo,straightening:.72,routeBlend:0,relaxationIterations:6,lengthIterations:10,meshClearance:Jo,foldGuardAngle:166,foldGuardStrength:.62,foldGuardPasses:2,foldGuardCenterPull:1.25,stabilityRepairSegmentError:.09,stabilityRepairBendAngle:150,stabilityRepairTargetBendAngle:112,stabilityRepairPasses:3,stabilityRepairLengthIterations:10,tipBacktrackAngle:108,tipBacktrackStrength:1,segmentProjectionBlend:.48,maxSegmentProjectionStep:.32,collisionProjectionRepeats:1,segmentSamples:[.1,.24,.38,.52,.66,.8,.93],finalCollisionPasses:3,finalLengthPasses:2,finalProjectionPasses:2});function sT(r,t,e){const n=Math.max(0,Math.min(1,(e-r)/Math.max(1e-6,t-r)));return n*n*(3-2*n)}function rT(){const r=qn.nodes.length-1;for(let t=0;t<qn.nodes.length;t++){const e=(r-t)*ps,n=sT(tT,QA,e);qn.nodes[t].bendingStiffness=JA*(1-n)+KA*n}}rT();Oe.initialize();Kc=Oe.progress;let _i=!1,Qh=0,mg=2,tu=2,Fp=10,jo=0,Up=0;const gg=new Fe({color:16777215,depthTest:!1,depthWrite:!1,toneMapped:!1}),zp=new Fn({vertexShader:`
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
    `,depthTest:!1,depthWrite:!1,toneMapped:!1});let Ce=!0,Un=null,zn=null,We=null;const Re=FE({camera:Be,cameraRadius:dg,vessel:fn,voxelGroup:mi,displayMaterial:xn,blendMaterial:Fc,wireMaterial:gg,onStartInjection:({rate:r,duration:t,volume:e})=>{_i||(_i=!0,Qh=0,tu=r,mg=t,Fp=e,jo=Fp)},onStopInjection:()=>{_i&&(_i=!1,jo=0)},onModeChange:r=>{Ce=r,Xr.visible=!Ce,Pr.visible=Ce,xi.visible=!Ce,Un&&(Un.visible=!Ce),zn&&(zn.visible=!Ce),We&&(We.visible=!Ce&&!!We.userData.hasPoint),rs&&(rs.visible=!Ce&&!!Lr.capsules),Us.visible=Ce,xn.uniforms.fluoroscopy.value=Ce},onDebugLayerChange:r=>{Object.assign(Lr,r),pg(),rs&&(rs.visible=!Ce&&!!Lr.capsules)},onStartBrowserBenchmark:r=>Eg({durationMs:r}),onStopBrowserBenchmark:()=>Ou("ui")}),{monitor:oT}=Re,aT=new gs(Fh,Fh,1,LA,1,!1),hs=new Hr(aT,gg,$c-1);hs.instanceMatrix.setUsage(Vc);hs.frustumCulled=!1;hs.renderOrder=7;hs.count=0;const ta=new De;ta.add(hs);le.add(ta);const Bp=new se,Op=new Nn,sc=new b,Gp=new b,Vp=new b(1,1,1),cT=new b(0,1,0),kp=new se,lT=new ai(1.35,12,8),hT=new ai(2.1,12,8);Un=new Hr(lT,new Fe({color:bA,transparent:!0,opacity:.95,depthTest:!1,depthWrite:!1,toneMapped:!1}),Fs);Un.instanceMatrix.setUsage(Vc);Un.count=0;Un.visible=!0;Un.frustumCulled=!1;Un.renderOrder=6;le.add(Un);zn=new Hr(hT,new Fe({color:CA,transparent:!0,opacity:1,depthTest:!1,depthWrite:!1,toneMapped:!1}),Fs);zn.instanceMatrix.setUsage(Vc);zn.count=0;zn.visible=!0;zn.frustumCulled=!1;zn.renderOrder=7;le.add(zn);We=new Zt(new ai(2.8,16,10),new Fe({color:RA,transparent:!0,opacity:1,depthTest:!1,depthWrite:!1,toneMapped:!1}));We.visible=!1;We.frustumCulled=!1;We.renderOrder=8;We.userData.hasPoint=!1;le.add(We);function xg(r){const t=new Float32Array(Fs*6),e=new ye;e.setAttribute("position",new Ne(t,3)),e.setDrawRange(0,0);const n=new ds(e,new vi({color:r,transparent:!0,opacity:.95,depthTest:!1,depthWrite:!1,toneMapped:!1}));return n.frustumCulled=!1,n.renderOrder=9.9,n}rs=new De;Rr=xg(2686935);es=xg(16732120);rs.add(Rr,es);rs.visible=!Ce&&!!Lr.capsules;le.add(rs);const uT={routeAssist:Zn==="legacy"},dT={resetVelocity:!0},fT={collisions:!1},_g={shapeCompliance:2e-4};ge=new E1({wire:qn,segmentLength:ps,guidewireLength:Qo,tailProgressRef:()=>Oe.progress,vessel:fn});ge.setExternalCollisionSolver(Zn==="xpbd-contact-v1");vn!==fn&&ge.setCollisionGeometry(vn);le.add(ge.mesh);xe=new wE({contactField:vn.contactField||null,fixedDt:1/120,maxSubsteps:2,iterations:6,penetrationIterations:8,highPenetration:.15,contactActivation:.2});Le=xe.createRod("guidewire",$c,ps,{...Tu.guidewire});Le.syncFromElasticRod(qn);Hi=xe.createRod("catheter",320,4,{...Tu.catheter,bendCompliance:2e-4,shapeCompliance:2e-4});ge.syncXpbdBody(Hi,_g);xe.addSheath({start:fn.sheath.start,end:fn.sheath.end,innerRadius:Gm,bodies:[Le,Hi]});Ar=xe.addContainment(Le,Hi,{innerRadius:Wm,openProximal:!0,openDistal:!0,searchWindow:8,outerStartNode:ge.physicsLumenStartNode,enabled:!1});Ls=xe.addToolContact(Le,Hi,{friction:.08,openDistalB:!0,enabled:!1});const Hp=[Le,Hi];globalThis.__OET_PHYSICS__={mode:Zn,world:xe,getStats:()=>xe.getStats()};const Bs=4e4,vg=qc*2,pT=60*1e3,eu=vg+pT,mT=610,Sg=256,Du=8,Uo=new Float32Array(Bs),nu=new Float32Array(mT),iu=new Float32Array(Bs),gT=new Float32Array(Bs),su=new Float32Array(Bs),ru=new Float32Array(Bs),un=new Float32Array(Sg*Du);let zo=0,zs=0,ea=0,na=0,Nu=0,Uc=0,Br=0,Bo=0,mc=0,zc=0,ou=0,Fu=0,Uu=0,us=0,Mg=performance.now();const Yt={count:0,simulationSumMs:0,updateSumMs:0,renderSumMs:0,totalSumMs:0,maximumMs:0,simulationMaximumMs:0,updateMaximumMs:0,renderMaximumMs:0,lastSimulationMs:0,lastUpdateMs:0,lastRenderMs:0,lastTotalMs:0},we={supported:!1,samples:0,startBytes:null,minimumBytes:null,maximumBytes:null,endBytes:null},yt={running:!1,warmingUp:!1,durationMs:Tc,warmupStartedAt:0,memorySettling:!1,startedAt:0,completedAt:0,simulationElapsedMs:0,stopReason:null,automated:!1},xT=["pointerdown","mousedown","touchstart","click","dblclick","wheel","keydown","input","change"];function _T(r){!yt.running||!yt.automated||(r.preventDefault(),r.stopImmediatePropagation())}for(const r of xT)window.addEventListener(r,_T,{capture:!0,passive:!1});const bs=MA(),ve={steps:0,maxPostStepPenetrationMm:0,maxPostStepPenetrationStep:-1,maxPostStepPenetrationBodyId:null,maxPostStepPenetrationSegment:-1,maxPostStepPenetrationT:0,maxPostStepPenetrationX:0,maxPostStepPenetrationY:0,maxPostStepPenetrationZ:0,maxTransientPenetrationMm:0,maxTransientPenetrationStep:-1,maxSegmentErrorPercent:0,maxSegmentErrorBodyId:null,maxSegmentErrorNodeIndex:-1,maxSegmentErrorStep:-1,maxBendAngleDegrees:0,maxBendBodyId:null,maxBendNodeIndex:-1,maxBendStep:-1,maxBendX:0,maxBendY:0,maxBendZ:0,finite:!0};let ia=null;function zu(){const r=performance.memory?.usedJSHeapSize;Number.isFinite(r)&&(we.supported=!0,we.samples===0?(we.startBytes=r,we.minimumBytes=r,we.maximumBytes=r):(we.minimumBytes=Math.min(we.minimumBytes,r),we.maximumBytes=Math.max(we.maximumBytes,r)),we.endBytes=r,we.samples++)}function vT(){we.supported=!1,we.samples=0,we.startBytes=null,we.minimumBytes=null,we.maximumBytes=null,we.endBytes=null,zu()}function ST(){return zu(),{...we,growthBytes:we.supported?we.endBytes-we.startBytes:null,rangeBytes:we.supported?we.maximumBytes-we.minimumBytes:null}}function Bu(){zo=0,zs=0,ea=0,na=0,Nu=0,Uc=0,Br=0,Bo=0,mc=0,zc=0,ou=Re.getCArmRevision?.()??0,Fu=0,Uu=0,us=document.hasFocus()?0:performance.now(),Yt.count=0,Yt.simulationSumMs=0,Yt.updateSumMs=0,Yt.renderSumMs=0,Yt.totalSumMs=0,Yt.maximumMs=0,Yt.simulationMaximumMs=0,Yt.updateMaximumMs=0,Yt.renderMaximumMs=0,Yt.lastSimulationMs=0,Yt.lastUpdateMs=0,Yt.lastRenderMs=0,Yt.lastTotalMs=0,vT(),ve.steps=0,ve.maxPostStepPenetrationMm=0,ve.maxPostStepPenetrationStep=-1,ve.maxPostStepPenetrationBodyId=null,ve.maxPostStepPenetrationSegment=-1,ve.maxTransientPenetrationMm=0,ve.maxTransientPenetrationStep=-1,ve.maxSegmentErrorPercent=0,ve.maxSegmentErrorBodyId=null,ve.maxSegmentErrorNodeIndex=-1,ve.maxSegmentErrorStep=-1,ve.maxBendAngleDegrees=0,ve.maxBendBodyId=null,ve.maxBendNodeIndex=-1,ve.maxBendStep=-1,ve.maxBendX=0,ve.maxBendY=0,ve.maxBendZ=0,ve.finite=!0,Mg=performance.now(),xe.resetPerformanceStats(),vn.contactField?.resetStats?.()}function MT(r){if(!(!Number.isFinite(r)||r<=0)){if(zs===Uo.length?ea-=Uo[zo]:zs++,Uo[zo]=r,ea+=r,na=Math.max(na,r),Bo+=r,mc++,Bo>=1e3&&(zu(),Br<nu.length&&(nu[Br++]=mc*1e3/Bo),Bo=0,mc=0),r>1e3/30&&(Nu++,zc<Sg)){const t=zc++*Du;un[t]=r,un[t+1]=yt.running?performance.now()-yt.startedAt:-1,un[t+2]=yt.simulationElapsedMs,un[t+3]=performance.memory?.usedJSHeapSize??-1,un[t+4]=Yt.lastSimulationMs,un[t+5]=Yt.lastUpdateMs,un[t+6]=Yt.lastRenderMs,un[t+7]=Yt.lastTotalMs}r>50&&Uc++,zo=(zo+1)%Uo.length}}window.addEventListener("blur",()=>{!yt.running||yt.warmingUp||us>0||(Fu++,us=performance.now())});window.addEventListener("focus",()=>{us<=0||(yt.running&&!yt.warmingUp&&(Uu+=performance.now()-us),us=0)});function yT(){const r=ve;if(r.steps++,xe.settledMaxPenetration>r.maxPostStepPenetrationMm&&(r.maxPostStepPenetrationMm=xe.settledMaxPenetration,r.maxPostStepPenetrationStep=r.steps,r.maxPostStepPenetrationBodyId=xe.settledContactBodyId,r.maxPostStepPenetrationSegment=xe.settledContactSegment,r.maxPostStepPenetrationT=xe.settledContactT,r.maxPostStepPenetrationX=xe.settledContactX,r.maxPostStepPenetrationY=xe.settledContactY,r.maxPostStepPenetrationZ=xe.settledContactZ),xe.maxPenetration>r.maxTransientPenetrationMm&&(r.maxTransientPenetrationMm=xe.maxPenetration,r.maxTransientPenetrationStep=r.steps),!(r.steps!==1&&r.steps%30!==0))for(let t=0;t<Hp.length;t++){const e=Hp[t];if(!e)continue;const n=e.activeStart,i=Math.min(e.activeEnd,e.segmentCount);for(let s=n;s<=e.activeEnd;s++)r.finite=r.finite&&Number.isFinite(e.x[s])&&Number.isFinite(e.y[s])&&Number.isFinite(e.z[s])&&Number.isFinite(e.velocityX[s])&&Number.isFinite(e.velocityY[s])&&Number.isFinite(e.velocityZ[s]);for(let s=n;s<i;s++){const a=e.x[s+1]-e.x[s],o=e.y[s+1]-e.y[s],c=e.z[s+1]-e.z[s],l=Math.sqrt(a*a+o*o+c*c),h=Math.abs(l-e.restLength[s])/Math.max(1e-8,e.restLength[s])*100;if(h>r.maxSegmentErrorPercent&&(r.maxSegmentErrorPercent=h,r.maxSegmentErrorBodyId=e.id,r.maxSegmentErrorNodeIndex=s,r.maxSegmentErrorStep=r.steps),s<=n)continue;const u=e.x[s]-e.x[s-1],f=e.y[s]-e.y[s-1],d=e.z[s]-e.z[s-1],g=Math.sqrt(a*a+o*o+c*c)*Math.sqrt(u*u+f*f+d*d);if(g<=1e-8)continue;const x=he.clamp((a*u+o*f+c*d)/g,-1,1),m=Math.acos(x)*180/Math.PI;m>r.maxBendAngleDegrees&&(r.maxBendAngleDegrees=m,r.maxBendBodyId=e.id,r.maxBendNodeIndex=s,r.maxBendStep=r.steps,r.maxBendX=e.x[s],r.maxBendY=e.y[s],r.maxBendZ=e.z[s])}}}function ET(r){if(!zs)return 0;const t=Array.from(Uo.subarray(0,zs));return t.sort((e,n)=>e-n),t[Math.min(t.length-1,Math.floor((t.length-1)*r))]}function wT(){if(!Br)return 0;const r=Array.from(nu.subarray(0,Br));r.sort((n,i)=>n-i);const t=Math.max(1,Math.ceil(r.length*.01));let e=0;for(let n=0;n<t;n++)e+=r[n];return e/t}function AT(){const r=[];for(let t=0;t<zc;t++){const e=t*Du;r.push({frameMs:un[e],elapsedMs:un[e+1],simulationElapsedMs:un[e+2],heapBytes:un[e+3],previousFrameCpu:{simulationMs:un[e+4],updateMs:un[e+5],renderMs:un[e+6],totalMs:un[e+7]}})}return r}function Wp(r,t,e){if(!yt.running)return;const n=performance.now(),i=t-r,s=e-t,a=n-e,o=n-r,c=Yt.count;c<Bs&&(iu[c]=i,gT[c]=s,su[c]=a,ru[c]=o),Yt.count++,Yt.simulationSumMs+=i,Yt.updateSumMs+=s,Yt.renderSumMs+=a,Yt.totalSumMs+=o,Yt.maximumMs=Math.max(Yt.maximumMs,o),Yt.simulationMaximumMs=Math.max(Yt.simulationMaximumMs,i),Yt.updateMaximumMs=Math.max(Yt.updateMaximumMs,s),Yt.renderMaximumMs=Math.max(Yt.renderMaximumMs,a),Yt.lastSimulationMs=i,Yt.lastUpdateMs=s,Yt.lastRenderMs=a,Yt.lastTotalMs=o}function mr(r,t){const e=Math.min(Yt.count,Bs);if(!e)return 0;const n=Array.from(r.subarray(0,e));return n.sort((i,s)=>i-s),n[Math.min(e-1,Math.floor((e-1)*t))]}function TT(){const r=Yt.count||1;return{samples:Yt.count,simulationAverageMs:Yt.simulationSumMs/r,updateAverageMs:Yt.updateSumMs/r,renderAverageMs:Yt.renderSumMs/r,totalAverageMs:Yt.totalSumMs/r,simulationP95Ms:mr(iu,.95),simulationP99Ms:mr(iu,.99),renderP95Ms:mr(su,.95),renderP99Ms:mr(su,.99),totalP95Ms:mr(ru,.95),totalP99Ms:mr(ru,.99),simulationMaximumMs:Yt.simulationMaximumMs,updateMaximumMs:Yt.updateMaximumMs,renderMaximumMs:Yt.renderMaximumMs,maximumMs:Yt.maximumMs}}function Jc(){const r=performance.now(),t=yt.warmingUp?0:yt.running?Math.min(yt.durationMs,r-yt.startedAt):yt.completedAt>yt.startedAt?Math.min(yt.durationMs,yt.completedAt-yt.startedAt):0;return{running:yt.running,warmingUp:yt.warmingUp,warmupPhase:yt.warmingUp?yt.memorySettling?"memory-settle":"choreography":"complete",warmupElapsedMs:yt.warmingUp?Math.min(eu,r-yt.warmupStartedAt):eu,durationMs:yt.durationMs,elapsedMs:t,simulationElapsedMs:yt.simulationElapsedMs,progress:yt.durationMs>0?Math.min(1,t/yt.durationMs):0,cycleIndex:Math.floor(yt.simulationElapsedMs/qc),catheterType:Qm(yt.simulationElapsedMs),stopReason:yt.stopReason,automated:yt.automated}}function au(){const r=performance.now(),t=ET(.99),e=wT(),n=xe.getStats(),i=vn.contactField?.getStats?.()||null,s=Jc(),a=!s.running&&s.durationMs>=Tc&&s.elapsedMs>=Tc,o=e>=55,c=Uc===0,l=n.phases.total.averageMs<=4&&n.phases.total.p95Ms<=6,h=ve.maxPostStepPenetrationMm<=.2,u=ve.maxSegmentErrorPercent<=1,f=ve.maxBendAngleDegrees<150,d=ve.finite,g=Zn==="xpbd-contact-v1",x=!!xe.contactField,m=Math.max(0,(Re.getCArmRevision?.()??ou)-ou),p=m===0,_=Uu+(us>0?r-us:0),v=_<=100,S=ST(),y=!S.supported||S.growthBytes<=4*1024*1024&&S.rangeBytes<=8*1024*1024,M=i?.resultAllocations===0,w=(i?.runtimeBytes??1/0)<=32*1024*1024,T=na<100&&y&&M;return{mode:Zn,durationMs:performance.now()-Mg,frameCount:zs,averageFps:ea>0?zs*1e3/ea:0,onePercentLowFps:e,p99FrameMs:t,instantaneousP99Fps:t>0?1e3/t:0,fpsWindowCount:Br,maxFrameMs:na,longFrame33Count:Nu,longFrame50Count:Uc,longFrameEvents:AT(),frameCpu:TT(),physics:n,physicsEnvelope:{...ve},contactField:i,cameraProjectionChanges:m,heapBytes:S.endBytes,heap:S,pageState:{visibilityState:document.visibilityState,hasFocus:document.hasFocus(),focusLossCount:Fu,focusLossMs:_},scenario:s,browserAcceptance:{durationPass:a,onePercentLowPass:o,noLongFramePass:c,noVisibleGcPausePass:T,physicsBudgetPass:l,narrowPhaseAllocationPass:M,memoryStabilityPass:y,runtimeAssetPass:w,penetrationPass:h,lengthPass:u,foldPass:f,finitePass:d,modePass:g,contactFieldPass:x,cameraStablePass:p,focusPass:v,passed:a&&o&&T&&l&&M&&y&&w&&h&&u&&f&&d&&g&&x&&p&&v}}}function Ou(r="manual"){return r==="ui"&&yt.automated?au():(yt.running&&(yt.running=!1,yt.warmingUp=!1,yt.completedAt=performance.now(),yt.stopReason=r),Re.setAutomatedBenchmarkMode?.(!1),ia=au(),ia)}function yg({resetAccumulator:r=!0}={}){Oe.reset(),Kc=Oe.progress,ge.reset(),Le.syncFromElasticRod(qn),ge.syncXpbdBody(Hi),Ar.enabled=!1,Ls.enabled=!1,xe.resetSimulationState(),r&&(_r=0)}function Eg({durationMs:r=Tc,automated:t=!1}={}){const e=Number(r);if(!Number.isFinite(e)||e<=0)throw new RangeError("Browser benchmark durationMs must be positive");if(!xe.contactField)throw new Error("Browser benchmark requires the precompiled vessel contact field");return yg(),Bu(),yt.durationMs=e,yt.warmupStartedAt=performance.now(),yt.memorySettling=!1,yt.startedAt=0,yt.completedAt=0,yt.simulationElapsedMs=0,yt.stopReason=null,yt.automated=t===!0,Re.setAutomatedBenchmarkMode?.(yt.automated),yt.running=!0,yt.warmingUp=!0,ia=null,Jc()}function bT(r){if(!yt.running)return null;const t=performance.now();if(yt.warmingUp){const i=t-yt.warmupStartedAt;if(i<vg){const s=Cp(yt.simulationElapsedMs,bs);return yt.simulationElapsedMs+=r*1e3,s}if(yt.memorySettling||(yg({resetAccumulator:!1}),yt.memorySettling=!0,yt.simulationElapsedMs=0),i<eu)return bs.guidewireAdvance=0,bs.catheterAdvance=0,bs.catheterRotation=0,bs.catheterType="pigtail",bs;Bu(),yt.warmingUp=!1,yt.memorySettling=!1,yt.startedAt=performance.now(),yt.completedAt=0,yt.simulationElapsedMs=0}if(performance.now()-yt.startedAt>=yt.durationMs)return Ou("duration"),null;const n=Cp(yt.simulationElapsedMs,bs);return yt.simulationElapsedMs+=r*1e3,n}globalThis.__OET_BENCHMARK__={reset:Bu,getReport:au,startScenario:Eg,stopScenario:Ou,getScenarioStatus:Jc,getLastScenarioReport:()=>ia};function CT(r,t){const e=Zn==="legacy"?vn:null,n=Oe.advance(r,t,e,uT);return Kc=Oe.progress,n}function RT(){let r=0;for(let t=0;t<qn.nodes.length-1;t++){const e=qn.nodes[t],n=qn.nodes[t+1];sc.set(n.x-e.x,n.y-e.y,n.z-e.z);const i=sc.length();i<1e-6||(sc.multiplyScalar(1/i),Gp.set((e.x+n.x)*.5,(e.y+n.y)*.5,(e.z+n.z)*.5),Op.setFromUnitVectors(cT,sc),Vp.set(1,i+IA,1),Bp.compose(Gp,Op,Vp),hs.setMatrixAt(r,Bp),r++)}hs.count=r,hs.instanceMatrix.needsUpdate=!0,ta.visible=r>0}function PT(){if(!Rr||!es||Zn!=="xpbd-contact-v1")return{normalCount:0,branchCount:0};const r=Rr.geometry.getAttribute("position"),t=es.geometry.getAttribute("position"),e=r.array,n=t.array,i=vn.contactField,s=i?.centerline,a=i?.centerlineStride||0,o=a>0&&s?s.length/a:0;let c=es.userData.seen;!c||c.length!==o?(c=new Uint8Array(o),es.userData.seen=c):c.fill(0);let l=0,h=0;for(const u of[Le,Hi]){if(!u)continue;const f=Math.min(u.segmentCount,u.activeEnd);for(let d=u.activeStart;d<f;d++){if(!u.wallActive[d])continue;if(l<Fs){const p=l*6,_=2.5+Math.min(4,u.wallLambda[d]*8);e[p]=u.wallX[d],e[p+1]=u.wallY[d],e[p+2]=u.wallZ[d],e[p+3]=u.wallX[d]+u.wallNormalX[d]*_,e[p+4]=u.wallY[d]+u.wallNormalY[d]*_,e[p+5]=u.wallZ[d]+u.wallNormalZ[d]*_,l++}const g=u.wallBranchId[d];if(g<0||g>=o||c[g]||h>=Fs)continue;c[g]=1;const x=g*a,m=h*6;for(let p=0;p<6;p++)n[m+p]=s[x+p];h++}}return Rr.geometry.setDrawRange(0,l*2),es.geometry.setDrawRange(0,h*2),r.needsUpdate=!0,t.needsUpdate=!0,{normalCount:l,branchCount:h}}function LT(){if(Ce){Re.updateGuidewireDiagnostics(null),Un.count=0,zn.count=0,We.userData.hasPoint=!1,We.visible=!1,Rr?.geometry.setDrawRange(0,0),es?.geometry.setDrawRange(0,0);return}const r=Oe.collectLumenDiagnostics(vn,{clearance:Oe.meshClearance,contactBand:PA,collectMarkers:!0,markerLimit:Fs});if(Zn==="xpbd-contact-v1"){const e=xe.getStats(),n=Oe.getPerformanceStats(),i=PT();r.performance={advanceMs:n.advanceMs,solveMs:e.phases.total.lastMs,projectMs:e.phases.narrowPhase.lastMs,diagnosticMs:0,pointContactCount:e.contacts,diagnosticPointContactCount:0,segmentSampleCount:vn.contactField?.getStats?.().capsuleSamples||0,activeBranchCount:i.branchCount,settledPenetration:e.settledMaxPenetration,maximumPenetration:e.maxPenetration}}else r.performance=Oe.getPerformanceStats();Re.updateGuidewireDiagnostics(r),r.worstPoint?(We.position.set(r.worstPoint.x,r.worstPoint.y,r.worstPoint.z),We.userData.hasPoint=!0,We.visible=!0):(We.userData.hasPoint=!1,We.visible=!1);const t=(e,n)=>{const i=Math.min(n.length,Fs);e.count=i;for(let s=0;s<i;s++){const a=n[s];kp.makeTranslation(a.x,a.y,a.z),e.setMatrixAt(s,kp)}e.instanceMatrix.needsUpdate=!0};t(Un,r.contacts||[]),t(zn,r.breaches||[])}function IT(){if(Zn!=="xpbd-contact-v1"){Re.updateGuidewireResistance(0,"");return}let r=0,t=0;for(let i=0;i<Le.wallLambda.length;i++)Le.wallActive[i]&&(r+=Le.wallLambda[i],t++);const e=t?r/t:0,n=Math.max(0,Math.min(1,e/.08));Re.updateGuidewireResistance(n,t?"Opór kontaktu prowadnika ze ścianą":"")}const xr=Zn==="xpbd-contact-v1"?1/120:1/60;let Xp=performance.now(),_r=0,_h=-1/0,Rs=0;const vh=new b;let Sh=tg,Mh=eg,yh=1/0;function DT(r){const t=xn.uniforms,e=Math.min(1,Math.max(0,r)*1.35);if(!t.autoExposureEnabled.value){Rs+=(0-Rs)*Math.min(1,e*1.6),t.autoExposureLevel.value=Rs;return}Be.getWorldDirection(vh);const n=Math.abs(vh.x),i=Math.abs(vh.y),s=Math.max(0,n-.1),a=he.clamp((t.collimation.value||0)/.45,0,1),o=1-a*.34,c=.012+s*.15+i*.035,l=he.clamp(c*o-a*.006,-.03,.18);Rs+=(l-Rs)*e,t.autoExposureLevel.value=Rs}function Yp(){const r=xn.uniforms,t=he.clamp(r.pulseRate.value||15,7.5,30),e=r.autoExposureEnabled.value?he.clamp(Rs/.18,0,1):.25,n=70+e*28,i=Math.pow(t/15,.72),a=1-he.clamp((r.collimation.value||0)/.45,0,1)*.42,o=(2.4+e*7.2)*i*a;Re.updateXrayTechnique(n,o)}function NT(r=xr){const t=bT(r),e=t?.guidewireAdvance??Re.getAdvance(),n=t?.catheterAdvance??Re.getCatheterAdvance(),i=t?.catheterRotation??Re.getCatheterRotation();CT(e,r);const s=Math.max(0,Kc);if(ge.setType(t?.catheterType??Re.getSelectedCatheterType()),ge.advance(n,r,s),ge.rotate(i,r),Zn==="xpbd-contact-v1"){Le.syncFromElasticRod(qn,dT),Le.setActiveRange(Math.min(Le.count-2,Math.max(0,Oe.firstInsertedNodeIndex()-1)),Le.count-1),Le.setCollisionRange(Math.max(0,Oe.firstLumenNodeIndex()-1),Le.segmentCount-1),ge.stepPhysics(r,fT);const l=ge.syncXpbdBody(Hi,_g);Ar.outerStartNode=ge.physicsLumenStartNode;const h=Math.max(0,Math.ceil((Qo-s)/ps)),u=Math.min(Le.count-1,Math.floor((Qo-s+ge.progress)/ps));Ar.enabled=ge.progress>.5&&l>=2&&u>=h,Ar.startNode=h,Ar.endNode=Math.max(h,u);const f=Math.max(0,l-2),d=Math.max(0,Math.min(Le.segmentCount-1,u));Ls.enabled=ge.progress>4&&l>=2&&s>ge.progress+.5&&d<=Le.activeEnd-1,Ls.startSegmentA=d,Ls.endSegmentA=Math.min(Le.activeEnd-1,d+16),Ls.startSegmentB=Math.max(0,f-8),Ls.endSegmentB=f,xe.stepFixed(),yt.running&&yT(),Le.syncToElasticRod(qn)}else Oe.solve(r,vn,{iterations:e===0?3:4}),ge.stepPhysics(r);const a=n!==0||i!==0,o=e!==0,c=ge.progress>4&&s>0;if(Zn==="legacy"&&(ge.constrainGuidewire(r,{reactionScale:o&&!a?.08:1}),o&&!a&&c&&(Oe.solve(r,vn,{iterations:8,forceRelax:!0}),ge.constrainGuidewire(r,{reactionScale:.04}),Oe.solve(r,vn,{iterations:5,forceRelax:!0})),a&&(Oe.solve(r,vn,{iterations:10,forceRelax:!0}),ge.constrainGuidewire(r),Oe.solve(r,vn,{iterations:8,forceRelax:!0}))),IT(),Re.updateInsertedLength(s/10),Re.updateCatheterLength(ge.progress/10),_i){const l=Math.min(tu*r,jo);Zo.injectThroughSheath(l,tu),Up+=l,Re.updateDose(Up),Qh+=r,jo-=l,(Qh>=mg||jo<=0)&&(_i=!1,Re.setStopInjectionDisabled(!0))}Zo.update(r),oT.update(r)}const rc=[];function vr(r,t,e){rc.length=0;for(const n of r.children){if(n.isCamera)continue;!(n===e&&n.visible)&&n.visible&&(rc.push(n),n.visible=!1)}Wt.render(r,t);for(let n=0;n<rc.length;n++)rc[n].visible=!0}const oc=[],Eh=[];function FT(){Be.updateMatrixWorld(!0);const r=Be.matrixWorld.elements,t=Be.projectionMatrix.elements;let e=!Nc;for(let n=0;n<16&&!e;n++)e=r[n]!==Ip[n]||t[n]!==Dp[n];return e?(Ip.set(r),Dp.set(t),Nc=!0,!0):!1}function UT(){oc.length=0,Eh.length=0;for(const r of le.children)r!==Us&&!r.isCamera&&(oc.push(r),Eh.push(r.visible),r.visible=!1);le.overrideMaterial=VA,Wt.setRenderTarget(Ic),Wt.clear(),Wt.render(le,Be),le.overrideMaterial=kA,Wt.setRenderTarget(Dc),Wt.clear(),Wt.render(le,Be),le.overrideMaterial=null,Wt.setRenderTarget(null);for(let r=0;r<oc.length;r++)oc[r].visible=Eh[r];Kh.uniforms.frontDepth.value=Ic.texture,Kh.uniforms.backDepth.value=Dc.texture,Wt.setRenderTarget(jc),Wt.render(ug,pc),Wt.setRenderTarget(null),Wt.setRenderTarget(Zc),Wt.clear(),le.overrideMaterial=WA,vr(le,Be,Us),le.overrideMaterial=null,Wt.setRenderTarget(null)}function cu(r){const t=performance.now(),e=r-Xp,n=Math.max(0,Math.min(.1,e/1e3));Xp=r,MT(e),_r+=n;let i=0;for(;_r+1e-9>=xr&&i<2;)NT(xr),_r-=xr,i++;_r>=xr&&(_r%=xr);const s=performance.now();if(RT(),Sh+=n,Sh>=tg&&(Sh=0,LT()),Mh+=n,Mh>=eg&&(Mh=0,ge.updateMesh()),_i||Zo.hasVisibleContrast()||gh>0){xh+=n;const l=_i?1/30:1/24;if(!Di||xh>=l){xh=0;const h=ZE(Zo,.01,!Ce,Di);gh=h.count,h.mesh&&h.mesh!==Di&&(Di&&mi.remove(Di),Di=h.mesh,mi.add(Di))}}else Di&&(Di.visible=!1);Ce&&mi.parent!==Po?(le.remove(mi),Po.add(mi)):!Ce&&mi.parent!==le&&(Po.remove(mi),le.add(mi));const o=gh>0||_i||Zo.hasVisibleContrast();if(Xr.visible=!Ce,Pr.visible=Ce,Un&&(Un.visible=!Ce),zn&&(zn.visible=!Ce),We&&(We.visible=!Ce&&!!We.userData.hasPoint),Us.visible=Ce,Re.setInjectButtonDisabled(o),Re.setStopInjectionDisabled(!_i),yh+=n,yh>=.25){yh=0;const l=Jc();Re.updateBrowserBenchmarkStatus(l,l.running?null:ia)}const c=performance.now();if(Ce){DT(n),Yp();const h=1e3/Math.max(1,xn.uniforms.pulseRate.value||15);if(!(r-_h>=h)){Wt.setRenderTarget(null),Wt.render(Jh,pc),Re.updatePerfStats(n),Wp(t,s,c),requestAnimationFrame(cu);return}_h=r,FT()&&UT(),Wt.setRenderTarget(Cc),Wt.setClearColor(0,0),Wt.clear(),Wt.render(Po,Be),Wt.setClearColor(0,1),Wt.setRenderTarget(Rc),Wt.setClearColor(0,0),Wt.clear(),le.overrideMaterial=zp,vr(le,Be,ta),le.overrideMaterial=null,Wt.setClearColor(0,1),Wt.setRenderTarget(Pc),Wt.setClearColor(0,0),Wt.clear(),vr(le,Be,ge.mesh),Wt.setClearColor(0,1),Wt.setRenderTarget(Lc),Wt.setClearColor(0,0),Wt.clear(),vr(le,Be,Pr),Wt.setClearColor(0,1),Wt.setRenderTarget($h),Wt.clear(),vr(le,Be,Pr);const f=Wt.autoClear;Wt.autoClear=!1,le.overrideMaterial=zp,vr(le,Be,ta),le.overrideMaterial=null,Wt.render(Po,Be),Wt.autoClear=f,Fc.uniforms.currentFrame.value=$h.texture,Fc.uniforms.previousFrame.value=fc.texture,Wt.setRenderTarget(ic),Wt.render(lg,pc),Wt.setRenderTarget(null),xn.uniforms.uTexture.value=ic.texture,xn.uniforms.contrastTexture.value=Cc.texture,xn.uniforms.thicknessTexture.value=jc.texture,xn.uniforms.metalTexture.value=Rc.texture,xn.uniforms.catheterTexture.value=Pc.texture,xn.uniforms.sheathTexture.value=Lc.texture,xn.uniforms.boneTexture.value=Zc.texture,xn.uniforms.time.value=r*.001,Wt.render(Jh,pc),Lp();const d=fc;fc=ic,ic=d}else _h=-1/0,Yp(),Wt.setRenderTarget(null),Wt.render(le,Be),Lp();Re.updatePerfStats(n),Wp(t,s,c),requestAnimationFrame(cu)}requestAnimationFrame(cu);window.addEventListener("resize",()=>{const r=window.innerWidth,t=window.innerHeight,e=Math.max(1,Math.round(r*bc)),n=Math.max(1,Math.round(t*bc));Wt.setSize(r,t),Be.aspect=r/t,Be.updateProjectionMatrix(),$h.setSize(e,n),Cc.setSize(e,n),Rc.setSize(e,n),Pc.setSize(e,n),Lc.setSize(e,n),Zc.setSize(e,n),ag.setSize(e,n),cg.setSize(e,n),Ic.setSize(e,n),Dc.setSize(e,n),jc.setSize(e,n),Nc=!1,xn.uniforms.resolution.value.set(e,n)});
