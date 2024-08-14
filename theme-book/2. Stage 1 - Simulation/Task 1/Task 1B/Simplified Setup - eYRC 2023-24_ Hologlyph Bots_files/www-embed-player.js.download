(function(){'use strict';var m;function aa(a){var b=0;return function(){return b<a.length?{done:!1,value:a[b++]}:{done:!0}}}
var ba="function"==typeof Object.defineProperties?Object.defineProperty:function(a,b,c){if(a==Array.prototype||a==Object.prototype)return a;a[b]=c.value;return a};
function ca(a){a=["object"==typeof globalThis&&globalThis,a,"object"==typeof window&&window,"object"==typeof self&&self,"object"==typeof global&&global];for(var b=0;b<a.length;++b){var c=a[b];if(c&&c.Math==Math)return c}throw Error("Cannot find global object");}
var fa=ca(this);function u(a,b){if(b)a:{var c=fa;a=a.split(".");for(var d=0;d<a.length-1;d++){var e=a[d];if(!(e in c))break a;c=c[e]}a=a[a.length-1];d=c[a];b=b(d);b!=d&&null!=b&&ba(c,a,{configurable:!0,writable:!0,value:b})}}
u("Symbol",function(a){function b(f){if(this instanceof b)throw new TypeError("Symbol is not a constructor");return new c(d+(f||"")+"_"+e++,f)}
function c(f,g){this.h=f;ba(this,"description",{configurable:!0,writable:!0,value:g})}
if(a)return a;c.prototype.toString=function(){return this.h};
var d="jscomp_symbol_"+(1E9*Math.random()>>>0)+"_",e=0;return b});
u("Symbol.iterator",function(a){if(a)return a;a=Symbol("Symbol.iterator");for(var b="Array Int8Array Uint8Array Uint8ClampedArray Int16Array Uint16Array Int32Array Uint32Array Float32Array Float64Array".split(" "),c=0;c<b.length;c++){var d=fa[b[c]];"function"===typeof d&&"function"!=typeof d.prototype[a]&&ba(d.prototype,a,{configurable:!0,writable:!0,value:function(){return ia(aa(this))}})}return a});
function ia(a){a={next:a};a[Symbol.iterator]=function(){return this};
return a}
function ja(a){return a.raw=a}
function ka(a,b){a.raw=b;return a}
function w(a){var b="undefined"!=typeof Symbol&&Symbol.iterator&&a[Symbol.iterator];if(b)return b.call(a);if("number"==typeof a.length)return{next:aa(a)};throw Error(String(a)+" is not an iterable or ArrayLike");}
function la(a){if(!(a instanceof Array)){a=w(a);for(var b,c=[];!(b=a.next()).done;)c.push(b.value);a=c}return a}
function ma(a,b){return Object.prototype.hasOwnProperty.call(a,b)}
var oa="function"==typeof Object.assign?Object.assign:function(a,b){for(var c=1;c<arguments.length;c++){var d=arguments[c];if(d)for(var e in d)ma(d,e)&&(a[e]=d[e])}return a};
u("Object.assign",function(a){return a||oa});
var pa="function"==typeof Object.create?Object.create:function(a){function b(){}
b.prototype=a;return new b},qa=function(){function a(){function c(){}
new c;Reflect.construct(c,[],function(){});
return new c instanceof c}
if("undefined"!=typeof Reflect&&Reflect.construct){if(a())return Reflect.construct;var b=Reflect.construct;return function(c,d,e){c=b(c,d);e&&Reflect.setPrototypeOf(c,e.prototype);return c}}return function(c,d,e){void 0===e&&(e=c);
e=pa(e.prototype||Object.prototype);return Function.prototype.apply.call(c,e,d)||e}}(),ra;
if("function"==typeof Object.setPrototypeOf)ra=Object.setPrototypeOf;else{var ta;a:{var ua={a:!0},va={};try{va.__proto__=ua;ta=va.a;break a}catch(a){}ta=!1}ra=ta?function(a,b){a.__proto__=b;if(a.__proto__!==b)throw new TypeError(a+" is not extensible");return a}:null}var wa=ra;
function x(a,b){a.prototype=pa(b.prototype);a.prototype.constructor=a;if(wa)wa(a,b);else for(var c in b)if("prototype"!=c)if(Object.defineProperties){var d=Object.getOwnPropertyDescriptor(b,c);d&&Object.defineProperty(a,c,d)}else a[c]=b[c];a.Ba=b.prototype}
function xa(){this.v=!1;this.m=null;this.i=void 0;this.h=1;this.A=this.l=0;this.K=this.j=null}
function ya(a){if(a.v)throw new TypeError("Generator is already running");a.v=!0}
xa.prototype.F=function(a){this.i=a};
function za(a,b){a.j={exception:b,hd:!0};a.h=a.l||a.A}
xa.prototype.return=function(a){this.j={return:a};this.h=this.A};
xa.prototype.yield=function(a,b){this.h=b;return{value:a}};
xa.prototype.B=function(a){this.h=a};
function Aa(a,b,c){a.l=b;void 0!=c&&(a.A=c)}
function Ba(a){a.l=0;var b=a.j.exception;a.j=null;return b}
function Ca(a){var b=a.K.splice(0)[0];(b=a.j=a.j||b)?b.hd?a.h=a.l||a.A:void 0!=b.B&&a.A<b.B?(a.h=b.B,a.j=null):a.h=a.A:a.h=0}
function Da(a){this.h=new xa;this.i=a}
function Ea(a,b){ya(a.h);var c=a.h.m;if(c)return Fa(a,"return"in c?c["return"]:function(d){return{value:d,done:!0}},b,a.h.return);
a.h.return(b);return Ga(a)}
function Fa(a,b,c,d){try{var e=b.call(a.h.m,c);if(!(e instanceof Object))throw new TypeError("Iterator result "+e+" is not an object");if(!e.done)return a.h.v=!1,e;var f=e.value}catch(g){return a.h.m=null,za(a.h,g),Ga(a)}a.h.m=null;d.call(a.h,f);return Ga(a)}
function Ga(a){for(;a.h.h;)try{var b=a.i(a.h);if(b)return a.h.v=!1,{value:b.value,done:!1}}catch(c){a.h.i=void 0,za(a.h,c)}a.h.v=!1;if(a.h.j){b=a.h.j;a.h.j=null;if(b.hd)throw b.exception;return{value:b.return,done:!0}}return{value:void 0,done:!0}}
function Ha(a){this.next=function(b){ya(a.h);a.h.m?b=Fa(a,a.h.m.next,b,a.h.F):(a.h.F(b),b=Ga(a));return b};
this.throw=function(b){ya(a.h);a.h.m?b=Fa(a,a.h.m["throw"],b,a.h.F):(za(a.h,b),b=Ga(a));return b};
this.return=function(b){return Ea(a,b)};
this[Symbol.iterator]=function(){return this}}
function Ia(a){function b(d){return a.next(d)}
function c(d){return a.throw(d)}
return new Promise(function(d,e){function f(g){g.done?d(g.value):Promise.resolve(g.value).then(b,c).then(f,e)}
f(a.next())})}
function A(a){return Ia(new Ha(new Da(a)))}
function B(){for(var a=Number(this),b=[],c=a;c<arguments.length;c++)b[c-a]=arguments[c];return b}
u("Reflect",function(a){return a?a:{}});
u("Reflect.construct",function(){return qa});
u("Reflect.setPrototypeOf",function(a){return a?a:wa?function(b,c){try{return wa(b,c),!0}catch(d){return!1}}:null});
u("Promise",function(a){function b(g){this.h=0;this.j=void 0;this.i=[];this.v=!1;var h=this.l();try{g(h.resolve,h.reject)}catch(k){h.reject(k)}}
function c(){this.h=null}
function d(g){return g instanceof b?g:new b(function(h){h(g)})}
if(a)return a;c.prototype.i=function(g){if(null==this.h){this.h=[];var h=this;this.j(function(){h.A()})}this.h.push(g)};
var e=fa.setTimeout;c.prototype.j=function(g){e(g,0)};
c.prototype.A=function(){for(;this.h&&this.h.length;){var g=this.h;this.h=[];for(var h=0;h<g.length;++h){var k=g[h];g[h]=null;try{k()}catch(l){this.l(l)}}}this.h=null};
c.prototype.l=function(g){this.j(function(){throw g;})};
b.prototype.l=function(){function g(l){return function(n){k||(k=!0,l.call(h,n))}}
var h=this,k=!1;return{resolve:g(this.da),reject:g(this.A)}};
b.prototype.da=function(g){if(g===this)this.A(new TypeError("A Promise cannot resolve to itself"));else if(g instanceof b)this.ia(g);else{a:switch(typeof g){case "object":var h=null!=g;break a;case "function":h=!0;break a;default:h=!1}h?this.ba(g):this.m(g)}};
b.prototype.ba=function(g){var h=void 0;try{h=g.then}catch(k){this.A(k);return}"function"==typeof h?this.xa(h,g):this.m(g)};
b.prototype.A=function(g){this.F(2,g)};
b.prototype.m=function(g){this.F(1,g)};
b.prototype.F=function(g,h){if(0!=this.h)throw Error("Cannot settle("+g+", "+h+"): Promise already settled in state"+this.h);this.h=g;this.j=h;2===this.h&&this.ga();this.K()};
b.prototype.ga=function(){var g=this;e(function(){if(g.W()){var h=fa.console;"undefined"!==typeof h&&h.error(g.j)}},1)};
b.prototype.W=function(){if(this.v)return!1;var g=fa.CustomEvent,h=fa.Event,k=fa.dispatchEvent;if("undefined"===typeof k)return!0;"function"===typeof g?g=new g("unhandledrejection",{cancelable:!0}):"function"===typeof h?g=new h("unhandledrejection",{cancelable:!0}):(g=fa.document.createEvent("CustomEvent"),g.initCustomEvent("unhandledrejection",!1,!0,g));g.promise=this;g.reason=this.j;return k(g)};
b.prototype.K=function(){if(null!=this.i){for(var g=0;g<this.i.length;++g)f.i(this.i[g]);this.i=null}};
var f=new c;b.prototype.ia=function(g){var h=this.l();g.Yb(h.resolve,h.reject)};
b.prototype.xa=function(g,h){var k=this.l();try{g.call(h,k.resolve,k.reject)}catch(l){k.reject(l)}};
b.prototype.then=function(g,h){function k(r,t){return"function"==typeof r?function(y){try{l(r(y))}catch(v){n(v)}}:t}
var l,n,p=new b(function(r,t){l=r;n=t});
this.Yb(k(g,l),k(h,n));return p};
b.prototype.catch=function(g){return this.then(void 0,g)};
b.prototype.Yb=function(g,h){function k(){switch(l.h){case 1:g(l.j);break;case 2:h(l.j);break;default:throw Error("Unexpected state: "+l.h);}}
var l=this;null==this.i?f.i(k):this.i.push(k);this.v=!0};
b.resolve=d;b.reject=function(g){return new b(function(h,k){k(g)})};
b.race=function(g){return new b(function(h,k){for(var l=w(g),n=l.next();!n.done;n=l.next())d(n.value).Yb(h,k)})};
b.all=function(g){var h=w(g),k=h.next();return k.done?d([]):new b(function(l,n){function p(y){return function(v){r[y]=v;t--;0==t&&l(r)}}
var r=[],t=0;do r.push(void 0),t++,d(k.value).Yb(p(r.length-1),n),k=h.next();while(!k.done)})};
return b});
u("Object.setPrototypeOf",function(a){return a||wa});
u("WeakMap",function(a){function b(k){this.h=(h+=Math.random()+1).toString();if(k){k=w(k);for(var l;!(l=k.next()).done;)l=l.value,this.set(l[0],l[1])}}
function c(){}
function d(k){var l=typeof k;return"object"===l&&null!==k||"function"===l}
function e(k){if(!ma(k,g)){var l=new c;ba(k,g,{value:l})}}
function f(k){var l=Object[k];l&&(Object[k]=function(n){if(n instanceof c)return n;Object.isExtensible(n)&&e(n);return l(n)})}
if(function(){if(!a||!Object.seal)return!1;try{var k=Object.seal({}),l=Object.seal({}),n=new a([[k,2],[l,3]]);if(2!=n.get(k)||3!=n.get(l))return!1;n.delete(k);n.set(l,4);return!n.has(k)&&4==n.get(l)}catch(p){return!1}}())return a;
var g="$jscomp_hidden_"+Math.random();f("freeze");f("preventExtensions");f("seal");var h=0;b.prototype.set=function(k,l){if(!d(k))throw Error("Invalid WeakMap key");e(k);if(!ma(k,g))throw Error("WeakMap key fail: "+k);k[g][this.h]=l;return this};
b.prototype.get=function(k){return d(k)&&ma(k,g)?k[g][this.h]:void 0};
b.prototype.has=function(k){return d(k)&&ma(k,g)&&ma(k[g],this.h)};
b.prototype.delete=function(k){return d(k)&&ma(k,g)&&ma(k[g],this.h)?delete k[g][this.h]:!1};
return b});
u("Map",function(a){function b(){var h={};return h.previous=h.next=h.head=h}
function c(h,k){var l=h[1];return ia(function(){if(l){for(;l.head!=h[1];)l=l.previous;for(;l.next!=l.head;)return l=l.next,{done:!1,value:k(l)};l=null}return{done:!0,value:void 0}})}
function d(h,k){var l=k&&typeof k;"object"==l||"function"==l?f.has(k)?l=f.get(k):(l=""+ ++g,f.set(k,l)):l="p_"+k;var n=h[0][l];if(n&&ma(h[0],l))for(h=0;h<n.length;h++){var p=n[h];if(k!==k&&p.key!==p.key||k===p.key)return{id:l,list:n,index:h,entry:p}}return{id:l,list:n,index:-1,entry:void 0}}
function e(h){this[0]={};this[1]=b();this.size=0;if(h){h=w(h);for(var k;!(k=h.next()).done;)k=k.value,this.set(k[0],k[1])}}
if(function(){if(!a||"function"!=typeof a||!a.prototype.entries||"function"!=typeof Object.seal)return!1;try{var h=Object.seal({x:4}),k=new a(w([[h,"s"]]));if("s"!=k.get(h)||1!=k.size||k.get({x:4})||k.set({x:4},"t")!=k||2!=k.size)return!1;var l=k.entries(),n=l.next();if(n.done||n.value[0]!=h||"s"!=n.value[1])return!1;n=l.next();return n.done||4!=n.value[0].x||"t"!=n.value[1]||!l.next().done?!1:!0}catch(p){return!1}}())return a;
var f=new WeakMap;e.prototype.set=function(h,k){h=0===h?0:h;var l=d(this,h);l.list||(l.list=this[0][l.id]=[]);l.entry?l.entry.value=k:(l.entry={next:this[1],previous:this[1].previous,head:this[1],key:h,value:k},l.list.push(l.entry),this[1].previous.next=l.entry,this[1].previous=l.entry,this.size++);return this};
e.prototype.delete=function(h){h=d(this,h);return h.entry&&h.list?(h.list.splice(h.index,1),h.list.length||delete this[0][h.id],h.entry.previous.next=h.entry.next,h.entry.next.previous=h.entry.previous,h.entry.head=null,this.size--,!0):!1};
e.prototype.clear=function(){this[0]={};this[1]=this[1].previous=b();this.size=0};
e.prototype.has=function(h){return!!d(this,h).entry};
e.prototype.get=function(h){return(h=d(this,h).entry)&&h.value};
e.prototype.entries=function(){return c(this,function(h){return[h.key,h.value]})};
e.prototype.keys=function(){return c(this,function(h){return h.key})};
e.prototype.values=function(){return c(this,function(h){return h.value})};
e.prototype.forEach=function(h,k){for(var l=this.entries(),n;!(n=l.next()).done;)n=n.value,h.call(k,n[1],n[0],this)};
e.prototype[Symbol.iterator]=e.prototype.entries;var g=0;return e});
function Ja(a,b,c){if(null==a)throw new TypeError("The 'this' value for String.prototype."+c+" must not be null or undefined");if(b instanceof RegExp)throw new TypeError("First argument to String.prototype."+c+" must not be a regular expression");return a+""}
u("String.prototype.endsWith",function(a){return a?a:function(b,c){var d=Ja(this,b,"endsWith");b+="";void 0===c&&(c=d.length);c=Math.max(0,Math.min(c|0,d.length));for(var e=b.length;0<e&&0<c;)if(d[--c]!=b[--e])return!1;return 0>=e}});
function Ma(a,b){a instanceof String&&(a+="");var c=0,d=!1,e={next:function(){if(!d&&c<a.length){var f=c++;return{value:b(f,a[f]),done:!1}}d=!0;return{done:!0,value:void 0}}};
e[Symbol.iterator]=function(){return e};
return e}
u("Array.prototype.entries",function(a){return a?a:function(){return Ma(this,function(b,c){return[b,c]})}});
u("Array.prototype.keys",function(a){return a?a:function(){return Ma(this,function(b){return b})}});
u("String.prototype.startsWith",function(a){return a?a:function(b,c){var d=Ja(this,b,"startsWith");b+="";var e=d.length,f=b.length;c=Math.max(0,Math.min(c|0,d.length));for(var g=0;g<f&&c<e;)if(d[c++]!=b[g++])return!1;return g>=f}});
u("Number.isFinite",function(a){return a?a:function(b){return"number"!==typeof b?!1:!isNaN(b)&&Infinity!==b&&-Infinity!==b}});
u("Array.prototype.find",function(a){return a?a:function(b,c){a:{var d=this;d instanceof String&&(d=String(d));for(var e=d.length,f=0;f<e;f++){var g=d[f];if(b.call(c,g,f,d)){b=g;break a}}b=void 0}return b}});
u("Set",function(a){function b(c){this.h=new Map;if(c){c=w(c);for(var d;!(d=c.next()).done;)this.add(d.value)}this.size=this.h.size}
if(function(){if(!a||"function"!=typeof a||!a.prototype.entries||"function"!=typeof Object.seal)return!1;try{var c=Object.seal({x:4}),d=new a(w([c]));if(!d.has(c)||1!=d.size||d.add(c)!=d||1!=d.size||d.add({x:4})!=d||2!=d.size)return!1;var e=d.entries(),f=e.next();if(f.done||f.value[0]!=c||f.value[1]!=c)return!1;f=e.next();return f.done||f.value[0]==c||4!=f.value[0].x||f.value[1]!=f.value[0]?!1:e.next().done}catch(g){return!1}}())return a;
b.prototype.add=function(c){c=0===c?0:c;this.h.set(c,c);this.size=this.h.size;return this};
b.prototype.delete=function(c){c=this.h.delete(c);this.size=this.h.size;return c};
b.prototype.clear=function(){this.h.clear();this.size=0};
b.prototype.has=function(c){return this.h.has(c)};
b.prototype.entries=function(){return this.h.entries()};
b.prototype.values=function(){return this.h.values()};
b.prototype.keys=b.prototype.values;b.prototype[Symbol.iterator]=b.prototype.values;b.prototype.forEach=function(c,d){var e=this;this.h.forEach(function(f){return c.call(d,f,f,e)})};
return b});
u("Array.prototype.values",function(a){return a?a:function(){return Ma(this,function(b,c){return c})}});
u("Number.MAX_SAFE_INTEGER",function(){return 9007199254740991});
u("Number.isInteger",function(a){return a?a:function(b){return Number.isFinite(b)?b===Math.floor(b):!1}});
u("Number.isSafeInteger",function(a){return a?a:function(b){return Number.isInteger(b)&&Math.abs(b)<=Number.MAX_SAFE_INTEGER}});
u("Math.trunc",function(a){return a?a:function(b){b=Number(b);if(isNaN(b)||Infinity===b||-Infinity===b||0===b)return b;var c=Math.floor(Math.abs(b));return 0>b?-c:c}});
u("Object.values",function(a){return a?a:function(b){var c=[],d;for(d in b)ma(b,d)&&c.push(b[d]);return c}});
u("Object.is",function(a){return a?a:function(b,c){return b===c?0!==b||1/b===1/c:b!==b&&c!==c}});
u("Array.prototype.includes",function(a){return a?a:function(b,c){var d=this;d instanceof String&&(d=String(d));var e=d.length;c=c||0;for(0>c&&(c=Math.max(c+e,0));c<e;c++){var f=d[c];if(f===b||Object.is(f,b))return!0}return!1}});
u("String.prototype.includes",function(a){return a?a:function(b,c){return-1!==Ja(this,b,"includes").indexOf(b,c||0)}});
u("Number.isNaN",function(a){return a?a:function(b){return"number"===typeof b&&isNaN(b)}});
u("Array.from",function(a){return a?a:function(b,c,d){c=null!=c?c:function(h){return h};
var e=[],f="undefined"!=typeof Symbol&&Symbol.iterator&&b[Symbol.iterator];if("function"==typeof f){b=f.call(b);for(var g=0;!(f=b.next()).done;)e.push(c.call(d,f.value,g++))}else for(f=b.length,g=0;g<f;g++)e.push(c.call(d,b[g],g));return e}});
u("Object.entries",function(a){return a?a:function(b){var c=[],d;for(d in b)ma(b,d)&&c.push([d,b[d]]);return c}});
u("globalThis",function(a){return a||fa});/*

 Copyright The Closure Library Authors.
 SPDX-License-Identifier: Apache-2.0
*/
var Na=Na||{},C=this||self;function D(a,b,c){a=a.split(".");c=c||C;a[0]in c||"undefined"==typeof c.execScript||c.execScript("var "+a[0]);for(var d;a.length&&(d=a.shift());)a.length||void 0===b?c[d]&&c[d]!==Object.prototype[d]?c=c[d]:c=c[d]={}:c[d]=b}
function Oa(a){var b=E("CLOSURE_FLAGS");a=b&&b[a];return null!=a?a:!1}
function E(a,b){a=a.split(".");b=b||C;for(var c=0;c<a.length;c++)if(b=b[a[c]],null==b)return null;return b}
function Pa(a){var b=typeof a;return"object"!=b?b:a?Array.isArray(a)?"array":b:"null"}
function Qa(a){var b=Pa(a);return"array"==b||"object"==b&&"number"==typeof a.length}
function Ra(a){var b=typeof a;return"object"==b&&null!=a||"function"==b}
function Sa(a){return Object.prototype.hasOwnProperty.call(a,Ta)&&a[Ta]||(a[Ta]=++Ua)}
var Ta="closure_uid_"+(1E9*Math.random()>>>0),Ua=0;function Va(a,b,c){return a.call.apply(a.bind,arguments)}
function Wa(a,b,c){if(!a)throw Error();if(2<arguments.length){var d=Array.prototype.slice.call(arguments,2);return function(){var e=Array.prototype.slice.call(arguments);Array.prototype.unshift.apply(e,d);return a.apply(b,e)}}return function(){return a.apply(b,arguments)}}
function Xa(a,b,c){Xa=Function.prototype.bind&&-1!=Function.prototype.bind.toString().indexOf("native code")?Va:Wa;return Xa.apply(null,arguments)}
function Ya(a,b){var c=Array.prototype.slice.call(arguments,1);return function(){var d=c.slice();d.push.apply(d,arguments);return a.apply(this,d)}}
function Za(){return Date.now()}
function $a(a,b){function c(){}
c.prototype=b.prototype;a.Ba=b.prototype;a.prototype=new c;a.prototype.constructor=a;a.base=function(d,e,f){for(var g=Array(arguments.length-2),h=2;h<arguments.length;h++)g[h-2]=arguments[h];return b.prototype[e].apply(d,g)}}
function ab(a){return a}
;function bb(a,b){if(Error.captureStackTrace)Error.captureStackTrace(this,bb);else{var c=Error().stack;c&&(this.stack=c)}a&&(this.message=String(a));void 0!==b&&(this.cause=b)}
$a(bb,Error);bb.prototype.name="CustomError";function cb(a){a=a.url;var b=/[?&]dsh=1(&|$)/.test(a);this.j=!b&&/[?&]ae=1(&|$)/.test(a);this.l=!b&&/[?&]ae=2(&|$)/.test(a);if((this.h=/[?&]adurl=([^&]*)/.exec(a))&&this.h[1]){try{var c=decodeURIComponent(this.h[1])}catch(d){c=null}this.i=c}}
;var db=String.prototype.trim?function(a){return a.trim()}:function(a){return/^[\s\xa0]*([\s\S]*?)[\s\xa0]*$/.exec(a)[1]};var eb;function fb(){if(void 0===eb){var a=null,b=C.trustedTypes;if(b&&b.createPolicy){try{a=b.createPolicy("goog#html",{createHTML:ab,createScript:ab,createScriptURL:ab})}catch(c){C.console&&C.console.error(c.message)}eb=a}else eb=a}return eb}
;function gb(a,b){this.h=a===hb&&b||""}
gb.prototype.toString=function(){return this.h};
function ib(a){return new gb(hb,a)}
var hb={};ib("");function jb(a){this.h=a}
jb.prototype.toString=function(){return this.h+""};
function kb(a){if(a instanceof jb&&a.constructor===jb)return a.h;Pa(a);return"type_error:TrustedResourceUrl"}
var lb={};function mb(a){var b=fb();a=b?b.createScriptURL(a):a;return new jb(a,lb)}
;/*

 SPDX-License-Identifier: Apache-2.0
*/
var nb=ja([""]),ob=ka(["\x00"],["\\0"]),pb=ka(["\n"],["\\n"]),qb=ka(["\x00"],["\\u0000"]);function rb(a){return-1===a.toString().indexOf("`")}
rb(function(a){return a(nb)})||rb(function(a){return a(ob)})||rb(function(a){return a(pb)})||rb(function(a){return a(qb)});function sb(a){this.h=a}
sb.prototype.toString=function(){return this.h};
var tb=new sb("about:invalid#zClosurez");function ub(a){this.oe=a}
function vb(a){return new ub(function(b){return b.substr(0,a.length+1).toLowerCase()===a+":"})}
var wb=[vb("data"),vb("http"),vb("https"),vb("mailto"),vb("ftp"),new ub(function(a){return/^[^:]*([/?#]|$)/.test(a)})],xb=/^\s*(?!javascript:)(?:[\w+.-]+:|[^:/?#]*(?:[/?#]|$))/i;
function yb(a){if(a instanceof sb)if(a instanceof sb)a=a.h;else throw Error("");else a=xb.test(a)?a:void 0;return a}
;function zb(a,b){b=yb(b);void 0!==b&&(a.href=b)}
;function Ab(){this.h=Bb[0].toLowerCase()}
Ab.prototype.toString=function(){return this.h};var Cb=Array.prototype.indexOf?function(a,b){return Array.prototype.indexOf.call(a,b,void 0)}:function(a,b){if("string"===typeof a)return"string"!==typeof b||1!=b.length?-1:a.indexOf(b,0);
for(var c=0;c<a.length;c++)if(c in a&&a[c]===b)return c;return-1},Db=Array.prototype.forEach?function(a,b,c){Array.prototype.forEach.call(a,b,c)}:function(a,b,c){for(var d=a.length,e="string"===typeof a?a.split(""):a,f=0;f<d;f++)f in e&&b.call(c,e[f],f,a)},Eb=Array.prototype.filter?function(a,b){return Array.prototype.filter.call(a,b,void 0)}:function(a,b){for(var c=a.length,d=[],e=0,f="string"===typeof a?a.split(""):a,g=0;g<c;g++)if(g in f){var h=f[g];
b.call(void 0,h,g,a)&&(d[e++]=h)}return d},Fb=Array.prototype.map?function(a,b){return Array.prototype.map.call(a,b,void 0)}:function(a,b){for(var c=a.length,d=Array(c),e="string"===typeof a?a.split(""):a,f=0;f<c;f++)f in e&&(d[f]=b.call(void 0,e[f],f,a));
return d},Gb=Array.prototype.reduce?function(a,b,c){return Array.prototype.reduce.call(a,b,c)}:function(a,b,c){var d=c;
Db(a,function(e,f){d=b.call(void 0,d,e,f,a)});
return d};
function Hb(a,b){a:{for(var c=a.length,d="string"===typeof a?a.split(""):a,e=0;e<c;e++)if(e in d&&b.call(void 0,d[e],e,a)){b=e;break a}b=-1}return 0>b?null:"string"===typeof a?a.charAt(b):a[b]}
function Ib(a,b){b=Cb(a,b);var c;(c=0<=b)&&Array.prototype.splice.call(a,b,1);return c}
function Jb(a,b){for(var c=1;c<arguments.length;c++){var d=arguments[c];if(Qa(d)){var e=a.length||0,f=d.length||0;a.length=e+f;for(var g=0;g<f;g++)a[e+g]=d[g]}else a.push(d)}}
;function Kb(a,b){for(var c in a)b.call(void 0,a[c],c,a)}
function Lb(a){var b=Mb,c;for(c in b)if(a.call(void 0,b[c],c,b))return c}
function Nb(a){for(var b in a)return!1;return!0}
function Ob(a,b){if(null!==a&&b in a)throw Error('The object already contains the key "'+b+'"');a[b]=!0}
function Pb(a){return null!==a&&"privembed"in a?a.privembed:!1}
function Qb(a,b){for(var c in a)if(!(c in b)||a[c]!==b[c])return!1;for(var d in b)if(!(d in a))return!1;return!0}
function Rb(a){var b={},c;for(c in a)b[c]=a[c];return b}
function Sb(a){if(!a||"object"!==typeof a)return a;if("function"===typeof a.clone)return a.clone();if("undefined"!==typeof Map&&a instanceof Map)return new Map(a);if("undefined"!==typeof Set&&a instanceof Set)return new Set(a);if(a instanceof Date)return new Date(a.getTime());var b=Array.isArray(a)?[]:"function"!==typeof ArrayBuffer||"function"!==typeof ArrayBuffer.isView||!ArrayBuffer.isView(a)||a instanceof DataView?{}:new a.constructor(a.length),c;for(c in a)b[c]=Sb(a[c]);return b}
var Tb="constructor hasOwnProperty isPrototypeOf propertyIsEnumerable toLocaleString toString valueOf".split(" ");function Ub(a,b){for(var c,d,e=1;e<arguments.length;e++){d=arguments[e];for(c in d)a[c]=d[c];for(var f=0;f<Tb.length;f++)c=Tb[f],Object.prototype.hasOwnProperty.call(d,c)&&(a[c]=d[c])}}
;function Vb(a){this.h=a}
Vb.prototype.toString=function(){return this.h.toString()};function Wb(a){var b="true".toString(),c=[new Ab];if(0===c.length)throw Error("");if(c.map(function(d){if(d instanceof Ab)d=d.h;else throw Error("");return d}).every(function(d){return 0!=="data-loaded".indexOf(d)}))throw Error('Attribute "data-loaded" does not match any of the allowed prefixes.');
a.setAttribute("data-loaded",b)}
;function Xb(a,b){throw Error(void 0===b?"unexpected value "+a+"!":b);}
;var Yb="alternate author bookmark canonical cite help icon license modulepreload next prefetch dns-prefetch prerender preconnect preload prev search subresource".split(" ");function Zb(a,b){if(b instanceof jb)a.href=kb(b).toString();else{if(-1===Yb.indexOf("stylesheet"))throw Error('TrustedResourceUrl href attribute required with rel="stylesheet"');b=yb(b);if(void 0===b)return;a.href=b}a.rel="stylesheet"}
;function $b(a){var b,c;return(a=null==(c=(b=a.document).querySelector)?void 0:c.call(b,"script[nonce]"))?a.nonce||a.getAttribute("nonce")||"":""}
;function ac(a){this.h=a}
ac.prototype.toString=function(){return this.h.toString()};function bc(a){var b=$b(a.ownerDocument&&a.ownerDocument.defaultView||window);b&&a.setAttribute("nonce",b)}
function cc(a,b){if(b instanceof ac)b=b.h;else throw Error("");a.textContent=b;bc(a)}
function dc(a,b){a.src=kb(b);bc(a)}
;function ec(a,b){a.__closure__error__context__984382||(a.__closure__error__context__984382={});a.__closure__error__context__984382.severity=b}
;function fc(a){var b=E("window.location.href");null==a&&(a='Unknown Error of type "null/undefined"');if("string"===typeof a)return{message:a,name:"Unknown error",lineNumber:"Not available",fileName:b,stack:"Not available"};var c=!1;try{var d=a.lineNumber||a.line||"Not available"}catch(g){d="Not available",c=!0}try{var e=a.fileName||a.filename||a.sourceURL||C.$googDebugFname||b}catch(g){e="Not available",c=!0}b=hc(a);if(!(!c&&a.lineNumber&&a.fileName&&a.stack&&a.message&&a.name)){c=a.message;if(null==
c){if(a.constructor&&a.constructor instanceof Function){if(a.constructor.name)c=a.constructor.name;else if(c=a.constructor,ic[c])c=ic[c];else{c=String(c);if(!ic[c]){var f=/function\s+([^\(]+)/m.exec(c);ic[c]=f?f[1]:"[Anonymous]"}c=ic[c]}c='Unknown Error of type "'+c+'"'}else c="Unknown Error of unknown type";"function"===typeof a.toString&&Object.prototype.toString!==a.toString&&(c+=": "+a.toString())}return{message:c,name:a.name||"UnknownError",lineNumber:d,fileName:e,stack:b||"Not available"}}return{message:a.message,
name:a.name,lineNumber:a.lineNumber,fileName:a.fileName,stack:b}}
function hc(a,b){b||(b={});b[jc(a)]=!0;var c=a.stack||"";(a=a.cause)&&!b[jc(a)]&&(c+="\nCaused by: ",a.stack&&0==a.stack.indexOf(a.toString())||(c+="string"===typeof a?a:a.message+"\n"),c+=hc(a,b));return c}
function jc(a){var b="";"function"===typeof a.toString&&(b=""+a);return b+a.stack}
var ic={};function kc(a){for(var b=0,c=0;c<a.length;++c)b=31*b+a.charCodeAt(c)>>>0;return b}
;var lc=RegExp("^(?:([^:/?#.]+):)?(?://(?:([^\\\\/?#]*)@)?([^\\\\/?#]*?)(?::([0-9]+))?(?=[\\\\/?#]|$))?([^?#]+)?(?:\\?([^#]*))?(?:#([\\s\\S]*))?$");function mc(a){return a?decodeURI(a):a}
function nc(a,b){return b.match(lc)[a]||null}
function oc(a){return mc(nc(3,a))}
function pc(a){var b=a.match(lc);a=b[5];var c=b[6];b=b[7];var d="";a&&(d+=a);c&&(d+="?"+c);b&&(d+="#"+b);return d}
function qc(a){var b=a.indexOf("#");return 0>b?a:a.slice(0,b)}
function rc(a,b,c){if(Array.isArray(b))for(var d=0;d<b.length;d++)rc(a,String(b[d]),c);else null!=b&&c.push(a+(""===b?"":"="+encodeURIComponent(String(b))))}
function sc(a){var b=[],c;for(c in a)rc(c,a[c],b);return b.join("&")}
function tc(a,b){b=sc(b);if(b){var c=a.indexOf("#");0>c&&(c=a.length);var d=a.indexOf("?");if(0>d||d>c){d=c;var e=""}else e=a.substring(d+1,c);a=[a.slice(0,d),e,a.slice(c)];c=a[1];a[1]=b?c?c+"&"+b:b:c;b=a[0]+(a[1]?"?"+a[1]:"")+a[2]}else b=a;return b}
function uc(a,b,c,d){for(var e=c.length;0<=(b=a.indexOf(c,b))&&b<d;){var f=a.charCodeAt(b-1);if(38==f||63==f)if(f=a.charCodeAt(b+e),!f||61==f||38==f||35==f)return b;b+=e+1}return-1}
var vc=/#|$/,wc=/[?&]($|#)/;function xc(a,b){for(var c=a.search(vc),d=0,e,f=[];0<=(e=uc(a,d,b,c));)f.push(a.substring(d,e)),d=Math.min(a.indexOf("&",e)+1||c,c);f.push(a.slice(d));return f.join("").replace(wc,"$1")}
;function yc(a){this.h=a}
;function zc(a,b,c){this.l=a;this.j=b;this.fields=c||[];this.h=new Map}
m=zc.prototype;m.Jd=function(a){var b=B.apply(1,arguments),c=this.xc(b);c?c.push(new yc(a)):this.vd(a,b)};
m.vd=function(a){var b=this.Rc(B.apply(1,arguments));this.h.set(b,[new yc(a)])};
m.xc=function(){var a=this.Rc(B.apply(0,arguments));return this.h.has(a)?this.h.get(a):void 0};
m.be=function(){var a=this.xc(B.apply(0,arguments));return a&&a.length?a[0]:void 0};
m.clear=function(){this.h.clear()};
m.Rc=function(){var a=B.apply(0,arguments);return a?a.join(","):"key"};function Ac(a,b){zc.call(this,a,3,b)}
x(Ac,zc);Ac.prototype.i=function(a){var b=B.apply(1,arguments),c=0,d=this.be(b);d&&(c=d.h);this.vd(c+a,b)};function Bc(a,b){zc.call(this,a,2,b)}
x(Bc,zc);Bc.prototype.record=function(a){this.Jd(a,B.apply(1,arguments))};function Cc(a){a&&"function"==typeof a.dispose&&a.dispose()}
;function Dc(a){for(var b=0,c=arguments.length;b<c;++b){var d=arguments[b];Qa(d)?Dc.apply(null,d):Cc(d)}}
;function F(){this.V=this.V;this.A=this.A}
F.prototype.V=!1;F.prototype.dispose=function(){this.V||(this.V=!0,this.U())};
function Ec(a,b){a.addOnDisposeCallback(Ya(Cc,b))}
F.prototype.addOnDisposeCallback=function(a,b){this.V?void 0!==b?a.call(b):a():(this.A||(this.A=[]),this.A.push(void 0!==b?Xa(a,b):a))};
F.prototype.U=function(){if(this.A)for(;this.A.length;)this.A.shift()()};function Fc(a,b){this.type=a;this.h=this.target=b;this.defaultPrevented=this.j=!1}
Fc.prototype.stopPropagation=function(){this.j=!0};
Fc.prototype.preventDefault=function(){this.defaultPrevented=!0};var Gc=function(){if(!C.addEventListener||!Object.defineProperty)return!1;var a=!1,b=Object.defineProperty({},"passive",{get:function(){a=!0}});
try{var c=function(){};
C.addEventListener("test",c,b);C.removeEventListener("test",c,b)}catch(d){}return a}();var Hc=Oa(610401301),Ic=Oa(188588736);function Jc(){var a=C.navigator;return a&&(a=a.userAgent)?a:""}
var Kc,Lc=C.navigator;Kc=Lc?Lc.userAgentData||null:null;function Mc(a){return Hc?Kc?Kc.brands.some(function(b){return(b=b.brand)&&-1!=b.indexOf(a)}):!1:!1}
function H(a){return-1!=Jc().indexOf(a)}
;function Nc(){return Hc?!!Kc&&0<Kc.brands.length:!1}
function Oc(){return Nc()?!1:H("Opera")}
function Pc(){return H("Firefox")||H("FxiOS")}
function Qc(){return Nc()?Mc("Chromium"):(H("Chrome")||H("CriOS"))&&!(Nc()?0:H("Edge"))||H("Silk")}
;function Rc(){return Hc?!!Kc&&!!Kc.platform:!1}
function Sc(){return H("iPhone")&&!H("iPod")&&!H("iPad")}
;function Tc(a){Tc[" "](a);return a}
Tc[" "]=function(){};var Uc=Oc(),Vc=Nc()?!1:H("Trident")||H("MSIE"),Wc=H("Edge"),Xc=H("Gecko")&&!(-1!=Jc().toLowerCase().indexOf("webkit")&&!H("Edge"))&&!(H("Trident")||H("MSIE"))&&!H("Edge"),Yc=-1!=Jc().toLowerCase().indexOf("webkit")&&!H("Edge");Yc&&H("Mobile");Rc()||H("Macintosh");Rc()||H("Windows");(Rc()?"Linux"===Kc.platform:H("Linux"))||Rc()||H("CrOS");var Zc=Rc()?"Android"===Kc.platform:H("Android");Sc();H("iPad");H("iPod");Sc()||H("iPad")||H("iPod");Jc().toLowerCase().indexOf("kaios");function $c(a,b){Fc.call(this,a?a.type:"");this.relatedTarget=this.h=this.target=null;this.button=this.screenY=this.screenX=this.clientY=this.clientX=0;this.key="";this.charCode=this.keyCode=0;this.metaKey=this.shiftKey=this.altKey=this.ctrlKey=!1;this.state=null;this.pointerId=0;this.pointerType="";this.i=null;a&&this.init(a,b)}
$a($c,Fc);var ad={2:"touch",3:"pen",4:"mouse"};
$c.prototype.init=function(a,b){var c=this.type=a.type,d=a.changedTouches&&a.changedTouches.length?a.changedTouches[0]:null;this.target=a.target||a.srcElement;this.h=b;if(b=a.relatedTarget){if(Xc){a:{try{Tc(b.nodeName);var e=!0;break a}catch(f){}e=!1}e||(b=null)}}else"mouseover"==c?b=a.fromElement:"mouseout"==c&&(b=a.toElement);this.relatedTarget=b;d?(this.clientX=void 0!==d.clientX?d.clientX:d.pageX,this.clientY=void 0!==d.clientY?d.clientY:d.pageY,this.screenX=d.screenX||0,this.screenY=d.screenY||
0):(this.clientX=void 0!==a.clientX?a.clientX:a.pageX,this.clientY=void 0!==a.clientY?a.clientY:a.pageY,this.screenX=a.screenX||0,this.screenY=a.screenY||0);this.button=a.button;this.keyCode=a.keyCode||0;this.key=a.key||"";this.charCode=a.charCode||("keypress"==c?a.keyCode:0);this.ctrlKey=a.ctrlKey;this.altKey=a.altKey;this.shiftKey=a.shiftKey;this.metaKey=a.metaKey;this.pointerId=a.pointerId||0;this.pointerType="string"===typeof a.pointerType?a.pointerType:ad[a.pointerType]||"";this.state=a.state;
this.i=a;a.defaultPrevented&&$c.Ba.preventDefault.call(this)};
$c.prototype.stopPropagation=function(){$c.Ba.stopPropagation.call(this);this.i.stopPropagation?this.i.stopPropagation():this.i.cancelBubble=!0};
$c.prototype.preventDefault=function(){$c.Ba.preventDefault.call(this);var a=this.i;a.preventDefault?a.preventDefault():a.returnValue=!1};var bd="closure_listenable_"+(1E6*Math.random()|0);var cd=0;function dd(a,b,c,d,e){this.listener=a;this.proxy=null;this.src=b;this.type=c;this.capture=!!d;this.ec=e;this.key=++cd;this.Ob=this.Xb=!1}
function ed(a){a.Ob=!0;a.listener=null;a.proxy=null;a.src=null;a.ec=null}
;function fd(a){this.src=a;this.listeners={};this.h=0}
fd.prototype.add=function(a,b,c,d,e){var f=a.toString();a=this.listeners[f];a||(a=this.listeners[f]=[],this.h++);var g=gd(a,b,d,e);-1<g?(b=a[g],c||(b.Xb=!1)):(b=new dd(b,this.src,f,!!d,e),b.Xb=c,a.push(b));return b};
fd.prototype.remove=function(a,b,c,d){a=a.toString();if(!(a in this.listeners))return!1;var e=this.listeners[a];b=gd(e,b,c,d);return-1<b?(ed(e[b]),Array.prototype.splice.call(e,b,1),0==e.length&&(delete this.listeners[a],this.h--),!0):!1};
function hd(a,b){var c=b.type;c in a.listeners&&Ib(a.listeners[c],b)&&(ed(b),0==a.listeners[c].length&&(delete a.listeners[c],a.h--))}
function gd(a,b,c,d){for(var e=0;e<a.length;++e){var f=a[e];if(!f.Ob&&f.listener==b&&f.capture==!!c&&f.ec==d)return e}return-1}
;var id="closure_lm_"+(1E6*Math.random()|0),jd={},kd=0;function ld(a,b,c,d,e){if(d&&d.once)md(a,b,c,d,e);else if(Array.isArray(b))for(var f=0;f<b.length;f++)ld(a,b[f],c,d,e);else c=nd(c),a&&a[bd]?a.listen(b,c,Ra(d)?!!d.capture:!!d,e):od(a,b,c,!1,d,e)}
function od(a,b,c,d,e,f){if(!b)throw Error("Invalid event type");var g=Ra(e)?!!e.capture:!!e,h=pd(a);h||(a[id]=h=new fd(a));c=h.add(b,c,d,g,f);if(!c.proxy){d=qd();c.proxy=d;d.src=a;d.listener=c;if(a.addEventListener)Gc||(e=g),void 0===e&&(e=!1),a.addEventListener(b.toString(),d,e);else if(a.attachEvent)a.attachEvent(rd(b.toString()),d);else if(a.addListener&&a.removeListener)a.addListener(d);else throw Error("addEventListener and attachEvent are unavailable.");kd++}}
function qd(){function a(c){return b.call(a.src,a.listener,c)}
var b=sd;return a}
function md(a,b,c,d,e){if(Array.isArray(b))for(var f=0;f<b.length;f++)md(a,b[f],c,d,e);else c=nd(c),a&&a[bd]?a.h.add(String(b),c,!0,Ra(d)?!!d.capture:!!d,e):od(a,b,c,!0,d,e)}
function td(a,b,c,d,e){if(Array.isArray(b))for(var f=0;f<b.length;f++)td(a,b[f],c,d,e);else(d=Ra(d)?!!d.capture:!!d,c=nd(c),a&&a[bd])?a.h.remove(String(b),c,d,e):a&&(a=pd(a))&&(b=a.listeners[b.toString()],a=-1,b&&(a=gd(b,c,d,e)),(c=-1<a?b[a]:null)&&ud(c))}
function ud(a){if("number"!==typeof a&&a&&!a.Ob){var b=a.src;if(b&&b[bd])hd(b.h,a);else{var c=a.type,d=a.proxy;b.removeEventListener?b.removeEventListener(c,d,a.capture):b.detachEvent?b.detachEvent(rd(c),d):b.addListener&&b.removeListener&&b.removeListener(d);kd--;(c=pd(b))?(hd(c,a),0==c.h&&(c.src=null,b[id]=null)):ed(a)}}}
function rd(a){return a in jd?jd[a]:jd[a]="on"+a}
function sd(a,b){if(a.Ob)a=!0;else{b=new $c(b,this);var c=a.listener,d=a.ec||a.src;a.Xb&&ud(a);a=c.call(d,b)}return a}
function pd(a){a=a[id];return a instanceof fd?a:null}
var vd="__closure_events_fn_"+(1E9*Math.random()>>>0);function nd(a){if("function"===typeof a)return a;a[vd]||(a[vd]=function(b){return a.handleEvent(b)});
return a[vd]}
;function wd(){F.call(this);this.h=new fd(this);this.Ya=this;this.ga=null}
$a(wd,F);wd.prototype[bd]=!0;m=wd.prototype;m.addEventListener=function(a,b,c,d){ld(this,a,b,c,d)};
m.removeEventListener=function(a,b,c,d){td(this,a,b,c,d)};
function xd(a,b){var c=a.ga;if(c){var d=[];for(var e=1;c;c=c.ga)d.push(c),++e}a=a.Ya;c=b.type||b;"string"===typeof b?b=new Fc(b,a):b instanceof Fc?b.target=b.target||a:(e=b,b=new Fc(c,a),Ub(b,e));e=!0;if(d)for(var f=d.length-1;!b.j&&0<=f;f--){var g=b.h=d[f];e=yd(g,c,!0,b)&&e}b.j||(g=b.h=a,e=yd(g,c,!0,b)&&e,b.j||(e=yd(g,c,!1,b)&&e));if(d)for(f=0;!b.j&&f<d.length;f++)g=b.h=d[f],e=yd(g,c,!1,b)&&e}
m.U=function(){wd.Ba.U.call(this);this.removeAllListeners();this.ga=null};
m.listen=function(a,b,c,d){return this.h.add(String(a),b,!1,c,d)};
m.removeAllListeners=function(a){if(this.h){var b=this.h;a=a&&a.toString();var c=0,d;for(d in b.listeners)if(!a||d==a){for(var e=b.listeners[d],f=0;f<e.length;f++)++c,ed(e[f]);delete b.listeners[d];b.h--}b=c}else b=0;return b};
function yd(a,b,c,d){b=a.h.listeners[String(b)];if(!b)return!0;b=b.concat();for(var e=!0,f=0;f<b.length;++f){var g=b[f];if(g&&!g.Ob&&g.capture==c){var h=g.listener,k=g.ec||g.src;g.Xb&&hd(a.h,g);e=!1!==h.call(k,d)&&e}}return e&&!d.defaultPrevented}
;function zd(a,b){this.j=a;this.l=b;this.i=0;this.h=null}
zd.prototype.get=function(){if(0<this.i){this.i--;var a=this.h;this.h=a.next;a.next=null}else a=this.j();return a};
function Ad(a,b){a.l(b);100>a.i&&(a.i++,b.next=a.h,a.h=b)}
;function Bd(){}
function Cd(a){var b=!1,c;return function(){b||(c=a(),b=!0);return c}}
;"ARTICLE SECTION NAV ASIDE H1 H2 H3 H4 H5 H6 HEADER FOOTER ADDRESS P HR PRE BLOCKQUOTE OL UL LH LI DL DT DD FIGURE FIGCAPTION MAIN DIV EM STRONG SMALL S CITE Q DFN ABBR RUBY RB RT RTC RP DATA TIME CODE VAR SAMP KBD SUB SUP I B U MARK BDI BDO SPAN BR WBR INS DEL PICTURE PARAM TRACK MAP TABLE CAPTION COLGROUP COL TBODY THEAD TFOOT TR TD TH SELECT DATALIST OPTGROUP OPTION OUTPUT PROGRESS METER FIELDSET LEGEND DETAILS SUMMARY MENU DIALOG SLOT CANVAS FONT CENTER ACRONYM BASEFONT BIG DIR HGROUP STRIKE TT".split(" ").concat(["BUTTON",
"INPUT"]);function Dd(a,b){this.x=void 0!==a?a:0;this.y=void 0!==b?b:0}
m=Dd.prototype;m.clone=function(){return new Dd(this.x,this.y)};
m.equals=function(a){return a instanceof Dd&&(this==a?!0:this&&a?this.x==a.x&&this.y==a.y:!1)};
m.ceil=function(){this.x=Math.ceil(this.x);this.y=Math.ceil(this.y);return this};
m.floor=function(){this.x=Math.floor(this.x);this.y=Math.floor(this.y);return this};
m.round=function(){this.x=Math.round(this.x);this.y=Math.round(this.y);return this};
m.scale=function(a,b){this.x*=a;this.y*="number"===typeof b?b:a;return this};function Ed(a,b){this.width=a;this.height=b}
m=Ed.prototype;m.clone=function(){return new Ed(this.width,this.height)};
m.aspectRatio=function(){return this.width/this.height};
m.ceil=function(){this.width=Math.ceil(this.width);this.height=Math.ceil(this.height);return this};
m.floor=function(){this.width=Math.floor(this.width);this.height=Math.floor(this.height);return this};
m.round=function(){this.width=Math.round(this.width);this.height=Math.round(this.height);return this};
m.scale=function(a,b){this.width*=a;this.height*="number"===typeof b?b:a;return this};function Fd(a){var b=document;return"string"===typeof a?b.getElementById(a):a}
function Gd(a){var b=document;a=String(a);"application/xhtml+xml"===b.contentType&&(a=a.toLowerCase());return b.createElement(a)}
function Hd(a,b){for(var c=0;a;){if(b(a))return a;a=a.parentNode;c++}return null}
;var Id;function Jd(){var a=C.MessageChannel;"undefined"===typeof a&&"undefined"!==typeof window&&window.postMessage&&window.addEventListener&&!H("Presto")&&(a=function(){var e=Gd("IFRAME");e.style.display="none";document.documentElement.appendChild(e);var f=e.contentWindow;e=f.document;e.open();e.close();var g="callImmediate"+Math.random(),h="file:"==f.location.protocol?"*":f.location.protocol+"//"+f.location.host;e=Xa(function(k){if(("*"==h||k.origin==h)&&k.data==g)this.port1.onmessage()},this);
f.addEventListener("message",e,!1);this.port1={};this.port2={postMessage:function(){f.postMessage(g,h)}}});
if("undefined"!==typeof a){var b=new a,c={},d=c;b.port1.onmessage=function(){if(void 0!==c.next){c=c.next;var e=c.Vc;c.Vc=null;e()}};
return function(e){d.next={Vc:e};d=d.next;b.port2.postMessage(0)}}return function(e){C.setTimeout(e,0)}}
;function Kd(a){C.setTimeout(function(){throw a;},0)}
;function Ld(){this.i=this.h=null}
Ld.prototype.add=function(a,b){var c=Md.get();c.set(a,b);this.i?this.i.next=c:this.h=c;this.i=c};
Ld.prototype.remove=function(){var a=null;this.h&&(a=this.h,this.h=this.h.next,this.h||(this.i=null),a.next=null);return a};
var Md=new zd(function(){return new Nd},function(a){return a.reset()});
function Nd(){this.next=this.scope=this.h=null}
Nd.prototype.set=function(a,b){this.h=a;this.scope=b;this.next=null};
Nd.prototype.reset=function(){this.next=this.scope=this.h=null};var Od,Pd=!1,Qd=new Ld;function Rd(a,b){Od||Sd();Pd||(Od(),Pd=!0);Qd.add(a,b)}
function Sd(){if(C.Promise&&C.Promise.resolve){var a=C.Promise.resolve(void 0);Od=function(){a.then(Td)}}else Od=function(){var b=Td;
"function"!==typeof C.setImmediate||C.Window&&C.Window.prototype&&C.Window.prototype.setImmediate==C.setImmediate?(Id||(Id=Jd()),Id(b)):C.setImmediate(b)}}
function Td(){for(var a;a=Qd.remove();){try{a.h.call(a.scope)}catch(b){Kd(b)}Ad(Md,a)}Pd=!1}
;function Ud(a){this.h=0;this.v=void 0;this.l=this.i=this.j=null;this.A=this.m=!1;if(a!=Bd)try{var b=this;a.call(void 0,function(c){Vd(b,2,c)},function(c){Vd(b,3,c)})}catch(c){Vd(this,3,c)}}
function Wd(){this.next=this.context=this.h=this.i=this.child=null;this.j=!1}
Wd.prototype.reset=function(){this.context=this.h=this.i=this.child=null;this.j=!1};
var Xd=new zd(function(){return new Wd},function(a){a.reset()});
function Yd(a,b,c){var d=Xd.get();d.i=a;d.h=b;d.context=c;return d}
function Zd(a){return new Ud(function(b,c){c(a)})}
Ud.prototype.then=function(a,b,c){return $d(this,"function"===typeof a?a:null,"function"===typeof b?b:null,c)};
Ud.prototype.$goog_Thenable=!0;m=Ud.prototype;m.pc=function(a,b){return $d(this,null,a,b)};
m.catch=Ud.prototype.pc;m.cancel=function(a){if(0==this.h){var b=new ae(a);Rd(function(){be(this,b)},this)}};
function be(a,b){if(0==a.h)if(a.j){var c=a.j;if(c.i){for(var d=0,e=null,f=null,g=c.i;g&&(g.j||(d++,g.child==a&&(e=g),!(e&&1<d)));g=g.next)e||(f=g);e&&(0==c.h&&1==d?be(c,b):(f?(d=f,d.next==c.l&&(c.l=d),d.next=d.next.next):ce(c),de(c,e,3,b)))}a.j=null}else Vd(a,3,b)}
function ee(a,b){a.i||2!=a.h&&3!=a.h||fe(a);a.l?a.l.next=b:a.i=b;a.l=b}
function $d(a,b,c,d){var e=Yd(null,null,null);e.child=new Ud(function(f,g){e.i=b?function(h){try{var k=b.call(d,h);f(k)}catch(l){g(l)}}:f;
e.h=c?function(h){try{var k=c.call(d,h);void 0===k&&h instanceof ae?g(h):f(k)}catch(l){g(l)}}:g});
e.child.j=a;ee(a,e);return e.child}
m.gf=function(a){this.h=0;Vd(this,2,a)};
m.hf=function(a){this.h=0;Vd(this,3,a)};
function Vd(a,b,c){if(0==a.h){a===c&&(b=3,c=new TypeError("Promise cannot resolve to itself"));a.h=1;a:{var d=c,e=a.gf,f=a.hf;if(d instanceof Ud){ee(d,Yd(e||Bd,f||null,a));var g=!0}else{if(d)try{var h=!!d.$goog_Thenable}catch(l){h=!1}else h=!1;if(h)d.then(e,f,a),g=!0;else{if(Ra(d))try{var k=d.then;if("function"===typeof k){ge(d,k,e,f,a);g=!0;break a}}catch(l){f.call(a,l);g=!0;break a}g=!1}}}g||(a.v=c,a.h=b,a.j=null,fe(a),3!=b||c instanceof ae||he(a,c))}}
function ge(a,b,c,d,e){function f(k){h||(h=!0,d.call(e,k))}
function g(k){h||(h=!0,c.call(e,k))}
var h=!1;try{b.call(a,g,f)}catch(k){f(k)}}
function fe(a){a.m||(a.m=!0,Rd(a.Vd,a))}
function ce(a){var b=null;a.i&&(b=a.i,a.i=b.next,b.next=null);a.i||(a.l=null);return b}
m.Vd=function(){for(var a;a=ce(this);)de(this,a,this.h,this.v);this.m=!1};
function de(a,b,c,d){if(3==c&&b.h&&!b.j)for(;a&&a.A;a=a.j)a.A=!1;if(b.child)b.child.j=null,ie(b,c,d);else try{b.j?b.i.call(b.context):ie(b,c,d)}catch(e){je.call(null,e)}Ad(Xd,b)}
function ie(a,b,c){2==b?a.i.call(a.context,c):a.h&&a.h.call(a.context,c)}
function he(a,b){a.A=!0;Rd(function(){a.A&&je.call(null,b)})}
var je=Kd;function ae(a){bb.call(this,a)}
$a(ae,bb);ae.prototype.name="cancel";function ke(a,b){wd.call(this);this.j=a||1;this.i=b||C;this.l=Xa(this.df,this);this.m=Za()}
$a(ke,wd);m=ke.prototype;m.enabled=!1;m.Ea=null;m.setInterval=function(a){this.j=a;this.Ea&&this.enabled?(this.stop(),this.start()):this.Ea&&this.stop()};
m.df=function(){if(this.enabled){var a=Za()-this.m;0<a&&a<.8*this.j?this.Ea=this.i.setTimeout(this.l,this.j-a):(this.Ea&&(this.i.clearTimeout(this.Ea),this.Ea=null),xd(this,"tick"),this.enabled&&(this.stop(),this.start()))}};
m.start=function(){this.enabled=!0;this.Ea||(this.Ea=this.i.setTimeout(this.l,this.j),this.m=Za())};
m.stop=function(){this.enabled=!1;this.Ea&&(this.i.clearTimeout(this.Ea),this.Ea=null)};
m.U=function(){ke.Ba.U.call(this);this.stop();delete this.i};
function le(a,b,c){if("function"===typeof a)c&&(a=Xa(a,c));else if(a&&"function"==typeof a.handleEvent)a=Xa(a.handleEvent,a);else throw Error("Invalid listener argument");return 2147483647<Number(b)?-1:C.setTimeout(a,b||0)}
;function me(a){F.call(this);this.F=a;this.j=0;this.l=100;this.m=!1;this.i=new Map;this.v=new Set;this.flushInterval=3E4;this.h=new ke(this.flushInterval);this.h.listen("tick",this.Aa,!1,this);Ec(this,this.h)}
x(me,F);m=me.prototype;m.sendIsolatedPayload=function(a){this.m=a;this.l=1};
function ne(a){a.h.enabled||a.h.start();a.j++;a.j>=a.l&&a.Aa()}
m.Aa=function(){var a=this.i.values();a=[].concat(la(a)).filter(function(b){return b.h.size});
a.length&&this.F.flush(a,this.m);oe(a);this.j=0;this.h.enabled&&this.h.stop()};
m.Qa=function(a){var b=B.apply(1,arguments);this.i.has(a)||this.i.set(a,new Ac(a,b))};
m.Db=function(a){var b=B.apply(1,arguments);this.i.has(a)||this.i.set(a,new Bc(a,b))};
function pe(a,b){return a.v.has(b)?void 0:a.i.get(b)}
m.zb=function(a){this.Hd(a,1,B.apply(1,arguments))};
m.Hd=function(a,b){var c=B.apply(2,arguments),d=pe(this,a);d&&d instanceof Ac&&(d.i(b,c),ne(this))};
m.record=function(a,b){var c=B.apply(2,arguments),d=pe(this,a);d&&d instanceof Bc&&(d.record(b,c),ne(this))};
function oe(a){for(var b=0;b<a.length;b++)a[b].clear()}
;function qe(a){this.h=a;this.h.Qa("/client_streamz/bg/fic",{S:3,R:"ke"})}
function re(a){this.h=a;this.h.Qa("/client_streamz/bg/fiec",{S:3,R:"rk"},{S:3,R:"ke"},{S:2,R:"ec"})}
function se(a){this.h=a;this.h.Db("/client_streamz/bg/fil",{S:3,R:"rk"},{S:3,R:"ke"})}
se.prototype.record=function(a,b,c){this.h.record("/client_streamz/bg/fil",a,b,c)};
function te(a){this.h=a;this.h.Qa("/client_streamz/bg/fcc",{S:2,R:"ph"},{S:3,R:"ke"})}
function ue(a){this.h=a;this.h.Db("/client_streamz/bg/fcd",{S:2,R:"ph"},{S:3,R:"ke"})}
ue.prototype.record=function(a,b,c){this.h.record("/client_streamz/bg/fcd",a,b,c)};
function ve(a){this.h=a;this.h.Qa("/client_streamz/bg/fsc",{S:3,R:"rk"},{S:3,R:"ke"})}
function we(a){this.h=a;this.h.Db("/client_streamz/bg/fsl",{S:3,R:"rk"},{S:3,R:"ke"})}
we.prototype.record=function(a,b,c){this.h.record("/client_streamz/bg/fsl",a,b,c)};
function xe(a){this.h=a;this.h.Db("/client_streamz/bg/wrl",{S:3,R:"mn"},{S:2,R:"ac"},{S:2,R:"sc"},{S:3,R:"rk"})}
xe.prototype.record=function(a,b,c,d,e){this.h.record("/client_streamz/bg/wrl",a,b,c,d,e)};
function ye(a){this.h=a;this.h.Db("/client_streamz/bg/el",{S:3,R:"en"},{S:3,R:"rk"})}
ye.prototype.record=function(a,b,c){this.h.record("/client_streamz/bg/el",a,b,c)};
function ze(a){this.h=a;this.h.Qa("/client_streamz/bg/cec",{S:2,R:"ec"},{S:3,R:"rk"})}
function Ae(a){this.h=a;this.h.Qa("/client_streamz/bg/po/csc",{S:2,R:"cs"},{S:3,R:"rk"})}
function Be(a){this.h=a;this.h.Qa("/client_streamz/bg/po/ctav",{S:3,R:"av"},{S:3,R:"rk"})}
function Ce(a){this.h=a;this.h.Qa("/client_streamz/bg/po/cwsc",{S:3,R:"su"},{S:3,R:"rk"})}
;Pc();var De=Sc()||H("iPod"),Ee=H("iPad");!H("Android")||Qc()||Pc()||Oc()||H("Silk");Qc();var Fe=H("Safari")&&!(Qc()||(Nc()?0:H("Coast"))||Oc()||(Nc()?0:H("Edge"))||(Nc()?Mc("Microsoft Edge"):H("Edg/"))||(Nc()?Mc("Opera"):H("OPR"))||Pc()||H("Silk")||H("Android"))&&!(Sc()||H("iPad")||H("iPod"));var Ge={},He=null;function Ie(a,b){Qa(a);void 0===b&&(b=0);Je();b=Ge[b];for(var c=Array(Math.floor(a.length/3)),d=b[64]||"",e=0,f=0;e<a.length-2;e+=3){var g=a[e],h=a[e+1],k=a[e+2],l=b[g>>2];g=b[(g&3)<<4|h>>4];h=b[(h&15)<<2|k>>6];k=b[k&63];c[f++]=""+l+g+h+k}l=0;k=d;switch(a.length-e){case 2:l=a[e+1],k=b[(l&15)<<2]||d;case 1:a=a[e],c[f]=""+b[a>>2]+b[(a&3)<<4|l>>4]+k+d}return c.join("")}
function Ke(a){var b=a.length,c=3*b/4;c%3?c=Math.floor(c):-1!="=.".indexOf(a[b-1])&&(c=-1!="=.".indexOf(a[b-2])?c-2:c-1);var d=new Uint8Array(c),e=0;Le(a,function(f){d[e++]=f});
return e!==c?d.subarray(0,e):d}
function Le(a,b){function c(k){for(;d<a.length;){var l=a.charAt(d++),n=He[l];if(null!=n)return n;if(!/^[\s\xa0]*$/.test(l))throw Error("Unknown base64 encoding at char: "+l);}return k}
Je();for(var d=0;;){var e=c(-1),f=c(0),g=c(64),h=c(64);if(64===h&&-1===e)break;b(e<<2|f>>4);64!=g&&(b(f<<4&240|g>>2),64!=h&&b(g<<6&192|h))}}
function Je(){if(!He){He={};for(var a="ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789".split(""),b=["+/=","+/","-_=","-_.","-_"],c=0;5>c;c++){var d=a.concat(b[c].split(""));Ge[c]=d;for(var e=0;e<d.length;e++){var f=d[e];void 0===He[f]&&(He[f]=e)}}}}
;var Me="undefined"!==typeof Uint8Array,Ne=!Vc&&"function"===typeof btoa;function Oe(a){if(!Ne)return Ie(a);for(var b="",c=0,d=a.length-10240;c<d;)b+=String.fromCharCode.apply(null,a.subarray(c,c+=10240));b+=String.fromCharCode.apply(null,c?a.subarray(c):a);return btoa(b)}
var Pe=/[-_.]/g,Qe={"-":"+",_:"/",".":"="};function Re(a){return Qe[a]||""}
function Se(a){return Me&&null!=a&&a instanceof Uint8Array}
var Te={};var Ue;function Ve(a){if(a!==Te)throw Error("illegal external caller");}
function We(a,b){Ve(b);this.h=a;if(null!=a&&0===a.length)throw Error("ByteString should be constructed with non-empty values");}
We.prototype.sizeBytes=function(){Ve(Te);var a=this.h;if(null!=a&&!Se(a))if("string"===typeof a)if(Ne){Pe.test(a)&&(a=a.replace(Pe,Re));a=atob(a);for(var b=new Uint8Array(a.length),c=0;c<a.length;c++)b[c]=a.charCodeAt(c);a=b}else a=Ke(a);else Pa(a),a=null;return(a=null==a?a:this.h=a)?a.length:0};function Xe(){return"function"===typeof BigInt}
;var Ye=0,Ze=0;function $e(a){var b=0>a;a=Math.abs(a);var c=a>>>0;a=Math.floor((a-c)/4294967296);b&&(c=w(af(c,a)),b=c.next().value,a=c.next().value,c=b);Ye=c>>>0;Ze=a>>>0}
function bf(a,b){b>>>=0;a>>>=0;if(2097151>=b)var c=""+(4294967296*b+a);else Xe()?c=""+(BigInt(b)<<BigInt(32)|BigInt(a)):(c=(a>>>24|b<<8)&16777215,b=b>>16&65535,a=(a&16777215)+6777216*c+6710656*b,c+=8147497*b,b*=2,1E7<=a&&(c+=Math.floor(a/1E7),a%=1E7),1E7<=c&&(b+=Math.floor(c/1E7),c%=1E7),c=b+cf(c)+cf(a));return c}
function cf(a){a=String(a);return"0000000".slice(a.length)+a}
function df(){var a=Ye,b=Ze;b&2147483648?Xe()?a=""+(BigInt(b|0)<<BigInt(32)|BigInt(a>>>0)):(b=w(af(a,b)),a=b.next().value,b=b.next().value,a="-"+bf(a,b)):a=bf(a,b);return a}
function af(a,b){b=~b;a?a=~a+1:b+=1;return[a,b]}
;function ef(a){return Array.prototype.slice.call(a)}
;function ff(a){return"function"===typeof Symbol&&"symbol"===typeof Symbol()?Symbol():a}
var gf=ff(),hf=ff("0di"),jf=ff("2ex");Math.max.apply(Math,la(Object.values({sg:1,qg:2,pg:4,vg:8,ug:16,tg:32,yf:64,xg:128,og:256,ng:512,rg:1024,Df:2048,wg:4096,Ef:8192})));var kf=gf?function(a,b){a[gf]|=b}:function(a,b){void 0!==a.Ta?a.Ta|=b:Object.defineProperties(a,{Ta:{value:b,
configurable:!0,writable:!0,enumerable:!1}})};
function lf(a,b,c){return c?a|b:a&~b}
var mf=gf?function(a){return a[gf]|0}:function(a){return a.Ta|0},nf=gf?function(a){return a[gf]}:function(a){return a.Ta},of=gf?function(a,b){a[gf]=b;
return a}:function(a,b){void 0!==a.Ta?a.Ta=b:Object.defineProperties(a,{Ta:{value:b,
configurable:!0,writable:!0,enumerable:!1}});return a};
function pf(a){kf(a,34);return a}
function qf(a,b){of(b,(a|0)&-14591)}
function rf(a,b){of(b,(a|34)&-14557)}
function sf(a){a=a>>14&1023;return 0===a?536870912:a}
;var tf={},uf={};function vf(a){return!(!a||"object"!==typeof a||a.qe!==uf)}
function wf(a){return null!==a&&"object"===typeof a&&!Array.isArray(a)&&a.constructor===Object}
var xf;function yf(a,b,c){if(!Array.isArray(a)||a.length)return!1;var d=mf(a);if(d&1)return!0;if(!(b&&(Array.isArray(b)?b.includes(c):b.has(c))))return!1;of(a,d|1);return!0}
var zf,Af=[];of(Af,55);zf=Object.freeze(Af);function Bf(a){if(a&2)throw Error();}
function Cf(a,b,c){this.j=0;this.h=a;this.i=b;this.thisArg=c}
Cf.prototype.next=function(){if(this.j<this.h.length){var a=this.h[this.j++];return{done:!1,value:this.i?this.i.call(this.thisArg,a):a}}return{done:!0,value:void 0}};
Cf.prototype[Symbol.iterator]=function(){return new Cf(this.h,this.i,this.thisArg)};
Object.freeze(new function(){});
Object.freeze(new function(){});var Df;function Ef(a){a=Error(a);ec(a,"warning");return a}
;function Ff(a){return a.displayName||a.name||"unknown type name"}
function Gf(a){if(null!=a&&"boolean"!==typeof a)throw Error("Expected boolean but got "+Pa(a)+": "+a);return a}
var Hf=/^-?([1-9][0-9]*|0)(\.[0-9]+)?$/;function If(a){var b=typeof a;return"number"===b?Number.isFinite(a):"string"!==b?!1:Hf.test(a)}
function Jf(a){if("number"!==typeof a)throw Ef("int32");if(!Number.isFinite(a))throw Ef("int32");return a|0}
function Kf(a){return null==a?a:Jf(a)}
function Lf(a){if(null==a)return a;if("string"===typeof a){if(!a)return;a=+a}if("number"===typeof a)return Number.isFinite(a)?a|0:void 0}
function Mf(a){if(null!=a){var b=!!b;if(!If(a))throw Ef("int64");a="string"===typeof a?Nf(a):b?Of(a):Pf(a)}return a}
function Qf(a){return"-"===a[0]?20>a.length?!0:20===a.length&&-922337<Number(a.substring(0,7)):19>a.length?!0:19===a.length&&922337>Number(a.substring(0,6))}
function Pf(a){If(a);a=Math.trunc(a);if(!Number.isSafeInteger(a)){$e(a);var b=Ye,c=Ze;if(a=c&2147483648)b=~b+1>>>0,c=~c>>>0,0==b&&(c=c+1>>>0);b=4294967296*c+(b>>>0);a=a?-b:b}return a}
function Of(a){If(a);a=Math.trunc(a);if(Number.isSafeInteger(a))a=String(a);else{var b=String(a);Qf(b)?a=b:($e(a),a=df())}return a}
function Nf(a){If(a);var b=Math.trunc(Number(a));if(Number.isSafeInteger(b))return String(b);b=a.indexOf(".");-1!==b&&(a=a.substring(0,b));a.indexOf(".");if(!Qf(a)){if(16>a.length)$e(Number(a));else if(Xe())a=BigInt(a),Ye=Number(a&BigInt(4294967295))>>>0,Ze=Number(a>>BigInt(32)&BigInt(4294967295));else{b=+("-"===a[0]);Ze=Ye=0;for(var c=a.length,d=0+b,e=(c-b)%6+b;e<=c;d=e,e+=6)d=Number(a.slice(d,e)),Ze*=1E6,Ye=1E6*Ye+d,4294967296<=Ye&&(Ze+=Math.trunc(Ye/4294967296),Ze>>>=0,Ye>>>=0);b&&(b=w(af(Ye,Ze)),
a=b.next().value,b=b.next().value,Ye=a,Ze=b)}a=df()}return a}
function Rf(a){if("string"!==typeof a)throw Error();return a}
function Sf(a){if(null!=a&&"string"!==typeof a)throw Error();return a}
function Tf(a,b){if(!(a instanceof b))throw Error("Expected instanceof "+Ff(b)+" but got "+(a&&Ff(a.constructor)));}
function Uf(a,b,c,d){if(null!=a&&"object"===typeof a&&a.Ec===tf)return a;if(!Array.isArray(a))return c?d&2?(a=b[hf])?b=a:(a=new b,pf(a.D),b=b[hf]=a):b=new b:b=void 0,b;var e=c=mf(a);0===e&&(e|=d&32);e|=d&2;e!==c&&of(a,e);return new b(a)}
;var Vf;function Wf(a,b){mf(b);Vf=b;a=new a(b);Vf=void 0;return a}
function I(a,b,c){null==a&&(a=Vf);Vf=void 0;if(null==a){var d=96;c?(a=[c],d|=512):a=[];b&&(d=d&-16760833|(b&1023)<<14)}else{if(!Array.isArray(a))throw Error("narr");d=mf(a);if(d&2048)throw Error("farr");if(d&64)return a;d|=64;if(c&&(d|=512,c!==a[0]))throw Error("mid");a:{c=a;var e=c.length;if(e){var f=e-1;if(wf(c[f])){d|=256;b=f-(+!!(d&512)-1);if(1024<=b)throw Error("pvtlmt");d=d&-16760833|(b&1023)<<14;break a}}if(b){b=Math.max(b,e-(+!!(d&512)-1));if(1024<b)throw Error("spvt");d=d&-16760833|(b&1023)<<
14}}}of(a,d);return a}
;var Xf=function(){try{var a=function(){return qa(Map,[],this.constructor)};
x(a,Map);Tc(new a);return!1}catch(b){return!0}}();
function Yf(){this.h=new Map}
m=Yf.prototype;m.get=function(a){return this.h.get(a)};
m.set=function(a,b){this.h.set(a,b);this.size=this.h.size;return this};
m.delete=function(a){a=this.h.delete(a);this.size=this.h.size;return a};
m.clear=function(){this.h.clear();this.size=this.h.size};
m.has=function(a){return this.h.has(a)};
m.entries=function(){return this.h.entries()};
m.keys=function(){return this.h.keys()};
m.values=function(){return this.h.values()};
m.forEach=function(a,b){return this.h.forEach(a,b)};
Yf.prototype[Symbol.iterator]=function(){return this.entries()};
var Zf=function(){function a(){return qa(Map,[],this.constructor)}
if(Xf)return Object.setPrototypeOf(Yf.prototype,Map.prototype),Object.defineProperties(Yf.prototype,{size:{value:0,configurable:!0,enumerable:!0,writable:!0}}),Yf;x(a,Map);return a}();
function $f(a){return a}
function ag(a,b,c,d){c=void 0===c?$f:c;d=void 0===d?$f:d;var e=Zf.call(this)||this;var f=mf(a);f|=64;of(a,f);e.Tb=f;e.qc=b;e.Jb=c;e.Oc=e.qc?bg:d;for(var g=0;g<a.length;g++){var h=a[g],k=c(h[0],!1,!0),l=h[1];b?void 0===l&&(l=null):l=d(h[1],!1,!0,void 0,void 0,f);Zf.prototype.set.call(e,k,l)}return e}
x(ag,Zf);function cg(a){if(a.Tb&2)throw Error("Cannot mutate an immutable Map");}
function dg(a,b){b=void 0===b?eg:b;if(0!==a.size)return fg(a,b)}
function fg(a,b){b=void 0===b?eg:b;var c=[];a=Zf.prototype.entries.call(a);for(var d;!(d=a.next()).done;)d=d.value,d[0]=b(d[0]),d[1]=b(d[1]),c.push(d);return c}
m=ag.prototype;m.clear=function(){cg(this);Zf.prototype.clear.call(this)};
m.delete=function(a){cg(this);return Zf.prototype.delete.call(this,this.Jb(a,!0,!1))};
m.entries=function(){var a=Array.from(Zf.prototype.keys.call(this));return new Cf(a,gg,this)};
m.keys=function(){return Zf.prototype.keys.call(this)};
m.values=function(){var a=Array.from(Zf.prototype.keys.call(this));return new Cf(a,ag.prototype.get,this)};
m.forEach=function(a,b){var c=this;Zf.prototype.forEach.call(this,function(d,e){a.call(b,c.get(e),e,c)})};
m.set=function(a,b){cg(this);a=this.Jb(a,!0,!1);return null==a?this:null==b?(Zf.prototype.delete.call(this,a),this):Zf.prototype.set.call(this,a,this.Oc(b,!0,!0,this.qc,!1,this.Tb))};
m.has=function(a){return Zf.prototype.has.call(this,this.Jb(a,!1,!1))};
m.get=function(a){a=this.Jb(a,!1,!1);var b=Zf.prototype.get.call(this,a);if(void 0!==b){var c=this.qc;return c?(c=this.Oc(b,!1,!0,c,this.Cg,this.Tb),c!==b&&Zf.prototype.set.call(this,a,c),c):b}};
ag.prototype[Symbol.iterator]=function(){return this.entries()};
ag.prototype.toJSON=void 0;ag.prototype.qe=uf;function bg(a,b,c,d,e,f){b&&Tf(a,d);a=Uf(a,d,c,f);e&&(a=hg(a));f&2&&mf(a.D);return a}
function eg(a){return a}
function gg(a){return[a,this.get(a)]}
;function ig(a,b){return jg(b)}
function jg(a){switch(typeof a){case "number":return isFinite(a)?a:String(a);case "boolean":return a?1:0;case "object":if(a)if(Array.isArray(a)){if(yf(a,void 0,0))return}else{if(Se(a))return Oe(a);if(a instanceof We){var b=a.h;return null==b?"":"string"===typeof b?b:a.h=Oe(b)}if(a instanceof ag)return dg(a)}}return a}
;function kg(a,b,c){a=ef(a);var d=a.length,e=b&256?a[d-1]:void 0;d+=e?-1:0;for(b=b&512?1:0;b<d;b++)a[b]=c(a[b]);if(e){b=a[b]={};for(var f in e)b[f]=c(e[f])}return a}
function lg(a,b,c,d,e){if(null!=a){if(Array.isArray(a))a=yf(a,void 0,0)?void 0:e&&mf(a)&2?a:mg(a,b,c,void 0!==d,e);else if(wf(a)){var f={},g;for(g in a)f[g]=lg(a[g],b,c,d,e);a=f}else a=b(a,d);return a}}
function mg(a,b,c,d,e){var f=d||c?mf(a):0;d=d?!!(f&32):void 0;a=ef(a);for(var g=0;g<a.length;g++)a[g]=lg(a[g],b,c,d,e);c&&c(f,a);return a}
function ng(a){return lg(a,og,void 0,void 0,!1)}
function og(a){return a.Ec===tf?a.toJSON():a instanceof ag?dg(a,ng):jg(a)}
;function pg(a,b,c){c=void 0===c?rf:c;if(null!=a){if(Me&&a instanceof Uint8Array)return b?a:new Uint8Array(a);if(Array.isArray(a)){var d=mf(a);d&2||(b&&(b=0===d||!!(d&32)&&!(d&64||!(d&16))),a=b?of(a,(d|34)&-12293):mg(a,pg,d&4?rf:c,!0,!0));return a}a.Ec===tf?(c=a.D,d=nf(c),a=d&2?a:Wf(a.constructor,qg(c,d,!0))):a instanceof ag&&!(a.Tb&2)&&(c=pf(fg(a,pg)),a=new ag(c,a.qc,a.Jb,a.Oc));return a}}
function qg(a,b,c){var d=c||b&2?rf:qf,e=!!(b&32);a=kg(a,b,function(f){return pg(f,e,d)});
kf(a,32|(c?2:0));return a}
function hg(a){var b=a.D,c=nf(b);return c&2?Wf(a.constructor,qg(b,c,!1)):a}
;function rg(a,b){a=a.D;return sg(a,nf(a),b)}
function tg(a,b,c,d){b=d+(+!!(b&512)-1);if(!(0>b||b>=a.length||b>=c))return a[b]}
function sg(a,b,c,d){if(-1===c)return null;var e=sf(b);if(c>=e){if(b&256)return a[a.length-1][c]}else{var f=a.length;if(d&&b&256&&(d=a[f-1][c],null!=d)){if(tg(a,b,e,c)&&null!=jf){var g;a=null!=(g=Df)?g:Df={};g=a[jf]||0;4<=g||(a[jf]=g+1,g=Error(),ec(g,"incident"),Kd(g))}return d}return tg(a,b,e,c)}}
function J(a,b,c){var d=a.D,e=nf(d);Bf(e);ug(d,e,b,c);return a}
function ug(a,b,c,d,e){wf(d);var f=sf(b);if(c>=f||e){var g=b;if(b&256)e=a[a.length-1];else{if(null==d)return g;e=a[f+(+!!(b&512)-1)]={};g|=256}e[c]=d;c<f&&(a[c+(+!!(b&512)-1)]=void 0);g!==b&&of(a,g);return g}a[c+(+!!(b&512)-1)]=d;b&256&&(a=a[a.length-1],c in a&&delete a[c]);return b}
function vg(a){return void 0!==wg(a,xg,11,!1)}
function yg(a){return!!(2&a)&&!!(4&a)||!!(2048&a)}
function zg(a,b,c){var d=a.D,e=nf(d);Bf(e);if(null==b)return ug(d,e,3),a;if(!Array.isArray(b))throw Ef();var f=mf(b),g=f,h=!!(2&f)||Object.isFrozen(b),k=!h&&!1;if(!(4&f))for(f=21,h&&(b=ef(b),g=0,f=Ag(f,e),f=Bg(f,e,!0)),h=0;h<b.length;h++)b[h]=c(b[h]);k&&(b=ef(b),g=0,f=Ag(f,e),f=Bg(f,e,!0));f!==g&&of(b,f);ug(d,e,3,b);return a}
function Cg(a,b,c,d){a=a.D;var e=nf(a);Bf(e);for(var f=e,g=0,h=0;h<c.length;h++){var k=c[h];null!=sg(a,f,k)&&(0!==g&&(f=ug(a,f,g)),g=k)}(c=g)&&c!==b&&null!=d&&(e=ug(a,e,c));ug(a,e,b,d)}
function wg(a,b,c,d){a=a.D;var e=nf(a),f=sg(a,e,c,d);b=Uf(f,b,!1,e);b!==f&&null!=b&&ug(a,e,c,b,d);return b}
function Dg(a,b,c,d){d=void 0===d?!1:d;b=wg(a,b,c,d);if(null==b)return b;a=a.D;var e=nf(a);if(!(e&2)){var f=hg(b);f!==b&&(b=f,ug(a,e,c,b,d))}return b}
function Eg(a,b,c,d){null!=d?Tf(d,b):d=void 0;return J(a,c,d)}
function Fg(a,b,c,d){var e=a.D,f=nf(e);Bf(f);if(null==d)return ug(e,f,c),a;if(!Array.isArray(d))throw Ef();for(var g=mf(d),h=g,k=!!(2&g)||!!(2048&g),l=k||Object.isFrozen(d),n=!l&&!1,p=!0,r=!0,t=0;t<d.length;t++){var y=d[t];Tf(y,b);k||(y=!!(mf(y.D)&2),p&&(p=!y),r&&(r=y))}k||(g=lf(g,5,!0),g=lf(g,8,p),g=lf(g,16,r));if(n||l&&g!==h)d=ef(d),h=0,g=Ag(g,f),g=Bg(g,f,!0);g!==h&&of(d,g);ug(e,f,c,d);return a}
function Ag(a,b){a=lf(a,2,!!(2&b));a=lf(a,32,!0);return a=lf(a,2048,!1)}
function Bg(a,b,c){32&b&&c||(a=lf(a,32,!1));return a}
function Gg(a,b){a=rg(a,b);var c;null==a?c=a:If(a)?"number"===typeof a?c=Pf(a):c=Nf(a):c=void 0;return c}
function Hg(a){a=rg(a,1);var b=void 0===b?!1:b;b=null==a?a:If(a)?"string"===typeof a?Nf(a):b?Of(a):Pf(a):void 0;return b}
function Ig(a){a=rg(a,1);return null==a?a:Number.isFinite(a)?a|0:void 0}
function Jg(a,b,c){return J(a,b,Sf(c))}
function Kg(a,b,c){if(null!=c){if(!Number.isFinite(c))throw Ef("enum");c|=0}return J(a,b,c)}
;function L(a,b,c){this.D=I(a,b,c)}
m=L.prototype;m.toJSON=function(){if(xf)var a=Lg(this,this.D,!1);else a=mg(this.D,og,void 0,void 0,!1),a=Lg(this,a,!0);return a};
m.serialize=function(){xf=!0;try{return JSON.stringify(this.toJSON(),ig)}finally{xf=!1}};
function Mg(a,b){if(null==b||""==b)return new a;b=JSON.parse(b);if(!Array.isArray(b))throw Error("dnarr");kf(b,32);return Wf(a,b)}
m.clone=function(){var a=this.D,b=nf(a);return Wf(this.constructor,qg(a,b,!1))};
m.Ec=tf;m.toString=function(){return Lg(this,this.D,!1).toString()};
function Lg(a,b,c){var d=Ic?void 0:a.constructor.Pa;var e=nf(c?a.D:b);a=b.length;if(!a)return b;var f;if(wf(c=b[a-1])){a:{var g=c;var h={},k=!1,l;for(l in g){var n=g[l];if(Array.isArray(n)){var p=n;if(yf(n,d,+l)||vf(n)&&0===n.size)n=null;n!=p&&(k=!0)}null!=n?h[l]=n:k=!0}if(k){for(var r in h){g=h;break a}g=null}}g!=c&&(f=!0);a--}for(l=+!!(e&512)-1;0<a;a--){r=a-1;c=b[r];r-=l;if(!(null==c||yf(c,d,r)||vf(c)&&0===c.size))break;var t=!0}if(!f&&!t)return b;b=Array.prototype.slice.call(b,0,a);g&&b.push(g);
return b}
;function Ng(a){a.Ig=!0;return a}
;function Qg(a){this.D=I(a)}
x(Qg,L);Qg.Pa=[1,2,3,4];var Rg={toString:function(a){var b=[],c=0;a-=-2147483648;b[c++]="abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ".charAt(a%52);for(a=Math.floor(a/52);0<a;)b[c++]="abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789".charAt(a%62),a=Math.floor(a/62);return b.join("")}};function Sg(a){function b(){c-=d;c-=e;c^=e>>>13;d-=e;d-=c;d^=c<<8;e-=c;e-=d;e^=d>>>13;c-=d;c-=e;c^=e>>>12;d-=e;d-=c;d^=c<<16;e-=c;e-=d;e^=d>>>5;c-=d;c-=e;c^=e>>>3;d-=e;d-=c;d^=c<<10;e-=c;e-=d;e^=d>>>15}
a=Tg(a);for(var c=2654435769,d=2654435769,e=314159265,f=a.length,g=f,h=0;12<=g;g-=12,h+=12)c+=Ug(a,h),d+=Ug(a,h+4),e+=Ug(a,h+8),b();e+=f;switch(g){case 11:e+=a[h+10]<<24;case 10:e+=a[h+9]<<16;case 9:e+=a[h+8]<<8;case 8:d+=a[h+7]<<24;case 7:d+=a[h+6]<<16;case 6:d+=a[h+5]<<8;case 5:d+=a[h+4];case 4:c+=a[h+3]<<24;case 3:c+=a[h+2]<<16;case 2:c+=a[h+1]<<8;case 1:c+=a[h+0]}b();return Rg.toString(e)}
function Tg(a){for(var b=[],c=0;c<a.length;c++)b.push(a.charCodeAt(c));return b}
function Ug(a,b){return a[b+0]+(a[b+1]<<8)+(a[b+2]<<16)+(a[b+3]<<24)}
;function Vg(a){this.D=I(a)}
x(Vg,L);var Wg=[1,2,3];function Xg(a){this.D=I(a)}
x(Xg,L);var Yg=[1,2,3];function Zg(a){this.D=I(a)}
x(Zg,L);Zg.Pa=[1];function $g(a){this.D=I(a)}
x($g,L);$g.Pa=[3,6,4];function ah(a){this.D=I(a)}
x(ah,L);ah.Pa=[1];function bh(a){if(!a)return"";if(/^about:(?:blank|srcdoc)$/.test(a))return window.origin||"";0===a.indexOf("blob:")&&(a=a.substring(5));a=a.split("#")[0].split("?")[0];a=a.toLowerCase();0==a.indexOf("//")&&(a=window.location.protocol+a);/^[\w\-]*:\/\//.test(a)||(a=window.location.href);var b=a.substring(a.indexOf("://")+3),c=b.indexOf("/");-1!=c&&(b=b.substring(0,c));c=a.substring(0,a.indexOf("://"));if(!c)throw Error("URI is missing protocol: "+a);if("http"!==c&&"https"!==c&&"chrome-extension"!==
c&&"moz-extension"!==c&&"file"!==c&&"android-app"!==c&&"chrome-search"!==c&&"chrome-untrusted"!==c&&"chrome"!==c&&"app"!==c&&"devtools"!==c)throw Error("Invalid URI scheme in origin: "+c);a="";var d=b.indexOf(":");if(-1!=d){var e=b.substring(d+1);b=b.substring(0,d);if("http"===c&&"80"!==e||"https"===c&&"443"!==e)a=":"+e}return c+"://"+b+a}
;function ch(){function a(){e[0]=1732584193;e[1]=4023233417;e[2]=2562383102;e[3]=271733878;e[4]=3285377520;n=l=0}
function b(p){for(var r=g,t=0;64>t;t+=4)r[t/4]=p[t]<<24|p[t+1]<<16|p[t+2]<<8|p[t+3];for(t=16;80>t;t++)p=r[t-3]^r[t-8]^r[t-14]^r[t-16],r[t]=(p<<1|p>>>31)&4294967295;p=e[0];var y=e[1],v=e[2],z=e[3],G=e[4];for(t=0;80>t;t++){if(40>t)if(20>t){var K=z^y&(v^z);var N=1518500249}else K=y^v^z,N=1859775393;else 60>t?(K=y&v|z&(y|v),N=2400959708):(K=y^v^z,N=3395469782);K=((p<<5|p>>>27)&4294967295)+K+G+N+r[t]&4294967295;G=z;z=v;v=(y<<30|y>>>2)&4294967295;y=p;p=K}e[0]=e[0]+p&4294967295;e[1]=e[1]+y&4294967295;e[2]=
e[2]+v&4294967295;e[3]=e[3]+z&4294967295;e[4]=e[4]+G&4294967295}
function c(p,r){if("string"===typeof p){p=unescape(encodeURIComponent(p));for(var t=[],y=0,v=p.length;y<v;++y)t.push(p.charCodeAt(y));p=t}r||(r=p.length);t=0;if(0==l)for(;t+64<r;)b(p.slice(t,t+64)),t+=64,n+=64;for(;t<r;)if(f[l++]=p[t++],n++,64==l)for(l=0,b(f);t+64<r;)b(p.slice(t,t+64)),t+=64,n+=64}
function d(){var p=[],r=8*n;56>l?c(h,56-l):c(h,64-(l-56));for(var t=63;56<=t;t--)f[t]=r&255,r>>>=8;b(f);for(t=r=0;5>t;t++)for(var y=24;0<=y;y-=8)p[r++]=e[t]>>y&255;return p}
for(var e=[],f=[],g=[],h=[128],k=1;64>k;++k)h[k]=0;var l,n;a();return{reset:a,update:c,digest:d,Rd:function(){for(var p=d(),r="",t=0;t<p.length;t++)r+="0123456789ABCDEF".charAt(Math.floor(p[t]/16))+"0123456789ABCDEF".charAt(p[t]%16);return r}}}
;function dh(a,b,c){var d=String(C.location.href);return d&&a&&b?[b,eh(bh(d),a,c||null)].join(" "):null}
function eh(a,b,c){var d=[],e=[];if(1==(Array.isArray(c)?2:1))return e=[b,a],Db(d,function(h){e.push(h)}),fh(e.join(" "));
var f=[],g=[];Db(c,function(h){g.push(h.key);f.push(h.value)});
c=Math.floor((new Date).getTime()/1E3);e=0==f.length?[c,b,a]:[f.join(":"),c,b,a];Db(d,function(h){e.push(h)});
a=fh(e.join(" "));a=[c,a];0==g.length||a.push(g.join(""));return a.join("_")}
function fh(a){var b=ch();b.update(a);return b.Rd().toLowerCase()}
;var gh={};function hh(a){this.h=a||{cookie:""}}
m=hh.prototype;m.isEnabled=function(){if(!C.navigator.cookieEnabled)return!1;if(this.h.cookie)return!0;this.set("TESTCOOKIESENABLED","1",{Mb:60});if("1"!==this.get("TESTCOOKIESENABLED"))return!1;this.remove("TESTCOOKIESENABLED");return!0};
m.set=function(a,b,c){var d=!1;if("object"===typeof c){var e=c.Le;d=c.secure||!1;var f=c.domain||void 0;var g=c.path||void 0;var h=c.Mb}if(/[;=\s]/.test(a))throw Error('Invalid cookie name "'+a+'"');if(/[;\r\n]/.test(b))throw Error('Invalid cookie value "'+b+'"');void 0===h&&(h=-1);c=f?";domain="+f:"";g=g?";path="+g:"";d=d?";secure":"";h=0>h?"":0==h?";expires="+(new Date(1970,1,1)).toUTCString():";expires="+(new Date(Date.now()+1E3*h)).toUTCString();this.h.cookie=a+"="+b+c+g+h+d+(null!=e?";samesite="+
e:"")};
m.get=function(a,b){for(var c=a+"=",d=(this.h.cookie||"").split(";"),e=0,f;e<d.length;e++){f=db(d[e]);if(0==f.lastIndexOf(c,0))return f.slice(c.length);if(f==a)return""}return b};
m.remove=function(a,b,c){var d=void 0!==this.get(a);this.set(a,"",{Mb:0,path:b,domain:c});return d};
m.clear=function(){for(var a=(this.h.cookie||"").split(";"),b=[],c=[],d,e,f=0;f<a.length;f++)e=db(a[f]),d=e.indexOf("="),-1==d?(b.push(""),c.push(e)):(b.push(e.substring(0,d)),c.push(e.substring(d+1)));for(a=b.length-1;0<=a;a--)this.remove(b[a])};
var ih=new hh("undefined"==typeof document?null:document);function jh(a){return!!gh.FPA_SAMESITE_PHASE2_MOD||!(void 0===a||!a)}
function kh(a){a=void 0===a?!1:a;var b=C.__SAPISID||C.__APISID||C.__3PSAPISID||C.__OVERRIDE_SID;jh(a)&&(b=b||C.__1PSAPISID);if(b)return!0;if("undefined"!==typeof document){var c=new hh(document);b=c.get("SAPISID")||c.get("APISID")||c.get("__Secure-3PAPISID");jh(a)&&(b=b||c.get("__Secure-1PAPISID"))}return!!b}
function lh(a,b,c,d){(a=C[a])||"undefined"===typeof document||(a=(new hh(document)).get(b));return a?dh(a,c,d):null}
function mh(a,b){b=void 0===b?!1:b;var c=bh(String(C.location.href)),d=[];if(kh(b)){c=0==c.indexOf("https:")||0==c.indexOf("chrome-extension:")||0==c.indexOf("moz-extension:");var e=c?C.__SAPISID:C.__APISID;e||"undefined"===typeof document||(e=new hh(document),e=e.get(c?"SAPISID":"APISID")||e.get("__Secure-3PAPISID"));(e=e?dh(e,c?"SAPISIDHASH":"APISIDHASH",a):null)&&d.push(e);c&&jh(b)&&((b=lh("__1PSAPISID","__Secure-1PAPISID","SAPISID1PHASH",a))&&d.push(b),(a=lh("__3PSAPISID","__Secure-3PAPISID",
"SAPISID3PHASH",a))&&d.push(a))}return 0==d.length?null:d.join(" ")}
;function nh(a){this.D=I(a)}
x(nh,L);nh.Pa=[2];function oh(a,b){this.intervalMs=a;this.callback=b;this.enabled=!1;this.h=function(){return Za()};
this.i=this.h()}
oh.prototype.setInterval=function(a){this.intervalMs=a;this.timer&&this.enabled?(this.stop(),this.start()):this.timer&&this.stop()};
oh.prototype.start=function(){var a=this;this.enabled=!0;this.timer||(this.timer=setTimeout(function(){a.tick()},this.intervalMs),this.i=this.h())};
oh.prototype.stop=function(){this.enabled=!1;this.timer&&(clearTimeout(this.timer),this.timer=void 0)};
oh.prototype.tick=function(){var a=this;if(this.enabled){var b=Math.max(this.h()-this.i,0);b<.8*this.intervalMs?this.timer=setTimeout(function(){a.tick()},this.intervalMs-b):(this.timer&&(clearTimeout(this.timer),this.timer=void 0),this.callback(),this.enabled&&(this.stop(),this.start()))}else this.timer=void 0};function ph(a){this.D=I(a)}
x(ph,L);function qh(a){this.D=I(a)}
x(qh,L);function rh(a){this.h=this.i=this.j=a}
rh.prototype.reset=function(){this.h=this.i=this.j};
rh.prototype.getValue=function(){return this.i};function sh(a){this.D=I(a)}
x(sh,L);sh.prototype.dc=function(){return Ig(this)};function th(a){this.D=I(a)}
x(th,L);function uh(a){this.D=I(a)}
x(uh,L);function vh(a,b){Fg(a,th,1,b)}
uh.Pa=[1];function xg(a){this.D=I(a)}
x(xg,L);var wh=["platform","platformVersion","architecture","model","uaFullVersion"],xh=new uh,yh=null;function zh(a,b){b=void 0===b?wh:b;if(!yh){var c;a=null==(c=a.navigator)?void 0:c.userAgentData;if(!a||"function"!==typeof a.getHighEntropyValues||a.brands&&"function"!==typeof a.brands.map)return Promise.reject(Error("UACH unavailable"));c=(a.brands||[]).map(function(e){var f=new th;f=Jg(f,1,e.brand);return Jg(f,2,e.version)});
vh(J(xh,2,Gf(a.mobile)),c);yh=a.getHighEntropyValues(b)}var d=new Set(b);return yh.then(function(e){var f=xh.clone();d.has("platform")&&Jg(f,3,e.platform);d.has("platformVersion")&&Jg(f,4,e.platformVersion);d.has("architecture")&&Jg(f,5,e.architecture);d.has("model")&&Jg(f,6,e.model);d.has("uaFullVersion")&&Jg(f,7,e.uaFullVersion);return f}).catch(function(){return xh.clone()})}
;function Ah(a){this.D=I(a)}
x(Ah,L);function Bh(a){this.D=I(a,4)}
x(Bh,L);function Ch(a){this.D=I(a,35)}
x(Ch,L);Ch.Pa=[3,20,27];function Dh(a){this.D=I(a,19)}
x(Dh,L);Dh.prototype.Pb=function(a){return Kg(this,2,a)};
Dh.Pa=[3,5];function Eh(a){this.D=I(a,8)}
x(Eh,L);var Fh=function(a){return function(b){return Mg(a,b)}}(Eh);
Eh.Pa=[5,6,7];function Gh(a){this.D=I(a)}
x(Gh,L);var Hh;Hh=new function(a,b){this.h=a;this.ctor=b;this.isRepeated=0;this.i=Dg;this.defaultValue=void 0}(175237375,Gh);function Ih(a){F.call(this);var b=this;this.componentId="";this.j=[];this.da="";this.pageId=null;this.ga=this.W=-1;this.experimentIds=null;this.K=this.m=0;this.ia=1;this.timeoutMillis=0;this.logSource=a.logSource;this.Ib=a.Ib||function(){};
this.i=new Jh(a.logSource,a.cb);this.network=a.network;this.xb=a.xb||null;this.bufferSize=1E3;this.v=a.jf||null;this.sessionIndex=a.sessionIndex||null;this.Gb=a.Gb||!1;this.logger=null;this.withCredentials=!a.Yc;this.cb=a.cb||!1;this.F="undefined"!==typeof URLSearchParams&&!!(new URL(Kh())).searchParams&&!!(new URL(Kh())).searchParams.set;var c=Kg(new Ah,1,1);Lh(this.i,c);this.l=new rh(1E4);a=Mh(this,a.Sc);this.h=new oh(this.l.getValue(),a);this.ba=new oh(6E5,a);this.Gb||this.ba.start();this.cb||
(document.addEventListener("visibilitychange",function(){"hidden"===document.visibilityState&&b.wc()}),document.addEventListener("pagehide",this.wc.bind(this)))}
x(Ih,F);function Mh(a,b){return a.F?b?function(){b().then(function(){a.flush()})}:function(){a.flush()}:function(){}}
m=Ih.prototype;m.U=function(){this.wc();this.h.stop();this.ba.stop();F.prototype.U.call(this)};
m.log=function(a){if(this.F){a=a.clone();var b=this.ia++;a=J(a,21,Mf(b));this.componentId&&Jg(a,26,this.componentId);if(!Hg(a)){var c=Date.now();b=a;c=Number.isFinite(c)?c.toString():"0";J(b,1,Mf(c))}null==Gg(a,15)&&J(a,15,Mf(60*(new Date).getTimezoneOffset()));this.experimentIds&&(b=a,c=this.experimentIds.clone(),Eg(b,nh,16,c));b=this.j.length-this.bufferSize+1;0<b&&(this.j.splice(0,b),this.m+=b);this.j.push(a);this.Gb||this.h.enabled||this.h.start()}};
m.flush=function(a,b){var c=this;if(0===this.j.length)a&&a();else{var d=Date.now();if(this.ga>d&&this.W<d)b&&b("throttled");else{this.network&&("function"===typeof this.network.dc?Nh(this.i,this.network.dc()):Nh(this.i,0));var e=Oh(this.i,this.j,this.m,this.K,this.xb);d={};var f=this.Ib();f&&(d.Authorization=f);this.v||(this.v=Kh());try{var g=(new URL(this.v)).toString()}catch(k){g=(new URL(this.v,window.location.origin)).toString()}g=new URL(g);this.sessionIndex&&(d["X-Goog-AuthUser"]=this.sessionIndex,
g.searchParams.set("authuser",this.sessionIndex));this.pageId&&(Object.defineProperty(d,"X-Goog-PageId",{value:this.pageId}),g.searchParams.set("pageId",this.pageId));if(f&&this.da===f)b&&b("stale-auth-token");else{this.j=[];this.h.enabled&&this.h.stop();this.m=0;var h=e.serialize();d={url:g.toString(),body:h,Ag:1,rd:d,requestType:"POST",withCredentials:this.withCredentials,timeoutMillis:this.timeoutMillis};g=function(k){c.l.reset();c.h.setInterval(c.l.getValue());if(k){var l=null;try{var n=JSON.stringify(JSON.parse(k.replace(")]}'\n",
"")));l=Fh(n)}catch(r){}if(l){k=Number;n="-1";n=void 0===n?"0":n;var p=Hg(l);k=k(null!=p?p:n);0<k&&(c.W=Date.now(),c.ga=c.W+k);l=Hh.ctor?Hh.i(l,Hh.ctor,Hh.h,!0):Hh.i(l,Hh.h,null,!0);if(k=null===l?void 0:l)l=-1,l=void 0===l?0:l,k=Lf(rg(k,1)),l=null!=k?k:l,-1!==l&&(c.l=new rh(1>l?1:l),c.h.setInterval(c.l.getValue()))}}a&&a();c.K=0};
h=function(k,l){var n=e.D;var p=nf(n),r=p,t=!(2&p),y=!!(2&r);p=y?1:2;t&&(t=!y);y=sg(n,r,3);y=Array.isArray(y)?y:zf;var v=mf(y),z=!!(4&v);if(!z){var G=v;0===G&&(G=Ag(G,r));G=lf(G,1,!0);v=y;var K=r,N=!!(2&G);N&&(K=lf(K,2,!0));for(var S=!N,da=!0,sa=0,P=0;sa<v.length;sa++){var ea=Uf(v[sa],Ch,!1,K);if(ea instanceof Ch){if(!N){var na=!!(mf(ea.D)&2);S&&(S=!na);da&&(da=na)}v[P++]=ea}}P<sa&&(v.length=P);G=lf(G,4,!0);G=lf(G,16,da);G=lf(G,8,S);of(v,G);N&&Object.freeze(v);v=G}if(t&&!(8&v||!y.length&&(1===p||
4===p&&32&v))){yg(v)&&(y=ef(y),v=Ag(v,r),r=ug(n,r,3,y));t=y;for(G=0;G<t.length;G++)K=t[G],N=hg(K),K!==N&&(t[G]=N);v=lf(v,8,!0);v=lf(v,16,!t.length);of(t,v)}yg(v)||(t=v,(G=1===p||4===p&&!!(32&v))?(K=!!(32&v),v=lf(v,!y.length||16&v&&(!z||K)?2:2048,!0)):v=Bg(v,r,!1),v!==t&&of(y,v),G&&Object.freeze(y));2===p&&yg(v)&&(y=ef(y),v=Ag(v,r),v=Bg(v,r,!1),of(y,v),ug(n,r,3,y));n=y;r=Gg(e,14);p=c.l;p.h=Math.min(3E5,2*p.h);p.i=Math.min(3E5,p.h+Math.round(.2*(Math.random()-.5)*p.h));c.h.setInterval(c.l.getValue());
401===k&&f&&(c.da=f);r&&(c.m+=r);void 0===l&&(l=c.isRetryable(k));l&&(c.j=n.concat(c.j),c.Gb||c.h.enabled||c.h.start());b&&b("net-send-failed",k);++c.K};
c.network&&c.network.send(d,g,h)}}}};
m.wc=function(){Ph(this.i,!0);this.flush();Ph(this.i,!1)};
m.isRetryable=function(a){return 500<=a&&600>a||401===a||0===a};
function Kh(){return"https://play.google.com/log?format=json&hasfast=true"}
function Jh(a,b){this.cb=b=void 0===b?!1:b;this.i=this.locale=null;this.h=new Dh;Number.isInteger(a)&&this.h.Pb(a);b||(this.locale=document.documentElement.getAttribute("lang"));Lh(this,new Ah)}
Jh.prototype.Pb=function(a){this.h.Pb(a);return this};
function Lh(a,b){Eg(a.h,Ah,1,b);Ig(b)||Kg(b,1,1);if(!a.cb){b=Qh(a);var c=rg(b,5);(null==c||"string"===typeof c)&&c||Jg(b,5,a.locale)}a.i&&(b=Qh(a),Dg(b,uh,9)||Eg(b,uh,9,a.i))}
function Nh(a,b){vg(Rh(a))&&(a=Sh(a),Kg(a,1,b))}
function Ph(a,b){vg(Rh(a))&&(a=Sh(a),J(a,2,Gf(b)))}
function Rh(a){return Dg(a.h,Ah,1)}
function Th(a){var b=void 0===b?wh:b;var c=a.cb?void 0:window;c?zh(c,b).then(function(d){a.i=d;d=Qh(a);Eg(d,uh,9,a.i);return!0}).catch(function(){return!1}):Promise.resolve(!1)}
function Qh(a){a=Rh(a);var b=Dg(a,xg,11);b||(b=new xg,Eg(a,xg,11,b));return b}
function Sh(a){a=Qh(a);var b=Dg(a,sh,10);b||(b=new sh,J(b,2,Gf(!1)),Eg(a,sh,10,b));return b}
function Oh(a,b,c,d,e){var f=0,g=0;c=void 0===c?0:c;f=void 0===f?0:f;g=void 0===g?0:g;d=void 0===d?0:d;if(vg(Rh(a))){var h=Sh(a);J(h,3,Kf(d))}vg(Rh(a))&&(d=Sh(a),J(d,4,Kf(f)));vg(Rh(a))&&(f=Sh(a),J(f,5,Kf(g)));a=a.h.clone();g=Date.now().toString();a=J(a,4,Mf(g));b=Fg(a,Ch,3,b);e&&(a=new ph,e=J(a,13,Kf(e)),a=new qh,e=Eg(a,ph,2,e),a=new Bh,e=Eg(a,qh,1,e),e=Kg(e,2,9),Eg(b,Bh,18,e));c&&J(b,14,Mf(c));return b}
;function Uh(){this.Id="undefined"!==typeof AbortController}
Uh.prototype.send=function(a,b,c){var d=this,e,f,g,h,k,l,n,p,r,t,y,v;return A(function(z){switch(z.h){case 1:return f=(e=d.Id?new AbortController:void 0)?setTimeout(function(){e.abort()},a.timeoutMillis):void 0,Aa(z,2,3),g=Object.assign({},{method:a.requestType,
headers:Object.assign({},a.rd)},a.body&&{body:a.body},a.withCredentials&&{credentials:"include"},{signal:a.timeoutMillis&&e?e.signal:null}),z.yield(fetch(a.url,g),5);case 5:h=z.i;if(200!==h.status){null==(k=c)||k(h.status);z.B(3);break}if(null==(l=b)){z.B(7);break}p=n=l;return z.yield(h.text(),8);case 8:p(z.i);case 7:case 3:z.K=[z.j];z.l=0;z.A=0;clearTimeout(f);Ca(z);break;case 2:r=Ba(z);switch(null==(t=r)?void 0:t.name){case "AbortError":null==(y=c)||y(408);break;default:null==(v=c)||v(400)}z.B(3)}})};
Uh.prototype.dc=function(){return 4};function Vh(a,b){F.call(this);this.logSource=a;this.sessionIndex=b;this.j="https://play.google.com/log?format=json&hasfast=true";this.h=null;this.l=!1;this.network=null;this.componentId="";this.pageId=this.i=this.xb=null}
x(Vh,F);Vh.prototype.Yc=function(){this.m=!0;return this};
function Wh(a){a.network||(a.network=new Uh);var b=new Ih({logSource:a.logSource,Ib:a.Ib?a.Ib:mh,sessionIndex:a.sessionIndex,jf:a.j,cb:a.l,Gb:!1,Yc:a.m,Sc:a.Sc,network:a.network});Ec(a,b);if(a.h){var c=a.h,d=Qh(b.i);Jg(d,7,c)}a.componentId&&(b.componentId=a.componentId);a.xb&&(b.xb=a.xb);a.pageId&&(b.pageId=a.pageId);a.i&&((d=a.i)?(b.experimentIds||(b.experimentIds=new nh),c=b.experimentIds,d=d.serialize(),Jg(c,4,d)):b.experimentIds&&J(b.experimentIds,4));Th(b.i);a.network.Pb&&a.network.Pb(a.logSource);
a.network.Ve&&a.network.Ve(b);return b}
;function Xh(a,b,c,d,e,f,g){a=void 0===a?-1:a;b=void 0===b?"":b;c=void 0===c?"":c;d=void 0===d?!1:d;e=void 0===e?"":e;F.call(this);this.logSource=a;this.componentId=b;f?b=f:(a=new Vh(a,"0"),a.componentId=b,Ec(this,a),""!==c&&(a.j=c),d&&(a.l=!0),e&&(a.h=e),g&&(a.network=g),b=Wh(a));this.h=b}
x(Xh,F);
Xh.prototype.flush=function(a){var b=a||[];if(b.length){a=new ah;for(var c=[],d=0;d<b.length;d++){var e=b[d];var f=new $g;f=Jg(f,1,e.l);for(var g=[],h=0;h<e.fields.length;h++)g.push(e.fields[h].R);f=zg(f,g,Rf);g=[];h=[];for(var k=w(e.h.keys()),l=k.next();!l.done;l=k.next())h.push(l.value.split(","));for(k=0;k<h.length;k++){l=h[k];var n=e.j;for(var p=e.xc(l)||[],r=[],t=0;t<p.length;t++){var y=p[t],v=y&&y.h;y=new Xg;switch(n){case 3:v=Number(v);Number.isFinite(v)&&Cg(y,1,Yg,Mf(v));break;case 2:v=Number(v);
if(null!=v&&"number"!==typeof v)throw Error("Value of float/double field must be a number, found "+typeof v+": "+v);Cg(y,2,Yg,v)}r.push(y)}n=r;for(p=0;p<n.length;p++){r=n[p];t=new Zg;r=Eg(t,Xg,2,r);t=l;y=[];v=[];for(var z=0;z<e.fields.length;z++)v.push(e.fields[z].S);for(z=0;z<v.length;z++){var G=v[z],K=t[z],N=new Vg;switch(G){case 3:Cg(N,1,Wg,Sf(String(K)));break;case 2:G=Number(K);Number.isFinite(G)&&Cg(N,2,Wg,Kf(G));break;case 1:Cg(N,3,Wg,Gf("true"===K))}y.push(N)}Fg(r,Vg,1,y);g.push(r)}}Fg(f,
Zg,4,g);c.push(f);e.clear()}Fg(a,$g,1,c);b=this.h;b.F&&(a instanceof Ch?b.log(a):(c=new Ch,a=a.serialize(),a=Jg(c,8,a),b.log(a)));this.h.flush()}};function Yh(){}
Yh.prototype.serialize=function(a){var b=[];Zh(this,a,b);return b.join("")};
function Zh(a,b,c){if(null==b)c.push("null");else{if("object"==typeof b){if(Array.isArray(b)){var d=b;b=d.length;c.push("[");for(var e="",f=0;f<b;f++)c.push(e),Zh(a,d[f],c),e=",";c.push("]");return}if(b instanceof String||b instanceof Number||b instanceof Boolean)b=b.valueOf();else{c.push("{");e="";for(d in b)Object.prototype.hasOwnProperty.call(b,d)&&(f=b[d],"function"!=typeof f&&(c.push(e),$h(d,c),c.push(":"),Zh(a,f,c),e=","));c.push("}");return}}switch(typeof b){case "string":$h(b,c);break;case "number":c.push(isFinite(b)&&
!isNaN(b)?String(b):"null");break;case "boolean":c.push(String(b));break;case "function":c.push("null");break;default:throw Error("Unknown type: "+typeof b);}}}
var ai={'"':'\\"',"\\":"\\\\","/":"\\/","\b":"\\b","\f":"\\f","\n":"\\n","\r":"\\r","\t":"\\t","\v":"\\u000b"},bi=/\uffff/.test("\uffff")?/[\\"\x00-\x1f\x7f-\uffff]/g:/[\\"\x00-\x1f\x7f-\xff]/g;function $h(a,b){b.push('"',a.replace(bi,function(c){var d=ai[c];d||(d="\\u"+(c.charCodeAt(0)|65536).toString(16).slice(1),ai[c]=d);return d}),'"')}
;function ci(){}
ci.prototype.h=null;ci.prototype.getOptions=function(){var a;(a=this.h)||(a=this.h={});return a};var di;function ei(){}
$a(ei,ci);di=new ei;function fi(a){wd.call(this);this.headers=new Map;this.Fa=a||null;this.i=!1;this.K=this.T=null;this.l=this.da="";this.j=this.ba=this.m=this.W=!1;this.F=0;this.v=null;this.xa="";this.ia=!1}
$a(fi,wd);var gi=/^https?$/i,hi=["POST","PUT"],ii=[];function ji(a,b,c,d,e,f,g){var h=new fi;ii.push(h);b&&h.listen("complete",b);h.h.add("ready",h.Pd,!0,void 0,void 0);f&&(h.F=Math.max(0,f));g&&(h.ia=g);h.send(a,c,d,e)}
m=fi.prototype;m.Pd=function(){this.dispose();Ib(ii,this)};
m.send=function(a,b,c,d){if(this.T)throw Error("[goog.net.XhrIo] Object is active with another request="+this.da+"; newUri="+a);b=b?b.toUpperCase():"GET";this.da=a;this.l="";this.W=!1;this.i=!0;this.T=new XMLHttpRequest;this.K=this.Fa?this.Fa.getOptions():di.getOptions();this.T.onreadystatechange=Xa(this.ld,this);try{this.getStatus(),this.ba=!0,this.T.open(b,String(a),!0),this.ba=!1}catch(g){this.getStatus();ki(this,g);return}a=c||"";c=new Map(this.headers);if(d)if(Object.getPrototypeOf(d)===Object.prototype)for(var e in d)c.set(e,
d[e]);else if("function"===typeof d.keys&&"function"===typeof d.get){e=w(d.keys());for(var f=e.next();!f.done;f=e.next())f=f.value,c.set(f,d.get(f))}else throw Error("Unknown input type for opt_headers: "+String(d));d=Array.from(c.keys()).find(function(g){return"content-type"==g.toLowerCase()});
e=C.FormData&&a instanceof C.FormData;!(0<=Cb(hi,b))||d||e||c.set("Content-Type","application/x-www-form-urlencoded;charset=utf-8");b=w(c);for(d=b.next();!d.done;d=b.next())c=w(d.value),d=c.next().value,c=c.next().value,this.T.setRequestHeader(d,c);this.xa&&(this.T.responseType=this.xa);"withCredentials"in this.T&&this.T.withCredentials!==this.ia&&(this.T.withCredentials=this.ia);try{li(this),0<this.F&&(this.getStatus(),this.v=le(this.ff,this.F,this)),this.getStatus(),this.m=!0,this.T.send(a),this.m=
!1}catch(g){this.getStatus(),ki(this,g)}};
m.ff=function(){"undefined"!=typeof Na&&this.T&&(this.l="Timed out after "+this.F+"ms, aborting",this.getStatus(),xd(this,"timeout"),this.abort(8))};
function ki(a,b){a.i=!1;a.T&&(a.j=!0,a.T.abort(),a.j=!1);a.l=b;mi(a);ni(a)}
function mi(a){a.W||(a.W=!0,xd(a,"complete"),xd(a,"error"))}
m.abort=function(){this.T&&this.i&&(this.getStatus(),this.i=!1,this.j=!0,this.T.abort(),this.j=!1,xd(this,"complete"),xd(this,"abort"),ni(this))};
m.U=function(){this.T&&(this.i&&(this.i=!1,this.j=!0,this.T.abort(),this.j=!1),ni(this,!0));fi.Ba.U.call(this)};
m.ld=function(){this.V||(this.ba||this.m||this.j?oi(this):this.ze())};
m.ze=function(){oi(this)};
function oi(a){if(a.i&&"undefined"!=typeof Na)if(a.K[1]&&4==pi(a)&&2==a.getStatus())a.getStatus();else if(a.m&&4==pi(a))le(a.ld,0,a);else if(xd(a,"readystatechange"),a.isComplete()){a.getStatus();a.i=!1;try{if(qi(a))xd(a,"complete"),xd(a,"success");else{try{var b=2<pi(a)?a.T.statusText:""}catch(c){b=""}a.l=b+" ["+a.getStatus()+"]";mi(a)}}finally{ni(a)}}}
function ni(a,b){if(a.T){li(a);var c=a.T,d=a.K[0]?function(){}:null;
a.T=null;a.K=null;b||xd(a,"ready");try{c.onreadystatechange=d}catch(e){}}}
function li(a){a.v&&(C.clearTimeout(a.v),a.v=null)}
m.isActive=function(){return!!this.T};
m.isComplete=function(){return 4==pi(this)};
function qi(a){var b=a.getStatus();a:switch(b){case 200:case 201:case 202:case 204:case 206:case 304:case 1223:var c=!0;break a;default:c=!1}if(!c){if(b=0===b)a=nc(1,String(a.da)),!a&&C.self&&C.self.location&&(a=C.self.location.protocol.slice(0,-1)),b=!gi.test(a?a.toLowerCase():"");c=b}return c}
function pi(a){return a.T?a.T.readyState:0}
m.getStatus=function(){try{return 2<pi(this)?this.T.status:-1}catch(a){return-1}};
m.getLastError=function(){return"string"===typeof this.l?this.l:String(this.l)};function ri(){}
ri.prototype.send=function(a,b,c){b=void 0===b?function(){}:b;
c=void 0===c?function(){}:c;
ji(a.url,function(d){d=d.target;if(qi(d)){try{var e=d.T?d.T.responseText:""}catch(f){e=""}b(e)}else c(d.getStatus())},a.requestType,a.body,a.rd,a.timeoutMillis,a.withCredentials)};
ri.prototype.dc=function(){return 1};function si(a,b,c){this.logger=a;this.event=b;if(void 0===c||c)this.h=ti()}
si.prototype.start=function(){this.h=ti()};
si.prototype.done=function(){null!=this.h&&this.logger.ic(this.event,ti()-this.h)};
function ui(){}
ui.prototype.Bc=function(){};
ui.prototype.ic=function(){};
ui.prototype.Ga=function(){};
ui.prototype.Aa=function(){};
function vi(a,b,c,d){F.call(this);this.i=b;this.j=new Map;this.l=new Map;b=new Vh(1828,"0");b.h="21";b.network=new ri;if(d){var e=new Qg;d=zg(e,d,Jf);b.i=d}this.v=new Xh(1828,"","",!1,"",Wh(b));this.h=new me(this.v);c&&(this.h.l=1E5,c=this.h,c.flushInterval=3E4,c.h.setInterval(3E4));this.da=new se(this.h);this.ga=new ve(this.h);this.ia=new we(this.h);this.ba=new re(this.h);this.F=new te(this.h);this.K=new ue(this.h);this.errorCount=new ze(this.h);this.W=new ye(this.h);new xe(this.h);new Ae(this.h);
new Be(this.h);new Ce(this.h);this.m=Sg(a);a=new qe(this.h);this.j.set("h",1);this.j.set("u",2);this.j.set("k",3);this.j.set("P",4);this.j.set("p",5);this.l.set(25,1);this.l.set(26,2);this.l.set(27,3);this.l.set(28,4);a.h.zb("/client_streamz/bg/fic",this.i);Ec(this,this.v);Ec(this,this.h)}
x(vi,F);vi.prototype.Bc=function(){this.ga.h.zb("/client_streamz/bg/fsc",this.m,this.i)};
vi.prototype.ic=function(a,b){if("t"===a)this.da.record(b,this.m,this.i);else if("n"===a)this.ia.record(b,this.m,this.i);else if("h"===a||"u"===a||"k"===a||"P"===a||"p"===a){if(a=this.j.get(a))this.F.h.zb("/client_streamz/bg/fcc",a,this.i),this.K.record(b,a,this.i)}else this.W.record(b,a,this.i)};
vi.prototype.Ga=function(a){var b=this.l.get(a);b?this.ba.h.zb("/client_streamz/bg/fiec",this.m,this.i,b):this.errorCount.h.zb("/client_streamz/bg/cec",a,this.i)};
vi.prototype.Aa=function(){this.h.Aa()};
function ti(){var a,b,c;return null!=(c=null==(a=globalThis.performance)?void 0:null==(b=a.now)?void 0:b.call(a))?c:Date.now()}
;function wi(){var a=this;this.promise=new Promise(function(b,c){a.resolve=b;a.reject=c})}
;function xi(a){function b(t,y,v){Promise.resolve().then(function(){p.done();d.h&&d.ea.Aa();n.resolve({Ld:t,Ye:y,Og:v})})}
function c(t,y,v){if(h){var z="k";y?z="h":v&&(z="u");"k"!==z?d.ea.ic(z,t):0>=d.i?(d.ea.ic(z,t),d.i=Math.floor(200*Math.random())):d.i--}}
F.call(this);var d=this;this.h=!1;this.i=Math.floor(200*Math.random());var e=a.program;var f=a.de;var g=Math.random(),h=.3>g;null!=a.Nd&&(h=g<a.Nd);h&&(this.h=!0);var k=new F;this.addOnDisposeCallback(function(){d.j.then(function(t){t=t.Ye;d.ea.Aa();null==t||t();k.dispose()})});
if(!1!==a.Fe)if(a.ea)this.ea=a.ea;else{var l;Ec(k,this.ea=new vi(f,null!=(l=a.we)?l:"_",this.h))}else this.ea=new ui;var n=new wi;this.j=n.promise;var p=new si(this.ea,"t",!1);if(!C[f])throw this.ea.Ga(25),this.ea.Aa(),Error("EGOU");if(!C[f].a)throw this.ea.Ga(26),this.ea.Aa(),Error("ELIU");try{var r=C[f].a;p.start();this.l=w(r(e,b,!0,a.Yg,c)).next().value;this.Xe=n.promise.then(function(){})}catch(t){throw this.ea.Ga(28),this.ea.Aa(),t;
}}
x(xi,F);xi.prototype.snapshot=function(a){var b=this;if(this.V)throw Error("Already disposed");this.ea.Bc();return this.j.then(function(c){var d=c.Ld;return new Promise(function(e){var f=new si(b.ea,"n");d(function(g){f.done();b.h&&b.ea.Aa();e(g)},[a.Xc,
a.Ze,a.lf,a.af])})})};
xi.prototype.xd=function(a){if(this.V)throw Error("Already disposed");this.ea.Bc();var b=new si(this.ea,"n");a=this.l([a.Xc,a.Ze,a.lf,a.af]);b.done();this.h&&this.ea.Aa();return a};var yi=window;ib("csi.gstatic.com");ib("googleads.g.doubleclick.net");ib("partner.googleadservices.com");ib("pubads.g.doubleclick.net");ib("securepubads.g.doubleclick.net");ib("tpc.googlesyndication.com");function zi(a){var b=Ai;if(b)for(var c in b)Object.prototype.hasOwnProperty.call(b,c)&&a(b[c],c,b)}
function Bi(){var a=[];zi(function(b){a.push(b)});
return a}
var Ai={mf:"allow-forms",nf:"allow-modals",pf:"allow-orientation-lock",qf:"allow-pointer-lock",rf:"allow-popups",sf:"allow-popups-to-escape-sandbox",tf:"allow-presentation",uf:"allow-same-origin",vf:"allow-scripts",wf:"allow-top-navigation",xf:"allow-top-navigation-by-user-activation"},Ci=Cd(function(){return Bi()});
function Di(){var a=Ei(),b={};Db(Ci(),function(c){a.sandbox&&a.sandbox.supports&&a.sandbox.supports(c)&&(b[c]=!0)});
return b}
function Ei(){var a=void 0===a?document:a;return a.createElement("iframe")}
;function Fi(a){"number"==typeof a&&(a=Math.round(a)+"px");return a}
;var Gi=(new Date).getTime();function Hi(){var a=Ii;return Ng(function(b){for(var c in a)if(b===a[c]&&!/^[0-9]+$/.test(c))return!0;return!1})}
;function Ji(a){wd.call(this);var b=this;this.v=this.j=0;this.Da=null!=a?a:{pa:function(e,f){return setTimeout(e,f)},
qa:function(e){clearTimeout(e)}};
var c,d;this.i=null!=(d=null==(c=window.navigator)?void 0:c.onLine)?d:!0;this.l=function(){return A(function(e){return e.yield(Ki(b),0)})};
window.addEventListener("offline",this.l);window.addEventListener("online",this.l);this.v||Li(this)}
x(Ji,wd);function Mi(){var a=Ni;Ji.h||(Ji.h=new Ji(a));return Ji.h}
Ji.prototype.dispose=function(){window.removeEventListener("offline",this.l);window.removeEventListener("online",this.l);this.Da.qa(this.v);delete Ji.h};
Ji.prototype.va=function(){return this.i};
function Li(a){a.v=a.Da.pa(function(){var b;return A(function(c){if(1==c.h)return a.i?(null==(b=window.navigator)?0:b.onLine)?c.B(3):c.yield(Ki(a),3):c.yield(Ki(a),3);Li(a);c.h=0})},3E4)}
function Ki(a,b){return a.m?a.m:a.m=new Promise(function(c){var d,e,f,g;return A(function(h){switch(h.h){case 1:return d=window.AbortController?new window.AbortController:void 0,f=null==(e=d)?void 0:e.signal,g=!1,Aa(h,2,3),d&&(a.j=a.Da.pa(function(){d.abort()},b||2E4)),h.yield(fetch("/generate_204",{method:"HEAD",
signal:f}),5);case 5:g=!0;case 3:h.K=[h.j];h.l=0;h.A=0;a.m=void 0;a.j&&(a.Da.qa(a.j),a.j=0);g!==a.i&&(a.i=g,a.i?xd(a,"networkstatus-online"):xd(a,"networkstatus-offline"));c(g);Ca(h);break;case 2:Ba(h),g=!1,h.B(3)}})})}
;function Oi(){this.data=[];this.h=-1}
Oi.prototype.set=function(a,b){b=void 0===b?!0:b;0<=a&&52>a&&Number.isInteger(a)&&this.data[a]!==b&&(this.data[a]=b,this.h=-1)};
Oi.prototype.get=function(a){return!!this.data[a]};
function Pi(a){-1===a.h&&(a.h=a.data.reduce(function(b,c,d){return b+(c?Math.pow(2,d):0)},0));
return a.h}
;function Qi(){this.blockSize=-1}
;function Ri(){this.blockSize=-1;this.blockSize=64;this.h=[];this.A=[];this.m=[];this.j=[];this.j[0]=128;for(var a=1;a<this.blockSize;++a)this.j[a]=0;this.l=this.i=0;this.reset()}
$a(Ri,Qi);Ri.prototype.reset=function(){this.h[0]=1732584193;this.h[1]=4023233417;this.h[2]=2562383102;this.h[3]=271733878;this.h[4]=3285377520;this.l=this.i=0};
function Si(a,b,c){c||(c=0);var d=a.m;if("string"===typeof b)for(var e=0;16>e;e++)d[e]=b.charCodeAt(c)<<24|b.charCodeAt(c+1)<<16|b.charCodeAt(c+2)<<8|b.charCodeAt(c+3),c+=4;else for(e=0;16>e;e++)d[e]=b[c]<<24|b[c+1]<<16|b[c+2]<<8|b[c+3],c+=4;for(e=16;80>e;e++){var f=d[e-3]^d[e-8]^d[e-14]^d[e-16];d[e]=(f<<1|f>>>31)&4294967295}b=a.h[0];c=a.h[1];var g=a.h[2],h=a.h[3],k=a.h[4];for(e=0;80>e;e++){if(40>e)if(20>e){f=h^c&(g^h);var l=1518500249}else f=c^g^h,l=1859775393;else 60>e?(f=c&g|h&(c|g),l=2400959708):
(f=c^g^h,l=3395469782);f=(b<<5|b>>>27)+f+k+l+d[e]&4294967295;k=h;h=g;g=(c<<30|c>>>2)&4294967295;c=b;b=f}a.h[0]=a.h[0]+b&4294967295;a.h[1]=a.h[1]+c&4294967295;a.h[2]=a.h[2]+g&4294967295;a.h[3]=a.h[3]+h&4294967295;a.h[4]=a.h[4]+k&4294967295}
Ri.prototype.update=function(a,b){if(null!=a){void 0===b&&(b=a.length);for(var c=b-this.blockSize,d=0,e=this.A,f=this.i;d<b;){if(0==f)for(;d<=c;)Si(this,a,d),d+=this.blockSize;if("string"===typeof a)for(;d<b;){if(e[f]=a.charCodeAt(d),++f,++d,f==this.blockSize){Si(this,e);f=0;break}}else for(;d<b;)if(e[f]=a[d],++f,++d,f==this.blockSize){Si(this,e);f=0;break}}this.i=f;this.l+=b}};
Ri.prototype.digest=function(){var a=[],b=8*this.l;56>this.i?this.update(this.j,56-this.i):this.update(this.j,this.blockSize-(this.i-56));for(var c=this.blockSize-1;56<=c;c--)this.A[c]=b&255,b/=256;Si(this,this.A);for(c=b=0;5>c;c++)for(var d=24;0<=d;d-=8)a[b]=this.h[c]>>d&255,++b;return a};function Ti(a){return"string"==typeof a.className?a.className:a.getAttribute&&a.getAttribute("class")||""}
function Ui(a,b){"string"==typeof a.className?a.className=b:a.setAttribute&&a.setAttribute("class",b)}
function Vi(a,b){a.classList?b=a.classList.contains(b):(a=a.classList?a.classList:Ti(a).match(/\S+/g)||[],b=0<=Cb(a,b));return b}
function Wi(){var a=document.body;a.classList?a.classList.remove("inverted-hdpi"):Vi(a,"inverted-hdpi")&&Ui(a,Array.prototype.filter.call(a.classList?a.classList:Ti(a).match(/\S+/g)||[],function(b){return"inverted-hdpi"!=b}).join(" "))}
;function Xi(){}
Xi.prototype.next=function(){return Yi};
var Yi={done:!0,value:void 0};Xi.prototype.lb=function(){return this};function Zi(a){if(a instanceof $i||a instanceof aj||a instanceof bj)return a;if("function"==typeof a.next)return new $i(function(){return a});
if("function"==typeof a[Symbol.iterator])return new $i(function(){return a[Symbol.iterator]()});
if("function"==typeof a.lb)return new $i(function(){return a.lb()});
throw Error("Not an iterator or iterable.");}
function $i(a){this.h=a}
$i.prototype.lb=function(){return new aj(this.h())};
$i.prototype[Symbol.iterator]=function(){return new bj(this.h())};
$i.prototype.i=function(){return new bj(this.h())};
function aj(a){this.h=a}
x(aj,Xi);aj.prototype.next=function(){return this.h.next()};
aj.prototype[Symbol.iterator]=function(){return new bj(this.h)};
aj.prototype.i=function(){return new bj(this.h)};
function bj(a){$i.call(this,function(){return a});
this.j=a}
x(bj,$i);bj.prototype.next=function(){return this.j.next()};function M(a){F.call(this);this.m=1;this.j=[];this.l=0;this.h=[];this.i={};this.v=!!a}
$a(M,F);m=M.prototype;m.subscribe=function(a,b,c){var d=this.i[a];d||(d=this.i[a]=[]);var e=this.m;this.h[e]=a;this.h[e+1]=b;this.h[e+2]=c;this.m=e+3;d.push(e);return e};
m.unsubscribe=function(a,b,c){if(a=this.i[a]){var d=this.h;if(a=a.find(function(e){return d[e+1]==b&&d[e+2]==c}))return this.Ab(a)}return!1};
m.Ab=function(a){var b=this.h[a];if(b){var c=this.i[b];0!=this.l?(this.j.push(a),this.h[a+1]=function(){}):(c&&Ib(c,a),delete this.h[a],delete this.h[a+1],delete this.h[a+2])}return!!b};
m.Xa=function(a,b){var c=this.i[a];if(c){for(var d=Array(arguments.length-1),e=1,f=arguments.length;e<f;e++)d[e-1]=arguments[e];if(this.v)for(e=0;e<c.length;e++){var g=c[e];cj(this.h[g+1],this.h[g+2],d)}else{this.l++;try{for(e=0,f=c.length;e<f&&!this.V;e++)g=c[e],this.h[g+1].apply(this.h[g+2],d)}finally{if(this.l--,0<this.j.length&&0==this.l)for(;c=this.j.pop();)this.Ab(c)}}return 0!=e}return!1};
function cj(a,b,c){Rd(function(){a.apply(b,c)})}
m.clear=function(a){if(a){var b=this.i[a];b&&(b.forEach(this.Ab,this),delete this.i[a])}else this.h.length=0,this.i={}};
m.U=function(){M.Ba.U.call(this);this.clear();this.j.length=0};function dj(a){this.h=a}
dj.prototype.set=function(a,b){void 0===b?this.h.remove(a):this.h.set(a,(new Yh).serialize(b))};
dj.prototype.get=function(a){try{var b=this.h.get(a)}catch(c){return}if(null!==b)try{return JSON.parse(b)}catch(c){throw"Storage: Invalid value was encountered";}};
dj.prototype.remove=function(a){this.h.remove(a)};function ej(a){this.h=a}
$a(ej,dj);function fj(a){this.data=a}
function gj(a){return void 0===a||a instanceof fj?a:new fj(a)}
ej.prototype.set=function(a,b){ej.Ba.set.call(this,a,gj(b))};
ej.prototype.i=function(a){a=ej.Ba.get.call(this,a);if(void 0===a||a instanceof Object)return a;throw"Storage: Invalid value was encountered";};
ej.prototype.get=function(a){if(a=this.i(a)){if(a=a.data,void 0===a)throw"Storage: Invalid value was encountered";}else a=void 0;return a};function hj(a){this.h=a}
$a(hj,ej);hj.prototype.set=function(a,b,c){if(b=gj(b)){if(c){if(c<Za()){hj.prototype.remove.call(this,a);return}b.expiration=c}b.creation=Za()}hj.Ba.set.call(this,a,b)};
hj.prototype.i=function(a){var b=hj.Ba.i.call(this,a);if(b){var c=b.creation,d=b.expiration;if(d&&d<Za()||c&&c>Za())hj.prototype.remove.call(this,a);else return b}};function ij(){}
;function jj(){}
$a(jj,ij);jj.prototype[Symbol.iterator]=function(){return Zi(this.lb(!0)).i()};
jj.prototype.clear=function(){var a=Array.from(this);a=w(a);for(var b=a.next();!b.done;b=a.next())this.remove(b.value)};function kj(a){this.h=a;this.i=null}
$a(kj,jj);m=kj.prototype;m.isAvailable=function(){var a=this.h;if(a)try{a.setItem("__sak","1");a.removeItem("__sak");var b=!0}catch(c){b=c instanceof DOMException&&("QuotaExceededError"===c.name||22===c.code||1014===c.code||"NS_ERROR_DOM_QUOTA_REACHED"===c.name)&&a&&0!==a.length}else b=!1;return this.i=b};
m.set=function(a,b){lj(this);try{this.h.setItem(a,b)}catch(c){if(0==this.h.length)throw"Storage mechanism: Storage disabled";throw"Storage mechanism: Quota exceeded";}};
m.get=function(a){lj(this);a=this.h.getItem(a);if("string"!==typeof a&&null!==a)throw"Storage mechanism: Invalid value was encountered";return a};
m.remove=function(a){lj(this);this.h.removeItem(a)};
m.lb=function(a){lj(this);var b=0,c=this.h,d=new Xi;d.next=function(){if(b>=c.length)return Yi;var e=c.key(b++);if(a)return{value:e,done:!1};e=c.getItem(e);if("string"!==typeof e)throw"Storage mechanism: Invalid value was encountered";return{value:e,done:!1}};
return d};
m.clear=function(){lj(this);this.h.clear()};
m.key=function(a){lj(this);return this.h.key(a)};
function lj(a){if(null==a.h)throw Error("Storage mechanism: Storage unavailable");var b;(null!=(b=a.i)?b:a.isAvailable())||Kd(Error("Storage mechanism: Storage unavailable"))}
;function mj(){var a=null;try{a=C.localStorage||null}catch(b){}kj.call(this,a)}
$a(mj,kj);function nj(a,b){this.i=a;this.h=b+"::"}
$a(nj,jj);nj.prototype.set=function(a,b){this.i.set(this.h+a,b)};
nj.prototype.get=function(a){return this.i.get(this.h+a)};
nj.prototype.remove=function(a){this.i.remove(this.h+a)};
nj.prototype.lb=function(a){var b=this.i[Symbol.iterator](),c=this,d=new Xi;d.next=function(){var e=b.next();if(e.done)return e;for(e=e.value;e.slice(0,c.h.length)!=c.h;){e=b.next();if(e.done)return e;e=e.value}return{value:a?e.slice(c.h.length):c.i.get(e),done:!1}};
return d};/*

 (The MIT License)

 Copyright (C) 2014 by Vitaly Puzrin

 Permission is hereby granted, free of charge, to any person obtaining a copy
 of this software and associated documentation files (the "Software"), to deal
 in the Software without restriction, including without limitation the rights
 to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 copies of the Software, and to permit persons to whom the Software is
 furnished to do so, subject to the following conditions:

 The above copyright notice and this permission notice shall be included in
 all copies or substantial portions of the Software.

 THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 THE SOFTWARE.

 -----------------------------------------------------------------------------
 Ported from zlib, which is under the following license
 https://github.com/madler/zlib/blob/master/zlib.h

 zlib.h -- interface of the 'zlib' general purpose compression library
   version 1.2.8, April 28th, 2013
   Copyright (C) 1995-2013 Jean-loup Gailly and Mark Adler
   This software is provided 'as-is', without any express or implied
   warranty.  In no event will the authors be held liable for any damages
   arising from the use of this software.
   Permission is granted to anyone to use this software for any purpose,
   including commercial applications, and to alter it and redistribute it
   freely, subject to the following restrictions:
   1. The origin of this software must not be misrepresented; you must not
      claim that you wrote the original software. If you use this software
      in a product, an acknowledgment in the product documentation would be
      appreciated but is not required.
   2. Altered source versions must be plainly marked as such, and must not be
      misrepresented as being the original software.
   3. This notice may not be removed or altered from any source distribution.
   Jean-loup Gailly        Mark Adler
   jloup@gzip.org          madler@alumni.caltech.edu
   The data format used by the zlib library is described by RFCs (Request for
   Comments) 1950 to 1952 in the files http://tools.ietf.org/html/rfc1950
   (zlib format), rfc1951 (deflate format) and rfc1952 (gzip format).
*/
var O={},oj="undefined"!==typeof Uint8Array&&"undefined"!==typeof Uint16Array&&"undefined"!==typeof Int32Array;O.assign=function(a){for(var b=Array.prototype.slice.call(arguments,1);b.length;){var c=b.shift();if(c){if("object"!==typeof c)throw new TypeError(c+"must be non-object");for(var d in c)Object.prototype.hasOwnProperty.call(c,d)&&(a[d]=c[d])}}return a};
O.Mc=function(a,b){if(a.length===b)return a;if(a.subarray)return a.subarray(0,b);a.length=b;return a};
var pj={mb:function(a,b,c,d,e){if(b.subarray&&a.subarray)a.set(b.subarray(c,c+d),e);else for(var f=0;f<d;f++)a[e+f]=b[c+f]},
bd:function(a){var b,c;var d=c=0;for(b=a.length;d<b;d++)c+=a[d].length;var e=new Uint8Array(c);d=c=0;for(b=a.length;d<b;d++){var f=a[d];e.set(f,c);c+=f.length}return e}},qj={mb:function(a,b,c,d,e){for(var f=0;f<d;f++)a[e+f]=b[c+f]},
bd:function(a){return[].concat.apply([],a)}};
O.We=function(){oj?(O.kb=Uint8Array,O.Ia=Uint16Array,O.Gd=Int32Array,O.assign(O,pj)):(O.kb=Array,O.Ia=Array,O.Gd=Array,O.assign(O,qj))};
O.We();var rj=!0;try{new Uint8Array(1)}catch(a){rj=!1}
function sj(a){var b,c,d=a.length,e=0;for(b=0;b<d;b++){var f=a.charCodeAt(b);if(55296===(f&64512)&&b+1<d){var g=a.charCodeAt(b+1);56320===(g&64512)&&(f=65536+(f-55296<<10)+(g-56320),b++)}e+=128>f?1:2048>f?2:65536>f?3:4}var h=new O.kb(e);for(b=c=0;c<e;b++)f=a.charCodeAt(b),55296===(f&64512)&&b+1<d&&(g=a.charCodeAt(b+1),56320===(g&64512)&&(f=65536+(f-55296<<10)+(g-56320),b++)),128>f?h[c++]=f:(2048>f?h[c++]=192|f>>>6:(65536>f?h[c++]=224|f>>>12:(h[c++]=240|f>>>18,h[c++]=128|f>>>12&63),h[c++]=128|f>>>
6&63),h[c++]=128|f&63);return h}
;var tj={};tj=function(a,b,c,d){var e=a&65535|0;a=a>>>16&65535|0;for(var f;0!==c;){f=2E3<c?2E3:c;c-=f;do e=e+b[d++]|0,a=a+e|0;while(--f);e%=65521;a%=65521}return e|a<<16|0};for(var uj={},vj,wj=[],xj=0;256>xj;xj++){vj=xj;for(var yj=0;8>yj;yj++)vj=vj&1?3988292384^vj>>>1:vj>>>1;wj[xj]=vj}uj=function(a,b,c,d){c=d+c;for(a^=-1;d<c;d++)a=a>>>8^wj[(a^b[d])&255];return a^-1};var zj={};zj={2:"need dictionary",1:"stream end",0:"","-1":"file error","-2":"stream error","-3":"data error","-4":"insufficient memory","-5":"buffer error","-6":"incompatible version"};function Aj(a){for(var b=a.length;0<=--b;)a[b]=0}
var Bj=[0,0,0,0,0,0,0,0,1,1,1,1,2,2,2,2,3,3,3,3,4,4,4,4,5,5,5,5,0],Cj=[0,0,0,0,1,1,2,2,3,3,4,4,5,5,6,6,7,7,8,8,9,9,10,10,11,11,12,12,13,13],Dj=[0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,2,3,7],Ej=[16,17,18,0,8,7,9,6,10,5,11,4,12,3,13,2,14,1,15],Fj=Array(576);Aj(Fj);var Gj=Array(60);Aj(Gj);var Hj=Array(512);Aj(Hj);var Ij=Array(256);Aj(Ij);var Jj=Array(29);Aj(Jj);var Kj=Array(30);Aj(Kj);function Lj(a,b,c,d,e){this.yd=a;this.Yd=b;this.Xd=c;this.Sd=d;this.ue=e;this.ed=a&&a.length}
var Mj,Nj,Oj;function Pj(a,b){this.Zc=a;this.ub=0;this.Va=b}
function Qj(a,b){a.Z[a.pending++]=b&255;a.Z[a.pending++]=b>>>8&255}
function Rj(a,b,c){a.ja>16-c?(a.oa|=b<<a.ja&65535,Qj(a,a.oa),a.oa=b>>16-a.ja,a.ja+=c-16):(a.oa|=b<<a.ja&65535,a.ja+=c)}
function Sj(a,b,c){Rj(a,c[2*b],c[2*b+1])}
function Tj(a,b){var c=0;do c|=a&1,a>>>=1,c<<=1;while(0<--b);return c>>>1}
function Uj(a,b,c){var d=Array(16),e=0,f;for(f=1;15>=f;f++)d[f]=e=e+c[f-1]<<1;for(c=0;c<=b;c++)e=a[2*c+1],0!==e&&(a[2*c]=Tj(d[e]++,e))}
function Vj(a){var b;for(b=0;286>b;b++)a.ra[2*b]=0;for(b=0;30>b;b++)a.ab[2*b]=0;for(b=0;19>b;b++)a.ka[2*b]=0;a.ra[512]=1;a.Oa=a.yb=0;a.ya=a.matches=0}
function Wj(a){8<a.ja?Qj(a,a.oa):0<a.ja&&(a.Z[a.pending++]=a.oa);a.oa=0;a.ja=0}
function Xj(a,b,c){Wj(a);Qj(a,c);Qj(a,~c);O.mb(a.Z,a.window,b,c,a.pending);a.pending+=c}
function Yj(a,b,c,d){var e=2*b,f=2*c;return a[e]<a[f]||a[e]===a[f]&&d[b]<=d[c]}
function Zj(a,b,c){for(var d=a.aa[c],e=c<<1;e<=a.Ma;){e<a.Ma&&Yj(b,a.aa[e+1],a.aa[e],a.depth)&&e++;if(Yj(b,d,a.aa[e],a.depth))break;a.aa[c]=a.aa[e];c=e;e<<=1}a.aa[c]=d}
function ak(a,b,c){var d=0;if(0!==a.ya){do{var e=a.Z[a.Fb+2*d]<<8|a.Z[a.Fb+2*d+1];var f=a.Z[a.Ac+d];d++;if(0===e)Sj(a,f,b);else{var g=Ij[f];Sj(a,g+256+1,b);var h=Bj[g];0!==h&&(f-=Jj[g],Rj(a,f,h));e--;g=256>e?Hj[e]:Hj[256+(e>>>7)];Sj(a,g,c);h=Cj[g];0!==h&&(e-=Kj[g],Rj(a,e,h))}}while(d<a.ya)}Sj(a,256,b)}
function bk(a,b){var c=b.Zc,d=b.Va.yd,e=b.Va.ed,f=b.Va.Sd,g,h=-1;a.Ma=0;a.pb=573;for(g=0;g<f;g++)0!==c[2*g]?(a.aa[++a.Ma]=h=g,a.depth[g]=0):c[2*g+1]=0;for(;2>a.Ma;){var k=a.aa[++a.Ma]=2>h?++h:0;c[2*k]=1;a.depth[k]=0;a.Oa--;e&&(a.yb-=d[2*k+1])}b.ub=h;for(g=a.Ma>>1;1<=g;g--)Zj(a,c,g);k=f;do g=a.aa[1],a.aa[1]=a.aa[a.Ma--],Zj(a,c,1),d=a.aa[1],a.aa[--a.pb]=g,a.aa[--a.pb]=d,c[2*k]=c[2*g]+c[2*d],a.depth[k]=(a.depth[g]>=a.depth[d]?a.depth[g]:a.depth[d])+1,c[2*g+1]=c[2*d+1]=k,a.aa[1]=k++,Zj(a,c,1);while(2<=
a.Ma);a.aa[--a.pb]=a.aa[1];g=b.Zc;k=b.ub;d=b.Va.yd;e=b.Va.ed;f=b.Va.Yd;var l=b.Va.Xd,n=b.Va.ue,p,r=0;for(p=0;15>=p;p++)a.Ja[p]=0;g[2*a.aa[a.pb]+1]=0;for(b=a.pb+1;573>b;b++){var t=a.aa[b];p=g[2*g[2*t+1]+1]+1;p>n&&(p=n,r++);g[2*t+1]=p;if(!(t>k)){a.Ja[p]++;var y=0;t>=l&&(y=f[t-l]);var v=g[2*t];a.Oa+=v*(p+y);e&&(a.yb+=v*(d[2*t+1]+y))}}if(0!==r){do{for(p=n-1;0===a.Ja[p];)p--;a.Ja[p]--;a.Ja[p+1]+=2;a.Ja[n]--;r-=2}while(0<r);for(p=n;0!==p;p--)for(t=a.Ja[p];0!==t;)d=a.aa[--b],d>k||(g[2*d+1]!==p&&(a.Oa+=(p-
g[2*d+1])*g[2*d],g[2*d+1]=p),t--)}Uj(c,h,a.Ja)}
function ck(a,b,c){var d,e=-1,f=b[1],g=0,h=7,k=4;0===f&&(h=138,k=3);b[2*(c+1)+1]=65535;for(d=0;d<=c;d++){var l=f;f=b[2*(d+1)+1];++g<h&&l===f||(g<k?a.ka[2*l]+=g:0!==l?(l!==e&&a.ka[2*l]++,a.ka[32]++):10>=g?a.ka[34]++:a.ka[36]++,g=0,e=l,0===f?(h=138,k=3):l===f?(h=6,k=3):(h=7,k=4))}}
function dk(a,b,c){var d,e=-1,f=b[1],g=0,h=7,k=4;0===f&&(h=138,k=3);for(d=0;d<=c;d++){var l=f;f=b[2*(d+1)+1];if(!(++g<h&&l===f)){if(g<k){do Sj(a,l,a.ka);while(0!==--g)}else 0!==l?(l!==e&&(Sj(a,l,a.ka),g--),Sj(a,16,a.ka),Rj(a,g-3,2)):10>=g?(Sj(a,17,a.ka),Rj(a,g-3,3)):(Sj(a,18,a.ka),Rj(a,g-11,7));g=0;e=l;0===f?(h=138,k=3):l===f?(h=6,k=3):(h=7,k=4)}}}
function ek(a){var b=4093624447,c;for(c=0;31>=c;c++,b>>>=1)if(b&1&&0!==a.ra[2*c])return 0;if(0!==a.ra[18]||0!==a.ra[20]||0!==a.ra[26])return 1;for(c=32;256>c;c++)if(0!==a.ra[2*c])return 1;return 0}
var fk=!1;function gk(a,b,c){a.Z[a.Fb+2*a.ya]=b>>>8&255;a.Z[a.Fb+2*a.ya+1]=b&255;a.Z[a.Ac+a.ya]=c&255;a.ya++;0===b?a.ra[2*c]++:(a.matches++,b--,a.ra[2*(Ij[c]+256+1)]++,a.ab[2*(256>b?Hj[b]:Hj[256+(b>>>7)])]++);return a.ya===a.Lb-1}
;function hk(a,b){a.msg=zj[b];return b}
function ik(a){for(var b=a.length;0<=--b;)a[b]=0}
function jk(a){var b=a.state,c=b.pending;c>a.M&&(c=a.M);0!==c&&(O.mb(a.output,b.Z,b.Nb,c,a.vb),a.vb+=c,b.Nb+=c,a.Nc+=c,a.M-=c,b.pending-=c,0===b.pending&&(b.Nb=0))}
function kk(a,b){var c=0<=a.ta?a.ta:-1,d=a.o-a.ta,e=0;if(0<a.level){2===a.I.vc&&(a.I.vc=ek(a));bk(a,a.hc);bk(a,a.ac);ck(a,a.ra,a.hc.ub);ck(a,a.ab,a.ac.ub);bk(a,a.Tc);for(e=18;3<=e&&0===a.ka[2*Ej[e]+1];e--);a.Oa+=3*(e+1)+14;var f=a.Oa+3+7>>>3;var g=a.yb+3+7>>>3;g<=f&&(f=g)}else f=g=d+5;if(d+4<=f&&-1!==c)Rj(a,b?1:0,3),Xj(a,c,d);else if(4===a.strategy||g===f)Rj(a,2+(b?1:0),3),ak(a,Fj,Gj);else{Rj(a,4+(b?1:0),3);c=a.hc.ub+1;d=a.ac.ub+1;e+=1;Rj(a,c-257,5);Rj(a,d-1,5);Rj(a,e-4,4);for(f=0;f<e;f++)Rj(a,a.ka[2*
Ej[f]+1],3);dk(a,a.ra,c-1);dk(a,a.ab,d-1);ak(a,a.ra,a.ab)}Vj(a);b&&Wj(a);a.ta=a.o;jk(a.I)}
function R(a,b){a.Z[a.pending++]=b}
function lk(a,b){a.Z[a.pending++]=b>>>8&255;a.Z[a.pending++]=b&255}
function mk(a,b){var c=a.jd,d=a.o,e=a.wa,f=a.kd,g=a.o>a.ma-262?a.o-(a.ma-262):0,h=a.window,k=a.Wa,l=a.Ha,n=a.o+258,p=h[d+e-1],r=h[d+e];a.wa>=a.dd&&(c>>=2);f>a.u&&(f=a.u);do{var t=b;if(h[t+e]===r&&h[t+e-1]===p&&h[t]===h[d]&&h[++t]===h[d+1]){d+=2;for(t++;h[++d]===h[++t]&&h[++d]===h[++t]&&h[++d]===h[++t]&&h[++d]===h[++t]&&h[++d]===h[++t]&&h[++d]===h[++t]&&h[++d]===h[++t]&&h[++d]===h[++t]&&d<n;);t=258-(n-d);d=n-258;if(t>e){a.tb=b;e=t;if(t>=f)break;p=h[d+e-1];r=h[d+e]}}}while((b=l[b&k])>g&&0!==--c);return e<=
a.u?e:a.u}
function nk(a){var b=a.ma,c;do{var d=a.Ed-a.u-a.o;if(a.o>=b+(b-262)){O.mb(a.window,a.window,b,b,0);a.tb-=b;a.o-=b;a.ta-=b;var e=c=a.fc;do{var f=a.head[--e];a.head[e]=f>=b?f-b:0}while(--c);e=c=b;do f=a.Ha[--e],a.Ha[e]=f>=b?f-b:0;while(--c);d+=b}if(0===a.I.na)break;e=a.I;c=a.window;f=a.o+a.u;var g=e.na;g>d&&(g=d);0===g?c=0:(e.na-=g,O.mb(c,e.input,e.gb,g,f),1===e.state.wrap?e.H=tj(e.H,c,g,f):2===e.state.wrap&&(e.H=uj(e.H,c,g,f)),e.gb+=g,e.jb+=g,c=g);a.u+=c;if(3<=a.u+a.sa)for(d=a.o-a.sa,a.J=a.window[d],
a.J=(a.J<<a.La^a.window[d+1])&a.Ka;a.sa&&!(a.J=(a.J<<a.La^a.window[d+3-1])&a.Ka,a.Ha[d&a.Wa]=a.head[a.J],a.head[a.J]=d,d++,a.sa--,3>a.u+a.sa););}while(262>a.u&&0!==a.I.na)}
function ok(a,b){for(var c;;){if(262>a.u){nk(a);if(262>a.u&&0===b)return 1;if(0===a.u)break}c=0;3<=a.u&&(a.J=(a.J<<a.La^a.window[a.o+3-1])&a.Ka,c=a.Ha[a.o&a.Wa]=a.head[a.J],a.head[a.J]=a.o);0!==c&&a.o-c<=a.ma-262&&(a.P=mk(a,c));if(3<=a.P)if(c=gk(a,a.o-a.tb,a.P-3),a.u-=a.P,a.P<=a.Cc&&3<=a.u){a.P--;do a.o++,a.J=(a.J<<a.La^a.window[a.o+3-1])&a.Ka,a.Ha[a.o&a.Wa]=a.head[a.J],a.head[a.J]=a.o;while(0!==--a.P);a.o++}else a.o+=a.P,a.P=0,a.J=a.window[a.o],a.J=(a.J<<a.La^a.window[a.o+1])&a.Ka;else c=gk(a,0,
a.window[a.o]),a.u--,a.o++;if(c&&(kk(a,!1),0===a.I.M))return 1}a.sa=2>a.o?a.o:2;return 4===b?(kk(a,!0),0===a.I.M?3:4):a.ya&&(kk(a,!1),0===a.I.M)?1:2}
function pk(a,b){for(var c,d;;){if(262>a.u){nk(a);if(262>a.u&&0===b)return 1;if(0===a.u)break}c=0;3<=a.u&&(a.J=(a.J<<a.La^a.window[a.o+3-1])&a.Ka,c=a.Ha[a.o&a.Wa]=a.head[a.J],a.head[a.J]=a.o);a.wa=a.P;a.nd=a.tb;a.P=2;0!==c&&a.wa<a.Cc&&a.o-c<=a.ma-262&&(a.P=mk(a,c),5>=a.P&&(1===a.strategy||3===a.P&&4096<a.o-a.tb)&&(a.P=2));if(3<=a.wa&&a.P<=a.wa){d=a.o+a.u-3;c=gk(a,a.o-1-a.nd,a.wa-3);a.u-=a.wa-1;a.wa-=2;do++a.o<=d&&(a.J=(a.J<<a.La^a.window[a.o+3-1])&a.Ka,a.Ha[a.o&a.Wa]=a.head[a.J],a.head[a.J]=a.o);
while(0!==--a.wa);a.eb=0;a.P=2;a.o++;if(c&&(kk(a,!1),0===a.I.M))return 1}else if(a.eb){if((c=gk(a,0,a.window[a.o-1]))&&kk(a,!1),a.o++,a.u--,0===a.I.M)return 1}else a.eb=1,a.o++,a.u--}a.eb&&(gk(a,0,a.window[a.o-1]),a.eb=0);a.sa=2>a.o?a.o:2;return 4===b?(kk(a,!0),0===a.I.M?3:4):a.ya&&(kk(a,!1),0===a.I.M)?1:2}
function qk(a,b){for(var c,d,e,f=a.window;;){if(258>=a.u){nk(a);if(258>=a.u&&0===b)return 1;if(0===a.u)break}a.P=0;if(3<=a.u&&0<a.o&&(d=a.o-1,c=f[d],c===f[++d]&&c===f[++d]&&c===f[++d])){for(e=a.o+258;c===f[++d]&&c===f[++d]&&c===f[++d]&&c===f[++d]&&c===f[++d]&&c===f[++d]&&c===f[++d]&&c===f[++d]&&d<e;);a.P=258-(e-d);a.P>a.u&&(a.P=a.u)}3<=a.P?(c=gk(a,1,a.P-3),a.u-=a.P,a.o+=a.P,a.P=0):(c=gk(a,0,a.window[a.o]),a.u--,a.o++);if(c&&(kk(a,!1),0===a.I.M))return 1}a.sa=0;return 4===b?(kk(a,!0),0===a.I.M?3:4):
a.ya&&(kk(a,!1),0===a.I.M)?1:2}
function rk(a,b){for(var c;;){if(0===a.u&&(nk(a),0===a.u)){if(0===b)return 1;break}a.P=0;c=gk(a,0,a.window[a.o]);a.u--;a.o++;if(c&&(kk(a,!1),0===a.I.M))return 1}a.sa=0;return 4===b?(kk(a,!0),0===a.I.M?3:4):a.ya&&(kk(a,!1),0===a.I.M)?1:2}
function sk(a,b,c,d,e){this.ee=a;this.te=b;this.ye=c;this.se=d;this.ae=e}
var tk;tk=[new sk(0,0,0,0,function(a,b){var c=65535;for(c>a.za-5&&(c=a.za-5);;){if(1>=a.u){nk(a);if(0===a.u&&0===b)return 1;if(0===a.u)break}a.o+=a.u;a.u=0;var d=a.ta+c;if(0===a.o||a.o>=d)if(a.u=a.o-d,a.o=d,kk(a,!1),0===a.I.M)return 1;if(a.o-a.ta>=a.ma-262&&(kk(a,!1),0===a.I.M))return 1}a.sa=0;if(4===b)return kk(a,!0),0===a.I.M?3:4;a.o>a.ta&&kk(a,!1);return 1}),
new sk(4,4,8,4,ok),new sk(4,5,16,8,ok),new sk(4,6,32,32,ok),new sk(4,4,16,16,pk),new sk(8,16,32,32,pk),new sk(8,16,128,128,pk),new sk(8,32,128,256,pk),new sk(32,128,258,1024,pk),new sk(32,258,258,4096,pk)];
function uk(){this.I=null;this.status=0;this.Z=null;this.wrap=this.pending=this.Nb=this.za=0;this.G=null;this.Ca=0;this.method=8;this.rb=-1;this.Wa=this.Qc=this.ma=0;this.window=null;this.Ed=0;this.head=this.Ha=null;this.kd=this.dd=this.strategy=this.level=this.Cc=this.jd=this.wa=this.u=this.tb=this.o=this.eb=this.nd=this.P=this.ta=this.La=this.Ka=this.yc=this.fc=this.J=0;this.ra=new O.Ia(1146);this.ab=new O.Ia(122);this.ka=new O.Ia(78);ik(this.ra);ik(this.ab);ik(this.ka);this.Tc=this.ac=this.hc=
null;this.Ja=new O.Ia(16);this.aa=new O.Ia(573);ik(this.aa);this.pb=this.Ma=0;this.depth=new O.Ia(573);ik(this.depth);this.ja=this.oa=this.sa=this.matches=this.yb=this.Oa=this.Fb=this.ya=this.Lb=this.Ac=0}
function vk(a,b){if(!a||!a.state||5<b||0>b)return a?hk(a,-2):-2;var c=a.state;if(!a.output||!a.input&&0!==a.na||666===c.status&&4!==b)return hk(a,0===a.M?-5:-2);c.I=a;var d=c.rb;c.rb=b;if(42===c.status)if(2===c.wrap)a.H=0,R(c,31),R(c,139),R(c,8),c.G?(R(c,(c.G.text?1:0)+(c.G.Sa?2:0)+(c.G.extra?4:0)+(c.G.name?8:0)+(c.G.comment?16:0)),R(c,c.G.time&255),R(c,c.G.time>>8&255),R(c,c.G.time>>16&255),R(c,c.G.time>>24&255),R(c,9===c.level?2:2<=c.strategy||2>c.level?4:0),R(c,c.G.os&255),c.G.extra&&c.G.extra.length&&
(R(c,c.G.extra.length&255),R(c,c.G.extra.length>>8&255)),c.G.Sa&&(a.H=uj(a.H,c.Z,c.pending,0)),c.Ca=0,c.status=69):(R(c,0),R(c,0),R(c,0),R(c,0),R(c,0),R(c,9===c.level?2:2<=c.strategy||2>c.level?4:0),R(c,3),c.status=113);else{var e=8+(c.Qc-8<<4)<<8;e|=(2<=c.strategy||2>c.level?0:6>c.level?1:6===c.level?2:3)<<6;0!==c.o&&(e|=32);c.status=113;lk(c,e+(31-e%31));0!==c.o&&(lk(c,a.H>>>16),lk(c,a.H&65535));a.H=1}if(69===c.status)if(c.G.extra){for(e=c.pending;c.Ca<(c.G.extra.length&65535)&&(c.pending!==c.za||
(c.G.Sa&&c.pending>e&&(a.H=uj(a.H,c.Z,c.pending-e,e)),jk(a),e=c.pending,c.pending!==c.za));)R(c,c.G.extra[c.Ca]&255),c.Ca++;c.G.Sa&&c.pending>e&&(a.H=uj(a.H,c.Z,c.pending-e,e));c.Ca===c.G.extra.length&&(c.Ca=0,c.status=73)}else c.status=73;if(73===c.status)if(c.G.name){e=c.pending;do{if(c.pending===c.za&&(c.G.Sa&&c.pending>e&&(a.H=uj(a.H,c.Z,c.pending-e,e)),jk(a),e=c.pending,c.pending===c.za)){var f=1;break}f=c.Ca<c.G.name.length?c.G.name.charCodeAt(c.Ca++)&255:0;R(c,f)}while(0!==f);c.G.Sa&&c.pending>
e&&(a.H=uj(a.H,c.Z,c.pending-e,e));0===f&&(c.Ca=0,c.status=91)}else c.status=91;if(91===c.status)if(c.G.comment){e=c.pending;do{if(c.pending===c.za&&(c.G.Sa&&c.pending>e&&(a.H=uj(a.H,c.Z,c.pending-e,e)),jk(a),e=c.pending,c.pending===c.za)){f=1;break}f=c.Ca<c.G.comment.length?c.G.comment.charCodeAt(c.Ca++)&255:0;R(c,f)}while(0!==f);c.G.Sa&&c.pending>e&&(a.H=uj(a.H,c.Z,c.pending-e,e));0===f&&(c.status=103)}else c.status=103;103===c.status&&(c.G.Sa?(c.pending+2>c.za&&jk(a),c.pending+2<=c.za&&(R(c,a.H&
255),R(c,a.H>>8&255),a.H=0,c.status=113)):c.status=113);if(0!==c.pending){if(jk(a),0===a.M)return c.rb=-1,0}else if(0===a.na&&(b<<1)-(4<b?9:0)<=(d<<1)-(4<d?9:0)&&4!==b)return hk(a,-5);if(666===c.status&&0!==a.na)return hk(a,-5);if(0!==a.na||0!==c.u||0!==b&&666!==c.status){d=2===c.strategy?rk(c,b):3===c.strategy?qk(c,b):tk[c.level].ae(c,b);if(3===d||4===d)c.status=666;if(1===d||3===d)return 0===a.M&&(c.rb=-1),0;if(2===d&&(1===b?(Rj(c,2,3),Sj(c,256,Fj),16===c.ja?(Qj(c,c.oa),c.oa=0,c.ja=0):8<=c.ja&&
(c.Z[c.pending++]=c.oa&255,c.oa>>=8,c.ja-=8)):5!==b&&(Rj(c,0,3),Xj(c,0,0),3===b&&(ik(c.head),0===c.u&&(c.o=0,c.ta=0,c.sa=0))),jk(a),0===a.M))return c.rb=-1,0}if(4!==b)return 0;if(0>=c.wrap)return 1;2===c.wrap?(R(c,a.H&255),R(c,a.H>>8&255),R(c,a.H>>16&255),R(c,a.H>>24&255),R(c,a.jb&255),R(c,a.jb>>8&255),R(c,a.jb>>16&255),R(c,a.jb>>24&255)):(lk(c,a.H>>>16),lk(c,a.H&65535));jk(a);0<c.wrap&&(c.wrap=-c.wrap);return 0!==c.pending?0:1}
;var wk={};wk=function(){this.input=null;this.jb=this.na=this.gb=0;this.output=null;this.Nc=this.M=this.vb=0;this.msg="";this.state=null;this.vc=2;this.H=0};var xk=Object.prototype.toString;
function yk(a){if(!(this instanceof yk))return new yk(a);a=this.options=O.assign({level:-1,method:8,chunkSize:16384,windowBits:15,memLevel:8,strategy:0,to:""},a||{});a.raw&&0<a.windowBits?a.windowBits=-a.windowBits:a.gzip&&0<a.windowBits&&16>a.windowBits&&(a.windowBits+=16);this.err=0;this.msg="";this.ended=!1;this.chunks=[];this.I=new wk;this.I.M=0;var b=this.I;var c=a.level,d=a.method,e=a.windowBits,f=a.memLevel,g=a.strategy;if(b){var h=1;-1===c&&(c=6);0>e?(h=0,e=-e):15<e&&(h=2,e-=16);if(1>f||9<
f||8!==d||8>e||15<e||0>c||9<c||0>g||4<g)b=hk(b,-2);else{8===e&&(e=9);var k=new uk;b.state=k;k.I=b;k.wrap=h;k.G=null;k.Qc=e;k.ma=1<<k.Qc;k.Wa=k.ma-1;k.yc=f+7;k.fc=1<<k.yc;k.Ka=k.fc-1;k.La=~~((k.yc+3-1)/3);k.window=new O.kb(2*k.ma);k.head=new O.Ia(k.fc);k.Ha=new O.Ia(k.ma);k.Lb=1<<f+6;k.za=4*k.Lb;k.Z=new O.kb(k.za);k.Fb=1*k.Lb;k.Ac=3*k.Lb;k.level=c;k.strategy=g;k.method=d;if(b&&b.state){b.jb=b.Nc=0;b.vc=2;c=b.state;c.pending=0;c.Nb=0;0>c.wrap&&(c.wrap=-c.wrap);c.status=c.wrap?42:113;b.H=2===c.wrap?
0:1;c.rb=0;if(!fk){d=Array(16);for(f=g=0;28>f;f++)for(Jj[f]=g,e=0;e<1<<Bj[f];e++)Ij[g++]=f;Ij[g-1]=f;for(f=g=0;16>f;f++)for(Kj[f]=g,e=0;e<1<<Cj[f];e++)Hj[g++]=f;for(g>>=7;30>f;f++)for(Kj[f]=g<<7,e=0;e<1<<Cj[f]-7;e++)Hj[256+g++]=f;for(e=0;15>=e;e++)d[e]=0;for(e=0;143>=e;)Fj[2*e+1]=8,e++,d[8]++;for(;255>=e;)Fj[2*e+1]=9,e++,d[9]++;for(;279>=e;)Fj[2*e+1]=7,e++,d[7]++;for(;287>=e;)Fj[2*e+1]=8,e++,d[8]++;Uj(Fj,287,d);for(e=0;30>e;e++)Gj[2*e+1]=5,Gj[2*e]=Tj(e,5);Mj=new Lj(Fj,Bj,257,286,15);Nj=new Lj(Gj,
Cj,0,30,15);Oj=new Lj([],Dj,0,19,7);fk=!0}c.hc=new Pj(c.ra,Mj);c.ac=new Pj(c.ab,Nj);c.Tc=new Pj(c.ka,Oj);c.oa=0;c.ja=0;Vj(c);c=0}else c=hk(b,-2);0===c&&(b=b.state,b.Ed=2*b.ma,ik(b.head),b.Cc=tk[b.level].te,b.dd=tk[b.level].ee,b.kd=tk[b.level].ye,b.jd=tk[b.level].se,b.o=0,b.ta=0,b.u=0,b.sa=0,b.P=b.wa=2,b.eb=0,b.J=0);b=c}}else b=-2;if(0!==b)throw Error(zj[b]);a.header&&(b=this.I)&&b.state&&2===b.state.wrap&&(b.state.G=a.header);if(a.dictionary){var l;"string"===typeof a.dictionary?l=sj(a.dictionary):
"[object ArrayBuffer]"===xk.call(a.dictionary)?l=new Uint8Array(a.dictionary):l=a.dictionary;a=this.I;f=l;g=f.length;if(a&&a.state)if(l=a.state,b=l.wrap,2===b||1===b&&42!==l.status||l.u)b=-2;else{1===b&&(a.H=tj(a.H,f,g,0));l.wrap=0;g>=l.ma&&(0===b&&(ik(l.head),l.o=0,l.ta=0,l.sa=0),c=new O.kb(l.ma),O.mb(c,f,g-l.ma,l.ma,0),f=c,g=l.ma);c=a.na;d=a.gb;e=a.input;a.na=g;a.gb=0;a.input=f;for(nk(l);3<=l.u;){f=l.o;g=l.u-2;do l.J=(l.J<<l.La^l.window[f+3-1])&l.Ka,l.Ha[f&l.Wa]=l.head[l.J],l.head[l.J]=f,f++;while(--g);
l.o=f;l.u=2;nk(l)}l.o+=l.u;l.ta=l.o;l.sa=l.u;l.u=0;l.P=l.wa=2;l.eb=0;a.gb=d;a.input=e;a.na=c;l.wrap=b;b=0}else b=-2;if(0!==b)throw Error(zj[b]);this.yg=!0}}
yk.prototype.push=function(a,b){var c=this.I,d=this.options.chunkSize;if(this.ended)return!1;var e=b===~~b?b:!0===b?4:0;"string"===typeof a?c.input=sj(a):"[object ArrayBuffer]"===xk.call(a)?c.input=new Uint8Array(a):c.input=a;c.gb=0;c.na=c.input.length;do{0===c.M&&(c.output=new O.kb(d),c.vb=0,c.M=d);a=vk(c,e);if(1!==a&&0!==a)return zk(this,a),this.ended=!0,!1;if(0===c.M||0===c.na&&(4===e||2===e))if("string"===this.options.to){var f=O.Mc(c.output,c.vb);b=f;f=f.length;if(65537>f&&(b.subarray&&rj||!b.subarray))b=
String.fromCharCode.apply(null,O.Mc(b,f));else{for(var g="",h=0;h<f;h++)g+=String.fromCharCode(b[h]);b=g}this.chunks.push(b)}else b=O.Mc(c.output,c.vb),this.chunks.push(b)}while((0<c.na||0===c.M)&&1!==a);if(4===e)return(c=this.I)&&c.state?(d=c.state.status,42!==d&&69!==d&&73!==d&&91!==d&&103!==d&&113!==d&&666!==d?a=hk(c,-2):(c.state=null,a=113===d?hk(c,-3):0)):a=-2,zk(this,a),this.ended=!0,0===a;2===e&&(zk(this,0),c.M=0);return!0};
function zk(a,b){0===b&&(a.result="string"===a.options.to?a.chunks.join(""):O.bd(a.chunks));a.chunks=[];a.err=b;a.msg=a.I.msg}
function Ak(a,b){b=b||{};b.gzip=!0;b=new yk(b);b.push(a,!0);if(b.err)throw b.msg||zj[b.err];return b.result}
;function Bk(a){if(!a)return null;a=a.privateDoNotAccessOrElseTrustedResourceUrlWrappedValue;var b;a?b=mb(a):b=null;return b}
;function Ck(a){return mb(null===a?"null":void 0===a?"undefined":a)}
;function Dk(a){this.name=a}
;var Ek=new Dk("rawColdConfigGroup");var Fk=new Dk("rawHotConfigGroup");function Gk(a){this.D=I(a)}
x(Gk,L);var Hk=new Dk("continuationCommand");var Ik=new Dk("webCommandMetadata");var Jk=new Dk("signalServiceEndpoint");var Kk={Cf:"EMBEDDED_PLAYER_MODE_UNKNOWN",zf:"EMBEDDED_PLAYER_MODE_DEFAULT",Bf:"EMBEDDED_PLAYER_MODE_PFP",Af:"EMBEDDED_PLAYER_MODE_PFL"};var Lk=new Dk("feedbackEndpoint");function Mk(a){this.D=I(a)}
x(Mk,L);Mk.prototype.setTrackingParams=function(a){if(null!=a)if("string"===typeof a)a=a?new We(a,Te):Ue||(Ue=new We(null,Te));else if(a.constructor!==We)if(Se(a))a=a.length?new We(new Uint8Array(a),Te):Ue||(Ue=new We(null,Te));else throw Error();return J(this,1,a)};var Ii={dg:"GENERIC_CLIENT_EXPERIMENT_EVENT_TYPE_UNKNOWN",Mf:"GENERIC_CLIENT_EXPERIMENT_EVENT_TYPE_FOR_TESTING",Tf:"GENERIC_CLIENT_EXPERIMENT_EVENT_TYPE_RESUME_TO_HOME_TTL",Xf:"GENERIC_CLIENT_EXPERIMENT_EVENT_TYPE_START_TO_SHORTS_ANALYSIS_SLICE",Jf:"GENERIC_CLIENT_EXPERIMENT_EVENT_TYPE_DEVICE_LAYER_SLICE",cg:"GENERIC_CLIENT_EXPERIMENT_EVENT_TYPE_UNIFIED_LAYER_SLICE",eg:"GENERIC_CLIENT_EXPERIMENT_EVENT_TYPE_VISITOR_LAYER_SLICE",Wf:"GENERIC_CLIENT_EXPERIMENT_EVENT_TYPE_SHOW_SHEET_COMMAND_HANDLER_BLOCK",
gg:"GENERIC_CLIENT_EXPERIMENT_EVENT_TYPE_WIZ_NEXT_MIGRATED_COMPONENT",fg:"GENERIC_CLIENT_EXPERIMENT_EVENT_TYPE_WIZ_NEXT_CHANNEL_NAME_TOOLTIP",Uf:"GENERIC_CLIENT_EXPERIMENT_EVENT_TYPE_ROTATION_LOCK_SUPPORTED",Zf:"GENERIC_CLIENT_EXPERIMENT_EVENT_TYPE_THEATER_MODE_ENABLED",kg:"GENERIC_CLIENT_EXPERIMENT_EVENT_TYPE_WOULD_SHOW_PIN_SUGGESTION",jg:"GENERIC_CLIENT_EXPERIMENT_EVENT_TYPE_WOULD_SHOW_LONG_PRESS_EDU_TOAST",ig:"GENERIC_CLIENT_EXPERIMENT_EVENT_TYPE_WOULD_SHOW_AMBIENT",ag:"GENERIC_CLIENT_EXPERIMENT_EVENT_TYPE_TIME_WATCHED_PANEL",
Vf:"GENERIC_CLIENT_EXPERIMENT_EVENT_TYPE_SEARCH_FROM_SEARCH_BAR_OVERLAY",lg:"GENERIC_CLIENT_EXPERIMENT_EVENT_TYPE_WOULD_SHOW_VOICE_SEARCH_EDU_TOAST",Yf:"GENERIC_CLIENT_EXPERIMENT_EVENT_TYPE_SUGGESTED_LANGUAGE_SELECTED",mg:"GENERIC_CLIENT_EXPERIMENT_EVENT_TYPE_WOULD_TRIGGER_SHORTS_PIP",Nf:"GENERIC_CLIENT_EXPERIMENT_EVENT_TYPE_IN_ZP_VOICE_CRASHY_SET",Pf:"GENERIC_CLIENT_EXPERIMENT_EVENT_TYPE_REEL_FAST_SWIPE_SUPPRESSED",Of:"GENERIC_CLIENT_EXPERIMENT_EVENT_TYPE_REEL_FAST_SWIPE_ALLOWED",Rf:"GENERIC_CLIENT_EXPERIMENT_EVENT_TYPE_REEL_PULL_TO_REFRESH_ATTEMPT",
hg:"GENERIC_CLIENT_EXPERIMENT_EVENT_TYPE_WOULD_BLOCK_KABUKI",Sf:"GENERIC_CLIENT_EXPERIMENT_EVENT_TYPE_REEL_TALL_SCREEN",Qf:"GENERIC_CLIENT_EXPERIMENT_EVENT_TYPE_REEL_NORMAL_SCREEN",Gf:"GENERIC_CLIENT_EXPERIMENT_EVENT_TYPE_ACCESSIBILITY_MODE_ENABLED",Ff:"GENERIC_CLIENT_EXPERIMENT_EVENT_TYPE_ACCESSIBILITY_MODE_DISABLED",Hf:"GENERIC_CLIENT_EXPERIMENT_EVENT_TYPE_AUTOPLAY_ENABLED",If:"GENERIC_CLIENT_EXPERIMENT_EVENT_TYPE_CAST_MATCH_OCCURRED",Kf:"GENERIC_CLIENT_EXPERIMENT_EVENT_TYPE_EMC3DS_ELIGIBLE",Lf:"GENERIC_CLIENT_EXPERIMENT_EVENT_TYPE_ENDSCREEN_TRIGGERED"};var Nk=new Dk("webPlayerShareEntityServiceEndpoint");var Ok=new Dk("playlistEditEndpoint");var Pk=new Dk("modifyChannelNotificationPreferenceEndpoint");var Qk=new Dk("unsubscribeEndpoint");var Rk=new Dk("subscribeEndpoint");function Sk(){var a=Tk;E("yt.ads.biscotti.getId_")||D("yt.ads.biscotti.getId_",a)}
function Uk(a){D("yt.ads.biscotti.lastId_",a)}
;function Vk(a,b){1<b.length?a[b[0]]=b[1]:1===b.length&&Object.assign(a,b[0])}
;var Wk=C.window,Xk,Yk,Zk=(null==Wk?void 0:null==(Xk=Wk.yt)?void 0:Xk.config_)||(null==Wk?void 0:null==(Yk=Wk.ytcfg)?void 0:Yk.data_)||{};D("yt.config_",Zk);function $k(){Vk(Zk,arguments)}
function T(a,b){return a in Zk?Zk[a]:b}
function al(a){var b=Zk.EXPERIMENT_FLAGS;return b?b[a]:void 0}
;var bl=[];function cl(a){bl.forEach(function(b){return b(a)})}
function dl(a){return a&&window.yterr?function(){try{return a.apply(this,arguments)}catch(b){el(b)}}:a}
function el(a){var b=E("yt.logging.errors.log");b?b(a,"ERROR",void 0,void 0,void 0,void 0,void 0):(b=T("ERRORS",[]),b.push([a,"ERROR",void 0,void 0,void 0,void 0,void 0]),$k("ERRORS",b));cl(a)}
function fl(a,b,c,d,e){var f=E("yt.logging.errors.log");f?f(a,"WARNING",b,c,d,void 0,e):(f=T("ERRORS",[]),f.push([a,"WARNING",b,c,d,void 0,e]),$k("ERRORS",f))}
;var gl=/^[\w.]*$/,hl={q:!0,search_query:!0};function il(a,b){b=a.split(b);for(var c={},d=0,e=b.length;d<e;d++){var f=b[d].split("=");if(1===f.length&&f[0]||2===f.length)try{var g=jl(f[0]||""),h=jl(f[1]||"");if(g in c){var k=c[g];Array.isArray(k)?Jb(k,h):c[g]=[k,h]}else c[g]=h}catch(r){var l=r,n=f[0],p=String(il);l.args=[{key:n,value:f[1],query:a,method:kl===p?"unchanged":p}];hl.hasOwnProperty(n)||fl(l)}}return c}
var kl=String(il);function ll(a){var b=[];Kb(a,function(c,d){var e=encodeURIComponent(String(d));c=Array.isArray(c)?c:[c];Db(c,function(f){""==f?b.push(e):b.push(e+"="+encodeURIComponent(String(f)))})});
return b.join("&")}
function ml(a){"?"===a.charAt(0)&&(a=a.substring(1));return il(a,"&")}
function nl(a){return-1!==a.indexOf("?")?(a=(a||"").split("#")[0],a=a.split("?",2),ml(1<a.length?a[1]:a[0])):{}}
function ol(a,b,c){var d=a.split("#",2);a=d[0];d=1<d.length?"#"+d[1]:"";var e=a.split("?",2);a=e[0];e=ml(e[1]||"");for(var f in b)!c&&null!==e&&f in e||(e[f]=b[f]);return tc(a,e)+d}
function pl(a){if(!b)var b=window.location.href;var c=nc(1,a),d=oc(a);c&&d?(a=a.match(lc),b=b.match(lc),a=a[3]==b[3]&&a[1]==b[1]&&a[4]==b[4]):a=d?oc(b)===d&&(Number(nc(4,b))||null)===(Number(nc(4,a))||null):!0;return a}
function jl(a){return a&&a.match(gl)?a:decodeURIComponent(a.replace(/\+/g," "))}
;function ql(a){var b=rl;a=void 0===a?E("yt.ads.biscotti.lastId_")||"":a;var c=Object,d=c.assign,e={};e.dt=Gi;e.flash="0";a:{try{var f=b.h.top.location.href}catch(Ka){f=2;break a}f=f?f===b.i.location.href?0:1:2}e=(e.frm=f,e);try{e.u_tz=-(new Date).getTimezoneOffset();var g=void 0===g?yi:g;try{var h=g.history.length}catch(Ka){h=0}e.u_his=h;var k;e.u_h=null==(k=yi.screen)?void 0:k.height;var l;e.u_w=null==(l=yi.screen)?void 0:l.width;var n;e.u_ah=null==(n=yi.screen)?void 0:n.availHeight;var p;e.u_aw=
null==(p=yi.screen)?void 0:p.availWidth;var r;e.u_cd=null==(r=yi.screen)?void 0:r.colorDepth}catch(Ka){}h=b.h;try{var t=h.screenX;var y=h.screenY}catch(Ka){}try{var v=h.outerWidth;var z=h.outerHeight}catch(Ka){}try{var G=h.innerWidth;var K=h.innerHeight}catch(Ka){}try{var N=h.screenLeft;var S=h.screenTop}catch(Ka){}try{G=h.innerWidth,K=h.innerHeight}catch(Ka){}try{var da=h.screen.availWidth;var sa=h.screen.availTop}catch(Ka){}t=[N,S,t,y,da,sa,v,z,G,K];try{var P=(b.h.top||window).document,ea="CSS1Compat"==
P.compatMode?P.documentElement:P.body;var na=(new Ed(ea.clientWidth,ea.clientHeight)).round()}catch(Ka){na=new Ed(-12245933,-12245933)}P=na;na={};var La=void 0===La?C:La;ea=new Oi;"SVGElement"in La&&"createElementNS"in La.document&&ea.set(0);y=Di();y["allow-top-navigation-by-user-activation"]&&ea.set(1);y["allow-popups-to-escape-sandbox"]&&ea.set(2);La.crypto&&La.crypto.subtle&&ea.set(3);"TextDecoder"in La&&"TextEncoder"in La&&ea.set(4);La=Pi(ea);na.bc=La;na.bih=P.height;na.biw=P.width;na.brdim=t.join();
b=b.i;b=(na.vis=b.prerendering?3:{visible:1,hidden:2,prerender:3,preview:4,unloaded:5}[b.visibilityState||b.webkitVisibilityState||b.mozVisibilityState||""]||0,na.wgl=!!yi.WebGLRenderingContext,na);c=d.call(c,e,b);c.ca_type="image";a&&(c.bid=a);return c}
var rl=new function(){var a=window.document;this.h=window;this.i=a};
D("yt.ads_.signals_.getAdSignalsString",function(a){return ll(ql(a))});Za();navigator.userAgent.indexOf(" (CrKey ");var sl="XMLHttpRequest"in C?function(){return new XMLHttpRequest}:null;
function tl(){if(!sl)return null;var a=sl();return"open"in a?a:null}
function ul(a){switch(a&&"status"in a?a.status:-1){case 200:case 201:case 202:case 203:case 204:case 205:case 206:case 304:return!0;default:return!1}}
;function vl(a,b){"function"===typeof a&&(a=dl(a));return window.setTimeout(a,b)}
;var wl="client_dev_domain client_dev_expflag client_dev_regex_map client_dev_root_url client_rollout_override expflag forcedCapability jsfeat jsmode mods".split(" ");[].concat(la(wl),["client_dev_set_cookie"]);function U(a){a=xl(a);return"string"===typeof a&&"false"===a?!1:!!a}
function yl(a,b){a=xl(a);return void 0===a&&void 0!==b?b:Number(a||0)}
function xl(a){return T("EXPERIMENT_FLAGS",{})[a]}
function zl(){for(var a=[],b=T("EXPERIMENTS_FORCED_FLAGS",{}),c=w(Object.keys(b)),d=c.next();!d.done;d=c.next())d=d.value,a.push({key:d,value:String(b[d])});c=T("EXPERIMENT_FLAGS",{});d=w(Object.keys(c));for(var e=d.next();!e.done;e=d.next())e=e.value,e.startsWith("force_")&&void 0===b[e]&&a.push({key:e,value:String(c[e])});return a}
;var Al={Authorization:"AUTHORIZATION","X-Goog-EOM-Visitor-Id":"EOM_VISITOR_DATA","X-Goog-Visitor-Id":"SANDBOXED_VISITOR_ID","X-Youtube-Domain-Admin-State":"DOMAIN_ADMIN_STATE","X-Youtube-Chrome-Connected":"CHROME_CONNECTED_HEADER","X-YouTube-Client-Name":"INNERTUBE_CONTEXT_CLIENT_NAME","X-YouTube-Client-Version":"INNERTUBE_CONTEXT_CLIENT_VERSION","X-YouTube-Delegation-Context":"INNERTUBE_CONTEXT_SERIALIZED_DELEGATION_CONTEXT","X-YouTube-Device":"DEVICE","X-Youtube-Identity-Token":"ID_TOKEN","X-YouTube-Page-CL":"PAGE_CL",
"X-YouTube-Page-Label":"PAGE_BUILD_LABEL","X-YouTube-Variants-Checksum":"VARIANTS_CHECKSUM","X-Goog-AuthUser":"SESSION_INDEX","X-Goog-PageId":"DELEGATED_SESSION_ID"},Bl="app debugcss debugjs expflag force_ad_params force_ad_encrypted force_viral_ad_response_params forced_experiments innertube_snapshots innertube_goldens internalcountrycode internalipoverride absolute_experiments conditional_experiments sbb sr_bns_address".split(" ").concat(la(wl)),Cl=!1;
function Dl(a,b,c,d,e,f,g,h){function k(){4===(l&&"readyState"in l?l.readyState:0)&&b&&dl(b)(l)}
c=void 0===c?"GET":c;d=void 0===d?"":d;h=void 0===h?!1:h;var l=tl();if(!l)return null;"onloadend"in l?l.addEventListener("loadend",k,!1):l.onreadystatechange=k;U("debug_forward_web_query_parameters")&&(a=El(a));l.open(c,a,!0);f&&(l.responseType=f);g&&(l.withCredentials=!0);c="POST"===c&&(void 0===window.FormData||!(d instanceof FormData));if(e=Fl(a,e))for(var n in e)l.setRequestHeader(n,e[n]),"content-type"===n.toLowerCase()&&(c=!1);c&&l.setRequestHeader("Content-Type","application/x-www-form-urlencoded");
if(h&&"setAttributionReporting"in XMLHttpRequest.prototype){a={eventSourceEligible:!0,triggerEligible:!1};try{l.setAttributionReporting(a)}catch(p){fl(p)}}l.send(d);return l}
function Fl(a,b){b=void 0===b?{}:b;var c=pl(a),d=U("web_ajax_ignore_global_headers_if_set"),e;for(e in Al){var f=T(Al[e]),g="X-Goog-AuthUser"===e||"X-Goog-PageId"===e;"X-Goog-Visitor-Id"!==e||f||(f=T("VISITOR_DATA"));!f||!c&&oc(a)||d&&void 0!==b[e]||"TVHTML5_UNPLUGGED"===T("INNERTUBE_CLIENT_NAME")&&g||(b[e]=f)}"X-Goog-EOM-Visitor-Id"in b&&"X-Goog-Visitor-Id"in b&&delete b["X-Goog-Visitor-Id"];if(c||!oc(a))b["X-YouTube-Utc-Offset"]=String(-(new Date).getTimezoneOffset());if(c||!oc(a)){try{var h=(new Intl.DateTimeFormat).resolvedOptions().timeZone}catch(k){}h&&
(b["X-YouTube-Time-Zone"]=h)}document.location.hostname.endsWith("youtubeeducation.com")||!c&&oc(a)||(b["X-YouTube-Ad-Signals"]=ll(ql()));return b}
function Gl(a,b){b.method="POST";b.postParams||(b.postParams={});return Hl(a,b)}
function Hl(a,b){var c=b.format||"JSON";a=Il(a,b);var d=Jl(a,b),e=!1,f=Kl(a,function(k){if(!e){e=!0;h&&window.clearTimeout(h);var l=ul(k),n=null,p=400<=k.status&&500>k.status,r=500<=k.status&&600>k.status;if(l||p||r)n=Ll(a,c,k,b.convertToSafeHtml);l&&(l=Ml(c,k,n));n=n||{};p=b.context||C;l?b.onSuccess&&b.onSuccess.call(p,k,n):b.onError&&b.onError.call(p,k,n);b.onFinish&&b.onFinish.call(p,k,n)}},b.method,d,b.headers,b.responseType,b.withCredentials);
d=b.timeout||0;if(b.onTimeout&&0<d){var g=b.onTimeout;var h=vl(function(){e||(e=!0,f.abort(),window.clearTimeout(h),g.call(b.context||C,f))},d)}return f}
function Il(a,b){b.includeDomain&&(a=document.location.protocol+"//"+document.location.hostname+(document.location.port?":"+document.location.port:"")+a);var c=T("XSRF_FIELD_NAME");if(b=b.urlParams)b[c]&&delete b[c],a=ol(a,b||{},!0);return a}
function Jl(a,b){var c=T("XSRF_FIELD_NAME"),d=T("XSRF_TOKEN"),e=b.postBody||"",f=b.postParams,g=T("XSRF_FIELD_NAME"),h;b.headers&&(h=b.headers["Content-Type"]);b.excludeXsrf||oc(a)&&!b.withCredentials&&oc(a)!==document.location.hostname||"POST"!==b.method||h&&"application/x-www-form-urlencoded"!==h||b.postParams&&b.postParams[g]||(f||(f={}),f[c]=d);(U("ajax_parse_query_data_only_when_filled")&&f&&0<Object.keys(f).length||f)&&"string"===typeof e&&(e=ml(e),Ub(e,f),e=b.postBodyFormat&&"JSON"===b.postBodyFormat?
JSON.stringify(e):sc(e));f=e||f&&!Nb(f);!Cl&&f&&"POST"!==b.method&&(Cl=!0,el(Error("AJAX request with postData should use POST")));return e}
function Ll(a,b,c,d){var e=null;switch(b){case "JSON":try{var f=c.responseText}catch(g){throw d=Error("Error reading responseText"),d.params=a,fl(d),g;}a=c.getResponseHeader("Content-Type")||"";f&&0<=a.indexOf("json")&&(")]}'\n"===f.substring(0,5)&&(f=f.substring(5)),e=JSON.parse(f));break;case "XML":if(a=(a=c.responseXML)?Nl(a):null)e={},Db(a.getElementsByTagName("*"),function(g){e[g.tagName]=Ol(g)})}d&&Pl(e);
return e}
function Pl(a){if(Ra(a))for(var b in a){var c;(c="html_content"===b)||(c=b.length-5,c=0<=c&&b.indexOf("_html",c)==c);if(c){c=b;var d=a[b],e=fb();d=e?e.createHTML(d):d;a[c]=new Vb(d)}else Pl(a[b])}}
function Ml(a,b,c){if(b&&204===b.status)return!0;switch(a){case "JSON":return!!c;case "XML":return 0===Number(c&&c.return_code);case "RAW":return!0;default:return!!c}}
function Nl(a){return a?(a=("responseXML"in a?a.responseXML:a).getElementsByTagName("root"))&&0<a.length?a[0]:null:null}
function Ol(a){var b="";Db(a.childNodes,function(c){b+=c.nodeValue});
return b}
function El(a){var b=window.location.search,c=oc(a);U("debug_handle_relative_url_for_query_forward_killswitch")||!c&&pl(a)&&(c=document.location.hostname);var d=mc(nc(5,a));d=(c=c&&(c.endsWith("youtube.com")||c.endsWith("youtube-nocookie.com")))&&d&&d.startsWith("/api/");if(!c||d)return a;var e=ml(b),f={};Db(Bl,function(g){e[g]&&(f[g]=e[g])});
return ol(a,f||{},!1)}
var Kl=Dl;var Ql=[{Dc:function(a){return"Cannot read property '"+a.key+"'"},
jc:{Error:[{regexp:/(Permission denied) to access property "([^']+)"/,groups:["reason","key"]}],TypeError:[{regexp:/Cannot read property '([^']+)' of (null|undefined)/,groups:["key","value"]},{regexp:/\u65e0\u6cd5\u83b7\u53d6\u672a\u5b9a\u4e49\u6216 (null|undefined) \u5f15\u7528\u7684\u5c5e\u6027\u201c([^\u201d]+)\u201d/,groups:["value","key"]},{regexp:/\uc815\uc758\ub418\uc9c0 \uc54a\uc74c \ub610\ub294 (null|undefined) \ucc38\uc870\uc778 '([^']+)' \uc18d\uc131\uc744 \uac00\uc838\uc62c \uc218 \uc5c6\uc2b5\ub2c8\ub2e4./,
groups:["value","key"]},{regexp:/No se puede obtener la propiedad '([^']+)' de referencia nula o sin definir/,groups:["key"]},{regexp:/Unable to get property '([^']+)' of (undefined or null) reference/,groups:["key","value"]},{regexp:/(null) is not an object \(evaluating '(?:([^.]+)\.)?([^']+)'\)/,groups:["value","base","key"]}]}},{Dc:function(a){return"Cannot call '"+a.key+"'"},
jc:{TypeError:[{regexp:/(?:([^ ]+)?\.)?([^ ]+) is not a function/,groups:["base","key"]},{regexp:/([^ ]+) called on (null or undefined)/,groups:["key","value"]},{regexp:/Object (.*) has no method '([^ ]+)'/,groups:["base","key"]},{regexp:/Object doesn't support property or method '([^ ]+)'/,groups:["key"]},{regexp:/\u30aa\u30d6\u30b8\u30a7\u30af\u30c8\u306f '([^']+)' \u30d7\u30ed\u30d1\u30c6\u30a3\u307e\u305f\u306f\u30e1\u30bd\u30c3\u30c9\u3092\u30b5\u30dd\u30fc\u30c8\u3057\u3066\u3044\u307e\u305b\u3093/,
groups:["key"]},{regexp:/\uac1c\uccb4\uac00 '([^']+)' \uc18d\uc131\uc774\ub098 \uba54\uc11c\ub4dc\ub97c \uc9c0\uc6d0\ud558\uc9c0 \uc54a\uc2b5\ub2c8\ub2e4./,groups:["key"]}]}},{Dc:function(a){return a.key+" is not defined"},
jc:{ReferenceError:[{regexp:/(.*) is not defined/,groups:["key"]},{regexp:/Can't find variable: (.*)/,groups:["key"]}]}}];var Sl={Ua:[],Ra:[{callback:Rl,weight:500}]};function Rl(a){if("JavaException"===a.name)return!0;a=a.stack;return a.includes("chrome://")||a.includes("chrome-extension://")||a.includes("moz-extension://")}
;function Tl(){this.Ra=[];this.Ua=[]}
var Ul;function Vl(){if(!Ul){var a=Ul=new Tl;a.Ua.length=0;a.Ra.length=0;Sl.Ua&&a.Ua.push.apply(a.Ua,Sl.Ua);Sl.Ra&&a.Ra.push.apply(a.Ra,Sl.Ra)}return Ul}
;var Wl=new M;function Xl(a){function b(){return a.charCodeAt(d++)}
var c=a.length,d=0;do{var e=Yl(b);if(Infinity===e)break;var f=e>>3;switch(e&7){case 0:e=Yl(b);if(2===f)return e;break;case 1:if(2===f)return;d+=8;break;case 2:e=Yl(b);if(2===f)return a.substr(d,e);d+=e;break;case 5:if(2===f)return;d+=4;break;default:return}}while(d<c)}
function Yl(a){var b=a(),c=b&127;if(128>b)return c;b=a();c|=(b&127)<<7;if(128>b)return c;b=a();c|=(b&127)<<14;if(128>b)return c;b=a();return 128>b?c|(b&127)<<21:Infinity}
;function Zl(a,b,c,d){if(a)if(Array.isArray(a)){var e=d;for(d=0;d<a.length&&!(a[d]&&(e+=$l(d,a[d],b,c),500<e));d++);d=e}else if("object"===typeof a)for(e in a){if(a[e]){var f=e;var g=a[e],h=b,k=c;f="string"!==typeof g||"clickTrackingParams"!==f&&"trackingParams"!==f?0:(g=Xl(atob(g.replace(/-/g,"+").replace(/_/g,"/"))))?$l(f+".ve",g,h,k):0;d+=f;d+=$l(e,a[e],b,c);if(500<d)break}}else c[b]=am(a),d+=c[b].length;else c[b]=am(a),d+=c[b].length;return d}
function $l(a,b,c,d){c+="."+a;a=am(b);d[c]=a;return c.length+a.length}
function am(a){try{return("string"===typeof a?a:String(JSON.stringify(a))).substr(0,500)}catch(b){return"unable to serialize "+typeof a+" ("+b.message+")"}}
;function bm(a){var b=this;this.i=void 0;this.h=!1;a.addEventListener("beforeinstallprompt",function(c){c.preventDefault();b.i=c});
a.addEventListener("appinstalled",function(){b.h=!0},{once:!0})}
function cm(){if(!C.matchMedia)return"WEB_DISPLAY_MODE_UNKNOWN";try{return C.matchMedia("(display-mode: standalone)").matches?"WEB_DISPLAY_MODE_STANDALONE":C.matchMedia("(display-mode: minimal-ui)").matches?"WEB_DISPLAY_MODE_MINIMAL_UI":C.matchMedia("(display-mode: fullscreen)").matches?"WEB_DISPLAY_MODE_FULLSCREEN":C.matchMedia("(display-mode: browser)").matches?"WEB_DISPLAY_MODE_BROWSER":"WEB_DISPLAY_MODE_UNKNOWN"}catch(a){return"WEB_DISPLAY_MODE_UNKNOWN"}}
;function dm(){this.bf=!0}
function em(){dm.h||(dm.h=new dm);return dm.h}
function fm(a,b){a={};var c=[];"SESSION_ID"in Zk&&c.push({key:"u",value:T("SESSION_ID")});if(c=mh(c))a.Authorization=c,c=b=null==b?void 0:b.sessionIndex,void 0===c&&(c=Number(T("SESSION_INDEX",0)),c=isNaN(c)?0:c),U("voice_search_auth_header_removal")||(a["X-Goog-AuthUser"]=c.toString()),"INNERTUBE_HOST_OVERRIDE"in Zk||(a["X-Origin"]=window.location.origin),void 0===b&&"DELEGATED_SESSION_ID"in Zk&&(a["X-Goog-PageId"]=T("DELEGATED_SESSION_ID"));return a}
;var gm={identityType:"UNAUTHENTICATED_IDENTITY_TYPE_UNKNOWN"};function hm(a,b,c,d,e){ih.set(""+a,b,{Mb:c,path:"/",domain:void 0===d?"youtube.com":d,secure:void 0===e?!1:e})}
function im(a){return ih.get(""+a,void 0)}
function jm(a,b,c){ih.remove(""+a,void 0===b?"/":b,void 0===c?"youtube.com":c)}
function km(){if(U("embeds_web_enable_cookie_detection_fix")){if(!C.navigator.cookieEnabled)return!1}else if(!ih.isEnabled())return!1;if(ih.h.cookie)return!0;U("embeds_web_enable_cookie_detection_fix")?ih.set("TESTCOOKIESENABLED","1",{Mb:60,Le:"none",secure:!0}):ih.set("TESTCOOKIESENABLED","1",{Mb:60});if("1"!==ih.get("TESTCOOKIESENABLED"))return!1;ih.remove("TESTCOOKIESENABLED");return!0}
;var lm=E("ytglobal.prefsUserPrefsPrefs_")||{};D("ytglobal.prefsUserPrefsPrefs_",lm);function mm(){this.h=T("ALT_PREF_COOKIE_NAME","PREF");this.i=T("ALT_PREF_COOKIE_DOMAIN","youtube.com");var a=im(this.h);a&&this.parse(a)}
var nm;function om(){nm||(nm=new mm);return nm}
m=mm.prototype;m.get=function(a,b){pm(a);qm(a);a=void 0!==lm[a]?lm[a].toString():null;return null!=a?a:b?b:""};
m.set=function(a,b){pm(a);qm(a);if(null==b)throw Error("ExpectedNotNull");lm[a]=b.toString()};
function rm(a){return!!((sm("f"+(Math.floor(a/31)+1))||0)&1<<a%31)}
m.remove=function(a){pm(a);qm(a);delete lm[a]};
m.clear=function(){for(var a in lm)delete lm[a]};
function qm(a){if(/^f([1-9][0-9]*)$/.test(a))throw Error("ExpectedRegexMatch: "+a);}
function pm(a){if(!/^\w+$/.test(a))throw Error("ExpectedRegexMismatch: "+a);}
function sm(a){a=void 0!==lm[a]?lm[a].toString():null;return null!=a&&/^[A-Fa-f0-9]+$/.test(a)?parseInt(a,16):null}
m.parse=function(a){a=decodeURIComponent(a).split("&");for(var b=0;b<a.length;b++){var c=a[b].split("="),d=c[0];(c=c[1])&&(lm[d]=c.toString())}};var tm={bluetooth:"CONN_DISCO",cellular:"CONN_CELLULAR_UNKNOWN",ethernet:"CONN_WIFI",none:"CONN_NONE",wifi:"CONN_WIFI",wimax:"CONN_CELLULAR_4G",other:"CONN_UNKNOWN",unknown:"CONN_UNKNOWN","slow-2g":"CONN_CELLULAR_2G","2g":"CONN_CELLULAR_2G","3g":"CONN_CELLULAR_3G","4g":"CONN_CELLULAR_4G"},um={"slow-2g":"EFFECTIVE_CONNECTION_TYPE_SLOW_2G","2g":"EFFECTIVE_CONNECTION_TYPE_2G","3g":"EFFECTIVE_CONNECTION_TYPE_3G","4g":"EFFECTIVE_CONNECTION_TYPE_4G"};
function wm(){var a=C.navigator;return a?a.connection:void 0}
function xm(){var a=wm();if(a){var b=tm[a.type||"unknown"]||"CONN_UNKNOWN";a=tm[a.effectiveType||"unknown"]||"CONN_UNKNOWN";"CONN_CELLULAR_UNKNOWN"===b&&"CONN_UNKNOWN"!==a&&(b=a);if("CONN_UNKNOWN"!==b)return b;if("CONN_UNKNOWN"!==a)return a}}
function ym(){var a=wm();if(null!=a&&a.effectiveType)return um.hasOwnProperty(a.effectiveType)?um[a.effectiveType]:"EFFECTIVE_CONNECTION_TYPE_UNKNOWN"}
;function V(a){var b=B.apply(1,arguments);var c=Error.call(this,a);this.message=c.message;"stack"in c&&(this.stack=c.stack);this.args=[].concat(la(b))}
x(V,Error);function zm(){try{return Am(),!0}catch(a){return!1}}
function Am(a){if(void 0!==T("DATASYNC_ID"))return T("DATASYNC_ID");throw new V("Datasync ID not set",void 0===a?"unknown":a);}
;function Bm(){}
function Cm(a,b){return Ni.Za(a,0,b)}
Bm.prototype.pa=function(a,b){return this.Za(a,1,b)};
Bm.prototype.Cb=function(a){var b=E("yt.scheduler.instance.addImmediateJob");b?b(a):a()};var Dm=yl("web_emulated_idle_callback_delay",300),Em=1E3/60-3,Fm=[8,5,4,3,2,1,0];
function Gm(a){a=void 0===a?{}:a;F.call(this);this.i=[];this.j={};this.da=this.h=0;this.ba=this.m=!1;this.K=[];this.W=this.ga=!1;for(var b=w(Fm),c=b.next();!c.done;c=b.next())this.i[c.value]=[];this.l=0;this.uc=a.timeout||1;this.F=Em;this.v=0;this.xa=this.Ae.bind(this);this.sc=this.ef.bind(this);this.Ya=this.Kd.bind(this);this.Bb=this.ge.bind(this);this.Sb=this.De.bind(this);this.Fa=!!window.requestIdleCallback&&!!window.cancelIdleCallback&&!U("disable_scheduler_requestIdleCallback");(this.ia=!1!==
a.useRaf&&!!window.requestAnimationFrame)&&document.addEventListener("visibilitychange",this.xa)}
x(Gm,F);m=Gm.prototype;m.Cb=function(a){var b=Za();Hm(this,a);a=Za()-b;this.m||(this.F-=a)};
m.Za=function(a,b,c){++this.da;if(10===b)return this.Cb(a),this.da;var d=this.da;this.j[d]=a;this.m&&!c?this.K.push({id:d,priority:b}):(this.i[b].push(d),this.ba||this.m||(0!==this.h&&Im(this)!==this.v&&this.stop(),this.start()));return d};
m.qa=function(a){delete this.j[a]};
function Jm(a){a.K.length=0;for(var b=5;0<=b;b--)a.i[b].length=0;a.i[8].length=0;a.j={};a.stop()}
m.isHidden=function(){return!!document.hidden||!1};
function Km(a){return!a.isHidden()&&a.ia}
function Im(a){if(a.i[8].length){if(a.W)return 4;if(Km(a))return 3}for(var b=5;b>=a.l;b--)if(0<a.i[b].length)return 0<b?Km(a)?3:2:1;return 0}
m.Ga=function(a){var b=E("yt.logging.errors.log");b&&b(a)};
function Hm(a,b){try{b()}catch(c){a.Ga(c)}}
function Lm(a){for(var b=w(Fm),c=b.next();!c.done;c=b.next())if(a.i[c.value].length)return!0;return!1}
m.ge=function(a){var b=void 0;a&&(b=a.timeRemaining());this.ga=!0;Mm(this,b);this.ga=!1};
m.ef=function(){Mm(this)};
m.Kd=function(){Nm(this)};
m.De=function(a){this.W=!0;var b=Im(this);4===b&&b!==this.v&&(this.stop(),this.start());Mm(this,void 0,a);this.W=!1};
m.Ae=function(){this.isHidden()||Nm(this);this.h&&(this.stop(),this.start())};
function Nm(a){a.stop();a.m=!0;for(var b=Za(),c=a.i[8];c.length;){var d=c.shift(),e=a.j[d];delete a.j[d];e&&Hm(a,e)}Om(a);a.m=!1;Lm(a)&&a.start();b=Za()-b;a.F-=b}
function Om(a){for(var b=0,c=a.K.length;b<c;b++){var d=a.K[b];a.i[d.priority].push(d.id)}a.K.length=0}
function Mm(a,b,c){a.W&&4===a.v&&a.h||a.stop();a.m=!0;b=Za()+(b||a.F);for(var d=a.i[5];d.length;){var e=d.shift(),f=a.j[e];delete a.j[e];if(f){e=a;try{f(c)}catch(l){e.Ga(l)}}}for(d=a.i[4];d.length;)c=d.shift(),f=a.j[c],delete a.j[c],f&&Hm(a,f);d=a.ga?0:1;d=a.l>d?a.l:d;if(!(Za()>=b)){do{a:{c=a;f=d;for(e=3;e>=f;e--)for(var g=c.i[e];g.length;){var h=g.shift(),k=c.j[h];delete c.j[h];if(k){c=k;break a}}c=null}c&&Hm(a,c)}while(c&&Za()<b)}a.m=!1;Om(a);a.F=Em;Lm(a)&&a.start()}
m.start=function(){this.ba=!1;if(0===this.h)switch(this.v=Im(this),this.v){case 1:var a=this.Bb;this.h=this.Fa?window.requestIdleCallback(a,{timeout:3E3}):window.setTimeout(a,Dm);break;case 2:this.h=window.setTimeout(this.sc,this.uc);break;case 3:this.h=window.requestAnimationFrame(this.Sb);break;case 4:this.h=window.setTimeout(this.Ya,0)}};
m.pause=function(){this.stop();this.ba=!0};
m.stop=function(){if(this.h){switch(this.v){case 1:var a=this.h;this.Fa?window.cancelIdleCallback(a):window.clearTimeout(a);break;case 2:case 4:window.clearTimeout(this.h);break;case 3:window.cancelAnimationFrame(this.h)}this.h=0}};
m.U=function(){Jm(this);this.stop();this.ia&&document.removeEventListener("visibilitychange",this.xa);F.prototype.U.call(this)};var Pm=E("yt.scheduler.instance.timerIdMap_")||{},Qm=yl("kevlar_tuner_scheduler_soft_state_timer_ms",800),Rm=0,Sm=0;function Tm(){var a=E("ytglobal.schedulerInstanceInstance_");if(!a||a.V)a=new Gm(T("scheduler")||{}),D("ytglobal.schedulerInstanceInstance_",a);return a}
function Um(){Vm();var a=E("ytglobal.schedulerInstanceInstance_");a&&(Cc(a),D("ytglobal.schedulerInstanceInstance_",null))}
function Vm(){Jm(Tm());for(var a in Pm)Pm.hasOwnProperty(a)&&delete Pm[Number(a)]}
function Wm(a,b,c){if(!c)return c=void 0===c,-Tm().Za(a,b,c);var d=window.setTimeout(function(){var e=Tm().Za(a,b);Pm[d]=e},c);
return d}
function Xm(a){Tm().Cb(a)}
function Ym(a){var b=Tm();if(0>a)b.qa(-a);else{var c=Pm[a];c?(b.qa(c),delete Pm[a]):window.clearTimeout(a)}}
function Zm(){$m()}
function $m(){window.clearTimeout(Rm);Tm().start()}
function an(){Tm().pause();window.clearTimeout(Rm);Rm=window.setTimeout(Zm,Qm)}
function bn(){window.clearTimeout(Sm);Sm=window.setTimeout(function(){cn(0)},Qm)}
function cn(a){bn();var b=Tm();b.l=a;b.start()}
function dn(a){bn();var b=Tm();b.l>a&&(b.l=a,b.start())}
function en(){window.clearTimeout(Sm);var a=Tm();a.l=0;a.start()}
;function fn(){Bm.apply(this,arguments)}
x(fn,Bm);function gn(){fn.h||(fn.h=new fn);return fn.h}
fn.prototype.Za=function(a,b,c){void 0!==c&&Number.isNaN(Number(c))&&(c=void 0);var d=E("yt.scheduler.instance.addJob");return d?d(a,b,c):void 0===c?(a(),NaN):vl(a,c||0)};
fn.prototype.qa=function(a){if(void 0===a||!Number.isNaN(Number(a))){var b=E("yt.scheduler.instance.cancelJob");b?b(a):window.clearTimeout(a)}};
fn.prototype.start=function(){var a=E("yt.scheduler.instance.start");a&&a()};
fn.prototype.pause=function(){var a=E("yt.scheduler.instance.pause");a&&a()};
var Ni=gn();
U("web_scheduler_auto_init")&&!E("yt.scheduler.initialized")&&(D("yt.scheduler.instance.dispose",Um),D("yt.scheduler.instance.addJob",Wm),D("yt.scheduler.instance.addImmediateJob",Xm),D("yt.scheduler.instance.cancelJob",Ym),D("yt.scheduler.instance.cancelAllJobs",Vm),D("yt.scheduler.instance.start",$m),D("yt.scheduler.instance.pause",an),D("yt.scheduler.instance.setPriorityThreshold",cn),D("yt.scheduler.instance.enablePriorityThreshold",dn),D("yt.scheduler.instance.clearPriorityThreshold",en),D("yt.scheduler.initialized",
!0));function hn(a){var b=new mj;this.h=(a=b.isAvailable()?a?new nj(b,a):b:null)?new hj(a):null;this.i=document.domain||window.location.hostname}
hn.prototype.set=function(a,b,c,d){c=c||31104E3;this.remove(a);if(this.h)try{this.h.set(a,b,Date.now()+1E3*c);return}catch(f){}var e="";if(d)try{e=escape((new Yh).serialize(b))}catch(f){return}else e=escape(b);hm(a,e,c,this.i)};
hn.prototype.get=function(a,b){var c=void 0,d=!this.h;if(!d)try{c=this.h.get(a)}catch(e){d=!0}if(d&&(c=im(a))&&(c=unescape(c),b))try{c=JSON.parse(c)}catch(e){this.remove(a),c=void 0}return c};
hn.prototype.remove=function(a){this.h&&this.h.remove(a);jm(a,"/",this.i)};var jn=function(){var a;return function(){a||(a=new hn("ytidb"));return a}}();
function kn(){var a;return null==(a=jn())?void 0:a.get("LAST_RESULT_ENTRY_KEY",!0)}
;var ln=[],mn,nn=!1;function on(){var a={};for(mn=new pn(void 0===a.handleError?qn:a.handleError,void 0===a.logEvent?rn:a.logEvent);0<ln.length;)switch(a=ln.shift(),a.type){case "ERROR":mn.Ga(a.payload);break;case "EVENT":mn.logEvent(a.eventType,a.payload)}}
function sn(a){nn||(mn?mn.Ga(a):(ln.push({type:"ERROR",payload:a}),10<ln.length&&ln.shift()))}
function tn(a,b){nn||(mn?mn.logEvent(a,b):(ln.push({type:"EVENT",eventType:a,payload:b}),10<ln.length&&ln.shift()))}
;function un(a){if(0<=a.indexOf(":"))throw Error("Database name cannot contain ':'");}
function vn(a){return a.substr(0,a.indexOf(":"))||a}
;var wn=De||Ee;function xn(a){var b=Jc();return b?0<=b.toLowerCase().indexOf(a):!1}
;var yn={},zn=(yn.AUTH_INVALID="No user identifier specified.",yn.EXPLICIT_ABORT="Transaction was explicitly aborted.",yn.IDB_NOT_SUPPORTED="IndexedDB is not supported.",yn.MISSING_INDEX="Index not created.",yn.MISSING_OBJECT_STORES="Object stores not created.",yn.DB_DELETED_BY_MISSING_OBJECT_STORES="Database is deleted because expected object stores were not created.",yn.DB_REOPENED_BY_MISSING_OBJECT_STORES="Database is reopened because expected object stores were not created.",yn.UNKNOWN_ABORT="Transaction was aborted for unknown reasons.",
yn.QUOTA_EXCEEDED="The current transaction exceeded its quota limitations.",yn.QUOTA_MAYBE_EXCEEDED="The current transaction may have failed because of exceeding quota limitations.",yn.EXECUTE_TRANSACTION_ON_CLOSED_DB="Can't start a transaction on a closed database",yn.INCOMPATIBLE_DB_VERSION="The binary is incompatible with the database version",yn),An={},Bn=(An.AUTH_INVALID="ERROR",An.EXECUTE_TRANSACTION_ON_CLOSED_DB="WARNING",An.EXPLICIT_ABORT="IGNORED",An.IDB_NOT_SUPPORTED="ERROR",An.MISSING_INDEX=
"WARNING",An.MISSING_OBJECT_STORES="ERROR",An.DB_DELETED_BY_MISSING_OBJECT_STORES="WARNING",An.DB_REOPENED_BY_MISSING_OBJECT_STORES="WARNING",An.QUOTA_EXCEEDED="WARNING",An.QUOTA_MAYBE_EXCEEDED="WARNING",An.UNKNOWN_ABORT="WARNING",An.INCOMPATIBLE_DB_VERSION="WARNING",An),Cn={},Dn=(Cn.AUTH_INVALID=!1,Cn.EXECUTE_TRANSACTION_ON_CLOSED_DB=!1,Cn.EXPLICIT_ABORT=!1,Cn.IDB_NOT_SUPPORTED=!1,Cn.MISSING_INDEX=!1,Cn.MISSING_OBJECT_STORES=!1,Cn.DB_DELETED_BY_MISSING_OBJECT_STORES=!1,Cn.DB_REOPENED_BY_MISSING_OBJECT_STORES=
!1,Cn.QUOTA_EXCEEDED=!1,Cn.QUOTA_MAYBE_EXCEEDED=!0,Cn.UNKNOWN_ABORT=!0,Cn.INCOMPATIBLE_DB_VERSION=!1,Cn);function En(a,b,c,d,e){b=void 0===b?{}:b;c=void 0===c?zn[a]:c;d=void 0===d?Bn[a]:d;e=void 0===e?Dn[a]:e;V.call(this,c,Object.assign({},{name:"YtIdbKnownError",isSw:void 0===self.document,isIframe:self!==self.top,type:a},b));this.type=a;this.message=c;this.level=d;this.h=e;Object.setPrototypeOf(this,En.prototype)}
x(En,V);function Fn(a,b){En.call(this,"MISSING_OBJECT_STORES",{expectedObjectStores:b,foundObjectStores:a},zn.MISSING_OBJECT_STORES);Object.setPrototypeOf(this,Fn.prototype)}
x(Fn,En);function Gn(a,b){var c=Error.call(this);this.message=c.message;"stack"in c&&(this.stack=c.stack);this.index=a;this.objectStore=b;Object.setPrototypeOf(this,Gn.prototype)}
x(Gn,Error);var Hn=["The database connection is closing","Can't start a transaction on a closed database","A mutation operation was attempted on a database that did not allow mutations"];
function In(a,b,c,d){b=vn(b);var e=a instanceof Error?a:Error("Unexpected error: "+a);if(e instanceof En)return e;a={objectStoreNames:c,dbName:b,dbVersion:d};if("QuotaExceededError"===e.name)return new En("QUOTA_EXCEEDED",a);if(Fe&&"UnknownError"===e.name)return new En("QUOTA_MAYBE_EXCEEDED",a);if(e instanceof Gn)return new En("MISSING_INDEX",Object.assign({},a,{objectStore:e.objectStore,index:e.index}));if("InvalidStateError"===e.name&&Hn.some(function(f){return e.message.includes(f)}))return new En("EXECUTE_TRANSACTION_ON_CLOSED_DB",
a);
if("AbortError"===e.name)return new En("UNKNOWN_ABORT",a,e.message);e.args=[Object.assign({},a,{name:"IdbError",md:e.name})];e.level="WARNING";return e}
function Jn(a,b,c){var d=kn();return new En("IDB_NOT_SUPPORTED",{context:{caller:a,publicName:b,version:c,hasSucceededOnce:null==d?void 0:d.hasSucceededOnce}})}
;function Kn(a){if(!a)throw Error();throw a;}
function Ln(a){return a}
function Mn(a){this.h=a}
function Nn(a){function b(e){if("PENDING"===d.state.status){d.state={status:"REJECTED",reason:e};e=w(d.i);for(var f=e.next();!f.done;f=e.next())f=f.value,f()}}
function c(e){if("PENDING"===d.state.status){d.state={status:"FULFILLED",value:e};e=w(d.h);for(var f=e.next();!f.done;f=e.next())f=f.value,f()}}
var d=this;this.state={status:"PENDING"};this.h=[];this.i=[];a=a.h;try{a(c,b)}catch(e){b(e)}}
Nn.all=function(a){return new Nn(new Mn(function(b,c){var d=[],e=a.length;0===e&&b(d);for(var f={qb:0};f.qb<a.length;f={qb:f.qb},++f.qb)Nn.resolve(a[f.qb]).then(function(g){return function(h){d[g.qb]=h;e--;0===e&&b(d)}}(f)).catch(function(g){c(g)})}))};
Nn.resolve=function(a){return new Nn(new Mn(function(b,c){a instanceof Nn?a.then(b,c):b(a)}))};
Nn.reject=function(a){return new Nn(new Mn(function(b,c){c(a)}))};
Nn.prototype.then=function(a,b){var c=this,d=null!=a?a:Ln,e=null!=b?b:Kn;return new Nn(new Mn(function(f,g){"PENDING"===c.state.status?(c.h.push(function(){On(c,c,d,f,g)}),c.i.push(function(){Pn(c,c,e,f,g)})):"FULFILLED"===c.state.status?On(c,c,d,f,g):"REJECTED"===c.state.status&&Pn(c,c,e,f,g)}))};
Nn.prototype.catch=function(a){return this.then(void 0,a)};
function On(a,b,c,d,e){try{if("FULFILLED"!==a.state.status)throw Error("calling handleResolve before the promise is fulfilled.");var f=c(a.state.value);f instanceof Nn?Qn(a,b,f,d,e):d(f)}catch(g){e(g)}}
function Pn(a,b,c,d,e){try{if("REJECTED"!==a.state.status)throw Error("calling handleReject before the promise is rejected.");var f=c(a.state.reason);f instanceof Nn?Qn(a,b,f,d,e):d(f)}catch(g){e(g)}}
function Qn(a,b,c,d,e){b===c?e(new TypeError("Circular promise chain detected.")):c.then(function(f){f instanceof Nn?Qn(a,b,f,d,e):d(f)},function(f){e(f)})}
;function Rn(a,b,c){function d(){c(a.error);f()}
function e(){b(a.result);f()}
function f(){try{a.removeEventListener("success",e),a.removeEventListener("error",d)}catch(g){}}
a.addEventListener("success",e);a.addEventListener("error",d)}
function Sn(a){return new Promise(function(b,c){Rn(a,b,c)})}
function Tn(a){return new Nn(new Mn(function(b,c){Rn(a,b,c)}))}
;function Un(a,b){return new Nn(new Mn(function(c,d){function e(){var f=a?b(a):null;f?f.then(function(g){a=g;e()},d):c()}
e()}))}
;var Vn=window,W=Vn.ytcsi&&Vn.ytcsi.now?Vn.ytcsi.now:Vn.performance&&Vn.performance.timing&&Vn.performance.now&&Vn.performance.timing.navigationStart?function(){return Vn.performance.timing.navigationStart+Vn.performance.now()}:function(){return(new Date).getTime()};function Wn(a,b){this.h=a;this.options=b;this.transactionCount=0;this.j=Math.round(W());this.i=!1}
m=Wn.prototype;m.add=function(a,b,c){return Xn(this,[a],{mode:"readwrite",la:!0},function(d){return d.objectStore(a).add(b,c)})};
m.clear=function(a){return Xn(this,[a],{mode:"readwrite",la:!0},function(b){return b.objectStore(a).clear()})};
m.close=function(){this.h.close();var a;(null==(a=this.options)?0:a.closed)&&this.options.closed()};
m.count=function(a,b){return Xn(this,[a],{mode:"readonly",la:!0},function(c){return c.objectStore(a).count(b)})};
function Yn(a,b,c){a=a.h.createObjectStore(b,c);return new Zn(a)}
m.delete=function(a,b){return Xn(this,[a],{mode:"readwrite",la:!0},function(c){return c.objectStore(a).delete(b)})};
m.get=function(a,b){return Xn(this,[a],{mode:"readonly",la:!0},function(c){return c.objectStore(a).get(b)})};
function $n(a,b,c){return Xn(a,[b],{mode:"readwrite",la:!0},function(d){d=d.objectStore(b);return Tn(d.h.put(c,void 0))})}
m.objectStoreNames=function(){return Array.from(this.h.objectStoreNames)};
function Xn(a,b,c,d){var e,f,g,h,k,l,n,p,r,t,y,v;return A(function(z){switch(z.h){case 1:var G={mode:"readonly",la:!1,tag:"IDB_TRANSACTION_TAG_UNKNOWN"};"string"===typeof c?G.mode=c:Object.assign(G,c);e=G;a.transactionCount++;f=e.la?3:1;g=0;case 2:if(h){z.B(4);break}g++;k=Math.round(W());Aa(z,5);l=a.h.transaction(b,e.mode);G=z.yield;var K=new ao(l);K=bo(K,d);return G.call(z,K,7);case 7:return n=z.i,p=Math.round(W()),co(a,k,p,g,void 0,b.join(),e),z.return(n);case 5:r=Ba(z);t=Math.round(W());y=In(r,
a.h.name,b.join(),a.h.version);if((v=y instanceof En&&!y.h)||g>=f)co(a,k,t,g,y,b.join(),e),h=y;z.B(2);break;case 4:return z.return(Promise.reject(h))}})}
function co(a,b,c,d,e,f,g){b=c-b;e?(e instanceof En&&("QUOTA_EXCEEDED"===e.type||"QUOTA_MAYBE_EXCEEDED"===e.type)&&tn("QUOTA_EXCEEDED",{dbName:vn(a.h.name),objectStoreNames:f,transactionCount:a.transactionCount,transactionMode:g.mode}),e instanceof En&&"UNKNOWN_ABORT"===e.type&&(c-=a.j,0>c&&c>=Math.pow(2,31)&&(c=0),tn("TRANSACTION_UNEXPECTEDLY_ABORTED",{objectStoreNames:f,transactionDuration:b,transactionCount:a.transactionCount,dbDuration:c}),a.i=!0),eo(a,!1,d,f,b,g.tag),sn(e)):eo(a,!0,d,f,b,g.tag)}
function eo(a,b,c,d,e,f){tn("TRANSACTION_ENDED",{objectStoreNames:d,connectionHasUnknownAbortedTransaction:a.i,duration:e,isSuccessful:b,tryCount:c,tag:void 0===f?"IDB_TRANSACTION_TAG_UNKNOWN":f})}
m.getName=function(){return this.h.name};
function Zn(a){this.h=a}
m=Zn.prototype;m.add=function(a,b){return Tn(this.h.add(a,b))};
m.autoIncrement=function(){return this.h.autoIncrement};
m.clear=function(){return Tn(this.h.clear()).then(function(){})};
function fo(a,b,c){a.h.createIndex(b,c,{unique:!1})}
m.count=function(a){return Tn(this.h.count(a))};
function go(a,b){return ho(a,{query:b},function(c){return c.delete().then(function(){return io(c)})}).then(function(){})}
m.delete=function(a){return a instanceof IDBKeyRange?go(this,a):Tn(this.h.delete(a))};
m.get=function(a){return Tn(this.h.get(a))};
m.index=function(a){try{return new jo(this.h.index(a))}catch(b){if(b instanceof Error&&"NotFoundError"===b.name)throw new Gn(a,this.h.name);throw b;}};
m.getName=function(){return this.h.name};
m.keyPath=function(){return this.h.keyPath};
function ho(a,b,c){a=a.h.openCursor(b.query,b.direction);return ko(a).then(function(d){return Un(d,c)})}
function ao(a){var b=this;this.h=a;this.i=new Map;this.aborted=!1;this.done=new Promise(function(c,d){b.h.addEventListener("complete",function(){c()});
b.h.addEventListener("error",function(e){e.currentTarget===e.target&&d(b.h.error)});
b.h.addEventListener("abort",function(){var e=b.h.error;if(e)d(e);else if(!b.aborted){e=En;for(var f=b.h.objectStoreNames,g=[],h=0;h<f.length;h++){var k=f.item(h);if(null===k)throw Error("Invariant: item in DOMStringList is null");g.push(k)}e=new e("UNKNOWN_ABORT",{objectStoreNames:g.join(),dbName:b.h.db.name,mode:b.h.mode});d(e)}})})}
function bo(a,b){var c=new Promise(function(d,e){try{b(a).then(function(f){d(f)}).catch(e)}catch(f){e(f),a.abort()}});
return Promise.all([c,a.done]).then(function(d){return w(d).next().value})}
ao.prototype.abort=function(){this.h.abort();this.aborted=!0;throw new En("EXPLICIT_ABORT");};
ao.prototype.objectStore=function(a){a=this.h.objectStore(a);var b=this.i.get(a);b||(b=new Zn(a),this.i.set(a,b));return b};
function jo(a){this.h=a}
m=jo.prototype;m.count=function(a){return Tn(this.h.count(a))};
m.delete=function(a){return lo(this,{query:a},function(b){return b.delete().then(function(){return io(b)})})};
m.get=function(a){return Tn(this.h.get(a))};
m.keyPath=function(){return this.h.keyPath};
m.unique=function(){return this.h.unique};
function lo(a,b,c){a=a.h.openCursor(void 0===b.query?null:b.query,void 0===b.direction?"next":b.direction);return ko(a).then(function(d){return Un(d,c)})}
function mo(a,b){this.request=a;this.cursor=b}
function ko(a){return Tn(a).then(function(b){return b?new mo(a,b):null})}
function io(a){a.cursor.continue(void 0);return ko(a.request)}
mo.prototype.delete=function(){return Tn(this.cursor.delete()).then(function(){})};
mo.prototype.getValue=function(){return this.cursor.value};
mo.prototype.update=function(a){return Tn(this.cursor.update(a))};function no(a,b,c){return new Promise(function(d,e){function f(){r||(r=new Wn(g.result,{closed:p}));return r}
var g=void 0!==b?self.indexedDB.open(a,b):self.indexedDB.open(a);var h=c.Md,k=c.blocking,l=c.cf,n=c.upgrade,p=c.closed,r;g.addEventListener("upgradeneeded",function(t){try{if(null===t.newVersion)throw Error("Invariant: newVersion on IDbVersionChangeEvent is null");if(null===g.transaction)throw Error("Invariant: transaction on IDbOpenDbRequest is null");t.dataLoss&&"none"!==t.dataLoss&&tn("IDB_DATA_CORRUPTED",{reason:t.dataLossMessage||"unknown reason",dbName:vn(a)});var y=f(),v=new ao(g.transaction);
n&&n(y,function(z){return t.oldVersion<z&&t.newVersion>=z},v);
v.done.catch(function(z){e(z)})}catch(z){e(z)}});
g.addEventListener("success",function(){var t=g.result;k&&t.addEventListener("versionchange",function(){k(f())});
t.addEventListener("close",function(){tn("IDB_UNEXPECTEDLY_CLOSED",{dbName:vn(a),dbVersion:t.version});l&&l()});
d(f())});
g.addEventListener("error",function(){e(g.error)});
h&&g.addEventListener("blocked",function(){h()})})}
function oo(a,b,c){c=void 0===c?{}:c;return no(a,b,c)}
function po(a,b){b=void 0===b?{}:b;var c,d,e,f;return A(function(g){if(1==g.h)return Aa(g,2),c=self.indexedDB.deleteDatabase(a),d=b,(e=d.Md)&&c.addEventListener("blocked",function(){e()}),g.yield(Sn(c),4);
if(2!=g.h)g.h=0,g.l=0;else throw f=Ba(g),In(f,a,"",-1);})}
;function qo(a,b){this.name=a;this.options=b;this.j=!0;this.A=this.l=0}
qo.prototype.i=function(a,b,c){c=void 0===c?{}:c;return oo(a,b,c)};
qo.prototype.delete=function(a){a=void 0===a?{}:a;return po(this.name,a)};
function ro(a,b){return new En("INCOMPATIBLE_DB_VERSION",{dbName:a.name,oldVersion:a.options.version,newVersion:b})}
function so(a,b){if(!b)throw Jn("openWithToken",vn(a.name));return a.open()}
qo.prototype.open=function(){function a(){var f,g,h,k,l,n,p,r,t,y;return A(function(v){switch(v.h){case 1:return g=null!=(f=Error().stack)?f:"",Aa(v,2),v.yield(c.i(c.name,c.options.version,e),4);case 4:for(var z=h=v.i,G=c.options,K=[],N=w(Object.keys(G.wb)),S=N.next();!S.done;S=N.next()){S=S.value;var da=G.wb[S],sa=void 0===da.Ge?Number.MAX_VALUE:da.Ge;!(z.h.version>=da.Eb)||z.h.version>=sa||z.h.objectStoreNames.contains(S)||K.push(S)}k=K;if(0===k.length){v.B(5);break}l=Object.keys(c.options.wb);
n=h.objectStoreNames();if(c.A<yl("ytidb_reopen_db_retries",0))return c.A++,h.close(),sn(new En("DB_REOPENED_BY_MISSING_OBJECT_STORES",{dbName:c.name,expectedObjectStores:l,foundObjectStores:n})),v.return(a());if(!(c.l<yl("ytidb_remake_db_retries",1))){v.B(6);break}c.l++;return v.yield(c.delete(),7);case 7:return sn(new En("DB_DELETED_BY_MISSING_OBJECT_STORES",{dbName:c.name,expectedObjectStores:l,foundObjectStores:n})),v.return(a());case 6:throw new Fn(n,l);case 5:return v.return(h);case 2:p=Ba(v);
if(p instanceof DOMException?"VersionError"!==p.name:"DOMError"in self&&p instanceof DOMError?"VersionError"!==p.name:!(p instanceof Object&&"message"in p)||"An attempt was made to open a database using a lower version than the existing version."!==p.message){v.B(8);break}return v.yield(c.i(c.name,void 0,Object.assign({},e,{upgrade:void 0})),9);case 9:r=v.i;t=r.h.version;if(void 0!==c.options.version&&t>c.options.version+1)throw r.close(),c.j=!1,ro(c,t);return v.return(r);case 8:throw b(),p instanceof
Error&&!U("ytidb_async_stack_killswitch")&&(p.stack=p.stack+"\n"+g.substring(g.indexOf("\n")+1)),In(p,c.name,"",null!=(y=c.options.version)?y:-1);}})}
function b(){c.h===d&&(c.h=void 0)}
var c=this;if(!this.j)throw ro(this);if(this.h)return this.h;var d,e={blocking:function(f){f.close()},
closed:b,cf:b,upgrade:this.options.upgrade};return this.h=d=a()};var to=new qo("YtIdbMeta",{wb:{databases:{Eb:1}},upgrade:function(a,b){b(1)&&Yn(a,"databases",{keyPath:"actualName"})}});
function uo(a,b){var c;return A(function(d){if(1==d.h)return d.yield(so(to,b),2);c=d.i;return d.return(Xn(c,["databases"],{la:!0,mode:"readwrite"},function(e){var f=e.objectStore("databases");return f.get(a.actualName).then(function(g){if(g?a.actualName!==g.actualName||a.publicName!==g.publicName||a.userIdentifier!==g.userIdentifier:1)return Tn(f.h.put(a,void 0)).then(function(){})})}))})}
function vo(a,b){var c;return A(function(d){if(1==d.h)return a?d.yield(so(to,b),2):d.return();c=d.i;return d.return(c.delete("databases",a))})}
function wo(a,b){var c,d;return A(function(e){return 1==e.h?(c=[],e.yield(so(to,b),2)):3!=e.h?(d=e.i,e.yield(Xn(d,["databases"],{la:!0,mode:"readonly"},function(f){c.length=0;return ho(f.objectStore("databases"),{},function(g){a(g.getValue())&&c.push(g.getValue());return io(g)})}),3)):e.return(c)})}
function xo(a){return wo(function(b){return"LogsDatabaseV2"===b.publicName&&void 0!==b.userIdentifier},a)}
function yo(a,b,c){return wo(function(d){return c?void 0!==d.userIdentifier&&!a.includes(d.userIdentifier)&&c.includes(d.publicName):void 0!==d.userIdentifier&&!a.includes(d.userIdentifier)},b)}
function zo(a){var b,c;return A(function(d){if(1==d.h)return b=Am("YtIdbMeta hasAnyMeta other"),d.yield(wo(function(e){return void 0!==e.userIdentifier&&e.userIdentifier!==b},a),2);
c=d.i;return d.return(0<c.length)})}
;var Ao,Bo=new function(){}(new function(){});
function Co(){var a,b,c,d;return A(function(e){switch(e.h){case 1:a=kn();if(null==(b=a)?0:b.hasSucceededOnce)return e.return(!0);var f;if(f=wn)f=/WebKit\/([0-9]+)/.exec(Jc()),f=!!(f&&600<=parseInt(f[1],10));f&&(f=/WebKit\/([0-9]+)/.exec(Jc()),f=!(f&&602<=parseInt(f[1],10)));if(f||Wc)return e.return(!1);try{if(c=self,!(c.indexedDB&&c.IDBIndex&&c.IDBKeyRange&&c.IDBObjectStore))return e.return(!1)}catch(g){return e.return(!1)}if(!("IDBTransaction"in self&&"objectStoreNames"in IDBTransaction.prototype))return e.return(!1);
Aa(e,2);d={actualName:"yt-idb-test-do-not-use",publicName:"yt-idb-test-do-not-use",userIdentifier:void 0};return e.yield(uo(d,Bo),4);case 4:return e.yield(vo("yt-idb-test-do-not-use",Bo),5);case 5:return e.return(!0);case 2:return Ba(e),e.return(!1)}})}
function Do(){if(void 0!==Ao)return Ao;nn=!0;return Ao=Co().then(function(a){nn=!1;var b;if(null!=(b=jn())&&b.h){var c;b={hasSucceededOnce:(null==(c=kn())?void 0:c.hasSucceededOnce)||a};var d;null==(d=jn())||d.set("LAST_RESULT_ENTRY_KEY",b,2592E3,!0)}return a})}
function Eo(){return E("ytglobal.idbToken_")||void 0}
function Fo(){var a=Eo();return a?Promise.resolve(a):Do().then(function(b){(b=b?Bo:void 0)&&D("ytglobal.idbToken_",b);return b})}
;var Go=0;function Ho(a,b){Go||(Go=Ni.pa(function(){var c,d,e,f,g;return A(function(h){switch(h.h){case 1:return h.yield(Fo(),2);case 2:c=h.i;if(!c)return h.return();d=!0;Aa(h,3);return h.yield(yo(a,c,b),5);case 5:e=h.i;if(!e.length){d=!1;h.B(6);break}f=e[0];return h.yield(po(f.actualName),7);case 7:return h.yield(vo(f.actualName,c),6);case 6:h.h=4;h.l=0;break;case 3:g=Ba(h),sn(g),d=!1;case 4:Ni.qa(Go),Go=0,d&&Ho(a,b),h.h=0}})}))}
function Io(){var a;return A(function(b){return 1==b.h?b.yield(Fo(),2):(a=b.i)?b.return(zo(a)):b.return(!1)})}
new wi;function Jo(a){if(!zm())throw a=new En("AUTH_INVALID",{dbName:a}),sn(a),a;var b=Am();return{actualName:a+":"+b,publicName:a,userIdentifier:b}}
function Ko(a,b,c,d){var e,f,g,h,k,l;return A(function(n){switch(n.h){case 1:return f=null!=(e=Error().stack)?e:"",n.yield(Fo(),2);case 2:g=n.i;if(!g)throw h=Jn("openDbImpl",a,b),U("ytidb_async_stack_killswitch")||(h.stack=h.stack+"\n"+f.substring(f.indexOf("\n")+1)),sn(h),h;un(a);k=c?{actualName:a,publicName:a,userIdentifier:void 0}:Jo(a);Aa(n,3);return n.yield(uo(k,g),5);case 5:return n.yield(oo(k.actualName,b,d),6);case 6:return n.return(n.i);case 3:return l=Ba(n),Aa(n,7),n.yield(vo(k.actualName,
g),9);case 9:n.h=8;n.l=0;break;case 7:Ba(n);case 8:throw l;}})}
function Lo(a,b,c){c=void 0===c?{}:c;return Ko(a,b,!1,c)}
function Mo(a,b,c){c=void 0===c?{}:c;return Ko(a,b,!0,c)}
function No(a,b){b=void 0===b?{}:b;var c,d;return A(function(e){if(1==e.h)return e.yield(Fo(),2);if(3!=e.h){c=e.i;if(!c)return e.return();un(a);d=Jo(a);return e.yield(po(d.actualName,b),3)}return e.yield(vo(d.actualName,c),0)})}
function Oo(a,b,c){a=a.map(function(d){return A(function(e){return 1==e.h?e.yield(po(d.actualName,b),2):e.yield(vo(d.actualName,c),0)})});
return Promise.all(a).then(function(){})}
function Po(){var a=void 0===a?{}:a;var b,c;return A(function(d){if(1==d.h)return d.yield(Fo(),2);if(3!=d.h){b=d.i;if(!b)return d.return();un("LogsDatabaseV2");return d.yield(xo(b),3)}c=d.i;return d.yield(Oo(c,a,b),0)})}
function Qo(a,b){b=void 0===b?{}:b;var c;return A(function(d){if(1==d.h)return d.yield(Fo(),2);if(3!=d.h){c=d.i;if(!c)return d.return();un(a);return d.yield(po(a,b),3)}return d.yield(vo(a,c),0)})}
;function Ro(a,b){qo.call(this,a,b);this.options=b;un(a)}
x(Ro,qo);function So(a,b){var c;return function(){c||(c=new Ro(a,b));return c}}
Ro.prototype.i=function(a,b,c){c=void 0===c?{}:c;return(this.options.shared?Mo:Lo)(a,b,Object.assign({},c))};
Ro.prototype.delete=function(a){a=void 0===a?{}:a;return(this.options.shared?Qo:No)(this.name,a)};
function To(a,b){return So(a,b)}
;var Uo={},Vo=To("ytGcfConfig",{wb:(Uo.coldConfigStore={Eb:1},Uo.hotConfigStore={Eb:1},Uo),shared:!1,upgrade:function(a,b){b(1)&&(fo(Yn(a,"hotConfigStore",{keyPath:"key",autoIncrement:!0}),"hotTimestampIndex","timestamp"),fo(Yn(a,"coldConfigStore",{keyPath:"key",autoIncrement:!0}),"coldTimestampIndex","timestamp"))},
version:1});function Wo(a){return so(Vo(),a)}
function Xo(a,b,c){var d,e,f;return A(function(g){switch(g.h){case 1:return d={config:a,hashData:b,timestamp:W()},g.yield(Wo(c),2);case 2:return e=g.i,g.yield(e.clear("hotConfigStore"),3);case 3:return g.yield($n(e,"hotConfigStore",d),4);case 4:return f=g.i,g.return(f)}})}
function Yo(a,b,c,d){var e,f,g;return A(function(h){switch(h.h){case 1:return e={config:a,hashData:b,configData:c,timestamp:W()},h.yield(Wo(d),2);case 2:return f=h.i,h.yield(f.clear("coldConfigStore"),3);case 3:return h.yield($n(f,"coldConfigStore",e),4);case 4:return g=h.i,h.return(g)}})}
function Zo(a){var b,c;return A(function(d){return 1==d.h?d.yield(Wo(a),2):3!=d.h?(b=d.i,c=void 0,d.yield(Xn(b,["coldConfigStore"],{mode:"readwrite",la:!0},function(e){return lo(e.objectStore("coldConfigStore").index("coldTimestampIndex"),{direction:"prev"},function(f){c=f.getValue()})}),3)):d.return(c)})}
function $o(a){var b,c;return A(function(d){return 1==d.h?d.yield(Wo(a),2):3!=d.h?(b=d.i,c=void 0,d.yield(Xn(b,["hotConfigStore"],{mode:"readwrite",la:!0},function(e){return lo(e.objectStore("hotConfigStore").index("hotTimestampIndex"),{direction:"prev"},function(f){c=f.getValue()})}),3)):d.return(c)})}
;function ap(){F.call(this);this.i=[];this.h=[];var a=E("yt.gcf.config.hotUpdateCallbacks");a?(this.i=[].concat(la(a)),this.h=a):(this.h=[],D("yt.gcf.config.hotUpdateCallbacks",this.h))}
x(ap,F);ap.prototype.U=function(){for(var a=w(this.i),b=a.next();!b.done;b=a.next()){var c=this.h;b=c.indexOf(b.value);0<=b&&c.splice(b,1)}this.i.length=0;F.prototype.U.call(this)};function bp(){this.h=0;this.i=new ap}
function cp(){var a;return null!=(a=E("yt.gcf.config.hotConfigGroup"))?a:T("RAW_HOT_CONFIG_GROUP")}
function dp(a,b,c){var d,e,f;return A(function(g){switch(g.h){case 1:if(!U("start_client_gcf")){g.B(0);break}c&&(a.j=c,D("yt.gcf.config.hotConfigGroup",a.j||null));a.l(b);d=Eo();if(!d){g.B(3);break}if(c){g.B(4);break}return g.yield($o(d),5);case 5:e=g.i,c=null==(f=e)?void 0:f.config;case 4:return g.yield(Xo(c,b,d),3);case 3:if(c)for(var h=c,k=w(a.i.h),l=k.next();!l.done;l=k.next())l=l.value,l(h);g.h=0}})}
function ep(a,b,c){var d,e,f,g;return A(function(h){if(1==h.h){if(!U("start_client_gcf"))return h.B(0);a.coldHashData=b;D("yt.gcf.config.coldHashData",a.coldHashData||null);return(d=Eo())?c?h.B(4):h.yield(Zo(d),5):h.B(0)}4!=h.h&&(e=h.i,c=null==(f=e)?void 0:f.config);if(!c)return h.B(0);g=c.configData;return h.yield(Yo(c,b,g,d),0)})}
function fp(){if(!bp.h){var a=new bp;bp.h=a}a=bp.h;var b=W()-a.h;if(!(0!==a.h&&b<yl("send_config_hash_timer"))){b=E("yt.gcf.config.coldConfigData");var c=E("yt.gcf.config.hotHashData"),d=E("yt.gcf.config.coldHashData");b&&c&&d&&(a.h=W());return{coldConfigData:b,hotHashData:c,coldHashData:d}}}
bp.prototype.l=function(a){this.hotHashData=a;D("yt.gcf.config.hotHashData",this.hotHashData||null)};function gp(){return"INNERTUBE_API_KEY"in Zk&&"INNERTUBE_API_VERSION"in Zk}
function hp(){return{innertubeApiKey:T("INNERTUBE_API_KEY"),innertubeApiVersion:T("INNERTUBE_API_VERSION"),he:T("INNERTUBE_CONTEXT_CLIENT_CONFIG_INFO"),gd:T("INNERTUBE_CONTEXT_CLIENT_NAME","WEB"),Hg:T("INNERTUBE_CONTEXT_CLIENT_NAME",1),innertubeContextClientVersion:T("INNERTUBE_CONTEXT_CLIENT_VERSION"),ke:T("INNERTUBE_CONTEXT_HL"),je:T("INNERTUBE_CONTEXT_GL"),le:T("INNERTUBE_HOST_OVERRIDE")||"",ne:!!T("INNERTUBE_USE_THIRD_PARTY_AUTH",!1),me:!!T("INNERTUBE_OMIT_API_KEY_WHEN_AUTH_HEADER_IS_PRESENT",
!1),appInstallData:T("SERIALIZED_CLIENT_CONFIG_DATA")}}
function ip(a){var b={client:{hl:a.ke,gl:a.je,clientName:a.gd,clientVersion:a.innertubeContextClientVersion,configInfo:a.he}};navigator.userAgent&&(b.client.userAgent=String(navigator.userAgent));var c=C.devicePixelRatio;c&&1!=c&&(b.client.screenDensityFloat=String(c));c=T("EXPERIMENTS_TOKEN","");""!==c&&(b.client.experimentsToken=c);c=zl();0<c.length&&(b.request={internalExperimentFlags:c});c=a.gd;if(("WEB"===c||"MWEB"===c||1===c||2===c)&&b){var d;b.client.mainAppWebInfo=null!=(d=b.client.mainAppWebInfo)?
d:{};b.client.mainAppWebInfo.webDisplayMode=cm()}(d=E("yt.embedded_player.embed_url"))&&b&&(b.thirdParty={embedUrl:d});var e;if(U("web_log_memory_total_kbytes")&&(null==(e=C.navigator)?0:e.deviceMemory)){var f;e=null==(f=C.navigator)?void 0:f.deviceMemory;b&&(b.client.memoryTotalKbytes=""+1E6*e)}a.appInstallData&&b&&(b.client.configInfo=b.client.configInfo||{},b.client.configInfo.appInstallData=a.appInstallData);(a=xm())&&b&&(b.client.connectionType=a);U("web_log_effective_connection_type")&&(a=ym())&&
b&&(b.client.effectiveConnectionType=a);U("start_client_gcf")&&(e=fp())&&(a=e.coldConfigData,f=e.coldHashData,e=e.hotHashData,b&&(b.client.configInfo=b.client.configInfo||{},a&&(b.client.configInfo.coldConfigData=a),f&&(b.client.configInfo.coldHashData=f),e&&(b.client.configInfo.hotHashData=e)));T("DELEGATED_SESSION_ID")&&!U("pageid_as_header_web")&&(b.user={onBehalfOfUser:T("DELEGATED_SESSION_ID")});!U("fill_delegate_context_in_gel_killswitch")&&(a=T("INNERTUBE_CONTEXT_SERIALIZED_DELEGATION_CONTEXT"))&&
(b.user=Object.assign({},b.user,{serializedDelegationContext:a}));a=Object;f=a.assign;e=b.client;d={};c=w(Object.entries(ml(T("DEVICE",""))));for(var g=c.next();!g.done;g=c.next()){var h=w(g.value);g=h.next().value;h=h.next().value;"cbrand"===g?d.deviceMake=h:"cmodel"===g?d.deviceModel=h:"cbr"===g?d.browserName=h:"cbrver"===g?d.browserVersion=h:"cos"===g?d.osName=h:"cosver"===g?d.osVersion=h:"cplatform"===g&&(d.platform=h)}b.client=f.call(a,e,d);return b}
function jp(a,b,c){c=void 0===c?{}:c;var d={};T("EOM_VISITOR_DATA")?d={"X-Goog-EOM-Visitor-Id":T("EOM_VISITOR_DATA")}:d={"X-Goog-Visitor-Id":c.visitorData||T("VISITOR_DATA","")};if(b&&b.includes("www.youtube-nocookie.com"))return d;b=c.authorization||T("AUTHORIZATION");b||(a?b="Bearer "+E("gapi.auth.getToken")().zg:(a=fm(em()),U("pageid_as_header_web")||delete a["X-Goog-PageId"],d=Object.assign({},d,a)));b&&(d.Authorization=b);return d}
;var kp="undefined"!==typeof TextEncoder?new TextEncoder:null,lp=kp?function(a){return kp.encode(a)}:function(a){for(var b=[],c=0,d=0;d<a.length;d++){var e=a.charCodeAt(d);
128>e?b[c++]=e:(2048>e?b[c++]=e>>6|192:(55296==(e&64512)&&d+1<a.length&&56320==(a.charCodeAt(d+1)&64512)?(e=65536+((e&1023)<<10)+(a.charCodeAt(++d)&1023),b[c++]=e>>18|240,b[c++]=e>>12&63|128):b[c++]=e>>12|224,b[c++]=e>>6&63|128),b[c++]=e&63|128)}a=new Uint8Array(b.length);for(c=0;c<a.length;c++)a[c]=b[c];return a};function mp(a,b){this.version=a;this.args=b}
mp.prototype.serialize=function(){return{version:this.version,args:this.args}};function np(a,b){this.topic=a;this.h=b}
np.prototype.toString=function(){return this.topic};var op=E("ytPubsub2Pubsub2Instance")||new M;M.prototype.subscribe=M.prototype.subscribe;M.prototype.unsubscribeByKey=M.prototype.Ab;M.prototype.publish=M.prototype.Xa;M.prototype.clear=M.prototype.clear;D("ytPubsub2Pubsub2Instance",op);var pp=E("ytPubsub2Pubsub2SubscribedKeys")||{};D("ytPubsub2Pubsub2SubscribedKeys",pp);var qp=E("ytPubsub2Pubsub2TopicToKeys")||{};D("ytPubsub2Pubsub2TopicToKeys",qp);var rp=E("ytPubsub2Pubsub2IsAsync")||{};D("ytPubsub2Pubsub2IsAsync",rp);
D("ytPubsub2Pubsub2SkipSubKey",null);function sp(a,b){var c=tp();c&&c.publish.call(c,a.toString(),a,b)}
function up(a){var b=vp,c=tp();if(!c)return 0;var d=c.subscribe(b.toString(),function(e,f){var g=E("ytPubsub2Pubsub2SkipSubKey");g&&g==d||(g=function(){if(pp[d])try{if(f&&b instanceof np&&b!=e)try{var h=b.h,k=f;if(!k.args||!k.version)throw Error("yt.pubsub2.Data.deserialize(): serializedData is incomplete.");try{if(!h.Cd){var l=new h;h.Cd=l.version}var n=h.Cd}catch(z){}if(!n||k.version!=n)throw Error("yt.pubsub2.Data.deserialize(): serializedData version is incompatible.");try{n=Reflect;var p=n.construct;
var r=k.args,t=r.length;if(0<t){var y=Array(t);for(k=0;k<t;k++)y[k]=r[k];var v=y}else v=[];f=p.call(n,h,v)}catch(z){throw z.message="yt.pubsub2.Data.deserialize(): "+z.message,z;}}catch(z){throw z.message="yt.pubsub2.pubsub2 cross-binary conversion error for "+b.toString()+": "+z.message,z;}a.call(window,f)}catch(z){el(z)}},rp[b.toString()]?E("yt.scheduler.instance")?Ni.pa(g):vl(g,0):g())});
pp[d]=!0;qp[b.toString()]||(qp[b.toString()]=[]);qp[b.toString()].push(d);return d}
function wp(){var a=xp,b=up(function(c){a.apply(void 0,arguments);yp(b)});
return b}
function yp(a){var b=tp();b&&("number"===typeof a&&(a=[a]),Db(a,function(c){b.unsubscribeByKey(c);delete pp[c]}))}
function tp(){return E("ytPubsub2Pubsub2Instance")}
;function zp(a,b,c){c=void 0===c?{sampleRate:.1}:c;Math.random()<Math.min(.02,c.sampleRate/100)&&sp("meta_logging_csi_event",{timerName:a,Xg:b})}
;var Ap=void 0,Bp=void 0;function Cp(){Bp||(Bp=Bk(T("WORKER_SERIALIZATION_URL")));return Bp||void 0}
function Dp(){var a=Cp();Ap||void 0===a||(Ap=new Worker(kb(a),void 0));return Ap}
;var Ep=yl("max_body_size_to_compress",5E5),Fp=yl("min_body_size_to_compress",500),Gp=!0,Hp=0,Ip=0,Jp=yl("compression_performance_threshold_lr",250),Kp=yl("slow_compressions_before_abandon_count",4),Lp=!1,Mp=new Map,Np=1,Op=!0;function Pp(){if("function"===typeof Worker&&Cp()&&!Lp){var a=function(c){c=c.data;if("gzippedGelBatch"===c.op){var d=Mp.get(c.key);d&&(Qp(c.gzippedBatch,d.latencyPayload,d.url,d.options,d.sendFn),Mp.delete(c.key))}},b=Dp();
b&&(b.addEventListener("message",a),b.onerror=function(){Mp.clear()},Lp=!0)}}
function Rp(a,b,c,d,e){e=void 0===e?!1:e;var f={startTime:W(),ticks:{},infos:{}};if(Gp)try{var g=Sp(b);if(null!=g&&(g>Ep||g<Fp))d(a,c);else{if(U("gzip_gel_with_worker")&&(U("initial_gzip_use_main_thread")&&!Op||!U("initial_gzip_use_main_thread"))){Lp||Pp();var h=Dp();if(h&&!e){Mp.set(Np,{latencyPayload:f,url:a,options:c,sendFn:d});h.postMessage({op:"gelBatchToGzip",serializedBatch:b,key:Np});Np++;return}}var k=Ak(lp(b));Qp(k,f,a,c,d)}}catch(l){fl(l),d(a,c)}else d(a,c)}
function Qp(a,b,c,d,e){Op=!1;var f=W();b.ticks.gelc=f;Ip++;U("disable_compression_due_to_performance_degredation")&&f-b.startTime>=Jp&&(Hp++,U("abandon_compression_after_N_slow_zips")?Ip===yl("compression_disable_point")&&Hp>Kp&&(Gp=!1):Gp=!1);Tp(b);d.headers||(d.headers={});d.headers["Content-Encoding"]="gzip";d.postBody=a;d.postParams=void 0;e(c,d)}
function Up(a){var b=void 0===b?!1:b;var c=void 0===c?!1:c;var d=W(),e={startTime:d,ticks:{},infos:{}},f=b?E("yt.logging.gzipForFetch",!1):!0;if(Gp&&f){if(!a.body)return a;try{var g=c?a.body:"string"===typeof a.body?a.body:JSON.stringify(a.body);f=g;if(!c&&"string"===typeof g){var h=Sp(g);if(null!=h&&(h>Ep||h<Fp))return a;c=b?{level:1}:void 0;f=Ak(lp(g),c);var k=W();e.ticks.gelc=k;if(b){Ip++;if((U("disable_compression_due_to_performance_degredation")||U("disable_compression_due_to_performance_degradation_lr"))&&
k-d>=Jp)if(Hp++,U("abandon_compression_after_N_slow_zips")||U("abandon_compression_after_N_slow_zips_lr")){b=Hp/Ip;var l=Kp/yl("compression_disable_point");0<Ip&&0===Ip%yl("compression_disable_point")&&b>=l&&(Gp=!1)}else Gp=!1;Tp(e)}}a.headers=Object.assign({},{"Content-Encoding":"gzip"},a.headers||{});a.body=f;return a}catch(n){return fl(n),a}}else return a}
function Sp(a){try{return(new Blob(a.split(""))).size}catch(b){return fl(b),null}}
function Tp(a){U("gel_compression_csi_killswitch")||!U("log_gel_compression_latency")&&!U("log_gel_compression_latency_lr")||zp("gel_compression",a,{sampleRate:.1})}
;function Vp(a){a=Object.assign({},a);delete a.Authorization;var b=mh();if(b){var c=new Ri;c.update(T("INNERTUBE_API_KEY"));c.update(b);a.hash=Ie(c.digest(),3)}return a}
;var Wp;function Xp(){Wp||(Wp=new hn("yt.innertube"));return Wp}
function Yp(a,b,c,d){if(d)return null;d=Xp().get("nextId",!0)||1;var e=Xp().get("requests",!0)||{};e[d]={method:a,request:b,authState:Vp(c),requestTime:Math.round(W())};Xp().set("nextId",d+1,86400,!0);Xp().set("requests",e,86400,!0);return d}
function Zp(a){var b=Xp().get("requests",!0)||{};delete b[a];Xp().set("requests",b,86400,!0)}
function $p(a){var b=Xp().get("requests",!0);if(b){for(var c in b){var d=b[c];if(!(6E4>Math.round(W())-d.requestTime)){var e=d.authState,f=Vp(jp(!1));Qb(e,f)&&(e=d.request,"requestTimeMs"in e&&(e.requestTimeMs=Math.round(W())),aq(a,d.method,e,{}));delete b[c]}}Xp().set("requests",b,86400,!0)}}
;function bq(a){this.Wb=this.h=!1;this.potentialEsfErrorCounter=this.i=0;this.handleError=function(){};
this.ob=function(){};
this.now=Date.now;this.Hb=!1;var b;this.zd=null!=(b=a.zd)?b:100;var c;this.td=null!=(c=a.td)?c:1;var d;this.qd=null!=(d=a.qd)?d:2592E6;var e;this.od=null!=(e=a.od)?e:12E4;var f;this.sd=null!=(f=a.sd)?f:5E3;var g;this.X=null!=(g=a.X)?g:void 0;this.cc=!!a.cc;var h;this.Zb=null!=(h=a.Zb)?h:.1;var k;this.lc=null!=(k=a.lc)?k:10;a.handleError&&(this.handleError=a.handleError);a.ob&&(this.ob=a.ob);a.Hb&&(this.Hb=a.Hb);a.Wb&&(this.Wb=a.Wb);this.Y=a.Y;this.Da=a.Da;this.ha=a.ha;this.fa=a.fa;this.sendFn=a.sendFn;
this.Jc=a.Jc;this.Gc=a.Gc;cq(this)&&(!this.Y||this.Y("networkless_logging"))&&dq(this)}
function dq(a){cq(a)&&!a.Hb&&(a.h=!0,a.cc&&Math.random()<=a.Zb&&a.ha.Od(a.X),eq(a),a.fa.va()&&a.Rb(),a.fa.listen(a.Jc,a.Rb.bind(a)),a.fa.listen(a.Gc,a.Uc.bind(a)))}
m=bq.prototype;m.writeThenSend=function(a,b){var c=this;b=void 0===b?{}:b;if(cq(this)&&this.h){var d={url:a,options:b,timestamp:this.now(),status:"NEW",sendCount:0};this.ha.set(d,this.X).then(function(e){d.id=e;c.fa.va()&&fq(c,d)}).catch(function(e){fq(c,d);
gq(c,e)})}else this.sendFn(a,b)};
m.sendThenWrite=function(a,b,c){var d=this;b=void 0===b?{}:b;if(cq(this)&&this.h){var e={url:a,options:b,timestamp:this.now(),status:"NEW",sendCount:0};this.Y&&this.Y("nwl_skip_retry")&&(e.skipRetry=c);if(this.fa.va()||this.Y&&this.Y("nwl_aggressive_send_then_write")&&!e.skipRetry){if(!e.skipRetry){var f=b.onError?b.onError:function(){};
b.onError=function(g,h){return A(function(k){if(1==k.h)return k.yield(d.ha.set(e,d.X).catch(function(l){gq(d,l)}),2);
f(g,h);k.h=0})}}this.sendFn(a,b,e.skipRetry)}else this.ha.set(e,this.X).catch(function(g){d.sendFn(a,b,e.skipRetry);
gq(d,g)})}else this.sendFn(a,b,this.Y&&this.Y("nwl_skip_retry")&&c)};
m.sendAndWrite=function(a,b){var c=this;b=void 0===b?{}:b;if(cq(this)&&this.h){var d={url:a,options:b,timestamp:this.now(),status:"NEW",sendCount:0},e=!1,f=b.onSuccess?b.onSuccess:function(){};
d.options.onSuccess=function(g,h){void 0!==d.id?c.ha.nb(d.id,c.X):e=!0;c.fa.fb&&c.Y&&c.Y("vss_network_hint")&&c.fa.fb(!0);f(g,h)};
this.sendFn(d.url,d.options,void 0,!0);this.ha.set(d,this.X).then(function(g){d.id=g;e&&c.ha.nb(d.id,c.X)}).catch(function(g){gq(c,g)})}else this.sendFn(a,b,void 0,!0)};
m.Rb=function(){var a=this;if(!cq(this))throw Error("IndexedDB is not supported: throttleSend");this.i||(this.i=this.Da.pa(function(){var b;return A(function(c){if(1==c.h)return c.yield(a.ha.cd("NEW",a.X),2);if(3!=c.h)return b=c.i,b?c.yield(fq(a,b),3):(a.Uc(),c.return());a.i&&(a.i=0,a.Rb());c.h=0})},this.zd))};
m.Uc=function(){this.Da.qa(this.i);this.i=0};
function fq(a,b){var c;return A(function(d){switch(d.h){case 1:if(!cq(a))throw Error("IndexedDB is not supported: immediateSend");if(void 0===b.id){d.B(2);break}return d.yield(a.ha.re(b.id,a.X),3);case 3:(c=d.i)||a.ob(Error("The request cannot be found in the database."));case 2:if(hq(a,b,a.qd)){d.B(4);break}a.ob(Error("Networkless Logging: Stored logs request expired age limit"));if(void 0===b.id){d.B(5);break}return d.yield(a.ha.nb(b.id,a.X),5);case 5:return d.return();case 4:b.skipRetry||(b=iq(a,
b));if(!b){d.B(0);break}if(!b.skipRetry||void 0===b.id){d.B(8);break}return d.yield(a.ha.nb(b.id,a.X),8);case 8:a.sendFn(b.url,b.options,!!b.skipRetry),d.h=0}})}
function iq(a,b){if(!cq(a))throw Error("IndexedDB is not supported: updateRequestHandlers");var c=b.options.onError?b.options.onError:function(){};
b.options.onError=function(e,f){var g,h,k,l;return A(function(n){switch(n.h){case 1:g=jq(f);(h=kq(f))&&a.Y&&a.Y("web_enable_error_204")&&a.handleError(Error("Request failed due to compression"),b.url,f);if(!(a.Y&&a.Y("nwl_consider_error_code")&&g||a.Y&&!a.Y("nwl_consider_error_code")&&a.potentialEsfErrorCounter<=a.lc)){n.B(2);break}if(!a.fa.oc){n.B(3);break}return n.yield(a.fa.oc(),3);case 3:if(a.fa.va()){n.B(2);break}c(e,f);if(!a.Y||!a.Y("nwl_consider_error_code")||void 0===(null==(k=b)?void 0:k.id)){n.B(6);
break}return n.yield(a.ha.Kc(b.id,a.X,!1),6);case 6:return n.return();case 2:if(a.Y&&a.Y("nwl_consider_error_code")&&!g&&a.potentialEsfErrorCounter>a.lc)return n.return();a.potentialEsfErrorCounter++;if(void 0===(null==(l=b)?void 0:l.id)){n.B(8);break}return b.sendCount<a.td?n.yield(a.ha.Kc(b.id,a.X,!0,h?!1:void 0),12):n.yield(a.ha.nb(b.id,a.X),8);case 12:a.Da.pa(function(){a.fa.va()&&a.Rb()},a.sd);
case 8:c(e,f),n.h=0}})};
var d=b.options.onSuccess?b.options.onSuccess:function(){};
b.options.onSuccess=function(e,f){var g;return A(function(h){if(1==h.h)return void 0===(null==(g=b)?void 0:g.id)?h.B(2):h.yield(a.ha.nb(b.id,a.X),2);a.fa.fb&&a.Y&&a.Y("vss_network_hint")&&a.fa.fb(!0);d(e,f);h.h=0})};
return b}
function hq(a,b,c){b=b.timestamp;return a.now()-b>=c?!1:!0}
function eq(a){if(!cq(a))throw Error("IndexedDB is not supported: retryQueuedRequests");a.ha.cd("QUEUED",a.X).then(function(b){b&&!hq(a,b,a.od)?a.Da.pa(function(){return A(function(c){if(1==c.h)return void 0===b.id?c.B(2):c.yield(a.ha.Kc(b.id,a.X),2);eq(a);c.h=0})}):a.fa.va()&&a.Rb()})}
function gq(a,b){a.Fd&&!a.fa.va()?a.Fd(b):a.handleError(b)}
function cq(a){return!!a.X||a.Wb}
function jq(a){var b;return(a=null==a?void 0:null==(b=a.error)?void 0:b.code)&&400<=a&&599>=a?!1:!0}
function kq(a){var b;a=null==a?void 0:null==(b=a.error)?void 0:b.code;return!(400!==a&&415!==a)}
;var lq;
function mq(){if(lq)return lq();var a={};lq=To("LogsDatabaseV2",{wb:(a.LogsRequestsStore={Eb:2},a),shared:!1,upgrade:function(b,c,d){c(2)&&Yn(b,"LogsRequestsStore",{keyPath:"id",autoIncrement:!0});c(3);c(5)&&(d=d.objectStore("LogsRequestsStore"),d.h.indexNames.contains("newRequest")&&d.h.deleteIndex("newRequest"),fo(d,"newRequestV2",["status","interface","timestamp"]));c(7)&&b.h.objectStoreNames.contains("sapisid")&&b.h.deleteObjectStore("sapisid");c(9)&&b.h.objectStoreNames.contains("SWHealthLog")&&b.h.deleteObjectStore("SWHealthLog")},
version:9});return lq()}
;function nq(a){return so(mq(),a)}
function oq(a,b){var c,d,e,f;return A(function(g){if(1==g.h)return c={startTime:W(),infos:{transactionType:"YT_IDB_TRANSACTION_TYPE_WRITE"},ticks:{}},g.yield(nq(b),2);if(3!=g.h)return d=g.i,e=Object.assign({},a,{options:JSON.parse(JSON.stringify(a.options)),interface:T("INNERTUBE_CONTEXT_CLIENT_NAME",0)}),g.yield($n(d,"LogsRequestsStore",e),3);f=g.i;c.ticks.tc=W();pq(c);return g.return(f)})}
function qq(a,b){var c,d,e,f,g,h,k,l;return A(function(n){if(1==n.h)return c={startTime:W(),infos:{transactionType:"YT_IDB_TRANSACTION_TYPE_READ"},ticks:{}},n.yield(nq(b),2);if(3!=n.h)return d=n.i,e=T("INNERTUBE_CONTEXT_CLIENT_NAME",0),f=[a,e,0],g=[a,e,W()],h=IDBKeyRange.bound(f,g),k="prev",U("use_fifo_for_networkless")&&(k="next"),l=void 0,n.yield(Xn(d,["LogsRequestsStore"],{mode:"readwrite",la:!0},function(p){return lo(p.objectStore("LogsRequestsStore").index("newRequestV2"),{query:h,direction:k},
function(r){r.getValue()&&(l=r.getValue(),"NEW"===a&&(l.status="QUEUED",r.update(l)))})}),3);
c.ticks.tc=W();pq(c);return n.return(l)})}
function rq(a,b){var c;return A(function(d){if(1==d.h)return d.yield(nq(b),2);c=d.i;return d.return(Xn(c,["LogsRequestsStore"],{mode:"readwrite",la:!0},function(e){var f=e.objectStore("LogsRequestsStore");return f.get(a).then(function(g){if(g)return g.status="QUEUED",Tn(f.h.put(g,void 0)).then(function(){return g})})}))})}
function sq(a,b,c,d){c=void 0===c?!0:c;var e;return A(function(f){if(1==f.h)return f.yield(nq(b),2);e=f.i;return f.return(Xn(e,["LogsRequestsStore"],{mode:"readwrite",la:!0},function(g){var h=g.objectStore("LogsRequestsStore");return h.get(a).then(function(k){return k?(k.status="NEW",c&&(k.sendCount+=1),void 0!==d&&(k.options.compress=d),Tn(h.h.put(k,void 0)).then(function(){return k})):Nn.resolve(void 0)})}))})}
function tq(a,b){var c;return A(function(d){if(1==d.h)return d.yield(nq(b),2);c=d.i;return d.return(c.delete("LogsRequestsStore",a))})}
function uq(a){var b,c;return A(function(d){if(1==d.h)return d.yield(nq(a),2);b=d.i;c=W()-2592E6;return d.yield(Xn(b,["LogsRequestsStore"],{mode:"readwrite",la:!0},function(e){return ho(e.objectStore("LogsRequestsStore"),{},function(f){if(f.getValue().timestamp<=c)return f.delete().then(function(){return io(f)})})}),0)})}
function vq(){A(function(a){return a.yield(Po(),0)})}
function pq(a){U("nwl_csi_killswitch")||zp("networkless_performance",a,{sampleRate:1})}
;var wq={accountStateChangeSignedIn:23,accountStateChangeSignedOut:24,delayedEventMetricCaptured:11,latencyActionBaselined:6,latencyActionInfo:7,latencyActionTicked:5,offlineTransferStatusChanged:2,offlineImageDownload:335,playbackStartStateChanged:9,systemHealthCaptured:3,mangoOnboardingCompleted:10,mangoPushNotificationReceived:230,mangoUnforkDbMigrationError:121,mangoUnforkDbMigrationSummary:122,mangoUnforkDbMigrationPreunforkDbVersionNumber:133,mangoUnforkDbMigrationPhoneMetadata:134,mangoUnforkDbMigrationPhoneStorage:135,
mangoUnforkDbMigrationStep:142,mangoAsyncApiMigrationEvent:223,mangoDownloadVideoResult:224,mangoHomepageVideoCount:279,mangoHomeV3State:295,mangoImageClientCacheHitEvent:273,sdCardStatusChanged:98,framesDropped:12,thumbnailHovered:13,deviceRetentionInfoCaptured:14,thumbnailLoaded:15,backToAppEvent:318,streamingStatsCaptured:17,offlineVideoShared:19,appCrashed:20,youThere:21,offlineStateSnapshot:22,mdxSessionStarted:25,mdxSessionConnected:26,mdxSessionDisconnected:27,bedrockResourceConsumptionSnapshot:28,
nextGenWatchWatchSwiped:29,kidsAccountsSnapshot:30,zeroStepChannelCreated:31,tvhtml5SearchCompleted:32,offlineSharePairing:34,offlineShareUnlock:35,mdxRouteDistributionSnapshot:36,bedrockRepetitiveActionTimed:37,unpluggedDegradationInfo:229,uploadMp4HeaderMoved:38,uploadVideoTranscoded:39,uploadProcessorStarted:46,uploadProcessorEnded:47,uploadProcessorReady:94,uploadProcessorRequirementPending:95,uploadProcessorInterrupted:96,uploadFrontendEvent:241,assetPackDownloadStarted:41,assetPackDownloaded:42,
assetPackApplied:43,assetPackDeleted:44,appInstallAttributionEvent:459,playbackSessionStopped:45,adBlockerMessagingShown:48,distributionChannelCaptured:49,dataPlanCpidRequested:51,detailedNetworkTypeCaptured:52,sendStateUpdated:53,receiveStateUpdated:54,sendDebugStateUpdated:55,receiveDebugStateUpdated:56,kidsErrored:57,mdxMsnSessionStatsFinished:58,appSettingsCaptured:59,mdxWebSocketServerHttpError:60,mdxWebSocketServer:61,startupCrashesDetected:62,coldStartInfo:435,offlinePlaybackStarted:63,liveChatMessageSent:225,
liveChatUserPresent:434,liveChatBeingModerated:457,liveCreationCameraUpdated:64,liveCreationEncodingCaptured:65,liveCreationError:66,liveCreationHealthUpdated:67,liveCreationVideoEffectsCaptured:68,liveCreationStageOccured:75,liveCreationBroadcastScheduled:123,liveCreationArchiveReplacement:149,liveCreationCostreamingConnection:421,liveCreationStreamWebrtcStats:288,mdxSessionRecoveryStarted:69,mdxSessionRecoveryCompleted:70,mdxSessionRecoveryStopped:71,visualElementShown:72,visualElementHidden:73,
visualElementGestured:78,visualElementStateChanged:208,screenCreated:156,playbackAssociated:202,visualElementAttached:215,playbackContextEvent:214,cloudCastingPlaybackStarted:74,webPlayerApiCalled:76,tvhtml5AccountDialogOpened:79,foregroundHeartbeat:80,foregroundHeartbeatScreenAssociated:111,kidsOfflineSnapshot:81,mdxEncryptionSessionStatsFinished:82,playerRequestCompleted:83,liteSchedulerStatistics:84,mdxSignIn:85,spacecastMetadataLookupRequested:86,spacecastBatchLookupRequested:87,spacecastSummaryRequested:88,
spacecastPlayback:89,spacecastDiscovery:90,tvhtml5LaunchUrlComponentChanged:91,mdxBackgroundPlaybackRequestCompleted:92,mdxBrokenAdditionalDataDeviceDetected:93,tvhtml5LocalStorage:97,tvhtml5DeviceStorageStatus:147,autoCaptionsAvailable:99,playbackScrubbingEvent:339,flexyState:100,interfaceOrientationCaptured:101,mainAppBrowseFragmentCache:102,offlineCacheVerificationFailure:103,offlinePlaybackExceptionDigest:217,vrCopresenceStats:104,vrCopresenceSyncStats:130,vrCopresenceCommsStats:137,vrCopresencePartyStats:153,
vrCopresenceEmojiStats:213,vrCopresenceEvent:141,vrCopresenceFlowTransitEvent:160,vrCowatchPartyEvent:492,vrPlaybackEvent:345,kidsAgeGateTracking:105,offlineDelayAllowedTracking:106,mainAppAutoOfflineState:107,videoAsThumbnailDownload:108,videoAsThumbnailPlayback:109,liteShowMore:110,renderingError:118,kidsProfilePinGateTracking:119,abrTrajectory:124,scrollEvent:125,streamzIncremented:126,kidsProfileSwitcherTracking:127,kidsProfileCreationTracking:129,buyFlowStarted:136,mbsConnectionInitiated:138,
mbsPlaybackInitiated:139,mbsLoadChildren:140,liteProfileFetcher:144,mdxRemoteTransaction:146,reelPlaybackError:148,reachabilityDetectionEvent:150,mobilePlaybackEvent:151,courtsidePlayerStateChanged:152,musicPersistentCacheChecked:154,musicPersistentCacheCleared:155,playbackInterrupted:157,playbackInterruptionResolved:158,fixFopFlow:159,anrDetection:161,backstagePostCreationFlowEnded:162,clientError:163,gamingAccountLinkStatusChanged:164,liteHousewarming:165,buyFlowEvent:167,kidsParentalGateTracking:168,
kidsSignedOutSettingsStatus:437,kidsSignedOutPauseHistoryFixStatus:438,tvhtml5WatchdogViolation:444,ypcUpgradeFlow:169,yongleStudy:170,ypcUpdateFlowStarted:171,ypcUpdateFlowCancelled:172,ypcUpdateFlowSucceeded:173,ypcUpdateFlowFailed:174,liteGrowthkitPromo:175,paymentFlowStarted:341,transactionFlowShowPaymentDialog:405,transactionFlowStarted:176,transactionFlowSecondaryDeviceStarted:222,transactionFlowSecondaryDeviceSignedOutStarted:383,transactionFlowCancelled:177,transactionFlowPaymentCallBackReceived:387,
transactionFlowPaymentSubmitted:460,transactionFlowPaymentSucceeded:329,transactionFlowSucceeded:178,transactionFlowFailed:179,transactionFlowPlayBillingConnectionStartEvent:428,transactionFlowSecondaryDeviceSuccess:458,transactionFlowErrorEvent:411,liteVideoQualityChanged:180,watchBreakEnablementSettingEvent:181,watchBreakFrequencySettingEvent:182,videoEffectsCameraPerformanceMetrics:183,adNotify:184,startupTelemetry:185,playbackOfflineFallbackUsed:186,outOfMemory:187,ypcPauseFlowStarted:188,ypcPauseFlowCancelled:189,
ypcPauseFlowSucceeded:190,ypcPauseFlowFailed:191,uploadFileSelected:192,ypcResumeFlowStarted:193,ypcResumeFlowCancelled:194,ypcResumeFlowSucceeded:195,ypcResumeFlowFailed:196,adsClientStateChange:197,ypcCancelFlowStarted:198,ypcCancelFlowCancelled:199,ypcCancelFlowSucceeded:200,ypcCancelFlowFailed:201,ypcCancelFlowGoToPaymentProcessor:402,ypcDeactivateFlowStarted:320,ypcRedeemFlowStarted:203,ypcRedeemFlowCancelled:204,ypcRedeemFlowSucceeded:205,ypcRedeemFlowFailed:206,ypcFamilyCreateFlowStarted:258,
ypcFamilyCreateFlowCancelled:259,ypcFamilyCreateFlowSucceeded:260,ypcFamilyCreateFlowFailed:261,ypcFamilyManageFlowStarted:262,ypcFamilyManageFlowCancelled:263,ypcFamilyManageFlowSucceeded:264,ypcFamilyManageFlowFailed:265,restoreContextEvent:207,embedsAdEvent:327,autoplayTriggered:209,clientDataErrorEvent:210,experimentalVssValidation:211,tvhtml5TriggeredEvent:212,tvhtml5FrameworksFieldTrialResult:216,tvhtml5FrameworksFieldTrialStart:220,musicOfflinePreferences:218,watchTimeSegment:219,appWidthLayoutError:221,
accountRegistryChange:226,userMentionAutoCompleteBoxEvent:227,downloadRecommendationEnablementSettingEvent:228,musicPlaybackContentModeChangeEvent:231,offlineDbOpenCompleted:232,kidsFlowEvent:233,kidsFlowCorpusSelectedEvent:234,videoEffectsEvent:235,unpluggedOpsEogAnalyticsEvent:236,playbackAudioRouteEvent:237,interactionLoggingDebugModeError:238,offlineYtbRefreshed:239,kidsFlowError:240,musicAutoplayOnLaunchAttempted:242,deviceContextActivityEvent:243,deviceContextEvent:244,templateResolutionException:245,
musicSideloadedPlaylistServiceCalled:246,embedsStorageAccessNotChecked:247,embedsHasStorageAccessResult:248,embedsItpPlayedOnReload:249,embedsRequestStorageAccessResult:250,embedsShouldRequestStorageAccessResult:251,embedsRequestStorageAccessState:256,embedsRequestStorageAccessFailedState:257,embedsItpWatchLaterResult:266,searchSuggestDecodingPayloadFailure:252,siriShortcutActivated:253,tvhtml5KeyboardPerformance:254,latencyActionSpan:255,elementsLog:267,ytbFileOpened:268,tfliteModelError:269,apiTest:270,
yongleUsbSetup:271,touStrikeInterstitialEvent:272,liteStreamToSave:274,appBundleClientEvent:275,ytbFileCreationFailed:276,adNotifyFailure:278,ytbTransferFailed:280,blockingRequestFailed:281,liteAccountSelector:282,liteAccountUiCallbacks:283,dummyPayload:284,browseResponseValidationEvent:285,entitiesError:286,musicIosBackgroundFetch:287,mdxNotificationEvent:289,layersValidationError:290,musicPwaInstalled:291,liteAccountCleanup:292,html5PlayerHealthEvent:293,watchRestoreAttempt:294,liteAccountSignIn:296,
notaireEvent:298,kidsVoiceSearchEvent:299,adNotifyFilled:300,delayedEventDropped:301,analyticsSearchEvent:302,systemDarkThemeOptOutEvent:303,flowEvent:304,networkConnectivityBaselineEvent:305,ytbFileImported:306,downloadStreamUrlExpired:307,directSignInEvent:308,lyricImpressionEvent:309,accessibilityStateEvent:310,tokenRefreshEvent:311,genericAttestationExecution:312,tvhtml5VideoSeek:313,unpluggedAutoPause:314,scrubbingEvent:315,bedtimeReminderEvent:317,tvhtml5UnexpectedRestart:319,tvhtml5StabilityTraceEvent:478,
tvhtml5OperationHealth:467,tvhtml5WatchKeyEvent:321,voiceLanguageChanged:322,tvhtml5LiveChatStatus:323,parentToolsCorpusSelectedEvent:324,offerAdsEnrollmentInitiated:325,networkQualityIntervalEvent:326,deviceStartupMetrics:328,heartbeatActionPlayerTransitioned:330,tvhtml5Lifecycle:331,heartbeatActionPlayerHalted:332,adaptiveInlineMutedSettingEvent:333,mainAppLibraryLoadingState:334,thirdPartyLogMonitoringEvent:336,appShellAssetLoadReport:337,tvhtml5AndroidAttestation:338,tvhtml5StartupSoundEvent:340,
iosBackgroundRefreshTask:342,iosBackgroundProcessingTask:343,sliEventBatch:344,postImpressionEvent:346,musicSideloadedPlaylistExport:347,idbUnexpectedlyClosed:348,voiceSearchEvent:349,mdxSessionCastEvent:350,idbQuotaExceeded:351,idbTransactionEnded:352,idbTransactionAborted:353,tvhtml5KeyboardLogging:354,idbIsSupportedCompleted:355,creatorStudioMobileEvent:356,idbDataCorrupted:357,parentToolsAppChosenEvent:358,webViewBottomSheetResized:359,activeStateControllerScrollPerformanceSummary:360,navigatorValidation:361,
mdxSessionHeartbeat:362,clientHintsPolyfillDiagnostics:363,clientHintsPolyfillEvent:364,proofOfOriginTokenError:365,kidsAddedAccountSummary:366,musicWearableDevice:367,ypcRefundFlowEvent:368,tvhtml5PlaybackMeasurementEvent:369,tvhtml5WatermarkMeasurementEvent:370,clientExpGcfPropagationEvent:371,mainAppReferrerIntent:372,leaderLockEnded:373,leaderLockAcquired:374,googleHatsEvent:375,persistentLensLaunchEvent:376,parentToolsChildWelcomeChosenEvent:378,browseThumbnailPreloadEvent:379,finalPayload:380,
mdxDialAdditionalDataUpdateEvent:381,webOrchestrationTaskLifecycleRecord:382,startupSignalEvent:384,accountError:385,gmsDeviceCheckEvent:386,accountSelectorEvent:388,accountUiCallbacks:389,mdxDialAdditionalDataProbeEvent:390,downloadsSearchIcingApiStats:391,downloadsSearchIndexUpdatedEvent:397,downloadsSearchIndexSnapshot:398,dataPushClientEvent:392,kidsCategorySelectedEvent:393,mdxDeviceManagementSnapshotEvent:394,prefetchRequested:395,prefetchableCommandExecuted:396,gelDebuggingEvent:399,webLinkTtsPlayEnd:400,
clipViewInvalid:401,persistentStorageStateChecked:403,cacheWipeoutEvent:404,playerEvent:410,sfvEffectPipelineStartedEvent:412,sfvEffectPipelinePausedEvent:429,sfvEffectPipelineEndedEvent:413,sfvEffectChosenEvent:414,sfvEffectLoadedEvent:415,sfvEffectUserInteractionEvent:465,sfvEffectFirstFrameProcessedLatencyEvent:416,sfvEffectAggregatedFramesProcessedLatencyEvent:417,sfvEffectAggregatedFramesDroppedEvent:418,sfvEffectPipelineErrorEvent:430,sfvEffectGraphFrozenEvent:419,sfvEffectGlThreadBlockedEvent:420,
mdeVideoChangedEvent:442,mdePlayerPerformanceMetrics:472,mdeExporterEvent:497,genericClientExperimentEvent:423,homePreloadTaskScheduled:424,homePreloadTaskExecuted:425,homePreloadCacheHit:426,polymerPropertyChangedInObserver:427,applicationStarted:431,networkCronetRttBatch:432,networkCronetRttSummary:433,repeatChapterLoopEvent:436,seekCancellationEvent:462,lockModeTimeoutEvent:483,offlineTransferStarted:4,musicOfflineMixtapePreferencesChanged:16,mangoDailyNewVideosNotificationAttempt:40,mangoDailyNewVideosNotificationError:77,
dtwsPlaybackStarted:112,dtwsTileFetchStarted:113,dtwsTileFetchCompleted:114,dtwsTileFetchStatusChanged:145,dtwsKeyframeDecoderBufferSent:115,dtwsTileUnderflowedOnNonkeyframe:116,dtwsBackfillFetchStatusChanged:143,dtwsBackfillUnderflowed:117,dtwsAdaptiveLevelChanged:128,blockingVisitorIdTimeout:277,liteSocial:18,mobileJsInvocation:297,biscottiBasedDetection:439,coWatchStateChange:440,embedsVideoDataDidChange:441,shortsFirst:443,cruiseControlEvent:445,qoeClientLoggingContext:446,atvRecommendationJobExecuted:447,
tvhtml5UserFeedback:448,producerProjectCreated:449,producerProjectOpened:450,producerProjectDeleted:451,producerProjectElementAdded:453,producerProjectElementRemoved:454,tvhtml5ShowClockEvent:455,deviceCapabilityCheckMetrics:456,youtubeClearcutEvent:461,offlineBrowseFallbackEvent:463,getCtvTokenEvent:464,startupDroppedFramesSummary:466,screenshotEvent:468,miniAppPlayEvent:469,elementsDebugCounters:470,fontLoadEvent:471,webKillswitchReceived:473,webKillswitchExecuted:474,cameraOpenEvent:475,manualSmoothnessMeasurement:476,
tvhtml5AppQualityEvent:477,polymerPropertyAccessEvent:479,miniAppSdkUsage:480,cobaltTelemetryEvent:481,crossDevicePlayback:482,channelCreatedWithObakeImage:484,channelEditedWithObakeImage:485,offlineDeleteEvent:486,crossDeviceNotificationTransfer:487,androidIntentEvent:488,unpluggedAmbientInterludesCounterfactualEvent:489,keyPlaysPlayback:490,shortsCreationFallbackEvent:493,vssData:491,castMatch:494,miniAppPerformanceMetrics:495,userFeedbackEvent:496};var xq={},yq=To("ServiceWorkerLogsDatabase",{wb:(xq.SWHealthLog={Eb:1},xq),shared:!0,upgrade:function(a,b){b(1)&&fo(Yn(a,"SWHealthLog",{keyPath:"id",autoIncrement:!0}),"swHealthNewRequest",["interface","timestamp"])},
version:1});function zq(a){return so(yq(),a)}
function Aq(a){var b,c;A(function(d){if(1==d.h)return d.yield(zq(a),2);b=d.i;c=W()-2592E6;return d.yield(Xn(b,["SWHealthLog"],{mode:"readwrite",la:!0},function(e){return ho(e.objectStore("SWHealthLog"),{},function(f){if(f.getValue().timestamp<=c)return f.delete().then(function(){return io(f)})})}),0)})}
function Bq(a){var b;return A(function(c){if(1==c.h)return c.yield(zq(a),2);b=c.i;return c.yield(b.clear("SWHealthLog"),0)})}
;var Cq={},Dq=0;function Eq(a){var b=new Image,c=""+Dq++;Cq[c]=b;b.onload=b.onerror=function(){delete Cq[c]};
b.src=a}
;var Fq;function Gq(){Fq||(Fq=new hn("yt.offline"));return Fq}
function Hq(a){if(U("offline_error_handling")){var b=Gq().get("errors",!0)||{};b[a.message]={name:a.name,stack:a.stack};a.level&&(b[a.message].level=a.level);Gq().set("errors",b,2592E3,!0)}}
;function Iq(){this.h=new Map;this.i=!1}
function Jq(){if(!Iq.h){var a=E("yt.networkRequestMonitor.instance")||new Iq;D("yt.networkRequestMonitor.instance",a);Iq.h=a}return Iq.h}
Iq.prototype.requestComplete=function(a,b){b&&(this.i=!0);a=this.removeParams(a);this.h.get(a)||this.h.set(a,b)};
Iq.prototype.isEndpointCFR=function(a){a=this.removeParams(a);return(a=this.h.get(a))?!1:!1===a&&this.i?!0:null};
Iq.prototype.removeParams=function(a){return a.split("?")[0]};
Iq.prototype.removeParams=Iq.prototype.removeParams;Iq.prototype.isEndpointCFR=Iq.prototype.isEndpointCFR;Iq.prototype.requestComplete=Iq.prototype.requestComplete;Iq.getInstance=Jq;function Kq(){wd.call(this);var a=this;this.j=!1;this.i=Mi();this.i.listen("networkstatus-online",function(){if(a.j&&U("offline_error_handling")){var b=Gq().get("errors",!0);if(b){for(var c in b)if(b[c]){var d=new V(c,"sent via offline_errors");d.name=b[c].name;d.stack=b[c].stack;d.level=b[c].level;el(d)}Gq().set("errors",{},2592E3,!0)}}})}
x(Kq,wd);function Lq(){if(!Kq.h){var a=E("yt.networkStatusManager.instance")||new Kq;D("yt.networkStatusManager.instance",a);Kq.h=a}return Kq.h}
m=Kq.prototype;m.va=function(){return this.i.va()};
m.fb=function(a){this.i.i=a};
m.ce=function(){var a=window.navigator.onLine;return void 0===a?!0:a};
m.Td=function(){this.j=!0};
m.listen=function(a,b){return this.i.listen(a,b)};
m.oc=function(a){a=Ki(this.i,a);a.then(function(b){U("use_cfr_monitor")&&Jq().requestComplete("generate_204",b)});
return a};
Kq.prototype.sendNetworkCheckRequest=Kq.prototype.oc;Kq.prototype.listen=Kq.prototype.listen;Kq.prototype.enableErrorFlushing=Kq.prototype.Td;Kq.prototype.getWindowStatus=Kq.prototype.ce;Kq.prototype.networkStatusHint=Kq.prototype.fb;Kq.prototype.isNetworkAvailable=Kq.prototype.va;Kq.getInstance=Lq;function Mq(a){a=void 0===a?{}:a;wd.call(this);var b=this;this.i=this.m=0;this.j=Lq();var c=E("yt.networkStatusManager.instance.listen").bind(this.j);c&&(a.rateLimit?(this.rateLimit=a.rateLimit,c("networkstatus-online",function(){Nq(b,"publicytnetworkstatus-online")}),c("networkstatus-offline",function(){Nq(b,"publicytnetworkstatus-offline")})):(c("networkstatus-online",function(){xd(b,"publicytnetworkstatus-online")}),c("networkstatus-offline",function(){xd(b,"publicytnetworkstatus-offline")})))}
x(Mq,wd);Mq.prototype.va=function(){var a=E("yt.networkStatusManager.instance.isNetworkAvailable");return a?a.bind(this.j)():!0};
Mq.prototype.fb=function(a){var b=E("yt.networkStatusManager.instance.networkStatusHint").bind(this.j);b&&b(a)};
Mq.prototype.oc=function(a){var b=this,c;return A(function(d){c=E("yt.networkStatusManager.instance.sendNetworkCheckRequest").bind(b.j);return U("skip_network_check_if_cfr")&&Jq().isEndpointCFR("generate_204")?d.return(new Promise(function(e){var f;b.fb((null==(f=window.navigator)?void 0:f.onLine)||!0);e(b.va())})):c?d.return(c(a)):d.return(!0)})};
function Nq(a,b){a.rateLimit?a.i?(Ni.qa(a.m),a.m=Ni.pa(function(){a.l!==b&&(xd(a,b),a.l=b,a.i=W())},a.rateLimit-(W()-a.i))):(xd(a,b),a.l=b,a.i=W()):xd(a,b)}
;var Oq;function Pq(){var a=bq.call;Oq||(Oq=new Mq({Mg:!0,Fg:!0}));a.call(bq,this,{ha:{Od:uq,nb:tq,cd:qq,re:rq,Kc:sq,set:oq},fa:Oq,handleError:function(b,c,d){var e,f=null==d?void 0:null==(e=d.error)?void 0:e.code;if(400===f||415===f){var g;fl(new V(b.message,c,null==d?void 0:null==(g=d.error)?void 0:g.code),void 0,void 0,void 0,!0)}else el(b)},
ob:fl,sendFn:Qq,now:W,Fd:Hq,Da:gn(),Jc:"publicytnetworkstatus-online",Gc:"publicytnetworkstatus-offline",cc:!0,Zb:.1,lc:yl("potential_esf_error_limit",10),Y:U,Hb:!(zm()&&Rq())});this.j=new wi;U("networkless_immediately_drop_all_requests")&&vq();Qo("LogsDatabaseV2")}
x(Pq,bq);function Sq(){var a=E("yt.networklessRequestController.instance");a||(a=new Pq,D("yt.networklessRequestController.instance",a),U("networkless_logging")&&Fo().then(function(b){a.X=b;dq(a);a.j.resolve();a.cc&&Math.random()<=a.Zb&&a.X&&Aq(a.X);U("networkless_immediately_drop_sw_health_store")&&Tq(a)}));
return a}
Pq.prototype.writeThenSend=function(a,b){b||(b={});b=Uq(a,b);zm()||(this.h=!1);bq.prototype.writeThenSend.call(this,a,b)};
Pq.prototype.sendThenWrite=function(a,b,c){b||(b={});b=Uq(a,b);zm()||(this.h=!1);bq.prototype.sendThenWrite.call(this,a,b,c)};
Pq.prototype.sendAndWrite=function(a,b){b||(b={});b=Uq(a,b);zm()||(this.h=!1);bq.prototype.sendAndWrite.call(this,a,b)};
Pq.prototype.awaitInitialization=function(){return this.j.promise};
function Tq(a){var b;A(function(c){if(!a.X)throw b=Jn("clearSWHealthLogsDb"),b;return c.return(Bq(a.X).catch(function(d){a.handleError(d)}))})}
function Qq(a,b,c,d){d=void 0===d?!1:d;b=U("web_fp_via_jspb")?Object.assign({},b):b;U("use_cfr_monitor")&&Vq(a,b);if(U("use_request_time_ms_header"))b.headers&&pl(a)&&(b.headers["X-Goog-Request-Time"]=JSON.stringify(Math.round(W())));else{var e;if(null==(e=b.postParams)?0:e.requestTimeMs)b.postParams.requestTimeMs=Math.round(W())}if(c&&0===Object.keys(b).length){var f=void 0===f?"":f;var g=void 0===g?!1:g;var h=void 0===h?!1:h;if(a)if(f)Dl(a,void 0,"POST",f,void 0);else if(T("USE_NET_AJAX_FOR_PING_TRANSPORT",
!1)||h)Dl(a,void 0,"GET","",void 0,void 0,g,h);else{b:{try{var k=new cb({url:a});if(k.j&&k.i||k.l){var l=mc(nc(5,a)),n;if(!(n=!l||!l.endsWith("/aclk"))){var p=a.search(vc),r=uc(a,0,"ri",p);if(0>r)var t=null;else{var y=a.indexOf("&",r);if(0>y||y>p)y=p;t=decodeURIComponent(a.slice(r+3,-1!==y?y:0).replace(/\+/g," "))}n="1"!==t}var v=!n;break b}}catch(G){}v=!1}if(v){b:{try{if(window.navigator&&window.navigator.sendBeacon&&window.navigator.sendBeacon(a,"")){var z=!0;break b}}catch(G){}z=!1}c=z?!0:!1}else c=
!1;c||Eq(a)}}else b.compress?b.postBody?("string"!==typeof b.postBody&&(b.postBody=JSON.stringify(b.postBody)),Rp(a,b.postBody,b,Hl,d)):Rp(a,JSON.stringify(b.postParams),b,Gl,d):Hl(a,b)}
function Uq(a,b){U("use_event_time_ms_header")&&pl(a)&&(b.headers||(b.headers={}),b.headers["X-Goog-Event-Time"]=JSON.stringify(Math.round(W())));return b}
function Vq(a,b){var c=b.onError?b.onError:function(){};
b.onError=function(e,f){Jq().requestComplete(a,!1);c(e,f)};
var d=b.onSuccess?b.onSuccess:function(){};
b.onSuccess=function(e,f){Jq().requestComplete(a,!0);d(e,f)}}
function Rq(){return"www.youtube-nocookie.com"!==oc(document.location.toString())}
;var Wq=!1,Xq=C.ytNetworklessLoggingInitializationOptions||{isNwlInitialized:Wq};D("ytNetworklessLoggingInitializationOptions",Xq);function Yq(){var a;A(function(b){if(1==b.h)return b.yield(Fo(),2);a=b.i;if(!a||!zm()&&!U("nwl_init_require_datasync_id_killswitch")||!Rq())return b.B(0);Wq=!0;Xq.isNwlInitialized=Wq;return b.yield(Sq().awaitInitialization(),0)})}
;function Zq(a){var b=this;this.config_=null;a?this.config_=a:gp()&&(this.config_=hp());Cm(function(){$p(b)},5E3)}
Zq.prototype.isReady=function(){!this.config_&&gp()&&(this.config_=hp());return!!this.config_};
function aq(a,b,c,d){function e(y){y=void 0===y?!1:y;var v;if(d.retry&&"www.youtube-nocookie.com"!=h&&(y||U("skip_ls_gel_retry")||"application/json"!==g.headers["Content-Type"]||(v=Yp(b,c,l,k)),v)){var z=g.onSuccess,G=g.onFetchSuccess;g.onSuccess=function(S,da){Zp(v);z(S,da)};
c.onFetchSuccess=function(S,da){Zp(v);G(S,da)}}try{if(y&&d.retry&&!d.networklessOptions.bypassNetworkless)g.method="POST",d.networklessOptions.writeThenSend?Sq().writeThenSend(t,g):Sq().sendAndWrite(t,g);
else if(d.compress){var K=!d.networklessOptions.writeThenSend;if(g.postBody){var N=g.postBody;"string"!==typeof N&&(N=JSON.stringify(g.postBody));Rp(t,N,g,Hl,K)}else Rp(t,JSON.stringify(g.postParams),g,Gl,K)}else U("web_all_payloads_via_jspb")?Hl(t,g):Gl(t,g)}catch(S){if("InvalidAccessError"===S.name)v&&(Zp(v),v=0),fl(Error("An extension is blocking network request."));else throw S;}v&&Cm(function(){$p(a)},5E3)}
!T("VISITOR_DATA")&&"visitor_id"!==b&&.01>Math.random()&&fl(new V("Missing VISITOR_DATA when sending innertube request.",b,c,d));if(!a.isReady()){var f=new V("innertube xhrclient not ready",b,c,d);el(f);throw f;}var g={headers:d.headers||{},method:"POST",postParams:c,postBody:d.postBody,postBodyFormat:d.postBodyFormat||"JSON",onTimeout:function(){d.onTimeout()},
onFetchTimeout:d.onTimeout,onSuccess:function(y,v){if(d.onSuccess)d.onSuccess(v)},
onFetchSuccess:function(y){if(d.onSuccess)d.onSuccess(y)},
onError:function(y,v){if(d.onError)d.onError(v)},
onFetchError:function(y){if(d.onError)d.onError(y)},
timeout:d.timeout,withCredentials:!0,compress:d.compress};g.headers["Content-Type"]||(g.headers["Content-Type"]="application/json");var h="";(f=a.config_.le)&&(h=f);var k=a.config_.ne||!1,l=jp(k,h,d);Object.assign(g.headers,l);(f=g.headers.Authorization)&&!h&&k&&(g.headers["x-origin"]=window.location.origin);var n="/youtubei/"+a.config_.innertubeApiVersion+"/"+b,p={alt:"json"},r=a.config_.me&&f;r=r&&f.startsWith("Bearer");r||(p.key=a.config_.innertubeApiKey);var t=ol(""+h+n,p||{},!0);(E("ytNetworklessLoggingInitializationOptions")?
Xq.isNwlInitialized:Wq)?Do().then(function(y){e(y)}):e(!1)}
;var $q=0,ar=Yc?"webkit":Xc?"moz":Vc?"ms":Uc?"o":"";D("ytDomDomGetNextId",E("ytDomDomGetNextId")||function(){return++$q});var br={stopImmediatePropagation:1,stopPropagation:1,preventMouseEvent:1,preventManipulation:1,preventDefault:1,layerX:1,layerY:1,screenX:1,screenY:1,scale:1,rotation:1,webkitMovementX:1,webkitMovementY:1};
function cr(a){this.type="";this.state=this.source=this.data=this.currentTarget=this.relatedTarget=this.target=null;this.charCode=this.keyCode=0;this.metaKey=this.shiftKey=this.ctrlKey=this.altKey=!1;this.rotation=this.clientY=this.clientX=0;this.scale=1;this.changedTouches=this.touches=null;try{if(a=a||window.event){this.event=a;for(var b in a)b in br||(this[b]=a[b]);this.scale=a.scale;this.rotation=a.rotation;var c=a.target||a.srcElement;c&&3==c.nodeType&&(c=c.parentNode);this.target=c;var d=a.relatedTarget;
if(d)try{d=d.nodeName?d:null}catch(e){d=null}else"mouseover"==this.type?d=a.fromElement:"mouseout"==this.type&&(d=a.toElement);this.relatedTarget=d;this.clientX=void 0!=a.clientX?a.clientX:a.pageX;this.clientY=void 0!=a.clientY?a.clientY:a.pageY;this.keyCode=a.keyCode?a.keyCode:a.which;this.charCode=a.charCode||("keypress"==this.type?this.keyCode:0);this.altKey=a.altKey;this.ctrlKey=a.ctrlKey;this.shiftKey=a.shiftKey;this.metaKey=a.metaKey;this.h=a.pageX;this.i=a.pageY}}catch(e){}}
function dr(a){if(document.body&&document.documentElement){var b=document.body.scrollTop+document.documentElement.scrollTop;a.h=a.clientX+(document.body.scrollLeft+document.documentElement.scrollLeft);a.i=a.clientY+b}}
cr.prototype.preventDefault=function(){this.event&&(this.event.returnValue=!1,this.event.preventDefault&&this.event.preventDefault())};
cr.prototype.stopPropagation=function(){this.event&&(this.event.cancelBubble=!0,this.event.stopPropagation&&this.event.stopPropagation())};
cr.prototype.stopImmediatePropagation=function(){this.event&&(this.event.cancelBubble=!0,this.event.stopImmediatePropagation&&this.event.stopImmediatePropagation())};var Mb=C.ytEventsEventsListeners||{};D("ytEventsEventsListeners",Mb);var er=C.ytEventsEventsCounter||{count:0};D("ytEventsEventsCounter",er);
function fr(a,b,c,d){d=void 0===d?{}:d;a.addEventListener&&("mouseenter"!=b||"onmouseenter"in document?"mouseleave"!=b||"onmouseenter"in document?"mousewheel"==b&&"MozBoxSizing"in document.documentElement.style&&(b="MozMousePixelScroll"):b="mouseout":b="mouseover");return Lb(function(e){var f="boolean"===typeof e[4]&&e[4]==!!d,g=Ra(e[4])&&Ra(d)&&Qb(e[4],d);return!!e.length&&e[0]==a&&e[1]==b&&e[2]==c&&(f||g)})}
function gr(a,b,c,d){d=void 0===d?{}:d;if(!a||!a.addEventListener&&!a.attachEvent)return"";var e=fr(a,b,c,d);if(e)return e;e=++er.count+"";var f=!("mouseenter"!=b&&"mouseleave"!=b||!a.addEventListener||"onmouseenter"in document);var g=f?function(h){h=new cr(h);if(!Hd(h.relatedTarget,function(k){return k==a}))return h.currentTarget=a,h.type=b,c.call(a,h)}:function(h){h=new cr(h);
h.currentTarget=a;return c.call(a,h)};
g=dl(g);a.addEventListener?("mouseenter"==b&&f?b="mouseover":"mouseleave"==b&&f?b="mouseout":"mousewheel"==b&&"MozBoxSizing"in document.documentElement.style&&(b="MozMousePixelScroll"),hr()||"boolean"===typeof d?a.addEventListener(b,g,d):a.addEventListener(b,g,!!d.capture)):a.attachEvent("on"+b,g);Mb[e]=[a,b,c,g,d];return e}
function ir(a){a&&("string"==typeof a&&(a=[a]),Db(a,function(b){if(b in Mb){var c=Mb[b],d=c[0],e=c[1],f=c[3];c=c[4];d.removeEventListener?hr()||"boolean"===typeof c?d.removeEventListener(e,f,c):d.removeEventListener(e,f,!!c.capture):d.detachEvent&&d.detachEvent("on"+e,f);delete Mb[b]}}))}
var hr=Cd(function(){var a=!1;try{var b=Object.defineProperty({},"capture",{get:function(){a=!0}});
window.addEventListener("test",null,b)}catch(c){}return a});function jr(a){this.F=a;this.h=null;this.l=0;this.v=null;this.m=0;this.i=[];for(a=0;4>a;a++)this.i.push(0);this.j=0;this.W=gr(window,"mousemove",Xa(this.ba,this));a=Xa(this.K,this);"function"===typeof a&&(a=dl(a));this.da=window.setInterval(a,25)}
$a(jr,F);jr.prototype.ba=function(a){void 0===a.h&&dr(a);var b=a.h;void 0===a.i&&dr(a);this.h=new Dd(b,a.i)};
jr.prototype.K=function(){if(this.h){var a=W();if(0!=this.l){var b=this.v,c=this.h,d=b.x-c.x;b=b.y-c.y;d=Math.sqrt(d*d+b*b)/(a-this.l);this.i[this.j]=.5<Math.abs((d-this.m)/this.m)?1:0;for(c=b=0;4>c;c++)b+=this.i[c]||0;3<=b&&this.F();this.m=d}this.l=a;this.v=this.h;this.j=(this.j+1)%4}};
jr.prototype.U=function(){window.clearInterval(this.da);ir(this.W)};var kr={};
function lr(a){var b=void 0===a?{}:a;a=void 0===b.Ce?!1:b.Ce;b=void 0===b.Ud?!0:b.Ud;if(null==E("_lact",window)){var c=parseInt(T("LACT"),10);c=isFinite(c)?Date.now()-Math.max(c,0):-1;D("_lact",c,window);D("_fact",c,window);-1==c&&mr();gr(document,"keydown",mr);gr(document,"keyup",mr);gr(document,"mousedown",mr);gr(document,"mouseup",mr);a?gr(window,"touchmove",function(){nr("touchmove",200)},{passive:!0}):(gr(window,"resize",function(){nr("resize",200)}),b&&gr(window,"scroll",function(){nr("scroll",200)}));
new jr(function(){nr("mouse",100)});
gr(document,"touchstart",mr,{passive:!0});gr(document,"touchend",mr,{passive:!0})}}
function nr(a,b){kr[a]||(kr[a]=!0,Ni.pa(function(){mr();kr[a]=!1},b))}
function mr(){null==E("_lact",window)&&lr();var a=Date.now();D("_lact",a,window);-1==E("_fact",window)&&D("_fact",a,window);(a=E("ytglobal.ytUtilActivityCallback_"))&&a()}
function or(){var a=E("_lact",window);return null==a?-1:Math.max(Date.now()-a,0)}
;var sr=C.ytPubsubPubsubInstance||new M,tr=C.ytPubsubPubsubSubscribedKeys||{},ur=C.ytPubsubPubsubTopicToKeys||{},vr=C.ytPubsubPubsubIsSynchronous||{};function wr(a,b){var c=xr();if(c&&b){var d=c.subscribe(a,function(){function e(){tr[d]&&b.apply&&"function"==typeof b.apply&&b.apply(window,f)}
var f=arguments;try{vr[a]?e():vl(e,0)}catch(g){el(g)}},void 0);
tr[d]=!0;ur[a]||(ur[a]=[]);ur[a].push(d);return d}return 0}
function yr(a){var b=xr();b&&("number"===typeof a?a=[a]:"string"===typeof a&&(a=[parseInt(a,10)]),Db(a,function(c){b.unsubscribeByKey(c);delete tr[c]}))}
function zr(a,b){var c=xr();c&&c.publish.apply(c,arguments)}
function Ar(a){var b=xr();if(b)if(b.clear(a),a)Br(a);else for(var c in ur)Br(c)}
function xr(){return C.ytPubsubPubsubInstance}
function Br(a){ur[a]&&(a=ur[a],Db(a,function(b){tr[b]&&delete tr[b]}),a.length=0)}
M.prototype.subscribe=M.prototype.subscribe;M.prototype.unsubscribeByKey=M.prototype.Ab;M.prototype.publish=M.prototype.Xa;M.prototype.clear=M.prototype.clear;D("ytPubsubPubsubInstance",sr);D("ytPubsubPubsubTopicToKeys",ur);D("ytPubsubPubsubIsSynchronous",vr);D("ytPubsubPubsubSubscribedKeys",tr);var Cr=Symbol("injectionDeps");function Dr(a){this.name=a}
Dr.prototype.toString=function(){return"InjectionToken("+this.name+")"};
function Er(a){this.key=a}
function Fr(){this.i=new Map;this.j=new Map;this.h=new Map}
function Gr(a,b){a.i.set(b.nc,b);var c=a.j.get(b.nc);if(c)try{c.Tg(a.resolve(b.nc))}catch(d){c.Rg(d)}}
Fr.prototype.resolve=function(a){return a instanceof Er?Hr(this,a.key,[],!0):Hr(this,a,[])};
function Hr(a,b,c,d){d=void 0===d?!1:d;if(-1<c.indexOf(b))throw Error("Deps cycle for: "+b);if(a.h.has(b))return a.h.get(b);if(!a.i.has(b)){if(d)return;throw Error("No provider for: "+b);}d=a.i.get(b);c.push(b);if(void 0!==d.Bd)var e=d.Bd;else if(d.kf)e=d[Cr]?Ir(a,d[Cr],c):[],e=d.kf.apply(d,la(e));else if(d.Ad){e=d.Ad;var f=e[Cr]?Ir(a,e[Cr],c):[];e=new (Function.prototype.bind.apply(e,[null].concat(la(f))))}else throw Error("Could not resolve providers for: "+b);c.pop();d.Wg||a.h.set(b,e);return e}
function Ir(a,b,c){return b?b.map(function(d){return d instanceof Er?Hr(a,d.key,c,!0):Hr(a,d,c)}):[]}
;var Jr;function Kr(){Jr||(Jr=new Fr);return Jr}
;var Lr=window;function Mr(){var a,b;return"h5vcc"in Lr&&(null==(a=Lr.h5vcc.traceEvent)?0:a.traceBegin)&&(null==(b=Lr.h5vcc.traceEvent)?0:b.traceEnd)?1:"performance"in Lr&&Lr.performance.mark&&Lr.performance.measure?2:0}
function Nr(a){var b=Mr();switch(b){case 1:Lr.h5vcc.traceEvent.traceBegin("YTLR",a);break;case 2:Lr.performance.mark(a+"-start");break;case 0:break;default:Xb(b,"unknown trace type")}}
function Or(a){var b=Mr();switch(b){case 1:Lr.h5vcc.traceEvent.traceEnd("YTLR",a);break;case 2:b=a+"-start";var c=a+"-end";Lr.performance.mark(c);Lr.performance.measure(a,b,c);break;case 0:break;default:Xb(b,"unknown trace type")}}
;var Pr=U("web_enable_lifecycle_monitoring")&&0!==Mr(),Qr=U("web_enable_lifecycle_monitoring");function Rr(a){var b=this;var c=void 0===c?0:c;var d=void 0===d?gn():d;this.j=c;this.scheduler=d;this.i=new wi;this.h=a;for(a={bb:0};a.bb<this.h.length;a={kc:void 0,bb:a.bb},a.bb++)a.kc=this.h[a.bb],c=function(e){return function(){e.kc.zc();b.h[e.bb].mc=!0;b.h.every(function(f){return!0===f.mc})&&b.i.resolve()}}(a),d=this.getPriority(a.kc),d=this.scheduler.Za(c,d),this.h[a.bb]=Object.assign({},a.kc,{zc:c,
jobId:d})}
function Sr(a){var b=Array.from(a.h.keys()).sort(function(d,e){return a.getPriority(a.h[e])-a.getPriority(a.h[d])});
b=w(b);for(var c=b.next();!c.done;c=b.next())c=a.h[c.value],void 0===c.jobId||c.mc||(a.scheduler.qa(c.jobId),a.scheduler.Za(c.zc,10))}
Rr.prototype.cancel=function(){for(var a=w(this.h),b=a.next();!b.done;b=a.next())b=b.value,void 0===b.jobId||b.mc||this.scheduler.qa(b.jobId),b.mc=!0;this.i.resolve()};
Rr.prototype.getPriority=function(a){var b;return null!=(b=a.priority)?b:this.j};function Tr(a){this.state=a;this.plugins=[];this.l=void 0;this.v={};Pr&&Nr(this.state)}
m=Tr.prototype;m.install=function(a){this.plugins.push(a);return this};
m.uninstall=function(){var a=this;B.apply(0,arguments).forEach(function(b){b=a.plugins.indexOf(b);-1<b&&a.plugins.splice(b,1)})};
m.transition=function(a,b){var c=this;Pr&&Or(this.state);var d=this.transitions.find(function(f){return Array.isArray(f.from)?f.from.find(function(g){return g===c.state&&f.to===a}):f.from===c.state&&f.to===a});
if(d){this.j&&(Sr(this.j),this.j=void 0);Ur(this,a,b);this.state=a;Pr&&Nr(this.state);d=d.action.bind(this);var e=this.plugins.filter(function(f){return f[a]}).map(function(f){return f[a]});
d(Vr(this,e),b)}else throw Error("no transition specified from "+this.state+" to "+a);};
function Vr(a,b){var c=b.filter(function(e){return 10===Wr(a,e)}),d=b.filter(function(e){return 10!==Wr(a,e)});
return a.v.Vg?function(){var e=B.apply(0,arguments);return A(function(f){if(1==f.h)return f.yield(a.Je.apply(a,[c].concat(la(e))),2);a.wd.apply(a,[d].concat(la(e)));f.h=0})}:function(){var e=B.apply(0,arguments);
a.Ke.apply(a,[c].concat(la(e)));a.wd.apply(a,[d].concat(la(e)))}}
m.Ke=function(a){for(var b=B.apply(1,arguments),c=gn(),d=w(a),e=d.next(),f={};!e.done;f={Kb:void 0},e=d.next())f.Kb=e.value,c.Cb(function(g){return function(){Xr(g.Kb.name);g.Kb.callback.apply(g.Kb,la(b));Yr(g.Kb.name)}}(f))};
m.Je=function(a){var b=B.apply(1,arguments),c,d,e,f,g;return A(function(h){1==h.h&&(c=gn(),d=w(a),e=d.next(),f={});if(3!=h.h){if(e.done)return h.B(0);f.sb=e.value;f.Ub=void 0;g=function(k){return function(){Xr(k.sb.name);var l=k.sb.callback.apply(k.sb,la(b));"function"===typeof(null==l?void 0:l.then)?k.Ub=l.then(function(){Yr(k.sb.name)}):Yr(k.sb.name)}}(f);
c.Cb(g);return f.Ub?h.yield(f.Ub,3):h.B(3)}f={sb:void 0,Ub:void 0};e=d.next();return h.B(2)})};
m.wd=function(a){var b=B.apply(1,arguments),c=this,d=a.map(function(e){return{zc:function(){Xr(e.name);e.callback.apply(e,la(b));Yr(e.name)},
priority:Wr(c,e)}});
d.length&&(this.j=new Rr(d))};
function Wr(a,b){var c,d;return null!=(d=null!=(c=a.l)?c:b.priority)?d:0}
function Xr(a){Pr&&a&&Nr(a)}
function Yr(a){Pr&&a&&Or(a)}
function Ur(a,b,c){Qr&&console.groupCollapsed&&console.groupEnd&&(console.groupCollapsed("["+a.constructor.name+"] '"+a.state+"' to '"+b+"'"),console.log("with message: ",c),console.groupEnd())}
fa.Object.defineProperties(Tr.prototype,{currentState:{configurable:!0,enumerable:!0,get:function(){return this.state}}});function Zr(a){Tr.call(this,void 0===a?"none":a);this.h=null;this.l=10;this.transitions=[{from:"none",to:"application_navigating",action:this.i},{from:"application_navigating",to:"none",action:this.A},{from:"application_navigating",to:"application_navigating",action:function(){}},
{from:"none",to:"none",action:function(){}}]}
var $r;x(Zr,Tr);Zr.prototype.i=function(a,b){var c=this;this.h=Cm(function(){"application_navigating"===c.currentState&&c.transition("none")},5E3);
a(null==b?void 0:b.event)};
Zr.prototype.A=function(a,b){this.h&&(Ni.qa(this.h),this.h=null);a(null==b?void 0:b.event)};
function as(){$r||($r=new Zr);return $r}
;var bs=[];D("yt.logging.transport.getScrapedGelPayloads",function(){return bs});function cs(){this.store={};this.h={}}
cs.prototype.storePayload=function(a,b){a=ds(a);this.store[a]?this.store[a].push(b):(this.h={},this.store[a]=[b]);return a};
cs.prototype.smartExtractMatchingEntries=function(a){if(!a.keys.length)return[];for(var b=es(this,a.keys.splice(0,1)[0]),c=[],d=0;d<b.length;d++)this.store[b[d]]&&a.sizeLimit&&(this.store[b[d]].length<=a.sizeLimit?(c.push.apply(c,la(this.store[b[d]])),delete this.store[b[d]]):c.push.apply(c,la(this.store[b[d]].splice(0,a.sizeLimit))));(null==a?0:a.sizeLimit)&&c.length<(null==a?void 0:a.sizeLimit)&&(a.sizeLimit-=c.length,c.push.apply(c,la(this.smartExtractMatchingEntries(a))));return c};
cs.prototype.extractMatchingEntries=function(a){a=es(this,a);for(var b=[],c=0;c<a.length;c++)this.store[a[c]]&&(b.push.apply(b,la(this.store[a[c]])),delete this.store[a[c]]);return b};
cs.prototype.getSequenceCount=function(a){a=es(this,a);for(var b=0,c=0;c<a.length;c++){var d=void 0;b+=(null==(d=this.store[a[c]])?void 0:d.length)||0}return b};
function es(a,b){var c=ds(b);if(a.h[c])return a.h[c];var d=Object.keys(a.store)||[];if(1>=d.length&&ds(b)===d[0])return d;for(var e=[],f=0;f<d.length;f++){var g=d[f].split("/");if(gs(b.auth,g[0])){var h=b.isJspb;gs(void 0===h?"undefined":h?"true":"false",g[1])&&gs(b.cttAuthInfo,g[2])&&(h=b.tier,h=void 0===h?"undefined":JSON.stringify(h),gs(h,g[3])&&e.push(d[f]))}}return a.h[c]=e}
function gs(a,b){return void 0===a||"undefined"===a?!0:a===b}
cs.prototype.getSequenceCount=cs.prototype.getSequenceCount;cs.prototype.extractMatchingEntries=cs.prototype.extractMatchingEntries;cs.prototype.smartExtractMatchingEntries=cs.prototype.smartExtractMatchingEntries;cs.prototype.storePayload=cs.prototype.storePayload;function ds(a){return[void 0===a.auth?"undefined":a.auth,void 0===a.isJspb?"undefined":a.isJspb,void 0===a.cttAuthInfo?"undefined":a.cttAuthInfo,void 0===a.tier?"undefined":a.tier].join("/")}
;function hs(a,b){if(a)return a[b.name]}
;var is=yl("initial_gel_batch_timeout",2E3),js=yl("gel_queue_timeout_max_ms",6E4),ks=Math.pow(2,16)-1,ls=yl("gel_min_batch_size",5),ms=void 0;function ns(){this.l=this.h=this.i=0;this.j=!1}
var ps=new ns,qs=new ns,rs=new ns,ss=new ns,ts,us=!0,vs=C.ytLoggingTransportTokensToCttTargetIds_||{};D("ytLoggingTransportTokensToCttTargetIds_",vs);var ws={};function xs(){var a=E("yt.logging.ims");a||(a=new cs,D("yt.logging.ims",a));return a}
function ys(a,b){if("log_event"===a.endpoint){zs();var c=As(a),d=Bs(a.payload)||"";a:{if(U("enable_web_tiered_gel")){var e=wq[d||""];var f,g,h,k=null==Kr().resolve(new Er(bp))?void 0:null==(f=cp())?void 0:null==(g=f.loggingHotConfig)?void 0:null==(h=g.eventLoggingConfig)?void 0:h.payloadPolicies;if(k)for(f=0;f<k.length;f++)if(k[f].payloadNumber===e){e=k[f];break a}}e=void 0}k=200;if(e){if(!1===e.enabled&&!U("web_payload_policy_disabled_killswitch"))return;k=Cs(e.tier);if(400===k){Ds(a,b);return}}ws[c]=
!0;e={cttAuthInfo:c,isJspb:!1,tier:k};xs().storePayload(e,a.payload);Es(b,c,e,"gelDebuggingEvent"===d)}}
function Es(a,b,c,d){function e(){Fs({writeThenSend:!0},U("flush_only_full_queue")?b:void 0,f,c.tier)}
var f=!1;f=void 0===f?!1:f;d=void 0===d?!1:d;a&&(ms=new a);a=yl("tvhtml5_logging_max_batch_ads_fork")||yl("web_logging_max_batch")||100;var g=W(),h=Gs(f,c.tier),k=h.l;d&&(h.j=!0);d=0;c&&(d=xs().getSequenceCount(c));1E3<=d?e():d>=a?ts||(ts=Hs(function(){e();ts=void 0},0)):10<=g-k&&(Is(f,c.tier),h.l=g)}
function Ds(a,b){if("log_event"===a.endpoint){zs();var c=As(a),d=new Map;d.set(c,[a.payload]);var e=Bs(a.payload)||"";b&&(ms=new b);return new Ud(function(f,g){ms&&ms.isReady()?Js(d,ms,f,g,{bypassNetworkless:!0},!0,"gelDebuggingEvent"===e):f()})}}
function As(a){var b="";if(a.dangerousLogToVisitorSession)b="visitorOnlyApprovedKey";else if(a.cttAuthInfo){b=a.cttAuthInfo;var c={};b.videoId?c.videoId=b.videoId:b.playlistId&&(c.playlistId=b.playlistId);vs[a.cttAuthInfo.token]=c;b=a.cttAuthInfo.token}return b}
function Fs(a,b,c,d){a=void 0===a?{}:a;c=void 0===c?!1:c;new Ud(function(e,f){var g=Gs(c,d),h=g.j;g.j=!1;Ks(g.i);Ks(g.h);g.h=0;ms&&ms.isReady()?void 0===d&&U("enable_web_tiered_gel")?Ls(e,f,a,b,c,300,h):Ls(e,f,a,b,c,d,h):(Is(c,d),e())})}
function Ls(a,b,c,d,e,f,g){var h=ms;c=void 0===c?{}:c;e=void 0===e?!1:e;f=void 0===f?200:f;g=void 0===g?!1:g;var k=new Map,l={isJspb:e,cttAuthInfo:d,tier:f};e={isJspb:e,cttAuthInfo:d};if(void 0!==d)f=U("enable_web_tiered_gel")?xs().smartExtractMatchingEntries({keys:[l,e],sizeLimit:1E3}):xs().extractMatchingEntries(e),k.set(d,f);else for(d=w(Object.keys(ws)),l=d.next();!l.done;l=d.next())l=l.value,e=U("enable_web_tiered_gel")?xs().smartExtractMatchingEntries({keys:[{isJspb:!1,cttAuthInfo:l,tier:f},
{isJspb:!1,cttAuthInfo:l}],sizeLimit:1E3}):xs().extractMatchingEntries({isJspb:!1,cttAuthInfo:l}),0<e.length&&k.set(l,e),(U("web_fp_via_jspb_and_json")&&c.writeThenSend||!U("web_fp_via_jspb_and_json"))&&delete ws[l];Js(k,h,a,b,c,!1,g)}
function Is(a,b){function c(){Fs({writeThenSend:!0},void 0,a,b)}
a=void 0===a?!1:a;b=void 0===b?200:b;var d=Gs(a,b),e=d===ss||d===rs?5E3:js;U("web_gel_timeout_cap")&&!d.h&&(e=Hs(function(){c()},e),d.h=e);
Ks(d.i);e=T("LOGGING_BATCH_TIMEOUT",yl("web_gel_debounce_ms",1E4));U("shorten_initial_gel_batch_timeout")&&us&&(e=is);e=Hs(function(){0<yl("gel_min_batch_size")?xs().getSequenceCount({cttAuthInfo:void 0,isJspb:a,tier:b})>=ls&&c():c()},e);
d.i=e}
function Js(a,b,c,d,e,f,g){e=void 0===e?{}:e;var h=Math.round(W()),k=a.size,l=(void 0===g?0:g)&&U("vss_through_gel_video_stats")?"video_stats":"log_event";a=w(a);var n=a.next();for(g={};!n.done;g={Fc:void 0,batchRequest:void 0,dangerousLogToVisitorSession:void 0,Ic:void 0,Hc:void 0},n=a.next()){var p=w(n.value);n=p.next().value;p=p.next().value;g.batchRequest=Sb({context:ip(b.config_||hp())});if(!Qa(p)&&!U("throw_err_when_logevent_malformed_killswitch")){d();break}g.batchRequest.events=p;(p=vs[n])&&
Ms(g.batchRequest,n,p);delete vs[n];g.dangerousLogToVisitorSession="visitorOnlyApprovedKey"===n;Ns(g.batchRequest,h,g.dangerousLogToVisitorSession);U("always_send_and_write")&&(e.writeThenSend=!1);g.Ic=function(r){U("start_client_gcf")&&Ni.pa(function(){return A(function(t){return t.yield(Os(r),0)})});
k--;k||c()};
g.Fc=0;g.Hc=function(r){return function(){r.Fc++;if(e.bypassNetworkless&&1===r.Fc)try{aq(b,l,r.batchRequest,Ps({writeThenSend:!0},r.dangerousLogToVisitorSession,r.Ic,r.Hc,f)),us=!1}catch(t){el(t),d()}k--;k||c()}}(g);
try{aq(b,l,g.batchRequest,Ps(e,g.dangerousLogToVisitorSession,g.Ic,g.Hc,f)),us=!1}catch(r){el(r),d()}}}
function Ps(a,b,c,d,e){a={retry:!0,onSuccess:c,onError:d,networklessOptions:a,dangerousLogToVisitorSession:b,Bg:!!e,headers:{},postBodyFormat:"",postBody:"",compress:U("compress_gel")||U("compress_gel_lr")};Qs()&&(a.headers["X-Goog-Request-Time"]=JSON.stringify(Math.round(W())));return a}
function Ns(a,b,c){Qs()||(a.requestTimeMs=String(b));U("unsplit_gel_payloads_in_logs")&&(a.unsplitGelPayloadsInLogs=!0);!c&&(b=T("EVENT_ID"))&&((c=T("BATCH_CLIENT_COUNTER")||0)||(c=Math.floor(Math.random()*ks/2)),c++,c>ks&&(c=1),$k("BATCH_CLIENT_COUNTER",c),a.serializedClientEventId={serializedEventId:b,clientCounter:String(c)})}
function Ms(a,b,c){if(c.videoId)var d="VIDEO";else if(c.playlistId)d="PLAYLIST";else return;a.credentialTransferTokenTargetId=c;a.context=a.context||{};a.context.user=a.context.user||{};a.context.user.credentialTransferTokens=[{token:b,scope:d}]}
function zs(){var a;(a=E("yt.logging.transport.enableScrapingForTest"))||(a=xl("il_payload_scraping"),a="enable_il_payload_scraping"!==(void 0!==a?String(a):""));a||(bs=[],D("yt.logging.transport.enableScrapingForTest",!0),D("yt.logging.transport.scrapedPayloadsForTesting",bs),D("yt.logging.transport.payloadToScrape","visualElementShown visualElementHidden visualElementAttached screenCreated visualElementGestured visualElementStateChanged".split(" ")),D("yt.logging.transport.getScrapedPayloadFromClientEventsFunction"),
D("yt.logging.transport.scrapeClientEvent",!0))}
function Qs(){return U("use_request_time_ms_header")||U("lr_use_request_time_ms_header")}
function Hs(a,b){return!1===U("transport_use_scheduler")?vl(a,b):U("logging_avoid_blocking_during_navigation")||U("lr_logging_avoid_blocking_during_navigation")?Cm(function(){if("none"===as().currentState)a();else{var c={};as().install((c.none={callback:a},c))}},b):Cm(a,b)}
function Ks(a){U("transport_use_scheduler")?Ni.qa(a):window.clearTimeout(a)}
function Os(a){var b,c,d,e,f,g,h,k,l,n;return A(function(p){return 1==p.h?(d=null==(b=a)?void 0:null==(c=b.responseContext)?void 0:c.globalConfigGroup,e=hs(d,Fk),g=null==(f=d)?void 0:f.hotHashData,h=hs(d,Ek),l=null==(k=d)?void 0:k.coldHashData,(n=Kr().resolve(new Er(bp)))?g?e?p.yield(dp(n,g,e),2):p.yield(dp(n,g),2):p.B(2):p.return()):l?h?p.yield(ep(n,l,h),0):p.yield(ep(n,l),0):p.B(0)})}
function Gs(a,b){b=void 0===b?200:b;return a?300===b?ss:qs:300===b?rs:ps}
function Bs(a){a=Object.keys(a);a=w(a);for(var b=a.next();!b.done;b=a.next())if(b=b.value,wq[b])return b}
function Cs(a){switch(a){case "DELAYED_EVENT_TIER_UNSPECIFIED":return 0;case "DELAYED_EVENT_TIER_DEFAULT":return 100;case "DELAYED_EVENT_TIER_DISPATCH_TO_EMPTY":return 200;case "DELAYED_EVENT_TIER_FAST":return 300;case "DELAYED_EVENT_TIER_IMMEDIATE":return 400;default:return 200}}
;var Rs=C.ytLoggingGelSequenceIdObj_||{};D("ytLoggingGelSequenceIdObj_",Rs);
function Ss(a,b,c,d){d=void 0===d?{}:d;var e={},f=Math.round(d.timestamp||W());e.eventTimeMs=f<Number.MAX_SAFE_INTEGER?f:0;e[a]=b;a=or();e.context={lastActivityMs:String(d.timestamp||!isFinite(a)?-1:a)};d.sequenceGroup&&!U("web_gel_sequence_info_killswitch")&&(a=e.context,b=d.sequenceGroup,Rs[b]=b in Rs?Rs[b]+1:0,a.sequence={index:Rs[b],groupKey:b},d.endOfSequence&&delete Rs[d.sequenceGroup]);(d.sendIsolatedPayload?Ds:ys)({endpoint:"log_event",payload:e,cttAuthInfo:d.cttAuthInfo,dangerousLogToVisitorSession:d.dangerousLogToVisitorSession},
c)}
;function rn(a,b,c){c=void 0===c?{}:c;var d=Zq;T("ytLoggingEventsDefaultDisabled",!1)&&Zq===Zq&&(d=null);U("web_all_payloads_via_jspb")&&!c.timestamp&&(c.lact=or(),c.timestamp=W());Ss(a,b,d,c)}
;D("ytLoggingGelSequenceIdObj_",C.ytLoggingGelSequenceIdObj_||{});var Ts=new Set,Us=0,Vs=0,Ws=0,Xs=[],Ys=["PhantomJS","Googlebot","TO STOP THIS SECURITY SCAN go/scan"];function qn(a){Zs(a)}
function $s(a){Zs(a,"WARNING")}
function at(a){a instanceof Error?Zs(a):(a=Ra(a)?JSON.stringify(a):String(a),a=new V(a),a.name="RejectedPromiseError",$s(a))}
function Zs(a,b,c,d,e,f,g,h){f=void 0===f?{}:f;f.name=c||T("INNERTUBE_CONTEXT_CLIENT_NAME",1);f.version=d||T("INNERTUBE_CONTEXT_CLIENT_VERSION");c=f;b=void 0===b?"ERROR":b;g=void 0===g?!1:g;b=void 0===b?"ERROR":b;g=void 0===g?!1:g;if(a&&(a.hasOwnProperty("level")&&a.level&&(b=a.level),U("console_log_js_exceptions")&&(d=[],d.push("Name: "+a.name),d.push("Message: "+a.message),a.hasOwnProperty("params")&&d.push("Error Params: "+JSON.stringify(a.params)),a.hasOwnProperty("args")&&d.push("Error args: "+
JSON.stringify(a.args)),d.push("File name: "+a.fileName),d.push("Stacktrace: "+a.stack),d=d.join("\n"),window.console.log(d,a)),!(5<=Us))){d=Xs;var k=fc(a);e=k.message||"Unknown Error";f=k.name||"UnknownError";var l=k.stack||a.i||"Not available";if(l.startsWith(f+": "+e)){var n=l.split("\n");n.shift();l=n.join("\n")}n=k.lineNumber||"Not available";k=k.fileName||"Not available";var p=0;if(a.hasOwnProperty("args")&&a.args&&a.args.length)for(var r=0;r<a.args.length&&!(p=Zl(a.args[r],"params."+r,c,p),
500<=p);r++);else if(a.hasOwnProperty("params")&&a.params){var t=a.params;if("object"===typeof a.params)for(r in t){if(t[r]){var y="params."+r,v=am(t[r]);c[y]=v;p+=y.length+v.length;if(500<p)break}}else c.params=am(t)}if(d.length)for(r=0;r<d.length&&!(p=Zl(d[r],"params.context."+r,c,p),500<=p);r++);navigator.vendor&&!c.hasOwnProperty("vendor")&&(c["device.vendor"]=navigator.vendor);r={message:e,name:f,lineNumber:n,fileName:k,stack:l,params:c,sampleWeight:1};c=Number(a.columnNumber);isNaN(c)||(r.lineNumber=
r.lineNumber+":"+c);if("IGNORED"===a.level)a=0;else a:{a=Vl();c=w(a.Ua);for(d=c.next();!d.done;d=c.next())if(d=d.value,r.message&&r.message.match(d.Ng)){a=d.weight;break a}a=w(a.Ra);for(c=a.next();!c.done;c=a.next())if(c=c.value,c.callback(r)){a=c.weight;break a}a=1}r.sampleWeight=a;a=w(Ql);for(c=a.next();!c.done;c=a.next())if(c=c.value,c.jc[r.name])for(e=w(c.jc[r.name]),d=e.next();!d.done;d=e.next())if(f=d.value,d=r.message.match(f.regexp)){r.params["params.error.original"]=d[0];e=f.groups;f={};
for(n=0;n<e.length;n++)f[e[n]]=d[n+1],r.params["params.error."+e[n]]=d[n+1];r.message=c.Dc(f);break}r.params||(r.params={});a=Vl();r.params["params.errorServiceSignature"]="msg="+a.Ua.length+"&cb="+a.Ra.length;r.params["params.serviceWorker"]="false";C.document&&C.document.querySelectorAll&&(r.params["params.fscripts"]=String(document.querySelectorAll("script:not([nonce])").length));ib("sample").constructor!==gb&&(r.params["params.fconst"]="true");window.yterr&&"function"===typeof window.yterr&&window.yterr(r);
if(0!==r.sampleWeight&&!Ts.has(r.message)){if(g&&U("web_enable_error_204"))bt(void 0===b?"ERROR":b,r);else{b=void 0===b?"ERROR":b;"ERROR"===b?(Wl.Xa("handleError",r),U("record_app_crashed_web")&&0===Ws&&1===r.sampleWeight&&(Ws++,g={appCrashType:"APP_CRASH_TYPE_BREAKPAD"},U("report_client_error_with_app_crash_ks")||(g.systemHealth={crashData:{clientError:{logMessage:{message:r.message}}}}),rn("appCrashed",g)),Vs++):"WARNING"===b&&Wl.Xa("handleWarning",r);if(U("kevlar_gel_error_routing")){g=b;h=void 0===
h?{}:h;b:{a=w(Ys);for(c=a.next();!c.done;c=a.next())if(xn(c.value.toLowerCase())){a=!0;break b}a=!1}if(a)h=void 0;else{c={stackTrace:r.stack};r.fileName&&(c.filename=r.fileName);a=r.lineNumber&&r.lineNumber.split?r.lineNumber.split(":"):[];0!==a.length&&(1!==a.length||isNaN(Number(a[0]))?2!==a.length||isNaN(Number(a[0]))||isNaN(Number(a[1]))||(c.lineNumber=Number(a[0]),c.columnNumber=Number(a[1])):c.lineNumber=Number(a[0]));a={level:"ERROR_LEVEL_UNKNOWN",message:r.message,errorClassName:r.name,sampleWeight:r.sampleWeight};
"ERROR"===g?a.level="ERROR_LEVEL_ERROR":"WARNING"===g&&(a.level="ERROR_LEVEL_WARNNING");c={isObfuscated:!0,browserStackInfo:c};h.pageUrl=window.location.href;h.kvPairs=[];T("FEXP_EXPERIMENTS")&&(h.experimentIds=T("FEXP_EXPERIMENTS"));d=T("LATEST_ECATCHER_SERVICE_TRACKING_PARAMS");if(!al("web_disable_gel_stp_ecatcher_killswitch")&&d)for(e=w(Object.keys(d)),f=e.next();!f.done;f=e.next())f=f.value,h.kvPairs.push({key:f,value:String(d[f])});if(d=r.params)for(e=w(Object.keys(d)),f=e.next();!f.done;f=e.next())f=
f.value,h.kvPairs.push({key:"client."+f,value:String(d[f])});d=T("SERVER_NAME");e=T("SERVER_VERSION");d&&e&&(h.kvPairs.push({key:"server.name",value:d}),h.kvPairs.push({key:"server.version",value:e}));h={errorMetadata:h,stackTrace:c,logMessage:a}}h&&(rn("clientError",h),("ERROR"===g||U("errors_flush_gel_always_killswitch"))&&Fs(void 0,void 0,!1))}U("suppress_error_204_logging")||bt(b,r)}try{Ts.add(r.message)}catch(z){}Us++}}}
function bt(a,b){var c=b.params||{};a={urlParams:{a:"logerror",t:"jserror",type:b.name,msg:b.message.substr(0,250),line:b.lineNumber,level:a,"client.name":c.name},postParams:{url:T("PAGE_NAME",window.location.href),file:b.fileName},method:"POST"};c.version&&(a["client.version"]=c.version);if(a.postParams){b.stack&&(a.postParams.stack=b.stack);b=w(Object.keys(c));for(var d=b.next();!d.done;d=b.next())d=d.value,a.postParams["client."+d]=c[d];if(c=T("LATEST_ECATCHER_SERVICE_TRACKING_PARAMS"))for(b=w(Object.keys(c)),
d=b.next();!d.done;d=b.next())d=d.value,a.postParams[d]=c[d];c=T("SERVER_NAME");b=T("SERVER_VERSION");c&&b&&(a.postParams["server.name"]=c,a.postParams["server.version"]=b)}Hl(T("ECATCHER_REPORT_HOST","")+"/error_204",a)}
;function ct(){this.register=new Map}
function dt(a){a=w(a.register.values());for(var b=a.next();!b.done;b=a.next())b.value.Qg("ABORTED")}
ct.prototype.clear=function(){dt(this);this.register.clear()};
var et=new ct;var ft=Date.now().toString();
function gt(){a:{if(window.crypto&&window.crypto.getRandomValues)try{var a=Array(16),b=new Uint8Array(16);window.crypto.getRandomValues(b);for(var c=0;c<a.length;c++)a[c]=b[c];var d=a;break a}catch(e){}d=Array(16);for(a=0;16>a;a++){b=Date.now();for(c=0;c<b%23;c++)d[a]=Math.random();d[a]=Math.floor(256*Math.random())}if(ft)for(a=1,b=0;b<ft.length;b++)d[a%16]=d[a%16]^d[(a-1)%16]/4^ft.charCodeAt(b),a++}a=[];for(b=0;b<d.length;b++)a.push("ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789-_".charAt(d[b]&63));
return a.join("")}
;var ht,jt=C.ytLoggingDocDocumentNonce_;jt||(jt=gt(),D("ytLoggingDocDocumentNonce_",jt));ht=jt;function kt(a){this.h=a}
m=kt.prototype;m.getAsJson=function(){var a={};void 0!==this.h.trackingParams?a.trackingParams=this.h.trackingParams:(a.veType=this.h.veType,void 0!==this.h.veCounter&&(a.veCounter=this.h.veCounter),void 0!==this.h.elementIndex&&(a.elementIndex=this.h.elementIndex));void 0!==this.h.dataElement&&(a.dataElement=this.h.dataElement.getAsJson());void 0!==this.h.youtubeData&&(a.youtubeData=this.h.youtubeData);this.h.isCounterfactual&&(a.isCounterfactual=!0);return a};
m.getAsJspb=function(){var a=new Mk;void 0!==this.h.trackingParams?a.setTrackingParams(this.h.trackingParams):(void 0!==this.h.veType&&J(a,2,Kf(this.h.veType)),void 0!==this.h.veCounter&&J(a,6,Kf(this.h.veCounter)),void 0!==this.h.elementIndex&&J(a,3,Kf(this.h.elementIndex)),this.h.isCounterfactual&&J(a,5,Gf(!0)));if(void 0!==this.h.dataElement){var b=this.h.dataElement.getAsJspb();Eg(a,Mk,7,b)}void 0!==this.h.youtubeData&&Eg(a,Gk,8,this.h.jspbYoutubeData);return a};
m.toString=function(){return JSON.stringify(this.getAsJson())};
m.isClientVe=function(){return!this.h.trackingParams&&!!this.h.veType};
m.getLoggingDirectives=function(){return this.h.loggingDirectives};function lt(a){return T("client-screen-nonce-store",{})[void 0===a?0:a]}
function mt(a,b){b=void 0===b?0:b;var c=T("client-screen-nonce-store");c||(c={},$k("client-screen-nonce-store",c));c[b]=a}
function nt(a){a=void 0===a?0:a;return 0===a?"ROOT_VE_TYPE":"ROOT_VE_TYPE."+a}
function ot(a){return T(nt(void 0===a?0:a))}
D("yt_logging_screen.getRootVeType",ot);function pt(){var a=T("csn-to-ctt-auth-info");a||(a={},$k("csn-to-ctt-auth-info",a));return a}
function qt(){return Object.values(T("client-screen-nonce-store",{})).filter(function(a){return void 0!==a})}
function rt(a){a=lt(void 0===a?0:a);if(!a&&!T("USE_CSN_FALLBACK",!0))return null;a||(a="UNDEFINED_CSN");return a?a:null}
D("yt_logging_screen.getCurrentCsn",rt);function st(a,b,c){var d=pt();(c=rt(c))&&delete d[c];b&&(d[a]=b)}
function tt(a){return pt()[a]}
D("yt_logging_screen.getCttAuthInfo",tt);D("yt_logging_screen.setCurrentScreen",function(a,b,c,d){c=void 0===c?0:c;if(a!==lt(c)||b!==T(nt(c)))if(st(a,d,c),mt(a,c),$k(nt(c),b),b=function(){setTimeout(function(){a&&rn("foregroundHeartbeatScreenAssociated",{clientDocumentNonce:ht,clientScreenNonce:a})},0)},"requestAnimationFrame"in window)try{window.requestAnimationFrame(b)}catch(e){b()}else b()});function ut(){var a=Rb(vt),b;return(new Ud(function(c,d){a.onSuccess=function(e){ul(e)?c(new wt(e)):d(new xt("Request failed, status="+(e&&"status"in e?e.status:-1),"net.badstatus",e))};
a.onError=function(e){d(new xt("Unknown request error","net.unknown",e))};
a.onTimeout=function(e){d(new xt("Request timed out","net.timeout",e))};
b=Hl("//googleads.g.doubleclick.net/pagead/id",a)})).pc(function(c){if(c instanceof ae){var d;
null==(d=b)||d.abort()}return Zd(c)})}
function xt(a,b,c){bb.call(this,a+", errorCode="+b);this.errorCode=b;this.xhr=c;this.name="PromiseAjaxError"}
x(xt,bb);function wt(a){this.xhr=a}
;function zt(){this.h=0;this.i=null}
zt.prototype.then=function(a,b,c){return 1===this.h&&a?(a=a.call(c,this.i))&&"function"===typeof a.then?a:At(a):2===this.h&&b?(a=b.call(c,this.i))&&"function"===typeof a.then?a:Bt(a):this};
zt.prototype.getValue=function(){return this.i};
zt.prototype.isRejected=function(){return 2==this.h};
zt.prototype.$goog_Thenable=!0;function Bt(a){var b=new zt;a=void 0===a?null:a;b.h=2;b.i=void 0===a?null:a;return b}
function At(a){var b=new zt;a=void 0===a?null:a;b.h=1;b.i=void 0===a?null:a;return b}
;function Ct(a,b){var c=void 0===c?{}:c;a={method:void 0===b?"POST":b,mode:pl(a)?"same-origin":"cors",credentials:pl(a)?"same-origin":"include"};b={};for(var d=w(Object.keys(c)),e=d.next();!e.done;e=d.next())e=e.value,c[e]&&(b[e]=c[e]);0<Object.keys(b).length&&(a.headers=b);return a}
;function Dt(){return kh()||(De||Ee)&&xn("applewebkit")&&!xn("version")&&(!xn("safari")||xn("gsa/"))||Zc&&xn("version/")?!0:T("EOM_VISITOR_DATA")?!1:!0}
;function Et(a){a:{var b="EMBEDDED_PLAYER_MODE_UNKNOWN";window.location.hostname.includes("youtubeeducation.com")&&(b="EMBEDDED_PLAYER_MODE_PFL");var c=a.raw_embedded_player_response;if(!c&&(a=a.embedded_player_response))try{c=JSON.parse(a)}catch(e){break a}if(c)b:for(var d in Kk)if(Kk[d]==c.embeddedPlayerMode){b=Kk[d];break b}}return"EMBEDDED_PLAYER_MODE_PFL"===b}
;function Ft(a){bb.call(this,a.message||a.description||a.name);this.isMissing=a instanceof Gt;this.isTimeout=a instanceof xt&&"net.timeout"==a.errorCode;this.isCanceled=a instanceof ae}
x(Ft,bb);Ft.prototype.name="BiscottiError";function Gt(){bb.call(this,"Biscotti ID is missing from server")}
x(Gt,bb);Gt.prototype.name="BiscottiMissingError";var vt={format:"RAW",method:"GET",timeout:5E3,withCredentials:!0},Ht=null;function It(){if(U("disable_biscotti_fetch_entirely_for_all_web_clients"))return Error("Biscotti id fetching has been disabled entirely.");if(!Dt())return Error("User has not consented - not fetching biscotti id.");var a=T("PLAYER_VARS",{});if("1"==Pb(a))return Error("Biscotti ID is not available in private embed mode");if(Et(a))return Error("Biscotti id fetching has been disabled for pfl.")}
function Tk(){var a=It();if(void 0!==a)return Zd(a);Ht||(Ht=ut().then(Jt).pc(function(b){return Kt(2,b)}));
return Ht}
function Jt(a){a=a.xhr.responseText;if(0!=a.lastIndexOf(")]}'",0))throw new Gt;a=JSON.parse(a.substr(4));if(1<(a.type||1))throw new Gt;a=a.id;Uk(a);Ht=At(a);Lt(18E5,2);return a}
function Kt(a,b){b=new Ft(b);Uk("");Ht=Bt(b);0<a&&Lt(12E4,a-1);throw b;}
function Lt(a,b){vl(function(){ut().then(Jt,function(c){return Kt(b,c)}).pc(Bd)},a)}
function Mt(){try{var a=E("yt.ads.biscotti.getId_");return a?a():Tk()}catch(b){return Zd(b)}}
;var Bb=ja(["data-"]);function Nt(a){a&&(a.dataset?a.dataset[Ot()]="true":Wb(a))}
function Pt(a){return a?a.dataset?a.dataset[Ot()]:a.getAttribute("data-loaded"):null}
var Qt={};function Ot(){return Qt.loaded||(Qt.loaded="loaded".replace(/\-([a-z])/g,function(a,b){return b.toUpperCase()}))}
;function Rt(a,b,c){F.call(this);var d=this;c=c||T("POST_MESSAGE_ORIGIN")||window.document.location.protocol+"//"+window.document.location.hostname;this.i=b||null;this.targetOrigin="*";this.j=c;this.sessionId=null;this.channel="widget";this.F=!!a;this.v=function(e){a:if(!("*"!=d.j&&e.origin!=d.j||d.i&&e.source!=d.i||"string"!==typeof e.data)){try{var f=JSON.parse(e.data)}catch(g){break a}if(!(null==f||d.F&&(d.sessionId&&d.sessionId!=f.id||d.channel&&d.channel!=f.channel))&&f)switch(f.event){case "listening":"null"!=
e.origin&&(d.j=d.targetOrigin=e.origin);d.i=e.source;d.sessionId=f.id;d.h&&(d.h(),d.h=null);break;case "command":d.l&&(!d.m||0<=Cb(d.m,f.func))&&d.l(f.func,f.args,e.origin)}}};
this.m=this.h=this.l=null;window.addEventListener("message",this.v)}
x(Rt,F);Rt.prototype.sendMessage=function(a,b){if(b=b||this.i){this.sessionId&&(a.id=this.sessionId);this.channel&&(a.channel=this.channel);try{var c=JSON.stringify(a);b.postMessage(c,this.targetOrigin)}catch(d){fl(d)}}};
Rt.prototype.U=function(){window.removeEventListener("message",this.v);F.prototype.U.call(this)};function St(a){a=a||{};var b={},c={};this.url=a.url||"";this.args=a.args||Rb(b);this.assets=a.assets||{};this.attrs=a.attrs||Rb(c);this.fallback=a.fallback||null;this.fallbackMessage=a.fallbackMessage||null;this.html5=!!a.html5;this.disable=a.disable||{};this.loaded=!!a.loaded;this.messages=a.messages||{}}
St.prototype.clone=function(){var a=new St,b;for(b in this)if(this.hasOwnProperty(b)){var c=this[b];"object"==Pa(c)?a[b]=Rb(c):a[b]=c}return a};var Tt=["share/get_web_player_share_panel"],Ut=["feedback"],Vt=["notification/modify_channel_preference"],Wt=["browse/edit_playlist"],Xt=["subscription/subscribe"],Yt=["subscription/unsubscribe"];var Zt=window.yt&&window.yt.msgs_||window.ytcfg&&window.ytcfg.msgs||{};D("yt.msgs_",Zt);function $t(a){Vk(Zt,arguments)}
;function au(a,b,c){bu(a,b,void 0===c?null:c)}
function cu(a){a=du(a);var b=document.getElementById(a);b&&(Ar(a),b.parentNode.removeChild(b))}
function eu(a,b){a&&b&&(a=""+Sa(b),(a=fu[a])&&yr(a))}
function bu(a,b,c){c=void 0===c?null:c;var d=du(a),e=document.getElementById(d),f=e&&Pt(e),g=e&&!f;f?b&&b():(b&&(f=wr(d,b),b=""+Sa(b),fu[b]=f),g||(e=gu(a,d,function(){Pt(e)||(Nt(e),zr(d),vl(function(){Ar(d)},0))},c)))}
function gu(a,b,c,d){d=void 0===d?null:d;var e=Gd("SCRIPT");e.id=b;e.onload=function(){c&&setTimeout(c,0)};
e.onreadystatechange=function(){switch(e.readyState){case "loaded":case "complete":e.onload()}};
d&&e.setAttribute("nonce",d);dc(e,Ck(a));a=document.getElementsByTagName("head")[0]||document.body;a.insertBefore(e,a.firstChild);return e}
function du(a){var b=document.createElement("a");zb(b,a);a=b.href.replace(/^[a-zA-Z]+:\/\//,"//");return"js-"+kc(a)}
var fu={};function hu(a){var b=iu(a),c=document.getElementById(b),d=c&&Pt(c);d||c&&!d||(c=ju(a,b,function(){if(!Pt(c)){Nt(c);zr(b);var e=Ya(Ar,b);vl(e,0)}}))}
function ju(a,b,c){var d=document.createElement("link");d.id=b;d.onload=function(){c&&setTimeout(c,0)};
a=Ck(a);Zb(d,a);(document.getElementsByTagName("head")[0]||document.body).appendChild(d);return d}
function iu(a){var b=Gd("A");zb(b,new sb(a));a=b.href.replace(/^[a-zA-Z]+:\/\//,"//");return"css-"+kc(a)}
;function ku(a){var b=B.apply(1,arguments);if(!lu(a)||b.some(function(d){return!lu(d)}))throw Error("Only objects may be merged.");
b=w(b);for(var c=b.next();!c.done;c=b.next())mu(a,c.value)}
function mu(a,b){for(var c in b)if(lu(b[c])){if(c in a&&!lu(a[c]))throw Error("Cannot merge an object into a non-object.");c in a||(a[c]={});mu(a[c],b[c])}else if(nu(b[c])){if(c in a&&!nu(a[c]))throw Error("Cannot merge an array into a non-array.");c in a||(a[c]=[]);ou(a[c],b[c])}else a[c]=b[c];return a}
function ou(a,b){b=w(b);for(var c=b.next();!c.done;c=b.next())c=c.value,lu(c)?a.push(mu({},c)):nu(c)?a.push(ou([],c)):a.push(c);return a}
function lu(a){return"object"===typeof a&&!Array.isArray(a)}
function nu(a){return"object"===typeof a&&Array.isArray(a)}
;function pu(a){a=void 0===a?!1:a;F.call(this);this.h=new M(a);Ec(this,this.h)}
$a(pu,F);pu.prototype.subscribe=function(a,b,c){return this.V?0:this.h.subscribe(a,b,c)};
pu.prototype.unsubscribe=function(a,b,c){return this.V?!1:this.h.unsubscribe(a,b,c)};
pu.prototype.l=function(a,b){this.V||this.h.Xa.apply(this.h,arguments)};var qu="absolute_experiments app conditional_experiments debugcss debugjs expflag forced_experiments pbj pbjreload sbb spf spfreload sr_bns_address sttick".split(" ");
function ru(a,b){var c=void 0===c?!0:c;var d=T("VALID_SESSION_TEMPDATA_DOMAINS",[]),e=oc(window.location.href);e&&d.push(e);e=oc(a);if(0<=Cb(d,e)||!e&&0==a.lastIndexOf("/",0))if(d=document.createElement("a"),zb(d,a),a=d.href)if(a=pc(a),a=qc(a))if(c&&!b.csn&&(b.itct||b.ved)&&(b=Object.assign({csn:rt()},b)),f){var f=parseInt(f,10);isFinite(f)&&0<f&&su(a,b,f)}else su(a,b)}
function su(a,b,c){a=tu(a);b=b?sc(b):"";c=c||5;Dt()&&hm(a,b,c)}
function tu(a){for(var b=w(qu),c=b.next();!c.done;c=b.next())a=xc(a,c.value);return"ST-"+kc(a).toString(36)}
;function uu(a){mp.call(this,1,arguments);this.csn=a}
x(uu,mp);var vp=new np("screen-created",uu),vu=[],wu=0,xu=new Map,yu=new Map,zu=new Map;
function Au(a,b,c,d,e){e=void 0===e?!1:e;for(var f=Bu({cttAuthInfo:tt(b)||void 0},b),g=w(d),h=g.next();!h.done;h=g.next()){h=h.value;var k=h.getAsJson();(Nb(k)||!k.trackingParams&&!k.veType)&&$s(Error("Child VE logged with no data"));if(U("no_client_ve_attach_unless_shown")){var l=Cu(h,b);if(k.veType&&!yu.has(l)&&!zu.has(l)&&!e){if(!U("il_attach_cache_limit")||1E3>xu.size){xu.set(l,[a,b,c,h]);return}U("il_attach_cache_limit")&&1E3<xu.size&&$s(new V("IL Attach cache exceeded limit"))}h=Cu(c,b);xu.has(h)?
Du(c,b):zu.set(h,!0)}}d=d.filter(function(n){n.csn!==b?(n.csn=b,n=!0):n=!1;return n});
c={csn:b,parentVe:c.getAsJson(),childVes:Fb(d,function(n){return n.getAsJson()})};
"UNDEFINED_CSN"===b?Eu("visualElementAttached",f,c):a?Ss("visualElementAttached",c,a,f):rn("visualElementAttached",c,f)}
function Eu(a,b,c){vu.push({Be:a,payload:c,Jg:void 0,options:b});wu||(wu=wp())}
function xp(a){if(vu){for(var b=w(vu),c=b.next();!c.done;c=b.next())c=c.value,c.payload&&(c.payload.csn=a.csn,rn(c.Be,c.payload,c.options));vu.length=0}wu=0}
function Cu(a,b){return""+a.getAsJson().veType+a.getAsJson().veCounter+b}
function Du(a,b){a=Cu(a,b);xu.has(a)&&(b=xu.get(a)||[],Au(b[0],b[1],b[2],[b[3]],!0),xu.delete(a))}
function Bu(a,b){U("log_sequence_info_on_gel_web")&&(a.sequenceGroup=b);return a}
;function Fu(){try{return!!self.localStorage}catch(a){return!1}}
;function Gu(a){a=a.match(/(.*)::.*::.*/);if(null!==a)return a[1]}
function Hu(a){if(Fu()){var b=Object.keys(window.localStorage);b=w(b);for(var c=b.next();!c.done;c=b.next()){c=c.value;var d=Gu(c);void 0===d||a.includes(d)||self.localStorage.removeItem(c)}}}
function Iu(){if(!Fu())return!1;var a=Am(),b=Object.keys(window.localStorage);b=w(b);for(var c=b.next();!c.done;c=b.next())if(c=Gu(c.value),void 0!==c&&c!==a)return!0;return!1}
;function Ju(){var a=!1;try{a=!!window.sessionStorage.getItem("session_logininfo")}catch(b){a=!0}return("WEB"===T("INNERTUBE_CLIENT_NAME")||"WEB_CREATOR"===T("INNERTUBE_CLIENT_NAME"))&&a}
function Ku(a){if(T("LOGGED_IN",!0)&&Ju()){var b=T("VALID_SESSION_TEMPDATA_DOMAINS",[]);var c=oc(window.location.href);c&&b.push(c);c=oc(a);0<=Cb(b,c)||!c&&0==a.lastIndexOf("/",0)?(b=pc(a),(b=qc(b))?(b=tu(b),b=(b=im(b)||null)?ml(b):{}):b=null):b=null;null==b&&(b={});c=b;var d=void 0;Ju()?(d||(d=T("LOGIN_INFO")),d?(c.session_logininfo=d,c=!0):c=!1):c=!1;c&&ru(a,b)}}
;function Lu(a,b,c){b=void 0===b?{}:b;c=void 0===c?!1:c;var d=T("EVENT_ID");d&&(b.ei||(b.ei=d));b&&ru(a,b);if(c)return!1;Ku(a);var e=void 0===e?{}:e;var f=void 0===f?"":f;var g=void 0===g?window:g;a=tc(a,e);Ku(a);f=a+f;var h=void 0===h?wb:h;a:if(h=void 0===h?wb:h,f instanceof sb)h=f;else{for(a=0;a<h.length;++a)if(b=h[a],b instanceof ub&&b.oe(f)){h=new sb(f);break a}h=void 0}g=g.location;h=yb(h||tb);void 0!==h&&(g.href=h);return!0}
;function Mu(a){if("1"!=Pb(T("PLAYER_VARS",{}))){a&&Sk();try{Mt().then(function(){},function(){}),vl(Mu,18E5)}catch(b){el(b)}}}
;var Nu=new Map([["dark","USER_INTERFACE_THEME_DARK"],["light","USER_INTERFACE_THEME_LIGHT"]]);function Ou(){var a=void 0===a?window.location.href:a;if(U("kevlar_disable_theme_param"))return null;mc(nc(5,a));try{var b=nl(a).theme;return Nu.get(b)||null}catch(c){}return null}
;function Pu(){this.h={};if(this.i=km()){var a=im("CONSISTENCY");a&&Qu(this,{encryptedTokenJarContents:a})}}
Pu.prototype.handleResponse=function(a,b){if(!b)throw Error("request needs to be passed into ConsistencyService");var c,d;b=(null==(c=b.Na.context)?void 0:null==(d=c.request)?void 0:d.consistencyTokenJars)||[];var e;if(a=null==(e=a.responseContext)?void 0:e.consistencyTokenJar){e=w(b);for(c=e.next();!c.done;c=e.next())delete this.h[c.value.encryptedTokenJarContents];Qu(this,a)}};
function Qu(a,b){if(b.encryptedTokenJarContents&&(a.h[b.encryptedTokenJarContents]=b,"string"===typeof b.expirationSeconds)){var c=Number(b.expirationSeconds);setTimeout(function(){delete a.h[b.encryptedTokenJarContents]},1E3*c);
a.i&&hm("CONSISTENCY",b.encryptedTokenJarContents,c,void 0,!0)}}
;var Ru=window.location.hostname.split(".").slice(-2).join(".");function Su(){var a=T("LOCATION_PLAYABILITY_TOKEN");"TVHTML5"===T("INNERTUBE_CLIENT_NAME")&&(this.h=Tu(this))&&(a=this.h.get("yt-location-playability-token"));a&&(this.locationPlayabilityToken=a,this.i=void 0)}
var Uu;function Vu(){Uu=E("yt.clientLocationService.instance");Uu||(Uu=new Su,D("yt.clientLocationService.instance",Uu));return Uu}
m=Su.prototype;m.setLocationOnInnerTubeContext=function(a){a.client||(a.client={});this.i?(a.client.locationInfo||(a.client.locationInfo={}),a.client.locationInfo.latitudeE7=Math.floor(1E7*this.i.coords.latitude),a.client.locationInfo.longitudeE7=Math.floor(1E7*this.i.coords.longitude),a.client.locationInfo.horizontalAccuracyMeters=Math.round(this.i.coords.accuracy),a.client.locationInfo.forceLocationPlayabilityTokenRefresh=!0):this.locationPlayabilityToken&&(a.client.locationPlayabilityToken=this.locationPlayabilityToken)};
m.handleResponse=function(a){var b;a=null==(b=a.responseContext)?void 0:b.locationPlayabilityToken;void 0!==a&&(this.locationPlayabilityToken=a,this.i=void 0,"TVHTML5"===T("INNERTUBE_CLIENT_NAME")?(this.h=Tu(this))&&this.h.set("yt-location-playability-token",a,15552E3):hm("YT_CL",JSON.stringify({loctok:a}),15552E3,Ru,!0))};
function Tu(a){return void 0===a.h?new hn("yt-client-location"):a.h}
m.clearLocationPlayabilityToken=function(a){"TVHTML5"===a?(this.h=Tu(this))&&this.h.remove("yt-location-playability-token"):jm("YT_CL")};
m.getCurrentPositionFromGeolocation=function(){var a=this;if(!(navigator&&navigator.geolocation&&navigator.geolocation.getCurrentPosition))return Promise.reject(Error("Geolocation unsupported"));var b=!1,c=1E4;"MWEB"===T("INNERTUBE_CLIENT_NAME")&&(b=!0,c=15E3);return new Promise(function(d,e){navigator.geolocation.getCurrentPosition(function(f){a.i=f;d(f)},function(f){e(f)},{enableHighAccuracy:b,
maximumAge:0,timeout:c})})};
m.createUnpluggedLocationInfo=function(a){var b={};a=a.coords;if(null==a?0:a.latitude)b.latitudeE7=Math.floor(1E7*a.latitude);if(null==a?0:a.longitude)b.longitudeE7=Math.floor(1E7*a.longitude);if(null==a?0:a.accuracy)b.locationRadiusMeters=Math.round(a.accuracy);return b};
m.createLocationInfo=function(a){var b={};a=a.coords;if(null==a?0:a.latitude)b.latitudeE7=Math.floor(1E7*a.latitude);if(null==a?0:a.longitude)b.longitudeE7=Math.floor(1E7*a.longitude);return b};function Wu(a){var b={"Content-Type":"application/json"};T("EOM_VISITOR_DATA")?b["X-Goog-EOM-Visitor-Id"]=T("EOM_VISITOR_DATA"):T("VISITOR_DATA")&&(b["X-Goog-Visitor-Id"]=T("VISITOR_DATA"));b["X-Youtube-Bootstrap-Logged-In"]=T("LOGGED_IN",!1);T("DEBUG_SETTINGS_METADATA")&&(b["X-Debug-Settings-Metadata"]=T("DEBUG_SETTINGS_METADATA"));"cors"!==a&&((a=T("INNERTUBE_CONTEXT_CLIENT_NAME"))&&(b["X-Youtube-Client-Name"]=a),(a=T("INNERTUBE_CONTEXT_CLIENT_VERSION"))&&(b["X-Youtube-Client-Version"]=a),(a=T("CHROME_CONNECTED_HEADER"))&&
(b["X-Youtube-Chrome-Connected"]=a),(a=T("DOMAIN_ADMIN_STATE"))&&(b["X-Youtube-Domain-Admin-State"]=a));return b}
;function Xu(){this.h={}}
Xu.prototype.contains=function(a){return Object.prototype.hasOwnProperty.call(this.h,a)};
Xu.prototype.get=function(a){if(this.contains(a))return this.h[a]};
Xu.prototype.set=function(a,b){this.h[a]=b};
Xu.prototype.remove=function(a){delete this.h[a]};function Yu(){this.mappings=new Xu}
Yu.prototype.getModuleId=function(a){return a.serviceId.getModuleId()};
Yu.prototype.get=function(a){a:{var b=this.mappings.get(a.toString());switch(b.type){case "mapping":a=b.value;break a;case "factory":b=b.value();this.mappings.set(a.toString(),{type:"mapping",value:b});a=b;break a;default:a=Xb(b)}}return a};
new Yu;function Zu(a){return function(){return new a}}
;var $u={},av=($u.WEB_UNPLUGGED="^unplugged/",$u.WEB_UNPLUGGED_ONBOARDING="^unplugged/",$u.WEB_UNPLUGGED_OPS="^unplugged/",$u.WEB_UNPLUGGED_PUBLIC="^unplugged/",$u.WEB_CREATOR="^creator/",$u.WEB_KIDS="^kids/",$u.WEB_EXPERIMENTS="^experiments/",$u.WEB_MUSIC="^music/",$u.WEB_REMIX="^music/",$u.WEB_MUSIC_EMBEDDED_PLAYER="^music/",$u.WEB_MUSIC_EMBEDDED_PLAYER="^main_app/|^sfv/",$u);
function bv(a){var b=void 0===b?"UNKNOWN_INTERFACE":b;if(1===a.length)return a[0];var c=av[b];if(c){c=new RegExp(c);for(var d=w(a),e=d.next();!e.done;e=d.next())if(e=e.value,c.exec(e))return e}var f=[];Object.entries(av).forEach(function(g){var h=w(g);g=h.next().value;h=h.next().value;b!==g&&f.push(h)});
c=new RegExp(f.join("|"));a.sort(function(g,h){return g.length-h.length});
d=w(a);for(e=d.next();!e.done;e=d.next())if(e=e.value,!c.exec(e))return e;return a[0]}
;function cv(){}
cv.prototype.A=function(a,b,c){b=void 0===b?{}:b;c=void 0===c?gm:c;var d=a.clickTrackingParams,e=this.l,f=!1;f=void 0===f?!1:f;e=void 0===e?!1:e;var g=T("INNERTUBE_CONTEXT");if(g){g=Sb(g);U("web_no_tracking_params_in_shell_killswitch")||delete g.clickTracking;g.client||(g.client={});var h=g.client;"MWEB"===h.clientName&&"AUTOMOTIVE_FORM_FACTOR"!==h.clientFormFactor&&(h.clientFormFactor=T("IS_TABLET")?"LARGE_FORM_FACTOR":"SMALL_FORM_FACTOR");h.screenWidthPoints=window.innerWidth;h.screenHeightPoints=
window.innerHeight;h.screenPixelDensity=Math.round(window.devicePixelRatio||1);h.screenDensityFloat=window.devicePixelRatio||1;h.utcOffsetMinutes=-Math.floor((new Date).getTimezoneOffset());var k=void 0===k?!1:k;om();var l="USER_INTERFACE_THEME_LIGHT";rm(165)?l="USER_INTERFACE_THEME_DARK":rm(174)?l="USER_INTERFACE_THEME_LIGHT":!U("kevlar_legacy_browsers")&&window.matchMedia&&window.matchMedia("(prefers-color-scheme)").matches&&window.matchMedia("(prefers-color-scheme: dark)").matches&&(l="USER_INTERFACE_THEME_DARK");
k=k?l:Ou()||l;h.userInterfaceTheme=k;if(!f){if(k=xm())h.connectionType=k;U("web_log_effective_connection_type")&&(k=ym())&&(g.client.effectiveConnectionType=k)}var n;if(U("web_log_memory_total_kbytes")&&(null==(n=C.navigator)?0:n.deviceMemory)){var p;n=null==(p=C.navigator)?void 0:p.deviceMemory;g.client.memoryTotalKbytes=""+1E6*n}U("web_gcf_hashes_innertube")&&(k=fp())&&(p=k.coldConfigData,n=k.coldHashData,k=k.hotHashData,g.client.configInfo=g.client.configInfo||{},p&&(g.client.configInfo.coldConfigData=
p),n&&(g.client.configInfo.coldHashData=n),k&&(g.client.configInfo.hotHashData=k));p=nl(C.location.href);!U("web_populate_internal_geo_killswitch")&&p.internalcountrycode&&(h.internalGeo=p.internalcountrycode);"MWEB"===h.clientName||"WEB"===h.clientName?(h.mainAppWebInfo={graftUrl:C.location.href},U("kevlar_woffle")&&bm.h&&(p=bm.h,h.mainAppWebInfo.pwaInstallabilityStatus=!p.h&&p.i?"PWA_INSTALLABILITY_STATUS_CAN_BE_INSTALLED":"PWA_INSTALLABILITY_STATUS_UNKNOWN"),h.mainAppWebInfo.webDisplayMode=cm(),
h.mainAppWebInfo.isWebNativeShareAvailable=navigator&&void 0!==navigator.share):"TVHTML5"===h.clientName&&(!U("web_lr_app_quality_killswitch")&&(p=T("LIVING_ROOM_APP_QUALITY"))&&(h.tvAppInfo=Object.assign(h.tvAppInfo||{},{appQuality:p})),p=T("LIVING_ROOM_CERTIFICATION_SCOPE"))&&(h.tvAppInfo=Object.assign(h.tvAppInfo||{},{certificationScope:p}));if(!U("web_populate_time_zone_itc_killswitch")){b:{if("undefined"!==typeof Intl)try{var r=(new Intl.DateTimeFormat).resolvedOptions().timeZone;break b}catch(ea){}r=
void 0}r&&(h.timeZone=r)}(r=T("EXPERIMENTS_TOKEN",""))?h.experimentsToken=r:delete h.experimentsToken;r=zl();Pu.h||(Pu.h=new Pu);h=Pu.h.h;p=[];n=0;for(var t in h)p[n++]=h[t];g.request=Object.assign({},g.request,{internalExperimentFlags:r,consistencyTokenJars:p});!U("web_prequest_context_killswitch")&&(t=T("INNERTUBE_CONTEXT_PREQUEST_CONTEXT"))&&(g.request.externalPrequestContext=t);r=om();t=rm(58);r=r.get("gsml","");g.user=Object.assign({},g.user);t&&(g.user.enableSafetyMode=t);r&&(g.user.lockedSafetyMode=
!0);U("warm_op_csn_cleanup")?e&&(f=rt())&&(g.clientScreenNonce=f):!f&&(f=rt())&&(g.clientScreenNonce=f);d&&(g.clickTracking={clickTrackingParams:d});if(d=E("yt.mdx.remote.remoteClient_"))g.remoteClient=d;Vu().setLocationOnInnerTubeContext(g);try{var y=ql(),v=y.bid;delete y.bid;g.adSignalsInfo={params:[],bid:v};var z=w(Object.entries(y));for(var G=z.next();!G.done;G=z.next()){var K=w(G.value),N=K.next().value,S=K.next().value;y=N;v=S;d=void 0;null==(d=g.adSignalsInfo.params)||d.push({key:y,value:""+
v})}var da;if(U("add_ifa_to_tvh5_requests")&&"TVHTML5"===(null==(da=g.client)?void 0:da.clientName)){var sa=T("INNERTUBE_CONTEXT");sa.adSignalsInfo&&(g.adSignalsInfo.advertisingId=sa.adSignalsInfo.advertisingId,g.adSignalsInfo.advertisingIdSignalType="DEVICE_ID_TYPE_CONNECTED_TV_IFA",g.adSignalsInfo.limitAdTracking=sa.adSignalsInfo.limitAdTracking)}}catch(ea){Zs(ea)}z=g}else Zs(Error("Error: No InnerTubeContext shell provided in ytconfig.")),z={};z={context:z};if(G=this.i(a)){this.h(z,G,b);var P;
b="/youtubei/v1/"+bv(this.j());(G=null==(P=hs(a.commandMetadata,Ik))?void 0:P.apiUrl)&&(b=G);P=b;(b=T("INNERTUBE_HOST_OVERRIDE"))&&(P=String(b)+String(pc(P)));b={};U("web_api_key_killswitch")&&(b.key=T("INNERTUBE_API_KEY"));U("json_condensed_response")&&(b.prettyPrint="false");P=ol(P,b||{},!1);a=Object.assign({},{command:a},void 0);a={input:P,hb:Ct(P),Na:z,config:a};a.config.Vb?a.config.Vb.identity=c:a.config.Vb={identity:c};return a}Zs(new V("Error: Failed to create Request from Command.",a))};
fa.Object.defineProperties(cv.prototype,{l:{configurable:!0,enumerable:!0,get:function(){return!1}}});
function dv(){}
x(dv,cv);function ev(){}
x(ev,dv);ev.prototype.A=function(){return{input:"/getDatasyncIdsEndpoint",hb:Ct("/getDatasyncIdsEndpoint","GET"),Na:{}}};
ev.prototype.j=function(){return[]};
ev.prototype.i=function(){};
ev.prototype.h=function(){};var fv={},gv=(fv.GET_DATASYNC_IDS=Zu(ev),fv);function hv(a){var b;(b=E("ytcsi."+(a||"")+"data_"))||(b={tick:{},info:{}},D("ytcsi."+(a||"")+"data_",b));return b}
function iv(){var a=hv();a.info||(a.info={});return a.info}
function jv(a){a=hv(a);a.metadata||(a.metadata={});return a.metadata}
function kv(a){a=hv(a);a.tick||(a.tick={});return a.tick}
function lv(a){a=hv(a);if(a.gel){var b=a.gel;b.gelInfos||(b.gelInfos={});b.gelTicks||(b.gelTicks={})}else a.gel={gelTicks:{},gelInfos:{}};return a.gel}
function mv(a){a=lv(a);a.gelInfos||(a.gelInfos={});return a.gelInfos}
function nv(a){var b=hv(a).nonce;b||(b=gt(),hv(a).nonce=b);return b}
;function ov(){var a=E("ytcsi.debug");a||(a=[],D("ytcsi.debug",a),D("ytcsi.reference",{}));return a}
function pv(a){a=a||"";var b=E("ytcsi.reference");b||(ov(),b=E("ytcsi.reference"));if(b[a])return b[a];var c=ov(),d={timerName:a,info:{},tick:{},span:{},jspbInfo:[]};c.push(d);return b[a]=d}
;var X={},qv=(X.auto_search="LATENCY_ACTION_AUTO_SEARCH",X.ad_to_ad="LATENCY_ACTION_AD_TO_AD",X.ad_to_video="LATENCY_ACTION_AD_TO_VIDEO",X["analytics.explore"]="LATENCY_ACTION_CREATOR_ANALYTICS_EXPLORE",X.app_startup="LATENCY_ACTION_APP_STARTUP",X["artist.analytics"]="LATENCY_ACTION_CREATOR_ARTIST_ANALYTICS",X["artist.events"]="LATENCY_ACTION_CREATOR_ARTIST_CONCERTS",X["artist.presskit"]="LATENCY_ACTION_CREATOR_ARTIST_PROFILE",X["asset.claimed_videos"]="LATENCY_ACTION_CREATOR_CMS_ASSET_CLAIMED_VIDEOS",
X["asset.composition"]="LATENCY_ACTION_CREATOR_CMS_ASSET_COMPOSITION",X["asset.composition_ownership"]="LATENCY_ACTION_CREATOR_CMS_ASSET_COMPOSITION_OWNERSHIP",X["asset.composition_policy"]="LATENCY_ACTION_CREATOR_CMS_ASSET_COMPOSITION_POLICY",X["asset.embeds"]="LATENCY_ACTION_CREATOR_CMS_ASSET_EMBEDS",X["asset.history"]="LATENCY_ACTION_CREATOR_CMS_ASSET_HISTORY",X["asset.issues"]="LATENCY_ACTION_CREATOR_CMS_ASSET_ISSUES",X["asset.licenses"]="LATENCY_ACTION_CREATOR_CMS_ASSET_LICENSES",X["asset.metadata"]=
"LATENCY_ACTION_CREATOR_CMS_ASSET_METADATA",X["asset.ownership"]="LATENCY_ACTION_CREATOR_CMS_ASSET_OWNERSHIP",X["asset.policy"]="LATENCY_ACTION_CREATOR_CMS_ASSET_POLICY",X["asset.references"]="LATENCY_ACTION_CREATOR_CMS_ASSET_REFERENCES",X["asset.shares"]="LATENCY_ACTION_CREATOR_CMS_ASSET_SHARES",X["asset.sound_recordings"]="LATENCY_ACTION_CREATOR_CMS_ASSET_SOUND_RECORDINGS",X["asset_group.assets"]="LATENCY_ACTION_CREATOR_CMS_ASSET_GROUP_ASSETS",X["asset_group.campaigns"]="LATENCY_ACTION_CREATOR_CMS_ASSET_GROUP_CAMPAIGNS",
X["asset_group.claimed_videos"]="LATENCY_ACTION_CREATOR_CMS_ASSET_GROUP_CLAIMED_VIDEOS",X["asset_group.metadata"]="LATENCY_ACTION_CREATOR_CMS_ASSET_GROUP_METADATA",X["song.analytics"]="LATENCY_ACTION_CREATOR_SONG_ANALYTICS",X.browse="LATENCY_ACTION_BROWSE",X.cast_splash="LATENCY_ACTION_CAST_SPLASH",X.channels="LATENCY_ACTION_CHANNELS",X.creator_channel_dashboard="LATENCY_ACTION_CREATOR_CHANNEL_DASHBOARD",X["channel.analytics"]="LATENCY_ACTION_CREATOR_CHANNEL_ANALYTICS",X["channel.comments"]="LATENCY_ACTION_CREATOR_CHANNEL_COMMENTS",
X["channel.content"]="LATENCY_ACTION_CREATOR_POST_LIST",X["channel.content.promotions"]="LATENCY_ACTION_CREATOR_PROMOTION_LIST",X["channel.copyright"]="LATENCY_ACTION_CREATOR_CHANNEL_COPYRIGHT",X["channel.editing"]="LATENCY_ACTION_CREATOR_CHANNEL_EDITING",X["channel.monetization"]="LATENCY_ACTION_CREATOR_CHANNEL_MONETIZATION",X["channel.music"]="LATENCY_ACTION_CREATOR_CHANNEL_MUSIC",X["channel.music_storefront"]="LATENCY_ACTION_CREATOR_CHANNEL_MUSIC_STOREFRONT",X["channel.playlists"]="LATENCY_ACTION_CREATOR_CHANNEL_PLAYLISTS",
X["channel.translations"]="LATENCY_ACTION_CREATOR_CHANNEL_TRANSLATIONS",X["channel.videos"]="LATENCY_ACTION_CREATOR_CHANNEL_VIDEOS",X["channel.live_streaming"]="LATENCY_ACTION_CREATOR_LIVE_STREAMING",X.chips="LATENCY_ACTION_CHIPS",X.commerce_transaction="LATENCY_ACTION_COMMERCE_TRANSACTION",X["dialog.copyright_strikes"]="LATENCY_ACTION_CREATOR_DIALOG_COPYRIGHT_STRIKES",X["dialog.video_copyright"]="LATENCY_ACTION_CREATOR_DIALOG_VIDEO_COPYRIGHT",X["dialog.uploads"]="LATENCY_ACTION_CREATOR_DIALOG_UPLOADS",
X.direct_playback="LATENCY_ACTION_DIRECT_PLAYBACK",X.embed="LATENCY_ACTION_EMBED",X.entity_key_serialization_perf="LATENCY_ACTION_ENTITY_KEY_SERIALIZATION_PERF",X.entity_key_deserialization_perf="LATENCY_ACTION_ENTITY_KEY_DESERIALIZATION_PERF",X.explore="LATENCY_ACTION_EXPLORE",X.favorites="LATENCY_ACTION_FAVORITES",X.home="LATENCY_ACTION_HOME",X.inboarding="LATENCY_ACTION_INBOARDING",X.library="LATENCY_ACTION_LIBRARY",X.live="LATENCY_ACTION_LIVE",X.live_pagination="LATENCY_ACTION_LIVE_PAGINATION",
X.mini_app="LATENCY_ACTION_MINI_APP_PLAY",X.onboarding="LATENCY_ACTION_ONBOARDING",X.owner="LATENCY_ACTION_CREATOR_CMS_DASHBOARD",X["owner.allowlist"]="LATENCY_ACTION_CREATOR_CMS_ALLOWLIST",X["owner.analytics"]="LATENCY_ACTION_CREATOR_CMS_ANALYTICS",X["owner.art_tracks"]="LATENCY_ACTION_CREATOR_CMS_ART_TRACKS",X["owner.assets"]="LATENCY_ACTION_CREATOR_CMS_ASSETS",X["owner.asset_groups"]="LATENCY_ACTION_CREATOR_CMS_ASSET_GROUPS",X["owner.bulk"]="LATENCY_ACTION_CREATOR_CMS_BULK_HISTORY",X["owner.campaigns"]=
"LATENCY_ACTION_CREATOR_CMS_CAMPAIGNS",X["owner.channel_invites"]="LATENCY_ACTION_CREATOR_CMS_CHANNEL_INVITES",X["owner.channels"]="LATENCY_ACTION_CREATOR_CMS_CHANNELS",X["owner.claimed_videos"]="LATENCY_ACTION_CREATOR_CMS_CLAIMED_VIDEOS",X["owner.claims"]="LATENCY_ACTION_CREATOR_CMS_MANUAL_CLAIMING",X["owner.claims.manual"]="LATENCY_ACTION_CREATOR_CMS_MANUAL_CLAIMING",X["owner.delivery"]="LATENCY_ACTION_CREATOR_CMS_CONTENT_DELIVERY",X["owner.delivery_templates"]="LATENCY_ACTION_CREATOR_CMS_DELIVERY_TEMPLATES",
X["owner.issues"]="LATENCY_ACTION_CREATOR_CMS_ISSUES",X["owner.licenses"]="LATENCY_ACTION_CREATOR_CMS_LICENSES",X["owner.pitch_music"]="LATENCY_ACTION_CREATOR_CMS_PITCH_MUSIC",X["owner.policies"]="LATENCY_ACTION_CREATOR_CMS_POLICIES",X["owner.releases"]="LATENCY_ACTION_CREATOR_CMS_RELEASES",X["owner.reports"]="LATENCY_ACTION_CREATOR_CMS_REPORTS",X["owner.videos"]="LATENCY_ACTION_CREATOR_CMS_VIDEOS",X.parent_profile_settings="LATENCY_ACTION_KIDS_PARENT_PROFILE_SETTINGS",X.parent_tools_collection="LATENCY_ACTION_PARENT_TOOLS_COLLECTION",
X.parent_tools_dashboard="LATENCY_ACTION_PARENT_TOOLS_DASHBOARD",X.player_att="LATENCY_ACTION_PLAYER_ATTESTATION",X["playlist.videos"]="LATENCY_ACTION_CREATOR_PLAYLIST_VIDEO_LIST",X["post.comments"]="LATENCY_ACTION_CREATOR_POST_COMMENTS",X["post.edit"]="LATENCY_ACTION_CREATOR_POST_EDIT",X.prebuffer="LATENCY_ACTION_PREBUFFER",X.prefetch="LATENCY_ACTION_PREFETCH",X.profile_settings="LATENCY_ACTION_KIDS_PROFILE_SETTINGS",X.profile_switcher="LATENCY_ACTION_LOGIN",X.reel_watch="LATENCY_ACTION_REEL_WATCH",
X.results="LATENCY_ACTION_RESULTS",X["promotion.edit"]="LATENCY_ACTION_CREATOR_PROMOTION_EDIT",X.red="LATENCY_ACTION_PREMIUM_PAGE_GET_BROWSE",X.premium="LATENCY_ACTION_PREMIUM_PAGE_GET_BROWSE",X.search_overview_answer="LATENCY_ACTION_SEARCH_OVERVIEW_ANSWER",X.search_ui="LATENCY_ACTION_SEARCH_UI",X.search_suggest="LATENCY_ACTION_SUGGEST",X.search_zero_state="LATENCY_ACTION_SEARCH_ZERO_STATE",X.secret_code="LATENCY_ACTION_KIDS_SECRET_CODE",X.seek="LATENCY_ACTION_PLAYER_SEEK",X.settings="LATENCY_ACTION_SETTINGS",
X.store="LATENCY_ACTION_STORE",X.tenx="LATENCY_ACTION_TENX",X.video_to_ad="LATENCY_ACTION_VIDEO_TO_AD",X.watch="LATENCY_ACTION_WATCH",X.watch_it_again="LATENCY_ACTION_KIDS_WATCH_IT_AGAIN",X["watch,watch7"]="LATENCY_ACTION_WATCH",X["watch,watch7_html5"]="LATENCY_ACTION_WATCH",X["watch,watch7ad"]="LATENCY_ACTION_WATCH",X["watch,watch7ad_html5"]="LATENCY_ACTION_WATCH",X.wn_comments="LATENCY_ACTION_LOAD_COMMENTS",X.ww_rqs="LATENCY_ACTION_WHO_IS_WATCHING",X["video.analytics"]="LATENCY_ACTION_CREATOR_VIDEO_ANALYTICS",
X["video.claims"]="LATENCY_ACTION_CREATOR_VIDEO_CLAIMS",X["video.comments"]="LATENCY_ACTION_CREATOR_VIDEO_COMMENTS",X["video.copyright"]="LATENCY_ACTION_CREATOR_VIDEO_COPYRIGHT",X["video.edit"]="LATENCY_ACTION_CREATOR_VIDEO_EDIT",X["video.editor"]="LATENCY_ACTION_CREATOR_VIDEO_VIDEO_EDITOR",X["video.editor_async"]="LATENCY_ACTION_CREATOR_VIDEO_VIDEO_EDITOR_ASYNC",X["video.live_settings"]="LATENCY_ACTION_CREATOR_VIDEO_LIVE_SETTINGS",X["video.live_streaming"]="LATENCY_ACTION_CREATOR_VIDEO_LIVE_STREAMING",
X["video.monetization"]="LATENCY_ACTION_CREATOR_VIDEO_MONETIZATION",X["video.policy"]="LATENCY_ACTION_CREATOR_VIDEO_POLICY",X["video.rights_management"]="LATENCY_ACTION_CREATOR_VIDEO_RIGHTS_MANAGEMENT",X["video.translations"]="LATENCY_ACTION_CREATOR_VIDEO_TRANSLATIONS",X.voice_assistant="LATENCY_ACTION_VOICE_ASSISTANT",X.cast_load_by_entity_to_watch="LATENCY_ACTION_CAST_LOAD_BY_ENTITY_TO_WATCH",X.networkless_performance="LATENCY_ACTION_NETWORKLESS_PERFORMANCE",X.gel_compression="LATENCY_ACTION_GEL_COMPRESSION",
X.gel_jspb_serialize="LATENCY_ACTION_GEL_JSPB_SERIALIZE",X);function rv(a,b){mp.call(this,1,arguments);this.timer=b}
x(rv,mp);var sv=new np("aft-recorded",rv);var tv=C.ytLoggingLatencyUsageStats_||{};D("ytLoggingLatencyUsageStats_",tv);function uv(){this.h=0}
function vv(){uv.h||(uv.h=new uv);return uv.h}
uv.prototype.tick=function(a,b,c,d){wv(this,"tick_"+a+"_"+b)||rn("latencyActionTicked",{tickName:a,clientActionNonce:b},{timestamp:c,cttAuthInfo:d})};
uv.prototype.info=function(a,b,c){var d=Object.keys(a).join("");wv(this,"info_"+d+"_"+b)||(a=Object.assign({},a),a.clientActionNonce=b,rn("latencyActionInfo",a,{cttAuthInfo:c}))};
uv.prototype.jspbInfo=function(){};
uv.prototype.span=function(a,b,c){var d=Object.keys(a).join("");wv(this,"span_"+d+"_"+b)||(a.clientActionNonce=b,rn("latencyActionSpan",a,{cttAuthInfo:c}))};
function wv(a,b){tv[b]=tv[b]||{count:0};var c=tv[b];c.count++;c.time=W();a.h||(a.h=Cm(function(){var d=W(),e;for(e in tv)tv[e]&&6E4<d-tv[e].time&&delete tv[e];a&&(a.h=0)},5E3));
return 5<c.count?(6===c.count&&1>1E5*Math.random()&&(c=new V("CSI data exceeded logging limit with key",b.split("_")),0<=b.indexOf("plev")||$s(c)),!0):!1}
;var xv=window;function yv(){this.timing={};this.clearResourceTimings=function(){};
this.webkitClearResourceTimings=function(){};
this.mozClearResourceTimings=function(){};
this.msClearResourceTimings=function(){};
this.oClearResourceTimings=function(){}}
function zv(){var a;if(U("csi_use_performance_navigation_timing")||U("csi_use_performance_navigation_timing_tvhtml5")){var b,c,d,e=null==Y?void 0:null==(a=Y.getEntriesByType)?void 0:null==(b=a.call(Y,"navigation"))?void 0:null==(c=b[0])?void 0:null==(d=c.toJSON)?void 0:d.call(c);e?(e.requestStart=Av(e.requestStart),e.responseEnd=Av(e.responseEnd),e.redirectStart=Av(e.redirectStart),e.redirectEnd=Av(e.redirectEnd),e.domainLookupEnd=Av(e.domainLookupEnd),e.connectStart=Av(e.connectStart),e.connectEnd=
Av(e.connectEnd),e.responseStart=Av(e.responseStart),e.secureConnectionStart=Av(e.secureConnectionStart),e.domainLookupStart=Av(e.domainLookupStart),e.isPerformanceNavigationTiming=!0,a=e):a=Y.timing}else a=U("csi_performance_timing_to_object")?JSON.parse(JSON.stringify(Y.timing)):Y.timing;return a}
function Av(a){return Math.round(Bv()+a)}
function Bv(){return(U("csi_use_time_origin")||U("csi_use_time_origin_tvhtml5"))&&Y.timeOrigin?Math.floor(Y.timeOrigin):Y.timing.navigationStart}
var Y=xv.performance||xv.mozPerformance||xv.msPerformance||xv.webkitPerformance||new yv;var Cv=!1,Dv=!1,Ev={'script[name="scheduler/scheduler"]':"sj",'script[name="player/base"]':"pj",'link[rel="preload"][name="player/embed"]':"pej",'link[rel="stylesheet"][name="www-player"]':"pc",'link[rel="stylesheet"][name="player/www-player"]':"pc",'script[name="desktop_polymer/desktop_polymer"]':"dpj",'link[rel="import"][name="desktop_polymer"]':"dph",'script[name="mobile-c3"]':"mcj",'link[rel="stylesheet"][name="mobile-c3"]':"mcc",'script[name="player-plasma-ias-phone/base"]':"mcppj",'script[name="player-plasma-ias-tablet/base"]':"mcptj",
'link[rel="stylesheet"][name="mobile-polymer-player-ias"]':"mcpc",'link[rel="stylesheet"][name="mobile-polymer-player-svg-ias"]':"mcpsc",'script[name="mobile_blazer_core_mod"]':"mbcj",'link[rel="stylesheet"][name="mobile_blazer_css"]':"mbc",'script[name="mobile_blazer_logged_in_users_mod"]':"mbliuj",'script[name="mobile_blazer_logged_out_users_mod"]':"mblouj",'script[name="mobile_blazer_noncore_mod"]':"mbnj","#player_css":"mbpc",'script[name="mobile_blazer_desktopplayer_mod"]':"mbpj",'link[rel="stylesheet"][name="mobile_blazer_tablet_css"]':"mbtc",
'script[name="mobile_blazer_watch_mod"]':"mbwj"};Xa(Y.clearResourceTimings||Y.webkitClearResourceTimings||Y.mozClearResourceTimings||Y.msClearResourceTimings||Y.oClearResourceTimings||Bd,Y);function Fv(a,b){if(!U("web_csi_action_sampling_enabled")||!hv(b).actionDisabled){var c=pv(b||"");ku(c.info,a);a.loadType&&(c=a.loadType,jv(b).loadType=c);ku(mv(b),a);c=nv(b);b=hv(b).cttAuthInfo;vv().info(a,c,b)}}
function Gv(){var a,b,c,d;return(null!=(d=null==Kr().resolve(new Er(bp))?void 0:null==(a=cp())?void 0:null==(b=a.loggingHotConfig)?void 0:null==(c=b.csiConfig)?void 0:c.debugTicks)?d:[]).map(function(e){return Object.values(e)[0]})}
function Z(a,b,c){if(!U("web_csi_action_sampling_enabled")||!hv(c).actionDisabled){var d=nv(c),e;if(e=U("web_csi_debug_sample_enabled")&&d){(null==Kr().resolve(new Er(bp))?0:cp())&&!Dv&&(Dv=!0,Z("gcfl",W(),c));var f,g,h;e=(null==Kr().resolve(new Er(bp))?void 0:null==(f=cp())?void 0:null==(g=f.loggingHotConfig)?void 0:null==(h=g.csiConfig)?void 0:h.debugSampleWeight)||0;if(f=0!==e)b:{f=Gv();if(0<f.length)for(g=0;g<f.length;g++)if(a===f[g]){f=!0;break b}f=!1}if(f){for(g=f=0;g<d.length;g++)f=31*f+d.charCodeAt(g),
g<d.length-1&&(f%=Math.pow(2,47));e=0!==f%1E5%e;hv(c).debugTicksExcludedLogged||(f={},f.debugTicksExcluded=e,Fv(f,c));hv(c).debugTicksExcludedLogged=!0}else e=!1}if(!e){if("_"!==a[0]&&(e=a,f=b,Y.mark))if(e.startsWith("mark_")||(e="mark_"+e),c&&(e+=" ("+c+")"),void 0===f||U("web_csi_disable_alt_time_performance_mark"))Y.mark(e);else{f-=Y.timeOrigin||Y.timing.navigationStart;try{Y.mark(e,{startTime:f})}catch(k){}}e=pv(c||"");e.tick[a]=b||W();if(e.callback&&e.callback[a])for(e=w(e.callback[a]),f=e.next();!f.done;f=
e.next())f=f.value,f();e=lv(c);e.gelTicks&&(e.gelTicks[a]=!0);f=kv(c);e=b||W();U("log_repeated_ytcsi_ticks")?a in f||(f[a]=e):f[a]=e;f=hv(c).cttAuthInfo;"_start"===a?(a=vv(),wv(a,"baseline_"+d)||rn("latencyActionBaselined",{clientActionNonce:d},{timestamp:b,cttAuthInfo:f})):vv().tick(a,d,b,f);Hv(c);return e}}}
function Iv(){var a=document;if("visibilityState"in a)a=a.visibilityState;else{var b=ar+"VisibilityState";a=b in a?a[b]:void 0}switch(a){case "hidden":return 0;case "visible":return 1;case "prerender":return 2;case "unloaded":return 3;default:return-1}}
function Jv(){function a(f,g,h){g=g.match("_rid")?g.split("_rid")[0]:g;"number"===typeof h&&(h=JSON.stringify(h));f.requestIds?f.requestIds.push({endpoint:g,id:h}):f.requestIds=[{endpoint:g,id:h}]}
for(var b={},c=w(Object.entries(T("TIMING_INFO",{}))),d=c.next();!d.done;d=c.next()){var e=w(d.value);d=e.next().value;e=e.next().value;switch(d){case "GetBrowse_rid":a(b,d,e);break;case "GetGuide_rid":a(b,d,e);break;case "GetHome_rid":a(b,d,e);break;case "GetPlayer_rid":a(b,d,e);break;case "GetSearch_rid":a(b,d,e);break;case "GetSettings_rid":a(b,d,e);break;case "GetTrending_rid":a(b,d,e);break;case "GetWatchNext_rid":a(b,d,e);break;case "yt_red":b.isRedSubscriber=!!e;break;case "yt_ad":b.isMonetized=
!!e}}return b}
function Kv(a,b){a=document.querySelector(a);if(!a)return!1;var c="",d=a.nodeName;"SCRIPT"===d?(c=a.src,c||(c=a.getAttribute("data-timing-href"))&&(c=window.location.protocol+c)):"LINK"===d&&(c=a.href);$b(window)&&a.setAttribute("nonce",$b(window));return c?(a=Y.getEntriesByName(c))&&a[0]&&(a=a[0],c=Bv(),Z("rsf_"+b,c+Math.round(a.fetchStart)),Z("rse_"+b,c+Math.round(a.responseEnd)),void 0!==a.transferSize&&0===a.transferSize)?!0:!1:!1}
function Lv(){var a=window.location.protocol,b=Y.getEntriesByType("resource");b=Eb(b,function(c){return 0===c.name.indexOf(a+"//fonts.gstatic.com/s/")});
(b=Gb(b,function(c,d){return d.duration>c.duration?d:c},{duration:0}))&&0<b.startTime&&0<b.responseEnd&&(Z("wffs",Av(b.startTime)),Z("wffe",Av(b.responseEnd)))}
function Mv(a){var b=Nv("aft",a);if(b)return b;b=T((a||"")+"TIMING_AFT_KEYS",["ol"]);for(var c=b.length,d=0;d<c;d++){var e=Nv(b[d],a);if(e)return e}return NaN}
function Nv(a,b){if(a=kv(b)[a])return"number"===typeof a?a:a[a.length-1]}
function Hv(a){var b=Nv("_start",a),c=Mv(a);b&&c&&!Cv&&(sp(sv,new rv(Math.round(c-b),a)),Cv=!0)}
function Ov(){if(Y.getEntriesByType){var a=Y.getEntriesByType("paint");if(a=Hb(a,function(b){return"first-paint"===b.name}))return Av(a.startTime)}a=Y.timing;
return a.xe?Math.max(0,a.xe):0}
;function Pv(a,b){dl(function(){pv("").info.actionType=a;b&&$k("TIMING_AFT_KEYS",b);$k("TIMING_ACTION",a);var c=Jv();0<Object.keys(c).length&&Fv(c);c={isNavigation:!0,actionType:qv[T("TIMING_ACTION")]||"LATENCY_ACTION_UNKNOWN"};var d=T("PREVIOUS_ACTION");d&&(c.previousAction=qv[d]||"LATENCY_ACTION_UNKNOWN");if(d=T("CLIENT_PROTOCOL"))c.httpProtocol=d;if(d=T("CLIENT_TRANSPORT"))c.transportProtocol=d;(d=rt())&&"UNDEFINED_CSN"!==d&&(c.clientScreenNonce=d);d=Iv();if(1===d||-1===d)c.isVisible=!0;jv();iv();
c.loadType="cold";d=iv();var e=zv(),f=Bv(),g=T("CSI_START_TIMESTAMP_MILLIS",0);0<g&&!U("embeds_web_enable_csi_start_override_killswitch")&&(f=g);f&&(Z("srt",e.responseStart),1!==d.prerender&&Z("_start",f,void 0));d=Ov();0<d&&Z("fpt",d);d=zv();d.isPerformanceNavigationTiming&&Fv({performanceNavigationTiming:!0},void 0);Z("nreqs",d.requestStart,void 0);Z("nress",d.responseStart,void 0);Z("nrese",d.responseEnd,void 0);0<d.redirectEnd-d.redirectStart&&(Z("nrs",d.redirectStart,void 0),Z("nre",d.redirectEnd,
void 0));0<d.domainLookupEnd-d.domainLookupStart&&(Z("ndnss",d.domainLookupStart,void 0),Z("ndnse",d.domainLookupEnd,void 0));0<d.connectEnd-d.connectStart&&(Z("ntcps",d.connectStart,void 0),Z("ntcpe",d.connectEnd,void 0));d.secureConnectionStart>=Bv()&&0<d.connectEnd-d.secureConnectionStart&&(Z("nstcps",d.secureConnectionStart,void 0),Z("ntcpe",d.connectEnd,void 0));Y&&"getEntriesByType"in Y&&Lv();d=[];if(document.querySelector&&Y&&Y.getEntriesByName)for(var h in Ev)Ev.hasOwnProperty(h)&&(e=Ev[h],
Kv(h,e)&&d.push(e));if(0<d.length)for(c.resourceInfo=[],h=w(d),d=h.next();!d.done;d=h.next())c.resourceInfo.push({resourceCache:d.value});Fv(c);c=lv();c.preLoggedGelInfos||(c.preLoggedGelInfos=[]);h=c.preLoggedGelInfos;c=mv();d=void 0;for(e=0;e<h.length;e++)if(f=h[e],f.loadType){d=f.loadType;break}if("cold"===jv().loadType&&("cold"===c.loadType||"cold"===d)){d=kv();e=lv();e=e.gelTicks?e.gelTicks:e.gelTicks={};for(var k in d)if(!(k in e))if("number"===typeof d[k])Z(k,Nv(k));else if(U("log_repeated_ytcsi_ticks"))for(f=
w(d[k]),g=f.next();!g.done;g=f.next())g=g.value,Z(k.slice(1),g);k={};d=!1;h=w(h);for(e=h.next();!e.done;e=h.next())d=e.value,ku(c,d),ku(k,d),d=!0;d&&Fv(k)}D("ytglobal.timingready_",!0);k=T("TIMING_ACTION");E("ytglobal.timingready_")&&k&&Qv()&&Mv()&&Hv()})()}
function Qv(){return dl(function(){return Rv()})()}
function Sv(a,b,c){dl(Fv)(a,b,void 0===c?!1:c)}
function Tv(a,b,c){return dl(Z)(a,b,c)}
function Rv(){return dl(function(){return"_start"in kv()})()}
function Uv(){dl(function(){var a=nv();requestAnimationFrame(function(){setTimeout(function(){a===nv()&&Tv("ol",void 0,void 0)},0)})})()}
var Vv=window;Vv.ytcsi&&(Vv.ytcsi.infoGel=Sv,Vv.ytcsi.tick=Tv);var Wv="tokens consistency mss client_location entities adblock_detection response_received_commands store PLAYER_PRELOAD".split(" "),Xv=["type.googleapis.com/youtube.api.pfiinnertube.YoutubeApiInnertube.BrowseResponse","type.googleapis.com/youtube.api.pfiinnertube.YoutubeApiInnertube.PlayerResponse"];function Yv(a,b,c,d){this.A=a;this.fa=b;this.l=c;this.j=d;this.i=void 0;this.h=new Map;a.Qb||(a.Qb={});a.Qb=Object.assign({},gv,a.Qb)}
function Zv(a,b,c,d){if(void 0!==Yv.h){if(d=Yv.h,a=[a!==d.A,b!==d.fa,c!==d.l,!1,!1,!1,void 0!==d.i],a.some(function(e){return e}))throw new V("InnerTubeTransportService is already initialized",a);
}else Yv.h=new Yv(a,b,c,d)}
function $v(a){var b={signalServiceEndpoint:{signal:"GET_DATASYNC_IDS"}};var c=void 0===c?gm:c;var d=aw(a,b);return d?new Ud(function(e,f){var g,h,k,l,n;return A(function(p){switch(p.h){case 1:return p.yield(d,2);case 2:g=p.i;h=g.A(b,void 0,c);if(!h){f(new V("Error: Failed to build request for command.",b));p.B(0);break}Ku(h.input);l="cors"===(null==(k=h.hb)?void 0:k.mode)?"cors":void 0;if(a.l.bf){var r=h.config,t;r=null==r?void 0:null==(t=r.Vb)?void 0:t.sessionIndex;t=fm(0,{sessionIndex:r});n=Object.assign({},
Wu(l),t);p.B(4);break}return p.yield(bw(h.config,l),5);case 5:n=p.i;case 4:e(cw(a,h,n)),p.h=0}})}):Zd(new V("Error: No request builder found for command.",b))}
function dw(a,b,c){var d;if(b&&!(null==b?0:null==(d=b.sequenceMetaData)?0:d.skipProcessing)&&a.j){d=w(Wv);for(var e=d.next();!e.done;e=d.next())e=e.value,a.j[e]&&a.j[e].handleResponse(b,c)}}
function cw(a,b,c){var d=void 0===d?function(){}:d;
var e,f,g,h,k,l,n,p,r,t,y,v,z,G,K,N,S,da,sa,P,ea,na,La,Ka,Og,Pg,pr,qr,rr;return A(function(ha){switch(ha.h){case 1:ha.B(2);break;case 3:if((e=ha.i)&&!e.isExpired())return ha.return(Promise.resolve(e.h()));case 2:if(!(null==(f=b)?0:null==(g=f.Na)?0:g.context)){ha.B(4);break}h=b.Na.context;ha.B(5);break;case 5:k=w([]),l=k.next();case 7:if(l.done){ha.B(4);break}n=l.value;return ha.yield(n.Pg(h),8);case 8:l=k.next();ha.B(7);break;case 4:if(null==(p=a.i)||!p.Ug(b.input,b.Na)){ha.B(11);break}return ha.yield(a.i.Lg(b.input,
b.Na),12);case 12:return r=ha.i,U("kevlar_process_local_innertube_responses_killswitch")||dw(a,r,b),ha.return(r);case 11:return(v=null==(y=b.config)?void 0:y.Sg)&&a.h.has(v)?t=a.h.get(v):(z=JSON.stringify(b.Na),N=null!=(K=null==(G=b.hb)?void 0:G.headers)?K:{},b.hb=Object.assign({},b.hb,{headers:Object.assign({},N,c)}),S=Object.assign({},b.hb),"POST"===b.hb.method&&(S=Object.assign({},S,{body:z})),(null==(da=b.config)?0:da.He)&&Tv(b.config.He),sa=function(){return a.fa.fetch(b.input,S,b.config)},t=
sa(),v&&a.h.set(v,t)),ha.yield(t,13);
case 13:if((P=ha.i)&&"error"in P&&(null==(ea=P)?0:null==(na=ea.error)?0:na.details))for(La=P.error.details,Ka=w(La),Og=Ka.next();!Og.done;Og=Ka.next())Pg=Og.value,(pr=Pg["@type"])&&-1<Xv.indexOf(pr)&&(delete Pg["@type"],P=Pg);v&&a.h.has(v)&&a.h.delete(v);(null==(qr=b.config)?0:qr.Ie)&&Tv(b.config.Ie);if(P||null==(rr=a.i)||!rr.Dg(b.input,b.Na)){ha.B(14);break}return ha.yield(a.i.Kg(b.input,b.Na),15);case 15:P=ha.i;case 14:return dw(a,P,b),d(),ha.return(P||void 0)}})}
function aw(a,b){a:{a=a.A;var c,d=null==(c=hs(b,Jk))?void 0:c.signal;if(d&&a.Qb&&(c=a.Qb[d])){var e=c();break a}var f;if((c=null==(f=hs(b,Hk))?void 0:f.request)&&a.Qd&&(f=a.Qd[c])){e=f();break a}for(e in b)if(a.Wc[e]&&(b=a.Wc[e])){e=b();break a}e=void 0}if(void 0!==e)return Promise.resolve(e)}
function bw(a,b){var c,d,e,f;return A(function(g){if(1==g.h){e=null==(c=a)?void 0:null==(d=c.Vb)?void 0:d.sessionIndex;var h=g.yield;var k=fm(0,{sessionIndex:e});if(!(k instanceof Ud)){var l=new Ud(Bd);Vd(l,2,k);k=l}return h.call(g,k,2)}f=g.i;return g.return(Promise.resolve(Object.assign({},Wu(b),f)))})}
;var ew=new Dr("INNERTUBE_TRANSPORT_TOKEN");function fw(){}
x(fw,dv);fw.prototype.j=function(){return Xt};
fw.prototype.i=function(a){return hs(a,Rk)||void 0};
fw.prototype.h=function(a,b,c){c=void 0===c?{}:c;b.channelIds&&(a.channelIds=b.channelIds);b.siloName&&(a.siloName=b.siloName);b.params&&(a.params=b.params);c.botguardResponse&&(a.botguardResponse=c.botguardResponse);c.feature&&(a.clientFeature=c.feature)};
fa.Object.defineProperties(fw.prototype,{l:{configurable:!0,enumerable:!0,get:function(){return!0}}});function gw(){}
x(gw,dv);gw.prototype.j=function(){return Yt};
gw.prototype.i=function(a){return hs(a,Qk)||void 0};
gw.prototype.h=function(a,b){b.channelIds&&(a.channelIds=b.channelIds);b.siloName&&(a.siloName=b.siloName);b.params&&(a.params=b.params)};
fa.Object.defineProperties(gw.prototype,{l:{configurable:!0,enumerable:!0,get:function(){return!0}}});function hw(){}
x(hw,dv);hw.prototype.j=function(){return Ut};
hw.prototype.i=function(a){return hs(a,Lk)||void 0};
hw.prototype.h=function(a,b,c){a.feedbackTokens=[];b.feedbackToken&&a.feedbackTokens.push(b.feedbackToken);if(b=b.cpn||c.cpn)a.feedbackContext={cpn:b};a.isFeedbackTokenUnencrypted=!!c.is_feedback_token_unencrypted;a.shouldMerge=!1;c.extra_feedback_tokens&&(a.shouldMerge=!0,a.feedbackTokens=a.feedbackTokens.concat(c.extra_feedback_tokens))};
fa.Object.defineProperties(hw.prototype,{l:{configurable:!0,enumerable:!0,get:function(){return!0}}});function iw(){}
x(iw,dv);iw.prototype.j=function(){return Vt};
iw.prototype.i=function(a){return hs(a,Pk)||void 0};
iw.prototype.h=function(a,b){b.params&&(a.params=b.params);b.secondaryParams&&(a.secondaryParams=b.secondaryParams)};function jw(){}
x(jw,dv);jw.prototype.j=function(){return Wt};
jw.prototype.i=function(a){return hs(a,Ok)||void 0};
jw.prototype.h=function(a,b){b.actions&&(a.actions=b.actions);b.params&&(a.params=b.params);b.playlistId&&(a.playlistId=b.playlistId)};function kw(){}
x(kw,dv);kw.prototype.j=function(){return Tt};
kw.prototype.i=function(a){return hs(a,Nk)};
kw.prototype.h=function(a,b,c){c=void 0===c?{}:c;b.serializedShareEntity&&(a.serializedSharedEntity=b.serializedShareEntity);c.includeListId&&(a.includeListId=!0)};function lw(a,b){var c=B.apply(2,arguments);a=void 0===a?0:a;V.call(this,b,c);this.errorType=a;Object.setPrototypeOf(this,this.constructor.prototype)}
x(lw,V);var mw=new Dr("NETWORK_SLI_TOKEN");function nw(a){this.h=a}
nw.prototype.fetch=function(a,b,c){var d=this,e;return A(function(f){e=ow(d,a,b);return f.return(fetch(e).then(function(g){return d.handleResponse(g,c)}).catch(function(g){$s(g);
if((null==c?0:c.Wd)&&g instanceof lw&&1===g.errorType)return Promise.reject(g)}))})};
function ow(a,b,c){if(a.h){var d=mc(nc(5,xc(b,"key")))||"/UNKNOWN_PATH";a.h.start(d)}a=c;U("wug_networking_gzip_request")&&(a=Up(c));return new window.Request(b,a)}
nw.prototype.handleResponse=function(a,b){var c=a.text().then(function(d){if((null==b?0:b.pe)&&a.ok)return Mg(b.pe,d);d=d.replace(")]}'","");if((null==b?0:b.Wd)&&d)try{var e=JSON.parse(d)}catch(g){throw new lw(1,"JSON parsing failed after fetch");}var f;return null!=(f=e)?f:JSON.parse(d)});
a.redirected||a.ok?this.h&&this.h.success():(this.h&&this.h.Gg(),c=c.then(function(d){$s(new V("Error: API fetch failed",a.status,a.url,d));return Object.assign({},d,{errorMetadata:{status:a.status}})}));
return c};
nw[Cr]=[new Er(mw)];var pw=new Dr("NETWORK_MANAGER_TOKEN");var qw;function rw(){var a,b,c;return A(function(d){if(1==d.h)return a=Kr().resolve(ew),a?d.yield($v(a),2):($s(Error("InnertubeTransportService unavailable in fetchDatasyncIds")),d.return(void 0));if(b=d.i){if(b.errorMetadata)return $s(Error("Datasync IDs fetch responded with "+b.errorMetadata.status+": "+b.error)),d.return(void 0);c=b.Eg;return d.return(c)}$s(Error("Network request to get Datasync IDs failed."));return d.return(void 0)})}
;function sw(){var a;return null==(a=T("WEB_PLAYER_CONTEXT_CONFIGS"))?void 0:a.WEB_PLAYER_CONTEXT_CONFIG_ID_EMBEDDED_PLAYER}
;var tw=C.caches,uw;function vw(a){var b=a.indexOf(":");return-1===b?{md:a}:{md:a.substring(0,b),datasyncId:a.substring(b+1)}}
function ww(){return A(function(a){if(void 0!==uw)return a.return(uw);uw=new Promise(function(b){var c;return A(function(d){switch(d.h){case 1:return Aa(d,2),d.yield(tw.open("test-only"),4);case 4:return d.yield(tw.delete("test-only"),5);case 5:d.h=3;d.l=0;break;case 2:if(c=Ba(d),c instanceof Error&&"SecurityError"===c.name)return b(!1),d.return();case 3:b("caches"in window),d.h=0}})});
return a.return(uw)})}
function xw(a){var b,c,d,e,f,g,h;A(function(k){if(1==k.h)return k.yield(ww(),2);if(3!=k.h){if(!k.i)return k.return(!1);b=[];return k.yield(tw.keys(),3)}c=k.i;d=w(c);for(e=d.next();!e.done;e=d.next())f=e.value,g=vw(f),h=g.datasyncId,!h||a.includes(h)||b.push(tw.delete(f));return k.return(Promise.all(b).then(function(l){return l.some(function(n){return n})}))})}
function yw(){var a,b,c,d,e,f,g;return A(function(h){if(1==h.h)return h.yield(ww(),2);if(3!=h.h){if(!h.i)return h.return(!1);a=Am("cache contains other");return h.yield(tw.keys(),3)}b=h.i;c=w(b);for(d=c.next();!d.done;d=c.next())if(e=d.value,f=vw(e),(g=f.datasyncId)&&g!==a)return h.return(!0);return h.return(!1)})}
;function zw(){rw().then(function(a){a&&(Ho(a),xw(a),Hu(a))})}
function Aw(){var a=new Mq;Ni.pa(function(){var b,c,d,e;return A(function(f){switch(f.h){case 1:if(U("ytidb_clear_optimizations_killswitch")){f.B(2);break}b=Am("clear");if(b.startsWith("V")&&b.endsWith("||")){var g=[b];Ho(g);xw(g);Hu(g);return f.return()}c=Iu();return f.yield(yw(),3);case 3:return d=f.i,f.yield(Io(),4);case 4:if(e=f.i,!c&&!d&&!e)return f.return();case 2:a.va()?zw():a.h.add("publicytnetworkstatus-online",zw,!0,void 0,void 0),f.h=0}})})}
;function Bw(){this.state=1;this.h=null}
m=Bw.prototype;m.initialize=function(a,b,c){if(a.program){var d,e=null!=(d=a.interpreterUrl)?d:null;if(a.interpreterSafeScript){var f=a.interpreterSafeScript;f?((f=f.privateDoNotAccessOrElseSafeScriptWrappedValue)?(d=fb(),f=new ac(d?d.createScript(f):f)):f=null,d=f):d=null}else d=null!=(f=a.interpreterScript)?f:null;a.interpreterSafeUrl&&(e=Bk(a.interpreterSafeUrl).toString());Cw(this,d,e,a.program,b,c)}else $s(Error("Cannot initialize botguard without program"))};
function Cw(a,b,c,d,e,f){var g=void 0===g?"trayride":g;c?(a.state=2,au(c,function(){window[g]?Dw(a,d,g,e):(a.state=3,cu(c),$s(new V("Unable to load Botguard","from "+c)))},f)):b?(f=Gd("SCRIPT"),b instanceof ac?cc(f,b):f.textContent=b,f.nonce=$b(window),document.head.appendChild(f),document.head.removeChild(f),window[g]?Dw(a,d,g,e):(a.state=4,$s(new V("Unable to load Botguard from JS")))):$s(new V("Unable to load VM; no url or JS provided"))}
m.isLoading=function(){return 2===this.state};
function Dw(a,b,c,d){a.state=5;try{var e=new xi({program:b,de:c,Fe:U("att_web_record_metrics"),we:"aGIf"});e.Xe.then(function(){a.state=6;d&&d(b)});
a.Lc(e)}catch(f){a.state=7,f instanceof Error&&$s(f)}}
m.invoke=function(a){a=void 0===a?{}:a;return this.Pc()?this.Dd({Xc:a}):null};
m.dispose=function(){this.Lc(null);this.state=8};
m.Pc=function(){return!!this.h};
m.Dd=function(a){return this.h.xd(a)};
m.Lc=function(a){Cc(this.h);this.h=a};var Ew=[],Fw=!1;function Gw(){if(!U("disable_biscotti_fetch_for_ad_blocker_detection")&&!U("disable_biscotti_fetch_entirely_for_all_web_clients")&&Dt()){var a=T("PLAYER_VARS",{});if("1"!=Pb(a)&&!Et(a)){var b=function(){Fw=!0;"google_ad_status"in window?$k("DCLKSTAT",1):$k("DCLKSTAT",2)};
try{au("//static.doubleclick.net/instream/ad_status.js",b)}catch(c){}Ew.push(Ni.pa(function(){if(!(Fw||"google_ad_status"in window)){try{eu("//static.doubleclick.net/instream/ad_status.js",b)}catch(c){}Fw=!0;$k("DCLKSTAT",3)}},5E3))}}}
function Hw(){var a=Number(T("DCLKSTAT",0));return isNaN(a)?0:a}
;function Iw(){var a=E("yt.abuse.playerAttLoader");return a&&["bgvma","bgvmb","bgvmc"].every(function(b){return b in a})?a:null}
;function Jw(){Bw.apply(this,arguments)}
x(Jw,Bw);Jw.prototype.Lc=function(a){var b;null==(b=Iw())||b.bgvma();a?(b={bgvma:a.dispose.bind(a),bgvmb:a.snapshot.bind(a),bgvmc:a.xd.bind(a)},D("yt.abuse.playerAttLoader",b),D("yt.abuse.playerAttLoaderRun",function(c){return a.snapshot(c)})):(D("yt.abuse.playerAttLoader",null),D("yt.abuse.playerAttLoaderRun",null))};
Jw.prototype.Pc=function(){return!!Iw()};
Jw.prototype.Dd=function(a){return Iw().bgvmc(a)};function Kw(a){Tr.call(this,void 0===a?"document_active":a);var b=this;this.l=10;this.h=new Map;this.transitions=[{from:"document_active",to:"document_disposed_preventable",action:this.F},{from:"document_active",to:"document_disposed",action:this.A},{from:"document_disposed_preventable",to:"document_disposed",action:this.A},{from:"document_disposed_preventable",to:"flush_logs",action:this.m},{from:"document_disposed_preventable",to:"document_active",action:this.i},{from:"document_disposed",to:"flush_logs",
action:this.m},{from:"document_disposed",to:"document_active",action:this.i},{from:"document_disposed",to:"document_disposed",action:function(){}},
{from:"flush_logs",to:"document_active",action:this.i}];window.addEventListener("pagehide",function(c){b.transition("document_disposed",{event:c})});
window.addEventListener("beforeunload",function(c){b.transition("document_disposed_preventable",{event:c})})}
x(Kw,Tr);Kw.prototype.F=function(a,b){if(!this.h.get("document_disposed_preventable")){a(null==b?void 0:b.event);var c,d;if((null==b?0:null==(c=b.event)?0:c.defaultPrevented)||(null==b?0:null==(d=b.event)?0:d.returnValue)){b.event.returnValue||(b.event.returnValue=!0);b.event.defaultPrevented||b.event.preventDefault();this.h=new Map;this.transition("document_active");return}}this.h.set("document_disposed_preventable",!0);this.h.get("document_disposed")?this.transition("flush_logs"):this.transition("document_disposed")};
Kw.prototype.A=function(a,b){this.h.get("document_disposed")?this.transition("document_active"):(a(null==b?void 0:b.event),this.h.set("document_disposed",!0),this.transition("flush_logs"))};
Kw.prototype.m=function(a,b){a(null==b?void 0:b.event);this.transition("document_active")};
Kw.prototype.i=function(){this.h=new Map};function Lw(a){Tr.call(this,void 0===a?"document_visibility_unknown":a);var b=this;this.transitions=[{from:"document_visibility_unknown",to:"document_visible",action:this.i},{from:"document_visibility_unknown",to:"document_hidden",action:this.h},{from:"document_visibility_unknown",to:"document_foregrounded",action:this.m},{from:"document_visibility_unknown",to:"document_backgrounded",action:this.A},{from:"document_visible",to:"document_hidden",action:this.h},{from:"document_visible",to:"document_foregrounded",
action:this.m},{from:"document_visible",to:"document_visible",action:this.i},{from:"document_foregrounded",to:"document_visible",action:this.i},{from:"document_foregrounded",to:"document_hidden",action:this.h},{from:"document_foregrounded",to:"document_foregrounded",action:this.m},{from:"document_hidden",to:"document_visible",action:this.i},{from:"document_hidden",to:"document_backgrounded",action:this.A},{from:"document_hidden",to:"document_hidden",action:this.h},{from:"document_backgrounded",to:"document_hidden",
action:this.h},{from:"document_backgrounded",to:"document_backgrounded",action:this.A},{from:"document_backgrounded",to:"document_visible",action:this.i}];document.addEventListener("visibilitychange",function(c){"visible"===document.visibilityState?b.transition("document_visible",{event:c}):b.transition("document_hidden",{event:c})});
U("visibility_lifecycles_dynamic_backgrounding")&&(window.addEventListener("blur",function(c){b.transition("document_backgrounded",{event:c})}),window.addEventListener("focus",function(c){b.transition("document_foregrounded",{event:c})}))}
x(Lw,Tr);Lw.prototype.i=function(a,b){a(null==b?void 0:b.event);U("visibility_lifecycles_dynamic_backgrounding")&&this.transition("document_foregrounded")};
Lw.prototype.h=function(a,b){a(null==b?void 0:b.event);U("visibility_lifecycles_dynamic_backgrounding")&&this.transition("document_backgrounded")};
Lw.prototype.A=function(a,b){a(null==b?void 0:b.event)};
Lw.prototype.m=function(a,b){a(null==b?void 0:b.event)};function Mw(){this.l=new Kw;this.A=new Lw}
Mw.prototype.install=function(){var a=B.apply(0,arguments),b=this;a.forEach(function(c){b.l.install(c)});
a.forEach(function(c){b.A.install(c)})};function Nw(){this.l=[];this.i=new Map;this.h=new Map;this.j=new Set}
Nw.prototype.clickCommand=function(a,b,c){var d=a.clickTrackingParams;c=void 0===c?0:c;if(d)if(c=rt(void 0===c?0:c)){a=this.client;d=new kt({trackingParams:d});var e=void 0;if(U("no_client_ve_attach_unless_shown")){var f=Cu(d,c);yu.set(f,!0);Du(d,c)}e=e||"INTERACTION_LOGGING_GESTURE_TYPE_GENERIC_CLICK";f=Bu({cttAuthInfo:tt(c)||void 0},c);d={csn:c,ve:d.getAsJson(),gestureType:e};b&&(d.clientData=b);"UNDEFINED_CSN"===c?Eu("visualElementGestured",f,d):a?Ss("visualElementGestured",d,a,f):rn("visualElementGestured",
d,f);b=!0}else b=!1;else b=!1;return b};
Nw.prototype.stateChanged=function(a,b,c){this.visualElementStateChanged(new kt({trackingParams:a}),b,void 0===c?0:c)};
Nw.prototype.visualElementStateChanged=function(a,b,c){c=void 0===c?0:c;if(0===c&&this.j.has(c))this.l.push([a,b]);else{var d=c;d=void 0===d?0:d;c=rt(d);a||(a=(a=ot(void 0===d?0:d))?new kt({veType:a,youtubeData:void 0,jspbYoutubeData:void 0}):null);var e=a;c&&e&&(a=this.client,d=Bu({cttAuthInfo:tt(c)||void 0},c),b={csn:c,ve:e.getAsJson(),clientData:b},"UNDEFINED_CSN"===c?Eu("visualElementStateChanged",d,b):a?Ss("visualElementStateChanged",b,a,d):rn("visualElementStateChanged",b,d))}};
function Ow(a,b){if(void 0===b)for(var c=qt(),d=0;d<c.length;d++)void 0!==c[d]&&Ow(a,c[d]);else a.i.forEach(function(e,f){(f=a.h.get(f))&&Au(a.client,b,f,e)}),a.i.clear(),a.h.clear()}
;function Pw(){Mw.call(this);var a={};this.install((a.document_disposed={callback:this.h},a));U("combine_ve_grafts")&&(a={},this.install((a.document_disposed={callback:this.i},a)));a={};this.install((a.flush_logs={callback:this.j},a));U("web_log_cfg_cee_ks")||Cm(Qw)}
x(Pw,Mw);Pw.prototype.j=function(){rn("finalPayload",{csn:rt()})};
Pw.prototype.h=function(){dt(et)};
Pw.prototype.i=function(){var a=Ow;Nw.h||(Nw.h=new Nw);a(Nw.h)};
function Qw(){var a=T("CLIENT_EXPERIMENT_EVENTS");if(a){var b=Hi();a=w(a);for(var c=a.next();!c.done;c=a.next())c=c.value,b(c)&&rn("genericClientExperimentEvent",{eventType:c});delete Zk.CLIENT_EXPERIMENT_EVENTS}}
;function Rw(){}
function Sw(){var a=E("ytglobal.storage_");a||(a=new Rw,D("ytglobal.storage_",a));return a}
Rw.prototype.estimate=function(){var a,b,c;return A(function(d){a=navigator;return(null==(b=a.storage)?0:b.estimate)?d.return(a.storage.estimate()):(null==(c=a.webkitTemporaryStorage)?0:c.queryUsageAndQuota)?d.return(Tw()):d.return()})};
function Tw(){var a=navigator;return new Promise(function(b,c){var d;null!=(d=a.webkitTemporaryStorage)&&d.queryUsageAndQuota?a.webkitTemporaryStorage.queryUsageAndQuota(function(e,f){b({usage:e,quota:f})},function(e){c(e)}):c(Error("webkitTemporaryStorage is not supported."))})}
D("ytglobal.storageClass_",Rw);function pn(a,b){var c=this;this.handleError=a;this.h=b;this.i=!1;void 0===self.document||self.addEventListener("beforeunload",function(){c.i=!0});
this.j=.2>=Math.random()}
pn.prototype.Ga=function(a){this.handleError(a)};
pn.prototype.logEvent=function(a,b){switch(a){case "IDB_DATA_CORRUPTED":U("idb_data_corrupted_killswitch")||this.h("idbDataCorrupted",b);break;case "IDB_UNEXPECTEDLY_CLOSED":this.h("idbUnexpectedlyClosed",b);break;case "IS_SUPPORTED_COMPLETED":U("idb_is_supported_completed_killswitch")||this.h("idbIsSupportedCompleted",b);break;case "QUOTA_EXCEEDED":Uw(this,b);break;case "TRANSACTION_ENDED":this.j&&.1>=Math.random()&&this.h("idbTransactionEnded",b);break;case "TRANSACTION_UNEXPECTEDLY_ABORTED":a=
Object.assign({},b,{hasWindowUnloaded:this.i}),this.h("idbTransactionAborted",a)}};
function Uw(a,b){Sw().estimate().then(function(c){c=Object.assign({},b,{isSw:void 0===self.document,isIframe:self!==self.top,deviceStorageUsageMbytes:Vw(null==c?void 0:c.usage),deviceStorageQuotaMbytes:Vw(null==c?void 0:c.quota)});a.h("idbQuotaExceeded",c)})}
function Vw(a){return"undefined"===typeof a?"-1":String(Math.ceil(a/1048576))}
;var Ww={},Xw=(Ww["api.invalidparam"]=2,Ww.auth=150,Ww["drm.auth"]=150,Ww["heartbeat.net"]=150,Ww["heartbeat.servererror"]=150,Ww["heartbeat.stop"]=150,Ww["html5.unsupportedads"]=5,Ww["fmt.noneavailable"]=5,Ww["fmt.decode"]=5,Ww["fmt.unplayable"]=5,Ww["html5.missingapi"]=5,Ww["html5.unsupportedlive"]=5,Ww["drm.unavailable"]=5,Ww["mrm.blocked"]=151,Ww);var Yw=new Set("endSeconds startSeconds mediaContentUrl suggestedQuality videoId rct rctn".split(" "));function Zw(a){return(0===a.search("cue")||0===a.search("load"))&&"loadModule"!==a}
function $w(a,b,c){if("string"===typeof a)return{videoId:a,startSeconds:b,suggestedQuality:c};b={};c=w(Yw);for(var d=c.next();!d.done;d=c.next())d=d.value,a[d]&&(b[d]=a[d]);return b}
function ax(a,b,c,d){if(Ra(a)&&!Array.isArray(a)){b="playlist list listType index startSeconds suggestedQuality".split(" ");c={};for(d=0;d<b.length;d++){var e=b[d];a[e]&&(c[e]=a[e])}return c}b={index:b,startSeconds:c,suggestedQuality:d};"string"===typeof a&&16===a.length?b.list="PL"+a:b.playlist=a;return b}
;function bx(){var a=cx;this.i=[];this.isReady=!1;this.j={};this.listeners=[];this.l=!1;var b=this.h=new Rt(!!T("WIDGET_ID_ENFORCE")),c=this.Ee.bind(this);b.l=c;b.m=null;this.h.channel="widget";if(b=T("WIDGET_ID"))this.h.sessionId=b;this.api=a;this.addEventListener("onReady",this.onReady.bind(this));this.addEventListener("onVideoProgress",this.Se.bind(this));this.addEventListener("onVolumeChange",this.Te.bind(this));this.addEventListener("onApiChange",this.Me.bind(this));this.addEventListener("onPlaybackQualityChange",
this.Pe.bind(this));this.addEventListener("onPlaybackRateChange",this.Qe.bind(this));this.addEventListener("onStateChange",this.Re.bind(this));this.addEventListener("onWebglSettingsChanged",this.Ue.bind(this));this.addEventListener("captionssettingschanged",this.Ne.bind(this))}
m=bx.prototype;
m.Ee=function(a,b,c){if("addEventListener"===a&&b)a=b[0],"onReady"===a?this.api.logApiCall(a+" invocation",c):"onError"===a&&this.l&&(this.api.logApiCall(a+" invocation",c,this.errorCode),this.errorCode=void 0),this.api.logApiCall(a+" registration",c),this.j[a]||"onReady"===a||(this.addEventListener(a,dx(this,a,c)),this.j[a]=!0);else if(this.api.isExternalMethodAvailable(a,c)){b=b||[];if(0<b.length&&Zw(a)){var d=b;if(Ra(d[0])&&!Array.isArray(d[0]))var e=d[0];else switch(e={},a){case "loadVideoById":case "cueVideoById":e=$w(d[0],
void 0!==d[1]?Number(d[1]):void 0,d[2]);break;case "loadVideoByUrl":case "cueVideoByUrl":e=d[0];"string"===typeof e&&(e={mediaContentUrl:e,startSeconds:void 0!==d[1]?Number(d[1]):void 0,suggestedQuality:d[2]});b:{if((d=e.mediaContentUrl)&&(d=/\/([ve]|embed)\/([^#?]+)/.exec(d))&&d[2]){d=d[2];break b}d=null}e.videoId=d;e=$w(e);break;case "loadPlaylist":case "cuePlaylist":e=ax(d[0],d[1],d[2],d[3])}b.length=1;b[0]=e}this.api.handleExternalCall(a,b,c);Zw(a)&&ex(this,fx(this))}};
m.Zd=function(){this.isReady=!0;this.sendMessage("initialDelivery",fx(this));this.sendMessage("onReady");Db(this.i,this.ud,this);this.i=[]};
function ex(a,b){a.sendMessage("infoDelivery",b)}
m.ud=function(a){this.isReady?this.h.sendMessage(a):this.i.push(a)};
m.sendMessage=function(a,b){this.ud({event:a,info:void 0===b?null:b})};
function dx(a,b,c){return function(d){"onError"===b?a.api.logApiCall(b+" invocation",c,d):a.api.logApiCall(b+" invocation",c);a.sendMessage(b,d)}}
m.onReady=function(){var a=this.h,b=this.Zd.bind(this);a.h=b;a=this.api.getVideoData();if(!a.isPlayable){this.l=!0;a=a.errorCode;var c=void 0===c?5:c;this.errorCode=a?Xw[a]||c:c;this.sendMessage("onError",this.errorCode.toString())}};
m.addEventListener=function(a,b){this.listeners.push({eventType:a,listener:b});this.api.addEventListener(a,b)};
function fx(a){if(!a.api)return null;var b=a.api.getApiInterface();Ib(b,"getVideoData");for(var c={apiInterface:b},d=0,e=b.length;d<e;d++){var f=b[d];if(0===f.search("get")||0===f.search("is")){var g=0;0===f.search("get")?g=3:0===f.search("is")&&(g=2);g=f.charAt(g).toLowerCase()+f.substr(g+1);try{var h=a.api[f]();c[g]=h}catch(k){}}}c.videoData=a.api.getVideoData();c.currentTimeLastUpdated_=Date.now()/1E3;return c}
m.Re=function(a){a={playerState:a,currentTime:this.api.getCurrentTime(),duration:this.api.getDuration(),videoData:this.api.getVideoData(),videoStartBytes:0,videoBytesTotal:this.api.getVideoBytesTotal(),videoLoadedFraction:this.api.getVideoLoadedFraction(),playbackQuality:this.api.getPlaybackQuality(),availableQualityLevels:this.api.getAvailableQualityLevels(),currentTimeLastUpdated_:Date.now()/1E3,playbackRate:this.api.getPlaybackRate(),mediaReferenceTime:this.api.getMediaReferenceTime()};this.api.getVideoUrl&&
(a.videoUrl=this.api.getVideoUrl());this.api.getVideoContentRect&&(a.videoContentRect=this.api.getVideoContentRect());this.api.getProgressState&&(a.progressState=this.api.getProgressState());this.api.getPlaylist&&(a.playlist=this.api.getPlaylist());this.api.getPlaylistIndex&&(a.playlistIndex=this.api.getPlaylistIndex());this.api.getStoryboardFormat&&(a.storyboardFormat=this.api.getStoryboardFormat());ex(this,a)};
m.Pe=function(a){a={playbackQuality:a};this.api.getAvailableQualityLevels&&(a.availableQualityLevels=this.api.getAvailableQualityLevels());this.api.getPreferredQuality&&(a.preferredQuality=this.api.getPreferredQuality());ex(this,a)};
m.Qe=function(a){ex(this,{playbackRate:a})};
m.Me=function(){for(var a=this.api.getOptions(),b={namespaces:a},c=0,d=a.length;c<d;c++){var e=a[c],f=this.api.getOptions(e);a.join(", ");b[e]={options:f};for(var g=0,h=f.length;g<h;g++){var k=f[g],l=this.api.getOption(e,k);b[e][k]=l}}this.sendMessage("apiInfoDelivery",b)};
m.Te=function(){ex(this,{muted:this.api.isMuted(),volume:this.api.getVolume()})};
m.Se=function(a){a={currentTime:a,videoBytesLoaded:this.api.getVideoBytesLoaded(),videoLoadedFraction:this.api.getVideoLoadedFraction(),currentTimeLastUpdated_:Date.now()/1E3,playbackRate:this.api.getPlaybackRate(),mediaReferenceTime:this.api.getMediaReferenceTime()};this.api.getProgressState&&(a.progressState=this.api.getProgressState());ex(this,a)};
m.Ue=function(){var a={sphericalProperties:this.api.getSphericalProperties()};ex(this,a)};
m.Ne=function(){if(this.api.getSubtitlesUserSettings){var a={subtitlesUserSettings:this.api.getSubtitlesUserSettings()};ex(this,a)}};
m.dispose=function(){this.h=null;for(var a=0;a<this.listeners.length;a++){var b=this.listeners[a];this.api.removeEventListener(b.eventType,b.listener)}this.listeners=[]};function gx(a,b){F.call(this);this.h={};this.started=!1;this.connection=b;this.connection.subscribe("command",this.pd,this);this.api=a;this.start()}
x(gx,F);m=gx.prototype;m.start=function(){this.started||this.V||(this.started=!0,this.connection.ib("RECEIVING"))};
m.ib=function(a,b){this.started&&!this.V&&this.connection.ib(a,b)};
m.pd=function(a,b,c){if(this.started&&!this.V){var d=b||{};switch(a){case "addEventListener":"string"===typeof d.event&&this.addListener(d.event);break;case "removeEventListener":"string"===typeof d.event&&this.removeListener(d.event);break;default:this.api.isReady()&&this.api.isExternalMethodAvailable(a,c||null)&&(b=hx(a,b||{}),c=this.api.handleExternalCall(a,b,c||null),(c=ix(a,c))&&this.ib(a,c))}}};
m.addListener=function(a){if(!(a in this.h)){var b=this.Oe.bind(this,a);this.h[a]=b;this.addEventListener(a,b)}};
m.Oe=function(a,b){this.started&&!this.V&&this.connection.ib(a,jx(a,b))};
m.removeListener=function(a){a in this.h&&(this.removeEventListener(a,this.h[a]),delete this.h[a])};
m.addEventListener=function(a,b){this.api.addEventListener(a,b)};
m.removeEventListener=function(a,b){this.api.removeEventListener(a,b)};
function hx(a,b){switch(a){case "loadVideoById":return a=$w(b),[a];case "cueVideoById":return a=$w(b),[a];case "loadVideoByPlayerVars":return[b];case "cueVideoByPlayerVars":return[b];case "loadPlaylist":return a=ax(b),[a];case "cuePlaylist":return a=ax(b),[a];case "seekTo":return[b.seconds,b.allowSeekAhead];case "playVideoAt":return[b.index];case "setVolume":return[b.volume];case "setPlaybackQuality":return[b.suggestedQuality];case "setPlaybackRate":return[b.suggestedRate];case "setLoop":return[b.loopPlaylists];
case "setShuffle":return[b.shufflePlaylist];case "getOptions":return[b.module];case "getOption":return[b.module,b.option];case "setOption":return[b.module,b.option,b.value];case "handleGlobalKeyDown":return[b.keyCode,b.shiftKey,b.ctrlKey,b.altKey,b.metaKey,b.key,b.code]}return[]}
function ix(a,b){switch(a){case "isMuted":return{muted:b};case "getVolume":return{volume:b};case "getPlaybackRate":return{playbackRate:b};case "getAvailablePlaybackRates":return{availablePlaybackRates:b};case "getVideoLoadedFraction":return{videoLoadedFraction:b};case "getPlayerState":return{playerState:b};case "getCurrentTime":return{currentTime:b};case "getPlaybackQuality":return{playbackQuality:b};case "getAvailableQualityLevels":return{availableQualityLevels:b};case "getDuration":return{duration:b};
case "getVideoUrl":return{videoUrl:b};case "getVideoEmbedCode":return{videoEmbedCode:b};case "getPlaylist":return{playlist:b};case "getPlaylistIndex":return{playlistIndex:b};case "getOptions":return{options:b};case "getOption":return{option:b}}}
function jx(a,b){switch(a){case "onReady":return;case "onStateChange":return{playerState:b};case "onPlaybackQualityChange":return{playbackQuality:b};case "onPlaybackRateChange":return{playbackRate:b};case "onError":return{errorCode:b}}if(null!=b)return{value:b}}
m.U=function(){this.connection.unsubscribe("command",this.pd,this);this.connection=null;for(var a in this.h)this.h.hasOwnProperty(a)&&this.removeListener(a);F.prototype.U.call(this);delete this.api};function kx(a,b,c){pu.call(this);this.j=a;this.i=b;this.id=c}
x(kx,pu);kx.prototype.ib=function(a,b){this.V||this.j.ib(this.i,this.id,a,b)};
kx.prototype.U=function(){this.i=this.j=null;pu.prototype.U.call(this)};function lx(a,b,c){F.call(this);this.h=a;this.origin=c;this.i=gr(window,"message",this.j.bind(this));this.connection=new kx(this,a,b);Ec(this,this.connection)}
x(lx,F);lx.prototype.ib=function(a,b,c,d){this.V||a!==this.h||(a={id:b,command:c},d&&(a.data=d),this.h.postMessage(JSON.stringify(a),this.origin))};
lx.prototype.j=function(a){if(!this.V&&a.origin===this.origin){var b=a.data;if("string"===typeof b){try{b=JSON.parse(b)}catch(d){return}if(b.command){var c=this.connection;c.V||c.l("command",b.command,b.data,a.origin)}}}};
lx.prototype.U=function(){ir(this.i);this.h=null;F.prototype.U.call(this)};var mx=new Jw;function nx(){return mx.Pc()}
function ox(a){a=void 0===a?{}:a;return mx.invoke(a)}
;function px(a,b,c,d,e){F.call(this);var f=this;this.v=b;this.webPlayerContextConfig=d;this.sc=e;this.Ya=!1;this.api={};this.ia=this.m=null;this.W=new M;this.h={};this.da=this.xa=this.elementId=this.Bb=this.config=null;this.ba=!1;this.j=this.F=null;this.Fa={};this.uc=["onReady"];this.lastError=null;this.Sb=NaN;this.K={};this.ga=0;this.i=this.l=a;Ec(this,this.W);qx(this);c?this.ga=setTimeout(function(){f.loadNewVideoConfig(c)},0):d&&(rx(this),sx(this))}
x(px,F);m=px.prototype;m.getId=function(){return this.v};
m.loadNewVideoConfig=function(a){if(!this.V){this.ga&&(clearTimeout(this.ga),this.ga=0);var b=a||{};b instanceof St||(b=new St(b));this.config=b;this.setConfig(a);sx(this);this.isReady()&&tx(this)}};
function rx(a){var b;a.webPlayerContextConfig?b=a.webPlayerContextConfig.rootElementId:b=a.config.attrs.id;a.elementId=b||a.elementId;"video-player"===a.elementId&&(a.elementId=a.v,a.webPlayerContextConfig?a.webPlayerContextConfig.rootElementId=a.v:a.config.attrs.id=a.v);var c;(null==(c=a.i)?void 0:c.id)===a.elementId&&(a.elementId+="-player",a.webPlayerContextConfig?a.webPlayerContextConfig.rootElementId=a.elementId:a.config.attrs.id=a.elementId)}
m.setConfig=function(a){this.Bb=a;this.config=ux(a);rx(this);if(!this.xa){var b;this.xa=vx(this,(null==(b=this.config.args)?void 0:b.jsapicallback)||"onYouTubePlayerReady")}this.config.args?this.config.args.jsapicallback=null:this.config.args={jsapicallback:null};var c;if(null==(c=this.config)?0:c.attrs)a=this.config.attrs,(b=a.width)&&this.i&&(this.i.style.width=Fi(Number(b)||b)),(a=a.height)&&this.i&&(this.i.style.height=Fi(Number(a)||a))};
function tx(a){if(a.config&&!0!==a.config.loaded)if(a.config.loaded=!0,!a.config.args||"0"!==a.config.args.autoplay&&0!==a.config.args.autoplay&&!1!==a.config.args.autoplay){var b;a.api.loadVideoByPlayerVars(null!=(b=a.config.args)?b:null)}else a.api.cueVideoByPlayerVars(a.config.args)}
function wx(a){var b=!0,c=xx(a);c&&a.config&&(b=c.dataset.version===yx(a));return b&&!!E("yt.player.Application.create")}
function sx(a){if(!a.V&&!a.ba){var b=wx(a);if(b&&"html5"===(xx(a)?"html5":null))a.da="html5",a.isReady()||zx(a);else if(Ax(a),a.da="html5",b&&a.j&&a.l)a.l.appendChild(a.j),zx(a);else{a.config&&(a.config.loaded=!0);var c=!1;a.F=function(){c=!0;var d=Bx(a,"player_bootstrap_method")?E("yt.player.Application.createAlternate")||E("yt.player.Application.create"):E("yt.player.Application.create");var e=a.config?ux(a.config):void 0;d&&d(a.l,e,a.webPlayerContextConfig,a.sc);zx(a)};
a.ba=!0;b?a.F():(au(yx(a),a.F),(b=Cx(a))&&hu(b||""),Dx(a)&&!c&&D("yt.player.Application.create",null))}}}
function xx(a){var b=Fd(a.elementId);!b&&a.i&&a.i.querySelector&&(b=a.i.querySelector("#"+a.elementId));return b}
function zx(a){if(!a.V){var b=xx(a),c=!1;b&&b.getApiInterface&&b.getApiInterface()&&(c=!0);if(c){a.ba=!1;if(!Bx(a,"html5_remove_not_servable_check_killswitch")){var d;if((null==b?0:b.isNotServable)&&a.config&&(null==b?0:b.isNotServable(null==(d=a.config.args)?void 0:d.video_id)))return}Ex(a)}else a.Sb=setTimeout(function(){zx(a)},50)}}
function Ex(a){qx(a);a.Ya=!0;var b=xx(a);if(b){a.m=Fx(a,b,"addEventListener");a.ia=Fx(a,b,"removeEventListener");var c=b.getApiInterface();c=c.concat(b.getInternalApiInterface());for(var d=a.api,e=0;e<c.length;e++){var f=c[e];d[f]||(d[f]=Fx(a,b,f))}}for(var g in a.h)a.h.hasOwnProperty(g)&&a.m&&a.m(g,a.h[g]);tx(a);a.xa&&a.xa(a.api);a.W.Xa("onReady",a.api)}
function Fx(a,b,c){var d=b[c];return function(){var e=B.apply(0,arguments);try{return a.lastError=null,d.apply(b,e)}catch(f){if("sendAbandonmentPing"!==c)throw f.params=c,a.lastError=f,e=new V("PlayerProxy error in method call",{error:f,method:c,playerId:a.v}),e.level="WARNING",e;}}}
function qx(a){a.Ya=!1;if(a.ia)for(var b in a.h)a.h.hasOwnProperty(b)&&a.ia(b,a.h[b]);for(var c in a.K)a.K.hasOwnProperty(c)&&clearTimeout(Number(c));a.K={};a.m=null;a.ia=null;b=a.api;for(var d in b)b.hasOwnProperty(d)&&(b[d]=null);b.addEventListener=function(e,f){a.addEventListener(e,f)};
b.removeEventListener=function(e,f){a.removeEventListener(e,f)};
b.destroy=function(){a.dispose()};
b.getLastError=function(){return a.getLastError()};
b.getPlayerType=function(){return a.getPlayerType()};
b.getCurrentVideoConfig=function(){return a.Bb};
b.loadNewVideoConfig=function(e){a.loadNewVideoConfig(e)};
b.isReady=function(){return a.isReady()}}
m.isReady=function(){return this.Ya};
m.addEventListener=function(a,b){var c=this,d=vx(this,b);d&&(0<=Cb(this.uc,a)||this.h[a]||(b=Gx(this,a),this.m&&this.m(a,b)),this.W.subscribe(a,d),"onReady"===a&&this.isReady()&&setTimeout(function(){d(c.api)},0))};
m.removeEventListener=function(a,b){this.V||(b=vx(this,b))&&this.W.unsubscribe(a,b)};
function vx(a,b){var c=b;if("string"===typeof b){if(a.Fa[b])return a.Fa[b];c=function(){var d=B.apply(0,arguments),e=E(b);if(e)try{e.apply(C,d)}catch(f){throw d=new V("PlayerProxy error when executing callback",{error:f}),d.level="ERROR",d;}};
a.Fa[b]=c}return c?c:null}
function Gx(a,b){function c(d){var e=setTimeout(function(){if(!a.V){try{a.W.Xa(b,null!=d?d:void 0)}catch(h){var f=new V("PlayerProxy error when creating global callback",{error:h.message,event:b,playerId:a.v,data:d,originalStack:h.stack});f.level="WARNING";throw f;}f=a.K;var g=String(e);g in f&&delete f[g]}},0);
Ob(a.K,String(e))}
return a.h[b]=c}
m.getPlayerType=function(){return this.da||(xx(this)?"html5":null)};
m.getLastError=function(){return this.lastError};
function Ax(a){a.cancel();qx(a);a.da=null;a.config&&(a.config.loaded=!1);var b=xx(a);b&&(wx(a)||!Dx(a)?a.j=b:(b&&b.destroy&&b.destroy(),a.j=null));if(a.l)for(a=a.l;b=a.firstChild;)a.removeChild(b)}
m.cancel=function(){this.F&&eu(yx(this),this.F);clearTimeout(this.Sb);this.ba=!1};
m.U=function(){Ax(this);if(this.j&&this.config&&this.j.destroy)try{this.j.destroy()}catch(b){var a=new V("PlayerProxy error during disposal",{error:b});a.level="ERROR";throw a;}this.Fa=null;for(a in this.h)this.h.hasOwnProperty(a)&&delete this.h[a];this.Bb=this.config=this.api=null;delete this.l;delete this.i;F.prototype.U.call(this)};
function Dx(a){var b,c;a=null==(b=a.config)?void 0:null==(c=b.args)?void 0:c.fflags;return!!a&&-1!==a.indexOf("player_destroy_old_version=true")}
function yx(a){return a.webPlayerContextConfig?a.webPlayerContextConfig.jsUrl:(a=a.config.assets)?a.js:""}
function Cx(a){return a.webPlayerContextConfig?a.webPlayerContextConfig.cssUrl:(a=a.config.assets)?a.css:""}
function Bx(a,b){if(a.webPlayerContextConfig)var c=a.webPlayerContextConfig.serializedExperimentFlags;else{var d;if(null==(d=a.config)?0:d.args)c=a.config.args.fflags}return(c||"").split("&").includes(b+"=true")}
function ux(a){for(var b={},c=w(Object.keys(a)),d=c.next();!d.done;d=c.next()){d=d.value;var e=a[d];b[d]="object"===typeof e?Rb(e):e}return b}
;var Hx={},Ix="player_uid_"+(1E9*Math.random()>>>0);function Jx(a,b){var c="player",d=!1;d=void 0===d?!0:d;c="string"===typeof c?Fd(c):c;var e=Ix+"_"+Sa(c),f=Hx[e];if(f&&d)return Kx(a,b)?f.api.loadVideoByPlayerVars(a.args||null):f.loadNewVideoConfig(a),f.api;f=new px(c,e,a,b,void 0);Hx[e]=f;f.addOnDisposeCallback(function(){delete Hx[f.getId()]});
return f.api}
function Kx(a,b){return b&&b.serializedExperimentFlags?b.serializedExperimentFlags.includes("web_player_remove_playerproxy=true"):a&&a.args&&a.args.fflags?a.args.fflags.includes("web_player_remove_playerproxy=true"):!1}
;var cx=null,Lx=null,Mx=null;
function Nx(){Uv();var a=om(),b=rm(119),c=1<window.devicePixelRatio;if(document.body&&Vi(document.body,"exp-invert-logo"))if(c&&!Vi(document.body,"inverted-hdpi")){var d=document.body;if(d.classList)d.classList.add("inverted-hdpi");else if(!Vi(d,"inverted-hdpi")){var e=Ti(d);Ui(d,e+(0<e.length?" inverted-hdpi":"inverted-hdpi"))}}else!c&&Vi(document.body,"inverted-hdpi")&&Wi();if(b!=c){b="f"+(Math.floor(119/31)+1);d=sm(b)||0;d=c?d|67108864:d&-67108865;0===d?delete lm[b]:(c=d.toString(16),lm[b]=c.toString());
c=!0;U("web_secure_pref_cookie_killswitch")&&(c=!1);b=a.h;d=[];for(f in lm)lm.hasOwnProperty(f)&&d.push(f+"="+encodeURIComponent(String(lm[f])));var f=d.join("&");hm(b,f,63072E3,a.i,c)}}
function Ox(){Px()}
function Qx(){Tv("ep_init_pr");Px()}
function Px(){var a=cx.getVideoData(1);a=a.title?a.title+" - YouTube":"YouTube";document.title!==a&&(document.title=a)}
function Rx(){cx&&cx.sendAbandonmentPing&&cx.sendAbandonmentPing();T("PL_ATT")&&mx.dispose();for(var a=Ni,b=0,c=Ew.length;b<c;b++)a.qa(Ew[b]);Ew.length=0;cu("//static.doubleclick.net/instream/ad_status.js");Fw=!1;$k("DCLKSTAT",0);Dc(Mx,Lx);cx&&(cx.removeEventListener("onVideoDataChange",Ox),cx.destroy())}
;D("yt.setConfig",$k);D("yt.config.set",$k);D("yt.setMsg",$t);D("yt.msgs.set",$t);D("yt.logging.errors.log",Zs);
D("writeEmbed",function(){var a=T("PLAYER_CONFIG");if(!a){var b=T("PLAYER_VARS");b&&(a={args:b})}Mu(!0);"gvn"===a.args.ps&&(document.body.style.backgroundColor="transparent");a.attrs||(a.attrs={width:"100%",height:"100%",id:"video-player"});var c=document.referrer;b=T("POST_MESSAGE_ORIGIN");window!==window.top&&c&&c!==document.URL&&(a.args.loaderUrl=c);Pv("embed",["ol"]);c=sw();if(!c.serializedForcedExperimentIds){var d=nl(window.location.href);d.forced_experiments&&(c.serializedForcedExperimentIds=
d.forced_experiments)}var e;(null==(e=a.args)?0:e.autoplay)&&Pv("watch",["pbs","pbu","pbp"]);cx=Jx(a,c);cx.addEventListener("onVideoDataChange",Ox);cx.addEventListener("onReady",Qx);a=T("POST_MESSAGE_ID","player");T("ENABLE_JS_API")?Mx=new bx:T("ENABLE_POST_API")&&"string"===typeof a&&"string"===typeof b&&(Lx=new lx(window.parent,a,b),Mx=new gx(cx,Lx.connection));Gw();U("ytidb_create_logger_embed_killswitch")||on();a={};Pw.h||(Pw.h=new Pw);Pw.h.install((a.flush_logs={callback:function(){Fs()}},a));
Yq();U("ytidb_clear_embedded_player")&&Ni.pa(function(){var f,g;if(!qw){var h=Kr();Gr(h,{nc:pw,Ad:nw});var k={Wc:{feedbackEndpoint:Zu(hw),modifyChannelNotificationPreferenceEndpoint:Zu(iw),playlistEditEndpoint:Zu(jw),subscribeEndpoint:Zu(fw),unsubscribeEndpoint:Zu(gw),webPlayerShareEntityServiceEndpoint:Zu(kw)}},l=Vu(),n={};l&&(n.client_location=l);void 0===f&&(f=em());void 0===g&&(g=h.resolve(pw));Zv(k,g,f,n);Gr(h,{nc:ew,Bd:Yv.h});qw=h.resolve(ew)}Aw()})});
D("yt.abuse.player.botguardInitialized",E("yt.abuse.player.botguardInitialized")||nx);D("yt.abuse.player.invokeBotguard",E("yt.abuse.player.invokeBotguard")||ox);D("yt.abuse.dclkstatus.checkDclkStatus",E("yt.abuse.dclkstatus.checkDclkStatus")||Hw);D("yt.player.exports.navigate",E("yt.player.exports.navigate")||Lu);D("yt.util.activity.init",E("yt.util.activity.init")||lr);D("yt.util.activity.getTimeSinceActive",E("yt.util.activity.getTimeSinceActive")||or);
D("yt.util.activity.setTimestamp",E("yt.util.activity.setTimestamp")||mr);window.addEventListener("load",dl(function(){Nx()}));
window.addEventListener("pageshow",dl(function(a){a.persisted||Nx()}));
window.addEventListener("pagehide",dl(function(a){U("embeds_web_enable_dispose_player_if_page_not_cached_killswitch")?Rx():a.persisted||Rx()}));
window.onerror=function(a,b,c,d,e){b=void 0===b?"Unknown file":b;c=void 0===c?0:c;var f=!1,g=al("log_window_onerror_fraction");if(g&&Math.random()<g)f=!0;else{g=document.getElementsByTagName("script");for(var h=0,k=g.length;h<k;h++)if(0<g[h].src.indexOf("/debug-")){f=!0;break}}f&&(f=!1,e?f=!0:("string"===typeof a?g=a:ErrorEvent&&a instanceof ErrorEvent?(f=!0,g=a.message,b=a.filename,c=a.lineno,d=a.colno):(g="Unknown error",b="Unknown file",c=0),e=new V(g),e.name="UnhandledWindowError",e.message=g,
e.fileName=b,e.lineNumber=c,isNaN(d)?delete e.columnNumber:e.columnNumber=d),f?Zs(e):$s(e))};
je=at;window.addEventListener("unhandledrejection",function(a){at(a.reason)});
Db(T("ERRORS")||[],function(a){Zs.apply(null,a)});
$k("ERRORS",[]);}).call(this);
