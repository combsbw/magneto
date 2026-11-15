// slider_test.js - the pulse-width slider's safety contract, checked against the page's own code.
//
//   node test/slider_test.js        (also run by "make test")
//
// It lifts periodUs / pwCapUs / pwFor / pwClamp straight out of ui/app.html - the same arithmetic as
// mg::periodUs / mg::pwCapUs / mg::pwFor in engine.h, which test/host_test.cpp checks on the C++ side -
// and asserts the three things the slider promises:
//   1. with a ceiling set, the top of the track IS the ceiling and the scale under it is even
//      (half way up is half the ceiling, a quarter up is a quarter of it);
//   2. no slider position, at any frequency, can store or drive a value above the ceiling;
//   3. the number on the slider is the number the coil is driven with - and a pulse is always
//      shorter than its own period, so no setting can hold the gate on continuously.
const fs=require('fs');
const src=fs.readFileSync(require('path').join(__dirname,'..','ui','app.html'),'utf8');
const js=src.match(/<script>([\s\S]*)<\/script>/)[1];
const grab=name=>{
  const i=js.indexOf('function '+name+'(');
  if(i<0)throw new Error('missing '+name);
  let d=0,j=js.indexOf('{',i);
  for(let k=j;k<js.length;k++){if(js[k]==='{')d++;else if(js[k]==='}'){d--;if(!d)return js.slice(i,k+1);}}
};
const clamp=(v,a,b)=>v<a?a:(v>b?b:v);
const HZ_MAX=250,PW_MIN=1,PW_MAX=500000;
eval(grab('periodUs'));eval(grab('pwCapUs'));eval(grab('pwFor'));eval(grab('pwClamp'));

let fail=0;
const ck=(c,m)=>{if(!c){console.log('FAIL '+m);fail++;}};

// normalize() clamps every stored stage with these as the page loads, so they have to be declared
// above it. As consts further down they were still in their temporal dead zone at that moment and
// the whole script threw before `cfg` existed - a blank page for anyone with a saved config.
console.log('--- the drive limits are declared before the code that uses them at load time ---');
ck(js.indexOf('const HZ_MAX=')>=0,'HZ_MAX/PW_MIN/PW_MAX are declared');
ck(js.indexOf('const HZ_MAX=')<js.indexOf('function normalize('),
   'the drive-limit consts come before normalize()');
ck(js.indexOf('const HZ_MAX=')<js.indexOf('let cfg=normalize('),
   'the drive-limit consts come before the cfg that normalize() builds at load');

// The slider maps 0..top linearly. top() for pulse width = min(ceiling, period cap).
const top=(hz,ceil)=>{const cap=pwCapUs(hz);if(cap<=0)return ceil>0?ceil:PW_MAX;return ceil>0?Math.min(ceil,cap):cap;};
const unfrac=(f,t)=>t*f;

console.log('--- the ceiling is the top of the slider, and the scale below it is even ---');
for(const ceil of [50,100,250,1000,2500,10000,50000]){
  for(const hz of [0.5,1,5,10,25,50]){               // frequencies whose period is well above the ceiling
    const t=top(hz,ceil);
    if(t!==ceil)continue;                            // only the cases where the ceiling is binding
    ck(Math.round(pwClamp(hz,unfrac(1,t),ceil))===ceil,`top of slider = ceiling (${ceil}µs @ ${hz}Hz)`);
    ck(Math.round(pwClamp(hz,unfrac(0.5,t),ceil))===Math.round(ceil/2),`midpoint = half the ceiling (${ceil})`);
    ck(Math.round(pwClamp(hz,unfrac(0.25,t),ceil))===Math.round(ceil/4),`quarter = a quarter of the ceiling (${ceil})`);
    ck(Math.round(pwClamp(hz,unfrac(0.75,t),ceil))===Math.round(ceil*0.75),`three quarters (${ceil})`);
    for(let i=0;i<=100;i++){
      const v=pwClamp(hz,unfrac(i/100,t),ceil);
      ck(v<=ceil,`no slider position exceeds the ceiling (${ceil} @ ${i}%)`);
      ck(Math.abs(v-Math.max(1,Math.round(ceil*i/100)))<=1,`position ${i}% = ${i}% of ceiling ${ceil}`);
    }
  }
}

console.log('--- nothing the page can store drives the coil above the ceiling, at any frequency ---');
for(const ceil of [0,25,250,1000,10000]){
  for(let hz=0.1;hz<=250;hz+=0.3){
    for(const asked of [1,37,250,999,5000,99999,500000]){
      const stored=pwClamp(hz,asked,ceil);
      const driven=pwFor(hz,stored,ceil);
      if(ceil>0)ck(driven<=ceil,`driven<=ceiling (${ceil}, ${hz}Hz, asked ${asked})`);
      ck(driven<periodUs(hz),`driven pulse shorter than its period (${hz}Hz, asked ${asked})`);
      ck(driven===stored||pwCapUs(hz)<stored,`stored value is what gets driven (${hz}Hz, ${asked})`);
    }
  }
}

console.log('--- raising the frequency cannot leave a too-wide pulse behind (the old DC path) ---');
let pw=pwClamp(10,10000,0);
ck(pw===10000,'10 ms at 10 Hz is fine');
pw=pwClamp(200,pw,0);
ck(pw===4999,'re-clamped to 4999 µs at 200 Hz, not 10000');
ck(pwFor(200,10000,0)===4999,'and the drive clamp agrees');

console.log(fail?fail+' failure(s)':'all slider checks passed');
process.exit(fail?1:0);
