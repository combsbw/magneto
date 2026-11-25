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
  let i=js.indexOf('async function '+name+'(');
  if(i<0)i=js.indexOf('function '+name+'(');
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

(async()=>{
  console.log('--- sensor poller survives hung sockets, background sleep, and rapid tab switches ---');
  const vm=require('vm');
  let nowMs=10000,hidden=false,inFlightNow=0,maxInFlight=0,stateSeq=0,fetchMode='ok';
  const timers=new Map();let nextTid=1;
  const ctx={
    Date:{now:()=>nowMs},
    AbortController:class{constructor(){this.signal={};}abort(){}},
    Promise,Error,Object,Math,Number,JSON,
    setTimeout:(fn,ms)=>{const id=nextTid++;timers.set(id,{fn,at:nowMs+(ms||0)});return id;},
    clearTimeout:id=>{timers.delete(id);},
    document:{get hidden(){return hidden;}},
    TAB:'treat',ST:null,cfgRev:0,cfgPushing:0,
    clockBase:{t:0,at:0,run:false,total:0},
    SCOPE:{ch:0,ms:1000,hold:false,wave:null,fold:null,spec:null},
    UPD:{busy:false},
    DEV:{on:true,fails:0,polling:false,rebooting:false,net:'ap',fail:false},
    CONN:{busy:false,since:0,n:0,t:0},
    renderState:()=>{},renderMeasure:()=>{},renderPresets:()=>{},connRender:()=>{},
    tempRecord:()=>{},cfgPull:()=>{},scopeDraw:()=>{},foldDraw:()=>{},specDraw:()=>{},
    fetch:path=>{
      inFlightNow++;if(inFlightNow>maxInFlight)maxInFlight=inFlightNow;
      if(fetchMode==='hang')return new Promise(()=>{});      // simulates WebKit socket wedged across sleep
      const body=path.startsWith('/api/state')
        ?JSON.stringify({app:'magneto',s:'run',t:12,total:600,b0:100+stateSeq,tc:78.5+stateSeq*0.1,tf:80.2,okc:1,okf:1,seq:++stateSeq})
        :JSON.stringify({n:4,fs:10000,ms:1000,mn:[0,0,0,0],mx:[1,1,1,1]});
      return Promise.resolve({ok:true,status:200,text:()=>Promise.resolve(body),json:()=>Promise.resolve(JSON.parse(body))})
        .finally(()=>{inFlightNow--;});
    }
  };
  const flush=async()=>{for(let i=0;i<24;i++)await Promise.resolve();};
  const stepTo=async target=>{
    while(true){
      let soonest=null;
      for(const [id,t] of timers)if(t.at<=target&&(!soonest||t.at<soonest.at))soonest={id,at:t.at,fn:t.fn};
      if(!soonest)break;
      timers.delete(soonest.id);
      nowMs=Math.max(nowMs,soonest.at);
      soonest.fn();
      await flush();
    }
    nowMs=Math.max(nowMs,target);
    await flush();
  };
  const snippet=[
    grab('fetchOpts'),grab('api'),grab('hedged'),
    'const apiQuiet=(p,o)=>api(p,o).catch(()=>null);',
    'const BUSY={n:0,since:0};',
    grab('hiPri'),
    'async function devConnect(){return false;}',
    grab('kick'),
    'const POLL={timer:0,inFlight:false,wake:false,wakeMeasure:false,n:0,gen:0,startedAt:0,tickAt:0};',
    grab('pollWake'),grab('pollKick'),grab('measureKick'),grab('pollTick'),grab('pollWatchdog'),
    'this.__exp={api,hedged,hiPri,BUSY,POLL,kick,pollKick,measureKick,pollWatchdog};'
  ].join('\n');
  vm.runInNewContext(snippet,ctx);
  const E=ctx.__exp;

  // 1) Hung fetch that ignores AbortSignal still rejects on timeout and releases hiPri(BUSY.n)
  fetchMode='hang';
  let timedOut=false;
  E.hiPri(E.api('/api/cmd?c=tune',{timeout:500})).catch(()=>{timedOut=true;});
  ck(E.BUSY.n===1,'hiPri increments BUSY.n while request is in flight');
  await stepTo(nowMs+600);
  ck(timedOut,'api() rejects on timeout even when AbortController cannot wake fetch');
  ck(E.BUSY.n===0,'BUSY.n returns to 0 after timeout so polling is never blocked forever');

  let hedgedOut=false;
  E.hedged('/api/state',{timeout:400,gap:200,max:2}).catch(()=>{hedgedOut=true;});
  await stepTo(nowMs+1500);
  ck(hedgedOut,'hedged() also settles on hung sockets');
  inFlightNow=0;maxInFlight=0;fetchMode='ok';

  // 2) Hidden for minutes ("not read for a few minutes in the app") then foregrounded via kick()
  E.pollKick();
  await flush();
  const seqBeforeHide=stateSeq;
  ck(seqBeforeHide>=1&&ctx.ST&&ctx.ST.seq===seqBeforeHide,'initial poll populates ST');
  hidden=true;
  await stepTo(nowMs+180000);                       // 3 minutes hidden: no /api/state traffic
  ck(stateSeq===seqBeforeHide,'no /api/state polling while hidden');
  timers.clear();                                   // simulate mobile OS dropping background timers
  hidden=false;
  E.kick();                                         // visibilitychange / pageshow wake
  await flush();
  ck(stateSeq>seqBeforeHide&&ctx.ST.seq===stateSeq,'kick() immediately resumes sensor polling after backgrounding');

  // 3) Request wedged mid-flight across sleep (>6 s): pollWatchdog supersedes it and recovers
  fetchMode='hang';
  await stepTo(nowMs+400);                          // starts a pollTick whose fetch hangs
  timers.clear();                                   // simulate OS suspend freezing the timeout timer too
  nowMs+=10000;
  fetchMode='ok';
  inFlightNow=0;
  const seqBeforeWake=stateSeq;
  E.pollWatchdog();
  await flush();
  ck(stateSeq>seqBeforeWake,'pollWatchdog recovers from a request hung across device sleep');

  // 4) Rapid measureKick + pollKick bursts stay strictly single-flight
  maxInFlight=0;
  ctx.TAB='measure';
  for(let i=0;i<6;i++){E.measureKick();E.pollKick();}
  await stepTo(nowMs+1200);
  ck(maxInFlight<=1,'single-flight poller never has more than 1 request in flight (got '+maxInFlight+')');

  console.log(fail?fail+' failure(s)':'all slider & poller checks passed');
  process.exit(fail?1:0);
})();
