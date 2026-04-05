#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <ArduinoJson.h>
#include <ESP32Servo.h>
#include <DNSServer.h>

// ═══════════════════════════════════════════════════════════════
// CONFIGURATION — Change these constants to tune the system
// ═══════════════════════════════════════════════════════════════

// ── WiFi Access Point Credentials ──────────────────────────────
const char* AP_SSID     = "CanalMonitor";
const char* AP_PASSWORD = "12345678";

// ── Pin Assignments ──────────────────────────────────────────────
const int SERVO_PIN    = 13;
const int TRIG_PIN     = 14;
const int ECHO_PIN     = 12;
const int WL_VCC_PIN   = 33;
const int WL_DATA_PIN  = 34;   // MUST be ADC1 pin (WiFi blocks ADC2)
const int LED_GREEN    = 25;
const int LED_YELLOW   = 26;
const int LED_RED      = 27;
const int BUZZER_PIN   = 32;

// ── Detection & Radar Settings ───────────────────────────────────
const float MAX_DETECTION_RANGE_CM = 20.0;

const int SWEEP_MIN  = 25;
const int SWEEP_MAX  = 155;
const int SWEEP_STEP = 1;
const int SWEEP_MARGIN = 3;  // Skip first/last 3° to avoid servo jitter on reversal
const unsigned long STEP_INTERVAL_MS = 100;

const float OBSTRUCT_THRESH  = 2.0;   // cm difference from baseline to flag obstruction
const float DEPTH_ELEVATED   = 10.0;  // cm water depth for ELEVATED
const float DEPTH_CRITICAL   = 15.0;  // cm water depth for CRITICAL
const float VARIANCE_THRESH  = 3.0;   // variance threshold for waste movement detection
const float MEAN_DELTA_THRESH = 1.5;  // avg delta from baseline to flag stationary object
const int   OBSTRUCTION_HOLD = 5;    // Hold obstruction for 5 steps (0.5 seconds)
const int   WL_READ_INTERVAL = 10;   // Read water level every 10 steps (1 second)

// ── Clog Detection Settings ──────────────────────────────────
const int   HISTORY_DEPTH       = 2;     // Readings per angle to track (activates after ~3 sweeps)
const float STATIC_VAR_THRESH   = 2.0;   // Temporal variance threshold (tolerant of sensor noise)
const float STATIC_DELTA_THRESH = 2.0;   // Minimum delta from baseline to be significant
const int   CLOG_ANGLE_COUNT    = 3;     // Adjacent static angles to flag clog

const float SIMULATED_WATER_DEPTH = 0.0;
const float WL_EMA_ALPHA = 0.8;

// ═══════════════════════════════════════════════════════════════
// STATE VARIABLES
// ═══════════════════════════════════════════════════════════════
const int   BASELINE_STEPS = (SWEEP_MAX - SWEEP_MIN) / SWEEP_STEP + 1;
float       baseline[BASELINE_STEPS];
bool        baselineReady = false;

const int BUF_SIZE = 12;
float     buf[BUF_SIZE];
int       bIdx    = 0;
bool      bufFull = false;

const int DEBOUNCE_ESCALATE   = 2;
const int DEBOUNCE_DEESCALATE = 2;



enum Status { NORMAL, ELEVATED, WASTE, CRITICAL };

Servo  radarServo;
int    sweepAngle          = SWEEP_MIN;
int    sweepDir            = 1;
int    stepCount           = 0;
float  waterDepth          = SIMULATED_WATER_DEPTH;
float  currentDist         = -1.0;
bool   obstructionDetected = false;
int    obstructionTimer    = 0;
Status confirmedStatus     = NORMAL;
Status candidateStatus     = NORMAL;
int    candidateCount      = 0;
unsigned long lastStepTime = 0;

// ── Clog Detection State ─────────────────────────────────────
float angleHistory[BASELINE_STEPS][HISTORY_DEPTH];
int   angleHistCount[BASELINE_STEPS];
bool  isClogged           = false;

// ── Server Instances ─────────────────────────────────────────────
AsyncWebServer server(80);
AsyncWebSocket ws("/ws");
DNSServer dnsServer;

// ═══════════════════════════════════════════════════════════════
// HTML INTERFACE (PROGMEM)
// ═══════════════════════════════════════════════════════════════
const char INDEX_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0">
<title>Canal Flood Monitor</title>
<style>
  *{margin:0;padding:0;box-sizing:border-box}
  body{
    background:#070d13;color:#e2ecf3;
    font-family:system-ui, -apple-system, sans-serif;
    display:flex;flex-direction:column;
    align-items:center;min-height:100vh;padding:15px;
  }
  h1{color:#00e5ff;font-size:20px;letter-spacing:4px;margin:15px 0;text-transform:uppercase;text-align:center;text-shadow:0 0 10px rgba(0,229,255,0.3)}
  #header{display:flex;align-items:center;justify-content:center;gap:10px;margin-bottom:20px;font-size:12px;font-weight:600}
  #main{display:flex;gap:20px;flex-wrap:wrap;justify-content:center;width:100%;max-width:1200px}
  canvas#radar{border:1px solid rgba(0,180,216,0.3);border-radius:8px;background:#050E18;width:100%;max-width:800px;box-shadow:0 8px 32px rgba(0,0,0,0.4);display:block;aspect-ratio:800/460}
  #panel{background:#111e2b;border:1px solid rgba(0,180,216,0.3);border-radius:8px;padding:20px;flex:1;min-width:300px;max-width:360px;box-shadow:0 8px 32px rgba(0,0,0,0.3)}
  .ptitle{color:#00B4D8;font-size:12px;font-weight:700;letter-spacing:2px;margin-bottom:15px;text-align:center}
  .sbox{border-radius:6px;padding:15px;margin-bottom:15px;border:1px solid currentColor;transition:all .3s ease;text-align:center;background:rgba(0,0,0,0.2)}
  .sbox.critical{animation:critPulse 1s infinite}
  @keyframes critPulse{0%,100%{box-shadow:0 0 10px rgba(231,76,60,0.5)}50%{box-shadow:0 0 25px rgba(231,76,60,0.9)}}
  .sbox.clogged{animation:clogPulse 1.5s infinite}
  @keyframes clogPulse{0%,100%{box-shadow:0 0 10px rgba(211,84,0,0.5)}50%{box-shadow:0 0 25px rgba(211,84,0,0.9)}}
  .slbl{font-size:11px;letter-spacing:1px;opacity:.7;margin-bottom:5px;font-weight:600}
  .sval{font-size:22px;font-weight:800;letter-spacing:1px}
  
  #metrics{display:grid;grid-template-columns:1fr 1fr;gap:12px;margin-bottom:15px}
  .metric-card{background:#0a131c;border:1px solid rgba(0,180,216,0.15);border-radius:6px;padding:12px 5px;display:flex;flex-direction:column;align-items:center;justify-content:center}
  .metric-card.full{grid-column:1 / -1}
  .m-lbl{font-size:10px;color:#8AAFC8;letter-spacing:1px;margin-bottom:4px;font-weight:600}
  .m-val{font-size:15px;font-weight:bold;color:#e2ecf3;font-family:'Courier New',monospace}
  
  #log{background:#050a0f;border-radius:6px;padding:10px;font-size:11px;height:110px;overflow-y:auto;font-family:'Courier New',monospace;border:1px solid rgba(0,180,216,0.15)}
  .le{color:#6A8FAA;padding:3px 0;border-bottom:1px solid rgba(255,255,255,0.03)}.le.ch{color:#00e5ff;font-weight:bold}
  #cd{display:inline-block;width:10px;height:10px;border-radius:50%;background:#E74C3C;animation:pulse 1.5s infinite}
  @keyframes pulse{0%,100%{opacity:1}50%{opacity:.3}}
  #cd.live{background:#2ECC71;animation:none;box-shadow:0 0 8px #2ECC71}
  #graphs{display:flex;gap:20px;flex-wrap:wrap;justify-content:center;margin-top:20px;width:100%;max-width:1200px}
  .gb{background:#111e2b;border:1px solid rgba(0,180,216,0.3);border-radius:8px;padding:15px;flex:1;min-width:300px;box-shadow:0 8px 32px rgba(0,0,0,0.3)}
  .gt{font-size:11px;color:#8AAFC8;font-weight:600;letter-spacing:1px;margin-bottom:10px}
  canvas.ch{width:100%;height:80px;background:#050a0f;border-radius:4px;border:1px solid rgba(0,180,216,0.1);display:block}
  #banner{background:linear-gradient(135deg,#132230,#0d1822);border:1px solid rgba(0,180,216,0.4);border-radius:8px;padding:12px 18px;margin-bottom:20px;font-size:13px;display:flex;align-items:center;justify-content:space-between;gap:15px;max-width:1200px;width:100%;animation:fadeIn .5s ease;box-shadow:0 4px 15px rgba(0,0,0,0.4)}
  @keyframes fadeIn{from{opacity:0;transform:translateY(-10px)}to{opacity:1;transform:translateY(0)}}
  #banner .bt{color:#e2ecf3;line-height:1.5}
  #banner .bt b{color:#00e5ff}
  #banner .bx{background:rgba(255,255,255,0.05);border:1px solid rgba(255,255,255,0.1);color:#e2ecf3;cursor:pointer;border-radius:4px;padding:4px 10px;font-size:16px;transition:all .2s ease}
  #banner .bx:hover{background:rgba(231,76,60,0.2);border-color:#E74C3C;color:#E74C3C}
  #ab{background:rgba(0,180,216,0.1);border:1px solid #00B4D8;color:#00e5ff;padding:6px 14px;border-radius:6px;cursor:pointer;font-family:inherit;font-weight:700;font-size:11px;letter-spacing:1px;transition:all .3s ease;margin-left:10px;text-transform:uppercase}
  #ab:hover{background:rgba(0,180,216,0.25);box-shadow:0 0 12px rgba(0,180,216,0.4)}
  #ab.on{background:rgba(46,204,113,0.15);border-color:#2ECC71;color:#2ECC71;box-shadow:none;cursor:default}
  .hidden{display:none!important}
</style>
</head>
<body>
<h1>CANAL FLOOD AND WASTE RADAR</h1>
<div id="header"><span id="cd"></span><span id="cl">Connecting...</span><button id="ab" onclick="enableAudio()">ENABLE ALERTS</button></div>
<div id="banner">
  <div class="bt">For uninterrupted monitoring, open your full browser and visit <b>http://192.168.4.1</b></div>
  <button class="bx" onclick="this.parentElement.classList.add('hidden')">&times;</button>
</div>

<div id="main">
  <canvas id="radar"></canvas>
  <div id="panel">
    <div class="ptitle">SYSTEM STATUS</div>
    <div class="sbox" id="sb"><div class="slbl">CONFIRMED STATUS</div><div class="sval" id="sv">---</div></div>
    <div id="metrics">
      <div class="metric-card"><div class="m-lbl">ANGLE</div><div class="m-val" id="ra">---</div></div>
      <div class="metric-card"><div class="m-lbl">DISTANCE</div><div class="m-val" id="rd">---</div></div>
      <div class="metric-card"><div class="m-lbl">DEPTH</div><div class="m-val" id="rp">---</div></div>
      <div class="metric-card"><div class="m-lbl">VARIANCE</div><div class="m-val" id="rr">---</div></div>

    </div>
    <div id="log"></div>
  </div>
</div>

<div id="graphs">
  <div class="gb"><div class="gt">WATER DEPTH HISTORY (cm)</div><canvas id="dg" class="ch" width="400" height="80"></canvas></div>
  <div class="gb"><div class="gt">VARIANCE HISTORY</div><canvas id="vg" class="ch" width="400" height="80"></canvas></div>
</div>

<script>
var MD=20,SM=25,SX=155,SS=SX-SM+1,MSA=400;
var rc=document.getElementById('radar'),cx=rc.getContext('2d');

var dpr=window.devicePixelRatio||1;
var CX,CY,R;

function initRadar(){
  var w=rc.clientWidth,h=rc.clientHeight;
  if(w<100)w=800; if(h<60)h=460;
  rc.width=Math.round(w*dpr);rc.height=Math.round(h*dpr);
  cx.setTransform(dpr,0,0,dpr,0,0);
  CX=w/2;CY=h-30;R=Math.min(CX-20,CY-20);
}
initRadar();
var sw=SM;

// Per-angle scan memory
var sDist=new Float32Array(SS).fill(-1);
var sAge=new Float32Array(SS).fill(9999);
var sObs=new Uint8Array(SS).fill(0);
var sConf=new Array(SS).fill('Normal');

// Sweep trail
var trail=[];
var TMAX=30;

var sys={angle:90,dist:-1,depth:0,variance:0,obstr:false,confirmed:'Normal',clogged:false};
var lastC='';
var logE=document.getElementById('log');
var HN=120,dH=new Float32Array(HN),vH=new Float32Array(HN),hI=0;

function sCol(s){
  if(s==='Normal')return'#2ECC71';
  if(s==='Elevated')return'#F1C40F';
  if(s==='Waste Detected')return'#E67E22';
  if(s==='Waste Detected (Clogged)')return'#D35400';
  if(s==='Critical Flood Risk')return'#E74C3C';
  return'#6A8FAA';
}
function sRGB(s){
  if(s==='Normal')return[46,204,113];
  if(s==='Elevated')return[241,196,15];
  if(s==='Waste Detected')return[230,126,34];
  if(s==='Waste Detected (Clogged)')return[211,84,0];
  if(s==='Critical Flood Risk')return[231,76,60];
  return[106,143,170];
}
function aLog(m,h){
  var d=new Date();
  var t=('0'+d.getHours()).slice(-2)+':'+('0'+d.getMinutes()).slice(-2)+':'+('0'+d.getSeconds()).slice(-2);
  var e=document.createElement('div');
  e.className='le'+(h?' ch':'');
  e.textContent='['+t+'] '+m;
  logE.prepend(e);
  while(logE.children.length>25)logE.removeChild(logE.lastChild);
}

function toRad(a){
  var s=((a-SM)/(SX-SM))*180;
  return(180-s)*Math.PI/180;
}
function toXY(a,d){
  var r=toRad(a),f=Math.min(d/MD,1);
  return{x:CX+Math.cos(r)*R*f,y:CY-Math.sin(r)*R*f};
}

function drawRadar(){
  var w=rc.width/dpr,h=rc.height/dpr;
  cx.clearRect(0,0,w,h);

  // Background semicircle
  cx.fillStyle='#050E18';
  cx.beginPath();cx.arc(CX,CY,R,Math.PI,2*Math.PI);cx.fill();

  // Range rings
  var rings=[.25,.5,.75,1];
  for(var i=0;i<4;i++){
    var rr=R*rings[i];
    cx.beginPath();cx.arc(CX,CY,rr,Math.PI,2*Math.PI);
    cx.strokeStyle='rgba(0,200,120,0.25)';cx.lineWidth=.5;cx.stroke();
    cx.fillStyle='rgba(0,255,120,0.6)';cx.font='11px Courier New';cx.textAlign='left';
    cx.fillText((MD*rings[i]).toFixed(0)+'cm',CX+4,CY-rr+14);
  }

  // Radial spokes
  cx.textAlign='center';
  for(var d=30;d<=150;d+=30){
    var p=toXY(d,MD);
    cx.beginPath();cx.moveTo(CX,CY);cx.lineTo(p.x,p.y);
    cx.strokeStyle='rgba(0,200,120,0.25)';cx.lineWidth=.5;cx.stroke();
    var lp=toXY(d,MD*1.12);
    cx.fillStyle='rgba(0,255,120,0.7)';cx.font='11px Courier New';
    cx.fillText(d+'\u00b0',lp.x,lp.y);
  }

  // Baseline
  cx.beginPath();cx.moveTo(CX-R-8,CY);cx.lineTo(CX+R+8,CY);
  cx.strokeStyle='rgba(0,200,120,0.4)';cx.lineWidth=1;cx.stroke();

  // ── GREEN FADING WAKE (recently scanned clear areas) ──
  for(var i=0;i<SS;i++){
    if(sAge[i]>=MSA)continue;
    var alpha=(1-sAge[i]/MSA)*0.18;
    if(alpha<0.01)continue;
    var ang=SM+i;
    var tp=toXY(ang,MD);
    cx.beginPath();cx.moveTo(CX,CY);cx.lineTo(tp.x,tp.y);
    cx.strokeStyle='rgba(0,250,120,'+alpha.toFixed(3)+')';
    cx.lineWidth=1.5;cx.stroke();
  }

  // ── DETECTION BLIPS (objects in range, colored by status) ──
  for(var i=0;i<SS;i++){
    if(sAge[i]>=MSA)continue;
    var dd=sDist[i];
    if(dd<=0||dd>MD)continue;
    var alpha=Math.max(0,(1-sAge[i]/MSA));
    if(alpha<0.02)continue;
    var ang=SM+i;
    var rgb=sRGB(sConf[i]);
    var op=toXY(ang,dd);
    var ep=toXY(ang,MD);

    // Line from object to edge
    cx.beginPath();cx.moveTo(op.x,op.y);cx.lineTo(ep.x,ep.y);
    cx.strokeStyle='rgba('+rgb[0]+','+rgb[1]+','+rgb[2]+','+(alpha*0.4).toFixed(3)+')';
    cx.lineWidth=2;cx.stroke();

    // Detection dot
    if(alpha>0.08){
      cx.beginPath();cx.arc(op.x,op.y,2+alpha*2,0,2*Math.PI);
      cx.fillStyle='rgba('+rgb[0]+','+rgb[1]+','+rgb[2]+','+(alpha*0.85).toFixed(3)+')';
      cx.fill();
    }
  }

  // ── SWEEP TRAIL (fading wake behind sweep arm) ──
  for(var t=0;t<trail.length;t++){
    var a=((t+1)/trail.length)*0.3;
    var r2=toRad(trail[t]);
    var tx=CX+Math.cos(r2)*R;
    var ty=CY-Math.sin(r2)*R;
    cx.beginPath();cx.moveTo(CX,CY);cx.lineTo(tx,ty);
    cx.strokeStyle='rgba(0,250,120,'+a.toFixed(4)+')';
    cx.lineWidth=1.5;cx.stroke();
  }

  // ── MAIN SWEEP ARM with glow ──
  sw+=(sys.angle-sw)*0.3;
  var mr=toRad(sw),mx=CX+Math.cos(mr)*R,my=CY-Math.sin(mr)*R;

  cx.beginPath();cx.moveTo(CX,CY);cx.lineTo(mx,my);
  cx.strokeStyle='rgba(0,250,120,0.1)';cx.lineWidth=8;cx.stroke();
  cx.beginPath();cx.moveTo(CX,CY);cx.lineTo(mx,my);
  cx.strokeStyle='rgba(0,250,120,0.2)';cx.lineWidth=5;cx.stroke();
  cx.beginPath();cx.moveTo(CX,CY);cx.lineTo(mx,my);
  cx.strokeStyle='rgba(0,250,120,0.35)';cx.lineWidth=3;cx.stroke();
  cx.beginPath();cx.moveTo(CX,CY);cx.lineTo(mx,my);
  cx.strokeStyle='rgba(0,255,120,0.9)';cx.lineWidth=1.5;cx.stroke();

  // Arc border
  cx.beginPath();cx.arc(CX,CY,R,Math.PI,2*Math.PI);
  cx.strokeStyle='#0077A8';cx.lineWidth=1.5;cx.stroke();

  // Center dot
  cx.beginPath();cx.arc(CX,CY,5,0,2*Math.PI);
  cx.fillStyle='#00B4D8';cx.fill();

  // Status label on radar
  var sc=sCol(sys.confirmed);
  cx.fillStyle=sc;cx.font='bold 11px Courier New';cx.textAlign='left';
  cx.fillText(sys.confirmed,8,16);

}

function uPanel(){
  var c=sCol(sys.confirmed);
  var sb=document.getElementById('sb');
  sb.style.borderColor=c;sb.style.backgroundColor=c+'20';
  if(sys.confirmed==='Critical Flood Risk')sb.className='sbox critical';
  else if(sys.confirmed==='Waste Detected (Clogged)')sb.className='sbox clogged';
  else sb.className='sbox';
  var sv=document.getElementById('sv');
  sv.style.color=c;sv.textContent=sys.confirmed;

  document.getElementById('ra').textContent=sys.angle+'\u00b0';
  document.getElementById('rd').textContent=(sys.dist>0&&sys.dist<=MD)?sys.dist.toFixed(1)+' cm':'NO ECHO';
  document.getElementById('rp').textContent=sys.depth.toFixed(1)+' cm';
  document.getElementById('rr').textContent=sys.variance.toFixed(2);

  if(sys.confirmed!==lastC&&lastC!==''){
    aLog('STATUS: '+lastC+' -> '+sys.confirmed,true);
  }
  updateAlarm(sys.confirmed);
  lastC=sys.confirmed;
}

function dGraph(id,data,mv,th){
  var g=document.getElementById(id),c=g.getContext('2d');
  c.clearRect(0,0,g.width,g.height);
  for(var t=0;t<th.length;t++){
    var y=g.height-8-((th[t].v/mv)*(g.height-16));
    c.beginPath();c.moveTo(0,y);c.lineTo(g.width,y);
    c.strokeStyle=th[t].c+'50';c.lineWidth=1;c.stroke();
    c.fillStyle=th[t].c;c.font='9px Courier New';
    c.textAlign='right';c.fillText(th[t].l,g.width-2,y-3);
  }
  c.beginPath();
  for(var i=0;i<HN;i++){
    var hi=(hI-HN+i+HN*10)%HN;
    var v=Math.min(data[hi],mv);
    var px=(i/(HN-1))*g.width;
    var py=g.height-8-(v/mv)*(g.height-16);
    i===0?c.moveTo(px,py):c.lineTo(px,py);
  }
  c.strokeStyle='#00B4D8';c.lineWidth=2;c.stroke();
}

function render(){
  for(var i=0;i<SS;i++)if(sAge[i]<MSA)sAge[i]++;
  drawRadar();
  dGraph('dg',dH,30,[{v:10,c:'#F1C40F',l:'ELEV 10'},{v:20,c:'#E74C3C',l:'CRIT 15'}]);
  dGraph('vg',vH,10,[{v:3,c:'#E67E22',l:'VAR 3.0'}]);
  requestAnimationFrame(render);
}

function connect(){
  var ws=new WebSocket('ws://'+location.host+'/ws');
  var dot=document.getElementById('cd'),lbl=document.getElementById('cl');

  ws.onopen=function(){dot.className='live';lbl.textContent='Live Data Connected';aLog('System online',true);};

  ws.onmessage=function(e){
    try{
      var p=JSON.parse(e.data);
      if(p.maxDist!==undefined)MD=p.maxDist;
      if(p.angle!==undefined)sys.angle=p.angle;
      if(p.dist!==undefined)sys.dist=p.dist;
      if(p.depth!==undefined)sys.depth=p.depth;
      if(p.variance!==undefined)sys.variance=p.variance;
      if(p.obstr!==undefined)sys.obstr=p.obstr;
      if(p.confirmed!==undefined)sys.confirmed=p.confirmed;
      if(p.clogged!==undefined)sys.clogged=p.clogged;

      // Update per-angle scan data with status
      var idx=sys.angle-SM;
      if(idx>=0&&idx<SS){
        sDist[idx]=sys.dist;
        sObs[idx]=sys.obstr?1:0;
        sConf[idx]=sys.confirmed;
        sAge[idx]=0;
      }

      // Sweep trail
      trail.push(sys.angle);
      if(trail.length>TMAX)trail.shift();

      dH[hI%HN]=sys.depth;
      vH[hI%HN]=sys.variance;
      hI++;

      uPanel();
    }catch(err){}
  };

  ws.onclose=function(){
    dot.className='';lbl.textContent='Reconnecting...';
    setTimeout(connect,2000);
  };
}

// ── Audio Alarm System ──────────────────────────────────────
var audioCtx=null,audioOn=false,alarmInt=null,lastAlarm='';

function enableAudio(){
  if(audioOn)return;
  audioCtx=new(window.AudioContext||window.webkitAudioContext)();
  audioOn=true;
  var b=document.getElementById('ab');
  b.textContent='ALERTS ACTIVE';b.className='on';
  playChime();
}

function playTone(f,dur,vol,type){
  if(!audioCtx||!audioOn)return;
  var o=audioCtx.createOscillator(),g=audioCtx.createGain();
  o.connect(g);g.connect(audioCtx.destination);
  o.frequency.value=f;o.type=type||'sine';g.gain.value=vol||0.12;
  o.start();
  g.gain.exponentialRampToValueAtTime(0.001,audioCtx.currentTime+dur/1000);
  o.stop(audioCtx.currentTime+dur/1000);
}

function playChime(){
  playTone(523,150,0.08,'sine');
  setTimeout(function(){playTone(659,150,0.08,'sine');},160);
  setTimeout(function(){playTone(784,200,0.08,'sine');},320);
}

function playElevated(){
  playTone(440,180,0.08,'sine');
  setTimeout(function(){playTone(554,180,0.08,'sine');},220);
}

function playWaste(){
  playTone(740,100,0.12,'triangle');
  setTimeout(function(){playTone(740,100,0.12,'triangle');},180);
  setTimeout(function(){playTone(880,140,0.12,'triangle');},360);
}

function playClogged(){
  playTone(900,180,0.13,'sawtooth');
  setTimeout(function(){playTone(700,180,0.13,'sawtooth');},250);
  setTimeout(function(){playTone(500,300,0.15,'sawtooth');},500);
}

function playCritical(){
  if(!audioCtx||!audioOn)return;
  var now=audioCtx.currentTime;
  var o=audioCtx.createOscillator(),g=audioCtx.createGain();
  var lfo=audioCtx.createOscillator(),lfoG=audioCtx.createGain();
  o.connect(g);g.connect(audioCtx.destination);
  lfo.connect(lfoG);lfoG.connect(o.frequency);
  o.frequency.value=800;o.type='sawtooth';
  lfo.frequency.value=8;lfoG.gain.value=400;
  g.gain.value=0.15;
  o.start(now);lfo.start(now);
  g.gain.exponentialRampToValueAtTime(0.001,now+0.9);
  o.stop(now+0.9);lfo.stop(now+0.9);
}

function updateAlarm(s){
  if(!audioOn)return;
  if(s===lastAlarm)return;
  lastAlarm=s;
  if(alarmInt){clearInterval(alarmInt);alarmInt=null;}
  if(s==='Elevated'){
    playElevated();alarmInt=setInterval(playElevated,5000);
  }else if(s==='Waste Detected'){
    playWaste();alarmInt=setInterval(playWaste,3000);
  }else if(s==='Waste Detected (Clogged)'){
    playClogged();alarmInt=setInterval(playClogged,2500);
  }else if(s==='Critical Flood Risk'){
    playCritical();alarmInt=setInterval(playCritical,2000);
  }
}

connect();
render();
var resizeTimer;
window.addEventListener('resize',function(){
  clearTimeout(resizeTimer);
  resizeTimer=setTimeout(function(){dpr=window.devicePixelRatio||1;initRadar();},200);
});
</script>
</body>
</html>
)rawliteral";

// ═══════════════════════════════════════════════════════════════
// SENSOR & SYSTEM LOGIC
// ═══════════════════════════════════════════════════════════════

float readUltrasonic() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  // Reduced timeout to 15000us (~250cm max) for faster sweep
  long duration = pulseIn(ECHO_PIN, HIGH, 15000);
  if (duration == 0) return -1.0;

  float distance = (duration * 0.0343) / 2.0;
  if (distance < 2.0) return 1.0;
  if (distance > 400.0) return -1.0;

  return distance;
}

float readUltrasonicMedian() {
  float r[3];
  for (int i = 0; i < 3; i++) {
    r[i] = readUltrasonic();
    if (i < 2) delay(8);
  }
  // Simple sort for 3 elements
  for (int i = 0; i < 2; i++) {
    for (int j = i + 1; j < 3; j++) {
      if (r[j] < r[i]) {
        float tmp = r[i];
        r[i] = r[j];
        r[j] = tmp;
      }
    }
  }
  // Median is r[1]
  if (r[1] < 0) return -1.0;

  // Require at least 2 of 3 readings to agree within 3cm
  bool p01 = (r[0] > 0 && r[1] > 0 && fabs(r[0] - r[1]) < 3.0);
  bool p12 = (r[1] > 0 && r[2] > 0 && fabs(r[1] - r[2]) < 3.0);
  if (!p01 && !p12) return -1.0;

  return r[1];
}

float readWaterLevel() {
  digitalWrite(WL_VCC_PIN, HIGH);
  delayMicroseconds(500);

  int raw = analogRead(WL_DATA_PIN);
  digitalWrite(WL_VCC_PIN, LOW);

  // Only reject truly invalid readings (no signal)
  if (raw <= 0) return -1.0;

  // Float mapping instead of integer map() for better precision
  float depth = (float)raw * 30.0 / 4095.0;
  if (depth < 0.0)  depth = 0.0;
  if (depth > 50.0) return -1.0;

  return depth;
}

void stepServo() {
  sweepAngle += sweepDir * SWEEP_STEP;
  if (sweepAngle >= SWEEP_MAX) {
    sweepAngle = SWEEP_MAX;
    sweepDir   = -1;
  } else if (sweepAngle <= SWEEP_MIN) {
    sweepAngle = SWEEP_MIN;
    sweepDir   = 1;
  }
  radarServo.write(sweepAngle);
}

void calibrateBaseline() {
  Serial.println(F("Calibrating baseline..."));
  delay(1000);

  int tempAngle = SWEEP_MIN;
  for (int i = 0; i < BASELINE_STEPS; i++) {
    radarServo.write(tempAngle);
    delay(80);

    // Use median for more reliable baseline
    float d = readUltrasonicMedian();
    baseline[i] = (d > 0) ? d : 50.0;

    Serial.printf("  Angle: %d  Baseline: %.1f cm\n", tempAngle, baseline[i]);
    tempAngle += SWEEP_STEP;
  }
  baselineReady = true;
  Serial.println(F("Baseline complete."));
}

bool isObstructed(int angle, float dist) {
  if (!baselineReady || dist < 0) return false;
  // Skip boundary angles where servo jitter causes unreliable readings
  if (angle <= SWEEP_MIN + SWEEP_MARGIN || angle >= SWEEP_MAX - SWEEP_MARGIN) return false;
  // Reading beyond detection range — can't reliably determine obstruction
  if (dist > MAX_DETECTION_RANGE_CM) return false;
  int idx = (angle - SWEEP_MIN) / SWEEP_STEP;
  if (idx < 0 || idx >= BASELINE_STEPS) return false;
  // Skip angles where baseline was far (>2x max range) — unreliable comparison
  if (baseline[idx] > MAX_DETECTION_RANGE_CM * 2.0) return false;
  return ((baseline[idx] - dist) >= OBSTRUCT_THRESH);
}

// Push the DELTA from baseline (not raw distance) to the variance buffer.
// This prevents different angles seeing different wall distances from
// inflating variance. Only actual changes from baseline cause variance.
void pushReading(float d, int angle) {
  if (d < 0 || d > MAX_DETECTION_RANGE_CM) return;
  if (!baselineReady) return;

  int idx = (angle - SWEEP_MIN) / SWEEP_STEP;
  if (idx < 0 || idx >= BASELINE_STEPS) return;

  float delta = fabs(baseline[idx] - d);
  // Cap delta to prevent extreme values from unreliable baselines
  if (delta > MAX_DETECTION_RANGE_CM) delta = MAX_DETECTION_RANGE_CM;
  buf[bIdx % BUF_SIZE] = delta;
  bIdx++;
  if (bIdx >= BUF_SIZE) bufFull = true;
}

float calcVariance() {
  if (!bufFull && bIdx < BUF_SIZE) return 0.0;
  float mean = 0.0;
  for (int i = 0; i < BUF_SIZE; i++) mean += buf[i];
  mean /= BUF_SIZE;
  float v = 0.0;
  for (int i = 0; i < BUF_SIZE; i++) v += pow(buf[i] - mean, 2);
  return v / BUF_SIZE;
}

// Mean delta: catches STATIONARY objects that differ from baseline
// (variance = 0 for consistent deltas, but mean delta is high)
float calcMeanDelta() {
  if (!bufFull && bIdx < BUF_SIZE) return 0.0;
  float sum = 0.0;
  for (int i = 0; i < BUF_SIZE; i++) sum += buf[i];
  return sum / BUF_SIZE;
}

// Track per-angle delta history for clog detection
void updateAngleHistory(int angle, float dist) {
  if (!baselineReady || dist < 0 || dist > MAX_DETECTION_RANGE_CM) return;
  // Skip boundary angles where servo jitter causes unreliable readings
  if (angle <= SWEEP_MIN + SWEEP_MARGIN || angle >= SWEEP_MAX - SWEEP_MARGIN) return;
  int idx = (angle - SWEEP_MIN) / SWEEP_STEP;
  if (idx < 0 || idx >= BASELINE_STEPS) return;
  float delta = fabs(baseline[idx] - dist);
  int hi = angleHistCount[idx] % HISTORY_DEPTH;
  angleHistory[idx][hi] = delta;
  if (angleHistCount[idx] < HISTORY_DEPTH * 100) angleHistCount[idx]++;
}

// Evaluate clog status at each sweep boundary
void checkClogStatus() {
  int consecutive = 0;
  int maxConsecutive = 0;

  for (int i = 0; i < BASELINE_STEPS; i++) {
    if (angleHistCount[i] < HISTORY_DEPTH) {
      consecutive = 0;
      continue;
    }
    // Calculate mean and temporal variance of recent deltas at this angle
    float mean = 0.0;
    for (int j = 0; j < HISTORY_DEPTH; j++) mean += angleHistory[i][j];
    mean /= HISTORY_DEPTH;

    float var = 0.0;
    for (int j = 0; j < HISTORY_DEPTH; j++) var += pow(angleHistory[i][j] - mean, 2);
    var /= HISTORY_DEPTH;

    // Static = consistently high delta with low variance (object not moving)
    if (var < STATIC_VAR_THRESH && mean > STATIC_DELTA_THRESH) {
      consecutive++;
      if (consecutive > maxConsecutive) maxConsecutive = consecutive;
    } else {
      consecutive = 0;
    }
  }

  isClogged = (maxConsecutive >= CLOG_ANGLE_COUNT);
  if (isClogged) {
    Serial.printf("[CLOG] Detected %d adjacent static angles\n", maxConsecutive);
  }
}

Status classify(float depth, float variance, float meanDelta, bool obstruction) {
  bool wasteFlag = variance > VARIANCE_THRESH || obstruction || meanDelta > MEAN_DELTA_THRESH;
  if (depth >= DEPTH_CRITICAL) return CRITICAL;
  if (depth >= DEPTH_ELEVATED && wasteFlag) return CRITICAL;
  if (wasteFlag) return WASTE;
  if (depth >= DEPTH_ELEVATED) return ELEVATED;
  return NORMAL;
}

void updateDebounce(Status raw) {
  if (raw == candidateStatus) {
    candidateCount++;
  } else {
    candidateStatus = raw;
    candidateCount  = 1;
  }

  int needed;
  if ((int)raw > (int)confirmedStatus) {
    // Instant escalation to Critical if already alerting (Waste/Elevated)
    if (raw == CRITICAL && confirmedStatus != NORMAL) {
      needed = 1;
    } else {
      needed = DEBOUNCE_ESCALATE;      // Quick to confirm higher severity
    }
  } else if ((int)raw < (int)confirmedStatus) {
    needed = DEBOUNCE_DEESCALATE;    // Slow to drop severity
  } else {
    needed = DEBOUNCE_ESCALATE;      // Same level
  }

  if (candidateCount >= needed) {
    Status prev = confirmedStatus;
    confirmedStatus = candidateStatus;
    candidateCount  = needed;

    // On any transition to NORMAL: flush all detection data for clean slate
    // Prevents stale history from causing false re-triggers
    if (confirmedStatus == NORMAL && prev != NORMAL) {
      for (int i = 0; i < BUF_SIZE; i++) buf[i] = 0.0;
      bIdx = 0;
      bufFull = false;
      memset(angleHistCount, 0, sizeof(angleHistCount));
      isClogged = false;
      obstructionTimer = 0;
      obstructionDetected = false;
    }
  }
}

void setOutput(Status s) {
  unsigned long now = millis();
  bool blinkState = (now / 500) % 2 == 0;

  // ── LEDs ──
  digitalWrite(LED_GREEN, (s == NORMAL) ? HIGH : LOW);

  if (s == WASTE && isClogged) {
    // Clogged: fast yellow blink + slow red blink
    bool fastBlink = (now / 200) % 2 == 0;
    digitalWrite(LED_YELLOW, fastBlink ? HIGH : LOW);
    digitalWrite(LED_RED, blinkState ? HIGH : LOW);
  } else if (s == WASTE) {
    digitalWrite(LED_YELLOW, blinkState ? HIGH : LOW);
    digitalWrite(LED_RED, LOW);
  } else {
    digitalWrite(LED_YELLOW, (s == ELEVATED) ? HIGH : LOW);
    digitalWrite(LED_RED, (s == CRITICAL) ? HIGH : LOW);
  }

  // ── Buzzer (passive — distinct patterns per status) ──
  if (s == NORMAL) {
    ledcWriteTone(BUZZER_PIN, 0);
    ledcWrite(BUZZER_PIN, 0);
  } else if (s == ELEVATED) {
    // Short chirp every 3 seconds
    unsigned long cycle = now % 3000;
    if (cycle < 100) {
      ledcWriteTone(BUZZER_PIN, 2500);
    } else {
      ledcWriteTone(BUZZER_PIN, 0);
      ledcWrite(BUZZER_PIN, 0);
    }
  } else if (s == WASTE && !isClogged) {
    // Double chirp every 2 seconds
    unsigned long cycle = now % 2000;
    if (cycle < 100 || (cycle > 200 && cycle < 300)) {
      ledcWriteTone(BUZZER_PIN, 3000);
    } else {
      ledcWriteTone(BUZZER_PIN, 0);
      ledcWrite(BUZZER_PIN, 0);
    }
  } else if (s == WASTE && isClogged) {
    // Rising warble every 1.5 seconds
    unsigned long cycle = now % 1500;
    if (cycle < 400) {
      int freq = 2000 + (int)((cycle / 400.0) * 1500.0);
      ledcWriteTone(BUZZER_PIN, freq);
    } else {
      ledcWriteTone(BUZZER_PIN, 0);
      ledcWrite(BUZZER_PIN, 0);
    }
  } else if (s == CRITICAL) {
    // Fast alternating siren (4500/3000 Hz at 120ms)
    bool phase = (now / 120) % 2 == 0;
    ledcWriteTone(BUZZER_PIN, phase ? 4500 : 3000);
  }
}

const char* statusLabel(Status s) {
  switch (s) {
    case NORMAL:   return "Normal";
    case ELEVATED: return "Elevated";
    case WASTE:    return isClogged ? "Waste Detected (Clogged)" : "Waste Detected";
    case CRITICAL: return "Critical Flood Risk";
    default:       return "Unknown";
  }
}

// ═══════════════════════════════════════════════════════════════
// WEBSERVER CONTROL
// ═══════════════════════════════════════════════════════════════

void onWsEvent(AsyncWebSocket* server, AsyncWebSocketClient* client,
               AwsEventType type, void* arg, uint8_t* data, size_t len) {
  if (type == WS_EVT_CONNECT) {
    Serial.printf("[WS] Client %u connected\n", client->id());
  } else if (type == WS_EVT_DISCONNECT) {
    Serial.printf("[WS] Client %u disconnected\n", client->id());
  }
}

void pushLiveData() {
  if (ws.count() == 0) return;
  ws.cleanupClients();

  JsonDocument doc;
  doc["maxDist"]   = MAX_DETECTION_RANGE_CM;
  doc["angle"]     = sweepAngle;
  doc["dist"]      = (currentDist > 0) ? currentDist : -1.0;
  doc["depth"]     = waterDepth;
  doc["variance"]  = calcVariance();
  doc["obstr"]     = obstructionDetected;
  doc["confirmed"] = statusLabel(confirmedStatus);
  doc["clogged"]   = isClogged;

  String payload;
  serializeJson(doc, payload);
  ws.textAll(payload);
}

// ═══════════════════════════════════════════════════════════════
// MAIN ARDUINO HOOKS
// ═══════════════════════════════════════════════════════════════

void setup() {
  Serial.begin(115200);

  Serial.printf("\nStarting WiFi AP: %s\n", AP_SSID);
  WiFi.softAP(AP_SSID, AP_PASSWORD);
  Serial.print("IP: ");
  Serial.println(WiFi.softAPIP());

  // Captive Portal: redirect all DNS queries to our IP
  dnsServer.start(53, "*", WiFi.softAPIP());
  Serial.println("DNS server started (captive portal).");

  ws.onEvent(onWsEvent);
  server.addHandler(&ws);
  server.on("/", HTTP_GET, [](AsyncWebServerRequest* request) {
    request->send_P(200, "text/html", INDEX_HTML);
  });

  // Captive portal detection endpoints (Android / iOS / Windows)
  server.on("/generate_204", HTTP_GET, [](AsyncWebServerRequest* request) {
    request->redirect("http://192.168.4.1/");
  });
  server.on("/hotspot-detect.html", HTTP_GET, [](AsyncWebServerRequest* request) {
    request->redirect("http://192.168.4.1/");
  });
  server.on("/connecttest.txt", HTTP_GET, [](AsyncWebServerRequest* request) {
    request->redirect("http://192.168.4.1/");
  });
  server.on("/fwlink", HTTP_GET, [](AsyncWebServerRequest* request) {
    request->redirect("http://192.168.4.1/");
  });

  // Catch-all: redirect any unknown URL to dashboard
  server.onNotFound([](AsyncWebServerRequest* request) {
    request->redirect("http://192.168.4.1/");
  });

  server.begin();
  Serial.println("Web server ready (captive portal active).");

  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);

  radarServo.setPeriodHertz(50);
  radarServo.attach(SERVO_PIN, 500, 2400);
  radarServo.write(SWEEP_MIN);

  pinMode(TRIG_PIN,   OUTPUT);
  pinMode(ECHO_PIN,   INPUT);
  pinMode(WL_VCC_PIN, OUTPUT);
  digitalWrite(WL_VCC_PIN, LOW);

  pinMode(LED_GREEN,  OUTPUT);
  pinMode(LED_YELLOW, OUTPUT);
  pinMode(LED_RED,    OUTPUT);

  ledcAttach(BUZZER_PIN, 2000, 8);

  for (int i = 0; i < BUF_SIZE; i++) buf[i] = 0.0;  // Zero delta = calm baseline
  memset(angleHistory, 0, sizeof(angleHistory));
  memset(angleHistCount, 0, sizeof(angleHistCount));
  setOutput(NORMAL);
  calibrateBaseline();
  lastStepTime = millis();
}

void loop() {
  dnsServer.processNextRequest();
  unsigned long now = millis();
  if (now - lastStepTime < STEP_INTERVAL_MS) {
    delay(1);
    return;
  }
  lastStepTime = now;

  int prevSweepDir = sweepDir;
  stepServo();
  currentDist = readUltrasonicMedian();

  // Detect sweep boundary (direction changed) — run clog analysis
  if (sweepDir != prevSweepDir) {
    checkClogStatus();
  }

  // Push delta-from-baseline for in-range readings; push zero for out-of-range
  if (currentDist > 0 && currentDist <= MAX_DETECTION_RANGE_CM) {
    pushReading(currentDist, sweepAngle);
    updateAngleHistory(sweepAngle, currentDist);
  } else if (!obstructionDetected) {
    // No object in range and no active obstruction — push zero delta
    // to naturally flush old detections from the buffer
    buf[bIdx % BUF_SIZE] = 0.0;
    bIdx++;
    if (bIdx >= BUF_SIZE) bufFull = true;
  }

  // Obstruction with hold timer — persists across a full sweep cycle
  if (isObstructed(sweepAngle, currentDist)) {
    obstructionTimer = OBSTRUCTION_HOLD;
  }
  if (obstructionTimer > 0) {
    obstructionTimer--;
    obstructionDetected = true;
  } else {
    obstructionDetected = false;
  }


  // Read water level frequently (~1s) or every step if already alerting
  int currentInterval = (confirmedStatus != NORMAL) ? 1 : WL_READ_INTERVAL;
  if (++stepCount >= currentInterval) {
    // Silence buzzer during ADC read to prevent PWM noise coupling
    ledcWriteTone(BUZZER_PIN, 0);
    ledcWrite(BUZZER_PIN, 0);
    delayMicroseconds(200);  // Let noise settle
    float rawDepth = readWaterLevel();
    if (rawDepth >= 0.0) {
      waterDepth = WL_EMA_ALPHA * rawDepth + (1.0 - WL_EMA_ALPHA) * waterDepth;
    }
    stepCount = 0;
  }

  // Classify and output
  float  variance  = calcVariance();
  float  meanDelta = calcMeanDelta();
  Status raw       = classify(waterDepth, variance, meanDelta, obstructionDetected);

  updateDebounce(raw);
  setOutput(confirmedStatus);

  pushLiveData();

  Serial.printf("A:%d D:%.1f Dp:%.1f V:%.2f M:%.2f O:%d C:%d S:%s\n",
                sweepAngle, currentDist, waterDepth, variance, meanDelta,
                obstructionDetected, isClogged, statusLabel(confirmedStatus));
}
