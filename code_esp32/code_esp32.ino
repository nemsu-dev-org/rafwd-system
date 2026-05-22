#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <ArduinoJson.h>
#include <DNSServer.h>
const char* AP_SSID = "CanalMonitor";
const int SERVO_PIN    = 13;
const int TRIG_PIN     = 14;
const int ECHO_PIN     = 12;
const int WL_VCC_PIN   = 33;
const int WL_DATA_PIN  = 34;
const int LED_GREEN    = 25;
const int LED_YELLOW   = 26;
const int LED_RED      = 27;
const int BUZZER_PIN   = 32;
const float SENSOR_MAX_RANGE_CM  = 5.0;
const float WATER_SAFETY_MARGIN  = 0.5;
const float MIN_EFFECTIVE_RANGE  = 1.0;
const int SWEEP_MIN  = 35;
const int SWEEP_MAX  = 145;
const int SWEEP_STEP = 1;
const int SWEEP_MARGIN = 3;
const unsigned long STEP_INTERVAL_MS = 50;
const float OBSTRUCT_THRESH  = 2.0;
const float DEPTH_ELEVATED   = 2.0;
const float DEPTH_CRITICAL   = 4.0;
const float VARIANCE_THRESH  = 3.0;
const float MEAN_DELTA_THRESH = 1.5;
const int   OBSTRUCTION_HOLD = 5;
const int   WL_READ_INTERVAL = 10;
const int   HISTORY_DEPTH       = 2;
const float STATIC_VAR_THRESH   = 2.0;
const float STATIC_DELTA_THRESH = 2.0;
const int   CLOG_ANGLE_COUNT    = 3;
const float SIMULATED_WATER_DEPTH = 0.0;
const float WL_EMA_ALPHA = 0.8;
const int   BASELINE_STEPS = (SWEEP_MAX - SWEEP_MIN) / SWEEP_STEP + 1;
float       baseline[BASELINE_STEPS];
bool        baselineReady = false;
const int BUF_SIZE = 12;
float     buf[BUF_SIZE];
int       bIdx    = 0;
bool      bufFull = false;
const int DEBOUNCE_ESCALATE   = 2;
const int DEBOUNCE_DEESCALATE = 3;
enum Status { NORMAL, ELEVATED, WASTE, CRITICAL };
int    sweepAngle          = SWEEP_MIN;
int    sweepDir            = 1;
int    stepCount           = 0;
float  waterDepth          = SIMULATED_WATER_DEPTH;
float  effectiveRange      = SENSOR_MAX_RANGE_CM;
float  currentDist         = -1.0;
bool   obstructionDetected = false;
int    obstructionTimer    = 0;
Status confirmedStatus     = NORMAL;
Status candidateStatus     = NORMAL;
int    candidateCount      = 0;
unsigned long lastStepTime = 0;
float angleHistory[BASELINE_STEPS][HISTORY_DEPTH];
int   angleHistCount[BASELINE_STEPS];
bool  isClogged           = false;
AsyncWebServer server(80);
AsyncWebSocket ws("/ws");
DNSServer dnsServer;
const char INDEX_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html><html lang="en"><head><meta charset="UTF-8"><meta name="viewport" content="width=device-width, initial-scale=1.0"><title>Canal Flood Monitor</title><style>*{margin:0;padding:0;box-sizing:border-box}body{background:#070d13;color:#e2ecf3;font-family:system-ui, -apple-system, sans-serif;display:flex;flex-direction:column;align-items:center;min-height:100vh;padding:15px;}h1{color:#00e5ff;font-size:20px;letter-spacing:4px;margin:15px 0;text-transform:uppercase;text-align:center;text-shadow:0 0 10px rgba(0,229,255,0.3)}#header{display:flex;align-items:center;justify-content:center;gap:10px;margin-bottom:20px;font-size:12px;font-weight:600}#main{display:flex;gap:20px;flex-wrap:wrap;justify-content:center;width:100%;max-width:1200px}canvas#radar{border:1px solid rgba(0,180,216,0.3);border-radius:8px;background:#050E18;width:100%;max-width:800px;box-shadow:0 8px 32px rgba(0,0,0,0.4);display:block;aspect-ratio:800/460}#panel{background:#111e2b;border:1px solid rgba(0,180,216,0.3);border-radius:8px;padding:20px;flex:1;min-width:300px;max-width:360px;box-shadow:0 8px 32px rgba(0,0,0,0.3)}.ptitle{color:#00B4D8;font-size:12px;font-weight:700;letter-spacing:2px;margin-bottom:15px;text-align:center}.sbox{border-radius:6px;padding:15px;margin-bottom:15px;border:1px solid currentColor;transition:all .3s ease;text-align:center;background:rgba(0,0,0,0.2)}.sbox.critical{animation:critPulse 1s infinite}@keyframes critPulse{0%,100%{box-shadow:0 0 10px rgba(231,76,60,0.5)}50%{box-shadow:0 0 25px rgba(231,76,60,0.9)}}.sbox.clogged{animation:clogPulse 1.5s infinite}@keyframes clogPulse{0%,100%{box-shadow:0 0 10px rgba(211,84,0,0.5)}50%{box-shadow:0 0 25px rgba(211,84,0,0.9)}}.slbl{font-size:11px;letter-spacing:1px;opacity:.7;margin-bottom:5px;font-weight:600}.sval{font-size:22px;font-weight:800;letter-spacing:1px}#metrics{display:grid;grid-template-columns:1fr 1fr;gap:12px;margin-bottom:15px}.metric-card{background:#0a131c;border:1px solid rgba(0,180,216,0.15);border-radius:6px;padding:12px 5px;display:flex;flex-direction:column;align-items:center;justify-content:center}.metric-card.full{grid-column:1 / -1}.m-lbl{font-size:10px;color:#8AAFC8;letter-spacing:1px;margin-bottom:4px;font-weight:600}.m-val{font-size:15px;font-weight:bold;color:#e2ecf3;font-family:'Courier New',monospace}#log{background:#050a0f;border-radius:6px;padding:10px;font-size:11px;height:110px;overflow-y:auto;font-family:'Courier New',monospace;border:1px solid rgba(0,180,216,0.15)}.le{color:#6A8FAA;padding:3px 0;border-bottom:1px solid rgba(255,255,255,0.03)}.le.ch{color:#00e5ff;font-weight:bold}#cd{display:inline-block;width:10px;height:10px;border-radius:50%;background:#E74C3C;animation:pulse 1.5s infinite}@keyframes pulse{0%,100%{opacity:1}50%{opacity:.3}}#cd.live{background:#2ECC71;animation:none;box-shadow:0 0 8px #2ECC71}#graphs{display:flex;gap:20px;flex-wrap:wrap;justify-content:center;margin-top:20px;width:100%;max-width:1200px}.gb{background:#111e2b;border:1px solid rgba(0,180,216,0.3);border-radius:8px;padding:15px;flex:1;min-width:300px;box-shadow:0 8px 32px rgba(0,0,0,0.3)}.gt{font-size:11px;color:#8AAFC8;font-weight:600;letter-spacing:1px;margin-bottom:10px}canvas.ch{width:100%;height:80px;background:#050a0f;border-radius:4px;border:1px solid rgba(0,180,216,0.1);display:block}#banner{background:linear-gradient(135deg,#132230,#0d1822);border:1px solid rgba(0,180,216,0.4);border-radius:8px;padding:12px 18px;margin-bottom:20px;font-size:13px;display:flex;align-items:center;justify-content:space-between;gap:15px;max-width:1200px;width:100%;animation:fadeIn .5s ease;box-shadow:0 4px 15px rgba(0,0,0,0.4)}@keyframes fadeIn{from{opacity:0;transform:translateY(-10px)}to{opacity:1;transform:translateY(0)}}#banner .bt{color:#e2ecf3;line-height:1.5}#banner .bt b{color:#00e5ff}#banner .bx{background:rgba(255,255,255,0.05);border:1px solid rgba(255,255,255,0.1);color:#e2ecf3;cursor:pointer;border-radius:4px;padding:4px 10px;font-size:16px;transition:all .2s ease}#banner .bx:hover{background:rgba(231,76,60,0.2);border-color:#E74C3C;color:#E74C3C}#ab{background:rgba(0,180,216,0.1);border:1px solid #00B4D8;color:#00e5ff;padding:6px 14px;border-radius:6px;cursor:pointer;font-family:inherit;font-weight:700;font-size:11px;letter-spacing:1px;transition:all .3s ease;margin-left:10px;text-transform:uppercase}#ab:hover{background:rgba(0,180,216,0.25);box-shadow:0 0 12px rgba(0,180,216,0.4)}#ab.on{background:rgba(46,204,113,0.15);border-color:#2ECC71;color:#2ECC71;cursor:pointer}#ab.muted{background:rgba(231,76,60,0.12);border-color:#E74C3C;color:#E74C3C;cursor:pointer}#dlb{display:block;width:100%;margin-top:8px;padding:5px 12px;background:rgba(0,180,216,0.06);border:1px solid rgba(0,180,216,0.3);border-radius:5px;color:#8AAFC8;font-family:inherit;font-weight:700;font-size:10px;letter-spacing:1px;text-transform:uppercase;cursor:pointer;transition:all .3s}#dlb:hover{background:rgba(0,180,216,0.18);color:#00e5ff;border-color:#00B4D8}.hidden{display:none!important}</style>
</head>
<body><h1>CANAL FLOOD AND WASTE RADAR</h1><div id="header"><span id="cd"></span><span id="cl">Connecting...</span><button id="ab" onclick="toggleAudio()">ENABLE ALERTS</button></div><div id="banner"><div class="bt">For uninterrupted monitoring, open your full browser and visit <b>http://192.168.4.1</b></div><button class="bx" onclick="this.parentElement.classList.add('hidden')">&times;</button></div>
<div id="main"><canvas id="radar"></canvas><div id="panel"><div class="ptitle">SYSTEM STATUS</div><div class="sbox" id="sb"><div class="slbl">CONFIRMED STATUS</div><div class="sval" id="sv">---</div></div><div id="metrics"><div class="metric-card"><div class="m-lbl">ANGLE</div><div class="m-val" id="ra">---</div></div><div class="metric-card"><div class="m-lbl">DISTANCE</div><div class="m-val" id="rd">---</div></div><div class="metric-card"><div class="m-lbl">DEPTH</div><div class="m-val" id="rp">---</div></div><div class="metric-card"><div class="m-lbl">VARIANCE</div><div class="m-val" id="rr">---</div></div></div><div id="log"></div><button id="dlb" onclick="dlCSV()">&#11123; DOWNLOAD CSV</button></div></div>
<div id="graphs"><div class="gb"><div class="gt">WATER DEPTH HISTORY (cm)</div><canvas id="dg" class="ch" width="400" height="80"></canvas></div><div class="gb"><div class="gt">VARIANCE HISTORY</div><canvas id="vg" class="ch" width="400" height="80"></canvas></div></div>
<script>var MD=5.0,SM=35,SX=145,SS=SX-SM+1,MSA=200;var rc=document.getElementById('radar'),cx=rc.getContext('2d');var dpr=window.devicePixelRatio||1;var CX,CY,R;
function initRadar(){var w=rc.clientWidth,h=rc.clientHeight;if(w<100)w=800; if(h<60)h=460;rc.width=Math.round(w*dpr);rc.height=Math.round(h*dpr);cx.setTransform(dpr,0,0,dpr,0,0);CX=w/2;CY=h-30;R=Math.min(CX-20,CY-20);}initRadar();var sw=SM;var sDist=new Float32Array(SS).fill(-1);var sAge=new Float32Array(SS).fill(9999);var sObs=new Uint8Array(SS).fill(0);var sConf=new Array(SS).fill('Normal');var trail=[];var TMAX=30;var sys={angle:90,dist:-1,depth:0,variance:0,obstr:false,confirmed:'Normal',clogged:false};var lastC='';var logE=document.getElementById('log');var HN=120,dH=new Float32Array(HN),vH=new Float32Array(HN),hI=0;
function sCol(s){if(s==='Normal')return'#2ECC71';if(s==='Elevated')return'#F1C40F';if(s==='Waste Detected')return'#E67E22';if(s==='Waste Detected (Clogged)')return'#D35400';if(s==='Critical Flood Risk')return'#E74C3C';return'#6A8FAA';}
function sRGB(s){if(s==='Normal')return[46,204,113];if(s==='Elevated')return[241,196,15];if(s==='Waste Detected')return[230,126,34];if(s==='Waste Detected (Clogged)')return[211,84,0];if(s==='Critical Flood Risk')return[231,76,60];return[106,143,170];}
function aLog(m,h){var d=new Date();var t=('0'+d.getHours()).slice(-2)+':'+('0'+d.getMinutes()).slice(-2)+':'+('0'+d.getSeconds()).slice(-2);var e=document.createElement('div');e.className='le'+(h?' ch':'');e.textContent='['+t+'] '+m;logE.prepend(e);while(logE.children.length>25)logE.removeChild(logE.lastChild);}
function toRad(a){ var s=((a-SM)/(SX-SM))*180; return(180-s)*Math.PI/180; }
function toXY(a,d){ var r=toRad(a),f=Math.min(d/MD,1); return{x:CX+Math.cos(r)*R*f,y:CY-Math.sin(r)*R*f}; }
function drawRadar(){var w=rc.width/dpr,h=rc.height/dpr;cx.clearRect(0,0,w,h);cx.fillStyle='#050E18';cx.beginPath();cx.arc(CX,CY,R,Math.PI,2*Math.PI);cx.fill();var rings=[.25,.5,.75,1];for(var i=0;i<4;i++){var rr=R*rings[i];cx.beginPath();cx.arc(CX,CY,rr,Math.PI,2*Math.PI);cx.strokeStyle='rgba(0,200,120,0.25)';cx.lineWidth=.5;cx.stroke();cx.fillStyle='rgba(0,255,120,0.6)';cx.font='11px Courier New';cx.textAlign='left';cx.fillText((MD*rings[i]).toFixed(1)+'cm',CX+4,CY-rr+14);}cx.textAlign='center';for(var d=30;d<=150;d+=30){var p=toXY(d,MD);cx.beginPath();cx.moveTo(CX,CY);cx.lineTo(p.x,p.y);cx.strokeStyle='rgba(0,200,120,0.25)';cx.lineWidth=.5;cx.stroke();var lp=toXY(d,MD*1.12);cx.fillStyle='rgba(0,255,120,0.7)';cx.font='11px Courier New';cx.fillText(d+'\u00b0',lp.x,lp.y);}cx.beginPath();cx.moveTo(CX-R-8,CY);cx.lineTo(CX+R+8,CY);cx.strokeStyle='rgba(0,200,120,0.4)';cx.lineWidth=1;cx.stroke();for(var i=0;i<SS;i++){if(sAge[i]>=MSA)continue;var alpha=(1-sAge[i]/MSA)*0.18;if(alpha<0.01)continue;var ang=SM+i,tp=toXY(ang,MD);cx.beginPath();cx.moveTo(CX,CY);cx.lineTo(tp.x,tp.y);cx.strokeStyle='rgba(0,250,120,'+alpha.toFixed(3)+')';cx.lineWidth=1.5;cx.stroke();}for(var i=0;i<SS;i++){if(sAge[i]>=MSA)continue;var dd=sDist[i];if(dd<=0||dd>MD)continue;var alpha=Math.max(0,(1-sAge[i]/MSA));if(alpha<0.02)continue;var ang=SM+i,rgb=sRGB(sConf[i]);var op=toXY(ang,dd),ep=toXY(ang,MD);cx.beginPath();cx.moveTo(op.x,op.y);cx.lineTo(ep.x,ep.y);cx.strokeStyle='rgba('+rgb[0]+','+rgb[1]+','+rgb[2]+','+(alpha*0.4).toFixed(3)+')';cx.lineWidth=2;cx.stroke();if(alpha>0.08){cx.beginPath();cx.arc(op.x,op.y,2+alpha*2,0,2*Math.PI);cx.fillStyle='rgba('+rgb[0]+','+rgb[1]+','+rgb[2]+','+(alpha*0.85).toFixed(3)+')';cx.fill();}}for(var t=0;t<trail.length;t++){var a=((t+1)/trail.length)*0.3;var r2=toRad(trail[t]),tx=CX+Math.cos(r2)*R,ty=CY-Math.sin(r2)*R;cx.beginPath();cx.moveTo(CX,CY);cx.lineTo(tx,ty);cx.strokeStyle='rgba(0,250,120,'+a.toFixed(4)+')';cx.lineWidth=1.5;cx.stroke();}sw+=(sys.angle-sw)*0.45;var mr=toRad(sw),mx=CX+Math.cos(mr)*R,my=CY-Math.sin(mr)*R;cx.beginPath();cx.moveTo(CX,CY);cx.lineTo(mx,my);cx.strokeStyle='rgba(0,250,120,0.1)';cx.lineWidth=8;cx.stroke();cx.beginPath();cx.moveTo(CX,CY);cx.lineTo(mx,my);cx.strokeStyle='rgba(0,250,120,0.2)';cx.lineWidth=5;cx.stroke();cx.beginPath();cx.moveTo(CX,CY);cx.lineTo(mx,my);cx.strokeStyle='rgba(0,250,120,0.35)';cx.lineWidth=3;cx.stroke();cx.beginPath();cx.moveTo(CX,CY);cx.lineTo(mx,my);cx.strokeStyle='rgba(0,255,120,0.9)';cx.lineWidth=1.5;cx.stroke();cx.beginPath();cx.arc(CX,CY,R,Math.PI,2*Math.PI);cx.strokeStyle='#0077A8';cx.lineWidth=1.5;cx.stroke();cx.beginPath();cx.arc(CX,CY,5,0,2*Math.PI);cx.fillStyle='#00B4D8';cx.fill();var sc=sCol(sys.confirmed);cx.fillStyle=sc;cx.font='bold 11px Courier New';cx.textAlign='left';cx.fillText(sys.confirmed,8,16);}
function uPanel(){var c=sCol(sys.confirmed);var sb=document.getElementById('sb');sb.style.borderColor=c;sb.style.backgroundColor=c+'20';if(sys.confirmed==='Critical Flood Risk')sb.className='sbox critical';else if(sys.confirmed==='Waste Detected (Clogged)')sb.className='sbox clogged';else sb.className='sbox';var sv=document.getElementById('sv');sv.style.color=c;sv.textContent=sys.confirmed;document.getElementById('ra').textContent=sys.angle+'\u00b0';document.getElementById('rd').textContent=(sys.dist>0&&sys.dist<=MD)?sys.dist.toFixed(1)+' cm':'NO ECHO';document.getElementById('rp').textContent=sys.depth.toFixed(1)+' cm';document.getElementById('rr').textContent=sys.variance.toFixed(2);if(sys.confirmed!==lastC&&lastC!=='') aLog('STATUS: '+lastC+' -> '+sys.confirmed,true);updateAlarm(sys.confirmed);lastC=sys.confirmed;}
function dGraph(id,data,mv,th){var g=document.getElementById(id),c=g.getContext('2d');c.clearRect(0,0,g.width,g.height);for(var t=0;t<th.length;t++){var y=g.height-8-((th[t].v/mv)*(g.height-16));c.beginPath();c.moveTo(0,y);c.lineTo(g.width,y);c.strokeStyle=th[t].c+'50';c.lineWidth=1;c.stroke();c.fillStyle=th[t].c;c.font='9px Courier New';c.textAlign='right';c.fillText(th[t].l,g.width-2,y-3);}c.beginPath();for(var i=0;i<HN;i++){var hi=(hI-HN+i+HN*10)%HN;var v=Math.min(data[hi],mv);var px=(i/(HN-1))*g.width,py=g.height-8-(v/mv)*(g.height-16);i===0?c.moveTo(px,py):c.lineTo(px,py);}c.strokeStyle='#00B4D8';c.lineWidth=2;c.stroke();}
function render(){for(var i=0;i<SS;i++)if(sAge[i]<MSA)sAge[i]++;drawRadar();dGraph('dg',dH,6,[{v:2,c:'#F1C40F',l:'ELEV 2cm'},{v:4,c:'#E74C3C',l:'CRIT 4cm'}]);dGraph('vg',vH,10,[{v:3,c:'#E67E22',l:'VAR 3.0'}]);requestAnimationFrame(render);}
function connect(){var ws=new WebSocket('ws://'+location.host+'/ws');var dot=document.getElementById('cd'),lbl=document.getElementById('cl');ws.onopen=function(){dot.className='live';lbl.textContent='Live Data Connected';aLog('System online',true);};ws.onmessage=function(e){try{var p=JSON.parse(e.data);if(p.maxDist!==undefined)MD=p.maxDist;if(p.angle!==undefined)sys.angle=p.angle;if(p.dist!==undefined)sys.dist=p.dist;if(p.depth!==undefined)sys.depth=p.depth;if(p.variance!==undefined)sys.variance=p.variance;if(p.obstr!==undefined)sys.obstr=p.obstr;if(p.confirmed!==undefined)sys.confirmed=p.confirmed;if(p.clogged!==undefined)sys.clogged=p.clogged;var idx=sys.angle-SM;if(idx>=0&&idx<SS){sDist[idx]=sys.dist;sObs[idx]=sys.obstr?1:0;sConf[idx]=sys.confirmed;sAge[idx]=0;}trail.push(sys.angle);if(trail.length>TMAX)trail.shift();dH[hI%HN]=sys.depth;vH[hI%HN]=sys.variance;hI++;logCSV(p);uPanel();}catch(err){}};ws.onclose=function(){dot.className='';lbl.textContent='Reconnecting...';setTimeout(connect,2000);};}var audioCtx=null,audioOn=false,audioMuted=false,alarmInt=null,lastAlarm='',csvRecs=[],lastCSVTime=0;function logCSV(p){var n=Date.now();if(p.confirmed!==lastC||n-lastCSVTime>10000){csvRecs.push([new Date().toISOString(),p.confirmed||'Normal',sys.depth.toFixed(2),sys.variance.toFixed(2),p.clogged?1:0,sys.angle,sys.dist>0?sys.dist.toFixed(2):'NO_ECHO',sys.obstr?1:0]);lastCSVTime=n;}}function dlCSV(){if(!csvRecs.length){aLog('No data recorded yet',false);return;}var b='Timestamp,Status,Depth_cm,Variance,Clogged,Angle,Distance_cm,Obstruction\n'+csvRecs.map(function(r){return r.join(',');}).join('\n');var a=document.createElement('a');a.href=URL.createObjectURL(new Blob([b],{type:'text/csv'}));a.download='canal_'+new Date().toISOString().slice(0,10)+'.csv';a.click();aLog('CSV downloaded ('+csvRecs.length+' records)',false);}
function toggleAudio(){var b=document.getElementById('ab');if(!audioCtx){audioCtx=new(window.AudioContext||window.webkitAudioContext)();audioOn=true;audioMuted=false;b.textContent='MUTE ALERTS';b.className='on';playChime();}else{audioMuted=!audioMuted;if(audioMuted){clearInterval(alarmInt);alarmInt=null;lastAlarm='';b.textContent='UNMUTE ALERTS';b.className='muted';}else{b.textContent='MUTE ALERTS';b.className='on';}}}
function playTone(f,dur,vol,type){if(!audioCtx||!audioOn||audioMuted)return;var o=audioCtx.createOscillator(),g=audioCtx.createGain();o.connect(g);g.connect(audioCtx.destination);o.frequency.value=f;o.type=type||'sine';g.gain.value=vol||0.12;o.start();g.gain.exponentialRampToValueAtTime(0.001,audioCtx.currentTime+dur/1000);o.stop(audioCtx.currentTime+dur/1000);}
function playChime(){playTone(523,150,0.08,'sine');setTimeout(function(){playTone(659,150,0.08,'sine');},160);setTimeout(function(){playTone(784,200,0.08,'sine');},320);}
function playElevated(){playTone(550,300,0.09,'sine');}
function playWaste(){playTone(700,130,0.11,'sine');setTimeout(function(){playTone(700,130,0.11,'sine');},240);}
function playClogged(){playTone(880,110,0.13,'square');setTimeout(function(){playTone(770,110,0.13,'square');},200);setTimeout(function(){playTone(660,160,0.13,'square');},400);}
function playCritical(){if(!audioCtx||!audioOn||audioMuted)return;playTone(880,220,0.13,'sine');setTimeout(function(){playTone(660,220,0.13,'sine');},250);setTimeout(function(){playTone(880,220,0.13,'sine');},500);}
function updateAlarm(s){if(!audioOn||audioMuted)return;if(s===lastAlarm)return;lastAlarm=s;if(alarmInt){clearInterval(alarmInt);alarmInt=null;}if(s==='Elevated'){playElevated();alarmInt=setInterval(playElevated,5000);}else if(s==='Waste Detected'){playWaste();alarmInt=setInterval(playWaste,3000);}else if(s==='Waste Detected (Clogged)'){playClogged();alarmInt=setInterval(playClogged,2500);}else if(s==='Critical Flood Risk'){playCritical();alarmInt=setInterval(playCritical,1800);}}connect();render();var resizeTimer;window.addEventListener('resize',function(){clearTimeout(resizeTimer);resizeTimer=setTimeout(function(){dpr=window.devicePixelRatio||1;initRadar();},200);});</script>
</body>
</html>
)rawliteral";
float readUltrasonic() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  long duration = pulseIn(ECHO_PIN, HIGH, 6000);
  if (duration == 0) return -1.0;
  float distance = (duration * 0.0343) / 2.0;
  if (distance < 2.0) return 1.0;
  if (distance > 400.0) return -1.0;
  return distance;}
float readUltrasonicMedian() {
  float r[3];
  for (int i = 0; i < 3; i++) { r[i] = readUltrasonic(); if (i < 2) delay(4); }
  for (int i = 0; i < 2; i++)
    for (int j = i + 1; j < 3; j++)
      if (r[j] < r[i]) { float tmp = r[i]; r[i] = r[j]; r[j] = tmp; }
  if (r[1] < 0) return -1.0;
  bool p01 = (r[0] > 0 && r[1] > 0 && fabs(r[0] - r[1]) < 3.0);
  bool p12 = (r[1] > 0 && r[2] > 0 && fabs(r[1] - r[2]) < 3.0);
  if (!p01 && !p12) return -1.0;
  return r[1];}
float readWaterLevel() {
  digitalWrite(WL_VCC_PIN, HIGH);
  delayMicroseconds(500);
  int raw = analogRead(WL_DATA_PIN);
  digitalWrite(WL_VCC_PIN, LOW);
  if (raw <= 0) return -1.0;
  float depth = (float)raw * 30.0 / 4095.0;
  if (depth < 0.0)  depth = 0.0;
  if (depth > 50.0) return -1.0;
  return depth;}
void writeServo(int angle) {
  uint32_t duty = map(angle, 0, 180, 1638, 7864);
  ledcWrite(SERVO_PIN, duty);}
bool buzzerIsOn = false;
void buzzerOn() {
  if (!buzzerIsOn) {
    digitalWrite(BUZZER_PIN, HIGH);
    buzzerIsOn = true; }}
void buzzerOff() {
  if (buzzerIsOn) {
    digitalWrite(BUZZER_PIN, LOW);
    buzzerIsOn = false; }}
void stepServo() {
  sweepAngle += sweepDir * SWEEP_STEP;
  if (sweepAngle >= SWEEP_MAX) { sweepAngle = SWEEP_MAX; sweepDir = -1; }
  else if (sweepAngle <= SWEEP_MIN) { sweepAngle = SWEEP_MIN; sweepDir = 1; }
  writeServo(sweepAngle);}
void calibrateBaseline() {
  Serial.println(F("Calibrating baseline..."));
  delay(1000);
  int tempAngle = SWEEP_MIN;
  for (int i = 0; i < BASELINE_STEPS; i++) {
    writeServo(tempAngle);
    delay(80);
    float d = readUltrasonicMedian();
    baseline[i] = (d > 0) ? d : 50.0;
    Serial.printf("  Angle: %d  Baseline: %.1f cm\n", tempAngle, baseline[i]);
    tempAngle += SWEEP_STEP;}
  baselineReady = true;
  Serial.println(F("Baseline complete."));}
bool isObstructed(int angle, float dist) {
  if (!baselineReady || dist < 0) return false;
  if (angle <= SWEEP_MIN + SWEEP_MARGIN || angle >= SWEEP_MAX - SWEEP_MARGIN) return false;
  if (dist > effectiveRange) return false;
  int idx = (angle - SWEEP_MIN) / SWEEP_STEP;
  if (idx < 0 || idx >= BASELINE_STEPS) return false;
  if (baseline[idx] > SENSOR_MAX_RANGE_CM * 2.0) return false;
  return ((baseline[idx] - dist) >= OBSTRUCT_THRESH);}
void pushReading(float d, int angle) {
  if (d < 0 || d > effectiveRange || !baselineReady) return;
  int idx = (angle - SWEEP_MIN) / SWEEP_STEP;
  if (idx < 0 || idx >= BASELINE_STEPS) return;
  float delta = fabs(baseline[idx] - d);
  if (delta > effectiveRange) delta = effectiveRange;
  buf[bIdx % BUF_SIZE] = delta;
  bIdx++;
  if (bIdx >= BUF_SIZE) bufFull = true;}
float calcVariance() {
  if (!bufFull && bIdx < BUF_SIZE) return 0.0;
  float mean = 0.0;
  for (int i = 0; i < BUF_SIZE; i++) mean += buf[i];
  mean /= BUF_SIZE;
  float v = 0.0;
  for (int i = 0; i < BUF_SIZE; i++) v += pow(buf[i] - mean, 2);
  return v / BUF_SIZE;}
float calcMeanDelta() {
  if (!bufFull && bIdx < BUF_SIZE) return 0.0;
  float sum = 0.0;
  for (int i = 0; i < BUF_SIZE; i++) sum += buf[i];
  return sum / BUF_SIZE;}
void updateAngleHistory(int angle, float dist) {
  if (!baselineReady || dist < 0 || dist > effectiveRange) return;
  if (angle <= SWEEP_MIN + SWEEP_MARGIN || angle >= SWEEP_MAX - SWEEP_MARGIN) return;
  int idx = (angle - SWEEP_MIN) / SWEEP_STEP;
  if (idx < 0 || idx >= BASELINE_STEPS) return;
  float delta = fabs(baseline[idx] - dist);
  int hi = angleHistCount[idx] % HISTORY_DEPTH;
  angleHistory[idx][hi] = delta;
  if (angleHistCount[idx] < HISTORY_DEPTH * 100) angleHistCount[idx]++;}
void checkClogStatus() {
  int consecutive = 0, maxConsecutive = 0;
  for (int i = 0; i < BASELINE_STEPS; i++) {
    if (angleHistCount[i] < HISTORY_DEPTH) { consecutive = 0; continue; }
    float mean = 0.0;
    for (int j = 0; j < HISTORY_DEPTH; j++) mean += angleHistory[i][j];
    mean /= HISTORY_DEPTH;
    float var = 0.0;
    for (int j = 0; j < HISTORY_DEPTH; j++) var += pow(angleHistory[i][j] - mean, 2);
    var /= HISTORY_DEPTH;
    if (var < STATIC_VAR_THRESH && mean > STATIC_DELTA_THRESH) {
      consecutive++;
      if (consecutive > maxConsecutive) maxConsecutive = consecutive;
    } else consecutive = 0;}
  isClogged = (maxConsecutive >= CLOG_ANGLE_COUNT);
  if (isClogged) Serial.printf("[CLOG] Detected %d adjacent static angles\n", maxConsecutive);}
Status classify(float depth, float variance, float meanDelta, bool obstruction) {
  bool wasteFlag = variance > VARIANCE_THRESH || obstruction || meanDelta > MEAN_DELTA_THRESH;
  if (depth >= DEPTH_CRITICAL) return CRITICAL;
  if (depth >= DEPTH_ELEVATED && wasteFlag) return CRITICAL;
  if (wasteFlag) return WASTE;
  if (depth >= DEPTH_ELEVATED) return ELEVATED;
  return NORMAL;}
void updateDebounce(Status raw) {
  if (raw == candidateStatus) candidateCount++;
  else { candidateStatus = raw; candidateCount = 1; }
  int needed;
  if ((int)raw > (int)confirmedStatus) needed = (raw == CRITICAL && confirmedStatus != NORMAL) ? 1 : DEBOUNCE_ESCALATE;
  else if ((int)raw < (int)confirmedStatus) needed = DEBOUNCE_DEESCALATE;
  else needed = DEBOUNCE_ESCALATE;
  if (candidateCount >= needed) {
    Status prev = confirmedStatus;
    confirmedStatus = candidateStatus;
    candidateCount  = needed;
    if (confirmedStatus == NORMAL && prev != NORMAL) {
      for (int i = 0; i < BUF_SIZE; i++) buf[i] = 0.0;
      bIdx = 0; bufFull = false;
      memset(angleHistCount, 0, sizeof(angleHistCount));
      isClogged = false; obstructionTimer = 0; obstructionDetected = false;}}}
void setOutput(Status s) {
  unsigned long now = millis();
  bool blinkState = (now / 500) % 2 == 0;
  digitalWrite(LED_GREEN, (s == NORMAL) ? HIGH : LOW);
  if (s == WASTE && isClogged) {
    bool flicker = (now / 100) % 2 == 0;
    digitalWrite(LED_YELLOW, flicker ? HIGH : LOW);
    digitalWrite(LED_RED, LOW);
  } else if (s == WASTE) {
    digitalWrite(LED_YELLOW, blinkState ? HIGH : LOW);
    digitalWrite(LED_RED, LOW);
  } else {
    digitalWrite(LED_YELLOW, (s == ELEVATED) ? HIGH : LOW);
    digitalWrite(LED_RED, (s == CRITICAL) ? HIGH : LOW);}
  if (s == NORMAL) buzzerOff();
  else if (s == ELEVATED) {
    // Single short beep every 4s
    unsigned long cycle = now % 4000;
    if (cycle < 120) buzzerOn(); else buzzerOff();
  } else if (s == WASTE && !isClogged) {
    // Double-beep every 2s: beep-beep ... pause
    unsigned long cycle = now % 2000;
    if (cycle < 150 || (cycle >= 300 && cycle < 450)) buzzerOn(); else buzzerOff();
  } else if (s == WASTE && isClogged) {
    // Triple rapid-beep every 2s: beep-beep-beep ... pause
    unsigned long cycle = now % 2000;
    if (cycle < 100 || (cycle >= 200 && cycle < 300) || (cycle >= 400 && cycle < 500)) buzzerOn(); else buzzerOff();
  } else if (s == CRITICAL) {
    buzzerOn(); // Continuous solid tone
  }}
const char* statusLabel(Status s) {
  switch (s) {
    case NORMAL:   return "Normal";
    case ELEVATED: return "Elevated";
    case WASTE:    return isClogged ? "Waste Detected (Clogged)" : "Waste Detected";
    case CRITICAL: return "Critical Flood Risk";
    default:       return "Unknown";}}
void onWsEvent(AsyncWebSocket* server, AsyncWebSocketClient* client,
               AwsEventType type, void* arg, uint8_t* data, size_t len) {
  if (type == WS_EVT_CONNECT) Serial.printf("[WS] Client %u connected\n", client->id());
  else if (type == WS_EVT_DISCONNECT) Serial.printf("[WS] Client %u disconnected\n", client->id());}
void pushLiveData() {
  if (ws.count() == 0) return;
  ws.cleanupClients();
  JsonDocument doc;
  doc["maxDist"]   = effectiveRange;
  doc["angle"]     = sweepAngle;
  doc["dist"]      = (currentDist > 0) ? currentDist : -1.0;
  doc["depth"]     = waterDepth;
  doc["variance"]  = calcVariance();
  doc["obstr"]     = obstructionDetected;
  doc["confirmed"] = statusLabel(confirmedStatus);
  doc["clogged"]   = isClogged;
  String payload;
  serializeJson(doc, payload);
  ws.textAll(payload);}
void setup() {
  Serial.begin(115200);
  Serial.printf("\nStarting WiFi AP: %s\n", AP_SSID);
  WiFi.disconnect(true); WiFi.softAPdisconnect(true); delay(100); WiFi.mode(WIFI_AP);
  WiFi.softAP(AP_SSID);
  Serial.print("IP: "); Serial.println(WiFi.softAPIP());
  dnsServer.start(53, "*", WiFi.softAPIP());
  Serial.println("DNS server started (captive portal).");
  ws.onEvent(onWsEvent);
  server.addHandler(&ws);
  server.on("/", HTTP_GET, [](AsyncWebServerRequest* request) { request->send_P(200, "text/html", INDEX_HTML); });
  server.on("/generate_204", HTTP_GET, [](AsyncWebServerRequest* request) { request->redirect("http://192.168.4.1/"); });
  server.on("/hotspot-detect.html", HTTP_GET, [](AsyncWebServerRequest* request) { request->redirect("http://192.168.4.1/"); });
  server.on("/connecttest.txt", HTTP_GET, [](AsyncWebServerRequest* request) { request->redirect("http://192.168.4.1/"); });
  server.on("/fwlink", HTTP_GET, [](AsyncWebServerRequest* request) { request->redirect("http://192.168.4.1/"); });
  server.onNotFound([](AsyncWebServerRequest* request) { request->redirect("http://192.168.4.1/"); });
  server.begin();
  Serial.println("Web server ready (captive portal active).");
  ledcAttachChannel(SERVO_PIN, 50, 16, 0);
  writeServo(SWEEP_MIN);
  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  pinMode(WL_VCC_PIN, OUTPUT);
  digitalWrite(WL_VCC_PIN, LOW);
  pinMode(LED_GREEN, OUTPUT);
  pinMode(LED_YELLOW, OUTPUT);
  pinMode(LED_RED, OUTPUT);
  for (int i = 0; i < BUF_SIZE; i++) buf[i] = 0.0;
  memset(angleHistory, 0, sizeof(angleHistory));
  memset(angleHistCount, 0, sizeof(angleHistCount));
  setOutput(NORMAL);
  calibrateBaseline();
  lastStepTime = millis();}
void loop() {
  dnsServer.processNextRequest();
  unsigned long now = millis();
  if (now - lastStepTime < STEP_INTERVAL_MS) { delay(1); return; }
  lastStepTime = now;
  int prevSweepDir = sweepDir;
  stepServo();
  currentDist = readUltrasonicMedian();
  if (sweepDir != prevSweepDir) checkClogStatus();
  if (currentDist > 0 && currentDist <= effectiveRange) {
    pushReading(currentDist, sweepAngle);
    updateAngleHistory(sweepAngle, currentDist);
  } else if (!obstructionDetected) {
    buf[bIdx % BUF_SIZE] = 0.0; bIdx++;
    if (bIdx >= BUF_SIZE) bufFull = true;}
  if (isObstructed(sweepAngle, currentDist)) obstructionTimer = OBSTRUCTION_HOLD;
  if (obstructionTimer > 0) { obstructionTimer--; obstructionDetected = true; }
  else obstructionDetected = false;
  int currentInterval = (confirmedStatus != NORMAL) ? 1 : WL_READ_INTERVAL;
  if (++stepCount >= currentInterval) {
    buzzerOff();
    delayMicroseconds(200);
    float rawDepth = readWaterLevel();
    if (rawDepth >= 0.0) waterDepth = WL_EMA_ALPHA * rawDepth + (1.0 - WL_EMA_ALPHA) * waterDepth;
    float newRange = SENSOR_MAX_RANGE_CM - waterDepth - WATER_SAFETY_MARGIN;
    effectiveRange = max(MIN_EFFECTIVE_RANGE, newRange);
    stepCount = 0;}
  float  variance  = calcVariance();
  float  meanDelta = calcMeanDelta();
  Status raw       = classify(waterDepth, variance, meanDelta, obstructionDetected);
  updateDebounce(raw);
  setOutput(confirmedStatus);
  pushLiveData();
  Serial.printf("A:%d D:%.1f Dp:%.1f V:%.2f M:%.2f O:%d C:%d S:%s\n",
                sweepAngle, currentDist, waterDepth, variance, meanDelta,
                obstructionDetected, isClogged, statusLabel(confirmedStatus));}