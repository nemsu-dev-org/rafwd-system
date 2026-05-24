#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <esp_wifi.h>
#include <ArduinoJson.h>
#include <DNSServer.h>
const char* AP_SSID="CanalMonitor";
const int SERVO_PIN=13,TRIG_PIN=14,ECHO_PIN=12,WL_VCC_PIN=33,WL_DATA_PIN=34,LED_GREEN=25,LED_YELLOW=26,LED_RED=27,BUZZER_PIN=32;
const float SENSOR_HEIGHT_CM=9.0,CONTAINER_HALF_WIDTH=8.5,WATER_SAFETY_MARGIN=0.5,MIN_EFFECTIVE_RANGE=2.5;
const int SWEEP_MIN=47,SWEEP_MAX=133,SWEEP_STEP=1,SWEEP_MARGIN=5;
const unsigned long STEP_INTERVAL_MS=40,DWELL_INTERVAL_MS=120;
const float OBSTRUCT_THRESH=0.5,BASELINE_TOLERANCE=0.4;
const float DEPTH_ELEVATED=2.7,DEPTH_CRITICAL=3.5,WL_NOISE_FLOOR=0.3;
const float VARIANCE_THRESH=1.2,MEAN_DELTA_THRESH=1.0;
const int OBSTRUCTION_HOLD=13;
const int HISTORY_DEPTH=3;
const float STATIC_VAR_THRESH=0.4;
const float STATIC_DELTA_THRESH=0.8;
const int CLOG_ANGLE_COUNT=5;
const float WL_CHANGE_THRESH=0.5,WL_TRACE_LENGTH=4.0,WL_NOISE_FLOOR_RAW=0.3;
const int BASELINE_STEPS=(SWEEP_MAX-SWEEP_MIN)/SWEEP_STEP+1;
float baseline[BASELINE_STEPS];
bool baselineReady=false;
const int BUF_SIZE=6;
float buf[BUF_SIZE];
int bIdx=0;
bool bufFull=false;
const int DEBOUNCE_ESCALATE=5,DEBOUNCE_ESCALATE_WL=12,DEBOUNCE_DEESCALATE=30;
const unsigned long STATE_HOLD_TO_NORMAL_MS=6000,STATE_HOLD_BETWEEN_MS=4000;
enum Status{NORMAL,ELEVATED,WASTE,CRITICAL,CALIBRATING};
bool wasteActive=false,waterRising=false;
int wlCounter=0,sweepAngle=SWEEP_MIN,sweepDir=1;
float waterDepth=0.0,effectiveRange=SENSOR_HEIGHT_CM,currentDist=-1.0;
bool obstructionDetected=false;
int obstructionTimer=0;
Status confirmedStatus=NORMAL,candidateStatus=NORMAL;
int candidateCount=0;
unsigned long lastStepTime=0,stateConfirmedAt=0;
float mapAccumulator[BASELINE_STEPS];
int mapCounts[BASELINE_STEPS];
int calibrationPasses=0;
float lastCalibratedDepth=0.0;
float wlAtCalibration=0.0,currentWLRaw=0.0;
float angleHistory[BASELINE_STEPS][HISTORY_DEPTH];
int angleHistCount[BASELINE_STEPS];
bool isClogged=false;
int obstructedAngleMin=-1,obstructedAngleMax=-1;
AsyncWebServer server(80);
AsyncWebSocket ws("/ws");
DNSServer dnsServer;
const char INDEX_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html><html lang="en"><head><meta charset="UTF-8"><meta name="viewport" content="width=device-width, initial-scale=1.0"><title>Canal Flood Monitor</title><style>*{margin:0;padding:0;box-sizing:border-box}body{background:#070d13;color:#e2ecf3;font-family:system-ui, -apple-system, sans-serif;display:flex;flex-direction:column;align-items:center;min-height:100vh;padding:15px;}h1{color:#00e5ff;font-size:20px;letter-spacing:4px;margin:15px 0;text-transform:uppercase;text-align:center;text-shadow:0 0 10px rgba(0,229,255,0.3)}#header{display:flex;align-items:center;justify-content:center;gap:10px;margin-bottom:20px;font-size:12px;font-weight:600}#main{display:flex;gap:20px;flex-wrap:wrap;justify-content:center;width:100%;max-width:1200px}canvas#radar{border:1px solid rgba(0,180,216,0.3);border-radius:8px;background:#050E18;width:100%;max-width:800px;box-shadow:0 8px 32px rgba(0,0,0,0.4);display:block;aspect-ratio:800/460}#panel{background:#111e2b;border:1px solid rgba(0,180,216,0.3);border-radius:8px;padding:20px;flex:1;min-width:300px;max-width:360px;box-shadow:0 8px 32px rgba(0,0,0,0.3)}.ptitle{color:#00B4D8;font-size:12px;font-weight:700;letter-spacing:2px;margin-bottom:15px;text-align:center}.sbox{border-radius:6px;padding:15px;margin-bottom:15px;border:1px solid currentColor;transition:all .3s ease;text-align:center;background:rgba(0,0,0,0.2)}.sbox.critical{animation:critPulse 1s infinite}@keyframes critPulse{0%,100%{box-shadow:0 0 10px rgba(231,76,60,0.5)}50%{box-shadow:0 0 25px rgba(231,76,60,0.9)}}.sbox.clogged{animation:clogPulse 1.5s infinite}@keyframes clogPulse{0%,100%{box-shadow:0 0 10px rgba(211,84,0,0.5)}50%{box-shadow:0 0 25px rgba(211,84,0,0.9)}}.slbl{font-size:11px;letter-spacing:1px;opacity:.7;margin-bottom:5px;font-weight:600}.sval{font-size:22px;font-weight:800;letter-spacing:1px}#metrics{display:grid;grid-template-columns:1fr 1fr;gap:12px;margin-bottom:15px}.metric-card{background:#0a131c;border:1px solid rgba(0,180,216,0.15);border-radius:6px;padding:12px 5px;display:flex;flex-direction:column;align-items:center;justify-content:center}.metric-card.full{grid-column:1 / -1}.m-lbl{font-size:10px;color:#8AAFC8;letter-spacing:1px;margin-bottom:4px;font-weight:600}.m-val{font-size:15px;font-weight:bold;color:#e2ecf3;font-family:'Courier New',monospace}#log{background:#050a0f;border-radius:6px;padding:10px;font-size:11px;height:110px;overflow-y:auto;font-family:'Courier New',monospace;border:1px solid rgba(0,180,216,0.15)}.le{color:#6A8FAA;padding:3px 0;border-bottom:1px solid rgba(255,255,255,0.03)}.le.ch{color:#00e5ff;font-weight:bold}#cd{display:inline-block;width:10px;height:10px;border-radius:50%;background:#E74C3C;animation:pulse 1.5s infinite}@keyframes pulse{0%,100%{opacity:1}50%{opacity:.3}}#cd.live{background:#2ECC71;animation:none;box-shadow:0 0 8px #2ECC71}#graphs{display:flex;gap:20px;flex-wrap:wrap;justify-content:center;margin-top:20px;width:100%;max-width:1200px}.gb{background:#111e2b;border:1px solid rgba(0,180,216,0.3);border-radius:8px;padding:15px;flex:1;min-width:300px;box-shadow:0 8px 32px rgba(0,0,0,0.3)}.gt{font-size:11px;color:#8AAFC8;font-weight:600;letter-spacing:1px;margin-bottom:10px}canvas.ch{width:100%;height:80px;background:#050a0f;border-radius:4px;border:1px solid rgba(0,180,216,0.1);display:block}#banner{background:linear-gradient(135deg,#132230,#0d1822);border:1px solid rgba(0,180,216,0.4);border-radius:8px;padding:12px 18px;margin-bottom:20px;font-size:13px;display:flex;align-items:center;justify-content:space-between;gap:15px;max-width:1200px;width:100%;animation:fadeIn .5s ease;box-shadow:0 4px 15px rgba(0,0,0,0.4)}@keyframes fadeIn{from{opacity:0;transform:translateY(-10px)}to{opacity:1;transform:translateY(0)}}#banner .bt{color:#e2ecf3;line-height:1.5}#banner .bt b{color:#00e5ff}#banner .bx{background:rgba(255,255,255,0.05);border:1px solid rgba(255,255,255,0.1);color:#e2ecf3;cursor:pointer;border-radius:4px;padding:4px 10px;font-size:16px;transition:all .2s ease}#banner .bx:hover{background:rgba(231,76,60,0.2);border-color:#E74C3C;color:#E74C3C}#ab{background:rgba(0,180,216,0.1);border:1px solid #00B4D8;color:#00e5ff;padding:6px 14px;border-radius:6px;cursor:pointer;font-family:inherit;font-weight:700;font-size:11px;letter-spacing:1px;transition:all .3s ease;margin-left:10px;text-transform:uppercase}#ab:hover{background:rgba(0,180,216,0.25);box-shadow:0 0 12px rgba(0,180,216,0.4)}#ab.on{background:rgba(46,204,113,0.15);border-color:#2ECC71;color:#2ECC71;cursor:pointer}#ab.muted{background:rgba(231,76,60,0.12);border-color:#E74C3C;color:#E74C3C;cursor:pointer}#dlb{display:block;width:100%;margin-top:8px;padding:5px 12px;background:rgba(0,180,216,0.06);border:1px solid rgba(0,180,216,0.3);border-radius:5px;color:#8AAFC8;font-family:inherit;font-weight:700;font-size:10px;letter-spacing:1px;text-transform:uppercase;cursor:pointer;transition:all .3s}#dlb:hover{background:rgba(0,180,216,0.18);color:#00e5ff;border-color:#00B4D8}.wbox{border-radius:6px;padding:12px;margin-bottom:15px;border:1px solid #6A8FAA;text-align:center;background:rgba(0,0,0,0.2);transition:all .3s ease}.wbox.wactive{border-color:#E67E22;background:rgba(230,126,34,0.1)}.wbox.wclogged{border-color:#D35400;background:rgba(211,84,0,0.15);animation:clogPulse 1.5s infinite}.hidden{display:none!important}</style>
</head>
<body><h1>CANAL FLOOD AND WASTE RADAR</h1><div id="header"><span id="cd"></span><span id="cl">Connecting...</span><button id="ab" onclick="toggleAudio()">ENABLE ALERTS</button></div><div id="banner"><div class="bt">For uninterrupted monitoring, open your full browser and visit <b>http://192.168.4.1</b></div><button class="bx" onclick="this.parentElement.classList.add('hidden')">&times;</button></div>
<div id="main"><canvas id="radar"></canvas><div id="panel"><div class="ptitle">SYSTEM STATUS</div><div class="sbox" id="sb"><div class="slbl">CONFIRMED STATUS</div><div class="sval" id="sv">---</div></div><div id="metrics"><div class="metric-card"><div class="m-lbl">ANGLE</div><div class="m-val" id="ra">---</div></div><div class="metric-card"><div class="m-lbl">DISTANCE</div><div class="m-val" id="rd">---</div></div><div class="metric-card"><div class="m-lbl">DEPTH</div><div class="m-val" id="rp">---</div></div><div class="metric-card"><div class="m-lbl">VARIANCE</div><div class="m-val" id="rr">---</div></div></div><div id="log"></div><button id="dlb" onclick="dlCSV()">&#11123; DOWNLOAD CSV</button></div></div>
<div id="graphs"><div class="gb"><div class="gt">WATER DEPTH HISTORY (cm)</div><canvas id="dg" class="ch" width="400" height="80"></canvas></div><div class="gb"><div class="gt">VARIANCE HISTORY</div><canvas id="vg" class="ch" width="400" height="80"></canvas></div></div>
<script>var MD=9.0,SM=47,SX=133,SS=SX-SM+1,MSA=200;var rc=document.getElementById('radar'),cx=rc.getContext('2d');var dpr=window.devicePixelRatio||1;var CX,CY,R;
function initRadar(){var w=rc.clientWidth,h=rc.clientHeight;if(w<100)w=800; if(h<60)h=460;rc.width=Math.round(w*dpr);rc.height=Math.round(h*dpr);cx.setTransform(dpr,0,0,dpr,0,0);CX=w/2;CY=h-30;R=Math.min(CX-20,CY-20);}initRadar();var sw=SM;var sDist=new Float32Array(SS).fill(-1);var sAge=new Float32Array(SS).fill(9999);var sObs=new Uint8Array(SS).fill(0);var sConf=new Array(SS).fill('Normal');var trail=[];var TMAX=60;var sys={angle:90,dist:-1,depth:0,variance:0,meanDelta:0,obstr:false,rawObs:false,confirmed:'Normal',clogged:false,waste:false};var lastC='';var logE=document.getElementById('log');var HN=120,dH=new Float32Array(HN),vH=new Float32Array(HN),hI=0;
function sCol(s){if(s==='Normal')return'#2ECC71';if(s==='Elevated')return'#F1C40F';if(s==='Waste Detected')return'#E67E22';if(s==='Waste Detected (Clogged)')return'#D35400';if(s==='Critical Flood Risk')return'#E74C3C';if(s==='CALIBRATING')return'#00e5ff';return'#6A8FAA';}
function sRGB(s){if(s==='Normal')return[46,204,113];if(s==='Elevated')return[241,196,15];if(s==='Waste Detected')return[230,126,34];if(s==='Waste Detected (Clogged)')return[211,84,0];if(s==='Critical Flood Risk')return[231,76,60];if(s==='CALIBRATING')return[0,229,255];return[106,143,170];}
function aLog(m,h){var d=new Date();var t=('0'+d.getHours()).slice(-2)+':'+('0'+d.getMinutes()).slice(-2)+':'+('0'+d.getSeconds()).slice(-2);var e=document.createElement('div');e.className='le'+(h?' ch':'');e.textContent='['+t+'] '+m;logE.prepend(e);while(logE.children.length>25)logE.removeChild(logE.lastChild);}
function toRad(a){ return a*Math.PI/180; }
function toXY(a,d){ var r=toRad(a),f=Math.min(d/MD,1); return{x:CX+Math.cos(r)*R*f,y:CY-Math.sin(r)*R*f}; }
function drawRadar(){var w=rc.width/dpr,h=rc.height/dpr;cx.clearRect(0,0,w,h);cx.fillStyle='#050E18';cx.beginPath();cx.arc(CX,CY,R,Math.PI,2*Math.PI);cx.fill();var rings=[.25,.5,.75,1];for(var i=0;i<4;i++){var rr=R*rings[i];cx.beginPath();cx.arc(CX,CY,rr,Math.PI,2*Math.PI);cx.strokeStyle='rgba(0,200,120,0.25)';cx.lineWidth=.5;cx.stroke();cx.fillStyle='rgba(0,255,120,0.6)';cx.font='11px Courier New';cx.textAlign='left';cx.fillText((MD*rings[i]).toFixed(1)+'cm',CX+4,CY-rr+14);}cx.textAlign='center';for(var d=0;d<=180;d+=45){var p=toXY(d,MD);cx.beginPath();cx.moveTo(CX,CY);cx.lineTo(p.x,p.y);cx.strokeStyle='rgba(0,200,120,0.25)';cx.lineWidth=.5;cx.stroke();var lp=toXY(d,MD*1.12);cx.fillStyle='rgba(0,255,120,0.7)';cx.font='11px Courier New';cx.fillText(d+'\u00b0',lp.x,lp.y);}cx.beginPath();cx.moveTo(CX-R-8,CY);cx.lineTo(CX+R+8,CY);cx.strokeStyle='rgba(0,200,120,0.4)';cx.lineWidth=1;cx.stroke();var clusters=[];var inCluster=false;var cStart=0;var last1=-1;for(var i=0;i<SS;i++){if(sObs[i]==1){if(!inCluster){inCluster=true;cStart=i;}last1=i;}else{if(inCluster&&(i-last1>5)){clusters.push({s:cStart,e:last1});inCluster=false;}}}if(inCluster)clusters.push({s:cStart,e:last1});var tNow=Date.now();for(var c=0;c<clusters.length;c++){var cl=clusters[c];var sumDist=0,count=0;var maxStatus='Waste Detected';for(var i=cl.s;i<=cl.e;i++){if(sDist[i]>0){sumDist+=sDist[i];count++;}if(sConf[i]=='Waste Detected (Clogged)'||sConf[i]=='Critical Flood Risk')maxStatus=sConf[i];}if(count==0)continue;var avgDist=sumDist/count;var centerAng=SM+(cl.s+cl.e)/2;var op=toXY(centerAng,avgDist);var beat=(tNow%1000)/1000;var pulse=0;if(beat<0.2)pulse=Math.sin(beat*Math.PI*5);else if(beat>0.3&&beat<0.5)pulse=Math.sin((beat-0.3)*Math.PI*5);var rad=4+2*Math.max(0,pulse);var rgb=sRGB(maxStatus);cx.beginPath();cx.arc(op.x,op.y,rad,0,2*Math.PI);cx.fillStyle='rgba('+rgb[0]+','+rgb[1]+','+rgb[2]+',1.0)';cx.fill();}for(var t=0;t<trail.length;t++){var a=((t+1)/trail.length)*0.3;var r2=toRad(trail[t]),tx=CX+Math.cos(r2)*R,ty=CY-Math.sin(r2)*R;cx.beginPath();cx.moveTo(CX,CY);cx.lineTo(tx,ty);cx.strokeStyle='rgba(0,250,120,'+a.toFixed(4)+')';cx.lineWidth=1.5;cx.stroke();}sw+=(sys.angle-sw)*0.60;trail.push(sw);if(trail.length>TMAX)trail.shift();var mr=toRad(sw),mx=CX+Math.cos(mr)*R,my=CY-Math.sin(mr)*R;cx.beginPath();cx.moveTo(CX,CY);cx.lineTo(mx,my);cx.strokeStyle='rgba(0,250,120,0.1)';cx.lineWidth=8;cx.stroke();cx.beginPath();cx.moveTo(CX,CY);cx.lineTo(mx,my);cx.strokeStyle='rgba(0,250,120,0.2)';cx.lineWidth=5;cx.stroke();cx.beginPath();cx.moveTo(CX,CY);cx.lineTo(mx,my);cx.strokeStyle='rgba(0,250,120,0.35)';cx.lineWidth=3;cx.stroke();cx.beginPath();cx.moveTo(CX,CY);cx.lineTo(mx,my);cx.strokeStyle='rgba(0,255,120,0.9)';cx.lineWidth=1.5;cx.stroke();cx.beginPath();cx.arc(CX,CY,R,Math.PI,2*Math.PI);cx.strokeStyle='#0077A8';cx.lineWidth=1.5;cx.stroke();cx.beginPath();cx.arc(CX,CY,5,0,2*Math.PI);cx.fillStyle='#00B4D8';cx.fill();var sc=sCol(sys.confirmed);cx.fillStyle=sc;cx.font='bold 11px Courier New';cx.textAlign='left';cx.fillText(sys.confirmed,8,16);}
function uPanel(){var c=sCol(sys.confirmed);var sb=document.getElementById('sb');sb.style.borderColor=c;sb.style.backgroundColor=c+'20';if(sys.confirmed==='Critical Flood Risk')sb.className='sbox critical';else if(sys.confirmed==='Waste Detected (Clogged)')sb.className='sbox clogged';else sb.className='sbox';var sv=document.getElementById('sv');sv.style.color=c;sv.textContent=sys.confirmed;document.getElementById('ra').textContent=sys.angle+'\u00b0';document.getElementById('rd').textContent=(sys.dist>0&&sys.dist<=MD)?sys.dist.toFixed(1)+' cm':'NO ECHO';document.getElementById('rp').textContent=sys.depth.toFixed(1)+' cm';document.getElementById('rr').textContent=sys.variance.toFixed(2);if(sys.confirmed!==lastC&&lastC!=='') aLog('STATUS: '+lastC+' -> '+sys.confirmed,true);updateAlarm(sys.confirmed);lastC=sys.confirmed;}
function dGraph(id,data,mv,th){var g=document.getElementById(id),c=g.getContext('2d');c.clearRect(0,0,g.width,g.height);for(var t=0;t<th.length;t++){var y=g.height-8-((th[t].v/mv)*(g.height-16));c.beginPath();c.moveTo(0,y);c.lineTo(g.width,y);c.strokeStyle=th[t].c+'50';c.lineWidth=1;c.stroke();c.fillStyle=th[t].c;c.font='9px Courier New';c.textAlign='right';c.fillText(th[t].l,g.width-2,y-3);}c.beginPath();for(var i=0;i<HN;i++){var hi=(hI-HN+i+HN*10)%HN;var v=Math.min(data[hi],mv);var px=(i/(HN-1))*g.width,py=g.height-8-(v/mv)*(g.height-16);i===0?c.moveTo(px,py):c.lineTo(px,py);}c.strokeStyle='#00B4D8';c.lineWidth=2;c.stroke();}
function render(){for(var i=0;i<SS;i++)if(sAge[i]<MSA)sAge[i]++;drawRadar();dGraph('dg',dH,5,[{v:2.7,c:'#F1C40F',l:'ELEV 2.7cm'},{v:3.5,c:'#E74C3C',l:'CRIT 3.5cm'}]);dGraph('vg',vH,10,[{v:4.0,c:'#E67E22',l:'VAR 4.0'}]);requestAnimationFrame(render);}
function connect(){var ws=new WebSocket('ws://'+location.host+'/ws');var dot=document.getElementById('cd'),lbl=document.getElementById('cl');ws.onopen=function(){dot.className='live';lbl.textContent='Live Data Connected';aLog('System online',true);};ws.onmessage=function(e){try{var p=JSON.parse(e.data);if(p.maxDist!==undefined)MD=p.maxDist;if(p.angle!==undefined)sys.angle=p.angle;if(p.dist!==undefined)sys.dist=p.dist;if(p.depth!==undefined)sys.depth=p.depth;if(p.variance!==undefined)sys.variance=p.variance;if(p.obstr!==undefined)sys.obstr=p.obstr;if(p.confirmed!==undefined)sys.confirmed=p.confirmed;if(p.clogged!==undefined)sys.clogged=p.clogged;if(p.waste!==undefined)sys.waste=p.waste;if(p.meanDelta!==undefined)sys.meanDelta=p.meanDelta;if(p.rawObs!==undefined)sys.rawObs=p.rawObs;var idx=sys.angle-SM;if(idx>=0&&idx<SS){sDist[idx]=sys.dist;sObs[idx]=sys.rawObs?1:0;sConf[idx]=sys.confirmed;sAge[idx]=0;}dH[hI%HN]=sys.depth;vH[hI%HN]=sys.variance;hI++;logCSV(p);uPanel();}catch(err){}};ws.onclose=function(){dot.className='';lbl.textContent='Reconnecting...';setTimeout(connect,2000);};}var audioCtx=null,audioOn=false,audioMuted=false,alarmInt=null,lastAlarm='',csvRecs=[],lastCSVTime=0;function logCSV(p){var n=Date.now();if(p.confirmed!==lastC||n-lastCSVTime>10000){csvRecs.push([new Date().toISOString(),p.confirmed||'Normal',sys.depth.toFixed(2),sys.variance.toFixed(2),sys.meanDelta.toFixed(2),p.clogged?1:0,sys.waste?1:0,sys.angle,sys.dist>0?sys.dist.toFixed(2):'NO_ECHO',sys.obstr?1:0,(p.effRange||0).toFixed(2)]);lastCSVTime=n;}}function dlCSV(){if(!csvRecs.length){aLog('No data recorded yet',false);return;}var b='Timestamp,Status,Depth_cm,Variance,MeanDelta,Clogged,Waste,Angle,Distance_cm,Obstruction,EffRange_cm\n'+csvRecs.map(function(r){return r.join(',');}).join('\n');var a=document.createElement('a');a.href=URL.createObjectURL(new Blob([b],{type:'text/csv'}));a.download='canal_'+new Date().toISOString().slice(0,10)+'.csv';a.click();aLog('CSV downloaded ('+csvRecs.length+' records)',false);}
function toggleAudio(){var b=document.getElementById('ab');if(!audioCtx){audioCtx=new(window.AudioContext||window.webkitAudioContext)();audioOn=true;audioMuted=false;b.textContent='MUTE ALERTS';b.className='on';playChime();}else{audioMuted=!audioMuted;if(audioMuted){clearInterval(alarmInt);alarmInt=null;lastAlarm='';b.textContent='UNMUTE ALERTS';b.className='muted';}else{b.textContent='MUTE ALERTS';b.className='on';}}}
function playTone(f,dur,vol,type){if(!audioCtx||!audioOn||audioMuted)return;var o=audioCtx.createOscillator(),g=audioCtx.createGain();o.connect(g);g.connect(audioCtx.destination);o.frequency.value=f;o.type=type||'sine';g.gain.value=vol||0.12;o.start();g.gain.exponentialRampToValueAtTime(0.001,audioCtx.currentTime+dur/1000);o.stop(audioCtx.currentTime+dur/1000);}
function playChime(){playTone(523,150,0.08,'sine');setTimeout(function(){playTone(659,150,0.08,'sine');},160);setTimeout(function(){playTone(784,200,0.08,'sine');},320);}
function playElevated(){playTone(550,300,0.09,'sine');}
function playWaste(){playTone(700,130,0.11,'sine');setTimeout(function(){playTone(700,130,0.11,'sine');},240);}
function playClogged(){playTone(880,110,0.13,'square');setTimeout(function(){playTone(770,110,0.13,'square');},200);setTimeout(function(){playTone(660,160,0.13,'square');},400);}
function playCritical(){if(!audioCtx||!audioOn||audioMuted)return;var o=audioCtx.createOscillator(),g=audioCtx.createGain();o.connect(g);g.connect(audioCtx.destination);o.type='sine';g.gain.setValueAtTime(0,audioCtx.currentTime);g.gain.linearRampToValueAtTime(0.13,audioCtx.currentTime+0.06);g.gain.setValueAtTime(0.13,audioCtx.currentTime+0.38);g.gain.linearRampToValueAtTime(0,audioCtx.currentTime+0.5);o.frequency.setValueAtTime(960,audioCtx.currentTime);o.frequency.linearRampToValueAtTime(620,audioCtx.currentTime+0.44);o.start();o.stop(audioCtx.currentTime+0.55);}
function updateAlarm(s){if(!audioOn||audioMuted)return;if(s===lastAlarm)return;lastAlarm=s;if(alarmInt){clearInterval(alarmInt);alarmInt=null;}if(s==='Elevated'){playElevated();alarmInt=setInterval(playElevated,5000);}else if(s==='Waste Detected'){playWaste();alarmInt=setInterval(playWaste,3000);}else if(s==='Waste Detected (Clogged)'){playClogged();alarmInt=setInterval(playClogged,2500);}else if(s==='Critical Flood Risk'){playCritical();alarmInt=setInterval(playCritical,1800);}}connect();render();var resizeTimer;window.addEventListener('resize',function(){clearTimeout(resizeTimer);resizeTimer=setTimeout(function(){dpr=window.devicePixelRatio||1;initRadar();},200);});
</script>
</body>
</html>
)rawliteral";
float readUltrasonic(){
  digitalWrite(TRIG_PIN,LOW);delayMicroseconds(2);
  digitalWrite(TRIG_PIN,HIGH);delayMicroseconds(10);
  digitalWrite(TRIG_PIN,LOW);
  long d=pulseIn(ECHO_PIN,HIGH,3000);
  if(!d) return -1.0;
  float dist=(d*0.0343)/2.0;
  return(dist<1.0||dist>400.0)?-1.0:dist;}
float readUltrasonicMedian(){
  float r[3];
  for(int i=0;i<3;i++){r[i]=readUltrasonic();if(i<2){delay(10);yield();}}
  for(int i=0;i<2;i++) for(int j=i+1;j<3;j++) if(r[j]<r[i]){float t=r[i];r[i]=r[j];r[j]=t;}
  if(r[1]<0) return -1.0;
  bool p01=(r[0]>0&&r[1]>0&&fabs(r[0]-r[1])<2.5),p12=(r[1]>0&&r[2]>0&&fabs(r[1]-r[2])<2.5);
  return(!p01&&!p12)?-1.0:r[1];}
float readWaterLevelRaw(){
  digitalWrite(WL_VCC_PIN,HIGH);delayMicroseconds(500);
  int r[5];for(int i=0;i<5;i++){r[i]=analogRead(WL_DATA_PIN);delayMicroseconds(200);}
  digitalWrite(WL_VCC_PIN,LOW);
  for(int i=0;i<4;i++)for(int j=i+1;j<5;j++)if(r[j]<r[i]){int t=r[i];r[i]=r[j];r[j]=t;}
  if(r[2]<=0)return -1.0;
  float depth=(r[2]/4095.0f)*WL_TRACE_LENGTH;
  return(depth<WL_NOISE_FLOOR_RAW)?0.0f:min(depth,5.0f);}
float calcAngleRange(int angle){
  float a=abs(angle-90)*DEG_TO_RAD,cosA=cos(a),sinA=sin(a);
  float beam=(sinA<0.02f)?SENSOR_HEIGHT_CM:min(SENSOR_HEIGHT_CM/cosA,CONTAINER_HALF_WIDTH/sinA);
  if(waterDepth<=0) return min(beam,SENSOR_HEIGHT_CM);
  return max(MIN_EFFECTIVE_RANGE,min(beam,(SENSOR_HEIGHT_CM-waterDepth)/max(cosA,0.1f)-WATER_SAFETY_MARGIN));}
void writeServo(int angle){ledcWrite(SERVO_PIN,(uint32_t)map(angle,0,180,1638,7864));}
bool buzzerIsOn=false;
void buzzerOn(){if(!buzzerIsOn){digitalWrite(BUZZER_PIN,HIGH);buzzerIsOn=true;}}
void buzzerOff(){if(buzzerIsOn){digitalWrite(BUZZER_PIN,LOW);buzzerIsOn=false;}}
void stepServo(){
  sweepAngle+=sweepDir*SWEEP_STEP;
  if(sweepAngle>=SWEEP_MAX){sweepAngle=SWEEP_MAX;sweepDir=-1;}
  else if(sweepAngle<=SWEEP_MIN){sweepAngle=SWEEP_MIN;sweepDir=1;}
  writeServo(sweepAngle);}
void startCalibration(){
  confirmedStatus=CALIBRATING;candidateStatus=CALIBRATING;candidateCount=0;calibrationPasses=0;
  memset(mapAccumulator,0,sizeof(mapAccumulator));memset(mapCounts,0,sizeof(mapCounts));
  memset(angleHistCount,0,sizeof(angleHistCount));
  for(int i=0;i<BUF_SIZE;i++) buf[i]=0.0;
  bIdx=0;bufFull=false;baselineReady=false;
  wasteActive=false;isClogged=false;obstructionDetected=false;
  obstructionTimer=0;obstructedAngleMin=-1;obstructedAngleMax=-1;
  waterDepth=0.0;waterRising=false;}
bool isObstructed(int angle,float dist){
  if(!baselineReady||dist<0||waterRising) return false;
  if(angle<=SWEEP_MIN+SWEEP_MARGIN||angle>=SWEEP_MAX-SWEEP_MARGIN) return false;
  int idx=(angle-SWEEP_MIN)/SWEEP_STEP;
  if(idx<0||idx>=BASELINE_STEPS) return false;
  if(dist>calcAngleRange(angle)) return false;
  return((baseline[idx]-dist)>=OBSTRUCT_THRESH);}
void pushReading(float d,int angle){
  if(d<0||d>effectiveRange||!baselineReady) return;
  if(angle<=SWEEP_MIN+SWEEP_MARGIN||angle>=SWEEP_MAX-SWEEP_MARGIN) return;
  int idx=(angle-SWEEP_MIN)/SWEEP_STEP;
  if(idx<0||idx>=BASELINE_STEPS) return;
  bool clean=fabs(baseline[idx]-d)<BASELINE_TOLERANCE;
  buf[bIdx%BUF_SIZE]=clean?0.0f:min(fabs(baseline[idx]-d),effectiveRange);
  if(++bIdx>=BUF_SIZE) bufFull=true;}
float calcVariance(){
  if(!bufFull&&bIdx<BUF_SIZE) return 0.0;
  float mean=0.0; for(int i=0;i<BUF_SIZE;i++) mean+=buf[i]; mean/=BUF_SIZE;
  float v=0.0; for(int i=0;i<BUF_SIZE;i++) v+=pow(buf[i]-mean,2);
  return v/BUF_SIZE;}
float calcMeanDelta(){
  if(!bufFull&&bIdx<BUF_SIZE) return 0.0;
  float s=0.0; for(int i=0;i<BUF_SIZE;i++) s+=buf[i];
  return s/BUF_SIZE;}
void updateAngleHistory(int angle,float dist){
  if(!baselineReady||dist<0||dist>effectiveRange) return;
  if(angle<=SWEEP_MIN+SWEEP_MARGIN||angle>=SWEEP_MAX-SWEEP_MARGIN) return;
  int idx=(angle-SWEEP_MIN)/SWEEP_STEP;
  if(idx<0||idx>=BASELINE_STEPS) return;
  bool clean=fabs(baseline[idx]-dist)<BASELINE_TOLERANCE;
  angleHistory[idx][angleHistCount[idx]%HISTORY_DEPTH]=clean?0.0f:fabs(baseline[idx]-dist);
  if(angleHistCount[idx]<HISTORY_DEPTH*100) angleHistCount[idx]++;}
void checkClogStatus(){
  int consecutive=0,maxConsecutive=0;
  for(int i=0;i<BASELINE_STEPS;i++){
    if(angleHistCount[i]<HISTORY_DEPTH){consecutive=0;continue;}
    float mean=0.0; for(int j=0;j<HISTORY_DEPTH;j++) mean+=angleHistory[i][j]; mean/=HISTORY_DEPTH;
    float var=0.0; for(int j=0;j<HISTORY_DEPTH;j++) var+=pow(angleHistory[i][j]-mean,2); var/=HISTORY_DEPTH;
    if(var<STATIC_VAR_THRESH&&mean>STATIC_DELTA_THRESH){consecutive++;if(consecutive>maxConsecutive)maxConsecutive=consecutive;}
    else consecutive=0;}
  isClogged=(maxConsecutive>=CLOG_ANGLE_COUNT);}
Status classify(float depth,float variance,float meanDelta,bool obstruction){
  waterRising=fabs(depth-lastCalibratedDepth)>0.4f;
  wasteActive=!waterRising&&(variance>VARIANCE_THRESH||meanDelta>MEAN_DELTA_THRESH);
  if(depth>=DEPTH_CRITICAL) return CRITICAL;
  if(depth>=DEPTH_ELEVATED) return ELEVATED;
  if(wasteActive) return WASTE;
  return NORMAL;}
void updateDebounce(Status raw){
  if(raw==candidateStatus) candidateCount++; else{candidateStatus=raw;candidateCount=1;}
  bool escalating=(int)raw>(int)confirmedStatus,deescalating=(int)raw<(int)confirmedStatus;
  bool waterCorrection=waterRising&&raw==ELEVATED&&confirmedStatus==WASTE;
  int needed=waterCorrection?DEBOUNCE_ESCALATE_WL:(escalating?((raw==WASTE)?DEBOUNCE_ESCALATE:DEBOUNCE_ESCALATE_WL):deescalating?DEBOUNCE_DEESCALATE:DEBOUNCE_ESCALATE);
  unsigned long holdRequired=(raw==NORMAL)?STATE_HOLD_TO_NORMAL_MS:STATE_HOLD_BETWEEN_MS;
  if(candidateCount>=needed&&(!deescalating||waterCorrection||millis()-stateConfirmedAt>=holdRequired)){
    Status prev=confirmedStatus; confirmedStatus=candidateStatus; candidateCount=needed; stateConfirmedAt=millis();
    if(confirmedStatus==NORMAL&&prev!=NORMAL){
      for(int i=0;i<BUF_SIZE;i++) buf[i]=0.0;
      bIdx=0;bufFull=true;memset(angleHistCount,0,sizeof(angleHistCount));
      isClogged=false;obstructionTimer=0;obstructionDetected=false;wasteActive=false;obstructedAngleMin=-1;obstructedAngleMax=-1;}}}
void setOutput(Status s){
  unsigned long now=millis();
  if(s==CALIBRATING){digitalWrite(LED_GREEN,(now%600<300)?HIGH:LOW);digitalWrite(LED_YELLOW,LOW);digitalWrite(LED_RED,LOW);buzzerOff();return;}
  digitalWrite(LED_GREEN,(s==NORMAL)?HIGH:LOW);
  if(s==WASTE&&isClogged){bool on=(now%600<150);digitalWrite(LED_YELLOW,LOW);digitalWrite(LED_RED,on?HIGH:LOW);on?buzzerOn():buzzerOff();}
  else if(s==WASTE){bool on=(now%1000<300);digitalWrite(LED_YELLOW,on?HIGH:LOW);digitalWrite(LED_RED,LOW);on?buzzerOn():buzzerOff();}
  else if(s==ELEVATED){digitalWrite(LED_YELLOW,HIGH);digitalWrite(LED_RED,LOW);buzzerOff();}
  else if(s==CRITICAL){digitalWrite(LED_YELLOW,LOW);digitalWrite(LED_RED,HIGH);buzzerOn();}
  else{digitalWrite(LED_YELLOW,LOW);digitalWrite(LED_RED,LOW);buzzerOff();}}
const char* statusLabel(Status s){
  switch(s){
    case NORMAL: return "Normal";
    case ELEVATED: return "Elevated";
    case WASTE: return isClogged?"Waste Detected (Clogged)":"Waste Detected";
    case CRITICAL: return "Critical Flood Risk";
    case CALIBRATING: return "CALIBRATING";}
  return "Unknown";}
void onWsEvent(AsyncWebSocket* server,AsyncWebSocketClient* client,AwsEventType type,void* arg,uint8_t* data,size_t len){
  if(type==WS_EVT_CONNECT) Serial.printf("[WS] Client %u connected\n",client->id());
  else if(type==WS_EVT_DISCONNECT) Serial.printf("[WS] Client %u disconnected\n",client->id());}
void pushLiveData(){
  if(ws.count()==0) return;
  ws.cleanupClients();
  JsonDocument doc;
  doc["maxDist"]=SENSOR_HEIGHT_CM;doc["angle"]=sweepAngle;
  doc["dist"]=(currentDist>0&&currentDist<=effectiveRange)?currentDist:-1.0;
  doc["depth"]=waterDepth;doc["variance"]=calcVariance();doc["meanDelta"]=calcMeanDelta();
  doc["obstr"]=obstructionDetected;doc["rawObs"]=isObstructed(sweepAngle,currentDist);
  doc["confirmed"]=statusLabel(confirmedStatus);doc["clogged"]=isClogged;
  doc["waste"]=wasteActive;doc["effRange"]=effectiveRange;
  String payload;serializeJson(doc,payload);ws.textAll(payload);}
void setup(){
  Serial.begin(115200);
  WiFi.disconnect(true);WiFi.softAPdisconnect(true);delay(100);WiFi.mode(WIFI_AP);WiFi.softAP(AP_SSID);
  esp_wifi_set_ps(WIFI_PS_NONE);
  dnsServer.start(53,"*",WiFi.softAPIP());
  ws.onEvent(onWsEvent);server.addHandler(&ws);
  server.on("/",HTTP_GET,[](AsyncWebServerRequest* r){r->send_P(200,"text/html",INDEX_HTML);});
  server.on("/generate_204",HTTP_GET,[](AsyncWebServerRequest* r){r->redirect("http://192.168.4.1/");});
  server.on("/hotspot-detect.html",HTTP_GET,[](AsyncWebServerRequest* r){r->redirect("http://192.168.4.1/");});
  server.on("/connecttest.txt",HTTP_GET,[](AsyncWebServerRequest* r){r->redirect("http://192.168.4.1/");});
  server.on("/fwlink",HTTP_GET,[](AsyncWebServerRequest* r){r->redirect("http://192.168.4.1/");});
  server.onNotFound([](AsyncWebServerRequest* r){r->redirect("http://192.168.4.1/");});
  server.begin();
  ledcAttachChannel(SERVO_PIN,50,16,0);writeServo(SWEEP_MIN);
  pinMode(BUZZER_PIN,OUTPUT);digitalWrite(BUZZER_PIN,LOW);
  pinMode(TRIG_PIN,OUTPUT);pinMode(ECHO_PIN,INPUT);
  pinMode(WL_VCC_PIN,OUTPUT);digitalWrite(WL_VCC_PIN,LOW);
  pinMode(LED_GREEN,OUTPUT);pinMode(LED_YELLOW,OUTPUT);pinMode(LED_RED,OUTPUT);
  memset(angleHistory,0,sizeof(angleHistory));memset(angleHistCount,0,sizeof(angleHistCount));
  setOutput(NORMAL);startCalibration();lastStepTime=millis();}
void loop(){
  dnsServer.processNextRequest();setOutput(confirmedStatus);
  unsigned long now=millis();
  unsigned long interval=(obstructedAngleMin>0&&sweepAngle>=obstructedAngleMin&&sweepAngle<=obstructedAngleMax)?DWELL_INTERVAL_MS:STEP_INTERVAL_MS;
  if(now-lastStepTime<interval){delay(1);return;}
  lastStepTime=now;
  int prevDir=sweepDir;stepServo();
  effectiveRange=calcAngleRange(sweepAngle);currentDist=readUltrasonicMedian();
  if(confirmedStatus!=CALIBRATING&&sweepAngle>=85&&sweepAngle<=95&&currentDist>0){
    static float centralBuf[11];static int cIdx=0;
    centralBuf[cIdx%11]=currentDist;cIdx++;
    float maxD=0;for(int ci=0;ci<min(cIdx,11);ci++) if(centralBuf[ci]>maxD) maxD=centralBuf[ci];
    if(maxD>0) waterDepth=max(0.0f,SENSOR_HEIGHT_CM-maxD);}
  if(sweepDir!=prevDir){
    checkClogStatus();
    if(confirmedStatus==CALIBRATING&&++calibrationPasses>=4){
      int cIdx=(90-SWEEP_MIN)/SWEEP_STEP;
      float centerDist=(mapCounts[cIdx]>0)?(mapAccumulator[cIdx]/mapCounts[cIdx]):-1.0f;
      for(int i=0;i<BASELINE_STEPS;i++) baseline[i]=(mapCounts[i]>0)?(mapAccumulator[i]/mapCounts[i]):SENSOR_HEIGHT_CM;
      baselineReady=true;confirmedStatus=NORMAL;candidateStatus=NORMAL;candidateCount=0;
      waterDepth=(centerDist>0)?max(0.0f,SENSOR_HEIGHT_CM-centerDist):0.0f;
      lastCalibratedDepth=waterDepth;
      float wl=readWaterLevelRaw();wlAtCalibration=(wl>=0)?wl:0.0f;currentWLRaw=wlAtCalibration;}}
  if(++wlCounter>=3){
    wlCounter=0;
    float wl=readWaterLevelRaw();
    if(wl>=0.0f){
      currentWLRaw=wl;
      if(confirmedStatus!=CALIBRATING&&fabs(currentWLRaw-wlAtCalibration)>=WL_CHANGE_THRESH){
        wlAtCalibration=currentWLRaw;startCalibration();}}}
  if(confirmedStatus==CALIBRATING){
    if(currentDist>0){int idx=(sweepAngle-SWEEP_MIN)/SWEEP_STEP;if(idx>=0&&idx<BASELINE_STEPS){mapAccumulator[idx]+=currentDist;mapCounts[idx]++;}}
  }else{
    if(currentDist>0&&currentDist<=effectiveRange){pushReading(currentDist,sweepAngle);updateAngleHistory(sweepAngle,currentDist);}
    else{buf[bIdx%BUF_SIZE]=0.0;if(++bIdx>=BUF_SIZE)bufFull=true;}
    if(isObstructed(sweepAngle,currentDist)){obstructionTimer=OBSTRUCTION_HOLD;if(obstructedAngleMin<0||sweepAngle<obstructedAngleMin)obstructedAngleMin=sweepAngle;if(obstructedAngleMax<0||sweepAngle>obstructedAngleMax)obstructedAngleMax=sweepAngle;}
    if(obstructionTimer>0){obstructionTimer--;obstructionDetected=true;}else{obstructionDetected=false;obstructedAngleMin=-1;obstructedAngleMax=-1;}
    updateDebounce(classify(waterDepth,calcVariance(),calcMeanDelta(),obstructionDetected));}
  static int ldc=0;if(++ldc>=2){ldc=0;pushLiveData();}
  Serial.printf("A:%d D:%.1f Dp:%.1f WL:%.2f WLref:%.2f V:%.2f M:%.2f O:%d C:%d S:%s\n",sweepAngle,currentDist,waterDepth,currentWLRaw,wlAtCalibration,calcVariance(),calcMeanDelta(),obstructionDetected,isClogged,statusLabel(confirmedStatus));}
