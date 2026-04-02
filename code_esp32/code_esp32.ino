#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <ArduinoJson.h>
#include <ESP32Servo.h>

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
// FIX 3: 1° per step for smoother, more accurate scanning
const int SWEEP_STEP = 1;

// FIX 3: 100ms per step — slow constant sweep (~13s per arc)
const unsigned long STEP_INTERVAL_MS = 100;

const float OBSTRUCT_THRESH  = 2.0;   // cm difference from baseline to flag obstruction
const float DEPTH_ELEVATED   = 10.0;  // cm water depth for ELEVATED
const float DEPTH_CRITICAL   = 15.0;  // cm water depth for CRITICAL
const float VARIANCE_THRESH  = 3.0;   // variance threshold for waste movement detection
const float MEAN_DELTA_THRESH = 1.5;  // avg delta from baseline to flag stationary object
const int   OBSTRUCTION_HOLD = 20;   // Hold obstruction for 20 steps (2 seconds) to cover close-range sensor blindness
const int   WL_READ_INTERVAL = 10;   // Read water level every 10 steps (1 second)

// Fallback value when no water level sensor is wired
const float SIMULATED_WATER_DEPTH = 0.0;
const float WL_EMA_ALPHA = 0.8;  // EMA smoothing: 0.8 = very fast response, slight smoothing

// ═══════════════════════════════════════════════════════════════
// STATE VARIABLES
// ═══════════════════════════════════════════════════════════════
const int   BASELINE_STEPS = (SWEEP_MAX - SWEEP_MIN) / SWEEP_STEP + 1;
float       baseline[BASELINE_STEPS];
bool        baselineReady = false;

// FIX 4: Larger variance buffer for smoother readings
const int BUF_SIZE = 12;
float     buf[BUF_SIZE];
int       bIdx    = 0;
bool      bufFull = false;

// Asymmetric debounce — quick to escalate, slow to de-escalate
const int DEBOUNCE_ESCALATE   = 2;
const int DEBOUNCE_DEESCALATE = 3;

// Stale variance buffer flush — reset after N consecutive out-of-range readings
int noInRangeCount = 0;
const int STALE_FLUSH_COUNT = 30;

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

// ── Server Instances ─────────────────────────────────────────────
AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

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
    background:#0D1B2A;color:#B8D4E8;
    font-family:'Courier New',monospace;
    display:flex;flex-direction:column;
    align-items:center;min-height:100vh;padding:10px;
  }
  h1{color:#00B4D8;font-size:16px;letter-spacing:3px;margin:10px 0;text-transform:uppercase;text-align:center}
  #header{display:flex;align-items:center;justify-content:center;gap:8px;margin-bottom:12px;font-size:11px}
  #main{display:flex;gap:16px;flex-wrap:wrap;justify-content:center;width:100%;max-width:900px}
  canvas#radar{border:1px solid #0077A8;border-radius:4px;background:#050E18;max-width:100%;height:auto}
  #panel{background:#152638;border:1px solid #0077A8;border-radius:4px;padding:14px;flex:1;min-width:260px;max-width:320px}
  .ptitle{color:#00B4D8;font-size:11px;letter-spacing:2px;margin-bottom:12px}
  .sbox{border-radius:4px;padding:10px;margin-bottom:10px;border:1px solid currentColor;transition:all .3s ease}
  .sbox.critical{animation:critPulse 1s infinite}
  @keyframes critPulse{0%,100%{box-shadow:0 0 8px rgba(231,76,60,0.5)}50%{box-shadow:0 0 20px rgba(231,76,60,0.9)}}
  .slbl{font-size:10px;letter-spacing:1px;opacity:.7;margin-bottom:4px}
  .sval{font-size:18px;font-weight:bold}
  .row{display:flex;justify-content:space-between;font-size:12px;padding:6px 0;border-bottom:1px solid #1C3354}
  .rl{color:#6A8FAA}.rv{color:#B8D4E8;font-weight:bold}
  #log{margin-top:10px;background:#0A1828;border-radius:4px;padding:8px;font-size:10px;height:100px;overflow-y:auto}
  .le{color:#6A8FAA;padding:2px 0}.le.ch{color:#00B4D8;font-weight:bold}
  #cd{display:inline-block;width:8px;height:8px;border-radius:50%;background:#E74C3C;animation:pulse 1.5s infinite}
  @keyframes pulse{0%,100%{opacity:1}50%{opacity:.3}}
  #cd.live{background:#2ECC71;animation:none}
  #graphs{display:flex;gap:10px;flex-wrap:wrap;justify-content:center;margin-top:16px;width:100%;max-width:900px}
  .gb{background:#152638;border:1px solid #0077A8;border-radius:4px;padding:8px;flex:1;min-width:260px}
  .gt{font-size:10px;color:#6A8FAA;letter-spacing:1px;margin-bottom:6px}
  canvas.ch{width:100%;height:70px;background:#0A1828}
</style>
</head>
<body>
<h1>CANAL FLOOD AND WASTE RADAR</h1>
<div id="header"><span id="cd"></span><span id="cl">Connecting...</span></div>

<div id="main">
  <canvas id="radar" width="560" height="340"></canvas>
  <div id="panel">
    <div class="ptitle">SYSTEM STATUS</div>
    <div class="sbox" id="sb"><div class="slbl">CONFIRMED STATUS</div><div class="sval" id="sv">---</div></div>
    <div class="row"><span class="rl">ANGLE</span><span class="rv" id="ra">---</span></div>
    <div class="row"><span class="rl">DISTANCE</span><span class="rv" id="rd">---</span></div>
    <div class="row"><span class="rl">DEPTH</span><span class="rv" id="rp">---</span></div>
    <div class="row"><span class="rl">VARIANCE</span><span class="rv" id="rr">---</span></div>
    <div class="row"><span class="rl">OBSTRUCTION</span><span class="rv" id="ro">---</span></div>
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
var CX=rc.width/2,CY=rc.height-30,R=Math.min(CX-20,CY-20);
var sw=SM;

// Per-angle scan memory
var sDist=new Float32Array(SS).fill(-1);
var sAge=new Float32Array(SS).fill(9999);
var sObs=new Uint8Array(SS).fill(0);
var sConf=new Array(SS).fill('Normal');

// Sweep trail
var trail=[];
var TMAX=30;

var sys={angle:90,dist:-1,depth:0,variance:0,obstr:false,confirmed:'Normal'};
var lastC='';
var logE=document.getElementById('log');
var HN=120,dH=new Float32Array(HN),vH=new Float32Array(HN),hI=0;

function sCol(s){
  if(s==='Normal')return'#2ECC71';
  if(s==='Elevated')return'#F1C40F';
  if(s==='Waste Detected')return'#E67E22';
  if(s==='Critical Flood Risk')return'#E74C3C';
  return'#6A8FAA';
}
function sRGB(s){
  if(s==='Normal')return[46,204,113];
  if(s==='Elevated')return[241,196,15];
  if(s==='Waste Detected')return[230,126,34];
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
  cx.clearRect(0,0,rc.width,rc.height);

  // Background semicircle
  cx.fillStyle='#050E18';
  cx.beginPath();cx.arc(CX,CY,R,Math.PI,2*Math.PI);cx.fill();

  // Range rings
  var rings=[.25,.5,.75,1];
  for(var i=0;i<4;i++){
    var rr=R*rings[i];
    cx.beginPath();cx.arc(CX,CY,rr,Math.PI,2*Math.PI);
    cx.strokeStyle='rgba(0,200,120,0.25)';cx.lineWidth=.5;cx.stroke();
    cx.fillStyle='rgba(0,255,120,0.6)';cx.font='10px Courier New';cx.textAlign='center';
    cx.fillText((MD*rings[i]).toFixed(0)+'cm',CX,CY-rr-5);
  }

  // Radial spokes
  for(var d=30;d<=150;d+=30){
    var p=toXY(d,MD);
    cx.beginPath();cx.moveTo(CX,CY);cx.lineTo(p.x,p.y);
    cx.strokeStyle='rgba(0,200,120,0.25)';cx.lineWidth=.5;cx.stroke();
    var lp=toXY(d,MD*1.08);
    cx.fillStyle='rgba(0,255,120,0.7)';
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

  // Info label
  cx.fillStyle='#4A6A7A';cx.font='10px Courier New';cx.textAlign='center';
  cx.fillText('HC-SR04  |  130\u00b0 ARC  |  MAX '+MD.toFixed(0)+'cm',CX,CY+16);
}

function uPanel(){
  var c=sCol(sys.confirmed);
  var sb=document.getElementById('sb');
  sb.style.borderColor=c;sb.style.backgroundColor=c+'20';
  sb.className='sbox'+(sys.confirmed==='Critical Flood Risk'?' critical':'');
  var sv=document.getElementById('sv');
  sv.style.color=c;sv.textContent=sys.confirmed;

  document.getElementById('ra').textContent=sys.angle+'\u00b0';
  document.getElementById('rd').textContent=(sys.dist>0&&sys.dist<=MD)?sys.dist.toFixed(1)+' cm':'NO ECHO';
  document.getElementById('rp').textContent=sys.depth.toFixed(1)+' cm';
  document.getElementById('rr').textContent=sys.variance.toFixed(2);
  var ro=document.getElementById('ro');
  ro.textContent=sys.obstr?'DETECTED':'CLEAR';
  ro.style.color=sys.obstr?'#E74C3C':'#2ECC71';

  if(sys.confirmed!==lastC&&lastC!==''){
    aLog('STATUS: '+lastC+' -> '+sys.confirmed,true);
  }
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

connect();
render();
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

// FIX 4: Take 3 readings and return the median, with consistency check.
// Eliminates single-reading spikes from multi-path reflections.
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

// FIX 1 & 2: Accept raw=4095 as valid (max water level), use float mapping
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

Status classify(float depth, float variance, float meanDelta, bool obstruction) {
  bool wasteFlag = variance > VARIANCE_THRESH || obstruction || meanDelta > MEAN_DELTA_THRESH;
  if (depth >= DEPTH_CRITICAL) return CRITICAL;
  if (depth >= DEPTH_ELEVATED && wasteFlag) return CRITICAL;
  if (wasteFlag) return WASTE;
  if (depth >= DEPTH_ELEVATED) return ELEVATED;
  return NORMAL;
}

// FIX 5: Asymmetric debounce — 2 to escalate, 5 to de-escalate
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
    confirmedStatus = candidateStatus;
    candidateCount  = needed;
  }
}

void setOutput(Status s) {
  unsigned long now = millis();
  bool blinkState = (now / 500) % 2 == 0; // 500ms ON, 500ms OFF
  bool fastBlink  = (now / 150) % 2 == 0; // 150ms for two-tone critical alarm

  digitalWrite(LED_GREEN,  (s == NORMAL) ? HIGH : LOW);
  
  if (s == WASTE) {
    digitalWrite(LED_YELLOW, blinkState ? HIGH : LOW);
  } else {
    digitalWrite(LED_YELLOW, (s == ELEVATED) ? HIGH : LOW);
  }
  
  digitalWrite(LED_RED,    (s == CRITICAL) ? HIGH : LOW);

  if (s == WASTE) {
    if (blinkState) {
      // 3000Hz is near the resonant frequency of most passive buzzers (much louder)
      ledcWriteTone(BUZZER_PIN, 3000);
    } else {
      ledcWriteTone(BUZZER_PIN, 0);
      ledcWrite(BUZZER_PIN, 0);
    }
  } else if (s == CRITICAL) {
    // Unique two-tone piercing alarm (Hi-Lo siren) for critical state
    if (fastBlink) {
      ledcWriteTone(BUZZER_PIN, 4000);
    } else {
      ledcWriteTone(BUZZER_PIN, 3000);
    }
  } else {
    // ledcWriteTone(pin, 0) doesn't reliably stop on all ESP32 cores.
    // Force duty cycle to 0 to guarantee silence.
    ledcWriteTone(BUZZER_PIN, 0);
    ledcWrite(BUZZER_PIN, 0);
  }
}

const char* statusLabel(Status s) {
  switch (s) {
    case NORMAL:   return "Normal";
    case ELEVATED: return "Elevated";
    case WASTE:    return "Waste Detected";
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

  ws.onEvent(onWsEvent);
  server.addHandler(&ws);
  server.on("/", HTTP_GET, [](AsyncWebServerRequest* request) {
    request->send_P(200, "text/html", INDEX_HTML);
  });
  server.begin();
  Serial.println("Web server ready.");

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
  setOutput(NORMAL);
  calibrateBaseline();
  lastStepTime = millis();
}

void loop() {
  unsigned long now = millis();
  if (now - lastStepTime < STEP_INTERVAL_MS) {
    delay(1);
    return;
  }
  lastStepTime = now;

  stepServo();
  currentDist = readUltrasonicMedian();

  // Push delta-from-baseline for valid in-range readings
  if (currentDist > 0 && currentDist <= MAX_DETECTION_RANGE_CM) {
    pushReading(currentDist, sweepAngle);
    noInRangeCount = 0;
  } else {
    // Track consecutive out-of-range readings
    noInRangeCount++;
    if (noInRangeCount >= STALE_FLUSH_COUNT && bufFull) {
      // Buffer has stale data from old detections — flush to calm state
      for (int i = 0; i < BUF_SIZE; i++) buf[i] = 0.0;
      bIdx = 0;
      bufFull = false;
      noInRangeCount = 0;
    }
  }

  // Obstruction with short hold timer — clears ~500ms after object is removed
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
    float rawDepth = readWaterLevel();
    if (rawDepth >= 0.0) {
      // EMA smoothing: eliminates ADC jitter while staying responsive
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

  Serial.printf("A:%d D:%.1f Dp:%.1f V:%.2f M:%.2f O:%d S:%s\n",
                sweepAngle, currentDist, waterDepth, variance, meanDelta,
                obstructionDetected, statusLabel(confirmedStatus));
}
