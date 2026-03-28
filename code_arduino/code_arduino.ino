#include <Servo.h>

const int SERVO_PIN    =  9;
const int TRIG_PIN     =  6;
const int ECHO_PIN     =  7;
const int WL_VCC_PIN   =  8;
const int WL_DATA_PIN  = A0;
const int LED_GREEN    =  2;
const int LED_YELLOW   =  3;
const int LED_RED      =  4;
const int BUZZER_PIN   =  5;

const int SWEEP_MIN    = 25;
const int SWEEP_MAX    = 155;
const int SWEEP_STEP   =  5;
const int SWEEP_DELAY  = 30; 

const int   BASELINE_STEPS  = (SWEEP_MAX - SWEEP_MIN) / SWEEP_STEP + 1;
float       baseline[BASELINE_STEPS];
bool        baselineReady   = false;
const float OBSTRUCT_THRESH = 8.0;

const float DEPTH_ELEVATED  = 10.0;
const float DEPTH_CRITICAL  = 20.0;
const float VARIANCE_THRESH =  3.0;

const int BUF_SIZE = 8;
float     buf[BUF_SIZE];
int       bIdx     = 0;
bool      bufFull  = false;

const int DEBOUNCE_COUNT = 3;

float simWaterDepth = 0.0;
float simDistance   = 35.0;
bool  useSim        = true;

enum Status { NORMAL, ELEVATED, WASTE, CRITICAL };
Servo  radarServo;
int    sweepAngle          = SWEEP_MIN;
int    sweepDir            = 1;
int    stepCount           = 0;
float  waterDepth          = 0.0;
bool   obstructionDetected = false;
Status confirmedStatus     = NORMAL;
Status candidateStatus     = NORMAL;
int    candidateCount      = 0;

float readUltrasonic() {
  return useSim ? simDistance : -1.0;
}

float readWaterLevel() {
  return useSim ? simWaterDepth : -1.0;
}

void resetState() {
  bIdx = 0;
  bufFull = false;
  obstructionDetected = false;
  stepCount = 0;
  waterDepth = 0.0;
  simWaterDepth = 0.0;
  simDistance = 35.0;
  confirmedStatus = NORMAL;
  candidateStatus = NORMAL;
  candidateCount = 0;
  for (int i = 0; i < BUF_SIZE; i++) buf[i] = 35.0;
  Serial.println(F("[SYSTEM] State Reset Complete"));
}

String serialBuffer = "";

void handleSerialSimulation() {
  while (Serial.available() > 0) {
    char c = Serial.read();
    if (c == '\n') {
      serialBuffer.trim();
      if (serialBuffer.length() >= 3) {
        char type = serialBuffer.charAt(0);
        float val = serialBuffer.substring(2).toFloat();

        if (type == 'W')      simWaterDepth = val;
        else if (type == 'U') simDistance   = val;
        else if (type == 'R') { resetState(); serialBuffer = ""; return; }
        
        Serial.print(F("[ACK] ")); Serial.print(type); 
        Serial.print(F(":")); Serial.println(val);
      }
      serialBuffer = "";
    } else {
      serialBuffer += c;
    }
  }
}

void pushReading(float d) {
  if (d < 0) return;
  buf[bIdx % BUF_SIZE] = d;
  bIdx++;
  if (bIdx >= BUF_SIZE) bufFull = true;
}

float calcVariance() {
  if (!bufFull && bIdx < BUF_SIZE) return 0.0;
  float mean = 0.0;
  for (int i = 0; i < BUF_SIZE; i++) mean += buf[i];
  mean /= BUF_SIZE;
  if (mean <= 0.0) return 0.0;
  float v = 0.0;
  for (int i = 0; i < BUF_SIZE; i++) v += pow(buf[i] - mean, 2);
  return v / BUF_SIZE;
}

Status classify(float depth, float variance, bool obstruction) {
  if (depth >= DEPTH_CRITICAL) return CRITICAL;
  if (variance > VARIANCE_THRESH || obstruction) return WASTE;
  if (depth >= DEPTH_ELEVATED) return ELEVATED;
  return NORMAL;
}

void updateDebounce(Status raw) {
  if (raw == candidateStatus) {
    if (++candidateCount >= DEBOUNCE_COUNT) {
      confirmedStatus = candidateStatus;
      candidateCount  = DEBOUNCE_COUNT;
    }
  } else {
    candidateStatus = raw;
    candidateCount  = 1;
  }
}

void setOutput(Status s) {
  digitalWrite(LED_GREEN,  (s == NORMAL)   ? HIGH : LOW);
  digitalWrite(LED_YELLOW, (s == ELEVATED || s == WASTE) ? HIGH : LOW);
  digitalWrite(LED_RED,    (s == CRITICAL) ? HIGH : LOW);
  if (s == WASTE) tone(BUZZER_PIN, 1200, 300);
  else if (s == CRITICAL) tone(BUZZER_PIN, 2500);
  else noTone(BUZZER_PIN);
}

const char* statusLabel(Status s) {
  switch (s) {
    case NORMAL:   return "NORMAL";
    case ELEVATED: return "ELEVATED";
    case WASTE:    return "WASTE_DETECTED";
    case CRITICAL: return "CRITICAL_FLOOD";
    default:       return "UNKNOWN";
  }
}

void setup() {
  Serial.begin(115200);
  
  radarServo.attach(SERVO_PIN);
  radarServo.write(SWEEP_MIN);
  delay(1000); 

  pinMode(LED_GREEN,   OUTPUT);
  pinMode(LED_YELLOW,  OUTPUT);
  pinMode(LED_RED,     OUTPUT);
  pinMode(BUZZER_PIN,  OUTPUT);

  resetState();
  for (int i = 0; i < BASELINE_STEPS; i++) baseline[i] = 35.0;
  baselineReady = true;

  Serial.println(F("[SYSTEM_READY] Simulation Mode Active (V2)"));
}

void loop() {
  const int STEPS_PER_SWEEP = (SWEEP_MAX - SWEEP_MIN) / SWEEP_STEP;

  handleSerialSimulation();

  if (stepCount == 0) obstructionDetected = false;

  sweepAngle += sweepDir * SWEEP_STEP;
  if (sweepAngle >= SWEEP_MAX) { sweepAngle = SWEEP_MAX; sweepDir = -1; }
  else if (sweepAngle <= SWEEP_MIN) { sweepAngle = SWEEP_MIN; sweepDir = 1; }
  radarServo.write(sweepAngle);

  float dist = readUltrasonic();
  pushReading(dist);

  int idx = (sweepAngle - SWEEP_MIN) / SWEEP_STEP;
  if (baseline[idx] - dist >= OBSTRUCT_THRESH) obstructionDetected = true;

  waterDepth = readWaterLevel();

  float variance = calcVariance();
  Status raw = classify(waterDepth, variance, obstructionDetected);
  updateDebounce(raw);
  setOutput(confirmedStatus);

  Serial.print(sweepAngle); Serial.print(",");
  Serial.print(dist, 1); Serial.print(",");
  Serial.print(waterDepth, 1); Serial.print(",");
  Serial.print(variance, 2); Serial.print(",");
  Serial.print(obstructionDetected ? 1 : 0); Serial.print(",");
  Serial.println(statusLabel(confirmedStatus));

  delay(SWEEP_DELAY);
}
