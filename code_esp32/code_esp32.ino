#include <WiFi.h>
#include <HTTPClient.h>
#include <ESP32Servo.h>

const char* ssid       = "YOUR_WIFI_SSID";
const char* password   = "YOUR_WIFI_PASSWORD";
const char* serverUrl  = "http://192.168.1.100/alert_endpoint";

// Note: WL_DATA_PIN MUST be ADC1 (e.g., GPIO34) because ADC2 is 
// disabled when WiFi is used on the ESP32.
const int SERVO_PIN    = 13;
const int TRIG_PIN     = 14;
const int ECHO_PIN     = 12;
const int WL_VCC_PIN   = 33; 
const int WL_DATA_PIN  = 34; 
const int LED_GREEN    = 25;
const int LED_YELLOW   = 26;
const int LED_RED      = 27;
const int BUZZER_PIN   = 32;

const int BUZZER_CHAN  = 0;

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
Status lastPrintedStatus   = NORMAL;

float readUltrasonic() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  long duration = pulseIn(ECHO_PIN, HIGH, 25000);
  if (duration == 0) return -1.0;

  float distance = (duration * 0.0343) / 2.0;
  if (distance < 2.0 || distance > 400.0) return -1.0;

  return distance;
}

float readWaterLevel() {
  digitalWrite(WL_VCC_PIN, HIGH);
  delayMicroseconds(500);

  int raw = analogRead(WL_DATA_PIN);
  digitalWrite(WL_VCC_PIN, LOW);

  if (raw <= 0 || raw >= 4095) return -1.0;

  float depth = map(raw, 0, 4095, 0, 30);

  if (depth < 0.0)  depth = 0.0;
  if (depth > 50.0) return -1.0;

  return depth;
}

float sweepAndRead() {
  sweepAngle += sweepDir * SWEEP_STEP;

  if (sweepAngle >= SWEEP_MAX) {
    sweepAngle = SWEEP_MAX;
    sweepDir   = -1;
  } else if (sweepAngle <= SWEEP_MIN) {
    sweepAngle = SWEEP_MIN;
    sweepDir   = 1;
  }

  radarServo.write(sweepAngle);
  delay(SWEEP_DELAY);
  return readUltrasonic();
}

void calibrateBaseline() {
  Serial.println(F("Calibrating baseline — canal must be empty..."));
  delay(1000);

  int tempAngle = SWEEP_MIN;
  for (int i = 0; i < BASELINE_STEPS; i++) {
    radarServo.write(tempAngle);
    delay(100);

    float sum = 0.0;
    int valid = 0;
    for (int r = 0; r < 3; r++) {
      float d = readUltrasonic();
      if (d > 0) { sum += d; valid++; }
      delay(30);
    }

    baseline[i] = (valid > 0) ? (sum / valid) : 50.0;
    Serial.printf("  Angle: %d°  Baseline: %.1f cm\n", tempAngle, baseline[i]);
    tempAngle += SWEEP_STEP;
  }
  baselineReady = true;
  Serial.println(F("Baseline complete. Starting..."));
}

bool isObstructed(int angle, float dist) {
  if (!baselineReady || dist < 0) return false;
  int idx = (angle - SWEEP_MIN) / SWEEP_STEP;
  return ((baseline[idx] - dist) >= OBSTRUCT_THRESH);
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
  digitalWrite(LED_GREEN,  (s == NORMAL) ? HIGH : LOW);
  digitalWrite(LED_YELLOW, (s == ELEVATED || s == WASTE) ? HIGH : LOW);
  digitalWrite(LED_RED,    (s == CRITICAL) ? HIGH : LOW);

  if (s == WASTE) {
    ledcWriteTone(BUZZER_CHAN, 1200);   
  } else if (s == CRITICAL) {
    ledcWriteTone(BUZZER_CHAN, 2500);   
  } else {
    ledcWriteTone(BUZZER_CHAN, 0);      
  }
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

void sendAlert(Status s) {
  Serial.printf("[ALERT] Status changed -> %s\n", statusLabel(s));

  if (WiFi.status() == WL_CONNECTED) {
    HTTPClient http;
    http.begin(serverUrl);
    http.addHeader("Content-Type", "application/json");

    String jsonPayload = "{\"status\":\"" + String(statusLabel(s)) + "\",\"depth\":" + String(waterDepth) + "}";
    
    int httpResponseCode = http.POST(jsonPayload);
    
    if (httpResponseCode > 0) {
      Serial.printf("HTTP Response code: %d\n", httpResponseCode);
    } else {
      Serial.printf("Error code: %d\n", httpResponseCode);
    }
    http.end();
  } else {
    Serial.println("WiFi Disconnected. Alert not sent.");
  }
}

void setup() {
  Serial.begin(115200);

  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi..");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nConnected to WiFi");

  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);
  
  radarServo.setPeriodHertz(50);
  radarServo.attach(SERVO_PIN, 500, 2400);
  radarServo.write(SWEEP_MIN);
  delay(1000); 

  pinMode(TRIG_PIN,    OUTPUT);
  pinMode(ECHO_PIN,    INPUT);
  pinMode(WL_VCC_PIN,  OUTPUT);
  digitalWrite(WL_VCC_PIN, LOW);

  pinMode(LED_GREEN,   OUTPUT);
  pinMode(LED_YELLOW,  OUTPUT);
  pinMode(LED_RED,     OUTPUT);
  
  ledcSetup(BUZZER_CHAN, 2000, 8);
  ledcAttachPin(BUZZER_PIN, BUZZER_CHAN);

  for (int i = 0; i < BUF_SIZE; i++) buf[i] = 30.0;

  setOutput(NORMAL);
  calibrateBaseline();
}

void loop() {
  const int STEPS_PER_SWEEP = (SWEEP_MAX - SWEEP_MIN) / SWEEP_STEP;

  if (stepCount == 0) obstructionDetected = false;

  float dist = sweepAndRead();
  pushReading(dist);

  if (isObstructed(sweepAngle, dist)) obstructionDetected = true;

  if (++stepCount >= STEPS_PER_SWEEP) {
    float rawDepth = readWaterLevel();
    if (rawDepth >= 0.0) waterDepth = rawDepth;
    stepCount = 0;
  }

  float  variance = calcVariance();
  Status raw      = classify(waterDepth, variance, obstructionDetected);

  updateDebounce(raw);
  setOutput(confirmedStatus);

  if (confirmedStatus != lastPrintedStatus) {
    sendAlert(confirmedStatus);
    lastPrintedStatus = confirmedStatus;
  }

  Serial.print(sweepAngle); Serial.print(",");
  Serial.print(dist, 1); Serial.print(",");
  Serial.print(waterDepth, 1); Serial.print(",");
  Serial.print(variance, 2); Serial.print(",");
  Serial.print(obstructionDetected ? 1 : 0); Serial.print(",");
  Serial.println(statusLabel(confirmedStatus));
}

