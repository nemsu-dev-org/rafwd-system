import processing.serial.*;

Serial       port;
String       activePortName = "NONE";
String       rawLine        = "";
boolean      serialReady    = false;
boolean      portError      = false;
String       portErrorMsg   = "";
int          lastDataMillis = 0;
final int    DATA_TIMEOUT   = 3000;
boolean      selectingPort  = true;
String[]     availablePorts;
int          hoveredPort    = -1;

float   parsedAngle     = 90;
float   parsedDist      = -1;
float   parsedDepth     = 0;
float   parsedVariance  = 0;
boolean parsedObstr     = false;
String  parsedRaw       = "NORMAL";
String  parsedConfirmed = "NORMAL";

final int   SWEEP_MIN      = 25;
final int   SWEEP_MAX      = 155;
final float MAX_DIST_CM    = 60.0;

// Array history removed for test.pde motion blur style

float   sweepAngle         = SWEEP_MIN;
float   sweepAngleSmooth   = SWEEP_MIN;

int     radarCX, radarCY;
int     radarR;
int     dashY;

color   COL_BG         = color(8, 20, 35);
color   COL_RADAR_BG   = color(5, 14, 24);
color   COL_GRID       = color(0, 80, 60, 80);
color   COL_SWEEP      = color(0, 200, 120, 200);
color   COL_CLEAR      = color(0, 220, 100);
color   COL_WASTE_MOV  = color(255, 165, 0);
color   COL_WASTE_STA  = color(220, 50, 50);
color   COL_CRITICAL   = color(255, 30, 30);
color   COL_ELEVATED   = color(255, 220, 0);
color   COL_NORMAL     = color(0, 220, 100);
color   COL_TEXT       = color(180, 220, 200);
color   COL_MUTED      = color(80, 120, 100);
color   COL_PANEL_BG   = color(12, 28, 45);
color   COL_ACCENT     = color(0, 180, 216);

PFont   fontMono;
PFont   fontUI;

final int   LOG_MAX    = 5;
String[]    logLines   = new String[LOG_MAX];
int         logCount   = 0;
String      lastConfirmed = "";

final int   VAR_HISTORY   = 100;
float[]     varHistory    = new float[VAR_HISTORY];
int         varHIdx       = 0;

void setup() {
  size(1280, 800);
  surface.setTitle("Canal Flood & Waste — Horizontal Radar");
  smooth(4);

  fontMono = createFont("Courier New", 13);
  fontUI   = createFont("Arial", 13);

  radarCX = width / 2;
  radarCY = 520;
  radarR  = 460;
  dashY   = 540;

  // Scan arrays removed
  for (int i = 0; i < LOG_MAX; i++) logLines[i] = "";
  for (int i = 0; i < VAR_HISTORY; i++) varHistory[i] = 0;

  availablePorts = Serial.list();
  if (availablePorts.length == 0) portErrorMsg = "No COM ports found.";
  
  addLog("System initialized.");
}

void draw() {
  if (selectingPort) {
    background(COL_BG);
    drawPortSelector();
    return;
  }

  processSerial();

  // Smooth the angle to create a solid, gap-less sweep
  sweepAngleSmooth += (sweepAngle - sweepAngleSmooth) * 0.4;

  // Motion blur fading — identical to test.pde
  noStroke();
  fill(0, 4); 
  rect(0, 0, width, dashY);

  // Draw Radar (Top)
  pushMatrix();
  translate(radarCX, radarCY);
  drawRadarBackground();
  drawSweepLine();
  drawObject();
  popMatrix();

  // Solid redraw of dashboard area
  drawDashboard();
  drawStatusBar();
}

void processSerial() {
  if (port == null) return;
  while (port.available() > 0) {
    String inStr = port.readStringUntil('\n');
    if (inStr != null) {
      inStr = trim(inStr);
      if (inStr.length() > 0) parseData(inStr);
    }
  }
}

void parseData(String line) {
  String[] parts = split(line, ',');
  if (parts.length >= 6) {
    try {
      lastDataMillis = millis();

      float rawAngle = float(parts[0]);
      parsedAngle    = constrain(rawAngle, SWEEP_MIN, SWEEP_MAX);
      sweepAngle     = parsedAngle;
      
      parsedDist     = float(parts[1]);
      parsedDepth    = float(parts[2]);
      parsedVariance = float(parts[3]);
      parsedObstr    = (int(parts[4]) == 1);
      parsedConfirmed= trim(parts[5]);

      if (parsedConfirmed.length() == 0) parsedConfirmed = "UNKNOWN";

      // Log status changes
      if (!parsedConfirmed.equals(lastConfirmed) && lastConfirmed.length() > 0) {
        addLog(lastConfirmed + " -> " + parsedConfirmed);
      }
      lastConfirmed = parsedConfirmed;

      // Update var history
      varHistory[varHIdx % VAR_HISTORY] = parsedVariance;
      varHIdx++;

      // Array updates removed for test.pde approach

    } catch (Exception e) {
      addLog("Data parse error");
    }
  }
}

void drawPortSelector() {
  background(COL_BG);
  textFont(fontUI);
  textAlign(CENTER, CENTER);
  
  fill(COL_ACCENT);
  textSize(24);
  text("CANAL FLOOD & WASTE DETECTION", width/2, 100);
  textSize(16);
  fill(COL_TEXT);
  text("Select Arduino COM Port:", width/2, 150);

  if (portError) {
    fill(COL_CRITICAL);
    text("ERROR: " + portErrorMsg, width/2, 190);
  }

  if (availablePorts.length == 0) {
    fill(COL_MUTED);
    text("No active COM ports detected. Press 'R' to refresh.", width/2, 250);
    return;
  }

  int startY = 220;
  int btnW = 300, btnH = 40;
  hoveredPort = -1;

  for (int i = 0; i < availablePorts.length; i++) {
    int by = startY + i * 50;
    boolean hover = mouseX > width/2 - btnW/2 && mouseX < width/2 + btnW/2 &&
                    mouseY > by - btnH/2 && mouseY < by + btnH/2;
    if (hover) {
      hoveredPort = i;
      fill(0, 120, 200);
    } else {
      fill(COL_PANEL_BG);
    }
    
    stroke(COL_ACCENT);
    strokeWeight(1);
    rect(width/2 - btnW/2, by - btnH/2, btnW, btnH, 5);
    
    fill(255);
    textSize(14);
    text(availablePorts[i], width/2, by);
  }

  fill(COL_MUTED);
  textSize(12);
  text("Press 'R' to rescan ports", width/2, startY + availablePorts.length * 50 + 30);
  textAlign(LEFT, BASELINE);
}

void drawRadarBackground() {
  noFill();
  strokeWeight(2);
  stroke(98, 245, 31);
  
  float d = radarR * 2;
  arc(0, 0, d, d, PI, TWO_PI);
  arc(0, 0, d * 0.75, d * 0.75, PI, TWO_PI);
  arc(0, 0, d * 0.50, d * 0.50, PI, TWO_PI);
  arc(0, 0, d * 0.25, d * 0.25, PI, TWO_PI);

  line(-radarR, 0, radarR, 0);
  for (int deg = 30; deg <= 150; deg += 30) {
    line(0, 0, -radarR * cos(radians(deg)), -radarR * sin(radians(deg)));
  }
}

void drawObject() {
  if (parsedDist > 2 && parsedDist <= MAX_DIST_CM) {
    float pixsDistance = map(parsedDist, 0, MAX_DIST_CM, 0, radarR);
    strokeWeight(9);
    
    if (parsedConfirmed.equals("CRITICAL")) stroke(255, 30, 30);
    else if (parsedConfirmed.equals("WASTE")) stroke(255, 165, 0);
    else if (parsedConfirmed.equals("ELEVATED")) stroke(255, 220, 0);
    else stroke(255, 10, 10); // Red default as in test.pde

    float angleRads = radians(180 - sweepAngleSmooth);
    
    line(pixsDistance * cos(angleRads), -pixsDistance * sin(angleRads),
         radarR * cos(angleRads), -radarR * sin(angleRads));
  }
}

void drawSweepLine() {
  strokeWeight(9);
  stroke(30, 250, 60);
  float angleRads = radians(180 - sweepAngleSmooth);
  line(0, 0, radarR * cos(angleRads), -radarR * sin(angleRads));
}

void drawDashboard() {
  int cy = dashY;
  
  // Dash BG
  fill(COL_PANEL_BG);
  noStroke();
  rect(10, cy, width - 20, 220, 8);

  // Section 1: STATUS
  drawStatusBadge(20, cy + 15, 260);

  // Section 2: READINGS
  drawReadings(300, cy + 15, 300);
  
  // Section 3: GRAPH
  drawVarGraph(620, cy + 15, 340);

  // Section 4: LOG
  drawEventLog(980, cy + 15, width - 980 - 20);
}

void drawStatusBadge(int x, int y, int w) {
  fill(COL_ACCENT);
  textFont(fontUI);
  textSize(12);
  text("SYSTEM STATUS", x, y + 10);

  color sc = statusToColor(parsedConfirmed);
  fill(red(sc), green(sc), blue(sc), 30);
  stroke(sc);
  strokeWeight(1.5);
  rect(x, y + 25, w, 70, 6);
  noStroke();

  fill(sc);
  textSize(24);
  textAlign(CENTER, CENTER);
  text(parsedConfirmed, x + w/2, y + 25 + 35);
  textAlign(LEFT, BASELINE);

  // Obstruction flag
  int oy = y + 110;
  fill(color(15, 35, 55));
  rect(x, oy, w, 30, 4);
  fill(COL_MUTED);
  textSize(11);
  text("OBSTRUCTION:", x + 10, oy + 20);
  
  if (parsedObstr) {
    fill(COL_WASTE_STA);
    text("DETECTED", x + 120, oy + 20);
  } else {
    fill(COL_CLEAR);
    text("CLEAR", x + 120, oy + 20);
  }
}

void drawReadings(int x, int y, int w) {
  fill(COL_ACCENT);
  textFont(fontUI);
  textSize(12);
  text("LIVE READINGS", x, y + 10);

  int ry = y + 25;
  drawDataRow(x, ry, w, "ANGLE", (int)parsedAngle + "°", (parsedAngle - SWEEP_MIN)/(SWEEP_MAX - SWEEP_MIN), COL_ACCENT);
  ry += 35;
  float distFrac = (parsedDist > 0) ? constrain(parsedDist/MAX_DIST_CM, 0, 1) : 0;
  String distStr = (parsedDist > 0) ? nf(parsedDist, 1, 1) + " cm" : "NO ECHO";
  drawDataRow(x, ry, w, "DISTANCE", distStr, distFrac, COL_CLEAR);
  ry += 35;
  drawDataRow(x, ry, w, "DEPTH", nf(parsedDepth, 1, 1) + " cm", constrain(parsedDepth/30.0, 0, 1), COL_ACCENT);
  ry += 35;
  drawDataRow(x, ry, w, "VARIANCE", nf(parsedVariance, 1, 2), constrain(parsedVariance/5.0, 0, 1), parsedVariance > 3.0 ? COL_WASTE_MOV : COL_CLEAR);
}

void drawDataRow(int x, int y, int w, String label, String val, float frac, color col) {
  fill(color(15, 35, 55));
  rect(x, y, w, 28, 4);
  fill(red(col), green(col), blue(col), 80);
  rect(x, y, w * frac, 28, 4);
  
  fill(COL_MUTED);
  textFont(fontMono);
  textSize(10);
  textAlign(LEFT, CENTER);
  text(label, x + 8, y + 14);
  
  fill(col);
  textSize(13);
  textAlign(RIGHT, CENTER);
  text(val, x + w - 8, y + 14);
  textAlign(LEFT, BASELINE);
}

void drawVarGraph(int x, int y, int w) {
  fill(COL_ACCENT);
  textFont(fontUI);
  textSize(12);
  text("VARIANCE HISTORY", x, y + 10);

  int gy = y + 25;
  int gh = 135;
  fill(color(10, 22, 35));
  noStroke();
  rect(x, gy, w, gh, 4);

  float threshY = map(3.0, 0, 10, gy + gh - 10, gy + 10);
  stroke(COL_WASTE_MOV, 100);
  strokeWeight(1);
  line(x, threshY, x + w, threshY);

  stroke(COL_WASTE_MOV);
  strokeWeight(2);
  noFill();
  beginShape();
  for(int i = 0; i < VAR_HISTORY; i++) {
    int idx = (varHIdx - VAR_HISTORY + i + VAR_HISTORY * 2) % VAR_HISTORY;
    float v = varHistory[idx];
    float px = map(i, 0, VAR_HISTORY - 1, x, x + w);
    float py = map(constrain(v, 0, 10), 0, 10, gy + gh - 10, gy + 10);
    vertex(px, py);
  }
  endShape();
  noStroke();
}

void drawEventLog(int x, int y, int w) {
  fill(COL_ACCENT);
  textFont(fontUI);
  textSize(12);
  text("EVENT LOG", x, y + 10);

  int ly = y + 25;
  fill(color(10, 22, 35));
  noStroke();
  rect(x, ly, w, 135, 4);

  for (int i = 0; i < LOG_MAX; i++) {
    int idx = (logCount - LOG_MAX + i + LOG_MAX * 10) % LOG_MAX;
    if (logLines[idx].length() > 0) {
      float a = map(i, 0, LOG_MAX - 1, 100, 255);
      fill(COL_TEXT, a);
      textFont(fontMono);
      textSize(11);
      text(logLines[idx], x + 10, ly + 25 + i * 22);
    }
  }
}

void drawStatusBar() {
  int bh = 28;
  int by = height - bh;
  color sc = statusToColor(parsedConfirmed);
  
  fill(red(sc), green(sc), blue(sc), 40);
  rect(0, by, width, bh);
  stroke(sc);
  strokeWeight(1);
  line(0, by, width, by);
  noStroke();

  fill(sc);
  textFont(fontMono);
  textSize(12);
  textAlign(CENTER, CENTER);
  
  int elapsed = millis() - lastDataMillis;
  boolean timeout = (lastDataMillis > 0 && elapsed > DATA_TIMEOUT);
  String sigStr = (lastDataMillis == 0) ? "WAITING..." : (timeout ? "NO SIGNAL" : "LIVE SIGNAL");

  text(sigStr + "   |   STATUS: " + parsedConfirmed +
       "   |   VAR: " + nf(parsedVariance, 1, 2) + 
       "   |   PORT: " + activePortName,
       width/2, by + bh/2);
  textAlign(LEFT, BASELINE);
}

void addLog(String msg) {
  logLines[logCount % LOG_MAX] = "[" + nf(hour(),2) + ":" + nf(minute(),2) + ":" + nf(second(),2) + "] " + msg;
  logCount++;
}

color statusToColor(String s) {
  if (s == null) return COL_MUTED;
  switch (s.trim()) {
    case "NORMAL":   return COL_NORMAL;
    case "ELEVATED": return COL_ELEVATED;
    case "WASTE":    return COL_WASTE_MOV;
    case "CRITICAL": return COL_CRITICAL;
    default:         return COL_MUTED;
  }
}

void keyPressed() {
  if (key == 'r' || key == 'R') {
    if (port != null) port.stop();
    serialReady = false;
    selectingPort = true;
    availablePorts = Serial.list();
    portError = false;
    activePortName = "NONE";
  }
}

void mousePressed() {
  if (selectingPort && hoveredPort >= 0 && hoveredPort < availablePorts.length) {
    try {
      port = new Serial(this, availablePorts[hoveredPort], 9600);
      port.bufferUntil('\n');
      serialReady = true;
      selectingPort = false;
      activePortName = availablePorts[hoveredPort];
      addLog("Connected to " + activePortName);
    } catch (Exception e) {
      portError = true;
      portErrorMsg = "Failed to open port";
    }
  }
}
