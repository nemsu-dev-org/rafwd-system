#include <WiFi.h>
#include <WebServer.h>

// ===== AP CONFIG =====
const char* ap_ssid = "ESP32-Server";
const char* ap_password = "12345678";

// ===== SERVER =====
WebServer server(80);

void handleRoot() {
  String html = R"rawliteral(
    <!DOCTYPE html>
    <html>
    <head>
        <meta name="viewport" content="width=device-width, initial-scale=1">
        <title>ESP32</title>
        <style>
            body { font-family: Arial; text-align: center; margin-top: 50px; }
            h1 { color: #333; }
        </style>
    </head>
    <body>
        <h1>Hello World from ESP32 🚀</h1>
    </body>
    </html>
  )rawliteral";
  server.send(200, "text/html", html);
}

void setup() {
  Serial.begin(115200);

  // ===== CREATE AP =====
  WiFi.softAP(ap_ssid, ap_password);
  Serial.println("AP Started!");
  Serial.print("Connect to WiFi: ");
  Serial.println(ap_ssid);
  Serial.print("Then open: http://");
  Serial.println(WiFi.softAPIP());

  server.on("/", handleRoot);
  server.begin();
}

void loop() {
  server.handleClient();
}
