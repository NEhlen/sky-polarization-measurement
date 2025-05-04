#include <WiFi.h>
#include <WebServer.h>

const char* ssid = "PhotodiodeESP32";
const char* password = "12345678";

const int sensorPins[4] = {32, 33, 34, 35};
WebServer server(80);

// sensor values
int sensorVals[4] = {0, 0, 0, 0};

void handleRoot() {
  String json = "{";
  for (int i = 0; i < 4; i++) {
    json += "\"sensor" + String(i) + "\":" + String(sensorVals[i]);
    if (i < 3) json += ",";
  }
  json += "}";
  server.send(200, "application/json", json);
}

void setup() {
  Serial.begin(115200);
  WiFi.softAP(ssid, password);
  delay(100);
  Serial.print("Access Point IP: ");
  Serial.println(WiFi.softAPIP());
  server.on("/", handleRoot);
  server.begin();
  Serial.println("Web server started");
}

void loop() {
  // read sensors
  for (int i = 0; i < 4; i++) {
    sensorVals[i] = analogRead(sensorPins[i]);
  }

  // print to serial plotter
  for (int i = 0; i < 4; i++) {
    Serial.print(sensorVals[i]);
    if (i < 3) Serial.print(" ");
    else Serial.println();
  }

  server.handleClient();
  delay(100);  // ~10Hz update rate
}

