#include <WiFi.h>
#include <WebServer.h>
#include <ICM_20948.h>

// WiFi credentials for ESP32 AP mode
const char* ssid = "PhotodiodeESP32";
const char* password = "12345678";

// Analog sensor pins
const int sensorPins[4] = {32, 33, 34, 35};
WebServer server(80);

// I2C address configuration for the IMU
#define AD0_VAL 1
ICM_20948_I2C myICM;

// Logging settings
constexpr size_t MAX_SAMPLES = 2000;  // ~4 minutes @ 10Hz
bool loggingEnabled = false;
size_t currentIndex = 0;

// Data structure for each logged sample
struct Sample {
  float alt, az, rot, acc;     // Orientation data
  int16_t s0, s1, s2, s3;       // Averaged sensor values
};
Sample logBuffer[MAX_SAMPLES];

// Buffers for accumulating sensor data between samples
float sensorSum[4] = {0};
int sensorCount = 0;

unsigned long lastSampleTime = 0;

// HTTP endpoint: default help text
void handleRoot() {
  String html = R"rawliteral(
    <!DOCTYPE html>
    <html>
    <head><meta charset='utf-8'><title>ESP32 Logger</title></head>
    <body>
      <h2>ESP32 Datalogger</h2>
      <p>Status: <span id="status">Stopped</span></p>
      <p>Logged Samples: <span id="sampleCount">0</span> / <span id="totalSamples">2000</span></p>
      <button onclick="fetch('/start').then(updateStatus)">Start Logging</button>
      <button onclick="fetch('/stop').then(updateStatus)">Stop Logging</button>
      <br><br>
      <a href="/log.csv" download><button>Download CSV</button></a>

      <script>
        function updateStatus() {
          fetch('/status')
            .then(res => res.json())
            .then(data => {
              document.getElementById('status').textContent = data.logging ? 'Logging' : 'Stopped';
              document.getElementById('sampleCount').textContent = data.samples;
            });
        }
        setInterval(updateStatus, 1000);
        updateStatus();
      </script>
    </body>
    </html>
  )rawliteral";
  server.send(200, "text/html", html);
}

// HTTP endpoint: start logging
void handleStart() {
  loggingEnabled = true;
  currentIndex = 0;
  myICM.resetFIFO();
  memset(sensorSum, 0, sizeof(sensorSum));
  sensorCount = 0;
  server.send(200, "text/plain", "Logging started");
}

// HTTP endpoint: stop logging
void handleStop() {
  loggingEnabled = false;
  server.send(200, "text/plain", "Logging stopped");
}

// HTTP endpoint: return collected log as CSV
void handleCSV() {
  server.setContentLength(CONTENT_LENGTH_UNKNOWN);
  server.send(200, "text/csv", "");
  server.sendContent("alt,az,rot,acc,s0,s1,s2,s3\n");
  for (size_t i = 0; i < currentIndex; ++i) {
    char line[128];
    snprintf(line, sizeof(line), "%.3f,%.3f,%.3f,%.3f,%d,%d,%d,%d\n",
             logBuffer[i].alt, logBuffer[i].az, logBuffer[i].rot, logBuffer[i].acc,
             logBuffer[i].s0, logBuffer[i].s1, logBuffer[i].s2, logBuffer[i].s3);
    server.sendContent(line);
  }
}

void setup() {
  Serial.begin(115200);

  WiFi.softAP(ssid, password);
  delay(100);
  Serial.print("Access Point IP: ");
  Serial.println(WiFi.softAPIP());

  server.on("/", handleRoot);
  server.on("/start", handleStart);
  server.on("/stop", handleStop);
  server.on("/log.csv", handleCSV);
  server.on("/status", []() {
    String json = "{";
    json += "\"logging\":" + String(loggingEnabled ? "true" : "false") + ",";
    json += "\"samples\":" + String(currentIndex);
    json += "}";
    server.send(200, "application/json", json);
  });
  server.begin();

  Wire.begin(21, 22);
  Wire.setClock(400000);

  bool initialized = false;
  while (!initialized) {
    myICM.begin(Wire, AD0_VAL);
    Serial.print(F("Initialization of the sensor returned: "));
    Serial.println(myICM.statusString());
    if (myICM.status != ICM_20948_Stat_Ok) {
      delay(500);
    } else {
      initialized = true;
    }
  }

  bool success = true;
  success &= (myICM.initializeDMP() == ICM_20948_Stat_Ok);
  success &= (myICM.enableDMPSensor(INV_ICM20948_SENSOR_ORIENTATION) == ICM_20948_Stat_Ok);
  success &= (myICM.setDMPODRrate(DMP_ODR_Reg_Quat9, 2) == ICM_20948_Stat_Ok);
  success &= (myICM.enableFIFO() == ICM_20948_Stat_Ok);
  success &= (myICM.enableDMP() == ICM_20948_Stat_Ok);
  success &= (myICM.resetDMP() == ICM_20948_Stat_Ok);
  success &= (myICM.resetFIFO() == ICM_20948_Stat_Ok);

  if (success) Serial.println("DMP enabled");
  else Serial.println("DMP init failed");
}

void loop() {
  unsigned long now = millis();

  // Always accumulate analog sensor values
  for (int i = 0; i < 4; i++) {
    sensorSum[i] += analogRead(sensorPins[i]);
  }
  sensorCount++;

  if (now - lastSampleTime >= 100) {
    lastSampleTime = now;

    if (loggingEnabled && currentIndex < MAX_SAMPLES) {
      // Compute averaged sensor values
      int averagedVals[4];
      for (int i = 0; i < 4; i++) {
        averagedVals[i] = round(sensorSum[i] / sensorCount);
        sensorSum[i] = 0;
      }
      sensorCount = 0;

      float alt = 0, az = 0, rot = 0, acc = 0;
      bool imuDataValid = false;

      icm_20948_DMP_data_t data;
      myICM.readDMPdataFromFIFO(&data);

      if ((myICM.status == ICM_20948_Stat_Ok || myICM.status == ICM_20948_Stat_FIFOMoreDataAvail)
          && (data.header & DMP_header_bitmap_Quat9)) {
        double q1 = ((double)data.Quat9.Data.Q1) / 1073741824.0;
        double q2 = ((double)data.Quat9.Data.Q2) / 1073741824.0;
        double q3 = ((double)data.Quat9.Data.Q3) / 1073741824.0;
        double q_squared_sum = q1*q1 + q2*q2 + q3*q3;
        double q0 = 0;
        if (q_squared_sum <= 1.0) {
          q0 = sqrt(1.0 - q_squared_sum);
          az   = atan2(2.0 * (q0*q3 + q1*q2), 1.0 - 2.0 * (q2*q2 + q3*q3)) * 180.0 / PI;
          alt  = asin(2.0 * (q0*q2 - q3*q1)) * 180.0 / PI;
          rot  = atan2(2.0 * (q0*q1 + q2*q3), 1.0 - 2.0 * (q1*q1 + q2*q2)) * 180.0 / PI;
          if (az < 0) az += 360.0;
          acc = data.Quat9.Data.Accuracy;
          imuDataValid = true;
        }
      }

      // Store sample only if valid IMU data was read
      if (imuDataValid) {
        logBuffer[currentIndex++] = {
          alt, az, rot, acc,
          (int16_t)averagedVals[0], (int16_t)averagedVals[1],
          (int16_t)averagedVals[2], (int16_t)averagedVals[3]
        };
      }
    }
  }

  server.handleClient();
}
