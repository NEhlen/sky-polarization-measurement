#include <WiFi.h>
#include <WebServer.h>

#include <ICM_20948.h> // Click here to get the library: http://librarymanager/All#SparkFun_ICM_20948_IMU

const char* ssid = "PhotodiodeESP32";
const char* password = "12345678";

const int sensorPins[4] = {32, 33, 34, 35};
WebServer server(80);


// The value of the last bit of the I2C address.
// On the SparkFun 9DoF IMU breakout the default is 1, and when the ADR jumper is closed the value becomes 0
#define AD0_VAL 1

ICM_20948_I2C myICM; // create an ICM_20948_I2C object

// sensor values
int sensorVals[4] = {0, 0, 0, 0};
float altazrotVals[4] = {0.0, 0.0, 0.0, 0.0};


void handleRoot() {
  String json = "{";
  for (int i = 0; i < 4; i++) {
    json += "\"sensor" + String(i) + "\":" + String(sensorVals[i]);
    if (i < 3) json += ",";
  }
  json += ",";
  json += "\"alt\":" + String(altazrotVals[0]);
  json += ",";
  json += "\"az\":" + String(altazrotVals[1]);
  json += ",";
  json += "\"rot\":" + String(altazrotVals[2]);
  json += ",";
  json += "\"acc\":" + String(altazrotVals[3]);
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

  Wire.begin(21, 22);
  Wire.setClock(400000);

  bool initialized = false;
  while (!initialized)
  {
    // Initialize the ICM-20948
    // If the DMP is enabled, .begin performs a minimal startup. We need to configure the sample mode etc. manually.
    myICM.begin(Wire, AD0_VAL);

    Serial.print(F("Initialization of the sensor returned: "));
    Serial.println(myICM.statusString());
    if (myICM.status != ICM_20948_Stat_Ok)
    {
      delay(500);
    }
    else
    {
        initialized = true;
    }

  }
  bool success = true; // Use success to show if the DMP configuration was successful
  // Initialize the DMP. initializeDMP is a weak function. You can overwrite it if you want to e.g. to change the sample rate
  success &= (myICM.initializeDMP() == ICM_20948_Stat_Ok);

  // DMP sensor options are defined in ICM_20948_DMP.h
  //    INV_ICM20948_SENSOR_ACCELEROMETER               (16-bit accel)
  //    INV_ICM20948_SENSOR_GYROSCOPE                   (16-bit gyro + 32-bit calibrated gyro)
  //    INV_ICM20948_SENSOR_RAW_ACCELEROMETER           (16-bit accel)
  //    INV_ICM20948_SENSOR_RAW_GYROSCOPE               (16-bit gyro + 32-bit calibrated gyro)
  //    INV_ICM20948_SENSOR_MAGNETIC_FIELD_UNCALIBRATED (16-bit compass)
  //    INV_ICM20948_SENSOR_GYROSCOPE_UNCALIBRATED      (16-bit gyro)
  //    INV_ICM20948_SENSOR_STEP_DETECTOR               (Pedometer Step Detector)
  //    INV_ICM20948_SENSOR_STEP_COUNTER                (Pedometer Step Detector)
  //    INV_ICM20948_SENSOR_GAME_ROTATION_VECTOR        (32-bit 6-axis quaternion)
  //    INV_ICM20948_SENSOR_ROTATION_VECTOR             (32-bit 9-axis quaternion + heading accuracy)
  //    INV_ICM20948_SENSOR_GEOMAGNETIC_ROTATION_VECTOR (32-bit Geomag RV + heading accuracy)
  //    INV_ICM20948_SENSOR_GEOMAGNETIC_FIELD           (32-bit calibrated compass)
  //    INV_ICM20948_SENSOR_GRAVITY                     (32-bit 6-axis quaternion)
  //    INV_ICM20948_SENSOR_LINEAR_ACCELERATION         (16-bit accel + 32-bit 6-axis quaternion)
  //    INV_ICM20948_SENSOR_ORIENTATION                 (32-bit 9-axis quaternion + heading accuracy)

  // Enable the DMP orientation sensor
  success &= (myICM.enableDMPSensor(INV_ICM20948_SENSOR_ORIENTATION) == ICM_20948_Stat_Ok);
  // Enable any additional sensors / features
  //success &= (myICM.enableDMPSensor(INV_ICM20948_SENSOR_RAW_GYROSCOPE) == ICM_20948_Stat_Ok);
  //success &= (myICM.enableDMPSensor(INV_ICM20948_SENSOR_RAW_ACCELEROMETER) == ICM_20948_Stat_Ok);
  //success &= (myICM.enableDMPSensor(INV_ICM20948_SENSOR_MAGNETIC_FIELD_UNCALIBRATED) == ICM_20948_Stat_Ok);

  // Configuring DMP to output data at multiple ODRs:
  // DMP is capable of outputting multiple sensor data at different rates to FIFO.
  // Setting value can be calculated as follows:
  // Value = (DMP running rate / ODR ) - 1
  // E.g. For a 5Hz ODR rate when DMP is running at 55Hz, value = (55/5) - 1 = 10.
  success &= (myICM.setDMPODRrate(DMP_ODR_Reg_Quat9, 0) == ICM_20948_Stat_Ok); // Set to the maximum

  // Enable the FIFO
  success &= (myICM.enableFIFO() == ICM_20948_Stat_Ok);

  // Enable the DMP
  success &= (myICM.enableDMP() == ICM_20948_Stat_Ok);

  // Reset DMP
  success &= (myICM.resetDMP() == ICM_20948_Stat_Ok);

  // Reset FIFO
  success &= (myICM.resetFIFO() == ICM_20948_Stat_Ok);

  // Check success
  if (success)
  {
    Serial.println("DMP enabled!");
  } else {
    Serial.println("Enable DMP failed!");
  }
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

  // DMP
  icm_20948_DMP_data_t data;
  myICM.readDMPdataFromFIFO(&data);
  if ((myICM.status == ICM_20948_Stat_Ok) || (myICM.status == ICM_20948_Stat_FIFOMoreDataAvail)) // Was valid data available?
  {
    //SERIAL_PORT.print(F("Received data! Header: 0x")); // Print the header in HEX so we can see what data is arriving in the FIFO
    //if ( data.header < 0x1000) SERIAL_PORT.print( "0" ); // Pad the zeros
    //if ( data.header < 0x100) SERIAL_PORT.print( "0" );
    //if ( data.header < 0x10) SERIAL_PORT.print( "0" );
    //SERIAL_PORT.println( data.header, HEX );

    if ((data.header & DMP_header_bitmap_Quat9) > 0) // We have asked for orientation data so we should receive Quat9
    {
      // Q0 value is computed from this equation: Q0^2 + Q1^2 + Q2^2 + Q3^2 = 1.
      // In case of drift, the sum will not add to 1, therefore, quaternion data need to be corrected with right bias values.
      // The quaternion data is scaled by 2^30.

      //SERIAL_PORT.printf("Quat9 data is: Q1:%ld Q2:%ld Q3:%ld Accuracy:%d\r\n", data.Quat9.Data.Q1, data.Quat9.Data.Q2, data.Quat9.Data.Q3, data.Quat9.Data.Accuracy);

      // Scale to +/- 1
      double q1 = ((double)data.Quat9.Data.Q1) / 1073741824.0; // Convert to double. Divide by 2^30
      double q2 = ((double)data.Quat9.Data.Q2) / 1073741824.0; // Convert to double. Divide by 2^30
      double q3 = ((double)data.Quat9.Data.Q3) / 1073741824.0; // Convert to double. Divide by 2^30
      double q0 = sqrt(1.0 - ((q1 * q1) + (q2 * q2) + (q3 * q3)));

      // calculate alt-az-rot
      double yaw   = atan2(2.0 * (q0*q3 + q1*q2), 1.0 - 2.0 * (q2*q2 + q3*q3));
      double pitch = asin(2.0 * (q0*q2 - q3*q1));
      double roll  = atan2(2.0 * (q0*q1 + q2*q3), 1.0 - 2.0 * (q1*q1 + q2*q2));

      // Convert to degrees:
      yaw   *= 180.0 / PI;
      pitch *= 180.0 / PI;
      roll  *= 180.0 / PI;

      // Normalize yaw to [0, 360)
      if (yaw < 0) yaw += 360.0;

      altazrotVals[0] = pitch;
      altazrotVals[1] = yaw;
      altazrotVals[2] = roll;
      altazrotVals[3] = data.Quat9.Data.Accuracy;

      Serial.print("az:");
      Serial.print(yaw, 1);
      Serial.print("alt");
      Serial.print(pitch, 1);
      Serial.print("roll");
      Serial.print(roll, 1);
      Serial.print("acc");
      Serial.println(data.Quat9.Data.Accuracy);
    }
  }
  server.handleClient();
  delay(100);  // ~10Hz update rate
}
