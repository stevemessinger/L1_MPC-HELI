#include <WiFi.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <WebSocketsServer.h>
#include <sstream>
#include <iomanip>

WebSocketsServer webSocket = WebSocketsServer(80);
const char *ssid = "ESP32";
const char *password = "ESP32";

Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);

std::stringstream AX, AY, AZ, GX, GY, GZ, calSystem, calAccel, calGyro, calMag, dtStr;
std::string message;

int numClients;

uint8_t sys, gyro, accel, mag;

float wx, wy, wz, ax, ay, az;
imu::Vector<3> gyroVector, IMUVector;

int start, dt;

// Callback: receiving any WebSocket message
void onWebSocketEvent(uint8_t client_num,
                      WStype_t type,
                      uint8_t * payload,
                      size_t length) {

  // Figure out the type of WebSocket event
  switch (type) {

    // Client has disconnected
    case WStype_DISCONNECTED:
      Serial.printf("[%u] Disconnected!\n", client_num);
      numClients -= 1;
      break;

    // New client has connected
    case WStype_CONNECTED:
      {
        IPAddress ip = webSocket.remoteIP(client_num);
        Serial.printf("[%u] Connection from ", client_num);
        Serial.println(ip.toString());
        numClients += 1;
      }
      break;

    // Handle text messages from client
    case WStype_TEXT:

      break;

    // For everything else: do nothing
    case WStype_BIN:
    case WStype_ERROR:
    case WStype_FRAGMENT_TEXT_START:
    case WStype_FRAGMENT_BIN_START:
    case WStype_FRAGMENT:
    case WStype_FRAGMENT_FIN:
    default:
      break;
  }
}

// task for performing wifi tasks
void webSocketTask(void* pvParameters) {

  /*
  WiFi.begin(ssid, password);  // ESP-32 as access point
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  */

  WiFi.softAP(ssid, password);
  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.softAPIP());

  webSocket.begin();
  webSocket.onEvent(onWebSocketEvent);

  for (;;) {
    if (numClients > 0) {
      for (int i = 0; i <= numClients; i++) {

        AX.str(std::string());
        AY.str(std::string());
        AZ.str(std::string());
        GX.str(std::string());
        GY.str(std::string());
        GZ.str(std::string());
        calSystem.str(std::string());
        calAccel.str(std::string());
        calGyro.str(std::string());
        calMag.str(std::string());
        dtStr.str(std::string());


        AX << std::fixed << std::setprecision(10) << ax;
        AY << std::fixed << std::setprecision(10) << ay;
        AZ << std::fixed << std::setprecision(10) << az;
        GX << std::fixed << std::setprecision(10) << wx;
        GY << std::fixed << std::setprecision(10) << wy;
        GZ << std::fixed << std::setprecision(10) << wz;
        calSystem << std::fixed  << int(sys);
        calAccel << std::fixed  << int(accel);
        calGyro << std::fixed  << int(gyro);
        calMag << std::fixed  << int(mag);
        dtStr <<std::fixed << int(dt);

        message = AX.str() + ":" + AY.str() + ":" + AZ.str() + ":" +
                  GX.str() + ":" + GY.str() + ":" + GZ.str() + ":" +
                  calSystem.str() + ":" + calAccel.str() + ":" + calGyro.str() + ":" + calMag.str() + ":" +
                  dtStr.str();
        webSocket.sendTXT(i, message.c_str());
      }
    }
    webSocket.loop();
  }
}

// task for collecting/updating IMU data
void dataCollectionTask(void* pvParameters) {
  start = millis();
  /* Initialise the sensor */
  if (!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    abort();
  }

  for (;;) {
    gyroVector = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
    IMUVector = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
    bno.getCalibration(&sys, &gyro, &accel, &mag);

    wx = gyroVector.x();
    wy = gyroVector.y();
    wz = gyroVector.z();

    ax = IMUVector.x();
    ay = IMUVector.y();
    az = IMUVector.z();

    dt = millis()-start;
    start = millis();
  }
}

void setup() {
  Serial.begin(115200);

  // create task for wifi operations (wifi should only be on core 0)
  xTaskCreate(
    webSocketTask,
    "WifiTask",
    10000,
    NULL,
    1,
    NULL
  );

  //create task for IMU data collection operations (core 1 is free!)
  xTaskCreate(
    dataCollectionTask,
    "IMUTask",
    10000,
    NULL,
    1,
    NULL
  );
}

void loop() {
  delay(1);
  // nothing in loop, everything done in tasks
}
