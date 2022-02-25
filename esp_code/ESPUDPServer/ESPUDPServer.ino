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

std::stringstream msg;
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

      webSocket.sendTXT(client_num, message.c_str());
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

void setup() {
  Serial.begin(115200);
  WiFi.softAP(ssid, password);
  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.softAPIP());
  if (!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    abort();
  }

  webSocket.begin();
  webSocket.onEvent(onWebSocketEvent);
  start = millis();
}

void loop() {

  gyroVector = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
  IMUVector = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
  bno.getCalibration(&sys, &gyro, &accel, &mag);

  wx = gyroVector.x();
  wy = gyroVector.y();
  wz = gyroVector.z();

  ax = IMUVector.x();
  ay = IMUVector.y();
  az = IMUVector.z();
  msg.str(std::string());

  dt = millis() - start;
  start = millis();

  msg << std::fixed << std::setprecision(5) << ax << ":" << std::fixed << std::setprecision(5) << ay << ":" << std::fixed << std::setprecision(5) << az << ":"
      << std::fixed << std::setprecision(5) << wx << ":" << std::fixed << std::setprecision(5) << wy << ":" << std::fixed << std::setprecision(5) << wz << ":"
      << std::fixed  << int(sys) << ":" << std::fixed  << int(accel) << ":" << std::fixed  << int(gyro) << ":" << std::fixed  << int(mag) << ":" << std::fixed << int(dt);
  message = msg.str();
  webSocket.loop();
}
