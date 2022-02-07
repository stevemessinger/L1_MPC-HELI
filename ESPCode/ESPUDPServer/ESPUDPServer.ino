#include <WiFi.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <WebSocketsServer.h>
#include <sstream>
#include <iomanip>

WebSocketsServer webSocket = WebSocketsServer(80);
const char *ssid = "DontConnect";
const char *password = "Pennstate2020";

Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);

std::stringstream AX, AY, AZ, GX, GY, GZ;

int numClients;

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

void setup() {
  Serial.begin(115200);

  /* Initialise the sensor */
  if (!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    abort();
  }

  WiFi.begin(ssid, password);  // ESP-32 as access point
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  
  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
  
  webSocket.begin();
  webSocket.onEvent(onWebSocketEvent);

}

void loop() {
  auto start = millis();
  delay(5);

  imu::Vector<3> gyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
  imu::Vector<3> IMU = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);

  float wx = gyro.x();
  float wy = gyro.y();
  float wz = gyro.z();

  float ax = IMU.x();
  float ay = IMU.y();
  float az = IMU.z();

  if(numClients > 0){
    for(int i = 0; i <= numClients; i++){

          AX.str(std::string());
          AY.str(std::string());
          AZ.str(std::string());
          GX.str(std::string());
          GY.str(std::string());
          GZ.str(std::string());

          AX << std::fixed << std::setprecision(10) << ax;
          AY << std::fixed << std::setprecision(10) << ay;
          AZ << std::fixed << std::setprecision(10) << az;
          GX << std::fixed << std::setprecision(10) << wx;
          GY << std::fixed << std::setprecision(10) << wy;
          GZ << std::fixed << std::setprecision(10) << wz;

          std::string IMUData = AX.str() + ":" + AY.str() + ":" + AZ.str() + ":" +
                                GX.str() + ":" + GY.str() + ":" + GZ.str();
          
          webSocket.sendTXT(i, IMUData.c_str());
    }
  }

  webSocket.loop();
  auto now = millis();
  Serial.print("dt: ");
  Serial.println(now - start);
  start = now;
}
