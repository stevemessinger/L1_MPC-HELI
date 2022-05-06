/*
 Name:		ESPController.ino
 Created:	3/21/2022 11:01:02 AM
 Author:	matthew
*/

#include <sbus.h>
#include <WiFi.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <WebSocketsServer.h>
#include <sstream>
#include <iomanip>

//Websocket Globals
WebSocketsServer webSocket = WebSocketsServer(80);
const char* ssid = "ESP32";
const char* password = "ESP32";
std::stringstream msg;
std::string message;
int numClients;

//IMU globals
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);
uint8_t sys, gyro, accel, mag;
float wx, wy, wz, ax, ay, az;
imu::Vector<3> gyroVector, IMUVector;
int start, dt;

//FreeRTOS Globals
static SemaphoreHandle_t mutex;
int webSocketStartTime, IMUStartTime;

//Websocket Callback
void onWebSocketEvent(const uint8_t client_num,
    const WStype_t type,
    const uint8_t* payload,
    const size_t length) {

    // Figure out the type of WebSocket event
    switch (type) {

        // Client has disconnected
    case WStype_DISCONNECTED:
    {
        Serial.printf("[%u] Disconnected!\n", client_num);
        numClients -= 1;
    }break;


    // New client has connected
    case WStype_CONNECTED:
    {
        IPAddress ip = webSocket.remoteIP(client_num);
        Serial.printf("[%u] Connection from ", client_num);
        Serial.println(ip.toString());
        numClients += 1;
    }break;


    // Handle text messages from client
    case WStype_TEXT:// upon reciving a message, parse channel commands and send IMU data back
    {
        xSemaphoreTake(mutex, portMAX_DELAY);
        //send the IMU data back to the client
        webSocket.sendTXT(client_num, message.c_str());
        xSemaphoreGive(mutex);
    }break;

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

void websocketTask(void* parameter) {
    for (;;) {
        /// !!ENTERING CRITIAL SECTION!!
        //xSemaphoreTake(mutex, portMAX_DELAY);
        webSocket.loop(); // perform websocket function
 
        Serial.print("WebSocket dt: ");
        Serial.println(millis() - webSocketStartTime);
        webSocketStartTime = millis();
        //xSemaphoreGive(mutex);
        /// !!ENDING CRITIAL SECTION!! 
    }
}

// the setup function runs once when you press reset or power the board
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
    webSocketStartTime = millis();
    IMUStartTime = millis();


    mutex = xSemaphoreCreateMutex();

    xTaskCreate(websocketTask,
        "websocketTask",
        10000,
        NULL,
        1,
        NULL);
}

// the loop function runs over and over again until power down or reset
void loop() {
    // Get data from the IMU and form the message (more efficient this way)
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
        << std::fixed << int(sys) << ":" << std::fixed << int(accel) << ":" << std::fixed << int(gyro) << ":" << std::fixed << int(mag) << ":" << std::fixed << int(dt);

    /// !!ENTERING CRITAL SECTION!!
    xSemaphoreTake(mutex, portMAX_DELAY);
    message = msg.str();
    xSemaphoreGive(mutex);
    //Serial.println(message.c_str());

    /// !!END CRITICAL SECTION!!

    Serial.print("IMU dt: ");
    Serial.println(millis() - IMUStartTime);
    IMUStartTime = millis();
}
