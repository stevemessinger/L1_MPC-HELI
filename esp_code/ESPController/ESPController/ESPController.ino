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

//SBUS globals
bfs::SbusRx sbus_rx(&Serial2);
bfs::SbusTx sbus_tx(&Serial2);
std::array<int16_t, bfs::SbusRx::NUM_CH()> sbus_data;
int commands[4] = { 0, 0, 0, 0 };

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
        // convert payload to char array
        char* rcvMessage; // not sure why the compiler doesnt like this, length is const
        for (int i = 0; i < length; i++) {
            rcvMessage[i] = *(payload + i);
        }
        
        //Tokenize the received message 
        char* token;
        int i = 0;
        token = strtok(rcvMessage, ":");
        while (token != NULL) {
            commands[i] = atoi(token);
            token = strtok(NULL, ":");
            i++;
        }

        //send the IMU data back to the client
        webSocket.sendTXT(client_num, message.c_str());
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
    start = millis();

    sbus_rx.Begin(16,17);
    sbus_tx.Begin(16,17);
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
    message = msg.str();

    //Send sbus packet
    sbus_data[0] = commands[1]; // roll (A)
    sbus_data[1] = commands[2]; // pitch (E)
    sbus_data[2] = 1800; // throttle (T)
    sbus_data[3] = commands[3]; // yaw (R)
    sbus_data[4] = 1500; // gyro (needs to be in 3D mode) (G)
    sbus_data[5] = commands[0]; // collective (Col - Pitch)
    sbus_tx.ch(sbus_data);
    sbus_tx.Write();

    // perform websocket application
    webSocket.loop();
}
