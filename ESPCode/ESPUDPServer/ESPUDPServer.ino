#include <WiFi.h>
#include <WiFiUdp.h>

WiFiUDP udp; // Creation of wifi Udp instance

char packetBuffer[255];

unsigned int localPort = 9999;

const char *ssid = "BB9ESERVER";  
const char *password = "BB9ESERVER";

void setup() {
  Serial.begin(115200);
  WiFi.softAP(ssid, password);  // ESP-32 as access point
  Serial.println(WiFi.softAPIP());
  udp.begin(localPort);
  }

void loop() {
  int prevTime = millis(); 
  
  int packetSize = udp.parsePacket();
  
  if (packetSize) {
    udp.read(packetBuffer, 255);
    Serial.print("Recieved(IP/Size/Data): ");
    Serial.print(udp.remoteIP());Serial.print(" / ");
    Serial.print(packetSize);Serial.print(" / ");
    Serial.println(packetBuffer);
     }
     
  Serial.println(millis() - prevTime);
}
