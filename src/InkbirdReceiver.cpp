#include <Arduino.h>
#include "cc1101.h"
#include "inkbird.h"
#include "network.h"

CC1101 cc1101;
Network network;

void setup() {
    Serial.begin(115200);
    cc1101.begin();
    network.begin();
}

void loop() {
    network.keepAlive(); // Keep the network connection alive
    byte* buffer = cc1101.checkFIFO(); // Check for received packets
    if(buffer != nullptr) {
       InkbirdData data = decodePacket(buffer); // Decode and print the packet
       if(data.valid) {
           Serial.printf("Temperature: %.1f°C, Battery: %u%%\n", data.temperature, data.battery);
           network.sendMqtt("probst/outdoor/whirlpool/temp", String(data.temperature, 1)); // Send temperature to MQTT
           network.sendMqtt("probst/outdoor/whirlpool/battery", String(data.battery)); // Send battery level to MQTT
       } else {
           Serial.println("Invalid packet received.");
       }
    }
    delay(100);
}

