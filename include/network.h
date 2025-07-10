#include <Arduino.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include "credentials.h"

class Network {
    public:
        void begin();
        void sendMqtt(const String &topic, const String &message);
        void keepAlive();

    private:
        void setup_wifi();
        void setup_mqtt();
        static void WiFiEvent(WiFiEvent_t event);
        static void callback_mqtt(char *topic, byte *payload, unsigned int length);

        WiFiClient wifiClient;
        PubSubClient mqttclient{wifiClient};
};