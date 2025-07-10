#include "network.h"

void Network::keepAlive()
{
  if (WiFi.status() == WL_CONNECTED && !mqttclient.connected())
  {
    if (mqttclient.connect(MQTT_CLIENT_ID, MQTT_USER, MQTT_PASS))
    {
      Serial.println("[MQTT] connected " + String(MQTT_CLIENT_ID));
      mqttclient.subscribe(MQTT_TOPIC_TEMP);
    }
    else
      Serial.println("[MQTT] connection lost - attempting to reconnect");
  }
  
  mqttclient.loop();
}
void Network::sendMqtt(const String &topic, const String &message) {
    if (WiFi.status() == WL_CONNECTED && mqttclient.connected())
    {
        if (mqttclient.publish(topic.c_str(), message.c_str(), true))
        {
        Serial.println("[MQTT] Message sent [" + topic + "]: " + message);
        }
        else
        {
        Serial.println("[MQTT] Failed to send message [" + topic + "]: " + message);
        }
    }
    else
    {
        Serial.println("[MQTT] Not connected to WiFi or MQTT broker");
    }
}
void Network::begin()
{
  Serial.println("Starting Network");
  setup_wifi();
  setup_mqtt();
}

void Network::WiFiEvent(WiFiEvent_t event)
{
  if (event == ARDUINO_EVENT_WIFI_STA_GOT_IP)
  {
    Serial.println("[WiFi] connected " + WiFi.localIP().toString());
  }
  else if (event == ARDUINO_EVENT_WIFI_STA_DISCONNECTED)
  {
    Serial.println("[WiFi] connection lost - attempting to reconnect");
    WiFi.reconnect();
  }
}
void Network::callback_mqtt(char *topic, byte *payload, unsigned int length)
{
  String incommingMessage = "";
  for (int i = 0; i < length; i++)
    incommingMessage += (char)payload[i];
  //--- check the incomming message
  Serial.println("[MQTT] Message arrived [" + String(topic) + "]: " + incommingMessage);
}
void Network::setup_mqtt()
{
  Serial.println("Setup MQTT");
  mqttclient.setServer(MQTT_SERVER, MQTT_PORT);
  mqttclient.setCallback(callback_mqtt);
}
void Network::setup_wifi()
{
  Serial.println("Setup WiFi");
  WiFi.config(IPAddress(WIFI_IP), IPAddress(192, 168, 100, 1), IPAddress(255, 255, 255, 0));
  WiFi.hostname(WIFI_HOSTNAME);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  WiFi.onEvent(WiFiEvent);
}