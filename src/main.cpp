#include <main.h>

#include <stdint.h>

#include <Arduino.h>

#include <PubSubClient.h>
#include <WiFi.h>

// esp32 softAP credentials
#define ESP_SSID "ESP_DISCGOLF"
#define ESP_PASS "alarautaan"

// MQTT info
#define MQTT_SERVER "192.168.4.2"

WiFiClient espClient;

PubSubClient mqttClient(espClient);

mpu_meas_t measBuff[64];

uint16_t acc_x_raw, acc_y_raw, acc_z_raw;
uint16_t gyro_x_raw, gyro_y_raw, gyro_z_raw;
uint16_t acc_x_err, acc_y_err, acc_z_err;
uint16_t gyro_x_err, gyro_y_err, gyro_z_err;
uint16_t roll, pitch, yaw;
uint16_t elapsed_time, current_time, previous_t;

void setup() {
   Serial.begin(115200);

   Serial.print("Setting AP (Access Point)…");
   WiFi.softAP(ESP_SSID, ESP_PASS);

   delay(10000);

   mqttClient.setServer(MQTT_SERVER, 1883);

}

void loop() {
   delay(2000);
   if (!mqttClient.connected()) {
      Serial.print("MQTT not connected, reconnecting...\n");
      if(mqttClient.connect("mac")) Serial.print("MQTT connected.\n");
   }

   mqttClient.publish("sensors/acc", "testval");
   
   mqttClient.loop();
}
