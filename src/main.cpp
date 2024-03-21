#include <main.h>

#include <stdint.h>

#include <Arduino.h>

#include <PubSubClient.h>
#include <WiFi.h>

#include <CompFilter.hpp>

// esp32 softAP credentials
#define ESP_SSID "ESP_DISCGOLF"
#define ESP_PASS "alarautaan"

#define BUFFER_SIZE 128

#define ACC_WEIGHT   1
#define GYRO_WEIGHT  1

// WiFi and MQTT

#define MQTT_SERVER "192.168.4.2"

WiFiClient espClient;

PubSubClient mqttClient(espClient);

// Data

mpu_meas_t rawMeasBuff[BUFFER_SIZE];
comp_filter_t filteredMeasBuff[BUFFER_SIZE];

unsigned long deltaTime[BUFFER_SIZE];

uint16_t acc_x_raw, acc_y_raw, acc_z_raw;
uint16_t gyro_x_raw, gyro_y_raw, gyro_z_raw;
uint16_t acc_x_err, acc_y_err, acc_z_err;
uint16_t gyro_x_err, gyro_y_err, gyro_z_err;
uint16_t roll, pitch, yaw;
uint16_t delta_time, current_time, previous_t;


   void fillMeasBuffer(mpu_meas_t* rawBuff, uint16_t sampleCount)
   {

      unsigned long elapsedTime;

      // Read raw sensor data

      for (int i = 0;i<sampleCount;i++)
      {
         // read from I2C
         /*
         acc_x_raw =
         acc_y_raw =
         acc_z_raw =

         gyro_x_raw =
         gyro_y_raw =
         gyro_z_raw =
         */

         rawBuff[i].acc_x = acc_x_raw;
         rawBuff[i].acc_y = acc_y_raw;
         rawBuff[i].acc_z = acc_z_raw;

         rawBuff[i].gyro_z = gyro_z_raw;

         deltaTime[i] = millis() - elapsedTime;
         elapsedTime = millis();
         
      }

   }

   void fillFilteredBuffer(comp_filter_t* filter, mpu_meas_t* rawBuff, uint16_t sampleCount)
   {

      float filteredAngle = 0;

      for (int i = 0;i<sampleCount;i++)
      {
         float accAngle = getAccAngle(rawBuff[i].acc_x, rawBuff[i].acc_y, rawBuff[i].acc_z);
         float gyroAngle = getGyroAngle(rawBuff[i].gyro_x, rawBuff[i].gyro_y, rawBuff[i].gyro_z, 1);

         float filteredAngle = getCompAngle(accAngle * ACC_WEIGHT, gyroAngle * GYRO_WEIGHT, filteredAngle, GYRO_WEIGHT);

         filteredMeasBuff[i].val = filteredAngle;
      }
   }


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



   fillMeasBuffer(rawMeasBuff, BUFFER_SIZE);
   fillFilteredBuffer(filteredMeasBuff, rawMeasBuff, BUFFER_SIZE);
}
