#include <main.h>

#include <stdint.h>

#include <Arduino.h>

#include <PubSubClient.h>
#include <WiFi.h>

#include <CompFilter.hpp>

#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

// esp32 softAP credentials
#define ESP_SSID "ESP_DISCGOLF"
#define ESP_PASS "alarautaan"

#define MPU_ADDR 0x68

#define BUFFER_SIZE 128

#define ACC_WEIGHT   1
#define GYRO_WEIGHT  1

TwoWire I2CMPU = TwoWire(0);
Adafruit_MPU6050 mpu;


// PWM

int pinMotor = 27;
int enablePinMotor = 14;

int pinBuzzer = 39;
int enablePinBuzzer = 15;

// PWM properties
const int freqMotor = 20;
const int pwmChannelMotor = 1;
const int resolutionMotor = 8;
int dutyCycleMotor = 200;

const int freqBuzzer = 500;
const int pwmChannelBuzzer = 0;
const int resolutionBuzzer = 8;
int dutyCycleBuzzer = 100;

// WiFi and MQTT

#define MQTT_SERVER "192.168.4.2"

WiFiClient espClient;

PubSubClient mqttClient(espClient);

// Data


mpu_meas_t rawMeasBuff[BUFFER_SIZE];
comp_filter_t filteredMeasBuff[BUFFER_SIZE];

unsigned long deltaTime[BUFFER_SIZE];

float accX, accY, accZ;
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
   Serial.print("MPU6050 test...");

   I2CMPU.begin(9, 10, 100000);

   // Try to initialize
   if (!mpu.begin(MPU_ADDR, &I2CMPU)) {
      Serial.print("Failed to find MPU6050 chip");
      while (1) {
         delay(10);
      }
  }
  Serial.print("MPU6050 Found!");

  Serial.print("Configuring acc and gyro");

   
   I2CMPU.beginTransmission(MPU_ADDR);       // Start communication with MPU6050
   I2CMPU.write(0x6B);                  // Talk to the register 6B
   I2CMPU.write(0x00);                  // Make reset - place a 0 into the 6B register
   I2CMPU.endTransmission(true);        //end the transmission
   
  
   // Configure Accelerometer Sensitivity - Full Scale Range (default +/- 2g)
   I2CMPU.beginTransmission(MPU_ADDR);
   I2CMPU.write(0x1C);                  //Talk to the ACCEL_CONFIG register (1C hex)
   I2CMPU.write(0x10);                  //Set the register bits as 00010000 (+/- 8g full scale range)
   I2CMPU.endTransmission(true);
   // Configure Gyro Sensitivity - Full Scale Range (default +/- 250deg/s)
   I2CMPU.beginTransmission(MPU_ADDR);
   I2CMPU.write(0x1B);                   // Talk to the GYRO_CONFIG register (1B hex)
   I2CMPU.write(0x10);                   // Set the register bits as 00010000 (1000deg/s full scale)
   I2CMPU.endTransmission(true);
   delay(20);
  

   // GPIO setup
   
   // Motor
   //ledcSetup(pwmChannelMotor, freqMotor, resolutionMotor);
   //ledcAttachPin(pinMotor, pwmChannelMotor);
   

   // Buzzer
   //ledcSetup(pwmChannelBuzzer, freqBuzzer, resolutionBuzzer);
   //ledcAttachPin(pinBuzzer, pwmChannelBuzzer);

   // Trigger
   pinMode(21, INPUT);

   Serial.print("Setting AP (Access Point)…");
   WiFi.softAP(ESP_SSID, ESP_PASS);

   delay(1000);

   mqttClient.setServer(MQTT_SERVER, 1883);
}

void loop() {
   // Read accelerometer data if trigger is pressed down
   bool isTriggerPressed = !digitalRead(21); // pin outputs 0 when pressed down
   
   accX = 0;
   accY = 0;
   accZ = 0;

   char accPrint[256] = "";

   Serial.print(isTriggerPressed);

   if (isTriggerPressed)
   {
      I2CMPU.beginTransmission(MPU_ADDR);
      I2CMPU.write(0x3B); // Start with register 0x3B (ACCEL_XOUT_H)
      I2CMPU.endTransmission(false);
      I2CMPU.requestFrom(MPU_ADDR, 6, true); // Read 6 registers total, each axis value is stored in 2 registers
      //For a range of +-8g, we need to divide the raw values by 4096
      accX = (int16_t)((I2CMPU.read() << 8) | I2CMPU.read()) / (4096.0);
      accY = (int16_t)((I2CMPU.read() << 8) | I2CMPU.read()) / (4096.0);
      accZ = (int16_t)((I2CMPU.read() << 8) | I2CMPU.read()) / (4096.0);

      accZ -= 1; // eliminate error due to gravitation

      float accMagnitude = sqrt(pow(accX, 2) + pow(accY, 2) + pow(accZ, 2));

      sprintf(accPrint, "%f,%f,%f,%f\n", accX, accY, accZ, accMagnitude);
      //Serial.print(accPrint);
   }
   
   if (!mqttClient.connected()) {
      Serial.print("MQTT not connected, reconnecting...\n");
      if(mqttClient.connect("mac")) Serial.print("MQTT connected.\n");
   }

   mqttClient.publish("sensors/acc", accPrint);
   
   mqttClient.loop();
   
   delay(100);

   //ledcWrite(pwmChannelBuzzer, dutyCycleBuzzer); buzzer toimii

}
