#include <main.h>

#include <stdint.h>

#include <Arduino.h>

#include <PubSubClient.h>
#include <WiFi.h>

#include <CompFilter.hpp>

#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_DRV2605.h>
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

TwoWire I2CDRV = TwoWire(1);
Adafruit_DRV2605 drv;

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

// LEDs
int ledGreen = 38;
int ledYellow = 48;
int ledRed = 47;

int ledPower0 = 15;
int ledPower1 = 16;
int ledPower2 = 17;
int ledPower3 = 18;

int pinVoltage = 4;
int voltage = 0;

bool lastTriggerState = false;

// WiFi and MQTT

#define MQTT_SERVER "192.168.4.2"

WiFiClient espClient;

PubSubClient mqttClient(espClient);

// Data


mpu_meas_t rawMeasBuff[BUFFER_SIZE];
comp_filter_t filteredMeasBuff[BUFFER_SIZE];

unsigned long deltaTime[BUFFER_SIZE];

float accX, accY, accZ;
float accXError, accYError, accZError;
float gyroX, gyroY, gyroZ;
float gyroXError, gyroYError, gyroZError;
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

void calibrateSensor()
{
   float accXSum = 0;
   float accYSum = 0;
   float accZSum = 0;
   float gyroXSum = 0;
   float gyroYSum = 0;
   float gyroZSum = 0;

   uint16_t numOfSamples = 200;

   for (int i = 0;i<numOfSamples;i++)
   {
      // ACCELEROMETER
      I2CMPU.beginTransmission(MPU_ADDR);
      I2CMPU.write(0x3B); // Start with register 0x3B (ACCEL_XOUT_H)
      I2CMPU.endTransmission(false);
      I2CMPU.requestFrom(MPU_ADDR, 6, true); // Read 6 registers total, each axis value is stored in 2 registers
      //For a range of +-8g, we need to divide the raw values by 4096
      accX = (int16_t)((I2CMPU.read() << 8) | I2CMPU.read()) / (4096.0);
      accY = (int16_t)((I2CMPU.read() << 8) | I2CMPU.read()) / (4096.0);
      accZ = (int16_t)((I2CMPU.read() << 8) | I2CMPU.read()) / (4096.0);

      accZ -= 1; // eliminate error due to gravitation

      accXSum += accX;
      accYSum += accY;
      accZSum += accZ;

      float accMagnitude = sqrt(pow(accX, 2) + pow(accY, 2) + pow(accZ, 2));

      // GYROSCOPE
      I2CMPU.beginTransmission(MPU_ADDR);
      I2CMPU.write(0x43); // Start with register 0x43 (GYRO_XOUT_H)
      I2CMPU.endTransmission(false);
      I2CMPU.requestFrom(MPU_ADDR, 6, true); // Read 6 registers total, each axis value is stored in 2 registers
      //For a range of +-8g, we need to divide the raw values by 4096
      gyroX = (int16_t)((I2CMPU.read() << 8) | I2CMPU.read()) / (4096.0);
      gyroY = (int16_t)((I2CMPU.read() << 8) | I2CMPU.read()) / (4096.0);
      gyroZ = (int16_t)((I2CMPU.read() << 8) | I2CMPU.read()) / (4096.0);

      gyroXSum += gyroX;
      gyroYSum += gyroY;
      gyroZSum += gyroZ;
   }
   accXError = accXSum / numOfSamples;
   accYError = accYSum / numOfSamples;
   accZError = accZSum / numOfSamples;

   gyroXError = gyroXSum / numOfSamples;
   gyroYError = gyroYSum / numOfSamples;
   gyroZError = gyroZSum / numOfSamples;

   Serial.print("Calibration completed.\n");
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


   Serial.println("DRV2605 test");
   I2CDRV.begin(13, 14, 100000);

   // Try to initialize DRV
   if (!drv.begin(&I2CDRV)) {
      Serial.print("Could not find DRV2605");
      while (1) {
         delay(10);
      }
   }
   Serial.println("DRV2605 found");

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


   drv.selectLibrary(1);
   drv.setMode(DRV2605_MODE_INTTRIG);

   // GPIO setup

   // Motor
   //ledcSetup(pwmChannelMotor, freqMotor, resolutionMotor);
   //ledcAttachPin(pinMotor, pwmChannelMotor);


   // Buzzer
   //ledcSetup(pwmChannelBuzzer, freqBuzzer, resolutionBuzzer);
   //ledcAttachPin(pinBuzzer, pwmChannelBuzzer);

   // Trigger
   pinMode(21, INPUT);

   // Push buttons
   pinMode(1, INPUT);
   pinMode(40, INPUT);

   // LEDs
   pinMode(ledGreen, OUTPUT);
   pinMode(ledYellow, OUTPUT);
   pinMode(ledRed, OUTPUT);

   pinMode(ledPower0, OUTPUT);
   pinMode(ledPower1, OUTPUT);
   pinMode(ledPower2, OUTPUT);
   pinMode(ledPower3, OUTPUT);

   // Set all LEDs inital value to low
   digitalWrite(ledGreen, LOW);
   digitalWrite(ledYellow, LOW);
   digitalWrite(ledRed, LOW);
   digitalWrite(ledPower0, LOW);
   digitalWrite(ledPower1, LOW);
   digitalWrite(ledPower2, LOW);
   digitalWrite(ledPower3, LOW);

   // Voltage measurement
   pinMode(pinVoltage, INPUT);

   Serial.print("Setting AP (Access Point)…");
   WiFi.softAP(ESP_SSID, ESP_PASS);

   delay(1000);

   mqttClient.setServer(MQTT_SERVER, 1883);

   calibrateSensor();
}

void loop() {
   // Read accelerometer data if trigger is pressed down
   bool isTriggerPressed = !digitalRead(21); // pin outputs 0 when pressed down

   // Push buttons
   bool isDebugPressed = !digitalRead(1);
   bool isCalibPressed = !digitalRead(40);

   accX = 0;
   accY = 0;
   accZ = 0;

   char accPrint[256] = "";

   //Serial.print(isTriggerPressed);

   if (isTriggerPressed)
   {
      lastTriggerState = true;
      digitalWrite(ledYellow, HIGH);
      // ACCELEROMETER
      I2CMPU.beginTransmission(MPU_ADDR);
      I2CMPU.write(0x3B); // Start with register 0x3B (ACCEL_XOUT_H)
      I2CMPU.endTransmission(false);
      I2CMPU.requestFrom(MPU_ADDR, 6, true); // Read 6 registers total, each axis value is stored in 2 registers
      //For a range of +-8g, we need to divide the raw values by 4096
      accX = (int16_t)((I2CMPU.read() << 8) | I2CMPU.read()) / (4096.0) - accXError;
      accY = (int16_t)((I2CMPU.read() << 8) | I2CMPU.read()) / (4096.0) - accYError;
      accZ = (int16_t)((I2CMPU.read() << 8) | I2CMPU.read()) / (4096.0) - accZError;

      accZ -= 1; // eliminate error due to gravitation

      float accMagnitude = sqrt(pow(accX, 2) + pow(accY, 2) + pow(accZ, 2));

      sprintf(accPrint, "%f,%f,%f,%f\n", accX, accY, accZ, accMagnitude);
      //Serial.print(accPrint);

      // GYROSCOPE
      I2CMPU.beginTransmission(MPU_ADDR);
      I2CMPU.write(0x43); // Start with register 0x43 (GYRO_XOUT_H)
      I2CMPU.endTransmission(false);
      I2CMPU.requestFrom(MPU_ADDR, 6, true); // Read 6 registers total, each axis value is stored in 2 registers
      //For a range of +-8g, we need to divide the raw values by 4096
      gyroX = (int16_t)((I2CMPU.read() << 8) | I2CMPU.read()) / (4096.0) - gyroXError;
      gyroY = (int16_t)((I2CMPU.read() << 8) | I2CMPU.read()) / (4096.0) - gyroYError;
      gyroZ = (int16_t)((I2CMPU.read() << 8) | I2CMPU.read()) / (4096.0) - gyroZError;

      if (!mqttClient.connected()) {
         Serial.print("MQTT not connected, reconnecting...\n");
         if(mqttClient.connect("mac")) Serial.print("MQTT connected.\n");
      }

      mqttClient.publish("sensors/acc", accPrint);

      mqttClient.loop();
   } else if (lastTriggerState && !isTriggerPressed) { // After trigger is released vibrate the moter and set yellow led low
      lastTriggerState = false;

      drv.setWaveform(0, 90);
      drv.setWaveform(1, 0);
      drv.go();

      digitalWrite(ledYellow, LOW);
   } else {
      digitalWrite(ledYellow, LOW);
   }


   delay(100);

   //ledcWrite(pwmChannelBuzzer, dutyCycleBuzzer); buzzer toimii

   // Voltage measurement if calibration button is pressed
   if (isCalibPressed)
   {
      voltage = analogRead(pinVoltage);
      Serial.println(voltage);
      // 75-100%
      if (voltage > 3313) {
         digitalWrite(ledPower0, HIGH);
         digitalWrite(ledPower1, HIGH);
         digitalWrite(ledPower2, HIGH);
         digitalWrite(ledPower3, HIGH);
      // 50-75%
      } else if (voltage > 2904) {
         digitalWrite(ledPower0, HIGH);
         digitalWrite(ledPower1, HIGH);
         digitalWrite(ledPower2, HIGH);
         digitalWrite(ledPower3, LOW);
      // 25-50%
      } else if (voltage > 2495) {
         digitalWrite(ledPower0, HIGH);
         digitalWrite(ledPower1, HIGH);
         digitalWrite(ledPower2, LOW);
         digitalWrite(ledPower3, LOW);
      // 0-25%
      } else {
         digitalWrite(ledPower0, HIGH);
         digitalWrite(ledPower1, LOW);
         digitalWrite(ledPower2, LOW);
         digitalWrite(ledPower3, LOW);
      }
   } else {
         digitalWrite(ledPower0, LOW);
         digitalWrite(ledPower1, LOW);
         digitalWrite(ledPower2, LOW);
         digitalWrite(ledPower3, LOW);
   }

}