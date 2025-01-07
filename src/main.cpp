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

TwoWire I2CMPU = TwoWire(0);
Adafruit_MPU6050 mpu;

TwoWire I2CDRV = TwoWire(1);
Adafruit_DRV2605 drv;

int pinBuzzer = 39;
int enablePinBuzzer = 15;

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

volatile float accMagnitude;
volatile float accX = 0;
volatile float accY = 0;
volatile float accZ = 0;
float accXError, accYError, accZError;
float gyroX, gyroY, gyroZ;
float gyroXError, gyroYError, gyroZError;

void callback(char* topic, byte* message, unsigned int length) {
   String messageTemp;
   for (int i = 0; i < length; i++) {
      Serial.print((char)message[i]);
      messageTemp += (char)message[i];
   }
   if (String(topic) == "game/hitresult")
   {
      if(messageTemp == "1")
      {
         // Play high note for basket hit
         ledcSetup(pwmChannelBuzzer, 500, resolutionBuzzer);
         ledcAttachPin(pinBuzzer, pwmChannelBuzzer);
         // play short double vibration
         drv.setWaveform(0, 86);
         drv.setWaveform(1, 86);
         drv.setWaveform(2, 0);
         digitalWrite(ledGreen, HIGH);
      }
      else if(messageTemp == "0")
      {
         // Play low note for basket miss
         ledcSetup(pwmChannelBuzzer, 300, resolutionBuzzer);
         ledcAttachPin(pinBuzzer, pwmChannelBuzzer);
         // play lower vibration
         drv.setWaveform(0, 79);
         drv.setWaveform(1, 79);
         drv.setWaveform(2, 0);
         digitalWrite(ledRed, HIGH);
      }
      ledcWrite(pwmChannelBuzzer, dutyCycleBuzzer);
      drv.go();
      delay(1500);
      ledcDetachPin(pinBuzzer);
      digitalWrite(ledGreen, LOW);
      digitalWrite(ledRed, LOW);
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
      I2CMPU.requestFrom(MPU_ADDR, 6, true);
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
      I2CMPU.requestFrom(MPU_ADDR, 6, true);
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

   // Try to initialize MPU
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

   // Reset configuration
   I2CMPU.beginTransmission(MPU_ADDR);
   I2CMPU.write(0x6B);
   I2CMPU.write(0x00);
   I2CMPU.endTransmission(true);


   // Configure Accelerometer Sensitivity
   I2CMPU.beginTransmission(MPU_ADDR);
   I2CMPU.write(0x1C);
   I2CMPU.write(0x10);
   I2CMPU.endTransmission(true);

   // Configure Gyro Sensitivity
   I2CMPU.beginTransmission(MPU_ADDR);
   I2CMPU.write(0x1B);
   I2CMPU.write(0x10);
   I2CMPU.endTransmission(true);
   delay(20);

   drv.selectLibrary(1);
   drv.setMode(DRV2605_MODE_INTTRIG);

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

   delay(300);

   mqttClient.setServer(MQTT_SERVER, 1883);
   mqttClient.setCallback(callback);

   calibrateSensor();
}

void loop() {
   // Read accelerometer data if trigger is pressed down
   bool isTriggerPressed = !digitalRead(21); // pin outputs 0 when pressed down

   // Push buttons
   bool isDebugPressed = !digitalRead(1);
   bool isCalibPressed = !digitalRead(40);

   char accPrint[256] = "";

   if (!mqttClient.connected()) {
         Serial.print("MQTT not connected, reconnecting...\n");
         if(mqttClient.connect("mac"))
         {
            Serial.print("MQTT connected.\n");
            mqttClient.subscribe("game/hitresult");
         }
         
   }
   mqttClient.loop();

   if (isTriggerPressed)
   {
      lastTriggerState = true;
      digitalWrite(ledYellow, HIGH);
      
      // ACCELEROMETER
      I2CMPU.beginTransmission(MPU_ADDR);
      I2CMPU.write(0x3B); // Start with register 0x3B (ACCEL_XOUT_H)
      I2CMPU.endTransmission(false);
      I2CMPU.requestFrom(MPU_ADDR, 6, true);
      accX = (int16_t)((I2CMPU.read() << 8) | I2CMPU.read()) / (4096.0) - accXError;
      accY = (int16_t)((I2CMPU.read() << 8) | I2CMPU.read()) / (4096.0) - accYError;
      accZ = (int16_t)((I2CMPU.read() << 8) | I2CMPU.read()) / (4096.0) - accZError;

      accZ -= 1; // eliminate error due to gravitation

      accMagnitude = sqrt(pow(accX, 2) + pow(accY, 2) + pow(accZ, 2));

      sprintf(accPrint, "%f,%f,%f,%f\n", accX, accY, accZ, accMagnitude);

      // GYROSCOPE
      I2CMPU.beginTransmission(MPU_ADDR);
      I2CMPU.write(0x43); // Start with register 0x43 (GYRO_XOUT_H)
      I2CMPU.endTransmission(false);
      I2CMPU.requestFrom(MPU_ADDR, 6, true);
      gyroX = (int16_t)((I2CMPU.read() << 8) | I2CMPU.read()) / (4096.0) - gyroXError;
      gyroY = (int16_t)((I2CMPU.read() << 8) | I2CMPU.read()) / (4096.0) - gyroYError;
      gyroZ = (int16_t)((I2CMPU.read() << 8) | I2CMPU.read()) / (4096.0) - gyroZError;

      mqttClient.publish("sensors/acc", accPrint);

   }
   else if (lastTriggerState && !isTriggerPressed) { // After trigger is released vibrate the moter and set yellow led low
      lastTriggerState = false;
      drv.setWaveform(0, 90);
      drv.setWaveform(1, 0);
      drv.go();

      digitalWrite(ledYellow, LOW);
   }
   else {
      digitalWrite(ledYellow, LOW);
   }

   delay(100);

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