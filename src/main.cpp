#include <main.h>

#include <stdint.h>

#include <Arduino.h>

mpu_meas_t measBuff[64];

uint16_t acc_x_raw, acc_y_raw, acc_z_raw;
uint16_t gyro_x_raw, gyro_y_raw, gyro_z_raw;
uint16_t acc_x_err, acc_y_err, acc_z_err;
uint16_t gyro_x_err, gyro_y_err, gyro_z_err;
uint16_t roll, pitch, yaw;
uint16_t elapsed_time, current_time, previous_t;

void setup() {
  Serial.begin(115200);
}

void loop() {
  Serial.write("test\n");
}
