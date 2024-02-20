#include "stdint.h"

#define MPU6050_ADDR = 0x68

typedef struct
{
   uint16_t acc_x;
   uint16_t acc_y;
   uint16_t acc_z;
   uint16_t gyro_x;
   uint16_t gyro_y;
   uint16_t gyro_z;
}mpu_meas_t;