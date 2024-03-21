#include <stdint.h>

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

typedef struct
{
   float val;
}comp_filter_t;

void fillMeasBuffer(mpu_meas_t* rawBuff, uint16_t sampleCount);
void fillFilteredBuffer(comp_filter_t* filter, mpu_meas_t* rawBuff, uint16_t sampleCount);