#include <CompFilter.hpp>

#include <stdint.h>

float getAccAngle(uint16_t x_raw, uint16_t y_raw, uint16_t z_raw)
{
   return (float)(x_raw/y_raw * 180/3.14159);
}

float getGyroAngle(uint16_t x_raw, uint16_t y_raw, uint16_t z_raw, double dt)
{
   return (float)(z_raw * dt);
}

float getCompAngle(float weightedAccAngle, float weightedGyroAngle, float prevCompAngle, float gyroWeight)
{
   return (float)((weightedGyroAngle + prevCompAngle) * gyroWeight + weightedAccAngle);
}