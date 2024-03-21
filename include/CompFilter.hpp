#include <stdint.h>

#ifndef COMP_FILTER_HPP
#define COMP_FILTER_HPP

float getAccAngle(uint16_t x_raw, uint16_t y_raw, uint16_t z_raw);
float getGyroAngle(uint16_t x_raw, uint16_t y_raw, uint16_t z_raw, double dt);
float getCompAngle(float weightedAccAngle, float weightedGyroAngle, float prevCompAngle, float gyroWeight);

#endif
