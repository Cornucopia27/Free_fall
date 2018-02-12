/*
 * free_fall.h
 *
 *  Created on: Feb 7, 2018
 *      Author: ALEX
 */

#ifndef FREE_FALL_H_
#define FREE_FALL_H_

void I2C_common_init();
void Red_Led_init();
void Pit_common_init();
void i2c_release_bus_delay();
void i2c_ReleaseBus();
void IMU_accelerometer_init();
void Accelerometer_measures();

#endif /* FREE_FALL_H_ */
