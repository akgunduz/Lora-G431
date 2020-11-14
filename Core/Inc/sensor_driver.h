/*
 * sensor_driver.h
 *
 *  Created on: Mar 14, 2020
 *      Author: akgun
 */

#ifndef INC_SENSOR_DRIVER_H_
#define INC_SENSOR_DRIVER_H_

/* Includes ------------------------------------------------------------------*/

#include "component.h"
#include "accelerometer.h"
#include "LSM303AGR_ACC.h"

DrvStatusTypeDef SensorDevicesInit();
DrvStatusTypeDef CollectSensorData();

DrvContextTypeDef ACCELERO_SensorHandle[ ACCELERO_SENSORS_MAX_NUM ];
ACCELERO_Data_t ACCELERO_Data[ ACCELERO_SENSORS_MAX_NUM ]; // Accelerometer - all.

extern uint16_t pressure;
extern int16_t temperature;
extern uint16_t humidity;
extern uint16_t magneto;
extern int16_t accelero[3];
extern uint32_t fftMaxAmpl;
extern uint32_t fftMaxFreq;
extern volatile uint8_t AXL_INT_received;




#endif /* INC_SENSOR_DRIVER_H_ */
