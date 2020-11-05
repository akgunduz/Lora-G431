/*
 * sensor_driver.h
 *
 *  Created on: Mar 14, 2020
 *      Author: akgun
 */

#ifndef INC_SENSOR_HTS221_DRIVER_H_
#define INC_SENSOR_HTS221_DRIVER_H_

/* Includes ------------------------------------------------------------------*/
#include "component.h"

extern DrvStatusTypeDef BSP_HTS221_HUMIDITY_Init(void **handle);
extern DrvStatusTypeDef BSP_HTS221_TEMPERATURE_Init(void **handle);

#endif /* INC_SENSOR_HTS221_DRIVER_H_ */
