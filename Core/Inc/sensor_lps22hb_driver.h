/*
 * sensor_driver.h
 *
 *  Created on: Mar 14, 2020
 *      Author: akgun
 */

#ifndef INC_SENSOR_LPS22HB_DRIVER_H_
#define INC_SENSOR_LPS22HB_DRIVER_H_

/* Includes ------------------------------------------------------------------*/
#include "component.h"

extern DrvStatusTypeDef BSP_LPS22HB_TEMPERATURE_Init(void **handle);
extern DrvStatusTypeDef BSP_LPS22HB_PRESSURE_Init(void **handle);

#endif /* INC_SENSOR_LPS22HB_DRIVER_H_ */
