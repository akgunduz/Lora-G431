/*
 * sensor_board.h
 *
 *  Created on: Mar 14, 2020
 *      Author: akgun
 */

#ifndef INC_SENSOR_BOARD_H_
#define INC_SENSOR_BOARD_H_

#include "component.h"

#define I2C_TIMEOUT_MAX    0x1000

DrvStatusTypeDef Sensor_IO_Init(void);
uint8_t Sensor_IO_Write(void *handle, uint8_t Reg, uint8_t *pBuffer, uint16_t nBytesToWrite);
uint8_t Sensor_IO_Read(void *handle, uint8_t Reg, uint8_t *pBuffer, uint16_t nBytesToRead);

#endif /* INC_SENSOR_BOARD_H_ */
