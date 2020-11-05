/*
 * sensor_board.c
 *
 *  Created on: Mar 14, 2020
 *      Author: akgun
 */

#include "sensor_board.h"
#include "i2c.h"

DrvStatusTypeDef Sensor_IO_Init(void)
{

  //MX_I2C1_Init();

  return COMPONENT_OK;
}

static void Sensor_IO_Error()
{

  /* De-initialize the I2C communication bus */
  HAL_I2C_DeInit(&hi2c1);

  /* Re-Initiaize the I2C communication bus */
  MX_I2C1_Init();
}

uint8_t Sensor_IO_Write(void *handle, uint8_t Reg, uint8_t *pBuffer, uint16_t nBytesToWrite)
{
  DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;

  HAL_StatusTypeDef status = HAL_I2C_Mem_Write(&hi2c1, ctx->address, (uint16_t)Reg, I2C_MEMADD_SIZE_8BIT, pBuffer, nBytesToWrite, 5000);

  if (status != HAL_OK)
  {
    /* Execute user timeout callback */
	Sensor_IO_Error();
    return 1;
  }
  else
  {
    return 0;
  }
}

uint8_t Sensor_IO_Read(void *handle, uint8_t Reg, uint8_t *pBuffer, uint16_t nBytesToRead)
{
  DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;

  HAL_StatusTypeDef status = HAL_I2C_Mem_Read(&hi2c1, ctx->address, (uint16_t)Reg, I2C_MEMADD_SIZE_8BIT, pBuffer, nBytesToRead, 5000);
  if (status != HAL_OK)
  {
    /* Execute user timeout callback */
	Sensor_IO_Error();
    return 1;
  }
  else
  {
    return 0;
  }
}

